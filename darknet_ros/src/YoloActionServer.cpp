/*	YoloMultiActionServer Class definition	*/

#include "darknet_ros/YoloActionServer.hpp"

/*
	non-simple ActionServer
	circular buffer storing images (access to it is protected by lock)
	each time a goal request is received save image in the buffer, then start a yolo
	thread working on it
	if a rqst is rec when the buffer is full reply with a failure
*/


namespace darknet_ros{

YoloActionServer::YoloActionServer(ros::NodeHandle nh)
    : YoloCommonHead(nh), image_status(ImageStatus::NONE) {

  init();
  ROS_INFO("[YoloActionServer] Node started.");
}

YoloActionServer::~YoloActionServer() {
  yolo_clock.stop();
  if (yoloThread_.joinable())
    yoloThread_.join();
  checkForObjectsActionServer_->shutdown();
}

void YoloActionServer::init()
{
	//YoloCommonHead::init();
  
	// Action servers.
  std::string checkForObjectsActionName;
  nodeHandle_.param("actions/camera_reading/topic", checkForObjectsActionName, std::string("check_for_objects"));
  yolo_clock = nodeHandle_.createTimer(ros::Duration(yolo_clock_period), &YoloActionServer::yoloClockCB, this, false, false);

  checkForObjectsActionServer_.reset(new CheckForObjectsActionServer(nodeHandle_, checkForObjectsActionName, false));
  checkForObjectsActionServer_->registerGoalCallback(boost::bind(&YoloActionServer::checkForObjectsActionGoalCB, this));
  checkForObjectsActionServer_->registerPreemptCallback(boost::bind(&YoloActionServer::checkForObjectsActionPreemptCB, this));
  checkForObjectsActionServer_->start();

  yolo_clock.start();
}

void YoloActionServer::yoloClockCB(const ros::TimerEvent&)
{
  ImageStatus tempIS;
  {
    boost::shared_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
    tempIS = image_status;
  }
  if (tempIS != ImageStatus::REC && tempIS != ImageStatus::DONE)
    return;
  // - join the previous yolo thread if it's still running
  if (yoloThread_.joinable())
      yoloThread_.join();

  // if the current image has tag FETCH
  if (tempIS == ImageStatus::REC)
  {    
    // - launch the yolo thread
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      image_status = ImageStatus::FETCH;
    }
    yoloThread_ = std::thread(&YoloActionServer::yolo, this);
  }
  // if the current image has tag DONE
  else if (tempIS == ImageStatus::DONE)
  {
    // - send the image as a result
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      image_status = ImageStatus::NONE;
    }
    if (yoloThread_.joinable())
      yoloThread_.join();

    publish();
  }
  // - set the image status to NONE
}

/*  TODO: test how long it takes and if it's the case to pass it as a thread  */
void YoloActionServer::publish() {
  // Publish image.

  // Publish bounding boxes and detection result.
  int num = roiBoxes_[0].num;
  if (num > 0 && num <= 100)
  {
    for (int i = 0; i < num; i++) {
      for (int j = 0; j < numClasses_; j++) {
        if (roiBoxes_[i].Class == j) {
          rosBoxes_[j].push_back(roiBoxes_[i]);
          rosBoxCounter_[j]++;
        }
      }
    }
    for (int i = 0; i < numClasses_; i++) {
      if (rosBoxCounter_[i] > 0) {
        darknet_ros_msgs::BoundingBox boundingBox;

        for (int j = 0; j < rosBoxCounter_[i]; j++) {
          int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
          int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
          int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
          int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;

          boundingBox.Class = classLabels_[i];
          boundingBox.id = i;
          boundingBox.probability = rosBoxes_[i][j].prob;
          boundingBox.xmin = xmin;
          boundingBox.ymin = ymin;
          boundingBox.xmax = xmax;
          boundingBox.ymax = ymax;
          boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
        }
      }
    }
    boundingBoxesResults_.header.stamp = ros::Time::now();
    boundingBoxesResults_.header.frame_id = "detection";
    boundingBoxesResults_.image_header = current_head_;
  }
  else
  {
    darknet_ros_msgs::ObjectCount msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "detection";
    msg.count = 0;
  }
  ROS_DEBUG("[YoloObjectDetector] check for objects in image.");
  darknet_ros_msgs::CheckForObjectsResult objectsActionResult;
  objectsActionResult.id = actionId_;
  objectsActionResult.bounding_boxes = boundingBoxesResults_;
  checkForObjectsActionServer_->setSucceeded(objectsActionResult, "Send bounding boxes.");

  boundingBoxesResults_.bounding_boxes.clear();
  for (int i = 0; i < numClasses_; i++) {
    rosBoxes_[i].clear();
    rosBoxCounter_[i] = 0;
  }
  return;
}

void YoloActionServer::checkForObjectsActionGoalCB()
{
  ROS_INFO("[YoloActionServer] Received goal image."); 
  boost::shared_ptr<const darknet_ros_msgs::CheckForObjectsGoal> imageActionPtr = checkForObjectsActionServer_->acceptNewGoal();
  sensor_msgs::Image imageAction = imageActionPtr->image;
  actionId_ = imageActionPtr->id;

  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(imageAction, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    { 
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      camImageCopy_ = cam_image->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      image_status = ImageStatus::REC;
    }
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
  }
  return;
}
void YoloActionServer::checkForObjectsActionPreemptCB()
{
  ROS_DEBUG("[YoloObjectDetector] Preempt check for objects action.");
  checkForObjectsActionServer_->setPreempted();
}

void YoloActionServer::yolo(){
  ROS_DEBUG("yolo thread started");
  srand(3333333);

  demoTotal_ = sizeNetwork(net_);
  prediction_ = (float*)calloc(demoTotal_, sizeof(float));
  
  pred_val_ = (float*)calloc(demoTotal_, sizeof(float));

  layer l = net_->layers[net_->n - 1];
  roiBoxes_ = (darknet_ros::RosBox_*)calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_));

  {
    boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
    IplImageWithHeader_ imageAndHeader = getIplImageWithHeader();
    IplImage* ROS_img = imageAndHeader.image;
    current_img_ = ipl_to_image(ROS_img);
    current_head_ = imageAndHeader.header;
  }
  current_letter_ = letterbox_image(current_img_, net_->w, net_->h);
  ipl_ = cvCreateImage(cvSize(current_img_.w, current_img_.h), IPL_DEPTH_8U, current_img_.c);
  fetch();
  {
    boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
    image_status = ImageStatus::DETECT;
  }
  //display();
  detect();
  {
    boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
    image_status = ImageStatus::DONE;
  }
}

void YoloActionServer::fetch() {
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
    IplImageWithHeader_ imageAndHeader = getIplImageWithHeader();
    IplImage* ROS_img = imageAndHeader.image;
    ipl_into_image(ROS_img, current_img_);
    current_head_ = imageAndHeader.header;
  }
  rgbgr_image(current_img_);
  letterbox_image_into(current_img_, net_->w, net_->h, current_letter_);
  return ;
}

detection* YoloActionServer::singlePrediction(network* net, int* nboxes) {
  int count = 0;
  int i, j;
  fill_cpu(demoTotal_, 0, pred_val_, 1);
  axpy_cpu(demoTotal_, 1. / demoFrame_, prediction_, 1, pred_val_, 1);

  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      memcpy(l.output, pred_val_ + count, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
  detection *dets = get_network_boxes(net, current_img_.w, current_img_.h, demoThresh_, demoHier_, 0, 1, nboxes);
  return dets;
}

void YoloActionServer::rememberNetwork(network* net) {
  int i;
  int count = 0;
  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      memcpy(prediction_ + count, net->layers[i].output, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
}
void YoloActionServer::detect() {
  //running_ = 1;
  float nms = .4;

  layer l = net_->layers[net_->n - 1];
  float* X = current_letter_.data;


  float *prediction = network_predict(net_, X);

  rememberNetwork(net_);
  detection* dets = 0;
  int nboxes = 0;
  dets = singlePrediction(net_, &nboxes);

  if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

  /*
  draw_detections(display, dets, nboxes, demoThresh_, demoNamesChar, demoAlphabet_, demoClasses_);
  */
  // extract the bounding boxes and send them to ROS
  int i, j;
  int count = 0;
  for (i = 0; i < nboxes; ++i) {
    float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
    float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
    float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
    float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

    if (xmin < 0) xmin = 0;
    if (ymin < 0) ymin = 0;
    if (xmax > 1) xmax = 1;
    if (ymax > 1) ymax = 1;

    // iterate through possible boxes and collect the bounding boxes
    for (j = 0; j < demoClasses_; ++j) {
      if (dets[i].prob[j]) {

        float x_center = (xmin + xmax) / 2;
        float y_center = (ymin + ymax) / 2;
        float BoundingBox_width = xmax - xmin;
        float BoundingBox_height = ymax - ymin;

        // define bounding box
        // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
        if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) {
          roiBoxes_[count].x = x_center;
          roiBoxes_[count].y = y_center;
          roiBoxes_[count].w = BoundingBox_width;
          roiBoxes_[count].h = BoundingBox_height;
          roiBoxes_[count].Class = j;
          roiBoxes_[count].prob = dets[i].prob[j];
          count++;
        }
      }
    }
  }

  // create array to store found bounding boxes
  // if no object detected, make sure that ROS knows that num = 0
  if (count == 0) {
    roiBoxes_[0].num = 0;
  } else {
    roiBoxes_[0].num = count;
  }

  free_detections(dets, nboxes);
  //running_ = 0;
  
  
  return;
}


void YoloActionServer::display() {
  show_image_cv(current_img_, "YOLO V3", ipl_);
  int c = cv::waitKey(3000);
  if (c != -1) c = c % 256;
  if (c == 82) {
    demoThresh_ += .02;
  } else if (c == 84) {
    demoThresh_ -= .02;
    if (demoThresh_ <= .02) demoThresh_ = .02;
  } else if (c == 83) {
    demoHier_ += .02;
  } else if (c == 81) {
    demoHier_ -= .02;
    if (demoHier_ <= .0) demoHier_ = .0;
  }
  return;
}

} //namespace darknet_ros
