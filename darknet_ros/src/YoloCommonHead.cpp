#include "darknet_ros/YoloCommonHead.hpp"


#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace darknet_ros{

YoloCommonHead::YoloCommonHead(ros::NodeHandle nh)
: nodeHandle_(nh),  numClasses_(0), classLabels_(0), rosBoxes_(0), rosBoxCounter_(0)
{
	  // Read parameters from config file.
  if (!readParameters()) {
    ros::requestShutdown();
  }

  init();
}

YoloCommonHead::~YoloCommonHead(){}

void YoloCommonHead::init()
{
  ROS_INFO("Common init");
  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // Threshold of object detection.
  float thresh;
  nodeHandle_.param("yolo_model/threshold/value", thresh, (float)0.3);

  // Path to weights file.
  nodeHandle_.param("yolo_model/weight_file/name", weightsModel, std::string("yolov2-tiny.weights"));
  nodeHandle_.param("weights_path", weightsPath, std::string("/default"));
  weightsPath += "/" + weightsModel;
  weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  nodeHandle_.param("yolo_model/config_file/name", configModel, std::string("yolov2-tiny.cfg"));
  nodeHandle_.param("config_path", configPath, std::string("/default"));
  configPath += "/" + configModel;
  cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_;
  dataPath += "/data";
  data = new char[dataPath.length() + 1];
  strcpy(data, dataPath.c_str());

  // Get classes.
  //detectionNames = (char**)realloc((void*)detectionNames, (numClasses_ + 1) * sizeof(char*));
  ROS_INFO("-- Pre reserve --");
  detectionNames.reserve(numClasses_ + 1);

  for (int i = 0; i < numClasses_; i++) {
    //detectionNames[i] = new char[classLabels_[i].length() + 1];
    //strcpy(detectionNames[i], classLabels_[i].c_str());
    detectionNames.push_back(std::string(classLabels_[i]));
  }
  ROS_INFO("-- Post strcpy --");

  // Load network.
  setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_, 0, 0, 1, 0.5, 0, 0, 0, 0);
  //ROS_INFO("-- Post network --");

}


bool YoloCommonHead::readParameters() {
  // Set vector sizes.
  nodeHandle_.param("yolo_model/detection_classes/names", classLabels_, std::vector<std::string>(0));
  numClasses_ = classLabels_.size();
  rosBoxes_ = std::vector<std::vector<RosBox_> >(numClasses_);
  rosBoxCounter_ = std::vector<int>(numClasses_);

  return true;
}


void YoloCommonHead::setupNetwork(char* cfgfile, char* weightfile, char* datafile, float thresh, std::vector<std::string> names, int classes, int delay,
                                      char* prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen) {
  demoPrefix_ = prefix;
  demoDelay_ = delay;
  demoFrame_ = avg_frames;
  image** alphabet = load_alphabet_with_file(datafile);
  demoNames_ = names;
  demoAlphabet_ = alphabet;
  demoClasses_ = classes;
  demoThresh_ = thresh;
  demoHier_ = hier;
  fullScreen_ = fullscreen;
  printf("YOLO V3\n");
  net_ = load_network(cfgfile, weightfile, 0);
  set_batch_network(net_, 1);
}

int YoloCommonHead::sizeNetwork(network* net) {
  int i;
  int count = 0;
  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      count += l.outputs;
    }
  }
  return count;
}

IplImageWithHeader_ YoloCommonHead::getIplImageWithHeader() {
  IplImage* ROS_img = new IplImage(camImageCopy_);
  IplImageWithHeader_ header = {.image = ROS_img, .header = imageHeader_};
  return header;
}


}