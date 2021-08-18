/*
 * YoloCommonHead.hpp
 *
 *  Created on: August 14, 2021
 *      Author: Marco Gabriele Fedozzi
 *   Institute: University of Genoa, MSc Robotics Engineering
 */

#pragma once
#ifndef YOLO_COMMON_HEAD_H
#define YOLO_COMMON_HEAD_H
// c++
#include <pthread.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

// ROS

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

// OpenCv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h>

// Darknet.
#ifdef GPU
#include "cublas_v2.h"
#include "cuda_runtime.h"
#include "curand.h"
#endif

extern "C" {
#include <sys/time.h>
#include "box.h"
#include "cost_layer.h"
#include "darknet_ros/image_interface.h"
#include "detection_layer.h"
#include "network.h"
#include "parser.h"
#include "region_layer.h"
#include "utils.h"
}




extern "C" void ipl_into_image(IplImage* src, image im);
extern "C" image ipl_to_image(IplImage* src);
extern "C" void show_image_cv(image p, const char* name, IplImage* disp);
//! Bounding box of the detected object.
namespace darknet_ros{

typedef struct {
  float x, y, w, h, prob;
  int num, Class;
} RosBox_;

typedef struct {
  IplImage* image;
  std_msgs::Header header;
} IplImageWithHeader_;



class YoloCommonHead
{
  public:
    explicit YoloCommonHead(ros::NodeHandle nh);
    /*!
    * Destructor.
    */
    ~YoloCommonHead();

  protected:
  // ----- METHODS -----
    /*!
    * Reads and verifies the ROS parameters.
    * @return true if successful.
    */
    bool readParameters(); //-> common first part for both classes, then expanded

    /*!
    * Initialize the ROS connections.
    */
    void init(); 

    int sizeNetwork(network* net);

    virtual void rememberNetwork(network* net) = 0;
    
    void setupNetwork(char* cfgfile, char* weightfile, char* datafile, float thresh, std::vector<std::string> names, int classes, int delay, char* prefix,
                      int avg_frames, float hier, int w, int h, int frames, int fullscreen);

    virtual void yolo() = 0;

    bool getImageStatus(void);

    bool isNodeRunning(void);

    IplImageWithHeader_ getIplImageWithHeader();

    // ----- MEMBERS -----

		//! ROS node handle.
		ros::NodeHandle nodeHandle_;

		//! Class labels.
		int numClasses_;
		std::vector<std::string> classLabels_;
 
    //! Detected objects.
    std::vector<std::vector<RosBox_> > rosBoxes_;
    std::vector<int> rosBoxCounter_;
    darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;

    //! Camera related parameters.
    int frameWidth_;
    int frameHeight_;
    
    // Darknet.
    std::vector<std::string> demoNames_;
    image **demoAlphabet_;
    int demoClasses_;

    network* net_;
    
    IplImage* ipl_;
    float fps_ = 0;
    float demoThresh_ = 0;
    float demoHier_ = .5;
    int demoDelay_;
    int demoFrame_;
    int running_ = 0;
    
    RosBox_* roiBoxes_;

		// + Basically all the params needed by the network
        
    char* cfg;
    char* weights;
    char* data;
    std::vector<std::string> detectionNames;


    int fullScreen_;
    char* demoPrefix_;
    
    int demoTotal_ = 0;
    //! Imagelocks
    std_msgs::Header imageHeader_;
    cv::Mat camImageCopy_;
    boost::shared_mutex mutexImageCallback_;

    // Yolo running on thread.
    std::thread yoloThread_;
};
}
#endif // YOLO_COMMON_HEAD_H