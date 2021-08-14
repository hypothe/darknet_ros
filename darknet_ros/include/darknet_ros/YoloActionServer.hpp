/*	YoloMultiActionServer Class declaration	*/

/*
	libs
*/

/*
 * YoloActionServer.hpp
 *
 *  Created on: August 14, 2021
 *      Author: Marco Gabriele Fedozzi
 *   Institute: University of Genoa, MSc Robotics Engineering
 */

/*
	The idea:
	- Simple Action Server, only serve one image at a time
	- the client will send multiple images, but each one as a separate goal request
	- to deal with possible delays it could either:
		- try to send images with a fixed rate, discarding images taken when the previous
			one hasn't returned yet
		- same, but instead of discarding that frame it could wait (imagining it's moving its head-camera
			up and down in the meantime) until a result is received from the action server.
	- this is less computationally efficient than doing a non-simple AS with threaded yolo for each
		image, but it's easier to deal with.
	- notice that, if the communicatoion overhead ends up being too much, we could opt for a nodelet
		implementation.
*/

#pragma once

#include "darknet_ros/YoloCommonHead.hpp"
#include <actionlib/server/simple_action_server.h>

namespace darknet_ros {

class YoloActionServer : public YoloCommonHead
{
	public:
		// constructor -> see example in YoloObjectDetector
		// destructor -> remember to join all active threads (forcing it with std::terminate if necessary)
		YoloActionServer(ros::NodeHandle nh);
		~YoloActionServer();

	private:
		//! Using.
		using CheckForObjectsActionServer = actionlib::SimpleActionServer<darknet_ros_msgs::CheckForObjectsAction>;
		using CheckForObjectsActionServerPtr = std::shared_ptr<CheckForObjectsActionServer>;
		// ---- METHODS ----
		void init();

		void yoloClockCB(const ros::TimerEvent&);

		/*!
		* Check for objects action goal callback.
		*/
		void checkForObjectsActionGoalCB();

		/*!
		* Check for objects action preempt callback.
		*/
		void checkForObjectsActionPreemptCB();

		/*!
		* Check if a preempt for the check for objects action has been requested.
		* @return false if preempt has been requested or inactive.
		*/
		bool isCheckingForObjects() const;

		void yolo();

		void fetch();
		detection *singlePrediction(network *net, int *nboxes);
		void rememberNetwork(network *net);
		void detect();

		void publish();

		// ---- MEMBERS ----

	 	//! Check for objects action server.
  	CheckForObjectsActionServerPtr checkForObjectsActionServer_;

		// redefine inherited
		// ...
		
		// ActionGoal Callback
		// ActionPreempt Callback
		// yolo (to run in a thread)
		// Clock Callback


		const uint8_t IMG_BUF_LEN = 16;
		const float yolo_clock_period = 0.02; // 50Hz
		ros::Timer yolo_clock;
		// a clock (which checks, for all images, if they've been eval, in which case it joins their thread)

		int actionId_;
		boost::shared_mutex mutexActionStatus_;

		enum class ImageStatus
		{
			FETCH,
			DETECT,
			DONE,
			NONE
		};

		ImageStatus image_status;
		boost::shared_mutex mutexImageStatus_;

		
    image current_img_;
    image current_letter_;
		std_msgs::Header current_head_;
		float *prediction_;
		float* pred_val_;

};
}
//#endif // YOLOMULTIACTIONSERVER_H