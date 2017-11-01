#ifndef __OBJECTTRACKER__
#define __OBJECTTRACKER__

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <iostream>


class ObjectTracker {


private:

	// ROS variables
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	tf::TransformListener listener_;
	tf::TransformBroadcaster brodcaster_;

	std::string frame_platform_base_;
	std::string frame_arm_base_;
	std::string frame_robot_calibration_;
	std::string frame_robot_endeffector_;
	std::string frame_robot_object_;
	std::string frame_mocap_calibration_;
	std::string frame_mocap_endeffector_;
	std::string frame_mocap_object_;

	tf::Quaternion quat_offset_;

	tf::StampedTransform transform_static_base_;

	ros::Duration time_diff_;


public:
	ObjectTracker(ros::NodeHandle &n,
	              double frequency,
	              std::string frame_platform_base,
	              std::string frame_arm_base,
	              std::string frame_robot_calibration,
	              std::string frame_robot_endeffector,
	              std::string frame_robot_object,
	              std::string frame_mocap_calibration,
	              std::string frame_mocap_endeffector,
	              std::string frame_mocap_object);


	void Run();


private:



};


#endif //__OBJECTTRACKER__