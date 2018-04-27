#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>

#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

using namespace std;



class EE_STATE_SPLITTER {

private:
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;

	ros::Subscriber sub_ee_state_;
	ros::Publisher pub_ee_state_pose_;
	ros::Publisher pub_ee_state_twist_;

public:
	EE_STATE_SPLITTER(ros::NodeHandle &n, double frequency)
		: nh_(n), loop_rate_(frequency) {

		ROS_INFO_STREAM("The splitter node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
	}

	void Initialize() {

		sub_ee_state_ = nh_.subscribe("/ur5/ur5_cartesian_velocity_controller_sim/ee_state" ,
		                              1000, &EE_STATE_SPLITTER::state_arm_callback, this);

		pub_ee_state_pose_ = nh_.advertise<geometry_msgs::Pose>(
		                     "/ur5/ur5_cartesian_velocity_controller_sim/ee_state_pose", 5);

		pub_ee_state_twist_ = nh_.advertise<geometry_msgs::Twist>(
		                      "/ur5/ur5_cartesian_velocity_controller_sim/ee_state_twist", 5);


	}


	void Run() {

		while (nh_.ok()) {

			ros::spinOnce();
			loop_rate_.sleep();
		}

	}

	void state_arm_callback(
	    const cartesian_state_msgs::PoseTwistConstPtr msg) {

		pub_ee_state_pose_.publish(msg->pose);
		pub_ee_state_twist_.publish(msg->twist);
	}

};






int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "demonstration_recorder");



	ros::NodeHandle nh;
	double frequency = 125.0;
	EE_STATE_SPLITTER splitter(nh, frequency);

	splitter.Initialize();

	splitter.Run();

	return 0;
}
