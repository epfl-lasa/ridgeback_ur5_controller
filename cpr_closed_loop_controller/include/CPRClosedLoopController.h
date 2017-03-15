#ifndef CPRCLOSEDLOOPCONTROLLER_H
#define CPRCLOSEDLOOPCONTROLLER_H

#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "ur_modern_driver/ur_driver.h"

// CPRClosedLoopController class //
// A base class implementing the necesssary interfaces of a closed loop
// controller for the ridgeback + UR5 platform at the LASA lab. A
// CPRClosedLoopController is a while loop that:
// 1) Reads the robot state (arm+platform)
// 1) Sends a desired twist to the robot platform
// 2) Sends a desired twist to the robot arm
//
// Communication interfaces:
// - The arm is controlled directly through the UR driver ina  UDP connection.
// - The platform's driver is implemented in ROS control. The commands are sent
// through a ROS topic.
// - Note that the UR driver also provides a ROS control interface but for
// performance reasons we opted for a faster low level connection
//
//
//
// Usage example:
// ros::NodeHandle nh;
// double frequency = 100.0;
// CPRClosedLoopController cpr_closed_loop_controller(nh, frequency);
// cpr_closed_loop_controller.Run();

namespace CPR {

using namespace Eigen;

typedef Matrix<double,6,1> Vector6d;

class CPRClosedLoopController
{
protected:
  std::condition_variable rt_msg_cond_;
  std::condition_variable msg_cond_;

  // Robot state in joint space
  Vector6d joint_state_arm_;
  Vector6d joint_state_platform_;
  Vector6d joint_state_vel_arm_;
  Vector6d joint_state_twist_platform_;

  // Robot state in cartesian space
  Vector6d cart_state_arm_;
  Vector6d cart_state_platform_;
  Vector6d cart_state_twist_arm_;
  Vector6d cart_state_twist_platform_;

  // Acceleration values of the sensor at the tool
  Vector6d accel_ee_;

  // Force/torque at the end-effector
  Vector6d ft_cart_;

  // A handle to the node in ros
  ros::NodeHandle nh_;
  // Rate of the run loop
  ros::Rate loop_rate_;
  // Publisher for the twist of the platform
  // It will publish to the topic /cmd_vel
  ros::Publisher rb_twist_pub_;
  // It will publish to the topic /odom
  ros::Subscriber odom_sub_;
  // UR5 arm driver handle
  UrDriver ur5_arm_;

  // Twist message from the platform
  geometry_msgs::Twist platform_twist_;

  // A placeholder to initialize whatever is needed for the computeVelCmd right
  // before starting the loop
  void init();

  // Reads the state of the robot and updates its local copies
  void updateRobotState();

  // A placeholder to the main method of the controller
  void computeVelCmd(Vector6d & desired_twist_platform,
                     Vector6d & desired_joint_vel_arm);

  void odom_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

public:
  CPRClosedLoopController(ros::NodeHandle &n, double frequency,
                          std::string cmd_topic_platform,
                          std::string odm_topic_platform,
                          std::string host_arm,
                          int reverse_port_arm);
  void run();
};

}

#endif // CPRCLOSEDLOOPCONTROLLER_H
