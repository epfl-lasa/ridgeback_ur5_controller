#ifndef ADMITTANCECONTROLLER_H
#define ADMITTANCECONTROLLER_H

#include "ros/ros.h"

#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/Wrench.h"
#include "nav_msgs/Odometry.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "ur_modern_driver/ur_driver.h"

// AdmittanceController class //
// A base class implementing the necesssary interfaces of a closed loop
// controller for the ridgeback + UR5 platform at the LASA lab. A
// AdmittanceController is a while loop that:
// 1) Reads the robot state (arm+platform)
// 1) Sends a desired twist to the robot platform
// 2) Sends a desired twist to the robot arm
//
// Communication interfaces:
// - The desired twists are both send through ROS topics. 
// The low level controllers of both the arm and the platform are 
// implemented in ROS control. This node should be ideally a ROS 
// controller, but the current implementation of ROS control in 
// indigo requires writing a specific interface for this matter. 
// The kinetic version of ROS control enables controllers with 
// multiple interfaces (FT sensor, platform and arm), but a ROS
// control implementation of this controller remains future work.
//
// Usage example:
// ros::NodeHandle nh;
// double frequency = 100.0;
// AdmittanceController admittance_controller(nh, frequency);
// admittance_controller.Run();

using namespace Eigen;

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,7,1> Vector7d;

class AdmittanceController
{
protected:
  // Robot state in cartesian space
  Vector7d cart_state_arm_;
  Vector7d cart_state_platform_;
  Vector6d cart_twist_arm_;
  Vector6d cart_twist_platform_;

  // Force/torque at the end-effector
  Vector6d cart_ft_;

  // A handle to the node in ros
  ros::NodeHandle nh_;
  // Rate of the run loop
  ros::Rate loop_rate_;
  // Publisher for the twist of the platform
  ros::Publisher platform_pub_;
  // Subscriber for the platform state
  ros::Subscriber platform_sub_;
  // Publisher for the twist of arm endeffector
  ros::Publisher arm_pub_;
  // Subscriber for the arm state
  ros::Subscriber arm_sub_;
  // Subscriber for the wrench at the endeffector
  ros::Subscriber wrench_sub_;

  // A placeholder to initialize whatever is needed for the
  // computeVelCmd right before starting the loop
  void init();

  // Reads the state of the robot and updates its local copies
  void updateRobotState();

  // A placeholder to the main method of the controller
  void compute_admittance(Vector6d & desired_twist_platform,
                     Vector6d & desired_vel_arm, ros::Duration cycle_time);

  void state_platform_callback(const nav_msgs::OdometryConstPtr msg);
  void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);
  void wrench_callback(const geometry_msgs::WrenchConstPtr msg);

public:
  AdmittanceController(ros::NodeHandle &n, double frequency,
                       std::string cmd_topic_platform,
                       std::string state_topic_platform,
                       std::string cmd_topic_arm,
                       std::string state_topic_arm,
                       std::string wrench_topic);
  void run();
};

#endif // ADMITTANCECONTROLLER_H

