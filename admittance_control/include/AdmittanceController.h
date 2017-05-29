#ifndef ADMITTANCECONTROLLER_H
#define ADMITTANCECONTROLLER_H

#include "ros/ros.h"

#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"


// AdmittanceController class //
// A simple class implementing an admittance
// controller for the ridgeback + UR5 platform at the LASA lab. An
// AdmittanceController is a while loop that:
// 1) Reads the robot state (arm+platform)
// 1) Sends a desired twist to the robot platform
// 2) Sends a desired twist to the robot arm
//
// Communication interfaces:
// - The desired twists are both sent through ROS topics.
// The low level controllers of both the arm and the platform are 
// implemented in ROS control.
// NOTE: This node should be ideally a ROS
// controller, but the current implementation of ROS control in 
// indigo requires writing a specific interface for this matter. 
// The kinetic version of ROS control enables controllers with 
// multiple interfaces (FT sensor, platform and arm), but a proper
// ROS control implementation of this controller remains future work.
//
// USAGE EXAMPLE;
// ros::NodeHandle nh;
// double frequency = 1000.0;
// std::string state_topic_arm, cmd_topic_arm, topic_arm_twist_world,
//   topic_wrench_u_e, topic_wrench_u_c, cmd_topic_platform,
//   state_topic_platform, wrench_topic, wrench_control_topic,
//   laser_front_topic, laser_rear_topic;
// std::vector<double> M_p, M_a, D, D_p, D_a, K, d_e;
// double wrench_filter_factor, force_dead_zone_thres,
//    torque_dead_zone_thres, obs_distance_thres, self_detect_thres;
//
//
// // Fill in values
// ...
//
//
// AdmittanceController admittance_controller(nh, frequency,
//                                           cmd_topic_platform,
//                                           state_topic_platform,
//                                           cmd_topic_arm,
//                                           topic_arm_twist_world,
//                                           topic_wrench_u_e,
//                                           topic_wrench_u_c,
//                                           state_topic_arm,
//                                           wrench_topic,
//                                           wrench_control_topic,
//                                           laser_front_topic,
//                                           laser_rear_topic,
//                                           M_p, M_a, D, D_p, D_a, K, d_e,
//                                           wrench_filter_factor,
//                                           force_dead_zone_thres,
//                                           torque_dead_zone_thres,
//                                           obs_distance_thres,
//                                           self_detect_thres);
// admittance_controller.run();

using namespace Eigen;

typedef Matrix<double,7,1> Vector7d;
typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;

class AdmittanceController
{
protected:
  // ROS VARIABLES:
  // A handle to the node in ros
  ros::NodeHandle nh_;
  // Rate of the run loop
  ros::Rate loop_rate_;

  // Publishers and subscribers:
  // Publisher for the twist of the platform
  ros::Publisher platform_pub_;
  // Subscriber for the platform state
  ros::Subscriber platform_sub_;
  // Publisher for the twist of arm endeffector
  ros::Publisher arm_pub_;
  // Publisher for the pose of arm endeffector in the world frame
  ros::Publisher arm_pose_pub_world_;
  // Publisher for the twist of arm endeffector in the world frame
  ros::Publisher arm_twist_pub_world_;
  // Publisher for the obstacle vector
  ros::Publisher obs_pub_;
  // Subscriber for the arm state
  ros::Subscriber arm_sub_;
  // Subscriber for the ft sensor at the endeffector
  ros::Subscriber wrench_sub_;
  // Subscriber for the ft sensor at the endeffector
  ros::Subscriber wrench_control_sub_;

  // Subscriber for the admittance control forces in the ur5_arm_base_link, for rviz
  ros::Publisher wrench_pub_u_e_;
  ros::Publisher wrench_pub_u_c_;

  // Subscribers for the lasers
  ros::Subscriber laser_front_sub_;
  ros::Subscriber laser_rear_sub_;

  // For publishing ee state in world frame
  geometry_msgs::Twist twist_arm_world_frame_;
  geometry_msgs::Pose pose_ee_world_frame_;

  // STATE VARIABLES:
  // x_p_position_, x_p_orientation_, x_dot_p_, x_ddot_p ->
  //                             Platform state and time derivatives
  //                             (in platform base_link)
  // x_a_position_, x_a_orientation_, x_dot_a_, x_ddot_a ->
  //                             Arm state and time derivatives
  //                             (in ur5_arm_base_link frame)
  // u_e_ -> external wrench (force/torque sensor)
  //         (in ur5_arm_base_link frame)
  // u_c_ -> control wrench (from yout goal-oriented controller)
  //         (in ur5_arm_base_link frame)
  Vector3d x_a_position_, x_p_position_;
  Quaterniond x_a_orientation_, x_p_orientation_;
  Vector6d x_dot_a_;
  Vector6d x_dot_p_;
  Vector6d u_e_, u_c_;

  Matrix6d rotation_base_; // Transform from base_link to
                                         // ur5_arm_base_link
  Matrix6d kin_constraints_; // Derivative of kinematic constraints
                             // between the arm and the platform

  // ADMITTANCE PARAMETERS:
  // M_p_, M_a_ -> Desired mass of platform/arm
  // D_ -> Desired damping of the coupling
  // K_ -> Desired Stiffness of the coupling
  // D_p_, D_a_ -> Desired damping of platform/arm
  // d_e_position_ -> equilibrium position of the coupling spring
  // d_e_orientation -> equilibrium orientation of the coupling spring
  Matrix6d M_p_, M_a_, D_, D_p_, D_a_, K_;
  Vector3d d_e_position_;
  Quaterniond d_e_orientation_;


  // RENDERED DYNAMICS:
  // x_ddot_p = M_p_^{-1}(+D_*(x_dot_a_)
  //              - D_p_ x_dot_p_ + K_(error))
  // x_ddot_a = M_a_^{-1}(-D_*(x_dot_a_)
  //              - D_a_ x_dot_a_ - K_(error)) + u_e_

  // OBSTACLE AVOIDANCE:
  // Vector defining the position of an obstacle in the base_link
  // If no obstacle is detected then it is set to 0
  Eigen::Vector3d obs_vector_;

  // The robot assumes that there is one single obstacle and all measurements
  // between self_detect_thres_ and obs_distance_thres_ are considered parts of the
  // same obstacle
  // Maximum distance threshold to consider an obstacle
  double obs_distance_thres_;
  // Threshold starting at the end of the platform to consider an obstacle
  // to avoid self detections like cables
  double self_detect_thres_;
  // Flag indicating if the front of the platform should be avoided too or not.
  // For human-robot interaction settings this might be desireable if the robot
  // is always carrying something with the robot.
  bool dont_avoid_front_;

  // Point cloud from the laser scans
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud laser_front_cloud_;
  sensor_msgs::PointCloud laser_rear_cloud_;

  // TF:
  // Listeners
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;
  tf::TransformListener listener_laser_front_;
  tf::TransformListener listener_laser_rear_;

  // Guards
  bool ft_arm_ready_;
  bool arm_world_ready_;
  bool base_world_ready_;
  bool world_arm_ready_;

  // FT FILTER:
  // Parameters for the noisy wrench
  double wrench_filter_factor_;
  double force_dead_zone_thres_;
  double torque_dead_zone_thres_;

  // Initialization
  void init_TF();

  // Control
  void compute_admittance(Vector6d & desired_twist_platform,
                     Vector6d & desired_vel_arm, ros::Duration cycle_time);
  void avoid_obstacles(Vector6d &desired_twist_platform);

  // Obstacle avoidance
  Vector3d get_closest_point_on_platform(Vector3d obstacle);
  void update_obstacles();
  bool isObstacleMeasurement(Vector3d &measurement);

  // Callbacks
  void state_platform_callback(const nav_msgs::OdometryConstPtr msg);
  void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);
  void wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg);
  void wrench_control_callback(const geometry_msgs::WrenchStampedConstPtr msg);
  void laser_front_callback(const sensor_msgs::LaserScanPtr msg);
  void laser_rear_callback(const sensor_msgs::LaserScanPtr msg);

  // Util
  bool get_rotation_matrix(Matrix6d & rotation_matrix,
                           tf::TransformListener & listener,
                           std::string from_frame,  std::string to_frame);
  void get_arm_twist_world(geometry_msgs::Twist & ee_twist_world,
                           tf::TransformListener & listener);
  void get_ee_pose_world(geometry_msgs::Pose & ee_pose_world,
                           tf::TransformListener & listener);

public:
  AdmittanceController(ros::NodeHandle &n, double frequency,
                       std::string cmd_topic_platform,
                       std::string state_topic_platform,
                       std::string cmd_topic_arm,
                       std::string topic_arm_pose_world,
                       std::string topic_arm_twist_world,
                       std::string topic_wrench_u_e,
                       std::string topic_wrench_u_c,
                       std::string state_topic_arm,
                       std::string wrench_topic,
                       std::string wrench_control_topic,
                       std::string laser_front_topic,
                       std::string laser_rear_topic,
                       std::vector<double> M_p,
                       std::vector<double> M_a,
                       std::vector<double> D,
                       std::vector<double> D_p,
                       std::vector<double> D_a,
                       std::vector<double> K,
                       std::vector<double> d_e,
                       double wrench_filter_factor,
                       double force_dead_zone_thres,
                       double torque_dead_zone_thres,
                       double obs_distance_thres,
                       double self_detect_thres,
                       bool dont_avoid_front);
  void run();
};

#endif // ADMITTANCECONTROLLER_H

