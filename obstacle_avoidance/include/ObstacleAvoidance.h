#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include "ros/ros.h"

#include "geometry_msgs/PointStamped.h"
#include "cartesian_state_msgs/PoseTwist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"


using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class ObstacleAvoidance
{
protected:
  // ROS VARIABLES:
  // A handle to the node in ros
  ros::NodeHandle nh_;
  // Rate of the run loop
  ros::Rate loop_rate_;


  // Subscribers:

  // Subscriber for the platform state
  ros::Subscriber sub_platform_state_;
  // Subscriber for the desired platform velocity
  ros::Subscriber sub_platform_desired_twist_;
  // Subscriber for the front laser
  ros::Subscriber sub_laser_front_;
  // Subscriber for the rear laser
  ros::Subscriber sub_laser_rear_;

  // Publishers:

  // Publisher for the twist of the platform
  ros::Publisher pub_platform_cmd_;
  // Publisher for the obstacle vector
  ros::Publisher pub_obstacle_position_;

  // Input signal
  Vector6d platform_desired_twist_;

  // used as the input of the filter
  Vector6d twist_target_;


  // OUTPUT COMMANDS
  Vector6d platform_desired_twist_modified_;


  // STATE VARIABLES:
  // Platform state: position, orientation, and twist (in "platform base_link")
  Vector3d platform_real_position_;
  Quaterniond platform_real_orientation_;
  Vector6d platform_real_twist_;


  // Transform from base_link to world
  Matrix6d rotation_base_;




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
  tf::TransformListener listener_laser_front_;
  tf::TransformListener listener_laser_rear_;

  // Guards
  bool base_world_ready_;

  // Initialization
  void wait_for_transformations();



  void avoid_obstacles();

  // Obstacle avoidance
  Vector3d get_closest_point_on_platform(Vector3d obstacle);
  void update_obstacles();
  bool isObstacleMeasurement(Vector3d &measurement);

  // Callbacks
  void state_platform_callback(const nav_msgs::OdometryConstPtr msg);
  void laser_front_callback(const sensor_msgs::LaserScanPtr msg);
  void laser_rear_callback(const sensor_msgs::LaserScanPtr msg);

  // Util
  bool get_rotation_matrix(Matrix6d & rotation_matrix,
                           tf::TransformListener & listener,
                           std::string from_frame,  std::string to_frame);


  void send_commands_to_platform();

  void desired_velocity_callback(const geometry_msgs::TwistConstPtr msg);


public:
  ObstacleAvoidance(ros::NodeHandle &n, double frequency,
                    std::string topic_platform_state,
                    std::string topic_platform_desired_twist,
                    std::string topic_platform_command,
                    std::string laser_front_topic,
                    std::string laser_rear_topic,
                    double obs_distance_thres,
                    double self_detect_thres,
                    bool dont_avoid_front);
  void run();
};

#endif // OBSTACLE_AVOIDANCE_H

