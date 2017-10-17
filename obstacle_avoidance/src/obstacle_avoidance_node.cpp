#include "ros/ros.h"
#include "ObstacleAvoidance.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_avoidance_node");

  ros::NodeHandle nh;
  double frequency = 100.0;

  // Parameters
  std::string topic_platform_state;
  std::string topic_platform_desired_twist;
  std::string topic_platform_command;
  std::string topic_laser_front;
  std::string topic_laser_rear;

  double obs_distance_thres;
  double self_detect_thres;

  bool dont_avoid_front;



  // LOADING PARAMETERS FROM THE ROS SERVER

  if (!nh.getParam("topic_platform_state", topic_platform_state)) {
    ROS_ERROR("Couldn't retrieve the topic name for the state of the platform.");
    return -1;
  }

  if (!nh.getParam("topic_platform_desired_twist", topic_platform_desired_twist)) {
    ROS_ERROR("Couldn't retrieve the topic name for the desired twist for the platform.");
    return -1;
  }

  if (!nh.getParam("topic_platform_command", topic_platform_command)) {
    ROS_ERROR("Couldn't retrieve the topic name for commanding the platform.");
    return -1;
  }

  if (!nh.getParam("topic_laser_front", topic_laser_front)) {
    ROS_ERROR("Couldn't retrieve the topic name for the front laser. ");
    return -1;
  }

  if (!nh.getParam("topic_laser_rear", topic_laser_rear)) {
    ROS_ERROR("Couldn't retrieve the topic name for the rear laser. ");
    return -1;
  }

  if (!nh.getParam("obs_distance_thres", obs_distance_thres)) {
    ROS_ERROR("Couldn't retrieve the desired obs_distance_thres. ");
    return -1;
  }

  if (!nh.getParam("self_detect_thres", self_detect_thres)) {
    ROS_ERROR("Couldn't retrieve the desired self_detect_thres. ");
    return -1;
  }

  if (!nh.getParam("dont_avoid_front", dont_avoid_front)) {
    ROS_ERROR("Couldn't retrieve the dont_avoid_front flag. ");
    return -1;
  }


  // Constructing the controller
  ObstacleAvoidance obstacle_avoidance(
    nh,
    frequency,
    topic_platform_state,
    topic_platform_desired_twist,
    topic_platform_command,
    topic_laser_front,
    topic_laser_rear,
    obs_distance_thres,
    self_detect_thres,
    dont_avoid_front);

  // Running the controller
  obstacle_avoidance.run();

  return 0;
}
