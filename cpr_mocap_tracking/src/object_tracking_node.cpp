#include "ros/ros.h"
#include "ObjectTracker.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_tracking_node");

  ros::NodeHandle nh;
  double frequency = 100.0;


  // Parameters
  std::string frame_platform_base;
  std::string frame_arm_base;

  std::string frame_robot_calibration;
  std::string frame_robot_endeffector;
  std::string frame_robot_object;


  std::string frame_mocap_calibration;
  std::string frame_mocap_endeffector;
  std::string frame_mocap_object;


  if (!nh.getParam("frame_platform_base", frame_platform_base))   {
    ROS_ERROR("Couldn't retrieve the frame id for the base of the platform (eg. base_link).");
    return -1;
  }

  if (!nh.getParam("frame_arm_base", frame_arm_base))   {
    ROS_ERROR("Couldn't retrieve the frame id for the base of the arm (eg. ur5_arm_base).");
    return -1;
  }

  if (!nh.getParam("frame_robot_calibration", frame_robot_calibration))   {
    ROS_ERROR("Couldn't retrieve the frame id for the calibration tf (eg. cpr_mid_pc).");
    return -1;
  }

  if (!nh.getParam("frame_robot_endeffector", frame_robot_endeffector))   {
    ROS_ERROR("Couldn't retrieve the frame id for the end-effector tf (eg. robotiq_force_torque_frame_id).");
    return -1;
  }

  if (!nh.getParam("frame_robot_object", frame_robot_object))   {
    ROS_ERROR("Couldn't retrieve the frame id for the object to be published on (eg. object).");
    return -1;
  }

  if (!nh.getParam("frame_mocap_calibration", frame_mocap_calibration))   {
    ROS_ERROR("Couldn't retrieve the mocap tf for the calibration tf (eg. mocap_mid_pc).");
    return -1;
  }

  if (!nh.getParam("frame_mocap_endeffector", frame_mocap_endeffector))   {
    ROS_ERROR("Couldn't retrieve the mocap tf for the end-effector tf (eg. mocap_palm).");
    return -1;
  }


  if (!nh.getParam("frame_mocap_object", frame_mocap_object))   {
    ROS_ERROR("Couldn't retrieve the MOCAP id for the object to be tracked (eg. mocap_object).");
    return -1;
  }


  ObjectTracker object_tracker(nh, frequency,
                               frame_platform_base,
                               frame_arm_base,
                               frame_robot_calibration,
                               frame_robot_endeffector,
                               frame_robot_object,
                               frame_mocap_calibration,
                               frame_mocap_endeffector,
                               frame_mocap_object);

  object_tracker.Run();


  return 0;
}