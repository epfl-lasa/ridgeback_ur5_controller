#!/usr/bin/env python

import rospy
import sys
import subprocess

from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

SET_MODEL_CONFIGURATION = "/gazebo/set_model_configuration"
SWITCH_CONTROLLER = "/controller_manager/switch_controller"

def set_initial_pose_gazebo():
    set_model_config_client = rospy.ServiceProxy(SET_MODEL_CONFIGURATION, SetModelConfiguration)
    req = SetModelConfigurationRequest()
    req.model_name = "ridgeback"
    req.urdf_param_name = "robot_description"
    req.joint_names = rospy.get_param("/initial_pose/joint_names")
    req.joint_positions = rospy.get_param("/initial_pose/joint_values")
    rospy.wait_for_service(SET_MODEL_CONFIGURATION)
    res = set_model_config_client(req)

def pause_gazebo():
    subprocess.check_call("rosservice call /gazebo/pause_physics", shell=True)

def unpause_gazebo():
    subprocess.check_call("rosservice call /gazebo/unpause_physics", shell=True)

def start_ros_controllers():
    start_controllers_client = rospy.ServiceProxy(SWITCH_CONTROLLER, SwitchController)
    req = SwitchControllerRequest()
    req.start_controllers.append("ur5_cartesian_velocity_controller_sim")
    req.start_controllers.append("joint_state_controller")
    req.start_controllers.append("ridgeback_joint_publisher")
    req.start_controllers.append("ridgeback_velocity_controller")
    req.strictness = req.BEST_EFFORT
    rospy.wait_for_service(SWITCH_CONTROLLER)
    res = start_controllers_client(req)
 
if __name__ == "__main__":
    pause_gazebo()
    set_initial_pose_gazebo()
    unpause_gazebo()
    start_ros_controllers()
