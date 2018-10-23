#ifndef ADMITTANCECONTROLLER_H
#define ADMITTANCECONTROLLER_H

#include "ros/ros.h"

#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/PoseStamped.h"

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

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

typedef Matrix<double, 9, 9> Matrix9d;
typedef Matrix<double, 9, 6> Matrix9_6d;
typedef Matrix<double, 6, 9> Matrix6_9d;
typedef Matrix<double, 9, 1> Vector9d;

class AdmittanceController
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
  // Subscriber for the arm state
  ros::Subscriber sub_arm_state_;
  // Subscriber for the ft sensor at the endeffector
  ros::Subscriber sub_wrench_external_;
  // Subscriber for the ft sensor at the endeffector
  ros::Subscriber sub_wrench_control_;
  // Subscriber for the offset of the attractor
  ros::Subscriber sub_equilibrium_desired_;

  // Subscriber for the grasped object pose in world defined by mocap
  ros::Subscriber sub_object_ee_pose_mocap_world;

  // Subscriber for the mid pc pose in world defined by mocap
  ros::Subscriber sub_mid_pc_pose_mocap_world;

  


  // Publishers:

  // Publisher for the twist of the platform
  ros::Publisher pub_platform_cmd_;
  // Publisher for the twist of arm endeffector
  ros::Publisher pub_arm_cmd_;
  // Publisher for the pose of arm endeffector in the world frame
  ros::Publisher pub_ee_pose_world_;
  // Publisher for the twist of arm endeffector in the world frame
  ros::Publisher pub_ee_twist_world_;
  // Publisher for the external wrench specified in the world frame
  ros::Publisher pub_wrench_external_;
  // Publisher for the control wrench specified in the world frame
  ros::Publisher pub_wrench_control_;
  // Publisher to visualize the real equilibrium used by admittance.
  ros::Publisher pub_equilibrium_real_;


  // INPUT SIGNAL
  // external wrench (force/torque sensor) in "robotiq_force_torque_frame_id" frame
  Vector6d wrench_external_;
  // control wrench (from any controller) expected to be in "ur5_arm_base_link" frame
  Vector6d wrench_control_;


  // FORCE/TORQUE-SENSOR FILTER:
  // Parameters for the noisy wrench
  double wrench_filter_factor_;
  double force_dead_zone_thres_;
  double torque_dead_zone_thres_;


  // ADMITTANCE PARAMETERS:
  // M_p_, M_a_ -> Desired mass of platform/arm
  // D_ -> Desired damping of the coupling
  // D_p_, D_a_ -> Desired damping of platform/arm
  // K_ -> Desired Stiffness of the coupling
  Matrix6d M_p_, M_a_, D_, D_p_, D_a_, K_;
  // equilibrium position of the coupling spring
  Vector3d equilibrium_position_;
  Vector3d equilibrium_position_seen_by_platform;
  // equilibrium orientation of the coupling spring
  Quaterniond equilibrium_orientation_;

  // receiving a new equilibrium from a topic
  Vector3d equilibrium_new_;



  // OUTPUT COMMANDS
  // the desired velcoities computed by the admittance control
  Vector6d arm_desired_twist_;
  Vector6d platform_desired_twist_;
  Vector6d desired_platform_velocity_;

  // limiting the workspace of the arm
  Vector6d workspace_limits_;
  double arm_max_vel_;
  double arm_max_acc_;
  double platform_max_vel_;
  double platform_max_acc_;


  // STATE VARIABLES:
  // Platform state: position, orientation, and twist (in "platform base_link")
  Vector3d platform_real_position_;
  Quaterniond platform_real_orientation_;
  Vector6d platform_real_twist_;

  // Arm state: position, orientation, and twist (in "ur5_arm_base_link")
  Vector3d arm_real_position_;
  Quaterniond arm_real_orientation_;
  Vector6d arm_real_twist_;

  // End-effector state: pose and twist (in "world" frame)
  Vector7d ee_pose_world_;
  Vector6d ee_twist_world_;


  // Transform from base_link to world
  Matrix6d rotation_base_;
  // Derivative of kinematic constraints between the arm and the platform
  Matrix6d kin_constraints_;

  // TF:
  // Listeners
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;

  // Guards
  bool ft_arm_ready_;
  bool arm_world_ready_;
  bool base_world_ready_;
  bool world_arm_ready_;

  // ============================================================================================================

  // parameters for the apparent dynamics of the arm-platform system

  // ===============================================================

  Matrix9d M_vap, D_vap;       // virtual impedance
  Matrix9_6d K_vap;
  //Matrix6d  M_obj_, D_obj_, K_obj;  // object impedance
  Matrix6_9d v_Jacobian_arm;
  Matrix6_9d v_Jacobian_platform;
  //
  MatrixXd Inverse_M_vap;  // inverse of the virtual inertia matrix

// 
  Vector9d v_bias_forces;
  Vector9d v_arm_platform_desired_acceleration;
  Vector9d v_arm_platform_desired_twist;
  Vector9d v_arm_platform_control_wrench;

// object impedance Gain Matrices
  Matrix6d M_obj_imp_;                // virtual intertia
  Matrix6d D_obj_imp_;                // virtual damping
  Matrix6d K_obj_imp_;                // virtual stiffness
  // robot Posture Gain matrices
  Vector9d K_posture_;               // stiffness gain 
  Vector9d D_posture_;               // damping gain

  // Internal Wrench Estimator
  Matrix6d K_estimator_;               // gaim matrix estimator

  //Vector6d measured_Wrench;           // measured wrench at the end-effector

  // object dynamics variables view from the mobile manipulator
  Matrix6d obj_inertiaMx;
  Vector6d obj_bias_forces;
  Matrix6d distributed_obj_inertiaMx;
  Vector6d distributed_obj_bias_forces; 

  // reference object trajectory
  // ---------------------------
  Eigen::Vector3d obj_reference_position_world;
  Eigen::Quaterniond obj_reference_orientation_world;
  Vector6d obj_reference_twist_world;
  Vector6d obj_reference_acceleration_world;

  // object real state
  // -----------------
  Eigen::Vector3d obj_real_position_world;           //  from the object state callback function
  Eigen::Quaterniond obj_real_orientation_world;     //  from the object state callback function
  // velocity twist
  Vector6d obj_real_twist_world;
  //
  // rotation matrix of the object wrt to the world frame
  Eigen::Matrix3d Rot_real_obj_world_previous;
  // position vector of the object in the world frame
  Eigen::Vector3d obj_real_position_world_previous;

  // Object task Jacobian
  Matrix6_9d Jacobian_object_previous;
  // 
  Vector6d internal_force_contribution;
  // virtual arm platform torque
  Vector9d v_arm_platform_torque;

  // grasp matrix
  Vector3d end_eff_position_in_object;  // Position of the end-effector in  the object frame // from the initialization file
  Matrix3d Rot_arm_base2world;
  Matrix6d Rot_arm_base2world6x6;


  // information about the platfrom in mocap world
  Vector3d mid_pc_position_mocap_world;
  Quaterniond mid_pc_orientation_mocap_world;
  
 
  // Usefull rotation matrices
  // ---------------------------------
  // Rotation from platform to world
  Eigen::Matrix3d Rot_platform2world;

   // Activation of the projected inverse dynamics
  bool useInverseDynamics;    

  // Equilibrium define with respect to initial pose
  // starting time equilibrium
  Vector3d initial_posture_ee_position;
  Quaterniond initial_posture_ee_orientation;

  // for the object
  Vector3d initial_object_position_world;
  Quaterniond initial_object_orientation_world;

  // 
  // maximum linear and angular velocities
  double max_arm_lin_vel;
  double max_arm_ang_vel;
  double max_platform_lin_vel;
  double max_platform_ang_vel;

  int iter;

  // ================================================================
      // methods to compute the object twist from the pose measurements
  void compute_object_twist();

      //  method to to compute the  grasp matrix of the object 
      // that maps wrench in the world frame to wrench the arm base frame
  Matrix6d get_grasp_matrix(Vector3d end_eff_position_in_object,  // Position of the end-effector in  the object frame // from the initialization file
                            Matrix3d Rot_arm_base2world, 
                            Matrix3d Rot_end_eff2arm_base);

  // compute the virtual arm Jacobian
  Matrix6_9d get_virtual_arm_Jacobian(Matrix3d Rot_arm_base2world,
                                       Vector3d arm_real_position_);

  // // compute the sub-matrix of v_Jacobian_arm related to the platform
  Matrix6_9d get_virtual_platform_Jacobian(Matrix3d Rot_arm_base2world,
                                            Vector3d arm_real_position_);

  // Estimation of the internal wrench contribution
  // It is based on disturbance observer and uses the measured wrench at the end-effector
  void estimate_internal_wrench(Vector6d measured_Wrench); 

  // method that compute the virtual arm-platform torque using the inverse of the apparent (virtual) dynamics
  void compute_virtual_torques();

  bool get_object_reference_trajectory();

 
  // ================================================================

  // Initialization
  void wait_for_transformations();

  // Control
  void compute_admittance();



  // Callbacks
  void state_platform_callback(const nav_msgs::OdometryConstPtr msg);
  void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg);
  void wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg);
  void wrench_control_callback(const geometry_msgs::WrenchStampedConstPtr msg);


  // Util
  bool get_rotation_matrix(Matrix6d & rotation_matrix,
                           tf::TransformListener & listener,
                           std::string from_frame,  std::string to_frame);

  void publish_arm_state_in_world();

  // void get_ee_pose_world(geometry_msgs::Pose & ee_pose_world,
  //                        tf::TransformListener & listener);

  void limit_to_workspace();

  void publish_debuggings_signals();

  void send_commands_to_robot();

  void equilibrium_callback(const geometry_msgs::PointPtr msg);

  //void desired_platform_velocity_callback(const geometry_msgs::Twist msg);

  void object_ee_pose_world_callback(const geometry_msgs::PoseStampedConstPtr msg);

  void mid_pc_pose_mocap_world_callback(const geometry_msgs::PoseStampedConstPtr msg);

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
                         std::string topic_equilibrium_deisred,
                         std::string topic_equilibrium_real,
                         std::vector<double> M_p,
                         std::vector<double> M_a,
                         std::vector<double> D,
                         std::vector<double> D_p,
                         std::vector<double> D_a,
                         std::vector<double> K,
                         std::vector<double> d_e,
                         std::vector<double> workspace_limits,
                         double arm_max_vel,
                         double arm_max_acc,
                         double platform_max_vel,
                         double platform_max_acc,
                         double wrench_filter_factor,
                         double force_dead_zone_thres,
                         double torque_dead_zone_thres,

                        std::string topic_object_ee_pose_mocap_world,
                        std::string topic_mid_pc_pose_mocap_world,
                        std::vector<double> M_obj_imp,
                        std::vector<double> D_obj_imp,
                        std::vector<double> K_obj_imp,
                        std::vector<double> D_posture,
                        std::vector<double> K_posture,
                        std::vector<double> K_estimator);
  
  //ros::Subscriber sub_desired_platform_velocity;
  void run();
};

#endif // ADMITTANCECONTROLLER_H

