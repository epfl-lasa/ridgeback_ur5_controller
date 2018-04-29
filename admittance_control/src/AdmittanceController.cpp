#include "AdmittanceController.h"

AdmittanceController::AdmittanceController(ros::NodeHandle &n,
    double frequency,
    std::string topic_platform_command,
    std::string topic_platform_state,
    std::string topic_arm_command,
    std::string topic_arm_pose_world,
    std::string topic_arm_twist_world,
    std::string topic_external_wrench_arm_frame,
    std::string topic_control_wrench_arm_frame,
    std::string topic_arm_state,
    std::string topic_external_wrench,
    std::string topic_control_wrench,
    std::string topic_admittance_ratio,
    std::string topic_equilibrium_desired,
    std::string topic_equilibrium_real,
    std::string topic_ds_velocity,
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
    double torque_dead_zone_thres) :
  nh_(n), loop_rate_(frequency),
  wrench_filter_factor_(wrench_filter_factor),
  force_dead_zone_thres_(force_dead_zone_thres),
  torque_dead_zone_thres_(torque_dead_zone_thres),
  M_p_(M_p.data()), M_a_(M_a.data()), D_(D.data()),
  D_p_(D_p.data()), D_a_(D_a.data()), K_(K.data()),
  workspace_limits_(workspace_limits.data()),
  arm_max_vel_(arm_max_vel),
  arm_max_acc_(arm_max_acc),
  platform_max_vel_(platform_max_vel),
  platform_max_acc_(platform_max_acc) {


  // Subscribers
  sub_platform_state_ = nh_.subscribe(topic_platform_state, 5,
                                      &AdmittanceController::state_platform_callback, this,
                                      ros::TransportHints().reliable().tcpNoDelay());
  sub_arm_state_ = nh_.subscribe(topic_arm_state, 10,
                                 &AdmittanceController::state_arm_callback, this,
                                 ros::TransportHints().reliable().tcpNoDelay());

  sub_wrench_external_ = nh_.subscribe(topic_external_wrench, 5,
                                       &AdmittanceController::wrench_callback, this,
                                       ros::TransportHints().reliable().tcpNoDelay());
  sub_wrench_control_ = nh_.subscribe(topic_control_wrench, 5,
                                      &AdmittanceController::wrench_control_callback, this,
                                      ros::TransportHints().reliable().tcpNoDelay());

  sub_equilibrium_desired_ = nh_.subscribe(topic_equilibrium_desired, 10,
                             &AdmittanceController::equilibrium_callback, this,
                             ros::TransportHints().reliable().tcpNoDelay());

  sub_ds_velocity_ = nh_.subscribe(topic_ds_velocity, 10,
                              &AdmittanceController::ds_velocity_callback , this,
                              ros::TransportHints().reliable().tcpNoDelay());

  sub_admittance_ratio_ = nh_.subscribe(topic_admittance_ratio, 10,
                                        &AdmittanceController::admittance_ratio_callback, this,
                                        ros::TransportHints().reliable().tcpNoDelay());


  // Publishers
  pub_platform_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_platform_command, 5);
  pub_arm_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 5);

  pub_ee_pose_world_ = nh_.advertise<geometry_msgs::PoseStamped>(
                         topic_arm_pose_world, 5);
  pub_ee_twist_world_ = nh_.advertise<geometry_msgs::TwistStamped>(
                          topic_arm_twist_world, 5);

  pub_wrench_external_ = nh_.advertise<geometry_msgs::WrenchStamped>(
                           topic_external_wrench_arm_frame, 5);
  pub_wrench_control_ = nh_.advertise<geometry_msgs::WrenchStamped>(
                          topic_control_wrench_arm_frame, 5);

  pub_equilibrium_real_ = nh_.advertise<geometry_msgs::PointStamped>(
                            topic_equilibrium_real, 5);




  ROS_INFO_STREAM("Arm max vel:" << arm_max_vel_ << " max acc:" << arm_max_acc_);
  ROS_INFO_STREAM("Platform max vel:" << platform_max_vel_ << " max acc:" << platform_max_acc_);


  // initializing the class variables
  wrench_external_.setZero();
  wrench_control_.setZero();

  ee_pose_world_.setZero();
  ee_twist_world_.setZero();

  // setting the equilibrium position and orientation
  Vector7d equilibrium_full(d_e.data());
  equilibrium_position_ << equilibrium_full.topRows(3);

  // This does not change
  equilibrium_position_seen_by_platform << equilibrium_full.topRows(3);
  // Make sure the orientation goal is normalized
  equilibrium_orientation_.coeffs() << equilibrium_full.bottomRows(4) /
                                    equilibrium_full.bottomRows(4).norm();

  equilibrium_new_.setZero();


  // starting from a state that does not create movements on the robot
  arm_real_orientation_ = equilibrium_orientation_;

  // setting the robot state to zero and wait for data
  arm_real_position_.setZero();
  platform_real_position_.setZero();

  while (nh_.ok() && !arm_real_position_(0)) {
    ROS_WARN_THROTTLE(1, "Waiting for the state of the arm...");
    ros::spinOnce();
    loop_rate_.sleep();
  }

  // Init integrator
  arm_desired_twist_adm_.setZero();
  platform_desired_twist_.setZero();

  arm_desired_twist_ds_.setZero();
  arm_desired_twist_final_.setZero();


  // Kinematic constraints between base and arm at the equilibrium
  // the base only effected by arm in x,y and rz
  kin_constraints_.setZero();
  kin_constraints_.topLeftCorner(2, 2).setIdentity();
  kin_constraints_.bottomRightCorner(1, 1).setIdentity();


  // kin_constraints_.setZero();
  // kin_constraints_.topLeftCorner(3, 3).setIdentity();
  // kin_constraints_.bottomRightCorner(3, 3).setIdentity();
  // Screw on the z torque axis
  // kin_constraints_.topRightCorner(3, 3) <<
  //                                       0, 0, equilibrium_position_(1),
  //                                       0, 0, -equilibrium_position_(0),
  //                                       0, 0, 0;


  ft_arm_ready_ = false;
  arm_world_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;

  admittance_ratio_ = 1;

  wait_for_transformations();
}

///////////////////////////////////////////////////////////////
///////////////////// Control Loop ////////////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::run() {

  ROS_INFO("Running the admittance control loop .................");

  while (nh_.ok()) {
    // Admittance Dynamics computation
    compute_admittance();

    // sum the vel from admittance to DS in this function
    // limit the the movement of the arm to the permitted workspace
    limit_to_workspace();

    // Copy commands to messages
    send_commands_to_robot();

    // Arm pose/twist in the world frame
    publish_arm_state_in_world();

    // publishing visualization/debugging info
    publish_debuggings_signals();

    ros::spinOnce();
    loop_rate_.sleep();
  }


}



///////////////////////////////////////////////////////////////
///////////////////// Admittance Dynamics /////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::compute_admittance() {

  Vector6d platform_desired_acceleration;
  Vector6d arm_desired_accelaration;

  Vector6d error;


  // Orientation error w.r.t. desired equilibriums
  if (equilibrium_orientation_.coeffs().dot(arm_real_orientation_.coeffs()) < 0.0) {
    arm_real_orientation_.coeffs() << -arm_real_orientation_.coeffs();
  }

  Eigen::Quaterniond quat_rot_err(arm_real_orientation_
                                  * equilibrium_orientation_.inverse());
  if (quat_rot_err.coeffs().norm() > 1e-3) {
    // Normalize error quaternion
    quat_rot_err.coeffs() << quat_rot_err.coeffs() / quat_rot_err.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() *
                      err_arm_des_orient.angle();


  // Translation error w.r.t. desired equilibrium
  error.topRows(3) = arm_real_position_ - equilibrium_position_seen_by_platform;
  Vector6d coupling_wrench_platform =  D_ * (arm_desired_twist_adm_) + K_ * error;

  error.topRows(3) = arm_real_position_ - equilibrium_position_;
  Vector6d coupling_wrench_arm =  D_ * (arm_desired_twist_adm_) + K_ * error;



  platform_desired_acceleration = M_p_.inverse() * (- D_p_ * platform_desired_twist_
                                  + rotation_base_ * kin_constraints_ * coupling_wrench_platform);
  arm_desired_accelaration = M_a_.inverse() * ( - coupling_wrench_arm - D_a_ * arm_desired_twist_adm_
                             + admittance_ratio_ * wrench_external_ + wrench_control_);

  // limiting the accelaration for better stability and safety
  // x and y for  platform and x,y,z for the arm
  double p_acc_norm = (platform_desired_acceleration.segment(0, 2)).norm();
  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();

  if (p_acc_norm > platform_max_acc_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high platform accelaration!"
                             << " norm: " << p_acc_norm);
    platform_desired_acceleration.segment(0, 2) *= (platform_max_acc_ / p_acc_norm);
  }

  if (a_acc_norm > arm_max_acc_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!"
                             << " norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }

  // Integrate for velocity based interface
  ros::Duration duration = loop_rate_.expectedCycleTime();

  platform_desired_twist_ += platform_desired_acceleration * duration.toSec();
  arm_desired_twist_adm_      += arm_desired_accelaration      * duration.toSec();


}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::state_platform_callback(
  const nav_msgs::OdometryConstPtr msg) {
  platform_real_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y,
                          msg->pose.pose.position.z;
  platform_real_orientation_.coeffs() << msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                                    msg->pose.pose.orientation.w;

  platform_real_twist_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                       msg->twist.twist.linear.z, msg->twist.twist.angular.x,
                       msg->twist.twist.angular.y, msg->twist.twist.angular.z;
}

void AdmittanceController::state_arm_callback(
  const cartesian_state_msgs::PoseTwistConstPtr msg) {
  arm_real_position_ << msg->pose.position.x, msg->pose.position.y,
                     msg->pose.position.z;

  arm_real_orientation_.coeffs() << msg->pose.orientation.x,
                               msg->pose.orientation.y,
                               msg->pose.orientation.z,
                               msg->pose.orientation.w;

  arm_real_twist_ << msg->twist.linear.x, msg->twist.linear.y,
                  msg->twist.linear.z, msg->twist.angular.x, msg->twist.angular.y,
                  msg->twist.angular.z;
}

void AdmittanceController::wrench_callback(
  const geometry_msgs::WrenchStampedConstPtr msg) {
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;
  if (ft_arm_ready_) {

    // Reading the FT-sensor in its own frame (robotiq_force_torque_frame_id)
    wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y,
                    msg->wrench.force.z, msg->wrench.torque.x,
                    msg->wrench.torque.y, msg->wrench.torque.z;

    // Dead zone for the FT sensor
    // if (wrench_ft_frame.topRows(3).norm() < force_dead_zone_thres_) {
    //   wrench_ft_frame.topRows(3).setZero();
    // }
    // if (wrench_ft_frame.bottomRows(3).norm() < torque_dead_zone_thres_) {
    //   wrench_ft_frame.bottomRows(3).setZero();
    // }

    for (int i = 0; i < 3; i++) {
      if (abs(wrench_ft_frame(i)) < force_dead_zone_thres_) {
        wrench_ft_frame(i) = 0;
      }
      if (abs(wrench_ft_frame(i + 3)) < torque_dead_zone_thres_) {
        wrench_ft_frame(i + 3) = 0;
      }
    }




    // Get transform from arm base link to platform base link
    get_rotation_matrix(rotation_ft_base, listener_ft_,
                        "ur5_arm_base_link", "robotiq_force_torque_frame_id");

    // Filter and update
    wrench_external_ <<  (1 - wrench_filter_factor_) * wrench_external_ +
                     wrench_filter_factor_ * rotation_ft_base * wrench_ft_frame;
  }
}

void AdmittanceController::wrench_control_callback(
  const geometry_msgs::WrenchStampedConstPtr msg) {

  if (msg->header.frame_id == "ur5_arm_base_link") {
    wrench_control_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                    msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  }
  else  {
    ROS_WARN_THROTTLE(5, "wrench_control_callback: The frame_id is not specified as ur5_arm_base_link");
  }
}

void AdmittanceController::ds_velocity_callback(const geometry_msgs::TwistStampedPtr msg) {

  arm_desired_twist_ds_ << msg->twist.linear.x , msg->twist.linear.y , msg->twist.linear.z ;
  // ROS_INFO_STREAM_THROTTLE(1,"received velocity, z:" << arm_desired_twist_ds_(2));

}
void AdmittanceController::equilibrium_callback(const geometry_msgs::PointPtr msg) {

  equilibrium_new_ << msg->x , msg->y, msg->z;

  // bool equ_update = true;
  // if (equilibrium_new_(0) < workspace_limits_(0) || equilibrium_new_(0) > workspace_limits_(1)) {
  //   ROS_WARN_STREAM_THROTTLE (1, "Desired equilibrium is out of workspace.  x = "
  //                             << equilibrium_new_(0) << " not in [" << workspace_limits_(0) << " , "
  //                             << workspace_limits_(1) << "]");
  //   equ_update = false;
  // }

  // if (equilibrium_new_(1) < workspace_limits_(2) || equilibrium_new_(1) > workspace_limits_(3)) {
  //   ROS_WARN_STREAM_THROTTLE (1, "Desired equilibrium is out of workspace.  y = "
  //                             << equilibrium_new_(1) << " not in [" << workspace_limits_(2) << " , "
  //                             << workspace_limits_(3) << "]");
  //   equ_update = false;
  // }

  // if (equilibrium_new_(2) < workspace_limits_(4) || equilibrium_new_(2) > workspace_limits_(5)) {
  //   ROS_WARN_STREAM_THROTTLE (1, "Desired equilibrium is out of workspace.  x = "
  //                             << equilibrium_new_(2) << " not in [" << workspace_limits_(4) << " , "
  //                             << workspace_limits_(5) << "]");
  //   equ_update = false;
  // }

  // if (equ_update) {
  //   equilibrium_position_ = equilibrium_new_;
  //   // ROS_INFO_STREAM_THROTTLE(2, "New eauiibrium at : " <<
  //   //                          equilibrium_position_(0) << " " <<
  //   //                          equilibrium_position_(1) << " " <<
  //   //                          equilibrium_position_(2)   );
  // }


  if (equilibrium_new_(0) > workspace_limits_(0) && equilibrium_new_(0) < workspace_limits_(1)) {

    equilibrium_position_(0) = equilibrium_new_(0);
  }

  if (equilibrium_new_(1) > workspace_limits_(2) && equilibrium_new_(1) < workspace_limits_(3)) {
    equilibrium_position_(1) = equilibrium_new_(1);

  }

  if (equilibrium_new_(2) > workspace_limits_(4) && equilibrium_new_(2) < workspace_limits_(5)) {
    equilibrium_position_(2) = equilibrium_new_(2);

  }

}


void AdmittanceController::admittance_ratio_callback(const std_msgs::Float32Ptr msg) {

  double h = msg->data;

  if (h > 1) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance ration higher than one is recieved " <<  h);
    h = 1;
  }
  else if (h < 0 ) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance ratio lower than zero is recieved " << h);
    h = 0;
  }
  else {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance ratio between 0 and 1 recieved " << h);
  }

  admittance_ratio_ = h;

}

///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::send_commands_to_robot() {

  // for the platform
  geometry_msgs::Twist platform_twist_cmd;

  platform_twist_cmd.linear.x  = platform_desired_twist_(0);
  platform_twist_cmd.linear.y  = platform_desired_twist_(1);
  platform_twist_cmd.linear.z  = platform_desired_twist_(2);
  platform_twist_cmd.angular.x = platform_desired_twist_(3);
  platform_twist_cmd.angular.y = platform_desired_twist_(4);
  platform_twist_cmd.angular.z = platform_desired_twist_(5);

  pub_platform_cmd_.publish(platform_twist_cmd);


  // for the arm
  geometry_msgs::Twist arm_twist_cmd;

  arm_twist_cmd.linear.x  = arm_desired_twist_final_(0);
  arm_twist_cmd.linear.y  = arm_desired_twist_final_(1);
  arm_twist_cmd.linear.z  = arm_desired_twist_final_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_final_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_final_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_final_(5);

  // ROS_WARN_STREAM_THROTTLE(1,"sending z vel: " << arm_twist_cmd.linear.z);

  pub_arm_cmd_.publish(arm_twist_cmd);
}


void AdmittanceController::limit_to_workspace() {

  if (arm_real_position_(0) < workspace_limits_(0) || arm_real_position_(0) > workspace_limits_(1)) {
    ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  x = "
                              << arm_real_position_(0) << " not in [" << workspace_limits_(0) << " , "
                              << workspace_limits_(1) << "]");
  }

  if (arm_real_position_(1) < workspace_limits_(2) || arm_real_position_(1) > workspace_limits_(3)) {
    ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  y = "
                              << arm_real_position_(1) << " not in [" << workspace_limits_(2) << " , "
                              << workspace_limits_(3) << "]");
  }

  if (arm_real_position_(2) < workspace_limits_(4) || arm_real_position_(2) > workspace_limits_(5)) {
    ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  z = "
                              << arm_real_position_(2) << " not in [" << workspace_limits_(4) << " , "
                              << workspace_limits_(5) << "]");
  }


  arm_desired_twist_final_ = arm_desired_twist_adm_;
  // arm_desired_twist_final_.segment(0,3) += (1- admittance_ratio_) * arm_desired_twist_ds_;
  arm_desired_twist_final_.segment(0,3) += arm_desired_twist_ds_;

  if (arm_desired_twist_final_(0) < 0 && arm_real_position_(0) < workspace_limits_(0)) {
    arm_desired_twist_final_(0) = 0;
  }

  if (arm_desired_twist_final_(0) > 0 && arm_real_position_(0) > workspace_limits_(1)) {
    arm_desired_twist_final_(0) = 0;
  }

  if (arm_desired_twist_final_(1) < 0 && arm_real_position_(1) < workspace_limits_(2)) {
    arm_desired_twist_final_(1) = 0;
  }

  if (arm_desired_twist_final_(1) > 0 && arm_real_position_(1) > workspace_limits_(3)) {
    arm_desired_twist_final_(1) = 0;
  }

  if (arm_desired_twist_final_(2) < 0 && arm_real_position_(2) < workspace_limits_(4)) {
    arm_desired_twist_final_(2) = 0;
  }

  if (arm_desired_twist_final_(2) > 0 && arm_real_position_(2) > workspace_limits_(5)) {
    arm_desired_twist_final_(2) = 0;
  }

  // velocity of the arm along x, y, and z axis
  double norm_vel_des = (arm_desired_twist_final_.segment(0, 3)).norm();

  if (norm_vel_des > arm_max_vel_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast arm movements! velocity norm: " << norm_vel_des);

    arm_desired_twist_final_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);

  }

  // velocity of the platfrom only along x and y axis
  double norm_vel_platform = (platform_desired_twist_.segment(0, 2)).norm();

  if (norm_vel_platform > platform_max_vel_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast platform movements! velocity norm: " << norm_vel_platform);

    platform_desired_twist_.segment(0, 2) *= (platform_max_vel_ / norm_vel_platform);

  }
}


//////////////////////
/// INITIALIZATION ///
//////////////////////
void AdmittanceController::wait_for_transformations() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  rotation_base_.setZero();

  // Makes sure all TFs exists before enabling all transformations in the callbacks
  while (!get_rotation_matrix(rotation_base_, listener,
                              "base_link", "ur5_arm_base_link")) {
    sleep(1);
  }

  while (!get_rotation_matrix(rot_matrix, listener,
                              "world", "base_link")) {
    sleep(1);
  }
  base_world_ready_ = true;

  while (!get_rotation_matrix(rot_matrix, listener,
                              "world", "ur5_arm_base_link")) {
    sleep(1);
  }
  arm_world_ready_ = true;
  while (!get_rotation_matrix(rot_matrix, listener,
                              "ur5_arm_base_link", "world")) {
    sleep(1);
  }
  world_arm_ready_ = true;

  while (!get_rotation_matrix(rot_matrix, listener,
                              "ur5_arm_base_link", "FT300_link")) {
    sleep(1);
  }

  ft_arm_ready_ = true;
  ROS_INFO("The Force/Torque sensor is ready to use.");
}




////////////
/// UTIL ///
////////////

bool AdmittanceController::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try {
    listener.lookupTransform(from_frame, to_frame,
                             ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) {
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }

  return true;
}



void AdmittanceController::publish_arm_state_in_world() {
  // publishing the cartesian velocity of the EE in the world-frame
  Matrix6d rotation_a_base_world;
  Matrix6d rotation_p_base_world;


  if (arm_world_ready_ && base_world_ready_) {
    get_rotation_matrix(rotation_a_base_world, listener_arm_,
                        "world", "ur5_arm_base_link");
    get_rotation_matrix(rotation_p_base_world, listener_arm_,
                        "world", "base_link");

    ee_twist_world_ = rotation_a_base_world * arm_real_twist_
                      + rotation_p_base_world * platform_real_twist_;
    // ee_twist_world_ = arm_real_twist_ + platform_real_twist_;
  }

  geometry_msgs::TwistStamped msg_twist;
  msg_twist.header.stamp    = ros::Time::now();
  msg_twist.header.frame_id = "world";
  msg_twist.twist.linear.x = ee_twist_world_(0);
  msg_twist.twist.linear.y = ee_twist_world_(1);
  msg_twist.twist.linear.z = ee_twist_world_(2);
  msg_twist.twist.angular.x = ee_twist_world_(3);
  msg_twist.twist.angular.y = ee_twist_world_(4);
  msg_twist.twist.angular.z = ee_twist_world_(5);
  pub_ee_twist_world_.publish(msg_twist);


  // publishing the cartesian position of the EE in the world-frame

  tf::StampedTransform transform;


  if (arm_world_ready_ && base_world_ready_) {
    try {
      // listener.lookupTransform("ur5_arm_base_link", "robotiq_force_torque_frame_id",
      listener_arm_.lookupTransform("world", "robotiq_force_torque_frame_id",
                                    ros::Time(0), transform);

      // transform.getRotation().getW();

      ee_pose_world_(0) = transform.getOrigin().x();
      ee_pose_world_(1) = transform.getOrigin().y();
      ee_pose_world_(2) = transform.getOrigin().z();
      ee_pose_world_(3) = transform.getRotation().x();
      ee_pose_world_(4) = transform.getRotation().y();
      ee_pose_world_(5) = transform.getRotation().z();
      ee_pose_world_(6) = transform.getRotation().w();
    }
    catch (tf::TransformException ex) {
      ROS_WARN_THROTTLE(1, "Couldn't lookup for ee to world transform...");
      ee_pose_world_.setZero();
      ee_pose_world_(6) = 1; // quat.w = 1
    }
  }

  geometry_msgs::PoseStamped msg_pose;
  msg_pose.header.stamp    = ros::Time::now();
  msg_pose.header.frame_id = "world";
  msg_pose.pose.position.x = ee_pose_world_(0);
  msg_pose.pose.position.y = ee_pose_world_(1);
  msg_pose.pose.position.z = ee_pose_world_(2);
  msg_pose.pose.orientation.x = ee_pose_world_(3);
  msg_pose.pose.orientation.y = ee_pose_world_(4);
  msg_pose.pose.orientation.z = ee_pose_world_(5);
  msg_pose.pose.orientation.w = ee_pose_world_(6);
  pub_ee_pose_world_.publish(msg_pose);

}



void AdmittanceController::publish_debuggings_signals() {

  geometry_msgs::WrenchStamped msg_wrench;

  msg_wrench.header.stamp    = ros::Time::now();
  msg_wrench.header.frame_id = "ur5_arm_base_link";
  msg_wrench.wrench.force.x  = wrench_external_(0);
  msg_wrench.wrench.force.y  = wrench_external_(1);
  msg_wrench.wrench.force.z  = wrench_external_(2);
  msg_wrench.wrench.torque.x = wrench_external_(3);
  msg_wrench.wrench.torque.y = wrench_external_(4);
  msg_wrench.wrench.torque.z = wrench_external_(5);
  pub_wrench_external_.publish(msg_wrench);

  msg_wrench.header.stamp    = ros::Time::now();
  msg_wrench.header.frame_id = "ur5_arm_base_link";
  msg_wrench.wrench.force.x  = wrench_control_(0);
  msg_wrench.wrench.force.y  = wrench_control_(1);
  msg_wrench.wrench.force.z  = wrench_control_(2);
  msg_wrench.wrench.torque.x = wrench_control_(3);
  msg_wrench.wrench.torque.y = wrench_control_(4);
  msg_wrench.wrench.torque.z = wrench_control_(5);
  pub_wrench_control_.publish(msg_wrench);


  geometry_msgs::PointStamped msg_point;

  msg_point.header.stamp     = ros::Time::now();
  msg_point.header.frame_id  = "ur5_arm_base_link";
  msg_point.point.x          = equilibrium_position_(0);
  msg_point.point.y          = equilibrium_position_(1);
  msg_point.point.z          = equilibrium_position_(2);
  pub_equilibrium_real_.publish(msg_point);

}

