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
    std::string topic_equilibrium_desired,
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
    std::vector<double> K_estimator):
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
                                      platform_max_acc_(platform_max_acc),

                                      M_obj_imp_(M_obj_imp.data()),
                                      D_obj_imp_(D_obj_imp.data()), 
                                      K_obj_imp_(K_obj_imp.data()),
                                      D_posture_(D_posture.data()), 
                                      K_posture_(K_posture.data()),
                                      K_estimator_(K_estimator.data()) {


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
  // New added
  // sub_desired_platform_velocity = nh_.subscribe("/topic_desired_platform_velocity", 10,
  //                                     &AdmittanceController::desired_platform_velocity_callback, this,
  //                                     ros::TransportHints().reliable().tcpNoDelay());

  sub_object_ee_pose_mocap_world = nh_.subscribe(topic_object_ee_pose_mocap_world, 10,
                                   &AdmittanceController::object_ee_pose_world_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());

  sub_mid_pc_pose_mocap_world = nh_.subscribe(topic_mid_pc_pose_mocap_world, 10,
                                   &AdmittanceController::mid_pc_pose_mocap_world_callback, this,
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
  arm_desired_twist_.setZero();
  platform_desired_twist_.setZero();

  desired_platform_velocity_.setZero();

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

  wait_for_transformations();

  // varibales for the apparent dynamics of the system
  // ===============================================================================================
  //
  // Gains
  // -------
  // M_obj_imp_;                // virtual intertia
  // D_obj_imp_;                // virtual damping
  // K_obj_imp_;                // virtual stiffness
  // // robot Posture Gain matrices
  // K_posture_;               // stiffness gain 
  // D_posture_;               // damping gain

  // // Internal Wrench Estimator
  // K_estimator_;               // gaim matrix estimator
 
  //
  //
  // virtual inertia matrixss
  M_vap.block<6,6>(0,0) = M_a_;
  M_vap.block<6,6>(6,6) = M_p_;
  // virtual Damping matrix
  D_vap.block<6,6>(0,0) = D_a_;
  D_vap.block<6,6>(6,6) = D_p_;
  // virtual Stiffness matrix
  K_vap.block<6,6>(0,0) = K_;
  K_vap.block<6,6>(6,0) = -1.0*rotation_base_ * kin_constraints_ * K_;

  // interse of the virtual inertia matrix

  Inverse_M_vap = M_vap.inverse();


  // object distributed inertia
  obj_inertiaMx.setZero();
  obj_bias_forces.setZero();

  distributed_obj_inertiaMx = 0.5 * obj_inertiaMx;
  distributed_obj_bias_forces = 0.5 * obj_bias_forces; 

  // contribution from the internal wrench
  internal_force_contribution.setZero();
 



  //  initialization of some variables
  v_arm_platform_desired_acceleration.setZero();    // virtual system dynamics acceleration
  v_arm_platform_desired_twist.setZero();           // virtual system dynamics velocity
  v_arm_platform_control_wrench.setZero();          // virtual system control wrench


  v_arm_platform_torque.setZero();                  // torque of the virtual (apparent) arm-platform dynamics

  // 
  obj_real_position_world.setZero();     // USE FOR NOW THE ARM POSITION (OBJECT COINCIDE WITH THE PALM)
  obj_real_orientation_world = Eigen::Quaterniond(Eigen::Matrix3d::Identity());  // To be initialized to the equilibrium orientation

  while (nh_.ok() && !obj_real_position_world(0)) {
    ROS_WARN_THROTTLE(1, "Waiting for grasped object pose in mocap world...");
    ros::spinOnce();
    loop_rate_.sleep();
  }
  obj_real_twist_world.setZero();


  // mid_pc
  mid_pc_position_mocap_world.setZero();
  mid_pc_orientation_mocap_world = Eigen::Quaterniond(Eigen::Matrix3d::Identity());

  while (nh_.ok() && !mid_pc_position_mocap_world(0)) {
    ROS_WARN_THROTTLE(1, "Waiting for mid pic pose in mocap world...");
    ros::spinOnce();
    loop_rate_.sleep();
  }
  // Rotation from platform to world
  Rot_platform2world.setIdentity();   // set to identity for a time
  Rot_platform2world = mid_pc_orientation_mocap_world.normalized().toRotationMatrix();


  // reference trajectory of the object  (from a DS)
  // -----------------------------------
  obj_reference_position_world.setZero();
  obj_reference_position_world = obj_real_position_world;

  obj_reference_twist_world.setZero(); 
  obj_reference_acceleration_world.setZero();
  obj_reference_orientation_world = obj_real_orientation_world;    // ASSUMING NO ORIENTATION ERROR
  
  obj_real_position_world_previous = obj_real_position_world;
  Rot_real_obj_world_previous      = obj_real_orientation_world.normalized().toRotationMatrix();

  // 
  

  // Rotation from arm_base to world
  Rot_arm_base2world = Rot_platform2world * rotation_base_.block<3,3>(0,0);
  Rot_arm_base2world6x6.setIdentity();

  Rot_arm_base2world6x6.block<3,3>(0,0) = Rot_arm_base2world;
  Rot_arm_base2world6x6.block<3,3>(3,3) = Rot_arm_base2world;

  // Initialization of apparent dynamics Jacobian

  // Skew-symmetric matrix of position_arm_world
  //v_Jacobian_arm.resize(6, 12);
  v_Jacobian_arm.setZero();
  v_Jacobian_arm = AdmittanceController::get_virtual_arm_Jacobian(Rot_arm_base2world, arm_real_position_);

  v_Jacobian_platform.setZero();
  v_Jacobian_platform = AdmittanceController::get_virtual_platform_Jacobian(Rot_arm_base2world, arm_real_position_);

  // // Initialization of Object dynamics
  Jacobian_object_previous = v_Jacobian_arm;

  // vector of virtual bias forces
  // arm real pose w axis angle
  Vector6d arm_real_pose, arm_real_pose_error;
  arm_real_pose.setZero(); arm_real_pose_error.setZero();

  Eigen::AngleAxisd orientation_arm(arm_real_orientation_.normalized());

  arm_real_pose.head(3) = arm_real_position_;
  arm_real_pose.tail(3) = orientation_arm.axis() * orientation_arm.angle();

  //v_bias_forces = D_vap * v_arm_platform_desired_twist + K_vap * arm_real_pose - v_Jacobian_platform.transpose()* 0.0* Rot_arm_base2world6x6*wrench_external_;
  v_bias_forces = D_vap * v_arm_platform_desired_twist + K_vap * arm_real_pose_error - v_Jacobian_platform.transpose()* 1.0* Rot_arm_base2world6x6 * wrench_external_;


  // Activation of the object augmented inverse dynamics
  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  useInverseDynamics = true;

  // ==========================================================================================

}

///////////////////////////////////////////////////////////////
///////////////////// Control Loop ////////////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::run() {

  ROS_INFO("Running the admittance control loop .................");

  while (nh_.ok()) {
    // Admittance Dynamics computation
    compute_admittance();

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
  Vector6d coupling_wrench_platform =  D_ * (arm_desired_twist_) + K_ * error;

  error.topRows(3) = arm_real_position_ - equilibrium_position_;
  Vector6d coupling_wrench_arm =  D_ * (arm_desired_twist_) + K_ * error;

  
  // platform_desired_acceleration = M_p_.inverse() * (- D_p_ * platform_desired_twist_
  //                                 + rotation_base_ * kin_constraints_ * coupling_wrench_platform);
  // arm_desired_accelaration = M_a_.inverse() * ( - coupling_wrench_arm - D_a_ * arm_desired_twist_
  //                            + wrench_external_ + wrench_control_);

  // update the virtual control wrench for the apparent dynamics
      // AdmittanceController::compute_virtual_torques();
      // // compute the desired accelaration 
      // v_arm_platform_desired_acceleration = Inverse_M_vap * (v_arm_platform_torque + v_Jacobian_arm.transpose() * 1.0* Rot_arm_base2world6x6 * wrench_external_ - v_bias_forces);

  if (useInverseDynamics)
  {
      // update the virtual control wrench for the apparent dynamics
      AdmittanceController::compute_virtual_torques();

      // compute the desired accelaration 
      v_arm_platform_desired_acceleration = Inverse_M_vap * (v_arm_platform_torque + v_Jacobian_arm.transpose() * Rot_arm_base2world6x6* wrench_external_ - v_bias_forces);

      // Extracting the platform and the arm desired acceleration
      platform_desired_acceleration = 0.5*v_arm_platform_desired_acceleration.segment(0, 6);

      arm_desired_accelaration      = 0.5*kin_constraints_ * v_arm_platform_desired_acceleration.segment(6, 6);

  }
  else
  {
      platform_desired_acceleration = M_p_.inverse() * (- D_p_ * platform_desired_twist_
                                    + rotation_base_ * kin_constraints_ * coupling_wrench_platform);
      arm_desired_accelaration = M_a_.inverse() * ( - coupling_wrench_arm - D_a_ * arm_desired_twist_
                               + wrench_external_ + wrench_control_);
  }

  
  // std::cout << " ee_pose_world_ is :\n" << ee_pose_world_ << std::endl;
  // std::cout << " ee_twist_world_ is :\n" << ee_twist_world_ << std::endl;

  // std::cout << " error is :\n" << error << std::endl;
  //  std::cout << " wrench_external_ is :\n" << wrench_external_ << std::endl;
  //  std::cout << " internal_force_contribution is : \n"<< internal_force_contribution << std::endl; 

  // std::cout << " v_arm_desired_acceleration is :\n" << v_arm_platform_desired_acceleration.segment(0, 6) << std::endl;
  // std::cout << " arm_desired_accelaration is :\n" << arm_desired_accelaration << std::endl;

  // std::cout << " v_platform_desired_acceleration is :\n" << kin_constraints_ *v_arm_platform_desired_acceleration.segment(6, 6) << std::endl;
  // std::cout << " platform_desired_acceleration is :\n" << platform_desired_acceleration << std::endl;

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

  platform_desired_twist_ += 1.0* platform_desired_acceleration * duration.toSec();
  arm_desired_twist_      += 1.0* arm_desired_accelaration      * duration.toSec();

  // added
  // std::cout << " platform_desired_twist_ after is :\n " << platform_desired_twist_<< std::endl;
  // std::cout << " arm_desired_twist_ after is :\n " << arm_desired_twist_<< std::endl;  
  
  // std::cout << " Rot_real_obj_world_previous after is :\n " << Rot_real_obj_world_previous << std::endl;
  // std::cout << " Rot_platform2world after is :\n " << Rot_platform2world << std::endl;


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
      if (abs(wrench_ft_frame(i+3)) < torque_dead_zone_thres_) {
        wrench_ft_frame(i+3) = 0;
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
    ROS_WARN_THROTTLE(5, "The frame_id is not specified as ur5_arm_base_link");
  }
}

void AdmittanceController::equilibrium_callback(const geometry_msgs::PointPtr msg) {

  equilibrium_new_ << msg->x, msg->y, msg->z;

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

        ROS_INFO_STREAM_THROTTLE(2, "New eauiibrium at : " <<
                             equilibrium_position_(0) << " " <<
                             equilibrium_position_(1) << " " <<
                             equilibrium_position_(2)   );

}



// New Added
// void AdmittanceController::desired_platform_velocity_callback(const geometry_msgs::Twist msg) {

//   // vel_linx_ << msg.linear.x;
//   // vel_liny_ << msg.linear.y;
//   // vel_linz_ << msg.linear.z;

//   // vel_angx_ << msg.angular.x;
//   // vel_angy_ << msg.angular.y;
//   // vel_angz_ << msg.angular.z;
//   desired_platform_velocity_ << msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z;

//   // std::cout << desired_platform_velocity_(0)  << desired_platform_velocity_(1) << desired_platform_velocity_(2) << desired_platform_velocity_(3) << desired_platform_velocity_(4) << desired_platform_velocity_(5);
//   ROS_INFO_STREAM_THROTTLE(1, "Michael got his velocity ~~~~~ " << desired_platform_velocity_(0) << "\t"
//                                                               << desired_platform_velocity_(1) << "\t");
// }

///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::send_commands_to_robot() {

  geometry_msgs::Twist platform_twist_cmd;
  geometry_msgs::Twist arm_twist_cmd;

  platform_twist_cmd.linear.x  = platform_desired_twist_(0) + 1.0*desired_platform_velocity_(0); // New added
  platform_twist_cmd.linear.y  = platform_desired_twist_(1) + 1.0*desired_platform_velocity_(1);
  platform_twist_cmd.linear.z  = platform_desired_twist_(2) + 1.0*desired_platform_velocity_(2);
  platform_twist_cmd.angular.x = platform_desired_twist_(3) + 1.0*desired_platform_velocity_(3);
  platform_twist_cmd.angular.y = platform_desired_twist_(4) + 1.0*desired_platform_velocity_(4);
  platform_twist_cmd.angular.z = platform_desired_twist_(5) + 1.0*desired_platform_velocity_(5);

  arm_twist_cmd.linear.x  = arm_desired_twist_(0);
  arm_twist_cmd.linear.y  = arm_desired_twist_(1);
  arm_twist_cmd.linear.z  = arm_desired_twist_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_(5);

  pub_platform_cmd_.publish(platform_twist_cmd);
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


  if (arm_desired_twist_(0) < 0 && arm_real_position_(0) < workspace_limits_(0)) {
    arm_desired_twist_(0) = 0;
  }

  if (arm_desired_twist_(0) > 0 && arm_real_position_(0) > workspace_limits_(1)) {
    arm_desired_twist_(0) = 0;
  }

  if (arm_desired_twist_(1) < 0 && arm_real_position_(1) < workspace_limits_(2)) {
    arm_desired_twist_(1) = 0;
  }

  if (arm_desired_twist_(1) > 0 && arm_real_position_(1) > workspace_limits_(3)) {
    arm_desired_twist_(1) = 0;
  }

  if (arm_desired_twist_(2) < 0 && arm_real_position_(2) < workspace_limits_(4)) {
    arm_desired_twist_(2) = 0;
  }

  if (arm_desired_twist_(2) > 0 && arm_real_position_(2) > workspace_limits_(5)) {
    arm_desired_twist_(2) = 0;
  }

  // velocity of the arm along x, y, and z axis
  double norm_vel_des = (arm_desired_twist_.segment(0, 3)).norm();

  if (norm_vel_des > arm_max_vel_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast arm movements! velocity norm: " << norm_vel_des);

    arm_desired_twist_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);

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
                                                std::string to_frame) 
{
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


// ==========================================================================================================================
// ==========================================================================================================================
//                        Apparent dynamics
// ==========================================================================================================================
// ==========================================================================================================================

// eeObject_pose_world_callback

void AdmittanceController::object_ee_pose_world_callback(const geometry_msgs::PoseStampedConstPtr msg) 
{
  obj_real_position_world << msg->pose.position.x, 
                             msg->pose.position.y,
                             msg->pose.position.z;

  obj_real_orientation_world.coeffs() << msg->pose.orientation.x,
                                         msg->pose.orientation.y,
                                         msg->pose.orientation.z,
                                         msg->pose.orientation.w;

}

void AdmittanceController::mid_pc_pose_mocap_world_callback(const geometry_msgs::PoseStampedConstPtr msg) 
{
  mid_pc_position_mocap_world << msg->pose.position.x, 
                                 msg->pose.position.y,
                                 msg->pose.position.z;

  mid_pc_orientation_mocap_world.coeffs() << msg->pose.orientation.x,
                                             msg->pose.orientation.y,
                                             msg->pose.orientation.z,
                                             msg->pose.orientation.w;

}


// methods for the virtual dynamics
void AdmittanceController::compute_object_twist()
{

  // getting the sampling time
  ros::Duration duration = loop_rate_.expectedCycleTime();

  // compute the rotation mx from quaternion
  Eigen::Matrix3d Rot_real_obj_world = obj_real_orientation_world.normalized().toRotationMatrix();
  // get the skew-symmetric matrix associated with angular velocity
  Eigen::Matrix3d skew_omega_obj_world = 1./duration.toSec() * (Rot_real_obj_world - Rot_real_obj_world_previous) * Rot_real_obj_world.transpose();
  
  obj_real_twist_world.topRows(3)    = 1./duration.toSec() * (obj_real_position_world - obj_real_position_world_previous);
  obj_real_twist_world.bottomRows(3) << skew_omega_obj_world(1,0), skew_omega_obj_world(0,2), skew_omega_obj_world(2,1);

  obj_real_position_world_previous = obj_real_position_world;
  Rot_real_obj_world_previous = Rot_real_obj_world;

}

//  method to to compute the  grasp matrix of the object 
// that maps wrench in the world frame to wrench the arm base frame
Matrix6d AdmittanceController::get_grasp_matrix(Vector3d end_eff_position_in_object,  // Position of the end-effector in  the object frame // from the initialization file
                                                Matrix3d Rot_arm_base2world, 
                                                Matrix3d Rot_end_eff2arm_base)
{
  
  // 6x6 rotation from end-effector to object
  MatrixXd Rot6x6_eeffector2world(6, 6);
  Rot6x6_eeffector2world.setZero();
  Rot6x6_eeffector2world.block<3,3>(0, 0) = Rot_arm_base2world * Rot_end_eff2arm_base;
  Rot6x6_eeffector2world.block<3,3>(3, 3) = Rot_arm_base2world * Rot_end_eff2arm_base;
  // 6x6 rotation from end-effetor to the arm base
  MatrixXd Rot6x6_endeffector2arm_base(6, 6);
  Rot6x6_endeffector2arm_base.setZero();
  Rot6x6_endeffector2arm_base.block<3,3>(0, 0) = Rot_end_eff2arm_base;
  Rot6x6_endeffector2arm_base.block<3,3>(3, 3) = Rot_end_eff2arm_base;

  // grasp matrix wrt the object frame
  // skew matrix related to the end-effector position wrt to the object
  Matrix3d skew_mx_eef_in_obj;
  skew_mx_eef_in_obj.setZero();
  skew_mx_eef_in_obj(0, 1) = -end_eff_position_in_object(2);
  skew_mx_eef_in_obj(0, 2) =  end_eff_position_in_object(1);
  skew_mx_eef_in_obj(1, 0) =  end_eff_position_in_object(2);
  skew_mx_eef_in_obj(1, 2) = -end_eff_position_in_object(0);
  skew_mx_eef_in_obj(2, 0) = -end_eff_position_in_object(1);
  skew_mx_eef_in_obj(2, 1) =  end_eff_position_in_object(0);

  // grasp matrix wrt the object frame
  Matrix6d grasp_matrix_obj_obj;
  grasp_matrix_obj_obj.setIdentity();
  grasp_matrix_obj_obj.block<3,3>(3, 0) =  skew_mx_eef_in_obj;

  // grasp matrix
  Matrix6d obj_grasp_mx_world2arm_base = Rot6x6_eeffector2world * grasp_matrix_obj_obj * Rot6x6_endeffector2arm_base;

  return obj_grasp_mx_world2arm_base;

}

// compute the virtual arm Jacobian
Matrix6_12d AdmittanceController::get_virtual_arm_Jacobian(Matrix3d Rot_arm_base2world,
                                                           Vector3d arm_real_position_)
{
  //
  Eigen::Vector3d position_arm_world = Rot_arm_base2world * arm_real_position_;
  Eigen::Matrix3d skew_mx_position_arm_world;

  skew_mx_position_arm_world.setZero();
  skew_mx_position_arm_world(0,1) = -position_arm_world(2);
  skew_mx_position_arm_world(0,2) =  position_arm_world(1);
  skew_mx_position_arm_world(1,0) =  position_arm_world(2);
  skew_mx_position_arm_world(1,2) = -position_arm_world(0);
  skew_mx_position_arm_world(2,0) = -position_arm_world(1);
  skew_mx_position_arm_world(2,1) =  position_arm_world(0);

  // Jacobian of the arm
  Matrix6_12d a_Jacobian_arm;
  a_Jacobian_arm.setZero();

  a_Jacobian_arm.block<3,3>(0,0) = Rot_arm_base2world;  // rotation from arm_base to world frame.  w_R_p * p_R_ab
  a_Jacobian_arm.block<3,3>(3,3) = Rot_arm_base2world;  // rotation from arm_base to world frame.  w_R_p * p_R_ab
  a_Jacobian_arm.block<3,3>(0,6) = Rot_platform2world;  // rotation from platform to world frame.  w_R_p 
  a_Jacobian_arm.block<3,3>(0,9) = -1.0*skew_mx_position_arm_world * Rot_arm_base2world; 
  a_Jacobian_arm.block<3,3>(3,9) = Rot_platform2world; // rotation from platform to world frame.  w_R_p

  return a_Jacobian_arm;

}

// Get the sub-matrix of v_Jacobian_arm related to the platform
Matrix6_12d AdmittanceController::get_virtual_platform_Jacobian(Matrix3d Rot_arm_base2world,
                                                                Vector3d arm_real_position_)
{
  //
  Eigen::Vector3d position_arm_world = Rot_arm_base2world * arm_real_position_;
  Eigen::Matrix3d skew_mx_position_arm_world;

  skew_mx_position_arm_world.setZero();
  skew_mx_position_arm_world(0,1) = -position_arm_world(2);
  skew_mx_position_arm_world(0,2) =  position_arm_world(1);
  skew_mx_position_arm_world(1,0) =  position_arm_world(2);
  skew_mx_position_arm_world(1,2) = -position_arm_world(0);
  skew_mx_position_arm_world(2,0) = -position_arm_world(1);
  skew_mx_position_arm_world(2,1) =  position_arm_world(0);

  // Jacobian of the arm
  Matrix6_12d a_Jacobian_platform;
  a_Jacobian_platform.setZero();

  a_Jacobian_platform.block<3,3>(0,6) = Rot_platform2world;  // rotation from platform to world frame.  w_R_p 
  a_Jacobian_platform.block<3,3>(0,9) = -1.0*skew_mx_position_arm_world * Rot_arm_base2world; 
  a_Jacobian_platform.block<3,3>(3,9) = Rot_platform2world; // rotation from platform to world frame.  w_R_p

  return a_Jacobian_platform;

}

// Estimation of the internal wrench contribution
void AdmittanceController::estimate_internal_wrench(Vector6d measured_Wrench)
{
  // 
  // getting the sampling time
    ros::Duration duration = loop_rate_.expectedCycleTime();
    // 
    internal_force_contribution += -duration.toSec() *  K_estimator_ * (internal_force_contribution + obj_bias_forces - measured_Wrench) 
                                                       + K_estimator_ * obj_inertiaMx * obj_real_twist_world;
}


// // 
void AdmittanceController::compute_virtual_torques() 
{
  
  
  // getting the sampling time
    ros::Duration duration = loop_rate_.expectedCycleTime();

    // Rotation from arm_base to world
    Rot_platform2world = mid_pc_orientation_mocap_world.normalized().toRotationMatrix();
    Rot_arm_base2world = Rot_platform2world * rotation_base_.block<3,3>(0,0);

    
    // rotation matrix from arm base to world frame
    Rot_arm_base2world6x6.block<3,3>(0,0) = Rot_arm_base2world;
    Rot_arm_base2world6x6.block<3,3>(3,3) = Rot_arm_base2world;

    // Object State error with respect to the world frame
    // ====================
    Vector6d obj_pose_error_world;                      // between current and desired eef pose

    // Orientation error w.r.t. desired equilibriums
    if (obj_reference_orientation_world.coeffs().dot(obj_real_orientation_world.coeffs()) < 0.0) {
       obj_real_orientation_world.coeffs() << - obj_real_orientation_world.coeffs();
    }
    // object orientation  error
    Eigen::Quaterniond quat_obj_rot_error(obj_real_orientation_world  * obj_reference_orientation_world.inverse());
    // Normalize error quaternion
    if (quat_obj_rot_error.coeffs().norm() > 1e-3) {
        quat_obj_rot_error.coeffs() << quat_obj_rot_error.coeffs() / quat_obj_rot_error.coeffs().norm();
      }
    // expressing the orienattion with axis/ angle representation
    Eigen::AngleAxisd obj_des_orient_error(quat_obj_rot_error);

    // object pose error with axis/angle representation of the orientation
    obj_pose_error_world.topRows(3)    = obj_real_position_world - obj_reference_position_world;
    obj_pose_error_world.bottomRows(3) = obj_des_orient_error.axis() * obj_des_orient_error.angle();

    
    // object twist error 
    Vector6d obj_twist_error_world  = obj_real_twist_world - obj_reference_twist_world;   // error between current and desired twist of the object
    
  // Posture state error
  // --------------------
    // pose state error
    Vector12d v_arm_platform_posture_error;
    v_arm_platform_posture_error.setZero();

    // 
    Eigen::Quaterniond arm_equilibrium_orientation_ = equilibrium_orientation_;

    // Orientation error w.r.t. desired equilibriums
    if (arm_equilibrium_orientation_.coeffs().dot(arm_real_orientation_.coeffs()) < 0.0) {
        arm_real_orientation_.coeffs() << -arm_real_orientation_.coeffs();
    }

    Eigen::Quaterniond quat_arm_rot_error(arm_real_orientation_ * arm_equilibrium_orientation_.inverse());
    // Normalize error quaternion
    if (quat_arm_rot_error.coeffs().norm() > 1e-3) {
        quat_arm_rot_error.coeffs() << quat_arm_rot_error.coeffs() / quat_arm_rot_error.coeffs().norm();
    }
    Eigen::AngleAxisd err_arm_des_orient(quat_arm_rot_error);

    v_arm_platform_posture_error.segment(0, 3) = arm_real_position_ - equilibrium_position_seen_by_platform;    // To do : make it variable online
    v_arm_platform_posture_error.segment(3, 3) = err_arm_des_orient.axis() * err_arm_des_orient.angle();
    // v_arm_platform_posture_error.segment(6, 3) << 0.0, 0.0, 0.0;                         
    // v_arm_platform_posture_error.segment(9, 3) << 0.0, 0.0, 0.0;

    // posture velocity twist
    Vector12d v_arm_platform_posture_twist;
    v_arm_platform_posture_twist.setZero();
    v_arm_platform_posture_twist.head(6) = arm_desired_twist_;
    
  // Task space dynamics
  // ====================
    
  // Object Jacobian
  // ==================
  // grasp matrix 

  Eigen::Matrix3d Rot_end_eff2arm_base = rotation_base_.block<3,3>(0,0);

  Matrix6d grasp_mx_obj_world2arm_base = AdmittanceController::get_grasp_matrix(end_eff_position_in_object,   
                                                                                Rot_arm_base2world, 
                                                                                Rot_end_eff2arm_base);

  //MatrixXd Jacobian_object(6, v_Jacobian_arm.cols());
    // update the v_arm Jacobian
    v_Jacobian_arm = AdmittanceController::get_virtual_arm_Jacobian(Rot_arm_base2world, arm_real_position_);

  MatrixXd Inverse_grasp_mx_obj_world2arm_base = grasp_mx_obj_world2arm_base.inverse(); 
  MatrixXd Jacobian_object = Inverse_grasp_mx_obj_world2arm_base.transpose() * v_Jacobian_arm;

  Eigen::MatrixXd Jacobian_object_dot(6, v_Jacobian_arm.cols());

  Jacobian_object_dot = 1.0/duration.toSec() * (Jacobian_object - Jacobian_object_previous);

  Jacobian_object_previous = Jacobian_object;

  // Task space inertia matrix
  //MatrixXd InvM_vap = M_vap.inverse();
  MatrixXd v_Jo_InvM_vap_JoT    = Jacobian_object * Inverse_M_vap * Jacobian_object.transpose();
  Matrix6d v_taskspace_inertia  = v_Jo_InvM_vap_JoT.inverse();

  // task space bias forces
  Vector6d arm_real_pose;
  arm_real_pose.setZero(); 

  Eigen::AngleAxisd orientation_arm(arm_real_orientation_.normalized());

  arm_real_pose.head(3) = arm_real_position_;
  arm_real_pose.tail(3) = orientation_arm.axis() * orientation_arm.angle();
  // update virtual bias forces
  v_bias_forces = D_vap * v_arm_platform_desired_twist + K_vap * v_arm_platform_posture_error.segment(0, 6) - v_Jacobian_platform.transpose()* 1.0* Rot_arm_base2world6x6 * wrench_external_;
  //v_bias_forces = D_vap * v_arm_platform_desired_twist + K_vap * arm_real_pose - v_Jacobian_platform.transpose()* 0.0 * Rot_arm_base2world6x6*wrench_external_;


  VectorXd v_taskspace_bias_forces = v_taskspace_inertia * (Jacobian_object * Inverse_M_vap * v_bias_forces - Jacobian_object_dot * v_arm_platform_desired_twist);
  // dynamically consistent pseudoinverse of Jacobian_object
  MatrixXd Jacobian_object_harsh  = v_taskspace_inertia * Jacobian_object * Inverse_M_vap;
  // Nullspace projector operator of Jacobian_object
  MatrixXd Nullspace_Jacobian_obj = MatrixXd::Identity(Jacobian_object.cols(),Jacobian_object.cols()) - Jacobian_object.transpose() * Jacobian_object_harsh;



  // object augmented task space dynamics
  // -------------------------------------
  // object augmented iniertia matrix (eef in object space + object inertia)
  Matrix6d obj_augmented_inertia      = v_taskspace_inertia + distributed_obj_inertiaMx;   
  // object augmented bias forces (Damping + Elestic forces)      
  Vector6d obj_augmented_bias_forces  = v_taskspace_bias_forces + distributed_obj_bias_forces; 
  

  // compute the generalized forces
  // ===============================
  // estimate the internal force contribution
  Vector6d measured_Wrench = Rot_arm_base2world6x6 * wrench_external_;
  AdmittanceController::estimate_internal_wrench(measured_Wrench);

  Vector6d object_generalized_force = obj_augmented_inertia * (obj_reference_acceleration_world - M_obj_imp_.inverse() * (D_obj_imp_ * obj_twist_error_world + K_obj_imp_ * obj_pose_error_world))
                                      + (obj_augmented_inertia * M_obj_imp_.inverse() * - MatrixXd::Identity(6,6))* 1.0* Rot_arm_base2world6x6 * wrench_external_ 
                                      + obj_augmented_bias_forces 
                                      + 0.0*internal_force_contribution;  

  // computing the internal motion virtual torque (posture torque)
  // -------------------------------------------------------------
  // posture reference acceleration

  // Vector12d v_desired_posture_accel = - K_posture_.asDiagonal() * v_arm_platform_posture_error  D_ * (arm_desired_twist_)
  //                  - D_posture_.asDiagonal() * v_arm_platform_desired_twist;
  Vector12d v_desired_posture_torque = -1.0* K_posture_.asDiagonal() * v_arm_platform_posture_error - D_posture_.asDiagonal() * v_arm_platform_posture_twist;

  //posture torque                  
  //Vector12d v_posture_torque = M_vap * v_desired_posture_accel + v_bias_forces;
  Vector12d v_posture_torque = v_desired_posture_torque;


  // compute the overall torque
  // ==========================
  v_arm_platform_torque = Jacobian_object.transpose() * object_generalized_force + 0.0*Nullspace_Jacobian_obj * v_posture_torque;

  
  // std::cout << " object_generalized_force is : \n"<< object_generalized_force << std::endl;
  // std::cout << " Inverse_M_vap *v_bias_forces is : \n"<< Inverse_M_vap *v_bias_forces << std::endl;
  // std::cout << " obj_pose_error_world is : \n"<< obj_pose_error_world << std::endl; 


  //std::cout << " v_arm_platform_posture_error.segment(0, 6) is : \n"<< v_arm_platform_posture_error.segment(0, 6) << std::endl; 
  //std::cout << " v_desired_posture_torque is :\n" << v_desired_posture_torque << std::endl;




  // std::cout << desired_platform_velocity_(0)  << desired_platform_velocity_(1) << desired_platform_velocity_(2) << desired_platform_velocity_(3) << desired_platform_velocity_(4) << desired_platform_velocity_(5);
  // ROS_INFO_STREAM_THROTTLE(1, "The virtual arm torques are ~~~~~ "  << v_arm_platform_torque(0) << "\t"
  //                                                               << v_arm_platform_torque(1) << "\t"
  //                                                               << v_arm_platform_torque(2) << "\t"
  //                                                               << v_arm_platform_torque(3) << "\t"
  //                                                               << v_arm_platform_torque(4) << "\t"
  //                                                               << v_arm_platform_torque(5) << "\t"
  //                                                               << v_arm_platform_torque(6) << "\t"
  //                                                               << v_arm_platform_torque(7) << "\t"
  //                                                               << v_arm_platform_torque(9) << "\t");

  // ROS_INFO_STREAM_THROTTLE(1, "The virtual platform torques are ~~~~~ "   << v_arm_platform_torque(6) << "\t"
  //                                                                         << v_arm_platform_torque(7) << "\t"
  //                                                                         << v_arm_platform_torque(9) << "\t");




}