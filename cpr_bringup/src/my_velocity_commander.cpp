#include "ros/ros.h"
#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Twist.h"
#include "eigen3/Eigen/Core"
#include "std_msgs/String.h"


/**
 * This function sends velocity twist command for the platform through the arm.
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "velocity_twist_sender");

  if (argc != 5)
  {
    ROS_INFO("usage: velocity_twist_sender X Y Wz  duration_in_sec");
    return 1;
  }

  double duration_in_sec;

  Eigen::VectorXd ee_velo_twist(6);
  ee_velo_twist.setZero();

  ee_velo_twist(0) = atof(argv[1]);
  ee_velo_twist(1) = atof(argv[2]);
  ee_velo_twist(5) = atof(argv[3]);

  std::cout<< " what I got " << atof(argv[1]) << std::endl;

  duration_in_sec = atof(argv[4]);

  // NodeHanle object
  ros::NodeHandle n;
  // declare the publisher of 
  ros::Publisher velo_sender_pub = n.advertise<geometry_msgs::Twist>("topic_desired_platform_velocity", 10);
  //ros::Publisher velo_sender_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  //
  ros::Rate loop_rate(125.0);

  geometry_msgs::Twist platform_twist_cmd;

  bool looping = true;

  
  double begin_in_sec = ros::Time::now().toSec();

  //while (ros::ok())
  while (looping && ros::ok())
  {
    platform_twist_cmd.linear.x = ee_velo_twist(0);
    platform_twist_cmd.linear.y = ee_velo_twist(1);
    platform_twist_cmd.linear.z = ee_velo_twist(2);
    platform_twist_cmd.angular.x = ee_velo_twist(3);
    platform_twist_cmd.angular.y = ee_velo_twist(4);
    platform_twist_cmd.angular.z = ee_velo_twist(5);

    ROS_INFO_STREAM_THROTTLE(1, "Publishing the arm twist cmd. x: " << platform_twist_cmd.linear.x  );

    velo_sender_pub.publish(platform_twist_cmd);

    ros::spinOnce();

    loop_rate.sleep();

    if (ros::Time::now().toSec() - begin_in_sec > duration_in_sec){

      ee_velo_twist.setZero();
      platform_twist_cmd.linear.x = ee_velo_twist(0);
      platform_twist_cmd.linear.y = ee_velo_twist(1);
      platform_twist_cmd.linear.z = ee_velo_twist(2);
      platform_twist_cmd.angular.x = ee_velo_twist(3);
      platform_twist_cmd.angular.y = ee_velo_twist(4);
      platform_twist_cmd.angular.z = ee_velo_twist(5);

      velo_sender_pub.publish(platform_twist_cmd);

      looping = false;
    }


  }


  return 0;
}
// %EndTag(FULLTEXT)%
