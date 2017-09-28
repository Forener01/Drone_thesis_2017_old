/*
This file transmits to the drone the trajectories to take, via ardrone_autonomy
commands.

Author: Aurian d'Avernas
Date: 2017
*/

// #include <thesis_aurian/controller/trajectories.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectories");

  ros::NodeHandle nh;
  ros::Rate loop_rate(50);

  // Creating the publisher objects
  ros::Publisher takeoff_pub =
      nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1);
  ros::Publisher reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset", 1);

  // std_msgs::Empty msg;
  // takeoff_pub.publish(msg);
  // Setting values for hover mode
  geometry_msgs::Twist cmd_hover;
  cmd_hover.linear.x = 0.0;
  cmd_hover.linear.y = 0.0;
  cmd_hover.linear.z = 0.0;
  cmd_hover.angular.x = 0.0;
  cmd_hover.angular.y = 0.0;
  cmd_hover.angular.z = 0.0;

  geometry_msgs::Twist cmd_rotate_left;
  cmd_rotate_left.linear.x = 0.0;
  cmd_rotate_left.linear.y = 0.0;
  cmd_rotate_left.linear.z = 0.0;
  cmd_rotate_left.angular.x = 0.0;
  cmd_rotate_left.angular.y = 0.0;
  cmd_rotate_left.angular.z = 0.325;

  geometry_msgs::Twist cmd_move_left;
  cmd_move_left.linear.x = 0;
  cmd_move_left.linear.y = 0.15;
  cmd_move_left.linear.z = 0;
  cmd_move_left.angular.x = 0;
  cmd_move_left.angular.y = 0;
  cmd_move_left.angular.z = 0;

  int i = 1;

  // double time_start = (double)ros::Time::now().toSec();
  //
  // while (ros::ok()) {
  //   // TAKING OFF
  //   if ((double)ros::Time::now().toSec() < time_start + 10.0) {
  //     takeoff_pub.publish(std_msgs::Empty());
  //     ROS_INFO_STREAM_ONCE("The drone is taking off !");
  //     vel_pub.publish(cmd_hover);
  //   }
  //
  //   // RESETTING
  //   else if ((double)ros::Time::now().toSec() > time_start + 125.0) {
  //     reset_pub.publish(std_msgs::Empty());
  //     ROS_INFO_STREAM_ONCE("The drone has been reset !");
  //   }
  //
  //   // LANDING
  //   else if ((double)ros::Time::now().toSec() > time_start + 120.0) {
  //     land_pub.publish(std_msgs::Empty());
  //     ROS_INFO_STREAM_ONCE("The drone just landed !");
  //   }
  //
  //   // ROTATING
  //   else if (((double)ros::Time::now().toSec() > time_start + 30.0) &&
  //            ((double)ros::Time::now().toSec() < time_start + 40.0)) {
  //     vel_pub.publish(cmd_rotate_left);
  //     ROS_INFO_STREAM_ONCE("The drone is rotating");
  //   }
  //
  //   // HOVER mode
  //   else {
  //     vel_pub.publish(cmd_hover);
  //     ROS_INFO_STREAM_ONCE("The drone is in hover mode !");
  //   }
  //
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  ///////////////////////////
  // DIRECT IMPLEMENTATION //
  ///////////////////////////

  ros::Duration(8.0).sleep();

  // TAKE OFF
  takeoff_pub.publish(std_msgs::Empty());
  ROS_INFO_STREAM_ONCE("The drone is taking off !");
  ros::Duration(5.0).sleep();
  ROS_INFO_STREAM_ONCE("The drone is in hover mode !");
  vel_pub.publish(cmd_hover);
  ros::Duration(5.0).sleep();

  for (i = 1; i <= 4; i++) {
    // MOVING LEFT #1
    vel_pub.publish(cmd_move_left);
    ROS_INFO_STREAM("The drone is moving !");
    ros::Duration(1.25).sleep();

    vel_pub.publish(cmd_hover);
    ROS_INFO_STREAM("The drone is stabilizing !");
    ros::Duration(1.5).sleep();

    // ROTATING LEFT #1
    vel_pub.publish(cmd_rotate_left);
    ROS_INFO_STREAM("The drone is rotating !");
    ros::Duration(2.25).sleep();

    vel_pub.publish(cmd_hover);
    ROS_INFO_STREAM("The drone is stabilizing !");
    ros::Duration(1.5).sleep();
  }

  // FINAL STAB
  ROS_INFO_STREAM_ONCE("The drone just finished !");
  vel_pub.publish(cmd_hover);
  ros::Duration(5.0).sleep();

  // LANDING
  land_pub.publish(std_msgs::Empty());
  ROS_INFO_STREAM_ONCE("The drone just landed !");

  return 0;
}
