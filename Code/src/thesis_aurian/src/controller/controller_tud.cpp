/*
This file transmits to the drone the trajectories with velocity commands,
via ardrone_autonomy commands, following a square trajectory.

Author: Aurian d'Avernas
Date: september 2017
*/
#include <thesis_aurian/controller.hpp>

Controller::Controller() {
  ros::NodeHandle nh;

  // Subscribers

  // Publishers
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); // without control
  // vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_ref", 1); // for TUD
  // control

  takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1);
  reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset", 1);
}

void Controller::land() { land_pub.publish(std_msgs::Empty()); }

void Controller::takeoff() { takeoff_pub.publish(std_msgs::Empty()); }

// This function sets the hover mode.
void Controller::hover() {
  geometry_msgs::Twist cmd;

  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.z = 0.0;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;

  vel_pub.publish(cmd);
}

// This function sets new velocities values and publishes them.
void Controller::load_vel(double linX, double linY, double linZ, double angZ) {
  geometry_msgs::Twist cmd;

  cmd.linear.x = linX;
  cmd.linear.y = linY;
  cmd.linear.z = linZ;
  cmd.angular.z = angZ;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;

  vel_pub.publish(cmd);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller");
  Controller controlnode;
  ros::Rate loop_rate(500);

  ros::Duration(8.0).sleep();

  controlnode.takeoff();
  ROS_INFO_STREAM_ONCE("The drone is taking off !");
  ros::Duration(5.0).sleep();

  ROS_INFO_STREAM_ONCE("The drone is in hover mode !");
  controlnode.hover();
  ros::Duration(5.0).sleep();

  // Moving to the left #1
  ROS_INFO_STREAM_ONCE("The drone starts the path-planning !");
  controlnode.load_vel(0.0, 0.15, 0.0, 0.0);
  ros::Duration(1.5).sleep();
  // Stabilizing #1
  controlnode.hover();
  ros::Duration(1.5).sleep();

  // Moving backward #2
  controlnode.load_vel(-0.15, 0.0, 0.0, 0.0);
  ros::Duration(1.5).sleep();
  // Stabilizing #2
  controlnode.hover();
  ros::Duration(1.5).sleep();

  // Moving to the right #3
  controlnode.load_vel(0.0, -0.15, 0.0, 0.0);
  ros::Duration(1.5).sleep();
  // Stabilizing #3
  controlnode.hover();
  ros::Duration(1.5).sleep();

  // Moving forward #4
  controlnode.load_vel(0.15, 0.0, 0.0, 0.0);
  ros::Duration(1.5).sleep();
  // Stabilizing #2
  controlnode.hover();
  ros::Duration(1.5).sleep();

  // Final step
  ROS_INFO_STREAM_ONCE("The drone just finished !");
  controlnode.hover();
  ros::Duration(1.5).sleep();
  controlnode.land();
  ROS_INFO_STREAM_ONCE("The drone just landed !");

  return 0;
}
