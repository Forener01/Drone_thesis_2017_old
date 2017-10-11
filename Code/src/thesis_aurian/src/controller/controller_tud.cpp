/*
This file transmits to the drone the trajectories with velocity commands,
via ardrone_autonomy commands, following a square trajectory.

Author: Aurian d'Avernas
Date: september 2017
*/
#include <thesis_aurian/controller_tud.hpp>

Controller::Controller() {
  ros::NodeHandle nh;

  // Subscribers
  velin_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel_PID", 1);
  // Publishers
  velout_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller");
  Controller controlnode;
  ros::Rate loop_rate(500);

  ros::Duration(8.0).sleep();

while (ros::ok()) {
 // PID CONTROL
}

  return 0;
}
