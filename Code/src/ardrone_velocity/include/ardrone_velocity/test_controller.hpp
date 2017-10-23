#ifndef TEST_CONTROLLER_HPP
#define TEST_CONTROLLER_HPP

#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <stdlib.h>
#include <ardrone_velocity/path_planning.hpp>
#include <string.h>

class TestController{
public:
  TestController();
  ros::Publisher vel_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher reset_pub;
  ros::Publisher poseref_pub;
  void land(void);
  void takeoff(void);
  void hover(void);
  void load_vel(double linX, double linY, double linZ, double angZ);
  void load_pose(double X, double Y, double Z);
};
#endif // TEST_CONTROLLER_HPP
