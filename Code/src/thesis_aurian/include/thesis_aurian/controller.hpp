#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

class Controller {
public:
  Controller();

  ros::NodeHandle nh;
  void land(void);
  void takeoff(void);
  void hover(void);
  void load_vel(double linX, double linY, double linZ, double angZ);

private:
  ros::Publisher vel_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher reset_pub;
};

#endif // CONTROLLER_HPP
