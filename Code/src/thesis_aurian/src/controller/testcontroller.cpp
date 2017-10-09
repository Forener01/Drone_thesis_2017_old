#include <thesis_aurian/controller.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "testcontroller");
  ros::NodeHandle nh;
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher takeoff_pub =
      nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  ros::Publisher land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1);
  ros::Publisher reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset", 1);

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