#include <thesis_aurian/testcontroller.hpp>

ros::Publisher vel_pub, takeoff_pub, land_pub, reset_pub;

void Controller::land() { land_pub.publish(std_msgs::Empty()); }

void Controller::takeoff() { takeoff_pub.publish(std_msgs::Empty()); }

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_controller");
  ros::NodeHandle nh;

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_PID", 1);
  takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1);
  reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset", 1);

  ros::Rate loop_rate(500);

  ros::Duration(8.0).sleep();

  takeoff();
  ROS_INFO_STREAM_ONCE("The drone is taking off !");
  ros::Duration(5.0).sleep();

  ROS_INFO_STREAM_ONCE("The drone is in hover mode !");
  hover();
  ros::Duration(5.0).sleep();

  // Moving to the left #1
  ROS_INFO_STREAM_ONCE("The drone starts the path-planning !");
  load_vel(0.0, 0.15, 0.0, 0.0);
  ros::Duration(1.5).sleep();
  // Stabilizing #1
  hover();
  ros::Duration(1.5).sleep();

  // Moving backward #2
  load_vel(-0.15, 0.0, 0.0, 0.0);
  ros::Duration(1.5).sleep();
  // Stabilizing #2
  hover();
  ros::Duration(1.5).sleep();

  // Moving to the right #3
  load_vel(0.0, -0.15, 0.0, 0.0);
  ros::Duration(1.5).sleep();
  // Stabilizing #3
  hover();
  ros::Duration(1.5).sleep();

  // Moving forward #4
  load_vel(0.15, 0.0, 0.0, 0.0);
  ros::Duration(1.5).sleep();
  // Stabilizing #2
  hover();
  ros::Duration(1.5).sleep();

  // Final step
  ROS_INFO_STREAM_ONCE("The drone just finished !");
  hover();
  ros::Duration(1.5).sleep();
  land();
  ROS_INFO_STREAM_ONCE("The drone just landed !");

  return 0;
}
