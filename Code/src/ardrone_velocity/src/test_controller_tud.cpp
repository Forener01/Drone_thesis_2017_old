// #include <ardrone_velocity/test_controller_tud.hpp>
#include <ardrone_velocity/test_controller.hpp>

TestController::TestController(){
char const* test_type;
ros::param::get("~test_type", test_type);
char const* without_control = "without_control";
char const* velocity_control = "velocity_control";
char const* position_control = "position_control";

poseref_pub = nh.advertise<geometry_msgs::Pose>("pose_ref", 1);
vel_pub =
    nh.advertise<geometry_msgs::Twist>("cmd_PID", 1); // with TUD Controller
// vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); // without
// controller
takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1);
reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset", 1);
}

void TestController::land(void) { land_pub.publish(std_msgs::Empty()); }

void TestController::takeoff(void) { takeoff_pub.publish(std_msgs::Empty()); }

// This function sets new velocities values and publishes them to navdata.
void TestController::load_vel(double linX, double linY, double linZ, double angZ) {
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
void TestController::hover(void) {
  geometry_msgs::Twist cmd;

  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.z = 0.0;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;

  vel_pub.publish(cmd);
}

void TestController::load_pose(double X, double Y, double Z){
  geometry_msgs::Pose pose;

  pose.position.x = X;
  pose.position.y = Y;
  pose.position.z = Z;

  poseref_pub.publish(pose);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_controller");
  ros::NodeHandle nh;
  TestController mytest;

  ros::Rate loop_rate(50);

  double sleeptime = 2.0; // unit of s
  double speed = 0.6;     // unit of m/s

  ros::Duration(8.0).sleep();

  mytest.takeoff();
  ROS_INFO_STREAM_ONCE("The drone is taking off !");
  ros::Duration(5.0).sleep();

  ROS_INFO_STREAM_ONCE("The drone is in hover mode !");
  mytest.hover();
  ros::Duration(5.0).sleep();

  if (strcmp(test_type, velocity_control)){
    // Moving to the left #1
    ROS_INFO_STREAM_ONCE("The drone starts the path-planning with velocity control !");
    mytest.load_vel(0.0, speed, 0.0, 0.0);
    ros::Duration(sleeptime).sleep();
    ROS_INFO_STREAM_ONCE("Corner #1");
    // Stabilizing #1
    mytest.hover();
    ros::Duration(sleeptime).sleep();

    // Moving backward #2
    mytest.load_vel(-speed, 0.0, 0.0, 0.0);
    ros::Duration(sleeptime).sleep();
    ROS_INFO_STREAM_ONCE("Corner #2");
    // Stabilizing #2
    mytest.hover();
    ros::Duration(sleeptime).sleep();

    // Moving to the right #3
    mytest.load_vel(0.0, -speed, 0.0, 0.0);
    ros::Duration(sleeptime).sleep();
    ROS_INFO_STREAM_ONCE("Corner #3");
    // Stabilizing #3
    mytest.hover();
    ros::Duration(sleeptime).sleep();

    // Moving forward #4
    mytest.load_vel(speed, 0.0, 0.0, 0.0);
    ros::Duration(sleeptime).sleep();
    ROS_INFO_STREAM_ONCE("Corner #4");
    // Stabilizing #4
    mytest.hover();
    ros::Duration(sleeptime).sleep();
  }

  else if (strcmp(test_type, position_control)){
    // Going to Corner #1
    ROS_INFO_STREAM_ONCE("The drone starts the path-planning with position control !");
    mytest.load_pose(0.0, 2.0, 0.0);
    ros::Duration(sleeptime).sleep();
    ROS_INFO_STREAM_ONCE("Corner #1");
    // Stabilizing #1
    mytest.hover();
    ros::Duration(sleeptime).sleep();

    // Going to Corner #2
    mytest.load_pose(-2.0, 2.0, 0.0);
    ros::Duration(sleeptime).sleep();
    ROS_INFO_STREAM_ONCE("Corner #2");
    // Stabilizing #2
    mytest.hover();
    ros::Duration(sleeptime).sleep();

    // Going to Corner #3
    mytest.load_pose(-2.0, 0.0, 0.0);
    ros::Duration(sleeptime).sleep();
    ROS_INFO_STREAM_ONCE("Corner #3");
    // Stabilizing #3
    mytest.hover();
    ros::Duration(sleeptime).sleep();

    // Going to Corner #4
    mytest.load_pose(0.0, 0.0, 0.0);
    ros::Duration(sleeptime).sleep();
    ROS_INFO_STREAM_ONCE("Corner #4");
    // Stabilizing #4
    mytest.hover();
    ros::Duration(sleeptime).sleep();
  }

  // Final step
  ROS_INFO_STREAM_ONCE("The drone just finished !");
  mytest.hover();
  ros::Duration(5.0).sleep();
  mytest.land();
  ROS_INFO_STREAM_ONCE("The drone just landed !");

  return 0;
}
