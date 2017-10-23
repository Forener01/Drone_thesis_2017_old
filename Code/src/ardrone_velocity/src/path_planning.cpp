#include <ardrone_velocity/path_planning.hpp>

Path_Planning::Path_Planning(){
  ros::NodeHandle nh;

  // Subscribers
  odom_sub = nh.subscribe("ardrone/odometry", 1, &Path_Planning::odomCb, this);
  poseref_sub = nh.subscribe("pose_ref", 1, &Path_Planning::poserefCb, this);
  // poseout_sub = nh.subscribe("pose_out", 1, &Path_Planning::pose_to_velCb, this);
  // Publishers
  // pose_pub = nh.advertise<geometry_msgs::Pose>("pose_out", 1);
  poseref_pub = nh.advertise<geometry_msgs::Pose>("pose_ref", 1);
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_PID", 1);
}

void Path_Planning::poserefCb(const geometry_msgs::Pose &pose){
  pose_ref = pose;
}

void Path_Planning::odomCb(const nav_msgs::Odometry &odo){
  odo_msg = odo;

  position_control();
}

void Path_Planning::load_pose(double X, double Y, double Z){
  geometry_msgs::Pose pose;

  pose.position.x = X;
  pose.position.y = Y;
  pose.position.z = Z;

  poseref_pub.publish(pose);
}

void Path_Planning::position_control(void){
  double distX, distY, distZ;

  distX = sqrt(pow(pose_ref.position.x, 2) - pow(odo_msg.pose.pose.position.x, 2));
  distY = sqrt(pow(pose_ref.position.y, 2) - pow(odo_msg.pose.pose.position.y, 2));
  distZ = sqrt(pow(pose_ref.position.z, 2) - pow(odo_msg.pose.pose.position.z, 2));

  if (distX > 0.2){
    pose_out.position.x = pose_ref.position.x - odo_msg.pose.pose.position.x;
  }

  if (distY > 0.2){
    pose_out.position.y = pose_ref.position.y - odo_msg.pose.pose.position.y;
  }

  if (distZ > 0.2){
    pose_out.position.z = pose_ref.position.z - odo_msg.pose.pose.position.z;
  }

  //pose_pub.publish(pose_out);

  K = 1/10;

  velIn.linear.x = K*(pose_out.position.x);
  velIn.linear.y = K*(pose_out.position.y);
  velIn.linear.z = K*(pose_out.position.z);

  vel_pub.publish(velIn);
}

// void Path_Planning::pose_to_velCb(const nav_msgs::Pose &pose_out){
//   K = 1/10;
//
//   velIn.linear.x = K*(pose_out.position.x);
//   velIn.linear.y = K*(pose_out.position.y);
//   velIn.linear.z = K*(pose_out.position.z);
//
//   vel_pub.publish(velIn);
// }

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "path_planning");  // Name of the node
  Path_Planning mypath;

   int32_t looprate = 2000; //hz
   ros::Rate loop_rate(looprate);

  // ros::spin();
  while (mypath.nh.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
