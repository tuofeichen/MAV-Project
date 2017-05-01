#include "px4_offboard/include.h"
#include <sensor_msgs/Imu.h>
// #include <posedetection_msgs/ObjectDetection.h>
// #include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/PoseArray.h>
#include "px4_offboard/MavState.h"

px4_offboard::MavState g_command;

bool is_update = 0;
bool is_takeoff = 0;
my_pos pos_read;
float az = 0.0;

void tagCallback(const geometry_msgs::PoseArray);
void poseCallback(const geometry_msgs::PoseStamped pos);
void spCallback(const geometry_msgs::PoseStamped sp);
void velCallback(const geometry_msgs::TwistStamped vel);
void imuCallback(const sensor_msgs::Imu imu);

int main(int argc, char **argv) {
  ros::init(argc, argv, "ARTag");
  ros::NodeHandle nh;
  // publish joy command
  ros::Publisher state_pub =
   nh.advertise<px4_offboard::MavState>("/april/cmd_mav", 100);
  // listen to checkerboard
  ros::Subscriber april_sub = nh.subscribe(
      "/tag_detections_pose", 100, &tagCallback);
  ros::Subscriber pose_sub = nh.subscribe(
      "/mavros/local_position/pose", 100, &poseCallback);
  ros::Subscriber sp_sub = nh.subscribe(
       "/mavros/setpoint_position/local", 100, &spCallback);
  ros::Subscriber vel_sub = nh.subscribe(
      "/mavros/setpoint_velocity/cmd_vel", 100, &velCallback);
   ros::Subscriber att_sub = nh.subscribe(
      "/mavros/imu/data", 100, &imuCallback);


  ROS_INFO("AR TAG START!");
  int update_cnt = 0;
  int scan_sign = 1;
  ros::Rate loop_rate(100);

  while (ros::ok()) {

    if (is_update) {
      is_update = 0;
      update_cnt ++;
      //state_pub.publish(g_command);
      ROS_INFO("Tag Publishing setpoint: x %f  y %f z %f", g_command.position.x,
               g_command.position.y, g_command.position.z);

      state_pub.publish(g_command);

    }
    else {
	//ROS_INFO("pos read yaw %f",pos_read.yaw);
      	if (update_cnt <= 0){
      	update_cnt = 0;
      	if((is_takeoff)&&(az<0.05)){
            ROS_INFO("New Surveillance Round!");
            //yaw_start = pos_read.yaw; //note down the yaw when starting
             g_command.position.x = 0;
             g_command.position.y = 0;
             g_command.position.z = 0;
             g_command.yaw = 1;
             state_pub.publish(g_command);
            }
      	}
	      // TODO: lost track handling?
    }


    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void tagCallback(const geometry_msgs::PoseArray tag_pose) {
  if(!tag_pose.poses.empty()){
    ROS_INFO("read position!");
    is_update = 1;

    g_command.position.x = 0; // clear previous command
    g_command.position.y = 0;
    g_command.position.z = 0;
    g_command.yaw = 0;

    g_command.position.x = 0.2* tag_pose.poses[0].position.x;
    g_command.position.z = 0.2 * tag_pose.poses[0].position.y;
    g_command.position.y = 0.12* (tag_pose.poses[0].position.z - 1);
    g_command.yaw = -0.5*tag_pose.poses[0].orientation.w;

  }
  else{
  	g_command.position.x = 0;
  	g_command.position.y = 0;
    g_command.position.z = 0;
	}

}

void spCallback(const geometry_msgs::PoseStamped sp)
{
//	ROS_INFO("Position Setpoint received");
	is_takeoff = 1;
}

void poseCallback(const geometry_msgs::PoseStamped pos)
{
  pos_read.px = pos.pose.position.x;
  pos_read.py = pos.pose.position.y;
  pos_read.pz = pos.pose.position.z;

  pos_read.q(0) = pos.pose.orientation.w; // qw
  pos_read.q(1) = pos.pose.orientation.x; // qx
  pos_read.q(2) = pos.pose.orientation.y; // qy
  pos_read.q(3) = pos.pose.orientation.z; // qz

  pos_read.yaw = atan2(2*(pos_read.q[0] * pos_read.q[3]+ pos_read.q[1]*pos_read.q[2]), \
  1-2*(pos_read.q[3]*pos_read.q[3]+pos_read.q[2]*pos_read.q[2]));

}
void velCallback(const geometry_msgs::TwistStamped vel)
{
  is_takeoff = 0;
}

void imuCallback(const sensor_msgs::Imu imu)
{
//	ROS_INFO("Z angular velocity is %f", imu.angular_velocity.z);
	az = imu.angular_velocity.z;
}
