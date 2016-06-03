#include <ros/ros.h>
// #include <posedetection_msgs/ObjectDetection.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "px4_offboard/JoyCommand.h"

px4_offboard::JoyCommand g_command;
bool is_update = 0;

void tagCallback(const geometry_msgs::PoseArray);

int main(int argc, char **argv) {
  ros::init(argc, argv, "ARTag");
  ros::NodeHandle nh;
  // publish joy command
  ros::Publisher state_pub =
      nh.advertise<px4_offboard::JoyCommand>("/joy/cmd_mav", 100);
  // listen to checkerboard
  ros::Subscriber checker_sub = nh.subscribe(
      "/tag_detections_pose", 100, &tagCallback);
  // checkerboard always armed
  g_command.arm = 1;
  g_command.offboard = 1;

  ros::Rate loop_rate(100);
  
  while (ros::ok()) {
    if (is_update) {
      is_update = 0;
      state_pub.publish(g_command);
      ROS_INFO("Tag Publishing setpoint: x %f  y %f z %f", g_command.position.x,
               g_command.position.y, g_command.position.z);
      // clear buffer
      g_command.position.x = 0;
      g_command.position.y = 0;
      g_command.yaw = 0;
    }
    else // if can't find it turn around and keep finding 
    {
      g_command.yaw = 0.1;
    }


    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void tagCallback(const geometry_msgs::PoseArray tag_pose) {
  // double linear  = 0.5;
  // double angular = 0.5;

  if(!tag_pose.poses.empty()){
  	ROS_INFO("read position!");
  	is_update = 1;

  g_command.position.x = 0; // clean previous command
  g_command.position.y = 0;
  g_command.position.z = 0;
  g_command.yaw = 0;
  
  if (fabs(tag_pose.poses[0].position.x > 0.1))
    g_command.position.x = tag_pose.poses[0].position.x;  // left

  if (fabs(tag_pose.poses[0].position.y > 0.1))
    g_command.position.z = tag_pose.poses[0].position.y;

  if (tag_pose.poses[0].position.z > 0.5)
    g_command.position.y = tag_pose.poses[0].position.z - 0.5;
  else if (tag_pose.poses[0].position.z < 0.3)
    g_command.position.y = -tag_pose.poses[0].position.z + 0.5;  
  
  }
  

}
