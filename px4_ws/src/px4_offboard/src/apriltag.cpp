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
   nh.advertise<px4_offboard::JoyCommand>("/april/cmd_mav", 100);
  // listen to checkerboard
  ros::Subscriber checker_sub = nh.subscribe(
      "/tag_detections_pose", 100, &tagCallback);
  int update_cnt = 1;
  int scan_sign = 1;
  ros::Rate loop_rate(100);
  
  while (ros::ok()) {
    if (is_update) {
      is_update = 0;
      update_cnt += 10 ;
      if (update_cnt == 100)
	update_cnt = 100;
      state_pub.publish(g_command);
      ROS_INFO("Tag Publishing setpoint: x %f  y %f z %f", g_command.position.x,
               g_command.position.y, g_command.position.z);
      // clear buffer
      g_command.position.x = 0;
      g_command.position.y = 0;
      g_command.position.z = 0;
      g_command.yaw = -1/update_cnt;
    }
    else // if can't find it turn around and keep finding 
    {
      if (update_cnt >= 1)
      	update_cnt -= 0.001;
	
     	if (update_cnt < 3){
      	g_command.position.x = 0;
      	g_command.position.y = 0;
      	g_command.position.z = 0;
	
	if (update_cnt > 1){ // scan mode
      		g_command.yaw = 0.1/update_cnt*scan_sign;
		scan_sign ++;
	        scan_sign  = -scan_sign;
	}
	else // normal rotate mode
	{
		scan_sign = 1;
		g_command.yaw = 0.1/update_cnt;
	}
      	state_pub.publish(g_command);
	}
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

  g_command.position.x = 0; // clean previous command
  g_command.position.y = 0;
  g_command.position.z = 0;
  g_command.yaw = 0;
  
  if (fabs(tag_pose.poses[0].position.x > 0.1))
    g_command.position.x = tag_pose.poses[0].position.x;  // left

  if (fabs(tag_pose.poses[0].position.y > 0.1))
    g_command.position.z = tag_pose.poses[0].position.y;

  if (tag_pose.poses[0].position.z > 0.6)
    g_command.position.y = tag_pose.poses[0].position.z - 0.6;
  else if (tag_pose.poses[0].position.z < 0.3)
    g_command.position.y = -tag_pose.poses[0].position.z + 0.3;  
  
  }
  

}
