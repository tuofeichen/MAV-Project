#include "obj_test/MoveCommand.h"
#include "ros/ros.h"
#include <iostream>
#include <string.h>

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"

////////////////////////////////////////
// test 1
//
// takes off
// 
// if object is seen, land
//
// end
//////////////////////////////////////

bool is_takeoff_done = false; 
bool finish = false;
obj_test::MoveCommand cmd;

void spCallback(const geometry_msgs::PoseStamped sp);
void velCallback(const geometry_msgs::TwistStamped vel);
void CameraCallBack(const geometry_msgs::Point TargetPosition);


bool detected = 0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TakeOffAndFindAnObject");	
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);
	ros::Subscriber find_object_sub = nh.subscribe("/FindObjectSub", 100, &CameraCallBack);
	ros::Subscriber sp_sub = nh.subscribe("/mavros/setpoint_position/local", 100, &spCallback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/setpoint_velocity/cmd_vel", 100, &velCallback);
	ros::Publisher cmd_pub = nh.advertise<obj_test::MoveCommand>("/mov_cmd", 100);
	
	
	while(ros::ok())
	{
	
	
	if(detected){
		ROS_INFO("Object was detected in the scene.");
		cmd.distance = 0;
		cmd.code = -1;
	
		cmd.distance = 0;
		cmd.code = 5; 
		cmd_pub.publish(cmd); // stop moving and then hover
	
		ROS_INFO("landing");
		//finish = true;
	}
		
		ros::spinOnce();
		loop_rate.sleep();
		// if(finish) return 0;
	}
	
	return 0;
}


void spCallback(const geometry_msgs::PoseStamped sp)
{	
  is_takeoff_done = true;
}


void velCallback(const geometry_msgs::TwistStamped vel)
{
  is_takeoff_done = false;
}


void CameraCallBack(const geometry_msgs::Point TargetPosition) 
{
	detected = 1;
}