#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include "obj_test/JoyCommand.h"
#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"

////////////////////////////////////////
// test 3
//
// takes off
// 
// if object is seen, track it
//
// end
//////////////////////////////////////


geometry_msgs::Point ActualObjectPosition;
obj_test::JoyCommand MovePosition;
bool take_off_done=false;
bool not_found = true;


void CameraCallBack(const geometry_msgs::Point ObjectPosition);
void velCallback(const geometry_msgs::TwistStamped vel);
void spCallback(const geometry_msgs::PoseStamped sp);


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "FindObjectAndTrackIt");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	
	ros::Publisher move_pub = nh.advertise<obj_test::JoyCommand>("/april/cmd_mav", 100); // publish how much the drone should move
	ros::Subscriber camera_sub = nh.subscribe("/FindObjectSub", 100, &CameraCallBack); // get target position
	ros::Subscriber sp_sub = nh.subscribe("/mavros/setpoint_position/local", 100, &spCallback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/setpoint_velocity/cmd_vel", 100, &velCallback);
		
	while(ros::ok())
	{
		
		if(take_off_done)
		{
			if(not_found)
			{
				ROS_INFO("Spinning!");
				
				MovePosition.position.x = 0;
            	MovePosition.position.y = 0;
            	MovePosition.position.z = 0;
            	MovePosition.yaw = 1;
				move_pub.publish(MovePosition);
			}
			else
			{
				ROS_INFO("Object Found!");
				
				move_pub.publish(MovePosition);
				not_found = true;
			}
			
		}
		
	ros::spinOnce();
	loop_rate.sleep();	
	}

	return 0;
}


void CameraCallBack(const geometry_msgs::Point ObjectPosition)
{
  not_found = false;
  if(ObjectPosition.z != -1)
  {
  	ActualObjectPosition.z = ObjectPosition.z/(1000);
  	ActualObjectPosition.x = (ActualObjectPosition.z)*(ObjectPosition.x - 159.5)/(285);
  	ActualObjectPosition.y = (ActualObjectPosition.z)*(ObjectPosition.y - 119.5)/(285);
  
  
  	MovePosition.position.x = 0;
  	MovePosition.position.y = 0;
  	MovePosition.position.z = 0;
  
  	MovePosition.position.x = 0.2  * ActualObjectPosition.x;  
 	MovePosition.position.z = 0.2  * ActualObjectPosition.y;
  	MovePosition.position.y = 0.12 * (ActualObjectPosition.z - 1);
  }
	
}


void velCallback(const geometry_msgs::TwistStamped vel) // when it is in velocity control the take off is not complete yet
{
  take_off_done = false;
}

void spCallback(const geometry_msgs::PoseStamped sp)    // when it is in position control the take off is finished
{	
  take_off_done = true;
}

