#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "Frame.h"
#include "AsusProLiveOpenNI2.h"
#include "obj_test/JoyCommand.h"
#include "obj_test/MoveCommand.h"

////////////////////////////////////////
// test 4
//
// takes off
// spin arround
// 
// if perpendicular wall is seen, go towards it until distance d1, hover
//
// end
//////////////////////////////////////

obj_test::JoyCommand MovePosition;
obj_test::MoveCommand cmd;

void wallCallback(const geometry_msgs::Point ang);
void CheckForWall();
//bool wall_found = false;




int main(int argc, char **argv) 
{
	ros::init(argc, argv, "ObjectFind");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);
	
	ros::Publisher move_pub = nh.advertise<obj_test::JoyCommand>("/april/cmd_mav", 100); // publish how much the drone should move
	ros::Publisher cmd_pub = nh.advertise<obj_test::MoveCommand>("/mov_cmd", 100);
	ros::Subscriber sp_sub = nh.subscribe("/mavros/setpoint_position/local", 100, &spCallback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/setpoint_velocity/cmd_vel", 100, &velCallback);	
	ros::Subscriber wall_sub = nh.subscribe("/PerpendicularWallFind", 100, &wallCallback);	
	
	while(ros::ok())
	{
		
		if(take_off_done)
		{
			
			if(!CheckForWall())
			{
				 move_pub.publish(MovePosition);
			}
			else
			{				
				GO_TO_WALL;
			}			
			
		}
		
	ros::spinOnce();
	loop_rate.sleep();	
	}

	return 0;
}


void wallCallback(const geometry_msgs::Point ang) 
{	
	angle_rad = ang.x;	
}





void CheckForWall()
{	
	if(angle_rad==-5000)
			{
				ROS_INFO("Spinning!");						// if the depth values are bigger than threshold (3500), keep spinning
			 	MovePosition.position.x = 0;
            	MovePosition.position.y = 0;
             	MovePosition.position.z = 0;
             	MovePosition.yaw = 1;
			 	move_pub.publish(MovePosition);
				return false;
				
			}
			else
			{
				if(fabs(angle_rad) - 0.05  > 0 )					// P controller to adjust drone to a found wall
				{
					ROS_INFO("Wall Found. Adjusting");
			 		MovePosition.position.x = 0;
             		MovePosition.position.y = 0;
             		MovePosition.position.z = 0;
             		MovePosition.yaw = K*angle_rad*PI;
					return false;
			 		move_pub.publish(MovePosition);
				}
				else
				{
					ROS_INFO("Wall is perpendicular now.");   // drone is perpendicular, hover now
					MovePosition.position.x = 0;
             		MovePosition.position.y = 0;
             		MovePosition.position.z = 0;
             		MovePosition.yaw = 0;
			 		move_pub.publish(MovePosition);
					
					return true;
					
				}
			}
	
	
			
	
}

	
	
	
	
	
	
	
	
	
	
	
	
	