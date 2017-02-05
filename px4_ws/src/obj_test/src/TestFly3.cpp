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
#include "std_msgs/Float32.h"

#define PI 3.14159
#define K 0.5

////////////////////////////////////////
// test 3
//
// takes off
// spin arround
// 
// if perpendicular wall is seen, hover
//
// end
//////////////////////////////////////

obj_test::MoveCommand cmd;
float angle_rad=-5000;



void wallCallback(const geometry_msgs::Point ang);

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "WallFind");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	int counter = 0;
	int counter2 = 0;
	int counterLand = 0;
	
	  // publish how much the drone should move
	ros::Publisher cmd_pub = nh.advertise<obj_test::MoveCommand>("/mov_cmd", 100); 
	ros::Subscriber wall_sub = nh.subscribe("/PerpendicularWallFind", 100, &wallCallback);	// receives angle of the drone relative to wall
																						   //  0 is perpendicularc
	
	

	while(ros::ok())
	{
		std::cout << "angle degrees: " << angle_rad*180/PI<<std::endl; 
		

		
			if(angle_rad==-5000 || angle_rad > 0.785 || angle_rad < -0.785)
			{
//				ROS_INFO("Hovering!");						// if the depth values are bigger than threshold (3500), keep spinning
				if(counter==0){	
				cmd.distance = 0;
				cmd.code = 6;
			 	cmd_pub.publish(cmd);
				counter++;
				counter2=0;
				counterLand=0;
						}
				
			}
			else
			{
				counter = 0;
				if(fabs(angle_rad) - 0.06  > 0 )					// P controller to adjust drone to a found wall
				{
//					ROS_INFO("Wall Found. Adjusting");
			 		
					cmd.distance = K*angle_rad;
					cmd.code = 2; 
			 		cmd_pub.publish(cmd);
					counter2=0;
					counterLand=0;
				}
				else
				{
					if(counter2 == 0 && counterLand>=20){
			//		ROS_INFO("Wall is perpendicular now.");   // drone is perpendicular, hover now
					cmd.distance = 0;
					cmd.code = 5;
			 		cmd_pub.publish(cmd);
					counter2++;
					}

					counterLand++;
					//return 0;
					
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




