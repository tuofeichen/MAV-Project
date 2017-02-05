#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include "ros/ros.h"
#include <Eigen/Dense>
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
#include <math.h>

////////////////////////////////////////
// test 5
//
// takes off
// spin arround
// 
// if perpendicular wall is seen, go towards it until distance d1, turn 90
//
// end
//////////////////////////////////////

obj_test::JoyCommand MovePosition;
obj_test::MoveCommand cmd;
bool take_off_done=false;
bool not_found = true;
double depth_right_most_pixel=-1;
double depth_central_pixel=-1;
double initial_yaw;

typedef struct pos_s {
  double px; 
  double py;
  double pz;
  double roll;
  double pitch;
  double yaw;
  Eigen::Vector4f q; 
} my_pos;


my_pos pos_read_; 
void velCallback(const geometry_msgs::TwistStamped vel);
void spCallback(const geometry_msgs::PoseStamped sp);
void wallCallback(const geometry_msgs::Point pixel_depth);
void poseCallback(const geometry_msgs::PoseStamped pos_read);
	
void CheckForWall();
bool wall_found = false;
int first_yaw_counter = 0;






int main(int argc, char **argv) 
{
	ros::init(argc, argv, "GoToWallAndTurn");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);
	
	ros::Publisher move_pub = nh.advertise<obj_test::JoyCommand>("/april/cmd_mav", 100); // publish how much the drone should move
	ros::Publisher cmd_pub = nh.advertise<obj_test::MoveCommand>("/mov_cmd", 100);
	ros::Subscriber sp_sub = nh.subscribe("/mavros/setpoint_position/local", 100, &spCallback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/setpoint_velocity/cmd_vel", 100, &velCallback);	
	ros::Subscriber wall_sub = nh.subscribe("/PerpendicularWallFind", 100, &wallCallback);	
	ros::Subscriber pos_sub_ = nh.subscribe("/mavros/local_position/pose", 100, &poseCallback);
	
	
	
	pos_read_.px = 0;
	pos_read_.py = 0;
	pos_read_.pz = 0;
	pos_read_.roll = 0;
	pos_read_.pitch = 0;
	pos_read_.yaw = 0;
	pos_read_.q(0) = 0;
	pos_read_.q(1) = 0;
	pos_read_.q(2) = 0;
	pos_read_.q(3) = 0;
	
	
	
	while(ros::ok())
	{
		
		if(take_off_done)
		{
			CheckForWall();
			
			if(!wall_found)
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
				 ROS_INFO("Wall Found. Going torwards it.");
				 MovePosition.position.x = 0;
          	     MovePosition.position.y = 0;
         	     MovePosition.position.z = 0;
          	     MovePosition.yaw = 0;
				 move_pub.publish(MovePosition);
				 boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			 	 wall_found = true;	
				
				if(depth_central_pixel - 10 > 0)
				{
					ROS_INFO("depth: %f", depth_central_pixel);
					cmd.distance = depth_central_pixel - 10;
					cmd.code = 0;
					cmd_pub.publish(cmd);
				}
				else
				{
					ROS_INFO("You have reached the first wall!.");
					
					if(first_yaw_counter==0)
					{
						initial_yaw = pos_read_.yaw;
						first_yaw_counter++;
					}
					else
					{
						
						if((3.14 + initial_yaw - pos_read_.yaw) > 3.14/1000.)	
						{	
							ROS_INFO("Turning");
							cmd.distance = 3.14 - pos_read_.yaw;
							cmd.code = 2;
							cmd_pub.publish(cmd);
							//ROS_INFO("Initial yaw: %f, Current Yaw: %f", initial_yaw, pos_read_.yaw);
						}
						else
						{
							ROS_INFO("You have turned 90 degrees successfully");
							cmd.distance = 0;
							cmd.code = 6;
							cmd_pub.publish(cmd);
							boost::this_thread::sleep(boost::posix_time::milliseconds(300));	
							return 0;
						}
					}

				}
				
			}			
			
		}
		
	ros::spinOnce();
	loop_rate.sleep();	
	}

	return 0;
}


void velCallback(const geometry_msgs::TwistStamped vel) // when it is in velocity control the take off is not complete yet
{
  take_off_done = false;
}

void spCallback(const geometry_msgs::PoseStamped sp)    // when it is in position control the take off is finished
{	
  take_off_done = true;
}

void wallCallback(const geometry_msgs::Point pixel_depth)    // when it is in position control the take off is finished
{	
  depth_central_pixel = pixel_depth.x ;
  depth_right_most_pixel = pixel_depth.y ;
}

void CheckForWall()
{
	// 58 is the AOV
	
	//ROS_INFO("depth_central_pixel: %f",depth_central_pixel);
	ROS_INFO("%f,%f,%f,%f,%f", depth_right_most_pixel, cos(((58./2)/180.)*3.14) , depth_central_pixel,depth_right_most_pixel*cos(((58./2)/180.)*3.14) ,fabs(depth_right_most_pixel*cos(((58./2)/180.)*3.14)  - depth_central_pixel) );
	
	if(depth_central_pixel > 0 && depth_right_most_pixel > 0 && depth_right_most_pixel < 3500 && depth_central_pixel < 3500){
		
		if(fabs(depth_right_most_pixel*cos(((58./2)/180.)*3.14) - depth_central_pixel) < 10 )
		{
			ROS_INFO("hi");
			wall_found = true;
		}
	}
	
	
}


void poseCallback(const geometry_msgs::PoseStamped pos_read) {
  pos_read_.px = pos_read.pose.position.x;
  pos_read_.py = pos_read.pose.position.y;
  pos_read_.pz = pos_read.pose.position.z;

  pos_read_.q(0) = pos_read.pose.orientation.w; // qw
  pos_read_.q(1) = pos_read.pose.orientation.x; // qx
  pos_read_.q(2) = pos_read.pose.orientation.y; // qy
  pos_read_.q(3) = pos_read.pose.orientation.z; // qz

  pos_read_.roll =  atan2(2.0*(pos_read_.q[0]*pos_read_.q[1] + pos_read_.q[3]*pos_read_.q[2]), pos_read_.q[3]*pos_read_.q[3] + pos_read_.q[0]*pos_read_.q[0] - pos_read_.q[1] *pos_read_.q[1] - pos_read_.q[2]*pos_read_.q[2]);

  pos_read_.pitch = asin(-2.0*(pos_read_.q[0]*pos_read_.q[2] - pos_read_.q[3]*pos_read_.q[1]));

  pos_read_.yaw = atan2(2*(pos_read_.q[0] * pos_read_.q[3]+ pos_read_.q[1]*pos_read_.q[2]), \
  1-2*(pos_read_.q[3]*pos_read_.q[3]+pos_read_.q[2]*pos_read_.q[2]));

}

	
	
	
	
	
	
	
	
	
	
	
	
	