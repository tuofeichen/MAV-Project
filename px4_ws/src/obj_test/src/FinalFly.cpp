#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <math.h>
#include <Eigen/Dense>
#include <sstream>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include "Frame.h"
#include "AsusProLiveOpenNI2.h"


#include "obj_test/JoyCommand.h"
#include "obj_test/MoveCommand.h"

#define PI 3.14


typedef struct pos_s {
  double px; 
  double py;
  double pz;
  double roll;
  double pitch;
  double yaw;
  Eigen::Vector4f q; 
} my_pos;




bool stages[] = { false, false, false, false, false, false, false, false, false };
std::vector<int> distances = {1};
std::vector<double> heights = {1};
int corners = 0;
int distance_counter = 0;
int height_counter = 0;
double depth_from_central_pixel = -1;
double depth_from_right_most_pixel = -1;
int yaw_counter = 0;
double initial_yaw = 0;
bool turn_stage = false;
bool wall_stage = true;
bool finish = false;
float angle_rad=-5000;
obj_test::JoyCommand MovePosition;
obj_test::MoveCommand cmd;
my_pos pos_read_;

void velCallback(const geometry_msgs::TwistStamped vel);
void spCallback(const geometry_msgs::PoseStamped sp);
void wallCallback(const geometry_msgs::Point ang);
void poseCallback(const geometry_msgs::PoseStamped pos_read);
void CameraCallBack(const geometry_msgs::Point TargetPosition);
void hover();
void spin();
void down(double distance);
void forward(double distance);
void yawRight(double distance);
bool findWall();



int main(int argc, char **argv) 
{
	ros::init(argc, argv, "FindObjectInARoom");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);
	
	
	ros::Publisher move_pub = nh.advertise<obj_test::JoyCommand>("/april/cmd_mav", 100); 
	ros::Publisher cmd_pub = nh.advertise<obj_test::MoveCommand>("/mov_cmd", 100);
	ros::Subscriber find_object_sub = nh.subscribe("/FindObjectSub", 100, &CameraCallBack);
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
	
	// first stage - take off
		if(!stages[0])
		{
			ROS_INFO("Stage 1 - Taking off");
		}

		
		
	// second stage - find perpendicular wall
		if(stages[0] && !stages[1])
		{
			ROS_INFO("Stage 2 - Finding perpendicular wall");
			
			if( findWall() )
			{
				stages[1] = true;
				hover();
				move_pub.publish(MovePosition);
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			}
 			else
			{
				move_pub.publish(MovePosition);
			}
			
		}
		
		
		
		
	// third stage - go down 
		if(stages[1] && !stages[2])
		{
			ROS_INFO("Stage 3 - Going to appropriate height");
			
			if(pos_read_.pz - heights[height_counter] < 10)
			{
				stages[2] = true;
				hover();
				move_pub.publish(MovePosition);
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			}
			else
			{
				down(0.5);
				move_pub.publish(MovePosition);
			}
			
		}
		
		
		
		
	// fourth stage - go torwards perpendicular wall
		if(stages[2] && !stages[3])
		{
			ROS_INFO("Stage 4 - Going torwards perpendicular wall");
			
			if (depth_from_central_pixel - distances[0] < 10)
			{
				stages[3] = true;
				hover();
				move_pub.publish(MovePosition);
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			}
			else
			{
				forward(0.5);
				move_pub.publish(MovePosition);
			}
		}
		
		
		
		
	// fifth stage - turn 90degrees
		if(stages[3] && !stages[4])
		{
				ROS_INFO("Stage 5 - Turning 90 degrees");
			
				if(yaw_counter==0) initial_yaw =  pos_read_.yaw;
				yaw_counter++;
			
				ROS_INFO("initial yaw: %f, current yaw: %f", initial_yaw, pos_read_.yaw);
			
				if(fabs(pos_read_.yaw - initial_yaw - 3.14/2) < 3.14/100)
				{
					stages[4] = true;
					hover();
					move_pub.publish(MovePosition);
					boost::this_thread::sleep(boost::posix_time::milliseconds(500));
				}
				else
				{
					yawRight(3.14/20);
					move_pub.publish(MovePosition);
				}
		}
		
		
		
		
	// sixth stage - go forward until distance d
		if(stages[4] && !stages[5])
		{
				ROS_INFO("Stage 6 - going forward until distance d");	
			
				if (depth_from_central_pixel - distances[0] < 10)
				{
					stages[5] = true;
					hover();
					move_pub.publish(MovePosition);
					boost::this_thread::sleep(boost::posix_time::milliseconds(500));
				}
				else
				{
					forward(0.5);
					move_pub.publish(MovePosition);
				}
					
		}
		
		
		
		
	//	seventh stage - round the room
		if(stages[5] && !stages[6])
		{
				ROS_INFO("Stage 7 - Rounding the room");
			
				if(yaw_counter==0) initial_yaw =  pos_read_.yaw;
			
			    yaw_counter++;
				if(!turn_stage)
				{
					if(fabs(pos_read_.yaw - initial_yaw - 3.14/2) < 3.14/100)
					{
						turn_stage = true;
						wall_stage = false;
						corners++;
					}
					else
					{
						yawRight(3.14/20.);
						move_pub.publish(MovePosition);
					}
					
				}
			
			
				if(!wall_stage)
				{
					if(depth_from_central_pixel - distances[0] < 10)
					{
						corners++;
						wall_stage = true;
						turn_stage = false;
						//initial_yaw = current_yaw;
						yaw_counter = 0;
					}
					else
					{
						forward(0.5);
					}
					
				}
			
				if(corners>=8)
				{
					corners=0;
					distance_counter++;
				}
			
			
				if(distance_counter > distances.size() - 1) stages[6] = true;
						
		}
	
		
		
		
		
	// eighth stage - go to next height
		if(stages[6] && !stages[7])
		{
			ROS_INFO("Stage 8 - Going to next height");

			if(pos_read_.pz - heights[height_counter+1] < 10)
			{
				stages[7] = true;
			}
			else
			{
				down(0.5);
			}
		}
		
		
		
		
	// nineth stage - change height number
		if(stages[7] && !stages[8])
		{
			ROS_INFO("Stage 9 - Changing height");
			
			height_counter++;
			stages[8] = true;
			
			// reset stage 7,8 and 9
			stages[6] = false;
			stages[7] = false;
			stages[8] = false;
			
			
			if(height_counter > heights.size() - 1) 
			{
				ROS_INFO("Object not found");
				return 0;
			}
			
		}	
		
		if(finish)
		{   
			hover();
			cmd_pub.publish(cmd);
			return 0;
		}
		ros::spinOnce();
		loop_rate.sleep();	
	}

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////	
void velCallback(const geometry_msgs::TwistStamped vel) 
{
	stages[0] = false;
}

void spCallback(const geometry_msgs::PoseStamped sp)    
{	
	stages[0] = true;
}

void hover()
{
	cmd.distance = 0;
	cmd.code = 6;
	cmd_pub.publish(cmd);
	
}

void spin()
{
	MovePosition.position.x = 0;
    MovePosition.position.y = 0;
    MovePosition.position.z = 0;
    MovePosition.yaw = 1;
}


void down(double distance)
{
	ros::NodeHandle nh;
	ros::Publisher cmd_pub = nh.advertise<obj_test::MoveCommand>("/mov_cmd", 100);
	
	cmd.distance = distance;
	cmd.code = 4;
	cmd_pub.publish(cmd);	
}


void forward(double distance)
{
	ros::NodeHandle nh;
	ros::Publisher cmd_pub = nh.advertise<obj_test::MoveCommand>("/mov_cmd", 100);
	
	cmd.distance = distance;
	cmd.code = 0;
	cmd_pub.publish(cmd);	
}


void yawRight(double distance)
{
	ros::NodeHandle nh;
	ros::Publisher cmd_pub = nh.advertise<obj_test::MoveCommand>("/mov_cmd", 100);
	
	cmd.distance = distance;
	cmd.code = 2;
	cmd_pub.publish(cmd);
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
	
  ROS_INFO("yaw: %f", pos_read_.yaw);

}

void CameraCallBack(const geometry_msgs::Point TargetPosition) 
{
	if(stages[0])
	{
		finish = false;
	}
}


void wallCallback(const geometry_msgs::Point ang) 
{	
	angle_rad = ang.x;	
}


bool CheckForWall()
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
			 		move_pub.publish(MovePosition);
					return false;
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

	
	
	
	
	









