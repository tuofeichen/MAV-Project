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
#define K 1
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
bool stages[] = { false, false, false};
float angle_rad=-5000;
float height = 1.0;
obj_test::JoyCommand MovePosition;
obj_test::MoveCommand cmd;

void velCallback(const geometry_msgs::TwistStamped vel);
void spCallback(const geometry_msgs::PoseStamped sp);
void wallCallback(const geometry_msgs::Point ang);
void poseCallback(const geometry_msgs::PoseStamped pos_read);
void hover();
void down(double distance);
bool findWall();
float distance=0;




int main(int argc, char **argv) 
{
	ros::init(argc, argv, "PerpWall");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	
	ros::Publisher move_pub = nh.advertise<obj_test::JoyCommand>("/april/cmd_mav", 100); 
	ros::Publisher cmd_pub = nh.advertise<obj_test::MoveCommand>("/mov_cmd", 100);
	ros::Subscriber sp_sub = nh.subscribe("/mavros/setpoint_position/local", 100, &spCallback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/setpoint_velocity/cmd_vel", 100, &velCallback);	
	ros::Subscriber wall_sub = nh.subscribe("/PerpendicularWallFind", 100, &wallCallback);	
	
	
	
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
			
			if(fabs(pos_read_.pz - height) < 0.2)
			{
				stages[2] = true;
				hover();
				move_pub.publish(MovePosition);
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			}
			else
			{
				down(0.1);
				cmd_pub.publish(cmd);
			}
			
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
}




void spin()
{
	MovePosition.position.x = 0;
    MovePosition.position.y = 0;
    MovePosition.position.z = 0;
    MovePosition.yaw = 1;
}




void wallCallback(const geometry_msgs::Point ang) 
{	
	angle_rad = ang.x;	
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



void down(double distance)
{
	cmd.distance = distance;
	cmd.code = 4;	
}




bool findWall()
{	
	if(angle_rad==-5000)
			{
				ROS_INFO("Spinning!");						// if the depth values are bigger than threshold (3500), keep spinning
			 	MovePosition.position.x = 0;
            	MovePosition.position.y = 0;
             	MovePosition.position.z = 0;
             	MovePosition.yaw = 1;
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
				}
				else
				{
					ROS_INFO("Wall is perpendicular now.");   // drone is perpendicular, hover now
					MovePosition.position.x = 0;
             		MovePosition.position.y = 0;
             		MovePosition.position.z = 0;
             		MovePosition.yaw = 0;
					return true;
				}
			}
}

	
	
	
	
	









