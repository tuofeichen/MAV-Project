#include "px4_offboard/include.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include "px4_offboard/JoyCommand.h"

//////////////////////
// check if the drone already took off
// if object if found in the image => get its position in relation to the drone
//								   => publish (x,y)[position of the object]  as the new position of the drone
//


geometry_msgs::Point ActualObjectPosition;
px4_offboard::JoyCommand MovePosition;
bool take_off_done=false;


void CameraCallBack(const geometry_msgs::Point ObjectPosition);
void velCallback(const geometry_msgs::TwistStamped vel);
void spCallback(const geometry_msgs::PoseStamped sp);


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "ObjectFind");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Publisher state_pub = nh.advertise<obj_test::JoyCommand>("/april/cmd_mav", 100); // publish how much the drone should move
	ros::Subscriber camera_sub = nh.subscribe("/FindObjectSub", 100, &CameraCallBack); // get target position
	ros::Subscriber sp_sub = nh.subscribe("/mavros/setpoint_position/local", 100, &spCallback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/setpoint_velocity/cmd_vel", 100, &velCallback);
		
	while(ros::ok())
	{
		
		if(take_off_done)
		{
			if(found){
			ROS_INFO("Object Found!");
			found = false;
			move_pub.publish(MovePosition);
			}
			else{
			ROS_INFO("Spinning!");
			 MovePosition.position.x = 0;
             MovePosition.position.y = 0;
             MovePosition.position.z = 0;
             MovePosition.yaw = 1;
			 move_pub.publish(MovePosition);		
			}
			
		}
		
	ros::spinOnce();
	loop_rate.sleep();	
	}

	return 0;
}



void CameraCallBack(const geometry_msgs::Point ObjectPosition)
{
  false = true;
  if(ObjectPosition.z != -1){
  ActualObjectPosition.z = ObjectPosition.z/(1000f);
  ActualObjectPosition.x = (ActualObjectPosition.z)*(ObjectPosition.x - 159.5f)/(285f);
  ActualObjectPosition.y = (ActualObjectPosition.z)*(ObjectPosition.y - 119.5f)/(285f);
  
  
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

