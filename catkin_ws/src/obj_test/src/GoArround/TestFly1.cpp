#include "px4_offboard/include.h"
#include "px4_offboard/MoveCommand.h"

// test 1

// takes off
// spin arround
// 
// if object if seen, land
//
// end


boolean is_takeoff_done = false; 
px4_offboard::MoveCommand cmd;

void spin();
void spCallback(const geometry_msgs::PoseStamped sp);
void velCallback(const geometry_msgs::TwistStamped vel);
void CameraCallBack(const geometry_msgs::Point TargetPosition) 



int main(int argc, char **argv)
{
	ros::init(argc, argv, "SpinAndFindAnObject");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Publisher cmd_pub = nh.advertise<obj_test::MoveCommand>("/mov_cmd", 100);
	ros::Subscriber find_object_sub = nh.subscribe("/FindObject", 100, &CameraCallBack);
	ros::Subscriber sp_sub = nh.subscribe("/mavros/setpoint_position/local", 100, &spCallback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/setpoint_velocity/cmd_vel", 100, &velCallback);
	
	
	while(ros::ok())
	{
		if(is_takeoff_done)
		{
			spin();
		}
	
	
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}

void spCallback(const geometry_msgs::PoseStamped sp)
{	
  is_takeoff_done = true;
}


void velCallback(const geometry_msgs::TwistStamped vel)
{
  is_takeoff_done = false;
}

void spin(){
	cmd.distance = (3.14/4);
	cmd.code = 0;
}

void CameraCallBack(const geometry_msgs::Point TargetPosition) 
{
	ROS_INFO("Object was detected in the scene.");
	cmd.distance = 0;
	cmd.code = -1;
	
	cmd.distance = 0;
	cmd.code = 5; 
	cmd_pub.publish(cmd); // stop moving and then hover
}