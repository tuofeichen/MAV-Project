
#define MAX_DISTANCE_MEASUREMENT 2.5f // max distance the depth camera can acquire (2.5meters)

boolean is_takeoff = false; 
obj_test::MoveCommand cmd;
float distances[] = {1,2,3,4,5}; // distances have to be lower than MAX_DISTANCE_MEASUREMENT
float heights[] = {5,4,3,2};     // heights have to be bigger than 0.2m
float distance_to_ground = 0;
float distance_to_wall = MAX_DISTANCE_MEASUREMENT; // dont know how to acquire that
						// check the depth value of the central pixel maybe?
float distance_to_wall_instantaneous = MAX_DISTANCE_MEASUREMENT;

void spCallback(const geometry_msgs::PoseStamped sp);	// check if the drone already took off
void velCallback(const geometry_msgs::TwistStamped vel);
void CameraCallBack(const geometry_msgs::Point TargetPosition);
poseCallback(const geometry_msgs::PoseStamped pos_read);

														// controller of the drone
void MoveToWall(float d);
void poseCallback(const geometry_msgs::PoseStamped pos_read);
void down(float d);
void start_round();
void RoundTheRoom(float d);
void Turn90();
void TurnMinus90();
void MoveFirst(float d);
void MoveRight(float d);


//////////////////////////////////////
// msg								//
// 		: float distance			//
//		: int code					//
//									//
//									//
// code								//
//		0: moveFoward(distance)		//
//		1: moveRight(distance)		//
//		2: yawRight(1.57f)			//
//		3: yawLeft(1.57f)			//
//		4: moveDown(distance)		//
//		5: land						//
//////////////////////////////////////


int main(int argc, char **argv)
{
	ros::init(argc, argv, "FindObjectInARoom");
	ros::NodeHandle nh;
	ros::Publisher cmd_pub = nh.advertise<obj_test::MoveCommand>("/mov_cmd", 100);
	ros::Subscriber find_object_sub = nh.subscribe("/FindObject", 100, &CameraCallBack);
	ros::Subscriber sp_sub = nh.subscribe("/mavros/setpoint_position/local", 100, &spCallback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/setpoint_velocity/cmd_vel", 100, &velCallback);
	
	
	// takes off from the center of the room (it is done by the controller)
	
	// get perpendicular to the wall 
		// how?
	
	start_round();	// facing a wall perpendiculary, the drones moves forward, turns right
	
	
	for(int i = 0; i < heights.size(); i++)
	{
		
		for(int j = 0; j < distances.size(); j++)
			{
				RoundTheRoom(distances[j]);			
			}
		
		down(distance_to_ground - heights[i]);		
	}
	

	
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void spCallback(const geometry_msgs::PoseStamped sp)
{	
  is_takeoff = true;
}


void velCallback(const geometry_msgs::TwistStamped vel)
{
  is_takeoff = false;
}


void MoveFirst(float d)
{
	// get distance_to_wall
	distance_to_wall_instantaneous = distance_to_wall;	
	
	while(distance_to_wall_instantaneous > d) // has to put a test to see if the drone is moving
		{
			cmd.distance = 0;
			cmd.code = -1;
			
			cmd.distance = distance_to_wall_instantaneous - d;
			cmd.code = 0;
			
			cmd_pub.publish(cmd);
		}	
}


void MoveRight(float d)
{
	// reset values
	cmd.distance = 0;
	cmd.code = -1;
	
	while(distance_to_wall_instantaneous > d && distance_to_wall_instantaneous > MAX_DISTANCE_MEASUREMENT)
	{
		
		// move right X distance
		cmd.distance(distance_to_wall_instantaneous - d);
		cmd.code = 1;
		cmd_pub.publish(cmd); // need a way to check if the whole distance is travelled before starting the next command
		
		// yaw 90degrees
		Turn90();
		
		// get distance_to_wall here
		distance_to_wall_instantaneous = distance_to_wall;
		
		// yaw -90degrees
		TurnMinus90();
		

	}
}


void Turn90() // still have to implement a way to wait until the complete 90degree rotation is done
{
	cmd.distance = 0;
	cmd.code = -1;
		
	cmd.distance = 0;
	cmd.code = 2;
	cmd_pub.publish(cmd);
}

void TurnMinus90() // still have to implement a way to wait until the complete 90degree rotation is done
{
	cmd.distance = 0;
	cmd.code = -1;
	
	cmd.distance = 0;
	cmd.code = 3;
	cmd_pub.publish(cmd);
}


void RoundTheRoom(float d)
{
	Turn90();
	Turn90();
	// get distance to wall (distance_to_wall)
	distance_to_wall_instantaneous = distance_to_wall;
	TurnMinus90();
	MoveRight(d);
	
	Turn90();
	Turn90();
	// get distance to wall (distance_to_wall)
	distance_to_wall_instantaneous = distance_to_wall;
	TurnMinus90();
	MoveRight(d);
	
	Turn90();
	Turn90();
	// get distance to wall (distance_to_wall)
	distance_to_wall_instantaneous = distance_to_wall;
	TurnMinus90();
	MoveRight(d);
	
	Turn90();
	Turn90();
	// get distance to wall (distance_to_wall)
	distance_to_wall_instantaneous = distance_to_wall;
	TurnMinus90();
	MoveRight(d);	
}


void start_round()
{
	MoveFirst(distances[0]);
	Turn90();
	// get distance to wall (distance_to_wall)
	distance_to_wall_instantaneous = distance_to_wall;
	
	TurnMinus90();
	MoveRight(distances[0]);
}

void down(float d)
{
	cmd.distance = 0;
	cmd.code = -1;
	
	cmd.distance = d;
	cmd.code = 4;
	cmd_pub.publish(cmd);
}

void poseCallback(const geometry_msgs::PoseStamped pos_read) 
{
	 distance_to_groud = pos_read.pose.position.z;
}

void CameraCallBack(const geometry_msgs::Point TargetPosition) 
{
	cmd.distance = 0;
	cmd.code = -1;
	
	cmd.distance = 0;
	cmd.code = 5; 
	cmd_pub.publish(cmd)
}
/////////////////////////////////////////
/////////////////////////////////////////
// Put in CtrlPx4.cpp
/*
void CtrlPx4::FindObjectCallback(const px4_offboard::MoveCommand move_cmd)
{
 
 switch(move_cmd.code){
	 
	case 0:
		forward(move_cmd.distance);
	break;	 
	
	case 1:
		right(move_cmd.distance);
	break;
	 
	case 2:
		yawRight(1.57f);
	break;
	
	case 3:
		yawLeft(1.57f);
	break;
	
	case 4: 
		down(move_cmd.distance);
	break;
	
	case 5:
		land(0.5f);
	break;
	
	default:
		ROS_INFO("Code input error ! ");
	break;
	 
 }

}
*/
