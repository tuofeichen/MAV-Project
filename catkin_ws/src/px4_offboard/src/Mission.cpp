
#include "px4_offboard/Mission.h"

// flight logic / state machine handler

enum {takingoff, calibration, traverse, tracking, turning, landing};

Mission::Mission()
{
	_flight_mode  = takingoff; // start with takeoff
	_flight_mode_prev = takingoff;

	_is_takeoff   = 0;
	_is_land = 0;
	_is_calibrate = 0;
	_is_fail = 0;

	_safe_dist = 1400; // mm 
	_obst_found = 0;
	_wall_cnt = _obj_cnt = 0;
	_cannot_find_wall_cnt = 0;
	_roll = _pitch = _yaw = 0;

	_lpe.setIdentity();

	resetCommand(_emptyCommand); // hover command

	_mission_state_pub = _nh.advertise<px4_offboard::CtrlState> ("/obj/main_state",100);
	_mission_ctrl_pub = _nh.advertise<px4_offboard::JoyCommand> ("/obj/cmd_mav",100);

	_state_sub = _nh.subscribe("/px4_offboard/state",  10,  &Mission::stateCallback, this);
	_lpe_sub   = _nh.subscribe("/mavros/local_position/pose", 100, &Mission::lpeCallback,this);

	_wall_sub  = _nh.subscribe("/objDetect/wall_pose", 100, &Mission::wallCallback, this);
	_obj_sub   = _nh.subscribe("/objDetect/obj_pose",   100, &Mission::objCallback, this);
	_obst_sub  = _nh.subscribe("/objDetect/obst_dist",100,&Mission::obstCallback,this);

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_planner");
  int printDelay = 0;
  int printMod = 1000;
  Mission mission;

  ros::Rate loop_rate(200);

  while(ros::ok())
  {

  		ros::spinOnce();
  		
  		printDelay++; 
  		if (mission.getFailFlag())
  		{
  			ROS_INFO("[Mission] Failure detected");
  			return 0; // override mission command
  		}
		
		switch(mission.getFlightMode())
	  	{
	  		case takingoff:
	  			if (mission.getTakeoffFlag())
	  			{
	  				ROS_INFO("[Mission] Finished taking off");
	  				// mission.setFlightMode(landing);
					mission.setFlightMode(calibration);
		  		}
		  		else
		  		{
		  			if (!(printDelay%printMod))
	  					ROS_INFO("[Mission] Takeoff mode");
		  			mission.takeoff();
		  		}

	  		break;
	  			
	  		case calibration:
	  			if (!(printDelay%printMod))
	  				ROS_INFO("[Mission] Calibration mode");

	  			if(!mission.getCalibrateFlag()) // then wait for state transition
		  			mission.publish();

	  		break;

	  		case traverse:
	  		    if (!(printDelay%printMod))
	  				ROS_INFO("[Mission] Traversing mode");
		  		mission.publish();
	  		break;
	  		
	  		case tracking:
	  		    if (!(printDelay%printMod))
	  				ROS_INFO("[Mission] Tracking mode");
		  		mission.publish();
	  		break;

	  		case turning:
	  			if (!(printDelay%printMod))
	  				ROS_INFO("[Mission] Turning mode");

	  			if(mission.turnLeft90()) // if finished turning
	  				mission.setFlightMode(landing);
		  			// mission.setFlightMode(mission.getPrevFlightMode()); // go back to previous flight mode
		  		else
		  			mission.publish();
		  			
		  	break; 
	  		
	  		case landing:

	  		    if(mission.getLandFlag())
	  		    {
	  		    	ROS_INFO("[Mission] Landed");
	  		    	return 0;
	  		    }
	  		    else if(!(printDelay%printMod))
	  		    {
	  		    	
		  			ROS_INFO("[Mission] Landing mode");
	  		    }
	  		    mission.land();
	  			
	  		break;
	  	}

	  	loop_rate.sleep();
  }
}


void Mission::takeoff()
{
	resetCommand(_objCommand);
	_objCommand.takeoff = 1;
	_objCommand.arm = 1;
 	_objCommand.offboard = 1;
	_mission_ctrl_pub.publish(_objCommand);
};

void Mission::land()
{
	resetCommand(_objCommand);
	_objCommand.takeoff = 0;
	_objCommand.land =1 ;
	_mission_ctrl_pub.publish(_objCommand);
}

bool Mission::turnLeft90()
{
	resetCommand(_objCommand); 
	// if _yaw > M_PI see if we need something here for wrap around

	_objCommand.yaw = 0.5 * fabs(_yaw - _yaw_prev - 0.5*M_PI);
	// ROS_INFO("What's yaw right now? %f", _yaw);


	return (fabs(_yaw - _yaw_prev - 0.5 * M_PI) < 0.05); // true if finished turning 
}

void Mission::objCallback(const geometry_msgs::Point pos)
{
	_obj_cnt ++ ;
	const float Kp = 0.5; // tracking gain
	if ((_obj_cnt > 10)&&(_flight_mode != landing) && (_flight_mode!=takingoff))
	{
		setFlightMode(tracking);
		// _flight_mode = tracking;
		
		resetCommand(_objCommand);
		_objCommand.position.x = Kp * (pos.z - 1000.0) / 1000.0; // keep 1m distance away from target
		_objCommand.position.y = Kp * (pos.x - 160.0)  / 200.0;  
		_objCommand.position.z = Kp * (pos.y - 120.0)  / 200.0;

		ROS_INFO("Current Set Point: x %f y %f", abs(pos.x-160), abs(pos.y-120));

		if ((pos.z < 1100) && (abs(pos.x-160) < 35 )&& (abs(pos.y-120)< 35)) //center arranged
		{
			resetCommand(_objCommand);
			_objCommand.land = 1;
			setFlightMode(landing);

			// _flight_mode = landing;
		}
	}

}


void Mission::wallCallback(const geometry_msgs::Point ang)
{
	const float K = 0.5; //listen to wall if in calibration mode

	if((_flight_mode == calibration) && (!_is_calibrate))
	{
		double angle_rad = ang.x;

		if(angle_rad == -5000 || angle_rad > 0.785 || angle_rad < -0.785)
		{
				ROS_INFO("Cannot find wall");						// if the depth values are bigger than threshold (3500), keep spinning
				_cannot_find_wall_cnt++;
				if(_cannot_find_wall_cnt >100) // if cannot find wall for too long land
				{
					resetCommand(_objCommand);
					setFlightMode(landing);
					_objCommand.land = 1;
				}
				
		}
		else 
		{
			_cannot_find_wall_cnt = 0; 
			if( (fabs(angle_rad) > 0.15) && (!_is_calibrate) )					// P controller to adjust drone to a found wall
			{
				ROS_INFO("Wall Found. Adjusting... %f", angle_rad);
				resetCommand(_objCommand);
				_objCommand.yaw = K*angle_rad;
				_wall_cnt = 0;
			}
			else
			{
				_wall_cnt++;
				_objCommand.position.x = (ang.z- _safe_dist)/1000.0; // move to 1.5 from obstacles

				if((_wall_cnt > 10) && (fabs( ang.z - _safe_dist) < 20))
				{
					ROS_INFO("Wall is perpendicular now.");   // drone is perpendicular, hover now	
					if (!_is_calibrate)
					{
						_is_calibrate = 1;
						_yaw_prev = _yaw;
						ROS_WARN("Yaw pin down is %f, z pin down is %f", _yaw_prev,_lpe(2,3));
						resetCommand(_objCommand);					
						_objCommand.position.z = - 0.5; // go down to around 1 m
						_mission_ctrl_pub.publish(_objCommand);						
					}  // issue this setpoint once

				}
				
			}
		}
	}
	else if ((_flight_mode == calibration) && (_lpe(2,3) < 1)) // after going down: turn 90 degrees
	{ 
		ROS_INFO("Start turning after going down. What's the height right now? %3.2f", _lpe(2,3));		
		setFlightMode(traverse); // so that after turning it will return to traverse rather than calibration
		setFlightMode(turning);

		// if (turnLeft90()) // should be directional
		// {
		// 	ROS_INFO("Turned 90 and ready for traversal");
		// 	resetCommand(_objCommand); // go to 
		// }
	}

};

void Mission::obstCallback(geometry_msgs::Point msg)
{
	if (_flight_mode == traverse) // maybe want some threshold in case wrong depth occur
	{
		if (msg.x > 10) // need to do something
		{
			
			resetCommand(_objCommand);
			if (msg.y < 800) 		// failsafe case land regardless (minimum distance is too small)
			{
				setFlightMode(landing);
				ROS_WARN("obstacles too close detected. Land");
				_objCommand.land = 1;
			}

			else if (msg.z < _safe_dist) 
			{
				if (!_obst_found)
				{
					_obst_found = 1;
					ROS_INFO("Find an obstacle: Turning 90 degrees"); // 
					_obst_cnt ++ ; 
					_yaw_prev = _yaw; 
					// _flight_mode = turning;
					setFlightMode(turning);
				}


				if ((_obst_cnt == 3) &&( _safe_dist < 2000)) // one cycle
				{
					ROS_INFO("go to an inner layer");
					_obst_cnt = 0; // reset obst cnt
					_safe_dist += 500; // traverse inner circle 
				}
			}
		}
		else
		{
			resetCommand(_objCommand);
			_obst_found = 0;
			_objCommand.position.x = 1; // go forward
		}
	}

}

void Mission::lpeCallback(const geometry_msgs::PoseStamped pos_read)
{

	Eigen::Quaternionf q (pos_read.pose.orientation.w, pos_read.pose.orientation.x, pos_read.pose.orientation.y,pos_read.pose.orientation.z);
	_lpe.setIdentity(); 	// clear buffer
	_lpe.topLeftCorner (3,3) = q.matrix(); 
	_lpe.topRightCorner(3,1) << pos_read.pose.position.x, pos_read.pose.position.y, pos_read.pose.position.z;
	rot2rpy(q.matrix(), _roll, _pitch, _yaw);

}
void Mission::resetCommand(px4_offboard::JoyCommand& command)
{
	command.takeoff = 0;
	command.land = 0;
	command.arm = 1;
	command.offboard = 1;
	command.position.x = 0;
	command.position.y = 0;
	command.position.z = 0;
	command.yaw = 0;
}

inline void Mission::rot2rpy(Matrix3f R,float& r, float& p, float& y)
{
	float beta, alpha, gamma;
	beta = atan2(-R(2,0), sqrt(R(0,0)*R(0,0)+R(1,0)*R(1,0)));
	
	if(fabs(beta - M_PI/2) < 0.001){
	alpha = 0;
	gamma = atan2(R(0,1),R(1,1));
	}
	else if (fabs(beta + M_PI/2) < 0.001){
	alpha = 0;
	gamma = -atan2(R(0,1),R(1,1));
	}
	else{
	alpha = atan2(R(1,0), R(0,0));
	gamma = atan2(R(2,1), R(2,2));
	}
	
	y = alpha ;
	p = beta  ;
	r = gamma ;
}

// inline bool Mission::turnLeft90() // maybe do something? 
// {

// }

// takes off really high
// see the wall
// get perpendicular to the wall
// go towards it
//
// go to apropriate height1
//
//
// <change_height>
//
// <walk arround>
// turn 90
// go towards 2nd wall until distance d(i)
// 
// turn 90
// go towards 3nd wall until distance d(i)
//
// turn 90
// go towards 4nd wall until distance d(i)
//
// turn 90
// go towards 1st all until distance d(i)
//
// go to <walk arround>
//
// if walked all possible distances go to change_height
//
