#include <unistd.h>
#include "px4_offboard/Mission.h"

// flight logic / state machine handler
int  crash_cnt = 0;
Mission::Mission()
{
	_flight_mode  = takingoff; // start with takeoff
	_flight_mode_prev = takingoff;
	setControlMode(POS); // start with position control
	setFlightMode(takingoff);

	_is_update = 0;
	_is_takeoff   = 0;
	_is_land = 0;
	_is_calibrate = 0;
	_is_fail = 0;


	_safe_dist = 880; // inital safe distance
	_obst_found = 0;
	_obst_cnt = 1; 		// start with 1 obstacles;
	_wall_cnt = _obj_cnt = _cali_cnt = 0;
	_cannot_find_wall_cnt = 0;
	_roll = _pitch = _yaw = 0;

	_lpe.setIdentity();
	_vel.setZero();
	resetCommand(_emptyCommand); // hover command

	_mission_state_pub = _nh.advertise<px4_offboard::MavState> ("/obj/main_state",100);
	_mission_ctrl_pub = _nh.advertise<px4_offboard::MavState>  ("/obj/cmd_mav",10);

	_state_sub = _nh.subscribe("/px4_offboard/state",  10,  &Mission::stateCallback, this);
	_lpe_sub   = _nh.subscribe("/mavros/local_position/pose", 100, &Mission::lpeCallback,this);
	_vel_sub   = _nh.subscribe("/mavros/local_position/velocity", 100, &Mission::velCallback,this);

	_wall_sub  = _nh.subscribe("/objDetect/wall_pose", 100, &Mission::wallCallback, this);
	_obj_sub   = _nh.subscribe("/objDetect/obj_pose",   100, &Mission::objCallback, this);
	_obst_sub  = _nh.subscribe("/objDetect/obst_dist",100,&Mission::obstCallback,this);

// write logger header
	logMissionSp.open("/home/odroid/MAV-Project/px4_ws/src/px4_offboard/mavSp.csv", std::ofstream::out | std::ofstream::trunc);
	logMissionSp << "time,flight mode,control,x,y,z,yaw,wall_angle" <<endl;

}




int main(int argc, char** argv)
{
	ros::init(argc, argv, "mission_planner");

	Mission mission;
	int printDelay  = 0;
	int printMod    = 1;
  int takeoff_set = 0; //takeoff flag only set once

  ros::Rate loop_rate(120);

  while(ros::ok())
  {

			if (mission.getFlightMode()!=traverse) // if outside of traverse mode use position control
				mission.setControlMode(POS);

			if (mission.getFlightMode()> takingoff && mission.getFlightMode() < landing)
			{
				if (mission.update()) // wait for update
				{
					mission.correctTraverseHeight();

					if(!(++printDelay%printMod)) // printing and logging
							mission.logSp();
					mission.publish(); // publish control command once
 					mission.hover();   // clear _objCommand
				}
			}

  		if (mission.getFailFlag())
  		{
  			ROS_INFO("[Mission] Failsafe detected");
  			return 0; // override mission command
  		}

// check main state machine switches
			switch(mission.getFlightMode())
	  	{
	  		case takingoff:
	  			if (mission.getTakeoffFlag())
	  			{
	  				ROS_INFO("[Mission] Finished taking off");
					  mission.setFlightMode(calibration);
		  		}
		  		else
		  		{
		  			// if (mission.getStateSwitchFlag())
	  				// 	ROS_INFO("[Mission] Takeoff mode");

						if (takeoff_set < 30) // ask for takeoff
		  			{
							takeoff_set++;
							mission.takeoff();
							mission.publish();
						}
						else
						{
							mission.hover(); //clear
						}
		  		}

	  		break;

	  		case calibration:
	  			  // if (mission.getStateSwitchFlag())
	  				//   ROS_INFO("[Mission] Calibration mode");

	  		break;

	  		case traverse:
	  		    // if (mission.getStateSwitchFlag())
	  				// 	ROS_INFO("[Mission] Traversing mode");

	  		break;

	  		case tracking:
	  		    // if (mission.getStateSwitchFlag())
	  				// 	ROS_INFO("[Mission] Tracking mode");
	  		break;

	  		case turning:
	  			// if (mission.getStateSwitchFlag())
	  			// 	ROS_INFO("[Mission] Turning mode");

	  			if(mission.turnLeft90()) {
		  			mission.setFlightMode(mission.getPrevFlightMode()); // go back to previous flight mode
					  ROS_INFO("[Mission]Finished turning !!");
					}

		  	break;

	  		case landing:
	  		    if(mission.getLandFlag())
	  		    {
	  		    	ROS_INFO("[Mission] Landed");
	  		    	return 0;
	  		    }
	  		    // else if (mission.getStateSwitchFlag())
	  		    // {
		  			// 	ROS_INFO("[Mission] Landing mode");
	  		    // }
	  		    mission.land();
						mission.publish();
	  		break;
	  	}

		ros::spinOnce(); // listen to subscriptions
	  loop_rate.sleep();
  }
}


void Mission::takeoff()
{
	_objCommand.position.z = _traverse_height;
	_objCommand.takeoff = 1;
	_objCommand.arm = 1;
 	_objCommand.offboard =  1;
	_objCommand.control  =  POS;
};

void Mission::land()
{
  _objCommand.takeoff = 0;
	_objCommand.land =1;
}

bool Mission::turnLeft90()
{
	if ((_yaw_prev > 0.5 * M_PI)&&(_yaw < 0))
	{
		_yaw += 2* M_PI; // warp around
	}

	if (fabs(_yaw_prev - _yaw) < 10 * _ang_tol)
	{
		_is_update = 1;
		_objCommand.yaw = _Kyaw * 5 * fabs(_yaw - _yaw_prev - 0.5*M_PI);
	} // only publish when perceived rotation is small (rotation rate is small)

	return (fabs(_yaw - _yaw_prev - 0.5 * M_PI) < (_ang_tol * 3)); // true if finished turning (set this threshold to be higher if overturn)
}

void Mission::objCallback(const geometry_msgs::Point pos)
{
	_obj_cnt ++ ;
	const float Ktrack = 0.01; // tracking gain
	if ((_obj_cnt > 5)&&(_flight_mode != landing) && (_flight_mode!=takingoff))
	{
		setFlightMode(tracking);
		_is_update = 1;
		_objCommand.position.y = Ktrack * (pos.z - 850.0) / 1000.0; // keep 0.9 m distance away from target
		_objCommand.position.x = Ktrack * (pos.x - 160.0)  / 120.0;
		_objCommand.position.z = Ktrack * (pos.y - 120.0)  / 120.0;
		_objCommand.yaw =  _Kyaw * _angle_rad;

		if ((pos.z < 1000) && (abs(pos.x-160) < 35 )&& (abs(pos.y-120)< 35)) //center arranged
		{
			ROS_INFO("[Mission] Found object ><><><><>< Land!");
			setFlightMode(landing);
		}
		else if (_obj_cnt>0)
		 _obj_cnt --;
	}

}


void Mission::wallCallback(const geometry_msgs::Point ang)
{
	_angle_rad = ang.x; // need to update this shit my friend
	if((_flight_mode == calibration) && (!_is_calibrate))
	{
		if(_angle_rad < -100 || _angle_rad > 0.5 || _angle_rad < -0.5)
		{
				ROS_INFO("[Mission] Cannot find wall");
				_cannot_find_wall_cnt++;
				if(_cannot_find_wall_cnt > 4) // if cannot find wall for too long land
				{
					setFlightMode(landing);
					_objCommand.land = 1;
				}

		}
		else
		{
			_cannot_find_wall_cnt--;
			if( (fabs(_angle_rad) > 0.1) && (!_is_calibrate) )					// P controller to adjust drone to a found wall
			{
				_is_update = 1;
				ROS_INFO("[Mission] Adjust Angle %3.2f", _angle_rad);
				_objCommand.yaw = _Kyaw * _angle_rad;
				_wall_cnt = 0;
			}
			else
			{
				_wall_cnt++;
				if(_wall_cnt > 20) {
					if (ang.z > 0) // valid angle
					{
					_is_update = 1;
					_objCommand.yaw =  _Kyaw * _angle_rad;
					_objCommand.position.y =  _Kpxy * (ang.z -_safe_dist)/1000.0; // move to 1 from obstacles
					 ROS_INFO("[Mission] Adjust Distance %4.3f,%4.3f",ang.z, _objCommand.position.y);
				 	}
				}

				if((_wall_cnt > 20) && (fabs(ang.z - (float)_safe_dist) < _lin_tol))
				{
					ROS_INFO("[Mission] Wall is perpendicular now.");   // drone is perpendicular, hover now
					if (!_is_calibrate)
					{
						_is_calibrate = 1;
						_yaw_prev = _yaw;
						ROS_WARN("[Mission] Yaw pin down is %f, z pin down is %f", _yaw_prev,_lpe(2,3));
						usleep(200*1000);
					}

				}

			}
		}
	}
	else if ((_flight_mode == calibration) && (_lpe(2,3) < 1.1)) // after going down: turn 90 degrees
	{
		ROS_INFO("[Mission] Start turning after going down. What's the height right now? %3.2f", _lpe(2,3));
		hover();
		setFlightMode(traverse); // so that after turning it will return to traverse rather than calibration
		setFlightMode(turning);

	}
};

void Mission::obstCallback(geometry_msgs::Point msg)
{
	_objCommand.control = 1; // defaul position control mode

	if ((_flight_mode > tracking) && (msg.y < _obj_fail))//failsafe object detect
	{
		crash_cnt++;
		if(crash_cnt > 20){
			setFlightMode(landing);
			ROS_ERROR("[Mission] obstacles too close detected. Land");
		};
	}
	else
		crash_cnt = 0;
	if (_flight_mode == traverse) // maybe want some threshold in case wrong depth occur
	{
			if ((msg.y < (_track_dist + 300))||(_obst_found == 1)) // start to decelerate at 30 cm
			{ // observation / calibration mode
					if (!_obst_found)
					{
					if((fabs(_angle_rad) > _ang_tol)||(fabs(_lpe(2,3) - _traverse_height)>_lin_tol)){
						if (_cali_cnt > 0){
								_cali_cnt--;
						}
						ROS_WARN("[Mission] == cali yaw and wall %5.3f",_angle_rad*180/3.1415926);
					}
					else if ((msg.y < (_track_dist+30))&& (msg.y > _track_dist))
					{
						_cali_cnt++;
					} // always get close to the wall

					// correct yaw always
					_is_update = 1;
					_objCommand.yaw = _Kyaw * _angle_rad;
					_objCommand.position.y = _Kpxy * (msg.z - _track_dist)/1000.0; // gradually calibrate to obstacle
				 }

				 if ((_cali_cnt > 10)||(_obst_found)) // back off mode
				 {
					_obst_found = 1;
					ROS_INFO("[Mission] Finish Observation Backoff %4.2f",msg.y); //
					if ((_safe_dist < _room_size)&&(!(_obst_cnt%3))){ // room dimension
						ROS_INFO("[Mission] Increment traverse distance");
						_safe_dist += _traverse_inc;
					}

					_objCommand.yaw = _Kyaw * _angle_rad;
					_objCommand.position.y = _Kpxy * (msg.z - _safe_dist)/1000.0;
					_is_update = 1;

					if ((msg.y < (_safe_dist+30)) && (msg.y > _safe_dist-30)){
						ROS_INFO("[Mission] Finished backing off start turning");
						_obst_found = 0;
						_obst_cnt++;
						_cali_cnt = 0;
						_yaw_prev = _yaw;
						setFlightMode(turning); // enter turning mode
					}
				}
			}
			else
			{
				_is_update = 1;
				_objCommand.control = VEL; // during traverse use velocity control mode (steady proceed)
				_obst_found = 0;

				if ((_vel(1)*_vel(1)+_vel(0)*_vel(0)) < 0.1) //
				{
					_objCommand.position.y = _traverse_speed; // proceed at velocity 0.1
				}
				else
				{
					hover(); // hover if velocity too big
				}
			}
	}

}

void Mission::velCallback(const geometry_msgs::TwistStamped vel_read)
{
	_vel.setZero();
	_vel(0) = vel_read.twist.linear.x;
	_vel(1) = vel_read.twist.linear.y;
	_vel(2) = vel_read.twist.linear.z;
}

void Mission::lpeCallback(const geometry_msgs::PoseStamped pos_read)
{

	Eigen::Quaternionf q (pos_read.pose.orientation.w, pos_read.pose.orientation.x, pos_read.pose.orientation.y,pos_read.pose.orientation.z);
	_lpe.setIdentity(); 	// clear buffer
	_lpe.topLeftCorner (3,3) = q.matrix();
	_lpe.topRightCorner(3,1) << pos_read.pose.position.x, pos_read.pose.position.y, pos_read.pose.position.z;
	rot2rpy(q.matrix(), _roll, _pitch, _yaw);

}
void Mission::resetCommand(px4_offboard::MavState& command)
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

inline void Mission::correctTraverseHeight()
{

	if (_objCommand.control == POS)
		_objCommand.position.z =  - _Kpz * (_lpe(2,3) - _traverse_height);
	else
		_objCommand.position.z =  - _Kv  * (_lpe(2,3) - _traverse_height);
}


void Mission::logSp()
{
	logMissionSp << ros::Time::now()
	<< "," << _flight_mode
	<< "," << (int)_objCommand.control
	<< "," << _objCommand.position.x
	<< "," << _objCommand.position.y
	<< "," << _objCommand.position.z
	<< "," << _objCommand.yaw
	<< "," << _angle_rad <<  endl;
};

void Mission::setFlightMode(int request_mode)
{
	if (_flight_mode!=request_mode)
	{
		_flight_mode_prev = _flight_mode; // note down previous flight mode
		_flight_mode = request_mode;
	}
  cout << "[Mission] Flight mode switched to " << flight_mode_string[_flight_mode] << endl;
};

// takes off really high
// see the wall
// get perpendicular to the wall
// go towards it
//
// go to apropriate height1
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
