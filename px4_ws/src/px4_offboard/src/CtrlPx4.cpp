#include <unistd.h>
#include <stdio.h>
#include "px4_offboard/CtrlPx4.h"
#include "px4_offboard/include.h"

#define TAKEOFF_RATIO 0.9
//#define M_PI 3.1415926

bool yaw_calibrate = 0;
bool vision_init = 0;
float yaw_cali_value = 0;
double secs;

CtrlPx4::CtrlPx4() {

  nh_.getParam("/fcu/sim",sim_);
  nh_.getParam("/fcu/ctrl",ctrl_);
  nh_.getParam("/fcu/cali",yaw_cali_value);

//  nh_.getParam("/fcu/takeoff",v_takeoff_);
  std::cout << std::fixed<<std::setprecision(4);

  off_en_ = sim_;     // initialize off_en_to be 1 always if in simulation mode
  auto_tl_ = ctrl_;
//  yaw_cali_value = cali_;

  ROS_WARN("Yaw cali value is %5.4f", yaw_cali_value);

  controller_state_.fail  = 0;
  yaw_sp_ = 0;

//PID pid - Take off;
  pid_takeoff.setKp(1);
  pid_takeoff.setKi(0);
  pid_takeoff.setKd(0.2);
//  pid_takeoff.setInput(1);

//PID pid - Land;
  pid_land.setKp(0.6);
  pid_land.setKi(0);
  pid_land.setKd(0);
  pid_land.setInput(0);


  mavros_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_position/local", 100);
  mavros_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 100);

  px4_offboard_pub_ = nh_.advertise<px4_offboard::CtrlState>("/px4_offboard/state",100);

  mavros_set_mode_client_ =
      nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_armed_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  bat_sub_ = nh_.subscribe("/mavros/battery",10,&CtrlPx4::batCallback, this);

  local_vel_sub_ = nh_.subscribe("/mavros/local_position/velocity", 100,
                                 &CtrlPx4::velCallback, this);
  local_pos_sub_ = nh_.subscribe("/mavros/local_position/pose", 100,
                                 &CtrlPx4::poseCallback, this);
  state_sub_ =
      nh_.subscribe("/mavros/state", 100, &CtrlPx4::stateCallback, this);
  radio_sub_ =
      nh_.subscribe("/mavros/rc/in", 10, &CtrlPx4::radioCallback, this);

  // wall_sub = nh_.subscribe("/objDetect/wall_pose", 100, &wallCallback);

  // compact message subscription
  joy_sub_ = nh_.subscribe("/joy/cmd_mav", 100, &CtrlPx4::joyCallback, this);
  // april_sub_ = nh_.subscribe("/april/cmd_mav",100,&CtrlPx4::aprilCallback,this);

  obj_sub_ = nh_.subscribe("/obj/cmd_mav",100,&CtrlPx4::objCallback,this);

  // checker_sub_ = nh_.subscribe("/checker/cmd_mav", 100, &CtrlPx4::checkerCallback, this);
};


bool CtrlPx4::commandUpdate() {



float yaw_old, yaw_new;
  if (off_en_) {

    // check if in auto takeoff landing mode
    if (auto_tl_>0){

    if((state_set_.takeoff)&&(!state_read_.takeoff)){
        takeoff(1, 2); //takeoff to 1 m at 2m/s initially // to adjust to the wall
  	}
    else if((state_set_.land)&&(state_read_.arm))
        land(1);    // land at 1m/s
    }

    // arm check
    if ((state_set_.arm!=state_read_.arm)&&(!state_set_.land))
      setArm(state_set_.arm);

    // mode check
    if (state_set_.mode != state_read_.mode)
      setMode(state_set_.mode);

    // controller input
    if (state_set_.arm) {
      if(( auto_tl_> 0 )&&((state_set_.takeoff)&&(!state_read_.takeoff))||((state_set_.land)&&(state_read_.arm))){
        if(state_set_.takeoff) // pos control takeoff
          mavros_pos_pub_.publish(fcu_pos_setpoint_);
        else // vel control landing
          mavros_vel_pub_.publish(fcu_vel_setpoint_);
	   }

      else {
	          mavros_pos_pub_.publish(fcu_pos_setpoint_);
  	  }
    }
    updateState(); // brodcast state of the drone
   }

   else{


/*if (!yaw_calibrate)
  	{
	yaw_calibrate = 1;
	yaw_old = pos_read_.yaw;
        secs = ros::Time::now().toSec();
	// ROS_INFO("Yaw old is %5.2f", yaw_old);
  	}

	if (yaw_calibrate && (ros::Time::now().toSec() - secs)> 3){
		yaw_calibrate = 0;
		yaw_new = pos_read_.yaw;

		yaw_cali_value =  (yaw_new-yaw_old)/(ros::Time::now().toSec() - secs);
		ROS_INFO("Yaw cali is %5.4f", yaw_cali_value);

	}
*/
hover(); // always note down the most recent position
}

// regardless always check for failsafe
  if(fabs(pos_read_.roll) > (M_PI/4) || fabs(pos_read_.pitch) > (M_PI/4)){ // disarm by angle desbalance
  controller_state_.fail = 1;

  if(state_read_.arm)
	ROS_WARN("[PX4 CTRL] Lost balance! Disarm!");

	state_set_.arm = false;
  state_set_.mode = MANUAL;
	state_set_.takeoff = 0;
	state_set_.land = 0;

  }

  if(state_set_.failsafe)
  {
    controller_state_.fail = 1;
    state_set_.takeoff = 0;
    state_read_.takeoff = 0;
    state_set_.land = 0;

    if (state_read_.arm){
    state_set_.arm = false;
    state_set_.mode = MANUAL;
    }
    else
    state_set_.failsafe = false;
  }


  ros::spinOnce();

}

void CtrlPx4::joyCallback(const px4_offboard::JoyCommand joy) {

  ROS_INFO("Joy setpoint received");

  moveToPoint(joy.position.x,joy.position.y,joy.position.z,joy.yaw);

  // state_set_.offboard = joy.offboard;
  if (joy.offboard)
	   state_set_.mode = OFFBOARD;

  state_set_.arm      = joy.arm;
  state_set_.takeoff  = joy.takeoff;
  state_set_.land     = joy.land;
  state_set_.failsafe = joy.failsafe;

}

void CtrlPx4::objCallback(const px4_offboard::JoyCommand joy)
{

//  if (joy.yaw!=0.0)
// {
//  cout << "yaw delta is " <<  joy.yaw << " current yaw " << pos_read_.yaw << endl;
// }

 moveToPoint(joy.position.x,joy.position.y,joy.position.z,joy.yaw);
     // state_set_.offboard = joy.offboard;
  if (joy.offboard)
     state_set_.mode = OFFBOARD;

  state_set_.arm      = joy.arm;
  state_set_.takeoff  = joy.takeoff;
  state_set_.land     = joy.land;
  state_set_.failsafe = joy.failsafe;

}


void CtrlPx4::aprilCallback(const px4_offboard::JoyCommand joy)
{
 // ROS_INFO("Current Yaw command is %f",joy.yaw);
 moveToPoint(joy.position.x,joy.position.y,joy.position.z,joy.yaw);


}


void CtrlPx4::batCallback(const mavros_msgs::BatteryStatus bat)
{
	if (bat.voltage < 14)
	{
	  ROS_WARN("[PX4 CTRL] Low battery!");
	  // state_set_.land = 1;
	}

}


void CtrlPx4::stateCallback(const mavros_msgs::State vehicle_state) {

  state_read_.arm = (vehicle_state.armed == 128);

  if (strcmp(vehicle_state.mode.c_str(), "OFFBOARD") == 0)
    state_read_.mode = OFFBOARD;

  else if (strcmp(vehicle_state.mode.c_str(), "MANUAL") == 0)
    state_read_.mode = MANUAL;

  else if (strcmp(vehicle_state.mode.c_str(), "POSCTL") == 0)
    state_read_.mode = POSCTL;

  state_read_.offboard = (strcmp(vehicle_state.mode.c_str(), "OFFBOARD") == 0);

};

void CtrlPx4::radioCallback(const mavros_msgs::RCIn rc_in) {
  off_en_ = (rc_in.channels[6] > 1200)&&((rc_in.channels[4] > 900)&&(rc_in.channels[4] < 1200));
};


void CtrlPx4::poseCallback(const geometry_msgs::PoseStamped pos_read) {
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

};

void CtrlPx4::velCallback(const geometry_msgs::TwistStamped vel_read) {
  vel_.vx = vel_read.twist.linear.x;
  vel_.vy = vel_read.twist.linear.y;
  vel_.vz = vel_read.twist.linear.z;
};


void CtrlPx4::updateState()
{
  controller_state_.arm         = state_read_.arm;
  controller_state_.takeoff     = state_read_.takeoff;
  controller_state_.land        = state_read_.land;
  controller_state_.offboard    = state_read_.offboard;
  px4_offboard_pub_.publish(controller_state_);
}


bool CtrlPx4::takeoff(double altitude, double velocity) {

  double current_height = pos_read_.pz;
  double vel_sp =  0;
  pid_takeoff.setInput(altitude);

 if (fabs(current_height - altitude)>5){
	ROS_INFO("[PX4 CTRL] Takeoff height too far!");
	return 0;
  }

  if (!state_read_.arm)
  {
	home_.px = pos_read_.px;
	home_.py = pos_read_.py;
        set_armed_.request.value = true; // send arm request
        mavros_armed_client_.call(set_armed_);
  }


  if (current_height < (TAKEOFF_RATIO * altitude)) {

    // sensor read
    if(auto_tl_ == 1)
    {
      fcu_pos_setpoint_.pose.position.x = home_.px;
      fcu_pos_setpoint_.pose.position.y = home_.py;
      fcu_pos_setpoint_.pose.position.z = altitude; //directly publish pos setpoint
    }
    else
    {
      pid_takeoff.setSensor(pos_read_.pz);
      vel_sp = pid_takeoff.update();
      // velocity saturation
      if(vel_sp < 0) {
        fcu_vel_setpoint_.twist.linear.z   = 0;
      }
      else if (vel_sp > velocity){
        fcu_vel_setpoint_.twist.linear.z = velocity;
      }
      else {
        fcu_vel_setpoint_.twist.linear.z = vel_sp;
      }
    }

  }

  else{
    //ROS_INFO("[PX4 CTRL] Finished Takeoff at Height: %f", current_height);
    std::cout << "================================== finished taking off";
    state_set_.takeoff = 0;
    //hover();
    //cout << " with height setpoint " <<fcu_pos_setpoint_.pose.position.z<<std::endl;
  }

  state_read_.takeoff = (current_height > TAKEOFF_RATIO *altitude);
  return state_read_.takeoff;
}


bool CtrlPx4::land(double velocity)
{
  double current_height = pos_read_.pz;
  pid_land.setSensor(pos_read_.pz);
  //  ROS_INFO("start landing");

  if(current_height > 0.11 || fabs(pid_land.getDerivate()) > 0.1 )
  {
   // ROS_INFO("[Land] PID derivative term is %f, height is %f", pid_land.getDerivate(),current_height);
    //if (auto_tl_ == 1)
    	//fcu_pos_setpoint_.pose.position.z = 0.1;
    //else
    //{
      fcu_vel_setpoint_.twist.linear.z = pid_land.update(); // a rough p controller for velocity
      mavros_vel_pub_.publish(fcu_vel_setpoint_);
    //}
  }
  else
  {
    state_set_.land = 0;
    state_read_.land = 0;
    state_set_.takeoff = 0;
    state_read_.takeoff = 0; // clean takeoff flag to allow takeoff again
    state_set_.mode = MANUAL;
    state_set_.arm = false; // free fall
  }

}

void CtrlPx4::hover() {
  double yaw = atan2(2*(pos_read_.q[0] * pos_read_.q[3]+ pos_read_.q[1]*pos_read_.q[2]), \
  1-2*(pos_read_.q[3]*pos_read_.q[3]+pos_read_.q[2]*pos_read_.q[2]));

      fcu_pos_setpoint_.pose.position.x = pos_read_.px;
      fcu_pos_setpoint_.pose.position.y = pos_read_.py;
      fcu_pos_setpoint_.pose.position.z = pos_read_.pz;

      fcu_pos_setpoint_.pose.orientation.w = cos(0.5 * (yaw));
      fcu_pos_setpoint_.pose.orientation.z = sin(0.5 * (yaw));

};

bool CtrlPx4::setMode(int mode)
{
  switch(mode)
  {
    case MANUAL:
      ROS_INFO("[PX4 CTRL] set manual");
      state_set_.prev_mode = state_read_.mode; //note down mode before arming
      set_mode_.request.custom_mode = "MANUAL";
      mavros_set_mode_client_.call(set_mode_);
      return (state_read_.mode == MANUAL);

      break;

    case POSCTL:
      ROS_INFO("[PX4 CTRL] set position control");
      state_set_.prev_mode = state_read_.mode; //note down mode before arming
      set_mode_.request.custom_mode = "POSCTL";
      mavros_set_mode_client_.call(set_mode_);
      return (state_read_.mode == POSCTL);
      break;


    case OFFBOARD:
      ROS_INFO("[PX4 CTRL] set offboard");
      state_set_.prev_mode = state_read_.mode; //note down mode before arming
      set_mode_.request.custom_mode = "OFFBOARD";
      mavros_set_mode_client_.call(set_mode_);
      return (state_read_.mode == OFFBOARD);
      break;

    default:
      ROS_INFO("[PX4 CTRL] Mode not implemented, current mode is %d", state_read_.mode);
      return 0;

  }
};

bool CtrlPx4::setArm(bool arm)
{
    set_armed_.request.value = arm; // forcefully send disarm request
    mavros_armed_client_.call(set_armed_);
    return state_read_.arm;
};

void CtrlPx4::moveToPoint (float dx_sp, float dy_sp, float dz_sp, float dyaw_sp)
{
  Vector3f pos_body, pos_nav;


// basic geometry, current yaw
  double yaw = pos_read_.yaw;
  pos_body << dx_sp, dy_sp, dz_sp;
  pos_nav (0) = -pos_body(0)*sin(yaw) + pos_body(1)*cos(yaw); // left and right
  pos_nav (1) = pos_body(0)*cos(yaw) + pos_body(1)*sin(yaw); // front and back

  if (state_read_.takeoff)
{

	float yaw_comp = yaw_cali_value * (ros::Time::now().toSec()-secs);
	ROS_INFO("Compensate yaw drift for %3.2f" , yaw_comp);
  	yaw_sp_ -=  yaw_comp;
	secs = ros::Time::now().toSec();
}


  float x = fcu_pos_setpoint_.pose.position.x + pos_nav(0);
  float y = fcu_pos_setpoint_.pose.position.y + pos_nav(1);
  float z = fcu_pos_setpoint_.pose.position.z + pos_body(2);

// need yaw drift handling
  if( fabs(yaw-yaw_sp_) < MAX_DYAW){
	yaw = yaw_sp_;  // maintain previous yaw setpoint (prevent drifting)
  }
  else
  {
	ROS_INFO("reset yaw_sp");
  }
  yaw_sp_ = yaw + dyaw_sp;

  float qw = cos(0.5 * yaw + dyaw_sp);
  float qz = sin(0.5 * yaw + dyaw_sp);

  //ROS_INFO("Current yaw angle setpoint %3.2f, quat %3.2f, %3.2f", yaw+dyaw_sp, qw, qz);
  //std::cout << "current yaw setpoint and yaw " << yaw+dyaw_sp << " " << pos_read_.yaw << std::endl;

  //std::cout << "current x y z setpont " << fcu_pos_setpoint_.pose.position.x << " " << fcu_pos_setpoint_.pose.position.y << " "<<fcu_pos_setpoint_.pose.position.z << std:: endl;
  //std::cout << "current x y z estimate " << pos_read_.px << " " << pos_read_.py <<" "<<pos_read_.pz << std:: endl<< std::endl;


  // decided if we need to reset position setpoint
  if ((fabs(pos_read_.px + pos_nav(0) - x) + fabs(pos_read_.py + pos_nav(1)-y)) > MAX_DXY){
    fcu_pos_setpoint_.pose.position.x =   pos_read_.px +  pos_nav(0);
    fcu_pos_setpoint_.pose.position.y =   pos_read_.py +  pos_nav(1);
    ROS_INFO("[PX4 CTRL] Reset xy");
  } else {
     fcu_pos_setpoint_.pose.position.x =  x;
     fcu_pos_setpoint_.pose.position.y  = y;
   }

  if (fabs(pos_read_.pz + pos_body(2) - z) > MAX_DZ){
  fcu_pos_setpoint_.pose.position.z =  pos_read_.pz;
  ROS_INFO("[PX4 CTRL] Reset z");
  } else if ( z < MAX_Z){
    fcu_pos_setpoint_.pose.position.z = z;
  }
  else{
    fcu_pos_setpoint_.pose.position.z = MAX_Z; // saturate z to avoid going off
  }


  fcu_pos_setpoint_.pose.orientation.w = qw; // TODO add local
  fcu_pos_setpoint_.pose.orientation.z = qz;

  // note down a prev state
  prev_pos_read_.px = pos_read_.px;
  prev_pos_read_.py = pos_read_.py;
  prev_pos_read_.pz = pos_read_.pz;

  //ROS_INFO("setpoint: [x: %f y:%f z: %f]", fcu_pos_setpoint_.pose.position.x, \
    //fcu_pos_setpoint_.pose.position.y,fcu_pos_setpoint_.pose.position.z);

};

void CtrlPx4::forward(float distance)
{
 moveToPoint(distance,0,0,0);
}

void CtrlPx4::backward(float distance)
{
  moveToPoint(-distance,0,0,0);
};

void CtrlPx4::left (float distance)
{
  moveToPoint(0,distance,0,0);
}
void CtrlPx4::right(float distance)
{
  moveToPoint(0,-distance,0,0);
}

void CtrlPx4::up(float distance)
{
  moveToPoint(0,0,distance,0);
}

void CtrlPx4::down(float distance)
{
  moveToPoint(0,0,-distance,0);
}

void CtrlPx4::yawLeft(float radian)
{
  moveToPoint(0,0,0,radian);
}

void CtrlPx4::yawRight(float radian)
{
  moveToPoint(0,0,0,-radian);
}
