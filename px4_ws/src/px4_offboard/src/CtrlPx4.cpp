#include "px4_offboard/CtrlPx4.h"
#include "px4_offboard/include.h"

#define TAKEOFF_RATIO 0.9

CtrlPx4::CtrlPx4() {

  nh_.getParam("/fcu/sim",sim_); 
  nh_.getParam("/fcu/ctrl",ctrl_);
//  nh_.getParam("/fcu/takeoff",v_takeoff_);

  off_en_ = sim_;     // initialize off_en_to be 1 always if in simulation mode
  auto_tl_ = ctrl_;

  mavros_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_position/local", 100);
  mavros_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 100);

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

  // compact message subscription
  joy_sub_ = nh_.subscribe("/joy/cmd_mav", 100, &CtrlPx4::joyCallback, this);
  april_sub_ = nh_.subscribe("april/cmd_mav",100,&CtrlPx4::aprilCallback,this);  
  // checker_sub_ = nh_.subscribe("/checker/cmd_mav", 100, &CtrlPx4::checkerCallback, this);
};

bool CtrlPx4::commandUpdate() {

  if (off_en_) {  

    // check if in auto takeoff landing mode
    if (auto_tl_){

      if((state_set_.takeoff)&&(!state_read_.takeoff)){
        takeoff(1.3,2); //takeoff to 2m at 2m/s initially
  	}
      else if((state_set_.land)&&(state_read_.arm))
        land(0.5);
    }

    if ((state_set_.arm!=state_read_.arm)&&(!state_set_.land))
      setArm(state_set_.arm);

    // mode check
    if (state_set_.mode != state_read_.mode)
      setMode(state_set_.mode);

    if (state_set_.arm) {
      if(auto_tl_&&((state_set_.takeoff)&&(!state_read_.takeoff))||((state_set_.land)&&(state_read_.arm))){
          mavros_vel_pub_.publish(fcu_vel_setpoint_);
	    }
      else 
  	  { 
  	     // ROS_INFO("Hover setpoint: [x: %f y:%f z: %f]", fcu_pos_setpoint_.pose.position.x, \
      	     //	fcu_pos_setpoint_.pose.position.y,fcu_pos_setpoint_.pose.position.z);

	        mavros_pos_pub_.publish(fcu_pos_setpoint_);
  	  }
    }

   }
// regardless always check for failsafe
  if(state_set_.failsafe)
  {
    state_set_.takeoff = 0;
    state_set_.land = 0;
    if (state_read_.arm){
    state_set_.arm = false;
    state_set_.mode = MANUAL;
    // set_armed_.request.value = false; // forcefully send disarm request
    // mavros_armed_client_.call(set_armed_);
    }
    else
    state_set_.failsafe = false;
  }

  ros::spinOnce();

}

bool CtrlPx4::stateCmp() {
  my_state *read = &state_read_;
  my_state *set = &state_set_;
  
  bool arm = (read->arm == set->arm);
  bool offboard = (read->offboard == set->offboard);
  bool in_the_sky = (arm && set->arm == 1); //&& read->takeoff;
  bool on_the_ground =
      (arm && set->arm == 0); 

  return offboard;
}

void CtrlPx4::joyCallback(const px4_offboard::JoyCommand joy) {

  ROS_INFO("Current setpoint command is x %f y %f z %f yaw %f",joy.position.x,joy.position.y,joy.position.z,joy.yaw);

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
 ROS_INFO("Current Yaw command is %f",joy.yaw);
 moveToPoint(joy.position.x,joy.position.y,joy.position.z,joy.yaw);
 // only matter to the position setpoint

}


void CtrlPx4::batCallback(const mavros_msgs::BatteryStatus bat)
{
	if (bat.voltage < 13)
	{
	  ROS_INFO("Low battery!");
	  state_set_.land = 1;
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
  off_en_ = (rc_in.channels[6] > 1200)&&((rc_in.channels[4] > 1200));
};
 
void CtrlPx4::poseCallback(const geometry_msgs::PoseStamped pos_read) {
  pos_read_.px = pos_read.pose.position.x;
  pos_read_.py = pos_read.pose.position.y;
  pos_read_.pz = pos_read.pose.position.z;

  pos_read_.q(0) = pos_read.pose.orientation.w; // qw
  pos_read_.q(1) = pos_read.pose.orientation.x; // qx
  pos_read_.q(2) = pos_read.pose.orientation.y; // qy
  pos_read_.q(3) = pos_read.pose.orientation.z; // qz

  pos_read_.yaw = atan2(2*(pos_read_.q[0] * pos_read_.q[3]+ pos_read_.q[1]*pos_read_.q[2]), \
  1-2*(pos_read_.q[3]*pos_read_.q[3]+pos_read_.q[2]*pos_read_.q[2]));

};

void CtrlPx4::velCallback(const geometry_msgs::TwistStamped vel_read) {
  vel_.vx = vel_read.twist.linear.x;
  vel_.vy = vel_read.twist.linear.y;
  vel_.vz = vel_read.twist.linear.z;
};


bool CtrlPx4::takeoff(double altitude, double velocity) {
  double current_height = pos_read_.pz;
  if (fabs(current_height - altitude)>5){
	ROS_INFO("Takeoff height too far!");
	return 0;
}
  if (!state_read_.arm)
{
//	ROS_INFO("Send Arming Command");
        set_armed_.request.value = true; // send arm request
        mavros_armed_client_.call(set_armed_);
}

  if (current_height < (TAKEOFF_RATIO * altitude)) {
    fcu_vel_setpoint_.twist.linear.z = (altitude - current_height) * velocity; // a rough p controller for velocity
    if (fcu_vel_setpoint_.twist.linear.z > velocity)
	fcu_vel_setpoint_.twist.linear.z = velocity;
    mavros_vel_pub_.publish(fcu_vel_setpoint_);
  }  
  else{
    ROS_INFO("[Takeoff] Current Height: %f", current_height);
    ROS_INFO("Finished Taking off");
    state_set_.takeoff = 0;
    hover();
  }

  state_read_.takeoff = current_height > (TAKEOFF_RATIO * altitude);

  return current_height > altitude;
}

bool CtrlPx4::land(double velocity) 
{
  double current_height = pos_read_.pz;
  //ROS_INFO("[Landing] Current Height: %f", current_height);
  if(current_height > 0.20)
  {
    fcu_vel_setpoint_.twist.linear.z = - current_height * velocity; // a rough p controller for velocity
    mavros_vel_pub_.publish(fcu_vel_setpoint_);
  }
  else {
    state_set_.land = 0;
    state_read_.land = 0;
    state_read_.takeoff = 0; // clean takeoff flag to allow takeoff again
    state_set_.mode = MANUAL;
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
      ROS_INFO("set manual");
      state_set_.prev_mode = state_read_.mode; //note down mode before arming
      set_mode_.request.custom_mode = "MANUAL";
      mavros_set_mode_client_.call(set_mode_);
      return (state_read_.mode == MANUAL);

      break;

    case POSCTL:
      ROS_INFO("set position control");
      state_set_.prev_mode = state_read_.mode; //note down mode before arming
      set_mode_.request.custom_mode = "POSCTL";
      mavros_set_mode_client_.call(set_mode_);
      return (state_read_.mode == POSCTL);
      break;


    case OFFBOARD:
      ROS_INFO("set offboard");
      state_set_.prev_mode = state_read_.mode; //note down mode before arming
      set_mode_.request.custom_mode = "OFFBOARD";
      mavros_set_mode_client_.call(set_mode_);
      return (state_read_.mode == OFFBOARD);
      break;

    default:
      ROS_INFO("Mode not implemented, current mode is %d", state_read_.mode);
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

  //ROS_INFO("Current yaw angle is %f", yaw);
  float x = fcu_pos_setpoint_.pose.position.x + pos_nav(0);
  float y = fcu_pos_setpoint_.pose.position.y + pos_nav(1);
  float z = fcu_pos_setpoint_.pose.position.z + pos_body(2);
  float qw = cos(0.5 * yaw + dyaw_sp);
  float qz = sin(0.5 * yaw + dyaw_sp);

  // decided if we need to reset position setpoint 
  if ((fabs(pos_read_.px + pos_nav(0) - x) + fabs(pos_read_.py + pos_nav(1)-y)) > 1){
    fcu_pos_setpoint_.pose.position.x =   pos_read_.px;
    fcu_pos_setpoint_.pose.position.y =   pos_read_.py;
    ROS_INFO("Reset xy");
  } else {
     fcu_pos_setpoint_.pose.position.x =  x; 
     fcu_pos_setpoint_.pose.position.y  = y;
   }

  if (fabs(pos_read_.pz + pos_body(2) - z) > 3){
  fcu_pos_setpoint_.pose.position.z =  pos_read_.pz;
  ROS_INFO("Reset z"); 
  } else {
    fcu_pos_setpoint_.pose.position.z = z;
  }

  fcu_pos_setpoint_.pose.orientation.w = qw; // TODO add local
  fcu_pos_setpoint_.pose.orientation.z = qz;

  // note down a prev state 
  prev_pos_read_.px = pos_read_.px;
  prev_pos_read_.py = pos_read_.py;
  prev_pos_read_.pz = pos_read_.pz;

//  ROS_INFO("setpoint: [x: %f y:%f z: %f]", fcu_pos_setpoint_.pose.position.x, \
 //   fcu_pos_setpoint_.pose.position.y,fcu_pos_setpoint_.pose.position.z);
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
