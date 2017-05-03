#include <unistd.h>
#include <stdio.h>
#include "px4_offboard/CtrlPx4.h"
#include "px4_offboard/include.h"

#define TAKEOFF_RATIO 0.8

bool yaw_calibrate = 0;
bool vision_init = 0;
float yaw_cali_value = 0;
double secs;

CtrlPx4::CtrlPx4() {

  nh_.getParam("/fcu/sim", sim_);
  nh_.getParam("/fcu/takeoff", auto_tl_);
  // nh_.getParam("/fcu/cali", yaw_cali_value);

  std::cout << std::fixed << std::setprecision(4);

  off_en_ = sim_;                   // initialize off_en_to be 1 always if in simulation mode
  pos_ctrl_ = POS;                    // default position control
  controller_state_.failsafe = 0;   // clear fail safe flag
  prev_yaw_sp_ = 0;                 // clear previous yaw setpoint
  // PID pid - Land;
  pid_land.setKp(0.6);
  pid_land.setKi(0);
  pid_land.setKd(0);
  pid_land.setInput(0);

  mavros_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_position/local", 100);
  mavros_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 100);

  px4_offboard_pub_ =
      nh_.advertise<px4_offboard::MavState>("/px4_offboard/state", 100);

  mavros_set_mode_client_ =
      nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_armed_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  bat_sub_ = nh_.subscribe("/mavros/battery", 10, &CtrlPx4::batCallback, this);

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
  obj_sub_ = nh_.subscribe("/obj/cmd_mav", 100, &CtrlPx4::objCallback, this);
  // checker_sub_ = nh_.subscribe("/checker/cmd_mav", 100, &CtrlPx4::checkerCallback, this);
};

bool CtrlPx4::commandUpdate() {

  float yaw_old, yaw_new;
  if (off_en_) {
    // check if in auto takeoff landing mode
    if (auto_tl_ > 0) {

      if ((state_set_.takeoff) && (!state_read_.takeoff)) {
        takeoff(0.85, 1); // takeoff to 1 m at 2m/s initially // to adjust to the wall
      } else if ((state_set_.land) && (state_read_.arm))
        land(1); // land at 1m/s
    }

    // arm check
    if ((state_set_.arm != state_read_.arm) && (!state_set_.land))
      setArm(state_set_.arm);

    // mode check
    if (state_set_.mode != state_read_.mode)
      setMode(state_set_.mode);

    // controller input
    if (state_set_.arm) {
      if ((auto_tl_ > 0) && ((state_set_.takeoff) && (!state_read_.takeoff)) ||
          ((state_set_.land) && (state_read_.arm))) {
        if (state_set_.takeoff) // pos control takeoff
          mavros_pos_pub_.publish(fcu_pos_setpoint_);
        else // vel control landing
          mavros_vel_pub_.publish(fcu_vel_setpoint_);
      }

      else {
        if (pos_ctrl_)
          mavros_pos_pub_.publish(fcu_pos_setpoint_);
        else
          mavros_vel_pub_.publish(fcu_vel_setpoint_);
      }
    }
    updateState(); // brodcast state of the drone
  }

  else {
    hover(); // always note down the most recent position
  }

  // regardless always check for failsafe
  if (fabs(pos_read_.roll) > (M_PI / 4) ||
      fabs(pos_read_.pitch) > (M_PI / 4)) { // disarm by angle desbalance
    controller_state_.failsafe = 1;

    if (state_read_.arm)
      ROS_WARN("[PX4 CTRL] Lost balance! Disarm!");

    state_set_.arm = false;
    state_set_.mode = MANUAL;
    state_set_.takeoff = 0;
    state_set_.land = 0;
  }

  if (state_set_.failsafe) {
    controller_state_.failsafe = 1;
    state_set_.takeoff = 0;
    state_read_.takeoff = 0;
    state_set_.land = 0;

    if (state_read_.arm) {
      state_set_.arm = false;
      state_set_.mode = MANUAL;
    } else
      state_set_.failsafe = false;
  }

  ros::spinOnce();
}

void CtrlPx4::joyCallback(const px4_offboard::MavState joy) {

  ROS_INFO("Joy setpoint received");

  moveToPoint(joy.position.x, joy.position.y, joy.position.z, joy.yaw);

  // state_set_.offboard = joy.offboard;
  if (joy.offboard)
    state_set_.mode = OFFBOARD;

  state_set_.arm = joy.arm;
  state_set_.takeoff = joy.takeoff;
  state_set_.land = joy.land;
  state_set_.failsafe = joy.failsafe;
}

void CtrlPx4::objCallback(const px4_offboard::MavState joy) {

  //  if (joy.yaw!=0.0)
  // {
  //  cout << "yaw delta is " <<  joy.yaw << " current yaw " << pos_read_.yaw <<
  //  endl;
  // }

  moveToPoint(joy.position.x, joy.position.y, joy.position.z, joy.yaw);

  if (joy.offboard)
    state_set_.mode = OFFBOARD;

  pos_ctrl_ = joy.control; // note down if we need to change control mode (in traverse mode we use velocity setpoint)
  state_set_.arm = joy.arm;
  state_set_.takeoff = joy.takeoff;
  state_set_.land = joy.land;
  state_set_.failsafe = joy.failsafe;
}

void CtrlPx4::aprilCallback(const px4_offboard::MavState joy) {
  moveToPoint(joy.position.x, joy.position.y, joy.position.z, joy.yaw);
}

void CtrlPx4::batCallback(const mavros_msgs::BatteryStatus bat) {
  if (bat.voltage < BAT_LOW_THRESH) {
    ROS_WARN("[PX4 CTRL] Low battery!");
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
  off_en_ = (rc_in.channels[6] > 1200) &&
            ((rc_in.channels[4] > 900) && (rc_in.channels[4] < 1200));
};

void CtrlPx4::poseCallback(const geometry_msgs::PoseStamped pos_read) {
  pos_read_.px = pos_read.pose.position.x;
  pos_read_.py = pos_read.pose.position.y;
  pos_read_.pz = pos_read.pose.position.z;

  pos_read_.q(0) = pos_read.pose.orientation.w; // qw
  pos_read_.q(1) = pos_read.pose.orientation.x; // qx
  pos_read_.q(2) = pos_read.pose.orientation.y; // qy
  pos_read_.q(3) = pos_read.pose.orientation.z; // qz

  pos_read_.roll = atan2(
      2.0 * (pos_read_.q[0] * pos_read_.q[1] + pos_read_.q[3] * pos_read_.q[2]),
      pos_read_.q[3] * pos_read_.q[3] + pos_read_.q[0] * pos_read_.q[0] -
          pos_read_.q[1] * pos_read_.q[1] - pos_read_.q[2] * pos_read_.q[2]);

  pos_read_.pitch = asin(-2.0 * (pos_read_.q[0] * pos_read_.q[2] -
                                 pos_read_.q[3] * pos_read_.q[1]));

  pos_read_.yaw = atan2(
      2 * (pos_read_.q[0] * pos_read_.q[3] + pos_read_.q[1] * pos_read_.q[2]),
      1 -  2 * (pos_read_.q[3] * pos_read_.q[3] +
               pos_read_.q[2] * pos_read_.q[2]));
};

void CtrlPx4::velCallback(const geometry_msgs::TwistStamped vel_read) {
  vel_.vx = vel_read.twist.linear.x;
  vel_.vy = vel_read.twist.linear.y;
  vel_.vz = vel_read.twist.linear.z;
};

void CtrlPx4::updateState() {
  // echo vechicle position to path planning / mission

  controller_state_.position.x = pos_read_.px;
  controller_state_.position.y = pos_read_.py;
  controller_state_.position.z = pos_read_.pz;
  controller_state_.yaw = pos_read_.yaw;

  controller_state_.control = pos_ctrl_;
  controller_state_.arm = state_read_.arm;
  controller_state_.takeoff = state_read_.takeoff;
  controller_state_.land = state_read_.land;
  controller_state_.offboard = state_read_.offboard;
  px4_offboard_pub_.publish(controller_state_);
}

bool CtrlPx4::takeoff(double altitude, double velocity) {

  double current_height = pos_read_.pz;
  double vel_sp = 0;
  pid_takeoff.setInput(altitude);

  if (fabs(current_height - altitude) > 5) {
    ROS_INFO("[PX4 CTRL] Takeoff height too far!");
    return 0;
  }

  if (!state_read_.arm) {
    home_.px = pos_read_.px;
    home_.py = pos_read_.py;
    set_armed_.request.value = true; // send arm request
    mavros_armed_client_.call(set_armed_);
  } else if (current_height < (TAKEOFF_RATIO * altitude)) {

    // sensor read
    if (auto_tl_ == 1) {
      fcu_pos_setpoint_.pose.position.x = home_.px;
      fcu_pos_setpoint_.pose.position.y = home_.py;
      fcu_pos_setpoint_.pose.position.z = altitude; // directly publish pos setpoint
    } else {
      pid_takeoff.setSensor(pos_read_.pz);
      vel_sp = pid_takeoff.update();
      // velocity saturation
      if (vel_sp < 0) {
        fcu_vel_setpoint_.twist.linear.z = 0;
      } else if (vel_sp > velocity) {
        fcu_vel_setpoint_.twist.linear.z = velocity;
      } else {
        fcu_vel_setpoint_.twist.linear.z = vel_sp;
      }
    }

  }
  else {
    std::cout << "================================== finished taking off"
              << std::endl;
    state_set_.takeoff = 0;
    hover();
  }

  state_read_.takeoff = (current_height > TAKEOFF_RATIO * altitude);
  return state_read_.takeoff;
}

bool CtrlPx4::land(double velocity) {
  double current_height = pos_read_.pz;
  pid_land.setSensor(pos_read_.pz);

  if (current_height > 0.11 || fabs(pid_land.getDerivate()) > 0.1) {
    fcu_vel_setpoint_.twist.linear.z =
        pid_land.update(); // a rough p controller for velocity
    mavros_vel_pub_.publish(fcu_vel_setpoint_);
    //}
  } else {
    std::cout << "---------finished landing" << std::endl;
    state_set_.land = 0;
    state_read_.land = 0;
    state_set_.takeoff = 0;
    state_read_.takeoff = 0; // clean takeoff flag to allow takeoff again
    state_set_.mode = MANUAL;
    state_set_.arm = false; // free fall
  }
}

void CtrlPx4::hover() {
  double yaw = atan2(
      2 * (pos_read_.q[0] * pos_read_.q[3] + pos_read_.q[1] * pos_read_.q[2]),
      1 - 2 * (pos_read_.q[3] * pos_read_.q[3] +  pos_read_.q[2] * pos_read_.q[2]));

  fcu_pos_setpoint_.pose.position.x = pos_read_.px;
  fcu_pos_setpoint_.pose.position.y = pos_read_.py;
  fcu_pos_setpoint_.pose.position.z = pos_read_.pz;

  fcu_pos_setpoint_.pose.orientation.w = cos(0.5 * (yaw));
  fcu_pos_setpoint_.pose.orientation.z = sin(0.5 * (yaw));

  fcu_vel_setpoint_.twist.linear.x = 0;
  fcu_vel_setpoint_.twist.linear.y = 0;
  fcu_vel_setpoint_.twist.linear.z = 0;
  fcu_vel_setpoint_.twist.angular.z = 0;
};

bool CtrlPx4::setMode(int mode) {
  switch (mode) {
  case MANUAL:
    ROS_INFO("[PX4 CTRL] set manual");
    state_set_.prev_mode = state_read_.mode; // note down mode before arming
    set_mode_.request.custom_mode = "MANUAL";
    mavros_set_mode_client_.call(set_mode_);
    return (state_read_.mode == MANUAL);

    break;

  case POSCTL:
    ROS_INFO("[PX4 CTRL] set position control");
    state_set_.prev_mode = state_read_.mode; // note down mode before arming
    set_mode_.request.custom_mode = "POSCTL";
    mavros_set_mode_client_.call(set_mode_);
    return (state_read_.mode == POSCTL);
    break;

  case OFFBOARD:
    ROS_INFO("[PX4 CTRL] set offboard");
    state_set_.prev_mode = state_read_.mode; // note down mode before arming
    set_mode_.request.custom_mode = "OFFBOARD";
    mavros_set_mode_client_.call(set_mode_);
    return (state_read_.mode == OFFBOARD);
    break;

  default:
    ROS_INFO("[PX4 CTRL] Mode not implemented, current mode is %d",
             state_read_.mode);
    return 0;
  }
};

bool CtrlPx4::setArm(bool arm) {
  set_armed_.request.value = arm; // forcefully send disarm request
  mavros_armed_client_.call(set_armed_);
  return state_read_.arm;
};

void CtrlPx4::moveToPoint(float dx_sp, float dy_sp, float dz_sp,
                          float dyaw_sp) {
  Vector3f pos_body, pos_nav;

  // basic geometry, current yaw
  double yaw = pos_read_.yaw;

  pos_body << dx_sp, dy_sp, dz_sp;
  pos_nav(0) =
      -pos_body(0) * sin(yaw) + pos_body(1) * cos(yaw); // left and right
  pos_nav(1) =
      pos_body(0) * cos(yaw) + pos_body(1) * sin(yaw); // front and back

  float x = fcu_pos_setpoint_.pose.position.x + pos_nav(0);
  float y = fcu_pos_setpoint_.pose.position.y + pos_nav(1);
  float z = fcu_pos_setpoint_.pose.position.z + pos_body(2);

  // need yaw drift handling
  if (fabs(yaw - prev_yaw_sp_) < MAX_DYAW) {
    yaw = prev_yaw_sp_; // maintain previous yaw setpoint (prevent drifting)
  } else {
    ROS_INFO("reset yaw_sp");
  }

  float qw = cos(0.5 * yaw + dyaw_sp);
  float qz = sin(0.5 * yaw + dyaw_sp);
  prev_yaw_sp_ = yaw + dyaw_sp;

  // decided if we need to reset position setpoint
  if ((fabs(pos_read_.px + pos_nav(0) - x) +
       fabs(pos_read_.py + pos_nav(1) - y)) > MAX_DXY) {
    fcu_pos_setpoint_.pose.position.x = pos_read_.px + pos_nav(0);
    fcu_pos_setpoint_.pose.position.y = pos_read_.py + pos_nav(1);
    ROS_INFO("[PX4 CTRL] Reset xy");
  } else {
    fcu_pos_setpoint_.pose.position.x = x;
    fcu_pos_setpoint_.pose.position.y = y;
  }

  if (fabs(pos_read_.pz + pos_body(2) - z) > MAX_DZ) {
    fcu_pos_setpoint_.pose.position.z = pos_read_.pz;
    ROS_INFO("[PX4 CTRL] Reset z");
  } else if (z < MAX_Z) {
    fcu_pos_setpoint_.pose.position.z = z;
  } else {
    fcu_pos_setpoint_.pose.position.z = MAX_Z; // saturate z to avoid going off
  }

  fcu_pos_setpoint_.pose.orientation.w = qw; // TODO add local
  fcu_pos_setpoint_.pose.orientation.z = qz;

  // note down the previous setpoint
  prev_pos_read_.px = pos_read_.px;
  prev_pos_read_.py = pos_read_.py;
  prev_pos_read_.pz = pos_read_.pz;

  // also update velocity setpoint here in case we need it
  fcu_vel_setpoint_.twist.linear.x =
      -pos_body(0) * sin(yaw) + pos_body(1) * cos(yaw);
  fcu_vel_setpoint_.twist.linear.y =
      pos_body(0) * cos(yaw) + pos_body(1) * sin(yaw);
  fcu_vel_setpoint_.twist.linear.z  = dz_sp;     // z velocity setpoint (usually 0)
  fcu_vel_setpoint_.twist.angular.z = dyaw_sp;   // yaw setpoint

  ROS_INFO("setpoint: [x: %f y:%f z: %f]", dx_sp, dy_sp, dz_sp);
};

void CtrlPx4::forward(float distance) { moveToPoint(distance, 0, 0, 0); }

void CtrlPx4::backward(float distance) { moveToPoint(-distance, 0, 0, 0); };

void CtrlPx4::left(float distance) { moveToPoint(0, distance, 0, 0); }

void CtrlPx4::right(float distance) { moveToPoint(0, -distance, 0, 0); }

void CtrlPx4::up(float distance) { moveToPoint(0, 0, distance, 0); }

void CtrlPx4::down(float distance) { moveToPoint(0, 0, -distance, 0); }

void CtrlPx4::yawLeft(float radian) { moveToPoint(0, 0, 0, radian); }

void CtrlPx4::yawRight(float radian) { moveToPoint(0, 0, 0, -radian); }
