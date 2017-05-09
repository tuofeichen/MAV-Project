#ifndef CtrlPx4_H_
#define CtrlPx4_H_

#include "px4_offboard/PID.h"
#include "px4_offboard/MavState.h"

#define AUTO_FLIGHT

enum flightmode_t { MANUAL=0, ALTCTL, POSCTL, AUTO_MISSION, AUTO_LOITER, AUTO_RTL, AUTO_ACRO, OFFBOARD }; // following px4 convention


class CtrlPx4 {

public:
  CtrlPx4();

  bool commandUpdate();

  bool takeoff(double altitude, double velcity);
  bool land(double velocity);
  void hover();

  void forward(float distance);
  void backward(float distance);
  void left (float distance);
  void right(float distance);
  void up(float distance);
  void down(float distance);
  void yawLeft(float radian);
  void yawRight(float radian);
  bool getTakeoffSignal() {return state_set_.takeoff;};

private:


// SETTINGS

// Controller Saturation (meters)
  static constexpr float MAX_Z    = 1.2;
  static constexpr float MAX_DZ   = 1.1;
  static constexpr float MAX_DXY  = 0.5;
  static constexpr float MAX_DYAW = 0.4;

  static constexpr float BAT_LOW_THRESH = 14;


  // subscriber callbacks from MAV
  void stateCallback(const mavros_msgs::State);
  void radioCallback(const mavros_msgs::RCIn);
  void poseCallback(const geometry_msgs::PoseStamped);
  void velCallback(const geometry_msgs::TwistStamped);
  void batCallback(const mavros_msgs::BatteryStatus);


  // subscriber callback from joystick
  void joyCallback(const px4_offboard::MavState);
  void aprilCallback(const px4_offboard::MavState);
  void objCallback(const px4_offboard::MavState);

  // void findObjectCallback(const px4_offboard::MoveCommand move_cmd);

  void updateState();

  // flight controller
  void  moveToPoint (float x_sp, float y_sp, float z_sp, float yaw_sp);
  bool  setMode(int mode);
  bool  setArm (bool arm);

  // state of vehile
  bool sim_;     // disregard all offboard command
  bool off_en_;  // offboard enable flag  (if at sim_, always on, however, no ros node will be sending offboard command)
  bool auto_tl_; // auto takeoff landing flag
  bool pos_ctrl_;
  float tl_height_ = 1.0;

  // controller
  PID pid_takeoff;
  PID pid_land;
  PID pid_object;

  // setpoint related
  float prev_yaw_sp_;
  my_state state_set_{0, 0, 0, 0}, state_read_{0, 0, 0, 0};
  my_pos home_;
  my_pos pos_read_;
  my_pos prev_pos_read_;
  my_vel vel_;

  // node initialization
  ros::NodeHandle nh_;

  // Publisher
  ros::Publisher mavros_vel_pub_;
  ros::Publisher mavros_pos_pub_;
  ros::Publisher mavros_acc_pub_;
  ros::Publisher px4_offboard_pub_;

  // mode change service
  ros::ServiceClient mavros_set_mode_client_;
  ros::ServiceClient mavros_armed_client_;

  // subscriber
  ros::Subscriber local_pos_sub_;
  ros::Subscriber local_vel_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber radio_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber april_sub_;
  ros::Subscriber bat_sub_;
  ros::Subscriber obj_sub_;
  ros::Subscriber wall_sub;// = nh.subscribe("/PerpendicularWallFind", 100, &wallCallback);

  // publishing msgs
  mavros_msgs::SetMode set_mode_;
  mavros_msgs::CommandBool set_armed_;
  geometry_msgs::TwistStamped fcu_vel_setpoint_;
  geometry_msgs::PoseStamped fcu_pos_setpoint_;

  px4_offboard::MavState controller_state_;

};

#endif
