#ifndef INCLUDE_H_
#define INCLUDE_H_

#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <math.h>
#include <Eigen/Dense>
#include <mavros_msgs/State.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/RCIn.h>
#include "std_msgs/String.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Byte.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "px4_offboard/MavState.h"
// #include "px4_offboard/MoveCommand.h"
#include <sstream>
#include <string.h>

using namespace Eigen;
using namespace std;

enum control_t { POS = 0, VEL, RAW };

typedef struct vel_s {
  double vx;
  double vy;
  double vz;
} my_vel;

typedef struct pos_s {
  double px; // current POSITION + yaw
  double py;
  double pz;
  double roll;
  double pitch;
  double yaw;
  Vector4f q; // current rotation
} my_pos;

typedef struct state_s {
  bool dirty;
  bool arm;
  bool failsafe;
  bool offboard;
  bool takeoff;
  bool land;

  int mode;
  int prev_mode;
  // control_t control;


  // need differential for drift cancellation
  // my_pos pos_read;
  // my_pos pos_set;
  // my_pos prev_pos_read;
  // my_vel vel;

} my_state;

#endif
