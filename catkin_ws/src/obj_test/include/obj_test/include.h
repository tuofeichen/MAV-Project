#ifndef INCLUDE_H_
#define INCLUDE_H_

#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <math.h>
#include <Eigen/Dense>
#include <sstream>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include "Frame.h"
#include "AsusProLiveOpenNI2.h"


#include "obj_test/JoyCommand.h"
#include "obj_test/MoveCommand.h"

#define PI 3.14


typedef struct pos_s {
  double px; 
  double py;
  double pz;
  double roll;
  double pitch;
  double yaw;
  Eigen::Vector4f q; 
} my_pos;




#endif