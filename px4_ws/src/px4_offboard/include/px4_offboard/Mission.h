#include "ros/ros.h"
#include "px4_offboard/MavState.h"
#include "geometry_msgs/Point.h"


#include "px4_offboard/include.h"
#include "px4_offboard/CtrlPx4.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>

enum {takingoff = 0, calibration, tracking, traverse, turning, landing};
const char* flight_mode_string [landing+1] = {"takeoff","calibration","tracking","traverse","turning","land"};

class Mission
{
public:

Mission();
~Mission(){};

void publish(){ //ROS_INFO("Publishing");
 _mission_ctrl_pub.publish(_objCommand);}; // for timing control

void hover(){resetCommand(_objCommand);_angle_rad = 0;_is_update = 0;};
void takeoff();
void land();
bool update(){return _is_update;};
bool turnLeft90();
void correctTraverseHeight();
void setFlightMode(int flight_mode);
void setControlMode(bool pos_ctrl){ _objCommand.control = pos_ctrl;};

bool getTakeoffFlag(){ return _is_takeoff;     };
bool getLandFlag(){ return _is_land;     };
bool getCalibrateFlag(){return _is_calibrate;};
bool getFailFlag() {return _is_fail;};
int  getFlightMode() {return _flight_mode;};
int  getPrevFlightMode(){return _flight_mode_prev;};
void logSp();


private:

ros::NodeHandle _nh;
ros::Subscriber _wall_sub;
ros::Subscriber _obj_sub;
ros::Subscriber _state_sub;
ros::Subscriber	_obst_sub;
ros::Subscriber _lpe_sub;
ros::Subscriber _vel_sub;
px4_offboard::MavState _objCommand;


int  _flight_mode;
int  _flight_mode_prev;

int  _cali_cnt;
int  _wall_cnt;
int  _obj_cnt;
int  _cannot_find_wall_cnt;
int	 _obst_cnt ; // how many obstacles have the drone run into
bool _obst_found;



bool _is_takeoff;
bool _is_land;
bool _is_calibrate;
bool _is_fail;
bool _is_update;



std::ofstream logMissionSp;

int  _trav_dist         = 900;  // distance to keep away from obstacles / walls for traversal
int  _obj_fail_dist     = 800;  // failsafe distance
int  _track_dist        = 900;  //  object recognition distance
int _traverse_inc = 100;  // (mm)

int _room_size    = 1500;   // (mm)



float _Kpxy   = 0.012;
float _Kpz    = 0.012;

float _Kvz     = 1.8;
float _Kyaw    = 0.1;
float _ang_tol = 0.06; // rad
float _lin_tol = 50;  // mm


float _traverse_height = 0.75;
float _traverse_speed  = 0.35;

// position info
Eigen::Matrix4f _lpe;
Eigen::Matrix<float, 6, 1> _vel;
float _roll, _pitch, _yaw, _yaw_prev, _angle_rad;


void readParam();
void stateCallback(const px4_offboard::MavState state);
void velCallback(const geometry_msgs::TwistStamped vel_read);
void lpeCallback(const geometry_msgs::PoseStamped pos_read);
void wallCallback(const geometry_msgs::Point ang);
void obstCallback(const geometry_msgs::Point msg);
void objCallback(const geometry_msgs::Point);


void rot2rpy(Matrix3f R,float& r, float& p, float& y);
void resetCommand(px4_offboard::MavState& command);

ros::Publisher _mission_state_pub;
ros::Publisher _mission_ctrl_pub;


px4_offboard::MavState _emptyCommand;

};
