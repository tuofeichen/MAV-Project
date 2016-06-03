#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/MagneticField.h"
#include "px_comm/OpticalFlow.h"
#include "std_msgs/String.h"

using namespace std;

typedef struct lpe_message
{
  float px;
  float py;
  float pz;

}lpeMsg;

typedef struct flow_messgae
{
  float vx;
  float vy;
  float pz;
  int   quality;

}flowMsg;

typedef struct mag_message
{
  float magx;
  float magy;
}magMsg;

class Logger
{
public:
  Logger();
  ~Logger();
  void clearLog();
  string flow_logname;
  string lpe_logname;

  // void csv_dump     (const char*);
private:
  // subscriber callback
  void lpeCallback  (const geometry_msgs::PoseStamped);
  void flowCallback (const px_comm::OpticalFlow);
  void magCallback  (const sensor_msgs::MagneticField);
  magMsg    mag_msgs;
  lpeMsg    lpe_msgs;
  flowMsg   flow_msgs;

  ros::NodeHandle nh;
  ros::Subscriber lpe_sub;
  ros::Subscriber flow_sub;
  ros::Subscriber mag_sub;

};
