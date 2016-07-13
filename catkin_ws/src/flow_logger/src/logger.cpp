#include "logger.h"

int main(int argv, char **argc)
{
  ros::init (argv, argc, "logger");
  Logger lpe_flow_logger;

  cout << "Input vision log filename: "<<endl;
  cin >> lpe_flow_logger.flow_logname;
  cout << "Input LPE log filename: " <<endl;
  cin >> lpe_flow_logger.lpe_logname;

  lpe_flow_logger.clearLog();
  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


Logger::Logger()
{
  //mag_sub = nh.subscribe("/mavros/imu/mag",1000,&Logger::magCallback, this);
  lpe_sub = nh.subscribe("/mavros/local_position/pose",1000,&Logger::lpeCallback, this);
  flow_sub = nh.subscribe("/rgbd_slam/pose",1000,&Logger::flowCallback,this);
}

Logger::~Logger()
{
  ros::shutdown();
}

void Logger::clearLog()
{
  char* logname_w = new char[lpe_logname.size()+1];
  std::copy(lpe_logname.begin(),lpe_logname.end(), logname_w);
  logname_w[lpe_logname.size()]='\0';
  char* full_name= strcat(logname_w,".csv");
  if (remove(full_name)!=0)
    perror("Error Deleting File");
  else
    puts("Delte lpe log");

  logname_w = new char[flow_logname.size()+1];
  std::copy(flow_logname.begin(),flow_logname.end(), logname_w);
  logname_w[flow_logname.size()]='\0';
  full_name= strcat(logname_w,".csv");
  if (remove(full_name)!=0)
    perror("Error Deleting File");
  else
    puts("Delete Flow log");

// // remove buffer
//     if (remove("mag.csv")!=0)
//       perror("Error Deleting File");
//     else
//       puts("Delte magnetic_field log");


}

void Logger::lpeCallback(const geometry_msgs::PoseStamped lpe)
{

  lpe_msgs.px = lpe.pose.position.x;
  lpe_msgs.py = lpe.pose.position.y;
  lpe_msgs.pz = lpe.pose.position.z;
  // ROS_INFO("Logging LPE: z is %f", lpe_msgs.pz);

  ofstream myfile;
  char* logname_w = new char[lpe_logname.size()+1];
  std::copy(lpe_logname.begin(),lpe_logname.end(), logname_w);
  logname_w[lpe_logname.size()]='\0';
  char* full_name= strcat(logname_w,".csv");

  myfile.open(full_name,ios::in|ios::app);
  myfile<< fixed << setprecision(4)<<lpe_msgs.px<<',';
  myfile<< fixed << setprecision(4)<<lpe_msgs.py<<',';
  myfile<< fixed << setprecision(4)<<lpe_msgs.pz<<',';
  myfile<< fixed << setprecision(6)<<lpe.pose.orientation.w<<',';
  myfile<< fixed << setprecision(6)<<lpe.pose.orientation.x<<',';
  myfile<< fixed << setprecision(6)<<lpe.pose.orientation.y<<',';
  myfile<< fixed << setprecision(6)<<lpe.pose.orientation.z<<',' <<endl;

  myfile.close();
}

// void Logger::flowCallback(const px_comm::OpticalFlow flow)
// {
//   // ROS_INFO("Logging FLOW");
//   flow_msgs.vx = flow.velocity_x;
//   flow_msgs.vy = flow.velocity_y;
//   flow_msgs.pz = flow.ground_distance;
//   flow_msgs.quality = flow.quality;
//   // lpe_flow_logger.csv_dump(flow_logname.c_str());
//   ofstream myfile;
//   char* logname_w = new char[flow_logname.size()+1];
//   std::copy(flow_logname.begin(),flow_logname.end(), logname_w);
//   logname_w[flow_logname.size()]='\0';
//   char* full_name= strcat(logname_w,".csv");
//   myfile.open(full_name,ios::in|ios::app);
//   myfile<< fixed << setprecision(4)<<flow_msgs.vx<<',';
//   myfile<< fixed << setprecision(4)<<flow_msgs.vy<<',';
//   myfile<< fixed << setprecision(4)<<flow_msgs.pz<<',';
//   myfile<< fixed << setprecision(4)<<flow_msgs.quality<<','<<endl;
//   myfile.close();
// }

 void Logger::flowCallback(const geometry_msgs::PoseStamped vision){
  vision_msgs.px = vision.pose.position.x;
  vision_msgs.py = vision.pose.position.y;
  vision_msgs.pz = vision.pose.position.z;

  ofstream myfile;
  char* logname_w = new char[flow_logname.size()+1];
  std::copy(flow_logname.begin(),flow_logname.end(), logname_w);
  logname_w[flow_logname.size()]='\0';
  char* full_name= strcat(logname_w,".csv");
  myfile.open(full_name,ios::in|ios::app);

  myfile<< fixed << setprecision(4)<<vision_msgs.px<<',';
  myfile<< fixed << setprecision(4)<<vision_msgs.py<<',';
  myfile<< fixed << setprecision(4)<<vision_msgs.pz<<',';
  myfile<< fixed << setprecision(6)<<vision.pose.orientation.w<<',';
  myfile<< fixed << setprecision(6)<<vision.pose.orientation.x<<',';
  myfile<< fixed << setprecision(6)<<vision.pose.orientation.y<<',';
  myfile<< fixed << setprecision(6)<<vision.pose.orientation.z<<',' <<endl;

  myfile.close();
}


void Logger::magCallback(const sensor_msgs::MagneticField mag)
{
  ROS_INFO("Logging mag");
  mag_msgs.magx = mag.magnetic_field.x;
  mag_msgs.magy = mag.magnetic_field.y;

  // lpe_flow_logger.csv_dump(flow_logname.c_str());
  ofstream myfile;
  myfile.open("mag.csv",ios::in|ios::app);
  myfile<< fixed << setprecision(8)<<mag.magnetic_field.x<<',';
  myfile<< fixed << setprecision(8)<<mag.magnetic_field.y<<','<<endl;
  myfile.close();
}
