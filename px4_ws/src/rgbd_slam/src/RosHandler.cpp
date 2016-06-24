#include "RosHandler.h"
#include <math.h>
#define CAL 100
using namespace Eigen;
using namespace std;

bool g_lpe_update = false;

RosHandler::RosHandler()
{	
	int calibrate = CAL;
	Matrix4f yaw_offset;
	yaw_offset.setZero();

	_rgbd_slam_pub = _nh.advertise<geometry_msgs::PoseStamped>("/rgbd_slam/pose",100);
	_lpe_sub = _nh.subscribe("/mavros/local_position/pose",100,&RosHandler::lpeCallback,this);
	_lpe.setZero();
	_lpe_time = 0.0 ;
	_lpe_yaw_offset = 0.0;

	// frame transformation between UAV and Camera
	_lpe2cam = (Matrix4f() << 0,0,1,0,
				  1,0,0,0,
				  0,1,0,0,
				  0,0,0,1).finished();

	// calibrate yaw offset
	while(calibrate)
	{	
		ros::spinOnce();
		if (g_lpe_update){
		calibrate --;
		g_lpe_update = false;
		_lpe_yaw_offset += _rpy(2);
		}
		
	}
	_lpe_yaw_offset /= CAL; 
//	_lpe2cam = (Matrix4f() << 0,-1, 0, 0,
//				  1, 0, 0, 0,
//		        	  0, 0, 1, 0,
//		         	  0, 0, 0, 1).finished() * _lpe2cam; 
	
};


RosHandler::~RosHandler()
{
	ros::shutdown();
};


void RosHandler::lpeCallback(const geometry_msgs::PoseStamped pos_read)
{
	g_lpe_update = true;

	Quaternionf q (pos_read.pose.orientation.w, pos_read.pose.orientation.x, pos_read.pose.orientation.y,pos_read.pose.orientation.z);
	q.normalize();
	q2rpy(q,_rpy(0),_rpy(1),_rpy(2)); // note down body frame rpy
	/*
	AngleAxisf roll  (_rpy(1),Vector3f::UnitX());
	AngleAxisf pitch (_rpy(2),Vector3f::UnitY());
	AngleAxisf yaw   (_rpy(0),Vector3f::UnitZ());
	Quaternionf q_transform = yaw * pitch * roll;
	*/  

	// lpe time stamp
	_lpe_time = pos_read.header.stamp.sec + pos_read.header.stamp.nsec / 1000000000;
	_lpe.setZero(); 	// clear buffer
	_lpe.topLeftCorner(3,3)  = rpy2rot(_rpy(1),_rpy(2),_rpy(0));
	_lpe.topRightCorner(4,1) << pos_read.pose.position.y, pos_read.pose.position.z, pos_read.pose.position.x, 1; 

	// ugly frame transformation... duhhh


}

void RosHandler::updatePos(Matrix4f& currentTME)
{
		Matrix3f rot_mat   =  currentTME.topLeftCorner(3,3);    //  get rotation    matrix
		float r, p, y;
		Quaternionf q(rot_mat); //convert
		_xyz =  currentTME.topRightCorner(3,1);   // original rpy
		cout << " current r p y is : "   << _rpy(0) << " "  << _rpy(1) << " " << _rpy(2) << endl;

		rot2rpy(rot_mat, p , y , r); // transformed rpy
		cout << " transformed r p y is : "   << r << " "  << p << " " << y << endl;

//		cout << " current position is: " << _xyz(0) << " "  << _xyz(1) << " " << _xyz(2) << endl; 

		Vector3f trans_vec  = currentTME.topRightCorner(3,1); //  get translation vector

		_rgbd_slam_pos.pose.position.x = trans_vec(0);
		_rgbd_slam_pos.pose.position.y = trans_vec(1);
		_rgbd_slam_pos.pose.position.z = trans_vec(2);

		_rgbd_slam_pos.pose.orientation.w = q.w();
		_rgbd_slam_pos.pose.orientation.x = q.x();
		_rgbd_slam_pos.pose.orientation.y = q.y();
		_rgbd_slam_pos.pose.orientation.z = q.z();
		_rgbd_slam_pub.publish(_rgbd_slam_pos);


}

// debug utility
void RosHandler::q2rpy(Quaternionf q, float& r, float& p, float& y)
{
	r = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(q.x()*q.x()+q.y()*q.y()));
	p = asin(2*(q.w()*q.y()-q.z()*q.x()));
	y = atan2(2*(q.w()*q.z()+q.y()*q.x()), 1-2*(q.z()*q.z()+q.y()*q.y()));

}

Matrix3f RosHandler::rpy2rot(float r, float p, float y)
{
	Matrix3f roll, pitch, yaw;
	roll.setIdentity();
	pitch.setIdentity();
	yaw.setIdentity();

	roll(1,1)  = cos(r);  roll(2,1)  = sin(r);  roll(1,2)  = -sin(r); roll(2,2)   = cos(r);
	pitch(0,0) = cos(p);  pitch(0,2) = sin(p);  pitch(2,0) = -sin(p); pitch (2,2) = cos(p);
	yaw (0,0)  = cos(y);  yaw(0,1)   = -sin(y); yaw(1,0)   =  sin(y); yaw(1,1)    = cos(y);	
	return yaw*pitch*roll;

}

void RosHandler::rot2rpy(Matrix3f R,float& r, float& p, float& y)
{
	float beta, alpha, gamma;
	beta = atan2(-R(2,0), sqrt(R(0,0)*R(0,0)+R(1,0)*R(1,0)));
	
	if(fabs(beta - 3.1415926/2) < 0.001){
	alpha = 0;
	gamma = atan2(R(0,1),R(1,1));
	}
	else if (fabs(beta + 3.1415926/2) < 0.001){
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





