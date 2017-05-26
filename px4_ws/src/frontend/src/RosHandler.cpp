#include "RosHandler.h"
#include <math.h>

using namespace Eigen;
using namespace std;

RosHandler::RosHandler()
{
	Matrix4f yaw_offset;
	yaw_offset.setZero();

	_rgbd_slam_pub  = _nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1000);
	_object_pub 	 = _nh.advertise<geometry_msgs::Point>("/objDetect/obj_pose", 100);
	_wall_pub  		 = _nh.advertise<geometry_msgs::Point>("/objDetect/wall_pose", 100);
	_obst_pub		   = _nh.advertise<geometry_msgs::Point>("/objDetect/obst_dist",100);

	_state_sub 		= _nh.subscribe("/px4_offboard/state",  10,  &RosHandler::stateCallback, this);
	_lpe_sub 		= _nh.subscribe("/mavros/local_position/pose",100,&RosHandler::lpeCallback,this);
	_flow_valid_sub = _nh.subscribe("/mavros/global_position/rel_alt",100,&RosHandler::flowValidCallback,this);
	_bat_sub 		= _nh.subscribe("/mavros/battery",10, &RosHandler::batCallback, this);

	_lpe.setIdentity();
	_lpe_cam.setIdentity();
	_time = 0.0 ;
	_time_cam = 0.0;

	_is_takeoff   = 0;
	_is_land = 0;
	_is_fail = 0;
	_lpe_valid = false; // flag to know if LPE is valid

};


RosHandler::~RosHandler()
{
	ros::shutdown();
};


void RosHandler::lpeCallback(const geometry_msgs::PoseStamped pos_read)
{
	Quaternionf q (pos_read.pose.orientation.w, pos_read.pose.orientation.x, pos_read.pose.orientation.y,pos_read.pose.orientation.z);
	_time = pos_read.header.stamp.sec + pos_read.header.stamp.nsec / 1000000000.0;
	_lpe.setIdentity(); 	// clear buffer
	_lpe.topLeftCorner (3,3) = q.matrix();

	// float r, p, y;
	// rot2rpy(q.matrix(),r,p,y);
	// cout << "lpe rpy is " << r << " " << p << " "<< y << endl;

	_lpe.topRightCorner(3,1) << pos_read.pose.position.x, pos_read.pose.position.y, pos_read.pose.position.z;

}

void RosHandler::flowValidCallback(const std_msgs::Float64 data)
{
	_lpe_valid   = true; // not used at this point
	_timeout = _time;
}


void RosHandler::batCallback(const mavros_msgs::BatteryStatus bat)
{
	// if (bat.voltage < 14.5)
	// {
	//   ROS_WARN("Low battery!");
	// }
}

void RosHandler::updateObstacleDistance(geometry_msgs::Point obsDist)
{
	_obst_pub.publish(obsDist);
}

void RosHandler::updateObjPos(geometry_msgs::Point objPos)
{
	_object_pub.publish(objPos);
}

void RosHandler::updateWallPos(geometry_msgs::Point wallPos)
{
	_wall_pub.publish(wallPos);
}

// everthing aligned in camera frame
void RosHandler::updateCamPos(double timeStamp, Matrix4f currentTME) // update slam estimate
{
		Matrix3f rot_mat   =  currentTME.topLeftCorner(3,3);    //  get rotation matrix

		Quaternionf q(rot_mat);
		float r, p, y;
		_xyz =  currentTME.topRightCorner(3,1);   // translation
		rot2rpy(rot_mat, r , p , y); 		  // transformed rpy for debug

//		cout << " transformed r p y is : "   << r << " "  << p << " " << y << endl;
//		cout << " current position is: " << _xyz(0) << " "  << _xyz(1) << " " << _xyz(2) << endl;

		_rgbd_slam_pos.header.stamp.sec  = floor(timeStamp);
		_rgbd_slam_pos.header.stamp.nsec = (timeStamp - floor(timeStamp)) *1000000000.0;

		// cout << "current time sec is " << _rgbd_slam_pos.header.stamp.sec << endl;
		// cout << "current time nsec is " << _rgbd_slam_pos.header.stamp.nsec << endl;

		_rgbd_slam_pos.pose.position.x = _xyz(0);
		_rgbd_slam_pos.pose.position.y = _xyz(1);
		_rgbd_slam_pos.pose.position.z = _xyz(2);

		_rgbd_slam_pos.pose.orientation.w = q.w();
		_rgbd_slam_pos.pose.orientation.x = q.x();
		_rgbd_slam_pos.pose.orientation.y = q.y();
		_rgbd_slam_pos.pose.orientation.z = q.z();

		_rgbd_slam_pub.publish(_rgbd_slam_pos);

}

Matrix4f RosHandler::getLpe() {
		float r, p, y;
		Matrix3f rot = _lpe.topLeftCorner(3,3);
		rot2rpy(rot,r,p,y);

		// cout << "[LPE] rpy :   "<< r <<"   " << p << "   " << y << endl;
		// cout << "[LPE] position : " << _lpe.topRightCorner(3,1).transpose()<< endl << endl;

		return _lpe;
};


void  RosHandler::getTm(Matrix4f& tm, Matrix<float, 6, 6>& im, double&dt)
{
	tm =  _lpe_cam.inverse() * _lpe; // (body frame A = B * T, therefore T = inv(B) * A)
	im = Matrix<float, 6, 6>::Identity() * 5000;
	// TODO get actual covariance matrix from mavlink (need to modify mavros to subscribe)

	dt = _time - _time_cam;
}


// debug utility
void RosHandler::q2rpy(Quaternionf q, float& r, float& p, float& y)
{
	r = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(q.x()*q.x()+q.y()*q.y()));
	p = asin(2*(q.w()*q.y()-q.z()*q.x()));
	y = atan2(2*(q.w()*q.z()+q.y()*q.x()), 1-2*(q.z()*q.z()+q.y()*q.y()));

}


Matrix3f RosHandler::fuseRpy(Matrix3f vRot)
{
	const float lpe_weight = 0.8;
	float r, p, y,r_v,p_v,y_v;
	rot2rpy(_lpe.topLeftCorner(3,3),r,p,y);
	rot2rpy(vRot, r_v,p_v,y_v);

	r = lpe_weight*r + (1-lpe_weight)*r_v;
	p = lpe_weight*p + (1-lpe_weight)*p_v;
	y = lpe_weight*y + (1-lpe_weight)*y_v; // average the rpy with lpe estimate (complimentary filter)

	return rpy2rot(r,p,y);
}

inline Matrix3f RosHandler::rpy2rot(float r, float p, float y)
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

inline void RosHandler::rot2rpy(Matrix3f R,float& r, float& p, float& y)
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
