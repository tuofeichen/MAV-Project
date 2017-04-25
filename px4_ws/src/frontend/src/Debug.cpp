/**
 * @file Debug.cpp
 * @author Tuofei Chen (IVPL)
 * Debug Utilities (Log position, Save Frames, Save Pose graph)
 *
 *
 */
#include "Debug.h"

using namespace SLAM;
using namespace std;
using namespace cv;


static std::ofstream logSlamPos;
static std::ofstream logLpePos;
// static std::ofstream logKpts;
// static std::ofstream logKpts3D;
void initLog()
{
		logSlamPos.open("/home/odroid/MAV-Project/log/position_slam.csv", std::ofstream::out | std::ofstream::trunc);
		logSlamPos << "time,x,y,z,roll,pitch,yaw,framenum,valid" <<endl;

		logLpePos.open("/home/odroid/MAV-Project/log/position_lpe.csv", std::ofstream::out | std::ofstream::trunc);
		logLpePos << "time,x,y,z,roll,pitch,yaw,framenum,valid" <<endl;
}

void logSlamNode(const Frame& frame, const Eigen::Isometry3d& pose,int frameNum, bool validUpdate)
{
	Eigen::Affine3d trafo(pose.affine());
	Eigen::Quaterniond q(trafo.rotation());
	Matrix3d t = trafo.rotation();

	double r, p, y;

	r = atan2(t(2,1),t(2,2)); // roll (around x)
	p = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2))); // pitch (around y)
	y = atan2(t(1,0),t(0,0)); // yaw (around z)

	logSlamPos << to_string(frame.getTime()) << ","
		 << pose.translation().x() << ","
		 << pose.translation().y() << ","
		 << pose.translation().z() << ","
		 << r << ","
		 << p << ","
		 << y  << "," << frameNum << "," << validUpdate << endl; // note down quaternion or rpy? (should probably note down quaternion)

	// cout << fixed << setprecision(4);
	// cout << endl << "[VSLAM] roll  " <<  r << "  pitch  " << p << " yaw " << y << endl;
	// cout << "[VSLAM] x     " <<  pose.translation().x() << "  y     " << pose.translation().y() <<" z   " << pose.translation().z() << endl;

}

void logLpeNode(Matrix4f lpe, double time, int frameNum, bool validUpdate)
{
	double r, p, y;
	r = atan2(lpe(2,1),lpe(2,2)); 									// roll (around x)
	p = atan2(-lpe(2,0),sqrt(lpe(2,1)*lpe(2,1)+lpe(2,2)*lpe(2,2))); // pitch (around y)
	y = atan2(lpe(1,0),lpe(0,0));

	logSlamPos << to_string(time) << ","
		 << lpe(0,3) << ","
		 << lpe(1,3) << ","
		 << lpe(2,3) << ","
		 << r << ","
		 << p << ","
		 << y  << "," << frameNum << "," << validUpdate <<",";// << endl; // note down quaternion or rpy? (should probably note down quaternion)

	cout << fixed << setprecision(4);
	cout << endl << "[LPE] roll  " <<  r << "  pitch  " << p << " yaw " << y << endl;
	cout << "[LPE] x     " <<  lpe(0,3)  << "  y     " << lpe(1,3)  <<" z   " <<lpe(2,3)  << endl;

}


void logPoseGraphEnd(Frame frame, G2oPoseGraph graph, int nodeCnt)
{
	frame.setNewNodeFlag(true);
	for (int i = 0; i < nodeCnt; i++)
	{
		logSlamNode(frame, graph.getPositionOfId(i),i,1);
	} // log pose graph after optimization

}

void showImage(Frame frame)
{
	cv::imshow("Gray Scale Image",frame.getGray());
	cv::Mat depthMap;
	const float scaleFactor=0.05;
	frame.getDepth().convertTo(depthMap, CV_8UC1, scaleFactor);
	cv::imshow("Depth Image",depthMap);
}


void saveImage(Frame frame, int id)
{
		char imgName [100];
		std::vector<int> params;
    	params.push_back(cv::IMWRITE_PNG_COMPRESSION);
   		params.push_back(9);   // compression level, 9 == full , 0 == none

		sprintf(imgName, "/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/Frames/rgb_%d.png",id);
		cv::imwrite(imgName,frame.getGray(),params);
		sprintf(imgName, "/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/Frames/dep_%d.png",id);
		cv::imwrite(imgName,frame.getDepth(),params);

}

bool readImage(Frame& frame, int id) // implemented for playback debug
{
	char imgName [100];
	int i = 0;
	string buffer;
	string value;
	double timeStamp;

	ifstream mylog("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/position.csv");

	boost::shared_ptr<double> time_ptr (new double);
	boost::shared_ptr<cv::Mat> rgb_ptr(new cv::Mat);
	boost::shared_ptr<cv::Mat> gray_ptr(new cv::Mat);
	boost::shared_ptr<cv::Mat> dep_ptr(new cv::Mat);

	sprintf(imgName, "/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/Frames/rgb_%d.png",id);
	cv::Mat img1 = cv::imread(imgName, CV_LOAD_IMAGE_GRAYSCALE);
	sprintf(imgName, "/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/Frames/dep_%d.png",id);
	cv::Mat map1 = cv::imread(imgName, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

	img1.copyTo(*gray_ptr);
	map1.copyTo(*dep_ptr);


	while(mylog.good() && (i < id))
	{
		i++;
		getline(mylog,buffer); 	   // skip line
		getline(mylog, value,','); // read time stamp

	}
	timeStamp = stod(value);


	*time_ptr = timeStamp;
	frame = Frame(rgb_ptr,gray_ptr,dep_ptr,time_ptr);// empty RGB-image

	return (!img1.empty());

}
