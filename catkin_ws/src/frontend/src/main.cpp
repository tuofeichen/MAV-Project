/**
 * @file main.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the main function of the back end project.
 *
 */

#include <iostream>
#include <fstream>
#include <string>

#include "opencv2/highgui.hpp" // debug

#include <ros/ros.h>
#include "Frontend.h"
#include "Mapping.h"
#include "OrbDetSurfDesc.h"
#include "SURF.h"
#include "SIFT.h"
#include "PointCloudMap.h"
#include "RANSACBasedTME.h"
#include "G2oPoseGraph.h"
#include "AsusProLiveOpenNI2.h"


#include "Backend.h"
#include "RosHandler.h" // Ros stuff 
// 

using namespace std;
using namespace SLAM;


// 
//
// settings
//
static constexpr char frontEndIp[] = "192.168.144.11"; // WLAN
//static constexpr char frontEndIp[] = "127.0.0.1"; // local host

enum {
	backendPort = 11000, ///< port of the front end
	minMatches = 25, ///< minimal number of matches
	maxNrOfFeatures = 600, ///< maximal number of features to detect (only needed for OrbDetSurfDesc)
	sufficientMatches = 600, ///< sufficient number of matches to return
	maxRansacIterations = 2000, ///< maximal number of RANSAC iterations
	minNrOfInlier = 20 ///< minimal number of inlier for the RANSAC
};

static constexpr float maxDistanceForInlier = 0.01f; ///< maximal distance for inlier in meter (RANSAC)
static constexpr float thresholdAbsolutDistanceTest = 0.02f; ///< threshold for the absolute distance test in meter (RANSAC)
static constexpr float sufficentPercentageOfInlier = 0.65f; ///< percentage of inlier to terminate the RANSAC
static constexpr float descriptorRatio = 0.5f; ///< feature match ratio
static constexpr float voxelSize = 0.02f; ///< voxel grid size


// static objects
//
static SURF fdem(descriptorRatio, minMatches, sufficientMatches);
// static OrbDetSurfDesc fdem(descriptorRatio, maxNrOfFeatures, minMatches, sufficientMatches);
// static SIFT fdem(descriptorRatio, minMatches, sufficientMatches);
static RANSACBasedTME tme(maxRansacIterations, maxDistanceForInlier, thresholdAbsolutDistanceTest, sufficentPercentageOfInlier, minNrOfInlier);
static G2oPoseGraph graph;
// static PointCloudMap pointCloudMap(voxelSize);

//
// log position
//
static std::ofstream logPos;
static std::ofstream logLPE;

static std::ofstream logKpts;
static std::ofstream logKpts3D;

static int valid_update = 0;
static int frameNum = 0;

static void logPoseGraphNode(const Frame& frame, const Eigen::Isometry3d& pose)
{
	Eigen::Affine3d trafo(pose.affine());
	Eigen::Quaterniond q(trafo.rotation());
	Matrix3d t = trafo.rotation();

	double r, p, y; 

	r = atan2(t(2,1),t(2,2)); // roll (around x)
	p = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2))); // pitch (around y)
	y = atan2(t(1,0),t(0,0)); // yaw (around z)

	logPos << to_string(frame.getTime()) << ","
		 << pose.translation().x() << ","
		 << pose.translation().y() << ","
		 << pose.translation().z() << ","
		 << r << ","
		 << p << ","
		 << y  << "," << frameNum << "," << valid_update << endl; // note down quaternion or rpy? (should probably note down quaternion)

	// cout << fixed << setprecision(6);
	cout << "[VSLAM] roll  " <<  r << "  pitch  " << p << " yaw " << y << endl; 
	// cout << "[VSLAM] x     " <<  pose.translation().x() << "  y     " << pose.translation().y() <<" z   " << pose.translation().z() << endl;
	
}

void saveImage(Frame frame, int id)
{	
		char imgName [100];
		std::vector<int> params;
    	params.push_back(cv::IMWRITE_PNG_COMPRESSION);
   		params.push_back(9);   // that's compression level, 9 == full , 0 == none

		sprintf(imgName, "/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/Frames/rgb_%d.png",id);
		cv::imwrite(imgName,frame.getGray(),params);
		sprintf(imgName, "/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/Frames/dep_%d.png",id);
		cv::imwrite(imgName,frame.getDepth(),params);

}

bool readImage(Frame& frame, int id) // implemented for playback debug
{
	char imgName [100];
	int i = 0; // reiterator
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

int main(int argc, char **argv)
{
	// init
	ros::init(argc,argv,"rgbd_backend");
	RosHandler px4;  // pixhawk communication via mavros
	boost::mutex backendMutex;
	Backend backend(backendPort,backendMutex);
	
	pcl::console::TicToc time; // debug 


	Frame frame, frame_prev;
	int nodeId = 1; // node id
	int badFrameCnt = 0;
	double timeDiff = 0; // debug processing time

	Eigen::Isometry3d tm;
	Eigen::Matrix<double,6,6> im;

	Eigen::Matrix4f lpe_tm; 	// pixhawk fusion tm 
	Eigen::Matrix4f lpe_prev; 	
	char fileName[100];

	// start camera
	AsusProLiveOpenNI2::start();
	logKpts3D.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/matlab_util/keypoints3D.csv", std::ofstream::out | std::ofstream::trunc);
	logKpts.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/matlab_util/keypoints.csv", std::ofstream::out | std::ofstream::trunc);


	// start viewer and logger
#ifdef DEBUG
	logPos.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/position_debug.csv", std::ofstream::out | std::ofstream::trunc);
	logPos << "time,x,y,z,roll,pitch,yaw,framenum,valid" <<endl;

#else

	logPos.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/VSLAM.csv", std::ofstream::out | std::ofstream::trunc);
	logPos << "time,x,y,z,roll,pitch,yaw,framenum,valid" <<endl;

	logLPE.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/LPE.csv", std::ofstream::out | std::ofstream::trunc);
	logLPE << "x,y,z,valid" <<endl;

#endif	

	// setup mapping class
	Mapping slam(&fdem, &tme, &graph, &px4); //, &pointCloudMap);

	// process frames
	bool stop = false;
	bool noError = false; 
	
#ifndef DEBUG // initialize camera
	for(int i=0; i < 5; ++i)
	{
		if(!AsusProLiveOpenNI2::grab(frame))
		{
			--i;
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		}
	}
#endif
	

	while(!stop && ros::ok())
	{
		ros::spinOnce(); // get up-to-date lpe regardless 
		time.tic();

#ifdef DEBUG

		if(readImage(frame,nodeId))//nodeId is node number
			nodeId ++ ;
		else
			break;
#else

		noError = AsusProLiveOpenNI2::grab(frame);

		if(!noError)
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
			continue;
		}

#endif	
		
		if(nodeId > -1)
		{
// visualization
			// cv::imshow("RGB Image", frame.getRgb());
			// cout << "process node " << nodeId << endl; 
			// cv::imshow("Gray Image", frame.getGray());
			// const float scaleFactor = 0.05f;
			// cv::Mat depthMap;
			// frame.getDepth().convertTo( depthMap, CV_8UC1, scaleFactor );
			// cv::imshow("Depth Image", depthMap);

			frameNum = nodeId - 1;
			
			// start slam
			slam.addFrame(frame);
			slam.run();

// Debug 
			// sprintf(fileName,"Keypoint Frame `%d", nodeId);
			// logKpts3D << fileName  << endl;
			// logKpts << fileName  << endl;
			
			// Eigen::Vector3f kpts3D ;
			// cv::KeyPoint kpts;

			// if ((frame.getKeypoints().size()>0)&&((frame.getId()== DEBUG_NEW)||(frame.getId()==DEBUG_OLD)))
			// {	
			// 	for (int i = 0; i < frame.getKeypoints3D().size();i++)
			// 	{
			// 		kpts3D = frame.getKeypoints3D().at(i);	
			// 		logKpts3D << kpts3D.x() << "," << kpts3D.y() << "," << kpts3D.z()<<endl;
			// 	}

			// 	for (int i = 0; i< frame.getKeypoints().size();i++)
			// 	{
			// 		kpts = frame.getKeypoints().at(i);
			// 		logKpts << kpts.pt.x << "," << kpts.pt.y << endl;
			// 	}

			// }
// Debug
			valid_update = frame.getBadFrameFlag();
			if (slam.getImuCompensateCounter()== badFrameCnt)
			{ 
				// cout << "publish setpoint" << endl;
				tm = slam.getCurrentPosition();
				px4.updateCamPos(frame.getTime(), tm.matrix().cast<float>()); // publish to mavros
			}
			else	
			{
				// cout << "skip frame due to " << valid_update << endl;
				badFrameCnt = slam.getImuCompensateCounter();
			}

			
#ifndef DEBUG		
		cv::waitKey(30);
		if(frame.getNewNodeFlag()){
			saveImage(frame,nodeId);
			nodeId ++ ; // increase id
		}
#else
		cv::waitKey(0);
#endif


			if(frame.getNewNodeFlag()||(frame.getId() == 0))
			{
				if (frame.getKeyFrameFlag())
					backend.setNewNode(frame);

				for (int i = 0; i<slam.getNodes().size();i++)
				{
					// update optimized graph
					if(slam.getNodes().at(i).getKeyFrameFlag()){
						backend.sendCurrentPos(graph.getPositionOfId(i).matrix().cast<float>());
						boost::this_thread::sleep(boost::posix_time::milliseconds(1));// necessary?
					}
					// maybe redesign this communication
				}
				backend.sendCurrentPos((-1)*Eigen::Matrix<float, 4, 4>::Identity()); // end signal
				logPoseGraphNode(frame, slam.getCurrentPosition());		
			}

			Matrix4f lpe = px4.getLpe();

			logLPE << lpe(0,3) << "," << lpe(1,3) << "," << lpe(2,3) << ","<<valid_update<< endl;
			timeDiff = time.toc();			
		}

		else
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}
	}


	graph.optimizeTillConvergenz();
	graph.save();
	logPos.close();
	logLPE.close();

	// plot statistic
	cout << "Total number of Frames " << slam.getFrameCounter() << endl;
	cout << "Number of bad frames " << slam.getBadFrameCounter() << endl;
	cout << "Number of frames where no transformation was found " << slam.getNoTrafoFoundCounter()<< endl;
	cout << "Number of frames where replaced by a dummy node " << slam.getDummyNodeCounter() << endl;
	cout << "Number of loop closures found " << slam.getDetLoopClsrsCounter() << endl;
	cout << "Number of too small transformations " << slam.getTrafoToSmallCounter() << endl;
	cout << "Number of transformations with too big velocities " << slam.getTrafoVelocityToBigCounter() << endl;
	cout << "Number of key frames " << slam.getKeyFrames().size() << endl;
	cout << "Number of nodes " << slam.getNodes().size() << endl << endl;
	cout << "Current graph Id " << slam.getNodes().back().getId() << endl << endl;


ros::shutdown();

	return 0;
}


void logPoseGraphEnd(Frame frame, G2oPoseGraph graph, int nodeCnt)
{
	frame.setNewNodeFlag(true);
	for (int i = 0; i < nodeCnt; i++)
	{
		logPoseGraphNode(frame, graph.getPositionOfId(i));
	} // log pose graph after optimization

}