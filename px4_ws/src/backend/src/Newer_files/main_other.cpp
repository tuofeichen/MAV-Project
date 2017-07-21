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
// #include "PointCloudMap.h"
#include "RANSACBasedTME.h"
#include "G2oPoseGraph.h"
#include "AsusProLiveOpenNI2.h"


#include "RosHandler.h" // Ros stuff 


using namespace std;
using namespace SLAM;

// #define DEBUG

// 
//
// settings
//
static constexpr char frontEndIp[] = "192.168.144.11"; // WLAN
//static constexpr char frontEndIp[] = "127.0.0.1"; // local host

enum {
	frontEndPortNr = 11000, ///< port of the front end
	minMatches = 15, ///< minimal number of matches
	maxNrOfFeatures = 600, ///< maximal number of features to detect (only needed for OrbDetSurfDesc)
	sufficientMatches = 600, ///< sufficient number of matches to return
	maxRansacIterations = 1000, ///< maximal number of RANSAC iterations
	minNrOfInlier = 15 ///< minimal number of inlier for the RANSAC
};

static constexpr float maxDistanceForInlier = 0.02f; ///< maximal distance for inlier in meter (RANSAC)
static constexpr float thresholdAbsolutDistanceTest = 0.03f; ///< threshold for the absolute distance test in meter (RANSAC)
static constexpr float sufficentPercentageOfInlier = 0.95f; ///< percentage of inlier to terminate the RANSAC
static constexpr float descriptorRatio = 0.15f; ///< feature match ratio
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
static std::ofstream logKpts;

static int frameNum = 0;

static void logPoseGraphNode(const Frame& frame, const Eigen::Isometry3d& pose)
{
	Eigen::Affine3d trafo(pose.affine());
	Eigen::Quaterniond q(trafo.rotation());
	Matrix3d t = trafo.rotation();

	double r, p, y; 
	
	bool new_node = frame.getNewNodeFlag();

	if (new_node || (frame.getId() == 0)){
	r = atan2(t(2,1),t(2,2)); // roll (around x)
	p = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2))); // pitch (around y)
	y = atan2(t(1,0),t(0,0)); // yaw (around z)

	logPos << to_string(frame.getTime()) << ","
		 << pose.translation().x() << ","
		 << pose.translation().y() << ","
		 << pose.translation().z() << ","
		 << r << ","
		 << p << ","
		 << y  << "," << frameNum << endl; // note down quaternion or rpy? (should probably note down quaternion)

	cout << fixed << setprecision(3);
	cout << "[VSLAM] roll  " <<  r << "  pitch  " << p << " yaw " << y << endl; 
	cout << "[VSLAM] x " <<  pose.translation().x() << "  y " << pose.translation().y() <<" z " << pose.translation().z() << endl;
	}
	
}

void saveImage(Frame frame, int id)
{	
		char imgName [100];
		std::vector<int> params;
    	params.push_back(cv::IMWRITE_PNG_COMPRESSION);
   		params.push_back(9);   // that's compression level, 9 == full , 0 == none

		sprintf(imgName, "/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/backend/Frames/rgb_%d.png",id);
		cv::imwrite(imgName,frame.getGray(),params);
		sprintf(imgName, "/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/backend/Frames/dep_%d.png",id);
		cv::imwrite(imgName,frame.getDepth(),params);

}

void readImage(Frame& frame, int id) // implemented for playback debug
{
	char imgName [100];
	int i = 0; // reiterator
	string buffer;
	string value;  
	double timeStamp; 

	ifstream mylog("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/backend/position.csv");

	boost::shared_ptr<double> time_ptr (new double);
	boost::shared_ptr<cv::Mat> rgb_ptr(new cv::Mat);
	boost::shared_ptr<cv::Mat> gray_ptr(new cv::Mat);
	boost::shared_ptr<cv::Mat> dep_ptr(new cv::Mat);

	sprintf(imgName, "/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/backend/Frames/rgb_%d.png",id);
	cv::Mat img1 = cv::imread(imgName, CV_LOAD_IMAGE_GRAYSCALE);
	sprintf(imgName, "/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/backend/Frames/dep_%d.png",id);
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
	
	return;  

}

int main(int argc, char **argv)
{
	//
	// init
	ros::init(argc,argv,"rgbd_backend");
	RosHandler px4;  // pixhawk communication via mavros

	Frame frame, frame_prev;
	int toNode = 1; // node id
	int badFrameCnt = 0;
	int noTrafoCnt  = 0; 


	Eigen::Isometry3d tm;
	Eigen::Matrix<double,6,6> im;

	Eigen::Matrix4f lpe_tm; 	// pixhawk fusion tm 
	Eigen::Matrix4f lpe_prev; 	//
	cv::FileStorage fileStore("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/backend/keypoints.yml",cv::FileStorage::WRITE);
 	char fileName[100];

	// start camera
	//AsusProLiveOpenNI2::start();

	// start viewer and logger
#ifdef DEBUG
	logPos.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/backend/position_debug.csv", std::ofstream::out | std::ofstream::trunc);
	logPos << "time,x,y,z,roll,pitch,yaw" <<endl;

	logKpts.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/backend/keypoints.csv", std::ofstream::out | std::ofstream::trunc);

#else

	logPos.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/backend/position.csv", std::ofstream::out | std::ofstream::trunc);
	logPos << "time,x,y,z,roll,pitch,yaw" <<endl;
#endif	

	// setup mapping class

	Mapping slam(&fdem, &tme, &graph); //, &pointCloudMap);

	
	// pointCloudMap.startMapViewer();

	// set start position on front end
	// frontend.setCurrentPosition(slam.getCurrentPosition());

	// process frames
	bool stop = false;
	bool noError = false; 

	// while(frontend.running() && !stop)
	// stop communication 
	
#ifndef DEBUG
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
		// int toNode = frontend.getNewNode(frame, tm, im);

		ros::spinOnce(); // get up-to-date lpe regardless 

#ifdef DEBUG

		readImage(frame,toNode);//toNode is node number
		toNode ++ ;
#else

		noError = AsusProLiveOpenNI2::grab(frame);

		if(!noError)
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
			continue;
		}

#endif	
		
		if(toNode > -1)
		{
			// visualization
			// cv::imshow("RGB Image", frame.getRgb());
			// cout << "process node " << toNode << endl; 
			// cv::imshow("Gray Image", frame.getGray());
			const float scaleFactor = 0.05f;
			// cv::Mat depthMap;
			// frame.getDepth().convertTo( depthMap, CV_8UC1, scaleFactor );
			// cv::imshow("Depth Image", depthMap);

			frameNum = toNode - 1;
			cv::waitKey(20);
			// stop |= (cv::waitKey(10) >= 0); //stop when key pressed
			
			// start slam
			
			slam.addFrame(frame);
			slam.run(px4);

			sprintf(fileName,"Matching%d", toNode);
			logKpts << fileName  << endl;
			
			// if (frame.getKeypoints().size())
			// cv::KeyPoint kpts ; 
			// if (frame.getKeypoints().size()>0)
			// {	
			// 	// cout << "start logging keypoints of size " <<frame.getKeypoints().size()<< endl;
			// 	for (int i = 0; i < frame.getKeypoints().size();i++)
			// 	{
			// 		kpts = frame.getKeypoints().at(i);	
			// 		// cout << " key point is " << kpts.pt << endl;
			// 		// cv::write(fileStore, fileName, frame.getKeypoints());
			// 		// cv::write(fileStore, fileName, kpts.pt);
			// 		logKpts << kpts.pt.x << "," << kpts.pt.y << endl;
			// 	}
			// }

			// cout << "dummy frame? " << frame.getDummyFrameFlag() << endl;

			if (slam.getBadFrameCounter() == badFrameCnt)
			{ // valid frame
				tm = slam.getCurrentPosition();
				px4.updateCamPos(frame.getTime(), tm.matrix().cast<float>());
			}
			else
			{
				badFrameCnt = slam.getBadFrameCounter();
				// noTrafoCnt  = slam.getNoTrafoFoundCounter();
			}

			// cv::write(fileStore, fileName, frame.getKeypoints());
			// frontend.setCurrentPosition(slam.getCurrentPosition());
#ifndef DEBUG		
		if(frame.getNewNodeFlag()){
			saveImage(frame,toNode);
			toNode ++ ; // increase id
		}
#endif
			logPoseGraphNode(frame, slam.getCurrentPosition());
			if(frame.getNewNodeFlag())
				px4.getLpe();//log lpe


			
//			pointCloudMap.updateMapViewer();
		}
		else
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}
	}

	// fileStore.release(); 

//	graph.optimize();
//  graph.removeEdgesWithErrorBiggerThen(10.0/(0.02*0.02));
	
	graph.optimizeTillConvergenz();

//pointCloudMap.updateMapViewer();

	graph.save();
	
	logPos.close();
//	pointCloudMap.saveMap("Map.pcd");


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

//	pointCloudMap.showMap();
//	pointCloudMap.stopMapViewer();

// ros::shutdown();

	return 0;
}


void logPoseGraphEnd(Frame frame, G2oPoseGraph graph, int nodeCnt)
{
	// cout << " total number of node ? " << nodeCnt <<  endl; 
	frame.setNewNodeFlag(true);
	for (int i = 0; i < nodeCnt; i++)
	{
	  // cout << "position of id is " << graph.getPositionOfId(i) << endl;
		logPoseGraphNode(frame, graph.getPositionOfId(i));
	} // log pose graph after optimization

}