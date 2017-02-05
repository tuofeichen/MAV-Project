/**
 * @file main.cpp
 * @author Gian Danuser & Michael Eugster (HSR/IVPL)
 * @author Tuofei Chen (IVPL)
 * main function for fronend implementation
 * realization of SLAM workflow
 *
 */


#include <iostream>
#include <fstream>
#include <string>
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

#include "Debug.h"
#include "Backend.h"
#include "RosHandler.h"
#include "ObjDetection.h"


using namespace std;
using namespace SLAM;



// settings
//

static constexpr char frontEndIp[] = "192.168.144.11"; // WLAN
//static constexpr char frontEndIp[] = "127.0.0.1"; // local host

enum {
	backendPort = 11000, ///< port of the front end
	minMatches = 25, ///< minimal number of matches
	maxNrOfFeatures = 600, ///< maximal number of features to detect (only needed for OrbDetSurfDesc)
	sufficientMatches = 600, ///< sufficient number of matches to return
	maxRansacIterations = 1500, ///< maximal number of RANSAC iterations
	minNrOfInlier = 20 ///< minimal number of inlier for the RANSAC
};

static constexpr float maxDistanceForInlier = 0.01f; ///< maximal distance for inlier in meter (RANSAC)
static constexpr float thresholdAbsolutDistanceTest = 0.025f; ///< threshold for the absolute distance test in meter (RANSAC)
static constexpr float sufficentPercentageOfInlier = 0.65f; ///< percentage of inlier to terminate the RANSAC
static constexpr float descriptorRatio = 0.65f; ///< feature match ratio
static constexpr float voxelSize = 0.02f; ///< voxel grid size


// static objects
static SURF fdem(descriptorRatio, minMatches, sufficientMatches);
static SURF objdem(0.7, 35, sufficientMatches);
// static OrbDetSurfDesc fdem(descriptorRatio, maxNrOfFeatures, minMatches, sufficientMatches);
// static SIFT fdem(descriptorRatio, minMatches, sufficientMatches);

static RANSACBasedTME tme(maxRansacIterations, maxDistanceForInlier, thresholdAbsolutDistanceTest, sufficentPercentageOfInlier, minNrOfInlier);
static G2oPoseGraph graph;
// static PointCloudMap pointCloudMap(voxelSize);


int main(int argc, char **argv)
{
	// init
	ros::init(argc,argv,"rgbd_backend");
	RosHandler px4;
	boost::mutex backendMutex;
	// start camera
	AsusProLiveOpenNI2::start();

	Backend backend(backendPort,backendMutex);

	pcl::console::TicToc time; // debug
	Frame frame;

	int nodeId = 1; // node id (for debug mode?)
	int badFrameCnt = 0;
	bool badFrame = 0;
	double timeDiff = 0; // debug processing time

	static Eigen::Matrix4f rot; // transform back for PCL
		rot = (Eigen::Matrix4f() <<
				  0,1,0,0,
				  0,0,1,0,
				  1,0,0,0,
				  0,0,0,1).finished();

	Eigen::Isometry3d tm;
	Eigen::Matrix<double,6,6> im;

	Eigen::Matrix4f lpe_tm; 	// pixhawk fusion tm
	Eigen::Matrix4f lpe_prev;
	char fileName[100];

// logKpts3D.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/matlab_util/keypoints3D.csv", std::ofstream::out | std::ofstream::trunc);
// logKpts.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/matlab_util/keypoints.csv", std::ofstream::out | std::ofstream::trunc);

// start viewer and logger
// #ifdef DEBUG
// 	logPos.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/position_debug.csv", std::ofstream::out | std::ofstream::trunc);
// 	logPos << "time,x,y,z,roll,pitch,yaw,framenum,valid,x_lpe,y_lpe,z_lpe,r_lpe,p_lpe,y_lpe" <<endl;
//
// #else
// 	logPos.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/frontend/VSLAM.csv", std::ofstream::out | std::ofstream::trunc);
// 	logPos << "time,x,y,z,roll,pitch,yaw,framenum,valid,x_lpe,y_lpe,z_lpe,r_lpe,p_lpe,y_lpe" <<endl;
// #endif

	// setup mapping class
	Mapping slam(&fdem, &tme, &graph, &px4);//,&pointCloudMap); //, &pointCloudMap);
	ObjDetection obj(&objdem,&px4);

	// process frames
	bool noError = false;

#ifndef DEBUG // initialize camera
	for(int i=0; i < 15; ++i) // grab a bit more frame
	{
		if(!AsusProLiveOpenNI2::grab(frame))
		{
			--i;
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		}
	}
#endif


	while(ros::ok())
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
			// start slam
			slam.addFrame(frame);
			// slam workflow (matching -> Trafo -> pose graph)
			slam.run();
			// object detection
			obj.processFrame(frame);

			badFrame = frame.getBadFrameFlag();
			if (slam.getImuCompensateCounter()== badFrameCnt)
			{
				tm = slam.getCurrentPosition();
				if (!px4.getTakeoffFlag())
					px4.updateCamPos(frame.getTime(), tm.matrix().cast<float>()); // publish to mavros
			}
			else
			{
				badFrameCnt = slam.getImuCompensateCounter();
			}

// Debug Mode (Use cameara infeed or dataset)
#ifndef DEBUG
		cv::waitKey(30);
		if(frame.getNewNodeFlag()){
		  saveImage(frame,nodeId);
			nodeId ++ ; // increase id
		}
#else
		cv::waitKey(0);
#endif

// New Node Processing (currently only logging and timing)
			if (frame.getNewNodeFlag())
			{
				Matrix4f lpe = px4.getLpe();
				double r_lpe, p_lpe, y_lpe;
				r_lpe = atan2(lpe(2,1),lpe(2,2)); 									// roll (around x)
				p_lpe = atan2(-lpe(2,0),sqrt(lpe(2,1)*lpe(2,1)+lpe(2,2)*lpe(2,2))); // pitch (around y)
				y_lpe = atan2(lpe(1,0),lpe(0,0)); 									// yaw (around z)
			  logPoseGraphNode(frame, slam.getCurrentPosition(),nodeId,badFrame);
				timeDiff = time.toc();
				// if (frame.getKeyFrameFlag())
					cout << "total processing time " << timeDiff << endl;
			}

// Keyframe Processing (Add New Node in backend PCL)
			if(frame.getKeyFrameFlag()||(frame.getId() == 0)) // send back key frame for PCL
			{

				backend.setNewNode(frame);
				for (int i = 0; i<slam.getKeyFrames().size();i++)
				{
					// update optimized graph
					int keyId = slam.getKeyFrames().at(i).getId();
					Eigen::Matrix4f tm_temp, tm;
					tm =  graph.getPositionOfId(keyId).matrix().cast<float>();
					tm_temp = tm;
					tm.row(0) = tm_temp.row(1) * rot.inverse(); // get back to PCL frame
					tm.row(1) = tm_temp.row(2) * rot.inverse();
					tm.row(2) = tm_temp.row(0) * rot.inverse();
					tm.col(3) =	rot * tm_temp.col(3);
					backend.sendCurrentPos(tm);
					boost::this_thread::sleep(boost::posix_time::milliseconds(1)); // necessary?
				}

				backend.sendCurrentPos((-1)*Eigen::Matrix<float, 4, 4>::Identity()); // end signal
			}

		}

		else
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}
	}


	graph.optimizeTillConvergenz();

	// transmit finalized graph
	backend.setNewNode(slam.getKeyFrames().back());
	for (int i = 0; i<slam.getKeyFrames().size();i++)
	{
		// update optimized graph
		int keyId = slam.getKeyFrames().at(i).getId();
		Eigen::Matrix4f tm_temp, tm;
		tm =  graph.getPositionOfId(keyId).matrix().cast<float>();
		tm_temp = tm;
		tm.row(0) = tm_temp.row(1) * rot.inverse(); // get back to PCL frame... this is so dumb
		tm.row(1) = tm_temp.row(2) * rot.inverse();
		tm.row(2) = tm_temp.row(0) * rot.inverse();
		tm.col(3) =	rot * tm_temp.col(3);

		backend.sendCurrentPos(tm);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));// necessary?
	}

	backend.sendCurrentPos((-1)*Eigen::Matrix<float, 4, 4>::Identity()); // end signal

	graph.save();

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
