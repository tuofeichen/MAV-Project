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


// #include <synch.h>


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
// static SURF fdem(descriptorRatio, minMatches, sufficientMatches);
static SURF objdem(0.7, 35, sufficientMatches);
static OrbDetSurfDesc fdem(descriptorRatio, maxNrOfFeatures, minMatches, sufficientMatches);
// static SIFT fdem(descriptorRatio, minMatches, sufficientMatches);

static RANSACBasedTME tme(maxRansacIterations, maxDistanceForInlier, thresholdAbsolutDistanceTest, sufficentPercentageOfInlier, minNrOfInlier);
static G2oPoseGraph graph;

int main(int argc, char **argv)
{
	// init
	ros::init(argc,argv,"rgbd_backend");
	initLog();
	RosHandler px4;
	boost::mutex backendMutex;
	// start camera
	AsusProLiveOpenNI2::start();
	Backend backend(backendPort,backendMutex);


// threading for processFrame
	boost::thread t_procFrame;
	boost::thread t_slam;

	pcl::console::TicToc time;
	Frame frame;
	int nodeId = 1; 				// node id (for debug mode?)
	int badFrameCnt = 0;
	bool badFrame = 0;			// bad frame flag
  bool noError = false;		// grab frame
	double timeDiff = 0; 		// debug processing time


	static Eigen::Matrix4f rot; // transform SLAM frame to PCL frame
		rot = (Eigen::Matrix4f() <<
				  0,1,0,0,
				  0,0,1,0,
				  1,0,0,0,
				  0,0,0,1).finished();

	Eigen::Isometry3d tm;			// slam frame tm
	Eigen::Matrix<double,6,6> im;

	Eigen::Matrix4f lpe_tm; 	// pixhawk fusion tm
	Eigen::Matrix4f lpe_prev;

	// setup mapping class
	Mapping slam(&fdem, &tme, &graph, &px4);//,&pointCloudMap); //, &pointCloudMap);
	ObjDetection obj(&objdem,&px4);


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

// in debug mode,read stored images (video frames)
// otherwise use camera input
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
		//
		// readMutex.lock();
		// cond_signal(&readFrameDone);
		// readMutex.unlock();
#endif

		if(nodeId > -1)
		{
			// start slam

			// put object detection into separate thread， sort of dependecncy
			slam.addFrame(frame);
			// obj.processFrame(frame, readFrameDone);

			if(slam.extractFeature()){
				t_slam 			= boost::thread (&Mapping::run, &slam);
				t_procFrame = boost::thread (&ObjDetection::processFrame, &obj, boost::ref(frame));
				t_slam.join();
				// time.tic();
				t_procFrame.join();// wait for procFrame to finish (shouldn't be an issue)
				// cout << "obj detect takes " << time.toc() << " ms " << endl;
			}

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
// #ifndef DEBUG
// 		cv::waitKey(30);
// 		if(frame.getNewNodeFlag()){
// 		  saveImage(frame,nodeId);
// 			nodeId ++ ; // increase id
// 		}
// #else
// 		cv::waitKey(0);
// #endif

// New Node Processing (currently only logging and timing)
			if (frame.getNewNodeFlag())
			{
			  logSlamNode(frame, slam.getCurrentPosition(),nodeId,badFrame);
			  // logLpeNode(px4.getLpe(), frame.getTime(),nodeId,badFrame);
				timeDiff = time.toc();
				cout << "total processing time " << timeDiff << endl << endl;
			}

// Keyframe Processing (Add New Node to backend PCL)
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
