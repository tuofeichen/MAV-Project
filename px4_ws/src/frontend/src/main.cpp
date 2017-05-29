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
static constexpr bool backEndSupport = false;
//static constexpr char frontEndIp[] = "127.0.0.1"; // local host

enum {
	backendPort = 11000, ///< port of the front end
	minMatches = 30, ///< minimal number of matches
	maxNrOfFeatures = 600, ///< maximal number of features to detect (only needed for OrbDetSurfDesc)
	sufficientMatches = 100, ///< sufficient number of matches to return
	maxRansacIterations = 1500, ///< maximal number of RANSAC iterations
	minNrOfInlier = 20 ///< minimal number of inlier for the RANSAC
};

static constexpr float maxDistanceForInlier = 0.01f; ///< maximal distance for inlier in meter (RANSAC)
static constexpr float thresholdAbsolutDistanceTest = 0.025f; ///< threshold for the absolute distance test in meter (RANSAC)
static constexpr float sufficentPercentageOfInlier = 0.65f; ///< percentage of inlier to terminate the RANSAC
static constexpr float descriptorRatio = 0.65f; ///< feature match ratio
static constexpr float voxelSize = 0.02f; ///< voxel grid size


// static objects
static SURF fdem(descriptorRatio, minMatches, sufficientMatches,100);
static SURF objdem(0.8, 20, sufficientMatches,100);
// static OrbDetSurfDesc fdem(descriptorRatio, maxNrOfFeatures, minMatches, sufficientMatches);
// static OrbDetSurfDesc objdem (descriptorRatio, maxNrOfFeatures, 15, sufficientMatches);
// static SIFT fdem(descriptorRatio, minMatches, sufficientMatches);
// static SIFT objdem(descriptorRatio, 15, sufficientMatches);

static RANSACBasedTME tme(maxRansacIterations, maxDistanceForInlier, thresholdAbsolutDistanceTest, sufficentPercentageOfInlier, minNrOfInlier);
static G2oPoseGraph graph;

int main(int argc, char **argv)
{
	// init
	ros::init(argc,argv,"rgbd_backend");
	initLog();

// start camera
	AsusProLiveOpenNI2::start();

// start backend
	boost::mutex backendMutex;
	Backend backend(backendPort,backendMutex,backEndSupport);


// threads for parallelizing object detection and slam
	boost::thread t_obj;
	boost::thread t_slam;

// Timing Variables
	pcl::console::TicToc time;
	double timeDiff = 0; 		    // debug processing time

// debug mode counter
	int nodeId = 1; 						// node id (for debug mode when reading from dataset)

// position estimate related
	static Eigen::Matrix4f rot; // transform SLAM frame to PCL frame
		rot = (Eigen::Matrix4f() << \
				  0,1,0,0,
				  0,0,1,0,
				  1,0,0,0,
				  0,0,0,1).finished();

	Eigen::Isometry3d pos;			// slam position estimate

	// setup SLAM related objects
	RosHandler px4; 													// roshandler for communication with pixhawk
	Frame frame;	  													// frame class that contains information of each frame
	Mapping slam(&fdem, &tme, &graph, &px4);	// SLAM workflow
	ObjDetection obj(&objdem,&px4);						// object detection workflow


#ifndef DEBUG // initialize camera
	for(int i=0; i < 15; ++i) // grab and discard some inital frames
	{
		if(!AsusProLiveOpenNI2::grab(frame))
		{
			--i;
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		}
	}
	double camInitTime = frame.getTime();
#endif


	while(ros::ok())
	{
		ros::spinOnce(); // get up-to-date lpe regardless
		time.tic();


#ifdef DEBUG // in debug mode,read stored images (video frames)
		if(readImage(frame,nodeId))//nodeId is node number
			nodeId ++ ;
		else
			break;
#else // else grab image from camera
		if(! AsusProLiveOpenNI2::grab(frame)) // no error for grabbing frame
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
			continue;
		}
#endif
			// start slam
			slam.addFrame(frame);
			if(slam.extractFeature() && (!obj.getObjDetectFlag())){ // stop slam when is in tracking mode
				t_slam 		= boost::thread (&Mapping::run, &slam); // only run slam if we have enough feature
			}
			// run object detection all the time (for obstacle detection despite we don't have enough feature)
			t_obj = boost::thread (&ObjDetection::processFrame, &obj, frame);
			t_slam.join();
			t_obj.join();

			if ((slam.getBadFrameFlag() < 1) && (!obj.getObjDetectFlag()))
			{
			  pos = slam.getCurrentPosition();
				if (px4.getArmFlag())  // log only when armed
						logSlamNode(frame, pos, frame.getId(), slam.getBadFrameFlag()); // log the published data
				px4.updateCamPos(frame.getTime() - camInitTime, pos.matrix().cast<float>()); // publish to mavros
			}

// Debug Mode (Use cameara infeed or dataset)
#ifndef DEBUG
		cv::waitKey(20);
		// if(frame.getNewNodeFlag()){
		//   saveImage(frame,nodeId);
		// 	nodeId ++ ; // increase id
		// }
#else
		cv::waitKey(20);
#endif

// New Node Processing (currently only logging and timing)
			if (frame.getNewNodeFlag())
			{
			  // logLpeNode(px4.getLpe(), frame.getTime(),nodeId,badFrame);
				timeDiff = time.toc();
				cout << "total processing time " << timeDiff << endl << endl;
			}

// Keyframe Processing (Add New Node to backend PCL) (can be threaded)
			if((slam.mapUpdate || (frame.getId() == 0)) && backEndSupport) // send back key frame for PCL
			{
				slam.mapUpdate = false;
				backend.setNewNode(frame);
				for (int i = 0; i<slam.getKeyFrames().size();i++)
				{
					// update optimized graph
					int keyId = slam.getKeyFrames().at(i).getId();
					Eigen::Matrix4f tm_temp, pos;
					pos =  graph.getPositionOfId(keyId).matrix().cast<float>();
					tm_temp = pos;
					pos.row(0) = tm_temp.row(1) * rot.inverse(); // convert to PCL frame
					pos.row(1) = tm_temp.row(2) * rot.inverse();
					pos.row(2) = tm_temp.row(0) * rot.inverse();
					pos.col(3) =	rot * tm_temp.col(3);
					backend.sendCurrentPos(pos); // send to backend for point cloud reconstruction
					boost::this_thread::sleep(boost::posix_time::milliseconds(1)); //
				}
				backend.sendCurrentPos((-1)*Eigen::Matrix<float, 4, 4>::Identity()); // end signal
			}

		else
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}
	}

// end of while(1) loop post processing

	graph.optimizeTillConvergenz(); // full graph optimization after video stream ended

	// transmit finalized graph
	if (backEndSupport){
	backend.setNewNode(slam.getKeyFrames().back());
	for (int i = 0; i<slam.getKeyFrames().size();i++)
	{
		// update optimized graph
		int keyId = slam.getKeyFrames().at(i).getId();
		Eigen::Matrix4f tm_temp, pos;
		pos =  graph.getPositionOfId(keyId).matrix().cast<float>();
		tm_temp = pos;
		pos.row(0) = tm_temp.row(1) * rot.inverse(); // get back to PCL frame... this is so dumb
		pos.row(1) = tm_temp.row(2) * rot.inverse();
		pos.row(2) = tm_temp.row(0) * rot.inverse();
		pos.col(3) =	rot * tm_temp.col(3);
		backend.sendCurrentPos(pos);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));// necessary?
	}
		backend.sendCurrentPos((-1)*Eigen::Matrix<float, 4, 4>::Identity()); // end signal
	}

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
