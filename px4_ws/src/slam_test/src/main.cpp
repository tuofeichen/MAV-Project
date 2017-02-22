 /**
 * @file TestRSLAM.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file tests the SLAM library
 */


#include <RGDBSimulator.h>
#include <iostream>
#include "Debug.h"

// open cv
#include "opencv2/highgui.hpp"

//HAL
#include "AsusProLive.h"
#include "G2oPoseGraph.h"
#include "ORB.h"
#include "SURF.h"
#include "SIFT.h"
#include "OrbDetSurfDesc.h"
#include "RANSACBasedTME.h"
#include "PointCloudMap.h"
#include "Mapping.h"

// using namespaces
using namespace std;
using namespace HAL;
using namespace SLAM;

static std::ofstream logPos;
static std::ofstream logPerf;// performance log

int main_wrap(float dRatio, float ransacRatio);
static void logPoseGraphNode(const Frame& frame, const Eigen::Isometry3d& pose);
// //
int main()
{
	logPerf.open("param_id.csv", std::ofstream::out | std::ofstream::trunc);
	logPerf << "descriptor ratio, ransac ratio,mean proc time, max proc time, error" <<endl;

	for (float aRatio = 0.5; aRatio < 1; aRatio += 0.1)
	  for (float ransacRatio = 0.5; ransacRatio < 0.95; ransacRatio += 0.1)
		{
			cout << "Descriptor Ratio " << aRatio << " RANSAC Ratio is " << ransacRatio << endl;
			main_wrap(aRatio,ransacRatio);
		}

	logPerf.close();
}

static void logPoseGraphNode(const Frame& frame, const Eigen::Isometry3d& pose)
{
	Eigen::Affine3d trafo(pose.affine());
	Eigen::Quaterniond rotation(trafo.rotation());
	logPos << to_string(frame.getTime()) << " "
		 << pose.translation().x() << " "
		 << pose.translation().y() << " "
		 << pose.translation().z() << " "
		 << rotation.x() << " "
		 << rotation.y() << " "
		 << rotation.z() << " "
		 << rotation.w() << endl;
}

int main_wrap(float dRatio,float rRatio)
// int main()
{
	// float dRatio = 0.8;
	//
	// downloaded datasets
	// note: to run the the datasets successfully check that the Frame class of the SLAM library uses the correct intinsic paramters (f_x, f_y, c_x, c_y) and depth scale factor
	//
	string folder = "/home/tuofeichen/SLAM/MAV-Project/px4_ws/src/slam_test/simData/rgbd_dataset_freiburg1_desk/";
	RGDBSimulator rgbdSensor(folder);

	// Features
	//
	// OrbDetSurfDesc fdem(0.7f, 600, 50, 600); // small ratio --> more accurate position, but more images dropped and less position estimates (fast parts missing)
  SURF fdem(dRatio, 50, 600); // small ratio --> more accurate position, but more images dropped and less position estimates (fast parts missing)
	// SIFT fdem(0.8f, 50, 600);
	// ORB fdem(0.9f, 500, 50, 600);

	RANSACBasedTME tme(1000, 0.02, 0.03, rRatio, 30);
	PointCloudMap map3d(0.02f);
	G2oPoseGraph go;

	Mapping slam(&fdem, &tme, &go, &map3d);
	IRGBDSensor& sensor = rgbdSensor;

	boost::shared_ptr<cv::Mat> rgbImage;
	boost::shared_ptr<cv::Mat> grayImage;
	boost::shared_ptr<cv::Mat> depthImage;
	boost::shared_ptr<double> timeStamp;

	//
	// start rgdb sensor
	//
	while (!sensor.start())
	{
		sleep(5);
	}

	// logPos.open("position.txt", std::ofstream::out | std::ofstream::trunc);
	// map3d.startMapViewer();

	bool stop = false;
	while(!stop) // endless
	{

		// grabb frame
		stop = !sensor.grab(rgbImage, grayImage, depthImage, timeStamp);

		//
		// imshow
		// cv::imshow("RGB Image", *rgbImage);
		// cv::imshow("Gray Image", *grayImage);
		// const float scaleFactor = 0.05f;
		// cv::Mat depthMap;
		// depthImage->convertTo( depthMap, CV_8UC1, scaleFactor );
		// cv::imshow("Depth Image", depthMap);
		stop |= (cv::waitKey(1) >= 0); //stop when key pressed

		// start slam
		Frame frame(rgbImage, grayImage, depthImage, timeStamp);
		slam.addFrame(frame);
		slam.run();

		// map3d.updateTrajectory(slam.getCurrentPosition());
		// logPoseGraphNode(frame, slam.getCurrentPosition());
		// map3d.updateMapViewer();
	}

	sensor.stop();

	// plot statistic
	cout << "Total number of Frames " << slam.getFrameCounter() << endl;
	cout << "Number of bad frames " << slam.getBadFrameCounter() << endl;
	cout << "Number of frames where no transformation was found " << slam.getNoTrafoFoundCounter() << endl;
	cout << "Number of frames where replaced by a dummy node " << slam.getDummyNodeCounter() << endl;
	cout << "Number of loop closures found " << slam.getDetLoopClsrsCounter() << endl;
	cout << "Number of too small transformations " << slam.getTrafoToSmallCounter() << endl;
	cout << "Number of transformations with too big velocities " << slam.getTrafoVelocityToBigCounter() << endl;
	cout << "Number of key frames " << slam.getKeyFrames().size() << endl;
	cout << "Number of nodes " << slam.getNodes().size() << endl << endl;
	cout << "Current graph Id " << slam.getNodes().back().getId() << endl << endl;
	cout << "Mean time for frame processing: " << slam.getFrameProcMeanTime() << endl;
	cout << "Max time for frame processing: " << slam.getFrameProcMaxTime() << endl << endl;

	// logPos.close();
	// map3d.saveMap("Map.pcd");
	// map3d.saveTrajectory("Traj.pcd");
	// map3d.showMap();

	// optimize
	go.optimizeTillConvergenz();

	// remove edges with big errors
//	while(go.removeEdgesWithErrorBiggerThen(edgeErrorThreshold))
//		go.optimizeTillConvergenz();


	// Save optimized map and position
	map3d.clearTrajectory();
	cout << "start final graph optimization" << endl;
	logPos.open("positionOpt.txt", std::ofstream::out | std::ofstream::trunc);
	int key = 0;
	for(int i=0; i < static_cast<int>(slam.getNodes().size()); ++i)
	{
		const Eigen::Isometry3d& pose = go.getPositionOfId(slam.getNodes().at(i).getId());
		logPoseGraphNode(slam.getNodes().at(i), pose);
		map3d.updateTrajectory(pose);
		if(slam.getNodes().at(i).getKeyFrameFlag())
			map3d.updatePose(key++, pose);
		map3d.updateMapViewer();
	}

	logPos.close();
	map3d.saveTrajectory("OptTraj.pcd");
	map3d.saveMap("OptMap.pcd");

	cout << "SLAM done." << endl << endl;

	//
		string evalCommand = "python /home/tuofeichen/SLAM/MAV-Project/px4_ws/src/slam_test/log/evaluate_rpe.py ";

		evalCommand += folder;
		evalCommand += "groundtruth.txt ";
		evalCommand += " /home/tuofeichen/SLAM/MAV-Project/px4_ws/src/slam_test/log/positionOpt.txt";
		cout << "evaluation result with respect to the ground truth is: " << endl;
		// system(evalCommand.c_str());
		string rpe_error = exec(evalCommand.c_str());

		logPerf << dRatio << "," << rRatio << "," <<  slam.getFrameProcMeanTime() << "," \
		<< slam.getFrameProcMaxTime() << "," << rpe_error;

		cout << rpe_error << endl;

	// map3d.showMap();
	// map3d.stopMapViewer();

	return 0;
}
