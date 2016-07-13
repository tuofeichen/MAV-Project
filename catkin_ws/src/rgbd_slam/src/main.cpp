/*
 * main.cpp
 *
 *  Created on: May 15, 2015
 *      Author: Gian Danuser & Michael Eugster
 */
#include <iostream>
#include <fstream>
#include <ctime>
#include "RosHandler.h"

#include "TransformTest.cpp"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "VisualOdometry.h"
#include "AsusProLiveOpenNI2.h"
#include "Backend.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace Eigen;
using namespace cv; 

// #define STREAM_ONLY
#define LPE_DUMMY 

//#define DEBUG_ONLY
#define DEBUG
//  Settings

enum {
        setNewNodeAfterNrOfFailedMatches = 1
};

static constexpr int backendPort = 11000;


#ifdef STREAM_ONLY
int main(int argc, char **argv)
{
	ros::init(argc,argv,"rgbd");
	//
	// initialization
	//
	static Frame frame;
	Matrix4f tmCurToNode = Eigen::Matrix4f::Identity();
	Matrix<float, 6, 6> imCurToNode = Matrix<float, 6, 6>::Identity();
	boost::mutex backendMutex;
	Backend backend(backendPort, backendMutex);
	
	//
	// start the RGBD sensor
//	AsusProLive::start();
	AsusProLiveOpenNI2::start();
	
	//
	// grab some images until brightness is adjusted correctly
	for(int i=0; i < 5; ++i)
	{
//		AsusProLive::grab(frame);
		AsusProLiveOpenNI2::grab(frame);
	}
	
	//
	// wait until backend sets the starting position
	while(!backend.currentPosUpToDate()) boost::this_thread::sleep(boost::posix_time::milliseconds(1));

	//
	// first frame
	double start, end, dif;
	while(backend.running())
	{
		
		//debug
		start = static_cast<double>( clock () ) /  CLOCKS_PER_SEC;
		
//		AsusProLive::grab(frame);
		AsusProLiveOpenNI2::grab(frame);
		
		// debug
		end = static_cast<double>( clock () ) /  CLOCKS_PER_SEC;
		dif = (end - start)*1000.0;
		cout << "Grabbing took " << dif << "ms" << endl;

		//debug
		start = static_cast<double>( clock () ) /  CLOCKS_PER_SEC;		
		backend.setNewNode(frame,tmCurToNode,imCurToNode,1);
		
		// debug
		end = static_cast<double>( clock () ) /  CLOCKS_PER_SEC;
		dif = (end - start)*1000.0;
		cout << "Sending took " << dif << "ms" << endl;
	}
//	AsusProLive::stop();
	AsusProLiveOpenNI2::stop();
	return 0;	
}

#else

static std::ofstream logPos;

static bool processFrame(const Frame& newFrame, const Frame& prevNode, Matrix4f& tmCurToNode, Matrix<float, 6, 6>& imCurToNode, bool& newNode)
{


	//
	// estimate transforamtion
	bool validFlag;
	std::vector<int> srcMatches;
	srcMatches = VisualOdometry::estimateTrafo(newFrame, prevNode, tmCurToNode, imCurToNode, validFlag);
	

	VisualOdometry::Result res;


	if (validFlag){
		res = VisualOdometry::checkReliability(tmCurToNode, newFrame.getTime() - prevNode.getTime());
		validFlag = (res != VisualOdometry::invalid); 
	}

	if (validFlag){
	newNode = (res == VisualOdometry::valid);

#ifndef DEBUG		
		if (newNode){
			logPos << endl << "new frame" <<endl; 
			for (int i = 0; i< srcMatches.size(); i++)	
				logPos << srcMatches[i] << ",";
			logPos << endl; 
		}
#endif

		return true;
	}

	else {
		newNode = false;
		return false;
	}
}


static void deleteLog()
{
	char imgName [100];
	for (int i = 0; i< 100; i++)
	{
		sprintf(imgName, "/home/tuofeichen/Downloads/Frames/%d_new_node_rgb.png",i);
		std::remove(imgName);
		sprintf(imgName, "/home/tuofeichen/Downloads/Frames/%d_new_node_dep.png",i);
		std::remove(imgName);
	}
	cout << "done deleting log" << endl;
}


enum{newNode = 0, prevNode = 1};

int main(int argc, char **argv)
{	

#ifndef DEBUG_ONLY
	ros::init(argc,argv,"rgbd");
	RosHandler logger;

	// initialization
	//
	int  state = newNode;
	bool lastFrame = true; // last bad frame handling 

	static Frame frames[2];
	static Matrix4f frontendPos;
	
	cv::FileStorage storage("test.yml", cv::FileStorage::WRITE);
    cv::FileStorage fileStore("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/rgbd_slam/Frames/keypoints.txt",cv::FileStorage::WRITE);
	
	std::vector<int> srcMatches;
	static Matrix4f tmCurToNode;
	static Matrix<float, 6, 6> imCurToNode;

	char imgName [100];

	// lpe related
	double currentTime = 0;; 
	Matrix4f currentPos, cameraPos;
	Matrix3f currentRM;
	Vector3f currentTV;
	frontendPos.setIdentity();
	currentPos.setIdentity();
	currentRM.setIdentity();
	currentTV.setZero();

	deleteLog();

	boost::mutex backendMutex;

	Backend backend(backendPort, backendMutex);
	unsigned int couldNotMatchCounter = 0;
	unsigned int totalNumberOfFrames = 0; // debug
	unsigned int dummyFrameCounter = 0; // debug
	unsigned int badFramesCounter = 0; // debug
	unsigned int totalCouldNotMatchCounter = 0; // debug
	unsigned int newNodeCounter = 0; // debug

//	AsusProLive::start();
	AsusProLiveOpenNI2::start();
	logPos.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/rgbd_slam/Frames/frontend.csv",std::ofstream::out | std::ofstream::trunc);
    logPos << "X,Y,Z,LPE_Z"<<endl;
   // grab some images until brightness is adjusted correctly
	
	for(int i=0; i < 5; ++i)
	{
		if(!AsusProLiveOpenNI2::grab(frames[state]))
		{
			--i;
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		}
	}

	// wait until backend sets the starting position
	while(!backend.currentPosUpToDate()) boost::this_thread::sleep(boost::posix_time::milliseconds(1));

	// first frame
	bool noError = false;
	while(backend.running() && AsusProLiveOpenNI2::running())
	{

    	noError = AsusProLiveOpenNI2::grab(frames[state]);

		if(!noError)
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(10));
			continue;
		}
		else
			++totalNumberOfFrames; // debug

		if(!VisualOdometry::setKeypoints(frames[state]))
		{
			cout << "Init Bad Frame Nr. " << ++badFramesCounter << endl; // debug
			continue;
		}

		tmCurToNode = Eigen::Matrix4f::Identity();
		imCurToNode = Matrix<float, 6, 6>::Identity(); // value not needed
		backend.setNewNode(frames[state],tmCurToNode,imCurToNode,1);
		
		// sprintf(imgName, "/home/tuofeichen/Downloads/Frames/1_new_node_rgb.png");
		// cv::imwrite(imgName,frames[state].getRgb());
		// sprintf(imgName, "/home/tuofeichen/Downloads/Frames/1_new_node_dep.png");
		// cv::imwrite(imgName,frames[state].getDepth());

		sprintf(imgName, "depth1");
		storage << imgName << frames[state].getDepth();
			
		sprintf(imgName, "gray1");
		storage << imgName << frames[state].getGray();

#ifdef DEBUG
		logPos << frontendPos(0,3) << "," << frontendPos(1,3) << "," << frontendPos(2,3) << ",";
		logPos << currentPos(1,3)<< endl; // also note down LPE pose?
#else 
		// logPos << frames[state].getDescriptors() << endl << endl;
		
		// for (int i = 0;i< (frames[state].getKeypoints3D().size());i++)
		// {
		// 	std::vector<Eigen::Vector3f> temp3D =frames[state].getKeypoints3D();
		// 	logPos << temp3D[i] << endl; 
		// }

		// logPos << endl; 


		// logPos << frontendPos(0,3) << "," << frontendPos(1,3) << "," << frontendPos(2,3) << ",";
		// logPos << currentPos(1,3)<< endl; // also note down LPE pose?
		// sprintf(imgName,"keypoint1");	
		// cv::write(fileStore, imgName,frames[state].getKeypoints3D());

#endif
		// first frame
		state = state?0:1; // flip state  
		break;
	}

	newNodeCounter = 1;

	
	// process
	double start, end, dif; // debug
	while(backend.running() && AsusProLiveOpenNI2::running() && ros::ok())
	{
		
		//debug
		start = static_cast<double>( clock () ) /  CLOCKS_PER_SEC;
		
		bool newNode;
		noError = AsusProLiveOpenNI2::grab(frames[state]);

		if(!noError){
        	boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        	continue;
        	}
		else
			++totalNumberOfFrames; // debug
		
		// if (fabs(logger.getTime() - currentTime)> 10) //reinit frozen logger
		// 	{	
		// 		ROS_INFO("reinit time and lpe");
		// 		currentTime = logger.getTime();
		// 		currentPos  = logger.getLpe();
		// 	}

		// TODO debug

		end = static_cast<double>( clock () ) /  CLOCKS_PER_SEC; //debug
		dif = (end - start)*1000.0; // note down grabbing time 

		//cout << "Grabbing took " << dif << "ms" << endl; //debug

		start = static_cast<double>( clock () ) /  CLOCKS_PER_SEC; //debug
			
		if(!VisualOdometry::setKeypoints(frames[state])){	// bad key point
			cout << "Bad Frame Nr. " << ++badFramesCounter << endl; 
			// tmCurToNode = logger.getLpe() * currentPos.inverse();	// get edge from LPE
 
			// // check delta movement
			// if ((logger.getTime()!= 0) && (!VisualOdometry::checkReliability(tmCurToNode,logger.getTime() - currentTime))) 				
			// {			

			// imCurToNode = Matrix<float, 6, 6>::Identity() * 2000; 	// experimental (need adjustable?) value for LPE measurement
			
			// backend.setNewNode(frames[state], tmCurToNode, imCurToNode,1);


			// cout << "[LPE] Node " << ++newNodeCounter << " sent to backend" << endl; //debug
			// //cout << tmCurToNode<<endl;		
			// currentTime = logger.getTime(); 
			// currentPos  = logger.getLpe();

			// frontendPos = frontendPos * tmCurToNode; 
			// logPos << frontendPos(0,3) << "," << frontendPos(1,3) << "," << frontendPos(2,3) << ",";
			// logPos << currentPos(1,3)<< endl; // also note down LPE pose ? 

			// }

			// ros::spinOnce();
			// lastFrame = false; // last frame is bad
			continue;
		}

		if (!lastFrame)
		{
			state = state ? 0: 1; // get new frame
			cout << "wait for valid frame" << endl; 
			lastFrame = true;
			continue;
		}

		else
		{
		if(processFrame(frames[state],frames[state?0:1],tmCurToNode,imCurToNode,newNode))
		{
			couldNotMatchCounter = 0;
			// cameraPos = backend.getCurrentPosition(); 

			// fuse data with IMU TODO

			// send position to UAV TODO

			// set new node
			if(newNode)
			{	
				
				cout << "Node " << ++newNodeCounter << " sent to backend" << endl; // debug	
				
				// note down the exact frame being processed. 
			
				sprintf(imgName, "depth%d",newNodeCounter);
				storage << imgName << frames[state].getDepth();

				sprintf(imgName, "gray%d",newNodeCounter);
				storage << imgName << frames[state].getGray();



				// << endl << endl; 

				// cout << frames[state].getRgb().depth() << " is bits for rgb  ";
				// cout << frames[state].getDepth().depth() << " is bits for depth " << endl;
					
				// cout << tmCurToNode << endl; 	 					
				// logger.updatePos(cameraPos);
				backend.setNewNode(frames[state],tmCurToNode,imCurToNode,1);
				frontendPos = frontendPos * tmCurToNode; // actual integration

				// sprintf(imgName, "/home/tuofeichen/Downloads/Frames/%d_new_node_rgb.png",newNodeCounter);
				// cv::imwrite(imgName,frames[state].getGray());
				// sprintf(imgName, "/home/tuofeichen/Downloads/Frames/%d_new_node_dep.png",newNodeCounter);
				// cv::imwrite(imgName,frames[state].getDepth()); // save new frame image right away

				//when image kicks in at anytime, we want edge from that point. (note this relative approach) 				 
				// currentTime = logger.getTime(); 
				// currentPos  = logger.getLpe();

#ifdef DEBUG
				logPos << frontendPos(0,3) << "," << frontendPos(1,3) << "," << frontendPos(2,3) << ",";
				logPos << currentPos(1,3)<< endl; // also note down LPE pose ? 
#else	
			    // logPos  << frames[state].getDescriptors() << endl << endl; 

		// for (int i = 0;i< (frames[state].getKeypoints3D().size());i++)
		// {
		// 	std::vector<Eigen::Vector3f> temp3D =frames[state].getKeypoints3D();
		// 	logPos << temp3D[i] << endl; 
		// }

		// logPos << "new frame" <<endl; 


			 //   	sprintf(imgName,"keypoint%d",newNodeCounter);	
				// cv::write(fileStore, imgName,frames[state].getKeypoints3D());


#endif
				state = state ? 0 : 1;
			}

		}
		else // should change with IMU
		{
			cout << "Could not match current frame!" << endl; // debug
			++totalCouldNotMatchCounter; // debug
// 			if( ++couldNotMatchCounter >= setNewNodeAfterNrOfFailedMatches) // after certain threshold, send back a dummy frame (with nearly no 
// 			{
// 				 cout << "Dummy node " << ++dummyFrameCounter << " sent to backend" << endl; // debug
// 				// dummy transformation, TODO exchange with IMU estimate

// #ifdef LPE_DUMMY				
// 				tmCurToNode = logger.getLpe() * currentPos.inverse();	// get edge from LPE
// 				// check delta movement
// 				if ((logger.getTime()!= 0) && (!VisualOdometry::checkReliability(tmCurToNode,logger.getTime() - currentTime))) 				
// 				{					
// 				imCurToNode = Matrix<float, 6, 6>::Identity() * 2000; 	
// 				backend.setNewNode(frames[state], tmCurToNode, imCurToNode,0);
// 				cout << "[LPE] Dummy Node " << endl; //debug

// 				frontendPos = frontendPos * tmCurToNode; // if directly integrate? 
// 				logPos << frontendPos(0,3) << "," << frontendPos(1,3) << "," << frontendPos(2,3) << "," ;
// 				logPos << currentPos(1,3)<< endl; 
				
// 				state = state ? 0:1; //replace old picture regardless (same to bad frame) ? 

// 				}
// #else
// 				tmCurToNode = Eigen::Matrix4f::Identity();
// 				imCurToNode = Matrix<float, 6, 6>::Identity() * 1e-100;
// 				backend.setNewNode(frames[state],tmCurToNode,imCurToNode,0);
// 				state = state ? 0:1;
// #endif
// 			}
		}
	}

		
		// TODO debug
		end = static_cast<double>( clock () ) /  CLOCKS_PER_SEC; //debug
		dif = (end - start)*1000.0;
		ros::spinOnce();
		//cout << "Frontend processing took " << dif << "ms" << endl; //debug
	}

//	AsusProLive::stop();
	AsusProLiveOpenNI2::stop();
	logPos.close();
	storage.release();  // try different file format
#endif

	testTransform();

#ifndef DEBUG_ONLY
	// debug stats
	cout << "totalNumberOfFrames = " << totalNumberOfFrames << endl; // debug
	cout << "newNodeCounter = " << newNodeCounter << endl; // debug
	cout << "dummyFrameCounter = " << dummyFrameCounter << endl; // debug
	cout << "badFramesCounter = " << badFramesCounter << endl; // debug
	cout << "totalCouldNotMatchCounter = " << totalCouldNotMatchCounter << endl; // debug

	while(backend.running()) boost::this_thread::sleep(boost::posix_time::milliseconds(100)); //TODO debug
#endif

	return 0;
}

#endif
