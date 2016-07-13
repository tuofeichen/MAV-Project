// test file for 2 frames transformation estimate 
// alright!! ready to knock this down!


#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "VisualOdometry.h"

using namespace std;
using namespace cv; 

static std::ofstream logPosDebug, logTmDebug;

static bool processFrameDebug(const Frame& newFrame, const Frame& prevNode, Matrix4f& tmCurToNode, Matrix<float, 6, 6>& imCurToNode, bool& newNode)
{


	//
	// estimate transforamtion

	bool validFlag;
	std::vector<int>  srcMatches;
	srcMatches = VisualOdometry::estimateTrafo(newFrame, prevNode, tmCurToNode, imCurToNode, validFlag);
	
	// logPosDebug << endl << "new frame" <<endl; 
	// for (int i = 0; i< srcMatches.size(); i++)	
	// 	logPosDebug << srcMatches[i] << ",";

	// logPosDebug << endl; 

	return validFlag;

	VisualOdometry::Result res;
	
	if (validFlag){
		res = VisualOdometry::checkReliability(tmCurToNode, newFrame.getTime() - prevNode.getTime());
		validFlag = (res != VisualOdometry::invalid); 
	}

	if (validFlag){
		newNode = (res == VisualOdometry::valid);
		return true;
	}

	else {
		newNode = false;
		return false;
	}
}



void testTransform()
{
	static Frame frame1,frame2; // new frame
	const float scaleFactor = 0.05f; // for visualization

	static Matrix4f tmCurToNode, frontendPos;
	static Matrix<float, 6, 6> imCurToNode;

	bool newNode = 1;
    bool validFlag = 0;
    //char file_name [100];
    char imgName[100];

    cv::FileStorage fileRead("/home/tuofeichen/.ros/test.yml", cv::FileStorage::READ);
    cv::FileStorage fileStore("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/rgbd_slam/Frames/keypoints_sim.txt",cv::FileStorage::WRITE);


    frontendPos.setIdentity();

	logPosDebug.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/rgbd_slam/Frames/frontend_sim.csv",std::ofstream::out | std::ofstream::trunc);
    logPosDebug << "X,Y,Z"<<endl;

    logTmDebug.open("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/rgbd_slam/Frames/trans_sim.txt",std::ofstream::out | std::ofstream::trunc);
    logTmDebug << "Transofrmation estimate " << endl;

    for (int i = 1; i<1000;i++){ 

 //    sprintf(file_name,"/home/tuofeichen/Downloads/Frames/%d_new_node_rgb.png", i);
	// Mat img1 = imread(file_name, CV_LOAD_IMAGE_GRAYSCALE);
	// sprintf(file_name,"/home/tuofeichen/Downloads/Frames/%d_new_node_dep.png", i);
	// Mat map1 = imread(file_name, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

 //    sprintf(file_name,"/home/tuofeichen/Downloads/Frames/%d_new_node_rgb.png", i+1);
	// Mat img2 = imread(file_name, CV_LOAD_IMAGE_GRAYSCALE);
	// sprintf(file_name,"/home/tuofeichen/Downloads/Frames/%d_new_node_dep.png", i+1);
	// Mat map2 = imread(file_name, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);



	Mat dep1, dep2, img1, img2, map1, map2; // actual depth map to be processed  
	// dep1 = map1;
	// dep2 = map2; 

	// should be exactly the same image being processed

	cout << "read from storage" << endl; 
	sprintf(imgName, "depth%d",i);
	fileRead[imgName] >> dep1;
	sprintf(imgName, "depth%d",i+1);
	fileRead[imgName] >> dep2;

	sprintf(imgName, "gray%d",i);
	fileRead[imgName] >> img1;
	sprintf(imgName, "gray%d",i+1);
	fileRead[imgName] >> img2;

	if (dep2.depth() == 0){
		cout << "finished processing" << endl;
		waitKey(0);
		break;
	}

	map1 = dep1; // visual
	map2 = dep2; 
	// cout << "finished reading? !" << endl; 
	
	cout << dep1.size() << endl; 

	map1.convertTo(map1,CV_8UC1,scaleFactor); // debug visual
	map2.convertTo(map2,CV_8UC1,scaleFactor);

	// img1 = imdecode(img1,CV_LOAD_IMAGE_GRAYSCALE);
	namedWindow("img1",CV_WINDOW_AUTOSIZE);
	imshow("img1",img1);
	waitKey(30);
	
	namedWindow("img2",CV_WINDOW_AUTOSIZE);
	imshow("img2",img2);
	waitKey(30);

	dep1.copyTo(frame1.setDepth());
	img1.copyTo(frame1.setGray());
	dep2.copyTo(frame2.setDepth());
	img2.copyTo(frame2.setGray());

// setkey point in both frames
	if(VisualOdometry::setKeypoints(frame1))
		cout << i <<  " is good frame!" <<endl; 
	else 
		cout << i << " is bad frame!" << endl;

	VisualOdometry::setKeypoints(frame2);

	// 	cout << i+1 << " is good frame!"<<endl;
	// else
	// 	cout << i+1 << " is bad frame!" << endl;

	validFlag = processFrameDebug(frame2, frame1, tmCurToNode, imCurToNode, newNode);

	frontendPos = frontendPos * tmCurToNode; 
	// logPosDebug << img1 << endl << endl;

	logPosDebug << frontendPos(0,3) << "," << frontendPos(1,3) << "," << frontendPos(2,3) << endl;
	

	// for (int i = 0;i< (frame1.getKeypoints3D().size());i++)
	// {
	// 	std::vector<Eigen::Vector3f> temp3D = frame1.getKeypoints3D();
	// 	logPosDebug << temp3D[i] << endl; 
	// }
	// logPosDebug << endl; 

 //    sprintf(file_name,"keypoint%d", i);	
	// cv::write(fileStore, file_name,frame1.getKeypoints3D());



	logTmDebug << "transformation number " << i << endl; 
	logTmDebug <<  tmCurToNode << endl; 


	// cout << "resulting transform is " << endl << tmCurToNode << endl; 
	// cout << "information matrix is " << endl << imCurToNode << endl; 
	cout << "valid flag is " << validFlag << endl << endl ; 
	}

	fileStore.release(); 
}
