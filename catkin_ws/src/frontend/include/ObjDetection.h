#include <iostream>
#include <opencv2/core.hpp>
#include <math.h>

#include "Frame.h"
#include "SURF.h"
#include "RosHandler.h"

#include "geometry_msgs/Point.h"

using namespace SLAM;

class ObjDetection {
	
	public:
	ObjDetection(IFeatures* aFDEM, RosHandler* aRos);
	ObjDetection(int detectorThreshold, int minMathces, int maxMatches);
	void processFrame(Frame newFrame);

	private:
// object detection
	void readTemplate();
	void objectDetect();

// navigation

	void checkObstacles(cv::Mat depth, int d_row, int d_col, int safe_dist);
	void checkForWall(cv::Mat Depth);
	void checkObjAngle(cv::Mat Depth);

	boost::shared_ptr <Frame> frame; // initialization 

	IFeatures* dem;  // feature detection / extraction
	RosHandler* px4; // pixhawk communication via mavros

	cv::Mat tempImage;
	boost::shared_ptr<std::vector<cv::KeyPoint>> tempKeyPoints; 	//template key points
	boost::shared_ptr<cv::Mat> 					 tempDescriptors;	//template descriptors


	geometry_msgs::Point objPoint;
	geometry_msgs::Point objAngle;
	geometry_msgs::Point wallAngle;

	geometry_msgs::Point obstacleDistance; 


	cv::Point2f past_centroid;
	cv::Point2f past_centroid2;
	cv::Point2f centroid;
	cv::Point2f centroid2;

	// boost::shared_ptr<std::vector<cv::KeyPoint>>  sceneKeyPoints; 	//template key points
	// boost::shared_ptr<cv::Mat> 					  sceneDescriptors;	//template descriptors



};