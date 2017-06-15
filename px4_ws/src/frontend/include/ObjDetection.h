#include <iostream>
#include <opencv2/core.hpp>
#include <math.h>

// #include <synch.h>
#include "Frame.h"
#include "SURF.h"
#include "RosHandler.h"
#include "geometry_msgs/Point.h"

using namespace SLAM;

class ObjDetection {

	public:
		/**
		 * @brief constructor
		 */
	ObjDetection(IFeatures* aFDEM, RosHandler* aRos);
	ObjDetection(int detectorThreshold, int minMathces, int maxMatches);
	/**
	 * @brief Highlevel function call for object and obstacle detection
	 */
	void processFrame(Frame newFrame);
	/**
	 * @brief return if the object is detected (to decide if publish SLAM)
	 */
	bool getObjDetectFlag(){return objDetected;};

	private:
		/**
		 * @brief Read object template at start up
		 */
	void  readTemplate();
	/**
	 * @brief run object detection, return if object is detected
	 */
	bool  objectDetect();
	/**
	 * @brief check obstacles from depth map within safe_dist
	 */
	void checkObstacles(cv::Mat depth, int d_row, int d_col, int safe_dist);
	/**
	 * @brief check wall angle
	 */
	void checkForWall(cv::Mat Depth);
	/**
	 * @brief check object angle (can be combined with checkForWall)
	 */
	void checkObjAngle(cv::Mat Depth);


	Frame objFrame; // initialization

	IFeatures* dem;  // feature detection / extraction
	RosHandler* px4; // pixhawk communication via mavros

	cv::Mat tempImage;
	boost::shared_ptr<std::vector<cv::KeyPoint>> tempKeyPoints; 	//template key points
	boost::shared_ptr<cv::Mat> 					 				 tempDescriptors;	//template descriptors


	geometry_msgs::Point objPoint;
	geometry_msgs::Point objAngle;
	geometry_msgs::Point wallAngle;
	geometry_msgs::Point obstacleDistance;
	bool objDetected;

	cv::Point2f prevObjCentroid;
	cv::Point2f objCentroid;
};
