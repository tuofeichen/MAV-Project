 /**
 * @file Frame.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the Frame class
 *
 */
 
#ifndef INCLUDES_FRAME_H_
#define INCLUDES_FRAME_H_

#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"

#include "Eigen/Core"

/**
 * @class Frame Frame.h "Frame.h"
 * @brief The Frame stores all important informations about a rgb-d frame
 */
class Frame
{
public:
	static constexpr float depthScale = 1000.0; ///< Depth scale factor for the asus xtion pro live sensor

	//
	// QVGA
	static constexpr float fx = 285.0; ///< focal length in x direction in pixel
	static constexpr float fy = 285.0; ///< focal length in y direction in pixel
	static constexpr float cx = 159.5; ///< center point in x direction in pixel
	static constexpr float cy = 119.5; ///< center point in y direction in pixel
	enum{
		rows = 240, ///< rows of the images
		cols = 320, ///< columns of the images
		depthMode = 0, ///< select index=0 320x240, 30 fps, 1mm
		colorMode = 0, ///< select index 0: 320x240, 30 fps, 200 format (RGB)
	};

	//
	// VGA
//	static constexpr float fx = 570.0; ///< focal length in x direction in pixel
//	static constexpr float fy = 570.0; ///< focal length in y direction in pixel
//	static constexpr float cx = 319.5; ///< center point in x direction in pixel
//	static constexpr float cy = 239.5; ///< center point in y direction in pixel
//	enum{
//		rows = 480, ///< rows of the images
//		cols = 640, ///< columns of the images
//		depthMode = 4, ///< select index=4 640x480, 30 fps, 1mm
//		colorMode = 9, ///< select index 9: 640x480, 30 fps, 200 format (RGB)
//	};


	static constexpr float ransacThresholdAbsolutDistTest = 0.03f; ///< threshold for the absolut distance test in meter(RANSAC)
	static constexpr float ransacMaxDistInlier = 0.02f; ///< max distance for inlier in meter(RANSAC)
	static constexpr float ransacTermInlierPct = 0.6f; ///< percentage of inlier to terminate (RANSAC)

	static constexpr float minTranslation = 0.10; ///< minimal translation in meter
	static constexpr float minRotation = 5.0/180.0*M_PI; ///< minimal rotation in rad
	static constexpr float maxTranslationPerSecond = 10.0; ///< maximal velocity in meter/sec
	static constexpr float maxAngularVelocit = 180.0/180.0*M_PI; ///< maximal angular velocity in rad/sec

	enum{
		maxNrOfKeyPoints = 600, ///< maximal number of features to return
		ransacMaxIter = 1000, ///< maximal number of iteration for the RANSAC algorithm
		ransacMinInlier = 10, ///< minimal number of inlier for the RANSAC algorithm
		minNrOfMatches = 17, ///< minimal number of matches
		minNrOfKeyPoints = 30, ///< minimal number of valid feature points
	};

	/**
	 * @brief Constructor
	 */
	Frame() : time(0), rgb(rows,cols,CV_8UC3), gray(rows, cols, CV_8UC1), depth(rows, cols, CV_16UC1), keypoints(maxNrOfKeyPoints), keypoints3D(maxNrOfKeyPoints) { }
	
	/**
	 * @breif Destructor
	 */
	~Frame() { }

	/**
	 * @breif setTime sets time
	 *
	 * @return returns time
	 */
	double& setTime() { return time; }
	
	/**
	 * @breif setRgb sets RGB image
	 *
	 * @return returns RGB image
	 */
	cv::Mat& setRgb() { return rgb; }
	
	/**
	 * @breif setGray sets gray image
	 *
	 * @return returns gray image
	 */
	cv::Mat& setGray() { return gray; }
	
	/**
	 * @breif setDepth sets depth image
	 *
	 * @return returns depth image
	 */
	cv::Mat& setDepth() { return depth; }
	
	
	/**
	 * @breif getTime gets time stamp
	 *
	 * @return returns time time stamp
	 */
	const double& getTime() const { return time; }
	
	/**
	 * @breif getRgb gets RGB image
	 *
	 * @return returns RGB image
	 */
	const cv::Mat& getRgb() const { return rgb; }
	
	/**
	 * @breif getGray gets gray image
	 *
	 * @return returns gray image
	 */
	const cv::Mat& getGray() const { return gray; }
	
	/**
	 * @breif getDepth gets depth image
	 *
	 * @return returns depth image
	 */
	const cv::Mat& getDepth() const { return depth; }

	
	/**
	 * @breif setKeypoints sets keypoints of the image
	 *
	 * @param keys found feature points in the image (input)
	 */
	void setKeypoints(std::vector<cv::KeyPoint>& keys);
	
	/**
	 * @breif getKeypoints gets keypoints of the image
	 *
	 * @return returns key points
	 */
	const std::vector<cv::KeyPoint>& getKeypoints() const { return keypoints; }
	
	/**
	 * @breif getKeypoints3D gets 3D keypoints of the image
	 *
	 * @return returns 3D key points
	 */
	const std::vector<Eigen::Vector3f>& getKeypoints3D() const { return keypoints3D; }

	
	/**
	 * @breif setDescriptors sets feature descriptors of the image
	 *
	 * @param keys found feature descriptors of the image (input)
	 */
	cv::Mat& setDescriptors() { return descriptors; }
	
	/**
	 * @breif getDescriptors gets feature descriptors of the image
	 *
	 * @return returns feature descriptor
	 */
	const cv::Mat& getDescriptors() const { return descriptors; }

private:
	double time;
	cv::Mat rgb;
	cv::Mat gray;
	cv::Mat depth;
	std::vector<cv::KeyPoint> keypoints;
	std::vector<Eigen::Vector3f> keypoints3D;
	cv::Mat descriptors;

	static constexpr float ifx = 1.0/fx;
	static constexpr float ify = 1.0/fy;
	static constexpr float idepthScale = 1.0/depthScale;
};

#endif /* INCLUDES_FRAME_H_ */
