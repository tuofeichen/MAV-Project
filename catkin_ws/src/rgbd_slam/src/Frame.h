/*
 * Frame.h
 *
 *  Created on: May 6, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#ifndef INCLUDES_FRAME_H_
#define INCLUDES_FRAME_H_

#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"

#include "Eigen/Core"

class Frame
{
public:
	static constexpr float depthScale = 1000.0;

	//
	// QVGA
	static constexpr float fx = 285.0;
	static constexpr float fy = 285.0;
	static constexpr float cx = 159.5;
	static constexpr float cy = 119.5;
	enum{
		rows = 240,
		cols = 320,
		depthMode = 0, // select index=0 320x240, 30 fps, 1mm
		colorMode = 0, // select index 0: 320x240, 30 fps, 200 format (RGB)
	};

	//
	// VGA
//	static constexpr float fx = 570.0;
//	static constexpr float fy = 570.0;
//	static constexpr float cx = 319.5;
//	static constexpr float cy = 239.5;
//	enum{
//		rows = 480,
//		cols = 640,
//		depthMode = 4, // select index=4 640x480, 30 fps, 1mm
//		colorMode = 9, // select index 9: 640x480, 30 fps, 200 format (RGB)
//	};
	
	static constexpr float ransacMaxDistInitialTest = 0.03f; // 3 cm
	static constexpr float ransacMaxDistInlier = 0.02f; // 2 cm
	static constexpr float ransacTermInlierPct = 0.7f; // 60%

	static constexpr float minTranslation = 0.10; // 10 cm
	static constexpr float minRotation = 5.0/180.0*M_PI; // 5°
	static constexpr float maxTranslationPerSecond = 10.0; // 10.0 m/s
	static constexpr float maxAngularVelocit = 180.0/180.0*M_PI; // 180°/s

	enum{
		maxNrOfKeyPoints = 500,
		ransacMaxIter = 1000,
		ransacMinInlier = 10,
		minNrOfMatches = 17,
		minNrOfKeyPoints = 30,
	};

	Frame() : time(0), rgb(rows,cols,CV_8UC3), gray(rows, cols, CV_8UC1), depth(rows, cols, CV_16UC1), keypoints(maxNrOfKeyPoints), keypoints3D(maxNrOfKeyPoints) { }
	~Frame() { }

	double& setTime() { return time; }
	cv::Mat& setRgb() { return rgb; }
	cv::Mat& setGray() { return gray; }
	cv::Mat& setDepth() { return depth; }

	const double& getTime() const { return time; }
	const cv::Mat& getRgb() const { return rgb; }
	const cv::Mat& getGray() const { return gray; }
	const cv::Mat& getDepth() const { return depth; }

	void setKeypoints(std::vector<cv::KeyPoint>& keys);
	const std::vector<cv::KeyPoint>& getKeypoints() const { return keypoints; }
	const std::vector<Eigen::Vector3f>& getKeypoints3D() const { return keypoints3D; }

	cv::Mat& setDescriptors() { return descriptors; }
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
