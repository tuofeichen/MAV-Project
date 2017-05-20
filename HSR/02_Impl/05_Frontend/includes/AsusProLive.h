 /**
 * @file AsusProLive.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the AsusProLive class using OpenNI and OpenCV
 *
 */

#ifndef ASUSPROLIVE_H_
#define ASUSPROLIVE_H_

#include "opencv2/videoio.hpp"

/**
 * @class AsusProLive AsusProLive.h "AsusProLive.h"
 * @brief The AsusProLive class is the OpenNI driver for the Asus Xtion Pro Live
 */
class AsusProLive
{
public:

	/**
	 * @brief start starts the Asus Xtion Pro Live camera
	 *
	 * @return Returns true if the startup was successful and false otherwise.
	 */
	static bool start();
	
	/**
	 * @brief stop stops the Asus Xtion Pro Live camera
	 */
	static void stop();
	
	/**
	 * @brief grab grabs images from the Asus Xtion Pro Live camera
	 *
	 * @param rgbImage RGB image (output)
	 * @param grayImage gray image (output)
	 * @param depthImage depth image (output)
	 * @param timeStamp time when the images were taken (output)
	 *
	 * @return Returns false if the grabbing procedure failed, true otherwise.
	 */
	static bool grab(cv::Mat& rgbImage, cv::Mat& grayImage, cv::Mat& depthImage, double& timeStamp);

private:
	AsusProLive() { } // no objects
	static cv::VideoCapture asusCapture;
};

#endif /* ASUSPROLIVE_H_ */
