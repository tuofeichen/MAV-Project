 /**
 * @file AsusProLiveOpenNI2.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the AsusProLiveOpenNI2 class using OpenNI2
 *
 */

#ifndef ASUSPROLIVE_H_
#define ASUSPROLIVE_H_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "OpenNI.h"

#include "Eigen/Core"

#include "Frame.h"

/**
 * @class AsusProLiveOpenNI2 AsusProLiveOpenNI2.h "AsusProLiveOpenNI2.h"
 * @brief The AsusProLiveOpenNI2 class is the OpenNI2 driver for the Asus Xtion Pro Live
 */
class AsusProLiveOpenNI2
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
	 * @brief  grab grabs images from the Asus Xtion Pro Live camera
	 *
	 * @param frame captured frame (output)
	 *
	 * @return Returns false if the grabbing procedure failed, true otherwise.
	 */
	static bool grab(Frame& frame);

	/**
	 * @brief  running checks if the Asus Xtion Pro Live camera is running
	 *
	 * @return Returns true if the Asus Xtion Pro Live camera runs, false otherwise.
	 */
	static volatile bool running() { return !stopFlag; }

private:
	static void grabber();

	static openni::Device device;
	static openni::VideoStream depth;
	static openni::VideoStream color;
	static openni::VideoFrameRef cFrame;
	static openni::VideoFrameRef dFrame;

	static cv::Mat colorImg;
	static cv::Mat depthImg;
	static double colorTimeStamp;
	static double depthTimeStamp;
	static boost::mutex grabbingMutex;
	static boost::thread grabberHandler;

	static volatile bool stopFlag;
	static volatile bool depthReadyFlag;
	static volatile bool colorReadyFlag;
};

#endif /* ASUSPROLIVE_H_ */
