/*
 * AsusProLive.h
 *
 *  Created on: May 13, 2015
 *      Author: user
 */

#ifndef ASUSPROLIVE_H_
#define ASUSPROLIVE_H_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "OpenNI.h"

#include "Eigen/Core"

#include "Frame.h"

using namespace SLAM;

class AsusProLiveOpenNI2
{
public:
	static bool start();
	static void stop();
	static bool grab(Frame& frame);

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
