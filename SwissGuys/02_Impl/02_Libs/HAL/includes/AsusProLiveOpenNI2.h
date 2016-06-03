 /**
 * @file AsusProLiveOpenNI2.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the decalration of the AsusProLiveOpenNI2 class.
 *
 */

#ifndef ASUSPROLIVE_H_
#define ASUSPROLIVE_H_

#include <vector>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "OpenNI.h"

#include "Eigen/Core"

#include "IRGBDSensor.h"

namespace HAL {

/**
 * @class AsusProLiveOpenNI2 AsusProLiveOpenNI2.h "AsusProLiveOpenNI2.h"
 * @brief The AsusProLiveOpenNI2 class is the OpenNI2 driver for the Asus Xtion Pro Live
 */
class AsusProLiveOpenNI2 : public IRGBDSensor
{
public:
	/**
	 * @brief Constructor
	 */
	AsusProLiveOpenNI2();

	/**
	 * @brief Destructor
	 */
	virtual ~AsusProLiveOpenNI2();

	//
	// see IRGBDSensor.h
	virtual bool start();
	virtual void stop();
	virtual bool grab(boost::shared_ptr<cv::Mat>& rgbImage, boost::shared_ptr<cv::Mat>& grayImage, boost::shared_ptr<cv::Mat>& depthImage, boost::shared_ptr<double>& timeStamp);
	virtual bool grab(boost::shared_ptr<cv::Mat>& rgbImage, boost::shared_ptr<cv::Mat>& grayImage, boost::shared_ptr<cv::Mat>& depthImage, boost::shared_ptr<cv::Mat>& ptCloud, boost::shared_ptr<double>& timeStamp);

	/**
	 * @brief checks if the driver is still running
	 *
	 * @return true if the driver is running, false otherwise
	 */
	volatile bool running() { return !stopFlag; }

private:
	void grabber();

	//
	// QVGA
	enum{
		rows = 240,
		cols = 320,
		depthMode = 0, // select index=0 320x240, 30 fps, 1mm
		colorMode = 0, // select index 0: 320x240, 30 fps, 200 format (RGB)
	};

	//
	// VGA
//	enum{
//		rows = 480,
//		cols = 640,
//		depthMode = 4, // select index=4 640x480, 30 fps, 1mm
//		colorMode = 9, // select index 9: 640x480, 30 fps, 200 format (RGB)
//	};

	openni::Device device;
	openni::VideoStream depth;
	openni::VideoStream color;
	openni::VideoFrameRef dFrame;
	openni::VideoFrameRef cFrame;

	cv::Mat colorImg;
	cv::Mat depthImg;
	double colorTimeStamp;
	double depthTimeStamp;
	boost::mutex grabbingMutex;
	boost::thread grabberHandler;

	volatile bool stopFlag = true;
	bool modeRes = false;
	volatile bool depthReadyFlag = false;
	volatile bool colorReadyFlag = false;
};

} /* namespace HAL */

#endif /* ASUSPROLIVE_H_ */
