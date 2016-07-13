/*
 * AsusProLive.cpp
 *
 *  Created on: May 13, 2015
 *      Author: user
 */

#include <iostream>

#include "opencv2/imgproc.hpp"

#include "AsusProLiveOpenNI2.h"

using namespace openni;

Device AsusProLiveOpenNI2::device;
VideoStream AsusProLiveOpenNI2::depth;
VideoStream AsusProLiveOpenNI2::color;
VideoFrameRef AsusProLiveOpenNI2::cFrame;
VideoFrameRef AsusProLiveOpenNI2::dFrame;

cv::Mat AsusProLiveOpenNI2::colorImg(Frame::rows,Frame::cols,CV_8UC3);
cv::Mat AsusProLiveOpenNI2::depthImg(Frame::rows,Frame::cols,CV_16UC1);
double AsusProLiveOpenNI2::colorTimeStamp;
double AsusProLiveOpenNI2::depthTimeStamp;
boost::mutex AsusProLiveOpenNI2::grabbingMutex;
boost::thread AsusProLiveOpenNI2::grabberHandler;

volatile bool AsusProLiveOpenNI2::stopFlag = true;
volatile bool AsusProLiveOpenNI2::depthReadyFlag = false;
volatile bool AsusProLiveOpenNI2::colorReadyFlag = false;


bool AsusProLiveOpenNI2::start()
{
	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return false;
	}

	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK)
	{
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return false;
	}

	if (device.getSensorInfo(SENSOR_COLOR) != NULL && device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		rc = depth.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return false;
		}

		rc = color.create(device, SENSOR_COLOR);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
			return false;
		}

		rc = device.setDepthColorSyncEnabled(true);
		if (rc != STATUS_OK)
		{
			printf("Couldn't set depth sync\n%s\n", OpenNI::getExtendedError());
			return false;
		}

		rc = device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		if (rc != STATUS_OK)
		{
			printf("Couldn't set registration\n%s\n", OpenNI::getExtendedError());
			return false;
		}

		rc = depth.setMirroringEnabled(false);
		if (rc != STATUS_OK)
		{
			printf("Couldn't set mirror enable of the depth stream\n%s\n", OpenNI::getExtendedError());
			return false;
		}

		rc = color.setMirroringEnabled(false);
		if (rc != STATUS_OK)
		{
			printf("Couldn't set color enable of the depth stream\n%s\n", OpenNI::getExtendedError());
			return false;
		}

		const SensorInfo* sinfo = device.getSensorInfo(SENSOR_DEPTH);
		const Array<VideoMode>& modesDepth = sinfo->getSupportedVideoModes();
		for (int i = 0; i<modesDepth.getSize(); i++) {
			printf("%i: %ix%i, %i fps, %i format\n", i, modesDepth[i].getResolutionX(), modesDepth[i].getResolutionY(),
				modesDepth[i].getFps(), modesDepth[i].getPixelFormat()); //PIXEL_FORMAT_DEPTH_1_MM = 100, PIXEL_FORMAT_DEPTH_100_UM = 101
		}

		rc = depth.setVideoMode(modesDepth[Frame::depthMode]);
		if (rc != STATUS_OK)
		{
			printf("Couldn't set video mode of the depth stream\n%s\n", OpenNI::getExtendedError());
			return false;
		}

		const SensorInfo* scinfo = device.getSensorInfo(SENSOR_COLOR);
		const Array<VideoMode>& modesColor = scinfo->getSupportedVideoModes();
		for (int i = 0; i<modesColor.getSize(); i++) {
			printf("%i: %ix%i, %i fps, %i format\n", i, modesColor[i].getResolutionX(), modesColor[i].getResolutionY(),
					modesColor[i].getFps(), modesColor[i].getPixelFormat());
		}

		rc = color.setVideoMode(modesColor[Frame::colorMode]);
		if (rc != STATUS_OK)
		{
			printf("Couldn't set video mode of the color stream\n%s\n", OpenNI::getExtendedError());
			return false;
		}

		rc = depth.start();
		if (rc != STATUS_OK)
		{
			printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
			return false;
		}

		rc = color.start();
		if (rc != STATUS_OK)
		{
			printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
			return false;
		}
	}
	else
	{
		printf("No depth and/or color sensor found!\n");
		return false;
	}

	stopFlag = false;
	grabberHandler = boost::thread(&AsusProLiveOpenNI2::grabber);

	return true;
}

void AsusProLiveOpenNI2::grabber()
{
	while(!stopFlag)
	{
		Status rc = depth.readFrame(&dFrame);
       	if (rc != STATUS_OK)
        {
               	printf("Failed to read depth frame\n%s\n", OpenNI::getExtendedError());
               	break;;
        }

        rc = color.readFrame(&cFrame);
        if (rc != STATUS_OK)
        {
        	printf("Failed to read color frame\n%s\n", OpenNI::getExtendedError());
            break;
        }

        {
			boost::mutex::scoped_lock(grabbingMutex);
			
			DepthPixel* pDepth = (DepthPixel*)dFrame.getData();
            memcpy( depthImg.data, pDepth, Frame::cols*Frame::rows*sizeof(uint16_t));
           	depthTimeStamp = static_cast<double>(dFrame.getTimestamp()) * 1e-6;
            depthReadyFlag = true;
			
        	RGB888Pixel* pColor = (RGB888Pixel*)cFrame.getData();
            memcpy( colorImg.data, pColor, 3*Frame::rows*Frame::cols*sizeof(uint8_t));
            colorTimeStamp = static_cast<double>(cFrame.getTimestamp()) * 1e-6;
            colorReadyFlag = true;
        }

		boost::this_thread::sleep(boost::posix_time::milliseconds(60));
	}
	stopFlag = true;
}

void AsusProLiveOpenNI2::stop()
{
	stopFlag = true;
	grabberHandler.join();
	depth.stop();
	color.stop();
	depth.destroy();
	color.destroy();
	device.close();
	OpenNI::shutdown();
}

bool AsusProLiveOpenNI2::grab(Frame& frame)
{
	if(colorReadyFlag && depthReadyFlag)
	{
		boost::mutex::scoped_lock(grabbingMutex);
		// grab setRgb, setTime, setGray, set Depth
		
		cv::cvtColor(colorImg,frame.setRgb(),CV_BGR2RGB);
		depthImg.copyTo(frame.setDepth());
		frame.setTime() = depthTimeStamp; // colorTimeStamp
		cv::cvtColor(colorImg,frame.setGray(),CV_BGR2GRAY);
		colorReadyFlag = false;
		depthReadyFlag = false;
		return true;
	}
	else
		return false;
}
