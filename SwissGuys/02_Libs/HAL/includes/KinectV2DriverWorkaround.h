 /**
 * @file KinectV2DriverWorkaround.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the KinectV2DriverWorkaround class
 *
 */
#ifndef KINECTV2DRIVERWORKAROUND_H_
#define KINECTV2DRIVERWORKAROUND_H_

#include <vector>

#include "opencv2/core/core.hpp"

#include "TcpClient.h"
#include "IRGBDSensor.h"

namespace HAL {

/**
 * @class KinectV2DriverWorkaround KinectV2DriverWorkaround.h "KinectV2DriverWorkaround.h"
 * @brief The KinectV2DriverWorkaround class to get kinect v2 working
 */
class KinectV2DriverWorkaround
{
public:
	/**
	 * @brief Constructor
	 */
	KinectV2DriverWorkaround();

	/**
	 * @brief Destructor
	 */
	virtual ~KinectV2DriverWorkaround();

	/**
	 * @brief register a receive callback to read frames
	 *
	 * @param callbackFun callback function (in)
	 * @param context object context (in)
	 */
	virtual bool registerCallback(void (*callbackFun)(void*, cv::Mat& rgbImage, cv::Mat& depthImage, double& timeStamp), void* context);

	/**
	 * @brief tells the driver that the read images was processed and can be deleted
	 */
	virtual void imagesProcessed();

	/**
	 * @brief start starts the RGB-D camera
	 *
	 * @return Returns true if the startup was successful and false otherwise.
	 */
	virtual void start();

	/**
	 * @brief stop stops the RGB-D camera
	 */
	virtual void stop();

	/**
	 * @brief sets the save flag. If set to true, the images will be saved on the hard disk
	 *
	 * @param flag save flag (in)
	 */
	void setSaveFlag(bool flag) { saveFlag = flag; }

	/**
	 * @brief returns the save flag. If set to true, the images will be saved on the hard disk
	 *
	 * @return true if imges will be saved and false otherwise
	 */
	bool getSaveFlag() { return saveFlag; }

	// call back for internal use
	void handleKinectStream(const  uint8_t* data, int size);

private:
	void saveRgb();
	void saveDepth();
	void saveIr();
	std::vector<int> imgPara;
	unsigned int rgbCnt;
	unsigned int depthCnt;
	unsigned int irCnt;

	enum Header {headerSize = 8, rgb=0x11, depth=0x22, ir=0x44, headerSig = 0xAA};

	enum{ listenerPort = 11000 };
	TcpClient kinect;

	int length;
	int row;
	int col;
	cv::Mat* pIm;
	volatile bool* pReadActImg;
	int* pPingPong;
	uint8_t* pImgNr;
	int rows;
	int cols;
	int headerByteCnt;
	bool isHeader;

	enum{rgbColsMax = 480, rgbRowsMax = 360, rgbColBytesMax = rgbColsMax*3};
	cv::Mat* rgbImg[2];
	double timeStamp[2];
	int rgbPingPong;
	uint8_t rgbImgNr;
	volatile int actRgbPingPong;

	enum{depthColsMax = 480, depthRowsMax = 360, depthColBytesMax = depthColsMax*2};
	cv::Mat* depthImg[2];
	int depthPingPong;
	uint8_t depthImgNr;
	volatile int actDepthPingPong;

	enum{irColsMax = 512, irRowsMax = 424, irColBytesMax = irColsMax};
	cv::Mat* irImg[2];
	int irPingPong;
	uint8_t irImgNr;
	volatile int actIrPingPong;

	void (*callback)(void*, cv::Mat& rgbImage, cv::Mat& depthImage, double& timeStamp);
	void* callbackContext;

	volatile bool readActRgbImg;
	volatile bool readActDepthImg;
	volatile bool saveFlag;
};

} /* namespace HAL */

#endif /* KINECTV2DRIVERWORKAROUND_H_ */
