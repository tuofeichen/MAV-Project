 /**
 * @file KinectV2DriverWorkaround.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the KinectV2DriverWorkaround class.
 *
 */
#include <iostream>
#include <ctime>

#include "opencv2/highgui/highgui.hpp"

#include "KinectV2DriverWorkaround.h"

namespace HAL {

//
// static callback functions
//
static void workaroundCallback(void* context, const uint8_t* data, int size)
{
	((KinectV2DriverWorkaround*)context)->handleKinectStream(data, size);
}

KinectV2DriverWorkaround::KinectV2DriverWorkaround()
 : rgbCnt(0), depthCnt(0), irCnt(0), kinect("192.168.220.1", listenerPort),
   length(0), row(0), col(0), pIm(0), pReadActImg(0), pPingPong(0), pImgNr(0),
   rows(0), cols(0), headerByteCnt(0), isHeader(false),
   rgbPingPong(0), rgbImgNr(0), actRgbPingPong(0),
   depthPingPong(0), depthImgNr(0), actDepthPingPong(0),
   irPingPong(0), irImgNr(0), actIrPingPong(0),
   callback(0), readActRgbImg(true), readActDepthImg(true), saveFlag(false)
{
	rgbImg[0] = new cv::Mat(rgbRowsMax,rgbColsMax,CV_8UC3);
	rgbImg[1] = new cv::Mat(rgbRowsMax,rgbColsMax,CV_8UC3);
	depthImg[0] = new cv::Mat(depthRowsMax,depthColsMax,CV_16UC1);
	depthImg[1] = new cv::Mat(depthRowsMax,depthColsMax,CV_16UC1);
	irImg[0] = new cv::Mat(irRowsMax,irColsMax,CV_8UC1);
	irImg[1] = new cv::Mat(irRowsMax,irColsMax,CV_8UC1);

	kinect.registerRecvCallback(0, 0);

	imgPara.push_back(CV_IMWRITE_PNG_COMPRESSION);
	imgPara.push_back(0);
}

KinectV2DriverWorkaround::~KinectV2DriverWorkaround()
{
	delete rgbImg[0];
	delete rgbImg[1];
	delete depthImg[0];
	delete depthImg[1];
	delete irImg[0];
	delete irImg[1];
}

bool KinectV2DriverWorkaround::registerCallback(void (*callbackFun)(void*, cv::Mat& rgbImage, cv::Mat& depthImage, double& timeStamp), void* context) {
	callback = callbackFun;
	callbackContext = context;
	return true;
}

void KinectV2DriverWorkaround::start() {
	kinect.registerRecvCallback(workaroundCallback, this);
	kinect.run();
}

void KinectV2DriverWorkaround::stop() {
	kinect.stop();
	kinect.registerRecvCallback(0, 0);
}

void KinectV2DriverWorkaround::imagesProcessed()
{
	actRgbPingPong = (actRgbPingPong) ? 0 : 1;
	actDepthPingPong = (actDepthPingPong) ? 0 : 1;
	readActRgbImg = readActDepthImg = true;
}

void KinectV2DriverWorkaround::handleKinectStream(const  uint8_t* dataBuf, int sizeBuf)
{
	bool allDataProcessed = false;
	while (!allDataProcessed) {
		const uint8_t* data = dataBuf;
		int size = sizeBuf;
		for(int i = 0; i < size && !isHeader; ++i)
		{
			bool headerOk = false;
			switch (headerByteCnt)
			{
			case 0:
				if(data[i] == headerSig) headerOk = true;
				break;
			case 1:
				switch (data[i])
				{
				case rgb:
					pIm = rgbImg[rgbPingPong];
					rows = rgbRowsMax;
					cols = rgbColBytesMax;
					pReadActImg = &readActRgbImg;
					pPingPong = &rgbPingPong;
					pImgNr = &rgbImgNr;
					headerOk = true;
//					std::cout << "RGB ";
					break;
				case depth:
					pIm = depthImg[depthPingPong];
					rows = depthRowsMax;
					cols = depthColBytesMax;
					pReadActImg = &readActDepthImg;
					pPingPong = &depthPingPong;
					pImgNr = &depthImgNr;
					headerOk = true;
//					std::cout << "Depth ";
					break;
				case ir:
					pIm = irImg[irPingPong];
					rows = irRowsMax;
					cols = irColBytesMax;
					//pReadActImg = &readActIrImg;
					pPingPong = &irPingPong;
					pImgNr = &irImgNr;
					headerOk = true;
//					std::cout << "IR ";
					break;
				default:
					break;
				}
				break;
			case 2:
				if(data[i] == headerSig) headerOk = true;
				break;
			case 3:
				*pImgNr = data[i];
//				std::cout << "header of image nr. " << static_cast<unsigned int>(data[i]) << " received!" << std::endl;
				headerOk = true;
				break;
			case 4:
				length = static_cast<int>(data[i]);
				headerOk = true;
				break;
			case 5:
				length += static_cast<int>(data[i]<<8);
				headerOk = true;
				break;
			case 6:
				length += static_cast<int>(data[i]<<16);
				headerOk = true;
				break;
			case 7:
				length += static_cast<int>(data[i]<<24);
				headerOk = true;
				break;
			}

			if (headerOk)
				++headerByteCnt;
			else
				headerByteCnt = 0;

			if (headerByteCnt >= headerSize)
			{
				const int idx = i+1;
				size -=  idx;
				data = &data[idx];
				isHeader = true;
				headerByteCnt = 0;
			}
		}

		if (isHeader)
		{
			for (int i = 0; i < size && i < length; ++i)
			{
				// fill image into Mat
				pIm->at<uint8_t>(row, col++) = data[i];
				if(col >= cols)
				{
					col = 0;
					if(++row >= rows)
					{
						row = 0;
						// TODO somting missing?
					}
				}
			}

			if (length > size)
			{
				length -= size;
				allDataProcessed = true;
			}
			else
			{
				if (length == size)
				{
					allDataProcessed = true;
				}
				else
				{
					size -= length;
					data = &data[length];
				}

				length = 0;
				isHeader = false;

				if (*pReadActImg)
				{
					*pPingPong = (*pPingPong) ? 0 : 1;
					*pReadActImg = false;
					if (!readActRgbImg && !readActDepthImg) // add ir if needed
					{
						if (rgbImgNr == depthImgNr)
						{
							if(saveFlag)
							{
								saveRgb();
								saveDepth();
							}

							timeStamp[actRgbPingPong] = static_cast<double>( clock () ) /  CLOCKS_PER_SEC;

							callback(callbackContext,*rgbImg[actRgbPingPong],*depthImg[actDepthPingPong], timeStamp[actRgbPingPong]);
						}
						else
						{
							std::cerr << "Error: lost an image! RGB Nr. == " << static_cast<int>(rgbImgNr)
									<< " Depth Nr == " << static_cast<int>(depthImgNr) << std::endl;
							// TODO dag: over think that!
							if(rgbImgNr > depthImgNr)
							{
								depthPingPong = actDepthPingPong;
								readActDepthImg = true;
							}
							else
							{
								rgbPingPong = actRgbPingPong;
								readActRgbImg = true;
							}
						}
					}
				}
			}
		}
		else
		{
//			if(!foundHeader) std::cout << "No image header found in part of data stream!" << std::endl;
			allDataProcessed = true;
		}
	}
}

void KinectV2DriverWorkaround::saveRgb()
{
	const std::string path = "/home/user/Downloads/out/rgb/";
	const std::string type = ".png";
	std::string imfile = path+std::to_string(timeStamp[actRgbPingPong])+type;

	if(!cv::imwrite(imfile.data(),*rgbImg[actRgbPingPong],imgPara))
		std::cerr << "Rgb image could not be saved!" << std::endl;

	++rgbCnt;
}

void KinectV2DriverWorkaround::saveDepth()
{
	const std::string path = "/home/user/Downloads/out/depth/";
	const std::string type = ".png";
	std::string imfile = path+std::to_string(timeStamp[actRgbPingPong])+type; // TODO uses the times stamp of the rgb image, change with own time stamp

	if (!cv::imwrite(imfile,*depthImg[actDepthPingPong],imgPara))
		std::cerr << "Depth image could not be saved!" << std::endl;
	++depthCnt;
}

void KinectV2DriverWorkaround::saveIr()
{
	std::ostringstream number;
	number << irCnt;
	std::string nrString = number.str();
	const std::string path = "/home/user/Downloads/out/ir/";
	const std::string type = ".png";
	std::string imfile = path+nrString+type;

	if (!cv::imwrite(imfile,*irImg[actIrPingPong],imgPara))
		std::cerr << "Ir image could not be saved!" << std::endl;
	++irCnt;
}

} /* namespace HAL */
