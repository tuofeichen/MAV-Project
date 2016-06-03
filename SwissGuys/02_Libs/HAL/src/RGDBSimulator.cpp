 /**
 * @file RGDBSimulator.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the RGDBSimulator class
 *
 */

//std
#include <iostream>

// boost
#include <boost/date_time/posix_time/posix_time.hpp>

// openCV
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "RGDBSimulator.h"


using namespace cv;
using namespace std;

namespace HAL {

RGDBSimulator::RGDBSimulator(std::string aSimDataPath)
: simDataPath(aSimDataPath), imgList("rgbd.txt")
{ }

RGDBSimulator::~RGDBSimulator()
{
	// close files
	list.close();
}

bool RGDBSimulator::start()
{
	// open files
	list.open((simDataPath + imgList).data());
	if (!list.is_open())
	{
		cerr << "Could not open image list file!" << endl;
		return false;
	}
	else
		return true;
}

void RGDBSimulator::stop()
{
	// close files
	list.close();
}

bool RGDBSimulator::grab(boost::shared_ptr<cv::Mat>& rgbImage, boost::shared_ptr<cv::Mat>& grayImage, boost::shared_ptr<cv::Mat>& depthImage, boost::shared_ptr<double>& timeStamp)
{
	string rgbLine;
	string dLine;
	string rgbImgName;
	string dImgName;

	string line;

	// work with content
	if (getline(list,line))
	{
		while (line.data()[0] == '#')
			getline(list,line);

		// rgb
		int pos = line.find(" ");
		timeStamp = boost::shared_ptr<double>(new double);
		*timeStamp = std::stod(line.substr(0, pos));
		line = line.substr(pos+1);
		pos = line.find(" ");
		rgbImgName = line.substr(0, pos);
		line = line.substr(pos+1);

		// depth
		pos = line.find(" ");
		line = line.substr(pos+1);
		pos = line.find(" ");
		dImgName = line.substr(0, pos);
		line = line.substr(pos+1);

		// process RGB image
		bool noError = true;
		{
			Mat tmp = imread((simDataPath + rgbImgName).data(), IMREAD_COLOR );
			if (tmp.empty())
			{
				cout << "Error: Reading RGB Image failed!" << endl;
				noError = false;
			}
			else
			{
				// rgbd
				rgbImage = boost::shared_ptr<cv::Mat>(new cv::Mat);
				tmp.copyTo(*rgbImage);

				// gray
				grayImage = boost::shared_ptr<cv::Mat>(new cv::Mat);
				(cv::Mat(tmp.size(), CV_8UC1)).copyTo(*grayImage);
				cv::cvtColor(tmp, *grayImage, CV_BGR2GRAY);
			}
		}


		// process depth image
		{
			Mat tmp =  imread((simDataPath + dImgName).data(), IMREAD_ANYDEPTH );
			if (tmp.empty())
			{
				cout << "Error: Reading RGB Image failed!" << endl;
				noError = false;
			}
			else
			{
				depthImage = boost::shared_ptr<cv::Mat>(new cv::Mat);
				tmp.copyTo(*depthImage);
			}
		}


		return noError;
	}
	else
		return false;
}

} /* namespace HAL */
