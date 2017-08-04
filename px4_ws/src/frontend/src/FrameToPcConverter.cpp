/**
 * @file FrameToPcConverter.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the FrameToPcConverter class.
 *
 */
#include "FrameToPcConverter.h"

namespace SLAM {

void FrameToPcConverter::getGrayPC(const Frame& frame, pcl::PointCloud<pcl::PointXYZ>& pcOut)
{
	const cv::Mat& depthIm = frame.getDepth();

	float tmpX, tmpY, tmpZ;
	const int width = depthIm.cols;
	const int hight = depthIm.rows;

	pcOut.clear();
	for (int row = 0; row < hight; ++row) {

		for (int col = 0; col < width; ++col) {

				uint16_t depth = depthIm.at<uint16_t>(row * width + col);
				if(depth != 0) {
					tmpZ = static_cast<float>(depth) * idepthScale;
					tmpX = tmpZ * (static_cast<float>(col) - cx) * ifx;
					tmpY = tmpZ * (static_cast<float>(row) - cy) * ify;
					pcl::PointXYZ tmpPoint(tmpX, tmpY, tmpZ);
					pcOut.push_back(tmpPoint);
				}
		}
	}
}

void FrameToPcConverter::getColorPC(const Frame& frame, pcl::PointCloud<pcl::PointXYZRGB>& pcOut)
{
	const cv::Mat& depthIm = frame.getDepth();
	const cv::Mat& rgbIm = frame.getRgb();

	float tmpX, tmpY, tmpZ;
	const int width = depthIm.cols;
	const int hight = depthIm.rows;

	pcOut.clear();
	pcOut.width    = 320; 
    pcOut.height   = 240;
    pcOut.is_dense = false;
    pcOut.points.resize(pcOut.width * pcOut.height);

	for (int row = 0; row < hight; ++row) {

		for (int col = 0; col < width; ++col) {

				uint16_t depth = depthIm.at<uint16_t>(row * width + col);
				const int rgbCol = 3*col;
				if(depth != 0) {
					tmpZ = static_cast<float>(depth)*idepthScale;
					tmpX = tmpZ * (static_cast<float>(col) - cx) * ifx;
					tmpY = tmpZ * (static_cast<float>(row) - cy) * ify;

					pcl::PointXYZRGB tmpPoint(rgbIm.at<uint8_t>(row, rgbCol+2),rgbIm.at<uint8_t>(row, rgbCol+1),rgbIm.at<uint8_t>(row, rgbCol));
					tmpPoint.x = tmpX;
					tmpPoint.y = tmpY;
					tmpPoint.z = tmpZ;
					pcOut.points[row*pcOut.width + col] = tmpPoint;
				}
				else {
					pcl::PointXYZRGB tmpPoint(rgbIm.at<uint8_t>(row, rgbCol+2),rgbIm.at<uint8_t>(row, rgbCol+1),rgbIm.at<uint8_t>(row, rgbCol));
					tmpPoint.x = std::numeric_limits<float>::quiet_NaN();
					tmpPoint.y = std::numeric_limits<float>::quiet_NaN();
					tmpPoint.z = std::numeric_limits<float>::quiet_NaN();
					pcOut.points[row*pcOut.width + col] = tmpPoint;
				}
		}
	}
}

} /* namespace SLAM */
