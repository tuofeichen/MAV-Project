 /**
 * @file FrameToPcConverter.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the FrameToPcConverter class
 *
 */

#ifndef FRAMTOPCCONVERTER_H_
#define FRAMTOPCCONVERTER_H_

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include "Frame.h"
#include <limits>       

namespace SLAM {

/**
 * @class FrameToPcConverter FrameToPcConverter.h "FrameToPcConverter.h"
 * @brief The FrameToPcConverter tranfroms a depth image to a point cloud
 */
class FrameToPcConverter {
public:
	/**
	 * @breif crate a point cloud out of a depth map
	 *
	 * @param frame frame to convert into a point cloud (in)
	 * @param pcOut resulting point cloud (out)
	 */
	static void getGrayPC(const Frame& frame, pcl::PointCloud<pcl::PointXYZ>& pcOut);

	/**
	 * @breif crate a colored point cloud out of a depth map
	 *
	 * @param frame frame to convert into a point cloud (in)
	 * @param pcOut resulting point cloud (out)
	 */
	static void getColorPC(const Frame& frame, pcl::PointCloud<pcl::PointXYZRGB>& pcOut);

private:
	static constexpr float cx = Frame::cx;
	static constexpr float cy = Frame::cy;
	static constexpr float fx = Frame::fx;
	static constexpr float fy = Frame::fy;
	static constexpr float depthScale = Frame::depthScale;
	static constexpr float ifx = 1.0/fx;
	static constexpr float ify = 1.0/fy;
	static constexpr float idepthScale = 1.0/depthScale;

};

} /* namespace SLAM */

#endif /* FRAMTOPCCONVERTER_H_ */
