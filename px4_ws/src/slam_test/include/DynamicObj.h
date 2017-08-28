 /**
 * @file DynamicObj.h
 * @author Eric Dexheimer
 * @brief This file contains the declaration of the DynamicObj class
 *
 */

#ifndef INCLUDES_DYNAMICOBJ_H_
#define INCLUDES_DYNAMICOBJ_H_

#include "Frame.h"
#include "FrameToPcConverter.h"
#include "Eigen/Eigen"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace SLAM {

class DynamicObj 
{

public:
	DynamicObj();
	~DynamicObj();

	static constexpr double dynamicDepthThreshold = 0.2; //0.01;//0.2;//0.005;	// depth difference in meters needed to be considered dynamic 
	static constexpr int intensityThreshold = 1000; //500

	void getObjCandidates(Eigen::Matrix4f tm, 
							Frame& currFrame, 
							Frame& prevFrame
							); 

	void clusterPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
							pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud);

	void dynamicObjectRemoval(Eigen::Matrix4f tm, 
							Frame currFrame, 
							Frame prevFrame);

	cv::Mat getFilteredDepth() {return outputDepth;}

private:
	cv::Mat currCandidates;
	cv::Mat prevCandidates;
	cv::Mat outputDepth;

};

}
#endif /* INCLUDES_DYNAMICOBJ_H_ */