 /**
 * @file ICP.h
 * @author Eric Dexheimer
 * @brief This file contains the declaration of the ICP class
 *
 */

#ifndef INCLUDES_ICP_H_
#define INCLUDES_ICP_H_

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
//#include <pcl/filters/random_sample.h>
#include <pcl/filters/normal_space.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>

namespace SLAM {

class ICP 
{

public:
	ICP();
	~ICP();
	static void filteringAndProcessing(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
	bool run(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc1,pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc2);
	void setTransformationGuess(Eigen::Matrix4f guess);
	Eigen::Matrix4f getFinalTransformation();

private:
	bool converged;

	Eigen::Matrix4f finalTransformation;
	Eigen::Matrix4f initialTransformation;	


};

}
#endif /* INCLUDES_ICP_H_ */