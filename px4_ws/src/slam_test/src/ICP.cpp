/**
 * @file ICP.cpp
 * @author Eric Dexheimer
 * @brief This file contains the implementation of the ICP class.
 *
 */
#include "ICP.h"

namespace SLAM
{

ICP::ICP()
{
	/*transformedCloud = new pcl::PointCloud<pcl::PointXYZRGB> ();
	filteredCloud = new pcl::PointCloud<pcl::PointXYZRGBNormal> ();
	prevCloud = new pcl::PointCloud<pcl::PointXYZRGBNormal> ();
	downsampledCloud = new pcl::PointCloud<pcl::PointXYZRGBNormal> ();*/

	/*cumulative_transform = 	(Eigen::Matrix4f() <<  -1,0,0,0,
                                                   0,1,0,0,
                                                   0,0,-1,0,
                                                   0,0,0,1).finished();*/
	finalTransformation = Eigen::Matrix4f::Identity();
	initialTransformation = Eigen::Matrix4f::Identity();
	converged = false;

}

ICP::~ICP() {};

void ICP::filteringAndProcessing(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr currCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	// get current cloud
    pcl::copyPointCloud(*cloud, *currCloud);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    // downsample point cloud from 640x320 to 320x240
    downsampledCloud->width    = 160; //320;
    downsampledCloud->height   = 120; //240;
    downsampledCloud->is_dense = false;
    downsampledCloud->points.resize (downsampledCloud->width * downsampledCloud->height);
    for(int i = 0; i < currCloud->height; i+=4) {
      for(int j = 0; j < currCloud->width; j+=4) {
        downsampledCloud->points[i/4*downsampledCloud->width + j/4] = currCloud->points[i*currCloud->width + j];
      }
    }

    // bilateral filter for organized point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*downsampledCloud,*points);
    pcl::FastBilateralFilter<pcl::PointXYZRGB> bilateral_filter;
    bilateral_filter.setInputCloud(points);
    bilateral_filter.setSigmaS(5);	// 5
    bilateral_filter.setSigmaR(0.005f);	// 0.35f
    bilateral_filter.applyFilter(*points);
    pcl::copyPointCloud(*points, *downsampledCloud);
    //duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    //std::cout<<"bilateral filter time: "<< duration <<'\n';

    // estimate normals for organized point cloud 
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(downsampledCloud);
    ne.compute(*downsampledCloud);

    // remove NaN (must be done after estimating normals)
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*downsampledCloud,*downsampledCloud, indices);
    pcl::removeNaNNormalsFromPointCloud(*downsampledCloud,*downsampledCloud,indices);

    //sampling
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
    // random sampling
    /*pcl::RandomSample<pcl::PointXYZRGBNormal> random_sample;
    random_sample.setInputCloud(downsampledCloud);      //the size of it 307200
    random_sample.setSample( 5000 ); //5000
    random_sample.setSeed(rand());
    random_sample.filter(*filteredCloud); */

    // normal space sampling
    pcl::NormalSpaceSampling<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> normal_space_sampling;
    normal_space_sampling.setInputCloud(downsampledCloud);
    normal_space_sampling.setNormals (downsampledCloud);
    normal_space_sampling.setBins(10, 10, 10); //10
    normal_space_sampling.setSeed(rand());
    normal_space_sampling.setSample(5000); //1000
    normal_space_sampling.filter(*filteredCloud);
    
    pcl::copyPointCloud(*filteredCloud, *cloud);
}

void ICP::preprocessing(Frame& frame, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB> pc;
	FrameToPcConverter::getColorPC(frame,pc);
	pcl::copyPointCloud(pc,*cloud);
	ICP::filteringAndProcessing(cloud);
}

bool ICP::run(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc1,pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc2)
{
	std::clock_t start;
    double duration;

	pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;

	//correspondence estimation
	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr ce (new pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>);
	ce->setInputSource(pc2);
	ce->setInputTarget(pc1);
	ce->setSourceNormals(pc2);
	icp.setCorrespondenceEstimation(ce);

	//registration
	pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr trans_lls (new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>);      
	icp.setTransformationEstimation(trans_lls);

	// rejection pipeline
	// normal rejection
	pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rejector_normal  (new pcl::registration::CorrespondenceRejectorSurfaceNormal);
	rejector_normal->initializeDataContainer<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal>();
	rejector_normal->setInputSource<pcl::PointXYZRGBNormal>(pc2);
	rejector_normal->setInputTarget<pcl::PointXYZRGBNormal>(pc1);
	rejector_normal->setInputNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(pc2);
	rejector_normal->setTargetNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(pc1); 
	rejector_normal->setThreshold (cos(3*M_PI/180)); // 10 degrees converted to radians, threshold is cosine of angle
	icp.addCorrespondenceRejector(rejector_normal);
	// distance rejection
	pcl::registration::CorrespondenceRejectorMedianDistance::Ptr rejector_median_distance (new pcl::registration::CorrespondenceRejectorMedianDistance);
	rejector_median_distance->setMedianFactor(2);
	icp.addCorrespondenceRejector(rejector_median_distance);
	// boundary points rejection
	//pcl::registrationCorrespondenceRejectionOrganizedBoundary::Ptr rejector_organized_boundary (new pcl::registrationCorrespondenceRejectionOrganizedBoundary);
	//rejector_organized_boundary


	// ICP
	icp.setInputSource(pc2);
	icp.setInputTarget(pc1);
	// filter correspondence distances
	icp.setMaxCorrespondenceDistance(0.2);  // test out values of this
	// termination criteria
	icp.setMaximumIterations(20); // 20
	icp.setTransformationEpsilon(1e-5);
	icp.setEuclideanFitnessEpsilon(1); 
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Final(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	start = std::clock();
	icp.align(*Final,initialTransformation);
	duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"alignment takes: "<< duration <<'\n';

	if (icp.hasConverged() && (icp.getFitnessScore() < 0.005))
		converged = true;
	/*std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;*/
	//cumulative_transform *= icp.getFinalTransformation();
	finalTransformation = icp.getFinalTransformation();

	// Executing the transformation
	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::copyPointCloud(*downsampledCloud, *original_cloud);
	pcl::transformPointCloud(*original_cloud, *transformed_cloud, cumulative_transform);*/

	// NOTE: This assigment causes both the source and target point clouds to be randomly sampled, could lead to inconsistency
	//*prevCloud = *downsampledCloud;
	return converged;

}

void ICP::setTransformationGuess(Eigen::Matrix4f guess) {
	initialTransformation = guess;
}

Eigen::Matrix4f ICP::getFinalTransformation() {
	return finalTransformation;
}


}