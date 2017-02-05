 /**
 * @file PointCloudMap.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the PointCloudMap class
 *
 */
#ifndef POINTCLOUDMAP_H_
#define POINTCLOUDMAP_H_

#include "pcl/visualization/cloud_viewer.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"

#include "IMap.h"

namespace SLAM {

/**
 * @class PointCloudMap PointCloudMap.h "PointCloudMap.h"
 * @brief The PointCloudMap class build the point cloud map
 */
class PointCloudMap : public IMap {
public:
	/**
	 * @brief Constructor
	 *
	 * @param voxelGridSize voxel grid size in meter (in)
	 */
	PointCloudMap(float voxelGridSize);

	/**
	 * @brief Destructor
	 */
	virtual ~PointCloudMap();

	//
	// see IMap.h for description
	virtual void clear();
	virtual int addMapPart(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const Eigen::Isometry3d& pose);
	virtual void updatePose(int id, const Eigen::Isometry3d& pose);
	virtual void clearMap();
	virtual void updateTrajectory(const Eigen::Isometry3d& pose);
	virtual void clearTrajectory();
	virtual void startMapViewer();
	virtual void stopMapViewer();
	virtual void updateMapViewer();
	virtual void showMap();
	virtual void saveMap(const std::string& file);
	virtual void saveTrajectory(const std::string& file) { pcl::io::savePCDFile (file, *trajectory); }
	virtual const pcl::PointCloud<pcl::PointXYZRGB>& getMapAsPointCloud() { updateMap(); return *map; }

private:
	void updateMap();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr map;
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;

	pcl::visualization::PCLVisualizer::Ptr viewer;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr trajectory;

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>> keyFramePointClouds;
	std::vector<Eigen::Isometry3d> keyFrameMapPoses;
	struct PointCloudInMapInfo {PointCloudInMapInfo(unsigned int p, unsigned int s, bool u) : pos(p), size(s),  uptodate(u){}; unsigned int pos; unsigned int size; bool uptodate;};
	std::vector<PointCloudInMapInfo> pointCloudLocationInMap;

	volatile bool mapPosUpdated = false;

};

} /* namespace SLAM */

#endif /* POINTCLOUDMAP_H_ */
