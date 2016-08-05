/**
 * @file PointCloudMap.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the PointCloudMap class.
 *
 */

#include "pcl/common/transforms.h"

#include "PointCloudMap.h"

namespace SLAM {

//PointCloudMap::PointCloudMap(float voxelGridSize) : map(new pcl::PointCloud<pcl::PointXYZRGB>), viewer(0), trajectory(new pcl::PointCloud<pcl::PointXYZRGB>)
PointCloudMap::PointCloudMap(float voxelGridSize) : map(new pcl::PointCloud<pcl::PointXYZRGB>), trajectory(new pcl::PointCloud<pcl::PointXYZRGB>)

{
	clearMap();
	clearTrajectory();
	clear();
	sor.setLeafSize (voxelGridSize, voxelGridSize, voxelGridSize);
}

PointCloudMap::~PointCloudMap()
{ }

int PointCloudMap::addMapPart(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const Eigen::Isometry3d& pose)
{
	pcl::PointCloud<pcl::PointXYZRGB> dummy;
	keyFramePointClouds.push_back(dummy);
	keyFrameMapPoses.push_back(pose);

	sor.setInputCloud (cloud.makeShared());
	sor.filter(keyFramePointClouds.back());

	pointCloudLocationInMap.push_back(PointCloudInMapInfo(map->size(), keyFramePointClouds.back().size(), false));

	for(pcl::PointCloud<pcl::PointXYZRGB>::const_iterator i = keyFramePointClouds.back().begin(); i != keyFramePointClouds.back().end(); ++i)
	{
		Eigen::Vector3f tmp = pose.matrix().topLeftCorner<3,4>().cast<float>() * i->getVector4fMap();
		pcl::PointXYZRGB p(*i);
		p.x = tmp.x();
		p.y = tmp.y();
		p.z = tmp.z();
		map->push_back(p);
	}

	pointCloudLocationInMap.back().uptodate = true;

	return keyFramePointClouds.size()-1;
}

void PointCloudMap::updatePose(int id, const Eigen::Isometry3d& pose)
{
	assert(pointCloudLocationInMap.at(id).size == keyFramePointClouds.at(id).size());
	if(!pose.isApprox(keyFrameMapPoses.at(id), 0.001))
	{
		pointCloudLocationInMap.at(id).uptodate = false;
		mapPosUpdated = true;
	}
	keyFrameMapPoses.at(id) = pose;
}

void PointCloudMap::updateTrajectory(const Eigen::Isometry3d& pose)
{
	pcl::PointXYZRGB p(255,0,0);
	p.x = static_cast<float>(pose.translation().x());
	p.y = static_cast<float>(pose.translation().y());
	p.z = static_cast<float>(pose.translation().z());
	trajectory->push_back(p);
}

void PointCloudMap::updateMap()
{
	if(mapPosUpdated)
	{
		mapPosUpdated = false;
		for(int id = 0; id < pointCloudLocationInMap.size(); ++id)
		{
			if(!pointCloudLocationInMap.at(id).uptodate)
			{
				int pos = pointCloudLocationInMap.at(id).pos;
				for(pcl::PointCloud<pcl::PointXYZRGB>::const_iterator i = keyFramePointClouds.at(id).begin(); i != keyFramePointClouds.at(id).end(); ++i)
				{
					Eigen::Vector3f tmp = keyFrameMapPoses.at(id).matrix().topLeftCorner<3,4>().cast<float>() * i->getVector4fMap();
					pcl::PointXYZRGB p(*i);
					p.x = tmp.x();
					p.y = tmp.y();
					p.z = tmp.z();
					map->at(pos++) = p;
				}
				pointCloudLocationInMap.at(id).uptodate = true;
			}
		}
	}
}

void PointCloudMap::startMapViewer()
{
	viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Point Cloud Map"));
	viewer->addPointCloud (map, "cloud");
	viewer->addPointCloud (map, "trajectory");
	viewer->setCameraPosition (0, 2, -6, 0, -1, -0.5, 0);
//	viewer->setCameraFieldOfView(,0);
	viewer->setSize (2*640, 2*480);  // Visualiser window size
//	viewer->addCoordinateSystem (0.1, "axis", 0);
	viewer->setBackgroundColor (255, 255, 255, 0);	// Setting background to a white
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewer->spinOnce();
}

void PointCloudMap::stopMapViewer()
{
	if(viewer) viewer->close();
}

void PointCloudMap::updateMapViewer()
{
	if(viewer)
	{
		updateMap();
		viewer->updatePointCloud (trajectory, "trajectory");
		viewer->updatePointCloud(map, "cloud");
		viewer->spinOnce (0,true);
	}
}

void PointCloudMap::showMap()
{
	if(viewer)
	{
		updateMap();
		while (!viewer->wasStopped ())
		{
			viewer->spinOnce ();
		}
	}
}

void PointCloudMap::clearMap()
{
	map->clear();
	map->is_dense = true;
}

void PointCloudMap::clearTrajectory()
{
	trajectory->clear();
	trajectory->is_dense = true;
}

void PointCloudMap::clear()
{
	keyFramePointClouds.clear();
	keyFrameMapPoses.clear();
	pointCloudLocationInMap.clear();
}

void PointCloudMap::saveMap(const std::string& file)
{
	updateMap();

	pcl::PointCloud<pcl::PointXYZRGB> dummy;
	sor.setInputCloud (map);
	sor.filter(dummy);

	pcl::io::savePCDFile (file, dummy);
}

} /* namespace SLAM */
