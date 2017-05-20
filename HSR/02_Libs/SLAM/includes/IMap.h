/**
 * @file IMap.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the IMap interface.
 *
 */

#ifndef IMAP_H_
#define IMAP_H_

#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace SLAM {

/**
 * @class IMap IMap.h "IMap.h"
 * @brief The IMap interface for a map class.
 */
class IMap {
public:
	/**
	 * @brief Destructur
	 */
	virtual ~IMap() { };

	/**
	 * @brief clears the map and all other variables
	 */
	virtual void clear() = 0;

	/**
	 * @brief adds a point cloud to the map with the specified pose.
	 *
	 * @param cloud point cloud (input)
	 * @param pose pose (input)
	 *
	 * @return the id of the point clouds which has been added to the map
	 */
	virtual int addMapPart(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const Eigen::Isometry3d& pose) = 0;

	/**
	 * @brief update the pose of a point cloud part in the map
	 *
	 * @param id id of the map part (input)
	 * @param newPose the new pose for this part (input)
	 *
	 */
	virtual void updatePose(int id, const Eigen::Isometry3d& newPose) = 0;

	/**
	 * @brief clears the map
	 */
	virtual void clearMap() = 0;

	/**
	 * @brief update trajectory map
	 *
	 * @param pose position of the trajectorie point (input)
	 *
	 */
	virtual void updateTrajectory(const Eigen::Isometry3d& pose) = 0;

	/**
	 * @brief cealrs the trajectory map
	 */
	virtual void clearTrajectory() = 0;

	/**
	 * @brief starts the map viewer
	 */
	virtual void startMapViewer() = 0;

	/**
	 * @brief stops the map viewer
	 */
	virtual void stopMapViewer() = 0;

	/**
	 * @brief update the map viewer with the new map poses
	 */
	virtual void updateMapViewer() = 0;

	/**
	 * @brief shows the map
	 * @note blacking call
	 */
	virtual void showMap() = 0;

	/**
	 * @brief saves the map to a file
	 *
	 * @param file file name
	 */
	virtual void saveMap(const std::string& file) = 0;

	/**
	 * @brief saves the trajectory to a file
	 *
	 * @param file file name
	 */
	virtual void saveTrajectory(const std::string& file) = 0;

	/**
	 * @brief returns the map as a colored point cloud
	 *
	 * @return the colored point cloud of the map
	 */
	virtual const pcl::PointCloud<pcl::PointXYZRGB>& getMapAsPointCloud() = 0;
};

} /* namespace SLAM */

#endif /* IMAP_H_ */
