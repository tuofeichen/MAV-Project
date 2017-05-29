/**
 * @file IPoseGraph.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the IPoseGraph interface.
 *
 */

#ifndef IPOSEGRAPH_H_
#define IPOSEGRAPH_H_

#include "Eigen/Geometry"

namespace SLAM {

/**
 * @class IPoseGraph IPoseGraph.h "IPoseGraph.h"
 * @brief The IPoseGraph interface for a pose graph class.
 */
class IPoseGraph {
public:
	/**
	 * @brief Destructur
	 */
	virtual ~IPoseGraph() { };

	/**
	 * @brief returns the node id of the current node
	 *
	 * @return current node id
	 */
	virtual int getCurrentId() = 0;

	/**
	 * @brief adds the first node (given an initial position)
	 *
	 * @return returns the id of the first node
	 */
	virtual int addFirstNode(const Eigen::Isometry3d& position) = 0;

	/**
	 * @brief adds a node to the pose graph
	 *
	 * @param pose pose (input)
	 *
	 * @return returns the id of the added node
	 */
	virtual int addNode(const Eigen::Isometry3d& pos) = 0;

	/**
	 * @brief removes the node with the specified id
	 *
	 * @param id node id (inpute)
	 */
	virtual void removeNode(int id) = 0;

	/**
	 * @brief adds an edge between the previous and the current node
	 *
	 * @param measurement measurement (inpute)
	 * @param informationMatrix informationMatrix (inpute)
	 *
	 */
	virtual void addEdgePreviousToCurrent(const Eigen::Isometry3d& measurement, const Eigen::Matrix<double, 6, 6>& informationMatrix) = 0;

	/**
	 * @brief adds an edge between the id and the current node
	 *
	 * @param measurement measurement (inpute)
	 * @param informationMatrix informationMatrix (inpute)
	 * @param idFrom source node id (inpute)
	 *
	 */
	virtual void addEdgeFromIdToCurrent(const Eigen::Isometry3d& measurement, const Eigen::Matrix<double, 6, 6>& informationMatrix, int idFrom) = 0;

	/**
	 * @brief adds an edge between the current node and the id
	 *
	 * @param measurement measurement (inpute)
	 * @param informationMatrix informationMatrix (inpute)
	 * @param idTo target node id (inpute)
	 *
	 */
	virtual void addEdgeFromCurrentToId(const Eigen::Isometry3d& measurement, const Eigen::Matrix<double, 6, 6>& informationMatrix, int idTo) = 0;

	/**
	 * @brief adds an edge between the source node and targed node
	 *
	 * @param measurement measurement (inpute)
	 * @param informationMatrix informationMatrix (inpute)
	 * @param idFrom source node id (inpute)
	 * @param idTo target node id (inpute)
	 *
	 */
	virtual void addEdgeFromIdToId(const Eigen::Isometry3d& measurement, const Eigen::Matrix<double, 6, 6>& informationMatrix, int idFrom, int idTo) = 0;

	/**
	 * @brief removes edges with a bigger error than the threshold
	 *
	 * @param threshold threshold (inpute)
	 *
	 * @return true if a edge was removed, otherwise false
	 */
	virtual bool removeEdgesWithErrorBiggerThen(double threshold) = 0;

	/**
	 * @brief optimizes the graph
	 */
	virtual void optimize() = 0;

	/**
	 * @brief optimizes the graph till convergence
	 */
	virtual void optimizeTillConvergenz() = 0;

	/**
	 * @brief returns the pose of the node with id
	 *
	 * @return pose
	 */
	virtual const Eigen::Isometry3d& getPositionOfId(int id) = 0;

	/**
	 * @brief returns the pose of the current node
	 *
	 * @return pose
	 */
	virtual const Eigen::Isometry3d& getCurrentPosition() = 0;

	/**
	 * @brief returns the geodesic graph neighbors of the current node, by excluding the sequential nodes
	 *
	 * @param sequential number of sequential nodes to elude (input)
	 * @param id's of neighbor nodes (output)
	 *
	 */
	virtual void getEdgeNeighborsToCurrentNode(int sequential, std::vector<int>& neighborIds) = 0;
};

} /* namespace SLAM */

#endif /* IPOSEGRAPH_H_ */
