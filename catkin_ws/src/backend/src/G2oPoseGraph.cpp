/**
 * @file G2oPoseGraph.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the G2oPoseGraph class.
 *
 */
#include "g2o/core/hyper_dijkstra.h"

#include "G2oPoseGraph.h"

namespace SLAM {

G2oPoseGraph::G2oPoseGraph()
 : blockSolver(&linearSolver), solver(&blockSolver), currentIndex(0)
{
	linearSolver.setBlockOrdering(false);
	optimizer.setAlgorithm(&solver);
	optimizer.setVerbose( false );
}

G2oPoseGraph::~G2oPoseGraph()
{
	optimizer.clear(); // freeing the graph memory
}

int G2oPoseGraph::addFirstNode()
{
	boost::mutex::scoped_lock(graphMutex);

	optimizer.clear(); // freeing the graph memory
	currentIndex = 0;

	g2o::VertexSE3* vertex = new g2o::VertexSE3();
	vertex->setId(currentIndex);
	vertex->setToOriginImpl();
	vertex->setFixed(true);

	optimizer.addVertex(vertex);
	return currentIndex;
}

int G2oPoseGraph::addNode(const Eigen::Isometry3d& position)
{
	boost::mutex::scoped_lock(graphMutex);

	// set node
	g2o::VertexSE3* vertex = new g2o::VertexSE3();
	vertex->setId(++currentIndex);
	vertex->setEstimate(position);
	optimizer.addVertex(vertex);

//	std::cout << "Added node nr. " << currentIndex << std::endl;
	return currentIndex;
}

void G2oPoseGraph::removeNode(int id)
{
	boost::mutex::scoped_lock(graphMutex);
	assert(id >= 0 && id <= currentIndex);
	g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(id));
	optimizer.removeVertex(v);
}

void G2oPoseGraph::addEdgePreviousToCurrent(const Eigen::Isometry3d& measurement, const Eigen::Matrix<double, 6, 6>& informationMatrix) {
	addEdgeFromIdToId(measurement, informationMatrix, currentIndex-1, currentIndex);
}

void G2oPoseGraph::addEdgeFromIdToCurrent(const Eigen::Isometry3d& measurement, const Eigen::Matrix<double, 6, 6>& informationMatrix, int idFrom) {
	addEdgeFromIdToId(measurement, informationMatrix, idFrom, currentIndex);
}

void G2oPoseGraph::addEdgeFromCurrentToId(const Eigen::Isometry3d& measurement, const Eigen::Matrix<double, 6, 6>& informationMatrix, int idTo) {
	addEdgeFromIdToId(measurement, informationMatrix, currentIndex, idTo);
}

void G2oPoseGraph::addEdgeFromIdToId(const Eigen::Isometry3d& measurement, const Eigen::Matrix<double, 6, 6>& informationMatrix, int idFrom, int idTo)
{
	boost::mutex::scoped_lock(graphMutex);
	assert(idFrom <= currentIndex && idFrom >= 0);
	assert(idTo <= currentIndex && idTo >= 0);
	assert(idFrom != idTo);

	// set edge
	g2o::EdgeSE3* edge = new g2o::EdgeSE3();
	edge->vertices()[0] = optimizer.vertex(idFrom);
	edge->vertices()[1] = optimizer.vertex(idTo);
	edge->setInformation( informationMatrix );
	edge->setMeasurement(measurement);
	optimizer.addEdge(edge);
}

bool G2oPoseGraph::removeEdgesWithErrorBiggerThen(double threshold)
{
	boost::mutex::scoped_lock(graphMutex);
	bool ret = false;
	int cnt = 0;
	optimizer.computeActiveErrors();
	for(g2o::HyperGraph::EdgeSet::iterator i = optimizer.edges().begin(); i != optimizer.edges().end(); ++i)
	{
		g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(*i);
		if( e->chi2() > threshold)
		{
			// replace edge
			Eigen::Isometry3d tm = Eigen::Isometry3d::Identity();
			Eigen::Matrix<double,6,6> im = Eigen::Matrix<double,6,6>::Identity()*1e-100;
			e->setMeasurement(tm);
			e->setInformation(im);
			ret = true;
			++cnt;
		}
	}
	std::cout << "Replaced "<< cnt << " bad edge!" << std::endl;
	return ret;
}

void G2oPoseGraph::optimize()
{
	boost::mutex::scoped_lock(graphMutex);

	optimizer.vertex(0)->setFixed(true);
	if(optimizer.initializeOptimization())
	{
		// optimize max iterations
		optimizer.optimize(optMaxIter);
	}
	else
		std::cerr << "Initialization of the optimizer failed!" << std::endl;
}

void G2oPoseGraph::optimizeTillConvergenz()
{
	boost::mutex::scoped_lock(graphMutex);
//	optimizer.save("graph.g2o");

	optimizer.vertex(0)->setFixed(true);
	if(optimizer.initializeOptimization())
	{
		// optimize till convergence
		double chi2 = std::numeric_limits<double>::max();
		double prevChi2;
		do
		{
			prevChi2 = chi2;
			optimizer.optimize(10);
			optimizer.computeActiveErrors();
			chi2 = optimizer.chi2();
		}
		while(chi2/prevChi2 < optConvergence);
//		optimizer.save("graphOpt.g2o");
	}
	else
		std::cerr << "Initialization of the optimizer failed!" << std::endl;
}

const Eigen::Isometry3d& G2oPoseGraph::getPositionOfId(int id)
{
	boost::mutex::scoped_lock(graphMutex);
	assert(id >= 0 && id <= currentIndex);
	return ((g2o::VertexSE3*) optimizer.vertex(id))->estimate();
}

void G2oPoseGraph::getEdgeNeighborsToCurrentNode(int sequential, std::vector<int>& neighborIds)
{
	boost::mutex::scoped_lock(graphMutex);
	neighborIds.clear();

	g2o::HyperDijkstra dijkstra(&optimizer);
	g2o::UniformCostFunction costFun;
	g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(currentIndex));
	dijkstra.shortestPaths(v, &costFun, 3); // geodesic distance == 3

	// shortest path search
	
	g2o::HyperGraph::VertexSet& vs = dijkstra.visited();

	for (g2o::HyperGraph::VertexSet::iterator i=vs.begin(); i!=vs.end(); ++i)
	{
		int vid = (*i)->id();
		if (vid < (currentIndex-sequential) && vid >= 0 && (currentIndex-sequential) > 0) // not sequential!
		{
			neighborIds.push_back(vid);
		}
	}
}

} /* namespace SLAM */
