/**
 * @file Mapping.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the Mapping class.
 *
 */

#include <iostream>
#include <assert.h>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "Mapping.h"

using namespace std;

namespace SLAM {

Mapping::Mapping(
		IFeatures* aFDEM,
		ITransformMatEst* aTME,
		IPoseGraph* aGO)
		//IMap* aMap
		
 : fdem(aFDEM), tme(aTME), poseGraph(aGO) //, map3d(aMap)
{
	// check element are not pointing to 0
	assert(fdem);
	assert(tme);
	assert(poseGraph);
	// assert(map3d);

	currentPosition = Eigen::Isometry3d::Identity();
	keyFrames.clear();
	nodes.clear();
}

Mapping::~Mapping()
{ }

void Mapping::run()
{
//	std::cout << "Mapping::start called!" << std::endl;
	double totalTime = 0;
	double relTime;

	// initialize variables
	bestInforamtionValue = 0;
	smallestId = lcSmallestId = poseGraph->getCurrentId();
	frames = 0;

	time.tic();
	if (!featureDetectionAndExtraction())
	{
		cout << "bad frame!"<< endl; 
		++badFrameCounter;
		return;
	}

	cout << "finished feature extraction" << endl; 

	relTime = time.toc();
	totalTime += relTime;
//	cout << "Feature detection and extraction took " << relTime << "ms" << endl;
	
	if(!initDone)
	{
		addFirstNode();
		// updateMap();
		return;
	}

	// match current frame with older frames

	time.tic();
	parallelMatching();
	relTime = time.toc();
	totalTime += relTime;
	cout << "Parallel matching took " << relTime << "ms" << endl;

	
	// process graph
	time.tic();
	for(int thread = 0; thread < frames && !currentFrame.getDummyFrameFlag(); ++thread)
	{
		// first add node then add edge
		if (currentFrame.getId() < 0)
			tryToAddNode(thread);
		else
			addEdges(thread);
	}

	cout << "try to add node or edge" << endl; 

	if(searchLoopClosures && nodes.size() > neighborsToMatch + contFramesToMatch && lcRandomMatching == 0)
	{
		
		lcHandler.join();

		if(lcBestIndex > 0 && currentFrame.getId() >= 0)
		{
			cout << lcBestIndex << " is lc best index" << endl; 
			if(lcValidTrafo && lcEnoughMatches)
			{
				GraphProcessingResult res = processGraph(lcTm, lcIm, keyFrames.at(lcBestIndex).getId(), (currentFrame.getTime()-keyFrames.at(lcBestIndex).getTime()), false, true);
				if (res == trafoValid)
					smallestId = std::min(smallestId, keyFrames.at(lcBestIndex).getId());
			}
		}
	}


	// cout << "process graph" << endl; 
	relTime = time.toc();
	totalTime += relTime;
//	cout << "Graph processing took " << relTime << "ms" << endl;

	if(loopClosureFound)
	{
		++detLoopClsrsCounter;
		std::cout << "<----------------------------------------------------------------------- loop detected!" << std::endl;
	}

	//
	// print
	cout << "SLAM of frame nr " << frameCounter << " was ";
	if(currentFrame.getId() < 0)
	{
		if(currentFrame.getDummyFrameFlag())
			cout << "dropped (transformation to small or to big) ";
		else
			cout << "dropped (added as dummy) ";
	}
	else
		cout << "added ";

	//
	// key frame, map and optimization
	time.tic();
	if(currentFrame.getId() < 0)
	{ // invalid frame
		if(exchangeFirstNode && nodes.size() == 1)
		{
			exchangeFirstFrame();
		}
		else
		{
			++noTrafoFoundCounter;
			// setting a dummy node is needed, otherwise the track of frames can be lost
			if(addDummyNodeFlag)
			{
				++sequenceOfLostFramesCntr;
				setDummyNode();
			}
		}
	}
	else if(currentFrame.getDummyFrameFlag())
	{ // trafo to small or to big
		sequenceOfLostFramesCntr = 0;
		if(exchangeFirstNode && nodes.size() == 1)
		{
			exchangeFirstFrame();
		}
	}
	else
	{
		sequenceOfLostFramesCntr = 0;

		// search key frame
		bool addedKeyFrame = searchKeyFrames();

		//
		// optimize graph once
		optimizeGraph(false);
//		boost::thread(&Mapping::optimizeGraph,this,false);

		if(addedKeyFrame)
		{
			if(loopClosureFound)
			{
				if(optimizeTillConvergence)
				{
					//
					// optimize graph till convergenz
					optimizeGraph(true);
//					boost::thread(&Mapping::optimizeGraph,this,true);
				}
				loopClosureFound = false;
			}

			//
			// update map
			// updateMap();
		}
	}

	relTime = time.toc();
	totalTime += relTime;
//	cout << "Optimization and map update took " << relTime << "ms" << endl;

	cout << " and took " << totalTime << "ms"<< endl;

	++nframeProc;
	frameProcMeanTime += totalTime;
	if(totalTime > frameProcMaxTime)
		frameProcMaxTime = totalTime;
}

void Mapping::addFrame(Frame& frame)
{
	++frameCounter;

	if (!initDone)
	{
		if (!nodes.empty())
			initDone = true;
	}

	currentFrame = frame;
}

void Mapping::matchTwoFrames(
		const Frame& frame1, // in: new frame
		const Frame& frame2, // in: older frames
		bool& enoughMatches, // out
		bool& validTrafo,// out
		Eigen::Isometry3d& transformationMatrix, // out: transformation from frame 1 to frame 2
		Eigen::Matrix<double, 6, 6>& informationMatrix  // out: information matrix of the estimation
		)
{
	assert(frame1.getId() != frame2.getId());

	//
	// feature detecting, extracting and matching
	//
	std::vector<int> matchesIdx1;
	std::vector<int> matchesIdx2;
	enoughMatches = fdem->match(frame1.getKeypoints(), frame1.getDescriptors(), frame2.getKeypoints(), frame2.getDescriptors(), matchesIdx1, matchesIdx2);

	if (enoughMatches)
	{
		//
		// estimate transformation matrix
		//
		Eigen::Matrix4f tm;
		validTrafo = tme->estimateTrafo(frame1.getKeypoints3D(), matchesIdx1, frame2.getKeypoints3D(), matchesIdx2, tm, informationMatrix);
		transformationMatrix.matrix() = tm.cast<double>();
	}
	else
		validTrafo = false;
}

Mapping::GraphProcessingResult Mapping::processGraph(const Eigen::Isometry3d& transformationMatrix, const Eigen::Matrix<double, 6, 6>& informationMatrix, int prevId, double deltaTime, bool tryToAddNode, bool possibleLoopClosure)
{
	assert(prevId >= 0);
	assert(!(tryToAddNode && possibleLoopClosure));
	cout << "processing graph" << endl; 
	if(isVelocitySmallEnough(transformationMatrix, deltaTime)) 
	{
		// try to add current node
		if (tryToAddNode)
		{
			if(isMovementBigEnough(transformationMatrix))
			{
				currentPosition =  (poseGraph->getPositionOfId(prevId))*transformationMatrix;
				poseGraph->addNode(currentPosition);
				currentFrame.setId(poseGraph->getCurrentId());
				nodes.push_back(currentFrame);
			}
			else
			{ // transformation too  small
				currentPosition =  poseGraph->getPositionOfId(prevId)*transformationMatrix;
				return trafoToSmall;
			}
		}
		else if(possibleLoopClosure){
			// cout << "found  loop closure!" <<  endl; 
			loopClosureFound = true;
		}

		// add edge to current node
		poseGraph->addEdgeFromIdToCurrent(transformationMatrix, informationMatrix, prevId);
		return trafoValid;
	}
	else
	{
		return trafoToBig;
	}
}

bool Mapping::featureDetectionAndExtraction()
{
	//
	// run feature detection and extraction on new frame
	//
	currentFrame.setKeypoints( fdem->detect(currentFrame.getGray()) );

	// extract descriptors
	currentFrame.setDescriptors( fdem->extract(currentFrame.getGray(), currentFrame.getKeypoints()) );

	if (currentFrame.getKeypoints().size() < minNumberOfKeyPoints)
		return false;
	else
		return true;
}

void Mapping::exchangeFirstFrame()
{
		if(currentFrame.getKeypoints().size() > nodes.back().getKeypoints().size())
		{
			addFirstNode();
			// map3d->clear();
			// map3d->clearMap();
			// map3d->clearTrajectory();
			currentPosition = poseGraph->getCurrentPosition();

			// updateMap();
		}
}

void Mapping::parallelMatching()
{
	std::vector<int> neighborIds;
	const int contFramesToMatchTmp = (nodes.size() >= (neighborsToMatch + contFramesToMatch)) ? contFramesToMatch : neighborsToMatch + contFramesToMatch;

	//
	// match continuous
	if(contFramesToMatch > 0 && nodes.size() > 0)
	{
		const int nrOfContFramesToMatch = std::min<int>(nodes.size(), contFramesToMatchTmp);
		std::vector<Frame>::const_iterator pNode = nodes.end() - 1;

		for (int frame = 0; frame < nrOfContFramesToMatch; ++frame, ++frames, --pNode)
		{
			graphIds[frames] = pNode->getId();
			lcSmallestId = std::min(lcSmallestId, graphIds[frames]);

			// match frames
			deltaT[frames] = currentFrame.getTime() - pNode->getTime();
			handler[frames] = boost::thread(&Mapping::matchTwoFrames, this,
					boost::ref(currentFrame),
					boost::ref(*pNode),
					boost::ref(enoughMatches[frames]),
					boost::ref(validTrafo[frames]),
					boost::ref(transformationMatrices[frames]),
					boost::ref(informationMatrices[frames]) );
		}
	}

	if ( nodes.size() >= (neighborsToMatch + contFramesToMatch) && nodes.size() > 0 )
	{
		//
		// find neighbors of current node, exclude continuous nodes
		poseGraph->getEdgeNeighborsToCurrentNode(contFramesToMatch, neighborIds);

		//
		// match graph neighbors (sequential nodes are excluded)
		if (neighborsToMatch > 0 && neighborIds.size() > 0)
		{
			int tmpPrev = -1;
			const int tmpNeighborsToMatch = std::min<int>(neighborIds.size(), neighborsToMatch);
			for(int i = 0; i < tmpNeighborsToMatch; ++i, ++frames)
			{
				int tmp;
				do { tmp = rand() % neighborIds.size(); } while( tmpPrev == tmp ); //TODO how to choose randomly? with weights?
				graphIds[frames] = neighborIds.at(tmp);
				lcSmallestId = std::min(lcSmallestId, graphIds[frames]);
				tmpPrev = tmp;

				const int nodesId = nodes.size() - 1 - (poseGraph->getCurrentId() - graphIds[frames]);

				// match frames
				deltaT[frames] = currentFrame.getTime() - nodes.at(nodesId).getTime();
				handler[frames] = boost::thread(&Mapping::matchTwoFrames, this,
						boost::ref(currentFrame),
						boost::ref(nodes.at(nodesId)),
						boost::ref(enoughMatches[frames]),
						boost::ref(validTrafo[frames]),
						boost::ref(transformationMatrices[frames]),
						boost::ref(informationMatrices[frames]) );
			}
		}
	}

	//
	// loop closures
	if(searchLoopClosures && nodes.size() > neighborsToMatch + contFramesToMatch && keyFrames.size() > lcRandomMatching)
	{
		if(lcRandomMatching == 0)
		{
			lcHandler = boost::thread(&Mapping::loopClosureDetection,this);
		}
		else
		{
			int prevId = -1;
			for(int i = 0; i < lcRandomMatching; ++i,  ++frames)
			{
				// find non neighbor and sequencial nodes
				int id;
				bool isNeighbor;
				do {
					id = rand() % keyFrames.size();

					const int keyFrameId = keyFrames.at(id).getId();

					// no sequencials
					if(keyFrameId >= nodes.at(nodes.size()-contFramesToMatch).getId())
						continue;

					isNeighbor = false; // no neighbors
					for(int k=0; k < static_cast<int>(neighborIds.size()); ++k) // TODO faster implementation or better strategy
					{
						if(neighborIds.at(k) == keyFrameId)
						{
							isNeighbor = true;
							break;
						}
					}
				} while(isNeighbor); //TODO check that not the same key frames are matched

				// matching
				graphIds[frames] = keyFrames.at(id).getId();
				deltaT[frames] = currentFrame.getTime() - keyFrames.at(id).getTime();
				handler[frames] = boost::thread(&Mapping::matchTwoFrames, this,
						boost::ref(currentFrame),
						boost::ref(keyFrames.at(id)),
						boost::ref(enoughMatches[frames]),
						boost::ref(validTrafo[frames]),
						boost::ref(transformationMatrices[frames]),
						boost::ref(informationMatrices[frames]) );
				prevId = id;
			}
		}
	}

	// join threads
	for (int thread = 0; thread < frames; ++thread)
	{
		handler[thread].join(); // sleep until all threads are finished with their work
	}
}

void Mapping::tryToAddNode(int thread)
{
	if (validTrafo[thread] && enoughMatches[thread])
	{
		GraphProcessingResult result = processGraph(transformationMatrices[thread], informationMatrices[thread], graphIds[thread], deltaT[thread], true, false);
		if(result == trafoValid)
		{
			smallestId = std::min(smallestId, graphIds[thread]);

			bestInforamtionValue = informationMatrices[thread](0,0);
			currentPosition = poseGraph->getPositionOfId(graphIds[thread])*transformationMatrices[thread];
		}
		else if(result == trafoToSmall)
		{
			currentFrame.setDummyFrameFlag(true);
			++trafoToSmallCounter; // debug
		}
		else if (result == trafoToBig)
		{
			currentFrame.setDummyFrameFlag(true);
			++trafoVelocityToBigCounter; // debug
		}
	}
}

void Mapping::addEdges(int thread)
{
	if (validTrafo[thread] && enoughMatches[thread])
	{
		GraphProcessingResult result;
		if(lcRandomMatching == 0)
			result = processGraph(transformationMatrices[thread], informationMatrices[thread], graphIds[thread], deltaT[thread], false, false);
		else
			result = processGraph(transformationMatrices[thread], informationMatrices[thread], graphIds[thread], deltaT[thread], false, (thread >= contFramesToMatch + neighborsToMatch));

		if(result == trafoValid)
		{
			smallestId = std::min(smallestId, graphIds[thread]);

			// update current position if the estimate is better
			if(bestInforamtionValue < informationMatrices[thread](0,0))
			{
				bestInforamtionValue = informationMatrices[thread](0,0);
				currentPosition = poseGraph->getPositionOfId(graphIds[thread])*transformationMatrices[thread];
			}
		}
	}
}

void Mapping::setDummyNode()
{
	if(sequenceOfLostFramesCntr > dummyFrameAfterLostFrames)
	{
		if(!nodes.back().getDummyFrameFlag())
		{
			// add node
			const Eigen::Isometry3d dummyTm = Eigen::Isometry3d::Identity();
			const Eigen::Matrix<double, 6, 6> dummyInfoMat = Eigen::Matrix<double, 6, 6>::Identity() * 1e-100;
			poseGraph->addNode(currentPosition);
			poseGraph->addEdgeFromIdToCurrent(dummyTm, dummyInfoMat, nodes.back().getId());
			currentFrame.setId(poseGraph->getCurrentId());
			currentFrame.setDummyFrameFlag(true);
			nodes.push_back(currentFrame);

			++dummyNodeCounter; //debug
		}
		else if (currentFrame.getKeypoints().size() > nodes.back().getKeypoints().size())
		{
			currentFrame.setDummyFrameFlag(true);
			currentFrame.setId(nodes.back().getId());
			nodes.back() = currentFrame;
		}
	}
}

bool Mapping::searchKeyFrames()
{
	bool ret = false;
	if (smallestId > keyFrames.back().getId())
	{
		for(int thread = 0; thread < frames; ++thread)
		{
			if (validTrafo[thread] &&  isVelocitySmallEnough(transformationMatrices[thread], deltaT[thread]))
			{
					Frame& keyFrame = nodes.back();
					keyFrame.setKeyFrameFlag(true);
					keyFrames.push_back(keyFrame);
					cout << "Added id " << keyFrame.getId() << " as key frame <================================================================================" << endl;

					ret = true;
					break;
			}
		}
	}

	// delete images
	nodes.back().deleteRgb();
	nodes.back().deleteDepth();
	nodes.back().deleteGray();

	return ret;
}

void Mapping::optimizeGraph(bool tillConvergenz)
{
	if (onlineOptimization)
	{
		optTime.tic();
		if(tillConvergenz)
		{
			if(removeEdgesWithBigErrors)
				poseGraph->removeEdgesWithErrorBiggerThen(edgeErrorThreshold);
			poseGraph->optimizeTillConvergenz();
		}
		else
			poseGraph->optimize();

		double took = optTime.toc();
		++nOpt;
		optAvrTime += took;
		if(took > optMaxTime)
			optMaxTime = took;

		currentPosition = poseGraph->getCurrentPosition();
	}
}

void Mapping::updateMap()
{
	// pcl::PointCloud<pcl::PointXYZRGB> cloud;
	// FrameToPcConverter::getColorPC(keyFrames.back(), cloud);
	// map3d->addMapPart(cloud, poseGraph->getPositionOfId(keyFrames.back().getId()));

	// // TODO when is a good time to update poses?
	// for(int i = 0; i < keyFrames.size(); ++i)
	// 	map3d->updatePose(i,poseGraph->getPositionOfId(keyFrames.at(i).getId()));

	// delete images out of the stored frames
	keyFrames.back().deleteRgb();
	keyFrames.back().deleteDepth();
	keyFrames.back().deleteGray();
}

void Mapping::convertRotMatToEulerAngles(const Eigen::Matrix3d& t, double& roll, double& pitch, double& yaw) const
{
	roll = atan2(t(2,1),t(2,2)); // roll (around x)
	pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2))); // pitch (around y)
	yaw = atan2(t(1,0),t(0,0)); // yaw (around z)
}

bool Mapping::isMovementBigEnough(const Eigen::Isometry3d& trafo) const
{
	double roll, pitch, yaw;
	convertRotMatToEulerAngles(trafo.rotation(),roll,pitch,yaw);
	double maxAngle = std::max(fabs(roll),std::max(fabs(pitch),fabs(yaw)));
    double distSqrt = (trafo.translation()).squaredNorm();
    return (distSqrt > minTranslation*minTranslation || maxAngle > minRotation); // movment
}

bool Mapping::isVelocitySmallEnough(const Eigen::Isometry3d& trafo, double dt) const
{
	double roll, pitch, yaw;
    convertRotMatToEulerAngles(trafo.rotation(),roll,pitch,yaw);
    double maxAngle = std::max(fabs(roll),std::max(fabs(pitch),fabs(yaw)));
    double dist = trafo.translation().norm();
    return ((dist/dt) < maxVelocity && (maxAngle/dt) < maxAngularVelocity); // velocity
}

void Mapping::addFirstNode()
{
	nodes.clear();
	keyFrames.clear();
	poseGraph->addFirstNode();
	currentFrame.setId(poseGraph->getCurrentId());
	nodes.push_back(currentFrame);
	nodes.back().setKeyFrameFlag(true);
	keyFrames.push_back(nodes.back());
	nodes.back().deleteDepth();
	nodes.back().deleteRgb();
	nodes.back().deleteGray();
}

void Mapping::printPosition(const Eigen::Isometry3d& position)
{
	const Eigen::Matrix4d& matrix = position.matrix();
	printf ("Rotation :\n");
	printf ("    | %6.3lf %6.3lf %6.3lf | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
	printf ("R = | %6.3lf %6.3lf %6.3lf | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
	printf ("    | %6.3lf %6.3lf %6.3lf | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
	printf ("Translation x, y, z:\n");
	printf ("t = < %6.3lf, %6.3lf, %6.3lf >\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void Mapping::loopClosureDetection()
{
	lcBestIndex = -1;
	double bestDist = loopClosureDetectionThreshold;

	int nrOfKeyFramesToMatch = keyFrames.size();
	cout << "key frame size " << keyFrames.size() << endl; 
	while(nrOfKeyFramesToMatch > 0)
	{
		if(lcSmallestId > keyFrames.at(nrOfKeyFramesToMatch-1).getId())
			break;

		--nrOfKeyFramesToMatch;
	}

	//
	// search new loop closure
	for (int n = 0; n < nrOfKeyFramesToMatch; ++n)
	{
		double dist = cv::norm(keyFrames.at(n).getAverageDescriptors(), currentFrame.getAverageDescriptors(), cv::NORM_L2);
		if (dist < bestDist)
		{
			lcBestIndex = n;
			bestDist = dist;
		}
	}

	if(lcBestIndex >= 0)
	{
		matchTwoFrames(currentFrame, keyFrames.at(lcBestIndex), lcEnoughMatches, lcValidTrafo, lcTm, lcIm);
	}

	cout << lcBestIndex << "is the number of best index?" << endl; 
}

} /* namespace SLAM */
