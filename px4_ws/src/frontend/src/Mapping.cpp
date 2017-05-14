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
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
// #include <synch.h>

#include "Mapping.h"
#include <ctime> // for randn

using namespace std;
using namespace cv;



namespace SLAM {
Mapping::Mapping(
		IFeatures* aFDEM,
		ITransformMatEst* aTME,
		IPoseGraph* aGO,
		RosHandler* aRos//,
		// IMap* aMap
		)

 : fdem(aFDEM), tme(aTME), poseGraph(aGO), px4(aRos)//, map3d(aMap)
{
	// check element are not pointing to 0
	// assert(map3d);
	assert(fdem);
	assert(tme);
	assert(poseGraph);
	srand(std::time(NULL));

	currentPosition = Eigen::Isometry3d::Identity();
	keyFrames.clear();
	nodes.clear();
	nodes.reserve(100);
}

Mapping::~Mapping()
{ }

bool Mapping::extractFeature()
{
	double relTime = 0;
	// time.tic();

	if (!featureDetectionAndExtraction())
	{

		++badFrameCounter;
		// if(!(badFrameCounter%50))
			// cout << "bad feature!"<< endl;
		fusePX4LPE(badFrame);
		return 0;
	}

	return 1;
}

void Mapping::addNewNode()
{
	std::vector<Frame>::const_iterator lastNode = nodes.end()-1; // this is correct (vector::end is pass-of-end)
	graphIds[0] 	= lastNode->getId();
	lcSmallestId 	= std::min(lcSmallestId,graphIds[0]);
	deltaT[0] 		= currentFrame.getTime() - lastNode->getTime();

  // needs to be thread safe
	matchTwoFrames(boost::ref(currentFrame),
					boost::ref(*lastNode),
					boost::ref(enoughMatches[0]),
					boost::ref(validTrafo[0]),
					boost::ref(transformationMatrices[0]),
					boost::ref(informationMatrices[0]));
	 					//  start from second node when doing parallel matching

	tryToAddNode(0) ;	  //  changes currentFrame

	if (currentFrame.getNewNodeFlag()) {
			px4->updateLpeLastPose(); //update last pose (not necessary now)
	}
	else if (!validTrafo[0]) // invalid trafo estimate
	{
				setDummyNode();
	}

	// if (currentFrame.getId() > 0)
	// 	cout << "added frame " << currentFrame.getId() << "  to pose graph" << endl;

}

void Mapping::optPoseGraph()
{
	frames = 1;
	double totalTime = 0;
	double relTime = 0;
	// need to be thread safe
	mapMutex.lock();
	Frame procFrame = currentFrame; // make local copy due to threading
	mapMutex.unlock();

	time_delay.tic();

	parallelMatching(procFrame);
	relTime = time_delay.toc();
	totalTime += relTime;

	// process graph
	time_delay.tic();
	// again don't deal with immediate node
	for(int thread = 1; thread < frames && !procFrame.getDummyFrameFlag(); ++thread)
	{
		if (procFrame.getId() > 0)
			addEdges(thread,procFrame.getId());
	}

	if(searchLoopClosures && nodes.size() > neighborsToMatch + contFramesToMatch && lcRandomMatching == 0)
	{
		lcHandler.join();
		if(lcBestIndex > 0 && procFrame.getId() >= 0)
		{
			if(lcValidTrafo && lcEnoughMatches)
			{
				GraphProcessingResult res = processGraph(lcTm, lcIm, keyFrames.at(lcBestIndex).getId(),procFrame.getId(), (procFrame.getTime()-keyFrames.at(lcBestIndex).getTime()), false, true);
				if (res == trafoValid)
					smallestId = std::min(smallestId, keyFrames.at(lcBestIndex).getId());
			}
		}
	}

	relTime = time_delay.toc();
	totalTime += relTime;

	// cout << "Graph processing took " << relTime << " ms" << endl;
	if(loopClosureFound)
	{
		++detLoopClsrsCounter;
		// std::cout << "<----------------------------------------------------------------------- loop detected!" << std::endl;
	}


	// key frame, map and optimization
	time_delay.tic();

	if(procFrame.getDummyFrameFlag())
	{ // trafo to small or to big
		if(exchangeFirstNode && nodes.size() == 1)
		{
			exchangeFirstFrame();
		}
	}
	else
	{
		// search key frame
		bool addedKeyFrame = searchKeyFrames(procFrame);

		// optimize graph once
		graphHandler.join(); // avoid overflowing
		// optimizeGraph(false);
		graphHandler = boost::thread(&Mapping::optimizeGraph,this,false);

		if(addedKeyFrame)
		{
			if(loopClosureFound)
			{
				if(optimizeTillConvergence)
				{

					// optimize graph till convergenz
					// optimizeGraph(true);
					graphHandler = boost::thread(&Mapping::optimizeGraph,this,true);
				}
				loopClosureFound = false;
			}

		}
	}

	relTime = time_delay.toc();
	totalTime += relTime;
	// cout << "Optimization and map update took " << relTime << "ms" << endl;

	++nframeProc;
	frameProcMeanTime += totalTime;
	if(totalTime > frameProcMaxTime)
		frameProcMaxTime = totalTime;

	optFlag = true; // finished optimizing

}

void Mapping::run()
{

	double totalTime = 0;
	double relTime;
	// initialize variables
	bestInforamtionValue = 0;
	smallestId = lcSmallestId = poseGraph->getCurrentId();

	if(!initDone)
	{
		addFirstNode();
		return;
	}

	time.tic();

	addNewNode();

	if (optFlag && currentFrame.getNewNodeFlag() > 0) // new node then lets do graph optimization
	{
		optFlag 		 = false; // don't add new thread to the queue, this should be asynchronous
		// mapMutex.lock(); // flag is thread safe
		delayProc 	 = boost::thread(&Mapping::optPoseGraph,this); //shouldn't overlap with itself
		// mapMutex.unlock();
		// delayProc.join();
	}

}

void Mapping::addFrame(Frame& frame)
{
	++frameCounter;

	if (!initDone) // init done shouldn't be set here
	{
		if (!nodes.empty())
			initDone = true;
	}

	lastFrame = currentFrame;
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
	assert(frame1.getBadFrameFlag()!=1);
	assert(frame2.getBadFrameFlag()!=1);

	//
	// feature detecting, extracting and matching
	//


	std::vector<int> matchesIdx1;
	std::vector<int> matchesIdx2;
	std::vector< DMatch > matches;

	// if we want to show matches or not
	// enoughMatches = fdem->match(frame1.getKeypoints(), frame1.getDescriptors(), frame2.getKeypoints(), frame2.getDescriptors(), matchesIdx1, matchesIdx2);
	enoughMatches = fdem->match(frame1.getKeypoints(), frame1.getDescriptors(), frame2.getKeypoints(), frame2.getDescriptors(), matches);

	if (enoughMatches)
	{
		for (int i = 0;i < static_cast<int>(matches.size()); i++)
		{
			matchesIdx1.push_back(matches[i].queryIdx);
			matchesIdx2.push_back(matches[i].trainIdx);
		}

		// estimate transformation matrix
		//
		Eigen::Matrix4f tm, tm_temp, rot;
		rot = (Eigen::Matrix4f() <<
				  0,1,0,0,
				  0,0,1,0,
				  1,0,0,0,
				  0,0,0,1).finished();

		std::vector<int> consensus;
		validTrafo = tme->estimateTrafo(frame1.getKeypoints3D(), matchesIdx1, frame2.getKeypoints3D(), matchesIdx2, tm, informationMatrix,consensus);


		// align with px4 frame
		tm_temp = tm;
		tm.row(0) = tm_temp.row(2) * rot;
		tm.row(1) = tm_temp.row(0) * rot;
		tm.row(2) = tm_temp.row(1) * rot;
		tm.col(3) =	rot.inverse() * tm_temp.col(3);
		transformationMatrix.matrix() = tm.cast<double>();
	}
	else
		validTrafo = false;
}

Mapping::GraphProcessingResult Mapping::processGraph(const Eigen::Isometry3d& transformationMatrix, const Eigen::Matrix<double, 6, 6>& informationMatrix, int prevId, int currId, double deltaTime, bool tryToAddNode, bool possibleLoopClosure)
{

	assert(prevId >= 0);
	assert(!(tryToAddNode && possibleLoopClosure));
	if(isVelocitySmallEnough(transformationMatrix, deltaTime))
	{
		// try to add current node
		if (tryToAddNode)
		{
			if(isMovementBigEnough(transformationMatrix))
			{
				currentFrame.setNewNodeFlag(true);
				currentPosition = (poseGraph->getPositionOfId(prevId))*transformationMatrix;

				// if (px4->getLpe()(2,3)> 0.2){ //valid rangefinder
				// 	cout << "fuse with rangefinder" << endl;
				// 	Matrix4d pos = currentPosition.matrix(); // fuse height with LPE constantly
				// 	double height = pos(2,3);
				// 	pos(2,3) = 0.1*height + 0.9*px4->getLpe()(2,3);
				// 	currentPosition.matrix() = pos;
				// }

				currentFrame.setPosition(currentPosition.matrix().cast<float>()); // note down LPE
				poseGraph->addNode(currentPosition);
				currentFrame.setId(poseGraph->getCurrentId());
				currId = currentFrame.getId();
				nodes.push_back(currentFrame);
			}
			else
			{
  			currentFrame.setNewNodeFlag(false);
			  currentPosition =  poseGraph->getPositionOfId(prevId)*transformationMatrix;
				return trafoToSmall;
			}
		}
		else if(possibleLoopClosure){
			loopClosureFound = true;
		}

		// add edge to current node (only add edges)
		poseGraph->addEdgeFromIdToId(transformationMatrix, informationMatrix, prevId,currId);

		return trafoValid;
	}
	else
	{
		return trafoToBig;
	}
}

bool Mapping::featureDetectionAndExtraction()
{
	// run feature detection and extraction on new frame
	//
	currentFrame.setKeypoints( fdem->detect(currentFrame.getGray()) );
	// extract descriptors
	currentFrame.setDescriptors( fdem->extract(currentFrame.getGray(), currentFrame.getKeypoints()) );

	if (currentFrame.getKeypoints().size() < minNumberOfKeyPoints){
		return false;
	}
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
		}
}

void Mapping::parallelMatching(Frame procFrame)
{
	std::vector<int> neighborIds;
	const int contFramesToMatchTmp = (nodes.size() >= (neighborsToMatch + contFramesToMatch)) \
	? contFramesToMatch : neighborsToMatch + contFramesToMatch;

	// match continuous
	if(contFramesToMatch > 0 && nodes.size()-1 > 0)
	{
		const int nrOfContFramesToMatch = std::min<int>(nodes.size()-1, contFramesToMatchTmp);
		std::vector<Frame>::const_iterator pNode = nodes.end() - 2;
		// let first consecutive match be handled by addnode

		for (int frame = 0; frame < nrOfContFramesToMatch-1; ++frame, ++frames, --pNode)
		{
			//  one less continuous frame to match !!
			//  'frame'  is the loop variables
			// 	'frames' is the cont+neighbor+lc overall counter
			graphIds[frames] = pNode->getId();
			lcSmallestId = std::min(lcSmallestId, graphIds[frames]);

			// match frames
			deltaT[frames] = procFrame.getTime() - pNode->getTime();
			handler[frames] = boost::thread(&Mapping::matchTwoFrames, this,
					boost::ref(procFrame),
					boost::ref(*pNode),
					boost::ref(enoughMatches[frames]),
					boost::ref(validTrafo[frames]),
					boost::ref(transformationMatrices[frames]),
					boost::ref(informationMatrices[frames]) );
		}
	}
	if (nodes.size() >= (neighborsToMatch + contFramesToMatch) && nodes.size() > 0 )
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
				deltaT[frames] = procFrame.getTime() - nodes.at(nodesId).getTime();
				handler[frames] = boost::thread(&Mapping::matchTwoFrames, this,
						boost::ref(procFrame),
						boost::ref(nodes.at(nodesId)),
						boost::ref(enoughMatches[frames]),
						boost::ref(validTrafo[frames]),
						boost::ref(transformationMatrices[frames]),
						boost::ref(informationMatrices[frames]) );
			}
		}
	}


	// loop closures
	if(searchLoopClosures && nodes.size() > neighborsToMatch + contFramesToMatch && keyFrames.size() > lcRandomMatching)
	{
	if(lcRandomMatching == 0)
		{
			lcHandler = boost::thread(&Mapping::loopClosureDetection,this,procFrame);
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
				deltaT[frames] = procFrame.getTime() - keyFrames.at(id).getTime();
				handler[frames] = boost::thread(&Mapping::matchTwoFrames, this,
						boost::ref(procFrame),
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
		for (int thread = 0; thread <= frames; ++thread)
		{
			handler[thread].join(); // sleep until all threads are finished with their work
		}

}

void Mapping::tryToAddNode(int thread)
{
	if (validTrafo[thread] && enoughMatches[thread])
	{
		GraphProcessingResult result = processGraph(transformationMatrices[thread], informationMatrices[thread], graphIds[thread], currentFrame.getId(), deltaT[thread], true, false);

		if(result == trafoValid)
		{
			smallestId = std::min(smallestId, graphIds[thread]);
			bestInforamtionValue = informationMatrices[thread](0,0);
			currentPosition = poseGraph->getPositionOfId(graphIds[thread])*transformationMatrices[thread];
		}
		else if(result == trafoToSmall)
		{
			currentPosition = poseGraph->getPositionOfId(graphIds[thread])*transformationMatrices[thread];
			currentFrame.setDummyFrameFlag(true);
			++trafoToSmallCounter; // debug
		}
		else if (result == trafoToBig)
		{
			currentFrame.setDummyFrameFlag(true);
			++trafoVelocityToBigCounter; // debug
		}
	}
	else
	{
			// cout << "invalid trafo " << endl;
	}
}

void Mapping::addEdges(int thread, int currId)
{
	if (validTrafo[thread] && enoughMatches[thread])
	{
		GraphProcessingResult result;
		if(lcRandomMatching == 0)
			result = processGraph(transformationMatrices[thread], informationMatrices[thread], graphIds[thread], currId, deltaT[thread], false, false);
		else
			result = processGraph(transformationMatrices[thread], informationMatrices[thread], graphIds[thread], currId, deltaT[thread], false, (thread >= contFramesToMatch + neighborsToMatch));

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

	currentFrame.setBadFrameFlag(3);    // set dummy flag so that not publish this position

	++noTrafoFoundCounter;

	if(addDummyNodeFlag)
	{
		++sequenceOfLostFramesCntr;
	}

	if(sequenceOfLostFramesCntr > dummyFrameAfterLostFrames)
	{
		sequenceOfLostFramesCntr = 0; //reset sequence lost frame counter
		fusePX4LPE(dummyFrame);
	}

		// if(!nodes.back().getDummyFrameFlag())
		// {
		// 	// add node
		// 	const Eigen::Isometry3d dummyTm = Eigen::Isometry3d::Identity();
		// 	const Eigen::Matrix<double, 6, 6> dummyInfoMat = Eigen::Matrix<double, 6, 6>::Identity() * 1e-100;
		// 	poseGraph->addNode(currentPosition);
		// 	poseGraph->addEdgeFromIdToCurrent(dummyTm, dummyInfoMat, nodes.back().getId());
		// 	currentFrame.setId(poseGraph->getCurrentId());
		// 	currentFrame.setDummyFrameFlag(true);
		// 	nodes.push_back(currentFrame);

		// 	++dummyNodeCounter; //debug
		// }
		// else if (currentFrame.getKeypoints().size() > nodes.back().getKeypoints().size())
		// {

		// 	currentFrame.setDummyFrameFlag(true);
		// 	currentFrame.setId(nodes.back().getId());
		// 	nodes.back() = currentFrame;
		// }
	// }
}

bool Mapping::searchKeyFrames(Frame procFrame)
{
	bool ret = false;
	if (smallestId > keyFrames.back().getId())
	{
		// cout << "smallest id " << smallestId << " that match to current frame " << procFrame.getId() << endl;

		for(int thread = 0; thread < frames; ++thread)
		{
			if (validTrafo[thread] &&  isVelocitySmallEnough(transformationMatrices[thread], deltaT[thread]))
			{
					Frame& keyFrame = nodes.back();
					keyFrame.setKeyFrameFlag(true);
					mapUpdate = true;
					keyFrames.push_back(keyFrame);
					// cout << "Added id " << keyFrame.getId() << " as key frame <================================================================================" << endl;
					// cout << "Current frame flag ? " << currentFrame.getKeyFrameFlag() << " " << procFrame.getKeyFrameFlag() << endl;
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

// void Mapping::updateMap()
// {
// 	// pcl::PointCloud<pcl::PointXYZRGB> cloud;
// 	// FrameToPcConverter::getColorPC(keyFrames.back(), cloud);
// 	// map3d->addMapPart(cloud, poseGraph->getPositionOfId(keyFrames.back().getId()));
// 	// //TODO when is a good time to update poses? (this is pretty good)
// 	// for(int i = 0; i < keyFrames.size(); ++i)
// 	// {
// 	// 	map3d->updatePose(i,poseGraph->getPositionOfId(keyFrames.at(i).getId()));
// 	// }
//
// 	// delete images out of the stored frames
// 	// keyFrames.back().deleteRgb();
// 	// keyFrames.back().deleteDepth();
// 	// keyFrames.back().deleteGray();
// }

void Mapping::convertRotMatToEulerAngles(const Eigen::Matrix3d& t, double& roll, double& pitch, double& yaw) const
{
	roll  = 	atan2(t(2,1),t(2,2)); // roll (around x)
	pitch = 	atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2))); // pitch (around y)
	yaw   = 	atan2(t(1,0),t(0,0)); // yaw (around z)
}

bool Mapping::isMovementBigEnough(const Eigen::Isometry3d& trafo) const
{
	double roll, pitch, yaw;
	convertRotMatToEulerAngles(trafo.rotation(),roll,pitch,yaw);
	double maxAngle = std::max(fabs(roll),std::max(fabs(pitch),fabs(yaw)));
    double distSqrt = (trafo.translation()).squaredNorm();

    if (minTranslation < 0)
    	return (maxAngle > minRotation);
    else
	    return (maxAngle > minRotation)||(distSqrt > minTranslation*minTranslation);
}

bool Mapping::isVelocitySmallEnough(const Eigen::Isometry3d& trafo, double dt) const
{
	double roll, pitch, yaw;
    convertRotMatToEulerAngles(trafo.rotation(),roll,pitch,yaw);
    double maxAngle = std::max(fabs(roll),std::max(fabs(pitch),fabs(yaw)));
    double dist = trafo.translation().norm();
    return ((dist/dt) < maxVelocity && (maxAngle/dt) < maxAngularVelocity); // velocity
}

//TODO need to handle if first node is not valid node

void Mapping::addFirstNode()
{
	nodes.clear();
	keyFrames.clear();
	poseGraph->addFirstNode();
	currentFrame.setId(poseGraph->getCurrentId());
	nodes.push_back(currentFrame);
	mapUpdate = true;
	nodes.back().setKeyFrameFlag(true);
	keyFrames.push_back(nodes.back());

	nodes.back().deleteDepth();
	nodes.back().deleteRgb();
	nodes.back().deleteGray();
}


void Mapping::fusePX4LPE(int frameType)
{
		Isometry3d	  		 tm_lpe;
		Matrix<double, 6, 6>  im_lpe;
		int 						id;
		imuCompensateCounter++;

		switch(frameType)
		{
			case badFrame:
				currentFrame.setBadFrameFlag(1); 		 // bad feature
				currentFrame.setKeyFrameFlag(false); // shouldn't allow bad frame as key frame for PCL
				currentPosition = px4 -> getLpe().cast<double>();  // directly use LPE to be consistant
				// no new node is set, no keyframe is set
			break;

			// case recoverFrame:							  // recover from bad frame (not necessary)
			// 	currentFrame.setBadFrameFlag(2);
			// break;

			case dummyFrame:

			// most likely due to recovery from bad frame, directly add new node
				currentFrame.setBadFrameFlag(3);    // dummy frame flag
				currentFrame.setNewNodeFlag(true);  // new node flag (should we?)
				currentPosition = px4 -> getLpe().cast<double>();  // directly use LPE to be consistant
				currentFrame.setPosition(currentPosition.matrix().cast<float>());
				poseGraph->addNode(currentPosition);

				id = poseGraph->getCurrentId(); // current id
				currentFrame.setId(id);
				nodes.push_back(currentFrame);
				tm_lpe = poseGraph->getPositionOfId(id-1).inverse() * currentPosition;
				im_lpe = Matrix<double, 6, 6>::Identity() * 5000; // (some constant value, maybe should poll mavros)

				poseGraph->addEdgeFromIdToId(tm_lpe,im_lpe,id-1,id); //add immediate edge
			break;
		}

		if (currentFrame.getNewNodeFlag()){ // after fusion

			// should we consider search key frame for dummy frame?
			// Eigen::Isometry3d prevKey = poseGraph->getPositionOfId(keyFrames.back().getId());
			// Eigen::Matrix4d keyTm = prevKey.matrix().inverse()*currentPosition.matrix();
			// double maxDisp  = keyTm.topRightCorner(3,1).norm();
			// double roll, pitch, yaw;
			// convertRotMatToEulerAngles(keyTm.topLeftCorner(3,3),roll,pitch,yaw);
			// double maxAngle = std::max(fabs(roll),std::max(fabs(pitch),fabs(yaw)));
			// // if (maxAngle > 30 * minRotation || maxDisp > 30 * minTranslation)
			// // {
			// // 	Frame& keyFrame = nodes.back();
			// // 	keyFrame.setKeyFrameFlag(true);
			// // 	keyFrames.push_back(keyFrame);
			// // 	cout << "Added frame (dummy) " << keyFrames.back().getId() << " as key frame------------------- " << keyFrames.size() << endl;
			// // }

			// if (!(currentFrame.getId()%10))
			// 	cout << "frame type " << frameType << " compensated by px4" << endl;
				// cout << "update lpe cam pose in bad frame" << endl;
				px4->updateLpeLastPose();
		}
		return;
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

void Mapping::loopClosureDetection(Frame procFrame)
{
	lcBestIndex = -1;
	double bestDist = loopClosureDetectionThreshold;

	int nrOfKeyFramesToMatch = keyFrames.size();
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
		double dist = cv::norm(keyFrames.at(n).getAverageDescriptors(), procFrame.getAverageDescriptors(), cv::NORM_L2);
		if (dist < bestDist)
		{
			lcBestIndex = n;
			bestDist = dist;
		}
	}

	if(lcBestIndex >= 0) // match to best keyframe
	{
		matchTwoFrames(procFrame, keyFrames.at(lcBestIndex), lcEnoughMatches, lcValidTrafo, lcTm, lcIm);
		// impose constraint here if needed
		// if (lcEnoughMatches && lcValidTrafo)
		// {
		// 	Eigen::Isometry3d prevPos = poseGraph->getPositionOfId(nodes.size()-1);
		// 	Eigen::Isometry3d keyPos  = poseGraph->getPositionOfId(keyFrames.at(lcBestIndex).getId()); //current position
		// 	Eigen::Matrix4d graphTm = keyPos.matrix().inverse() * prevPos.matrix(); // make the jump
		//
		// 	// double dt  = procFrame.getTime()- pNode->getTime();
		// 	// only check distance (angle can have drastic change)
		// 	// double roll, pitch, yaw;
		// 	// convertRotMatToEulerAngles(graphTm.topLeftCorner(3,3),roll,pitch,yaw);
		// 	// double maxAngle = std::max(fabs(roll),std::max(fabs(pitch),fabs(yaw)));
		// 	double maxDisp  = graphTm.topRightCorner(3,1).norm();
		//
		// 	if (maxDisp > maxVelocity){
		// 		lcBestIndex = -1;
		// 	}
		// 	// else
		// 	// {
		// 	// 	// cout << "valid! match to key frame nr " << lcBestIndex << endl;
		// 	// }
		// }
	}
}

} /* namespace SLAM */
