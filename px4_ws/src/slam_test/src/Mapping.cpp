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
		IPoseGraph* aGO,
		IMap* aMap
		)
 : fdem(aFDEM), tme(aTME), poseGraph(aGO), map3d(aMap),
 pc1normals(new pcl::PointCloud<pcl::PointXYZRGBNormal> ()), pc2normals(new pcl::PointCloud<pcl::PointXYZRGBNormal> ())
{
	// check element are not pointing to 0
	assert(fdem);
	assert(tme);
	assert(poseGraph);
	assert(map3d);

	currentPosition = Eigen::Isometry3d::Identity();
	keyframePosition = Eigen::Isometry3d::Identity();
	previousTransformation = Eigen::Isometry3d::Identity();
	keyFrames.clear();
	nodes.clear();
	nodes.reserve(1); // allocate memory for nodes
}

Mapping::~Mapping()
{
	// delete fdem;
	// delete tme;
	// delete poseGraph;
	// delete map3d;
}

void Mapping::addNewNode()
{

	std::vector<Frame>::const_iterator lastNode = nodes.end()-1; // this is correct (vector::end is pass of end)
	graphIds[0] 	= lastNode->getId();
	lcSmallestId 	= std::min(lcSmallestId,graphIds[0]);
	deltaT[0] 		= currentFrame.getTime() - lastNode->getTime();

  // needs to be thread safe
	// mapMutex.lock();
	matchTwoFrames(boost::ref(currentFrame),
					boost::ref(previousFrame),
					boost::ref(enoughMatches[0]),
					boost::ref(validTrafo[0]),
					boost::ref(transformationMatrices[0]),
					boost::ref(informationMatrices[0]));
	//cv::imshow("prev frame", previousFrame.getRgb());
	//cv::waitKey(10);
	//previousFrame = *lastNode;
	/*dynamicHandler = boost::thread(&DynamicObj::dynamicObjectRemoval,
								&dynamicObj,
								transformationMatrices[0].matrix().cast <float>(),
								currentFrame, 
								previousFrame);
	dynamicHandler.join();*/
	//cv::Mat filteredFrame = dynamicObj.getFilteredDepth();
	//currentFrame.setDepth(dynamicObj.getFilteredDepth());
	//cv::imshow("measured depth", currentFrame.getDepth());
	//cv::imshow("filtered depth", filteredFrame);
	//cv::waitKey(10);

	 					//  start from second node when doing parallel matching
	/*pcl::PointCloud<pcl::PointXYZRGB> pc1;
	pcl::PointCloud<pcl::PointXYZRGB> pc2;
	FrameToPcConverter::getColorPC(currentFrame, pc1);
	FrameToPcConverter::getColorPC(previousFrame, pc2);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc1normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc2normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(pc1,*pc1normals);
	pcl::copyPointCloud(pc2,*pc2normals);
	//ICP icp;*/
	if (validTrafo[0])
		icp.setTransformationGuess(transformationMatrices[0].matrix().cast <float>());
	else
		icp.setTransformationGuess(previousTransformation.matrix().cast <float>());  // does inverse make sense??
	//icp.filteringAndProcessing(pc1normals);
	//icp.filteringAndProcessing(pc2normals);
	/*validTrafo[0] = icp.run(pc2normals,pc1normals) || validTrafo[0];
	//enoughMatches[0] = validTrafo[0];
	transformationMatrices[0].matrix() = icp.getFinalTransformation().cast <double>();
	previousTransformation = transformationMatrices[0];*/
	//*pc2normals = *pc1normals;
	//mapMutex.lock();
	//currentFrame.setDepth(dynamicObj.getFilteredDepth());
	//mapMutex.unlock();
	//previousFrame = currentFrame;

	tryToAddNode(0) ;	  //  chages currentFrame
	// mapMutex.unlock();
	// cout << "added frame " << currentFrame.getId() << "  to pose graph" << endl;
	// if (currentFrame.getId()<0)
	// {
	// currentPosition is updated (but we should forget current node? )

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
		std::cout << "<---------------------------- loop detected!" << std::endl;
	}


	// if(procFrame.getId() < 0)
	// {
	// 	if(procFrame.getDummyFrameFlag()){
	// 		cout << "SLAM of frame nr" << frameCounter << " was ";
	// 		cout << "dropped (transformation to small or to big) ";
	// 	}
	// 	else {
	// 		cout << "SLAM of frame nr" << frameCounter << " was ";
	// 		cout << "dropped (added as dummy) " << endl;
	// 	}
	// }

	// key frame, map and optimization
	time_delay.tic();
	// if(procFrame.getId() < 0)
	// {
	// 	// invalid frame
	// 	if(exchangeFirstNode && nodes.size() == 1)
	// 	{
	// 		exchangeFirstFrame();
	// 	}
	// 	else
	// 	{
	// 		++noTrafoFoundCounter;
	// 		// setting a dummy node is needed, otherwise the track of frames can be lost
	// 		if(addDummyNodeFlag)
	// 		{
	// 			++sequenceOfLostFramesCntr;
	// 			setDummyNode();
	// 		}
	// 	}
	// }
	// else

	if(procFrame.getDummyFrameFlag())
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

			// update map
			updateMap();
		}
	}

	relTime = time_delay.toc();
	totalTime += relTime;
	// cout << "Optimization and map update took " << relTime << "ms" << endl;


	++nframeProc;
	frameProcMeanTime += totalTime;
	if(totalTime > frameProcMaxTime)
		frameProcMaxTime = totalTime;

	cout << "xxxxxx graph took  " << totalTime << " ms"<< endl;

	optFlag = true;

}

void Mapping::run()
{

	double totalTime = 0;
	double relTime;
	// cout << "=== run start " << endl;

	/*pcl::PointCloud<pcl::PointXYZRGB> pc1;
	pcl::PointCloud<pcl::PointXYZRGB> pc2;
	icpHandler1 = boost::thread(&FrameToPcConverter::getColorPC,currentFrame,boost::ref(pc1));	// just use previousFrame data instead, store it
	icpHandler2 = boost::thread(&FrameToPcConverter::getColorPC,previousFrame,boost::ref(pc2));	
	icpHandler2.join();
	icpHandler1.join();

	pcl::copyPointCloud(pc1,*pc1normals);
	pcl::copyPointCloud(pc2,*pc2normals);

	icpHandler1 = boost::thread(&ICP::filteringAndProcessing,pc1normals);
	icpHandler2 = boost::thread(&ICP::filteringAndProcessing,pc2normals);*/
	icpHandler1 = boost::thread(&ICP::preprocessing,currentFrame,pc1normals);
	icpHandler1.join();
	icpHandler2 = boost::thread(&ICP::preprocessing,previousFrame,pc2normals);
	icpHandler2.join();
	bool enoughFeatures = featureDetectionAndExtraction();
	//icpHandler2.join();
	//icpHandler1.join();

	// initialize variables
	bestInforamtionValue = 0;
	smallestId = lcSmallestId = poseGraph->getCurrentId();
	// frames = 0; unnecessary

	time.tic();
	if (!enoughFeatures)
	{
		++badFrameCounter;
		return; // immediately return if bad frame
	}

	relTime = time.toc();
	cout << "------ Feature dem took " << relTime << " ms" << endl;

	if(!initDone)
	{
		addFirstNode();
		updateMap();
		return;
	}

	// realTimeProc = boost::thread(&Mapping::addNewNode,this);
	time.tic();
	// realTimeProc.join(); // wait for real time processing to join
	addNewNode();
	cout << "===== add node takes " << time.toc() << " ms" << endl;

	if (optFlag && currentFrame.getId() > 0) // new node then lets do graph optimization
	{
		optFlag 		 = false; // don't add new thread to the queue, this should be asynchronous
		mapMutex.lock(); // flag is thread safe
		delayProc 	 = boost::thread(&Mapping::optPoseGraph,this); //shouldn't overlap with itself
		mapMutex.unlock();
		// delayProc.join();
	}

}

void Mapping::addFrame(Frame& frame)
{
	++frameCounter;

	if (!initDone)
	{
		if (!nodes.empty())
			initDone = true;
	}

	currentFrame = frame;	// am I freeing anything here?
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
				// add new node here
				currentPosition =  (poseGraph->getPositionOfId(prevId))*transformationMatrix;
				keyframePosition =  keyframePosition*transformationMatrix;
				poseGraph->addNode(currentPosition);
				currentFrame.setId(poseGraph->getCurrentId());
				currId = currentFrame.getId();
				nodes.push_back(currentFrame);
			}
			else
			{ // transformation to small
				currentPosition =  poseGraph->getPositionOfId(prevId)*transformationMatrix; // still update current position
				keyframePosition =  keyframePosition*transformationMatrix;
				return trafoToSmall; // don't add node onto the pose graph but should update position
			}
		}
		else if(possibleLoopClosure)
			loopClosureFound = true;

		// add edge to current node
		poseGraph->addEdgeFromIdToId(transformationMatrix, informationMatrix, prevId, currId);
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
	currentFrame.setKeypoints(fdem->detect(currentFrame.getGray()));

	// extract descriptors
	currentFrame.setDescriptors(fdem->extract(currentFrame.getGray(), currentFrame.getKeypoints()) );

	if (currentFrame.getKeypoints().size() < minNumberOfKeyPoints) // valid frame or not
		return false;
	else
		return true;
}


void Mapping::exchangeFirstFrame()
{
		if(currentFrame.getKeypoints().size() > nodes.back().getKeypoints().size())
		{
			addFirstNode();
			map3d->clear();
			map3d->clearMap();
			map3d->clearTrajectory();
			currentPosition = poseGraph->getCurrentPosition();
			updateMap();
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

	if (validTrafo[thread])
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
			currentFrame.setDummyFrameFlag(true);
			currentPosition = poseGraph->getPositionOfId(graphIds[thread])*transformationMatrices[thread];
			cout << "small trafo!!" << endl;
			++trafoToSmallCounter; // debug
		}
		else if (result == trafoToBig)
		{
			currentFrame.setDummyFrameFlag(true);
			++trafoVelocityToBigCounter; // debug
		}
	}
}

void Mapping::addEdges(int thread, int currId)
{
	if (validTrafo[thread])
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
	if(sequenceOfLostFramesCntr > dummyFrameAfterLostFrames)
	{
		if(!nodes.back().getDummyFrameFlag()) // last node is valid
		{
			// add dummy node
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
			nodes.back() = currentFrame; // replace last frame with current frame?
		}
	}
}

bool Mapping::searchKeyFrames(Frame procFrame)
{
	bool ret = false;
	//procFrame.setDepth(dynamicObj.getFilteredDepth());
	/*if (smallestId > keyFrames.back().getId())
	{
		// cout << "smallest id " << smallestId << " that match to current frame " << procFrame.getId() << endl;

		for(int thread = 0; thread < frames; ++thread)
		{
			if (validTrafo[thread] &&  isVelocitySmallEnough(transformationMatrices[thread], deltaT[thread]))
			{
					keyframePosition = Eigen::Isometry3d::Identity();
					Frame& keyFrame = nodes.back();
					keyFrame.setKeyFrameFlag(true);
					keyFrames.push_back(keyFrame);
					cout << "Added id " << keyFrame.getId() << " as key frame <=========================================" << endl;

					ret = true;
					break;
			}
		}
	}*/
	if (isKeyframeMovementBigEnough(keyframePosition))
	{
		keyframePosition = Eigen::Isometry3d::Identity();
		Frame& keyFrame = nodes.back();
		keyFrame.setKeyFrameFlag(true);
		keyFrames.push_back(keyFrame);
		cout << "Added id " << keyFrame.getId() << " as key frame <=========================================" << endl;

		ret = true;
	}
	

	previousFrame = nodes.back();
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
	mapMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	FrameToPcConverter::getColorPC(keyFrames.back(), cloud);
	map3d->addMapPart(cloud, poseGraph->getPositionOfId(keyFrames.back().getId()));

	// TODO when is a good time to update poses?
	for(int i = 0; i < keyFrames.size(); ++i)
		map3d->updatePose(i,poseGraph->getPositionOfId(keyFrames.at(i).getId()));

	// delete images out of the stored frames
	keyFrames.back().deleteRgb();
	keyFrames.back().deleteDepth();
	keyFrames.back().deleteGray();
	mapMutex.unlock();
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

bool Mapping::isKeyframeMovementBigEnough(const Eigen::Isometry3d& trafo) const
{
	double roll, pitch, yaw;
	convertRotMatToEulerAngles(trafo.rotation(),roll,pitch,yaw);
	double maxAngle = std::max(fabs(roll),std::max(fabs(pitch),fabs(yaw)));
    double distSqrt = (trafo.translation()).squaredNorm();
    return (distSqrt > keyframeMinTranslation*keyframeMinTranslation || maxAngle > keyframeMinRotation); // movment
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
	nodes.push_back(currentFrame);		//  1st copy constructor
	nodes.back().setKeyFrameFlag(true);
	keyFrames.push_back(nodes.back()); // 2nd copy constrictpr
	previousFrame = nodes.back();
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
	}
}

} /* namespace SLAM */
