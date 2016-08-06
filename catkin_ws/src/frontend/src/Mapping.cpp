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

#include "Mapping.h"
#include <ctime>

using namespace std;
using namespace cv;



namespace SLAM {

cv::FileStorage fileStore("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/backend/matlab_util/match1.txt",cv::FileStorage::WRITE);
cv::FileStorage fileStore2("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/backend/matlab_util/match2.txt",cv::FileStorage::WRITE);
cv::FileStorage fileConsensus("/home/tuofeichen/SLAM/MAV-Project/catkin_ws/src/backend/matlab_util/consensus.txt",cv::FileStorage::WRITE);


Mapping::Mapping(
		IFeatures* aFDEM,
		ITransformMatEst* aTME,
		IPoseGraph* aGO,
		RosHandler* aRos
		)
		//IMap* aMap
		
 : fdem(aFDEM), tme(aTME), poseGraph(aGO), px4(aRos)//, map3d(aMap)
{
	// check element are not pointing to 0
	assert(fdem);
	assert(tme);
	assert(poseGraph);
	srand(std::time(NULL));

	PosDebug.setIdentity();
	currentPosition = Eigen::Isometry3d::Identity();
	keyFrames.clear();
	nodes.clear();
}

Mapping::~Mapping()
{ }

void Mapping::run()
{

	double totalTime = 0;
	double relTime;

	// initialize variables
	bestInforamtionValue = 0;
	smallestId = lcSmallestId = poseGraph->getCurrentId();
	frames = 0;

	time.tic();

	if (!featureDetectionAndExtraction())
	{
		// cout << "bad feature!"<< endl;
		++badFrameCounter;
		fusePX4LPE(badFrame);

		return;
	}
	else if(nodes.size()!=0 && (nodes.back().getBadFrameFlag()))
	{
		// cout << "recover from badframe" << endl;
		fusePX4LPE(recoverFrame);
		return;
	}

	relTime = time.toc();
	totalTime += relTime;
//	cout << "Feature detection and extraction took " << relTime << "ms" << endl;
	
	if(!initDone)
	{
		addFirstNode();
		// updateMap();
		return;
	}

	time.tic();
	parallelMatching();
	
	relTime = time.toc();
	totalTime += relTime;
	// cout << "Parallel matching took " << relTime << "ms" << endl;

	
	// process graph
	time.tic();
	for(int thread = 0; thread < frames && !currentFrame.getDummyFrameFlag(); ++thread)
	{
		if (currentFrame.getId() < 0)
		{	
			tryToAddNode(thread); 
			
			if (currentFrame.getNewNodeFlag()) {
				px4->updateLpeCam(); //update px4 pose if new node if camera frame
			}
		}
		else
		{	
			addEdges(thread);
		}

	}

	if(searchLoopClosures && nodes.size() > neighborsToMatch + contFramesToMatch && lcRandomMatching == 0)
	{
		lcHandler.join();

		if(lcBestIndex > 0 && currentFrame.getId() >= 0) // (need lcBestIndex > 0 ?)
		{
			if(lcValidTrafo && lcEnoughMatches)
			{
				GraphProcessingResult res = processGraph(lcTm, lcIm, keyFrames.at(lcBestIndex).getId(), (currentFrame.getTime()-keyFrames.at(lcBestIndex).getTime()), false, true);
				if (res == trafoValid)
					smallestId = std::min(smallestId, keyFrames.at(lcBestIndex).getId());
			}
		}
	}


	 
	relTime = time.toc();
	totalTime += relTime;
	// cout << "Graph processing took " << relTime << "ms" << endl;

	if(loopClosureFound)
	{
		loopClosureFound = false; // avoid stuck in wrong loop closure for too long 
		++detLoopClsrsCounter;
		std::cout << "<---------------------------------------------- loop detected!" << std::endl;
	}

	//
	// print
	
	// if(currentFrame.getId() < 0)
	// {
	// 	// if(currentFrame.getDummyFrameFlag())
	// 	// 	// cout << "dropped (transformation too small or too big) " << endl;
	// 	// else
	// 	// 	cout << "dropped (added as dummy) " << endl;

	// 	if(!currentFrame.getDummyFrameFlag())
	// 		cout << "dropped (added as dummy) " << endl;

	// }

	// else
	// {
	// 	cout << "SLAM of frame nr " << frameCounter << " was processed" << endl;	
	// }

	
	// key frame, map and optimization
	time.tic();
	if((currentFrame.getId() < 0) && !currentFrame.getDummyFrameFlag())
	{ 
		if(exchangeFirstNode && nodes.size() == 1)
		{
			exchangeFirstFrame();
		}
		else
		{
			++noTrafoFoundCounter;
			if(addDummyNodeFlag)
			{
				++sequenceOfLostFramesCntr; 
				setDummyNode();
			}
		}
	}
	else if(currentFrame.getDummyFrameFlag())
	{ 
		// trafo to small or to big
		sequenceOfLostFramesCntr = 0;
		if(exchangeFirstNode && nodes.size() == 1)
		{
			exchangeFirstFrame();
		}
	}
	else
	{
		sequenceOfLostFramesCntr = 0;
		bool addedKeyFrame = searchKeyFrames();
		optimizeGraph(false);
//		boost::thread(&Mapping::optimizeGraph,this,false);

		if(addedKeyFrame)
		{
			if(loopClosureFound)
			{
				if(optimizeTillConvergence) // never actually enters
				{
					// optimize graph till convergenz
					optimizeGraph(true);
//					boost::thread(&Mapping::optimizeGraph,this,true);
				}
				loopClosureFound = false; // reset loop closure flag ONLY when new key frame is added
			}
			// update map
			// updateMap();
		}
	}

	relTime = time.toc();
	totalTime += relTime;
	// cout << "Optimization and map update took " << relTime << "ms" << endl;
	
	// cout << " and took " << totalTime << "ms"<< endl;

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

	if ((frame2.getBadFrameFlag())||frame2.getKeypoints().empty())
	{
		// just recover from a bad frame
		validTrafo = false;
		enoughMatches = false;
		return;
	}

	//
	// feature detecting, extracting and matching
	// 

	static char logName [50];

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
		
// #ifdef DEBUG
// 		if(( nodes.size() == DEBUG_NEW)&& (frame2.getId() == DEBUG_OLD) && validTrafo){ // sequential node
// 		// if (validTrafo)s
// 			cv::Mat imgMatch;
// 			cv::drawMatches(frame1.getGray(),frame1.getKeypoints(),frame2.getGray(),frame2.getKeypoints(), matches, \
// 			imgMatch,Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
// 		 	cv::namedWindow("Matching", WINDOW_NORMAL);
// 			cv::imshow("Matching",imgMatch);

// 			sprintf(logName,"Matching %d ",currentFrame.getId()); // new frame 
// 			cv::write(fileStore, logName, matchesIdx1);

// 			sprintf(logName,"Matching %d ", frame2.getId());	  // old frame
// 			cv::write(fileStore2,logName, matchesIdx2);
			

// 			sprintf(logName,"Consensus");	  // old frame
// 			cv::write(fileConsensus,logName, consensus);
			
// 			cout << " enter matching " << endl; 
// 			cv::waitKey(30);
		
// 		 }
// 		 else if (nodes.size()>DEBUG_NEW)
// 		 {
// 		 	fileStore.release();
// 		 	fileStore2.release();
// 		 	fileConsensus.release();
// 		 }
// #endif

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

Mapping::GraphProcessingResult Mapping::processGraph(const Eigen::Isometry3d& transformationMatrix, const Eigen::Matrix<double, 6, 6>& informationMatrix, int prevId, double deltaTime, bool tryToAddNode, bool possibleLoopClosure)
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
				
				if (px4->getLpe()(2,3)> 0.2){ //valid rangefinder 
					Matrix4d pos = currentPosition.matrix(); // fuse height with LPE constantly
					double height = pos(2,3);
					pos(2,3) = 0.1*height + 0.9*px4->getLpe()(2,3);
					currentPosition.matrix() = pos; 
				}

				poseGraph->addNode(currentPosition);
				currentFrame.setId(poseGraph->getCurrentId());
				nodes.push_back(currentFrame);
			}
			else
			{ 
			// still update current position: important
				currentFrame.setNewNodeFlag(false); 
			    currentPosition =  poseGraph->getPositionOfId(prevId)*transformationMatrix; 
				return trafoToSmall;
			}
		}
		else if(possibleLoopClosure){ 
			loopClosureFound = true;
		}

		// add edge to current node (only add edges)
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

			if (pNode->getBadFrameFlag()) // skip bad frames (should we match more? )
				continue;

			// reinitialize graph id every time new processing occus
			graphIds[frames] = pNode->getId(); // continuous node 
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
		// find neighbors of current node, exclude continuous nodes
		poseGraph->getEdgeNeighborsToCurrentNode(contFramesToMatch, neighborIds);

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
				lcSmallestId = std::min(lcSmallestId, graphIds[frames]); // also match with geodesic neighbor
				tmpPrev = tmp;

				const int nodesId = nodes.size() - 1 - (poseGraph->getCurrentId() - graphIds[frames]);

				if (nodes.at(nodesId).getBadFrameFlag())
					continue; // don't match bad frames
				
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

	// loop closures
	if(searchLoopClosures && nodes.size() > neighborsToMatch + contFramesToMatch && keyFrames.size() > lcRandomMatching)
	{
		if(lcRandomMatching == 0)
		{
			lcHandler = boost::thread(&Mapping::loopClosureDetection,this); // average descriptor method
		}
		else
		{
			int prevId = -1;

			for(int i = 0; i < lcRandomMatching; ++i,  ++frames)
			{
				// find non neighbor and sequencial nodes
				int id;
				int* ids 	  = new int [keyFrames.size()+1];
				int* temp_ids = new int [keyFrames.size()+1];
				*ids 		  = -1; 	// backward termination 
				bool isNeighbor;
				bool isRepeat = false; 

				do { 

					// randomly select key frames to match
					id = rand() % keyFrames.size();		
					ids++ ;
					*ids = id; 		// note down array of ids;  
		
					const int keyFrameId = keyFrames.at(id).getId();

					// no sequencials
					if(keyFrameId >= nodes.at(nodes.size()-contFramesToMatch).getId())
						continue;

					isNeighbor = false; // initialize as no neighbors

					for(int k=0; k < static_cast<int>(neighborIds.size()); ++k) // TODO faster implementation or better strategy
					{
						if(neighborIds.at(k) == keyFrameId)
						{
							isNeighbor = true;
							break;
						}
					}

					temp_ids = ids; //  check repeating key frame
					
					while(*temp_ids != -1)
					{ 
						if (*temp_ids == id) { // same key frames matched all the time?  {
							isNeighbor = false;
							isRepeat = true;
							break; 
						}
						else
							temp_ids --; 
					}

				} while(isNeighbor); //TODO check that not the same key frames are matched
				if (!isRepeat){
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
			// trafo too small 
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
		
		fusePX4LPE(dummyFrame);

		// fusePX4LPE(px4,dummyFrame);

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
	}
}

bool Mapping::searchKeyFrames()
{
	bool ret = false;

	if (keyFrames.size() == 0)
	{
		return false; // can be the case if first frame is bad frame
	}

	if (smallestId > keyFrames.back().getId())
	{
		for(int thread = 0; thread < frames; ++thread)
		{
			if (validTrafo[thread] &&  isVelocitySmallEnough(transformationMatrices[thread], deltaT[thread]))
			{
				Frame& keyFrame = nodes.back();
				// if (!keyFrame.getBadFrameFlag()) // bad frame should never be key frame ? but how then to build entire map ?
				// {
					keyFrame.setKeyFrameFlag(true);
					keyFrames.push_back(keyFrame);
					cout << "Added id " << keyFrame.getId() << " as key frame nr " << keyFrames.size() << "==========================================" << endl;
					ret = true;
					break;
				// }
			}
		}
	}

	// delete images
	// nodes.back().deleteRgb();
	// nodes.back().deleteDepth();
	// nodes.back().deleteGray();

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
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	FrameToPcConverter::getColorPC(keyFrames.back(), cloud);
	map3d->addMapPart(cloud, poseGraph->getPositionOfId(keyFrames.back().getId()));

	// // TODO when is a good time to update poses?
	for(int i = 0; i < keyFrames.size(); ++i)
		map3d->updatePose(i,poseGraph->getPositionOfId(keyFrames.at(i).getId()));

	// delete images out of the stored frames
	// keyFrames.back().deleteRgb();
	// keyFrames.back().deleteDepth(); // ?
	// keyFrames.back().deleteGray();
}

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
    // cout << "vel is " << maxVelocity << " ang is  " << maxAngularVelocity << endl; 
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

	// nodes.back().deleteDepth();
	// nodes.back().deleteRgb();
	// nodes.back().deleteGray();

}


void Mapping::fusePX4LPE(int frameType)
{


		Matrix4f 			  tm_lpe; 
		Matrix<float, 6, 6>   im_lpe; 
		double 				  dt_lpe; 

		imuCompensateCounter++;

		px4->getTm(tm_lpe,im_lpe,dt_lpe);
		validTrafo[0] 	 = 1;
		enoughMatches[0] = 1;
		transformationMatrices[0] = tm_lpe.cast<double>();
		informationMatrices[0] =  im_lpe.cast<double>();
		
		if (nodes.size()==0){
			cout << " add first node" << endl; 
			nodes.clear();
			keyFrames.clear();
			poseGraph->addFirstNode();
			currentFrame.setId(poseGraph->getCurrentId());
			nodes.push_back(currentFrame);
			nodes.back().setKeyFrameFlag(true);

			return;
		}
		

		graphIds[0] = nodes.size()-1;
		deltaT[0] = dt_lpe;

		tryToAddNode(0); 		// try to add one sequential node

		switch(frameType)
		{
			case badFrame:
				currentFrame.setBadFrameFlag(true);
				currentFrame.setKeyFrameFlag(false);
			break;

			case recoverFrame:
				currentFrame.setBadFrameFlag(false);
			break;

			case dummyFrame:
				currentFrame.setBadFrameFlag(false);
				currentFrame.setDummyFrameFlag(false);
			break;
		}

		if (currentFrame.getNewNodeFlag()){
			cout << "frame type " << frameType << " compensated by px4" << endl;
			px4->updateLpeCam();
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

void Mapping::loopClosureDetection()
{
	// need to change here? 
	lcBestIndex = -1;

	double bestDist = loopClosureDetectionThreshold;

	int nrOfKeyFramesToMatch = keyFrames.size() ;
	// 
	while(nrOfKeyFramesToMatch > 0)
	{
		if(lcSmallestId > keyFrames.at(nrOfKeyFramesToMatch-1).getId()){
			break;
		}

		--nrOfKeyFramesToMatch;
	}


	for (int n = 0; n < nrOfKeyFramesToMatch; ++n)
	{

		if (!keyFrames.at(n).getBadFrameFlag()){ // filter out bad frames there
			double dist = cv::norm(keyFrames.at(n).getAverageDescriptors(), currentFrame.getAverageDescriptors(), cv::NORM_L2);
			if (dist < bestDist)
			{
				lcBestIndex = n; // loop through to find the best index 
				bestDist = dist;
			}
		}
	}

	if(lcBestIndex >= 0)
	{

		matchTwoFrames(currentFrame, keyFrames.at(lcBestIndex), lcEnoughMatches, lcValidTrafo, lcTm, lcIm);

		// impose constraint here  

		if (lcEnoughMatches && lcValidTrafo)
		{

			// cout << nodes.size() << " is node size = " << pNode->getId() << endl;

			// Eigen::Isometry3d prevPos = poseGraph->getPositionOfId(pNode->getId(); // previous node position
			Eigen::Isometry3d prevPos = poseGraph->getPositionOfId(nodes.size()-1);

			Eigen::Isometry3d keyPos  = poseGraph->getPositionOfId(keyFrames.at(lcBestIndex).getId()); //current position

			Eigen::Matrix4d graphTm = keyPos.matrix() * prevPos.matrix().inverse(); // make the jump 
			
			// double dt  = currentFrame.getTime()- pNode->getTime();
			
			// only check distance (angle can have drastic change)
			
			// double roll, pitch, yaw; 
			
			// convertRotMatToEulerAngles(graphTm.topLeftCorner(3,3),roll,pitch,yaw);

			// double maxAngle = std::max(fabs(roll),std::max(fabs(pitch),fabs(yaw)));

			double maxDisp  = graphTm.topRightCorner(3,1).norm(); 
			
			if (maxDisp > maxVelocity){ 
				lcBestIndex = -1; 
			}
			else
			{
				// cout << "valid! match to key frame nr " << lcBestIndex << endl;   
			}
		}

	}

}

} /* namespace SLAM */
