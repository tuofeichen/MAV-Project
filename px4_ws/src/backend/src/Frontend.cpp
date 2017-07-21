/**
 * @file Frontend.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the Frontend class.
 *
 */

#include <iostream>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "Frontend.h"

using namespace SLAM;

//
// callback wrappers
//
static void deserializeRgbWrapper(const uint8_t* data, int size, void* context)
{
	reinterpret_cast<Frontend*>(context)->deserializeRgb(data, size);
}

static void deserializeDepthWrapper(const uint8_t* data, int size, void* context)
{
	reinterpret_cast<Frontend*>(context)->deserializeDepth(data, size);
}

//static void deserializeEdgeWrapper(const uint8_t* data, int size, void* context)
//{
//	reinterpret_cast<Frontend*>(context)->deserializeEdge(data, size);
//}

static void deserializeNodeWrapper(const uint8_t* data, int size, void* context)
{
	reinterpret_cast<Frontend*>(context)->deserializeCurrentNode(data, size);
}



//
// implementation frontend
//
Frontend::Frontend(const char* ip, int port)
 : client(ip, port), protocolHandler(&client)
{
	protocolHandler.registerRecvCallback(UavComProtocol::CMD1::RGB,deserializeRgbWrapper,this);
	protocolHandler.registerRecvCallback(UavComProtocol::CMD1::DEPTH,deserializeDepthWrapper,this);
//	protocolHandler.registerRecvCallback(UavComProtocol::CMD1::EDGE,deserializeEdgeWrapper,this);
	protocolHandler.registerRecvCallback(UavComProtocol::CMD1::CURRENT_NODE,deserializeNodeWrapper,this);

	queueSRgb.push(std::vector<uint8_t>());
	queueSDepth.push(std::vector<uint8_t>());
	queueSTimeStamp.push(std::vector<uint8_t>());
	queueSTm.push(std::vector<uint8_t>());
	queueSIm.push(std::vector<uint8_t>());
	queueSPos.push(std::vector<uint8_t>());
	queueToNode.push(-1);

	std::cout << "Try to connect to server..." << std::endl;
	client.connect();
	std::cout << "Connection established!" << std::endl;

	// start receiving
	boost::thread(&TcpClient::run, &client);
}



Frontend::~Frontend()
{
	client.stop();
}

void Frontend::getUpdateGraph(std::vector<Eigen::Isometry3d> &tm, int numNode)
{
	Eigen::Matrix4f endFlag;
	endFlag << -1, 0,  0,  0,
				0, -1, 0,  0,
				0, 0, -1,  0,
				0, 0,  0, -1;

	int cnt = 0; // update all node positions on the graph

	while (cnt < numNode)
	{
		if (queueSPos.size() > 1){
		deserializeTm(queueSPos.front(),trafoMat);

		if (endFlag.cast<double>().isApprox(trafoMat.matrix()))
		{
			boost::mutex::scoped_lock lock(queueEdgeMutex); // lock queue
			queueSPos.pop();
			break; // avoid overflow
		}

		tm.push_back (trafoMat);
		{
			boost::mutex::scoped_lock lock(queueEdgeMutex); // lock queue
			queueSPos.pop();
		}
		cnt++;
		}
		else
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(5));
		}
	}

	// wait for more buffer to come


}

int Frontend::getNewNode(Frame& frame)
{
	if( queueSRgb.size() > 1 && queueSDepth.size() > 1)
	{
		// create images
		rgb = decodeRgb(queueSRgb.front());
		gray =  boost::shared_ptr<cv::Mat>(new cv::Mat(rgb->size(), CV_8UC1));
		cv::cvtColor(*rgb, *gray, CV_BGR2GRAY);

		// create image
		depth = decodeDepth(queueSDepth.front());

//		// create edge
		timeStamp = boost::shared_ptr<double>(new double);
		*timeStamp = 0.1;		// return new node data


		frame = Frame(rgb, gray, depth, timeStamp);
//		tm = trafoMat;
//		im = infoMat;

		// pop queues
		{
			boost::mutex::scoped_lock lock(queueRgbMutex); // lock queue
			queueSRgb.pop();
		}
		{
			boost::mutex::scoped_lock lock(queueDepthMutex); // lock queue
			queueSDepth.pop();
		}


		return 1;
	}
	else
		return -1;
}

void Frontend::setCurrentPosition(const Eigen::Isometry3d& pos)
{
	sNode.clear();
	serializeTm(pos, sNode);
	protocolHandler.send(UavComProtocol::CMD1::CURRENT_NODE, sNode);
}

void Frontend::deserializeRgb(const uint8_t* data, int size)
{
	boost::mutex::scoped_lock lock(queueRgbMutex); // lock queue

	for(int i = 0; i < size; ++i)
		queueSRgb.back().push_back(data[i]);

	assert(queueSRgb.back().size() <= protocolHandler.getDynamicLenField());
	if(queueSRgb.back().size() == protocolHandler.getDynamicLenField())
	{
//		std::cout << "Rgb received!" << std::endl;
		queueSRgb.push(std::vector<uint8_t>());
	}
}

void Frontend::deserializeDepth(const uint8_t* data, int size)
{
	boost::mutex::scoped_lock lock(queueDepthMutex); // lock queue
	for(int i = 0; i < size; ++i)
		queueSDepth.back().push_back(data[i]);

	assert(queueSDepth.back().size() <= protocolHandler.getDynamicLenField());
	if(queueSDepth.back().size() == protocolHandler.getDynamicLenField())
	{
//		std::cout << "Depth received!" << std::endl;
		queueSDepth.push(std::vector<uint8_t>());
	}
}


void Frontend::deserializeCurrentNode(const uint8_t* data, int size)
{
	boost::mutex::scoped_lock lock(queuePosMutex); // lock queue
	for(int i = 0; i < size; ++i)
	{
		queueSPos.back().push_back(data[i]); // push to the last node
	}

	assert(queueSPos.back().size() <= protocolHandler.getDynamicLenField());

	if(queueSPos.back().size() == protocolHandler.getDynamicLenField())
	{
		queueSPos.push(std::vector<uint8_t>());
	}
}


void Frontend::deserializeEdge(const uint8_t* data, int size)
{
	boost::mutex::scoped_lock lock(queueEdgeMutex); // lock queue

	for(int i = 0; i < size; ++i)
	{
		if (queueSTimeStamp.back().empty() && i == 0)
		{
			queueToNode.back() = static_cast<int>(data[i]);
			continue;
		}

		if(queueSTimeStamp.back().size() < sizeof(double))
			queueSTimeStamp.back().push_back(data[i]);
		else if(queueSTm.back().size() < rowsTm*colsTm*sizeTypeTm)
			queueSTm.back().push_back(data[i]);
		else
			queueSIm.back().push_back(data[i]);
	}

	assert(queueSTm.back().size()+queueSIm.back().size()+sizeof(double)+1 <= protocolHandler.getDynamicLenField());
	if(queueSTm.back().size()+queueSIm.back().size()+sizeof(double)+1 == protocolHandler.getDynamicLenField())
	{
//		std::cout << "Edge received!" << std::endl;
		queueSTimeStamp.push(std::vector<uint8_t>());
		queueSTm.push(std::vector<uint8_t>());
		queueSIm.push(std::vector<uint8_t>());
		queueToNode.push(-1);
	}
}

boost::shared_ptr<cv::Mat> Frontend::decodeRgb(const std::vector<uint8_t>& out)
{
	boost::shared_ptr<cv::Mat> img(new cv::Mat);
	cv::Mat tmp = cv::imdecode(out, cv::IMREAD_COLOR );
	tmp.copyTo(*img);
	return img;
}

boost::shared_ptr<cv::Mat> Frontend::decodeDepth(const std::vector<uint8_t>& out)
{
	boost::shared_ptr<cv::Mat> img(new cv::Mat);
	cv::Mat tmp = cv::imdecode(out, cv::IMREAD_ANYDEPTH );
	tmp.copyTo(*img);
	return img;
}

void Frontend::serializeTm(const Eigen::Isometry3d& tm, std::vector<uint8_t>& data)
{
	Eigen::Matrix4f tmp = tm.matrix().cast<float>();
	for(int row = 0; row < rowsTm; ++row)
	{
		for(int col = 0; col < colsTm; ++col)
		{
			float val = tmp(row,col);
			uint8_t* p = reinterpret_cast<uint8_t*>(&val);
			for(unsigned int i = 0; i < sizeof(float); ++i, ++p)
				data.push_back(*p);
		}
	}
}

void Frontend::deserializeTimeStamp(const std::vector<uint8_t>& data, double& timeStamp)
{
	uint8_t* p = reinterpret_cast<uint8_t*>(&timeStamp);
	for(unsigned int i = 0; i < sizeof(double); ++i, ++p)
		*p = data[i];
}

void Frontend::deserializeTm(const std::vector<uint8_t>& data, Eigen::Isometry3d& tm)
{
	Eigen::Matrix4f tmp;
	int dataCntr = 0;
	for(int row = 0; row < rowsTm; ++row)
	{
		for(int col = 0; col < colsTm; ++col)
		{
			uint8_t* p = reinterpret_cast<uint8_t*>(&tmp(row,col));
			for(unsigned int i = 0; i < sizeof(float); ++i, ++p)
				*p = data[dataCntr++];
		}
	}

	tm.matrix() = tmp.cast<double>();
}

void Frontend::deserializeIm(const std::vector<uint8_t>& data, Eigen::Matrix<double, 6, 6>& im)
{
	Eigen::Matrix<float, 6, 6> tmp;

	int dataCntr = 0;
	for(int row = 0; row < rowsIm; ++row)
	{
		for(int col = 0; col < colsIm; ++col)
		{
			uint8_t* p = reinterpret_cast<uint8_t*>(&tmp(row,col));
			for(unsigned int i = 0; i < sizeof(float); ++i, ++p)
				*p = data[dataCntr++];
		}
	}

	im = tmp.cast<double>();
}
