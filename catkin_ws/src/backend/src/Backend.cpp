/*
 * Backend.cpp
 *
 *  Created on: May 20, 2015
 *      Author: Gian Danuser & Michael Eugster
 */
#include <iostream> //TODO remove

#include <boost/thread/thread.hpp>

#include "opencv2/imgcodecs.hpp"

#include "Backend.h"

static void deserializeCurrentNodeWrapper(const uint8_t* data, int size, void* context)
{
	reinterpret_cast<Backend*>(context)->deserializeCurrentNode(data, size);
}

Backend::Backend(int port, boost::mutex& mutex)
 : server(port), protocolHandler(&server), sEdge(rowsTm*colsTm*sizeTypeTm + rowsIm*colsIm*sizeTypeIm + sizeof(double) + 1), sRgbImg(Frame::rows*Frame::cols*3), sDepthImg(Frame::rows*Frame::cols*2), backendMutex(mutex), sTm(rowsTm*colsTm*sizeTypeTm)
{
	protocolHandler.registerRecvCallback(UavComProtocol::CMD1::CURRENT_NODE,deserializeCurrentNodeWrapper,this);

	currentPosition = Eigen::Matrix4f::Identity();

	std::cout << "Listen for client..." << std::endl;
	server.listen();
	std::cout << "Connection established!" << std::endl;

	sTm.clear();

	// start receiving
	boost::thread(&TcpServer::run, &server);
}

Backend::~Backend()
{
	server.stop();
}

void Backend::setNewNode(const Frame& frame, const Eigen::Matrix4f& tm, const Eigen::Matrix<float, 6, 6>& im, uint8_t toNode)
{
	boost::mutex::scoped_lock lock(backendMutex);

	curPosUpToDateFlag = false;

	sEdge.clear();
	sEdge.push_back(toNode);
	serializeTimeStamp(frame.getTime(),sEdge);
	serializeTm(tm, sEdge);
	serializeIm(im, sEdge);
	
	protocolHandler.send(UavComProtocol::CMD1::EDGE, sEdge);
//	boost::this_thread::sleep(boost::posix_time::milliseconds(300));

	encodeImage(frame.getDepth(),sDepthImg);
	protocolHandler.send(UavComProtocol::CMD1::DEPTH, sDepthImg);
//	boost::this_thread::sleep(boost::posix_time::milliseconds(300));

	encodeImage(frame.getRgb(),sRgbImg);
	protocolHandler.send(UavComProtocol::CMD1::RGB, sRgbImg);
	
//	boost::this_thread::sleep(boost::posix_time::milliseconds(300));
}

void Backend::deserializeCurrentNode(const uint8_t* data, int size)
{
	for(int i = 0; i < size; ++i)
	{
		sTm.push_back(data[i]);
	}

	assert(sTm.size() <= protocolHandler.getDynamicLenField());
	if(sTm.size() == protocolHandler.getDynamicLenField())
	{
//		std::cout << "Current node received!" << std::endl;
		deserializeTm(sTm, currentPosition);
		curPosUpToDateFlag = true;
		sTm.clear();
	}
}

void Backend::encodeImage(const cv::Mat& img, std::vector<uint8_t>& out) const
{
	std::vector<int> params;
	params.push_back(cv::IMWRITE_PNG_COMPRESSION);
	params.push_back(9);   // that's compression level, 9 == full , 0 == none
	out.clear();
	cv::imencode(".png",img, out, params);
}

void Backend::serializeTimeStamp(const double& timeStamp, std::vector<uint8_t>& data) const
{
	const uint8_t* p = reinterpret_cast<const uint8_t*>(&timeStamp);
	for(unsigned int i = 0; i < sizeof(double); ++i, ++p)
		data.push_back(*p);
}

void Backend::serializeTm(const Eigen::Matrix4f& tm, std::vector<uint8_t>& data) const
{
	for(int row = 0; row < rowsTm; ++row)
	{
		for(int col = 0; col < colsTm; ++col)
		{
			float val = tm(row,col);
			uint8_t* p = reinterpret_cast<uint8_t*>(&val);
			for(unsigned int i = 0; i < sizeof(float); ++i, ++p)
				data.push_back(*p);
		}
	}
}

void Backend::deserializeTm(const std::vector<uint8_t>& data, Eigen::Matrix4f& tm) const
{
	int dataCntr = 0;
	for(int row = 0; row < rowsTm; ++row)
	{
		for(int col = 0; col < colsTm; ++col)
		{
			uint8_t* p = reinterpret_cast<uint8_t*>(&tm(row,col));
			for(unsigned int i = 0; i < sizeof(float); ++i, ++p)
				*p = data[dataCntr++];
		}
	}
}

void Backend::serializeIm(const Eigen::Matrix<float, 6, 6>& im, std::vector<uint8_t>& data) const
{
	for(int row = 0; row < rowsIm; ++row)
	{
		for(int col = 0; col < colsIm; ++col)
		{
			float val = im(row,col);
			uint8_t* p = reinterpret_cast<uint8_t*>(&val);
			for(unsigned int i = 0; i < sizeof(float); ++i, ++p)
				data.push_back(*p);
		}
	}
}
