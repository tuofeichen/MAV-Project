/*
 * Backend.h
 *
 *  Created on: May 20, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#ifndef INCLUDES_BACKEND_H_
#define INCLUDES_BACKEND_H_

#include <string>

#include <boost/thread/mutex.hpp>

#include "Frame.h"
#include "TcpServer.h"
#include "UavComProtocolHandler.h"

class Backend
{
public:
	Backend(int port, boost::mutex& mutex);
	virtual ~Backend();

	void setNewNode(const Frame& frame, const Eigen::Matrix4f& tm, const Eigen::Matrix<float, 6, 6>& im, uint8_t toNode); // copy
	const Eigen::Matrix4f& getCurrentPosition() { return currentPosition; }
	volatile bool currentPosUpToDate() { return curPosUpToDateFlag; }
	volatile bool running() { return server.serverRunning(); } //TODO remove, debugging
	void deserializeCurrentNode(const uint8_t* data, int size);

private:
	void encodeImage(const cv::Mat& img, std::vector<uint8_t>& out) const;
	void serializeTimeStamp(const double& timeStamp, std::vector<uint8_t>& data) const;
	void serializeTm(const Eigen::Matrix4f& tm, std::vector<uint8_t>& data) const;
	void deserializeTm(const std::vector<uint8_t>& data, Eigen::Matrix4f& tm) const;
	void serializeIm(const Eigen::Matrix<float, 6, 6>& im, std::vector<uint8_t>& data) const;

	TcpServer server;
	UavComProtocol::UavComProtocolHandler protocolHandler;

	Eigen::Matrix4f currentPosition;
	volatile bool curPosUpToDateFlag = false;

	//
	// send stuff
	enum {
		rowsTm = 4, colsTm = 4, sizeTypeTm = sizeof(float),
		rowsIm = 6, colsIm = 6, sizeTypeIm = sizeof(float)
	};

	std::vector<uint8_t> sEdge;
	std::vector<uint8_t> sRgbImg;
	std::vector<uint8_t> sDepthImg;
	boost::mutex& backendMutex;

	
	// recive stuff
	std::vector<uint8_t> sTm;
};

#endif /* INCLUDES_BACKEND_H_ */
