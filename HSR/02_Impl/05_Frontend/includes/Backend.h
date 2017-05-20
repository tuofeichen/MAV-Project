 /**
 * @file Backend.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the Backend class
 *
 */
 
#ifndef INCLUDES_BACKEND_H_
#define INCLUDES_BACKEND_H_

#include <string>

#include <boost/thread/mutex.hpp>

#include "Frame.h"
#include "TcpServer.h"
#include "UavComProtocolHandler.h"

/**
 * @class Backend Backend.h "Backend.h"
 * @brief The Backend class is to communicate with the back end
 */
class Backend
{
public:

	/**
	 * @brief Backend Constructor
	 *
	 * @param port communication port
	 * @param mutex semaphore if send function is called in a thread
	 */
	Backend(int port, boost::mutex& mutex);
	
	/**
	 * @breif Backend Destructor
	 */
	virtual ~Backend();

	/**
	 * @brief setNewNode sends the new RGB-D frame and the estimated relative
transformation between the new frame and the latest node in the pose graph of the back end
	 *
	 * @param frame which has to be sent (input)
	 * @param tm transformation matrix (input)
	 * @param im information matrix (input)
	 * @param toNode if 0 the node is a dummy node, otherwise regular frame (input)
	 */
	void setNewNode(const Frame& frame, const Eigen::Matrix4f& tm, const Eigen::Matrix<float, 6, 6>& im, uint8_t toNode); // copy
	
	/**
	 * @brief getCurrentPosition return the current position
	 *
	 * @return Returns current position
	 */
	const Eigen::Matrix4f& getCurrentPosition() { return currentPosition; }
	
	/**
	 * @brief currentPosUpToDate checks if the current position is updated from the back end
	 *
	 * @return Returns true if the current pose is up-to-date
	 */
	volatile bool currentPosUpToDate() { return curPosUpToDateFlag; }

	/**
	 * @brief running checks if the TCP server is still running
	 *
	 * @return Returns true if the TCP server is running
	 */
	volatile bool running() { return server.serverRunning(); } //TODO remove, debugging

	/**
	 * @brief deserializeCurrentNode deserializes the transformation estimation sent from the back end
	 * @note result is stored in currentPosition
	 *
	 * @param data serial data stream (input)
	 * @param size size of this stream (input)
	 *
	 */
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

	//
	// recive stuff
	std::vector<uint8_t> sTm;
};

#endif /* INCLUDES_BACKEND_H_ */
