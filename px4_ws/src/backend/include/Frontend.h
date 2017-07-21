/**
 * @file Frontend.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the Frontend class.
 *
 */

#ifndef INCLUDES_FRONTEND_H_
#define INCLUDES_FRONTEND_H_

#include <queue>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "Eigen/Geometry"

#include "Frame.h"
#include "TcpClient.h"
#include "UavComProtocolHandler.h"

/**
 * @class Frontend Frontend.h "Frontend.h"
 * @brief The Frontend class is used to exchange data with the front end.
 */
class Frontend
{
public:
	/**
	 * @brief Constructor
	 *
	 * @param ip ip address of the front end (input)
	 * @param port port of the fornt end (input)
	 */
	Frontend(const char* ip, int port);

	/**
	 * @brief Destructur
	 */
	virtual ~Frontend();

	/**
	 * @brief check if a new node was send by the front end
	 *
	 * @param frame frame of the new node (output)
	 * @param tm estimated relative transformation by the front end (output)
	 * @param im informtion matrix of the transformation (output)
	 *
	 * @return -1 if no new node is present and 0 otherwise
	 */
	int getNewNode(SLAM::Frame& frame);
//	, Eigen::Isometry3d& tm, Eigen::Matrix<double, 6, 6>& im);

	void getUpdateGraph(std::vector<Eigen::Isometry3d> &tm, int numNode);
	/**
	 * @brief sends the current pose to the front end
	 *
	 * @param pos current pose (input)
	 *
	 */
	void setCurrentPosition(const Eigen::Isometry3d& pos);

	/**
	 * @brief deserializes the RGB image send by the front end
	 *
	 * @param data data to serialize (input)
	 * @param size size of the data (input)
	 *
	 */
	void deserializeRgb(const uint8_t* data, int size);

	/**
	 * @brief deserializes the depth map send by the front end
	 *
	 * @param data data to serialize (input)
	 * @param size size of the data (input)
	 *
	 */
	void deserializeDepth(const uint8_t* data, int size);

	/**
	 * @brief deserializes the pose graph edge send by the front end
	 *
	 * @param data data to serialize (input)
	 * @param size size of the data (input)
	 *
	 */
	void deserializeEdge(const uint8_t* data, int size);
	/**
		 * @brief deserializes the pose graph node send by the front end
		 * @param data data to serialize (input)
		 * @param size size of the data (input)
		 *
	*/

	void deserializeCurrentNode(const uint8_t* data, int size);

	/**
	 * @brief checks if the connection between front end and back end is established
	 *
	 * @return true if connection established, otherwise false
	 */
	volatile bool running() { return client.clientRunning(); } //TODO remove, debugging

private:
	boost::shared_ptr<cv::Mat> decodeRgb(const std::vector<uint8_t>& out);
	boost::shared_ptr<cv::Mat> decodeDepth(const std::vector<uint8_t>& out);

	void serializeTm(const Eigen::Isometry3d& tm, std::vector<uint8_t>& data);
	void deserializeTimeStamp(const std::vector<uint8_t>& data, double& timeStamp);
	void deserializeTm(const std::vector<uint8_t>& data, Eigen::Isometry3d& tm);
	void deserializeIm(const std::vector<uint8_t>& data, Eigen::Matrix<double, 6, 6>& im);

	TcpClient client;
	UavComProtocol::UavComProtocolHandler protocolHandler;

	enum {
		rowsTm = 4, colsTm = 4, sizeTypeTm = sizeof(float),
		rowsIm = 6, colsIm = 6, sizeTypeIm = sizeof(float)
	};

	// receive stuff
    boost::mutex queueRgbMutex;
    boost::mutex queueDepthMutex;
    boost::mutex queueEdgeMutex;
    boost::mutex queuePosMutex;

	std::queue<std::vector<uint8_t>> queueSRgb;
	std::queue<std::vector<uint8_t>> queueSDepth;
	std::queue<std::vector<uint8_t>> queueSTimeStamp;
	std::queue<std::vector<uint8_t>> queueSTm;
	std::queue<std::vector<uint8_t>> queueSIm;
	std::queue<std::vector<uint8_t>> queueSPos;
	std::queue<int> queueToNode;

	boost::shared_ptr<cv::Mat> rgb;
	boost::shared_ptr<cv::Mat> gray;
	boost::shared_ptr<cv::Mat> depth;
	boost::shared_ptr<double> timeStamp;
	Eigen::Isometry3d trafoMat;
	Eigen::Matrix<double,6,6> infoMat;

	// send stuff
	std::vector<uint8_t> sNode;
};

#endif /* INCLUDES_FRONTEND_H_ */
