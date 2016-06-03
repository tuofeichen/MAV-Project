 /**
 * @file TcpClient.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the TcpClient class
 *
 */
#ifndef TCPCLIENT_H_
#define TCPCLIENT_H_

#include <boost/array.hpp>
#include <boost/asio.hpp>

namespace HAL {

/**
 * @class TcpClient TcpClient.h "TcpClient.h"
 * @brief The TcpClient class to communicate with the windows 8 kinect sdk
 */
class TcpClient {
public:
	/**
	 * @brief Constructor
	 *
	 * @param address ip address of windows 8 (in)
	 * @param port port (in)
	 */
	TcpClient(const char* address, int port);

	/**
	 * @brief Destructor
	 */
	~TcpClient();

	/**
	 * @brief starts receiveing data
	 * @note blocking call
	 */
	void run();

	/**
	 * @brief stops the client
	 */
	void stop() { stopTheSocket = true; }

	/**
	 * @brief register a receive callback function to handel received data
	 *
	 * pFunCallback callback function (in)
	 * context object context (in)
	 */
	void registerRecvCallback(void(*pFunCallback)(void* context, const uint8_t* data, int size), void* context) { recvCallback = pFunCallback; callerContext = context; }

private:
	boost::asio::io_service ioService;
	boost::asio::ip::tcp::endpoint endp;
	boost::asio::ip::tcp::socket client;

	enum { maxLen = 2048 };
	boost::array<uint8_t, maxLen> data;

	bool stopTheSocket;

	void (*recvCallback)(void* context, const uint8_t* data, int size);
	void* callerContext;
};

} /* namespace HAL */

#endif /* TCPCLIENT_H_ */
