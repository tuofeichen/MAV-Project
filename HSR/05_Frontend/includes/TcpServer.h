 /**
 * @file TcpServer.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the TcpServer class
 *
 */

#ifndef INCLUDES_TCPSERVER_H_
#define INCLUDES_TCPSERVER_H_

#include <string>
#include <array>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "ILink.h"

/**
 * @class TcpServer TcpServer.h "TcpServer.h"
 * @brief The TcpServer class is the TCP server for the connection between front end and back end
 */
class TcpServer : public UavComProtocol::ILink
{
public:
	/**
	 * @brief TcpServer Constructor
	 *
	 */
	TcpServer(int port);
	
	/**
	 * @breif TcpServer Destructor
	 *
	 */
	~TcpServer();

	/**
	 * @breif listen establishes the connection to the client
	 *
	 */
	void listen(); // establish the connection
	
	/**
	 * @breif stop stops the connection to the client
	 *
	 */
	void stop();

	
	/**
	 * @breif send sends data to the client
	 *
	 * @param data serialized data (input)
	 *
	 * @return returns true if sending was successful, false otherwise
	 *
	 */
	virtual bool send(const std::vector<uint8_t>& data);
	
	/**
	 * @brief registera a callback function, which will be called when data have been received over the link
	 *
	 * @param callback callback function, which will be called (input)
	 * @param context object context (input)
	 *
	 */
	virtual void registerRecvCallback(void(*callback)(const uint8_t* data, int len, void*), void* context) { recvCallback = callback; callerContext = context; }

	/**
	 * @breif serverRunning checks if the server is still running
	 *
	 * @return returns true if the server is running, false otherwise
	 *
	 */
	volatile bool serverRunning() { return !stopped; }
	
	/**
	 * @breif run starts receiving from the client
	 *
	 * @note this funciton call is blocking
	 *
	 */
	void run(); // blocking call, runs receive loop

private:
	boost::asio::io_service ioService;
	boost::asio::ip::tcp::tcp::acceptor acceptor;
	boost::asio::ip::tcp::tcp::socket socket;

	volatile bool stopped;

	enum{receiveBufferSize = 2*65536};
	std::array<uint8_t,receiveBufferSize> data;
	boost::thread handler;


	void (*recvCallback)(const uint8_t* data, int len, void* context);
	void* callerContext;
};

#endif /* INCLUDES_TCPSERVER_H_ */
