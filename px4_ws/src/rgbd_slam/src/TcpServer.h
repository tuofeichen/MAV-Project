/*
 * TcpServer.h
 *
 *  Created on: May 19, 2015
 *      Author: user
 */

#ifndef INCLUDES_TCPSERVER_H_
#define INCLUDES_TCPSERVER_H_

#include <string>
#include <array>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "ILink.h"

class TcpServer : public UavComProtocol::ILink
{
public:
	TcpServer(int port);
	~TcpServer();

	void listen(); // establish the connection
	void stop();

	virtual bool send(const std::vector<uint8_t>& data);
	virtual void registerRecvCallback(void(*callback)(const uint8_t* data, int len, void*), void* context) { recvCallback = callback; callerContext = context; }

	volatile bool serverRunning() { return !stopped; }
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
