/**
 * @file TcpClient.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the TcpClient class.
 *
 */

#ifndef TCPCLIENT_H_
#define TCPCLIENT_H_

#include <string>
#include <vector>
#include <array>

#include <boost/asio.hpp>

#include "ILink.h"

/**
 * @class TcpClient TcpClient.h "TcpClient.h"
 * @brief The TcpClient class is dervied from the ILink interface to work with the UavComProtocol.
 */
class TcpClient : public UavComProtocol::ILink
{
public:
	/**
	 * @brief Constructor
	 *
	 * @param ip ip address of the server (input)
	 * @param port port of the server (input)
	 */
	TcpClient(const char* address, int port);

	/**
	 * @brief Destructur
	 */
	~TcpClient();

	/**
	 * @brief Establishes a connection to the server. This function is blocking.
	 */
	void connect();

	/**
	 * @brief stops the client and closes the connection to the server.
	 */
	void stop();

	/**
	 * @brief send send data to the server.
	 *
	 * @param data data to send (input)
	 *
	 * @return true on success, false otherwise
	 */
	bool send(const std::vector<uint8_t>& data); // blocking

	/**
	 * @brief register the receive callback function which should be called when data are received from the server
	 *
	 * @param callback callback function (input)
	 * @param context object context (input)
	 *
	 */
	void registerRecvCallback(void(*callback)(const uint8_t*, int, void*), void* context) { recvCallback = callback; callerContext = context; }

	/**
	 * @brief checks if the connection is established
	 *
	 * @return true if the connection between server and client is established, false otherwise
	 */
	volatile bool clientRunning() { return !stopTheSocket; }

	/**
	 * @brief runs the receive loop
	 * @note this function call is blocking
	 */
	void run();

private:
	boost::asio::io_service ioService;
	boost::asio::ip::tcp::endpoint endp;
	boost::asio::ip::tcp::socket client;

	enum{receiveBufferSize = 65536};
	std::array<uint8_t,receiveBufferSize> data;

	volatile bool stopTheSocket = true;

	void (*recvCallback)(const uint8_t*, int, void*);
	void* callerContext;
};

#endif /* TCPCLIENT_H_ */
