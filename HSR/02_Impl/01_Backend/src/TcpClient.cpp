/**
 * @file TcpClient.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the TcpClient class.
 *
 */

#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#include "TcpClient.h"

using boost::asio::ip::tcp;

TcpClient::TcpClient(const char* address, int port)
 : endp(boost::asio::ip::address::from_string(address), port), client(ioService),
   recvCallback(0), callerContext(0)
{ }

TcpClient::~TcpClient() {
	client.close();
}

void TcpClient::stop()
{
	ioService.stop();
	stopTheSocket = true;
}

void TcpClient::connect()
{
	boost::system::error_code error = boost::asio::error::host_not_found;
	while (error)
	{
		client.close();
		client.connect(endp, error);
		ioService.reset();
	}
	if (error)
	{
		std::cerr << "Error: Client could not connect to server!" << std::endl;
		return;
	}
	client.set_option(boost::asio::ip::tcp::no_delay(true));
}

void TcpClient::run()
{
	boost::system::error_code error;
	size_t len;

	// start receiving
	stopTheSocket = false;
	while(!stopTheSocket)
	{
		len = client.read_some(boost::asio::buffer(data), error);

		if (error)
		{
			if (error == boost::asio::error::eof)
			{ // TODO what to do, when connection closed?
				stop();
				std::cerr << "Error: Connection closed by server!" << std::endl;
			}
			else
			{
				std::cerr << "Error: Reading tcp socket!" << std::endl;
			}
		}
		else if (len > 0)
		{
			if (recvCallback)
			{
				recvCallback(data.data(), len, callerContext);
			}
		}
		else
			boost::this_thread::sleep(boost::posix_time::milliseconds(5));
	}
}

bool TcpClient::send(const std::vector<uint8_t>& data)
{
	if(!stopTheSocket)
	{
		boost::system::error_code error;
		size_t bytesToSend = data.size();
		size_t bytes = 0;
		do {
			bytes += boost::asio::write(client, boost::asio::buffer(&data.data()[bytes],bytesToSend-bytes), boost::asio::transfer_all(), error);
		} while(bytes < bytesToSend && !error);
		if(error)
		{
			if (error == boost::asio::error::eof)
			{ // TODO what to do, when connection closed?
				stop();
				std::cerr << "Error: Connection closed by server!" << std::endl;
			}
			else
			{
				std::cerr << "Error: Sending on tcp socket!" << std::endl;
			}
			return false;
		}
		else
			return true;
	}
		return false;
}
