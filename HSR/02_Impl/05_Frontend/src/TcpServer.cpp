 /**
 * @file TcpServer.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the TcpServer class.
 *
 */

#include <iostream>

#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "TcpServer.h"

using boost::asio::ip::tcp;

TcpServer::TcpServer(int port)
 : acceptor(ioService, tcp::endpoint(tcp::v4(), port)),
   socket(ioService), stopped(true)
{ }

TcpServer::~TcpServer()
{
	ioService.stop();
	socket.close();
}


void TcpServer::listen()
{
	bool error = true;
	while(error)
	{
		error = false;
		ioService.reset();
		socket.close();
		try {
			acceptor.accept(socket); // block until connection established or error occures
		} catch(std::exception& e) {
			error = true;
		}
	}
	socket.set_option(boost::asio::ip::tcp::no_delay(true));
}

void TcpServer::run()
{
	boost::system::error_code error;
	size_t len;

	// start receiving
	stopped = false;
	while(!stopped)
	{
		len = socket.read_some(boost::asio::buffer(data), error);

		if (error)
		{
			if (error == boost::asio::error::eof)
			{ // TODO what to do, when connection closed?
				stop();
				std::cerr << "Error: Connection closed by client!" << std::endl;
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

void TcpServer::stop()
{
	ioService.stop();
	stopped = true;
}

bool TcpServer::send(const std::vector<uint8_t>& data)
{
	if(!stopped)
	{
		boost::system::error_code error;
		size_t bytesToSend = data.size();
		size_t bytes = 0;
		do {
			bytes += boost::asio::write(socket, boost::asio::buffer(&data.data()[bytes],bytesToSend-bytes), boost::asio::transfer_all(), error);
		} while(bytes < bytesToSend && !error);
		if(error)
		{
			if (error == boost::asio::error::eof)
			{ // TODO what to do, when connection closed?
				stop();
				std::cerr << "Error: Connection closed by client!" << std::endl;
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
