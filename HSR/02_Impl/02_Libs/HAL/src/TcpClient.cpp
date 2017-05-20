 /**
 * @file TcpClient.cpp
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the implementation of the TcpClient class
 *
 */
#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#include "TcpClient.h"

using boost::asio::ip::tcp;

namespace HAL {

TcpClient::TcpClient(const char* address, int port)
 : endp(boost::asio::ip::address::from_string(address), port), client(ioService),
   recvCallback(0), callerContext(0)
{ }

TcpClient::~TcpClient() {
	client.close();
}

void TcpClient::run()
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

	 while(!stopTheSocket)
	 {
		 size_t len = client.read_some(boost::asio::buffer(data), error);
		 if (error)
		 {
			 if (error == boost::asio::error::eof)
				 stopTheSocket = true;
			 else
			 {
				 std::cerr << "Error: Reading tcp socket!" << std::endl;
				 boost::this_thread::sleep(boost::posix_time::milliseconds(1));
			 }
		 }
		 else
		 {
			 if (len > 0)
			 {
				if (recvCallback)
				{
					recvCallback(callerContext, data.data(), len);
				}
			 }
			 else
				 boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		 }

	 }
	 client.close();
}

} /* namespace HAL */
