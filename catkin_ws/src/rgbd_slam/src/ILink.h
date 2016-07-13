/*
 * ILink.h
 *
 *  Created on: May 20, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#ifndef INCLUDES_ILINK_H_
#define INCLUDES_ILINK_H_

#include <stdint.h>
#include <vector>

namespace UavComProtocol
{

class ILink
{
public:
	virtual ~ILink() { }
	virtual bool send(const std::vector<uint8_t>& data) = 0;
	virtual void registerRecvCallback(void(*callback)(const uint8_t* data, int len, void* context), void* context) = 0;
};

}

#endif /* INCLUDES_ILINK_H_ */
