/*
 * UavComProtocolHandler.h
 *
 *  Created on: May 20, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#ifndef INCLUDES_UAVCOMPROTOCOLHANDLER_H_
#define INCLUDES_UAVCOMPROTOCOLHANDLER_H_

#include <vector>
#include <map>
#include <stdint.h>

#include "UavComProtocol.h"
#include "ILink.h"

namespace UavComProtocol
{

class UavComProtocolHandler
{
public:
	UavComProtocolHandler(ILink* aLink);
	virtual ~UavComProtocolHandler();

	void send(CMD1 cmd, const std::vector<uint8_t>& data);
	void registerRecvCallback(CMD1 cmd, void(*callback)(const uint8_t*, int, void*), void* context);
	void decode(const uint8_t* data, int len);

	volatile uint32_t getDynamicLenField() { return payloadLen; };

private:
	std::map<CMD1, int> associateCmdCallbacks;
	std::vector<void(*)(const uint8_t*, int, void*)> callbacks;
	std::vector<void*> contexts;

	ILink* link;

	// codeing stuff
	std::vector<uint8_t> header;

	// decode stuff
	enum DecodeStates {
		sCmd0,
		sImage,
		sPoseGraph,
		sRgb,
		sDepth,
		sEdge,
		sNode
	};
	DecodeStates decodeState = sCmd0;

	void decodeCmd0(uint8_t byte);
	void decodeImage(uint8_t byte);
	void decodePoseGraph(uint8_t byte);
	int decodeType(CMD1 type, const uint8_t* data, int len); // returns number of processed bytes
	volatile uint32_t payloadLen = 0;
	uint32_t payloadLenLocal = 0;
	int payloadLenByteCntr = 0;

};

}

#endif /* INCLUDES_UAVCOMPROTOCOLHANDLER_H_ */
