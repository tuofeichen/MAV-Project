/**
 * @file UavComProtocolHandler.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the UavComProtocolHandler class.
 *
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

/**
 * @class UavComProtocolHandler UavComProtocolHandler.h "UavComProtocolHandler.h"
 * @brief The UavComProtocolHandler class is used to send and decode data using the UavComProtocol.
 */
class UavComProtocolHandler
{
public:
	/**
	 * @brief Constructor
	 *
	 * @param aLink class which is derived from the ILink interface to send and receive data (input)
	 */
	UavComProtocolHandler(ILink* aLink);

	/**
	 * @brief Destructur
	 */
	virtual ~UavComProtocolHandler();

	/**
	 * @brief sends the given command
	 *
	 * @param cmd command, which specify the data in the payload (input)
	 * @param data payload (input)
	 *
	 */
	void send(CMD1 cmd, const std::vector<uint8_t>& data);

	/**
	 * @brief register a callback function, which should be called when the corresponding command has been received
	 *
	 * @param cmd command, which specify when the callback function should be called (input)
	 * @param callback callback function (input)
	 * @param context object context (input)
	 *
	 */
	void registerRecvCallback(CMD1 cmd, void(*callback)(const uint8_t*, int, void*), void* context);

	/**
	 * @brief decodes the received command. This function should only be used as ILink receive callback.
	 *
	 * @param data pointer to the data (input)
	 * @param len number of bytes in the data array (input)
	 *
	 */
	void decode(const uint8_t* data, int len);

	/**
	 * @brief reads the LEN field of the protocol header
	 *
	 * @return LEN flied
	 */
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
