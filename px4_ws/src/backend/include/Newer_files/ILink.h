/**
 * @file ILink.h
 * @author Gian Danuser & Michael Eugster
 * @brief This file contains the declaration of the ILink interface.
 *
 */

#ifndef INCLUDES_ILINK_H_
#define INCLUDES_ILINK_H_

#include <stdint.h>
#include <vector>

namespace UavComProtocol
{

/**
 * @class ILink ILink.h "ILink.h"
 * @brief The ILink interface is used by the UavComProtocolHandler to receive and send data.
 */
class ILink
{
public:
	/**
	 * @brief destructor
	 */
	virtual ~ILink() { }

	/**
	 * @brief sends data
	 *
	 * @param data data which will be send over the link (input)
	 *
	 * @return Returns false if sending failed, true otherwise.
	 *
	 */
	virtual bool send(const std::vector<uint8_t>& data) = 0;

	/**
	 * @brief registera a callback function, which will be called when data have been received over the link
	 *
	 * @param callback callback function, which will be called (input)
	 * @param context object context (input)
	 *
	 */
	virtual void registerRecvCallback(void(*callback)(const uint8_t* data, int len, void* context), void* context) = 0;
};

}

#endif /* INCLUDES_ILINK_H_ */
