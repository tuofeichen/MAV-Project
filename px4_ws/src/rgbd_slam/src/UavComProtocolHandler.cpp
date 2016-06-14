/*
 * UavComProtocolHandler.cpp
 *
 *  Created on: May 20, 2015
 *      Author:  Gian Danuf & Michael Eugster
 */

#include <assert.h>

#include "UavComProtocolHandler.h"

namespace UavComProtocol
{

static void linkRecvCallBack(const uint8_t* data, int len, void* context)
{
	reinterpret_cast<UavComProtocolHandler*>(context)->decode(data, len);
}

UavComProtocolHandler::UavComProtocolHandler(ILink* aLink)
 : link(aLink), header(CMD_SIZE_IN_BYTE+IMAGE_LEN_FIELD_SIZE_IN_BYTE), decodeState(sCmd0)
{
	assert(link);
	link->registerRecvCallback(linkRecvCallBack,this);
}

UavComProtocolHandler::~UavComProtocolHandler()
{

}

void UavComProtocolHandler::send(CMD1 cmd, const std::vector<uint8_t>& data)
{
	header.clear();
	switch(cmd)
	{
	case CMD1::RGB:
	case CMD1::DEPTH:
		header.push_back(CMD0::IMAGE);
		header.push_back(cmd);
		for(int i=0; i < IMAGE_LEN_FIELD_SIZE_IN_BYTE; ++i)
			header.push_back( static_cast<uint8_t>(0x000000FF & (data.size() >> (i<<3))));
		link->send(header);
		link->send(data);
		break;
	case CMD1::EDGE:
	case CMD1::CURRENT_NODE:
		header.push_back(CMD0::POSE_GRAPH);
		header.push_back(cmd);
		link->send(header);
		link->send(data);
		break;
	default:
		assert(false);
		break;
	}
}

void UavComProtocolHandler::registerRecvCallback(CMD1 cmd, void (*callback)(const uint8_t*, int, void*), void* context)
{
	associateCmdCallbacks.insert(std::pair<CMD1, int>(cmd,callbacks.size()));
	callbacks.push_back(callback);
	contexts.push_back(context);
}

void UavComProtocolHandler::decode(const uint8_t* data, int len)
{
	//
	// decode cmd
	for(int i=0; i<len; ++i)
	{
		switch(decodeState)
		{
		case sCmd0:
			payloadLenByteCntr = 0;
			decodeCmd0(data[i]);
			break;
		case sImage:
			decodeImage(data[i]);
			break;
		case sPoseGraph:
			decodePoseGraph(data[i]);
			break;
		case sRgb:
			i += decodeType(CMD1::RGB, &data[i], len-i)-1;
			break;
		case sDepth:
			i += decodeType(CMD1::DEPTH, &data[i], len-i)-1;
			break;
		case sEdge:
			i += decodeType(CMD1::EDGE, &data[i], len-i)-1;
			break;
		case sNode:
			i += decodeType(CMD1::CURRENT_NODE, &data[i], len-i)-1;
			break;
		default:
			assert(false); //TODO shut not happen
			decodeState = sCmd0;
			break;
		}
	}

}

void UavComProtocolHandler::decodeCmd0(uint8_t byte)
{
	switch(byte)
	{
	case CMD0::IMAGE:
		decodeState = sImage;
		break;
	case CMD0::POSE_GRAPH:
		decodeState = sPoseGraph;
		break;
	default:
		assert(false);
		decodeState = sCmd0;
		break;
	}
}

void UavComProtocolHandler::decodeImage(uint8_t byte)
{
	switch(byte)
	{
	case CMD1::RGB:
		decodeState = sRgb;
		break;
	case CMD1::DEPTH:
		decodeState = sDepth;
		break;
	default:
		decodeState = sCmd0;
		break;
	}
}

void UavComProtocolHandler::decodePoseGraph(uint8_t byte)
{
	switch(byte)
	{
	case CMD1::EDGE:
		decodeState = sEdge;
		break;
	case CMD1::CURRENT_NODE:
		decodeState = sNode;
		break;
	default:
		decodeState = sCmd0;
		break;
	}
}

int UavComProtocolHandler::decodeType(CMD1 type, const uint8_t* data, int len)
{
	for(int i=0; i < len; ++i)
	{
		if( payloadLenByteCntr < IMAGE_LEN_FIELD_SIZE_IN_BYTE)
		{
			if(type == CMD1::RGB || type == CMD1::DEPTH)
			{
				if(payloadLenByteCntr == 0)
				{
					payloadLen = 0;
					payloadLenLocal = 0;
				}

				payloadLen += static_cast<uint32_t>(data[i]) << (payloadLenByteCntr<<3);
				payloadLenLocal = payloadLen;
				++payloadLenByteCntr;
				continue;
			}
			else
			{
				switch(type)
				{
				case CMD1::EDGE:
					payloadLen = payloadLenLocal = POSE_GRAPH_EDGE_LEN;
					payloadLenByteCntr = IMAGE_LEN_FIELD_SIZE_IN_BYTE;
					break;
				case CMD1::CURRENT_NODE:
					payloadLen = payloadLenLocal = POSE_GRAPH_CURRENT_NODE_LEN;
					payloadLenByteCntr = IMAGE_LEN_FIELD_SIZE_IN_BYTE;
					break;
				default:
					assert(false); //TODO error, should not happen!
					break;
				}
			}
		}

		std::map<CMD1, int>::iterator cbIdx = associateCmdCallbacks.find(type);

		if(static_cast<uint32_t>(len-i) >= payloadLenLocal)
		{
			if(cbIdx != associateCmdCallbacks.end()) callbacks.at(cbIdx->second)(&data[i],payloadLenLocal,contexts.at(cbIdx->second));
			payloadLenByteCntr = 0;
			decodeState = sCmd0;
			len = i+payloadLenLocal;
			break;
		}
		else
		{
			if(cbIdx != associateCmdCallbacks.end()) callbacks.at(cbIdx->second)(&data[i],len-i,contexts.at(cbIdx->second));
			payloadLenLocal -= len-i;
			break;
		}
	}
	return len;
}

}
