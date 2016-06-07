/*
 * UavComProtocol.h
 *
 *  Created on: May 20, 2015
 *      Author: Gian Danuser & Michael Eugster
 */

#ifndef UAVCOMPROTOCOL_H_
#define UAVCOMPROTOCOL_H_

namespace UavComProtocol
{
	//
	// commands
	//
	enum CMD0 : uint8_t {
		//
		// CMD 0
		IMAGE = 0x01,
		POSE_GRAPH = 0x02,
//		NAVIGATION = 0x03,
		// TODO add ...
	};


	enum CMD1 : uint8_t {
		//
		// IMAGE CMD 1
		RGB = 0x01,
		DEPTH = 0x02,
		// TODO add ...


		//
		// POSE_GRAPH CMD 1
		EDGE = 0x0A,
		CURRENT_NODE = 0x0B,
		// TODO add ...


		//
		// NAVIGATION CMD 1
//		HOVER = 0x10,
			// TODO add ...
	};


	//
	// defined length
	//
	enum {
		CMD_SIZE_IN_BYTE = 3,

		// Length is dynamic, legth filed is n byte long
		IMAGE_LEN_FIELD_SIZE_IN_BYTE = 4,

		// Leng of the payload
		POSE_GRAPH_EDGE_LEN = 1 + sizeof(double) + 4*4*sizeof(float) + 6*6*sizeof(float),
		POSE_GRAPH_CURRENT_NODE_LEN = 4*4*sizeof(float),
	};
};

#endif /* UAVCOMPROTOCOL_H_ */
