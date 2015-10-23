// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef ALOHA_TRACEDEF_H
#define ALOHA_TRACEDEF_H

#include <stdint.h>

#include "scensim_time.h"
#include "scensim_nodeid.h"

namespace Aloha {

using ScenSim::NodeIdType;
using ScenSim::TimeType;


//mac

static const size_t ALOHA_MAC_PACKET_DEQUEUE_TRACE_RECORD_BYTES = 16;
struct AlohaMacPacketDequeueTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char padding[4];
};

static const size_t ALOHA_MAC_TX_DATA_TRACE_RECORD_BYTES = 16;
struct AlohaMacTxDataTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    uint32_t retryCount;
};

static const size_t ALOHA_MAC_PACKET_RETRY_EXCEEDED_TRACE_RECORD_BYTES = 16;
struct AlohaMacPacketRetryExceededTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char padding[4];
};


static const size_t ALOHA_MAC_FRAME_RECEIVE_TRACE_RECORD_BYTES = 16;
struct AlohaMacFrameReceiveTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char frameType; // const static defined
    unsigned char padding[3];
};


//phy

static const size_t ALOHA_PHY_TX_START_TRACE_RECORD_BYTES = 40;
struct AlohaPhyTxStartTraceRecord {
    NodeIdType sourceNodeId;
    unsigned char padding[4];
    uint64_t sourceNodeSequenceNumber;
    double txPower;
    uint64_t dataRate;//actual: DatarateBitsPerSecType
    TimeType duration; //long long int

};

static const size_t ALOHA_PHY_SIGNAL_INTERFERENCE_TRACE_RECORD_BYTES = 16;
struct AlohaPhySignalInterferenceTraceRecord {
    NodeIdType sourceNodeId;
    unsigned char padding[4];
    uint64_t sourceNodeSequenceNumber;
};

static const size_t ALOHA_PHY_RX_END_TRACE_RECORD_BYTES = 16;
struct AlohaPhyRxEndTraceRecord {
    NodeIdType sourceNodeId;
    bool error;
    unsigned char padding[2];
    uint64_t sourceNodeSequenceNumber;
};

static const size_t ALOHA_PHY_RX_START_TRACE_RECORD_BYTES = 24;
struct AlohaPhyRxStartTraceRecord {
    NodeIdType sourceNodeId;
    unsigned char padding[4];
    uint64_t sourceNodeSequenceNumber;
    double rxPower;
};


}//namespace//


#endif

