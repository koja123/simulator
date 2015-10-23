// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_TRACEDEF_H
#define SCENSIM_TRACEDEF_H

#include <stdint.h>

#include "scensim_time.h"
#include "scensim_nodeid.h"
#include "scensim_netaddress.h"
#include "scensim_queues.h"

namespace ScenSim {

//mobility

const size_t NODE_POSITION_TRACE_RECORD_BYTES = 40;
struct NodePositionTraceRecord {
    double xPositionMeters;
    double yPositionMeters;
    double theHeightFromGroundMeters;
    double attitudeAzimuthDegrees;
    double attitudeElevationDegrees;
};


//application

const size_t APPLICATION_SEND_TRACE_RECORD_BYTES = 24;
struct ApplicationSendTraceRecord {
    uint32_t packetSequenceNumber;
    NodeIdType sourceNodeId;
    uint64_t sourceNodeSequenceNumber;
    NodeIdType destinationNodeId;
    unsigned char padding[4];
};

const size_t APPLICATION_RECEIVE_TRACE_RECORD_BYTES = 32;
struct ApplicationReceiveTraceRecord {
    uint32_t packetSequenceNumber;
    NodeIdType sourceNodeId;
    uint64_t sourceNodeSequenceNumber;
    TimeType delay;
    uint32_t receivedPackets;
    uint16_t packetLengthBytes; // after format version 3
    unsigned char padding[2];
};

const size_t APPLICATION_BUFFER_TRACE_RECORD_BYTES = 16;
struct ApplicationBufferTraceRecord {
    uint32_t packetSequenceNumber;
    NodeIdType sourceNodeId;
    uint64_t sourceNodeSequenceNumber;
};

const size_t APPLICATION_END_FLOW_TRACE_RECORD_BYTES = 16;
struct ApplicationEndFlowTraceRecord {
    TimeType transmissionDelay;
    uint64_t flowDeliveredBytes;
};

const size_t APPLICATION_RECEIVE_DATA_TRACE_RECORD_BYTES = 8;
struct ApplicationReceiveDataTraceRecord {
    NodeIdType sourceNodeId;
    uint32_t dataLengthBytes;
};

const size_t IPERF_APPLICATION_TCP_START_RECORD_BYTES = 16;
struct IperfTcpApplicationStartRecord {
    uint64_t totalTimeOrBytes;
    uint32_t timeModeEnabled;
    unsigned char padding[4];
};

const size_t IPERF_APPLICATION_TCP_END_RECORD_BYTES = 16;
struct IperfTcpApplicationEndRecord {
    TimeType totalTime;
    uint64_t totalBytes;
};

const size_t IPERF_APPLICATION_UDP_START_RECORD_BYTES = 16;
struct IperfUdpApplicationStartRecord {
    uint64_t totalTimeOrBytes;
    uint32_t timeModeEnabled;
    unsigned char padding[4];
};

const size_t IPERF_APPLICATION_UDP_END_RECORD_BYTES = 40;
struct IperfUdpApplicationEndRecord {
    TimeType totalTime;
    TimeType jitter;
    uint64_t totalBytes;
    uint32_t totalPackets;
    uint32_t errorPackets;
    uint32_t outOfOrderPackets;
    unsigned char padding[4];
};

const size_t FLOODING_APPLICATION_DISCARD_TRACE_RECORD_BYTES = 24;
struct FloodingApplicationDiscardTraceRecord {
    uint32_t packetSequenceNumber;
    NodeIdType sourceNodeId;
    uint64_t sourceNodeSequenceNumber;
    uint16_t packetLengthBytes; // after format version 3
    unsigned char padding[6];
};

inline
string ConvertBundleMessageTypeToString(
    const unsigned char messageType)
{
    string messageTypeString;

    switch (messageType) {
    case 0x00:
        messageTypeString = "HELLO";
        break;
    case 0x01:
        messageTypeString = "REQUEST";
        break;
    case 0x02:
        messageTypeString = "BUNDLE";
        break;
    default:
        assert(false);
        exit(1);
    }//switch//

    return messageTypeString;

}//ConvertMessageTypeToString//


const size_t BUNDLE_CONTROL_SEND_TRACE_RECORD_BYTES = 8;
struct BundleControlSendTraceRecord {
    NodeIdType destinationNodeId;
    unsigned char messageType;
    unsigned char padding[3];
};

const size_t BUNDLE_CONTROL_RECEIVE_TRACE_RECORD_BYTES = 8;
struct BundleControlReceiveTraceRecord {
    NodeIdType sourceNodeId;
    unsigned char messageType;
    unsigned char padding[3];
};

const size_t BUNDLE_GENERATE_TRACE_RECORD_BYTES = 16;
struct BundleGenerateTraceRecord {
    uint32_t bundleSizeBytes;
    NodeIdType originatorNodeId;
    uint32_t originatorSequenceNumber;
    NodeIdType targetNodeId;
};

const size_t BUNDLE_DELIVERY_TRACE_RECORD_BYTES = 24;
struct BundleDeliveryTraceRecord {
    uint32_t bundleSizeBytes;
    NodeIdType originatorNodeId;
    uint32_t originatorSequenceNumber;
    NodeIdType targetNodeId;
    TimeType delay;
};

const size_t BUNDLE_SEND_TRACE_RECORD_BYTES = 24;
struct BundleSendTraceRecord {
    NodeIdType destinationNodeId;
    uint32_t bundleSizeBytes;
    NodeIdType originatorNodeId;
    uint32_t originatorSequenceNumber;
    NodeIdType targetNodeId;
    unsigned char padding[4];
};


const size_t BUNDLE_RECEIV_TRACE_RECORD_BYTES = 24;
struct BundleReceiveTraceRecord {
    NodeIdType sourceNodeId;
    uint32_t bundleSizeBytes;
    NodeIdType originatorNodeId;
    uint32_t originatorSequenceNumber;
    NodeIdType targetNodeId;
    unsigned char padding[4];
};

const size_t SENSING_DETECTION_TRACE_RECORD_BYTES = 8;
struct SensingDetectionTraceRecord {
    uint32_t numberNodeDetections;
    uint32_t numberGisObjectDetections;
};

//transport

const size_t UDP_SEND_TRACE_RECORD_BYTES = 16;
struct UdpSendTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char padding[4];
};

const size_t UDP_RECEIVE_TRACE_RECORD_BYTES = 16;
struct UdpReceiveTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char padding[4];
};

const size_t TCP_DATA_SEND_TRACE_RECORD_BYTES = 16;
struct TcpDataSendTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char padding[4];
};

const size_t TCP_CONTROL_SEND_TRACE_RECORD_BYTES = 16;
struct TcpControlSendTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char padding[4];
};

const size_t TCP_RECEIVE_TRACE_RECORD_BYTES = 16;
struct TcpReceiveTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char padding[4];
};


//network

const size_t IP_SEND_TRACE_RECORD_BYTES = 16;
struct IpSendTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char padding[4];
};

const size_t IP_RECEIVE_TRACE_RECORD_BYTES = 16;
struct IpReceiveTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    uint16_t packetLengthBytes; // after format version 2
    unsigned char padding[2];
};

const size_t IP_ADDRESS_CHANGE_TRACE_RECORD_BYTES = 32;
struct IpAddressChangeTraceRecord {
    Ipv6NetworkAddress ipAddress;
    uint32_t subnetMaskLengthBits;
    uint32_t interfaceIndex;
    bool useIpv6:8;
    unsigned char padding[7];
};

const size_t IP_PACKET_DROP_TRACE_RECORD_BYTES = 32;
struct IpPacketDropTraceRecord {
    Ipv6NetworkAddress destinationAddress;
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    bool useIpv6:8;
    unsigned char padding[3];
};

inline
string ConvertToEnqueueResultString(const EnqueueResultType enqueuResult)
{
    switch (enqueuResult) {
    case ENQUEUE_SUCCESS: return "Sccuess";
    case ENQUEUE_FAILURE_BY_MAX_PACKETS: return "FailureByMaxPackets";
    case ENQUEUE_FAILURE_BY_MAX_BYTES: return "FailureByMaxBytes";
    case ENQUEUE_FAILURE_BY_OUT_OF_SCOPE: return "FailureByOutOfScope";
    default:
        break;
    }//switch//

    return "";
}

const size_t IP_FULL_QUEUE_DROP_TRACE_RECORD_BYTES = 16;
struct IpFullQueueDropTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    uint16_t enqueueResult; // after format version 3
    unsigned char padding[2];
};

const size_t IP_PACKET_UNDELIVERED_TRACE_RECORD_BYTES = 16;
struct IpPacketUndeliveredTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char padding[4];
};


//routing

const size_t ROUTING_TABLE_ADD_ENTRY_TRACE_RECORD_BYTES = 72;
struct RoutingTableAddEntryTraceRecord {
    Ipv6NetworkAddress destinationAddress;
    Ipv6NetworkAddress netmaskAddress;
    Ipv6NetworkAddress nextHopAddress;
    Ipv6NetworkAddress localAddress;
    bool useIpv6:8;
    unsigned char padding[7];
};

const size_t ROUTING_TABLE_DELETE_ENTRY_TRACE_RECORD_BYTES = 40;
struct RoutingTableDeleteEntryTraceRecord {
    Ipv6NetworkAddress destinationAddress;
    Ipv6NetworkAddress netmaskAddress;
    bool useIpv6:8;
    unsigned char padding[7];
};


const size_t AODV_SEND_TRACE_RECORD_BYTES = 16;
struct AodvSendTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char packetType; // const static defined
    bool broadcastPacket:8; //0: broadcast, 1: unicast
    unsigned char hopLimit;
    unsigned char padding[1];
};

const size_t AODV_RECEIVE_TRACE_RECORD_BYTES = 16;
struct AodvReceiveTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char packetType; // const static defined
    bool broadcastPacket:8; //0: broadcast, 1: unicast
    unsigned char hopLimit;
    unsigned char padding[1];
};


const size_t OLSR_SEND_TRACE_RECORD_BYTES = 40;
struct OlsrSendTraceRecord {
    Ipv6NetworkAddress originatorAddress;
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned int originatorSequenceNumber;
    unsigned char messageType;
    unsigned char validityTime;
    unsigned char hopLimit;
    unsigned char hopCount;
    bool useIpv6:8;
    unsigned char padding[3];
};

const size_t OLSR_RECEIVE_TRACE_RECORD_BYTES = 40;
struct OlsrReceiveTraceRecord {
    Ipv6NetworkAddress originatorAddress;
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned int originatorSequenceNumber;
    unsigned char messageType;
    unsigned char validityTime;
    unsigned char hopLimit;
    unsigned char hopCount;
    bool useIpv6:8;
    unsigned char padding[3];
};

const size_t OLSRV2_SEND_TRACE_RECORD_BYTES = 16;
struct Olsrv2SendTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char pad[4];
};

const size_t OLSRV2_RECEIVE_TRACE_RECORD_BYTES = 16;
struct Olsrv2ReceiveTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    unsigned char pad[4];
};


//mac

const size_t ABSTRACT_MAC_PACKET_TRACE_RECORD_BYTES = 16;
struct AbstractMacPacketTraceRecord {
    uint64_t sourceNodeSequenceNumber;
    NodeIdType sourceNodeId;
    uint16_t packetLengthBytes; // after format version 3
    unsigned char padding[2];
};


// others

const size_t GROUP_TRACE_RECORD_BYTES = 40;
struct GroupTraceRecord {
    NodeIdType nodeId;
    char groupId[24];

    int groupEnterState;
    // 0:leave from group
    // 1:enter to group
    // 2:others

    double value;
};

const size_t SECTOR_TRACE_RECORD_BYTES = 40;
struct SectorTraceRecord {
    NodeIdType sourceNodeId;
    char padding[4];

    double radiousMeters;

    double directionDegrees;
    double directionLengthDegrees;

    double value;
};

const size_t STATE_TRACE_RECORD_BYTES = 32;
struct StateTraceRecord {
    TimeType duration;
    uint32_t priority;
    char stateId[24];

    StateTraceRecord() : duration(ZERO_TIME), priority(0) {}
};

const size_t LINK_TRACE_RECORD_BYTES = 16;
struct LinkTraceRecord {
    NodeIdType srcNodeId;
    NodeIdType destNodeId;

    double value;

    LinkTraceRecord() {}
};

const size_t POPULATION_TRACE_RECORD_BYTES = 16;
struct PopulationTraceRecord {
    double populationDensity;
    uint32_t population;
    unsigned char padding[4];

    PopulationTraceRecord() {}

};


}//namespace//


#endif

