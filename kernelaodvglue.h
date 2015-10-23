// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef KERNELAODVPROTOCOL_H
#define KERNELAODVPROTOCOL_H

#include "scensim_netsim.h"

#include "aodv.h"
#include "aodv_route.h"
#include "aodv_neigh.h"
#include "packet_in.h"
#include "packet_out.h"
#include "rrep.h"

#include <string>
#include <map>

namespace ScenSim {

using std::string;
using std::map;
using std::pair;


inline
string GetAodvMessageTypeString(const unsigned int messageType)
{
    // Ref. aodv.h

    string outputString;

    switch (messageType) {
    case RREQ_MESSAGE: {
        outputString += "RREQ";
        break;
    }
    case RREP_MESSAGE: {
        outputString += "RREP";
        break;
    }
    case RERR_MESSAGE: {
        outputString += "RERR";
        break;
    }
    case RREP_ACK_MESSAGE: {
        outputString += "RREP_ACK";
        break;
    }
    default:
        outputString += "";
        break;
    }//switch//

    return outputString;

}//GetAodvMessageTypeString//


inline
string GetAodvTaskString(const unsigned int taskType)
{
    // Ref. aodv.h

    string outputString;

    switch (taskType) {
    case TASK_RREQ: {
        outputString += "RREQ";
        break;
    }
    case TASK_RREP: {
        outputString += "RREP";
        break;
    }
    case TASK_RERR: {
        outputString += "RERR";
        break;
    }
    case TASK_RREP_ACK: {
        outputString += "RREP_ACK";
        break;
    }
    case TASK_RESEND_RREQ: {
        outputString += "RESEND_RREQ";
        break;
    }
    case TASK_HELLO: {
        outputString += "HELLO";
        break;
    }
    case TASK_NEIGHBOR: {
        outputString += "NEIGHBOR";
        break;
    }
    case TASK_CLEANUP: {
        outputString += "CLEANUP";
        break;
    }
    case TASK_ROUTE_CLEANUP: {
        outputString += "ROUTE_CLEANUP";
        break;
    }
    default:
        outputString += "";
        break;
    }//switch//

    return outputString;

}//GetAodvTaskString//



class KernelAodvProtocol :
        public OnDemandRoutingProtocolInterface,
        public ProtocolPacketHandler {
public:
    static const string modelName;

    KernelAodvProtocol(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const unsigned int interfaceIndex,
        const InterfaceIdType& interfaceId,
        const RandomNumberGeneratorSeedType& nodeSeed);


    KernelAodvPort::aodv_route* GetMyRoute() { return g_my_route; }
    unsigned int GetIpAddressLow32Bits() const
        { return networkLayerPtr->GetNetworkAddress(interfaceIndex).GetRawAddressLow32Bits(); }


    void AddRoutingTableEntry(
        const unsigned int destinationIp,
        const unsigned int nextHopIp,
        const unsigned int netmaskIp,
        char* interf);
    void DeleteRoutingTableEntry(
        const unsigned int destinationIp,
        const unsigned int nextHopIp,
        const unsigned int netmaskIp);

    bool SearchEventTicket(
        const unsigned char type,
        const unsigned int id) const;
    void CancelReservedEvents(
        const unsigned char type,
        const unsigned int id);
    void ProcessTaskForAodvRouting(KernelAodvPort::task* processingTask);
    void ScheduleEventForAodvRouting(KernelAodvPort::task* scheduledTask);

    template<typename T>
    void SendBroadcastAodvControlPacket(
        const unsigned char hopLimit,
        const T& data);
    void SendBroadcastAodvControlPacket(
        const unsigned char hopLimit,
        const unsigned char* data,
        const unsigned int dataSizeBytes);
    template<typename T>
    void SendUnicastAodvControlPacket(
        const unsigned int nextHopIp,
        const unsigned char hopLimit,
        const T& data);


    // OnDemandRoutingProtocolInterface
    void HandlePacketWithNoRoute(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const NetworkAddress& lastHopAddress,
        bool& wasAccepted);

    void HandlePacketUndeliveredByMac(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destination,
        const unsigned int interfaceIndex,
        const NetworkAddress& nextHop,
        bool& wasAccepted);

    void InspectPacket(
        const NetworkAddress& lastHopAddress,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& nextAddress,
        const NetworkAddress& destinationAddress,
        const bool sourceIsThisNode);
    bool NeedToInspectAllDataPackets() const { return needNotificationOfDataPacket; }

    // ProtocolPacketHandler
    void ReceivePacketFromNetworkLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress_notused,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass_notused,
        const NetworkAddress& lastHopAddress,
        const unsigned char hopLimit,
        const unsigned int interfaceIndex_notused);

    void GetPortNumbersFromPacket(
        const Packet& aPacket,
        const unsigned int transportHeaderOffset,
        bool& portNumbersWereRetrieved,
        unsigned short int& sourcePort,
        unsigned short int& destinationPort) const
    { assert(false && "This function is not currently used for AODV."); }

    //Get Parameter defined in AODV(based on RFC3561 section10).
    TimeType GetAodvActiveRouteTimeout() { return aodvActiveRouteTimeout; }
    TimeType GetAodvHelloInterval() { return aodvHelloInterval; }
    TimeType GetAodvDeletePeriod() { return aodvDeletePeriod; }
    TimeType GetAodvMyRouteTimeout() { return aodvMyRouteTimeout; }
    TimeType GetAodvNetTraversalTime() { return aodvNetTraversalTime; }
    TimeType GetAodvPathDiscoveryTime() { return aodvPathDiscoveryTime; }
    int GetAodvAllowedHelloLoss() { return aodvAllowedHelloLoss; }
    unsigned char GetAodvNetDiameter() { return static_cast<unsigned char>(aodvNetDiameter); }
    unsigned char GetAodvRreqRetries() { return static_cast<unsigned char>(aodvRreqRetries); }


    ~KernelAodvProtocol() { }

private:
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    shared_ptr<NetworkLayer> networkLayerPtr;

    unsigned int interfaceIndex;
    InterfaceIdType interfaceId;

    KernelAodvPort::aodv_route* g_my_route;

    KernelAodvPort::AodvRoutingTable* aodvRoutingTablePtr;
    KernelAodvPort::AodvScheduleTimer* aodvScheduleTimerPtr;
    KernelAodvPort::AodvFloodId* aodvFloodIdPtr;
    KernelAodvPort::AodvNeighbor* aodvNeighborPtr;
    KernelAodvPort::AodvHello* aodvHelloPtr;

    bool needNotificationOfDataPacket;

    typedef pair<unsigned int, unsigned char> IdAndTaskTypeKey;
    map <IdAndTaskTypeKey, vector<shared_ptr<EventRescheduleTicket> > > eventTicketStocks;

    //Parameter defined in AODV(based on RFC3561 section10).
    TimeType aodvActiveRouteTimeout;
    TimeType aodvHelloInterval;
    TimeType aodvDeletePeriod;
    TimeType aodvMyRouteTimeout;
    TimeType aodvNetTraversalTime;
    TimeType aodvPathDiscoveryTime;
    int aodvAllowedHelloLoss;
    int aodvNetDiameter;
    int aodvRreqRetries;


    void ReadParameterConfiguration(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId);

    void InitializeSelfRoute();

    void OutputAodvControlPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destinationAddress,
        const unsigned char hopLimit);

    void ProcessIncomingPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& lastHopAddress,
        const NetworkAddress& destinationAddress,
        const unsigned char hopLimit);

    void ExpireReservedEvents(
        const shared_ptr<EventRescheduleTicket>& eventTicketPtr,
        const unsigned char type,
        const unsigned int id);

    void ProcessWakeupTimerEvent(
        const shared_ptr<EventRescheduleTicket>& eventTicket,
        KernelAodvPort::task* tmp_task);

    void RegisterReservedEvents(
        const shared_ptr<EventRescheduleTicket>& eventTicketPtr,
        const unsigned char type,
        const unsigned int id);

    //--------------------------------------------------------------------------

    class WakeupTimerEvent : public SimulationEvent {
    public:
        WakeupTimerEvent(
            KernelAodvProtocol* initKernelAodvProtocolPtr,
            KernelAodvPort::task* tmp_task)
            :
            kernelAodvProtocolPtr(initKernelAodvProtocolPtr),
            temporaryTask(tmp_task) { }

        void ExecuteEvent() {
            if (eventTicketPtr != nullptr) {
                kernelAodvProtocolPtr->ProcessWakeupTimerEvent(
                    eventTicketPtr,
                    temporaryTask);
            }
            else {
                cerr << "Error: Event is invalid." << endl;
            }//if//
        }

        void HoldAnEventTicket(const shared_ptr<EventRescheduleTicket>& ticketPtr)
            { eventTicketPtr = ticketPtr; }

    private:
        KernelAodvProtocol* kernelAodvProtocolPtr;
        KernelAodvPort::task* temporaryTask;
        shared_ptr<EventRescheduleTicket> eventTicketPtr;

    };//WakeupTimerEvent//



    // Trace and Statistics
    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;


    void OutputTraceAndStatsForRunAodvTask(const int taskType) const;

    void OutputTraceAndStatsForSendControlPacket(
        const Packet& packet,
        const NetworkAddress& destinationAddress,
        const unsigned char hopLimit) const;
    void OutputTraceAndStatsForReceiveControlPacket(
        const Packet& packet,
        const NetworkAddress& destinationAddress,
        const unsigned char hopLimit) const;

    void OutputTraceAndStatsForAddRoutingTableEntry(
        const NetworkAddress& destinationAddress,
        const NetworkAddress& netmaskAddress,
        const NetworkAddress& nextHopAddress) const;
    void OutputTraceAndStatsForDeleteRoutingTableEntry(
        const NetworkAddress& destinationAddress,
        const NetworkAddress& netmaskAddress) const;


    // Disable:

    KernelAodvProtocol(const KernelAodvProtocol&);
    void operator=(const KernelAodvProtocol&);

};//KernelAodvProtocol//



inline
void KernelAodvProtocol::OutputTraceAndStatsForRunAodvTask(const int taskType) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "AodvTask", taskType);
        }
        else {

            ostringstream outStream;
            outStream << "Type= " << GetAodvTaskString(taskType);

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "AodvTask", outStream.str());

        }//if//

    }//if//

}//OutputTraceAndStatsForRunAodvTask//


inline
void KernelAodvProtocol::OutputTraceAndStatsForSendControlPacket(
    const Packet& packet,
    const NetworkAddress& destinationAddress,
    const unsigned char hopLimit) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            AodvSendTraceRecord traceData;

            const PacketIdType& packetId = packet.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            const unsigned char aodvControlPacketType =
                packet.GetAndReinterpretPayloadData<unsigned char>();
            traceData.packetType = aodvControlPacketType;

            if (destinationAddress == NetworkAddress::broadcastAddress) {
                traceData.broadcastPacket = true;
            }
            else {
                traceData.broadcastPacket = false;
            }//if//

            traceData.hopLimit = hopLimit;

            assert(sizeof(traceData) == AODV_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "AodvSend", traceData);
        }
        else {
            const PacketIdType& packetId = packet.GetPacketId();

            ostringstream outStream;
            outStream << "PktId= " << packetId;

            const unsigned char aodvControlPacketType =
                packet.GetAndReinterpretPayloadData<unsigned char>();

            outStream << " Type= " << GetAodvMessageTypeString(aodvControlPacketType);

            if (destinationAddress == NetworkAddress::broadcastAddress) {
                outStream << "-B";
            }
            else {
                outStream << "-U";
            }//if//

            outStream << " HopLimit= " << static_cast<int>(hopLimit);

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "AodvSend", outStream.str());

        }//if//

    }//if//

    packetsSentStatPtr->IncrementCounter();

    const unsigned int packetLengthBytes = packet.LengthBytes();
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForSendControlPacket//


inline
void KernelAodvProtocol::OutputTraceAndStatsForReceiveControlPacket(
    const Packet& packet,
    const NetworkAddress& destinationAddress,
    const unsigned char hopLimit) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            AodvReceiveTraceRecord traceData;

            const PacketIdType& packetId = packet.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            const unsigned char aodvControlPacketType =
                packet.GetAndReinterpretPayloadData<unsigned char>();
            traceData.packetType = aodvControlPacketType;

            if (destinationAddress == NetworkAddress::broadcastAddress) {
                traceData.broadcastPacket = true;
            }
            else {
                traceData.broadcastPacket = false;
            }//if//

            traceData.hopLimit = hopLimit;

            assert(sizeof(traceData) == AODV_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "AodvRecv", traceData);
        }
        else {
            const PacketIdType& packetId = packet.GetPacketId();

            ostringstream outStream;
            outStream << "PktId= " << packetId;

            const unsigned char aodvControlPacketType =
                packet.GetAndReinterpretPayloadData<unsigned char>();

            outStream << " Type= " << GetAodvMessageTypeString(aodvControlPacketType);

            if (destinationAddress == NetworkAddress::broadcastAddress) {
                outStream << "-B";
            }
            else {
                outStream << "-U";
            }//if//

            outStream << " HopLimit= " << static_cast<int>(hopLimit);

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "AodvRecv", outStream.str());

        }//if//

    }//if//

    packetsReceivedStatPtr->IncrementCounter();

    const unsigned int packetLengthBytes = packet.LengthBytes();
    bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForReceiveControlPacket//


inline
void KernelAodvProtocol::OutputTraceAndStatsForAddRoutingTableEntry(
    const NetworkAddress& destinationAddress,
    const NetworkAddress& netmaskAddress,
    const NetworkAddress& nextHopAddress) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            RoutingTableAddEntryTraceRecord traceData;

            traceData.destinationAddress = Ipv6NetworkAddress(destinationAddress);
            traceData.netmaskAddress = Ipv6NetworkAddress(netmaskAddress);
            traceData.nextHopAddress = Ipv6NetworkAddress(nextHopAddress);
            traceData.localAddress =
                Ipv6NetworkAddress(networkLayerPtr->GetNetworkAddress(interfaceIndex));
            traceData.useIpv6 = (NetworkAddress::numberBits == 128);

            assert(sizeof(traceData) == ROUTING_TABLE_ADD_ENTRY_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "AodvAddEntry", traceData);
        }
        else {

            ostringstream outStream;
            outStream
                << "Dest= " << destinationAddress.ConvertToString()
                << " Mask= " << netmaskAddress.ConvertToString()
                << " Next= " << nextHopAddress.ConvertToString()
                << " IF= " << networkLayerPtr->GetNetworkAddress(interfaceIndex).ConvertToString();

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "AodvAddEntry", outStream.str());

        }//if//

    }//if//

}//OutputTraceAndStatsForAddRoutingTableEntry//


inline
void KernelAodvProtocol::OutputTraceAndStatsForDeleteRoutingTableEntry(
    const NetworkAddress& destinationAddress,
    const NetworkAddress& netmaskAddress) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            RoutingTableDeleteEntryTraceRecord traceData;

            traceData.destinationAddress = Ipv6NetworkAddress(destinationAddress);
            traceData.netmaskAddress = Ipv6NetworkAddress(netmaskAddress);
            traceData.useIpv6 = (NetworkAddress::numberBits == 128);

            assert(sizeof(traceData) == ROUTING_TABLE_DELETE_ENTRY_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "AodvDelEntry", traceData);
        }
        else {

            ostringstream outStream;
            outStream
                << "Dest= " << destinationAddress.ConvertToString()
                << " Mask= " << netmaskAddress.ConvertToString();

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "AodvDelEntry", outStream.str());

        }//if//

    }//if//

}//OutputTraceAndStatsForDeleteRoutingTableEntry//


}//namespace//

#endif
