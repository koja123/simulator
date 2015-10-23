// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "kernelaodvglue.h"

namespace ScenSim {

const string KernelAodvProtocol::modelName = "Aodv";

KernelAodvProtocol::KernelAodvProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const unsigned int initInterfaceIndex,
    const InterfaceIdType& initInterfaceId,
    const RandomNumberGeneratorSeedType& nodeSeed)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    networkLayerPtr(initNetworkLayerPtr),
    interfaceIndex(initInterfaceIndex),
    interfaceId(initInterfaceId),
    needNotificationOfDataPacket(true),
    packetsSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            modelName + "_" + initInterfaceId + "_PacketsSent")),
    packetsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            modelName + "_" + initInterfaceId + "_PacketsReceived")),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            modelName + "_" + initInterfaceId + "_BytesSent")),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            modelName + "_" + initInterfaceId + "_BytesReceived"))
{
    (*this).ReadParameterConfiguration(theParameterDatabaseReader, nodeId);

    aodvScheduleTimerPtr =
        new KernelAodvPort::AodvScheduleTimer(
            simulationEngineInterfacePtr,
            this);

    aodvFloodIdPtr =
        new KernelAodvPort::AodvFloodId(
            simulationEngineInterfacePtr,
            this,
            aodvScheduleTimerPtr);

    aodvNeighborPtr =
        new KernelAodvPort::AodvNeighbor(
            simulationEngineInterfacePtr,
            this,
            aodvScheduleTimerPtr);

    aodvRoutingTablePtr =
        new KernelAodvPort::AodvRoutingTable(
            simulationEngineInterfacePtr,
            this,
            aodvNeighborPtr,
            (*this).GetIpAddressLow32Bits());

    aodvHelloPtr =
        new KernelAodvPort::AodvHello(
            simulationEngineInterfacePtr,
            this,
            aodvRoutingTablePtr,
            aodvScheduleTimerPtr);

    (*this).InitializeSelfRoute();

    RandomNumberGenerator aRandomNumberGenerator(
        HashInputsToMakeSeed(nodeSeed, initInterfaceIndex));

    unsigned int helloStartTimeJitterMilliSec = aRandomNumberGenerator.GenerateRandomInt(0, 1000);

    aodvScheduleTimerPtr->insert_timer(
        TASK_HELLO, helloStartTimeJitterMilliSec, (*this).GetIpAddressLow32Bits());

    aodvScheduleTimerPtr->insert_timer(
        TASK_CLEANUP, helloStartTimeJitterMilliSec, (*this).GetIpAddressLow32Bits());
}


void KernelAodvProtocol::ReadParameterConfiguration(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId)
{

    aodvActiveRouteTimeout = theParameterDatabaseReader.ReadTime("aodv-active-route-timeout", nodeId, interfaceId);

    if (aodvActiveRouteTimeout <= ZERO_TIME) {
        cerr << "Error: aodv-active-route-timeout value is invalid."
             << endl;
        exit(1);
    }


    aodvAllowedHelloLoss = theParameterDatabaseReader.ReadInt("aodv-allowed-hello-loss", nodeId, interfaceId);

    if (aodvAllowedHelloLoss < 0) {
        cerr << "Error: aodv-allowed-hello-loss value is invalid."
             << endl;
        exit(1);
    }


    aodvDeletePeriod = aodvActiveRouteTimeout * aodvAllowedHelloLoss;


    aodvHelloInterval = theParameterDatabaseReader.ReadTime("aodv-hello-interval", nodeId, interfaceId);

    if (aodvHelloInterval <= ZERO_TIME) {
        cerr << "Error: aodv-hello-interval value is invalid."
             << endl;
        exit(1);
    }


    aodvMyRouteTimeout = theParameterDatabaseReader.ReadTime("aodv-my-route-timeout", nodeId, interfaceId);

    if (aodvMyRouteTimeout <= ZERO_TIME) {
        cerr << "Error: aodv-my-route-timeout value is invalid."
             << endl;
        exit(1);
    }


    aodvNetDiameter = theParameterDatabaseReader.ReadInt("aodv-net-diameter", nodeId, interfaceId);

    if (aodvNetDiameter > UCHAR_MAX || aodvNetDiameter <= 0) {
        cerr << "Error: aodv-net-diameter value is invalid."
             << endl;
        exit(1);
    }


    TimeType aodvNodeTraversalTime = theParameterDatabaseReader.ReadTime("aodv-node-traversal-time", nodeId, interfaceId);

    if (aodvNodeTraversalTime <= ZERO_TIME) {
        cerr << "Error: aodv-node-traversal-time value is invalid."
             << endl;
        exit(1);
    }


    aodvNetTraversalTime = 2 * aodvNodeTraversalTime * aodvNetDiameter;

    aodvPathDiscoveryTime = 2 * aodvNetTraversalTime;


    aodvRreqRetries = theParameterDatabaseReader.ReadInt("aodv-rreq-retries", nodeId, interfaceId);

    if (aodvRreqRetries > UCHAR_MAX || aodvRreqRetries < 0) {
        cerr << "Error: aodv-rreq-retries value is invalid."
             << endl;
        exit(1);
    }

}


void KernelAodvProtocol::InitializeSelfRoute()
{
    const unsigned int my_address = (*this).GetIpAddressLow32Bits();

    KernelAodvPort::aodv_route *tmp_route = new KernelAodvPort::aodv_route();

    tmp_route = aodvRoutingTablePtr->create_aodv_route(my_address);
    tmp_route->ip = my_address;
    //ScenSim-Port://tmp_route->netmask = calculate_netmask(0); //ifa->ifa_mask;
    tmp_route->self_route = 1;
    tmp_route->seq = 1;
    tmp_route->old_seq = 0;
    tmp_route->rreq_id = 1;
    tmp_route->metric = 0;
    tmp_route->next_hop = tmp_route->ip;
    tmp_route->lifetime = -1;
    tmp_route->route_valid = 1;
    tmp_route->route_seq_valid = 1;
    //ScenSim-Port://tmp_route->dev = dev;

    g_my_route = tmp_route;

}//InitializeSelfRoute//





void KernelAodvProtocol::AddRoutingTableEntry(
    const unsigned int destinationIp,
    const unsigned int nextHopIp,
    const unsigned int netmaskIp,
    char* interf)
{
    const NetworkAddress destinationAddress(destinationIp);
    const NetworkAddress nextHopAddress(nextHopIp);
    const NetworkAddress netmaskAddress(netmaskIp);

    shared_ptr<RoutingTable> routingTablePtr = networkLayerPtr->GetRoutingTableInterface();

    routingTablePtr->AddOrUpdateRoute(
        destinationAddress,
        netmaskAddress,
        nextHopAddress,
        interfaceIndex);

    (*this).OutputTraceAndStatsForAddRoutingTableEntry(
        destinationAddress, netmaskAddress, nextHopAddress);

}//AddRoutingTableEntry//


void KernelAodvProtocol::DeleteRoutingTableEntry(
    const unsigned int destinationIp,
    const unsigned int nextHopIp,
    const unsigned int netmaskIp)
{
    const NetworkAddress destinationAddress(destinationIp);
    const NetworkAddress netmaskAddress(netmaskIp);

    shared_ptr<RoutingTable> routingTablePtr = networkLayerPtr->GetRoutingTableInterface();

    routingTablePtr->DeleteRoute(
        destinationAddress,
        netmaskAddress);

    (*this).OutputTraceAndStatsForDeleteRoutingTableEntry(destinationAddress, netmaskAddress);

}//DeleteRoutingTableEntry//




bool KernelAodvProtocol::SearchEventTicket(
    const unsigned char type,
    const unsigned int id) const
{
    typedef map<IdAndTaskTypeKey, vector<shared_ptr<EventRescheduleTicket> > >::const_iterator IterType;

    IdAndTaskTypeKey key(id, type);

    IterType eventTicketIter = eventTicketStocks.find(key);

    return ((eventTicketIter != eventTicketStocks.end()) && !(eventTicketIter->second.empty()));

}//SearchEventTicket//


void KernelAodvProtocol::CancelReservedEvents(
    const unsigned char type,
    const unsigned int id)
{
    typedef map<IdAndTaskTypeKey, vector<shared_ptr<EventRescheduleTicket> > >::iterator IterType;

    IdAndTaskTypeKey key(id, type);

    IterType eventTicketIter = eventTicketStocks.find(key);

    if (eventTicketIter != eventTicketStocks.end()) {

        for(unsigned int i = 0; i < (eventTicketIter->second).size(); i++) {
            simulationEngineInterfacePtr->CancelEvent(*eventTicketIter->second[i]);
        }//for//

        eventTicketIter->second.clear();
    }//if//

}//CancelReservedEvents//



void KernelAodvProtocol::ProcessTaskForAodvRouting(KernelAodvPort::task* processingTask)
{
    (*this).OutputTraceAndStatsForRunAodvTask(processingTask->type);

    switch (processingTask->type) {
    case TASK_RREQ: {
        KernelAodvPort::recv_rreq(
            simulationEngineInterfacePtr,
            this,
            aodvRoutingTablePtr,
            aodvFloodIdPtr,
            aodvNeighborPtr,
            processingTask);
        //ScenSim-Port://kfree(tmp_task->data);
        break;
    }
    case TASK_RREP: {
        KernelAodvPort::recv_rrep(
            this,
            aodvRoutingTablePtr,
            aodvScheduleTimerPtr,
            aodvNeighborPtr,
            aodvHelloPtr,
            processingTask);
        //ScenSim-Port://kfree(tmp_task->data);
        break;
    }
    case TASK_RERR: {
        KernelAodvPort::recv_rerr(
            this,
            aodvRoutingTablePtr,
            processingTask);
        //ScenSim-Port://kfree(tmp_task->data);
        break;
    }
//ScenSim-Port://    case TASK_RREP_ACK:
//ScenSim-Port://        recv_rrep_ack(tmp_task);
//ScenSim-Port://        //ScenSim-Port://kfree(tmp_task->data);
//ScenSim-Port://        break;
//ScenSim-Port://
    case TASK_RESEND_RREQ: {
        KernelAodvPort::resend_rreq(
            simulationEngineInterfacePtr,
            this,
            aodvRoutingTablePtr,
            aodvScheduleTimerPtr,
            aodvFloodIdPtr,
            processingTask);
        break;
    }
    case TASK_HELLO: {
        aodvHelloPtr->send_hello();
//ScenSim-Port://#ifdef AODV_SIGNAL
//ScenSim-Port://        get_wireless_stats();
//ScenSim-Port://#endif
        break;
    }
    case TASK_NEIGHBOR: {
        aodvNeighborPtr->delete_aodv_neigh(
            aodvRoutingTablePtr,
            processingTask->src_ip);
        break;
    }
    case TASK_CLEANUP: {
        aodvFloodIdPtr->flush_flood_id_queue();
        aodvRoutingTablePtr->flush_aodv_route_table();
        break;
    }
//ScenSim-Port://    case TASK_ROUTE_CLEANUP:
//ScenSim-Port://        flush_aodv_route_table();
//ScenSim-Port://        break;
//ScenSim-Port://
    default:
        cerr << "Error: Processing packet must be an aodv control packet."
             << " Type = " << processingTask->type << " is not known."
             << endl;
        exit(1);
        break;
    }//switch//

}//ProcessTaskForAodvRouting//


void KernelAodvProtocol::ExpireReservedEvents(
    const shared_ptr<EventRescheduleTicket>& eventTicketPtr,
    const unsigned char type,
    const unsigned int id)
{
    //Delete an expired event ticket from eventTicketStocks.

    typedef map<IdAndTaskTypeKey, vector<shared_ptr<EventRescheduleTicket> > >::iterator IterType;

    IdAndTaskTypeKey key(id, type);

    IterType eventTicketIter = eventTicketStocks.find(key);

    if (eventTicketIter != eventTicketStocks.end()) {

        int count = 0;

        for(unsigned int i = 0; i < eventTicketIter->second.size(); i++) {

            if (eventTicketIter->second[i] == eventTicketPtr) {
                eventTicketIter->second[i] = eventTicketIter->second.back();
                count++;
            }//if//
        }//for//

        eventTicketIter->second.pop_back();

        assert(count == 1 && "Error: Non or duplicated ticket is detected.");

    }
    else {

        cerr << "Error: Event tickets must be deleted in eventTicketStocks." << endl;
        exit(1);

    }//if//

}//ExpireReservedEvents//


void KernelAodvProtocol::ProcessWakeupTimerEvent(
    const shared_ptr<EventRescheduleTicket>& eventTicketPtr,
    KernelAodvPort::task* tmp_task)
{
    (*this).ExpireReservedEvents(
        eventTicketPtr,
        static_cast<unsigned char>(tmp_task->type),
        tmp_task->id);

    (*this).ProcessTaskForAodvRouting(tmp_task);

}//ProcessWakeupTimerEvent//


void KernelAodvProtocol::RegisterReservedEvents(
    const shared_ptr<EventRescheduleTicket>& eventTicketPtr,
    const unsigned char type,
    const unsigned int id)
{
    //Stock all event tickets for cancellation.

    typedef map<IdAndTaskTypeKey, vector<shared_ptr<EventRescheduleTicket> > >::iterator IterType;

    IdAndTaskTypeKey key(id, type);

    IterType eventTicketIter = eventTicketStocks.find(key);

    if (eventTicketIter == eventTicketStocks.end()) {
        vector<shared_ptr<EventRescheduleTicket> > eventTicketVector;
        eventTicketVector.push_back(eventTicketPtr);
        eventTicketStocks.insert(make_pair(key, eventTicketVector));
    }
    else {
        (eventTicketIter->second).push_back(eventTicketPtr);
    }//if//

}//RegisterReservedEvents//


void KernelAodvProtocol::ScheduleEventForAodvRouting(KernelAodvPort::task* scheduledTask)
{
    shared_ptr<EventRescheduleTicket> eventTicketPtr =
        shared_ptr<EventRescheduleTicket>(new EventRescheduleTicket());

    shared_ptr<WakeupTimerEvent> wakeupTimerEventPtr =
        shared_ptr<WakeupTimerEvent>(new WakeupTimerEvent(this, scheduledTask));

    simulationEngineInterfacePtr->ScheduleEvent(
        wakeupTimerEventPtr, scheduledTask->time, *eventTicketPtr);

    wakeupTimerEventPtr->HoldAnEventTicket(eventTicketPtr);

    (*this).RegisterReservedEvents(
        eventTicketPtr,
        static_cast<unsigned char>(scheduledTask->type),
        scheduledTask->id);

}//ScheduleEventForAodvRouting//





void KernelAodvProtocol::OutputAodvControlPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destinationAddress,
    const unsigned char hopLimit)
{
    (*this).OutputTraceAndStatsForSendControlPacket(
        *packetPtr, destinationAddress, hopLimit);

    const unsigned char trafficClass = 1;

    IpHeaderModel header(
        trafficClass,
        packetPtr->LengthBytes(),
        hopLimit,
        ScenSim::IP_PROTOCOL_NUMBER_AODV,
        networkLayerPtr->GetNetworkAddress(interfaceIndex),
        destinationAddress);

    packetPtr->AddRawHeader(header.GetPointerToRawBytes(), header.GetNumberOfRawBytes());
    packetPtr->AddTrailingPadding(header.GetNumberOfTrailingBytes());

    networkLayerPtr->ReceiveRoutedNetworkPacketFromRoutingProtocol(
        packetPtr,
        interfaceIndex,
        destinationAddress);

}//OutputAodvControlPacket//


template<typename T> inline
void KernelAodvProtocol::SendBroadcastAodvControlPacket(
    const unsigned char hopLimit,
    const T& data)
{
    assert(hopLimit > 0 && "Error: Hop Limit value must be more than 1.");

    unique_ptr<Packet> packetPtr = Packet::CreatePacket(*simulationEngineInterfacePtr, data);

    (*this).OutputAodvControlPacket(packetPtr, NetworkAddress::broadcastAddress, hopLimit);

}//SendBroadcastAodvControlPacket//


template
void KernelAodvProtocol::SendBroadcastAodvControlPacket<KernelAodvPort::rreq>(
    const unsigned char hopLimit,
    const KernelAodvPort::rreq& data);


template
void KernelAodvProtocol::SendBroadcastAodvControlPacket<KernelAodvPort::rrep>(
    const unsigned char hopLimit,
    const KernelAodvPort::rrep& data);


void KernelAodvProtocol::SendBroadcastAodvControlPacket(
    const unsigned char hopLimit,
    const unsigned char* data,
    const unsigned int dataSizeBytes)
{
    assert(hopLimit > 0 && "Error: Hop Limit value must be more than 1.");

    unique_ptr<Packet> packetPtr = Packet::CreatePacket(
        *simulationEngineInterfacePtr,
        data,
        dataSizeBytes);

    (*this).OutputAodvControlPacket(packetPtr, NetworkAddress::broadcastAddress, hopLimit);

}//SendBroadcastAodvControlPacket//


template<typename T> inline
void KernelAodvProtocol::SendUnicastAodvControlPacket(
    const unsigned int nextHopIp,
    const unsigned char hopLimit,
    const T& data)
{
    assert(hopLimit > 0 && "Error: Hop Limit value must be more than 1.");

    unique_ptr<Packet> packetPtr = Packet::CreatePacket(*simulationEngineInterfacePtr, data);

    const NetworkAddress nextHopAddress(nextHopIp);

    (*this).OutputAodvControlPacket(packetPtr, nextHopAddress, hopLimit);

}//SendUnicastAodvControlPacket//


template
void KernelAodvProtocol::SendUnicastAodvControlPacket<KernelAodvPort::rrep>(
    const unsigned int nextHopIp,
    const unsigned char hopLimit,
    const KernelAodvPort::rrep& data);





void KernelAodvProtocol::ProcessIncomingPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& lastHopAddress,
    const NetworkAddress& destinationAddress,
    const unsigned char hopLimit)
{
    //This packet is for this node.

    const unsigned char aodvControlPacketType =
        packetPtr->GetAndReinterpretPayloadData<unsigned char>();

    if ((aodvControlPacketType != RREQ_MESSAGE) &&
        (aodvControlPacketType != RREP_MESSAGE) &&
        (aodvControlPacketType != RERR_MESSAGE) &&
        (aodvControlPacketType != RREP_ACK_MESSAGE)) {

        cerr << "Error: Packet must be an AODV control packet." << endl;
        exit(1);
    }//if//

    (*this).OutputTraceAndStatsForReceiveControlPacket(
        *packetPtr, destinationAddress, hopLimit);

    KernelAodvPort::sk_buff *skb_p = new KernelAodvPort::sk_buff();
    KernelAodvPort::sk_buff **skb_pp = &skb_p;

    (*skb_pp)->data = packetPtr->GetRawPayloadData();

    const unsigned int lastHopIp = lastHopAddress.GetRawAddressLow32Bits();
    const unsigned int destinationIp = destinationAddress.GetRawAddressLow32Bits();


    KernelAodvPort::input_handler(
        this,
        aodvControlPacketType,
        lastHopIp,
        destinationIp,
        hopLimit,
        skb_pp);

    packetPtr = nullptr;

}//ProcessIncomingPacket//



void KernelAodvProtocol::HandlePacketWithNoRoute(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const NetworkAddress& lastHopAddress,
    bool& wasAccepted)
{
    //This packet is not for this node and came from Upper layer or Lower Layer.

    if (lastHopAddress == NetworkAddress::invalidAddress) {

        //Packet transmitted for myself

        assert(sourceAddress == networkLayerPtr->GetNetworkAddress(interfaceIndex));

        const unsigned int sourceIp = (*this).GetIpAddressLow32Bits();
        const unsigned int destinationIp = destinationAddress.GetRawAddressLow32Bits();

        // RREQ
        KernelAodvPort::output_handler(
            simulationEngineInterfacePtr,
            this,
            aodvRoutingTablePtr,
            aodvScheduleTimerPtr,
            aodvFloodIdPtr,
            sourceIp,
            destinationIp);
    }//if//

    wasAccepted = false;

}//HandlePacketWithNoRoute//

void KernelAodvProtocol::HandlePacketUndeliveredByMac(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destination,
    const unsigned int interfaceIndex,
    const NetworkAddress& nextHop,
    bool& wasAccepted)
{
    wasAccepted = false;

    vector<NetworkAddress> linkBrokenRoutes;

    linkBrokenRoutes.push_back(destination);
    linkBrokenRoutes.push_back(nextHop);

    for(unsigned int i = 0; i < linkBrokenRoutes.size(); i++) {
        const unsigned int destinationIpAddress =
            linkBrokenRoutes[i].GetRawAddressLow32Bits();

        aodvNeighborPtr->delete_aodv_neigh(aodvRoutingTablePtr, destinationIpAddress);
    }
}//HandlePacketUndeliveredByMac//

void KernelAodvProtocol::InspectPacket(
    const NetworkAddress& lastHopAddress,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& nextAddress,
    const NetworkAddress& destinationAddress,
    const bool sourceIsThisNode)
{
    if (sourceIsThisNode) {
        return;
    }

    vector<NetworkAddress> lifetimeUpdateRoutes;

    lifetimeUpdateRoutes.push_back(lastHopAddress);
    lifetimeUpdateRoutes.push_back(sourceAddress);

    const TimeType lifetime =
        simulationEngineInterfacePtr->CurrentTime() + (*this).GetAodvActiveRouteTimeout();

    for(unsigned int i = 0; i < lifetimeUpdateRoutes.size(); i++) {
        const unsigned int destinationIpAddress =
            lifetimeUpdateRoutes[i].GetRawAddressLow32Bits();

        KernelAodvPort::aodv_route* targetRoute =
            aodvRoutingTablePtr->find_aodv_route(destinationIpAddress);

        if ((targetRoute != NULL) && (targetRoute->route_valid)) {
            assert(!targetRoute->self_route);

            targetRoute->lifetime = lifetime;
        }//if//

    }//for//

}//InspectPacket//


void KernelAodvProtocol::ReceivePacketFromNetworkLayer(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress_notused,
    const NetworkAddress& destinationAddress,
    const PacketPriorityType trafficClass_notused,
    const NetworkAddress& lastHopAddress,
    const unsigned char hopLimit,
    const unsigned int interfaceIndex_notused)
{
    (*this).ProcessIncomingPacket(
        packetPtr,
        lastHopAddress,
        destinationAddress,
        hopLimit);

}//ReceivePacketFromNetworkLayer//




}//namespace//
