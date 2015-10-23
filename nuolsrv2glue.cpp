// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "nuolsrv2glue.h"
#include "scensim_tracedefs.h"

static ScenSim::NuOLSRv2Protocol* currentNode;

namespace NuOLSRv2Port {

void NuOLSRv2ProtocolScheduleEvent(
    const nu_time_t& time)
{
    return currentNode->ScheduleEvent(time);

}//NuOLSRv2ProtocolScheduleEvent//

double NuOLSRv2ProtocolGenerateRandomDouble()
{
    return currentNode->GenerateRandomDouble();

}//NuOLSRv2ProtocolGenerateRandomDouble//

nu_bool_t tuple_i_configure(
    tuple_i_t* localInterfaceTuplePtr)
{
    return localInterfaceTuplePtr->configured = true;

}//tuple_i_configure//


nu_bool_t tuple_i_restore_settings(
    tuple_i_t* localInterfaceTuplePtr)
{
    return true;

}//tuple_i_restore_settings//


void olsrv2_send_packets()
{
    FOREACH_I(localInterfaceTuplePtr) {

        nu_obuf_t* nuObufPtr;

        while ((nuObufPtr = tuple_i_build_packet(localInterfaceTuplePtr)) != nullptr) {
            currentNode->SendPacket(nuObufPtr, localInterfaceTuplePtr->iface_index);
            nu_obuf_free(nuObufPtr);
        }//while//

    }//FOREACH_I//

}//olsrv2_send_packets//


nu_link_metric_t olsrv2_get_link_metric(
    tuple_i_t* localInterfaceTuplePtr,
    nu_ip_t sourceNuIp)
{
    return UNDEF_METRIC;

}//olsrv2_get_link_metric//


nu_bool_t nu_route_add(
    const nu_ip_t destinationNuIp,
    const nu_ip_t nextHopNuIp,
    const nu_ip_t localNuIp,
    const uint8_t hopCount,
    const char* interfaceName)
{
    return currentNode->AddRoute(
        destinationNuIp,
        nextHopNuIp,
        localNuIp,
        hopCount,
        interfaceName);

}//nu_route_add//


nu_bool_t nu_route_delete(
    const nu_ip_t destinationNuIp,
    const nu_ip_t nextHopNuIp,
    const nu_ip_t localNuIp,
    const uint8_t hopCount,
    const char* interfaceName)
{
    return currentNode->DeleteRoute(
        destinationNuIp,
        nextHopNuIp,
        localNuIp,
        hopCount,
        interfaceName);

}//nu_route_delete//

}//namespace//

namespace ScenSim {

using NuOLSRv2Port::current_core;
using NuOLSRv2Port::current_olsrv2;
using NuOLSRv2Port::current_packet;
using NuOLSRv2Port::ibase_al_add;
using NuOLSRv2Port::ibase_i_add;
using NuOLSRv2Port::ibase_i_search_name;
using NuOLSRv2Port::ibase_i_search_index;
using NuOLSRv2Port::nu_bool_t;
using NuOLSRv2Port::nu_ibuf_create;
using NuOLSRv2Port::nu_ibuf_t;
using NuOLSRv2Port::nu_ip_prefix;
using NuOLSRv2Port::nu_ip_set_default_prefix;
using NuOLSRv2Port::nu_ip_set_prefix;
using NuOLSRv2Port::nu_ip_t;
using NuOLSRv2Port::nu_obuf_t;
using NuOLSRv2Port::nu_scheduler_exec_events;
using NuOLSRv2Port::nu_set_af;
using NuOLSRv2Port::nu_time_t;
using NuOLSRv2Port::olsrv2_create;
using NuOLSRv2Port::olsrv2_init_events;
using NuOLSRv2Port::olsrv2_metric_list_load;
using NuOLSRv2Port::olsrv2_set_originator;
using NuOLSRv2Port::tuple_i_t;

const string NuOLSRv2Protocol::modelName = "Olsrv2";

NuOLSRv2Protocol::NuOLSRv2Protocol(
    const NodeIdType& initNodeId,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const RandomNumberGeneratorSeedType& initNodeSeed)
    :
    nodeId(initNodeId),
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    networkLayerPtr(initNetworkLayerPtr),
    broadcastPriority(0),
    aRandomNumberGenerator(HashInputsToMakeSeed(initNodeSeed, SEED_HASH)),
    isParameterSet(false),
    startHello(ZERO_TIME),
    startTc(ZERO_TIME),
    metricListFile(),
    attachedNetworkListStr(),
    attachedNetworkMaskListStr(),
    attachedNetworkDistanceListStr(),
    nuOLSRv2Stats(),
    nuOLSRv2Ptr(nullptr)
{
}//NuOLSRv2Protocol//


NuOLSRv2Protocol::~NuOLSRv2Protocol()
{
    if (nuOLSRv2Ptr != nullptr) {
        SetCurrentNode();
        olsrv2_free(nuOLSRv2Ptr);
        nuOLSRv2Ptr = nullptr;
    }//if//

}//~NuOLSRv2Protocol//


void NuOLSRv2Protocol::CompleteInitialization(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    currentNode = this;

    nuOLSRv2Ptr = olsrv2_create();

    SetCurrentNode();

#ifdef USE_IPV6
    nu_set_af(AF_INET6);
#else
    nu_set_af(AF_INET);
#endif

    nu_logger_set_output_prio(NU_LOGGER, NU_LOGGER_ERR);

}//CompleteInitialization//


void NuOLSRv2Protocol::SetInterfaceId(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceIdType& interfaceId)
{
    SetCurrentNode();

    ReadParameterConfiguration(theParameterDatabaseReader, interfaceId);

    const unsigned int interfaceIndex =
        networkLayerPtr->LookupInterfaceIndex(interfaceId);

    const NetworkAddress ipAddress =
        networkLayerPtr->GetNetworkAddress(interfaceIndex);

    tuple_i_t* localInterfaceTuplePtr = ibase_i_add(interfaceId.c_str());

    localInterfaceTuplePtr->iface_index = interfaceIndex;

    nu_ip_t nuIp = NetworkAddressToNuIp(ipAddress);
    nu_ip_set_default_prefix(&nuIp);
    tuple_i_add_ip(localInterfaceTuplePtr, nuIp);

    localInterfaceTuplePtr->configured = true;

    nuOLSRv2Stats[interfaceIndex].packetsSentStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            modelName + "_" + interfaceId + "_PacketsSent");
    nuOLSRv2Stats[interfaceIndex].packetsReceivedStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            modelName + "_" + interfaceId + "_PacketsReceived");
    nuOLSRv2Stats[interfaceIndex].bytesSentStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            modelName + "_" + interfaceId + "_BytesSent");
    nuOLSRv2Stats[interfaceIndex].bytesReceivedStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            modelName + "_" + interfaceId + "_BytesReceived");

}//SetInterfaceId//


void NuOLSRv2Protocol::Start()
{
    SetCurrentNode();

    olsrv2_init_events();

}//Start//


void NuOLSRv2Protocol::SendPacket(
    const nu_obuf_t* nuObufPtr,
    const unsigned int interfaceIndex)
{
    vector<unsigned char> payload(nuObufPtr->len);

    for (size_t i = 0; i < nuObufPtr->len; i++) {
        payload[i] = nuObufPtr->data[i];
    }//for//

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, payload);

    const InterfaceIdType interfaceId =
        networkLayerPtr->GetInterfaceId(interfaceIndex);

    OutputTraceAndStatsForSendPacket(*packetPtr, interfaceIndex, interfaceId);

    networkLayerPtr->ReceiveOutgoingBroadcastPacket(
        packetPtr,
        interfaceIndex,
        broadcastPriority,
        IP_PROTOCOL_NUMBER_OLSRV2);

}//SendPacket//


nu_bool_t NuOLSRv2Protocol::AddRoute(
    const nu_ip_t destinationNuIp,
    const nu_ip_t nextHopNuIp,
    const nu_ip_t localNuIp,
    const uint8_t hopCount,
    const char* interfaceName)
{
    const NetworkAddress destinationAddress =
        NuIpToNetworkAddress(destinationNuIp);

    const unsigned int numberPrefixbits =
        nu_ip_prefix(destinationNuIp);

    const NetworkAddress maskAddress =
        NetworkAddress::MakeSubnetMask(numberPrefixbits);

    const NetworkAddress nextHopAddress =
        NuIpToNetworkAddress(nextHopNuIp);

    const string interfaceId(interfaceName);
    const unsigned int interfaceIndex =
        networkLayerPtr->LookupInterfaceIndex(interfaceId);

    OutputTraceAndStatsForAddRoute(
        destinationAddress,
        maskAddress,
        nextHopAddress,
        interfaceIndex,
        interfaceId);

    shared_ptr<RoutingTable> routingTable =
        networkLayerPtr->GetRoutingTableInterface();

    routingTable->AddOrUpdateRoute(
        destinationAddress,
        maskAddress,
        nextHopAddress,
        interfaceIndex);

    return true;

}//AddRoute//


nu_bool_t NuOLSRv2Protocol::DeleteRoute(
    const nu_ip_t destinationNuIp,
    const nu_ip_t nextHopNuIp,
    const nu_ip_t localNuIp,
    const uint8_t hopCount,
    const char* interfaceName)
{
    const NetworkAddress destinationAddress =
        NuIpToNetworkAddress(destinationNuIp);

    const unsigned int numberPrefixbits =
        nu_ip_prefix(destinationNuIp);

    const NetworkAddress maskAddress =
        NetworkAddress::MakeSubnetMask(numberPrefixbits);

    const string interfaceId(interfaceName);

    OutputTraceAndStatsForDeleteRoute(
        destinationAddress,
        maskAddress,
        interfaceId);

    shared_ptr<RoutingTable> routingTable =
        networkLayerPtr->GetRoutingTableInterface();

    routingTable->DeleteRoute(
        destinationAddress,
        maskAddress);

    return true;

}//DeleteRoute//


void NuOLSRv2Protocol::ReceivePacketFromNetworkLayer(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress_notused,
    const PacketPriorityType trafficClass_notused,
    const NetworkAddress& lastHopAddress_notused,
    const unsigned char hopLimit_notused,
    const unsigned int interfaceIndex)
{
    SetCurrentNode();

    const InterfaceIdType interfaceId =
        networkLayerPtr->GetInterfaceId(interfaceIndex);

    tuple_i_t* localInterfaceTuplePtr = ibase_i_search_name(interfaceId.c_str());

    if (localInterfaceTuplePtr != nullptr) {

        nu_ibuf_t* nuIbufPtr = nu_ibuf_create(packetPtr->GetRawPayloadData(), packetPtr->LengthBytes());
        nuIbufPtr->src_ip = NetworkAddressToNuIp(sourceAddress);
        nu_ip_set_default_prefix(&nuIbufPtr->src_ip);

        tuple_i_recvq_enq(localInterfaceTuplePtr, nuIbufPtr);

        OutputTraceAndStatsForReceivePacket(*packetPtr, interfaceIndex, interfaceId);
    }//if//

    packetPtr = nullptr;

}//ReceivePacketFromNetworkLayer//


void NuOLSRv2Protocol::GetPortNumbersFromPacket(
    const Packet& aPacket,
    const unsigned int transportHeaderOffset,
    bool& portNumbersWereRetrieved,
    unsigned short int& sourcePort,
    unsigned short int& destinationPort) const
{
    portNumbersWereRetrieved = false;

}//GetPortNumbersFromPacket//


void NuOLSRv2Protocol::NotifyNetworkAddressIsChanged(
    const unsigned int interfaceIndex,
    const NetworkAddress& newInterfaceAddress,
    const unsigned int subnetMaskLengthBits)
{
    SetCurrentNode();

    if (newInterfaceAddress != NetworkAddress::invalidAddress) {

        tuple_i_t* localInterfaceTuplePtr = ibase_i_search_index(interfaceIndex);
        assert(localInterfaceTuplePtr != nullptr);

        nu_ip_set_remove_head(&localInterfaceTuplePtr->local_ip_list);

        nu_ip_t nuIp = NetworkAddressToNuIp(newInterfaceAddress);
        nu_ip_set_default_prefix(&nuIp);
        tuple_i_add_ip(localInterfaceTuplePtr, nuIp);

        localInterfaceTuplePtr->configured = true;

        olsrv2_set_originator();
    }//if//

}//NotifyNetworkAddressIsChanged//


void NuOLSRv2Protocol::ScheduleEvent(
    const nu_time_t& time)
{
    const ScenSim::TimeType eventTime =
        time.tv_sec * SECOND + time.tv_usec * MICRO_SECOND;

    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new NuOLSRv2ProcessEvent(shared_from_this())), eventTime);

}//ScheduleEvent//


void NuOLSRv2Protocol::SetAttachedNetwork()
{
    if (!attachedNetworkListStr.empty()) {
        bool success;

        vector<string> attachedNetworkList;
        ConvertAStringIntoAVectorOfStrings(
            attachedNetworkListStr, success, attachedNetworkList);

        if (!success) {
            cerr << "Error: olsrv2-attached-network-address-list("
                 << attachedNetworkListStr << ") is invalid format." << endl;
            exit(1);
        }//if//

        vector<uint8_t> attachedNetworkMaskList;
        ConvertAStringSequenceOfANumericTypeIntoAVector(
            attachedNetworkMaskListStr, success, attachedNetworkMaskList);

        if (!success) {
            cerr << "Error: olsrv2-attached-network-mask-list("
                 << attachedNetworkMaskListStr << ") is invalid format." << endl;
            exit(1);
        }//if//

        vector<int> attachedNetworkDistanceList;
        ConvertAStringSequenceOfANumericTypeIntoAVector(
            attachedNetworkDistanceListStr, success, attachedNetworkDistanceList);

        if (!success) {
            cerr << "Error: olsrv2-attached-network-distance-list("
                 << attachedNetworkDistanceListStr << ") is invalid format." << endl;
            exit(1);
        }//if//

        if (attachedNetworkList.size() != attachedNetworkMaskList.size()) {
            cerr << "Error: A size of olsrv2-attached-network-address-list("
                 << attachedNetworkList.size() << ") must be equal to a size of "
                 << "olsrv2-attached-network-mask-list("
                 << attachedNetworkMaskList.size() << ")." << endl;
            exit(1);
        }//if//

        if (attachedNetworkList.size() != attachedNetworkDistanceList.size()) {
            cerr << "Error: A size of olsrv2-attached-network-address-list("
                 << attachedNetworkList.size() << ") must be equal to a size of "
                 << "olsrv2-attached-network-distance-list("
                 << attachedNetworkDistanceList.size() << ")." << endl;
            exit(1);
        }//if//

        vector<string>::const_iterator attachedNetworkListIter = attachedNetworkList.begin();
        vector<uint8_t>::const_iterator attachedNetworkMaskListIter = attachedNetworkMaskList.begin();
        vector<int>::const_iterator attachedNetworkDistanceListIter = attachedNetworkDistanceList.begin();

        while (attachedNetworkListIter != attachedNetworkList.end()) {
            const string attachedNetworkStr = *attachedNetworkListIter;
            const uint8_t attachedNetworkMask = *attachedNetworkMaskListIter;
            const int attachedNetworkDistance = *attachedNetworkDistanceListIter;

            NetworkAddress subnetAddress;
            subnetAddress.SetAddressFromString(attachedNetworkStr, success);
            if (!success) {
                cerr << "Error: olsrv2-attached-network-address-list("
                     << attachedNetworkListStr << ") is invalid format." << endl;
                exit(1);
            }//if//
            nu_ip_t subnetNuIp = NetworkAddressToNuIp(subnetAddress);

            nu_ip_set_prefix(&subnetNuIp, attachedNetworkMask);
            ibase_al_add(subnetNuIp, attachedNetworkDistance);

            ++attachedNetworkListIter;
            ++attachedNetworkMaskListIter;
            ++attachedNetworkDistanceListIter;
        }//while//
    }//if//

}//SetAttachedNetwork//


void NuOLSRv2Protocol::ReadParameterConfiguration(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceIdType& interfaceId)
{
    if (theParameterDatabaseReader.ParameterExists("olsrv2-hello-interval", nodeId, interfaceId)) {
        const TimeType helloInterval =
            theParameterDatabaseReader.ReadTime("olsrv2-hello-interval", nodeId, interfaceId);
        if (helloInterval <= ZERO_TIME) {
            cerr
                << "Error: olsrv2-hello-interval("
                << helloInterval << ") should be larger than "
                << ZERO_TIME << "." << endl;
            exit(1);
        }//if//
        const double helloIntervalDouble = ConvertTimeToDoubleSecs(helloInterval);
        if (isParameterSet && (GLOBAL_HELLO_INTERVAL != helloIntervalDouble)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        GLOBAL_HELLO_INTERVAL = helloIntervalDouble;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-hello-max-jitter", nodeId, interfaceId)) {
        const TimeType hpMaxJitter =
            theParameterDatabaseReader.ReadTime("olsrv2-hello-max-jitter", nodeId, interfaceId);
        if (hpMaxJitter < ZERO_TIME) {
            cerr
                << "Error: olsrv2-hello-max-jitter("
                << hpMaxJitter << ") should be "
                << ZERO_TIME << " or larger." << endl;
            exit(1);
        }//if//
        const double hpMaxJitterDouble = ConvertTimeToDoubleSecs(hpMaxJitter);
        if (isParameterSet && (GLOBAL_HP_MAXJITTER != hpMaxJitterDouble)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        GLOBAL_HP_MAXJITTER = hpMaxJitterDouble;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-hello-start-time", nodeId, interfaceId)) {
        const TimeType newStartHello =
            theParameterDatabaseReader.ReadTime("olsrv2-hello-start-time", nodeId, interfaceId);
        if (newStartHello < ZERO_TIME) {
            cerr
                << "Error: olsrv2-hello-start-time("
                << newStartHello << ") should be "
                << ZERO_TIME << " or larger." << endl;
            exit(1);
        }//if//
        if (isParameterSet && (startHello != newStartHello)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        startHello = newStartHello;
        START_HELLO = ConvertTimeToDoubleSecs(startHello);
    }//if//

    START_HELLO += aRandomNumberGenerator.GenerateRandomDouble() * GLOBAL_HP_MAXJITTER;

    if (theParameterDatabaseReader.ParameterExists("olsrv2-link-quality-type", nodeId, interfaceId)) {
        string linkQualityType =
            theParameterDatabaseReader.ReadString("olsrv2-link-quality-type", nodeId, interfaceId);
        ConvertStringToLowerCase(linkQualityType);
        if (linkQualityType == "no") {
            if (isParameterSet && (LQ_TYPE != LQ__NO)) {
                cerr
                    << "Error: nuOLSRv2 doesn't support that"
                    << " each interface on same node has different parameters" << endl;
                exit(1);
            }//if//
            LQ_TYPE = LQ__NO;
        }
        else if (linkQualityType == "hello") {
            if (isParameterSet && (LQ_TYPE != LQ__HELLO)) {
                cerr
                    << "Error: nuOLSRv2 doesn't support that"
                    << " each interface on same node has different parameters" << endl;
                exit(1);
            }//if//
            LQ_TYPE = LQ__HELLO;
        }
        else {
            cerr
                << "Error: Unknown olsrv2-link-quality-type(" << linkQualityType << ")."
                << " olsrv2-link-quality-type should be no or hello." << endl;
            exit(1);
        }//if//
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-lq-hyst-accept", nodeId, interfaceId)) {
        const double hystAccept =
            theParameterDatabaseReader.ReadDouble("olsrv2-lq-hyst-accept", nodeId, interfaceId);
        if (hystAccept < 0.0 || 1.0 < hystAccept) {
            cerr
                << "Error: olsrv2-lq-hyst-accept("
                << hystAccept << ") should be between 0.0 and 1.0." << endl;
            exit(1);
        }//if//
        if (isParameterSet && (HYST_ACCEPT != hystAccept)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        HYST_ACCEPT = hystAccept;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-lq-hyst-reject", nodeId, interfaceId)) {
        const double hystReject =
            theParameterDatabaseReader.ReadDouble("olsrv2-lq-hyst-reject", nodeId, interfaceId);
        if (hystReject < 0.0 || 1.0 < hystReject) {
            cerr
                << "Error: olsrv2-lq-hyst-reject("
                << hystReject << ") should be between 0.0 and 1.0." << endl;
            exit(1);
        }//if//
        if (isParameterSet && (HYST_REJECT != hystReject)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        HYST_REJECT = hystReject;
    }//if//

    if (HYST_ACCEPT < HYST_REJECT) {
        cerr
            << "Error: olsrv2-lq-hyst-accept("
            << HYST_ACCEPT << ") should be OLSRV2-HYST-REJECT("
            << HYST_REJECT << ") or larger." << endl;
        exit(1);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-lq-initial-quality", nodeId, interfaceId)) {
        const double initialQuality =
            theParameterDatabaseReader.ReadDouble("olsrv2-lq-initial-quality", nodeId, interfaceId);
        if (initialQuality < 0.0 || 1.0 < initialQuality) {
            cerr
                << "Error: olsrv2-lq-initial-quality("
                << initialQuality << ") should be between 0.0 and 1.0." << endl;
            exit(1);
        }//if//
        if (isParameterSet && (INITIAL_QUALITY != initialQuality)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        INITIAL_QUALITY = initialQuality;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-lq-initial-pending", nodeId, interfaceId)) {
        const bool initialPending =
            theParameterDatabaseReader.ReadBool("olsrv2-lq-initial-pending", nodeId, interfaceId);
        if (isParameterSet && ((INITIAL_PENDING && !initialPending) || (!INITIAL_PENDING && initialPending))) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        INITIAL_PENDING = initialPending;
    }//if//

    if (INITIAL_PENDING) {
        if (HYST_ACCEPT <= INITIAL_QUALITY) {
            cerr
                << "Error: If olsrv2-lq-initial-pending("
                << (INITIAL_PENDING != false) << ") is true, olsrv2-lq-initial-quality("
                << INITIAL_QUALITY << ") should be less than olsrv2-lq-hyst-accept("
                << HYST_ACCEPT << ")." << endl;
            exit(1);
        }
        else if (LQ_TYPE == LQ__NO) {
            cerr
                << "Error: If olsrv2-lq-initial-pending("
                << (INITIAL_PENDING != false) << ") is true and "
                << "olsrv2-link-quality-type(no) is no, olsrv2-lq-initial-quality("
                << INITIAL_QUALITY << ") should be olsrv2-lq-hyst-accept("
                << HYST_ACCEPT << ") or larger." << endl;
            exit(1);
        }//if//
    }
    else {
        if (INITIAL_QUALITY < HYST_REJECT) {
            cerr
                << "Error: If olsrv2-lq-initial-pending("
                << (INITIAL_PENDING != false) << ") is false, olsrv2-lq-initial-quality("
                << INITIAL_QUALITY << ") should be olsrv2-lq-hyst-reject("
                << HYST_REJECT << ") or larger." << endl;
            exit(1);
        }//if//
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-lq-hyst-scale", nodeId, interfaceId)) {
        const double hystScale =
            theParameterDatabaseReader.ReadDouble("olsrv2-lq-hyst-scale", nodeId, interfaceId);
        if (hystScale < 0.0 || 1.0 < hystScale) {
            cerr
                << "Error: olsrv2-lq-hyst-scale("
                << hystScale << ") should be between 0.0 and 1.0." << endl;
            exit(1);
        }//if//
        if (isParameterSet && (HYST_SCALE != hystScale)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        HYST_SCALE = hystScale;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-lq-loss-detect-scale", nodeId, interfaceId)) {
        const double lossDetectScale =
            theParameterDatabaseReader.ReadDouble("olsrv2-lq-loss-detect-scale", nodeId, interfaceId);
        if (lossDetectScale <= 0.0) {
            cerr
                << "Error: olsrv2-lq-loss-detect-scale("
                << lossDetectScale << ") should be larger than 0.0." << endl;
            exit(1);
        }//if//
        if (isParameterSet && (LOSS_DETECT_SCALE != lossDetectScale)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        LOSS_DETECT_SCALE = lossDetectScale;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-link-metric-type", nodeId, interfaceId)) {
        string linkMetricType =
            theParameterDatabaseReader.ReadString("olsrv2-link-metric-type", nodeId, interfaceId);
        ConvertStringToLowerCase(linkMetricType);
        if (linkMetricType == "none") {
            if (isParameterSet && (LINK_METRIC_TYPE != LINK_METRIC_TYPE__NONE)) {
                cerr
                    << "Error: nuOLSRv2 doesn't support that"
                    << " each interface on same node has different parameters" << endl;
                exit(1);
            }//if//
            LINK_METRIC_TYPE = LINK_METRIC_TYPE__NONE;
        }
        else if (linkMetricType == "etx") {
            if (isParameterSet && (LINK_METRIC_TYPE != LINK_METRIC_TYPE__ETX)) {
                cerr
                    << "Error: nuOLSRv2 doesn't support that"
                    << " each interface on same node has different parameters" << endl;
                exit(1);
            }//if//
            LINK_METRIC_TYPE = LINK_METRIC_TYPE__ETX;
        }
        else if (linkMetricType == "static") {
            if (isParameterSet && (LINK_METRIC_TYPE != LINK_METRIC_TYPE__STATIC)) {
                cerr
                    << "Error: nuOLSRv2 doesn't support that"
                    << " each interface on same node has different parameters" << endl;
                exit(1);
            }//if//
            LINK_METRIC_TYPE = LINK_METRIC_TYPE__STATIC;
        }
        else {
            cerr
                << "Error: Unknown olsrv2-link-metric-type(" << linkMetricType << ")."
                << " olsrv2-link-metric-type should be none or etx or static." << endl;
            exit(1);
        }//if//
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-lm-etx-memory-length", nodeId, interfaceId)) {
        const int etxMemoryLength =
            theParameterDatabaseReader.ReadInt("olsrv2-lm-etx-memory-length", nodeId, interfaceId);
        if (etxMemoryLength <= 0) {
            cerr
                << "Error: olsrv2-lm-etx-memory-length("
                << etxMemoryLength << ") should be larger than 0." << endl;
            exit(1);
        }//if//
        if (isParameterSet && (ETX_MEMORY_LENGTH != etxMemoryLength)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        ETX_MEMORY_LENGTH = etxMemoryLength;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-lm-etx-metric-interval", nodeId, interfaceId)) {
        const TimeType etxMetricInterval =
            theParameterDatabaseReader.ReadTime("olsrv2-lm-etx-metric-interval", nodeId, interfaceId);
        if (etxMetricInterval <= ZERO_TIME) {
            cerr
                << "Error: olsrv2-lm-etx-metric-interval("
                << etxMetricInterval << ") should be larger than "
                << ZERO_TIME << "." << endl;
            exit(1);
        }//if//
        const double etxMetricIntervalDouble = ConvertTimeToDoubleSecs(etxMetricInterval);
        if (isParameterSet && (ETX_METRIC_INTERVAL != etxMetricIntervalDouble)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        ETX_METRIC_INTERVAL = etxMetricIntervalDouble;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-lm-metric-list-file", nodeId, interfaceId)) {
        const string newMetricListFile =
            theParameterDatabaseReader.ReadString("olsrv2-lm-metric-list-file", nodeId, interfaceId);
        if (isParameterSet && (metricListFile != newMetricListFile)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        metricListFile = newMetricListFile;
        if (!isParameterSet) {
            if (metricListFile.size() != 0) {
                olsrv2_metric_list_load(metricListFile.c_str());
            }//if//
        }//if//
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-tc-interval", nodeId, interfaceId)) {
        const TimeType tcInterval =
            theParameterDatabaseReader.ReadTime("olsrv2-tc-interval", nodeId, interfaceId);
        if (tcInterval <= ZERO_TIME) {
            cerr
                << "Error: olsrv2-tc-interval("
                << tcInterval << ") should be larger than "
                << ZERO_TIME << "." << endl;
            exit(1);
        }//if//
        const double tcIntervalDouble = ConvertTimeToDoubleSecs(tcInterval);
        if (isParameterSet && (TC_INTERVAL != tcIntervalDouble)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        TC_INTERVAL = tcIntervalDouble;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-tc-max-jitter", nodeId, interfaceId)) {
        const TimeType tpMaxJitter =
            theParameterDatabaseReader.ReadTime("olsrv2-tc-max-jitter", nodeId, interfaceId);
        if (tpMaxJitter < ZERO_TIME) {
            cerr
                << "Error: olsrv2-tc-max-jitter("
                << tpMaxJitter << ") should be "
                << ZERO_TIME << " or larger." << endl;
            exit(1);
        }//if//
        const double tpMaxJitterDouble = ConvertTimeToDoubleSecs(tpMaxJitter);
        if (isParameterSet && (TP_MAXJITTER != tpMaxJitterDouble)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        TP_MAXJITTER = tpMaxJitterDouble;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-tc-start-time", nodeId, interfaceId)) {
        const TimeType newStartTc =
            theParameterDatabaseReader.ReadTime("olsrv2-tc-start-time", nodeId, interfaceId);
        if (newStartTc < ZERO_TIME) {
            cerr
                << "Error: olsrv2-tc-start-time("
                << newStartTc << ") should be "
                << ZERO_TIME << " or larger." << endl;
            exit(1);
        }//if//
        if (isParameterSet && (startTc != newStartTc)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        startTc = newStartTc;
        START_TC = ConvertTimeToDoubleSecs(startTc);
    }//if//

    START_TC += aRandomNumberGenerator.GenerateRandomDouble() * TP_MAXJITTER;

    if (theParameterDatabaseReader.ParameterExists("olsrv2-tc-hop-limit", nodeId, interfaceId)) {
        const int tcHopLimit =
            theParameterDatabaseReader.ReadInt("olsrv2-tc-hop-limit", nodeId, interfaceId);
        if (tcHopLimit < 2 || 255 < tcHopLimit) {
            cerr
                << "Error: olsrv2-tc-hop-limit("
                << tcHopLimit << ") should be between 2 and 255." << endl;
            exit(1);
        }//if//
        const uint8_t tcHopLimitUint8t = static_cast<uint8_t>(tcHopLimit);
        if (isParameterSet && (TC_HOP_LIMIT != tcHopLimitUint8t)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        TC_HOP_LIMIT = tcHopLimitUint8t;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-willingness", nodeId, interfaceId)) {
        const int willingness =
            theParameterDatabaseReader.ReadInt("olsrv2-willingness", nodeId, interfaceId);
        if (willingness < WILLINGNESS__NEVER || WILLINGNESS__ALWAYS < willingness) {
            cerr
                << "Error: olsrv2-willingness("
                << willingness << ") should be between "
                << WILLINGNESS__NEVER << " and "
                << WILLINGNESS__ALWAYS << "." << endl;
            exit(1);
        }//if//
        const uint8_t willingnessUint8t = static_cast<uint8_t>(willingness);
        if (isParameterSet && (WILLINGNESS != willingnessUint8t)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        WILLINGNESS = willingnessUint8t;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-broadcast-priority", nodeId, interfaceId)) {
        const PacketPriorityType newBroadcastPriority = static_cast<PacketPriorityType>(
            theParameterDatabaseReader.ReadInt("olsrv2-broadcast-priority", nodeId, interfaceId));
        if (isParameterSet && (broadcastPriority != newBroadcastPriority)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        broadcastPriority = newBroadcastPriority;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-attached-network-address-list", nodeId, interfaceId)) {
        const string newAttachedNetworkListStr =
            theParameterDatabaseReader.ReadString("olsrv2-attached-network-address-list", nodeId, interfaceId);
        if (isParameterSet && (attachedNetworkListStr != newAttachedNetworkListStr)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        attachedNetworkListStr = newAttachedNetworkListStr;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-attached-network-mask-list", nodeId, interfaceId)) {
        const string newAttachedNetworkMaskListStr =
            theParameterDatabaseReader.ReadString("olsrv2-attached-network-mask-list", nodeId, interfaceId);
        if (isParameterSet && (attachedNetworkMaskListStr != newAttachedNetworkMaskListStr)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        attachedNetworkMaskListStr = newAttachedNetworkMaskListStr;
    }//if//

    if (theParameterDatabaseReader.ParameterExists("olsrv2-attached-network-distance-list", nodeId, interfaceId)) {
        const string newAttachedNetworkDistanceListStr =
            theParameterDatabaseReader.ReadString("olsrv2-attached-network-distance-list", nodeId, interfaceId);
        if (isParameterSet && (attachedNetworkDistanceListStr != newAttachedNetworkDistanceListStr)) {
            cerr
                << "Error: nuOLSRv2 doesn't support that"
                << " each interface on same node has different parameters" << endl;
            exit(1);
        }//if//
        attachedNetworkDistanceListStr = newAttachedNetworkDistanceListStr;
    }//if//

    if (!isParameterSet) {
        SetAttachedNetwork();
    }//if//

    isParameterSet = true;

}//ReadParameterConfiguration//


void NuOLSRv2Protocol::SetCurrentNode()
{
    currentNode = this;
    current_olsrv2 = nuOLSRv2Ptr;
    current_packet = nuOLSRv2Ptr->packet;
    current_core = nuOLSRv2Ptr->core;

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    NU_NOW.tv_sec = static_cast<long>(currentTime / SECOND);
    NU_NOW.tv_usec = static_cast<long>((currentTime % SECOND) / MICRO_SECOND);

}//SetCurrentNode//


void NuOLSRv2Protocol::Process()
{
    SetCurrentNode();

    nu_scheduler_exec_events();

}//Process//


void NuOLSRv2Protocol::OutputTraceAndStatsForSendPacket(
    const Packet& packet,
    const unsigned int interfaceIndex,
    const InterfaceIdType& interfaceId)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            const unsigned long long int sourceNodeSequenceNumber =
                packet.GetPacketId().GetSourceNodeSequenceNumber();

            const NodeIdType sourceNodeId =
                packet.GetPacketId().GetSourceNodeId();

            Olsrv2SendTraceRecord traceData;

            traceData.sourceNodeSequenceNumber = sourceNodeSequenceNumber;
            traceData.sourceNodeId = sourceNodeId;

            assert(sizeof(traceData) == OLSRV2_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "Olsrv2Send", traceData);
        }
        else {

            ostringstream outStream;

            outStream << "PktId= " << packet.GetPacketId();

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "Olsrv2Send", outStream.str());
        }//if//
    }//if//

    nuOLSRv2Stats[interfaceIndex].packetsSentStatPtr->IncrementCounter();

    const size_t packetLengthBytes = packet.LengthBytes();
    nuOLSRv2Stats[interfaceIndex].bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForSendPacket//


void NuOLSRv2Protocol::OutputTraceAndStatsForReceivePacket(
    const Packet& packet,
    const unsigned int interfaceIndex,
    const InterfaceIdType& interfaceId)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            const unsigned long long int sourceNodeSequenceNumber =
                packet.GetPacketId().GetSourceNodeSequenceNumber();

            const NodeIdType sourceNodeId =
                packet.GetPacketId().GetSourceNodeId();

            Olsrv2ReceiveTraceRecord traceData;

            traceData.sourceNodeSequenceNumber = sourceNodeSequenceNumber;
            traceData.sourceNodeId = sourceNodeId;

            assert(sizeof(traceData) == OLSRV2_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "Olsrv2Recv", traceData);
        }
        else {

            ostringstream outStream;

            outStream << "PktId= " << packet.GetPacketId();

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "Olsrv2Recv", outStream.str());
        }//if//
    }//if//

    nuOLSRv2Stats[interfaceIndex].packetsReceivedStatPtr->IncrementCounter();

    const size_t packetLengthBytes = packet.LengthBytes();
    nuOLSRv2Stats[interfaceIndex].bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForReceivePacket//


void NuOLSRv2Protocol::OutputTraceAndStatsForAddRoute(
    const NetworkAddress& destinationAddress,
    const NetworkAddress& netmaskAddress,
    const NetworkAddress& nextHopAddress,
    const unsigned int interfaceIndex,
    const InterfaceIdType& interfaceId)
{
    const NetworkAddress localAddress =
        networkLayerPtr->GetNetworkAddress(interfaceIndex);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            RoutingTableAddEntryTraceRecord traceData;

            traceData.destinationAddress = Ipv6NetworkAddress(destinationAddress);
            traceData.netmaskAddress = Ipv6NetworkAddress(netmaskAddress);
            traceData.nextHopAddress = Ipv6NetworkAddress(nextHopAddress);
            traceData.localAddress = Ipv6NetworkAddress(localAddress);
            traceData.useIpv6 = (NetworkAddress::numberBits == 128);

            assert(sizeof(traceData) == ROUTING_TABLE_ADD_ENTRY_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "Olsrv2AddEntry", traceData);
        }
        else {

            ostringstream outStream;

            outStream
                << "Dest= " << destinationAddress.ConvertToString()
                << " Mask= " << netmaskAddress.ConvertToString()
                << " Next= " << nextHopAddress.ConvertToString()
                << " IF= " << localAddress.ConvertToString();

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "Olsrv2AddEntry", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForAddRoute//


void NuOLSRv2Protocol::OutputTraceAndStatsForDeleteRoute(
    const NetworkAddress& destinationAddress,
    const NetworkAddress& netmaskAddress,
    const InterfaceIdType& interfaceId)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            RoutingTableDeleteEntryTraceRecord traceData;

            traceData.destinationAddress = Ipv6NetworkAddress(destinationAddress);
            traceData.netmaskAddress = Ipv6NetworkAddress(netmaskAddress);
            traceData.useIpv6 = (NetworkAddress::numberBits == 128);

            assert(sizeof(traceData) == ROUTING_TABLE_DELETE_ENTRY_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "Olsrv2DelEntry", traceData);
        }
        else {

            ostringstream outStream;

            outStream
                << "Dest= " << destinationAddress.ConvertToString()
                << " Mask= " << netmaskAddress.ConvertToString();

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "Olsrv2DelEntry", outStream.str());
        }//if//
    }//if//
}//OutputTraceAndStatsForDeleteRoute//

}//namespace//
