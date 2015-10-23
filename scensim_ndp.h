// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_NDP_H
#define SCENSIM_NDP_H

#include <scensim_network.h>
#include <algorithm>

namespace ScenSim {

using std::copy;

class NeighborCache: public enable_shared_from_this<NeighborCache> {
public:
    NeighborCache(): cache() {}
    virtual ~NeighborCache() {}

    bool NeighborIsCached(
        const NetworkAddress& networkAddress) const;

    void LookupByNetworkAddress(
        const NetworkAddress& networkAddress,
        bool& wasFound,
        SixByteMacAddressType& resolvedMacAddress) const;

    void LookupByMacAddress(
        const SixByteMacAddressType& macAddress,
        bool& wasFound,
        vector<NetworkAddress>& networkAddresses) const;

    void AddOrUpdate(
        const NetworkAddress& networkAddress,
        const SixByteMacAddressType& resolvedMacAddress);

    void Delete(const NetworkAddress& networkAddress);

private:
    class NeighborEntry: public enable_shared_from_this<NeighborEntry> {
    public:
        NeighborEntry()
            :
            networkAddress(),
            macAddress()
        {
        }//NeighborEntry//

        NetworkAddress networkAddress;
        SixByteMacAddressType macAddress;
    };

    vector<NeighborEntry> cache;

};//NeighborCache//

inline
bool NeighborCache::NeighborIsCached(
    const NetworkAddress& networkAddress) const
{
    bool wasFound;
    SixByteMacAddressType resolvedMacAddress;

    LookupByNetworkAddress(
        networkAddress,
        wasFound,
        resolvedMacAddress);

    return wasFound;

}//NeighborIsCached//

inline
void NeighborCache::LookupByNetworkAddress(
    const NetworkAddress& networkAddress,
    bool& wasFound,
    SixByteMacAddressType& resolvedMacAddress) const
{
    wasFound = false;

    if (networkAddress.IsTheBroadcastAddress()) {
        wasFound = true;
        resolvedMacAddress = SixByteMacAddressType::GetBroadcastAddress();
        return;
    }//if//

    vector<NeighborEntry>::const_iterator it;

    for(it = cache.begin(); it != cache.end(); it++) {
        const NeighborEntry& entry = *it;
        if (entry.networkAddress == networkAddress) {
            wasFound = true;
            resolvedMacAddress = entry.macAddress;
            return;
        }//if//
    }//for//

}//LookupByNetworkAddress//

inline
void NeighborCache::LookupByMacAddress(
    const SixByteMacAddressType& macAddress,
    bool& wasFound,
    vector<NetworkAddress>& networkAddresses) const
{
    wasFound = false;
    vector<NeighborEntry>::const_iterator it;

    for(it = cache.begin(); it != cache.end(); it++) {
        const NeighborEntry& entry = *it;
        if (entry.macAddress == macAddress) {
            wasFound = true;
            networkAddresses.push_back(entry.networkAddress);
        }//if//
    }//for//

}//LookupByMacAddress//

inline
void NeighborCache::AddOrUpdate(
    const NetworkAddress& networkAddress,
    const SixByteMacAddressType& macAddress)
{
    Delete(networkAddress);

    NeighborEntry entry;
    entry.networkAddress = networkAddress;
    entry.macAddress = macAddress;

    cache.push_back(entry);

}//AddOrUpdate//

inline
void NeighborCache::Delete(
    const NetworkAddress& networkAddress)
{
    vector<NeighborEntry>::iterator it;

    for(it = cache.begin(); it != cache.end(); it++) {
        const NeighborEntry& entry = *it;
        if (entry.networkAddress == networkAddress) {
            cache.erase(it);
            break;
        }//if//
    }//for//

}//Delete//

class NeighborDiscoveryProtocol: public enable_shared_from_this<NeighborDiscoveryProtocol> {
public:
    NeighborDiscoveryProtocol(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const RandomNumberGeneratorSeedType& nodeSeed);

    virtual ~NeighborDiscoveryProtocol();

    bool NeighborIsCached(
        const NetworkAddress& networkAddress) const;

    bool IsAddressResolutionEnabled() const { return addressResolution; }

    void ProcessLinkIsUpNotification();
    void ProcessLinkIsDownNotification();
    void ProcessNewLinkToANodeNotification(const GenericMacAddressType& macAddress);

    void ProcessIcmpPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const NetworkAddress& lastHopAddress);

    void SendNeighborSolicitationMessage(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& targetAddress,
        const PacketPriorityType& priority,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED);

    shared_ptr<NeighborCache> GetNeighborCachePtr() const { return neighborCachePtr; }

private:
    static const TimeType connectionAdvertisementLostRetryDelay = 20 * MILLI_SECOND;
    static const int arbitraryHashValue = 237877;

    class SolicitationRetryEvent: public SimulationEvent {
    public:
        SolicitationRetryEvent(
            const shared_ptr<NeighborDiscoveryProtocol>& initNdpPtr)
            :
            ndpPtr(initNdpPtr)
        {
        }//SolicitationRetryEvent//

        void ExecuteEvent() {
            ndpPtr->ProcessRouterSolicitationRetryEvent();
        }//ExecuteEvent//

    private:
        const shared_ptr<NeighborDiscoveryProtocol> ndpPtr;

    };//SolicitationRetryEvent//

    class AdvertisementEvent: public SimulationEvent {
    public:
        AdvertisementEvent(
            NeighborDiscoveryProtocol* initNdpPtr)
            :
            ndpPtr(initNdpPtr)
        {
        }//AdvertisementEvent//

        void ExecuteEvent() {
            ndpPtr->ProcessRouterAdvertisementEvent();
        }//ExecuteEvent//

    private:
        NeighborDiscoveryProtocol* ndpPtr;

    };//AdvertisementEvent//

    struct PacketCacheItemType {

        unique_ptr<Packet> packetPtr;
        NetworkAddress nextHopAddress;
        PacketPriorityType priority;
        EtherTypeFieldType etherType;

        PacketCacheItemType(
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initNextHopAddress,
            const PacketPriorityType& initPriority,
            const EtherTypeFieldType& initEtherType)
            :
            packetPtr(move(initPacketPtr)),
            nextHopAddress(initNextHopAddress),
            priority(initPriority),
            etherType(initEtherType)
        {
        }//PacketCacheItemType//

        void operator=(PacketCacheItemType&& right) {
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
            priority = right.priority;
            etherType = right.etherType;
        }

        PacketCacheItemType(PacketCacheItemType&& right) { (*this) = move(right); }

    };//PacketCacheItemType//

    void SendNeighborAdvertisementMessage(
        const NetworkAddress& destinationAddress);
    void SendRouterSolicitationMessage();
    void SendRouterAdvertisementMessage();

    void ProcessNeighborSolicitationMessage(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const NetworkAddress& lastHopAddress);

    void ProcessNeighborAdvertisementMessage(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const NetworkAddress& lastHopAddress);

    void ProcessRouterSolicitationMessage(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const NetworkAddress& lastHopAddress);

    void ProcessRouterAdvertisementMessage(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const NetworkAddress& lastHopAddress);

    void StartRouterAdvertisementTimer(const TimeType& eventTime);

    void ProcessRouterSolicitationRetryEvent();
    void ProcessRouterAdvertisementEvent();
    void NotifyCacheTableUpdated(const NetworkAddress& targetAddress);

    const string modelName;
    unsigned int interfaceIndex;
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    shared_ptr<NetworkLayer> networkLayerPtr;

    bool isRouter;
    bool accessPointSendsIpAddressOnConnection;
    bool addressResolution;
    bool addressAutoconfiguration;
    bool gatewayAutoconfiguration;

    TimeType routerAdvertisementInterval;
    TimeType routerAdvertisementJitter;

    shared_ptr<AdvertisementEvent> advertisementTimerEventPtr;
    EventRescheduleTicket advertisementEventTicket;
    shared_ptr<NeighborCache> neighborCachePtr;
    map<NetworkAddress, PacketCacheItemType> packetCache;

    RandomNumberGenerator aRandomNumberGenerator;

    shared_ptr<CounterStatistic> nsPacketsSentStatPtr;
    shared_ptr<CounterStatistic> naPacketsSentStatPtr;
    shared_ptr<CounterStatistic> rsPacketsSentStatPtr;
    shared_ptr<CounterStatistic> raPacketsSentStatPtr;
    shared_ptr<CounterStatistic> nsPacketsReceivedStatPtr;
    shared_ptr<CounterStatistic> naPacketsReceivedStatPtr;
    shared_ptr<CounterStatistic> rsPacketsReceivedStatPtr;
    shared_ptr<CounterStatistic> raPacketsReceivedStatPtr;
    shared_ptr<CounterStatistic> fullQueueDropsStatPtr;

};//NeighborDiscoveryProtocol//

inline
NeighborDiscoveryProtocol::NeighborDiscoveryProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId,
    const unsigned int initInterfaceIndex,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const RandomNumberGeneratorSeedType& nodeSeed)
    :
    modelName("Ndp"),
    interfaceIndex(initInterfaceIndex),
    simulationEngineInterfacePtr(initSimEngineInterfacePtr),
    networkLayerPtr(initNetworkLayerPtr),
    isRouter(false),
    accessPointSendsIpAddressOnConnection(false),
    addressResolution(false),
    addressAutoconfiguration(true),
    gatewayAutoconfiguration(true),
    routerAdvertisementInterval(),
    routerAdvertisementJitter(),
    advertisementTimerEventPtr(),
    advertisementEventTicket(),
    neighborCachePtr(),
    packetCache(),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(nodeSeed, initInterfaceIndex, arbitraryHashValue)),
    nsPacketsSentStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + interfaceId + "_NeighborSolicitationPacketsSent"))),
    naPacketsSentStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + interfaceId + "_NeighborAdvertisementPacketsSent"))),
    rsPacketsSentStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + interfaceId + "_RouterSolicitationPacketsSent"))),
    raPacketsSentStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + interfaceId + "_RouterAdvertisementPacketsSent"))),
    nsPacketsReceivedStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + interfaceId + "_NeighborSolicitationPacketsReceived"))),
    naPacketsReceivedStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + interfaceId + "_NeighborAdvertisementPacketsReceived"))),
    rsPacketsReceivedStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + interfaceId + "_RouterSolicitationPacketsReceived"))),
    raPacketsReceivedStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + interfaceId + "_RouterAdvertisementPacketsReceived"))),
    fullQueueDropsStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + interfaceId + "_FullQueueDrops")))
{
    if (!IpHeaderModel::usingVersion6) {
        cerr << "Error: NDP doesn't support IPv4" << endl;
        exit(1);
    }//if//

    if (!theParameterDatabaseReader.ParameterExists("network-enable-ndp", nodeId, interfaceId) ||
        !theParameterDatabaseReader.ReadBool("network-enable-ndp", nodeId, interfaceId)) {
        cerr << "Error: network-enable-ndp(false) should be true" << endl;
        exit(1);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("network-ndp-mode", nodeId, interfaceId)) {
        const string ndpMode = MakeLowerCaseString(
            theParameterDatabaseReader.ReadString("network-ndp-mode", nodeId, interfaceId));
        if (ndpMode == "host") {
            isRouter = false;
        }
        else if (ndpMode == "router") {
            isRouter = true;
        }
        else {
            cerr << "Error: network-ndp-mode(" << ndpMode
                 << ") should be host or router" << endl;
            exit(1);
        }//if//
    }
    else {
        cerr << "Error: network-ndp-mode should be defined"
             << " if network-enable-ndp is true" << endl;
        exit(1);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("network-ndp-address-resolution", nodeId, interfaceId)) {
        addressResolution = theParameterDatabaseReader.ReadBool("network-ndp-address-resolution", nodeId, interfaceId);
    }
    else {
        cerr << "Error: network-ndp-address-resolution should be defined"
             << " if network-enable-ndp is true" << endl;
        exit(1);
    }//if//

    if (!isRouter) {
        if (theParameterDatabaseReader.ParameterExists("network-ndp-address-autoconfiguration", nodeId, interfaceId)) {
            addressAutoconfiguration = theParameterDatabaseReader.ReadBool("network-ndp-address-autoconfiguration", nodeId, interfaceId);
        }
        else {
            cerr << "Error: network-ndp-address-autoconfiguration should be defined"
                 << " if network-ndp-mode is host" << endl;
            exit(1);
        }//if//

        if (theParameterDatabaseReader.ParameterExists("network-ndp-gateway-autoconfiguration", nodeId, interfaceId)) {
            gatewayAutoconfiguration = theParameterDatabaseReader.ReadBool("network-ndp-gateway-autoconfiguration", nodeId, interfaceId);
        }
        else {
            cerr << "Error: network-ndp-gateway-autoconfiguration should be defined"
                 << " if network-ndp-mode is host" << endl;
            exit(1);
        }//if//
    }//if//

    if (isRouter) {
        if (theParameterDatabaseReader.ParameterExists("network-ndp-router-advertisement-interval", nodeId, interfaceId)) {
            routerAdvertisementInterval =
                theParameterDatabaseReader.ReadTime(
                    "network-ndp-router-advertisement-interval", nodeId, interfaceId);
        }
        else {
            cerr << "Error: network-ndp-router-advertisement-interval should be defined"
                 << " if network-ndp-mode is router" << endl;
            exit(1);
        }//if//

        if (theParameterDatabaseReader.ParameterExists("network-ndp-router-advertisement-jitter", nodeId, interfaceId)) {
            routerAdvertisementJitter =
                theParameterDatabaseReader.ReadTime(
                    "network-ndp-router-advertisement-jitter", nodeId, interfaceId);
        }
        else {
            cerr << "Error: network-ndp-router-advertisement-jitter should be defined"
                 << " if network-ndp-mode is router" << endl;
            exit(1);
        }//if//

        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        const TimeType eventTime = currentTime + routerAdvertisementInterval;
        const TimeType jitterTime = static_cast<TimeType>(
            aRandomNumberGenerator.GenerateRandomDouble() * routerAdvertisementJitter);

        this->StartRouterAdvertisementTimer(eventTime + jitterTime);
    }//if//

    neighborCachePtr.reset(new NeighborCache());

}//NeighborDiscoveryProtocol()

inline
NeighborDiscoveryProtocol::~NeighborDiscoveryProtocol()
{
    map<NetworkAddress, PacketCacheItemType>::iterator iter;

    for(iter = packetCache.begin(); iter != packetCache.end(); ++iter) {
        assert(iter->second.packetPtr != nullptr);
        iter->second.packetPtr = nullptr;
    }//for//

}//~NeighborDiscoveryProtocol//

inline
bool NeighborDiscoveryProtocol::NeighborIsCached(
    const NetworkAddress& networkAddress) const
{
    if (addressResolution) {
        return neighborCachePtr->NeighborIsCached(networkAddress);
    }
    else {
        return true;
    }//if//

}//NeighborIsCached//

inline
void NeighborDiscoveryProtocol::ProcessLinkIsUpNotification()
{
    if (!isRouter) {
        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        TimeType eventTime = currentTime;

        if (!accessPointSendsIpAddressOnConnection) {
            // Send now in wireless with access points no point in delaying.
        }
        else {
            // Send solicitation after short duration if advertisement did not get through.
            eventTime += connectionAdvertisementLostRetryDelay;
        }//if//

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new SolicitationRetryEvent(shared_from_this())),
            eventTime);
    }//if//

}//LinkIsUpNotification//

inline
void NeighborDiscoveryProtocol::ProcessLinkIsDownNotification()
{
    if (!isRouter) {
        if (addressAutoconfiguration) {
            const unsigned int prefixLength = networkLayerPtr->GetSubnetMaskBitLength(interfaceIndex);
            networkLayerPtr->SetInterfaceIpAddress(
                interfaceIndex,
                NetworkAddress::invalidAddress,
                prefixLength);
        }//if//

        if (gatewayAutoconfiguration) {
            shared_ptr<RoutingTable> routingTablePtr = networkLayerPtr->GetRoutingTableInterface();
            routingTablePtr->DeleteRoute(
                NetworkAddress::anyAddress,
                NetworkAddress::anyAddress);
        }//if//
    }//if//

}//ProcessLinkIsDownNotification//

inline
void NeighborDiscoveryProtocol::ProcessNewLinkToANodeNotification(
    const GenericMacAddressType& newNodeMacAddress)
{
    if (isRouter && accessPointSendsIpAddressOnConnection) {
        const TimeType eventTime = simulationEngineInterfacePtr->CurrentTime();
        this->StartRouterAdvertisementTimer(eventTime);
    }//if//

}//NewLinkToNodeNotification//

inline
void NeighborDiscoveryProtocol::ProcessIcmpPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const PacketPriorityType trafficClass,
    const NetworkAddress& lastHopAddress)
{
    const IpIcmpHeaderType& icmpHeader =
        packetPtr->GetAndReinterpretPayloadData<IpIcmpHeaderType>();

    if (icmpHeader.icmpType == IpRouterSolicitationMessage::icmpTypeNum) {
        ProcessRouterSolicitationMessage(
            packetPtr,
            sourceAddress,
            destinationAddress,
            trafficClass,
            lastHopAddress);
    }
    else if (icmpHeader.icmpType == IpRouterAdvertisementMessage::icmpTypeNum) {
        ProcessRouterAdvertisementMessage(
            packetPtr,
            sourceAddress,
            destinationAddress,
            trafficClass,
            lastHopAddress);
    }
    else if (icmpHeader.icmpType == IpNeighborSolicitationMessage::icmpTypeNum) {
        ProcessNeighborSolicitationMessage(
            packetPtr,
            sourceAddress,
            destinationAddress,
            trafficClass,
            lastHopAddress);
    }
    else if (icmpHeader.icmpType == IpNeighborAdvertisementMessage::icmpTypeNum) {
        ProcessNeighborAdvertisementMessage(
            packetPtr,
            sourceAddress,
            destinationAddress,
            trafficClass,
            lastHopAddress);
    }//if//

}//ProcessIcmpPacket//

inline
void NeighborDiscoveryProtocol::SendNeighborSolicitationMessage(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& targetAddress,
    const PacketPriorityType& priority,
    const EtherTypeFieldType etherType)
{
    if (neighborCachePtr->NeighborIsCached(targetAddress)) {
        assert(false);
        abort();
    }//if//

    map<NetworkAddress, PacketCacheItemType>::iterator iter = packetCache.find(targetAddress);
    if (iter != packetCache.end()) {
        if (iter->second.packetPtr != nullptr) {
            iter->second.packetPtr = nullptr;
            fullQueueDropsStatPtr->IncrementCounter();
        }//if//
        packetCache.erase(iter);
    }//if//
    PacketCacheItemType packetCacheItem(packetPtr, targetAddress, priority, etherType);
    packetCache.insert(move(make_pair(targetAddress, move(packetCacheItem))));

    IpNeighborSolicitationMessage solicitationMessage;
    solicitationMessage.icmpHeader.icmpType = IpNeighborSolicitationMessage::icmpTypeNum;
    solicitationMessage.targetAddressHighBits = targetAddress.GetRawAddressHighBits();
    solicitationMessage.targetAddressLowBits = targetAddress.GetRawAddressLowBits();
    solicitationMessage.linkLayerAddressOption =
       SixByteMacAddressType(networkLayerPtr->GetMacLayerPtr(interfaceIndex)->GetGenericMacAddress());

    unique_ptr<Packet> messagePacketPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, solicitationMessage);

    networkLayerPtr->ReceiveOutgoingBroadcastPacket(
        messagePacketPtr,
        interfaceIndex,
        MAX_AVAILABLE_PACKET_PRIORITY,
        IP_PROTOCOL_NUMBER_ICMP);

    nsPacketsSentStatPtr->IncrementCounter();

}//SendNeighborSolicitationMessage//

inline
void NeighborDiscoveryProtocol::SendNeighborAdvertisementMessage(
    const NetworkAddress& destinationAddress)
{
    const NetworkAddress myAddress =
        networkLayerPtr->GetNetworkAddress(interfaceIndex);

    IpNeighborAdvertisementMessage advertMessage;
    advertMessage.icmpHeader.icmpType = IpNeighborAdvertisementMessage::icmpTypeNum;
    advertMessage.targetAddressHighBits = myAddress.GetRawAddressHighBits();
    advertMessage.targetAddressLowBits = myAddress.GetRawAddressLowBits();
    advertMessage.linkLayerAddressOption =
        SixByteMacAddressType(networkLayerPtr->GetMacLayerPtr(interfaceIndex)->GetGenericMacAddress());

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, advertMessage);

    networkLayerPtr->ReceiveOutgoingBroadcastPacket(
        packetPtr,
        interfaceIndex,
        MAX_AVAILABLE_PACKET_PRIORITY,
        IP_PROTOCOL_NUMBER_ICMP);

    naPacketsSentStatPtr->IncrementCounter();

}//SendNeighborAdvertisementMessage//

inline
void NeighborDiscoveryProtocol::SendRouterSolicitationMessage()
{
    IpRouterSolicitationMessage solicitationMessage;
    solicitationMessage.icmpHeader.icmpType = IpRouterSolicitationMessage::icmpTypeNum;

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, solicitationMessage);

    networkLayerPtr->ReceiveOutgoingBroadcastPacket(
        packetPtr,
        interfaceIndex,
        MAX_AVAILABLE_PACKET_PRIORITY,
        IP_PROTOCOL_NUMBER_ICMP);

    rsPacketsSentStatPtr->IncrementCounter();

}//SendRouterSolicitationMessage//

inline
void NeighborDiscoveryProtocol::SendRouterAdvertisementMessage()
{
    const NetworkAddress myAddress = networkLayerPtr->GetNetworkAddress(interfaceIndex);
    const NetworkAddress mySubnetAddress =
        myAddress.MakeSubnetAddress(networkLayerPtr->GetSubnetMask(interfaceIndex));

    IpRouterAdvertisementMessage advertMessage;
    advertMessage.icmpHeader.icmpType = IpRouterAdvertisementMessage::icmpTypeNum;
    advertMessage.ipPrefixHighBits = mySubnetAddress.GetRawAddressHighBits();
    advertMessage.ipPrefixLowBits = mySubnetAddress.GetRawAddressLowBits();
    advertMessage.prefixLength = static_cast<uint16_t>(
        networkLayerPtr->GetSubnetMaskBitLength(interfaceIndex));

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, advertMessage);

    networkLayerPtr->ReceiveOutgoingBroadcastPacket(
        packetPtr,
        interfaceIndex,
        MAX_AVAILABLE_PACKET_PRIORITY,
        IP_PROTOCOL_NUMBER_ICMP);

    raPacketsSentStatPtr->IncrementCounter();

}//SendRouterAdvertisementMessage//

inline
void NeighborDiscoveryProtocol::ProcessNeighborSolicitationMessage(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const PacketPriorityType trafficClass,
    const NetworkAddress& lastHopAddress)
{
    nsPacketsReceivedStatPtr->IncrementCounter();

    const IpNeighborSolicitationMessage& solicitationMessage =
        packetPtr->GetAndReinterpretPayloadData<IpNeighborSolicitationMessage>();

    NetworkAddress targetAddress(
        solicitationMessage.targetAddressHighBits,
        solicitationMessage.targetAddressLowBits);

    neighborCachePtr->AddOrUpdate(sourceAddress, solicitationMessage.linkLayerAddressOption);
    NotifyCacheTableUpdated(sourceAddress);

    const NetworkAddress myAddress = networkLayerPtr->GetNetworkAddress(interfaceIndex);
    if (myAddress == targetAddress) {
        SendNeighborAdvertisementMessage(sourceAddress);
    }//if//

    packetPtr = nullptr;

}//ProcessNeighborSolicitationMessage//

inline
void NeighborDiscoveryProtocol::ProcessNeighborAdvertisementMessage(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const PacketPriorityType trafficClass,
    const NetworkAddress& lastHopAddress)
{
    naPacketsReceivedStatPtr->IncrementCounter();

    const IpNeighborAdvertisementMessage& advertMessage =
        packetPtr->GetAndReinterpretPayloadData<IpNeighborAdvertisementMessage>();

    NetworkAddress targetAddress(
        advertMessage.targetAddressHighBits,
        advertMessage.targetAddressLowBits);

    neighborCachePtr->AddOrUpdate(targetAddress, advertMessage.linkLayerAddressOption);
    NotifyCacheTableUpdated(sourceAddress);

    packetPtr = nullptr;

}//ProcessNeighborAdvertisementMessage//

inline
void NeighborDiscoveryProtocol::ProcessRouterSolicitationMessage(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const PacketPriorityType trafficClass,
    const NetworkAddress& lastHopAddress)
{
    if (isRouter) {
        rsPacketsReceivedStatPtr->IncrementCounter();
        const TimeType eventTime = simulationEngineInterfacePtr->CurrentTime();
        this->StartRouterAdvertisementTimer(eventTime);
    }//if//

    packetPtr = nullptr;

}//ProcessRouterSolicitationMessage//

inline
void NeighborDiscoveryProtocol::ProcessRouterAdvertisementMessage(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const PacketPriorityType trafficClass,
    const NetworkAddress& lastHopAddress)
{
    if (!isRouter) {
        raPacketsReceivedStatPtr->IncrementCounter();

        if (addressAutoconfiguration) {
            const IpRouterAdvertisementMessage& routerAdvert =
                packetPtr->GetAndReinterpretPayloadData<IpRouterAdvertisementMessage>();

            const NetworkAddress routerPrefix(routerAdvert.ipPrefixHighBits, routerAdvert.ipPrefixLowBits);
            const NetworkAddress newAddress(routerPrefix, NetworkAddress(networkLayerPtr->GetNodeId()));

            if (newAddress != networkLayerPtr->GetNetworkAddress(interfaceIndex)) {
                networkLayerPtr->SetInterfaceIpAddress(
                    interfaceIndex,
                    newAddress,
                    routerAdvert.prefixLength);
            }//if//
        }//if//

        if (gatewayAutoconfiguration) {
            //TBD: multiple routers are not supported
            shared_ptr<RoutingTable> routingTablePtr = networkLayerPtr->GetRoutingTableInterface();
            routingTablePtr->AddOrUpdateRoute(
                NetworkAddress::anyAddress,
                NetworkAddress::anyAddress,
                sourceAddress,
                interfaceIndex);
        }//if//
    }//if//

    packetPtr = nullptr;

}//ProcessRouterAdvertisementMessage//

inline
void NeighborDiscoveryProtocol::StartRouterAdvertisementTimer(
    const TimeType& eventTime)
{
    if (advertisementTimerEventPtr == nullptr) {
        advertisementTimerEventPtr.reset(new AdvertisementEvent(this));
    }//if//

    if (advertisementEventTicket.IsNull()) {
        simulationEngineInterfacePtr->ScheduleEvent(
            advertisementTimerEventPtr, eventTime, advertisementEventTicket);
    }
    else {
        simulationEngineInterfacePtr->RescheduleEvent(
            advertisementEventTicket, eventTime);
    }//if//

}//StartRouterAdvertisementTimer//

inline
void NeighborDiscoveryProtocol::ProcessRouterSolicitationRetryEvent()
{
    assert(!isRouter);

    if (networkLayerPtr->GetNetworkAddress(interfaceIndex) == NetworkAddress::invalidAddress) {
        this->SendRouterSolicitationMessage();
    }//if//

}//ProcessRouterSolicitationRetryEvent//

inline
void NeighborDiscoveryProtocol::ProcessRouterAdvertisementEvent()
{
    assert(isRouter);

    advertisementEventTicket.Clear();
    this->SendRouterAdvertisementMessage();

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    const TimeType eventTime = currentTime + routerAdvertisementInterval;
    const TimeType jitterTime = static_cast<TimeType>(
        aRandomNumberGenerator.GenerateRandomDouble() * routerAdvertisementJitter);
    this->StartRouterAdvertisementTimer(eventTime + jitterTime);

}//ProcessRouterAdvertisementEvent//

inline
void NeighborDiscoveryProtocol::NotifyCacheTableUpdated(
    const NetworkAddress& targetAddress)
{
    map<NetworkAddress, PacketCacheItemType>::iterator iter =
        packetCache.find(targetAddress);

    if (iter != packetCache.end()) {
        assert(iter->second.packetPtr != nullptr);
        networkLayerPtr->InsertPacketIntoAnOutputQueue(
            iter->second.packetPtr,
            interfaceIndex,
            iter->second.nextHopAddress,
            iter->second.priority,
            iter->second.etherType);
        packetCache.erase(iter);
    }//if//

}//NotifyCacheTableUpdated//

class NdpMacAddressResolver: public MacAddressResolver<SixByteMacAddressType> {
public:
    NdpMacAddressResolver(
        const shared_ptr<NeighborDiscoveryProtocol>& ndpPtr);

    virtual void GetMacAddress(
        const NetworkAddress& aNetworkAddress,
        const NetworkAddress& networkAddressMask,
        bool& wasFound,
        SixByteMacAddressType& resolvedMacAddress);

    virtual void GetNetworkAddressIfAvailable(
        const SixByteMacAddressType& macAddress,
        const NetworkAddress& subnetNetworkAddress,
        bool& wasFound,
        NetworkAddress& resolvedNetworkAddress);

private:
    shared_ptr<NeighborCache> neighborCachePtr;

};//NdpMacAddressResolver//

inline
NdpMacAddressResolver::NdpMacAddressResolver(
    const shared_ptr<NeighborDiscoveryProtocol>& ndpPtr)
{
    neighborCachePtr = ndpPtr->GetNeighborCachePtr();

}//NdpMacAddressResolver//

inline
void NdpMacAddressResolver::GetMacAddress(
    const NetworkAddress& aNetworkAddress,
    const NetworkAddress& networkAddressMask,
    bool& wasFound,
    SixByteMacAddressType& resolvedMacAddress)
{
    neighborCachePtr->LookupByNetworkAddress(aNetworkAddress, wasFound, resolvedMacAddress);

}//GetMacAddress//

inline
void NdpMacAddressResolver::GetNetworkAddressIfAvailable(
    const SixByteMacAddressType& macAddress,
    const NetworkAddress& subnetNetworkAddress,
    bool& wasFound,
    NetworkAddress& resolvedNetworkAddress)
{
    vector<NetworkAddress> networkAddresses;
    neighborCachePtr->LookupByMacAddress(macAddress, wasFound, networkAddresses);

    //TBD: multiple IP addresses on an interface are not supported
    if (wasFound) {
        resolvedNetworkAddress = networkAddresses[0];
    }//if//

}//GetNetworkAddressIfAvailable//

}//namespace//

#endif
