// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_ARP_H
#define SCENSIM_ARP_H

#include <scensim_network.h>
#include <algorithm>

namespace ScenSim {

using std::copy;

class AddressResolutionProtocol : public enable_shared_from_this<AddressResolutionProtocol> {

public:
    AddressResolutionProtocol(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const InterfaceIdType& initInterfaceId,
        const unsigned int initInterfaceIndex,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const RandomNumberGeneratorSeedType& initNodeSeed);
    virtual ~AddressResolutionProtocol();

    void NotifyProtocolAddressChanged();

    void SendArpRequest(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType& priority,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED);

    void ReceiveArpPacket(const Packet& aPacket);

    void LookupHardwareAddress(
        const NetworkAddress& nextHopAddress,
        bool& wasFound,
        GenericMacAddressType& macAddress) const;

    bool HardwareAddressIsInTable(const NetworkAddress& nextHopAddress) const
    {
        bool wasFound;
        GenericMacAddressType notUsed;
        LookupHardwareAddress(nextHopAddress, wasFound, notUsed);
        return (wasFound);
    }

    void LookupProtocolAddress(
        const GenericMacAddressType& macAddress,
        bool& wasFound,
        NetworkAddress& nextHopAddress) const;

//private:
    static const int hashValue = 8265227;

    enum HardwareType { Ethernet = 1 };
    enum ProtocolType { Ip = ETHERTYPE_IP, Arp = ETHERTYPE_ARP };
    enum OperationType { ArpRequest = 1, ArpReply = 2 };
    enum AddressStateType { NotAssigned, Assigned, Duplicated };
    enum TimerStateType { Idle, Probe, Announce };

    struct ArpPacketFormatType {

        uint16_t hardwareType;
        uint16_t protocolType;
        uint8_t hardwareAddressLength;
        uint8_t protocolAddressLength;
        uint16_t operationType;

        SixByteMacAddressType senderHardwareAddress;
        uint16_t senderProtocolAddressHighBits;        // Off Alignment, so split
        uint16_t senderProtocolAddressLowBits;
        SixByteMacAddressType targetHardwareAddress;
        uint32_t targetProtocolAddress;

        ArpPacketFormatType(
            enum OperationType operationType,
            const NetworkAddress& senderIpAddress,
            const SixByteMacAddressType& senderMacAddress,
            const NetworkAddress& targetIpAddress,
            const SixByteMacAddressType& targetMacAddress)
            :
            hardwareType(HostToNet16(Ethernet)),
            protocolType(HostToNet16(Ip)),
            hardwareAddressLength(SixByteMacAddressType::numberMacAddressBytes),
            protocolAddressLength(NetworkAddress::numberBits/8),
            operationType(HostToNet16(static_cast<uint16_t>(operationType))),
            senderHardwareAddress(senderMacAddress),
            targetHardwareAddress(targetMacAddress),
            targetProtocolAddress(HostToNet32(targetIpAddress.GetRawAddressLow32Bits()))
        {
            if (IpHeaderModel::usingVersion6) {
                cerr << "Error: ARP doesn't support IPv6 as a protocol address." << endl;
                exit(1);
            }//if//

            ConvertHost32ToTwoNet16(
                senderIpAddress.GetRawAddressLow32Bits(),
                senderProtocolAddressHighBits, senderProtocolAddressLowBits);

        }//ArpPacketFormatType//

        GenericMacAddressType GetSenderHardwareAddress() const {
            return (senderHardwareAddress.ConvertToGenericMacAddress());
        }

        NetworkAddress GetSenderProtocolAddress() const {
            return (
                NetworkAddress(
                    ConvertTwoNet16ToHost32(
                        senderProtocolAddressHighBits,
                        senderProtocolAddressLowBits)));
        }

        GenericMacAddressType GetTargetHardwareAddress() const {
            return targetHardwareAddress.ConvertToGenericMacAddress();
        }

        NetworkAddress GetTargetProtocolAddress() const {
            return (NetworkAddress(NetToHost32(targetProtocolAddress)));
        }

    };//ArpPacketFormatType//

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
        }//PacketCacheItemType()//

        void operator=(PacketCacheItemType&& right)
        {
            packetPtr = move(right.packetPtr);
            nextHopAddress = right.nextHopAddress;
            priority = right.priority;
            etherType = right.etherType;
        }

        PacketCacheItemType(PacketCacheItemType&& right) { (*this) = move(right); }

    private:
        //PacketCacheItemType(const PacketCacheItemType&);
        //void operator=(const PacketCacheItemType&);
    };//PacketCacheItemType//

    class TimerEvent : public SimulationEvent {
    public:
        TimerEvent(const shared_ptr<AddressResolutionProtocol>& initArpPtr)
            : arpPtr(initArpPtr) {}
        void ExecuteEvent() { arpPtr->ProcessTimer(); }
    private:
        shared_ptr<AddressResolutionProtocol> arpPtr;
    };//TimerEvent//

    bool CanBeProxy(
        const NetworkAddress& senderIpAddress,
        const NetworkAddress& targetIpAddress) const;
    void SendRequest(const NetworkAddress& targetIpAddress);
    void SendReply(
        const NetworkAddress& targetIpAddress,
        const SixByteMacAddressType& targetMacAddress,
        const NetworkAddress *proxyIpAddress = nullptr);
    void SendProbe();
    void SendAnnounce();
    void SendArpPacket(
        const NetworkAddress& targetIpAddress,
        const ArpPacketFormatType& arpPacket);
    void NotifyCacheTableUpdated(
        const NetworkAddress& targetIpAddress);
    void NotifyProtocolAddressAssigned();
    bool HandleDuplicatedProtocolAddress(
        const NetworkAddress& senderIpAddress,
        const GenericMacAddressType& senderGenericMacAddress,
        const NetworkAddress& targetIpAddress);
    void ClearCache();
    void StartTimer(const TimeType& intervalTime);
    void StopTimer();
    void ProcessTimer();

    const string modelName;
    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    NodeIdType nodeId;
    unsigned int interfaceIndex;
    shared_ptr<NetworkLayer> networkLayerPtr;
    RandomNumberGenerator randomNumberGenerator;
    map<NetworkAddress, GenericMacAddressType> cacheTable;
    map<NetworkAddress, PacketCacheItemType> packetCache;
    enum AddressStateType addressState;
    enum TimerStateType timerState;
    shared_ptr<TimerEvent> timerEventPtr;
    EventRescheduleTicket timerEventTicket;
    bool proxyArp;
    TimeType probeWait;
    int probeNum;
    TimeType probeMin;
    TimeType probeMax;
    TimeType announceWait;
    int announceNum;
    TimeType announceInterval;
    int maxConflicts;
    TimeType rateLimitInterval;
    PacketPriorityType priority;
    int currentProbeNum;
    int currentAnnounceNum;
    int currentConflicts;

    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> fullQueueDropsStatPtr;
    shared_ptr<CounterStatistic> ipConflictsStatPtr;

};//AddressResolutionProtocol//

inline
AddressResolutionProtocol::AddressResolutionProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const NodeIdType& initNodeId,
    const InterfaceIdType& initInterfaceId,
    const unsigned int initInterfaceIndex,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const RandomNumberGeneratorSeedType& initNodeSeed)
    :
    modelName("Arp"),
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    nodeId(initNodeId),
    interfaceIndex(initInterfaceIndex),
    networkLayerPtr(initNetworkLayerPtr),
    randomNumberGenerator(HashInputsToMakeSeed(initNodeSeed, initInterfaceIndex, hashValue)),
    cacheTable(),
    packetCache(),
    addressState(NotAssigned),
    timerState(Idle),
    timerEventPtr(),
    timerEventTicket(),
    proxyArp(false),
    probeWait(1 * SECOND),
    probeNum(3),
    probeMin(1 * SECOND),
    probeMax(2 * SECOND),
    announceWait(2 * SECOND),
    announceNum(2),
    announceInterval(2 * SECOND),
    maxConflicts(10),
    rateLimitInterval(60 * SECOND),
    priority(0),
    currentProbeNum(0),
    currentAnnounceNum(0),
    currentConflicts(0),
    packetsSentStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initInterfaceId + "_PacketsSent"))),
    packetsReceivedStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initInterfaceId + "_PacketsReceived"))),
    fullQueueDropsStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initInterfaceId + "_FullQueueDrops"))),
    ipConflictsStatPtr(
        initSimEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initInterfaceId + "_IpConflicts")))
{
    if (!theParameterDatabaseReader.ParameterExists("network-enable-arp", nodeId, initInterfaceId) ||
        !theParameterDatabaseReader.ReadBool("network-enable-arp", nodeId, initInterfaceId)) {
        cerr << "Error: network-enable-arp(false) should be true" << endl;
        abort();
    }//if//
    if (theParameterDatabaseReader.ParameterExists("network-enable-proxy-arp", nodeId, initInterfaceId)) {
        proxyArp = theParameterDatabaseReader.ReadBool("network-enable-proxy-arp", nodeId, initInterfaceId);
    }//if//
    if (theParameterDatabaseReader.ParameterExists("network-arp-probe-wait", nodeId, initInterfaceId)) {
        probeWait = theParameterDatabaseReader.ReadTime("network-arp-probe-wait", nodeId, initInterfaceId);
    }//if//
    if (theParameterDatabaseReader.ParameterExists("network-arp-probe-num", nodeId, initInterfaceId)) {
        probeNum = theParameterDatabaseReader.ReadInt("network-arp-probe-num", nodeId, initInterfaceId);
    }//if//
    if (probeNum <= 0) {
        cerr << "Error: network-arp-probe-num(" << probeNum << ")";
        cerr << " should be greater than 0" << endl;
        abort();
    }//if//
    if (theParameterDatabaseReader.ParameterExists("network-arp-probe-min", nodeId, initInterfaceId)) {
        probeMin = theParameterDatabaseReader.ReadTime("network-arp-probe-min", nodeId, initInterfaceId);
    }//if//
    if (theParameterDatabaseReader.ParameterExists("network-arp-probe-max", nodeId, initInterfaceId)) {
        probeMax = theParameterDatabaseReader.ReadTime("network-arp-probe-max", nodeId, initInterfaceId);
    }//if//
    if (probeMin > probeMax) {
        cerr << "Error: network-arp-probe-max(" << probeMax << ")";
        cerr << " should be greater than or equal to";
        cerr << " network-arp-probe-min(" << probeMin << ")" << endl;
        abort();
    }//if//
    if (theParameterDatabaseReader.ParameterExists("network-arp-announce-wait", nodeId, initInterfaceId)) {
        announceWait = theParameterDatabaseReader.ReadTime("network-arp-announce-wait", nodeId, initInterfaceId);
    }//if//
    if (theParameterDatabaseReader.ParameterExists("network-arp-announce-num", nodeId, initInterfaceId)) {
        announceNum = theParameterDatabaseReader.ReadInt("network-arp-announce-num", nodeId, initInterfaceId);
    }//if//
    if (announceNum <= 0) {
        cerr << "Error: network-arp-announce-num(" << announceNum << ")";
        cerr << " should be greater than 0" << endl;
        abort();
    }//if//
    if (theParameterDatabaseReader.ParameterExists("network-arp-announce-interval", nodeId, initInterfaceId)) {
        announceInterval = theParameterDatabaseReader.ReadTime("network-arp-announce-interval", nodeId, initInterfaceId);
    }//if//
    if (theParameterDatabaseReader.ParameterExists("network-arp-max-conflicts", nodeId, initInterfaceId)) {
        maxConflicts = theParameterDatabaseReader.ReadInt("network-arp-max-conflicts", nodeId, initInterfaceId);
    }//if//
    if (maxConflicts < 0) {
        cerr << "Error: network-arp-announce-num(" << maxConflicts << ")";
        cerr << " should be greater than or equal to 0" << endl;
        abort();
    }//if//
    if (theParameterDatabaseReader.ParameterExists("network-arp-rate-limit-interval", nodeId, initInterfaceId)) {
        rateLimitInterval = theParameterDatabaseReader.ReadTime("network-arp-rate-limit-interval", nodeId, initInterfaceId);
    }//if//
    if (theParameterDatabaseReader.ParameterExists("network-arp-packet-priority", nodeId, initInterfaceId)) {
        priority = static_cast<PacketPriorityType>(
            theParameterDatabaseReader.ReadInt("network-arp-packet-priority", nodeId, initInterfaceId));
    }//if//

}//AddressResolutionProtocol//

inline
AddressResolutionProtocol::~AddressResolutionProtocol()
{
    map<NetworkAddress, PacketCacheItemType>::iterator iter;

    for (iter = packetCache.begin(); iter != packetCache.end(); ++iter) {
        assert(iter->second.packetPtr != nullptr);
        iter->second.packetPtr = nullptr;
    }//for//

}//~AddressResolutionProtocol//

inline
void AddressResolutionProtocol::NotifyProtocolAddressChanged()
{
    ClearCache();

    addressState = NotAssigned;
    currentProbeNum = 0;
    currentAnnounceNum = 0;

    const NetworkAddress senderIpAddress =
        networkLayerPtr->GetNetworkAddress(interfaceIndex);

    if (senderIpAddress == NetworkAddress::invalidAddress) {
        timerState = Idle;
        StopTimer();
    }
    else {
        timerState = Probe;
        if (currentConflicts >= maxConflicts) {
            StartTimer(rateLimitInterval);
        }
        else {
            const TimeType intervalTime = static_cast<TimeType>(
                randomNumberGenerator.GenerateRandomDouble() * probeWait);
            StartTimer(intervalTime);
        }//if//
    }//if//

}//NotifyProtocolAddressChanged//

inline
void AddressResolutionProtocol::SendArpRequest(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType& priority,
    const EtherTypeFieldType etherType)
{
    assert(!HardwareAddressIsInTable(nextHopAddress));

    map<NetworkAddress, PacketCacheItemType>::iterator iter =
        packetCache.find(nextHopAddress);
    if (iter != packetCache.end()) {
        if (iter->second.packetPtr != nullptr) {
            iter->second.packetPtr = nullptr;
            fullQueueDropsStatPtr->IncrementCounter();
        }//if//
        packetCache.erase(iter);
    }//if//

    PacketCacheItemType packetCacheItem(packetPtr, nextHopAddress, priority, etherType);
    packetCache.insert(
        move(pair<NetworkAddress, PacketCacheItemType>(nextHopAddress, move(packetCacheItem))));

    SendRequest(nextHopAddress);

}//CreateArpRequestPacket//

inline
void AddressResolutionProtocol::ReceiveArpPacket(const Packet& aPacket)
{
    packetsReceivedStatPtr->IncrementCounter();

    const ArpPacketFormatType& arpPacket =
        aPacket.GetAndReinterpretPayloadData<ArpPacketFormatType>();

    assert(NetToHost16(arpPacket.hardwareType) == Ethernet);
    assert(NetToHost16(arpPacket.protocolType) == Ip);

    const uint16_t operationType = NetToHost16(arpPacket.operationType);
    const NetworkAddress senderIpAddress = arpPacket.GetSenderProtocolAddress();
    const GenericMacAddressType senderGenericMacAddress = arpPacket.GetSenderHardwareAddress();
    const NetworkAddress targetIpAddress = arpPacket.GetTargetProtocolAddress();

    if (HandleDuplicatedProtocolAddress(senderIpAddress, senderGenericMacAddress, targetIpAddress)) {
        return;
    }//if//

    bool isMerged = false;

    map<NetworkAddress, GenericMacAddressType>::iterator iter =
        cacheTable.find(senderIpAddress);
    if (iter != cacheTable.end()) {
        assert(!senderIpAddress.IsAnyAddress());
        iter->second = senderGenericMacAddress;
        isMerged = true;
    }//if//

    const bool toMyself = (networkLayerPtr->GetNetworkAddress(interfaceIndex) == targetIpAddress);
    const bool canBeProxy = CanBeProxy(senderIpAddress, targetIpAddress);
    if (toMyself || canBeProxy) {
        if (!isMerged && !senderIpAddress.IsAnyAddress()) {
            cacheTable.insert(make_pair(senderIpAddress, senderGenericMacAddress));
            isMerged = true;
        }//if//
        if (operationType == ArpRequest) {
            const SixByteMacAddressType senderMacAddress(senderGenericMacAddress);
            if (toMyself) {
                SendReply(senderIpAddress, senderMacAddress);
            }
            else {
                SendReply(senderIpAddress, senderMacAddress, &targetIpAddress);
            }//if//
        }//if//
    }//if//

    if (isMerged) {
        NotifyCacheTableUpdated(senderIpAddress);
    }//if//

}//ReceiveArpPacket//



inline
void AddressResolutionProtocol::LookupHardwareAddress(
    const NetworkAddress& nextHopAddress,
    bool& wasFound,
    GenericMacAddressType& macAddress) const
{
    wasFound = false;

    if (nextHopAddress.IsTheBroadcastAddress()) {
        macAddress = SixByteMacAddressType::GetBroadcastAddress().ConvertToGenericMacAddress();
        wasFound = true;
        return;
    }//if//

    if (nextHopAddress.IsAMulticastAddress()) {

        SixByteMacAddressType sixByteMacAddress;
        sixByteMacAddress.SetToAMulticastAddress(nextHopAddress.GetMulticastGroupNumber());

        macAddress = sixByteMacAddress.ConvertToGenericMacAddress();
        wasFound = true;
        return;
    }//if//

    map<NetworkAddress, GenericMacAddressType>::const_iterator iter =
        cacheTable.find(nextHopAddress);

    if (iter != cacheTable.end()) {
        macAddress = iter->second;
        wasFound = true;
    }//if//

}//LookupHardwareAddress//


inline
void AddressResolutionProtocol::LookupProtocolAddress(
    const GenericMacAddressType& macAddress,
    bool& wasFound,
    NetworkAddress& nextHopAddress) const
{
    wasFound = false;

    map<NetworkAddress, GenericMacAddressType>::const_iterator iter;

    for (iter = cacheTable.begin(); iter != cacheTable.end(); ++iter) {
        if (iter->second == macAddress) {
            nextHopAddress = iter->first;
            wasFound = true;
            break;
        }//if//
    }//for//
}//LookupProtocolAddress//


inline
bool AddressResolutionProtocol::CanBeProxy(
    const NetworkAddress& senderIpAddress,
    const NetworkAddress& targetIpAddress) const
{
    if (!proxyArp) {
        return false;
    }//if//

    const NetworkAddress ipAddress =
        networkLayerPtr->GetNetworkAddress(interfaceIndex);
    const unsigned int subnetMaskBitLength =
        networkLayerPtr->GetSubnetMaskBitLength(interfaceIndex);
    const NetworkAddress subnetMask =
        NetworkAddress::MakeSubnetMask(subnetMaskBitLength);

    const bool isProbe = senderIpAddress.IsAnyAddress();
    const bool isAnnounce = (senderIpAddress == targetIpAddress);
    const bool isInSameSubnet =
        ipAddress.IsInSameSubnetAs(targetIpAddress, subnetMask);

    shared_ptr<RoutingTable> routingTable =
        networkLayerPtr->GetRoutingTableInterface();

    bool foundRoute;
    NetworkAddress nextHopAddress;
    unsigned int nextHopInterfaceIndex;

    routingTable->LookupRoute(
        targetIpAddress,
        foundRoute,
        nextHopAddress,
        nextHopInterfaceIndex);

    return (!isProbe && !isAnnounce && (isInSameSubnet || foundRoute));

}//CanBeProxy//

inline
void AddressResolutionProtocol::SendRequest(
    const NetworkAddress& targetIpAddress)
{
    if (addressState != Assigned) {
        return;
    }//if//

    const NetworkAddress senderIpAddress =
        networkLayerPtr->GetNetworkAddress(interfaceIndex);

    const GenericMacAddressType genericMacAddress =
        networkLayerPtr->GetMacLayerPtr(interfaceIndex)->GetGenericMacAddress();
    const SixByteMacAddressType senderMacAddress(genericMacAddress);

    ArpPacketFormatType arpPacket(
        ArpRequest,
        senderIpAddress,
        senderMacAddress,
        targetIpAddress,
        SixByteMacAddressType::invalidMacAddress);

    SendArpPacket(
        NetworkAddress::broadcastAddress,
        arpPacket);

}//SendRequest//

inline
void AddressResolutionProtocol::SendReply(
    const NetworkAddress& targetIpAddress,
    const SixByteMacAddressType& targetMacAddress,
    const NetworkAddress *proxyIpAddress)
{
    if (addressState != Assigned) {
        return;
    }//if//

    NetworkAddress senderIpAddress;

    if (proxyIpAddress) {
        senderIpAddress = *proxyIpAddress;
    }
    else {
        senderIpAddress = networkLayerPtr->GetNetworkAddress(interfaceIndex);
    }//if//

    const GenericMacAddressType genericMacAddress =
        networkLayerPtr->GetMacLayerPtr(interfaceIndex)->GetGenericMacAddress();
    const SixByteMacAddressType senderMacAddress(genericMacAddress);

    ArpPacketFormatType arpPacket(
        ArpReply,
        senderIpAddress,
        senderMacAddress,
        targetIpAddress,
        targetMacAddress);

    SendArpPacket(
        targetIpAddress,
        arpPacket);

}//SendReply//

inline
void AddressResolutionProtocol::SendProbe()
{
    if (timerState != Probe || addressState == Assigned) {
        assert(false); abort();
    }//if//

    const NetworkAddress senderIpAddress =
        networkLayerPtr->GetNetworkAddress(interfaceIndex);

    const GenericMacAddressType genericMacAddress =
        networkLayerPtr->GetMacLayerPtr(interfaceIndex)->GetGenericMacAddress();
    const SixByteMacAddressType senderMacAddress(genericMacAddress);

    ArpPacketFormatType arpPacket(
        ArpRequest,
        NetworkAddress::invalidAddress,
        senderMacAddress,
        senderIpAddress,
        SixByteMacAddressType::invalidMacAddress);

    SendArpPacket(
        NetworkAddress::broadcastAddress,
        arpPacket);

}//SendProbe//

inline
void AddressResolutionProtocol::SendAnnounce()
{
    if (timerState != Announce || addressState != Assigned) {
        assert(false); abort();
    }//if//

    const NetworkAddress senderIpAddress =
        networkLayerPtr->GetNetworkAddress(interfaceIndex);

    const GenericMacAddressType genericMacAddress =
        networkLayerPtr->GetMacLayerPtr(interfaceIndex)->GetGenericMacAddress();
    const SixByteMacAddressType senderMacAddress(genericMacAddress);

    ArpPacketFormatType arpPacket(
        ArpRequest,
        senderIpAddress,
        senderMacAddress,
        senderIpAddress,
        SixByteMacAddressType::invalidMacAddress);

    SendArpPacket(
        NetworkAddress::broadcastAddress,
        arpPacket);

}//SendAnnounce//

inline
void AddressResolutionProtocol::SendArpPacket(
    const NetworkAddress& targetIpAddress,
    const ArpPacketFormatType& arpPacket)
{
    unique_ptr<Packet> packetPtr = Packet::CreatePacket(*simEngineInterfacePtr, arpPacket);

    networkLayerPtr->InsertPacketIntoAnOutputQueue(
        packetPtr, interfaceIndex, targetIpAddress, priority, Arp);

    packetsSentStatPtr->IncrementCounter();

}//SendArpPacket//

inline
void AddressResolutionProtocol::NotifyCacheTableUpdated(
    const NetworkAddress& targetIpAddress)
{
    map<NetworkAddress, PacketCacheItemType>::iterator iter =
        packetCache.find(targetIpAddress);

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

inline
void AddressResolutionProtocol::NotifyProtocolAddressAssigned()
{
    typedef map<NetworkAddress, PacketCacheItemType>::iterator IterType;

    IterType iter = packetCache.begin();

    while (iter != packetCache.end()) {
        if (HardwareAddressIsInTable(iter->first)) {
            assert(iter->second.packetPtr != nullptr);
            networkLayerPtr->InsertPacketIntoAnOutputQueue(
                iter->second.packetPtr,
                interfaceIndex,
                iter->second.nextHopAddress,
                iter->second.priority,
                iter->second.etherType);

            const IterType deleteIter = iter;
            iter++;
            packetCache.erase(deleteIter);
        }
        else {
            SendRequest(iter->first);
            ++iter;
        }//if//
    }//while//

}//NotifyProtocolAddressAssigned//

inline
bool AddressResolutionProtocol::HandleDuplicatedProtocolAddress(
    const NetworkAddress& senderIpAddress,
    const GenericMacAddressType& senderGenericMacAddress,
    const NetworkAddress& targetIpAddress)
{
    const NetworkAddress ipAddress =
        networkLayerPtr->GetNetworkAddress(interfaceIndex);

    const GenericMacAddressType genericMacAddress =
        networkLayerPtr->GetMacLayerPtr(interfaceIndex)->GetGenericMacAddress();

    switch (addressState) {
    case NotAssigned:
        switch (timerState) {
        case Idle:
            break;
        case Probe:
            if ((ipAddress == targetIpAddress) &&
                (senderIpAddress == NetworkAddress::invalidAddress)) {
                ++currentConflicts;
                ipConflictsStatPtr->IncrementCounter();
                timerState = Idle;
                StopTimer();
            }//if//
            break;
        default:
            assert(false); abort();
        }//switch//
        return true;
    case Assigned:
        switch (timerState) {
        case Idle:
        case Announce:
            if ((ipAddress == senderIpAddress) &&
                (genericMacAddress != senderGenericMacAddress)) {
                addressState = Duplicated;
                ++currentConflicts;
                ipConflictsStatPtr->IncrementCounter();
                timerState = Idle;
                StopTimer();
                return true;
            }//if//
            break;
        default:
            assert(false); abort();
        }//switch//
        break;
    case Duplicated:
        switch (timerState) {
        case Idle:
            if (ipAddress == senderIpAddress) {
                ++currentConflicts;
                ipConflictsStatPtr->IncrementCounter();
            }//if//
            break;
        default:
            assert(false); abort();
        }//switch//
        return true;
    default:
        assert(false); abort();
    }//switch//

    return false;

}//HandleDuplicatedProtocolAddress//

inline
void AddressResolutionProtocol::ClearCache()
{
    cacheTable.clear();

}//ClearCache//

inline
void AddressResolutionProtocol::StartTimer(
    const TimeType& intervalTime)
{
    if (timerEventPtr == nullptr) {
        timerEventPtr = shared_ptr<TimerEvent>(new TimerEvent(shared_from_this()));
    }//if//

    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    if (timerEventTicket.IsNull()) {
        simEngineInterfacePtr->ScheduleEvent(
            timerEventPtr,
            currentTime + intervalTime,
            timerEventTicket);
    }
    else {
        simEngineInterfacePtr->RescheduleEvent(
            timerEventTicket,
            currentTime + intervalTime);
    }//if//

}//StartTimer//

inline
void AddressResolutionProtocol::StopTimer()
{
    if (!timerEventTicket.IsNull()) {
        simEngineInterfacePtr->CancelEvent(timerEventTicket);
        timerEventTicket.Clear();
    }//if//

}//StopTimer//

inline
void AddressResolutionProtocol::ProcessTimer()
{
    timerEventTicket.Clear();

    switch (addressState) {
    case NotAssigned:
        switch (timerState) {
        case Probe:
            if (currentProbeNum == probeNum) {
                addressState = Assigned;
                NotifyProtocolAddressAssigned();
                timerState = Announce;
                StartTimer(announceInterval);
                ++currentAnnounceNum;
                SendAnnounce();
            }
            else {
                ++currentProbeNum;
                SendProbe();
                if (currentProbeNum == probeNum) {
                    StartTimer(announceWait);
                }
                else {
                    if (currentConflicts >= maxConflicts) {
                        StartTimer(rateLimitInterval);
                    }
                    else {
                        const TimeType intervalTime = static_cast<TimeType>(
                            randomNumberGenerator.GenerateRandomDouble() * (probeMax - probeMin));
                        StartTimer(intervalTime);
                    }//if//
                }//if//
            }//if//
            break;
        default:
            assert(false); abort();
        }//switch//
        break;
    case Assigned:
        switch (timerState) {
        case Announce:
            if (currentAnnounceNum == announceNum) {
                timerState = Idle;
                StopTimer();
            }
            else {
                ++currentAnnounceNum;
                SendAnnounce();
                StartTimer(announceInterval);
            }//if//
            break;
        default:
            assert(false); abort();
        }//switch//
        break;
    default:
        assert(false); abort();
    }//switch//

}//ProcessTimer//

class ArpMacAddressResolver : public MacAddressResolver<SixByteMacAddressType> {
public:
    ArpMacAddressResolver(
        const shared_ptr<AddressResolutionProtocol> initArpPtr)
        :
        arpPtr(initArpPtr)
    {}

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
    shared_ptr<AddressResolutionProtocol> arpPtr;

};//ArpMacAddressResolver//

inline
void ArpMacAddressResolver::GetMacAddress(
    const NetworkAddress& aNetworkAddress,
    const NetworkAddress& networkAddressMask,
    bool& wasFound,
    SixByteMacAddressType& resolvedMacAddress)
{
    GenericMacAddressType genericMacAddress;
    arpPtr->LookupHardwareAddress(aNetworkAddress, wasFound, genericMacAddress);
    if (wasFound) {
        resolvedMacAddress = SixByteMacAddressType(genericMacAddress);
    }//if//

}//GetMacAddress//

inline
void ArpMacAddressResolver::GetNetworkAddressIfAvailable(
    const SixByteMacAddressType& macAddress,
    const NetworkAddress& subnetNetworkAddress,
    bool& wasFound,
    NetworkAddress& resolvedNetworkAddress)
{
    GenericMacAddressType genericMacAddress = macAddress.ConvertToGenericMacAddress();
    arpPtr->LookupProtocolAddress(genericMacAddress, wasFound, resolvedNetworkAddress);

}//GetNetworkAddressIfAvailable//

}//namespace//

#endif
