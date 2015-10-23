// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_NETWORK_H
#define SCENSIM_NETWORK_H

#include "scensim_bercurves.h"
#include "scensim_ip_model.h"
#include "scensim_mobility.h"
#include "scensim_mac.h"
#include "scensim_tracedefs.h"

namespace ScenSim {

using std::istringstream;
using std::unique_ptr;
using std::move;


const unsigned char IP_PROTOCOL_NUMBER_ICMP = 1;
const unsigned char IP_PROTOCOL_NUMBER_TCP = 6;
const unsigned char IP_PROTOCOL_NUMBER_UDP = 17;
const unsigned char IP_PROTOCOL_NUMBER_AODV = 143;
const unsigned char IP_PROTOCOL_NUMBER_OLSR = 144;
const unsigned char IP_PROTOCOL_NUMBER_OLSRV2 = 145;
//
//

class ProtocolPacketHandler;
class SensingSubsystemInterface;

////==================================================================================================
//
class AccessPointFinderInterface {
public:
    virtual void LookupAccessPointFor(
        const NetworkAddress& destinationAddress,
        bool& foundTheAccessPoint,
        NetworkAddress& accessPointAddress) = 0;

    virtual void SetAccessPointFor(
        const NetworkAddress& nodeAddress,
        const NetworkAddress& accessPointAddress) = 0;

    virtual void ClearAccessPointFor(const NetworkAddress& nodeAddress) = 0;

};//AccessPointFinderInterface//


class NetworkAddressLookupInterface;

// GlobalNetworkingObjects is just a parameter passing convenience "bag" type.

struct GlobalNetworkingObjectBag {

    shared_ptr<NetworkAddressLookupInterface> networkAddressLookupInterfacePtr;
    shared_ptr<AbstractNetwork> abstractNetworkPtr;
    shared_ptr<AccessPointFinderInterface> extraSimulationAccessPointFinderPtr;
    shared_ptr<BitOrBlockErrorRateCurveDatabase> bitOrBlockErrorRateCurveDatabasePtr;
    shared_ptr<AntennaPatternDatabase> antennaPatternDatabasePtr;
    shared_ptr<SensingSubsystemInterface> sensingSubsystemInterfacePtr;

};//GlobalNetworkingObjectBag//

class AbstractNetwork: public enable_shared_from_this<AbstractNetwork> {
public:
    AbstractNetwork(const ParameterDatabaseReader& theParameterDatabaseReader);

    void CreateAnAbstractNetworkMac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const unsigned int macInterfaceIndex,
        const RandomNumberGeneratorSeedType& nodeSeed,
        shared_ptr<AbstractNetworkMac>& macPtr);

    shared_ptr<AbstractNetworkMac> GetDestinationMacPtr(const NetworkAddress destinationAddress);
    void GetDestinationMacPtrsForBroadcast(
        const NetworkAddress sourceAddress,
        const NetworkAddress sourceMaskAddress,
        list<shared_ptr<AbstractNetworkMac> >& macPtrs);

private:
    map<NetworkAddress, shared_ptr<AbstractNetworkMac> > addressToMacMap;

    //Disable:
    AbstractNetwork(AbstractNetwork&);
    void operator=(AbstractNetwork&);

};//AbstractNetwork//


//--------------------------------------------------------------------------------------------------
// Network Layer
//--------------------------------------------------------------------------------------------------


class RoutingTable {
public:
    RoutingTable() { }

    void AddOrUpdateRoute(
        const NetworkAddress& destinationAddress,
        const NetworkAddress& nextHopAddress,
        const unsigned int nextHopInterfaceIndex);

    void AddOrUpdateRoute(
        const NetworkAddress& destinationAddress,
        const NetworkAddress& destinationAddressSubnetMask,
        const NetworkAddress& nextHopAddress,
        const unsigned int nextHopInterfaceIndex);

    void DeleteRoute(const NetworkAddress& destinationAddress)
        { noMaskRoutingTable.erase(destinationAddress); }

    void DeleteRoute(
        const NetworkAddress& destinationAddress,
        const NetworkAddress& destinationAddressSubnetMask);

    void LookupRoute(
        const NetworkAddress& destinationAddress,
        bool& foundRoute,
        NetworkAddress& nextHopAddress,
        unsigned int& nextHopInterfaceIndex) const;

private:
    struct NoMaskRoutingTableEntry {
        NetworkAddress nextHopAddress;
        unsigned int nextHopInterfaceIndex;

        NoMaskRoutingTableEntry() : nextHopAddress(NetworkAddress(0)), nextHopInterfaceIndex(0) { }

        NoMaskRoutingTableEntry(const NetworkAddress& initNextHopAddress, const unsigned int initNextHopInterfaceIndex)
            : nextHopAddress(initNextHopAddress), nextHopInterfaceIndex(initNextHopInterfaceIndex)
        {}
    };

    map<NetworkAddress, NoMaskRoutingTableEntry> noMaskRoutingTable;

    struct GeneralRoutingTableEntry {
        NetworkAddress destinationSubnetAddress;
        NetworkAddress destinationSubnetMask;
        NetworkAddress nextHopAddress;
        unsigned int nextHopInterfaceIndex;
        //int entryMetric;

        bool operator<(const GeneralRoutingTableEntry& right) const
        {
            return (destinationSubnetMask > right.destinationSubnetMask);
        }

        GeneralRoutingTableEntry(
            const NetworkAddress initDestinationSubnetAddress,
            const NetworkAddress initDestinationSubnetMask,
            const NetworkAddress initNextHopAddress,
            const unsigned int initNextHopInterfaceIndex)
            :
            destinationSubnetAddress(initDestinationSubnetAddress),
            destinationSubnetMask(initDestinationSubnetMask),
            nextHopAddress(initNextHopAddress),
            nextHopInterfaceIndex(initNextHopInterfaceIndex)
        {}

    };//GeneralRoutingTableEntry//

    vector<GeneralRoutingTableEntry> generalRoutingTable;

    void FindGeneralRouteEntry(
        const NetworkAddress& destinationSubnetAddress,
        const NetworkAddress& destinationSubnetMask,
        bool& foundEntry,
        unsigned int& entryIndex);

    //Disabled:

    RoutingTable(const RoutingTable&);
    void operator=(const RoutingTable&);

};//RoutingTable//


inline
void RoutingTable::AddOrUpdateRoute(
    const NetworkAddress& destinationAddress,
    const NetworkAddress& nextHopAddress,
    const unsigned int nextHopInterfaceIndex)
{
    noMaskRoutingTable[destinationAddress] = NoMaskRoutingTableEntry(nextHopAddress, nextHopInterfaceIndex);
}



inline
void RoutingTable::FindGeneralRouteEntry(
    const NetworkAddress& destinationSubnetAddress,
    const NetworkAddress& destinationSubnetMask,
    bool& foundEntry,
    unsigned int& entryIndex)
{
    for(unsigned int i = 0; (i < generalRoutingTable.size()); i++) {

        if ((generalRoutingTable[i].destinationSubnetAddress == destinationSubnetAddress) &&
            (generalRoutingTable[i].destinationSubnetMask == destinationSubnetMask)) {

            foundEntry = true;
            entryIndex = i;
            return;

        }//if//
    }//for//

    foundEntry = false;

}//FindGeneralRouteEntry//


inline
void RoutingTable::AddOrUpdateRoute(
    const NetworkAddress& destinationAddress,
    const NetworkAddress& destinationSubnetMask,
    const NetworkAddress& nextHopAddress,
    const unsigned int nextHopInterfaceIndex)
{
    bool entryWasFound;
    unsigned int entryIndex;
    FindGeneralRouteEntry(destinationAddress, destinationSubnetMask, entryWasFound, entryIndex);
    if (entryWasFound) {
        generalRoutingTable[entryIndex].nextHopAddress = nextHopAddress;
        generalRoutingTable[entryIndex].nextHopInterfaceIndex = nextHopInterfaceIndex;
    }
    else {
        generalRoutingTable.push_back(
            GeneralRoutingTableEntry(destinationAddress, destinationSubnetMask, nextHopAddress, nextHopInterfaceIndex));

        // Sort by network mask (larger (more exact) masks first).

        std::sort(generalRoutingTable.begin(), generalRoutingTable.end());

    }//if//

}//AddOrUpdateRoute//




inline
void RoutingTable::LookupRoute(
    const NetworkAddress& destinationAddress,
    bool& foundRoute,
    NetworkAddress& nextHopAddress,
    unsigned int& nextHopInterfaceIndex) const
{
    map<NetworkAddress, NoMaskRoutingTableEntry>::const_iterator iter =
        noMaskRoutingTable.find(destinationAddress);

    if (iter != noMaskRoutingTable.end()) {
        foundRoute = true;
        nextHopAddress = iter->second.nextHopAddress;
        nextHopInterfaceIndex = iter->second.nextHopInterfaceIndex;
        return;
    }//if/


    for(unsigned int i = 0; (i < generalRoutingTable.size()); i++) {

        if(destinationAddress.IsInSameSubnetAs(
               generalRoutingTable[i].destinationSubnetAddress,
               generalRoutingTable[i].destinationSubnetMask)) {

            foundRoute = true;
            nextHopAddress = generalRoutingTable[i].nextHopAddress;
            nextHopInterfaceIndex = generalRoutingTable[i].nextHopInterfaceIndex;
            return;
        }//if//

    }//for//

    foundRoute = false;

}//LookupRoute//


inline
void RoutingTable::DeleteRoute(
    const NetworkAddress& destinationAddress,
    const NetworkAddress& destinationSubnetMask)
{
    for(vector<GeneralRoutingTableEntry>::iterator i = generalRoutingTable.begin();
         (i != generalRoutingTable.end()); ++i) {

        if ((i->destinationSubnetAddress == destinationAddress) &&
                (i->destinationSubnetMask == destinationSubnetMask)) {
            generalRoutingTable.erase(i);
            return;
        }//if//
    }//for//

}//DeleteRoute//


class NetworkInterfaceStatusChangeNotificationInterface {
public:

    virtual ~NetworkInterfaceStatusChangeNotificationInterface() { }

    virtual void HandleMajorInterfaceStatusChange(const unsigned int interfaceIndex) = 0;

};//NetworkInterfaceStatusChangeNotificationInterface//





class OnDemandRoutingProtocolInterface {
public:
    virtual ~OnDemandRoutingProtocolInterface() { }

    virtual void HandlePacketWithNoRoute(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const NetworkAddress& lastHopAddress,
        bool& wasAccepted) { wasAccepted = false; }

    virtual void HandlePacketUndeliveredByMac(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destination,
        const unsigned int interfaceIndex,
        const NetworkAddress& nextHop,
        bool& wasAccepted) { wasAccepted = false; }

    // AODV
    virtual void InspectPacket(
        const NetworkAddress& lastHopAddress,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& nextAddress,
        const NetworkAddress& destinationAddress,
        const bool sourceIsThisNode) = 0;

    virtual bool NeedToInspectAllDataPackets() const = 0;

};//OnDemandRoutingProtocolInterface//


class DhcpClientInterface {
public:

    virtual ~DhcpClientInterface() { }

    virtual void HandleLinkIsUpNotification(
        const unsigned int interfaceIndex,
        const GenericMacAddressType& genericMacAddress) = 0;

    virtual void EnableForThisInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex) = 0;

};


class NetworkAddressInterface {
public:

    virtual ~NetworkAddressInterface() { }

    virtual void NotifyNetworkAddressIsChanged(
        const unsigned int interfaceIndex,
        const NetworkAddress& newInterfaceAddress,
        const unsigned int subnetMaskLengthBits) = 0;

};//NetworkAddressInterface//


class MobileIpMobileNodeSubsystem;
class MobileIpHomeAgentSubsystem;
class NetworkInterfaceManager;
class ApplicationLayer;
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


class NetworkLayer {
public:
    NetworkLayer() { }
    virtual ~NetworkLayer() { }

    virtual void DisconnectFromOtherLayers() = 0;

    virtual NodeIdType GetNodeId() const = 0;

    virtual shared_ptr<RoutingTable> GetRoutingTableInterface() = 0;

    virtual NetworkAddress GetPrimaryNetworkAddress() const = 0;

    virtual unsigned int NumberOfInterfaces() const = 0;
    virtual unsigned int LookupInterfaceIndex(const NetworkAddress& interfaceAddress) const = 0;
    virtual unsigned int LookupInterfaceIndex(const InterfaceIdType& interfaceName) const = 0;

    virtual InterfaceIdType GetInterfaceId(const unsigned int interfaceIndex) const = 0;
    virtual NetworkAddress GetNetworkAddress(const unsigned int interfaceIndex) const = 0;
    virtual NetworkAddress GetSubnetAddress(const unsigned int interfaceIndex) const = 0;
    virtual NetworkAddress GetSubnetMask(const unsigned int interfaceIndex) const = 0;
    virtual unsigned int GetSubnetMaskBitLength(const unsigned int interfaceIndex) const = 0;
    virtual NetworkAddress MakeBroadcastAddressForInterface(const unsigned int interfaceIndex) const = 0;

    virtual void SetInterfaceIpAddress(
        const unsigned int interfaceIndex,
        const NetworkAddress& newInterfaceAddress,
        const unsigned int subnetMaskLengthBits) = 0;

    virtual void SetInterfaceGatewayAddress(
        const unsigned int interfaceIndex,
        const NetworkAddress& newGatewayAddress,
        const bool gatewayIsForcedNextHop = false) = 0;

    virtual void ClearInterfaceIpInformation(const unsigned int interfaceIndex) = 0;

    virtual void RegisterPacketHandlerForProtocol(
        const unsigned char protocolNum,
        const shared_ptr<ProtocolPacketHandler>& packetHandlerPtr) = 0;

    virtual void RegisterOnDemandRoutingProtocolInterface(
        const shared_ptr<OnDemandRoutingProtocolInterface>& interfacePtr) = 0;

    virtual void RegisterNetworkAddressInterface(
        const shared_ptr<NetworkAddressInterface>& interfacePtr) = 0;

    virtual void ReceivePacketFromUpperLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const unsigned char protocol) = 0;

    virtual void ReceivePacketFromUpperLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const unsigned char protocol) = 0;

    virtual void ReceiveOutgoingBroadcastPacket(
        unique_ptr<Packet>& packetPtr,
        const unsigned int outgoingInterfaceIndex,
        const PacketPriorityType trafficClass,
        const unsigned char protocol) = 0;

    virtual void ReceiveOutgoingPreformedNetworkPacket(unique_ptr<Packet>& packetPtr) = 0;

    virtual void ReceiveRoutedNetworkPacketFromRoutingProtocol(
        unique_ptr<Packet>& packetPtr,
        const unsigned int interfaceIndex,
        const NetworkAddress& nextHopAddress) = 0;

    virtual void GetInterfaceIndexForOneHopDestination(
        const NetworkAddress& destinationAddress,
        bool& success,
        unsigned int& interfaceIndex) const = 0;

    virtual void GetNextHopAddressAndInterfaceIndexForDestination(
        const NetworkAddress& destinationAddress,
        bool& success,
        NetworkAddress& nextHopAddress,
        unsigned int& interfaceIndex) const = 0;

    virtual NetworkAddress GetSourceAddressForDestination(const NetworkAddress& destinationAddress) const = 0;

    virtual void GetNextHopAddressAndInterfaceIndexForNetworkPacket(
        const Packet& aPacket,
        bool& success,
        NetworkAddress& nextHopAddress,
        unsigned int& interfaceIndex) const = 0;

    virtual void ReceivePacketFromMac(
        const unsigned int macIndex,
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& lastHopAddress,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED) = 0;

    virtual void ReceiveUndeliveredPacketFromMac(
        const unsigned int macIndex,
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress) = 0;

    virtual void SetInterfaceMacLayer(const unsigned int interfaceIndex, const shared_ptr<MacLayer>& macLayerPtr) = 0;

    virtual shared_ptr<MacLayer> GetMacLayerPtr(const unsigned int interfaceIndex) const = 0;

    virtual shared_ptr<NetworkInterfaceManager> GetNetworkInterfaceManagerPtr(const unsigned int interfaceIndex) const = 0;

    virtual void SetInterfaceOutputQueue(
        const unsigned int interfaceIndex, const shared_ptr<InterfaceOutputQueue>& outputQueuePtr) = 0;

    virtual void ProcessLinkIsUpNotification(const unsigned int interfaceIndex) = 0;
    virtual void ProcessLinkIsDownNotification(const unsigned int interfaceIndex) = 0;
    virtual void ProcessNewLinkToANodeNotification(
        const unsigned int interfaceIndex,
        const GenericMacAddressType& macAddress) = 0;

    virtual bool MacSupportsQualityOfService(const unsigned int interfaceIndex) const = 0;

    virtual shared_ptr<MacQualityOfServiceControlInterface> GetMacQualityOfServiceInterface(
        const unsigned int interfaceIndex) const = 0;

    virtual void InsertPacketIntoAnOutputQueue(
        unique_ptr<Packet>& packetPtr,
        const unsigned int interfaceIndex,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType trafficClass,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED) = 0;

    virtual void SetupDhcpServerAndClientIfNessesary(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const NodeIdType& nodeId,
        const shared_ptr<ApplicationLayer>& appLayerPtr) = 0;

private:

    // Disable:

    NetworkLayer(const NetworkLayer&);
    void operator=(const NetworkLayer&);

};//NetworkLayer//


class BasicNetworkLayer: public NetworkLayer, public enable_shared_from_this<BasicNetworkLayer> {
public:
    static const string modelName;

    static shared_ptr<BasicNetworkLayer> CreateNetworkLayer(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjects,
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const RandomNumberGeneratorSeedType& nodeSeed);

    ~BasicNetworkLayer() { }

    void DisconnectFromOtherLayers();

    NodeIdType GetNodeId() const { return nodeId; }

    shared_ptr<RoutingTable> GetRoutingTableInterface() { return routingTablePtr; }

    bool IsANetworkAddressForThisNode(const NetworkAddress& anAddress) const;

    void CheckIfNetworkAddressIsForThisNode(
        const NetworkAddress& anAddress,
        bool& addressIsForThisNode,
        unsigned int& interfaceIndex) const;

    NetworkAddress GetPrimaryNetworkAddress() const { return primaryNetworkAddress; }

    NetworkAddress GetNetworkAddress(const unsigned int interfaceIndex) const
        { return networkInterfaces.at(interfaceIndex).address; }

    NetworkAddress GetSubnetAddress(const unsigned int interfaceIndex) const {
        return (
            networkInterfaces.at(interfaceIndex).address.MakeSubnetAddress(
                networkInterfaces.at(interfaceIndex).subnetMask));
    }

    NetworkAddress GetSubnetMask(const unsigned int interfaceIndex) const
        { return networkInterfaces.at(interfaceIndex).subnetMask; }

    unsigned int GetSubnetMaskBitLength(const unsigned int interfaceIndex) const
        { return networkInterfaces.at(interfaceIndex).subnetMaskLengthBits; }

    NetworkAddress MakeBroadcastAddressForInterface(const unsigned int interfaceIndex) const {
        return
            (NetworkAddress::MakeABroadcastAddress(
                GetNetworkAddress(interfaceIndex),
                GetSubnetMask(interfaceIndex)));
    }

    void SetInterfaceIpAddress(
        const unsigned int interfaceIndex,
        const NetworkAddress& newInterfaceAddress,
        const unsigned int subnetMaskLengthBits);

    void SetInterfaceGatewayAddress(
        const unsigned int interfaceIndex,
        const NetworkAddress& newGatewayAddress,
        const bool gatewayIsForcedNextHop);

    void ClearInterfaceIpInformation(const unsigned int interfaceIndex) { assert(false && "TBD"); }

    void RegisterPacketHandlerForProtocol(
        const unsigned char protocolNum,
        const shared_ptr<ProtocolPacketHandler>& packetHandlerPtr)
    {
        map<unsigned char, shared_ptr<ProtocolPacketHandler> >::const_iterator mapIter =
            protocolPacketHandlerMap.find(protocolNum);

        if (mapIter != protocolPacketHandlerMap.end()) {
            cerr << "Error: Dupulicated ProtocolNum= " << static_cast<int>(protocolNum) << endl;
            exit(1);
        }//if//

        protocolPacketHandlerMap.insert(make_pair(protocolNum, packetHandlerPtr));
    }

    void RegisterOnDemandRoutingProtocolInterface(
        const shared_ptr<OnDemandRoutingProtocolInterface>& interfacePtr)
    {
        onDemandRoutingProtocols.push_back(interfacePtr);
    }

    void RegisterNetworkAddressInterface(
        const shared_ptr<NetworkAddressInterface>& interfacePtr)
    {
        networkAddressInterfaces.push_back(interfacePtr);
    }

    void GetInterfaceIndexForOneHopDestination(
        const NetworkAddress& destinationAddress,
        bool& success,
        unsigned int& interfaceIndex) const;

    void GetNextHopAddressAndInterfaceIndexForDestination(
        const NetworkAddress& destinationAddress,
        bool& success,
        NetworkAddress& nextHopAddress,
        unsigned int& interfaceIndex) const;

    NetworkAddress GetSourceAddressForDestination(const NetworkAddress& destinationAddress) const;

    void GetNextHopAddressAndInterfaceIndexForNetworkPacket(
        const Packet& aPacket,
        bool& success,
        NetworkAddress& nextHopAddress,
        unsigned int& interfaceIndex) const;

    unsigned int LookupInterfaceIndex(const NetworkAddress& interfaceAddress) const;

    unsigned int LookupInterfaceIndex(const InterfaceIdType& interfaceName) const;

    InterfaceIdType GetInterfaceId(const unsigned int interfaceIndex) const
        { return networkInterfaces.at(interfaceIndex).interfaceName; }

    virtual unsigned int NumberOfInterfaces() const
        {return static_cast<unsigned int>(networkInterfaces.size());}

    void SetInterfaceMacLayer(const unsigned int interfaceIndex, const shared_ptr<MacLayer>& macLayerPtr);

    void ReceivePacketFromUpperLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const unsigned char protocol);

    void ReceivePacketFromUpperLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const unsigned char protocol);

    void ReceiveOutgoingBroadcastPacket(
        unique_ptr<Packet>& packetPtr,
        const unsigned int outgoingInterfaceIndex,
        const PacketPriorityType trafficClass,
        const unsigned char protocol);

    void ReceiveOutgoingPreformedNetworkPacket(unique_ptr<Packet>& packetPtr);

    void ReceiveRoutedNetworkPacketFromRoutingProtocol(
        unique_ptr<Packet>& packetPtr,
        const unsigned int interfaceIndex,
        const NetworkAddress& nextHopAddress);


    void ReceivePacketFromMac(
        const unsigned int macIndex,
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& lastHopAddress,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED);

    void ReceiveUndeliveredPacketFromMac(
        const unsigned int macIndex,
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress);

    shared_ptr<InterfaceOutputQueue> GetInterfaceOutputQueue(const unsigned int interfaceIndex) const
        { return networkInterfaces.at(interfaceIndex).outputQueuePtr; }

    shared_ptr<MacLayer> GetMacLayerPtr(const unsigned int interfaceIndex) const
        { return (networkInterfaces.at(interfaceIndex).macLayerPtr); }

    shared_ptr<NetworkInterfaceManager> GetNetworkInterfaceManagerPtr(const unsigned int interfaceIndex) const
        { return networkInterfaces.at(interfaceIndex).networkInterfaceManagerPtr; }

    void SetInterfaceOutputQueue(
        const unsigned int interfaceIndex,
        const shared_ptr<InterfaceOutputQueue>& outputQueuePtr)
    {
        networkInterfaces.at(interfaceIndex).outputQueuePtr = outputQueuePtr;
    }

    void ProcessLinkIsUpNotification(const unsigned int interfaceIndex);
    void ProcessLinkIsDownNotification(const unsigned int interfaceIndex);
    void ProcessNewLinkToANodeNotification(
        const unsigned int interfaceIndex,
        const GenericMacAddressType& newNodeMacAddress);

    bool MacSupportsQualityOfService(const unsigned int interfaceIndex) const
        { return (GetMacQualityOfServiceInterface(interfaceIndex) != nullptr); }


    shared_ptr<MacQualityOfServiceControlInterface> GetMacQualityOfServiceInterface(
        const unsigned int interfaceIndex) const;

    void InsertPacketIntoAnOutputQueue(
        unique_ptr<Packet>& packetPtr,
        const unsigned int interfaceIndex,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType trafficClass,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED);

    void SetupDhcpServerAndClientIfNessesary(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const NodeIdType& nodeId,
        const shared_ptr<ApplicationLayer>& appLayerPtr);

private:
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    NodeIdType nodeId;
    NetworkAddress primaryNetworkAddress;

    shared_ptr<AccessPointFinderInterface> extraSimulationAccessPointFinderPtr;

    map<unsigned char, shared_ptr<ProtocolPacketHandler> > protocolPacketHandlerMap;

    static const unsigned int DefaultMaxIpPacketSizeAkaMtuBytes = 1500;

    //Reference: http://www.iana.org/assignments/ip-parameters/ip-parameters.xml
    static const unsigned char DEFAULT_HOP_LIMIT = 64;

    struct NetworkInterfaceInfoType {
        InterfaceIdType interfaceName;
        NetworkAddress address;
        NetworkAddress subnetMask;
        unsigned int subnetMaskLengthBits;
        NetworkAddress gatewayAddress;
        bool gatewayIsForcedNextHop;
        bool subnetIsMultiHop;
        bool isPrimary;
        bool allowRoutingBackOutSameInterface;
        bool ignoreUnregisteredProtocol;
        shared_ptr<InterfaceOutputQueue> outputQueuePtr;
        shared_ptr<MacLayer> macLayerPtr;
        shared_ptr<NetworkInterfaceManager> networkInterfaceManagerPtr;
        shared_ptr<DhcpClientInterface> dhcpClientInterfacePtr;

        unsigned int maxIpPacketSizeAkaMtuBytes;

        NetworkInterfaceInfoType() : subnetMaskLengthBits(0), gatewayIsForcedNextHop(false),
            subnetIsMultiHop(false), isPrimary(false), allowRoutingBackOutSameInterface(false),
            ignoreUnregisteredProtocol(false),
            maxIpPacketSizeAkaMtuBytes(DefaultMaxIpPacketSizeAkaMtuBytes) {}
    };

    vector<NetworkInterfaceInfoType> networkInterfaces;

    bool gatewayAddressExists;
    unsigned int gatewayInterfaceIndex;

    bool terminateSimWhenRoutingFails;
    unsigned char hopLimit;
    TimeType loopbackDelay;

    shared_ptr<RoutingTable> routingTablePtr;

    vector<shared_ptr<OnDemandRoutingProtocolInterface> > onDemandRoutingProtocols;

    shared_ptr<MobileIpMobileNodeSubsystem> mobileIpMobileNodeSubsystemPtr;
    shared_ptr<MobileIpHomeAgentSubsystem> mobileIpHomeAgentSubsystemPtr;

    vector<shared_ptr<NetworkAddressInterface> > networkAddressInterfaces;

    class ProcessIpPacketEvent: public SimulationEvent {
    public:
        explicit
        ProcessIpPacketEvent(
            const shared_ptr<BasicNetworkLayer>& initNetworkLayerPtr,
            const unsigned int initInterfaceIndex,
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initLastHopAddress)
            :
            networkLayerPtr(initNetworkLayerPtr),
            interfaceIndex(initInterfaceIndex),
            packetPtr(move(initPacketPtr)),
            lastHopAddress(initLastHopAddress)
        {
        }//ProcessIpPacketEvent//

        ProcessIpPacketEvent(ProcessIpPacketEvent&& right) { (*this) = move(right); }

        void operator=(ProcessIpPacketEvent&& right) {
            networkLayerPtr = right.networkLayerPtr;
            interfaceIndex = right.interfaceIndex;
            packetPtr = move(right.packetPtr);
            lastHopAddress = right.lastHopAddress;
        }

        virtual void ExecuteEvent()
        {
            networkLayerPtr->ReceivePacketFromMac(
                interfaceIndex,
                packetPtr,
                lastHopAddress);

        }//ExecuteEvent//

    private:
        shared_ptr<BasicNetworkLayer> networkLayerPtr;
        unsigned int interfaceIndex;
        unique_ptr<Packet> packetPtr;
        NetworkAddress lastHopAddress;

    };//ProcessIpPacketEvent//

    // Statistics:

    shared_ptr<CounterStatistic> packetHopLimitDropsStatPtr;
    shared_ptr<CounterStatistic> packetMaxPacketsQueueDropsStatPtr;
    shared_ptr<CounterStatistic> packetMaxBytesQueueDropsStatPtr;
    shared_ptr<CounterStatistic> packetNoRouteDropsStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;

    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> packetsUndeliveredStatPtr;

    // Private Constructors called by CreateNetworkLayer.

    BasicNetworkLayer(
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const RandomNumberGeneratorSeedType& initNodeSeed);

    void ConstructNetworkLayer(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjects);

    void InitMobileIpMobileNodeSubsystem(const ParameterDatabaseReader& theParameterDatabaseReader);
    void InitMobileIpHomeAgentSubsystem(const ParameterDatabaseReader& theParameterDatabaseReader);

    void SetupInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId);

    void CreateMacOnInterfaceIfNotCustom(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const InterfaceIdType& interfaceId,
        const RandomNumberGeneratorSeedType& nodeSeed);

    RandomNumberGeneratorSeedType nodeSeed;

    bool NetworkAddressIsForThisNode(const NetworkAddress& anAddress, const unsigned int macIndex);
    bool NetworkAddressIsMyAddress(const NetworkAddress& anAddress) const;

    void ProcessLoopbackPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& initialSourceAddress,
        const NetworkAddress& destinationAddress,
        PacketPriorityType trafficClass,
        const unsigned char protocol);

    bool IsOneOfMyMobileIpHomeAddresses(const NetworkAddress& anAddress, const unsigned int interfaceIndex);

    void SendBroadcastOrMulticastPacket(
        unique_ptr<Packet>& packetPtr,
        const unsigned int interfaceIndex,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const unsigned char protocol);

    void BroadcastPacketOnAllInterfaces(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const unsigned char protocol);

    void GiveNetworkPacketToOnDemandRoutingProtocol(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const NetworkAddress& lastHopAddress,
        bool& wasAcceptedForRouting);

    void GiveNetworkPacketToOnDemandRoutingProtocol(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        bool& wasAcceptedForRouting);

    void NotifySendingOrForwardingDataPacketToOnDemandRoutingProtocol(
        const NetworkAddress& lastHopAddress,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& nextAddress,
        const NetworkAddress& destinationAddress);

    void GetTransportLayerPortNumbersFromIpPacket(
        const Packet& aPacket,
        bool& portNumbersWereRetrieved,
        unsigned short int& sourcePort,
        unsigned short int& destinationPort);

    void OutputTraceAndStatsForIpAddressChanged(
        const unsigned int interfaceIndex,
        const NetworkAddress& networkAddress,
        const unsigned int subnetMaskLengthBits) const;

    void OutputTraceAndStatsForHopLimitDrop(
        const Packet& packet,
        const NetworkAddress& destinationAddress) const;

    void OutputTraceAndStatsForNoRouteDrop(
        const Packet& packet,
        const NetworkAddress& destinationAddress) const;

    void OutputTraceAndStatsForFullQueueDrop(
        const Packet& packet, const EnqueueResultType enqueueResult) const;

    void OutputTraceAndStatsForInsertPacketIntoQueue(const Packet& packet) const;

    void OutputTraceAndStatsForReceivePacketFromMac(const Packet& packet) const;

    void OutputTraceAndStatsForUndeliveredPacketFromMacDrop(const Packet& packet) const;


    // Disable:

    BasicNetworkLayer(const BasicNetworkLayer&);
    void operator=(const BasicNetworkLayer&);

};//BasicNetworkLayer//



//---------------------------------------------------------

inline
void ReadStaticRoutingTableFile(
    const string& routeTableFilename,
    const NodeIdType& nodeId,
    const NetworkLayer& theNetworkLayer,
    RoutingTable& theRoutingTable)
{
    ifstream routeTableStream(routeTableFilename.c_str());

    if (!routeTableStream.good()) {
        cerr << "Error: Could not open static routing table file: " << routeTableFilename << endl;
        exit(1);
    }//if//

    while (!routeTableStream.eof()) {
        string aLine;
        getline(routeTableStream, aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            // Comment Line => skip.
        }
        else {
            DeleteTrailingSpaces(aLine);

            istringstream aLineStream(aLine);

            NodeIdType readNodeId;
            aLineStream >> readNodeId;

            if (!aLineStream.good()) {
                cerr << "Error in static route file on line: " << endl;
                cerr << aLine;
                exit(1);
            }//if//

            if (readNodeId == nodeId) {
                string destinationAddressString;
                string destinationMaskString;
                string nextHopAddressString;

                aLineStream >> destinationAddressString;
                aLineStream >> destinationMaskString;
                aLineStream >> nextHopAddressString;

                if ((aLineStream.fail()) || (!aLineStream.eof())) {
                    cerr << "Error in static route file on line: " << endl;
                    cerr << aLine;
                    exit(1);
                }//if//

                bool success1;
                NetworkAddress destinationAddress;
                destinationAddress.SetAddressFromString(destinationAddressString, nodeId, success1);

                bool success2;
                NetworkAddress destinationMask;
                destinationMask.SetAddressFromString(destinationMaskString, nodeId, success2);

                bool success3;
                NetworkAddress nextHopAddress;
                nextHopAddress.SetAddressFromString(nextHopAddressString, nodeId, success3);

                if (!success1 || !success2 || !success3) {
                    cerr << "Error in static route file format on line: " << endl;
                    cerr << aLine;
                    exit(1);
                }//if//

                if (!destinationAddress.IsInSameSubnetAs(destinationAddress, destinationMask)) {
                    cerr << "Error in static route file: Bad mask for address on line:" << endl;
                    cerr << aLine;
                    exit(1);
                }//if//

                unsigned int interfaceIndex;

                bool success;

                theNetworkLayer.GetInterfaceIndexForOneHopDestination(
                    nextHopAddress, success, interfaceIndex);

                if (!success) {
                    cerr << "Error in static route file: Bad non-neighbor next hop on line:" << endl;
                    cerr << aLine;
                    exit(1);
                }//if//


                theRoutingTable.AddOrUpdateRoute(
                    destinationAddress, destinationMask, nextHopAddress, interfaceIndex);

            }//if//
        }//if//
    }//while//

}//ReadStaticRoutingTableFile//



//---------------------------------------------------------






inline
void BasicNetworkLayer::CreateMacOnInterfaceIfNotCustom(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
    const InterfaceIdType& interfaceId,
    const RandomNumberGeneratorSeedType& nodeSeed)
{
    string  macProtocolString;

    if (theParameterDatabaseReader.ParameterExists("mac-protocol", nodeId, interfaceId)) {
        macProtocolString = theParameterDatabaseReader.ReadString("mac-protocol", nodeId, interfaceId);
        ConvertStringToLowerCase(macProtocolString);
    }
    else {
        cerr << "Error in Configuration: mac-protocol for interface: " << interfaceId
             << " is not defined." << endl;
        exit(1);
    }//if//

    unsigned int interfaceIndex = LookupInterfaceIndex(interfaceId);

    if (macProtocolString == "abstract-network") {

       shared_ptr<AbstractNetworkMac> macPtr;
       theGlobalNetworkingObjectBag.abstractNetworkPtr->CreateAnAbstractNetworkMac(
           theParameterDatabaseReader,
           simEngineInterfacePtr,
           shared_from_this(),
           interfaceIndex,
           nodeSeed,
           /*out*/macPtr);

           networkInterfaces.at(interfaceIndex).macLayerPtr = macPtr;
    }//if//

}//CreateMacOnInterfaceIfNotCustom//


inline
BasicNetworkLayer::BasicNetworkLayer(
    const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
    const NodeIdType& initNodeId,
    const RandomNumberGeneratorSeedType& initNodeSeed)
    :
    simulationEngineInterfacePtr(simEngineInterfacePtr),
    nodeId(initNodeId),
    primaryNetworkAddress(NetworkAddress::anyAddress),
    gatewayAddressExists(false),
    gatewayInterfaceIndex(0),
    routingTablePtr(new RoutingTable()),
    terminateSimWhenRoutingFails(false),
    hopLimit(DEFAULT_HOP_LIMIT),
    loopbackDelay(1 * NANO_SECOND),
    nodeSeed(initNodeSeed),
    packetHopLimitDropsStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_HopLimitDrops")),
    packetNoRouteDropsStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_NoRouteDrops")),
    packetMaxPacketsQueueDropsStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_MaxPacketsQueueDrops")),
    packetMaxBytesQueueDropsStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_MaxBytesQueueDrops")),
    bytesReceivedStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_BytesReceived")),
    bytesSentStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_BytesSent")),
    packetsReceivedStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_PacketsReceived")),
    packetsSentStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_PacketsSent")),
    packetsUndeliveredStatPtr(
        simEngineInterfacePtr->CreateCounterStat(modelName + "_PacketsUndelivered"))
{}


inline
void BasicNetworkLayer::ConstructNetworkLayer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag)
{
    extraSimulationAccessPointFinderPtr =
        theGlobalNetworkingObjectBag.extraSimulationAccessPointFinderPtr;

    if (theParameterDatabaseReader.ParameterExists("mobile-ip-address", nodeId)) {
        (*this).InitMobileIpMobileNodeSubsystem(theParameterDatabaseReader);
    }

    if ((theParameterDatabaseReader.ParameterExists("mobile-ip-home-agent", nodeId)) &&
        (theParameterDatabaseReader.ReadBool("mobile-ip-home-agent", nodeId) == true)) {

        (*this).InitMobileIpHomeAgentSubsystem(theParameterDatabaseReader);
    }//if//

    set<InterfaceIdType> setOfInterfaceIds;

    theParameterDatabaseReader.MakeSetOfAllInterfaceIdsForANode(nodeId, "network-address", setOfInterfaceIds);

    for(set<InterfaceIdType>::iterator iter = setOfInterfaceIds.begin();
        (iter != setOfInterfaceIds.end()); iter++) {

        (*this).SetupInterface(theParameterDatabaseReader, *iter);
        (*this).CreateMacOnInterfaceIfNotCustom(
            theParameterDatabaseReader, theGlobalNetworkingObjectBag, simulationEngineInterfacePtr, *iter, nodeSeed);

    }//for//

    if ((primaryNetworkAddress.IsAnyAddress()) && (!setOfInterfaceIds.empty())) {

        primaryNetworkAddress = networkInterfaces[0].address;
        networkInterfaces[0].isPrimary = true;

    }//if//

    if (theParameterDatabaseReader.ParameterExists("network-terminate-sim-when-routing-fails")) {
        terminateSimWhenRoutingFails =
            theParameterDatabaseReader.ReadBool("network-terminate-sim-when-routing-fails");
    }//if//

    if (theParameterDatabaseReader.ParameterExists("network-hop-limit", nodeId)) {
        const int hopLimitInt =
            theParameterDatabaseReader.ReadInt("network-hop-limit", nodeId);
        if (hopLimitInt <= 0) {
            cerr << "Error: network-hop-limit(" << hopLimitInt
                 << ") should be greater than 0" << endl;;
            exit(1);
        } else if (hopLimitInt >= 256) {
            cerr << "Error: network-hop-limit(" << hopLimitInt
                 << ") should be less than 256" << endl;;
            exit(1);
        }//if//
        hopLimit = static_cast<unsigned char>(hopLimitInt);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("network-loopback-delay", nodeId)) {
        loopbackDelay = theParameterDatabaseReader.ReadTime("network-loopback-delay", nodeId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("network-static-route-file")) {
        const string routeTableFilename =
            theParameterDatabaseReader.ReadString("network-static-route-file");

        ReadStaticRoutingTableFile(
            routeTableFilename,
            nodeId,
            *this,
            *(*this).routingTablePtr);
    }//if//

}//ConstructNetworkLayer//



inline
shared_ptr<BasicNetworkLayer> BasicNetworkLayer::CreateNetworkLayer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
    const NodeIdType& nodeId,
    const RandomNumberGeneratorSeedType& nodeSeed)
{
    shared_ptr<BasicNetworkLayer> networkLayerPtr(
        new BasicNetworkLayer(simEngineInterfacePtr, nodeId, nodeSeed));

    networkLayerPtr->ConstructNetworkLayer(
        theParameterDatabaseReader,
        theGlobalNetworkingObjectBag);

    return networkLayerPtr;
}



inline
unsigned int BasicNetworkLayer::LookupInterfaceIndex(const NetworkAddress& interfaceAddress) const
{
    for(unsigned int i = 0; (i < networkInterfaces.size()); i++) {
        if (networkInterfaces.at(i).address == interfaceAddress) {
            return i;

        }//if//
    }//for//
    assert(false && "No interface for network Address");
    abort();
    return 0;
}


inline
unsigned int BasicNetworkLayer::LookupInterfaceIndex(const InterfaceIdType& interfaceName) const
{
    for(unsigned int i = 0; (i < networkInterfaces.size()); i++) {
        if (networkInterfaces.at(i).interfaceName == interfaceName) {
            return i;

        }//if//
    }//for//

    cerr << "Error: No interface defined for interfaceName: " << interfaceName << endl;
    exit(1);

    assert(false && "No interface for network Address");
    abort();
    return 0;
}


inline
void BasicNetworkLayer::SetInterfaceMacLayer(const unsigned int interfaceIndex, const shared_ptr<MacLayer>& macLayerPtr)
{
    networkInterfaces.at(interfaceIndex).macLayerPtr = macLayerPtr;
}


inline
void BasicNetworkLayer::SetInterfaceGatewayAddress(
    const unsigned int interfaceIndex,
    const NetworkAddress& newGatewayAddress,
    const bool gatewayIsForcedNextHop)
{
    networkInterfaces.at(interfaceIndex).gatewayAddress = newGatewayAddress;
    networkInterfaces.at(interfaceIndex).gatewayIsForcedNextHop = gatewayIsForcedNextHop;
    (*this).gatewayAddressExists = (newGatewayAddress != NetworkAddress::invalidAddress);
    (*this).gatewayInterfaceIndex = interfaceIndex;
}


inline
bool BasicNetworkLayer::IsANetworkAddressForThisNode(const NetworkAddress& anAddress) const
{
    for(unsigned int i = 0; (i < networkInterfaces.size()); i++) {
        if (networkInterfaces[i].address == anAddress) {
            return true;
        }//if//
    }//for//
    return false;
}

inline
void BasicNetworkLayer::CheckIfNetworkAddressIsForThisNode(
    const NetworkAddress& anAddress,
    bool& addressIsForThisNode,
    unsigned int& interfaceIndex) const
{
    addressIsForThisNode = false;

    for(unsigned int i = 0; (i < networkInterfaces.size()); i++) {
        if (networkInterfaces[i].address == anAddress) {
            addressIsForThisNode = true;
            interfaceIndex = i;
            break;
        }//if//
    }//for//
}//CheckIfNetworkAddressIsForThisNode//





inline
void BasicNetworkLayer::GetInterfaceIndexForOneHopDestination(
    const NetworkAddress& destinationAddress,
    bool& success,
    unsigned int& interfaceIndex) const
{
    for(unsigned int i = 0; (i < networkInterfaces.size()); i++) {
        if (destinationAddress.IsInSameSubnetAs(networkInterfaces[i].address, networkInterfaces[i].subnetMask)) {
            interfaceIndex = i;
            success = true;
            return;
        }//if//
    }//for//

    success = false;

}//GetInterfaceIndexForOneHopDestination//




inline
void BasicNetworkLayer::GetNextHopAddressAndInterfaceIndexForDestination(
    const NetworkAddress& initialDestinationAddress,
    bool& foundRoute,
    NetworkAddress& nextHopAddress,
    unsigned int& interfaceIndex) const
{
    NetworkAddress destinationAddress = initialDestinationAddress;

    // Try Extra-simulation access point finder routing first (if it is being used)
    // Note this code assumes that all traffic to a node with an Access Point (AP)
    // must go through the AP, i.e. no direct communication of mobiles.

    if (extraSimulationAccessPointFinderPtr.get() != 0) {
        bool foundAccessPoint;
        NetworkAddress accessPointAddress;

        extraSimulationAccessPointFinderPtr->LookupAccessPointFor(
            destinationAddress, foundAccessPoint, accessPointAddress);

        if (foundAccessPoint) {
            bool isAnInterfaceNetworkAddressOnThisNode;

            CheckIfNetworkAddressIsForThisNode(
                accessPointAddress,
                isAnInterfaceNetworkAddressOnThisNode,
                interfaceIndex);

            if (isAnInterfaceNetworkAddressOnThisNode) {
                foundRoute = true;
                nextHopAddress = initialDestinationAddress;
                return;
            }//if//

            // reset destination to access point

            destinationAddress = accessPointAddress;
        }//if//

    }//if//

    // Then, try the local routing table.

    routingTablePtr->LookupRoute(destinationAddress, foundRoute, nextHopAddress, interfaceIndex);

    if (foundRoute) {
        return;
    }//if//

    // Then, see if the destination is in a local subnet.

    for(unsigned int i = 0; (i < networkInterfaces.size()); i++) {
        const NetworkInterfaceInfoType& interface = networkInterfaces[i];

        if (destinationAddress.IsInSameSubnetAs(interface.address, interface.subnetMask)) {

            const bool destinationAddressIsABroadcastAddress =
                (destinationAddress.IsABroadcastOrAMulticastAddress(interface.subnetMask));

            if ((!destinationAddressIsABroadcastAddress) &&
                ((interface.subnetIsMultiHop) || (interface.gatewayIsForcedNextHop))) {

                // Multi-hop (ad-hoc) network do not automatically assume node on local subnet is connected
                //   for sending non-broadcast packets.
                // Basestation Topology: Next hop (Gateway) will be forced to the basestation.
            }
            else {
                interfaceIndex = i;
                if (destinationAddressIsABroadcastAddress) {
                    // Hack: For convenience just set a "subnet/mask specific" broadcast address to
                    //       the generic broadcast address.
                    nextHopAddress = NetworkAddress::broadcastAddress;
                }
                else {
                    nextHopAddress = destinationAddress;
                }//if//

                foundRoute = true;
                return;

            }//if//
        }//if//
    }//for//

    // Finally, see if there is a default gateway.

    if (gatewayAddressExists) {
        interfaceIndex = gatewayInterfaceIndex;
        nextHopAddress = networkInterfaces[gatewayInterfaceIndex].gatewayAddress;
        foundRoute = true;
        return;
    }//if//

    if (terminateSimWhenRoutingFails) {
        cerr << "Error: At node " << nodeId << " Destination Address: "
             << destinationAddress.ConvertToString() << " is not reachable." << endl;
        cerr << "  set network-terminate-sim-when-routing-fails to false if this is okay." << endl;

        exit(1);
    }//if//

    foundRoute = false;

}//GetNextHopAddressAndInterfaceIndexForDestination//




inline
void BasicNetworkLayer::GetNextHopAddressAndInterfaceIndexForNetworkPacket(
    const Packet& aPacket,
    bool& success,
    NetworkAddress& nextHopAddress,
    unsigned int& interfaceIndex) const
{
    const IpHeaderOverlayModel ipHeader(aPacket.GetRawPayloadData(), aPacket.LengthBytes());

    (*this).GetNextHopAddressAndInterfaceIndexForDestination(
        ipHeader.GetDestinationAddress(),
        success,
        nextHopAddress,
        interfaceIndex);

}//GetNextHopAddressAndInterfaceIndexForNetworkPacket//


inline
void BasicNetworkLayer::OutputTraceAndStatsForIpAddressChanged(
    const unsigned int interfaceIndex,
    const NetworkAddress& networkAddress,
    const unsigned int subnetMaskLengthBits) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceNetwork)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            IpAddressChangeTraceRecord traceData;
            traceData.ipAddress = Ipv6NetworkAddress(networkAddress);
            traceData.subnetMaskLengthBits = subnetMaskLengthBits;
            traceData.interfaceIndex = interfaceIndex;
            traceData.useIpv6 = (NetworkAddress::numberBits == 128);

            assert(sizeof(traceData) == IP_ADDRESS_CHANGE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "IpAddrChange", traceData);

        }
        else {
            ostringstream outStream;

            outStream << "IfIndex= " << interfaceIndex
                      << " NewIpAddr= " << networkAddress.ConvertToString()
                      << " SubnetMaskBits= " << subnetMaskLengthBits;

            simulationEngineInterfacePtr->OutputTrace(modelName, "", "IpAddrChange", outStream.str());
        }//if//

    }//if//

}//OutputTraceAndStatsForIpAddressChanged//


inline
void BasicNetworkLayer::OutputTraceAndStatsForHopLimitDrop(
    const Packet& packet,
    const NetworkAddress& destinationAddress) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceNetwork)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            IpPacketDropTraceRecord traceData;
            const PacketIdType& packetId = packet.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.destinationAddress = Ipv6NetworkAddress(destinationAddress);
            traceData.useIpv6 = (NetworkAddress::numberBits == 128);

            assert(sizeof(traceData) == IP_PACKET_DROP_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "HopLimitDrop", traceData);

        }
        else {
            ostringstream outStream;

            outStream << "PktId= " << packet.GetPacketId()
                      << " DestAddr= " << destinationAddress.ConvertToString();

            simulationEngineInterfacePtr->OutputTrace(modelName, "", "HopLimitDrop", outStream.str());
        }//if//

    }//if//

    packetHopLimitDropsStatPtr->IncrementCounter();

}//OutputTraceAndStatsForHopLimitDrop//


inline
void BasicNetworkLayer::OutputTraceAndStatsForNoRouteDrop(
    const Packet& packet,
    const NetworkAddress& destinationAddress) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceNetwork)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            IpPacketDropTraceRecord traceData;
            const PacketIdType& packetId = packet.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.destinationAddress = Ipv6NetworkAddress(destinationAddress);
            traceData.useIpv6 = (NetworkAddress::numberBits == 128);

            assert(sizeof(traceData) == IP_PACKET_DROP_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "NoRouteDrop", traceData);

        }
        else {
            ostringstream outStream;

            outStream << "PktId= " << packet.GetPacketId()
                      << " DestAddr= " << destinationAddress.ConvertToString();

            simulationEngineInterfacePtr->OutputTrace(modelName, "", "NoRouteDrop", outStream.str());
        }

    }//if//

    packetNoRouteDropsStatPtr->IncrementCounter();

}//OutputTraceAndStatsForNoRouteDrop//


inline
void BasicNetworkLayer::OutputTraceAndStatsForFullQueueDrop(
    const Packet& packet, const EnqueueResultType enqueueResult) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceNetwork)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            IpFullQueueDropTraceRecord traceData;
            const PacketIdType& packetId = packet.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.enqueueResult = static_cast<uint16_t>(enqueueResult);

            assert(sizeof(traceData) == IP_FULL_QUEUE_DROP_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "FullQueueDrop", traceData);

        }
        else {

            ostringstream outStream;
            outStream << "PktId= " << packet.GetPacketId();
            outStream << " Result= " << ConvertToEnqueueResultString(enqueueResult);

            simulationEngineInterfacePtr->OutputTrace(modelName, "", "FullQueueDrop", outStream.str());

        }//if//

    }//if//

    if (enqueueResult == ENQUEUE_FAILURE_BY_MAX_PACKETS) {
        packetMaxPacketsQueueDropsStatPtr->IncrementCounter();
    }
    else if (enqueueResult == ENQUEUE_FAILURE_BY_MAX_BYTES) {
        packetMaxBytesQueueDropsStatPtr->IncrementCounter();
    }//if//

}//OutputTraceAndStatsForFullQueueDrop//

inline
void BasicNetworkLayer::OutputTraceAndStatsForInsertPacketIntoQueue(const Packet& packet) const
{

    if (simulationEngineInterfacePtr->TraceIsOn(TraceNetwork)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            IpSendTraceRecord traceData;
            const PacketIdType& packetId = packet.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == IP_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "IpSend", traceData);

        }
        else {

            ostringstream outStream;
            const PacketIdType& packetId = packet.GetPacketId();
            outStream << "PktId= " << packetId;
            simulationEngineInterfacePtr->OutputTrace(modelName, "", "IpSend", outStream.str());

        }//if//
    }//if//

    packetsSentStatPtr->IncrementCounter();

    const unsigned int packetLengthBytes = packet.LengthBytes();
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForInsertPacketIntoQueue//

inline
void BasicNetworkLayer::OutputTraceAndStatsForReceivePacketFromMac(const Packet& packet) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceNetwork)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            IpReceiveTraceRecord traceData;
            const PacketIdType& packetId = packet.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.packetLengthBytes = static_cast<uint16_t>(packet.LengthBytes());

            assert(sizeof(traceData) == IP_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "IpRecv", traceData);

        }
        else {

            ostringstream outStream;
            const PacketIdType& packetId = packet.GetPacketId();
            outStream << "PktId= " << packetId;

            outStream << " PacketBytes= " << packet.LengthBytes();

            simulationEngineInterfacePtr->OutputTrace(modelName, "", "IpRecv", outStream.str());

        }//if//
    }//if//

    packetsReceivedStatPtr->IncrementCounter();

    const unsigned int packetLengthBytes = packet.LengthBytes();
    bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForReceivePacketFromMac//

inline
void BasicNetworkLayer::OutputTraceAndStatsForUndeliveredPacketFromMacDrop(const Packet& packet) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceNetwork)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

             IpPacketUndeliveredTraceRecord traceData;
             const PacketIdType& packetId = packet.GetPacketId();
             traceData.sourceNodeId = packetId.GetSourceNodeId();
             traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

             assert(sizeof(traceData) == IP_PACKET_UNDELIVERED_TRACE_RECORD_BYTES);

             simulationEngineInterfacePtr->OutputTraceInBinary(
                 modelName, "", "PacketUndelivered", traceData);

        }
        else {
            ostringstream outStream;

            outStream << "PktId= " << packet.GetPacketId();

            simulationEngineInterfacePtr->OutputTrace(
                modelName, "", "PacketUndelivered", outStream.str());
        }

    }//if//

    packetsUndeliveredStatPtr->IncrementCounter();

}//OutputTraceAndStatsForUndeliveredPacketFromMacDrop//

inline
void BasicNetworkLayer::GiveNetworkPacketToOnDemandRoutingProtocol(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const NetworkAddress& lastHopAddress,
    bool& wasAcceptedForRouting)
{
    for(unsigned int i = 0; (i < onDemandRoutingProtocols.size()); i++) {
        onDemandRoutingProtocols[i]->HandlePacketWithNoRoute(
            packetPtr,
            sourceAddress,
            destinationAddress,
            lastHopAddress,
            wasAcceptedForRouting);

        if (wasAcceptedForRouting) {
            assert(packetPtr == nullptr);
            return;
        }//if//
    }//for//

    assert((packetPtr != nullptr) && (!wasAcceptedForRouting));

}//GiveNetworkPacketToOnDemandRoutingProtocol//


inline
void BasicNetworkLayer::GiveNetworkPacketToOnDemandRoutingProtocol(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    bool& wasAcceptedForRouting)
{
    (*this).GiveNetworkPacketToOnDemandRoutingProtocol(
        packetPtr,
        sourceAddress,
        destinationAddress,
        NetworkAddress::invalidAddress,
        wasAcceptedForRouting);
}

inline
 void BasicNetworkLayer::NotifySendingOrForwardingDataPacketToOnDemandRoutingProtocol(
    const NetworkAddress& lastHopAddress,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& nextAddress,
    const NetworkAddress& destinationAddress)
{
    const bool sourceIsThisNode =
        (*this).IsANetworkAddressForThisNode(sourceAddress);

     for(unsigned int i = 0; (i < onDemandRoutingProtocols.size()); i++) {
        onDemandRoutingProtocols[i]->InspectPacket(
            lastHopAddress,
            sourceAddress,
            nextAddress,
            destinationAddress,
            sourceIsThisNode);
     }
}

inline
void BasicNetworkLayer::ReceivePacketFromUpperLayer(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destinationAddress,
    PacketPriorityType trafficClass,
    const unsigned char protocol)
{
    (*this).ReceivePacketFromUpperLayer(
        packetPtr, NetworkAddress::anyAddress, destinationAddress, trafficClass, protocol);
}



inline
void BasicNetworkLayer::ReceiveRoutedNetworkPacketFromRoutingProtocol(
    unique_ptr<Packet>& packetPtr,
    const unsigned int interfaceIndex,
    const NetworkAddress& nextHopAddress)
{
    IpHeaderOverlayModel ipHeader(packetPtr->GetRawPayloadData(), packetPtr->LengthBytes());

    if (ipHeader.GetSourceAddress() == NetworkAddress::anyAddress) {
       // Set the source Address with respect to the passed in interface.

       ipHeader.SetSourceAddress(networkInterfaces.at(interfaceIndex).address);
    }//if//

    (*this).InsertPacketIntoAnOutputQueue(
        packetPtr,
        interfaceIndex,
        nextHopAddress,
        ipHeader.GetTrafficClass());

}//ReceiveRoutedIpPacketFromRoutingProtocol//


inline
bool BasicNetworkLayer::NetworkAddressIsForThisNode(
    const NetworkAddress& anAddress,
    const unsigned int interfaceIndex)
{
    //check all interface addresses
    if (NetworkAddressIsMyAddress(anAddress)) {
        return true;
    }//if//

    //broadcast
    if ((anAddress.IsInSameSubnetAs(GetNetworkAddress(interfaceIndex), GetSubnetMask(interfaceIndex))) &&
        (anAddress.IsABroadcastOrAMulticastAddress(GetSubnetMask(interfaceIndex)))) {

        return true;

    }//if//

    return (IsOneOfMyMobileIpHomeAddresses(anAddress, interfaceIndex));

}//NetworkAddressIsForThisNode//


inline
bool BasicNetworkLayer::NetworkAddressIsMyAddress(
    const NetworkAddress& anAddress) const
{
    for(unsigned int i = 0; (i < networkInterfaces.size()); i++) {
        if (networkInterfaces[i].address == anAddress) {
            return true;
        }//if//
    }//for//

    return false;

}//NetworkAddressIsMyAddress//


inline
void BasicNetworkLayer::ProcessLoopbackPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& initialSourceAddress,
    const NetworkAddress& destinationAddress,
    PacketPriorityType trafficClass,
    const unsigned char protocol)
{
    NetworkAddress sourceNetworkAddress;

    if (NetworkAddressIsMyAddress(initialSourceAddress)) {
        sourceNetworkAddress = initialSourceAddress;
    }
    else {
        sourceNetworkAddress = GetPrimaryNetworkAddress();
    }//if//

    IpHeaderModel
        header(
            trafficClass,
            packetPtr->LengthBytes(),
            hopLimit,
            protocol,
            sourceNetworkAddress,
            destinationAddress);

    packetPtr->AddRawHeader(header.GetPointerToRawBytes(), header.GetNumberOfRawBytes());
    packetPtr->AddTrailingPadding(header.GetNumberOfTrailingBytes());

    OutputTraceAndStatsForInsertPacketIntoQueue(*packetPtr);

    const TimeType processTime = simulationEngineInterfacePtr->CurrentTime() + loopbackDelay;

    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new ProcessIpPacketEvent(
            shared_from_this(),
            LookupInterfaceIndex(sourceNetworkAddress),
            packetPtr,
            sourceNetworkAddress)),
        processTime);

}//ProcessLoopbackPacket//


inline
void BasicNetworkLayer::ReceiveUndeliveredPacketFromMac(
    const unsigned int interfaceIndex,
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& nextHopAddress)
{
    bool wasHandled = false;
    for(unsigned int i = 0; (i < onDemandRoutingProtocols.size()); i++) {

        const IpHeaderOverlayModel ipHeader(packetPtr->GetRawPayloadData(), packetPtr->LengthBytes());

        onDemandRoutingProtocols[i]->HandlePacketUndeliveredByMac(
            packetPtr,
            ipHeader.GetDestinationAddress(),
            interfaceIndex,
            nextHopAddress,
            wasHandled);

        if (wasHandled) {
            assert(packetPtr == nullptr);
            return;
        }//if//
    }//for//

    if (!wasHandled) {
        (*this).OutputTraceAndStatsForUndeliveredPacketFromMacDrop(*packetPtr);

        packetPtr = nullptr;
    }//if//

}//ReceiveUndeliveredPacketFromMac//


inline
void BasicNetworkLayer::DisconnectFromOtherLayers()
{
    protocolPacketHandlerMap.clear();

    for(unsigned int interfaceIndex = 0; (interfaceIndex < networkInterfaces.size()); interfaceIndex++) {

        NetworkInterfaceInfoType& networkInterface = networkInterfaces[interfaceIndex];

        if (networkInterface.macLayerPtr != nullptr) {

            networkInterface.macLayerPtr->DisconnectFromOtherLayers();
            networkInterface.macLayerPtr.reset();
        }//if//

    }//for//
}


inline
shared_ptr<MacQualityOfServiceControlInterface> BasicNetworkLayer::GetMacQualityOfServiceInterface(
    const unsigned int interfaceIndex) const
{
    return (networkInterfaces.at(interfaceIndex).macLayerPtr->GetQualityOfServiceInterface());
}




inline
AbstractNetwork::AbstractNetwork(const ParameterDatabaseReader& theParameterDatabaseReader)
{
}//AbstractNetwork()//

inline
void AbstractNetwork::CreateAnAbstractNetworkMac(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<NetworkLayer>& networkLayerPtr,
    const unsigned int interfaceIndex,
    const RandomNumberGeneratorSeedType& nodeSeed,
    shared_ptr<AbstractNetworkMac>& macPtr)
{
    macPtr = shared_ptr<AbstractNetworkMac>(
        new AbstractNetworkMac(
            theParameterDatabaseReader,
            shared_from_this(),
            simulationEngineInterfacePtr,
            networkLayerPtr,
            interfaceIndex,
            nodeSeed));


    addressToMacMap[networkLayerPtr->GetNetworkAddress(interfaceIndex)] = macPtr;

}//CreateAnAbstractNetworkMac//

inline
shared_ptr<AbstractNetworkMac> AbstractNetwork::GetDestinationMacPtr(const NetworkAddress destinationAddress)
{
    map<NetworkAddress, shared_ptr<AbstractNetworkMac> >::iterator iter =
        addressToMacMap.find(destinationAddress);

    if (iter == addressToMacMap.end()) {
        cerr << "Error in AbstractNetwork: Destination interface address ";
        cerr << destinationAddress.ConvertToString() << " does not exist." << endl;
        exit(1);
    }//if//

    return (iter->second);

}//GetDestinationMacPtr//



inline
void AbstractNetwork::GetDestinationMacPtrsForBroadcast(
    const NetworkAddress sourceAddress,
    const NetworkAddress sourceMaskAddress,
    list<shared_ptr<AbstractNetworkMac> >& macPtrs)
{
    typedef map<NetworkAddress, shared_ptr<AbstractNetworkMac> >::iterator IterType;

    IterType iter = addressToMacMap.begin();
    while(iter != addressToMacMap.end()) {

        const NetworkAddress anAddress = (iter->first);

        if ((anAddress != sourceAddress) && (anAddress.IsInSameSubnetAs(
                                                 sourceAddress, sourceMaskAddress))) {
            macPtrs.push_back(iter->second);
        }//if//

        iter++;
    }//while//

}//GetDestinationMacPtrsForBroadcast//


}//namespace//


#endif
