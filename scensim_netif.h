// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_NETIF_H
#define SCENSIM_NETIF_H

#include <scensim_arp.h>
#include <scensim_ndp.h>

namespace ScenSim {

class NetworkInterfaceManager: public enable_shared_from_this<NetworkInterfaceManager> {
public:
    NetworkInterfaceManager(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const RandomNumberGeneratorSeedType& nodeSeed);

    virtual ~NetworkInterfaceManager();

    bool IsAddressResolutionCompleted(
        const NetworkAddress& networkAddress) const;

    void NotifyProtocolAddressChanged();

    void ProcessLinkIsUpNotification();
    void ProcessLinkIsDownNotification();
    void ProcessNewLinkToANodeNotification(const GenericMacAddressType& macAddress);

    void ProcessArpPacket(unique_ptr<Packet>& packetPtr);

    void ProcessIcmpPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const NetworkAddress& lastHopAddress);

    void SendAddressResolutionRequest(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& targetAddress,
        const PacketPriorityType& priority,
        const EtherTypeFieldType etherType = ETHERTYPE_IS_NOT_SPECIFIED);

    bool IsAddressResolutionEnabled() const;

    MacAddressResolver<SixByteMacAddressType>* CreateMacAddressResolver() const;

private:
    shared_ptr<AddressResolutionProtocol> arpPtr;
    shared_ptr<NeighborDiscoveryProtocol> ndpPtr;

};//NetworkInterfaceManager//

inline
NetworkInterfaceManager::NetworkInterfaceManager(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId,
    const unsigned int interfaceIndex,
    const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
    const shared_ptr<NetworkLayer>& networkLayerPtr,
    const RandomNumberGeneratorSeedType& nodeSeed)
    :
    arpPtr(),
    ndpPtr()
{
    if (theParameterDatabaseReader.ParameterExists("network-enable-arp", nodeId, interfaceId) &&
        theParameterDatabaseReader.ReadBool("network-enable-arp", nodeId, interfaceId)) {
        arpPtr.reset(
            new AddressResolutionProtocol(
                theParameterDatabaseReader,
                simEngineInterfacePtr,
                nodeId,
                interfaceId,
                interfaceIndex,
                networkLayerPtr,
                nodeSeed));
    }//if//

    if (theParameterDatabaseReader.ParameterExists("network-enable-ndp", nodeId, interfaceId) &&
        theParameterDatabaseReader.ReadBool("network-enable-ndp", nodeId, interfaceId)) {
        ndpPtr.reset(
            new NeighborDiscoveryProtocol(
                theParameterDatabaseReader,
                nodeId,
                interfaceId,
                interfaceIndex,
                simEngineInterfacePtr,
                networkLayerPtr,
                nodeSeed));
    }//if//

}//NetworkInterfaceManager//

inline
NetworkInterfaceManager::~NetworkInterfaceManager()
{
}//~NetworkInterfaceManager//

inline
bool NetworkInterfaceManager::IsAddressResolutionCompleted(
    const NetworkAddress& networkAddress) const
{
    if (arpPtr != nullptr) {
        return arpPtr->HardwareAddressIsInTable(networkAddress);
    }
    else if (ndpPtr != nullptr) {
        return ndpPtr->NeighborIsCached(networkAddress);
    }
    else {
        return true;
    }//if//

}//IsAddressResolutionCompleted//

inline
void NetworkInterfaceManager::NotifyProtocolAddressChanged()
{
    if (arpPtr != nullptr) {
        arpPtr->NotifyProtocolAddressChanged();
    }//if//

}//NotifyProtocolAddressChanged//

inline
void NetworkInterfaceManager::ProcessLinkIsUpNotification()
{
    if (ndpPtr != nullptr) {
        ndpPtr->ProcessLinkIsUpNotification();
    }//if//

}//ProcessLinkIsUpNotification//

inline
void NetworkInterfaceManager::ProcessLinkIsDownNotification()
{
    if (ndpPtr != nullptr) {
        ndpPtr->ProcessLinkIsDownNotification();
    }//if//

}//ProcessLinkIsDownNotification//

inline
void NetworkInterfaceManager::ProcessNewLinkToANodeNotification(
    const GenericMacAddressType& newNodeMacAddress)
{
    if (ndpPtr != nullptr) {
        ndpPtr->ProcessNewLinkToANodeNotification(newNodeMacAddress);
    }//if//

}//NewLinkToNodeNotification//

inline
void NetworkInterfaceManager::ProcessArpPacket(unique_ptr<Packet>& packetPtr)
{
    if (arpPtr != nullptr) {
        arpPtr->ReceiveArpPacket(*packetPtr);
    }//if//

    packetPtr = nullptr;

}//ProcessArpPacket//

inline
void NetworkInterfaceManager::ProcessIcmpPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const PacketPriorityType trafficClass,
    const NetworkAddress& lastHopAddress)
{
    if (ndpPtr != nullptr) {
        ndpPtr->ProcessIcmpPacket(
            packetPtr,
            sourceAddress,
            destinationAddress,
            trafficClass,
            lastHopAddress);
    }//if//

}//ProcessIcmpPacket//

inline
void NetworkInterfaceManager::SendAddressResolutionRequest(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& targetAddress,
    const PacketPriorityType& priority,
    const EtherTypeFieldType etherType)
{
    if (arpPtr != nullptr) {
        arpPtr->SendArpRequest(
            packetPtr,
            targetAddress,
            priority,
            etherType);
    }
    else if (ndpPtr != nullptr) {
        ndpPtr->SendNeighborSolicitationMessage(
            packetPtr,
            targetAddress,
            priority,
            etherType);
    }//if//

}//SendAddressResolutionRequest//

inline
bool NetworkInterfaceManager::IsAddressResolutionEnabled() const
{
    if (arpPtr != nullptr) {
        return true;
    }
    else if ((ndpPtr != nullptr) && (ndpPtr->IsAddressResolutionEnabled())) {
        return true;
    }
    else {
        return false;
    }//if//

}//IsAddressResolutionEnabled//

inline
MacAddressResolver<SixByteMacAddressType>* NetworkInterfaceManager::CreateMacAddressResolver() const
{
    if (arpPtr != nullptr) {
        return new ArpMacAddressResolver(arpPtr);
    }
    else if ((ndpPtr != nullptr) && (ndpPtr->IsAddressResolutionEnabled())) {
        return new NdpMacAddressResolver(ndpPtr);
    }
    else {
        return nullptr;
    }//if//

}//CreateMacAddressResolver//

}//namespace//

#endif
