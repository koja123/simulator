// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_MOBILEIP_H
#define SCENSIM_MOBILEIP_H

#include <assert.h>
#include <memory>
#include "scensim_network.h"

namespace ScenSim {

using std::shared_ptr;

class AbstractMobileIpRouteSelection {
public:
    AbstractMobileIpRouteSelection() {}
    virtual ~AbstractMobileIpRouteSelection() {}
    void RoundRobinSelection(
        const NetworkAddress& destinationAddress,
        const map<unsigned short, NetworkAddress>& careOfAddress,
        NetworkAddress& returnAddress);
    void FirstEntrySelection(
        const map<unsigned short, NetworkAddress>& careOfAddress,
        NetworkAddress& returnAddress);
    static const int ROUNDROBIN = 0;
    static const int FIRSTENTRY = 1;

private:
    map<NetworkAddress, unsigned short> roundrobinId;
};


inline
void AbstractMobileIpRouteSelection::RoundRobinSelection(
    const NetworkAddress& destinationAddress,
    const map<unsigned short, NetworkAddress>& careOfAddress,
    NetworkAddress& returnAddress)
{
    map<NetworkAddress, unsigned short>::iterator idIter = roundrobinId.find(destinationAddress);
    if (idIter == roundrobinId.end()) {
       roundrobinId[destinationAddress] = 0;
       idIter = roundrobinId.find(destinationAddress);
    }
    if (careOfAddress.size() == 0) {
        returnAddress = NetworkAddress::invalidAddress;
    }
    else {
        if (careOfAddress.size() == idIter->second) {
             idIter->second = 0;
        }
        map<unsigned short, NetworkAddress>::const_iterator iter = careOfAddress.begin();
        for(int i = 0 ; i < idIter->second; i++) {
            iter++;
        }
        returnAddress = iter->second;
        idIter->second++;
    }
}

inline
void AbstractMobileIpRouteSelection::FirstEntrySelection(
    const map<unsigned short, NetworkAddress>& careOfAddress,
    NetworkAddress& returnAddress)
{
    map<unsigned short, NetworkAddress>::const_iterator iter = careOfAddress.begin();
    if (iter == careOfAddress.end()) {
        returnAddress = NetworkAddress::invalidAddress;
    }
    else {
        returnAddress = iter->second;
    }
}


class MobileIpMobileNodeSubsystem: public NetworkInterfaceStatusChangeNotificationInterface {
public:
    MobileIpMobileNodeSubsystem(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface> initSimEngineInterfacePtr,
        const shared_ptr<NetworkLayer> initNetworkLayerPtr,
        const unsigned char initHopLimit);

    void AddInterfaceIfMobileIpEnabled(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const unsigned int interfaceIndex,
        const string& interfaceId);

    NetworkAddress GetHomeAddress() const { return homeAddress; }
    NetworkAddress GetHomeAgentAddress() const { return homeAgentAddress; }
    void HandleMajorInterfaceStatusChange(const unsigned int interfaceIndex);

    void TunnelPacketToCorrespondentNode(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        PacketPriorityType trafficClass,
        const unsigned char protocol);

private:
    NetworkAddress homeAddress;
    NetworkAddress homeAgentAddress;
    unsigned short currentSequenceNumber;
    const static unsigned short bindingLifetimein4SecUnits = USHRT_MAX;
    unsigned short numberEnabledInterfaces;
    unsigned char hopLimit;
    static const unsigned short maxNumberEnabledInterfaces = 4;

    struct InterfaceSpecificInfo {
        bool isEnabledOnInterface;
        NetworkAddress currentCareOfAddress;
        unsigned short bindingId;

        InterfaceSpecificInfo() :
            isEnabledOnInterface(false),
            currentCareOfAddress(NetworkAddress::invalidAddress),
            bindingId(0) {}
    };

    vector<InterfaceSpecificInfo> interfaceInfos;

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    shared_ptr<NetworkLayer> networkLayerPtr;

    // Implementation routines

    void FindInterfaceForAHomeAddress(
        const NetworkAddress& homeAddress,
        bool& wasFound,
        unsigned int& interfaceIndex) const;

};//MobileIpMobileNodeSubsystem//

inline
MobileIpMobileNodeSubsystem::MobileIpMobileNodeSubsystem(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface> initSimEngineInterfacePtr,
    const shared_ptr<NetworkLayer> initNetworkLayerPtr,
    const unsigned char initHopLimit)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    networkLayerPtr(initNetworkLayerPtr),
    currentSequenceNumber(0),
    numberEnabledInterfaces(0),
    hopLimit(initHopLimit)
{
    const NodeIdType nodeId = networkLayerPtr->GetNodeId();

    string mobileIpAddressString =
        theParameterDatabaseReader.ReadString("mobile-ip-address", nodeId);

    bool success;
    homeAddress.SetAddressFromString(mobileIpAddressString, nodeId, success);

    if (!success) {
        cerr << "Error: bad mobile-ip-address format: " << mobileIpAddressString;
        cerr << " for node: " << nodeId << endl;
        exit(1);
    }//if//

    string homeAgentAddressString =
        theParameterDatabaseReader.ReadString("mobile-ip-home-agent-address", nodeId);

    homeAgentAddress.SetAddressFromString(homeAgentAddressString, nodeId, success);

    if (!success) {
        cerr << "Error: bad mobile-ip-home-agent-address format: " << homeAgentAddressString;
        cerr << " for node: " << nodeId << endl;
        exit(1);
    }//if//

}//MobileIpMobileNodeSubsystem//


inline
void MobileIpMobileNodeSubsystem::AddInterfaceIfMobileIpEnabled(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const unsigned int interfaceIndex,
    const string& interfaceId)
{
    interfaceInfos.resize(interfaceIndex+1);

    if ((theParameterDatabaseReader.ParameterExists(
            "mobile-ip-enabled-interface",
            networkLayerPtr->GetNodeId(),
            interfaceId))
        &&
        (theParameterDatabaseReader.ReadBool(
             "mobile-ip-enabled-interface",
             networkLayerPtr->GetNodeId(),
             interfaceId)) == true) {

        // Add Mobile IP enabled interface.

        interfaceInfos.resize(interfaceIndex+1);

        interfaceInfos[interfaceIndex].isEnabledOnInterface = true;
        interfaceInfos[interfaceIndex].bindingId = numberEnabledInterfaces;
        numberEnabledInterfaces++;

        assert(numberEnabledInterfaces <= maxNumberEnabledInterfaces);

    }//if//

}//AddInterfaceIfMobileIpEnabled//


inline
void MobileIpMobileNodeSubsystem::HandleMajorInterfaceStatusChange(const unsigned int interfaceIndex)
{
    if ((interfaceIndex >= interfaceInfos.size()) ||
        (!interfaceInfos[interfaceIndex].isEnabledOnInterface)) {

        // Not a Mobile IP interface.
        return;
    }//if//

    NetworkAddress newAddress = networkLayerPtr->GetNetworkAddress(interfaceIndex);

    if ((newAddress == NetworkAddress::invalidAddress) ||
        (newAddress == interfaceInfos[interfaceIndex].currentCareOfAddress)) {

        // Disconnected or same address ==> nothing to do.
        return;
    }//if//

    IpHeaderModel ipHeader(
        0,
        0,
        hopLimit,
        IpHeaderModel::ipProtoNoneProtocolNumber,
        newAddress,
        homeAgentAddress);

    ipHeader.AddHomeAddressDestinationOptionsHeader(homeAddress);

    currentSequenceNumber++;

    ipHeader.AddBindingUpdateExtensionHeader(
        currentSequenceNumber,
        bindingLifetimein4SecUnits,
        interfaceInfos[interfaceIndex].bindingId);

    ipHeader.AddIpsecEspOverhead();

    unique_ptr<Packet> bindingUpdatePacketPtr = Packet::CreatePacket(*simEngineInterfacePtr);

    bindingUpdatePacketPtr->AddRawHeader(ipHeader.GetPointerToRawBytes(), ipHeader.GetNumberOfRawBytes());
    bindingUpdatePacketPtr->AddTrailingPadding(ipHeader.GetNumberOfTrailingBytes());

    networkLayerPtr->ReceiveOutgoingPreformedNetworkPacket(bindingUpdatePacketPtr);

}//HandleMajorInterfaceStatusChange//



inline
void MobileIpMobileNodeSubsystem::TunnelPacketToCorrespondentNode(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    PacketPriorityType trafficClass,
    const unsigned char protocol)
{
    // Add tunneled IP header.

    IpHeaderModel ipHeader(trafficClass, packetPtr->LengthBytes(), hopLimit, protocol, sourceAddress, destinationAddress);

    packetPtr->AddRawHeader(ipHeader.GetPointerToRawBytes(), ipHeader.GetNumberOfRawBytes());
    packetPtr->AddTrailingPadding(ipHeader.GetNumberOfTrailingBytes());

    // Call will slap on external IP header.

    networkLayerPtr->ReceivePacketFromUpperLayer(
        packetPtr,
        homeAgentAddress,
        trafficClass,
        IpHeaderModel::ipInIpProtocolNumber);

}//TunnelPacketToCorrespondentNode//



//--------------------------------------------------------------------------------------------------

class MobileIpHomeAgentSubsystem : public AbstractMobileIpRouteSelection{
public:
    MobileIpHomeAgentSubsystem(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface> initSimEngineInterfacePtr,
        const shared_ptr<NetworkLayer> initNetworkLayerPtr);

    void ProcessMobileIpProtocolOptions(const IpHeaderOverlayModel& ipHeader);

    void RoutePacketIfHaveCareOfAddress(
        const NetworkAddress& destinationAddress,
        unique_ptr<Packet>& packetPtr,
        bool& packetWasRouted);

private:
    void SelectOneOfCareOfAddressFromBindingCache(
        const NetworkAddress& destinationAddress,
        const map<unsigned short, NetworkAddress>& careOfAddressMap,
        NetworkAddress& tunnelDestAddress);
    struct BindingCacheElement {
        map<unsigned short, NetworkAddress> careOfAddress;
        unsigned short lastSequenceNumber;

        BindingCacheElement() :lastSequenceNumber(0) {}

    };//BindingCacheElement//
    int bindingCacheSelectionArgorithm;

    map<NetworkAddress, BindingCacheElement> bindingCache;

    typedef map<NetworkAddress, BindingCacheElement>::iterator BindingCacheIterator;

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    shared_ptr<NetworkLayer> networkLayerPtr;

    void ProcessBindingUpdate(const IpHeaderOverlayModel& ipHeader);

};//MobileIpHomeAgent//



MobileIpHomeAgentSubsystem::MobileIpHomeAgentSubsystem(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface> initSimEngineInterfacePtr,
    const shared_ptr<NetworkLayer> initNetworkLayerPtr)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    networkLayerPtr(initNetworkLayerPtr),
    bindingCacheSelectionArgorithm(AbstractMobileIpRouteSelection::FIRSTENTRY)
{
}


inline
void MobileIpHomeAgentSubsystem::ProcessBindingUpdate(const IpHeaderOverlayModel& ipHeader)
{
    const MobileIpBindingUpdateExtensionHeader& bindingUpdateHeader =
        ipHeader.GetMobileIpBindingUpdateHeader();

    assert(ipHeader.HomeAddressDestinationOptionsHeaderExists() && "Binding Update with no Home Address!");

    NetworkAddress homeAddress = ipHeader.GetHomeAddressFromDestinationOptionsHeader();

    BindingCacheElement& bindingCacheElem = bindingCache[homeAddress];

    if (!SequenceNumberIsLessThan(bindingCacheElem.lastSequenceNumber, bindingUpdateHeader.sequenceNumber)) {

        // Old Update => Do nothing
        return;
    }
    else {
        // Binding Update
        bindingCacheElem.careOfAddress[bindingUpdateHeader.bindingId] = ipHeader.GetSourceAddress();
        bindingCacheElem.lastSequenceNumber = bindingUpdateHeader.sequenceNumber;
    }
}//ProcessBindingUpdate//

inline
void MobileIpHomeAgentSubsystem::ProcessMobileIpProtocolOptions(const IpHeaderOverlayModel& ipHeader)
{
    if (ipHeader.MobileIpBindingUpdateHeaderExists()) {
        (*this).ProcessBindingUpdate(ipHeader);
    }//if//

}//ProcessMobileIpProtocolOptions//

inline
void MobileIpHomeAgentSubsystem::SelectOneOfCareOfAddressFromBindingCache(
        const NetworkAddress& destinationAddress,
        const map<unsigned short, NetworkAddress>& careOfAddressMap, NetworkAddress& tunnelDestAddress)
{
    if (bindingCacheSelectionArgorithm == ROUNDROBIN) {
        RoundRobinSelection(destinationAddress, careOfAddressMap, tunnelDestAddress);
    }
    else if (bindingCacheSelectionArgorithm == FIRSTENTRY) {
        FirstEntrySelection(careOfAddressMap, tunnelDestAddress);
    }
    // Add other algorithm
}//SelectOneOfCareOfAddressFromBindingCache//

inline
void MobileIpHomeAgentSubsystem::RoutePacketIfHaveCareOfAddress(
    const NetworkAddress& destinationAddress,
    unique_ptr<Packet>& packetPtr,
    bool& packetWasRouted)
{
    BindingCacheIterator bindingCacheIter = bindingCache.find(destinationAddress);

    if (bindingCacheIter == bindingCache.end()) {
        packetWasRouted = false;
        return;
    }//if//

    IpHeaderOverlayModel ipHeader(packetPtr->GetRawPayloadData(), packetPtr->LengthBytes());

    // Have care of address ==> Tunnel packet.
    NetworkAddress tunnelDestAddress;
    SelectOneOfCareOfAddressFromBindingCache(destinationAddress, bindingCacheIter->second.careOfAddress, tunnelDestAddress);
    networkLayerPtr->ReceivePacketFromUpperLayer(
        packetPtr,
        tunnelDestAddress,
        ipHeader.GetTrafficClass(),
        IpHeaderModel::ipInIpProtocolNumber);

    packetWasRouted = true;

}//RoutePacketIfHaveCareOfAddress//

// class MobileIpCorrespondentNodeSubsystem  ... If we have route optimization (direct routing).


}//namespace//

#endif


