// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_netsim.h"
#include "scensim_mobileip.h"
#include "scensim_arp.h"
#include "scensim_netif.h"
#include "scensim_dhcp.h"
#include "scensim_iscdhcp.h"

namespace ScenSim {


// Could be put in own "scensim_qoscontrol.cpp" file.

const FlowIdType FlowIdType::nullFlowId;


void BasicNetworkLayer::InitMobileIpMobileNodeSubsystem(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    assert(mobileIpHomeAgentSubsystemPtr == nullptr);

    mobileIpMobileNodeSubsystemPtr.reset(
        new MobileIpMobileNodeSubsystem(
            theParameterDatabaseReader,
            simulationEngineInterfacePtr,
            shared_from_this(),
            hopLimit));

    (*this).primaryNetworkAddress = mobileIpMobileNodeSubsystemPtr->GetHomeAddress();
}


void BasicNetworkLayer::InitMobileIpHomeAgentSubsystem(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    assert(mobileIpMobileNodeSubsystemPtr == nullptr);

    mobileIpHomeAgentSubsystemPtr.reset(
        new MobileIpHomeAgentSubsystem(
            theParameterDatabaseReader,
            simulationEngineInterfacePtr,
            shared_from_this()));
}


bool BasicNetworkLayer::IsOneOfMyMobileIpHomeAddresses(
    const NetworkAddress& anAddress, const unsigned int interfaceIndex)
{
    if (mobileIpMobileNodeSubsystemPtr == nullptr) {
        return false;
    }//if//
    return (mobileIpMobileNodeSubsystemPtr->GetHomeAddress() == anAddress);
}


void BasicNetworkLayer::SetInterfaceIpAddress(
    const unsigned int interfaceIndex,
    const NetworkAddress& newInterfaceAddress,
    const unsigned int subnetMaskLengthBits)
{
    NetworkInterfaceInfoType& interfaceInfo = networkInterfaces.at(interfaceIndex);

    if (interfaceInfo.address != newInterfaceAddress) {

        OutputTraceAndStatsForIpAddressChanged(
            interfaceIndex, newInterfaceAddress, subnetMaskLengthBits);

        interfaceInfo.address = newInterfaceAddress;
        interfaceInfo.subnetMaskLengthBits = subnetMaskLengthBits;
        interfaceInfo.subnetMask = NetworkAddress::MakeSubnetMask(subnetMaskLengthBits);
        if (interfaceInfo.isPrimary) {
            primaryNetworkAddress = newInterfaceAddress;
        }//if//

        if (mobileIpMobileNodeSubsystemPtr != nullptr) {
            mobileIpMobileNodeSubsystemPtr->HandleMajorInterfaceStatusChange(interfaceIndex);
        }//if//

        interfaceInfo.networkInterfaceManagerPtr->NotifyProtocolAddressChanged();

        for(size_t i = 0; i < networkAddressInterfaces.size(); ++i) {
            networkAddressInterfaces[i]->NotifyNetworkAddressIsChanged(
                interfaceIndex,
                interfaceInfo.address,
                interfaceInfo.subnetMaskLengthBits);
        }//for//
    }//if//

}//SetInterfaceIpAddress//


void BasicNetworkLayer::SetupInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceIdType& interfaceId)
{
    networkInterfaces.push_back(NetworkInterfaceInfoType());

    NetworkInterfaceInfoType& interfaceInfo = networkInterfaces.back();

    interfaceInfo.networkInterfaceManagerPtr.reset(
        new NetworkInterfaceManager(
            theParameterDatabaseReader,
            nodeId,
            interfaceId,
            static_cast<unsigned int>((networkInterfaces.size() - 1)),
            simulationEngineInterfacePtr,
            shared_from_this(),
            nodeSeed));

    interfaceInfo.interfaceName = interfaceId;
    string networkAddressString;
    if (theParameterDatabaseReader.ParameterExists("network-address", nodeId, interfaceId)) {
        networkAddressString = theParameterDatabaseReader.ReadString("network-address", nodeId, interfaceId);
    }
    else {
        cerr << "Error: No network-address for node and interface: "
             << nodeId << ' ' << interfaceId << endl;
        exit(1);
    }//if//

    bool success = false;

    interfaceInfo.address.SetAddressFromString(networkAddressString, nodeId, success);

    if (!success) {
        cerr << "Error: bad network-address format: " << networkAddressString;
        cerr << " for node: " << nodeId << endl;
        exit(1);

    }//if//

    interfaceInfo.networkInterfaceManagerPtr->NotifyProtocolAddressChanged();

    if (mobileIpMobileNodeSubsystemPtr != nullptr) {
        mobileIpMobileNodeSubsystemPtr->AddInterfaceIfMobileIpEnabled(
            theParameterDatabaseReader,
            static_cast<unsigned int>((networkInterfaces.size() - 1)),
            interfaceId);
    }//if//

    interfaceInfo.isPrimary = false;

    if (theParameterDatabaseReader.ParameterExists("network-address-is-primary", nodeId, interfaceId)) {

        if (theParameterDatabaseReader.ReadBool("network-address-is-primary", nodeId, interfaceId) == true) {

            if (!primaryNetworkAddress.IsAnyAddress()) {
                cerr << "Error: Too many primary network addresses for node: " << nodeId << endl;
                exit(1);
            }//if//

           interfaceInfo.isPrimary = true;
           (*this).primaryNetworkAddress = interfaceInfo.address;
        }//if//

    }//if//

    int prefixLengthBits;
    if (theParameterDatabaseReader.ParameterExists("network-prefix-length-bits", nodeId, interfaceId)) {
        prefixLengthBits = theParameterDatabaseReader.ReadInt("network-prefix-length-bits", nodeId, interfaceId);
    }
    else {
        cerr << "Error: No network-prefix-length-bits for node and interface: "
             << nodeId << ' ' << interfaceId << endl;
        exit(1);
    }//if//

    if ((prefixLengthBits <= 0) || (prefixLengthBits >= NetworkAddress::numberBits)) {
        cerr << "Error: Bad network-prefix-length-bits parameter, value = " << prefixLengthBits << endl;
        exit(1);

    }//if//

    if ((NetworkAddress::numberBits == 128) && (NetworkAddress::IsIpv4StyleAddressString(networkAddressString))) {
        //ajust prefix length from IPv4(32bit) style to IPv6(128bit) style
        prefixLengthBits += 96;
    }//if//

    interfaceInfo.subnetMaskLengthBits = prefixLengthBits;
    interfaceInfo.subnetMask = NetworkAddress::MakeSubnetMask(prefixLengthBits);

    interfaceInfo.subnetIsMultiHop = false;

    if (theParameterDatabaseReader.ParameterExists("network-subnet-is-multihop", nodeId, interfaceId)) {
        interfaceInfo.subnetIsMultiHop =
            theParameterDatabaseReader.ReadBool("network-subnet-is-multihop", nodeId, interfaceId);
    }//if//

    interfaceInfo.gatewayIsForcedNextHop = false;

    interfaceInfo.allowRoutingBackOutSameInterface = false;

    if (theParameterDatabaseReader.ParameterExists(
        "network-allow-routing-back-out-same-interface", nodeId, interfaceId)) {

        interfaceInfo.allowRoutingBackOutSameInterface =
            theParameterDatabaseReader.ReadBool(
            "network-allow-routing-back-out-same-interface", nodeId, interfaceId);
    }//if//

    interfaceInfo.ignoreUnregisteredProtocol = false;

    if (theParameterDatabaseReader.ParameterExists(
        "network-ignore-unregistered-protocol", nodeId, interfaceId)) {

        interfaceInfo.ignoreUnregisteredProtocol =
            theParameterDatabaseReader.ReadBool(
            "network-ignore-unregistered-protocol", nodeId, interfaceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("network-gateway-address", nodeId, interfaceId)) {

        string gatewayAddressString = theParameterDatabaseReader.ReadString("network-gateway-address", nodeId, interfaceId);

        bool success = false;

        interfaceInfo.gatewayAddress.SetAddressFromString(gatewayAddressString, nodeId, success);

        if (!success) {
            cerr << "Error: bad network-gateway-address format: " << gatewayAddressString;
            cerr << " for node: " << nodeId << endl;
            exit(1);

        }//if//

        if (gatewayAddressExists) {
            cerr << "Error: two interfaces on node: " << nodeId << " have specified default gateways." << endl;
            exit(1);

        }//if//


        if(!interfaceInfo.gatewayAddress.IsInSameSubnetAs(interfaceInfo.address, interfaceInfo.subnetMask)) {
            cerr << "Error: on node/interface: " << nodeId << "/" << interfaceId
                 << " specified gateway is not in the interface's subnet." << endl;
            exit(1);

        }//if//

        (*this).gatewayAddressExists = true;
        (*this).gatewayInterfaceIndex = static_cast<unsigned int>(networkInterfaces.size() - 1);

        if (primaryNetworkAddress != NetworkAddress::invalidAddress) {
            (*this).primaryNetworkAddress = interfaceInfo.address;
        }//if//

    }//if//

    if (theParameterDatabaseReader.ParameterExists("network-mtu-bytes", nodeId, interfaceId)) {
        interfaceInfo.maxIpPacketSizeAkaMtuBytes =
            theParameterDatabaseReader.ReadNonNegativeInt("network-mtu-bytes", nodeId, interfaceId);
    }//if//

    OutputTraceAndStatsForIpAddressChanged(
        static_cast<unsigned int>((networkInterfaces.size() - 1)),
        interfaceInfo.address,
        interfaceInfo.subnetMaskLengthBits);

}//SetupInterface//



void BasicNetworkLayer::ReceiveOutgoingPreformedNetworkPacket(unique_ptr<Packet>& packetPtr)
{
    const IpHeaderOverlayModel ipHeader(packetPtr->GetRawPayloadData(), packetPtr->LengthBytes());

    if ((mobileIpMobileNodeSubsystemPtr != nullptr) &&
        (mobileIpMobileNodeSubsystemPtr->GetHomeAddress() == ipHeader.GetSourceAddress())) {

        mobileIpMobileNodeSubsystemPtr->TunnelPacketToCorrespondentNode(
            packetPtr,
            ipHeader.GetSourceAddress(),
            ipHeader.GetDestinationAddress(),
            ipHeader.GetTrafficClass(),
            ipHeader.GetNextHeaderProtocolCode());

        return;

    }//if//


    bool foundARoute;
    unsigned int interfaceIndex;
    NetworkAddress nextHopAddress;

    GetNextHopAddressAndInterfaceIndexForDestination(
        ipHeader.GetDestinationAddress(), foundARoute, nextHopAddress, interfaceIndex);

    if (!foundARoute) {
        bool wasAcceptedForRouting = false;

        (*this).GiveNetworkPacketToOnDemandRoutingProtocol(
            packetPtr,
            ipHeader.GetSourceAddress(),
            ipHeader.GetDestinationAddress(),
            wasAcceptedForRouting);

        if (!wasAcceptedForRouting) {
            // Drop packet.

            OutputTraceAndStatsForNoRouteDrop(*packetPtr, ipHeader.GetDestinationAddress());

            packetPtr = nullptr;
        }//if//

        return;
    }//if//

    (*this).InsertPacketIntoAnOutputQueue(
        packetPtr,
        interfaceIndex,
        nextHopAddress,
        ipHeader.GetTrafficClass());

}//ReceiveOutgoingPreformedNetworkPacket//


inline
void BasicNetworkLayer::SendBroadcastOrMulticastPacket(
    unique_ptr<Packet>& packetPtr,
    const unsigned int interfaceIndex,
    const NetworkAddress& destinationAddress,
    const PacketPriorityType trafficClass,
    const unsigned char protocol)
{
    IpHeaderModel
        header(
            trafficClass,
            packetPtr->LengthBytes(),
            hopLimit,
            protocol,
            networkInterfaces.at(interfaceIndex).address,
            destinationAddress);

    packetPtr->AddRawHeader(header.GetPointerToRawBytes(), header.GetNumberOfRawBytes());
    packetPtr->AddTrailingPadding(header.GetNumberOfTrailingBytes());

    (*this).InsertPacketIntoAnOutputQueue(
        packetPtr, interfaceIndex, destinationAddress, trafficClass);

}//SendBroadcastOrMulticastPacket//


void BasicNetworkLayer::BroadcastPacketOnAllInterfaces(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destinationAddress,
    const PacketPriorityType trafficClass,
    const unsigned char protocol)
{
    if (networkInterfaces.size() == 0) {
        packetPtr = nullptr;
        return;
    }//if//

    for(unsigned int interfaceIndex = 0; (interfaceIndex < networkInterfaces.size()); interfaceIndex++) {
        unique_ptr<Packet> packetToSendPtr;

        if (interfaceIndex < (networkInterfaces.size() - 1)) {
            // Copy packet
            packetToSendPtr = unique_ptr<Packet>(new Packet(*packetPtr));
        }
        else {
            // Last interface, send original packet.

            packetToSendPtr = move(packetPtr);
        }//if//

        SendBroadcastOrMulticastPacket(
            packetToSendPtr,
            interfaceIndex,
            destinationAddress,
            trafficClass,
            protocol);

    }//for//

}//BroadcastPacketOnAllInterfaces//


NetworkAddress BasicNetworkLayer::GetSourceAddressForDestination(const NetworkAddress& destinationAddress) const
{
    if (mobileIpMobileNodeSubsystemPtr != nullptr) {
        return (mobileIpMobileNodeSubsystemPtr->GetHomeAddress());
    }//if//

    bool success;
    NetworkAddress notUsed;
    unsigned int interfaceIndex;
    (*this).GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress,
        success,
        notUsed,
        interfaceIndex);

    if (!success) {
        cerr << "Error in GetSourceAddressForDestination: At node " << nodeId << " Destination Address: "
             << destinationAddress.ConvertToString() << " is not reachable." << endl;
        exit(1);
    }//if//

    return networkInterfaces.at(interfaceIndex).address;

}//GetSourceAddressForDestination//


void BasicNetworkLayer::ReceivePacketFromUpperLayer(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& initialSourceAddress,
    const NetworkAddress& destinationAddress,
    PacketPriorityType trafficClass,
    const unsigned char protocol)
{
    if (NetworkAddressIsMyAddress(destinationAddress)) {
        ProcessLoopbackPacket(packetPtr, initialSourceAddress, destinationAddress, trafficClass, protocol);
        return;
    }//if//

    NetworkAddress sourceAddress = initialSourceAddress;

    // Check Mobile IP to see if we are a Mobile Node and the packet must go through Home Agent.

    if ((mobileIpMobileNodeSubsystemPtr != nullptr) &&
        (mobileIpMobileNodeSubsystemPtr->GetHomeAddress() == sourceAddress)) {

        mobileIpMobileNodeSubsystemPtr->TunnelPacketToCorrespondentNode(
            packetPtr, sourceAddress, destinationAddress, trafficClass, protocol);

        return;

    }//if//

    //Jay// // Check Mobile IP to see if we are a Home Agent for the address.
    //Jay//
    //Jay// if (mobileIpHomeAgentSubsystemPtr != nullptr) {
    //Jay//     bool packetWasRouted = false;
    //Jay//
    //Jay//     mobileIpHomeAgentSubsystemPtr->TunnelPacketIfHaveCareOfAddress(
    //Jay//         destinationAddress, packetPtr, packetWasRouted);
    //Jay//
    //Jay//     if (packetWasRouted) {
    //Jay//         return;
    //Jay//     }//if//
    //Jay//
    //Jay// }//if//

    if ((sourceAddress.IsAnyAddress()) &&
        (destinationAddress.IsTheBroadcastOrAMulticastAddress())) {

        (*this).BroadcastPacketOnAllInterfaces(
            packetPtr, destinationAddress, trafficClass, protocol);

        return;

    }//if//

    bool foundARoute = false;
    unsigned int interfaceIndex;
    NetworkAddress nextHopAddress;

    if (destinationAddress.IsTheBroadcastOrAMulticastAddress() ||
        destinationAddress.IsLinkLocalAddress()) {

        foundARoute = true;
        nextHopAddress = destinationAddress;
        interfaceIndex = LookupInterfaceIndex(sourceAddress);
    }
    else {
        GetNextHopAddressAndInterfaceIndexForDestination(
            destinationAddress, foundARoute, nextHopAddress, interfaceIndex);
    }//if//

    if (foundARoute) {
        (*this).NotifySendingOrForwardingDataPacketToOnDemandRoutingProtocol(sourceAddress, sourceAddress, nextHopAddress, destinationAddress);
    }
    else {
        bool wasAcceptedForRouting = false;

        IpHeaderModel
            header(
                trafficClass,
                packetPtr->LengthBytes(),
                hopLimit,
                protocol,
                NetworkAddress::anyAddress, // Null Address, will be set later.
                destinationAddress);

        packetPtr->AddRawHeader(header.GetPointerToRawBytes(), header.GetNumberOfRawBytes());
        packetPtr->AddTrailingPadding(header.GetNumberOfTrailingBytes());

        (*this).GiveNetworkPacketToOnDemandRoutingProtocol(
            packetPtr,
            sourceAddress,
            destinationAddress,
            wasAcceptedForRouting);

        if (!wasAcceptedForRouting) {
            // Drop packet.

            OutputTraceAndStatsForNoRouteDrop(*packetPtr, destinationAddress);

            packetPtr = nullptr;
        }//if//

        return;
    }//if//

    if (sourceAddress == NetworkAddress::anyAddress) {
        sourceAddress = networkInterfaces.at(interfaceIndex).address;
    }//if//

    IpHeaderModel
        header(
            trafficClass,
            packetPtr->LengthBytes(),
            hopLimit,
            protocol,
            sourceAddress,
            destinationAddress);

    packetPtr->AddRawHeader(header.GetPointerToRawBytes(), header.GetNumberOfRawBytes());
    packetPtr->AddTrailingPadding(header.GetNumberOfTrailingBytes());

    (*this).InsertPacketIntoAnOutputQueue(packetPtr, interfaceIndex, nextHopAddress, trafficClass);

}//ReceivePacketFromUpperLayer//


void BasicNetworkLayer::ReceiveOutgoingBroadcastPacket(
    unique_ptr<Packet>& packetPtr,
    const unsigned int interfaceIndex,
    const PacketPriorityType trafficClass,
    const unsigned char protocol)
{
    (*this).SendBroadcastOrMulticastPacket(
        packetPtr,
        interfaceIndex,
        NetworkAddress::broadcastAddress,
        trafficClass,
        protocol);

}//ReceiveOutgoingBroadcastPacket//



void BasicNetworkLayer::ReceivePacketFromMac(
    const unsigned int interfaceIndex,
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& lastHopAddress,
    const EtherTypeFieldType etherType)
{
    if (etherType == ETHERTYPE_ARP) {
        NetworkInterfaceInfoType& interface = networkInterfaces.at(interfaceIndex);
        interface.networkInterfaceManagerPtr->ProcessArpPacket(packetPtr);
        assert(packetPtr == nullptr);
        return;
    }//if//

    OutputTraceAndStatsForReceivePacketFromMac(*packetPtr);

    IpHeaderOverlayModel ipHeader(packetPtr->GetRawPayloadData(), packetPtr->LengthBytes());

    NetworkAddress sourceAddress(ipHeader.GetSourceAddress());
    NetworkAddress destinationAddress(ipHeader.GetDestinationAddress());
    PacketPriorityType trafficClass(ipHeader.GetTrafficClass());
    unsigned char hopLimit(ipHeader.GetHopLimit());
    assert(hopLimit != 0);

    if (NetworkAddressIsForThisNode(destinationAddress, interfaceIndex)) {
        // Packet is for me or everyone, send it up the stack.

        unsigned char protocolNum;
        unsigned int ipHeaderLength;
        ipHeader.GetHeaderTotalLengthAndNextHeaderProtocolCode(ipHeaderLength, protocolNum);

        if ((mobileIpHomeAgentSubsystemPtr != nullptr) &&
            (ipHeader.MobilityExtensionHeaderExists())) {

            mobileIpHomeAgentSubsystemPtr->ProcessMobileIpProtocolOptions(ipHeader);
            assert(protocolNum == IpHeaderModel::ipProtoNoneProtocolNumber);
            packetPtr = nullptr;
            return;
        }//if//

        ipHeader.StopOverlayingHeader();
        packetPtr->DeleteHeader(ipHeaderLength);

        if (protocolNum == IpHeaderModel::ipInIpProtocolNumber) {

            (*this).ReceivePacketFromMac(interfaceIndex, packetPtr, lastHopAddress, etherType);
            return;

        }//if//

        if (protocolNum == IP_PROTOCOL_NUMBER_ICMP) {
            NetworkInterfaceInfoType& interface = networkInterfaces.at(interfaceIndex);
            interface.networkInterfaceManagerPtr->ProcessIcmpPacket(
                packetPtr,
                sourceAddress,
                destinationAddress,
                trafficClass,
                lastHopAddress);
            if (packetPtr != nullptr) {
                if (!networkInterfaces.at(interfaceIndex).ignoreUnregisteredProtocol) {
                    assert(false && "An upper layer cannot be prepared.");
                }//if//
                packetPtr = nullptr;
            }//if//
            return;
        }//if//

        map<unsigned char, shared_ptr<ProtocolPacketHandler> >::iterator mapIter =
            protocolPacketHandlerMap.find(protocolNum);

        if (mapIter != protocolPacketHandlerMap.end()) {
            mapIter->second->ReceivePacketFromNetworkLayer(
                packetPtr,
                sourceAddress,
                destinationAddress,
                trafficClass,
                lastHopAddress,
                hopLimit,
                interfaceIndex);
        }
        else {
            if (!networkInterfaces.at(interfaceIndex).ignoreUnregisteredProtocol) {
                assert(false && "An upper layer cannot be prepared.");
            }//if//

            packetPtr = nullptr;
        }//if//
    }
    else {
        // "Routing" packet to new interface.

        hopLimit -= 1;
        if (hopLimit == 0) {
            OutputTraceAndStatsForHopLimitDrop(*packetPtr, destinationAddress);

            packetPtr = nullptr;

            return;
        }
        else {
            ipHeader.SetHopLimit(hopLimit);
        }//if//

        bool foundARoute;
        unsigned int newInterfaceIndex;
        NetworkAddress nextHopAddress;

        // Try Mobile IP to see if we are a Home Agent for the address.

        if (mobileIpHomeAgentSubsystemPtr != nullptr) {
            bool packetWasRouted = false;

            mobileIpHomeAgentSubsystemPtr->RoutePacketIfHaveCareOfAddress(
                destinationAddress, packetPtr, packetWasRouted);

            if (packetWasRouted) {
                return;
            }//if//

        }//if//

        GetNextHopAddressAndInterfaceIndexForDestination(
            destinationAddress, foundARoute, nextHopAddress, newInterfaceIndex);

        if (foundARoute) {
            (*this).NotifySendingOrForwardingDataPacketToOnDemandRoutingProtocol(lastHopAddress, sourceAddress, nextHopAddress, destinationAddress);
        }
        else {
            bool wasAcceptedForRouting = false;

            (*this).GiveNetworkPacketToOnDemandRoutingProtocol(
                packetPtr,
                sourceAddress,
                destinationAddress,
                lastHopAddress,
                wasAcceptedForRouting);

            if (!wasAcceptedForRouting) {
                // Drop packet.

                OutputTraceAndStatsForNoRouteDrop(*packetPtr, destinationAddress);

                packetPtr = nullptr;

            }//if//

            return;

        }//if//

        if ((newInterfaceIndex == interfaceIndex) &&
            (!networkInterfaces.at(interfaceIndex).allowRoutingBackOutSameInterface)) {

            cerr << "Packet routed back out same interface and " << endl
                 << "   network-allow-routing-back-out-same-interface not set (true)." << endl;
            exit(1);

        }//if//

        (*this).InsertPacketIntoAnOutputQueue(packetPtr, newInterfaceIndex, nextHopAddress, trafficClass);

    }//if//

    assert(packetPtr == nullptr);

}//ReceivePacketFromMac//


void BasicNetworkLayer::GetTransportLayerPortNumbersFromIpPacket(
    const Packet& aPacket,
    bool& portNumbersWereRetrieved,
    unsigned short int& sourcePort,
    unsigned short int& destinationPort)
{
    typedef map<unsigned char, shared_ptr<ProtocolPacketHandler> >::const_iterator IterType;
    const IpHeaderOverlayModel ipHeader(aPacket.GetRawPayloadData(), aPacket.LengthBytes());

    portNumbersWereRetrieved = false;

    unsigned char protocolNum;
    unsigned int ipHeaderLength;
    ipHeader.GetHeaderTotalLengthAndNextHeaderProtocolCode(ipHeaderLength, protocolNum);

    IterType foundIter = protocolPacketHandlerMap.find(protocolNum);
    if (foundIter != protocolPacketHandlerMap.end()) {
        const ProtocolPacketHandler& protocolHandler = *foundIter->second;

        protocolHandler.GetPortNumbersFromPacket(
            aPacket,
            ipHeaderLength,
            portNumbersWereRetrieved,
            sourcePort,
            destinationPort);

    }//if//

}//GetTransportLayerPortNumbersFromIpPacket//


void BasicNetworkLayer::ProcessLinkIsUpNotification(const unsigned int interfaceIndex)
{
    NetworkInterfaceInfoType& interface = networkInterfaces.at(interfaceIndex);
    interface.networkInterfaceManagerPtr->ProcessLinkIsUpNotification();

    if (interface.dhcpClientInterfacePtr != nullptr) {
        interface.dhcpClientInterfacePtr->HandleLinkIsUpNotification(
            interfaceIndex,
            interface.macLayerPtr->GetGenericMacAddress());
    }//if//

}//ProcessLinkIsUpNotification//


void BasicNetworkLayer::ProcessLinkIsDownNotification(const unsigned int interfaceIndex)
{
    NetworkInterfaceInfoType& interface = networkInterfaces.at(interfaceIndex);
    interface.networkInterfaceManagerPtr->ProcessLinkIsDownNotification();
}


void BasicNetworkLayer::ProcessNewLinkToANodeNotification(
    const unsigned int interfaceIndex,
    const GenericMacAddressType& newNodeMacAddress)
{
    NetworkInterfaceInfoType& interface = networkInterfaces.at(interfaceIndex);
    interface.networkInterfaceManagerPtr->ProcessNewLinkToANodeNotification(newNodeMacAddress);

    if (interface.networkInterfaceManagerPtr->IsAddressResolutionEnabled()) {
        if (interfaceIndex == gatewayInterfaceIndex) {
            interface.gatewayAddress =
                NetworkAddress(
                    GetSubnetAddress(interfaceIndex),
                    NetworkAddress(CalcNodeId(newNodeMacAddress)));
        }//if//
    }//if//

}//ProcessNewLinkToANodeNotification//


void BasicNetworkLayer::InsertPacketIntoAnOutputQueue(
    unique_ptr<Packet>& packetPtr,
    const unsigned int interfaceIndex,
    const NetworkAddress& nextHopAddress,
    const PacketPriorityType initialTrafficClass,
    const EtherTypeFieldType etherType)
{
    NetworkInterfaceInfoType interface = networkInterfaces.at(interfaceIndex);
    
    if ((interface.outputQueuePtr == nullptr) ||
        (interface.macLayerPtr == nullptr)) {
        cerr << "Error in Network Layer Model: No communication interface(or mac) to send packet." << endl;
        exit(1);
    }//if//
    

    if (packetPtr->LengthBytes() > interface.maxIpPacketSizeAkaMtuBytes) {
        cerr << "Error in Network Layer Model: IP Packet size is too large for interface." << endl;
        cerr << "  IP Packet Size = " << packetPtr->LengthBytes()
             << "  Max MTU = " << interface.maxIpPacketSizeAkaMtuBytes << endl;
        exit(1);
    }//if//



    PacketPriorityType trafficClass = initialTrafficClass;
    if (trafficClass == MAX_AVAILABLE_PACKET_PRIORITY) {
        trafficClass = interface.outputQueuePtr->MaxPossiblePacketPriority();
    }//if//

    if (!interface.networkInterfaceManagerPtr->IsAddressResolutionCompleted(nextHopAddress)) {
        interface.networkInterfaceManagerPtr->SendAddressResolutionRequest(
            packetPtr, nextHopAddress, trafficClass);
        return;
    }//if//

    EnqueueResultType enqueueResult;
    unique_ptr<Packet> packetToDropPtr;

    OutputTraceAndStatsForInsertPacketIntoQueue(*packetPtr);

    InterfaceOutputQueue& outputQueue = *interface.outputQueuePtr;

    if (!outputQueue.InsertWithFullPacketInformationModeIsOn()) {
        outputQueue.Insert(packetPtr, nextHopAddress, trafficClass, enqueueResult, packetToDropPtr, etherType);
    }
    else {
        const IpHeaderOverlayModel ipHeader(packetPtr->GetRawPayloadData(), packetPtr->LengthBytes());

        bool portNumbersWereRetrieved;
        unsigned short int sourcePort = ANY_PORT;
        unsigned short int destinationPort = ANY_PORT;

        GetTransportLayerPortNumbersFromIpPacket(
            *packetPtr,
            portNumbersWereRetrieved,
            sourcePort,
            destinationPort);

        unsigned short int ipv6FlowLabel = NULL_FLOW_LABEL;

        if (IpHeaderModel::usingVersion6) {
            ipv6FlowLabel = ipHeader.GetFlowLabel();
        }//if//

        outputQueue.InsertWithFullPacketInformation(
            packetPtr,
            nextHopAddress,
            ipHeader.GetSourceAddress(),
            sourcePort,
            ipHeader.GetDestinationAddress(),
            destinationPort,
            ipHeader.GetNextHeaderProtocolCode(),
            trafficClass,
            ipv6FlowLabel,
            enqueueResult,
            packetToDropPtr);

    }//if//

    if (enqueueResult != ENQUEUE_SUCCESS) {

        OutputTraceAndStatsForFullQueueDrop(*packetToDropPtr, enqueueResult);

        packetToDropPtr = nullptr;
        packetPtr = nullptr;
    }//if//

    interface.macLayerPtr->NetworkLayerQueueChangeNotification();

}//InsertPacketIntoAnOutputQueue//


void BasicNetworkLayer::SetupDhcpServerAndClientIfNessesary(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const NodeIdType& nodeId,
    const shared_ptr<ApplicationLayer>& appLayerPtr)
{

    assert(appLayerPtr != nullptr);

    shared_ptr<DhcpClientInterface> dhcpClientInterfacePtr;
    shared_ptr<IscDhcpApplication> iscDhcpServerAppPtr;

    for(unsigned int interfaceIndex = 0; (interfaceIndex < networkInterfaces.size()); interfaceIndex++) {

        const InterfaceIdType interfaceId = (*this).GetInterfaceId(interfaceIndex);

        if ((theParameterDatabaseReader.ParameterExists("network-enable-dhcp-server", nodeId, interfaceId)) &&
            (theParameterDatabaseReader.ReadBool("network-enable-dhcp-server", nodeId, interfaceId))) {

            const string dhcpModel = MakeLowerCaseString(
                theParameterDatabaseReader.ReadString("network-dhcp-model", nodeId, interfaceId));

            if (dhcpModel == "abstract") {
                shared_ptr<DhcpServerApplication> dhcpServerAppPtr(
                    new DhcpServerApplication(
                        theParameterDatabaseReader,
                        simulationEngineInterfacePtr,
                        nodeId,
                        interfaceId,
                        interfaceIndex,
                        networkInterfaces[interfaceIndex].address,
                        networkInterfaces[interfaceIndex].subnetMaskLengthBits));
                appLayerPtr->AddApp(dhcpServerAppPtr);
                dhcpServerAppPtr->CompleteInitialization();
            }
            else if (dhcpModel == "isc") {
                if (iscDhcpServerAppPtr == nullptr) {
                    iscDhcpServerAppPtr = shared_ptr<IscDhcpApplication>(
                        new IscDhcpApplication(simulationEngineInterfacePtr, nodeId, true));
                    appLayerPtr->AddApp(iscDhcpServerAppPtr);
                    iscDhcpServerAppPtr->CompleteInitialization(theParameterDatabaseReader);
                }//if//
                assert(iscDhcpServerAppPtr);
                iscDhcpServerAppPtr->EnableForThisInterface(
                    theParameterDatabaseReader,
                    interfaceId,
                    interfaceIndex);
            }
            else {
                cerr << "Error: network-dhcp-model(" << dhcpModel
                     << ") should be abstract or isc" << endl;
                exit(1);
            }//if//
        }//if//

        if ((theParameterDatabaseReader.ParameterExists("network-enable-dhcp-client", nodeId, interfaceId)) &&
            (theParameterDatabaseReader.ReadBool("network-enable-dhcp-client", nodeId, interfaceId))) {

            const string dhcpModel = MakeLowerCaseString(
                theParameterDatabaseReader.ReadString("network-dhcp-model", nodeId, interfaceId));

            if (dhcpModel == "abstract") {
                if (dhcpClientInterfacePtr == nullptr) {
                    shared_ptr<DhcpClientApplication> dhcpClientAppPtr(
                        new DhcpClientApplication(
                            theParameterDatabaseReader,
                            simulationEngineInterfacePtr,
                            nodeId,
                            networkInterfaces.size()));
                    appLayerPtr->AddApp(dhcpClientAppPtr);
                    dhcpClientAppPtr->CompleteInitialization();
                    dhcpClientInterfacePtr = dhcpClientAppPtr;
                }//if//
                networkInterfaces.at(interfaceIndex).dhcpClientInterfacePtr = dhcpClientInterfacePtr;
                dhcpClientInterfacePtr->EnableForThisInterface(
                    theParameterDatabaseReader,
                    interfaceId,
                    interfaceIndex);
            }
            else if (dhcpModel == "isc") {
                if (dhcpClientInterfacePtr == nullptr) {
                    shared_ptr<IscDhcpApplication> iscDhcpClientAppPtr(
                        new IscDhcpApplication(simulationEngineInterfacePtr, nodeId, false));
                    appLayerPtr->AddApp(iscDhcpClientAppPtr);
                    iscDhcpClientAppPtr->CompleteInitialization(theParameterDatabaseReader);
                    dhcpClientInterfacePtr = iscDhcpClientAppPtr;
                }//if//
                networkInterfaces.at(interfaceIndex).dhcpClientInterfacePtr = dhcpClientInterfacePtr;
                dhcpClientInterfacePtr->EnableForThisInterface(
                    theParameterDatabaseReader,
                    interfaceId,
                    interfaceIndex);
            }
            else {
                cerr << "Error: network-dhcp-model(" << dhcpModel
                     << ") should be abstract or isc" << endl;
                exit(1);
            }//if//
        }//if//
    }//for//

}//SetupDhcpServerAndClientIfNessesary//


}//namespace//
