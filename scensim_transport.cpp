// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_transport.h"
#include "scensim_network.h"

namespace ScenSim {



void UdpProtocol::ConnectToNetworkLayer(const shared_ptr<NetworkLayer>& initNetworkLayerPtr)
{
    this->networkLayerPtr = initNetworkLayerPtr;

    networkLayerPtr->RegisterPacketHandlerForProtocol(IP_PROTOCOL_NUMBER_UDP, shared_from_this());
}


void UdpProtocol::SendPacket(
    unique_ptr<Packet>& packetPtr,
    const unsigned short int sourcePort,
    const NetworkAddress& destinationAddress,
    const unsigned short int destinationPort,
    const PacketPriorityType& priority)
{
    packetPtr->AddPlainStructHeader(
        UdpHeaderType(
            HostToNet16(sourcePort),
            HostToNet16(destinationPort),
            HostToNet16(static_cast<unsigned short int>(
                packetPtr->LengthBytes() + sizeof(UdpHeaderType)))));

    OutputTraceAndStatsForSendPacketToNetworkLayer(*packetPtr);

    networkLayerPtr->ReceivePacketFromUpperLayer(
        packetPtr,
        destinationAddress,
        priority,
        IP_PROTOCOL_NUMBER_UDP);
}


void UdpProtocol::SendPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const unsigned short int sourcePort,
    const NetworkAddress& destinationAddress,
    const unsigned short int destinationPort,
    const PacketPriorityType& priority)
{
    packetPtr->AddPlainStructHeader(
        UdpHeaderType(
            HostToNet16(sourcePort),
            HostToNet16(destinationPort),
            HostToNet16(static_cast<unsigned short int>(
                packetPtr->LengthBytes() + sizeof(UdpHeaderType)))));

    OutputTraceAndStatsForSendPacketToNetworkLayer(*packetPtr);

    networkLayerPtr->ReceivePacketFromUpperLayer(
        packetPtr,
        sourceAddress,
        destinationAddress,
        priority,
        IP_PROTOCOL_NUMBER_UDP);
}//SendPacket//


void UdpProtocol::ReceivePacketFromNetworkLayer(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const PacketPriorityType trafficClass,
    const NetworkAddress& lastHopAddress_notused,
    const unsigned char hopLimit_notused,
    const unsigned int interfaceIndex)
{
    OutputTraceAndStatsForReceivePacketFromNetworkLayer(*packetPtr);

    UdpHeaderType udpHeader = packetPtr->GetAndReinterpretPayloadData<UdpHeaderType>();
    const unsigned short int sourcePort = NetToHost16(udpHeader.sourcePort);
    const unsigned short int destinationPort = NetToHost16(udpHeader.destinationPort);

    packetPtr->DeleteHeader(sizeof(UdpHeaderType));

    typedef map<PortKeyType, shared_ptr<PacketForAppFromTransportLayerHandler> >::iterator IterType;


    const IterType anyAddressIter = portMap.find(PortKeyType(destinationPort, NetworkAddress::anyAddress));

    if (anyAddressIter != portMap.end()) {
        //any bind

        //deliver to application
        anyAddressIter->second->ReceivePacket(
            packetPtr, sourceAddress, sourcePort, destinationAddress, trafficClass);
    }
    else {
        //address specific bind

        NetworkAddress portAddress = destinationAddress;
        if (destinationAddress.IsABroadcastOrAMulticastAddress(
            networkLayerPtr->GetSubnetMask(interfaceIndex))) {

            portAddress = networkLayerPtr->GetNetworkAddress(interfaceIndex);
        }//if//

        const IterType addressSpecificIter = portMap.find(PortKeyType(destinationPort, portAddress));
        if (addressSpecificIter != portMap.end()) {
            //deliver to application
            addressSpecificIter->second->ReceivePacket(
                packetPtr, sourceAddress, sourcePort, destinationAddress, trafficClass);
        }
        else {
            //ignore packet
            packetPtr = nullptr;
            //TBD: output trace and stat
        }//if

    }//if//

}//ReceivePacketFromNetworkLayer//


}//namespace//
