// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_DHCP_H
#define SCENSIM_DHCP_H

#include <memory>

#include "scensim_application.h"
#include "scensim_transport.h"
#include "scensim_network.h"


namespace ScenSim {

using std::shared_ptr;
using std::enable_shared_from_this;


//DHCP(Dynamic Host Configuration Protocol), RFC 2131

enum DhcpMessageType {

    DHCP_DISCOVER = 1,
    DHCP_OFFER = 2,
    DHCP_REQUEST = 3,
    DHCP_ACK = 4,
    //DHCP_NAK
    //DHCP_DECLINE
    //DHCP_RELEASE
    //DHCP_INFORM

};

static const unsigned int DHCP_MESSAGE_SIZE_BYTES = 300;

struct DhcpMessageFormat {

    unsigned char notUsed_opCode;
    unsigned char notUsed_hardwareAddredssType;
    unsigned char notUsed_hardwareAddredssLength;
    unsigned char notUsed_hops;

    uint32_t notUsed_transactionId;

    uint16_t notUsed_secs;
    uint16_t notUsed_flags;

    NetworkAddress notUsed_clientIpAddress;
    NetworkAddress yourIpAddress;
    NetworkAddress serverIpAddress;
    NetworkAddress notUsed_gatewayIpAddress;

    GenericMacAddressType clientHardwareAddress;

    //sname (optional server host name): 64bytes

    //file (boot file name): 128 bytes

    //options (vender specific)

    DhcpMessageType messageType;

    NetworkAddress defaultGatewayIpAddress;
    unsigned int subnetMaskLengthBits;


    DhcpMessageFormat(
        const DhcpMessageType initMessageType,
        const GenericMacAddressType& initClientHardwareAddress = 0)
        :
        messageType(initMessageType),
        clientHardwareAddress(initClientHardwareAddress),
        yourIpAddress(NetworkAddress::invalidAddress),
        serverIpAddress(NetworkAddress::invalidAddress),
        defaultGatewayIpAddress(NetworkAddress::invalidAddress),
        subnetMaskLengthBits(0)
    {}

};//DhcpMessageFormat//


//--------------------------------------------------------------
//--------------------------------------------------------------
class DhcpServerApplication : public Application,
    public enable_shared_from_this<DhcpServerApplication> {

public:

    DhcpServerApplication(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const InterfaceIdType& initInterfaceId,
        const unsigned int initInterfaceIndex,
        const NetworkAddress& initServerNetworkAddress,
        const unsigned int initSubnetMaskLengthBits);

    ~DhcpServerApplication()
    { assert(initializationIsComplete); }

    void CompleteInitialization();

private:
    NodeIdType nodeId;
    InterfaceIdType interfaceId;
    unsigned int interfaceIndex;
    NetworkAddress serverNetworkAddress;
    NetworkAddress serverSubnetAddress;
    unsigned int subnetMaskLengthBits;

    PacketPriorityType packetPriority;
    NetworkAddress defaultGatewayNetworkAddress;

    bool initializationIsComplete;

    enum NetworkAddressStatus {
        UNASSIGNED,
        RESERVED,
        ASSIGNED
    };

    struct NetworkAddressInfo {
        NetworkAddress networkAddress;
        NetworkAddressStatus status;

        NetworkAddressInfo(
            const NetworkAddress& initNetworkAddress = NetworkAddress::invalidAddress,
            const NetworkAddressStatus& initStatus = UNASSIGNED)
            :
            networkAddress(initNetworkAddress),
            status(initStatus)
        {}
    };//NetworkAddressInfo//

    map<GenericMacAddressType, NetworkAddressInfo> assignedNetworkAddresses;

    NetworkAddress AssignANetworkAddress(
        const GenericMacAddressType& targetMacAddress) const;

    //interface from UDP
    class UdpPacketHandler: public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        UdpPacketHandler(
            const shared_ptr<DhcpServerApplication>& initDhcpServerAppPtr)
            :
            dhcpServerAppPtr(initDhcpServerAppPtr)
        {}

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePort,
            const NetworkAddress& destinationAddress,
            const PacketPriorityType& priority)
        {
            dhcpServerAppPtr->ReceivePacket(packetPtr, sourceAddress);
        }//ReceivePacket//

    private:
        shared_ptr<DhcpServerApplication> dhcpServerAppPtr;

    };//UdpPacketHandler//

    void ReceivePacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceNetworkAddress);

    void SendPacket(
        unique_ptr<Packet>& packetPtr,
        const DhcpMessageType& messageType,
        const NetworkAddress& destinationAddress);

    void ProcessDhcpDiscoverMessage(
        DhcpMessageFormat& dhcpMessage);

    void ProcessDhcpRequestMessage(
        DhcpMessageFormat& dhcpMessage);

};//DhcpServerApplication//


inline
DhcpServerApplication::DhcpServerApplication(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const NodeIdType& initNodeId,
    const InterfaceIdType& initInterfaceId,
    const unsigned int initInterfaceIndex,
    const NetworkAddress& initServerNetworkAddress,
    const unsigned int initSubnetMaskLengthBits)
    :
    Application(initSimEngineInterfacePtr, "DHCP"/*ApplicationId*/),
    initializationIsComplete(false),
    nodeId(initNodeId),
    interfaceId(initInterfaceId),
    interfaceIndex(initInterfaceIndex),
    serverNetworkAddress(initServerNetworkAddress),
    subnetMaskLengthBits(initSubnetMaskLengthBits),

    packetPriority(0)
{

    serverSubnetAddress =
        serverNetworkAddress.MakeSubnetAddress(NetworkAddress::MakeSubnetMask(subnetMaskLengthBits));

    if (theParameterDatabaseReader.ParameterExists("dhcp-server-packet-priority", nodeId, interfaceId)) {
        packetPriority = static_cast<PacketPriorityType>(
            theParameterDatabaseReader.ReadNonNegativeInt(
                "dhcp-server-packet-priority", nodeId, interfaceId));
    }//if//

    //default
    defaultGatewayNetworkAddress = serverNetworkAddress;
    if (theParameterDatabaseReader.ParameterExists("dhcp-server-use-server-address-as-default-gateway", nodeId, interfaceId)) {
        if (!theParameterDatabaseReader.ReadBool("dhcp-server-use-server-address-as-default-gateway", nodeId, interfaceId)) {

            const string networkAddressString =
                theParameterDatabaseReader.ReadString("dhcp-server-default-gateway-network-address", nodeId, interfaceId);

            bool success = false;
            defaultGatewayNetworkAddress.SetAddressFromString(networkAddressString, success);

            if (!success) {
                cerr << "Error: dhcp-server-default-gateway-network-address value: " << networkAddressString;
                cerr << " for node: " << nodeId << endl;
                exit(1);
            }//if//

        }//if//
    }//if//

}//DhcpServerApplication//


inline
void DhcpServerApplication::CompleteInitialization()
{
    assert(transportLayerPtr != nullptr);
    assert(transportLayerPtr->udpPtr->PortIsAvailable(serverNetworkAddress, DHCP_SERVER_PORT));

    transportLayerPtr->udpPtr->OpenSpecificUdpPort(
        serverNetworkAddress,
        DHCP_SERVER_PORT,
        shared_ptr<UdpPacketHandler>(new UdpPacketHandler(shared_from_this())));

    initializationIsComplete = true;

}//CompleteInitialization//


inline
NetworkAddress DhcpServerApplication::AssignANetworkAddress(
    const GenericMacAddressType& targetMacAddress) const
{
    const map<GenericMacAddressType, NetworkAddressInfo>::const_iterator iter =
        assignedNetworkAddresses.find(targetMacAddress);

    if (iter != assignedNetworkAddresses.end()) {

        const NetworkAddressInfo& addressInfo = iter->second;

        return addressInfo.networkAddress;
    }
    else {
        //generate ip address (abstract model: use server subnet address and node id)
        const NetworkAddress newAddress(serverSubnetAddress, NetworkAddress(CalcNodeId(targetMacAddress)));

        return newAddress;
    }//if//

}//AssignANetworkAddress//


inline
void DhcpServerApplication::SendPacket(
    unique_ptr<Packet>& packetPtr,
    const DhcpMessageType& messageType,
    const NetworkAddress& destinationAddress)
{

    transportLayerPtr->udpPtr->SendPacket(
        packetPtr,(*this).serverNetworkAddress, DHCP_SERVER_PORT, destinationAddress, DHCP_CLIENT_PORT, packetPriority);

}//SendPacket//


inline
void DhcpServerApplication::ReceivePacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceNetworkAddress)
{

    assert(packetPtr->LengthBytes() == DHCP_MESSAGE_SIZE_BYTES);
    DhcpMessageFormat& dhcpMessage =
        packetPtr->GetAndReinterpretPayloadData<DhcpMessageFormat>();

    switch (dhcpMessage.messageType) {
    case DHCP_DISCOVER:
    {
        ProcessDhcpDiscoverMessage(dhcpMessage);
        break;
    }
    case DHCP_REQUEST:
    {
        ProcessDhcpRequestMessage(dhcpMessage);
        break;
    }
    case DHCP_OFFER:
    case DHCP_ACK:
    {
        assert(false&&"Should not receive this message type");
        exit(1);
        break;
    }
    default:
        assert(false&&"Unkonwn message type");
        exit(1);
        break;
    }//switch//

    packetPtr = nullptr;

}//ReceivePacket//


inline
void DhcpServerApplication::ProcessDhcpDiscoverMessage(
    DhcpMessageFormat& dhcpMessage)
{

    const GenericMacAddressType& targetMacAddress = dhcpMessage.clientHardwareAddress;
    const NetworkAddress offeredAddress((*this).AssignANetworkAddress(targetMacAddress));

    const map<GenericMacAddressType, NetworkAddressInfo>::iterator iter =
        assignedNetworkAddresses.find(targetMacAddress);

    assignedNetworkAddresses[targetMacAddress] = NetworkAddressInfo(offeredAddress, RESERVED);

    dhcpMessage.yourIpAddress = offeredAddress;
    dhcpMessage.serverIpAddress = (*this).serverNetworkAddress;
    dhcpMessage.defaultGatewayIpAddress = (*this).defaultGatewayNetworkAddress;
    dhcpMessage.subnetMaskLengthBits = (*this).subnetMaskLengthBits;
    dhcpMessage.messageType = DHCP_OFFER;

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, dhcpMessage, DHCP_MESSAGE_SIZE_BYTES);

    (*this).SendPacket(packetPtr, DHCP_OFFER, NetworkAddress::broadcastAddress);


}//ProcessDhcpDiscoverMessage//


inline
void DhcpServerApplication::ProcessDhcpRequestMessage(
    DhcpMessageFormat& dhcpMessage)
{

    //unexpected message
    if (dhcpMessage.serverIpAddress != (*this).serverNetworkAddress) return;

    const GenericMacAddressType& targetMacAddress = dhcpMessage.clientHardwareAddress;

    const map<GenericMacAddressType, NetworkAddressInfo>::iterator iter =
        assignedNetworkAddresses.find(targetMacAddress);

    assert(iter != assignedNetworkAddresses.end());

    NetworkAddressInfo& networkAddressInfo = (iter->second);

    //unexpected message (send nack)
    if ((dhcpMessage.yourIpAddress != networkAddressInfo.networkAddress) ||
        (dhcpMessage.defaultGatewayIpAddress != (*this).defaultGatewayNetworkAddress) ||
        (networkAddressInfo.status != RESERVED)) return;

    //update status
    networkAddressInfo.status = RESERVED;
    dhcpMessage.messageType = DHCP_ACK;

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, dhcpMessage, DHCP_MESSAGE_SIZE_BYTES);

    (*this).SendPacket(packetPtr, DHCP_ACK, NetworkAddress::broadcastAddress);

}//ProcessDhcpRequestMessage//


//--------------------------------------------------------------
//--------------------------------------------------------------
class DhcpClientApplication : public Application,
    public DhcpClientInterface ,
    public enable_shared_from_this<DhcpClientApplication> {
public:

    DhcpClientApplication(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const size_t numberOfInterfaces);

    ~DhcpClientApplication()
    { assert(initializationIsComplete); }

    void CompleteInitialization();

    //interfaces from network

    void EnableForThisInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex);

    void HandleLinkIsUpNotification(
        const unsigned int interfaceIndex,
        const GenericMacAddressType& genericMacAddress);

private:
    NodeIdType nodeId;
    bool initializationIsComplete;

    static const int SeedHash = 2678631;
    RandomNumberGenerator aRandomNumberGenerator;

    static const TimeType INITIAL_RETRY_INTERVAL = 4 * SECOND;
    static const TimeType RETRY_INTERVAL_JITTER_RANGE = 2 * SECOND;
    static const unsigned int MAX_RETRY_COUNT = 4;

    enum DhcpClientStatus {
        INACTIVE,
        WAITING_FOR_DHCP_OFFER,
        WAITING_FOR_DHCP_ACK,
        ASSIGNED
    };

    struct ClientInterfaceInfo {
        bool isEnabled;
        GenericMacAddressType genericMacAddress;

        //parameter
        PacketPriorityType packetPriority;
        TimeType currentRetryInterval;

        //variable
        DhcpClientStatus status;
        unsigned int retryCount;
        EventRescheduleTicket timeoutTimerEventTicket;
        unique_ptr<Packet> retryPacketPtr;

        NetworkAddress assignedNetworkAddress;

        ClientInterfaceInfo()
            :
            isEnabled(false),
            genericMacAddress(0),
            packetPriority(0),
            currentRetryInterval(INITIAL_RETRY_INTERVAL),
            status(INACTIVE),
            retryCount(0),
            retryPacketPtr(nullptr),
            assignedNetworkAddress(NetworkAddress::invalidAddress)
        {}

        void operator=(ClientInterfaceInfo&& right)
        {
            isEnabled = right.isEnabled;
            genericMacAddress = right.genericMacAddress;
            packetPriority = right.packetPriority;
            currentRetryInterval = right.currentRetryInterval;
            status = right.status;
            retryCount = right.retryCount;
            assignedNetworkAddress = right.assignedNetworkAddress;
            timeoutTimerEventTicket = right.timeoutTimerEventTicket;
            retryPacketPtr = move(right.retryPacketPtr);
        }

        ClientInterfaceInfo(ClientInterfaceInfo&& right) { (*this) = move(right); }

    private:
        //ClientInterfaceInfo(const ClientInterfaceInfo&);
        //void operator=(const ClientInterfaceInfo&);
    };//ClientInterfaceInfo//

    vector<ClientInterfaceInfo> interfaceInfo;

    void LookupInterfaceIndex(
        const GenericMacAddressType& targetMacAddress,
        unsigned int& interfaceIndex,
        bool& success) const;


    //interface from UDP
    class UdpPacketHandler: public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        UdpPacketHandler(
            const shared_ptr<DhcpClientApplication>& initDhcpClientAppPtr)
            :
            dhcpClientAppPtr(initDhcpClientAppPtr)
        {}

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePort,
            const NetworkAddress& destinationAddress,
            const PacketPriorityType& priority)
        {
            dhcpClientAppPtr->ReceivePacket(packetPtr, sourceAddress);
        }//ReceivePacket//

    private:
        shared_ptr<DhcpClientApplication> dhcpClientAppPtr;

    };//UdpPacketHandler//

    //------------------------------------------------------
    class TimeoutTimerEvent : public SimulationEvent {
    public:
        TimeoutTimerEvent(
            const shared_ptr<DhcpClientApplication>& initDhcpClientPtr,
            const unsigned int initInterfaceIndex,
            const DhcpClientStatus& initStatus)
            :
            dhcpClientPtr(initDhcpClientPtr),
            interfaceIndex(initInterfaceIndex),
            status(initStatus) { }

        void ExecuteEvent() { dhcpClientPtr->ProcessTimeout(interfaceIndex, status); }
    private:
        shared_ptr<DhcpClientApplication> dhcpClientPtr;
        unsigned int interfaceIndex;
        DhcpClientStatus status;
    };//TimeoutTimerEvent//

    void ProcessTimeout(
        const unsigned int interfaceIndex,
        const DhcpClientStatus& status);

    void ScheduleTimeoutEvent(
        const unsigned int interfaceIndex,
        const DhcpClientStatus& status);

    //------------------------------------------------------


    void ReceivePacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceNetworkAddress);

    void SendPacket(
        const unsigned int interfaceIndex,
        unique_ptr<Packet>& packetPtr,
        const DhcpMessageType& messageType,
        const NetworkAddress& destinationAddress);

    void SendInitialDhcpDiscoverMessage(
        const unsigned int interfaceIndex,
        const GenericMacAddressType& genericMacAddress);

    void ProcessDhcpOfferMessage(
        DhcpMessageFormat& dhcpMessage);

    void ProcessDhcpAckMessage(
        const DhcpMessageFormat& dhcpMessage);

};//DhcpServerApplication//


inline
DhcpClientApplication::DhcpClientApplication(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const NodeIdType& initNodeId,
    const size_t numberOfInterfaces)
    :
    Application(initSimEngineInterfacePtr, "DHCP"/*ApplicationId*/),
    initializationIsComplete(false),
    nodeId(initNodeId),
    aRandomNumberGenerator(HashInputsToMakeSeed(initNodeId, SeedHash))
{
    interfaceInfo.resize(numberOfInterfaces);

}//DhcpServerApplication//


inline
void DhcpClientApplication::CompleteInitialization()
{
    assert(transportLayerPtr != nullptr);

    //open udp port
    assert(transportLayerPtr->udpPtr->PortIsAvailable(DHCP_CLIENT_PORT));
    transportLayerPtr->udpPtr->OpenSpecificUdpPort(
        NetworkAddress::anyAddress,
        DHCP_CLIENT_PORT,
        shared_ptr<UdpPacketHandler>(new UdpPacketHandler(shared_from_this())));

    initializationIsComplete = true;

}//CompleteInitialization//



inline
void DhcpClientApplication::EnableForThisInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceIdType& interfaceId,
    const unsigned int interfaceIndex)
{

    assert(interfaceIndex < interfaceInfo.size());

    interfaceInfo[interfaceIndex].isEnabled = true;

    if (theParameterDatabaseReader.ParameterExists("dhcp-client-packet-priority", nodeId, interfaceId)) {
        interfaceInfo[interfaceIndex].packetPriority = static_cast<PacketPriorityType>(
            theParameterDatabaseReader.ReadNonNegativeInt(
                "dhcp-client-packet-priority", nodeId, interfaceId));
    }//if//

}//EnableForThisInterface//



inline
void DhcpClientApplication::HandleLinkIsUpNotification(
    const unsigned int interfaceIndex,
    const GenericMacAddressType& genericMacAddress)
{

    //ignore
    if (!interfaceInfo[interfaceIndex].isEnabled) return;

    //store mac adress
    interfaceInfo[interfaceIndex].genericMacAddress = genericMacAddress;

    (*this).SendInitialDhcpDiscoverMessage(interfaceIndex, genericMacAddress);


}//HandleLinkIsUpNotification//


inline
void DhcpClientApplication::LookupInterfaceIndex(
    const GenericMacAddressType& targetMacAddress,
    unsigned int& interfaceIndex,
    bool& success) const
{
    for(unsigned int i = 0; i < interfaceInfo.size(); i++) {
        if (interfaceInfo[i].genericMacAddress == targetMacAddress) {
            interfaceIndex = i;
            success = true;
            return;
        }//if//
    }//for//

    success = false;

}//LookupInterfaceIndex//

inline
void DhcpClientApplication::SendPacket(
    const unsigned int interfaceIndex,
    unique_ptr<Packet>& packetPtr,
    const DhcpMessageType& messageType,
    const NetworkAddress& destinationAddress)
{
    assert(interfaceInfo[interfaceIndex].isEnabled);

    transportLayerPtr->udpPtr->SendPacket(
        packetPtr,
        DHCP_CLIENT_PORT,
        destinationAddress,
        DHCP_SERVER_PORT,
        interfaceInfo[interfaceIndex].packetPriority);

}//SendPacket//


inline
void DhcpClientApplication::ReceivePacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceNetworkAddress)
{

    assert(packetPtr->LengthBytes() == DHCP_MESSAGE_SIZE_BYTES);
    DhcpMessageFormat& dhcpMessage =
        packetPtr->GetAndReinterpretPayloadData<DhcpMessageFormat>();

    switch (dhcpMessage.messageType) {
    case DHCP_OFFER:
    {
        ProcessDhcpOfferMessage(dhcpMessage);
        break;
    }
    case DHCP_ACK:
    {
        ProcessDhcpAckMessage(dhcpMessage);
        break;
    }
    case DHCP_DISCOVER:
    case DHCP_REQUEST:
    {
        assert(false&&"Should not receive this message type");
        exit(1);
        break;
    }
    default:
        assert(false&&"Unkonwn message type");
        exit(1);
        break;
    }//switch//

    packetPtr = nullptr;

}//ReceivePacket//


inline
void DhcpClientApplication::ScheduleTimeoutEvent(
    const unsigned int interfaceIndex,
    const DhcpClientStatus& status)
{

    const TimeType eventTime =
        simulationEngineInterfacePtr->CurrentTime() +
        interfaceInfo[interfaceIndex].currentRetryInterval +
        static_cast<TimeType>(aRandomNumberGeneratorPtr->GenerateRandomDouble() * RETRY_INTERVAL_JITTER_RANGE) -
        (RETRY_INTERVAL_JITTER_RANGE / 2);

    simulationEngineInterfacePtr->ScheduleEvent(
        shared_ptr<TimeoutTimerEvent>(new TimeoutTimerEvent(shared_from_this(), interfaceIndex, status)),
        eventTime,
        (interfaceInfo[interfaceIndex].timeoutTimerEventTicket));


}//ScheduleTimeoutEvent//


inline
void DhcpClientApplication::ProcessTimeout(
    const unsigned int interfaceIndex,
    const DhcpClientStatus& status)
{
    interfaceInfo[interfaceIndex].timeoutTimerEventTicket.Clear();

    if (interfaceInfo[interfaceIndex].retryCount < MAX_RETRY_COUNT) {

        interfaceInfo[interfaceIndex].retryCount++;
        interfaceInfo[interfaceIndex].currentRetryInterval =
            interfaceInfo[interfaceIndex].currentRetryInterval * 2;

        //set timeout timer
        (*this).ScheduleTimeoutEvent(interfaceIndex, status);

        DhcpMessageType messageType;
        if (status == WAITING_FOR_DHCP_OFFER) {
            messageType = DHCP_DISCOVER;
        }
        else if (status == WAITING_FOR_DHCP_ACK) {
            messageType = DHCP_REQUEST;
        }
        else {
            assert(false&&"Unexpected status."); exit(1);
        }//if//


        //retransmit
        unique_ptr<Packet> packetToSendPtr =
            unique_ptr<Packet>(new Packet(*interfaceInfo[interfaceIndex].retryPacketPtr));
        (*this).SendPacket(
            interfaceIndex, packetToSendPtr, messageType, NetworkAddress::broadcastAddress);

    }
    else {
        //exceed max retry count

        //delete stored packet
        assert(interfaceInfo[interfaceIndex].retryPacketPtr != nullptr);
        interfaceInfo[interfaceIndex].retryPacketPtr = nullptr;

        //restart initial process
        (*this).SendInitialDhcpDiscoverMessage(
            interfaceIndex,
            interfaceInfo[interfaceIndex].genericMacAddress);
    }//if//


}//ProcessTimeout//


inline
void DhcpClientApplication::SendInitialDhcpDiscoverMessage(
    const unsigned int interfaceIndex,
    const GenericMacAddressType& genericMacAddress)
{
    interfaceInfo[interfaceIndex].status = WAITING_FOR_DHCP_OFFER;
    interfaceInfo[interfaceIndex].retryCount = 0;
    interfaceInfo[interfaceIndex].currentRetryInterval = INITIAL_RETRY_INTERVAL;

    const DhcpMessageFormat dhcpDiscoverMessage(DHCP_DISCOVER, genericMacAddress);

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, dhcpDiscoverMessage, DHCP_MESSAGE_SIZE_BYTES);

    interfaceInfo[interfaceIndex].retryPacketPtr = unique_ptr<Packet>(new Packet(*packetPtr));

    //cancel timeout timer if necessary
    if (!(interfaceInfo[interfaceIndex].timeoutTimerEventTicket.IsNull())) {
        simulationEngineInterfacePtr->CancelEvent((interfaceInfo[interfaceIndex].timeoutTimerEventTicket));
    }//if//
    //set timeout timer
    (*this).ScheduleTimeoutEvent(interfaceIndex, WAITING_FOR_DHCP_OFFER);

    (*this).SendPacket(
        interfaceIndex, packetPtr, DHCP_DISCOVER, NetworkAddress::broadcastAddress);

}//SendDhcpDiscoverMessage//


inline
void DhcpClientApplication::ProcessDhcpOfferMessage(
    DhcpMessageFormat& dhcpMessage)
{

    unsigned int interfaceIndex;
    bool success;
    (*this).LookupInterfaceIndex(
        dhcpMessage.clientHardwareAddress,
        interfaceIndex,
        success);

    //unexpected message
    if (!success) return;

    //unexpected message
    if (interfaceInfo[interfaceIndex].status != WAITING_FOR_DHCP_OFFER) return;

    interfaceInfo[interfaceIndex].status = WAITING_FOR_DHCP_ACK;
    interfaceInfo[interfaceIndex].retryCount = 0;
    interfaceInfo[interfaceIndex].currentRetryInterval = INITIAL_RETRY_INTERVAL;

    interfaceInfo[interfaceIndex].assignedNetworkAddress = dhcpMessage.yourIpAddress;

    dhcpMessage.messageType = DHCP_REQUEST;

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, dhcpMessage, DHCP_MESSAGE_SIZE_BYTES);

    assert(interfaceInfo[interfaceIndex].retryPacketPtr != nullptr);
    interfaceInfo[interfaceIndex].retryPacketPtr = unique_ptr<Packet>(new Packet(*packetPtr));

    //cancel timeout timer
    simulationEngineInterfacePtr->CancelEvent(
        (interfaceInfo[interfaceIndex].timeoutTimerEventTicket));

    //set timeout timer
    (*this).ScheduleTimeoutEvent(interfaceIndex, WAITING_FOR_DHCP_ACK);

    (*this).SendPacket(
        interfaceIndex, packetPtr, DHCP_REQUEST, NetworkAddress::broadcastAddress);

}//ProcessDhcpOfferMessage//


inline
void DhcpClientApplication::ProcessDhcpAckMessage(
    const DhcpMessageFormat& dhcpMessage)
{

    unsigned int interfaceIndex;
    bool success;
    (*this).LookupInterfaceIndex(
        dhcpMessage.clientHardwareAddress,
        interfaceIndex,
        success);

    //unexpected message
    if (!success) return;

    //unexpected message
    if ((interfaceInfo[interfaceIndex].status != WAITING_FOR_DHCP_ACK) ||
        (interfaceInfo[interfaceIndex].assignedNetworkAddress != dhcpMessage.yourIpAddress)) return;

    //set network address and default gateway
    const shared_ptr<NetworkLayer>& networkLayerPtr =
        transportLayerPtr->GetNetworkLayerPtr();

    networkLayerPtr->SetInterfaceIpAddress(
        interfaceIndex,
        dhcpMessage.yourIpAddress,
        dhcpMessage.subnetMaskLengthBits);

    networkLayerPtr->SetInterfaceGatewayAddress(
        interfaceIndex,
        dhcpMessage.defaultGatewayIpAddress);

    //uddate status
    interfaceInfo[interfaceIndex].status = ASSIGNED;
    interfaceInfo[interfaceIndex].retryCount = 0;
    interfaceInfo[interfaceIndex].currentRetryInterval = INITIAL_RETRY_INTERVAL;
    assert(interfaceInfo[interfaceIndex].retryPacketPtr != nullptr);
    interfaceInfo[interfaceIndex].retryPacketPtr = nullptr;

    //cancel timeout timer
    simulationEngineInterfacePtr->CancelEvent(
        (interfaceInfo[interfaceIndex].timeoutTimerEventTicket));

}//ProcessDhcpAckMessage//


}//namespace//

#endif
