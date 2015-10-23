// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include <fstream>
#include <iterator>
#include "iscdhcp_porting.h"
#include "iscdhcpglue.h"

namespace ScenSim {

using std::ifstream;
using std::ofstream;
using std::istreambuf_iterator;
using std::ostreambuf_iterator;
using std::ios_base;

using IscDhcpPort::client_location_changed;
using IscDhcpPort::context;
using IscDhcpPort::curctx;
using IscDhcpPort::free_everything;
using IscDhcpPort::iaddr;
using IscDhcpPort::receive_packet;
using IscDhcpPort::start_client;
using IscDhcpPort::start_server;

class IscDhcpImplementation
    : public enable_shared_from_this<IscDhcpImplementation> {
public:
    IscDhcpImplementation(
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<TransportLayer>& initTransportLayerPtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const shared_ptr<RandomNumberGenerator>& initRandomNumberGeneratorPtr,
        const NodeIdType& initNodeId,
        const bool initIsServer);

    virtual ~IscDhcpImplementation();

    void CompleteInitialization(
        const ParameterDatabaseReader& theParameterDatabaseReader);

    void HandleLinkIsUpNotification(
        const unsigned int interfaceIndex,
        const GenericMacAddressType& genericMacAddress);

    void EnableForThisInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex);

    void Reboot(const TimeType& waitTime);

    int32_t GenerateRandomInt();

    unsigned int NumberOfInterfaces() const;
    InterfaceIdType GetInterfaceId(unsigned int interfaceIndex) const;

    void GetIpAddress(
        unsigned int interfaceIndex,
        uint64_t *ipAddressHighBits,
        uint64_t *ipAddressLowBits) const;

    void SetIpAddress(
        unsigned int interfaceIndex,
        uint64_t ipAddressHighBits,
        uint64_t ipAddressLowBits,
        unsigned int subnetMaskLengthBits);

    void SetGatewayAddress(
        unsigned int interfaceIndex,
        uint64_t ipAddressHighBits,
        uint64_t ipAddressLowBits) const;

    void GetHardwareAddress(
        unsigned int interfaceIndex,
        uint8_t *hardwareAddress,
        uint8_t *hardwareAddressLength) const;

    void AddTimer(const TimeType& eventTime, void (*func)(void *), void *arg);
    void CancelTimer(void (*func)(void *), void *arg);
    void CancelAllTimers();

    void SendPacket(
        const unsigned char *dataPtr,
        const size_t dataLength,
        const NetworkAddress& sourceAddress,
        const unsigned short int sourcePortId,
        const NetworkAddress& destinationAddress,
        const unsigned short int destinationPortId);

private:
    enum DhcpEventKind {
        START
    };//DhcpEventKind//

    class PacketHandler
        : public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        PacketHandler(
            const shared_ptr<IscDhcpImplementation>& initIscDhcpImplPtr)
            : iscDhcpImplPtr(initIscDhcpImplPtr) {}

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePortId,
            const NetworkAddress& destinationAddress,
            const PacketPriorityType& priority)
        {
            iscDhcpImplPtr->ReceivePacket(
                packetPtr, sourceAddress, destinationAddress, sourcePortId);

        }//ReceivePacket//

    private:
        shared_ptr<IscDhcpImplementation> iscDhcpImplPtr;

    };//PacketHandler//

    class DhcpEvent: public SimulationEvent {
    public:
        DhcpEvent(
            const shared_ptr<IscDhcpImplementation>& initIscDhcpImplPtr,
            const DhcpEventKind& initDhcpEventKind)
            :
            iscDhcpImplPtr(initIscDhcpImplPtr),
            dhcpEventKind(initDhcpEventKind)
        {}

        void ExecuteEvent()
        {
            switch (dhcpEventKind) {
            case START:
                if (iscDhcpImplPtr->isServer) {
                    iscDhcpImplPtr->StartServer();
                }
                else {
                    iscDhcpImplPtr->StartClient();
                }//if//
                break;
            default:
                assert(false);
            }//switch//

        }//ExecuteEvent//

    private:
        shared_ptr<IscDhcpImplementation> iscDhcpImplPtr;
        DhcpEventKind dhcpEventKind;

    };//DhcpEvent//

    class TimerEvent: public SimulationEvent {
    public:
        TimerEvent(
            const shared_ptr<IscDhcpImplementation>& initIscDhcpImplPtr,
            size_t initIndex)
            :
            iscDhcpImplPtr(initIscDhcpImplPtr),
            index(initIndex)
        {}

        void ExecuteEvent()
        {
            iscDhcpImplPtr->ProcessTimerEvent(index);

        }//ExecuteEvent//

    private:
        shared_ptr<IscDhcpImplementation> iscDhcpImplPtr;
        size_t index;

    };//TimerEvent//

    struct TimerEntry {
        shared_ptr<TimerEvent> timerEventPtr;
        shared_ptr<EventRescheduleTicket> ticketPtr;
        void (*func)(void *);
        void *arg;

        TimerEntry(
            const shared_ptr<TimerEvent>& initTimerEventPtr,
            const shared_ptr<EventRescheduleTicket>& initTicketPtr,
            void (*initFunc)(void *),
            void *initArg)
            :
            timerEventPtr(initTimerEventPtr),
            ticketPtr(initTicketPtr),
            func(initFunc),
            arg(initArg)
        {
        }//TimerEntry//

    };//TimerEntry//

    void BeginDhcpProcess();
    void EndDhcpProcess();

    void ProcessTimerEvent(size_t index);

    void StartClient();
    void StartServer();
    void ReceivePacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const unsigned short int sourcePortId);

    static void ReplaceWithNodeId(
        const NodeIdType& nodeId,
        string& targetString);

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    shared_ptr<TransportLayer> transportLayerPtr;
    shared_ptr<NetworkLayer> networkLayerPtr;
    shared_ptr<RandomNumberGenerator> randomNumberGeneratorPtr;
    const NodeIdType nodeId;
    const bool isServer;

    string configFileName;
    string leaseFileName;
    PacketPriorityType priority;
    shared_ptr<PacketHandler> udpPacketHandlerPtr;
    vector<TimerEntry> vectorOfTimer;
    vector<unsigned int> vectorOfInterfaceIndex;
    context *ctxPtr;

};//IscDhcpImplementation//

IscDhcpImplementation::IscDhcpImplementation(
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<TransportLayer>& initTransportLayerPtr,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const shared_ptr<RandomNumberGenerator>& initRandomNumberGeneratorPtr,
    const NodeIdType& initNodeId,
    const bool initIsServer)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    transportLayerPtr(initTransportLayerPtr),
    networkLayerPtr(initNetworkLayerPtr),
    randomNumberGeneratorPtr(initRandomNumberGeneratorPtr),
    nodeId(initNodeId),
    isServer(initIsServer),
    configFileName(),
    leaseFileName(),
    priority(0),
    udpPacketHandlerPtr(),
    vectorOfTimer(),
    vectorOfInterfaceIndex(),
    ctxPtr(nullptr)
{
    ctxPtr = new context(IpHeaderModel::usingVersion6);
    ctxPtr->glue = this;

}//IscDhcpImplementation//

IscDhcpImplementation::~IscDhcpImplementation()
{
    delete ctxPtr;
    ctxPtr = nullptr;

}//~IscDhcpImplementation//

void IscDhcpImplementation::CompleteInitialization(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    udpPacketHandlerPtr = shared_ptr<PacketHandler>(new PacketHandler(shared_from_this()));

    if (isServer) {
        const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
        simEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new DhcpEvent(shared_from_this(), START)),
            currentTime);
    }//if//

}//CompleteInitialization//

void IscDhcpImplementation::HandleLinkIsUpNotification(
    const unsigned int interfaceIndex,
    const GenericMacAddressType& genericMacAddress)
{
    assert(!isServer);

    if (!isServer) {
        const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
        simEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new DhcpEvent(shared_from_this(), START)), currentTime);
    }//if//

}//HandleLinkIsUpNotification//

void IscDhcpImplementation::EnableForThisInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceIdType& interfaceId,
    const unsigned int interfaceIndex)
{
    if (vectorOfInterfaceIndex.size() != 0) {
        cerr << "DHCP cannot work on multiple interfaces" << endl;
        exit(1);
    }//if//

    vectorOfInterfaceIndex.push_back(interfaceIndex);

    if (isServer) {
        if (theParameterDatabaseReader.ParameterExists("iscdhcp-server-config-file", nodeId, interfaceId)) {
            configFileName = theParameterDatabaseReader.ReadString("iscdhcp-server-config-file", nodeId, interfaceId);
        }
        else {
            cerr << "Error: iscdhcp-server-config-file is not defined." << endl;
            exit(1);
        }//if//

        if (theParameterDatabaseReader.ParameterExists("iscdhcp-server-output-lease-file", nodeId, interfaceId)) {
            leaseFileName = theParameterDatabaseReader.ReadString("iscdhcp-server-output-lease-file", nodeId, interfaceId);
        }
        else {
            cerr << "Error: iscdhcp-server-output-lease-file is not defined." << endl;
            exit(1);
        }//if//

        ReplaceWithNodeId(nodeId, configFileName);
        ReplaceWithNodeId(nodeId, leaseFileName);

        if (theParameterDatabaseReader.ParameterExists("iscdhcp-server-input-lease-file", nodeId, interfaceId)) {
            string inputLeaseFileName = theParameterDatabaseReader.ReadString("iscdhcp-server-input-lease-file", nodeId, interfaceId);
            ReplaceWithNodeId(nodeId, inputLeaseFileName);
            ifstream inputLeaseFile(inputLeaseFileName.c_str(), ios_base::in | ios_base::binary);
            if (!inputLeaseFile.good()) {
                cerr << "Error: iscdhcp-server-input-lease-file(" << inputLeaseFileName << ") doesn't exist." << endl;
                exit(1);
            }//if//
            if (inputLeaseFileName != leaseFileName) {
                ofstream outputLeaseFile(leaseFileName.c_str(), ios_base::out | ios_base::binary | ios_base::trunc);
                istreambuf_iterator<char> beginInputLeaseFileIter(inputLeaseFile);
                istreambuf_iterator<char> endInputLeaseFileIter;
                ostreambuf_iterator<char> beginOutputLeaseFileIter(outputLeaseFile);
                copy(beginInputLeaseFileIter, endInputLeaseFileIter, beginOutputLeaseFileIter);
            }//if//
        }
        else {
            ofstream outputLeaseFile(leaseFileName.c_str(), ios_base::out | ios_base::binary | ios_base::trunc);
        }//if//

        CURCTX_SET(ctxPtr);
        path_dhcpd_conf = configFileName.c_str();
        path_dhcpd_db = leaseFileName.c_str();
        CURCTX_CLEAR();

        if (theParameterDatabaseReader.ParameterExists("iscdhcp-server-packet-priority", nodeId, interfaceId)) {
            priority = theParameterDatabaseReader.ReadInt("iscdhcp-server-packet-priority", nodeId, interfaceId);
        }//if//
    }
    else {
        if (theParameterDatabaseReader.ParameterExists("iscdhcp-client-config-file", nodeId, interfaceId)) {
            configFileName = theParameterDatabaseReader.ReadString("iscdhcp-client-config-file", nodeId, interfaceId);
        }
        else {
            cerr << "Error: iscdhcp-client-config-file is not defined." << endl;
            exit(1);
        }//if//

        if (theParameterDatabaseReader.ParameterExists("iscdhcp-client-output-lease-file", nodeId, interfaceId)) {
            leaseFileName = theParameterDatabaseReader.ReadString("iscdhcp-client-output-lease-file", nodeId, interfaceId);
        }
        else {
            cerr << "Error: iscdhcp-client-output-lease-file is not defined." << endl;
            exit(1);
        }//if//

        ReplaceWithNodeId(nodeId, configFileName);
        ReplaceWithNodeId(nodeId, leaseFileName);

        if (theParameterDatabaseReader.ParameterExists("iscdhcp-client-input-lease-file", nodeId, interfaceId)) {
            string inputLeaseFileName = theParameterDatabaseReader.ReadString("iscdhcp-client-input-lease-file", nodeId, interfaceId);
            ReplaceWithNodeId(nodeId, inputLeaseFileName);
            ifstream inputLeaseFile(inputLeaseFileName.c_str(), ios_base::in | ios_base::binary);
            if (!inputLeaseFile.good()) {
                cerr << "Error: iscdhcp-client-input-lease-file(" << inputLeaseFileName << ") doesn't exist." << endl;
                exit(1);
            }//if//
            if (inputLeaseFileName != leaseFileName) {
                ofstream outputLeaseFile(leaseFileName.c_str(), ios_base::out | ios_base::binary | ios_base::trunc);
                istreambuf_iterator<char> beginInputLeaseFileIter(inputLeaseFile);
                istreambuf_iterator<char> endInputLeaseFileIter;
                ostreambuf_iterator<char> beginOutputLeaseFileIter(outputLeaseFile);
                copy(beginInputLeaseFileIter, endInputLeaseFileIter, beginOutputLeaseFileIter);
            }//if//
        }
        else {
            ofstream outputLeaseFile(leaseFileName.c_str(), ios_base::out | ios_base::binary | ios_base::trunc);
        }//if//

        CURCTX_SET(ctxPtr);
        path_dhclient_conf = configFileName.c_str();
        path_dhclient_db = leaseFileName.c_str();
        CURCTX_CLEAR();

        if (theParameterDatabaseReader.ParameterExists("iscdhcp-client-packet-priority", nodeId, interfaceId)) {
            priority = theParameterDatabaseReader.ReadInt("iscdhcp-client-packet-priority", nodeId, interfaceId);
        }//if//
    }//if//

}//EnableForThisInterface//

void IscDhcpImplementation::Reboot(const TimeType& waitTime)
{
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

    simEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new DhcpEvent(shared_from_this(), START)),
        (currentTime + waitTime));

}//Reboot//

void IscDhcpImplementation::BeginDhcpProcess()
{
    CURCTX_SET(ctxPtr);
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    cur_tv.tv_sec = currentTime / SECOND;
    cur_tv.tv_usec = (currentTime % SECOND) / MICRO_SECOND;

}//BeginDhcpProcess//

void IscDhcpImplementation::EndDhcpProcess()
{
    CURCTX_CLEAR();

}//EndDhcpProcess//

void IscDhcpImplementation::ProcessTimerEvent(size_t index)
{
    vectorOfTimer[index].ticketPtr->Clear();

    BeginDhcpProcess();
    vectorOfTimer[index].func(vectorOfTimer[index].arg);
    EndDhcpProcess();

}//ProcessTimerEvent//

void IscDhcpImplementation::StartClient()
{
    BeginDhcpProcess();
    if (interfaces) {
        client_location_changed();
    }
    else {
        start_client();
        transportLayerPtr->udpPtr->OpenSpecificUdpPort(
            NetworkAddress::anyAddress, NetToHost16(local_port), udpPacketHandlerPtr);
    }//if//
    EndDhcpProcess();

}//StartClient//

void IscDhcpImplementation::StartServer()
{
    BeginDhcpProcess();
    if (interfaces) {
        free_everything();
        EndDhcpProcess();
        CancelAllTimers();
        vectorOfTimer.clear();
        delete ctxPtr;
        ctxPtr = new context(IpHeaderModel::usingVersion6);
        ctxPtr->glue = this;
        BeginDhcpProcess();
        path_dhcpd_conf = configFileName.c_str();
        path_dhcpd_db = leaseFileName.c_str();
        start_server();
    }
    else {
        start_server();
        transportLayerPtr->udpPtr->OpenSpecificUdpPort(
            NetworkAddress::anyAddress, NetToHost16(local_port), udpPacketHandlerPtr);
    }
    EndDhcpProcess();

}//StartServer//

void IscDhcpImplementation::ReceivePacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const unsigned short int sourcePortId)
{
    const uint64_t sourceAddressHighBits = sourceAddress.GetRawAddressHighBits();
    const uint64_t sourceAddressLowBits = sourceAddress.GetRawAddressLowBits();

    struct iaddr ia;

    if (ScenSim::IpHeaderModel::usingVersion6) {
        ia.len = 16;
        ConvertTwoHost64ToNet128(sourceAddressHighBits, sourceAddressLowBits, ia.iabuf);
    }
    else {
        ia.len = 4;
        *reinterpret_cast<uint32_t*>(ia.iabuf) =
            HostToNet32(static_cast<const uint32_t>(sourceAddressLowBits));
    }//if//

    BeginDhcpProcess();
    receive_packet(
        0,//TBD: multiple interfaces are not supported
        packetPtr->GetRawPayloadData(),
        packetPtr->LengthBytes(),
        sourcePortId,
        &ia,
        !destinationAddress.IsAMulticastAddress());
    EndDhcpProcess();

    packetPtr = nullptr;

}//ReceivePacket//

void IscDhcpImplementation::ReplaceWithNodeId(
    const NodeIdType& nodeId,
    string& targetString)
{
    const string replacedString("$n");
    string::size_type pos = targetString.find(replacedString);

    if (pos != string::npos) {
        targetString.replace(pos, replacedString.length(), ConvertToString(nodeId));
    }//if//

}//ReplaceWithNodeId//

IscDhcp::IscDhcp(
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<TransportLayer>& initTransportLayerPtr,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const shared_ptr<RandomNumberGenerator>& initRandomNumberGeneratorPtr,
    const NodeIdType& initNodeId,
    const bool initIsServer)
    :
    implPtr(
        new IscDhcpImplementation(
            initSimEngineInterfacePtr,
            initTransportLayerPtr,
            initNetworkLayerPtr,
            initRandomNumberGeneratorPtr,
            initNodeId,
            initIsServer))
{
}//IscDhcp//

IscDhcp::~IscDhcp()
{
}//~IscDhcp//

void IscDhcp::CompleteInitialization(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    implPtr->CompleteInitialization(
        theParameterDatabaseReader);

}//CompleteInitialization//

void IscDhcp::HandleLinkIsUpNotification(
    const unsigned int interfaceIndex,
    const GenericMacAddressType& genericMacAddress)
{
    implPtr->HandleLinkIsUpNotification(
        interfaceIndex,
        genericMacAddress);

}//HandleLinkIsUpNotification//

void IscDhcp::EnableForThisInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceIdType& interfaceId,
    const unsigned int interfaceIndex)
{
    implPtr->EnableForThisInterface(
        theParameterDatabaseReader,
        interfaceId,
        interfaceIndex);

}//EnableForThisInterface//

void IscDhcp::Reboot(const TimeType& waitTime)
{
    implPtr->Reboot(waitTime);

}//Reboot//

//------------------------------------------------------------------------------
// Functions called by DHCP
//------------------------------------------------------------------------------

int32_t IscDhcpImplementation::GenerateRandomInt()
{
    return randomNumberGeneratorPtr->GenerateRandomInt(0, INT_MAX - 1);

}//GenerateRandomInt//

unsigned int IscDhcpImplementation::NumberOfInterfaces() const
{
    return vectorOfInterfaceIndex.size();

}//NumberOfInterfaces//

InterfaceIdType IscDhcpImplementation::GetInterfaceId(
    unsigned int interfaceIndex) const
{
    return networkLayerPtr->GetInterfaceId(
        vectorOfInterfaceIndex.at(interfaceIndex));

}//GetInterfaceId//

void IscDhcpImplementation::GetIpAddress(
    unsigned int interfaceIndex,
    uint64_t *ipAddressHighBits,
    uint64_t *ipAddressLowBits) const
{
    assert(ipAddressHighBits);
    assert(ipAddressLowBits);

    const NetworkAddress networkAddress = networkLayerPtr->
        GetNetworkAddress(vectorOfInterfaceIndex.at(interfaceIndex));

    *ipAddressHighBits = networkAddress.GetRawAddressHighBits();
    *ipAddressLowBits = networkAddress.GetRawAddressLowBits();

}//GetIpAddress//

void IscDhcpImplementation::SetIpAddress(
    unsigned int interfaceIndex,
    uint64_t ipAddressHighBits,
    uint64_t ipAddressLowBits,
    unsigned int subnetMaskLengthBits)
{
    const NetworkAddress networkAddress(ipAddressHighBits, ipAddressLowBits);

    networkLayerPtr->SetInterfaceIpAddress(
        vectorOfInterfaceIndex.at(interfaceIndex),
        networkAddress,
        subnetMaskLengthBits);

}//SetIpAddress//

void IscDhcpImplementation::SetGatewayAddress(
    unsigned int interfaceIndex,
    uint64_t ipAddressHighBits,
    uint64_t ipAddressLowBits) const
{
    const NetworkAddress networkAddress(ipAddressHighBits, ipAddressLowBits);

    networkLayerPtr->SetInterfaceGatewayAddress(
        vectorOfInterfaceIndex.at(interfaceIndex),
        networkAddress);

}//SetGatewayAddress//

void IscDhcpImplementation::GetHardwareAddress(
    unsigned int interfaceIndex,
    uint8_t *hardwareAddress,
    uint8_t *hardwareAddressLength) const
{
    assert(hardwareAddress);
    assert(hardwareAddressLength);

    const GenericMacAddressType genericMacAddress = networkLayerPtr->
        GetMacLayerPtr(vectorOfInterfaceIndex.at(interfaceIndex))->GetGenericMacAddress();
    const SixByteMacAddressType macAddress(genericMacAddress);

    *hardwareAddressLength = static_cast<uint8_t>(
        SixByteMacAddressType::numberMacAddressBytes + 1);

    hardwareAddress[0] = HTYPE_ETHER;
    for (uint8_t i = 1; i < *hardwareAddressLength; ++i) {
        hardwareAddress[i] = macAddress.addressBytes[i-1];
    }//for//

}//GetHardwareAddress//

void IscDhcpImplementation::AddTimer(
    const TimeType& eventTime,
    void (*func)(void *),
    void *arg)
{
    assert(func);
    assert(arg);

    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    TimeType nextTime;

    if (currentTime <= eventTime) {
        nextTime = eventTime;
    }
    else {
        nextTime = currentTime;
    }//if//

    size_t i;

    for (i = 0; i < vectorOfTimer.size(); ++i) {
        if ((vectorOfTimer[i].func == func) &&
            (vectorOfTimer[i].arg == arg)) {
            break;
        }//if//
    }//for//

    if (i == vectorOfTimer.size()) {
        shared_ptr<TimerEvent> timerEventPtr(new TimerEvent(shared_from_this(), i));
        shared_ptr<EventRescheduleTicket> ticketPtr(new EventRescheduleTicket());
        vectorOfTimer.push_back(TimerEntry(timerEventPtr, ticketPtr, func, arg));
        simEngineInterfacePtr->ScheduleEvent(timerEventPtr, nextTime, *ticketPtr);
    }
    else {
        shared_ptr<TimerEvent>& timerEventPtr = vectorOfTimer[i].timerEventPtr;
        shared_ptr<EventRescheduleTicket>& ticketPtr = vectorOfTimer[i].ticketPtr;
        if (ticketPtr->IsNull()) {
            simEngineInterfacePtr->ScheduleEvent(timerEventPtr, nextTime, *ticketPtr);
        }
        else {
            simEngineInterfacePtr->RescheduleEvent(*ticketPtr, nextTime);
        }//if//
    }//if//

}//AddTimer//

void IscDhcpImplementation::CancelTimer(
    void (*func)(void *),
    void *arg)
{
    assert(func);
    assert(arg);

    for (size_t i = 0; i < vectorOfTimer.size(); ++i) {
        if ((vectorOfTimer[i].func == func) &&
            (vectorOfTimer[i].arg == arg) &&
            !vectorOfTimer[i].ticketPtr->IsNull()) {
            simEngineInterfacePtr->CancelEvent(*vectorOfTimer[i].ticketPtr);
            break;
        }//if//
    }//for//

}//CancelTimer//

void IscDhcpImplementation::CancelAllTimers()
{
    for (size_t i = 0; i < vectorOfTimer.size(); ++i) {
        if (!vectorOfTimer[i].ticketPtr->IsNull()) {
            simEngineInterfacePtr->CancelEvent(*vectorOfTimer[i].ticketPtr);
        }//if//
    }//for//

}//CancelAllTimers//

void IscDhcpImplementation::SendPacket(
    const unsigned char *dataPtr,
    const size_t dataLength,
    const NetworkAddress& sourceAddress,
    const unsigned short int sourcePortId,
    const NetworkAddress& destinationAddress,
    const unsigned short int destinationPortId)
{
    unique_ptr<Packet> packetPtr = Packet::CreatePacket(
        *simEngineInterfacePtr, dataPtr, static_cast<unsigned int>(dataLength));

    transportLayerPtr->udpPtr->SendPacket(
        packetPtr, sourceAddress, sourcePortId,
        destinationAddress, destinationPortId, priority);

}//SendPacket//

}//namespace ScenSim//

int32_t IscDhcpPort::GenerateRandomInt(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr)
{
    return iscDhcpImplPtr->GenerateRandomInt();

}//GenerateRandomInt//

unsigned int IscDhcpPort::NumberOfInterfaces(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr)
{
    return iscDhcpImplPtr->NumberOfInterfaces();

}//NumberOfInterfaces//

const char *IscDhcpPort::GetInterfaceId(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    unsigned int interfaceIndex)
{
    return iscDhcpImplPtr->GetInterfaceId(interfaceIndex).c_str();

}//GetInterfaceId//

void IscDhcpPort::GetIpAddress(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    unsigned int interfaceIndex,
    struct iaddr *ia)
{
    assert(ia);

    uint64_t ipAddressHighBits;
    uint64_t ipAddressLowBits;

    iscDhcpImplPtr->GetIpAddress(interfaceIndex, &ipAddressHighBits, &ipAddressLowBits);

    if (ScenSim::IpHeaderModel::usingVersion6) {
        ia->len = 16;
        ConvertTwoHost64ToNet128(ipAddressHighBits, ipAddressLowBits, ia->iabuf);
    }
    else {
        ia->len = 4;
        *reinterpret_cast<uint32_t*>(ia->iabuf) =
            HostToNet32(static_cast<uint32_t>(ipAddressLowBits));
    }//if//

}//GetIpAddress//

void IscDhcpPort::SetIpAddress(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    unsigned int interfaceIndex,
    struct iaddr *ia,
    struct iaddr *mask)
{
    assert(ia);
    assert(ia->len == 4 || ia->len == 16);

    uint64_t ipAddressHighBits = 0;
    uint64_t ipAddressLowBits = 0;

    if (ia->len == 16) {
        ConvertNet128ToTwoHost64(ia->iabuf, ipAddressHighBits, ipAddressLowBits);
    }
    else if (ia->len == 4) {
        ipAddressLowBits =
            NetToHost32(*reinterpret_cast<uint32_t*>(ia->iabuf));
    }//if//

    uint64_t subnetMaskHighBits = 0;
    uint64_t subnetMaskLowBits = 0;
    unsigned int subnetMaskLengthBits = 0;

    if (mask != nullptr) {
        assert(mask->len == 4 || mask->len == 16);
        if (mask->len == 16) {
            ConvertNet128ToTwoHost64(mask->iabuf, subnetMaskHighBits, subnetMaskLowBits);
            while ((subnetMaskHighBits >> 63) != 0) {
                subnetMaskHighBits = subnetMaskHighBits << 1;
                subnetMaskLengthBits += 1;
            }//while//
            while ((subnetMaskLowBits >> 63) != 0) {
                subnetMaskLowBits = subnetMaskLowBits << 1;
                subnetMaskLengthBits += 1;
            }//while//
        }
        else if (mask->len == 4) {
            subnetMaskLowBits =
                NetToHost32(*reinterpret_cast<uint32_t*>(mask->iabuf));
            subnetMaskLowBits = subnetMaskLowBits << 32;
            while ((subnetMaskLowBits >> 63) != 0) {
                subnetMaskLowBits = subnetMaskLowBits << 1;
                subnetMaskLengthBits += 1;
            }//while//
        }//if//
    }
    else {
        assert(ia->len == 16);
        subnetMaskLengthBits = 64;
    }//if//

    iscDhcpImplPtr->SetIpAddress(
        interfaceIndex,
        ipAddressHighBits,
        ipAddressLowBits,
        subnetMaskLengthBits);

}//SetIpAddress//

void IscDhcpPort::SetGatewayAddress(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    unsigned int interfaceIndex,
    struct iaddr *ia)
{
    assert(ia);
    assert(ia->len == 4 || ia->len == 4);

    uint64_t ipAddressHighBits = 0;
    uint64_t ipAddressLowBits = 0;

    if (ia->len == 16) {
        ConvertNet128ToTwoHost64(ia->iabuf, ipAddressHighBits, ipAddressLowBits);
    }
    else if (ia->len == 4) {
        ipAddressLowBits =
            NetToHost32(*reinterpret_cast<uint32_t*>(ia->iabuf));
    }//if//

    iscDhcpImplPtr->SetGatewayAddress(
        interfaceIndex,
        ipAddressHighBits,
        ipAddressLowBits);

}//SetGatewayAddress//

void IscDhcpPort::GetHardwareAddress(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    unsigned int interfaceIndex,
    uint8_t *hardwareAddress,
    uint8_t *hardwareAddressLength)
{
    return iscDhcpImplPtr->GetHardwareAddress(
        interfaceIndex,
        hardwareAddress,
        hardwareAddressLength);

}//GetHardwareAddress//

void IscDhcpPort::AddTimer(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    const timeval *event_time,
    void (*func)(void *),
    void *arg)
{
    assert(event_time);

    const ScenSim::TimeType eventTime =
        event_time->tv_sec * ScenSim::SECOND +
            event_time->tv_usec * ScenSim::MICRO_SECOND;

    return iscDhcpImplPtr->AddTimer(eventTime, func, arg);

}//AddTimer//

void IscDhcpPort::CancelTimer(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    void (*func)(void *),
    void *arg)
{
    return iscDhcpImplPtr->CancelTimer(func, arg);

}//CancelTimer//

void IscDhcpPort::SendPacket(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    const unsigned char *dataPtr,
    const size_t dataLength,
    const uint32_t source_address,
    const uint16_t source_port,
    const uint32_t destination_address,
    const uint16_t destination_port)
{
    iscDhcpImplPtr->SendPacket(
        dataPtr, dataLength,
        ScenSim::NetworkAddress(NetToHost32(source_address)),
        NetToHost16(source_port),
        ScenSim::NetworkAddress(NetToHost32(destination_address)),
        NetToHost16(destination_port));

}//SendPacket//

void IscDhcpPort::SendPacket6(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    const unsigned char *dataPtr,
    const size_t dataLength,
    const uint8_t *source_address,
    const uint16_t source_port,
    const uint8_t *destination_address,
    const uint16_t destination_port)
{
    uint64_t sourceAddressHighBits;
    uint64_t sourceAddressLowBits;
    ConvertNet128ToTwoHost64(source_address, sourceAddressHighBits, sourceAddressLowBits);

    uint64_t destinationAddressHighBits;
    uint64_t destinationAddressLowBits;
    ConvertNet128ToTwoHost64(destination_address, destinationAddressHighBits, destinationAddressLowBits);

    iscDhcpImplPtr->SendPacket(
        dataPtr, dataLength,
        ScenSim::NetworkAddress(sourceAddressHighBits, sourceAddressLowBits),
        NetToHost16(source_port),
        ScenSim::NetworkAddress(destinationAddressHighBits, destinationAddressLowBits),
        NetToHost16(destination_port));

}//SendPacket//

uint32_t IscDhcpPort::htonl(uint32_t hostlong)
{
    return HostToNet32(hostlong);

}//htonl//

uint16_t IscDhcpPort::htons(uint16_t hostshort)
{
    return HostToNet16(hostshort);

}//htons//

uint32_t IscDhcpPort::ntohl(uint32_t netlong)
{
    return NetToHost32(netlong);

}//ntohl//

uint16_t IscDhcpPort::ntohs(uint16_t netshort)
{
    return NetToHost16(netshort);

}//ntohs//
