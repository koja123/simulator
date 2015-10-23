// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef NRLOLSRGLUE_H
#define NRLOLSRGLUE_H

#include "scensim_netsim.h"
#include "nrlprotolibglue.h"

#include "nrlolsr.h"

#include "protokit.h"
#include "protoTimer.h"
#include "protoRouteMgr.h"
#include "protoAddress.h"


namespace ScenSim {

using NrlOlsrPort::Nrlolsr;
using NrlOlsrPort::OlsrMessage;
using NrlOlsrPort::OlsrPacket;
using NrlProtolibPort::ProtoAddressList;
using NrlProtolibPort::ProtoRouteMgr;
using NrlProtolibPort::ProtoRouteTable;
using NrlProtolibPort::SIMADDR;



//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
class ScenargieRouteMgr : public ProtoRouteMgr {
public:
    ScenargieRouteMgr()
        : interfaceAddress(NetworkAddress::invalidAddress) { }

    ScenargieRouteMgr(
        const shared_ptr<NrlolsrProtocol>& initNrlolsrProtocolPtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const unsigned int initInterfaceIndex)
        :
        nrlolsrProtocolPtr(initNrlolsrProtocolPtr),
        networkLayerPtr(initNetworkLayerPtr),
        interfaceIndex(initInterfaceIndex),
        interfaceAddress(NetworkAddress::invalidAddress)
    { }

    ~ScenargieRouteMgr() { Close(); }


    static ScenargieRouteMgr* Create(
        const shared_ptr<NrlolsrProtocol>& initNrlolsrProtocolPtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const unsigned int initInterfaceIndex)
    {
        return (
            new ScenargieRouteMgr(
                initNrlolsrProtocolPtr,
                initNetworkLayerPtr,
                initInterfaceIndex));
    }



    virtual bool Open(const void* interfaceDataPtr = nullptr);
    virtual bool IsOpen() const
        { return (interfaceAddress != NetworkAddress::invalidAddress); }

    virtual void Close()
    {
        protoRouteTable.Destroy();
        interfaceAddress = NetworkAddress::invalidAddress;
    }

    virtual bool GetAllRoutes(
        ProtoAddress::Type protoAddressType,
        ProtoRouteTable& protoRouteTable)
    { return false; }//Note: dummy implementation

    virtual bool GetRoute(
        const ProtoAddress& destinationProtoAddress,
        unsigned int prefixLength,
        ProtoAddress& gatewayProtoAddress,
        unsigned int& protoInterfaceIndex,
        int& metric);

    virtual bool SetRoute(
        const ProtoAddress& destinationProtoAddress,
        unsigned int prefixLength,
        const ProtoAddress& gatewayProtoAddress,
        unsigned int protoInterfaceIndex,
        int metric);

    virtual bool DeleteRoute(
        const ProtoAddress& destinationProtoAddress,
        unsigned int prefixLength,
        const ProtoAddress& gatewayProtoAddress,
        unsigned int protoInterfaceIndex);

    virtual bool SetForwarding(bool state)
    {
        forwardingIsOn = state;
        return true;
    }

    virtual unsigned int GetInterfaceIndex(
        const char* protoInterfaceName)
    { return 1; }//Note: dummy implementation

    virtual bool GetInterfaceName(
        unsigned int protoInterfaceIndex,
        char* buffer,
        unsigned int buflen)
    { return true; }

    // Note: retrieve all addresses for a given interface
    //       and add to the provided address list
    virtual bool GetInterfaceAddressList(
        unsigned int ifIndex,
        ProtoAddress::Type protoAddressType,
        ProtoAddressList& protoAddressList);


private:
    shared_ptr<NetworkLayer> networkLayerPtr;
    unsigned int interfaceIndex;
    NetworkAddress interfaceAddress;

    int forwardingIsOn;

    ProtoRouteTable protoRouteTable;


    //Note: required for accessing trace and statistics output functions
    shared_ptr<NrlolsrProtocol> nrlolsrProtocolPtr;

    // Disable:
    ScenargieRouteMgr(const ScenargieRouteMgr&);
    void operator=(const ScenargieRouteMgr&);

};//ScenargieRouteMgr//




inline
bool ScenargieRouteMgr::Open(const void* interfaceDataPtr)
{
    interfaceAddress = *(NetworkAddress*)interfaceDataPtr;
    protoRouteTable.Init();

    return true;
}//Open//


inline
bool ScenargieRouteMgr::GetRoute(
    const ProtoAddress& destinationProtoAddress,
    unsigned int prefixLength,
    ProtoAddress& gatewayProtoAddress,
    unsigned int& protoInterfaceIndex,
    int& metric)
{
    bool foundARoute = protoRouteTable.FindRoute(
        destinationProtoAddress,
        prefixLength,
        gatewayProtoAddress,
        protoInterfaceIndex,
        metric);

    return foundARoute;
}//GetRoute//


inline
bool ScenargieRouteMgr::GetInterfaceAddressList(
    unsigned int protoInterfaceIndex,
    ProtoAddress::Type protoAddressType,
    ProtoAddressList& protoAddressList)
{
    ProtoAddress interfaceProtoAddress;
    interfaceProtoAddress.SimSetAddress(interfaceAddress.GetRawAddressLow32Bits());

    return (protoAddressList.Insert(interfaceProtoAddress));
}//GetInterfaceAddress//





//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
class NrlolsrProtocol : public ScenargieProtoSimAgent,
                        public ProtocolPacketHandler,
                        public enable_shared_from_this<NrlolsrProtocol>
{
    //Note: required for accessing trace and statistics output functions
    friend class ScenargieRouteMgr;

public:
    static const string modelName;


    //Note: assigned by IANA for OLSR
    static const unsigned short int OLSR_PORT = 698;


    NrlolsrProtocol(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const unsigned int initInterfaceIndex,
        const InterfaceIdType& initInterfaceId,
        const RandomNumberGeneratorSeedType& nodeSeed);

    void CompleteInitialization(
        const ParameterDatabaseReader& theParameterDatabaseReader);

    ~NrlolsrProtocol() { assert(initializationIsComplete); }



    virtual void ReceivePacketFromNetworkLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceNetworkAddress,
        const NetworkAddress& destinationNetworkAddress,
        const PacketPriorityType trafficClass_notused,
        const NetworkAddress& lastHopNetworkAddress_notused,
        const unsigned char ttl_notused,
        const unsigned int interfaceIndex_notused);

    virtual void GetPortNumbersFromPacket(
        const Packet& aPacket,
        const unsigned int transportHeaderOffset,
        bool& portNumbersWereRetrieved,
        unsigned short int& sourcePort,
        unsigned short int& destinationPort) const
    { assert(false && "This function is not available for NrlolsrProtocol"); }


protected:

    ProtoSimAgent::SocketProxy* OpenSocket(ProtoSocket& theSocket);
    void CloseSocket(ProtoSocket& theSocket);



    class UdpSocketProxy : public ScenargieProtoSimAgent::ScenargieSocketProxy
    {
    public:
        UdpSocketProxy(const shared_ptr<NrlolsrProtocol>& initNrlolsrProtocolPtr)
            : nrlolsrProtocolPtr(initNrlolsrProtocolPtr)
        { }

        virtual ~UdpSocketProxy() { }

        virtual bool SendTo(
            const char* payloadBuffer,
            unsigned int& payloadSizeInBytes,
            const ProtoAddress& destinationProtoAddress);

        virtual void Receive(
            unique_ptr<Packet>& packetPtr,
            const ProtoAddress& sourceProtoAddress,
            const ProtoAddress& destinationProtoAddress);

    private:
        shared_ptr<NrlolsrProtocol> nrlolsrProtocolPtr;
    };//UdpSocketProxy//



    //Note: dummy implementation
    class TcpSocketProxy : public ScenargieSocketProxy
    {
    public:
        TcpSocketProxy() { assert(false && "This function is not available for NrlolsrProtocol"); }

        virtual ~TcpSocketProxy() { }

        virtual bool SendTo(
            const char* payloadBuffer,
            unsigned int& payloadSizeInBytes,
            const ProtoAddress& destinationProtoAddress)
        { assert(false && "This function is not available for NrlolsrProtocol"); return false; }

        void Receive(
            unique_ptr<Packet>& packetPtr,
            const ProtoAddress& sourceProtoAddress,
            const ProtoAddress& destinationProtoAddress)
        { assert(false && "This function is not available for NrlolsrProtocol"); }
    };//TcpSocketProxy//




private:
    const RandomNumberGeneratorSeedType nodeSeed;
    bool initializationIsComplete;

    ScenargieRouteMgr* protoRoutingTablePtr;
    Nrlolsr* nrlolsrPtr;

    unsigned int currentPacketSequenceNumber;
    InterfaceIdType interfaceId;

    bool OnStartup(char* commandString);
    void OnShutdown();

    bool OnStartup(int argc, const char*const* argv);
    bool ProcessCommands(int argc, const char*const* argv);

    bool GetRoute (
        SIMADDR destinationRawAddress,
        SIMADDR& route);

    void SendMulticastPacketToNetworkLayer(
        unique_ptr<Packet> packetPtr,
        const NetworkAddress& destinationNetworkAddress,
        unsigned char ttl)
    { assert(false && "This function is not available for NrlolsrProtocol"); }

    void SendBroadcastPacketToNetworkLayer(
        unique_ptr<Packet> packetPtr,
        const NetworkAddress& destinationNetworkAddress,
        const unsigned char ttl)
    { assert(false && "This function is not available for NrlolsrProtocol"); }

    void SendPacketToNetworkLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& destinationNetworkAddress,
        const unsigned char ttl,
        const unsigned short int flowLabel);

    void ReadParameterConfiguration(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        std::stringstream& stringStream);



    //Trace and statistics
    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;

    string GetOlsrMessageType(const Packet& packet) const;

    void GetOlsrMessageAttributes(
        const Packet& packet,
        unsigned char& olsrMessageType,
        unsigned char& validityTime,
        unsigned int& originatorRawAddress,
        unsigned int& olsrMessageSequenceNumber,
        unsigned char& ttl,
        unsigned char& hopCount) const;

    string ConverOlsrMessageTypeToString(unsigned int olsrMessageType);


    void OutputTraceAndStatsForSendOlsrPacket(
        const Packet& packet,
        const NetworkAddress& destinationNetworkAddress) const;

    void OutputTraceAndStatsForReceiveOlsrPacket(
        const Packet& packet,
        const NetworkAddress& destinationNetworkAddress) const;

    void OutputTraceAndStatsForAddRoutingTableEntry(
        const NetworkAddress& destinationAddress,
        const NetworkAddress& netmaskAddress,
        const NetworkAddress& nextHopAddress) const;

    void OutputTraceAndStatsForDeleteRoutingTableEntry(
        const NetworkAddress& destinationAddress,
        const NetworkAddress& netmaskAddress) const;

};//NrlolsrProtocol//





inline
NrlolsrProtocol::NrlolsrProtocol(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const unsigned int initInterfaceIndex,
    const InterfaceIdType& initInterfaceId,
    const RandomNumberGeneratorSeedType& initNodeSeed)
    :
    ScenargieProtoSimAgent(
        initSimulationEngineInterfacePtr,
        initNetworkLayerPtr,
        initInterfaceIndex),
    nodeSeed(initNodeSeed),
    initializationIsComplete(false),
    currentPacketSequenceNumber(0),
    interfaceId(initInterfaceId),
    packetsSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            modelName + "_" + initInterfaceId + "_PacketsSent")),
    packetsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            modelName + "_" + initInterfaceId + "_PacketsReceived")),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            modelName + "_" + initInterfaceId + "_BytesSent")),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            modelName + "_" + initInterfaceId + "_BytesReceived"))
{
}//NrlolsrProtocol//


inline
void NrlolsrProtocol::CompleteInitialization(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    nrlolsrPtr =
        new Nrlolsr(
            shared_from_this()->GetSimulationEngineInterfacePtr(),
            nodeSeed,
            GetSocketNotifier(),
            GetTimerMgr());

    NodeIdType nodeId = networkLayerPtr->GetNodeId();


    //Note: nrlolsr options(<n>) are passed as command-line arguments
    //
    // argv   : (name<1>) (value<1>) (name<2>) (value<2>)... '\0'
    // [index]: 0        1         2          3         4    argc
    // argc   : sizeof(argv) - 1
    //

    std::stringstream argvStringStream;


    (*this).ReadParameterConfiguration(
        theParameterDatabaseReader, nodeId, argvStringStream);

    //Note: parameter for simulation environment
    argvStringStream << "target";

    char* argvString = new char [argvStringStream.str().size() + 1];
    strcpy(argvString, argvStringStream.str().c_str());

    (*this).OnStartup(argvString);

    delete[] argvString;

    initializationIsComplete = true;

}//CompleteInitialization//



inline
bool NrlolsrProtocol::OnStartup(char* commandString)
{
    protoRoutingTablePtr =
        ScenargieRouteMgr::Create(
            shared_from_this(), networkLayerPtr, interfaceIndex);

    if (protoRoutingTablePtr != nullptr) {

        if (!protoRoutingTablePtr->Open((void*)&interfaceAddress)) {
            return false;
        }//if//

    }
    else {
        return false;
    }//if//

    nrlolsrPtr->SetOlsrRouteTable(protoRoutingTablePtr);

    if (nrlolsrPtr->StringProcessCommands(commandString)) {
        nrlolsrPtr->Start();
    }
    else {
        return false;
    }//if//

    return true;

}//OnStartup//


inline
bool NrlolsrProtocol::OnStartup(int argc, const char*const* argv)
{
    protoRoutingTablePtr =
        ScenargieRouteMgr::Create(
            shared_from_this(), networkLayerPtr, interfaceIndex);

    if (protoRoutingTablePtr != nullptr) {

        if (!protoRoutingTablePtr->Open((void*)&interfaceAddress)) {
            return false;
        }//if//

    }
    else {
        return false;
    }//if//

    nrlolsrPtr->SetOlsrRouteTable(protoRoutingTablePtr);

    if (ProcessCommands(argc, argv)) {
        nrlolsrPtr->Start();
    }
    else {
        return false;
    }//if//


    return true;
}//OnStartup//


inline
bool NrlolsrProtocol::ProcessCommands(int argc, const char*const* argv)
{
    return nrlolsrPtr->ProcessCommands(argc, argv);

}//ProcessCommands//


inline
void NrlolsrProtocol::OnShutdown()
{
    nrlolsrPtr->Stop();
    protoRoutingTablePtr->Close();

    delete protoRoutingTablePtr;
    protoRoutingTablePtr = nullptr;

    return;
}//OnShutdown//



inline
bool NrlolsrProtocol::GetRoute(
    SIMADDR destinationRawAddress,
    SIMADDR& route)
{
    assert(protoRoutingTablePtr != nullptr);

    bool foundRouteEntry = false;

    ProtoAddress destinationProtoAddress;
    ProtoAddress gatewayProtoAddress;
    unsigned int protoInterfaceIndex;
    unsigned int prefixLength = 32;
    int metric;

    destinationProtoAddress.SimSetAddress(destinationRawAddress);

    foundRouteEntry = protoRoutingTablePtr->GetRoute(
        destinationProtoAddress,
        prefixLength,
        gatewayProtoAddress,
        protoInterfaceIndex,
        metric);


    if (foundRouteEntry) {

        if (gatewayProtoAddress.IsValid()) {
            route = gatewayProtoAddress.SimGetAddress();
        }
        else {
            route = destinationProtoAddress.SimGetAddress();
        }//if//

    }//if//

    return foundRouteEntry;

}//GetRoute//



inline
void NrlolsrProtocol::SendPacketToNetworkLayer(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& destinationNetworkAddress,
    const unsigned char ttl,
    const unsigned short int flowLabel)
{
    const unsigned char trafficClass = 1;

    IpHeaderModel
        header(
            trafficClass,
            packetPtr->LengthBytes(),
            ttl,
            IP_PROTOCOL_NUMBER_OLSR,
            networkLayerPtr->GetNetworkAddress(interfaceIndex),
            destinationNetworkAddress);

    packetPtr->AddRawHeader(header.GetPointerToRawBytes(), header.GetNumberOfRawBytes());
    packetPtr->AddTrailingPadding(header.GetNumberOfTrailingBytes());

    if (header.usingVersion6) {
        shared_ptr<IpHeaderOverlayModel> ipHeaderPtr =
            shared_ptr<IpHeaderOverlayModel>(
                new IpHeaderOverlayModel(
                    packetPtr->GetRawPayloadData(), packetPtr->LengthBytes()));
        ipHeaderPtr->SetFlowLabel(flowLabel);
    }

    networkLayerPtr->ReceiveRoutedNetworkPacketFromRoutingProtocol(
        packetPtr,
        interfaceIndex,
        destinationNetworkAddress);

}//SendPacketToNetworkLayer//




inline
void NrlolsrProtocol::ReceivePacketFromNetworkLayer(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress,
    const PacketPriorityType trafficClass_notused,
    const NetworkAddress& lastHopAddress_notused,
    const unsigned char ttl_notused,
    const unsigned int interfaceIndex_notused)
{
    ProtoAddress sourceProtoAddress;
    sourceProtoAddress.SimSetAddress(
        (SIMADDR)sourceAddress.GetRawAddressLow32Bits());

    ProtoAddress destinationProtoAddress;
    destinationProtoAddress.SimSetAddress(
        (SIMADDR)destinationAddress.GetRawAddressLow32Bits());

    ProtoSocket::Proxy* proxy = nrlolsrPtr->GetSocket().GetHandle();
    NrlolsrProtocol::UdpSocketProxy* udpSocketProxyPtr =
        static_cast<NrlolsrProtocol::UdpSocketProxy*>(proxy);

    udpSocketProxyPtr->Receive(
        packetPtr,
        sourceProtoAddress,
        destinationProtoAddress);

}//ReceivePacketFromNetworkLayer//



inline
bool NrlolsrProtocol::UdpSocketProxy::SendTo(
    const char* payloadBuffer,
    unsigned int& payloadSizeInBytes,
    const ProtoAddress& destinationProtoAddress)
{
    assert((nrlolsrProtocolPtr->currentPacketSequenceNumber + 1) < UINT_MAX);
    nrlolsrProtocolPtr->currentPacketSequenceNumber++;

    const unsigned short int destinationPort = destinationProtoAddress.GetPort();
    const unsigned short int sourcePort = OLSR_PORT;

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(
            *(nrlolsrProtocolPtr->GetSimulationEngineInterfacePtr()),
            (const unsigned char*)payloadBuffer,
            payloadSizeInBytes);

    NetworkAddress destinationNetworkAddress(
        destinationProtoAddress.SimGetAddress());

    unsigned char ttl;
    if (destinationProtoAddress.IsMulticast()) {
        ttl = sharedTtl;
    }
    else {
        ttl = 255;
    }//if//

    assert(ttl != 0);

    nrlolsrProtocolPtr->OutputTraceAndStatsForSendOlsrPacket(
        *packetPtr,
        destinationNetworkAddress);

    packetPtr->AddPlainStructHeader(
        UdpHeaderType(
            HostToNet16(sourcePort),
            HostToNet16(destinationPort),
            HostToNet16(static_cast<unsigned short int>(
                packetPtr->LengthBytes() + sizeof(UdpHeaderType)))));


    assert(sharedFlowLabel <= USHRT_MAX);

    nrlolsrProtocolPtr->SendPacketToNetworkLayer(
        packetPtr,
        destinationNetworkAddress,
        ttl,
        static_cast<unsigned short int>(sharedFlowLabel));

    return true;
}//SendTo//


inline
void NrlolsrProtocol::UdpSocketProxy::Receive(
    unique_ptr<Packet>& packetPtr,
    const ProtoAddress& sourceProtoAddress,
    const ProtoAddress& destinationProtoAddress)
{
    UdpHeaderType udpHeader =
        packetPtr->GetAndReinterpretPayloadData<UdpHeaderType>();

    packetPtr->DeleteHeader(sizeof(UdpHeaderType));

    sharedDataBuffer = (char*)packetPtr->GetRawPayloadData();
    sharedDataBufferLength = static_cast<UINT16>(packetPtr->LengthBytes());
    sharedSourceProtoAddress = sourceProtoAddress;

    const NetworkAddress destinationNetworkAddress(
        destinationProtoAddress.SimGetAddress());

    nrlolsrProtocolPtr->OutputTraceAndStatsForReceiveOlsrPacket(
        *packetPtr,
        destinationNetworkAddress);

    if (proto_socket != nullptr) {
        proto_socket->OnNotify(ProtoSocket::NOTIFY_INPUT);
    }//if//

    packetPtr = nullptr;
}//Receive//



inline
ProtoSimAgent::SocketProxy* NrlolsrProtocol::OpenSocket(
    ProtoSocket& theSocket)
{
    assert((theSocket.GetProtocol() == ProtoSocket::UDP) ||
           (theSocket.GetProtocol() == ProtoSocket::TCP));

    if (theSocket.GetProtocol() == ProtoSocket::UDP) {

        UdpSocketProxy* udpSocketProxyPtr =
            new UdpSocketProxy(shared_from_this());

        if (udpSocketProxyPtr != nullptr) {
            udpSocketProxyPtr->AttachSocket(theSocket);
            socketProxyList.Prepend(*udpSocketProxyPtr);

            return udpSocketProxyPtr;
        }
        else {
            return nullptr;
        }//if//

    }
    else if (theSocket.GetProtocol() == ProtoSocket::TCP) {

        TcpSocketProxy* tcpSocketProxyPtr =
            new TcpSocketProxy();

        if (tcpSocketProxyPtr != nullptr) {
            tcpSocketProxyPtr->AttachSocket(theSocket);
            socketProxyList.Prepend(*tcpSocketProxyPtr);

            return tcpSocketProxyPtr;
        }
        else {
            return nullptr;
        }//if//

    }
    else {
        return nullptr;
    }//if//
}//OpenSocket//


inline
void NrlolsrProtocol::CloseSocket(ProtoSocket& theSocket)
{
    assert(theSocket.IsOpen());

    ScenargieSocketProxy* socketProxy =
        static_cast<ScenargieSocketProxy*>(theSocket.GetHandle());

    socketProxyList.Remove(*socketProxy);
    delete socketProxy;

}//CloseSocket//



inline
void NrlolsrProtocol::ReadParameterConfiguration(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    std::stringstream& stringStream)
{
    //[-flooding off | s-mpr | ns-mpr | not-sym | simple | ecds | mpr-cds]
    if (theParameterDatabaseReader.ParameterExists("olsr-flooding-method", nodeId, interfaceId)) {

        string floodingMethod =
            theParameterDatabaseReader.ReadString("olsr-flooding-method", nodeId, interfaceId);

        ConvertStringToLowerCase(floodingMethod);

        if ((floodingMethod == "off")
            || (floodingMethod == "s-mpr")
            || (floodingMethod == "ns-mpr")
            || (floodingMethod == "not-sym")
            || (floodingMethod == "simple")
            || (floodingMethod == "ecds")
            || (floodingMethod == "mpr-cds")) {

            stringStream << "-flooding" << ' ' << floodingMethod << ' ';
        }
        else {
            cerr << "Invalid String (olsr-flooding-method): "
                 << floodingMethod << endl;
            exit(1);
        }//if//
    }//if//


    //[-fdelay <MaxForwardDelay>]
    if (theParameterDatabaseReader.ParameterExists("olsr-forward-delay", nodeId, interfaceId)) {

        const TimeType maximumForwardDelay =
            theParameterDatabaseReader.ReadTime("olsr-forward-delay", nodeId, interfaceId);

        if (maximumForwardDelay >= ZERO_TIME) {
            string parameterValue = ConvertTimeToStringSecs(maximumForwardDelay);
            stringStream << "-fdelay" << ' ' << parameterValue << ' ';
        }
        else {
            cerr << "Invalid value (olsr-forward-delay): "
                 << maximumForwardDelay << endl;
            exit(1);
        }//if//
    }//if//


    //[-hi <HelloInterval>]
    if (theParameterDatabaseReader.ParameterExists("olsr-hello-interval", nodeId, interfaceId)) {

        const TimeType helloInterval =
            theParameterDatabaseReader.ReadTime("olsr-hello-interval", nodeId, interfaceId);

        if (helloInterval > ZERO_TIME) {
            string parameterValue = ConvertTimeToStringSecs(helloInterval);
            stringStream << "-hi" << ' ' << parameterValue << ' ';
        }
        else {
            cerr << "Invalid value (olsr-hello-interval): "
                 << helloInterval << endl;
            exit(1);
        }//if//
    }//if//


    //[-hj <HelloJitter>]
    if (theParameterDatabaseReader.ParameterExists("olsr-hello-jitter", nodeId, interfaceId)) {

        const double helloJitter =
            theParameterDatabaseReader.ReadDouble("olsr-hello-jitter", nodeId, interfaceId);

        if ((helloJitter >= 0.0) && (helloJitter < 1.0)) {
            string parameterValue = ConvertToString(helloJitter);
            stringStream << "-hj" << ' ' << parameterValue << ' ';
        }
        else {
            cerr << "Invalid value (olsr-hello-jitter): "
                 << helloJitter << endl;
            exit(1);
        }//if//
    }//if//


    //[-ht <HelloTimeoutfactor>]
    if (theParameterDatabaseReader.ParameterExists("olsr-hello-timeout-factor", nodeId, interfaceId)) {

        const double helloTimeoutFactor =
            theParameterDatabaseReader.ReadDouble("olsr-hello-timeout-factor", nodeId, interfaceId);

        if (helloTimeoutFactor > 1.0) {
            string parameterValue = ConvertToString(helloTimeoutFactor);
            stringStream << "-ht" << ' ' << parameterValue << ' ';
        }
        else {
            cerr << "Invalid value (olsr-hello-timeout-factor): "
                 << helloTimeoutFactor << endl;
            exit(1);
        }//if//
    }//if//


    //[-shortesthop] [-spf] [-minmax] [-robustroute]
    if (theParameterDatabaseReader.ParameterExists("olsr-shortest-path-algorithm", nodeId, interfaceId)) {

        string shortestPathAlgorithm =
            theParameterDatabaseReader.ReadString("olsr-shortest-path-algorithm", nodeId, interfaceId);

        ConvertStringToLowerCase(shortestPathAlgorithm);

        if (shortestPathAlgorithm == "shortesthop") {

            stringStream << "-shortesthop" << ' ';
        }
        else if (shortestPathAlgorithm == "spf") {

            stringStream << "-spf" << ' ';
        }
        else if (shortestPathAlgorithm == "minmax") {

            stringStream << "-minmax" << ' ';
        }
        else if (shortestPathAlgorithm == "robustroute") {

            stringStream << "-robustroute" << ' ';
        }
        else {
            cerr << "Invalid String (olsr-shortest-path-algorithm): "
                 << shortestPathAlgorithm << endl;
            exit(1);
        }//if//
    }//if//


    //[-tci <TCInterval>]
    if (theParameterDatabaseReader.ParameterExists("olsr-tc-interval", nodeId, interfaceId)) {

        const TimeType tcInterval =
            theParameterDatabaseReader.ReadTime("olsr-tc-interval", nodeId, interfaceId);

        if (tcInterval > ZERO_TIME) {
            string parameterValue = ConvertTimeToStringSecs(tcInterval);
            stringStream << "-tci" << ' ' << parameterValue << ' ';
        }
        else {
            cerr << "Invalid value (olsr-tc-interval): "
                 << tcInterval << endl;
            exit(1);
        }//if//
    }//if//


    //[-tcj <TCJitter>]
    if (theParameterDatabaseReader.ParameterExists("olsr-tc-jitter", nodeId, interfaceId)) {

        const double tcJitter =
            theParameterDatabaseReader.ReadDouble("olsr-tc-jitter", nodeId, interfaceId);

        if ((tcJitter >= 0.0) && (tcJitter < 1.0)) {
            string parameterValue = ConvertToString(tcJitter);
            stringStream << "-tcj" << ' ' << parameterValue << ' ';
        }
        else {
            cerr << "Invalid value (olsr-tc-jitter): "
                 << tcJitter << endl;
            exit(1);
        }//if//
    }//if//


    //[-tct <TCTimeoutfactor>]
    if (theParameterDatabaseReader.ParameterExists("olsr-tc-timeout-factor", nodeId, interfaceId)) {

        const double tcTimeoutFactor =
            theParameterDatabaseReader.ReadDouble("olsr-tc-timeout-factor", nodeId, interfaceId);

        if (tcTimeoutFactor > 1.0) {
            string parameterValue = ConvertToString(tcTimeoutFactor);
            stringStream << "-tct" << ' ' << parameterValue << ' ';
        }
        else {
            cerr << "Invalid value (olsr-tc-timeout-factor): "
                 << tcTimeoutFactor << endl;
            exit(1);
        }//if//
    }//if//


    //[-w <willingness>]
    if (theParameterDatabaseReader.ParameterExists("olsr-willingness", nodeId, interfaceId)) {

        const int nodeWillingness =
            theParameterDatabaseReader.ReadInt("olsr-willingness", nodeId, interfaceId);

        if ((nodeWillingness >= 0) && (nodeWillingness <= 7)) {
            string parameterValue = ConvertToString(nodeWillingness);
            stringStream << "-w" << ' ' << parameterValue << ' ';
        }
        else {
            cerr << "Invalid value (olsr-willingness): "
                 << nodeWillingness << endl;
            exit(1);
        }//if//
    }//if//

}//ReadParameterConfiguration//

//-------------------------------------------------------------------------


inline
string NrlolsrProtocol::GetOlsrMessageType(
    const Packet& packet) const
{
    char* buffer = (char*)packet.GetRawPayloadData();
    unsigned int len = static_cast<unsigned int>(packet.LengthBytes());

    ProtoAddress::Type ipvMode = ProtoAddress::SIM;

    OlsrPacket olsrPacket;
    olsrPacket.unpack(buffer, len, ipvMode);

    char* littlebuffer;
    int olsrMessageSize = 0;

    while ((littlebuffer = (char*)olsrPacket.messages.peekNext(&olsrMessageSize))) {

        OlsrMessage olsrMessage;
        olsrMessage.unpack(littlebuffer, olsrMessageSize, ipvMode);

        if (olsrMessage.type == NRLOLSR_HELLO) {
            return "OLSR_HELLO";
        }
        if (olsrMessage.type == NRLOLSR_TC_MESSAGE) {
            return "OLSR_TC";
        }
        if (olsrMessage.type == NRLOLSR_TC_MESSAGE_EXTRA) {
            return "OLSR_TC_EXTRA";
        }
        if (olsrMessage.type == NRLOLSR_HNA_MESSAGE) {
            return "OLSR_HNA";
        }

    }//while//

    return "";

}//GetOlsrPacketType//


inline
void NrlolsrProtocol::GetOlsrMessageAttributes(
    const Packet& packet,
    unsigned char& olsrMessageType,
    unsigned char& validityTime,
    unsigned int& originatorRawAddress,
    unsigned int& olsrMessageSequenceNumber,
    unsigned char& ttl,
    unsigned char& hopCount) const
{
    char* buffer = (char*)packet.GetRawPayloadData();
    unsigned int len = static_cast<unsigned int>(packet.LengthBytes());

    ProtoAddress::Type ipvMode = ProtoAddress::SIM;

    OlsrPacket olsrPacket;
    olsrPacket.unpack(buffer, len, ipvMode);

    char* littlebuffer;
    int olsrMessageSize = 0;

    while (true) {
        littlebuffer = (char*)olsrPacket.messages.peekNext(&olsrMessageSize);
        if (littlebuffer == nullptr) {
            break;
        }

        OlsrMessage olsrMessage;
        olsrMessage.unpack(littlebuffer, olsrMessageSize, ipvMode);

        olsrMessageType = olsrMessage.type;
        validityTime = olsrMessage.Vtime;

        originatorRawAddress = olsrMessage.O_addr.SimGetAddress();
        olsrMessageSequenceNumber = olsrMessage.D_seq_num;
        ttl = olsrMessage.ttl;
        hopCount = olsrMessage.hopc;

    }//while//

}//GetOlsrMessageAttributes//


inline
string ConvertOlsrMessageTypeToString(unsigned char olsrMessageType)
{
    string olsrMessageTypeInString;

    switch (olsrMessageType) {
    case NRLOLSR_HELLO:
        olsrMessageTypeInString += "OLSR_HELLO";
        break;
    case NRLOLSR_TC_MESSAGE:
        olsrMessageTypeInString += "OLSR_TC";
        break;
    case NRLOLSR_TC_MESSAGE_EXTRA:
        olsrMessageTypeInString += "OLSR_TC_EXTRA";
        break;
    case NRLOLSR_HNA_MESSAGE:
        olsrMessageTypeInString += "OLSR_HNA";
        break;
    default:
        olsrMessageTypeInString += "";
        break;
    }//switch//

    return olsrMessageTypeInString;
}//ConvertOlsrMessageTypeToString//


inline
void NrlolsrProtocol::OutputTraceAndStatsForSendOlsrPacket(
    const Packet& packet,
    const NetworkAddress& destinationNetworkAddress) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            OlsrSendTraceRecord traceData;

            const PacketIdType& packetId = packet.GetPacketId();

            unsigned char olsrMessageType;
            unsigned char validityTime;
            unsigned int originatorRawAddress;
            unsigned int olsrMessageSequenceNumber;
            unsigned char hopLimit;
            unsigned char hopCount;

            GetOlsrMessageAttributes(
                packet,
                olsrMessageType,
                validityTime,
                originatorRawAddress,
                olsrMessageSequenceNumber,
                hopLimit,
                hopCount);

            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.sourceNodeId = packetId.GetSourceNodeId();

            traceData.messageType = olsrMessageType;
            traceData.validityTime = validityTime;

            traceData.originatorSequenceNumber = olsrMessageSequenceNumber;
            traceData.originatorAddress = Ipv6NetworkAddress(originatorRawAddress);

            traceData.hopLimit = hopLimit;
            traceData.hopCount = hopCount;
            traceData.useIpv6 = (NetworkAddress::numberBits == 128);

            assert(sizeof(traceData) == OLSR_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "OlsrSend", traceData);

        }
        else {

            ostringstream outStream;

            const PacketIdType& packetId = packet.GetPacketId();

            unsigned char olsrMessageType;
            unsigned char validityTime;
            unsigned int originatorRawAddress;
            unsigned int olsrMessageSequenceNumber;
            unsigned char hopLimit;
            unsigned char hopCount;

            GetOlsrMessageAttributes(
                packet,
                olsrMessageType,
                validityTime,
                originatorRawAddress,
                olsrMessageSequenceNumber,
                hopLimit,
                hopCount);

            outStream << "PktId= " << packetId;
            outStream << " Type= " << ConvertOlsrMessageTypeToString(olsrMessageType);
            outStream << " VTime= " << static_cast<int>(validityTime);

            outStream << " OrgAddr= " << NetworkAddress(originatorRawAddress).ConvertToString();

            outStream << " HopLimit= " << static_cast<int>(hopLimit);
            outStream << " HopCount= " << static_cast<int>(hopCount);
            outStream << " OlsrMsgId= " << olsrMessageSequenceNumber;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "OlsrSend", outStream.str());

        }//if//


    }//if//

    packetsSentStatPtr->IncrementCounter();

    const unsigned int packetLengthInBytes = packet.LengthBytes();
    bytesSentStatPtr->IncrementCounter(packetLengthInBytes);

}//OutputTraceAndStatsForSendOlsrPacket//


inline
void NrlolsrProtocol::OutputTraceAndStatsForReceiveOlsrPacket(
    const Packet& packet,
    const NetworkAddress& destinationNetworkAddress) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            OlsrReceiveTraceRecord traceData;

            const PacketIdType& packetId = packet.GetPacketId();

            unsigned char olsrMessageType;
            unsigned char validityTime;
            unsigned int originatorRawAddress;
            unsigned int olsrMessageSequenceNumber;
            unsigned char hopLimit;
            unsigned char hopCount;

            GetOlsrMessageAttributes(
                packet,
                olsrMessageType,
                validityTime,
                originatorRawAddress,
                olsrMessageSequenceNumber,
                hopLimit,
                hopCount);

            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.sourceNodeId = packetId.GetSourceNodeId();

            traceData.messageType = olsrMessageType;
            traceData.validityTime = validityTime;

            traceData.originatorSequenceNumber = olsrMessageSequenceNumber;
            traceData.originatorAddress = Ipv6NetworkAddress(originatorRawAddress);

            traceData.hopLimit = hopLimit;
            traceData.hopCount = hopCount;
            traceData.useIpv6 = (NetworkAddress::numberBits == 128);

            assert(sizeof(traceData) == OLSR_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "OlsrRecv", traceData);

        }
        else {

            ostringstream outStream;

            const PacketIdType& packetId = packet.GetPacketId();

            unsigned char olsrMessageType;
            unsigned char validityTime;
            unsigned int originatorRawAddress;
            unsigned int olsrMessageSequenceNumber;
            unsigned char hopLimit;
            unsigned char hopCount;

            GetOlsrMessageAttributes(
                packet,
                olsrMessageType,
                validityTime,
                originatorRawAddress,
                olsrMessageSequenceNumber,
                hopLimit,
                hopCount);

            outStream << "PktId= " << packetId;
            outStream << " Type= " << ConvertOlsrMessageTypeToString(olsrMessageType);
            outStream << " VTime= " << static_cast<int>(validityTime);

            outStream << " OrgAddr= " << NetworkAddress(originatorRawAddress).ConvertToString();

            outStream << " HopLimit= " << static_cast<int>(hopLimit);
            outStream << " HopCount= " << static_cast<int>(hopCount);
            outStream << " OlsrMsgId= " << olsrMessageSequenceNumber;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "OlsrRecv", outStream.str());

        }//if//

    }//if//

    packetsReceivedStatPtr->IncrementCounter();

    const unsigned int packetLengthInBytes = packet.LengthBytes();
    bytesReceivedStatPtr->IncrementCounter(packetLengthInBytes);

}//OutputTraceAndStatsForReceiveOlsrPacket//


inline
void NrlolsrProtocol::OutputTraceAndStatsForAddRoutingTableEntry(
    const NetworkAddress& destinationAddress,
    const NetworkAddress& netmaskAddress,
    const NetworkAddress& nextHopAddress) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            RoutingTableAddEntryTraceRecord traceData;

            traceData.destinationAddress = Ipv6NetworkAddress(destinationAddress);
            traceData.netmaskAddress = Ipv6NetworkAddress(netmaskAddress);
            traceData.nextHopAddress = Ipv6NetworkAddress(nextHopAddress);
            traceData.localAddress =
                Ipv6NetworkAddress(networkLayerPtr->GetNetworkAddress(interfaceIndex));
            traceData.useIpv6 = (NetworkAddress::numberBits == 128);

            assert(sizeof(traceData) == ROUTING_TABLE_ADD_ENTRY_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "OlsrAddEntry", traceData);
        }
        else {

            ostringstream outStream;
            outStream
                << "Dest= " << destinationAddress.ConvertToString()
                << " Mask= " << netmaskAddress.ConvertToString()
                << " Next= " << nextHopAddress.ConvertToString()
                << " IF= " << networkLayerPtr->GetNetworkAddress(interfaceIndex).ConvertToString();

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "OlsrAddEntry", outStream.str());

        }//if//

    }//if//

}//OutputTraceAndStatsForAddRoutingTableEntry//


inline
void NrlolsrProtocol::OutputTraceAndStatsForDeleteRoutingTableEntry(
    const NetworkAddress& destinationAddress,
    const NetworkAddress& netmaskAddress) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceRouting)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            RoutingTableDeleteEntryTraceRecord traceData;

            traceData.destinationAddress = Ipv6NetworkAddress(destinationAddress);
            traceData.netmaskAddress = Ipv6NetworkAddress(netmaskAddress);
            traceData.useIpv6 = (NetworkAddress::numberBits == 128);

            assert(sizeof(traceData) == ROUTING_TABLE_DELETE_ENTRY_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "OlsrDelEntry", traceData);
        }
        else {

            ostringstream outStream;
            outStream
                << "Dest= " << destinationAddress.ConvertToString()
                << " Mask= " << netmaskAddress.ConvertToString();

            simulationEngineInterfacePtr->OutputTrace(
                modelName, interfaceId, "OlsrDelEntry", outStream.str());

        }//if//

    }//if//

}//OutputTraceAndStatsForDeleteRoutingTableEntry//


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------



}//namespace//

#endif
