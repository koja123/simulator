// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_TRANSPORT_H
#define SCENSIM_TRANSPORT_H

#include "scensim_packet.h"
#include "scensim_stats.h"
#include "scensim_tracedefs.h"
#include "randomnumbergen.h"

namespace ScenSim {


const unsigned short int ANY_PORT = 0;
const unsigned short int INVALID_PORT = 0;

//wellknown ports
//UDP
const unsigned short int DHCP_SERVER_PORT = 67;
const unsigned short int DHCP_CLIENT_PORT = 68;



class NetworkLayer;

class ProtocolPacketHandler {
public:
    virtual void ReceivePacketFromNetworkLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const NetworkAddress& lastHopAddress,
        const unsigned char hopLimit,
        const unsigned int interfaceIndex) = 0;

    virtual void GetPortNumbersFromPacket(
        const Packet& aPacket,
        const unsigned int transportHeaderOffset,
        bool& portNumbersWereRetrieved,
        unsigned short int& sourcePort,
        unsigned short int& destinationPort) const = 0;

    virtual ~ProtocolPacketHandler() { }
};



//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

struct UdpHeaderType {
    UdpHeaderType(
        unsigned short int initSourcePort,
        unsigned short int initDestinationPort,
        unsigned short int initLength)
        :
        sourcePort(initSourcePort),
        destinationPort(initDestinationPort),
        length(initLength),
        unused(0)
    {}
    unsigned short int sourcePort;
    unsigned short int destinationPort;
    unsigned short int length;
    unsigned short int unused;
};



class UdpProtocol: public ProtocolPacketHandler, public enable_shared_from_this<UdpProtocol> {
public:
    static const string modelName;

    UdpProtocol(const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr);

    void DisconnectFromOtherLayers();

    void ConnectToNetworkLayer(const shared_ptr<NetworkLayer>& networkLayerPtr);

    //-----------------------------------------------------
    // UDP Interface for Applications

    void SendPacket(
        unique_ptr<Packet>& packetPtr,
        const unsigned short int sourcePort,
        const NetworkAddress& destinationAddress,
        const unsigned short int destinationPort,
        const PacketPriorityType& priority);

    void SendPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const unsigned short int sourcePort,
        const NetworkAddress& destinationAddress,
        const unsigned short int destinationPort,
        const PacketPriorityType& priority);

    bool PortIsAvailable(const NetworkAddress& portAddress, const unsigned short int portNumber) const
    { return (portMap.find(PortKeyType(portNumber, portAddress)) == portMap.end()); }

    bool PortIsAvailable(const unsigned short int portNumber) const
    { return PortIsAvailable(NetworkAddress::anyAddress, portNumber); }

    //---------------------------------

    class PacketForAppFromTransportLayerHandler {
    public:
        virtual void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePort,
            const NetworkAddress& destinationAddress,
            const PacketPriorityType& priority) = 0;

        virtual ~PacketForAppFromTransportLayerHandler() { }

    };//PacketForAppFromTransportLayerHandler//

    //---------------------------------

    void OpenSpecificUdpPort(
        const NetworkAddress& address,
        const unsigned short int portNumber,
        const shared_ptr<PacketForAppFromTransportLayerHandler>& packetHandlerPtr);

    virtual void ReceivePacketFromNetworkLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const NetworkAddress& lastHopAddress_notused,
        const unsigned char hopLimit_notused,
        const unsigned int interfaceIndex);

    virtual void GetPortNumbersFromPacket(
        const Packet& aPacket,
        const unsigned int transportHeaderOffset,
        bool& portNumbersWereRetrieved,
        unsigned short int& sourcePort,
        unsigned short int& destinationPort) const;


private:
    shared_ptr<NetworkLayer> networkLayerPtr;

    struct PortKeyType {
        unsigned short int portNumber;
        NetworkAddress portAddress;

        PortKeyType(
            const unsigned short int initPortNumber,
            const NetworkAddress& initPortAddress)
            :
            portNumber(initPortNumber),
            portAddress(initPortAddress)
        { }

        bool operator<(const PortKeyType& right) const
        {
            return (((*this).portNumber < right.portNumber) ||
                    (((*this).portNumber == right.portNumber) &&
                     ((*this).portAddress < right.portAddress)));
        }

    };

    map<PortKeyType, shared_ptr<PacketForAppFromTransportLayerHandler> > portMap;

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;

    // Statistics
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;

    void OutputTraceAndStatsForSendPacketToNetworkLayer(const Packet& packet) const;
    void OutputTraceAndStatsForReceivePacketFromNetworkLayer(const Packet& packet) const;

    // Disable:
    UdpProtocol(const UdpProtocol&);
    void operator=(const UdpProtocol&);

};//UdpProtocol//

inline
UdpProtocol::UdpProtocol(const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(modelName + "_BytesReceived")),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(modelName + "_BytesSent"))
{
}


inline
void UdpProtocol::DisconnectFromOtherLayers()
{
    (*this).networkLayerPtr.reset();
    (*this).portMap.clear();
}


inline
void UdpProtocol::OpenSpecificUdpPort(
    const NetworkAddress& portAddress,
    const unsigned short int portNumber,
    const shared_ptr<PacketForAppFromTransportLayerHandler>& packetHandlerPtr)
{
    const PortKeyType key(portNumber, portAddress);

    assert(portMap.find(key) == portMap.end());
    portMap[key] = packetHandlerPtr;

}//OpenSpecificUdpPort//


inline
void UdpProtocol::OutputTraceAndStatsForSendPacketToNetworkLayer(const ScenSim::Packet &packet) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceTransport)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            UdpSendTraceRecord traceData;
            const PacketIdType& packetId = packet.GetPacketId();

            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == UDP_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "UdpSend", traceData);
        }
        else {
            ostringstream outStream;
            const PacketIdType& packetId = packet.GetPacketId();
            outStream << "PktId= " << packetId;
            simulationEngineInterfacePtr->OutputTrace(modelName, "", "UdpSend", outStream.str());

        }//if//
    }//if//

    const unsigned int packetLengthBytes = packet.LengthBytes();
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForSendPacketToNetworkLayer//


inline
void UdpProtocol::OutputTraceAndStatsForReceivePacketFromNetworkLayer(const ScenSim::Packet &packet) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceTransport)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            UdpReceiveTraceRecord traceData;
            const PacketIdType& packetId = packet.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == UDP_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "UdpRecv", traceData);

        }
        else {

            ostringstream outStream;
            const PacketIdType& packetId = packet.GetPacketId();
            outStream << "PktId= " << packetId;
            simulationEngineInterfacePtr->OutputTrace(modelName, "", "UdpRecv", outStream.str());

        }//if//
    }//if//

    const unsigned int packetLengthBytes = packet.LengthBytes();
    bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForReceivePacketFromNetworkLayer//


inline
void UdpProtocol::GetPortNumbersFromPacket(
    const Packet& aPacket,
    const unsigned int transportHeaderOffset,
    bool& portNumbersWereRetrieved,
    unsigned short int& sourcePort,
    unsigned short int& destinationPort) const
{
    const UdpHeaderType& udpHeader =
        aPacket.GetAndReinterpretPayloadData<UdpHeaderType>(transportHeaderOffset);

    sourcePort = NetToHost16(udpHeader.sourcePort);
    destinationPort = NetToHost16(udpHeader.destinationPort);
    portNumbersWereRetrieved = true;

}//GetPortNumbersFromPacket//



//--------------------------------------------------------------------------------------------------
// Connection Oriented protocols (Only TCP for now, but can be abstracted later).
//

class TcpProtocol;
class TcpConnection;

class ConnectionFromTcpProtocolHandler {
public:
    virtual void HandleNewConnection(const shared_ptr<TcpConnection>& connectionPtr) = 0;

    virtual ~ConnectionFromTcpProtocolHandler() { }
};

class TcpConnectionImplementation;

class TcpConnection {
public:
    ~TcpConnection();

    bool IsConnected() const;

    class AppTcpEventHandler {
    public:
        virtual ~AppTcpEventHandler() { }

        virtual void DoTcpIsReadyForMoreDataAction() = 0;
        virtual void ReceiveDataBlock(
            const unsigned char dataBlock[],
            const unsigned int dataLength,
            const unsigned int actualDataLength,
            bool& stallIncomingDataFlow) = 0;

        virtual void DoTcpRemoteHostClosedAction() { }
        virtual void DoTcpLocalHostClosedAction() { }

    };//AppTcpEventHandler//

    void EnableVirtualPayload();
    void SetAppTcpEventHandler(const shared_ptr<AppTcpEventHandler>& newAppTcpEventHandlerPtr);
    void ClearAppTcpEventHandler();

    void SendDataBlock(
        shared_ptr<vector<unsigned char> >& dataBlockPtr,
        const unsigned int dataLength);

    void SendDataBlock(
        shared_ptr<vector<unsigned char> >& dataBlockPtr)
    {
        SendDataBlock(
            dataBlockPtr,
            static_cast<unsigned int>(dataBlockPtr->size()));

    }//SendDataBlock//

    unsigned long long int GetNumberOfReceivedBytes() const;

    // Number bytes given to TCP.
    unsigned long long int GetNumberOfSentBytes() const;

    // Number Acked bytes.
    unsigned long long int GetNumberOfDeliveredBytes() const;

    // Buffered Bytes have not been sent to the IP layer yet.

    unsigned long long int GetCurrentNumberOfUnsentBufferedBytes() const;

    unsigned long long int GetCurrentNumberOfAvailableBufferBytes() const;

    NetworkAddress GetForeignAddress() const;

    void Close();

private:
    friend class TcpProtocol;
    friend class TcpProtocolImplementation;

    TcpConnection(
        const shared_ptr<TcpProtocol>& tcpProtocolPtr,
        const shared_ptr<TcpConnection::AppTcpEventHandler>& appEventHandlerPtr,
        const NetworkAddress& sourceAddress,
        const unsigned short int sourcePort,
        const NetworkAddress& destinationAddress,
        const unsigned short int destinationPort);

    auto_ptr<TcpConnectionImplementation> implPtr;

    // Disable:
    TcpConnection(const TcpConnection&);
    void operator=(const TcpConnection&);

};//TcpConnection//


class TcpProtocolImplementation;

class TcpProtocol: public ProtocolPacketHandler, public enable_shared_from_this<TcpProtocol> {
public:
    static const string modelName;

    TcpProtocol(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& nodeId,
        const RandomNumberGeneratorSeedType& nodeSeed);

    void ConnectToNetworkLayer(const shared_ptr<NetworkLayer>& networkLayerPtr);
    void DisconnectFromOtherLayers();

    ~TcpProtocol();

    //-------------------------------------------------------------------------

    void CreateOutgoingTcpConnection(
        const NetworkAddress& localAddress,
        const unsigned short int localPort,
        const NetworkAddress& destinationAddress,
        const unsigned short int destinationPort,
        const PacketPriorityType& priority,
        const shared_ptr<TcpConnection::AppTcpEventHandler>& appEventHandlerPtr,
        shared_ptr<TcpConnection>& newTcpConnectionPtr);

    void CreateOutgoingTcpConnection(
        const unsigned short int localPort,
        const NetworkAddress& destinationAddress,
        const unsigned short int destinationPort,
        const PacketPriorityType& priority,
        const shared_ptr<TcpConnection::AppTcpEventHandler>& appEventHandlerPtr,
        shared_ptr<TcpConnection>& newTcpConnectionPtr)
    {
        (*this).CreateOutgoingTcpConnection(
            NetworkAddress::anyAddress, localPort, destinationAddress, destinationPort, priority,
            appEventHandlerPtr, newTcpConnectionPtr);
    }

    bool PortIsAvailable(const int portNumber) const;

    void OpenSpecificTcpPort(
        const NetworkAddress& address,
        const unsigned short int portNumber,
        const shared_ptr<ConnectionFromTcpProtocolHandler>& connectionHandlerPtr);

    void DisconnectConnectionHandlerForPort(
        const NetworkAddress& address,
        const unsigned short int portNumber);

    //-------------------------------------------------------------------------

    virtual void ReceivePacketFromNetworkLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress,
        const PacketPriorityType trafficClass,
        const NetworkAddress& lastHopAddress_notused,
        const unsigned char hopLimit_notused,
        const unsigned int interfaceIndex_notused);

    virtual void GetPortNumbersFromPacket(
        const Packet& aPacket,
        const unsigned int transportHeaderOffset,
        bool& portNumbersWereRetrieved,
        unsigned short int& sourcePort,
        unsigned short int& destinationPort) const;

private:
    friend class TcpConnection;
    friend class TcpConnectionImplementation;
    friend class TcpProtocolImplementation;

    auto_ptr<TcpProtocolImplementation> implPtr;

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;

    //Statistics
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<RealStatistic> rttStatPtr;
    shared_ptr<RealStatistic> cwndStatPtr;
    shared_ptr<CounterStatistic> retransmissionStatPtr;

    void OutputTraceAndStatsForSendDataPacketToNetworkLayer(const Packet& packet) const;
    void OutputTraceAndStatsForSendControlPacketToNetworkLayer(const Packet& packet) const;
    void OutputTraceAndStatsForReceivePacketFromNetworkLayer(const Packet& packet) const;

    // Disable:
    TcpProtocol(const TcpProtocol&);
    void operator=(const TcpProtocol&);

};//TcpProtocol//


inline
void TcpProtocol::OutputTraceAndStatsForSendDataPacketToNetworkLayer(const ScenSim::Packet &packet) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceTransport)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            TcpDataSendTraceRecord traceData;
            const PacketIdType& packetId = packet.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == TCP_DATA_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "TcpDataSend", traceData);

        }
        else {

            ostringstream outStream;
            const PacketIdType& packetId = packet.GetPacketId();
            outStream << "PktId= " << packetId;
            simulationEngineInterfacePtr->OutputTrace(modelName, "", "TcpDataSend", outStream.str());

        }//if//
    }//if//

    const unsigned int packetLengthBytes = packet.LengthBytes();
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForSendDataPacketToNetworkLayer//


inline
void TcpProtocol::OutputTraceAndStatsForSendControlPacketToNetworkLayer(const ScenSim::Packet &packet) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceTransport)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            TcpControlSendTraceRecord traceData;
            const PacketIdType& packetId = packet.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == TCP_CONTROL_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "TcpCtrlSend", traceData);

        }
        else {

            ostringstream outStream;
            const PacketIdType& packetId = packet.GetPacketId();
            outStream << "PktId= " << packetId;
            simulationEngineInterfacePtr->OutputTrace(modelName, "", "TcpCtrlSend", outStream.str());

        }//if//
    }//if//

    const unsigned int packetLengthBytes = packet.LengthBytes();
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForSendControlPacketToNetworkLayer//


inline
void TcpProtocol::OutputTraceAndStatsForReceivePacketFromNetworkLayer(const ScenSim::Packet &packet) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceTransport)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            TcpReceiveTraceRecord traceData;
            const PacketIdType& packetId = packet.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == TCP_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(modelName, "", "TcpRecv", traceData);

        }
        else {

            ostringstream outStream;
            const PacketIdType& packetId = packet.GetPacketId();
            outStream << "PktId= " << packetId;
            simulationEngineInterfacePtr->OutputTrace(modelName, "", "TcpRecv", outStream.str());

        }//if//
    }//if//

    const unsigned int packetLengthBytes = packet.LengthBytes();
    bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForReceivePacketFromNetworkLayer//



class TransportLayer {
public:
    TransportLayer(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const NodeIdType& nodeId,
        const RandomNumberGeneratorSeedType& nodeSeed);

    shared_ptr<UdpProtocol> udpPtr;
    shared_ptr<TcpProtocol> tcpPtr;

    shared_ptr<NetworkLayer> GetNetworkLayerPtr() const { return networkLayerPtr; }

    void DisconnectProtocolsFromOtherLayers()
    {
        udpPtr->DisconnectFromOtherLayers();
        tcpPtr->DisconnectFromOtherLayers();
        networkLayerPtr.reset();
    }

private:
    shared_ptr<NetworkLayer> networkLayerPtr;

};//TransportLayer//


inline
TransportLayer::TransportLayer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const NodeIdType& nodeId,
    const RandomNumberGeneratorSeedType& nodeSeed)
    :
    networkLayerPtr(initNetworkLayerPtr),
    udpPtr(new UdpProtocol(simulationEngineInterfacePtr)),
    tcpPtr(new TcpProtocol(theParameterDatabaseReader, simulationEngineInterfacePtr, nodeId, nodeSeed))
{
    udpPtr->ConnectToNetworkLayer(networkLayerPtr);
    tcpPtr->ConnectToNetworkLayer(networkLayerPtr);
}



}//namespace//


#endif
