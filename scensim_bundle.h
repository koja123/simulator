// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_BUNDLE_H
#define SCENSIM_BUNDLE_H

#include "scensim_netsim.h"
#include <queue>

namespace ScenSim {

using std::cerr;
using std::endl;
using std::queue;

typedef unsigned long long int BundleIdType;

//--------------------------------------------------------------------------------------------------

class BundleProtocol: public Application, public enable_shared_from_this<BundleProtocol> {
public:
    static const string modelName;

    BundleProtocol(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const unsigned short int initDefaultDestinationPortId);

    void CompleteInitialization();

    void AddSenderSetting(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationIdType& applicationId,
        const NodeIdType& targetNodeId);

    ~BundleProtocol();

private:
    NodeIdType nodeId;
    unsigned short int destinationPortNumber;
    unsigned short int nextAvailableLocalPortNumber;
    PacketPriorityType controlPriority;
    PacketPriorityType messagePriority;
    bool useVirtualPayload;

    unsigned long long int maxStorageSizeBytes;

    //bundle properties
    TimeType bundleLifeTime;
    unsigned int maximumCopies;

    enum TransportModeType {
        TCP, UDP
    };

    TransportModeType transportMode;

    enum RoutingAlgorithmType {
        EPIDEMIC,
        DIRECT_DELIVERY,
        SPRAY_AND_WAIT_REGULAR,
        SPRAY_AND_WAIT_BINARY,
        //FIRST_CONTACT,
        //MAX_PROP,
        //PROPHET,
    };

    RoutingAlgorithmType routingAlgorithm;

    //message
    enum MessageType {
        HELLO = 0x00,
        REQUEST = 0x01,
        BUNDLE = 0x02
        //ACK
    };

    struct PacketHeader {
        uint32_t senderNodeId;
        uint16_t type;
        uint16_t length;

        PacketHeader(
            const NodeIdType initSenderNodeId,
            const MessageType initType,
            const size_t initLength)
            :
            senderNodeId(initSenderNodeId),
            type(static_cast<uint16_t>(initType)),
            length(static_cast<uint16_t>(initLength))
            { }

    };

    struct BundleHeader {
        uint64_t bundleId;
        uint32_t bundleSizeBytes;
        uint32_t targetNodeId;
        TimeType sendTime;
        TimeType expirationTime;
        uint32_t numOfCopies;

        BundleHeader() {}

        BundleHeader(
            const BundleIdType& initBundleId,
            const unsigned int initBundleSizeBytes,
            const NodeIdType& initTargetNodeId,
            const TimeType& initSendTime,
            const TimeType& initExpirationTime,
            const unsigned int initNumOfCopies)
            :
            bundleId(initBundleId),
            bundleSizeBytes(initBundleSizeBytes),
            targetNodeId(initTargetNodeId),
            sendTime(initSendTime),
            expirationTime(initExpirationTime),
            numOfCopies(initNumOfCopies)
            { }

    };

    struct BundleInfo {
        unsigned int remainingCopies;
        BundleHeader bundleHeader;

        BundleInfo(
            const unsigned int initRemainingCopies,
            const BundleHeader& initBundleHeader)
            :
            remainingCopies(initRemainingCopies),
            bundleHeader(initBundleHeader)
            { }

    };

    map<BundleIdType, BundleInfo> storedBundles;
    unsigned long long int currentStrageUsageBytes;

    map<BundleIdType, TimeType> requestedBundleIds;
    TimeType requestResendInterval;

    void SetNumberOfCopiesToZero(
        const BundleIdType& targetBundleId);

    void SubtractNumberOfCopies(
        const BundleIdType& targetBundleId);

    void ReadMessageEventFile(
        const string& messageEventFile,
        const int nodeIdOffset);

    //as receiver
    //-------------------------------------------------------------------------

    //UDP
    class UdpPacketHandler: public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        UdpPacketHandler(
            BundleProtocol* initAppPtr)
            :
            appPtr(initAppPtr) { }

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePort,
            const NetworkAddress& destinationAddress,
            const PacketPriorityType& priority)
        {
            appPtr->ReceiveUdpPacket(packetPtr, sourceAddress);
        }

    private:
        BundleProtocol* appPtr;

    };//PacketHandler//

    shared_ptr<UdpPacketHandler> udpPacketHandlerPtr;

    void ReceiveUdpPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceNetworkAddress);

   void ProcessHelloPacket(
        const Packet& packet,
        const unsigned int messageLength,
        const NetworkAddress& sourceNetworkAddress);

    void ProcessRequestPacket(
        const Packet& packet,
        const unsigned int messageLength,
        const NodeIdType senderNodeId,
        const NetworkAddress& sourceNetworkAddress);

    void ProcessBundlePacket(
        const Packet& packet,
        const unsigned int messageLength,
        const NetworkAddress& sourceNetworkAddress);

    void ProcessBundleData(
        const BundleHeader& bundleHeader,
        const NetworkAddress& sourceAddress);


    //as sender
    //-------------------------------------------------------------------------
    class SendHelloEvent: public SimulationEvent {
    public:
        explicit
        SendHelloEvent(
            BundleProtocol* initAppPtr)
            :
            appPtr(initAppPtr)
        {}

        void ExecuteEvent()
        {
            appPtr->SendHello();

        }//ExecuteEvent//

    private:
        BundleProtocol* appPtr;

    };//SendHelloEvent//

    class CreateBundleEvent: public SimulationEvent {
    public:
        explicit
        CreateBundleEvent(
            BundleProtocol* initAppPtr,
            const NodeIdType initTargetNodeId,
            const unsigned int& initBundleSizeBytes,
            const TimeType& initBundleEndTime,
            const TimeType& initBundleInterval)
            :
            appPtr(initAppPtr),
            targetNodeId(initTargetNodeId),
            bundleSizeBytes(initBundleSizeBytes),
            bundleEndTime(initBundleEndTime),
            bundleInterval(initBundleInterval)
        {}

        void ExecuteEvent()
        {
            appPtr->CreateBundle(targetNodeId, bundleSizeBytes, bundleEndTime, bundleInterval);

        }//ExecuteEvent//

    private:
        BundleProtocol* appPtr;
        NodeIdType targetNodeId;
        unsigned int bundleSizeBytes;
        TimeType bundleEndTime;
        TimeType bundleInterval;

    };//CreateBundleEvent//

    shared_ptr<SendHelloEvent> sendHelloEventPtr;
    EventRescheduleTicket sendHelloEventTicket;
    TimeType helloInterval;
    TimeType helloStartTimeJitter;

    unsigned int currentSequenceNumber;

    BundleIdType GenerateNewBundleId();

    NodeIdType ExtractOriginatorNodeId(
        const BundleIdType& bundleId) const;

    unsigned int ExtractOriginatorSequenceNumber(
        const BundleIdType& bundleId) const;

    void IncrementStorageUsage(
        const unsigned int dataSizeBytes);

    void DecrementStorageUsage(
        const unsigned int dataSizeBytes);

    void StoreBundle(
        const BundleIdType& bundleId,
        const BundleInfo& bundle,
        bool& success);

    void CreateBundle(
        const NodeIdType targetNodeId,
        const unsigned int bundleSizeBytes,
        const TimeType& bundleEndTime,
        const TimeType& bundleInterval);

    void DiscardExpiredBundles();

    void SendHello();

    void SendRequest(
        const set<BundleIdType>& newBundleIds,
        const NetworkAddress& targetNetworkAddress);

    void SendBundleWithUdp(
        const vector<BundleIdType>& bundleIds,
        const NetworkAddress& destinationNetworkAddress);


    //TCP
    class TcpConnectionHandler: public ConnectionFromTcpProtocolHandler {
    public:
        TcpConnectionHandler(
            BundleProtocol* initAppPtr)
            :
            appPtr(initAppPtr)
        { }

        void HandleNewConnection(const shared_ptr<TcpConnection>& newTcpConnectionPtr)
        {
            appPtr->AcceptTcpConnection(newTcpConnectionPtr);
        }//HandleNewConnection//

    private:
        BundleProtocol* appPtr;

    };//TcpConnectionHandler//

    class TcpEventHandler: public TcpConnection::AppTcpEventHandler {
    public:
        TcpEventHandler(
            BundleProtocol* initAppPtr)
            :
            appPtr(initAppPtr)
        { }

        void DoTcpIsReadyForMoreDataAction()
        {
           appPtr->DoTcpIsReadyForMoreDataAction(this);

        }//DoTcpIsReadyForMoreDataAction//

        void ReceiveDataBlock(
            const unsigned char dataBlock[],
            const unsigned int dataLength,
            const unsigned int actualDataLength,
            bool& stallIncomingDataFlow)
        {
            appPtr->ReceiveDataBlock(this, dataBlock, dataLength, actualDataLength);
            stallIncomingDataFlow = false;

        }//ReceiveDataBlock//

        void DoTcpRemoteHostClosedAction()
        {
            appPtr->DoTcpRemoteHostClosedAction(this);

        }//DoTcpRemoteHostClosedAction//

        void DoTcpLocalHostClosedAction()
        {
            appPtr->DoTcpLocalHostClosedAction(this);

        }//DoTcpLocalHostClosedAction//

    private:
        BundleProtocol* appPtr;

    };//TcpEventHandler//

    void AcceptTcpConnection(
        const shared_ptr<TcpConnection>& connectionPtr);
    shared_ptr<TcpConnectionHandler> tcpConnectionHandlerPtr;

    enum TcpStateType {
        CLOSE,
        WAITING_FOR_ESTABLISHING,
        ESTABLISHED,
        WAITING_FOR_CLOSING
    };//TcpStateType//

    struct TcpConnectionInfoType {
        shared_ptr<TcpEventHandler> tcpEventHandlerPtr;
        shared_ptr<TcpConnection> tcpConnectionPtr;
        TcpStateType tcpState;
        NetworkAddress destinationAddress;
        size_t sendingBundleIndex;
        vector<BundleIdType> sendingBundleIds;
        vector<unsigned int> sendingBundleSizeBytes;
        vector<shared_ptr<vector<unsigned char> > > sendingData;
        unsigned long long int tcpAccumulatedDeliveredBytes;
        unsigned long long int totalSendingDataBytes;
        unsigned long long int bundleSizeBytesToBeReceived;
        unsigned long long int receivedDataBytes;
        vector<unsigned char> receivedData;
        BundleHeader receivingBundleHeader;

        TcpConnectionInfoType()
            :
            tcpEventHandlerPtr(),
            tcpConnectionPtr(),
            tcpState(CLOSE),
            destinationAddress(),
            sendingBundleIndex(0),
            tcpAccumulatedDeliveredBytes(0),
            totalSendingDataBytes(0),
            receivedDataBytes(0),
            bundleSizeBytesToBeReceived(0),
            receivedData()
        {}
    };//TcpConnectionInfoType//

    list<TcpConnectionInfoType> tcpConnectionDatabase;

    void AddTcpConnectionInfo(
        const TcpConnectionInfoType& tcpConnectionInfo);

    void DeleteTcpConnectionInfo(
        const TcpEventHandler* tcpEventHandlerPtr);

    TcpConnectionInfoType* FindTcpConnectionInfo(
        const TcpEventHandler* tcpEventHandlerPtr);

    TcpConnectionInfoType& GetTcpConnectionInfo(
        const TcpEventHandler* tcpEventHandlerPtr);

    void UpdateTcpConnectionInfo(
        const TcpConnectionInfoType& tcpConnectionInfo);

    void RegisterBundleToTcpConnectionInfo(
        const vector<BundleIdType>& bundleIds,
        TcpConnectionInfoType& tcpConnectionInfo);

    void SendBundleWithTcp(
        const vector<BundleIdType>& bundleIds,
        const NetworkAddress& destinationNetworkAddress);


    void DoTcpIsReadyForMoreDataAction(
        const TcpEventHandler* tcpHandler);

    void SendDataBlock(
        const TcpEventHandler* tcpHandler);

    void ReceiveDataBlock(
        const TcpEventHandler* tcpHandler,
        const unsigned char dataBlock[],
        const unsigned int dataLength,
        const unsigned int actualDataLength);

    void DoTcpRemoteHostClosedAction(
        const TcpEventHandler* tcpHandler);

    void DoTcpLocalHostClosedAction(
        const TcpEventHandler* tcpHandler);

    //trace and stats

    //at originator or target node
    shared_ptr<CounterStatistic> bundlesGeneratedStatPtr;
    shared_ptr<CounterStatistic> bundleGenerationFailedDueToLackOfStorageStatPtr;
    shared_ptr<CounterStatistic> bytesGeneratedStatPtr;
    shared_ptr<CounterStatistic> bundlesDeliveredStatPtr;
    shared_ptr<CounterStatistic> bytesDeliveredStatPtr;
    shared_ptr<RealStatistic> bundleEndToEndDelayStatPtr;

    //at any nodes
    shared_ptr<CounterStatistic> bundlesSentStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> bundlesReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<CounterStatistic> duplicateBundleReceivedStatPtr;
    shared_ptr<CounterStatistic> bundlesDiscardedDueToLackOfStorageStatPtr;
    shared_ptr<RealStatistic> storageUsageBytesStatPtr;


    void OutputTraceAndStatForStorageUsage();

    void OutputTraceForControlPacketSend(
        const MessageType& messageType,
        const NetworkAddress& destinationAddress);

    void OutputTraceForCtrlPacketReceive(
        const MessageType& messageType,
        const NetworkAddress& sourceAddress);

    void OutputTraceAndStatsForGenerateBundle(
        const BundleIdType& bundleId,
        const NodeIdType& targetNodeId,
        const size_t& bundleSizeBytes);

    void OutputTraceAndStatsForDeliveryBundle(
        const BundleIdType& bundleId,
        const NodeIdType& targetNodeId,
        const size_t& bundleSizeBytes,
        const TimeType& delay);

    void OutputTraceAndStatsForSendBundle(
        const NetworkAddress& nextNodeAddress,
        const BundleIdType& bundleId,
        const NodeIdType& targetNodeId,
        const size_t& bundleSizeBytes);

    void OutputTraceAndStatsForReceiveBundle(
        const NetworkAddress& lastNodeAddress,
        const BundleIdType& bundleId,
        const NodeIdType& targetNodeId,
        const size_t& bundleSizeBytes);

};


inline
BundleProtocol::BundleProtocol(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeIdType& initNodeId,
    const unsigned short int initDefaultDestinationPortId)
    :
    Application(initSimulationEngineInterfacePtr, modelName),
    nodeId(initNodeId),
    helloInterval(INFINITE_TIME),
    helloStartTimeJitter(ZERO_TIME),
    controlPriority(0),
    messagePriority(0),
    destinationPortNumber(initDefaultDestinationPortId),
    nextAvailableLocalPortNumber(0),
    maxStorageSizeBytes(ULLONG_MAX),
    currentStrageUsageBytes(0),
    bundleLifeTime(INFINITE_TIME),
    maximumCopies(UINT_MAX),
    transportMode(TCP),
    routingAlgorithm(EPIDEMIC),
    currentSequenceNumber(0),
    requestResendInterval(2 * SECOND),
    useVirtualPayload(false),
    bundlesGeneratedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BundlesGenerated"))),
    bundleGenerationFailedDueToLackOfStorageStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BundleGenerationFailedDueToLackOfStorage"))),
    bytesGeneratedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BytesGenerated"))),
    bundlesDeliveredStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BundlesDelivered"))),
    bytesDeliveredStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BytesDelivered"))),
    bundleEndToEndDelayStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_BundleEndToEndDelay"))),
    bundlesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BundlesSent"))),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BytesSent"))),
    bundlesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BundlesReceived"))),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BytesReceived"))),
    duplicateBundleReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_DuplicateBundleReceived"))),
    bundlesDiscardedDueToLackOfStorageStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_BundlesDiscardedDueToLackOfStorage"))),
    storageUsageBytesStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_StorageUsageBytes")))
{

    nextAvailableLocalPortNumber = destinationPortNumber + 1;

    if (theParameterDatabaseReader.ParameterExists("bundle-max-storage-size-bytes", nodeId)) {
        maxStorageSizeBytes =
            theParameterDatabaseReader.ReadBigInt("bundle-max-storage-size-bytes", nodeId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("bundle-use-virtual-payload", nodeId)) {
        useVirtualPayload =
            theParameterDatabaseReader.ReadBool("bundle-use-virtual-payload", nodeId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("bundle-transport-mode", nodeId)) {

        const string transportModeString =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("bundle-transport-mode", nodeId));

        if (transportModeString == "tcp") {
            transportMode = TCP;
        }
        else if (transportModeString == "udp") {
            transportMode = UDP;
        }
        else {
            cerr << "Error: bundle-transport-mode = " << transportModeString << " is unknown." << endl;
            exit(1);
        }//if//

    }//if//

    const string routingAlgorithmString =
        MakeLowerCaseString(theParameterDatabaseReader.ReadString("bundle-routing-algorithm", nodeId));

    if (routingAlgorithmString == "epidemic") {
        routingAlgorithm = EPIDEMIC;
    }
    else if (routingAlgorithmString == "direct-delivery") {
        routingAlgorithm = DIRECT_DELIVERY;
    }
    else if (routingAlgorithmString == "spray-and-wait") {

        bool sprayAndWaitBinaryMode = false;
        if (theParameterDatabaseReader.ParameterExists(
                "bundle-spray-and-wait-binary-mode", nodeId)) {

            sprayAndWaitBinaryMode =
                theParameterDatabaseReader.ReadBool(
                    "bundle-spray-and-wait-binary-mode", nodeId);

        }//if//

        if (sprayAndWaitBinaryMode) {
            routingAlgorithm = SPRAY_AND_WAIT_BINARY;
        }
        else {
            routingAlgorithm = SPRAY_AND_WAIT_REGULAR;
        }//if//

    }
    else {
        cerr << "Error: bundle-routing-algorithm = " << routingAlgorithmString << " is unknown." << endl;
        exit(1);
    }//if//


    switch (routingAlgorithm) {
    case EPIDEMIC:
        maximumCopies = UINT_MAX;
        break;
    case DIRECT_DELIVERY:
        maximumCopies = 1;
        break;
    case SPRAY_AND_WAIT_REGULAR:
    case SPRAY_AND_WAIT_BINARY:

        maximumCopies =
            theParameterDatabaseReader.ReadNonNegativeInt("bundle-maximum-number-of-copies", nodeId);

        if (maximumCopies < 1) {
            cerr << "Error: bundle-maximum-number-of-copies must be greater than or equal to 1: " << maximumCopies << endl;
            exit(1);
        }//if//

        break;
    default:
        assert(false);exit(1);
    }//switch//


    helloInterval =
        theParameterDatabaseReader.ReadTime("bundle-hello-interval", nodeId);

    if (theParameterDatabaseReader.ParameterExists(
            "bundle-hello-max-jitter", nodeId)) {

        helloStartTimeJitter =
            theParameterDatabaseReader.ReadTime(
                "bundle-hello-max-jitter", nodeId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            "bundle-request-resend-interval", nodeId)) {

        requestResendInterval =
            theParameterDatabaseReader.ReadTime(
                "bundle-request-resend-interval", nodeId);

    }//if//


    if (theParameterDatabaseReader.ParameterExists("bundle-control-packet-priority", nodeId, applicationId)) {
        controlPriority = static_cast<PacketPriorityType>(
            theParameterDatabaseReader.ReadNonNegativeInt("bundle-control-packet-priority", nodeId, applicationId));
    }//if//


}//BundleProtocol//


inline
BundleProtocol::~BundleProtocol()
{


}


inline
void BundleProtocol::ReadMessageEventFile(
    const string& messageEventFile,
    const int nodeIdOffset)
{
    ifstream inFile(messageEventFile.c_str());

    if (!inFile.good()) {
        cerr << "Error: Could not open message event file: " << messageEventFile << endl;
        exit(1);
    }//if//

    while(!inFile.eof()) {

        string aLine;
        getline(inFile, aLine);

        if (IsAConfigFileCommentLine(aLine)) continue;

        DeleteTrailingSpaces(aLine);
        assert(HasNoTrailingSpaces(aLine));

        istringstream lineStream(aLine);

        string eventTimeString;
        lineStream >> eventTimeString;

        bool success;
        TimeType  eventTime;
        ConvertStringToTime(eventTimeString, eventTime, success);

        if (!success) {
            cerr << "Error: message event file: " << aLine << endl;
            exit(1);
        }//if//

        string messageType;
        lineStream >> messageType;

        if (messageType != "C") {
            cerr << "Error: support only message type \"C\": " << messageType << endl;
            exit(1);

        }//if//

        //do nothing
        string messageName;
        lineStream >> messageName;

        unsigned int sourceNodeId;
        lineStream >> sourceNodeId;

        sourceNodeId += nodeIdOffset;

        //for others
        if (sourceNodeId != nodeId) continue;

        int destinationNodeId;
        lineStream >> destinationNodeId;

        destinationNodeId += nodeIdOffset;

        int messageSizeBytes;
        lineStream >> messageSizeBytes;

        //schedule event
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new CreateBundleEvent(this, destinationNodeId, messageSizeBytes, eventTime, INFINITE_TIME)), eventTime);

    }//while//

}//ReadMessageEventFile//


inline
void BundleProtocol::CompleteInitialization()
{

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    TimeType helloStartTime = currentTime + helloInterval;

    if (helloStartTimeJitter !=  ZERO_TIME) {
        helloStartTime += static_cast<TimeType>(
            aRandomNumberGeneratorPtr->GenerateRandomDouble() * helloStartTimeJitter);
    }//if//

    sendHelloEventPtr = shared_ptr<SendHelloEvent>(new SendHelloEvent(this));

    simulationEngineInterfacePtr->ScheduleEvent(sendHelloEventPtr, helloStartTime, sendHelloEventTicket);


    //as receiver
    udpPacketHandlerPtr = shared_ptr<UdpPacketHandler>(new UdpPacketHandler(this));

    assert(transportLayerPtr->udpPtr->PortIsAvailable(destinationPortNumber));

    transportLayerPtr->udpPtr->OpenSpecificUdpPort(
        NetworkAddress::anyAddress,
        destinationPortNumber,
        udpPacketHandlerPtr);

    tcpConnectionHandlerPtr =
        shared_ptr<TcpConnectionHandler>(new TcpConnectionHandler(this));

    assert(transportLayerPtr->tcpPtr->PortIsAvailable(destinationPortNumber));

    transportLayerPtr->tcpPtr->OpenSpecificTcpPort(
        NetworkAddress::anyAddress,
        destinationPortNumber,
        tcpConnectionHandlerPtr);

    OutputTraceAndStatForStorageUsage();

}//CompleteInitialization//

inline
void BundleProtocol::AddSenderSetting(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationIdType& applicationId,
    const NodeIdType& targetNodeId)
{

    if (theParameterDatabaseReader.ParameterExists("bundle-message-lifetime", nodeId, applicationId)) {
        bundleLifeTime =
            theParameterDatabaseReader.ReadTime("bundle-message-lifetime", nodeId, applicationId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("bundle-message-priority", nodeId, applicationId)) {
        messagePriority = static_cast<PacketPriorityType>(
            theParameterDatabaseReader.ReadNonNegativeInt("bundle-message-priority", nodeId, applicationId));
    }//if//


    TimeType messageSendTimeJitter = ZERO_TIME;
    if (theParameterDatabaseReader.ParameterExists(
        "bundle-message-max-jitter", nodeId, applicationId)) {

        const TimeType messageSendTimeMaxJitter =
            theParameterDatabaseReader.ReadTime(
                "bundle-message-max-jitter", nodeId, applicationId);

        messageSendTimeJitter =
            static_cast<TimeType>(aRandomNumberGeneratorPtr->GenerateRandomDouble() * messageSendTimeMaxJitter);

    }//if//

    TimeType messageSendTime =
        theParameterDatabaseReader.ReadTime(
            "bundle-message-start-time", nodeId, applicationId);

    messageSendTime += messageSendTimeJitter;

    const TimeType messageEndTime =
        theParameterDatabaseReader.ReadTime(
            "bundle-message-end-time", nodeId, applicationId);

    const TimeType messageSendInterval =
        theParameterDatabaseReader.ReadTime(
            "bundle-message-send-interval", nodeId, applicationId);

    const unsigned int messageSizeBytes =
        theParameterDatabaseReader.ReadNonNegativeInt(
            "bundle-message-size-bytes", nodeId, applicationId);

    if (messageSizeBytes < sizeof(BundleHeader)) {

        cerr << "'Error: bundle-message-size-bytes = " << messageSizeBytes
            << " should be larger than " << sizeof(BundleHeader) << endl;
        exit(1);

    }//if//

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (currentTime > messageSendTime) {
        const size_t nextTransmissionTime = size_t(ceil(double(currentTime - messageSendTime) / messageSendInterval));
        messageSendTime += nextTransmissionTime * messageSendInterval;
    }//if//

    if (messageSendTime < messageEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new CreateBundleEvent(this, targetNodeId, messageSizeBytes, messageEndTime, messageSendInterval)), messageSendTime);
    }//if//

    //Future
    //if (theParameterDatabaseReader.ParameterExists(
    //        "bundle-message-event-file", nodeId, applicationId)) {

    //    const string ReadApplicationFile =
    //        theParameterDatabaseReader.ReadString(
    //            "bundle-message-event-file", nodeId, applicationId);

    //    int nodeIdOffset = 0;
    //    if (theParameterDatabaseReader.ParameterExists(
    //            "bundle-node-id-offset-in-message-file", nodeId, applicationId)) {

    //        nodeIdOffset =
    //            theParameterDatabaseReader.ReadInt("bundle-node-id-offset-in-message-file", nodeId, applicationId);

    //    }//if//

    //    ReadMessageEventFile(ReadApplicationFile, nodeIdOffset);
    //
    //}//if//

}//AddSenderSetting//



inline
BundleIdType BundleProtocol::GenerateNewBundleId()
{
    currentSequenceNumber++;

    assert(currentSequenceNumber < UINT_MAX);

    BundleIdType nodeIdPart = nodeId;
    nodeIdPart = nodeIdPart << 32;
    return (nodeIdPart + currentSequenceNumber);

}//GenerateNewBundleId//


inline
NodeIdType BundleProtocol::ExtractOriginatorNodeId(
    const BundleIdType& bundleId) const
{
    return static_cast<NodeIdType>(bundleId / UINT_MAX);

}//ExtractOriginatorNodeId//


inline
unsigned int BundleProtocol::ExtractOriginatorSequenceNumber(
    const BundleIdType& bundleId) const
{

    return static_cast<unsigned int>(bundleId);

}//ExtractSequcenceNumber//



inline
void BundleProtocol::IncrementStorageUsage(
    const unsigned int dataSizeBytes)
{

    assert((*this).currentStrageUsageBytes <= (ULLONG_MAX - dataSizeBytes));

    (*this).currentStrageUsageBytes += dataSizeBytes;

    OutputTraceAndStatForStorageUsage();

}//IncrementStorageUsage//


inline
void BundleProtocol::DecrementStorageUsage(
    const unsigned int dataSizeBytes)
{

    assert((*this).currentStrageUsageBytes >= dataSizeBytes);

    (*this).currentStrageUsageBytes -= dataSizeBytes;

    OutputTraceAndStatForStorageUsage();

}//DecrementStorageUsage//


inline
void BundleProtocol::StoreBundle(
    const BundleIdType& bundleId,
    const BundleInfo& bundle,
    bool& success)
{

    const unsigned int dataSizeBytes = static_cast<unsigned int>(bundle.bundleHeader.bundleSizeBytes);

    //discard expired bundles before storing bundle
    DiscardExpiredBundles();

    //check storage space
    if (((*this).currentStrageUsageBytes + dataSizeBytes) > (*this).maxStorageSizeBytes) {
        success = false;
        return;
    }//if//

    storedBundles.insert(make_pair(bundleId, bundle));

    //update strage usage
    IncrementStorageUsage(dataSizeBytes);

    success = true;

}//StoreBundle//


inline
void BundleProtocol::CreateBundle(
    const NodeIdType targetNodeId,
    const unsigned int bundleSizeBytes,
    const TimeType& bundleEndTime,
    const TimeType& bundleInterval)
{

    const BundleIdType newBundleId = (*this).GenerateNewBundleId();

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    TimeType bundleExpirationTime;
    if (bundleLifeTime > (INFINITE_TIME - currentTime)) {
        bundleExpirationTime = INFINITE_TIME;
    }
    else {
        bundleExpirationTime = currentTime + bundleLifeTime;
    }//if//

    const BundleHeader bundleHeader(newBundleId, bundleSizeBytes, targetNodeId, currentTime, bundleExpirationTime, maximumCopies);

    const BundleInfo bundle(maximumCopies, bundleHeader);

    bool success = false;
    StoreBundle(newBundleId, bundle, success);

    if (success) {
        (*this).OutputTraceAndStatsForGenerateBundle(
            newBundleId, targetNodeId, bundleSizeBytes);
    }
    else {
        bundleGenerationFailedDueToLackOfStorageStatPtr->IncrementCounter();
    }//if//

    //schedule next event
    if (bundleInterval <= (INFINITE_TIME - currentTime)) {

        const TimeType nextEventTime = currentTime + bundleInterval;
        if (nextEventTime < bundleEndTime) {
            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(new CreateBundleEvent(this, targetNodeId, bundleSizeBytes, bundleEndTime, bundleInterval)), nextEventTime);
        }
    }//if//

}//CreateBundle//


inline
void BundleProtocol::DiscardExpiredBundles()
{

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    typedef map<BundleIdType, BundleInfo>::iterator IterType;

    IterType iter = storedBundles.begin();

    while (iter != storedBundles.end()) {

        const BundleInfo& bundle = (iter->second);

        if (bundle.bundleHeader.expirationTime <= currentTime) {

            //update strage usage
            DecrementStorageUsage(
                static_cast<unsigned int>((iter->second).bundleHeader.bundleSizeBytes));

            IterType deleteIter = iter;
            ++iter;
            storedBundles.erase(deleteIter);

        }
        else {
            ++iter;
        }//if//

    }//while//

}//DiscardExpiredBundles//


inline
void BundleProtocol::SendHello()
{
    sendHelloEventTicket.Clear();

    //discard expired bundles before sending hello with bundle info
    DiscardExpiredBundles();

    vector<unsigned char> payload;

    map<BundleIdType, BundleInfo>::const_iterator iter;
    size_t bundleIndex = 0;
    for(iter = storedBundles.begin(); iter != storedBundles.end(); ++iter) {

        const BundleIdType& bundleId = (iter->first);
        const BundleInfo& bundleInfo = (iter->second);

        //check remaining copies:
        //0: cannot transfer, 1: can transfer to final destination, 2: can transfer to anytime
        if (bundleInfo.remainingCopies < 1) continue;

        payload.resize(sizeof(BundleIdType) * (bundleIndex + 1));
        memcpy(&payload[sizeof(BundleIdType) * bundleIndex], &bundleId, sizeof(BundleIdType));
        bundleIndex++;

    }//for//

    if (payload.size() > USHRT_MAX) {
        cerr << "Too big hello message: " << payload.size() << endl;
        exit(1);
    }//if//

    PacketHeader packetHeader(nodeId, HELLO, payload.size());

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, payload);

    packetPtr->AddPlainStructHeader(packetHeader);

    OutputTraceForControlPacketSend(
        HELLO,
        NetworkAddress::broadcastAddress);

    transportLayerPtr->udpPtr->SendPacket(
        packetPtr, 0, NetworkAddress::broadcastAddress, destinationPortNumber, controlPriority);

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    simulationEngineInterfacePtr->ScheduleEvent(
        sendHelloEventPtr, currentTime + helloInterval, sendHelloEventTicket);

}//SendHello//


inline
void BundleProtocol::SendRequest(
    const set<BundleIdType>& newBundleIds,
    const NetworkAddress& targetNetworkAddress)
{

    vector<unsigned char> payload;
    payload.resize(newBundleIds.size() * sizeof(BundleIdType));

    set<BundleIdType>::const_iterator iter;
    size_t bundleIndex = 0;
    for(iter = newBundleIds.begin(); iter != newBundleIds.end(); ++iter) {

        memcpy(&payload[sizeof(BundleIdType) * bundleIndex], &(*iter), sizeof(BundleIdType));
        bundleIndex++;

    }//for//

    if (payload.size() > USHRT_MAX) {
        cerr << "Too big request message: " << payload.size() << endl;
        exit(1);
    }//if//

    PacketHeader packetHeader(nodeId, REQUEST, payload.size());

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(*simulationEngineInterfacePtr, payload);

    packetPtr->AddPlainStructHeader(packetHeader);

    OutputTraceForControlPacketSend(
        REQUEST,
        targetNetworkAddress);

    transportLayerPtr->udpPtr->SendPacket(
        packetPtr, 0, targetNetworkAddress, destinationPortNumber, controlPriority);

}//SendRequest//


inline
void BundleProtocol::SetNumberOfCopiesToZero(
    const BundleIdType& targetBundleId)
{
    BundleInfo& bundleInfo = storedBundles.at(targetBundleId);

    bundleInfo.remainingCopies = 0;

}//SubtractNumberOfCopies//


inline
void BundleProtocol::SubtractNumberOfCopies(
    const BundleIdType& targetBundleId)
{
    BundleInfo& bundleInfo = storedBundles.at(targetBundleId);

    assert(bundleInfo.remainingCopies >= 1);

    unsigned int passCopies;

    switch (routingAlgorithm) {
    case EPIDEMIC:
    case DIRECT_DELIVERY:
        //do nothing;
        return;
        break;
    case SPRAY_AND_WAIT_REGULAR:
    {
        passCopies = 1;
        bundleInfo.remainingCopies--;
        break;
    }
    case SPRAY_AND_WAIT_BINARY:
    {
        passCopies = static_cast<unsigned int>(std::ceil(double(bundleInfo.remainingCopies) / 2));
        bundleInfo.remainingCopies /= 2;
        break;
    }
    default:
        assert(false);
        exit(1);
    }//switch//

    //update Bundle header
    bundleInfo.bundleHeader.numOfCopies = passCopies;

}//SubtractNumberOfCopies//


inline
void BundleProtocol::SendBundleWithUdp(
    const vector<BundleIdType>& bundleIds,
    const NetworkAddress& destinationNetworkAddress)
{

    assert(transportMode == UDP);

    for(size_t i = 0; i < bundleIds.size(); i++) {

        const BundleIdType bundleId = bundleIds[i];

        //already deleted
        if (storedBundles.find(bundleId) == storedBundles.end()) {
            continue;
        }//if//

        //subtract num of copies
        SubtractNumberOfCopies(bundleId);

        const BundleInfo& bundleInfo = storedBundles.at(bundleId);

        if (bundleInfo.bundleHeader.bundleSizeBytes > USHRT_MAX) {
            cerr << "Too big bundle message for UDP packet: " << bundleInfo.bundleHeader.bundleSizeBytes << endl;
            exit(1);
        }//if//

        PacketHeader packetHeader(nodeId, BUNDLE, bundleInfo.bundleHeader.bundleSizeBytes);

        unique_ptr<Packet> packetPtr =
            Packet::CreatePacket(*simulationEngineInterfacePtr, bundleInfo.bundleHeader,
            bundleInfo.bundleHeader.bundleSizeBytes, useVirtualPayload);

        OutputTraceAndStatsForSendBundle(
            destinationNetworkAddress, bundleId, bundleInfo.bundleHeader.targetNodeId, (*packetPtr).LengthBytes());

        packetPtr->AddPlainStructHeader(packetHeader);

        transportLayerPtr->udpPtr->SendPacket(
            packetPtr, 0, destinationNetworkAddress, destinationPortNumber, messagePriority);

    }//for//

}//SendBundleWithUdp//


inline
void BundleProtocol::AddTcpConnectionInfo(
    const TcpConnectionInfoType& tcpConnectionInfo)
{
    tcpConnectionDatabase.push_back(tcpConnectionInfo);

}//AddTcpConnectionInfo//


inline
void BundleProtocol::DeleteTcpConnectionInfo(
    const TcpEventHandler* tcpEventHandlerPtr)
{
    list<TcpConnectionInfoType>::iterator iter = tcpConnectionDatabase.begin();

    while (iter != tcpConnectionDatabase.end()) {
        TcpConnectionInfoType& tcpConnectionInfo = *iter;
        if (tcpConnectionInfo.tcpEventHandlerPtr.get() == tcpEventHandlerPtr) {
            tcpConnectionDatabase.erase(iter);
            break;
        }//if//
        ++iter;
    }//while//

}//DeleteTcpConnectionInfo//


inline
BundleProtocol::TcpConnectionInfoType* BundleProtocol::FindTcpConnectionInfo(
    const TcpEventHandler* tcpEventHandlerPtr)
{
    list<TcpConnectionInfoType>::iterator iter = tcpConnectionDatabase.begin();

    while (iter != tcpConnectionDatabase.end()) {
        TcpConnectionInfoType& tcpConnectionInfo = *iter;
        if (tcpConnectionInfo.tcpEventHandlerPtr.get() == tcpEventHandlerPtr) {
            return &tcpConnectionInfo;
        }//if//
        ++iter;
    }//while//

    return nullptr;

}//FindTcpConnectionInfo//


inline
BundleProtocol::TcpConnectionInfoType& BundleProtocol::GetTcpConnectionInfo(
    const TcpEventHandler* tcpEventHandlerPtr)
{
    TcpConnectionInfoType* tcpConnectionInfoPtr =
        FindTcpConnectionInfo(tcpEventHandlerPtr);

    assert(tcpConnectionInfoPtr != nullptr);

    return *tcpConnectionInfoPtr;

}//GetTcpConnectionInfo//


inline
void BundleProtocol::UpdateTcpConnectionInfo(
    const TcpConnectionInfoType& tcpConnectionInfo)
{
    TcpConnectionInfoType* tcpConnectionInfoPtr =
        FindTcpConnectionInfo(tcpConnectionInfo.tcpEventHandlerPtr.get());

    if (tcpConnectionInfoPtr != nullptr) {
        *tcpConnectionInfoPtr = tcpConnectionInfo;
    }
    else {
        AddTcpConnectionInfo(tcpConnectionInfo);
    }//if//

}//UpdateTcpConnectionInfo//


inline
void BundleProtocol::RegisterBundleToTcpConnectionInfo(
    const vector<BundleIdType>& bundleIds,
    TcpConnectionInfoType& tcpConnectionInfo)
{

    for(size_t i = 0; i < bundleIds.size(); i++) {

        const BundleIdType bundleId = bundleIds[i];

        //already deleated
        if (storedBundles.find(bundleId) == storedBundles.end()) {
            continue;
        }//if//

        //subtract num of copies
        SubtractNumberOfCopies(bundleId);

        const BundleInfo& bundleInfo = storedBundles.at(bundleId);

        const unsigned int bundleSizeBytes = bundleInfo.bundleHeader.bundleSizeBytes;
        const unsigned int bundleHeaderSizeBytes = sizeof(bundleInfo.bundleHeader);

        assert(sizeof(bundleInfo.bundleHeader) == sizeof(BundleHeader));

        //copy info
        tcpConnectionInfo.sendingBundleIds.push_back(bundleId);
        tcpConnectionInfo.sendingBundleSizeBytes.push_back(bundleSizeBytes);

        shared_ptr<vector<unsigned char> > bundleDataPtr(new vector<unsigned char>(bundleHeaderSizeBytes));
        memcpy(&(*bundleDataPtr)[0], &(bundleInfo.bundleHeader), sizeof(BundleHeader));

        tcpConnectionInfo.sendingData.push_back(bundleDataPtr);
        tcpConnectionInfo.totalSendingDataBytes += bundleSizeBytes;

        OutputTraceAndStatsForSendBundle(
            tcpConnectionInfo.destinationAddress,
            bundleId, bundleInfo.bundleHeader.targetNodeId, bundleSizeBytes);

    }//for//

}//RegisterBundleToTcpConnectionInfo//


inline
void BundleProtocol::SendBundleWithTcp(
    const vector<BundleIdType>& bundleIds,
    const NetworkAddress& destinationNetworkAddress)
{

    assert(transportMode == TCP);
    assert(destinationNetworkAddress != NetworkAddress::invalidAddress);

    const TcpEventHandler* tcpHandler = nullptr;

    list<TcpConnectionInfoType>::iterator iter;
    for (iter = tcpConnectionDatabase.begin(); iter != tcpConnectionDatabase.end(); ++iter) {

        if (iter->destinationAddress == destinationNetworkAddress) {

            tcpHandler = iter->tcpEventHandlerPtr.get();

            break;
        }//if//

    }//for//

    if (tcpHandler == nullptr) {

        //new connection

        TcpConnectionInfoType tcpConnectionInfo;

        tcpConnectionInfo.tcpEventHandlerPtr =
            shared_ptr<TcpEventHandler>(new TcpEventHandler(this));

        const NetworkAddress sourceNetworkAddress =
            networkAddressLookupInterfacePtr->LookupNetworkAddress(nodeId);

        transportLayerPtr->tcpPtr->CreateOutgoingTcpConnection(
            sourceNetworkAddress, nextAvailableLocalPortNumber, destinationNetworkAddress, destinationPortNumber, messagePriority,
            tcpConnectionInfo.tcpEventHandlerPtr, tcpConnectionInfo.tcpConnectionPtr);

        if (useVirtualPayload) {
            tcpConnectionInfo.tcpConnectionPtr->EnableVirtualPayload();
        }//if//

        nextAvailableLocalPortNumber++;

        tcpConnectionInfo.tcpState = WAITING_FOR_ESTABLISHING;
        tcpConnectionInfo.destinationAddress = destinationNetworkAddress;

        RegisterBundleToTcpConnectionInfo(bundleIds, tcpConnectionInfo);

        UpdateTcpConnectionInfo(tcpConnectionInfo);

    }
    else {

        TcpConnectionInfoType& tcpConnectionInfo = GetTcpConnectionInfo(tcpHandler);

        if (tcpConnectionInfo.totalSendingDataBytes != 0) {
            //now transferring other bundles
            return;
        }//if

        assert(tcpConnectionInfo.totalSendingDataBytes == 0);
        assert(tcpConnectionInfo.sendingBundleIds.empty());

        RegisterBundleToTcpConnectionInfo(bundleIds, tcpConnectionInfo);

        if (tcpConnectionInfo.tcpState == ESTABLISHED) {
            //send data
            SendDataBlock(tcpHandler);
        }//if//

    }//if//

}//SendBundleWithTcp//


inline
void BundleProtocol::ReceiveUdpPacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& sourceNetworkAddress)
{

    int offsetBytes = 0;
    while (offsetBytes < (int)packetPtr->LengthBytes()) {

        const PacketHeader& packetHeader =
            packetPtr->GetAndReinterpretPayloadData<PacketHeader>(offsetBytes);

        switch (packetHeader.type) {
        case HELLO:

            OutputTraceForCtrlPacketReceive(HELLO, sourceNetworkAddress);

            packetPtr->DeleteHeader(sizeof(PacketHeader));
            ProcessHelloPacket((*packetPtr), packetHeader.length, sourceNetworkAddress);

            break;
        case REQUEST:

            OutputTraceForCtrlPacketReceive(REQUEST, sourceNetworkAddress);

            packetPtr->DeleteHeader(sizeof(PacketHeader));
            ProcessRequestPacket((*packetPtr), packetHeader.length, packetHeader.senderNodeId, sourceNetworkAddress);

            break;
        case BUNDLE:

            packetPtr->DeleteHeader(sizeof(PacketHeader));
            ProcessBundlePacket((*packetPtr), packetHeader.length, sourceNetworkAddress);

            break;
        default:
            assert(false);
            exit(1);
        }//switch//

        offsetBytes += sizeof(PacketHeader) + packetHeader.length;

    }//while//

    packetPtr = nullptr;

}//ReceiveUdpPacket//


inline
void BundleProtocol::ProcessHelloPacket(
    const Packet& packet,
    const unsigned int messageLength,
    const NetworkAddress& sourceNetworkAddress)
{

    assert((messageLength % sizeof(BundleIdType)) == 0);
    const size_t numberBundleIds = messageLength / sizeof(BundleIdType);

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    set<BundleIdType> newBundleIds;
    int offsetBytes = 0;
    for(size_t i = 0; i < numberBundleIds; i++) {

        const BundleIdType bundleId =
            packet.GetAndReinterpretPayloadData<BundleIdType>(offsetBytes);

        if (requestedBundleIds.find(bundleId) != requestedBundleIds.end()) {

            //check requested bundles expiration
            if ((requestedBundleIds.at(bundleId) + requestResendInterval) < currentTime) {

                requestedBundleIds.erase(bundleId);

            }//if//

        }//if//

        if ((ExtractOriginatorNodeId(bundleId) != nodeId) &&
            (storedBundles.find(bundleId) == storedBundles.end()) &&
            (requestedBundleIds.find(bundleId) == requestedBundleIds.end())) {

            //brand new bundle
            newBundleIds.insert(bundleId);
            requestedBundleIds[bundleId] = currentTime;

        }//if//

        offsetBytes += sizeof(BundleIdType);

    }//for//


    if(!newBundleIds.empty()) {

        //send request bundle
        SendRequest(
            newBundleIds,
            sourceNetworkAddress);

    }//if//

}//ProcessHelloPacket//


inline
void BundleProtocol::ProcessRequestPacket(
    const Packet& packet,
    const unsigned int messageLength,
    const NodeIdType senderNodeId,
    const NetworkAddress& sourceNetworkAddress)
{

    assert((messageLength % sizeof(BundleIdType)) == 0);
    const size_t numberBundleIds = messageLength / sizeof(BundleIdType);

    int offsetBytes = 0;
    vector<BundleIdType> requestedBundleIds;
    for(size_t i = 0; i < numberBundleIds; i++) {

        const BundleIdType bundleId =
            packet.GetAndReinterpretPayloadData<BundleIdType>(offsetBytes);

        map<BundleIdType, BundleInfo>::iterator iter =
            storedBundles.find(bundleId);

        //already deleted
        if (iter == storedBundles.end()) {
            offsetBytes += sizeof(BundleIdType);
            continue;
        }//if//

        const BundleInfo& bundleInfo = (iter->second);

        //check number of remaining copies
        if ((bundleInfo.remainingCopies >= 2) ||
            ((bundleInfo.remainingCopies == 1) && ((bundleInfo.bundleHeader.targetNodeId == senderNodeId) || (bundleInfo.bundleHeader.targetNodeId == ANY_NODEID)))) {

            requestedBundleIds.push_back(bundleId);

        }//if//

        offsetBytes += sizeof(BundleIdType);

    }//for//

    if (transportMode == TCP) {
        SendBundleWithTcp(requestedBundleIds, sourceNetworkAddress);
    }
    else if (transportMode == UDP) {
        SendBundleWithUdp(requestedBundleIds, sourceNetworkAddress);
    }
    else {
        assert(false);
        exit(1);
    }//if//

}//ProcessRequestPacket//


inline
void BundleProtocol::ProcessBundlePacket(
    const Packet& packet,
    const unsigned int messageLength,
    const NetworkAddress& sourceNetworkAddress)
{

    const BundleHeader bundleHeader = packet.GetAndReinterpretPayloadData<BundleHeader>();

    ProcessBundleData(bundleHeader, sourceNetworkAddress);

}//ProcessRequestPacket//


inline
void BundleProtocol::ProcessBundleData(
    const BundleHeader& bundleHeader,
    const NetworkAddress& sourceAddress)
{

    OutputTraceAndStatsForReceiveBundle(
        sourceAddress,
        bundleHeader.bundleId,
        bundleHeader.targetNodeId,
        bundleHeader.bundleSizeBytes);

    //already received
    if (storedBundles.find(bundleHeader.bundleId) != storedBundles.end()) {
        duplicateBundleReceivedStatPtr->IncrementCounter();
        return;
    }//if//

    const BundleInfo bundleInfo(bundleHeader.numOfCopies, bundleHeader);

    //no storage space
    if (((*this).currentStrageUsageBytes + bundleHeader.bundleSizeBytes) >= (*this).maxStorageSizeBytes) {
        bundlesDiscardedDueToLackOfStorageStatPtr->IncrementCounter();
        return;
    }//if//

    bool success = false;
    StoreBundle(bundleHeader.bundleId, bundleInfo, success);

    if (success) {

        if ((bundleHeader.targetNodeId == nodeId) || (bundleHeader.targetNodeId == ANY_NODEID)) {

            //for me

            //set zero
            SetNumberOfCopiesToZero(bundleHeader.bundleId);

            const TimeType delay =
                simulationEngineInterfacePtr->CurrentTime() - bundleHeader.sendTime;

            OutputTraceAndStatsForDeliveryBundle(
                bundleHeader.bundleId,
                bundleHeader.targetNodeId,
                bundleHeader.bundleSizeBytes,
                delay);

        }//if//

    }
    else {
        bundlesDiscardedDueToLackOfStorageStatPtr->IncrementCounter();
    }//if//

}//ProcessBundleData//


inline
void BundleProtocol::AcceptTcpConnection(
    const shared_ptr<TcpConnection>& newTcpConnectionPtr)
{

    TcpConnectionInfoType tcpConnectionInfo;

    tcpConnectionInfo.tcpEventHandlerPtr =
        shared_ptr<TcpEventHandler>(new TcpEventHandler(this));

    tcpConnectionInfo.tcpConnectionPtr = newTcpConnectionPtr;
    tcpConnectionInfo.tcpConnectionPtr->SetAppTcpEventHandler(tcpConnectionInfo.tcpEventHandlerPtr);

    if (useVirtualPayload) {
        tcpConnectionInfo.tcpConnectionPtr->EnableVirtualPayload();
    }//if//

    tcpConnectionInfo.tcpState = WAITING_FOR_ESTABLISHING;
    tcpConnectionInfo.destinationAddress =
        newTcpConnectionPtr->GetForeignAddress();

    UpdateTcpConnectionInfo(tcpConnectionInfo);


}//AcceptTcpConnection//


inline
void BundleProtocol::DoTcpIsReadyForMoreDataAction(
    const TcpEventHandler* tcpHandler)
{
    TcpStateType& tcpState = GetTcpConnectionInfo(tcpHandler).tcpState;

    assert(
        tcpState == WAITING_FOR_ESTABLISHING ||
        tcpState == ESTABLISHED ||
        tcpState == WAITING_FOR_CLOSING);

    if (tcpState == WAITING_FOR_ESTABLISHING) {
        tcpState = ESTABLISHED;
    }
    else if (tcpState == WAITING_FOR_CLOSING) {
        return;
    }//if//

    SendDataBlock(tcpHandler);

}//DoTcpIsReadyForMoreDataAction//


inline
void BundleProtocol::SendDataBlock(
    const TcpEventHandler* tcpHandler)
{

    TcpConnectionInfoType& tcpConnectionInfo = GetTcpConnectionInfo(tcpHandler);

    shared_ptr<TcpConnection>& tcpConnectionPtr = tcpConnectionInfo.tcpConnectionPtr;
    const TcpStateType tcpState = tcpConnectionInfo.tcpState;
    unsigned long long int& tcpAccumulatedDeliveredBytes = tcpConnectionInfo.tcpAccumulatedDeliveredBytes;
    unsigned long long int& totalSendingDataBytes = tcpConnectionInfo.totalSendingDataBytes;
    vector<BundleIdType>& sendingBundleIds = tcpConnectionInfo.sendingBundleIds;
    size_t& sendingBundleIndex = tcpConnectionInfo.sendingBundleIndex;

    assert(tcpState == ESTABLISHED);

    if (totalSendingDataBytes == 0) {
        return;
    }//if//

    const unsigned int tcpSentDataBytes =
        static_cast<unsigned int>(tcpConnectionPtr->GetNumberOfSentBytes() - tcpAccumulatedDeliveredBytes);

    assert(tcpSentDataBytes >= 0);

    if (tcpSentDataBytes < totalSendingDataBytes) {

        const unsigned int restOfSendingData = static_cast<unsigned int>(totalSendingDataBytes - tcpSentDataBytes);

        const unsigned int numberOfAvailableBufferBytes =
            static_cast<unsigned int>(tcpConnectionPtr->GetCurrentNumberOfAvailableBufferBytes());

        unsigned int dataBlockSize = 0;

        if (restOfSendingData > numberOfAvailableBufferBytes) {
            dataBlockSize = numberOfAvailableBufferBytes;
        }
        else {
            dataBlockSize = restOfSendingData;
        }//if//

        if (dataBlockSize > 0) {

            unsigned int remainingSentBytes = tcpSentDataBytes;

            //calculate remainingSentBytes
            for(size_t i = 0; i < sendingBundleIndex; i++) {

                assert(tcpConnectionInfo.sendingBundleSizeBytes[i] <= remainingSentBytes);

                remainingSentBytes -= tcpConnectionInfo.sendingBundleSizeBytes[i];

            }//for//

            BundleIdType workingBundleId;
            shared_ptr<vector<unsigned char> > workingBundlePtr;
            unsigned int workingBundleSizeBytes;

            shared_ptr<vector<unsigned char> > dataBlockPtr =
                shared_ptr<vector<unsigned char> >(new vector<unsigned char>());
            const unsigned int bundleHeaderSize = sizeof(BundleHeader);

            unsigned int availableDataBlockSize = dataBlockSize;

            while ((sendingBundleIndex < sendingBundleIds.size()) && (availableDataBlockSize > 0)) {

                workingBundleId = sendingBundleIds[sendingBundleIndex];
                workingBundlePtr = tcpConnectionInfo.sendingData[sendingBundleIndex];
                workingBundleSizeBytes = tcpConnectionInfo.sendingBundleSizeBytes[sendingBundleIndex];

                assert((*workingBundlePtr).size() == sizeof(BundleHeader));

                bool virtualPayloadUnavailable = false;
                if ((workingBundleSizeBytes - remainingSentBytes) < availableDataBlockSize) {
                    //can send more than one bundle
                    virtualPayloadUnavailable = true;
                }//if//

                //header part
                if (remainingSentBytes < bundleHeaderSize) {

                    const unsigned int headerPartSizeToBeSent = std::min((bundleHeaderSize - remainingSentBytes), availableDataBlockSize);
                    vector<unsigned char>::const_iterator iter =
                        (*workingBundlePtr).begin() + remainingSentBytes;

                    (*dataBlockPtr).insert((*dataBlockPtr).end(), iter, iter + headerPartSizeToBeSent);

                    availableDataBlockSize -= headerPartSizeToBeSent;

                }//if//

                //non header part
                if (bundleHeaderSize < (remainingSentBytes + availableDataBlockSize)) {

                    unsigned int nonHeaderPartSizeToBeSent;
                    if ((workingBundleSizeBytes - std::max(bundleHeaderSize, remainingSentBytes)) <= availableDataBlockSize) {
                        //can send rest of one bundle data
                        nonHeaderPartSizeToBeSent = (workingBundleSizeBytes - std::max(bundleHeaderSize, remainingSentBytes));

                        //go to next bundle
                        sendingBundleIndex++;
                        remainingSentBytes = 0;

                    }
                    else {

                        nonHeaderPartSizeToBeSent = availableDataBlockSize;

                    }//if//

                    if (virtualPayloadUnavailable) {
                        //add dummy data
                        (*dataBlockPtr).insert((*dataBlockPtr).end(), nonHeaderPartSizeToBeSent, 0);
                    }//if//

                    availableDataBlockSize -= nonHeaderPartSizeToBeSent;

                }//if//

                assert(availableDataBlockSize >= 0);

            }//while//

            tcpConnectionPtr->SendDataBlock(dataBlockPtr, dataBlockSize);

        }//if//
    }
    else {
        assert(tcpSentDataBytes == totalSendingDataBytes);
    }//if//

    const unsigned long long int numberOfDeliveredBytes =
        tcpConnectionPtr->GetNumberOfDeliveredBytes() - tcpAccumulatedDeliveredBytes;

    assert(numberOfDeliveredBytes >= 0);

    if (numberOfDeliveredBytes == totalSendingDataBytes) {

        assert(sendingBundleIndex == tcpConnectionInfo.sendingBundleSizeBytes.size());

        tcpAccumulatedDeliveredBytes = tcpConnectionPtr->GetNumberOfDeliveredBytes();
        totalSendingDataBytes = 0;
        sendingBundleIds.clear();
        sendingBundleIndex = 0;
        tcpConnectionInfo.sendingData.clear();
        tcpConnectionInfo.sendingBundleSizeBytes.clear();

    }
    else {
        assert(numberOfDeliveredBytes < totalSendingDataBytes);
    }//if//

}//SendDataBlock//


inline
void BundleProtocol::ReceiveDataBlock(
    const TcpEventHandler* tcpHandler,
    const unsigned char dataBlock[],
    const unsigned int dataLength,
    const unsigned int actualDataLength)
{

    TcpConnectionInfoType& tcpConnectionInfo = GetTcpConnectionInfo(tcpHandler);

    shared_ptr<TcpConnection>& tcpConnectionPtr = tcpConnectionInfo.tcpConnectionPtr;
    const TcpStateType& tcpState = tcpConnectionInfo.tcpState;
    unsigned long long int& bundleSizeBytesToBeReceived = tcpConnectionInfo.bundleSizeBytesToBeReceived;
    unsigned long long int& receivedDataBytes = tcpConnectionInfo.receivedDataBytes;
    vector<unsigned char>& receivedData = tcpConnectionInfo.receivedData;
    BundleHeader& receivingBundleHeader = tcpConnectionInfo.receivingBundleHeader;

    if (tcpState != ESTABLISHED) return;

    assert(tcpState == ESTABLISHED);
    assert(dataLength > 0);

    receivedDataBytes += dataLength;

    for (unsigned int i = 0; i < actualDataLength; i++) {
        back_inserter(receivedData) = dataBlock[i];
    }//for//

    if (dataLength != actualDataLength) {
        //insert dummy data
        receivedData.insert(receivedData.end(), (dataLength - actualDataLength), '0');

    }//if//

    while (((bundleSizeBytesToBeReceived == 0) && (receivedDataBytes >= sizeof(BundleHeader))) ||
        ((bundleSizeBytesToBeReceived != 0) && (receivedDataBytes >= bundleSizeBytesToBeReceived))) {

        //read header
        if ((bundleSizeBytesToBeReceived == 0) && (receivedDataBytes >= sizeof(BundleHeader))) {

            BundleHeader bundleHeader;
            memcpy(&bundleHeader, &receivedData[0], sizeof(BundleHeader));

            receivingBundleHeader = bundleHeader;

            bundleSizeBytesToBeReceived = bundleHeader.bundleSizeBytes;

        }//if//

        //read body
        if ((bundleSizeBytesToBeReceived != 0) && (receivedDataBytes >= bundleSizeBytesToBeReceived)) {

            vector<unsigned char>::const_iterator iter = receivedData.begin();

            vector<unsigned char> nextFragmentedBundle(iter + static_cast<unsigned int>(bundleSizeBytesToBeReceived), iter + static_cast<unsigned int>(receivedDataBytes));

            //received one bundle
            ProcessBundleData(
                receivingBundleHeader,
                tcpConnectionPtr->GetForeignAddress());

            receivedDataBytes -= bundleSizeBytesToBeReceived;
            bundleSizeBytesToBeReceived = 0;
            receivedData.assign(nextFragmentedBundle.begin(), nextFragmentedBundle.end());
        }
        else {
            if (receivedDataBytes >= sizeof(BundleHeader)) {
                assert(receivedDataBytes < bundleSizeBytesToBeReceived);
            }//if//
        }//if//

    }//while//

}//ReceiveDataBlock//


inline
void BundleProtocol::DoTcpRemoteHostClosedAction(
    const TcpEventHandler* tcpHandler)
{
    TcpStateType& tcpState = GetTcpConnectionInfo(tcpHandler).tcpState;

    assert(
        tcpState == WAITING_FOR_ESTABLISHING ||
        tcpState == ESTABLISHED);

    shared_ptr<TcpConnection>& tcpConnectionPtr = GetTcpConnectionInfo(tcpHandler).tcpConnectionPtr;

    tcpConnectionPtr->Close();
    tcpState = WAITING_FOR_CLOSING;

}//DoTcpRemoteHostClosedAction//


inline
void BundleProtocol::DoTcpLocalHostClosedAction(
    const TcpEventHandler* tcpHandler)
{
    TcpStateType& tcpState = GetTcpConnectionInfo(tcpHandler).tcpState;

    assert(
        tcpState == WAITING_FOR_ESTABLISHING ||
        tcpState == ESTABLISHED ||
        tcpState == WAITING_FOR_CLOSING);

    tcpState = CLOSE;

    DeleteTcpConnectionInfo(tcpHandler);

}//DoTcpLocalHostClosedAction//


inline
void BundleProtocol::OutputTraceForControlPacketSend(
    const MessageType& messageType,
    const NetworkAddress& destinationAddress)
{

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        NodeIdType destinationNodeId;
        if (destinationAddress.IsTheBroadcastOrAMulticastAddress()) {
            destinationNodeId = ANY_NODEID;
        }
        else {

            bool foundNodeId;
            networkAddressLookupInterfacePtr->LookupNodeId(
                destinationAddress, destinationNodeId, foundNodeId);
            if (!foundNodeId) {
                destinationNodeId = INVALID_NODEID;
            }//if//

        }//if//

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BundleControlSendTraceRecord traceData;

            traceData.messageType = static_cast<unsigned char>(messageType);
            traceData.destinationNodeId = destinationNodeId;

            assert(sizeof(traceData) == BUNDLE_CONTROL_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "CtrlSend", traceData);

        }
        else {

            ostringstream outStream;

            outStream
                << "Type= " << ConvertBundleMessageTypeToString(static_cast<unsigned char>(messageType))
                << " DestNodeId= " << destinationNodeId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "CtrlSend", outStream.str());

        }//if//

    }//if//

}//OutputTraceForControlPacketSend//


inline
void BundleProtocol::OutputTraceForCtrlPacketReceive(
    const MessageType& messageType,
    const NetworkAddress& sourceAddress)
{

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        NodeIdType sourceNodeId;
        bool foundNodeId;
        networkAddressLookupInterfacePtr->LookupNodeId(
            sourceAddress, sourceNodeId, foundNodeId);

        if (!foundNodeId) {
            sourceNodeId = INVALID_NODEID;
        }//if//

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BundleControlReceiveTraceRecord traceData;

            traceData.messageType = static_cast<unsigned char>(messageType);
            traceData.sourceNodeId = sourceNodeId;

            assert(sizeof(traceData) == BUNDLE_CONTROL_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "CtrlRecv", traceData);

        }
        else {

            ostringstream outStream;

            outStream
                << "Type= " << ConvertBundleMessageTypeToString(static_cast<unsigned char>(messageType))
                << " SrcNodeId= " << sourceNodeId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "CtrlRecv", outStream.str());

        }//if//

    }//if//

}//OutputTraceForCtrlPacketReceive//


inline
void BundleProtocol::OutputTraceAndStatsForGenerateBundle(
    const BundleIdType& bundleId,
    const NodeIdType& targetNodeId,
    const size_t& bundleSizeBytes)
{
    bundlesGeneratedStatPtr->IncrementCounter();
    bytesGeneratedStatPtr->IncrementCounter(bundleSizeBytes);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BundleGenerateTraceRecord traceData;

            traceData.bundleSizeBytes = static_cast<uint32_t>(bundleSizeBytes);
            traceData.originatorNodeId = ExtractOriginatorNodeId(bundleId);
            traceData.originatorSequenceNumber = ExtractOriginatorSequenceNumber(bundleId);
            traceData.targetNodeId = targetNodeId;

            assert(sizeof(traceData) == BUNDLE_GENERATE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "Generate", traceData);
        }
        else {

            ostringstream outStream;

            outStream
                << "Size= " << bundleSizeBytes
                << " OrigNodeId= " << ExtractOriginatorNodeId(bundleId)
                << " OrigSeq= " << ExtractOriginatorSequenceNumber(bundleId)
                << " TargetNodeId= " << targetNodeId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "Generate", outStream.str());

        }//if//

    }//if//


}//OutputTraceAndStatsForGenerateBundle//


inline
void BundleProtocol::OutputTraceAndStatsForDeliveryBundle(
    const BundleIdType& bundleId,
    const NodeIdType& targetNodeId,
    const size_t& bundleSizeBytes,
    const TimeType& delay)
{
    bundlesDeliveredStatPtr->IncrementCounter();
    bytesDeliveredStatPtr->IncrementCounter(bundleSizeBytes);
    bundleEndToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delay));

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BundleDeliveryTraceRecord traceData;

            traceData.bundleSizeBytes = static_cast<uint32_t>(bundleSizeBytes);
            traceData.originatorNodeId = ExtractOriginatorNodeId(bundleId);
            traceData.originatorSequenceNumber = ExtractOriginatorSequenceNumber(bundleId);
            traceData.targetNodeId = targetNodeId;
            traceData.delay = delay;

            assert(sizeof(traceData) == BUNDLE_DELIVERY_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "Delivery", traceData);

        }
        else {

            ostringstream outStream;

            outStream
                << "Size= " << bundleSizeBytes
                << " OrigNodeId= " << ExtractOriginatorNodeId(bundleId)
                << " OrigSeq= " << ExtractOriginatorSequenceNumber(bundleId)
                << " TargetNodeId= " << targetNodeId
                << " Delay= " << ConvertTimeToStringSecs(delay);

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "Delivery", outStream.str());

        }//if//
    }//if//

}//OutputTraceAndStatsForDeliveryBundle//



inline
void BundleProtocol::OutputTraceAndStatsForSendBundle(
    const NetworkAddress& nextNodeAddress,
    const BundleIdType& bundleId,
    const NodeIdType& targetNodeId,
    const size_t& bundleSizeBytes)
{
    bundlesSentStatPtr->IncrementCounter();
    bytesSentStatPtr->IncrementCounter(bundleSizeBytes);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        NodeIdType nextNodeId;
        bool foundNodeId;
        networkAddressLookupInterfacePtr->LookupNodeId(
            nextNodeAddress, nextNodeId, foundNodeId);
        if (!foundNodeId) {
            nextNodeId = INVALID_NODEID;
        }//if//

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BundleSendTraceRecord traceData;

            traceData.destinationNodeId = nextNodeId;
            traceData.bundleSizeBytes = static_cast<uint32_t>(bundleSizeBytes);
            traceData.originatorNodeId = ExtractOriginatorNodeId(bundleId);
            traceData.originatorSequenceNumber = ExtractOriginatorSequenceNumber(bundleId);
            traceData.targetNodeId = targetNodeId;

            assert(sizeof(traceData) == BUNDLE_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "Send", traceData);

        }
        else {

            ostringstream outStream;

            outStream
                << "DestNodeId= " << nextNodeId
                << " Size= " << bundleSizeBytes
                << " OrigNodeId= " << ExtractOriginatorNodeId(bundleId)
                << " OrigSeq= " << ExtractOriginatorSequenceNumber(bundleId)
                << " TargetNodeId= " << targetNodeId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "Send", outStream.str());

        }//if//

    }//if//

}//OutputTraceAndStatsForSendBundle//



inline
void BundleProtocol::OutputTraceAndStatsForReceiveBundle(
    const NetworkAddress& lastNodeAddress,
    const BundleIdType& bundleId,
    const NodeIdType& targetNodeId,
    const size_t& bundleSizeBytes)
{
    bundlesReceivedStatPtr->IncrementCounter();
    bytesReceivedStatPtr->IncrementCounter(bundleSizeBytes);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        NodeIdType lastNodeId;
        bool foundNodeId;
        networkAddressLookupInterfacePtr->LookupNodeId(
            lastNodeAddress, lastNodeId, foundNodeId);
        if (!foundNodeId) {
            lastNodeId = INVALID_NODEID;
        }//if//

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            BundleReceiveTraceRecord traceData;

            traceData.sourceNodeId = lastNodeId;
            traceData.bundleSizeBytes = static_cast<uint32_t>(bundleSizeBytes);
            traceData.originatorNodeId = ExtractOriginatorNodeId(bundleId);
            traceData.originatorSequenceNumber = ExtractOriginatorSequenceNumber(bundleId);
            traceData.targetNodeId = targetNodeId;

            assert(sizeof(traceData) == BUNDLE_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "Recv", traceData);

        }
        else {

            ostringstream outStream;

            outStream
                << "SrcNodeId= " << lastNodeId
                << " Size= " << bundleSizeBytes
                << " OrigNodeId= " << ExtractOriginatorNodeId(bundleId)
                << " OrigSeq= " << ExtractOriginatorSequenceNumber(bundleId)
                << " TargetNodeId= " << targetNodeId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "Recv", outStream.str());

        }//if//

    }//if//

}//OutputTraceAndStatsForReceiveBundle//


inline
void BundleProtocol::OutputTraceAndStatForStorageUsage()
{
    storageUsageBytesStatPtr->RecordStatValue(static_cast<double>(currentStrageUsageBytes));

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "StorageUsage", currentStrageUsageBytes);

        }
        else {

            ostringstream outStream;

            outStream << "StorageUsage= " << currentStrageUsageBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "StorageUsage", outStream.str());

        }//if//
    }//if//

}//OutputTraceAndStatForStorageUsage//


}//namespace//

#endif
