// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_APP_FLOODING_H
#define SCENSIM_APP_FLOODING_H

#include "scensim_netsim.h"

namespace ScenSim {

using std::cerr;
using std::endl;


//--------------------------------------------------------------------------------------------------

class FloodingApplication: public Application, public enable_shared_from_this<FloodingApplication> {
public:
    static const string modelName;
    static const int APPLICATION_ID_CHAR_MAX_LENGTH = 16;
    typedef char ApplicationIdCharType[APPLICATION_ID_CHAR_MAX_LENGTH];

    FloodingApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationIdType& initApplicationId,
        const NodeIdType& initNodeId,
        const unsigned short int initDefaultApplicationPortId);

    void CompleteInitialization();

    void AddSenderSetting(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationIdType& initApplicationId);

    void AddReceiverSetting(
        const ApplicationIdType& applicationId);

    virtual ~FloodingApplication() {}

    struct FloodingPayloadIdType {
        ApplicationIdCharType applicationIdChar;
        NodeIdType sourceNodeId;
        unsigned int sequenceNumber;

        FloodingPayloadIdType(
            const ApplicationIdType& initApplicationId,
            const NodeIdType& initSourceNodeId,
            const unsigned int initSequenceNumber)
            :
            sourceNodeId(initSourceNodeId),
            sequenceNumber(initSequenceNumber)
        {
            if (initApplicationId.size() >= APPLICATION_ID_CHAR_MAX_LENGTH) {
                cerr << "Error: A length of application ID ("
                    << initApplicationId << ") should be less than "
                    << APPLICATION_ID_CHAR_MAX_LENGTH << ")." << endl;
                exit(1);
            }//if//

            std::fill_n(applicationIdChar, APPLICATION_ID_CHAR_MAX_LENGTH, 0);
            initApplicationId.copy(applicationIdChar, APPLICATION_ID_CHAR_MAX_LENGTH - 1, 0);

        }//FloodingPayloadIdType//

        bool operator<(const FloodingPayloadIdType& right) const
        {
            return ((strcmp(applicationIdChar, right.applicationIdChar) < 0) ||
                    ((strcmp(applicationIdChar, right.applicationIdChar) == 0) &&
                     (sourceNodeId < right.sourceNodeId)) ||
                    ((strcmp(applicationIdChar, right.applicationIdChar) == 0) &&
                     (sourceNodeId == right.sourceNodeId) &&
                     (sequenceNumber < right.sequenceNumber)));
        }
    };//FloodingPayloadIdType//

    struct FloodingPayloadType {
        FloodingPayloadIdType id;
        TimeType broadcastTime;
        double nodePositionX;
        double nodePositionY;
        PacketPriorityType floodingPriority;
        unsigned int maxHopCount;
        unsigned int hopCount;
        TimeType minWaitingPeriod;
        TimeType maxWaitingPeriod;
        unsigned int counterThreshold;
        double distanceThresholdInMeters;

        FloodingPayloadType(
            const ApplicationIdType initApplicationId,
            const NodeIdType initSourceNodeId,
            const unsigned int initSequenceNumber,
            const TimeType initBroadcastTime,
            const double initNodePositionX,
            const double initNodePositionY,
            const PacketPriorityType initFloodingPriority,
            const unsigned int initMaxHopCount,
            const unsigned int initHopCount,
            const TimeType initMinWaitingPeriod,
            const TimeType initMaxWaitingPeriod,
            const unsigned int initCounterThreshold,
            const double initDistanceThresholdInMeters)
            :
            id(initApplicationId, initSourceNodeId, initSequenceNumber),
            broadcastTime(initBroadcastTime),
            nodePositionX(initNodePositionX),
            nodePositionY(initNodePositionY),
            floodingPriority(initFloodingPriority),
            maxHopCount(initMaxHopCount),
            hopCount(initHopCount),
            minWaitingPeriod(initMinWaitingPeriod),
            maxWaitingPeriod(initMaxWaitingPeriod),
            counterThreshold(initCounterThreshold),
            distanceThresholdInMeters(initDistanceThresholdInMeters)
        {}
    };//FloodingPayloadType//

private:

    class FloodingStartEvent: public SimulationEvent {
    public:
        explicit
        FloodingStartEvent(
            const shared_ptr<FloodingApplication>& initFloodingApplicationPtr,
            const ApplicationIdType& initApplicationId)
            :
            floodingApplicationPtr(initFloodingApplicationPtr),
            applicationId(initApplicationId)
        {}

        virtual void ExecuteEvent()
        {
            floodingApplicationPtr->Broadcast(applicationId);

        }//ExecuteEvent//

    private:
        shared_ptr<FloodingApplication> floodingApplicationPtr;
        ApplicationIdType applicationId;

    };//FloodingStartEvent//

    class FloodingRebroadcastEvent: public SimulationEvent {
    public:
        explicit
        FloodingRebroadcastEvent(
            const shared_ptr<FloodingApplication>& initFloodingApplicationPtr,
            const FloodingPayloadType& initFloodingPayload,
            const unsigned int initPacketPayloadSizeByte)
            :
            floodingApplicationPtr(initFloodingApplicationPtr),
            floodingPayload(
                string(initFloodingPayload.id.applicationIdChar),
                initFloodingPayload.id.sourceNodeId,
                initFloodingPayload.id.sequenceNumber,
                initFloodingPayload.broadcastTime,
                initFloodingPayload.nodePositionX,
                initFloodingPayload.nodePositionY,
                initFloodingPayload.floodingPriority,
                initFloodingPayload.maxHopCount,
                initFloodingPayload.hopCount,
                initFloodingPayload.minWaitingPeriod,
                initFloodingPayload.maxWaitingPeriod,
                initFloodingPayload.counterThreshold,
                initFloodingPayload.distanceThresholdInMeters),
            packetPayloadSizeBytes(initPacketPayloadSizeByte)
        {}

        virtual void ExecuteEvent()
        {
            floodingApplicationPtr->RebroadcastIfNecessary(
                floodingPayload,
                packetPayloadSizeBytes);

        }//ExecuteEvent//

    private:
        shared_ptr<FloodingApplication> floodingApplicationPtr;
        FloodingPayloadType floodingPayload;
        unsigned int packetPayloadSizeBytes;

    };//FloodingRebroadcastEvent//

    class PacketHandler: public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        PacketHandler(
            const shared_ptr<FloodingApplication>& initFloodingApplicationPtr)
            :
            floodingApplicationPtr(initFloodingApplicationPtr)
        {}

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePort,
            const NetworkAddress& destinationAddress,
            const PacketPriorityType& priority)
        {
            floodingApplicationPtr->ReceivePacket(packetPtr);

        }//ReceivePacket//

    private:
        shared_ptr<FloodingApplication> floodingApplicationPtr;

    };//PacketHandler//

    struct FloodingSenderSettingType {
        int packetPayloadSizeBytes;
        TimeType packetInterval;
        TimeType floodingEndTime;
        PacketPriorityType floodingPriority;
        unsigned int maxHopCount;
        TimeType minWaitingPeriod;
        TimeType maxWaitingPeriod;
        unsigned int counterThreshold;
        double distanceThresholdInMeters;
        unsigned int sequenceNumber;

        FloodingSenderSettingType(
            const int initPacketPayloadSizeBytes,
            const TimeType initPacketInterval,
            const TimeType initFloodingEndTime,
            const PacketPriorityType initFloodingPriority,
            const unsigned int initMaxHopCount,
            const TimeType initMinWaitingPeriod,
            const TimeType initMaxWaitingPeriod,
            const unsigned int initCounterThreshold,
            const double initDistanceThresholdInMeters)
            :
            packetPayloadSizeBytes(initPacketPayloadSizeBytes),
            packetInterval(initPacketInterval),
            floodingEndTime(initFloodingEndTime),
            floodingPriority(initFloodingPriority),
            maxHopCount(initMaxHopCount),
            minWaitingPeriod(initMinWaitingPeriod),
            maxWaitingPeriod(initMaxWaitingPeriod),
            counterThreshold(initCounterThreshold),
            distanceThresholdInMeters(initDistanceThresholdInMeters),
            sequenceNumber(0)
        {}
    };//FloodingSenderSettingType//

    struct FloodingSenderStatType {
        shared_ptr<CounterStatistic> packetsBroadcastStatPtr;
        shared_ptr<CounterStatistic> bytesBroadcastStatPtr;

        FloodingSenderStatType(
            const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
            const string& initModelName,
            const ApplicationIdType initApplicationId)
            :
            packetsBroadcastStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_PacketsBroadcast"))),
            bytesBroadcastStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_BytesBroadcast")))
        {}
    };//FloodingSenderStatType//

    struct FloodingReceiverStatType {
        shared_ptr<CounterStatistic> packetsRebroadcastStatPtr;
        shared_ptr<CounterStatistic> bytesRebroadcastStatPtr;
        shared_ptr<CounterStatistic> packetsReceivedStatPtr;
        shared_ptr<CounterStatistic> bytesReceivedStatPtr;
        shared_ptr<CounterStatistic> packetsDiscardedStatPtr;
        shared_ptr<CounterStatistic> bytesDiscardedStatPtr;
        shared_ptr<RealStatistic> endToEndDelayStatPtr;
        shared_ptr<RealStatistic> hopCountStatPtr;
        unsigned int countOfOriginalPacketReceived;

        FloodingReceiverStatType(
            const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
            const string& initModelName,
            const ApplicationIdType initApplicationId)
            :
            packetsRebroadcastStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_PacketsRebroadcast"))),
            bytesRebroadcastStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_BytesRebroadcast"))),
            packetsReceivedStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_PacketsReceived"))),
            bytesReceivedStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_BytesReceived"))),
            packetsDiscardedStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName +"_" +  initApplicationId + "_PacketsDiscarded"))),
            bytesDiscardedStatPtr(
                initSimulationEngineInterfacePtr->CreateCounterStat(
                    (initModelName + "_" + initApplicationId + "_BytesDiscarded"))),
            endToEndDelayStatPtr(
                initSimulationEngineInterfacePtr->CreateRealStat(
                    (initModelName + "_" + initApplicationId + "_EndToEndDelay"))),
            hopCountStatPtr(
                initSimulationEngineInterfacePtr->CreateRealStat(
                    (initModelName + "_" + initApplicationId + "_HopCount"))),
            countOfOriginalPacketReceived(0)
        {}
    };//FloodingReceiverStatType//

    void Broadcast(const ApplicationIdType& applicationId);

    void Rebroadcast(
        FloodingPayloadType& floodingPayload,
        const unsigned int packetPayloadSizeBytes);

    void RebroadcastIfNecessary(
        FloodingPayloadType& floodingPayload,
        const unsigned int packetPayloadSizeBytes);

    void ScheduleRebroadcastEvent(
        const FloodingPayloadType& floodingPayload,
        const unsigned int packetPayloadSizeBytes);

    void ReceivePacket(unique_ptr<Packet>& packetPtr);

    void IncrementCountOfPacketReceived(const FloodingPayloadIdType& id);
    void UpdateMinDistanceBetweenNodesInMeters(const FloodingPayloadType& floodingPayload);

    double CalculateDistanceBetweenPointsInMeters(
        const double& x1, const double& y1,
        const double& x2, const double& y2) const;

    bool IsSenderMyself(const FloodingPayloadIdType& id) const;
    bool IsPacketFirstlyReceived(const FloodingPayloadIdType& id) const;
    bool IsPacketReceived(const FloodingPayloadType& floodingPayload) const;
    bool IsMaxHopCountReached(const FloodingPayloadType& floodingPayload) const;
    bool IsCounterThresholdReached(const FloodingPayloadType& floodingPayload) const;
    bool IsLessThanDistanceThresholdInMeters(const FloodingPayloadType& floodingPayload) const;

    void OutputTraceAndStatsForBroadcast(
        const ApplicationIdType& applicatoinId,
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const unsigned int packetLengthBytes);

    void OutputTraceAndStatsForRebroadcast(
        const ApplicationIdType& applicatoinId,
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const unsigned int packetLengthBytes);

    void OutputTraceAndStatsForReceivePacket(
        const ApplicationIdType& applicatoinId,
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const unsigned int packetLengthBytes,
        const unsigned int hopCount,
        const TimeType& delay);

    void OutputTraceAndStatsForDiscardPacket(
        const ApplicationIdType& applicatoinId,
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const unsigned int packetLengthBytes,
        const TimeType& delay);

    NodeIdType nodeId;
    unsigned short int destinationPortId;
    shared_ptr<PacketHandler> packetHandlerPtr;
    map<ApplicationIdType, FloodingSenderSettingType> floodingSenderSettings;
    map<ApplicationIdType, FloodingSenderStatType> floodingSenderStats;
    map<ApplicationIdType, FloodingReceiverStatType> floodingReceiverStats;
    map<FloodingPayloadIdType, unsigned int> countOfPacketReceived;
    map<FloodingPayloadIdType, double> minDistanceBetweenNodesInMeters;
    bool useVirtualPayload;

};//FloodingApplication//


inline
FloodingApplication::FloodingApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationIdType& initApplicationId,
    const NodeIdType& initNodeId,
    const unsigned short int initDefaultApplicationPortId)
    :
    Application(initSimulationEngineInterfacePtr, modelName),
    nodeId(initNodeId),
    destinationPortId(initDefaultApplicationPortId),
    packetHandlerPtr(),
    floodingSenderSettings(),
    floodingSenderStats(),
    floodingReceiverStats(),
    countOfPacketReceived(),
    minDistanceBetweenNodesInMeters(),
    useVirtualPayload(false)
{
    if (parameterDatabaseReader.ParameterExists(
            "flooding-auto-port-mode", nodeId, initApplicationId)) {

        if (!parameterDatabaseReader.ReadBool(
                "flooding-auto-port-mode", nodeId, initApplicationId)) {

            destinationPortId = static_cast<unsigned short int>(
                parameterDatabaseReader.ReadNonNegativeInt(
                    "flooding-destination-port", nodeId, initApplicationId));
        }//if//
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        "flooding-use-virtual-payload", nodeId, initApplicationId)) {

        useVirtualPayload = parameterDatabaseReader.ReadBool(
            "flooding-use-virtual-payload", nodeId, initApplicationId);
    }//if//

}//FloodingApplication//


inline
void FloodingApplication::CompleteInitialization()
{
    packetHandlerPtr = shared_ptr<PacketHandler>(new PacketHandler(shared_from_this()));

    assert(transportLayerPtr->udpPtr->PortIsAvailable(destinationPortId));

    transportLayerPtr->udpPtr->OpenSpecificUdpPort(
        NetworkAddress::anyAddress,
        destinationPortId,
        packetHandlerPtr);

}//CompleteInitialization//


inline
void FloodingApplication::AddSenderSetting(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationIdType& initApplicationId)
{
    applicationId = initApplicationId;

    const int packetPayloadSizeBytes =
        theParameterDatabaseReader.ReadInt("flooding-payload-size-bytes", nodeId, applicationId);

    const TimeType packetInterval =
        theParameterDatabaseReader.ReadTime("flooding-interval", nodeId, applicationId);

    const TimeType floodingStartTime =
        theParameterDatabaseReader.ReadTime("flooding-start-time", nodeId, applicationId);

    const TimeType floodingEndTime =
        theParameterDatabaseReader.ReadTime("flooding-end-time", nodeId, applicationId);

    const PacketPriorityType floodingPriority = static_cast<PacketPriorityType>(
        theParameterDatabaseReader.ReadNonNegativeInt("flooding-priority", nodeId, applicationId));

    const unsigned int maxHopCount =
        theParameterDatabaseReader.ReadNonNegativeInt("flooding-max-hop-count", nodeId, applicationId);

    const TimeType minWaitingPeriod =
        theParameterDatabaseReader.ReadTime("flooding-min-waiting-period", nodeId, applicationId);

    const TimeType maxWaitingPeriod =
        theParameterDatabaseReader.ReadTime("flooding-max-waiting-period", nodeId, applicationId);

    const unsigned int counterThreshold =
        theParameterDatabaseReader.ReadNonNegativeInt("flooding-counter-threshold", nodeId, applicationId);

    const double distanceThresholdInMeters =
        theParameterDatabaseReader.ReadDouble("flooding-distance-threshold-in-meters", nodeId, applicationId);

    if (packetPayloadSizeBytes < (int)sizeof(FloodingPayloadType)) {
        cerr << "Error: Packet payload size ("
            << packetPayloadSizeBytes << ") should be "
            << sizeof(FloodingPayloadType) << " bytes or larger." << endl;
        exit(1);
    }//if//

    if (packetInterval <= ZERO_TIME) {
        cerr << "Error: Broadcast interval ("
            << minWaitingPeriod << ") should be larger than "
            << ZERO_TIME << "." << endl;
        exit(1);
    }//if//

    if (floodingStartTime < ZERO_TIME) {
        cerr << "Error: Start time ("
            << floodingStartTime << ") should be "
            << ZERO_TIME << " or larger." << endl;
        exit(1);
    }//if//

    if (floodingStartTime >= floodingEndTime) {
        cerr << "Error: End time ("
            << floodingEndTime << ") should be larger than start time ("
            << floodingStartTime << ")." << endl;
        exit(1);
    }//if//

    if (minWaitingPeriod < ZERO_TIME) {
        cerr << "Error: Min waiting period ("
            << minWaitingPeriod << ") should be "
            << ZERO_TIME << " or larger." << endl;
        exit(1);
    }//if//

    if (minWaitingPeriod > maxWaitingPeriod) {
        cerr << "Error: Max waiting period ("
            << maxWaitingPeriod << ") should be min waiting period ("
            << minWaitingPeriod << ") or larger." << endl;
        exit(1);
    }//if//

    if (distanceThresholdInMeters < 0) {
        cerr << "Error: Distance threshold ("
            << distanceThresholdInMeters << ") should be 0 meters or larger." << endl;
        exit(1);
    }//if//

    FloodingSenderSettingType floodingSenderSetting(
        packetPayloadSizeBytes,
        packetInterval,
        floodingEndTime,
        floodingPriority,
        maxHopCount,
        minWaitingPeriod,
        maxWaitingPeriod,
        counterThreshold,
        distanceThresholdInMeters);

    floodingSenderSettings.insert(make_pair(applicationId, floodingSenderSetting));

    FloodingSenderStatType floodingSenderStat(
        simulationEngineInterfacePtr,
        modelName,
        applicationId);

    floodingSenderStats.insert(make_pair(applicationId, floodingSenderStat));

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    TimeType startTime = floodingStartTime;

    if (theParameterDatabaseReader.ParameterExists(
            "flooding-start-time-max-jitter", nodeId, applicationId)) {

        const TimeType maxStartTimeJitter =
            theParameterDatabaseReader.ReadTime(
                "flooding-start-time-max-jitter", nodeId, applicationId);

        startTime += static_cast<TimeType>(
            aRandomNumberGeneratorPtr->GenerateRandomDouble() * maxStartTimeJitter);
    }//if//

    if (currentTime > startTime) {
        const size_t nextTransmissionTime =
            size_t(ceil(double(currentTime - floodingStartTime) / packetInterval));
        startTime += nextTransmissionTime * packetInterval;
    }//if//

    if (startTime < floodingEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new FloodingStartEvent(shared_from_this(), applicationId)),
            startTime);
    }//if//

}//AddSenderSetting//


inline
void FloodingApplication::AddReceiverSetting(
    const ApplicationIdType& senderApplicationId)
{
    FloodingReceiverStatType floodingReceiverStat(
        simulationEngineInterfacePtr,
        modelName,
        senderApplicationId);

    floodingReceiverStats.insert(
        make_pair(senderApplicationId, floodingReceiverStat));

}//AddReceiverSetting//


inline
void FloodingApplication::Broadcast(const ApplicationIdType& applicationId)
{
    typedef map<ApplicationIdType, FloodingSenderSettingType>::iterator IterType;
    IterType iter = floodingSenderSettings.find(applicationId);

    assert(iter != floodingSenderSettings.end());
    assert((*iter).second.sequenceNumber < UINT_MAX);
    (*iter).second.sequenceNumber++;

    ObjectMobilityPosition nodePosition;
    nodeMobilityModelPtr->GetPositionForTime(
        simulationEngineInterfacePtr->CurrentTime(), nodePosition);

    const int packetPayloadSizeBytes = (*iter).second.packetPayloadSizeBytes;
    const TimeType packetInterval = (*iter).second.packetInterval;
    const TimeType floodingEndTime = (*iter).second.floodingEndTime;
    const unsigned int sequenceNumber = (*iter).second.sequenceNumber;
    const PacketPriorityType floodingPriority = (*iter).second.floodingPriority;
    const unsigned int maxHopCount = (*iter).second.maxHopCount;
    const TimeType minWaitingPeriod = (*iter).second.minWaitingPeriod;
    const TimeType maxWaitingPeriod = (*iter).second.maxWaitingPeriod;
    const unsigned int counterThreshold = (*iter).second.counterThreshold;
    const double distanceThresholdInMeters = (*iter).second.distanceThresholdInMeters;
    const double nodePositionX = nodePosition.X_PositionMeters();
    const double nodePositionY = nodePosition.Y_PositionMeters();
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    const TimeType nextPacketTime = currentTime + packetInterval;

    FloodingPayloadType floodingPayload(
        applicationId,
        nodeId,
        sequenceNumber,
        currentTime,
        nodePositionX,
        nodePositionY,
        floodingPriority,
        maxHopCount,
        0,
        minWaitingPeriod,
        maxWaitingPeriod,
        counterThreshold,
        distanceThresholdInMeters);

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(
            *simulationEngineInterfacePtr,
            floodingPayload,
            packetPayloadSizeBytes,
            useVirtualPayload);

    OutputTraceAndStatsForBroadcast(
        applicationId,
        sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes());

    transportLayerPtr->udpPtr->SendPacket(
        packetPtr, 0, NetworkAddress::broadcastAddress, destinationPortId, floodingPriority);

    if (nextPacketTime < floodingEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new FloodingStartEvent(shared_from_this(), applicationId)),
            nextPacketTime);
    }//if//

}//Broadcast//


inline
void FloodingApplication::Rebroadcast(
    FloodingPayloadType& floodingPayload,
    const unsigned int packetPayloadSizeBytes)
{
    const PacketPriorityType floodingPriority = floodingPayload.floodingPriority;

    ObjectMobilityPosition nodePosition;
    nodeMobilityModelPtr->GetPositionForTime(
        simulationEngineInterfacePtr->CurrentTime(), nodePosition);

    floodingPayload.nodePositionX = nodePosition.X_PositionMeters();
    floodingPayload.nodePositionY = nodePosition.Y_PositionMeters();

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(
            *simulationEngineInterfacePtr,
            floodingPayload,
            packetPayloadSizeBytes,
            useVirtualPayload);

    OutputTraceAndStatsForRebroadcast(
        string(floodingPayload.id.applicationIdChar),
        floodingPayload.id.sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes());

    transportLayerPtr->udpPtr->SendPacket(
        packetPtr, 0, NetworkAddress::broadcastAddress, destinationPortId, floodingPriority);

}//Rebroadcast//


inline
void FloodingApplication::RebroadcastIfNecessary(
    FloodingPayloadType& floodingPayload,
    const unsigned int packetPayloadSizeBytes)
{
    if (IsCounterThresholdReached(floodingPayload)) {
        return;
    }//if//

    if (IsLessThanDistanceThresholdInMeters(floodingPayload)) {
        return;
    }//if//

    Rebroadcast(floodingPayload, packetPayloadSizeBytes);

}//RebroadcastIfNecessary//


inline
void FloodingApplication::ScheduleRebroadcastEvent(
    const FloodingPayloadType& floodingPayload,
    const unsigned int packetPayloadSizeBytes)
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    const TimeType minWaitingPeriod = floodingPayload.minWaitingPeriod;
    const TimeType maxWaitingPeriod = floodingPayload.maxWaitingPeriod;

    TimeType nextPacketTime = currentTime + minWaitingPeriod;

    nextPacketTime += static_cast<TimeType>(
        aRandomNumberGeneratorPtr->GenerateRandomDouble() * (maxWaitingPeriod - minWaitingPeriod));

    simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(
            new FloodingRebroadcastEvent(
                shared_from_this(),
                floodingPayload,
                packetPayloadSizeBytes)),
        nextPacketTime);

}//ScheduleRebroadcastEvent//


inline
void FloodingApplication::ReceivePacket(unique_ptr<Packet>& packetPtr)
{
    FloodingPayloadType floodingPayload =
        packetPtr->GetAndReinterpretPayloadData<FloodingPayloadType>();

    assert(floodingPayload.hopCount < floodingPayload.maxHopCount);
    floodingPayload.hopCount++;

    IncrementCountOfPacketReceived(floodingPayload.id);
    UpdateMinDistanceBetweenNodesInMeters(floodingPayload);

    const TimeType delay =
        simulationEngineInterfacePtr->CurrentTime() - floodingPayload.broadcastTime;

    if (IsPacketReceived(floodingPayload)) {

        OutputTraceAndStatsForReceivePacket(
            string(floodingPayload.id.applicationIdChar),
            floodingPayload.id.sequenceNumber,
            packetPtr->GetPacketId(),
            packetPtr->LengthBytes(),
            floodingPayload.hopCount,
            delay);

        if (!IsMaxHopCountReached(floodingPayload)) {
            ScheduleRebroadcastEvent(floodingPayload, packetPtr->LengthBytes());
        }//if//
    }
    else {

        OutputTraceAndStatsForDiscardPacket(
            string(floodingPayload.id.applicationIdChar),
            floodingPayload.id.sequenceNumber,
            packetPtr->GetPacketId(),
            packetPtr->LengthBytes(),
            delay);
    }//if//

    packetPtr = nullptr;

}//ReceivePacket//


inline
void FloodingApplication::IncrementCountOfPacketReceived(
    const FloodingPayloadIdType& id)
{
    typedef map<FloodingPayloadIdType, unsigned int>::iterator IterType;
    IterType iter = countOfPacketReceived.find(id);

    if (iter == countOfPacketReceived.end()) {
        countOfPacketReceived.insert(make_pair(id, 1));
    }
    else {
        unsigned int count = (*iter).second;
        count++;
        (*iter).second = count;
    }//if//

}//IncrementCountOfPacketReceived//


inline
void FloodingApplication::UpdateMinDistanceBetweenNodesInMeters(
    const FloodingPayloadType& floodingPayload)
{
    ObjectMobilityPosition nodePosition;
    nodeMobilityModelPtr->GetPositionForTime(
        simulationEngineInterfacePtr->CurrentTime(), nodePosition);

    const double distanceInMeters =
        CalculateDistanceBetweenPointsInMeters(
            nodePosition.X_PositionMeters(),
            nodePosition.Y_PositionMeters(),
            floodingPayload.nodePositionX,
            floodingPayload.nodePositionY);

    typedef map<FloodingPayloadIdType, double>::iterator IterType;
    IterType iter = minDistanceBetweenNodesInMeters.find(floodingPayload.id);

    if (iter == minDistanceBetweenNodesInMeters.end()) {
        minDistanceBetweenNodesInMeters.insert(make_pair(floodingPayload.id, distanceInMeters));
    }
    else {
        double minDistanceInMeters = (*iter).second;

        if (distanceInMeters < minDistanceInMeters) {
            minDistanceInMeters = distanceInMeters;
            (*iter).second = minDistanceInMeters;
        }//if//
    }//if//

}//UpdateMinDistanceBetweenNodesInMeters//


inline
double FloodingApplication::CalculateDistanceBetweenPointsInMeters(
    const double& x1, const double& y1,
    const double& x2, const double& y2) const
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

}//CalculateDistanceBetweenPointsInMeters//


inline
bool FloodingApplication::IsSenderMyself(
    const FloodingPayloadIdType& id) const
{
    if (id.sourceNodeId == nodeId) {
        return true;
    }
    else {
        return false;
    }//if//

}//IsSenderMyself//


inline
bool FloodingApplication::IsPacketFirstlyReceived(
    const FloodingPayloadIdType& id) const
{
    typedef map<FloodingPayloadIdType, unsigned int>::const_iterator IterType;
    IterType iter = countOfPacketReceived.find(id);

    assert(iter != countOfPacketReceived.end());

    const unsigned int count = (*iter).second;

    if (count == 1) {
        return true;
    }
    else {
        return false;
    }//if//

}//IsPacketFirstlyReceived//


inline
bool FloodingApplication::IsPacketReceived(
    const FloodingPayloadType& floodingPayload) const
{
    if (!IsSenderMyself(floodingPayload.id) &&
        IsPacketFirstlyReceived(floodingPayload.id)) {

        return true;
    }
    else {
        return false;
    }//if//

}//IsPacketReceived//


inline
bool FloodingApplication::IsMaxHopCountReached(
    const FloodingPayloadType& floodingPayload) const
{
    if (floodingPayload.hopCount == floodingPayload.maxHopCount) {
        return true;
    }
    else {
        return false;
    }//if//

}//IsMaxHopCountReached//


inline
bool FloodingApplication::IsCounterThresholdReached(
    const FloodingPayloadType& floodingPayload) const
{
    typedef map<FloodingPayloadIdType, unsigned int>::const_iterator IterType;
    IterType iter = countOfPacketReceived.find(floodingPayload.id);

    assert(iter != countOfPacketReceived.end());

    const unsigned int count = (*iter).second;

    if (count >= floodingPayload.counterThreshold) {
        return true;
    }
    else {
        return false;
    }//if//

}//IsCounterThresholdReached//


inline
bool FloodingApplication::IsLessThanDistanceThresholdInMeters(
    const FloodingPayloadType& floodingPayload) const
{
    typedef map<FloodingPayloadIdType, double>::const_iterator IterType;
    IterType iter = minDistanceBetweenNodesInMeters.find(floodingPayload.id);

    assert(iter != minDistanceBetweenNodesInMeters.end());

    const double minDistanceInMeters = (*iter).second;

    if (minDistanceInMeters < floodingPayload.distanceThresholdInMeters) {
        return true;
    }
    else {
        return false;
    }//if//

}//IsLessThanDistanceThresholdInMeters//


inline
void FloodingApplication::OutputTraceAndStatsForBroadcast(
    const ApplicationIdType& applicationId,
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const unsigned int packetLengthBytes)
{
    typedef map<ApplicationIdType, FloodingSenderStatType>::iterator IterType;
    IterType iter = floodingSenderStats.find(applicationId);

    assert(iter != floodingSenderStats.end());

    (*iter).second.packetsBroadcastStatPtr->IncrementCounter();
    (*iter).second.bytesBroadcastStatPtr->IncrementCounter(packetLengthBytes);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.destinationNodeId = ANY_NODEID;
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "FloodingBroadcast", traceData);
        }
        else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "FloodingBroadcast", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForBroadcast//


inline
void FloodingApplication::OutputTraceAndStatsForRebroadcast(
    const ApplicationIdType& receiveApplicationId,
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const unsigned int packetLengthBytes)
{
    typedef map<ApplicationIdType, FloodingReceiverStatType>::iterator IterType;
    IterType iter = floodingReceiverStats.find(receiveApplicationId);

    assert(iter != floodingReceiverStats.end());

    (*iter).second.packetsRebroadcastStatPtr->IncrementCounter();
    (*iter).second.bytesRebroadcastStatPtr->IncrementCounter(packetLengthBytes);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.destinationNodeId = ANY_NODEID;
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, receiveApplicationId, "FloodingRebroadcast", traceData);
        }
        else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, receiveApplicationId, "FloodingRebroadcast", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForRebroadcast//


inline
void FloodingApplication::OutputTraceAndStatsForReceivePacket(
    const ApplicationIdType& receiveApplicationId,
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const unsigned int packetLengthBytes,
    const unsigned int hopCount,
    const TimeType& delay)
{
    typedef map<ApplicationIdType, FloodingReceiverStatType>::iterator IterType;
    IterType iter = floodingReceiverStats.find(receiveApplicationId);

    assert(iter != floodingReceiverStats.end());

    (*iter).second.packetsReceivedStatPtr->IncrementCounter();
    (*iter).second.bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);
    (*iter).second.endToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delay));
    (*iter).second.hopCountStatPtr->RecordStatValue(static_cast<double>(hopCount));
    (*iter).second.countOfOriginalPacketReceived++;

    const unsigned int countOfOriginalPacketReceived = (*iter).second.countOfOriginalPacketReceived;

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.delay = delay;
            traceData.receivedPackets = countOfOriginalPacketReceived;
            traceData.packetLengthBytes = static_cast<uint16_t>(packetLengthBytes);

            assert(sizeof(traceData) == APPLICATION_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, receiveApplicationId, "FloodingReceive", traceData);
        }
        else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId
                      << " Delay= " << ConvertTimeToStringSecs(delay)
                      << " Pdr= " << countOfOriginalPacketReceived << '/' << sequenceNumber
                      << " PacketBytes= " << packetLengthBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, receiveApplicationId, "FloodingReceive", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForReceivePacket//


inline
void FloodingApplication::OutputTraceAndStatsForDiscardPacket(
    const ApplicationIdType& receiveApplicationId,
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const unsigned int packetLengthBytes,
    const TimeType& delay)
{
    typedef map<ApplicationIdType, FloodingReceiverStatType>::iterator IterType;
    IterType iter = floodingReceiverStats.find(receiveApplicationId);

    assert(iter != floodingReceiverStats.end());

    (*iter).second.packetsDiscardedStatPtr->IncrementCounter();
    (*iter).second.bytesDiscardedStatPtr->IncrementCounter(packetLengthBytes);

    const unsigned int countOfOriginalPacketReceived = (*iter).second.countOfOriginalPacketReceived;

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            FloodingApplicationDiscardTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.packetLengthBytes = static_cast<uint16_t>(packetLengthBytes);

            assert(sizeof(traceData) == FLOODING_APPLICATION_DISCARD_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, receiveApplicationId, "FloodingDiscard", traceData);
        }
        else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId
                      << " PacketBytes= " << packetLengthBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, receiveApplicationId, "FloodingDiscard", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForDiscardPacket//






}//namespace//

#endif
