// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef ALOHAMAC_H
#define ALOHAMAC_H

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "scensim_prop.h"
#include "scensim_time.h"
#include "aloha_phy.h"

#include <memory>

#include <sstream>
#include <queue>
#include <map>

namespace Aloha {

using std::cerr;
using std::endl;
using std::ostringstream;
using std::shared_ptr;
using std::enable_shared_from_this;
using std::unique_ptr;
using std::move;

using std::queue;
using std::map;

using ScenSim::ParameterDatabaseReader;
using ScenSim::TimeType;
using ScenSim::ZERO_TIME;
using ScenSim::ConvertTimeToStringSecs;
using ScenSim::ConvertDoubleSecsToTime;
using ScenSim::ConvertTimeToDoubleSecs;
using ScenSim::MakeLowerCaseString;
using ScenSim::SimulationEngine;
using ScenSim::SimulationEngineInterface;
using ScenSim::SimulationEvent;
using ScenSim::EventRescheduleTicket;
using ScenSim::CounterStatistic;
using ScenSim::TraceMac;
using ScenSim::RandomNumberGenerator;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::HashInputsToMakeSeed;
using ScenSim::NodeIdType;
using ScenSim::InterfaceIdType;
using ScenSim::NetworkLayer;
using ScenSim::NetworkAddress;
using ScenSim::Packet;
using ScenSim::PacketPriorityType;
using ScenSim::FifoInterfaceOutputQueue;
using ScenSim::MacLayer;
using ScenSim::MacAddressResolver;


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

typedef ScenSim::SixByteMacAddressType MacAddressType;

typedef AlohaPhy::PropagationModelType PropagationModelType;
typedef AlohaPhy::PropagationInterfaceType PropagationInterfaceType;


enum AlohaNodeType { AP, STA, OTHER };

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class AlohaMac;

// Only allows for one non-infrastructure interface to the same channel at a time.

class SimpleMacAddressResolver : public MacAddressResolver<MacAddressType> {
public:
    SimpleMacAddressResolver(const shared_ptr<AlohaMac>& initMacPtr) : macPtr(initMacPtr) { }

    void GetMacAddress(
        const NetworkAddress& aNetworkAddress,
        const NetworkAddress& networkAddressMask,
        bool& wasFound,
        MacAddressType& resolvedMacAddress);

    // Used only to get last hop address.

    void GetNetworkAddressIfAvailable(
        const MacAddressType& macAddress,
        const NetworkAddress& subnetNetworkAddress,
        bool& wasFound,
        NetworkAddress& resolvedNetworkAddress);

private:
    shared_ptr<AlohaMac> macPtr;

};//SimpleMacAddressResolver//

inline
void SimpleMacAddressResolver::GetMacAddress(
    const NetworkAddress& aNetworkAddress,
    const NetworkAddress& networkAddressMask,
    bool& wasFound,
    MacAddressType& resolvedMacAddress)
{
    wasFound = true;

    if (aNetworkAddress.IsTheBroadcastAddress()) {
        resolvedMacAddress = MacAddressType::GetBroadcastAddress();
    }
    else {
        const NodeIdType nodeId =
            aNetworkAddress.MakeAddressWithZeroedSubnetBits(networkAddressMask).GetRawAddressLow32Bits();

        resolvedMacAddress = MacAddressType(nodeId, 0/*interfaceSelectorByte*/);
    }//if//

}//GetMacAddress//

inline
void SimpleMacAddressResolver::GetNetworkAddressIfAvailable(
    const MacAddressType& macAddress,
    const NetworkAddress& subnetworkAddress,
    bool& wasFound,
    NetworkAddress& resolvedNetworkAddress)
{
    wasFound = true;
    resolvedNetworkAddress =
        NetworkAddress(subnetworkAddress, NetworkAddress(macAddress.ExtractNodeId()));

}//GetNetworkAddressIfAvailable//

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

class AlohaMac : public MacLayer, public enable_shared_from_this<AlohaMac> {

public:
    static const string modelName;

    static
    shared_ptr<AlohaMac> Create(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<PropagationInterfaceType>& initPropModelInterfacePtr,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const RandomNumberGeneratorSeedType& nodeSeed)
    {
        shared_ptr<AlohaMac> macPtr(
            new AlohaMac(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                initPropModelInterfacePtr,
                nodeId,
                interfaceId,
                networkLayerPtr,
                nodeSeed));

        macPtr->CompleteInitialization(theParameterDatabaseReader, nodeId, interfaceId);
        return macPtr;
    }//Create//

    ~AlohaMac();

    // Network Layer Interface:

    void NetworkLayerQueueChangeNotification();

    void DisconnectFromOtherLayers()
    {
        theMacAddressResolverPtr.reset();
        networkLayerPtr.reset();
        physicalLayer.DisconnectFromOtherObjects();

    }//DisconnectFromOtherLayers//

    // Physical Layer Interface:

    void ReceiveFrameFromPhy(const Packet& aFrame);

    void TransmissionIsCompleteNotification();

private:
    bool initializationIsComplete;

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    NodeIdType nodeId;
    InterfaceIdType interfaceId;
    shared_ptr<NetworkLayer> networkLayerPtr;
    unsigned int interfaceIndex;

    static const int SEED_HASH = 20110825;
    RandomNumberGenerator aRandomNumberGenerator;

    shared_ptr<FifoInterfaceOutputQueue> networkOutputQueuePtr;

    unique_ptr<Packet> currentDataPacketPtr;

    //-----------------------------------------------------

    enum AlohaFrameType {DATA_FRAME_TYPE = 0, ACK_FRAME_TYPE = 1};

    struct FrameHeaderType {
        unsigned int sequenceNumber;
        MacAddressType transmitterAddress;
        MacAddressType receiverAddress;
        unsigned char frameType:1;
        unsigned char padding:7;

        FrameHeaderType()
            : sequenceNumber(0), frameType(0), padding(0)
        { }

        FrameHeaderType(const unsigned char initFrameType)
            : sequenceNumber(0), frameType(initFrameType), padding(0)
        { }

        FrameHeaderType(
            const unsigned char initFrameType,
            const MacAddressType initReceiverAddress,
            const MacAddressType initTransmitterAddress,
            const unsigned int initSequenceNumber)
            :
            sequenceNumber(initSequenceNumber),
            transmitterAddress(initTransmitterAddress),
            receiverAddress(initReceiverAddress),
            frameType(initFrameType),
            padding(0)
        { }

    };//FrameHeaderType//

    //-----------------------------------------------------

    unsigned int currentFrameSequenceNumber;
    FrameHeaderType currentDataFrameHeader;

    queue<unique_ptr<Packet> > ackFramePtrs;

    DatarateBitsPerSecType datarateBitsPerSec;
    double transmissionPowerDbm;

    TimeType maximumRetryInterval;
    TimeType minimumRetryInterval;

    int retryLimit;
    int currentRetryCount;

    enum MacStateType {
        IDLE_STATE,

        DATA_STATE,
        ACK_STATE,

        DATA_TRANSMITTING_STATE,
        ACK_TRANSMITTING_STATE,
    };//MacStateType//

    MacAddressType myMacAddress;
    MacStateType macState;
    shared_ptr<MacAddressResolver<MacAddressType> > theMacAddressResolverPtr;

    enum AlohaModelType { UNSLOTTED, SLOTTED };
    AlohaModelType alohaModel;

    TimeType slotDuration;

    AlohaPhy physicalLayer;

    enum AlohaTransmissionType { DATA_TRANSMISSION, ACK_TRANSMISSION };

    bool transmitCurrentDataAfterAckTransmission;

    map<MacAddressType, unsigned int> lastSequenceNumberMap;

    TimeType minDataTransmissionDuration;
    TimeType maxDataTransmissionJitter;

    TimeType currentMinDataTransmissionDuration;
    TimeType lastDataTransmissionEndTime;

    //-------------------------------------------------------------------------

    AlohaMac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<PropagationInterfaceType>& initPropModelInterfacePtr,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const RandomNumberGeneratorSeedType& nodeSeed);

    void CompleteInitialization(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId);

    TimeType GetStartTimeOfNextSlotAtAnInstant(const TimeType& arbitartyTime) const;
    TimeType GetNextFrameTransmissionTime(const AlohaTransmissionType& transmissionType) const;

    TimeType GenerateRandomDuration(
        const TimeType& minimumDuration,
        const TimeType& maximumDuration);

    TimeType GetRetryInterval();

    void ScheduleDataFrameTransmissionTimerEvent(const TimeType& wakeupTime);
    void ProcessDataFrameTransmissionTimerEvent();
    void TransmitDataOrWaitAckTransmissionEndIfAckState();

    void TransmitNextDataFrameIfNecessary();
    void TransmitCurrentDataFrame();
    void EndCurrentDataTransmission();

    void TransmitAcknowledgement();

    void RetrievePacketFromNetworkLayer(bool& wasRetrieved);

    bool HasAckToSend() const { return (!ackFramePtrs.empty()); }
    bool HasDataPacketToSend() const { return (currentDataPacketPtr != nullptr); }
    bool HasTransmissionWaitingData() const;

    bool FrameWithThisAddressIsForThisNode(const MacAddressType& theMacAddress) const;

    void ProcessReceivedAcknowledgementFrame(const Packet& aFrame);
    void ProcessReceivedDataFrame(const Packet& aFrame);

    void RememberNewSequenceNumber(
        const MacAddressType& transmitterAddress,
        const unsigned int sequenceNumber);

    bool HaveAlreadySeenThisPacketBefore(
        const MacAddressType& transmitterAddress,
        const unsigned int sequenceNumber) const;

    //-------------------------------------------------------------------------


    class DataFrameTransmissionTimerEvent : public SimulationEvent {
    public:
        DataFrameTransmissionTimerEvent(const shared_ptr<AlohaMac>& initMacPtr) : macPtr(initMacPtr) { }
        void ExecuteEvent() { macPtr->ProcessDataFrameTransmissionTimerEvent(); }
    private:
        shared_ptr<AlohaMac> macPtr;
    };//DataFrameTransmissionTimerEvent//

    shared_ptr<DataFrameTransmissionTimerEvent> dataFrameTransmissoinTimerEventPtr;
    EventRescheduleTicket dataFrameTransmissionTimerEventTicket;

    //-------------------------------------------------------------------------

    class AcknowledgementFrameTransmissionTimerEvent : public SimulationEvent {
    public:
        AcknowledgementFrameTransmissionTimerEvent(const shared_ptr<AlohaMac> & initMacPtr) : macPtr(initMacPtr) { }
        void ExecuteEvent() { macPtr->TransmitAcknowledgement(); }
    private:
        shared_ptr<AlohaMac> macPtr;
    };//AcknowledgementFrameTransmissionTimerEvent//

    shared_ptr<AcknowledgementFrameTransmissionTimerEvent> acknowledgementFrameTransmissionTimerEventPtr;


    //-------------------------------------------------------------------------

    // Trace and Statistics

    shared_ptr<CounterStatistic> dequeuedPacketsStatPtr;
    shared_ptr<CounterStatistic> droppedPacketsStatPtr;// For packets dropped at 'retries limit' expiration.

    shared_ptr<CounterStatistic> dataFramesSentStatPtr;
    shared_ptr<CounterStatistic> dataFramesReceivedStatPtr;

    shared_ptr<CounterStatistic> ackFramesSentStatPtr;
    shared_ptr<CounterStatistic> ackFramesReceivedStatPtr;


    void OutputTraceAndStatsForPacketDequeue() const;
    void OutputTraceAndStatsForFrameReceive(const Packet& aFrame) const;

    void OutputTraceAndStatsForDataFrameTransmission() const;
    void OutputTraceForAckFrameTransmission() const;

    void OutputTraceAndStatsForPacketRetriesExceeded() const;

    //-------------------------------------------------------------------------

    // Disable:
    AlohaMac(const AlohaMac&);
    void operator=(const AlohaMac&);

};//AlohaMac//


#pragma warning(push)
#pragma warning(disable:4355)

inline
AlohaMac::AlohaMac(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<PropagationInterfaceType>& initPropModelInterfacePtr,
    const NodeIdType& initNodeId,
    const InterfaceIdType& initInterfaceId,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const RandomNumberGeneratorSeedType& nodeSeed)
    :
    initializationIsComplete(false),
    simEngineInterfacePtr(simulationEngineInterfacePtr),
    nodeId(initNodeId),
    interfaceId(initInterfaceId),
    networkLayerPtr(initNetworkLayerPtr),
    interfaceIndex(initNetworkLayerPtr->LookupInterfaceIndex(initInterfaceId)),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(nodeSeed, initInterfaceId, SEED_HASH)),
    networkOutputQueuePtr(
        new FifoInterfaceOutputQueue(
            theParameterDatabaseReader,
            initInterfaceId,
            simulationEngineInterfacePtr)),
    currentDataPacketPtr(nullptr),
    currentFrameSequenceNumber(0),
    datarateBitsPerSec(0),
    transmissionPowerDbm(-DBL_MAX),
    maximumRetryInterval(ZERO_TIME),
    minimumRetryInterval(ZERO_TIME),
    retryLimit(0),
    currentRetryCount(0),
    macState(IDLE_STATE),
    alohaModel(UNSLOTTED),
    slotDuration(ZERO_TIME),
    physicalLayer(
        simulationEngineInterfacePtr,
        initPropModelInterfacePtr,
        this,
        theParameterDatabaseReader,
        initNodeId,
        initInterfaceId,
        nodeSeed),
    transmitCurrentDataAfterAckTransmission(false),
    minDataTransmissionDuration(ZERO_TIME),
    maxDataTransmissionJitter(ZERO_TIME),
    currentMinDataTransmissionDuration(ZERO_TIME),
    lastDataTransmissionEndTime(ZERO_TIME),
    dequeuedPacketsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_DequeuedPackets"))),
    droppedPacketsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_DroppedPackets"))),
    dataFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_DataFramesSent"))),
    dataFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_DataFramesReceived"))),
    ackFramesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_AckFramesSent"))),
    ackFramesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_AckFramesReceived")))
{
    networkLayerPtr->SetInterfaceOutputQueue(interfaceIndex, networkOutputQueuePtr);
    myMacAddress.SetLowerBitsWithNodeId(nodeId);

    // Read parameters from configuration file.

    const string alohaModelInString = MakeLowerCaseString(
        theParameterDatabaseReader.ReadString("aloha-model", initNodeId, initInterfaceId));

    if (alohaModelInString.find("unslotted") != string::npos) {
        alohaModel = UNSLOTTED;
    }
    else if (alohaModelInString.find("slotted") != string::npos) {

        alohaModel = SLOTTED;

        slotDuration =
            theParameterDatabaseReader.ReadTime("aloha-slot-time", nodeId, interfaceId);

    }
    else {
        cerr << "Invalid value (aloha-model): "
             << alohaModelInString << endl;
        exit(1);
    }//if//


    datarateBitsPerSec =
        theParameterDatabaseReader.ReadInt("aloha-datarate-bits-per-second", nodeId, interfaceId);

    if (datarateBitsPerSec <= 0) {
        cerr << "Invalid value (aloha-datarate-bits-per-second): "
             << datarateBitsPerSec << endl;
        exit(1);
    }//if//


    transmissionPowerDbm =
        theParameterDatabaseReader.ReadDouble("aloha-tx-power-dbm", nodeId, interfaceId);


    maximumRetryInterval =
        theParameterDatabaseReader.ReadTime("aloha-maximum-retry-interval", nodeId, interfaceId);

    if (maximumRetryInterval <= ZERO_TIME) {
        cerr << "Invalid value (aloha-maximum-retry-interval): "
             << maximumRetryInterval << endl;
        exit(1);
    }//if//


    minimumRetryInterval =
        theParameterDatabaseReader.ReadTime("aloha-minimum-retry-interval", nodeId, interfaceId);

    if (minimumRetryInterval <= ZERO_TIME) {
        cerr << "Invalid value (aloha-minimum-retry-interval): "
             << minimumRetryInterval << endl;
        exit(1);
    }
    else if (minimumRetryInterval > maximumRetryInterval) {
        cerr << "aloha-maximum-retry-interval is smaller than aloha-minimum-retry-interval." << endl;
        exit(1);
    }//if//

    retryLimit =
        theParameterDatabaseReader.ReadInt("aloha-retry-limit", nodeId, interfaceId);

    if (retryLimit < 0) {
        cerr << "Invalid value (aloha-retry-limit): "
             << retryLimit << endl;
        exit(1);
    }//if//

    minDataTransmissionDuration =
        theParameterDatabaseReader.ReadTime("aloha-minimum-data-transmission-interval", nodeId, interfaceId);

    maxDataTransmissionJitter =
        theParameterDatabaseReader.ReadTime("aloha-maximum-data-transmission-jitter", nodeId, interfaceId);

    lastDataTransmissionEndTime = -minDataTransmissionDuration;

    const unsigned int initialChannelNumber = 0;

    initPropModelInterfacePtr->SwitchToChannelNumber(initialChannelNumber);

}//AlohaMac//

#pragma warning(pop)


inline
void AlohaMac::CompleteInitialization(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId)
{
    theMacAddressResolverPtr.reset(
        new SimpleMacAddressResolver(shared_from_this()));

    dataFrameTransmissoinTimerEventPtr.reset(
        new DataFrameTransmissionTimerEvent(shared_from_this()));

    acknowledgementFrameTransmissionTimerEventPtr.reset(
        new AcknowledgementFrameTransmissionTimerEvent(shared_from_this()));

    initializationIsComplete = true;

}//CompleteInitialization//


inline
AlohaMac::~AlohaMac()
{
    assert(initializationIsComplete);

    if (currentDataPacketPtr != nullptr) {
        currentDataPacketPtr = nullptr;
    }//if//

    while (ackFramePtrs.empty()) {
        ackFramePtrs.pop();
    }//while//

}//~AlohaMac//


inline
TimeType AlohaMac::GetStartTimeOfNextSlotAtAnInstant(const TimeType& arbitraryTime) const
{
    return (arbitraryTime
            + ((*this).slotDuration - arbitraryTime % (*this).slotDuration));

}//GetStartTimeOfNextSlotAtAnInstant//

inline
TimeType AlohaMac::GetNextFrameTransmissionTime(const AlohaTransmissionType& transmissionType) const
{
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    TimeType necessaryWaitTime;

    if (transmissionType == DATA_TRANSMISSION) {
        const TimeType elapsedTime = currentTime - lastDataTransmissionEndTime;

        necessaryWaitTime = std::max(ZERO_TIME, (currentMinDataTransmissionDuration - elapsedTime));
    }
    else {
        necessaryWaitTime = ZERO_TIME;
    }//if//

    const TimeType earlyTransmissionTime = simEngineInterfacePtr->CurrentTime() + necessaryWaitTime;

    if (alohaModel == UNSLOTTED) {

        return earlyTransmissionTime;
    }
    else if (alohaModel == SLOTTED) {
        return (*this).GetStartTimeOfNextSlotAtAnInstant(earlyTransmissionTime);
    }
    else {
        cerr << "Cannot determine data frame transmission time for invalid value (aloha-model)." << endl;
        exit(1);
    }//if//

}//GetNextFrameTransmissionTime//

inline
TimeType AlohaMac::GenerateRandomDuration(
    const TimeType& minimumDuration,
    const TimeType& maximumDuration)
{
    assert(minimumDuration <= maximumDuration);

    if (maximumDuration == minimumDuration) {
        return minimumDuration;
    }
    else {

        const TimeType randomDuration =
            (minimumDuration
             + ConvertDoubleSecsToTime(
                 ConvertTimeToDoubleSecs(maximumDuration - minimumDuration)
                 * (*this).aRandomNumberGenerator.GenerateRandomDouble()));

        return randomDuration;

    }//if//

}//GenerateRandomDuration//


inline
TimeType AlohaMac::GetRetryInterval()
{
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    const TimeType elapsedTime = currentTime - lastDataTransmissionEndTime;
    const TimeType necessaryWaitTime = std::max(ZERO_TIME, (currentMinDataTransmissionDuration - elapsedTime));

    const TimeType exactRetryInterval =
        (*this).GenerateRandomDuration(
            std::max((*this).minimumRetryInterval, necessaryWaitTime),
            std::max((*this).maximumRetryInterval, necessaryWaitTime));

    if (alohaModel == UNSLOTTED) {
        return exactRetryInterval;
    }
    else if (alohaModel == SLOTTED) {

        const TimeType retryTime = currentTime + exactRetryInterval;
        const TimeType slottedRetryInterval =
            (*this).GetStartTimeOfNextSlotAtAnInstant(retryTime) - currentTime;

        return slottedRetryInterval;

    }
    else {
        cerr << "Cannot determine retry interval for invalid value (aloha-model)." << endl;
        exit(1);
    }//if//

}//GetRetryInterval//




inline
void AlohaMac::TransmitAcknowledgement()
{
    assert((*this).HasAckToSend());
    assert(macState == ACK_STATE);

    (*this).OutputTraceForAckFrameTransmission();

    // Send to physical layer, do not wait for acknowledgement.

    assert(!ackFramePtrs.empty());
    unique_ptr<Packet> ackFramePtr(move(ackFramePtrs.front()));
    ackFramePtrs.pop();

    const TimeType frameDuration =
        physicalLayer.CalculateFrameTransmitDuration(
            ackFramePtr->LengthBytes(),
            (*this).datarateBitsPerSec);

    physicalLayer.TransmitAFrame(
        ackFramePtr,
        (*this).datarateBitsPerSec,
        (*this).transmissionPowerDbm,
        frameDuration);

    macState = ACK_TRANSMITTING_STATE;
}//SendAcknowledgement//

inline
bool AlohaMac::FrameWithThisAddressIsForThisNode(const MacAddressType& theMacAddress) const
{
    return ((theMacAddress == this->myMacAddress) ||
            (theMacAddress.IsABroadcastOrAMulticastAddress()));

}//FrameWithThisAddressIsForThisNode//

inline
void AlohaMac::EndCurrentDataTransmission()
{
    if (!dataFrameTransmissionTimerEventTicket.IsNull()) {
        simEngineInterfacePtr->CancelEvent(dataFrameTransmissionTimerEventTicket);
    }//if//

    (*this).currentDataPacketPtr = nullptr;
    (*this).currentRetryCount = 0;

}//ProcessReceivedAcknowledgementFrame//

inline
void AlohaMac::ReceiveFrameFromPhy(const Packet& aFrame)
{
    const FrameHeaderType& header = aFrame.GetAndReinterpretPayloadData<FrameHeaderType>();

    if (!(*this).FrameWithThisAddressIsForThisNode(header.receiverAddress)) {
        // Frame not bound for this node ==> ignore.
        return;
    }//if//

    // Process frame bound for this node.

    (*this).OutputTraceAndStatsForFrameReceive(aFrame);

    switch (header.frameType) {
    case DATA_FRAME_TYPE: {
        (*this).ProcessReceivedDataFrame(aFrame);
        break;
    }
    case ACK_FRAME_TYPE: {
        (*this).ProcessReceivedAcknowledgementFrame(aFrame);
        break;
    }
    default:
        assert(false && "Unhandled Case"); abort(); break;
    }//switch//

}//ReceiveFrameFromPhy//

inline
void AlohaMac::ProcessReceivedDataFrame(const Packet& aFrame)
{
    assert(macState != ACK_TRANSMITTING_STATE &&
           macState != DATA_TRANSMITTING_STATE);

    if ((alohaModel == SLOTTED) && (macState == ACK_STATE)) {
        cerr << "Error: Slotted Aloha is not synchronized or received huge Aloha frame exceeding max frame size." << endl;
        exit(1);
    }//if//

    const FrameHeaderType& header = aFrame.GetAndReinterpretPayloadData<FrameHeaderType>();

    bool wasFound;
    NetworkAddress lastHopAddress;
    theMacAddressResolverPtr->GetNetworkAddressIfAvailable(
        header.transmitterAddress,
        networkLayerPtr->GetSubnetAddress(interfaceIndex),
        wasFound,
        lastHopAddress);

    assert(wasFound || (lastHopAddress == NetworkAddress::invalidAddress));

    unique_ptr<Packet> dataPacketPtr(new Packet(aFrame));
    dataPacketPtr->DeleteHeader(sizeof(FrameHeaderType));

    if (!header.receiverAddress.IsABroadcastAddress()) {
        unique_ptr<Packet> ackFramePtr(new Packet(*dataPacketPtr));
        const FrameHeaderType ackFrameHeader(
                ACK_FRAME_TYPE,
                header.transmitterAddress, // Recipient of this Acknowledgement
                myMacAddress,
                header.sequenceNumber);

        ackFramePtr->AddPlainStructHeader(ackFrameHeader);

        ackFramePtrs.push(move(ackFramePtr));

        if ((macState != ACK_TRANSMITTING_STATE) &&
            (macState != DATA_TRANSMITTING_STATE)) {

            macState = ACK_STATE;

            (*this).TransmitDataOrWaitAckTransmissionEndIfAckState();

            const TimeType ackTransmissionTime = (*this).GetNextFrameTransmissionTime(ACK_TRANSMISSION);

            simEngineInterfacePtr->ScheduleEvent(
                acknowledgementFrameTransmissionTimerEventPtr,
                ackTransmissionTime);
        }//if//
    }//if//

    if (!(*this).HaveAlreadySeenThisPacketBefore(header.transmitterAddress, header.sequenceNumber)) {
        (*this).RememberNewSequenceNumber(header.transmitterAddress, header.sequenceNumber);
        networkLayerPtr->ReceivePacketFromMac(interfaceIndex, dataPacketPtr, lastHopAddress);
    }
    else {
        dataPacketPtr = nullptr;
    }//if//

}//ProcessReceivedDataFrame//

inline
void AlohaMac::ProcessReceivedAcknowledgementFrame(const Packet& aFrame)
{
    const FrameHeaderType& header = aFrame.GetAndReinterpretPayloadData<FrameHeaderType>();

    if (header.sequenceNumber != (*this).currentFrameSequenceNumber) {
        // Received an ACK, but was expecting another ==> ignore.
        return;
    }//if//

    (*this).EndCurrentDataTransmission();

    if (macState == DATA_STATE) {
        // Current unicast packet is acked.

        macState = IDLE_STATE;

        (*this).TransmitNextDataFrameIfNecessary();
    }
    else if (macState == ACK_STATE ||
             macState == DATA_TRANSMITTING_STATE ||
             macState == ACK_TRANSMITTING_STATE ||
             macState == IDLE_STATE) {
        // nothing to do.
    }//if//

}//ProcessReceivedAcknowledgementFrame//

inline
void AlohaMac::RememberNewSequenceNumber(
    const MacAddressType& transmitterAddress,
    const unsigned int sequenceNumber)
{
    lastSequenceNumberMap[transmitterAddress] = sequenceNumber;
}

inline
bool AlohaMac::HaveAlreadySeenThisPacketBefore(
    const MacAddressType& transmitterAddress,
    const unsigned int sequenceNumber) const
{
    typedef map<MacAddressType, unsigned int>::const_iterator IterType;

    IterType iter = lastSequenceNumberMap.find(transmitterAddress);

    if ((iter == lastSequenceNumberMap.end()) || (sequenceNumber != iter->second)) {
        return false;
    }//if//

    return true;

}//HaveAlreadySeenThisPacketBefore//

inline
bool AlohaMac::HasTransmissionWaitingData() const
{
    return ((*this).HasDataPacketToSend() &&
            dataFrameTransmissionTimerEventTicket.IsNull());
}//HasTransmissionWaitingData//

inline
void AlohaMac::TransmissionIsCompleteNotification()
{
    // Callback for PHY's TxEnd process

    assert((macState == DATA_TRANSMITTING_STATE) ||
           (macState == ACK_TRANSMITTING_STATE));

    if (macState == DATA_TRANSMITTING_STATE) {
        lastDataTransmissionEndTime = simEngineInterfacePtr->CurrentTime();

        const TimeType jitter =
            (*this).GenerateRandomDuration(ZERO_TIME, maxDataTransmissionJitter);

        currentMinDataTransmissionDuration = minDataTransmissionDuration + jitter;

        if (currentDataFrameHeader.receiverAddress.IsABroadcastAddress()) {

            (*this).EndCurrentDataTransmission();
        }
        else {

            if ((*this).currentRetryCount < retryLimit) {

                const TimeType retryInterval = (*this).GetRetryInterval();
                const TimeType dataRetransmissionTime = simEngineInterfacePtr->CurrentTime() + retryInterval;

                //schedule retry event (when receiving ack, the event is cancelled)

                (*this).ScheduleDataFrameTransmissionTimerEvent(dataRetransmissionTime);
                (*this).currentRetryCount++;
            }
            else {

                (*this).OutputTraceAndStatsForPacketRetriesExceeded();
                (*this).EndCurrentDataTransmission();
            }//if//
        }//if//
    }
    else if (macState == ACK_TRANSMITTING_STATE) {
        // Do Nothing

    }//if//

    if ((*this).HasAckToSend()) {
        macState = ACK_STATE;

        const TimeType ackTransmissionTime = (*this).GetNextFrameTransmissionTime(ACK_TRANSMISSION);

        simEngineInterfacePtr->ScheduleEvent(
            acknowledgementFrameTransmissionTimerEventPtr,
            ackTransmissionTime);
    }
    else if ((*this).HasDataPacketToSend()) {

        if ((macState == ACK_TRANSMITTING_STATE) && transmitCurrentDataAfterAckTransmission) {
            macState = DATA_STATE;

            transmitCurrentDataAfterAckTransmission = false;

            const TimeType dataRetransmissionTime = (*this).GetNextFrameTransmissionTime(DATA_TRANSMISSION);
            (*this).ScheduleDataFrameTransmissionTimerEvent(dataRetransmissionTime);
        }
        else {
            macState = DATA_STATE;

            // unicast packet retransmission
            assert(!currentDataFrameHeader.receiverAddress.IsABroadcastAddress());
            assert(!dataFrameTransmissionTimerEventTicket.IsNull());

        }//if//
    }
    else {
        macState = IDLE_STATE;

        // If more packets in Network-layer Queue, try sending recursively.
        (*this).TransmitNextDataFrameIfNecessary();
    }//if//

}//TransmissionIsCompleteNotification//

inline
void AlohaMac::ProcessDataFrameTransmissionTimerEvent()
{
    dataFrameTransmissionTimerEventTicket.Clear();

    (*this).TransmitDataOrWaitAckTransmissionEndIfAckState();

}//ProcessDataFrameTransmissionTimerEvent//

inline
void AlohaMac::TransmitDataOrWaitAckTransmissionEndIfAckState()
{
    if (macState == DATA_STATE) {
        (*this).TransmitCurrentDataFrame();
    }
    else if (macState == ACK_STATE ||
             macState == DATA_TRANSMITTING_STATE ||
             macState == ACK_TRANSMITTING_STATE) {

        // transmit data frame after ack transmission
        transmitCurrentDataAfterAckTransmission = true;
    }
    else {
        assert(false && "Undefined action for IDLE_STATE"); abort();
    }//if//

}//TransmitDataOrWaitAckTransmissionEndIfAckState//

inline
void AlohaMac::TransmitCurrentDataFrame()
{
    assert((*this).HasDataPacketToSend());
    assert(macState == DATA_STATE);

    unique_ptr<Packet> dataFramePtr(new Packet(*((*this).currentDataPacketPtr)));

    (*this).currentDataFrameHeader.sequenceNumber = (*this).currentFrameSequenceNumber;

    // Note: receiver's address is inserted only once
    //       right after dequeueing the packet from network layer
    (*this).currentDataFrameHeader.transmitterAddress = (*this).myMacAddress;

    dataFramePtr->AddPlainStructHeader((*this).currentDataFrameHeader);

    (*this).OutputTraceAndStatsForDataFrameTransmission();

    const TimeType frameDuration =
        physicalLayer.CalculateFrameTransmitDuration(
            dataFramePtr->LengthBytes(),
            (*this).datarateBitsPerSec);

    physicalLayer.TransmitAFrame(
        dataFramePtr,
        (*this).datarateBitsPerSec,
        (*this).transmissionPowerDbm,
        frameDuration);

    macState = DATA_TRANSMITTING_STATE;

}//TransmitCurrentDataFrame//

inline
void AlohaMac::ScheduleDataFrameTransmissionTimerEvent(const TimeType& wakeupTime)
{
    if (!dataFrameTransmissionTimerEventTicket.IsNull()) {
        simEngineInterfacePtr->CancelEvent(dataFrameTransmissionTimerEventTicket);
    }//if//

    assert(macState != IDLE_STATE);
    assert(dataFrameTransmissionTimerEventTicket.IsNull());

    simEngineInterfacePtr->ScheduleEvent(
        dataFrameTransmissoinTimerEventPtr,
        wakeupTime,
        dataFrameTransmissionTimerEventTicket);
}//ScheduleDataFrameTransmissionTimerEvent//


inline
void AlohaMac::RetrievePacketFromNetworkLayer(bool& wasRetrieved)
{
    assert(macState == IDLE_STATE);
    assert(!(*this).HasDataPacketToSend());

    wasRetrieved = false;

    if (networkOutputQueuePtr->IsEmpty()) {
        return;
    }//if//

    NetworkAddress nextHopAddress;
    PacketPriorityType notUsed1;
    ScenSim::EtherTypeFieldType notUsed2;
    networkOutputQueuePtr->DequeuePacket(
        (*this).currentDataPacketPtr,
        nextHopAddress,
        notUsed1,
        notUsed2);

    bool receiverAddressIsResolved;
    theMacAddressResolverPtr->GetMacAddress(
        nextHopAddress,
        networkLayerPtr->GetSubnetMask(interfaceIndex),
        receiverAddressIsResolved,
        currentDataFrameHeader.receiverAddress);

    if (!receiverAddressIsResolved) {
        networkLayerPtr->ReceiveUndeliveredPacketFromMac(
            interfaceIndex,
            (*this).currentDataPacketPtr,
            NetworkAddress());

        (*this).RetrievePacketFromNetworkLayer(wasRetrieved);
        return;
    }//if//

    (*this).OutputTraceAndStatsForPacketDequeue();
    wasRetrieved = true;

}//RetrievePacketFromNetworkLayer//


inline
void AlohaMac::TransmitNextDataFrameIfNecessary()
{
    assert(macState == IDLE_STATE);

    bool wasRetrieved = false;
    (*this).RetrievePacketFromNetworkLayer(wasRetrieved);

    if (!wasRetrieved) {
        return;
    }//if

    (*this).macState = DATA_STATE;
    (*this).currentRetryCount = 0;

    assert(((*this).currentFrameSequenceNumber + 1) < UINT_MAX);
    (*this).currentFrameSequenceNumber++;

    const TimeType dataFrameTransmissionTime = (*this).GetNextFrameTransmissionTime(DATA_TRANSMISSION);
    (*this).ScheduleDataFrameTransmissionTimerEvent(dataFrameTransmissionTime);

}//TransmitAFrameIfNecessary//


inline
void AlohaMac::NetworkLayerQueueChangeNotification()
{
    if (macState == IDLE_STATE) {

        (*this).TransmitNextDataFrameIfNecessary();

    }//if//

}//NetworkLayerQueueChangeNotification//





inline
void AlohaMac::OutputTraceAndStatsForPacketDequeue() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            AlohaMacPacketDequeueTraceRecord traceData;
            const PacketIdType& packetId = (*this).currentDataPacketPtr->GetPacketId();

            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == ALOHA_MAC_PACKET_DEQUEUE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Dequeue", traceData);
        }
        else {
            ostringstream msgStream;

            msgStream << "PktId= " << (*this).currentDataPacketPtr->GetPacketId();
            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Dequeue", msgStream.str());
        }//if//
    }//if//

    (*this).dequeuedPacketsStatPtr->IncrementCounter();

}//OutputTraceAndStatsForPacketDequeue//


inline
void AlohaMac::OutputTraceAndStatsForFrameReceive(const Packet& aFrame) const
{
    const FrameHeaderType& header = aFrame.GetAndReinterpretPayloadData<FrameHeaderType>();

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            AlohaMacFrameReceiveTraceRecord traceData;
            const PacketIdType& packetId = aFrame.GetPacketId();

            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.frameType = header.frameType;

            assert(sizeof(traceData) == ALOHA_MAC_FRAME_RECEIVE_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "Rx_Frame", traceData);

        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << aFrame.GetPacketId();

            msgStream << " FrameType= ";

            switch (header.frameType) {
            case DATA_FRAME_TYPE: {
                msgStream << "DATA";
                break;
            }
            case ACK_FRAME_TYPE: {
                msgStream << "ACK";
                break;
            }
            default:
                assert(false && "Unhandled Case"); abort(); break;
            }//switch//

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Rx_Frame", msgStream.str());

        }//if
    }//if

    switch (header.frameType) {
    case DATA_FRAME_TYPE: {
        (*this).dataFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    case ACK_FRAME_TYPE: {
        (*this).ackFramesReceivedStatPtr->IncrementCounter();
        break;
    }
    default:
        assert(false && "Unhandled Case"); abort(); break;
    }//switch//

}//OutputTraceAndStatsForFrameReceive//


inline
void AlohaMac::OutputTraceAndStatsForDataFrameTransmission() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            AlohaMacTxDataTraceRecord traceData;
            const PacketIdType& packetId = (*this).currentDataPacketPtr->GetPacketId();

            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.retryCount = (*this).currentRetryCount;

            assert(sizeof(traceData) == ALOHA_MAC_TX_DATA_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "Tx_DATA", traceData);

        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << (*this).currentDataPacketPtr->GetPacketId();
            msgStream << " Retry= " << (*this).currentRetryCount;

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Tx_DATA", msgStream.str());

        }//if//
    }//if//

    (*this).dataFramesSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForDataFrameTransmission//


inline
void AlohaMac::OutputTraceForAckFrameTransmission() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {
            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, "Tx_ACK");
        }
        else {
            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Tx_ACK", "");
        }//if//
    }//if//

    (*this).ackFramesSentStatPtr->IncrementCounter();

}//OutputTraceForAckFrameTransmission//


inline
void AlohaMac::OutputTraceAndStatsForPacketRetriesExceeded() const
{
    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            AlohaMacPacketRetryExceededTraceRecord traceData;

            const PacketIdType& packetId = (*this).currentDataPacketPtr->GetPacketId();

            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == ALOHA_MAC_PACKET_RETRY_EXCEEDED_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(
                modelName, interfaceId, "Drop", traceData);

        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << (*this).currentDataPacketPtr->GetPacketId();

            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, "Drop", msgStream.str());

        }//if//
    }//if//

    (*this).droppedPacketsStatPtr->IncrementCounter();

}//OutputTraceAndStatsForPacketRetriesExceeded//


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


inline
shared_ptr<ScenSim::MacLayer> AlohaFactory(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<PropagationInterfaceType>& initPropModelInterfacePtr,
    const NodeIdType& initNodeId,
    const InterfaceIdType& initInterfaceId,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const RandomNumberGeneratorSeedType& nodeSeed)
{
    return (
        AlohaMac::Create(
            theParameterDatabaseReader,
            simulationEngineInterfacePtr,
            initPropModelInterfacePtr,
            initNodeId,
            initInterfaceId,
            initNetworkLayerPtr,
            nodeSeed));

}//AlohaFactory//


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

}//namespace//

#endif
