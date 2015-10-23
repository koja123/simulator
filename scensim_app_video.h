// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_APP_VIDEO_H
#define SCENSIM_APP_VIDEO_H

#include "scensim_netsim.h"

namespace ScenSim {

using std::cerr;
using std::endl;


//--------------------------------------------------------------------------------------------------

class VideoStreamingApplication: public Application {
public:
    static const string modelName;

    typedef MacQualityOfServiceControlInterface::SchedulingSchemeChoice SchedulingSchemeChoice;
    typedef MacQualityOfServiceControlInterface::ReservationSchemeChoice ReservationSchemeChoice;

    // RTP header : 12 bytes, VideoStreaming Payload Size : Conformed Truncated Pareto
    // VideoStreamingPayloadType : 12 bytes
    struct VideoStreamingPayloadType {
        VideoStreamingPayloadType(
            const unsigned int initSequenceNumber,
            const TimeType initSendTime)
            :
            sequenceNumber(initSequenceNumber),
            sendTime(initSendTime)
        {}

        unsigned int sequenceNumber;
        TimeType sendTime;

    };//VideoStreamingtPayloadType//

    //------------------------------------------------------------------------------------

    VideoStreamingApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationIdType& initApplicationId,
        const NodeIdType& initSourceNodeId,
        const NodeIdType& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl);

protected:
    NodeIdType sourceNodeId;
    NodeIdType destinationNodeId;
    unsigned short int destinationPortId;

    unsigned int frameRate;
    unsigned int numberPacketsInAFrame;

    // For packet size
    unsigned int minimumPacketPayloadSizeBytes;
    unsigned int maximumPacketPayloadSizeBytes;
    unsigned int meanPacketPayloadSizeBytes;

    // For inter-arrival time
    TimeType minimumInterArrivalTime;
    TimeType maximumInterArrivalTime;
    TimeType meanInterArrivalTime;

    TimeType videoStreamingStartTime;
    TimeType videoStreamingEndTime;
    PacketPriorityType videoStreamingPriority;
    TimeType maxStartTimeJitter;

    //---------------------------------

    bool reserveBandwidthModeIsOn;
    double qosMinBandwidth;
    double qosMaxBandwidth;

    FlowIdType macQosFlowId;

    SchedulingSchemeChoice schedulingScheme;
    ReservationSchemeChoice reservationScheme;

    bool useVirtualPayload;

    string GetParameterNamePrefix() const {
        if (reserveBandwidthModeIsOn) {
            return  "video-with-qos";
        }//if//
        return "video";
    }

};//VideoStreamingApplication//

inline
VideoStreamingApplication::VideoStreamingApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationIdType& initApplicationId,
    const NodeIdType& initSourceNodeId,
    const NodeIdType& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl)
    :
    Application(initSimulationEngineInterfacePtr, initApplicationId),
    sourceNodeId(initSourceNodeId),
    destinationNodeId(initDestinationNodeId),
    destinationPortId(initDefaultApplicationPortId),
    frameRate(0),
    numberPacketsInAFrame(0),
    minimumPacketPayloadSizeBytes(0),
    maximumPacketPayloadSizeBytes(0),
    meanPacketPayloadSizeBytes(0),
    minimumInterArrivalTime(ZERO_TIME),
    maximumInterArrivalTime(ZERO_TIME),
    meanInterArrivalTime(ZERO_TIME),
    videoStreamingStartTime(ZERO_TIME),
    videoStreamingEndTime(ZERO_TIME),
    videoStreamingPriority(0),
    maxStartTimeJitter(ZERO_TIME),
    reserveBandwidthModeIsOn(initEnableQosControl),
    qosMinBandwidth(0.0),
    qosMaxBandwidth(0.0),
    schedulingScheme(MacQualityOfServiceControlInterface::DefaultSchedulingScheme),
    reservationScheme(MacQualityOfServiceControlInterface::OptimisticLinkRate),
    useVirtualPayload(false)
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    frameRate =
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-frame-rate", sourceNodeId, applicationId);

    numberPacketsInAFrame =
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-number-packets-in-a-frame", sourceNodeId, applicationId);

    minimumPacketPayloadSizeBytes =
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-min-packet-payload-size-bytes", sourceNodeId, applicationId);

    maximumPacketPayloadSizeBytes =
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-max-packet-payload-size-bytes", sourceNodeId, applicationId);

    meanPacketPayloadSizeBytes =
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-mean-packet-size-bytes", sourceNodeId, applicationId);

    minimumInterArrivalTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-min-inter-arrival-time", sourceNodeId, applicationId);

    maximumInterArrivalTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-max-inter-arrival-time", sourceNodeId, applicationId);

    meanInterArrivalTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-mean-inter-arrival-time", sourceNodeId, applicationId);

    videoStreamingStartTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-start-time", sourceNodeId, applicationId);

    videoStreamingEndTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-end-time", sourceNodeId, applicationId);

    videoStreamingPriority = static_cast<PacketPriorityType>(
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-priority", sourceNodeId, applicationId));

    if (parameterDatabaseReader.ParameterExists(
            parameterPrefix + "-start-time-max-jitter", sourceNodeId, applicationId)) {

        maxStartTimeJitter =
            parameterDatabaseReader.ReadTime(
                parameterPrefix + "-start-time-max-jitter", sourceNodeId, applicationId);
    }//if//

    if (parameterDatabaseReader.ParameterExists(
            parameterPrefix + "-auto-port-mode", sourceNodeId, applicationId)) {

        if (!parameterDatabaseReader.ReadBool(
                parameterPrefix + "-auto-port-mode", sourceNodeId, applicationId)) {

            destinationPortId = static_cast<unsigned short int>(
                parameterDatabaseReader.ReadNonNegativeInt(
                    parameterPrefix + "-destination-port", sourceNodeId, applicationId));
        }//if//
    }//if//

    if (initEnableQosControl) {
        qosMinBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-baseline-bandwidth-bytes", sourceNodeId, applicationId);

        qosMaxBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-max-bandwidth-bytes", sourceNodeId, applicationId);

        if (parameterDatabaseReader.ParameterExists(
                parameterPrefix + "-schedule-scheme", sourceNodeId, applicationId)) {

            const string schedulingSchemeChoiceString =
                parameterDatabaseReader.ReadString(
                    parameterPrefix + "-schedule-scheme", sourceNodeId, applicationId);

            bool succeeded;

            ConvertStringToSchedulingSchemeChoice(
                MakeLowerCaseString(schedulingSchemeChoiceString),
                succeeded,
                schedulingScheme);

            if (!succeeded) {
                cerr << "Error in " << modelName << " Application: Scheduling Scheme not recognized in:" << endl;
                cerr << "      >" << schedulingSchemeChoiceString << endl;
                exit(1);
            }//if//
        }//if//
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-use-virtual-payload", sourceNodeId, applicationId)) {

        useVirtualPayload = parameterDatabaseReader.ReadBool(
            parameterPrefix + "-use-virtual-payload", sourceNodeId, applicationId);
    }//if//

}//VideoStreamingApplication//


class VideoStreamingSourceApplication:
    public VideoStreamingApplication, public enable_shared_from_this<VideoStreamingSourceApplication> {
public:
    VideoStreamingSourceApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationIdType& initApplicationId,
        const NodeIdType& initSourceNodeId,
        const NodeIdType& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl,
        const RandomNumberGeneratorSeedType& initNodeSeed);

    void CompleteInitialization();

    virtual ~VideoStreamingSourceApplication() { }

private:
    static const unsigned int START_SEQUENCE_NUMBER = 1;
    static const unsigned int START_FRAME_NUMBER = 1;
    static const unsigned int RTP_HEADER_SIZE_BYTES = 12;
    static const int SEED_HASH = 3415491;

    unsigned int GetPacketPayloadSizeBytesConformUntruncatedPareto();
    unsigned int GetPacketPayloadSizeBytesConformTruncatedPareto();

    TimeType GetInterArrivalTimeConformUntruncatedPareto();
    TimeType GetInterArrivalTimeConformTruncatedPareto();

    void SendFrame();

    void SendPacket(
        const unsigned int frameNumber,
        const unsigned int currentNumberInAFrame);

    void OutputTraceAndStatsForSendPacket(
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const NodeIdType& destinationNodeId,
        const unsigned int packetLengthBytes,
        const bool frameSent);

    class VideoStreamingFrameSendEvent: public SimulationEvent {
    public:
        explicit
        VideoStreamingFrameSendEvent(
            const shared_ptr<VideoStreamingSourceApplication>& initVideoStreamingSourceAppPtr)
            : videoStreamingSourceAppPtr(initVideoStreamingSourceAppPtr)
        { }
        virtual void ExecuteEvent() { videoStreamingSourceAppPtr->SendFrame(); }

    private:
        shared_ptr<VideoStreamingSourceApplication> videoStreamingSourceAppPtr;

    };//VideoStreamingFrameSendEvent//

    class VideoStreamingPacketSendEvent: public SimulationEvent {
    public:
        explicit
        VideoStreamingPacketSendEvent(
            const shared_ptr<VideoStreamingSourceApplication>& initVideoStreamingSourceAppPtr,
            const unsigned int initFrameNumber,
            const unsigned int initCurrentNumberInAFrame)
            :
            videoStreamingSourceAppPtr(initVideoStreamingSourceAppPtr),
            frameNumber(initFrameNumber),
            currentNumberInAFrame(initCurrentNumberInAFrame)
        { }
        virtual void ExecuteEvent() {
            videoStreamingSourceAppPtr->SendPacket(
                frameNumber, currentNumberInAFrame);
        }

    private:
        shared_ptr<VideoStreamingSourceApplication> videoStreamingSourceAppPtr;
        unsigned int frameNumber;
        unsigned int currentNumberInAFrame;

    };//VideoStreamingPacketSendEvent//

    double alphaForPacketPayloadSize;
    double alphaForInterArrivalTime;

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<VideoStreamingSourceApplication>& initAppPtr) : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowIdType& flowId) { appPtr->macQosFlowId = flowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }

    private:
        shared_ptr<VideoStreamingSourceApplication> appPtr;

    };//FlowRequsetReplyFielder//

    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

    //---------------------------------

    RandomNumberGenerator aRandomNumberGenerator;

    unsigned int currentPacketSequenceNumber;
    unsigned int currentFrameNumber;

    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> frameSentStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;

    //Disable
    VideoStreamingSourceApplication();

};//VideoStreamingSourceApplication//


inline
VideoStreamingSourceApplication::VideoStreamingSourceApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationIdType& initApplicationId,
    const NodeIdType& initSourceNodeId,
    const NodeIdType& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl,
    const RandomNumberGeneratorSeedType& initNodeSeed)
    :
    VideoStreamingApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initEnableQosControl),
    alphaForPacketPayloadSize(
        static_cast<double>(meanPacketPayloadSizeBytes) /
        (meanPacketPayloadSizeBytes - minimumPacketPayloadSizeBytes)),
    alphaForInterArrivalTime(
        ConvertTimeToDoubleSecs(meanInterArrivalTime) /
        ConvertTimeToDoubleSecs(meanInterArrivalTime - minimumInterArrivalTime)),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initNodeSeed, initApplicationId, SEED_HASH)),
    currentPacketSequenceNumber(START_SEQUENCE_NUMBER),
    currentFrameNumber(START_FRAME_NUMBER),
    packetsSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_PacketsSent"))),
    frameSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_FramesSent"))),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesSent")))
{
    if (minimumPacketPayloadSizeBytes < (int)(sizeof(VideoStreamingPayloadType) - RTP_HEADER_SIZE_BYTES)) {
        cerr << "Error: Video Streaming Application minimum payload size ("
            << minimumPacketPayloadSizeBytes << ") should be "
            << (sizeof(VideoStreamingPayloadType) - RTP_HEADER_SIZE_BYTES) << " bytes or larger." << endl;
        exit(1);
    }//if//

}//VideoStreamingSourceApplication//

// Two part initialization forced by shared_from_this().

inline
void VideoStreamingSourceApplication::CompleteInitialization()
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    if (maxStartTimeJitter != ZERO_TIME) {
        videoStreamingStartTime += static_cast<TimeType>(
            aRandomNumberGeneratorPtr->GenerateRandomDouble() * maxStartTimeJitter);
    }//if//

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    const TimeType minimumSetupTime = 1 * MILLI_SECOND;

    if (videoStreamingStartTime < (currentTime + minimumSetupTime)) {
        videoStreamingStartTime = (currentTime + minimumSetupTime);
    }//if//

    if (videoStreamingStartTime < videoStreamingEndTime) {

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new VideoStreamingFrameSendEvent(shared_from_this())),
            videoStreamingStartTime);

    }//if//

}//CompleteInitialization//


inline
unsigned int VideoStreamingSourceApplication::GetPacketPayloadSizeBytesConformUntruncatedPareto()
{
    const double randomNumber = aRandomNumberGenerator.GenerateRandomDouble();

    return static_cast<unsigned int>(
        minimumPacketPayloadSizeBytes / pow((1.0 - randomNumber), (1.0 / alphaForPacketPayloadSize)));

}//GetPacketPayloadSizeBytesConformUntruncatedParete//


inline
unsigned int VideoStreamingSourceApplication::GetPacketPayloadSizeBytesConformTruncatedPareto()
{
    unsigned int result;

    while(true) {
        result = (*this).GetPacketPayloadSizeBytesConformUntruncatedPareto();

        if (result <= maximumPacketPayloadSizeBytes) {
            break;
        }
    }

    return result;

}//GetPacketPayloadSizeBytesConformTruncatedParete//


inline
TimeType VideoStreamingSourceApplication::GetInterArrivalTimeConformUntruncatedPareto()
{
    const double randomNumber = aRandomNumberGenerator.GenerateRandomDouble();
    const double minimumInterArrivalTimeSecs = ConvertTimeToDoubleSecs(minimumInterArrivalTime);

    return ConvertDoubleSecsToTime(
        minimumInterArrivalTimeSecs / pow((1.0 - randomNumber), (1.0 / alphaForInterArrivalTime)));

}//GetInterArrivalTimeConformUntruncatedParete//


inline
TimeType VideoStreamingSourceApplication::GetInterArrivalTimeConformTruncatedPareto()
{
    TimeType result;

    while(true) {
        result = (*this).GetInterArrivalTimeConformUntruncatedPareto();

        if (result <= maximumInterArrivalTime) {
            break;
        }
    }

    return result;

}//GetInterArrivalTimeConformTruncatedParete//


inline
void VideoStreamingSourceApplication::SendFrame()
{

    //start with pakcet number 1 in a frame
    SendPacket(currentFrameNumber, 1);

    const TimeType nextFrameSendTime =
        simulationEngineInterfacePtr->CurrentTime() + ConvertDoubleSecsToTime(1.0 / frameRate);

    if (nextFrameSendTime < videoStreamingEndTime) {

        currentFrameNumber++;

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new VideoStreamingFrameSendEvent(shared_from_this())),
            nextFrameSendTime);
    }//if//

}//SendFrame//


inline
void VideoStreamingSourceApplication::SendPacket(
    const unsigned int frameNumber,
    const unsigned int currentNumberInAFrame)
{
    if ((currentPacketSequenceNumber == START_SEQUENCE_NUMBER) && (reserveBandwidthModeIsOn)) {
        (*this).ReserveBandwidth();
    }//if//

    const VideoStreamingPayloadType videoStreamingAppPayload(
        currentPacketSequenceNumber,
        simulationEngineInterfacePtr->CurrentTime());

    const unsigned int packetPayloadSizeBytes =
        RTP_HEADER_SIZE_BYTES + GetPacketPayloadSizeBytesConformTruncatedPareto();

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(
            *simulationEngineInterfacePtr,
            videoStreamingAppPayload,
            packetPayloadSizeBytes,
            useVirtualPayload);

    bool frameSent = false;
    if (currentNumberInAFrame == numberPacketsInAFrame) {
        frameSent = true;
    }//if//

    OutputTraceAndStatsForSendPacket(
        currentPacketSequenceNumber,
        packetPtr->GetPacketId(),
        destinationNodeId,
        packetPayloadSizeBytes,
        frameSent);

    const NetworkAddress sourceAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(sourceNodeId);

    NetworkAddress destAddress;
    bool foundDestAddress;
    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        destinationNodeId, destAddress, foundDestAddress);

    if (foundDestAddress) {
        transportLayerPtr->udpPtr->SendPacket(
            packetPtr,
            sourceAddress,
            0,
            destAddress,
            destinationPortId,
            videoStreamingPriority);
    }
    else {
        //cannot find destination address (destination node may not be created yet)
        //Future: output trace and stat
        packetPtr = nullptr;
    }//if//

    assert((currentPacketSequenceNumber + 1) < UINT_MAX);
    currentPacketSequenceNumber++;

    if (!frameSent) {

        const TimeType nextPacketSendTime =
            simulationEngineInterfacePtr->CurrentTime() + GetInterArrivalTimeConformTruncatedPareto();

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new VideoStreamingPacketSendEvent(
                shared_from_this(), frameNumber, (currentNumberInAFrame + 1))),
            nextPacketSendTime);

    }
    else {
        if ((frameNumber == currentFrameNumber) && (reserveBandwidthModeIsOn)) {
            //no more frame
            (*this).UnreserveBandwidth();
        }//if//
    }//if//

}//SendPacket//


inline
void VideoStreamingSourceApplication::OutputTraceAndStatsForSendPacket(
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const NodeIdType& destinationNodeId,
    const unsigned int packetLengthBytes,
    const bool frameSent)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;
            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.destinationNodeId = destinationNodeId;
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "VideoSend", traceData);
        }
        else {

            ostringstream outStream;
            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "VideoSend", outStream.str());

        }//if//
    }//if//

    const unsigned int numberPacketsSent = sequenceNumber;

    packetsSentStatPtr->UpdateCounter(numberPacketsSent);
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

    if (frameSent) {
        frameSentStatPtr->IncrementCounter();
    }//if//

}//OutputTraceAndStatsForSendPacket//



inline
void VideoStreamingSourceApplication::ReserveBandwidth()
{
    const NetworkAddress sourceAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(sourceNodeId);

    NetworkAddress destinationAddress;
    bool foundDestAddress;
    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        destinationNodeId, destinationAddress, foundDestAddress);

    if (!foundDestAddress) {
        //Destination node may not be created yet.
        return;
    }//if//

    const NetworkLayer& theNetworkLayer = *transportLayerPtr->GetNetworkLayerPtr();

    bool success;
    unsigned int interfaceIndex;
    NetworkAddress nextHopAddress;
    theNetworkLayer.GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress, success, nextHopAddress, interfaceIndex);

    if ((success) && (theNetworkLayer.MacSupportsQualityOfService(interfaceIndex))) {

        MacQualityOfServiceControlInterface& qosControlInterface =
            *theNetworkLayer.GetMacQualityOfServiceInterface(interfaceIndex);

        shared_ptr<FlowRequestReplyFielder> replyPtr(new FlowRequestReplyFielder(shared_from_this()));

        qosControlInterface.RequestUplinkFlowReservation(
            reservationScheme, schedulingScheme, videoStreamingPriority, qosMinBandwidth, qosMaxBandwidth,
            destinationAddress, destinationPortId, sourceAddress, ANY_PORT, IP_PROTOCOL_NUMBER_UDP,
            replyPtr);
    }//if//

}//ReserveBandwidth//


inline
void VideoStreamingSourceApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in Video with QoS application: Bandwidth request denied." << endl;
}


inline
void VideoStreamingSourceApplication::UnreserveBandwidth()
{
    NetworkAddress destinationAddress;
    bool foundDestAddress;
    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        destinationNodeId, destinationAddress, foundDestAddress);

    if (!foundDestAddress) {
        //Destination node might disappear.
        return;
    }//if//

    const NetworkLayer& theNetworkLayer = *transportLayerPtr->GetNetworkLayerPtr();

    bool success;
    unsigned int interfaceIndex;
    NetworkAddress nextHopAddress;
    theNetworkLayer.GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress, success, nextHopAddress, interfaceIndex);

    if ((success) && (theNetworkLayer.MacSupportsQualityOfService(interfaceIndex))) {

        MacQualityOfServiceControlInterface& qosControlInterface =
            *theNetworkLayer.GetMacQualityOfServiceInterface(interfaceIndex);

        qosControlInterface.DeleteFlow(macQosFlowId);
    }//if//

}//UnreserveBandwidth//



//--------------------------------------------------------------------------------------------------

struct JitterBufferEntry {
    unique_ptr<Packet> packetPtr;
    TimeType bufferedTime;

    JitterBufferEntry() : packetPtr(nullptr), bufferedTime(ZERO_TIME) { }
    JitterBufferEntry(
        unique_ptr<Packet>& initPacketPtr,
        const TimeType& initBufferedTime)
        :
        packetPtr(move(initPacketPtr)), bufferedTime(initBufferedTime)
    {
    }

    JitterBufferEntry(JitterBufferEntry&& right) { (*this) = move(right); }

    void operator=(JitterBufferEntry&& right) {
        bufferedTime = right.bufferedTime;
        packetPtr = move(right.packetPtr);
    }

};//JitterBufferEntry//


class JitterBufferController : public enable_shared_from_this<JitterBufferController> {
public:
    JitterBufferController(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const string& modelName,
        const ApplicationIdType& applicationId,
        const unsigned int frameRate,
        const unsigned int numberPacketsInAFrame,
        const TimeType& jitterBufferWindow,
        const TimeType& streamTrafficStartTime,
        const TimeType& streamTrafficEndTime);

    void Buffering(
        const unsigned int sequenceNumber,
        unique_ptr<Packet>& packetPtr);

    ~JitterBufferController() { }

private:

    static const unsigned int START_SEQUENCE_NUMBER = 1;

    bool decoderIsStarted;
    void DecodeAFrame();

    enum BufferingEventType {
        BUFFERING, DROP_PACKET, BUFFER_OVERFLOW
    };

    enum DecodeEventType {
        DECODE_SUCCESS, DECODE_FAIL, BUFFER_UNDERFLOW
    };

    void OutputTraceAndStatsForBuffering(
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const BufferingEventType& bufferingEventType);

    void OutputTraceAndStatsForDecodeAFrame(
        const DecodeEventType& decodeEventType);


    class DecodeEvent: public SimulationEvent {
    public:
        explicit
        DecodeEvent(
            const shared_ptr<JitterBufferController>& initJitterBufferControllerPtr)
            : jitterBufferControllerPtr(initJitterBufferControllerPtr)
        { }
        virtual void ExecuteEvent() { jitterBufferControllerPtr->DecodeAFrame(); }

    private:
        shared_ptr<JitterBufferController> jitterBufferControllerPtr;

    };//DecodeEvent//

    map<unsigned int, JitterBufferEntry> jitterBuffer;

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    string modelName;
    ApplicationIdType applicationId;

    unsigned int frameRate;
    unsigned int numberPacketsInAFrame;
    TimeType jitterBufferWindow;
    TimeType streamTrafficStartTime;
    TimeType streamTrafficEndTime;

    unsigned int maximumJitterBufferSizePackets;

    TimeType decodeInterval;
    unsigned int expectedDecodeStartSequenceNumber;
    unsigned int expectedDecodeEndSequenceNumber;

    shared_ptr<CounterStatistic> bufferedPacketsStatPtr;
    shared_ptr<CounterStatistic> droppedPacketsStatPtr;

    // "Unbufferd status" has "Overflow status" and "Received disable sequence number packet status".
    // If you need these status, please uncomment "overflowStatPtr".

    //shared_ptr<CounterStatistic> overflowStatPtr;

    shared_ptr<CounterStatistic> successFramesStatPtr;
    shared_ptr<CounterStatistic> failFramesStatPtr;
    // "Fail a frame status" has "Underflow(buffer is dry) status" and "Run out of pakcets status".
    // If you need these status, please uncomment "underflowStatPtr".

    //shared_ptr<CounterStatistic> underflowStatPtr;

};//JitterBufferController//


inline
JitterBufferController::JitterBufferController(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const string& initModelName,
    const ApplicationIdType& initApplicationId,
    const unsigned int initFrameRate,
    const unsigned int initNumberPacketsInAFrame,
    const TimeType& initJitterBufferWindow,
    const TimeType& initStreamTrafficStartTime,
    const TimeType& initStreamTrafficEndTime)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    modelName(initModelName),
    applicationId(initApplicationId),
    decoderIsStarted(false),
    frameRate(initFrameRate),
    numberPacketsInAFrame(initNumberPacketsInAFrame),
    jitterBufferWindow(initJitterBufferWindow),
    streamTrafficStartTime(initStreamTrafficStartTime),
    streamTrafficEndTime(initStreamTrafficEndTime),
    maximumJitterBufferSizePackets(0),
    decodeInterval(ZERO_TIME),
    expectedDecodeStartSequenceNumber(0),
    expectedDecodeEndSequenceNumber(0),
    bufferedPacketsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (initModelName + "_" + initApplicationId + "_PacketsBuffered"))),
    droppedPacketsStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (initModelName + "_" + initApplicationId + "_PacketsUnbuffered"))),
    //     overflowStatPtr(
    //         simulationEngineInterfacePtr->CreateCounterStat(
    //             (initModelName + "_" + initApplicationId + "_Overflow"))),
    successFramesStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (initModelName + "_" + initApplicationId + "_FramesSuccess"))),
    failFramesStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (initModelName + "_" + initApplicationId + "_FramesFailure")))
    //    underflowStatPtr(
    //        simulationEngineInterfacePtr->CreateCounterStat(
    //            (initModelName + "_" + initApplicationId + "_Underflow")))
{

    decodeInterval = ConvertDoubleSecsToTime(1.0 / frameRate);

    const unsigned int jitterBufferSizePackets =
        static_cast<unsigned int>(ceil(numberPacketsInAFrame * (frameRate * ConvertTimeToDoubleSecs(jitterBufferWindow) * 2)));

    //one frame + buffer
    maximumJitterBufferSizePackets = numberPacketsInAFrame + jitterBufferSizePackets;

}//JitterBufferController//


inline
void JitterBufferController::Buffering(
    const unsigned int sequenceNumber,
    unique_ptr<Packet>& packetPtr)
{

    //schedule initial decode event
    if (!decoderIsStarted) {

        decoderIsStarted = true;

        expectedDecodeStartSequenceNumber = START_SEQUENCE_NUMBER;
        expectedDecodeEndSequenceNumber = expectedDecodeStartSequenceNumber + (numberPacketsInAFrame - 1);

        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        const TimeType eventTime = currentTime + decodeInterval + jitterBufferWindow;
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new DecodeEvent(shared_from_this())), eventTime);

    }//if//

    BufferingEventType bufferingEventType;
    if (sequenceNumber < expectedDecodeStartSequenceNumber) {
        bufferingEventType = DROP_PACKET;
        OutputTraceAndStatsForBuffering(sequenceNumber, packetPtr->GetPacketId(), bufferingEventType);
        packetPtr = nullptr;
    }
    else if (jitterBuffer.size() >= maximumJitterBufferSizePackets) {
        bufferingEventType = BUFFER_OVERFLOW;
        OutputTraceAndStatsForBuffering(sequenceNumber, packetPtr->GetPacketId(), bufferingEventType);
        packetPtr = nullptr;
    }
    else {
        bufferingEventType = BUFFERING;
        OutputTraceAndStatsForBuffering(sequenceNumber, packetPtr->GetPacketId(), bufferingEventType);

        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        jitterBuffer[sequenceNumber] = JitterBufferEntry(packetPtr, currentTime);

    }//if//

}//Buffering//


inline
void JitterBufferController::DecodeAFrame()
{

    DecodeEventType decodeEventType;
    if (jitterBuffer.empty()) {

        decodeEventType = BUFFER_UNDERFLOW;

    }
    else {
        map<unsigned int, JitterBufferEntry>::iterator decodeEndIter = jitterBuffer.begin();

        assert(decodeEndIter->first >= expectedDecodeStartSequenceNumber);

        unsigned int numberPacketsInExpectedRange = 0;
        while(decodeEndIter != jitterBuffer.end()) {

            if (decodeEndIter->first > expectedDecodeEndSequenceNumber) {
                break;
            }//if//
            numberPacketsInExpectedRange++;
            decodeEndIter++;

        }//while//

        if (numberPacketsInExpectedRange == numberPacketsInAFrame) {
            decodeEventType = DECODE_SUCCESS;
        }
        else {
            decodeEventType = DECODE_FAIL;
        }//if//

        map<unsigned int, JitterBufferEntry>::iterator jitterBufferIter = jitterBuffer.begin();
        while(jitterBufferIter != jitterBuffer.end() && jitterBufferIter != decodeEndIter) {

            jitterBufferIter->second.packetPtr = nullptr;
            jitterBufferIter++;
        }//while//

        map<unsigned int, JitterBufferEntry>::iterator decodeStartIter = jitterBuffer.begin();
        jitterBuffer.erase(decodeStartIter, decodeEndIter);

    }//if//

    OutputTraceAndStatsForDecodeAFrame(decodeEventType);

    expectedDecodeStartSequenceNumber = expectedDecodeEndSequenceNumber + 1;
    expectedDecodeEndSequenceNumber = expectedDecodeStartSequenceNumber + (numberPacketsInAFrame - 1);

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    const TimeType eventStartTime = currentTime + decodeInterval;

    if (eventStartTime <= streamTrafficEndTime + decodeInterval + jitterBufferWindow) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new DecodeEvent(shared_from_this())), eventStartTime);
    }//if//

}//DecodeAFrame//



inline
void JitterBufferController::OutputTraceAndStatsForBuffering(
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const BufferingEventType& bufferingEventType)
{

    string eventTypeName;
    if (bufferingEventType == BUFFERING) {
        eventTypeName = "BufferSuccess";
        bufferedPacketsStatPtr->IncrementCounter();
    }
    else if (bufferingEventType == DROP_PACKET) {
        eventTypeName = "BufferFailure";
        droppedPacketsStatPtr->IncrementCounter();
    }
    else if (bufferingEventType == BUFFER_OVERFLOW) {
        eventTypeName = "BufferFailure";
        //overflowStatPtr->IncrementCounter();
        droppedPacketsStatPtr->IncrementCounter();
    }
    else {
        assert(false);
        exit(1);
    }//if//

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationBufferTraceRecord traceData;

            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            traceData.packetSequenceNumber = sequenceNumber;

            assert(sizeof(traceData) == APPLICATION_BUFFER_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, eventTypeName, traceData);
        }
        else {
            ostringstream outStream;
            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, eventTypeName, outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForBuffering//


inline
void JitterBufferController::OutputTraceAndStatsForDecodeAFrame(
    const DecodeEventType& decodeEventType)
{

    string eventTypeName;
    if (decodeEventType == DECODE_SUCCESS) {
        eventTypeName = "DecodeSuccess";
        successFramesStatPtr->IncrementCounter();
    }
    else if (decodeEventType == DECODE_FAIL) {
        eventTypeName = "DecodeFailure";
        failFramesStatPtr->IncrementCounter();
    }
    else if (decodeEventType == BUFFER_UNDERFLOW) {
        eventTypeName = "DecodeFailure";
        failFramesStatPtr->IncrementCounter();
        //underflowStatPtr->IncrementCounter();
    }
    else {
        assert(false);
        exit(1);
    }//if//

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {
            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, eventTypeName);
        }
        else {
            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, eventTypeName, "");
        }//if//
    }//if//

}//OutputTraceAndStatsForDecodeAFrame//



//--------------------------------------------------------------------------------------------------

class VideoStreamingSinkApplication:
    public VideoStreamingApplication, public enable_shared_from_this<VideoStreamingSinkApplication> {
public:
    VideoStreamingSinkApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationIdType& initApplicationId,
        const NodeIdType& initSourceNodeId,
        const NodeIdType& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl);

    void CompleteInitialization();

private:

    static const unsigned int START_SEQUENCE_NUMBER = 1;

    void ReceivePacket(unique_ptr<Packet>& packetPtr);

    void OutputTraceAndStatsForReceivePacket(
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const unsigned int packetLengthBytes,
        const TimeType& delay);

    class FlowReservationEvent: public SimulationEvent {
    public:
        explicit
        FlowReservationEvent(const shared_ptr<VideoStreamingSinkApplication>& initVideoStreamingSinkAppPtr)
            : videoStreamingSinkAppPtr(initVideoStreamingSinkAppPtr) { }
        virtual void ExecuteEvent() { videoStreamingSinkAppPtr->ReserveOrUnreserveBandwidth(); }
    private:
        shared_ptr<VideoStreamingSinkApplication> videoStreamingSinkAppPtr;

    };//FlowReservationEvent//


    class PacketHandler: public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        PacketHandler(const shared_ptr<VideoStreamingSinkApplication>& initVideoStreamingSinkAppPtr)
            : videoStreamingSinkAppPtr(initVideoStreamingSinkAppPtr) { }

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePort,
            const NetworkAddress& destinationAddress,
            const PacketPriorityType& priority)
        {
            videoStreamingSinkAppPtr->ReceivePacket(packetPtr);
        }

    private:
        shared_ptr<VideoStreamingSinkApplication> videoStreamingSinkAppPtr;

    };//PacketHandler//

    shared_ptr<PacketHandler> packetHandlerPtr;

    bool useJitterBuffer;
    shared_ptr<JitterBufferController> jitterBufferControllerPtr;

    unsigned int numberPacketsReceived;
    TimeType beforeArrivalTime;
    unsigned int expectedSequenceNumber;

    //---------------------------------

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<VideoStreamingSinkApplication>& initAppPtr) : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowIdType& flowId)
            { appPtr->macQosFlowId = flowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<VideoStreamingSinkApplication> appPtr;

    };//FlowRequestReplyFielder//

    void ReserveOrUnreserveBandwidth();
    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

    //---------------------------------

    shared_ptr<CounterStatistic> packetsOutOfOrderStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<RealStatistic> endToEndDelayStatPtr;
    shared_ptr<RealStatistic> endToEndJitterStatPtr;

    friend class VideoStreamingSourceApplication;

};//VideoStreamingSinkApplication//


inline
VideoStreamingSinkApplication::VideoStreamingSinkApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationIdType& initApplicationId,
    const NodeIdType& initSourceNodeId,
    const NodeIdType& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl)
    :
    VideoStreamingApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initEnableQosControl),
    useJitterBuffer(false),
    numberPacketsReceived(0),
    beforeArrivalTime(ZERO_TIME),
    expectedSequenceNumber(START_SEQUENCE_NUMBER),
    packetsOutOfOrderStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_PacketsOutOfOrder"))),
    packetsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_PacketsReceived"))),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesReceived"))),
    endToEndDelayStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_" + initApplicationId + "_EndToEndDelay"))),
    endToEndJitterStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_" + initApplicationId + "_EndToEndJitter")))
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    const TimeType jitterBufferWindow =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-jitter-buffer-window", sourceNodeId, applicationId);

    if (jitterBufferWindow == ZERO_TIME) {
        useJitterBuffer = false;
    }
    else {
        useJitterBuffer = true;

        jitterBufferControllerPtr = shared_ptr<JitterBufferController>(
            new JitterBufferController(
                parameterDatabaseReader,
                initSimulationEngineInterfacePtr,
                modelName,
                applicationId,
                frameRate,
                numberPacketsInAFrame,
                jitterBufferWindow,
                videoStreamingStartTime,
                videoStreamingEndTime));
    }//if//

}//VideoStreamingSinkApplication//

// Two part initialization forced by shared_from_this().

inline
void VideoStreamingSinkApplication::CompleteInitialization()
{
    packetHandlerPtr = shared_ptr<PacketHandler>(new PacketHandler(shared_from_this()));

    assert(transportLayerPtr->udpPtr->PortIsAvailable(destinationPortId));

    transportLayerPtr->udpPtr->OpenSpecificUdpPort(
        NetworkAddress(0),
        destinationPortId,
        packetHandlerPtr);

    if (reserveBandwidthModeIsOn) {

        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        const TimeType minimumSetupTime = 1 * MILLI_SECOND;
        const TimeType reservationLeewayTime = 1 * MILLI_SECOND;

        TimeType reservationTime;
        if ((currentTime + minimumSetupTime + reservationLeewayTime) <= videoStreamingStartTime) {
            reservationTime = (videoStreamingStartTime - reservationLeewayTime);
        }
        else {
            reservationTime = (currentTime + minimumSetupTime);
        }//if//

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new FlowReservationEvent(shared_from_this())),
            reservationTime);

        //Future: QoS Flow termination delay parameter.

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new FlowReservationEvent(shared_from_this())),
            (videoStreamingEndTime + reservationLeewayTime));

    }//if//

}//CompleteInitialization//



inline
void VideoStreamingSinkApplication::ReceivePacket(unique_ptr<Packet>& packetPtr)
{
    const VideoStreamingPayloadType videoStreamingPayload =
        packetPtr->GetAndReinterpretPayloadData<VideoStreamingPayloadType>();

    const TimeType delay = simulationEngineInterfacePtr->CurrentTime() - videoStreamingPayload.sendTime;

    OutputTraceAndStatsForReceivePacket(
        videoStreamingPayload.sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes(),
        delay);

    if (useJitterBuffer) {
        jitterBufferControllerPtr->Buffering(videoStreamingPayload.sequenceNumber, packetPtr);
    }
    else {

        packetPtr = nullptr;

    }//if//

}//RecievePacket//



inline
void VideoStreamingSinkApplication::OutputTraceAndStatsForReceivePacket(
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const unsigned int packetLengthBytes,
    const TimeType& delay)
{
    numberPacketsReceived++;

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveTraceRecord traceData;
            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.delay = delay;
            traceData.receivedPackets = numberPacketsReceived;
            traceData.packetLengthBytes = static_cast<uint16_t>(packetLengthBytes);

            assert(sizeof(traceData) == APPLICATION_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "VideoRecv", traceData);
        }
        else {

            ostringstream outStream;
            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId
                      << " Delay= " << ConvertTimeToStringSecs(delay)
                      << " Pdr= " << numberPacketsReceived << '/' << sequenceNumber
                      << " PacketBytes= " << packetLengthBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "VideoRecv", outStream.str());
        }//if//
    }//if//

    packetsReceivedStatPtr->IncrementCounter();
    bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);
    endToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delay));

    //
    // Out of order packets counter
    //
    if (sequenceNumber < expectedSequenceNumber) {
        packetsOutOfOrderStatPtr->IncrementCounter();
    }
    else {
        expectedSequenceNumber = sequenceNumber;
    }

    //
    // Inter-arrival time counter
    //
    if (beforeArrivalTime != ZERO_TIME) {
        const TimeType jitter = simulationEngineInterfacePtr->CurrentTime() - beforeArrivalTime;
        endToEndJitterStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(jitter));
    }
    beforeArrivalTime = simulationEngineInterfacePtr->CurrentTime();

}//OutputTraceAndStatsForReceivePacket//



inline
void VideoStreamingSinkApplication::ReserveBandwidth()
{
    NetworkAddress sourceAddress;
    bool foundSourceAddress;
    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        sourceNodeId, sourceAddress, foundSourceAddress);

    if (!foundSourceAddress) {
        return;
    }//if//

    const NetworkAddress destinationAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(destinationNodeId);

    const NetworkLayer& theNetworkLayer = *transportLayerPtr->GetNetworkLayerPtr();

    bool success;
    unsigned int interfaceIndex;
    NetworkAddress nextHopAddress;
    theNetworkLayer.GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress, success, nextHopAddress, interfaceIndex);

    if ((success) && (theNetworkLayer.MacSupportsQualityOfService(interfaceIndex))) {

        MacQualityOfServiceControlInterface& qosControlInterface =
            *theNetworkLayer.GetMacQualityOfServiceInterface(interfaceIndex);

        shared_ptr<FlowRequestReplyFielder> replyPtr(new FlowRequestReplyFielder(shared_from_this()));

        qosControlInterface.RequestDownlinkFlowReservation(
            reservationScheme, schedulingScheme, videoStreamingPriority, qosMinBandwidth, qosMaxBandwidth,
            sourceAddress, ANY_PORT, destinationAddress, destinationPortId, IP_PROTOCOL_NUMBER_UDP,
            replyPtr);

    }//if//

}//ReserveBandwidth//


inline
void VideoStreamingSinkApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in Video with QoS application: Bandwidth request denied." << endl;
}


inline
void VideoStreamingSinkApplication::UnreserveBandwidth()
{
    const NetworkAddress destinationAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(destinationNodeId);

    const NetworkLayer& theNetworkLayer = *transportLayerPtr->GetNetworkLayerPtr();

    bool success;
    unsigned int interfaceIndex;
    NetworkAddress nextHopAddress;
    theNetworkLayer.GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress, success, nextHopAddress, interfaceIndex);

    if ((success) && (theNetworkLayer.MacSupportsQualityOfService(interfaceIndex))) {

        MacQualityOfServiceControlInterface& qosControlInterface =
            *theNetworkLayer.GetMacQualityOfServiceInterface(interfaceIndex);

        qosControlInterface.DeleteFlow(macQosFlowId);
    }//if//

}//UnreserveBandwidth//


inline
void VideoStreamingSinkApplication::ReserveOrUnreserveBandwidth()
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (currentTime < videoStreamingEndTime) {
        (*this).ReserveBandwidth();
    }
    else {
        (*this).UnreserveBandwidth();
    }//if//
}//ReserveOrUnreserveBandwidth//



}//namespace//

#endif
