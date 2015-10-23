// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_APP_VBR_H
#define SCENSIM_APP_VBR_H

#include "scensim_netsim.h"

namespace ScenSim {

using std::cerr;
using std::endl;


//--------------------------------------------------------------------------------------------------

class VbrApplication: public Application {
public:
    static const string modelName;

    typedef MacQualityOfServiceControlInterface::SchedulingSchemeChoice SchedulingSchemeChoice;
    typedef MacQualityOfServiceControlInterface::ReservationSchemeChoice ReservationSchemeChoice;

    VbrApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationIdType& initApplicationId,
        const NodeIdType& initSourceNodeId,
        const NodeIdType& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl);

    struct VbrPayloadType {
        unsigned int sequenceNumber;
        TimeType sendTime;

        VbrPayloadType(
            const unsigned int initSequenceNumber,
            const TimeType initSendTime)
            :
            sequenceNumber(initSequenceNumber),
            sendTime(initSendTime)
        {}
    };//VbrPayloadType//

protected:

    NodeIdType sourceNodeId;
    NodeIdType destinationNodeId;
    NetworkAddress destinationMulticastIpAddress;
    unsigned short int destinationPortId;
    int packetPayloadSizeBytes;

    TimeType minimumPacketInterval;
    TimeType maximumPacketInterval;
    TimeType meanPacketInterval;

    TimeType vbrStartTime;
    TimeType vbrEndTime;
    PacketPriorityType vbrPriority;
    TimeType maxStartTimeJitter;

    bool reserveBandwidthModeIsOn;
    double qosMinBandwidth;
    double qosMaxBandwidth;

    FlowIdType macQosFlowId;

    SchedulingSchemeChoice schedulingScheme;
    ReservationSchemeChoice reservationScheme;

    bool useVirtualPayload;

    string GetParameterNamePrefix() const {
        if (reserveBandwidthModeIsOn) {
            return  "vbr-with-qos";
        }//if//
        return "vbr";
    }
};//VbrApplication//


inline
VbrApplication::VbrApplication(
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
    packetPayloadSizeBytes(0),
    minimumPacketInterval(ZERO_TIME),
    maximumPacketInterval(ZERO_TIME),
    meanPacketInterval(ZERO_TIME),
    vbrStartTime(ZERO_TIME),
    vbrEndTime(ZERO_TIME),
    vbrPriority(0),
    maxStartTimeJitter(ZERO_TIME),
    reserveBandwidthModeIsOn(initEnableQosControl),
    qosMinBandwidth(0.0),
    qosMaxBandwidth(0.0),
    schedulingScheme(MacQualityOfServiceControlInterface::DefaultSchedulingScheme),
    reservationScheme(MacQualityOfServiceControlInterface::OptimisticLinkRate),
    useVirtualPayload(false)
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    packetPayloadSizeBytes =
        parameterDatabaseReader.ReadInt(
            parameterPrefix + "-payload-size-bytes", sourceNodeId, applicationId);

    if (packetPayloadSizeBytes < (int)sizeof(VbrPayloadType)) {
        cerr << "Error: VBR Application payload size ("
            << packetPayloadSizeBytes << ") should be "
            << sizeof(VbrPayloadType) << " bytes or larger." << endl;
        exit(1);
    }//if//

    bool trafficSpecifiedInterval = false;
    bool trafficSpecifiedBps = false;
    bool trafficSpecifiedPps = false;

    if (parameterDatabaseReader.ParameterExists(
            parameterPrefix + "-traffic-defined-by", sourceNodeId, applicationId)) {

        const string trafficDefinedBy =
            MakeLowerCaseString(
                parameterDatabaseReader.ReadString(
                    parameterPrefix + "-traffic-defined-by", sourceNodeId, applicationId));

        if (trafficDefinedBy == "interval") {
            trafficSpecifiedInterval = true;
        }
        else if (trafficDefinedBy == "packetspersecond") {
            trafficSpecifiedPps = true;
        }
        else if (trafficDefinedBy == "bitspersecond") {
            trafficSpecifiedBps = true;
        }
        else {
            cerr << "Error : invalid traffic specification " << trafficDefinedBy << endl;
            exit(1);
        }//if//
    }
    else {
        trafficSpecifiedInterval =
            parameterDatabaseReader.ParameterExists(
                parameterPrefix + "-mean-packet-interval", sourceNodeId, applicationId);

        trafficSpecifiedBps =
            parameterDatabaseReader.ParameterExists(
                parameterPrefix + "-mean-traffic-bps", sourceNodeId, applicationId);

        trafficSpecifiedPps =
            parameterDatabaseReader.ParameterExists(
                parameterPrefix + "-mean-traffic-pps", sourceNodeId, applicationId);
    }//if//

    if ((trafficSpecifiedInterval && trafficSpecifiedBps) ||
        (trafficSpecifiedInterval && trafficSpecifiedPps) ||
        (trafficSpecifiedBps && trafficSpecifiedPps)) {
        cerr << "Error : application traffic specification is conflicted." << endl;
        exit(1);
    }//if//

    if (trafficSpecifiedBps) {
        const unsigned long long int  trafficBps =
            parameterDatabaseReader.ReadBigInt(
                parameterPrefix + "-mean-traffic-bps", sourceNodeId, applicationId);

        if (trafficBps == 0) {
            cerr << "Error : Set " << parameterPrefix << "-mean-traffic-bps > 0." << endl;
            exit(1);
        }

        const double trafficBytePerSec = static_cast<double>(trafficBps) / 8.;

        meanPacketInterval = ConvertDoubleSecsToTime(packetPayloadSizeBytes / trafficBytePerSec);
    }
    else if (trafficSpecifiedPps) {
        const double trafficPps =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-mean-traffic-pps", sourceNodeId, applicationId);

        if (trafficPps == 0) {
            cerr << "Error : Set " << parameterPrefix << "-mean-traffic-pps > 0." << endl;
            exit(1);
        }

        meanPacketInterval = ConvertDoubleSecsToTime(1./trafficPps);
    }
    else {
        meanPacketInterval =
            parameterDatabaseReader.ReadTime(
                parameterPrefix + "-mean-packet-interval", sourceNodeId, applicationId);
    }//if//

    minimumPacketInterval =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-minimum-packet-interval", sourceNodeId, applicationId);

    maximumPacketInterval =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-maximum-packet-interval", sourceNodeId, applicationId);

    vbrStartTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-start-time", sourceNodeId, applicationId);

    vbrEndTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-end-time", sourceNodeId, applicationId);

    vbrPriority = static_cast<PacketPriorityType>(
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-priority", sourceNodeId, applicationId));

    if (parameterDatabaseReader.ParameterExists(
            parameterPrefix + "-start-time-max-jitter", sourceNodeId, applicationId)) {

        maxStartTimeJitter =
            parameterDatabaseReader.ReadTime(
                parameterPrefix + "-start-time-max-jitter", sourceNodeId, applicationId);
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
    }//if/

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-auto-port-mode", sourceNodeId, applicationId)) {

        if (!parameterDatabaseReader.ReadBool(
                parameterPrefix + "-auto-port-mode", sourceNodeId, applicationId)) {

            destinationPortId = static_cast<unsigned short int>(
                parameterDatabaseReader.ReadNonNegativeInt(
                    parameterPrefix + "-destination-port", sourceNodeId, applicationId));
        }//if//
    }//if//


    if ((minimumPacketInterval > meanPacketInterval) ||
        (meanPacketInterval > maximumPacketInterval)) {
        cerr << "Error: VBR Application pakcet intervals shoule be \""
             << "Min(" << ConvertTimeToStringSecs(minimumPacketInterval) << ") <= "
             << "Mean(" << ConvertTimeToStringSecs(meanPacketInterval) << ") <= "
             << "Max(" << ConvertTimeToStringSecs(maximumPacketInterval) << ")\"" << endl;
        exit(1);
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-use-virtual-payload", sourceNodeId, applicationId)) {

        useVirtualPayload = parameterDatabaseReader.ReadBool(
            parameterPrefix + "-use-virtual-payload", sourceNodeId, applicationId);
    }//if//

    destinationMulticastIpAddress = NetworkAddress::invalidAddress;

    if (parameterDatabaseReader.ParameterExists(
        (parameterPrefix + "-destination-multicast-group-number"), sourceNodeId, applicationId)) {

        if (destinationNodeId != ANY_NODEID) {
            cerr << "Error in " << modelName << " Multicast group number but destination is to a single node." << endl;
            exit(1);
        }//if//

        const unsigned int multicastGroupNumber =
            parameterDatabaseReader.ReadNonNegativeInt(
                (parameterPrefix + "-destination-multicast-group-number"),
                sourceNodeId, applicationId);


        if (multicastGroupNumber > NetworkAddress::maxMulticastGroupNumber) {
            cerr << "Error in " << modelName << " Multicast group number is too large: "
                 << multicastGroupNumber << endl;
        }//if//

        destinationMulticastIpAddress = NetworkAddress::MakeAMulticastAddress(multicastGroupNumber);

    }//if//

}//VbrApplication//


class VbrSourceApplication:
     public VbrApplication, public enable_shared_from_this<VbrSourceApplication> {
public:
    VbrSourceApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationIdType& initApplicationId,
        const NodeIdType& initSourceNodeId,
        const NodeIdType& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl,
        const RandomNumberGeneratorSeedType& initNodeSeed);

    void CompleteInitialization();

private:
    TimeType GetExponentialInterval();
    TimeType GetNextPacketTransmissionTime();

    class VbrEvent: public SimulationEvent {
    public:
        explicit
        VbrEvent(const shared_ptr<VbrSourceApplication>& initVbrApplicationPtr)
            : vbrApplicationPtr(initVbrApplicationPtr) {}
        virtual void ExecuteEvent() { vbrApplicationPtr->SendPacket(); }

    private:
        shared_ptr<VbrSourceApplication> vbrApplicationPtr;

    };//VbrEvent//

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<VbrSourceApplication>& initAppPtr) : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowIdType& flowId)
            { appPtr->macQosFlowId = flowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<VbrSourceApplication> appPtr;

    };//FlowRequestReplyFielder//

    unsigned int currentPacketSequenceNumber;

    void SendPacket();

    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();


    static const int SEED_HASH = 20110728;

    // aRandomNumberGenerator for Exponential Distribution.
    double lambda;
    RandomNumberGenerator aRandomNumberGeneratorForExponential;

    //For stats

    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> bytesSentStatPtr;

    void OutputTraceAndStatsForSendPacket(
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const NodeIdType& destinationNodeId,
        const unsigned int packetLengthBytes);


};//VbrSourceApplication//



inline
VbrSourceApplication::VbrSourceApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationIdType& initApplicationId,
    const NodeIdType& initSourceNodeId,
    const NodeIdType& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl,
    const RandomNumberGeneratorSeedType& initNodeSeed)
    :
    VbrApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initEnableQosControl),
    currentPacketSequenceNumber(0),
    lambda(1.0/ConvertTimeToDoubleSecs(meanPacketInterval)),
    aRandomNumberGeneratorForExponential(
        HashInputsToMakeSeed(initNodeSeed, initApplicationId, SEED_HASH)),
    packetsSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_PacketsSent"))),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesSent")))
{
}//VbrSourceApplication//

// Two part initialization forced by shared_from_this().

inline
void VbrSourceApplication::CompleteInitialization()
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (maxStartTimeJitter != ZERO_TIME) {
        vbrStartTime += static_cast<TimeType>(
            aRandomNumberGeneratorPtr->GenerateRandomDouble()*maxStartTimeJitter);
    }//if//

    const TimeType minimumSetupTime = 1 * MILLI_SECOND;

    if (vbrStartTime < (currentTime + minimumSetupTime)) {
        const size_t nextTransmissionTime = size_t(ceil(double((currentTime + minimumSetupTime) - vbrStartTime) / meanPacketInterval));
        vbrStartTime += (nextTransmissionTime * meanPacketInterval);
    }//if//

    if (vbrStartTime < vbrEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new VbrEvent(shared_from_this())),
            vbrStartTime);
    }//if//

}//CompleteInitialization//

inline
TimeType VbrSourceApplication::GetExponentialInterval()
{
    //exponential distribution
    const double randomNumber = aRandomNumberGeneratorForExponential.GenerateRandomDouble();

    return ConvertDoubleSecsToTime(-log(1.0 - randomNumber) / lambda);

}//GetExponentialInterval//

inline
TimeType VbrSourceApplication::GetNextPacketTransmissionTime()
{

    assert((minimumPacketInterval <= meanPacketInterval) &&
        (meanPacketInterval <= maximumPacketInterval));

    if (minimumPacketInterval == maximumPacketInterval) return minimumPacketInterval;

    TimeType packetInterval;
    do {
        packetInterval = (*this).GetExponentialInterval();

    } while ((packetInterval < minimumPacketInterval) || (packetInterval > maximumPacketInterval));

    return packetInterval;

}//GetNextPacketTransmissionTime//



inline
void VbrSourceApplication::OutputTraceAndStatsForSendPacket(
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const NodeIdType& destinationNodeId,
    const unsigned int packetLengthBytes)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        string eventName;

        if (destinationNodeId == ANY_NODEID) {
            eventName = "VbrBcSend";
        } else {
            eventName = "VbrSend";
        }

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.destinationNodeId = destinationNodeId;
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, eventName, traceData);
        }
        else {
            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, eventName, outStream.str());
        }//if//
    }//if//

    const unsigned int numberPacketsSent = sequenceNumber;

    packetsSentStatPtr->UpdateCounter(numberPacketsSent);
    bytesSentStatPtr->IncrementCounter(packetLengthBytes);

}//OutputTraceAndStatsForSendPacket//




inline
void VbrSourceApplication::SendPacket()
{
    assert(currentPacketSequenceNumber < UINT_MAX);

    if ((currentPacketSequenceNumber == 0) && (reserveBandwidthModeIsOn)) {
        (*this).ReserveBandwidth();
    }//if//

    currentPacketSequenceNumber++;

    VbrPayloadType vbrAppPayload(
        currentPacketSequenceNumber, simulationEngineInterfacePtr->CurrentTime());

    unique_ptr<Packet> packetPtr =
        Packet::CreatePacket(
            *simulationEngineInterfacePtr,
            vbrAppPayload,
            packetPayloadSizeBytes,
            useVirtualPayload);

    // Use my primary address for source address.

    const NetworkAddress sourceAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(sourceNodeId);

    OutputTraceAndStatsForSendPacket(
        currentPacketSequenceNumber,
        packetPtr->GetPacketId(),
        destinationNodeId,
        packetPayloadSizeBytes);

    if (destinationNodeId == ANY_NODEID) {
        if (destinationMulticastIpAddress == NetworkAddress::invalidAddress) {
            transportLayerPtr->udpPtr->SendPacket(
                packetPtr, 0, NetworkAddress::broadcastAddress, destinationPortId, vbrPriority);
        }
        else {
            transportLayerPtr->udpPtr->SendPacket(
                packetPtr, 0, destinationMulticastIpAddress, destinationPortId, vbrPriority);
        }//if//
    }
    else {
        NetworkAddress destAddress;
        bool foundDestAddress;
        networkAddressLookupInterfacePtr->LookupNetworkAddress(destinationNodeId, destAddress, foundDestAddress);

        if (foundDestAddress) {
            transportLayerPtr->udpPtr->SendPacket(
                packetPtr, sourceAddress, 0, destAddress, destinationPortId, vbrPriority);
        }
        else {
            //cannot find destination address (destination node may not be created yet)
            //Future: output trace and stat
            packetPtr = nullptr;
        }//if//
    }//if//

    const TimeType nextPacketTime =
        simulationEngineInterfacePtr->CurrentTime() + (*this).GetNextPacketTransmissionTime();

    if (nextPacketTime < vbrEndTime) {
      simulationEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new VbrEvent(shared_from_this())),
        nextPacketTime);
    }
    else {
        if (reserveBandwidthModeIsOn) {
            (*this).UnreserveBandwidth();
        }//if//
    }//if//

}//SendPacket//



inline
void VbrSourceApplication::ReserveBandwidth()
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
            reservationScheme, schedulingScheme, vbrPriority, qosMinBandwidth, qosMaxBandwidth,
            destinationAddress, destinationPortId, sourceAddress, ANY_PORT, IP_PROTOCOL_NUMBER_UDP,
            replyPtr);
    }//if//

}//ReserveBandwidth//



inline
void VbrSourceApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in VBR with QoS application: Bandwidth request denied." << endl;
}


inline
void VbrSourceApplication::UnreserveBandwidth()
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

class VbrSinkApplication:
    public VbrApplication, public enable_shared_from_this<VbrSinkApplication> {
public:
    VbrSinkApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationIdType& initApplicationId,
        const NodeIdType& initSourceNodeId,
        const NodeIdType& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl);

    void CompleteInitialization();

private:
    class FlowReservationEvent: public SimulationEvent {
    public:
        explicit
        FlowReservationEvent(const shared_ptr<VbrSinkApplication>& initVbrApplicationPtr)
            : vbrApplicationPtr(initVbrApplicationPtr) {}
        virtual void ExecuteEvent() { vbrApplicationPtr->ReserveOrUnreserveBandwidth(); }

    private:
        shared_ptr<VbrSinkApplication> vbrApplicationPtr;

    };//FlowReservationEvent//

    class PacketHandler: public UdpProtocol::PacketForAppFromTransportLayerHandler {
    public:
        PacketHandler(const shared_ptr<VbrSinkApplication>& initVbrSinkPtr) : vbrSinkPtr(initVbrSinkPtr) { }

        void ReceivePacket(
            unique_ptr<Packet>& packetPtr,
            const NetworkAddress& sourceAddress,
            const unsigned short int sourcePort,
            const NetworkAddress& destinationAddress,
            const PacketPriorityType& priority)
        {
            vbrSinkPtr->ReceivePacket(packetPtr);
        }


    private:
        shared_ptr<VbrSinkApplication> vbrSinkPtr;

    };//PacketHandler//

    //-------------------------------------------------------------------------

    shared_ptr<PacketHandler> packetHandlerPtr;

    // This should be simulator parameters. TBD
    static const TimeType duplicateDetectorWindowTime = 60 * SECOND;
    static const unsigned int duplicateDetectorMinWindowSize = 128;

    static unsigned int CalcDuplicateDetectorWindowSize(
        const TimeType& timeInterval,
        const TimeType& totalTrafficTime)
    {
        const TimeType windowTime = std::min(duplicateDetectorWindowTime, totalTrafficTime);

        return (
            std::max(
                duplicateDetectorMinWindowSize,
                static_cast<unsigned int>(DivideAndRoundUp(windowTime, timeInterval))));
    }


    WindowedDuplicateSequenceNumberDetector<unsigned int> duplicateDetector;

    //---------------------------------

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<VbrSinkApplication>& initAppPtr) : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowIdType& flowId)
            { appPtr->macQosFlowId = flowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<VbrSinkApplication> appPtr;

    };//FlowRequestReplyFielder//

    //---------------------------------

    unsigned int numberPacketsReceived;
    unsigned int numberDuplicatePacketsReceived;

    shared_ptr<CounterStatistic> packetsReceivedStatPtr;
    shared_ptr<CounterStatistic> duplicatePacketsReceivedStatPtr;
    shared_ptr<CounterStatistic> duplicatePacketOutOfWindowErrorStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    shared_ptr<RealStatistic> endToEndDelayStatPtr;

    //---------------------------------

    void ReceivePacket(unique_ptr<Packet>& packetPtr);

    void OutputTraceAndStatsForReceivePacket(
        const unsigned int sequenceNumber,
        const PacketIdType& packetId,
        const unsigned int packetLengthBytes,
        const TimeType& delay);


    void ReserveOrUnreserveBandwidth();
    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

};//VbrSinkApplication//



inline
VbrSinkApplication::VbrSinkApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationIdType& initApplicationId,
    const NodeIdType& initSourceNodeId,
    const NodeIdType& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl)
    :
    VbrApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initEnableQosControl),
    numberPacketsReceived(0),
    numberDuplicatePacketsReceived(0),
    duplicateDetector(
        CalcDuplicateDetectorWindowSize(meanPacketInterval, (vbrEndTime - vbrStartTime))),
    packetsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_PacketsReceived"))),
    duplicatePacketsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_DuplicatePacketsReceived"))),
    duplicatePacketOutOfWindowErrorStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_DuplicatePacketOutOfWindowErrors"))),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesReceived"))),
    endToEndDelayStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_" + initApplicationId + "_EndToEndDelay")))
{
}//VbrSinkApplication//

// Two part initialization forced by shared_from_this().

inline
void VbrSinkApplication::CompleteInitialization()
{
    packetHandlerPtr = shared_ptr<PacketHandler>(new PacketHandler(shared_from_this()));

    assert(transportLayerPtr->udpPtr->PortIsAvailable(destinationPortId));

    transportLayerPtr->udpPtr->OpenSpecificUdpPort(
        NetworkAddress::anyAddress,
        destinationPortId,
        packetHandlerPtr);

    if (reserveBandwidthModeIsOn) {

        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        const TimeType minimumSetupTime = 1 * MILLI_SECOND;
        const TimeType reservationLeewayTime = 1 * MILLI_SECOND;

        TimeType reservationTime;
        if ((currentTime + minimumSetupTime + reservationLeewayTime) <= vbrStartTime) {
            reservationTime = (vbrStartTime - reservationLeewayTime);
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
            (vbrEndTime + reservationLeewayTime));

    }//if//

}//CompleteInitialization//

inline
void VbrSinkApplication::OutputTraceAndStatsForReceivePacket(
    const unsigned int sequenceNumber,
    const PacketIdType& packetId,
    const unsigned int packetLengthBytes,
    const TimeType& delay)
{
    if (duplicateDetector.IsInSequenceNumberWindow(sequenceNumber)) {
        if (!duplicateDetector.IsDuplicate(sequenceNumber)) {
            duplicateDetector.SetAsSeen(sequenceNumber);
            numberPacketsReceived++;
            packetsReceivedStatPtr->IncrementCounter();
            bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);
            endToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delay));
        }
        else {
            numberDuplicatePacketsReceived++;
            duplicatePacketsReceivedStatPtr->IncrementCounter();
        }//if//
    }
    else {
        duplicatePacketOutOfWindowErrorStatPtr->IncrementCounter();
    }//if//


    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        string eventName;

        if (destinationNodeId == ANY_NODEID) {
            eventName = "VbrBcRecv";
        } else {
            eventName = "VbrRecv";
        }

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
                modelName, applicationId, eventName, traceData);

        }
        else {
            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << packetId
                      << " Delay= " << ConvertTimeToStringSecs(delay)
                      << " Pdr= " << numberPacketsReceived << '/' << sequenceNumber
                      << " PacketBytes= " << packetLengthBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, eventName, outStream.str());

        }//if//

    }//if//

}//OutputTraceAndStatsForReceivePacket//



inline
void VbrSinkApplication::ReceivePacket(unique_ptr<Packet>& packetPtr)
{
    VbrPayloadType vbrPayload = packetPtr->GetAndReinterpretPayloadData<VbrPayloadType>();

    TimeType delay = simulationEngineInterfacePtr->CurrentTime() - vbrPayload.sendTime;

    OutputTraceAndStatsForReceivePacket(
        vbrPayload.sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes(),
        delay);

    packetPtr = 0;
}//ReceivePacket//


inline
void VbrSinkApplication::ReserveBandwidth()
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
            reservationScheme, schedulingScheme, vbrPriority, qosMinBandwidth, qosMaxBandwidth,
            sourceAddress, ANY_PORT, destinationAddress, destinationPortId, IP_PROTOCOL_NUMBER_UDP,
            replyPtr);
    }//if//

}//ReserveBandwidth//


inline
void VbrSinkApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in VBR with QoS application: Bandwidth request denied." << endl;
}


inline
void VbrSinkApplication::UnreserveBandwidth()
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
void VbrSinkApplication::ReserveOrUnreserveBandwidth()
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (currentTime < vbrEndTime) {
        (*this).ReserveBandwidth();
    }
    else {
        (*this).UnreserveBandwidth();
    }//if//
}//ReserveOrUnreserveBandwidth//


}//namespace//

#endif
