// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_APP_HTTP_H
#define SCENSIM_APP_HTTP_H

#include "scensim_netsim.h"

namespace ScenSim {

using std::cerr;
using std::endl;


//--------------------------------------------------------------------------------------------------

//
// ----- HTTP Model -----
//
// Refer to "IEEE802.16m Evaluation Methodology"
//

class HttpFlowApplication: public Application {
public:
    static const string modelName;

    typedef MacQualityOfServiceControlInterface::SchedulingSchemeChoice SchedulingSchemeChoice;
    typedef MacQualityOfServiceControlInterface::ReservationSchemeChoice ReservationSchemeChoice;

    HttpFlowApplication(
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
    unsigned short int sourcePortId;
    unsigned short int destinationPortId;
    unsigned long long int minimumMainObjectBytes;
    unsigned long long int maximumMainObjectBytes;
    unsigned int minimumNumberEmbeddedObjects;
    unsigned int maximumNumberEmbeddedObjects;
    unsigned long long int minimumEmbeddedObjectBytes;
    unsigned long long int maximumEmbeddedObjectBytes;
    TimeType flowStartTime;
    TimeType flowEndTime;
    PacketPriorityType flowPriority;
    TimeType maxStartTimeJitter;

    SchedulingSchemeChoice schedulingScheme;
    ReservationSchemeChoice reservationScheme;

    bool reserveBandwidthModeIsOn;
    double qosMinBandwidth;
    double qosMaxBandwidth;

    // For reservations of reverse channel (ACKs)

    double qosMinReverseBandwidth;
    double qosMaxReverseBandwidth;

    FlowIdType macQosFlowId;

    bool useVirtualPayload;

    string GetParameterNamePrefix() const {
        if (reserveBandwidthModeIsOn) {
            return  "http-with-qos";
        }//if//
        return "http";
    }
};//HttpFlowApplication//


inline
HttpFlowApplication::HttpFlowApplication(
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
    sourcePortId(initDefaultApplicationPortId),
    destinationNodeId(initDestinationNodeId),
    destinationPortId(initDefaultApplicationPortId),
    minimumMainObjectBytes(0),
    maximumMainObjectBytes(0),
    minimumNumberEmbeddedObjects(0),
    maximumNumberEmbeddedObjects(0),
    minimumEmbeddedObjectBytes(0),
    maximumEmbeddedObjectBytes(0),
    flowStartTime(ZERO_TIME),
    flowEndTime(ZERO_TIME),
    flowPriority(0),
    maxStartTimeJitter(ZERO_TIME),
    reserveBandwidthModeIsOn(initEnableQosControl),
    qosMinBandwidth(0.0),
    qosMaxBandwidth(0.0),
    qosMinReverseBandwidth(0.0),
    qosMaxReverseBandwidth(0.0),
    schedulingScheme(MacQualityOfServiceControlInterface::DefaultSchedulingScheme),
    reservationScheme(MacQualityOfServiceControlInterface::OptimisticLinkRate),
    useVirtualPayload(false)
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    minimumMainObjectBytes =
        parameterDatabaseReader.ReadBigInt(
            parameterPrefix + "-min-main-object-bytes", sourceNodeId, applicationId);

    maximumMainObjectBytes =
        parameterDatabaseReader.ReadBigInt(
            parameterPrefix + "-max-main-object-bytes", sourceNodeId, applicationId);

    minimumNumberEmbeddedObjects =
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-min-number-embedded-objects", sourceNodeId, applicationId);

    maximumNumberEmbeddedObjects =
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-max-number-embedded-objects", sourceNodeId, applicationId);

    minimumEmbeddedObjectBytes =
        parameterDatabaseReader.ReadBigInt(
            parameterPrefix + "-min-embedded-object-bytes", sourceNodeId, applicationId);

    maximumEmbeddedObjectBytes =
        parameterDatabaseReader.ReadBigInt(
            parameterPrefix + "-max-embedded-object-bytes", sourceNodeId, applicationId);

    flowStartTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-start-time", sourceNodeId, applicationId);

    flowEndTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-end-time", sourceNodeId, applicationId);

    flowPriority = static_cast<PacketPriorityType>(
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

            sourcePortId = destinationPortId;
        }//if//
    }//if//

    if (initEnableQosControl) {
        qosMinBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-baseline-bandwidth-bytes", sourceNodeId, applicationId);

        qosMaxBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-max-bandwidth-bytes", sourceNodeId, applicationId);

        qosMinReverseBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-baseline-reverse-bandwidth-bytes", sourceNodeId, applicationId);

        qosMaxReverseBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-max-reverse-bandwidth-bytes", sourceNodeId, applicationId);

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

}//HttpFlowApplication//


class HttpFlowSourceApplication:
    public HttpFlowApplication, public enable_shared_from_this<HttpFlowSourceApplication> {
public:

    HttpFlowSourceApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationIdType& initApplicationId,
        const NodeIdType& initSourceNodeId,
        const NodeIdType& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl,
        const RandomNumberGeneratorSeedType& initNodeSeed);

    void CompleteInitialization();

    void DisconnectFromOtherLayers();

private:
    static const int SEED_HASH = 87323412;

    static const double maximumTcpFlowBlockSizeRate;
    static const unsigned long long int minimumTcpFlowBlockSize = 576;
    static const unsigned long long int maximumTcpFlowBlockSize = 1500;

    unsigned long long int GetFlowTotalBytes();
    unsigned int GetNumberEmbeddedObjectsConformUntruncatedPareto();
    unsigned int GetNumberEmbeddedObjects();
    TimeType GetReadingTime();

    void StartFlow();
    void EmbeddedFlow();
    void SendDataIfNecessary();

    //----------------------------------

    class StartFlowEvent: public SimulationEvent {
    public:
        explicit
        StartFlowEvent(const shared_ptr<HttpFlowSourceApplication>& initHttpFlowApplicationPtr)
            : httpFlowApplicationPtr(initHttpFlowApplicationPtr) {}

        virtual void ExecuteEvent() { httpFlowApplicationPtr->StartFlow(); }

    private:
        shared_ptr<HttpFlowSourceApplication> httpFlowApplicationPtr;

    };//StartFlowEvent//

    class EmbeddedFlowEvent: public SimulationEvent {
    public:
        explicit
        EmbeddedFlowEvent(const shared_ptr<HttpFlowSourceApplication>& initHttpFlowApplicationPtr)
            : httpFlowApplicationPtr(initHttpFlowApplicationPtr) {}

        virtual void ExecuteEvent() { httpFlowApplicationPtr->EmbeddedFlow(); }

    private:
        shared_ptr<HttpFlowSourceApplication> httpFlowApplicationPtr;

    };//EmbeddedFlowEvent//


    class TcpEventHandler: public TcpConnection::AppTcpEventHandler {
    public:
        TcpEventHandler(const shared_ptr<HttpFlowSourceApplication> initHttpFlowSourceAppPtr)
            : httpFlowSourceAppPtr(initHttpFlowSourceAppPtr) {}

        void DoTcpIsReadyForMoreDataAction()
            { httpFlowSourceAppPtr->SendDataIfNecessary(); }
        void ReceiveDataBlock(
            const unsigned char dataBlock[],
            const unsigned int dataLength,
            const unsigned int actualDataLength,
            bool& stallIncomingDataFlow)
            { assert(false && "App does not receive data."); abort(); }

    private:
        shared_ptr<HttpFlowSourceApplication> httpFlowSourceAppPtr;

    };//SendCompletionHandler//


    //----------------------------------

    enum HttpSourceStateType {
        IDLE_STATE,
        SEND_MAIN_OBJECT,
        WAITING_FOR_EMBEDDED,
        SEND_EMBEDDED_OBJECT,
        WAITING_FOR_NEXT_PAGE
    };

    HttpSourceStateType httpSourceState;

    TimeType currentFlowStartTime;
    unsigned long long int currentMainObjectBytes;
    unsigned long long int currentEmbeddedObjectBytes;
    unsigned long long int flowTotalBytes;
    unsigned int tcpFlowBlockSizeBytes;
    unsigned long long int bytesSentSoFar;
    unsigned int currentNumberEmbeddedObjects;
    unsigned int numberSentEmbeddedObjects;
    unsigned long long int totalEmbeddedObjects;

    shared_ptr<TcpConnection> tcpConnectionPtr;
    shared_ptr<TcpEventHandler> tcpEventHandlerPtr;

    //----------------------------------

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<HttpFlowSourceApplication>& initAppPtr)
            : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowIdType& flowId)
            { appPtr->macQosFlowId = flowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<HttpFlowSourceApplication> appPtr;

    };//FlowRequestReplyFielder//

    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

    //----------------------------------

    // for Stats
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<RealStatistic> transmissionDelayStatPtr;

    void OutputTraceAndStatsStartFlow(
        const unsigned long long int& flowBytes);

    void OutputTraceAndStatsForEndFlow(
        const TimeType& transmissionDelay,
        const unsigned long long int& flowReceivedBytes);

    // for aRandomNumberGenerator

    shared_ptr<boost::variate_generator<boost::mt19937, boost::lognormal_distribution<> > > aRandomNumberGeneratorForMainObjectPtr;
    shared_ptr<boost::variate_generator<boost::mt19937, boost::lognormal_distribution<> > > aRandomNumberGeneratorForEmbeddedObjectPtr;

    double lambdaForPage;
    RandomNumberGenerator aRandomNumberGeneratorForPage;
    double lambdaForEmbedded;
    RandomNumberGenerator aRandomNumberGeneratorForEmbedded;
    double alphaForNumberEmbeddedObjects;
    RandomNumberGenerator aRandomNumberGeneratorForNumberEmbeddedObjects;

    RandomNumberGenerator aRandomNumberGenerator;


};//HttpFlowSourceApplication//


inline
HttpFlowSourceApplication::HttpFlowSourceApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationIdType& initApplicationId,
    const NodeIdType& initSourceNodeId,
    const NodeIdType& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl,
    const RandomNumberGeneratorSeedType& initNodeSeed)
    :
    HttpFlowApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initEnableQosControl),
    httpSourceState(IDLE_STATE),
    currentFlowStartTime(ZERO_TIME),
    currentMainObjectBytes(0),
    currentNumberEmbeddedObjects(0),
    currentEmbeddedObjectBytes(0),
    flowTotalBytes(0),
    tcpFlowBlockSizeBytes(0),
    bytesSentSoFar(0),
    numberSentEmbeddedObjects(0),
    totalEmbeddedObjects(0),
    lambdaForPage(0.),
    aRandomNumberGeneratorForPage(
        HashInputsToMakeSeed(initNodeSeed, initApplicationId, SEED_HASH)),
    lambdaForEmbedded(0.),
    aRandomNumberGeneratorForEmbedded(
        HashInputsToMakeSeed(initNodeSeed, initApplicationId, SEED_HASH)),
    alphaForNumberEmbeddedObjects(0.),
    aRandomNumberGeneratorForNumberEmbeddedObjects(
        HashInputsToMakeSeed(initNodeSeed, initApplicationId, SEED_HASH)),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initNodeSeed, initApplicationId, SEED_HASH)),
    bytesSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesSent"))),
    transmissionDelayStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_" + initApplicationId + "_TransmissionDelay")))
{
    const string parameterPrefix = (*this).GetParameterNamePrefix();

    const unsigned long long int meanMainObjectBytes =
        parameterDatabaseReader.ReadBigInt(
            parameterPrefix + "-mean-main-object-bytes", sourceNodeId, applicationId);

    const unsigned long long int standardDeviationMainObjectBytes =
        parameterDatabaseReader.ReadBigInt(
            parameterPrefix + "-standard-deviation-main-object-bytes", sourceNodeId, applicationId);

    const unsigned int meanNumberEmbeddedObjects =
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-mean-number-embedded-objects", sourceNodeId, applicationId);

    const unsigned long long int meanEmbeddedObjectBytes =
        parameterDatabaseReader.ReadNonNegativeBigInt(
            parameterPrefix + "-mean-embedded-object-bytes", sourceNodeId, applicationId);

    const unsigned int standardDeviationEmbeddedObjectBytes =
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-standard-deviation-embedded-object-bytes", sourceNodeId, applicationId);

    const TimeType meanPageReadingTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-mean-page-reading-time", sourceNodeId, applicationId);

    const TimeType meanEmbeddedReadingTime =
        parameterDatabaseReader.ReadTime(
            parameterPrefix + "-mean-embedded-reading-time", sourceNodeId, applicationId);

    lambdaForPage = 1.0/ConvertTimeToDoubleSecs(meanPageReadingTime);

    lambdaForEmbedded = 1.0/ConvertTimeToDoubleSecs(meanEmbeddedReadingTime);

    alphaForNumberEmbeddedObjects =
        static_cast<double>(meanNumberEmbeddedObjects) /
        (meanNumberEmbeddedObjects - minimumNumberEmbeddedObjects);

    aRandomNumberGeneratorForMainObjectPtr.reset(
        new boost::variate_generator<boost::mt19937, boost::lognormal_distribution<> >(
            boost::mt19937(static_cast<unsigned long>(HashInputsToMakeSeed(initNodeSeed, applicationId, SEED_HASH))),
            boost::lognormal_distribution<>(
                static_cast<double>(meanMainObjectBytes),
                static_cast<double>(standardDeviationMainObjectBytes))));

    aRandomNumberGeneratorForEmbeddedObjectPtr.reset(
        new boost::variate_generator<boost::mt19937, boost::lognormal_distribution<> >(
            boost::mt19937(static_cast<unsigned long>(HashInputsToMakeSeed(initNodeSeed, applicationId, SEED_HASH))),
            boost::lognormal_distribution<>(
                static_cast<double>(meanEmbeddedObjectBytes),
                static_cast<double>(standardDeviationEmbeddedObjectBytes))));

}//HttpFlowSourceApplication//

// Two part initialized caused by shared_from_this() limitation

inline
void HttpFlowSourceApplication::CompleteInitialization()
{
    if (maxStartTimeJitter != ZERO_TIME) {
        flowStartTime += static_cast<TimeType>(
            aRandomNumberGeneratorPtr->GenerateRandomDouble() * maxStartTimeJitter);
    }//if//

    tcpEventHandlerPtr.reset(new TcpEventHandler(shared_from_this()));

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    const TimeType minimumSetupTime = 1 * MILLI_SECOND;

    if ((currentTime + minimumSetupTime) <= flowStartTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new StartFlowEvent(shared_from_this())), flowStartTime);
    }
    else {
        //schedule next flow
        httpSourceState = WAITING_FOR_NEXT_PAGE;
        const TimeType nextFlowStartTime =
            simulationEngineInterfacePtr->CurrentTime() + minimumSetupTime + (*this).GetReadingTime();

        if (nextFlowStartTime < flowEndTime) {
            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(new StartFlowEvent(shared_from_this())),
                nextFlowStartTime);
        }//if//

    }//if//
}


inline
void HttpFlowSourceApplication::DisconnectFromOtherLayers()
{
    if (tcpConnectionPtr != nullptr) {
        tcpConnectionPtr->ClearAppTcpEventHandler();
        (*this).tcpConnectionPtr.reset();
    }//if//
    (*this).tcpEventHandlerPtr.reset();

}//DisconnectFromOtherLayers//



inline
unsigned long long int HttpFlowSourceApplication::GetFlowTotalBytes()
{
    unsigned long long int result;

    while(true) {

        if (httpSourceState == SEND_MAIN_OBJECT) {
            const double randomNumber = (*aRandomNumberGeneratorForMainObjectPtr)();
            result = static_cast<unsigned long long int>(randomNumber);

            if (result >= minimumMainObjectBytes && result <= maximumMainObjectBytes) {
                break;
            }

        }
        else if (httpSourceState == SEND_EMBEDDED_OBJECT) {
            const double randomNumber = (*aRandomNumberGeneratorForEmbeddedObjectPtr)();
            result = static_cast<unsigned long long int>(randomNumber);

            if (result >= minimumEmbeddedObjectBytes && result <= maximumEmbeddedObjectBytes) {
                break;
            }

        }
        else {
            assert(false && "Invalid HTTP STATE.");
        }

    }//while//

    return result;

}//GetFlowBlockSizeBytes//


inline
unsigned int HttpFlowSourceApplication::GetNumberEmbeddedObjectsConformUntruncatedPareto()
{
    const double randomNumber = aRandomNumberGeneratorForNumberEmbeddedObjects.GenerateRandomDouble();

    return static_cast<unsigned int>(
        minimumNumberEmbeddedObjects / pow((1.0 - randomNumber), (1.0 / alphaForNumberEmbeddedObjects)));

}//GetPacketPayloadSizeBytesConformUntruncatedParete//


inline
unsigned int HttpFlowSourceApplication::GetNumberEmbeddedObjects()
{
    unsigned int result;

    while(true) {
        result = (*this).GetNumberEmbeddedObjectsConformUntruncatedPareto();
        result -= minimumNumberEmbeddedObjects;

        if (result >= 0 && result <= maximumNumberEmbeddedObjects) {
            break;
        }
    }

    return result;

}//GetNumberEmbeddedObjects//


inline
TimeType HttpFlowSourceApplication::GetReadingTime()
{
    double randomNumber;
    TimeType result;

    if (httpSourceState == WAITING_FOR_EMBEDDED) {
        randomNumber = aRandomNumberGeneratorForEmbedded.GenerateRandomDouble();
        result = ConvertDoubleSecsToTime(-log(1.0 - randomNumber) / lambdaForEmbedded);
    }
    else if (httpSourceState == SEND_EMBEDDED_OBJECT) {
        result = ZERO_TIME;
    }
    else if (httpSourceState == WAITING_FOR_NEXT_PAGE) {
        randomNumber = aRandomNumberGeneratorForPage.GenerateRandomDouble();
        result = ConvertDoubleSecsToTime(-log(1.0 - randomNumber) / lambdaForPage);
    }
    else {
        assert(false && "Invalid HTTP STATE."); result = 0; abort();
    }

    return result;

}//GetReadingTime//



inline
void HttpFlowSourceApplication::StartFlow()
{
    httpSourceState = SEND_MAIN_OBJECT;

    currentFlowStartTime = simulationEngineInterfacePtr->CurrentTime();

    currentMainObjectBytes = (*this).GetFlowTotalBytes();
    flowTotalBytes += currentMainObjectBytes;

    currentNumberEmbeddedObjects = (*this).GetNumberEmbeddedObjects();
    totalEmbeddedObjects += currentNumberEmbeddedObjects;

    const double randomNumber = aRandomNumberGenerator.GenerateRandomDouble();
    if (randomNumber <= maximumTcpFlowBlockSizeRate) {
        tcpFlowBlockSizeBytes = maximumTcpFlowBlockSize;
    }
    else {
        tcpFlowBlockSizeBytes = minimumTcpFlowBlockSize;
    }//if//

    OutputTraceAndStatsStartFlow(currentMainObjectBytes);

    if (tcpConnectionPtr == nullptr) {

        if (reserveBandwidthModeIsOn) {
            (*this).ReserveBandwidth();
        }

        const NetworkAddress sourceAddress =
            networkAddressLookupInterfacePtr->LookupNetworkAddress(sourceNodeId);

        NetworkAddress destinationAddress;
        bool foundDestAddress;
        networkAddressLookupInterfacePtr->LookupNetworkAddress(
            destinationNodeId, destinationAddress, foundDestAddress);

        if (foundDestAddress) {

            transportLayerPtr->tcpPtr->CreateOutgoingTcpConnection(
                sourceAddress, sourcePortId, destinationAddress, destinationPortId, flowPriority,
                tcpEventHandlerPtr, tcpConnectionPtr);

            if (useVirtualPayload) {
                tcpConnectionPtr->EnableVirtualPayload();
            }//if//
        }
        else {
            //cannot find destination address (destination node may not be created yet)
            //Future: output trace and stat

            //schedule next flow
            httpSourceState = WAITING_FOR_NEXT_PAGE;
            const TimeType nextFlowStartTime =
                simulationEngineInterfacePtr->CurrentTime() + (*this).GetReadingTime();

            if (nextFlowStartTime < flowEndTime) {
                simulationEngineInterfacePtr->ScheduleEvent(
                    unique_ptr<SimulationEvent>(new StartFlowEvent(shared_from_this())),
                    nextFlowStartTime);
            }
            else {
                if (reserveBandwidthModeIsOn) {
                    // TBD Delay Unreserve.
                    (*this).UnreserveBandwidth();
                }
            }//if//
        }//if//
    }
    else {
        SendDataIfNecessary();
    }

}//StartFlow//


inline
void HttpFlowSourceApplication::EmbeddedFlow()
{
    httpSourceState = SEND_EMBEDDED_OBJECT;

    assert(currentNumberEmbeddedObjects > 0);

    currentEmbeddedObjectBytes = (*this).GetFlowTotalBytes();
    flowTotalBytes += currentEmbeddedObjectBytes;

    OutputTraceAndStatsStartFlow(currentEmbeddedObjectBytes);

    SendDataIfNecessary();

}//SetEmbeddedObject//


inline
void HttpFlowSourceApplication::SendDataIfNecessary()
{
    while ((bytesSentSoFar < flowTotalBytes) &&
           (tcpConnectionPtr->GetCurrentNumberOfAvailableBufferBytes() >= tcpFlowBlockSizeBytes)) {

        unsigned int blockSizeBytes = static_cast<size_t>(tcpFlowBlockSizeBytes);

        if ((flowTotalBytes - bytesSentSoFar) < tcpFlowBlockSizeBytes) {
            blockSizeBytes = static_cast<unsigned int>(flowTotalBytes - bytesSentSoFar);
        }//if//

        shared_ptr<vector<unsigned char> > dataBlockPtr(new vector<unsigned char>());

        bytesSentSoFar += blockSizeBytes;

        if (!useVirtualPayload) {
            dataBlockPtr->resize(blockSizeBytes);
            for(size_t i = 0; (i < blockSizeBytes); i++) {
                (*dataBlockPtr)[i] = static_cast<unsigned char>(bytesSentSoFar/tcpFlowBlockSizeBytes);
            }//for//
        }//if//

        bytesSentStatPtr->IncrementCounter(blockSizeBytes);

        tcpConnectionPtr->SendDataBlock(dataBlockPtr, blockSizeBytes);

    }//while//


    if (tcpConnectionPtr->GetNumberOfDeliveredBytes() >= flowTotalBytes) {

        if (httpSourceState == SEND_MAIN_OBJECT) {

            if (currentNumberEmbeddedObjects > 0) {
                httpSourceState = WAITING_FOR_EMBEDDED;
            }
            else {
                httpSourceState = WAITING_FOR_NEXT_PAGE;

                const TimeType transmissionDelay =
                    simulationEngineInterfacePtr->CurrentTime() - currentFlowStartTime;

                OutputTraceAndStatsForEndFlow(transmissionDelay, flowTotalBytes);

            }//if//

        }
        else if (httpSourceState == SEND_EMBEDDED_OBJECT) {

            numberSentEmbeddedObjects++;

            if (numberSentEmbeddedObjects >= totalEmbeddedObjects) {
                httpSourceState = WAITING_FOR_NEXT_PAGE;

                const TimeType transmissionDelay =
                    simulationEngineInterfacePtr->CurrentTime() - currentFlowStartTime;

                OutputTraceAndStatsForEndFlow(transmissionDelay, flowTotalBytes);

            }//if//
        }//if//

        const TimeType nextFlowStartTime =
            simulationEngineInterfacePtr->CurrentTime() + (*this).GetReadingTime();

        if (nextFlowStartTime < flowEndTime) {
            if (httpSourceState == WAITING_FOR_EMBEDDED || httpSourceState == SEND_EMBEDDED_OBJECT) {
                simulationEngineInterfacePtr->ScheduleEvent(
                    unique_ptr<SimulationEvent>(new EmbeddedFlowEvent(shared_from_this())),
                    nextFlowStartTime);
            }
            else if (httpSourceState == WAITING_FOR_NEXT_PAGE) {
                simulationEngineInterfacePtr->ScheduleEvent(
                    unique_ptr<SimulationEvent>(new StartFlowEvent(shared_from_this())),
                    nextFlowStartTime);
            }
        }
        else {
            if (reserveBandwidthModeIsOn) {
                // TBD Delay Unreserve.
                (*this).UnreserveBandwidth();
            }
        }//if//

    }//if//

}//SendDataIfNecessary//



inline
void HttpFlowSourceApplication::OutputTraceAndStatsStartFlow(
    const unsigned long long int& flowBytes)
{

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {

        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "HttpStartFlow", flowBytes);

        }
        else {

            ostringstream outStream;

            outStream << "FileBytes= " << flowBytes;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "HttpStartFlow", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsStartFlow//


inline
void HttpFlowSourceApplication::OutputTraceAndStatsForEndFlow(
    const TimeType& transmissionDelay,
    const unsigned long long int& flowDeliveredBytes)
{

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

        ApplicationEndFlowTraceRecord traceData;

        traceData.transmissionDelay = transmissionDelay;
        traceData.flowDeliveredBytes = flowDeliveredBytes;

        assert(sizeof(traceData) == APPLICATION_END_FLOW_TRACE_RECORD_BYTES);

        simulationEngineInterfacePtr->OutputTraceInBinary(
            modelName, applicationId, "HttpEndFlow", traceData);


        }
        else {
            ostringstream outStream;

            outStream << "DeliveredBytes= " << flowDeliveredBytes
                      << " Delay= " << ConvertTimeToStringSecs(transmissionDelay);

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "HttpEndFlow", outStream.str());
        }//if//
    }//if//

    // Stats
    transmissionDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(transmissionDelay));

}//OutputTraceAndStatsForFlowEnd//



inline
void HttpFlowSourceApplication::ReserveBandwidth()
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

        qosControlInterface.RequestDualFlowReservation(
            reservationScheme,
            schedulingScheme,
            flowPriority,
            qosMinReverseBandwidth, qosMaxReverseBandwidth,
            qosMinBandwidth, qosMaxBandwidth,
            destinationAddress, destinationPortId, sourceAddress, sourcePortId, IP_PROTOCOL_NUMBER_TCP,
            replyPtr);
    }//if//

}//ReserveBandwidth//


inline
void HttpFlowSourceApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in HTTP with QoS application: Bandwidth request denied." << endl;
}


inline
void HttpFlowSourceApplication::UnreserveBandwidth()
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

class HttpFlowSinkApplication:
    public HttpFlowApplication, public enable_shared_from_this<HttpFlowSinkApplication> {
public:
    HttpFlowSinkApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationIdType& initApplicationId,
        const NodeIdType& initSourceNodeId,
        const NodeIdType& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl);

    void CompleteInitialization();

    void DisconnectFromOtherLayers();

private:

    class FlowReservationEvent: public SimulationEvent {
    public:
        explicit
        FlowReservationEvent(const shared_ptr<HttpFlowSinkApplication>& initHttpFlowSinkAppPtr)
            : httpFlowSinkAppPtr(initHttpFlowSinkAppPtr) {}
        virtual void ExecuteEvent() { httpFlowSinkAppPtr->ReserveOrUnreserveBandwidth(); }

    private:
        shared_ptr<HttpFlowSinkApplication> httpFlowSinkAppPtr;

    };//FlowReservationEvent//

    class ConnectionHandler: public ConnectionFromTcpProtocolHandler {
    public:
        ConnectionHandler(const shared_ptr<HttpFlowSinkApplication>& initSinkAppPtr)
            : sinkApplicationPtr(initSinkAppPtr) {}

        void HandleNewConnection(const shared_ptr<TcpConnection>& connectionPtr)
        {
            sinkApplicationPtr->AcceptTcpConnection(connectionPtr);
        }
    private:
        shared_ptr<HttpFlowSinkApplication> sinkApplicationPtr;

    };//ConnectionHandler//

    //-------------------------------------------
    class TcpEventHandler: public TcpConnection::AppTcpEventHandler {
    public:
        TcpEventHandler(const shared_ptr<HttpFlowSinkApplication>& initSinkAppPtr)
            : sinkApplicationPtr(initSinkAppPtr) {}

        void DoTcpIsReadyForMoreDataAction() { }

        void ReceiveDataBlock(
            const unsigned char dataBlock[],
            const unsigned int dataLength,
            const unsigned int actualDataLength,
            bool& stallIncomingDataFlow)
        {
            sinkApplicationPtr->ReceiveData(
                dataBlock, dataLength, actualDataLength);
            stallIncomingDataFlow = false;
        }

        void DoTcpRemoteHostClosedAction() {
            sinkApplicationPtr->tcpConnectionPtr->Close();
        }

    private:
        shared_ptr<HttpFlowSinkApplication> sinkApplicationPtr;

    };//TcpEventHandler//

    //-------------------------------------------

    void AcceptTcpConnection(const shared_ptr<TcpConnection>& connectionPtr);

    void ReceiveData(
        const unsigned char dataBlock[],
        const unsigned int dataLength,
        const unsigned int actualDataLength);

    void OutputTraceAndStatsForReceiveData(
        const unsigned int length);

    class FlowRequestReplyFielder: public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<HttpFlowSinkApplication>& initAppPtr) : appPtr(initAppPtr) { }

        void RequestAccepted(const FlowIdType& flowId)
            { appPtr->macQosFlowId = flowId; }
        void RequestDenied() { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<HttpFlowSinkApplication> appPtr;

    };//FlowRequestReplyFielder//

    void ReserveOrUnreserveBandwidth();
    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

    //----------------------------------

    shared_ptr<TcpConnection> tcpConnectionPtr;

    shared_ptr<CounterStatistic> bytesReceivedStatPtr;

};//HttpFlowSinkApplication//



inline
HttpFlowSinkApplication::HttpFlowSinkApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationIdType& initApplicationId,
    const NodeIdType& initSourceNodeId,
    const NodeIdType& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl)
    :
    HttpFlowApplication(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        initApplicationId,
        initSourceNodeId,
        initDestinationNodeId,
        initDefaultApplicationPortId,
        initEnableQosControl),
    bytesReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + "_" + initApplicationId + "_BytesReceived")))
{
}//HttpFlowSinkApplication//

// Two part initialization forced by shared_from_this().

inline
void HttpFlowSinkApplication::CompleteInitialization()
{
    shared_ptr<ConnectionHandler> connectionHandlerPtr(new ConnectionHandler(shared_from_this()));

    assert(transportLayerPtr->tcpPtr->PortIsAvailable(destinationPortId));

    transportLayerPtr->tcpPtr->OpenSpecificTcpPort(
        NetworkAddress::anyAddress,
        destinationPortId,
        connectionHandlerPtr);

    if (reserveBandwidthModeIsOn) {

        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        const TimeType minimumSetupTime = 1 * MILLI_SECOND;
        const TimeType reservationLeewayTime = 1000 * MILLI_SECOND;

        TimeType reservationTime;
        if ((currentTime + minimumSetupTime + reservationLeewayTime) <= flowStartTime) {
            reservationTime = (flowStartTime - reservationLeewayTime);
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
            (flowEndTime + reservationLeewayTime));

    }//if//

}//CompleteInitialization//

inline
void HttpFlowSinkApplication::DisconnectFromOtherLayers()
{
    transportLayerPtr->tcpPtr->DisconnectConnectionHandlerForPort(
        NetworkAddress::anyAddress, destinationPortId);

    if (tcpConnectionPtr != nullptr) {
        tcpConnectionPtr->ClearAppTcpEventHandler();
        (*this).tcpConnectionPtr.reset();
    }//if//

}//DisconnectFromOtherLayers//



inline
void HttpFlowSinkApplication::AcceptTcpConnection(const shared_ptr<TcpConnection>& connectionPtr)
{
    assert((tcpConnectionPtr == nullptr) && "Should only be one connection");

    tcpConnectionPtr = connectionPtr;

    shared_ptr<TcpEventHandler> tcpEventHandlerPtr(new TcpEventHandler(shared_from_this()));

    tcpConnectionPtr->SetAppTcpEventHandler(tcpEventHandlerPtr);

    if (useVirtualPayload) {
        tcpConnectionPtr->EnableVirtualPayload();
    }//if//
}


inline
void HttpFlowSinkApplication::OutputTraceAndStatsForReceiveData(const unsigned int length)
{
    bytesReceivedStatPtr->IncrementCounter(length);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveDataTraceRecord traceData;

            traceData.sourceNodeId = sourceNodeId;
            traceData.dataLengthBytes = length;

            assert(sizeof(traceData) == APPLICATION_RECEIVE_DATA_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "HttpRecv", traceData);
        }
        else {
            ostringstream outStream;

            outStream << "SrcN= " << sourceNodeId
                      << " RecvBytes= " << length;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "HttpRecv", outStream.str());

        }//if//
    }//if//
}


inline
void HttpFlowSinkApplication::ReceiveData(
    const unsigned char dataBlock[],
    const unsigned int dataLength,
    const unsigned int actualDataLength)
{
    OutputTraceAndStatsForReceiveData(dataLength);
}



inline
void HttpFlowSinkApplication::ReserveBandwidth()
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

        qosControlInterface.RequestDualFlowReservation(
            reservationScheme,
            schedulingScheme,
            flowPriority,
            qosMinBandwidth, qosMaxBandwidth,
            qosMinReverseBandwidth, qosMaxReverseBandwidth,
            sourceAddress, sourcePortId, destinationAddress, destinationPortId, IP_PROTOCOL_NUMBER_TCP,
            replyPtr);
    }//if//

}//ReserveBandwidth//


inline
void HttpFlowSinkApplication::ReserveBandwidthRequestDeniedAction()
{
    cerr << "Warning in HTTP with QoS application: Bandwidth request denied." << endl;
}


inline
void HttpFlowSinkApplication::UnreserveBandwidth()
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
void HttpFlowSinkApplication::ReserveOrUnreserveBandwidth()
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    if (currentTime < flowEndTime) {
        (*this).ReserveBandwidth();
    }
    else {
        (*this).UnreserveBandwidth();
    }//if//
}//ReserveOrUnreserveBandwidth//



}//namespace//

#endif
