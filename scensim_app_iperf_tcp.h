// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_APP_IPERF_TCP_H
#define SCENSIM_APP_IPERF_TCP_H

#include "scensim_netsim.h"
#include <algorithm>

namespace ScenSim {

using std::min;

class IperfTcpApplication
    : public Application, public enable_shared_from_this<IperfTcpApplication> {
public:
    static const string modelName;
    static const string modelNameClientOnly;
    static const string modelNameServerOnly;

    IperfTcpApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const ApplicationIdType& initApplicationId,
        const NodeIdType& initNodeIdWithParameter,
        const NodeIdType& initSourceNodeId,
        const NodeIdType& initDestinationNodeId,
        const unsigned short int initDefaultApplicationPortId,
        const bool initEnableQosControl);

    void CompleteInitialization();

    virtual ~IperfTcpApplication() {}

private:
    enum IperfTcpEventKind {
        START_CLIENT,
        END_CLIENT,
        START_SERVER,
        FLOW_RESERVATION
    };//IperfTcpEventKind//

    struct ClientHeader {
        int32_t flags;
        int32_t numThreads;
        int32_t mPort;
        int32_t bufferlen;
        int32_t mWinBand;
        int32_t mAmount;
    };//ClientHeader//

    template<typename T> static T& GetHeader(
        vector<unsigned char>& buffer, int offset);
    static void FillPattern(vector<unsigned char>& buffer, int length);

    class TcpEventHandler
        : public TcpConnection::AppTcpEventHandler {
    public:
        TcpEventHandler(
            const shared_ptr<IperfTcpApplication>& initIperfTcpApplicationPtr)
            : iperfTcpApplicationPtr(initIperfTcpApplicationPtr) {}

        void DoTcpIsReadyForMoreDataAction()
        {
            if (!iperfTcpApplicationPtr->IsServer()) {
                iperfTcpApplicationPtr->SendDataFromClient();
            }//if//

        }//DoTcpIsReadyForMoreDataAction//

        void ReceiveDataBlock(
            const unsigned char dataBlock[],
            const unsigned int dataLength,
            const unsigned int actualDataLength,
            bool& stallIncomingDataFlow)
        {
            if (iperfTcpApplicationPtr->IsServer()) {
                iperfTcpApplicationPtr->ReceiveDataOnServer(
                    dataBlock, dataLength, actualDataLength);
            }//if//

            stallIncomingDataFlow = false;

        }//ReceiveDataBlock//

        void DoTcpRemoteHostClosedAction()
        {
            if (iperfTcpApplicationPtr->IsServer()) {
                iperfTcpApplicationPtr->ReceiveEndOfDataOnServer();
            }//if//

        }//DoTcpRemoteHostClosedAction//

        void DoTcpLocalHostClosedAction()
        {
            if (iperfTcpApplicationPtr->IsServer()) {
                iperfTcpApplicationPtr->EndServer();
            }
            else {
                iperfTcpApplicationPtr->EndClient();
            }//if//

        }//DoTcpLocalHostClosedAction//

    private:
        shared_ptr<IperfTcpApplication> iperfTcpApplicationPtr;

    };//PacketHandler//

    class TcpConnectionHandler
        : public ConnectionFromTcpProtocolHandler {
    public:
        TcpConnectionHandler(
            const shared_ptr<IperfTcpApplication>& initIperfTcpApplicationPtr)
            : iperfTcpApplicationPtr(initIperfTcpApplicationPtr) {}

        void HandleNewConnection(
            const shared_ptr<TcpConnection>& newTcpConnectionPtr)
        {
            iperfTcpApplicationPtr->HandleNewConnection(newTcpConnectionPtr);

        }//HandleNewConnection//

    private:
        shared_ptr<IperfTcpApplication> iperfTcpApplicationPtr;

    };//TcpConnectionHandler//

    class IperfTcpEvent: public SimulationEvent {
    public:
        explicit
        IperfTcpEvent(
            const shared_ptr<IperfTcpApplication>& initIperfTcpApplicationPtr,
            const IperfTcpEventKind& initIperfTcpEventKind)
            :
            iperfTcpApplicationPtr(initIperfTcpApplicationPtr),
            iperfTcpEventKind(initIperfTcpEventKind)
        {}

        virtual void ExecuteEvent()
        {
            switch (iperfTcpEventKind) {
            case START_CLIENT:
                iperfTcpApplicationPtr->StartClient();
                break;
            case END_CLIENT:
                iperfTcpApplicationPtr->SendDataFromClient();
                break;
            case START_SERVER:
                iperfTcpApplicationPtr->StartServer();
                break;
            case FLOW_RESERVATION:
                iperfTcpApplicationPtr->ReserveBandwidth();
                break;
            }//switch//

        }//ExecuteEvent//

    private:
        shared_ptr<IperfTcpApplication> iperfTcpApplicationPtr;
        IperfTcpEventKind iperfTcpEventKind;

    };//IperfTcpEvent//

    class FlowRequestReplyFielder
        : public MacQualityOfServiceControlInterface::FlowRequestReplyFielder {
    public:
        FlowRequestReplyFielder(const shared_ptr<IperfTcpApplication>& initAppPtr)
            : appPtr(initAppPtr) {}
        void RequestAccepted(const FlowIdType& flowId)
            { appPtr->macQosFlowId = flowId; }
        void RequestDenied()
            { appPtr->ReserveBandwidthRequestDeniedAction(); }
    private:
        shared_ptr<IperfTcpApplication> appPtr;

    };//FlowRequestReplyFielder//

    string GetModelName() const;
    bool IsServer() const;
    bool IsServerApp() const;
    bool IsClientApp() const;
    void InitClientHeader();

    void StartClient();
    void EndClient();
    void SendDataFromClient();

    void StartServer();
    void EndServer();
    void HandleNewConnection(
        const shared_ptr<TcpConnection>& newTcpConnectionPtr);
    void ReceiveEndOfDataOnServer();
    void ReceiveDataOnServer(
        const unsigned char dataBlock[],
        const unsigned int dataLength,
        const unsigned int actualDataLength);

    void OutputTraceAndStatsForStart();
    void OutputTraceAndStatsForEnd();
    void OutputTraceAndStatsForSendData();
    void OutputTraceAndStatsForReceiveData(
        const unsigned int length);

    void ReserveBandwidth();
    void ReserveBandwidthRequestDeniedAction();
    void UnreserveBandwidth();

    string GetParameterNamePrefix() const {
        string parameterNamePrefix = "iperf-tcp";

        if (IsServerApp()) {
            parameterNamePrefix += "-server";
        }
        else if (IsClientApp()) {
            parameterNamePrefix += "-client";
        }//if//

        if (reserveBandwidthModeIsOn) {
            parameterNamePrefix += "-with-qos";
        }//if//

        return parameterNamePrefix;

    }//GetParameterNamePrefix//

    bool autoAddressModeEnabled;
    NetworkAddress specifiedDestinationAddress;
    NodeIdType sourceNodeId;
    NodeIdType destinationNodeId;
    unsigned short int sourcePortId;
    unsigned short int destinationPortId;
    PacketPriorityType priority;
    TimeType startTime;

    bool reserveBandwidthModeIsOn;
    double qosMinBandwidth;
    double qosMaxBandwidth;
    double qosMinReverseBandwidth;
    double qosMaxReverseBandwidth;
    FlowIdType macQosFlowId;
    MacQualityOfServiceControlInterface::SchedulingSchemeChoice schedulingScheme;
    MacQualityOfServiceControlInterface::ReservationSchemeChoice reservationScheme;

    bool useVirtualPayload;
    bool waitingClosing;
    bool timeModeEnabled;
    TimeType totalTime;
    int totalSendingBytes;
    int bufferLength;
    TimeType endTime;
    vector<unsigned char> buffer;
    shared_ptr<TcpConnection> tcpConnectionPtr;
    shared_ptr<TcpEventHandler> tcpEventHandlerPtr;
    shared_ptr<TcpConnectionHandler> tcpConnectionHandlerPtr;

    unsigned long long int accumulatedBytesSent;
    unsigned long long int accumulatedbytesAcked;
    shared_ptr<CounterStatistic> bytesSentStatPtr;
    shared_ptr<CounterStatistic> bytesAckedStatPtr;
    shared_ptr<CounterStatistic> bytesReceivedStatPtr;

};//IperfTcpApplication//

inline
IperfTcpApplication::IperfTcpApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const ApplicationIdType& initApplicationId,
    const NodeIdType& initNodeIdWithParameter,
    const NodeIdType& initSourceNodeId,
    const NodeIdType& initDestinationNodeId,
    const unsigned short int initDefaultApplicationPortId,
    const bool initEnableQosControl)
    :
    Application(initSimulationEngineInterfacePtr, initApplicationId),
    autoAddressModeEnabled(true),
    specifiedDestinationAddress(NetworkAddress::invalidAddress),
    sourceNodeId(initSourceNodeId),
    destinationNodeId(initDestinationNodeId),
    sourcePortId(initDefaultApplicationPortId),
    destinationPortId(initDefaultApplicationPortId),
    priority(0),
    startTime(ZERO_TIME),
    reserveBandwidthModeIsOn(initEnableQosControl),
    qosMinBandwidth(0.0),
    qosMaxBandwidth(0.0),
    qosMinReverseBandwidth(0.0),
    qosMaxReverseBandwidth(0.0),
    macQosFlowId(),
    schedulingScheme(MacQualityOfServiceControlInterface::DefaultSchedulingScheme),
    reservationScheme(MacQualityOfServiceControlInterface::OptimisticLinkRate),
    useVirtualPayload(false),
    waitingClosing(false),
    timeModeEnabled(true),
    totalTime(10 * SECOND),
    totalSendingBytes(1024 * 1024 * 10 / 8),
    bufferLength(128 * 1024),
    endTime(ZERO_TIME),
    buffer(),
    tcpConnectionPtr(),
    tcpEventHandlerPtr(),
    tcpConnectionHandlerPtr(),
    accumulatedBytesSent(0),
    accumulatedbytesAcked(0),
    bytesSentStatPtr(),
    bytesAckedStatPtr(),
    bytesReceivedStatPtr()
{
    const string parameterPrefix = GetParameterNamePrefix();

    priority = static_cast<PacketPriorityType>(
        parameterDatabaseReader.ReadNonNegativeInt(
            parameterPrefix + "-priority", initNodeIdWithParameter, applicationId));

    startTime = parameterDatabaseReader.ReadTime(
        parameterPrefix + "-start-time", initNodeIdWithParameter, applicationId);

    endTime = startTime + totalTime;

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-auto-port-mode", initNodeIdWithParameter, applicationId)) {

        if (!parameterDatabaseReader.ReadBool(
                parameterPrefix + "-auto-port-mode", initNodeIdWithParameter, applicationId)) {

            destinationPortId = static_cast<unsigned short int>(
                parameterDatabaseReader.ReadNonNegativeInt(
                    parameterPrefix + "-destination-port", initNodeIdWithParameter, applicationId));

            sourcePortId = destinationPortId;
        }//if//
    }
    else {
        if (IsClientApp() || IsServerApp()) {
            destinationPortId = static_cast<unsigned short int>(
                parameterDatabaseReader.ReadNonNegativeInt(
                    parameterPrefix + "-destination-port", initNodeIdWithParameter, applicationId));

            sourcePortId = destinationPortId;
        }//if//
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-auto-address-mode", initNodeIdWithParameter, applicationId)) {

        autoAddressModeEnabled = parameterDatabaseReader.ReadBool(
            parameterPrefix + "-auto-address-mode", initNodeIdWithParameter, applicationId);
    }
    else {
        if (IsClientApp()) {
            autoAddressModeEnabled = false;
        }//if//
    }//if//

    if (!autoAddressModeEnabled) {
        const string addressString = parameterDatabaseReader.ReadString(
            parameterPrefix + "-destination-address", initNodeIdWithParameter, applicationId);

        bool success;
        specifiedDestinationAddress.SetAddressFromString(addressString, success);
        if (!success) {
            cerr << parameterPrefix + "-destination-address(" << addressString
                 << ") is invalid." << endl;
            exit(1);
        }//if//
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-time-mode", initNodeIdWithParameter, applicationId)) {

        timeModeEnabled = parameterDatabaseReader.ReadBool(
            parameterPrefix + "-time-mode", initNodeIdWithParameter, applicationId);
    }//if//

    if (timeModeEnabled) {
        if (parameterDatabaseReader.ParameterExists(
            parameterPrefix + "-total-time", initNodeIdWithParameter, applicationId)) {

            totalTime = parameterDatabaseReader.ReadTime(
                parameterPrefix + "-total-time", initNodeIdWithParameter, applicationId);

            endTime = startTime + totalTime;
        }//if//
    }
    else {
        if (parameterDatabaseReader.ParameterExists(
            parameterPrefix + "-total-size-bytes", initNodeIdWithParameter, applicationId)) {

            totalSendingBytes = parameterDatabaseReader.ReadInt(
                parameterPrefix + "-total-size-bytes", initNodeIdWithParameter, applicationId);
        }//if//
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-buffer-size-bytes", initNodeIdWithParameter, applicationId)) {

        bufferLength = parameterDatabaseReader.ReadInt(
            parameterPrefix + "-buffer-size-bytes", initNodeIdWithParameter, applicationId);

        const int leastLength = sizeof(ClientHeader);
        if (bufferLength < leastLength) {
            cerr << parameterPrefix + "-buffer-size-bytes(" << bufferLength
                 << ") is too small." << endl
                 << parameterPrefix + "-buffer-size-bytes should be greater or equal to "
                 << leastLength << "." << endl;
            exit(1);
        }//if//
    }//if//

    if (initEnableQosControl) {
        qosMinBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-baseline-bandwidth-bytes", initNodeIdWithParameter, applicationId);

        qosMaxBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-max-bandwidth-bytes", initNodeIdWithParameter, applicationId);

        qosMinReverseBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-baseline-reverse-bandwidth-bytes", initNodeIdWithParameter, applicationId);

        qosMaxReverseBandwidth =
            parameterDatabaseReader.ReadDouble(
                parameterPrefix + "-max-reverse-bandwidth-bytes", initNodeIdWithParameter, applicationId);

        if (parameterDatabaseReader.ParameterExists(
                parameterPrefix + "-schedule-scheme", initNodeIdWithParameter, applicationId)) {

            const string schedulingSchemeChoiceString =
                parameterDatabaseReader.ReadString(
                    parameterPrefix + "-schedule-scheme", initNodeIdWithParameter, applicationId);

            bool succeeded;

            ConvertStringToSchedulingSchemeChoice(
                MakeLowerCaseString(schedulingSchemeChoiceString),
                succeeded,
                schedulingScheme);

            if (!succeeded) {
                cerr << "Error in " << GetModelName() << " Application: Scheduling Scheme not recognized in:" << endl;
                cerr << "      >" << schedulingSchemeChoiceString << endl;
                exit(1);
            }//if//
        }//if//
    }//if//

    if (parameterDatabaseReader.ParameterExists(
        parameterPrefix + "-use-virtual-payload", initNodeIdWithParameter, applicationId)) {

        useVirtualPayload = parameterDatabaseReader.ReadBool(
            parameterPrefix + "-use-virtual-payload", initNodeIdWithParameter, applicationId);
    }//if//

    FillPattern(buffer, bufferLength);

    if (!IsServerApp()) {
        bytesSentStatPtr =
            simulationEngineInterfacePtr->CreateCounterStat(
                GetModelName() + "_" + initApplicationId + "_BytesSent");
        bytesAckedStatPtr =
            simulationEngineInterfacePtr->CreateCounterStat(
                GetModelName() + "_" + initApplicationId + "_BytesAcked");
    }//if//

    if (!IsClientApp()) {
        bytesReceivedStatPtr =
            simulationEngineInterfacePtr->CreateCounterStat(
                GetModelName() + "_" + initApplicationId + "_BytesReceived");
    }//if//

}//IperfTcpApplication//

inline
void IperfTcpApplication::CompleteInitialization()
{
    tcpEventHandlerPtr = shared_ptr<TcpEventHandler>(
        new TcpEventHandler(shared_from_this()));
    tcpConnectionHandlerPtr = shared_ptr<TcpConnectionHandler>(
        new TcpConnectionHandler(shared_from_this()));

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    const TimeType adjustedStartTime = std::max(currentTime, startTime);

    if (reserveBandwidthModeIsOn) {
        const TimeType minimumSetupTime = 1 * MILLI_SECOND;
        const TimeType reservationLeewayTime = 1000 * MILLI_SECOND;

        TimeType reservationTime;
        if ((currentTime + minimumSetupTime + reservationLeewayTime) <= startTime) {
            reservationTime = (startTime - reservationLeewayTime);
        }
        else {
            reservationTime = (currentTime + minimumSetupTime);
        }//if//

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new IperfTcpEvent(shared_from_this(), FLOW_RESERVATION)),
            reservationTime);

        endTime += reservationLeewayTime;
    }//if//

    if (IsServer()) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(new IperfTcpEvent(shared_from_this(), START_SERVER)),
            adjustedStartTime);
    }
    else {
        if (timeModeEnabled) {
            if (adjustedStartTime < endTime) {
                simulationEngineInterfacePtr->ScheduleEvent(
                    unique_ptr<SimulationEvent>(new IperfTcpEvent(shared_from_this(), START_CLIENT)),
                    adjustedStartTime);
                simulationEngineInterfacePtr->ScheduleEvent(
                    unique_ptr<SimulationEvent>(new IperfTcpEvent(shared_from_this(), END_CLIENT)),
                    endTime);
            }//if//
        }
        else {
            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(new IperfTcpEvent(shared_from_this(), START_CLIENT)),
                adjustedStartTime);
        }//if//
    }//if//

}//CompleteInitialization//

template<typename T> inline
T& IperfTcpApplication::GetHeader(vector<unsigned char>& buffer, int offset)
{
    assert(buffer.size() >= offset + sizeof(T));
    return *reinterpret_cast<T*>(&buffer[offset]);

}//GetHeader//

inline
void IperfTcpApplication::FillPattern(vector<unsigned char>& buffer, int length)
{
    buffer.resize(length);

    while (length-- > 0) {
        buffer.at(length) = (length % 10) + '0';
    }//while//

}//FillPattern//

inline
string IperfTcpApplication::GetModelName() const
{
    if (IsClientApp()) {
        return modelNameClientOnly;
    }
    else if (IsServerApp()) {
        return modelNameServerOnly;
    }
    else {
        return modelName;
    }//if//

}//GetModelName//

inline
bool IperfTcpApplication::IsServer() const
{
    const NodeIdType nodeId =
        simulationEngineInterfacePtr->GetNodeId();

    return (nodeId == destinationNodeId);

}//IsServer//

inline
bool IperfTcpApplication::IsServerApp() const
{
    return (sourceNodeId == INVALID_NODEID);

}//IsServerApp//

inline
bool IperfTcpApplication::IsClientApp() const
{
    return (destinationNodeId == INVALID_NODEID);

}//IsClientApp//


inline
void IperfTcpApplication::InitClientHeader()
{
    ClientHeader& clientHeader =
        GetHeader<ClientHeader>(buffer, 0);

    clientHeader.flags = 0;
    clientHeader.numThreads = 0;
    clientHeader.mPort = 0;
    clientHeader.bufferlen = 0;
    clientHeader.mWinBand = 0;
    clientHeader.mAmount = 0;

}//InitClientHeader//

inline
void IperfTcpApplication::StartClient()
{
    InitClientHeader();

    OutputTraceAndStatsForStart();

    const NetworkAddress sourceAddress =
        networkAddressLookupInterfacePtr->LookupNetworkAddress(sourceNodeId);

    NetworkAddress destinationAddress;
    bool foundDestAddress = false;
    if (autoAddressModeEnabled) {
        networkAddressLookupInterfacePtr->LookupNetworkAddress(
            destinationNodeId, destinationAddress, foundDestAddress);
    }
    else {
        destinationAddress = specifiedDestinationAddress;
        foundDestAddress = true;
    }//if//

    if (foundDestAddress) {
        transportLayerPtr->tcpPtr->CreateOutgoingTcpConnection(
            sourceAddress, sourcePortId, destinationAddress, destinationPortId,
            priority, tcpEventHandlerPtr, tcpConnectionPtr);

        if (useVirtualPayload) {
            tcpConnectionPtr->EnableVirtualPayload();
        }//if//
    }
    else {
        //cannot find destination address (destination node may not be created yet)
        //Future: output trace and stat
        waitingClosing = true;
    }//if//

}//StartClient//

inline
void IperfTcpApplication::EndClient()
{
    UnreserveBandwidth();

}//EndClient//

inline
void IperfTcpApplication::SendDataFromClient()
{
    if (waitingClosing) {
        return;
    }//if//

    const TimeType currentTime =
        simulationEngineInterfacePtr->CurrentTime();

    const int deliveredBytes =
        static_cast<int>(tcpConnectionPtr->GetNumberOfDeliveredBytes());

    if ((timeModeEnabled && endTime <= currentTime) ||
        (!timeModeEnabled && totalSendingBytes <= deliveredBytes)) {
        OutputTraceAndStatsForSendData();
        OutputTraceAndStatsForEnd();
        accumulatedBytesSent += tcpConnectionPtr->GetNumberOfSentBytes();
        accumulatedbytesAcked += tcpConnectionPtr->GetNumberOfDeliveredBytes();
        tcpConnectionPtr->Close();
        waitingClosing = true;
        return;
    }//if//

    const int availableBufferBytes =
        static_cast<int>(tcpConnectionPtr->GetCurrentNumberOfAvailableBufferBytes());

    const int sentBytes =
        static_cast<int>(tcpConnectionPtr->GetNumberOfSentBytes());

    if ((availableBufferBytes == 0) ||
        (!timeModeEnabled && (totalSendingBytes <= sentBytes))) {
        OutputTraceAndStatsForSendData();
        return;
    }//if//

    int blockSizeBytes = min(bufferLength, availableBufferBytes);

    if (!timeModeEnabled && (totalSendingBytes < sentBytes + blockSizeBytes)) {
        blockSizeBytes -= sentBytes + blockSizeBytes - totalSendingBytes;
    }//if//

    shared_ptr<vector<unsigned char> > dataBlockPtr(new vector<unsigned char>());
    if (!useVirtualPayload) {
        dataBlockPtr->assign(buffer.begin(), buffer.begin() + blockSizeBytes);
    }//if//
    tcpConnectionPtr->SendDataBlock(dataBlockPtr, blockSizeBytes);

    SendDataFromClient();

}//SendDataFromClient//

inline
void IperfTcpApplication::StartServer()
{
    transportLayerPtr->tcpPtr->OpenSpecificTcpPort(
        NetworkAddress::anyAddress, destinationPortId, tcpConnectionHandlerPtr);

}//StartServer//

inline
void IperfTcpApplication::EndServer()
{
    UnreserveBandwidth();

}//EndServer//

inline
void IperfTcpApplication::HandleNewConnection(
    const shared_ptr<TcpConnection>& newTcpConnectionPtr)
{
    tcpConnectionPtr = newTcpConnectionPtr;
    tcpConnectionPtr->SetAppTcpEventHandler(tcpEventHandlerPtr);

    if (useVirtualPayload) {
        tcpConnectionPtr->EnableVirtualPayload();
    }//if//

}//HandleNewConnection//

inline
void IperfTcpApplication::ReceiveDataOnServer(
    const unsigned char dataBlock[],
    const unsigned int dataLength,
    const unsigned int actualDataLength)
{
    OutputTraceAndStatsForReceiveData(dataLength);

}//ReceiveDataOnServer//

inline
void IperfTcpApplication::ReceiveEndOfDataOnServer()
{
    accumulatedBytesSent += tcpConnectionPtr->GetNumberOfSentBytes();
    accumulatedbytesAcked += tcpConnectionPtr->GetNumberOfDeliveredBytes();
    tcpConnectionPtr->Close();

}//ReceiveEndOfDataOnServer//

inline
void IperfTcpApplication::OutputTraceAndStatsForStart()
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            IperfTcpApplicationStartRecord traceData;

            traceData.timeModeEnabled = timeModeEnabled;
            if (timeModeEnabled) {
                traceData.totalTimeOrBytes = totalTime;
            }
            else {
                traceData.totalTimeOrBytes = totalSendingBytes;
            }//if//

            assert(sizeof(traceData) == IPERF_APPLICATION_TCP_START_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                GetModelName(), applicationId, "IperfTcpStart", traceData);
        }
        else {
            ostringstream outStream;

            if (timeModeEnabled) {
                outStream << "TotalTime= " << ConvertTimeToStringSecs(totalTime);
            }
            else {
                outStream << "TotalBytes= " << totalSendingBytes;
            }//if//

            simulationEngineInterfacePtr->OutputTrace(
                GetModelName(), applicationId, "IperfTcpStart", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForStart//

inline
void IperfTcpApplication::OutputTraceAndStatsForEnd()
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    const double totalTime = ConvertTimeToDoubleSecs(currentTime - startTime);
    const int totalBytes = static_cast<int>(tcpConnectionPtr->GetNumberOfDeliveredBytes());

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            IperfTcpApplicationEndRecord traceData;

            traceData.totalTime = currentTime - startTime;
            traceData.totalBytes = totalBytes;

            assert(sizeof(traceData) == IPERF_APPLICATION_TCP_END_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                GetModelName(), applicationId, "IperfTcpEnd", traceData);
        }
        else {
            ostringstream outStream;

            outStream << "TotalTime= " << ConvertTimeToStringSecs(currentTime - startTime);
            outStream << " TotalBytes= " << tcpConnectionPtr->GetNumberOfDeliveredBytes();
            outStream << " Bps= " << 8 * totalBytes * SECOND / (currentTime - startTime);

            simulationEngineInterfacePtr->OutputTrace(
                GetModelName(), applicationId, "IperfTcpEnd", outStream.str());
        }//if//
    }//if//

}//OutputTraceAndStatsForEnd//

inline
void IperfTcpApplication::OutputTraceAndStatsForSendData()
{
    bytesSentStatPtr->UpdateCounter(
        accumulatedBytesSent + tcpConnectionPtr->GetNumberOfSentBytes());
    bytesAckedStatPtr->UpdateCounter(
        accumulatedbytesAcked + tcpConnectionPtr->GetNumberOfDeliveredBytes());

}//OutputTraceAndStatsForSendData//

inline
void IperfTcpApplication::OutputTraceAndStatsForReceiveData(const unsigned int length)
{
    bytesReceivedStatPtr->IncrementCounter(length);

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveDataTraceRecord traceData;

            traceData.sourceNodeId = sourceNodeId;
            traceData.dataLengthBytes = length;

            assert(sizeof(traceData) == APPLICATION_RECEIVE_DATA_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                GetModelName(), applicationId, "IperfTcpRecv", traceData);
        }
        else {
            ostringstream outStream;

            outStream << "SrcN= " << sourceNodeId
                      << " RecvBytes= " << length;

            simulationEngineInterfacePtr->OutputTrace(
                GetModelName(), applicationId, "IperfTcpRecv", outStream.str());

        }//if//
    }//if//

}//OutputTraceAndStatsForReceiveData//

inline
void IperfTcpApplication::ReserveBandwidth()
{
    if (!reserveBandwidthModeIsOn) {
        return;
    }//if//

    NetworkAddress sourceAddress;
    bool foundSourceAddress;

    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        sourceNodeId, sourceAddress, foundSourceAddress);

    if (!foundSourceAddress) {
        return;
    }//if//

    NetworkAddress destinationAddress;
    bool foundDestAddress;

    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        destinationNodeId, destinationAddress, foundDestAddress);

    if (!foundDestAddress) {
        return;
    }//if//

    const NetworkLayer& theNetworkLayer = *transportLayerPtr->GetNetworkLayerPtr();

    bool success;
    NetworkAddress nextHopAddress;
    unsigned int interfaceIndex;

    theNetworkLayer.GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress, success, nextHopAddress, interfaceIndex);

    if (success && theNetworkLayer.MacSupportsQualityOfService(interfaceIndex)) {
        MacQualityOfServiceControlInterface& qosControlInterface =
            *theNetworkLayer.GetMacQualityOfServiceInterface(interfaceIndex);

        shared_ptr<FlowRequestReplyFielder> replyPtr(new FlowRequestReplyFielder(shared_from_this()));

        if (IsServer()) {
            qosControlInterface.RequestDualFlowReservation(
                reservationScheme, schedulingScheme, priority, qosMinBandwidth, qosMaxBandwidth,
                qosMinReverseBandwidth, qosMaxReverseBandwidth,
                sourceAddress, sourcePortId, destinationAddress, destinationPortId, IP_PROTOCOL_NUMBER_TCP,
                replyPtr);
        }
        else {
            qosControlInterface.RequestDualFlowReservation(
                reservationScheme, schedulingScheme, priority, qosMinReverseBandwidth, qosMaxReverseBandwidth,
                qosMinBandwidth, qosMaxBandwidth,
                destinationAddress, destinationPortId, sourceAddress, sourcePortId, IP_PROTOCOL_NUMBER_TCP,
                replyPtr);
        }//if//
    }//if//

}//ReserveBandwidth//

inline
void IperfTcpApplication::ReserveBandwidthRequestDeniedAction()
{
    assert(reserveBandwidthModeIsOn);
    cerr << "Warning in IperfUdp with QoS application: Bandwidth request denied." << endl;

}//ReserveBandwidthRequestDeniedAction//

inline
void IperfTcpApplication::UnreserveBandwidth()
{
    if (!reserveBandwidthModeIsOn) {
        return;
    }//if//

    NetworkAddress destinationAddress;
    bool foundDestAddress;

    networkAddressLookupInterfacePtr->LookupNetworkAddress(
        destinationNodeId, destinationAddress, foundDestAddress);

    if (!foundDestAddress) {
        return;
    }//if//

    const NetworkLayer& theNetworkLayer = *transportLayerPtr->GetNetworkLayerPtr();

    bool success;
    NetworkAddress nextHopAddress;
    unsigned int interfaceIndex;

    theNetworkLayer.GetNextHopAddressAndInterfaceIndexForDestination(
        destinationAddress, success, nextHopAddress, interfaceIndex);

    if (success && theNetworkLayer.MacSupportsQualityOfService(interfaceIndex)) {
        MacQualityOfServiceControlInterface& qosControlInterface =
            *theNetworkLayer.GetMacQualityOfServiceInterface(interfaceIndex);

        qosControlInterface.DeleteFlow(macQosFlowId);
    }//if//

}//UnreserveBandwidth//

}//namespace//

#endif
