// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_NETSIM_H
#define SCENSIM_NETSIM_H

#include "scensim_application.h"
#include "scensim_transport.h"
#include "scensim_network.h"
#include "scensim_sensing.h"

namespace ScenSim {


class NetworkSimulator;

class NetworkAddressLookupInterface {
public:
    virtual NetworkAddress LookupNetworkAddress(const NodeIdType& nodeId) const = 0;
    virtual void LookupNetworkAddress(
        const NodeIdType& nodeId, NetworkAddress& networkAddress, bool& success) const = 0;

    virtual NodeIdType LookupNodeId(const NetworkAddress& aNetworkAddress) const = 0;
    virtual void LookupNodeId(
        const NetworkAddress& aNetworkAddress, NodeIdType& nodeId, bool& success) const = 0;

    // virtual NetworkAddress LookupNetworkAddress(const string& DNS_Name) const;

    virtual ~NetworkAddressLookupInterface() { }
};



class ExtrasimulationNetAddressLookup: public NetworkAddressLookupInterface {
public:
    ExtrasimulationNetAddressLookup(NetworkSimulator* initNetworkSimulatorPtr)
        : networkSimulatorPtr(initNetworkSimulatorPtr) {}

    NetworkAddress LookupNetworkAddress(const NodeIdType& nodeId) const;
    void LookupNetworkAddress(
        const NodeIdType& nodeId, NetworkAddress& networkAddress, bool& success) const;

    NodeIdType LookupNodeId(const NetworkAddress& aNetworkAddress) const;
    void LookupNodeId(
        const NetworkAddress& aNetworkAddress, NodeIdType& nodeId, bool& success) const;


    ~ExtrasimulationNetAddressLookup() { }
private:
    NetworkSimulator* networkSimulatorPtr;
};

//Jay: I believe that the NetworkNode/BasicNetworkNode distinction was caused by
//     previous design of external Qualnet connector.


class NetworkNode {
public:

    NetworkNode(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& theNodeId,
        const RandomNumberGeneratorSeedType& runSeed);

    virtual ~NetworkNode();

    NodeIdType GetNodeId() const { return nodeId; }
    string GetNodeTypeName() const { return nodeTypeName; }
    void SetNodeTypeName(const string& typeName) { nodeTypeName = typeName; }
    NetworkAddress GetPrimaryNetworkAddress() { return (GetNetworkLayerPtr()->GetPrimaryNetworkAddress()); }

    virtual shared_ptr<NetworkLayer> GetNetworkLayerPtr() const = 0;
    virtual shared_ptr<TransportLayer> GetTransportLayerPtr() const = 0;
    virtual shared_ptr<ApplicationLayer> GetAppLayerPtr() const = 0;

    virtual bool HasALocation() const { return false; }
    virtual const ObjectMobilityPosition GetCurrentLocation() const
    {  assert(false); abort(); return ObjectMobilityPosition(0,0,0,0,0,0,false,0); }

    virtual void CalculatePathlossToLocation(
        const PropagationInformationType& informationType,
        const unsigned int interfaceIndex,
        const double& positionXMeters,
        const double& positionYMeters,
        const double& positionZMeters,
        PropagationStatisticsType& propagationStatistics) const
    {
        assert(false); abort();
    }

    virtual void CalculatePathlossToNode(
        const PropagationInformationType& informationType,
        const unsigned int interfaceIndex,
        const ObjectMobilityPosition& rxAntennaPosition,
        const AntennaModel& rxAntennaModel,
        PropagationStatisticsType& propagationStatistics) const
    {
        assert(false); abort();
    }

    virtual bool HasAntenna(const InterfaceIdType& channelId) const { return false; }
    virtual shared_ptr<AntennaModel> GetAntennaModelPtr(const unsigned int interfaceIndex) const { return shared_ptr<AntennaModel>(); }

    virtual ObjectMobilityPosition GetAntennaLocation(const unsigned int interfaceIndex) const
    {  assert(false); abort(); return ObjectMobilityPosition(0,0,0,0,0,0,false,0); }

    virtual void OutputTraceForNodePosition(const TimeType& lastOutputTime) const;

    void OutputTraceForAddNode() const;
    void OutputTraceForDeleteNode() const;

    virtual void TriggerApplication() {};

protected:
    RandomNumberGeneratorSeedType GetNodeSeed() const { return nodeSeed; }
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;

    NodeIdType nodeId;
    string nodeTypeName;
    RandomNumberGeneratorSeedType nodeSeed;

private:
    //Disabled:
    NetworkNode(NetworkNode&);
    void operator=(NetworkNode&);

};//NetworkNode//


inline
NetworkNode::NetworkNode(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeIdType& initNodeId,
    const RandomNumberGeneratorSeedType& runSeed)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    nodeId(initNodeId),
    nodeSeed(HashInputsToMakeSeed(runSeed, initNodeId))
{
}

inline
NetworkNode::~NetworkNode()
{
    simulationEngineInterfacePtr->ShutdownThisInterface();
}

inline
void NetworkNode::OutputTraceForNodePosition(const TimeType& lastOutputTime) const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceMobility)) {

        const ObjectMobilityPosition& nodePosition = GetCurrentLocation();

        if ((lastOutputTime == ZERO_TIME) || (nodePosition.LastMoveTime() > lastOutputTime)) {

            if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

                NodePositionTraceRecord traceData;

                traceData.xPositionMeters = nodePosition.X_PositionMeters();
                traceData.yPositionMeters = nodePosition.Y_PositionMeters();
                traceData.theHeightFromGroundMeters = nodePosition.HeightFromGroundMeters();
                traceData.attitudeAzimuthDegrees = nodePosition.AttitudeAzimuthFromNorthClockwiseDegrees();
                traceData.attitudeElevationDegrees = nodePosition.AttitudeElevationFromHorizonDegrees();

                assert(sizeof(traceData) == NODE_POSITION_TRACE_RECORD_BYTES);

                simulationEngineInterfacePtr->OutputTraceInBinary(
                    "Node", "", "NodePosition", traceData);

            }
            else {
                ostringstream msgStream;

                msgStream << "X= " << nodePosition.X_PositionMeters();
                msgStream << " Y= " << nodePosition.Y_PositionMeters();
                msgStream << " Z= " << nodePosition.HeightFromGroundMeters();
                msgStream << " Azm= " << nodePosition.AttitudeAzimuthFromNorthClockwiseDegrees();
                msgStream << " Elv= " << nodePosition.AttitudeElevationFromHorizonDegrees();

                simulationEngineInterfacePtr->OutputTrace("Node", "", "NodePosition", msgStream.str());

            }//if//

        }//if//

    }//if//

}//OutputTraceForNodePosition//


inline
void NetworkNode::OutputTraceForAddNode() const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceMobility)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {
            simulationEngineInterfacePtr->OutputTraceInBinary("Node", "", "AddNode");
        }
        else {
            simulationEngineInterfacePtr->OutputTrace("Node", "", "AddNode", "");
        }//if//
    }//if//
}//OutputTraceForAddNode//


inline
void NetworkNode::OutputTraceForDeleteNode() const
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceMobility)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {
            simulationEngineInterfacePtr->OutputTraceInBinary("Node", "", "DeleteNode");
        }
        else {
            simulationEngineInterfacePtr->OutputTrace("Node", "", "DeleteNode", "");
        }//if//
    }//if//
}//OutputTraceForAddNode//


class BasicNetworkNode: public NetworkNode {
public:
    BasicNetworkNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        const NodeIdType& theNodeId,
        const RandomNumberGeneratorSeedType& runSeed);

    virtual ~BasicNetworkNode() {
        appLayerPtr->DisconnectFromOtherLayers();
        transportLayerPtr->DisconnectProtocolsFromOtherLayers();
        networkLayerPtr->DisconnectFromOtherLayers();
    }


    //virtual//
    shared_ptr<NetworkLayer> GetNetworkLayerPtr() const { return networkLayerPtr; }

    //virtual//
    shared_ptr<TransportLayer> GetTransportLayerPtr() const { return transportLayerPtr; }

    //virtual//
    shared_ptr<ApplicationLayer> GetAppLayerPtr() const { return appLayerPtr; }

    void CreateDynamicApplication(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const NodeIdType& sourceNodeId,
        const InterfaceOrInstanceIdType& instanceId);

protected:
    shared_ptr<NetworkLayer> networkLayerPtr;
    shared_ptr<TransportLayer> transportLayerPtr;
    shared_ptr<ApplicationLayer> appLayerPtr;
    shared_ptr<NetworkAddressLookupInterface> networkAddressLookupInterfacePtr;

    //Disabled:
    BasicNetworkNode(BasicNetworkNode&);
    void operator=(BasicNetworkNode&);

};//BasicNetworkNode//



inline
BasicNetworkNode::BasicNetworkNode(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
    const NodeIdType& initNodeId,
    const RandomNumberGeneratorSeedType& runSeed)
    :
    NetworkNode(
        initSimulationEngineInterfacePtr,
        initNodeId,
        runSeed),
    networkAddressLookupInterfacePtr(theGlobalNetworkingObjectBag.networkAddressLookupInterfacePtr)
{
    networkLayerPtr =
        shared_ptr<BasicNetworkLayer>(
            BasicNetworkLayer::CreateNetworkLayer(
                theParameterDatabaseReader,
                theGlobalNetworkingObjectBag,
                initSimulationEngineInterfacePtr,
                initNodeId,
                nodeSeed));

    transportLayerPtr = shared_ptr<TransportLayer>(
        new TransportLayer(
            theParameterDatabaseReader,
            initSimulationEngineInterfacePtr,
            networkLayerPtr,
            initNodeId,
            nodeSeed));

    appLayerPtr = shared_ptr<ApplicationLayer>(
        new ApplicationLayer(
            networkAddressLookupInterfacePtr,
            initSimulationEngineInterfacePtr,
            transportLayerPtr,
            initNodeMobilityModelPtr,
            initNodeId,
            nodeSeed));

    networkLayerPtr->SetupDhcpServerAndClientIfNessesary(
        theParameterDatabaseReader,
        simulationEngineInterfacePtr,
        nodeId,
        appLayerPtr);

    ApplicationMaker appMaker(
        simulationEngineInterfacePtr,
        appLayerPtr,
        nodeId,
        nodeSeed);

    const bool specifiedBasciApplicationFile =
        theParameterDatabaseReader.ParameterExists("basic-applications-file", nodeId);

    const bool specifiedConfigBasedApplicationFile =
        theParameterDatabaseReader.ParameterExists("config-based-application-file", nodeId);

    if (specifiedBasciApplicationFile || specifiedConfigBasedApplicationFile) {
        cerr << "\"basic-applications-file\" and \"config-based-application-file\" are old appplication specification parameter." << endl
             << "Specify applications in \".config\"" << endl
             << "To convert old application specification of \".app\", use application converter. Usage: bin/update_old_config" << endl;
        exit(1);
    }

    appMaker.ReadApplicationLineFromConfig(
        theParameterDatabaseReader,
        theGlobalNetworkingObjectBag);

}//BasicNetworkNode()

inline
void BasicNetworkNode::CreateDynamicApplication(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const NodeIdType& sourceNodeId,
    const InterfaceOrInstanceIdType& instanceId)
{
    ApplicationMaker appMaker(
        simulationEngineInterfacePtr,
        appLayerPtr,
        nodeId,
        nodeSeed);

    appMaker.ReadSpecificApplicationLineFromConfig(
        theParameterDatabaseReader,
        theGlobalNetworkingObjectBag,
        sourceNodeId,
        instanceId);

}//CreateDynamicApplication//


//=============================================================================

void OutputConfigBasedAppFile(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& fileName);

class NetworkSimulator {
public:
    NetworkSimulator(
        const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
        const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
        const RandomNumberGeneratorSeedType& runSeed,
        const bool initRunSequentially = true);

    virtual ~NetworkSimulator();
    void DeleteAllNodes() { nodes.clear(); }

    void GetListOfNodeIds(vector<NodeIdType>& nodeIds);

    virtual NetworkAddress LookupNetworkAddress(const NodeIdType& nodeId) const;
    virtual void LookupNetworkAddress(
        const NodeIdType& nodeId, NetworkAddress& networkAddress, bool& success) const;

    virtual NodeIdType LookupNodeId(const NetworkAddress& aNetworkAddress) const;
    virtual void LookupNodeId(
        const NetworkAddress& aNetworkAddress, NodeIdType& nodeId, bool& success) const;

    virtual unsigned int LookupInterfaceIndex(const NodeIdType& nodeId, const InterfaceIdType& interfaceName) const;

    virtual void CreateNewNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const string& nodeTypeName = "") { assert(false); abort(); }

    virtual void CreateNewNode(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const shared_ptr<ObjectMobilityModel>& nodeMobilityModelPtr,
        const string& nodeTypeName = "") { assert(false); abort(); }

    virtual void DeleteNode(const NodeIdType& nodeId) { (*this).RemoveNode(nodeId); }

    void InsertApplicationIntoANode(const NodeIdType& nodeId, const shared_ptr<Application>& appPtr);

    shared_ptr<MacLayerInterfaceForEmulation> GetMacLayerInterfaceForEmulation(
        const NodeIdType& nodeId) const;

    const GlobalNetworkingObjectBag& GetGlobalNetworkingObjectBag() const
        { return theGlobalNetworkingObjectBag; }

    virtual void CalculatePathlossFromNodeToLocation(
        const NodeIdType& nodeId,
        const PropagationInformationType& informationType,
        const unsigned int interfaceIndex,
        const double& positionXMeters,
        const double& positionYMeters,
        const double& positionZMeters,
        PropagationStatisticsType& propagationStatistics);

    virtual void CalculatePathlossFromNodeToNode(
        const NodeIdType& txNodeId,
        const NodeIdType& rxNodeId,
        const PropagationInformationType& informationType,
        const unsigned int txInterfaceIndex,
        const unsigned int rxInterfaceIndex,
        PropagationStatisticsType& propagationStatistics);

    virtual void OutputNodePositionsInXY(
        const TimeType lastOutputTime,
        std::ostream& nodePositionOutStream) const;

    virtual void OutputTraceForAllNodePositions(
        const TimeType& lastOutputTime) const;

    void OutputAllNodeIds(std::ostream& outStream) const;
    void OutputRecentlyAddedNodeIdsWithTypes(std::ostream& outStream);
    void OutputRecentlyDeletedNodeIds(std::ostream& outStream);

    void RunSimulationUntil(const TimeType& simulateUpToTime);

    void AddPropagationCalculationTraceIfNecessary(
        const InterfaceIdType& channelId,
        const shared_ptr<SimplePropagationLossCalculationModel>& propagationCalculationModelPtr);

    virtual const ObjectMobilityPosition GetNodePosition(const NodeIdType& nodeId)
    {   return (nodes[nodeId]->GetCurrentLocation()); }

    virtual const ObjectMobilityPosition GetAntennaLocation(const NodeIdType& nodeId, const unsigned int interfaceIndex);

    virtual void TriggerApplication(const NodeIdType& nodeId)
    {
        typedef map<NodeIdType, shared_ptr<NetworkNode> >::iterator IterType;
        IterType iter = nodes.find(nodeId);

        if (iter != nodes.end()) {
            iter->second->TriggerApplication();
        }//if//

    }//TriggerApplication//

    TimeType GetTimeStepEventSynchronizationStep() const { return timeStepEventSynchronizationStep; }

protected:
    virtual void CompleteSimulatorConstruction();
    virtual bool SupportMultiAgent() const { return false; }

    void AddNode(const shared_ptr<NetworkNode>& aNodePtr);

    void RemoveNode(const NodeIdType& nodeId);

    void SetupStatOutputFile();

    void CheckTheNecessityOfMultiAgentSupport();

    virtual void ExecuteTimestepBasedEvent();

    shared_ptr<SimulationEngine> theSimulationEnginePtr;

    RandomNumberGeneratorSeedType runSeed; //seed for communication system
    RandomNumberGeneratorSeedType mobilitySeed;

    GlobalNetworkingObjectBag theGlobalNetworkingObjectBag;

    shared_ptr<ParameterDatabaseReader> theParameterDatabaseReaderPtr;
    shared_ptr<GisSubsystem> theGisSubsystemPtr;
    shared_ptr<SensingSubsystem> theSensingSubsystemPtr;
    InorderFileCache mobilityFileCache;

    TimeType timeStepEventSynchronizationStep;

    map<NodeIdType, shared_ptr<NetworkNode> > nodes;

    class NodeEnterEvent : public SimulationEvent {
    public:
        NodeEnterEvent(
            NetworkSimulator* initNetworkSimulator,
            const NodeIdType& initNodeId,
            const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr)
            :
            networkSimulator(initNetworkSimulator),
            nodeId(initNodeId),
            nodeMobilityModelPtr(initNodeMobilityModelPtr)
        {}
        virtual void ExecuteEvent() {
            networkSimulator->CreateNewNode(
                *networkSimulator->theParameterDatabaseReaderPtr, nodeId, nodeMobilityModelPtr);
        }

    private:
        NetworkSimulator* networkSimulator;
        NodeIdType nodeId;
        shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;
    };

    class NodeLeaveEvent : public SimulationEvent {
    public:
        NodeLeaveEvent(
            NetworkSimulator* initNetworkSimulator,
            const NodeIdType& initNodeId)
            :
            networkSimulator(initNetworkSimulator),
            nodeId(initNodeId)
        {}
        virtual void ExecuteEvent() { networkSimulator->DeleteNode(nodeId); }

    private:
        NetworkSimulator* networkSimulator;
        NodeIdType nodeId;
    };

private:

    vector<NodeIdType> recentlyAddedNodeList;
    vector<NodeIdType> recentlyDeletedNodeList;

    StatViewCollection fileControlledStatViews;
    string statsOutputFilename;
    bool noDataOutputIsEnabled;

    bool runSequentially;
    unsigned int nextSynchronizationTimeStep;

    struct PropagationTraceOutputInfo {
        InterfaceIdType channelId;
        shared_ptr<SimplePropagationLossCalculationModel> propagationCalculationModelPtr;
        string outputFileName;

        PropagationTraceOutputInfo(
            const InterfaceIdType& initChannelId,
            const shared_ptr<SimplePropagationLossCalculationModel>& initPropagationCalculationModelPtr,
            const string& initOutputFileName)
            :
            channelId(initChannelId),
            propagationCalculationModelPtr(initPropagationCalculationModelPtr),
            outputFileName(initOutputFileName)
        {}
    };
    vector<PropagationTraceOutputInfo> propagationTraceOutputInfos;

    void OutputPropagationTrace(const TimeType& time);

};//NetworkSimulator//


inline
NetworkSimulator::NetworkSimulator(
    const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const RandomNumberGeneratorSeedType& initRunSeed,
    const bool initRunSequentially)
    :
    theSimulationEnginePtr(initSimulationEnginePtr),
    runSeed(initRunSeed),
    mobilitySeed(initRunSeed),
    theParameterDatabaseReaderPtr(initParameterDatabaseReaderPtr),
    theGisSubsystemPtr(new GisSubsystem(*theParameterDatabaseReaderPtr, initSimulationEnginePtr)),
    timeStepEventSynchronizationStep(INFINITE_TIME),
    runSequentially(initRunSequentially),
    nextSynchronizationTimeStep(0)
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    theGisSubsystemPtr->LoadShapeFiles(theParameterDatabaseReader);
    theGisSubsystemPtr->SynchronizeTopology(ZERO_TIME);

    theSensingSubsystemPtr.reset(
        new SensingSubsystem(
            shared_ptr<NetworkSimulatorInterfaceForSensingSubsystem>(
                new NetworkSimulatorInterfaceForSensingSubsystem(this)),
            theGisSubsystemPtr,
            runSeed)),

    theGlobalNetworkingObjectBag.networkAddressLookupInterfacePtr =
        shared_ptr<NetworkAddressLookupInterface>(new ExtrasimulationNetAddressLookup(this));

    theGlobalNetworkingObjectBag.abstractNetworkPtr =
        shared_ptr<AbstractNetwork>(new AbstractNetwork(theParameterDatabaseReader));

    theGlobalNetworkingObjectBag.bitOrBlockErrorRateCurveDatabasePtr.reset(
            new BitOrBlockErrorRateCurveDatabase());

    theGlobalNetworkingObjectBag.sensingSubsystemInterfacePtr =
        theSensingSubsystemPtr->CreateSubsystemInterfacePtr();

    string antennaFileName;
    if (theParameterDatabaseReader.ParameterExists("custom-antenna-file")) {
        antennaFileName = theParameterDatabaseReader.ReadString("custom-antenna-file");
    }//if//

    bool useLegacyAntennaPatternFileFormatMode = false;

    if (theParameterDatabaseReader.ParameterExists("antenna-patterns-are-in-legacy-format")) {
        useLegacyAntennaPatternFileFormatMode =
            theParameterDatabaseReader.ReadBool("antenna-patterns-are-in-legacy-format");
    }//if//

    string antennaPatternDebugDumpFileName;
    if (theParameterDatabaseReader.ParameterExists("antenna-pattern-debug-dump-file")) {
        antennaPatternDebugDumpFileName=
            theParameterDatabaseReader.ReadString("antenna-pattern-debug-dump-file");
    }//if//

    int two2dTo3dInterpolationAlgorithmNumber = 1;
    if (theParameterDatabaseReader.ParameterExists(
        "antenna-pattern-two-2d-to-3d-interpolation-algorithm-number")) {

        two2dTo3dInterpolationAlgorithmNumber =
            theParameterDatabaseReader.ReadInt(
                "antenna-pattern-two-2d-to-3d-interpolation-algorithm-number");
    }//if//

    theGlobalNetworkingObjectBag.antennaPatternDatabasePtr.reset(
        new AntennaPatternDatabase(
            antennaFileName,
            useLegacyAntennaPatternFileFormatMode,
            two2dTo3dInterpolationAlgorithmNumber,
            antennaPatternDebugDumpFileName));

    if (theParameterDatabaseReader.ParameterExists("time-step-event-synchronization-step")) {
        timeStepEventSynchronizationStep = theParameterDatabaseReader.ReadTime("time-step-event-synchronization-step");
    }//if//

    (*this).SetupStatOutputFile();

    if (theParameterDatabaseReader.ParameterExists("mobility-seed")) {
        mobilitySeed = theParameterDatabaseReader.ReadInt("mobility-seed");
    }//if//

}//NetworkSimulator//

inline
void NetworkSimulator::AddNode(const shared_ptr<NetworkNode>& aNodePtr)
{
    const NodeIdType nodeId = aNodePtr->GetNodeId();

    assert(nodes.find(nodeId) == nodes.end());
    nodes.insert(make_pair(nodeId, aNodePtr));
    recentlyAddedNodeList.push_back(nodeId);
    aNodePtr->OutputTraceForAddNode();
    theSensingSubsystemPtr->AddNode(nodeId);
}//AddNode//

inline
void NetworkSimulator::RemoveNode(const NodeIdType& nodeId)
{
    nodes[nodeId]->OutputTraceForDeleteNode();
    nodes[nodeId].reset();
    nodes.erase(nodeId);
    recentlyDeletedNodeList.push_back(nodeId);

    theGisSubsystemPtr->RemoveMovingObject(nodeId);
    theSensingSubsystemPtr->RemoveNode(nodeId);
}//RemoveNode//


inline
void NetworkSimulator::RunSimulationUntil(const TimeType& simulateUpToTime)
{
    if (timeStepEventSynchronizationStep == INFINITE_TIME) {

        // Output positions for the first time step.
        if (theSimulationEnginePtr->CurrentTime() == ZERO_TIME &&
            nextSynchronizationTimeStep == 0) {

            (*this).OutputTraceForAllNodePositions(ZERO_TIME);
            nextSynchronizationTimeStep++;
        }//if//

    } else {

        const unsigned int endSynchronizationTimeStep =
            static_cast<unsigned int>(std::floor(double(simulateUpToTime) / timeStepEventSynchronizationStep));

        const unsigned int startSynchronizationTimeStep = nextSynchronizationTimeStep;

        TimeType lastPositionOutputTime = ZERO_TIME;

        for(unsigned int i = startSynchronizationTimeStep; i <= endSynchronizationTimeStep; i++) {

            const TimeType halfwaySimulationTime = timeStepEventSynchronizationStep * i;

            if (runSequentially) {
                theSimulationEnginePtr->RunSimulationSequentially(halfwaySimulationTime);
            }
            else {
                theSimulationEnginePtr->RunSimulationInParallel(halfwaySimulationTime);
            }//if//

            (*this).ExecuteTimestepBasedEvent();

            (*this).OutputTraceForAllNodePositions(lastPositionOutputTime);

            lastPositionOutputTime = halfwaySimulationTime;
            nextSynchronizationTimeStep = i+1;
        }//for//
    }//if//

    if (runSequentially) {
        theSimulationEnginePtr->RunSimulationSequentially(simulateUpToTime);
    }
    else {
        theSimulationEnginePtr->RunSimulationInParallel(simulateUpToTime);
    }//if//

}//RunSimulationUntil//

inline
void NetworkSimulator::ExecuteTimestepBasedEvent()
{
    const TimeType currentTime = theSimulationEnginePtr->CurrentTime();

    theGisSubsystemPtr->SynchronizeTopology(currentTime);

    theSensingSubsystemPtr->ExecuteTimestepBasedEvent(currentTime);

    (*this).OutputPropagationTrace(currentTime);

}//ExecuteTimestepBasedEvent//

inline
void NetworkSimulator::GetListOfNodeIds(vector<NodeIdType>& nodeIds)
{
    typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;
    nodeIds.clear();

    for(IterType iter = nodes.begin(); (iter != nodes.end()); ++iter) {
        nodeIds.push_back(iter->first);
    }//for//

}//GetListOfNodeIds//


inline
NetworkAddress NetworkSimulator::LookupNetworkAddress(const NodeIdType& nodeId) const
{
    typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;

    IterType iterPosition = nodes.find(nodeId);

    if (iterPosition == nodes.end()) {
        cerr << "Error: Network Node Id: " << nodeId << " Not Found." << endl;
        exit(1);
    }//if//

    return (iterPosition->second->GetPrimaryNetworkAddress());
}


inline
void NetworkSimulator::LookupNetworkAddress(
    const NodeIdType& nodeId, NetworkAddress& networkAddress, bool& success) const
{

    typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;

    IterType iterPosition = nodes.find(nodeId);

    if (iterPosition != nodes.end()) {
        success = true;
        networkAddress = (iterPosition->second->GetPrimaryNetworkAddress());
    }
    else {
        success = false;
    }//if//

}


inline
NodeIdType NetworkSimulator::LookupNodeId(const NetworkAddress& aNetworkAddress) const
{
    typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;

    for(IterType iter = nodes.begin(); (iter != nodes.end()); iter++) {
        if (iter->second->GetPrimaryNetworkAddress() == aNetworkAddress) {
            return (iter->first);
        }//if//
    }//for//

    cerr << "Error in NetworkSimulator::LookupNodeId: Network Address "
         << aNetworkAddress.ConvertToString() << " Not Found." << endl;
    exit(1);
}


inline
void NetworkSimulator::LookupNodeId(
    const NetworkAddress& aNetworkAddress, NodeIdType& nodeId, bool& success) const
{
    success = false;

    typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;

    for(IterType iter = nodes.begin(); (iter != nodes.end()); iter++) {
        if (iter->second->GetPrimaryNetworkAddress() == aNetworkAddress) {
            success = true;
            nodeId = (iter->first);
            return;
        }//if//
    }//for//
}


inline
unsigned int NetworkSimulator::LookupInterfaceIndex(const NodeIdType& nodeId, const InterfaceIdType& interfaceName) const
{
    typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;

    IterType iter = nodes.find(nodeId);

    if (iter == nodes.end()) {
        cerr << "Error: Network Node Id: " << nodeId << " Not Found." << endl;
        exit(1);
    }//if

    return iter->second->GetNetworkLayerPtr()->LookupInterfaceIndex(interfaceName);
}

inline
const ObjectMobilityPosition NetworkSimulator::GetAntennaLocation(
    const NodeIdType& nodeId,
    const unsigned int interfaceIndex)
{
    ObjectMobilityPosition antennaPosition =
        nodes[nodeId]->GetAntennaLocation(interfaceIndex);

    if (!antennaPosition.TheHeightContainsGroundHeightMeters()) {
        const Vertex antennaVertex(
            antennaPosition.X_PositionMeters(),
            antennaPosition.Y_PositionMeters(),
            antennaPosition.HeightFromGroundMeters());

        const shared_ptr<const GroundLayer> groundLayerPtr =
            theGisSubsystemPtr->GetGroundLayerPtr();

        const double groundMeters =
            groundLayerPtr->GetElevationMetersAt(antennaVertex);

        antennaPosition.SetHeightFromGroundMeters(
            antennaPosition.HeightFromGroundMeters() + groundMeters);
    }

    return antennaPosition;
}

inline
void NetworkSimulator::InsertApplicationIntoANode(
    const NodeIdType& nodeId,
    const shared_ptr<Application>& appPtr)
{
    nodes[nodeId]->GetAppLayerPtr()->AddApp(appPtr);
}

inline
shared_ptr<MacLayerInterfaceForEmulation>
    NetworkSimulator::GetMacLayerInterfaceForEmulation(const NodeIdType& nodeId) const
{
    typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;

    IterType iter = nodes.find(nodeId);

    if (iter == nodes.end()) {
        cerr << "Error: Network Node Id: " << nodeId << " Not Found." << endl;
        exit(1);
    }//if//

    const NetworkLayer& theNetworkLayer = *iter->second->GetNetworkLayerPtr();

    if (theNetworkLayer.NumberOfInterfaces() != 1) {
        cerr << "Emulation Error: The number of MAC layers for " << nodeId << " is not equal to 1." << endl;
        exit(1);
    }//if//

    return (theNetworkLayer.GetMacLayerPtr(0)->GetMacLayerInterfaceForEmulation());

}//GetMacLayerInterfaceForEmulation//


inline
NetworkAddress ExtrasimulationNetAddressLookup::LookupNetworkAddress(
    const NodeIdType& nodeId) const
{
    return networkSimulatorPtr->LookupNetworkAddress(nodeId);
}

inline
void ExtrasimulationNetAddressLookup::LookupNetworkAddress(
    const NodeIdType& nodeId, NetworkAddress& networkAddress, bool& success) const
{
    networkSimulatorPtr->LookupNetworkAddress(nodeId, networkAddress, success);
}

inline
NodeIdType ExtrasimulationNetAddressLookup::LookupNodeId(
    const NetworkAddress& aNetworkAddress) const
{
    return networkSimulatorPtr->LookupNodeId(aNetworkAddress);
}

inline
void ExtrasimulationNetAddressLookup::LookupNodeId(
    const NetworkAddress& aNetworkAddress, NodeIdType& nodeId, bool& success) const
{
    return networkSimulatorPtr->LookupNodeId(aNetworkAddress, nodeId, success);
}

class SimpleAccessPointFinder : public AccessPointFinderInterface {
public:
    void LookupAccessPointFor(
        const NetworkAddress& destinationAddress,
        bool& foundTheAccessPoint,
        NetworkAddress& accessPointAddress);

    void SetAccessPointFor(
        const NetworkAddress& nodeAddress,
        const NetworkAddress& accessPointAddress)
    {
        nodeToAccessPointMap[nodeAddress] = accessPointAddress;
    }

    void ClearAccessPointFor(const NetworkAddress& nodeAddress);


private:
    std::map<NetworkAddress, NetworkAddress> nodeToAccessPointMap;

};//SimpleAccessPointFinder//

inline
void SimpleAccessPointFinder::LookupAccessPointFor(
    const NetworkAddress& destinationAddress,
    bool& foundTheAccessPoint,
    NetworkAddress& accessPointAddress)
{
    typedef std::map<NetworkAddress, NetworkAddress>::iterator IterType;

    foundTheAccessPoint = false;

    IterType iter = nodeToAccessPointMap.find(destinationAddress);
    if (iter != nodeToAccessPointMap.end()) {

        foundTheAccessPoint = true;
        accessPointAddress = iter->second;
    }//if/

}//LookupAccessPointFor//


inline
void SimpleAccessPointFinder::ClearAccessPointFor(const NetworkAddress& nodeAddress)
{
    typedef std::map<NetworkAddress, NetworkAddress>::iterator IterType;

    IterType iter = nodeToAccessPointMap.find(nodeAddress);
    if (iter != nodeToAccessPointMap.end()) {
        nodeToAccessPointMap.erase(iter);
    }//if/

}//ClearAccessPointFor//


}//namespace//


#endif
