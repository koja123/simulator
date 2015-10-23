// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include <iomanip>
#include "scensim_netsim.h"
#include "scenargiesim.h"


namespace ScenSim {

using std::setw;
using std::ofstream;


const PacketIdType PacketIdType::nullPacketId = PacketIdType();


shared_ptr<AntennaModel> CreateAntennaModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceOrInstanceIdType& interfaceId,
    const AntennaPatternDatabase& anAntennaPatternDatabase)
{
    string antennaModelString = theParameterDatabaseReader.ReadString("antenna-model", nodeId, interfaceId);
    ConvertStringToLowerCase(antennaModelString);

    if ((antennaModelString == "omni") || (antennaModelString == "omnidirectional")) {

        double antennaGainDbi;

        if (theParameterDatabaseReader.ParameterExists("max-antenna-gain-dbi", nodeId, interfaceId)) {
            antennaGainDbi =
                theParameterDatabaseReader.ReadDouble("max-antenna-gain-dbi", nodeId, interfaceId);

            if (theParameterDatabaseReader.ParameterExists("antenna-gain-dbi", nodeId, interfaceId)) {
                cerr << "Error in configuration file: antenna-gain-dbi and max-antenna-gain-dbi both defined." << endl;
                exit(1);
            }//if//
        }
        else {
            antennaGainDbi =
                theParameterDatabaseReader.ReadDouble("antenna-gain-dbi", nodeId, interfaceId);
        }//if//

        return (shared_ptr<AntennaModel>(new OmniAntennaModel(antennaGainDbi)));
    }
    else if (antennaModelString == "sectored") {
        const double antennaGainDbi =
            theParameterDatabaseReader.ReadDouble("max-antenna-gain-dbi", nodeId, interfaceId);

        return (shared_ptr<AntennaModel>(new SectoredAntennaModel(antennaGainDbi)));
    }
    else if ((antennaModelString == "fupm") ||
             (antennaModelString == "hfpm") ||
             (antennaModelString == "fupm/hfpm")) {
        //use InSight .uan file
        return (shared_ptr<AntennaModel>(new OmniAntennaModel(0.0)));
    }
    else if (anAntennaPatternDatabase.IsDefined(antennaModelString)) {

        if (theParameterDatabaseReader.ParameterExists(
            "antenna-model-quasi-omni-mode-gain-dbi", nodeId, interfaceId)) {

            const double quasiOmniAntennaGainDbi =
                theParameterDatabaseReader.ReadDouble(
                    "antenna-model-quasi-omni-mode-gain-dbi", nodeId, interfaceId);

            return (shared_ptr<AntennaModel>(
                new CustomAntennaModel(
                    anAntennaPatternDatabase,
                    antennaModelString,
                    quasiOmniAntennaGainDbi)));
        }
        else {
            return (shared_ptr<AntennaModel>(
                new CustomAntennaModel(
                    anAntennaPatternDatabase,
                    antennaModelString)));
        }//if//
    }
    else {
        cerr << "Antenna Model: " << antennaModelString << " is invalid." << endl;
        exit(1);
    }//if//

}//CreateAntennaModel//



NetworkSimulator::~NetworkSimulator()
{
    assert(nodes.empty() &&
           "Error: All Nodes must be deleted sometime before the calling of NetworkSimulator"
           " destructor (base class) (can call DeleteAllNodes() in child's destructor)");

    if (!statsOutputFilename.empty()) {

        const TimeType currentTime = theSimulationEnginePtr->CurrentTime();

        OutputStatsToFile(statsOutputFilename, fileControlledStatViews, noDataOutputIsEnabled, ZERO_TIME, currentTime);
    }//if//
}


void NetworkSimulator::CalculatePathlossFromNodeToLocation(
    const NodeIdType& txNodeId,
    const PropagationInformationType& informationType,
    const unsigned int interfaceIndex,
    const double& positionXMeters,
    const double& positionYMeters,
    const double& positionZMeters,
    PropagationStatisticsType& propagationStatistics)
{
    nodes[txNodeId]->CalculatePathlossToLocation(
        informationType,
        interfaceIndex,
        positionXMeters,
        positionYMeters,
        positionZMeters,
        propagationStatistics);
}


void NetworkSimulator::CalculatePathlossFromNodeToNode(
    const NodeIdType& txNodeId,
    const NodeIdType& rxNodeId,
    const PropagationInformationType& informationType,
    const unsigned int txInterfaceIndex,
    const unsigned int rxInterfaceIndex,
    PropagationStatisticsType& propagationStatistics)
{
    const shared_ptr<NetworkNode> rxNodePtr = nodes[rxNodeId];

    nodes[txNodeId]->CalculatePathlossToNode(
        informationType,
        txInterfaceIndex,
        rxNodePtr->GetAntennaLocation(rxInterfaceIndex),
        *rxNodePtr->GetAntennaModelPtr(rxInterfaceIndex),
        propagationStatistics);

}//CalculatePathlossFromNodeToNode//



void NetworkSimulator::OutputNodePositionsInXY(
    const TimeType lastOutputTime,
    std::ostream& nodePositionOutStream) const
{
    typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;

    for(IterType iter = nodes.begin(); (iter != nodes.end()); ++iter) {
        const NodeIdType& nodeId = iter->first;
        const NetworkNode& node = *iter->second;

        const ObjectMobilityPosition nodePosition = node.GetCurrentLocation();

        // Don't output stationary nodes.

        if ((lastOutputTime == ZERO_TIME) || (nodePosition.LastMoveTime() > lastOutputTime)) {

            nodePositionOutStream.precision(10);

            nodePositionOutStream << ' ' << nodeId << ' '
                                  << setw(11) << nodePosition.X_PositionMeters() << ' '
                                  << setw(11) << nodePosition.Y_PositionMeters() << ' '
                                  << nodePosition.HeightFromGroundMeters() << ' '
                                  << nodePosition.AttitudeAzimuthFromNorthClockwiseDegrees() << ' '
                                  << nodePosition.AttitudeElevationFromHorizonDegrees();

        }//if//
    }//for//
    nodePositionOutStream.flush();

}//OutputNodePositionsInXY//



void NetworkSimulator::OutputTraceForAllNodePositions(const TimeType& lastOutputTime) const
{

    typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;

    for(IterType iter = nodes.begin(); (iter != nodes.end()); ++iter) {

        const NodeIdType& nodeId = iter->first;
        const NetworkNode& node = *iter->second;

        node.OutputTraceForNodePosition(lastOutputTime);

    }//for//

}//OutputTraceForAllNodePositions//



void NetworkSimulator::OutputAllNodeIds(std::ostream& outStream) const
{
    typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;

    for(IterType iter = nodes.begin(); (iter != nodes.end()); ++iter) {
        outStream << ' ' << iter->first;
    }//for//
}



void NetworkSimulator::OutputRecentlyAddedNodeIdsWithTypes(std::ostream& outStream)
{
    for(unsigned int i = 0; (i < recentlyAddedNodeList.size()); i++) {

        string nodeTypeName = nodes[recentlyAddedNodeList[i]]->GetNodeTypeName();
        if (nodeTypeName == "") {
            nodeTypeName = "no_node_type_name";
        }//if//

        outStream << ' ' << recentlyAddedNodeList[i] << ' ' << nodeTypeName;
    }//for//
    recentlyAddedNodeList.clear();

}//OutputRecentlyAddedNodeIdsWithTypes//



void NetworkSimulator::OutputRecentlyDeletedNodeIds(std::ostream& outStream)
{
    for(unsigned int i = 0; (i < recentlyDeletedNodeList.size()); i++) {
        outStream << ' ' << recentlyDeletedNodeList[i];
    }//for//
    recentlyDeletedNodeList.clear();
}

void NetworkSimulator::CompleteSimulatorConstruction()
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    set<NodeIdType> setOfNodeIds;
    theParameterDatabaseReader.MakeSetOfAllCommNodeIds(setOfNodeIds);

    typedef set<NodeIdType>::const_iterator IterType;

    for(IterType iter = setOfNodeIds.begin(); iter != setOfNodeIds.end(); ++iter) {

        const NodeIdType nodeId = (*iter);

        assert(nodeId <= MAX_COMMUNICATION_NODEID);

        unsigned int partitionIndex = 0;

        if (theParameterDatabaseReader.ParameterExists("parallelization-partition-index", nodeId)) {
            partitionIndex =
                (theParameterDatabaseReader.ReadNonNegativeInt("parallelization-partition-index", nodeId) %
                 theSimulationEnginePtr->GetNumberPartitionThreads());
        }//if//

        shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr(
            theSimulationEnginePtr->GetSimulationEngineInterface(
                theParameterDatabaseReader, nodeId, partitionIndex));

        shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr =
            CreateAntennaMobilityModel(
                theParameterDatabaseReader,
                nodeId,
                nullInstanceId,
                mobilitySeed,
                mobilityFileCache,
                theGisSubsystemPtr);

        const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
        const TimeType creationTime = nodeMobilityModelPtr->GetCreationTime();
        const TimeType deletionTime = nodeMobilityModelPtr->GetDeletionTime();

        if (creationTime <= currentTime) {

            (*this).CreateNewNode(theParameterDatabaseReader, nodeId, nodeMobilityModelPtr);

        }
        else if ((creationTime < deletionTime) && (creationTime < INFINITE_TIME)) {

            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(new NodeEnterEvent(this, nodeId, nodeMobilityModelPtr)),
                creationTime);

        }//if//

        if ((creationTime < deletionTime) && (deletionTime < INFINITE_TIME)) {

            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(new NodeLeaveEvent(this, nodeId)),
                deletionTime);

        }//if//
    }//for//

    (*this).CheckTheNecessityOfMultiAgentSupport();

    (*this).ExecuteTimestepBasedEvent();

}//CompleteSimulatorConstruction//

void NetworkSimulator::CheckTheNecessityOfMultiAgentSupport()
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    vector<string> multiagentParameters;

    multiagentParameters.push_back("multiagent-profile-type");
    multiagentParameters.push_back("multiagent-behavior-type");

    bool needMultiAgentModule = false;

    for(size_t i = 0; i < multiagentParameters.size(); i++) {
        set<NodeIdType> nodeIds;

        theParameterDatabaseReader.MakeSetOfAllNodeIdsWithParameter(multiagentParameters[i], nodeIds);

        if (!nodeIds.empty()) {
            needMultiAgentModule = true;
            break;
        }//if//
    }//for//


    if (needMultiAgentModule) {
        if (!(*this).SupportMultiAgent()) {
            cerr << "Error: Found a MultiAgent Module parameter specification."
                 << "Use MultiAgent build option for Simulator. (build option: MULTIAGENT_MODULE=on)" << endl;
            exit(1);
        }//if//
    }//if//

}//CheckTheNecessityOfMultiAgentSupport//

void NetworkSimulator::SetupStatOutputFile()
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    if (theParameterDatabaseReader.ParameterExists("statistics-output-file")) {
        statsOutputFilename = theParameterDatabaseReader.ReadString("statistics-output-file");

        SetupFileControlledStats(
            theParameterDatabaseReader,
            theSimulationEnginePtr->GetRuntimeStatisticsSystem(),
            fileControlledStatViews,
            statsOutputFilename,
            noDataOutputIsEnabled);

    }//if//

}//SetupStatOutputFile//



void NetworkSimulator::AddPropagationCalculationTraceIfNecessary(
    const InterfaceIdType& channelId,
    const shared_ptr<SimplePropagationLossCalculationModel>& propagationCalculationModelPtr)
{
    const ParameterDatabaseReader& theParameterDatabaseReader = (*theParameterDatabaseReaderPtr);

    if (!theParameterDatabaseReader.ParameterExists("proptrace-filename", channelId)) {
        return;
    }

    string propagationModel =
        MakeLowerCaseString(theParameterDatabaseReader.ReadString("propagation-model", channelId));

    if (propagationModel == "trace") {
        return;
    }

    // set propagation trace output

    if (theParameterDatabaseReader.ParameterExists("proptrace-filename", channelId)) {

        const string traceOutputFileName =
            theParameterDatabaseReader.ReadString("proptrace-filename", channelId);

        ofstream outStream(traceOutputFileName.c_str(), std::ios::out);

        if (!outStream.good()) {
            cerr << "Could Not open propagation trace file: \"" << traceOutputFileName << "\"" << endl;
            exit(1);
        }//if//

        outStream << "TimeStepPropagationTrace2 " << ConvertTimeToStringSecs(timeStepEventSynchronizationStep) << endl;

        propagationTraceOutputInfos.push_back(
            PropagationTraceOutputInfo(
                channelId,
                propagationCalculationModelPtr,
                traceOutputFileName));

    }//if//

}//AddPropagationCalculationTraceIfNecessary//




static inline
double CalculateXySquaredDistance(const ObjectMobilityPosition& p1, const ObjectMobilityPosition& p2)
{
    return SquaredXYDistanceBetweenVertices(
        Vertex(p1.X_PositionMeters(), p1.Y_PositionMeters(), p1.HeightFromGroundMeters()),
        Vertex(p2.X_PositionMeters(), p2.Y_PositionMeters(), p2.HeightFromGroundMeters()));

}//CalculateXySquaredDistance//


struct PropagationResult {
    NodeIdType nodeId1;
    NodeIdType nodeId2;
    double lossValueDb12;
    double lossValueDb21;

    PropagationResult()
        :
        nodeId1(INVALID_NODEID),
        nodeId2(INVALID_NODEID),
        lossValueDb12(0),
        lossValueDb21(0)
    {}

    PropagationResult(
        const NodeIdType& initNodeId1,
        const NodeIdType& initNodeId2,
        const double initLossValueDb12,
        const double initLossValueDb21)
        :
        nodeId1(initNodeId1),
        nodeId2(initNodeId2),
        lossValueDb12(initLossValueDb12),
        lossValueDb21(initLossValueDb21)
    {}
};//PropagationResult//


static inline
bool IsSingleInterfaceNode(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId)
{
    const string antennaParameterName = "antenna-model";

    set<InterfaceOrInstanceIdType> setOfInstances;
    theParameterDatabaseReader.MakeSetOfAllInterfaceIdsForANode(nodeId, antennaParameterName, setOfInstances);

    if (setOfInstances.empty()) {

        return theParameterDatabaseReader.ParameterExists(antennaParameterName, nodeId);
    }

    return (setOfInstances.size() == 1);
}

static inline
void CheckAllNodeIsSingleInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const map<NodeIdType, shared_ptr<NetworkNode> >& nodes)
{
    typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator IterType;

    for(IterType iter = nodes.begin(); iter != nodes.end(); iter++) {

        if (!IsSingleInterfaceNode(theParameterDatabaseReader, (*iter).first)) {
            cerr << "Error: For propagation trace output all communication nodes must be single interface." << endl;
            exit(1);
        }//if//
    }//for//
}

void NetworkSimulator::OutputPropagationTrace(const TimeType& time)
{
    if (propagationTraceOutputInfos.empty()) {
        return;
    }

    if (propagationTraceOutputInfos.size() > 1) {
        cerr << "Error: Found propagation trace output option for multiple propagation models" << endl
             << "       Propagation trace output is available for single propagation calculation." << endl;
        exit(1);
    }

    assert(propagationTraceOutputInfos.size() == 1);


    // Assume all node is single interface node.

    CheckAllNodeIsSingleInterface(*theParameterDatabaseReaderPtr, nodes);


    // Calculate propagation trace for all node(;interface) paris

    for(size_t i = 0; i < propagationTraceOutputInfos.size(); i++) {
        const PropagationTraceOutputInfo& propagationTraceOutputInfo = propagationTraceOutputInfos[i];

        ofstream outStream(propagationTraceOutputInfo.outputFileName.c_str(), std::ios::binary | std::ios::out | std::ios::app);

        assert(outStream.good());
        outStream.write(reinterpret_cast<const char *>(&time), sizeof(time));

        SimplePropagationLossCalculationModel& calculationModel = *propagationTraceOutputInfo.propagationCalculationModelPtr;

        if (calculationModel.SupportMultipointCalculation()) {
            calculationModel.CacheMultipointPropagationLossDb();
        }//if//

        typedef map<NodeIdType, shared_ptr<NetworkNode> >::const_iterator NodeIter;

        const InterfaceIdType& channelId = propagationTraceOutputInfo.channelId;

        vector<PropagationResult> propagationResults;

        for(NodeIter nodeIter1 = nodes.begin(); nodeIter1 != nodes.end(); nodeIter1++) {
            const NetworkNode& node1 = *(*nodeIter1).second;

            for(NodeIter nodeIter2 = nodeIter1; nodeIter2 != nodes.end(); nodeIter2++) {
                const NetworkNode& node2 = *(*nodeIter2).second;

                // Use first interface (interfaceIndexx = 0) for propagation trace calculation.
                const unsigned int propagationCalculationInterfaceIndex = 0;
                const ObjectMobilityPosition pos1 = node1.GetAntennaLocation(propagationCalculationInterfaceIndex);
                const ObjectMobilityPosition pos2 = node2.GetAntennaLocation(propagationCalculationInterfaceIndex);
                const double distance = CalculateXySquaredDistance(pos1, pos2);

                propagationResults.push_back(
                    PropagationResult(
                        node1.GetNodeId(),
                        node2.GetNodeId(),
                        calculationModel.CalculatePropagationLossDb(pos1, pos2, distance),
                        calculationModel.CalculatePropagationLossDb(pos2, pos1, distance)));
            }//for//
        }//for//


        // Output bidirection propagation calculation results.

        const uint32_t numberResults = static_cast<uint32_t>(propagationResults.size());

        outStream.write(reinterpret_cast<const char *>(&numberResults), sizeof(numberResults));

        for(size_t i = 0; i < propagationResults.size(); i++) {
            const PropagationResult& propagationResult = propagationResults[i];

            outStream.write(reinterpret_cast<const char *>(&propagationResult.nodeId1), sizeof(propagationResult.nodeId1));
            outStream.write(reinterpret_cast<const char *>(&propagationResult.nodeId2), sizeof(propagationResult.nodeId2));
            outStream.write(reinterpret_cast<const char *>(&propagationResult.lossValueDb12), sizeof(propagationResult.lossValueDb12));
            outStream.write(reinterpret_cast<const char *>(&propagationResult.lossValueDb21), sizeof(propagationResult.lossValueDb21));
        }//if//
    }//for//

}//OutputPropagationTrace//


}//namespace//

