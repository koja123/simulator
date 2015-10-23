// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_prop.h"

#include <boost/thread.hpp>
#include "scensim_gis.h"
#include "itm.h"

namespace ScenSim {


class SimplePropagationLossCalculationModel::Implementation {
public:
    ~Implementation();

private:
    friend class SimplePropagationLossCalculationModel;

    Implementation(
        SimplePropagationLossCalculationModel* initInterfacePtr,
        const unsigned int initNumberThreads);

    struct ParallelCalculationData {
        ObjectMobilityPosition txAntennaPosition;
        const AntennaModel* txAntennaModelPtr;
        unsigned int rxNodeIndex;
        ObjectMobilityPosition rxAntennaPosition;
        shared_ptr<AntennaModel> rxAntennaModelPtr;
        double lossDb;
        TimeType propagationDelay;

        ParallelCalculationData() : txAntennaModelPtr(nullptr) {}
        void Clear() { txAntennaModelPtr = nullptr; rxAntennaModelPtr.reset(); }
    };

    void ParallelCalculationThreadRoutine(const unsigned int threadNumber);

    class ThreadFunctor {
    public:
        ThreadFunctor(
            SimplePropagationLossCalculationModel::Implementation* initPropModelPtr,
            const unsigned int initThreadNumber)
        :
        propModelPtr(initPropModelPtr), threadNumber(initThreadNumber) { }

        void operator()() { propModelPtr->ParallelCalculationThreadRoutine(threadNumber); }

    private:
        SimplePropagationLossCalculationModel::Implementation* propModelPtr;
        unsigned int threadNumber;
    };//ThreadFunctor//


    unsigned int numberThreads;

    vector<shared_ptr<boost::thread> > threadList;

    SimplePropagationLossCalculationModel* interfacePtr;
    boost::barrier theThreadBarrier;
    volatile bool shuttingDownThreads;
    volatile long int numberCalculationsLeft;

    unsigned int currentNumberOfCalculations;
    vector<ParallelCalculationData> calcDataset;

    void CalculateLossesDbAndPropDelaysInParallel();

};//SimplePropagationLossCalculationModel::Implementation//



SimplePropagationLossCalculationModel::Implementation::Implementation(
    SimplePropagationLossCalculationModel* initInterfacePtr,
    const unsigned int initNumberThreads)
    :
    interfacePtr(initInterfacePtr),
    numberThreads(initNumberThreads),
    theThreadBarrier(initNumberThreads+1),
    shuttingDownThreads(false),
    currentNumberOfCalculations(0),
    numberCalculationsLeft(0)
{
    for(unsigned int i = 0; (i < numberThreads); i++) {
        threadList.push_back(
            shared_ptr<boost::thread>(new boost::thread(ThreadFunctor(this, i))));
    }//for//
}


SimplePropagationLossCalculationModel::Implementation::~Implementation()
{
    (*this).shuttingDownThreads = true;
    theThreadBarrier.wait();

    for(size_t i = 0; (i < threadList.size()); i++) {
        threadList[i]->join();
    }//for//

}//~Implementation//


SimplePropagationLossCalculationModel::SimplePropagationLossCalculationModel(
    const double& initCarrierFrequencyMhz,
    const double& maximumPropagationDistanceMeters,
    const bool initPropagationDelayIsEnabled,
    const int numberDataParallelThreads)
    :
    carrierFrequencyMhz(initCarrierFrequencyMhz),
    propagationDelayIsEnabled(initPropagationDelayIsEnabled),
    maximumPropagationDistanceSquaredMeters(DBL_MAX)
{
    if (maximumPropagationDistanceMeters != DBL_MAX) {
        maximumPropagationDistanceSquaredMeters =
            (maximumPropagationDistanceMeters * maximumPropagationDistanceMeters);
    }
    else {
        maximumPropagationDistanceSquaredMeters = DBL_MAX;
    }//if//

    if (numberDataParallelThreads > 1) {
        implPtr =
            auto_ptr<Implementation>(
                new Implementation(this, numberDataParallelThreads));
    }//if//

}

SimplePropagationLossCalculationModel::~SimplePropagationLossCalculationModel() { }


void SimplePropagationLossCalculationModel::ClearParallelCalculationSet()
{
    // Performance Hack: Don't

    for(unsigned int i = 0; (i < implPtr->currentNumberOfCalculations); i++) {
        implPtr->calcDataset[i].Clear();
    }//for//

    implPtr->currentNumberOfCalculations = 0;
}

void SimplePropagationLossCalculationModel::AddToParallelCalculationSet(
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel* txAntennaModelPtr,
    const unsigned int& rxNodeIndex,
    const ObjectMobilityPosition& rxAntennaPosition,
    const shared_ptr<AntennaModel>& rxAntennaModelPtr)
{
    typedef Implementation::ParallelCalculationData ParallelCalculationData;

    if (implPtr->currentNumberOfCalculations == implPtr->calcDataset.size()) {
        implPtr->calcDataset.push_back(ParallelCalculationData());
    }//if//
    implPtr->currentNumberOfCalculations++;

    ParallelCalculationData& calcData =
        implPtr->calcDataset.at((implPtr->currentNumberOfCalculations)-1);

    calcData.txAntennaPosition = txAntennaPosition;
    calcData.txAntennaModelPtr = txAntennaModelPtr;
    calcData.rxNodeIndex = rxNodeIndex;
    calcData.rxAntennaPosition = rxAntennaPosition;
    calcData.rxAntennaModelPtr = rxAntennaModelPtr;
    calcData.lossDb = 0.0;
    calcData.propagationDelay = ZERO_TIME;

}//AddParallelCalculationToSet//



unsigned int SimplePropagationLossCalculationModel::GetNumberOfParallelCalculations() const
{
    return (implPtr->currentNumberOfCalculations);
}


double SimplePropagationLossCalculationModel::GetTotalLossDb(const unsigned int jobIndex) const
{
    assert(jobIndex < implPtr->currentNumberOfCalculations);
    return (implPtr->calcDataset.at(jobIndex).lossDb);
}


TimeType SimplePropagationLossCalculationModel::GetPropagationDelay(const unsigned int jobIndex) const
{
    assert(jobIndex < implPtr->currentNumberOfCalculations);
    return (implPtr->calcDataset.at(jobIndex).propagationDelay);
}

ObjectMobilityPosition& SimplePropagationLossCalculationModel::GetTxAntennaPosition(
    const unsigned int jobIndex) const
{
    assert(jobIndex < implPtr->currentNumberOfCalculations);
    return (implPtr->calcDataset.at(jobIndex).txAntennaPosition);
}


unsigned int SimplePropagationLossCalculationModel::GetRxNodeIndex(const unsigned int jobIndex) const
{
    assert(jobIndex < implPtr->currentNumberOfCalculations);
    return (implPtr->calcDataset.at(jobIndex).rxNodeIndex);
}

ObjectMobilityPosition& SimplePropagationLossCalculationModel::GetRxAntennaPosition(
    const unsigned int jobIndex) const
{
    assert(jobIndex < implPtr->currentNumberOfCalculations);
    return (implPtr->calcDataset.at(jobIndex).rxAntennaPosition);
}


inline
void SimplePropagationLossCalculationModel::Implementation::CalculateLossesDbAndPropDelaysInParallel()
{
    if (currentNumberOfCalculations == 0) {
        // Nothing to do.
        return;
    }//if//

    numberCalculationsLeft = static_cast<long int>(currentNumberOfCalculations);

    // Computational threads go!
    theThreadBarrier.wait();

    // Wait for them all to complete.
    theThreadBarrier.wait();

    assert(numberCalculationsLeft < 0);

}//CalculateLossesDbAndPropDelaysInParallel//



void SimplePropagationLossCalculationModel::CalculateLossesDbAndPropDelaysInParallel()
{
    implPtr->CalculateLossesDbAndPropDelaysInParallel();
}//CalculateLossesDbAndPropDelaysInParallel//



void SimplePropagationLossCalculationModel::Implementation::ParallelCalculationThreadRoutine(
    const unsigned int threadNumber)
{
    while (true) {

        theThreadBarrier.wait();

        if (shuttingDownThreads) {
            break;
        }//if//

        // Everyone starts computing

        while (true) {

            // Claim a parallel computation (index of array) to compute.

            long int calcIndex = AtomicDecrement(&(*this).numberCalculationsLeft);

            if (calcIndex < 0) {
                // No more parallel calculations
                break;
            }//if//

            ParallelCalculationData& calcData = (*this).calcDataset.at(calcIndex);

            //double* lossInverse = new double();
            //TimeType* propDelayInverse = new TimeType();

            interfacePtr->CalculateTotalLossDbAndPropDelay(
                calcData.txAntennaPosition,
                *calcData.txAntennaModelPtr,
                calcData.rxAntennaPosition,
                *calcData.rxAntennaModelPtr,
                calcData.lossDb,
                calcData.propagationDelay,
                threadNumber);

        }//while//

        // Wait for everyone to complete calculations.

        theThreadBarrier.wait();

    }//while//

}//ParallelCalculationThreadRoutine//

void SingleThreadPropagationLossTraceModel::ReadHeaderLine(ifstream& inStream, string& aLine)
{
    using std::streampos;
    using std::streamoff;

    const streamoff singleOffset = streamoff(1);

    aLine.clear();

    streampos readPos = 0;

    while (true) {
        inStream.seekg(readPos);

        const int ic = inStream.peek();
        assert(ic != EOF);
        const unsigned char c = static_cast<unsigned char>(ic);
        readPos += singleOffset;
        inStream.seekg(readPos);

        if (c == '\n') {
            const unsigned char c2 = static_cast<unsigned char>(inStream.peek());

            if (c2 == '\n') {
                readPos += singleOffset;
                inStream.seekg(readPos);
            }
            return;
        }
        else if (c == '\r') {
            const unsigned char c2 = static_cast<unsigned char>(inStream.peek());

            if (c2 == '\n') {
                readPos += singleOffset;
                inStream.seekg(readPos);
            }
            return;
        }
        else {
            aLine.push_back(c);
        }//if//
    }//for//
}//ReadHeaderLine//

SingleThreadPropagationLossTraceModel::SingleThreadPropagationLossTraceModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const double& initCarrierFrequencyMhz,
    const bool initPropagationDelayIsEnabled,
    const int numberDataParallelThreads,
    const InterfaceOrInstanceIdType& initInstanceId,
    const shared_ptr<SimplePropagationLossCalculationModel>& initDefaultkPropagationCalculationModelPtr)
    :
    SimplePropagationLossCalculationModel(
        initCarrierFrequencyMhz,
        DBL_MAX,
        initPropagationDelayIsEnabled,
        numberDataParallelThreads),
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    defaultkPropagationCalculationModelPtr(initDefaultkPropagationCalculationModelPtr),
    traceFileName(
        theParameterDatabaseReader.ReadString("proptrace-filename", initInstanceId)),
    propagationTraceIsDependsOnNodeId(true),
    versionNumber(0),
    lossValueChangeTimeStep(ZERO_TIME),
    lastLossValueUpdatedTime(ZERO_TIME)
{
    ifstream inStream(traceFileName.c_str());

    if (!inStream.good()) {
        cerr << "Could Not open propagation trace file: " << traceFileName << endl;
        exit(1);
    }

    string version;

    while (!inStream.eof() && version.empty()) {
        string aLine;

        (*this).ReadHeaderLine(inStream, aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }

        string lossValueChangeTimeStepString;
        bool success;

        std::istringstream lineStream(aLine);
        lineStream >> version >> lossValueChangeTimeStepString;

        ConvertStringToTime(lossValueChangeTimeStepString, lossValueChangeTimeStep, success);
        if (!success) {
            cerr << "Error in propagation trace file: invalid time string " << lossValueChangeTimeStepString
                 << " file " << traceFileName << endl;
            exit(1);
        }
        currentFilePos = 0;
        currentFilePos = inStream.tellg();
    }

    if (version == "TimeStepPropagationTrace") {
        versionNumber = 1;
        propagationTraceIsDependsOnNodeId = true;
    }
    else if (version == "TimeStepPropagationTrace2") {
        versionNumber = 2;
        propagationTraceIsDependsOnNodeId = true;
    }
    else if (version == "MeshPropagationTrace") {
        versionNumber = 1;
        propagationTraceIsDependsOnNodeId = false;
    }
    else {
        cerr << "Error in propagation trace file: invalid version string " << version
             << " file " << traceFileName << endl;
        exit(1);
    }
    if (lossValueChangeTimeStep <= ZERO_TIME) {
        cerr << "Error in propagation trace file: negative(or 0) time step " << lossValueChangeTimeStep
             << " file " << traceFileName << endl;
        exit(1);
    }

    (*this).ReadLossValuesFor(ZERO_TIME);
}

struct PropagationResult {
    NodeIdType nodeId1;
    NodeIdType nodeId2;
    double lossValueDb;

    PropagationResult()
        :
        nodeId1(INVALID_NODEID),
        nodeId2(INVALID_NODEID),
        lossValueDb(0)
    {}

    PropagationResult(
        const NodeIdType& initNodeId1,
        const NodeIdType& initNodeId2,
        const double initLossValueDb)
        :
        nodeId1(initNodeId1),
        nodeId2(initNodeId2),
        lossValueDb(initLossValueDb)
    {}
};

void SingleThreadPropagationLossTraceModel::ReadLossValuesFor(
    const TimeType& time) const
{
    if (propagationTraceIsDependsOnNodeId) {
        if (versionNumber == 2) {
            (*this).ReadLossValuesVer2(time);
        }
        else {
            (*this).ReadLossValuesVer1(time);
        }
    }
    else {
        (*this).ReadMeshLossValuesVer1(time);
    }
}

void SingleThreadPropagationLossTraceModel::ReadLossValuesVer1(const TimeType& time) const
{
    lossCache.clear();

    ifstream inFile(traceFileName.c_str());
    inFile.seekg(currentFilePos);

    while (!inFile.eof()) {

        currentFilePos = inFile.tellg();

        string aLine;
        getline(inFile, aLine);

        string timeString;
        TimeType lineTime;
        bool success;

        std::istringstream lineStream(aLine);
        lineStream >> timeString;

        ConvertStringToTime(timeString, lineTime, success);

        if (!success) {
            cerr << "Error in propagation trace file: invalid time string " << timeString
                 << " file " << traceFileName << endl;
            exit(1);
        }

        if (lineTime > time) {
            return;
        }

        if (lineTime <= time &&
            time <= lineTime + lossValueChangeTimeStep) {
            lastLossValueUpdatedTime = lineTime;
            break;
        }

        getline(inFile, aLine);
    }

    if (!inFile.eof()) {
        string aLine;
        getline(inFile, aLine);

        currentFilePos = inFile.tellg();

        std::istringstream lineStream(aLine);

        while (!lineStream.eof()) {

            NodeIdType nodeId1;
            NodeIdType nodeId2;
            double lossValueDb;

            lineStream >> nodeId1 >> nodeId2 >> lossValueDb;

            if (!lineStream.good()) {
                break;
            }

            lossCache[NodeKeyType(nodeId1, nodeId2)] = lossValueDb;
        }
    }
}

void SingleThreadPropagationLossTraceModel::ReadMeshLossValuesVer1(const TimeType& time) const
{
    if (time == ZERO_TIME) {

        ifstream inStream(traceFileName.c_str(), std::ios::binary);
        inStream.seekg(currentFilePos);

        inStream.read(reinterpret_cast<char *>(&areaInfo.traceAreaBaseX), sizeof(areaInfo.traceAreaBaseX));
        inStream.read(reinterpret_cast<char *>(&areaInfo.traceAreaBaseY), sizeof(areaInfo.traceAreaBaseY));
        inStream.read(reinterpret_cast<char *>(&areaInfo.traceAreaBaseZ), sizeof(areaInfo.traceAreaBaseZ));
        inStream.read(reinterpret_cast<char *>(&areaInfo.traceAreaHorizontalLength), sizeof(areaInfo.traceAreaHorizontalLength));
        inStream.read(reinterpret_cast<char *>(&areaInfo.traceAreaVerticalLength), sizeof(areaInfo.traceAreaVerticalLength));
        inStream.read(reinterpret_cast<char *>(&areaInfo.traceAreaHeight), sizeof(areaInfo.traceAreaHeight));
        inStream.read(reinterpret_cast<char *>(&areaInfo.horizontalMeshLength), sizeof(areaInfo.horizontalMeshLength));
        inStream.read(reinterpret_cast<char *>(&areaInfo.verticalMeshLength), sizeof(areaInfo.verticalMeshLength));
        inStream.read(reinterpret_cast<char *>(&areaInfo.meshHeight), sizeof(areaInfo.meshHeight));

        inStream.read(reinterpret_cast<char *>(&areaInfo.numberHorizontalMesh), sizeof(areaInfo.numberHorizontalMesh));
        inStream.read(reinterpret_cast<char *>(&areaInfo.numberVerticalMesh), sizeof(areaInfo.numberVerticalMesh));
        inStream.read(reinterpret_cast<char *>(&areaInfo.numberZMesh), sizeof(areaInfo.numberZMesh));

        const int numberMeshes =
            areaInfo.numberHorizontalMesh*areaInfo.numberVerticalMesh*areaInfo.numberZMesh;

        assert(numberMeshes < sqrt(static_cast<double>(areaInfo.lossValues.max_size())));

        areaInfo.lossValues.resize(numberMeshes*numberMeshes);

        for(unsigned int i = 0; i < areaInfo.lossValues.size(); i++) {
            inStream.read(reinterpret_cast<char *>(&areaInfo.lossValues[i]), sizeof(double));
        }
    }
}

void SingleThreadPropagationLossTraceModel::ReadLossValuesVer2(const TimeType& time) const
{
    lossCache.clear();

    ifstream inStream(traceFileName.c_str(), std::ios::binary);
    inStream.seekg(currentFilePos);

    while (!inStream.eof()) {
        TimeType traceTime;
        inStream.read(reinterpret_cast<char *>(&traceTime), sizeof(traceTime));

        if (!inStream.good()) {
            cerr << "Error in propagation trace file: invalid time file " << traceFileName << endl;
            exit(1);
        }

        if (traceTime > time) {
            return;
        }

        uint32_t numberPropagationResults;
        inStream.read(reinterpret_cast<char *>(&numberPropagationResults), sizeof(numberPropagationResults));

        for(unsigned int i = 0; i < numberPropagationResults; i++) {
            NodeIdType nodeId1;
            NodeIdType nodeId2;
            double lossValueDb12;
            double lossValueDb21;

            inStream.read(reinterpret_cast<char *>(&nodeId1), sizeof(nodeId1));
            inStream.read(reinterpret_cast<char *>(&nodeId2), sizeof(nodeId2));
            inStream.read(reinterpret_cast<char *>(&lossValueDb12), sizeof(lossValueDb12));
            inStream.read(reinterpret_cast<char *>(&lossValueDb21), sizeof(lossValueDb21));

            lossCache[NodeKeyType(nodeId1, nodeId2)] = lossValueDb12;
            lossCache[NodeKeyType(nodeId2, nodeId1)] = lossValueDb21;
        }

        currentFilePos = inStream.tellg();
    }
}

double SingleThreadPropagationLossTraceModel::CalculatePropagationLossDb(
    const ObjectMobilityPosition& txPosition,
    const ObjectMobilityPosition& rxPosition,
    const double& xyDistanceSquaredMeters) const
{
    if (propagationTraceIsDependsOnNodeId) {
        return (*this).GetNodeIdBasedPropagationLossDb(txPosition, rxPosition, xyDistanceSquaredMeters);
    }
    else {
        return (*this).GetMeshBasedPropagationLossDb(txPosition, rxPosition, xyDistanceSquaredMeters);
    }
}

double SingleThreadPropagationLossTraceModel::GetNodeIdBasedPropagationLossDb(
    const ObjectMobilityPosition& txPosition,
    const ObjectMobilityPosition& rxPosition,
    const double& xyDistanceSquaredMeters) const
{
    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();

    if (currentTime >= lastLossValueUpdatedTime + lossValueChangeTimeStep) {
        (*this).ReadLossValuesFor(currentTime);
    }

    const NodeKeyType key(txPosition.GetObjectId(), rxPosition.GetObjectId());

    typedef map<NodeKeyType, double>::const_iterator IterType;

    IterType iter = lossCache.find(key);

    if (iter != lossCache.end()) {
        return (*iter).second;
    }

    return defaultkPropagationCalculationModelPtr->CalculatePropagationLossDb(
        txPosition, rxPosition, xyDistanceSquaredMeters);
}

void SingleThreadPropagationLossTraceModel::CalculateMeshIndex(
    const ObjectMobilityPosition& mobilityPosition,
    int& meshIndexX,
    int& meshIndexY,
    int& meshIndexZ) const
{
    meshIndexX = static_cast<int>((mobilityPosition.X_PositionMeters() - areaInfo.traceAreaBaseX) / areaInfo.horizontalMeshLength);
    meshIndexY = static_cast<int>((mobilityPosition.Y_PositionMeters() - areaInfo.traceAreaBaseY) / areaInfo.verticalMeshLength);
    meshIndexZ = static_cast<int>((mobilityPosition.HeightFromGroundMeters() - areaInfo.traceAreaBaseZ) / areaInfo.meshHeight);
}

double SingleThreadPropagationLossTraceModel::GetMeshBasedPropagationLossDb(
    const ObjectMobilityPosition& txPosition,
    const ObjectMobilityPosition& rxPosition,
    const double& xyDistanceSquaredMeters) const
{
    int txIndexX;
    int txIndexY;
    int txIndexZ;
    int rxIndexX;
    int rxIndexY;
    int rxIndexZ;

    (*this).CalculateMeshIndex(txPosition, txIndexX, txIndexY, txIndexZ);
    (*this).CalculateMeshIndex(rxPosition, rxIndexX, rxIndexY, rxIndexZ);

    if (!(0 <= txIndexX && txIndexX < areaInfo.numberHorizontalMesh &&
          0 <= txIndexY && txIndexY < areaInfo.numberVerticalMesh &&
          0 <= txIndexZ && txIndexZ < areaInfo.numberZMesh &&
          0 <= rxIndexX && rxIndexX < areaInfo.numberHorizontalMesh &&
          0 <= rxIndexY && rxIndexY < areaInfo.numberVerticalMesh &&
          0 <= rxIndexZ && rxIndexZ < areaInfo.numberZMesh)) {
        return defaultkPropagationCalculationModelPtr->CalculatePropagationLossDb(
            txPosition, rxPosition, xyDistanceSquaredMeters);
    }

    const int rxMeshOffset =
        areaInfo.numberHorizontalMesh*areaInfo.numberVerticalMesh*areaInfo.numberZMesh;

    const int txMeshIndex =
        (areaInfo.numberHorizontalMesh*areaInfo.numberVerticalMesh)*txIndexZ +
        (areaInfo.numberVerticalMesh*txIndexX) +
        txIndexY;

    const int rxMeshIndex =
        (areaInfo.numberHorizontalMesh*areaInfo.numberVerticalMesh)*rxIndexZ +
        (areaInfo.numberVerticalMesh*rxIndexX) +
        rxIndexY;

    return areaInfo.lossValues.at(txMeshIndex*rxMeshOffset + rxMeshIndex);
}

//------------------------------------------------------------------------------


IndoorPropagationLossCalculationModel::IndoorPropagationLossCalculationModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const double& initCarrierFrequencyMhz,
    const double& initMaximumPropagationDistanceMeters,
    const bool initPropagationDelayIsEnabled,
    const int numberDataParallelThreads,
    const InterfaceOrInstanceIdType& initInstanceId)
    :
    SimplePropagationLossCalculationModel(
        initCarrierFrequencyMhz,
        initMaximumPropagationDistanceMeters,
        initPropagationDelayIsEnabled,
        numberDataParallelThreads),
    freespacePropagationCalculationModelPtr(
        new FreeSpacePropagationLossCalculationModel(
            initCarrierFrequencyMhz,
            initMaximumPropagationDistanceMeters,
            initPropagationDelayIsEnabled,
            numberDataParallelThreads)),
    theGisSubsystemPtr(initGisSubsystemPtr),
    indoorBreakpointMeters(
        theParameterDatabaseReader.ReadDouble("propindoor-indoor-breakpoint-meters", initInstanceId))
{
}

double IndoorPropagationLossCalculationModel::CalculatePropagationLossDb(
    const ObjectMobilityPosition& txPosition,
    const ObjectMobilityPosition& rxPosition,
    const double& xyDistanceSquaredMeters) const
{
    const double distanceMeters = sqrt(xyDistanceSquaredMeters);

    const double freeSpaceLossDb =
        freespacePropagationCalculationModelPtr->CalculatePropagationLossDb(
            txPosition, rxPosition, xyDistanceSquaredMeters);

    const double attenuationDb =
        (*this).CalculateLinearAttenuation(distanceMeters);

    double totalLossDb = std::max(0.0, freeSpaceLossDb + attenuationDb);

    typedef map<string, shared_ptr<BuildingLayer> >::const_iterator IterType;

    const Vertex txVertex(
        txPosition.X_PositionMeters(), txPosition.Y_PositionMeters(), txPosition.HeightFromGroundMeters());
    const Vertex rxVertex(
        rxPosition.X_PositionMeters(), rxPosition.Y_PositionMeters(), rxPosition.HeightFromGroundMeters());

    const BuildingLayer& buildingLayer = *theGisSubsystemPtr->GetBuildingLayerPtr();

    totalLossDb += buildingLayer.CalculateTotalWallAndFloorLossDb(txVertex, rxVertex);

    return totalLossDb;
}

double IndoorPropagationLossCalculationModel::CalculateLinearAttenuation(const double distanceMeters) const
{
    return 0.2 * (distanceMeters - indoorBreakpointMeters);
}

void IndoorPropagationLossCalculationModel::CalculatePropagationPathInformation(
    const PropagationInformationType& informationType,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntennaModel,
    const ObjectMobilityPosition& rxAntennaPosition,
    const AntennaModel& rxAntennaModel,
    PropagationStatisticsType& propagationStatistics) const
{
    const double deltaX = txAntennaPosition.X_PositionMeters() - rxAntennaPosition.X_PositionMeters();
    const double deltaY = txAntennaPosition.Y_PositionMeters() - rxAntennaPosition.Y_PositionMeters();

    const double lossDb = CalculatePropagationLossDb(
        txAntennaPosition,
        rxAntennaPosition,
        deltaX*deltaX + deltaY*deltaY);

    const Vertex txVertex(
        txAntennaPosition.X_PositionMeters(),
        txAntennaPosition.Y_PositionMeters(),
        txAntennaPosition.HeightFromGroundMeters());
    const Vertex rxVertex(
        rxAntennaPosition.X_PositionMeters(),
        rxAntennaPosition.Y_PositionMeters(),
        rxAntennaPosition.HeightFromGroundMeters());

    propagationStatistics.paths.resize(1);

    PropagationVertexType txPathVertex;
    PropagationVertexType rxPathVertex;

    txPathVertex.x = txVertex.positionX;
    txPathVertex.y = txVertex.positionY;
    txPathVertex.z = txVertex.positionZ;
    txPathVertex.vertexType = "Tx";

    rxPathVertex.x = rxVertex.positionX;
    rxPathVertex.y = rxVertex.positionY;
    rxPathVertex.z = rxVertex.positionZ;
    rxPathVertex.vertexType = "Rx";

    PropagationPathType& propagationPath = propagationStatistics.paths.front();

    propagationPath.pathVertices.push_back(txPathVertex);

    propagationPath.lossValueDb =  lossDb - CalculateTotalAntennaGainDbi(
        txAntennaPosition,
        txAntennaModel,
        rxAntennaPosition,
        rxAntennaModel);

    propagationPath.pathVertices.push_back(rxPathVertex);
    propagationStatistics.totalLossValueDb = propagationPath.lossValueDb;
}


//------------------------------------------------------------------------------


TgAxIndoorPropLossCalculationModel::TgAxIndoorPropLossCalculationModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const double& initCarrierFrequencyMhz,
    const double& initMaximumPropagationDistanceMeters,
    const bool initPropagationDelayIsEnabled,
    const int numberDataParallelThreads,
    const InterfaceOrInstanceIdType& initInstanceId)
    :
    SimplePropagationLossCalculationModel(
        initCarrierFrequencyMhz,
        initMaximumPropagationDistanceMeters,
        initPropagationDelayIsEnabled,
        numberDataParallelThreads),
    theGisSubsystemPtr(initGisSubsystemPtr)
{
    breakpointMeters =
        theParameterDatabaseReader.ReadDouble(
            "prop-tgax-indoor-freespace-breakpoint-meters", initInstanceId);

    log10BreakpointMeters = log10(breakpointMeters);

    floorAttenuationDb =
        theParameterDatabaseReader.ReadDouble(
            "prop-tgax-indoor-floor-attenuation-db", initInstanceId);

    wallAttenuationDb =
        theParameterDatabaseReader.ReadDouble(
            "prop-tgax-indoor-wall-attenuation-db", initInstanceId);

    constantFactor = 40.05 + 20.0 * log10(carrierFrequencyMhz/ 2400.0);

}//TgAxIndoorPropLossCalculationModel()//


double TgAxIndoorPropLossCalculationModel::CalculatePropagationLossDb(
    const ObjectMobilityPosition& txPosition,
    const ObjectMobilityPosition& rxPosition,
    const double& xyDistanceSquaredMeters) const
{
    using std::max;

    const BuildingLayer& buildingLayer = *theGisSubsystemPtr->GetBuildingLayerPtr();

    const double deltaZ = (txPosition.HeightFromGroundMeters() - rxPosition.HeightFromGroundMeters());

    const double totalDistanceMeters = max(1.0, sqrt(xyDistanceSquaredMeters + (deltaZ * deltaZ)));

    const Vertex txVertex(
        txPosition.X_PositionMeters(), txPosition.Y_PositionMeters(), txPosition.HeightFromGroundMeters());
    const Vertex rxVertex(
        rxPosition.X_PositionMeters(), rxPosition.Y_PositionMeters(), rxPosition.HeightFromGroundMeters());

    unsigned int numberFloors;
    unsigned int numberWalls;

    buildingLayer.CalculateNumberOfFloorsAndWallsTraversed(
        txVertex, rxVertex, set<NodeIdType>(/*none*/), numberFloors, numberWalls);

    double pathlossDb = constantFactor;
    if (totalDistanceMeters <= breakpointMeters) {
        pathlossDb += (20.0 * log10(totalDistanceMeters));
    }
    else {
        pathlossDb +=
            ((20.0 * log10BreakpointMeters) +
             (35.0 * (log10(totalDistanceMeters) - log10BreakpointMeters)));
    }//if//


    if (numberFloors <= 1) {
        pathlossDb += floorAttenuationDb * numberFloors;
    }
    else {
        const double exponent =
            (((numberFloors + 2.0) / (numberFloors + 1.0)) - 0.46);

        pathlossDb += (floorAttenuationDb * pow(static_cast<double>(numberFloors), exponent));

    }//if//

    pathlossDb += wallAttenuationDb * numberWalls;

    return (pathlossDb);

}//CalculatePropagationLossDb//



void TgAxIndoorPropLossCalculationModel::CalculatePropagationPathInformation(
    const PropagationInformationType& informationType,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntennaModel,
    const ObjectMobilityPosition& rxAntennaPosition,
    const AntennaModel& rxAntennaModel,
    PropagationStatisticsType& propagationStatistics) const
{
    const double deltaX = txAntennaPosition.X_PositionMeters() - rxAntennaPosition.X_PositionMeters();
    const double deltaY = txAntennaPosition.Y_PositionMeters() - rxAntennaPosition.Y_PositionMeters();

    const double lossDb =
        CalculatePropagationLossDb(
            txAntennaPosition,
            rxAntennaPosition,
            deltaX*deltaX + deltaY*deltaY);

    const Vertex txVertex(
        txAntennaPosition.X_PositionMeters(),
        txAntennaPosition.Y_PositionMeters(),
        txAntennaPosition.HeightFromGroundMeters());
    const Vertex rxVertex(
        rxAntennaPosition.X_PositionMeters(),
        rxAntennaPosition.Y_PositionMeters(),
        rxAntennaPosition.HeightFromGroundMeters());

    propagationStatistics.paths.resize(1);

    PropagationVertexType txPathVertex;
    PropagationVertexType rxPathVertex;

    txPathVertex.x = txVertex.positionX;
    txPathVertex.y = txVertex.positionY;
    txPathVertex.z = txVertex.positionZ;
    txPathVertex.vertexType = "Tx";

    rxPathVertex.x = rxVertex.positionX;
    rxPathVertex.y = rxVertex.positionY;
    rxPathVertex.z = rxVertex.positionZ;
    rxPathVertex.vertexType = "Rx";

    PropagationPathType& propagationPath = propagationStatistics.paths.front();

    propagationPath.pathVertices.push_back(txPathVertex);

    propagationPath.lossValueDb =  lossDb - CalculateTotalAntennaGainDbi(
        txAntennaPosition,
        txAntennaModel,
        rxAntennaPosition,
        rxAntennaModel);

    propagationPath.pathVertices.push_back(rxPathVertex);
    propagationStatistics.totalLossValueDb = propagationPath.lossValueDb;

}//CalculatePropagationPathInformation//




//------------------------------------------------------------------------------



double WallCountPropagationLossCalculationModel::CalculatePropagationLossDb(
    const ObjectMobilityPosition& txPosition,
    const ObjectMobilityPosition& rxPosition,
    const double& xyDistanceSquaredMeters) const
{
    const double baselineLossDb =
        baselinePropCalculationModelPtr->CalculatePropagationLossDb(
            txPosition, rxPosition, xyDistanceSquaredMeters);

    const Vertex txVertex(
        txPosition.X_PositionMeters(), txPosition.Y_PositionMeters(), txPosition.HeightFromGroundMeters());
    const Vertex rxVertex(
        rxPosition.X_PositionMeters(), rxPosition.Y_PositionMeters(), rxPosition.HeightFromGroundMeters());

    const BuildingLayer& buildingLayer = *theGisSubsystemPtr->GetBuildingLayerPtr();
    const double totalLossDb =
        baselineLossDb +
        buildingLayer.CalculateNumberOfWallRoofFloorInteractions(txVertex, rxVertex) * penetrationLossDb;

    return totalLossDb;

}//CalculatePropagationLossDb

ItmPropagationLossCalculationModel::ItmPropagationLossCalculationModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const double& initCarrierFrequencyMhz,
        const shared_ptr<const GroundLayer>& initGroundLayerPtr,
        const double& initMaximumPropagationDistanceMeters,
        const bool initPropagationDelayIsEnabled,
        const int initNumberDataParallelThreads,
        const InterfaceOrInstanceIdType& initInstanceId)
    :
    SimplePropagationLossCalculationModel(
        initCarrierFrequencyMhz,
        initMaximumPropagationDistanceMeters,
        initPropagationDelayIsEnabled,
        initNumberDataParallelThreads),
    groundLayerPtr(initGroundLayerPtr),
    maxCalculationPointDivisionLength(1.0),
    earthDielectricConstant(15.0),
    earthConductivitySiemensPerMeters(0.005),
    atmosphericBendingConstant(350.0),
    radioClimate(RADIO_CLIMATE_MARITIME_TEMPERATE_OVER_LAND),
    polarization(POLARIZATION_HORIZONTAL),
    elevationArrayPerThread(initNumberDataParallelThreads),
    fractionOfTime(0.5),
    fractionOfSituations(0.5),
    enableVerticalDiffractionPathCalculation(false),
    addTreeHeightToElevation(false)
{
    if (theParameterDatabaseReader.ParameterExists(
            "propitm-calculation-point-division-length", initInstanceId)) {
        maxCalculationPointDivisionLength =
            theParameterDatabaseReader.ReadDouble("propitm-calculation-point-division-length", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists(
            "propitm-earth-dielectric-constant", initInstanceId)) {
        earthDielectricConstant =
            theParameterDatabaseReader.ReadDouble("propitm-earth-dielectric-constant", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists(
            "propitm-earth-conductivity", initInstanceId)) {
        earthConductivitySiemensPerMeters =
            theParameterDatabaseReader.ReadDouble("propitm-earth-conductivity", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists(
            "propitm-atmospheric-bending-constant", initInstanceId)) {
        atmosphericBendingConstant =
            theParameterDatabaseReader.ReadDouble("propitm-atmospheric-bending-constant", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists(
            "propitm-fraction-of-time", initInstanceId)) {
        fractionOfTime =
            theParameterDatabaseReader.ReadDouble("propitm-fraction-of-time", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists(
            "propitm-fraction-of-situations", initInstanceId)) {
        fractionOfSituations =
            theParameterDatabaseReader.ReadDouble("propitm-fraction-of-situations", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists(
            "propitm-radio-climate", initInstanceId)) {

        const string radioClimateString =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("propitm-radio-climate", initInstanceId));

        if (radioClimateString == "equatorial") {
            radioClimate = RADIO_CLIMATE_EQUATORIAL;
        }
        else if (radioClimateString == "continental-subtropical") {
            radioClimate = RADIO_CLIMATE_CONTINENTAL_SUBTROPICAL;
        }
        else if (radioClimateString == "maritime-tropical") {
            radioClimate = RADIO_CLIMATE_MARITIME_TROPICAL;
        }
        else if (radioClimateString == "desert") {
            radioClimate = RADIO_CLIMATE_DESERT;
        }
        else if (radioClimateString == "continental-temperate") {
            radioClimate = RADIO_CLIMATE_CONTINENTAL_TEMPERATE;
        }
        else if (radioClimateString == "maritime-temperate-over-land") {
            radioClimate = RADIO_CLIMATE_MARITIME_TEMPERATE_OVER_LAND;
        }
        else if (radioClimateString == "maritime-temperate-over-seea") {
            radioClimate = RADIO_CLIMATE_MARITIME_TEMPERATE_OVER_SEA;
        }
        else {
            cerr << "Error: ITM, invalid radio climate option: " << radioClimateString << endl;
            exit(1);
        }
    }

    if (theParameterDatabaseReader.ParameterExists(
            "propitm-polarization", initInstanceId)) {

        const string polarizationString =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("propitm-polarization", initInstanceId));

        if (polarizationString == "horizontal") {
            polarization = POLARIZATION_HORIZONTAL;
        }
        else if (polarizationString == "vertical") {
            polarization = POLARIZATION_VERTICAL;
        }
        else {
            cerr << "Error: ITM, invalid polarization option: " << polarizationString << endl;
            exit(1);
        }
    }

    if (theParameterDatabaseReader.ParameterExists(
            "propitm-polarization", initInstanceId)) {

        const string polarizationString =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("propitm-polarization", initInstanceId));

        if (polarizationString == "horizontal") {
            polarization = POLARIZATION_HORIZONTAL;
        }
        else if (polarizationString == "vertical") {
            polarization = POLARIZATION_VERTICAL;
        }
        else {
            cerr << "Error: ITM, invalid polarization option: " << polarizationString << endl;
            exit(1);
        }
    }

    if (theParameterDatabaseReader.ParameterExists(
            "propitm-enable-vertical-diffraction-path-calculation", initInstanceId)) {

        enableVerticalDiffractionPathCalculation =
            theParameterDatabaseReader.ReadBool(
                "propitm-enable-vertical-diffraction-path-calculation", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists(
            "propitm-enable-foliage-loss", initInstanceId)) {

        addTreeHeightToElevation =
            theParameterDatabaseReader.ReadBool(
                "propitm-enable-foliage-loss", initInstanceId);
    }
}

double ItmPropagationLossCalculationModel::CalculatePropagationLossDb(
    const ObjectMobilityPosition& txAntennaPosition,
    const ObjectMobilityPosition& rxAntennaPosition,
    const double& xyDistanceSquaredMeters) const
{
    if (xyDistanceSquaredMeters <= 0.) {
        return 0.;
    }

    const Vertex txPos(txAntennaPosition.X_PositionMeters(), txAntennaPosition.Y_PositionMeters(), txAntennaPosition.HeightFromGroundMeters());
    const Vertex rxPos(rxAntennaPosition.X_PositionMeters(), rxAntennaPosition.Y_PositionMeters(), rxAntennaPosition.HeightFromGroundMeters());

    const double txGroundMeters = groundLayerPtr->GetElevationMetersAt(txPos);
    const double rxGroundMeters = groundLayerPtr->GetElevationMetersAt(rxPos);

    const double txAntennaHeightFromGround = txPos.z - txGroundMeters;
    const double rxAntennaHeightFromGround = rxPos.z - rxGroundMeters;

    if (txAntennaHeightFromGround <= 0) {
        cerr << "Error in ITM propagation calculation: Tx antenna height is under the ground elevation." << endl
             << "  Set mobility-need-to-add-ground-height = true for Tx, or Set Tx antenna height is over the ground elevation." << endl;
        exit(1);
    }

    if (rxAntennaHeightFromGround <= 0) {
        //cerr << "Warning in ITM propagation calculation: Rx antenna height is under the ground elevation." << endl
        //     << "  Set mobility-need-to-add-ground-height = true for Rx, or Set Rx antenna height is over the ground elevation." << endl;

        return EXTREMELY_HIGH_LOSS_DB;
    }

    const Vertex normalVector = (rxPos - txPos).XYPoint().Normalized();
    const double distance = txPos.DistanceTo(rxPos);
    const int numberDivisions = static_cast<int>(ceil(distance / maxCalculationPointDivisionLength));
    const double calculationPointDivisionLength = distance / numberDivisions;

    ElevationArray& elevationArray = elevationArrayPerThread.front();
    elevationArray.PrepareEnoughArraySize(numberDivisions + 1);

    elevationArray.elevations[0] = static_cast<double>(std::max<int>(numberDivisions, 0));
    elevationArray.elevations[1] = calculationPointDivisionLength;

    vector<Vertex> points;

    groundLayerPtr->GetSeriallyCompleteElevationPoints(
        txPos,
        rxPos,
        numberDivisions,
        false/*addBuildingHeightToElevation*/,
        addTreeHeightToElevation,
        points);

    for(int i = 0; i <= numberDivisions; i++) {
        elevationArray.elevations[i+2] = points[i].z;
    }

    const double frequencyMhz = (*this).GetCarrierFrequencyMhz();

    double resultLossValue;
    int errorNumber;

    ITM::point_to_point(
        elevationArray.elevations.get(),
        txAntennaHeightFromGround,
        rxAntennaHeightFromGround,
        earthDielectricConstant,
        earthConductivitySiemensPerMeters,
        atmosphericBendingConstant,
        frequencyMhz,
        radioClimate,
        polarization,
        fractionOfSituations,
        fractionOfTime,
        resultLossValue,
        errorNumber);

    if (enableVerticalDiffractionPathCalculation) {

        double diffractionPathLossValue;

        groundLayerPtr->GetSeriallyCompleteElevationPoints(
            txPos,
            rxPos,
            numberDivisions,
            true/*addBuildingHeightToElevation*/,
            addTreeHeightToElevation,
            points);

        for(int i = 0; i <= numberDivisions; i++) {
            elevationArray.elevations[i+2] = points[i].z;
        }

        ITM::point_to_point(
            elevationArray.elevations.get(),
            txAntennaHeightFromGround,
            rxAntennaHeightFromGround,
            earthDielectricConstant,
            earthConductivitySiemensPerMeters,
            atmosphericBendingConstant,
            frequencyMhz,
            radioClimate,
            polarization,
            fractionOfSituations,
            fractionOfTime,
            diffractionPathLossValue,
            errorNumber);

        resultLossValue = std::min(resultLossValue, diffractionPathLossValue);
    }

    return std::max<double>(resultLossValue, 0.0);
}

double WeissbergerPropagationLossCalculationModel::CalculatePropagationLossDb(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        const double& xyDistanceSquaredMeters) const
{
    if (xyDistanceSquaredMeters <= 0.) {
        return 0.;
    }

    Vertex txPos(txAntennaPosition.X_PositionMeters(), txAntennaPosition.Y_PositionMeters(), txAntennaPosition.HeightFromGroundMeters());
    Vertex rxPos(rxAntennaPosition.X_PositionMeters(), rxAntennaPosition.Y_PositionMeters(), rxAntennaPosition.HeightFromGroundMeters());

    if (!txAntennaPosition.TheHeightContainsGroundHeightMeters()) {
        txPos.z += groundLayerPtr->GetElevationMetersAt(txPos);
    }
    if (!rxAntennaPosition.TheHeightContainsGroundHeightMeters()) {
        rxPos.z += groundLayerPtr->GetElevationMetersAt(rxPos);
    }

    const Vertex normalVector = (rxPos - txPos).XYPoint().Normalized();
    const double distance = txPos.DistanceTo(rxPos);
    const int numberDivisions = static_cast<int>(ceil(distance / maxCalculationPointDivisionLength));
    const double calculationPointDivisionLength = distance / numberDivisions;

    double lossDb = 0;
    int numberTreeCounts = 0;

    for(int i = 0; i <= numberDivisions; i++) {
        const Vertex posN = txPos + normalVector*(calculationPointDivisionLength*i);

        if (groundLayerPtr->HasTree(posN)) {
            numberTreeCounts++;
        }
    }
    if (groundLayerPtr->HasTree(rxPos)) {
        numberTreeCounts++;
    }

    const double treeDistance = numberTreeCounts*calculationPointDivisionLength;
    const double frequencyGhz = (*this).GetCarrierFrequencyMhz() / 1000.;

    if (0 < treeDistance && treeDistance <= 14) {
        lossDb = 0.45*pow(frequencyGhz, 0.284)*treeDistance;
    }
    else {
        lossDb = 1.33*pow(frequencyGhz, 0.284)*pow(treeDistance, 0.588);
    }

    return lossDb;
}


//-----------------------------------------------------------------------------


void AntennaPatternDatabase::LoadAntennaFile(
    const string& antennaFileName,
    const bool useLegacyFileFormatMode,
    const unsigned int two2dTo3dInterpolationAlgorithmNumber,
    const string& antennaPatternDebugDumpFileName)
{
    if (loadedFileNames.find(antennaFileName) != loadedFileNames.end()) {
        // Already loaded.
        return;

    }//if//

    loadedFileNames.insert(antennaFileName);

    ifstream customAntennaFile(antennaFileName.c_str());

    if (!customAntennaFile.good()) {
        cerr << "Error: Could not open custom antenna file: " << antennaFileName << endl;
        exit(1);
    }//if//

    ofstream debugDumpFile;

    if (antennaPatternDebugDumpFileName != "") {
        debugDumpFile.open(antennaPatternDebugDumpFileName.c_str());
    }//if//

    shared_ptr<AntennaPattern> antennaPatternPtr;

    enum GainDataType {
        HORIZONTAL, VERTICAL, THETA_AND_PHI
    };

    //initialize
    GainDataType dataType = THETA_AND_PHI;

    string antennaModelName = "";
    double maxGainDbi = DBL_MAX;

    while(!customAntennaFile.eof()) {
        string aLine;
        getline(customAntennaFile, aLine);

        if (customAntennaFile.eof()) {
            break;
        }
        else if (customAntennaFile.fail()) {
            cerr << "Error: Error reading custom antenna file: " << antennaFileName << endl;
            exit(1);
        }//if//

        if (!IsAConfigFileCommentLine(aLine)) {

            DeleteTrailingSpaces(aLine);

            istringstream lineStream(aLine);

            string firstValue;
            int firstValueInt;
            bool intValueSuccess;

            lineStream >> firstValue;

            ConvertStringToInt(firstValue, firstValueInt, intValueSuccess);

            if (firstValue == "NAME") {

                //add antenna pattern to database
                if (antennaPatternPtr != nullptr) {

                    assert(antennaModelName != "");

                    antennaPatternPtr->FinalizeShape();

                    if (antennaPatternDebugDumpFileName != "") {
                        (*this).DumpAntennaPattern(antennaModelName, *antennaPatternPtr, debugDumpFile);
                    }//if//

                    (*this).AddAntennaPattern(antennaModelName, antennaPatternPtr);

                    //reset
                    dataType = THETA_AND_PHI;
                    antennaModelName = "";
                    maxGainDbi = DBL_MAX;
                    antennaPatternPtr.reset();

                }//if//

                lineStream >> antennaModelName; //Names including space are invalid.
                ConvertStringToLowerCase(antennaModelName);

                if ((lineStream.fail()) || (!lineStream.eof())) {
                    cerr << "Error: Bad custom antenna file line: " << aLine << endl;
                    exit(1);
                }//if//
            }
            else if (firstValue == "GAIN"){

                lineStream >> maxGainDbi;

                string mustBeDbiString;
                lineStream >> mustBeDbiString;

                if ((lineStream.fail()) || (mustBeDbiString != "dBi") || (!lineStream.eof())) {
                    cerr << "Error: Bad custom antenna file line: " << aLine << endl;
                    exit(1);
                }//if//

                if (antennaPatternPtr != nullptr) {
                    cerr << "Error: NAME line should be placed before this line: " << aLine << endl;
                    exit(1);
                }//if//

            }
            else if ((firstValue == "HORIZONTAL") || (firstValue == "VERTICAL")) {

                //two and half pattern
                if (firstValue == "HORIZONTAL") {
                    dataType = HORIZONTAL;
                }
                else if (firstValue == "VERTICAL") {
                    dataType = VERTICAL;
                }//if//

                string mustBe360String;
                lineStream >> mustBe360String;

                if ((lineStream.fail()) || (mustBe360String != "360") ||(!lineStream.eof())) {
                    cerr << "Error: Bad custom antenna file line: " << aLine << endl;
                    exit(1);
                }//if//

            }
            else if ((firstValue == "BEAM_WIDTH") || (firstValue == "FREQUENCY") ||
                     (firstValue == "GAIN_UNIT") || (firstValue == "TILT") ||
                     (firstValue == "POLARIZATION") || (firstValue == "MAKE") ||
                     (firstValue == "H_WIDTH") || (firstValue == "V_WIDTH") ||
                     (firstValue == "FRONT_TO_BACK")) {

                // Ignore Line.
            }
            else if ((intValueSuccess) && (dataType == HORIZONTAL)) {

                //two and half pattern

                int angleDegrees = firstValueInt;

                if (!useLegacyFileFormatMode) {
                    if ((angleDegrees < -179) || (angleDegrees > 180)) {
                        cerr << "Error In Antenna Model File: Bad angle for gain: " << firstValue << "." << endl;
                        exit(1);
                    }//if//
                }
                else {
                    if ((angleDegrees < 0) || (angleDegrees >= 360)) {
                        cerr << "Error In Antenna Model File: Bad angle for gain: " << firstValue << "." << endl;
                        exit(1);
                    }//if//
                }//if//

                double gainDbi;
                lineStream >> gainDbi;

                if ((lineStream.fail()) || (!lineStream.eof())) {
                    cerr << "Error: Bad custom antenna file line: " << aLine << endl;
                    exit(1);
                }//if//

                if (useLegacyFileFormatMode) {
                    if (angleDegrees > 180) {
                        angleDegrees -= 360;
                    }//if//

                    if (gainDbi < 0.0) {
                        cerr << "Error In Antenna Model File: Legacy format has a negative gain value: " << gainDbi << endl;
                        cerr << "   (In the legacy file format the negative sign is implicit)." << endl;
                        exit(1);
                    }//if//

                    if (maxGainDbi == DBL_MAX) {
                        cerr << "Error In Antenna Model File: Legacy file format without maximum gain defined." << endl;
                        exit(1);
                    }//if//

                    gainDbi = maxGainDbi - gainDbi;
                }//if//

                if (antennaPatternPtr == nullptr) {
                    antennaPatternPtr.reset(new AntennaPattern(two2dTo3dInterpolationAlgorithmNumber));
                }//if//

                antennaPatternPtr->AddGainData(angleDegrees, 0, gainDbi);
            }
            else if ((intValueSuccess) && (dataType == VERTICAL)) {

                int angleDegrees = firstValueInt;

                if (!useLegacyFileFormatMode) {
                    if ((angleDegrees < -179) || (angleDegrees > 180)) {
                        cerr << "Error In Antenna Model File: Bad angle for gain: " << firstValue << "." << endl;
                        exit(1);
                    }//if//
                }
                else {
                    if ((angleDegrees < 0) || (angleDegrees >= 360)) {
                        cerr << "Error In Antenna Model File: Bad angle for gain: " << firstValue << "." << endl;
                        exit(1);
                    }//if//
                }//if//

                double gainDbi;
                lineStream >> gainDbi;

                if ((lineStream.fail()) || (!lineStream.eof())) {
                    cerr << "Error: Bad custom antenna file line: " << aLine << endl;
                    exit(1);
                }//if//

                if (useLegacyFileFormatMode) {
                    if (angleDegrees > 180) {
                        angleDegrees -= 360;
                    }//if//

                    if (gainDbi < 0.0) {
                        cerr << "Error In Antenna Model File: Legacy format has a negative gain value: " << gainDbi << endl;
                        cerr << "   (In the legacy file format the negative sign is implicit)." << endl;
                    exit(1);
                    }//if//
                    if (maxGainDbi == DBL_MAX) {
                        cerr << "Error In Antenna Model File: Legacy file format without maximum gain defined." << endl;
                        exit(1);
                    }//if//

                    gainDbi = maxGainDbi - gainDbi;
                }//if//

                if (antennaPatternPtr == nullptr) {
                    antennaPatternPtr.reset(new AntennaPattern(two2dTo3dInterpolationAlgorithmNumber));
                }//if//

                if (angleDegrees < -90) {
                    antennaPatternPtr->AddGainData(180, -(180 + angleDegrees), gainDbi);
                }
                else if (angleDegrees > 90) {
                    antennaPatternPtr->AddGainData(180, (180 - angleDegrees), gainDbi);
                }
                else {
                    antennaPatternPtr->AddGainData(0, angleDegrees, gainDbi);
                }//if//
            }
            else if (intValueSuccess && (dataType == THETA_AND_PHI)) {

                //3D pattern

                const int verticalDegree= firstValueInt;

                int horizontalDegree;
                lineStream >> horizontalDegree;

                if ((verticalDegree < -90) || (verticalDegree > 90) ||
                    (horizontalDegree < -179) || (horizontalDegree > 180)) {
                    cerr << "Error In Antenna Model File: Bad angle for gain: " << aLine << endl;
                    exit(1);
                }//if//

                double gain;
                lineStream >> gain;

                if ((lineStream.fail()) || (!lineStream.eof())) {
                    cerr << "Error: Bad custom antenna file line: " << aLine << endl;
                    exit(1);
                }//if//

                if (antennaPatternPtr == nullptr) {
                    antennaPatternPtr.reset(new AntennaPattern(two2dTo3dInterpolationAlgorithmNumber));
                }//if//

                antennaPatternPtr->AddGainData(horizontalDegree, verticalDegree, gain);
            }
            else {
                cerr << "Error: Bad custom antenna file line: " << aLine << endl;
                exit(1);

            }//if//

        }//if//

    }//while/

    //add antenna pattern to database
    if (antennaPatternPtr != nullptr) {

        assert(antennaModelName != "");

        antennaPatternPtr->FinalizeShape();

        if (antennaPatternDebugDumpFileName != "") {
            (*this).DumpAntennaPattern(antennaModelName, *antennaPatternPtr, debugDumpFile);
        }//if//

        (*this).AddAntennaPattern(antennaModelName, antennaPatternPtr);

    }//if//

}//LoadAntennaFile//


}//namespace//
