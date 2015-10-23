// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_FADING_H
#define SCENSIM_FADING_H

#include <cmath>
#include <assert.h>
#include <memory>
#include <map>
#include <complex>

#include "scensim_support.h"
#include "scensim_engine.h"
#include "randomnumbergen.h"
#include "scensim_parmio.h"


namespace ScenSim {

using std::make_pair;
using std::shared_ptr;
using std::map;
using std::complex;
using std::polar;
using std::norm;

static const unsigned int DEFAULT_NUMBER_OF_SUB_PATH = 20;

typedef complex<double> ComplexType;


class FadingCalculationModel {
public:

    FadingCalculationModel() { }

    virtual ~FadingCalculationModel() { }

    virtual void GetFadingValueDb(
        const TimeType& targetTime,
        const NodeIdType& lowNodeId,
        const NodeIdType& highNodeId,
        double& returnFadingValueDb) = 0;

     virtual void UpdateVelocity(
        const TimeType& currentTime,
        const NodeIdType& nodeId,
        const double& currentVelocityMetersPerSecond,
        const double& currentVelocityAzimuthDegrees,
        const double& currentVelocityElevationDegrees) = 0;

private:

};//FadingCalculationModel//


class FadingModel {
public:

    FadingModel(
        const string& fadingModelStringInLowerCase,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const RandomNumberGeneratorSeedType& runSeed,
        const InterfaceOrInstanceIdType& channelId,
        const double& initCarrierFrequencyMhz,
        const double& initChannelBandwidthMhz);

    ~FadingModel() { }

    void FadingResultValueDb(
        const TimeType& targetTime,
        const NodeIdType& lowNodeId,
        const NodeIdType& highNodeId,
        double& returnFadingValueDb);

     void UpdateVelocity(
        const TimeType& currentTime,
        const NodeIdType& nodeId,
        const double& currentVelocityMetersPerSecond,
        const double& currentVelocityAzimuthDegrees,
        const double& currentVelocityElevationDegrees);

private:
    vector<shared_ptr<FadingCalculationModel> > fadingCalculationModelPtrs;

    //diversity
    bool selectionCombiningDiversityIsAvailable;

};//FadingModel//



//based on jakes model
class RayleighSignalGenerator {
public:

    RayleighSignalGenerator(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const RandomNumberGeneratorSeedType& runSeed,
        const InterfaceOrInstanceIdType& channelId,
        const double& initCarrierFrequencyMhz,
        const double& initChannelBandwidthMhz,
        const unsigned int calcModelIndex,
        const unsigned int generatorIndex = 0);

    void GetValueNonDb(
        const TimeType& targetTime,
        const NodeIdType& lowNodeId,
        const NodeIdType& highNodeId,
        double& returnValueNonDb);

     void UpdateVelocity(
        const TimeType& currentTime,
        const NodeIdType& nodeId,
        const double& newVelocityMetersPerSecond,
        const double& newVelocityAzimuthDegrees,
        const double& newVelocityElevationDegrees);

private:
    double carrierFrequencyMhz;
    double channelBandwidthMhz;
    unsigned int modelInstanceSeed;

    static const int SEED_HASH = 16864187;

    unsigned int numOfPath;
    unsigned int numOfSubPath;

    double carrierWavelenghMeters;

    //fixed velocity
    bool fixedRelativeVelocity;
    double fixedRelativeVelocityMetersPerSecond;

    //dynamic velocity
    double minimumRelativeVelocityMetersPerSecond;

    struct VelocityInfo {
        TimeType timestamp;

        double currentVelocityMetersPerSecond;
        double currentVelocityAzimuthDegrees;
        double currentVelocityElevationDegrees;

        double previousVelocityMetersPerSecond;
        double previousVelocityAzimuthDegrees;
        double previousVelocityElevationDegrees;

        bool updated;

        VelocityInfo(
            const TimeType& initTimestamp = ZERO_TIME,
            const double& initCurrentVelocityMetersPerSecond = 0.0,
            const double& initCurrentVelocityAzimuthDegrees = 0.0,
            const double& initCurrentVelocityElevationDegrees = 0.0,
            const double& initPreviousVelocityMetersPerSecond = 0.0,
            const double& initPreviousVelocityAzimuthDegrees = 0.0,
            const double& initPreviousVelocityElevationDegrees = 0.0,
            const bool initUpdated = true)
            :
        timestamp(initTimestamp),
        currentVelocityMetersPerSecond(initCurrentVelocityMetersPerSecond),
        currentVelocityAzimuthDegrees(initCurrentVelocityAzimuthDegrees),
        currentVelocityElevationDegrees(initCurrentVelocityElevationDegrees),
        previousVelocityMetersPerSecond(initPreviousVelocityMetersPerSecond),
        previousVelocityAzimuthDegrees(initPreviousVelocityAzimuthDegrees),
        previousVelocityElevationDegrees(initPreviousVelocityElevationDegrees),
        updated(initUpdated) { }

    };//VelocityInfo//

    map<NodeIdType, VelocityInfo> velocityInfoMap;

    struct NodePairType {
        NodeIdType lowNodeId;
        NodeIdType highNodeId;

        NodePairType() { }
        NodePairType(NodeIdType initLowNodeId, NodeIdType initHighNodeId)
            : lowNodeId(initLowNodeId), highNodeId(initHighNodeId)
            { assert(initLowNodeId < initHighNodeId); }

        bool operator<(const NodePairType& right) const {
            return ((this->lowNodeId < right.lowNodeId) ||
                    ((this->lowNodeId == right.lowNodeId) &&
                     (this->highNodeId < right.highNodeId)));
        }

    };//NodePairType//

    struct SubPathInfoType {
        double initialPhaseRad;
        double thetaAoARad;
        double previousPhaseShiftRad;

    };//SubPathInfoType//

    struct PathInfoType {
        vector<SubPathInfoType> subPathInfo;
        TimeType previousCalculationTime;

    };//PathInfoType//

    void PathInitialization(
        const set<NodeIdType>& setOfNodeIds,
        const RandomNumberGeneratorSeedType& runSeed);

    void InitializePathInfo(
        const NodeIdType& lowNodeId,
        const NodeIdType& highNodeId,
        RandomNumberGenerator& aRandomNumberGenerator,
        map<NodePairType, vector<PathInfoType> >& pathInfoMap);

    void InitializeSubPathInfo(
        PathInfoType& pathInfo,
        RandomNumberGenerator& aRandomNumberGenerator);

    // Should be matrix for ultimate speed.
    map<NodePairType, vector<PathInfoType> > pathInfoMap;

    void CalculateRalativeVelocity(
        const TimeType& targetTime,
        const NodeIdType& lowNodeId,
        const NodeIdType& highNodeId,
        double& relativeVelocityMetersPerSecond,
        double& relativeVelocityAzimuthFromNorthClockwiseRad) const;

    void UpdateSubPathInfoIfNecessary(
        const TimeType& currentTime,
        const NodeIdType& nodeId);

    void UpdateSubPathInfo(
        const TimeType& currentTime,
        const NodeIdType& lowNodeId,
        const NodeIdType& highNodeId);

    void UpdateSubPathPhase(
        const TimeType& currentTime,
        const double& relativeVelocityMetersPerSecond,
        const double& relativeVelocityAzimuthFromNorthClockwiseRad,
        vector<PathInfoType>& pathInfoVector);

    double CalculatePhaseShiftRad(
        const TimeType& elapsedTime,
        const double& relativeVelocityMetersPerSecond,
        const double& relativeVelocityAzimuthFromNorthClockwiseRad,
        const SubPathInfoType& subPathInfo) const;

    double CalculateFadingValueNonDb(
        const TimeType& targetTime,
        const double& relativeVelocityMetersPerSecond,
        const double& relativeVelocityAzimuthFromNorthClockwiseRad,
        const PathInfoType& pathInfo) const;

};//RayleighSignalGenerator//


inline
RayleighSignalGenerator::RayleighSignalGenerator(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const RandomNumberGeneratorSeedType& runSeed,
    const InterfaceOrInstanceIdType& channelId,
    const double& initCarrierFrequencyMhz,
    const double& initChannelBandwidthMhz,
    const unsigned int calcModelIndex,
    const unsigned int generatorIndex)
    :
    carrierFrequencyMhz(initCarrierFrequencyMhz),
    channelBandwidthMhz(initChannelBandwidthMhz),
    modelInstanceSeed(HashInputsToMakeSeed(calcModelIndex, generatorIndex)),
    numOfPath(1),
    numOfSubPath(DEFAULT_NUMBER_OF_SUB_PATH),
    fixedRelativeVelocity(false),
    fixedRelativeVelocityMetersPerSecond(0.0),
    minimumRelativeVelocityMetersPerSecond(0.0),
    carrierWavelenghMeters(SPEED_OF_LIGHT_METERS_PER_SECOND / (initCarrierFrequencyMhz * 1000 * 1000))
{

    if (theParameterDatabaseReader.ParameterExists("fading-number-of-sub-path",channelId)) {
        numOfSubPath = theParameterDatabaseReader.ReadNonNegativeInt("fading-number-of-sub-path",channelId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("fading-enable-fixed-velocity",channelId)) {
        fixedRelativeVelocity =
            theParameterDatabaseReader.ReadBool("fading-enable-fixed-velocity",channelId);
    }//if//

    if (fixedRelativeVelocity) {

        const double fixedRelativeVelocityKiloMetersPerHour =
            theParameterDatabaseReader.ReadDouble("fading-fixed-velocity-km-per-hour", channelId);

        if (fixedRelativeVelocityKiloMetersPerHour < 0.0) {
            cerr << "Error: fading-fixed-velocity-km-per-hour value: " << fixedRelativeVelocityKiloMetersPerHour << endl;
            exit(1);
        }//if//

        fixedRelativeVelocityMetersPerSecond = fixedRelativeVelocityKiloMetersPerHour * 1000.0 / 3600.0;

    }
    else {

        const double minimumRelativeVelocityKiloMetersPerHour =
            theParameterDatabaseReader.ReadDouble("fading-minimum-velocity-km-per-hour", channelId);

        if (minimumRelativeVelocityKiloMetersPerHour < 0.0) {
            cerr << "Error: fading-minimum-velocity-km-per-hour value: " << minimumRelativeVelocityKiloMetersPerHour << endl;
            exit(1);
        }//if//

        minimumRelativeVelocityMetersPerSecond = minimumRelativeVelocityKiloMetersPerHour * 1000.0 / 3600.0;

    }//if//

    //get communication node id
    set<NodeIdType> setOfNodeIds;
    theParameterDatabaseReader.MakeSetOfAllCommNodeIds(setOfNodeIds);

    (*this).PathInitialization(setOfNodeIds, runSeed);

}


inline
void RayleighSignalGenerator::PathInitialization(
    const set<NodeIdType>& setOfNodeIds,
    const RandomNumberGeneratorSeedType& runSeed)
{

    //initialize path and subpath info
    typedef set<NodeIdType>::const_iterator IterType;
    IterType lowNodeIter;
    IterType highNodeIter;

    for(IterType lowNodeIter = setOfNodeIds.begin();
        lowNodeIter != setOfNodeIds.end(); ++lowNodeIter) {

        const NodeIdType lowNodeId = (*lowNodeIter);

        for(IterType highNodeIter = lowNodeIter;
            highNodeIter != setOfNodeIds.end(); ++highNodeIter) {

            const NodeIdType highNodeId = (*highNodeIter);

            if (lowNodeId >= highNodeId) continue;

            const RandomNumberGeneratorSeedType lowNodeSeed =
                HashInputsToMakeSeed(runSeed, lowNodeId);
            const RandomNumberGeneratorSeedType highNodeSeed =
                    HashInputsToMakeSeed(runSeed, highNodeId);

            const RandomNumberGeneratorSeedType nodePairSeed =
                    HashInputsToMakeSeed(lowNodeSeed, highNodeSeed);

            RandomNumberGenerator aRandomNumberGenerator(
                HashInputsToMakeSeed(nodePairSeed, modelInstanceSeed, SEED_HASH));

            (*this).InitializePathInfo(
                lowNodeId, highNodeId, aRandomNumberGenerator, pathInfoMap);

        }//for//

    }//for//

}//PathInitialization//


inline
void RayleighSignalGenerator::InitializePathInfo(
    const NodeIdType& lowNodeId,
    const NodeIdType& highNodeId,
    RandomNumberGenerator& aRandomNumberGenerator,
    map<NodePairType, vector<PathInfoType> >& pathInfoMap)
{
    vector<PathInfoType>& pathInfos = pathInfoMap[NodePairType(lowNodeId, highNodeId)];

    pathInfos.resize(numOfPath);

    //per path
    for(size_t pathIndex = 0; (pathIndex < pathInfos.size()); pathIndex++) {

        PathInfoType& pathInfo = pathInfos[pathIndex];

        pathInfo.previousCalculationTime = ZERO_TIME;

        //initilize subpath
        InitializeSubPathInfo(pathInfo, aRandomNumberGenerator);

        //calculate phase shift for delayed path

    }//for//

}//InitializePathInfo//


inline
void RayleighSignalGenerator::InitializeSubPathInfo(
    PathInfoType& pathInfo,
    RandomNumberGenerator& aRandomNumberGenerator)
{
    pathInfo.subPathInfo.resize(numOfSubPath);

    //per subpath
    for(size_t subPathIndex = 0; subPathIndex < numOfSubPath; subPathIndex++) {

        SubPathInfoType& subPathInfo = pathInfo.subPathInfo[subPathIndex];

        subPathInfo.initialPhaseRad =
            2.0 * PI * aRandomNumberGenerator.GenerateRandomDouble();

        subPathInfo.thetaAoARad =
            2.0 * PI * subPathIndex / numOfSubPath;//based on jakes model

        subPathInfo.previousPhaseShiftRad = 0.0;

    }//for//

}//InitializeSubPathInfo//


inline
void RayleighSignalGenerator::UpdateVelocity(
    const TimeType& currentTime,
    const NodeIdType& nodeId,
    const double& newVelocityMetersPerSecond,
    const double& newVelocityAzimuthDegrees,
    const double& newVelocityElevationDegrees)
{

    assert(!fixedRelativeVelocity);

    if (velocityInfoMap.find(nodeId) == velocityInfoMap.end()) {
        // new
        velocityInfoMap.insert(make_pair(
            nodeId, VelocityInfo(currentTime, newVelocityMetersPerSecond, newVelocityAzimuthDegrees, newVelocityElevationDegrees)));
    }
    else {

        VelocityInfo& velocityInfo = velocityInfoMap[nodeId];

        if ((velocityInfo.currentVelocityMetersPerSecond != newVelocityMetersPerSecond) ||
            (velocityInfo.currentVelocityAzimuthDegrees != newVelocityAzimuthDegrees) ||
            (velocityInfo.currentVelocityElevationDegrees != newVelocityElevationDegrees)) {

            //updated
            velocityInfo.updated = true;
            velocityInfo.previousVelocityMetersPerSecond = velocityInfo.currentVelocityMetersPerSecond;
            velocityInfo.previousVelocityAzimuthDegrees = velocityInfo.currentVelocityAzimuthDegrees;
            velocityInfo.previousVelocityElevationDegrees = velocityInfo.currentVelocityElevationDegrees;
            velocityInfo.currentVelocityMetersPerSecond = newVelocityMetersPerSecond;
            velocityInfo.currentVelocityAzimuthDegrees = newVelocityAzimuthDegrees;
            velocityInfo.currentVelocityElevationDegrees = newVelocityElevationDegrees;

        }
        else {
            velocityInfo.updated = false;
        }//if//

        velocityInfo.timestamp = currentTime;

    }//if//

    UpdateSubPathInfoIfNecessary(currentTime, nodeId);

}//UpdateVelocity//


inline
void RayleighSignalGenerator::UpdateSubPathInfoIfNecessary(
    const TimeType& currentTime,
    const NodeIdType& nodeId)
{

    assert(!fixedRelativeVelocity);

    const VelocityInfo& nodeVelocity = velocityInfoMap[nodeId];

    assert(nodeVelocity.timestamp == currentTime);

    map<NodeIdType, VelocityInfo>::iterator iter;

    for(iter = velocityInfoMap.begin(); iter != velocityInfoMap.end(); ++iter) {

        const NodeIdType& targetNodeId = iter->first;
        const VelocityInfo& targetNodeVelocity = iter->second;

        if((nodeVelocity.updated) ||
            ((targetNodeVelocity.timestamp == currentTime) && (targetNodeVelocity.updated))) {

            NodeIdType lowNodeId;
            NodeIdType highNodeId;

            if (nodeId < targetNodeId) {
                lowNodeId = nodeId;
                highNodeId = targetNodeId;
            }
            else if (nodeId > targetNodeId) {
                lowNodeId = targetNodeId;
                highNodeId = nodeId;
            }
            else {
                //next target
                continue;
            }

            const NodePairType nodePair(lowNodeId, highNodeId);

            if(pathInfoMap.find(nodePair) == pathInfoMap.end()) continue;

            UpdateSubPathInfo(currentTime, lowNodeId, highNodeId);

        }//if//

    }//for//

}//UpdateSubPathInfoIfNecessary//


inline
void RayleighSignalGenerator::UpdateSubPathPhase(
    const TimeType& currentTime,
    const double& relativeVelocityMetersPerSecond,
    const double& relativeVelocityAzimuthFromNorthClockwiseRad,
    vector<PathInfoType>& pathInfoVector)
{

    assert(!fixedRelativeVelocity);

    for(size_t pathIndex = 0; pathIndex < pathInfoVector.size(); pathIndex++) {

        PathInfoType& pathInfo = pathInfoVector[pathIndex];

        const TimeType elapsedTime =
            currentTime - pathInfo.previousCalculationTime;

        if (elapsedTime == ZERO_TIME) continue;

        for(size_t subPathIndex = 0; subPathIndex < pathInfo.subPathInfo.size(); subPathIndex++) {

            SubPathInfoType& subPathInfo = pathInfo.subPathInfo[subPathIndex];

            const double newPhaseShiftRad =
                CalculatePhaseShiftRad(
                    elapsedTime,
                    relativeVelocityMetersPerSecond,
                    relativeVelocityAzimuthFromNorthClockwiseRad,
                    subPathInfo);

            subPathInfo.previousPhaseShiftRad = newPhaseShiftRad;

        }//for//

        pathInfo.previousCalculationTime = currentTime;

    }//for//

}//UpdateSubPathPhase//


inline
void RayleighSignalGenerator::UpdateSubPathInfo(
    const TimeType& currentTime,
    const NodeIdType& lowNodeId,
    const NodeIdType& highNodeId)
{

    assert(!fixedRelativeVelocity);

    //calculate relative velocity

    const VelocityInfo& lowNodeVelocity = velocityInfoMap[lowNodeId];
    const VelocityInfo& highNodeVelocity = velocityInfoMap[highNodeId];

    double lowNodeVelocityMetersPerSecond = lowNodeVelocity.currentVelocityMetersPerSecond;
    double lowNodeVelocityAzimuthDegrees = lowNodeVelocity.currentVelocityAzimuthDegrees;
    double highNodeVelocityMetersPerSecond = highNodeVelocity.currentVelocityMetersPerSecond;
    double highNodeVelocityAzimuthDegrees = highNodeVelocity.currentVelocityAzimuthDegrees;

    if ((lowNodeVelocity.timestamp == currentTime) && (lowNodeVelocity.updated)) {
        lowNodeVelocityMetersPerSecond = lowNodeVelocity.previousVelocityMetersPerSecond;
        lowNodeVelocityAzimuthDegrees = lowNodeVelocity.previousVelocityAzimuthDegrees;
    }//if//

    if ((highNodeVelocity.timestamp == currentTime) && (highNodeVelocity.updated)) {
        highNodeVelocityMetersPerSecond = highNodeVelocity.previousVelocityMetersPerSecond;
        highNodeVelocityAzimuthDegrees = highNodeVelocity.previousVelocityAzimuthDegrees;
    }

    const ComplexType relativeVelocity =
        polar(highNodeVelocityMetersPerSecond, highNodeVelocityAzimuthDegrees * RADIANS_PER_DEGREE) -
        polar(lowNodeVelocityMetersPerSecond, lowNodeVelocityAzimuthDegrees * RADIANS_PER_DEGREE);

    double relativeVelocityMetersPerSecond = std::abs(relativeVelocity);
    double relativeVelocityAzimuthFromNorthClockwiseRad = arg(relativeVelocity);

    if (relativeVelocityMetersPerSecond < minimumRelativeVelocityMetersPerSecond) {
        relativeVelocityMetersPerSecond = minimumRelativeVelocityMetersPerSecond;
        relativeVelocityAzimuthFromNorthClockwiseRad = 0.0;
    }//if//

    //update phase

    const NodePairType nodePair(lowNodeId, highNodeId);

    assert(pathInfoMap.find(nodePair) != pathInfoMap.end());
    vector<PathInfoType>& firstPathInfoVector = pathInfoMap[nodePair];

    (*this).UpdateSubPathPhase(
        currentTime,
        relativeVelocityMetersPerSecond,
        relativeVelocityAzimuthFromNorthClockwiseRad,
        firstPathInfoVector);

}//UpdateSubPathInfo//


inline
double RayleighSignalGenerator::CalculatePhaseShiftRad(
    const TimeType& elapsedTime,
    const double& relativeVelocityMetersPerSecond,
    const double& relativeVelocityAzimuthFromNorthClockwiseRad,
    const SubPathInfoType& subPathInfo) const
{

    const double newPhaseShiftRad =
        2.0 * PI / carrierWavelenghMeters * relativeVelocityMetersPerSecond *
        cos(subPathInfo.thetaAoARad - relativeVelocityAzimuthFromNorthClockwiseRad) * ConvertTimeToDoubleSecs(elapsedTime)
        + subPathInfo.previousPhaseShiftRad;

    return newPhaseShiftRad;

}//CalculatePhaseShiftRad//


inline
void RayleighSignalGenerator::CalculateRalativeVelocity(
    const TimeType& targetTime,
    const NodeIdType& lowNodeId,
    const NodeIdType& highNodeId,
    double& relativeVelocityMetersPerSecond,
    double& relativeVelocityAzimuthFromNorthClockwiseRad) const
{
    assert(!fixedRelativeVelocity);

    if ((velocityInfoMap.find(lowNodeId) == velocityInfoMap.end()) ||
        (velocityInfoMap.find(highNodeId) == velocityInfoMap.end())) {

        //could happen when a node is created and a signal is transmitted to the node
        relativeVelocityMetersPerSecond = 0.0;
        relativeVelocityAzimuthFromNorthClockwiseRad = 0.0;
        return;
    }//if//

    const VelocityInfo& lowNodeVelocity = velocityInfoMap.find(lowNodeId)->second;
    const VelocityInfo& highNodeVelocity = velocityInfoMap.find(highNodeId)->second;

    const ComplexType relativeVelocity =
        polar(highNodeVelocity.currentVelocityMetersPerSecond, highNodeVelocity.currentVelocityAzimuthDegrees * RADIANS_PER_DEGREE) -
        polar(lowNodeVelocity.currentVelocityMetersPerSecond, lowNodeVelocity.currentVelocityAzimuthDegrees * RADIANS_PER_DEGREE);

    relativeVelocityMetersPerSecond = std::abs(relativeVelocity);
    relativeVelocityAzimuthFromNorthClockwiseRad = arg(relativeVelocity);

}//CalculateRalativeVelocity//


inline
double RayleighSignalGenerator::CalculateFadingValueNonDb(
    const TimeType& targetTime,
    const double& relativeVelocityMetersPerSecond,
    const double& relativeVelocityAzimuthFromNorthClockwiseRad,
    const PathInfoType& pathInfo) const
{

    ComplexType channelResponse(0.0, 0.0);

    const TimeType elapsedTime = targetTime - pathInfo.previousCalculationTime;

    assert(elapsedTime >= ZERO_TIME);

    for(size_t subPathIndex =0; subPathIndex < pathInfo.subPathInfo.size(); subPathIndex++) {

        const SubPathInfoType& subPathInfo = pathInfo.subPathInfo[subPathIndex];

        const double newPhaseShiftRad =
            CalculatePhaseShiftRad(
                elapsedTime,
                relativeVelocityMetersPerSecond,
                relativeVelocityAzimuthFromNorthClockwiseRad,
                subPathInfo);

        channelResponse += polar(1.0, newPhaseShiftRad + subPathInfo.initialPhaseRad);

    }//for//

    channelResponse /= sqrt((double)numOfSubPath);

    return (norm(channelResponse));

}//CalculateFadingValueNonDb//


inline
void RayleighSignalGenerator::GetValueNonDb(
    const TimeType& targetTime,
    const NodeIdType& lowNodeId,
    const NodeIdType& highNodeId,
    double& returnValueNonDb)
{
    //assuming unidirectional
    assert(lowNodeId < highNodeId);

    //calculate relative velocity
    double relativeVelocityMetersPerSecond;
    double relativeVelocityAzimuthFromNorthClockwiseRad;

    if (fixedRelativeVelocity) {
        relativeVelocityMetersPerSecond = fixedRelativeVelocityMetersPerSecond;
        relativeVelocityAzimuthFromNorthClockwiseRad = 0.0;

        //stopped
        if (relativeVelocityMetersPerSecond == 0.0) {
            returnValueNonDb = 1.0;
            return;
        }//if//

    }
    else {

        CalculateRalativeVelocity(
            targetTime,
            lowNodeId,
            highNodeId,
            relativeVelocityMetersPerSecond,
            relativeVelocityAzimuthFromNorthClockwiseRad);

        if (relativeVelocityMetersPerSecond < minimumRelativeVelocityMetersPerSecond) {
            relativeVelocityMetersPerSecond = minimumRelativeVelocityMetersPerSecond;
            relativeVelocityAzimuthFromNorthClockwiseRad = 0.0;
        }//if//

    }//if//

    const NodePairType nodePair(lowNodeId, highNodeId);

    map<NodePairType, vector<PathInfoType> >::iterator iter =
        pathInfoMap.find(nodePair);

    assert(iter != pathInfoMap.end());

    //currently assuming one path
    PathInfoType& pathInfo = (iter->second)[0];

    returnValueNonDb =
        (*this).CalculateFadingValueNonDb(
            targetTime,
            relativeVelocityMetersPerSecond,
            relativeVelocityAzimuthFromNorthClockwiseRad,
            pathInfo);

}//GetValueDb//



class RayleighFadingModel: public FadingCalculationModel {
public:

    RayleighFadingModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const RandomNumberGeneratorSeedType& runSeed,
        const InterfaceOrInstanceIdType& channelId,
        const double& initCarrierFrequencyMhz,
        const double& initChannelBandwidthMhz,
        const unsigned int calcModelIndex);

    void GetFadingValueDb(
        const TimeType& targetTime,
        const NodeIdType& lowNodeId,
        const NodeIdType& highNodeId,
        double& returnFadingValueDb);

     void UpdateVelocity(
        const TimeType& currentTime,
        const NodeIdType& nodeId,
        const double& newVelocityMetersPerSecond,
        const double& newVelocityAzimuthDegrees,
        const double& newVelocityElevationDegrees);

private:
    shared_ptr<RayleighSignalGenerator> rayleighSignalGenerator;

};//RayleighFadingModel//


inline
RayleighFadingModel::RayleighFadingModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const RandomNumberGeneratorSeedType& runSeed,
    const InterfaceOrInstanceIdType& channelId,
    const double& initCarrierFrequencyMhz,
    const double& initChannelBandwidthMhz,
    const unsigned int calcModelIndex)
    :
    rayleighSignalGenerator(new RayleighSignalGenerator(
        theParameterDatabaseReader,
        runSeed,
        channelId,
        initCarrierFrequencyMhz,
        initCarrierFrequencyMhz,
        calcModelIndex))
{

}


inline
void RayleighFadingModel::UpdateVelocity(
    const TimeType& currentTime,
    const NodeIdType& nodeId,
    const double& newVelocityMetersPerSecond,
    const double& newVelocityAzimuthDegrees,
    const double& newVelocityElevationDegrees)
{

    rayleighSignalGenerator->UpdateVelocity(
        currentTime,
        nodeId,
        newVelocityMetersPerSecond,
        newVelocityAzimuthDegrees,
        newVelocityElevationDegrees);

}//UpdateVelocity//


inline
void RayleighFadingModel::GetFadingValueDb(
    const TimeType& targetTime,
    const NodeIdType& lowNodeId,
    const NodeIdType& highNodeId,
    double& returnFadingValueDb)
{
    //assuming unidirectional
    assert(lowNodeId < highNodeId);

    double rayleighFadingValueNonDb;

    rayleighSignalGenerator->GetValueNonDb(
        targetTime,
        lowNodeId,
        highNodeId,
        rayleighFadingValueNonDb);

    returnFadingValueDb = ConvertToDb(rayleighFadingValueNonDb);

}//FadingResultValueDb//



class NakagamiFadingModel: public FadingCalculationModel {
public:

    NakagamiFadingModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const RandomNumberGeneratorSeedType& runSeed,
        const InterfaceOrInstanceIdType& channelId,
        const double& initCarrierFrequencyMhz,
        const double& initChannelBandwidthMhz,
        const unsigned int calcModelIndex);

    void GetFadingValueDb(
        const TimeType& targetTime,
        const NodeIdType& lowNodeId,
        const NodeIdType& highNodeId,
        double& returnFadingValueDb);

     void UpdateVelocity(
        const TimeType& currentTime,
        const NodeIdType& nodeId,
        const double& newVelocityMetersPerSecond,
        const double& newVelocityAzimuthDegrees,
        const double& newVelocityElevationDegrees);

private:
    vector<shared_ptr<RayleighSignalGenerator> > rayleighSignalGenerators;

};//NakagamiFadingModel//


inline
NakagamiFadingModel::NakagamiFadingModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const RandomNumberGeneratorSeedType& runSeed,
    const InterfaceOrInstanceIdType& channelId,
    const double& initCarrierFrequencyMhz,
    const double& initChannelBandwidthMhz,
    const unsigned int calcModelIndex)
{

    const unsigned int shapeFactorM =
        theParameterDatabaseReader.ReadNonNegativeInt("fading-nakagami-shape-factor-m", channelId);

    if (shapeFactorM < 1) {
        cerr << "fading-nakagami-shape-factor-m: " << shapeFactorM << " must be 1 or more" << endl;
        exit(1);
    }//if//

    for(unsigned int i = 0; i < shapeFactorM; i++) {

        const unsigned int rayleighGeneratorIndex = (calcModelIndex * shapeFactorM) + i;

        rayleighSignalGenerators.push_back(
            shared_ptr<RayleighSignalGenerator>(new RayleighSignalGenerator(
                theParameterDatabaseReader,
                runSeed,
                channelId,
                initCarrierFrequencyMhz,
                initCarrierFrequencyMhz,
                rayleighGeneratorIndex)));

    }//for//

}


inline
void NakagamiFadingModel::UpdateVelocity(
    const TimeType& currentTime,
    const NodeIdType& nodeId,
    const double& newVelocityMetersPerSecond,
    const double& newVelocityAzimuthDegrees,
    const double& newVelocityElevationDegrees)
{

    for(size_t i = 0; i < rayleighSignalGenerators.size(); i++) {

        rayleighSignalGenerators[i]->UpdateVelocity(
            currentTime,
            nodeId,
            newVelocityMetersPerSecond,
            newVelocityAzimuthDegrees,
            newVelocityElevationDegrees);

    }//for//

}//UpdateVelocity//


inline
void NakagamiFadingModel::GetFadingValueDb(
    const TimeType& targetTime,
    const NodeIdType& lowNodeId,
    const NodeIdType& highNodeId,
    double& returnFadingValueDb)
{
    //assuming unidirectional
    assert(lowNodeId < highNodeId);

    vector<double> rayleighElements(rayleighSignalGenerators.size());
    double sumOfSquares = 0.0;

    for(size_t i = 0; i < rayleighSignalGenerators.size(); i++) {

        rayleighSignalGenerators[i]->GetValueNonDb(
            targetTime,
            lowNodeId,
            highNodeId,
            rayleighElements[i]);

        sumOfSquares += (rayleighElements[i] * rayleighElements[i]);

    }//for//

    returnFadingValueDb = ConvertToDb(sqrt(sumOfSquares));

}//FadingResultValueDb//



inline
FadingModel::FadingModel(
    const string& fadingModelStringInLowerCase,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const RandomNumberGeneratorSeedType& runSeed,
    const InterfaceOrInstanceIdType& channelId,
    const double& initCarrierFrequencyMhz,
    const double& initChannelBandwidthMhz)
    :
    selectionCombiningDiversityIsAvailable(false)
{

    if (theParameterDatabaseReader.ParameterExists("fading-enable-selection-combining-diversity",channelId)) {
        selectionCombiningDiversityIsAvailable =
            theParameterDatabaseReader.ReadBool("fading-enable-selection-combining-diversity",channelId);
    }//if//

    size_t numberOfFadingCalculationModel = 1;

    //TBD: support more than two antennas
    if (selectionCombiningDiversityIsAvailable) {
        numberOfFadingCalculationModel = 2;
    }//if//

    for(unsigned int i = 0; i < numberOfFadingCalculationModel; i++) {


        if (fadingModelStringInLowerCase == "rayleigh") {

            fadingCalculationModelPtrs.push_back(
                shared_ptr<FadingCalculationModel>(new RayleighFadingModel(
                    theParameterDatabaseReader,
                    runSeed,
                    channelId,
                    initCarrierFrequencyMhz,
                    initChannelBandwidthMhz,
                    i)));


        }
        else if (fadingModelStringInLowerCase == "nakagami") {

            fadingCalculationModelPtrs.push_back(
                shared_ptr<FadingCalculationModel>(new NakagamiFadingModel(
                    theParameterDatabaseReader,
                    runSeed,
                    channelId,
                    initCarrierFrequencyMhz,
                    initChannelBandwidthMhz,
                    i)));
        }
        else {

            cerr << "Error: Fading Model: " << fadingModelStringInLowerCase << " is invalid." << endl;
            exit(1);

        }//if//

    }//for//

}


inline
void FadingModel::FadingResultValueDb(
    const TimeType& targetTime,
    const NodeIdType& lowNodeId,
    const NodeIdType& highNodeId,
    double& returnFadingValueDb)
{

    if (fadingCalculationModelPtrs.size() == 1) {

        //no diversity
        fadingCalculationModelPtrs[0]->GetFadingValueDb(
            targetTime,
            lowNodeId,
            highNodeId,
            returnFadingValueDb);

    }
    else {

        //diversity with selective combining

        assert(fadingCalculationModelPtrs.size() == 2);

        double firstValueDb;
        double secondValueDb;

        fadingCalculationModelPtrs[0]->GetFadingValueDb(
            targetTime,
            lowNodeId,
            highNodeId,
            firstValueDb);

        fadingCalculationModelPtrs[1]->GetFadingValueDb(
            targetTime,
            lowNodeId,
            highNodeId,
            secondValueDb);

        returnFadingValueDb = std::max(firstValueDb, secondValueDb);

    }//if//

}//FadingResultValueDb//


inline
void FadingModel::UpdateVelocity(
    const TimeType& currentTime,
    const NodeIdType& nodeId,
    const double& currentVelocityMetersPerSecond,
    const double& currentVelocityAzimuthDegrees,
    const double& currentVelocityElevationDegrees)
{

    for(size_t i = 0; i < fadingCalculationModelPtrs.size(); i++) {

        fadingCalculationModelPtrs[i]->UpdateVelocity(
            currentTime,
            nodeId,
            currentVelocityMetersPerSecond,
            currentVelocityAzimuthDegrees,
            currentVelocityElevationDegrees);


    }//for//

}//UpdateVelocity//

}//namespace//

#endif

