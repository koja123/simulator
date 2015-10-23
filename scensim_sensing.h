// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

//--------------------------------------------------------------
// Abstract sensing model based on a simulation time step synchronization.
// All sensing status will be updated on each time step.
// time step parameter: time-step-event-synchronization-step
//

#ifndef SCENSIM_SENSING_H
#define SCENSIM_SENSING_H

#include "scensim_nodeid.h"
#include "scensim_support.h"
#include "scensim_application.h"
#include "scensim_gis.h"

namespace ScenSim {

using std::cerr;
using std::endl;

class NetworkSimulator;
class SensingModel;
class ShapeSensingModel;
class SensingSubsystem;


const double DEGREES_PER_RADIAN = 1. / RADIANS_PER_DEGREE;

#ifdef ENABLE_SENSING_DEBUG
const bool DEBUG_SENSING_DETECTION = true;
#else
const bool DEBUG_SENSING_DETECTION = false;
#endif


static inline
ObjectMobilityPosition MakeStationaryMobilityPositionFromVertex(
    const Vertex& vertex,
    const NodeIdType& nodeId)
{
    return ObjectMobilityPosition(ZERO_TIME, INFINITE_TIME, vertex.x, vertex.y, vertex.z, true/*TheHeightContainsGroundHeightMeters*/, nodeId);
}//MakeVertexFromMobilityPosition//

static inline
double CalculateDistanceError(
    const double standardDeviation,
    RandomNumberGenerator& aRandomNumberGenerator)
{
    // central limit theorem

    double total = 0;

    for (int i = 0; i < 12; i++) {
        total += aRandomNumberGenerator.GenerateRandomDouble();
    }

    return standardDeviation * (total - 6.0);
}//CalculateDistanceError//

static inline
ObjectMobilityPosition CalculatePositionWithError(
    const ObjectMobilityPosition& origPosition,
    const double positionErrorStandardDeviationMeters,
    RandomNumberGenerator& aRandomNumberGenerator)
{
    // simple linear error correction.

    ObjectMobilityPosition targetPosition = origPosition;

    const double offsetMeters = CalculateDistanceError(
        positionErrorStandardDeviationMeters,
        aRandomNumberGenerator);

    const double horizontalOffsetRaians =
        aRandomNumberGenerator.GenerateRandomDouble() * (2*PI);

    const double verticalOffsetRaians =
        aRandomNumberGenerator.GenerateRandomDouble() * (2*PI);

    const double horizontalOffsetMeters =
        (std::cos(verticalOffsetRaians) * offsetMeters);

    const double heightOffsetMeters =
        (std::sin(verticalOffsetRaians) * offsetMeters);

    const double xOffsetMeters =
        (std::cos(horizontalOffsetRaians) * horizontalOffsetMeters);

    const double yOffsetMeters =
        (std::sin(horizontalOffsetRaians) * horizontalOffsetMeters);

    targetPosition.SetX_PositionMeters(targetPosition.X_PositionMeters() + xOffsetMeters);
    targetPosition.SetY_PositionMeters(targetPosition.Y_PositionMeters() + yOffsetMeters);
    targetPosition.SetHeightFromGroundMeters(targetPosition.HeightFromGroundMeters() + heightOffsetMeters);

    return targetPosition;

}//GetErrorCorrectionPosition//

static inline
void MakeSensingPositionFromMobilityCache(
    const map<NodeIdType, ObjectMobilityPosition>& mobilityPositionCache,
    const NodeIdType& nodeId,
    const double sensingHeightFromPlatformMeters,
    bool& success,
    ObjectMobilityPosition& sensingMobilityPosition)
{
    typedef map<NodeIdType, ObjectMobilityPosition>::const_iterator IterType;

    IterType iter = mobilityPositionCache.find(nodeId);

    if (iter == mobilityPositionCache.end()) {
        success = false;
        return;
    }//if//

    success = true;

    const ObjectMobilityPosition& sensingPlatformPosition = (*iter).second;

    sensingMobilityPosition = sensingPlatformPosition;

    sensingMobilityPosition.SetHeightFromGroundMeters(
        sensingPlatformPosition.HeightFromGroundMeters() +
        sensingHeightFromPlatformMeters);

}//MakeSensingPositionFromMobilityCache//


//--------------------------------------------------------------------------------------------------

enum DetectionConditionType {
    DETECTION_CONDITION_LOS_ONLY,
    DETECTION_CONDITION_LOS_AND_NLOS,
};

struct DetectedNodeResult {
    ObjectMobilityPosition position;
    bool hasLineOfSight;


    NodeIdType nodeId;

    DetectedNodeResult()
        :
        hasLineOfSight(false),
        nodeId(INVALID_NODEID)
    {}

    DetectedNodeResult(
        const NodeIdType& initNodeId,
        const ObjectMobilityPosition& initPosition,
        const bool initHasLineOfSight)
        :
        position(initPosition),
        hasLineOfSight(initHasLineOfSight),
        nodeId(initNodeId)
    {}

};//DetectedNodeResult//

struct DetectedGisObjectResult {
    ObjectMobilityPosition position;
    bool hasLineOfSight;


    GisPositionIdType positionId;

    DetectedGisObjectResult()
        :
        hasLineOfSight(false)
    {}

    DetectedGisObjectResult(
        const GisPositionIdType& initPositionId,
        const ObjectMobilityPosition& initPosition,
        const bool initHasLineOfSight)
        :
        position(initPosition),
        hasLineOfSight(initHasLineOfSight),
        positionId(initPositionId)
    {}

};//DetectedGisObjectResult//


struct SensingResult {

    vector<DetectedNodeResult> nodeDetectionResults;

    vector<DetectedGisObjectResult> gisObjectDetectionResults;

};//SensingResult//

typedef ExtrinsicPacketInfoContainer SensingSharedInfoType;

enum TransmissionConditionType {
    TRANSMISSION_CONDITION_SIMPLEX,
    TRANSMISSION_CONDITION_DUPLEX,
};


// Abstract sensing models peek the global information to detect communication objects.
// Only "SensingSubsystem" can update the global information variables.

struct SensingGlobalInfo {

    TimeType nextTopologyUpdateTime; //for optimization

    // should be quadrant tree cache
    map<NodeIdType, ObjectMobilityPosition> mobilityPositionCache;

    SensingGlobalInfo()
        :
        nextTopologyUpdateTime(ZERO_TIME)
    {}

};//SensingGlobalInfo//


// Sensing creation interface for node.
class SensingSubsystemInterface: public enable_shared_from_this<SensingSubsystemInterface> {
public:

    // Thread-safe between simulation partitions.
    shared_ptr<SensingModel> CreateSensingModel(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const InterfaceOrInstanceIdType& initInstanceId,
        const RandomNumberGeneratorSeedType& initNodeSeed);

    shared_ptr<ShapeSensingModel> CreateShapeSensingModel(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const InterfaceOrInstanceIdType& initInstanceId,
        const RandomNumberGeneratorSeedType& initNodeSeed);

    // Shared information will be transmitted ideally.
    void TransmitDataIdeally(
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SensingModel>& transmitterSensingPtr,
        const SensingSharedInfoType& sharedInfo,
        const TransmissionConditionType& transmissionCondition,
        const double errorRate);

    const SensingGlobalInfo& GetSensingGlobalInfo() const;

private:
    friend class SensingSubsystem;

    // construct from SensingSubsystem
    SensingSubsystemInterface(SensingSubsystem* initSensingSubsystemPtr)
        : sensingSubsystemPtr(initSensingSubsystemPtr) {}

    SensingSubsystem* sensingSubsystemPtr;
};//SensingSubsystemInterface//

//-----------------------------------------------------------

// Sensing Application
class SensingApplication: public Application, public enable_shared_from_this<SensingApplication> {
public:
    static const string modelName;

    SensingApplication(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<SensingModel>& initSensingModelPtr,
        const NodeIdType& initNodeId,
        const InterfaceOrInstanceIdType& initInstanceId);

    void CompleteInitialization();

    // Light detection check for one or more objects.
    bool DetectedAnObject();
    bool DetectedANode();
    bool DetectedAGisObject();


    // Get detected number of objects.
    size_t GetNumberOfDetectedAllObjects();// = numbef of nodes + numbef GIS objects
    size_t GetNumberOfDetectedNodes();
    size_t GetNumberOfDetectedGisObjects();


    // Detection check with object id specification.
    bool TargetNodeIsDetected(const NodeIdType& nodeId);
    bool TargetGisObjectIsDetected (const GisPositionIdType& positionId);


    // Detailed detection information
    const vector<DetectedNodeResult>& GetCurrentDetectedNodeResults();
    const vector<DetectedGisObjectResult>& GetCurrentDetectedGisObjectResults();



    void SetSensingModel(const shared_ptr<SensingModel>& initSensingModelPtr) { sensingModelPtr = initSensingModelPtr; }


protected:
    void UpdateSensingResultsIfNecessary();

    // Cache update functions
    void CacheDetectedNodeResult();
    void CacheDetectedGisObjectResult();

    // Sensing
    shared_ptr<SensingModel> sensingModelPtr;

    // information base
    SensingResult sensingResult;

    // cache
    set<NodeIdType> detectedNodeIdCache;
    set<GisPositionIdType> detectedGisPositionIdCache;



    // information output option
    TimeType sensingStartTime;
    TimeType sensingEndTime;
    TimeType sensingInterval;

    class SensingObjectsEvent: public SimulationEvent {
    public:
        explicit
        SensingObjectsEvent(
            const shared_ptr<SensingApplication>& initSensingAppPtr)
            :
            sensingAppPtr(initSensingAppPtr)
        {}

        virtual void ExecuteEvent() { sensingAppPtr->UpdateSensingObjectResults(); }
    private:
        shared_ptr<SensingApplication> sensingAppPtr;
    };//SensingObjectsEvent//


    // for Stats
    shared_ptr<RealStatistic> nodeDetectionStatPtr;
    shared_ptr<RealStatistic> gisObjectDetectionStatPtr;

    void UpdateSensingObjectResults();
    void OutputTraceAndStatsForObjectDetection();

};//SensingApplication//



//-----------------------------------------------------------------------

// Abstract Sensing Shape

class SensingShape {
public:
    virtual void SetInitialBasePosition(const ObjectMobilityPosition& basePosition) {}

    virtual bool IsInTheCoverage(
        const ObjectMobilityPosition& basePosition,
        const ObjectMobilityPosition& targetPosition) const = 0;

    virtual Rectangle CalculateMaxCoverageRectangle(
        const ObjectMobilityPosition& basePosition) const = 0;

};//SensingShape//


// Simple Fan Shape

class FanSensingShape : public SensingShape {
public:

    FanSensingShape()
        :
        azimuthFromPlatformDegrees(0.),
        elevationFromPlatformDegrees(0.),
        coverageDistanceMeters(100.),
        halfHorizontalCoverageFromBoresightDegrees(15.),
        halfVerticalCoverageFromBoresightDegrees(15.)
    {}

    FanSensingShape(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const NodeIdType& initNodeId,
        const InterfaceOrInstanceIdType& initInstanceId);

    virtual bool IsInTheCoverage(
        const ObjectMobilityPosition& basePosition,
        const ObjectMobilityPosition& targetPosition) const;

    virtual Rectangle CalculateMaxCoverageRectangle(
        const ObjectMobilityPosition& basePosition) const;

    void SetAzimuthFromPlatformDegrees(const double initAzimuthFromPlatformDegrees) { azimuthFromPlatformDegrees = NormalizeAngleToNeg180To180(initAzimuthFromPlatformDegrees); }

    void SetElevationFromPlatformDegrees(const double initElevationFromPlatformDegrees) { elevationFromPlatformDegrees = NormalizeAngleToNeg180To180(initElevationFromPlatformDegrees); }

    void SetCoverageDistanceMeters(const double initCoverageDistanceMeters) { coverageDistanceMeters = initCoverageDistanceMeters; }

    void SetHorizontalCoverageFromBoresightDegrees(const double initHorizontalCoverageFromBoresightDegrees) { halfHorizontalCoverageFromBoresightDegrees = initHorizontalCoverageFromBoresightDegrees*0.5; }

    void SetVerticalCoverageFromBoresightDegrees(const double initVerticalCoverageFromBoresightDegrees) { halfVerticalCoverageFromBoresightDegrees = initVerticalCoverageFromBoresightDegrees*0.5; }

private:
    double azimuthFromPlatformDegrees;
    double elevationFromPlatformDegrees;
    double coverageDistanceMeters;
    double halfHorizontalCoverageFromBoresightDegrees;
    double halfVerticalCoverageFromBoresightDegrees;

};//FanSensingShape//


// GIS Object polygon based Shape

class GisObjectSensingShape : public SensingShape {
public:
    GisObjectSensingShape()
        :
        coverageHeightMeters(10.)
    {}

    GisObjectSensingShape(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const NodeIdType& initNodeId,
        const InterfaceOrInstanceIdType& initInstanceId);

    virtual void SetInitialBasePosition(const ObjectMobilityPosition& basePosition);

    virtual bool IsInTheCoverage(
        const ObjectMobilityPosition& basePosition,
        const ObjectMobilityPosition& targetPosition) const;

    virtual Rectangle CalculateMaxCoverageRectangle(
        const ObjectMobilityPosition& basePosition) const {

        Rectangle entireRect;

        for(size_t i = 0; i < coverageGisObjectPolygonFromBases.size(); i++) {
            const vector<Vertex>& coverageGisObjectPolygonFromBase = coverageGisObjectPolygonFromBases[i];

            const Rectangle rect = GetPointsRect(
                (*this).GetCoverageGisObjectPolygon(
                    coverageGisObjectPolygonFromBase,
                    basePosition));

            entireRect += rect;
        }//for//

        return entireRect;
    }

    void SetCoverageArea(
        const vector<Vertex>& initCoverageGisObjectPolygonFromBase,
        const double initCoverageHeightMeters) {

        coverageGisObjectPolygonFromBases.clear();
        coverageGisObjectPolygonFromBases.push_back(initCoverageGisObjectPolygonFromBase);

        coverageHeightMeters = initCoverageHeightMeters;
    }

private:
    vector<Vertex> GetCoverageGisObjectPolygon(
        const vector<Vertex>& coverageGisObjectPolygonFromBase,
        const ObjectMobilityPosition& basePosition) const;

    vector<vector<Vertex> > coverageGisObjectPolygonFromBases;
    double coverageHeightMeters;

};//FanSensingShape//

inline
FanSensingShape::FanSensingShape(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const NodeIdType& initNodeId,
    const InterfaceOrInstanceIdType& initInstanceId)
    :
    azimuthFromPlatformDegrees(0.),
    elevationFromPlatformDegrees(0.),
    coverageDistanceMeters(100.),
    halfHorizontalCoverageFromBoresightDegrees(15.),
    halfVerticalCoverageFromBoresightDegrees(15.)
{
    coverageDistanceMeters =
        parameterDatabaseReader.ReadDouble("sensing-coverage-distance-meters", initNodeId, initInstanceId);

    azimuthFromPlatformDegrees =
        NormalizeAngleToNeg180To180(
            parameterDatabaseReader.ReadDouble("sensing-azimuth-degrees", initNodeId, initInstanceId));

    elevationFromPlatformDegrees =
        NormalizeAngleToNeg180To180(
            parameterDatabaseReader.ReadDouble("sensing-elevation-degrees", initNodeId, initInstanceId));

    const double horizontalCoverageFromBoresightDegrees =
        parameterDatabaseReader.ReadDouble("sensing-horizontal-coverage-degrees", initNodeId, initInstanceId);

    const double verticalCoverageFromBoresightDegrees =
        parameterDatabaseReader.ReadDouble("sensing-vertical-coverage-degrees", initNodeId, initInstanceId);

    if (horizontalCoverageFromBoresightDegrees < 0 || 360 < horizontalCoverageFromBoresightDegrees) {
        cerr << "Set sensing-horizontal-coverage-degrees value 0 to 360." << endl;
        exit(1);
    }//if//
    if (verticalCoverageFromBoresightDegrees < 0 || 180 < verticalCoverageFromBoresightDegrees) {
        cerr << "Set sensing-vertical-coverage-degrees value 0 to 180." << endl;
        exit(1);
    }//if//

    halfHorizontalCoverageFromBoresightDegrees = horizontalCoverageFromBoresightDegrees*0.5;
    halfVerticalCoverageFromBoresightDegrees = verticalCoverageFromBoresightDegrees*0.5;
}//FanSensingShape//


inline
bool FanSensingShape::IsInTheCoverage(
    const ObjectMobilityPosition& basePosition,
    const ObjectMobilityPosition& targetPosition) const
{
    const Vertex baseVertex =
        MakeVertexFromMobilityPosition(basePosition);

    const Vertex targetVertex =
        MakeVertexFromMobilityPosition(targetPosition);

    // First, check a straight distance to target.
    if (baseVertex.DistanceTo(targetVertex) > coverageDistanceMeters) {
        return false;
    }//if//

    const double platformAzimuthDegrees = basePosition.AttitudeAzimuthFromNorthClockwiseDegrees();
    const double platformElevationDegrees = basePosition.AttitudeElevationFromHorizonDegrees();

    const double boresightAzimuthDegrees =
        NormalizeAngleToNeg180To180(platformAzimuthDegrees + azimuthFromPlatformDegrees);

    const double boresightElevationDegrees =
        NormalizeAngleToNeg180To180(platformElevationDegrees + elevationFromPlatformDegrees);

    const Vertex targetDirectionVector = targetVertex - baseVertex;

    const double horizontalDirectionDegrees =
        NormalizeAngleToNeg180To180(
            90. - (std::atan2(targetDirectionVector.y, targetDirectionVector.x)) * DEGREES_PER_RADIAN);

    const double verticalDirectionDegrees =
        NormalizeAngleToNeg180To180(
            (std::atan2(targetDirectionVector.z, targetDirectionVector.XYDistance())) * DEGREES_PER_RADIAN);

    const double targetFromBoresightAzimuthDegrees =
        std::fabs(NormalizeAngleToNeg180To180(horizontalDirectionDegrees - boresightAzimuthDegrees));

    const double targetFromBoresightElevationDegrees =
        std::fabs(NormalizeAngleToNeg180To180(verticalDirectionDegrees - boresightElevationDegrees));

    // Both horizontal and vertical direction are coverted by the fan range from boresight.
    return ((targetFromBoresightAzimuthDegrees <= halfHorizontalCoverageFromBoresightDegrees) &&
            (targetFromBoresightElevationDegrees <= halfVerticalCoverageFromBoresightDegrees));
}//IsInTheCoverage//


inline
Rectangle FanSensingShape::CalculateMaxCoverageRectangle(
    const ObjectMobilityPosition& basePosition) const
{
    const Vertex baseVertex =
        MakeVertexFromMobilityPosition(basePosition);

    const Vertex coverateAreaOffset(
        coverageDistanceMeters,
        coverageDistanceMeters);

    return Rectangle(
        baseVertex - coverateAreaOffset,
        baseVertex + coverateAreaOffset);

}//CalculateMaxCoverageRectangle//


inline
GisObjectSensingShape::GisObjectSensingShape(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const NodeIdType& initNodeId,
    const InterfaceOrInstanceIdType& initInstanceId)
    :
    coverageHeightMeters(10.)
{
    const string coverageAreaReferenceName =
        parameterDatabaseReader.ReadString("sensing-coverage-area-gis-object-name", initNodeId, initInstanceId);

    vector<GisPositionIdType> foundPositionIds;

    initGisSubsystemPtr->GetPositions(
        coverageAreaReferenceName,
        foundPositionIds);

    for(size_t i = 0; i < foundPositionIds.size(); i++) {
        const GisPositionIdType& positionId = foundPositionIds[i];

        switch (positionId.type) {
        case GIS_BUILDING: {
            const Building& building = initGisSubsystemPtr->GetBuilding(positionId.id);
            coverageGisObjectPolygonFromBases.push_back(building.GetBuildingPolygon());
            break;
        }
        case GIS_AREA: {
            const Area& area = initGisSubsystemPtr->GetArea(positionId.id);
            coverageGisObjectPolygonFromBases.push_back(area.GetPolygon());
            break;
        }
        case GIS_PARK: {
            const Park& park = initGisSubsystemPtr->GetPark(positionId.id);
            coverageGisObjectPolygonFromBases.push_back(park.GetPolygon());
            break;
        }
        case GIS_ROAD: {
            const Road& road = initGisSubsystemPtr->GetRoad(positionId.id);
            coverageGisObjectPolygonFromBases.push_back(road.GetPolygon());
            break;
        }
        default:
            // no coverage area
            break;
        }//switch//

    }//for//

    coverageHeightMeters =
        parameterDatabaseReader.ReadDouble("sensing-coverage-area-height-meters", initNodeId, initInstanceId);

}//GisObjectSensingModel//

inline
void GisObjectSensingShape::SetInitialBasePosition(const ObjectMobilityPosition& basePosition)
{
    const Vertex baseVertex = MakeVertexFromMobilityPosition(basePosition);
    const Vertex offset(baseVertex.x, baseVertex.y, 0.);

    for(size_t i = 0; i < coverageGisObjectPolygonFromBases.size(); i++) {
        vector<Vertex>& coverageGisObjectPolygonFromBase = coverageGisObjectPolygonFromBases[i];

        for(size_t j = 0; j < coverageGisObjectPolygonFromBase.size(); j++) {
            coverageGisObjectPolygonFromBase[j] = coverageGisObjectPolygonFromBase[j] - offset;
        }//for//

    }//for//

}//SetInitialBasePosition//

inline
bool GisObjectSensingShape::IsInTheCoverage(
    const ObjectMobilityPosition& basePosition,
    const ObjectMobilityPosition& targetPosition) const
{
    for(size_t i = 0; i < coverageGisObjectPolygonFromBases.size(); i++) {
          const vector<Vertex>& coverageGisObjectPolygonFromBase = coverageGisObjectPolygonFromBases[i];

          assert(!coverageGisObjectPolygonFromBase.empty());

          const double baseGisObjectZMeters = coverageGisObjectPolygonFromBase.front().z;
          const double targetObjectZMeters = targetPosition.HeightFromGroundMeters();

          if ((targetObjectZMeters < baseGisObjectZMeters) ||
              (baseGisObjectZMeters + coverageHeightMeters < targetObjectZMeters)) {
              continue;
          }//if//

          if (PolygonContainsPoint(
                  (*this).GetCoverageGisObjectPolygon(coverageGisObjectPolygonFromBase, basePosition),
                  MakeVertexFromMobilityPosition(targetPosition))) {

              return true;
          }//if//
    }//for//

    return false;

}//IsInTheCoverage//

inline
vector<Vertex> GisObjectSensingShape::GetCoverageGisObjectPolygon(
    const vector<Vertex>& coverageGisObjectPolygonFromBase,
    const ObjectMobilityPosition& basePosition) const
{
    vector<Vertex> coverageGisObjectPolygon = coverageGisObjectPolygonFromBase;

    const Vertex baseVertex = MakeVertexFromMobilityPosition(basePosition);
    const Vertex offset(baseVertex.x, baseVertex.y, 0.);
    const RotationMatrix rotationMatrix(
        -basePosition.AttitudeAzimuthFromNorthClockwiseDegrees(),
        0.,
        basePosition.AttitudeElevationFromHorizonDegrees());

    for(size_t i = 0; i < coverageGisObjectPolygon.size(); i++) {
        coverageGisObjectPolygon[i] = rotationMatrix*coverageGisObjectPolygon[i] + offset;
    }

    return std::move(coverageGisObjectPolygon);
}//CalculateMaxCoverageRectangle//

//-----------------------------------------------------------------------

// Sensing model

class SensingModel {
public:
    SensingModel() : isOn(true) {}

    virtual ~SensingModel() {}

    virtual void SetInitialBasePosition(const ObjectMobilityPosition& initBasePosition) {}


    // Get all detection information
    virtual bool NeedToRetrieveSensingResult() const { return true; }
    virtual void RetrieveSensingResult(SensingResult& result) = 0;

    // Single check APIs that is easy to use.
    virtual bool TargetNodeIsVisible(const NodeIdType& targetNodeId) { return false; }
    virtual bool TargetGisObjectIsVisible(const GisPositionIdType& positionId) { return false; }


    // default is "on" state
    void TurnOn() { isOn = true; }
    void TurnOff() { isOn = false; }
    bool IsOn() const { return isOn; }



    // Extra data transmission API through SensingSubsystem.
    // Convenient to modeling a simple data exchange or physical adverticements.

    class DataReceiveHandler {
    public:
        DataReceiveHandler() {}
        virtual ~DataReceiveHandler() {}
        virtual void ReceiveData(SensingSharedInfoType sharedInfo/*No use reference variable, transfer deep copied data*/) = 0;
    };

    void SetDataReceiveHandler(
        const shared_ptr<DataReceiveHandler>& initReceiveHandlerPtr) {
        receiveHandlerPtr = initReceiveHandlerPtr;
    }

    void ReceiveData(const SensingSharedInfoType& sharedInfo) {
        if (receiveHandlerPtr != nullptr) {
            receiveHandlerPtr->ReceiveData(sharedInfo);
        }//if//
    }

    virtual void TransmitDataToDetectedCommNodes(const SensingSharedInfoType& sharedInfo) { assert(false && "Transmission is not supported."); }


protected:
    bool isOn;

    shared_ptr<DataReceiveHandler> receiveHandlerPtr;
};//SensingModel//


// Shape Sensing
// Communication/GIS objects will be detected through global information.

class ShapeSensingModel: public SensingModel, public enable_shared_from_this<ShapeSensingModel> {
public:
    ShapeSensingModel(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<SensingSubsystemInterface>& initSensingSubsystemInterfacePtr,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const NodeIdType& initNodeId,
        const InterfaceOrInstanceIdType& initInstanceId,
        const RandomNumberGeneratorSeedType& initNodeSeed);

    ShapeSensingModel(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<SensingSubsystemInterface>& initSensingSubsystemInterfacePtr,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const shared_ptr<SensingShape>& initSensingShapePtr,
        const NodeIdType& initNodeId,
        const InterfaceOrInstanceIdType& initInstanceId,
        const RandomNumberGeneratorSeedType& initNodeSeed);

    virtual void SetInitialBasePosition(const ObjectMobilityPosition& initBasePosition) {
        sensingShapePtr->SetInitialBasePosition(initBasePosition);
    }

    virtual bool NeedToRetrieveSensingResult() const {
        return (simulationEngineInterfacePtr->CurrentTime() > nextTopologyUpdateTime);
    }

    virtual void RetrieveSensingResult(SensingResult& result);

    virtual bool TargetNodeIsVisible(const NodeIdType& targetNodeId);
    virtual bool TargetGisObjectIsVisible(const GisPositionIdType& positionId);

    virtual void TransmitDataToDetectedCommNodes(const SensingSharedInfoType& sharedInfo);

    void SetSensingShape(const shared_ptr<SensingShape>& initSensingShapePtr) { sensingShapePtr = initSensingShapePtr; }

    void SetDetectionGranularityMeters(const double initDetectionGranularityMeters) { detectionGranularityMeters = initDetectionGranularityMeters; }

    void SetDetectionErrorRate(const double initDetectionErrorRate) { detectionErrorRate = initDetectionErrorRate; }

    void SetPositionErrorStandardDeviationMeters(const double initPositionErrorStandardDeviationMeters) { positionErrorStandardDeviationMeters = initPositionErrorStandardDeviationMeters; }

    void SetDetectionCondition(const DetectionConditionType& initDetectionCondition) { detectionCondition = initDetectionCondition; }

    void SetTransmissionConditionType(const TransmissionConditionType& initTransmissionCondition) { transmissionCondition = initTransmissionCondition; }

    void SetTransmissionDataErrorRate(const double initTransmissionDataErrorRate) { transmissionDataErrorRate = initTransmissionDataErrorRate; }

    void SetTargetObjectType(
        const bool initCommunicationObjectIsTarget,
        const list<GisObjectType>& initTargetGisObjectTypes) {
        communicationObjectIsTarget = initCommunicationObjectIsTarget;
        targetGisObjectTypes = initTargetGisObjectTypes;
    }

private:
    void DetectNodes(vector<DetectedNodeResult>& nodeDetectionResults);

    void DetectGisObjects(vector<DetectedGisObjectResult>& gisObjectDetectionResults);

    Rectangle CalculateMaxCoverageRectangle(
        const ObjectMobilityPosition& basePosition) const;

    // Check an object existence with multiple positions
    void DetectTargetObjectWithMultiPoints(
        const NodeIdType& targetObjectId,
        const ObjectMobilityPosition& basePosition,
        const vector<ObjectMobilityPosition>& checkPositions,
        bool& detectedOneOrMorePositions,
        ObjectMobilityPosition& detectedPosition,
        bool& hasLineOfSight);

    // for gis polygon objects
    void GetDetectionCheckPositionsForGisObject(
        const Rectangle& maxConverageRect,
        const ObjectMobilityPosition& basePosition,
        const GisPositionIdType& positionId,
        NodeIdType& objectId,
        vector<ObjectMobilityPosition>& checkPositions) const;

    void GetDetectionCheckPositionsFor3dPolygon(
        const Rectangle& maxConverageRect,
        const Vertex& baseVertex,
        const vector<Vertex>& floorVertices,
        const double heightMeters,
        const GisPositionIdType& positionId,
        const NodeIdType& objectId,
        vector<ObjectMobilityPosition>& checkPositions) const;

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    shared_ptr<SensingSubsystemInterface> sensingSubsystemInterfacePtr;
    shared_ptr<GisSubsystem> theGisSubsystemPtr;

    TimeType nextTopologyUpdateTime;
    NodeIdType nodeId;
    RandomNumberGenerator aRandomNumberGenerator;

    shared_ptr<SensingShape> sensingShapePtr;

    double heightFromPlatformMeters;

    double detectionGranularityMeters;

    double detectionErrorRate;
    double positionErrorStandardDeviationMeters;

    DetectionConditionType detectionCondition;

    TransmissionConditionType transmissionCondition;
    double transmissionDataErrorRate;

    bool communicationObjectIsTarget;
    list<GisObjectType> targetGisObjectTypes;
};//ShapeSensingModel//


//---------------------------

// no ".config" parameter version
inline
ShapeSensingModel::ShapeSensingModel(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<SensingSubsystemInterface>& initSensingSubsystemInterfacePtr,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const NodeIdType& initNodeId,
    const InterfaceOrInstanceIdType& initInstanceId,
    const RandomNumberGeneratorSeedType& initNodeSeed)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    sensingSubsystemInterfacePtr(initSensingSubsystemInterfacePtr),
    theGisSubsystemPtr(initGisSubsystemPtr),
    nextTopologyUpdateTime(ZERO_TIME),
    nodeId(initNodeId),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initNodeSeed, initInstanceId, initNodeId)),
    sensingShapePtr(new FanSensingShape()),
    heightFromPlatformMeters(1.5),
    detectionGranularityMeters(10.),
    detectionErrorRate(0.),
    positionErrorStandardDeviationMeters(0.),
    detectionCondition(DETECTION_CONDITION_LOS_ONLY),
    transmissionCondition(TRANSMISSION_CONDITION_SIMPLEX),
    transmissionDataErrorRate(0.),
    communicationObjectIsTarget(true)//default is true for no ".config" version
{}//ShapeSensingModel//

inline
ShapeSensingModel::ShapeSensingModel(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<SensingSubsystemInterface>& initSensingSubsystemInterfacePtr,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const shared_ptr<SensingShape>& initSensingShapePtr,
    const NodeIdType& initNodeId,
    const InterfaceOrInstanceIdType& initInstanceId,
    const RandomNumberGeneratorSeedType& initNodeSeed)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    sensingSubsystemInterfacePtr(initSensingSubsystemInterfacePtr),
    theGisSubsystemPtr(initGisSubsystemPtr),
    nextTopologyUpdateTime(ZERO_TIME),
    nodeId(initNodeId),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(initNodeSeed, initInstanceId, initNodeId)),
    sensingShapePtr(initSensingShapePtr),
    heightFromPlatformMeters(1.5),
    detectionGranularityMeters(10.),
    detectionErrorRate(0.),
    positionErrorStandardDeviationMeters(0.),
    detectionCondition(DETECTION_CONDITION_LOS_ONLY),
    transmissionCondition(TRANSMISSION_CONDITION_SIMPLEX),
    transmissionDataErrorRate(0.),
    communicationObjectIsTarget(false)// will be initialized by "sensing-detection-target".
{

    if (parameterDatabaseReader.ParameterExists("sensing-height-meters", initNodeId, initInstanceId)) {

        heightFromPlatformMeters =
            parameterDatabaseReader.ReadDouble("sensing-height-meters", initNodeId, initInstanceId);
    }//if//

    detectionGranularityMeters =
        parameterDatabaseReader.ReadDouble("sensing-detection-granularity-meters", initNodeId, initInstanceId);

    detectionErrorRate = parameterDatabaseReader.ReadDouble(
        "sensing-detection-error-rate", initNodeId, initInstanceId);

    positionErrorStandardDeviationMeters = parameterDatabaseReader.ReadDouble(
        "sensing-position-error-standard-deviation-meters", initNodeId, initInstanceId);

    const string detectionConditionString = MakeLowerCaseString(
        parameterDatabaseReader.ReadString(
            "sensing-detection-condition", initNodeId, initInstanceId));

    if (detectionConditionString == "losonly") {
        detectionCondition = DETECTION_CONDITION_LOS_ONLY;
    }
    else if (detectionConditionString == "losandnlos") {
        detectionCondition = DETECTION_CONDITION_LOS_AND_NLOS;
    }
    else {
        cerr << "Set sensing-detection-condition value LosOnly or LosAndNlos." << endl;
        exit(1);
    }//if//


    const string transmissionConditionString = MakeLowerCaseString(
        parameterDatabaseReader.ReadString(
            "sensing-transmission-condition", initNodeId, initInstanceId));

    if (transmissionConditionString == "simplex") {
        transmissionCondition = TRANSMISSION_CONDITION_SIMPLEX;
    }
    else if (transmissionConditionString == "duplex") {
        transmissionCondition = TRANSMISSION_CONDITION_DUPLEX;
    }
    else {
        cerr << "Set sensing-transmission-condition value Simplex or Duplex." << endl;
        exit(1);
    }//if//

    transmissionDataErrorRate =
        parameterDatabaseReader.ReadDouble("sensing-transmission-data-error-rate", initNodeId, initInstanceId);


    const string sensingDetectionTargetString =
        parameterDatabaseReader.ReadString("sensing-detection-target", initNodeId, initInstanceId);

    istringstream inStream(sensingDetectionTargetString);

    while (!inStream.eof()) {
        string objctTypeName;

        inStream >> objctTypeName;
        ConvertStringToLowerCase(objctTypeName);

        if (objctTypeName == "communicationobject") {
            communicationObjectIsTarget = true;
        }
        else if (objctTypeName == GIS_ROAD_STRING) {
            targetGisObjectTypes.push_back(GIS_ROAD);
        }
        else if ((objctTypeName == GIS_RAILROAD_STRING) ||
                 (objctTypeName == "railroad")) {
            targetGisObjectTypes.push_back(GIS_RAILROAD);
        }
        else if (objctTypeName == GIS_AREA_STRING) {
            targetGisObjectTypes.push_back(GIS_AREA);
        }
        else if (objctTypeName == GIS_STATION_STRING) {
            targetGisObjectTypes.push_back(GIS_RAILROAD_STATION);
        }
        else if (objctTypeName == GIS_OLD_TRAFFIC_LIGHT_STRING ||
                 objctTypeName == GIS_TRAFFIC_LIGHT_STRING) {
            targetGisObjectTypes.push_back(GIS_TRAFFICLIGHT);
        }
        else if (objctTypeName == GIS_BUSSTOP_STRING) {
            targetGisObjectTypes.push_back(GIS_BUSSTOP);
        }
        else if (objctTypeName == GIS_ENTRANCE_STRING) {
            targetGisObjectTypes.push_back(GIS_ENTRANCE);
        }
        else if (objctTypeName == GIS_BUILDING_STRING) {
            targetGisObjectTypes.push_back(GIS_BUILDING);
        }
        else if (objctTypeName == GIS_PARK_STRING) {
            targetGisObjectTypes.push_back(GIS_PARK);
        }
        else if (objctTypeName == GIS_WALL_STRING) {
            targetGisObjectTypes.push_back(GIS_WALL);
        }
        else if (objctTypeName == GIS_ROAD_INTERSECTION_STRING) {
            targetGisObjectTypes.push_back(GIS_INTERSECTION);
        }
        else if (objctTypeName == GIS_POI_STRING) {
            targetGisObjectTypes.push_back(GIS_POI);
        }
        else {
            cerr << "Error: Invalid object: " << objctTypeName << endl;
            exit(1);
        }//if//
    }//while//

    if (detectionGranularityMeters <= 0) {
        cerr << "Set sensing-detection-granularity-meters value larger than 0." << endl;
        exit(1);
    }//if//
    if (detectionErrorRate < 0 || 1 < detectionErrorRate) {
        cerr << "Set sensing-detection-error-rate value 0 to 1." << endl;
        exit(1);
    }//if//
    if (positionErrorStandardDeviationMeters < 0) {
        cerr << "Set sensing-position-error-standard-deviation-meters value larger than 0." << endl;
        exit(1);
    }//if//

}//ShapeSensingModel//

inline
void ShapeSensingModel::RetrieveSensingResult(SensingResult& result)
{

    (*this).DetectNodes(result.nodeDetectionResults);

    (*this).DetectGisObjects(result.gisObjectDetectionResults);

}//RetrieveSensingResult//

inline
void ShapeSensingModel::DetectNodes(vector<DetectedNodeResult>& nodeDetectionResults)
{
    const SensingGlobalInfo& theSensingGlobalInfo = sensingSubsystemInterfacePtr->GetSensingGlobalInfo();

    nextTopologyUpdateTime = theSensingGlobalInfo.nextTopologyUpdateTime;
    nodeDetectionResults.clear();

    if (!isOn) {
        return;
    }//if//

    if (!communicationObjectIsTarget) {
        return;
    }//if//

    const map<NodeIdType, ObjectMobilityPosition>& mobilityPositionCache =
        theSensingGlobalInfo.mobilityPositionCache;

    bool success;
    ObjectMobilityPosition basePosition;

    MakeSensingPositionFromMobilityCache(
        mobilityPositionCache, nodeId, heightFromPlatformMeters, success, basePosition);

    if (!success) {
        return;
    }//if//

    typedef map<NodeIdType, ObjectMobilityPosition>::const_iterator IterType;

    for(IterType iter = mobilityPositionCache.begin();
        (iter != mobilityPositionCache.end()); iter++) {

        const NodeIdType& targetNodeId = (*iter).first;

        if (nodeId == targetNodeId) {
            continue;
        }//if//

        const vector<ObjectMobilityPosition> checkPositions(1, (*iter).second);

        bool detectedAPosition;
        ObjectMobilityPosition detectedPosition;
        bool hasLineOfSight;

        (*this).DetectTargetObjectWithMultiPoints(
            targetNodeId,
            basePosition,
            checkPositions,
            detectedAPosition,
            detectedPosition,
            hasLineOfSight);

        if (detectedAPosition) {
            nodeDetectionResults.push_back(
                DetectedNodeResult(
                    targetNodeId, detectedPosition, hasLineOfSight));
        }//if//
    }//for//

}//DetectNodes//

inline
bool ShapeSensingModel::TargetNodeIsVisible(const NodeIdType& targetNodeId)
{
    if (!isOn) {
        return false;
    }//if//

    if (!communicationObjectIsTarget) {
        return false;
    }//if//

    const map<NodeIdType, ObjectMobilityPosition>& mobilityPositionCache =
        sensingSubsystemInterfacePtr->GetSensingGlobalInfo().mobilityPositionCache;

    bool success;
    ObjectMobilityPosition basePosition;

    MakeSensingPositionFromMobilityCache(
        mobilityPositionCache, nodeId, heightFromPlatformMeters, success, basePosition);

    if (!success) {
        return false;
    }//if//

    typedef map<NodeIdType, ObjectMobilityPosition>::const_iterator IterType;

    IterType iter = mobilityPositionCache.find(targetNodeId);

    if (iter != mobilityPositionCache.end()) {
        const vector<ObjectMobilityPosition> checkPositions(1, (*iter).second);

        bool detectedAPosition;
        ObjectMobilityPosition detectedPosition;
        bool hasLineOfSight;

        (*this).DetectTargetObjectWithMultiPoints(
            targetNodeId,
            basePosition,
            checkPositions,
            detectedAPosition,
            detectedPosition,
            hasLineOfSight);

        return (detectedAPosition);
    }//for//

    return false;
}//TargetNodeIsVisible//

inline
void ShapeSensingModel::DetectGisObjects(vector<DetectedGisObjectResult>& gisObjectDetectionResults)
{
    const SensingGlobalInfo& theSensingGlobalInfo = sensingSubsystemInterfacePtr->GetSensingGlobalInfo();

    nextTopologyUpdateTime = theSensingGlobalInfo.nextTopologyUpdateTime;
    gisObjectDetectionResults.clear();

    if (!isOn) {
        return;
    }//if//

    const map<NodeIdType, ObjectMobilityPosition>& mobilityPositionCache =
        theSensingGlobalInfo.mobilityPositionCache;

    bool success;
    ObjectMobilityPosition basePosition;

    MakeSensingPositionFromMobilityCache(
        mobilityPositionCache, nodeId, heightFromPlatformMeters, success, basePosition);

    if (!success) {
        return;
    }//if//

    const Rectangle maxConverageRect =
        sensingShapePtr->CalculateMaxCoverageRectangle(basePosition);

    list<GisPositionIdType> positionIds;

    theGisSubsystemPtr->GetGisObjectIds(
        maxConverageRect,
        targetGisObjectTypes,
        positionIds);

    typedef list<GisPositionIdType>::const_iterator IterType;

    for(IterType iter = positionIds.begin(); (iter != positionIds.end()); iter++) {
        const GisPositionIdType& positionId = (*iter);

        NodeIdType targetObjectId;
        vector<ObjectMobilityPosition> checkPositions;

        (*this).GetDetectionCheckPositionsForGisObject(
            maxConverageRect,
            basePosition,
            positionId,
            targetObjectId,
            checkPositions);

        bool detectedOneOrMorePositions;
        ObjectMobilityPosition detectedPosition;
        bool hasLineOfSight;

        (*this).DetectTargetObjectWithMultiPoints(
            targetObjectId,
            basePosition,
            checkPositions,
            detectedOneOrMorePositions,
            detectedPosition,
            hasLineOfSight);

        if (detectedOneOrMorePositions) {
            gisObjectDetectionResults.push_back(
                DetectedGisObjectResult(
                    positionId, detectedPosition, hasLineOfSight));
        }//if//
    }//for//
}//DetectGisObjects//

inline
bool ShapeSensingModel::TargetGisObjectIsVisible(const GisPositionIdType& positionId)
{
    if (!isOn) {
        return false;
    }//if//

    if (std::find(targetGisObjectTypes.begin(), targetGisObjectTypes.end(), positionId.type) == targetGisObjectTypes.end()) {
        return false;
    }//if//

    const map<NodeIdType, ObjectMobilityPosition>& mobilityPositionCache =
        sensingSubsystemInterfacePtr->GetSensingGlobalInfo().mobilityPositionCache;

    bool success;
    ObjectMobilityPosition basePosition;

    MakeSensingPositionFromMobilityCache(
        mobilityPositionCache, nodeId, heightFromPlatformMeters, success, basePosition);

    if (!success) {
        return false;
    }//if//

    NodeIdType targetObjectId;
    vector<ObjectMobilityPosition> checkPositions;

    (*this).GetDetectionCheckPositionsForGisObject(
        sensingShapePtr->CalculateMaxCoverageRectangle(basePosition),
        basePosition,
        positionId,
        targetObjectId,
        checkPositions);

    bool detectedOneOrMorePositions;
    ObjectMobilityPosition detectedPosition;
    bool hasLineOfSight;

    (*this).DetectTargetObjectWithMultiPoints(
        targetObjectId,
        basePosition,
        checkPositions,
        detectedOneOrMorePositions,
        detectedPosition,
        hasLineOfSight);

    return (detectedOneOrMorePositions);

}//TargetGisObjectIsVisible//

inline
void ShapeSensingModel::DetectTargetObjectWithMultiPoints(
    const NodeIdType& targetObjectId,
    const ObjectMobilityPosition& basePosition,
    const vector<ObjectMobilityPosition>& checkPositions,
    bool& detectedOneOrMorePositions,
    ObjectMobilityPosition& detectedPosition,
    bool& hasLineOfSight)
{
    detectedOneOrMorePositions = false;
    hasLineOfSight = false;

    const Vertex baseVertex = MakeVertexFromMobilityPosition(basePosition);

    vector<pair<Vertex, ObjectMobilityPosition> > coverageVertexAndOffsetPositions;

    // First, pick positions in sensing range.
    for(size_t i = 0; i < checkPositions.size(); i++) {
        const ObjectMobilityPosition& checkPosition = checkPositions[i];
        const Vertex checkVertex = MakeVertexFromMobilityPosition(checkPosition);

        const ObjectMobilityPosition offsetMobilityPosition =
            CalculatePositionWithError(
                checkPosition,
                positionErrorStandardDeviationMeters,
                aRandomNumberGenerator);

        // use (error)offset position for cheking.
        if (sensingShapePtr->IsInTheCoverage(basePosition, offsetMobilityPosition)) {
            coverageVertexAndOffsetPositions.push_back(
                make_pair(checkVertex, offsetMobilityPosition));
        }//if//
    }//for//

    if (!coverageVertexAndOffsetPositions.empty()) {
        const double aRandomnumber = aRandomNumberGenerator.GenerateRandomDouble();

        // Second, random error check
        if (aRandomnumber >= detectionErrorRate) {

            if (detectionCondition == DETECTION_CONDITION_LOS_ONLY) {

                // Search a line of sight path to original vertex.
                const BuildingLayer& buildingLayer = (*theGisSubsystemPtr->GetBuildingLayerPtr());

                set<NodeIdType> losIgnoringNodeIds;
                losIgnoringNodeIds.insert(nodeId);
                losIgnoringNodeIds.insert(targetObjectId);

                for(size_t i = 0; i < coverageVertexAndOffsetPositions.size(); i++) {
                    const Vertex& vertex = coverageVertexAndOffsetPositions[i].first;

                    if (buildingLayer.PositionsAreLineOfSight(baseVertex, vertex, losIgnoringNodeIds)) {
                        hasLineOfSight = true;
                        break;
                    }//if//
                }//for//

                if (!hasLineOfSight) {
                    return;
                }//if//

            }//if//

            // Search nearest position with error correction position
            double minDistanceToPosition = DBL_MAX;
            ObjectMobilityPosition nearestPosition;

            for(size_t i = 0; i < coverageVertexAndOffsetPositions.size(); i++) {
                const ObjectMobilityPosition& position = coverageVertexAndOffsetPositions[i].second;
                const double distance =
                    baseVertex.DistanceTo(MakeVertexFromMobilityPosition(position));

                if (distance < minDistanceToPosition) {
                    nearestPosition = position;
                    minDistanceToPosition = distance;
                }//if//
            }//for//

            detectedOneOrMorePositions = true;
            detectedPosition = nearestPosition;
        }//if//
    }//if//

}//DetectTargetObjectWithMultiPoints//


inline
void ShapeSensingModel::GetDetectionCheckPositionsForGisObject(
    const Rectangle& maxConverageRect,
    const ObjectMobilityPosition& basePosition,
    const GisPositionIdType& positionId,
    NodeIdType& objectId,
    vector<ObjectMobilityPosition>& checkPositions) const
{
    objectId = INVALID_NODEID;

    vector<Vertex> floorVertices;
    double heightMeters = 0;

    // Pick structure vertices

    switch (positionId.type) {
    case GIS_ROAD: {
        const Road& road = theGisSubsystemPtr->GetRoad(positionId.id);

        floorVertices = road.GetPolygon();
        objectId = road.GetObjectId();
        break;
    }
    case GIS_RAILROAD: {
        const RailRoad& railroad = theGisSubsystemPtr->GetRailRoad(positionId.id);

        floorVertices = railroad.GetVertices();
        objectId = railroad.GetObjectId();
        break;
    }
    case GIS_AREA: {
        const Area& area = theGisSubsystemPtr->GetArea(positionId.id);

        floorVertices = area.GetPolygon();
        objectId = area.GetObjectId();
        break;
    }
    case GIS_RAILROAD_STATION: {
        const RailRoadStation& station = theGisSubsystemPtr->GetStation(positionId.id);

        floorVertices.push_back(station.GetVertex());
        objectId = station.GetObjectId();
        break;
    }
    case GIS_TRAFFICLIGHT: {
        const TrafficLight& trafficLight = theGisSubsystemPtr->GetTrafficLight(positionId.id);

        floorVertices.push_back(trafficLight.GetVertex(0));
        objectId = trafficLight.GetObjectId();
        break;
    }
    case GIS_BUSSTOP: {
        const BusStop& busStop = theGisSubsystemPtr->GetBusStop(positionId.id);

        floorVertices.push_back(busStop.GetVertex());
        objectId = busStop.GetObjectId();
        break;
    }
    case GIS_ENTRANCE: {
        const Entrance& entrance = theGisSubsystemPtr->GetEntrance(positionId.id);

        floorVertices.push_back(entrance.GetVertex());
        objectId = entrance.GetObjectId();
        break;
    }
    case GIS_BUILDING: {
        const Building& building = theGisSubsystemPtr->GetBuilding(positionId.id);

        floorVertices = building.GetBuildingPolygon();
        heightMeters = building.GetHeightMeters();
        objectId = building.GetObjectId();
        break;
    }
    case GIS_PARK: {
        const Park& park = theGisSubsystemPtr->GetPark(positionId.id);

        floorVertices = park.GetPolygon();
        objectId = park.GetObjectId();
        break;
    }
    case GIS_WALL: {
        const Wall& wall = theGisSubsystemPtr->GetWall(positionId.id);

        floorVertices = wall.GetWallVertices();
        heightMeters = wall.GetHeightMeters();
        objectId = wall.GetObjectId();
        break;
    }
    case GIS_INTERSECTION: {
        const Intersection& intersection = theGisSubsystemPtr->GetIntersection(positionId.id);

        floorVertices.push_back(intersection.GetVertex());
        objectId = intersection.GetObjectId();
        break;
    }
    case GIS_POI: {
        const Poi& poi = theGisSubsystemPtr->GetPoi(positionId.id);

        floorVertices.push_back(poi.GetVertex());
        objectId = poi.GetObjectId();
        break;
    }
    default:
        break;
    }//switch//

    const Vertex baseVertex = MakeVertexFromMobilityPosition(basePosition);

    (*this).GetDetectionCheckPositionsFor3dPolygon(maxConverageRect, baseVertex, floorVertices, heightMeters, positionId, objectId, checkPositions);

}//GetDetectionCheckPositionsForGisObject//

inline
void ShapeSensingModel::GetDetectionCheckPositionsFor3dPolygon(
    const Rectangle& maxConverageRect,
    const Vertex& baseVertex,
    const vector<Vertex>& floorVertices,
    const double heightMeters,
    const GisPositionIdType& positionId,
    const NodeIdType& objectId,
    vector<ObjectMobilityPosition>& checkPositions) const
{
    if (floorVertices.empty()) {
        return;
    }//if//

    vector<Vertex> checkVertices;

    const size_t numberVerticalDivisions = static_cast<size_t>(ceil(heightMeters / detectionGranularityMeters));

    if (numberVerticalDivisions > 0) {
        const Vertex hv(0., 0., heightMeters);

        // Divide polygon planes to meshes due to sensing accuracy.

        for(size_t i = 0; i < floorVertices.size() - 1; i++) {
            const Vertex& v1 = floorVertices[i];
            const Vertex& v2 = floorVertices[i+1];
            const Vertex nearestPosition =
                CalculatePointToRectNearestVertex(baseVertex, v1, v2, v2 + hv, v1 + hv);

            if (!maxConverageRect.Contains(nearestPosition)) {
                continue;
            }//if//

            // Push straight nearest position
            checkVertices.push_back(nearestPosition);

            const Vertex directionVector = v2 - v1;
            const double distance = v1.DistanceTo(v2);
            const size_t numberHorizontalDivisions = static_cast<size_t>(ceil(distance / detectionGranularityMeters));

            vector<Vertex> hVertices;

            for(size_t h = 0; h < numberHorizontalDivisions; h++) {
                const double hRate = double(h) / numberHorizontalDivisions;
                hVertices.push_back(v1 + directionVector*hRate);
            }//for//

            // push last one line points
            if (positionId.type == GIS_WALL &&  (i == floorVertices.size() - 2)) {
                hVertices.push_back(v2);
            }//if//

            for(size_t j = 0; j < hVertices.size(); j++) {
                const Vertex& hVertex = hVertices[j];

                for(size_t v = 0; v <= numberVerticalDivisions; v++) {
                    const double vRate = double(v) / numberVerticalDivisions;
                    const Vertex vertex = hVertex + Vertex(0., 0., heightMeters*vRate);

                    // For more optimization, reduce positions by checking a LoS with their own wall.
                    checkVertices.push_back(vertex);
                }//for//
            }//for//

        }//for//

        // push additional roof-top or floor points

        if (heightMeters > 0 &&
            floorVertices.front() == floorVertices.back()) {

            const double floorZ = floorVertices.front().z;
            const Rectangle rect = GetPointsRect(floorVertices);

            if (baseVertex.z > floorZ + heightMeters) {
                vector<Vertex> roofTopVertices = floorVertices;

                for(size_t i = 0; i < roofTopVertices.size(); i++) {
                    roofTopVertices[i].z += heightMeters;
                }//for//

                checkVertices.push_back(
                    CalculatePointToPolygonNearestVertex(baseVertex, roofTopVertices));
            }
            else if (baseVertex.z < floorZ) {
                checkVertices.push_back(
                    CalculatePointToPolygonNearestVertex(baseVertex, floorVertices));
            }//if//
        }//if//

    }
    else {

        for(size_t i = 0; i < floorVertices.size() - 1; i++) {
            const Vertex& v1 = floorVertices[i];
            const Vertex& v2 = floorVertices[i+1];
            const Vertex directionVector = v2 - v1;
            const double distance = v1.DistanceTo(v2);
            const size_t numberHorizontalDivisions = static_cast<size_t>(ceil(distance / detectionGranularityMeters));

            for(size_t h = 0; h < numberHorizontalDivisions; h++) {
                const double hRate = double(h) / numberHorizontalDivisions;
                checkVertices.push_back(v1 + directionVector*hRate);
            }//for//
        }//for//

        assert(!floorVertices.empty());
        checkVertices.push_back(floorVertices.back());
    }//if//


    // pick coverage positions

    for(size_t i = 0; i < checkVertices.size(); i++) {
        const Vertex& vertex = checkVertices[i];

        if (maxConverageRect.Contains(vertex)) {
            checkPositions.push_back(
                MakeStationaryMobilityPositionFromVertex(vertex, objectId));
        }//if//
    }//for//

}//GetDetectionCheckPositionsFor3dPolygon//


//------------------------------------------------------------------



inline
SensingApplication::SensingApplication(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<SensingModel>& initSensingModelPtr,
    const NodeIdType& initNodeId,
    const InterfaceOrInstanceIdType& initInstanceId)
    :
    Application(initSimulationEngineInterfacePtr, initInstanceId),
    sensingModelPtr(initSensingModelPtr),
    sensingStartTime(INFINITE_TIME),
    sensingEndTime(INFINITE_TIME),
    sensingInterval(1 * SECOND),
    nodeDetectionStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_DetectedCommObjects"))),
    gisObjectDetectionStatPtr(
        simulationEngineInterfacePtr->CreateRealStat(
            (modelName + "_DetectedGisObjects")))
{
    sensingStartTime =
        parameterDatabaseReader.ReadTime("sensing-start-time", initNodeId, initInstanceId);

    sensingEndTime =
        parameterDatabaseReader.ReadTime("sensing-end-time", initNodeId, initInstanceId);

    sensingInterval =
        parameterDatabaseReader.ReadTime("sensing-interval", initNodeId, initInstanceId);

    if (sensingInterval <= ZERO_TIME) {
        cerr << "Set sensing-interval value larger than 0." << endl;
        exit(1);
    }//if//

}//SensingApplication//


inline
void SensingApplication::CompleteInitialization()
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    ObjectMobilityPosition basePosition;
    nodeMobilityModelPtr->GetPositionForTime(currentTime, basePosition);

    sensingModelPtr->SetInitialBasePosition(basePosition);

    if (sensingStartTime < INFINITE_TIME) {
        const TimeType nextTime = std::max(currentTime, sensingStartTime);

        const TimeType nextSensingTime = size_t(ceil(double(nextTime) / sensingInterval)) * sensingInterval;

        if (nextSensingTime < sensingEndTime) {
            simulationEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(
                    new SensingObjectsEvent(shared_from_this())),
                nextSensingTime);
        }//if//

    }//if//
}//CompleteInitialization//


inline
void SensingApplication::UpdateSensingObjectResults()
{
    if (DEBUG_SENSING_DETECTION) {
        cout << "Update Node "  << simulationEngineInterfacePtr->GetNodeId() << " Detection Information" << endl;
    }//if//

    (*this).UpdateSensingResultsIfNecessary();

    (*this).OutputTraceAndStatsForObjectDetection();

    if (DEBUG_SENSING_DETECTION) {
        const vector<DetectedNodeResult>& nodeDetectionResults = sensingResult.nodeDetectionResults;
        const vector<DetectedGisObjectResult>& gisObjectDetectionResults = sensingResult.gisObjectDetectionResults;
        const size_t numberNodeDetections = (*this).GetNumberOfDetectedNodes();
        const size_t numberGisObjectDetections = (*this).GetNumberOfDetectedGisObjects();

        cout << "Node " << simulationEngineInterfacePtr->GetNodeId()
             << " DetectionResult, comm=" << numberNodeDetections << "(" << nodeDetectionResults.size() - numberNodeDetections << ")"
             << ", gis=" << numberGisObjectDetections << "(" << gisObjectDetectionResults.size() - numberGisObjectDetections << ")" << endl;

        for(size_t i = 0; i < nodeDetectionResults.size(); i++) {
            const DetectedNodeResult& nodeDetectionResult = nodeDetectionResults[i];
            const ObjectMobilityPosition& position = nodeDetectionResult.position;

            cout << "  nodeid=" << nodeDetectionResult.nodeId << ", los=";
            if (nodeDetectionResult.hasLineOfSight) {
                cout << "yes";
            }
            else {
                cout << "no";
            }//if//
            cout << ", x=" << position.X_PositionMeters()
                 << ", y=" << position.Y_PositionMeters()
                 << ", z=" << position.HeightFromGroundMeters() << endl;
        }//for//

        for(size_t i = 0; i < gisObjectDetectionResults.size(); i++) {
            const DetectedGisObjectResult& gisObjectDetectionResult = gisObjectDetectionResults[i];
            const GisPositionIdType& positionId = gisObjectDetectionResult.positionId;
            const ObjectMobilityPosition& position = gisObjectDetectionResult.position;

            cout << "  ";

            if (positionId.type == GIS_BUILDING) {
                cout << "building";
            }
            else if (positionId.type == GIS_WALL) {
                cout << "wall";
            }
            else {
                cout << "unknowngis";
            }//if//

            cout << "id=" << positionId.id << ", los=";

            if (gisObjectDetectionResult.hasLineOfSight) {
                cout << "yes";
            }
            else {
                cout << "no";
            }//if//

            cout << ", x=" << position.X_PositionMeters()
                 << ", y=" << position.Y_PositionMeters()
                 << ", z=" << position.HeightFromGroundMeters() << endl;
        }//for//

        cout << endl;
    }//if//

    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();
    const TimeType nextSensingTime = currentTime + sensingInterval;

    if (nextSensingTime < sensingEndTime) {
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(
                new SensingObjectsEvent(shared_from_this())),
            currentTime + sensingInterval);
    }//if//

}//UpdateSensingObjectResults//


inline
void SensingApplication::UpdateSensingResultsIfNecessary()
{
    if (sensingModelPtr->NeedToRetrieveSensingResult()) {
        sensingModelPtr->RetrieveSensingResult(sensingResult);

        // Cache update functions
        (*this).CacheDetectedNodeResult();
        (*this).CacheDetectedGisObjectResult();
    }//if//
}//UpdateSensingResultsIfNecessary//

inline
bool SensingApplication::DetectedAnObject()
{
    if ((*this).DetectedANode()) {
        return true;
    }//if//

    return (*this).DetectedAGisObject();
}//DetectedAnObject//

inline
bool SensingApplication::DetectedANode()
{
    (*this).UpdateSensingResultsIfNecessary();

    return !detectedNodeIdCache.empty();
}//DetectedANode//

inline
bool SensingApplication::DetectedAGisObject()
{
    (*this).UpdateSensingResultsIfNecessary();

    return !detectedGisPositionIdCache.empty();
}//DetectedAGisObject//

inline
size_t SensingApplication::GetNumberOfDetectedAllObjects()
{
    return ((*this).GetNumberOfDetectedNodes() +
            (*this).GetNumberOfDetectedGisObjects());
}//GetNumberOfDetectedAllObjects//

inline
size_t SensingApplication::GetNumberOfDetectedNodes()
{
    (*this).UpdateSensingResultsIfNecessary();

    return detectedNodeIdCache.size();
}//GetNumberOfDetectedNodes//

inline
size_t SensingApplication::GetNumberOfDetectedGisObjects()
{
    (*this).UpdateSensingResultsIfNecessary();

    return detectedGisPositionIdCache.size();
}//GetNumberOfDetectedGisObjects//

inline
bool SensingApplication::TargetNodeIsDetected(const NodeIdType& nodeId)
{
    (*this).UpdateSensingResultsIfNecessary();

    return (detectedNodeIdCache.find(nodeId) != detectedNodeIdCache.end());
}//TargetNodeIsDetected//

inline
bool SensingApplication::TargetGisObjectIsDetected (const GisPositionIdType& positionId)
{
    (*this).UpdateSensingResultsIfNecessary();

    return (detectedGisPositionIdCache.find(positionId) != detectedGisPositionIdCache.end());
}//TargetGisObjectIsDetected //

inline
void SensingApplication::CacheDetectedNodeResult()
{
    detectedNodeIdCache.clear();

    const vector<DetectedNodeResult>& nodeDetectionResults = sensingResult.nodeDetectionResults;

    for(size_t i = 0; i < nodeDetectionResults.size(); i++) {
        const DetectedNodeResult& detectionResult = nodeDetectionResults[i];

        detectedNodeIdCache.insert(detectionResult.nodeId);
    }//for//

}//CacheDetectedNodeResult//

inline
void SensingApplication::CacheDetectedGisObjectResult()
{
    detectedGisPositionIdCache.clear();

    const vector<DetectedGisObjectResult>& gisObjectDetectionResults = sensingResult.gisObjectDetectionResults;

    for(size_t i = 0; i < gisObjectDetectionResults.size(); i++) {
        const DetectedGisObjectResult& detectionResult = gisObjectDetectionResults[i];

        detectedGisPositionIdCache.insert(detectionResult.positionId);
    }//for//

}//CacheDetectedGisObjectResult//

inline
const vector<DetectedNodeResult>& SensingApplication::GetCurrentDetectedNodeResults()
{
    return sensingResult.nodeDetectionResults;
}//GetCurrentDetectedNodeResults//

inline
const vector<DetectedGisObjectResult>& SensingApplication::GetCurrentDetectedGisObjectResults()
{
    return sensingResult.gisObjectDetectionResults;
}//GetCurrentDetectedNodeResults//

inline
void SensingApplication::OutputTraceAndStatsForObjectDetection()
{
    const size_t numberNodeDetections = (*this).GetNumberOfDetectedNodes();
    const size_t numberGisObjectDetections = (*this).GetNumberOfDetectedGisObjects();

    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            SensingDetectionTraceRecord traceData;

            traceData.numberNodeDetections = static_cast<uint32_t>(numberNodeDetections);
            traceData.numberGisObjectDetections = static_cast<uint32_t>(numberGisObjectDetections);

            assert(sizeof(traceData) == SENSING_DETECTION_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                modelName, applicationId, "SensingDetection", traceData);
        }
        else {
            ostringstream outStream;

            outStream << "NodeCount= " << numberNodeDetections
                      << " GisCount= " << numberGisObjectDetections;

            simulationEngineInterfacePtr->OutputTrace(
                modelName, applicationId, "SensingDetection", outStream.str());
        }//if//
    }//if//

    // Stats
    nodeDetectionStatPtr->RecordStatValue(static_cast<double>(numberNodeDetections));
    gisObjectDetectionStatPtr->RecordStatValue(static_cast<double>(numberGisObjectDetections));

}//OutputTraceAndStatsForObjectDetection//


//-----------------------------------------------------------------------------


// NetworkSimulator interface only for SensingSubsystem

class NetworkSimulatorInterfaceForSensingSubsystem {
public:
    NetworkSimulatorInterfaceForSensingSubsystem(
        NetworkSimulator* initNetworkSimulatorPtr)
        :
        networkSimulatorPtr(initNetworkSimulatorPtr)
    {}

    ObjectMobilityPosition GetNodePosition(const NodeIdType& nodeId) const;
    TimeType GetTimeStepEventSynchronizationStep() const;

private:
    NetworkSimulator* networkSimulatorPtr;
};//NetworkSimulatorInterfaceForSensingSubsystem//



// SensingSubsystem (cantains a propagation model for Sensing)

class SensingSubsystem {
public:
    SensingSubsystem(
        const shared_ptr<NetworkSimulatorInterfaceForSensingSubsystem>& initNetworkSimulatorInterfacePtr,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const RandomNumberGeneratorSeedType& initRunSeed)
        :
        networkSimulatorInterfacePtr(initNetworkSimulatorInterfacePtr),
        theGisSubsystemPtr(initGisSubsystemPtr),
        theSensingGlobalInfoPtr(new SensingGlobalInfo()),
        aRandomNumberGenerator(HashInputsToMakeSeed(initRunSeed, SEED_HASH))
    {}

    shared_ptr<SensingSubsystemInterface> CreateSubsystemInterfacePtr();

    void ExecuteTimestepBasedEvent(const TimeType& time);
    void AddNode(const NodeIdType& nodeId) { nodeIds.push_back(nodeId); }
    void RemoveNode(const NodeIdType& nodeId);

private:
    friend class SensingSubsystemInterface;

    struct SensingInfo {
        shared_ptr<SensingModel> sensingModelPtr;
        NodeIdType nodeId;
        InterfaceOrInstanceIdType instanceId;

        SensingInfo(
            const shared_ptr<SensingModel>& initSensingModelPtr,
            const NodeIdType& initNodeId,
            const InterfaceOrInstanceIdType& initInstanceId)
            :
            sensingModelPtr(initSensingModelPtr),
            nodeId(initNodeId),
            instanceId(initInstanceId)
        {}
    };//SensingInfo//

    struct SharingInfo {
        NodeIdType transmitterNodeId;
        shared_ptr<SensingModel> sensingPtr;
        SensingSharedInfoType sharedInfo;
        TransmissionConditionType transmissionCondition;
        double errorRate;

        SharingInfo(
            const NodeIdType& initTransmitterNodeId,
            const shared_ptr<SensingModel>& initSensingPtr,
            const SensingSharedInfoType& initBroadcastData,
            const TransmissionConditionType& initTransmissionCondition,
            const double initErrorRate)
            :
            transmitterNodeId(initTransmitterNodeId),
            sensingPtr(initSensingPtr),
            sharedInfo(initBroadcastData),
            transmissionCondition(initTransmissionCondition),
            errorRate(initErrorRate)
        {}
    };//SharingInfo//

    struct SensingPartition {
        list<SensingInfo> sensingInfos;
        list<SharingInfo> transmissionDataList;
    };//SensingPartition//

    vector<SensingPartition> partitions;
    list<NodeIdType> nodeIds;

    shared_ptr<NetworkSimulatorInterfaceForSensingSubsystem> networkSimulatorInterfacePtr;
    shared_ptr<GisSubsystem> theGisSubsystemPtr;
    shared_ptr<SensingGlobalInfo> theSensingGlobalInfoPtr;

    static const int SEED_HASH = 77465273;
    RandomNumberGenerator aRandomNumberGenerator;

    // Call from SensingSubsystemInterface
    shared_ptr<SensingModel> CreateSensingModel(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<SensingSubsystemInterface>& initSensingSubsystemInterfacePtr,
        const NodeIdType& initNodeId,
        const InterfaceOrInstanceIdType& initInstanceId,
        const RandomNumberGeneratorSeedType& initNodeSeed);

    shared_ptr<ShapeSensingModel> CreateShapeSensingModel(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<SensingSubsystemInterface>& initSensingSubsystemInterfacePtr,
        const NodeIdType& initNodeId,
        const InterfaceOrInstanceIdType& initInstanceId,
        const RandomNumberGeneratorSeedType& initNodeSeed);

    void AddSensingModelToLocalPartition(
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SensingModel>& sensingModelPtr,
        const NodeIdType& nodeId,
        const InterfaceOrInstanceIdType& instanceId);

    void TransmitDataIdeally(
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<SensingModel>& transmitterSensingPtr,
        const SensingSharedInfoType& sharedInfo,
        const TransmissionConditionType& transmissionCondition,
        const double errorRate);

    bool NeedToUpdateSensingInfo() const;

    void UpdateMobilityPositionCache();

    void DeliveryAllSharingInfo();

    void DeliveryOneSharingInfo(const SharingInfo& transmissionInfo);

};//SensingSubsystem//

inline
void SensingSubsystem::AddSensingModelToLocalPartition(
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<SensingModel>& sensingModelPtr,
    const NodeIdType& nodeId,
    const InterfaceOrInstanceIdType& instanceId)
{
    // expand thread partition
    const unsigned int partitionIndex =
        simulationEngineInterfacePtr->CurrentThreadPartitionIndex();

    while (partitions.size() <= partitionIndex) {
        partitions.push_back(SensingPartition());
    }//while//

    partitions[partitionIndex].sensingInfos.push_back(
        SensingInfo(sensingModelPtr, nodeId, instanceId));

}//AddSensingModelToLocalPartition//

inline
void SensingSubsystem::TransmitDataIdeally(
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<SensingModel>& transmitterSensingPtr,
    const SensingSharedInfoType& sharedInfo,
    const TransmissionConditionType& transmissionCondition,
    const double errorRate)
{
    // expand thread partition
    const unsigned int partitionIndex =
        simulationEngineInterfacePtr->CurrentThreadPartitionIndex();

    assert(partitionIndex < partitions.size());

    partitions[partitionIndex].transmissionDataList.push_back(
        SharingInfo(
            simulationEngineInterfacePtr->GetNodeId(),
            transmitterSensingPtr,
            sharedInfo,
            transmissionCondition,
            errorRate));

}//TransmitDataIdeally//

inline
void SensingSubsystem::DeliveryAllSharingInfo()
{
    typedef list<SharingInfo>::const_iterator IterType;

    for(size_t i = 0; i < partitions.size(); i++) {
        list<SharingInfo>& transmissionDataList = partitions[i].transmissionDataList;

        for(IterType iter = transmissionDataList.begin();
            (iter != transmissionDataList.end()); iter++) {

            (*this).DeliveryOneSharingInfo(*iter);
        }//for//

        // clear transmitted data
        transmissionDataList.clear();

    }//for//

}//DeliveryAllSharingInfo//

inline
void SensingSubsystem::DeliveryOneSharingInfo(const SharingInfo& transmissionInfo)
{
    typedef list<SensingInfo>::const_iterator IterType;

    const NodeIdType transmitterNodeId = transmissionInfo.transmitterNodeId;
    const shared_ptr<SensingModel>& transmitterSensingPtr = transmissionInfo.sensingPtr;

    for(size_t i = 0; i < partitions.size(); i++) {
        const list<SensingInfo>& receiverSensingInfos = partitions[i].sensingInfos;

        for(IterType iter = receiverSensingInfos.begin(); (iter != receiverSensingInfos.end()); iter++) {
            const SensingInfo& receiverSensingInfo = (*iter);
            const NodeIdType receiverNodeId = receiverSensingInfo.nodeId;
            const shared_ptr<SensingModel>& receiverSensingPtr = receiverSensingInfo.sensingModelPtr;

            if (aRandomNumberGenerator.GenerateRandomDouble() < transmissionInfo.errorRate) {
                continue;
            }//if//

            if (receiverNodeId == transmitterNodeId) {
                continue;
            }//if//

            if (!receiverSensingPtr->IsOn()) {
                continue;
            }//if//


            if (transmissionInfo.transmissionCondition == TRANSMISSION_CONDITION_SIMPLEX) {
                if (transmitterSensingPtr->TargetNodeIsVisible(receiverNodeId)) {
                    receiverSensingPtr->ReceiveData(transmissionInfo.sharedInfo);
                }//if//
            }
            else if (transmissionInfo.transmissionCondition == TRANSMISSION_CONDITION_DUPLEX) {

                // bidirection sensing detection check

                if (transmitterSensingPtr->TargetNodeIsVisible(receiverNodeId) &&
                    receiverSensingPtr->TargetNodeIsVisible(transmitterNodeId)) {
                    receiverSensingPtr->ReceiveData(transmissionInfo.sharedInfo);
                }//if//
            }//if

        }//for//
    }//for//

}//DeliveryOneSharingInfo//

inline
bool SensingSubsystem::NeedToUpdateSensingInfo() const
{
    typedef list<SensingInfo>::const_iterator IterType;

    for(size_t i = 0; i < partitions.size(); i++) {
        const list<SensingInfo>& sensingInfos = partitions[i].sensingInfos;

        for(IterType iter = sensingInfos.begin(); (iter != sensingInfos.end()); iter++) {
            const SensingInfo& sensingInfo = (*iter);

            if (sensingInfo.sensingModelPtr->IsOn()) {
                return true;
            }//if//
        }//for//
    }//for//

    return false;
}//NeedToUpdateSensingInfo//

inline
void SensingSubsystem::RemoveNode(const NodeIdType& nodeId)
{
    nodeIds.remove(nodeId);

    typedef list<SensingInfo>::iterator SensingIter;

    for(size_t i = 0; i < partitions.size(); i++) {
        list<SensingInfo>& sensingInfos = partitions[i].sensingInfos;

        SensingIter sensingIter = sensingInfos.begin();

        while (sensingIter != sensingInfos.end()) {
            const SensingInfo& sensingInfo = (*sensingIter);

            if (sensingInfo.nodeId == nodeId) {
                sensingIter = sensingInfos.erase(sensingIter);
            }
            else {
                sensingIter++;
            }//if//
        }//while//
    }//for//

}//RemoveNode//


}//namespace//

#endif
