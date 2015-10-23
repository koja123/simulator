// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_PROPLOSS_H
#define SCENSIM_PROPLOSS_H

#include <algorithm>
#include <iostream>
#include <assert.h>
#include <complex>
#include <vector>
#include <stack>
#include <map>
#include <fstream>
#include <string>
#include <cmath>
#include <memory>
#include "boost/shared_array.hpp"
#include <array>


#include "scensim_engine.h"
#include "scensim_parmio.h"
#include "scensim_support.h"
#include "randomnumbergen.h"

namespace ScenSim {

using std::vector;
using std::array;
using std::make_pair;
using std::shared_ptr;
using std::enable_shared_from_this;
using std::auto_ptr;
using std::map;
using std::cerr;
using std::cout;
using std::endl;
using boost::shared_array;
using std::ifstream;
using std::ofstream;
using std::string;
using std::complex;
using std::polar;
using std::abs;
using std::swap;

typedef complex<double> DoubleComplex;

//=============================================================================

const double NOT_CALCULATED = DBL_MAX;
const double InfinitePathlossDb = DBL_MAX;
const double EXTREMELY_HIGH_LOSS_DB = 1000.0;

inline
double NormalizeAngleToNeg180To180(const double& angleDegrees)
{
    if (angleDegrees <= 180.0) {
        if (angleDegrees > -180.0) {
            return (angleDegrees);
        }
        else if (angleDegrees > -540.00) {
            return (angleDegrees + 360.0);
        }
        else {
             assert(angleDegrees > -900.0);
             return (angleDegrees + 720.0);
        }//if//
    }
    else if (angleDegrees <= 540.0) {
        return (angleDegrees - 360.0);
    }
    else {
        assert(angleDegrees <= 900.0);
        return (angleDegrees - 720.0);
    }//if//

}//NormalizeAngleToNeg180To180//


inline
void NormalizeAzimuthAndElevation(double& azimuthDegrees, double& elevationDegrees)
{
    assert((elevationDegrees <= 360.0) && (elevationDegrees > -360.0));

    if (elevationDegrees <= 90.0) {
        if (elevationDegrees >= -90.0) {
            azimuthDegrees = NormalizeAngleToNeg180To180(azimuthDegrees);
        }
        else if (elevationDegrees >= -180.0) {
            elevationDegrees = -(180.0 + elevationDegrees);
            azimuthDegrees = NormalizeAngleToNeg180To180(azimuthDegrees + 180.0);
        }
        else {
            elevationDegrees = NormalizeAngleToNeg180To180(elevationDegrees);
            // Recurse.
            NormalizeAzimuthAndElevation(azimuthDegrees, elevationDegrees);
        }//if//
    }
    else if (elevationDegrees <= 180.0) {

        elevationDegrees = 180.0 - elevationDegrees;
        azimuthDegrees = NormalizeAngleToNeg180To180(azimuthDegrees + 180.0);
    }
    else {
        elevationDegrees = NormalizeAngleToNeg180To180(elevationDegrees);
        // Recurse.
        NormalizeAzimuthAndElevation(azimuthDegrees, elevationDegrees);
    }//if//

}//NormalizeAzimuthAndElevation//



typedef int PropagationInformationType;
enum {
    PROPAGATION_INFORMATION_PATHLOSS = (1 << 0),
    PROPAGATION_INFORMATION_PROPAGATIONPATH = (1 << 1),
    PROPAGATION_INFORMATION_IS_VERTICAL_PATHLOSS = (1 << 2),
};

struct PropagationVertexType {
    double x;
    double y;
    double z;
    string vertexType;

    PropagationVertexType() {}
    PropagationVertexType(
        const double initX,
        const double initY,
        const double initZ,
        const string& initVertexType)
        :
        vertexType(initVertexType),
        x(initX),
        y(initY),
        z(initZ)
    {}
};

struct PropagationPathType {

    // Be modified by ray path trace.
    mutable double lossValueDb;

    vector<PropagationVertexType> pathVertices;

    double efieldMagPhi;
    double efieldMagTheta;
    double efieldPhasePhi;
    double efieldPhaseTheta;

    PropagationPathType()
        :
        lossValueDb(0),
        efieldMagPhi(0),
        efieldMagTheta(0),
        efieldPhasePhi(0),
        efieldPhaseTheta(0)
    {
    }

    void SetLossValueDb(const double valueDb) const {
        lossValueDb = valueDb;
    }
};

struct PropagationStatisticsType {

    // Be modified by ray path trace.
    mutable double totalLossValueDb;

    vector<PropagationPathType> paths;

    double txGain;
    double rxGain;

    PropagationStatisticsType()
        :
        totalLossValueDb(0),
        txGain(0),
        rxGain(0)
    {
    }

    void SetTotalLossValueDb(const double valueDb) const {
        totalLossValueDb = valueDb;
    }
};

struct RayPathTraceKey {

    struct PositionType {
        PositionType(
            const double initXMeters,
            const double initYMeters,
            const double initZMeters)
            :
            xMilliMeters(MeterToMilliMeter(initXMeters)),
            yMilliMeters(MeterToMilliMeter(initYMeters)),
            zMilliMeters(MeterToMilliMeter(initZMeters)) {
        }

        int64_t xMilliMeters;
        int64_t yMilliMeters;
        int64_t zMilliMeters;

        bool operator<(const PositionType& right) const {
            if (xMilliMeters < right.xMilliMeters) {
                return true;
            } else if (xMilliMeters == right.xMilliMeters) {
                if (yMilliMeters < right.yMilliMeters) {
                    return true;
                } else if (yMilliMeters == right.yMilliMeters) {
                    return zMilliMeters < right.zMilliMeters;
                }
            }
            return false;
        }

        bool operator==(const PositionType& right) const {
            if (xMilliMeters == right.xMilliMeters &&
                yMilliMeters == right.yMilliMeters &&
                zMilliMeters == right.zMilliMeters) {
                return true;
            }
            return false;
        }

        bool operator!=(const PositionType& right) const {
            return !((*this) == right);
        }

    private:
        static inline int64_t MeterToMilliMeter(const double meter) {
            return static_cast<int64_t>(meter * 1000.0);
        }
    };

    bool operator<(const RayPathTraceKey& right) const {
        if (txPosition < right.txPosition) {
            return true;
        } else if (txPosition == right.txPosition) {
            return rxPosition < right.rxPosition;
        }
        return false;
    }

    RayPathTraceKey(
        const PositionType& initTxPosition,
        const PositionType& initRxPosition)
        :
        txPosition(initTxPosition),
        rxPosition(initRxPosition) {
    }

    PositionType txPosition, rxPosition;
};

struct LossKey {
    NodeIdType lowNodeId;
    NodeIdType highNodeId;

    bool operator<(const LossKey& right) const {
        return ((lowNodeId < right.lowNodeId) ||
                ((lowNodeId == right.lowNodeId &&
                  highNodeId < right.highNodeId)));
    }

    LossKey() {}
    LossKey(
        const NodeIdType& nodeId1,
        const NodeIdType& nodeId2)
        :
        lowNodeId(std::min(nodeId1, nodeId2)),
        highNodeId(std::max(nodeId1, nodeId2))
    {}
};

typedef map<RayPathTraceKey, PropagationStatisticsType> RayPathTraceType;
typedef map<LossKey, PropagationStatisticsType> LossValuesCacheType;

class ObjectMobilityPosition {
public:
    ObjectMobilityPosition(
        const TimeType& initLastMoveTime,
        const TimeType& initEarliestNextMoveTime,
        const double& initXPositionMeters,
        const double& initYPositionMeters,
        const double& initTheHeightFromGroundMeters,
        const bool& initTheHeightContainsGroundHeightMeters,
        const NodeIdType& initNodeIdType,
        const double& initAttitudeAzimuthDegrees = 0.0,
        const double& initAttitudeElevationDegrees = 0.0,
        const double& initVelocityMetersPerSecond = 0.0,
        const double& initVelocityAzimuthDegrees = 0.0,
        const double& initVelocityElevationDegrees = 0.0)
    :
        theLastMoveTime(initLastMoveTime),
        theEarliestNextMoveTime(initEarliestNextMoveTime),
        xPositionMeters(initXPositionMeters),
        yPositionMeters(initYPositionMeters),
        theHeightFromGroundMeters(initTheHeightFromGroundMeters),
        theHeightContainsGroundHeightMeters(initTheHeightContainsGroundHeightMeters),
        objectId(initNodeIdType),
        attitudeAzimuthDegrees(initAttitudeAzimuthDegrees),
        attitudeElevationDegrees(initAttitudeElevationDegrees),
        //attitudeRotationDegrees(initAttitudeRotationDegrees)
        velocityMetersPerSecond(initVelocityMetersPerSecond),
        velocityAzimuthDegrees(initVelocityAzimuthDegrees),
        velocityElevationDegrees(initVelocityElevationDegrees)
    {}

    ObjectMobilityPosition()
      : theLastMoveTime(ZERO_TIME), theEarliestNextMoveTime(ZERO_TIME), xPositionMeters(DBL_MIN),
        yPositionMeters(DBL_MIN), theHeightFromGroundMeters(DBL_MIN), attitudeAzimuthDegrees(0.0),
        theHeightContainsGroundHeightMeters(false),
        attitudeElevationDegrees(0.0), velocityMetersPerSecond(0.0),
        velocityAzimuthDegrees(0.0), velocityElevationDegrees(0.0) {}

    TimeType LastMoveTime() const { return theLastMoveTime; }
    TimeType EarliestNextMoveTime() const { return theEarliestNextMoveTime; }

    double X_PositionMeters() const { return xPositionMeters; }
    double Y_PositionMeters() const { return yPositionMeters; }
    double HeightFromGroundMeters() const { return theHeightFromGroundMeters; }
    bool TheHeightContainsGroundHeightMeters() const { return theHeightContainsGroundHeightMeters; }
    NodeIdType GetObjectId() const { return objectId; }
    void SetObjectId(const NodeIdType& id) { objectId = id; }

    double AttitudeAzimuthFromNorthClockwiseDegrees() const { return attitudeAzimuthDegrees; }
    double AttitudeElevationFromHorizonDegrees() const { return attitudeElevationDegrees; }

    double VelocityMetersPerSecond() const { return velocityMetersPerSecond; }
    double VelocityAzimuthFromNorthClockwiseDegrees() const { return velocityAzimuthDegrees; }
    double VelocityElevationFromHorizonDegrees() const { return velocityElevationDegrees; }

    void SetLastMoveTime(const TimeType& lastMoveTime) { theLastMoveTime = lastMoveTime; }
    void SetEarliestNextMoveTime(const TimeType& nextMoveTime) { theEarliestNextMoveTime = nextMoveTime; }
    void SetX_PositionMeters(const double& newXPosition) { xPositionMeters = newXPosition; }
    void SetY_PositionMeters(const double& newYPosition) { yPositionMeters = newYPosition; }
    void SetHeightFromGroundMeters(const double& newHeight) { theHeightFromGroundMeters = newHeight; }
    void SetTheHeightContainsGroundHeightMeters(const bool newTheHeightContainsGroundHeightMeters) { theHeightContainsGroundHeightMeters = newTheHeightContainsGroundHeightMeters; }

    void SetAttitudeFromNorthClockwiseDegrees(const double& newAttitude)
        { attitudeAzimuthDegrees = newAttitude; }

    void SetAttitudeElevationFromHorizonDegrees(const double& newElevation)
        { attitudeElevationDegrees = newElevation; }

    void SetVelocityMetersPerSecond(const double& newVelocity)
        { velocityMetersPerSecond = newVelocity; }

    void SetVelocityFromNorthClockwiseDegrees(const double& newVelocityAzimuth)
        { velocityAzimuthDegrees = newVelocityAzimuth; }

    void SetVelocityElevationFromHorizonDegrees(const double& newVelocityElevation)
        { velocityElevationDegrees = newVelocityElevation; }

    // Assuming no "roll" rotation of antenna patterns for now.

    //double AttitudeRotationDegrees() const { return attitudeRotationDegrees; }


private:
    TimeType theLastMoveTime;
    TimeType theEarliestNextMoveTime;
    double xPositionMeters;
    double yPositionMeters;
    double theHeightFromGroundMeters;
    bool theHeightContainsGroundHeightMeters;
    double attitudeAzimuthDegrees;
    double attitudeElevationDegrees;
    //double attitudeRotationDegrees;
    NodeIdType objectId;
    double velocityMetersPerSecond;
    double velocityAzimuthDegrees;
    double velocityElevationDegrees;

};//ObjectMobilityPosition//



class ObjectMobilityModel {
public:
    typedef unsigned int MobilityObjectIdType;

    ObjectMobilityModel()
        :
        relativeAttitudeAzimuthDegrees(0.0)
    {}

    ObjectMobilityModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const InterfaceOrInstanceIdType& interfaceId);

    virtual ~ObjectMobilityModel() { }

    void GetPositionForTime(
        const TimeType& snapshotTime,
        ObjectMobilityPosition& position);

    // Without relative azimuth rotation.
    virtual void GetUnadjustedPositionForTime(
        const TimeType& snapshotTime,
        ObjectMobilityPosition& position) = 0;

    virtual TimeType GetCreationTime() const { return ZERO_TIME; }
    virtual TimeType GetDeletionTime() const { return INFINITE_TIME; }

    // For steerable directional antenna support:


    void SetRelativeAttitudeAzimuth(
        const TimeType& currentTime,
        const double& azimuthDegrees)
    {
        (*this).relativeAttitudeAzimuthDegrees = azimuthDegrees;
    }

    double GetRelativeAttitudeAzimuth() const
    {
        return (relativeAttitudeAzimuthDegrees);
    }

private:
    double relativeAttitudeAzimuthDegrees;

};//ObjectMobilityModel//


inline
void ObjectMobilityModel::GetPositionForTime(
    const TimeType& snapshotTime,
    ObjectMobilityPosition& position)
{
    (*this).GetUnadjustedPositionForTime(snapshotTime, position);

    if (relativeAttitudeAzimuthDegrees != 0.0) {
        position.SetAttitudeFromNorthClockwiseDegrees(
            NormalizeAngleToNeg180To180(
                (position.AttitudeAzimuthFromNorthClockwiseDegrees() + relativeAttitudeAzimuthDegrees)));
    }//if//

}//GetPositionForTime//


class InorderFileCache {
public:
    void Clear() { fileName = ""; filePosition = 0; lastObjectId = 0; }
private:
    string   fileName;
    std::streampos filePosition;
    ObjectMobilityModel::MobilityObjectIdType lastObjectId;

    friend class TraceFileMobilityModel;
};

shared_ptr<ObjectMobilityModel> CreateAntennaMobilityModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceOrInstanceIdType& interfaceId,
    const RandomNumberGeneratorSeedType& seed,
    InorderFileCache& fileCache);


inline
shared_ptr<ObjectMobilityModel> CreateAntennaMobilityModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceOrInstanceIdType& interfaceId,
    const RandomNumberGeneratorSeedType& seed)
{
    InorderFileCache notUsedFileCache;
    return (CreateAntennaMobilityModel(
        theParameterDatabaseReader, nodeId, interfaceId, seed, notUsedFileCache));
}



//=============================================================================

class AntennaPattern {
public:
    AntennaPattern(const unsigned int interpolationAlgorithmNumber = 1);

    void AddGainData(
        const int azimuthDegrees,
        const int elevationDegrees,
        const double& gainValueDbi);

    void FinalizeShape();

    void SetNonInterpolationMode() { alwaysInterpolateBetweenDatapoints = false; }

    double IsTwoDimensional() const { return (isTwoDimensional); }

    double GetGainDbi(
        const double& azimuthDegrees,
        const double& elevationDegrees) const;

private:
    int interpolationAlgorithmNumber;
    bool shapeIsFinalized;
    bool isTwoDimensional;
    bool haveSeenSome3dData;
    bool alwaysInterpolateBetweenDatapoints;

    unsigned int numberDatapoints;

    vector<vector<double> > patternDataDbi;

    void BuildFull3dShapeFromHorizontalAndVertPlaneData();

};//AntennaPattern//



inline
AntennaPattern::AntennaPattern(const unsigned int initInterpolationAlgorithmNumber)
    :
    shapeIsFinalized(false),
    isTwoDimensional(true),
    numberDatapoints(0),
    haveSeenSome3dData(false),
    alwaysInterpolateBetweenDatapoints(true),
    interpolationAlgorithmNumber(initInterpolationAlgorithmNumber)
{}


inline
void ConvertSphericalCoordinatesToIndices(
    const double& azimuthDegrees,
    const double& elevationDegrees,
    unsigned int& azimuthIndex,
    unsigned int& elevationIndex)
{
    int signedAzimuthIndex = RoundToInt(azimuthDegrees) + 179;
    if (signedAzimuthIndex == -1) {
        signedAzimuthIndex = 359;
    }//if//
    assert((signedAzimuthIndex >=0) && (signedAzimuthIndex <= 359));

    azimuthIndex = static_cast<unsigned int>(signedAzimuthIndex);

    assert((elevationDegrees >= -90.0) && (elevationDegrees <= 90.0));

    elevationIndex = RoundToUint(elevationDegrees + 90.0);

}//ConvertSphericalCoordinatesToIndices//



inline
double AntennaPattern::GetGainDbi(
    const double& azimuthDegrees,
    const double& elevationDegrees) const
{
    assert(shapeIsFinalized);

    if(isTwoDimensional) {
        if (!alwaysInterpolateBetweenDatapoints) {
            int azimuthIndex = RoundToInt(azimuthDegrees) + 179;
            if (azimuthIndex == -1) {
                azimuthIndex = 359;
            }//if//

            return (patternDataDbi[0].at(azimuthIndex));
        }
        else {
            assert((azimuthDegrees > -180.0) && (azimuthDegrees <= 180.0));

            const double floorAzimuthDegrees = floor(azimuthDegrees);

            int lowAzimuthIndex = static_cast<int>(floorAzimuthDegrees + 179.0);
            if (lowAzimuthIndex == -1) {
                lowAzimuthIndex = 359;
            }//if//

            unsigned int highAzimuthIndex = static_cast<unsigned int>(lowAzimuthIndex + 1);

            if (highAzimuthIndex == 360) {
                highAzimuthIndex = 0;
            }//if//

            return (
                CalcInterpolatedValue(
                    floorAzimuthDegrees,
                    patternDataDbi[0][lowAzimuthIndex],
                    (floorAzimuthDegrees + 1.0),
                    patternDataDbi[0][highAzimuthIndex],
                    azimuthDegrees));
        }//if//
    }
    else {
        if (!alwaysInterpolateBetweenDatapoints) {
            unsigned int azimuthIndex;
            unsigned int elevationIndex;

            ConvertSphericalCoordinatesToIndices(
                azimuthDegrees,
                elevationDegrees,
                azimuthIndex,
                elevationIndex);

            return (patternDataDbi[elevationIndex][azimuthIndex]);
        }
        else {
            assert((azimuthDegrees > -180.0) && (azimuthDegrees <= 180.0));
            assert((elevationDegrees >= -90.0) && (elevationDegrees <= 90.0));

            const double floorAzimuthDegrees = floor(azimuthDegrees);
            const double floorElevationDegrees = floor(elevationDegrees);
            const double deltaElevation = elevationDegrees - floorElevationDegrees;

            int lowAzimuthIndex = static_cast<int>(floorAzimuthDegrees + 179.0);
            if (lowAzimuthIndex == -1) {
                lowAzimuthIndex = 359;
            }//if//

            unsigned int highAzimuthIndex = static_cast<unsigned int>(lowAzimuthIndex + 1);

            if (highAzimuthIndex == 360) {
                highAzimuthIndex = 0;
            }//if//

            const unsigned int lowElevationIndex =
                static_cast<unsigned int>(floorElevationDegrees + 90.0);

            unsigned int highElevationIndex = lowElevationIndex + 1;

            if (highElevationIndex == 181) {
                highElevationIndex = 0;
            }//if//

            const double lowerValue =
                CalcInterpolatedValue(
                    floorAzimuthDegrees,
                    patternDataDbi[lowElevationIndex][lowAzimuthIndex],
                    (floorAzimuthDegrees + 1.0),
                    patternDataDbi[lowElevationIndex][highAzimuthIndex],
                    azimuthDegrees);

            const double upperValue =
                CalcInterpolatedValue(
                    floorAzimuthDegrees,
                    patternDataDbi[highElevationIndex][lowAzimuthIndex],
                    (floorAzimuthDegrees + 1.0),
                    patternDataDbi[highElevationIndex][highAzimuthIndex],
                    azimuthDegrees);

            return((lowerValue * (1.0 - deltaElevation)) + (upperValue * deltaElevation));

        }//if//
    }//if//

}//GetGainDbi//


//
// This interpolation algorithm uses the horizonatal plane data without modification,
// i.e. the horizontal component of the interpolation will just be value for the
// azimuth from the horizontal data table. The vertical plane data component
// is the closest point to the specified point on the sphere intersection
// with the X-Z plane (vertical // pattern plane).  The two data components
// are combined (linear) using the normalized "elevation angle"
// in a plane parallel to the Y-Z plane (plane perpendicular to boresight)
// that goes through the specified point on the sphere.
// Note, this algorithm preserves horizontal features (e.g. nulls, lobes) with azimuth,
// where as vertical pattern features are continued in the Y direction (perpendicular
// to the boresight). Also note, this interpolation should exactly match on all specified data
// (horizontal/vert planes).
//

inline
double CalcInterpolatedGainFromHorizontalAndVertPlaneDataAlgo1(
    const vector<double>& horizontalPatternTable,
    const vector<double>& verticalPatternTable,
    const int azimuthClockwiseFromBoresightDegrees,
    const int elevationDegrees)
{
    assert(azimuthClockwiseFromBoresightDegrees > -180);
    assert(azimuthClockwiseFromBoresightDegrees <= 180);
    assert((elevationDegrees >= -90) && (elevationDegrees <= 90));

    const int azimuthIndex = 179 + azimuthClockwiseFromBoresightDegrees;

    // Handle non-interpolated special cases:

    if (elevationDegrees == 0) {
       return (horizontalPatternTable.at(azimuthIndex));
    }//if//

    if (std::abs(elevationDegrees) == 90) {
       return (verticalPatternTable.at(179 + elevationDegrees));
    }//if//

    if (azimuthClockwiseFromBoresightDegrees == 0) {
        return (verticalPatternTable.at(179 + elevationDegrees));
    }//if//

    if (azimuthClockwiseFromBoresightDegrees == 180) {
        if (elevationDegrees >= 0) {
            return (verticalPatternTable.at(179 + (180 - elevationDegrees)));
        }
        else {
            return (verticalPatternTable.at(179 + (-180 - elevationDegrees)));
        }//if//
    }//if//

    const double azimuthRadians =
        (azimuthClockwiseFromBoresightDegrees / 180.0) * PI;

    const double elevationRadians = (elevationDegrees / 180.0) * PI;

    // Slice sphere perpendicular to boresight at point on "gain" sphere.

    const double cosElevation = cos(elevationRadians);

    const double sliceOffsetX = cos(azimuthRadians) * cosElevation;
    const double sliceOffsetY = sin(azimuthRadians) * cosElevation;
    const double sliceRadius = sqrt(1.0 - (sliceOffsetX * sliceOffsetX));

    double sliceAzimuthRadians = acos(sliceOffsetX);

    if (azimuthClockwiseFromBoresightDegrees < 0) {
        sliceAzimuthRadians = -sliceAzimuthRadians;
    }//if//

    double sliceElevationRadians = acos(std::abs(sliceOffsetX));

    if (std::abs(azimuthClockwiseFromBoresightDegrees) > 90) {
        sliceElevationRadians = PI - sliceElevationRadians;
    }//if//

    if (elevationDegrees < 0) {
        sliceElevationRadians = -sliceElevationRadians;
    }//if//

    const double sliceElevationDegrees = 180.0 * (sliceElevationRadians / PI);

    int elevationIndex = RoundToInt(sliceElevationDegrees) + 179;
    if (elevationIndex == -1) {
        elevationIndex = 359;
    }//if//

    // Elevation angle on slice centered on boresight axis.

    if (sliceRadius < DBL_EPSILON) {
        assert(std::abs(verticalPatternTable.at(elevationIndex) - horizontalPatternTable.at(azimuthIndex)) <
               DBL_EPSILON);

        return (horizontalPatternTable.at(azimuthIndex));
    }//if//

    double sliceInternalElevationAngle = 0.0;
    if (std::abs(sliceOffsetY) < sliceRadius) {
        sliceInternalElevationAngle = acos(std::abs(sliceOffsetY) / sliceRadius);
    }
    else {
        assert((std::abs(sliceOffsetY) - sliceRadius) < (DBL_EPSILON * 10));
    }//if//

    // Normalize angle 0..PI/2 --> 0..1.0
    const double v = (sliceInternalElevationAngle / (PI/2.0));

    return
       ((horizontalPatternTable.at(azimuthIndex) * (1.0 - v)) +
       (verticalPatternTable.at(elevationIndex) * v));

}//CalcInterpolatedGainFromHorizontalAndVertPlaneDataAlgo1//



// This interpolation algorithm uses the horizontal plane data without modification,
// i.e. the horizontal component of the interpolation will just be value for the
// azimuth from the horizontal data table.  The vertical pattern data is rotated
// around the poles of the sphere.  The specified elevation is used to get
// the front and back data points from the vertical pattern data and the azimuth
// is used to interpolate (simple linear) between the two points.
// This vertical component is then combined with the horizontal data component
// as with "Algorithm 1", a plane perpendicular to the boresight (parallel to
// the Y-Z plane) is passed through the specified point on the sphere, and
// an "elevation" is calculated in this plane (instead of with respect to
// the center of the sphere).  This is angle is normalized to 0..1 for
// the linear interpolation between horizontal and vertical data components.
// Also note, this interpolation should exactly match on all specified data
// (horizontal/vert planes). Using the "alternate elevation" instead of
// just the elevation is to provide agreement with the vertical pattern
// data.
//
// Note that with this algorithm, vertical data near the poles will have
// only local effect.

inline
double CalcInterpolatedGainFromHorizontalAndVertPlaneDataAlgo2(
    const vector<double>& horizontalPatternTable,
    const vector<double>& verticalPatternTable,
    const int azimuthClockwiseFromBoresightDegrees,
    const int elevationDegrees)
{
    // This algorithm

    assert(azimuthClockwiseFromBoresightDegrees > -180);
    assert(azimuthClockwiseFromBoresightDegrees <= 180);
    assert((elevationDegrees >= -90) && (elevationDegrees <= 90));

    const int azimuthIndex = 179 + azimuthClockwiseFromBoresightDegrees;

    // Elevation Index 1 is in positive half of the sphere
    // (boresight direction side, Azimuths -90..+90),
    // elevation 2 is for the negative half.

    const int elevationIndex1 = 179 + elevationDegrees;

    int elevationIndex2;

    if (elevationDegrees >= 0.0) {
        elevationIndex2 = 179 + (180 - elevationDegrees);
    }
    else {
        elevationIndex2 = 179 + (-180 - elevationDegrees);
    }//if//

    // Handle non-interpolated special cases:

    if (elevationDegrees == 0) {
       return (horizontalPatternTable.at(azimuthIndex));
    }//if//

    if (std::abs(elevationDegrees) == 90) {
       return (verticalPatternTable.at(elevationIndex1));
    }//if//

    if (azimuthClockwiseFromBoresightDegrees == 0) {
        return (verticalPatternTable.at(elevationIndex1));
    }//if//

    if (azimuthClockwiseFromBoresightDegrees == 180) {
        return (verticalPatternTable.at(elevationIndex2));
    }//if//

    // Normalize to azimuth to 0..1.

    const double azimuthFactor = (std::abs(azimuthClockwiseFromBoresightDegrees) / 180.0);

    const double interpolatedVerticalValue =
        ((verticalPatternTable[elevationIndex1] * (1.0 - azimuthFactor)) +
        (verticalPatternTable[elevationIndex2] * (azimuthFactor)));

    const double azimuthRadians =
        (azimuthClockwiseFromBoresightDegrees / 180.0) * PI;
    const double elevationRadians = (elevationDegrees / 180.0) * PI;

    // Slice sphere perpendicular to boresight at point on "gain" sphere.

    const double cosElevation = cos(elevationRadians);
    const double sinElevation = sin(elevationRadians);

    const double sliceOffsetX = cos(azimuthRadians) * cosElevation;
    const double sliceOffsetY = sin(azimuthRadians) * cosElevation;
    const double sliceRadius = sqrt(1.0 - (sliceOffsetX * sliceOffsetX));

    double sliceInternalElevationAngle = 0.0;
    if (std::abs(sliceOffsetY) < sliceRadius) {
        sliceInternalElevationAngle = acos(std::abs(sliceOffsetY) / sliceRadius);
    }
    else {
        assert((std::abs(sliceOffsetY) - sliceRadius) < (DBL_EPSILON * 10));
    }//if//

    // Normalize angle 0..PI/2 --> 0..1.0
    const double v = (sliceInternalElevationAngle / (PI/2.0));

    return
       ((horizontalPatternTable.at(azimuthIndex) * (1.0 - v)) +
        (interpolatedVerticalValue * v));


}//CalcInterpolatedGainFromHorizontalAndVertPlaneDataAlgo2//





inline
double CalculateInterpolatedGainFromHorizontalAndVertPlaneData(
    const unsigned int interpolationAlgorithmNumber,
    const vector<double>& horizontalPatternTable,
    const vector<double>& verticalPatternTable,
    const int azimuthClockwiseFromBoresightDegrees,
    const int elevationDegrees)
{
    if (interpolationAlgorithmNumber == 1) {
       return (
           CalcInterpolatedGainFromHorizontalAndVertPlaneDataAlgo1(
               horizontalPatternTable,
               verticalPatternTable,
               azimuthClockwiseFromBoresightDegrees,
               elevationDegrees));
    }
    else if (interpolationAlgorithmNumber == 2) {
       return (
           CalcInterpolatedGainFromHorizontalAndVertPlaneDataAlgo2(
               horizontalPatternTable,
               verticalPatternTable,
               azimuthClockwiseFromBoresightDegrees,
               elevationDegrees));
    }
    else {
        cerr << "Error: 2x2D pattern to 3D pattern interpolation algorithm number:"
             << interpolationAlgorithmNumber << " does not exist." << endl;
        exit(1);
    }//if//

}//CalculateInterpolatedGainFromHorizontalAndVertPlaneData//



inline
void AntennaPattern::BuildFull3dShapeFromHorizontalAndVertPlaneData()
{
    vector<double> horizontalPatternTable(360);
    horizontalPatternTable = patternDataDbi[90 + 0];

    vector<double> verticalPatternTable(360);

    for(int angle = -179; (angle < -90); angle++) {
        verticalPatternTable[179 + angle] = patternDataDbi[90+(-180 - angle)][179+180];
    }//for//

    for(int angle = -90; (angle <=90); angle++) {
        verticalPatternTable[179 + angle] = patternDataDbi[90+angle][179+0];
    }//for//

    for(int angle = 91; (angle <= 180); angle++) {
        verticalPatternTable[179 + angle] = patternDataDbi[90 + (180 - angle)][179+180];
    }//for//

    for(int elevationDeg = -90; (elevationDeg <= 90); elevationDeg++) {
        for(int azimuthDeg = -179; (azimuthDeg <= 180); azimuthDeg++) {

            patternDataDbi[90 + elevationDeg][179 + azimuthDeg] =
                CalculateInterpolatedGainFromHorizontalAndVertPlaneData(
                    interpolationAlgorithmNumber,
                    horizontalPatternTable,
                    verticalPatternTable,
                    azimuthDeg,
                    elevationDeg);

        }//for//
    }//for//

}//BuildFull3dShapeFromHorizonatalAndVertPlaneData//




inline
void AntennaPattern::AddGainData(
    const int azimuthDegrees,
    const int elevationDegrees,
    const double& gainValueDbi)
{
    assert((azimuthDegrees > -180) && (azimuthDegrees <= 180));
    assert((elevationDegrees >= -90) && (elevationDegrees <= 90));
    assert(!shapeIsFinalized);

    if ((isTwoDimensional) && (elevationDegrees != 0)) {
        (*this).isTwoDimensional = false;
        if (patternDataDbi.empty()) {
           (*this).patternDataDbi.resize(181, vector<double>(360, DBL_MAX));
        }
        else {
           const vector<double> zeroAzimuthRow = patternDataDbi.at(0);
           (*this).patternDataDbi.clear();
           (*this).patternDataDbi.resize(181, vector<double>(360, DBL_MAX));
           (*this).patternDataDbi[90 + 0] = zeroAzimuthRow;
        }//if//
    }//if//

    if (isTwoDimensional) {
        if (patternDataDbi.empty()) {
            patternDataDbi.resize(1, vector<double>(360, DBL_MAX));
        }//if//

        double& element = (*this).patternDataDbi[0][azimuthDegrees + 179];

        if ((element != DBL_MAX) && (element != gainValueDbi)) {

            cerr << "Error in Antenna pattern: Conflicting data at azimuth = " << azimuthDegrees
                 << " and elevation = " << elevationDegrees << endl;
            exit(1);
        }//if//

        element = gainValueDbi;
        numberDatapoints++;
    }
    else {
        if ((!haveSeenSome3dData) &&
            ((azimuthDegrees != 0) && (azimuthDegrees !=180) && (elevationDegrees != 0))) {

            // Not on horizontal and vertical planes
            (*this).haveSeenSome3dData = true;
        }//if//

        double& element = (*this).patternDataDbi[elevationDegrees+90][azimuthDegrees + 179];
        if ((element != DBL_MAX) && (element != gainValueDbi)) {
            cerr << "Error in Antenna pattern file: Conflicting data at azimuth = " << azimuthDegrees
                 << " and elevation = " << elevationDegrees << endl;
            exit(1);
        }//if//

        element  = gainValueDbi;
        numberDatapoints++;
    }//if//

}//AddGainValue//




inline
void AntennaPattern::FinalizeShape()
{
    assert(!shapeIsFinalized);
    (*this).shapeIsFinalized = true;

    if  (((isTwoDimensional) && (numberDatapoints != 360)) ||
         ((!isTwoDimensional) && (!haveSeenSome3dData) && (numberDatapoints != (360+360))) ||
         ((haveSeenSome3dData) && (numberDatapoints != (360*181)))) {

        cerr << "Error in Antenna pattern: Wrong number of datapoints: "
             << numberDatapoints << "." << endl;
        cerr << "  Two Dimensional should have 360, 2-2d should have 720, 3d should have 65160." << endl;
        exit(1);
    }//if//


    if ((!isTwoDimensional) && (!haveSeenSome3dData)) {
        (*this).BuildFull3dShapeFromHorizontalAndVertPlaneData();
    }//if//

}//FinalizeShape//


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


class AntennaPatternDatabase {
public:
    AntennaPatternDatabase(
        const string& antennaFileName,
        const bool useLegacyFileFormatMode,
        const unsigned int two2dTo3dInterpolationAlgorithmNumber,
        const string& antennaPatternDebugDumpFileName);

    bool IsDefined(const string& antennaModelName) const;

    void LoadAntennaFile(
        const string& antennaFileName,
        const bool useLegacyFormatMode,
        const unsigned int two2dTo3dInterpolationAlgorithmNumber,
        const string& antennaPatternDebugDumpFileName);

    shared_ptr<AntennaPattern> GetAntennaPattern(const string& antennaModelName) const;

private:
    set<string> loadedFileNames;

    map<string, shared_ptr<AntennaPattern> > antennaPatternMap;

    void AddAntennaPattern(
        const string& antennaPatternName,
        const shared_ptr<AntennaPattern>& antennaPatternPtr);

    void DumpAntennaPattern(
        const string& antennaPatternName,
        const AntennaPattern& theAntennaPattern,
        ofstream& outputStream);

};//AntennaPatternDatabase//


inline
AntennaPatternDatabase::AntennaPatternDatabase(
    const string& antennaFileName,
    const bool useLegacyFileFormatMode,
    const unsigned int two2dTo3dInterpolationAlgorithmNumber,
    const string& antennaPatternDebugDumpFileName)
{
    if (antennaFileName != "") {
        (*this).LoadAntennaFile(
            antennaFileName,
            useLegacyFileFormatMode,
            two2dTo3dInterpolationAlgorithmNumber,
            antennaPatternDebugDumpFileName);
    }//if//

}//AntennaPatternDatabase//


inline
bool AntennaPatternDatabase::IsDefined(const string& antennaModelName) const
{

    return (antennaPatternMap.find(antennaModelName) != antennaPatternMap.end());

}//Defined//


inline
void AntennaPatternDatabase::AddAntennaPattern(
    const string& antennaPatternName,
    const shared_ptr<AntennaPattern>& antennaPatternPtr)
{
    if (antennaPatternMap.find(antennaPatternName) != antennaPatternMap.end()) {
        cerr << "Error duplicate custom antenna model name: " << antennaPatternName << endl;
        exit(1);
    }//if//

    antennaPatternMap[antennaPatternName] = antennaPatternPtr;

}//AddAntennaPattern//


inline
shared_ptr<AntennaPattern> AntennaPatternDatabase::GetAntennaPattern(
    const string& antennaModelName) const
{
    map<string, shared_ptr<AntennaPattern> >::const_iterator iter =
        antennaPatternMap.find(antennaModelName);

    if (iter == antennaPatternMap.end()) {
        cerr << "Error: No custom antenna model name: " << antennaModelName << endl;
        exit(1);
    }//if//

    return (iter->second);

}//GetAntennaPattern//


//=============================================================================


class AntennaModel {
public:
    AntennaModel() { }
    virtual ~AntennaModel() { }

    virtual bool IsOmniDirectional() const = 0;

    virtual double GetOmniGainDbi() const = 0;

    virtual double GainInDbForThisDirection(
        const double& azimuthFromBoresightClockwiseDegrees = 0.0,
        const double& elevationFromBoresightDegrees = 0.0,
        const double& currentAntennaRotation = 0.0) const = 0;

    // To support directional antenna systems with "Quasi-Omni mode".

    virtual bool SupportsQuasiOmniMode() const { return false; }
    virtual bool IsInQuasiOmniMode() const { return false; }
    virtual void SwitchToQuasiOmniMode() { assert(false); abort(); }
    virtual void SwitchToDirectionalMode() { assert(false); abort(); }

private:
    // Disabling:

    AntennaModel(const AntennaModel&);
    void operator=(const AntennaModel&);
};


class OmniAntennaModel: public AntennaModel {
public:
    explicit OmniAntennaModel(const double& initGainDbi) : gainDbi(initGainDbi) {}

    virtual bool IsOmniDirectional() const override { return true; }
    virtual double GetOmniGainDbi() const override { return gainDbi; }

    virtual double GainInDbForThisDirection(
        const double& azimuthFromNorthClockwiseDegrees = 0.0,
        const double& elevationDegrees = 0.0,
        const double& currentAntennaRotation = 0.0) const override
    {
        return gainDbi;
    }

private:
    double gainDbi;

};//OmniAntennaModel//


//reference: ITU-R M.2135
class SectoredAntennaModel: public AntennaModel {
public:
    SectoredAntennaModel(const double& boresightGainDbi) : maxGainDbi(boresightGainDbi) { }

    virtual bool IsOmniDirectional() const override { return false; }

    virtual double GetOmniGainDbi() const override { assert(false); abort(); return 0.0; }

    virtual double GainInDbForThisDirection(
        const double& azimuthFromBoresightDegrees,
        const double& elevationFromBoresightDegrees,
        const double& currentAntennaRotation = 0.0) const override
    {
        using std::max;

        assert(currentAntennaRotation == 0.0);
        assert(azimuthFromBoresightDegrees >= -180.0);
        assert(azimuthFromBoresightDegrees <= 180.0);
        assert(elevationFromBoresightDegrees >= -90.0);
        assert(elevationFromBoresightDegrees <= 90.0);

        double orientationRatio = azimuthFromBoresightDegrees / the3dbBeamWidthDegrees;
        double elevationRatio = elevationFromBoresightDegrees / the3dbBeamElevationDegrees;

        return
            (maxGainDbi +
            max(
                (orientationRatio * orientationRatio * double(gainAtTwiceBeamWidthDb)),
                -double(frontToBackRatioDb)) +
            max(
                (elevationRatio * elevationRatio * double(gainAtTwiceBeamWidthDb)),
                -double(frontToBackRatioDb))
             );

    }//GainInDbForThisDirection//


private:
    double maxGainDbi;
    static const int the3dbBeamWidthDegrees = 70;
    static const int frontToBackRatioDb = 20;
    static const int gainAtTwiceBeamWidthDb = -12;
    static const int the3dbBeamElevationDegrees = 15;
};//SectoredAntennaModel//



class CustomAntennaModel: public AntennaModel {
public:
    CustomAntennaModel(
        const double& initQuasiOmniModeGainDbi = DBL_MAX)
        :
        hasQuasiOmniMode(initQuasiOmniModeGainDbi != DBL_MAX),
        quasiOmniModeGainDbi(initQuasiOmniModeGainDbi)
    {
        isInQuasiOmniMode = hasQuasiOmniMode;
    }

    CustomAntennaModel(
        const AntennaPatternDatabase& anAntennaPatternDatabase,
        const string& antennaModelName,
        const double& initQuasiOmniModeGainDbi = DBL_MAX)
        :
        antennaPatternPtr(
            anAntennaPatternDatabase.GetAntennaPattern(antennaModelName)),
        hasQuasiOmniMode(initQuasiOmniModeGainDbi != DBL_MAX),
        quasiOmniModeGainDbi(initQuasiOmniModeGainDbi)
    {
        isInQuasiOmniMode = hasQuasiOmniMode;
    }

    virtual void SetAntennaPattern(const shared_ptr<AntennaPattern>& newAntennaPatternPtr)
        { (*this).antennaPatternPtr = newAntennaPatternPtr; }

    virtual bool IsOmniDirectional() const override { return false; }
    virtual double GetOmniGainDbi() const override { assert(false); abort(); return 0.0; }

    virtual double GainInDbForThisDirection(
        const double& azimuthFromBoresightDegrees,
        const double& elevationFromBoresightDegrees,
        const double& currentAntennaRotation = 0.0) const override;

    virtual bool SupportsQuasiOmniMode() const override { return hasQuasiOmniMode; }
    virtual void SwitchToQuasiOmniMode() override;
    virtual void SwitchToDirectionalMode() override { isInQuasiOmniMode = false; }

private:
    shared_ptr<AntennaPattern> antennaPatternPtr;
    //const string antennaModelName;
    bool hasQuasiOmniMode;
    double quasiOmniModeGainDbi;
    bool isInQuasiOmniMode;

};//CustomAntennaModel//


inline
double CustomAntennaModel::GainInDbForThisDirection(
    const double& azimuthFromBoresightDegrees,
    const double& elevationFromBoresightDegrees,
    const double& currentAntennaRotation) const
{
    assert(currentAntennaRotation == 0.0);
    assert(azimuthFromBoresightDegrees >= -180.0);
    assert(azimuthFromBoresightDegrees <= 180.0);
    assert(elevationFromBoresightDegrees >= -90.0);
    assert(elevationFromBoresightDegrees <= 90.0);

    if (isInQuasiOmniMode) {
        return (quasiOmniModeGainDbi);
    }
    else {
        assert(antennaPatternPtr != nullptr);

        return (
            antennaPatternPtr->GetGainDbi(
                azimuthFromBoresightDegrees,
                elevationFromBoresightDegrees));
    }//if//

}//GainInDbForThisDirection//


inline
void CustomAntennaModel::SwitchToQuasiOmniMode() {
    if (!hasQuasiOmniMode) {
        cerr << "Error: Tried to switch to Quasi-Omni Mode in CustomAntennaModel but the" << endl;
        cerr << "   Quasi-Omni gain has not been set (\"antenna-model-quasi-omni-mode-gain-dbi\")." << endl;
        exit(1);
    }//if//
    isInQuasiOmniMode = true;

}//SwitchToQuasiOmniMode//




shared_ptr<AntennaModel> CreateAntennaModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceOrInstanceIdType& interfaceId,
    const AntennaPatternDatabase& anAntennaPatternDatabase);


//=============================================================================

class SignalLossCache {
public:
    typedef ObjectMobilityModel::MobilityObjectIdType MobilityObjectIdType;

    SignalLossCache() : isSymmetricSignalLoss(true) {}

    void CacheTheValue(
        const MobilityObjectIdType objectId1,
        const MobilityObjectIdType objectId2,
        const unsigned int channelNumber,
        const double& lossValueToCacheDb,
        const TimeType& propagationDelay,
        const TimeType& exPIrationTime);

    void GetCachedValue(
        const MobilityObjectIdType objectId1,
        const MobilityObjectIdType objectId2,
        const unsigned int channelNumber,
        const TimeType& currentTime,
        bool& cachedValueFound,
        double& cachedLossValueDb,
        TimeType& propagationDelay);

    void ClearCacheInformationFor(const MobilityObjectIdType& objectId);
    void InvalidateCachedInformationFor(const MobilityObjectIdType& objectId);

    void DisableSymmetricSignalLossCache() { isSymmetricSignalLoss = false; }

private:
    struct KeyType {
        MobilityObjectIdType lowId;
        MobilityObjectIdType highId;
        unsigned int channelNumber;

        KeyType() { }
        KeyType(
            MobilityObjectIdType initLowId,
            MobilityObjectIdType initHighId,
            unsigned int initChannelNumber)
            :
            lowId(initLowId),
            highId(initHighId),
            channelNumber(initChannelNumber)
            { }

        bool operator<(const KeyType& right) const
        {
            return ((this->channelNumber < right.channelNumber) ||
                ((this->channelNumber == right.channelNumber) && (this->lowId < right.lowId)) ||
                ((this->channelNumber == right.channelNumber) &&
                (this->lowId == right.lowId) &&
                (this->highId < right.highId)));
        }

    };//KeyType//

    struct CacheRecordType {
        TimeType expirationTime;
        double lossDb;
        TimeType propagationDelay;

        CacheRecordType() { }
        CacheRecordType(
            const TimeType& initExpirationTime,
            const double& initLossDb,
            const TimeType& initPropagationDelay)
            : expirationTime(initExpirationTime), lossDb(initLossDb),
              propagationDelay(initPropagationDelay) {}

    };//CacheRecordType//

    bool isSymmetricSignalLoss;

    // Should be matrix for ultimate speed.

    map<KeyType, CacheRecordType> cache;

};//SignalLossCache//


inline
void SignalLossCache::CacheTheValue(
    const MobilityObjectIdType objectId1,
    const MobilityObjectIdType objectId2,
    const unsigned int channelNumber,
    const double& lossValueToCacheDb,
    const TimeType& propagationDelay,
    const TimeType& expirationTime)
{
    typedef map<KeyType, CacheRecordType>::iterator IterType;

    KeyType key;

    if (isSymmetricSignalLoss) {
        if (objectId1 < objectId2) {
            key = KeyType(objectId1, objectId2, channelNumber);
        }
        else {
            key = KeyType(objectId2, objectId1, channelNumber);

        }//if//
    }
    else {
        key = KeyType(objectId1, objectId2, channelNumber);
    }//if//

    IterType foundIter = cache.find(key);
    if (foundIter == cache.end()) {
        cache.insert(make_pair(key, CacheRecordType(expirationTime, lossValueToCacheDb, propagationDelay)));
    }
    else {
        foundIter->second.expirationTime = expirationTime;
        foundIter->second.lossDb = lossValueToCacheDb;
        foundIter->second.propagationDelay = propagationDelay;

    }//if//

}//CacheTheValue//

inline
void SignalLossCache::GetCachedValue(
    MobilityObjectIdType objectId1,
    MobilityObjectIdType objectId2,
    const unsigned int channelNumber,
    const TimeType& currentTime,
    bool& cachedValueWasFound,
    double& cachedLossValueDb,
    TimeType& cachedPropagationDelay)
{
    typedef map<KeyType, CacheRecordType>::iterator IterType;

    KeyType key;

    if (isSymmetricSignalLoss) {
        if (objectId1 < objectId2) {
            key = KeyType(objectId1, objectId2, channelNumber);
        }
        else {
            key = KeyType(objectId2, objectId1, channelNumber);

        }//if//
    }
    else {
        key = KeyType(objectId1, objectId2, channelNumber);
    }//if//

    cachedValueWasFound = false;

    IterType foundIter = cache.find(key);
    if ((foundIter != cache.end()) &&
        (foundIter->second.expirationTime > currentTime)) {

        cachedLossValueDb = foundIter->second.lossDb;
        cachedPropagationDelay = foundIter->second.propagationDelay;
        cachedValueWasFound = true;

    }//if//

}//GetCachedValue//


inline
void SignalLossCache::ClearCacheInformationFor(const MobilityObjectIdType& objectId)
{
    typedef map<KeyType, CacheRecordType>::iterator IterType;

    IterType iter = cache.begin();

    while (iter != cache.end()) {
        if ((iter->first.lowId == objectId) || (iter->first.highId == objectId)) {
            IterType deleteIter = iter;
            ++iter;
            cache.erase(deleteIter);
        }
        else {
          ++iter;
        }//if//
     }//while//

}//ClearCacheInformationFor//



inline
void SignalLossCache::InvalidateCachedInformationFor(const MobilityObjectIdType& objectId)
{
    typedef map<KeyType, CacheRecordType>::iterator IterType;

    IterType iter = cache.begin();

    while (iter != cache.end()) {
        if ((iter->first.lowId == objectId) || (iter->first.highId == objectId)) {
            iter->second.expirationTime = ZERO_TIME;
        }//if//
        ++iter;
     }//while//

}//InvalidateCacheInformationFor//



//=============================================================================

class SimplePropagationLossCalculationModel {
public:
    SimplePropagationLossCalculationModel(
        const double& carrierFrequencyMhz,
        const double& maximumPropagationDistanceMeters = DBL_MAX,
        const bool propagationDelayIsEnabled = false,
        const int numberDataParallelThreads = 0);

    virtual ~SimplePropagationLossCalculationModel();

    double GetCarrierFrequencyMhz() const { return carrierFrequencyMhz; }

    virtual double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        const double& xyDistanceSquaredMeters) const = 0;

    // This version is for InSite (needs the thread/"model set" ID).
    virtual double CalculatePropagationLossDbParallelVersion(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        const double& xyDistanceSquaredMeters,
        const int calculationThreadId) const
    {
        return CalculatePropagationLossDb(txAntennaPosition, rxAntennaPosition, xyDistanceSquaredMeters);
    }

    // for asymmetric propagation calculation model loss value caching.
    virtual bool PropagationLossIsSymmetricValue() const { return true; }

    // for InSite MIMO calculation
    virtual bool SupportMultipointCalculation() const { return false; }

    virtual void CacheMultipointPropagationLossDb() {}
        //Was// const TimeType& currentTime,
        //Was// const ObjectMobilityPosition& txAntennaPosition,
        //Was// const vector<ObjectMobilityPosition>& rxAntennaPositions) {}

    bool IsCloserThanMaxPropagationDistance(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition) const;

    void CalculateOrRetrieveTotalLossDbAndPropDelay(
        SignalLossCache& aSignalLossCache,
        const TimeType& currentTime,
        const unsigned int channelNumber,
        const NodeIdType& txNodeId,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntenna,
        const NodeIdType& rxNodeId,
        ObjectMobilityModel& rxMobilityModel,
        const AntennaModel& rxAntenna,
        double& totalLossDb,
        TimeType& propagationDelay);

    double CalculateTotalAntennaGainDbi(
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txNodeAntenna,
        const ObjectMobilityPosition& rxAntennaPosition,
        const AntennaModel& rxNodeAntenna) const;

    double CalculateAntennaGainDbi(
        const ObjectMobilityPosition& antennaPosition,
        const AntennaModel& nodeAntenna,
        const double destX,
        const double destY,
        const double destZ) const;

    virtual void CalculatePropagationPathInformation(
        const PropagationInformationType& informationType,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const ObjectMobilityPosition& rxAntennaPosition,
        const AntennaModel& rxAntennaModel,
        PropagationStatisticsType& propagationStatistics) const;

    static const unsigned int MAIN_THREAD_ID = UINT_MAX;

    void CalculateTotalLossDbAndPropDelay(
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const ObjectMobilityPosition& rxAntennaPosition,
        const AntennaModel& rxAntennaModel,
        double& totalLossDb,
        TimeType& propagationDelay,
        const unsigned int threadId = MAIN_THREAD_ID) const;


    // Parallel stuff:
    void ClearParallelCalculationSet();

    void AddToParallelCalculationSet(
       const ObjectMobilityPosition& txAntennaPosition,
       const AntennaModel* txAntennaModelPtr,
       const unsigned int& rxNodeIndex,
       const ObjectMobilityPosition& rxAntennaPosition,
       const shared_ptr<AntennaModel>& rxAntennaModelPtr);

    void CalculateLossesDbAndPropDelaysInParallel();

    unsigned int GetNumberOfParallelCalculations() const;

    double GetTotalLossDb(const unsigned int jobIndex) const;
    TimeType GetPropagationDelay(const unsigned int JobIndex) const;

    NodeIdType GetTxNodeId(const unsigned int jobIndex) const;
    ObjectMobilityPosition& GetTxAntennaPosition(const unsigned int jobIndex) const;

    unsigned int GetRxNodeIndex(const unsigned int jobIndex) const;
    ObjectMobilityPosition& GetRxAntennaPosition(const unsigned int jobIndex) const;

    // Ray path trace stuff:
    virtual bool RayPathTraceIsEnabled() const;
    virtual double CalculateTotalLossDbWithRayPathTrace(
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txNodeAntenna,
        const ObjectMobilityPosition& rxAntennaPosition,
        const AntennaModel& rxNodeAntenna) const;

protected:
    double carrierFrequencyMhz;

private:
    class Implementation;
    auto_ptr<Implementation> implPtr;

    double maximumPropagationDistanceSquaredMeters;
    bool propagationDelayIsEnabled;

    // Disable:
    SimplePropagationLossCalculationModel(const SimplePropagationLossCalculationModel&);
    void operator=(const SimplePropagationLossCalculationModel&);

};//SimplePropagationLossCalculationModel//



inline
bool SimplePropagationLossCalculationModel::IsCloserThanMaxPropagationDistance(
    const ObjectMobilityPosition& txAntennaPosition,
    const ObjectMobilityPosition& rxAntennaPosition) const
{
    double txPosX = txAntennaPosition.X_PositionMeters();
    double txPosY = txAntennaPosition.Y_PositionMeters();
    double rxPosX = rxAntennaPosition.X_PositionMeters();
    double rxPosY = rxAntennaPosition.Y_PositionMeters();
    double txPosZ = txAntennaPosition.HeightFromGroundMeters();
    double rxPosZ = rxAntennaPosition.HeightFromGroundMeters();

    double deltaX = rxPosX - txPosX;
    double deltaY = rxPosY - txPosY;
    double deltaZ = rxPosZ - txPosZ;

    double xyzDistanceSquared = deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ;

    return (xyzDistanceSquared < maximumPropagationDistanceSquaredMeters);
}

inline
void SimplePropagationLossCalculationModel::CalculateTotalLossDbAndPropDelay(
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txNodeAntenna,
    const ObjectMobilityPosition& rxAntennaPosition,
    const AntennaModel& rxNodeAntenna,
    double& totalLossDb,
    TimeType& propagationDelay,
    const unsigned int threadNumber) const
{
    double txPosX = txAntennaPosition.X_PositionMeters();
    double txPosY = txAntennaPosition.Y_PositionMeters();
    double rxPosX = rxAntennaPosition.X_PositionMeters();
    double rxPosY = rxAntennaPosition.Y_PositionMeters();
    double txPosZ = txAntennaPosition.HeightFromGroundMeters();
    double rxPosZ = rxAntennaPosition.HeightFromGroundMeters();

    double deltaX = rxPosX - txPosX;
    double deltaY = rxPosY - txPosY;
    double deltaZ = rxPosZ - txPosZ;

    double xyDistanceSquared = deltaX*deltaX + deltaY*deltaY;
    double xyzDistanceSquared = xyDistanceSquared + deltaZ*deltaZ;

    if (xyzDistanceSquared >= maximumPropagationDistanceSquaredMeters) {

        totalLossDb = InfinitePathlossDb;
        propagationDelay = INFINITE_TIME;
        return;

    }//if//

    if (propagationDelayIsEnabled) {
        propagationDelay = TimeType(SECOND * (sqrt(xyzDistanceSquared) * (1/SPEED_OF_LIGHT_METERS_PER_SECOND)));
    }
    else {
        propagationDelay = 0;

    }//if//

    if (RayPathTraceIsEnabled()) {
        totalLossDb = CalculateTotalLossDbWithRayPathTrace(
            txAntennaPosition,
            txNodeAntenna,
            rxAntennaPosition,
            rxNodeAntenna);

    } else {
        const double totalAntennaGainDb =
            CalculateTotalAntennaGainDbi(
                txAntennaPosition,
                txNodeAntenna,
                rxAntennaPosition,
                rxNodeAntenna);

        double propagationLossDb;

        if (threadNumber == MAIN_THREAD_ID) {

            propagationLossDb =
                CalculatePropagationLossDb(
                    txAntennaPosition,
                    rxAntennaPosition,
                    xyDistanceSquared);
        }
        else {
            propagationLossDb =
                CalculatePropagationLossDbParallelVersion(
                    txAntennaPosition,
                    rxAntennaPosition,
                    xyDistanceSquared,
                    threadNumber);
        }//if//

        assert(propagationLossDb >= 0.0);

        totalLossDb = propagationLossDb - totalAntennaGainDb;
    }

}//CalculateTotalLossDbAndPropDelay//

inline
bool SimplePropagationLossCalculationModel::RayPathTraceIsEnabled() const
{
    return false;

}//RayPathTraceIsEnabled//

inline
double SimplePropagationLossCalculationModel::CalculateTotalLossDbWithRayPathTrace(
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txNodeAntenna,
    const ObjectMobilityPosition& rxAntennaPosition,
    const AntennaModel& rxNodeAntenna) const
{
    // Shouldn't be called
    assert(false);
    abort();
    return 0.0;

}//CalculateTotalLossDbWithRayPathTrace//

inline
double SimplePropagationLossCalculationModel::CalculateTotalAntennaGainDbi(
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txNodeAntenna,
    const ObjectMobilityPosition& rxAntennaPosition,
    const AntennaModel& rxNodeAntenna) const
{
    // Tx->Rx direct
    const bool txAntennaIsOmni = txNodeAntenna.IsOmniDirectional();
    const bool rxAntennaIsOmni = rxNodeAntenna.IsOmniDirectional();
    if (txAntennaIsOmni && rxAntennaIsOmni) {
        return (txNodeAntenna.GetOmniGainDbi() + rxNodeAntenna.GetOmniGainDbi());
    }//if//

    const double txAntennaGain = (*this).CalculateAntennaGainDbi(
        txAntennaPosition,
        txNodeAntenna,
        rxAntennaPosition.X_PositionMeters(),
        rxAntennaPosition.Y_PositionMeters(),
        rxAntennaPosition.HeightFromGroundMeters());

    const double rxAntennaGain = (*this).CalculateAntennaGainDbi(
        rxAntennaPosition,
        rxNodeAntenna,
        txAntennaPosition.X_PositionMeters(),
        txAntennaPosition.Y_PositionMeters(),
        txAntennaPosition.HeightFromGroundMeters());

    return txAntennaGain + rxAntennaGain;

}//CalculateTotalAntennaGainDbi//





class WorldMatrix {

    // Branched from scensimg_gis.h: RotationMatrix

public:

    struct Vector3d {
        double x;
        double y;
        double z;

        Vector3d() : x(0.0), y(0.0), z(0.0) {}

        Vector3d(const double initX, const double initY, const double initZ)
            :
            x(initX), y(initY), z(initZ)
        {}

        double Dot(const Vector3d& v) const { return (x*v.x) + (y*v.y) + (z*v.z); }
    };//Vector3d//

    WorldMatrix(const double xDegrees, const double yDegrees, const double zDegrees) {

        using std::cos;
        using std::sin;

        const double xRadians = xDegrees*RADIANS_PER_DEGREE;
        const double yRadians = yDegrees*RADIANS_PER_DEGREE;
        const double zRadians = zDegrees*RADIANS_PER_DEGREE;

        const double cosX = cos(xRadians);
        const double sinX = sin(xRadians);
        const double cosY = cos(yRadians);
        const double sinY = sin(yRadians);
        const double cosZ = cos(zRadians);
        const double sinZ = sin(zRadians);

        rotationVector[0] = Vector3d(cosY*cosX, -(cosY*sinX), sinY);
        rotationVector[1] = Vector3d((sinZ*sinY*cosX) + (cosZ*sinX), -(sinZ*sinY*sinX) + (cosZ*cosX), -(sinZ*cosY));
        rotationVector[2] = Vector3d(-(cosZ*sinY*cosX) + (sinZ*sinX), (cosZ*sinY*sinX) + (sinZ*cosX), (cosZ*cosY));
    }

    Vector3d MapVector(const Vector3d& v) const { return Vector3d(rotationVector[0].Dot(v), rotationVector[1].Dot(v), rotationVector[2].Dot(v)); }

private:
    Vector3d rotationVector[3];

};//WorldMatrix//


inline
double SimplePropagationLossCalculationModel::CalculateAntennaGainDbi(
    const ObjectMobilityPosition& antennaPosition,
    const AntennaModel& nodeAntenna,
    const double destX,
    const double destY,
    const double destZ) const
{
    const bool antennaIsOmni = nodeAntenna.IsOmniDirectional();
    if (antennaIsOmni) {
        return nodeAntenna.GetOmniGainDbi();
    }//if//

    const double srcX = antennaPosition.X_PositionMeters();
    const double srcY = antennaPosition.Y_PositionMeters();
    const double srcZ = antennaPosition.HeightFromGroundMeters();

    const double deltaX = destX - srcX;
    const double deltaY = destY - srcY;
    const double deltaZ = destZ - srcZ;

    if ((deltaX == 0.0) &&
        (deltaY == 0.0) &&
        (deltaZ == 0.0)) {

        // Return gain 0 for perfect matching position regardless of the antenna model.
        return 0.0;
    }//if//

    const WorldMatrix boresightWorldMatrix(
        antennaPosition.AttitudeAzimuthFromNorthClockwiseDegrees(),
        0.0,// = Antenna Rotation on boresight axis
        -antennaPosition.AttitudeElevationFromHorizonDegrees());

    const WorldMatrix::Vector3d destVector(deltaX, deltaY, deltaZ);

    const WorldMatrix::Vector3d destFromBoresight =
        boresightWorldMatrix.MapVector(destVector);

    const double xyDistanceSquared =
        (destFromBoresight.x*destFromBoresight.x) +
        (destFromBoresight.y*destFromBoresight.y);

    double elevationFromBoresight =
        NormalizeAngleToNeg180To180(
            (180.0 / PI) * atan2(destFromBoresight.z, sqrt(xyDistanceSquared)));

    double azimuthFromBoresight =
        NormalizeAngleToNeg180To180(
            -(180.0 / PI) * atan2(destFromBoresight.y, destFromBoresight.x) + 90.0);

    NormalizeAzimuthAndElevation(azimuthFromBoresight, elevationFromBoresight);

    const double antennaGain =
        nodeAntenna.GainInDbForThisDirection(
            azimuthFromBoresight,
            elevationFromBoresight);

    return antennaGain;

}//CalculateAntennaGainDbi//



inline
void SimplePropagationLossCalculationModel::CalculatePropagationPathInformation(
    const PropagationInformationType& informationType,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntennaModel,
    const ObjectMobilityPosition& rxAntennaPosition,
    const AntennaModel& rxAntennaModel,
    PropagationStatisticsType& propagationStatistics) const
{
    TimeType notUsed;

    (*this).CalculateTotalLossDbAndPropDelay(
        txAntennaPosition,
        txAntennaModel,
        rxAntennaPosition,
        rxAntennaModel,
        propagationStatistics.totalLossValueDb,
        notUsed);
}



inline
void SimplePropagationLossCalculationModel::CalculateOrRetrieveTotalLossDbAndPropDelay(
    SignalLossCache& aSignalLossCache,
    const TimeType& currentTime,
    const unsigned int channelNumber,
    const NodeIdType& txNodeId,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntenna,
    const NodeIdType& rxNodeId,
    ObjectMobilityModel& rxMobilityModel,
    const AntennaModel& rxAntenna,
    double& totalLossDb,
    TimeType& propagationDelay)
{
    bool cachedValueFound = false;
    totalLossDb = 0.0;

    if (txNodeId == rxNodeId) {
        propagationDelay = ZERO_TIME;
        return;
    }//if//

    assert(((!rxAntenna.SupportsQuasiOmniMode()) || (!rxAntenna.IsInQuasiOmniMode()) ||
           (rxMobilityModel.GetRelativeAttitudeAzimuth() == 0.0)) &&
           "Relative Azimuth was not set to 0 in QuasiOmni Mode");

    aSignalLossCache.GetCachedValue(
        txNodeId, rxNodeId, channelNumber, currentTime,
        cachedValueFound, totalLossDb, propagationDelay);

    if (!cachedValueFound) {

        ObjectMobilityPosition rxAntennaPosition;

        rxMobilityModel.GetPositionForTime(currentTime, rxAntennaPosition);

        CalculateTotalLossDbAndPropDelay(
            txAntennaPosition, txAntenna, rxAntennaPosition, rxAntenna,
            totalLossDb, propagationDelay);

        aSignalLossCache.CacheTheValue(
            txNodeId,
            rxNodeId,
            channelNumber,
            totalLossDb,
            propagationDelay,
            std::min(
                txAntennaPosition.EarliestNextMoveTime(),
                rxAntennaPosition.EarliestNextMoveTime()));

    }//if//

}//CalculateOrRetrieveTotalLossDbAndPropDelay//



//=====================================================================

class FreeSpacePropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    FreeSpacePropagationLossCalculationModel(
        const double& initCarrierFrequencyMhz,
        const double& initMaximumPropagationDistanceMeters,
        const bool initPropagationDelayIsEnabled,
        const int numberDataParallelThreads)
        :
        SimplePropagationLossCalculationModel(
            initCarrierFrequencyMhz,
            initMaximumPropagationDistanceMeters,
            initPropagationDelayIsEnabled,
            numberDataParallelThreads),
        carrierFrequencyMhzLog10(log10(initCarrierFrequencyMhz))
    {}

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txPosition,
        const ObjectMobilityPosition& rxPosition,
        const double& xyDistanceSquaredMeters) const;

private:
    double carrierFrequencyMhzLog10;

};//FreeSpacePropagationLossCalculationModel//


inline
double FreeSpacePropagationLossCalculationModel::CalculatePropagationLossDb(
    const ObjectMobilityPosition& txPosition,
    const ObjectMobilityPosition& rxPosition,
    const double& xyDistanceSquaredMeters) const
{

    const double txHeight = txPosition.HeightFromGroundMeters();
    const double rxHeight = rxPosition.HeightFromGroundMeters();

    double heightDifferenceSquared = 0.0;
    if (txHeight != rxHeight) {
      heightDifferenceSquared = (txHeight - rxHeight) * (txHeight - rxHeight);
    }//if//

    const double xyzDistanceSquaredMeters = xyDistanceSquaredMeters + heightDifferenceSquared;

    const double radiusKilometersLog10 =
        (0.5 * log10(xyzDistanceSquaredMeters)) - 3.0;

    const double freeSpaceLossDb = 32.4 + (20 * radiusKilometersLog10) + (20 * carrierFrequencyMhzLog10);

    return (std::max(0.0, freeSpaceLossDb));

}//CalculatePropagationLossDb//


//=====================================================================


class TwoRayGroundPropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    TwoRayGroundPropagationLossCalculationModel(
        const double& initCarrierFrequencyMhz,
        const double& initMaximumPropagationDistanceMeters,
        const bool initPropagationDelayIsEnabled,
        const int numberDataParallelThreads)
        :
        SimplePropagationLossCalculationModel(
            initCarrierFrequencyMhz,
            initMaximumPropagationDistanceMeters,
            initPropagationDelayIsEnabled,
            numberDataParallelThreads),
        carrierFrequencyMhzLog10(log10(initCarrierFrequencyMhz))
    {}

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txPosition,
        const ObjectMobilityPosition& rxPosition,
        const double& xyDistanceSquaredMeters) const;

private:
    double carrierFrequencyMhzLog10;

};//TwoRayGroundPropagationLossCalculationModel//


inline
double TwoRayGroundPropagationLossCalculationModel::CalculatePropagationLossDb(
    const ObjectMobilityPosition& txPosition,
    const ObjectMobilityPosition& rxPosition,
    const double& xyDistanceSquaredMeters) const
{
    using std::max;

    // Two Ray ground pathloss with freespace in close.

    const double txHeight = txPosition.HeightFromGroundMeters();
    const double rxHeight = rxPosition.HeightFromGroundMeters();

    assert(((txHeight > 0.0) && (rxHeight > 0.0)));

    double heightDifferenceSquared = 0.0;
    if (txHeight != rxHeight) {
      heightDifferenceSquared = (txHeight - rxHeight) * (txHeight - rxHeight);
    }//if//

    const double xyzDistanceSquaredMeters = xyDistanceSquaredMeters + heightDifferenceSquared;

    const double distanceMetersLog10 = 0.5 * log10(xyDistanceSquaredMeters);
    const double distanceKilometersLog10 = distanceMetersLog10 - 3.0;

    double radiusKilometersLog10 = distanceKilometersLog10;

    if (txHeight != rxHeight) {
        // May need to optimize by precalculating crossover radius.

        radiusKilometersLog10 = (0.5 * log10(xyzDistanceSquaredMeters)) - 3.0;

    }//if//

    const double freeSpaceLossDb = 32.4 + (20 * radiusKilometersLog10) + (20 * carrierFrequencyMhzLog10);

    const double twoRayGroundLossDb =
        (40 * distanceMetersLog10) +
        (-20 * (log10(txHeight) + log10(rxHeight)));

    const double maxLossDb = max(freeSpaceLossDb, twoRayGroundLossDb);

    return (max(0.0, maxLossDb));

}//CalculatePropagationLossDb//


//====================================================================================

class OkumuraHataPropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    OkumuraHataPropagationLossCalculationModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const double& carrierFrequencyMhz,
        const double& maximumPropagationDistanceMeters,
        const bool propagationDelayIsEnabled,
        const int numberDataParallelCalculationThreads,
        const InterfaceOrInstanceIdType& instanceId);

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        const double& xyDistanceSquaredMeters) const;
private:
    double carrierFrequencyMhzLog10;

    double cM;

    enum EnvironmentType {
        UrbanLarge,
        UrbanMediumOrSmall,
        Suburban,
        Rural
    };

    EnvironmentType envType;

};//OkumuraHataPropagationLossCalculationModel//


inline
OkumuraHataPropagationLossCalculationModel::OkumuraHataPropagationLossCalculationModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const double& initCarrierFrequencyMhz,
    const double& initMaximumPropagationDistanceMeters,
    const bool initPropagationDelayIsEnabled,
    const int numberDataParallelCalculationThreads,
    const InterfaceOrInstanceIdType& instanceId)
    :
    SimplePropagationLossCalculationModel(
        initCarrierFrequencyMhz,
        initMaximumPropagationDistanceMeters,
        initPropagationDelayIsEnabled,
        numberDataParallelCalculationThreads),
    carrierFrequencyMhzLog10(log10(initCarrierFrequencyMhz)),
    envType(UrbanLarge)

{

    if (theParameterDatabaseReader.ParameterExists("prop-okumurahata-environment", instanceId)) {
        const string environmentString =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("prop-okumurahata-environment", instanceId));

        if (environmentString == "urban_largecity") {
            envType = UrbanLarge;
        }
        else if (environmentString == "urban_mediumorsmallcity") {
            envType = UrbanMediumOrSmall;
        }
        else if (environmentString == "suburban") {
            envType = Suburban;
        }
        else if (environmentString == "rural") {
            envType = Rural;
        }
        else {
            cerr << "Error: Unknown prop-okumurahata-environment: " << environmentString << endl;
            exit(1);
        }//if//
    }//if//

    switch(envType) {
    case UrbanLarge:
    case UrbanMediumOrSmall:
        cM = 0.0;
        break;
    case Suburban:
        cM = - 2.0 * log10(initCarrierFrequencyMhz / 28.0) * log10(initCarrierFrequencyMhz / 28.0) - 5.4;
        break;
    case Rural:
        cM = (- 4.78 * carrierFrequencyMhzLog10 * carrierFrequencyMhzLog10)
            + (18.33 * carrierFrequencyMhzLog10) - 40.94;
        break;
    default:
        assert(false);
        exit(1);
    }//switch//

}//if//


inline
double OkumuraHataPropagationLossCalculationModel::CalculatePropagationLossDb(
    const ObjectMobilityPosition& txAntennaPosition,
    const ObjectMobilityPosition& rxAntennaPosition,
    const double& xyDistanceSquaredMeters) const
{
    const double txHeight = txAntennaPosition.HeightFromGroundMeters();
    const double rxHeight = rxAntennaPosition.HeightFromGroundMeters();

    double baseAntennaHeight = txHeight;
    double mobileAntennaHeight = rxHeight;

    if (baseAntennaHeight < mobileAntennaHeight) {
        std::swap(baseAntennaHeight, mobileAntennaHeight);
    }//if//

    const double distanceKilometersLog10 = ((0.5 * log10(xyDistanceSquaredMeters)) - 3.0);
    const double baseAntennaHeightLog10 = log10(baseAntennaHeight);

    // Recommend:
    // Frequency: 150-1500MHz
    // Base station height: 30-200m
    // Mobile height: 1-10m
    // Distance: 1-20km

    // Original equations (for reference)
    //
    //double mobileStationAntennaHeightCorrection =
    //(UrbanMediumOrSmall, Suburban, Rural)
    //    0.8 +
    //    ((1.1 * carrierFrequencyMhzLog10 - 0.7) * mobileAntennaHeight)
    //    - (1.56 * carrierFrequencyMhzLog10);
    //(UrbanLarge; f <= 300MHz)
    //    8.29 * (log10(1.54 * mobileAntennaHeight)) * (log10(1.54 * mobileAntennaHeight))
    //    - 1.1;
    //(UrbanLarge; 300MHz < f)
    //    3.2 * (log10(11.75 * mobileAntennaHeight)) * (log10(11.75 * mobileAntennaHeight))
    //    - 4.97;
    //
    //double cM =
    //(UrbanLarge, UrbanMediumOrSmall)
    //    0.0
    //(Suburban)
    //    - 2.0 * log10(initCarrierFrequencyMhz / 28.0) * log10(initCarrierFrequencyMhz / 28.0)
    //    - 5.4
    //(Rural)
    //    - 4.78 * carrierFrequencyMhzLog10 * carrierFrequencyMhzLog10
    //    + 18.33 * carrierFrequencyMhzLog10 - 40.94
    //
    //double pathLossDb =
    //    69.55 +
    //    (26.16 * carrierFrequencyMhzLog10) +
    //    (-13.82 * baseAntennaHeightLog10) +
    //    (-mobileAntennaHeightCorrection) +
    //    ((44.9 - (6.55 * baseAntennaHeightLog10)) * distanceKilometersLog10) +
    //    cM

    double mobileAntennaHeightCorrection;

    switch(envType) {
    case UrbanLarge:

        if (carrierFrequencyMhz <= 300.0) {
            mobileAntennaHeightCorrection =
                8.29 * (log10(1.54 * mobileAntennaHeight)) * (log10(1.54 * mobileAntennaHeight)) - 1.1;
        }
        else {
            mobileAntennaHeightCorrection =
                3.2 * (log10(11.75 * mobileAntennaHeight)) * (log10(11.75 * mobileAntennaHeight)) - 4.97;
        }//if//

        break;
    case UrbanMediumOrSmall:
    case Suburban:
    case Rural:

        mobileAntennaHeightCorrection =
            0.8 +
            ((1.1 * carrierFrequencyMhzLog10 - 0.7) * mobileAntennaHeight)
            - (1.56 * carrierFrequencyMhzLog10);

        break;

    default:
        assert(false);
        exit(1);
    }//switch//

    const double pathLossDb =
        69.55 +
        (26.16 * carrierFrequencyMhzLog10) +
        (-13.82 * baseAntennaHeightLog10) +
        (-mobileAntennaHeightCorrection) +
        ((44.9 - (6.55 * baseAntennaHeightLog10)) * distanceKilometersLog10) +
        cM;

    return (std::max(0.0, pathLossDb));

}//CalculatePropagationLossDb//


//====================================================================================

class Cost231HataPropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    Cost231HataPropagationLossCalculationModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const double& carrierFrequencyMhz,
        const double& maximumPropagationDistanceMeters,
        const bool propagationDelayIsEnabled,
        const int numberDataParallelCalculationThreads,
        const InterfaceOrInstanceIdType& instanceId);

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        const double& xyDistanceSquaredMeters) const;
private:
    double carrierFrequencyMhzLog10;

    double cM;

};//Cost231HataPropagationModel//


inline
Cost231HataPropagationLossCalculationModel::Cost231HataPropagationLossCalculationModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const double& initCarrierFrequencyMhz,
    const double& initMaximumPropagationDistanceMeters,
    const bool initPropagationDelayIsEnabled,
    const int numberDataParallelCalculationThreads,
    const InterfaceOrInstanceIdType& instanceId)
    :
    SimplePropagationLossCalculationModel(
        initCarrierFrequencyMhz,
        initMaximumPropagationDistanceMeters,
        initPropagationDelayIsEnabled,
        numberDataParallelCalculationThreads),
    carrierFrequencyMhzLog10(log10(initCarrierFrequencyMhz))

{
    const double cSuburban = 0.0;
    const double cMetoropolitan = 3.0;

    cM = cMetoropolitan;//for backward compatibility

    if (theParameterDatabaseReader.ParameterExists("prop-cost231hata-environment", instanceId)) {
        const string environmentString =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("prop-cost231hata-environment", instanceId));

        if (environmentString == "suburban") {
            cM = cSuburban;
        }
        else if (environmentString == "metropolitan") {
            cM = cMetoropolitan;
        }
        else {
            cerr << "Error: Unknown prop-cost231hata-environment: " << environmentString << endl;
            exit(1);
        }//if//
    }//if//

}//if//


inline
double Cost231HataPropagationLossCalculationModel::CalculatePropagationLossDb(
    const ObjectMobilityPosition& txAntennaPosition,
    const ObjectMobilityPosition& rxAntennaPosition,
    const double& xyDistanceSquaredMeters) const
{
    const double txHeight = txAntennaPosition.HeightFromGroundMeters();
    const double rxHeight = rxAntennaPosition.HeightFromGroundMeters();

    double baseAntennaHeight = txHeight;
    double mobileAntennaHeight = rxHeight;

    if (baseAntennaHeight < mobileAntennaHeight) {
        std::swap(baseAntennaHeight, mobileAntennaHeight);
    }//if//

    const double distanceKilometersLog10 = ((0.5 * log10(xyDistanceSquaredMeters)) - 3.0);
    const double baseAntennaHeightLog10 = log10(baseAntennaHeight);

    // Recommend:
    // Frequency: 1500-2000MHz
    // Base station height: 30-200m
    // Mobile height: 1-10m
    // Distance: 1-20km

    // Original equations (for reference)
    //
    //double mobileAntennaHeightCorrection =
    //    0.8 +
    //    ((1.1 * carrierFrequencyMhzLog10 - 0.7) * mobileAntennaHeight)
    //    - (1.56 * carrierFrequencyMhzLog10);
    //
    //double cM = 0.0(Suburban)
    //            3.0(Metoropolitan)
    //
    //double pathLossDb =
    //    46.3 +
    //    (33.9 * carrierFrequencyMhzLog10) +
    //    (-13.82 * baseAntennaHeightLog10) +
    //    (-mobileAntennaHeightCorrection) +
    //    ((44.9 - (6.55 * baseAntennaHeightLog10)) * distanceKilometersLog10) +
    //    cM

    //
    // Alternate equation that just combines and optimizes the two equations.
    //

    const double pathLossDb =
        ((44.9 - (6.55 * baseAntennaHeightLog10)) * distanceKilometersLog10) +
        45.5 + ((35.46 - (1.1 * mobileAntennaHeight)) * (carrierFrequencyMhzLog10)) -
        (13.82 * baseAntennaHeightLog10) + (0.7 * mobileAntennaHeight) + cM;

    return (std::max(0.0, pathLossDb));

}//CalculatePropagationLossDb//

//=====================================================================

class SingleThreadPropagationLossTraceModel: public SimplePropagationLossCalculationModel {
public:
    SingleThreadPropagationLossTraceModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const double& initCarrierFrequencyMhz,
        const bool initPropagationDelayIsEnabled,
        const int numberDataParallelThreads,
        const InterfaceOrInstanceIdType& initInstanceId,
        const shared_ptr<SimplePropagationLossCalculationModel>& initDefaultkPropagationCalculationModelPtr);

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txPosition,
        const ObjectMobilityPosition& rxPosition,
        const double& xyDistanceSquaredMeters) const;

    double CalculatePropagationLossDbParallelVersion(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        const double& xyDistanceSquaredMeters,
        const int calculationThreadId) const {
        assert(false && "use single thread for trace-base propagation");
        return 0.0;
    }

private:

    struct NodeKeyType {
        NodeIdType lowNodeId;
        NodeIdType highNodeId;

        NodeKeyType() {}
        NodeKeyType(
            const NodeIdType& nodeId1,
            const NodeIdType& nodeId2)
            :
            lowNodeId(std::min(nodeId1, nodeId2)),
            highNodeId(std::max(nodeId1, nodeId2))
        {}

        bool operator<(const NodeKeyType& right) const {
            return (lowNodeId < right.lowNodeId ||
                    (lowNodeId == right.lowNodeId &&
                     highNodeId < right.highNodeId));
        }
    };

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;
    shared_ptr<SimplePropagationLossCalculationModel> defaultkPropagationCalculationModelPtr;

    string traceFileName;
    mutable std::streampos currentFilePos;
    bool propagationTraceIsDependsOnNodeId;
    int versionNumber;

    mutable TimeType lossValueChangeTimeStep;
    mutable TimeType lastLossValueUpdatedTime;
    mutable map<NodeKeyType, double> lossCache;

    struct AreaInfo {
        double traceAreaBaseX;
        double traceAreaBaseY;
        double traceAreaBaseZ;
        double traceAreaHorizontalLength;
        double traceAreaVerticalLength;
        double traceAreaHeight;

        double horizontalMeshLength;
        double verticalMeshLength;
        double meshHeight;

        int32_t numberHorizontalMesh;
        int32_t numberVerticalMesh;
        int32_t numberZMesh;

        vector<double> lossValues;

        AreaInfo()
            :
            traceAreaBaseX(0),
            traceAreaBaseY(0),
            traceAreaBaseZ(0),
            traceAreaHorizontalLength(0),
            traceAreaVerticalLength(0),
            traceAreaHeight(0),
            horizontalMeshLength(0),
            verticalMeshLength(0),
            meshHeight(0),
            numberHorizontalMesh(0),
            numberVerticalMesh(0),
            numberZMesh(0)
        {}
    };

    mutable AreaInfo areaInfo;

    void ReadHeaderLine(ifstream& inStream, string& aLine);

    void ReadLossValuesFor(const TimeType& time) const;
    void ReadLossValuesVer1(const TimeType& time) const;
    void ReadLossValuesVer2(const TimeType& time) const;
    void ReadMeshLossValuesVer1(const TimeType& time) const;

    double GetNodeIdBasedPropagationLossDb(
        const ObjectMobilityPosition& txPosition,
        const ObjectMobilityPosition& rxPosition,
        const double& xyDistanceSquaredMeters) const;

    double GetMeshBasedPropagationLossDb(
        const ObjectMobilityPosition& txPosition,
        const ObjectMobilityPosition& rxPosition,
        const double& xyDistanceSquaredMeters) const;

    void CalculateMeshIndex(
        const ObjectMobilityPosition& mobilityPosition,
        int& meshIndexX,
        int& meshIndexY,
        int& meshIndexZ) const;

};//SingleThreadPropagationLossTraceModel//


class GisSubsystem;

//=====================================================================

// Actually Motley and Keenan (modified) model.

class IndoorPropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    IndoorPropagationLossCalculationModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const double& initCarrierFrequencyMhz,
        const double& initMaximumPropagationDistanceMeters,
        const bool initPropagationDelayIsEnabled,
        const int numberDataParallelThreads,
        const InterfaceOrInstanceIdType& initInstanceId);

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txPosition,
        const ObjectMobilityPosition& rxPosition,
        const double& xyDistanceSquaredMeters) const;

    void CalculatePropagationPathInformation(
        const PropagationInformationType& informationType,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const ObjectMobilityPosition& rxAntennaPosition,
        const AntennaModel& rxAntennaModel,
        PropagationStatisticsType& propagationStatistics) const;

private:
    shared_ptr<SimplePropagationLossCalculationModel> freespacePropagationCalculationModelPtr;
    shared_ptr<GisSubsystem> theGisSubsystemPtr;

    const double indoorBreakpointMeters;

    double CalculateLinearAttenuation(const double distanceMeters) const;

};//IndoorPropagationLossCalculationModel//


//=====================================================================


class TgAxIndoorPropLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    TgAxIndoorPropLossCalculationModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const double& initCarrierFrequencyMhz,
        const double& initMaximumPropagationDistanceMeters,
        const bool initPropagationDelayIsEnabled,
        const int numberDataParallelThreads,
        const InterfaceOrInstanceIdType& initInstanceId);

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txPosition,
        const ObjectMobilityPosition& rxPosition,
        const double& xyDistanceSquaredMeters) const;

    void CalculatePropagationPathInformation(
        const PropagationInformationType& informationType,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const ObjectMobilityPosition& rxAntennaPosition,
        const AntennaModel& rxAntennaModel,
        PropagationStatisticsType& propagationStatistics) const;

private:
    double floorAttenuationDb;
    double wallAttenuationDb;

    double breakpointMeters;
    double log10BreakpointMeters;

    shared_ptr<GisSubsystem> theGisSubsystemPtr;

    double constantFactor;

};//IndoorPropagationLossCalculationModel//


//=====================================================================


class WallCountPropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    WallCountPropagationLossCalculationModel(
        const shared_ptr<SimplePropagationLossCalculationModel>& initBaselinePropCalculationModelPtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const double& initCarrierFrequencyMhz,
        const double& initMaximumPropagationDistanceMeters,
        const bool initPropagationDelayIsEnabled,
        const int numberDataParallelThreads,
        const InterfaceOrInstanceIdType& initInstanceId);

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txPosition,
        const ObjectMobilityPosition& rxPosition,
        const double& xyDistanceSquaredMeters) const;

private:
    shared_ptr<SimplePropagationLossCalculationModel> baselinePropCalculationModelPtr;
    shared_ptr<GisSubsystem> theGisSubsystemPtr;

    const double penetrationLossDb;

};//WallCountPropagationLossCalculationModel//


inline
WallCountPropagationLossCalculationModel::WallCountPropagationLossCalculationModel(
    const shared_ptr<SimplePropagationLossCalculationModel>& initBaselinePropCalculationModelPtr,
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
    baselinePropCalculationModelPtr(initBaselinePropCalculationModelPtr),
    theGisSubsystemPtr(initGisSubsystemPtr),
    penetrationLossDb(
        theParameterDatabaseReader.ReadDouble("propwallcount-penetration-loss-db", initInstanceId))
{
}


//=====================================================================


class TwoTierPropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    TwoTierPropagationLossCalculationModel(
        const shared_ptr<SimplePropagationLossCalculationModel>& initPrimaryPropModelPtr,
        const shared_ptr<SimplePropagationLossCalculationModel>& initSecondaryPropModelPtr,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const double& initCarrierFrequencyMhz,
        const double& initMaximumPropagationDistanceMeters,
        const bool initPropagationDelayIsEnabled,
        const int numberDataParallelThreads,
        const InterfaceOrInstanceIdType& initInstanceId);

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txPosition,
        const ObjectMobilityPosition& rxPosition,
        const double& xyDistanceSquaredMeters) const;

    void CalculatePropagationPathInformation(
        const PropagationInformationType& informationType,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const ObjectMobilityPosition& rxAntennaPosition,
        const AntennaModel& rxAntennaModel,
        PropagationStatisticsType& propagationStatistics) const;

private:
    shared_ptr<SimplePropagationLossCalculationModel> primaryPropModelPtr;
    shared_ptr<SimplePropagationLossCalculationModel> secondaryPropModelPtr;

    vector<shared_ptr<SimplePropagationLossCalculationModel> > propModels;

    NodeIdType maxNodeId;

    void AssignPropModelForLinks(
        const string& nodesString,
        const string& linksString);

    unsigned int GetPropModelIndex(
        const NodeIdType firstNodeId, const NodeIdType secondNodeId) const
    { return (firstNodeId * (maxNodeId + 1) + secondNodeId); }

};//TwoTierPropagationLossCalculationModel//

inline
TwoTierPropagationLossCalculationModel::TwoTierPropagationLossCalculationModel(
    const shared_ptr<SimplePropagationLossCalculationModel>& initPrimaryPropModelPtr,
    const shared_ptr<SimplePropagationLossCalculationModel>& initSecondaryPropModelPtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
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
    primaryPropModelPtr(initPrimaryPropModelPtr),
    secondaryPropModelPtr(initSecondaryPropModelPtr)
{

    //load nodes for secondary prop model
    string selectedNodesString = "";
    if (theParameterDatabaseReader.ParameterExists("proptwotier-nodes-running-secondary", initInstanceId)) {
        selectedNodesString = theParameterDatabaseReader.ReadString("proptwotier-nodes-running-secondary", initInstanceId);
    }//if//

    //load links for secondary prop model
    string selectedLinksString = "";
    if (theParameterDatabaseReader.ParameterExists("proptwotier-links-running-secondary", initInstanceId)) {
        selectedLinksString = theParameterDatabaseReader.ReadString("proptwotier-links-running-secondary", initInstanceId);
    }//if//

    set<NodeIdType> setOfNodeIds;
    theParameterDatabaseReader.MakeSetOfAllCommNodeIds(setOfNodeIds);

    (*this).maxNodeId = *setOfNodeIds.rbegin();

    AssignPropModelForLinks(selectedNodesString, selectedLinksString);

}


inline
double TwoTierPropagationLossCalculationModel::CalculatePropagationLossDb(
    const ObjectMobilityPosition& txPosition,
    const ObjectMobilityPosition& rxPosition,
    const double& xyDistanceSquaredMeters) const
{

    const unsigned int propModelIndex =
        GetPropModelIndex(txPosition.GetObjectId(), rxPosition.GetObjectId());

    return propModels[propModelIndex]->CalculatePropagationLossDb(
        txPosition,
        rxPosition,
        xyDistanceSquaredMeters);

}//CalculatePropagationLossDb//


inline
void TwoTierPropagationLossCalculationModel::CalculatePropagationPathInformation(
    const PropagationInformationType& informationType,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntennaModel,
    const ObjectMobilityPosition& rxAntennaPosition,
    const AntennaModel& rxAntennaModel,
    PropagationStatisticsType& propagationStatistics) const
{

    const unsigned int propModelIndex =
        GetPropModelIndex(txAntennaPosition.GetObjectId(),rxAntennaPosition.GetObjectId());

    return propModels[propModelIndex]->CalculatePropagationPathInformation(
        informationType,
        txAntennaPosition,
        txAntennaModel,
        rxAntennaPosition,
        rxAntennaModel,
        propagationStatistics);

}//CalculatePropagationPathInformation//


inline
void TwoTierPropagationLossCalculationModel::AssignPropModelForLinks(
    const string& nodesString,
    const string& linksString)
{

    //set primary prop model
    (*this).propModels.resize(((maxNodeId + 1) * (maxNodeId + 1)), (*this).primaryPropModelPtr);


    //extracted target nodes
    set<NodeIdType> selectedNodeIds;

    size_t currentPosition = 0;
    while (currentPosition < nodesString.length()) {

        size_t endOfSubfieldTerminatorPos = nodesString.find_first_of(",-", currentPosition);

        if (endOfSubfieldTerminatorPos == string::npos) {
            endOfSubfieldTerminatorPos = nodesString.length();
        }//if//

        if ((endOfSubfieldTerminatorPos == nodesString.length()) || (nodesString.at(endOfSubfieldTerminatorPos) == ',')) {

            const NodeIdType extractedNumber =
                atoi(nodesString.substr(currentPosition, (endOfSubfieldTerminatorPos - currentPosition)).c_str());

            if ((extractedNumber < 0) || (maxNodeId < extractedNumber)) {//node id 0 for RF propagation analyzer of GUI
                cerr << "Error: proptwotier-nodes-running-secondary: " << nodesString << endl;
                exit(1);
            }//if//

            selectedNodeIds.insert(extractedNumber);
        }
        else if (nodesString.at(endOfSubfieldTerminatorPos) == '-') {

            //This is a range.

            const size_t dashPosition = endOfSubfieldTerminatorPos;

            endOfSubfieldTerminatorPos = nodesString.find(',', dashPosition);

            if (endOfSubfieldTerminatorPos == string::npos) {
                endOfSubfieldTerminatorPos = nodesString.length();
            }//if//

            const NodeIdType lowerBoundNumber =
                atoi(nodesString.substr(currentPosition, (dashPosition - currentPosition)).c_str());
            if ((lowerBoundNumber < 0) || (maxNodeId < lowerBoundNumber)) {//node id 0 for RF propagation analyzer of GUI
                cerr << "Error: proptwotier-nodes-running-secondary: " << nodesString << endl;
                exit(1);
            }//if//

            const NodeIdType upperBoundNumber =
                atoi(nodesString.substr((dashPosition + 1), (endOfSubfieldTerminatorPos - dashPosition - 1)).c_str());
            if ((upperBoundNumber < 0) || (maxNodeId < upperBoundNumber)) {//node id 0 for RF propagation analyzer of GUI
                cerr << "Error: proptwotier-nodes-running-secondary: " << nodesString << endl;
                exit(1);
            }//if//

            for(NodeIdType nodeId = lowerBoundNumber; (nodeId <= upperBoundNumber); nodeId++) {
                selectedNodeIds.insert(nodeId);
            }//for//

        }//if//

        currentPosition = endOfSubfieldTerminatorPos + 1;

    }//while//

    //set secondary prop model for nodes
    for(set<NodeIdType>::const_iterator iter = selectedNodeIds.begin();
        iter != selectedNodeIds.end(); ++iter) {

        const NodeIdType targetNodeId = *iter;

        for(NodeIdType correspoindingNodeId = 0; correspoindingNodeId <= maxNodeId; correspoindingNodeId++) {

            propModels[GetPropModelIndex(targetNodeId, correspoindingNodeId)] = (*this).secondaryPropModelPtr;
            propModels[GetPropModelIndex(correspoindingNodeId, targetNodeId)] = (*this).secondaryPropModelPtr;

        }//for//

    }//for//


    currentPosition = 0;
    while (currentPosition < linksString.length()) {

        size_t endOfSubfieldTerminatorPos = linksString.find_first_of(",:", currentPosition);

        if (endOfSubfieldTerminatorPos == string::npos) {
            endOfSubfieldTerminatorPos = linksString.length();
        }//if//

        if ((endOfSubfieldTerminatorPos == linksString.length()) || (linksString.at(endOfSubfieldTerminatorPos) == ',')) {

            const NodeIdType extractedNumber =
                atoi(linksString.substr(currentPosition, (endOfSubfieldTerminatorPos - currentPosition)).c_str());

            if ((extractedNumber < 0) || (maxNodeId < extractedNumber)) {//node id 0 for RF propagation analyzer of GUI
                cerr << "Error: proptwotier-links-running-secondary: " << linksString << endl;
                exit(1);
            }//if//

            selectedNodeIds.insert(extractedNumber);
        }
        else if (linksString.at(endOfSubfieldTerminatorPos) == ':') {

            //This is a link (node pair)

            const size_t colonPosition = endOfSubfieldTerminatorPos;

            endOfSubfieldTerminatorPos = linksString.find(',', colonPosition);

            if (endOfSubfieldTerminatorPos == string::npos) {
                endOfSubfieldTerminatorPos = linksString.length();
            }//if//

            const NodeIdType firstNodeNumber =
                atoi(linksString.substr(currentPosition, (colonPosition - currentPosition)).c_str());
            if ((firstNodeNumber < 0) || (maxNodeId < firstNodeNumber)) {//node id 0 for RF propagation analyzer of GUI
                cerr << "Error: proptwotier-links-running-secondary: " << linksString << endl;
                exit(1);
            }//if//

            const NodeIdType secondNodeNumber =
                atoi(linksString.substr((colonPosition + 1), (endOfSubfieldTerminatorPos - colonPosition - 1)).c_str());
            if ((secondNodeNumber < 0) || (maxNodeId < secondNodeNumber)) {//node id 0 for RF propagation analyzer of GUI
                cerr << "Error: proptwotier-links-running-secondary: " << linksString << endl;
                exit(1);
            }//if//

            //set selected prop model for node pairs
            propModels[GetPropModelIndex(firstNodeNumber, secondNodeNumber)] = (*this).secondaryPropModelPtr;
            propModels[GetPropModelIndex(secondNodeNumber, firstNodeNumber)] = (*this).secondaryPropModelPtr;

        }//if//

        currentPosition = endOfSubfieldTerminatorPos + 1;

    }//while//


}//AssignPropModelForLinks//

//=====================================================================

class GroundLayer;

class ItmPropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    ItmPropagationLossCalculationModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const double& initCarrierFrequencyMhz,
        const shared_ptr<const GroundLayer>& initGroundLayerPtr,
        const double& initMaximumPropagationDistanceMeters,
        const bool initPropagationDelayIsEnabled,
        const int initNumberDataParallelThreads,
        const InterfaceOrInstanceIdType& initInstanceId);

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        const double& xyDistanceSquaredMeters) const;

private:
    typedef int RadioClimateType;
    enum {
        RADIO_CLIMATE_EQUATORIAL = 1,
        RADIO_CLIMATE_CONTINENTAL_SUBTROPICAL = 2,
        RADIO_CLIMATE_MARITIME_TROPICAL = 3,
        RADIO_CLIMATE_DESERT = 4,
        RADIO_CLIMATE_CONTINENTAL_TEMPERATE = 5,
        RADIO_CLIMATE_MARITIME_TEMPERATE_OVER_LAND = 6,
        RADIO_CLIMATE_MARITIME_TEMPERATE_OVER_SEA = 7,
    };

    typedef int PolarizationType;
    enum {
        POLARIZATION_HORIZONTAL = 0,
        POLARIZATION_VERTICAL = 1,
    };

    const shared_ptr<const GroundLayer> groundLayerPtr;

    double maxCalculationPointDivisionLength;

    double earthDielectricConstant;
    double earthConductivitySiemensPerMeters;
    double atmosphericBendingConstant; //n-Units,
    RadioClimateType radioClimate;
    PolarizationType polarization;
    double fractionOfTime;
    double fractionOfSituations;
    bool enableVerticalDiffractionPathCalculation;
    bool addTreeHeightToElevation;

    struct ElevationArray {
        int size;
        shared_array<double> elevations;

        ElevationArray() : size(0) {}

        void PrepareEnoughArraySize(const int necessarySize) {
            if (necessarySize + 2 >= size) {
                size = necessarySize + 2;
                elevations.reset(new double[size]);
            }//if//
        }
    };

    mutable vector<ElevationArray> elevationArrayPerThread;
};

class WeissbergerPropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    WeissbergerPropagationLossCalculationModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const double& initCarrierFrequencyMhz,
        const shared_ptr<const GroundLayer>& initGroundLayerPtr,
        const double& initMaximumPropagationDistanceMeters,
        const bool initPropagationDelayIsEnabled,
        const int initNumberDataParallelThreads,
        const InterfaceOrInstanceIdType& initInstanceId,
        const double initCalculationPointDivisionLength = 1.0);

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        const double& xyDistanceSquaredMeters) const;

private:
    const shared_ptr<const GroundLayer> groundLayerPtr;

    double maxCalculationPointDivisionLength;
};

inline
WeissbergerPropagationLossCalculationModel::WeissbergerPropagationLossCalculationModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const double& initCarrierFrequencyMhz,
        const shared_ptr<const GroundLayer>& initGroundLayerPtr,
        const double& initMaximumPropagationDistanceMeters,
        const bool initPropagationDelayIsEnabled,
        const int initNumberDataParallelThreads,
        const InterfaceOrInstanceIdType& initInstanceId,
        const double initCalculationPointDivisionLength)
    :
    SimplePropagationLossCalculationModel(
        initCarrierFrequencyMhz,
        initMaximumPropagationDistanceMeters,
        initPropagationDelayIsEnabled,
        initNumberDataParallelThreads),
    groundLayerPtr(initGroundLayerPtr),
    maxCalculationPointDivisionLength(initCalculationPointDivisionLength)
{
    if (theParameterDatabaseReader.ParameterExists(
            "propweissberger-calculation-point-division-length", initInstanceId)) {
        maxCalculationPointDivisionLength =
            theParameterDatabaseReader.ReadDouble("propweissberger-calculation-point-division-length", initInstanceId);
    }//if//
}

static inline
InterfaceOrInstanceIdType GetChannelInstanceId(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId)
{
    InterfaceOrInstanceIdType channelInstanceId = interfaceId;

    if (theParameterDatabaseReader.ParameterExists("channel-instance-id", nodeId, interfaceId)) {

        channelInstanceId = MakeLowerCaseString(
            theParameterDatabaseReader.ReadString("channel-instance-id", nodeId, interfaceId));

    }
    else if (theParameterDatabaseReader.ParameterExists("channel-model", nodeId, interfaceId)) { //for backward compatibility, will be removed in the future

        channelInstanceId = MakeLowerCaseString(
            theParameterDatabaseReader.ReadString("channel-model", nodeId, interfaceId));

    }//if//

    return channelInstanceId;

}//GetChannelInstanceId//


}//namespace//


#endif
