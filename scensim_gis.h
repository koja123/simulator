// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_GIS_H
#define SCENSIM_GIS_H

#include "scensim_nodeid.h"
#include "scensim_proploss.h"
#include "scensim_support.h"
#include "scensim_terrain.h"
#include "boost/shared_array.hpp"
#include <deque>
#include <queue>
#include <list>

namespace ScenSim {

using std::deque;
using std::map;
using std::pair;
using std::list;
using std::string;
using std::vector;
using std::min;
using boost::shared_array;

class InsiteGeometry;
class GisSubsystem;
class GisObject;
class RoadLayer;
class RailRoadLayer;
class Intersection;

const string GIS_ROAD_STRING = "road";
const string GIS_INTERSECTION_STRING = "intersection";
const string GIS_RAILROAD_STRING = "rail";
const string GIS_AREA_STRING = "area";
const string GIS_STATION_STRING = "station";
const string GIS_OLD_TRAFFIC_LIGHT_STRING = "signal";
const string GIS_TRAFFIC_LIGHT_STRING = "trafficlight";
const string GIS_BUSSTOP_STRING = "busstop";
const string GIS_ENTRANCE_STRING = "entrance";
const string GIS_PARK_STRING = "park";
const string GIS_BUILDING_STRING = "building";
const string GIS_WALL_STRING = "wall";
const string GIS_ROAD_INTERSECTION_STRING = "intersection";
const string GIS_RAILROAD_INTERSECTION_STRING = "railroadIntersection";
const string GIS_TRAINSTATION_STRING = "trainStation";
const string GIS_GROUND_STRING = "ground";
const string GIS_TREE_STRING = "tree";
const string GIS_POI_STRING = "poi";

typedef NodeIdType GisObjectIdType;
const GisObjectIdType INVALID_GIS_OBJECT_ID = UINT_MAX;

typedef int MaterialIdType;
const MaterialIdType INVALID_MATERIAL_ID = -1;

typedef int VariantIdType;
typedef VariantIdType PointIdType;
typedef VariantIdType IntersectionIdType;
typedef VariantIdType PedestrianPathIdType;
typedef VariantIdType RoadIdType;
typedef VariantIdType BusStopIdType;
typedef VariantIdType RailRoadIdType;
typedef VariantIdType RailRoadIntersectionIdType;
typedef VariantIdType RailRoadStationIdType;
typedef VariantIdType AreaIdType;
typedef VariantIdType ParkIdType;
typedef VariantIdType WallIdType;
typedef VariantIdType BuildingIdType;
typedef VariantIdType AreaIdType;
typedef VariantIdType EntranceIdType;
typedef VariantIdType PoiIdType;
typedef VariantIdType TrafficLightIdType;
typedef VariantIdType GenericGisObjectIdType;
typedef VariantIdType VertexIdType; // vertex is network path vertex
typedef VertexIdType RailVertexIdType;

typedef int RailRoadLineIdType;
const RailRoadLineIdType INVALID_RAILROAD_LINE_ID = -1;

typedef int BusLineIdType;
const BusLineIdType INVALID_BUS_LINE_ID = -1;

typedef int RouteIdType;
const RouteIdType INVALID_ROUTE_ID = -1;

const VariantIdType INVALID_VARIANT_ID = -1;
const VertexIdType INVALID_VERTEX_ID = -1;

const double DEFAULT_ROAD_WIDTH_METERS = 1;

const int NUMBER_SIGNINICANT_FIGURES = 10;
const double SIGNINICANT_FIGURE_MUL = std::pow(10., double(NUMBER_SIGNINICANT_FIGURES));
const double SIGNINICANT_FIGURE_DIV = 1. / SIGNINICANT_FIGURE_MUL;

enum RoadType {

    ROAD_MOTORWAY,
    ROAD_MOTORWAY_LINK,
    ROAD_TRUNK,
    ROAD_TRUNK_LINK,
    ROAD_PRIMARY,
    ROAD_PRIMARY_LINK,
    ROAD_SECONDARY,
    ROAD_SECONDARY_LINK,
    ROAD_TERTIARY,
    ROAD_RESIDENTIAL,
    ROAD_ROAD,
    ROAD_BUSGUIDEWAY,
    ROAD_LIVING_STREET,
    ROAD_SERVICE,
    ROAD_TRACK,
    ROAD_RACEWAY,

    ROAD_EXTRA_ROAD,
    ROAD_SERVICES,

    ROAD_PEDESTRIAN,
    ROAD_PATH,
    ROAD_CYCLEWAY,
    ROAD_FOOTWAY,
    ROAD_BRIDLEWAY,
    ROAD_BYWAY,
    ROAD_STEPS,

    ROAD_UNCLASSIFIED,
};

enum RailRoadType {
    RAILROAD_JR = 0,
    RAILROAD_NON_JR = 1,
    RAILROAD_STREET_CAR = 2,
    RAILROAD_SUBWAY = 3,
    RAILROAD_OTHER = 4,
    RAILROAD_BULLET_TRAIN = 5
};

typedef int8_t GisObjectType;
const GisObjectType INVALID_OBJECT_TYPE = -1;
enum {
    GIS_AREA = 0,
    GIS_POINT = 1,
    GIS_ROAD = 2,
    GIS_INTERSECTION = 3,
    GIS_RAILROAD = 4,
    GIS_RAILROAD_INTERSECTION = 5,
    GIS_RAILROAD_STATION = 6,
    GIS_PARK = 7,
    GIS_WALL = 8,
    GIS_BUILDING = 9,
    GIS_PEDESTRIAN_PATH = 10,
    GIS_BUSSTOP = 11,
    GIS_POI = 12,
    GIS_TRAFFICLIGHT = 13,
    GIS_ENTRANCE = 14,
    GIS_VEHICLE_LINE = 15,
    GIS_GROUND = 16,
    GIS_GENERIC_START,
};

typedef int RoadDirectionType;
const RoadDirectionType ROAD_DIRECTION_UP = 0;
const RoadDirectionType ROAD_DIRECTION_DOWN = 1;
const RoadDirectionType NUMBER_ROAD_DIRECTIONS = 2;

typedef int RoadTurnDirectionType;
const RoadTurnDirectionType ROAD_TURN_RIGHT = 0;
const RoadTurnDirectionType ROAD_TURN_STRAIGHT = 1;
const RoadTurnDirectionType ROAD_TURN_LEFT = 2;
const RoadTurnDirectionType ROAD_TURN_BACK = 3;
const RoadTurnDirectionType ROAD_TURN_NONE = 4;

struct RoadTurnType {
    RoadTurnDirectionType direction;
    double totalRoadWidth;
    bool hasLanes;

    RoadTurnType(
        const RoadTurnDirectionType& initDirection,
        const double initTotalRoadWidth,
        const bool initHasLanes = true)
        :
        direction(initDirection),
        totalRoadWidth(initTotalRoadWidth),
        hasLanes(initHasLanes)
    {}

    RoadTurnType()
        :
        direction(ROAD_TURN_NONE),
        totalRoadWidth(0),
        hasLanes(false)
    {}
};

//------------------------------------------------------------------------
// Basic primitive
//------------------------------------------------------------------------

struct Vertex {
    union {
        double positionX;
        double x;
    };
    union {
        double positionY;
        double y;
    };
    union {
        double positionZ;
        double z;
    };

    Vertex() : x(0.),  y(0.), z(0.) {}
    Vertex(
        const double initX,
        const double initY,
        const double initZ = 0.0)
        :
        x(initX),
        y(initY),
        z(initZ)
    {}

    bool operator==(const Vertex& right) const {
        return (x == right.x && y == right.y && z == right.z);
    }
    bool operator!=(const Vertex& right) const {
        return !((*this) == right);
    }
    void operator+=(const Vertex& right);
    Vertex operator+(const Vertex& right) const;
    Vertex operator-(const Vertex& right) const;
    Vertex operator/(const double scale) const;
    Vertex operator*(const double scale) const;

    Vertex XYPoint() const { return Vertex(x, y); }
    double XYDistance() const { return (sqrt(x*x + y*y)); }
    double Distance() const { return (sqrt(x*x + y*y + z*z)); }
    Vertex Normalized() const;
    bool IsNull() const { return (x == 0 && y == 0 && z == 0); }
    Vertex Rounded() const;
    double DistanceTo(const Vertex& vertex) const;
    double XYDistanceTo(const Vertex& vertex) const;
    double DirectionRadians() const { return (std::atan2(y, x)); }
    Vertex NormalVector(const Vertex& vertex = Vertex()) const { return Vertex(vertex.y - y, x - vertex.x); }


    Vertex Cross(const Vertex& v) const { return Vertex(y*v.z-z*v.y,z*v.x-x*v.z,x*v.y-y*v.x); }
    double Dot(const Vertex& v) const { return x*v.x+y*v.y+z*v.z; }
    Vertex Inverted() const;

    Vertex ToXyVertex(
        const double latitudeOriginDegrees,
        const double longitudeOriginDegrees) const;

    bool operator<(const Vertex& right) const {
        return ((x < right.x) ||
                (x == right.x && y < right.y) ||
                (x == right.x && y == right.y && z < right.z));
    }
};

struct RotationMatrix {

    RotationMatrix() {
        rv[0] = Vertex(1., 0., 0.);
        rv[1] = Vertex(0., 1., 0.);
        rv[2] = Vertex(0., 0., 1.);

        (*this).UpdateColumnVertex();
    }

    RotationMatrix(const Vertex& initRv0, const Vertex& initRv1, const Vertex& initRv2) {
        rv[0] = initRv0;
        rv[1] = initRv1;
        rv[2] = initRv2;

        (*this).UpdateColumnVertex();
    }

    // rotation matrix
    RotationMatrix(const double xDegrees, const double yDegrees, const double zDegrees) {
        const double xRadians = xDegrees*RADIANS_PER_DEGREE;
        const double yRadians = yDegrees*RADIANS_PER_DEGREE;
        const double zRadians = zDegrees*RADIANS_PER_DEGREE;
        const double cx = std::cos(xRadians);
        const double sx = std::sin(xRadians);
        const double cy = std::cos(yRadians);
        const double sy = std::sin(yRadians);
        const double cz = std::cos(zRadians);
        const double sz = std::sin(zRadians);

        rv[0] = Vertex(cy*cx, -(cy*sx), sy);
        rv[1] = Vertex((sz*sy*cx) + (cz*sx), -(sz*sy*sx) + (cz*cx), -(sz*cy));
        rv[2] = Vertex(-(cz*sy*cx) + (sz*sx), (cz*sy*sx) + (sz*cx), (cz*cy));

        (*this).UpdateColumnVertex();
    }

    static Vertex ApplyMatrix(const Vertex& base, const RotationMatrix& m) {
        return Vertex(
            base.x*m.rv[0].x + base.y*m.rv[1].x + base.z*m.rv[2].x,
            base.x*m.rv[0].y + base.y*m.rv[1].y + base.z*m.rv[2].y,
            base.x*m.rv[0].z + base.y*m.rv[1].z + base.z*m.rv[2].z);
    }

    RotationMatrix operator*(const RotationMatrix& right) const {
        return RotationMatrix(
            ApplyMatrix(rv[0], right),
            ApplyMatrix(rv[1], right),
            ApplyMatrix(rv[2], right));
    }

    Vertex operator*(const Vertex& v) const {
        return Vertex(rv[0].Dot(v), rv[1].Dot(v), rv[2].Dot(v));
    }

    void UpdateColumnVertex() {
        cv[0] = Vertex(rv[0].x, rv[1].x, rv[2].x);
        cv[1] = Vertex(rv[0].y, rv[1].y, rv[2].y);
        cv[2] = Vertex(rv[0].z, rv[1].z, rv[2].z);
    }

    Vertex rv[3];
    Vertex cv[3];
};

struct VertexConnection {
    VertexIdType vertexId;
    VariantIdType variantId;

    VertexConnection()
        :
        vertexId(INVALID_VERTEX_ID),
        variantId(INVALID_VARIANT_ID)
    {}

    VertexConnection(
        const VertexIdType initVertexId,
        const VariantIdType initVariantId)
        :
        vertexId(initVertexId),
        variantId(initVariantId)
    {}

    bool operator==(const VertexConnection& right) const {
        return (vertexId == right.vertexId &&
                variantId == right.variantId);
    }
};

struct GisPositionIdType {
    GisObjectType type;
    VariantIdType id;

    bool IsInvalid() const { return (id == INVALID_VARIANT_ID); }
    bool IsValid() const { return !(*this).IsInvalid(); }

    GisPositionIdType()
        :
        type(INVALID_OBJECT_TYPE),
        id(INVALID_VARIANT_ID)
    {}

    GisPositionIdType(
        const GisObjectType& initType,
        const VariantIdType& initId)
        :
        type(initType),
        id(initId)
    {}

    bool operator==(const GisPositionIdType& right) const {
        return (type == right.type &&
                id == right.id);
    }
    bool operator!=(const GisPositionIdType& right) const {
        return !((*this) == right);
    }
    bool operator<(const GisPositionIdType& right) const {
        return ((type < right.type) ||
                (type == right.type && id < right.id));
    }
};

struct GisVertex {
    Vertex vertex;

    map<GisObjectType, vector<VertexConnection> > connections;
    map<VertexIdType, GisPositionIdType> connectionsPerVertex;

    GisVertex(const Vertex& initVertex)
        :
        vertex(initVertex)
    {}

    const vector<VertexConnection>& GetVertexConnection() const;

    VariantIdType GetConnectedObjectId(const GisObjectType& objectType) const;
    vector<VariantIdType> GetConnectedObjectIds(const GisObjectType& objectType) const;

    bool HasConnection(const GisObjectType& objectType) const;
};

enum QuadrantType {
    QUADRANT_1_PX_PY = 0, //x -> positive, y -> positive
    QUADRANT_2_NX_PY = 1, //x -> negative, y -> positive
    QUADRANT_3_NX_NY = 2, //x -> negative, y -> negative
    QUADRANT_4_PX_NY = 3, //x -> positive, y -> negative
};

struct Rectangle {
    double minX;
    double minY;
    double maxX;
    double maxY;

    Rectangle() :  minX(0.0), minY(0.0),  maxX(0.0), maxY(0.0) {}
    Rectangle(
        const double& initMinX,
        const double& initMinY,
        const double& initMaxX,
        const double& initMaxY)
        :
        minX(initMinX),
        minY(initMinY),
        maxX(initMaxX),
        maxY(initMaxY)
    {}
    Rectangle(
        const Vertex& initCenterPoint,
        const double initHalfLength)
        :
        minX(initCenterPoint.x - initHalfLength),
        minY(initCenterPoint.y - initHalfLength),
        maxX(initCenterPoint.x + initHalfLength),
        maxY(initCenterPoint.y + initHalfLength)
    {}
    Rectangle(
        const Vertex& initCenterPoint,
        const double initHalfXLength,
        const double initHalfYLength)
        :
        minX(initCenterPoint.x - initHalfXLength),
        minY(initCenterPoint.y - initHalfYLength),
        maxX(initCenterPoint.x + initHalfXLength),
        maxY(initCenterPoint.y + initHalfYLength)
    {}
    Rectangle(
        const Vertex& initPoint1,
        const Vertex& initPoint2)
        :
        minX(std::min(initPoint1.x, initPoint2.x)),
        minY(std::min(initPoint1.y, initPoint2.y)),
        maxX(std::max(initPoint1.x, initPoint2.x)),
        maxY(std::max(initPoint1.y, initPoint2.y))
    {}

    void operator+=(const Rectangle& right);
    Rectangle Expanded(const double margin) const;

    Vertex GetCenter() const { return Vertex((minX+maxX)*0.5, (minY+maxY)*0.5); }
    Vertex GetBaseVertex() const { return Vertex(minX, minY); }

    Vertex GetBottomLeft() const { return Vertex(minX, minY); }
    Vertex GetBottomRight() const { return Vertex(maxX, minY); }
    Vertex GetTopLeft() const { return Vertex(minX, maxY); }
    Vertex GetTopRight() const { return Vertex(maxX, maxY); }

    QuadrantType GetQuadrantOf(const Vertex& v) const;

    Rectangle MakeChildRect(const QuadrantType& quadrantType) const;

    bool Contains(const Vertex& point) const;
    bool Contains(const Rectangle& rect) const;

    bool OverlappedWith(const Rectangle& rectangle) const;
    Rectangle GetOverlappedRectangle(const Rectangle& rectangle) const;
    bool IntersectsWithLine(
        const Vertex& lineEdge1,
        const Vertex& lineEdge2) const;

    double GetWidth() const { return (maxX - minX); }
    double GetHeight() const { return (maxY - minY); }

    double GetDiagonalLength() const {
        const double w = (*this).GetWidth();
        const double h = (*this).GetHeight();

        return (sqrt(w*w + h*h));
    }

    bool IsValid() const {
        return ((*this).GetWidth() > 0 ||  (*this).GetHeight() > 0);
    }
};

class Triangle {
public:
    Triangle() {}
    Triangle(
        const Vertex& initP1,
        const Vertex& initP2,
        const Vertex& initP3)
        : p1(initP1), p2(initP2), p3(initP3)
    {}

    const Vertex& GetP1() const { return p1; }
    const Vertex& GetP2() const { return p2; }
    const Vertex& GetP3() const { return p3; }

    Rectangle GetRect() const;
    double GetSurfaceHeightAt(const Vertex& pos) const;
    bool Contains(const Vertex& pos) const;
    bool IntersectsWith(const Rectangle& rect) const;
    bool IntersectsWithLine(
        const Vertex& lineEdge1,
        const Vertex& lineEdge2) const;

    pair<Vertex, Vertex> GetEdge1() const { return make_pair(std::min(p1, p2), std::max(p1, p2)); }
    pair<Vertex, Vertex> GetEdge2() const { return make_pair(std::min(p2, p3), std::max(p2, p3)); }
    pair<Vertex, Vertex> GetEdge3() const { return make_pair(std::min(p3, p1), std::max(p3, p1)); }

private:
    Vertex p1;
    Vertex p2;
    Vertex p3;
};

//------------------------------------------------------------------------
// Material
//------------------------------------------------------------------------

struct Material {
    string name;
    double transmissionLossDb;

    Material() : transmissionLossDb(0) {}
    Material(
        const string& initName,
        const double initTransmissionLossDb)
        :
        name(initName),
        transmissionLossDb(initTransmissionLossDb)
    {}
};

class MaterialSet {
public:
    MaterialSet();

    void AddMaterial(
        const string& name,
        const double transmissionLossDb);

    MaterialIdType GetMaterialId(const string& name) const;

    const Material& GetMaterial(const string& name) const;
    const Material& GetMaterial(const MaterialIdType& materialId) const;

private:

    shared_ptr<Material> defaultMaterialPtr;

    map<string, MaterialIdType> materialIds;
    vector<shared_ptr<Material> > materialPtrs;

    mutable map<string, MaterialIdType>::const_iterator cacheIter;
};

//------------------------------------------------------------------------
// LoS, NLoS resource
//------------------------------------------------------------------------

namespace Los {
    struct LosQuadTree;
    struct LosMovingObject;
}

struct NlosPathData {
    //Use shared_array for space reasons.

    int numberEndToEndRoadIds;
    shared_array<RoadIdType> endToEndRoadIds;

    int numberEndToEndIntersectionIds;
    shared_array<IntersectionIdType> endToEndIntersectionIds;

    double pathValue;

    std::pair<IntersectionIdType, IntersectionIdType> GetIntersectionPair() const {
        assert(numberEndToEndIntersectionIds > 2);
        return std::make_pair(endToEndIntersectionIds[0], endToEndIntersectionIds[numberEndToEndIntersectionIds-1]);
    }

    NlosPathData()
        :
        numberEndToEndRoadIds(0),
        numberEndToEndIntersectionIds(0),
        pathValue(0.0)
    {}

    NlosPathData(
        const RoadIdType& initRoadId1,
        const RoadIdType& initRoadId2,
        const IntersectionIdType& initIntersectionId1,
        const IntersectionIdType& initIntersectionId2,
        const IntersectionIdType& initNlosIntersectionId);

    NlosPathData(
        const RoadIdType& initRoadId1,
        const RoadIdType& initRoadId2,
        const IntersectionIdType& initIntersectionId1,
        const IntersectionIdType& initIntersectionId2,
        const IntersectionIdType& initNlosIntersectionId1,
        const IntersectionIdType& initNlosIntersectionId2,
        const RoadIdType& initNlosRoadId);

    ~NlosPathData() {}

    NlosPathData Clone() const;

    std::pair<RoadIdType, RoadIdType> GetRoadRelation() const;

    void Normalize();
    void Inverse();

    bool IsMultipleNlosPath() const;
    size_t GetNlosCount() const;
    double GetNlosPathDistance(const vector<Intersection>& intersections) const;

    IntersectionIdType GetStartIntersectionId(
        const RoadIdType& startRoadId) const;

    IntersectionIdType GetEndIntersectionId(
        const RoadIdType& startRoadId) const;

    IntersectionIdType GetIntersectionId(
        const RoadIdType& startRoadId,
        const size_t intersectionNumber) const;

    RoadIdType GetEndRoadId(
        const RoadIdType& startRoadId) const;

    RoadIdType GetRoadId(
        const RoadIdType& startRoadId,
        const size_t roadNumber) const;

    void ExpandRoad(
        const RoadIdType& startRoadId,
        const IntersectionIdType& baseIntersectionId,
        const RoadIdType& expandRoadId,
        const IntersectionIdType& endIntersectionId);

    void PushBackRoadId(const RoadIdType& roadId);
    void PushFrontRoadId(const RoadIdType& roadId);

    RoadIdType GetFrontRoadId() const;
    RoadIdType GetBackRoadId() const;
    RoadIdType GetNlosRoadId() const;
    RoadIdType GetLastNlosRoadId() const;

    bool RoadIdIsEmpty() const;
    bool ContainsRoad(const RoadIdType& roadId) const;
    size_t GetNumberOfRoads() const;

    void PushBackIntersectionId(const IntersectionIdType& intersectionId);
    void PushFrontIntersectionId(const IntersectionIdType& intersectionId);
    IntersectionIdType GetFrontIntersectionId() const;
    IntersectionIdType GetBackIntersectionId() const;

    size_t GetNumberOfIntersections() const;
};

enum RoadLosRelationType {
    ROAD_LOSRELATION_LOS,
    ROAD_LOSRELATION_NLOS,
    ROAD_LOSRELATION_OUTOFNLOS,
};

struct RoadLosRelationData {
    RoadLosRelationType relationType;
    map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData> nlosPaths;

    RoadLosRelationData() : relationType(ROAD_LOSRELATION_OUTOFNLOS) {}
    RoadLosRelationData(const RoadLosRelationType& initRelationType)
      : relationType(initRelationType)
    {}
};

class NlosPathValueCalculator {
public:
    NlosPathValueCalculator() {}
    virtual ~NlosPathValueCalculator() {}

    virtual double GetNlosPathValue(const NlosPathData& nlosPath) const = 0;
};

class RoadLosChecker {
public:
    RoadLosChecker(
        const shared_ptr<const RoadLayer>& initRoadLayerPtr,
        const shared_ptr<NlosPathValueCalculator>& initNlosValueCalculatorPtr,
        const size_t initMaxDiffractionCount,
        const double initLosThresholdRadians,
        const double initMaxNlosDistance);

    void MakeLosRelation();

    const RoadLosRelationData& GetLosRelation(
        const RoadIdType& roadId1,
        const RoadIdType& roadId2) const;

    double CalculateNlosPointRadians(
        const NlosPathData& nlosPath,
        const GisObjectIdType& startRoadId) const;

    vector<Vertex> CalculateNlosPoints(
        const NlosPathData& nlosPath,
        const RoadIdType& startRoadId) const;

    double CalculateNlosPointToStartPointCenterDistance(
        const NlosPathData& nlosPath,
        const RoadIdType& startRoadId,
        const Vertex& startPosition) const;
    double CalculateStartPointToEndPointCenterDistance(
        const NlosPathData& nlosPath,
        const RoadIdType& startRoadId,
        const Vertex& startPosition,
        const Vertex& endPosition) const;
    double CalculateNlosPointToEndPointCenterDistance(
        const NlosPathData& nlosPath,
        const RoadIdType& startRoadId,
        const Vertex& endPosition) const;
    double CalculateDistanceToLastNlosPoint(
        const NlosPathData& nlosPath,
        const RoadIdType& startRoadId,
        const Vertex& startPosition) const;

    bool IsCompleteNlosPath(
        const NlosPathData& nlosPath,
        const RoadIdType& startRoadId,
        const Vertex& startPosition,
        const Vertex& endPosition) const;

    bool PositionsAreLineOfSight(
        const Vertex& position1,
        const Vertex& position2) const;

private:
    void PushExtendedNlosPath(
        const map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection,
        const map<IntersectionIdType, set<IntersectionIdType> >& losIntersectionIdsPerIntersection,
        const NlosPathData& nlosPath,
        const RoadIdType& startRoadId,
        vector<NlosPathData>& nlosPaths) const;


    void MakeLosRelation(map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection);
    void MakeNlosRelation(const map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIn0tersection);
    void MakeNlos1Relation(const map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection);
    void MakeNlos2Relation(const map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection);
    void MakeNlos2ToNRelation(const map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection);

    void OutputLosRelation() const;

    const shared_ptr<const RoadLayer> roadLayerPtr;
    const shared_ptr<NlosPathValueCalculator> nlosValueCalculatorPtr;
    const size_t maxDiffractionCount;
    const double losThresholdRadians;
    const double maxNlosDistance;

    map<std::pair<RoadIdType, RoadIdType>, RoadLosRelationData> losRelationBetweenRoads;
    RoadLosRelationData outofnlosRelation;

};

template <typename T>
class ReverseAccess {
public:
    ReverseAccess(
        deque<T>& initOrigDeque,
        const bool initIsReverse)
        :
        isReverse(initIsReverse),
        origDeque(initOrigDeque)
    {}

    void pop_front() {
        if (isReverse) {
            origDeque.pop_back();
        } else {
            origDeque.pop_front();
        }
    }

    T& front() {
        return origDeque[(*this).index(0)];
    }
    T& operator[](const int i) {
        return origDeque[(*this).index(i)];
    }

    size_t index(const int i) const {
        if (isReverse) {
            return (origDeque.size() - i - 1);
        }
        return i;
    }

    size_t size() const { return origDeque.size(); }

    bool empty() const { return origDeque.empty(); }

    void push_back(const T& value) {
        if (isReverse) {
            origDeque.push_front(value);
        } else {
            origDeque.push_back(value);
        }
    }

private:
    const bool isReverse;
    deque<T>& origDeque;
};


template <typename T>
class ConstReverseAccess {
public:
    ConstReverseAccess(
        const deque<T>& initOrigDeque,
        const bool initIsReverse)
        :
        isReverse(initIsReverse),
        origDeque(initOrigDeque)
    {}

    const T& operator[](const size_t i) const {
        return origDeque[(*this).index(i)];
    }

    size_t index(const size_t i) const {
        if (isReverse) {
            return (origDeque.size() - i - 1);
        }
        return i;
    }

    size_t size() const { return origDeque.size(); }

    bool empty() const { return origDeque.empty(); }

    const T& front() const {
        return origDeque[(*this).index(0)];
    }

    const T& back() const {
        return origDeque[(*this).index((*this).size() - 1)];
    }

private:
    const bool isReverse;
    const deque<T>& origDeque;
};

//---------------------------------------------------------------------------
//class GisObject
//---------------------------------------------------------------------------

class GisObject {
public:
    GisObject(
        GisSubsystem* initSubsystemPtr,
        const GisObjectType& initObjectType,
        const GisObjectIdType& initObjectId,
        const VariantIdType& initVariantId);

    GisObject(
        GisSubsystem* initSubsystemPtr,
        const GisObjectType& initObjectType,
        const GisObjectIdType& initObjectId,
        const VariantIdType& initVariantId,
        const VertexIdType& initVertexId);

    virtual ~GisObject() {}

    GisObjectIdType GetObjectId() const;
    const string& GetObjectName() const;
    GisObjectType GetObjectType() const;

    size_t NumberOfVertices() const;

    const GisVertex& GetGisVertex(const size_t index) const;

    const Vertex& GetVertex(const size_t index) const;
    const Vertex& GetStartVertex() const;
    const Vertex& GetEndVertex() const;

    Vertex GetCenterPoint() const;
    vector<Vertex> GetVertices() const;
    const vector<VertexIdType>& GetVertexIds() const;

    const VertexIdType& GetVertexId(const size_t index) const;
    const VertexIdType& GetStartVertexId() const;
    const VertexIdType& GetEndVertexId() const;
    const VertexIdType& GetNearestVertexId(const Vertex& vertex) const;

    bool IsStartOrEndVertex(const VertexIdType& vertexId) const;
    bool IsStartVertex(const VertexIdType& vertexId) const;
    bool IsEndVertex(const VertexIdType& vertexId) const;
    bool ContainsVertexId(const VertexIdType& vertexId) const;

    const VertexIdType& GetOthersideVertexId(const VertexIdType& vertexId) const;

    const Rectangle& GetMinRectangle() const;
    const Material& GetMaterial() const;

    //virtual bool IsInPolygon(const Vertex& point) const;

    size_t CalculateNumberIntersections(const Vertex& p1, const Vertex& p2) const;

    virtual void UpdateMinRectangle() const;
    virtual bool IntersectsWith(const Rectangle& rect) const;

    void InsertVertex(
        const VertexIdType& prevVertexId,
        const VertexIdType& newVertexId,
        const VertexIdType& nextVertexId);

    void InsertVertex(const Vertex& newVertex);

    bool IsEnabled() const;

    void LoadParameters(const ParameterDatabaseReader& parameterDatabaseReader);

    virtual void SetEnabled(const bool enable);

    bool MasTraceIsOn() const;

    double GetElevationFromGroundMeters() const;

protected:
    friend class GisSubsystem;

    struct Implementation;

    shared_ptr<Implementation> commonImplPtr;

    class GisObjectEnableDisableEvent;

private:

};

class Point : public GisObject {
public:
    Point(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const PointIdType& initPointId,
        const VertexIdType& initVertexId)
        :
        GisObject(initSubsystemPtr, GIS_POINT, initObjectId, initPointId, initVertexId)
    {}

    PointIdType GetPointId() const;
};

class Entrance : public GisObject {
public:
    Entrance(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const EntranceIdType& initEntranceId,
        const VertexIdType& initVertexId)
        :
        GisObject(initSubsystemPtr, GIS_ENTRANCE, initObjectId, initEntranceId, initVertexId)
    {}

    EntranceIdType GetEntranceId() const;

    const Vertex& GetVertex() const;
    VertexIdType GetVertexId() const;
};

class Poi : public GisObject {
public:
    Poi(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const PoiIdType& initPoiId,
        const VertexIdType& initVertexId)
        :
        GisObject(initSubsystemPtr, GIS_POI, initObjectId, initPoiId, initVertexId),
        humanCapacity(INT_MAX)
    {}

    PoiIdType GetPoiId() const;

    const Vertex& GetVertex() const;
    VertexIdType GetVertexId() const;

    string GetInformation() const { return information; }

    RoadIdType GetNearestEntranceRoadId(const Vertex& position) const;

    int GetHumanCapacity() const { return humanCapacity; }
    bool IsAPartOfObject() const { return parentPositionId.IsValid(); }
    GisPositionIdType GetParentGisPositionId() const {
        assert((*this).IsAPartOfObject());
        return parentPositionId;
    }

private:
    friend class PoiLayer;

    string information;

    int humanCapacity;
    GisPositionIdType parentPositionId;
};

class Road : public GisObject {
public:
    Road(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const RoadIdType& initRoadId);

    RoadIdType GetRoadId() const;

    RoadType GetRoadType() const { return type; }

    double GetArcDistanceMeters(
        const bool calculate3dArcDistance = true
        /*false -> horizontal arc distance*/) const;

    double GetArcDistanceMeters(
        const VertexIdType& vertexId1,
        const VertexIdType& vertexId2) const;

    double GetDirectionRadians() const;
    bool Contains(const Vertex& point) const;
    double DistanceTo(const Vertex& point) const;

    IntersectionIdType GetOtherSideIntersectionId(
        const IntersectionIdType& intersectionId) const;
    IntersectionIdType GetStartIntersectionId() const;
    IntersectionIdType GetEndIntersectionId() const;

    const Vertex& GetNeighborVertex(
        const IntersectionIdType& intersectionId) const;

    Vertex GetNearestPosition(const Vertex& position) const;
    IntersectionIdType GetNearestIntersectionId(const Vertex& position) const;

    double GetLaneWidthMeters() const { return widthMeters / (numberStartToEndLanes + numberEndToStartLanes); }
    double GetRoadWidthMeters() const { return widthMeters; }

    // Obsolete
    double GetWidthMeters() const { return (*this).GetRoadWidthMeters(); }

    virtual void UpdateMinRectangle() const;
    virtual bool IntersectsWith(const Rectangle& rect) const;

    void GetLaneVertices(
        const size_t laneNumber,
        const bool waypointFromAdditionalStartPosition,
        const Vertex& startPosition/*added for elevation calculation*/,
        deque<Vertex>& laneVertices) const;

    pair<Vertex, Vertex> GetSideLineToIntersection(
        const IntersectionIdType& intersectionId,
        const double averageWidthMeters) const;

    Vertex GetInternalPoint(
        const IntersectionIdType& intersectionId,
        const double internalLengthMeters) const;

    void GetPedestrianVertices(
        const VertexIdType& lastVertexId,
        const VertexIdType& startVertexId,
        const bool walkLeftSide,
        const double maxOffset,
        const Vertex& startPosition/*added for elevation calculation*/,
        deque<Vertex>& waypoints) const;

    void GetPedestrianVertices(
        const VertexIdType& lastVertexId,
        const VertexIdType& startVertexId,
        const bool walkLeftSide,
        const Vertex& startPosition/*added for elevation calculation*/,
        deque<Vertex>& waypoints) const;

    size_t GetRandomOutgoingLaneNumber(
        const VertexIdType& startVertexId,
        HighQualityRandomNumberGenerator& aRandomNumberGenerator) const;

    size_t GetNearestOutgoingLaneNumber(
        const VertexIdType& startVertexId,
        const Vertex& position) const;

    size_t GetOutsideOutgoingLaneNumber(
        const VertexIdType& startVertexId) const;

    size_t GetNumberOfLanes() const { return (*this).GetNumberOfStartToEndLanes() + (*this).GetNumberOfEndToStartLanes(); }
    size_t GetNumberOfStartToEndLanes() const { return numberStartToEndLanes; }
    size_t GetNumberOfEndToStartLanes() const { return numberEndToStartLanes; }

    bool IsParking() const;

    bool IsBusRoad() const { return (type <= ROAD_SERVICES); } //obsolete
    bool IsVehicleRoad() const { return (type <= ROAD_SERVICES); } //obsolete

    bool PedestrianCanPass() const { return (type >= ROAD_TRUNK); }
    bool BusCanPass() const { return (type <= ROAD_SERVICES); }
    bool VehicleCanPass() const { return (type <= ROAD_SERVICES); }
    bool IsExtraPath() const { return (type == ROAD_EXTRA_ROAD); }
    bool IsPedestrianRoad() const { return (type == ROAD_PEDESTRIAN); }

    size_t GetParkingLaneNumber() const;

    const deque<Vertex>& GetCompleteVertices() const { return vertices; }
    const vector<Vertex>& GetPolygon() const { return polygon; }

    bool IsRightHandTraffic() const { return isRightHandTraffic; }

    bool CanApproach(const size_t laneNumber, const RoadIdType& roadId) const;

    size_t GetNextLaneNumber(const size_t laneNumber, const RoadIdType& roadId) const;

    size_t GetApproachLaneNumber(const RoadIdType& roadId) const;
    size_t GetNeighborLaneNumberToApproach(const size_t laneNumber, const RoadIdType& roadId) const;

    RoadDirectionType GetRoadDirection(const size_t laneNumber) const;

    bool HasPassingLane(const RoadDirectionType& directionType, const size_t laneNumer) const;
    bool HasNonPassingLane(const RoadDirectionType& directionType, const size_t laneNumer) const;

    size_t GetPassingLaneNumber(const RoadDirectionType& directionType, const size_t laneNumer) const;
    size_t GetNonPassingLaneNumber(const RoadDirectionType& directionType, const size_t laneNumer) const;

    size_t GetOutgoingRightLaneNumber(const VertexIdType& vertexId) const;
    size_t GetIncomingRightLaneNumber(const VertexIdType& vertexId) const;
    size_t GetOutgoingLeftLaneNumber(const VertexIdType& vertexId) const;
    size_t GetIncomingLeftLaneNumber(const VertexIdType& vertexId) const;

    vector<size_t> GetOutgoingLaneNumbers(const VertexIdType& vertexId) const;
    vector<size_t> GetIncomingLaneNumbers(const VertexIdType& vertexId) const;

    bool HasOutgoingLane(const VertexIdType& vertexId) const;
    bool HasIncomingLane(const VertexIdType& vertexId) const;

    int GetHumanCapacity() const { return humanCapacity; }
    double GetSpeedLimitMetersPerSec() const { return speedLimitMetersPerSec; }

    bool IsBaseGroundLevel() const { return isBaseGroundLevel; }

private:
    friend class RoadLayer;

    void SetIntersectionMargin(
        const IntersectionIdType& intersectionId,
        const double marginLength);

    void GetOffsetWaypoints(
        const double offset,
        const bool isReverse,
        const bool waypointFromAdditionalStartPosition,
        const Vertex& startPosition/*added for elevation calculation*/,
        deque<Vertex>& waypoints) const;

    void UpdatePolygon();

    void MakeTurnaroundLaneConnection();

    void MakeLaneConnection(
        const Road& otherRoad,
        const VertexIdType& vertexId,
        const RoadTurnDirectionType& direction);

    bool isRightHandTraffic;

    RoadType type;
    size_t numberStartToEndLanes;
    size_t numberEndToStartLanes;
    double widthMeters;
    bool isBaseGroundLevel;

    deque<Vertex> vertices;
    vector<Vertex> polygon;

    vector<map<RoadIdType, vector<pair<RoadTurnDirectionType, size_t> > > > laneConnections;

    int humanCapacity;
    double capacity;
    double speedLimitMetersPerSec;
};

enum TrafficLightType {
    TRAFFIC_LIGHT_GREEN,
    TRAFFIC_LIGHT_YELLOW,
    TRAFFIC_LIGHT_RED,
};

class TrafficLight : public GisObject {
public:
    TrafficLight(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const TrafficLightIdType& initTrafficLightId,
        const VertexIdType& initVertexId,
        const TimeType& initStartOffset,
        const TimeType& initGreenDuration,
        const TimeType& initYellowDuration,
        const TimeType& initRedDuration)
        :
        GisObject(initSubsystemPtr, GIS_TRAFFICLIGHT, initObjectId, initTrafficLightId, initVertexId),
        startOffset(initStartOffset),
        greenDuration(initGreenDuration),
        yellowDuration(initYellowDuration),
        redDuration(initRedDuration)
    {}

    TrafficLight(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const TrafficLightIdType& initTrafficLightId,
        const VertexIdType& initVertexId,
        const map<TimeType, TrafficLightType>& initTrafficLightIdsPerTime)
        :
        GisObject(initSubsystemPtr, GIS_TRAFFICLIGHT, initObjectId, initTrafficLightId, initVertexId),
        startOffset(ZERO_TIME),
        greenDuration(ZERO_TIME),
        yellowDuration(ZERO_TIME),
        redDuration(ZERO_TIME),
        trafficLightIdsPerTime(initTrafficLightIdsPerTime.begin(), initTrafficLightIdsPerTime.end())
    {
        trafficLightIdsPerTime.push_back(std::make_pair(INFINITE_TIME, TRAFFIC_LIGHT_GREEN));
    }

    void SyncTrafficLight(const TimeType& time);

    TrafficLightType GetTrafficLight(const TimeType& currentTime) const;

private:
    friend class Intersection;

    TimeType startOffset;
    TimeType greenDuration;
    TimeType yellowDuration;
    TimeType redDuration;

    list<pair<TimeType, TrafficLightType> > trafficLightIdsPerTime;
};

class Intersection : public GisObject {
public:
    Intersection(
        GisSubsystem* initSubsystemPtr,
        RoadLayer* initRoadLayerPtr,
        const GisObjectIdType& initObjectId,
        const IntersectionIdType& initIntersectionId,
        const VertexIdType& initVertexId)
        :
        GisObject(initSubsystemPtr, GIS_INTERSECTION, initObjectId, initIntersectionId, initVertexId),
        roadLayerPtr(initRoadLayerPtr),
        hasIntersectionPolygon(false)
    {}

    IntersectionIdType GetIntersectionId() const;

    vector<RoadIdType> GetConnectedRoadIds() const;
    RoadIdType GetRoadIdTo(const IntersectionIdType& endIntersectionId) const;

    const Vertex& GetVertex() const;
    VertexIdType GetVertexId() const;

    const GisVertex& GetGisVertex() const;
    bool IsTerminated() const;

    const map<GisObjectType, vector<VertexConnection> >& GetConnections() const;

    TrafficLightType GetTrafficLight(
        const TimeType& currentTime,
        const RoadIdType& incomingRoadId) const;

    TimeType CalculateCrossingStartTime(
        const TimeType& currentTime,
        const RoadIdType& roadId,
        const bool leftCrossing,
        const TimeType minWalkDuration) const;

    enum RoadSideEdgeType {
        ROAD_SIDE_LEFT_EDGE,
        ROAD_SIDE_RIGHT_EDGE,

        NUMBER_ROAD_SIDE_EDGE_TYPES,
    };

    struct Roadside {
        Vertex edges[NUMBER_ROAD_SIDE_EDGE_TYPES];

        Roadside(
            const Vertex& initLeftEdge,
            const Vertex& initRightEdge)
        {
            edges[ROAD_SIDE_LEFT_EDGE] = initLeftEdge;
            edges[ROAD_SIDE_RIGHT_EDGE] = initRightEdge;
        }
    };

    const vector<Roadside>& GetAntiClockwiseRoadsides() const { return antiClockwiseRoadsides; }
    size_t GetRoadsideNumber(const RoadIdType& roadId) const;
    bool ContainsRoadside(const RoadIdType& roadId) const;

    bool CanPassRoad(
        const RoadIdType& incomingRoadId,
        const RoadIdType& outgoingRoadId) const;

    const RoadTurnType& GetRoadTurnType(
        const RoadIdType& incomingRoadId,
        const RoadIdType& outgoingRoadId) const;

    bool PedestrianCanPass() const;

private:
    friend class RoadLayer;
    RoadLayer* roadLayerPtr;

    bool hasIntersectionPolygon;

    vector<Roadside> antiClockwiseRoadsides;
    map<RoadIdType, size_t> roadsideNumberPerRoadId;

    map<RoadIdType, TrafficLightIdType> trafficLightIds;
    map<pair<RoadIdType, RoadIdType>, RoadTurnType> roadTurnTypes;
};

class BusStop : public GisObject {
public:
    BusStop(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const BusStopIdType& initBusStopId)
        :
        GisObject(initSubsystemPtr, GIS_BUSSTOP, initObjectId, initBusStopId),
        humanCapacity(INT_MAX)
    {}

    const Vertex& GetVertex() const { return position; }
    VertexIdType GetVertexId() const;

    VertexIdType GetLineVertexId(
        const BusLineIdType& lineId,
        const RouteIdType& routeId) const;

    VertexIdType GetNearestEntranceVertexId(const Vertex& position) const;

    bool IsBusStopVertex(const VertexIdType& vertexId) const;

    virtual void UpdateMinRectangle() const;
    virtual bool IntersectsWith(const Rectangle& rect) const;

    void CompleteEntrances(const size_t defaultNumberEntrances);
    bool HasEntrance() const { return !entranceIds.empty(); }

    Vertex AddEntrance(const EntranceIdType& entranceId, const Vertex& point);

    const set<EntranceIdType>& GetEntranceIds() const { return entranceIds; }

    int GetHumanCapacity() const { return humanCapacity; }

private:
    friend class RoadLayer;

    Vertex position;

    set<EntranceIdType> entranceIds;

    int humanCapacity;
};

class RailRoad : public GisObject {
public:
    RailRoad(
        GisSubsystem* initSubsystemPtr,
        RailRoadLayer* initRailRoadLayerPtr,
        const GisObjectIdType& initObjectId,
        const RailRoadIdType& initRailRoadId)
        :
        GisObject(initSubsystemPtr, GIS_RAILROAD, initObjectId, initRailRoadId),
        railRoadLayerPtr(initRailRoadLayerPtr),
        type(RAILROAD_OTHER)
    {}

    RailRoadIdType GetRailRoadId() const;
    RailRoadType GetRailRoadType() const { return type; }

    virtual void UpdateMinRectangle() const;
    virtual bool IntersectsWith(const Rectangle& rect) const;

    virtual void SetEnabled(const bool enable);

private:
    friend class RailRoadLayer;
    RailRoadLayer* railRoadLayerPtr;

    RailRoadType type;

    deque<pair<RailVertexIdType, RailRoadStationIdType> > vertices;

    set<RailRoadLineIdType> railRoadLineIds;
};

class RailRoadIntersection : public GisObject {
public:
    RailRoadIntersection(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const RailRoadIntersectionIdType& initRailRoadIntersectionId,
        const VertexIdType& initVertexId)
        :
        GisObject(initSubsystemPtr, GIS_RAILROAD_INTERSECTION, initObjectId, initRailRoadIntersectionId, initVertexId)
    {}

    const vector<VertexConnection>& GetRailRoadConnections() const;

private:
    friend class RailRoadLayer;
};

class RailRoadStation : public GisObject {
public:
    RailRoadStation(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const RailRoadStationIdType& initRailRoadStationId)
        :
        GisObject(initSubsystemPtr, GIS_RAILROAD_STATION, initObjectId, initRailRoadStationId),
        humanCapacity(INT_MAX)
    {}

    const Vertex& GetVertex() const { return centerPosition; }
    RailRoadStationIdType GetStationId() const;

    const vector<Vertex>& GetPolygon() const { return polygon; }

    virtual void UpdateMinRectangle() const;
    virtual bool IntersectsWith(const Rectangle& rect) const;

    bool HasLineVertex(
        const RailRoadLineIdType& lineId,
        const RouteIdType& routeId) const;

    VertexIdType GetLineVertexId(
        const RailRoadLineIdType& lineId,
        const RouteIdType& routeId) const;

    VertexIdType GetNearestEntranceVertexId(const Vertex& position) const;

    bool IsStationVertex(const VertexIdType& vertexId) const {
        return (lineVertexIds.find(vertexId) != lineVertexIds.end());
    }

    void CompleteEntrances(const size_t defaultNumberEntrances);
    bool HasEntrance() const { return !entranceIds.empty(); }

    Vertex AddEntrance(const EntranceIdType& entranceId, const Vertex& point);

    const set<EntranceIdType>& GetEntranceIds() const { return entranceIds; }

    int GetHumanCapacity() const { return humanCapacity; }

private:
    friend class RailRoadLayer;

    void AddRailRoadConnection(
        const RailRoadLineIdType& lineId,
        const RouteIdType& routedId,
        const Vertex& railRoadVertex);

    Vertex centerPosition;
    vector<Vertex> polygon;

    set<RailVertexIdType> railVertexIds;
    set<VertexIdType> lineVertexIds;
    map<pair<RailRoadLineIdType, RouteIdType>, VertexIdType> vertexIdPerLine;

    set<EntranceIdType> entranceIds;

    int humanCapacity;
};

class Area : public GisObject {
public:
    Area(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const AreaIdType& initAreaId)
        :
        GisObject(initSubsystemPtr, GIS_AREA, initObjectId, initAreaId)
    {}

    virtual void UpdateMinRectangle() const;
    virtual bool IntersectsWith(const Rectangle& rect) const;

    double CalculateSize() const;

    const vector<Vertex>& GetPolygon() const { return polygon; }

private:
    friend class AreaLayer;

    vector<Vertex> polygon;
};

class Park : public GisObject {
public:
    Park(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const ParkIdType& initParkId)
        :
        GisObject(initSubsystemPtr, GIS_PARK, initObjectId, initParkId),
        humanCapacity(INT_MAX)
    {}

    ParkIdType GetParkId() const;

    Vertex GetRandomPosition(HighQualityRandomNumberGenerator& aRandomNumberGenerator) const;
    VertexIdType GetNearestEntranceVertexId(const Vertex& position) const;
    void GetNearEntranceVertexIds(const Vertex& position, vector<VertexIdType>& vertexIds) const;
    RoadIdType GetNearestEntranceRoadId(const Vertex& position) const;

    double CalculateSize() const;

    virtual void UpdateMinRectangle() const;
    virtual bool IntersectsWith(const Rectangle& rect) const;

    const vector<Vertex>& GetPolygon() const { return polygon; }

    void CompleteEntrances(const size_t defaultNumberEntrances);
    bool HasEntrance() const { return !entranceIds.empty(); }

    Vertex AddEntrance(const EntranceIdType& entranceId, const Vertex& point);

    const set<EntranceIdType>& GetEntranceIds() const { return entranceIds; }

    int GetHumanCapacity() const { return humanCapacity; }

private:
    friend class ParkLayer;

    vector<Vertex> polygon;

    set<EntranceIdType> entranceIds;

    int humanCapacity;
};

class Wall : public GisObject {
public:
    Wall(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const WallIdType& initWallId)
        :
        GisObject(initSubsystemPtr, GIS_WALL, initObjectId, initWallId),
        widthMeters(0),
        heightMeters(0),
        materialId(INVALID_MATERIAL_ID)
    {}

    virtual void UpdateMinRectangle() const;
    virtual bool IntersectsWith(const Rectangle& rect) const;

    const Material& GetMaterial() const;

    double GetHeightMeters() const { return heightMeters; }
    const vector<Vertex>& GetWallVertices() const { return vertices; }
    vector<Vertex> MakeWallPolygon() const;

private:
    friend class BuildingLayer;
    BuildingIdType buildingId;

    double widthMeters;
    double heightMeters;

    vector<Vertex> vertices;
    MaterialIdType materialId;
};

class Building : public GisObject {
public:
    Building(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const BuildingIdType& initBuildingId)
        :
        GisObject(initSubsystemPtr, GIS_BUILDING, initObjectId, initBuildingId),
        outerWallMaterialId(INVALID_MATERIAL_ID),
        roofWallMaterialId(INVALID_MATERIAL_ID),
        floorWallMaterialId(INVALID_MATERIAL_ID),
        numberOfRoofFaces(0),
        numberOfWallFaces(0),
        numberOfFloorFaces(0),
        humanCapacity(INT_MAX)
    {}

    BuildingIdType GetBuildingId() const;

    const Material& GetOuterWallMaterial() const;
    const Material& GetRoofMaterial() const;
    const Material& GetFloorMaterial() const;

    const vector<WallIdType>& GetWallIds() const { return wallIds; }

    double GetHeightMeters() const { return heightMeters; }

    Vertex GetRandomPosition(HighQualityRandomNumberGenerator& aRandomNumberGenerator) const;
    VertexIdType GetNearestEntranceVertexId(const Vertex& position) const;
    void GetNearEntranceVertexIds(const Vertex& position, vector<VertexIdType>& vertexIds) const;
    RoadIdType GetNearestEntranceRoadId(const Vertex& position) const;

    virtual void UpdateMinRectangle() const;
    virtual bool IntersectsWith(const Rectangle& rect) const;

    double CalculateSize() const;

    const vector<Vertex>& GetBuildingPolygon() const { return polygon; }

    void CompleteEntrances(const size_t defaultNumberEntrances);
    bool HasEntrance() const { return !entranceIds.empty(); }

    Vertex AddEntrance(const EntranceIdType& entranceId, const Vertex& point);

    const set<EntranceIdType>& GetEntranceIds() const { return entranceIds; }

    const bool Is3DMesh() const { return !(faces.empty()); }
    const vector<Triangle>& GetFaces() const { return faces; }
    const int GetNumberOfRoofFaces() const { return numberOfRoofFaces; }
    const int GetNumberOfWallFaces() const { return numberOfWallFaces; }
    const int GetNumberOfFloorFaces() const { return numberOfFloorFaces; }

    int GetHumanCapacity() const { return humanCapacity; }

    double GetRoofTopHeightMeters() const { assert(!polygon.empty()); return polygon.front().z + heightMeters; }

private:
    friend class BuildingLayer;

    vector<Vertex> polygon;
    vector<WallIdType> wallIds;

    double heightMeters;
    MaterialIdType outerWallMaterialId;
    MaterialIdType roofWallMaterialId;
    MaterialIdType floorWallMaterialId;

    vector<RoadIdType> garageRoadIds;

    set<EntranceIdType> entranceIds;

    vector<Triangle> faces;
    int numberOfRoofFaces;
    int numberOfWallFaces;
    int numberOfFloorFaces;

    int humanCapacity;
};

class GenericGisObject : public GisObject {
public:
    GenericGisObject(
        GisSubsystem* initSubsystemPtr,
        const GisObjectType& initObjectType,
        const GisObjectIdType& initObjectId,
        const GenericGisObjectIdType& initGenericGisObjectId)
        :
        GisObject(initSubsystemPtr, initObjectType, initObjectId, initGenericGisObjectId)
    {}
private:
    friend class GenericGisLayer;
};

class PedestrianPath : public GisObject {
public:
    PedestrianPath(
        GisSubsystem* initSubsystemPtr,
        const GisObjectIdType& initObjectId,
        const PedestrianPathIdType& initPathId);
private:
    friend class GisSubsystem;
};

// obsolete class compatibility
typedef Road RoadObject;
typedef Intersection RoadIntersectionObject;
typedef RailRoad RailRoadObject;
typedef RailRoadIntersection RailRoadIntersectionObject;
typedef RailRoadStation StationObject;
typedef Park ParkObject;
typedef Building BuildingObject;
typedef Wall WallObject;

//------------------------------------------------------------------------------
// Layer
//------------------------------------------------------------------------------

class SpatialObjectMap {
public:
    SpatialObjectMap();

    void SetMesh(
        const Rectangle& initMinRect,
        const double& initMeshUnit,
        const size_t maxMeshSize = MAX_MESH_SIZE);

    void InsertGisObject(
        const GisObject& gisObject,
        const VariantIdType& variantId);

    void RemoveGisObject(
        const GisObject& gisObject,
        const VariantIdType& variantId);

    void InsertVertex(
        const Vertex& vertex,
        const VariantIdType& variantId);

    void GetGisObject(
        const Rectangle& targetRect,
        list<VariantIdType>& variantIds) const;

    void GetGisObject(
        const Vertex& pos,
        list<VariantIdType>& variantIds) const;

    void Clear() { ids.clear(); }

    bool IsAvailable() const { return !ids.empty(); }

    static const size_t MAX_MESH_SIZE;
    static const size_t MAX_VERTEX_MESH_SIZE;

private:
    size_t GetMinHorizontalId(const Rectangle& rect) const;
    size_t GetMaxHorizontalId(const Rectangle& rect) const;
    size_t GetMinVerticalId(const Rectangle& rect) const;
    size_t GetMaxVerticalId(const Rectangle& rect) const;
    size_t GetHorizontalId(const Vertex& vertex) const;
    size_t GetVerticalId(const Vertex& vertex) const;

    Rectangle minRect;
    double meshUnit;

    size_t numberHorizontalMeshes;
    size_t numberVerticalMeshes;

    // TBD: grid list -> grid quadtree
    vector<set<VariantIdType> > ids;
};

class PoiLayer {
public:
    PoiLayer(GisSubsystem* initSubsystemPtr)
        :
        subsystemPtr(initSubsystemPtr)
    {}

    void ImportPoi(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);

    const Point& GetPoint(const PointIdType& pointId) const { return points[pointId]; }

    const Poi& GetPoi(const PoiIdType& poiId) const {
        return pois.at(poiId);
    }

    Poi& GetPoi(const PoiIdType& poiId) {
        return pois.at(poiId);
    }

    Point& GetPoint(const PointIdType& pointId) { return points[pointId]; }
    const vector<Point>& GetPoints() const { return  points; }

    vector<Poi>& GetPois() { return pois; }
    const vector<Poi>& GetPois() const { return pois; }

private:
    GisSubsystem* subsystemPtr;

    vector<Point> points;
    vector<Poi> pois;
};

class RoadLayer {
public:
    RoadLayer(
        const ParameterDatabaseReader& parameterDatabaseReader,
        GisSubsystem* initSubsystemPtr);

    void ImportIntersection(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);
    void ImportRoad(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);
    void ImportTrafficLight(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);
    void ImportBusStop(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);

    void CreateIntersectionIfNecessary(
        const Vertex& point,
        const string& name = string());

    const Intersection& GetIntersection(const IntersectionIdType& intersectionId) const {
        return intersections.at(intersectionId);
    }
    const Road& GetRoad(const RoadIdType& roadId) const {
        return roads.at(roadId);
    }
    const BusStop& GetBusStop(const BusStopIdType& busStopId) const {
        return busStops.at(busStopId);
    }

    const TrafficLight& GetTrafficLight(const TrafficLightIdType& trafficLightId) const {
        return trafficLights.at(trafficLightId);
    }

    Intersection& GetIntersection(const IntersectionIdType& intersectionId) {
        return intersections.at(intersectionId);
    }
    Road& GetRoad(const RoadIdType& roadId) {
        return roads.at(roadId);
    }
    BusStop& GetBusStop(const BusStopIdType& busStopId) {
        return busStops.at(busStopId);
    }
    TrafficLight& GetTrafficLight(const TrafficLightIdType& trafficLightId) {
        return trafficLights.at(trafficLightId);
    }

    bool ContainsBusStop(const string& name) const;

    BusLineIdType GetBusLineId(const string& lineName) const;
    BusStopIdType GetBusStopId(const string& name) const;

    void GetRoadIdsAt(
        const Vertex& position,
        vector<RoadIdType>& roadIds) const;

    void SetIntersectionMarginAndMakeLaneConnection();

    void SearchNearestIntersectionId(
        const Vertex& position,
        const Rectangle& searchRect,
        const set<RoadType>& availableRoadTypes, //throw empty to enable all road type.
        IntersectionIdType& nearestIntersectionId,
        bool& success) const;

    void FindRoadAt(
        const Vertex& position,
        bool& found,
        RoadIdType& roadId) const;

    void FindIntersectionAt(
        const Vertex& position,
        const double radius,
        bool& found,
        IntersectionIdType& intersectionId) const;

    void GetIntersectionIds(
        const Rectangle& searchRect,
        const set<RoadType>& availableRoadTypes, //throw empty to enable all road type.
        list<IntersectionIdType>& intersectionIds) const;

    const vector<Intersection>& GetIntersections() const { return intersections; }
    const vector<Road>& GetRoads() const { return roads; }
    const vector<BusStop>& GetBusStops() const { return busStops; }
    vector<BusStop>& GetBusStops() { return busStops; }

    void CreateIntersection(
        const VertexIdType& vertexId,
        const GisObjectIdType& objectId);

    void DivideRoad(
        const RoadIdType& srcRoadId,
        const VertexIdType& newVertexId);

    void MakeDirectPathToPoi(
        const RoadIdType& srcRoadId,
        const Vertex& position,
        RoadIdType& roadType,
        VertexIdType& intersectionVertexId);

    void AddBusStopVertex(
        const BusStopIdType& busStopId,
        const VertexIdType& vertexId);

    void MakeGarageRoad(
        const Vertex& position);

    bool IsParking(const VertexIdType& vertexId) const;

    void CreateParking(const VertexIdType& vertexId, const GisObjectIdType& objectId);

    bool IsRightHandTrafficRoad() const { return isRightHandTraffic; }

    size_t GetNumberOfEntrancesToBuilding() const { return numberEntrancesToBuilding; }
    size_t GetNumberOfEntrancesToStation() const { return numberEntrancesToStation; }
    size_t GetNumberOfEntrancesToBusStop() const { return numberEntrancesToBusStop; }
    size_t GetNumberOfEntrancesToPark() const { return numberEntrancesToPark; }

    RoadIdType AddSimplePath(
        const VertexIdType& vertexId1,
        const VertexIdType& vertexId2,
        const RoadType& roadType,
        const double widthMeters = DEFAULT_ROAD_WIDTH_METERS,
        const double capacity = 10000.);

    void AssignBusLine(
        const string& lineName,
        const deque<string>& busStopNames);

    RouteIdType GetRouteId(
        const BusLineIdType& lineId,
        const deque<BusStopIdType>& busStopIds) const;

    const deque<BusStopIdType>& GetRouteBusStopIds(
        const BusLineIdType& lineId,
        const RouteIdType& routeId) const;

    double GetMaxRoadWidthMeters() const { return maxRoadWidthMeters; }

    struct PeriodicTrafficLightPatternData {
        TimeType greenDuration;
        TimeType yellowDuration;
        TimeType redDuration;

        PeriodicTrafficLightPatternData()
            :
            greenDuration(ZERO_TIME),
            yellowDuration(ZERO_TIME),
            redDuration(ZERO_TIME)
        {}

        PeriodicTrafficLightPatternData(
            const TimeType& initGreenDuration,
            const TimeType& initYellowDuration,
            const TimeType& initRedDuration)
            :
            greenDuration(initGreenDuration),
            yellowDuration(initYellowDuration),
            redDuration(initRedDuration)
        {}
    };

    struct DistributedTrafficLightPatternData {
        map<TimeType, TrafficLightType> trafficLightPerTime;
    };

    void ReadTrafficLightPatternFileIfNecessary(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        map<string, PeriodicTrafficLightPatternData>& periodicTrafficLightPatterns,
        map<string, DistributedTrafficLightPatternData>& distributedTrafficLightPattern);

    void SyncTrafficLight(const TimeType& currentTime);

private:
    friend class RoadLosChecker;

    void SetIntersectionMargin(
        const IntersectionIdType& intersectionId,
        const map<double, vector<RoadIdType> >& roadIdsPerRadians);

    void MakeLaneConnection(
        const IntersectionIdType& intersectionId);

    string GetLineName(const RailRoadLineIdType& lineId) const;

    void AddTrafficLight(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GisObjectIdType& objectId,
        const VertexIdType& vertexId,
        const map<TimeType, TrafficLightType>& trafficLightPerTime,
        const TimeType& startOffset,
        const TimeType& greenDuration,
        const TimeType& yellowDuration,
        const TimeType& redDuration,
        map<TimeType, list<TrafficLightIdType> >& trafficLightIdsPerTime);

    GisSubsystem* subsystemPtr;

    vector<Intersection> intersections;
    vector<Road> roads;
    vector<BusStop> busStops;
    vector<TrafficLight> trafficLights;

    bool isRightHandTraffic;
    bool breakDownCurvedRoads;
    size_t numberEntrancesToBuilding;
    size_t numberEntrancesToStation;
    size_t numberEntrancesToBusStop;
    size_t numberEntrancesToPark;
    bool setIntersectionMargin;

    double maxRoadWidthMeters;

    struct RouteInfo {
        deque<BusStopIdType> busStopIds;
    };

    map<string, BusLineIdType> busLineIds;
    vector<vector<RouteInfo> > routeInfosPerLine;

    list<pair<TimeType, list<TrafficLightIdType> > > trafficLightChanges;
};

class RailRoadLayer {
public:
    RailRoadLayer(GisSubsystem* initSubsystemPtr)
        :
        subsystemPtr(initSubsystemPtr)
    {}

    struct RailLink {
        RailVertexIdType railVertexId;
        RailRoadIdType railId;

        RailLink()
            :
            railVertexId(INVALID_VERTEX_ID),
            railId(INVALID_VARIANT_ID)
        {}

        RailLink(
            const RailVertexIdType& initRailVertexId,
            const RailRoadIdType& initRailId = INVALID_VARIANT_ID)
            :
            railVertexId(initRailVertexId),
            railId(initRailId)
        {}
    };

    void ImportRailRoad(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);
    void ImportStation(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);

    const RailRoad& GetRailRoad(const RailRoadIdType& railRoadId) const { return railRoads[railRoadId]; }
    const RailRoadIntersection& GetRailRoadIntersection(const RailRoadIntersectionIdType& railRoadIntersectionId) const { return intersections[railRoadIntersectionId]; }
    const RailRoadStation& GetStation(const RailRoadStationIdType& stationId) const { return stations[stationId]; }

    RailRoad& GetRailRoad(const RailRoadIdType& railRoadId) { return railRoads[railRoadId]; }
    RailRoadIntersection& GetRailRoadIntersection(const RailRoadIntersectionIdType& railRoadIntersectionId) { return intersections[railRoadIntersectionId]; }
    RailRoadStation& GetStation(const RailRoadStationIdType& stationId) { return stations[stationId]; }

    const vector<RailRoadIntersection>& GetIntersections() const { return intersections; }
    const vector<RailRoad>& GetRailRoads() const { return railRoads; }
    const vector<RailRoadStation>& GetStations() const { return stations; }
    vector<RailRoadStation>& GetStations() { return stations; }

    RailRoadLineIdType GetRailRoadLineId(const string& lineName) const;
    RailRoadStationIdType GetStationId(
        const string& lineName,
        const string& stationName) const;

    void GetStationIds(const string& stationName, vector<RailRoadStationIdType>& stationIds) const;

    void GetLineVertices(
        const RailRoadLineIdType& lineId,
        const RouteIdType& routeId,
        vector<vector<Vertex> >& lineVertices) const;

    string GetLineName(const RailRoadLineIdType& lineId) const;
    map<string, RailRoadLineIdType> GetRailRoadLineIds() const { return railRoadLineIds; }

    void MakeEntrance(
        const RailRoadStationIdType& stationId,
        const Vertex& position);

    void ComplementLineInfo();

    void GetNearestStationId(
        const Vertex& position,
        bool& found,
        RailRoadStationIdType& nearStationId) const;

    void AssignRailRoadLine(
        const string& lineName,
        const deque<string>& stationNames,
        map<pair<RailRoadStationIdType, RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > >& sectionCandidatesCache,
        map<string, pair<deque<RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > > >& stationIdsAndRailCache);

    void GetShortestRailroadPath(
        const vector<vector<RailRoadStationIdType> >& stationIdsPerName,
        deque<RailRoadStationIdType>& bestStationIds,
        vector<pair<deque<RailLink>, double> >& bestSections,
        set<pair<RailRoadStationIdType, RailRoadStationIdType> >& noConnections,
        map<pair<RailRoadStationIdType, RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > >& sectionCandidatesCache);

    RouteIdType GetRouteId(
        const RailRoadLineIdType& lineId,
        const deque<RailRoadStationIdType>& stationIds) const;

    bool ContainsRouteId(
        const RailRoadLineIdType& lineId,
        const deque<RailRoadStationIdType>& stationIds) const;

    bool LineHasARoute(const string& lineName) const;

    const deque<RailRoadStationIdType>& GetRouteStationIds(
        const RailRoadLineIdType& lineId,
        const RouteIdType& routeId) const;

    bool IsRailRoadLineAvailable(const RailRoadLineIdType& lineId) const;

private:
    friend class RailRoad;
    GisSubsystem* subsystemPtr;

    vector<RailRoadIntersection> intersections;
    vector<RailRoad> railRoads;
    vector<RailRoadStation> stations;

    struct RouteInfo {
        deque<RailRoadStationIdType> stationIds;
        deque<deque<RailVertexIdType> > railVertexIdsPerSection;

        RouteInfo() {}
    };

    class RailNode;

    struct RailVertex {
        Vertex vertex;

        double distance;
        double expectedMinDistanceToDest;
        RailLink trackRailLink;

        vector<RailLink> railLinks;

        void Initialize() {
            distance = -1;
            expectedMinDistanceToDest = DBL_MAX;
        }

        void SetBestRoute(
            const RailLink& initTrackRailLink,
            const double initDistance) {
            trackRailLink = initTrackRailLink;
            distance = initDistance;
        }

        bool IsFastRoute(const double routeDistance) const {
            return ((distance < 0) || (routeDistance < distance));
        }
        bool FoundRoute() const {
            return  (distance > 0);
        }

        RailVertex(const Vertex& initVertex)
            :
            vertex(initVertex),
            distance(-1),
            expectedMinDistanceToDest(DBL_MAX),
            trackRailLink(INVALID_VERTEX_ID)
        {}
    };


    struct RailRoadLineInfo {
        bool isAvailable;
        set<RailRoadIdType> railIds;

        vector<RouteInfo> routeInfos;
        map<string, RailRoadStationIdType> railRoadStationIds;

        RailRoadLineInfo()
            :
            isAvailable(true)
        {}
    };

    vector<RailVertex> railVertices;
    map<Vertex, RailVertexIdType> railVertexIds;

    map<string, RailRoadLineIdType> railRoadLineIds;
    vector<RailRoadLineInfo> lineInfos;

    double CalculateDistance(const pair<deque<RailLink>, double>& section) const;

    void SearchRailRoadRoute(
        const RailVertexIdType& startVertexId,
        const RailVertexIdType& destVertexId,
        deque<RailLink>& routeLinks);

    void UpdateRailRoadLineAvailability(const RailRoadLineIdType& lineId);

    RailVertexIdType GetNewOrExistingRailVertexId(const Vertex& point);
};

class AreaLayer {
public:
    AreaLayer(GisSubsystem* initSubsystemPtr)
        :
        subsystemPtr(initSubsystemPtr)
    {}

    void ImportArea(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);

    const vector<Area>& GetAreas() const { return areas; }
    const Area& GetArea(const AreaIdType& areaId) const { return areas[areaId]; }
    Area& GetArea(const AreaIdType& areaId) { return areas[areaId]; }

private:
    GisSubsystem* subsystemPtr;

    vector<Area> areas;
};

class ParkLayer {
public:
    ParkLayer(GisSubsystem* initSubsystemPtr)
        :
        subsystemPtr(initSubsystemPtr)
    {}

    void ImportPark(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);

    const vector<Park>& GetParks() const { return parks; }
    const Park& GetPark(const ParkIdType& parkId) const { return parks[parkId]; }
    vector<Park>& GetParks() { return parks; }
    Park& GetPark(const ParkIdType& parkId) { return parks[parkId]; }

    void MakeEntrance(
        const ParkIdType& parkId,
        const Vertex& position);

private:
    GisSubsystem* subsystemPtr;

    vector<Park> parks;
};

class BuildingLayer {
public:
    BuildingLayer(
        const ParameterDatabaseReader& parameterDatabaseReader,
        GisSubsystem* initSubsystemPtr);

    void ImportBuilding(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);
    void ImportWall(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);

    const Wall& GetWall(const WallIdType& wallId) const { return walls[wallId]; }
    const Building& GetBuilding(const BuildingIdType& buildingId) const {
        return buildings.at(buildingId);
    }

    Wall& GetWall(const WallIdType& wallId) { return walls[wallId]; }
    Building& GetBuilding(const BuildingIdType& buildingId) {
       return buildings.at(buildingId);
    }

    const vector<Building>& GetBuildings() const { return buildings; }
    const vector<Wall>& GetWalls() const { return walls; }

    vector<Building>& GetBuildings() { return buildings; }

    void GetBuildingIdAt(
        const Vertex& position,
        bool& found,
        BuildingIdType& buildingId) const;

    bool PositionsAreLineOfSight(
        const Vertex& position1,
        const Vertex& position2,
        const set<NodeIdType>& ignoredNodeIds = set<NodeIdType>()/*set src/dest node id to ignore original surfaces*/) const;

    void CalculateWallCollisionPoints(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        vector<pair<Vertex, VariantIdType> >& collisionPoints,
        const set<NodeIdType>& ignoredNodeIds = set<NodeIdType>()/*set src/dest node id to ignore original surfaces*/) const;

    void CalculateWallCollisionPoints(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        vector<pair<Vertex, VariantIdType> >& collisionPoints,
        double& totalShieldingLossDb,
        const set<NodeIdType>& ignoredNodeIds = set<NodeIdType>()/*set src/dest node id to ignore original surfaces*/) const;

    double GetCollisionPointHeight(
        pair<Vertex, VariantIdType>& collisionPoint) const;

    double GetAverageBuildingHeightMeters() const { return averageHeightMeters; }
    double GetMinBuildingHeightMeters() const { return minHeightMeters; }
    double GetMaxBuildingHeightMeters() const { return maxHeightMeters; }
    double GetBuildingHeightMetersVariance() const { return heightMetersVariance; }
    double GetTotalBuildingHeightMeters() const { return totalHeightMeters; }

    size_t GetNumberOfBuildings() const { return buildings.size(); }

    void CalculateNumberOfFloorsAndWallsTraversed(
        const Vertex& vector1,
        const Vertex& vector2,
        const set<NodeIdType>& ignoredNodeIds,
        unsigned int& numberOfFloors,
        unsigned int& numberOfWalls) const;

    double CalculateTotalWallAndFloorLossDb(
        const Vertex& position1,
        const Vertex& position2) const;

    size_t CalculateNumberOfWallRoofFloorInteractions(
        const Vertex& position1,
        const Vertex& position2) const;

    Rectangle GetMinRectangle() const;

    void MakeEntrance(
        const BuildingIdType& buildingId,
        const Vertex& position);

    void AddMovingObject(
        const MaterialSet& materials,
        const NodeIdType nodeId,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        const string& shapeType);

    void RemoveMovingObject(const NodeIdType nodeId);

    void RemakeLosTopology();

    bool EnabledLosCalculation() const { return enabledLineOfSightCalculation; }

    void SyncMovingObjectTime(const TimeType& currentTime);

    shared_ptr<Los::LosQuadTree> GetQuadTreePtr() const { return theQuadTreePtr; }

private:
    void LoadMovingObjectShapeDefinition(
       const ParameterDatabaseReader& parameterDatabaseReader);

    void AddRectangularShapeTemplate(
        const string& shapeType,
        const double& length,
        const double& width,
        const double& height,
        const double shieldingLossDb);

    void DeleteMovingObjectPolygon(Los::LosMovingObject& movingObject);
    void UpdateMovingObjectPolygon(Los::LosMovingObject& movingObject);

    GisSubsystem* subsystemPtr;
    bool enabledLineOfSightCalculation;

    vector<Wall> walls;
    vector<Building> buildings;

    double totalHeightMeters;
    double averageHeightMeters;
    double minHeightMeters;
    double maxHeightMeters;
    double heightMetersVariance;

    //boost::mutex heightMetersVarianceMutex;

    shared_ptr<Los::LosQuadTree> theQuadTreePtr;

    struct MovingShape {
        double length;
        double width;
        double height;
        string materialName;
    };

    map<string, MovingShape> movingShapes;

    list<shared_ptr<Los::LosMovingObject> > losMovingObjects;
};

typedef uint32_t GroundObstructionType;
enum {
    GROUND_OBSTRUCTION_NOTHING = 0,

    GROUND_OBSTRUCTION_TREE = (1 << 0),
    GROUND_OBSTRUCTION_BASIC_BUILDING = (1 << 1),
};

struct GroundMesh {
    // Note using float's for space reasons.

    float pointX;
    float pointY;
    float heightMetersAboveSeaLevel;

    float obstructionHeightMetersFromGround;
    GroundObstructionType obstructionTypes;

    list<int> triangleIds;

    GroundMesh()
        :
        pointX(.0),
        pointY(.0),
        heightMetersAboveSeaLevel(.0),
        obstructionHeightMetersFromGround(.0),
        obstructionTypes(GROUND_OBSTRUCTION_NOTHING)
    {}

    bool IsTree() const { return ((obstructionTypes & GROUND_OBSTRUCTION_TREE) == GROUND_OBSTRUCTION_TREE); }

    bool IsBuilding() const { return ((obstructionTypes & GROUND_OBSTRUCTION_BASIC_BUILDING) == GROUND_OBSTRUCTION_BASIC_BUILDING); }

    double GetElevationtMetersAt(const bool isRoofTopElevation = false) const {

        if (isRoofTopElevation) {
            return (*this).GetRoofTopHeightWithGroundElevationtMetersAt();
        }

        return heightMetersAboveSeaLevel;
    }

    double GetRoofTopHeightWithGroundElevationtMetersAt() const {
        return heightMetersAboveSeaLevel + obstructionHeightMetersFromGround;
    }


};

class GroundLayer {
public:
    GroundLayer(
        const ParameterDatabaseReader& parameterDatabaseReader,
        GisSubsystem* initSubsystemPtr);

    void ImportGroundSurface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);

    void ImportTree(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);

    double GetElevationMetersAt(const Vertex& pos, const bool isRoofTopElevation = false) const;

    void GetSeriallyCompleteElevationPoints(
        const Vertex txPos,
        const Vertex rxPos,
        const int numberDivisions,
        const bool addBuildingHeightToElevation,
        const bool addTreeHeightToElevation,
        vector<Vertex>& points) const;
    
    double GetRoofTopHeightWithGroundElevationtMetersAt(const Vertex& pos) const;
    bool HasTree(const Vertex& pos) const;

    void GetGroundElevationCompletedVertices(
        const Vertex& lineEdge1,
        const Vertex& lineEdge2,
        deque<Vertex>& vertices) const;

private:
    typedef int TriangleIdType;
    typedef int32_t MeshIdType;

    MeshIdType CalculateMeshNumberAt(const MeshIdType indexX, const MeshIdType indexY) const;
    MeshIdType CalculateMeshNumberAt(const Vertex& pos) const;
    TriangleIdType GetTriangleIdAt(const Vertex& pos) const;

    GisSubsystem* subsystemPtr;

    Rectangle rect;
    MeshIdType numberVerticalMeshes;
    MeshIdType numberHorizontalMeshes;
    double meshLengthMeters;

    // Note using shared_array for space reasons.

    vector<Triangle> triangles;
    shared_array<GroundMesh> grounMeshes;
};

class GenericGisLayer {
public:
    GenericGisLayer(GisSubsystem* initSubsystemPtr)
        :
        subsystemPtr(initSubsystemPtr)
    {}

    void ImportGenericLayer(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);

    const GenericGisObject& GetGenericGisObject(const GenericGisObjectIdType& genericGisObjectId) const { return gisObjects[genericGisObjectId]; }

private:
    GisSubsystem* subsystemPtr;

    vector<GenericGisObject> gisObjects;
};

//---------------------------------------------------------------------------
//class GisSubsystem
//---------------------------------------------------------------------------

class GisSubsystem {
public:

    class GisChangeEventHandler {
    public:
        virtual ~GisChangeEventHandler() {}

        virtual void GisInformationChanged() = 0;
    };

    GisSubsystem(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngine>& initSimulationEnginePtr);
    ~GisSubsystem() {}

    void LoadShapeFiles(const ParameterDatabaseReader& theParameterDatabaseReader) { /*obsolete function.*/}
    double LatitudeOriginDegrees() const { return latitudeOriginDegrees; }
    double LongitudeOriginDegrees() const { return longitudeOriginDegrees; }

    const GisObject& GetGisObject(const GisObjectIdType& objectId) const;
    const GisObject& GetGisObject(const GisPositionIdType& positionId) const;

    const Vertex& GetVertex(const VertexIdType& vertexId) const;
    const GisVertex& GetGisVertex(const VertexIdType& vertexId) const;

    const Point& GetPoint(const PointIdType& pointId) const;
    const Intersection& GetIntersection(const IntersectionIdType& intersectionId) const;
    const Road& GetRoad(const RoadIdType& roadId) const;
    const Poi& GetPoi(const PoiIdType& poiId) const;
    const RailRoad& GetRailRoad(const RailRoadIdType& railRoadId) const;
    const RailRoadIntersection& GetRailRoadIntersection(const RailRoadIntersectionIdType& railRoadIntersectionId) const;
    const RailRoadStation& GetStation(const RailRoadStationIdType& stationId) const;
    const Area& GetArea(const AreaIdType& areaId) const;
    const Park& GetPark(const ParkIdType& parkId) const;
    const Wall& GetWall(const WallIdType& wallId) const;
    const Building& GetBuilding(const BuildingIdType& buildingId) const;
    const BusStop& GetBusStop(const BusStopIdType& busStopId) const;
    const Entrance& GetEntrance(const EntranceIdType& entranceId) const;
    const TrafficLight& GetTrafficLight(const TrafficLightIdType& trafficLightId) const;

    // get with object id
    const Point& GetPointObject(const GisObjectIdType& objectId) const;
    const Intersection& GetIntersectionObject(const GisObjectIdType& objectId) const;
    const Road& GetRoadObject(const GisObjectIdType& objectId) const;
    const Poi& GetPoiObject(const GisObjectIdType& objectId) const;
    const RailRoad& GetRailRoadObject(const GisObjectIdType& objectId) const;
    const RailRoadIntersection& GetRailRoadIntersectionObject(const GisObjectIdType& objectId) const;
    const RailRoadStation& GetStationObject(const GisObjectIdType& objectId) const;
    const Area& GetAreaObject(const GisObjectIdType& objectId) const;
    const Park& GetParkObject(const GisObjectIdType& objectId) const;
    const Wall& GetWallObject(const GisObjectIdType& objectId) const;
    const Building& GetBuildingObject(const GisObjectIdType& objectId) const;
    bool ContainsObject(const GisObjectIdType& objectId) const;

    GisPositionIdType GetPositionId(const GisObjectIdType& objectId) const;
    GisPositionIdType GetPositionId(
        const Vertex& position,
        const vector<GisObjectType>& prioritizedSearchObjectTypes,
        const double integrationLength = 0.01/*1cm*/) const;

    GisPositionIdType GetPosition(
        const string& name,
        const GisObjectType& objectType = INVALID_OBJECT_TYPE /*any*/) const;

    void GetPositions(
        const string& name,
        vector<GisPositionIdType>& positionIds,
        const GisObjectType& objectType = INVALID_OBJECT_TYPE /*any*/) const;

    bool ContainsPosition(
        const string& name) const;

    MaterialIdType GetMaterialId(const string& materialName) const;
    const Material& GetMaterial(const MaterialIdType& materialId) const;

    //bool PolygonContainsPoint(const vector<VertexIdType>& vertexIds, const Vertex& point) const;
    Vertex CalculateMedianPoint(const list<VertexIdType>& vertexIds) const;

    GisPositionIdType GetConnectedPositionId(
        const VertexIdType& srcVertexId,
        const VertexIdType& destVertexId) const;

    Vertex GetPositionVertex(
        const GisPositionIdType& positionId) const;

    bool HasConnection(
        const VertexIdType& srcVertexId,
        const VertexIdType& destVertexId) const;

    GisPositionIdType GetARandomPosition(
        const GisObjectType& objectType,
        const GisPositionIdType& ignoredDestinationId,
        HighQualityRandomNumberGenerator& aRandomNumberGenerator) const; //obsolete

    void GetARandomPosition(
        const GisObjectType& objectType,
        const set<GisPositionIdType>& ignoredDestinationIds,
        HighQualityRandomNumberGenerator& aRandomNumberGenerator,
        bool& found,
        GisPositionIdType& randomPositionId) const;

    GisObjectIdType GetARandomGisObjectId(
        const GisObjectType& objectType,
        RandomNumberGenerator& aRandomNumberGenerator) const;

    void GetBuildingPositionIdsInArea(
        const AreaIdType& areaId,
        vector<GisPositionIdType>& buildingPositionIds) const;

    void GetBuildingPositionIdsInArea(
        const AreaIdType& areaId,
        const set<GisPositionIdType>& ignoredDestinationIds,
        vector<GisPositionIdType>& buildingPositionIds) const;

    VertexIdType GetNearestVertexId(
        const GisPositionIdType& positionId,
        const Vertex& position) const;

    RoadIdType GetRoadId(
        const VertexIdType& vertexId) const;

    PoiIdType GetPoiId(
        const VertexIdType& vertexId) const;

    bool IsParkingVertex(
        const VertexIdType& vertexId) const;

    RoadIdType GetParkingRoadId(
        const VertexIdType& vertexId) const;

    IntersectionIdType GetIntersectionId(
        const VertexIdType& vertexId) const;

    RailRoadStationIdType GetStationId(
        const VertexIdType& vertexId) const;

    BusStopIdType GetBusStopId(
        const VertexIdType& vertexId) const;

    bool IsIntersectionVertex(
        const VertexIdType& vertexId) const;

    bool IsVertexOf(
        const GisObjectType& objectType,
        const VertexIdType& vertexId) const;

    bool VertexContains(
        const GisPositionIdType& positionId,
        const VertexIdType& vertexId) const;

    double CalculateDistance(
        const VertexIdType& vertexId1,
        const VertexIdType& vertexId2) const;

    shared_ptr<const PoiLayer> GetPoiLayerPtr() const { return poiLayerPtr; }
    shared_ptr<const RoadLayer> GetRoadLayerPtr() const { return roadLayerPtr; }
    shared_ptr<const RailRoadLayer> GetRailRoadLayerPtr() const { return railRoadLayerPtr; }
    shared_ptr<const AreaLayer> GetAreaLayerPtr() const { return areaLayerPtr; }
    shared_ptr<const ParkLayer> GetParkLayerPtr() const { return parkLayerPtr; }
    shared_ptr<const BuildingLayer> GetBuildingLayerPtr() const { return buildingLayerPtr; }
    shared_ptr<const GroundLayer> GetGroundLayerPtr() const { return groundLayerPtr; }
    shared_ptr<const GenericGisLayer> GetGenericLayerPtr(const string& layerName) const;
    double GetGroundElevationMetersAt(const Vertex& pos) const { return groundLayerPtr->GetElevationMetersAt(pos); }
    double GetGroundElevationMetersAt(const double x, const double y) const { return groundLayerPtr->GetElevationMetersAt(Vertex(x, y)); }
    double GetElevationMetersAt(
        const Vertex& pos,
        const GisPositionIdType& positionId) const;

    const vector<GisVertex>& GetVertices() const { return vertices; }

    const vector<Point>& GetPoints() const;
    const vector<Intersection>& GetIntersections() const;
    const vector<Road>& GetRoads() const;
    const vector<Poi>& GetPois() const;
    const vector<BusStop>& GetBusStops() const;
    const vector<Building>& GetBuildings() const;
    const vector<Park>& GetParks() const;
    const vector<Wall>& GetWalls() const;
    const vector<RailRoadStation>& GetStations() const;
    const vector<Area>& GetAreas() const;

    Entrance& GetEntrance(const EntranceIdType& entranceId);

    void GetVertexIds(
        const Rectangle& targetRect,
        list<VertexIdType>& variantIds) const;

    void SynchronizeTopology(const TimeType& currentTime);
    bool ChangedGisTopologyAtThisTime() const { return changedGisTopology; }

    shared_ptr<InsiteGeometry> GetInsiteGeometry() const { return insiteGeometryPtr; }

    void AddGisChangeEventHandler(
        const string& instanceId,
        const shared_ptr<GisChangeEventHandler>& initGisChangeEventHandlerPtr);

    void DeleteGisChangeEventHandler(
        const string& instanceId);

    bool IsVertexPoint(const Vertex& point) const;

    void OutputVertexInformation() const;

    const Rectangle& GetEntireRect() const { return entireRect; }

    bool IsRightHandTrafficRoad() const {
        return roadLayerPtr->IsRightHandTrafficRoad();
    }

   void GetNearEntranceVertexIds(
       const GisPositionIdType& positionId,
       const Vertex& position,
       vector<VertexIdType>& vertexIds) const;

    // obsolete
    size_t GetNumberOfInteractions(
        const Vertex& position1,
        const Vertex& position2) const {
        return buildingLayerPtr->CalculateNumberOfWallRoofFloorInteractions(position1, position2);
    }

    void SetEnabled(
        const GisObjectType& objectType,
        const VariantIdType& variantId,
        const bool isEnable);

    void GetGisObjectIds(
        const Rectangle& targetRect,
        const GisObjectType& objectType,
        list<VariantIdType>& variantIds) const;

    void GetGisObjectIds(
        const Vertex& vertex,
        const GisObjectType& objectType,
        list<VariantIdType>& variantIds) const;

    void GetGisObjectIds(
        const Rectangle& targetRect,
        const list<GisObjectType>& searchObjectTypes,
        list<GisPositionIdType>& positionIds) const;

    void GetGisObjectIds(
        const Vertex& vertex,
        const list<GisObjectType>& searchObjectTypes,
        list<GisPositionIdType>& positionIds) const;

    bool IntersectsWith(
        const GisPositionIdType& positionId,
        const Rectangle& rect) const;

    void RemoveMovingObject(const NodeIdType nodeId);

    void EnableMovingObjectIfNecessary(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const TimeType& currentTime,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        const string& stationTypeString = "");

    void ExecuteEvents(const TimeType& currentTime);
    void SetExecuteEventManually() { executeEventsManually = true; }

    void GetOffsetWaypoints(
        const double offset,
        const vector<Vertex>& srcWaypoints,
        const bool waypointFromAdditionalStartPosition,
        const Vertex& startPosition,
        const bool isBaseGroundLevel,
        deque<Vertex>& waypoints) const;

    void CompleteGroundElevation(deque<Vertex>& waypoints) const;

private:
    typedef int GenericGisLayerIdType;

    VariantIdType GetVariantId(
        const GisObjectType& objectType,
        const GisObjectIdType& objectId) const;

    shared_ptr<const GenericGisLayer> GetGenericLayerPtr(const GenericGisLayerIdType& layerId) const { return genericGisLayerPtrs[layerId]; }
    void LoadLocalMaterials(const string& materialFilePath);

    void CompleteConnections(const ParameterDatabaseReader& theParameterDatabaseReader);
    void ConnectBuildingToRoad(const BuildingIdType& buildingId);
    void GetRoadEntranceCandidates(
        const Rectangle& minSearchRectangle,
        const Vertex& entrancePosition ,
        const set<RoadIdType>& ignoreRoadIds,
        bool& success,
        RoadIdType& entranceRoadId);

    void ScheduleGisEvent(
        const shared_ptr<SimulationEvent>& gisEventPtr,
        const TimeType& eventTime);

    void LoadLineInfo(const string& fileName);

    SimulationEngine* theSimulationEnginePtr;

    bool isDebugMode;
    bool changedGisTopology;
    bool executeEventsManually;

    bool positionInLatlongDegree;
    double latitudeOriginDegrees;
    double longitudeOriginDegrees;
    Rectangle entireRect;

    // vertex is network path vertex
    // network vertex is applicable to route calculation
    vector<GisVertex> vertices;
    vector<PedestrianPath> pedestrianPaths;
    set<GisObjectType> importedGisObjectTypes;

    shared_ptr<PoiLayer> poiLayerPtr;
    shared_ptr<RoadLayer> roadLayerPtr;
    shared_ptr<RailRoadLayer> railRoadLayerPtr;
    shared_ptr<AreaLayer> areaLayerPtr;
    shared_ptr<ParkLayer> parkLayerPtr;
    shared_ptr<BuildingLayer> buildingLayerPtr;
    shared_ptr<GroundLayer> groundLayerPtr;
    map<string, GenericGisLayerIdType> genericGisLayerIds;
    vector<shared_ptr<GenericGisLayer> > genericGisLayerPtrs;

    shared_ptr<InsiteGeometry> insiteGeometryPtr;
    MaterialSet materials;

    vector<SpatialObjectMap> spatialObjectMaps;
    SpatialObjectMap spatialVertexMap;

    map<string, map<GisObjectType, set<VariantIdType> > > positionsPerName;
    map<GisObjectIdType, GisPositionIdType> objectIdMap;

    GisObjectIdType reservedObjectId;

    struct GisEventInfo {
        TimeType eventTime;
        shared_ptr<SimulationEvent> eventPtr;

        GisEventInfo()
            :
            eventTime(),
            eventPtr()
        {}

        GisEventInfo(
            const TimeType& initEventTime,
            const shared_ptr<SimulationEvent>& initEventPtr)
            :
            eventTime(initEventTime),
            eventPtr(initEventPtr)
        {}

        bool operator<(const GisEventInfo& right) const { return eventTime > right.eventTime; }
    };

    priority_queue_stable<GisEventInfo> gisEventInfos;
    map<string, shared_ptr<GisChangeEventHandler> > gisChangeEventHandlerPtrs;

    vector<Entrance> entrances;

    EntranceIdType CreateEntrance(const Vertex& vertex);

    //----------------------------------------------
    //----------------------------------------------

    friend class PoiLayer;
    friend class RoadLayer;
    friend class RailRoadLayer;
    friend class AreaLayer;
    friend class ParkLayer;
    friend class BuildingLayer;
    friend class GroundLayer;
    friend class GenericGisLayer;
    friend class GisObject;
    friend class RailRoad;
    friend class RailRoadStation;
    friend class Building;
    friend class BusStop;
    friend class Park;

    RoadLayer& GetRoadLayer() { return *roadLayerPtr; }

    VertexIdType GetVertexId(const Vertex& baseVertex);

    void CreatePedestrianPathIfPossible(
        const VertexIdType& vertexId1,
        const VertexIdType& vertexId2);

    void ConnectGisObject(
        const VertexIdType& srcVertexId,
        const GisObjectType& objectType,
        const VariantIdType& destVariantId);

    void ConnectGisObject(
        const VertexIdType& srcVertexId,
        const GisObjectType& objectType,
        const VertexIdType& destVertexId,
        const VariantIdType& destVariantId);

    void ConnectBidirectionalGisObject(
        const VertexIdType& vertexId1,
        const GisObjectType& objectType,
        const VertexIdType& vertexId2,
        const VariantIdType& destVariantId);

    void DisconnectGisObject(
        const VertexIdType& srcVertexId,
        const GisObjectType& objectType,
        const VertexIdType& destVertexId,
        const VariantIdType& destVariantId);

    void DisconnectBidirectionalGisObject(
        const VertexIdType& vertexId1,
        const GisObjectType& objectType,
        const VertexIdType& vertexId2,
        const VariantIdType& destVariantId);

    void UnregisterGisObject(
        const GisObject& gisObject,
        const VariantIdType& variantId);

    void RegisterGisObject(
        const GisObject& gisObject,
        const VariantIdType& variantId);

    GisObjectIdType CreateNewObjectId();

    void ImportEntrance(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);

    void ImportPoi(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const string& filePath);

    void GetSpatialIntersectedGisObjectIds(
        const Rectangle& targetRect,
        const GisObjectType& objectType,
        list<VariantIdType>& variantIds) const;

    void GetSpatialIntersectedGisObjectIds(
        const Vertex& vertex,
        const GisObjectType& objectType,
        list<VariantIdType>& variantIds) const;
};

double SquaredXYZDistanceBetweenVertices(
    const Vertex& vertex1,
    const Vertex& vertex2);

double XYZDistanceBetweenVertices(
    const Vertex& vertex1,
    const Vertex& vertex2);

double SquaredXYDistanceBetweenVertices(
    const Vertex& vertex1,
    const Vertex& vertex2);

double XYDistanceBetweenVertices(
    const Vertex& vertex1,
    const Vertex& vertex2);

void CalculateAzimuthAndElevationDegrees(
    const Vertex& firstPoint,
    const Vertex& secondPoint,
    double& azimuthDegrees,
    double& elevationDegrees);

Vertex CalculatePointToLineNearestPosition(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2);

Vertex CalculatePointToHorizontalLineNearestPosition(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2);

double CalculatePointToLineDistance(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2);

Vertex CalculatePointToRectNearestVertex(
    const Vertex& point,
    const Vertex& planeBottomLeft,
    const Vertex& planeBottomRight,
    const Vertex& planeTopRight,
    const Vertex& planeTopLeft);

Vertex CalculatePointToPolygonNearestVertex(
    const Vertex& point,
    const vector<Vertex>& planePolygon);

Vertex CalculatePointTo3dPolygonNearestVertex(
    const Vertex& point,
    const vector<Vertex>& vertices,
    const double polygonHeight);

Vertex CalculateIntersectionPosition(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2);

bool HorizontalLinesAreIntersection(
    const Vertex& lineEdge11,
    const Vertex& lineEdge12,
    const Vertex& lineEdge21,
    const Vertex& lineEdge22);

// -PI to PI
double CalculateFullRadiansBetweenVector(
    const Vertex& vertex1,
    const Vertex& middleVertex,
    const Vertex& vertex2);

// 0 to PI
double CalculateRadiansBetweenVector(
    const Vertex& vertex1,
    const Vertex& middleVertex,
    const Vertex& vertex2);

template <typename RandomAccessVertexContainer> inline
double CalculateArcDistance(const RandomAccessVertexContainer& points) //for vector, deque and others
{
    if (points.size() < 2) {
        return 0;
    }

    double totalDistance = 0;

    for(size_t i = 0; i < points.size() - 1; i++) {
        totalDistance += points[i].DistanceTo(points[i+1]);
    }

    return totalDistance;
}

template <typename RandomAccessVertexContainer> inline
void CalculatePointToArcNearestPosition(
    const RandomAccessVertexContainer& points,
    const Vertex& position,
    Vertex& nearestPosition,
    size_t& vertexNumber)
{
    assert(points.size() >= 2);

    double minDistance = DBL_MAX;

    for(size_t i = 0; i < points.size() - 1; i++) {
        const Vertex& edge1 = points[i];
        const Vertex& edge2 = points[i+1];

        const Vertex roadPosition =
            CalculatePointToLineNearestPosition(position, edge1, edge2);

        const double distance = roadPosition.DistanceTo(position);

        if (distance < minDistance) {
            minDistance = distance;
            nearestPosition = roadPosition;
            vertexNumber = i;
        }
    }
}

template <typename RandomAccessVertexContainer> inline
void CalculatePointToHorizontalArcNearestPosition(
    const RandomAccessVertexContainer& points,
    const Vertex& position,
    Vertex& nearestPosition,
    size_t& vertexNumber)
{
    assert(points.size() >= 2);

    double minDistance = DBL_MAX;

    for(size_t i = 0; i < points.size() - 1; i++) {
        const Vertex& edge1 = points[i];
        const Vertex& edge2 = points[i+1];

        const Vertex roadPosition =
            CalculatePointToHorizontalLineNearestPosition(position, edge1, edge2);

        const double distance = roadPosition.DistanceTo(position);

        if (distance < minDistance) {
            minDistance = distance;
            nearestPosition = roadPosition;
            vertexNumber = i;
        }
    }
}
template <typename RandomAccessVertexContainer> inline
void CalculatePointToArcNearestVertex(
    const RandomAccessVertexContainer& points,
    const Vertex& position,
    Vertex& nearestVertex,
    size_t& vertexNumber)
{
    assert(points.size() >= 2);

    double minDistance = DBL_MAX;

    for(size_t i = 0; i < points.size(); i++) {
        const Vertex& vertex = points[i];
        const double distance = vertex.DistanceTo(position);

        if (distance < minDistance) {
            minDistance = distance;
            nearestVertex = vertex;
            vertexNumber = i;
        }
    }
}


template <typename RandomAccessVertexContainer> inline
double CalculatePolygonSize(const RandomAccessVertexContainer& points)
{
    assert(points.size() >= 2);
    assert(points.front() == points.back());

    double polygonSize = 0;

    for(size_t i = 0; i < points.size() - 1; i++) {
        const Vertex& pointN = points[i];
        const Vertex& pointNp1 = points[i+1];

        polygonSize += (pointN.x - pointNp1.x) * (pointN.y + pointNp1.y);
    }

    return fabs(polygonSize * 0.5);
}

template <typename RandomAccessVertexContainer> inline
Vertex CalculateMedianPoint(const RandomAccessVertexContainer& points)
{
    Vertex vertex;

    for(size_t i = 0; i < points.size(); i++) {
        vertex += points[i];
    }

    return vertex / double(points.size());
}

template <typename RandomAccessVertexContainer> inline
Rectangle GetPointsRect(const RandomAccessVertexContainer& points)
{
    Rectangle minRectangle;

    minRectangle.minX = DBL_MAX;
    minRectangle.minY = DBL_MAX;
    minRectangle.maxX = -DBL_MAX;
    minRectangle.maxY = -DBL_MAX;

    assert(!points.empty());

    for(size_t i = 0; i < points.size(); i++) {
        const Vertex& vertex = points[i];

        minRectangle.minX = std::min(minRectangle.minX, vertex.x);
        minRectangle.minY = std::min(minRectangle.minY, vertex.y);
        minRectangle.maxX = std::max(minRectangle.maxX, vertex.x);
        minRectangle.maxY = std::max(minRectangle.maxY, vertex.y);
    }

    return minRectangle;
}

static inline
Vertex MakeVertexFromMobilityPosition(const ObjectMobilityPosition& mobilityPosition)
{
    return Vertex(
        mobilityPosition.X_PositionMeters(),
        mobilityPosition.Y_PositionMeters(),
        mobilityPosition.HeightFromGroundMeters());
}//MakeVertexFromMobilityPosition//

void PolygonToTriangles(const vector<Vertex>& polygon, vector<Triangle>& triangles);

bool PolygonContainsPoint(
    const vector<Vertex>& vertices,
    const Vertex& point);

Vertex CalculateIntersectionPositionBetweenLine(
    const Vertex& lineEdge11,
    const Vertex& lineEdge12,
    const Vertex& lineEdge21,
    const Vertex& lineEdge22);

void GetIntersectionPositionBetweenLineAndPolygon(
    const Vertex& lineEdge1,
    const Vertex& lineEdge2,
    const vector<Vertex>& polygon,
    vector<Vertex>& intersectionPoints);

template <typename T> inline
void operator+=(vector<T>& left, const vector<T>& right)
{
    left.reserve(left.size() + right.size());

    for(size_t i = 0; i < right.size(); i++) {
        left.push_back(right[i]);
    }
}

string TrimmedString(const string& aString);
void TokenizeToTrimmedLowerString(
    const string& aString,
    const string& deliminator,
    deque<string>& tokens,
    const bool skipEmptyToken = true);

}; //namespace ScenSim

#endif
