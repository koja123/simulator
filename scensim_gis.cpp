// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include <sstream>

#include "shapefil.h"
#include "s_hull.h"

#include "insiteglue_chooser.h"
#include "scensim_gis.h"
#include "scensim_route_search.h"
#include "scensim_mobility.h"
#include "scensim_proploss.h"
#include "scensim_tracedefs.h"

namespace ScenSim {

using std::istringstream;

static const GisObjectIdType RESERVED__GIS_OBJECT_ID = 150000000;

const size_t SpatialObjectMap::MAX_MESH_SIZE = 1000;
const size_t SpatialObjectMap::MAX_VERTEX_MESH_SIZE = 2000;

static const double DEFAULT_MESH_UNIT_METERS = 1000.0;

static const double MAX_STATION_SEARCH_AREA_METERS = 5000;
static const double MAX_INTERSECTION_SEARCH_AREA_METERS = 1000;

//well known field
//Note: case sensitive string
static const string GIS_DBF_ID_STRING = "id";
static const string GIS_DBF_NAME_STRING = "name";
static const string GIS_DBF_WIDTH_STRING = "width";
static const string GIS_DBF_HEIGHT_STRING = "height";
static const string GIS_DBF_ROOF_MATERIAL_STRING = "roofmateri";
static const string GIS_DBF_FLOOR_MATERIAL_STRING = "floormater";
static const string GIS_DBF_MATERIAL_STRING = "material";
static const string GIS_DBF_WIDTHTYPE_STRING = "widthType";
static const string GIS_DBF_TYPE_STRING = "type";
static const string GIS_DBF_START_POINT_ID_STRING = "startPtId";
static const string GIS_DBF_END_POINT_ID_STRING = "endPtId";
static const string GIS_DBF_STATION_ID_STRING = "staId";
static const string GIS_DBF_STATION_NAME_STRING = "staName";
static const string GIS_DBF_BUILDING_ID_STRING = "buildingId";
static const string GIS_DBF_LANE12_STRING = "lane12";
static const string GIS_DBF_LANE21_STRING = "lane21";
static const string GIS_DBF_MESH_KIND_STRING = "meshKind";
static const string GIS_DBF_NUMBER_OF_ROOF_FACES_STRING = "nofRoof";
static const string GIS_DBF_NUMBER_OF_WALL_FACES_STRING = "nofWall";
static const string GIS_DBF_NUMBER_OF_FLOOR_FACES_STRING = "nofFloor";
static const string GIS_DBF_CAPACITY_STRING = "capacity";
static const string GIS_DBF_SPEED_LIMIT_STRING = "speedlimit";
static const string GIS_DBF_INFO_STRING = "info";

static const string GIS_DBF_OFFSET_STRING = "signaloff";
static const string GIS_DBF_GREEN_STRING = "green";
static const string GIS_DBF_YELLOW_STRING = "yellow";
static const string GIS_DBF_RED_STRING = "red";
static const string GIS_DBF_PATTERN_STRING = "pattern";
static const string GIS_DBF_INTERSECTIONID_STRING = "intersecid";
static const string GIS_DBF_OBJECTID_STRING = "objectid";
static const string GIS_DBF_ROADID_STRING = "roadid";

static const string UNKNOWN_OBJECT_NAME = "noname";

enum ShapeType {
    POINT_TYPE,
    ARC_TYPE,
    POLYGON_TYPE
};

static inline
vector<Vertex> GetMiddlePointsOfPolygon(const vector<Vertex>& polygon)
{
    vector<Vertex> middlePoints;

    for(size_t i = 0; i < polygon.size() - 1; i++) {

        const Vertex& edge1 = polygon[i];
        const Vertex& edge2 = polygon[i+1];

        middlePoints.push_back((edge1 + edge2) / 2);
    }

    return middlePoints;
}

string TrimmedString(const string& aString)
{
    string trimmedString = aString;

    size_t pos = trimmedString.find_last_not_of(" \t");
    if (pos == string::npos) {
        trimmedString.clear();
    } else if (pos < (aString.size() - 1)) {
        trimmedString.erase(pos+1);
    }

    pos = trimmedString.find_first_not_of(" \t");

    if (pos != string::npos) {
        trimmedString = trimmedString.substr(pos);
    }
    else {
        // No trailing spaces => don't modify string.
    }

    return trimmedString;
}

void TokenizeToTrimmedLowerString(
    const string& aString,
    const string& deliminator,
    deque<string>& tokens,
    const bool skipEmptyToken)
{
    tokens.clear();

    size_t posOfReading = 0;
    size_t posOfDelim = aString.find_first_of(deliminator);

    while (posOfDelim != string::npos) {

        const string token =
            TrimmedString(aString.substr(posOfReading, posOfDelim - posOfReading));

        if (!skipEmptyToken || !token.empty()) {
            tokens.push_back(token);

            ConvertStringToLowerCase(tokens.back());
        }

        posOfReading = posOfDelim + 1;
        posOfDelim = aString.find_first_of(deliminator, posOfReading);
    }

    const size_t lastToenPos =
        aString.find_first_not_of(" ", posOfReading);

    if (lastToenPos != string::npos) {
        const string lastOneToken = aString.substr(
            lastToenPos,
            aString.find_last_not_of(" ") + 1);

        if (!skipEmptyToken || !lastOneToken.empty()) {
            tokens.push_back(lastOneToken);

            ConvertStringToLowerCase(tokens.back());
        }
    }
}

static inline
bool ElevationBaseIsGroundLevel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GisObjectIdType objectId)
{
    if (theParameterDatabaseReader.ParameterExists("gisobject-elevation-reference-type", objectId)) {

        string elevationBaseString = theParameterDatabaseReader.ReadString("gisobject-elevation-reference-type", objectId);

        ConvertStringToLowerCase(elevationBaseString);


        if (elevationBaseString == "groundlevel") {
            return true;
        } else if (elevationBaseString == "sealevel") {
            return false;
        } else {
            cerr << "Error: Invalid elevation base string" << elevationBaseString << endl;
            exit(1);
        }
    }

    return true;
}

//----------------------------------------------------------
// Shape Handling
//----------------------------------------------------------

class AttributeFinder {
public:
    AttributeFinder()
        :
        dbfHandle(nullptr),
        fieldNumber(-1),
        fieldType(FTInvalid)
    {}

    AttributeFinder(
        const DBFHandle initDbfHandle,
        const string& initAttributeName)
        :
        dbfHandle(initDbfHandle),
        attributeName(initAttributeName),
        fieldNumber(-1)
    {
        string lowerAttributeName = attributeName;
        ConvertStringToLowerCase(lowerAttributeName);

        const int numberAttributes = DBFGetFieldCount(dbfHandle);

        for(int i = 0; i < numberAttributes; i++) {

            char rawAttributeName[12];
            int width;
            int digits;

            DBFFieldType aFeldType =
                DBFGetFieldInfo(dbfHandle, i, rawAttributeName, &width, &digits);

            string fieldNameString = rawAttributeName;
            ConvertStringToLowerCase(fieldNameString);

            if (fieldNameString == lowerAttributeName) {
                fieldType = aFeldType;
                fieldNumber = i;
                break;
            }
        }
    }

    bool IsAvailable() const { return (fieldNumber != -1); }

    double GetDouble(const int shapeId) const {
        (*this).ErrorIfNotAvailable();
        (*this).ReadValue(shapeId);

        if (fieldType == FTDouble) {
            return doubleValue;
        } else if (fieldType == FTInteger) {
            return double(intValue);
        } else {
            bool success;
            double aValue;
            ConvertStringToDouble(stringValue, aValue, success);

            if (!success) {
                cerr << "Wrong format string for double value" << stringValue << endl;
                exit(1);
            }
            return aValue;
        }
    }

    int GetInt(const int shapeId) const {
        (*this).ErrorIfNotAvailable();
        (*this).ReadValue(shapeId);

        if (fieldType == FTInteger) {
            return intValue;
        } else if (fieldType == FTDouble) {
            return int(doubleValue);
        } else {
            bool success;
            int aValue;
            ConvertStringToInt(stringValue, aValue, success);

            if (!success) {
                cerr << "Wrong format string for int value" << stringValue << endl;
                exit(1);
            }
            return aValue;
        }
    }

    const string& GetString(const int shapeId) const {
        (*this).ErrorIfNotAvailable();
        (*this).ReadValue(shapeId);

        if (fieldType == FTDouble) {
            stringValue = ConvertToString(doubleValue);
        } else if (fieldType == FTInteger) {
            stringValue = ConvertToString(intValue);
        }
        return stringValue;
    }
    const string& GetLowerString(const int shapeId) const {
        stringValue = MakeLowerCaseString((*this).GetString(shapeId));
        return stringValue;
    }

    GisObjectIdType GetGisObjectId(const int shapeId) const {
        (*this).ErrorIfNotAvailable();
        (*this).ReadValue(shapeId);

        return GisObjectIdType((*this).GetInt(shapeId));
    }

private:
    void ErrorIfNotAvailable() const {
        if (!(*this).IsAvailable()) {
            cerr << "Error: Attribute " << attributeName << " doesn't exist in DBF" << endl;
            exit(1);
        }
    }

    const DBFHandle dbfHandle;
    const string attributeName;

    int fieldNumber;
    DBFFieldType fieldType;

    void ReadValue(const int shapeId) const {
        if (fieldType == FTDouble) {
            doubleValue = DBFReadDoubleAttribute(dbfHandle, shapeId, fieldNumber);
        } else if (fieldType == FTString) {
            stringValue = DBFReadStringAttribute(dbfHandle, shapeId, fieldNumber);
        } else if (fieldType == FTInteger) {
            intValue = DBFReadIntegerAttribute(dbfHandle, shapeId, fieldNumber);
        }
    }

    mutable string stringValue;
    mutable double doubleValue;
    mutable int intValue;
};

static inline
void GetStandardShapeInfo(
    const string& filePath,
    const bool isDebugMode,
    SHPHandle& hSHP,
    DBFHandle& hDBF,
    int& entities,
    Rectangle& rect)
{
    hSHP = SHPOpen(filePath.c_str(), "rb");
    if (hSHP == nullptr) {
        cerr << "Cannot open .shp file: " << filePath << endl;
        exit(1);
    }

    const size_t dotPos = filePath.find_last_of(".");
    const string fileNamePrefix = filePath.substr(0, dotPos);
    const string dbfFilePath = fileNamePrefix + ".dbf";

    hDBF = DBFOpen(dbfFilePath.c_str(), "rb");
    if (hDBF == NULL) {
        cerr << "Cannot open .dbf file: " << dbfFilePath << endl;
        SHPClose(hSHP);
        exit(1);
    }

    double adfMinBound[4];
    double adfMaxBound[4];
    int shapeFileType;

    SHPGetInfo(hSHP, &entities, &shapeFileType, adfMinBound, adfMaxBound);

    rect.minX = adfMinBound[0];
    rect.minY = adfMinBound[1];
    rect.maxX = adfMaxBound[0];
    rect.maxY = adfMaxBound[1];

    const int fields = DBFGetFieldCount(hDBF);
    const int records = DBFGetRecordCount(hDBF);

    if (isDebugMode) {
        cout << "Loading... " << filePath << endl
             << "Entries: " << entities << ", Fields: " << fields
             << ", Records: " << records << ", GisObjectType: " << GIS_ROAD
             << ", ShapeType: " << shapeFileType << endl;
    }

    assert(entities == records);
}

static inline
Rectangle PeekLayerRectangle(const string& filePath)
{
    const bool isDebugMode = false;
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, isDebugMode, hSHP, hDBF, entities, layerRect);

    SHPClose(hSHP);
    DBFClose(hDBF);

    const double extensionMargin = 100;

    return layerRect.Expanded(extensionMargin);
}

//----------------------------------------------------------
// LoS
//----------------------------------------------------------

namespace Los {

struct LosRay {

    LosRay(const Vertex& initOrig, const Vertex& initDest)
        :
        orig(initOrig),
        dest(initDest),
        dir(dest - orig)
    {
        invDir =  dir.Inverted();
        sign[0] = int(invDir.x < 0);
        sign[1] = int(invDir.y < 0);
        sign[2] = int(invDir.z < 0);
    }

    Vertex Position(const double t) const { return dir*t + orig; }
    Vertex CrossPointX(const double x) const {
        if (dir.x == 0.) {
            return orig;
        }
        return (*this).Position((x - orig.x) / dir.x);
    }
    Vertex CrossPointY(const double y) const {
        if (dir.y == 0.) {
            return orig;
        }
        return (*this).Position((y - orig.y) / dir.y);
    }

    Vertex orig;
    Vertex dest;
    Vertex dir;
    Vertex invDir;

    int sign[3];
};

struct LosPolygon {
    enum ObstructionType { Floor, Roof, OuterWall, InnerWall, Invalid};

    Triangle triangle;
    VariantIdType variantId;
    NodeIdType nodeId;
    double shieldingLossDb;
    ObstructionType anObstructionType;

    LosPolygon() : variantId(INVALID_VARIANT_ID), nodeId(INVALID_NODEID), shieldingLossDb(0) {}

    void SetTriangle(
        const Vertex& v1,
        const Vertex& v2,
        const Vertex& v3) {

        triangle = Triangle(v1, v2, v3);
    }
};


struct WallCollisionInfoType {
    VariantIdType variantId;
    double shieldingLossDb;
    LosPolygon::ObstructionType anObstructionType;

    WallCollisionInfoType(
        const VariantIdType& initVariantId,
        const double& initShieldingLossDb,
        const LosPolygon::ObstructionType& initAnObstructionType)
        :
        variantId(initVariantId),
        shieldingLossDb(initShieldingLossDb),
        anObstructionType(initAnObstructionType)
    {}
};


// Oriented Bounding Box aka OBB
struct LosOrientedBoundingBox {
    LosOrientedBoundingBox() {
        extentPerAxis[0] = 0.;
        extentPerAxis[1] = 0.;
        extentPerAxis[2] = 0.;
    }

    LosOrientedBoundingBox(
        const Vertex& initBttomCenter,
        const RotationMatrix& initRotationMatrix,
        const double& initLength, //y
        const double& initWidth, //x
        const double& initHeight, //z
        const NodeIdType& initNodeId,
        const double& initShieldingLossDb)
        :
        center(initBttomCenter.x, initBttomCenter.y, initBttomCenter.z + initHeight*0.5),
        rotationMatrix(initRotationMatrix),
        nodeId(initNodeId),
        shieldingLossDb(initShieldingLossDb)
    {
        extentPerAxis[0] = initWidth*0.5;
        extentPerAxis[1] = initLength*0.5;
        extentPerAxis[2] = initHeight*0.5;

        assert(initWidth > 0.);
        assert(initLength > 0.);
        assert(initHeight > 0.);

        vector<Vertex> vertices;

        vertices.push_back(rotationMatrix*Vertex(extentPerAxis[0], extentPerAxis[1], extentPerAxis[2]) + center);
        vertices.push_back(rotationMatrix*Vertex(-extentPerAxis[0], extentPerAxis[1], extentPerAxis[2]) + center);
        vertices.push_back(rotationMatrix*Vertex(extentPerAxis[0], -extentPerAxis[1], extentPerAxis[2]) + center);
        vertices.push_back(rotationMatrix*Vertex(extentPerAxis[0], extentPerAxis[1], -extentPerAxis[2]) + center);
        vertices.push_back(rotationMatrix*Vertex(-extentPerAxis[0], -extentPerAxis[1], extentPerAxis[2]) + center);
        vertices.push_back(rotationMatrix*Vertex(extentPerAxis[0], -extentPerAxis[1], -extentPerAxis[2]) + center);
        vertices.push_back(rotationMatrix*Vertex(-extentPerAxis[0], extentPerAxis[1], -extentPerAxis[2]) + center);
        vertices.push_back(rotationMatrix*Vertex(-extentPerAxis[0], -extentPerAxis[1], -extentPerAxis[2]) + center);

        rect = GetPointsRect(vertices);
    }

    const Rectangle& GetRect() const { return rect; }

    Vertex center;
    RotationMatrix rotationMatrix;
    double extentPerAxis[3];
    Rectangle rect;

    NodeIdType nodeId;
    double shieldingLossDb;
};

struct LosQuadTree {
    static const int MAXDEPTH = 6; //Max number of fields: 4^6 (=4096)

    LosQuadTree* child[4]; // child quadrant field

    Rectangle place;
    int depth;

    list<shared_ptr<LosOrientedBoundingBox> > losObbPtrs;
    list<LosPolygon> losPolygons;

    typedef list<shared_ptr<LosOrientedBoundingBox> >::iterator LosObbIter;

    LosQuadTree(const Rectangle& initPlace, const int initDepth)
        :
        place(initPlace),
        depth(initDepth)
    {
        for(int i = 0; i < 4; i++) {
            child[i] = nullptr;
        }
    }

    ~LosQuadTree();

    void PushLosPolygon(const LosPolygon& wall);

    void PushLosPolygonToChild(
        const QuadrantType& childQuadrant,
        const LosPolygon& wall);

    void PushLosOrientedBoundingBox(
        const shared_ptr<LosOrientedBoundingBox>& obbPtr,
        LosQuadTree*& insertedTreePtr,
        LosObbIter& insertedIter);

    void PushLosOrientedBoundingBoxToChild(
        const QuadrantType& childQuadrant,
        const shared_ptr<LosOrientedBoundingBox>& obbPtr,
        LosQuadTree*& insertedTreePtr,
        LosObbIter& insertedIter);

    void CheckCollision(
        const LosRay& ray,
        const set<NodeIdType>& ignoredNodeIds,
        const bool checkJustACollsion,
        const bool isJustHorizontalCheck,
        map<double, WallCollisionInfoType>& collisions);

    void CheckChildCollision(
        const QuadrantType& childQuadrant,
        const LosRay& ray,
        const set<NodeIdType>& ignoredNodeIds,
        const bool checkJustACollsion,
        const bool isJustHorizontalCheck,
        map<double, WallCollisionInfoType>& collisions);

    void CheckLosPolygonCollision(
        const LosRay& ray,
        const set<NodeIdType>& ignoredNodeIds,
        const bool checkJustACollsion,
        const bool isJustHorizontalCheck,
        map<double, WallCollisionInfoType>& collisions);

    void CheckLosOrientedBoundingBoxCollision(
        const LosRay& ray,
        const set<NodeIdType>& ignoredNodeIds,
        const bool checkJustACollsion,
        map<double, WallCollisionInfoType>& collisions);

    bool HasCollision(
        const LosRay& ray,
        const set<NodeIdType>& ignoredNodeIds);
};

struct LosMovingObject {
    NodeIdType nodeId;
    shared_ptr<ObjectMobilityModel> mobilityModelPtr;
    double length;
    double width;
    double height;
    double shieldingLossDb;

    ObjectMobilityPosition lastMobilityPosition;

    typedef list<shared_ptr<LosOrientedBoundingBox> >::iterator LosObbIter;

    LosQuadTree* lastTreePtr;
    LosObbIter lastObbIter;
    shared_ptr<LosOrientedBoundingBox> losObbPtr;

    LosMovingObject(
        const NodeIdType& initNodeId,
        const shared_ptr<ObjectMobilityModel>& initMobilityModelPtr,
        const double initLength,
        const double initWidth,
        const double initHeight,
        const double initShieldingLossDb)
        :
        nodeId(initNodeId),
        mobilityModelPtr(initMobilityModelPtr),
        length(initLength),
        width(initWidth),
        height(initHeight),
        shieldingLossDb(initShieldingLossDb),
        lastMobilityPosition(ZERO_TIME, ZERO_TIME, DBL_MAX, DBL_MAX, DBL_MAX, false/*mobilityContainsGroundHeightMeters*/, INVALID_NODEID),
        lastTreePtr(nullptr),
        losObbPtr(new LosOrientedBoundingBox())
    {}

    void UpdateMovingObjectMobilityPosition(
        const GroundLayer& groundLayer,
        const TimeType& currentTime,
        bool& positionChanged);

    void ReconstructLosPolygon(const GroundLayer& groundLayer);
};

void LosMovingObject::UpdateMovingObjectMobilityPosition(
    const GroundLayer& groundLayer,
    const TimeType& currentTime,
    bool& positionChanged)
{
    ObjectMobilityPosition currentMobilityPosition;
    mobilityModelPtr->GetPositionForTime(currentTime, currentMobilityPosition);

    if (lastMobilityPosition.X_PositionMeters() == currentMobilityPosition.X_PositionMeters() &&
        lastMobilityPosition.Y_PositionMeters() == currentMobilityPosition.Y_PositionMeters() &&
        lastMobilityPosition.HeightFromGroundMeters() == currentMobilityPosition.HeightFromGroundMeters() &&
        lastMobilityPosition.AttitudeAzimuthFromNorthClockwiseDegrees() == currentMobilityPosition.AttitudeAzimuthFromNorthClockwiseDegrees() &&
        lastMobilityPosition.AttitudeElevationFromHorizonDegrees() == currentMobilityPosition.AttitudeElevationFromHorizonDegrees()) {
        positionChanged = false;
        return;
    }

    positionChanged = true;

    lastMobilityPosition = currentMobilityPosition;

    (*this).ReconstructLosPolygon(groundLayer);
}

void LosMovingObject::ReconstructLosPolygon(const GroundLayer& groundLayer)
{
    Vertex baseVertex = MakeVertexFromMobilityPosition(lastMobilityPosition);

    // Adjust ground height if necessary
    if (!lastMobilityPosition.TheHeightContainsGroundHeightMeters()) {
        baseVertex.z += groundLayer.GetElevationMetersAt(baseVertex);
    }

    const double azimuthDegrees =
        lastMobilityPosition.AttitudeAzimuthFromNorthClockwiseDegrees();
    const double elevationDegrees =
        lastMobilityPosition.AttitudeElevationFromHorizonDegrees();

    const RotationMatrix rotationMatrix(-azimuthDegrees, 0., elevationDegrees);

    *losObbPtr = LosOrientedBoundingBox(
        baseVertex, rotationMatrix, length, width, height, nodeId, shieldingLossDb);
}

static inline
void JudgeLinePlane(
    const LosRay& ray,
    const LosPolygon& a,
    double& t)
{
    t = -1;

    const Vertex normalDir = ray.dir.Normalized();
    const Vertex e1 = a.triangle.GetP2() - a.triangle.GetP1();
    const Vertex e2 = a.triangle.GetP3() - a.triangle.GetP1();
    const Vertex pvec = normalDir.Cross(e2);
    const double det = e1.Dot(pvec);

    if (det > -DBL_EPSILON && det < DBL_EPSILON) {
        return;
    }

    const double invDet = 1.0 / det;
    const Vertex tvec = ray.orig - a.triangle.GetP1();
    const double u = tvec.Dot(pvec)*invDet;

    if (u < 0.0 || u > 1.0) {
        return;
    }

    const Vertex qvec = tvec.Cross(e1);
    const double v = normalDir.Dot(qvec)*invDet;

    if (v < 0.0 || u + v > 1.0) {
        return;
    }

    t = (e2.Dot(qvec) * invDet) / ray.dir.Distance();
}

static inline
void JudgeLineOrientedBoundingBox(
    const LosRay& ray,
    const LosOrientedBoundingBox& obb,
    double& outTmin,
    double& outTmax)
{
    outTmin = 0.;
    outTmax = DBL_MAX;

    double minT = -DBL_MAX;
    double maxT = DBL_MAX;

    const Vertex dirToCenter = obb.center - ray.orig;

    for(int i = 0; i < 3; i++) {
        const Vertex& axisDirection = obb.rotationMatrix.cv[i];
        const double e = axisDirection.Dot(dirToCenter);
        const double f = ray.dir.Dot(axisDirection);
        const double extent = obb.extentPerAxis[i];

        // ray is parallel
        if (f > -DBL_EPSILON && f < DBL_EPSILON) {

            // passing outside of OBB
            if (std::fabs(extent) < std::fabs(e))  {
                outTmin = -DBL_MAX;
                outTmax = DBL_MAX;
                return;
            }
            continue;
        }

        double t1 = (e - extent)/f;
        double t2 = (e + extent)/f;
        if (t1 > t2) {
            std::swap(t1, t2);
        }

        assert(t1 <= t2);
        minT = std::max(minT, t1); //close
        maxT = std::min(maxT, t2); //far

        if (maxT <= 0 || minT > maxT) {
            outTmin = -DBL_MAX;
            outTmax = DBL_MAX;
            return;
        }
    }

    outTmin = minT;
    outTmax = maxT;
}

inline
LosQuadTree::~LosQuadTree()
{
    for(int i = 0; i < 4; i++) {
        delete child[i];
    }
}

void LosQuadTree::PushLosPolygon(const LosPolygon& wall)
{
    if (depth < MAXDEPTH) {
        const Triangle& triangle = wall.triangle;
        const QuadrantType p1Quadrant = place.GetQuadrantOf(triangle.GetP1());
        const QuadrantType p2Quadrant = place.GetQuadrantOf(triangle.GetP2());
        const QuadrantType p3Quadrant = place.GetQuadrantOf(triangle.GetP3());

        if ((p1Quadrant == p2Quadrant) &&
            (p2Quadrant == p3Quadrant)) {
            // Put polygon to a more deep child rectangle

            (*this).PushLosPolygonToChild(p1Quadrant, wall);
        }
        else {
            losPolygons.push_back(wall);
        }
    }
    else {
        losPolygons.push_back(wall);
    }
}

void LosQuadTree::PushLosOrientedBoundingBox(
    const shared_ptr<LosOrientedBoundingBox>& obbPtr,
    LosQuadTree*& insertedTreePtr,
    LosObbIter& insertedIter)
{
    if (depth < MAXDEPTH) {
        const Rectangle& rect = obbPtr->GetRect();
        const QuadrantType bottomLeftQuadrant = place.GetQuadrantOf(rect.GetBottomLeft());
        const QuadrantType topRightQuadrant = place.GetQuadrantOf(rect.GetTopRight());

        if (bottomLeftQuadrant == topRightQuadrant) {
            // Put obb to a more deep child rectangle

            (*this).PushLosOrientedBoundingBoxToChild(bottomLeftQuadrant, obbPtr, insertedTreePtr, insertedIter);
        }
        else {
            insertedTreePtr = this;
            insertedIter = losObbPtrs.insert(losObbPtrs.end(), obbPtr);
        }
    }
    else {
        insertedTreePtr = this;
        insertedIter = losObbPtrs.insert(losObbPtrs.end(), obbPtr);
    }
}

void LosQuadTree::PushLosPolygonToChild(
    const QuadrantType& childQuadrant,
    const LosPolygon& wall)
{
    if (child[childQuadrant] == nullptr) {
        const Rectangle childPlace = place.MakeChildRect(childQuadrant);

        child[childQuadrant] = new LosQuadTree(childPlace, depth + 1);
    }

    child[childQuadrant]->PushLosPolygon(wall);
}

void LosQuadTree::PushLosOrientedBoundingBoxToChild(
    const QuadrantType& childQuadrant,
    const shared_ptr<LosOrientedBoundingBox>& obbPtr,
    LosQuadTree*& insertedTreePtr,
    LosObbIter& insertedIter)
{
    if (child[childQuadrant] == nullptr) {
        const Rectangle childPlace = place.MakeChildRect(childQuadrant);

        child[childQuadrant] = new LosQuadTree(childPlace, depth + 1);
    }

    child[childQuadrant]->PushLosOrientedBoundingBox(obbPtr, insertedTreePtr, insertedIter);
}

void LosQuadTree::CheckChildCollision(
    const QuadrantType& childQuadrant,
    const LosRay& ray,
    const set<NodeIdType>& ignoredNodeIds,
    const bool checkJustACollsion,
    const bool isJustHorizontalCheck,
    map<double, WallCollisionInfoType>& collisions)
{
    if (child[childQuadrant] == nullptr) {
        // no collision
        return;
    }

    child[childQuadrant]->CheckCollision(ray, ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
}

static inline
bool QuadrantXIsSame(const QuadrantType& q1, const QuadrantType& q2)
{
    if (q1 == q2) {
        return true;
    }

    return ((int(q1) + int(q2)) == 3);
}

static inline
bool QuadrantYIsSame(const QuadrantType& q1, const QuadrantType& q2)
{
    if (q1 == q2) {
        return true;
    }

    return (((int(q1) + int(q2)) == 1) ||
            ((int(q1) + int(q2)) == 5));
}

void LosQuadTree::CheckCollision(
    const LosRay& ray,
    const set<NodeIdType>& ignoredNodeIds,
    const bool checkJustACollsion,
    const bool isJustHorizontalCheck,
    map<double, WallCollisionInfoType>& collisions)
{
    if (checkJustACollsion && !collisions.empty()) {
        // already found a collision.
        return;
    }

    // check child place collision
    if (depth < MAXDEPTH) {
        const QuadrantType origQuadrant = place.GetQuadrantOf(ray.orig);
        const QuadrantType destQuadrant = place.GetQuadrantOf(ray.dest);

        if (origQuadrant == destQuadrant) {

            (*this).CheckChildCollision(origQuadrant, ray, ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);

        } else {
            const Vertex center = place.GetCenter();
            const Vertex edgeX = ray.CrossPointX(center.x);
            const Vertex edgeY = ray.CrossPointY(center.y);

            if (QuadrantXIsSame(origQuadrant, destQuadrant)) {
                // divide points by horizontal Y line
                (*this).CheckChildCollision(origQuadrant, LosRay(ray.orig, edgeY), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
                (*this).CheckChildCollision(destQuadrant, LosRay(edgeY, ray.dest), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
            }
            else if (QuadrantYIsSame(origQuadrant, destQuadrant)) {
                // divide points by vertical X line
                (*this).CheckChildCollision(origQuadrant, LosRay(ray.orig, edgeX), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
                (*this).CheckChildCollision(destQuadrant, LosRay(edgeX, ray.dest), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
            }
            else { // pass 3-place (2-place for the center point passing)

                // divide points by X and Y
                Vertex origEdge;
                Vertex destEdge;

                if (ray.orig.DistanceTo(edgeX) <= ray.orig.DistanceTo(edgeY)) {
                    origEdge = edgeX;
                    destEdge = edgeY;
                } else {
                    origEdge = edgeY;
                    destEdge = edgeX;
                }

                (*this).CheckChildCollision(origQuadrant, LosRay(ray.orig, origEdge), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
                (*this).CheckChildCollision(destQuadrant, LosRay(destEdge, ray.dest), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);

                // If edges are not on center point, check third place.
                if (origEdge != destEdge) {
                    const QuadrantType middleQuadrant = place.GetQuadrantOf((origEdge + destEdge) * 0.5);

                    if (middleQuadrant != origQuadrant &&
                        middleQuadrant != destQuadrant) {
                        (*this).CheckChildCollision(middleQuadrant, LosRay(origEdge, destEdge), ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);
                    }
                }
            }
        }
    }

    (*this).CheckLosPolygonCollision(ray, ignoredNodeIds, checkJustACollsion, isJustHorizontalCheck, collisions);

    (*this).CheckLosOrientedBoundingBoxCollision(ray, ignoredNodeIds, checkJustACollsion, collisions);
}






static inline
void InsertCollisionPoint(
    const double t,
    const VariantIdType& variantId,
    const double& shieldingLossDb,
    const LosPolygon::ObstructionType& anObstructionType,
    map<double, WallCollisionInfoType>& collisions)
{
    if (0.0 < t && t < 1.0) {
        const int roundInt = 100000000;

        const double roundedT =  floor(t * roundInt) / double(roundInt);
        const WallCollisionInfoType collisionData(variantId, shieldingLossDb, anObstructionType);

        typedef map<double, WallCollisionInfoType>::iterator IterType;

        IterType iter = collisions.find(roundedT);

        if (iter != collisions.end()) {
            const VariantIdType& dataId = (*iter).second.variantId;

            // update by low id data
            if (collisionData.variantId < dataId) {
                (*iter).second = collisionData;
            }

        } else {

            collisions.insert(make_pair(roundedT, collisionData));
        }
    }
}

void LosQuadTree::CheckLosPolygonCollision(
    const LosRay& ray,
    const set<NodeIdType>& ignoredNodeIds,
    const bool checkJustACollsion,
    const bool isJustHorizontalCheck,
    map<double, WallCollisionInfoType>& collisions)
{
    if (checkJustACollsion && !collisions.empty()) {
        // already found a collision.
        return;
    }

    typedef list<LosPolygon>::iterator IterType;

    // check own place collision
    for(IterType iter = losPolygons.begin(); iter != losPolygons.end(); iter++) {
        const LosPolygon& losPolygon = (*iter);

        // ignore this polygon
        if (ignoredNodeIds.find(losPolygon.nodeId) != ignoredNodeIds.end()) {
            continue;
        }

        if (isJustHorizontalCheck) {
            assert(ray.orig.z == 0.0);
            assert(ray.dest.z == 0.0);

            if ((losPolygon.anObstructionType == LosPolygon::Roof) &&
                losPolygon.triangle.IntersectsWithLine(ray.orig, ray.dest)) {

                const double distance = ray.dir.Distance();

                vector<Vertex> intersectionPoints;

                intersectionPoints.push_back(
                    CalculateIntersectionPositionBetweenLine(
                        ray.orig, ray.dest,
                        losPolygon.triangle.GetP1().XYPoint(),
                        losPolygon.triangle.GetP2().XYPoint()));
                        
                intersectionPoints.push_back(
                    CalculateIntersectionPositionBetweenLine(
                        ray.orig, ray.dest,
                        losPolygon.triangle.GetP2().XYPoint(),
                        losPolygon.triangle.GetP3().XYPoint()));
                        
                intersectionPoints.push_back(
                    CalculateIntersectionPositionBetweenLine(
                        ray.orig, ray.dest,
                        losPolygon.triangle.GetP3().XYPoint(),
                        losPolygon.triangle.GetP1().XYPoint()));
                        
                for(size_t i = 0; i < intersectionPoints.size(); i++) {

                    const double t = (intersectionPoints[i] - ray.orig).Distance() / distance;

                    InsertCollisionPoint(
                        t, losPolygon.variantId, losPolygon.shieldingLossDb, losPolygon.anObstructionType,
                        collisions);
                }
            }

        } else {
            double t;
            JudgeLinePlane(ray, losPolygon, t);
            
            InsertCollisionPoint(
                t, losPolygon.variantId, losPolygon.shieldingLossDb, losPolygon.anObstructionType,
                collisions);
        }
    }
}

void LosQuadTree::CheckLosOrientedBoundingBoxCollision(
    const LosRay& ray,
    const set<NodeIdType>& ignoredNodeIds,
    const bool checkJustACollsion,
    map<double, WallCollisionInfoType>& collisions)
{
    if (checkJustACollsion && !collisions.empty()) {
        // already found a collision.
        return;
    }

    typedef list<shared_ptr<LosOrientedBoundingBox> >::iterator IterType;

    // check own place collision
    for(IterType iter = losObbPtrs.begin(); iter != losObbPtrs.end(); iter++) {
        const LosOrientedBoundingBox& obb = *(*iter);

        // ignore this obb
        if (ignoredNodeIds.find(obb.nodeId) != ignoredNodeIds.end()) {
            continue;
        }

        double outTmin;
        double outTmax;

        JudgeLineOrientedBoundingBox(ray, obb, outTmin, outTmax);

        InsertCollisionPoint(
            outTmin, INVALID_GIS_OBJECT_ID, obb.shieldingLossDb, LosPolygon::Invalid,
            collisions);
        InsertCollisionPoint(
            outTmax, INVALID_GIS_OBJECT_ID, obb.shieldingLossDb, LosPolygon::Invalid,
            collisions);
    }
}


bool LosQuadTree::HasCollision(
    const LosRay& ray,
    const set<NodeIdType>& ignoredNodeIds)
{
    map<double, WallCollisionInfoType> collisions;

    (*this).CheckCollision(ray, ignoredNodeIds, true/*checkJustACollsion*/, false/*isJustHorizontalCheck*/, collisions);

    return (!collisions.empty());
}

};//namespace Los//

using Los::LosQuadTree;
using Los::LosPolygon;
using Los::LosOrientedBoundingBox;
using Los::LosMovingObject;
using Los::LosRay;

//------------------------------------------------------------

enum CrossProductDirection {
    COROSS_PRODUCT_POSITIVE,
    COROSS_PRODUCT_NEGATIVE,
};

// ciclic list
struct PointData {
    list<PointData* >::iterator iterToFarthestList;

    Vertex position;

    PointData* prev;
    PointData* next;

    PointData() : prev(nullptr), next(nullptr) {}
};

typedef list<PointData* >::iterator PointDataPtrIter;

class FarthestPoint {
public:
    FarthestPoint(const double initOffsetX, double initOffsetY)
        : offsetX(initOffsetX), offsetY(initOffsetY)
    {}

    bool operator()(const PointData* p1, const PointData* p2) const {
        return (((p1->position.x - offsetX) * (p1->position.x - offsetX) +
                 (p1->position.y - offsetY) * (p1->position.y - offsetY))
                >
                ((p2->position.x - offsetX) * (p2->position.x - offsetX) +
                 (p2->position.y - offsetY) * (p2->position.y - offsetY)));
    }
private:
    const double offsetX;
    const double offsetY;
};

// reinplementation for optimization

static inline
CrossProductDirection Get2dCrossProductDirection(
    const Vertex& p1,
    const Vertex& p2,
    const Vertex& p3)
{
    const double d21 = p1.x - p2.x;
    const double d22 = p1.y - p2.y;

    const double d31 = p1.x - p3.x;
    const double d32 = p1.y - p3.y;

    if((d21*d32 - d22*d31) >= 0) {
        return COROSS_PRODUCT_POSITIVE;
    } else {
        return COROSS_PRODUCT_NEGATIVE;
    }
}

static inline
bool IsInsideOfTheTriangle(
    const Vertex& checkPoint,
    const Vertex& p1,
    const Vertex& p2,
    const Vertex& p3)
{
    const CrossProductDirection d12 = Get2dCrossProductDirection(checkPoint, p1, p2);
    const CrossProductDirection d23 = Get2dCrossProductDirection(checkPoint, p2, p3);
    const CrossProductDirection d31 = Get2dCrossProductDirection(checkPoint, p3, p1);

    return ((d12 == d23) && (d12 == d31));
}

static inline
bool IsInsideOf3dTheTriangle(
    const Vertex& checkPoint,
    const Vertex& v1,
    const Vertex& v2,
    const Vertex& v3)
{
    const Vertex v12 = v2 - v1;
    const Vertex v2p = checkPoint - v2;

    const Vertex v23 = v3 - v2;
    const Vertex v3p = checkPoint - v3;

    const Vertex v31 = v3 - v1;
    const Vertex v1p = checkPoint - v1;

    const Vertex c1 = v12.Cross(v2p);
    const Vertex c2 = v23.Cross(v3p);
    const Vertex c3 = v31.Cross(v1p);

    const double d12 = c1.Dot(c2);
    const double d13 = c1.Dot(c3);

    if(d12 > 0 && d13 > 0) {
        return true;
    }

    return false;
}

static inline
bool IsInsideOfOtherPoint(const PointData& pointData)
{
    const PointData* prev = pointData.prev;

    for(const PointData* followingData = pointData.next->next;
        followingData != prev; followingData = followingData->next) {

        if(IsInsideOfTheTriangle(
               followingData->position,
               pointData.position,
               pointData.next->position,
               pointData.prev->position)) {
            return true;
        }
    }
    return false;
}

static inline
Triangle MakeRightHandRuleTriangle(
    const Vertex& p1,
    const Vertex& p2,
    const Vertex& p3)
{
    const CrossProductDirection direction =
        Get2dCrossProductDirection(p2, p1, p3);

    if(direction == COROSS_PRODUCT_POSITIVE) {
        return Triangle(p3, p2, p1);
    } else {
        return Triangle(p1, p2, p3);
    }
}

static inline
bool IsCompletePolygon(const vector<Vertex>& polygon)
{
    return (polygon.size() >= 4 && polygon.front() == polygon.back());
}

void PolygonToTriangles(const vector<Vertex>& polygon, vector<Triangle>& triangles)
{
    assert(IsCompletePolygon(polygon));

    vector<Vertex> minimalPolygon;
    minimalPolygon.push_back(polygon.front());
    for(size_t i = 0; i < polygon.size() - 1; i++) {
        if(minimalPolygon.back() != polygon[i]) {
            minimalPolygon.push_back(polygon[i]);
        }
    }

    const size_t numberVertices = minimalPolygon.size();
    if(numberVertices < 3) {
        return;
    }

    // prepare data sets
    PointData* pointDataChunk = new PointData[numberVertices];
    PointData* pointData;
    list<PointData* > farthests;

    const size_t lastVertexIndex = numberVertices - 1;
    for(size_t i = 0; i < lastVertexIndex; i++) {
        farthests.push_front(&pointDataChunk[i]);
        pointData = &pointDataChunk[i];
        pointData->position = minimalPolygon[i];
        pointData->iterToFarthestList = farthests.begin();
        pointData->next = &pointDataChunk[i + 1];
        pointDataChunk[i + 1].prev = pointData;
    }

    farthests.push_front(&pointDataChunk[lastVertexIndex]);
    pointData = &pointDataChunk[lastVertexIndex];
    pointData->position = minimalPolygon[lastVertexIndex];
    pointData->iterToFarthestList = farthests.begin();
    pointData->next = &pointDataChunk[0];
    pointDataChunk[0].prev = pointData;

    const PointData* firstPointData = &pointDataChunk[0];
    const PointData* lastPointData = &pointDataChunk[lastVertexIndex];

    for(const PointData* p1 = firstPointData; p1 != lastPointData; p1 = p1->next) {

        for(const PointData* p2 = p1->next; p2 != firstPointData; p2 = p2->next) {

            if(p1->position.x == p2->position.x &&
               p1->position.y == p2->position.y) {
                // illegal polygon
                delete [] pointDataChunk;
                return;
            }
        }
    }

    farthests.sort(
        FarthestPoint(
            minimalPolygon[0].x,
            minimalPolygon[0].y));


    while(farthests.size() > 3) {
        const PointData* startPointData = farthests.front();

        const CrossProductDirection crossProductDirection =
            Get2dCrossProductDirection(
                startPointData->position,
                startPointData->prev->position,
                startPointData->next->position);

        const PointData* currentPointData = startPointData;

        CrossProductDirection currentCrossProductDirection =
            crossProductDirection;

        while(true) {
            if(currentCrossProductDirection == crossProductDirection &&
               !IsInsideOfOtherPoint(*currentPointData)) {
                triangles.push_back(
                    MakeRightHandRuleTriangle(
                        currentPointData->position,
                        currentPointData->prev->position,
                        currentPointData->next->position));

                currentPointData->next->prev = currentPointData->prev;
                currentPointData->prev->next = currentPointData->next;
                farthests.erase(currentPointData->iterToFarthestList);
                break;
            }

            currentPointData = currentPointData->next;
            assert(currentPointData != nullptr);
            if(currentPointData == startPointData) {
                // no more available triangles.
                delete [] pointDataChunk;
                return;
            }

            currentCrossProductDirection =
                Get2dCrossProductDirection(
                    currentPointData->position,
                    currentPointData->prev->position,
                    currentPointData->next->position);
        }
    }
    assert(farthests.size() == 3);

    const PointData* basePoint = farthests.front();
    triangles.push_back(
        MakeRightHandRuleTriangle(
            basePoint->position,
            basePoint->next->position,
            basePoint->prev->position));

    delete [] pointDataChunk;
}

//------------------------------------------------------------

Vertex Vertex::Normalized() const
{
    const double distance = (*this).Distance();

    if (distance == 0) {
        return Vertex();
    }

    return Vertex(x/distance, y/distance, z/distance);
}

double Vertex::DistanceTo(const Vertex& vertex) const
{
    return XYZDistanceBetweenVertices(*this, vertex);
}

double Vertex::XYDistanceTo(const Vertex& vertex) const
{
    return XYDistanceBetweenVertices(*this, vertex);
}

Vertex Vertex::ToXyVertex(
    const double latitudeOriginDegrees,
    const double longitudeOriginDegrees) const
{
    return Vertex(
        ConvertLongitudeDegreesToXMeters(latitudeOriginDegrees, longitudeOriginDegrees, x),
        ConvertLatitudeDegreesToYMeters(latitudeOriginDegrees, y),
        z);
}

Vertex Vertex::Inverted() const
{
    double invX = 0.;
    if (x != 0.) {
        invX = 1. /x;
    }

    double invY = 0.;
    if (y != 0.) {
        invY = 1. /y;
    }

    double invZ = 0.;
    if (z != 0.) {
        invZ = 1. /z;
    }

    return Vertex(invX, invY, invZ);
}

void Vertex::operator+=(const Vertex& right)
{
    x += right.x;
    y += right.y;
    z += right.z;
}

Vertex Vertex::operator+(const Vertex& right) const
{
    return Vertex(x + right.x, y + right.y, z + right.z);
}

Vertex Vertex::operator-(const Vertex& right) const
{
    return Vertex(x - right.x, y - right.y, z - right.z);
}

Vertex Vertex::operator/(const double scale) const
{
    return Vertex(x/scale, y/scale, z/scale);
}

Vertex Vertex::operator*(const double scale) const
{
    return Vertex(x*scale, y*scale, z*scale);
}

VariantIdType GisVertex::GetConnectedObjectId(const GisObjectType& objectType) const
{
    typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

    IterType iter = connections.find(objectType);

    assert(iter != connections.end());

    return (*iter).second.front().variantId;
}

vector<VariantIdType> GisVertex::GetConnectedObjectIds(const GisObjectType& objectType) const
{
    typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

    vector<VariantIdType> variantIds;

    IterType iter = connections.find(objectType);

    if (iter != connections.end()) {

        const vector<VertexConnection>& vertexConnections = (*iter).second;

        for(size_t i = 0; i < vertexConnections.size(); i++) {
            variantIds.push_back(vertexConnections[i].variantId);
        }
    }

    return variantIds;
}

bool GisVertex::HasConnection(const GisObjectType& objectType) const
{
    typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

    IterType iter = connections.find(objectType);

    if (iter != connections.end()) {
        return !(*iter).second.empty();
    }

    return false;
}

void Rectangle::operator+=(const Rectangle& right)
{
    minX = std::min(minX, right.minX);
    minY = std::min(minY, right.minY);
    maxX = std::max(maxX, right.maxX);
    maxY = std::max(maxY, right.maxY);
}

QuadrantType Rectangle::GetQuadrantOf(const Vertex& v) const
{
    const Vertex center = (*this).GetCenter();

    if (v.x >= center.x) {
        if (v.y >= center.y) {
            return QUADRANT_1_PX_PY;
        } else {
            return QUADRANT_4_PX_NY;
        }
    } else {
        if (v.y >= center.y) {
            return QUADRANT_2_NX_PY;
        } else {
            return QUADRANT_3_NX_NY;
        }
    }
}

Rectangle Rectangle::MakeChildRect(const QuadrantType& quadrantType) const
{
    const Vertex center = (*this).GetCenter();
    const double halfWidth = (*this).GetWidth()*0.5;
    const double halfHeight = (*this).GetHeight()*0.5;

    switch (quadrantType) {
    case QUADRANT_1_PX_PY: return Rectangle(center, center + Vertex(halfWidth, halfHeight));
    case QUADRANT_2_NX_PY: return Rectangle(center, center + Vertex(-halfWidth, halfHeight));
    case QUADRANT_3_NX_NY: return Rectangle(center, center + Vertex(-halfWidth, -halfHeight));
    case QUADRANT_4_PX_NY: return Rectangle(center, center + Vertex(halfWidth, -halfHeight));
        break;
    }

    return Rectangle();
}

bool Rectangle::Contains(const Vertex& point) const
{
    if (minX >= maxX || minY >= maxY) {
        return false;
    }

    return ((minX <= point.x) && (point.x <= maxX) &&
            (minY <= point.y) && (point.y <= maxY));
}

bool Rectangle::Contains(const Rectangle& rectangle) const
{
    return ((minX <= rectangle.minX && rectangle.maxX <= maxX) &&
            (minY <= rectangle.minY && rectangle.maxY <= maxY));
}

Rectangle Rectangle::Expanded(const double margin) const
{
    return Rectangle(minX - margin, minY - margin, maxX + margin, maxY + margin);
}

bool Rectangle::OverlappedWith(const Rectangle& rectangle) const
{
    return (!((rectangle.maxX < minX) || (maxX < rectangle.minX) ||
              (rectangle.maxY < minY) || (maxY < rectangle.minY)));
}

Rectangle Rectangle::GetOverlappedRectangle(const Rectangle& rectangle) const
{
    assert((*this).OverlappedWith(rectangle));

    return Rectangle(
        std::max(minX, rectangle.minX),
        std::max(minY, rectangle.minY),
        std::min(maxX, rectangle.maxX),
        std::min(maxY, rectangle.maxY));
}

bool Rectangle::IntersectsWithLine(
    const Vertex& lineEdge1,
    const Vertex& lineEdge2) const
{
    if ((*this).Contains(lineEdge1) ||
        (*this).Contains(lineEdge2)) {
        return true;
    }

    const Vertex topLeft(minX, maxY);
    const Vertex bottomLeft(minX, minY);
    const Vertex topRight(maxX,maxY);
    const Vertex bottomRight(maxX, minY);

    if (HorizontalLinesAreIntersection(lineEdge1, lineEdge2, topLeft, bottomLeft) ||
        HorizontalLinesAreIntersection(lineEdge1, lineEdge2, bottomLeft, bottomRight) ||
        HorizontalLinesAreIntersection(lineEdge1, lineEdge2, bottomRight, topRight) ||
        HorizontalLinesAreIntersection(lineEdge1, lineEdge2, topRight, topLeft)) {
        return true;
    }

    return false;
}

NlosPathData::NlosPathData(
    const RoadIdType& initRoadId1,
    const RoadIdType& initRoadId2,
    const IntersectionIdType& initIntersectionId1,
    const IntersectionIdType& initIntersectionId2,
    const IntersectionIdType& initNlosIntersectionId)
    :
    numberEndToEndRoadIds(0),
    numberEndToEndIntersectionIds(0)
{
    (*this).PushBackRoadId(initRoadId1);
    (*this).PushBackRoadId(initRoadId2);

    (*this).PushBackIntersectionId(initIntersectionId1);
    (*this).PushBackIntersectionId(initNlosIntersectionId);
    (*this).PushBackIntersectionId(initIntersectionId2);

    assert((*this).GetNumberOfIntersections() == (*this).GetNumberOfRoads() + 1);
}

NlosPathData::NlosPathData(
    const RoadIdType& initRoadId1,
    const RoadIdType& initRoadId2,
    const IntersectionIdType& initIntersectionId1,
    const IntersectionIdType& initIntersectionId2,
    const IntersectionIdType& initNlosIntersectionId1,
    const IntersectionIdType& initNlosIntersectionId2,
    const RoadIdType& initNlosRoadId)
    :
    numberEndToEndRoadIds(0),
    numberEndToEndIntersectionIds(0)
{
    (*this).PushBackRoadId(initRoadId1);
    (*this).PushBackRoadId(initNlosRoadId);
    (*this).PushBackRoadId(initRoadId2);

    (*this).PushBackIntersectionId(initIntersectionId1);
    (*this).PushBackIntersectionId(initNlosIntersectionId1);
    (*this).PushBackIntersectionId(initNlosIntersectionId2);
    (*this).PushBackIntersectionId(initIntersectionId2);

    assert((*this).GetNumberOfIntersections() == (*this).GetNumberOfRoads() + 1);
}

NlosPathData NlosPathData::Clone() const
{
    NlosPathData cloneData;

    cloneData.numberEndToEndRoadIds = numberEndToEndRoadIds;
    cloneData.numberEndToEndIntersectionIds = numberEndToEndIntersectionIds;

    cloneData.endToEndRoadIds = shared_array<RoadIdType>(new RoadIdType[numberEndToEndIntersectionIds]);
    for(int i = 0; i < numberEndToEndIntersectionIds; i++) {
        cloneData.endToEndRoadIds[i] = endToEndRoadIds[i];
    }

    cloneData.endToEndIntersectionIds = shared_array<IntersectionIdType>(new IntersectionIdType[numberEndToEndIntersectionIds]);
    for(int i = 0; i < numberEndToEndIntersectionIds; i++) {
        cloneData.endToEndIntersectionIds[i] = endToEndIntersectionIds[i];
    }

    return cloneData;
}

void NlosPathData::PushBackRoadId(const RoadIdType& roadId)
{
    shared_array<RoadIdType> newRoadIds = shared_array<RoadIdType>(new RoadIdType[numberEndToEndRoadIds + 1]);

    for(int i = 0; i < numberEndToEndRoadIds; i++) {
        newRoadIds[i] = endToEndRoadIds[i];
    }
    newRoadIds[numberEndToEndRoadIds] = roadId;

    numberEndToEndRoadIds++;

    endToEndRoadIds = newRoadIds;
}
void NlosPathData::PushFrontRoadId(const RoadIdType& roadId)
{
    shared_array<RoadIdType> newRoadIds = shared_array<RoadIdType>(new RoadIdType[numberEndToEndRoadIds + 1]);

    newRoadIds[0] = roadId;

    for(int i = 0; i < numberEndToEndRoadIds; i++) {
        newRoadIds[i+1] = endToEndRoadIds[i];
    }

    numberEndToEndRoadIds++;

    endToEndRoadIds = newRoadIds;
}
RoadIdType NlosPathData::GetFrontRoadId() const
{
    assert(numberEndToEndRoadIds > 0);
    return endToEndRoadIds[0];
}
RoadIdType NlosPathData::GetBackRoadId() const
{
    assert(numberEndToEndRoadIds > 0);
    return endToEndRoadIds[numberEndToEndRoadIds-1];
}
RoadIdType NlosPathData::GetNlosRoadId() const
{
    assert(numberEndToEndRoadIds > 1);
    return endToEndRoadIds[1];
}
RoadIdType NlosPathData::GetLastNlosRoadId() const
{
    assert(numberEndToEndRoadIds > 1);
    return endToEndRoadIds[numberEndToEndRoadIds-2];
}
bool NlosPathData::RoadIdIsEmpty() const
{
    return (numberEndToEndRoadIds == 0);
}
size_t NlosPathData::GetNumberOfRoads() const
{
    return numberEndToEndRoadIds;
}

bool NlosPathData::ContainsRoad(const RoadIdType& roadId) const
{
    for(int i = 0; i < numberEndToEndRoadIds; i++) {
        if (endToEndRoadIds[i] == roadId) {
            return true;
        }
    }

    return false;
}

void NlosPathData::PushBackIntersectionId(const IntersectionIdType& intersectionId)
{
    shared_array<IntersectionIdType> newIntersectionIds = shared_array<IntersectionIdType>(new IntersectionIdType[numberEndToEndIntersectionIds + 1]);

    for(int i = 0; i < numberEndToEndIntersectionIds; i++) {
        newIntersectionIds[i] = endToEndIntersectionIds[i];
    }
    newIntersectionIds[numberEndToEndIntersectionIds] = intersectionId;

    numberEndToEndIntersectionIds++;

    endToEndIntersectionIds = newIntersectionIds;
}
void NlosPathData::PushFrontIntersectionId(const IntersectionIdType& intersectionId)
{
    shared_array<IntersectionIdType> newIntersectionIds = shared_array<IntersectionIdType>(new IntersectionIdType[numberEndToEndIntersectionIds + 1]);

    newIntersectionIds[0] = intersectionId;

    for(int i = 0; i < numberEndToEndIntersectionIds; i++) {
        newIntersectionIds[i+1] = endToEndIntersectionIds[i];
    }

    numberEndToEndIntersectionIds++;

    endToEndIntersectionIds = newIntersectionIds;
}
IntersectionIdType NlosPathData::GetFrontIntersectionId() const
{
    assert(numberEndToEndIntersectionIds > 0);
    return endToEndIntersectionIds[0];
}
IntersectionIdType NlosPathData::GetBackIntersectionId() const
{
    assert(numberEndToEndIntersectionIds > 0);
    return endToEndIntersectionIds[numberEndToEndIntersectionIds-1];
}

size_t NlosPathData::GetNumberOfIntersections() const
{
    return numberEndToEndIntersectionIds;
}

std::pair<RoadIdType, RoadIdType> NlosPathData::GetRoadRelation() const
{
    assert(!(*this).RoadIdIsEmpty());
    return make_pair((*this).GetFrontRoadId(), (*this).GetBackRoadId());
}

void NlosPathData::Normalize()
{
    assert((*this).GetNumberOfRoads() > 1);
    assert((*this).GetFrontRoadId() != (*this).GetBackRoadId());
    assert((*this).GetNumberOfIntersections() == (*this).GetNumberOfRoads() + 1);

    if ((*this).GetFrontRoadId() > (*this).GetBackRoadId()) {
        (*this).Inverse();
    }
}

void NlosPathData::Inverse()
{
    for(int i = 0; i < numberEndToEndRoadIds / 2; i++) {
        std::swap(endToEndRoadIds[i], endToEndRoadIds[numberEndToEndRoadIds - i - 1]);
    }
    for(int i = 0; i < numberEndToEndIntersectionIds / 2; i++) {
        std::swap(endToEndIntersectionIds[i], endToEndIntersectionIds[numberEndToEndIntersectionIds - i - 1]);
    }
}

bool NlosPathData::IsMultipleNlosPath() const
{
    return ((*this).GetNlosCount() > 0);
}

size_t NlosPathData::GetNlosCount() const
{
    assert(!(*this).RoadIdIsEmpty());
    assert((*this).GetNumberOfIntersections() == (*this).GetNumberOfRoads() + 1);

    return (*this).GetNumberOfRoads() - 1;
}

IntersectionIdType NlosPathData::GetIntersectionId(
    const RoadIdType& startRoadId,
    const size_t intersectionNumber) const
{
    if ((*this).GetFrontRoadId() == startRoadId) {

        return endToEndIntersectionIds[intersectionNumber];

    } else {
        assert((*this).GetBackRoadId() == startRoadId);

        return endToEndIntersectionIds[(*this).GetNumberOfIntersections() - intersectionNumber - 1];
    }
}

IntersectionIdType NlosPathData::GetStartIntersectionId(
    const RoadIdType& startRoadId) const
{
    if ((*this).GetFrontRoadId() == startRoadId) {
        return (*this).GetFrontIntersectionId();
    } else {
        assert((*this).GetBackRoadId() == startRoadId);

        return (*this).GetBackIntersectionId();
    }
}

IntersectionIdType NlosPathData::GetEndIntersectionId(
    const RoadIdType& startRoadId) const
{
    if ((*this).GetFrontRoadId() == startRoadId) {
        return (*this).GetBackIntersectionId();
    } else {
        assert((*this).GetBackRoadId() == startRoadId);

        return (*this).GetFrontIntersectionId();
    }
}

RoadIdType NlosPathData::GetRoadId(
    const RoadIdType& startRoadId,
    const size_t roadNumber) const
{
    if ((*this).GetFrontRoadId() == startRoadId) {

        return endToEndRoadIds[roadNumber];

    } else {
        assert((*this).GetBackRoadId() == startRoadId);

        return endToEndRoadIds[(*this).GetNumberOfRoads() - roadNumber - 1];
    }
}

RoadIdType NlosPathData::GetEndRoadId(const RoadIdType& startRoadId) const
{
    if ((*this).GetFrontRoadId() == startRoadId) {
        return (*this).GetBackRoadId();
    } else {
        assert((*this).GetBackRoadId() == startRoadId);

        return (*this).GetFrontRoadId();
    }
}

RoadLosChecker::RoadLosChecker(
    const shared_ptr<const RoadLayer>& initRoadLayerPtr,
    const shared_ptr<NlosPathValueCalculator>& initNlosValueCalculatorPtr,
    const size_t initMaxDiffractionCount,
    const double initLosThresholdRadians,
    const double initMaxNlosDistance)
    :
    roadLayerPtr(initRoadLayerPtr),
    nlosValueCalculatorPtr(initNlosValueCalculatorPtr),
    maxDiffractionCount(initMaxDiffractionCount),
    losThresholdRadians(initLosThresholdRadians),
    maxNlosDistance(initMaxNlosDistance)
{
}

void RoadLosChecker::MakeLosRelation()
{
    map<IntersectionIdType, set<RoadIdType> > losRoadIdsPerIntersection;

    (*this).MakeLosRelation(losRoadIdsPerIntersection);

    (*this).MakeNlos1Relation(losRoadIdsPerIntersection);

    if (maxDiffractionCount > 1) {
        (*this).MakeNlos2ToNRelation(losRoadIdsPerIntersection);
    }

    //(*this).OutputLosRelation();
}

double RoadLosChecker::CalculateNlosPointRadians(
    const NlosPathData& nlosPath,
    const GisObjectIdType& startRoadId) const
{
    const vector<Intersection>& intersections = roadLayerPtr->intersections;

    const IntersectionIdType startIntersectionId =
        nlosPath.GetIntersectionId(startRoadId, 0);

    const Road& startRoad = roadLayerPtr->GetRoad(startRoadId);
    const Road& nlosRoad = roadLayerPtr->GetRoad(nlosPath.GetRoadId(startRoadId, 1));

    const IntersectionIdType startRoadIntersectionId =
        startRoad.GetOtherSideIntersectionId(startIntersectionId);

    const IntersectionIdType nlosIntersectionId =
        nlosPath.GetIntersectionId(startRoadId, 1);

    const IntersectionIdType endIntersectionId =
        nlosRoad.GetOtherSideIntersectionId(nlosIntersectionId);

    const Vertex& startPosition = intersections.at(startRoadIntersectionId).GetVertex();
    const Vertex& nlos1Position = intersections.at(nlosIntersectionId).GetVertex();
    const Vertex& endPosition = intersections.at(endIntersectionId).GetVertex();

    const Vertex startVector = Vertex(
        startPosition.positionX - nlos1Position.positionX,
        startPosition.positionY - nlos1Position.positionY,
        startPosition.positionY - nlos1Position.positionY).Normalized();

    const Vertex endVector = Vertex(
        endPosition.positionX - nlos1Position.positionX,
        endPosition.positionY - nlos1Position.positionY,
        endPosition.positionY - nlos1Position.positionY).Normalized();

    const double cosAlpha =
        (startVector.positionX*endVector.positionX +
         startVector.positionY*endVector.positionY) /
        (startVector.XYDistance()*endVector.XYDistance());

    const double rad = (std::acos(cosAlpha));

    return rad;
}

vector<Vertex> RoadLosChecker::CalculateNlosPoints(
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId) const
{
    const vector<Intersection>& intersections = roadLayerPtr->intersections;

    vector<Vertex> nlosPoints;

    for(int i = 0; i < nlosPath.numberEndToEndIntersectionIds; i++) {
        const IntersectionIdType nlos1IntersectionId = nlosPath.GetIntersectionId(startRoadId, i);

        nlosPoints.push_back(intersections.at(nlos1IntersectionId).GetVertex());
    }

    return nlosPoints;
}

double NlosPathData::GetNlosPathDistance(const vector<Intersection>& intersections) const
{
    double distance = 0;

    for(int i = 0; i < numberEndToEndIntersectionIds - 1; i++) {
        distance += XYDistanceBetweenVertices(
            intersections.at(endToEndIntersectionIds[i]).GetVertex(),
            intersections.at(endToEndIntersectionIds[i+1]).GetVertex());
    }

    return distance;
}

void NlosPathData::ExpandRoad(
    const RoadIdType& startRoadId,
    const IntersectionIdType& baseIntersectionId,
    const RoadIdType& expandRoadId,
    const IntersectionIdType& endIntersectionId)
{
    assert(numberEndToEndIntersectionIds > 0);

    if (startRoadId == (*this).GetFrontRoadId()) {
        endToEndIntersectionIds[numberEndToEndIntersectionIds - 1] = baseIntersectionId;

        (*this).PushBackRoadId(expandRoadId);
        (*this).PushBackIntersectionId(endIntersectionId);
    } else {
        endToEndIntersectionIds[0] = baseIntersectionId;

        (*this).PushFrontRoadId(expandRoadId);
        (*this).PushFrontIntersectionId(endIntersectionId);
    }
}

double RoadLosChecker::CalculateNlosPointToStartPointCenterDistance(
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId,
    const Vertex& startPosition) const
{
    const vector<Intersection>& intersections = roadLayerPtr->intersections;

    const IntersectionIdType startRoadIntersectionId = nlosPath.GetStartIntersectionId(startRoadId);
    const IntersectionIdType nlosIntersectionId = nlosPath.GetIntersectionId(startRoadId, 1);
    const Road& road = roadLayerPtr->GetRoad(startRoadId);
    const IntersectionIdType othersideRoadIntersectionId =
        road.GetOtherSideIntersectionId(startRoadIntersectionId);

    const Vertex& v0 = intersections.at(othersideRoadIntersectionId).GetVertex();
    const Vertex& v1 = intersections.at(startRoadIntersectionId).GetVertex();
    const Vertex& v2 = intersections.at(nlosIntersectionId).GetVertex();

    const Vertex startPositionOnRoadCenter = CalculateIntersectionPosition(startPosition, v0, v1);

    return startPositionOnRoadCenter.XYDistanceTo(v1) + v1.XYDistanceTo(v2);
}

double RoadLosChecker::CalculateDistanceToLastNlosPoint(
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId,
    const Vertex& startPosition) const
{
    const vector<Intersection>& intersections = roadLayerPtr->intersections;

    double totalDistance =
        (*this).CalculateNlosPointToStartPointCenterDistance(nlosPath, startRoadId, startPosition);

    for(int i = 1; i < nlosPath.numberEndToEndIntersectionIds - 2; i++) {
        const IntersectionIdType intersectionId1 = nlosPath.GetIntersectionId(startRoadId, i);
        const IntersectionIdType intersectionId2 = nlosPath.GetIntersectionId(startRoadId, i+1);

        totalDistance += XYDistanceBetweenVertices(
            intersections.at(intersectionId1).GetVertex(),
            intersections.at(intersectionId2).GetVertex());
    }

    return totalDistance;
}

double RoadLosChecker::CalculateNlosPointToEndPointCenterDistance(
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId,
    const Vertex& endPosition) const
{
    const RoadIdType endRoadId = nlosPath.GetEndRoadId(startRoadId);

    return (*this).CalculateDistanceToLastNlosPoint(nlosPath, endRoadId, endPosition);
}

double RoadLosChecker::CalculateStartPointToEndPointCenterDistance(
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId,
    const Vertex& startPosition,
    const Vertex& endPosition) const
{
    return ((*this).CalculateNlosPointToStartPointCenterDistance(nlosPath, startRoadId, startPosition) +
            (*this).CalculateNlosPointToEndPointCenterDistance(nlosPath, startRoadId, endPosition));
}

bool RoadLosChecker::IsCompleteNlosPath(
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId,
    const Vertex& startPosition,
    const Vertex& endPosition) const
{
    const vector<Road>& roads = roadLayerPtr->roads;
    const RoadIdType endRoadId = nlosPath.GetEndRoadId(startRoadId);

    const double w12 =
        roads.at(nlosPath.GetRoadId(startRoadId, 1)).GetRoadWidthMeters();

    const double x1 = (*this).CalculateNlosPointToStartPointCenterDistance(
        nlosPath, startRoadId, startPosition);

    if (x1 < w12/2.) {
        return false;
    }

    const double w22 =
        roads.at(nlosPath.GetRoadId(endRoadId, 1)).GetRoadWidthMeters();

    const double x2 = (*this).CalculateNlosPointToStartPointCenterDistance(
        nlosPath, endRoadId, endPosition);

    if (x2 < w22/2.) {
        return false;
    }

    return true;
}

static inline
double NormalizedAbsRadians(const double radians)
{
    const double absRadians = (std::abs(radians));

    return std::min(std::abs(absRadians), std::abs(absRadians - PI));
}

void RoadLosChecker::MakeLosRelation(
    map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection)
{
    const vector<Road>& roads = roadLayerPtr->roads;
    const vector<Intersection>& intersections = roadLayerPtr->intersections;

    losRoadIdsPerIntersection.clear();

    for(RoadIdType roadId = 0; roadId < RoadIdType(roads.size()); roadId++) {
        const Road& aRoad = roads[roadId];

        if (aRoad.NumberOfVertices() != 2) {
            continue;
            //cerr << "Number of vertices of road must be 2 to load LOS relation." << endl;
            //exit(1);
        }

        set<RoadIdType>& losRoadIdsForStart = losRoadIdsPerIntersection[aRoad.GetStartIntersectionId()];
        set<RoadIdType>& losRoadIdsForEnd = losRoadIdsPerIntersection[aRoad.GetEndIntersectionId()];

        losRoadIdsForStart.insert(roadId);
        losRoadIdsForEnd.insert(roadId);

        const double baseRadians = aRoad.GetDirectionRadians();

        set<std::pair<RoadIdType, RoadIdType> > checkedRoadPair;
        std::queue<IntersectionIdType> losIntersectionIds;

        losIntersectionIds.push(aRoad.GetStartIntersectionId());
        losIntersectionIds.push(aRoad.GetEndIntersectionId());

        while (!losIntersectionIds.empty()) {

            const IntersectionIdType& losIntersectionId = losIntersectionIds.front();

            const Intersection& losIntersection = intersections[losIntersectionId];
            const vector<RoadIdType> connectedRoadIds = losIntersection.GetConnectedRoadIds();

            for(size_t i = 0; i < connectedRoadIds.size(); i++) {
                const RoadIdType& connectedRoadId = connectedRoadIds[i];
                const Road& connectedRoad = roads[connectedRoadId];

                const std::pair<RoadIdType, RoadIdType> roadPair =
                    make_pair(roadId, connectedRoadId);

                if (checkedRoadPair.find(roadPair) == checkedRoadPair.end()) {
                    checkedRoadPair.insert(roadPair);

                    const double directionDifferenceRadians =
                        NormalizedAbsRadians(baseRadians - connectedRoad.GetDirectionRadians());

                    if (directionDifferenceRadians < losThresholdRadians) {
                        losRelationBetweenRoads[roadPair].relationType = ROAD_LOSRELATION_LOS;

                        IntersectionIdType otherIntersectionId;
                        if (connectedRoad.GetStartIntersectionId() == losIntersectionId) {
                            otherIntersectionId = connectedRoad.GetEndIntersectionId();
                        } else {
                            assert(connectedRoad.GetStartIntersectionId() != losIntersectionId);
                            otherIntersectionId = connectedRoad.GetStartIntersectionId();
                        }

                        losRoadIdsForStart.insert(connectedRoadId);
                        losRoadIdsForEnd.insert(connectedRoadId);
                        losRoadIdsPerIntersection[otherIntersectionId].insert(connectedRoadId);
                        losIntersectionIds.push(otherIntersectionId);
                    }
                }
            }

            losIntersectionIds.pop();
        }
    }
}

void RoadLosChecker::MakeNlos1Relation(
    const map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection)
{
    const vector<Road>& roads = roadLayerPtr->roads;
    const vector<Intersection>& intersections = roadLayerPtr->intersections;

    typedef map<IntersectionIdType, set<RoadIdType> >::const_iterator IntersectionIter;

    // Make first depth
    for(IntersectionIter intersectionIter = losRoadIdsPerIntersection.begin();
        intersectionIter != losRoadIdsPerIntersection.end(); intersectionIter++) {

        const IntersectionIdType& intersectionId = (*intersectionIter).first;
        const set<RoadIdType>& losRoadIds = (*intersectionIter).second;
        const Vertex& intersectionPos = intersections[intersectionId].GetVertex();

        typedef set<RoadIdType>::const_iterator LosRoadIter;

        for(LosRoadIter roadIter1 = losRoadIds.begin(); roadIter1 != losRoadIds.end(); roadIter1++) {

            for(LosRoadIter roadIter2 = losRoadIds.begin(); roadIter2 != losRoadIds.end(); roadIter2++) {
                const RoadIdType& roadId1 = *roadIter1;
                const RoadIdType& roadId2 = *roadIter2;

                const std::pair<RoadIdType, RoadIdType> roadPair =
                    make_pair(roadId1, roadId2);

                typedef map<std::pair<IntersectionIdType, RoadIdType>, RoadLosRelationData>::iterator LosRelationIter;

                LosRelationIter losRelationIter = losRelationBetweenRoads.find(roadPair);

                if (losRelationIter != losRelationBetweenRoads.end() &&
                    (*losRelationIter).second.relationType == ROAD_LOSRELATION_LOS) {
                    continue;
                }

                const Road& road1 = roads[roadId1];
                const Road& road2 = roads[roadId2];

                if (road1.IsParking() || road2.IsParking()) {
                    continue;
                }

                const IntersectionIdType startIntersection1Id = road1.GetStartIntersectionId();
                const IntersectionIdType endIntersection1Id = road1.GetEndIntersectionId();
                const IntersectionIdType startIntersection2Id = road2.GetStartIntersectionId();
                const IntersectionIdType endIntersection2Id = road2.GetEndIntersectionId();

                if (startIntersection1Id == startIntersection2Id &&
                    endIntersection1Id == endIntersection2Id) {
                    RoadLosRelationData& losRelation = losRelationBetweenRoads[roadPair];
                    losRelation.relationType = ROAD_LOSRELATION_LOS;

                    continue;
                }

                const double distanceToRoad1Start =
                    SquaredXYDistanceBetweenVertices(
                        intersectionPos,
                        intersections[startIntersection1Id].GetVertex());

                const double distanceToRoad1End =
                    SquaredXYDistanceBetweenVertices(
                        intersectionPos,
                        intersections[endIntersection1Id].GetVertex());

                const double distanceToRoad2Start =
                    SquaredXYDistanceBetweenVertices(
                        intersectionPos,
                        intersections[startIntersection2Id].GetVertex());

                const double distanceToRoad2End =
                    SquaredXYDistanceBetweenVertices(
                        intersectionPos,
                        intersections[endIntersection2Id].GetVertex());

                IntersectionIdType startIntersectionId;
                if (distanceToRoad1Start < distanceToRoad1End) {
                    startIntersectionId = road1.GetStartIntersectionId();
                } else {
                    startIntersectionId = road1.GetEndIntersectionId();
                }

                assert(!(startIntersection1Id == startIntersection2Id &&
                         endIntersection1Id == endIntersection2Id));

                IntersectionIdType endIntersectionId;
                if (distanceToRoad2Start < distanceToRoad2End) {
                    endIntersectionId = road2.GetStartIntersectionId();
                } else {
                    endIntersectionId = road2.GetEndIntersectionId();
                }

                NlosPathData nlosPath(
                    roadId1,
                    roadId2,
                    startIntersectionId,
                    endIntersectionId,
                    intersectionId);

                if (nlosPath.GetNlosPathDistance(roadLayerPtr->intersections) <=
                    maxNlosDistance) {

                    RoadLosRelationData& losRelation = losRelationBetweenRoads[roadPair];
                    losRelation.relationType = ROAD_LOSRELATION_NLOS;

                    nlosPath.pathValue = nlosValueCalculatorPtr->GetNlosPathValue(nlosPath);

                    const std::pair<IntersectionIdType, IntersectionIdType> intersectionPair = nlosPath.GetIntersectionPair();

                    if (losRelation.nlosPaths.find(intersectionPair) == losRelation.nlosPaths.end() ||
                        losRelation.nlosPaths[intersectionPair].pathValue > nlosPath.pathValue) {

                        losRelation.nlosPaths[intersectionPair] = nlosPath;

                        assert(losRelation.nlosPaths.size() <= 4);
                    }
                }
            }
        }
    }
}

void RoadLosChecker::MakeNlos2ToNRelation(
    const map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection)
{
    const vector<Road>& roads = roadLayerPtr->roads;

    typedef map<std::pair<IntersectionIdType, RoadIdType>, RoadLosRelationData>::iterator LosRelationIter;
    typedef map<IntersectionIdType, set<RoadIdType> >::const_iterator IntersectionIter;

    map<IntersectionIdType, set<IntersectionIdType> > losIntersectionIdsPerIntersection;

    for(IntersectionIter intersectionIter = losRoadIdsPerIntersection.begin();
        intersectionIter != losRoadIdsPerIntersection.end(); intersectionIter++) {

        const IntersectionIdType& intersectionId = (*intersectionIter).first;
        const set<RoadIdType>& roadIds = (*intersectionIter).second;

        set<IntersectionIdType>& intersectionIds = losIntersectionIdsPerIntersection[intersectionId];

        typedef set<RoadIdType>::const_iterator RoadIter;

        for(RoadIter roadIter = roadIds.begin();
            roadIter != roadIds.end(); roadIter++) {

            const Road& road = roads[*roadIter];

            intersectionIds.insert(road.GetStartIntersectionId());
            intersectionIds.insert(road.GetEndIntersectionId());
        }
    }

    for(size_t i = 1; i < maxDiffractionCount; i++) {

        map<std::pair<RoadIdType, RoadIdType>, RoadLosRelationData> newLosRelationBetweenRoads = losRelationBetweenRoads;

        for(LosRelationIter losRelationIter = losRelationBetweenRoads.begin();
            losRelationIter != losRelationBetweenRoads.end(); losRelationIter++) {

            const RoadLosRelationData& relation = (*losRelationIter).second;
            const map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>& nlosPaths = relation.nlosPaths;

            typedef map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>::const_iterator NlosIter;

            vector<NlosPathData> extendedNlosPaths;

            for(NlosIter nlosIter = nlosPaths.begin(); nlosIter != nlosPaths.end(); nlosIter++) {
                const NlosPathData& nlosPath = (*nlosIter).second;

                if (nlosPath.GetNlosCount() == i) {

                    (*this).PushExtendedNlosPath(
                        losRoadIdsPerIntersection,
                        losIntersectionIdsPerIntersection,
                        nlosPath,
                        nlosPath.GetFrontRoadId(),
                        extendedNlosPaths);
                }
            }

            for(size_t j = 0; j < extendedNlosPaths.size(); j++) {
                const NlosPathData& nlosPath = extendedNlosPaths[j];
                const std::pair<RoadIdType, RoadIdType> roadIdPair = nlosPath.GetRoadRelation();

                RoadLosRelationData& losRelation = newLosRelationBetweenRoads[roadIdPair];

                if (losRelation.relationType != ROAD_LOSRELATION_LOS) {
                    losRelation.relationType = ROAD_LOSRELATION_NLOS;

                    const std::pair<IntersectionIdType, IntersectionIdType> intersectionPair =
                        nlosPath.GetIntersectionPair();

                    if (losRelation.nlosPaths.find(intersectionPair) == losRelation.nlosPaths.end() ||
                        losRelation.nlosPaths[intersectionPair].pathValue > nlosPath.pathValue) {

                        losRelation.nlosPaths[intersectionPair] = nlosPath;

                        assert(losRelation.nlosPaths.size() <= 4);
                    }
                }
            }
        }

        losRelationBetweenRoads = newLosRelationBetweenRoads;
    }
}

void RoadLosChecker::PushExtendedNlosPath(
    const map<IntersectionIdType, set<RoadIdType> >& losRoadIdsPerIntersection,
    const map<IntersectionIdType, set<RoadIdType> >& losIntersectionIdsPerIntersection,
    const NlosPathData& nlosPath,
    const RoadIdType& startRoadId,
    vector<NlosPathData>& nlosPaths) const
{
    const vector<Intersection>& intersections = roadLayerPtr->intersections;
    const RoadIdType endRoadId = nlosPath.GetEndRoadId(startRoadId);
    const vector<Road>& roads = roadLayerPtr->roads;

    assert(nlosPath.GetNumberOfIntersections() > 2);

    typedef map<std::pair<RoadIdType, RoadIdType>, RoadLosRelationData>::const_iterator LosRelationIter;
    typedef map<IntersectionIdType, set<RoadIdType> >::const_iterator IntersectionIter;
    typedef set<RoadIdType>::const_iterator NlosRoadIter;

    const Road& endRoad = roadLayerPtr->GetRoad(endRoadId);

    const IntersectionIdType lastEndIntersectionId =
        nlosPath.GetEndIntersectionId(startRoadId);

    const IntersectionIdType endIntersectionId =
        endRoad.GetOtherSideIntersectionId(lastEndIntersectionId);

    IntersectionIter nlosIntersectionIter = losRoadIdsPerIntersection.find(lastEndIntersectionId);
    IntersectionIter endIntersectionIter = losRoadIdsPerIntersection.find(endIntersectionId);
    IntersectionIter endLosIntersectionIter = losIntersectionIdsPerIntersection.find(endIntersectionId);

    const Vertex& endIntersectionPos = intersections[endIntersectionId].GetVertex();

    assert(nlosIntersectionIter != losRoadIdsPerIntersection.end());
    assert(endIntersectionIter != losRoadIdsPerIntersection.end());
    assert(endLosIntersectionIter != losIntersectionIdsPerIntersection.end());

    const set<RoadIdType>& endRoadIds = (*endIntersectionIter).second;
    const set<RoadIdType>& ignoreRoadIds = (*nlosIntersectionIter).second;

    for(NlosRoadIter nlosRoadIter = endRoadIds.begin();
        nlosRoadIter != endRoadIds.end(); nlosRoadIter++) {

        const RoadIdType& endRoadId = (*nlosRoadIter);

        LosRelationIter losRelationIter =
            losRelationBetweenRoads.find(make_pair(startRoadId, endRoadId));

        if (losRelationIter != losRelationBetweenRoads.end()) {
            const RoadLosRelationData& losRelation = (*losRelationIter).second;

            if (losRelation.relationType == ROAD_LOSRELATION_LOS) {
                continue;
            }
        }

        if (ignoreRoadIds.find(endRoadId) != ignoreRoadIds.end() ||
            nlosPath.ContainsRoad(endRoadId)) {
            continue;
        }

        const Road& road = roads[endRoadId];

        NlosPathData newNlosPath = nlosPath.Clone();

        const IntersectionIdType intersectionId1 = road.GetStartIntersectionId();
        const IntersectionIdType intersectionId2 = road.GetEndIntersectionId();

        const double distance1 =
            SquaredXYDistanceBetweenVertices(
                endIntersectionPos,
                intersections[intersectionId1].GetVertex());

        const double distance2 =
            SquaredXYDistanceBetweenVertices(
                endIntersectionPos,
                intersections[intersectionId2].GetVertex());

        IntersectionIdType expandIntersectionId;

        if (distance1 < distance2) {
            expandIntersectionId = intersectionId1;
        } else {
            expandIntersectionId = intersectionId2;
        }

        newNlosPath.ExpandRoad(startRoadId, endIntersectionId, endRoadId, expandIntersectionId);

        if (newNlosPath.GetNlosPathDistance(roadLayerPtr->intersections) <= maxNlosDistance) {
            newNlosPath.pathValue = nlosValueCalculatorPtr->GetNlosPathValue(newNlosPath);
            nlosPaths.push_back(newNlosPath);
        }
    }
}

void RoadLosChecker::OutputLosRelation() const
{
    const vector<Intersection>& intersections = roadLayerPtr->intersections;
    const vector<Road>& roads = roadLayerPtr->roads;

    typedef map<std::pair<RoadIdType, RoadIdType>, RoadLosRelationData>::const_iterator LosRelationIter;

    for(LosRelationIter iter = losRelationBetweenRoads.begin();
        iter != losRelationBetweenRoads.end(); iter++) {

        const std::pair<RoadIdType, RoadIdType>& roadIdPair = (*iter).first;
        const RoadLosRelationData& losRelation = (*iter).second;
        const Road& road1 = roads[roadIdPair.first];
        const Road& road2 = roads[roadIdPair.second];

        const Vertex& road1StartPos = road1.GetStartVertex();
        const Vertex& road1EndPos = road1.GetEndVertex();

        const Vertex& road2StartPos = road2.GetStartVertex();
        const Vertex& road2EndPos = road2.GetEndVertex();

        cout << roadIdPair.first
             << "(" << road1StartPos.x << "," << road1StartPos.y << ")"
             << "(" << road1EndPos.x << "," << road1EndPos.y << ")" << ", "
             << roadIdPair.second
             << "(" << road2StartPos.x << "," << road2StartPos.y << ")"
             << "(" << road2EndPos.x << "," << road2EndPos.y << "):";

        if (losRelation.relationType == ROAD_LOSRELATION_LOS) {
            cout << "LoS" << endl;
        } else if (losRelation.relationType == ROAD_LOSRELATION_NLOS) {
            cout << "NLoS";

            const map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>& nlosPaths = losRelation.nlosPaths;

            typedef map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>::const_iterator NlosIter;

            int i = 0;
            for(NlosIter nlosIter = nlosPaths.begin(); nlosIter != nlosPaths.end(); nlosIter++, i++) {
                const NlosPathData& nlosPath = (*nlosIter).second;
                const shared_array<RoadIdType> endToEndRoadIds = nlosPath.endToEndRoadIds;
                const shared_array<IntersectionIdType> endToEndIntersectionIds = nlosPath.endToEndIntersectionIds;

                if (losRelation.nlosPaths.size() > 1) {
                    cout << endl;
                }

                cout << "  - Path" << i << "(Road";
                for(unsigned int j = 0; j < nlosPath.GetNumberOfRoads(); j++) {
                    cout <<  "," << endToEndRoadIds[j];
                }
                cout << ",Intersection";
                for(unsigned int j = 0; j < nlosPath.GetNumberOfIntersections(); j++) {

                    const Vertex& position = intersections[endToEndIntersectionIds[j]].GetVertex();

                    cout << "," << endToEndIntersectionIds[j]
                         << "(" << position.x << "," << position.y << ")";

                }
                cout << ")";

            }
            cout << endl;

        } else {
            cout << "out of NLoS" << endl;
        }
    }
}

const RoadLosRelationData& RoadLosChecker::GetLosRelation(
    const RoadIdType& roadId1,
    const RoadIdType& roadId2) const
{
    const std::pair<RoadIdType, RoadIdType> roadPair =
        make_pair(roadId1, roadId2);

    typedef map<std::pair<RoadIdType, RoadIdType>, RoadLosRelationData>::const_iterator IterType;

    IterType iter = losRelationBetweenRoads.find(roadPair);

    if (iter != losRelationBetweenRoads.end()) {
        return (*iter).second;
    }

    return outofnlosRelation;
}

bool RoadLosChecker::PositionsAreLineOfSight(
    const Vertex& position1,
    const Vertex& position2) const
{
    vector<RoadIdType> roadIds1;
    roadLayerPtr->GetRoadIdsAt(position1, roadIds1);

    vector<RoadIdType> roadIds2;
    roadLayerPtr->GetRoadIdsAt(position2, roadIds2);

    for(size_t i = 0; i < roadIds1.size(); i++) {
        const RoadIdType& roadId1 = roadIds1[i];

        for(size_t j = 0; j < roadIds2.size(); j++) {
            const RoadIdType& roadId2 = roadIds2[j];

            const RoadLosRelationData& losRelationData =
                (*this).GetLosRelation(roadId1, roadId2);

            if (losRelationData.relationType == ROAD_LOSRELATION_LOS) {
                return true;
            }
        }
    }

    return false;
}

//------------------------------------------------

double SquaredXYZDistanceBetweenVertices(
    const Vertex& vertex1,
    const Vertex& vertex2)
{
    const double deltaX = vertex1.x - vertex2.x;
    const double deltaY = vertex1.y - vertex2.y;
    const double deltaZ = vertex1.z - vertex2.z;

    return ((deltaX * deltaX) + (deltaY * deltaY) + (deltaZ * deltaZ));

}//SquaredXYZDistanceBetweenVertices//

double XYZDistanceBetweenVertices(
    const Vertex& vertex1,
    const Vertex& vertex2)
{
    return (sqrt(SquaredXYZDistanceBetweenVertices(vertex1, vertex2)));

}//XYZDistanceBetweenVertices//

double SquaredXYDistanceBetweenVertices(
    const Vertex& vertex1,
    const Vertex& vertex2)
{
    const double deltaX = vertex1.x - vertex2.x;
    const double deltaY = vertex1.y - vertex2.y;

    return ((deltaX * deltaX) + (deltaY * deltaY));

}//SquaredXYDistanceBetweenVertices//

double XYDistanceBetweenVertices(
    const Vertex& vertex1,
    const Vertex& vertex2)
{
    return (sqrt(SquaredXYDistanceBetweenVertices(vertex1, vertex2)));

}//XYZDistanceBetweenVertices//

void CalculateAzimuthAndElevationDegrees(
    const Vertex& firstPoint,
    const Vertex& secondPoint,
    double& azimuthDegrees,
    double& elevationDegrees)
{
    const double deltaX =
        secondPoint.x - firstPoint.x;
    const double deltaY =
        secondPoint.y - firstPoint.y;
    const double deltaZ =
        secondPoint.z - firstPoint.z;
    const double xyDistanceSquared =
        deltaX * deltaX + deltaY * deltaY;

    azimuthDegrees = 0;
    azimuthDegrees =
        90.0 - (180.0 / PI) * (std::atan2(deltaY, deltaX));

    elevationDegrees = (180.0 / PI) * (std::atan2(deltaZ, (sqrt(xyDistanceSquared))));

    NormalizeAzimuthAndElevation(azimuthDegrees, elevationDegrees);

}

double CalculateFullRadiansBetweenVector(
    const Vertex& vertex1,
    const Vertex& middleVertex,
    const Vertex& vertex2)
{
    const double rad1 = (std::atan2(vertex1.y - middleVertex.y, vertex1.x - middleVertex.x));
    const double rad2 = (std::atan2(vertex2.y - middleVertex.y, vertex2.x - middleVertex.x));

    const double totalRad = rad2 - rad1;

    if (totalRad < 0) {
        return totalRad + 2*PI;
    } else if (totalRad > 2*PI) {
        return totalRad - 2*PI;
    }

    return totalRad;
}

double CalculateRadiansBetweenVector(
    const Vertex& vertex1,
    const Vertex& middleVertex,
    const Vertex& vertex2)
{
    const Vertex vector1 = vertex1 - middleVertex;
    const Vertex vector2 = vertex2 - middleVertex;

    const double distance1 = vector1.XYDistance();
    const double distance2 = vector2.XYDistance();

    if (distance1 == 0 || distance2 == 0) {
        return 0;
    }

    const double cosAlpha =
        (vector1.x*vector2.x +
         vector1.y*vector2.y) /
        (distance1*distance2);

    return ((std::acos(std::min(1., std::max(-1., cosAlpha)))));
}

static inline
double CalculatePointToLineCrossPoint(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2)
{
    const double dx = lineEdge2.x - lineEdge1.x;
    const double dy = lineEdge2.y - lineEdge1.y;
    const double dz = lineEdge2.z - lineEdge1.z;

    const double innerProduct =
        dx * (lineEdge1.x - point.x) +
        dy * (lineEdge1.y - point.y) +
        dz * (lineEdge1.z - point.z);

    double t;
    if (innerProduct >= 0.0) {
        t = 0.0;
    } else {
        t = - innerProduct / (dx*dx + dy*dy + dz*dz);

        if (t > 1.0) {
            t = 1.0;
        }
    }

    return t;
}

Vertex CalculatePointToLineNearestPosition(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2)
{
    const double t = CalculatePointToLineCrossPoint(point, lineEdge1, lineEdge2);

    return (lineEdge2 - lineEdge1)*t + lineEdge1;
}

Vertex CalculatePointToHorizontalLineNearestPosition(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2)
{
    const double t = CalculatePointToLineCrossPoint(
        point.XYPoint(),
        lineEdge1.XYPoint(),
        lineEdge2.XYPoint());

    return (lineEdge2 - lineEdge1)*t + lineEdge1;
}

double CalculatePointToLineDistance(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2)
{
    return CalculatePointToLineNearestPosition(
        point, lineEdge1, lineEdge2).DistanceTo(point);
}

Vertex CalculateIntersectionPosition(
    const Vertex& point,
    const Vertex& lineEdge1,
    const Vertex& lineEdge2)
{
    const double dx = lineEdge2.x - lineEdge1.x;
    const double dy = lineEdge2.y - lineEdge1.y;
    const double dz = lineEdge2.z - lineEdge1.z;

    const double innerProduct =
        dx * (lineEdge1.x - point.x) +
        dy * (lineEdge1.y - point.y) +
        dz * (lineEdge1.z - point.z);

    const double t = - innerProduct / (dx*dx + dy*dy + dz*dz);

    return Vertex(
        t * dx + lineEdge1.x,
        t * dy + lineEdge1.y,
        t * dz + lineEdge1.z);
}

static inline
Vertex CalculatePointToPlaneNearestPoint(
    const Vertex& point,
    const Vertex& planeV1,
    const Vertex& planeV2,
    const Vertex& planeV3)
{
    const Vertex planeVector = (planeV2 - planeV1).Cross(planeV3 - planeV1).Normalized();
    const double d = -planeVector.Dot(planeV1);
    const double distance = planeVector.Dot(point) + d;

    return point - planeVector*distance;
}

Vertex CalculatePointToRectNearestVertex(
    const Vertex& point,
    const Vertex& planeBottomLeft,
    const Vertex& planeBottomRight,
    const Vertex& planeTopRight,
    const Vertex& planeTopLeft)
{
    const Vertex nearestPlaneVertex =
        CalculatePointToPlaneNearestPoint(point, planeBottomLeft, planeBottomRight, planeTopRight);

    if (IsInsideOf3dTheTriangle(nearestPlaneVertex, planeBottomLeft, planeBottomRight, planeTopRight) ||
        IsInsideOf3dTheTriangle(nearestPlaneVertex, planeBottomRight, planeTopRight, planeTopLeft)) {

        return nearestPlaneVertex;
    }

    vector<Vertex> polygonVertices;

    polygonVertices.push_back(planeBottomLeft);
    polygonVertices.push_back(planeBottomRight);
    polygonVertices.push_back(planeTopRight);
    polygonVertices.push_back(planeTopLeft);
    polygonVertices.push_back(planeBottomLeft);

    Vertex nearestPosition;
    size_t vertexNumber;

    CalculatePointToArcNearestPosition(polygonVertices, nearestPlaneVertex, nearestPosition, vertexNumber);

    return nearestPosition;
}

Vertex CalculatePointToPolygonNearestVertex(
    const Vertex& point,
    const vector<Vertex>& planePolygon)
{
    if (planePolygon.size() >= 4 &&
        planePolygon.front() == planePolygon.back()) {

        vector<Triangle> triangles;

        PolygonToTriangles(planePolygon, triangles);

        assert(!triangles.empty());

        const Triangle& topTriangle = triangles[0];
        const Vertex nearestPlaneVertex =
            CalculatePointToPlaneNearestPoint(point, topTriangle.GetP1(), topTriangle.GetP2(), topTriangle.GetP3());

        for(size_t i = 0; i < triangles.size(); i++) {
            const Triangle& triangle = triangles[i];

            if (IsInsideOf3dTheTriangle(nearestPlaneVertex, triangle.GetP1(), triangle.GetP2(), triangle.GetP3())) {
                return nearestPlaneVertex;
            }
        }
    }

    Vertex nearestPosition;
    size_t vertexNumber;

    CalculatePointToArcNearestPosition(planePolygon, point, nearestPosition, vertexNumber);

    return nearestPosition;
}

Vertex CalculatePointTo3dPolygonNearestVertex(
    const Vertex& point,
    const vector<Vertex>& vertices,
    const double polygonHeight)
{
    assert(!vertices.empty());

    if (vertices.size() == 1) {
        return vertices.front();
    }

    if (polygonHeight == 0.) {
        return CalculatePointToPolygonNearestVertex(point, vertices);
    }

    Vertex nearestPosition = vertices.front();
    size_t vertexNumber;

    CalculatePointToArcNearestVertex(vertices, point, nearestPosition, vertexNumber);

    double minDistance = DBL_MAX;

    const Vertex hv(0., 0., polygonHeight);

    if (vertexNumber > 0) {
        const Vertex& v1 = vertices[vertexNumber - 1];
        const Vertex& v2 = vertices[vertexNumber];

        const Vertex rectPoosition =
            CalculatePointToRectNearestVertex(point, v1, v2, v2 + hv, v1 + hv);
        const double distance = point.DistanceTo(rectPoosition);

        if (distance < minDistance) {
            minDistance = distance;
            nearestPosition = rectPoosition;
        }
    }
    if (vertexNumber < vertices.size() - 1) {
        const Vertex& v1 = vertices[vertexNumber];
        const Vertex& v2 = vertices[vertexNumber + 1];

        const Vertex rectPoosition =
            CalculatePointToRectNearestVertex(point, v1, v2, v2 + hv, v1 + hv);
        const double distance = point.DistanceTo(rectPoosition);

        if (distance < minDistance) {
            minDistance = distance;
            nearestPosition = rectPoosition;
        }
    }

    // if closed polugon, check roof and floor plane.
    if (vertices.front() == vertices.back()) {
        const double polygonZ = vertices.front().z;

        if (point.z > polygonZ + polygonHeight) {
            vector<Vertex> roofVertices = vertices;

            for(size_t i = 0; i < vertices.size(); i++) {
                roofVertices[i] += hv;
            }

            const Vertex polygonPosition =
                CalculatePointToPolygonNearestVertex(point, roofVertices);
            const double distance = point.DistanceTo(polygonPosition);

            if (distance < minDistance) {
                minDistance = distance;
                nearestPosition = polygonPosition;
            }

        } else if (point.z < polygonZ) {
            const Vertex polygonPosition =
                CalculatePointToPolygonNearestVertex(point, vertices);
            const double distance = point.DistanceTo(polygonPosition);

            if (distance < minDistance) {
                minDistance = distance;
                nearestPosition = polygonPosition;
            }
        }
    }

    return nearestPosition;
}

bool HorizontalLinesAreIntersection(
    const Vertex& lineEdge11,
    const Vertex& lineEdge12,
    const Vertex& lineEdge21,
    const Vertex& lineEdge22)
{
    if (lineEdge11.x >= lineEdge12.x) {
        if ((lineEdge11.x < lineEdge21.x && lineEdge11.x < lineEdge22.x) ||
            (lineEdge12.x > lineEdge21.x && lineEdge12.x > lineEdge22.x)) {
            return false;
        }
    } else {
        if ((lineEdge12.x < lineEdge21.x && lineEdge12.x < lineEdge22.x) ||
            (lineEdge11.x > lineEdge21.x && lineEdge11.x > lineEdge22.x)) {
            return false;
        }
    }

    if (lineEdge11.y >= lineEdge12.y) {
        if ((lineEdge11.y < lineEdge21.y && lineEdge11.y < lineEdge22.y) ||
            (lineEdge12.y > lineEdge21.y && lineEdge12.y > lineEdge22.y)) {
            return false;
        }
    } else {
        if ((lineEdge12.y < lineEdge21.y && lineEdge12.y < lineEdge22.y) ||
            (lineEdge11.y > lineEdge21.y && lineEdge11.y > lineEdge22.y)) {
            return false;
        }
    }

    if (((lineEdge11.x - lineEdge12.x) * (lineEdge21.y - lineEdge11.y) +
         (lineEdge11.y - lineEdge12.y) * (lineEdge11.x - lineEdge21.x)) *
        ((lineEdge11.x - lineEdge12.x) * (lineEdge22.y - lineEdge11.y) +
         (lineEdge11.y - lineEdge12.y) * (lineEdge11.x - lineEdge22.x)) > 0) {
        return false;
    }
    if (((lineEdge21.x - lineEdge22.x) * (lineEdge11.y - lineEdge21.y) +
         (lineEdge21.y - lineEdge22.y) * (lineEdge21.x - lineEdge11.x)) *
        ((lineEdge21.x - lineEdge22.x) * (lineEdge12.y - lineEdge21.y) +
         (lineEdge21.y - lineEdge22.y) * (lineEdge21.x - lineEdge12.x)) > 0) {
        return false;
    }

    return true;
}

static inline
Vertex CalculateMiddlePoint(const vector<Vertex>& points, const double ratio)
{
    double distanceToPoint = CalculateArcDistance(points) * ratio;

    assert(!points.empty());
    Vertex middlePoint = points.back();

    for(size_t i = 0; i < points.size() - 1; i++) {
        const Vertex& p1 = points[i];
        const Vertex& p2 = points[i+1];

        const double aLineLength = p1.DistanceTo(p2);

        if (distanceToPoint < aLineLength) {
            middlePoint =  (p2 - p1)*(distanceToPoint/aLineLength) + p1;
            break;
        }

        distanceToPoint -= aLineLength;
    }

    return middlePoint;
}

bool PolygonContainsPoint(
    const vector<Vertex>& vertices,
    const Vertex& point)
{
    if (vertices.size() < 4 || vertices.front() != vertices.back()) {
        return false;
    }

    // Jordan Curve Theorem

    size_t verticallyUpLineCrossingCount = 0;

    for(size_t i = 1; i < vertices.size(); i++) {
        const Vertex& p1 = vertices[i-1];
        const Vertex& p2 = vertices[i];

        if ((point.x < p1.x && point.x < p2.x) ||
            (point.x > p1.x && point.x > p2.x)) {
            continue;
        }

        if (p1.x == p2.x) {
            if ((p1.y <= point.y && point.y <= p2.y) ||
                (p2.y <= point.y && point.y <= p1.y)) {
                // The point is on the line.
                return true;
            }
            continue;
        }

        if (point.x == std::min(p1.x, p2.x)) {
            continue;
        }

        const double crossingY =
            (p1.y + double(p2.y - p1.y)/(p2.x - p1.x) * (point.x - p1.x));

        if (crossingY > point.y) {

            verticallyUpLineCrossingCount++;

        } else if (crossingY == point.y) {
            return true;
        }
    }//for//

    return ((verticallyUpLineCrossingCount % 2) == 1);
}

static inline
bool CompleteCoveredPolygon(
    const vector<Vertex>& outsidePolygon,
    const vector<Vertex>& polygon)
{
    for(size_t i = 0; i < polygon.size() - 1; i++) {
        if (!PolygonContainsPoint(outsidePolygon, polygon[i])) {
            return false;
        }
    }

    return true;
}

static inline
bool RectIsIntersectsWithPolygon(
    const Rectangle& rect,
    const vector<Vertex>& polygon)
{
    for(size_t i = 0; i < polygon.size() - 1; i++) {
        if (rect.Contains(polygon[i])) {
            return true;
        }
    }

    const Vertex topLeft(rect.minX, rect.maxY);
    const Vertex bottomLeft(rect.minX, rect.minY);
    const Vertex topRight(rect.maxX,rect.maxY);
    const Vertex bottomRight(rect.maxX, rect.minY);

    if (PolygonContainsPoint(polygon, topLeft) ||
        PolygonContainsPoint(polygon, bottomLeft) ||
        PolygonContainsPoint(polygon, topRight) ||
        PolygonContainsPoint(polygon, bottomRight)) {
        return true;
    }

    for(size_t i = 0; i < polygon.size() - 1; i++) {
        const Vertex& p1 = polygon[i];
        const Vertex& p2 = polygon[i+1];

        if (HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft) ||
            HorizontalLinesAreIntersection(p1, p2, bottomLeft, bottomRight) ||
            HorizontalLinesAreIntersection(p1, p2, bottomRight, topRight) ||
            HorizontalLinesAreIntersection(p1, p2, topRight, topLeft)) {
            return true;
        }
    }

    return false;
}

static inline
bool LinesAreParallel(
    const Vertex& lineEdge11,
    const Vertex& lineEdge12,
    const Vertex& lineEdge21,
    const Vertex& lineEdge22)
{
    const Vertex vector1 = lineEdge12 - lineEdge11;
    const Vertex vector2 = lineEdge22 - lineEdge21;
    const Vertex vector3 = lineEdge21 - lineEdge11;

    return ((vector1.x*vector2.y) - (vector1.y*vector2.x) == 0);
}

Vertex CalculateIntersectionPositionBetweenLine(
    const Vertex& baseLineEdge11,
    const Vertex& baseLineEdge12,
    const Vertex& horizontalCossedLineEdge21,
    const Vertex& horizontalCossedLineEdge22)
{
    const Vertex vector1 = baseLineEdge12 - baseLineEdge11;
    const Vertex vector2 = horizontalCossedLineEdge22 - horizontalCossedLineEdge21;
    const Vertex vector3 = horizontalCossedLineEdge21 - baseLineEdge11;

    const double denom = (vector1.x*vector2.y) - (vector1.y*vector2.x);

    if (denom == 0) {
        // return middle point
        return (baseLineEdge11+baseLineEdge12+horizontalCossedLineEdge21+horizontalCossedLineEdge22) /4.;
    }

    assert(denom != 0);

    const double t =
        ((vector2.y*vector3.x) - (vector2.x*vector3.y))/denom;

    return (baseLineEdge12 - baseLineEdge11)*t + baseLineEdge11;
}

void GetIntersectionPositionBetweenLineAndPolygon(
    const Vertex& lineEdge1,
    const Vertex& lineEdge2,
    const vector<Vertex>& polygon,
    vector<Vertex>& intersectionPoints)
{
    intersectionPoints.clear();

    if (PolygonContainsPoint(polygon, lineEdge1)) {
        intersectionPoints.push_back(lineEdge1);
    }

    for(size_t i = 0; i < polygon.size() - 1; i++) {
        const Vertex& p1 = polygon[i];
        const Vertex& p2 = polygon[i+1];

        if (HorizontalLinesAreIntersection(lineEdge1, lineEdge2, p1, p2)) {
            const Vertex& intersectionPos =
                CalculateIntersectionPositionBetweenLine(
                    lineEdge1, lineEdge2, p1, p2);

            if (intersectionPos != lineEdge1 &&
                intersectionPos != lineEdge2) {
                intersectionPoints.push_back(intersectionPos);
            }
        }
    }

    if (PolygonContainsPoint(polygon, lineEdge2)) {
        intersectionPoints.push_back(lineEdge2);
    }
}

//----------------------------------------------------------------
// VertexMap Mesh
//----------------------------------------------------------------

SpatialObjectMap::SpatialObjectMap()
    :
    meshUnit(0),
    numberHorizontalMeshes(0),
    numberVerticalMeshes(0)
{
}

void SpatialObjectMap::SetMesh(
    const Rectangle& initMinRect,
    const double& initMeshUnit,
    const size_t maxMeshSize)
{
    minRect = initMinRect;
    meshUnit = initMeshUnit;

    numberHorizontalMeshes =
        std::max<size_t>(1, size_t(ceil((minRect.maxX - minRect.minX) / meshUnit)));

    numberVerticalMeshes =
        std::max<size_t>(1, size_t(ceil((minRect.maxY - minRect.minY) / meshUnit)));

    if ((numberHorizontalMeshes * numberVerticalMeshes) > (maxMeshSize * maxMeshSize)) {
        if (numberHorizontalMeshes > numberVerticalMeshes) {
            numberHorizontalMeshes =
                size_t(ceil((double)(maxMeshSize * maxMeshSize) / numberVerticalMeshes));
            meshUnit = ceil((minRect.maxX - minRect.minX) / numberHorizontalMeshes);
        } else {
            numberVerticalMeshes =
                size_t(ceil((double)(maxMeshSize * maxMeshSize) / numberHorizontalMeshes));
            meshUnit = ceil((minRect.maxY - minRect.minY) / numberVerticalMeshes);
        }
    }//if//

    assert(numberHorizontalMeshes * numberVerticalMeshes != 0);

    ids.clear();
    ids.resize(numberHorizontalMeshes * numberVerticalMeshes);
}

void SpatialObjectMap::RemoveGisObject(
    const GisObject& gisObject,
    const VariantIdType& variantId)
{
    Rectangle rect = gisObject.GetMinRectangle();

    if (rect.OverlappedWith(minRect)) {
        rect = rect.GetOverlappedRectangle(minRect);
    }

    const size_t minHorizontalId = (*this).GetMinHorizontalId(rect);
    const size_t maxHorizontalId = (*this).GetMaxHorizontalId(rect);
    const size_t minVerticalId = (*this).GetMinVerticalId(rect);
    const size_t maxVerticalId = (*this).GetMaxVerticalId(rect);

    for(size_t verticalId = minVerticalId; verticalId <= maxVerticalId; verticalId++) {
        for(size_t horizontalId = minHorizontalId; horizontalId <= maxHorizontalId; horizontalId++) {

            const Rectangle meshRect(
                minRect.minX + meshUnit*horizontalId,
                minRect.minY + meshUnit*verticalId,
                minRect.minX + meshUnit*(horizontalId+1),
                minRect.minY + meshUnit*(verticalId+1));

            if (gisObject.IntersectsWith(meshRect)) {
                const size_t meshId = numberHorizontalMeshes * verticalId + horizontalId;
                ids[meshId].erase(variantId);
            }
        }
    }
}

void SpatialObjectMap::InsertGisObject(
    const GisObject& gisObject,
    const VariantIdType& variantId)
{
    Rectangle rect = gisObject.GetMinRectangle();

    if (rect.OverlappedWith(minRect)) {
        rect = rect.GetOverlappedRectangle(minRect);
    }

    const size_t minHorizontalId = (*this).GetMinHorizontalId(rect);
    const size_t maxHorizontalId = (*this).GetMaxHorizontalId(rect);
    const size_t minVerticalId = (*this).GetMinVerticalId(rect);
    const size_t maxVerticalId = (*this).GetMaxVerticalId(rect);

    for(size_t verticalId = minVerticalId; verticalId <= maxVerticalId; verticalId++) {
        for(size_t horizontalId = minHorizontalId; horizontalId <= maxHorizontalId; horizontalId++) {

            const Rectangle meshRect(
                minRect.minX + meshUnit*horizontalId,
                minRect.minY + meshUnit*verticalId,
                minRect.minX + meshUnit*(horizontalId+1),
                minRect.minY + meshUnit*(verticalId+1));

            if (gisObject.IntersectsWith(meshRect)) {
                const size_t meshId = numberHorizontalMeshes * verticalId + horizontalId;
                ids[meshId].insert(variantId);
            }
        }
    }
}

void SpatialObjectMap::InsertVertex(
    const Vertex& vertex,
    const VariantIdType& variantId)
{
    const size_t horizontalId = (*this).GetHorizontalId(vertex);
    const size_t verticalId = (*this).GetVerticalId(vertex);

    const size_t meshId = numberHorizontalMeshes * verticalId + horizontalId;

    ids[meshId].insert(variantId);
}

void SpatialObjectMap::GetGisObject(
    const Rectangle& targetRect,
    list<VariantIdType>& variantIds) const
{
    variantIds.clear();

    if (!(*this).IsAvailable()) {
        return;
    }

    if (!targetRect.OverlappedWith(minRect)) {
        return;
    }

    const Rectangle rect = targetRect.GetOverlappedRectangle(minRect);
    const size_t minHorizontalId = (*this).GetMinHorizontalId(rect);
    const size_t maxHorizontalId = (*this).GetMaxHorizontalId(rect);
    const size_t minVerticalId = (*this).GetMinVerticalId(rect);
    const size_t maxVerticalId = (*this).GetMaxVerticalId(rect);

    set<VariantIdType> uniqueIds;

    for(size_t verticalId = minVerticalId; verticalId <= maxVerticalId; verticalId++) {
        for(size_t horizontalId = minHorizontalId; horizontalId <= maxHorizontalId; horizontalId++) {

            const size_t meshId = numberHorizontalMeshes * verticalId + horizontalId;

            if (meshId < ids.size()) {
                const set<VariantIdType>& targetIds = ids[meshId];

                uniqueIds.insert(targetIds.begin(), targetIds.end());
            }//if//
        }//for//
    }//for//

    variantIds.assign(uniqueIds.begin(), uniqueIds.end());
}

void SpatialObjectMap::GetGisObject(
    const Vertex& pos,
    list<VariantIdType>& variantIds) const
{
    const double integrationLength = 0.01;//1cm
    const Rectangle searchRect(pos, integrationLength);

    (*this).GetGisObject(searchRect, variantIds);
}

size_t SpatialObjectMap::GetMinHorizontalId(const Rectangle& rect) const
{
    const size_t horizontalId = size_t(floor((rect.minX - minRect.minX) / meshUnit));

    return std::max<size_t>(0, std::min(horizontalId, numberHorizontalMeshes - 1));
}
size_t SpatialObjectMap::GetMaxHorizontalId(const Rectangle& rect) const
{
    const size_t horizontalId = size_t(ceil((rect.maxX - minRect.minX) / meshUnit));

    return std::max<size_t>(0, std::min(horizontalId, numberHorizontalMeshes - 1));
}
size_t SpatialObjectMap::GetMinVerticalId(const Rectangle& rect) const
{
    const size_t verticalId = size_t(floor((rect.minY - minRect.minY) / meshUnit));

    return std::max<size_t>(0, std::min(verticalId, numberVerticalMeshes - 1));
}
size_t SpatialObjectMap::GetMaxVerticalId(const Rectangle& rect) const
{
    const size_t verticalId = size_t(ceil((rect.maxY - minRect.minY) / meshUnit));

    return std::max<size_t>(0, std::min(verticalId, numberVerticalMeshes - 1));
}
size_t SpatialObjectMap::GetHorizontalId(const Vertex& vertex) const
{
    const size_t horizontalId = size_t(floor((vertex.x - minRect.minX) / meshUnit));

    return std::max<size_t>(0, std::min(horizontalId, numberHorizontalMeshes - 1));
}
size_t SpatialObjectMap::GetVerticalId(const Vertex& vertex) const
{
    const size_t verticalId = size_t(floor((vertex.y - minRect.minY) / meshUnit));

    return std::max<size_t>(0, std::min(verticalId, numberVerticalMeshes - 1));
}

//--------------------------------------------------------


MaterialSet::MaterialSet()
    :
    defaultMaterialPtr(new Material("", 0)),
    cacheIter(materialIds.end())
{}

void MaterialSet::AddMaterial(
    const string& name,
    const double transmissionLossDb)
{
    assert(materialIds.find(name) == materialIds.end());

    const MaterialIdType materialId =
        static_cast<MaterialIdType>(materialIds.size());

    materialIds[name] = materialId;

    materialPtrs.push_back(
        shared_ptr<Material>(new Material(name, transmissionLossDb)));
}

MaterialIdType MaterialSet::GetMaterialId(const string& name) const
{
    if (cacheIter == materialIds.end() ||
        (*cacheIter).first != name) {

        cacheIter = materialIds.find(name);

        if (cacheIter == materialIds.end()) {
            return INVALID_MATERIAL_ID;
        }
    }

    return (*cacheIter).second;
}

const Material& MaterialSet::GetMaterial(const string& name) const
{
    return (*this).GetMaterial((*this).GetMaterialId(name));
}

const Material& MaterialSet::GetMaterial(const MaterialIdType& materialId) const
{
    if (materialId == INVALID_MATERIAL_ID) {
        return *defaultMaterialPtr;
    }

    return *materialPtrs[materialId];
}

//------------------------------------------------------------------

struct GisObject::Implementation {
    GisSubsystem* subsystemPtr;

    const GisObjectType objectType;
    const GisObjectIdType objectId;
    const VariantIdType variantId;

    bool isEnabled;

    bool outputGisTrace;
    bool outputMasTrace;

    vector<VertexIdType> vertexIds;
    Rectangle minRectangle;
    double elevationFromGroundMeters;
    string objectName;

    MaterialIdType materialId;

    Implementation(
        GisSubsystem* initSubsystemPtr,
        const GisObjectType& initObjectType,
        const GisObjectIdType& initObjectId,
        const VariantIdType& initVariantId)
        :
        subsystemPtr(initSubsystemPtr),
        objectType(initObjectType),
        objectId(initObjectId),
        variantId(initVariantId),
        isEnabled(true),
        outputGisTrace(false),
        outputMasTrace(false),
        elevationFromGroundMeters(0.),
        materialId(INVALID_MATERIAL_ID)
    {}
};

class GisObject::GisObjectEnableDisableEvent : public SimulationEvent {
public:
    GisObjectEnableDisableEvent(
        const shared_ptr<Implementation>& initCommonImplPtr,
        const bool initIsEnable)
        :
        commonImplPtr(initCommonImplPtr),
        isEnable(initIsEnable)
    {}

    virtual void ExecuteEvent() {
        commonImplPtr->subsystemPtr->SetEnabled(
            commonImplPtr->objectType,
            commonImplPtr->variantId,
            isEnable);
    }

private:
    shared_ptr<Implementation> commonImplPtr;
    bool isEnable;
};

GisObject::GisObject(
    GisSubsystem* initSubsystemPtr,
    const GisObjectType& initObjectType,
    const GisObjectIdType& initObjectId,
    const VariantIdType& initVariantId)
    :
    commonImplPtr(
        new Implementation(initSubsystemPtr, initObjectType, initObjectId, initVariantId))
{
}

GisObject::GisObject(
    GisSubsystem* initSubsystemPtr,
    const GisObjectType& initObjectType,
    const GisObjectIdType& initObjectId,
    const VariantIdType& initVariantId,
    const VertexIdType& initVertexId)
    :
    commonImplPtr(
        new Implementation(initSubsystemPtr, initObjectType, initObjectId, initVariantId))
{
    commonImplPtr->vertexIds.push_back(initVertexId);
}

void GisObject::LoadParameters(const ParameterDatabaseReader& theParameterDatabaseReader)
{
    const GisObjectIdType objectId = commonImplPtr->objectId;

    if (theParameterDatabaseReader.ParameterExists("gisobject-disable-time", objectId)) {

        const TimeType disableTime = theParameterDatabaseReader.ReadTime("gisobject-disable-time", objectId);

        if (disableTime < INFINITE_TIME) {
            commonImplPtr->subsystemPtr->ScheduleGisEvent(
                shared_ptr<SimulationEvent>(
                    new GisObjectEnableDisableEvent(commonImplPtr, false)),
                disableTime);
        }
    }

    if (theParameterDatabaseReader.ParameterExists("gisobject-enable-time", objectId)) {

        const TimeType enableTime = theParameterDatabaseReader.ReadTime("gisobject-enable-time", objectId);

        if (enableTime < INFINITE_TIME) {
            commonImplPtr->subsystemPtr->ScheduleGisEvent(
                shared_ptr<SimulationEvent>(
                    new GisObjectEnableDisableEvent(commonImplPtr, true)),
                enableTime);
        }
    }

    if (theParameterDatabaseReader.ParameterExists("trace-enabled-tags", objectId)) {
        string enabledTagsString = theParameterDatabaseReader.ReadString("trace-enabled-tags", objectId);
        ConvertStringToLowerCase(enabledTagsString);

        if (enabledTagsString.find("gis") != string::npos) {
            commonImplPtr->outputGisTrace = true;
        }
        if (enabledTagsString.find("mas") != string::npos) {
            commonImplPtr->outputMasTrace = true;
        }
    }
}

GisObjectIdType GisObject::GetObjectId() const
{
    return commonImplPtr->objectId;
}

const string& GisObject::GetObjectName() const
{
    return commonImplPtr->objectName;
}

GisObjectType GisObject::GetObjectType() const
{
    return commonImplPtr->objectType;
}

const Vertex& GisObject::GetVertex(const size_t index) const
{
    return commonImplPtr->subsystemPtr->GetVertex(commonImplPtr->vertexIds.at(index));
}

const VertexIdType& GisObject::GetVertexId(const size_t index) const
{
    assert(index < commonImplPtr->vertexIds.size());
    return commonImplPtr->vertexIds.at(index);
}

const VertexIdType& GisObject::GetStartVertexId() const
{
    return commonImplPtr->vertexIds.front();
}

const VertexIdType& GisObject::GetEndVertexId() const
{
    return commonImplPtr->vertexIds.back();
}

const VertexIdType& GisObject::GetNearestVertexId(const Vertex& vertex) const
{
    double minDistance = DBL_MAX;
    size_t nearesVeretxNumber = 0;

    assert(!commonImplPtr->vertexIds.empty());

    for(size_t i = 0; i < commonImplPtr->vertexIds.size(); i++) {
        const double distance = vertex.DistanceTo((*this).GetVertex(i));

        if (distance < minDistance) {
            minDistance = distance;
            nearesVeretxNumber = i;
        }
    }

    return commonImplPtr->vertexIds.at(nearesVeretxNumber);
}

bool GisObject::IsStartOrEndVertex(const VertexIdType& vertexId) const
{
    return ((*this).IsStartVertex(vertexId) ||
            (*this).IsEndVertex(vertexId));
}

bool GisObject::IsStartVertex(const VertexIdType& vertexId) const
{
    return (commonImplPtr->vertexIds.front() == vertexId);
}

bool GisObject::IsEndVertex(const VertexIdType& vertexId) const
{
    return (commonImplPtr->vertexIds.back() == vertexId);
}

bool GisObject::ContainsVertexId(const VertexIdType& vertexId) const
{
    for(size_t i = 0; i < commonImplPtr->vertexIds.size(); i++) {
        if (commonImplPtr->vertexIds[i] == vertexId) {
            return true;
        }
    }

    return false;
}

const VertexIdType& GisObject::GetOthersideVertexId(const VertexIdType& vertexId) const
{
    assert((*this).IsStartOrEndVertex(vertexId));

    if (commonImplPtr->vertexIds.front() == vertexId) {
        return commonImplPtr->vertexIds.back();
    }

    return commonImplPtr->vertexIds.front();
}

const GisVertex& GisObject::GetGisVertex(const size_t index) const
{
    return commonImplPtr->subsystemPtr->GetGisVertex(commonImplPtr->vertexIds.at(index));
}

const Vertex& GisObject::GetStartVertex() const
{
    return commonImplPtr->subsystemPtr->GetVertex(commonImplPtr->vertexIds.front());
}

const Vertex& GisObject::GetEndVertex() const
{
    return commonImplPtr->subsystemPtr->GetVertex(commonImplPtr->vertexIds.back());
}

Vertex GisObject::GetCenterPoint() const
{
    if (commonImplPtr->vertexIds.empty()) {
        return Vertex();
    }

    Vertex vertexSum;

    for(size_t i = 0; i < commonImplPtr->vertexIds.size(); i++) {
        vertexSum += (*this).GetVertex(i);
    }

    return vertexSum / double(commonImplPtr->vertexIds.size());
}

vector<Vertex> GisObject::GetVertices() const
{
    vector<Vertex> vertices;

    for(size_t i = 0; i < commonImplPtr->vertexIds.size(); i++) {
        vertices.push_back((*this).GetVertex(i));
    }

    return vertices;
}

size_t GisObject::NumberOfVertices() const
{
    return commonImplPtr->vertexIds.size();
}

const Rectangle& GisObject::GetMinRectangle() const
{
    return commonImplPtr->minRectangle;
}

void GisObject::UpdateMinRectangle() const
{
    assert((*this).NumberOfVertices() > 0);

    const double margin = 0.00001;

    Rectangle& minRectangle = commonImplPtr->minRectangle;

    minRectangle.minX = DBL_MAX;
    minRectangle.minY = DBL_MAX;
    minRectangle.maxX = -DBL_MAX;
    minRectangle.maxY = -DBL_MAX;

    for(size_t i = 0; i < (*this).NumberOfVertices(); i++) {
        const Vertex& vertex = (*this).GetVertex(i);

        minRectangle.minX = std::min(minRectangle.minX, vertex.x - margin);
        minRectangle.minY = std::min(minRectangle.minY, vertex.y - margin);
        minRectangle.maxX = std::max(minRectangle.maxX, vertex.x + margin);
        minRectangle.maxY = std::max(minRectangle.maxY, vertex.y + margin);
    }
}

const Material& GisObject::GetMaterial() const
{
    return commonImplPtr->subsystemPtr->GetMaterial(commonImplPtr->materialId);
}

size_t GisObject::CalculateNumberIntersections(const Vertex& p1, const Vertex& p2) const
{
    size_t numberIntersections = 0;

    for(size_t i = 0; i < (*this).NumberOfVertices() - 1; i++) {

        const Vertex& v1 = (*this).GetVertex(i);
        const Vertex& v2 = (*this).GetVertex(i+1);

        if (HorizontalLinesAreIntersection(p1, p2, v1, v2)) {
            numberIntersections++;
        }
    }

    return numberIntersections;
}

bool GisObject::IntersectsWith(const Rectangle& rect) const
{
    if ((*this).NumberOfVertices() == 0) {
        return false;
    }

    for(size_t i = 0; i < (*this).NumberOfVertices(); i++) {
        if (rect.Contains((*this).GetVertex(i))) {
            return true;
        }
    }

    const Vertex topLeft(rect.minX, rect.maxY);
    const Vertex bottomLeft(rect.minX, rect.minY);
    const Vertex topRight(rect.maxX,rect.maxY);
    const Vertex bottomRight(rect.maxX, rect.minY);

    for(size_t i = 0; i < (*this).NumberOfVertices() - 1; i++) {

        const Vertex& p1 = (*this).GetVertex(i);
        const Vertex& p2 = (*this).GetVertex(i+1);

        if (HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft) ||
            HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft) ||
            HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft) ||
            HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft)) {
            return true;
        }
    }

    return false;
}

void GisObject::InsertVertex(const Vertex& newVertex)
{
    GisSubsystem* subsystemPtr = commonImplPtr->subsystemPtr;
    vector<VertexIdType>& vertexIds = commonImplPtr->vertexIds;

    const double nearDistance = 0.01; //1cm
    const VertexIdType newVertexId = subsystemPtr->GetVertexId(newVertex);

    if (std::find(vertexIds.begin(), vertexIds.end(), newVertexId) !=
        vertexIds.end()) {
        return;
    }

    size_t vertexNumber;

    for(vertexNumber = 0; vertexNumber < vertexIds.size() - 1; vertexNumber++) {
        const Vertex& edge1 = (*this).GetVertex(vertexNumber);
        const Vertex& edge2 = (*this).GetVertex(vertexNumber+1);

        const double distanceToMiddlePoint =
            CalculatePointToLineDistance(newVertex, edge1, edge2);

        if (distanceToMiddlePoint < nearDistance) {
            break;
        }
    }

    assert(vertexNumber < vertexIds.size() - 1);

    const VertexIdType prevVertexId = vertexIds[vertexNumber];
    const VertexIdType nextVertexId = vertexIds[vertexNumber+1];

    (*this).InsertVertex(prevVertexId, newVertexId, nextVertexId);
}

void GisObject::InsertVertex(
    const VertexIdType& prevVertexId,
    const VertexIdType& newVertexId,
    const VertexIdType& nextVertexId)
{
    GisSubsystem* subsystemPtr = commonImplPtr->subsystemPtr;
    vector<VertexIdType>& vertexIds = commonImplPtr->vertexIds;

    if (std::find(vertexIds.begin(), vertexIds.end(), newVertexId) !=
        vertexIds.end()) {
        return;
    }

    const GisObjectType objectType = (*this).GetObjectType();
    const VariantIdType& variantId = commonImplPtr->variantId;

    subsystemPtr->DisconnectBidirectionalGisObject(
        prevVertexId, objectType, nextVertexId, variantId);

    subsystemPtr->ConnectBidirectionalGisObject(
        prevVertexId, objectType, newVertexId, variantId);

    subsystemPtr->ConnectBidirectionalGisObject(
        nextVertexId, objectType, newVertexId, variantId);

    vector<VertexIdType>::iterator iter = vertexIds.begin();

    for(size_t i = 0; i < vertexIds.size(); i++) {
        if (vertexIds[i] == nextVertexId) {
            assert(vertexIds[i-1] == prevVertexId);
            break;
        }
        assert(vertexIds[i] != newVertexId);

        iter++;
    }
    assert(vertexIds.back() != newVertexId);
    assert(iter != vertexIds.end());

    vertexIds.insert(iter, newVertexId);
}

const vector<VertexIdType>& GisObject::GetVertexIds() const
{
    return commonImplPtr->vertexIds;
}

void GisObject::SetEnabled(const bool enable)
{
    commonImplPtr->isEnabled = enable;

    if (commonImplPtr->outputGisTrace) {
        const TimeType currentTime =
            commonImplPtr->subsystemPtr->theSimulationEnginePtr->CurrentTime();

        TraceSubsystem& traceSubsystem =
            commonImplPtr->subsystemPtr->theSimulationEnginePtr->GetTraceSubsystem();

        StateTraceRecord stateTrace;
        ostringstream stateStream;

        const double value = 1.0;
        const string modelName = "Gis";
        const string instanceName = "";
        const string eventName = "State";
        const size_t threadPartitionIndex = 0;

        string stateName;

        if (enable) {
            stateName = "Enabled";
        } else {
            stateName = "Disabled";
        }

        const size_t stateSize =
            std::min(stateName.size(), sizeof(stateTrace.stateId) - 1);

        for(size_t i = 0; i < stateSize; i++) {
            stateTrace.stateId[i] = stateName[i];
        }
        stateTrace.stateId[stateSize] = '\0';

        if (traceSubsystem.BinaryOutputIsOn()) {
            traceSubsystem.OutputTraceInBinary(
                currentTime,
                commonImplPtr->objectId,
                modelName,
                instanceName,
                eventName,
                reinterpret_cast<const unsigned char* >(&stateTrace),
                sizeof(stateTrace),
                threadPartitionIndex);

        } else {

            ostringstream outStream;
            outStream << "V= " << value;

            traceSubsystem.OutputTrace(
                currentTime,
                commonImplPtr->objectId,
                modelName,
                instanceName,
                eventName,
                outStream.str(),
                threadPartitionIndex);
        }
    }
}

double GisObject::GetElevationFromGroundMeters() const
{
    return commonImplPtr->elevationFromGroundMeters;
}

bool GisObject::MasTraceIsOn() const
{
    return commonImplPtr->outputMasTrace;
}

bool GisObject::IsEnabled() const
{
    return commonImplPtr->isEnabled;
}

PointIdType Point::GetPointId() const
{
    return commonImplPtr->variantId;
}

EntranceIdType Entrance::GetEntranceId() const
{
    return commonImplPtr->variantId;
}

const Vertex& Entrance::GetVertex() const
{
    return (*this).GisObject::GetVertex(0);
}

VertexIdType Entrance::GetVertexId() const
{
    return (*this).GisObject::GetVertexId(0);
}

PoiIdType Poi::GetPoiId() const
{
    return commonImplPtr->variantId;
}

const Vertex& Poi::GetVertex() const
{
    return (*this).GisObject::GetVertex(0);
}

VertexIdType Poi::GetVertexId() const
{
    return (*this).GisObject::GetVertexId(0);
}

RoadIdType Poi::GetNearestEntranceRoadId(const Vertex& position) const
{
    if ((*this).IsAPartOfObject()) {
        GisSubsystem* subsystemPtr = commonImplPtr->subsystemPtr;

        if (parentPositionId.type == GIS_BUILDING) {
            return subsystemPtr->GetBuilding(parentPositionId.id).GetNearestEntranceRoadId(position);
        } else if (parentPositionId.type == GIS_PARK) {
            return subsystemPtr->GetPark(parentPositionId.id).GetNearestEntranceRoadId(position);
        }
    }

    const GisVertex& poiGisVertex =
        commonImplPtr->subsystemPtr->GetGisVertex((*this).GetVertexId());
    const vector<RoadIdType>& roadIds = poiGisVertex.GetConnectedObjectIds(GIS_ROAD);

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType& roadId = roadIds[i];

        if (commonImplPtr->subsystemPtr->GetRoad(roadId).IsParking()) {
            return roadId;
        }
    }

    cerr << "Error: No entrance road at " << commonImplPtr->objectName << endl;
    exit(1);

    return INVALID_VARIANT_ID;
}

PedestrianPath::PedestrianPath(
    GisSubsystem* initSubsystemPtr,
    const GisObjectIdType& initObjectId,
    const PedestrianPathIdType& initPathId)
    :
    GisObject(initSubsystemPtr, GIS_PEDESTRIAN_PATH, initObjectId, initPathId)
{}

Road::Road(
    GisSubsystem* initSubsystemPtr,
    const GisObjectIdType& initObjectId,
    const RoadIdType& initRoadId)
    :
    GisObject(initSubsystemPtr, GIS_ROAD, initObjectId, initRoadId),
    isRightHandTraffic(false),
    type(ROAD_UNCLASSIFIED),
    numberStartToEndLanes(1),
    numberEndToStartLanes(1),
    widthMeters(DEFAULT_ROAD_WIDTH_METERS),
    isBaseGroundLevel(true),
    humanCapacity(INT_MAX),
    capacity(1000.),
    speedLimitMetersPerSec(200.)
{
}

double Road::GetDirectionRadians() const
{
    const Vertex& startPosition = (*this).GetStartVertex();
    const Vertex& endPosition = (*this).GetEndVertex();

    return std::atan((endPosition.y - startPosition.y) /
                     (endPosition.x - startPosition.x));
}

double Road::GetArcDistanceMeters(const bool calculate3dArcDistance) const
{
    if (calculate3dArcDistance) {
        return CalculateArcDistance(vertices);
    }

    if (vertices.size() < 2) {
        return 0;
    }

    double totalDistance = 0;

    for(size_t i = 0; i < vertices.size() - 1; i++) {
        totalDistance += vertices[i].XYDistanceTo(vertices[i+1]);
    }

    return totalDistance;
}

double Road::GetArcDistanceMeters(
    const VertexIdType& vertexId1,
    const VertexIdType& vertexId2) const
{
    const vector<VertexIdType>& vertexIds = commonImplPtr->vertexIds;

    double distanceMeters = 0;
    bool found = false;

    for(size_t i = 0; i < vertexIds.size() - 1; i++) {
        const VertexIdType& vertexId = vertexIds[i];

        if (vertexId1 == vertexId || vertexId2 == vertexId) {
            if (!found) {
                found = true;
            } else {
                return distanceMeters;
            }
        }

        if (found) {
            const Vertex& v1 = (*this).GetVertex(i);
            const Vertex& v2 = (*this).GetVertex(i+1);

            distanceMeters += v1.DistanceTo(v2);
        }
    }

    if (vertexId1 == vertexIds.back() || vertexId2 == vertexIds.back()) {
        return distanceMeters;
    }


    cerr << "Error: Road " << (*this).GetRoadId() << " doesn't contain vertex " << vertexId1 << " and " << vertexId2 << endl;
    assert(false);

    return 0;
}

bool Road::Contains(const Vertex& point) const
{
    return PolygonContainsPoint(polygon, point);
}

double Road::DistanceTo(const Vertex& point) const
{
    if ((*this).IsParking()) {
        return (*this).GetVertex(0).DistanceTo(point);
    }

    double minDistance = DBL_MAX;

    assert(vertices.size() > 1);

    for(size_t i = 0; i < vertices.size() - 1; i++) {
        minDistance = std::min(minDistance, CalculatePointToLineDistance(point, vertices[i], vertices[+1]));
    }

    return minDistance;
}

RoadIdType Road::GetRoadId() const
{
    return commonImplPtr->variantId;
}

IntersectionIdType Road::GetOtherSideIntersectionId(const IntersectionIdType& intersectionId) const
{
    const IntersectionIdType startIntersectionId = (*this).GetStartIntersectionId();

    if (intersectionId != startIntersectionId) {
        return startIntersectionId;
    }

    return (*this).GetEndIntersectionId();
}

IntersectionIdType Road::GetStartIntersectionId() const
{
    const vector<IntersectionIdType> intersectionIds =
        (*this).GetGisVertex(0).GetConnectedObjectIds(GIS_INTERSECTION);

    assert(!intersectionIds.empty());

    return intersectionIds.front();
}

IntersectionIdType Road::GetEndIntersectionId() const
{
    if ((*this).IsParking()) {
        return (*this).GetStartIntersectionId();
    }

    const vector<IntersectionIdType> intersectionIds =
        (*this).GetGisVertex((*this).NumberOfVertices() - 1).GetConnectedObjectIds(GIS_INTERSECTION);

    assert(!intersectionIds.empty());

    return intersectionIds.front();
}

const Vertex& Road::GetNeighborVertex(
    const IntersectionIdType& intersectionId) const
{
    if ((*this).IsParking()) {
        return (*this).GetVertex(0);
    }

    if (intersectionId == (*this).GetStartIntersectionId()) {
        return vertices.at(1);
    } else {
        return vertices.at(vertices.size() - 2);
    }
}

Vertex Road::GetNearestPosition(const Vertex& position) const
{
    assert((*this).NumberOfVertices() > 0);

    double minDistance = DBL_MAX;
    Vertex nearestPosition;

    for(size_t i = 0; i < (*this).NumberOfVertices() - 1; i++) {

        const Vertex& edge1 = (*this).GetVertex(i);
        const Vertex& edge2 = (*this).GetVertex(i+1);

        const Vertex roadPosition = CalculatePointToLineNearestPosition(
            position.XYPoint(), edge1.XYPoint(), edge2.XYPoint());

        const double distance = roadPosition.XYDistanceTo(position);//use XY distance

        if (distance < minDistance) {
            minDistance = distance;
            nearestPosition = roadPosition;
        }
    }
    assert(minDistance != DBL_MAX);

    return nearestPosition;
}

IntersectionIdType Road::GetNearestIntersectionId(const Vertex& position) const
{
    const double distanceToStart = (*this).GetVertex(0).DistanceTo(position);
    const double distanceToEnd = (*this).GetVertex((*this).NumberOfVertices() - 1).DistanceTo(position);

    if (distanceToStart < distanceToEnd) {
        return (*this).GetStartIntersectionId();
    }

    return (*this).GetEndIntersectionId();
}

static inline
Vertex CalculateInterpolatedLineIntersection(
    const Vertex& lineEdge11,
    const Vertex& lineEdge12,
    const Vertex& lineEdge21,
    const Vertex& lineEdge22)
{
    const Vertex a = lineEdge12 - lineEdge11;
    const Vertex b = lineEdge21 - lineEdge22;
    const Vertex c = lineEdge11 - lineEdge21;

    const double denominator = a.y * b.x - a.x * b.y;

    if (denominator == 0 || denominator >= DBL_MAX) {
        return (lineEdge12 + lineEdge21) * 0.5;
    }

    const double reciprocal = 1. / denominator;
    const double na = (b.y * c.x - b.x * c.y) * reciprocal;

    return lineEdge11 + a * na;
}

static inline
void GetLinePolygon(
    const deque<Vertex>& vertices,
    const double widthMeters,
    vector<Vertex>& polygon)
{
    polygon.clear();

    if (vertices.size() < 2) {
        return;
    }

    typedef pair<Vertex, Vertex> LineType;

    const size_t numberPoints = vertices.size();
    const double halfWidth = widthMeters * 0.5;

    polygon.resize(numberPoints*2 + 1);
    vector<LineType> topLines(numberPoints - 1);
    vector<LineType> bottomLines(numberPoints - 1);

    const double expandLength = halfWidth;

    for(size_t i = 0; i < numberPoints - 1; i++) {
        Vertex p1 = vertices[i];
        Vertex p2 = vertices[i+1];

        const double distance = p1.DistanceTo(p2);

        if (distance == 0) {
            topLines[i].first = p1;
            topLines[i].second = p1;
            bottomLines[i].first = p1;
            bottomLines[i].second = p1;
            continue;
        }

        if (i == 0) {
            p1 = p1 + (p1 - p2) * (expandLength / distance);
        }
        if (i+1 == numberPoints - 1) {
            p2 = p2 + (p2 - p1) * (expandLength / distance);
        }

        const LineType aLine(p1, p2);
        const LineType normalVector(
            aLine.first,
            aLine.first + Vertex(aLine.second.y - aLine.first.y, aLine.first.x - aLine.second.x));

        Vertex offset;

        if (normalVector.second != normalVector.first) {
            offset =
                (normalVector.second - normalVector.first) *
                (halfWidth / normalVector.first.DistanceTo(normalVector.second));
        }

        topLines[i].first = aLine.first + offset;
        topLines[i].second = aLine.second + offset;

        bottomLines[i].first = aLine.first - offset;
        bottomLines[i].second = aLine.second - offset;
    }

    const size_t topLineStart = 0;
    const size_t bottomLineStart = numberPoints;

    polygon[topLineStart] = topLines.front().first;
    polygon[topLineStart + numberPoints - 1] = topLines.back().second;
    polygon[bottomLineStart] = bottomLines.back().second;
    polygon[bottomLineStart + numberPoints - 1] = bottomLines.front().first;

    const int numberLines = static_cast<int>(topLines.size());

    for(int i = 0; i < numberLines - 1; i++) {
        const LineType& topLine1 = topLines[i];
        const LineType& topLine2 = topLines[i+1];

        polygon[topLineStart + i + 1] = CalculateInterpolatedLineIntersection(
            topLine1.first, topLine1.second, topLine2.first, topLine2.second);

        const LineType& bottomLine1 = bottomLines[numberLines - i - 2];
        const LineType& bottomLine2 = bottomLines[numberLines - i - 1];

        polygon[bottomLineStart + i + 1] = CalculateInterpolatedLineIntersection(
            bottomLine1.first, bottomLine1.second, bottomLine2.first, bottomLine2.second);
    }

    polygon.back() = polygon.front();
}

void Road::UpdatePolygon()
{
    if (vertices.size() < 2) {
        return;
    }

    GetLinePolygon(vertices, widthMeters, polygon);
}

void Road::UpdateMinRectangle() const
{
    if (vertices.size() < 2) {
        commonImplPtr->minRectangle = GetPointsRect(vertices);
    } else {
        assert(polygon.size() > 2);
        commonImplPtr->minRectangle = GetPointsRect(polygon);
    }
}

bool Road::IntersectsWith(const Rectangle& rect) const
{
    return RectIsIntersectsWithPolygon(rect, polygon);
}

void Road::SetIntersectionMargin(
    const IntersectionIdType& intersectionId,
    const double marginLength)
{
    GisSubsystem* subsystemPtr = commonImplPtr->subsystemPtr;

    assert(!vertices.empty());

    const bool isReverse = ((*this).GetStartIntersectionId() != intersectionId);
    const Vertex& intersectionPos =
        subsystemPtr->GetIntersection(intersectionId).GetVertex();

    ReverseAccess<Vertex> verticesAccess(vertices, isReverse);

    const double distanceToIntersection =
        verticesAccess.front().DistanceTo(intersectionPos);

    if (marginLength > distanceToIntersection) {

        double marginLengthFromFront = marginLength - distanceToIntersection;

        while (true) {
            assert(verticesAccess.size() > 1);

            const Vertex& p1 = verticesAccess[0];
            const Vertex& p2 = verticesAccess[1];
            const double length = p1.DistanceTo(p2);

            if (marginLengthFromFront <= length) {
                verticesAccess.front() = (p2-p1)*(marginLengthFromFront/length) + p1;
                break;
            }
            marginLengthFromFront -= length;
            verticesAccess.pop_front();
        }

    } else {
        verticesAccess.front() =
            (intersectionPos - verticesAccess.front())*
            ((distanceToIntersection - marginLength)/distanceToIntersection) +
            verticesAccess.front();
    }
}

size_t Road::GetRandomOutgoingLaneNumber(
    const VertexIdType& startVertexId,
    HighQualityRandomNumberGenerator& aRandomNumberGenerator) const
{
    if (startVertexId == (*this).GetStartVertexId()) {

        return aRandomNumberGenerator.GenerateRandomInt(
            0, static_cast<int32_t>(numberStartToEndLanes - 1));
    }

    return numberStartToEndLanes +
        aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(numberEndToStartLanes - 1));
}

size_t Road::GetNearestOutgoingLaneNumber(
    const VertexIdType& startVertexId,
    const Vertex& position) const
{
    vector<size_t> laneNumberCandidates;

    if (startVertexId == (*this).GetStartVertexId()) {
        for(size_t i = 0; i < numberStartToEndLanes; i++) {
            laneNumberCandidates.push_back(i);
        }
    } else {
        for(size_t i = numberStartToEndLanes; i < numberStartToEndLanes + numberEndToStartLanes; i++) {
            laneNumberCandidates.push_back(i);
        }
    }

    size_t nearestLaneNumber = laneNumberCandidates.front();
    double nearestDistance = DBL_MAX;

    for(size_t i = 0; i < laneNumberCandidates.size(); i++) {
        const size_t laneNumber = laneNumberCandidates[i];
        deque<Vertex> laneVertices;

        (*this).GetLaneVertices(laneNumber, true/*waypointFromAdditionalStartPosition*/, position, laneVertices);

        const double distance = laneVertices.front().DistanceTo(position);

        if (distance < nearestDistance) {
            nearestDistance = distance;
            nearestLaneNumber = laneNumber;
        }
    }

    return nearestLaneNumber;
}

size_t Road::GetOutsideOutgoingLaneNumber(const VertexIdType& startVertexId) const
{
    if (startVertexId == (*this).GetStartVertexId()) {
        return numberStartToEndLanes - 1;
    }

    return numberStartToEndLanes + numberEndToStartLanes - 1;
}

bool Road::IsParking() const
{
    return (commonImplPtr->vertexIds.size() == 1);
}

size_t Road::GetParkingLaneNumber() const
{
    assert((*this).IsParking());
    return 0;
}

RoadDirectionType Road::GetRoadDirection(const size_t laneNumber) const
{
    if (laneNumber < numberStartToEndLanes) {
        return ROAD_DIRECTION_UP;
    }
    return ROAD_DIRECTION_DOWN;
}

bool Road::HasPassingLane(const RoadDirectionType& directionType, const size_t laneNumer) const
{
    if (directionType == ROAD_DIRECTION_UP) {
        if (isRightHandTraffic) {
            return (laneNumer > 0);
        } else {
            return (laneNumer + 1 < numberStartToEndLanes);
        }
    } else {
        if (isRightHandTraffic) {
            return (laneNumer + 1 < numberStartToEndLanes + numberEndToStartLanes);
        } else {
            return (laneNumer > numberStartToEndLanes);
        }
    }
}

bool Road::HasNonPassingLane(const RoadDirectionType& directionType, const size_t laneNumer) const
{
    if (directionType == ROAD_DIRECTION_UP) {
        if (isRightHandTraffic) {
            return (laneNumer + 1 < numberStartToEndLanes);
        } else {
            return (laneNumer > 0);
        }
    } else {
        if (isRightHandTraffic) {
            return (laneNumer > numberStartToEndLanes);
        } else {
            return (laneNumer + 1 < numberStartToEndLanes + numberEndToStartLanes);
        }
    }
}

size_t Road::GetPassingLaneNumber(
    const RoadDirectionType& directionType,
    const size_t laneNumer) const
{
    assert((*this).HasPassingLane(directionType, laneNumer));

    if (directionType == ROAD_DIRECTION_UP) {
        if (isRightHandTraffic) {
            return (laneNumer - 1);
        } else {
            return (laneNumer + 1);
        }
    } else {
        if (isRightHandTraffic) {
            return (laneNumer + 1);
        } else {
            return (laneNumer - 1);
        }
    }
}

size_t Road::GetNonPassingLaneNumber(
    const RoadDirectionType& directionType,
    const size_t laneNumer) const
{
    assert((*this).HasNonPassingLane(directionType, laneNumer));

    if (directionType == ROAD_DIRECTION_UP) {
        if (isRightHandTraffic) {
            return (laneNumer + 1);
        } else {
            return (laneNumer - 1);
        }
    } else {
        if (isRightHandTraffic) {
            return (laneNumer - 1);
        } else {
            return (laneNumer + 1);
        }
    }
}

bool Road::CanApproach(const size_t laneNumber, const RoadIdType& roadId) const
{
    const map<RoadIdType, vector<pair<RoadTurnDirectionType, size_t> > >& laneConnection =
        laneConnections.at(laneNumber);

    return (laneConnection.find(roadId) != laneConnection.end());
}

size_t Road::GetNextLaneNumber(const size_t laneNumber, const RoadIdType& roadId) const
{
    typedef map<RoadIdType, vector<pair<RoadTurnDirectionType, size_t> > >::const_iterator IterType;

    const map<RoadIdType, vector<pair<RoadTurnDirectionType, size_t> > >& laneConnection =
        laneConnections.at(laneNumber);

    IterType iter = laneConnection.find(roadId);

    assert(iter != laneConnection.end());

    const vector<pair<RoadTurnDirectionType, size_t> >& laneNumbers = (*iter).second;

    assert(!laneNumbers.empty());

    return laneNumbers.front().second;
}

size_t Road::GetApproachLaneNumber(const RoadIdType& roadId) const
{
    for(size_t i = 0; i < laneConnections.size(); i++) {
        const map<RoadIdType, vector<pair<RoadTurnDirectionType, size_t> > >& laneConnection = laneConnections[i];

        if (laneConnection.find(roadId) != laneConnection.end()) {
            return i;
        }
    }

    assert(false);

    return BAD_SIZE_T;
}

size_t Road::GetNeighborLaneNumberToApproach(
    const size_t laneNumber,
    const RoadIdType& roadId) const
{
    const size_t approachLaneNumber = (*this).GetApproachLaneNumber(roadId);

    if (approachLaneNumber < laneNumber) {
        return laneNumber - 1;
    } else if (approachLaneNumber > laneNumber) {
        return laneNumber + 1;
    }

    return laneNumber;
}

void Road::GetLaneVertices(
    const size_t laneNumber,
    const bool waypointFromAdditionalStartPosition,
    const Vertex& startPosition,
    deque<Vertex>& laneVertices) const
{
    const bool isReverse = (laneNumber >= numberStartToEndLanes);

    double offset;

    if ((*this).IsExtraPath()) {

        offset = 0.;

    } else {

        size_t numberOffsetLanes;
        if (isReverse) {
            numberOffsetLanes = laneNumber - numberStartToEndLanes;
        } else {
            numberOffsetLanes = numberStartToEndLanes - laneNumber - 1;
        }

        const double laneWidthMeters = (*this).GetLaneWidthMeters();

        if (numberStartToEndLanes == 0 || numberEndToStartLanes == 0) {
            offset = (numberOffsetLanes + 0.5) * laneWidthMeters - widthMeters*0.5;
        } else {
            offset = (numberOffsetLanes + 0.5) * laneWidthMeters;
        }

        if (!isRightHandTraffic) {
            offset *= -1;
        }
    }

    // calculate offset points from center line
    (*this).GetOffsetWaypoints(offset, isReverse, waypointFromAdditionalStartPosition, startPosition, laneVertices);
}

pair<Vertex, Vertex> Road::GetSideLineToIntersection(
    const IntersectionIdType& intersectionId,
    const double averageWidthMeters) const
{
    const bool isReverse = ((*this).GetStartIntersectionId() != intersectionId);

    pair<Vertex, Vertex> sideLine;
    deque<Vertex> laneVertices;

    (*this).GetOffsetWaypoints(averageWidthMeters/2., isReverse, false/*waypointFromAdditionalStartPosition*/, Vertex(), laneVertices);


    if (laneVertices.size() >= 2) {
        sideLine.first = laneVertices[0];
        sideLine.second = laneVertices[1];
    }

    return sideLine;
}

Vertex Road::GetInternalPoint(
    const IntersectionIdType& intersectionId,
    const double internalLengthMeters) const
{
    GisSubsystem* subsystemPtr = commonImplPtr->subsystemPtr;

    const bool isReverse = ((*this).GetStartIntersectionId() != intersectionId);

    ConstReverseAccess<Vertex> verticesAccess(vertices, isReverse);

    const Vertex& intersectionPos =
        subsystemPtr->GetIntersection(intersectionId).GetVertex();

    const double distanceToIntersection =
        verticesAccess.front().DistanceTo(intersectionPos);

    if (internalLengthMeters > distanceToIntersection) {

        double marginLengthFromFront = internalLengthMeters - distanceToIntersection;

        for(size_t i = 0; i < verticesAccess.size() - 1; i++) {
            const Vertex& p1 = verticesAccess[i];
            const Vertex& p2 = verticesAccess[i+1];
            const double length = p1.DistanceTo(p2);

            if (marginLengthFromFront <= length) {
                return (p2-p1)*(marginLengthFromFront/length) + p1;
            }
            marginLengthFromFront -= length;
        }

        return verticesAccess.back();
    } else {
        return (intersectionPos - verticesAccess.front())*
            ((distanceToIntersection - internalLengthMeters)/distanceToIntersection) +
            verticesAccess.front();
    }
}

void Road::GetPedestrianVertices(
    const VertexIdType& lastVertexId,
    const VertexIdType& startVertexId,
    const bool walkLeftSide,
    const double maxOffset,
    const Vertex& startPosition,
    deque<Vertex>& waypoints) const
{
    waypoints.clear();

    if (lastVertexId == startVertexId) {
        return;
    }

    const bool isReverse = (startVertexId != (*this).GetStartVertexId());

    double offset;

    if ((*this).IsExtraPath()) {

        offset = 0.;

    } else {

        offset = std::min((*this).GetRoadWidthMeters()/2, maxOffset);

        if (walkLeftSide) {
            offset = -offset;
        }
    }

    (*this).GetOffsetWaypoints(offset, isReverse, true/*waypointFromAdditionalStartPosition*/, startPosition, waypoints);
}

void Road::GetPedestrianVertices(
    const VertexIdType& lastVertexId,
    const VertexIdType& startVertexId,
    const bool walkLeftSide,
    const Vertex& startPosition,
    deque<Vertex>& waypoints) const
{
    waypoints.clear();

    if (lastVertexId == startVertexId) {
        return;
    }

    const bool isReverse = (startVertexId != (*this).GetStartVertexId());

    double offset;

    if ((*this).IsExtraPath()) {

        offset = 0.;

    } else {

        ConstReverseAccess<Vertex> verticesAccess(vertices, isReverse);

        const Vertex& p1 = verticesAccess[0];
        const Vertex& p2 = verticesAccess[1];

        assert(p1 != p2);

        const Vertex lastVeretx = p1.NormalVector(
            commonImplPtr->subsystemPtr->GetVertex(lastVertexId));

        if (CalculateRadiansBetweenVector(p2, p1, lastVeretx) < PI/2) {
            offset = numberStartToEndLanes*(*this).GetLaneWidthMeters();
        } else {
            offset = numberEndToStartLanes*(*this).GetLaneWidthMeters();
        }

        if (walkLeftSide) {
            offset = -offset;
        }

    }

    //TBD// if (length < offset) {
    //TBD//     waypoints.push_back(verticesAccess.front());
    //TBD//     waypoints.push_back(verticesAccess.back());
    //TBD//     return;
    //TBD// }

    (*this).GetOffsetWaypoints(offset, isReverse, true/*waypointFromAdditionalStartPosition*/, startPosition, waypoints);
}

void Road::GetOffsetWaypoints(
    const double offset,
    const bool isReverse,
    const bool waypointFromAdditionalStartPosition,
    const Vertex& startPosition,
    deque<Vertex>& waypoints) const
{
    waypoints.clear();

    if ((*this).IsParking()) {
        waypoints.push_back((*this).GetVertex(0));
        return;
    }

    ConstReverseAccess<Vertex> verticesAccess(vertices, isReverse);

    vector<Vertex> srcWaypoints;

    for(size_t i = 0; i < verticesAccess.size(); i++) {
        srcWaypoints.push_back(verticesAccess[i]);
    }

    commonImplPtr->subsystemPtr->GetOffsetWaypoints(
        offset,
        srcWaypoints,
        waypointFromAdditionalStartPosition,
        startPosition,
        isBaseGroundLevel,
        waypoints);
}

void GisSubsystem::GetOffsetWaypoints(
    const double offset,
    const vector<Vertex>& srcWaypoints,
    const bool waypointFromAdditionalStartPosition,
    const Vertex& startPosition,
    const bool isBaseGroundLevel,
    deque<Vertex>& waypoints) const
{
    if (offset == 0.) {

        for(size_t i = 0; i < srcWaypoints.size(); i++) {
            waypoints.push_back(srcWaypoints[i]);
        }

    } else {

        waypoints.clear();

        vector<pair<Vertex, Vertex> > lines(srcWaypoints.size() - 1);

        for(size_t i = 0; i < srcWaypoints.size() - 1; i++) {
            const Vertex& p1 = srcWaypoints[i];
            const Vertex& p2 = srcWaypoints[i+1];

            if (p1 == p2) {
                cerr << "Error: Waypoints ";

                for(size_t j = 0; j < srcWaypoints.size(); j++) {
                    cerr << " v" << j << "(" << srcWaypoints[j].x << "," << srcWaypoints[j].y << "," << srcWaypoints[j].z << ")";
                }

                cerr << " contains continuous duplicated vertex (" << p1.x << "," << p1.y << "," << p1.z << ")" << endl;

                exit(1);
            }

            assert(p1 != p2);

            const Vertex normal = p1.NormalVector(p2);
            const double xyDistance = p1.XYDistanceTo(p2);

            Vertex offsetPoint;

            if (xyDistance > 0) {
                offsetPoint = normal * (offset/xyDistance);
            }

            lines[i].first = p1 + offsetPoint;
            lines[i].second = p2 + offsetPoint;
        }

        waypoints.push_back(lines.front().first);
        for(size_t i = 0; i < lines.size() - 1; i++) {
            const pair<Vertex, Vertex>& line1 = lines[i];
            const pair<Vertex, Vertex>& line2 = lines[i+1];

            if (HorizontalLinesAreIntersection(
                    line1.first, line1.second,
                    line2.first, line2.second)) {

                waypoints.push_back(
                    CalculateIntersectionPositionBetweenLine(
                        line1.first, line1.second,
                        line2.first, line2.second));
            } else {

                waypoints.push_back(line1.second);

                const int numberDivisions = static_cast<int>(
                    std::ceil(line1.second.DistanceTo(line2.first) / (offset*0.1)));

                if (numberDivisions > 0) {
                    const double absOffset = fabs(offset);
                    const double divisorRate = 2./numberDivisions;

                    for(int j = 1; j < numberDivisions; j++) {
                        const double ratio1 = 2. - j*divisorRate;
                        const double ratio2 = 2. - ratio1;

                        const Vertex& p1 = srcWaypoints[i+1];
                        const Vertex p2 = (line1.second*ratio1 + line2.first*ratio2)*0.5;
                        const double distance = p1.DistanceTo(p2);

                        const Vertex dividiedVertex = p1 + (p2 - p1)*(absOffset/distance);

                        waypoints.push_back(dividiedVertex);
                    }
                }

                waypoints.push_back(line2.first);
            }
        }

        waypoints.push_back(lines.back().second);
    }

    if (waypointFromAdditionalStartPosition) {
        waypoints.push_front(startPosition);
    }

    if (isBaseGroundLevel) {
        (*this).CompleteGroundElevation(waypoints);
    }

    if (waypointFromAdditionalStartPosition) {
        waypoints.pop_front();
    }
}

void GisSubsystem::CompleteGroundElevation(deque<Vertex>& waypoints) const
{
    if (waypoints.empty()) {
        return;
    }

    const deque<Vertex> srcWaypoints = waypoints;

    waypoints.clear();

    for(size_t i = 0; i < srcWaypoints.size() - 1; i++) {
        deque<Vertex> completedVertices;

        groundLayerPtr->GetGroundElevationCompletedVertices(srcWaypoints[i], srcWaypoints[i+1], completedVertices);

        if (i == 0 && !completedVertices.empty()) {
            waypoints.push_back(completedVertices.front());
        }

        for(size_t j = 1; j < completedVertices.size(); j++) {
            waypoints.push_back(completedVertices[j]);
        }
    }
}

vector<size_t> static inline
MakeLaneTable(
    const vector<size_t>& outLaneNumbers,
    const vector<size_t>& inLaneNumbers)
{
    vector<size_t> laneNumbers(outLaneNumbers.size());

    if (!inLaneNumbers.empty() && !outLaneNumbers.empty()) {
        const double laneCompressionRate = double(inLaneNumbers.size() + 1) / outLaneNumbers.size();

        for(size_t i = 0; i < outLaneNumbers.size(); i++) {
            const size_t inLaneNumber =
                std::min(static_cast<size_t>(i*laneCompressionRate), inLaneNumbers.size() - 1);
            laneNumbers[i] = inLaneNumbers[inLaneNumber];
        }
    }

    return laneNumbers;
}

void Road::MakeTurnaroundLaneConnection()
{
    vector<size_t> laneNumbers1;
    vector<size_t> laneNumbers2;

    for(size_t i = 0; i < numberStartToEndLanes; i++) {
        laneNumbers1.push_back(i);
    }
    for(size_t i = 0; i < numberEndToStartLanes; i++) {
        laneNumbers2.push_back(numberStartToEndLanes + numberEndToStartLanes - i - 1);
    }

    const vector<size_t> laneNumbers12 = MakeLaneTable(laneNumbers1, laneNumbers2);
    const vector<size_t> laneNumbers21 = MakeLaneTable(laneNumbers2, laneNumbers1);

    assert(laneNumbers12.size() == laneNumbers1.size());
    assert(laneNumbers21.size() == laneNumbers2.size());

    const RoadIdType roadId = (*this).GetRoadId();

    for(size_t i = 0; i < laneNumbers1.size(); i++) {
        laneConnections[laneNumbers1[i]][roadId].
            push_back(make_pair(ROAD_TURN_BACK, laneNumbers12[i]));
    }
    for(size_t i = 0; i < laneNumbers2.size(); i++) {
        laneConnections[laneNumbers2[i]][roadId].
            push_back(make_pair(ROAD_TURN_BACK, laneNumbers21[i]));
    }
}

void Road::MakeLaneConnection(
    const Road& otherRoad,
    const VertexIdType& vertexId,
    const RoadTurnDirectionType& direction)
{
    const RoadIdType otherRoadId = otherRoad.GetRoadId();

    if (direction == ROAD_TURN_RIGHT) {

        if ((*this).HasOutgoingLane(vertexId) &&
            otherRoad.HasIncomingLane(vertexId)) {

            laneConnections[(*this).GetOutgoingRightLaneNumber(vertexId)][otherRoadId].
                push_back(make_pair(direction, otherRoad.GetIncomingRightLaneNumber(vertexId)));
        }

    } else if (direction == ROAD_TURN_LEFT) {

        if ((*this).HasOutgoingLane(vertexId) &&
            otherRoad.HasIncomingLane(vertexId)) {

            laneConnections[(*this).GetOutgoingLeftLaneNumber(vertexId)][otherRoadId].
                push_back(make_pair(direction, otherRoad.GetIncomingLeftLaneNumber(vertexId)));
        }

    } else {
        const vector<size_t> outgoingLaneNumbers = (*this).GetOutgoingLaneNumbers(vertexId);
        const vector<size_t> incomingLaneNumbers = otherRoad.GetIncomingLaneNumbers(vertexId);
        const vector<size_t> laneNumbers =
            MakeLaneTable(outgoingLaneNumbers, incomingLaneNumbers);

        assert(laneNumbers.size() == outgoingLaneNumbers.size());

        for(size_t i = 0; i < outgoingLaneNumbers.size(); i++) {
            laneConnections[outgoingLaneNumbers[i]][otherRoadId].
                push_back(make_pair(direction, laneNumbers[i]));
        }
    }
}

size_t Road::GetOutgoingRightLaneNumber(const VertexIdType& vertexId) const
{
    if (vertexId == (*this).GetStartVertexId()) {
        return numberStartToEndLanes;
    } else {
        return numberStartToEndLanes - 1;
    }
}

size_t Road::GetIncomingRightLaneNumber(const VertexIdType& vertexId) const
{
    if (vertexId == (*this).GetStartVertexId()) {
        return numberStartToEndLanes - 1;
    } else {
        return numberStartToEndLanes;
    }
}

size_t Road::GetOutgoingLeftLaneNumber(const VertexIdType& vertexId) const
{
    if (vertexId == (*this).GetStartVertexId()) {
        return numberStartToEndLanes + numberEndToStartLanes - 1;
    } else {
        return 0;
    }
}

size_t Road::GetIncomingLeftLaneNumber(const VertexIdType& vertexId) const
{
    if (vertexId == (*this).GetStartVertexId()) {
        return 0;
    } else {
        return numberStartToEndLanes + numberEndToStartLanes - 1;
    }
}

vector<size_t> Road::GetOutgoingLaneNumbers(const VertexIdType& vertexId) const
{
    vector<size_t> laneNumbers;

    if (vertexId == (*this).GetStartVertexId()) {
        for(size_t i = 0; i < numberEndToStartLanes; i++) {
            laneNumbers.push_back(numberStartToEndLanes + numberEndToStartLanes - i - 1);
        }
    } else {
        for(size_t i = 0; i < numberStartToEndLanes; i++) {
            laneNumbers.push_back(i);
        }
    }

    return laneNumbers;
}

vector<size_t> Road::GetIncomingLaneNumbers(const VertexIdType& vertexId) const
{
    vector<size_t> laneNumbers;

    if (vertexId == (*this).GetStartVertexId()) {
        for(size_t i = 0; i < numberStartToEndLanes; i++) {
            laneNumbers.push_back(i);
        }
    } else {
        for(size_t i = 0; i < numberEndToStartLanes; i++) {
            laneNumbers.push_back(numberStartToEndLanes + numberEndToStartLanes - i - 1);
        }
    }

    return laneNumbers;
}

bool Road::HasOutgoingLane(const VertexIdType& vertexId) const
{
    if (vertexId == (*this).GetStartVertexId()) {
        return (numberEndToStartLanes > 0);
    } else {
        return (numberStartToEndLanes > 0);
    }
}

bool Road::HasIncomingLane(const VertexIdType& vertexId) const
{
    if (vertexId == (*this).GetStartVertexId()) {
        return (numberStartToEndLanes > 0);
    } else {
        return (numberEndToStartLanes > 0);
    }
}

void TrafficLight::SyncTrafficLight(const TimeType& currentTime)
{
    while (!trafficLightIdsPerTime.empty() &&
           currentTime > trafficLightIdsPerTime.front().first) {
        trafficLightIdsPerTime.pop_front();
    }
}

TrafficLightType TrafficLight::GetTrafficLight(const TimeType& currentTime) const
{
    if (!trafficLightIdsPerTime.empty()) {
        return trafficLightIdsPerTime.front().second;
    }

    const TimeType cycleDuration =
        greenDuration + yellowDuration + redDuration;

    const TimeType timeOffset =
        (currentTime + startOffset) % cycleDuration;

    if (timeOffset < greenDuration) {
        return TRAFFIC_LIGHT_GREEN;
    } else if (timeOffset < greenDuration + yellowDuration) {
        return TRAFFIC_LIGHT_YELLOW;
    }

    return TRAFFIC_LIGHT_RED;
}

TrafficLightType Intersection::GetTrafficLight(
    const TimeType& currentTime,
    const RoadIdType& incomingRoadId) const
{
    if (!(*this).IsEnabled()) {
        return TRAFFIC_LIGHT_GREEN;
    }

    typedef map<RoadIdType, TrafficLightIdType>::const_iterator IterType;

    IterType iter = trafficLightIds.find(incomingRoadId);

    if (iter == trafficLightIds.end()) {
        return TRAFFIC_LIGHT_GREEN;
    }

    const TrafficLight& trafficLight = roadLayerPtr->GetTrafficLight((*iter).second);
    return trafficLight.GetTrafficLight(currentTime);
}

TimeType Intersection::CalculateCrossingStartTime(
    const TimeType& currentTime,
    const RoadIdType& roadId,
    const bool leftCrossing,
    const TimeType minWalkDuration) const
{
    if (!(*this).IsEnabled()) {
        return currentTime;
    }

    typedef map<RoadIdType, TrafficLightIdType>::const_iterator IterType;

    IterType iter = trafficLightIds.find(roadId);

    if (iter == trafficLightIds.end()) {
        return currentTime;
    }

    const TrafficLight& trafficLight = roadLayerPtr->GetTrafficLight((*iter).second);

    const TimeType cycleDuration =
        trafficLight.greenDuration + trafficLight.yellowDuration + trafficLight.redDuration;

    const TimeType timeOffset =
        (currentTime + trafficLight.startOffset) % cycleDuration;

    if (timeOffset < trafficLight.greenDuration + trafficLight.yellowDuration) {
        return currentTime + (trafficLight.greenDuration + trafficLight.yellowDuration - timeOffset);
    }

    if (timeOffset + minWalkDuration > cycleDuration) {
        return currentTime + (cycleDuration - timeOffset) + trafficLight.greenDuration + trafficLight.yellowDuration;
    }
    return currentTime;
}

IntersectionIdType Intersection::GetIntersectionId() const
{
    return commonImplPtr->variantId;
}

vector<RoadIdType> Intersection::GetConnectedRoadIds() const
{
    return (*this).GetGisVertex().GetConnectedObjectIds(GIS_ROAD);
}

RoadIdType Intersection::GetRoadIdTo(const IntersectionIdType& endIntersectionId) const
{
    const vector<RoadIdType> roadIds = (*this).GetConnectedRoadIds();
    const IntersectionIdType intersectionId = (*this).GetIntersectionId();

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType& roadId = roadIds[i];
        const Road& road = commonImplPtr->subsystemPtr->GetRoad(roadId);

        if (road.GetOtherSideIntersectionId(intersectionId) == endIntersectionId) {
            return roadId;
        }
    }

    assert(false && "No road connection");
    return INVALID_VARIANT_ID;
}

const Vertex& Intersection::GetVertex() const
{
    return (*this).GisObject::GetVertex(0);
}

VertexIdType Intersection::GetVertexId() const
{
    return (*this).GisObject::GetVertexId(0);
}

const GisVertex& Intersection::GetGisVertex() const
{
    return (*this).GisObject::GetGisVertex(0);
}

bool Intersection::IsTerminated() const
{
    return ((*this).GetConnections().size() == 1);
}

size_t Intersection::GetRoadsideNumber(const RoadIdType& roadId) const
{
    typedef map<RoadIdType, size_t>::const_iterator IterType;

    IterType iter = roadsideNumberPerRoadId.find(roadId);

    if (iter == roadsideNumberPerRoadId.end()) {
        cerr << "Error: invalid road id " << roadId << " for " << (*this).GetIntersectionId() << endl << "valid road id ";

        for(IterType validIter = roadsideNumberPerRoadId.begin();
            validIter != roadsideNumberPerRoadId.end(); validIter++) {
            cerr << "," << (*validIter).first;
        }
        exit(1);
    }

    return (*iter).second;
}

bool Intersection::ContainsRoadside(const RoadIdType& roadId) const
{
    typedef map<RoadIdType, size_t>::const_iterator IterType;

    return (roadsideNumberPerRoadId.find(roadId) != roadsideNumberPerRoadId.end());
}

bool Intersection::CanPassRoad(
    const RoadIdType& incomingRoadId,
    const RoadIdType& outgoingRoadId) const
{
    if (incomingRoadId == INVALID_VARIANT_ID) {
        return true;
    }

    typedef map<pair<RoadIdType, RoadIdType>, RoadTurnType>::const_iterator IterType;

    IterType iter = roadTurnTypes.find(make_pair(incomingRoadId, outgoingRoadId));

    if (iter == roadTurnTypes.end()) {
        return false;
    }

    return (*iter).second.hasLanes;
}

const RoadTurnType& Intersection::GetRoadTurnType(
    const RoadIdType& incomingRoadId,
    const RoadIdType& outgoingRoadId) const
{
    typedef map<pair<RoadIdType, RoadIdType>, RoadTurnType>::const_iterator IterType;

    IterType iter = roadTurnTypes.find(make_pair(incomingRoadId, outgoingRoadId));

    if (iter == roadTurnTypes.end()) {
        cerr << "No road connection from Road" << incomingRoadId << " to Road" << outgoingRoadId << endl;
        exit(1);
    }

    return (*iter).second;
}

bool Intersection::PedestrianCanPass() const
{
    const vector<RoadIdType> roadIds = (*this).GetConnectedRoadIds();

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType& roadId = roadIds[i];
        const Road& road = commonImplPtr->subsystemPtr->GetRoad(roadId);

        if (!(road.IsPedestrianRoad() || road.IsExtraPath())) {
            return false;
        }
    }

    return true;
}

const map<GisObjectType, vector<VertexConnection> >& Intersection::GetConnections() const
{
    return (*this).GetGisVertex().connections;
}

VertexIdType BusStop::GetLineVertexId(
    const BusLineIdType& lineId,
    const RouteIdType& routeId) const
{
    return (*this).GetVertexId();
}

bool BusStop::IsBusStopVertex(const VertexIdType& vertexId) const
{
    return ((*this).GetVertexId() == vertexId);
}

VertexIdType BusStop::GetVertexId() const
{
    return (*this).GisObject::GetVertexId(0);
}

VertexIdType BusStop::GetNearestEntranceVertexId(const Vertex& position) const
{
    typedef set<EntranceIdType>::const_iterator IterType;

    double minDisntace = DBL_MAX;

    assert(!entranceIds.empty());

    VertexIdType entranceVertexId = INVALID_VERTEX_ID;

    for(IterType iter = entranceIds.begin(); iter != entranceIds.end(); iter++) {
        const Entrance& entrance = commonImplPtr->subsystemPtr->GetEntrance(*iter);
        const VertexIdType vertexId = entrance.GetVertexId();
        const Vertex& entrancePosition = commonImplPtr->subsystemPtr->GetVertex(vertexId);

        const double distance = entrancePosition.DistanceTo(position);

        if (distance < minDisntace) {
            entranceVertexId = vertexId;
            minDisntace = distance;
        }
    }

    return entranceVertexId;
}

void BusStop::UpdateMinRectangle() const
{
    const double margin = 0.00001;

    commonImplPtr->minRectangle = Rectangle(position, margin);
}

bool BusStop::IntersectsWith(const Rectangle& rect) const
{
    return rect.Contains(position);
}

void BusStop::CompleteEntrances(const size_t defaultNumberEntrances)
{
    if (!entranceIds.empty()) {
        return;
    }

    entranceIds.insert(commonImplPtr->subsystemPtr->CreateEntrance((*this).GetVertex()));
}

Vertex BusStop::AddEntrance(const EntranceIdType& entranceId, const Vertex& point)
{
    entranceIds.insert(entranceId);

    return Vertex(
        point.x,
        point.y,
        (*this).GetVertex().z);
}

RailRoadIdType RailRoad::GetRailRoadId() const
{
    return commonImplPtr->variantId;
}

void RailRoad::UpdateMinRectangle() const
{
    Rectangle minRectangle;

    minRectangle.minX = DBL_MAX;
    minRectangle.minY = DBL_MAX;
    minRectangle.maxX = -DBL_MAX;
    minRectangle.maxY = -DBL_MAX;

    assert(!vertices.empty());

    for(size_t i = 0; i < vertices.size(); i++) {
        const Vertex& vertex = railRoadLayerPtr->railVertices.at(vertices[i].first).vertex;

        minRectangle.minX = std::min(minRectangle.minX, vertex.x);
        minRectangle.minY = std::min(minRectangle.minY, vertex.y);
        minRectangle.maxX = std::max(minRectangle.maxX, vertex.x);
        minRectangle.maxY = std::max(minRectangle.maxY, vertex.y);
    }

    commonImplPtr->minRectangle = minRectangle;
}

bool RailRoad::IntersectsWith(const Rectangle& rect) const
{
    for(size_t i = 0; i < vertices.size(); i++) {
        const Vertex& vertex = railRoadLayerPtr->railVertices.at(vertices[i].first).vertex;

        if (rect.Contains(vertex)) {
            return true;
        }
    }

    const Vertex topLeft(rect.minX, rect.maxY);
    const Vertex bottomLeft(rect.minX, rect.minY);
    const Vertex topRight(rect.maxX,rect.maxY);
    const Vertex bottomRight(rect.maxX, rect.minY);

    for(size_t i = 0; i < vertices.size() - 1; i++) {
        const Vertex& p1 = railRoadLayerPtr->railVertices.at(vertices[i].first).vertex;
        const Vertex& p2 = railRoadLayerPtr->railVertices.at(vertices[i+1].first).vertex;

        if (HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft) ||
            HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft) ||
            HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft) ||
            HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft)) {
            return true;
        }
    }

    return false;
}

void RailRoad::SetEnabled(const bool enable)
{
    GisObject::SetEnabled(enable);

    typedef set<RailRoadLineIdType>::const_iterator IterType;

    for(IterType iter = railRoadLineIds.begin(); iter != railRoadLineIds.end(); iter++) {
        railRoadLayerPtr->UpdateRailRoadLineAvailability(*iter);
    }
}

RailRoadStationIdType RailRoadStation::GetStationId() const
{
    return commonImplPtr->variantId;
}

void RailRoadStation::AddRailRoadConnection(
    const RailRoadLineIdType& lineId,
    const RouteIdType& routedId,
    const Vertex& railRoadVertex)
{
    GisSubsystem* subsystemPtr = commonImplPtr->subsystemPtr;

    const VertexIdType railRoadVertexId =
        subsystemPtr->GetVertexId(railRoadVertex);

    const vector<VertexIdType>& entranceVertexIds =
        commonImplPtr->vertexIds;

    for(size_t i = 0; i < entranceVertexIds.size(); i++) {
        const VertexIdType& entranceVertexId = entranceVertexIds[i];

        if (entranceVertexId != railRoadVertexId) {
            subsystemPtr->GetRoadLayer().AddSimplePath(
                entranceVertexId,
                railRoadVertexId,
                ROAD_PATH);
        }
    }

    vertexIdPerLine[make_pair(lineId, routedId)] = railRoadVertexId;
    lineVertexIds.insert(railRoadVertexId);
}

bool RailRoadStation::HasLineVertex(
    const RailRoadLineIdType& lineId,
    const RailRoadIdType& routeId) const
{
    typedef map<pair<RailRoadLineIdType, RailRoadIdType>, VertexIdType>::const_iterator IterType;

    IterType iter = vertexIdPerLine.find(make_pair(lineId, routeId));

    return (iter != vertexIdPerLine.end());
}

VertexIdType RailRoadStation::GetLineVertexId(
    const RailRoadLineIdType& lineId,
    const RailRoadIdType& routeId) const
{
    typedef map<pair<RailRoadLineIdType, RailRoadIdType>, VertexIdType>::const_iterator IterType;

    IterType iter = vertexIdPerLine.find(make_pair(lineId, routeId));

    assert(iter != vertexIdPerLine.end());

    return (*iter).second;
}

VertexIdType RailRoadStation::GetNearestEntranceVertexId(const Vertex& position) const
{
    double minSqrtDistance = DBL_MAX;
    VertexIdType nearestVertexId = INVALID_VERTEX_ID;

    const vector<VertexIdType>& entranceVertexIds = commonImplPtr->vertexIds;

    if (entranceVertexIds.empty()) {
        cerr << "Error: No entrance at " << commonImplPtr->objectName << endl;
        exit(1);
    }

    for(size_t i = 0; i < entranceVertexIds.size(); i++) {

        const VertexIdType& entranceVertexId = entranceVertexIds[i];

        if (!commonImplPtr->subsystemPtr->IsVertexOf(GIS_ROAD, entranceVertexId)) {
            continue;
        }

        const Vertex& entrancePos =
            commonImplPtr->subsystemPtr->GetVertex(entranceVertexId);

        const double sqrtDistance = SquaredXYZDistanceBetweenVertices(
            position, entrancePos);

        if (sqrtDistance < minSqrtDistance) {
            minSqrtDistance = sqrtDistance;
            nearestVertexId = entranceVertexId;
        }
    }
    assert(nearestVertexId != INVALID_VERTEX_ID);

    return nearestVertexId;
}

void RailRoadStation::CompleteEntrances(const size_t defaultNumberEntrances)
{
    if (!entranceIds.empty()) {
        return;
    }

    const vector<Vertex> middlePoints = GetMiddlePointsOfPolygon(polygon);

    for(size_t i = 0; i < std::min(defaultNumberEntrances, middlePoints.size()); i++) {
        entranceIds.insert(commonImplPtr->subsystemPtr->CreateEntrance(middlePoints[i]));
    }
}

void RailRoadStation::UpdateMinRectangle() const
{
    commonImplPtr->minRectangle = GetPointsRect(polygon);
}

bool RailRoadStation::IntersectsWith(const Rectangle& rect) const
{
    return RectIsIntersectsWithPolygon(rect, polygon);
}

Vertex RailRoadStation::AddEntrance(const EntranceIdType& entranceId, const Vertex& point)
{
    entranceIds.insert(entranceId);

    return Vertex(
        point.x,
        point.y,
        (*this).GetVertex().z);
}

void Wall::UpdateMinRectangle() const
{
    commonImplPtr->minRectangle = GetPointsRect(vertices);
}

bool Wall::IntersectsWith(const Rectangle& rect) const
{
    for(size_t i = 0; i < vertices.size(); i++) {
        if (rect.Contains(vertices[i])) {
            return true;
        }
    }

    const Vertex topLeft(rect.minX, rect.maxY);
    const Vertex bottomLeft(rect.minX, rect.minY);
    const Vertex topRight(rect.maxX,rect.maxY);
    const Vertex bottomRight(rect.maxX, rect.minY);

    for(size_t i = 0; i < vertices.size() - 1; i++) {
        const Vertex& p1 = vertices[i];
        const Vertex& p2 = vertices[i+1];

        if (HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft) ||
            HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft) ||
            HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft) ||
            HorizontalLinesAreIntersection(p1, p2, topLeft, bottomLeft)) {
            return true;
        }
    }

    return false;
}

const Material& Wall::GetMaterial() const
{
    return commonImplPtr->subsystemPtr->GetMaterial(materialId);
}

vector<Vertex> Wall::MakeWallPolygon() const
{
    vector<Vertex> polygon;

    const deque<Vertex> dequeVertices(vertices.begin(), vertices.end());

    GetLinePolygon(dequeVertices, widthMeters, polygon);

    return polygon;
}

BuildingIdType Building::GetBuildingId() const
{
    return commonImplPtr->variantId;
}

const Material& Building::GetOuterWallMaterial() const
{
    return commonImplPtr->subsystemPtr->GetMaterial(outerWallMaterialId);
}

const Material& Building::GetRoofMaterial() const
{
    return commonImplPtr->subsystemPtr->GetMaterial(roofWallMaterialId);
}

const Material& Building::GetFloorMaterial() const
{
    return commonImplPtr->subsystemPtr->GetMaterial(floorWallMaterialId);
}

void Building::CompleteEntrances(const size_t defaultNumberEntrances)
{
    if (!entranceIds.empty()) {
        return;
    }

    const vector<Vertex> middlePoints = GetMiddlePointsOfPolygon(polygon);

    for(size_t i = 0; i < std::min(defaultNumberEntrances, middlePoints.size()); i++) {
        entranceIds.insert(commonImplPtr->subsystemPtr->CreateEntrance(middlePoints[i]));
    }
}

Vertex Building::GetRandomPosition(HighQualityRandomNumberGenerator& aRandomNumberGenerator) const
{
    const Rectangle& minRect = (*this).GetMinRectangle();
    const int maxTryCount = 5;

    Vertex randomPosition;

    for(int i = 0; i < maxTryCount; i++) {
        randomPosition.x = minRect.minX + (minRect.maxX - minRect.minX)*aRandomNumberGenerator.GenerateRandomDouble();
        randomPosition.y = minRect.minY + (minRect.maxY - minRect.minY)*aRandomNumberGenerator.GenerateRandomDouble();

        if (PolygonContainsPoint(polygon, randomPosition)) {
            break;
        }
    }

    if (PolygonContainsPoint(polygon, randomPosition)) {
        randomPosition.z = polygon.front().z;
    } else {
        randomPosition = polygon[aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(polygon.size() - 1))];
    }

    return randomPosition;
}

VertexIdType Building::GetNearestEntranceVertexId(const Vertex& position) const
{
    double minSqrtDistance = DBL_MAX;
    VertexIdType nearestVertexId = 0;

    const vector<VertexIdType>& entranceVertexIds = commonImplPtr->vertexIds;

    if (entranceVertexIds.empty()) {
        cerr << "Error: No entrance at " << commonImplPtr->objectName << endl;
        exit(1);
    }

    for(size_t i = 0; i < entranceVertexIds.size(); i++) {

        const VertexIdType& entranceVertexId = entranceVertexIds[i];
        const Vertex& entrancePos =
            commonImplPtr->subsystemPtr->GetVertex(entranceVertexId);

        const double sqrtDistance = SquaredXYZDistanceBetweenVertices(
            position, entrancePos);

        if (sqrtDistance < minSqrtDistance) {
            minSqrtDistance = sqrtDistance;
            nearestVertexId = entranceVertexId;
        }
    }

    return nearestVertexId;
}

void Building::GetNearEntranceVertexIds(const Vertex& position, vector<VertexIdType>& vertexIds) const
{
    const vector<VertexIdType>& entranceVertexIds = commonImplPtr->vertexIds;

    if (entranceVertexIds.empty()) {
        cerr << "Error: No entrance at " << commonImplPtr->objectName << endl;
        exit(1);
    }

    multimap<double, VertexIdType> nearVertexIds;

    for(size_t i = 0; i < entranceVertexIds.size(); i++) {

        const VertexIdType& entranceVertexId = entranceVertexIds[i];
        const Vertex& entrancePos =
            commonImplPtr->subsystemPtr->GetVertex(entranceVertexId);

        const double sqrtDistance = SquaredXYZDistanceBetweenVertices(
            position, entrancePos);

        nearVertexIds.insert(make_pair(sqrtDistance, entranceVertexId));
    }

    typedef multimap<double, VertexIdType>::const_iterator IterType;

    vertexIds.clear();

    for(IterType iter = nearVertexIds.begin();
        iter != nearVertexIds.end(); iter++) {

        vertexIds.push_back((*iter).second);
    }
}

double Building::CalculateSize() const
{
    return CalculatePolygonSize(polygon);
}

Vertex Building::AddEntrance(
    const EntranceIdType& entranceId,
    const Vertex& point)
{
    entranceIds.insert(entranceId);

    Vertex entrancePosition(
        point.x,
        point.y);

    if (!polygon.empty()) {
        entrancePosition.z = polygon.front().z;
    }

    return entrancePosition;
}

RoadIdType Building::GetNearestEntranceRoadId(const Vertex& position) const
{
    const VertexIdType nearestVertexId = (*this).GetNearestEntranceVertexId(position);
    const GisVertex& poiGisVertex = commonImplPtr->subsystemPtr->GetGisVertex(nearestVertexId);
    const vector<RoadIdType>& roadIds = poiGisVertex.GetConnectedObjectIds(GIS_ROAD);

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType& roadId = roadIds[i];

        if (commonImplPtr->subsystemPtr->GetRoad(roadId).IsParking()) {
            return roadId;
        }
    }

    cerr << "Error: No entrance road at " << commonImplPtr->objectName << endl;
    exit(1);

    return INVALID_VARIANT_ID;
}

void Building::UpdateMinRectangle() const
{
    commonImplPtr->minRectangle = GetPointsRect(polygon);
}

bool Building::IntersectsWith(const Rectangle& rect) const
{
    return RectIsIntersectsWithPolygon(rect, polygon);
}

void Area::UpdateMinRectangle() const
{
    commonImplPtr->minRectangle = GetPointsRect(polygon);
}

double Area::CalculateSize() const
{
    return CalculatePolygonSize(polygon);
}

bool Area::IntersectsWith(const Rectangle& rect) const
{
    return RectIsIntersectsWithPolygon(rect, polygon);
}

ParkIdType Park::GetParkId() const
{
    return commonImplPtr->variantId;
}

void Park::CompleteEntrances(const size_t defaultNumberEntrances)
{
    if (!entranceIds.empty()) {
        return;
    }

    const vector<Vertex> middlePoints = GetMiddlePointsOfPolygon(polygon);

    for(size_t i = 0; i < std::min(defaultNumberEntrances, middlePoints.size()); i++) {
        entranceIds.insert(commonImplPtr->subsystemPtr->CreateEntrance(middlePoints[i]));
    }
}

void Park::UpdateMinRectangle() const
{
    commonImplPtr->minRectangle = GetPointsRect(polygon);
}

bool Park::IntersectsWith(const Rectangle& rect) const
{
    return RectIsIntersectsWithPolygon(rect, polygon);
}

Vertex Park::GetRandomPosition(HighQualityRandomNumberGenerator& aRandomNumberGenerator) const
{
    const Rectangle& minRect = (*this).GetMinRectangle();
    const int maxTryCount = 5;

    Vertex randomPosition;

    for(int i = 0; i < maxTryCount; i++) {
        randomPosition.x = minRect.minX + (minRect.maxX - minRect.minX)*aRandomNumberGenerator.GenerateRandomDouble();
        randomPosition.y = minRect.minY + (minRect.maxY - minRect.minY)*aRandomNumberGenerator.GenerateRandomDouble();

        if (PolygonContainsPoint(polygon, randomPosition)) {
            break;
        }
    }

    if (PolygonContainsPoint(polygon, randomPosition)) {
        randomPosition.z = polygon.front().z;
    } else {
        randomPosition = polygon[aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(polygon.size() - 1))];
    }

    return randomPosition;
}

VertexIdType Park::GetNearestEntranceVertexId(const Vertex& position) const
{
    double minSqrtDistance = DBL_MAX;
    VertexIdType nearestVertexId = 0;

    const vector<VertexIdType>& entranceVertexIds = commonImplPtr->vertexIds;
    assert(!entranceVertexIds.empty());

    for(size_t i = 0; i < entranceVertexIds.size(); i++) {

        const VertexIdType& entranceVertexId = entranceVertexIds[i];
        const Vertex& entrancePos =
            commonImplPtr->subsystemPtr->GetVertex(entranceVertexId);

        const double sqrtDistance = SquaredXYZDistanceBetweenVertices(
            position, entrancePos);

        if (sqrtDistance < minSqrtDistance) {
            minSqrtDistance = sqrtDistance;
            nearestVertexId = entranceVertexId;
        }
    }

    return nearestVertexId;
}

void Park::GetNearEntranceVertexIds(const Vertex& position, vector<VertexIdType>& vertexIds) const
{
    const vector<VertexIdType>& entranceVertexIds = commonImplPtr->vertexIds;

    if (entranceVertexIds.empty()) {
        cerr << "Error: No entrance at " << commonImplPtr->objectName << endl;
        exit(1);
    }

    multimap<double, VertexIdType> nearVertexIds;

    for(size_t i = 0; i < entranceVertexIds.size(); i++) {

        const VertexIdType& entranceVertexId = entranceVertexIds[i];
        const Vertex& entrancePos =
            commonImplPtr->subsystemPtr->GetVertex(entranceVertexId);

        const double sqrtDistance = SquaredXYZDistanceBetweenVertices(
            position, entrancePos);

        nearVertexIds.insert(make_pair(sqrtDistance, entranceVertexId));
    }

    typedef multimap<double, VertexIdType>::const_iterator IterType;

    vertexIds.clear();

    for(IterType iter = nearVertexIds.begin();
        iter != nearVertexIds.end(); iter++) {

        vertexIds.push_back((*iter).second);
    }
}

RoadIdType Park::GetNearestEntranceRoadId(const Vertex& position) const
{
    const VertexIdType nearestVertexId = (*this).GetNearestEntranceVertexId(position);
    const GisVertex& poiGisVertex = commonImplPtr->subsystemPtr->GetGisVertex(nearestVertexId);
    const vector<RoadIdType>& roadIds = poiGisVertex.GetConnectedObjectIds(GIS_ROAD);

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType& roadId = roadIds[i];

        if (commonImplPtr->subsystemPtr->GetRoad(roadId).IsParking()) {
            return roadId;
        }
    }

    cerr << "Error: No entrance road at " << commonImplPtr->objectName << endl;
    exit(1);

    return INVALID_VARIANT_ID;
}

double Park::CalculateSize() const
{
    return CalculatePolygonSize(polygon);
}

Vertex Park::AddEntrance(
    const EntranceIdType& entranceId,
    const Vertex& point)
{
    entranceIds.insert(entranceId);

    Vertex entrancePosition(
        point.x,
        point.y);

    if (!polygon.empty()) {
        entrancePosition.z = polygon.front().z;
    }

    return entrancePosition;
}

Rectangle Triangle::GetRect() const
{
    return Rectangle(
        std::min(std::min(p1.x, p2.x), p3.x),
        std::min(std::min(p1.y, p2.y), p3.y),
        std::max(std::max(p1.x, p2.x), p3.x),
        std::max(std::max(p1.y, p2.y), p3.y));
}

double Triangle::GetSurfaceHeightAt(const Vertex& pos) const
{
    const Vertex vec1 = p2 - p1;
    const Vertex vec2 = p3 - p1;
    const Vertex p(vec1.y*vec2.z - vec1.z*vec2.y, vec1.z*vec2.x - vec1.x*vec2.z, vec1.x*vec2.y - vec1.y*vec2.x);
    const double d = -(p.x*p1.x + p.y*p1.y + p.z*p1.z);

    if (p.z == 0.) {
        return CalculatePointToLineNearestPosition(pos, p2, p3).z;
    }

    return - (d + (p.x*pos.x + p.y*pos.y))/p.z;
}

bool Triangle::Contains(const Vertex& pos) const
{
    return IsInsideOfTheTriangle(pos, p1, p2, p3);
}

bool Triangle::IntersectsWith(const Rectangle& rect) const
{
    vector<Vertex> polygon;

    polygon.push_back(p1);
    polygon.push_back(p2);
    polygon.push_back(p3);
    polygon.push_back(p1);

    return RectIsIntersectsWithPolygon(rect, polygon);
}

bool Triangle::IntersectsWithLine(
    const Vertex& lineEdge1,
    const Vertex& lineEdge2) const
{
    if ((*this).Contains(lineEdge1) ||
        (*this).Contains(lineEdge2)) {
        return true;
    }

    if (HorizontalLinesAreIntersection(lineEdge1, lineEdge2, p1, p2) ||
        HorizontalLinesAreIntersection(lineEdge1, lineEdge2, p2, p3) ||
        HorizontalLinesAreIntersection(lineEdge1, lineEdge2, p3, p1)) {
        return true;
    }

    return false;
}

//------------------------------------------------------------------------------
// Layer
//------------------------------------------------------------------------------

RoadLayer::RoadLayer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    GisSubsystem* initSubsystemPtr)
    :
    subsystemPtr(initSubsystemPtr),
    isRightHandTraffic(false),
    breakDownCurvedRoads(false),
    numberEntrancesToBuilding(0),
    numberEntrancesToStation(0),
    numberEntrancesToBusStop(0),
    numberEntrancesToPark(0),
    setIntersectionMargin(false),
    maxRoadWidthMeters(0.01) //1cm
{
    if (theParameterDatabaseReader.ParameterExists("gis-road-driving-side")) {

        const string drivingSide =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("gis-road-driving-side"));

        if (drivingSide == "right") {
            isRightHandTraffic = true;
        }
        else if (drivingSide == "left") {
            isRightHandTraffic = false;
        }
        else {
            cerr << "Unknown Driving Side (right|left): " << drivingSide << endl;
            exit(1);
        }//if//

    }//if//

    if (theParameterDatabaseReader.ParameterExists("gis-los-break-down-cureved-road-into-straight-roads")) {
        // cost match memory
        breakDownCurvedRoads = theParameterDatabaseReader.ReadBool("gis-los-break-down-cureved-road-into-straight-roads");
    }

    if (theParameterDatabaseReader.ParameterExists("gis-number-entrances-to-building")) {
        numberEntrancesToBuilding = theParameterDatabaseReader.ReadNonNegativeInt("gis-number-entrances-to-building");
    }
    if (theParameterDatabaseReader.ParameterExists("gis-number-entrances-to-station")) {
        numberEntrancesToStation = theParameterDatabaseReader.ReadNonNegativeInt("gis-number-entrances-to-station");
    }
    if (theParameterDatabaseReader.ParameterExists("gis-number-entrances-to-busstop")) {
        numberEntrancesToBusStop = theParameterDatabaseReader.ReadNonNegativeInt("gis-number-entrances-to-busstop");
    }
    if (theParameterDatabaseReader.ParameterExists("gis-number-entrances-to-park")) {
        numberEntrancesToPark = theParameterDatabaseReader.ReadNonNegativeInt("gis-number-entrances-to-park");
    }

    if (theParameterDatabaseReader.ParameterExists("gis-road-set-intersection-margin")) {
        setIntersectionMargin = theParameterDatabaseReader.ReadBool("gis-road-set-intersection-margin");
    }
}

void RoadLayer::ImportIntersection(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);

    map<TimeType, list<TrafficLightIdType> > trafficLightIdsPerTime;

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);
        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);

        assert(shpObjPtr->nParts == 0);
        assert(shpObjPtr->nVertices == 1);

        const double x = shpObjPtr->padfX[0];
        const double y = shpObjPtr->padfY[0];
        double z = shpObjPtr->padfZ[0];

        if (ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId)) {
            z += subsystemPtr->GetGroundElevationMetersAt(x, y);
        }

        const Vertex point(x, y, z);

        string name;

        if (nameFinder.IsAvailable()) {
            name = nameFinder.GetLowerString(entryId);
        }

        (*this).CreateIntersectionIfNecessary(point, name);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

void RoadLayer::ImportRoad(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder widthFinder(hDBF, GIS_DBF_WIDTH_STRING);
    const AttributeFinder startToEndLaneFinder(hDBF, GIS_DBF_LANE12_STRING);
    const AttributeFinder endToStartLaneFinder(hDBF, GIS_DBF_LANE21_STRING);
    const AttributeFinder typeFinder(hDBF, GIS_DBF_TYPE_STRING);
    const AttributeFinder capacityFinder(hDBF, GIS_DBF_CAPACITY_STRING);
    const AttributeFinder speedLimitFinder(hDBF, GIS_DBF_SPEED_LIMIT_STRING);

    set<Vertex> vertices;

    // register intersection
    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        Vertex startPoint(
            shpObjPtr->padfX[0],
            shpObjPtr->padfY[0],
            shpObjPtr->padfZ[0]);

        Vertex endPoint(
            shpObjPtr->padfX[shpObjPtr->nVertices-1],
            shpObjPtr->padfY[shpObjPtr->nVertices-1],
            shpObjPtr->padfZ[shpObjPtr->nVertices-1]);

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        if (isBaseGroundLevel) {
            startPoint.z += subsystemPtr->GetGroundElevationMetersAt(startPoint);
            endPoint.z += subsystemPtr->GetGroundElevationMetersAt(endPoint);
        }

        (*this).CreateIntersectionIfNecessary(startPoint);
        (*this).CreateIntersectionIfNecessary(endPoint);

        vertices.insert(startPoint);
        vertices.insert(endPoint);

        if (breakDownCurvedRoads) {
            for(int i = 1; i < shpObjPtr->nVertices - 1; i++) {
                const double x = shpObjPtr->padfX[i];
                const double y = shpObjPtr->padfY[i];
                double z = shpObjPtr->padfZ[i];

                if (isBaseGroundLevel) {
                    z += subsystemPtr->GetGroundElevationMetersAt(x, y);
                }

                const Vertex point(x, y, z);

                if (vertices.find(point) == vertices.end()) {
                    (*this).CreateIntersectionIfNecessary(point);
                    subsystemPtr->GetVertexId(point);
                    vertices.insert(point);
                }
            }
        }

        SHPDestroyObject(shpObjPtr);
    }

    // register roads
    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        vector<VertexIdType> vertexIds;
        deque<Vertex> vertices;

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        for(int i = 0; i < shpObjPtr->nVertices; i++) {
            const double x = shpObjPtr->padfX[i];
            const double y = shpObjPtr->padfY[i];
            double z = shpObjPtr->padfZ[i];

            if (isBaseGroundLevel) {
                z += subsystemPtr->GetGroundElevationMetersAt(x, y);
            }

            const Vertex point(x, y, z);

            vertices.push_back(point);

            const VertexIdType vertexId = subsystemPtr->GetVertexId(point);
            vertexIds.push_back(vertexId);

            if (!breakDownCurvedRoads) {
                if (!subsystemPtr->IsVertexPoint(point)) {
                    continue;
                }
                if (!subsystemPtr->IsIntersectionVertex(vertexId)) {
                    continue;
                }
            }

            if (vertexIds.size() >= 2) {
                const RoadIdType roadId = static_cast<RoadIdType>(roads.size());
                roads.push_back(Road(subsystemPtr, objectId, roadId));

                Road& road = roads.back();

                road.LoadParameters(theParameterDatabaseReader);
                road.isBaseGroundLevel = isBaseGroundLevel;

                if (nameFinder.IsAvailable()) {
                    road.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
                }
                if (startToEndLaneFinder.IsAvailable()) {
                    const int numberStartToEndLanes =
                        startToEndLaneFinder.GetInt(entryId);

                    if (numberStartToEndLanes < 0) {
                        cerr << "Error: Number of lanes is negative value: " << numberStartToEndLanes << " for " << road.commonImplPtr->objectName << endl;
                        exit(1);
                    }

                    road.numberStartToEndLanes = static_cast<size_t>(numberStartToEndLanes);
                }
                if (endToStartLaneFinder.IsAvailable()) {
                    const int numberEndToStartLanes =
                        endToStartLaneFinder.GetInt(entryId);

                    if (numberEndToStartLanes < 0) {
                        cerr << "Error: Number of lanes is negative value: " << numberEndToStartLanes << " for " << road.commonImplPtr->objectName << endl;
                        exit(1);
                    }

                    road.numberEndToStartLanes = static_cast<size_t>(numberEndToStartLanes);
                }

                if (road.numberStartToEndLanes == 0 &&
                    road.numberEndToStartLanes == 0) {
                    cerr << "Error: Total number of lanes is 0 for " << road.commonImplPtr->objectName << endl
                         << "  Total number of lanes must be greater than 0." << endl;
                    exit(1);
                }

                if (widthFinder.IsAvailable()) {
                    road.widthMeters = widthFinder.GetDouble(entryId);
                } else {
                    const double defaultRoadWidthMeters = 6;
                    road.widthMeters =
                        defaultRoadWidthMeters*
                        (road.numberStartToEndLanes + road.numberEndToStartLanes);
                }

                if (typeFinder.IsAvailable()) {
                    const string typeName = typeFinder.GetLowerString(entryId);

                    if (typeName == "road" || typeName == "vehicleandpedestrian") {
                        road.type = ROAD_ROAD;
                    } else if (typeName == "pedestrian" || typeName == "pedestianonly") {
                        road.type = ROAD_PEDESTRIAN;
                    } else if (typeName == "motorway" || typeName == "vehicleonly") {
                        road.type = ROAD_MOTORWAY;
                    } else {
                        road.type = ROAD_ROAD;
                    }
                }

                road.isRightHandTraffic = isRightHandTraffic;
                road.vertices = vertices;
                road.commonImplPtr->vertexIds = vertexIds;
                road.UpdatePolygon();

                assert(road.polygon.size() > 2);

                if (capacityFinder.IsAvailable()) {
                    road.capacity = capacityFinder.GetDouble(entryId);
                    road.humanCapacity =
                        static_cast<int>(
                            std::min<double>(INT_MAX, std::ceil(road.capacity*CalculatePolygonSize(road.polygon))));
                    assert(road.humanCapacity >= 0);
                }

                if (speedLimitFinder.IsAvailable()) {
                    road.speedLimitMetersPerSec =
                        speedLimitFinder.GetDouble(entryId) / 3.6; //convert km/h -> m/s

                    if (road.speedLimitMetersPerSec <= 0.) {
                        cerr << "Error: Speed limit is " << speedLimitFinder.GetDouble(entryId) << " for " << road.commonImplPtr->objectName << endl
                             << "  Speed limit must be greater than 0." << endl;
                        exit(1);
                    }
                }

                const VertexIdType startVertexId = vertexIds.front();
                const VertexIdType endVertexId = vertexIds.back();

                subsystemPtr->ConnectBidirectionalGisObject(
                    startVertexId, GIS_ROAD, endVertexId, roadId);

                subsystemPtr->RegisterGisObject(road, roadId);

                maxRoadWidthMeters = std::max(maxRoadWidthMeters, road.widthMeters);

                vertices.clear();
                vertexIds.clear();

                vertices.push_back(point);
                vertexIds.push_back(vertexId);
            }
        }

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

static inline
bool IsStraightDirection(const double radians)
{
    const double directionRadians =
        std::max<double>(0, ((radians) - (PI/4.)) / (PI/2.));

    return (1. <= directionRadians && directionRadians < 2.);
}

void RoadLayer::ImportTrafficLight(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    map<string, PeriodicTrafficLightPatternData> periodicTrafficLightPatterns;
    map<string, DistributedTrafficLightPatternData> distributedTrafficLightPatterns;

    (*this).ReadTrafficLightPatternFileIfNecessary(
        theParameterDatabaseReader,
        periodicTrafficLightPatterns,
        distributedTrafficLightPatterns);

    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder offsetFinder(hDBF, GIS_DBF_OFFSET_STRING);
    const AttributeFinder greenFinder(hDBF, GIS_DBF_GREEN_STRING);
    const AttributeFinder yellowFinder(hDBF, GIS_DBF_YELLOW_STRING);
    const AttributeFinder redFinder(hDBF, GIS_DBF_RED_STRING);
    const AttributeFinder patternFinder(hDBF, GIS_DBF_PATTERN_STRING);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder roadIdFinder(hDBF, GIS_DBF_ROADID_STRING);

    map<TimeType, list<TrafficLightIdType> > trafficLightIdsPerTime;

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);
        GisObjectIdType objectId = INVALID_GIS_OBJECT_ID;

        if (idFinder.IsAvailable()) {
            objectId = idFinder.GetGisObjectId(entryId);
        }

        assert(shpObjPtr->nParts == 0);
        assert(shpObjPtr->nVertices == 1);

        const double x = shpObjPtr->padfX[0];
        const double y = shpObjPtr->padfY[0];
        double z = shpObjPtr->padfZ[0];

        if (ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId)) {
            z += subsystemPtr->GetGroundElevationMetersAt(x, y);
        }

        const Vertex point(x, y, z);

        string name;
        TimeType startOffset = ZERO_TIME;
        TimeType greenDuration = 60*SECOND;
        TimeType yellowDuration = 0*SECOND;
        TimeType redDuration = 60*SECOND;
        string patternName;
        map<TimeType, TrafficLightType> trafficLightPerTime;

        GisPositionIdType intersectionPositionId;
        GisPositionIdType roadPositionId;

        if (nameFinder.IsAvailable()) {
            name = nameFinder.GetLowerString(entryId);
        }


        if (patternFinder.IsAvailable()) {
            patternName = patternFinder.GetLowerString(entryId);
        }

        if (!patternName.empty()) {
            if (periodicTrafficLightPatterns.find(patternName) != periodicTrafficLightPatterns.end()) {
                const PeriodicTrafficLightPatternData& patternData =
                    periodicTrafficLightPatterns[patternName];

                greenDuration = patternData.greenDuration;
                yellowDuration = patternData.yellowDuration;
                redDuration = patternData.redDuration;

            } else if (distributedTrafficLightPatterns.find(patternName) != distributedTrafficLightPatterns.end()) {
                const DistributedTrafficLightPatternData& patternData =
                    distributedTrafficLightPatterns[patternName];

                trafficLightPerTime = patternData.trafficLightPerTime;
            }
        } else {
            if (offsetFinder.IsAvailable()) {
                startOffset = static_cast<TimeType>(SECOND*offsetFinder.GetDouble(entryId));
            }
            if (greenFinder.IsAvailable()) {
                greenDuration = static_cast<TimeType>(SECOND*greenFinder.GetDouble(entryId));
            }
            if (yellowFinder.IsAvailable()) {
                yellowDuration = static_cast<TimeType>(SECOND*yellowFinder.GetDouble(entryId));
            }
            if (redFinder.IsAvailable()) {
                redDuration = static_cast<TimeType>(SECOND*redFinder.GetDouble(entryId));
            }
        }

        if (redDuration == ZERO_TIME) {
            cerr << "Error: Set non-zero red duration for trafficlight " << name << endl;
            exit(1);
        }

        const VertexIdType vertexId = subsystemPtr->GetVertexId(point);

        if (subsystemPtr->IsIntersectionVertex(vertexId)) {
            if (greenDuration + yellowDuration > redDuration) {
                cerr << "Error: Set red duration to be larger than sum of green duration and yellow duration for trafficlight " << name << endl;
                exit(1);
            }

            const IntersectionIdType intersectionId =
                subsystemPtr->GetIntersectionId(vertexId);

            Intersection& intersection = intersections[intersectionId];

            const Vertex& origVertex = intersection.GetVertex();
            const vector<RoadIdType>& connectedRoadIds = intersection.GetConnectedRoadIds();

            set<RoadIdType> alreadyGroupedRoadIds;
            vector<vector<RoadIdType> > groupedRoadIds;

            for(size_t i = 0; i < connectedRoadIds.size(); i++) {
                const RoadIdType roadId1 = connectedRoadIds[i];

                if (alreadyGroupedRoadIds.find(roadId1) != alreadyGroupedRoadIds.end()) {
                    continue;
                }

                groupedRoadIds.push_back(vector<RoadIdType>());
                vector<RoadIdType>& roadIds = groupedRoadIds.back();

                const Road& road1 = roads[roadId1];

                roadIds.push_back(roadId1);
                alreadyGroupedRoadIds.insert(roadId1);

                for(size_t j = i+1; j < connectedRoadIds.size(); j++) {
                    const RoadIdType roadId2 = connectedRoadIds[j];

                    if (alreadyGroupedRoadIds.find(roadId2) != alreadyGroupedRoadIds.end()) {
                        continue;
                    }

                    const Road& road2 = roads[roadId2];
                    const Vertex& vertex1 = road1.GetNeighborVertex(intersectionId);
                    const Vertex& vertex2 = road2.GetNeighborVertex(intersectionId);

                    if (IsStraightDirection(CalculateFullRadiansBetweenVector(vertex1, origVertex, vertex2))) {

                        assert(IsStraightDirection(CalculateFullRadiansBetweenVector(vertex2, origVertex, vertex1)));
                        roadIds.push_back(roadId2);
                        alreadyGroupedRoadIds.insert(roadId2);
                    }
                }
            }

            if (groupedRoadIds.size() > 0) {
                const TimeType cycleDuration = greenDuration + yellowDuration;

                if (cycleDuration == ZERO_TIME) {
                    redDuration = INFINITE_TIME;
                } else {
                    redDuration = redDuration * (groupedRoadIds.size() - 1);
                }

                for(size_t i = 0; i < groupedRoadIds.size(); i++) {
                    const vector<RoadIdType>& roadIds = groupedRoadIds[i];

                    const TrafficLightIdType trafficLightId = static_cast<TrafficLightIdType>(trafficLights.size());

                    (*this).AddTrafficLight(
                        theParameterDatabaseReader,
                        objectId,
                        intersection.GetVertexId(),
                        trafficLightPerTime,
                        startOffset + cycleDuration*i,
                        greenDuration,
                        yellowDuration,
                        redDuration,
                        trafficLightIdsPerTime);

                    for(size_t j = 0; j < roadIds.size(); j++) {
                        intersection.trafficLightIds.insert(make_pair(roadIds[j], trafficLightId));
                    }
                }
            }

        } else if (roadIdFinder.IsAvailable()) {
            const GisObjectIdType roadObjectId = roadIdFinder.GetGisObjectId(entryId);

            if (!subsystemPtr->ContainsObject(roadObjectId)) {
                cerr << "Error: trafficlight [" << name << "] is located at invalid road." << endl;
                exit(1);
            }

            roadPositionId = subsystemPtr->GetPositionId(roadObjectId);

            assert(roadPositionId.type == GIS_ROAD);

            const Road& road = roads[roadPositionId.id];

            const IntersectionIdType intersectionid = road.GetNearestIntersectionId(point);

            Intersection& intersection = intersections[intersectionid];

            intersection.trafficLightIds.insert(
                make_pair(
                    roadPositionId.id,
                    (TrafficLightIdType)(trafficLights.size())));

            (*this).AddTrafficLight(
                theParameterDatabaseReader,
                objectId,
                intersection.GetVertexId(),
                trafficLightPerTime,
                startOffset,
                greenDuration,
                yellowDuration,
                redDuration,
                trafficLightIdsPerTime);

        } else {
        }

        SHPDestroyObject(shpObjPtr);
    }

    trafficLightChanges.assign(trafficLightIdsPerTime.begin(), trafficLightIdsPerTime.end());

    SHPClose(hSHP);
    DBFClose(hDBF);
}

void RoadLayer::AddTrafficLight(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GisObjectIdType& objectId,
    const VertexIdType& vertexId,
    const map<TimeType, TrafficLightType>& trafficLightPerTime,
    const TimeType& startOffset,
    const TimeType& greenDuration,
    const TimeType& yellowDuration,
    const TimeType& redDuration,
    map<TimeType, list<TrafficLightIdType> >& trafficLightIdsPerTime)
{
    const TrafficLightIdType trafficLightId = static_cast<TrafficLightIdType>(trafficLights.size());

    if (trafficLightPerTime.empty()) {
        trafficLights.push_back(
            TrafficLight(
                subsystemPtr,
                objectId,
                trafficLightId,
                vertexId,
                startOffset,
                greenDuration,
                yellowDuration,
                redDuration));
    } else {

        trafficLights.push_back(
            TrafficLight(
                subsystemPtr,
                objectId,
                trafficLightId,
                vertexId,
                trafficLightPerTime));

        typedef map<TimeType, TrafficLightType>::const_iterator IterType;

        for(IterType iter = trafficLightPerTime.begin();
            iter != trafficLightPerTime.end(); iter++) {

            const TimeType& time = (*iter).first;

            trafficLightIdsPerTime[time].push_back(trafficLightId);
        }
    }

    trafficLights.back().LoadParameters(theParameterDatabaseReader);
}

void RoadLayer::SyncTrafficLight(const TimeType& currentTime)
{
    while (!trafficLightChanges.empty() &&
           currentTime >= trafficLightChanges.front().first) {

        const list<TrafficLightIdType>& changedTrafficLightIds = trafficLightChanges.front().second;

        typedef list<TrafficLightIdType>::const_iterator IterType;

        for(IterType iter = changedTrafficLightIds.begin();
            iter != changedTrafficLightIds.end(); iter++) {

            const TrafficLightIdType& trafficLightId = (*iter);

            trafficLights[trafficLightId].SyncTrafficLight(currentTime);
        }

        trafficLightChanges.pop_front();
    }
}

void RoadLayer::ReadTrafficLightPatternFileIfNecessary(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    map<string, PeriodicTrafficLightPatternData>& periodicTrafficLightPatterns,
    map<string, DistributedTrafficLightPatternData>& distributedTrafficLightPattern)
{
    TimeType synchronizationStep = 1 * NANO_SECOND;

    if (theParameterDatabaseReader.ParameterExists("time-step-event-synchronization-step")) {
        synchronizationStep = theParameterDatabaseReader.ReadTime("time-step-event-synchronization-step");
    }

    if (theParameterDatabaseReader.ParameterExists("gis-trafficlight-pattern-definition-file")) {
        const string patternFilePath =
            theParameterDatabaseReader.ReadString("gis-trafficlight-pattern-definition-file");

        ifstream inStream(patternFilePath.c_str());

        if (!inStream.good()) {
            cerr << "Error: Couldn't open trafficlight pattern file: " << patternFilePath << endl;
            exit(1);
        }//if//

        while(!inStream.eof()) {
            string aLine;
            getline(inStream, aLine);

            DeleteTrailingSpaces(aLine);

            if (IsAConfigFileCommentLine(aLine)) {
                continue;
            }

            deque<string> tokens;
            TokenizeToTrimmedLowerString(aLine, " ", tokens);

            if (tokens.size() < 2) {
                continue;
            }

            const string patternName = tokens[0];
            tokens.pop_front();

            if (aLine.find(":") != string::npos) {

                DistributedTrafficLightPatternData& patternData = distributedTrafficLightPattern[patternName];

                for(size_t i = 0; i < tokens.size(); i++) {
                    deque<string> timeAndLightType;

                    TokenizeToTrimmedLowerString(tokens[i], ":", timeAndLightType);

                    if (timeAndLightType.size() != 2) {
                        cerr << "Error: trafficlight pattern file: " << patternFilePath << " " << aLine << endl
                             << "PatternName [Time:LightType]+ " << endl;
                        exit(1);
                    }

                    TimeType time;
                    bool success;

                    ConvertStringToTime(timeAndLightType[0], time, success);

                    if (!success) {
                        cerr << "Error: trafficlight pattern file: " << patternFilePath << " " << aLine << endl
                             << "PatternName [Time:LightType]+ " << endl;
                        exit(1);
                    }

                    if (time % synchronizationStep != ZERO_TIME) {
                        cerr << "Error: trafficlight pattern file: " << patternFilePath << " " << aLine << endl
                             << "Time must be multiple number of time-step-event-synchronization-step." << endl;
                    }

                    TrafficLightType lightType;

                    if (timeAndLightType[1].find("blue") != string::npos ||
                        timeAndLightType[1].find("green") != string::npos) {
                        lightType = TRAFFIC_LIGHT_GREEN;
                    } else if (timeAndLightType[1].find("yellow") != string::npos) {
                        lightType = TRAFFIC_LIGHT_YELLOW;
                    } else if (timeAndLightType[1].find("red") != string::npos) {
                        lightType = TRAFFIC_LIGHT_RED;
                    } else {
                        cerr << "Error: trafficlight pattern file: " << patternFilePath << " " << aLine << endl
                             << "PatternName [Time:LightType]+ " << endl;
                        exit(1);
                    }

                    patternData.trafficLightPerTime[time] = lightType;
                }

            } else {

                if (tokens.size() < 3) {
                    cerr << "Error: trafficlight pattern file: " << patternFilePath << " " << aLine << endl
                         << "PatternName BlueDration YellowDuration RedDuration" << endl;
                    exit(1);
                }

                TimeType greenDuration;
                TimeType yellowDuration;
                TimeType redDuration;

                bool success1;
                bool success2;
                bool success3;

                ConvertStringToTime(tokens[0], greenDuration, success1);
                ConvertStringToTime(tokens[1], yellowDuration, success2);
                ConvertStringToTime(tokens[2], redDuration, success3);

                if (!success1 || !success2 || !success3) {
                    cerr << "Error: trafficlight pattern file: " << patternFilePath << " " << aLine << endl
                         << "PatternName BlueDration YellowDuration RedDuration" << endl;
                    exit(1);
                }

                periodicTrafficLightPatterns.insert(
                    make_pair(
                        patternName,
                        PeriodicTrafficLightPatternData(
                            greenDuration,
                            yellowDuration,
                            redDuration)));
            }
        }
    }
}

void RoadLayer::ImportBusStop(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder capacityFinder(hDBF, GIS_DBF_CAPACITY_STRING);

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);
        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);

        assert(shpObjPtr->nParts == 0);
        assert(shpObjPtr->nVertices == 1);

        const double x = shpObjPtr->padfX[0];
        const double y = shpObjPtr->padfY[0];
        double z = shpObjPtr->padfZ[0];

        if (ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId)) {
            z += subsystemPtr->GetGroundElevationMetersAt(x, y);
        }

        const Vertex point(x, y, z);

        bool foundRoad;
        RoadIdType roadId;

        (*this).FindRoadAt(point, foundRoad, roadId);

        if (!foundRoad) {
            cerr << "Error: No road is located arround bus stop." << endl;
            exit(1);
        }

        const BusStopIdType busStopId = static_cast<BusStopIdType>(busStops.size());

        busStops.push_back(BusStop(subsystemPtr, objectId, busStopId));
        BusStop& busStop = busStops.back();

        busStop.position = point;

        if (nameFinder.IsAvailable()) {
            busStop.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }
        if (capacityFinder.IsAvailable()) {
            busStop.humanCapacity = capacityFinder.GetInt(entryId);
        }

        subsystemPtr->RegisterGisObject(busStop, busStopId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

void RoadLayer::CreateIntersectionIfNecessary(
    const Vertex& point,
    const string& name)
{
    const VertexIdType& vertexId = subsystemPtr->GetVertexId(point);

    if (!subsystemPtr->IsIntersectionVertex(vertexId)) {
        const IntersectionIdType intersectionId = static_cast<IntersectionIdType>(intersections.size());

        intersections.push_back(Intersection(subsystemPtr, this, subsystemPtr->CreateNewObjectId(), intersectionId, vertexId));
        intersections.back().commonImplPtr->objectName = name;

        subsystemPtr->ConnectGisObject(vertexId, GIS_INTERSECTION, intersectionId);
        subsystemPtr->RegisterGisObject(intersections.back(), intersectionId);
    }
}

void RoadLayer::DivideRoad(
    const RoadIdType& srcRoadId,
    const VertexIdType& newVertexId)
{
    size_t vertexNumber = 0;
    Road& srcRoad = roads.at(srcRoadId);

    if (newVertexId != srcRoad.GetStartVertexId() &&
        newVertexId != srcRoad.GetEndVertexId()) {

        const Vertex& position = subsystemPtr->GetVertex(newVertexId);
        const double capacity = srcRoad.capacity;

        subsystemPtr->DisconnectBidirectionalGisObject(
            srcRoad.GetStartVertexId(),
            GIS_ROAD,
            srcRoad.GetEndVertexId(),
            srcRoadId);

        subsystemPtr->UnregisterGisObject(
            srcRoad, srcRoadId);

        const RoadIdType tailRoadId = static_cast<RoadIdType>(roads.size());

        const IntersectionIdType& startIntersectionId = srcRoad.GetStartIntersectionId();
        const IntersectionIdType& endIntersectionId = srcRoad.GetEndIntersectionId();

        map<RoadIdType, TrafficLightIdType>& startTrafficLightIds = intersections[startIntersectionId].trafficLightIds;
        map<RoadIdType, TrafficLightIdType>& endTrafficLightIds = intersections[endIntersectionId].trafficLightIds;

        if (startTrafficLightIds.find(srcRoadId) != startTrafficLightIds.end()) {
            startTrafficLightIds[tailRoadId] = startTrafficLightIds[srcRoadId];
        }
        if (endTrafficLightIds.find(srcRoadId) != endTrafficLightIds.end()) {
            endTrafficLightIds[tailRoadId] = endTrafficLightIds[srcRoadId];
        }

        const GisObjectIdType srcObjectId = srcRoad.GetObjectId();

        roads.push_back(
            Road(subsystemPtr, srcObjectId, tailRoadId));

        Road& headRoad = roads.at(srcRoadId);
        Road& tailRoad = roads.back();

        tailRoad.commonImplPtr->objectName = headRoad.commonImplPtr->objectName;
        tailRoad.isRightHandTraffic = isRightHandTraffic;
        tailRoad.widthMeters = headRoad.widthMeters;
        tailRoad.isBaseGroundLevel = headRoad.isBaseGroundLevel;
        tailRoad.numberStartToEndLanes = headRoad.numberEndToStartLanes;
        tailRoad.numberEndToStartLanes = headRoad.numberStartToEndLanes;
        tailRoad.speedLimitMetersPerSec = headRoad.speedLimitMetersPerSec;
        tailRoad.type = headRoad.type;

        double minDistance = DBL_MAX;
        vertexNumber = 0;

        for(size_t i = 0; i < headRoad.NumberOfVertices() - 1; i++) {

            const Vertex& edge1 = headRoad.GetVertex(i);
            const Vertex& edge2 = headRoad.GetVertex(i+1);

            const Vertex roadPosition = CalculatePointToLineNearestPosition(
                position, edge1, edge2);

            const double distance = roadPosition.DistanceTo(position);

            if (distance < minDistance) {
                minDistance = distance;
                vertexNumber = i;
            }
        }

        while (headRoad.vertices.size() > vertexNumber + 1) {
            tailRoad.vertices.push_back(headRoad.vertices.back());
            tailRoad.commonImplPtr->vertexIds.push_back(headRoad.commonImplPtr->vertexIds.back());
            headRoad.vertices.pop_back();
            headRoad.commonImplPtr->vertexIds.pop_back();
        }

        assert(headRoad.commonImplPtr->vertexIds.size() >= 1);
        assert(tailRoad.commonImplPtr->vertexIds.size() >= 1);
        assert(headRoad.vertices.size() >= 1);
        assert(tailRoad.vertices.size() >= 1);

        if (tailRoad.commonImplPtr->vertexIds.back() != newVertexId) {
            tailRoad.commonImplPtr->vertexIds.push_back(newVertexId);
        }
        if (headRoad.commonImplPtr->vertexIds.back() != newVertexId) {
            headRoad.commonImplPtr->vertexIds.push_back(newVertexId);
        }
        if (headRoad.vertices.back() != position) {
            headRoad.vertices.push_back(position);
        }
        if (tailRoad.vertices.back() != position) {
            tailRoad.vertices.push_back(position);
        }

        assert(headRoad.vertices.size() >= 2);
        assert(tailRoad.vertices.size() >= 2);

        subsystemPtr->ConnectBidirectionalGisObject(
            headRoad.commonImplPtr->vertexIds.front(), GIS_ROAD,
            headRoad.commonImplPtr->vertexIds.back(), srcRoadId);

        subsystemPtr->ConnectBidirectionalGisObject(
            tailRoad.commonImplPtr->vertexIds.front(), GIS_ROAD,
            tailRoad.commonImplPtr->vertexIds.back(), tailRoadId);

        headRoad.UpdatePolygon();
        tailRoad.UpdatePolygon();

        subsystemPtr->RegisterGisObject(headRoad, srcRoadId);
        subsystemPtr->RegisterGisObject(tailRoad, tailRoadId);

        headRoad.capacity = capacity;
        headRoad.humanCapacity =
            static_cast<int>(
                std::min<double>(INT_MAX, std::ceil(capacity*CalculatePolygonSize(headRoad.polygon))));

        tailRoad.capacity = capacity;
        tailRoad.humanCapacity =
            static_cast<int>(
                std::min<double>(INT_MAX, std::ceil(capacity*CalculatePolygonSize(tailRoad.polygon))));

        assert(headRoad.humanCapacity >= 0);

        assert(tailRoad.humanCapacity >= 0);

        (*this).CreateIntersectionIfNecessary(position);
    }
}

void RoadLayer::MakeDirectPathToPoi(
    const RoadIdType& srcRoadId,
    const Vertex& position,
    RoadIdType& pathRoadId,
    VertexIdType& intersectionVertexId)
{
    Vertex nearestPosition;
    size_t vertexNumber = 0;

    Road& srcRoad = roads.at(srcRoadId);

    CalculatePointToHorizontalArcNearestPosition(
        srcRoad.vertices, position, nearestPosition, vertexNumber);

    const double srcRoadWidth = srcRoad.widthMeters;
    const double srcCapacity = srcRoad.capacity;

    bool found;
    IntersectionIdType intersectionId;

    (*this).FindIntersectionAt(nearestPosition, srcRoadWidth/2, found, intersectionId);

    if (found) {

        intersectionVertexId = intersections.at(intersectionId).GetVertexId();

    } else {

        intersectionVertexId = subsystemPtr->GetVertexId(nearestPosition);

        (*this).DivideRoad(srcRoadId, intersectionVertexId);
    }

    const VertexIdType parkingVertexId = subsystemPtr->GetVertexId(position);

    pathRoadId = (*this).AddSimplePath(intersectionVertexId, parkingVertexId, ROAD_EXTRA_ROAD, srcRoadWidth, srcCapacity);

    (*this).CreateParking(parkingVertexId, roads.at(srcRoadId).GetObjectId());

    (*this).CreateIntersectionIfNecessary(position);
}

void RoadLayer::AddBusStopVertex(
    const BusStopIdType& busStopId,
    const VertexIdType& vertexId)
{
    const BusStop& busStop = busStops.at(busStopId);

    if (!(*this).IsParking(vertexId)) {
        (*this).CreateParking(vertexId, subsystemPtr->CreateNewObjectId());
    }

    subsystemPtr->ConnectGisObject(vertexId, GIS_BUSSTOP, busStopId);

    busStop.commonImplPtr->vertexIds.push_back(vertexId);
}

bool RoadLayer::IsParking(const VertexIdType& vertexId) const
{
    const GisVertex& parkingGisVertex = subsystemPtr->GetGisVertex(vertexId);
    const vector<RoadIdType>& roadIds = parkingGisVertex.GetConnectedObjectIds(GIS_ROAD);

    for(size_t i = 0; i < roadIds.size(); i++) {

        if (roads[roadIds[i]].IsParking()) {
            return true;
        }
    }

    return false;
}

void RoadLayer::CreateParking(const VertexIdType& vertexId, const GisObjectIdType& objectId)
{
    if ((*this).IsParking(vertexId)) {
        return;
    }

    const RoadIdType tailRoadId = static_cast<RoadIdType>(roads.size());

    roads.push_back(Road(subsystemPtr, objectId, tailRoadId));

    Road& parkingRoad = roads.back();

    parkingRoad.commonImplPtr->vertexIds.push_back(vertexId);
    parkingRoad.type = ROAD_ROAD;

    subsystemPtr->ConnectGisObject(vertexId, GIS_ROAD, tailRoadId);
}

RoadIdType RoadLayer::AddSimplePath(
    const VertexIdType& vertexId1,
    const VertexIdType& vertexId2,
    const RoadType& roadType,
    const double widthMeters,
    const double capacity)
{
    if (vertexId1 == vertexId2) {
        return INVALID_VARIANT_ID;
    }

    const Vertex& position1 = subsystemPtr->GetVertex(vertexId1);
    const Vertex& position2 = subsystemPtr->GetVertex(vertexId2);

    const RoadIdType roadId = static_cast<RoadIdType>(roads.size());

    roads.push_back(Road(subsystemPtr, subsystemPtr->CreateNewObjectId(), roadId));

    Road& road = roads.back();

    road.commonImplPtr->vertexIds.push_back(vertexId1);
    road.commonImplPtr->vertexIds.push_back(vertexId2);

    road.vertices.push_back(position1);
    road.vertices.push_back(position2);
    road.type = roadType;
    road.widthMeters = widthMeters;

    road.UpdatePolygon();

    road.capacity = capacity;
    road.humanCapacity =
        static_cast<int>(
            std::min<double>(INT_MAX, std::ceil(capacity*CalculatePolygonSize(road.polygon))));

    assert(road.humanCapacity >= 0);

    subsystemPtr->ConnectBidirectionalGisObject(
        vertexId1, GIS_ROAD, vertexId2, roadId);

    subsystemPtr->RegisterGisObject(road, roadId);

    (*this).CreateIntersectionIfNecessary(position1);
    (*this).CreateIntersectionIfNecessary(position2);

    return roadId;
}

bool RoadLayer::ContainsBusStop(const string& name) const
{
    for(size_t i = 0; i < busStops.size(); i++) {
        if (busStops[i].GetObjectName() == name) {
            return true;
        }
    }

    return false;
}

BusLineIdType RoadLayer::GetBusLineId(const string& lineName) const
{
    typedef map<string, BusLineIdType>::const_iterator IterType;

    IterType iter = busLineIds.find(lineName);

    if (iter == busLineIds.end()) {
        cerr << "Error: Couldn't find bus line: " << lineName << endl;
        exit(1);
    }

    return (*iter).second;
}

BusStopIdType RoadLayer::GetBusStopId(const string& name) const
{
    for(size_t i = 0; i < busStops.size(); i++) {

        if (busStops[i].GetObjectName() == name) {
            return BusStopIdType(i);
        }
    }

    cerr << "Couldn't find bus stop " << name << endl;
    assert(false);
    return INVALID_VARIANT_ID;
}

void RoadLayer::GetRoadIdsAt(
    const Vertex& position,
    vector<RoadIdType>& roadIds) const
{
    roadIds.clear();

    const Rectangle searchRect(
        position.x - maxRoadWidthMeters,
        position.y - maxRoadWidthMeters,
        position.x + maxRoadWidthMeters,
        position.y + maxRoadWidthMeters);

    list<RoadIdType> searchedRoadIds;

    subsystemPtr->GetSpatialIntersectedGisObjectIds(searchRect, GIS_ROAD, searchedRoadIds);

    typedef list<RoadIdType>::const_iterator IterType;

    for(IterType iter = searchedRoadIds.begin(); iter != searchedRoadIds.end(); iter++) {

        const Road& aRoad = roads[*iter];

        if (aRoad.Contains(position)) {
            roadIds.push_back(*iter);
        }
    }
}

void RoadLayer::SearchNearestIntersectionId(
    const Vertex& position,
    const Rectangle& searchRect,
    const set<RoadType>& availableRoadTypes,
    IntersectionIdType& nearestIntersectionId,
    bool& success) const
{
    list<IntersectionIdType> intersectionIds;

    subsystemPtr->GetSpatialIntersectedGisObjectIds(searchRect, GIS_INTERSECTION, intersectionIds);

    typedef list<IntersectionIdType>::const_iterator IterType;

    success = false;

    double minDistance = DBL_MAX;

    for(IterType iter = intersectionIds.begin(); iter != intersectionIds.end(); iter++) {

        const IntersectionIdType& intersectionId = (*iter);
        const Intersection& intersection = intersections[intersectionId];

        if (!searchRect.Contains(intersection.GetVertex())) {
            continue;
        }

        const vector<RoadIdType> roadIds = intersection.GetConnectedRoadIds();

        for(size_t i = 0; i < roadIds.size(); i++) {
            const Road& road = roads[roadIds[i]];

            if (availableRoadTypes.empty() ||
                availableRoadTypes.find(road.GetRoadType()) != availableRoadTypes.end()) {

                const double distance = intersection.GetVertex().DistanceTo(position);

                if (distance < minDistance) {
                    minDistance = distance;
                    nearestIntersectionId = intersectionId;
                    success = true;
                }
            }
        }
    }
}

void RoadLayer::FindRoadAt(
    const Vertex& position,
    bool& found,
    RoadIdType& roadId) const
{
    const double findLength = 0.1;
    const double expandLength = 100;
    const int maxRetryCount = 10;

    found = false;

    for(int i = 0; i < maxRetryCount; i++) {
        list<RoadIdType> roadIds;

        subsystemPtr->GetSpatialIntersectedGisObjectIds(
            Rectangle(position, findLength + i*expandLength), GIS_ROAD, roadIds);

        typedef list<RoadIdType>::const_iterator IterType;

        if (roadIds.empty()) {
            continue;
        }

        double mindistance = DBL_MAX;

        for(IterType iter = roadIds.begin(); iter != roadIds.end(); iter++) {

            const Road& aRoad = roads[(*iter)];
            double distance = aRoad.DistanceTo(position);

            if (distance < mindistance) {
                mindistance = distance;
                roadId = (*iter);
                found = true;
            }
        }
    }
}

void RoadLayer::FindIntersectionAt(
    const Vertex& position,
    const double radius,
    bool& found,
    IntersectionIdType& foundIntersectionId) const
{
    const Rectangle searchRect(position, radius);

    list<IntersectionIdType> intersectionIds;

    typedef list<IntersectionIdType>::const_iterator IterType;

    subsystemPtr->GetSpatialIntersectedGisObjectIds(searchRect, GIS_INTERSECTION, intersectionIds);

    double minDistance = DBL_MAX;

    found = false;

    for(IterType iter = intersectionIds.begin(); iter != intersectionIds.end(); iter++) {

        const IntersectionIdType& intersectionId = (*iter);
        const Intersection& intersection = intersections[intersectionId];

        const double distance = position.DistanceTo(intersection.GetVertex());

        if (distance <= radius &&
            distance < minDistance) {
            minDistance = distance;

            found = true;
            foundIntersectionId = intersectionId;
        }
    }
}

void RoadLayer::SetIntersectionMarginAndMakeLaneConnection()
{
    if (!setIntersectionMargin) {
        return;
    }

    for(size_t i = 0; i < roads.size(); i++) {
        Road& road = roads[i];

        road.laneConnections.resize(road.numberStartToEndLanes + road.numberEndToStartLanes);
    }

    for(IntersectionIdType intersectionId = 0;
        intersectionId < IntersectionIdType(intersections.size()); intersectionId++) {

        const Intersection& intersection = intersections[intersectionId];

        const Vertex& basePoint = intersection.GetVertex();
        const vector<RoadIdType> roadIds = intersection.GetConnectedRoadIds();

        if (roadIds.size() <= 1) {
            continue;
        }

        //TBD// if (roadIds.size() <= 2) {
        //TBD//     continue;
        //TBD// }

        map<double, vector<RoadIdType> > roadIdsPerRadians;

        for(size_t j = 0; j < roadIds.size(); j++) {
            const RoadIdType roadId = roadIds[j];
            const Road& road = roads[roadId];

            const double directionRadians =
                (road.GetNeighborVertex(intersectionId) - basePoint).DirectionRadians();

            if (road.IsParking()) {
                continue;
            }

            const vector<VertexIdType>& vertexIds = road.commonImplPtr->vertexIds;

            if (vertexIds.size() == 2 &&
                vertexIds.front() == vertexIds.back()) {
                continue;
            }

            roadIdsPerRadians[directionRadians].push_back(roadId);
        }

        (*this).SetIntersectionMargin(intersectionId, roadIdsPerRadians);

        (*this).MakeLaneConnection(intersectionId);
    }
}

// -PI to PI
static inline
double CalculateFullRadiansBetweenVector(
    const Vertex& vertex1,
    const Vertex& vertex2)
{
    return (std::atan2(vertex2.y - vertex1.y, vertex2.x - vertex1.x));
}

struct RoadConnectionInfo {
    RoadIdType roadId;
    int prevMainRoadNumber;
    int nextMainRoadNumber;

    bool isExtraPath;

    RoadConnectionInfo(
        const RoadIdType& initRoadId,
        const int initPrevMainRoadNumber,
        const int initNextMainRoadNumber,
        const bool initIsExtraPath)
        :
        roadId(initRoadId),
        prevMainRoadNumber(initPrevMainRoadNumber),
        nextMainRoadNumber(initNextMainRoadNumber),
        isExtraPath(initIsExtraPath)
    {}
};


void RoadLayer::SetIntersectionMargin(
    const IntersectionIdType& intersectionId,
    const map<double, vector<RoadIdType> >& roadIdsPerRadians)
{
    Intersection& intersection = intersections[intersectionId];

    intersection.hasIntersectionPolygon = true;
    intersection.antiClockwiseRoadsides.clear();
    intersection.roadsideNumberPerRoadId.clear();

    if (roadIdsPerRadians.size() >= 2) {
        typedef map<double, vector<RoadIdType> >::const_iterator IterType;

        const Vertex& vertex = intersection.GetVertex();

        size_t i = 0;

        deque<RoadIdType> antiClockwiseMainRoadIds;
        deque<RoadConnectionInfo> antiClockwiseRoadInfos;

        for(IterType iter = roadIdsPerRadians.begin();
            iter != roadIdsPerRadians.end(); iter++, i++) {

            const vector<RoadIdType>& roadIds = (*iter).second;
            assert(!roadIds.empty());

            double maxRoadWidthMeters = 0;
            RoadIdType wideRoadId = roadIds.front();

            if (roadIds.size() > 2) {
                for(size_t j = 0; j < roadIds.size(); j++) {
                    const RoadIdType& roadId = roadIds[j];

                    const Road& road = roads[roadId];
                    const double roadWidthMeters = road.GetRoadWidthMeters();

                    if (roadWidthMeters > maxRoadWidthMeters) {
                        maxRoadWidthMeters = roadWidthMeters;
                        wideRoadId = roadId;
                    }
                }
            }

            const Road& road = roads.at(wideRoadId);
            const int prevRoadNumber = int(antiClockwiseMainRoadIds.size()) - 1;
            const bool isExtraPath = road.IsExtraPath();

            if (!isExtraPath) {

                antiClockwiseMainRoadIds.push_back(wideRoadId);
                antiClockwiseRoadInfos.push_back(RoadConnectionInfo(wideRoadId, prevRoadNumber, (int)(antiClockwiseMainRoadIds.size()), isExtraPath));

            } else {
                antiClockwiseRoadInfos.push_back(RoadConnectionInfo(wideRoadId, prevRoadNumber, (int)(antiClockwiseMainRoadIds.size()), isExtraPath));

            }
        }

        if (antiClockwiseMainRoadIds.empty()) {
            return;
        }

        vector<double> margins;

        i = 0;
        for(IterType iter = roadIdsPerRadians.begin();
            iter != roadIdsPerRadians.end(); iter++, i++) {

            const vector<RoadIdType>& roadIds = (*iter).second;
            const RoadConnectionInfo& roadInfo = antiClockwiseRoadInfos[i];

            int prevMainRoadNumber = roadInfo.prevMainRoadNumber;
            if (prevMainRoadNumber < 0) {
                prevMainRoadNumber += (int)(antiClockwiseMainRoadIds.size());
            } else if (prevMainRoadNumber >= (int)(antiClockwiseMainRoadIds.size())) {
                prevMainRoadNumber -= (int)(antiClockwiseMainRoadIds.size());
            }
            int nextMainRoadNumber = roadInfo.nextMainRoadNumber;
            if (nextMainRoadNumber < 0) {
                nextMainRoadNumber += (int)(antiClockwiseMainRoadIds.size());
            } else if (nextMainRoadNumber >= (int)(antiClockwiseMainRoadIds.size())) {
                nextMainRoadNumber -= (int)(antiClockwiseMainRoadIds.size());
            }

            const RoadIdType& prevRoadId = antiClockwiseMainRoadIds.at(prevMainRoadNumber);
            const RoadIdType& nextRoadId = antiClockwiseMainRoadIds.at(nextMainRoadNumber);

            const Road& prevRoad = roads.at(prevRoadId);
            const Road& road = roads.at(roadInfo.roadId);
            const Road& nextRoad = roads.at(nextRoadId);

            const double prevRoadWidth =
                (prevRoad.GetRoadWidthMeters() + road.GetRoadWidthMeters()) / 2.;

            const double nextRoadWidth =
                (nextRoad.GetRoadWidthMeters() + road.GetRoadWidthMeters()) / 2.;

            const pair<Vertex, Vertex> prevLine =
                prevRoad.GetSideLineToIntersection(intersectionId, -prevRoadWidth);

            const pair<Vertex, Vertex> line1 =
                road.GetSideLineToIntersection(intersectionId, prevRoadWidth);

            const pair<Vertex, Vertex> line2 =
                road.GetSideLineToIntersection(intersectionId, -nextRoadWidth);

            const pair<Vertex, Vertex> nextLine =
                nextRoad.GetSideLineToIntersection(intersectionId, nextRoadWidth);

            Vertex leftEdge;
            Vertex rightEdge;

            // vertex integration
            const double nearDistanceMeters = 0.1;

            if ((prevLine.first.DistanceTo(line1.first) > nearDistanceMeters) &&
                HorizontalLinesAreIntersection(prevLine.first, prevLine.second, line1.first, line1.second)) {

                leftEdge = CalculateIntersectionPositionBetweenLine(
                    prevLine.first, prevLine.second, line1.first, line1.second);
            } else {
                leftEdge = line1.first;
            }

            if ((nextLine.first.DistanceTo(line2.first) > nearDistanceMeters) &&
                HorizontalLinesAreIntersection(line2.first, line2.second, nextLine.first, nextLine.second)) {

                rightEdge = CalculateIntersectionPositionBetweenLine(
                    line2.first, line2.second, nextLine.first, nextLine.second);
            } else {
                rightEdge = line2.first;
            }

            if (leftEdge.DistanceTo(line1.first) > rightEdge.DistanceTo(line2.first)) {

                rightEdge = line2.first - (line1.first - leftEdge);

            } else {

                leftEdge = line1.first - (line2.first - rightEdge);
            }

            margins.push_back(leftEdge.DistanceTo(line1.first));

            for(size_t j = 0; j < roadIds.size(); j++) {
                const RoadIdType& roadId = roadIds[j];

                intersection.roadsideNumberPerRoadId[roadId] =
                    intersection.antiClockwiseRoadsides.size();
            }

            intersection.antiClockwiseRoadsides.push_back(
                Intersection::Roadside(leftEdge, rightEdge));
        }


        i = 0;
        for(IterType iter = roadIdsPerRadians.begin();
            iter != roadIdsPerRadians.end(); iter++, i++) {

            const double maxMargin = margins[i];
            const vector<RoadIdType>& roadIds = (*iter).second;

            for(size_t j = 0; j < roadIds.size(); j++) {
                const RoadIdType& roadId = roadIds[j];
                Road& road = roads[roadId];

                const double marginLength =
                    std::min(maxMargin, std::max(0., (road.GetArcDistanceMeters() - maxMargin)));

                if (marginLength > 0) {
                    road.SetIntersectionMargin(intersectionId, marginLength);
                }
            }
        }
    }
}

static inline
RoadTurnDirectionType CalculateDirectionFromRadians(const double radians)
{
    return static_cast<RoadTurnDirectionType>(
        std::max<double>(0, ((radians) - (PI/4.)) / (PI/2.)));
}

void RoadLayer::MakeLaneConnection(const IntersectionIdType& intersectionId)
{
    Intersection& intersection = intersections[intersectionId];

    const vector<RoadIdType> roadIds = intersection.GetConnectedRoadIds();

    const VertexIdType intersectionVertexId = intersection.GetVertexId();
    const Vertex& origVertex = intersection.GetVertex();

    map<pair<RoadIdType, RoadIdType>, RoadTurnType>& roadTurnTypes = intersection.roadTurnTypes;

    for(size_t i = 0; i < roadIds.size(); i++) {
        const RoadIdType roadId1 = roadIds[i];
        Road& road1 = roads[roadId1];

        const bool hasRoad1OutgoingLane =
            road1.HasOutgoingLane(intersectionVertexId);

        const bool hasRoad1IncomingLane =
            road1.HasIncomingLane(intersectionVertexId);

        roadTurnTypes[make_pair(roadId1, roadId1)] =
            RoadTurnType(ROAD_TURN_BACK, road1.GetRoadWidthMeters()*2, hasRoad1OutgoingLane && hasRoad1IncomingLane);

        road1.MakeTurnaroundLaneConnection();

        if (roadIds.size() == 1) {
            continue;
        }

        const double roadWidth1 = road1.GetRoadWidthMeters();

        for(size_t j = i+1; j < roadIds.size(); j++) {
            const RoadIdType roadId2 = roadIds[j];

            Road& road2 = roads[roadId2];
            const Vertex& vertex1 = road1.GetNeighborVertex(intersectionId);
            const Vertex& vertex2 = road2.GetNeighborVertex(intersectionId);
            const double totalRoadWidth = roadWidth1 + road2.GetRoadWidthMeters();

            const bool hasRoad2OutgoingLane =
                road2.HasOutgoingLane(intersectionVertexId);

            const bool hasRoad2IncomingLane =
                road2.HasIncomingLane(intersectionVertexId);

            const RoadTurnDirectionType direction12 =
                CalculateDirectionFromRadians(
                    CalculateFullRadiansBetweenVector(vertex1, origVertex, vertex2));

            const RoadTurnDirectionType direction21 =
                CalculateDirectionFromRadians(
                    CalculateFullRadiansBetweenVector(vertex2, origVertex, vertex1));

            road1.MakeLaneConnection(road2, intersectionVertexId, direction12);
            road2.MakeLaneConnection(road1, intersectionVertexId, direction21);

            roadTurnTypes[make_pair(roadId1, roadId2)] =
                RoadTurnType(direction12, totalRoadWidth, hasRoad1OutgoingLane && hasRoad2IncomingLane);
            roadTurnTypes[make_pair(roadId2, roadId1)] =
                RoadTurnType(direction21, totalRoadWidth, hasRoad2OutgoingLane && hasRoad1IncomingLane);
        }
    }
}

void RoadLayer::GetIntersectionIds(
    const Rectangle& searchRect,
    const set<RoadType>& availableRoadTypes, //throw empty to enable all road type.
    list<IntersectionIdType>& foundIntersectionIds) const
{
    foundIntersectionIds.clear();

    list<IntersectionIdType> intersectionIds;

    typedef list<IntersectionIdType>::const_iterator IterType;

    subsystemPtr->GetSpatialIntersectedGisObjectIds(searchRect, GIS_INTERSECTION, intersectionIds);

    for(IterType iter = intersectionIds.begin(); iter != intersectionIds.end(); iter++) {

        const IntersectionIdType& intersectionId = (*iter);
        const Intersection& intersection = intersections[intersectionId];

        if (searchRect.Contains(intersection.GetVertex())) {
            foundIntersectionIds.push_back(intersectionId);
        }
    }
}

void RoadLayer::CreateIntersection(
    const VertexIdType& vertexId,
    const GisObjectIdType& objectId)
{
    const IntersectionIdType intersectionId =
        static_cast<IntersectionIdType>(intersections.size());

    intersections.push_back(Intersection(subsystemPtr, this, objectId, intersectionId, vertexId));
    subsystemPtr->ConnectGisObject(vertexId, GIS_INTERSECTION, intersectionId);
    subsystemPtr->RegisterGisObject(intersections.back(), intersectionId);
}

void GisSubsystem::CreatePedestrianPathIfPossible(
    const VertexIdType& vertexId1,
    const VertexIdType& vertexId2)
{
    if (vertexId1 == vertexId2) {
        return;
    }

    const GisObjectIdType objectId = (*this).CreateNewObjectId();
    const PedestrianPathIdType pathId = static_cast<PedestrianPathIdType>(pedestrianPaths.size());

    pedestrianPaths.push_back(PedestrianPath(this, objectId, pathId));

    PedestrianPath& pedestrianPath = pedestrianPaths.back();
    vector<VertexIdType>& pathVertexIds = pedestrianPath.commonImplPtr->vertexIds;

    pathVertexIds.push_back(vertexId1);
    pathVertexIds.push_back(vertexId2);

    (*this).ConnectBidirectionalGisObject(
        vertexId1, GIS_PEDESTRIAN_PATH, vertexId2, pathId);

    (*this).RegisterGisObject(pedestrianPath, pathId);
}

string RoadLayer::GetLineName(const RailRoadLineIdType& lineId) const
{
    typedef map<string, BusLineIdType>::const_iterator IterType;

    for(IterType iter = busLineIds.begin(); iter != busLineIds.end(); iter++) {
        if ((*iter).second == lineId) {
            return (*iter).first;
        }
    }

    return string();
}

void RoadLayer::AssignBusLine(
    const string& lineName,
    const deque<string>& busStopNames)
{
    if (busStopNames.empty()) {
        return;
    }

    if (busLineIds.find(lineName) == busLineIds.end()) {
        busLineIds[lineName] = (BusLineIdType)(routeInfosPerLine.size());
        routeInfosPerLine.push_back(vector<RouteInfo>());
    }

    RailRoadLineIdType lineId = busLineIds[lineName];
    deque<BusStopIdType> busStopIds;
    for(size_t i = 0; i < busStopNames.size(); i++) {
        busStopIds.push_back((*this).GetBusStopId(busStopNames[i]));
    }

    vector<RouteInfo>& routeInfos = routeInfosPerLine.at(lineId);

    for(size_t i = 0; i < routeInfos.size(); i++) {
        if (routeInfos[i].busStopIds == busStopIds) {
            cerr << "Error: Already assigned a bus route for line: " << endl
                 << (*this).GetLineName(lineId);

            for(size_t i = 0; i < busStopIds.size(); i++) {
                cerr << "," << (*this).GetBusStop(busStopIds[i]).GetObjectName();
            }
            cerr << endl;

            exit(1);
        }
    }

    routeInfos.push_back(RouteInfo());
    RouteInfo& routeInfo = routeInfos.back();
    routeInfo.busStopIds = busStopIds;
}

RouteIdType RoadLayer::GetRouteId(
    const BusLineIdType& lineId,
    const deque<BusStopIdType>& busStopIds) const
{
    const vector<RouteInfo>& routeInfos = routeInfosPerLine.at(lineId);
    bool found = false;
    RouteIdType routeId;

    for(routeId = 0; routeId < RouteIdType(routeInfos.size()); routeId++) {
        found = (routeInfos[routeId].busStopIds == busStopIds);

        if (found) {
            break;
        }
    }
    if (!found) {
        cerr << "Error: No bus route for line: " << (*this).GetLineName(lineId) << endl;
        exit(1);
    }

    return routeId;
}

const deque<BusStopIdType>& RoadLayer::GetRouteBusStopIds(
    const BusLineIdType& lineId,
    const RouteIdType& routeId) const
{
    return routeInfosPerLine.at(lineId).at(routeId).busStopIds;
}

//-------------------------------------------------------------

void RailRoadLayer::ImportRailRoad(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);

    map<VertexIdType, GisObjectIdType> intersectionObjectIds;
    map<GisObjectIdType, pair<string, set<VertexIdType> > > verticesPerStation;

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const RailRoadIdType railRoadId = static_cast<RailRoadIdType>(railRoads.size());
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        railRoads.push_back(RailRoad(subsystemPtr, this, objectId, railRoadId));
        RailRoad& railRoad = railRoads.back();

        railRoad.LoadParameters(theParameterDatabaseReader);

        if (nameFinder.IsAvailable()) {
            railRoad.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        vector<VertexIdType>& vertexIds = railRoad.commonImplPtr->vertexIds;

        if (shpObjPtr->nVertices > 0) {
            railRoad.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
        }

        for(int i = 0; i < shpObjPtr->nVertices; i++) {
            const double x = shpObjPtr->padfX[i];
            const double y = shpObjPtr->padfY[i];
            double z = shpObjPtr->padfZ[i];

            if (isBaseGroundLevel) {
                z += subsystemPtr->GetGroundElevationMetersAt(x, y);
            }

            const Vertex point(x, y, z);
            const RailVertexIdType railVertexId = (*this).GetNewOrExistingRailVertexId(point);

            railRoad.vertices.push_back(make_pair(railVertexId, INVALID_VARIANT_ID));
        }

        const VertexIdType startVertexId =
            subsystemPtr->GetVertexId(railVertices.at(railRoad.vertices.front().first).vertex);

        const VertexIdType endVertexId =
            subsystemPtr->GetVertexId(railVertices.at(railRoad.vertices.back().first).vertex);

        vertexIds.push_back(startVertexId);
        vertexIds.push_back(endVertexId);

        // not linear connection

        subsystemPtr->ConnectBidirectionalGisObject(
            startVertexId, GIS_RAILROAD, endVertexId, railRoadId);

        subsystemPtr->RegisterGisObject(railRoad, railRoadId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

RailVertexIdType RailRoadLayer::GetNewOrExistingRailVertexId(const Vertex& point)
{
    if (railVertexIds.find(point) == railVertexIds.end()) {
        railVertexIds[point] = (RailVertexIdType)(railVertices.size());
        railVertices.push_back(point);
    }

    return railVertexIds[point];
}

void RailRoadLayer::ImportStation(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder capacityFinder(hDBF, GIS_DBF_CAPACITY_STRING);

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const RailRoadStationIdType stationId = static_cast<RailRoadStationIdType>(stations.size());
        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        stations.push_back(RailRoadStation(subsystemPtr, objectId, stationId));
        RailRoadStation& station = stations.back();
        station.LoadParameters(theParameterDatabaseReader);

        if (nameFinder.IsAvailable()) {
            station.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }
        if (capacityFinder.IsAvailable()) {
            station.humanCapacity = capacityFinder.GetInt(entryId);
        }

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        if (shpObjPtr->nVertices > 0) {
            station.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
        }

        for(int i = 0; i < shpObjPtr->nVertices; i++) {
            const double x = shpObjPtr->padfX[i];
            const double y = shpObjPtr->padfY[i];
            double z = shpObjPtr->padfZ[i];

            if (isBaseGroundLevel) {
                z += subsystemPtr->GetGroundElevationMetersAt(x, y);
            }

            station.polygon.push_back(Vertex(x, y, z));
        }

        station.UpdateMinRectangle();

        station.centerPosition = station.commonImplPtr->minRectangle.GetCenter();
        station.centerPosition.z =
            station.commonImplPtr->elevationFromGroundMeters +
            subsystemPtr->GetGroundElevationMetersAt(station.centerPosition);

        subsystemPtr->RegisterGisObject(station, stationId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

static inline
void CalculateRailRoadMatching(
    const list<RailRoadIdType>& origStationRailRoadIds,
    const list<RailRoadIdType>& otherStationRailIds,
    bool& isMatch,
    bool& isReverse)
{
    isMatch = false;
    isReverse = false;

    if (origStationRailRoadIds.empty() ||
        origStationRailRoadIds.size() != origStationRailRoadIds.size()) {
        return;
    }

    const size_t numberRailRoads = origStationRailRoadIds.size();

    list<RailRoadIdType> origRailRoadIds = origStationRailRoadIds;
    list<RailRoadIdType> reverseOrigRailRoadIds(origStationRailRoadIds.rbegin(), origStationRailRoadIds.rend());
    list<RailRoadIdType> railRoadIds2 = otherStationRailIds;

    for(size_t i = 0; i < numberRailRoads; i++) {

        if (origRailRoadIds == otherStationRailIds) {
            isMatch = true;
            isReverse = false;
            return;
        }
        if (reverseOrigRailRoadIds == otherStationRailIds) {
            isMatch = true;
            isReverse = true;
            return;
        }

        if (i < numberRailRoads - 1) {
            origRailRoadIds.splice(origRailRoadIds.end(), origRailRoadIds, origRailRoadIds.begin());
            reverseOrigRailRoadIds.splice(reverseOrigRailRoadIds.end(), reverseOrigRailRoadIds, reverseOrigRailRoadIds.begin());
        }
    }
}

void RailRoadLayer::GetShortestRailroadPath(
    const vector<vector<RailRoadStationIdType> >& stationIdsPerName,
    deque<RailRoadStationIdType>& bestStationIds,
    vector<pair<deque<RailLink>, double> >& bestSections,
    set<pair<RailRoadStationIdType, RailRoadStationIdType> >& noConnections,
    map<pair<RailRoadStationIdType, RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > >& sectionCandidatesCache)
{
    size_t numberStationPatterns = 1;

    for(size_t i = 0; i < stationIdsPerName.size(); i++) {
        numberStationPatterns *= stationIdsPerName[i].size();
    }

    double minRunningDistance = DBL_MAX;

    vector<size_t> bestCandidateNumbers;
    vector<vector<pair<deque<RailLink>, double> > > bestSectionCandidatesPerSection;

    bestStationIds.clear();
    bestSectionCandidatesPerSection.clear();
    noConnections.clear();

    for(size_t stationPatternId = 0; stationPatternId < numberStationPatterns; stationPatternId++) {

        size_t remain = stationPatternId;

        deque<RailRoadStationIdType> stationIds;

        for(size_t i = 0; i < stationIdsPerName.size(); i++) {
            const vector<RailRoadStationIdType>& stationIdCandidates = stationIdsPerName[i];

            stationIds.push_back(stationIdCandidates[remain % stationIdCandidates.size()]);

            assert(!stationIdCandidates.empty());
            remain /= stationIdCandidates.size();
        }

        typedef list<RailRoadIdType>::const_iterator RailIdIter;

        vector<vector<pair<deque<RailLink>, double> > > sectionCandidatesPerSection(stationIds.size() - 1);

        for(size_t i = 0; i < stationIds.size() - 1; i++) {

            const RailRoadStationIdType& stationId1 = stationIds[i];
            const RailRoadStationIdType& stationId2 = stationIds[i+1];

            const pair<RailRoadStationIdType, RailRoadStationIdType> stationIdPair(stationId1, stationId2);

            if (sectionCandidatesCache.find(stationIdPair) == sectionCandidatesCache.end()) {
                const RailRoadStation& station1 = stations[stationId1];
                const RailRoadStation& station2 = stations[stationId2];

                const Vertex& position1 = station1.GetVertex();
                const Vertex& position2 = station2.GetVertex();

                const set<RailVertexIdType>& railVertexIds1 = station1.railVertexIds;
                const set<RailVertexIdType>& railVertexIds2 = station2.railVertexIds;

                typedef set<RailVertexIdType>::const_iterator IterType;

                vector<pair<deque<RailLink>, double> >& sectionCandidates = sectionCandidatesCache[stationIdPair];

                for(IterType iter1 = railVertexIds1.begin(); iter1 != railVertexIds1.end(); iter1++) {

                    const RailVertexIdType& railVertexId1 = (*iter1);
                    const double distance1 = position1.DistanceTo(railVertices.at(railVertexId1).vertex);

                    for(IterType iter2 = railVertexIds2.begin(); iter2 != railVertexIds2.end(); iter2++) {

                        const RailVertexIdType& railVertexId2 = (*iter2);
                        const double distance2 = position2.DistanceTo(railVertices.at(railVertexId2).vertex);

                        deque<RailLink> routeLinks;

                        (*this).SearchRailRoadRoute(railVertexId1, railVertexId2, routeLinks);

                        if (!routeLinks.empty()) {
                            sectionCandidates.push_back(make_pair(routeLinks, distance1+distance2));
                        }
                    }
                }
            }

            sectionCandidatesPerSection[i] = sectionCandidatesCache[stationIdPair];
        }

        size_t minCandidatesSectionNumber = 0;
        size_t minNumberCandidates = INT_MAX;

        bool sectionsAreConnected = true;

        for(size_t i = 0; i < sectionCandidatesPerSection.size(); i++) {
            const vector<pair<deque<RailLink>, double> >& sectionCandidates =
                sectionCandidatesPerSection[i];

            const size_t numberCandidates = sectionCandidates.size();

            if (numberCandidates == 0) {
                sectionsAreConnected = false;
                noConnections.insert(
                    make_pair(
                        stationIdsPerName[i].front(),
                        stationIdsPerName[i+1].front()));
            }

            if (numberCandidates > 0 &&
                numberCandidates < minNumberCandidates) {
                minNumberCandidates = numberCandidates;
                minCandidatesSectionNumber = i;
            }
        }

        if (!sectionsAreConnected) {
            continue;
        }

        vector<size_t> currentCandidateNumbers(sectionCandidatesPerSection.size(), 0);

        const vector<pair<deque<RailLink>, double> >& mainSectionCandidates =
            sectionCandidatesPerSection[minCandidatesSectionNumber];

        for(size_t i = 0; i < mainSectionCandidates.size(); i++) {

            currentCandidateNumbers[minCandidatesSectionNumber] = i;

            for(int j = int(minCandidatesSectionNumber) - 1; j >= 0; j--) {
                const pair<deque<RailLink>, double>& lastSection =
                    sectionCandidatesPerSection.at(j+1).at(currentCandidateNumbers.at(j+1));
                const Vertex& frontVertex = railVertices.at(lastSection.first.front().railVertexId).vertex;

                const vector<pair<deque<RailLink>, double> >& sectionCandidates =
                    sectionCandidatesPerSection[j];

                double minDistanceBetweenSections = DBL_MAX;

                for(size_t k = 0; k < sectionCandidates.size(); k++) {
                    const pair<deque<RailLink>, double>& section = sectionCandidates[k];
                    const Vertex& lastVertex = railVertices.at(section.first.back().railVertexId).vertex;
                    const double distance = lastVertex.DistanceTo(frontVertex) + (*this).CalculateDistance(section);

                    const Vertex& lastVertex2 = railVertices.at(section.first.front().railVertexId).vertex;

                    if (distance < minDistanceBetweenSections) {
                        minDistanceBetweenSections = distance;
                        currentCandidateNumbers[j] = k;
                    }
                }
            }

            for(size_t j = minCandidatesSectionNumber + 1; j < sectionCandidatesPerSection.size(); j++) {
                const pair<deque<RailLink>, double>& lastSection =
                    sectionCandidatesPerSection.at(j-1).at(currentCandidateNumbers.at(j-1));
                const Vertex& lastVertex = railVertices.at(lastSection.first.back().railVertexId).vertex;

                const vector<pair<deque<RailLink>, double> >& sectionCandidates =
                    sectionCandidatesPerSection[j];

                double minDistanceBetweenSections = DBL_MAX;

                for(size_t k = 0; k < sectionCandidates.size(); k++) {
                    const pair<deque<RailLink>, double>& section = sectionCandidates[k];
                    const Vertex& frontVertex = railVertices.at(section.first.front().railVertexId).vertex;
                    const double distance = frontVertex.DistanceTo(lastVertex) + (*this).CalculateDistance(section);

                    if (distance < minDistanceBetweenSections) {
                        minDistanceBetweenSections = distance;
                        currentCandidateNumbers[j] = k;
                    }
                }
            }

            double totalRunnningDistance = 0;

            for(size_t j = 0; j < currentCandidateNumbers.size(); j++) {
                const size_t candidateNumber = currentCandidateNumbers[j];
                const pair<deque<RailLink>, double>& sectionCandidate =
                    sectionCandidatesPerSection[j].at(candidateNumber);
                const deque<RailLink>& railVertexIds = sectionCandidate.first;

                assert(railVertexIds.size() > 0);

                totalRunnningDistance += (*this).CalculateDistance(sectionCandidate) + sectionCandidate.second;
            }

            if (totalRunnningDistance < minRunningDistance) {
                minRunningDistance = totalRunnningDistance;
                bestCandidateNumbers = currentCandidateNumbers;
                bestSectionCandidatesPerSection = sectionCandidatesPerSection;
                bestStationIds = stationIds;
            }
        }
    }

    if (!bestCandidateNumbers.empty()) {
        noConnections.clear();

        for(size_t i = 0; i < bestCandidateNumbers.size(); i++) {
            const size_t bestCandidateNumber = bestCandidateNumbers[i];
            const vector<pair<deque<RailLink>, double> >& sectionCandidates =
                bestSectionCandidatesPerSection[i];

            const pair<deque<RailLink>, double>& bestSectionCandidate =
                sectionCandidates.at(bestCandidateNumber);

            bestSections.push_back(bestSectionCandidate);
        }
    }
}

void RailRoadLayer::AssignRailRoadLine(
    const string& lineName,
    const deque<string>& stationNames,
    map<pair<RailRoadStationIdType, RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > >& sectionCandidatesCache,
    map<string, pair<deque<RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > > >& stationIdsAndRailCache)
{
    if (stationNames.empty()) {
        return;
    }

    if (railRoadLineIds.find(lineName) == railRoadLineIds.end()) {
        railRoadLineIds[lineName] = (RailRoadLineIdType)(lineInfos.size());
        lineInfos.push_back(RailRoadLineInfo());
    }

    for(size_t i = 0; i < stationNames.size(); i++) {
        const string& stationName = stationNames[i];

        if (!subsystemPtr->ContainsPosition(stationName)) {
            cerr << "Warning: Skip line " << lineName << " which contains invalid station " << stationName << endl;
            return;
        }
    }


    RailRoadLineIdType lineId = railRoadLineIds[lineName];

    RailRoadLineInfo& lineInfo = lineInfos.at(lineId);
    vector<RouteInfo>& routeInfos = lineInfo.routeInfos;

    const RouteIdType routeId = RouteIdType(routeInfos.size());

    routeInfos.push_back(RouteInfo());
    RouteInfo& routeInfo = routeInfos.back();

    deque<RailRoadStationIdType> bestStationIds;
    vector<pair<deque<RailLink>, double> > bestSections;
    set<pair<RailRoadStationIdType, RailRoadStationIdType> > noConnections;

    ostringstream reverseStationNamesKeyStream;
    ostringstream stationNamesKeyStream;

    for(size_t i = 0; i < stationNames.size(); i++) {
        stationNamesKeyStream << stationNames[i];
        reverseStationNamesKeyStream << stationNames[stationNames.size() - i - 1];

        if (i < stationNames.size() - 1) {
            stationNamesKeyStream << ":";
            reverseStationNamesKeyStream << ":";
        }
    }

    const string stationNamesKey = stationNamesKeyStream.str();
    const string reverseStationNamesKey = reverseStationNamesKeyStream.str();

    bool isFromCache = false;

    if (stationIdsAndRailCache.find(stationNamesKey) != stationIdsAndRailCache.end()) {
        pair<deque<RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > >& stationIdsAndRailLinks = stationIdsAndRailCache[stationNamesKey];

        bestStationIds = stationIdsAndRailLinks.first;
        bestSections = stationIdsAndRailLinks.second;
        isFromCache = true;

    } else {

        vector<vector<RailRoadStationIdType> > stationIdsPerName;
        vector<vector<vector<RailRoadStationIdType> > > stationIdsPerNames;

        size_t numberStationPatterns = 1;
        size_t dividedNumberStationPatterns = 1;
        size_t totalDividedNumberStationPatterns = 0;

        bool lastStationIsAmbiguous = false;

        for(size_t i = 0; i < stationNames.size(); i++) {
            const string& stationName = stationNames[i];

            stationIdsPerName.push_back(vector<RailRoadStationIdType>());

            vector<RailRoadStationIdType>& stationIds = stationIdsPerName.back();

            (*this).GetStationIds(stationName, stationIds);

            numberStationPatterns *= stationIds.size();
            dividedNumberStationPatterns *= stationIds.size();

            if (lastStationIsAmbiguous && stationIds.size() == 1) {
                stationIdsPerNames.push_back(stationIdsPerName);

                stationIdsPerName.clear();
                stationIdsPerName.push_back(stationIdsPerNames.back().back());

                totalDividedNumberStationPatterns += dividedNumberStationPatterns;

                lastStationIsAmbiguous = false;
                dividedNumberStationPatterns = 1;
            }

            if (stationIds.size() > 1) {
                lastStationIsAmbiguous = true;
            }
        }

        if (stationIdsPerName.size() > 1) {
            stationIdsPerNames.push_back(stationIdsPerName);

            totalDividedNumberStationPatterns += dividedNumberStationPatterns;
        }

        for(size_t i = 0; i < stationIdsPerNames.size(); i++) {
            const vector<vector<RailRoadStationIdType> >& stationIdsPerNameInAPart = stationIdsPerNames[i];

            deque<RailRoadStationIdType> bestStationIdsInAPart;
            vector<pair<deque<RailLink>, double> > bestSectionsInAPart;
            set<pair<RailRoadStationIdType, RailRoadStationIdType> > noConnectionsInAPart;

            (*this).GetShortestRailroadPath(
                stationIdsPerNameInAPart,
                bestStationIdsInAPart,
                bestSectionsInAPart,
                noConnectionsInAPart,
                sectionCandidatesCache);

            if (bestStationIds.empty()) {
                bestStationIds = bestStationIdsInAPart;
            } else {
                for(size_t j = 1; j < bestStationIdsInAPart.size(); j++) {
                    bestStationIds.push_back(bestStationIdsInAPart[j]);
                }
            }

            for(size_t j = 0; j < bestSectionsInAPart.size(); j++) {
                bestSections.push_back(bestSectionsInAPart[j]);
            }

            noConnections.insert(noConnectionsInAPart.begin(), noConnectionsInAPart.end());
        }
    }

    if (!noConnections.empty()) {
        cerr << "Warning: no railroad connection candidate for line [" << lineName << "]" << endl;

        typedef set<pair<RailRoadStationIdType, RailRoadStationIdType> >::const_iterator IterType;

        for(IterType iter = noConnections.begin();
            iter != noConnections.end(); iter++) {
            cerr << "    between "
                 << stations[(*iter).first].GetObjectName() << "(" << (*iter).first << ")"
                 << " and "
                 << stations[(*iter).second].GetObjectName() << "(" << (*iter).second << ")" << endl;
        }
        return;
    }

    pair<deque<RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > >& reverseStationIdsAndRailLinks = stationIdsAndRailCache[reverseStationNamesKey];

    pair<deque<RailRoadStationIdType>, vector<pair<deque<RailLink>, double> > >& stationIdsAndRailLinks = stationIdsAndRailCache[stationNamesKey];

    routeInfo.stationIds = bestStationIds;

    assert(stationNames.size() == bestStationIds.size());

    for(size_t i = 0; i < bestStationIds.size(); i++) {
        lineInfo.railRoadStationIds[stationNames[i]] = bestStationIds[i];

        if (!isFromCache) {
            // cache
            stationIdsAndRailLinks.first.push_back(bestStationIds[i]);
            reverseStationIdsAndRailLinks.first.push_back(bestStationIds[bestStationIds.size() - i - 1]);
        }
    }

    assert(bestSections.size() == bestStationIds.size() - 1);

    for(size_t i = 0; i < bestSections.size(); i++) {
        const pair<deque<RailLink>, double>& bestSection = bestSections[i];

        if (!isFromCache) {
            // cache
            stationIdsAndRailLinks.second.push_back(bestSection);

            pair<deque<RailLink>, double> reverseSection = bestSections[bestSections.size() - i - 1];
            const deque<RailLink>& railLinks = bestSections[bestSections.size() - i - 1].first;

            reverseSection.first.clear();

            for(size_t j = 0; j < railLinks.size(); j++) {
                const RailLink& railLink = railLinks[j];

                reverseSection.first.push_front(railLink);
            }

            reverseStationIdsAndRailLinks.second.push_back(reverseSection);
        }

        deque<RailVertexIdType> railVertexIds;

        const deque<RailLink>& railLinks = bestSection.first;

        for(size_t j = 0; j < railLinks.size(); j++) {
            const RailLink& railLink = railLinks[j];

            railVertexIds.push_back(railLink.railVertexId);

            if (railLink.railId != INVALID_VARIANT_ID) {
                railRoads.at(railLink.railId).railRoadLineIds.insert(lineId);
                lineInfo.railIds.insert(railLink.railId);
            }
        }

        routeInfo.railVertexIdsPerSection.push_back(railVertexIds);

        assert(!railVertexIds.empty());
        const RailVertexIdType& frontVertexId = railVertexIds.front();
        const RailVertexIdType& backVertexId = railVertexIds.back();

        if (i == 0) {
            stations.at(bestStationIds[i]).AddRailRoadConnection(
                lineId, routeId, railVertices.at(frontVertexId).vertex);
        }

        stations.at(bestStationIds[i+1]).AddRailRoadConnection(
            lineId, routeId, railVertices.at(backVertexId).vertex);
    }
}

double RailRoadLayer::CalculateDistance(const pair<deque<RailLink>, double>& section) const
{
    const deque<RailLink>& railLinks = section.first;

    double pathDistance = 0;

    for(size_t i = 0; (i+1) < railLinks.size(); i++) {
        pathDistance +=
            railVertices.at(railLinks[i].railVertexId).vertex.
            DistanceTo(railVertices.at(railLinks[i+1].railVertexId).vertex);
    }

    return pathDistance;
}

void RailRoadLayer::ComplementLineInfo()
{
    for(RailRoadStationIdType stationId = 0; stationId < RailRoadStationIdType(stations.size()); stationId++) {
        RailRoadStation& station = stations[stationId];
        const Vertex& position = station.GetVertex();
        const vector<Vertex>& stationPolygon = station.GetPolygon();
        const Rectangle& rect = station.GetMinRectangle();
        const double nearRailRoadDistance = std::max(rect.GetWidth(), rect.GetHeight())*0.5;

        list<RailRoadIdType> railIds;

        subsystemPtr->GetSpatialIntersectedGisObjectIds(rect, GIS_RAILROAD, railIds);

        typedef list<RailRoadIdType>::const_iterator RailIdIter;

        for(RailIdIter railIdIter = railIds.begin(); railIdIter != railIds.end(); railIdIter++) {
            const RailRoadIdType& railId = (*railIdIter);

            RailRoad& rail = railRoads[railId];

            deque<pair<RailVertexIdType, RailRoadStationIdType> >& vertices = rail.vertices;

            typedef deque<pair<RailVertexIdType, RailRoadStationIdType> >::iterator VertexIter;

            VertexIter vertexIter = vertices.begin();

            Vertex nearestPoint;
            double nearestDistance = DBL_MAX;
            VertexIter nearestVertexIter = vertices.end();
            size_t nearestVertexNumber = 0;

            for(size_t i = 0; i < vertices.size() - 1; i++, vertexIter++) {
                const Vertex& vertex1 = railVertices.at(vertices[i].first).vertex;
                const Vertex& vertex2 = railVertices.at(vertices[i+1].first).vertex;

                const Vertex aNearestPoint =
                    CalculatePointToLineNearestPosition(position, vertex1, vertex2);

                const double distance = position.DistanceTo(aNearestPoint);

                if (distance <= nearestDistance) {
                    nearestDistance = distance;
                    nearestPoint = aNearestPoint;
                    nearestVertexIter = vertexIter;
                    nearestVertexNumber = i;
                }
            }

            if (nearestDistance <= nearRailRoadDistance) {
                assert(nearestVertexNumber < vertices.size());

                RailVertexIdType lastVertexId = vertices[nearestVertexNumber].first;
                RailVertexIdType nextVertexId = INVALID_VARIANT_ID;

                if (nearestVertexNumber < vertices.size() - 1) {
                    nextVertexId = vertices[nearestVertexNumber+1].first;
                }

                RailVertexIdType stationRailVertexId;

                const double nearStationDistance = 1; //1m

                if (railVertices.at(lastVertexId).vertex.DistanceTo(nearestPoint) <= nearStationDistance) {
                    stationRailVertexId = lastVertexId;

                    if (nextVertexId != INVALID_VARIANT_ID &&
                        railVertices.at(nextVertexId).vertex.DistanceTo(nearestPoint) < railVertices.at(lastVertexId).vertex.DistanceTo(nearestPoint)) {
                        stationRailVertexId = nextVertexId;
                    }

                } else if (nextVertexId != INVALID_VARIANT_ID &&
                           railVertices.at(nextVertexId).vertex.DistanceTo(nearestPoint) <= nearStationDistance) {
                    stationRailVertexId = nextVertexId;

                } else {

                    stationRailVertexId =
                        (*this).GetNewOrExistingRailVertexId(nearestPoint);

                    assert(nearestVertexIter != vertices.end());
                    nearestVertexIter++;

                    vertices.insert(nearestVertexIter, make_pair(stationRailVertexId, stationId));
                }

                station.railVertexIds.insert(stationRailVertexId);
            }
        }
    }

    for(RailRoadIdType railId = 0; railId < RailRoadIdType(railRoads.size()); railId++) {
        const RailRoad& rail = railRoads[railId];
        const deque<pair<RailVertexIdType, RailRoadStationIdType> >& vertices = rail.vertices;

        for(size_t i = 0; i < vertices.size() - 1; i++) {
            const pair<RailVertexIdType, RailRoadStationIdType>& vertex1 = vertices[i];
            const pair<RailVertexIdType, RailRoadStationIdType>& vertex2 = vertices[i+1];

            RailVertex& railVertex1 = railVertices.at(vertex1.first);
            RailVertex& railVertex2 = railVertices.at(vertex2.first);

            railVertex1.railLinks.push_back(RailLink(vertex2.first, railId));
            railVertex2.railLinks.push_back(RailLink(vertex1.first, railId));
        }
    }
}

class RailRoadLayer::RailNode {
public:
    RailNode()
        :
        railLink(),
        distance(),
        expectedDistance()
    {}

    RailNode(
        const RailLink& initRailLink,
        const double initDistance,
        const double initExpectedDistance)
        :
        railLink(initRailLink),
        distance(initDistance),
        expectedDistance(initExpectedDistance)
    {}

    bool operator<(const RailNode& right) const {
        return (expectedDistance > right.expectedDistance);
    }

    RailLink railLink;
    double distance;
    double expectedDistance;
};

void RailRoadLayer::SearchRailRoadRoute(
    const RailVertexIdType& startVertexId,
    const RailVertexIdType& destVertexId,
    deque<RailLink>& routeLinks)
{
    routeLinks.clear();

    for(size_t i = 0; i < railVertices.size(); i++) {
        railVertices[i].Initialize();
    }

    railVertices.at(startVertexId).distance = 0;

    double minDistanceToDest = DBL_MAX;
    Vertex destVertex = railVertices.at(destVertexId).vertex;
    Vertex startVertex = railVertices.at(startVertexId).vertex;

    std::priority_queue<RailNode> railNodes;

    railNodes.push(RailNode(RailLink(startVertexId), 0, startVertex.DistanceTo(destVertex)));

    while (!railNodes.empty()) {

        const RailNode railNode = railNodes.top();
        railNodes.pop();

        const RailVertex& railVertex = railVertices.at(railNode.railLink.railVertexId);

        if (railNode.distance > railVertex.distance) {
            continue;
        }

        const vector<RailLink>& railLinks = railVertex.railLinks;

        for(size_t i = 0; i < railLinks.size(); i++) {
            const RailLink& railLink = railLinks[i];
            const RailVertexIdType& linkedRailVertexId = railLink.railVertexId;
            RailVertex& linkedRailVertex = railVertices.at(linkedRailVertexId);

            const double distance =
                railVertex.distance + railVertex.vertex.DistanceTo(linkedRailVertex.vertex);

            if (linkedRailVertex.expectedMinDistanceToDest == DBL_MAX) {
                linkedRailVertex.expectedMinDistanceToDest =
                    linkedRailVertex.vertex.DistanceTo(destVertex);
            }

            if (distance + linkedRailVertex.expectedMinDistanceToDest < minDistanceToDest &&
                linkedRailVertex.IsFastRoute(distance)) {

                linkedRailVertex.SetBestRoute(RailLink(railNode.railLink.railVertexId, railLink.railId), distance);

                if (linkedRailVertexId == destVertexId) {

                    minDistanceToDest = distance;

                } else {

                    railNodes.push(
                        RailNode(
                            railLink,
                            distance,
                            distance + linkedRailVertex.expectedMinDistanceToDest));
                }
            }
        }
    }

    if (railVertices.at(destVertexId).FoundRoute()) {

        RailVertexIdType railVertexId = destVertexId;

        routeLinks.push_front(RailLink(railVertexId));

        while (railVertexId != startVertexId) {
            const RailVertex& railVertex = railVertices.at(railVertexId);

            railVertexId = railVertex.trackRailLink.railVertexId;

            routeLinks.push_front(railVertex.trackRailLink);
        }
    }
}

void RailRoadLayer::UpdateRailRoadLineAvailability(const RailRoadLineIdType& lineId)
{
    RailRoadLineInfo& lineInfo = lineInfos.at(lineId);

    const set<RailRoadIdType>& railIds = lineInfo.railIds;

    typedef set<RailRoadIdType>::const_iterator IterType;

    bool isAvailable = true;

    for(IterType iter = railIds.begin(); (isAvailable && iter != railIds.end()); iter++) {
        const RailRoadIdType& railId = (*iter);

        if (!railRoads[railId].IsEnabled()) {
            isAvailable = false;
        }
    }

    lineInfo.isAvailable = isAvailable;
}

void GisSubsystem::ScheduleGisEvent(
    const shared_ptr<SimulationEvent>& gisEventPtr,
    const TimeType& eventTime)
{
    gisEventInfos.push(GisEventInfo(eventTime, gisEventPtr));
}

void GisSubsystem::LoadLineInfo(const string& fileName)
{
    ifstream inStream(fileName.c_str());

    assert(inStream.good());

    bool isTrain = false;
    string lineName;

    map<pair<RailRoadStationIdType, RailRoadStationIdType>, vector<pair<deque<RailRoadLayer::RailLink>, double> > > sectionCandidatesCache;

    map<string, pair<deque<RailRoadStationIdType>, vector<pair<deque<RailRoadLayer::RailLink>, double> > > > stationIdsAndRailCache;

    const bool busStopIsAvailable =
        (importedGisObjectTypes.find(GIS_BUSSTOP) != importedGisObjectTypes.end());

    const bool stationIsAvailable =
        (importedGisObjectTypes.find(GIS_RAILROAD_STATION) != importedGisObjectTypes.end());


    while(!inStream.eof()) {
        string aLine;
        getline(inStream, aLine);

        DeleteTrailingSpaces(aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }

        deque<string> tokens;
        TokenizeToTrimmedLowerString(aLine, ",", tokens);

        if (tokens[0] == "line") {

            lineName.clear();
            isTrain = false;

            if (tokens.size() > 2) {
                lineName = tokens[1];
                isTrain = (tokens[2] == "train");
            }

        } else if (tokens[0] == "stop" && !lineName.empty()) {

            tokens.pop_front();

            if (isTrain) {

                if (stationIsAvailable) {
                    railRoadLayerPtr->AssignRailRoadLine(lineName, tokens, sectionCandidatesCache, stationIdsAndRailCache);
                }

            } else {

                if (busStopIsAvailable) {
                    roadLayerPtr->AssignBusLine(lineName, tokens);
                }
            }
        }
    }
}

RailRoadLineIdType RailRoadLayer::GetRailRoadLineId(const string& lineName) const
{
    typedef map<string, RailRoadLineIdType>::const_iterator IterType;

    IterType iter = railRoadLineIds.find(lineName);

    if (iter == railRoadLineIds.end()) {
        cerr << "Error: Couldn't find railroad line: " << lineName << endl;
        exit(1);
    }

    return (*iter).second;
}

bool RailRoadLayer::LineHasARoute(const string& lineName) const
{
    typedef map<string, RailRoadLineIdType>::const_iterator LineIter;

    LineIter lineIter = railRoadLineIds.find(lineName);

    if (lineIter == railRoadLineIds.end()) {
        return false;
    }

    return !lineInfos[(*lineIter).second].railRoadStationIds.empty();
}

RailRoadStationIdType RailRoadLayer::GetStationId(
    const string& lineName,
    const string& stationName) const
{
    typedef map<string, RailRoadLineIdType>::const_iterator LineIter;

    LineIter lineIter = railRoadLineIds.find(lineName);

    if (lineIter == railRoadLineIds.end()) {
        cerr << "Invalid line " << lineName << endl;
        exit(1);
    }

    const RailRoadLineIdType lineId = (*lineIter).second;
    const RailRoadLineInfo& lineInfo = lineInfos[lineId];

    typedef map<string, RailRoadStationIdType>::const_iterator StationIter;

    const map<string, RailRoadStationIdType>& railRoadStationIds = lineInfo.railRoadStationIds;

    StationIter stationIter = railRoadStationIds.find(stationName);

    if (stationIter == railRoadStationIds.end()) {
        cerr << "Line " << lineName << " does not contain " << stationName << endl;
        exit(1);
    }

    return (*stationIter).second;
}

void RailRoadLayer::GetStationIds(const string& stationName, vector<RailRoadStationIdType>& stationIds) const
{
    vector<GisPositionIdType> positionIds;

    subsystemPtr->GetPositions(stationName, positionIds, GIS_RAILROAD_STATION);

    stationIds.clear();

    for(size_t i = 0; i < positionIds.size(); i++) {
        stationIds.push_back(positionIds[i].id);
    }
}

RouteIdType RailRoadLayer::GetRouteId(
    const RailRoadLineIdType& lineId,
    const deque<RailRoadStationIdType>& stationIds) const
{
    const vector<RouteInfo>& routeInfos = lineInfos.at(lineId).routeInfos;
    bool found = false;
    RouteIdType routeId;

    for(routeId = 0; routeId < RouteIdType(routeInfos.size()); routeId++) {
        found = (routeInfos[routeId].stationIds == stationIds);

        if (found) {
            break;
        }
    }
    if (!found) {
        cerr << "Error: No railroad route for line: " << (*this).GetLineName(lineId) << endl;
        exit(1);
    }

    return routeId;
}

bool RailRoadLayer::ContainsRouteId(
    const RailRoadLineIdType& lineId,
    const deque<RailRoadStationIdType>& stationIds) const
{
    const vector<RouteInfo>& routeInfos = lineInfos.at(lineId).routeInfos;
    RouteIdType routeId;

    for(routeId = 0; routeId < RouteIdType(routeInfos.size()); routeId++) {

        if (routeInfos[routeId].stationIds == stationIds) {

            vector<vector<Vertex> > lineVertices;

            (*this).GetLineVertices(lineId, routeId, lineVertices);

            if (!lineVertices.empty()) {
                return true;
            }
        }
    }

    return false;
}

const deque<RailRoadStationIdType>& RailRoadLayer::GetRouteStationIds(
    const RailRoadLineIdType& lineId,
    const RouteIdType& routeId) const
{
    return lineInfos.at(lineId).routeInfos.at(routeId).stationIds;
}

bool RailRoadLayer::IsRailRoadLineAvailable(const RailRoadLineIdType& lineId) const
{
    return lineInfos.at(lineId).isAvailable;
}

void RailRoadLayer::GetLineVertices(
    const RailRoadLineIdType& lineId,
    const RouteIdType& routeId,
    vector<vector<Vertex> >& lineVertices) const
{
    lineVertices.clear();

    const vector<RouteInfo>& routeInfos = lineInfos.at(lineId).routeInfos;
    const RouteInfo& routeInfo = routeInfos.at(routeId);

    const deque<deque<RailVertexIdType> >& railVertexIdsPerSection = routeInfo.railVertexIdsPerSection;

    lineVertices.resize(railVertexIdsPerSection.size());

    for(size_t i = 0; i < railVertexIdsPerSection.size(); i++) {
        const deque<RailVertexIdType>& railVertexIds = railVertexIdsPerSection[i];
        vector<Vertex>& vertices = lineVertices[i];

        for(size_t j = 0; j < railVertexIds.size(); j++) {
            vertices.push_back(railVertices.at(railVertexIds[j]).vertex);
        }

        assert(!vertices.empty());
    }

    if (lineVertices.size() > 0) {

        for(size_t i = 0; i < lineVertices.size() - 1; i++) {
            vector<Vertex>& vertices1 = lineVertices[i];
            vector<Vertex>& vertices2 = lineVertices[i+1];

            if (vertices1.back() != vertices2.front()) {
                vertices1.push_back(vertices2.front());
            }
        }
    }
}

void RailRoadLayer::GetNearestStationId(
    const Vertex& position,
    bool& found,
    RailRoadStationIdType& nearStationId) const
{
    if (stations.empty()) {
        found = false;
        return;
    }

    const double lengthUnit = 500;

    found = false;

    for(double searchLength = lengthUnit; (!found); searchLength += lengthUnit) {

        list<RailRoadStationIdType> stationIds;

        subsystemPtr->GetSpatialIntersectedGisObjectIds(
            Rectangle(position, searchLength),
            GIS_RAILROAD_STATION,
            stationIds);

        typedef list<RailRoadStationIdType>::const_iterator IterType;

        double minDistanceToStop = DBL_MAX;

        for(IterType iter = stationIds.begin(); iter != stationIds.end(); iter++) {

            const RailRoadStationIdType& stationId = *iter;
            const double distance = position.DistanceTo(stations.at(stationId).GetVertex());

            if (distance < minDistanceToStop) {
                minDistanceToStop = distance;
                nearStationId = stationId;
                found = true;
            }
        }
    }
}

string RailRoadLayer::GetLineName(const RailRoadLineIdType& lineId) const
{
    typedef map<string, RailRoadLineIdType>::const_iterator IterType;

    // slow comparison
    for(IterType iter = railRoadLineIds.begin();
        iter != railRoadLineIds.end(); iter++) {

        if ((*iter).second == lineId) {
            return (*iter).first;
        }
    }

    return string();
}

void RailRoadLayer::MakeEntrance(
    const RailRoadStationIdType& stationId,
    const Vertex& position)
{
    RailRoadStation& station = stations[stationId];

    const VertexIdType entranceVertexId =
        station.commonImplPtr->subsystemPtr->GetVertexId(position);

    station.commonImplPtr->vertexIds.push_back(entranceVertexId);

    subsystemPtr->ConnectGisObject(entranceVertexId, GIS_RAILROAD_STATION, stationId);
}

void AreaLayer::ImportArea(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const AreaIdType areaId = static_cast<AreaIdType>(areas.size());
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        areas.push_back(Area(subsystemPtr, objectId, areaId));
        Area& area = areas.back();
        area.LoadParameters(theParameterDatabaseReader);

        if (nameFinder.IsAvailable()) {
            area.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        if (shpObjPtr->nVertices > 0) {
            area.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
        }

        for(int i = 0; i < shpObjPtr->nVertices; i++) {
            const double x = shpObjPtr->padfX[i];
            const double y = shpObjPtr->padfY[i];
            double z = shpObjPtr->padfZ[i];

            if (isBaseGroundLevel) {
                z += subsystemPtr->GetGroundElevationMetersAt(x, y);
            }

            area.polygon.push_back(Vertex(x, y, z));
        }

        subsystemPtr->RegisterGisObject(area, areaId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

void ParkLayer::ImportPark(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder capacityFinder(hDBF, GIS_DBF_CAPACITY_STRING);

    // register parks
    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const ParkIdType parkId = static_cast<ParkIdType>(parks.size());
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        parks.push_back(Park(subsystemPtr, objectId, parkId));
        Park& park = parks.back();

        park.LoadParameters(theParameterDatabaseReader);

        if (nameFinder.IsAvailable()) {
            park.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }
        if (capacityFinder.IsAvailable()) {
            park.humanCapacity = capacityFinder.GetInt(entryId);
        }

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        if (shpObjPtr->nVertices > 0) {
            park.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
        }

        for(int i = 0; i < shpObjPtr->nVertices; i++) {
            const double x = shpObjPtr->padfX[i];
            const double y = shpObjPtr->padfY[i];
            double z = shpObjPtr->padfZ[i];

            if (isBaseGroundLevel) {
                z += subsystemPtr->GetGroundElevationMetersAt(x, y);
            }

            park.polygon.push_back(Vertex(x, y, z));
        }

        subsystemPtr->RegisterGisObject(park, parkId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

BuildingLayer::BuildingLayer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    GisSubsystem* initSubsystemPtr)
    :
    subsystemPtr(initSubsystemPtr),
    enabledLineOfSightCalculation(true),
    totalHeightMeters(0),
    averageHeightMeters(0),
    minHeightMeters(DBL_MAX),
    maxHeightMeters(0),
    heightMetersVariance(0),
    theQuadTreePtr(new Los::LosQuadTree(Rectangle(), 0/*depth*/))
{
    (*this).LoadMovingObjectShapeDefinition(theParameterDatabaseReader);
}

void BuildingLayer::GetBuildingIdAt(
    const Vertex& position,
    bool& found,
    BuildingIdType& foundBuildingId) const
{
    found = false;

    list<BuildingIdType> buildingIds;

    subsystemPtr->GetSpatialIntersectedGisObjectIds(position, GIS_BUILDING, buildingIds);

    typedef list<BuildingIdType>::const_iterator IterType;

    for(IterType iter = buildingIds.begin(); iter != buildingIds.end(); iter++) {

        const BuildingIdType& buildingId = (*iter);
        const Building& building = buildings[buildingId];

        if (PolygonContainsPoint(building.polygon, position)) {
            found = true;
            foundBuildingId = buildingId;
            return;
        }
    }
}

void BuildingLayer::ImportBuilding(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder heightFinder(hDBF, GIS_DBF_HEIGHT_STRING);
    const AttributeFinder outerWallMaterialFinder(hDBF, GIS_DBF_MATERIAL_STRING);
    const AttributeFinder roofMaterialFinder(hDBF, GIS_DBF_ROOF_MATERIAL_STRING);
    const AttributeFinder floorMaterialFinder(hDBF, GIS_DBF_FLOOR_MATERIAL_STRING);

    const AttributeFinder meshKindFinder(hDBF, GIS_DBF_MESH_KIND_STRING);
    const AttributeFinder numberOfRoofFacesFinder(hDBF, GIS_DBF_NUMBER_OF_ROOF_FACES_STRING);
    const AttributeFinder numberOfWallFacesFinder(hDBF, GIS_DBF_NUMBER_OF_WALL_FACES_STRING);
    const AttributeFinder numberOfFloorFacesFinder(hDBF, GIS_DBF_NUMBER_OF_FLOOR_FACES_STRING);

    const AttributeFinder capacityFinder(hDBF, GIS_DBF_CAPACITY_STRING);

    // register buildings
    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const BuildingIdType buildingId = static_cast<BuildingIdType>(buildings.size());
        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        buildings.push_back(Building(subsystemPtr, objectId, buildingId));
        Building& building = buildings.back();

        building.LoadParameters(theParameterDatabaseReader);

        if (nameFinder.IsAvailable()) {
            building.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }
        if (heightFinder.IsAvailable()) {
            building.heightMeters = heightFinder.GetDouble(entryId);
        }
        if (outerWallMaterialFinder.IsAvailable()) {
            building.outerWallMaterialId =
                subsystemPtr->GetMaterialId(outerWallMaterialFinder.GetString(entryId));
        }
        if (roofMaterialFinder.IsAvailable()) {
            building.roofWallMaterialId =
                subsystemPtr->GetMaterialId(roofMaterialFinder.GetString(entryId));
        }
        if (floorMaterialFinder.IsAvailable()) {
            building.floorWallMaterialId =
                subsystemPtr->GetMaterialId(floorMaterialFinder.GetString(entryId));
        }
        if (capacityFinder.IsAvailable()) {
            building.humanCapacity = capacityFinder.GetInt(entryId);
        }

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        const int meshKind2D = 0;
        const int meshKind3D = 1;
        int meshKind = meshKind2D;
        if (meshKindFinder.IsAvailable()) {
            meshKind = meshKindFinder.GetInt(entryId);
        }

        double groundElevationMeters = 0.;

        // Use the lower ground level between vertices
        if (shpObjPtr->nVertices > 0) {
            groundElevationMeters = DBL_MAX;

            for(int i = 0; i < shpObjPtr->nVertices; i++) {
                const double x = shpObjPtr->padfX[i];
                const double y = shpObjPtr->padfY[i];

                groundElevationMeters = std::min(groundElevationMeters, subsystemPtr->GetGroundElevationMetersAt(x, y));
            }
        }

        if (meshKind == meshKind3D) {
            double minX = DBL_MAX;
            double minY = DBL_MAX;
            double maxX = -DBL_MAX;
            double maxY = -DBL_MAX;
            double maxZ = -DBL_MAX;

            for(int i = 0; i < shpObjPtr->nVertices; i += 3) {
                const double x1 = shpObjPtr->padfX[i];
                const double y1 = shpObjPtr->padfY[i];
                double z1 = shpObjPtr->padfZ[i];

                const double x2 = shpObjPtr->padfX[i+1];
                const double y2 = shpObjPtr->padfY[i+1];
                double z2 = shpObjPtr->padfZ[i+1];

                const double x3 = shpObjPtr->padfX[i+2];
                const double y3 = shpObjPtr->padfY[i+2];
                double z3 = shpObjPtr->padfZ[i+2];

                if (isBaseGroundLevel) {
                    z1 += groundElevationMeters;
                    z2 += groundElevationMeters;
                    z3 += groundElevationMeters;
                }

                const Vertex p1 = Vertex(x1, y1, z1);
                const Vertex p2 = Vertex(x2, y2, z2);
                const Vertex p3 = Vertex(x3, y3, z3);
                const Triangle face(p1, p2, p3);
                building.faces.push_back(face);

                minX = std::min(minX, p1.x);
                minX = std::min(minX, p2.x);
                minX = std::min(minX, p3.x);

                minY = std::min(minY, p1.y);
                minY = std::min(minY, p2.y);
                minY = std::min(minY, p3.y);

                maxX = std::max(maxX, p1.x);
                maxX = std::max(maxX, p2.x);
                maxX = std::max(maxX, p3.x);

                maxY = std::max(maxY, p1.y);
                maxY = std::max(maxY, p2.y);
                maxY = std::max(maxY, p3.y);

                maxZ = std::max(maxZ, p1.z);
                maxZ = std::max(maxZ, p2.z);
                maxZ = std::max(maxZ, p3.z);
            }

            building.polygon.reserve(5);
            building.polygon.push_back(Vertex(minX, minY, maxZ));
            building.polygon.push_back(Vertex(minX, maxY, maxZ));
            building.polygon.push_back(Vertex(maxX, maxY, maxZ));
            building.polygon.push_back(Vertex(maxX, minY, maxZ));
            building.polygon.push_back(Vertex(minX, minY, maxZ));

            if (numberOfRoofFacesFinder.IsAvailable()) {
                building.numberOfRoofFaces = numberOfRoofFacesFinder.GetInt(entryId);
            }
            if (numberOfWallFacesFinder.IsAvailable()) {
                building.numberOfWallFaces = numberOfWallFacesFinder.GetInt(entryId);
            }
            if (numberOfFloorFacesFinder.IsAvailable()) {
                building.numberOfFloorFaces = numberOfFloorFacesFinder.GetInt(entryId);
            }
        } else {

            if (shpObjPtr->nVertices > 0) {
                building.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
            }

            for(int i = 0; i < shpObjPtr->nVertices; i++) {
                const double x = shpObjPtr->padfX[i];
                const double y = shpObjPtr->padfY[i];
                double z = shpObjPtr->padfZ[i];

                if (isBaseGroundLevel) {
                    z += groundElevationMeters;
                }

                building.polygon.push_back(Vertex(x, y, z));
            }
        }

        subsystemPtr->RegisterGisObject(building, buildingId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

Rectangle BuildingLayer::GetMinRectangle() const
{
    if (buildings.empty()) {
        return Rectangle();
    }

    Rectangle rect = buildings.front().GetMinRectangle();

    for(size_t i = 1; i < buildings.size(); i++) {
        rect += buildings[i].GetMinRectangle();
    }

    return rect;
}

void BuildingLayer::RemakeLosTopology()
{
    if (!enabledLineOfSightCalculation) {
        return;
    }

    const Rectangle areaRect = (*this).GetMinRectangle();
    const double minAreaLength = 1000.;
    const double width = std::max(minAreaLength, areaRect.GetWidth());
    const double height = std::max(minAreaLength, areaRect.GetHeight());
    const Rectangle minRect = Rectangle(areaRect.GetCenter(), width, height);

    theQuadTreePtr.reset(new LosQuadTree(minRect, 0/*depth*/));
    totalHeightMeters = 0;
    minHeightMeters = DBL_MAX;
    maxHeightMeters = 0;

    for(BuildingIdType buildingId = 0;
        buildingId < BuildingIdType(buildings.size()); buildingId++) {
        const Building& building = buildings[buildingId];
        const double heightMeters = building.heightMeters;

        LosPolygon aPolygon;
        aPolygon.variantId = buildingId;
        aPolygon.nodeId = building.GetObjectId();
        aPolygon.shieldingLossDb = building.GetOuterWallMaterial().transmissionLossDb;
        aPolygon.anObstructionType = LosPolygon::OuterWall;

        const vector<Vertex>& polygon = building.polygon;
        for(size_t i = 0; i < polygon.size() - 1; i++) {
            const Vertex& v1 = polygon[i];
            const Vertex& v2 = polygon[i+1];

            aPolygon.SetTriangle(
                Vertex(v1.x, v1.y, v1.z),
                Vertex(v1.x, v1.y, v1.z + heightMeters),
                Vertex(v2.x, v2.y, v2.z));

            theQuadTreePtr->PushLosPolygon(aPolygon);

            aPolygon.SetTriangle(
                Vertex(v2.x, v2.y, v2.z),
                Vertex(v2.x, v2.y, v2.z + heightMeters),
                Vertex(v1.x, v1.y, v1.z + heightMeters));

            theQuadTreePtr->PushLosPolygon(aPolygon);
        }
        double roofTopMeters = heightMeters;

        vector<Triangle> roofTriangles;
        PolygonToTriangles(polygon, roofTriangles);

        for(size_t i = 0; i < roofTriangles.size(); i++) {

            const Triangle& aRoofTraiangle = roofTriangles[i];
            const Vertex& v1 = aRoofTraiangle.GetP1();
            const Vertex& v2 = aRoofTraiangle.GetP2();
            const Vertex& v3 = aRoofTraiangle.GetP3();

            aPolygon.SetTriangle(
                Vertex(v1.x, v1.y, v1.z),
                Vertex(v2.x, v2.y, v2.z),
                Vertex(v3.x, v3.y, v3.z));

            aPolygon.shieldingLossDb = building.GetRoofMaterial().transmissionLossDb;
            aPolygon.anObstructionType = LosPolygon::Roof;

            theQuadTreePtr->PushLosPolygon(aPolygon);

            aPolygon.SetTriangle(
                Vertex(v1.x, v1.y, v1.z + heightMeters),
                Vertex(v2.x, v2.y, v2.z + heightMeters),
                Vertex(v3.x, v3.y, v3.z + heightMeters));

            aPolygon.shieldingLossDb = building.GetFloorMaterial().transmissionLossDb;
            aPolygon.anObstructionType = LosPolygon::Floor;

            theQuadTreePtr->PushLosPolygon(aPolygon);

            roofTopMeters = std::max(roofTopMeters, v1.z + heightMeters);
            roofTopMeters = std::max(roofTopMeters, v2.z + heightMeters);
            roofTopMeters = std::max(roofTopMeters, v3.z + heightMeters);
        }

        totalHeightMeters += roofTopMeters;
        minHeightMeters = std::min(minHeightMeters, heightMeters);
        maxHeightMeters = std::max(maxHeightMeters, heightMeters);
    }

    for(WallIdType wallId = 0;  wallId < WallIdType(walls.size()); wallId++) {
        const Wall& wall = walls[wallId];
        const double heightMeters = wall.heightMeters;

        LosPolygon aPolygon;
        aPolygon.variantId =
            static_cast<VariantIdType>(buildings.size() + wallId);
        aPolygon.nodeId = wall.GetObjectId();
        aPolygon.shieldingLossDb = wall.GetMaterial().transmissionLossDb;
        aPolygon.anObstructionType = LosPolygon::InnerWall;

        const vector<Vertex>& vertices = wall.vertices;
        for(size_t i = 0; i < vertices.size() - 1; i++) {
            const Vertex& v1 = vertices[i];
            const Vertex& v2 = vertices[i+1];

            aPolygon.SetTriangle(
                Vertex(v1.x, v1.y, v1.z),
                Vertex(v1.x, v1.y, v1.z + heightMeters),
                Vertex(v2.x, v2.y, v2.z));

            theQuadTreePtr->PushLosPolygon(aPolygon);

            aPolygon.SetTriangle(
                Vertex(v2.x, v2.y, v2.z),
                Vertex(v2.x, v2.y, v2.z + heightMeters),
                Vertex(v1.x, v1.y, v1.z + heightMeters));

            theQuadTreePtr->PushLosPolygon(aPolygon);
        }

        totalHeightMeters += heightMeters;
        minHeightMeters = std::min(minHeightMeters, heightMeters);
        maxHeightMeters = std::max(maxHeightMeters, heightMeters);
    }

    // XXX
    // add wall polygons

    averageHeightMeters = totalHeightMeters / buildings.size();

    double squaredDifferenceSigma = 0;

    for(size_t i = 0; i < buildings.size(); i++) {
        const double heightDifference = buildings[i].heightMeters - averageHeightMeters;
        squaredDifferenceSigma += (heightDifference*heightDifference);
    }
    for(size_t i = 0; i < walls.size(); i++) {
        const double heightDifference = walls[i].heightMeters - averageHeightMeters;
        squaredDifferenceSigma += (heightDifference*heightDifference);
    }

    heightMetersVariance = squaredDifferenceSigma / (buildings.size() + walls.size());
}

void BuildingLayer::ImportWall(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder materialFinder(hDBF, GIS_DBF_MATERIAL_STRING);
    const AttributeFinder widthFinder(hDBF, GIS_DBF_WIDTH_STRING);
    const AttributeFinder heightFinder(hDBF, GIS_DBF_HEIGHT_STRING);

    // register walls
    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const WallIdType wallId = static_cast<WallIdType>(walls.size());
        const bool isBaseGroundLevel =
            ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId);

        walls.push_back(Wall(subsystemPtr, objectId, wallId));
        Wall& wall = walls.back();

        if (nameFinder.IsAvailable()) {
            wall.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }
        if (materialFinder.IsAvailable()) {
            wall.materialId = subsystemPtr->GetMaterialId(materialFinder.GetString(entryId));
        }
        if (widthFinder.IsAvailable()) {
            wall.widthMeters = widthFinder.GetDouble(entryId);
        }
        if (heightFinder.IsAvailable()) {
            wall.heightMeters = heightFinder.GetDouble(entryId);
        }

        assert(shpObjPtr->nParts == 1);
        assert(shpObjPtr->nVertices > 0);

        double groundElevationMeters = 0.;

        // Use the lower ground level between vertices
        if (shpObjPtr->nVertices > 0) {
            groundElevationMeters = DBL_MAX;

            for(int i = 0; i < shpObjPtr->nVertices; i++) {
                const double x = shpObjPtr->padfX[i];
                const double y = shpObjPtr->padfY[i];

                groundElevationMeters = std::min(groundElevationMeters, subsystemPtr->GetGroundElevationMetersAt(x, y));
            }
        }

        if (shpObjPtr->nVertices > 0) {
            wall.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
        }

        for(int i = 0; i < shpObjPtr->nVertices; i++) {
            const double x = shpObjPtr->padfX[i];
            const double y = shpObjPtr->padfY[i];
            double z = shpObjPtr->padfZ[i];

            if (isBaseGroundLevel) {
                z += groundElevationMeters;
            }

            wall.vertices.push_back(Vertex(x, y, z));
        }

        subsystemPtr->RegisterGisObject(wall, wallId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

bool BuildingLayer::PositionsAreLineOfSight(
    const Vertex& vector1,
    const Vertex& vector2,
    const set<NodeIdType>& ignoredNodeIds) const
{
    if (!enabledLineOfSightCalculation) {
        return false;
    }

    return (!theQuadTreePtr->HasCollision(LosRay(vector1, vector2), ignoredNodeIds));
}

void BuildingLayer::CalculateWallCollisionPoints(
    const Vertex& vector1,
    const Vertex& vector2,
    vector<pair<Vertex, VariantIdType> >& collisionPoints,
    const set<NodeIdType>& ignoredNodeIds) const
{
    double notUsedShieldingLossDb;

    (*this).CalculateWallCollisionPoints(
        vector1,
        vector2,
        collisionPoints,
        notUsedShieldingLossDb,
        ignoredNodeIds);
}

void BuildingLayer::CalculateWallCollisionPoints(
    const Vertex& vector1,
    const Vertex& vector2,
    vector<pair<Vertex, VariantIdType> >& collisionPoints,
    double& totalShieldingLossDb,
    const set<NodeIdType>& ignoredNodeIds) const
{
    collisionPoints.clear();
    totalShieldingLossDb = 0;

    //if (!(IsInRectangle(minRectangle, position1) &&
    //      IsInRectangle(minRectangle, position2))) {
    //    return;
    //}

    const LosRay ray(vector1, vector2);

    map<double, Los::WallCollisionInfoType> collisions;

    theQuadTreePtr->CheckCollision(ray, ignoredNodeIds, false/*checkJustACollsion*/, false/*isJustHorizontalCheck*/, collisions);

    if (!collisions.empty()) {

        typedef map<double, Los::WallCollisionInfoType>::iterator IterType;

        for(IterType iter = collisions.begin();
            iter != collisions.end(); iter++) {

            const double t = (*iter).first;
            const Los::WallCollisionInfoType& collision = (*iter).second;

            collisionPoints.push_back(make_pair(ray.Position(t), (*iter).second.variantId));

            totalShieldingLossDb += collision.shieldingLossDb;
        }
    }
}




void BuildingLayer::CalculateNumberOfFloorsAndWallsTraversed(
    const Vertex& vector1,
    const Vertex& vector2,
    const set<NodeIdType>& ignoredNodeIds,
    unsigned int& numberOfFloors,
    unsigned int& numberOfWalls) const
{
    numberOfFloors = 0;
    numberOfWalls = 0;

    const LosRay ray(vector1, vector2);

    map<double, Los::WallCollisionInfoType> collisions;

    theQuadTreePtr->CheckCollision(ray, ignoredNodeIds, false/*checkJustACollsion*/, false/*isJustHorizontalCheck*/, collisions);

    if (!collisions.empty()) {

        typedef map<double, Los::WallCollisionInfoType>::iterator IterType;

        for(IterType iter = collisions.begin(); iter != collisions.end(); iter++) {
            const Los::WallCollisionInfoType& collision = (*iter).second;

            switch (collision.anObstructionType) {
            case Los::LosPolygon::Floor:
                numberOfFloors++;
                break;

            case Los::LosPolygon::InnerWall:
            case Los::LosPolygon::OuterWall:
                numberOfWalls++;
                break;

            default:
                assert(false); abort(); break;
            }//switch//
        }
    }

}//CalculateNumberOfFloorsAndWallsTraversed//



double BuildingLayer::CalculateTotalWallAndFloorLossDb(
    const Vertex& position1,
    const Vertex& position2) const
{
    vector<pair<Vertex, VariantIdType> > collisionPoints;
    double totalShieldingLossDb;

    (*this).CalculateWallCollisionPoints(
        position1, position2, collisionPoints, totalShieldingLossDb);

    return totalShieldingLossDb;
}

double BuildingLayer::GetCollisionPointHeight(
    pair<Vertex, VariantIdType>& collisionPoint) const
{
    const VariantIdType& variantId = collisionPoint.second;

    // check if moving object
    if (variantId == INVALID_GIS_OBJECT_ID) {
        return 0.;
    }

    if (collisionPoint.second < VariantIdType(buildings.size())) {
        const Building& building = buildings[collisionPoint.second];

        return building.GetHeightMeters();

    } else {
        const WallIdType wallId = static_cast<WallIdType>(collisionPoint.second - buildings.size());
        const Wall& wall = walls.at(wallId);

        return wall.GetHeightMeters();
    }
}

size_t BuildingLayer::CalculateNumberOfWallRoofFloorInteractions(
    const Vertex& position1,
    const Vertex& position2) const
{
    vector<pair<Vertex, VariantIdType> > collisionPoints;
    double totalShieldingLossDb;

    (*this).CalculateWallCollisionPoints(position1, position2, collisionPoints, totalShieldingLossDb);

    return collisionPoints.size();
}

void BuildingLayer::MakeEntrance(
    const BuildingIdType& buildingId,
    const Vertex& position)
{
    Building& building = buildings[buildingId];

    const VertexIdType entranceVertexId =
        building.commonImplPtr->subsystemPtr->GetVertexId(position);

    building.commonImplPtr->vertexIds.push_back(entranceVertexId);

    subsystemPtr->ConnectGisObject(entranceVertexId, GIS_BUILDING, buildingId);
}

void BuildingLayer::LoadMovingObjectShapeDefinition(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    if (!theParameterDatabaseReader.ParameterExists("moving-object-shape-file")) {
        return;
    }//if//

    const string filename = theParameterDatabaseReader.ReadString("moving-object-shape-file");

    ifstream inStream(filename.c_str());
    if (!inStream.good()) {
        cerr << "Error: Couldn't open Moving Object Shape File: " << filename << endl;
        exit(1);
    }//if//

    while (!inStream.eof()) {

        string aLine;
        getline(inStream, aLine);
        DeleteTrailingSpaces(aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }//if//

        istringstream lineStream(aLine);

        string shapeType;
        lineStream >> shapeType;
        ConvertStringToLowerCase(shapeType);

        if (movingShapes.find(shapeType) != movingShapes.end()) {
            cerr << "Error: Duplicated shape: " << shapeType << endl;
            cerr << aLine << endl;
            exit(1);
        }//if//

        MovingShape& movingShape = movingShapes[shapeType];

        lineStream >> movingShape.length;
        lineStream >> movingShape.width;
        lineStream >> movingShape.height;
        lineStream >> movingShape.materialName;
    }
}

void BuildingLayer::AddMovingObject(
    const MaterialSet& materials,
    const NodeIdType nodeId,
    const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
    const string& shapeType)
{
    if (movingShapes.find(shapeType) == movingShapes.end()) {
        cerr << "Invalid object-shape-type: " << shapeType << endl;
        exit(1);
    }

    const MovingShape& movingShape = movingShapes[shapeType];
    const Material& material = materials.GetMaterial(movingShape.materialName);

    shared_ptr<LosMovingObject> movingObjectPtr(
        new LosMovingObject(
            nodeId,
            mobilityModelPtr,
            movingShape.length,
            movingShape.width,
            movingShape.height,
            material.transmissionLossDb * 0.5));

    losMovingObjects.push_back(movingObjectPtr);

}//AddMovingObject//

void BuildingLayer::SyncMovingObjectTime(const TimeType& currentTime)
{
    typedef list<shared_ptr<Los::LosMovingObject> >::iterator IterType;
    const GroundLayer& groundLayer = *subsystemPtr->GetGroundLayerPtr();

    for(IterType iter = losMovingObjects.begin();
        (iter != losMovingObjects.end()); iter++) {

        LosMovingObject& movingObject = *(*iter);
        bool positionChanged;

        movingObject.UpdateMovingObjectMobilityPosition(
            groundLayer,
            currentTime,
            positionChanged);

        if (positionChanged) {
            (*this).UpdateMovingObjectPolygon(movingObject);
        }
    }
}

void BuildingLayer::DeleteMovingObjectPolygon(Los::LosMovingObject& movingObject)
{
    LosQuadTree* lastTreePtr = movingObject.lastTreePtr;

    if (lastTreePtr != nullptr) {
        lastTreePtr->losObbPtrs.erase(movingObject.lastObbIter);
    }

    movingObject.lastTreePtr = nullptr;
}

void BuildingLayer::UpdateMovingObjectPolygon(Los::LosMovingObject& movingObject)
{
    LosQuadTree* const lastTreePtr = movingObject.lastTreePtr;

    (*this).DeleteMovingObjectPolygon(movingObject);

    LosQuadTree* startTreePtr = theQuadTreePtr.get();

    if (lastTreePtr != nullptr) {
        if (lastTreePtr->place.Contains(movingObject.losObbPtr->GetRect())) {
            // if in same place, restart from current tree
            startTreePtr = lastTreePtr;
        }
    }

    startTreePtr->PushLosOrientedBoundingBox(
        movingObject.losObbPtr,
        movingObject.lastTreePtr,
        movingObject.lastObbIter);
}

inline
void BuildingLayer::RemoveMovingObject(const NodeIdType nodeId)
{
    typedef list<shared_ptr<Los::LosMovingObject> >::iterator IterType;

    IterType iter = losMovingObjects.begin();

    while (iter != losMovingObjects.end()) {
        LosMovingObject& movingObject = *(*iter);

        if (movingObject.nodeId == nodeId) {
            // delete polygons before removing
            (*this).DeleteMovingObjectPolygon(movingObject);

            iter = losMovingObjects.erase(iter);
        } else {
            iter++;
        }
    }
}

void ParkLayer::MakeEntrance(
    const ParkIdType& parkId,
    const Vertex& position)
{
    Park& park = parks[parkId];

    const VertexIdType entranceVertexId =
        park.commonImplPtr->subsystemPtr->GetVertexId(position);

    park.commonImplPtr->vertexIds.push_back(entranceVertexId);

    subsystemPtr->ConnectGisObject(entranceVertexId, GIS_PARK, parkId);
}

GroundLayer::GroundLayer(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    GisSubsystem* initSubsystemPtr)
    :
    subsystemPtr(initSubsystemPtr),
    numberVerticalMeshes(0),
    numberHorizontalMeshes(0),
    meshLengthMeters(DBL_MAX)
{
}

void GroundLayer::ImportGroundSurface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    using SHull::Shx;
    using SHull::s_hull_del_ray2;
    using SHull::Triad;

    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, rect);

    if (entities <= 0) {
        return;
    }

    const double minMeshLengthMeters = 10.;

    // assume ground surface as a square area for mesh length calculation.
    meshLengthMeters = std::max<double>(
        minMeshLengthMeters,
        std::min<double>(rect.GetWidth(), rect.GetHeight()) / (sqrt(static_cast<double>(entities))));

    typedef double ShxFloat;

    const int seed = 1;
    const ShxFloat maxOffset = static_cast<ShxFloat>(meshLengthMeters*0.1);

    RandomNumberGenerator aRandomNumberGenerator(seed);

    vector<Vertex> points;
    vector<Shx> shxs;

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        assert(shpObjPtr->nParts == 0);
        assert(shpObjPtr->nVertices == 1);

        const Vertex point(shpObjPtr->padfX[0], shpObjPtr->padfY[0], shpObjPtr->padfZ[0]);

        points.push_back(point);

        //Note:
        // Completely grid vertex inputs generate a redudant delauney triangulation.
        // Use non-grid vertex with small offset.

        Shx pt;
        pt.id = static_cast<int>(shxs.size());
        pt.r = static_cast<ShxFloat>(point.x) + static_cast<ShxFloat>(aRandomNumberGenerator.GenerateRandomInt(0, INT_MAX))/INT_MAX * maxOffset;
        pt.c = static_cast<ShxFloat>(point.y) + static_cast<ShxFloat>(aRandomNumberGenerator.GenerateRandomInt(0, INT_MAX))/INT_MAX * maxOffset;

        shxs.push_back(pt);

        SHPDestroyObject(shpObjPtr);
    }

    vector<Triad> triads;
    s_hull_del_ray2(shxs, triads);

    rect = rect.Expanded(meshLengthMeters*0.5);

    const double width = rect.GetWidth();
    const double height = rect.GetHeight();

    numberHorizontalMeshes = static_cast<MeshIdType>(ceil(width / meshLengthMeters));
    numberVerticalMeshes = static_cast<MeshIdType>(ceil(height / meshLengthMeters));

    assert((numberVerticalMeshes*numberHorizontalMeshes) < ULONG_MAX);

    const MeshIdType numberMeshes =
        static_cast<MeshIdType>(numberVerticalMeshes * numberHorizontalMeshes);

    grounMeshes.reset(new GroundMesh[numberMeshes]);

    for(size_t i = 0; i < triads.size(); i++) {
        const Triad& triad = triads[i];
        const Triangle triangle(points[triad.a], points[triad.b], points[triad.c]);
        const Rectangle triangleRect = triangle.GetRect();

        if (triangleRect.GetWidth() > 0 && triangleRect.GetHeight() > 0) {
            const MeshIdType minX = static_cast<MeshIdType>((triangleRect.minX - rect.minX) / meshLengthMeters);
            const MeshIdType minY = static_cast<MeshIdType>((triangleRect.minY - rect.minY) / meshLengthMeters);

            const MeshIdType maxX = static_cast<MeshIdType>((triangleRect.maxX - rect.minX) / meshLengthMeters);
            const MeshIdType maxY = static_cast<MeshIdType>((triangleRect.maxY - rect.minY) / meshLengthMeters);

            for(MeshIdType indexX = minX; indexX <= maxX; indexX++) {
                for(MeshIdType indexY = minY; indexY <= maxY; indexY++) {
                    const Rectangle meshRect(
                        indexX*meshLengthMeters + rect.minX,
                        indexY*meshLengthMeters + rect.minY,
                        (indexX+1)*meshLengthMeters + rect.minX,
                        (indexY+1)*meshLengthMeters + rect.minY);

                    if (!triangle.IntersectsWith(meshRect)) {
                        continue;
                    }

                    GroundMesh& groundMesh =
                        grounMeshes[(*this).CalculateMeshNumberAt(indexX, indexY)];

                    groundMesh.triangleIds.push_back(TriangleIdType(triangles.size()));
                }
            }
            triangles.push_back(triangle);
        }
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

void GroundLayer::ImportTree(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;

    Rectangle treeRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, treeRect);

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        assert(shpObjPtr->nParts == 0);
        assert(shpObjPtr->nVertices == 1);

        const Vertex point(shpObjPtr->padfX[0], shpObjPtr->padfY[0], shpObjPtr->padfZ[0]);

        if (rect.Contains(point)) {
            const MeshIdType meshNumber = (*this).CalculateMeshNumberAt(point);
            GroundMesh& groundMesh = grounMeshes[meshNumber];

            groundMesh.obstructionHeightMetersFromGround = static_cast<float>(point.z);
            groundMesh.obstructionTypes = GROUND_OBSTRUCTION_TREE;
        }

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

double GroundLayer::GetElevationMetersAt(const Vertex& pos, const bool isRoofTopElevation) const
{
    const TriangleIdType triangleId = (*this).GetTriangleIdAt(pos);

    if (triangleId != INVALID_VARIANT_ID) {
        const Triangle& triangle = triangles[triangleId];

        return triangle.GetSurfaceHeightAt(pos);
    }

    return 0.;
}

GroundLayer::TriangleIdType GroundLayer::GetTriangleIdAt(const Vertex& pos) const
{
    if (!rect.Contains(pos)) {
        return INVALID_VARIANT_ID;
    }

    const MeshIdType meshNumber = (*this).CalculateMeshNumberAt(pos);
    const GroundMesh& groundMesh = grounMeshes[meshNumber];

    typedef list<TriangleIdType>::const_iterator IterType;

    for(IterType iter = groundMesh.triangleIds.begin(); iter != groundMesh.triangleIds.end(); iter++) {
        const TriangleIdType& triangleId = (*iter);
        const Triangle& triangle = triangles[triangleId];

        if (triangle.Contains(pos)) {
            return triangleId;
        }
    }

    return INVALID_VARIANT_ID;
}

void GroundLayer::GetSeriallyCompleteElevationPoints(
    const Vertex txPos,
    const Vertex rxPos,
    const int numberDivisions,
    const bool addBuildingHeightToElevation,
    const bool addTreeHeightToElevation,
    vector<Vertex>& points) const
{
    const Vertex directionVector = (rxPos - txPos);
    const Vertex normalVector = (rxPos - txPos).XYPoint().Normalized();
    const double distance = txPos.DistanceTo(rxPos);
    const double calculationPointDivisionLength = distance / numberDivisions;

    map<double, Los::WallCollisionInfoType> collisions;

    const BuildingLayer& buildingLayer = *subsystemPtr->GetBuildingLayerPtr();
    const shared_ptr<Los::LosQuadTree> theQuadTreePtr = buildingLayer.GetQuadTreePtr();

    if (addBuildingHeightToElevation) {
        const LosRay ray(txPos.XYPoint(), rxPos.XYPoint());
        
        set<NodeIdType> ignoredNodeIds;
        
        theQuadTreePtr->CheckCollision(ray, ignoredNodeIds, false/*checkJustACollsion*/, true/*isJustHorizontalCheck*/, collisions);
    }

    TriangleIdType triangleId = INVALID_VARIANT_ID;

    points.clear();

    typedef map<double, Los::WallCollisionInfoType>::const_iterator IterType;

    IterType collisionIter = collisions.begin();
    IterType lastIter = collisionIter;

    for(int i = 0; i <= numberDivisions; i++) {
        const double t = double(i) / numberDivisions;

        Vertex point = txPos + directionVector*t;
        bool isElevationAdjustedToRoofTop = false;

        if ((i > 0) && (i < numberDivisions)) {

            // Consider building height only for intermediate points.

            if (collisionIter != collisions.end()) {
                while ((collisionIter != collisions.end()) &&
                       ((*collisionIter).first < t)) {
                    
                    assert((*collisionIter).second.anObstructionType == LosPolygon::Roof);
                    
                    lastIter = collisionIter;
                    collisionIter++;
                }
            }
            
            if (lastIter != collisions.end()) {
                const BuildingIdType buildingId = (*lastIter).second.variantId;
                const Building& building = buildingLayer.GetBuilding(buildingId);
                
                if (PolygonContainsPoint(building.GetBuildingPolygon(), point)) {
                    point.z = building.GetRoofTopHeightMeters();
                    isElevationAdjustedToRoofTop = true;
                }
            }
        }
        

        if (!isElevationAdjustedToRoofTop) {
            
            if (triangleId == INVALID_VARIANT_ID) {

                triangleId = (*this).GetTriangleIdAt(point);

            } else {

                // Continue from last triangle

                const Triangle& triangle = triangles[triangleId];
                
                if (!triangle.Contains(point)) {
                    triangleId = (*this).GetTriangleIdAt(point);
                }
            }
            
            if (triangleId != INVALID_VARIANT_ID) {
                const Triangle& triangle = triangles[triangleId];
                
                point.z = triangle.GetSurfaceHeightAt(point);

            } else {

                // no ground triangle ==> "z = 0.0"

            }

            if (addTreeHeightToElevation) {
                
                // Consider obstacle height only for intermediate points.

                if ((rect.Contains(point)) &&
                    (i > 0) &&
                    (i < numberDivisions)) {

                    const MeshIdType meshNumber = (*this).CalculateMeshNumberAt(point);
                    const GroundMesh& groundMesh = grounMeshes[meshNumber];
                    
                    point.z += groundMesh.obstructionHeightMetersFromGround;
                }
            }

        }

        points.push_back(point);
    }
}

void GroundLayer::GetGroundElevationCompletedVertices(
    const Vertex& lineEdge1,
    const Vertex& lineEdge2,
    deque<Vertex>& vertices) const
{
    const Rectangle lineRect(lineEdge1, lineEdge2);

    const MeshIdType minX = std::min<MeshIdType>(static_cast<MeshIdType>(floor((lineRect.minX - rect.minX) / meshLengthMeters)), numberHorizontalMeshes -1);
    const MeshIdType maxX = std::max<MeshIdType>(0, static_cast<MeshIdType>(ceil((lineRect.maxX - rect.minX) / meshLengthMeters)));
    const MeshIdType minY = std::min<MeshIdType>(static_cast<MeshIdType>(floor((lineRect.minY - rect.minY) / meshLengthMeters)), numberVerticalMeshes -1);
    const MeshIdType maxY = std::max<MeshIdType>(0, static_cast<MeshIdType>(ceil((lineRect.maxY - rect.minY) / meshLengthMeters)));

    set<pair<Vertex, Vertex> > triangleEdges;
    set<TriangleIdType> triangleIds;

    for(MeshIdType indexX = minX; indexX <= maxX; indexX++) {
        for(MeshIdType indexY = minY; indexY <= maxY; indexY++) {
            const Rectangle meshRect(
                indexX*meshLengthMeters + rect.minX,
                indexY*meshLengthMeters + rect.minY,
                (indexX+1)*meshLengthMeters + rect.minX,
                (indexY+1)*meshLengthMeters + rect.minY);

            if (!meshRect.IntersectsWithLine(lineEdge1, lineEdge2)) {
                continue;
            }

            if ((0 <= indexX && indexX < numberHorizontalMeshes) &&
                (0 <= indexY && indexY < numberVerticalMeshes)) {

                const MeshIdType meshNumber = (*this).CalculateMeshNumberAt(indexX, indexY);
                const GroundMesh& groundMesh = grounMeshes[meshNumber];

                typedef list<TriangleIdType>::const_iterator IterType;

                for(IterType iter = groundMesh.triangleIds.begin();
                    iter != groundMesh.triangleIds.end(); iter++) {

                    const TriangleIdType& triangleId = (*iter);
                    const Triangle& triangle = triangles[triangleId];

                    if (triangleIds.find(triangleId) == triangleIds.end()) {
                        triangleIds.insert(triangleId);

                        if (triangle.IntersectsWithLine(lineEdge1, lineEdge2)) {
                            const pair<Vertex, Vertex>& triangleEdge1 = triangle.GetEdge1();
                            const pair<Vertex, Vertex>& triangleEdge2 = triangle.GetEdge2();
                            const pair<Vertex, Vertex>& triangleEdge3 = triangle.GetEdge3();

                            if (HorizontalLinesAreIntersection(
                                    lineEdge1, lineEdge2, triangleEdge1.first, triangleEdge1.second)) {
                                triangleEdges.insert(triangleEdge1);
                            }
                            if (HorizontalLinesAreIntersection(
                                    lineEdge1, lineEdge2, triangleEdge2.first, triangleEdge2.second)) {
                                triangleEdges.insert(triangleEdge2);
                            }
                            if (HorizontalLinesAreIntersection(
                                    lineEdge1, lineEdge2, triangleEdge3.first, triangleEdge3.second)) {
                                triangleEdges.insert(triangleEdge3);
                            }
                        }
                    }
                }
            }
        }
    }

    typedef set<pair<Vertex, Vertex> >::const_iterator EdgeIter;

    map<double, Vertex> elevationVertices;

    const double origEdge1Z = (*this).GetElevationMetersAt(lineEdge1);
    const double origEdge2Z = (*this).GetElevationMetersAt(lineEdge2);

    const Vertex origEdge1(lineEdge1.x, lineEdge1.y, std::max(0., lineEdge1.z - origEdge1Z));
    const Vertex origEdge2(lineEdge2.x, lineEdge2.y, std::max(0., lineEdge2.z - origEdge2Z));

    elevationVertices[0.] = Vertex(origEdge1.x, origEdge1.y, origEdge1.z + origEdge1Z);
    elevationVertices[1.] = Vertex(origEdge2.x, origEdge2.y, origEdge2.z + origEdge2Z);

    const double distance = lineEdge1.XYPoint().DistanceTo(lineEdge2.XYPoint());

    if (distance > 0) {
        for(EdgeIter iter = triangleEdges.begin(); iter != triangleEdges.end(); iter++) {
            const pair<Vertex, Vertex>& edge = (*iter);

            const Vertex intersectoinPosOfBase =
                CalculateIntersectionPositionBetweenLine(
                    lineEdge1, lineEdge2,
                    edge.first, edge.second);

            const double t = (intersectoinPosOfBase.XYPoint() - lineEdge1.XYPoint()).Distance() / distance;
            const Vertex intersectionPosOfOrig = (origEdge2 - origEdge1)*t + origEdge1;

            assert(intersectionPosOfOrig.z >= 0);

            elevationVertices[t] =
                intersectionPosOfOrig + Vertex(0., 0., (*this).GetElevationMetersAt(intersectionPosOfOrig));
        }
    }

    typedef map<double, Vertex>::const_iterator VertexIter;

    for(VertexIter iter = elevationVertices.begin(); iter != elevationVertices.end(); iter++) {
        vertices.push_back((*iter).second);
    }
}

double GroundLayer::GetRoofTopHeightWithGroundElevationtMetersAt(const Vertex& pos) const
{
    return (*this).GetElevationMetersAt(pos, true/*isRoofTopElevation*/);
}

bool GroundLayer::HasTree(const Vertex& pos) const
{
    if (!rect.Contains(pos)) {
        return false;
    }

    const MeshIdType meshNumber = (*this).CalculateMeshNumberAt(pos);
    const GroundMesh& groundMesh = grounMeshes[meshNumber];

    return groundMesh.IsTree();
}

GroundLayer::MeshIdType GroundLayer::CalculateMeshNumberAt(const Vertex& pos) const
{
    const MeshIdType indexX = static_cast<MeshIdType>((pos.x - rect.minX) / meshLengthMeters);
    const MeshIdType indexY = static_cast<MeshIdType>((pos.y - rect.minY) / meshLengthMeters);

    return (*this).CalculateMeshNumberAt(indexX, indexY);
}

GroundLayer::MeshIdType GroundLayer::CalculateMeshNumberAt(const MeshIdType indexX, const MeshIdType indexY) const
{
    assert(0 <= indexX && indexX < numberHorizontalMeshes);
    assert(0 <= indexY && indexY < numberVerticalMeshes);

    const MeshIdType retValue =  (indexY*numberHorizontalMeshes + indexX);

    return retValue;
}

//----------------------------------------------------
// GisSubsystem
//----------------------------------------------------

#pragma warning(disable:4355)

GisSubsystem::GisSubsystem(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr)
    :
    theSimulationEnginePtr(initSimulationEnginePtr.get()),
    isDebugMode(false),
    changedGisTopology(false),
    executeEventsManually(false),
    positionInLatlongDegree(false),
    latitudeOriginDegrees(LATITUDE_ORIGIN_DEGREES),
    longitudeOriginDegrees(LONGITUDE_ORIGIN_DEGREES),
    poiLayerPtr(new PoiLayer(this)),
    roadLayerPtr(new RoadLayer(theParameterDatabaseReader, this)),
    railRoadLayerPtr(new RailRoadLayer(this)),
    areaLayerPtr(new AreaLayer(this)),
    parkLayerPtr(new ParkLayer(this)),
    buildingLayerPtr(new BuildingLayer(theParameterDatabaseReader, this)),
    groundLayerPtr(new GroundLayer(theParameterDatabaseReader, this)),
    insiteGeometryPtr(new InsiteGeometry()),
    spatialObjectMaps(GIS_GENERIC_START),
    reservedObjectId(RESERVED__GIS_OBJECT_ID)
{
    if (theParameterDatabaseReader.ParameterExists("gis-debug-mode")) {
        isDebugMode = theParameterDatabaseReader.ReadBool("gis-debug-mode");
    }
    if (theParameterDatabaseReader.ParameterExists("gis-object-position-in-latlong-degree")) {
        positionInLatlongDegree =
            theParameterDatabaseReader.ReadBool("gis-object-position-in-latlong-degree");

        if (positionInLatlongDegree) {
            latitudeOriginDegrees =
                theParameterDatabaseReader.ReadDouble("gis-latitude-origin-degrees");
            longitudeOriginDegrees =
                theParameterDatabaseReader.ReadDouble("gis-longitude-origin-degrees");
        }
    }


    if (theParameterDatabaseReader.ParameterExists("material-file")) {
        (*this).LoadLocalMaterials(theParameterDatabaseReader.ReadString("material-file"));
    }

    string shapeFileDirPath;

    if (theParameterDatabaseReader.ParameterExists("gis-object-file-path")) {
        shapeFileDirPath = theParameterDatabaseReader.ReadString("gis-object-file-path");
    }

    if (theParameterDatabaseReader.ParameterExists("gis-object-files")) {
        const string fileNames = theParameterDatabaseReader.ReadString("gis-object-files");
        std::istringstream inStream(fileNames);

        typedef vector<pair<int, string> > ContainarType;
        typedef std::greater<pair<int, string> > PriorityType;

        std::priority_queue<pair<int, string>, ContainarType, PriorityType> fileNamesWithPriority;

        Vertex originPoint(0, 0);

        entireRect = Rectangle(originPoint, 1);

        while (!inStream.eof()) {
            string fileName;
            inStream >> fileName;

            int readingPriority = 0;

            if (fileName.find(GIS_RAILROAD_STRING) != string::npos) {
                readingPriority = 7;
            } else if (fileName.find(GIS_ROAD_STRING) != string::npos) {
                readingPriority = 1;
            } else if (fileName.find(GIS_INTERSECTION_STRING) != string::npos) {
                readingPriority = 0;
            } else if (fileName.find(GIS_AREA_STRING) != string::npos) {
                readingPriority = -1;
            } else if (fileName.find(GIS_PARK_STRING) != string::npos) {
                readingPriority = 2;
            } else if (fileName.find(GIS_BUILDING_STRING) != string::npos) {
                readingPriority = 3;
            } else if (fileName.find(GIS_WALL_STRING) != string::npos) {
                readingPriority = 5;
            } else if (fileName.find(GIS_STATION_STRING) != string::npos) {
                readingPriority = 6;
            } else if ((fileName.find(GIS_OLD_TRAFFIC_LIGHT_STRING) != string::npos) ||
                       (fileName.find(GIS_TRAFFIC_LIGHT_STRING) != string::npos)) {
                readingPriority = 8;
            } else if (fileName.find(GIS_BUSSTOP_STRING) != string::npos) {
                readingPriority = 9;
            } else if (fileName.find(GIS_ENTRANCE_STRING) != string::npos) {
                readingPriority = 10;
            } else if (fileName.find(GIS_POI_STRING) != string::npos) {
                readingPriority = 11;
            } else if (fileName.find(GIS_GROUND_STRING) != string::npos) {
                readingPriority = -2;
            } else if (fileName.find(GIS_TREE_STRING) != string::npos) {
                readingPriority = 12;
            } else if (!fileName.empty()) {
                readingPriority = 13;
            }

            fileNamesWithPriority.push(make_pair(readingPriority, fileName));

            entireRect += PeekLayerRectangle(shapeFileDirPath + fileName);
        }

        const double spatialMeshUnit = 100; //100m
        spatialVertexMap.SetMesh(
            entireRect, spatialMeshUnit,
            SpatialObjectMap::MAX_VERTEX_MESH_SIZE);
        for(size_t i = 0; i < spatialObjectMaps.size(); i++) {
            spatialObjectMaps[i].SetMesh(entireRect, spatialMeshUnit);
        }

        while (!fileNamesWithPriority.empty()) {
            const string& fileName = fileNamesWithPriority.top().second;
            const string filePath = shapeFileDirPath + fileName;

            if (fileName.find(GIS_RAILROAD_STRING) != string::npos) {
                railRoadLayerPtr->ImportRailRoad(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_RAILROAD);
            } else if (fileName.find(GIS_INTERSECTION_STRING) != string::npos) {
                roadLayerPtr->ImportIntersection(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_INTERSECTION);
            } else if (fileName.find(GIS_ROAD_STRING) != string::npos) {
                roadLayerPtr->ImportRoad(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_ROAD);
            } else if (fileName.find(GIS_AREA_STRING) != string::npos) {
                areaLayerPtr->ImportArea(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_AREA);
            } else if (fileName.find(GIS_PARK_STRING) != string::npos) {
                parkLayerPtr->ImportPark(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_PARK);
            } else if (fileName.find(GIS_BUILDING_STRING) != string::npos) {
                buildingLayerPtr->ImportBuilding(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_BUILDING);
            } else if (fileName.find(GIS_WALL_STRING) != string::npos) {
                buildingLayerPtr->ImportWall(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_WALL);
            } else if (fileName.find(GIS_STATION_STRING) != string::npos) {
                railRoadLayerPtr->ImportStation(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_RAILROAD_STATION);
            } else if ((fileName.find(GIS_OLD_TRAFFIC_LIGHT_STRING) != string::npos) ||
                       (fileName.find(GIS_TRAFFIC_LIGHT_STRING) != string::npos)) {
                roadLayerPtr->ImportTrafficLight(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_TRAFFICLIGHT);
            } else if (fileName.find(GIS_BUSSTOP_STRING) != string::npos) {
                roadLayerPtr->ImportBusStop(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_BUSSTOP);
            } else if (fileName.find(GIS_ENTRANCE_STRING) != string::npos) {
                (*this).ImportEntrance(theParameterDatabaseReader, filePath);
            } else if (fileName.find(GIS_POI_STRING) != string::npos) {
                poiLayerPtr->ImportPoi(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_POI);
            } else if (fileName.find(GIS_GROUND_STRING) != string::npos) {
                groundLayerPtr->ImportGroundSurface(theParameterDatabaseReader, filePath);
                importedGisObjectTypes.insert(GIS_GROUND);
            } else if (fileName.find(GIS_TREE_STRING) != string::npos) {
                groundLayerPtr->ImportTree(theParameterDatabaseReader, filePath);
            } else if (!fileName.empty()) {
                //(*this).ImportGenericLayer(theParameterDatabaseReader, filePath);
            }

            fileNamesWithPriority.pop();
        }
    }

    (*this).CompleteConnections(theParameterDatabaseReader);

    insiteGeometryPtr->AddBuilding(buildingLayerPtr);
    //(*this).OutputVertexInformation();
}

#pragma warning(default:4355)


const GisObject& GisSubsystem::GetGisObject(const GisObjectIdType& objectId) const
{
    typedef map<GisObjectIdType, GisPositionIdType>::const_iterator IterType;

    IterType iter = objectIdMap.find(objectId);

    assert(iter != objectIdMap.end());

    return (*this).GetGisObject((*iter).second);
}

const GisObject& GisSubsystem::GetGisObject(const GisPositionIdType& positionId) const
{
    const GisObjectType& objectType = positionId.type;
    const VariantIdType& variantId = positionId.id;

    switch (objectType) {
    case GIS_AREA: return areaLayerPtr->GetArea(variantId);
    case GIS_POINT: return poiLayerPtr->GetPoint(variantId);
    case GIS_ROAD: return roadLayerPtr->GetRoad(variantId);
    case GIS_INTERSECTION: return roadLayerPtr->GetIntersection(variantId);
    case GIS_RAILROAD: return railRoadLayerPtr->GetRailRoad(variantId);
    case GIS_RAILROAD_INTERSECTION: return railRoadLayerPtr->GetRailRoadIntersection(variantId);
    case GIS_RAILROAD_STATION: return railRoadLayerPtr->GetStation(variantId);
    case GIS_PARK: return parkLayerPtr->GetPark(variantId);
    case GIS_WALL: return buildingLayerPtr->GetWall(variantId);
    case GIS_BUILDING: return buildingLayerPtr->GetBuilding(variantId);
    case GIS_POI: return poiLayerPtr->GetPoi(variantId);
    case GIS_BUSSTOP: return roadLayerPtr->GetBusStop(variantId);
    case GIS_TRAFFICLIGHT: return roadLayerPtr->GetTrafficLight(variantId);
    case GIS_ENTRANCE: return (*this).GetEntrance(variantId);
    default:
        break;
    }

    return genericGisLayerPtrs[objectType]->GetGenericGisObject(variantId);
}

const Vertex& GisSubsystem::GetVertex(const VertexIdType& vertexId) const
{
    return vertices[vertexId].vertex;
}
const GisVertex& GisSubsystem::GetGisVertex(const VertexIdType& vertexId) const
{
    return vertices[vertexId];
}
const Point& GisSubsystem::GetPoint(const PointIdType& pointId) const
{
    return poiLayerPtr->GetPoint(pointId);
}
const Intersection& GisSubsystem::GetIntersection(const IntersectionIdType& intersectionId) const
{
    return roadLayerPtr->GetIntersection(intersectionId);
}
const Road& GisSubsystem::GetRoad(const RoadIdType& roadId) const
{
    return roadLayerPtr->GetRoad(roadId);
}
const Poi& GisSubsystem::GetPoi(const PoiIdType& poiId) const
{
    return poiLayerPtr->GetPoi(poiId);
}
const RailRoad& GisSubsystem::GetRailRoad(const RailRoadIdType& railRoadId) const
{
    return railRoadLayerPtr->GetRailRoad(railRoadId);
}
const RailRoadIntersection& GisSubsystem::GetRailRoadIntersection(const RailRoadIntersectionIdType& railRoadIntersectionId) const
{
    return railRoadLayerPtr->GetRailRoadIntersection(railRoadIntersectionId);
}
const RailRoadStation& GisSubsystem::GetStation(const RailRoadStationIdType& stationId) const
{
    return railRoadLayerPtr->GetStation(stationId);
}
const Area& GisSubsystem::GetArea(const AreaIdType& areaId) const
{
    return areaLayerPtr->GetArea(areaId);
}
const Park& GisSubsystem::GetPark(const ParkIdType& parkId) const
{
    return parkLayerPtr->GetPark(parkId);
}
const Wall& GisSubsystem::GetWall(const WallIdType& wallId) const
{
    return buildingLayerPtr->GetWall(wallId);
}
const Building& GisSubsystem::GetBuilding(const BuildingIdType& buildingId) const
{
    return buildingLayerPtr->GetBuilding(buildingId);
}
const BusStop& GisSubsystem::GetBusStop(const BusStopIdType& busStopId) const
{
    return roadLayerPtr->GetBusStop(busStopId);
}
const TrafficLight& GisSubsystem::GetTrafficLight(const TrafficLightIdType& busStopId) const
{
    return roadLayerPtr->GetTrafficLight(busStopId);
}

bool GisSubsystem::ContainsObject(const GisObjectIdType& objectId) const
{
    return (objectIdMap.find(objectId) != objectIdMap.end());
}

VariantIdType GisSubsystem::GetVariantId(
    const GisObjectType& objectType,
    const GisObjectIdType& objectId) const
{
    typedef map<GisObjectIdType, GisPositionIdType>::const_iterator IterType;

    IterType iter = objectIdMap.find(objectId);

    assert(iter != objectIdMap.end());

    const GisPositionIdType& idPair = (*iter).second;

    assert(idPair.type == objectType);

    return idPair.id;
}

const Point& GisSubsystem::GetPointObject(const GisObjectIdType& objectId) const
{
    return poiLayerPtr->GetPoint((*this).GetVariantId(GIS_POINT, objectId));
}
const Intersection& GisSubsystem::GetIntersectionObject(const GisObjectIdType& objectId) const
{
    return roadLayerPtr->GetIntersection((*this).GetVariantId(GIS_INTERSECTION, objectId));
}
const Road& GisSubsystem::GetRoadObject(const GisObjectIdType& objectId) const
{
    return roadLayerPtr->GetRoad((*this).GetVariantId(GIS_ROAD, objectId));
}
const Poi& GisSubsystem::GetPoiObject(const GisObjectIdType& objectId) const
{
    return poiLayerPtr->GetPoi((*this).GetVariantId(GIS_POI, objectId));
}
const RailRoad& GisSubsystem::GetRailRoadObject(const GisObjectIdType& objectId) const
{
    return railRoadLayerPtr->GetRailRoad((*this).GetVariantId(GIS_RAILROAD, objectId));
}
const RailRoadIntersection& GisSubsystem::GetRailRoadIntersectionObject(const GisObjectIdType& objectId) const
{
    return railRoadLayerPtr->GetRailRoadIntersection((*this).GetVariantId(GIS_RAILROAD_INTERSECTION, objectId));
}
const RailRoadStation& GisSubsystem::GetStationObject(const GisObjectIdType& objectId) const
{
    return railRoadLayerPtr->GetStation((*this).GetVariantId(GIS_RAILROAD_STATION, objectId));
}
const Area& GisSubsystem::GetAreaObject(const GisObjectIdType& objectId) const
{
    return areaLayerPtr->GetArea((*this).GetVariantId(GIS_AREA, objectId));
}
const Park& GisSubsystem::GetParkObject(const GisObjectIdType& objectId) const
{
    return parkLayerPtr->GetPark((*this).GetVariantId(GIS_PARK, objectId));
}
const Wall& GisSubsystem::GetWallObject(const GisObjectIdType& objectId) const
{
    return buildingLayerPtr->GetWall((*this).GetVariantId(GIS_WALL, objectId));
}
const Building& GisSubsystem::GetBuildingObject(const GisObjectIdType& objectId) const
{
    return buildingLayerPtr->GetBuilding((*this).GetVariantId(GIS_BUILDING, objectId));
}

const vector<Point>& GisSubsystem::GetPoints() const
{
    return poiLayerPtr->GetPoints();
}
const vector<Intersection>& GisSubsystem::GetIntersections() const
{
    return roadLayerPtr->GetIntersections();
}
const vector<Road>& GisSubsystem::GetRoads() const
{
    return roadLayerPtr->GetRoads();
}
const vector<BusStop>& GisSubsystem::GetBusStops() const
{
    return roadLayerPtr->GetBusStops();
}
const vector<Poi>& GisSubsystem::GetPois() const
{
    return poiLayerPtr->GetPois();
}
const vector<Building>& GisSubsystem::GetBuildings() const
{
    return buildingLayerPtr->GetBuildings();
}
const vector<Park>& GisSubsystem::GetParks() const
{
    return parkLayerPtr->GetParks();
}
const vector<Wall>& GisSubsystem::GetWalls() const
{
    return buildingLayerPtr->GetWalls();
}
const vector<RailRoadStation>& GisSubsystem::GetStations() const
{
    return railRoadLayerPtr->GetStations();
}
const vector<Area>& GisSubsystem::GetAreas() const
{
    return areaLayerPtr->GetAreas();
}

GisPositionIdType GisSubsystem::GetPosition(
    const string& name,
    const GisObjectType& objectType) const
{
    typedef map<string, map<GisObjectType, set<VariantIdType> > >::const_iterator NameIter;

    const string lowerName = MakeLowerCaseString(name);

    NameIter nameIter = positionsPerName.find(lowerName);

    if (nameIter == positionsPerName.end()) {
        cerr << "Error: There is no position: " << name << endl;
        assert(false);
        exit(1);
    }

    typedef map<GisObjectType, set<VariantIdType> >::const_iterator PositionIter;

    const map<GisObjectType, set<VariantIdType> >& positionIds = (*nameIter).second;

    PositionIter posIter;

    if (objectType == INVALID_OBJECT_TYPE) {
        posIter = positionIds.begin();
    } else {
        posIter = positionIds.find(objectType);
    }

    if (posIter == positionIds.end()) {
        cerr << "Error: Couldn't find gis object: " << name << endl;
        exit(1);
    }

    if ((*posIter).second.size() > 1) {
        cerr << "Error: Duplicated gis object name: " << name << endl;
        exit(1);
    }

    return GisPositionIdType((*posIter).first, *(*posIter).second.begin());
}

void GisSubsystem::GetPositions(
    const string& name,
    vector<GisPositionIdType>& foundPositionIds,
    const GisObjectType& objectType) const
{
    foundPositionIds.clear();

    typedef map<string, map<GisObjectType, set<VariantIdType> > >::const_iterator NameIter;

    const string lowerName = MakeLowerCaseString(name);

    NameIter nameIter = positionsPerName.find(lowerName);

    if (nameIter == positionsPerName.end()) {
        cerr << "Error: There is no position: " << name << endl;
        assert(false);
        exit(1);
    }

    typedef map<GisObjectType, set<VariantIdType> >::const_iterator PositionIter;

    const map<GisObjectType, set<VariantIdType> >& positionIds = (*nameIter).second;

    if (objectType != INVALID_OBJECT_TYPE) {

        if (positionIds.find(objectType) == positionIds.end()) {
            cerr << "Error: Couldn't find gis object: " << name << endl;
            exit(1);
        }
    }

    for(PositionIter posIter = positionIds.begin(); posIter != positionIds.end(); posIter++) {

        const GisObjectType& gisObjectType = (*posIter).first;

        if ((gisObjectType == objectType) ||
            (objectType == INVALID_OBJECT_TYPE/*target is any*/)) {

            const set<VariantIdType>& variantIds = (*posIter).second;

            typedef set<VariantIdType>::const_iterator VariantIdIter;

            for(VariantIdIter iter = variantIds.begin();
                iter != variantIds.end(); iter++) {

                foundPositionIds.push_back(GisPositionIdType((*posIter).first, *iter));
            }
        }
    }
}

bool GisSubsystem::ContainsPosition(const string& name) const
{
    typedef map<string, map<GisObjectType, set<VariantIdType> > >::const_iterator NameIter;

    const string lowerName = MakeLowerCaseString(name);

    return (positionsPerName.find(lowerName) != positionsPerName.end());
}

GisPositionIdType GisSubsystem::GetPositionId(
    const GisObjectIdType& objectId) const
{
    typedef map<GisObjectIdType, GisPositionIdType>::const_iterator IterType;

    IterType iter = objectIdMap.find(objectId);

    if (iter == objectIdMap.end()) {
        cerr << "Error: Couldn't find gis object: " << objectId << endl;
        exit(1);
    }

    return (*iter).second;
}

bool GisSubsystem::IntersectsWith(
    const GisPositionIdType& positionId,
    const Rectangle& rect) const
{
    switch (positionId.type) {
    case GIS_BUILDING:return buildingLayerPtr->GetBuilding(positionId.id).IntersectsWith(rect);
    case GIS_AREA: return areaLayerPtr->GetArea(positionId.id).IntersectsWith(rect);
    case GIS_POINT: return poiLayerPtr->GetPoint(positionId.id).IntersectsWith(rect);
    case GIS_ROAD: return roadLayerPtr->GetRoad(positionId.id).IntersectsWith(rect);
    case GIS_INTERSECTION: return roadLayerPtr->GetIntersection(positionId.id).IntersectsWith(rect);
    case GIS_POI: return poiLayerPtr->GetPoi(positionId.id).IntersectsWith(rect);
    case GIS_RAILROAD: return railRoadLayerPtr->GetRailRoad(positionId.id).IntersectsWith(rect);
    case GIS_RAILROAD_INTERSECTION: return railRoadLayerPtr->GetRailRoadIntersection(positionId.id).IntersectsWith(rect);
    case GIS_RAILROAD_STATION: return  railRoadLayerPtr->GetStation(positionId.id).IntersectsWith(rect);
    case GIS_PARK: return parkLayerPtr->GetPark(positionId.id).IntersectsWith(rect);
    case GIS_WALL: return buildingLayerPtr->GetWall(positionId.id).IntersectsWith(rect);
    case GIS_BUSSTOP: return roadLayerPtr->GetBusStop(positionId.id).IntersectsWith(rect);
    default:
        break;
    }

    return false;
}

void GisSubsystem::RemoveMovingObject(const NodeIdType nodeId)
{
    buildingLayerPtr->RemoveMovingObject(nodeId);
}

GisPositionIdType GisSubsystem::GetPositionId(
    const Vertex& position,
    const vector<GisObjectType>& prioritizedSearchObjectTypes,
    const double integrationLength) const
{
    const Rectangle searchRect(position, integrationLength);

    for(size_t i = 0 ; i < prioritizedSearchObjectTypes.size(); i++) {
        const GisObjectType& objectType = prioritizedSearchObjectTypes[i];

        list<VariantIdType> variantIds;

        (*this).GetSpatialIntersectedGisObjectIds(position, objectType, variantIds);

        typedef list<VariantIdType>::const_iterator IterType;

        for(IterType iter = variantIds.begin(); iter != variantIds.end(); iter++) {
            const VariantIdType& variantId = (*iter);

            if ((*this).IntersectsWith(GisPositionIdType(objectType, variantId), searchRect)) {
                return GisPositionIdType(objectType, variantId);
            }
        }
    }

    return GisPositionIdType();
}

//----------------------------------------------------

void GisSubsystem::GetGisObjectIds(
    const Rectangle& targetRect,
    const GisObjectType& objectType,
    list<VariantIdType>& variantIds) const
{
    variantIds.clear();

    list<VariantIdType> variantIdCandidates;

    (*this).GetSpatialIntersectedGisObjectIds(targetRect, objectType, variantIdCandidates);

    typedef list<VariantIdType>::const_iterator IterType;

    for(IterType iter = variantIdCandidates.begin(); iter != variantIdCandidates.end(); iter++) {
        const VariantIdType& variantId = (*iter);

        if ((*this).IntersectsWith(GisPositionIdType(objectType, variantId), targetRect)) {
            variantIds.push_back(variantId);
        }
    }
}

void GisSubsystem::GetGisObjectIds(
    const Vertex& vertex,
    const GisObjectType& objectType,
    list<VariantIdType>& variantIds) const
{
    const double integrationLength = 0.01;//1cm

    (*this).GetGisObjectIds(Rectangle(vertex, integrationLength), objectType, variantIds);
}

void GisSubsystem::GetGisObjectIds(
    const Rectangle& targetRect,
    const list<GisObjectType>& searchObjectTypes,
    list<GisPositionIdType>& positionIds) const
{
    typedef list<GisObjectType>::const_iterator TypeIter;
    typedef list<VariantIdType>::const_iterator IdIter;

    positionIds.clear();

    for(TypeIter typeIter = searchObjectTypes.begin();
        typeIter != searchObjectTypes.end(); typeIter++) {

        const GisObjectType& type = *typeIter;

        list<VariantIdType> variantIds;

        (*this).GetGisObjectIds(targetRect, type, variantIds);

        for(IdIter idIter = variantIds.begin();
            (idIter != variantIds.end()); idIter++) {
            positionIds.push_back(GisPositionIdType(type, *idIter));
        }
    }
}

void GisSubsystem::GetGisObjectIds(
    const Vertex& vertex,
    const list<GisObjectType>& searchObjectTypes,
    list<GisPositionIdType>& positionIds) const
{
    const double integrationLength = 0.01;//1cm

    (*this).GetGisObjectIds(Rectangle(vertex, integrationLength), searchObjectTypes, positionIds);
}

void GisSubsystem::GetSpatialIntersectedGisObjectIds(
    const Vertex& vertex,
    const GisObjectType& objectType,
    list<VariantIdType>& variantIds) const
{
    const double integrationLength = 0.01;//1cm

    spatialObjectMaps[objectType].GetGisObject(Rectangle(vertex, integrationLength), variantIds);
}

void GisSubsystem::GetSpatialIntersectedGisObjectIds(
    const Rectangle& targetRect,
    const GisObjectType& objectType,
    list<VariantIdType>& variantIds) const
{
    spatialObjectMaps[objectType].GetGisObject(targetRect, variantIds);
}

void GisSubsystem::GetVertexIds(
    const Rectangle& targetRect,
    list<VertexIdType>& variantIds) const
{
    spatialVertexMap.GetGisObject(targetRect, variantIds);
}

void GisSubsystem::ConnectGisObject(
    const VertexIdType& srcVertexId,
    const GisObjectType& objectType,
    const VariantIdType& destVariantId)
{
    (*this).ConnectGisObject(srcVertexId, objectType, srcVertexId, destVariantId);
}

void GisSubsystem::ConnectGisObject(
    const VertexIdType& srcVertexId,
    const GisObjectType& objectType,
    const VertexIdType& destVertexId,
    const VariantIdType& destVariantId)
{
    GisVertex& gisVertex = vertices[srcVertexId];

    gisVertex.connections[objectType].push_back(
        VertexConnection(destVertexId, destVariantId));

    gisVertex.connectionsPerVertex[destVertexId] =
        GisPositionIdType(objectType, destVariantId);

    assert(objectType != GIS_AREA &&
           objectType != GIS_POINT);
}

void GisSubsystem::ConnectBidirectionalGisObject(
    const VertexIdType& vertexId1,
    const GisObjectType& objectType,
    const VertexIdType& vertexId2,
    const VariantIdType& destVariantId)
{
    if (vertexId1 == vertexId2) {

        (*this).ConnectGisObject(
            vertexId1, objectType, destVariantId);

    } else {

        (*this).ConnectGisObject(
            vertexId1, objectType, vertexId2, destVariantId);

        (*this).ConnectGisObject(
            vertexId2, objectType, vertexId1, destVariantId);
    }
}

void GisSubsystem::DisconnectGisObject(
    const VertexIdType& srcVertexId,
    const GisObjectType& objectType,
    const VertexIdType& destVertexId,
    const VariantIdType& destVariantId)
{
    GisVertex& gisVertex = vertices[srcVertexId];
    vector<VertexConnection>& connections = gisVertex.connections[objectType];

    typedef vector<VertexConnection>::iterator IterType;

    const VertexConnection removeConnection(destVertexId, destVariantId);

    for(IterType iter = connections.begin(); iter != connections.end(); iter++) {
        if ((*iter) == removeConnection) {
            connections.erase(iter);
            break;
        }
    }

    gisVertex.connectionsPerVertex.erase(destVertexId);
}

const Entrance& GisSubsystem::GetEntrance(
    const EntranceIdType& entranceId) const
{
    return entrances.at(entranceId);
}

Entrance& GisSubsystem::GetEntrance(
    const EntranceIdType& entranceId)
{
    return entrances[entranceId];
}

void GisSubsystem::DisconnectBidirectionalGisObject(
    const VertexIdType& vertexId1,
    const GisObjectType& objectType,
    const VertexIdType& vertexId2,
    const VariantIdType& destVariantId)
{
    (*this).DisconnectGisObject(vertexId1, objectType, vertexId2, destVariantId);

    (*this).DisconnectGisObject(vertexId2, objectType, vertexId1, destVariantId);
}

void GisSubsystem::UnregisterGisObject(
    const GisObject& gisObject,
    const VariantIdType& variantId)
{
    const GisObjectType objectType = gisObject.GetObjectType();
    const GisObjectIdType objectId = gisObject.GetObjectId();
    const GisPositionIdType positionId(objectType, variantId);

    objectIdMap.erase(objectId);

    gisObject.UpdateMinRectangle();

    spatialObjectMaps[objectType].RemoveGisObject(gisObject, variantId);

    string name = gisObject.GetObjectName();

    if (!name.empty()) {
        positionsPerName[name][objectType].erase(variantId);
    }
}

void GisSubsystem::RegisterGisObject(
    const GisObject& gisObject,
    const VariantIdType& variantId)
{
    const GisObjectType objectType = gisObject.GetObjectType();
    const GisObjectIdType objectId = gisObject.GetObjectId();
    const GisPositionIdType positionId(objectType, variantId);

    objectIdMap[objectId] = positionId;

    gisObject.UpdateMinRectangle();

    spatialObjectMaps[objectType].InsertGisObject(gisObject, variantId);


    string name = gisObject.GetObjectName();

    if (!name.empty()) {
        map<GisObjectType, set<VariantIdType> >& positions = positionsPerName[name];

        if (objectType == GIS_BUSSTOP) {

            if (positions.find(objectType) != positions.end()) {
                if (objectType == GIS_RAILROAD_STATION) {
                    cerr << "Error: Station name [" << name << "] is duplicated." << endl;
                } else {
                    cerr << "Error: BusStop name [" << name << "] is duplicated." << endl;
                }
                exit(1);
            }
        }

        positions[objectType].insert(variantId);
    }
}

void GisSubsystem::CompleteConnections(const ParameterDatabaseReader& theParameterDatabaseReader)
{
    // create LoS topology before adding road.
    buildingLayerPtr->RemakeLosTopology();

    // Additional road connection
    vector<Building>& buildings = buildingLayerPtr->GetBuildings();
    vector<RailRoadStation>& stations = railRoadLayerPtr->GetStations();
    vector<BusStop>& busStops = roadLayerPtr->GetBusStops();
    vector<Park>& parks = parkLayerPtr->GetParks();
    vector<Poi>& pois = poiLayerPtr->GetPois();

    const size_t numberDefaultEntrancesToBuilding = roadLayerPtr->GetNumberOfEntrancesToBuilding();
    const size_t numberDefaultEntrancesToStation = roadLayerPtr->GetNumberOfEntrancesToStation();
    const size_t numberDefaultEntrancesToBusStop = roadLayerPtr->GetNumberOfEntrancesToBusStop();
    const size_t numberDefaultEntrancesToPark = roadLayerPtr->GetNumberOfEntrancesToPark();

    if (roadLayerPtr->GetRoads().empty()) {

        if ((!buildings.empty() && numberDefaultEntrancesToBuilding > 0) ||
            (!stations.empty() && numberDefaultEntrancesToStation > 0) ||
            (!busStops.empty() && numberDefaultEntrancesToBusStop > 0) ||
            (!parks.empty() && numberDefaultEntrancesToPark > 0) ||
            (!pois.empty()) ||
            (!entrances.empty())) {

            cerr << "Error: There is no road to connect an entrance. Place at least one road." << endl;
            exit(1);
        }

    } else {

        typedef set<EntranceIdType>::const_iterator IterType;

        set<RoadIdType> ignoreRoadIds;

        for(BuildingIdType buildingId = 0;
            buildingId < BuildingIdType(buildings.size()); buildingId++) {

            Building& building = buildings[buildingId];
            building.CompleteEntrances(numberDefaultEntrancesToBuilding);

            const set<EntranceIdType>& entranceIds = building.GetEntranceIds();

            for(IterType iter = entranceIds.begin(); iter != entranceIds.end(); iter++) {
                const EntranceIdType& entranceId = (*iter);
                const Vertex entrancePosition = entrances.at(entranceId).GetVertex();

                RoadIdType entranceRoadId;
                RoadIdType newRoadId;
                bool success;

                (*this).GetRoadEntranceCandidates(
                    building.GetMinRectangle(),
                    entrancePosition,
                    ignoreRoadIds,
                    success,
                    entranceRoadId);

                if (success) {
                    VertexIdType intersectionVertexId;

                    buildingLayerPtr->MakeEntrance(buildingId, entrancePosition);
                    roadLayerPtr->MakeDirectPathToPoi(entranceRoadId, entrancePosition, newRoadId, intersectionVertexId);
                    ignoreRoadIds.insert(newRoadId);

                } else {

                    cerr << "Error: There is no road to connect entrance" << (*this).GetEntrance(entranceId).GetObjectName() << " for " << building.GetObjectName() << ". Place at least one road." << endl;
                    exit(1);
                }
            }
        }

        for(RailRoadStationIdType stationId = 0;
            stationId < RailRoadStationIdType(stations.size()); stationId++) {

            RailRoadStation& station = stations[stationId];
            station.CompleteEntrances(numberDefaultEntrancesToStation);

            const set<EntranceIdType>& entranceIds = station.GetEntranceIds();

            for(IterType iter = entranceIds.begin(); iter != entranceIds.end(); iter++) {
                const EntranceIdType& entranceId = (*iter);
                const Vertex entrancePosition = entrances.at(entranceId).GetVertex();

                RoadIdType entranceRoadId;
                RoadIdType newRoadId;
                bool success;

                (*this).GetRoadEntranceCandidates(
                    station.GetMinRectangle(),
                    entrancePosition,
                    ignoreRoadIds,
                    success,
                    entranceRoadId);

                if (success) {
                    VertexIdType intersectionVertexId;

                    railRoadLayerPtr->MakeEntrance(stationId, entrancePosition);
                    roadLayerPtr->MakeDirectPathToPoi(entranceRoadId, entrancePosition, newRoadId, intersectionVertexId);
                    ignoreRoadIds.insert(newRoadId);

                } else {

                    cerr << "Error: There is no road to connect entrance" << (*this).GetEntrance(entranceId).GetObjectName() << " for " << station.GetObjectName() << ". Place at least one road." << endl;
                    exit(1);
                }
            }
        }

        for(BusStopIdType busStopId = 0; busStopId < BusStopIdType(busStops.size()); busStopId++) {

            BusStop& busStop = busStops[busStopId];
            busStop.CompleteEntrances(numberDefaultEntrancesToBusStop);
            busStop.LoadParameters(theParameterDatabaseReader);

            const set<EntranceIdType>& entranceIds = busStop.GetEntranceIds();

            for(IterType iter = entranceIds.begin(); iter != entranceIds.end(); iter++) {
                const EntranceIdType& entranceId = (*iter);
                const Entrance& entrance = entrances.at(entranceId);
                const Vertex entrancePosition = entrance.GetVertex();

                RoadIdType entranceRoadId;
                RoadIdType newRoadId;
                bool success;

                (*this).GetRoadEntranceCandidates(
                    Rectangle(entrancePosition, 1),
                    entrancePosition,
                    ignoreRoadIds,
                    success,
                    entranceRoadId);

                if (success) {
                    VertexIdType intersectionVertexId;

                    roadLayerPtr->MakeDirectPathToPoi(entranceRoadId, entrancePosition, newRoadId, intersectionVertexId);
                    roadLayerPtr->AddBusStopVertex(busStopId, intersectionVertexId);
                    ignoreRoadIds.insert(newRoadId);

                } else {

                    cerr << "Error: There is no road to connect entrance" << (*this).GetEntrance(entranceId).GetObjectName() << " for " << busStop.GetObjectName() << ". Place at least one road." << endl;
                    exit(1);
                }
            }
        }

        for(ParkIdType parkId = 0; parkId < ParkIdType(parks.size()); parkId++) {

            Park& park = parks[parkId];
            park.CompleteEntrances(numberDefaultEntrancesToPark);

            const set<EntranceIdType>& entranceIds = park.GetEntranceIds();

            for(IterType iter = entranceIds.begin(); iter != entranceIds.end(); iter++) {
                const EntranceIdType& entranceId = (*iter);
                const Vertex entrancePosition = entrances.at(entranceId).GetVertex();

                RoadIdType entranceRoadId;
                RoadIdType newRoadId;
                bool success;

                (*this).GetRoadEntranceCandidates(
                    park.GetMinRectangle(),
                    entrancePosition,
                    ignoreRoadIds,
                    success,
                    entranceRoadId);

                if (success) {
                    VertexIdType intersectionVertexId;

                    parkLayerPtr->MakeEntrance(parkId, entrancePosition);
                    roadLayerPtr->MakeDirectPathToPoi(entranceRoadId, entrancePosition, newRoadId, intersectionVertexId);
                    ignoreRoadIds.insert(newRoadId);

                } else {

                    cerr << "Error: There is no road to connect entrance" << (*this).GetEntrance(entranceId).GetObjectName() << " for " << park.GetObjectName() << ". Place at least one road." << endl;
                    exit(1);
                }
            }
        }

        for(PoiIdType poiId = 0; poiId < PoiIdType(pois.size()); poiId++) {
            const Poi& poi = pois[poiId];

            if (poi.IsAPartOfObject()) {
                continue;
            }

            const Vertex entrancePosition = poi.GetVertex();

            RoadIdType entranceRoadId;
            RoadIdType newRoadId;
            bool success;

            (*this).GetRoadEntranceCandidates(
                poi.GetMinRectangle(),
                entrancePosition,
                ignoreRoadIds,
                success,
                entranceRoadId);

            if (success) {
                VertexIdType intersectionVertexId;

                roadLayerPtr->MakeDirectPathToPoi(entranceRoadId, entrancePosition, newRoadId, intersectionVertexId);
                ignoreRoadIds.insert(newRoadId);

            } else {

                cerr << "Error: There is no road to connect POI " << poi.GetObjectName() << ". Place at least one road." << endl;
                exit(1);
            }
        }
    }

    // Rail road completion
    railRoadLayerPtr->ComplementLineInfo();

    if (theParameterDatabaseReader.ParameterExists("gis-public-vehicle-file")) {
        (*this).LoadLineInfo(
            theParameterDatabaseReader.ReadString("gis-public-vehicle-file"));
    }

    roadLayerPtr->SetIntersectionMarginAndMakeLaneConnection();

    //railRoadLayerPtr->MakeRailRoadToStationConnection();

    // final check
    // for(size_t i = 0; i < vertices.size(); i++) {
    //     GisVertex& gisVertex = vertices[i];

    //     const vector<RoadIdType>& roadIds =
    //         gisVertex.GetConnectedObjectIds(GIS_ROAD);

    //     if (roadIds.size() >= 2) {

    //         const vector<IntersectionIdType> intersectionIds =
    //             gisVertex.GetConnectedObjectIds(GIS_INTERSECTION);

    //         if (intersectionIds.empty()) {
    //             roadLayerPtr->CreateIntersection(
    //                 i, (*this).CreateNewObjectId());
    //         }
    //     }
    // }
}


void GisSubsystem::GetRoadEntranceCandidates(
    const Rectangle& minSearchRectangle,
    const Vertex& entrancePosition ,
    const set<RoadIdType>& ignoreRoadIds,
    bool& success,
    RoadIdType& entranceRoadId)
{
    const double maxSearchLength = 100 * 1000; //100km
    double searchLength = 500;
    Rectangle availableRect;
    list<RoadIdType> roadIds;

    typedef list<RoadIdType>::iterator RoadIdIter;

    for(; (roadIds.empty() && searchLength < maxSearchLength); searchLength += 50) {
        availableRect = minSearchRectangle.Expanded(searchLength);
        (*this).GetSpatialIntersectedGisObjectIds(availableRect, GIS_ROAD, roadIds);

        RoadIdIter roadIdIter = roadIds.begin();
        while (roadIdIter != roadIds.end()) {
            if (ignoreRoadIds.find(*roadIdIter) != ignoreRoadIds.end()) {
                roadIdIter = roadIds.erase(roadIdIter);
            } else {
                roadIdIter++;
            }
        }
    }

    typedef list<BuildingIdType>::const_iterator BuildingIdIter;

    double minDistance = DBL_MAX;

    success = false;

    for(RoadIdIter roadIdIter = roadIds.begin(); roadIdIter != roadIds.end(); roadIdIter++) {
        const RoadIdType roadId = (*roadIdIter);
        const Road& road = roadLayerPtr->GetRoad(roadId);
        const Vertex& nearestPosition = road.GetNearestPosition(entrancePosition);
        const double distance = entrancePosition.XYDistanceTo(nearestPosition);

        if (distance < minDistance) {
            minDistance = distance;
            entranceRoadId = roadId;
            success = true;
        }
    }
}

Vertex GisSubsystem::CalculateMedianPoint(const list<VertexIdType>& vertexIds) const
{
    typedef list<VertexIdType>::const_iterator IterType;

    Vertex vertex;

    for(IterType iter = vertexIds.begin(); iter != vertexIds.end(); iter++) {
        vertex += vertices[*iter].vertex;
    }

    return vertex / double(vertexIds.size());
}

GisPositionIdType GisSubsystem::GetConnectedPositionId(
    const VertexIdType& srcVertexId,
    const VertexIdType& destVertexId) const
{
    const GisVertex& srcVertex = vertices[srcVertexId];
    const map<VertexIdType, GisPositionIdType>& connectionsPerVertex = srcVertex.connectionsPerVertex;

    typedef map<VertexIdType, GisPositionIdType>::const_iterator IterType;

    IterType iter = connectionsPerVertex.find(destVertexId);

    assert(iter != connectionsPerVertex.end());

    return (*iter).second;
}

Vertex GisSubsystem::GetPositionVertex(
    const GisPositionIdType& positionId) const
{
    Vertex vertex;

    switch (positionId.type) {
    case GIS_AREA: vertex = areaLayerPtr->GetArea(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_POINT: vertex = poiLayerPtr->GetPoint(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_ROAD: vertex = roadLayerPtr->GetRoad(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_INTERSECTION: vertex = roadLayerPtr->GetIntersection(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_POI: vertex = poiLayerPtr->GetPoi(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_RAILROAD: vertex = railRoadLayerPtr->GetRailRoad(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_RAILROAD_INTERSECTION: vertex = railRoadLayerPtr->GetRailRoadIntersection(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_RAILROAD_STATION: vertex = railRoadLayerPtr->GetStation(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_PARK: vertex = parkLayerPtr->GetPark(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_WALL: vertex = buildingLayerPtr->GetWall(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_BUILDING: vertex = buildingLayerPtr->GetBuilding(positionId.id).GetMinRectangle().GetCenter(); break;
    case GIS_BUSSTOP: vertex = roadLayerPtr->GetBusStop(positionId.id).GetMinRectangle().GetCenter(); break;
    default:
        assert(false);
        break;
    }

    return vertex;
}

void GisSubsystem::GetNearEntranceVertexIds(
    const GisPositionIdType& positionId,
    const Vertex& position,
    vector<VertexIdType>& vertexIds) const
{
    vertexIds.clear();

    switch (positionId.type) {
    case GIS_PARK: parkLayerPtr->GetPark(positionId.id).GetNearEntranceVertexIds(position, vertexIds); break;
    case GIS_BUILDING: buildingLayerPtr->GetBuilding(positionId.id).GetNearEntranceVertexIds(position, vertexIds); break;
    case GIS_POI: vertexIds.push_back(poiLayerPtr->GetPoi(positionId.id).GetVertexId()); break;
    default:
        // no entrance vertex
        break;
    }
}

bool GisSubsystem::HasConnection(
    const VertexIdType& srcVertexId,
    const VertexIdType& destVertexId) const
{
    const GisVertex& srcVertex = vertices[srcVertexId];
    const map<VertexIdType, GisPositionIdType>& connectionsPerVertex = srcVertex.connectionsPerVertex;

    typedef map<VertexIdType, GisPositionIdType>::const_iterator IterType;

    IterType iter = connectionsPerVertex.find(destVertexId);

    return (iter != connectionsPerVertex.end());
}

GisPositionIdType GisSubsystem::GetARandomPosition(
    const GisObjectType& objectType,
    const GisPositionIdType& ignoredDestinationId,
    HighQualityRandomNumberGenerator& aRandomNumberGenerator) const
{
    bool found;
    GisPositionIdType randomPositionId;

    set<GisPositionIdType> ignoredDestinationIds;
    ignoredDestinationIds.insert(ignoredDestinationId);

    (*this).GetARandomPosition(
        objectType,
        ignoredDestinationIds,
        aRandomNumberGenerator,
        found,
        randomPositionId);

    if (found) {
        return randomPositionId;
    }

    return GisPositionIdType();
}

double GisSubsystem::GetElevationMetersAt(
    const Vertex& pos,
    const GisPositionIdType& positionId) const
{
    const double groundElevationMeters = groundLayerPtr->GetElevationMetersAt(pos);
    const GisObjectType& objectType = positionId.type;
    const VariantIdType& variantId = positionId.id;

    double elevationFromGroundMeters = 0.0;

    switch (objectType) {
    case GIS_ROAD: elevationFromGroundMeters = roadLayerPtr->GetRoad(variantId).GetElevationFromGroundMeters(); break;
    case GIS_BUILDING: elevationFromGroundMeters = buildingLayerPtr->GetBuilding(variantId).GetElevationFromGroundMeters(); break;
    case GIS_PARK: elevationFromGroundMeters = parkLayerPtr->GetPark(variantId).GetElevationFromGroundMeters(); break;
    case GIS_ENTRANCE: elevationFromGroundMeters = (*this).GetEntrance(variantId).GetElevationFromGroundMeters(); break;
    case GIS_BUSSTOP: roadLayerPtr->GetBusStop(variantId).GetElevationFromGroundMeters(); break;
    case GIS_WALL: elevationFromGroundMeters = buildingLayerPtr->GetWall(variantId).GetElevationFromGroundMeters(); break;
    case GIS_AREA: elevationFromGroundMeters = areaLayerPtr->GetArea(variantId).GetElevationFromGroundMeters(); break;
    case GIS_POINT: elevationFromGroundMeters = poiLayerPtr->GetPoint(variantId).GetElevationFromGroundMeters(); break;
    case GIS_INTERSECTION: elevationFromGroundMeters = roadLayerPtr->GetIntersection(variantId).GetElevationFromGroundMeters(); break;
    case GIS_RAILROAD: elevationFromGroundMeters = railRoadLayerPtr->GetRailRoad(variantId).GetElevationFromGroundMeters(); break;
    case GIS_RAILROAD_INTERSECTION: elevationFromGroundMeters = railRoadLayerPtr->GetRailRoadIntersection(variantId).GetElevationFromGroundMeters(); break;
    case GIS_RAILROAD_STATION: elevationFromGroundMeters = railRoadLayerPtr->GetStation(variantId).GetElevationFromGroundMeters(); break;
    case GIS_TRAFFICLIGHT: elevationFromGroundMeters = roadLayerPtr->GetTrafficLight(variantId).GetElevationFromGroundMeters(); break;
    case GIS_POI: elevationFromGroundMeters = poiLayerPtr->GetPoi(variantId).GetElevationFromGroundMeters(); break;
    }

    return groundElevationMeters + elevationFromGroundMeters;
}

void GisSubsystem::GetARandomPosition(
    const GisObjectType& objectType,
    const set<GisPositionIdType>& ignoredDestinationIds,
    HighQualityRandomNumberGenerator& aRandomNumberGenerator,
    bool& found,
    GisPositionIdType& randomPositionId) const
{
    vector<GisPositionIdType> candidatePositionIds;

    switch(objectType) {
    case GIS_BUILDING: {
        const vector<Building>& buildings = buildingLayerPtr->GetBuildings();

        for(size_t i = 0; i < buildings.size(); i++) {
            const GisPositionIdType positionId(GIS_BUILDING, buildings[i].GetBuildingId());

            if (ignoredDestinationIds.find(positionId) == ignoredDestinationIds.end()) {
                candidatePositionIds.push_back(positionId);
            }
        }
        break;
    }

    case GIS_PARK: {
        const vector<Park>& parks = parkLayerPtr->GetParks();

        for(size_t i = 0; i < parks.size(); i++) {
            const GisPositionIdType positionId(GIS_PARK, parks[i].GetParkId());

            if (ignoredDestinationIds.find(positionId) == ignoredDestinationIds.end()) {
                candidatePositionIds.push_back(positionId);
            }
        }
        break;
    }

    case GIS_POI: {
        const vector<Poi>& pois = poiLayerPtr->GetPois();

        for(size_t i = 0; i < pois.size(); i++) {
            const GisPositionIdType positionId(GIS_POI, pois[i].GetPoiId());

            if (ignoredDestinationIds.find(positionId) == ignoredDestinationIds.end()) {
                candidatePositionIds.push_back(positionId);
            }
        }
        break;
    }

    case GIS_INTERSECTION: {
        const vector<Intersection>& intersections = roadLayerPtr->GetIntersections();

        for(size_t i = 0; i < intersections.size(); i++) {
            const Intersection& intersection = intersections[i];

            if (!intersection.GetObjectName().empty()) {
                const GisPositionIdType positionId(GIS_INTERSECTION, intersection.GetIntersectionId());

                if (ignoredDestinationIds.find(positionId) == ignoredDestinationIds.end()) {
                    candidatePositionIds.push_back(positionId);
                }
            }
        }
        break;
    }

    default:
        break;
    }

    if (candidatePositionIds.empty()) {
        found = false;
        randomPositionId = GisPositionIdType();
    } else {
        found = true;

        if (candidatePositionIds.size() == 1) {
            randomPositionId = candidatePositionIds.front();
        } else {
            randomPositionId = candidatePositionIds[aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(candidatePositionIds.size() - 1))];
        }
    }
}

GisObjectIdType GisSubsystem::GetARandomGisObjectId(
    const GisObjectType& objectType,
    RandomNumberGenerator& aRandomNumberGenerator) const
{
    switch(objectType) {
    case GIS_ROAD: {
        const vector<Road>& roads = roadLayerPtr->GetRoads();
        return roads[aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(roads.size() - 1))].GetObjectId();
    }

    case GIS_INTERSECTION: {
        const vector<Intersection>& intersections = roadLayerPtr->GetIntersections();
        return intersections[aRandomNumberGenerator.GenerateRandomInt(0, static_cast<int32_t>(intersections.size() - 1))].GetObjectId();
    }

    default:
        break;
    }

    assert(false);

    return INVALID_GIS_OBJECT_ID;
}

void GisSubsystem::GetBuildingPositionIdsInArea(
    const AreaIdType& areaId,
    vector<GisPositionIdType>& buildingPositionIds) const
{
    set<GisPositionIdType> ignoredDestinationIds;

    (*this).GetBuildingPositionIdsInArea(areaId, ignoredDestinationIds, buildingPositionIds);
}

void GisSubsystem::GetBuildingPositionIdsInArea(
    const AreaIdType& areaId,
    const set<GisPositionIdType>& ignoredDestinationIds,
    vector<GisPositionIdType>& buildingPositionIds) const
{
    buildingPositionIds.clear();

    const vector<Building>& buildings = buildingLayerPtr->GetBuildings();

    const Area area = areaLayerPtr->GetArea(areaId);
    const Rectangle& areaRect = area.GetMinRectangle();
    const vector<Vertex>& areaPolygon = area.GetPolygon();

    list<BuildingIdType> buildingIds;

    (*this).GetSpatialIntersectedGisObjectIds(areaRect, GIS_BUILDING, buildingIds);

    typedef list<BuildingIdType>::const_iterator IterType;

    bool foundABuilding = false;

    for(IterType iter = buildingIds.begin(); iter != buildingIds.end(); iter++) {

        const BuildingIdType& buildingId = (*iter);
        const Building& building = buildings[buildingId];
        const GisPositionIdType positionId(GIS_BUILDING, *iter);

        if (ignoredDestinationIds.find(positionId) != ignoredDestinationIds.end()) {
            continue;
        }

        if (CompleteCoveredPolygon(areaPolygon, building.GetBuildingPolygon())) {

            foundABuilding = true;
            buildingPositionIds.push_back(positionId);
        }
    }

    if (!foundABuilding) {
        cerr << "Error: Specified area " << area.GetObjectName() << " doesn't contain any building." << endl;
        exit(1);
    }

}

VertexIdType GisSubsystem::GetNearestVertexId(
    const GisPositionIdType& positionId,
    const Vertex& position) const
{
    if (positionId.type == GIS_BUILDING) {
        const Building& building = buildingLayerPtr->GetBuilding(positionId.id);
        return building.GetNearestEntranceVertexId(position);

    } else if (positionId.type == GIS_ROAD) {
        const Road& road = roadLayerPtr->GetRoad(positionId.id);
        return road.GetNearestVertexId(position);

    } else if (positionId.type == GIS_PARK) {
        const Park& park = parkLayerPtr->GetPark(positionId.id);
        return park.GetNearestEntranceVertexId(position);
    }

    assert(false);
    return INVALID_VERTEX_ID;
}

bool GisSubsystem::IsParkingVertex(const VertexIdType& vertexId) const
{
    const vector<RoadIdType> connectedRoadIds =
        vertices[vertexId].GetConnectedObjectIds(GIS_ROAD);

    for(size_t i = 0; i < connectedRoadIds.size(); i++) {
        if ((*this).GetRoad(connectedRoadIds[i]).IsParking()) {
            return true;
        }
    }

    return false;
}


RoadIdType GisSubsystem::GetParkingRoadId(const VertexIdType& vertexId) const
{
    const vector<RoadIdType> connectedRoadIds =
        vertices[vertexId].GetConnectedObjectIds(GIS_ROAD);

    for(size_t i = 0; i < connectedRoadIds.size(); i++) {
        const RoadIdType& roadId = connectedRoadIds[i];

        if ((*this).GetRoad(roadId).IsParking()) {
            return roadId;
        }
    }

    assert(false);

    return INVALID_VARIANT_ID;
}

RoadIdType GisSubsystem::GetRoadId(const VertexIdType& vertexId) const
{
    return vertices[vertexId].GetConnectedObjectId(GIS_ROAD);
}

PoiIdType GisSubsystem::GetPoiId(const VertexIdType& vertexId) const
{
    return vertices[vertexId].GetConnectedObjectId(GIS_POI);
}

IntersectionIdType GisSubsystem::GetIntersectionId(
    const VertexIdType& vertexId) const
{
    return vertices[vertexId].GetConnectedObjectId(GIS_INTERSECTION);
}

RailRoadStationIdType GisSubsystem::GetStationId(
    const VertexIdType& vertexId) const
{
    return vertices[vertexId].GetConnectedObjectId(GIS_RAILROAD_STATION);
}

BusStopIdType GisSubsystem::GetBusStopId(
    const VertexIdType& vertexId) const
{
    return vertices[vertexId].GetConnectedObjectId(GIS_BUSSTOP);
}

bool GisSubsystem::IsIntersectionVertex(
    const VertexIdType& vertexId) const
{
    return (*this).IsVertexOf(GIS_INTERSECTION, vertexId);
}

bool GisSubsystem::IsVertexOf(
    const GisObjectType& objectType,
    const VertexIdType& vertexId) const
{
    const GisVertex& gisVertex = vertices[vertexId];

    typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

    IterType iter = gisVertex.connections.find(objectType);

    if (iter == gisVertex.connections.end()) {
        return false;
    }

    return !(*iter).second.empty();
}

bool GisSubsystem::VertexContains(
    const GisPositionIdType& positionId,
    const VertexIdType& vertexId) const
{
    const GisVertex& gisVertex = vertices[vertexId];

    typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

    IterType iter = gisVertex.connections.find(positionId.type);

    if (iter == gisVertex.connections.end()) {
        return false;
    }

    const vector<VertexConnection>& vertexConnections = (*iter).second;

    for(size_t i = 0; i < vertexConnections.size(); i++) {
        if (vertexConnections[i].variantId == positionId.id) {
            return true;
        }
    }

    return false;
}


double GisSubsystem::CalculateDistance(
    const VertexIdType& vertexId1,
    const VertexIdType& vertexId2) const
{
    return vertices[vertexId1].vertex.DistanceTo(vertices[vertexId2].vertex);
}

shared_ptr<const GenericGisLayer> GisSubsystem::GetGenericLayerPtr(const string& layerName) const
{
    typedef map<string, GenericGisLayerIdType>::const_iterator IterType;

    IterType iter = genericGisLayerIds.find(layerName);

    if (iter != genericGisLayerIds.end()) {
        return (*this).GetGenericLayerPtr((*iter).second);
    }

    return shared_ptr<const GenericGisLayer>();
}

MaterialIdType GisSubsystem::GetMaterialId(const string& materialName) const
{
    return materials.GetMaterialId(materialName);
}

const Material& GisSubsystem::GetMaterial(const MaterialIdType& materialId) const
{
    return materials.GetMaterial(materialId);
}

void GisSubsystem::LoadLocalMaterials(const string& materialFilePath)
{
    ifstream inFile(materialFilePath.c_str());

    if (!inFile.good()) {
        cerr << "Could Not open material file: " << materialFilePath << endl;
        exit(1);
    }

    while(!inFile.eof()) {
        string aLine;
        getline(inFile, aLine);

        if (IsAConfigFileCommentLine(aLine)) {
            continue;
        }

        DeleteTrailingSpaces(aLine);
        istringstream lineStream(aLine);

        string materialName;
        string materialType;
        double transmissionLossDb;

        lineStream >> materialName >> materialType;
        ConvertStringToLowerCase(materialType);

        if (materialType == "cost231indoor") {

            lineStream >> transmissionLossDb;
            materials.AddMaterial(materialName, transmissionLossDb);

        } else if (materials.GetMaterial(materialName).name.empty()) {
            materials.AddMaterial(materialName, 0);
        }

        if (lineStream.fail()) {
            cerr << "Error: Bad material file line: " << aLine << endl;
            exit(1);
        }
    }
}

bool GisSubsystem::IsVertexPoint(const Vertex& point) const
{
    Vertex vertex = point;

    if (positionInLatlongDegree) {
        vertex = vertex.ToXyVertex(
            latitudeOriginDegrees, longitudeOriginDegrees);
    }

    list<VariantIdType> vertexIds;

    const double integrationLength = 0.01;//1cm

    spatialVertexMap.GetGisObject(
        Rectangle(vertex, integrationLength), vertexIds);

    typedef list<VertexIdType>::const_iterator IterType;

    for(IterType iter = vertexIds.begin(); iter != vertexIds.end(); iter++) {
        const VertexIdType& vertexId = *iter;

        if (vertex.DistanceTo(vertices[vertexId].vertex) <= integrationLength) {
            return true;
        }
    }

    return false;
}

VertexIdType GisSubsystem::GetVertexId(const Vertex& baseVertex)
{
    Vertex vertex = baseVertex;

    if (positionInLatlongDegree) {
        vertex = vertex.ToXyVertex(
            latitudeOriginDegrees, longitudeOriginDegrees);
    }

    list<VariantIdType> vertexIds;

    const double integrationLength = 0.01;//1cm

    spatialVertexMap.GetGisObject(
        Rectangle(vertex, integrationLength), vertexIds);

    typedef list<VertexIdType>::const_iterator IterType;

    for(IterType iter = vertexIds.begin(); iter != vertexIds.end(); iter++) {
        const VertexIdType& vertexId = *iter;

        if (vertex.DistanceTo(vertices[vertexId].vertex) <= integrationLength) {
            return vertexId;
        }
    }

    VertexIdType vertexId = static_cast<VertexIdType>(vertices.size());

    vertices.push_back(GisVertex(vertex));
    spatialVertexMap.InsertVertex(vertex, vertexId);

    return vertexId;
}

void GisSubsystem::SynchronizeTopology(const TimeType& currentTime)
{
    changedGisTopology = false;

    insiteGeometryPtr->SyncMovingObjectTime(currentTime);

    buildingLayerPtr->SyncMovingObjectTime(currentTime);

    roadLayerPtr->SyncTrafficLight(currentTime);

    if (!executeEventsManually) {
        (*this).ExecuteEvents(currentTime);
    }
}

void GisSubsystem::AddGisChangeEventHandler(
    const string& instanceId,
    const shared_ptr<GisChangeEventHandler>& initGisChangeEventHandlerPtr)
{
    gisChangeEventHandlerPtrs[instanceId] = initGisChangeEventHandlerPtr;
}

void GisSubsystem::DeleteGisChangeEventHandler(
    const string& instanceId)
{
    gisChangeEventHandlerPtrs.erase(instanceId);
}

void GisSubsystem::ExecuteEvents(const TimeType& currentTime)
{
    bool changed = false;

    while (!gisEventInfos.empty() &&
           gisEventInfos.top().eventTime <= currentTime) {

        shared_ptr<SimulationEvent> eventPtr = gisEventInfos.top().eventPtr;

        gisEventInfos.pop();

        eventPtr->ExecuteEvent();

        changed = true;
    }

    if (changed) {
        typedef map<string, shared_ptr<GisChangeEventHandler> >::const_iterator IterType;

        for(IterType iter = gisChangeEventHandlerPtrs.begin();
            iter != gisChangeEventHandlerPtrs.end(); iter++) {

            (*iter).second->GisInformationChanged();
        }
    }
}

GisObjectIdType GisSubsystem::CreateNewObjectId()
{
    reservedObjectId++;
    return reservedObjectId;
}

EntranceIdType GisSubsystem::CreateEntrance(const Vertex& vertex)
{
    const EntranceIdType entranceId = static_cast<EntranceIdType>(entrances.size());
    const GisObjectIdType objectId = (*this).CreateNewObjectId();
    const VertexIdType vertexId = (*this).GetVertexId(vertex);

    entrances.push_back(Entrance(this, objectId, entranceId, vertexId));

    return entranceId;
}

void GisSubsystem::ImportEntrance(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder referedObjectIdFinder(hDBF, GIS_DBF_OBJECTID_STRING);

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        assert(shpObjPtr->nParts == 0);
        assert(shpObjPtr->nVertices == 1);

        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);
        const double x = shpObjPtr->padfX[0];
        const double y = shpObjPtr->padfY[0];
        double z = shpObjPtr->padfZ[0];

        if (ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId)) {
            z += (*this).GetGroundElevationMetersAt(x, y);
        }

        Vertex point(x, y, z);

        const EntranceIdType entranceId = static_cast<EntranceIdType>(entrances.size());

        GisObjectIdType referedObjectId;
        bool foundLocation = true;

        if (referedObjectIdFinder.IsAvailable()) {
            referedObjectId = referedObjectIdFinder.GetGisObjectId(entryId);

            if (objectIdMap.find(referedObjectId) != objectIdMap.end()) {
                const GisPositionIdType positionId = (*this).GetPositionId(referedObjectId);

                switch (positionId.type) {
                case GIS_PARK: point = parkLayerPtr->GetPark(positionId.id).AddEntrance(entranceId, point); break;
                case GIS_BUILDING: point = buildingLayerPtr->GetBuilding(positionId.id).AddEntrance(entranceId, point); break;
                case GIS_RAILROAD_STATION: point = railRoadLayerPtr->GetStation(positionId.id).AddEntrance(entranceId, point); break;
                case GIS_BUSSTOP: point = roadLayerPtr->GetBusStop(positionId.id).AddEntrance(entranceId, point); break;
                default:
                    foundLocation = false;
                break;
                }
            } else {
                foundLocation = false;
            }
        } else {
            foundLocation = false;
        }

        if (!foundLocation) {
            list<RailRoadStationIdType> stationIds;
            list<ParkIdType> parkIds;
            list<BuildingIdType> buildingIds;

            (*this).GetSpatialIntersectedGisObjectIds(point, GIS_PARK, parkIds);
            (*this).GetSpatialIntersectedGisObjectIds(point, GIS_BUILDING, buildingIds);
            (*this).GetSpatialIntersectedGisObjectIds(point, GIS_RAILROAD_STATION, stationIds);

            typedef list<VariantIdType>::const_iterator IterType;

            for(IterType iter = parkIds.begin(); iter != parkIds.end(); iter++) {
                Park& park = parkLayerPtr->GetPark(*iter);
                if (PolygonContainsPoint(park.GetPolygon(), point)) {
                    point = park.AddEntrance(entranceId, point);
                }
            }
            for(IterType iter = buildingIds.begin(); iter != buildingIds.end(); iter++) {
                Building& building = buildingLayerPtr->GetBuilding(*iter);
                if (PolygonContainsPoint(building.GetBuildingPolygon(), point)) {
                    point = building.AddEntrance(entranceId, point);
                }
            }
            for(IterType iter = stationIds.begin(); iter != stationIds.end(); iter++) {
                RailRoadStation& station = railRoadLayerPtr->GetStation(*iter);
                if (PolygonContainsPoint(station.GetPolygon(), point)) {
                    point = station.AddEntrance(entranceId, point);
                }
            }
        }

        const VertexIdType& vertexId = (*this).GetVertexId(point);
        entrances.push_back(Entrance(this, objectId, entranceId, vertexId));
        Entrance& entrance = entrances.back();

        if (nameFinder.IsAvailable()) {
            entrance.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }

        (*this).ConnectGisObject(vertexId, GIS_ENTRANCE, entranceId);

        (*this).RegisterGisObject(entrance, entranceId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

void PoiLayer::ImportPoi(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& filePath)
{
    SHPHandle hSHP;
    DBFHandle hDBF;
    int entities;
    Rectangle layerRect;

    GetStandardShapeInfo(filePath, subsystemPtr->isDebugMode, hSHP, hDBF, entities, layerRect);

    const AttributeFinder idFinder(hDBF, GIS_DBF_ID_STRING);
    const AttributeFinder nameFinder(hDBF, GIS_DBF_NAME_STRING);
    const AttributeFinder capacityFinder(hDBF, GIS_DBF_CAPACITY_STRING);
    const AttributeFinder infoFinder(hDBF, GIS_DBF_INFO_STRING);

    const ParkLayer& parkLayer = *subsystemPtr->GetParkLayerPtr();
    const BuildingLayer& buildingLayer = *subsystemPtr->GetBuildingLayerPtr();

    for(int entryId = 0; entryId < entities; entryId++) {
        SHPObject* shpObjPtr = SHPReadObject(hSHP, entryId);

        const PoiIdType poiId = static_cast<PoiIdType>(pois.size());
        const GisObjectIdType objectId = idFinder.GetGisObjectId(entryId);

        const double x = shpObjPtr->padfX[0];
        const double y = shpObjPtr->padfY[0];
        double z = shpObjPtr->padfZ[0];

        if (ElevationBaseIsGroundLevel(theParameterDatabaseReader, objectId)) {
            z += subsystemPtr->GetGroundElevationMetersAt(x, y);
        }

        const Vertex point(x, y, z);
        const VertexIdType& vertexId = subsystemPtr->GetVertexId(point);

        pois.push_back(Poi(subsystemPtr, objectId, poiId, vertexId));
        Poi& poi = pois.back();
        poi.LoadParameters(theParameterDatabaseReader);

        if (nameFinder.IsAvailable()) {
            poi.commonImplPtr->objectName = nameFinder.GetLowerString(entryId);
        }
        if (infoFinder.IsAvailable()) {
            poi.information = infoFinder.GetString(entryId);
        }
        if (shpObjPtr->nVertices > 0) {
            poi.commonImplPtr->elevationFromGroundMeters = shpObjPtr->padfZ[0];
        }

        list<ParkIdType> parkIds;
        subsystemPtr->GetSpatialIntersectedGisObjectIds(point, GIS_PARK, parkIds);

        const double integrationLength = 0.01;//1cm
        const double elevationFromGroundMeters = poi.commonImplPtr->elevationFromGroundMeters;

        typedef list<VariantIdType>::const_iterator IterType;

        for(IterType iter = parkIds.begin(); iter != parkIds.end(); iter++) {
            const Park& park = parkLayer.GetPark(*iter);
            if (PolygonContainsPoint(park.GetPolygon(), point) &&
                std::fabs(park.GetElevationFromGroundMeters() - elevationFromGroundMeters) <= integrationLength) {
                poi.parentPositionId = GisPositionIdType(GIS_PARK, *iter);
                break;
            }
        }

        if (poi.parentPositionId.IsInvalid()) {
            list<BuildingIdType> buildingIds;
            subsystemPtr->GetSpatialIntersectedGisObjectIds(point, GIS_BUILDING, buildingIds);

            for(IterType iter = buildingIds.begin(); iter != buildingIds.end(); iter++) {
                const Building& building = buildingLayer.GetBuilding(*iter);
                if (PolygonContainsPoint(building.GetBuildingPolygon(), point) &&
                std::fabs(building.GetElevationFromGroundMeters() - elevationFromGroundMeters) <= integrationLength) {
                    poi.parentPositionId = GisPositionIdType(GIS_BUILDING, *iter);
                    break;
                }
            }
        }

        if (poi.parentPositionId.IsInvalid()) {
            if (capacityFinder.IsAvailable()) {
                poi.humanCapacity = capacityFinder.GetInt(entryId);
            }
        }

        subsystemPtr->ConnectGisObject(vertexId, GIS_POI, poiId);

        subsystemPtr->RegisterGisObject(poi, poiId);

        SHPDestroyObject(shpObjPtr);
    }

    SHPClose(hSHP);
    DBFClose(hDBF);
}

void GisSubsystem::OutputVertexInformation() const
{
    static const char* GIS_OBJECTTYPE_NAMES[] = {
        "Area",
        "Point",
        "Road",
        "Intersection",
        "RailRoad",
        "RailRoadIntersection",
        "RailRoadStation",
        "Park",
        "Wall",
        "Building",
        "PedestrianlPath",
        "BusStop",
        "Poi"
    };

    for(size_t i = 0; i < vertices.size(); i++) {
        const GisVertex& gisVertex = vertices[i];

        typedef map<GisObjectType, vector<VertexConnection> >::const_iterator IterType;

        const map<GisObjectType, vector<VertexConnection> >& connections = gisVertex.connections;

        cout << "vertex" << i << "(" << gisVertex.vertex.x << "," << gisVertex.vertex.y << "," << gisVertex.vertex.z << ")";
        for(IterType iter = connections.begin(); iter != connections.end(); iter++) {
            const GisObjectType& objectType = (*iter).first;
            const vector<VertexConnection>& vertexConnections = (*iter).second;

            for(size_t j = 0; j < vertexConnections.size(); j++) {
                const VertexConnection& vertexConnection = vertexConnections[j];

                cout << " ";

                if (objectType < GisObjectType(sizeof(GIS_OBJECTTYPE_NAMES)/sizeof(GIS_OBJECTTYPE_NAMES[0]))) {
                    cout << GIS_OBJECTTYPE_NAMES[objectType];
                }

                cout << vertexConnection.variantId << "(" << vertexConnection.vertexId << ")";
            }
        }

        cout << endl;
    }
}

void GisSubsystem::SetEnabled(
    const GisObjectType& objectType,
    const VariantIdType& variantId,
    const bool isEnable)
{
    changedGisTopology = true;

    switch (objectType) {
    case GIS_ROAD: roadLayerPtr->GetRoad(variantId).SetEnabled(isEnable); break;
    case GIS_PARK: parkLayerPtr->GetPark(variantId).SetEnabled(isEnable); break;
    case GIS_AREA: areaLayerPtr->GetArea(variantId).SetEnabled(isEnable); break;
    case GIS_POINT: poiLayerPtr->GetPoint(variantId).SetEnabled(isEnable); break;
    case GIS_INTERSECTION: roadLayerPtr->GetIntersection(variantId).SetEnabled(isEnable); break;
    case GIS_RAILROAD: railRoadLayerPtr->GetRailRoad(variantId).SetEnabled(isEnable); break;
    case GIS_RAILROAD_INTERSECTION: railRoadLayerPtr->GetRailRoadIntersection(variantId).SetEnabled(isEnable); break;
    case GIS_RAILROAD_STATION: railRoadLayerPtr->GetStation(variantId).SetEnabled(isEnable); break;
    case GIS_TRAFFICLIGHT: roadLayerPtr->GetTrafficLight(variantId).SetEnabled(isEnable); break;
    case GIS_POI: poiLayerPtr->GetPoi(variantId).SetEnabled(isEnable); break;
    case GIS_BUILDING: buildingLayerPtr->GetBuilding(variantId).SetEnabled(isEnable); break;
    case GIS_WALL: buildingLayerPtr->GetWall(variantId).SetEnabled(isEnable); break;
    case GIS_BUSSTOP: roadLayerPtr->GetBusStop(variantId).SetEnabled(isEnable); break;
    case GIS_ENTRANCE: (*this).GetEntrance(variantId).SetEnabled(isEnable); break;
        
    default:
        assert(false);
        break;
    }
}

void GisSubsystem::EnableMovingObjectIfNecessary(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const TimeType& currentTime,
    const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
    const string& stationTypeString)
{
    if (theParameterDatabaseReader.ParameterExists("object-shape-type", nodeId)) {
        string shapeType = theParameterDatabaseReader.ReadString("object-shape-type", nodeId);
        ConvertStringToLowerCase(shapeType);

        buildingLayerPtr->AddMovingObject(materials, nodeId, mobilityModelPtr, shapeType);
    }

    insiteGeometryPtr->EnableMovingObjectWithAntennaIfNecessary(
        theParameterDatabaseReader,
        nodeId,
        currentTime,
        mobilityModelPtr,
        stationTypeString);
}

}; //namespace ScenSim
