// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_SHADOWING_H
#define SCENSIM_SHADOWING_H

#include "scensim_proploss.h"


namespace ScenSim {

using std::istringstream;

typedef unsigned int SiteIdType;

const SiteIdType COMMON_SITEID = 0;

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//mesh data structure for shadowing. eventually map/mesh-based pathloss and elevation
//

class MeshData {
public:
    MeshData(
        const string& initiDataTypeName,
        const double& initReferencePointX,
        const double& initReferencePointY,
        const double& initPointIntervalMeters,
        const size_t initNumberOfXPoints,
        const size_t initNumberOfYPoints,
        const unsigned int initDataIndex);

    void GetLinearInterpolatedValue(
        const double positionX,
        const double positionY,
        double& value,
        bool& success) const;

    void ReadALineAsConsecutiveData(const string& aLine);

private:
    string dataTypeName;

    double referencePointX;
    double referencePointY;

    double pointIntervalMeters;
    size_t numberOfXPoints;
    size_t numberOfYPoints;

    unsigned int dataIndex;

    bool dataIsCompleted;

    //raw mesh data
    vector<double> meshData;

    bool InsideMeshArea(
        const double& positionX,
        const double& positionY) const;


};//MeshData//


inline
MeshData::MeshData(
    const string& initiDataTypeName,
    const double& initReferencePointX,
    const double& initReferencePointY,
    const double& initPointIntervalMeters,
    const size_t initNumberOfXPoints,
    const size_t initNumberOfYPoints,
    const unsigned int initDataIndex)
    :
    dataTypeName(initiDataTypeName),
    referencePointX(initReferencePointX),
    referencePointY(initReferencePointY),
    pointIntervalMeters(initPointIntervalMeters),
    numberOfXPoints(initNumberOfXPoints),
    numberOfYPoints(initNumberOfYPoints),
    dataIndex(initDataIndex),
    dataIsCompleted(false)
{ 
    assert(initPointIntervalMeters > 0);
    assert(initNumberOfXPoints >= 1);
    assert(initNumberOfYPoints >= 1);
}


inline
bool MeshData::InsideMeshArea(
    const double& positionX,
    const double& positionY) const
{

    if ((positionX < referencePointX) ||
        (positionX > (referencePointX + (pointIntervalMeters * (numberOfXPoints - 1)))) ||
        (positionY < referencePointY) ||
        (positionY > (referencePointY + (pointIntervalMeters * (numberOfYPoints - 1))))) {
        return false;
    }

    return true;

}//InsideMeshArea//


inline
void MeshData::GetLinearInterpolatedValue(
    const double positionX,
    const double positionY,
    double& value,
    bool& success) const
{
    assert(dataIsCompleted);

    if(!InsideMeshArea(positionX, positionY)) {
        success = false;
        return;
    }//if//

    //liner interpolation

    const size_t lowerXPoint =
        static_cast<size_t>(floor((positionX - referencePointX) / pointIntervalMeters));
    const double remainingXRatio =
        (positionX - referencePointX - (pointIntervalMeters * lowerXPoint)) / pointIntervalMeters;
    
    const size_t lowerYPoint =
        static_cast<size_t>(floor((positionY - referencePointY) / pointIntervalMeters));
    const double remainingYRatio =
        (positionY - referencePointY - (pointIntervalMeters * lowerYPoint)) / pointIntervalMeters;

    assert(lowerXPoint <= (numberOfXPoints - 1));
    assert((remainingXRatio >= 0.0) && (remainingXRatio <= 1.0));
    assert(lowerYPoint <= (numberOfYPoints - 1));
    assert((remainingYRatio >= 0.0) && (remainingYRatio <= 1.0));

    const size_t lowerLeftIndex = numberOfXPoints * lowerYPoint + lowerXPoint;
    const size_t lowerRightIndex = lowerLeftIndex + 1;
    const size_t upperLeftIndex = lowerLeftIndex + numberOfXPoints;
    const size_t upperRightIndex = upperLeftIndex + 1;

    if ((lowerXPoint != (numberOfXPoints - 1)) && (lowerYPoint != (numberOfYPoints - 1))) {

        //regular
        value =
            (1.0 - remainingXRatio) * (1.0 - remainingYRatio) * meshData[lowerLeftIndex] +
            remainingXRatio * (1.0 - remainingYRatio) * meshData[lowerRightIndex] +
            (1.0 - remainingXRatio) * remainingYRatio * meshData[upperLeftIndex] +
            remainingXRatio * remainingYRatio * meshData[upperRightIndex];

    }
    else if ((lowerXPoint == (numberOfXPoints - 1)) && (lowerYPoint != (numberOfYPoints - 1))) {

        //x max border
        assert(remainingXRatio == 0.0);

        value =
            (1.0 - remainingXRatio) * (1.0 - remainingYRatio) * meshData[lowerLeftIndex] +
            (1.0 - remainingXRatio) * remainingYRatio * meshData[upperLeftIndex];

    }
    else if ((lowerXPoint != (numberOfXPoints - 1)) && (lowerYPoint == (numberOfYPoints - 1))) {

        //y max border
        assert(remainingYRatio == 0.0);

        value =
            (1.0 - remainingXRatio) * (1.0 - remainingYRatio) * meshData[lowerLeftIndex] +
            remainingXRatio * (1.0 - remainingYRatio) * meshData[lowerRightIndex];
    }
    else if ((lowerXPoint == (numberOfXPoints - 1)) && (lowerYPoint == (numberOfYPoints - 1))) {

        //x and y max border(point)
        assert(remainingXRatio == 0.0);
        assert(remainingYRatio == 0.0);

        value = meshData[lowerLeftIndex];

    }//if//

    success = true;

}//GetLinearInterpolatedValue//


inline
void MeshData::ReadALineAsConsecutiveData(const string& aLine)
{

    istringstream lineStream(aLine);

    double value;
    for(size_t i = 0; i < numberOfXPoints; i++) {

        lineStream >> value;

        if (lineStream.fail()) {
            cerr << "Error: Mesh Data line: " << aLine << endl;
            exit(1);
        }//if//

        meshData.push_back(value);

    }//for//

    if (!lineStream.eof()) {
        cerr << "Error: Mesh Data line (too many data): " << aLine << endl;
        exit(1);
    }//if//

    if (meshData.size() == (numberOfXPoints * numberOfYPoints)) {
        dataIsCompleted = true;
    }
    else if (meshData.size() > (numberOfXPoints * numberOfYPoints)) {
        cerr << "Error: Mesh Data line (too many data): " << aLine << endl;
        exit(1);
    }

}//ReadALineAsConsecutiveData//


//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------

// Shadow fading model

class ShadowingModel {
public:

    ShadowingModel() { }

    virtual ~ShadowingModel() { }

    virtual void GetShadowingValueDb(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        double& returnShadowingValueDb,
        bool& success) const = 0;

private:

};//FadingModel//



class TwoDimensionalCorrelatedShadowingModel : public ShadowingModel {
public:
    TwoDimensionalCorrelatedShadowingModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        //const RandomNumberGeneratorSeedType& runSeed,
        const string& shadowingMapFileName,
        const double& crossCorrelationFactor,
        const InterfaceOrInstanceIdType& initInstanceId);

    void GetShadowingValueDb(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        double& returnShadowingValueDb,
        bool& success) const;


private:
    const double crossCorrelationAlpha;
    const double crossCorrelationBeta;

    struct SiteInfo {
        SiteIdType siteId;
        double weightedCoefficient;

        SiteInfo(
            const SiteIdType initSiteId,
            const double initWeightedCoefficient)
            :
            siteId(initSiteId),
            weightedCoefficient(initWeightedCoefficient)
        {}
    };//ParameterStock//

    shared_ptr<MeshData> commonShadowingMapPtr;

    map<SiteIdType, shared_ptr<MeshData> > shadowingMap;
    map<NodeIdType, SiteInfo> siteInfoMap;

    void ReadShadowingMap(const string& shadowingMapFileName);

    void AddShadowingMap(
        const SiteIdType siteId,
        const shared_ptr<MeshData>& meshDataPtr);

};//TwoDimensionalCorrelatedShadowingModel//


inline
TwoDimensionalCorrelatedShadowingModel::TwoDimensionalCorrelatedShadowingModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const string& shadowingMapFileName,
    const double& crossCorrelationFactor,
    const InterfaceOrInstanceIdType& initInstanceId)
    :
    crossCorrelationAlpha(sqrt(crossCorrelationFactor)),
    crossCorrelationBeta(sqrt(1.0 - crossCorrelationFactor))
{

    set<NodeIdType> setOfNodeIds;
    theParameterDatabaseReader.MakeSetOfAllCommNodeIds(setOfNodeIds);

    //mapping node id to site id 
    for(set<NodeIdType>::const_iterator nodeIter = setOfNodeIds.begin();
        nodeIter != setOfNodeIds.end(); ++nodeIter) {

        const NodeIdType nodeId = (*nodeIter);

        if (!theParameterDatabaseReader.ParameterExists("shadowing-site-id", nodeId)) continue;

        const string siteIdString = theParameterDatabaseReader.ReadString("shadowing-site-id", nodeId);

        SiteIdType siteId;
        if (siteIdString == "$n") {
            siteId = nodeId;
        }
        else {
            int siteIdInt;
            bool success;

            ConvertStringToInt(siteIdString, siteIdInt, success);

            if (!success) {
                cerr << "Error: invalid shadowing site id: " << siteIdString << endl;
                exit(1);
            }//if//

            siteId = static_cast<SiteIdType>(siteIdInt);

        }//if//

        const double weightedCoefficient =
            theParameterDatabaseReader.ReadDouble("shadowing-weighted-coefficient", nodeId);

        const SiteInfo siteInfo(siteId, weightedCoefficient);

        siteInfoMap.insert(make_pair(nodeId, siteInfo));

    }//for//

    if (shadowingMapFileName == "") {
        cerr << "Error: specify Shadowing Map" << endl;
        exit(1);

        //TBD
        //(*this).GenerateShadowingMap(theParameterDatabaseReader, runSeed);
    }
    else {
        (*this).ReadShadowingMap(shadowingMapFileName);
    }//if//

    //check all data is stored
    for(map<NodeIdType, SiteInfo>::const_iterator iter = siteInfoMap.begin();
        iter != siteInfoMap.end(); ++iter) {

        const SiteInfo& siteInfo = iter->second;

        if (shadowingMap.find(siteInfo.siteId) == shadowingMap.end()) {
            cerr << "Error: Shadowing Map is not found for site id: " << siteInfo.siteId << endl;
            exit(1);
        }//if//

    }//for//

    //common shadowing map (site id = 0)
    map<SiteIdType, shared_ptr<MeshData> >::const_iterator commonIter;
    commonIter = shadowingMap.find(COMMON_SITEID);

    if (commonIter == shadowingMap.end()) {
        cerr << "Error: Shadowing Map is not found for common site id: " << COMMON_SITEID << endl;
        exit(1);
    }//if//

    commonShadowingMapPtr = commonIter->second;
    
}//TwoDimensionalCorrelatedShadowingModel//





inline
void TwoDimensionalCorrelatedShadowingModel::GetShadowingValueDb(
    const ObjectMobilityPosition& txAntennaPosition,
    const ObjectMobilityPosition& rxAntennaPosition,
    double& returnShadowingValueDb,
    bool& success) const
{

    success = false;

    map<NodeIdType, SiteInfo>::const_iterator siteInfoIter;
    double targetPositionX;
    double targetPositionY;

    const NodeIdType txNodeid = txAntennaPosition.GetObjectId();
    siteInfoIter = siteInfoMap.find(txNodeid);

    if (siteInfoIter != siteInfoMap.end()) {

        //tx has site id, then use rx position
        targetPositionX = rxAntennaPosition.X_PositionMeters();
        targetPositionY = rxAntennaPosition.Y_PositionMeters();

    }
    else {

        const NodeIdType rxNodeid = rxAntennaPosition.GetObjectId();
        siteInfoIter = siteInfoMap.find(rxNodeid);

        if (siteInfoIter != siteInfoMap.end()) {
            
            //rx has site id, then use tx position
            targetPositionX = txAntennaPosition.X_PositionMeters();
            targetPositionY = txAntennaPosition.Y_PositionMeters();

        }
        else {

            //no site info
            return;

        }//if//

    }//if//

    const SiteInfo& siteInfo = siteInfoIter->second;

    map<SiteIdType, shared_ptr<MeshData> >::const_iterator shadowingMapIter;
    shadowingMapIter = shadowingMap.find(siteInfo.siteId);
    assert(shadowingMapIter != shadowingMap.end());

    const shared_ptr<MeshData> meshDataPtr = shadowingMapIter->second;

    //this site
    double shadowingValueForThisSite;
    bool successForThisSite;
    meshDataPtr->GetLinearInterpolatedValue(
        targetPositionX,
        targetPositionY,
        shadowingValueForThisSite,
        successForThisSite);

    //common
    double shadowingValueForCommon;
    bool successForCommon;
    commonShadowingMapPtr->GetLinearInterpolatedValue(
        targetPositionX,
        targetPositionY,
        shadowingValueForCommon,
        successForCommon);

    if (successForThisSite && successForCommon) {

        returnShadowingValueDb =
            siteInfo.weightedCoefficient *
            ((crossCorrelationAlpha * shadowingValueForCommon) +
             (crossCorrelationBeta * shadowingValueForThisSite));

        success = true;

    }//if//

}//GetShadowingValueDb//


}//namespace//

#endif
