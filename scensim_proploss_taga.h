// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_PROPLOSS_TAGA_H
#define SCENSIM_PROPLOSS_TAGA_H

#include "scensim_proploss.h"
#include "scensim_gis.h"

namespace ScenSim {

using std::shared_ptr;
using std::cerr;
using std::endl;
using std::map;

class TagaPropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    TagaPropagationLossCalculationModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const double& initCarrierFrequencyMhz,
        const shared_ptr<const RoadLayer>& initRoadLayerPtr,
        const shared_ptr<const BuildingLayer>& initBuildingLayerPtr,
        const double& initMaximumPropagationDistanceMeters,
        const bool initPropagationDelayIsEnabled,
        const int initNumberDataParallelThreads,
        const InterfaceOrInstanceIdType& initInstanceId);
 
    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        const double& xyDistanceSquaredMeters) const;

    void CalculatePropagationPathInformation(
        const PropagationInformationType& informationType,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const ObjectMobilityPosition& rxAntennaPosition,
        const AntennaModel& rxAntennaModel,
        PropagationStatisticsType& propagationStat) const;
     
    bool PropagationLossIsSymmetricValue() const {
        return ((nlosDirection == NLOS_DIRECTION_BIDRECTIONAL_LARGE_LOSS) ||
                (nlosDirection == NLOS_DIRECTION_BIDRECTIONAL_SMALL_LOSS) ||
                (nlosDirection == NLOS_DIRECTION_SMALLNODEID_TO_LARGENODEID) ||
                (nlosDirection == NLOS_DIRECTION_LARGENODEID_TO_SMALLNODEID));
    }

private:
    const double carrierFrequencyGhz;
    const double wavelength;
    const double extremelyHighLossDb;
 
    static const size_t numberLosConsts = 7;
    static const size_t numberNlos1Consts = 4;
    static const size_t numberNlos2Consts = 8;

    enum NlosDirectionType {
        NLOS_DIRECTION_DIRECTOINAL,
        NLOS_DIRECTION_BIDRECTIONAL_LARGE_LOSS,
        NLOS_DIRECTION_BIDRECTIONAL_SMALL_LOSS,
        NLOS_DIRECTION_SMALLNODEID_TO_LARGENODEID,
        NLOS_DIRECTION_LARGENODEID_TO_SMALLNODEID,
    };

    class TagaNlosPathValueCalculator : public NlosPathValueCalculator {
    public:
        TagaNlosPathValueCalculator(TagaPropagationLossCalculationModel* initCalculationModelPtr) : calculationModelPtr(initCalculationModelPtr) {}
        virtual double GetNlosPathValue(const NlosPathData& nlosPath) const { 
            return calculationModelPtr->GetNlosPathValue(nlosPath);
        }
    private:
        TagaPropagationLossCalculationModel* calculationModelPtr;
    };
    
    NlosDirectionType nlosDirection;

    double losConsts[numberLosConsts];
    double nlos1Consts[numberNlos1Consts];
    double nlos2Consts[numberNlos2Consts];

    bool enabledBuildingLosCalculation;

    mutable bool enablePropagationPathTrace;
    mutable vector<PropagationPathType> currentPropgationPathTraces;

    const shared_ptr<const RoadLayer> roadLayerPtr;
    const shared_ptr<const BuildingLayer> buildingLayerPtr;

    shared_ptr<RoadLosChecker> roadLosCheckerPtr;

    double GetNlosPathValue(const NlosPathData& nlosPath) const;

    bool PositionIsInABuilding(
        const Vertex& txPosition) const;

    double CalculateLineOfSightPathlossDb(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const RoadIdType& txRoadId,
        const RoadIdType& rxRoadId) const;

    bool IsAcceptableLosSituation(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const double roadWidthMeters) const;

    double CalculateNonLineOfSightPathlossDb(
           const Vertex& txPosition,
           const Vertex& rxPosition,
           const double distanceMeters,
           const IntersectionIdType& nlosIntersectionId,
           const RoadIdType& txRoadId,
           const RoadIdType& rxRoadId) const;

    bool IsAcceptableNlosSituation(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double txRoadWidth,
        const double rxRoadWidth,
        const double distance1Meters,
        const double distance2Meters) const;

    void TracePropagationPathIfNecessary(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double lossValue) const;
        
    void TracePropagationPathIfNecessary(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double lossValue,
        const vector<Vertex>& diffractionPoints) const;
};


static inline
    void TokenizeString(
        const string& aString,
        const string& deliminator, 
        vector<string>& tokens)
{
    tokens.clear();
    
    size_t posOfReading = 0;
    size_t posOfDelim = aString.find_first_of(deliminator);
    
    while (posOfDelim != string::npos) {

        tokens.push_back(
            aString.substr(posOfReading, posOfDelim - posOfReading));

        posOfReading = posOfDelim + 1;
        posOfDelim = aString.find_first_of(deliminator, posOfReading);
    }
    
    const string lastOneToken = aString.substr(posOfReading);
    if (!lastOneToken.empty()) {
        tokens.push_back(lastOneToken);
    }
}

inline
TagaPropagationLossCalculationModel::TagaPropagationLossCalculationModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const double& initCarrierFrequencyMhz,
    const shared_ptr<const RoadLayer>& initRoadLayerPtr,
    const shared_ptr<const BuildingLayer>& initBuildingLayerPtr,
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
    carrierFrequencyGhz(initCarrierFrequencyMhz / 1000.),
    wavelength((SPEED_OF_LIGHT_METERS_PER_SECOND / 1000000) / initCarrierFrequencyMhz),
    extremelyHighLossDb(EXTREMELY_HIGH_LOSS_DB),
    nlosDirection(NLOS_DIRECTION_DIRECTOINAL),
    enabledBuildingLosCalculation(false),
    enablePropagationPathTrace(false),
    roadLayerPtr(initRoadLayerPtr),
    buildingLayerPtr(initBuildingLayerPtr)
{
    assert(numberLosConsts >= 7);
    losConsts[0] = 10.4;
    losConsts[1] = 1.3;
    losConsts[2] = 24.6;
    losConsts[3] = 1.;
    losConsts[4] = 19.4;
    losConsts[5] = 3.9;
    losConsts[6] = 33.0;
    
    assert(numberNlos1Consts >= 4);
    nlos1Consts[0] = 3.2;
    nlos1Consts[1] = -0.033;
    nlos1Consts[2] = -0.022;
    nlos1Consts[3] = 39.4;
 
    assert(numberNlos2Consts >= 8);
    nlos2Consts[0] = -6.7;
    nlos2Consts[1] = 11.2;
    nlos2Consts[2] = 25.9;
    nlos2Consts[3] = 10.1;
    nlos2Consts[4] = 1.;
    nlos2Consts[5] = 19.8;
    nlos2Consts[6] = -3.8;
    nlos2Consts[7] = 57.7;
 
    const string losConstsCsv = 
        theParameterDatabaseReader.ReadString(
            "proptaga-los-calculation-consts-csv", initInstanceId);
        
    vector<string> losConstsStrings;
    TokenizeString(losConstsCsv, ",", losConstsStrings);
    
    if (losConstsStrings.size() != numberLosConsts) {
        cerr << "Error: LOS calculation CSV is invalid: " << losConstsCsv << endl;
        exit(1);
    }
        
    for(size_t i = 0; i < numberLosConsts; i++) {
        bool success;
            
        ConvertStringToDouble(losConstsStrings[i], losConsts[i], success);
        
        if (!success) {
            cerr << "Error: Invalid LOS calculation consts [" 
                 << losConstsStrings[i] << "] in " << losConstsCsv << endl;
            exit(1);
        }
    }
    
    const string nlos1ConstsCsv = 
        theParameterDatabaseReader.ReadString(
            "proptaga-nlos1-calculation-consts-csv", initInstanceId);
    
    vector<string> nlos1ConstsStrings;
    TokenizeString(nlos1ConstsCsv, ",", nlos1ConstsStrings);
    
    if (nlos1ConstsStrings.size() != numberNlos1Consts) {
        cerr << "Error: NLOS1 calculation CSV is invalid: " << nlos1ConstsCsv << endl;
        exit(1);
    }
    
    for(size_t i = 0; i < numberNlos1Consts; i++) {
        bool success;

        ConvertStringToDouble(nlos1ConstsStrings[i], nlos1Consts[i], success);
      
        if (!success) {
            cerr << "Error: Invalid NLOS1 calculation consts [" 
                 << nlos1ConstsStrings[i] << "] in " << nlos1ConstsCsv << endl;
            exit(1);
        }
    }

    const string nlos2ConstsCsv = 
        theParameterDatabaseReader.ReadString(
            "proptaga-nlos2-calculation-consts-csv", initInstanceId);
    
    vector<string> nlos2ConstsStrings;
    TokenizeString(nlos2ConstsCsv, ",", nlos2ConstsStrings);
    
    if (nlos2ConstsStrings.size() != numberNlos2Consts) {
        cerr << "Error: NLOS2 calculation CSV is invalid: " << nlos2ConstsCsv << endl;
        exit(1);
    }
    
    for(size_t i = 0; i < numberNlos2Consts; i++) {
        bool success;

        ConvertStringToDouble(nlos2ConstsStrings[i], nlos2Consts[i], success);
      
        if (!success) {
            cerr << "Error: Invalid NLOS2 calculation consts [" 
                 << nlos2ConstsStrings[i] << "] in " << nlos2ConstsCsv << endl;
            exit(1);
        }
    }

    if (theParameterDatabaseReader.ParameterExists(
            "proptaga-nlos-loss-direction", initInstanceId)) {
            
        string policyString = theParameterDatabaseReader.ReadString(
            "proptaga-nlos-loss-direction", initInstanceId);
        
        ConvertStringToLowerCase(policyString);
        
        if (policyString == "directional") {
            nlosDirection = NLOS_DIRECTION_DIRECTOINAL;
        } else if (policyString == "bidirectionallargeloss") {
            nlosDirection = NLOS_DIRECTION_BIDRECTIONAL_LARGE_LOSS;
        } else if (policyString == "bidirectionalsmallloss") {
            nlosDirection = NLOS_DIRECTION_BIDRECTIONAL_SMALL_LOSS;
        } else if (policyString == "smallnodeidtolargenodeidloss") {
            nlosDirection = NLOS_DIRECTION_SMALLNODEID_TO_LARGENODEID;
        } else if (policyString == "largenodeidtosmallnodeidloss") {
            nlosDirection = NLOS_DIRECTION_LARGENODEID_TO_SMALLNODEID;
        }
    }

    const int maxDiffractionCount = 1;
    double losThresholdRadians = 1. * RADIANS_PER_DEGREE;
    double maxNlosDistance = DBL_MAX;
 
    if (theParameterDatabaseReader.ParameterExists("proptaga-los-angle-degrees-between-roads", initInstanceId)) {
        losThresholdRadians = theParameterDatabaseReader.ReadDouble("proptaga-los-angle-degrees-between-roads", initInstanceId) * RADIANS_PER_DEGREE;
    }

    if (theParameterDatabaseReader.ParameterExists("proptaga-nlos-max-distance-meters", initInstanceId)) {
        maxNlosDistance = theParameterDatabaseReader.ReadDouble("proptaga-nlos-max-distance-meters", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists("proptaga-enable-building-based-los-calculation", initInstanceId)) {
        enabledBuildingLosCalculation =
            theParameterDatabaseReader.ReadBool("proptaga-enable-building-based-los-calculation", initInstanceId);
    }

    roadLosCheckerPtr.reset(
        new RoadLosChecker(
            roadLayerPtr,
            shared_ptr<NlosPathValueCalculator>(new TagaNlosPathValueCalculator(this)),
            maxDiffractionCount,
            losThresholdRadians,
            maxNlosDistance));

    roadLosCheckerPtr->MakeLosRelation();
}

inline
double TagaPropagationLossCalculationModel::CalculatePropagationLossDb(
    const ObjectMobilityPosition& txAntennaPosition,
    const ObjectMobilityPosition& rxAntennaPosition,
    const double& xyDistanceSquaredMeters) const
{
    Vertex txPosition(
        txAntennaPosition.X_PositionMeters(),
        txAntennaPosition.Y_PositionMeters(),
        txAntennaPosition.HeightFromGroundMeters());

    Vertex rxPosition(
        rxAntennaPosition.X_PositionMeters(),
        rxAntennaPosition.Y_PositionMeters(),
        rxAntennaPosition.HeightFromGroundMeters());
 
    if (nlosDirection == NLOS_DIRECTION_SMALLNODEID_TO_LARGENODEID) {
        if (txAntennaPosition.GetObjectId() > rxAntennaPosition.GetObjectId()) {
            std::swap(txPosition, rxPosition);
        }
    } else if (nlosDirection == NLOS_DIRECTION_LARGENODEID_TO_SMALLNODEID) {
        if (txAntennaPosition.GetObjectId() < rxAntennaPosition.GetObjectId()) {
            std::swap(txPosition, rxPosition);
        }
    }

    shared_ptr<RoadLayer> txRxPlacedRoadLayerPtr;
    vector<RoadIdType> txRoadIds;
    vector<RoadIdType> rxRoadIds;

    roadLayerPtr->GetRoadIdsAt(txPosition, txRoadIds);
    roadLayerPtr->GetRoadIdsAt(rxPosition, rxRoadIds);

    if (txRoadIds.empty() || rxRoadIds.empty()) {
        return extremelyHighLossDb;
    }
    
    if ((*this).PositionIsInABuilding(txPosition) ||
        (*this).PositionIsInABuilding(rxPosition)) {
        return extremelyHighLossDb;
    }

    const double distanceMeters = XYDistanceBetweenVertices(txPosition, rxPosition);

    if (distanceMeters <= 0.) {
        return 0.;
    }

    if ((enabledBuildingLosCalculation && buildingLayerPtr->PositionsAreLineOfSight(txPosition, rxPosition)) ||
        roadLosCheckerPtr->PositionsAreLineOfSight(txPosition, rxPosition)) {
        double maxLossValueDb = 0;
        
        for (size_t i = 0; i < txRoadIds.size(); i++) {
            const RoadIdType& txRoadId = txRoadIds[i];
            
            for (size_t j = 0; j < rxRoadIds.size(); j++) {
                const RoadIdType& rxRoadId = rxRoadIds[j];
                const double lossValueDb = (*this).CalculateLineOfSightPathlossDb(
                    txPosition,
                    rxPosition,
                    distanceMeters,
                    txRoadId,
                    rxRoadId);
                
                (*this).TracePropagationPathIfNecessary(txPosition, rxPosition, lossValueDb);

                maxLossValueDb = std::max(maxLossValueDb, lossValueDb);
            }
        }

        return maxLossValueDb;

    } else {
        double minLossValueDb = extremelyHighLossDb;

        for (size_t i = 0; i < txRoadIds.size(); i++) {
            const RoadIdType& txRoadId = txRoadIds[i];
            
            for (size_t j = 0; j < rxRoadIds.size(); j++) {
                const RoadIdType& rxRoadId = rxRoadIds[j];
            
                const RoadLosRelationData& losRelation = 
                    roadLosCheckerPtr->GetLosRelation(txRoadId, rxRoadId);
                
                if (losRelation.relationType != ROAD_LOSRELATION_NLOS) {
                    continue;
                }

                const map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>& nlosPaths = losRelation.nlosPaths;
                
                typedef map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>::const_iterator NlosIter;
                
                for(NlosIter nlosIter = nlosPaths.begin(); nlosIter != nlosPaths.end(); nlosIter++) {
                    const NlosPathData& nlosPath = (*nlosIter).second;
                                        
                    double lossValueDb = 
                        (*this).CalculateNonLineOfSightPathlossDb(
                            txPosition,
                            rxPosition,
                            distanceMeters,
                            nlosPath.GetIntersectionId(txRoadId, 1),
                            txRoadId,
                            rxRoadId);
                    
                    if (nlosDirection == NLOS_DIRECTION_BIDRECTIONAL_LARGE_LOSS ||
                        nlosDirection == NLOS_DIRECTION_BIDRECTIONAL_SMALL_LOSS) {

                        lossValueDb = std::max(
                            lossValueDb,
                            (*this).CalculateNonLineOfSightPathlossDb(
                                rxPosition,
                                txPosition,
                                distanceMeters,
                                nlosPath.GetIntersectionId(rxRoadId, 1),
                                rxRoadId,
                                txRoadId));
                    }

                    (*this).TracePropagationPathIfNecessary(
                        txPosition, rxPosition, lossValueDb,
                        roadLosCheckerPtr->CalculateNlosPoints(nlosPath, txRoadId));

                    minLossValueDb = std::min(minLossValueDb, lossValueDb);
                }

                if (nlosDirection == NLOS_DIRECTION_BIDRECTIONAL_LARGE_LOSS ||
                    nlosDirection == NLOS_DIRECTION_BIDRECTIONAL_SMALL_LOSS) {
        
                    const RoadLosRelationData& inverseLosRelationData =
                        roadLosCheckerPtr->GetLosRelation(rxRoadId, txRoadId);
                    
                    const map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>& inverseNlosPaths = inverseLosRelationData.nlosPaths;
                    
                    if (inverseLosRelationData.relationType != ROAD_LOSRELATION_NLOS) {
                        continue;
                    }

                    double inverseMinLossDb = extremelyHighLossDb;
                    
                    for(NlosIter nlosIter = inverseNlosPaths.begin(); nlosIter != inverseNlosPaths.end(); nlosIter++) {
                        const NlosPathData& nlosPath = (*nlosIter).second;
                        
                        const double lossValueDb = std::max(
                            (*this).CalculateNonLineOfSightPathlossDb(
                                rxPosition,
                                txPosition,
                                distanceMeters,
                                nlosPath.GetIntersectionId(rxRoadId, 1),
                                rxRoadId,
                                txRoadId),
                            (*this).CalculateNonLineOfSightPathlossDb(
                                txPosition,
                                rxPosition,
                                distanceMeters,
                                nlosPath.GetIntersectionId(txRoadId, 1),
                                txRoadId,
                                rxRoadId));
                    
                        (*this).TracePropagationPathIfNecessary(
                            txPosition, rxPosition, lossValueDb,
                            roadLosCheckerPtr->CalculateNlosPoints(nlosPath, txRoadId));
                    
                        inverseMinLossDb = std::min(inverseMinLossDb, lossValueDb);
                    }

                    if (nlosDirection == NLOS_DIRECTION_BIDRECTIONAL_LARGE_LOSS) {
                        minLossValueDb = std::max(minLossValueDb, inverseMinLossDb);
                    } else {
                        minLossValueDb = std::min(minLossValueDb, inverseMinLossDb);
                    }
                }
            }
        }

        return minLossValueDb;
    }

}//CalculatePropagationLossDb;

inline
bool TagaPropagationLossCalculationModel::PositionIsInABuilding(
    const Vertex& txPosition) const
{ 
    bool found;
    BuildingIdType buildingId;
    
    buildingLayerPtr->GetBuildingIdAt(txPosition, found, buildingId);

    if (found) {
        return true;
    }

    return false;
}

inline
void TagaPropagationLossCalculationModel::CalculatePropagationPathInformation(
    const PropagationInformationType& informationType,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntennaModel,
    const ObjectMobilityPosition& rxAntennaPosition,
    const AntennaModel& rxAntennaModel,
    PropagationStatisticsType& propagationStatistics) const
{
    enablePropagationPathTrace = true;
    currentPropgationPathTraces.clear();

    const double dx = txAntennaPosition.X_PositionMeters() - rxAntennaPosition.X_PositionMeters();
    const double dy = txAntennaPosition.Y_PositionMeters() - rxAntennaPosition.Y_PositionMeters();

    const double lossValueDb =
        (*this).CalculatePropagationLossDb(
            txAntennaPosition,
            rxAntennaPosition,
            dx*dx + dy*dy);
    
    propagationStatistics.totalLossValueDb = lossValueDb - CalculateTotalAntennaGainDbi(
        txAntennaPosition,
        txAntennaModel,
        rxAntennaPosition,
        rxAntennaModel);
    propagationStatistics.paths = currentPropgationPathTraces;

    enablePropagationPathTrace = false;
}

inline
double TagaPropagationLossCalculationModel::CalculateLineOfSightPathlossDb(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const RoadIdType& txRoadId,
    const RoadIdType& rxRoadId) const
{
    const double minWidthMeters = std::min(
        roadLayerPtr->GetRoad(txRoadId).GetWidthMeters(),
        roadLayerPtr->GetRoad(rxRoadId).GetWidthMeters());
       
    assert(distanceMeters > 0);

    if (!(*this).IsAcceptableLosSituation(
            txPosition,
            rxPosition,
            distanceMeters, 
            minWidthMeters)) {        
        return extremelyHighLossDb;
    }

    const double txHeightByRxHeightDivWavelength =
        (txPosition.positionZ*rxPosition.positionZ) / wavelength;

    const double breakpointDistanceMeters =
        8 * txHeightByRxHeightDivWavelength;
 
    assert(breakpointDistanceMeters != 0);
    
    return ((losConsts[0] + losConsts[1]*log10(txHeightByRxHeightDivWavelength)) * log10(distanceMeters) +
            losConsts[2]*log10(losConsts[3]+distanceMeters/breakpointDistanceMeters) +
            losConsts[4]*log10(carrierFrequencyGhz) +
            losConsts[5]*log10(minWidthMeters) + losConsts[6]);
}

inline
bool TagaPropagationLossCalculationModel::IsAcceptableLosSituation(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const double roadWidthMeters) const
{        
    if (!(/*2 <= distanceMeters && */ distanceMeters <= 1000)) {
        //TBD// cerr << "Warning (Propagation for LOS):" << endl
        //TBD//      << "tx and rx distace meters must be 2 to 1000 [m]" << endl;
        return false;
    }

    //TBD// if (!(0.4 <= carrierFrequencyGhz && carrierFrequencyGhz <= 6)) {
    //TBD//     cerr << "Warning (Propagation for LOS):" << endl
    //TBD//          << "frequency must be 0.4 to 6 [GHz]" << endl;
    //TBD//     return false;
    //TBD// }

    //TBD// if (!(8 <= roadWidthMeters && roadWidthMeters <= 60)) {
    //TBD//     cerr << "Warning (Propagation for LOS):" << endl
    //TBD//          << "road width must be 8 to 60 [m]" << endl;
    //TBD//     return false;
    //TBD// }
    
    //TBD// if (!((0.5 <= txPosition.positionZ && txPosition.positionZ <= 3.5) &&
    //TBD//       (0.5 <= rxPosition.positionZ && rxPosition.positionZ <= 3.5))) {
    //TBD//     cerr << "Warning (Propagation for LOS):" << endl
    //TBD//          << "tx and rx height must be 0.5 to 3.5 [m]" << endl;
    //TBD//     return false;
    //TBD// }

    return true;
}

inline
double TagaPropagationLossCalculationModel::CalculateNonLineOfSightPathlossDb(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const IntersectionIdType& nlosIntersectionId,
    const RoadIdType& txRoadId,
    const RoadIdType& rxRoadId) const
{
    const Vertex& intersectionPosition =
        roadLayerPtr->GetIntersection(nlosIntersectionId).GetVertex();
        
    const Road& txRoad = roadLayerPtr->GetRoad(txRoadId);
    const Road& rxRoad = roadLayerPtr->GetRoad(rxRoadId);

    // Note: Diff rx from tx road width. And diff tx from rx road width. 
    const double distance1Meters = 
        XYDistanceBetweenVertices(txPosition, intersectionPosition) - rxRoad.GetWidthMeters() / 2.;
    const double distance2Meters =
        XYDistanceBetweenVertices(rxPosition, intersectionPosition) - txRoad.GetWidthMeters() / 2.;
    
    if (!(*this).IsAcceptableNlosSituation(
            txPosition,
            rxPosition,
            txRoad.GetWidthMeters(),
            rxRoad.GetWidthMeters(),
            distance1Meters,
            distance2Meters)) {
        return extremelyHighLossDb;
    }

    const Vertex& txIntersection1Pos = 
        roadLayerPtr->GetIntersection(txRoad.GetStartIntersectionId()).GetVertex();
    const Vertex& txIntersection2Pos = 
        roadLayerPtr->GetIntersection(txRoad.GetEndIntersectionId()).GetVertex();
    const Vertex txNormalPos =
        CalculateIntersectionPosition(txPosition, txIntersection1Pos, txIntersection2Pos);

    double dw1Meters;
    if (HorizontalLinesAreIntersection(
            txPosition, rxPosition, txIntersection1Pos, txIntersection2Pos)) {        
        dw1Meters = txRoad.GetWidthMeters() / 2. + sqrt(SquaredXYDistanceBetweenVertices(txPosition, txNormalPos));
    } else {
        dw1Meters = txRoad.GetWidthMeters() / 2. - sqrt(SquaredXYDistanceBetweenVertices(txPosition, txNormalPos));
    }

    const Vertex& rxIntersection1Pos = 
        roadLayerPtr->GetIntersection(rxRoad.GetStartIntersectionId()).GetVertex();
    const Vertex& rxIntersection2Pos = 
        roadLayerPtr->GetIntersection(rxRoad.GetEndIntersectionId()).GetVertex();

    const Vertex rxNormalPos =
        CalculateIntersectionPosition(rxPosition, rxIntersection1Pos, rxIntersection2Pos);

    double dw2Meters;
    if (HorizontalLinesAreIntersection(
            txPosition, rxPosition, rxIntersection1Pos, rxIntersection2Pos)) {        
        dw2Meters = rxRoad.GetWidthMeters() / 2. + sqrt(SquaredXYDistanceBetweenVertices(rxPosition, rxNormalPos));
    } else {
        dw2Meters = rxRoad.GetWidthMeters() / 2. - sqrt(SquaredXYDistanceBetweenVertices(rxPosition, rxNormalPos));
    }

    const double totalDistanceMeters =
        distance1Meters + dw1Meters + distance2Meters + dw2Meters;

    assert(totalDistanceMeters != 0);
    assert(distance1Meters != 0);

    const double firstTermDistanceMeters = distance1Meters + dw2Meters;
    const double secondTermDistanceMeters = dw1Meters + (dw1Meters*dw2Meters)/distance1Meters;

    const double straightDistanceMetersToLosBound = sqrt(
        firstTermDistanceMeters * firstTermDistanceMeters +
        secondTermDistanceMeters * secondTermDistanceMeters);
    
    if (totalDistanceMeters <= straightDistanceMetersToLosBound) {

        return (*this).CalculateLineOfSightPathlossDb(
            txPosition, rxPosition, distanceMeters,
            txRoadId, rxRoadId);

    } else {

        const double nlos1LossDb =
            ((nlos1Consts[0] + nlos1Consts[1]*txRoad.GetWidthMeters() + nlos1Consts[2]*rxRoad.GetWidthMeters())*distance1Meters + nlos1Consts[3]) *
            (log10(totalDistanceMeters) - log10(straightDistanceMetersToLosBound)) +
            (*this).CalculateLineOfSightPathlossDb(
                txPosition, rxPosition, straightDistanceMetersToLosBound,
                txRoadId, rxRoadId);

        const double txHeightByRxHeightDivWavelength =
            (txPosition.positionZ*rxPosition.positionZ) / wavelength;
        
        const double breakpointDistancMeters = 
            4 * txHeightByRxHeightDivWavelength;
        
        const double nlos2LossDb =
            ((nlos2Consts[0] + nlos2Consts[1]*log10(txHeightByRxHeightDivWavelength))*log10(totalDistanceMeters) +
             (nlos2Consts[2] + nlos2Consts[3]*log10(distance1Meters/wavelength))*log10(nlos2Consts[4] + totalDistanceMeters/breakpointDistancMeters) +
             nlos2Consts[5]*log10(carrierFrequencyGhz) +
             nlos2Consts[6]*log10(txRoad.GetWidthMeters()*rxRoad.GetWidthMeters()) + nlos2Consts[7]);
 
        return std::min(nlos1LossDb, nlos2LossDb);
    }
}

inline
bool TagaPropagationLossCalculationModel::IsAcceptableNlosSituation(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double txRoadWidth,
    const double rxRoadWidth,
    const double distance1Meters,
    const double distance2Meters) const
{
    //TBD// if (!(0.4 <= carrierFrequencyGhz && carrierFrequencyGhz <= 6)) {
    //TBD//     cerr << "Warning (Propagation for NLOS):" << endl
    //TBD//          << "frequency must be 0.4 to 6 [GHz]" << endl;
    //TBD//     return false;
    //TBD// }

    if (!(/*10 <= distance1Meters && */distance1Meters <= 300)) {
        //TBD// cerr << "Warning (Propagation for NLOS):" << endl
        //TBD//      << "distace1 meters must be 10 to 300 [m]" << endl;
        return false;
    }
    if (!(/*0 <= distance2Meters && */distance2Meters <= 300)) {
        //TBD// cerr << "Warning (Propagation for NLOS):" << endl
        //TBD//      << "distace2 meters must be 0 to 300 [m]" << endl;
        return false;
    }
    
    //TBD// if (!((0.5 <= txPosition.positionZ && txPosition.positionZ <= 3.5) &&
    //TBD//       (0.5 <= rxPosition.positionZ && rxPosition.positionZ <= 3.5))) {
    //TBD//     cerr << "Warning (Propagation for NLOS):" << endl
    //TBD//          << "tx and rx height must be 0.5 to 3.5 [m]" << endl;
    //TBD//     return false;
    //TBD// }
    
    //TBD// if (!((5 <= txRoadWidth && txRoadWidth <= 60) &&
    //TBD//       (5 <= rxRoadWidth && rxRoadWidth <= 60))) {
    //TBD//     cerr << "Warning (Propagation for NLOS):" << endl
    //TBD//          << "tx and rx road width must be 5 to 60 [m]" << endl;
    //TBD//     return false;
    //TBD// }

    return true;
}

inline
void TagaPropagationLossCalculationModel::TracePropagationPathIfNecessary(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double lossValue) const
{
    if (!enablePropagationPathTrace) {
        return;
    }

    const PropagationVertexType txVertex(
        txPosition.positionX,
        txPosition.positionY,
        txPosition.positionZ,
        "Tx");

    const PropagationVertexType rxVertex(
        rxPosition.positionX,
        rxPosition.positionY,
        rxPosition.positionZ,
        "Rx");
    
    currentPropgationPathTraces.push_back(PropagationPathType());
    PropagationPathType& aPropagationPath = currentPropgationPathTraces.back();

    aPropagationPath.lossValueDb = lossValue;
    aPropagationPath.pathVertices.push_back(txVertex);
    aPropagationPath.pathVertices.push_back(rxVertex);
}

inline
void TagaPropagationLossCalculationModel::TracePropagationPathIfNecessary(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double lossValue,
    const vector<Vertex>& diffractionPoints) const
{
    if (!enablePropagationPathTrace) {
        return;
    }

    currentPropgationPathTraces.push_back(PropagationPathType());
    PropagationPathType& aPropagationPath = currentPropgationPathTraces.back();

    aPropagationPath.lossValueDb = lossValue;
    
    const PropagationVertexType txVertex(
        txPosition.positionX,
        txPosition.positionY,
        txPosition.positionZ,
        "Tx");

    aPropagationPath.pathVertices.push_back(txVertex);

    for(size_t i = 0; i < diffractionPoints.size(); i++) {
        const Vertex& diffractionPoint = diffractionPoints[i];
        
        const PropagationVertexType diffractionVertex(
            diffractionPoint.positionX,
            diffractionPoint.positionY,
            diffractionPoint.positionZ,
            "D");
        
        aPropagationPath.pathVertices.push_back(diffractionVertex);
    }
    
    const PropagationVertexType rxVertex(
        rxPosition.positionX,
        rxPosition.positionY,
        rxPosition.positionZ,
        "Rx");
    
    aPropagationPath.pathVertices.push_back(rxVertex);
}

inline
double TagaPropagationLossCalculationModel::GetNlosPathValue(const NlosPathData& nlosPath) const
{
    const RoadIdType txRoadId = nlosPath.GetFrontRoadId();
    const RoadIdType rxRoadId = nlosPath.GetBackRoadId();

    const IntersectionIdType txIntersectionId = nlosPath.GetFrontIntersectionId();
    const IntersectionIdType rxIntersectionId = nlosPath.GetBackIntersectionId();

    const double w1 = roadLayerPtr->GetRoad(nlosPath.GetNlosRoadId()).GetRoadWidthMeters() + 0.01;
    const double w2 = roadLayerPtr->GetRoad(nlosPath.GetLastNlosRoadId()).GetRoadWidthMeters() + 0.01;

    Vertex txPosition =
        roadLayerPtr->GetRoad(txRoadId).GetInternalPoint(txIntersectionId, w1*0.5);

    Vertex rxPosition =
        roadLayerPtr->GetRoad(rxRoadId).GetInternalPoint(rxIntersectionId, w2*0.5);

    const double antennaHeightMeters = 1.5;

    txPosition.z = antennaHeightMeters;
    rxPosition.z = antennaHeightMeters;
    
    return(*this).CalculateNonLineOfSightPathlossDb(
        txPosition,
        rxPosition,
        txPosition.DistanceTo(rxPosition),
        nlosPath.GetIntersectionId(txRoadId, 1),
        txRoadId,
        rxRoadId);
}

} //namespace ScenSim//

#endif
