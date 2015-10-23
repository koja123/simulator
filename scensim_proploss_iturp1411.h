// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_PROPLOSS_ITURP1411N_H
#define SCENSIM_PROPLOSS_ITURP1411N_H

#include "scensim_proploss.h"
#include "scensim_gis.h"

namespace ScenSim {

using std::shared_ptr;
using std::cerr;
using std::endl;
using std::map;

//=====================================================================

// See ITU-R P.1411
//
// implemented
// - 4.1: LoS situations within street canyons
// - 4.2: Models for NLos situations
// - 4.2.1: Propagation over roof-tops for urban area
// - 4.2.2: Propagation over roof-tops for suburban area
// - 4.2.3: Propagation within street canyons for frequency range from 800 to 2000 MHz
// - 4.2.4: Propagation within street canyons for frequency range from 2 to 16 GHz
// - 4.3: Propagation between terminals located below roof-top height at UHF

class P1411PropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    P1411PropagationLossCalculationModel(
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
        return ((nlos2Direction == NLOS2_DIRECTION_BIDRECTIONAL_LARGE_LOSS) ||
                (nlos2Direction == NLOS2_DIRECTION_BIDRECTIONAL_SMALL_LOSS) ||
                (nlos2Direction == NLOS2_DIRECTION_SMALLNODEID_TO_LARGENODEID) ||
                (nlos2Direction == NLOS2_DIRECTION_LARGENODEID_TO_SMALLNODEID));
    }

private:
    const double wavelength;
    const double carrierFrequencyMhzLog10;
    const double extremelyHighLossDb;
    bool isShf;

    enum LosCalculationPolicyType {
        LOS_CALCULATION_POLICY_LOWERBOUND,
        LOS_CALCULATION_POLICY_UPPERBOUND,
        LOS_CALCULATION_POLICY_GEOMETRICMEAN,
        LOS_CALCULATION_POLICY_MEDIAN,
        LOS_CALCULATION_POLICY_FREESPACE,
    };

    LosCalculationPolicyType losCalculationPolicy;
    double shfShortDistanceMeters;
    double effectiveRoadHeightMeters;

    enum Nlos1CalculationPolicyType {
        NLOS1_CALCULATION_POLICY_URBAN,
        NLOS1_CALCULATION_POLICY_SUBURBAN,
    };

    enum Nlos2UsePolicyType {
        NLOS2_USE_POLICY_DEFAULT,
        NLOS2_ALWAYS_USE_800TO2000MHZ,
        NLOS2_ALWAYS_USE_2TO16GHZ,
    };

    enum Nlos2ExtensionType {
        NLOS2_EXTENSION_NONE,
        NLOS2_EXTENSION_USE_DISTANCE_TO_LAST_NLOS_FOR_CORRECTION_X1,
    };

    enum Nlos2DirectionType {
        NLOS2_DIRECTION_DIRECTOINAL,
        NLOS2_DIRECTION_BIDRECTIONAL_LARGE_LOSS,
        NLOS2_DIRECTION_BIDRECTIONAL_SMALL_LOSS,
        NLOS2_DIRECTION_SMALLNODEID_TO_LARGENODEID,
        NLOS2_DIRECTION_LARGENODEID_TO_SMALLNODEID,
    };

    enum StreetCanyon800To2000NlosCalculationPolicyType {
        NLOS800TO2000_CALCULATION_POLICY_LOWER,
        NLOS800TO2000_CALCULATION_POLICY_UPPER,
        NLOS800TO2000_CALCULATION_POLICY_GEOMETRICMEAN,
    };

    class P1411NlosPathValueCalculator : public NlosPathValueCalculator {
    public:
        P1411NlosPathValueCalculator(P1411PropagationLossCalculationModel* initCalculationModelPtr) : calculationModelPtr(initCalculationModelPtr) {}
        virtual double GetNlosPathValue(const NlosPathData& nlosPath) const {
            return calculationModelPtr->GetNlosPathValue(nlosPath);
        }
    private:
        P1411PropagationLossCalculationModel* calculationModelPtr;
    };

    Nlos1CalculationPolicyType nlos1CalculationPolicy;
    Nlos2UsePolicyType nlos2UsePolicy;
    Nlos2ExtensionType nlos2Extension;
    Nlos2DirectionType nlos2Direction;

    bool useLargerLossAtNlosBound;
    bool enabledBuildingLosCalculation;

    StreetCanyon800To2000NlosCalculationPolicyType nlos800To2000CalculationPolicy;

    double Lcorner;
    double heightDifferThresholdMeters;
    bool allBuildingRoofTopsAreAboutTheSameHeight;
    bool enabledPropagationBetweenTerminalsLocatedBelowRoofTop;

    double currentAverageBuildingHeightMeters;
    double wellBelowRoofTopHeightMeters;

    double belowRoofTopCalculationLosDistanceMeters;
    double transitionRegionMeters;
    double belowRoofTopCalculationLosDeltaLossDb;
    double belowRoofTopCalculationNlosDeltaLossDb;
    double Lurban;

    mutable bool enablePropagationPathTrace;
    mutable vector<PropagationPathType> currentPropgationPathTraces;

    const shared_ptr<const RoadLayer> roadLayerPtr;
    const shared_ptr<const BuildingLayer> buildingLayerPtr;

    shared_ptr<RoadLosChecker> roadLosCheckerPtr;

    double GetNlosPathValue(const NlosPathData& nlosPath) const;

    double CalculateLineOfSightPathlossDb(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const bool isPureLosCalculation = true,
        const bool forceMeanLossCalculation = false) const;

    double CalculateNonLineOfSightPathlossDb(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const RoadIdType& rxRoadId) const;

    struct BuildingInformation {
        double heightMeters;
        double distanceMeters;

        BuildingInformation(
            const double initHeightMeters,
            const double initDistanceMeters)
          : heightMeters(initHeightMeters),
            distanceMeters(initDistanceMeters)
        {}
    };

    void CalculateLinearRoofCollisions(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        double& maxRoofHeightMeters,
        double& averageRoofHeightMeters,
        double& roofHeightDifferMeters,
        double& averageBuildingSeparationMeters,
        double& lengthMetersOfThePathCoveredByBuildings) const;

    double CalculateNonLineOfSight1UrbanPathlossDb(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const double rxRoadWidthMeters,
        const double averageRoofHeightMeters,
        const double averageBuildingSeparationMeters,
        const double roofHeightDifferMeters,
        const double lengthMetersOfThePathCoveredByBuildings,
        const double rayToRxStreetRadians) const;

    bool IsAcceptableNlos1UrbanSituation(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const double rxRoadWidthMeters,
        const double averageRoofHeightMeters) const;

    double CalculateRoofTopsNonLineOfSight1UrbanPathlossDb(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const double rxRoadWidthMeters,
        const double roofHeightMeters,
        const double averageBuildingSeparationMeters,
        const double lengthMetersOfThePathCoveredByBuildings,
        const double rayToRxStreetRadians) const;

    double CalculateNonLineOfSight1SuburbanPathlossDb(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const double rxRoadWidthMeters,
        const double roofHeightMeters,
        const double rayToRxStreetRadians) const;

    bool IsAcceptableNlos1SuburbanSituation(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const double rxRoadWidthMeters,
        const double averageRoofHeightMeters) const;

    double CalculateRoofTopsNonLineOfSight1SuburbanPathlossDb(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const double rxRoadWidthMeters,
        const double roofHeightMeters,
        const double rayToStreetRadians) const;

    double CalculateMinNonLineOfSight2Frequency800To2000MHzPathlossDb(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const RoadIdType& txRoadId,
        const RoadIdType& rxRoadId) const;

    double CalculateNonLineOfSight2Frequency800To2000MHzPathlossDb(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const RoadIdType& txRoadId,
        const RoadIdType& rxRoadId,
        const NlosPathData& nlosPath) const;

    double CalculateMinNonLineOfSight2Frequency2To16GHzPathlossDb(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const RoadIdType& txRoadId,
        const RoadIdType& rxRoadId) const;

    double CalculateNonLineOfSight2Frequency2To16GHzPathlossDb(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double distanceMeters,
        const RoadIdType& txRoadId,
        const RoadIdType& rxRoadId,
        const NlosPathData& nlosPath) const;

    bool TxAndRxAreWellBelowRoofTopAtUhf(
        const Vertex& txPosition,
        const Vertex& rxPosition) const;

    double CalculateBelowRoofTopHeightUhfPathlossDb(const double distanceMeters) const;

    double CalculateBelowRoofTopHeightLosPathlossDb(const double distanceMeters) const;

    double CalculateBelowRoofTopHeightNlosPathlossDb(const double distanceMeters) const;

    double CalculateFreeSpaceLossDb(const double distanceMeters) const;


    static double Calculate800To20000LossDb(
        const double reflectionLossDb,
        const double difractionLossDb);

    static double CalculateL1msd(
        const double Lbsh,
        const double Ka,
        const double Kd,
        const double d,
        const double Kf,
        const double carrierFrequencyMhzLog10,
        const double averageBuildingSeparationMeters);

    static double CalculateL2msd(
        const double txHeightMeters,
        const double roofHeightMeters,
        const double deltaHupp,
        const double deltaTxHeightMeters,
        const double d,
        const double averageBuildingSeparationMeters,
        const double wavelength,
        const double deltaHlow,
        const double rho,
        const double sita);

    static void CalculateLdn(
        const size_t k,
        const double txRxHeightDifferenceMeters,
        const double rxRoadWidthMeters,
        const double deltaRxHeightMeters,
        const double phi,
        const double wavelength,
        double& Dk,
        double& Ldk);

    static double CalculateDrd(
        const double carrierFrequencyGhz,
        const double d1,
        const double d2,
        const double d3);

    static double CalculateLdRd(
        const vector<double>& ds,
        const vector<double>& Lds,
        const double dRd,
        const size_t k);

    bool PositionIsInABuilding(
        const Vertex& txPosition) const;

    void TracePropagationPathIfNecessary(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double lossValue) const;

    void TracePropagationPathIfNecessary(
        const Vertex& txPosition,
        const Vertex& rxPosition,
        const double lossValue,
        const vector<Vertex>& diffractionPoints) const;

};//P1411PropagationLossCalculationModel//

inline
P1411PropagationLossCalculationModel::P1411PropagationLossCalculationModel(
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
    wavelength((SPEED_OF_LIGHT_METERS_PER_SECOND / 1000000) / initCarrierFrequencyMhz),
    carrierFrequencyMhzLog10(log10(initCarrierFrequencyMhz)),
    extremelyHighLossDb(EXTREMELY_HIGH_LOSS_DB),
    isShf(3000 <= initCarrierFrequencyMhz && initCarrierFrequencyMhz < 30000),
    losCalculationPolicy(LOS_CALCULATION_POLICY_MEDIAN),
    shfShortDistanceMeters(20),
    effectiveRoadHeightMeters(0),
    nlos1CalculationPolicy(NLOS1_CALCULATION_POLICY_URBAN),
    nlos2UsePolicy(NLOS2_USE_POLICY_DEFAULT),
    nlos2Extension(NLOS2_EXTENSION_NONE),
    nlos2Direction(NLOS2_DIRECTION_DIRECTOINAL),
    useLargerLossAtNlosBound(false),
    enabledBuildingLosCalculation(false),
    nlos800To2000CalculationPolicy(NLOS800TO2000_CALCULATION_POLICY_UPPER),
    Lcorner(20),
    heightDifferThresholdMeters(1),
    allBuildingRoofTopsAreAboutTheSameHeight(false),
    enabledPropagationBetweenTerminalsLocatedBelowRoofTop(true),
    currentAverageBuildingHeightMeters(0),
    wellBelowRoofTopHeightMeters(3.0),
    belowRoofTopCalculationLosDistanceMeters(0),
    transitionRegionMeters(20),
    belowRoofTopCalculationLosDeltaLossDb(0),
    belowRoofTopCalculationNlosDeltaLossDb(0),
    Lurban(6.8),
    enablePropagationPathTrace(false),
    roadLayerPtr(initRoadLayerPtr),
    buildingLayerPtr(initBuildingLayerPtr)
{
    if (theParameterDatabaseReader.ParameterExists(
            "p1411-los-calculation-policy", initInstanceId)) {

        string policyString = theParameterDatabaseReader.ReadString(
            "p1411-los-calculation-policy", initInstanceId);

        ConvertStringToLowerCase(policyString);

        if (policyString == "lower") {
            losCalculationPolicy = LOS_CALCULATION_POLICY_LOWERBOUND;
        }
        else if (policyString == "upper") {
            losCalculationPolicy = LOS_CALCULATION_POLICY_UPPERBOUND;
        }
        else if (policyString == "geometric") {
            losCalculationPolicy = LOS_CALCULATION_POLICY_GEOMETRICMEAN;
        }
        else if (policyString == "median") {
            losCalculationPolicy = LOS_CALCULATION_POLICY_MEDIAN;
        }
        else if (policyString == "freespace") {
            losCalculationPolicy = LOS_CALCULATION_POLICY_FREESPACE;
        }
        else {
            cerr << "Unknown policy (p1411-los-calculation-policy): " << policyString << endl;
            exit(1);
        }//if//

    }//if//

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-nlos1-calculation-policy", initInstanceId)) {

        string policyString = theParameterDatabaseReader.ReadString(
            "p1411-nlos1-calculation-policy", initInstanceId);

        ConvertStringToLowerCase(policyString);

        if (policyString == "suburban") {
            nlos1CalculationPolicy = NLOS1_CALCULATION_POLICY_SUBURBAN;
        }
        else if (policyString == "urban") {
            nlos1CalculationPolicy = NLOS1_CALCULATION_POLICY_URBAN;
        }
        else {
            cerr << "Unknown policy (p1411-nlos1-calculation-policy): " << policyString << endl;
            exit(1);
        }//if//
    }

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-nlos800to2000-calculation-policy", initInstanceId)) {

        string policyString = theParameterDatabaseReader.ReadString(
            "p1411-nlos800to2000-calculation-policy", initInstanceId);

        ConvertStringToLowerCase(policyString);

        if (policyString == "lower") {
            nlos800To2000CalculationPolicy = NLOS800TO2000_CALCULATION_POLICY_LOWER;
        }
        else if (policyString == "geometricmean") {
            nlos800To2000CalculationPolicy = NLOS800TO2000_CALCULATION_POLICY_GEOMETRICMEAN;
        }
        else if (policyString == "upper") {
            nlos800To2000CalculationPolicy = NLOS800TO2000_CALCULATION_POLICY_UPPER;
        }
        else {
            cerr << "Unknown policy (p1411-nlos800to2000-calculation-policy): " << policyString << endl;
            exit(1);
        }//if//        
    }

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-nlos2-calculation-policy", initInstanceId)) {

        string policyString = theParameterDatabaseReader.ReadString(
            "p1411-nlos2-calculation-policy", initInstanceId);

        ConvertStringToLowerCase(policyString);

        if (policyString == "residential") {
            Lcorner = 30;
        }
        else if (policyString == "urban") {
            Lcorner = 20;
        }
        else {
            cerr << "Unknown policy (p1411-nlos2-calculation-policy): " << policyString << endl;
            exit(1);
        }//if//    
    }

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-nlos2-use-policy", initInstanceId)) {

        string policyString = theParameterDatabaseReader.ReadString(
            "p1411-nlos2-use-policy", initInstanceId);

        ConvertStringToLowerCase(policyString);

        if (policyString == "alwaysuse800to2000mhzcalculation") {
            nlos2UsePolicy = NLOS2_ALWAYS_USE_800TO2000MHZ;
        }
        else if (policyString == "alwaysuse2to16ghzcalculation") {
            nlos2UsePolicy = NLOS2_ALWAYS_USE_2TO16GHZ;
        }
        else if (policyString == "default") {
            nlos2UsePolicy = NLOS2_USE_POLICY_DEFAULT;
        }
        else {
            cerr << "Unknown policy (p1411-nlos2-use-policy): " << policyString << endl;
            exit(1);
        }//if//
    }

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-nlos2-extension", initInstanceId)) {

        string policyString = theParameterDatabaseReader.ReadString(
            "p1411-nlos2-extension", initInstanceId);

        ConvertStringToLowerCase(policyString);

        if (policyString == "usedistancetolastnlosforcorrectionx1") {
            nlos2Extension = NLOS2_EXTENSION_USE_DISTANCE_TO_LAST_NLOS_FOR_CORRECTION_X1;
        }
        else if (policyString == "no") {
            nlos2Extension = NLOS2_EXTENSION_NONE;
        }
        else {
            cerr << "Unknown extension (p1411-nlos2-extension): " << policyString << endl;
            exit(1);
        }//if//
    }

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-nlos2-loss-direction", initInstanceId)) {

        string policyString = theParameterDatabaseReader.ReadString(
            "p1411-nlos2-loss-direction", initInstanceId);

        ConvertStringToLowerCase(policyString);

        if (policyString == "directional") {
            nlos2Direction = NLOS2_DIRECTION_DIRECTOINAL;
        }
        else if (policyString == "bidirectionallargeloss") {
            nlos2Direction = NLOS2_DIRECTION_BIDRECTIONAL_LARGE_LOSS;
        }
        else if (policyString == "bidirectionalsmallloss") {
            nlos2Direction = NLOS2_DIRECTION_BIDRECTIONAL_SMALL_LOSS;
        }
        else if (policyString == "smallnodeidtolargenodeidloss") {
            nlos2Direction = NLOS2_DIRECTION_SMALLNODEID_TO_LARGENODEID;
        }
        else if (policyString == "largenodeidtosmallnodeidloss") {
            nlos2Direction = NLOS2_DIRECTION_LARGENODEID_TO_SMALLNODEID;
        }
        else {
            cerr << "Unknown direction (p1411-nlos2-loss-direction): " << policyString << endl;
            exit(1);
        }//if//
    }

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-nlos2-use-larger-loss-at-nlos-bound", initInstanceId)) {
        useLargerLossAtNlosBound =
            theParameterDatabaseReader.ReadBool(
                "p1411-nlos2-use-larger-loss-at-nlos-bound", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-enable-shf-los-calculation", initInstanceId)) {

        if (isShf) {
            isShf = theParameterDatabaseReader.ReadBool(
                "p1411-enable-shf-los-calculation", initInstanceId);
        }
    }

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-shf-short-distance-meters", initInstanceId)) {

        shfShortDistanceMeters = theParameterDatabaseReader.ReadDouble(
            "p1411-shf-short-distance-meters", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-shf-effective-road-height-meters", initInstanceId)) {

        effectiveRoadHeightMeters = theParameterDatabaseReader.ReadDouble(
            "p1411-shf-effective-road-height-meters", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-building-height-differ-threshold-meters", initInstanceId)) {

        heightDifferThresholdMeters = theParameterDatabaseReader.ReadDouble(
            "p1411-building-height-differ-threshold-meters", initInstanceId);
    }


    allBuildingRoofTopsAreAboutTheSameHeight = true;

    const double heightVarianceMeters = buildingLayerPtr->GetBuildingHeightMetersVariance();

    if (heightVarianceMeters > heightDifferThresholdMeters) {
        allBuildingRoofTopsAreAboutTheSameHeight = false;
    }

    currentAverageBuildingHeightMeters =
        buildingLayerPtr->GetTotalBuildingHeightMeters() / buildingLayerPtr->GetNumberOfBuildings();

    if (std::abs(buildingLayerPtr->GetAverageBuildingHeightMeters() - currentAverageBuildingHeightMeters) >
        heightDifferThresholdMeters) {
        allBuildingRoofTopsAreAboutTheSameHeight = false;
    }

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-well-below-rooftop-height-meters", initInstanceId)) {

        wellBelowRoofTopHeightMeters = theParameterDatabaseReader.ReadDouble(
            "p1411-well-below-rooftop-height-meters", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists(
            "p1411-enable-propagation-between-terminals-located-below-rooftop-height-at-uhf", initInstanceId)) {

        enabledPropagationBetweenTerminalsLocatedBelowRoofTop =
            theParameterDatabaseReader.ReadBool(
                "p1411-enable-propagation-between-terminals-located-below-rooftop-height-at-uhf", initInstanceId);
    }

    if (enabledPropagationBetweenTerminalsLocatedBelowRoofTop) {
        int locationPercentage = 99;

        if (theParameterDatabaseReader.ParameterExists(
                "p1411-below-rooftop-location-percentage", initInstanceId)) {

            locationPercentage = theParameterDatabaseReader.ReadInt(
                "p1411-below-rooftop-location-percentage", initInstanceId);
        }

        switch(locationPercentage) {
        case 1:
            belowRoofTopCalculationLosDeltaLossDb = -11.3;
            belowRoofTopCalculationNlosDeltaLossDb = -16.3;
            belowRoofTopCalculationLosDistanceMeters = 976;
            break;
        case 10:
            belowRoofTopCalculationLosDeltaLossDb = -7.9;
            belowRoofTopCalculationNlosDeltaLossDb = -9.0;
            belowRoofTopCalculationLosDistanceMeters = 276;
            break;
        case 50:
            belowRoofTopCalculationLosDeltaLossDb = 0.0;
            belowRoofTopCalculationNlosDeltaLossDb = 0.0;
            belowRoofTopCalculationLosDistanceMeters = 44;
            break;
        case 90:
            belowRoofTopCalculationLosDeltaLossDb = 10.6;
            belowRoofTopCalculationNlosDeltaLossDb = 9.0;
            belowRoofTopCalculationLosDistanceMeters = 16;
            break;
        case 99:
            belowRoofTopCalculationLosDeltaLossDb = 20.3;
            belowRoofTopCalculationNlosDeltaLossDb = 16.3;
            belowRoofTopCalculationLosDistanceMeters = 10;
            break;
        default:
            assert(false && "Error: ITU-R P.1411 invalid location percentage.");
            break;
        }

        if (theParameterDatabaseReader.ParameterExists(
                "p1411-below-rooftop-transition-region-meters", initInstanceId)) {

            transitionRegionMeters = theParameterDatabaseReader.ReadDouble(
                "p1411-below-rooftop-transition-region-meters", initInstanceId);
        }

        if (theParameterDatabaseReader.ParameterExists(
                "p1411-below-rooftop-calculation-policy", initInstanceId)) {

            string policyString = theParameterDatabaseReader.ReadString(
                "p1411-below-rooftop-calculation-policy", initInstanceId);

            ConvertStringToLowerCase(policyString);

            if (policyString == "suburban") {
                Lurban = 0;
            }
            else if (policyString == "dense" || policyString == "high-rise") {
                Lurban = 2.3;
            }
            else if (policyString == "urban")  {
                Lurban = 6.8;
            }
            else {
                cerr << "Unknown policy (p1411-below-rooftop-calculation-policy): " << policyString << endl;
                exit(1);
            }//if//
        }
    }

    int maxDiffractionCount = 1;
    double losThresholdRadians = 1. * RADIANS_PER_DEGREE;
    double maxNlosDistance = DBL_MAX;

    if (theParameterDatabaseReader.ParameterExists("p1411-max-diffraction-count", initInstanceId)) {
        maxDiffractionCount = theParameterDatabaseReader.ReadInt("p1411-max-diffraction-count", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists("p1411-los-angle-degrees-between-roads", initInstanceId)) {
        losThresholdRadians = theParameterDatabaseReader.ReadDouble("p1411-los-angle-degrees-between-roads", initInstanceId) * RADIANS_PER_DEGREE;
    }

    if (theParameterDatabaseReader.ParameterExists("p1411-nlos-max-distance-meters", initInstanceId)) {
        maxNlosDistance = theParameterDatabaseReader.ReadDouble("p1411-nlos-max-distance-meters", initInstanceId);
    }

    if (theParameterDatabaseReader.ParameterExists("p1411-enable-building-based-los-calculation", initInstanceId)) {
        enabledBuildingLosCalculation =
            theParameterDatabaseReader.ReadBool("p1411-enable-building-based-los-calculation", initInstanceId);
    }

    roadLosCheckerPtr.reset(
        new RoadLosChecker(
            roadLayerPtr,
            shared_ptr<NlosPathValueCalculator>(new P1411NlosPathValueCalculator(this)),
            maxDiffractionCount,
            losThresholdRadians,
            maxNlosDistance));

    roadLosCheckerPtr->MakeLosRelation();
}

inline
bool P1411PropagationLossCalculationModel::PositionIsInABuilding(
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
double P1411PropagationLossCalculationModel::CalculatePropagationLossDb(
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

    if (nlos2Direction == NLOS2_DIRECTION_SMALLNODEID_TO_LARGENODEID) {
        if (txAntennaPosition.GetObjectId() > rxAntennaPosition.GetObjectId()) {
            std::swap(txPosition, rxPosition);
        }
    } else if (nlos2Direction == NLOS2_DIRECTION_LARGENODEID_TO_SMALLNODEID) {
        if (txAntennaPosition.GetObjectId() < rxAntennaPosition.GetObjectId()) {
            std::swap(txPosition, rxPosition);
        }
    }

    vector<RoadIdType> txRoadIds;
    vector<RoadIdType> rxRoadIds;

    roadLayerPtr->GetRoadIdsAt(txPosition, txRoadIds);
    roadLayerPtr->GetRoadIdsAt(rxPosition, rxRoadIds);

    if (txRoadIds.empty() || rxRoadIds.empty()) {
        return extremelyHighLossDb;
    }

    const double distanceMeters =
        XYDistanceBetweenVertices(txPosition, rxPosition);

    if (distanceMeters <= 0.) {
        return 0.;
    }

    if (enabledPropagationBetweenTerminalsLocatedBelowRoofTop &&
        (*this).TxAndRxAreWellBelowRoofTopAtUhf(txPosition, rxPosition)) {

        const double lossValueDb = (*this).CalculateBelowRoofTopHeightUhfPathlossDb(distanceMeters);
        (*this).TracePropagationPathIfNecessary(txPosition, rxPosition, lossValueDb);

        // A loss value between close points may be negative value. (for log10() calculation)
        return std::max<double>(0., lossValueDb);
    }

    if ((*this).PositionIsInABuilding(txPosition) ||
        (*this).PositionIsInABuilding(rxPosition)) {
        return extremelyHighLossDb;
    }

    double lossValueDb = extremelyHighLossDb;

    if ((enabledBuildingLosCalculation && buildingLayerPtr->PositionsAreLineOfSight(txPosition, rxPosition)) ||
        roadLosCheckerPtr->PositionsAreLineOfSight(txPosition, rxPosition)) {

        lossValueDb = (*this).CalculateLineOfSightPathlossDb(
            txPosition, rxPosition, distanceMeters);

    } else {

        for(size_t i = 0; i < rxRoadIds.size(); i++) {

            const RoadIdType rxRoadId = rxRoadIds[i];

            lossValueDb = std::min(
                lossValueDb,
                (*this).CalculateNonLineOfSightPathlossDb(
                    txPosition,
                    rxPosition,
                    distanceMeters,
                    rxRoadId));
        }
    }

    // A loss value between close points may be negative value. (for log10() calculation)
    return std::max<double>(0., lossValueDb);

}//CalculatePropagationLossDb;

inline
void P1411PropagationLossCalculationModel::CalculatePropagationPathInformation(
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
double P1411PropagationLossCalculationModel::CalculateLineOfSightPathlossDb(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const bool isPureLosCalculation,
    const bool forceMeanLossCalculation) const
{
    assert(distanceMeters > 0);

    if (!forceMeanLossCalculation) {
        if (losCalculationPolicy == LOS_CALCULATION_POLICY_FREESPACE) {

            const double lossValueDb = (*this).CalculateFreeSpaceLossDb(distanceMeters);

            if (isPureLosCalculation) {
                (*this).TracePropagationPathIfNecessary(txPosition, rxPosition, lossValueDb);
            }

            return lossValueDb;
        }
    }

    double txHeightMeters = txPosition.positionZ;
    double rxHeightMeters = rxPosition.positionZ;

    if (isShf && distanceMeters <= 1000) {

        // At SHF, for path lengths up to about 1 km,
        // road traffic will influence the effective road height
        // and will thus affect the breakpoint distance.

        if (rxPosition.positionZ <= effectiveRoadHeightMeters) {
            const double shfBasicShortLossDb =
                std::abs(20 * log10(wavelength / (2 * PI * shfShortDistanceMeters)));

            double lossValueDb = shfBasicShortLossDb;

            if (distanceMeters >= shfShortDistanceMeters) {
                if (losCalculationPolicy == LOS_CALCULATION_POLICY_MEDIAN) {
                    lossValueDb += 6 + 30 * log10(distanceMeters / shfShortDistanceMeters);
                } else if (losCalculationPolicy == LOS_CALCULATION_POLICY_LOWERBOUND) {
                    lossValueDb += 30 * log10(distanceMeters / shfShortDistanceMeters);
                } else if (losCalculationPolicy == LOS_CALCULATION_POLICY_UPPERBOUND) {
                    lossValueDb += 20 + 30 * log10(distanceMeters / shfShortDistanceMeters);
                }
            }

            if (isPureLosCalculation) {
                (*this).TracePropagationPathIfNecessary(txPosition, rxPosition, lossValueDb);
            }

            return lossValueDb;

        } else {
            //assert(txHeightMeters >= rxHeightMeters);

            txHeightMeters -= effectiveRoadHeightMeters;
            rxHeightMeters -= effectiveRoadHeightMeters;
        }
    }

    assert(txHeightMeters > 0);
    assert(rxHeightMeters > 0);
    assert(wavelength > 0);

    const double breakpointDistance = (4. * txHeightMeters * rxHeightMeters) / wavelength;
    const double breakpointLoss = std::abs(20 * log10((wavelength*wavelength) / (8 * PI * txHeightMeters * rxHeightMeters)));

    assert(breakpointDistance > 0);

    const double log10DistanceDivBpDistance = log10(distanceMeters/breakpointDistance);

    double lossValueDb;

    if (!forceMeanLossCalculation && losCalculationPolicy == LOS_CALCULATION_POLICY_MEDIAN) {

        if (distanceMeters <= breakpointDistance) {
            lossValueDb = breakpointLoss + 6 + 20 * log10DistanceDivBpDistance;
        } else {
            lossValueDb = breakpointLoss + 6 + 40 * log10DistanceDivBpDistance;
        }

    } else {
        double lowerLossDb;
        double upperLossDb;

        if (distanceMeters <= breakpointDistance) {
            lowerLossDb = breakpointLoss + 20 * log10DistanceDivBpDistance;
            upperLossDb = breakpointLoss + 20 + 25 * log10DistanceDivBpDistance;
        } else {
            lowerLossDb = breakpointLoss + 40 * log10DistanceDivBpDistance;
            upperLossDb = breakpointLoss + 20 + 40 * log10DistanceDivBpDistance;
        }

        if (forceMeanLossCalculation || losCalculationPolicy == LOS_CALCULATION_POLICY_GEOMETRICMEAN) {

            lossValueDb = (lowerLossDb + upperLossDb) / 2.;

        } else if (losCalculationPolicy == LOS_CALCULATION_POLICY_LOWERBOUND) {

            lossValueDb = lowerLossDb;

        } else if (losCalculationPolicy == LOS_CALCULATION_POLICY_UPPERBOUND) {

            lossValueDb = upperLossDb;
        }
        else {
            assert(false); abort(); lossValueDb = 0;
        }
    }

    if (isPureLosCalculation) {
        (*this).TracePropagationPathIfNecessary(txPosition, rxPosition, lossValueDb);
    }

    return lossValueDb;
}


inline
double P1411PropagationLossCalculationModel::CalculateNonLineOfSightPathlossDb(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const RoadIdType& rxRoadId) const
{
    double maxRoofHeightMeters;
    double averageRoofHeightMeters;
    double roofHeightDifferMeters;
    double averageBuildingSeparationMeters;
    double lengthMetersOfThePathCoveredByBuildings;

    (*this).CalculateLinearRoofCollisions(
        txPosition,
        rxPosition,
        distanceMeters,
        maxRoofHeightMeters,
        averageRoofHeightMeters,
        roofHeightDifferMeters,
        averageBuildingSeparationMeters,
        lengthMetersOfThePathCoveredByBuildings);

    if (txPosition.positionZ >= maxRoofHeightMeters) {

        const Road& rxRoad = roadLayerPtr->GetRoad(rxRoadId);
        const double rxRoadWidthMeters = rxRoad.GetWidthMeters();
        const double rayToRxStreetRadians =
            std::abs(std::abs(atan((txPosition.positionY - rxPosition.positionY) / (txPosition.positionX - rxPosition.positionX))) - std::abs(rxRoad.GetDirectionRadians()));

        if (nlos1CalculationPolicy == NLOS1_CALCULATION_POLICY_URBAN) {

            return (*this).CalculateNonLineOfSight1UrbanPathlossDb(
                txPosition,
                rxPosition,
                distanceMeters,
                rxRoadWidthMeters,
                averageRoofHeightMeters,
                averageBuildingSeparationMeters,
                roofHeightDifferMeters,
                lengthMetersOfThePathCoveredByBuildings,
                rayToRxStreetRadians);

        } else if (nlos1CalculationPolicy == NLOS1_CALCULATION_POLICY_SUBURBAN) {

            return (*this).CalculateNonLineOfSight1SuburbanPathlossDb(
                txPosition,
                rxPosition,
                distanceMeters,
                rxRoadWidthMeters,
                averageRoofHeightMeters,
                rayToRxStreetRadians);

        } else {
            assert(false && "Error: Failed to calculate ITU-R.p1411 NLOS1.");
            return extremelyHighLossDb;
        }

    } else {

        const double carrierFrequencyMhz = (*this).GetCarrierFrequencyMhz();

        vector<RoadIdType> txRoadIds;
        roadLayerPtr->GetRoadIdsAt(txPosition, txRoadIds);

        if (txRoadIds.empty()) {
            return extremelyHighLossDb;
        }

        double lossValueDb = extremelyHighLossDb;

        for(size_t i = 0; i < txRoadIds.size(); i++) {

            const RoadIdType txRoadId = txRoadIds[i];

            if (nlos2UsePolicy == NLOS2_ALWAYS_USE_2TO16GHZ ||
                (nlos2UsePolicy == NLOS2_USE_POLICY_DEFAULT &&
                 2000 <= carrierFrequencyMhz/* && carrierFrequencyMhz <= 16000*/)) {

                lossValueDb = std::min(
                    lossValueDb,
                    (*this).CalculateMinNonLineOfSight2Frequency2To16GHzPathlossDb(
                        txPosition,
                        rxPosition,
                        distanceMeters,
                        txRoadId,
                        rxRoadId));

            } else { // under 2GHz (800 to 2000 MHz)

                lossValueDb = std::min(
                    lossValueDb,
                    (*this).CalculateMinNonLineOfSight2Frequency800To2000MHzPathlossDb(
                        txPosition,
                        rxPosition,
                        distanceMeters,
                        txRoadId,
                        rxRoadId));
            }
        }

        return lossValueDb;
    }
}

inline
void P1411PropagationLossCalculationModel::CalculateLinearRoofCollisions(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    double& maxRoofHeightMeters,
    double& averageRoofHeightMeters,
    double& roofHeightDifferMeters,
    double& averageBuildingSeparationMeters,
    double& lengthMetersOfThePathCoveredByBuildings) const
{
    const BuildingLayer& aBuildingLayer = *buildingLayerPtr;

    if (!enabledBuildingLosCalculation ||
        !aBuildingLayer.EnabledLosCalculation()) {
        maxRoofHeightMeters = DBL_MAX;
        return;
    }

    vector<BuildingInformation> buildingInformation;

    double totalHeightMeters = 0;

    vector<Vertex> startCollisionPoints;
    vector<Vertex> endCollisionPoints;

    maxRoofHeightMeters = 0;

    const double minDistanceUnitMeters = 0.001; // 1mm

    const double heightLessThanMinBuildingHeight = aBuildingLayer.GetMinBuildingHeightMeters() / 2.;

    const Vertex txFloorPosition(txPosition.positionX, txPosition.positionY, heightLessThanMinBuildingHeight);
    const Vertex rxFloorPosition(rxPosition.positionX, rxPosition.positionY, heightLessThanMinBuildingHeight);

    vector<std::pair<Vertex, VariantIdType> > collisionPoints;
    aBuildingLayer.CalculateWallCollisionPoints(txFloorPosition, rxFloorPosition, collisionPoints);

    if (collisionPoints.empty()) {
        aBuildingLayer.CalculateWallCollisionPoints(txPosition, rxPosition, collisionPoints);
    }

    if (!collisionPoints.empty()) {
        size_t wallIndex = 0;
        bool lastBuildingIsVertex = true;

        while (wallIndex < collisionPoints.size() - 1) {

            const Vertex& wallPosition1 = collisionPoints[wallIndex].first;
            const Vertex& wallPosition2 = collisionPoints[wallIndex+1].first;

            const Vertex centerPoint(
                (wallPosition2.positionX - wallPosition1.positionX) * 0.5 + wallPosition1.positionX,
                (wallPosition2.positionY - wallPosition1.positionY) * 0.5 + wallPosition1.positionY,
                (wallPosition2.positionZ - wallPosition1.positionZ) * 0.5 + wallPosition1.positionZ);

            bool pointIsInABuilding;
            RoadIdType buildingId;

            aBuildingLayer.GetBuildingIdAt(centerPoint, pointIsInABuilding, buildingId);

            double buildingDistanceMeters = minDistanceUnitMeters;
            double heightMeters;

            if (pointIsInABuilding) {

                buildingDistanceMeters = sqrt(SquaredXYZDistanceBetweenVertices(wallPosition1, wallPosition2));
                wallIndex += 2;
                lastBuildingIsVertex = false;
                heightMeters = aBuildingLayer.GetBuilding(buildingId).GetHeightMeters();

            } else {

                wallIndex += 1;
                lastBuildingIsVertex = true;
                heightMeters = aBuildingLayer.GetCollisionPointHeight(collisionPoints[wallIndex]);
            }

            buildingInformation.push_back(BuildingInformation(heightMeters, buildingDistanceMeters));
            totalHeightMeters += heightMeters;

            maxRoofHeightMeters = std::max(maxRoofHeightMeters, heightMeters);

        }

        if (lastBuildingIsVertex) {
            const double heightMeters =
                aBuildingLayer.GetCollisionPointHeight(collisionPoints.back());

            buildingInformation.push_back(BuildingInformation(heightMeters, minDistanceUnitMeters));
            totalHeightMeters += heightMeters;

            maxRoofHeightMeters = std::max(maxRoofHeightMeters, heightMeters);
        }

        assert(!collisionPoints.empty());

        startCollisionPoints.push_back(collisionPoints.front().first);

        const Vertex& lastPoint = collisionPoints.back().first;

        endCollisionPoints.push_back(lastPoint);
    }

    for(size_t i = 0; i < startCollisionPoints.size(); i++) {
        for(size_t j = 0; j < endCollisionPoints.size(); j++) {

            double pathLengthMeters;

            if (startCollisionPoints[i] == endCollisionPoints[j]) {
                pathLengthMeters = minDistanceUnitMeters;
            } else {
                pathLengthMeters = XYZDistanceBetweenVertices(startCollisionPoints[i], endCollisionPoints[j]);
            }

            lengthMetersOfThePathCoveredByBuildings = std::max(
                lengthMetersOfThePathCoveredByBuildings, pathLengthMeters);
        }
    }

    averageBuildingSeparationMeters = distanceMeters / (buildingInformation.size() + 1);

    if (allBuildingRoofTopsAreAboutTheSameHeight) {

        averageRoofHeightMeters = currentAverageBuildingHeightMeters;
        roofHeightDifferMeters = 0;

    } else {

        if (!buildingInformation.empty()) {
            averageRoofHeightMeters = totalHeightMeters / buildingInformation.size();
        }  else {
            averageRoofHeightMeters = 0;
        }

        roofHeightDifferMeters = 0;

        for(size_t i = 0; i < buildingInformation.size(); i++) {

            const BuildingInformation& aBuildingData = buildingInformation[i];

            // use median

            if (std::abs(aBuildingData.heightMeters - averageRoofHeightMeters) > heightDifferThresholdMeters) {
                roofHeightDifferMeters += aBuildingData.distanceMeters;
            }
        }
    }
}

inline
double P1411PropagationLossCalculationModel::CalculateNonLineOfSight1UrbanPathlossDb(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const double rxRoadWidthMeters,
    const double averageRoofHeightMeters,
    const double averageBuildingSeparationMeters,
    const double roofHeightDifferMeters,
    const double lengthMetersOfThePathCoveredByBuildings,
    const double rayToRxStreetRadians) const
{
    // propagation urban area

    if (!(*this).IsAcceptableNlos1UrbanSituation(
            txPosition,
            rxPosition,
            distanceMeters,
            rxRoadWidthMeters,
            averageRoofHeightMeters)) {
        return extremelyHighLossDb;
    }

    assert(lengthMetersOfThePathCoveredByBuildings > 0 &&
           "Could't calculate NLOS1. Tx and Rx are line of sight.");

    //const double firstFrenelZoneMaxRadious = sqrt(wavelength * distanceMeters / 2.);

    //if (roofHeightDifferMeters <= firstFrenelZoneMaxRadious) {

        return (*this).CalculateRoofTopsNonLineOfSight1UrbanPathlossDb(
            txPosition,
            rxPosition,
            distanceMeters,
            rxRoadWidthMeters,
            averageRoofHeightMeters,
            averageBuildingSeparationMeters,
            lengthMetersOfThePathCoveredByBuildings,
            rayToRxStreetRadians);

    // } else {


        //return (*this).CalculateKnifeEdgePathlossDb();
    //}
}

inline
bool P1411PropagationLossCalculationModel::IsAcceptableNlos1UrbanSituation(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const double rxRoadWidthMeters,
    const double averageRoofHeightMeters) const
{
    if (!(4 <= txPosition.positionZ && txPosition.positionZ <= 50)) {
        cerr << "Warning (Propagation for urban area):" << endl
             << "tx height must be 4 to 50 [m]" << endl;
        return false;
    }

    if (!(1 <= rxPosition.positionZ && rxPosition.positionZ <= 3)) {
        cerr << "Warning (Propagation for urban area):" << endl
                 << "rx height must be 1 to 3 [m]" << endl;
        return false;
    }

    const double carrierFrequencyMhz = (*this).GetCarrierFrequencyMhz();

    if (!((/*800 <= carrierFrequencyMhz &&*/ carrierFrequencyMhz <= 5000) ||
          (2000 <= carrierFrequencyMhz && carrierFrequencyMhz <= 16000 &&
           txPosition.positionZ < averageRoofHeightMeters && rxRoadWidthMeters < 10))) {

        cerr << "Warning (Propagation for urban area):" << endl
             << "frequency must be 800 to 5000MHz" << endl
             << " or 2 to 16GHz for tx height < average building height and rx road width < 10 [m]" << endl;

        return false;
    }

    if (!(20 <= distanceMeters && distanceMeters <= 5000)) {
        //TBD// cerr << "Warning (Propagation for suburban are):" << endl
        //TBD//      << "tx-rx distance must be 20 to 5000 [m]" << endl;
        return false;
    }

    if (!(txPosition.positionZ > averageRoofHeightMeters)) {
        cerr << "Warning (Propagation for suburban are):" << endl
             << "roof height must be less than tx height." << endl;
        return false;
    }

    return true;
}

inline
double P1411PropagationLossCalculationModel::CalculateRoofTopsNonLineOfSight1UrbanPathlossDb(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const double rxRoadWidthMeters,
    const double roofHeightMeters,
    const double averageBuildingSeparationMeters,
    const double lengthMetersOfThePathCoveredByBuildings,
    const double rayToStreetRadians) const
{
    const double carrierFrequencyMhz = (*this).GetCarrierFrequencyMhz();
    const double txHeightMeters = txPosition.positionZ;
    const double rxHeightMeters = rxPosition.positionZ;
    const double distanceDiv1000Log10 = log10(distanceMeters / 1000.);
    const double averageBuildingSeparationMeters2 = averageBuildingSeparationMeters*averageBuildingSeparationMeters;

    const double phi = rayToStreetRadians / RADIANS_PER_DEGREE;

    double Lori;
    if (0 <= phi && phi < 35) {
        Lori = -10 + 0.354*phi;
    } else if (35 <= phi && phi < 55) {
        Lori = 2.5 + 0.075*(phi-35);
    } else {
        Lori = 4.0 - 0.114*(phi-55);
    }

    const double deltaTxHeightMeters = txHeightMeters - roofHeightMeters;
    const double deltaRxHeightMeters = roofHeightMeters - rxHeightMeters;

    const double Lrts =
        -8.2 - 10*log10(rxRoadWidthMeters) +
        10*carrierFrequencyMhzLog10 +
        20*log10(deltaRxHeightMeters) + Lori;

    assert(deltaTxHeightMeters > 0);

    const double settledFieldDistance =
        (wavelength * distanceMeters*distanceMeters) /
        (deltaTxHeightMeters*deltaTxHeightMeters);

    double Ka;
    if (txHeightMeters > roofHeightMeters) {
        if (carrierFrequencyMhz > 2000) {
            Ka = 71.4;
        } else {
            Ka = 54;
        }
    } else {
        if (carrierFrequencyMhz > 2000) {
            Ka = 73;
        } else {
            Ka = 54;
        }
        if (distanceMeters >= 500) {
            Ka -= 0.8*deltaTxHeightMeters;
        } else {
            Ka -= 1.6*deltaTxHeightMeters*distanceMeters/1000.;
        }
    }

    double Kd;
    if (txHeightMeters <= roofHeightMeters) {
        Kd = 18;
    } else {
        Kd = 18 - 15*(deltaTxHeightMeters/roofHeightMeters);
    }

    double Kf;
    if (carrierFrequencyMhz < 2000) {
        Kf = -8;
    } else if (carrierFrequencyMhz < 2000) {
        Kf = -4 + 0.7*(carrierFrequencyMhz/925. - 1);
    } else {
        Kf = -4 + 1.5*(carrierFrequencyMhz/925. - 1);
    }

    double Lbsh;
    if (txHeightMeters > roofHeightMeters) {
        Lbsh = - 18 * log10(1 + deltaTxHeightMeters);
    } else {
        Lbsh = 0;
    }

    assert(averageBuildingSeparationMeters > 0);

    const double sita = atan(deltaTxHeightMeters / averageBuildingSeparationMeters);
    const double rho =
        sqrt(deltaTxHeightMeters*deltaTxHeightMeters + averageBuildingSeparationMeters2);

    const double sqrtBuildingSeparationDivWavelength = sqrt(averageBuildingSeparationMeters/wavelength);

    const double deltaHuExponent =
        -log10(sqrtBuildingSeparationDivWavelength) -
        log10(distanceMeters)/9. +
        (10./9.)*log10(averageBuildingSeparationMeters/2.35);

    const double deltaHupp = pow(10, deltaHuExponent);

    const double deltaHlow =
        ((0.00023*averageBuildingSeparationMeters2 - 0.1827*averageBuildingSeparationMeters - 9.4978) /
         pow(log10(carrierFrequencyMhz), 2.938)) + 0.000781*averageBuildingSeparationMeters + 0.06923;

    assert(distanceMeters > 0);

    const double chi = 0.1;
    const double epsilon = 0.0417;
    const double breakpointDistanceMeters = std::abs(deltaTxHeightMeters) * sqrt(lengthMetersOfThePathCoveredByBuildings / wavelength);


    const double L1msdbp = CalculateL1msd(
        Lbsh,
        Ka,
        Kd,
        breakpointDistanceMeters,
        Kf,
        carrierFrequencyMhzLog10,
        averageBuildingSeparationMeters);

    const double L2msdbp = CalculateL2msd(
        txHeightMeters,
        roofHeightMeters,
        deltaHupp,
        deltaTxHeightMeters,
        breakpointDistanceMeters,
        averageBuildingSeparationMeters,
        wavelength,
        deltaHlow,
        rho,
        sita);

    const double Lupp = L1msdbp;
    const double Llow = L2msdbp;
    const double Ldiff = Lupp - Llow;
    const double Lmid = (Lupp + Llow) / 2;
    const double zeta = Ldiff * epsilon;

    // Note:: Lmsd uses log (not log10).

    double Lmsd;

    const double L1msd = CalculateL1msd(
        Lbsh,
        Ka,
        Kd,
        distanceMeters,
        Kf,
        carrierFrequencyMhzLog10,
        averageBuildingSeparationMeters);

    const double L2msd = CalculateL2msd(
        txHeightMeters,
        roofHeightMeters,
        deltaHupp,
        deltaTxHeightMeters,
        distanceMeters,
        averageBuildingSeparationMeters,
        wavelength,
        deltaHlow,
        rho,
        sita);

    if (Ldiff > 0) {

        assert(chi != 0);

        const double firstCoefficieent =
            tanh((log10(distanceMeters) - log10(breakpointDistanceMeters)) / chi);

        if (lengthMetersOfThePathCoveredByBuildings > settledFieldDistance) {
            Lmsd = - firstCoefficieent * (L1msd - Lmid) + Lmid;
        } else {
            Lmsd = firstCoefficieent * (L2msd - Lmid) + Lmid;
        }

    } else if (Ldiff == 0) {
        Lmsd = L2msd;
    } else {

        assert(zeta != 0);

        const double firstCoefficieent =
            tanh((log10(distanceMeters) - log10(breakpointDistanceMeters)) / zeta);

        if (lengthMetersOfThePathCoveredByBuildings > settledFieldDistance) {
            Lmsd = L1msd - firstCoefficieent * (Lupp - Lmid) - Lupp + Lmid;
        } else {
            Lmsd = L2msd + firstCoefficieent * (Lmid - Llow) + Lmid - Llow;
        }
    }

    const double freeSpaceLossDb =
        32.4 + 20*distanceDiv1000Log10 + 20*carrierFrequencyMhzLog10;

    double lossValueDb;

    if (Lrts + Lmsd > 0) {

        lossValueDb = (freeSpaceLossDb + Lrts + Lmsd);

    } else {

        lossValueDb = freeSpaceLossDb;
    }

    (*this).TracePropagationPathIfNecessary(txPosition, rxPosition, lossValueDb);
    return lossValueDb;
}

inline
double P1411PropagationLossCalculationModel::CalculateNonLineOfSight1SuburbanPathlossDb(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const double rxRoadWidthMeters,
    const double roofHeightMeters,
    const double rayToRxStreetRadians) const
{
    if (!(*this).IsAcceptableNlos1SuburbanSituation(
            txPosition,
            rxPosition,
            distanceMeters,
            rxRoadWidthMeters,
            roofHeightMeters)) {
        return extremelyHighLossDb;
    }

    return (*this).CalculateRoofTopsNonLineOfSight1SuburbanPathlossDb(
        txPosition,
        rxPosition,
        distanceMeters,
        rxRoadWidthMeters,
        roofHeightMeters,
        rayToRxStreetRadians);
}

inline
bool P1411PropagationLossCalculationModel::IsAcceptableNlos1SuburbanSituation(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const double rxRoadWidthMeters,
    const double averageRoofHeightMeters) const
{
    // propagation suburban area

    if (!(txPosition.positionZ > averageRoofHeightMeters)) {
        cerr << "Warning (Propagation for suburban area):" << endl
             << "tx height must be larger than average building height" << endl;
        return false;
    }

    const double deltaTxHeightMeters = txPosition.positionZ - averageRoofHeightMeters;

    if (!(1 <= deltaTxHeightMeters && deltaTxHeightMeters <= 100)) {
        cerr << "Warning (Propagation for suburban area):" << endl
             << "delta tx height must be 1 to 100 [m]" << endl;
        return false;
    }

    if (!(rxPosition.positionZ < averageRoofHeightMeters)) {
        cerr << "Warning (Propagation for suburban area):" << endl
             << "rx height must be less than average building height" << endl;
        return false;
    }

    const double deltaRxHeightMeters = averageRoofHeightMeters - rxPosition.positionZ;

    if (!(4 <= deltaRxHeightMeters && deltaRxHeightMeters <= 10)) {
        cerr << "Warning (Propagation for suburban are):" << endl
             << "delta rx height must be 4 to 10 [m]" << endl;
        return false;
    }

    const double carrierFrequencyMhz = (*this).GetCarrierFrequencyMhz();

    if (!(/*800 <= carrierFrequencyMhz && */carrierFrequencyMhz < 20000)) {
        cerr << "Warning (Propagation for suburban area):" << endl
             << "frequency must be 0.8 to 20GHz" << endl;
        return false;
    }

    if (!(10 <= rxRoadWidthMeters && rxRoadWidthMeters <= 25)) {
        cerr << "Warning (Propagation for suburban area):" << endl
             << "rx road width must be 10 to 25 [m]" << endl;
        return false;
    }

    if (!(10 <= distanceMeters && distanceMeters <= 50000)) {
        cerr << "Warning (Propagation for suburban area):" << endl
             << "tx-rx distance must be 10 to 5000 [m]" << endl;
        return false;
    }

    return true;
}


inline
double P1411PropagationLossCalculationModel::CalculateRoofTopsNonLineOfSight1SuburbanPathlossDb(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const double rxRoadWidthMeters,
    const double roofHeightMeters,
    const double rayToStreetRadians) const
{
    const double carrierFrequencyGhz = (*this).GetCarrierFrequencyMhz() / 1000.;
    const double txHeightMeters = txPosition.positionZ;
    const double rxHeightMeters = rxPosition.positionZ;

    const double txRxHeightDifferenceMeters = txHeightMeters - rxHeightMeters;

    assert(txPosition != rxPosition);

    const double deltaRxHeightMeters = roofHeightMeters - rxHeightMeters;

    vector<double> ds(3, 0);
    vector<double> Lds(3, 0);

    for(int i = 0; i < 3; i++) {
        CalculateLdn(
            i,
            txRxHeightDifferenceMeters,
            rxRoadWidthMeters,
            deltaRxHeightMeters,
            rayToStreetRadians,
            wavelength,
            ds[i],
            Lds[i]);
    }

    double lossValueDb;

    if (distanceMeters < ds[0]) {

        assert(wavelength > 0);
        lossValueDb = (20 * log10(4*PI*distanceMeters/wavelength));

    } else {

        const double dRd = CalculateDrd(carrierFrequencyGhz, ds[0], ds[1], ds[2]);
        const size_t maxK = 10;

        size_t k = 0;
        double LdRd = 0;
        bool L0nIsCase1 = true;

        while (k + 1 < maxK + 2) {

            while (ds.size() < k + 2) {
                ds.push_back(0);
                Lds.push_back(0);

                CalculateLdn(
                    ds.size(),
                    txRxHeightDifferenceMeters,
                    rxRoadWidthMeters,
                    deltaRxHeightMeters,
                    rayToStreetRadians,
                    wavelength,
                    ds.back(),
                    Lds.back());
            }

            LdRd = CalculateLdRd(ds, Lds, dRd, k);

            if (distanceMeters >= dRd) {

                if (ds[k] <= LdRd && LdRd <= ds[k+1]) {
                    break;
                }

            } else if (ds[k] <= distanceMeters &&
                       distanceMeters < ds[k+1] &&
                       ds[k+1] < dRd &&
                       dRd < ds[k+2]) {
                L0nIsCase1 = true;
                break;

            } else if (ds[k+1] <= distanceMeters &&
                       distanceMeters < dRd &&
                       dRd <  ds[k+2]) {
                L0nIsCase1 = false;
                break;
            }

            k++;
        }

        if (distanceMeters >= dRd) {

            lossValueDb = (32.1 * log10(distanceMeters/dRd) + LdRd);

        } else if (L0nIsCase1) {

            lossValueDb = Lds[k] + ((Lds[k+1] - Lds[k])/(ds[k+1] - ds[k])) * (distanceMeters - ds[k]);
        } else {

            lossValueDb = Lds[k+1] + ((LdRd - Lds[k+1])/(dRd - ds[k+1])) * (distanceMeters - ds[k+1]);
        }
    }

    (*this).TracePropagationPathIfNecessary(txPosition, rxPosition, lossValueDb);
    return lossValueDb;
}

inline
double P1411PropagationLossCalculationModel::CalculateMinNonLineOfSight2Frequency800To2000MHzPathlossDb(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const RoadIdType& txRoadId,
    const RoadIdType& rxRoadId) const
{
    const RoadLosRelationData& losRelationData =
        roadLosCheckerPtr->GetLosRelation(txRoadId, rxRoadId);

    assert(losRelationData.relationType != ROAD_LOSRELATION_LOS);

    if (losRelationData.relationType == ROAD_LOSRELATION_OUTOFNLOS) {
        return extremelyHighLossDb;
    }

    double minLossDb = extremelyHighLossDb;

    const map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>& nlosPaths = losRelationData.nlosPaths;

    typedef map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>::const_iterator NlosIter;

    for(NlosIter nlosIter = nlosPaths.begin(); nlosIter != nlosPaths.end(); nlosIter++) {
        const NlosPathData& nlosPath = (*nlosIter).second;

        if (!roadLosCheckerPtr->IsCompleteNlosPath(nlosPath, txRoadId, txPosition, rxPosition)) {
            continue;
        }

        double lossDb = (*this).CalculateNonLineOfSight2Frequency800To2000MHzPathlossDb(
            txPosition,
            rxPosition,
            txRoadId,
            rxRoadId,
            nlosPath);

        if (nlos2Direction == NLOS2_DIRECTION_BIDRECTIONAL_LARGE_LOSS ||
            nlos2Direction == NLOS2_DIRECTION_BIDRECTIONAL_SMALL_LOSS) {

            lossDb = std::max(
                lossDb,
                (*this).CalculateNonLineOfSight2Frequency800To2000MHzPathlossDb(
                    rxPosition,
                    txPosition,
                    rxRoadId,
                    txRoadId,
                    nlosPath));
        }

        (*this).TracePropagationPathIfNecessary(
            txPosition, rxPosition, lossDb,
            roadLosCheckerPtr->CalculateNlosPoints(nlosPath, txRoadId));

        minLossDb = std::min(minLossDb, lossDb);
    }

    if (nlos2Direction == NLOS2_DIRECTION_BIDRECTIONAL_LARGE_LOSS ||
        nlos2Direction == NLOS2_DIRECTION_BIDRECTIONAL_SMALL_LOSS) {

        const RoadLosRelationData& inverseLosRelationData =
            roadLosCheckerPtr->GetLosRelation(rxRoadId, txRoadId);

        const map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>& inverseNlosPaths = inverseLosRelationData.nlosPaths;

        double inverseMinLossDb = extremelyHighLossDb;

        for(NlosIter nlosIter = inverseNlosPaths.begin(); nlosIter != inverseNlosPaths.end(); nlosIter++) {

            const NlosPathData& nlosPath = (*nlosIter).second;

            if (!roadLosCheckerPtr->IsCompleteNlosPath(nlosPath, rxRoadId, rxPosition, txPosition)) {
                continue;
            }

            const double lossDb = std::max(
                (*this).CalculateNonLineOfSight2Frequency800To2000MHzPathlossDb(
                    rxPosition,
                    txPosition,
                    rxRoadId,
                    txRoadId,
                    nlosPath),
                (*this).CalculateNonLineOfSight2Frequency800To2000MHzPathlossDb(
                    txPosition,
                    rxPosition,
                    txRoadId,
                    rxRoadId,
                    nlosPath));

            (*this).TracePropagationPathIfNecessary(
                rxPosition, txPosition, lossDb,
                roadLosCheckerPtr->CalculateNlosPoints(nlosPath, rxRoadId));

            inverseMinLossDb = std::min(inverseMinLossDb, lossDb);
        }

        if (nlos2Direction == NLOS2_DIRECTION_BIDRECTIONAL_LARGE_LOSS) {
            minLossDb = std::max(minLossDb, inverseMinLossDb);
        } else {
            minLossDb = std::min(minLossDb, inverseMinLossDb);
        }
    }

    return minLossDb;
}

inline
double P1411PropagationLossCalculationModel::CalculateNonLineOfSight2Frequency800To2000MHzPathlossDb(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const RoadIdType& txRoadId,
    const RoadIdType& rxRoadId,
    const NlosPathData& nlosPath) const
{
    const double w1 = roadLayerPtr->GetRoad(txRoadId).GetWidthMeters();

    assert(nlosPath.GetNumberOfRoads() >= 2);

    // use second road width for w2
    const double w2 = roadLayerPtr->GetRoad(nlosPath.endToEndRoadIds[1]).GetWidthMeters();

    double x1 = roadLosCheckerPtr->CalculateNlosPointToStartPointCenterDistance(nlosPath, txRoadId, txPosition);
    double x2 = roadLosCheckerPtr->CalculateNlosPointToEndPointCenterDistance(nlosPath, txRoadId, rxPosition);

    if (x1 <= 0 || x2 <= 0) {
        return extremelyHighLossDb;
    }

    const double alpha = roadLosCheckerPtr->CalculateNlosPointRadians(nlosPath, txRoadId);

    if (!(0.6 < alpha && alpha < PI)) {
        return extremelyHighLossDb;
    }

    const double falpha = 3.86 / pow(alpha, 3.5);

    assert(w1 > 0 && w2 > 0);
    assert(wavelength > 0);

    double reflectionLossDb =
        20 * log10(x1+x2) + x1*x2*(falpha/(w1*w2)) +
        20 * log10((4*PI)/wavelength);

    const double Da = (40/(2*PI)) * (atan(x2/w2) + atan(x1/w1) - PI/2);

    double difractionLossDb =
        10 * log10(x1*x2*(x1+x2)) +
        2*Da - 0.1*(90-alpha*(180/PI)) +
        20*log10((4*PI)/wavelength);

    double lowerLnlos2 = 0;
    double upperLnlos2 = 0;

    if (nlos800To2000CalculationPolicy == NLOS800TO2000_CALCULATION_POLICY_LOWER ||
        nlos800To2000CalculationPolicy == NLOS800TO2000_CALCULATION_POLICY_GEOMETRICMEAN) {

        lowerLnlos2 = Calculate800To20000LossDb(reflectionLossDb, difractionLossDb);
    }
    if (nlos800To2000CalculationPolicy == NLOS800TO2000_CALCULATION_POLICY_UPPER ||
        nlos800To2000CalculationPolicy == NLOS800TO2000_CALCULATION_POLICY_GEOMETRICMEAN) {

        double correctionX1 = x1;

        if (nlos2Extension == NLOS2_EXTENSION_USE_DISTANCE_TO_LAST_NLOS_FOR_CORRECTION_X1) {
            correctionX1 = roadLosCheckerPtr->CalculateDistanceToLastNlosPoint(nlosPath, txRoadId, txPosition);
        }

        const bool isPureLosCalculation = false;
        const bool forceMeanLossCalculation = true;

        double upperBoundCorrectionLossDb =
            (*this).CalculateLineOfSightPathlossDb(
                txPosition,
                rxPosition,
                correctionX1,
                isPureLosCalculation,
                forceMeanLossCalculation) -
            (*this).CalculateFreeSpaceLossDb(correctionX1);

        reflectionLossDb += upperBoundCorrectionLossDb;
        difractionLossDb += upperBoundCorrectionLossDb;

        upperLnlos2 = Calculate800To20000LossDb(reflectionLossDb, difractionLossDb);
    }

    double Lnlos2;

    if (nlos800To2000CalculationPolicy == NLOS800TO2000_CALCULATION_POLICY_LOWER) {

        Lnlos2 = lowerLnlos2;

    } else if (nlos800To2000CalculationPolicy == NLOS800TO2000_CALCULATION_POLICY_UPPER) {

        Lnlos2 = upperLnlos2;

    } else if (nlos800To2000CalculationPolicy == NLOS800TO2000_CALCULATION_POLICY_GEOMETRICMEAN) {

        Lnlos2 = (lowerLnlos2 + upperLnlos2) / 2.;
    }
    else {
        assert(false); abort(); Lnlos2 = 0.0;
    }

    if (useLargerLossAtNlosBound) {
        const bool isPureLosCalculation = false;

        Lnlos2 = std::max(
            Lnlos2,
            (*this).CalculateLineOfSightPathlossDb(
                txPosition,
                rxPosition,
                roadLosCheckerPtr->CalculateStartPointToEndPointCenterDistance(nlosPath, txRoadId, txPosition, rxPosition),
                isPureLosCalculation,
                isPureLosCalculation));
    }

    return Lnlos2;
}

inline
double P1411PropagationLossCalculationModel::CalculateMinNonLineOfSight2Frequency2To16GHzPathlossDb(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const RoadIdType& txRoadId,
    const RoadIdType& rxRoadId) const
{
    const RoadLosRelationData& losRelationData =
        roadLosCheckerPtr->GetLosRelation(txRoadId, rxRoadId);

    assert(losRelationData.relationType != ROAD_LOSRELATION_LOS);

    if (losRelationData.relationType == ROAD_LOSRELATION_OUTOFNLOS) {
        return extremelyHighLossDb;
    }

    double minLossDb = extremelyHighLossDb;

    const map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>& nlosPaths = losRelationData.nlosPaths;

    typedef map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>::const_iterator NlosIter;

    for(NlosIter nlosIter = nlosPaths.begin(); nlosIter != nlosPaths.end(); nlosIter++) {
        const NlosPathData& nlosPath = (*nlosIter).second;

        if (!roadLosCheckerPtr->IsCompleteNlosPath(nlosPath, txRoadId, txPosition, rxPosition)) {
            continue;
        }

        double lossDb = (*this).CalculateNonLineOfSight2Frequency2To16GHzPathlossDb(
            txPosition,
            rxPosition,
            distanceMeters,
            txRoadId,
            rxRoadId,
            nlosPath);

        if (nlos2Direction == NLOS2_DIRECTION_BIDRECTIONAL_LARGE_LOSS ||
            nlos2Direction == NLOS2_DIRECTION_BIDRECTIONAL_SMALL_LOSS) {

            lossDb = std::max(
                lossDb,
                (*this).CalculateNonLineOfSight2Frequency800To2000MHzPathlossDb(
                    rxPosition,
                    txPosition,
                    rxRoadId,
                    txRoadId,
                    nlosPath));
        }

        (*this).TracePropagationPathIfNecessary(
            txPosition, rxPosition, lossDb,
            roadLosCheckerPtr->CalculateNlosPoints(nlosPath, txRoadId));

        minLossDb = std::min(minLossDb, lossDb);
    }

    if (nlos2Direction == NLOS2_DIRECTION_BIDRECTIONAL_LARGE_LOSS ||
        nlos2Direction == NLOS2_DIRECTION_BIDRECTIONAL_SMALL_LOSS) {

        const RoadLosRelationData& inverseLosRelationData =
            roadLosCheckerPtr->GetLosRelation(rxRoadId, txRoadId);

        const map<std::pair<IntersectionIdType, IntersectionIdType>, NlosPathData>& inverseNlosPaths = inverseLosRelationData.nlosPaths;

        double inverseMinLossDb = extremelyHighLossDb;

        for(NlosIter nlosIter = inverseNlosPaths.begin(); nlosIter != inverseNlosPaths.end(); nlosIter++) {

            const NlosPathData& nlosPath = (*nlosIter).second;

            if (!roadLosCheckerPtr->IsCompleteNlosPath(nlosPath, rxRoadId, rxPosition, txPosition)) {
                continue;
            }

            const double lossDb = std::max(
                (*this).CalculateNonLineOfSight2Frequency2To16GHzPathlossDb(
                    rxPosition,
                    txPosition,
                    distanceMeters,
                    rxRoadId,
                    txRoadId,
                    nlosPath),
                (*this).CalculateNonLineOfSight2Frequency800To2000MHzPathlossDb(
                    txPosition,
                    rxPosition,
                    txRoadId,
                    rxRoadId,
                    nlosPath));

            (*this).TracePropagationPathIfNecessary(
                rxPosition, txPosition, lossDb,
                roadLosCheckerPtr->CalculateNlosPoints(nlosPath, rxRoadId));

            inverseMinLossDb = std::min(inverseMinLossDb, lossDb);
        }

        if (nlos2Direction == NLOS2_DIRECTION_BIDRECTIONAL_LARGE_LOSS) {
            minLossDb = std::max(minLossDb, inverseMinLossDb);
        } else {
            minLossDb = std::min(minLossDb, inverseMinLossDb);
        }
    }

    return minLossDb;
}

inline
double P1411PropagationLossCalculationModel::CalculateNonLineOfSight2Frequency2To16GHzPathlossDb(
    const Vertex& txPosition,
    const Vertex& rxPosition,
    const double distanceMeters,
    const RoadIdType& txRoadId,
    const RoadIdType& rxRoadId,
    const NlosPathData& nlosPath) const
{
    const double w1 = roadLayerPtr->GetRoad(txRoadId).GetWidthMeters();
    const double Dcorner = 30;

    const double x1 = roadLosCheckerPtr->CalculateNlosPointToStartPointCenterDistance(nlosPath, txRoadId, txPosition);
    const double x2 = roadLosCheckerPtr->CalculateNlosPointToEndPointCenterDistance(nlosPath, txRoadId, rxPosition);

    if (x1 <= 0 || x2 <= 0) {
        return extremelyHighLossDb;
    }

    // Lc must be positive value.
    double Lc = 0;
    if (x2 > w1/2 + 1) {
        if (x2 <= w1/2 + 1 + Dcorner) {
            Lc = std::max(Lc, (Lcorner / log10(1+Dcorner))*log10(x2-w1/2));
        } else {
            Lc = Lcorner;
        }
    }

    assert(Lc >= 0);

    double Latt;
    if (/*w1/2 + 1 < x2 && */x2 > w1/2 + 1 + Dcorner) {
        const double beta = 6;

        Latt = 10 * beta * log10((x1+x2)/(x1 + w1/2 + Dcorner));
    } else {
        Latt = 0;
    }

    assert(Latt >= 0);

    const bool isPureLosCalculation = false;
    const bool forceMeanLossCalculation = false;

    // assert(x1 > 20);

    const double Llos = (*this).CalculateLineOfSightPathlossDb(
        txPosition, rxPosition, x1, isPureLosCalculation, forceMeanLossCalculation);

    double Lnlos2 = Llos + Lc + Latt;

    if (useLargerLossAtNlosBound) {
        Lnlos2 = std::max(
            Lnlos2,
            (*this).CalculateLineOfSightPathlossDb(
                txPosition,
                rxPosition,
                roadLosCheckerPtr->CalculateStartPointToEndPointCenterDistance(nlosPath, txRoadId, txPosition, rxPosition),
                isPureLosCalculation,
                forceMeanLossCalculation));
    }

    return Lnlos2;
}

inline
double P1411PropagationLossCalculationModel::CalculateL1msd(
    const double Lbsh,
    const double Ka,
    const double Kd,
    const double d,
    const double Kf,
    const double carrierFrequencyMhzLog10,
    const double averageBuildingSeparationMeters)
{
    return (Lbsh + Ka + Kd*log10(d/1000) +
            Kf*carrierFrequencyMhzLog10 -
            9*log10(averageBuildingSeparationMeters));
}

inline
double P1411PropagationLossCalculationModel::CalculateL2msd(
    const double txHeightMeters,
    const double roofHeightMeters,
    const double deltaHupp,
    const double deltaTxHeightMeters,
    const double d,
    const double averageBuildingSeparationMeters,
    const double wavelength,
    const double deltaHlow,
    const double rho,
    const double sita)
{
    double Qm;
    if (txHeightMeters > roofHeightMeters + deltaHupp) {
        Qm = 2.35 * pow((deltaTxHeightMeters/d) * sqrt(averageBuildingSeparationMeters/wavelength), 0.9);
    } else if (txHeightMeters >= roofHeightMeters + deltaHlow) {
        Qm = averageBuildingSeparationMeters / d;
    } else {
        Qm = averageBuildingSeparationMeters / (2*PI*d) * sqrt(wavelength/rho) * (1/sita - 1 / (2*PI + sita));
    }

    return (-10*log10(Qm*Qm));
}

inline
void P1411PropagationLossCalculationModel::CalculateLdn(
    const size_t k,
    const double txRxHeightDifferenceMeters,
    const double rxRoadWidthMeters,
    const double deltaRxHeightMeters,
    const double phi,
    const double wavelength,
    double& Dk,
    double& Ldk)
{
    const double txRxHeightDifferenceMeters2 = txRxHeightDifferenceMeters*txRxHeightDifferenceMeters;

    assert(deltaRxHeightMeters > 0);

    const double Ak = (rxRoadWidthMeters * txRxHeightDifferenceMeters * (2*k + 1)) / (2 * deltaRxHeightMeters);
    const double Bk = Ak - k*rxRoadWidthMeters;

    const double phik = atan((Bk/Ak) * tan(phi));

    const double Dkp = 1 / std::sin(phik) * sqrt(Ak*Ak + txRxHeightDifferenceMeters2);

    Dk = 1 / std::sin(phi) * sqrt(Bk*Bk + txRxHeightDifferenceMeters2);
    Ldk = 20 * log10((4*PI*Dkp) / (pow(0.4, int(k))*wavelength));
}

inline
double P1411PropagationLossCalculationModel::CalculateDrd(
    const double carrierFrequencyGhz,
    const double d1,
    const double d2,
    const double d3)
{
    return 0.625*(d3 - d1)*log10(carrierFrequencyGhz) + 0.44*d1 + 0.5*d2 + 0.06*d3;
}

inline
double P1411PropagationLossCalculationModel::CalculateLdRd(
    const vector<double>& ds,
    const vector<double>& Lds,
    const double dRd,
    const size_t k)
{
    return Lds.at(k+1) + ((Lds.at(k+2) - Lds.at(k+1))/(ds.at(k+2) - ds.at(k+1))) * (dRd - ds.at(k+1));
}

inline
bool P1411PropagationLossCalculationModel::TxAndRxAreWellBelowRoofTopAtUhf(
    const Vertex& txPosition,
    const Vertex& rxPosition) const
{
    const double carrierFrequencyMhz = (*this).GetCarrierFrequencyMhz();

    if (!(300 <= carrierFrequencyMhz && carrierFrequencyMhz <= 3000)) {
        return false;
    }

    if (!(txPosition.positionZ <= wellBelowRoofTopHeightMeters &&
          rxPosition.positionZ <= wellBelowRoofTopHeightMeters)) {
        return false;
    }

    return true;
}

inline
double P1411PropagationLossCalculationModel::CalculateBelowRoofTopHeightUhfPathlossDb(
    const double distanceMeters) const
{
    if (distanceMeters < belowRoofTopCalculationLosDistanceMeters) {
        return (*this).CalculateBelowRoofTopHeightLosPathlossDb(distanceMeters);
    }

    if (distanceMeters > belowRoofTopCalculationLosDistanceMeters + transitionRegionMeters) {
        return (*this).CalculateBelowRoofTopHeightNlosPathlossDb(distanceMeters);
    }

    const double Llos = (*this).CalculateBelowRoofTopHeightLosPathlossDb(
        belowRoofTopCalculationLosDistanceMeters);

    const double Lnlos = (*this).CalculateBelowRoofTopHeightNlosPathlossDb(
        belowRoofTopCalculationLosDistanceMeters + transitionRegionMeters);

    return ((Lnlos - Llos) * (distanceMeters - belowRoofTopCalculationLosDistanceMeters) / transitionRegionMeters);
}

inline
double P1411PropagationLossCalculationModel::CalculateBelowRoofTopHeightLosPathlossDb(
    const double distanceMeters) const
{
    const double Lmedian =
        32.45 + 20*carrierFrequencyMhzLog10 + 20*log10(distanceMeters/1000.);

    return (Lmedian + belowRoofTopCalculationLosDeltaLossDb);
}

inline
double P1411PropagationLossCalculationModel::CalculateBelowRoofTopHeightNlosPathlossDb(
    const double distanceMeters) const
{
    const double Lmedian =
        9.5 + 45*carrierFrequencyMhzLog10 + 40*log10(distanceMeters/1000.) + Lurban;

    return (Lmedian + belowRoofTopCalculationNlosDeltaLossDb);
}

inline
double P1411PropagationLossCalculationModel::CalculateFreeSpaceLossDb(
    const double distanceMeters) const
{
    return 20 * log10((4. * PI * distanceMeters) / wavelength);
}

inline
double P1411PropagationLossCalculationModel::Calculate800To20000LossDb(
    const double reflectionLossDb,
    const double difractionLossDb)
{
    return -10 * log10(pow(10,-reflectionLossDb/10) + pow(10,-difractionLossDb/10));
}

inline
void P1411PropagationLossCalculationModel::TracePropagationPathIfNecessary(
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
void P1411PropagationLossCalculationModel::TracePropagationPathIfNecessary(
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
double P1411PropagationLossCalculationModel::GetNlosPathValue(const NlosPathData& nlosPath) const
{
    const RoadIdType txRoadId = nlosPath.GetFrontRoadId();
    const RoadIdType rxRoadId = nlosPath.GetBackRoadId();

    const IntersectionIdType txIntersectionId = nlosPath.GetFrontIntersectionId();
    const IntersectionIdType rxIntersectionId = nlosPath.GetBackIntersectionId();

    const double w1 = roadLayerPtr->GetRoad(nlosPath.GetNlosRoadId()).GetRoadWidthMeters();
    const double w2 = roadLayerPtr->GetRoad(nlosPath.GetLastNlosRoadId()).GetRoadWidthMeters();

    Vertex txPosition =
        roadLayerPtr->GetRoad(txRoadId).GetInternalPoint(txIntersectionId, w1*0.5);

    Vertex rxPosition =
        roadLayerPtr->GetRoad(rxRoadId).GetInternalPoint(rxIntersectionId, w2*0.5);

    const double antennaHeightMeters = 1.5;

    txPosition.z = antennaHeightMeters;
    rxPosition.z = antennaHeightMeters;

    return(*this).CalculateNonLineOfSight2Frequency800To2000MHzPathlossDb(
        txPosition,
        rxPosition,
        txRoadId,
        rxRoadId,
        nlosPath);
}

} //namespace ScenSim//

#endif
