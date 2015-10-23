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

#include "scensim_proploss_iturp1411.h"
#include "scensim_proploss_taga.h"
#include "lte_extension_chooser.h"

namespace ScenSim {

shared_ptr<SimplePropagationLossCalculationModel>
CreatePropagationLossCalculationModel(
    const shared_ptr<SimulationEngine>& simulationEnginePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<GisSubsystem>& gisSubsystemPtr,
    const string& propModelName,
    const double& carrierFrequencyMhz,
    const double& maximumPropagationDistanceMeters,
    const bool propagationDelayIsEnabled,
    const unsigned int numberThreadsForDataParallelPropCalculation,
    const InterfaceOrInstanceIdType& instanceId)
{

    if (propModelName == "" || propModelName == "tworayground") {

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new TwoRayGroundPropagationLossCalculationModel(
                carrierFrequencyMhz,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation));

    }
    else if (propModelName == "freespace") {

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new FreeSpacePropagationLossCalculationModel(
                carrierFrequencyMhz,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation));

    }
    else if (propModelName == "okumurahata") {

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new OkumuraHataPropagationLossCalculationModel(
                theParameterDatabaseReader,
                carrierFrequencyMhz,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId));

    }
    else if (propModelName == "cost231hata") {

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new Cost231HataPropagationLossCalculationModel(
                theParameterDatabaseReader,
                carrierFrequencyMhz,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId));

    }
    else if (propModelName == "fupm") {

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new FupmPropagationLossCalculationModel(
                theParameterDatabaseReader,
                carrierFrequencyMhz,
                gisSubsystemPtr->GetInsiteGeometry(),
                propModelName,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId));
    }
    else if (propModelName == "hfpm") {

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new HfpmPropagationLossCalculationModel(
                theParameterDatabaseReader,
                carrierFrequencyMhz,
                gisSubsystemPtr->GetInsiteGeometry(),
                propModelName,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId));
    }
    else if (propModelName == "itm") {

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new ItmPropagationLossCalculationModel(
                theParameterDatabaseReader,
                carrierFrequencyMhz,
                gisSubsystemPtr->GetGroundLayerPtr(),
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId));
    }

    else if (propModelName == "itu-r_p.1411" ||
             propModelName == "itu-r.p1411-4" /*obsolete*/) {

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new P1411PropagationLossCalculationModel(
                theParameterDatabaseReader,
                carrierFrequencyMhz,
                gisSubsystemPtr->GetRoadLayerPtr(),
                gisSubsystemPtr->GetBuildingLayerPtr(),
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId));
    }
    else if (propModelName == "taga" ||
             propModelName == "simplified-taga") {

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new TagaPropagationLossCalculationModel(
                theParameterDatabaseReader,
                carrierFrequencyMhz,
                gisSubsystemPtr->GetRoadLayerPtr(),
                gisSubsystemPtr->GetBuildingLayerPtr(),
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId));
    }
    else if (propModelName == "trace") {

        if(numberThreadsForDataParallelPropCalculation != 1) {
            cerr << "Number of propagation calculation threads should be 1 for Trace propagation model." << endl;
            exit(1);
        }//if//

        const string defaultPropModelName =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("proptrace-default-propagation-model", instanceId));

        if (defaultPropModelName == "trace") {
            cerr << "Error: proptrace-default-propagation-model: " << defaultPropModelName << " is not valid." << endl;
            exit(1);
        }

        const shared_ptr<SimplePropagationLossCalculationModel> defaultPropagationCalculationModelPtr =
            CreatePropagationLossCalculationModel(
                simulationEnginePtr,
                theParameterDatabaseReader,
                gisSubsystemPtr,
                defaultPropModelName,
                carrierFrequencyMhz,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId);

        const unsigned int DUMMY_NODEID = 100000;

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new SingleThreadPropagationLossTraceModel(
                theParameterDatabaseReader,
                simulationEnginePtr->GetSimulationEngineInterface(
                    theParameterDatabaseReader,
                    DUMMY_NODEID, 0),
                carrierFrequencyMhz,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId,
                defaultPropagationCalculationModelPtr));
    }
    else if (propModelName == "cost231indoor" ||
             propModelName == "indoor") {

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new IndoorPropagationLossCalculationModel(
                theParameterDatabaseReader,
                gisSubsystemPtr,
                carrierFrequencyMhz,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId));
    }
    else if (propModelName == "tgaxindoor") {

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new TgAxIndoorPropLossCalculationModel(
                theParameterDatabaseReader,
                gisSubsystemPtr,
                carrierFrequencyMhz,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId));
    }
    else if (propModelName == "wallcount") {

        const string basePropModelName =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("propwallcount-baseline-propagation-model", instanceId));

        if ((basePropModelName == "twotier") || (basePropModelName == "wallcount")) {
            cerr << "Error: propwallcount-baseline-propagation-model: " << basePropModelName << " is not valid." << endl;
            exit(1);
        }

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new WallCountPropagationLossCalculationModel(
                CreatePropagationLossCalculationModel(
                    simulationEnginePtr,
                    theParameterDatabaseReader,
                    gisSubsystemPtr,
                    basePropModelName,
                    carrierFrequencyMhz,
                    maximumPropagationDistanceMeters,
                    propagationDelayIsEnabled,
                    numberThreadsForDataParallelPropCalculation,
                    instanceId),
                theParameterDatabaseReader,
                gisSubsystemPtr,
                carrierFrequencyMhz,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId));
    }
    else if (propModelName == "lte_macro" ||
             propModelName == "lte_pico") {

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new Lte::LtePropagationLossCalculationModel(
                propModelName,
                theParameterDatabaseReader,
                gisSubsystemPtr,
                carrierFrequencyMhz,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId));
    }
    else if (propModelName == "twotier") {

        const string defaultPropModelName =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("proptwotier-primary-propagation-model", instanceId));

        if (defaultPropModelName == "twotier") {
            cerr << "Error: proptwotier-primary-propagation-model: " << defaultPropModelName << " is not valid." << endl;
            exit(1);
        }

        const string selectedPropModelName =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("proptwotier-secondary-propagation-model", instanceId));

        if (selectedPropModelName == "twotier") {
            cerr << "Error: proptwotier-secondary-propagation-model: " << selectedPropModelName << " is not valid." << endl;
            exit(1);
        }

        return shared_ptr<SimplePropagationLossCalculationModel>(
            new TwoTierPropagationLossCalculationModel(
                CreatePropagationLossCalculationModel(
                    simulationEnginePtr,
                    theParameterDatabaseReader,
                    gisSubsystemPtr,
                    defaultPropModelName,
                    carrierFrequencyMhz,
                    maximumPropagationDistanceMeters,
                    propagationDelayIsEnabled,
                    numberThreadsForDataParallelPropCalculation,
                    instanceId),
                CreatePropagationLossCalculationModel(
                    simulationEnginePtr,
                    theParameterDatabaseReader,
                    gisSubsystemPtr,
                    selectedPropModelName,
                    carrierFrequencyMhz,
                    maximumPropagationDistanceMeters,
                    propagationDelayIsEnabled,
                    numberThreadsForDataParallelPropCalculation,
                    instanceId),
                theParameterDatabaseReader,
                carrierFrequencyMhz,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId));


    }
    else if (propModelName == "custom") {
        // Custom propagation model, set in subclass.
    }
    else {
        cerr << "Error: Propagation Model: " << propModelName << " is not valid." << endl;
        exit(1);

    }//if//

    return shared_ptr<SimplePropagationLossCalculationModel>();

}//CreatePropagationLossCalculationModel//

}//namespace//
