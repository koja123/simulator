// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef LTE_EXTENSION_NOTAVAILABLE_H

#ifndef LTE_EXTENSION_CHOOSER_H
#error "Include lte_extension_chooser.h not lte_extension_notavailable.h (indirect include only)"
#endif

#include "scensim_prop.h"
#include "scensim_proploss.h"
#include "scensim_gis.h"
#include "scensim_mac.h"
#include "scensim_application.h"
#include "scensim_network.h"
#include "scensim_mimo_channel.h"
#include "scensim_freqsel_channel.h"

namespace Lte {

using std::cerr;
using std::endl;
using std::string;
using std::shared_ptr;
using ScenSim::TimeType;
using ScenSim::NodeIdType;
using ScenSim::SimplePropagationLossCalculationModel;
using ScenSim::GisSubsystem;
using ScenSim::ParameterDatabaseReader;
using ScenSim::InterfaceOrInstanceIdType;
using ScenSim::ObjectMobilityPosition;
using ScenSim::MacLayer;
using ScenSim::SimulationEngineInterface;
using ScenSim::BitOrBlockErrorRateCurveDatabase;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::NetworkLayer;
using ScenSim::ApplicationLayer;
using ScenSim::NetworkAddress;

const string parameterNamePrefix = "";

class DownlinkFrame {};
class UplinkFrame {};

typedef shared_ptr<const DownlinkFrame> PropModelDownlinkFrameType;

typedef ScenSim::SimplePropagationModel<PropModelDownlinkFrameType> DownlinkPropagationModel;
typedef ScenSim::SimplePropagationModelForNode<PropModelDownlinkFrameType> DownlinkPropagationModelInterface;

typedef shared_ptr<const UplinkFrame> PropModelUplinkFrameType;

typedef ScenSim::SimplePropagationModel<PropModelUplinkFrameType> UplinkPropagationModel;
typedef ScenSim::SimplePropagationModelForNode<PropModelUplinkFrameType> UplinkPropagationModelInterface;


inline
void CheckDownlinkAndUplinkChannelNumbers(
    const unsigned int baseDownlinkChannelNumber,
    const unsigned int numberDownlinkChannels,
    const unsigned int baseUplinkChannelNumber,
    const unsigned int numberUplinkChannels,
    unsigned int& baseChannelNumber,
    unsigned int& totalNumberChannels)

{
    cerr << "Error: LTE module is not available." << endl;
    cerr << "Please confirm that the executable (simulator) enables LTE Module." << endl;
    exit(1);
}


inline
void ChooseAndCreateChannelModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceOrInstanceIdType& propagationModeInstanceId,
    const unsigned int baseChannelNumber,
    const unsigned int numberChannels,
    shared_ptr<ScenSim::MimoChannelModel>& mimoChannelModelPtr,
    shared_ptr<ScenSim::FrequencySelectiveFadingModel>& frequencySelectiveFadingModelPtr)
{
    cerr << "Error: LTE module is not available." << endl;
    cerr << "Please confirm that the executable (simulator) enables LTE Module." << endl;
    exit(1);
}


class LteGlobalParameters {
public:
    LteGlobalParameters(const ParameterDatabaseReader& theParameterDatabaseReader) {
        cerr << "Error: LTE module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables LTE Module." << endl;
        exit(1);
    }

    InterfaceOrInstanceIdType GetDownlinkPropagationInstanceId() { return "ltedownlink"; }
    InterfaceOrInstanceIdType GetUplinkPropagationInstanceId() { return "lteuplink"; }

};//LteGlobalParameters//

class LtePropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:
    LtePropagationLossCalculationModel(
        const string& propModelName,
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
        const double& initCarrierFrequencyMhz,
        const double& initMaximumPropagationDistanceMeters,
        const bool initPropagationDelayIsEnabled,
        const int initNumberDataParallelThreads,
        const InterfaceOrInstanceIdType& initInstanceId)
        :
        SimplePropagationLossCalculationModel(initCarrierFrequencyMhz)
    {}

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        const double& xyDistanceSquaredMeters) const {
        cerr << "Error: LTE propagation option is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables LTE Module." << endl;
        exit(1);
    }

};//LtePropagationLossCalculationModel//

inline
shared_ptr<ScenSim::MacLayer> LteFactory(
    const ParameterDatabaseReader& theParameterFileReader,
    const NodeIdType& theNodeId,
    const ScenSim::InterfaceIdType& interfaceId,
    const unsigned int macInterfaceNumber,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<DownlinkPropagationModelInterface>& downlinkPropModelInterfacePtr,
    const shared_ptr<UplinkPropagationModelInterface>& uplinkPropModelInterfacePtr,
    const shared_ptr<ScenSim::MimoChannelModel>& downloadMimoChannelModelPtr,
    const shared_ptr<ScenSim::MimoChannelModel>& uploadMimoChannelModelPtr,
    const shared_ptr<ScenSim::FrequencySelectiveFadingModel>& downloadFadingModelPtr,
    const shared_ptr<ScenSim::FrequencySelectiveFadingModel>& uploadFadingModelPtr,
    const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
    const shared_ptr<const LteGlobalParameters>& globalParamsPtr,
    const RandomNumberGeneratorSeedType& nodeSeed,
    const shared_ptr<NetworkLayer>& networkLayerPtr,
    ApplicationLayer& appLayer)
{
    cerr << "Error: LTE MAC is not available." << endl;
    cerr << "Please confirm that the executable (simulator) enables LTE Module." << endl;
    exit(1);
}//LteFactory//

class LteGatewayController {
public:
    static shared_ptr<LteGatewayController> Create(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const NetworkAddress& interfacesNetworkAddress,
        ApplicationLayer& appLayer)
    {
        cerr << "Error: LTE module is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables LTE Module." << endl;
        exit(1);
        return shared_ptr<LteGatewayController>();
    }//Create//
};//LteGatewayController//


}; //namespace Lte//

#endif
