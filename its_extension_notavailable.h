// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef ITS_EXTENSION_NOTAVAILABLE_H

#ifndef ITS_EXTENSION_CHOOSER_H
#error "Include its_extension_chooser.h (indirect include only)"
#endif

#include "scensim_proploss.h"

#include "scensim_prop.h"
#include "scensim_gis.h"
#include "scensim_mac.h"
#include "scensim_network.h"
#include "scensim_application.h"
#include "dot11_extension_chooser.h"

using std::cerr;
using std::endl;
using std::string;
using std::shared_ptr;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::SimulationEngineInterface;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeIdType;
using ScenSim::InterfaceIdType;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::NetworkLayer;
using ScenSim::BitOrBlockErrorRateCurveDatabase;
using ScenSim::MacLayer;
using ScenSim::Application;
using Dot11::Dot11Phy;
using ScenSim::ObjectMobilityModel;
using ScenSim::ObjectMobilityPosition;
using ScenSim::ApplicationIdType;
using ScenSim::TimeType;

namespace Mates {

class MatesMobilityModel : public ObjectMobilityModel {
public:
    MatesMobilityModel() {}

    virtual void GetUnadjustedPositionForTime(
        const TimeType& snapshotTime,
        ObjectMobilityPosition& position) override
    {
        cerr << "Error: MATES mobility option is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables ITS Extention Module." << endl;
        exit(1);
    }

};//MatesMobilityModel//

}; //namespace Mates//

namespace Wave {

typedef uint8_t ChannelNumberIndexType;
typedef uint8_t ChannelCategoryType;

static inline
void GetChannelCategoryAndNumber(
    const string& channelString,
    ChannelCategoryType& channelCategory,
    ChannelNumberIndexType& channelNumberId)
{}//GetChannelCategoryAndNumber//

struct WaveMacPhyInput {
    string phyDeviceName;
    shared_ptr<Dot11Phy> phyPtr;
    vector<ChannelNumberIndexType> channelNumberIds[2];
};//WaveMacPhyInput

class WaveMac : public MacLayer {
public:
    WaveMac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int initInterfaceIndex,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const vector<WaveMacPhyInput>& initPhyInputs,
        const RandomNumberGeneratorSeedType& nodeSeed)
    {
        cerr << "Error: WAVE is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables ITS Extention Module." << endl;
        exit(1);
    }

    void NetworkLayerQueueChangeNotification() {}
    void DisconnectFromOtherLayers() {}

};//WaveMac//

class WsmpLayer {
public:
    WsmpLayer(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const InterfaceIdType& initInterfaceId,
        const RandomNumberGeneratorSeedType& initNodeSeed,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const shared_ptr<WaveMac>& initWaveMacPtr)
    {
        cerr << "Error: WSMP is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables ITS Extention Module." << endl;
        exit(1);
    }
};//WsmpLayer//

class DsrcMessageApplication : public Application {
public:
    DsrcMessageApplication(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<WsmpLayer>& initWsmpLayerPtr,
        const NodeIdType& initNodeId,
        const RandomNumberGeneratorSeedType& initNodeSeed,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr)
        :
        Application(initSimEngineInterfacePtr, ApplicationIdType())
    {
        cerr << "Error: DsrcMessageApplication is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables ITS Extention Module." << endl;
        exit(1);
    }
};//DsrcMessageApplication//

}; //namespace Wave//

namespace GeoNet {

class GeoNetMac : public MacLayer {
public:
    GeoNetMac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const InterfaceIdType& initInterfaceId,
        const unsigned int initInterfaceIndex,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const shared_ptr<Dot11Phy>& initDot11PhyPtr,
        const RandomNumberGeneratorSeedType& nodeSeed)
    {
        cerr << "Error: GeoNet is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables ITS Extention Module." << endl;
        exit(1);
    }

    void NetworkLayerQueueChangeNotification() {}
    void DisconnectFromOtherLayers() {}

};//GeoNetMac//

class GeoNetworkingProtocol {
public:
    GeoNetworkingProtocol(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        const shared_ptr<GeoNetMac>& initGeoNetMacPtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr)
    {
        cerr << "Error: GeoNet is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables ITS Extention Module." << endl;
        exit(1);
    }

};//GeoNetworkingProtocol//

class BasicTransportProtocol {
public:
    BasicTransportProtocol() {
        cerr << "Error: GeoNet is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables ITS Extention Module." << endl;
        exit(1);
    }
    BasicTransportProtocol(const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr) {
        cerr << "Error: GeoNet is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables ITS Extention Module." << endl;
        exit(1);
    }

    void ConnectToNetworkLayer(const shared_ptr<GeoNetworkingProtocol>& initGeoNetworkingPtr) {}

};//BasicTransportProtocol//

}; //namespace GeoNet//

namespace Its {

class ItsBroadcastApplication : public Application {
public:

    enum TransportLayerType {
        UDP_MODE, BTP_MODE
    };

    ItsBroadcastApplication(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const InterfaceIdType& initInterfaceId,
        const TransportLayerType& tranportLayerType,
        const shared_ptr<GeoNet::BasicTransportProtocol>& initBasicTransportProtocolPtr = shared_ptr<GeoNet::BasicTransportProtocol>())
        :
        Application(initSimEngineInterfacePtr, ApplicationIdType())
    {
        cerr << "Error: ItsBroadcastApplication is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables ITS Extention Module." << endl;
        exit(1);
    }

    void CompleteInitialization(const ParameterDatabaseReader& theParameterDatabaseReader) {}

};//ItsBroadcastApplication//

}; //namespace Its//

namespace T109 {

class T109Phy {
public:
    struct PropFrameType {};

    T109Phy() {}
};//T109Phy//

class T109Mac : public MacLayer {
public:
    T109Mac() {}

    void NetworkLayerQueueChangeNotification() {}
    void DisconnectFromOtherLayers() {}
};//T109Mac//

class T109App : public Application {
public:
    T109App(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<SimplePropagationModelForNode<T109Phy::PropFrameType> >& initPropModelInterfacePtr,
        const shared_ptr<BitOrBlockErrorRateCurveDatabase>& initBerCurveDatabasePtr,
        const NodeIdType& initNodeId,
        const InterfaceIdType& initInterfaceId,
        const size_t initInterfaceIndex,
        const RandomNumberGeneratorSeedType& initNodeSeed,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr)
        :
        Application(initSimEngineInterfacePtr, ApplicationIdType())
    {
        cerr << "Error: T109 is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables ITS Extention Module." << endl;
        exit(1);
    }

    shared_ptr<T109Mac> GetMacPtr() const { return shared_ptr<T109Mac>(); }

};//T109App//

}; //namespace T109//

#endif
