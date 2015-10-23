// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_APPLICATION_H
#define SCENSIM_APPLICATION_H

#include "randomnumbergen.h"

#include "scensim_parmio.h"
#include "scensim_engine.h"
#include "scensim_transport.h"
#include "scensim_network.h"
#include "scensim_mac.h"

namespace ScenSim {

using std::istringstream;
using std::shared_ptr;
using std::dynamic_pointer_cast;
using std::vector;
using std::map;
using std::make_pair;

class NetworkAddressLookupInterface;
class TransportLayer;
class ObjectMobilityModel;
class ApplicationLayer;

typedef string ApplicationIdType;

class Application {
public:
    Application(
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const ApplicationIdType initApplicationId)
    :
        simulationEngineInterfacePtr(initSimEngineInterfacePtr), applicationId(initApplicationId) { }

    virtual ~Application() { }

    virtual void DisconnectFromOtherLayers();

    shared_ptr<MacAndPhyInfoInterface> GetMacAndPhyInfoInterface(
        const InterfaceIdType& interfaceId);

protected:
    ApplicationIdType applicationId;

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;

    // Note these are set when Application is added to the App Layer.
    shared_ptr<NetworkAddressLookupInterface> networkAddressLookupInterfacePtr;
    shared_ptr<TransportLayer> transportLayerPtr;
    shared_ptr<RandomNumberGenerator> aRandomNumberGeneratorPtr;
    shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;
    shared_ptr<ApplicationLayer> applicationLayerPtr;

    friend class ApplicationLayer;

};//Application//

inline
void Application::DisconnectFromOtherLayers()
{
    (*this).transportLayerPtr.reset();
    (*this).applicationLayerPtr.reset();
}

inline
shared_ptr<MacAndPhyInfoInterface> Application::GetMacAndPhyInfoInterface(
    const InterfaceIdType& interfaceId)
{
    shared_ptr<NetworkLayer> networkLayerPtr = transportLayerPtr->GetNetworkLayerPtr();
    const unsigned int interfaceIndex = networkLayerPtr->LookupInterfaceIndex(interfaceId);
    shared_ptr<MacLayer> macLayerPtr = networkLayerPtr->GetMacLayerPtr(interfaceIndex);
    shared_ptr<MacAndPhyInfoInterface> macAndPhyInfoInterfacePtr =
        macLayerPtr->GetMacAndPhyInfoInterface();
    return macAndPhyInfoInterfacePtr;

};//GetMacAndPhyInfoInterface//

NodeIdType App_ConvertStringToNodeIdOrAnyNodeId(const string& nodeIdString);

class ApplicationLayer : public enable_shared_from_this<ApplicationLayer> {
public:
    ApplicationLayer(
        const shared_ptr<NetworkAddressLookupInterface>& networkAddressLookupInterfacePtr,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<TransportLayer>& transportLayerPtr,
        const shared_ptr<ObjectMobilityModel>& nodeMobilityModelPtr,
        const NodeIdType& initNodeId,
        const RandomNumberGeneratorSeedType& initNodeSeed);

    void AddApp(const shared_ptr<Application>& appPtr);

    void DisconnectFromOtherLayers();

    template <typename T>
    shared_ptr<T> GetApplicationPtr(const ApplicationIdType& applicationId);

    unsigned short int GetNewApplicationInstanceNumber() {
        assert(applicationInstanceNumber < USHRT_MAX);
        applicationInstanceNumber++;
        return applicationInstanceNumber;
    }

private:
    static const int SEED_HASH = 1087324;

    map<ApplicationIdType, shared_ptr<Application> > applications;

    shared_ptr<NetworkAddressLookupInterface> networkAddressLookupInterfacePtr;
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    shared_ptr<TransportLayer> transportLayerPtr;
    shared_ptr<RandomNumberGenerator> aRandomNumberGeneratorPtr;
    shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;

    unsigned short int applicationInstanceNumber;

};//ApplicationLayer//




inline
ApplicationLayer::ApplicationLayer(
    const shared_ptr<NetworkAddressLookupInterface>& initNetworkAddressLookupInterfacePtr,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<TransportLayer>& initTransportLayerPtr,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
    const NodeIdType& initNodeId,
    const RandomNumberGeneratorSeedType& initNodeSeed)
    :
    networkAddressLookupInterfacePtr(initNetworkAddressLookupInterfacePtr),
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    transportLayerPtr(initTransportLayerPtr),
    nodeMobilityModelPtr(initNodeMobilityModelPtr),
    aRandomNumberGeneratorPtr(new RandomNumberGenerator(HashInputsToMakeSeed(initNodeSeed, initNodeId, SEED_HASH))),
    applicationInstanceNumber(0)
{
}


inline
void ApplicationLayer::DisconnectFromOtherLayers()
{
    map<ApplicationIdType, shared_ptr<Application> >::iterator iter =
        applications.begin();

    while (iter != applications.end()) {
        iter->second->DisconnectFromOtherLayers();
        ++iter;
    }//while//

}//DisconnectFromOtherLayers//


template <typename T> inline
shared_ptr<T> ApplicationLayer::GetApplicationPtr(
    const ApplicationIdType& applicationId)
{
    map<ApplicationIdType, shared_ptr<Application> >::iterator iter =
        applications.find(applicationId);

    if (iter != applications.end()) {
        return dynamic_pointer_cast<T>(iter->second);
    }
    else {
        return shared_ptr<T>();
    }//if//

}//GetApplicationPtr//

inline
void ApplicationLayer::AddApp(const shared_ptr<Application>& appPtr)
{

    // Integrate App into node.
    if (appPtr->simulationEngineInterfacePtr == nullptr) {
        appPtr->simulationEngineInterfacePtr = simulationEngineInterfacePtr;
    }//if//
    assert(appPtr->simulationEngineInterfacePtr == simulationEngineInterfacePtr);

    appPtr->networkAddressLookupInterfacePtr = networkAddressLookupInterfacePtr;
    appPtr->transportLayerPtr = transportLayerPtr;
    appPtr->aRandomNumberGeneratorPtr = aRandomNumberGeneratorPtr;
    appPtr->nodeMobilityModelPtr = nodeMobilityModelPtr;
    appPtr->applicationLayerPtr = shared_from_this();

    applications.insert(make_pair(appPtr->applicationId, appPtr));

}


//-----------------------------------------------------------------------------

class FloodingApplication;
class BundleProtocol;

class ApplicationMaker {
public:
    static const int FIRST_DESTINATION_PORT_NUMBER = 37700;

    ApplicationMaker(
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<ApplicationLayer>& appLayerPtr,
        const NodeIdType& nodeId,
        const RandomNumberGeneratorSeedType& nodeSeed);

    void ReadApplicationLineFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag);

    void ReadSpecificApplicationLineFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const NodeIdType& sourceNodeId,
        const InterfaceOrInstanceIdType& instanceId);

private:
    enum ApplicationType {
        APPLICATION_CBR,
        APPLICATION_CBR_WITH_QOS,
        APPLICATION_VBR,
        APPLICATION_VBR_WITH_QOS,
        APPLICATION_FTP,
        APPLICATION_FTP_WITH_QOS,
        APPLICATION_MULTIFTP,
        APPLICATION_MULTIFTP_WITH_QOS,
        APPLICATION_VIDEO,
        APPLICATION_VIDEO_WITH_QOS,
        APPLICATION_VOIP,
        APPLICATION_VOIP_WITH_QOS,
        APPLICATION_HTTP,
        APPLICATION_HTTP_WITH_QOS,
        APPLICATION_FLOODING,
        APPLICATION_IPERF_UDP,
        APPLICATION_IPERF_UDP_CLIENT,
        APPLICATION_IPERF_UDP_SERVER,
        APPLICATION_IPERF_UDP_WITH_QOS,
        APPLICATION_IPERF_TCP,
        APPLICATION_IPERF_TCP_CLIENT,
        APPLICATION_IPERF_TCP_SERVER,
        APPLICATION_IPERF_TCP_WITH_QOS,
        APPLICATION_BUNDLE_PROTOCOL,
        APPLICATION_BUNDLE_MESSAGE,
        APPLICATION_TRACE_BASED,
        APPLICATION_SENSING

        //Add new app
    };

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    shared_ptr<ApplicationLayer> appLayerPtr;

    NodeIdType nodeId;
    RandomNumberGeneratorSeedType nodeSeed;

    set<NodeIdType> commNodeIds;

    map<ApplicationType, string> appSpecificParameterNames;

    shared_ptr<FloodingApplication> floodingApplicationPtr; //One app instance per node
    shared_ptr<BundleProtocol> bundleProtocolPtr; //One app instance per node
    unsigned short defaultBundleProtocolPortId; //should be shared between all bundle protocol

    struct ApplicationInstanceInfo {
        NodeIdType nodeIdWithParameter;
        InterfaceOrInstanceIdType instanceId;
        unsigned int globalApplicationInstanceNumber;

        ApplicationInstanceInfo()
            :
            nodeIdWithParameter(INVALID_NODEID),
            instanceId(nullInstanceId),
            globalApplicationInstanceNumber(0)
        {}

        ApplicationInstanceInfo(
            const NodeIdType& initNodeIdWithParameter,
            const InterfaceOrInstanceIdType& initInstanceId,
            const unsigned int initGlobalApplicationInstanceNumber)
            :
            nodeIdWithParameter(initNodeIdWithParameter),
            instanceId(initInstanceId),
            globalApplicationInstanceNumber(initGlobalApplicationInstanceNumber)
        {}

        unsigned short int GetDefaultDestinationPortNumber() const {
            return (static_cast<unsigned short int>(
                (FIRST_DESTINATION_PORT_NUMBER + globalApplicationInstanceNumber))); }

    };//ApplicationInstanceInfo//

    void ReadApplicationIntancesFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const map<ApplicationType, vector<ApplicationInstanceInfo> >& applicationInstanceIdsPerApplicationModel);

    void ReadCbrFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadCbrWithQosFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadVbrFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadVbrWithQosFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadFtpFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadFtpWithQosFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadMultiFtpFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadMultiFtpWithQosFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadVideoFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadVideoWithQosFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadVoipFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadVoipWithQosFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadHttpFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadHttpWithQosFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadFloodingFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadIperfUdpFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId,
        const bool createClient,
        const bool createServer);

    void ReadIperfUdpWithQosFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadIperfTcpFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId,
        const bool createClient,
        const bool createServer);

    void ReadIperfTcpWithQosFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadBundleProtocolFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadBundleMessageFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadTraceBasedFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const ApplicationInstanceInfo& applicationInstanceId);

    void ReadSensingFromConfig(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
        const ApplicationInstanceInfo& applicationInstanceId);

    //Add new app


    void GetApplicationInstanceIds(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        map<ApplicationType, vector<ApplicationInstanceInfo> >& applicationInstanceIdsPerApplicationModel);
};//ApplicationMaker//


}//namespace//


#endif
