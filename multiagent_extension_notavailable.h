// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef MULTI_AGENT_EXTENSION_NOTAVAILABLE_H
#define MULTI_AGENT_EXTENSION_NOTAVAILABLE_H

#ifndef MULTI_AGENT_EXTENSION_CHOOSER_H
#error "Include multiagent_extension_chooser.h (indirect include only)"
#endif

#include "scensim_netsim.h"
#include "scensim_prop.h"

namespace MultiAgent {

using std::cerr;
using std::endl;
using std::string;
using std::map;
using std::shared_ptr;
using ScenSim::SimulationEngine;
using ScenSim::NetworkSimulator;
using ScenSim::BasicNetworkNode;
using ScenSim::GlobalNetworkingObjectBag;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::SimulationEngineInterface;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeIdType;
using ScenSim::InterfaceIdType;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::ObjectMobilityModel;
using ScenSim::TimeType;

class AgentCommunicationNode : public BasicNetworkNode {
public:
    AgentCommunicationNode(
        const ParameterDatabaseReader& parameterDatabaseReader,
        const GlobalNetworkingObjectBag& initGlobalNetworkingObjectBag,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr,
        const NodeIdType& initNodeId,
        const RandomNumberGeneratorSeedType& initRunSeed)
        :
        BasicNetworkNode(
            parameterDatabaseReader,
            initGlobalNetworkingObjectBag,
            initSimulationEngineInterfacePtr,
            initNodeMobilityModelPtr,
            initNodeId,
            initRunSeed),
        nodeMobilityModelPtr(initNodeMobilityModelPtr)
    {}

protected:
    shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;

private:
    virtual void Attach(const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr) { nodeMobilityModelPtr = initNodeMobilityModelPtr; }
    virtual void Detach() { nodeMobilityModelPtr.reset(); }
};//AgentCommunicationNode//

class MultiAgentSimulator : public NetworkSimulator {
public:
    MultiAgentSimulator(
        const shared_ptr<ParameterDatabaseReader>& initParameterDatabaseReaderPtr,
        const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
        const RandomNumberGeneratorSeedType& initRunSeed,
        const bool initRunSequentially)
         :
         NetworkSimulator(
             initParameterDatabaseReaderPtr,
             initSimulationEnginePtr,
             initRunSeed,
             initRunSequentially)
    {}

    void AddCommunicationNode(const shared_ptr<AgentCommunicationNode>& aNodePtr) {
        (*this).AddNode(aNodePtr);
    }

    void CreateCommunicationNodeAtWakeupTimeFor(const NodeIdType& agentId) {}

protected:
    map<NodeIdType, TimeType> wakeupTimes;

    bool IsEqualToAgentId(const NodeIdType& nodeId) const { return false; }
    TimeType GetWakeupTime(const NodeIdType& nodeId) const { return ScenSim::ZERO_TIME; }

};//MultiAgentSimulator//

} //namespace


#endif
