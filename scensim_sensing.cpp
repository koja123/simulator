// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_sensing.h"
#include "scensim_netsim.h"

namespace ScenSim {

using std::make_shared;

const string SensingApplication::modelName = "SensingApp";

shared_ptr<SensingModel> SensingSubsystemInterface::CreateSensingModel(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeIdType& initNodeId,
    const InterfaceOrInstanceIdType& initInstanceId,
    const RandomNumberGeneratorSeedType& initNodeSeed)
{
    return sensingSubsystemPtr->CreateSensingModel(
        parameterDatabaseReader,
        initSimulationEngineInterfacePtr,
        (*this).shared_from_this(),
        initNodeId,
        initInstanceId,
        initNodeSeed);
}//CreateSensingModel//

shared_ptr<ShapeSensingModel> SensingSubsystemInterface::CreateShapeSensingModel(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const NodeIdType& initNodeId,
    const InterfaceOrInstanceIdType& initInstanceId,
    const RandomNumberGeneratorSeedType& initNodeSeed)
{
    return sensingSubsystemPtr->CreateShapeSensingModel(
        initSimulationEngineInterfacePtr,
        (*this).shared_from_this(),
        initNodeId,
        initInstanceId,
        initNodeSeed);
}//CreateShapeSensingModel//

void SensingSubsystemInterface::TransmitDataIdeally(
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<SensingModel>& transmitterSensingPtr,
    const SensingSharedInfoType& sharedInfo,
    const TransmissionConditionType& transmissionCondition,
    const double errorRate)
{
    sensingSubsystemPtr->TransmitDataIdeally(
        simulationEngineInterfacePtr,
        transmitterSensingPtr,
        sharedInfo,
        transmissionCondition,
        errorRate);
}//TransmitDataIdeally//

const SensingGlobalInfo& SensingSubsystemInterface::GetSensingGlobalInfo() const
{
    return *sensingSubsystemPtr->theSensingGlobalInfoPtr;
}//GetSensingGlobalInfo//


void ShapeSensingModel::TransmitDataToDetectedCommNodes(
    const SensingSharedInfoType& sharedInfo)
{
    sensingSubsystemInterfacePtr->TransmitDataIdeally(
        simulationEngineInterfacePtr,
        (*this).shared_from_this(),
        sharedInfo,
        transmissionCondition,
        transmissionDataErrorRate);
}//TransmitDataToDetectedCommNodes//


ObjectMobilityPosition NetworkSimulatorInterfaceForSensingSubsystem::GetNodePosition(const NodeIdType& nodeId) const
{
    return networkSimulatorPtr->GetNodePosition(nodeId);
}//GetNodePosition//

TimeType NetworkSimulatorInterfaceForSensingSubsystem::GetTimeStepEventSynchronizationStep() const
{
    return networkSimulatorPtr->GetTimeStepEventSynchronizationStep();
}//GetTimeStepEventSynchronizationStep//


shared_ptr<SensingModel> SensingSubsystem::CreateSensingModel(
    const ParameterDatabaseReader& parameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<SensingSubsystemInterface>& initSensingSubsystemInterfacePtr,
    const NodeIdType& initNodeId,
    const InterfaceOrInstanceIdType& initInstanceId,
    const RandomNumberGeneratorSeedType& initNodeSeed)
{
    shared_ptr<SensingModel> sensingModelPtr;

    const string sensingShapeString = MakeLowerCaseString(
        parameterDatabaseReader.ReadString("sensing-coverage-shape-type", initNodeId, initInstanceId));

    if (sensingShapeString == "fanshape") {
        shared_ptr<SensingShape> sensingShapePtr(
            new FanSensingShape(
                parameterDatabaseReader,
                initNodeId,
                initInstanceId));

        sensingModelPtr =
            make_shared<ShapeSensingModel>(
                parameterDatabaseReader,
                initSimulationEngineInterfacePtr,
                initSensingSubsystemInterfacePtr,
                theGisSubsystemPtr,
                sensingShapePtr,
                initNodeId,
                initInstanceId,
                initNodeSeed);

    }
    else if (sensingShapeString == "gisobject") {
        shared_ptr<SensingShape> sensingShapePtr(
            new GisObjectSensingShape(
                parameterDatabaseReader,
                theGisSubsystemPtr,
                initNodeId,
                initInstanceId));

        sensingModelPtr =
            make_shared<ShapeSensingModel>(
                parameterDatabaseReader,
                initSimulationEngineInterfacePtr,
                initSensingSubsystemInterfacePtr,
                theGisSubsystemPtr,
                sensingShapePtr,
                initNodeId,
                initInstanceId,
                initNodeSeed);
    }
    else {
        cerr << "Set sensing-coverage-shape-type value FanShape or GisObject." << endl;
        exit(1);
    }//if//

    (*this).AddSensingModelToLocalPartition(
        initSimulationEngineInterfacePtr,
        sensingModelPtr,
        initNodeId,
        initInstanceId);

    return sensingModelPtr;
}//CreateSensingApplication//


shared_ptr<ShapeSensingModel> SensingSubsystem::CreateShapeSensingModel(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<SensingSubsystemInterface>& initSensingSubsystemInterfacePtr,
    const NodeIdType& initNodeId,
    const InterfaceOrInstanceIdType& initInstanceId,
    const RandomNumberGeneratorSeedType& initNodeSeed)
{
    shared_ptr<ShapeSensingModel> sensingModelPtr(
        new ShapeSensingModel(
            initSimulationEngineInterfacePtr,
            initSensingSubsystemInterfacePtr,
            theGisSubsystemPtr,
            initNodeId,
            initInstanceId,
            initNodeSeed));

    (*this).AddSensingModelToLocalPartition(
        initSimulationEngineInterfacePtr,
        sensingModelPtr,
        initNodeId,
        initInstanceId);

    return sensingModelPtr;
}//CreateSimpleFanSensing//


shared_ptr<SensingSubsystemInterface> SensingSubsystem::CreateSubsystemInterfacePtr()
{
    return shared_ptr<SensingSubsystemInterface>(new SensingSubsystemInterface(this));
}//CreateSubsystemInterfacePtr//


void SensingSubsystem::ExecuteTimestepBasedEvent(const TimeType& time)
{
    if (!(*this).NeedToUpdateSensingInfo()) {
        return;
    }//if//

    (*this).UpdateMobilityPositionCache();

    (*this).DeliveryAllSharingInfo();

    // set global information next update time
    theSensingGlobalInfoPtr->nextTopologyUpdateTime =
        time + networkSimulatorInterfacePtr->GetTimeStepEventSynchronizationStep();

}//ExecuteTimestepBasedEvent//


void SensingSubsystem::UpdateMobilityPositionCache()
{
    map<NodeIdType, ObjectMobilityPosition>& mobilityPositionCache = theSensingGlobalInfoPtr->mobilityPositionCache;

    mobilityPositionCache.clear();

    const GroundLayer& groundLayer = (*theGisSubsystemPtr->GetGroundLayerPtr());

    // update mobility poisitons
    typedef list<NodeIdType>::const_iterator MobIter;

    for(MobIter iter = nodeIds.begin(); (iter != nodeIds.end()); iter++) {
        const NodeIdType nodeId = (*iter);

        ObjectMobilityPosition mobilityPosition =
            networkSimulatorInterfacePtr->GetNodePosition(nodeId);

        // Adjust ground height if necessary
        if (!mobilityPosition.TheHeightContainsGroundHeightMeters()) {

            const double groundHeightMeters =
                groundLayer.GetElevationMetersAt(
                    MakeVertexFromMobilityPosition(mobilityPosition));

            mobilityPosition.SetHeightFromGroundMeters(
                groundHeightMeters + mobilityPosition.HeightFromGroundMeters());
        }//if//

        mobilityPositionCache.insert(make_pair(nodeId,  mobilityPosition));

    }//for//

}//MakeMobilityPositionCache//


}//namespace//
