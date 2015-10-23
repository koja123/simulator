// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_ISCDHCP_H
#define SCENSIM_ISCDHCP_H

#include "scensim_netsim.h"
#include "iscdhcpglue.h"

namespace ScenSim {

class IscDhcpApplication
    :
    public Application,
    public DhcpClientInterface,
    public enable_shared_from_this<IscDhcpApplication> {
public:

    IscDhcpApplication(
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const NodeIdType& initNodeId,
        const bool initIsServer);

    virtual ~IscDhcpApplication() {}

    void CompleteInitialization(
        const ParameterDatabaseReader& theParameterDatabaseReader);

    void HandleLinkIsUpNotification(
        const unsigned int interfaceIndex,
        const GenericMacAddressType& genericMacAddress);

    void EnableForThisInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex);

private:
    const NodeIdType nodeId;
    const bool isServer;

    shared_ptr<IscDhcp> iscDhcpPtr;

};//IscDhcpApplication//

inline
IscDhcpApplication::IscDhcpApplication(
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const NodeIdType& initNodeId,
    const bool initIsServer)
    :
    Application(initSimEngineInterfacePtr, ""/*applicationId*/),
    nodeId(initNodeId),
    isServer(initIsServer),
    iscDhcpPtr()
{
}//IscDhcpApplication//

inline
void IscDhcpApplication::CompleteInitialization(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    iscDhcpPtr = shared_ptr<IscDhcp>(
        new IscDhcp(
            simulationEngineInterfacePtr,
            transportLayerPtr,
            transportLayerPtr->GetNetworkLayerPtr(),
            aRandomNumberGeneratorPtr,
            nodeId,
            isServer));

    iscDhcpPtr->CompleteInitialization(theParameterDatabaseReader);

}//CompleteInitialization//

inline
void IscDhcpApplication::HandleLinkIsUpNotification(
    const unsigned int interfaceIndex,
    const GenericMacAddressType& genericMacAddress)
{
    assert(iscDhcpPtr);

    iscDhcpPtr->HandleLinkIsUpNotification(
        interfaceIndex,
        genericMacAddress);

}//HandleLinkIsUpNotification//

inline
void IscDhcpApplication::EnableForThisInterface(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const InterfaceIdType& interfaceId,
    const unsigned int interfaceIndex)
{
    assert(iscDhcpPtr);

    iscDhcpPtr->EnableForThisInterface(
        theParameterDatabaseReader,
        interfaceId,
        interfaceIndex);

}//EnableInterface//

}//namespace ScenSim//

#endif //SCENSIM_ISCDHCP_H//
