// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef ISCDHCPGLUE_H
#define ISCDHCPGLUE_H

#include "scensim_engine.h"
#include "scensim_transport.h"
#include "scensim_network.h"
#include "scensim_application.h"

namespace ScenSim {

class IscDhcpImplementation;

#ifdef USE_ISCDHCP
class IscDhcp {
public:
    IscDhcp(
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<TransportLayer>& initTransportLayerPtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const shared_ptr<RandomNumberGenerator>& initRandomNumberGeneratorPtr,
        const NodeIdType& initNodeId,
        const bool initIsServer);

    virtual ~IscDhcp();

    void CompleteInitialization(
        const ParameterDatabaseReader& theParameterDatabaseReader);

    void HandleLinkIsUpNotification(
        const unsigned int interfaceIndex,
        const GenericMacAddressType& genericMacAddress);

    void EnableForThisInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex);

    void Reboot(const TimeType& waitTime);

private:
    shared_ptr<IscDhcpImplementation> implPtr;
    IscDhcp(const IscDhcp&);
    IscDhcp& operator=(const IscDhcp&);

};//IscDhcp//
#else
class IscDhcp {
public:
    IscDhcp(
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<TransportLayer>& initTransportLayerPtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const shared_ptr<RandomNumberGenerator>& initRandomNumberGeneratorPtr,
        const NodeIdType& initNodeId,
        const bool initIsServer) { assert(false); abort(); }

    virtual ~IscDhcp() { assert(false); }

    void CompleteInitialization(
        const ParameterDatabaseReader& theParameterDatabaseReader) { assert(false); abort(); }

    void HandleLinkIsUpNotification(
        const unsigned int interfaceIndex,
        const GenericMacAddressType& genericMacAddress) { assert(false); abort(); }

    void EnableForThisInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex) { assert(false); abort(); }

    void Reboot(const TimeType& waitTime) { assert(false); abort(); }

private:
    shared_ptr<IscDhcpImplementation> implPtr;
    IscDhcp(const IscDhcp&);
    IscDhcp& operator=(const IscDhcp&);

};//IscDhcp//
#endif//USE_ISCDHCP//

}//namespace ScenSim//

#endif //ISCDHCPGLUE_H//
