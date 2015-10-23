// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "nrlolsrglue.h"


namespace NrlProtolibPort {

inline
ProtoRouteMgr* ProtoRouteMgr::Create()
{
    return (ProtoRouteMgr*)(new ScenSim::ScenargieRouteMgr);

}//Create//

}//namespace//


namespace ScenSim {

const string NrlolsrProtocol::modelName = "Olsr";



bool ScenargieRouteMgr::SetRoute(
    const ProtoAddress& destinationProtoAddress,
    unsigned int prefixLength,
    const ProtoAddress& gatewayProtoAddress,
    unsigned int protoInterfaceIndex,
    int metric)
{
    bool routeIsSetToProtoRouteTable = protoRouteTable.SetRoute(
        destinationProtoAddress,
        prefixLength,
        gatewayProtoAddress,
        protoInterfaceIndex,
        metric);

    if (routeIsSetToProtoRouteTable){

        NetworkAddress destinationNetworkAddress(
            destinationProtoAddress.SimGetAddress());

        NetworkAddress destinationNetworkAddressSubnetMask =
            NetworkAddress(0xFFFFFFFF << (32 - prefixLength));

        SIMADDR gatewayRawAddress = gatewayProtoAddress.SimGetAddress();
        NetworkAddress nextHopNetworkAddress(gatewayRawAddress);
        if(gatewayRawAddress == 0){
            nextHopNetworkAddress = destinationNetworkAddress;
        }

        shared_ptr<RoutingTable> routingTablePtr =
            networkLayerPtr->GetRoutingTableInterface();

        routingTablePtr->AddOrUpdateRoute(
            destinationNetworkAddress,
            destinationNetworkAddressSubnetMask,
            nextHopNetworkAddress,
            interfaceIndex);

        nrlolsrProtocolPtr->OutputTraceAndStatsForAddRoutingTableEntry(
            destinationNetworkAddress,
            destinationNetworkAddressSubnetMask,
            nextHopNetworkAddress);

    }//if//

    return routeIsSetToProtoRouteTable;
}//SetRoute//



bool ScenargieRouteMgr::DeleteRoute(
    const ProtoAddress& destinationProtoAddress,
    unsigned int prefixLength,
    const ProtoAddress& gatewayProtoAddress,
    unsigned int protoInterfaceIndex)
{
    bool routeIsDeletedFromProtoRouteTable = protoRouteTable.DeleteRoute(
        destinationProtoAddress,
        prefixLength,
        &gatewayProtoAddress);

    if(routeIsDeletedFromProtoRouteTable){

        NetworkAddress destinationNetworkAddress(
            destinationProtoAddress.SimGetAddress());

        NetworkAddress destinationNetworkAddressSubnetMask =
            NetworkAddress(0xFFFFFFFF << (32 - prefixLength));

        shared_ptr<RoutingTable> routingTablePtr =
            networkLayerPtr->GetRoutingTableInterface();

        routingTablePtr->DeleteRoute(
            destinationNetworkAddress,
            destinationNetworkAddressSubnetMask);

        nrlolsrProtocolPtr->OutputTraceAndStatsForDeleteRoutingTableEntry(
            destinationNetworkAddress,
            destinationNetworkAddressSubnetMask);

    }//if//

    return routeIsDeletedFromProtoRouteTable;
}//DeleteRoute//



}//namespace//


