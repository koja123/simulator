/*********************************************************************
 *
 * AUTHORIZATION TO USE AND DISTRIBUTE
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that: 
 *
 * (1) source code distributions retain this paragraph in its entirety, 
 *  
 * (2) distributions including binary code include this paragraph in
 *     its entirety in the documentation or other materials provided 
 *     with the distribution, and 
 *
 * (3) all advertising materials mentioning features or use of this 
 *     software display the following acknowledgment:
 * 
 *      "This product includes software written and developed 
 *       by Code 5520 of the Naval Research Laboratory (NRL)." 
 *         
 *  The name of NRL, the name(s) of NRL  employee(s), or any entity
 *  of the United States Government may not be used to endorse or
 *  promote  products derived from this software, nor does the 
 *  inclusion of the NRL written and developed software  directly or
 *  indirectly suggest NRL or United States  Government endorsement
 *  of this product.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 ********************************************************************/

#include "protoRouteMgr.h"
#include "protoDebug.h"


namespace NrlProtolibPort //ScenSim-Port://
{ //ScenSim-Port://



bool ProtoRouteMgr::DeleteAllRoutes(ProtoAddress::Type addrType, unsigned int maxIterations)
{
    ProtoRouteTable rt;
    // Make multiple passes to get rid of possible
    // multiple routes to destinations
    // (TBD) extend ProtoRouteTable to support multiple routes per destination
    while (maxIterations-- > 0)
    {
        if (!GetAllRoutes(addrType, rt))
        {
            DMSG(0, "ProtoRouteMgr::DeleteAllRoutes() error getting routes\n");
            return false;
        }
        if (rt.IsEmpty()) break;
        if (!DeleteRoutes(rt))
        {
            DMSG(0, "ProtoRouteMgr::DeleteAllRoutes() error deleting routes\n");
            return false;
        }
        rt.Destroy();
    }

    if (0 == maxIterations)
    {
        DMSG(0, "ProtoRouteMgr::DeleteAllRoutes() couldn't seem to delete everything!\n");
        return false;
    }
    else
    {
        return true;
    }
}  // end ProtoRouteMgr::DeleteAllRoutes()

bool ProtoRouteMgr::SetRoutes(const ProtoRouteTable& routeTable)
{
    bool result = true;
    ProtoRouteTable::Iterator iterator(routeTable);
    ProtoRouteTable::Entry* entry;
    // First, set direct (interface) routes
    while (true)
    {
        entry = iterator.GetNextEntry();
        if (entry == nullptr) {
            break;
        }

        if (entry->IsDirectRoute())
        {
            if (!SetRoute(entry->GetDestination(),
                          entry->GetPrefixSize(),
                          entry->GetGateway(),
                          entry->GetInterfaceIndex(),
                          entry->GetMetric()))
            {
                DMSG(0, "ProtoRouteMgr::SetAllRoutes() failed to set direct route to: %s\n",
                        entry->GetDestination().GetHostString());
                result = false;
            }
        }
    }
    // Second, set gateway routes
    iterator.Reset();
    while (true)
    {
        entry = iterator.GetNextEntry();
        if (entry == nullptr) {
            break;
        }
        if (entry->IsGatewayRoute())
        {
            if (!SetRoute(entry->GetDestination(),
                          entry->GetPrefixSize(),
                          entry->GetGateway(),
                          entry->GetInterfaceIndex(),
                          entry->GetMetric()))
            {
                DMSG(0, "ProtoRouteMgr::SetAllRoutes() failed to set gateway route to: %s\n",
                        entry->GetDestination().GetHostString());
                result = false;
            }
        }
    }

    return result;
}  // end ProtoRouteMgr::SetRoutes()

bool ProtoRouteMgr::DeleteRoutes(const ProtoRouteTable& routeTable)
{
    bool result = true;
    ProtoRouteTable::Iterator iterator(routeTable);
    const ProtoRouteTable::Entry* entry;
    // First, delete gateway routes
    while (true)
    {
        entry = iterator.GetNextEntry();
        if (entry == nullptr) {
            break;
        }

        if (entry->IsGatewayRoute())
        {
            if (!DeleteRoute(entry->GetDestination(),
                             entry->GetPrefixSize(),
                             entry->GetGateway(),
                             entry->GetInterfaceIndex()))
            {
                DMSG(0, "ProtoRouteMgr::DeleteAllRoutes() failed to delete gateway route to: %s\n",
                        entry->GetDestination().GetHostString());
                result = false;
            }
        }
    }
    // Second, delete direct (interface) routes
    iterator.Reset();
    while (true)
    {
        entry = iterator.GetNextEntry();
        if (entry == nullptr) {
            break;
        }

        if (entry->IsDirectRoute())
        {
            if (!DeleteRoute(entry->GetDestination(),
                             entry->GetPrefixSize(),
                             entry->GetGateway(),
                             entry->GetInterfaceIndex()))
            {
                DMSG(0, "ProtoRouteMgr::DeleteAllRoutes() failed to delete direct route to: %s\n",
                        entry->GetDestination().GetHostString());
                result = false;
            }
        }
    }
    // If there's a default entry delete it, too
    entry = routeTable.GetDefaultEntry();
    if (entry)
    {
        if (!DeleteRoute(entry->GetDestination(),
                         entry->GetPrefixSize(),
                         entry->GetGateway(),
                         entry->GetInterfaceIndex()))
        {
            DMSG(0, "ProtoRouteMgr::DeleteAllRoutes() failed to delete default route\n");
            result = false;
        }
    }
    return result;
}  // end ProtoRouteMgr::DeleteRoutes()



} //namespace// //ScenSim-Port://

