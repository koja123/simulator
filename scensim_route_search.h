// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_ROUTE_SEARCH_H
#define SCENSIM_ROUTE_SEARCH_H

#include "scensim_gis.h"

#include <vector>
#include <list>
#include <queue>


namespace ScenSim {

using std::find;
using std::vector;
using std::priority_queue;
using std::cout;
using std::cerr;
using std::endl;

static const double INVALID_PATH_COST = -DBL_MAX;
static const double DEFAULT_ROUTE_SEARCH_RAILROAD_CONNECTION_COST = 200;



class RouteSearchSubsystem {
public:
    RouteSearchSubsystem(
        const ParameterDatabaseReader& theParameterDatabaseReader)
    {}

    virtual ~RouteSearchSubsystem() {}

    virtual void RouteSearch(
        const GisSubsystem& gisSubsystem,
        const IntersectionIdType& sourceIntersectionId,
        const IntersectionIdType& destinationIntersectionId,
        const list<IntersectionIdType>& targetIntersectionIds,
        list<IntersectionIdType>& resultIntersectionIds,
        bool& success) = 0;

};//GisRouteSearchSubsystem//


//-------------------------------------------------------------------
// Dijkstra Algorithm
//-------------------------------------------------------------------
class DijkstraAlgorithm : public RouteSearchSubsystem {
public:
    DijkstraAlgorithm(
        const ParameterDatabaseReader& theParameterDatabaseReader)
        :
        RouteSearchSubsystem(theParameterDatabaseReader)
    {}

    virtual void RouteSearch(
        const GisSubsystem& gisSubsystem,
        const IntersectionIdType& sourceIntersectionId,
        const IntersectionIdType& destinationIntersectionId,
        const list<IntersectionIdType>& targetIntersectionIds,
        list<IntersectionIdType>& resultIntersectionIds,
        bool& success);

private:
    struct RouteSearchNodeType {
        IntersectionIdType intersectionId;
        double pathCost;

        bool isClosed;
        IntersectionIdType backIntersectionId;

        RouteSearchNodeType()
            :
            intersectionId(INVALID_VARIANT_ID),
            pathCost(INVALID_PATH_COST),
            isClosed(false),
            backIntersectionId(INVALID_VARIANT_ID)
        {}

        RouteSearchNodeType(
            const IntersectionIdType& initIntersectionId,
            const double& initPathCost)
            :
            intersectionId(initIntersectionId),
            pathCost(initPathCost),
            isClosed(false),
            backIntersectionId(INVALID_VARIANT_ID)
        {}

        bool operator>(const RouteSearchNodeType& right) const {
            return ((*this).pathCost > right.pathCost);
        }
        bool operator==(const IntersectionIdType& right) const {
            return ((*this).intersectionId == right);
        }
    };

};//DijkstraAlgorithm//



inline
void DijkstraAlgorithm::RouteSearch(
    const GisSubsystem& gisSubsystem,
    const IntersectionIdType& sourceIntersectionId,
    const IntersectionIdType& destinationIntersectionId,
    const list<IntersectionIdType>& targetIntersectionIds,
    list<IntersectionIdType>& resultIntersectionIds,
    bool& success)
{
    success = false;
    resultIntersectionIds.clear();

    // --- initialize ---
    vector<RouteSearchNodeType> targetNodes;
    targetNodes.clear();
    targetNodes.reserve(targetIntersectionIds.size());
    
    for(list<IntersectionIdType>::const_iterator iter = targetIntersectionIds.begin();
        iter != targetIntersectionIds.end(); iter++) {
        targetNodes.push_back(RouteSearchNodeType(*iter, INVALID_PATH_COST));
    }//for//

    priority_queue<RouteSearchNodeType, vector<RouteSearchNodeType>, std::greater<RouteSearchNodeType> > openQueue;
    
    typedef vector<RouteSearchNodeType>::iterator IntersectionIter;

    // --- path cost update ---
    IntersectionIter sourceNodeIter = find(targetNodes.begin(), targetNodes.end(), sourceIntersectionId);
    assert(find(targetIntersectionIds.begin(), targetIntersectionIds.end(), sourceIntersectionId) != targetIntersectionIds.end());
    assert(sourceNodeIter != targetNodes.end());

    (*sourceNodeIter).pathCost = 0;
    openQueue.push((*sourceNodeIter));

    while(!openQueue.empty() ) {
        RouteSearchNodeType aNode = openQueue.top();
        openQueue.pop();

        aNode.isClosed = true;

        const IntersectionIdType& startIntersectionId = aNode.intersectionId;
        const vector<RoadIdType> roadIds = gisSubsystem.GetIntersection(startIntersectionId).GetConnectedRoadIds();

        for (size_t i = 0; i < roadIds.size(); i++) {
            const Road& road = gisSubsystem.GetRoad(roadIds[i]);

            IntersectionIter targetNodeIter =
                find(targetNodes.begin(), targetNodes.end(), road.GetOtherSideIntersectionId(startIntersectionId));

            if (targetNodeIter == targetNodes.end()) {
                continue;
            }

            const double candidatePathCost = aNode.pathCost + road.GetArcDistanceMeters();

            if ((*targetNodeIter).pathCost < 0 || candidatePathCost < (*targetNodeIter).pathCost) {

                (*targetNodeIter).pathCost = candidatePathCost;
                (*targetNodeIter).backIntersectionId = aNode.intersectionId;

                if (!(*targetNodeIter).isClosed) {
                    openQueue.push((*targetNodeIter));
                }
            }

        }//for//
    }//while//


    // --- route update ---
    IntersectionIter routeNodeIter = find(targetNodes.begin(), targetNodes.end(), destinationIntersectionId);
    assert(routeNodeIter != targetNodes.end());

    do {
        resultIntersectionIds.push_front(routeNodeIter->intersectionId);

        routeNodeIter = find(targetNodes.begin(), targetNodes.end(), routeNodeIter->backIntersectionId);
    } while(routeNodeIter != targetNodes.end());
    
    success = (resultIntersectionIds.front() == sourceIntersectionId);

}//RouteSearch//



//-------------------------------------------------------------------
// A* Algorithm
//-------------------------------------------------------------------
class AStarAlgorithm : public RouteSearchSubsystem {
public:
    AStarAlgorithm(
        const ParameterDatabaseReader& theParameterDatabaseReader)
        :
        RouteSearchSubsystem(theParameterDatabaseReader)
    {}

    virtual void RouteSearch(
        const GisSubsystem& gisSubsystem,
        const IntersectionIdType& sourceIntersectionId,
        const IntersectionIdType& destinationIntersectionId,
        const list<IntersectionIdType>& targetIntersectionIds,
        list<IntersectionIdType>& resultIntersectionIds,
        bool& success);

private:
    struct RouteSearchNodeType {
        IntersectionIdType intersectionId;
        double pathCost;
        double expcetedMinPathCostToDest;

        bool isClosed;
        IntersectionIdType backIntersectionId;

        RouteSearchNodeType()
            :
            intersectionId(INVALID_VARIANT_ID),
            pathCost(INVALID_PATH_COST),
            expcetedMinPathCostToDest(INVALID_PATH_COST),
            isClosed(false),
            backIntersectionId(INVALID_VARIANT_ID)
        {}

        RouteSearchNodeType(
            const IntersectionIdType& initIntersectionId,
            const double& initPathCost,
            const double& initExpcetedMinPathCostToDest)
            :
            intersectionId(initIntersectionId),
            pathCost(initPathCost),
            expcetedMinPathCostToDest(initExpcetedMinPathCostToDest),
            isClosed(false),
            backIntersectionId(INVALID_VARIANT_ID)
        {}

        bool operator<(const RouteSearchNodeType& right) const {
            return ((*this).expcetedMinPathCostToDest > right.expcetedMinPathCostToDest);
        }
        bool operator==(const IntersectionIdType& right) const {
            return ((*this).intersectionId == right);
        }
    };

};//DijkstraAlgorithm//



inline
void AStarAlgorithm::RouteSearch(
    const GisSubsystem& gisSubsystem,
    const IntersectionIdType& sourceIntersectionId,
    const IntersectionIdType& destinationIntersectionId,
    const list<IntersectionIdType>& targetIntersectionIds,
    list<IntersectionIdType>& resultIntersectionIds,
    bool& success)
{
    success = false;
    resultIntersectionIds.clear();

    const Vertex& srcVertex = gisSubsystem.GetIntersection(sourceIntersectionId).GetVertex();
    const Vertex& destVertex = gisSubsystem.GetIntersection(destinationIntersectionId).GetVertex();

    // --- initialize ---
    vector<RouteSearchNodeType> targetNodes;
    targetNodes.clear();
    targetNodes.reserve(targetIntersectionIds.size());
    
    for(list<IntersectionIdType>::const_iterator iter = targetIntersectionIds.begin();
        iter != targetIntersectionIds.end(); iter++) {
        targetNodes.push_back(RouteSearchNodeType(*iter, INVALID_PATH_COST, INVALID_PATH_COST));
    }//for//

    priority_queue<RouteSearchNodeType> openQueue;
    
    typedef vector<RouteSearchNodeType>::iterator IntersectionIter;

    // --- path cost update ---
    IntersectionIter sourceNodeIter = find(targetNodes.begin(), targetNodes.end(), sourceIntersectionId);
    assert(find(targetIntersectionIds.begin(), targetIntersectionIds.end(), sourceIntersectionId) != targetIntersectionIds.end());
    assert(sourceNodeIter != targetNodes.end());

    (*sourceNodeIter).pathCost = 0;
    (*sourceNodeIter).expcetedMinPathCostToDest = srcVertex.DistanceTo(destVertex);

    openQueue.push((*sourceNodeIter));

    while(!openQueue.empty() ) {
        RouteSearchNodeType aNode = openQueue.top();
        openQueue.pop();

        aNode.isClosed = true;

        const IntersectionIdType& startIntersectionId = aNode.intersectionId;
        const vector<RoadIdType> roadIds = gisSubsystem.GetIntersection(startIntersectionId).GetConnectedRoadIds();

        for (size_t i = 0; i < roadIds.size(); i++) {
            const Road& road = gisSubsystem.GetRoad(roadIds[i]);

            IntersectionIter targetNodeIter =
                find(targetNodes.begin(), targetNodes.end(), road.GetOtherSideIntersectionId(startIntersectionId));

            if (targetNodeIter == targetNodes.end()) {
                continue;
            }

            const double candidatePathCost = aNode.pathCost + road.GetArcDistanceMeters();
            RouteSearchNodeType& targetNode = (*targetNodeIter);

            if (targetNode.expcetedMinPathCostToDest == INVALID_PATH_COST) {
                const Vertex& linkVertex = gisSubsystem.GetIntersection(targetNode.intersectionId).GetVertex();

                targetNode.expcetedMinPathCostToDest = linkVertex.DistanceTo(destVertex);
            }

            if (targetNode.pathCost < 0 || candidatePathCost < targetNode.pathCost) {

                targetNode.pathCost = candidatePathCost;
                targetNode.backIntersectionId = aNode.intersectionId;

                if (!targetNode.isClosed) {
                    openQueue.push(targetNode);
                }
            }

        }//for//
    }//while//


    // --- route update ---
    IntersectionIter routeNodeIter = find(targetNodes.begin(), targetNodes.end(), destinationIntersectionId);
    assert(routeNodeIter != targetNodes.end());

    do {
        resultIntersectionIds.push_front(routeNodeIter->intersectionId);

        routeNodeIter = find(targetNodes.begin(), targetNodes.end(), routeNodeIter->backIntersectionId);
    } while(routeNodeIter != targetNodes.end());
    
    success = (resultIntersectionIds.front() == sourceIntersectionId);

}//RouteSearch//


}//namespace//

#endif

