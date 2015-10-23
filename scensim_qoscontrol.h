// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_QOSCONTROL_H
#define SCENSIM_QOSCONTROL_H


#include <memory>

#include "scensim_support.h"
#include "scensim_netaddress.h"
#include "scensim_packet.h"


namespace ScenSim {

using std::shared_ptr;

const double NO_MAXIMUM_BANDWIDTH_VALUE = DBL_MAX;

class FlowIdType {
public:
    static const FlowIdType nullFlowId;
    FlowIdType() : clientNodeId(INVALID_NODEID), clientFlowId(0) {}
    FlowIdType(const NodeIdType& initClientNodeId, const unsigned int initClientFlowId)
        : clientNodeId(initClientNodeId), clientFlowId(initClientFlowId) {}

    bool operator==(const FlowIdType& right) const;
    bool operator!=(const FlowIdType& right) const { return (!((*this) == right)); }

    bool operator<(const FlowIdType& right) const;

    NodeIdType GetClientNodeId() const { return clientNodeId; }
    unsigned int GetClientFlowId() const { return clientFlowId; }

private:
    NodeIdType clientNodeId;
    unsigned int clientFlowId;

};//FlowIdType//

inline
bool FlowIdType::operator==(const FlowIdType& right) const
{
    return ((clientNodeId == right.clientNodeId) &&
            (clientFlowId == right.clientFlowId));
}//<//


inline
bool FlowIdType::operator<(const FlowIdType& right) const
{
    if (clientNodeId < right.clientNodeId) {
        return true;
    }//if//

    if (clientNodeId > right.clientNodeId) {
        return false;
    }//if//

    return (clientFlowId < right.clientFlowId);
}//<//


// LTE "QoS Class Identifier" (QCI), LTE Priority, IP ToS Priority Map.


//    IP ToS   LTE Priority    QCI    GBR = Guaranteed Bit Rate
//-------------------------------------------------------------
//      7        1             5
//      6        2             1      GBR
//      5        3             4      GBR
//      4        4             2      GBR
//      3        5             3      GBR
//      2        6             7
//      1        7             6
//      0        8             8
//     No        9             9
//
//     QCI    LTE Priority    IP ToS
//----------------------------------
//      1         2            6      GBR
//      2         4            4      GBR
//      3         5            3      GBR
//      4         3            5      GBR
//      5         1            7
//      6         7            1
//      7         6            2
//      8         8            0
//      9         9           No


//
// Note: This interface will likely get very large with routines for each QoS flow type.
//       Will try to "abstact away" exact LTE vs Dot16/Wimax scheduling disciplines, i.e.
//       generic QoS will be mapped to specific MAC technology QoS parameters.


class MacQualityOfServiceControlInterface {
public:
    // LTE Definition: Minimum Guaranteed Bit Rate (GBR)

    enum SchedulingSchemeChoice {
        StaticReservation,                     // LTE: Persistent GBR, 802.16: "UGS"
        RealTimePollingService,                // 802.16 Only, rtPS
        ReservedBandwidthWithCdmaInitiatedScheduling, // LTE: Dynamic GBR, 802.16: nrtPS
        BestEffort,                            // LTE: Non-GBR ("Cdma Initiated" but no reserved bandwidth)
        BasedOnPacketPriority,                 // LTE model picks which scheme internally.
        DefaultSchedulingScheme
    };

    enum ReservationSchemeChoice {
        OptimisticLinkRate,
        ConservativeLinkRate
    };

    // With ReservedBandwidthWithCdmaInitiatedScheduling, if a mobile
    // does not have any bandwidth to send a bandwidth request ("Buffer Status Report" (BSR)),
    // then it will send a very low # bits CDMA request on a special "control channel"
    // to request a small amount of bandwidth to send a full bandwidth request (BSR) on
    // the normal channel.

    class FlowRequestReplyFielder {
    public:
        virtual void RequestAccepted(const FlowIdType& flowId) = 0;
        virtual void RequestDenied() = 0;
    };//FlowRequestReply//

    // Downlink means to this node.

    virtual void RequestDownlinkFlowReservation(
        const ReservationSchemeChoice& reservationScheme,
        const SchedulingSchemeChoice& schedulingScheme,
        const PacketPriorityType& priority,
        const double& baselineBytesPerSec,
        const double& maxBytesPerSec,
        const NetworkAddress& basestationSideAddress,
        const unsigned short basestationSidePort,
        const NetworkAddress& mobileSideAddress,
        const unsigned short mobileSidePort,
        const unsigned char protocolCode,
        const shared_ptr<FlowRequestReplyFielder>& replyPtr) = 0;

    // Uplink means from this node.

    virtual void RequestUplinkFlowReservation(
        const ReservationSchemeChoice& reservationScheme,
        const SchedulingSchemeChoice& schedulingScheme,
        const PacketPriorityType& priority,
        const double& baselineBytesPerSec,
        const double& maxBytesPerSec,
        const NetworkAddress& basestationSideAddress,
        const unsigned short basestationSidePort,
        const NetworkAddress& mobileSideAddress,
        const unsigned short mobileSidePort,
        const unsigned char protocolCode,
        const shared_ptr<FlowRequestReplyFielder>& replyPtr) = 0;

    virtual void RequestDualFlowReservation(
        const ReservationSchemeChoice& reservationScheme,
        const SchedulingSchemeChoice& schedulingScheme,
        const PacketPriorityType& priority,
        const double& downlinkBaselineBytesPerSec,
        const double& downlinkMaxBytesPerSec,
        const double& uplinkBaselineBytesPerSec,
        const double& uplinkMaxBytesPerSec,
        const NetworkAddress& basestationSideAddress,
        const unsigned short basestationSidePort,
        const NetworkAddress& mobileSideAddress,
        const unsigned short mobileSidePort,
        const unsigned char protocolCode,
        const shared_ptr<FlowRequestReplyFielder>& replyPtr) = 0;

    virtual void DeleteFlow(const FlowIdType& flowId) = 0;

};//MacQualityOfServiceControlInterface//


inline
void ConvertStringToSchedulingSchemeChoice(
    const string& aString,
    bool& succeeded,
    MacQualityOfServiceControlInterface::SchedulingSchemeChoice& schedulingScheme)
{
    assert(StringIsAllLowerCase(aString));

    succeeded = true;
    if (aString == "ugs") {
        schedulingScheme = MacQualityOfServiceControlInterface::StaticReservation;
    }
    else if (aString == "rtps") {
        schedulingScheme = MacQualityOfServiceControlInterface::RealTimePollingService;
    }
    else if (aString == "nrtps") {
        schedulingScheme =
            MacQualityOfServiceControlInterface::ReservedBandwidthWithCdmaInitiatedScheduling;
    }
    else if (aString == "be") {
        schedulingScheme = MacQualityOfServiceControlInterface::BestEffort;
    }
    else if ((aString == "lte") || (aString == "pribased")) {
        schedulingScheme = MacQualityOfServiceControlInterface::BasedOnPacketPriority;
    }
    else {
        succeeded = false;
    }//if//

}//ConvertStringToSchedulingScheme//



}//namespace//

#endif
