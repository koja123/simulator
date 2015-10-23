// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "aloha_phy.h"
#include "aloha_mac.h"


namespace Aloha {

const string AlohaPhy::modelName = "AlohaPhy";

void AlohaPhy::ProcessEndOfTheSignalCurrentlyBeingReceived(const IncomingSignal& aSignal)
{
    assert(phyRxState == RECEIVING_STATE);

    (*this).OutputTraceAndStatsForRxEnd(aSignal);

    // Clear variables as required.
    (*this).currentlyReceivingFrameInfo.Clear();

    // Assuming no action by ALOHA for packets with error.
    if (!currentPacketHasAnError) {
        macLayerPtr->ReceiveFrameFromPhy(*aSignal.GetFrame().macFramePtr);
    }//if//

    if (!IsReceivingInterferingSignals()) {
        phyRxState = SCANNING_STATE;
    }//if//

}//ProcessEndOfTheSignalCurrentlyBeingReceived//



void AlohaPhy::TransmitAFrame(
    unique_ptr<Packet>& packetPtr,
    const DatarateBitsPerSecType& datarateBitsPerSecond,
    const double& transmissionPowerDbm,
    const TimeType& frameDuration)
{
    assert((phyTxState != TX_STARTING_STATE) && (phyTxState != TRANSMITTING_STATE));

    phyTxState = TX_STARTING_STATE;

    if (phyRxState == RECEIVING_STATE) {
        (*this).currentPacketHasAnError = true;
    }//if//

    (*this).currentPropagatedFramePtr = propagatedFramePtrs.front();
    propagatedFramePtrs.pop();
    propagatedFramePtrs.push(currentPropagatedFramePtr);

    (*this).currentPropagatedFramePtr->datarateBitsPerSecond = datarateBitsPerSecond;

    (*this).currentPropagatedFramePtr->macFramePtr = move(packetPtr);

    (*this).outgoingTransmissionPowerDbm = transmissionPowerDbm;
    (*this).outgoingFrameDuration = frameDuration;

    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    simEngineInterfacePtr->ScheduleEvent(
        transmissionTimerEventPtr,
        currentTime + delayUntilAirborne);

}//TransmitAFrame//


void AlohaPhy::EndCurrentTransmission()
{
    assert(phyTxState == TRANSMITTING_STATE);

    (*this).OutputTraceForTxEnd();

    phyTxState = IDLE_STATE;

    macLayerPtr->TransmissionIsCompleteNotification();

}//EndCurrentTransmission//


}//namespace//
