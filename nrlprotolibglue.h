// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef PROTOLIBGLUE_H
#define PROTOLIBGLUE_H

#include <assert.h>

#include "scensim_netsim.h"

#include "protoSimAgent.h"
#include "protoAddress.h"

namespace ScenSim {

using NrlProtolibPort::ProtoAddress;
using NrlProtolibPort::ProtoSimAgent;
using NrlProtolibPort::ProtoSocket;
using NrlProtolibPort::ProtoTimer;

class NrlolsrProtocol;


class ScenargieProtoSimAgent : public ProtoSimAgent {
public:
    virtual ~ScenargieProtoSimAgent() { }

    //Note: ProtoSimAgent's pure virtual function
    virtual bool OnStartup(int argc, const char*const* argv) = 0;
    virtual bool ProcessCommands(int argc, const char*const* argv) = 0;
    virtual void OnShutdown() = 0;

    virtual bool GetLocalAddress(ProtoAddress& localProtoAddress)
    {
        localProtoAddress.SimSetAddress(interfaceAddress.GetRawAddressLow32Bits());
        return true;
    }


    // Note: override ProtoTimerMgr::GetSystemTime()
    virtual void GetSystemTime(struct timeval& theTime);


    const shared_ptr<SimulationEngineInterface> GetSimulationEngineInterfacePtr() const
    {
        return simulationEngineInterfacePtr;
    }



protected:

    //Note: ProtoSimAgent's pure virtual function
    virtual ProtoSimAgent::SocketProxy* OpenSocket(ProtoSocket& theSocket) = 0;
    virtual void CloseSocket(ProtoSocket& theSocket) = 0;


    //Note: ProtoSimAgent's abstract class
    class ScenargieSocketProxy : public ProtoSimAgent::SocketProxy {

        //Note: for using ProtoSimAgent::SocketProxy::List
        friend class ScenargieProtoSimAgent;

        //Note: for accessing protected members of ProtoSimAgent::SocketProxy
        friend class NrlolsrProtocol;

    public:
        virtual ~ScenargieSocketProxy() { }

        virtual bool SendTo(
            const char* payloadBuffer,
            unsigned int& payloadSizeInBytes,
            const ProtoAddress& destinationProtoAddress) = 0;

        virtual void Receive(
            unique_ptr<Packet>& packetPtr,
            const ProtoAddress& sourceProtoAddress,
            const ProtoAddress& destinationProtoAddress) = 0;

        bool RecvFrom(
            char* payloadBuffer,
            unsigned int& payloadSizeInBytes,
            ProtoAddress& sourceProtoAddress);

        bool Bind(UINT16& thePort);
        bool JoinGroup(const ProtoAddress& groupProtoAddress) { return true; }
        bool LeaveGroup(const ProtoAddress& groupProtoAddress) { return true; }

        void SetTTL(unsigned char ttl) { sharedTtl = ttl; }
        void SetLoopback(bool loopback) { sharedLoopback = loopback; }

        //Note: dummy implementation; need to re-implement for TCP support
        bool Connect(const ProtoAddress& theAddress){ return false; }
        bool Accept(ProtoSocket* theSocket){ return false; }
        bool Listen(UINT16 thePort){ return false; }

        bool SetFlowLabel(UINT32 flowLabel)
        {
            sharedFlowLabel = htonl(flowLabel);
            return true;
        }


    protected:
        ScenargieSocketProxy();

        unsigned char sharedTtl;
        bool sharedLoopback;
        char* sharedDataBuffer;
        UINT16 sharedDataBufferLength;
        ProtoAddress sharedSourceProtoAddress;
        UINT16 sharedPort;

        UINT32 sharedFlowLabel;

        shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    };//ScenargieSocketProxy//


    // Note: override ProtoSimAgent::UpdateSystemTimer()
    bool UpdateSystemTimer(ProtoTimer::Command protoTimerCommand, double delay);

    ScenargieProtoSimAgent::ScenargieSocketProxy::List socketProxyList;



protected:
    ScenargieProtoSimAgent(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const unsigned int initInterfaceIndex);

    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    shared_ptr<NetworkLayer> networkLayerPtr;

    unsigned int interfaceIndex;
    NetworkAddress interfaceAddress;



private:
    void ScheduleWakeupTimer(const TimeType& wakeupTime);
    void ProcessWakeupTimerEvent();


    class WakeupTimerEvent : public SimulationEvent {
    public:
        WakeupTimerEvent(ScenargieProtoSimAgent* initScenargieProtoSimAgentPtr)
            :
            scenargieProtoSimAgentPtr(initScenargieProtoSimAgentPtr)
        { }

        void ExecuteEvent()
        {
            scenargieProtoSimAgentPtr->ProcessWakeupTimerEvent();
        }

    private:
        ScenargieProtoSimAgent* scenargieProtoSimAgentPtr;
    };//WakeupTimerEvent//


    shared_ptr<WakeupTimerEvent> wakeupTimerEventPtr;
    EventRescheduleTicket wakeupTimerEventTicket;
}; //ScenargieProtoSimAgent//




//-----------------------------------------------------------------------------
inline
ScenargieProtoSimAgent::ScenargieProtoSimAgent(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const unsigned int initInterfaceIndex)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    networkLayerPtr(initNetworkLayerPtr),
    interfaceIndex(initInterfaceIndex)
{
    interfaceAddress = networkLayerPtr->GetNetworkAddress(interfaceIndex);

    wakeupTimerEventPtr = shared_ptr<WakeupTimerEvent>(new WakeupTimerEvent(this));
}//ScenargieProtoSimAgent//




inline
void ScenargieProtoSimAgent::ScheduleWakeupTimer(const TimeType& wakeupTime)
{
    if (wakeupTimerEventTicket.IsNull()) {
        simulationEngineInterfacePtr->ScheduleEvent(
            wakeupTimerEventPtr, wakeupTime, wakeupTimerEventTicket);
    }
    else {
        simulationEngineInterfacePtr->RescheduleEvent(
            wakeupTimerEventTicket, wakeupTime);
    }//if//
}//ScheduleWakeupTime//



inline
void ScenargieProtoSimAgent::ProcessWakeupTimerEvent()
{
    assert(!wakeupTimerEventTicket.IsNull());
    wakeupTimerEventTicket.Clear();

    ProtoSimAgent::OnSystemTimeout();
}//ProcessWakeupTimerEvent//



inline
void ScenargieProtoSimAgent::GetSystemTime(struct timeval& theTime)
{
    const TimeType currentTime = simulationEngineInterfacePtr->CurrentTime();

    theTime.tv_sec = (unsigned int)(currentTime / SECOND);
    theTime.tv_usec = (unsigned int)((currentTime % SECOND) / MICRO_SECOND);
}//GetSystemTime//




inline
bool ScenargieProtoSimAgent::UpdateSystemTimer(
    ProtoTimer::Command protoTimerCommand,
    double delay)
{
    const TimeType wakeupTime =
        simulationEngineInterfacePtr->CurrentTime() + ConvertDoubleSecsToTime(delay);

    switch (protoTimerCommand)
    {
    case ProtoTimer::INSTALL:
    case ProtoTimer::MODIFY:
        ScheduleWakeupTimer(wakeupTime);
        break;
    case ProtoTimer::REMOVE:
        assert(!wakeupTimerEventTicket.IsNull());
        simulationEngineInterfacePtr->CancelEvent(wakeupTimerEventTicket);
        break;
    }//switch//

    return true;
}//UpdatesSystemTime//







inline
ScenargieProtoSimAgent::ScenargieSocketProxy::ScenargieSocketProxy()
    :
    sharedFlowLabel(NULL_FLOW_LABEL),
    sharedTtl(255),
    sharedLoopback(false),
    sharedDataBuffer(nullptr),
    sharedDataBufferLength(0)
{
}//ScenargieSocketProxy//






inline
bool ScenargieProtoSimAgent::ScenargieSocketProxy::RecvFrom(
    char* payloadBuffer,
    unsigned int& payloadSizeInBytes,
    ProtoAddress& sourceProtoAddress)
{
    if (sharedDataBuffer != nullptr){

        if (payloadSizeInBytes >= sharedDataBufferLength) {
            payloadSizeInBytes = sharedDataBufferLength;
            memcpy(payloadBuffer, sharedDataBuffer, payloadSizeInBytes);

            sourceProtoAddress = sharedSourceProtoAddress;
            sharedDataBuffer = nullptr;
            sharedDataBufferLength = 0;

            return true;
        }
        else {
            payloadSizeInBytes = 0;

            return false;
        }//if//

    }
    else {
        payloadSizeInBytes = 0;

        return false;
    }//if//
}//RecvFrom//




inline
bool ScenargieProtoSimAgent::ScenargieSocketProxy::Bind(
    UINT16& thePort)
{
    //Note: proto_socket is in ProtoSimAgent
    assert(proto_socket && proto_socket->IsOpen());

    sharedPort = thePort;
    return true;
}//Bind//



}//namespace//


#endif
