/***************************************************************************
                          rrep.h  -  description
                             -------------------
    begin                : Wed Aug 6 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#ifndef RREP_H
#define RREP_H

//ScenSim-Port://#include <linux/module.h>
//ScenSim-Port://#include <linux/kernel.h>
//ScenSim-Port://#include <linux/skbuff.h>
//ScenSim-Port://#include <linux/in.h>
#include "scensim_engine.h"
#include "aodv.h"
#include "aodv_route.h"
#include "fakeout_aodv.h"
#include "timer_queue.h"
#include "hello.h"

namespace KernelAodvPort {


int recv_rrep(
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    AodvScheduleTimer* aodvScheduleTimerPtr,
    AodvNeighbor* aodvNeighborPtr,
    AodvHello* aodvHelloPtr,
    task * tmp_packet);
int gen_rrep(
    const shared_ptr<ScenSim::SimulationEngineInterface>& simulationEngineInterfacePtr,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    const unsigned int src_ip,
    const unsigned int dst_ip,
    rreq *tmp_rreq);

int check_rrep(
    AodvScheduleTimer* aodvScheduleTimerPtr,
    AodvNeighbor* aodvNeighborPtr,
    AodvHello* aodvHelloPtr,
    rrep * tmp_rrep,
    task * tmp_packet);
}//namespace//
#endif
