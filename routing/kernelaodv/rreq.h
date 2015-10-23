/***************************************************************************
                          rreq.h  -  description
                             -------------------
    begin                : Mon Aug 4 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#ifndef RREQ_H
#define RREQ_H

//ScenSim-Port://#include <linux/module.h>
//ScenSim-Port://#include <linux/kernel.h>
//ScenSim-Port://#include <linux/ip.h>

#include "scensim_engine.h"
#include "aodv.h"
#include "aodv_route.h"
#include "timer_queue.h"
#include "flood_id.h"
#include "aodv_neigh.h"


namespace KernelAodvPort {

using std::shared_ptr;//ScenSim-Port://
using ScenSim::KernelAodvProtocol;//ScenSim-Port://
using ScenSim::SimulationEngineInterface;//ScenSim-Port://

int gen_rreq(
    const shared_ptr<ScenSim::SimulationEngineInterface>& simulationEngineInterfacePtr,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    AodvScheduleTimer* aodvScheduleTimerPtr,
    AodvFloodId* aodvFloodIdPtr,
    const unsigned int src_ip,
    const unsigned int dst_ip);

int resend_rreq(
    const shared_ptr<ScenSim::SimulationEngineInterface>& simulationEngineInterfacePtr,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    AodvScheduleTimer* aodvScheduleTimerPtr,
    AodvFloodId* aodvFloodIdPtr,
    task * tmp_packet);

int recv_rreq(
    const shared_ptr<ScenSim::SimulationEngineInterface>& simulationEngineInterfacePtr,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    AodvFloodId* aodvFloodIdPtr,
    AodvNeighbor* aodvNeighborPtr,
    task * tmp_packet);

}//namespace//

#endif
