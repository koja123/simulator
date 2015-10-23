/***************************************************************************
                          packet_out.h  -  description
                             -------------------
    begin                : Mon Aug 4 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/
#ifndef PACKET_OUT_H
#define PACKET_OUT_H

//ScenSim-Port://#include <linux/module.h>
//ScenSim-Port://#include <linux/kernel.h>
//ScenSim-Port://#include <linux/netfilter.h>
//ScenSim-Port://#include <linux/skbuff.h>
//ScenSim-Port://#include <linux/udp.h>
//ScenSim-Port://#include <linux/ip.h>
//ScenSim-Port://#include <net/sock.h>

#include "scensim_engine.h"

#include "aodv.h"
#include "fakeout_aodv.h"
#include "rreq.h"

namespace KernelAodvPort {


AodvAction output_handler(
    //ScenSim-Port://unsigned int hooknum,
    const shared_ptr<ScenSim::SimulationEngineInterface>& simulationEngineInterfacePtr,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    AodvScheduleTimer* aodvScheduleTimerPtr,
    AodvFloodId* aodvFloodIdPtr,
    //ScenSim-Port://struct sk_buff **skb,
    const unsigned int sourceIpAddress,
    const unsigned int destinationIpAddress
    //ScenSim-Port://,
    //ScenSim-Port://const struct net_device *in,
    //ScenSim-Port://const struct net_device *out,
    //ScenSim-Port://int (*okfn) (struct sk_buff *)
    );

}//namespace//

#endif
