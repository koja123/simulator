/***************************************************************************
                          rerr.h  -  description
                             -------------------
    begin                : Mon Aug 11 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/
#ifndef RERR_H
#define RERR_H

//ScenSim-Port://#include <linux/module.h>
//ScenSim-Port://#include <linux/kernel.h>
//ScenSim-Port://#include <linux/skbuff.h>
//ScenSim-Port://#include <linux/in.h>

#include "aodv.h"
#include "aodv_route.h"


namespace KernelAodvPort {


int gen_rerr(
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    const unsigned int brk_dst_ip);
int recv_rerr(
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    task * tmp_packet);


}//namespace//
#endif
