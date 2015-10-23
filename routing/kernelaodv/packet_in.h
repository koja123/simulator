/***************************************************************************
                          packet_in.h  -  description
                             -------------------
    begin                : Mon Jul 29 2002
    copyright            : (C) 2002 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/
#ifndef PACKET_IN_H
#define PACKET_IN_H

//ScenSim-Port://#include <linux/module.h>
//ScenSim-Port://#include <linux/kernel.h>
//ScenSim-Port://#include <linux/skbuff.h>
//ScenSim-Port://#include <linux/netdevice.h>
//ScenSim-Port://#include <linux/ip.h>
//ScenSim-Port://#include <linux/udp.h>
//ScenSim-Port://#include <linux/netfilter_ipv4.h>


#include "aodv.h"
#include "task_queue.h"
//#include "aodv_route.h"

namespace KernelAodvPort {

using ScenSim::KernelAodvProtocol;//ScenSim-Port://

unsigned int input_handler(
    //SenSim-Port://unsigned int hooknum,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    const unsigned char aodvControlPacketType,
    const unsigned int lastHopIp,
    const unsigned int destinationIp,
    const unsigned char ttl,
    struct sk_buff **skb
    //ScenSim-Port://,
    //ScenSim-Port://const struct net_device *in,
    //ScenSim-Port://const struct net_device *out,
    //ScenSim-Port://int (*okfn) (struct sk_buff *)
    );

}//namespace//

#endif
