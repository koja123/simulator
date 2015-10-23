/***************************************************************************
                          task_queue.h  -  description
                             -------------------
    begin                : Tue Jul 8 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#ifndef TASK_QUEUE_H
#define TASK_QUEUE_H

//ScenSim-Port://#include <linux/module.h>
//ScenSim-Port://#include <linux/kernel.h>
//ScenSim-Port://#include <linux/ip.h>
//ScenSim-Port://#include <linux/udp.h>
#include <iostream>
#include "aodv.h"
#include "fakeout_aodv.h"

namespace ScenSim {
    class KernelAodvProtocol;
}

namespace KernelAodvPort {

using ScenSim::KernelAodvProtocol;//ScenSim-Port://

//ScenSim-Port://task *get_task();
task *create_task(const int type);
int insert_task(
    KernelAodvProtocol* kernelAodvProtocolPtr,
    const unsigned int lastHopIp,
    const unsigned int destinationIp,
    const int type,
    const unsigned char ttl,
    struct sk_buff *packet);
//ScenSim-Port://int insert_task_from_timer(task * timer_task);
//ScenSim-Port://void init_task_queue();
//ScenSim-Port://void cleanup_task_queue();


}//namespace//

#endif
