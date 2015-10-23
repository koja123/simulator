/***************************************************************************
                          aodv_neigh.h  -  description
                             -------------------
    begin                : Thu Jul 31 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#ifndef AODV_NEIGH_H
#define AODV_NEIGH_H

//ScenSim-Port://#include <linux/module.h>
//ScenSim-Port://#include <linux/kernel.h>
//ScenSim-Port://#include <linux/types.h>
//ScenSim-Port://#include <linux/if.h>
//ScenSim-Port://#include <linux/netdevice.h>

//ScenSim-Port://#include "aodv.h"
//ScenSim-Port://#include "aodv_route.h"
//ScenSim-Port://#include "kernel_route.h"

#include "scensim_engine.h"
#include "aodv.h"
#include "timer_queue.h"
#include "rerr.h"

namespace KernelAodvPort {


using std::shared_ptr;//ScenSim-Port://
using ScenSim::KernelAodvProtocol;//ScenSim-Port://
using ScenSim::SimulationEngineInterface;//ScenSim-Port://


class AodvNeighbor {
public:
    AodvNeighbor(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        KernelAodvProtocol* initKernelAodvProtocolPtr,
        AodvScheduleTimer* initAodvScheduleTimerPtr)
        :
        simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
        kernelAodvProtocolPtr(initKernelAodvProtocolPtr),
        aodvScheduleTimerPtr(initAodvScheduleTimerPtr)
    {
        (*this).init_aodv_neigh_list();
    }

    ~AodvNeighbor();

    int valid_aodv_neigh(const unsigned int target_ip);
    aodv_neigh *find_aodv_neigh(const unsigned int target_ip);
    int update_aodv_neigh(
        AodvRoutingTable* aodvRoutingTablePtr,
        aodv_neigh *tmp_neigh,
        rrep *tmp_rrep);
    aodv_neigh *create_aodv_neigh(const unsigned int ip);
    int delete_aodv_neigh(
        AodvRoutingTable* aodvRoutingTablePtr,
        const unsigned int ip);

//ScenSim-Port://void update_aodv_neigh_packets(u_int32_t target_ip);
//ScenSim-Port://void update_aodv_neigh_packets_dropped(char *hw_addr);
//ScenSim-Port://void update_aodv_neigh_link(char *hw_addr, u_int8_t link);
//ScenSim-Port://int read_neigh_proc(char *buffer, char **buffer_location, off_t offset, int buffer_length,int *eof,void *data);
//ScenSim-Port://int flush_aodv_neigh();
//ScenSim-Port://aodv_neigh *first_aodv_neigh();
//ScenSim-Port://rrep * create_valid_neigh_hello();
//ScenSim-Port://int aodv_neigh_lost_confirmation(aodv_neigh *tmp_neigh);
//ScenSim-Port://int aodv_neigh_got_confirmation(aodv_neigh *tmp_neigh);


private:
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    KernelAodvProtocol* kernelAodvProtocolPtr;
    AodvScheduleTimer* aodvScheduleTimerPtr;
    aodv_neigh* aodv_neigh_list;

    void init_aodv_neigh_list();
    void update_aodv_neigh_route(
        AodvRoutingTable* aodvRoutingTablePtr,
        aodv_neigh * tmp_neigh,
        rrep *tmp_rrep);
};

}//namespace//


#endif
