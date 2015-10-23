/***************************************************************************
                          aodv_route.h  -  description
                             -------------------
    begin                : Mon Jul 29 2002
    copyright            : (C) 2002 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/
#ifndef AODV_ROUTE_H
#define AODV_ROUTE_H

//ScenSim-Port://#include <linux/module.h>
//ScenSim-Port://#include <linux/kernel.h>
//ScenSim-Port://#include <linux/netdevice.h>

#include "scensim_engine.h"
#include "scensim_netsim.h"
#include "aodv.h"
#include "fakeout_aodv.h"

//ScenSim-Port://#include "utils.h"
//ScenSim-Port://#include "packet_queue.h"
//ScenSim-Port://#include "kernel_route.h"


namespace ScenSim {
    class KernelAodvProtocol;
}


namespace KernelAodvPort {

using std::shared_ptr;//ScenSim-Port://
using ScenSim::KernelAodvProtocol;//ScenSim-Port://
using ScenSim::SimulationEngineInterface;//ScenSim-Port://


class AodvNeighbor;

class AodvRoutingTable {
public:
    AodvRoutingTable(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        KernelAodvProtocol* initKernelAodvProtocolPtr,
        AodvNeighbor* initAodvNeighborPtr,
        const unsigned int initInterfaceIp)
        :
        simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
        kernelAodvProtocolPtr(initKernelAodvProtocolPtr),
        aodvNeighborPtr(initAodvNeighborPtr)
    {
        (*this).init_aodv_route_table(initInterfaceIp);
    }

    ~AodvRoutingTable();


    void expire_aodv_route(aodv_route * tmp_route);
    int update_aodv_route(
        const unsigned int ip,
        const unsigned int next_hop_ip,
        const unsigned char metric,
        const unsigned int seq,
        struct net_device *dev);
    int flush_aodv_route_table();
    aodv_route *find_aodv_route(const unsigned int target_ip);
    aodv_route *create_aodv_route(const unsigned int ip);
    aodv_route *first_aodv_route();
    int find_metric(const unsigned int tmp_ip);

    int compare_aodv_route(aodv_route * tmp_route,
                           const unsigned int target_ip);
    void insert_aodv_route(aodv_route* new_route);
    void remove_aodv_route(aodv_route * dead_route);

//ScenSim-Port://int read_route_table_proc(char *buffer, char **buffer_location, off_t offset, int buffer_length,int *eof,void *data);
//ScenSim-Port://int cleanup_aodv_route_table();
//ScenSim-Port://int delete_aodv_route(u_int32_t target_ip);


private:
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    KernelAodvProtocol* kernelAodvProtocolPtr;
    AodvNeighbor* aodvNeighborPtr;
    aodv_route* aodv_route_table;

    void init_aodv_route_table(const unsigned int initInterfaceIp);
};

}//namespace AODV//

#endif
