/***************************************************************************
                          hello.h  -  description
                             -------------------
    begin                : Wed Aug 13 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/
#ifndef HELLO_H
#define HELLO_H

#include "scensim_engine.h"
#include "aodv.h"
#include "timer_queue.h"
#include "aodv_neigh.h"

namespace ScenSim {
    class KernelAodvProtocol;
}

namespace KernelAodvPort {

using std::shared_ptr;//ScenSim-Port://
using ScenSim::KernelAodvProtocol;//ScenSim-Port://
using ScenSim::SimulationEngineInterface;//ScenSim-Port://

class AodvHello {
public:
    AodvHello(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        KernelAodvProtocol* initKernelAodvProtocolPtr,
        AodvRoutingTable* initAodvRoutingTablePtr,
        AodvScheduleTimer* initAodvScheduleTimerPtr)
        :
        simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
        kernelAodvProtocolPtr(initKernelAodvProtocolPtr),
        aodvRoutingTablePtr(initAodvRoutingTablePtr),
        aodvScheduleTimerPtr(initAodvScheduleTimerPtr)
    { }
    ~AodvHello();

int send_hello();
int recv_hello(
    AodvNeighbor* aodvNeighborPtr,
    rrep * tmp_rrep,
    task * tmp_packet);

private:
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    KernelAodvProtocol* kernelAodvProtocolPtr;
    AodvRoutingTable* aodvRoutingTablePtr;
    AodvScheduleTimer* aodvScheduleTimerPtr;
};

}//namespace//

#endif
