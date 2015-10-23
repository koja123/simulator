/***************************************************************************
                          flood_id.h  -  description
                             -------------------
    begin                : Mon Aug 4 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/
#ifndef FLOOD_ID_QUEUE_H
#define FLOOD_ID_QUEUE_H

//ScenSim-Port://#include <linux/module.h>
//ScenSim-Port://#include <linux/kernel.h>
//ScenSim-Port://#include <linux/skbuff.h>
//ScenSim-Port://#include <linux/in.h>
//ScenSim-Port://#include <asm/div64.h>

#include "scensim_engine.h"
#include "aodv.h"
#include "timer_queue.h"

namespace ScenSim {
    class KernelAodvProtocol;
}

namespace KernelAodvPort {

using std::shared_ptr;//ScenSim-Port://
using ScenSim::KernelAodvProtocol;//ScenSim-Port://
using ScenSim::SimulationEngineInterface;//ScenSim-Port://


class AodvFloodId
{
public:

    AodvFloodId(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        KernelAodvProtocol* initKernelAodvProtocolPtr,
        AodvScheduleTimer* initAodvScheduleTimerPtr)
        :
        simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
        kernelAodvProtocolPtr(initKernelAodvProtocolPtr),
        aodvScheduleTimerPtr(initAodvScheduleTimerPtr)
    {
        (*this).init_flood_id_queue();
    }

    ~AodvFloodId();

    //ScenSim-Port://int read_flood_id_proc(char *buffer, char **buffer_location, off_t offset, int buffer_length,int *eof,void *data);
    int insert_flood_id(
        const unsigned int src_ip,
        const unsigned int dst_ip,
        const unsigned int id,
        const long long int& lt);

    flood_id* find_flood_id(
        const unsigned int src_ip,
        const unsigned int id);

    int flush_flood_id_queue();


private:
   shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
   KernelAodvProtocol* kernelAodvProtocolPtr;
   AodvScheduleTimer* aodvScheduleTimerPtr;

   flood_id* flood_id_queue;

   void init_flood_id_queue();
};

}//namespace//

#endif
