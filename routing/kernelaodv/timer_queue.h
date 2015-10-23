/***************************************************************************
                          timer_queue.h  -  description
                             -------------------
    begin                : Mon Jul 14 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#ifndef TIMER_QUEUE_H
#define TIMER_QUEUE_H

//ScenSim-Port://#include <asm/div64.h>

#include "scensim_engine.h"

#include "aodv.h"
#include "task_queue.h"

namespace ScenSim { class KernelAodvProtocol; }


namespace KernelAodvPort {

using std::shared_ptr;//ScenSim-Port://
using ScenSim::KernelAodvProtocol;//ScenSim-Port://
using ScenSim::SimulationEngineInterface;//ScenSim-Port://


class AodvScheduleTimer
{
public:

    AodvScheduleTimer(
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        KernelAodvProtocol* initKernelAodvProtocolPtr)
        :
        simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
        kernelAodvProtocolPtr(initKernelAodvProtocolPtr)
    {
    }

    ~AodvScheduleTimer();


    bool find_timer(
        const unsigned int id,
        const unsigned char type);
    int insert_rreq_timer(
        rreq * tmp_rreq,
        const unsigned char retries);
    int insert_timer(
        const unsigned char task_type,
        const unsigned int delay,
        const unsigned int ip);
    void delete_timer(
        const unsigned char task_type,
        const unsigned int ip);

//ScenSim-Port://void timer_queue_signal();
//ScenSim-Port://void update_timer_queue();
//ScenSim-Port://    task *find_timer(const unsigned int& id,
//ScenSim-Port://                     const unsigned char& flags);
//ScenSim-Port://int read_timer_queue_proc(char *buffer, char **buffer_location, off_t offset, int buffer_length,int *eof,void *data);
//ScenSim-Port://int resend_rreq(task * tmp_packet);


private:
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    KernelAodvProtocol* kernelAodvProtocolPtr;

    int init_timer_queue();
};


}//namespace//
#endif
