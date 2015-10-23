//
// Scheduler
//
#ifndef NU_CORE_SCHEDULER_H_
#define NU_CORE_SCHEDULER_H_

#include "core/core.h"
#include "core/mem.h"
#include "core/event_list.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_scheduler Core :: Scheduler
 * @{
 */

/**
 * Type of callback function which called at start of the main loop.
 */
typedef void (*nu_scheduler_pre_func_t)(void);

/**
 * Type of callback function which called at end of the main loop.
 */
typedef void (*nu_scheduler_post_func_t)(void);

/*
 * Scheduler interface
 */
PUBLIC void nu_scheduler_init(void);
PUBLIC void nu_scheduler_shutdown(void);
PUBLIC void nu_scheduler_start(void);

PUBLIC void nu_scheduler_set_pollrate(double);
PUBLIC void nu_scheduler_set_pre_func(nu_scheduler_pre_func_t);
PUBLIC void nu_scheduler_set_post_func(nu_scheduler_post_func_t);

PUBLIC nu_event_t* nu_scheduler_add_event(nu_event_func_t func,
        void* param, double timeout, uint8_t prio, const char* memo);
PUBLIC nu_event_t* nu_scheduler_add_periodic_event(
        nu_event_func_t func, void* param, uint8_t prio,
        double start, double interval, double jitter,
        const char* memo);
PUBLIC void nu_scheduler_cancel_event(nu_event_t*);
PUBLIC void nu_scheduler_resort_event(nu_event_t*);
PUBLIC void nu_scheduler_exec_events(void);

PUBLIC void nu_scheduler_put_log(nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
