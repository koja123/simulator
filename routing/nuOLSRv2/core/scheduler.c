#include "config.h"

#include <math.h>
#include <time.h>
#include <errno.h>

#include "core/core.h"
#include "core/mem.h"
#include "core/scheduler.h"
#ifdef USE_NETIO
#include "netio/netio.h"
#endif

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_scheduler
 * @{
 */

/** Current event_list */
#define EVENT_LIST    (&current_core->event_list)

// function to call before/after execute events.
static nu_scheduler_pre_func_t  pre_event_exec_function  = NULL;
static nu_scheduler_post_func_t post_event_exec_function = NULL;

#ifndef noarch
static int nano_sleep(double);
#endif

/** Adds the new event to the scheduler.
 *
 * @param func
 * @param param
 * @param timeout
 * @param prio
 * @param memo
 * @return the new event object
 */
PUBLIC nu_event_t*
nu_scheduler_add_event(nu_event_func_t func, void* param,
        double timeout, uint8_t prio, const char* memo)
{
    return nu_event_list_add(EVENT_LIST, func, param, timeout, prio, memo);
}

/** Adds the new periodic event to the scheduler.
 *
 * @param func
 * @param param
 * @param prio
 * @param start
 * @param interval
 * @param jitter
 * @param memo
 * @return the new event object
 */
PUBLIC nu_event_t*
nu_scheduler_add_periodic_event(nu_event_func_t func, void* param,
        uint8_t prio, double start, double interval, double jitter,
        const char* memo)
{
    return nu_event_list_add_periodic(EVENT_LIST, func, param,
            prio, start, interval, jitter, memo);
}

/** Cancels the scheduled event.
 *
 * @param event
 */
PUBLIC void
nu_scheduler_cancel_event(nu_event_t* event)
{
    nu_event_list_cancel(EVENT_LIST, event);
}

/** Resorts the scheduled event.
 *
 * @param event
 */
PUBLIC void
nu_scheduler_resort_event(nu_event_t* event)
{
    nu_event_list_resort(EVENT_LIST, event);
}

/**
 * @param rate
 */
PUBLIC void
nu_scheduler_set_pollrate(double rate)
{
    NU_POLLRATE = rate;
}

/**
 * @param func
 */
PUBLIC void
nu_scheduler_set_pre_func(nu_scheduler_pre_func_t func)
{
    pre_event_exec_function = func;
}

/**
 * @param func
 */
PUBLIC void
nu_scheduler_set_post_func(nu_scheduler_post_func_t func)
{
    post_event_exec_function = func;
}

/**
 * Executes events.
 */
PUBLIC void
nu_scheduler_exec_events(void)
{
    NU_CORE_DO_LOG(scheduler,
            nu_logger_push_prefix(NU_LOGGER, "SCHEDULER:");
            nu_scheduler_put_log(NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );

#ifdef USE_NETIO
    nu_socket_poll();
#endif

    if (pre_event_exec_function)
        pre_event_exec_function();

    nu_event_list_exec(EVENT_LIST);

    if (post_event_exec_function)
        post_event_exec_function();
}

/*
 */
#if !defined(_WIN32) && !defined(_WIN64) //ScenSim-Port://
static int
nano_sleep(double t)
{
    struct timespec req;
    struct timespec rem;
    int res;

    req.tv_sec  = 0;
    req.tv_nsec = (long)t * 1000000;
    rem.tv_sec  = 0;
    rem.tv_nsec = 0;
    if ((res = nanosleep(&req, &rem)) < 0) {
        perror("nanosleep");
        exit(1);
    }
    return res;
}
#endif                                   //ScenSim-Port://

/** Initializes scheuler.
 */
PUBLIC void
nu_scheduler_init(void)
{
    NU_POLLRATE = NU_DEFAULT_POLLRATE;
    nu_event_list_init(EVENT_LIST);
}

#ifndef nuOLSRv2_SIMULATOR

/** Starts scheduler.
 */
PUBLIC void
nu_scheduler_start(void)
{
    const double poll = NU_POLLRATE * 1000; // [sec] -> [msec]
    for (;;) {
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
        assert(false);                 //ScenSim-Port://
        abort();                       //ScenSim-Port://
#else                                  //ScenSim-Port://
        nano_sleep(poll);
#endif                                 //ScenSim-Port://
        nu_time_set_now(&NU_NOW);
        nu_scheduler_exec_events();
    }
}

#endif

/** Stops scheduler.
 */
PUBLIC void
nu_scheduler_shutdown(void)
{
    NU_CORE_DO_LOG(scheduler,
            nu_logger_push_prefix(NU_LOGGER, "SCHEDULER:Shutdown: ");
            );
    nu_event_t* ev;
    while (1) {
        ev = nu_event_list_shift(EVENT_LIST);
        if (ev == NULL) {
            break;
        }
        NU_CORE_DO_LOG(scheduler,
                nu_event_put_log(ev, NU_LOGGER);
                );
        nu_event_free(ev);
    }
    NU_CORE_DO_LOG(scheduler,
            nu_logger_pop_prefix(NU_LOGGER);
            );
}

/** Outputs the event queue.
 *
 * @param logger
 */
PUBLIC void
nu_scheduler_put_log(nu_logger_t* logger)
{
    nu_logger_log(logger, "Scheduler:[");
    nu_logger_push_prefix(logger, "  ");
    FOREACH_EVENT_LIST(e, EVENT_LIST) {
        nu_event_put_log(e, logger);
    }
    nu_logger_pop_prefix(logger);
    nu_logger_log(logger, "]");
}

/** @} */

}//namespace// //ScenSim-Port://
