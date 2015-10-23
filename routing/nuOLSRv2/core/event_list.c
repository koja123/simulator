#include "config.h"

#include <math.h>
#include <time.h>
#include <errno.h>

#include "core/core.h"
#include "core/mem.h"
#include "core/event_list.h"
#include "core/scheduler.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_scheduler
 * @{
 */

/*
 */
static nu_event_t*
nu_event_create(nu_event_func_t func, void* param,
        double timeout, uint8_t prio, const char* memo)
{
    nu_event_t* ev = nu_mem_alloc(nu_event_t);
    ev->func  = func;
    ev->param = param;
    ev->memo  = memo;

    ev->periodic = false;
    ev->interval = 0;
    ev->max_jitter = 0;

    ev->prio = prio;
    ev->time = NU_NOW;
    nu_time_add_f(&ev->time, timeout);

    return ev;
}

/*
 */
static nu_event_t*
nu_event_create_periodic(nu_event_func_t func, void* param,
        uint8_t prio, double start, double interval, double jitter,
        const char* memo)
{
    assert(interval > -1.0);
    assert(jitter >= 0.0);

    nu_event_t* ev = nu_mem_alloc(nu_event_t);
    ev->func  = func;
    ev->param = param;
    ev->memo  = memo;

    ev->periodic = true;
    ev->interval = interval;
    ev->max_jitter = jitter;
    ev->prio = prio;

    ev->time = NU_NOW;
    nu_time_add_f(&ev->time, start);
    return ev;
}

#if 0

/**
 */
static nu_event_t*
nu_event_set_timeout(nu_event_t* ev, double timeout)
{
    ev->time = NU_NOW;
    nu_time_add_f(&ev->time, timeout);
    return ev;
}

#endif

/**
 */
static nu_event_t*
nu_event_set_next_time(nu_event_t* ev)
{
    const double jitter = ev->max_jitter * nu_rand();
    ev->time = NU_NOW;
    nu_time_add_f(&ev->time, ev->interval - jitter);
    return ev;
}

/*
 */
static inline int
nu_event_cmp(const nu_event_t* x, const nu_event_t* y)
{
    int r = 0;
    if ((r = nu_time_cmp(x->time, y->time)) != 0)
        return r;
    return ((int)x->prio) - ((int)y->prio);
}

/*
 */
static inline int
nu_event_eq(const nu_event_t* x, const nu_event_t* y)
{
    return nu_time_cmp(x->time, y->time) == 0 &&
           x->prio == x->prio &&
           x->func == y->func &&
           x->param == y->param;
}

/* Inserts the new event.
 *
 * This function searches the insert point of the new event from
 * the top of the event list.
 *
 * @param self
 * @param event
 */
static void
nu_event_list_insert(nu_event_list_t* self, nu_event_t* event)
{
    NuOLSRv2ProtocolScheduleEvent(event->time); //ScenSim-Port://
    FOREACH_EVENT_LIST(ev, self) {
        if (nu_event_cmp(ev, event) >= 0) {
            nu_event_list_iter_insert_before(ev, self, event);
            return;
        }
    }
    nu_event_list_insert_tail(self, event);
}

/** Inserts the new event.
 *
 * This function searches the insert point of the new event from
 * the tail of the event list.
 *
 * @param self
 * @param event
 */
PUBLIC void
nu_event_list_insert_r(nu_event_list_t* self, nu_event_t* event)
{
    NuOLSRv2ProtocolScheduleEvent(event->time); //ScenSim-Port://
    FOREACH_EVENT_LIST_R(ev, self) {
        if (nu_event_cmp(ev, event) < 0) {
            nu_event_list_iter_insert_after(ev, self, event);
            return;
        }
    }
    nu_event_list_insert_head(self, event);
}

/*
 */
static inline void
nu_event_list_cut(nu_event_list_t* self, nu_event_t* event)
{
    nu_event_list_iter_cut(event, self);
}

/** Creates and inserts the new event.
 *
 * @param self
 * @param func
 * @param param
 * @param timeout
 * @param prio
 * @param memo
 * @return the new event object
 */
PUBLIC nu_event_t*
nu_event_list_add(nu_event_list_t* self,
        nu_event_func_t func, void* param,
        double timeout, uint8_t prio, const char* memo)
{
    nu_event_t* ev = nu_event_create(func, param, timeout, prio, memo);
    nu_event_list_insert(self, ev);
    return ev;
}

/** Creates and inserts the new periodic event.
 *
 * @param self
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
nu_event_list_add_periodic(nu_event_list_t* self,
        nu_event_func_t func, void* param,
        uint8_t prio, double start, double interval, double jitter,
        const char* memo)
{
    nu_event_t* ev = nu_event_create_periodic(func, param,
            prio, start, interval, jitter, memo);
    nu_event_list_insert(self, ev);
    return ev;
}

/** Resorts the event.
 *
 * @param self
 * @param event
 */
PUBLIC void
nu_event_list_resort(nu_event_list_t* self, nu_event_t* event)
{
    nu_event_list_cut(self, event);
    nu_event_list_insert(self, event);
}

/** Cancels the event.
 *
 * @param self
 * @param event
 */
PUBLIC void
nu_event_list_cancel(nu_event_list_t* self, nu_event_t* event)
{
    nu_event_list_cut(self, event);
    nu_mem_free(event);
}

/** Executes the events.
 *
 * @param self
 */
PUBLIC void
nu_event_list_exec(nu_event_list_t* self)
{
    nu_event_t* ev = NULL;
    while ((ev = nu_event_list_head(self)) != NULL) {
        if (nu_time_cmp(ev->time, NU_NOW) > 0)
            break;
        ev->func(ev);
        nu_event_list_iter_cut(ev, self);
        if (ev->periodic)
            nu_event_list_insert(self, nu_event_set_next_time(ev));
        else
            nu_mem_free(ev);
    }
}

/** Outputs debug info.
 *
 * @param self
 * @param logger
 */
PUBLIC void
nu_event_put_log(const nu_event_t* self, nu_logger_t* logger)
{
    if (self->periodic) {
        nu_logger_log(logger, "ev[t:%g step:%g jitter:%g %s]",
                nu_time_sub(self->time, NU_NOW),
                self->interval, self->max_jitter,
                (self->memo ? self->memo : ""));
    } else {
        nu_logger_log(logger, "ev[t:%g %s]",
                nu_time_sub(self->time, NU_NOW),
                (self->memo ? self->memo : ""));
    }
}

/** Outputs debug info.
 *
 * @param self
 * @param logger
 */
PUBLIC void
nu_event_list_put_log(nu_event_list_t* self, nu_logger_t* logger)
{
    nu_logger_log(logger, "EventList:[");
    nu_logger_push_prefix(logger, "  ");
    FOREACH_EVENT_LIST(e, self) {
        nu_event_put_log(e, logger);
    }
    nu_logger_pop_prefix(logger);
    nu_logger_log(logger, "]");
}

/** @} */

}//namespace// //ScenSim-Port://
