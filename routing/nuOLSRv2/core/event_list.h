#ifndef NU_CORE_EVENT_LIST_H_
#define NU_CORE_EVENT_LIST_H_

#include "core/core.h"
#include "core/mem.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_event Core :: Event list
 * @brief event and event list
 * @{
 */

#define NU_EVENT_PRIO__HIGH       (0x40)  ///< event priority
#define NU_EVENT_PRIO__DEFAULT    (0x80)  ///< event priority
#define NU_EVENT_PRIO__LOW        (0xc0)  ///< event priority

struct nu_event;

/**
 * Eventor function
 */
typedef void (*nu_event_func_t)(struct nu_event*);

/**
 * Event
 */
typedef struct nu_event {
    struct nu_event* next;        ///< prev event
    struct nu_event* prev;        ///< next event

    nu_time_t        time;        ///< time that event occurs
    uint8_t          prio;        ///< priority

    nu_event_func_t  func;        ///< calllback function
    void*            param;       ///< parameter for callback function

    nu_bool_t        periodic;    ///< periodic flag
    double           interval;    ///< interval (if periodic)
    double           max_jitter;  ///< maximum jitter (if periodic)

    const char*      memo;        ///< memo (for debug)
} nu_event_t;

/**
 * Event list
 */
typedef struct nu_event_list {
    nu_event_t* next;     ///< the first event
    nu_event_t* prev;     ///< the last  event
    size_t      n;        ///< size
} nu_event_list_t;

#define nu_event_free    nu_mem_free    ///< nu_event destructor.

}//namespace// //ScenSim-Port://

#include "core/event_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC void nu_event_put_log(const nu_event_t*, nu_logger_t*);

PUBLIC nu_event_t* nu_event_list_add(nu_event_list_t*,
        nu_event_func_t func, void* param,
        double timeout, uint8_t prio, const char* memo);
PUBLIC nu_event_t* nu_event_list_add_periodic(nu_event_list_t*,
        nu_event_func_t func, void* param,
        uint8_t prio, double start, double interval, double jitter,
        const char* memo);

PUBLIC void nu_event_list_resort(nu_event_list_t*, nu_event_t* ev);
PUBLIC void nu_event_list_cancel(nu_event_list_t*, nu_event_t* ev);
PUBLIC void nu_event_list_exec(nu_event_list_t*);

PUBLIC void nu_event_list_put_log(nu_event_list_t*, nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
