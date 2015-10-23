#ifndef NU_CORE_H_
#define NU_CORE_H_

#include "config.h" //ScenSim-Port://
#include "core/time.h"
#include "core/logger.h"
#include "core/ip.h"
#include "core/scheduler.h"

#if defined(_WIN32) || defined(_WIN64)                           //ScenSim-Port://
typedef int ssize_t;                                             //ScenSim-Port://
#define snprintf(buf, fmt, ...) _snprintf(buf, fmt, __VA_ARGS__) //ScenSim-Port://
#endif                                                           //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_core Core :: Core module
 * @{
 */

/**
 * Core module
 */
typedef struct {
    nu_logger_t              logger;              ///< Logger
    short                    af;                  ///< Address family (IP version)
    nu_time_t                now;                 ///< Current time
    nu_ip_heap_t             ip_heap;             ///< IP address table

    nu_ip_attr_create_func_t ip_attr_constructor; ///< ip_attr's constructor
    nu_ip_attr_free_func_t   ip_attr_destructor;  ///< ip_attr's destructor

    double                   pollrate;            ///< Polling rate
    nu_event_list_t          event_list;          ///< Event list

    nu_bool_t                log_ip_heap;         ///< ip_heap log flag
    nu_bool_t                log_scheduler;       ///< scheduler log flag
} nu_core_t;

/**
 * Log flag
 */
typedef struct nu_log_flag {
    const char* name;       ///< log flag name
    const char* short_name; ///< short log flag name
    size_t      offset;     ///< offset in the module object
    const char* memo;       ///< memo about the log flag
} nu_log_flag_t;

#define NU_LOGGER     (&current_core->logger)             ///< current logger
#define NU_AF         (current_core->af)                  ///< current af
#define NU_NOW        (current_core->now)                 ///< current time
#define NU_IP_HEAP    (current_core->ip_heap)             ///< current ip_heap

#define NU_IP_ATTR_CONSTRUCTOR \
    (current_core->ip_attr_constructor)                 ///< current ip_attr_constructor
#define NU_IP_ATTR_DESTRUCTOR \
    (current_core->ip_attr_destructor)                  ///< current ip_attr_destructor

#define NU_POLLRATE            (current_core->pollrate) ///< current poling rate

#define NU_DEFAULT_POLLRATE    (0.05)                   ///< default polling
#define NU_MAX_POLLRATE        (10.0)                   ///< maximum polling rate
#define NU_MIN_POLLRATE        (0.01)                   ///< minimum polling rate


/**
 * @return true if current default IP version is v4.
 */
#define NU_IS_V4    (NU_AF == AF_INET)

/**
 * @return true if current default IP version is v6.
 */
#define NU_IS_V6    (NU_AF == AF_INET6)

/** @fn NU_CORE_DO_LOG(flag, x)
 *
 * Output debug log
 *
 * @param flag
 * @param x
 */
#ifndef NU_NDEBUG
#define NU_CORE_DO_LOG(flag, x)       \
    if (current_core->log_ ## flag && \
        nu_logger_set_prio(NU_LOGGER, LOG_DEBUG)) { x }
#else
#define NU_CORE_DO_LOG(flag, x)
#endif

/**
 * The pointer to current core module
 */
#ifdef nuOLSRv2_ALL_STATIC
static nu_core_t* current_core = NULL;
#else
extern nu_core_t* current_core;
#endif

PUBLIC nu_core_t* nu_core_create(void);
PUBLIC nu_core_t* nu_core_dup(const nu_core_t*);
PUBLIC void nu_core_free(nu_core_t*);

PUBLIC void nu_core_put_params(const char*, FILE*);
PUBLIC nu_bool_t nu_core_set_param(const char*);

// Utility functions for log flags
PUBLIC nu_bool_t nu_set_log_flag(const char*, nu_log_flag_t*, void*);
PUBLIC void nu_put_log_flags(const char*, nu_log_flag_t*, FILE*);
PUBLIC void nu_init_log_flags(nu_log_flag_t*, void*);

/** Sets default address family.
 *
 * @param af
 */
PUBLIC_INLINE void
nu_set_af(short af)
{
    NU_AF = af;
}

/** Compares sequence number.
 *
 * @param s1
 * @param s2
 * @retval true  if s1 > s2
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_seqnum_gt(const uint16_t s1, const uint16_t s2)
{
    return (int16_t)(s1 - s2) > 0;
}

/** Calculates difference between sequence numbers.
 *
 * nu_seqnum_gt(s1, s2) must be greater or equal than 0.
 *
 * @param s1
 * @param s2
 * @return s1 - s2
 */
PUBLIC_INLINE uint16_t
nu_seqnum_diff(const uint16_t s1, const uint16_t s2)
{
    return s1 - s2;
}

/** Calculates the remaining time until timeout.
 *
 * @param time
 * @return time - (current time)
 */
PUBLIC_INLINE double
nu_time_calc_timeout(const nu_time_t time)
{
    return nu_time_sub(time, NU_NOW);
}

/** Checks whether the time is expired or not.
 *
 * @param time
 * @retval true  if time is expired
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_time_is_expired(const nu_time_t time)
{
    return nu_time_cmp(time, NU_NOW) < 0;
}

/** Calculates timeout time.
 *
 * @param[out] time
 * @param period
 */
PUBLIC_INLINE void
nu_time_set_timeout(nu_time_t* time, const double period)
{
    *time = NU_NOW;
    nu_time_add_f(time, period);
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
