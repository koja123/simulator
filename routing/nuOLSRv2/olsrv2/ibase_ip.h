//
// Information base of network address and timeout.
//
//  (IR)
//  (NL)
//  (O)
//  (N2)
//
#ifndef OLSRV2_IBASE_IP_H_
#define OLSRV2_IBASE_IP_H_

#include "olsrv2/ibase_time.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_ip OLSRv2 :: (IR) (NL) (O) (N2) Simple information base.
 * @{
 */

struct ibase_ip;

/**
 * Tuple of timeout and ip address pair
 */
typedef struct tuple_ip {
    struct tuple_ip*  next;         ///< next tuple
    struct tuple_ip*  prev;         ///< prev tuple

    tuple_time_t*     timeout_next; ///< next in timeout list
    tuple_time_t*     timeout_prev; ///< prev in timeout list
    tuple_time_proc_t timeout_proc; ///< timeout processor
    nu_time_t         time;         ///< timeout time
    struct ibase_ip*  ibase;        ///< ibase
    ibase_time_t*     time_list;    ///< timeout tuple list

    nu_ip_t           ip;           ///< ip
} tuple_ip_t;

/**
 * Information base for tuple_ip
 */
typedef struct ibase_ip {
    tuple_ip_t* next;   ///< first tuple
    tuple_ip_t* prev;   ///< last tuple
    size_t      n;      ///< size
    nu_bool_t   change; ///< change flag
} ibase_ip_t;

////////////////////////////////////////////////////////////////
//
// ibase_ip_t
//

/** Sets the change flag of the ibase_ip.
 *
 * @param ibase
 */
#define ibase_ip_change(ibase)     \
    do { (ibase)->change = true; } \
    while (0)

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_ip_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC void ibase_ip_init(ibase_ip_t*);
PUBLIC void ibase_ip_destroy(ibase_ip_t*);
PUBLIC tuple_ip_t* ibase_ip_add(ibase_ip_t*, const nu_ip_t, ibase_time_t*);
PUBLIC tuple_ip_t* ibase_ip_add_with_timeout(ibase_ip_t*, const nu_ip_t, double, ibase_time_t*);
PUBLIC nu_bool_t ibase_ip_contain(ibase_ip_t*, const nu_ip_t);
PUBLIC nu_bool_t ibase_ip_contain_orig_addr(ibase_ip_t*, const nu_ip_t);
PUBLIC tuple_ip_t* ibase_ip_search(ibase_ip_t*, const nu_ip_t);
PUBLIC tuple_ip_t* ibase_ip_iter_remove(tuple_ip_t*, ibase_ip_t*);
PUBLIC void ibase_ip_remove(ibase_ip_t*, const nu_ip_t);
PUBLIC void ibase_ip_remove_all(ibase_ip_t*);

PUBLIC void ibase_ip_put_log(ibase_ip_t*, nu_logger_t*, const char*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
