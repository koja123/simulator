//
// (NL) Lost Neighbor Set
//
#ifndef OLSRV2_IBASE_NL_H_
#define OLSRV2_IBASE_NL_H_

#include "olsrv2/ibase_ip.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_nl OLSRv2 :: (NL) Lost Neighbor Tuple
 * @{
 */

/**
 * (NL) Lost Neighbor Tuple
 *
 * @see tuple_ip_t
 */
typedef struct tuple_nl {
    struct tuple_nl*  next;         ///< next tuple
    struct tuple_nl*  prev;         ///< prev tuple

    tuple_time_t*     timeout_next; ///< next in timeout list
    tuple_time_t*     timeout_prev; ///< prev in timeout list
    tuple_time_proc_t timeout_proc; ///< timeout processor
    nu_time_t         time;         ///< timeout time
    ibase_ip_t*       ibase;        ///< ibase
    ibase_time_t*     time_list;    ///< timeout list

    nu_ip_t           neighbor_ip;  ///< NL_neighbor_addr
} tuple_nl_t;

/**
 * (NL) Lost Neighbor Set
 *
 * @see ibase_ip_t
 */
typedef ibase_ip_t   ibase_nl_t;

////////////////////////////////////////////////////////////////
//
// ibase_nl_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_nl_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Initializes the ibase_nl.
 */
#define ibase_nl_init()         ibase_ip_init(IBASE_NL)

/** Destroys the ibase_nl.
 */
#define ibase_nl_destroy()      ibase_ip_destroy(IBASE_NL)

/** Adds the ip.
 *
 * @param ip
 */
#define ibase_nl_add(ip)        ibase_ip_add_with_timeout(IBASE_NL, ip, N_HOLD_TIME, &OLSR->ibase_nl_time_list)

/** Removes the ip.
 *
 * @param ip
 */
#define ibase_nl_remove(ip)     ibase_ip_remove(IBASE_NL, ip)

/** Checks whether the ibase_nl contains the ip.
 *
 * @param ip
 * @return true if the ibase_nl contains the ip.
 */
#define ibase_nl_contain(ip)    ibase_ip_contain(IBASE_NL, ip)

/** Outputs log.
 *
 * @param logger
 */
#define ibase_nl_put_log(logger) \
    ibase_ip_put_log(IBASE_NL, logger, "I_NL")

/** @} */

}//namespace// //ScenSim-Port://

#endif
