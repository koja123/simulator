//
// (O) Originator Set
//
#ifndef OLSRV2_IBASE_O_H_
#define OLSRV2_IBASE_O_H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_o OLSRv2 :: (O) Originator Tuple
 * @{
 */

/**
 * (O) Originator Tuple
 *
 * @see tuple_ip_t
 */
typedef struct tuple_o {
    struct tuple_o*   next;         ///< next tuple
    struct tuple_o*   prev;         ///< prev tuple

    tuple_time_t*     timeout_next; ///< next in timeout list
    tuple_time_t*     timeout_prev; ///< prev in timeout list
    tuple_time_proc_t timeout_proc; ///< timeout processor
    nu_time_t         time;         ///< timeout time
    ibase_ip_t*       ibase;        ///< ibase
    ibase_time_t*     time_list;    ///< timeout list

    nu_ip_t           orig_addr;    ///< O_orig_addr
} tuple_o_t;

/**
 * (O) Originator Set
 *
 * @see ibase_ip_t
 */
typedef ibase_ip_t   ibase_o_t;

////////////////////////////////////////////////////////////////
//
// ibase_o_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_o_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Initializes the ibase_o. */
#define ibase_o_init()       ibase_ip_init(IBASE_O)
/** Destroys the ibase_o. */
#define ibase_o_destroy()    ibase_ip_destroy(IBASE_O)

/** Adds the ip to the ibase_o.
 *
 * @param ip
 */
#define ibase_o_add(ip) \
    ibase_ip_add_with_timeout(IBASE_O, ip, O_HOLD_TIME, &OLSR->ibase_o_time_list)

/** Checks whether the ibase_o contains the ip.
 *
 * @param ip
 * @return true if the ibase_o contains the ip.
 */
#define ibase_o_contain(ip)    ibase_ip_contain_orig_addr(IBASE_O, ip)

/** Outputs log.
 *
 * @param logger
 */
#define ibase_o_put_log(logger) \
    ibase_ip_put_log(IBASE_O, logger, "I_O")

/** @} */

}//namespace// //ScenSim-Port://

#endif
