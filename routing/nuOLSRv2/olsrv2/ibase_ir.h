//
// (IR) Removed Interface Address Set
//
#ifndef OLSRV2_IBASE_IR_H_
#define OLSRV2_IBASE_IR_H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_ir OLSRv2 :: (IR) Removed Interface Address Tuple
 * @{
 */

/**
 * (IR) Removed Interface Address Tuple
 *
 * @see tuple_ip_t
 */
typedef struct tuple_ir {
    struct tuple_ir*  next;         ///< next
    struct tuple_ir*  prev;         ///< prev

    tuple_time_t*     timeout_next; ///< next in timeout_list
    tuple_time_t*     timeout_prev; ///< prev in timeout_lsit
    tuple_time_proc_t timeout_proc; ///< timeout procedure.
    nu_time_t         time;         ///< timeout time
    ibase_ip_t*       ibase;        ///< ibase
    ibase_time_t*     time_list;    ///< timeout list

    nu_ip_t           local_ip;     ///< IR_local_ifaddr
} tuple_ir_t;

/**
 * (IR) Removed Interface Address Set
 *
 * @see ibase_ip_t
 */
typedef ibase_ip_t   ibase_ir_t;

////////////////////////////////////////////////////////////////
//
// ibase_ir
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_ir_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/**
 * Initializes ibase_ir.
 */
#define ibase_ir_init()         ibase_ip_init(IBASE_IR)

/**
 * Destroys ibase_ir.
 */
#define ibase_ir_destroy()      ibase_ip_destroy(IBASE_IR)

/** Adds the ip to the ibase_ir.
 *
 * @param ip
 */
#define ibase_ir_add(ip)        ibase_ip_add(IBASE_IR, ip, &OLSR->ibase_ir_time_list)

/** Removes the ip from ibase_ir.
 *
 * @param ip
 */
#define ibase_ir_remove(ip)     ibase_ip_remove(IBASE_IR, ip)

/** Checks whether the ibase_ir contains ip.
 *
 * @param ip
 */
#define ibase_ir_contain(ip)    ibase_ip_contain(IBASE_IR, ip)

/** Outputs log.
 *
 * @param logger
 */
#define ibase_ir_put_log(logger) \
    ibase_ip_put_log(IBASE_IR, logger, "I_IR")

/** @} */

}//namespace// //ScenSim-Port://

#endif
