//
// (TA) Routable Address Topology Set
//
#ifndef OLSRV2_IBASE_TA_H_
#define OLSRV2_IBASE_TA_H_

#include "core/ip.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_ta OLSRv2 :: (TA) Routable Address Topology
 * @{
 */

/**
 * (TA) Routable Address Topology Tuple
 */
typedef struct tuple_ta {
    struct tuple_ta*  next;           ///< next tuple
    struct tuple_ta*  prev;           ///< prev tuple

    tuple_time_t*     timeout_next;   ///< next tuple in timeout list
    tuple_time_t*     timeout_prev;   ///< prev tuple in timeout list
    tuple_time_proc_t timeout_proc;   ///< timeout processor
    nu_time_t         time;           ///< TA_time

    nu_ip_t           from_orig_addr; ///< TA_from_orig_addr
    nu_ip_t           dest_ip;        ///< TA_dest_ifaddr
    uint16_t          seqnum;         ///< TA_seq_number

    nu_link_metric_t  metric;         ///< TA_metric
} tuple_ta_t;

/**
 * (TA) Routable Address Topology Set
 */
typedef struct ibase_ta {
    tuple_ta_t* next;   ///< first tuple
    tuple_ta_t* prev;   ///< last tuple
    size_t      n;      ///< size
    nu_bool_t   change; ///< change flag
} ibase_ta_t;

////////////////////////////////////////////////////////////////
//
// ibase_taa_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_ta_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC void ibase_ta_init(void);
PUBLIC void ibase_ta_destroy(void);

PUBLIC tuple_ta_t* ibase_ta_get(const nu_ip_t from_orig_addr,
        const nu_ip_t dest_addr);
PUBLIC tuple_ta_t* ibase_ta_iter_remove(tuple_ta_t*);
PUBLIC void ibase_ta_remove_orig_addr(const nu_ip_t);

PUBLIC void tuple_ta_put_log(tuple_ta_t*, nu_logger_t*);
PUBLIC void ibase_ta_put_log(nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
