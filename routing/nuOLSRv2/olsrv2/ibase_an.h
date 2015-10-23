//
// (AN) Attached Network Set
//
#ifndef OLSRV2_IBASE_AN_H_
#define OLSRV2_IBASE_AN_H_

#include "olsrv2/ibase_time.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_an OLSRv2 :: (AN) Attached Network Tuple
 * @{
 */

/**
 * (AN) Attached Network Tuple
 */
typedef struct tuple_an {
    struct tuple_an*  next;         ///< next tuple
    struct tuple_an*  prev;         ///< prev tuple

    tuple_time_t*     timeout_next; ///< next in timeout list
    tuple_time_t*     timeout_prev; ///< prev in timeout list
    tuple_time_proc_t timeout_proc; ///< timeout processor
    nu_time_t         time;         ///< timeout time

    nu_ip_t           net_addr;     ///< AN_net_addr
    nu_ip_t           orig_addr;    ///< AN_orig_addr
    uint8_t           dist;         ///< AN_dist
    uint16_t          seqnum;       ///< AN_seq_number

    nu_link_metric_t  metric;       ///< AN_metric
} tuple_an_t;

/**
 * (AN) Attached Network Set
 */
typedef struct ibase_an {
    tuple_an_t* next;   ///< first tuple
    tuple_an_t* prev;   ///< last tuple
    size_t      n;      ///< size
    nu_bool_t   change; ///< change flag
} ibase_an_t;

////////////////////////////////////////////////////////////////
//
// ibase_an_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_an_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC void ibase_an_init(void);
PUBLIC void ibase_an_destroy(void);

PUBLIC tuple_an_t* ibase_an_get(
        const nu_ip_t orig_addr, const uint16_t seqnum,
        const nu_ip_t net_addr, const uint8_t dist);
PUBLIC tuple_an_t* ibase_an_iter_remove(tuple_an_t*);
PUBLIC void ibase_an_remove_orig_addr(const nu_ip_t);

PUBLIC void ibase_an_put_log(nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
