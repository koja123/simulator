//
// (TR) Router Topology Set
//
#ifndef OLSRV2_IBASE_TR_H_
#define OLSRV2_IBASE_TR_H_

#include "core/ip.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_tr OLSRv2 :: (TR) Router Topology Tuple
 * @{
 */

/**
 * (TR) Router Topology Tuple
 */
typedef struct tuple_tr {
    struct tuple_tr*  next;           ///< next tuple
    struct tuple_tr*  prev;           ///< prev tuple

    tuple_time_t*     timeout_next;   ///< next tuple in timeout list
    tuple_time_t*     timeout_prev;   ///< next tuple in timeout list
    tuple_time_proc_t timeout_proc;   ///< timeout processor
    nu_time_t         time;           ///< TR_time

    nu_ip_t           from_orig_addr; ///< TR_from_orig_addr
    nu_ip_t           to_orig_addr;   ///< TR_to_orig_addr
    uint16_t          seqnum;         ///< TR_seq_number

    nu_link_metric_t  metric;         ///< TR_metric
} tuple_tr_t;

/**
 * (TR) Router Topology Set
 */
typedef struct ibase_tr {
    tuple_tr_t* next;   ///< first tuple
    tuple_tr_t* prev;   ///< last tuple
    size_t      n;      ///< size
    nu_bool_t   change; ///< change flag
} ibase_tr_t;

////////////////////////////////////////////////////////////////
//
// ibase_tr_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_tr_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC void ibase_tr_init(void);
PUBLIC void ibase_tr_destroy(void);

PUBLIC tuple_tr_t* ibase_tr_get(const nu_ip_t from_orig_addr,
        const nu_ip_t to_orig_addr);
PUBLIC tuple_tr_t* ibase_tr_iter_remove(tuple_tr_t*);
PUBLIC void ibase_tr_remove_orig_addr(const nu_ip_t);

PUBLIC void ibase_tr_put_log(nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
