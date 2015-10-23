//
// (LT) Local Topology Set
//
#ifndef OLSRV2_IBASE_LT_H_
#define OLSRV2_IBASE_LT_H_

namespace NuOLSRv2Port { //ScenSim-Port://

struct tuple_n;
struct tuple_l;
struct tuple_n2;

/************************************************************//**
 * @defgroup ibase_lt OLSRv2 :: (LT) Local Topology Set
 * @{
 */

/**
 * (LT) Local Topology Tuple
 */
typedef struct tuple_lt {
    struct tuple_lt* next;          ///< next tuple
    struct tuple_lt* prev;          ///< prev tuple

    struct tuple_n*  next_id;       ///< next_orig_addr
    struct tuple_n*  last_id;       ///< last_orig_addr
    struct tuple_n2* final_ip;      ///< final ip
    nu_link_metric_t last_metric;   ///< last metric
    nu_link_metric_t final_metric;  ///< final metric
    size_t           number_hops;   ///< the number of hops
} tuple_lt_t;

/**
 * (LT) Local Topology Set
 */
typedef struct ibase_lt {
    tuple_lt_t* next;   ///< first tuple
    tuple_lt_t* prev;   ///< last tuple
    size_t      n;      ///< size
    nu_bool_t   sorted; ///< true if elements are sorted.
} ibase_lt_t;

////////////////////////////////////////////////////////////////
//
// tuple_lt_t
//

////////////////////////////////////////////////////////////////
//
// ibase_lt_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_lt_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC tuple_lt_t* ibase_lt_iter_remove(tuple_lt_t*, ibase_lt_t*);

PUBLIC void ibase_lt_init(ibase_lt_t*);
PUBLIC void ibase_lt_destroy(ibase_lt_t*);

PUBLIC void ibase_lt_add(ibase_lt_t*, struct tuple_n*, struct tuple_n*, struct tuple_n2*);
PUBLIC void ibase_lt_resort(ibase_lt_t*, tuple_lt_t*);
PUBLIC void ibase_lt_sort(ibase_lt_t*);
PUBLIC void ibase_lt_sort_by_final_ip(ibase_lt_t*);

PUBLIC void tuple_lt_put_log(tuple_lt_t*, nu_logger_t*);
PUBLIC void ibase_lt_put_log(ibase_lt_t*, nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
