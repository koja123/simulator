//
// Temporal information base for calculating flooding mpr
//
#ifndef OLSRV2_IBASE_L2_H_
#define OLSRV2_IBASE_L2_H_

namespace NuOLSRv2Port { //ScenSim-Port://

struct tuple_n;
struct tuple_l;
struct tuple_n2;

/************************************************************//**
 * @defgroup ibase_l2 OLSRv2 :: (L2)
 * @{
 */

/**
 * (L2)
 */
typedef struct tuple_l2 {
    struct tuple_l2* next;          ///< next tuple
    struct tuple_l2* prev;          ///< prev tuple

    struct tuple_n*  next_id;       ///< next_orig_addr
    struct tuple_l*  last_id;       ///< last_orig_addr
    struct tuple_n2* final_ip;      ///< final_ip
} tuple_l2_t;

/**
 * (L2)
 */
typedef struct ibase_l2 {
    tuple_l2_t* next;   ///< first tuple
    tuple_l2_t* prev;   ///< last tuple
    size_t      n;      ///< size
    nu_bool_t   sorted; ///< true if tuples are sorted.
} ibase_l2_t;

////////////////////////////////////////////////////////////////
//
// tuple_l2_t
//

////////////////////////////////////////////////////////////////
//
// ibase_l2_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_l2_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC tuple_l2_t* ibase_l2_iter_remove(tuple_l2_t*, ibase_l2_t*);

PUBLIC void ibase_l2_init(ibase_l2_t*);
PUBLIC void ibase_l2_destroy(ibase_l2_t*);

PUBLIC void ibase_l2_add(ibase_l2_t*, struct tuple_n*, struct tuple_l*, struct tuple_n2*);
PUBLIC void ibase_l2_resort(ibase_l2_t*, tuple_l2_t*);
PUBLIC void ibase_l2_sort(ibase_l2_t*);
PUBLIC void ibase_l2_sort_by_final_ip(ibase_l2_t*);

PUBLIC void tuple_l2_put_log(tuple_l2_t*, nu_logger_t*);
PUBLIC void ibase_l2_put_log(ibase_l2_t*, nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
