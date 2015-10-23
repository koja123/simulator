//
// (R) Routing Set
//
#ifndef OLSRV2_IBASE_R_H_
#define OLSRV2_IBASE_R_H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_r OLSRv2 :: Routing Tuple
 * @{
 */

/**
 * (R) Routing Tuple
 */
typedef struct tuple_r {
    struct tuple_r*  next;    ///< next tuple
    struct tuple_r*  prev;    ///< prev tuple

    nu_ip_t          dest_ip; ///< R_dest_addr
    nu_ip_t          next_ip; ///< R_next_iface_addr
    uint8_t          dist;    ///< R_dist
    const tuple_i_t* tuple_i; ///< tuple_I

    nu_link_metric_t metric;  ///< R_metric

    uint8_t          mark;    ///< for mark & sweep
} tuple_r_t;

/**
 * (R) Routing Set
 */
typedef struct ibase_r {
    tuple_r_t* next;   ///< first tuple
    tuple_r_t* prev;   ///< last tuple

    size_t     n;      ///< size
    nu_bool_t  change; ///< change flag
} ibase_r_t;

////////////////////////////////////////////////////////////////
//
// ibase_r_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_r_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC void ibase_r_init(void);
PUBLIC void ibase_r_destroy(void);

PUBLIC void ibase_r_reset(void);
PUBLIC void ibase_r_update(void);

PUBLIC tuple_r_t* ibase_r_add(const nu_ip_t dest, const nu_ip_t next,
        const nu_link_metric_t metric, const uint8_t dist, const tuple_i_t*);
PUBLIC nu_bool_t ibase_r_peek(const nu_ip_t dest, const nu_ip_t next,
        const uint8_t dist, const tuple_i_t* tuple_i);
PUBLIC void ibase_r_delete(tuple_r_t*);
PUBLIC nu_bool_t ibase_r_contain_dest(const nu_ip_t);
PUBLIC tuple_r_t* ibase_r_search_dest(const nu_ip_t);
PUBLIC tuple_r_t* ibase_r_search_dest_and_dist(
        const nu_ip_t, const uint8_t);
PUBLIC tuple_r_t* ibase_r_search_dest_ip_set(const nu_ip_set_t*);

PUBLIC void tuple_r_put_log(tuple_r_t*, nu_logger_t*);
PUBLIC void ibase_r_put_log(nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
