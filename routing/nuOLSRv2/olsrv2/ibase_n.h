//
// (N) Neighbor Set
//
#ifndef OLSRV2_IBASE_N_H_
#define OLSRV2_IBASE_N_H_

#include "core/pvector.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_n OLSRv2 :: (N) Neighbor Tuple
 * @{
 */

/**
 * (N) Neighbor Tuple
 */
typedef struct tuple_n {
    struct tuple_n*  next;              ///< next tuple
    struct tuple_n*  prev;              ///< prev tuple

    nu_ip_set_t      neighbor_ip_list;  ///< N_neighbor_ifaddr_list
    nu_bool_t        symmetric;         ///< N_symmetric
    nu_ip_t          orig_addr;         ///< N_orig_addr
    uint8_t          willingness;       ///< N_willingness
    nu_bool_t        routing_mpr;       ///< N_routing_mpr
    nu_bool_t        flooding_mpr;      ///< N_flooding_mpr
    nu_bool_t        mpr_selector;      ///< N_mpr_selector
    nu_bool_t        advertised;        ///< N_advertised
    nu_link_metric_t _in_metric;        ///< N_in_metric
    nu_link_metric_t _out_metric;       ///< N_out_metric

    nu_bool_t        flooding_mprs;     ///< ...
    nu_bool_t        advertised_save;   ///< previous advertised status
    nu_bool_t        routing_mpr_save;  ///< previous mpr status
    nu_bool_t        flooding_mpr_save; ///< previous mpr status
    nu_link_metric_t in_metric_save;    ///< previous N_in_metric
    nu_link_metric_t out_metric_save;   ///< previous N_out_metric

    size_t           tuple_l_count;     ///< # of tuple_l
} tuple_n_t;

/**
 * (N) Neighbor Set
 */
typedef struct ibase_n {
    tuple_n_t* next;          ///< first tuple
    tuple_n_t* prev;          ///< last tuple
    size_t     n;             ///< size
    uint16_t   ansn;          ///< ANSN
    nu_bool_t  change;        ///< change flag for links
    nu_bool_t  sym_change;    ///< change flag for symmetric links
} ibase_n_t;

////////////////////////////////////////////////////////////////
//
// tuple_n_t
//

PUBLIC nu_bool_t tuple_n_has_symlink(const tuple_n_t*);
PUBLIC nu_bool_t tuple_n_has_link(const tuple_n_t*);
PUBLIC void tuple_n_add_neighbor_ip_list(tuple_n_t*, nu_ip_t);
PUBLIC void tuple_n_set_orig_addr(tuple_n_t*, nu_ip_t);
PUBLIC void tuple_n_set_symmetric(tuple_n_t*, nu_bool_t);
PUBLIC void tuple_n_set_willingness(tuple_n_t*, nu_bool_t);
PUBLIC void tuple_n_set_flooding_mpr(tuple_n_t*, nu_bool_t);
PUBLIC void tuple_n_set_routing_mpr(tuple_n_t*, nu_bool_t);
PUBLIC void tuple_n_set_mpr_selector(tuple_n_t*, nu_bool_t);
PUBLIC void tuple_n_set_advertised(tuple_n_t*, nu_bool_t);

////////////////////////////////////////////////////////////////
//
// ibase_n_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_n_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

#undef  ibase_n_clear_change_flag

/** Clears the change flag.
 */
#define ibase_n_clear_change_flag()  \
    do {                             \
        IBASE_N->change = false;     \
        IBASE_N->sym_change = false; \
    }                                \
    while (0)

#undef ibase_n_is_changed

/** Checks whether ibase_n is changed or not.
 */
#define ibase_n_is_changed()    (IBASE_N->sym_change || IBASE_N->change)

PUBLIC void ibase_n_init(void);
PUBLIC void ibase_n_destroy(void);

PUBLIC tuple_n_t* ibase_n_add(const nu_ip_set_t*, const nu_ip_t);
PUBLIC tuple_n_t* ibase_n_search_tuple_l(tuple_l_t*);
PUBLIC nu_bool_t ibase_n_contain_symmetric(const nu_ip_t);
PUBLIC nu_bool_t ibase_n_contain_symmetric_without_prefix(const nu_ip_t);
PUBLIC tuple_n_t* ibase_n_iter_remove(tuple_n_t*);
PUBLIC void ibase_n_save_advertised(void);
PUBLIC nu_bool_t ibase_n_commit_advertised(void);
PUBLIC void ibase_n_save_flooding_mpr(void);
PUBLIC void ibase_n_commit_flooding_mpr(void);
PUBLIC void ibase_n_save_routing_mpr(void);
PUBLIC void ibase_n_commit_routing_mpr(void);
PUBLIC void ibase_n_save_advertised(void);
PUBLIC nu_bool_t ibase_n_commit_advertised(void);

PUBLIC void tuple_n_put_log(tuple_n_t*, nu_logger_t*);
PUBLIC void ibase_n_put_log(nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
