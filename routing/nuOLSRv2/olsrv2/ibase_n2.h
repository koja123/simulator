//
// (N2) 2-Hop Set
//
#ifndef OLSRV2_IBASE_N2_H_
#define OLSRV2_IBASE_N2_H_

#include "core/ip.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/*************************************************************//**
 * @defgroup ibase_n2 OLSRv2 :: (N2) 2-Hop Tuple
 * @{
 */

/**
 * (N2) 2-Hop Tuple
 *
 * @see tuple_ip_t
 */
typedef struct tuple_n2 {
    struct tuple_n2*  next;         ///< next tuple
    struct tuple_n2*  prev;         ///< prev tuple

    tuple_time_t*     timeout_next; ///< next in timeout list
    tuple_time_t*     timeout_prev; ///< prev in timeout list
    tuple_time_proc_t timeout_proc; ///< timeout processor
    nu_time_t         time;         ///< timeout time
    struct ibase_n2*  ibase;        ///< ibase
    ibase_time_t*     time_list;    ///< timeout list

    nu_ip_t           hop2_ip;      ///< N2_2hop_addr

    nu_link_metric_t  in_metric;    ///< N2_in_metric
    nu_link_metric_t  out_metric;   ///< N2_out_metric

    struct tuple_l*   tuple_l;      ///< corresponding tuple_l
} tuple_n2_t;

/**
 * (N2) 2-Hop Set
 */
typedef struct ibase_n2 {
    tuple_n2_t* next; ///< first tuple
    tuple_n2_t* prev; ///< last tuple
    size_t      n;    ///< size
} ibase_n2_t;

////////////////////////////////////////////////////////////////
//
// tuple_n2
//

PUBLIC void tuple_n2_set_in_metric(tuple_n2_t*, nu_link_metric_t);
PUBLIC void tuple_n2_set_out_metric(tuple_n2_t*, nu_link_metric_t);

////////////////////////////////////////////////////////////////
//
// ibase_n2
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_n2_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Clears the change flag of the ibase_n2.
 */
#define ibase_n2_clear_change_flag()      \
    do { OLSR->ibase_n2_change = false; } \
    while (0)

/** Checks whether the ibase_n2 has been changed.
 *
 * @return true if at least one of the  ibase_n2s has been changed.
 */
#define ibase_n2_is_changed() \
    (OLSR->ibase_n2_change)

/** Sets the change flag of the ibase_n2.
 */
#define ibase_n2_change()                \
    do { OLSR->ibase_n2_change = true; } \
    while (0)

PUBLIC void ibase_n2_init(ibase_n2_t*);
PUBLIC void ibase_n2_destroy(ibase_n2_t*);

PUBLIC tuple_n2_t* ibase_n2_search(ibase_n2_t*, const nu_ip_t);

PUBLIC tuple_n2_t* ibase_n2_iter_remove(tuple_n2_t*, ibase_n2_t*);

PUBLIC tuple_n2_t* ibase_n2_add(const nu_ip_t, struct tuple_l*);
PUBLIC void ibase_n2_remove_all(ibase_n2_t*);

struct tuple_i;
struct tuple_l;
PUBLIC void ibase_n2_put_log(ibase_n2_t*, const struct tuple_i*,
        struct tuple_l*, nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
