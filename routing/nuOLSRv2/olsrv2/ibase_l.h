//
// (L) Link Set
//
#ifndef OLSRV2_IBASE_L_H_
#define OLSRV2_IBASE_L_H_

#include "core/ip_set.h"
#include "core/scheduler.h"
#include "olsrv2/ibase_time.h"

#include "olsrv2/etx.h"
#include "olsrv2/lifo.h"

namespace NuOLSRv2Port { //ScenSim-Port://

struct tuple_i;
struct tuple_n;
struct ibase_l;

/************************************************************//**
 * @defgroup ibase_l OLSRv2 :: (L) Link Tuple
 * @{
 */

/**
 * (L) Link Tuple
 *
 * This implementation is different from the definition of draft.
 * tuple_l_t contains the list of tuple_n2_t which has same
 * neighbor_ip_list.
 */
typedef struct tuple_l {
    struct tuple_l*   next;             ///< next tuple
    struct tuple_l*   prev;             ///< prev tuple

    tuple_time_t*     timeout_next;     ///< next in timeout list
    tuple_time_t*     timeout_prev;     ///< prev in timeout list
    tuple_time_proc_t timeout_proc;     ///< timeout processor
    nu_time_t         time;             ///< timeout time
    struct ibase_l*   ibase;            ///< ibase

    nu_ip_set_t       neighbor_ip_list; ///< L_neighbor_ifaddr_list
    nu_time_t         HEARD_time;       ///< L_HEARD_time
    nu_time_t         SYM_time;         ///< L_SYM_time
    float             quality;          ///< L_quality
    nu_bool_t         pending;          ///< L_pending
    nu_bool_t         lost;             ///< L_lost

    uint8_t           willingness;      ///< L_willingness
    uint8_t           status;           ///< L_status
    uint8_t           prev_status;      ///< previous link status

    nu_link_metric_t  in_metric;        ///< L_in_metric
    nu_link_metric_t  out_metric;       ///< L_out_metric
    nu_bool_t         mpr_selector;     ///< L_mpr_selector

    // ETX
    lifo_t            metric_received_lifo;       ///< received lifo
    lifo_t            metric_total_lifo;          ///< total lifo
    int32_t           metric_last_pkt_seqnum;     ///< last packet seqnum
    double            metric_r_etx;               ///< r_etx
    double            metric_d_etx;               ///< d_etx
    double            metric_hello_interval;      ///< hello interval
    int               metric_lost_hellos;         ///< the # of lost hellos
    nu_event_t*       metric_hello_timeout_event; ///< hello timeout event

    ibase_n2_t        ibase_n2;                   ///< ibase_n2
    struct tuple_i*   tuple_i;                    ///< corresponding tuple_i
    struct tuple_n*   tuple_n;                    ///< corresponding tuple_n

    nu_event_t*       lq_event;                   ///< link quality event list
} tuple_l_t;

/**
 * (L) Link Set
 */
typedef struct ibase_l {
    tuple_l_t* next; ///< first tuple
    tuple_l_t* prev; ///< last tuple
    size_t     n;    ///< size
} ibase_l_t;

////////////////////////////////////////////////////////////////
//
// tuple_l_t
//

/** Sets timeout time
 *
 * @param tuple
 * @param t
 */
#define tuple_l_set_time(tuple, t) \
    tuple_time_set_time((tuple_time_t*)(tuple), &OLSR->ibase_l_time_list, (t))

PUBLIC void tuple_l_remove_ip_set(tuple_l_t*, const nu_ip_set_t*);
PUBLIC void tuple_l_update_status(tuple_l_t*);

PUBLIC void tuple_l_quality_up(tuple_l_t*);
PUBLIC void tuple_l_quality_down(nu_event_t*);

PUBLIC nu_ip_t tuple_l_selected_local_address(tuple_l_t*);

PUBLIC void tuple_l_set_mpr_selector(tuple_l_t*, nu_bool_t);

PUBLIC void tuple_l_set_in_metric(tuple_l_t*, nu_link_metric_t);
PUBLIC void tuple_l_set_out_metric(tuple_l_t*, nu_link_metric_t);

////////////////////////////////////////////////////////////////
//
// ibase_l_t
//

PUBLIC tuple_l_t* ibase_l_iter_remove(tuple_l_t*, ibase_l_t*);

/** Redefine ibase_n_clear_change_flag */
#define ibase_l_clear_change_flag()           \
    do { OLSR->ibase_l_change = false;        \
         OLSR->ibase_l_sym_change = false;  } \
    while (0)

/** Checks whether the ibase has been changed.
 *
 * @return true if ibase has been changed.
 */
#define ibase_l_is_changed() \
    (OLSR->ibase_l_change || OLSR->ibase_l_sym_change)

/** Checks whether the ibase has been changed.
 *
 * @return true if ibase has been changed.
 */
#define ibase_l_sym_is_changed()    (OLSR->ibase_l_sym_change)

/** Sets the change flag of the ibase.
 */
#define ibase_l_change()                \
    do { OLSR->ibase_l_change = true; } \
    while (0)

/** Sets the change flag of the ibase.
 */
#define ibase_l_sym_change()                \
    do { OLSR->ibase_l_sym_change = true; } \
    while (0)

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_l_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC void ibase_l_init(ibase_l_t*);
PUBLIC void ibase_l_destroy(ibase_l_t*);

PUBLIC tuple_l_t* ibase_l_add(ibase_l_t*, struct tuple_i*, struct tuple_n*);
PUBLIC tuple_l_t* ibase_l_search_neighbor_ip(ibase_l_t*, nu_ip_t);
PUBLIC tuple_l_t* ibase_l_search_neighbor_ip_without_prefix(ibase_l_t*,
        nu_ip_t);

PUBLIC void ibase_l_clear_status(void);
PUBLIC void ibase_l_update_status(void);

/** Get selected tuple_l for tuple_n */
PUBLIC tuple_l_t* ibase_l_search_tuple_n(const struct tuple_n*);

PUBLIC void tuple_l_put_log(tuple_l_t*, nu_logger_t*);
PUBLIC void ibase_l_put_log(ibase_l_t*, struct tuple_i*, nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
