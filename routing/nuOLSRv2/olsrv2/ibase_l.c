//
// (L) Link Set
//
#include "config.h"

#include "core/core.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"

#include "olsrv2/ibase_time_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup @ibase_l
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tuple_l_t
//

/*
 */
static void
tuple_l_timeout(tuple_time_t* tuple)
{
    tuple_l_t* tuple_l = (tuple_l_t*)tuple;
    ibase_l_iter_remove(tuple_l, tuple_l->ibase);
}

/*
 */
static inline tuple_l_t*
tuple_l_create(ibase_l_t* ibase_l, tuple_i_t* tuple_i, tuple_n_t* tuple_n)
{
    tuple_l_t* tuple = nu_mem_alloc(tuple_l_t);
    tuple->next = tuple->prev = NULL;
    nu_ip_set_init(&tuple->neighbor_ip_list);
    nu_time_set_timeout(&tuple->HEARD_time, -1);
    nu_time_set_timeout(&tuple->SYM_time, -1);
#if defined(_WIN32) || defined(_WIN64)       //ScenSim-Port://
    tuple->quality = (float)INITIAL_QUALITY; //ScenSim-Port://
#else                                        //ScenSim-Port://
    tuple->quality = INITIAL_QUALITY;
#endif                                       //ScenSim-Port://
    tuple->pending = INITIAL_PENDING;
    tuple->lost = false;
    tuple->willingness = WILLINGNESS__DEFAULT;
    tuple->status = LINK_STATUS__UNKNOWN;
    tuple->prev_status = LINK_STATUS__UNKNOWN;

    tuple->in_metric = tuple->out_metric = UNDEF_METRIC;
    tuple->mpr_selector = false;

    ibase_n2_init(&tuple->ibase_n2);
    tuple->tuple_i = tuple_i;
    tuple->tuple_n = NULL;

    tuple->ibase = ibase_l;
    tuple->timeout_proc = tuple_l_timeout;
    tuple_time_init((tuple_time_t*)tuple, &OLSR->ibase_l_time_list, -1);

    tuple->lq_event = NULL;

    tuple->metric_hello_timeout_event = NULL;
    if (LINK_METRIC_TYPE == LINK_METRIC_TYPE__ETX) {
        lifo_init(&tuple->metric_received_lifo, ETX_MEMORY_LENGTH);
        lifo_init(&tuple->metric_total_lifo, ETX_MEMORY_LENGTH);
        tuple->metric_last_pkt_seqnum = UNDEF_SEQNUM;
        tuple->metric_r_etx = UNDEF_ETX;
        tuple->metric_d_etx = UNDEF_ETX;
        tuple->metric_hello_timeout_event = NULL;
        tuple->metric_hello_interval = UNDEF_INTERVAL;
        tuple->metric_lost_hellos = 0;
    }

    return tuple;
}

/*
 */
static inline void
tuple_l_free(tuple_l_t* tuple)
{
    if (tuple->lq_event)
        nu_event_list_cancel(LQ_EVENT_LIST, tuple->lq_event);
    if (tuple->metric_hello_timeout_event)
        nu_event_list_cancel(METRIC_EVENT_LIST, tuple->metric_hello_timeout_event);
    if (LINK_METRIC_TYPE == LINK_METRIC_TYPE__ETX) {
        lifo_destroy(&tuple->metric_received_lifo);
        lifo_destroy(&tuple->metric_total_lifo);
    }
    nu_ip_set_destroy(&tuple->neighbor_ip_list);
    ibase_n2_destroy(&tuple->ibase_n2);
    ibase_time_cancel(&OLSR->ibase_l_time_list, (tuple_time_t*)tuple);
    nu_mem_free(tuple);
}

/** Set mpr_selector.
 *
 * @param tuple
 * @param status
 */
PUBLIC void
tuple_l_set_mpr_selector(tuple_l_t* tuple, nu_bool_t status)
{
    if (tuple->mpr_selector != status) {
        tuple->mpr_selector = status;
        ibase_l_change();
#if 0
        if (tuple->status == LINK_STATUS__SYMMETRIC)
            ibase_l_sym_change();
#endif
    }
}

/** Updates the link status.
 *
 * @param tuple
 */
PUBLIC void
tuple_l_update_status(tuple_l_t* tuple)
{
    if (tuple->pending)
        tuple->status = LINK_STATUS__PENDING;
    else if (tuple->lost)
        tuple->status = LINK_STATUS__LOST;
    else if (nu_time_cmp(tuple->SYM_time, NU_NOW) >= 0)
        tuple->status = LINK_STATUS__SYMMETRIC;
    else if (nu_time_cmp(tuple->HEARD_time, NU_NOW) >= 0)
        tuple->status = LINK_STATUS__HEARD;
    else
        tuple->status = LINK_STATUS__LOST;

    if (tuple->status != tuple->prev_status) {
        ibase_l_change();
        if (tuple->status == LINK_STATUS__SYMMETRIC ||
            tuple->prev_status == LINK_STATUS__SYMMETRIC)
            ibase_l_sym_change();
    }
}

/** Removes the ips from the L_neighbor_ifaddr_list.
 *
 * @param tuple
 * @param ip_set
 */
PUBLIC void
tuple_l_remove_ip_set(tuple_l_t* tuple, const nu_ip_set_t* ip_set)
{
    const size_t prev_size = nu_ip_set_size(&tuple->neighbor_ip_list);
    nu_ip_set_remove_ip_set(&tuple->neighbor_ip_list, ip_set);
    if (prev_size != nu_ip_set_size(&tuple->neighbor_ip_list)) {
        ibase_l_change();
        if (tuple->status == LINK_STATUS__SYMMETRIC)
            ibase_l_sym_change();
    }
}

/** Gets the local address.
 *
 * @param tuple_l
 */
PUBLIC nu_ip_t
tuple_l_selected_local_address(tuple_l_t* tuple_l)
{
    return tuple_i_local_ip(tuple_l->tuple_i);
}

/** Decreases the link quality
 *
 * @param ev
 */
PUBLIC void
tuple_l_quality_down(nu_event_t* ev)
{
    tuple_l_t* tuple_l = (tuple_l_t*)ev->param;
#ifndef NU_NDEBUG
    double old_quality = tuple_l->quality;
#endif
#if defined(_WIN32) || defined(_WIN64)                                 //ScenSim-Port://
    tuple_l->quality = (float)((1.0 - HYST_SCALE) * tuple_l->quality); //ScenSim-Port://
#else                                                                  //ScenSim-Port://
    tuple_l->quality = (1.0 - HYST_SCALE) * tuple_l->quality;
#endif                                                                 //ScenSim-Port://
    if (tuple_l->status != LINK_STATUS__PENDING &&
        tuple_l->quality <= HYST_REJECT) {
        if (tuple_l->lost == false) {
            tuple_l->lost = true;
            nu_time_t t = NU_NOW;
            nu_time_add_f(&t, L_HOLD_TIME);
            if (nu_time_cmp(tuple_l->time, t) > 0)
                tuple_l_set_time(tuple_l, t);
        }
    }

    OLSRV2_DO_LOG(link_quality,
            nu_logger_log(NU_LOGGER, "LQ_DOWN:%f -> %f:",
                    old_quality, tuple_l->quality);
            tuple_l_put_log(tuple_l, NU_LOGGER);
            );
}

/** Increases the link quality.
 *
 * @param tuple_l
 */
PUBLIC void
tuple_l_quality_up(tuple_l_t* tuple_l)
{
#ifndef NU_NDEBUG
    double old_quality = tuple_l->quality;
#endif
#if defined(_WIN32) || defined(_WIN64)                                              //ScenSim-Port://
    tuple_l->quality = (float)((1.0 - HYST_SCALE) * tuple_l->quality + HYST_SCALE); //ScenSim-Port://
#else                                                                               //ScenSim-Port://
    tuple_l->quality = (1.0 - HYST_SCALE) * tuple_l->quality + HYST_SCALE;
#endif                                                                              //ScenSim-Port://
    if (tuple_l->quality >= HYST_ACCEPT) {
        tuple_l->pending = false;
        tuple_l->lost = false;
        tuple_l_update_status(tuple_l);
        if (tuple_l->status == LINK_STATUS__HEARD ||
            tuple_l->status == LINK_STATUS__SYMMETRIC) {
            nu_time_t t = tuple_l->HEARD_time;
            nu_time_add_f(&t, L_HOLD_TIME);
            if (nu_time_cmp(tuple_l->time, t) < 0)
                tuple_l_set_time(tuple_l, t);
        }
    }

    OLSRV2_DO_LOG(link_quality,
            nu_logger_log(NU_LOGGER, "LQ_UP:%f -> %f:",
                    old_quality, tuple_l->quality);
            tuple_l_put_log(tuple_l, NU_LOGGER);
            );
}

/** Updates L_in_metric.
 *
 * @param tuple_l
 * @param metric
 */
PUBLIC void
tuple_l_set_in_metric(tuple_l_t* tuple_l, nu_link_metric_t metric)
{
    if (tuple_l->in_metric != metric) {
        tuple_l->in_metric = metric;
        if (OLSR->update_by_metric) {
            ibase_l_change();
            if (tuple_l->status == LINK_STATUS__SYMMETRIC)
                ibase_l_sym_change();
        }
    }
}

/** Updates L_out_metric.
 *
 * @param tuple_l
 * @param metric
 */
PUBLIC void
tuple_l_set_out_metric(tuple_l_t* tuple_l, nu_link_metric_t metric)
{
    if (tuple_l->out_metric != metric) {
        tuple_l->out_metric = metric;
        if (OLSR->update_by_metric) {
            ibase_l_change();
            if (tuple_l->status == LINK_STATUS__SYMMETRIC)
                ibase_l_sym_change();
        }
    }
}

////////////////////////////////////////////////////////////////
//
// ibase_l_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_l_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Initializes the ibase_l.
 *
 * @param ibase_l
 */
PUBLIC void
ibase_l_init(ibase_l_t* ibase_l)
{
    _ibase_l_init(ibase_l);
}

/** Destroys the ibase_l.
 *
 * @param ibase_l
 */
PUBLIC void
ibase_l_destroy(ibase_l_t* ibase_l)
{
    while (!ibase_l_is_empty(ibase_l)) {
        ibase_l_iter_remove(ibase_l_head(ibase_l), ibase_l);
    }
}

/** Adds the new tuple.
 *
 * @param ibase_l
 * @param tuple_i
 * @param tuple_n
 * @return the new tuple
 */
PUBLIC tuple_l_t*
ibase_l_add(ibase_l_t* ibase_l, tuple_i_t* tuple_i, tuple_n_t* tuple_n)
{
    tuple_l_t* new_tuple = tuple_l_create(ibase_l, tuple_i, tuple_n);
    _ibase_l_insert_tail(ibase_l, new_tuple);
    ibase_l_change();
    return new_tuple;
}

/** Removes the tuple.
 *
 * @param tuple
 * @param ibase_l
 * @return the next tuple
 */
PUBLIC tuple_l_t*
ibase_l_iter_remove(tuple_l_t* tuple, ibase_l_t* ibase_l)
{
    ibase_l_change();
    if (tuple->status == LINK_STATUS__SYMMETRIC)
        ibase_l_sym_change();
    return _ibase_l_iter_remove(tuple, ibase_l);
}

/** Searches the tuple whose neighbor_ifaddr_list contains the ip.
 *
 * @param ibase_l
 * @param ip
 * @return the tuple or NULL.
 */
PUBLIC tuple_l_t*
ibase_l_search_neighbor_ip(ibase_l_t* ibase_l, nu_ip_t ip)
{
    FOREACH_L(tuple_l, ibase_l) {
        if (nu_ip_set_contain(&tuple_l->neighbor_ip_list, ip))
            return tuple_l;
    }
    return NULL;
}

/** Searches the tuple whose neighbor_ifaddr_list contains the ip.
 *
 * @param ibase_l
 * @param ip
 * @return the tuple or NULL.
 */
PUBLIC tuple_l_t*
ibase_l_search_neighbor_ip_without_prefix(ibase_l_t* ibase_l, nu_ip_t ip)
{
    FOREACH_L(tuple_l, ibase_l) {
        if (nu_ip_set_contain_without_prefix(&tuple_l->neighbor_ip_list, ip))
            return tuple_l;
    }
    return NULL;
}

/** Clears the link statuses.
 */
PUBLIC void
ibase_l_clear_status(void)
{
    FOREACH_I(tuple_i) {
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            tuple_l->prev_status = tuple_l->status;
        }
    }
}

/** Updates the link statuses.
 */
PUBLIC void
ibase_l_update_status(void)
{
    FOREACH_I(tuple_i) {
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            tuple_l_update_status(tuple_l);
        }
    }
}

/** Searches the corresponding link tuple.
 *
 * @param tuple_n
 * @return the tuple or NULL.
 */
PUBLIC tuple_l_t*
ibase_l_search_tuple_n(const tuple_n_t* tuple_n)
{
    FOREACH_I(tuple_i) {
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            if (tuple_l->tuple_n == tuple_n)
                return tuple_l;
        }
    }
    return NULL;
}

/** Outputs tuple_l.
 *
 * @param tuple
 * @param logger
 */
PUBLIC void
tuple_l_put_log(tuple_l_t* tuple, nu_logger_t* logger)
{
    char sts = '\0';
    switch (tuple->status) {
    case LINK_STATUS__LOST:
        sts = 'L';
        break;
    case LINK_STATUS__SYMMETRIC:
        sts = 'S';
        break;
    case LINK_STATUS__HEARD:
        sts = 'H';
        break;
    case LINK_STATUS__PENDING:
        sts = 'P';
        break;
    case LINK_STATUS__UNKNOWN:
        sts = 'U';
        break;
    default:
        nu_fatal("Unknown LINK_STATUS:%d", tuple->status);
    }
    nu_logger_log(logger,
            "I_L:%s:[t=%T heard:%T sym:%T q:%g sts:%c pending:%B in:%g out:%g mprs:%B neighb:(%S)]",
            tuple->tuple_i->name,
            &tuple->time, &tuple->HEARD_time, &tuple->SYM_time,
            tuple->quality, sts, tuple->pending,
            tuple->in_metric, tuple->out_metric, tuple->mpr_selector,
            &tuple->neighbor_ip_list);
}

/** Outputs ibase_l.
 *
 * @param ibase_l
 * @param tuple_i
 * @param logger
 */
PUBLIC void
ibase_l_put_log(ibase_l_t* ibase_l, tuple_i_t* tuple_i, nu_logger_t* logger)
{
    nu_logger_log(logger, "I_L:%s:%d:--",
            tuple_i->name, ibase_l_size(ibase_l));
    FOREACH_L(p, ibase_l) {
        tuple_l_put_log(p, logger);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
