#include "config.h"

#include "core/core.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_n2
 */

////////////////////////////////////////////////////////////////
//
// tuple_n2_t
//

/*
 */
static void
tuple_n2_timeout(tuple_time_t* tuple)
{
    tuple_n2_t* tuple_n2 = (tuple_n2_t*)tuple;
    ibase_n2_iter_remove(tuple_n2, tuple_n2->ibase);
}

/*
 */
static tuple_n2_t*
tuple_n2_create(const nu_ip_t ip, tuple_l_t* tuple_l)
{
    tuple_n2_t* tuple = nu_mem_alloc(tuple_n2_t);
    tuple->next = tuple->prev = NULL;
    tuple->hop2_ip = ip;
    tuple->in_metric = tuple->out_metric = UNDEF_METRIC;
    tuple->tuple_l = tuple_l;

    tuple->ibase = &tuple_l->ibase_n2;
    tuple->timeout_proc = tuple_n2_timeout;
    tuple_time_init((tuple_time_t*)tuple, &OLSR->ibase_n2_time_list, -1);

    return tuple;
}

static inline void
tuple_n2_free(tuple_n2_t* tuple)
{
    ibase_time_cancel(&OLSR->ibase_n2_time_list, (tuple_time_t*)tuple);
    nu_mem_free(tuple);
}

/** Sets the in_metric of tuple_n2.
 *
 * @param tuple_n2
 * @param in_metric
 */
PUBLIC void
tuple_n2_set_in_metric(tuple_n2_t* tuple_n2, nu_link_metric_t in_metric)
{
    if (OLSR->update_by_metric) {
        if (tuple_n2->in_metric != in_metric) {
            tuple_n2->in_metric = in_metric;
            ibase_n2_change();
        }
    } else {
        tuple_n2->in_metric = in_metric;
    }
}

/** Sets the out_metric of tuple_n2.
 *
 * @param tuple_n2
 * @param out_metric
 */
PUBLIC void
tuple_n2_set_out_metric(tuple_n2_t* tuple_n2, nu_link_metric_t out_metric)
{
    if (OLSR->update_by_metric) {
        if (tuple_n2->out_metric != out_metric) {
            tuple_n2->out_metric = out_metric;
            ibase_n2_change();
        }
    } else {
        tuple_n2->out_metric = out_metric;
    }
}

////////////////////////////////////////////////////////////////
//
// ibase_n2_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_n2_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Initializes the ibase_n2.
 *
 * @param ibase_n2
 */
PUBLIC void
ibase_n2_init(ibase_n2_t* ibase_n2)
{
    _ibase_n2_init(ibase_n2);
}

/** Destroys the ibase_n2.
 *
 * @param ibase_n2
 */
PUBLIC void
ibase_n2_destroy(ibase_n2_t* ibase_n2)
{
    while (!ibase_n2_is_empty(ibase_n2)) {
        ibase_n2_iter_remove(ibase_n2_head(ibase_n2), ibase_n2);
    }
}

static tuple_n2_t*
ibase_n2_cmp(ibase_n2_t* ibase, nu_ip_t h2, nu_bool_t* eq)
{
    FOREACH_N2(p, ibase) {
        int c;
        if ((c = nu_ip_cmp(h2, p->hop2_ip)) > 0) {
            *eq = false;
            return p;
        }
        if (c == 0) {
            *eq = true;
            return p;
        }
    }
    *eq = false;
    return ibase_n2_iter_end(ibase);
}

/** Adds the ip to ibase_n2.
 *
 * @param ip
 * @param tuple_l
 * @return the new tuple
 */
PUBLIC tuple_n2_t*
ibase_n2_add(const nu_ip_t ip, tuple_l_t* tuple_l)
{
    nu_bool_t   eq = false;
    tuple_n2_t* tuple = ibase_n2_cmp(&tuple_l->ibase_n2, ip, &eq);
    if (eq)
        return tuple;
    tuple_n2_t* new_tuple = tuple_n2_create(ip, tuple_l);
    _ibase_n2_iter_insert_before(tuple, &tuple_l->ibase_n2, new_tuple);
    ibase_n2_change();
    return new_tuple;
}

/** Searchs the ip.
 *
 * @param ibase
 * @param ip
 * @return the tuple or NULL.
 */
PUBLIC tuple_n2_t*
ibase_n2_search(ibase_n2_t* ibase, const nu_ip_t ip)
{
    FOREACH_IBASE_N2(p, ibase) {
        if (nu_ip_eq(ip, p->hop2_ip))
            return p;
    }
    return NULL;
}

/** Removes the tuple.
 *
 * @param tuple
 * @param ibase_n2
 * @return the next tuple
 */
PUBLIC tuple_n2_t*
ibase_n2_iter_remove(tuple_n2_t* tuple, ibase_n2_t* ibase_n2)
{
    ibase_n2_change();
    return _ibase_n2_iter_remove((tuple_n2_t*)tuple, ibase_n2);
}

/** Removes all the tuples.
 *
 *  @param ibase_n2
 */
PUBLIC void
ibase_n2_remove_all(ibase_n2_t* ibase_n2)
{
#if 0                                      //ScenSim-Port://
    if (ibase_n2_is_empty(ibase_n2) > 0) {
#else                                      //ScenSim-Port://
    if (!ibase_n2_is_empty(ibase_n2)) {    //ScenSim-Port://
#endif                                     //ScenSim-Port://
        _ibase_n2_remove_all(ibase_n2);
        ibase_n2_change();
    }
}

/** Outputs ibase_n2.
 *
 * @param ibase_n2
 * @param tuple_i
 * @param tuple_l
 * @param logger
 */
PUBLIC void
ibase_n2_put_log(ibase_n2_t* ibase_n2, const struct tuple_i* tuple_i,
        struct tuple_l* tuple_l, nu_logger_t* logger)
{
    nu_logger_log(logger, "I_N2:%s:%S:--",
            tuple_i->name, &tuple_l->neighbor_ip_list);
    FOREACH_N2(p, ibase_n2) {
        nu_logger_log(logger,
                "I_N2:%s:(%S):[t=%T hop2:%I in:%g out:%g]",
                tuple_i->name, &tuple_l->neighbor_ip_list,
                &p->time, p->hop2_ip, p->in_metric, p->out_metric);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
