//
// (R) Routing Set
//
#include "config.h"

#include "olsrv2/olsrv2.h"
#include "netio/route.h"

#define R_REMOVE    'R'
#define R_NEW       'N'
#define R_UPDATE    'U'

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_r
 */

////////////////////////////////////////////////////////////////
//
// tuple_r_t
//

/*
 */
static inline tuple_r_t*
tuple_r_create(const nu_ip_t dest, const nu_ip_t next,
        const nu_link_metric_t metric, const uint8_t dist,
        const tuple_i_t* tuple_i)
{
    tuple_r_t* tuple = nu_mem_alloc(tuple_r_t);
    tuple->next = tuple->prev = NULL;
    tuple->dest_ip = dest;
    tuple->next_ip = next;
    tuple->metric  = metric;
    tuple->dist = dist;
    tuple->tuple_i = tuple_i;
    tuple->mark = R_NEW;
    return tuple;
}

/*
 */
static inline void
tuple_r_free(tuple_r_t* tuple)
{
    nu_mem_free(tuple);
}

////////////////////////////////////////////////////////////////
//
// ibase_r_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_r_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Initializes the ibase_r.
 */
PUBLIC void
ibase_r_init(void)
{
    _ibase_r_init(IBASE_R);
}

/** Destroys the ibase_r.
 */
PUBLIC void
ibase_r_destroy(void)
{
    FOREACH_R(t) {
        nu_route_delete(t->dest_ip, t->next_ip,
                tuple_i_local_ip(t->tuple_i),
                t->dist, t->tuple_i->name);
    }
    _ibase_r_destroy(IBASE_R);
}

/** Adds new tuple.
 *
 * @param dest
 * @param next
 * @param metric
 * @param dist
 * @param tuple_i
 * @return the new tuple
 */
PUBLIC tuple_r_t*
ibase_r_add(const nu_ip_t dest, const nu_ip_t next,
        const nu_link_metric_t metric, const uint8_t dist,
        const tuple_i_t* tuple_i)
{
    FOREACH_R(tuple_r) {
        if (nu_ip_eq(dest, tuple_r->dest_ip) &&
            nu_ip_eq(next, tuple_r->next_ip) &&
            dist == tuple_r->dist &&
            tuple_i == tuple_r->tuple_i) {
            if (tuple_r->mark == R_REMOVE) {
                tuple_r->mark = R_UPDATE;
                olsrv2_ip_attr_t* ipa = olsrv2_ip_attr_get(dest);
                assert(ipa->tuple_r_dest == NULL);
                ipa->tuple_r_dest = tuple_r;
            }
            tuple_r->metric = metric;
            OLSRV2_DO_LOG(calc_route,
                    nu_logger_push_prefix(NU_LOGGER, "U:");
                    tuple_r_put_log(tuple_r, NU_LOGGER);
                    nu_logger_pop_prefix(NU_LOGGER);
                    );
            return tuple_r;
        }
    }
    tuple_r_t* new_tuple  = tuple_r_create(dest, next, metric, dist, tuple_i);
    olsrv2_ip_attr_t* ipa = olsrv2_ip_attr_get(dest);
    if (ipa->tuple_r_dest != NULL) {
        ibase_r_put_log(NU_LOGGER);
        tuple_r_put_log(new_tuple, NU_LOGGER);
        abort();
    }
    ipa->tuple_r_dest = new_tuple;
    _ibase_r_insert_tail(IBASE_R, new_tuple);

    OLSRV2_DO_LOG(calc_route,
            nu_logger_push_prefix(NU_LOGGER, "N:");
            tuple_r_put_log(new_tuple, NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );
    return new_tuple;
}

/** Checks the ibase_r contains a tuple.
 *
 * @param dest
 * @param next
 * @param dist
 * @param tuple_i
 * @return true if the ibase_r contains the tuple.
 */
PUBLIC nu_bool_t
ibase_r_peek(const nu_ip_t dest, const nu_ip_t next,
        const uint8_t dist, const tuple_i_t* tuple_i)
{
    FOREACH_R(tuple_r) {
        if (nu_ip_eq(dest, tuple_r->dest_ip) &&
            nu_ip_eq(next, tuple_r->next_ip) &&
            dist == tuple_r->dist &&
            tuple_i == tuple_r->tuple_i) {
            return true;
        }
    }
    return false;
}

/** Adds remove mark to the tuple.
 *
 * @param tuple
 */
PUBLIC void
ibase_r_delete(tuple_r_t* tuple)
{
    olsrv2_ip_attr_t* ipa = olsrv2_ip_attr_get(tuple->dest_ip);
    assert(ipa->tuple_r_dest == tuple);
    ipa->tuple_r_dest = NULL;
    if (tuple->mark == R_NEW)
        _ibase_r_iter_remove(tuple, IBASE_R);
    else
        tuple->mark = R_REMOVE;
}

/*
 */
static inline tuple_r_t*
ibase_r_iter_remove(tuple_r_t* tuple)
{
    return _ibase_r_iter_remove(tuple, IBASE_R);
}

/** Checks whether the ibase_r contains the tuple for the destination.
 *
 * @param dest
 * @return true if the ibase_r contains the tuple.
 */
PUBLIC nu_bool_t
ibase_r_contain_dest(const nu_ip_t dest)
{
    FOREACH_R(tuple_r) {
        if (tuple_r->mark != R_REMOVE &&
            nu_ip_eq(dest, tuple_r->dest_ip))
            return true;
    }
    return false;
}

/** Searches the tuple for the destination.
 *
 * @param dest
 * @return the tuple or NULL
 */
PUBLIC tuple_r_t*
ibase_r_search_dest(const nu_ip_t dest)
{
    return olsrv2_ip_attr_get(dest)->tuple_r_dest;
#if 0
    FOREACH_R(tuple_r) {
        if (tuple_r->mark != R_REMOVE &&
            nu_ip_eq(dest, tuple_r->dest_ip))
            return tuple_r;
    }
    return NULL;
#endif
}

/** Searches the tuple.
 *
 * @param dest
 * @param dist
 * @return the tuple or NULL
 */
PUBLIC tuple_r_t*
ibase_r_search_dest_and_dist(const nu_ip_t dest,
        const uint8_t dist)
{
    tuple_r_t* tuple_r = olsrv2_ip_attr_get(dest)->tuple_r_dest;
    if (tuple_r == NULL)
        return NULL;
    if (tuple_r->dist != tuple_r->dist)
        return NULL;
    return tuple_r;
#if 0
    FOREACH_R(tuple_r) {
        if (tuple_r->mark != R_REMOVE &&
            nu_ip_eq(dest, tuple_r->dest_ip) &&
            dist == tuple_r->dist)
            return tuple_r;
    }
    return NULL;
#endif
}

/** Searches the tuple.
 *
 * @param ip_set
 * @return the tuple or NULL
 */
PUBLIC tuple_r_t*
ibase_r_search_dest_ip_set(const nu_ip_set_t* ip_set)
{
    FOREACH_IP_SET(i, ip_set) {
        tuple_r_t* tuple_r;
        if ((tuple_r = ibase_r_search_dest(i->ip)) != NULL)
            return tuple_r;
    }
    return NULL;
}

/** Resets the marks of all the tuples.
 */
PUBLIC void
ibase_r_reset(void)
{
    FOREACH_R(tuple_r) {
        tuple_r->mark = R_REMOVE;
        olsrv2_ip_attr_t* ipa = olsrv2_ip_attr_get(tuple_r->dest_ip);
        assert(ipa->tuple_r_dest == NULL || ipa->tuple_r_dest == tuple_r);
        ipa->tuple_r_dest = NULL;
    }
}

/** Updates the routing table.
 */
PUBLIC void
ibase_r_update(void)
{
    for (tuple_r_t* r = ibase_r_iter();
         !ibase_r_iter_is_end(r);) {
        if (r->mark == R_REMOVE) {
            nu_route_delete(r->dest_ip, r->next_ip,
                    tuple_i_local_ip(r->tuple_i),
                    r->dist, r->tuple_i->name);
            r = ibase_r_iter_remove(r);
            IBASE_R->change = true;
        } else
            r = ibase_r_iter_next(r);
    }

    FOREACH_R(r) {
        if (r->mark == R_NEW) {
            nu_route_add(r->dest_ip, r->next_ip,
                    tuple_i_local_ip(r->tuple_i),
                    r->dist, r->tuple_i->name);
            IBASE_R->change = true;
        }
    }
}

/** Outputs tuple_r.
 *
 * @param tuple_r
 * @param logger
 */
PUBLIC void
tuple_r_put_log(tuple_r_t* tuple_r, nu_logger_t* logger)
{
    nu_logger_log(logger,
            "I_R:[dest:%I next:%I dist:%u iface:%s metric:%g mark:%c]",
            tuple_r->dest_ip, tuple_r->next_ip,
            tuple_r->dist, tuple_r->tuple_i->name, tuple_r->metric,
            tuple_r->mark);
}

/** Outputs ibase_r.
 *
 * @param logger
 */
PUBLIC void
ibase_r_put_log(nu_logger_t* logger)
{
    nu_logger_log(logger, "I_R:%d:--", ibase_r_size());
    FOREACH_R(p) {
        tuple_r_put_log(p, logger);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
