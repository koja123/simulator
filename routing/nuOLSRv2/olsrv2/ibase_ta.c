//
// (TA) Routable Address Topology Set
//
#include "config.h"

// Define ORDERED_SET if you want to sort tuple_ta by dest_orig_addr,
// and from_orig_addr.
//#define ORDERED_SET

#include "core/core.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_ta
 */

////////////////////////////////////////////////////////////////
//
// tuple_ta_t
//

/*
 */
static void
tuple_ta_free(tuple_ta_t* tuple)
{
    ibase_time_cancel(&OLSR->ibase_ta_time_list, (tuple_time_t*)tuple);
    nu_mem_free(tuple);
}

/*
 */
static void
tuple_ta_timeout(tuple_time_t* tuple)
{
    ibase_ta_iter_remove((tuple_ta_t*)tuple);
}

/*
 */
static inline tuple_ta_t*
tuple_ta_create(const nu_ip_t from_orig_addr, const nu_ip_t dest_ip)
{
    tuple_ta_t* tuple = nu_mem_alloc(tuple_ta_t);
    tuple->next = tuple->prev = NULL;
    tuple->from_orig_addr = from_orig_addr;
    tuple->dest_ip = dest_ip;
    tuple->metric  = DEFAULT_METRIC;

    tuple->timeout_proc = tuple_ta_timeout;
    tuple_time_init((tuple_time_t*)tuple, &OLSR->ibase_ta_time_list, 0);

    return tuple;
}

////////////////////////////////////////////////////////////////
//
// ibase_ta_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_ta_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Initializes the ibase_ta.
 */
PUBLIC void
ibase_ta_init(void)
{
    _ibase_ta_init(IBASE_TA);
    IBASE_TA->change = false;
}

/** Destroys the ibase_ta.
 */
PUBLIC void
ibase_ta_destroy(void)
{
    _ibase_ta_destroy(IBASE_TA);
}

#ifndef ORDERED_SET

/** Gets the tuple.
 *
 * @param from_orig_addr
 * @param dest_ip
 * @return the tuple
 */
PUBLIC tuple_ta_t*
ibase_ta_get(const nu_ip_t from_orig_addr, const nu_ip_t dest_ip)
{
    FOREACH_TA(tuple_ta) {
        if (nu_ip_eq(dest_ip, tuple_ta->dest_ip) &&
            nu_orig_addr_eq(from_orig_addr, tuple_ta->from_orig_addr)) {
            return tuple_ta;
        }
    }
    tuple_ta_t* new_tuple = tuple_ta_create(from_orig_addr, dest_ip);
    _ibase_ta_insert_tail(IBASE_TA, new_tuple);
    IBASE_TA->change = true;
    return new_tuple;
}

#else

/*
 */
static tuple_ta_t*
ibase_ta_cmp(const nu_ip_t from_orig_addr, const nu_ip_t dest_ip,
        nu_bool_t* eq)
{
    FOREACH_TA(p) {
        int c;
        if ((c = nu_ip_cmp(dest_ip, p->dest_ip)) > 0) {
            *eq = false;
            return p;
        }
        if (c == 0) {
            int c2;
            if ((c2 = nu_orig_addr_cmp(from_orig_addr, p->from_orig_addr)) > 0) {
                *eq = false;
                return p;
            } else if (c2 == 0) {
                *eq = true;
                return p;
            }
        }
    }
    *eq = false;
    return ibase_ta_iter_end();
}

/**
 */
PUBLIC tuple_ta_t*
ibase_ta_get(const nu_ip_t from_orig_addr, const nu_ip_t dest_ip)
{
    nu_bool_t   eq = false;
    tuple_ta_t* tuple_ta = ibase_ta_cmp(from_orig_addr, dest_ip, &eq);
    if (eq)
        return tuple_ta;
    tuple_ta_t* new_tuple = tuple_ta_create(from_orig_addr, dest_ip);
    _ibase_ta_iter_insert_before(tuple_ta, IBASE_TA, new_tuple);
    IBASE_TA->change = true;
    return new_tuple;
}

#endif

/** Remove the tuple.
 *
 * @param tuple
 * @return the next tuple
 */
PUBLIC tuple_ta_t*
ibase_ta_iter_remove(tuple_ta_t* tuple)
{
    IBASE_TA->change = true;
    return _ibase_ta_iter_remove(tuple, IBASE_TA);
}

/** Removes tuples which contains the orig_addr in TR_from_orig_addr.
 *
 * @param orig_addr
 */
PUBLIC void
ibase_ta_remove_orig_addr(const nu_ip_t orig_addr)
{
    for (tuple_ta_t* tuple_ta = ibase_ta_iter();
         !ibase_ta_iter_is_end(tuple_ta);) {
        if (nu_orig_addr_eq(orig_addr, tuple_ta->from_orig_addr))
            tuple_ta = ibase_ta_iter_remove(tuple_ta);
        else
            tuple_ta = ibase_ta_iter_next(tuple_ta);
    }
}

/** Outputs tuple_ta.
 *
 * @param tuple
 * @param logger
 */
PUBLIC void
tuple_ta_put_log(tuple_ta_t* tuple, nu_logger_t* logger)
{
    nu_logger_log(logger,
            "I_TA:[t=%T from:%I dest:%I seq:%u metric:%g]",
            &tuple->time, tuple->from_orig_addr, tuple->dest_ip, tuple->seqnum, tuple->metric);
}

/** Outputs ibase_ta.
 *
 * @param logger
 */
PUBLIC void
ibase_ta_put_log(nu_logger_t* logger)
{
    nu_logger_log(logger, "I_TA:%d:--", ibase_ta_size());
    FOREACH_TA(p) {
        tuple_ta_put_log(p, logger);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
