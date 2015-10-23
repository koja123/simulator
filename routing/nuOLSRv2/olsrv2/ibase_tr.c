//
// (T) Router Topology Set
//
#include "config.h"

// Define ORDERED_SET if you want to sort tuple_tr by dest_ip, and orig_addr.
#define ORDERED_SET

#include "core/core.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_tr
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tuple_tr_t
//

/*
 */
static void
tuple_tr_free(tuple_tr_t* tuple)
{
    ibase_time_cancel(&OLSR->ibase_tr_time_list, (tuple_time_t*)tuple);
    nu_mem_free(tuple);
}

/*
 */
static void
tuple_tr_timeout(tuple_time_t* tuple)
{
    ibase_tr_iter_remove((tuple_tr_t*)tuple);
}

/*
 */
static inline tuple_tr_t*
tuple_tr_create(const nu_ip_t from_orig_addr, const nu_ip_t to_orig_addr)
{
    tuple_tr_t* tuple = nu_mem_alloc(tuple_tr_t);
    tuple->next = tuple->prev = NULL;
    tuple->from_orig_addr = from_orig_addr;
    tuple->to_orig_addr = to_orig_addr;
    tuple->seqnum = 0;
    tuple->metric = DEFAULT_METRIC;

    tuple->timeout_proc = tuple_tr_timeout;
    tuple_time_init((tuple_time_t*)tuple, &OLSR->ibase_tr_time_list, 0);

    return tuple;
}

////////////////////////////////////////////////////////////////
//
// ibase_tr_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_tr_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Initializes the ibase_tr.
 */
PUBLIC void
ibase_tr_init(void)
{
    _ibase_tr_init(IBASE_TR);
    IBASE_TR->change = false;
}

/** Destroys the ibase_tr.
 */
PUBLIC void
ibase_tr_destroy(void)
{
    _ibase_tr_destroy(IBASE_TR);
}

#ifndef ORDERED_SET
#if 0

/** Gets the tuple.
 *
 * @param from_orig_addr
 * @param to_orig_addr
 * @return the tuple
 */
PUBLIC tuple_tr_t*
ibase_tr_get(const nu_ip_t from_orig_addr, const nu_ip_t to_orig_addr)
{
    FOREACH_TR(tuple_tr) {
        if (nu_orig_addr_eq(from_orig_addr, tuple_tr->from_orig_addr) &&
            nu_orig_addr_eq(to_orig_addr, tuple_tr->to_orig_addr)) {
            return tuple_tr;
        }
    }
    tuple_tr_t* new_tuple = tuple_tr_create(from_orig_addr, to_orig_addr);
    _ibase_tr_insert_tail(IBASE_TR, new_tuple);
    IBASE_TR->changed = true;
    return new_tuple;
}

#endif
#else

/*
 */
static tuple_tr_t*
ibase_tr_cmp(const nu_ip_t from_orig_addr,
        const nu_ip_t to_orig_addr, nu_bool_t* eq)
{
    FOREACH_TR(p) {
        int c;
        if ((c = nu_orig_addr_cmp(from_orig_addr, p->from_orig_addr)) > 0) {
            *eq = false;
            return p;
        }
        if (c == 0) {
            int c2;
            if ((c2 = nu_orig_addr_cmp(to_orig_addr, p->to_orig_addr)) > 0) {
                *eq = false;
                return p;
            } else if (c2 == 0) {
                *eq = true;
                return p;
            }
        }
    }
    *eq = false;
    return ibase_tr_iter_end();
}

/** Gets the tuple.
 *
 * @param from_orig_addr
 * @param to_orig_addr
 * @return the tuple
 */
PUBLIC tuple_tr_t*
ibase_tr_get(const nu_ip_t from_orig_addr, const nu_ip_t to_orig_addr)
{
    nu_bool_t   eq = false;
    tuple_tr_t* tuple_tr = ibase_tr_cmp(from_orig_addr, to_orig_addr, &eq);
    if (eq)
        return tuple_tr;
    tuple_tr_t* new_tuple = tuple_tr_create(from_orig_addr, to_orig_addr);
    _ibase_tr_iter_insert_before(tuple_tr, IBASE_TR, new_tuple);
    IBASE_TR->change = true;
    return new_tuple;
}

#endif

/** Removes the tuple.
 *
 * @param tuple
 * @return the next tuple
 */
PUBLIC tuple_tr_t*
ibase_tr_iter_remove(tuple_tr_t* tuple)
{
    IBASE_TR->change = true;
    return _ibase_tr_iter_remove(tuple, IBASE_TR);
}

/** Removes tuples which contains the orig_addr in TR_from_orig_addr.
 *
 * @param orig_addr
 */
PUBLIC void
ibase_tr_remove_orig_addr(const nu_ip_t orig_addr)
{
    for (tuple_tr_t* tuple_tr = ibase_tr_iter();
         !ibase_tr_iter_is_end(tuple_tr);) {
        if (nu_orig_addr_eq(orig_addr, tuple_tr->from_orig_addr))
            tuple_tr = ibase_tr_iter_remove(tuple_tr);
        else
            tuple_tr = ibase_tr_iter_next(tuple_tr);
    }
}

/** Outputs ibase_tr.
 *
 * @param logger
 */
PUBLIC void
ibase_tr_put_log(nu_logger_t* logger)
{
    nu_logger_log(logger, "I_TR:%d:--", ibase_tr_size());
    FOREACH_TR(p) {
        nu_logger_log(logger,
                "I_TR:[t=%T from:%I to:%I seq:%u metric:%g]",
                &p->time, p->from_orig_addr, p->to_orig_addr, p->seqnum, p->metric);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
