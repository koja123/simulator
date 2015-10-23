//
// (AN) Attached Network Set
//
#include "config.h"

#include "core/core.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup @ibase_an
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tuple_an_t
//

static void
tuple_an_free(tuple_an_t* tuple)
{
    ibase_time_cancel(&OLSR->ibase_an_time_list, (tuple_time_t*)tuple);
    nu_mem_free(tuple);
}

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_an_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/*
 */
static void
tuple_an_timeout(tuple_time_t* tuple)
{
    tuple_an_t* tuple_an = (tuple_an_t*)tuple;
    ibase_an_iter_remove(tuple_an);
}

/*
 */
static inline tuple_an_t*
tuple_an_create(const nu_ip_t orig_addr, const uint16_t seqnum,
        const nu_ip_t net_addr, const uint8_t dist)
{
    tuple_an_t* tuple = nu_mem_alloc(tuple_an_t);
    tuple->next = tuple->prev = NULL;
    tuple->orig_addr = orig_addr;
    tuple->seqnum = seqnum;
    tuple->net_addr = net_addr;
    tuple->dist = dist;
    tuple->metric = UNDEF_METRIC;

    tuple->timeout_proc = tuple_an_timeout;
    tuple_time_init((tuple_time_t*)tuple, &OLSR->ibase_an_time_list, -1);

    return tuple;
}

////////////////////////////////////////////////////////////////
//
// ibase_an_t
//

/** Initializes the ibase_an.
 */
PUBLIC void
ibase_an_init(void)
{
    _ibase_an_init(IBASE_AN);
    IBASE_AN->change = false;
}

/** Destroys the ibase_an.
 */
PUBLIC void
ibase_an_destroy(void)
{
    _ibase_an_destroy(IBASE_AN);
}

/** Gets the tuple of ibase_an.
 *
 * @param orig_addr
 * @param seqnum
 * @param net_addr
 * @param dist
 * @return the tuple
 */
PUBLIC tuple_an_t*
ibase_an_get(const nu_ip_t orig_addr, const uint16_t seqnum,
        const nu_ip_t net_addr, const uint8_t dist)
{
    FOREACH_AN(tuple_an) {
        if (nu_ip_eq(net_addr, tuple_an->net_addr) &&
            nu_orig_addr_eq(orig_addr, tuple_an->orig_addr)) {
            if (tuple_an->seqnum != seqnum) {
                tuple_an->seqnum = seqnum;
                IBASE_AN->change = true;
            }
            if (tuple_an->dist != dist) {
                tuple_an->dist = dist;
                IBASE_AN->change = true;
            }
            return tuple_an;
        }
    }
    tuple_an_t* new_tuple = tuple_an_create(orig_addr, seqnum,
            net_addr, dist);
    _ibase_an_insert_tail(IBASE_AN, new_tuple);
    IBASE_AN->change = true;
    return new_tuple;
}

/** Removes the tuple.
 *
 * @param tuple
 * @return the next tuple
 */
PUBLIC tuple_an_t*
ibase_an_iter_remove(tuple_an_t* tuple)
{
    IBASE_AN->change = true;
    return _ibase_an_iter_remove(tuple, IBASE_AN);
}

/** Removes the tuple.
 *
 * @param orig_addr
 */
PUBLIC void
ibase_an_remove_orig_addr(const nu_ip_t orig_addr)
{
    for (tuple_an_t* tuple_an = ibase_an_iter();
         !ibase_an_iter_is_end(tuple_an);) {
        if (nu_ip_eq_without_prefix(orig_addr, tuple_an->orig_addr))
            tuple_an = ibase_an_iter_remove(tuple_an);
        else
            tuple_an = ibase_an_iter_next(tuple_an);
    }
}

/** Outputs ibase_an.
 *
 * @param logger
 */
PUBLIC void
ibase_an_put_log(nu_logger_t* logger)
{
    nu_logger_log(logger, "I_AN:%d:--", ibase_an_size());
    FOREACH_AN(p) {
        nu_logger_log(logger,
                "I_AN:[t=%T orig:%I seq:%u net:%I dist:%d]",
                &p->time, p->orig_addr, p->seqnum,
                p->net_addr, p->dist);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
