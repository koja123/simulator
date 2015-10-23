//
// (AR) Advertising Remote Routing Set
//
#include "config.h"

#include "core/core.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup @ibase_ar
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tuple_ar_t
//

static void
tuple_ar_free(tuple_ar_t* tuple)
{
    ibase_time_cancel(&OLSR->ibase_ar_time_list, (tuple_time_t*)tuple);
    nu_mem_free(tuple);
}

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_ar_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/*
 */
static void
tuple_ar_timeout(tuple_time_t* tuple)
{
    tuple_ar_t* tuple_ar = (tuple_ar_t*)tuple;
    ibase_ar_iter_remove(tuple_ar);
}

/*
 */
static inline tuple_ar_t*
tuple_ar_create(const nu_ip_t orig_addr, const uint16_t seqnum)
{
    tuple_ar_t* tuple = nu_mem_alloc(tuple_ar_t);
    tuple->next = tuple->prev = NULL;
    tuple->orig_addr = orig_addr;
    tuple->seqnum = seqnum;

    tuple->timeout_proc = tuple_ar_timeout;
    tuple_time_init((tuple_time_t*)tuple, &OLSR->ibase_ar_time_list, -1);

    return tuple;
}

////////////////////////////////////////////////////////////////
//
// ibase_ar_t
//

/** Initializes the ibase_ar.
 */
PUBLIC void
ibase_ar_init(void)
{
    _ibase_ar_init(IBASE_AR);
    IBASE_AR->change = false;
}

/** Destroys the ibase_ar.
 */
PUBLIC void
ibase_ar_destroy(void)
{
    _ibase_ar_destroy(IBASE_AR);
}

/** Clears the change flag.
 */
PUBLIC void
ibase_ar_clear_change_flag(void)
{
    IBASE_AR->change = false;
}

/** Checks whether the ibase_ar has been changed.
 */
PUBLIC nu_bool_t
ibase_ar_is_changed(void)
{
    if (IBASE_AR->change)
        return true;
    return false;
}

/** Adds new tuple.
 *
 * @param orig_addr
 * @param seqnum
 * @return the new tuple
 */
PUBLIC tuple_ar_t*
ibase_ar_add(const nu_ip_t orig_addr, const uint16_t seqnum)
{
    tuple_ar_t* new_tuple = tuple_ar_create(orig_addr, seqnum);
    _ibase_ar_insert_tail(IBASE_AR, new_tuple);
    return new_tuple;
}

/** Searchs tuple.
 *
 * @param orig_addr
 * @return the tuple or NULL
 */
PUBLIC tuple_ar_t*
ibase_ar_search_orig_addr(const nu_ip_t orig_addr)
{
    FOREACH_AR(tuple_ar) {
        if (nu_orig_addr_eq(orig_addr, tuple_ar->orig_addr))
            return tuple_ar;
    }
    return NULL;
}

/** Removes the tuple
 *
 * @param tuple
 * @return the next tuple
 */
PUBLIC tuple_ar_t*
ibase_ar_iter_remove(tuple_ar_t* tuple)
{
    ibase_tr_remove_orig_addr(tuple->orig_addr);
    ibase_ta_remove_orig_addr(tuple->orig_addr);
    ibase_an_remove_orig_addr(tuple->orig_addr);
    return _ibase_ar_iter_remove(tuple, IBASE_AR);
}

/** Outputs ibase_ar.
 *
 * @param logger
 */
PUBLIC void
ibase_ar_put_log(nu_logger_t* logger)
{
    nu_logger_log(logger, "I_AR:%d:--", ibase_ar_size());
    FOREACH_AR(p) {
        nu_logger_log(logger,
                "I_AR:[t=%T orig:%I seq:%u]",
                &p->time, p->orig_addr, p->seqnum);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
