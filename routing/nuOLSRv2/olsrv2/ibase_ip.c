#include "config.h"

#include "core/core.h"
#include "core/mem.h"
#include "olsrv2/ibase_ip.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup @ibase_ip
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tuple_ip_t
//

/*
 */
static void
tuple_ip_free(tuple_ip_t* tuple)
{
    ibase_time_cancel(tuple->time_list, (tuple_time_t*)tuple);
    nu_mem_free(tuple);
}

/*
 */
static void
tuple_ip_timeout(tuple_time_t* tuple_ip)
{
    tuple_ip_t* tuple = (tuple_ip_t*)tuple_ip;
    ibase_ip_iter_remove(tuple, tuple->ibase);
}

////////////////////////////////////////////////////////////////
//
// ibase_ip_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_ip_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Initializes the ibase_ip.
 *
 * @param ibase
 */
PUBLIC void
ibase_ip_init(ibase_ip_t* ibase)
{
    _ibase_ip_init(ibase);
    ibase->change = false;
}

/** Destroys the ibase_ip.
 *
 * @param ibase
 */
PUBLIC void
ibase_ip_destroy(ibase_ip_t* ibase)
{
    _ibase_ip_destroy(ibase);
}

/** Adds the tuple.
 *
 * @param ibase
 * @param ip
 * @param time_list
 * @return the new tuple.
 */
PUBLIC tuple_ip_t*
ibase_ip_add(ibase_ip_t* ibase, const nu_ip_t ip, ibase_time_t* time_list)
{
    tuple_ip_t* new_tuple = nu_mem_alloc(tuple_ip_t);
    new_tuple->ip = ip;
    _ibase_ip_insert_tail(ibase, new_tuple);
    ibase->change = true;

    new_tuple->ibase = ibase;
    new_tuple->timeout_proc = tuple_ip_timeout;
    new_tuple->time_list = time_list;
    tuple_time_init((tuple_time_t*)new_tuple, time_list, -1);

    return new_tuple;
}

/** Adds the tuple.
 *
 * @param ibase
 * @param ip
 * @param timeout
 * @param time_list
 * @return the new tuple
 */
PUBLIC tuple_ip_t*
ibase_ip_add_with_timeout(ibase_ip_t* ibase, const nu_ip_t ip,
        double timeout, ibase_time_t* time_list)
{
    tuple_ip_t* new_tuple = nu_mem_alloc(tuple_ip_t);
    new_tuple->ip = ip;
    _ibase_ip_insert_tail(ibase, new_tuple);
    ibase->change = true;

    new_tuple->ibase = ibase;
    new_tuple->timeout_proc = tuple_ip_timeout;
    new_tuple->time_list = time_list;
    tuple_time_init((tuple_time_t*)new_tuple, time_list, timeout);

    return new_tuple;
}

/** Checks whether the ibase_ip contains the ip.
 *
 * @param ibase
 * @param ip
 * @return true if the ibase_ip contains the ip
 */
PUBLIC nu_bool_t
ibase_ip_contain(ibase_ip_t* ibase, const nu_ip_t ip)
{
    FOREACH_IBASE_IP(p, ibase) {
        if (nu_ip_eq(ip, p->ip))
            return true;
    }
    return false;
}

/** Checks whether the ibase_ip contains the orig_addr
 *
 * @param ibase
 * @param orig_addr
 * @return true if the ibase_ip contains the orig_addr.
 */
PUBLIC nu_bool_t
ibase_ip_contain_orig_addr(ibase_ip_t* ibase, const nu_ip_t orig_addr)
{
    FOREACH_IBASE_IP(p, ibase) {
        if (nu_orig_addr_eq(orig_addr, p->ip))
            return true;
    }
    return false;
}

/** Searchs the ip.
 *
 * @param ibase
 * @param ip
 * @return the tuple or NULL.
 */
PUBLIC tuple_ip_t*
ibase_ip_search(ibase_ip_t* ibase, const nu_ip_t ip)
{
    FOREACH_IBASE_IP(p, ibase) {
        if (nu_ip_eq(ip, p->ip))
            return p;
    }
    return NULL;
}

/** Removes the tuple.
 *
 * @param tuple
 * @param ibase
 * @return the next tuple
 */
PUBLIC tuple_ip_t*
ibase_ip_iter_remove(tuple_ip_t* tuple, ibase_ip_t* ibase)
{
    ibase->change = true;
    return _ibase_ip_iter_remove(tuple, ibase);
}

/** Removes the ip.
 *
 * @param ibase
 * @param ip
 */
PUBLIC void
ibase_ip_remove(ibase_ip_t* ibase, const nu_ip_t ip)
{
    tuple_ip_t* tuple = ibase_ip_search(ibase, ip);
    if (tuple != NULL)
        ibase_ip_iter_remove(tuple, ibase);
}

/** Removes all the tuples.
 *
 * @param ibase
 */
PUBLIC void
ibase_ip_remove_all(ibase_ip_t* ibase)
{
    if (!_ibase_ip_is_empty(ibase))
        ibase->change = true;
    _ibase_ip_remove_all(ibase);
}

/** Outputs ibase.
 *
 * @param ibase
 * @param logger
 * @param name
 */
PUBLIC void
ibase_ip_put_log(ibase_ip_t* ibase,
        nu_logger_t* logger, const char* name)
{
    nu_logger_log(logger, "%s:%d:--", name, ibase_ip_size(ibase));
    FOREACH_IBASE_IP(p, ibase) {
        nu_logger_log(logger, "%s:[ip=%I t=%T]", name, p->ip, &p->time);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
