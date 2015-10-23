#include "config.h"

#include "core/logger.h"
#include "olsrv2/olsrv2.h"
#include "core/scheduler.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_proc
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tuple_proc_t
//

/*
 */
static inline nu_bool_t
tuple_proc_eq(const tuple_proc_t* tuple, const nu_msg_t* msg)
{
    if (!nu_ip_eq_without_prefix(tuple->ip, msg->orig_addr))
        return false;
    if (tuple->type != msg->type)
        return false;
    return tuple->seqnum == msg->seqnum;
}

static inline void
tuple_proc_free(tuple_proc_t* ibase_proc)
{
    ibase_time_cancel(&OLSR->ibase_proc_time_list, (tuple_time_t*)ibase_proc);
    nu_mem_free(ibase_proc);
}

////////////////////////////////////////////////////////////////
//
// ibase_proc_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/tuple_proc_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Traverses the ibase_proc.
 *
 * @param p
 * @param ibase_proc
 */
#define FOREACH_PROC(p, ibase_proc)                     \
    for (tuple_proc_t* p = ibase_proc_iter(ibase_proc); \
         !ibase_proc_iter_is_end(p, ibase_proc);        \
         p = ibase_proc_iter_next(p, ibase_proc))

/*
 */
static inline tuple_proc_t*
ibase_proc_iter(const ibase_proc_t* ibase_proc)
{
    for (int i = 0; i < IBASE_PROC_TABLE_SIZE; ++i) {
        if (!tuple_proc_list_is_empty(&ibase_proc->table[i]))
            return tuple_proc_list_iter(&ibase_proc->table[i]);
    }
    return tuple_proc_list_iter(&ibase_proc->table[0]);
}

/*
 * Returns true if the iteration is end.
 */
static inline nu_bool_t
ibase_proc_iter_is_end(tuple_proc_t* p, const ibase_proc_t* ibase_proc)
{
    return p == (tuple_proc_t*)&ibase_proc->table[0];
}

/*
 * Returns the iterator which points to the next element.
 */
static inline tuple_proc_t*
ibase_proc_iter_next(tuple_proc_t* p, const ibase_proc_t* ibase_proc)
{
    size_t i = nu_ip_hash(p->ip) % IBASE_PROC_TABLE_SIZE;
    if (p->next != (tuple_proc_t*)&ibase_proc->table[i])
        return p->next;

    for (++i; i < IBASE_PROC_TABLE_SIZE; ++i) {
        if (!tuple_proc_list_is_empty(&ibase_proc->table[i]))
            return tuple_proc_list_iter(&ibase_proc->table[i]);
    }
    return (tuple_proc_t*)&ibase_proc->table[0];
}

/** Gets the size of the ibase_proc.
 *
 * @param ibase_proc
 * @return the size of the ibase_proc.
 */
PUBLIC size_t
ibase_proc_size(const ibase_proc_t* ibase_proc)
{
    size_t n = 0;
    for (int i = 0; i < IBASE_PROC_TABLE_SIZE; ++i)
        n += tuple_proc_list_size(&ibase_proc->table[i]);
    return n;
}

/** Checks whether the ibase_proc is empty.
 *
 * @param ibase_proc
 * @return true if the ibase_proc is empty.
 */
PUBLIC nu_bool_t
ibase_proc_is_empty(const ibase_proc_t* ibase_proc)
{
    for (int i = 0; i < IBASE_PROC_TABLE_SIZE; ++i) {
        if (!tuple_proc_list_is_empty(&ibase_proc->table[i]))
            return false;
    }
    return true;
}

/*
 */
static inline tuple_proc_t*
ibase_proc_lookup_internal(const ibase_proc_t* ibase_proc,
        const nu_msg_t* msg)
{
    const nu_ip_t ip = msg->orig_addr;
    const size_t  h  = nu_ip_hash(ip);
    FOREACH_TUPLE_PROC_LIST(p, &ibase_proc->table[h % IBASE_PROC_TABLE_SIZE]) {
        if (tuple_proc_eq(p, msg))
            return p;
    }
    return tuple_proc_list_iter_end(&ibase_proc->table[0]);
}

/** Checks whether the ibase_proc contains the message information.
 *
 * @param ibase_proc
 * @param msg
 * @return true if the ibase_proc contains the message information
 */
PUBLIC nu_bool_t
ibase_proc_contain(const ibase_proc_t* ibase_proc, const nu_msg_t* msg)
{
    tuple_proc_t* p = ibase_proc_lookup_internal(ibase_proc, msg);
    if (!ibase_proc_iter_is_end(p, ibase_proc))
        return true;
    else
        return false;
}

/*
 */
static void
tuple_proc_timeout(tuple_time_t* ibase_proc)
{
    tuple_proc_t* tuple = (tuple_proc_t*)ibase_proc;
    tuple_proc_list_iter_remove(tuple, tuple->ibase);
}

/** Adds the message information.
 *
 * @param ibase_proc
 * @param msg
 * @param hold_time
 */
PUBLIC void
ibase_proc_add(ibase_proc_t* ibase_proc,
        const nu_msg_t* msg, const double hold_time)
{
    nu_ip_t       key = msg->orig_addr;
    const size_t     h = nu_ip_hash(key);
    tuple_proc_t* tuple = nu_mem_alloc(tuple_proc_t);
    tuple->type = msg->type;
    tuple->ip = msg->orig_addr;
    tuple->seqnum = msg->seqnum;
    tuple_proc_list_t* list = &ibase_proc->table[h % IBASE_PROC_TABLE_SIZE];
    tuple_proc_list_insert_head(list, tuple);

    tuple->ibase = list;
    tuple->timeout_proc = tuple_proc_timeout;
    tuple_time_init((tuple_time_t*)tuple, &OLSR->ibase_proc_time_list, hold_time);
}

/*
 */
static inline void
ibase_proc_remove_all(ibase_proc_t* ibase_proc)
{
    for (int i = 0; i < IBASE_PROC_TABLE_SIZE; ++i) {
        tuple_proc_list_remove_all(&ibase_proc->table[i]);
    }
}

/** Initializes the ibase_proc.
 *
 * @param ibase_proc
 */
PUBLIC void
ibase_proc_init(ibase_proc_t* ibase_proc)
{
    for (int i = 0; i < IBASE_PROC_TABLE_SIZE; ++i)
        tuple_proc_list_init(&ibase_proc->table[i]);
}

/** Destroys the ibase_proc.
 */
PUBLIC void
ibase_proc_destroy(ibase_proc_t* ibase_proc)
{
    ibase_proc_remove_all(ibase_proc);
}

/** Outputs ibase_proc.
 *
 * @param ibase_proc
 * @param logger
 * @param name
 */
PUBLIC void
ibase_proc_put_log(const ibase_proc_t* ibase_proc,
        nu_logger_t* logger, const char* name)
{
    nu_logger_log(logger, "%s:%d:--", name, ibase_proc_size(ibase_proc));
    FOREACH_PROC(p, ibase_proc) {
        nu_logger_log(logger,
                "%s:[time=%T type=%d ip=%I seq=%d]",
                name, &p->time, p->type, p->ip, p->seqnum);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
