//
//
//
#include "config.h"
#include "olsrv2/olsrv2.h"

#include "olsrv2/ibase_time_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_time
 */

/*
 */
static int
tuple_time_cmp(const tuple_time_t* x, const tuple_time_t* y)
{
    return nu_time_cmp(x->time, y->time);
}

#if 0

/*
 */
static void
timeout_list_insert(tuple_time_t* new_tuple)
{
    FOREACH_IBASE_TIME(tuple, IBASE_TIME) {
        if (tuple_time_cmp(tuple, new_tuple) >= 0) {
            _ibase_time_iter_insert_before(tuple, IBASE_TIME,
                    new_tuple);
            return;
        }
    }
    _ibase_time_insert_tail(IBASE_TIME, new_tuple);
}

#endif

/*
 */
static void
timeout_list_insert_r(tuple_time_t* new_tuple, ibase_time_t* ibase)
{
    FOREACH_IBASE_TIME_R(tuple, ibase) {
        if (tuple_time_cmp(tuple, new_tuple) < 0) {
            _ibase_time_iter_insert_after(tuple, ibase, new_tuple);
            return;
        }
    }
    _ibase_time_insert_head(ibase, new_tuple);
}

/*
 */
static inline void
timeout_list_remove(tuple_time_t* target, ibase_time_t* ibase)
{
    _ibase_time_iter_remove(target, ibase);
}

/** Initializes the tuple_time.
 *
 * @param tuple
 * @param ibase
 * @param timeout
 */
PUBLIC void
tuple_time_init(tuple_time_t* tuple, ibase_time_t* ibase, double timeout)
{
    assert(tuple && tuple->timeout_proc);
    tuple->time = NU_NOW;
    nu_time_add_f(&tuple->time, timeout);
    if (timeout <= 0)
        _ibase_time_insert_head(ibase, tuple);
    else
        timeout_list_insert_r(tuple, ibase);
}

/** Sets the timeout time.
 *
 * @param tuple
 * @param ibase
 * @param timeout
 */
PUBLIC void
tuple_time_set_timeout(tuple_time_t* tuple, ibase_time_t* ibase, double timeout)
{
    assert(tuple && tuple->timeout_proc);
    tuple->time = NU_NOW;
    nu_time_add_f(&tuple->time, timeout);
    _ibase_time_iter_cut(tuple, ibase);
    if (timeout <= 0)
        _ibase_time_insert_head(ibase, tuple);
    else
        timeout_list_insert_r(tuple, ibase);
}

/** Sets the timeout time.
 *
 * @param tuple
 * @param time_list
 * @param time
 */
PUBLIC void
tuple_time_set_time(tuple_time_t* tuple, ibase_time_t* time_list,
        const nu_time_t time)
{
    assert(tuple && tuple->timeout_proc);
    tuple->time = time;
    _ibase_time_iter_cut(tuple, time_list);
    timeout_list_insert_r(tuple, time_list);
}

/** Initializes the ibase_time.
 *
 * @param ibase
 */
PUBLIC void
ibase_time_init(ibase_time_t* ibase)
{
    _ibase_time_init(ibase);
}

/** Cancels the timeout tuple.
 *
 * @param ibase
 * @param tuple
 */
PUBLIC void
ibase_time_cancel(ibase_time_t* ibase, tuple_time_t* tuple)
{
    FOREACH_IBASE_TIME(t, ibase) {
        if (t == tuple) {
            _ibase_time_iter_cut(tuple, ibase);
            return;
        }
        if (tuple_time_cmp(t, tuple) > 0)
            return;
    }
}

/** Removes the timeouted tuples.
 *
 * @param ibase
 */
PUBLIC void
ibase_time_remove(ibase_time_t* ibase)
{
//  FOREACH_IBASE_TIME(tuple, ibase) {
//      nu_logger_tagf(NU_LOGGER,
//              "ibase_time_remove:I:tuple_time[t=%T]\n", &tuple->time);
//      assert(tuple && tuple->timeout_proc);
//  }
    while (!_ibase_time_is_empty(ibase)) {
        if (nu_time_cmp(_ibase_time_head(ibase)->time, NU_NOW) > 0)
            break;
        tuple_time_t* tuple = _ibase_time_shift(ibase);
        tuple->timeout_proc(tuple);
    }
//  FOREACH_IBASE_TIME(tuple, ibase) {
//      nu_logger_tagf(NU_LOGGER,
//              "ibase_time_remove:O:tuple_time[t=%T]\n", &tuple->time);
//      assert(tuple && tuple->timeout_proc);
//  }
}

/** @} */

}//namespace// //ScenSim-Port://
