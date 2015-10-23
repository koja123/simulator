//
// Temporal information base for calculating flooding mpr
//
#include "config.h"

#include "core/core.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"

#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
#include <vector>                      //ScenSim-Port://
#endif                                 //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup @ibase_l2
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tuple_l2_t
//

/*
 */
static inline tuple_l2_t*
tuple_l2_create(tuple_n_t* next, tuple_l_t* last, tuple_n2_t* final)
{
    tuple_l2_t* tuple = nu_mem_alloc(tuple_l2_t);
    tuple->next_id  = next;
    tuple->last_id  = last;
    tuple->final_ip = final;
    return tuple;
}

/*
 */
static inline void
tuple_l2_free(tuple_l2_t* tuple)
{
    nu_mem_free(tuple);
}

/*
 */
static inline int
tuple_l2_cmp(tuple_l2_t* a, tuple_l2_t* b)
{
    int r;
    if ((r = (int)((uintptr_t)a->next_id - (uintptr_t)b->next_id)) != 0)
        return r;
    if ((r = (int)((uintptr_t)a->last_id - (uintptr_t)b->last_id)) != 0)
        return r;
    return (int)((uintptr_t)a->final_ip - (uintptr_t)b->final_ip);
}

/*
 */
static inline int
tuple_l2_cmp_by_final_ip(tuple_l2_t* a, tuple_l2_t* b)
{
    int r;
    if ((r = nu_ip_cmp(a->final_ip->hop2_ip, b->final_ip->hop2_ip)) != 0)
        return r;
    if ((r = (int)((uintptr_t)a->next_id - (uintptr_t)b->next_id)) != 0)
        return r;
    return (int)((uintptr_t)a->last_id - (uintptr_t)b->last_id);
}

////////////////////////////////////////////////////////////////
//
// ibase_l2_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_l2_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Removes the tuple.
 *
 * @param tuple
 * @param ibase_l2
 * @return the next tuple
 */
PUBLIC tuple_l2_t*
ibase_l2_iter_remove(tuple_l2_t* tuple, ibase_l2_t* ibase_l2)
{
    return _ibase_l2_iter_remove(tuple, ibase_l2);
}

/** Initializes the ibase_l2.
 *
 * @param ibase_l2
 */
PUBLIC void
ibase_l2_init(ibase_l2_t* ibase_l2)
{
    _ibase_l2_init(ibase_l2);
    ibase_l2->sorted = false;
}

/** Destroys the ibase_l2.
 *
 * @param ibase_l2
 */
PUBLIC void
ibase_l2_destroy(ibase_l2_t* ibase_l2)
{
    while (!ibase_l2_is_empty(ibase_l2)) {
        ibase_l2_iter_remove(ibase_l2_head(ibase_l2), ibase_l2);
    }
}

/** (PRIVATE)
 *
 * @param self
 * @param ip
 * @param eq
 */
static inline tuple_l2_t*
ibase_l2_lookup(ibase_l2_t* self, tuple_l2_t* tuple, nu_bool_t* eq)
{
    *eq = false;
    FOREACH_L2(p, self) {
        int c;
        if ((c = tuple_l2_cmp(tuple, p)) <= 0) {
            *eq = (c == 0);
            return p;
        }
    }
    return ibase_l2_iter_end(self);
}

/** Sort the tuple.
 *
 * @param self
 * @param tuple_l2
 */
PUBLIC void
ibase_l2_resort(ibase_l2_t* self, tuple_l2_t* tuple_l2)
{
    assert(self->sorted);
    _ibase_l2_iter_cut(tuple_l2, self);
    nu_bool_t eq;
    return _ibase_l2_iter_insert_before(ibase_l2_lookup(self, tuple_l2, &eq),
            self, tuple_l2);
}

/** Sort all tuples
 *
 * @param self
 */
PUBLIC void
ibase_l2_sort(ibase_l2_t* self)
{
    size_t sz = ibase_l2_size(self);
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    std::vector<tuple_l2_t*> tl(sz);   //ScenSim-Port://
#else                                  //ScenSim-Port://
    tuple_l2_t* tl[sz];
#endif                                 //ScenSim-Port://
    tuple_l2_t* t;
    size_t i = 0;
    while ((t = _ibase_l2_shift(self)) != NULL) {
        tl[i++] = t;
    }
    for (i = 0; i < sz; ++i) {
        nu_bool_t eq;
        _ibase_l2_iter_insert_before(ibase_l2_lookup(self, tl[i], &eq),
                self, tl[i]);
    }
    self->sorted = true;
}

/** (PRIVATE)
 *
 * @param self
 * @param ip
 * @param eq
 */
static inline tuple_l2_t*
ibase_l2_lookup_by_final_ip(ibase_l2_t* self, tuple_l2_t* tuple, nu_bool_t* eq)
{
    *eq = false;
    FOREACH_L2(p, self) {
        int c;
        if ((c = tuple_l2_cmp_by_final_ip(tuple, p)) <= 0) {
            *eq = (c == 0);
            return p;
        }
    }
    return ibase_l2_iter_end(self);
}

/** Sort all tuples by final_ip
 *
 * @param self
 */
PUBLIC void
ibase_l2_sort_by_final_ip(ibase_l2_t* self)
{
    size_t sz = ibase_l2_size(self);
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    std::vector<tuple_l2_t*> tl(sz);   //ScenSim-Port://
#else                                  //ScenSim-Port://
    tuple_l2_t* tl[sz];
#endif                                 //ScenSim-Port://
    tuple_l2_t* t;
    size_t i = 0;
    while ((t = _ibase_l2_shift(self)) != NULL) {
        tl[i++] = t;
    }
    for (i = 0; i < sz; ++i) {
        nu_bool_t eq;
        _ibase_l2_iter_insert_before(ibase_l2_lookup_by_final_ip(self, tl[i], &eq),
                self, tl[i]);
    }
    self->sorted = true;
}

/** Adds the new tuple.
 *
 * @param self ibase_l2.
 * @param next
 * @param last
 * @param final
 * @return the new tuple
 */
PUBLIC void
ibase_l2_add(ibase_l2_t* self, tuple_n_t* next, tuple_l_t* last, tuple_n2_t* final)
{
#if 0
    FOREACH_L2(t, self) {
        if (t->next_id == next && t->last_id == last &&
            t->final_ip == final->hop2_ip) {
            if (t->final_metric > final->in_metric)
                t->final_metric = final->in_metric;
            return;
        }
    }
#endif
    self->sorted = false;
    tuple_l2_t* tuple_l2 = tuple_l2_create(next, last, final);
    _ibase_l2_insert_head(self, tuple_l2);
}

/** Outputs tuple_l2.
 *
 * @param tuple
 * @param logger
 */
PUBLIC void
tuple_l2_put_log(tuple_l2_t* tuple, nu_logger_t* logger)
{
    nu_logger_log(logger,
            "I_L2:[next=(%S) last:(%S) final:%I]",
            &tuple->next_id->neighbor_ip_list,
            &tuple->last_id->neighbor_ip_list,
            tuple->final_ip->hop2_ip);
}

/** Outputs ibase_l2.
 *
 * @param ibase_l2
 * @param logger
 */
PUBLIC void
ibase_l2_put_log(ibase_l2_t* ibase_l2, nu_logger_t* logger)
{
    nu_logger_log(logger, "I_L2:%d:--", ibase_l2_size(ibase_l2));
    FOREACH_L2(p, ibase_l2) {
        tuple_l2_put_log(p, logger);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
