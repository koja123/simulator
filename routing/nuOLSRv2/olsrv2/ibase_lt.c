//
// (L) Link Set
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
 * @addtogroup @ibase_lt
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tuple_lt_t
//

/*
 */
static inline tuple_lt_t*
tuple_lt_create(tuple_n_t* next, tuple_n_t* last, tuple_n2_t* final)
{
    tuple_lt_t* tuple = nu_mem_alloc(tuple_lt_t);
    tuple->next_id  = next;
    tuple->last_id  = last;
    tuple->final_ip = final;
    tuple->last_metric  = last->_in_metric;
    tuple->final_metric = final->in_metric;
    tuple->number_hops  = 2;
    return tuple;
}

/*
 */
static inline void
tuple_lt_free(tuple_lt_t* tuple)
{
    nu_mem_free(tuple);
}

/*
 */
static inline int
tuple_lt_cmp(tuple_lt_t* a, tuple_lt_t* b)
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
tuple_lt_cmp_by_final_ip(tuple_lt_t* a, tuple_lt_t* b)
{
    int r;
    if ((r = a->final_ip->hop2_ip - b->final_ip->hop2_ip) != 0)
        return r;
    if ((r = (int)((uintptr_t)a->next_id - (uintptr_t)b->next_id)) != 0)
        return r;
    return (int)((uintptr_t)a->last_id - (uintptr_t)b->last_id);
}

////////////////////////////////////////////////////////////////
//
// ibase_lt_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_lt_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Removes the tuple.
 *
 * @param tuple
 * @param ibase_lt
 * @return the next tuple
 */
PUBLIC tuple_lt_t*
ibase_lt_iter_remove(tuple_lt_t* tuple, ibase_lt_t* ibase_lt)
{
    return _ibase_lt_iter_remove(tuple, ibase_lt);
}

/** Initializes the ibase_lt.
 *
 * @param ibase_lt
 */
PUBLIC void
ibase_lt_init(ibase_lt_t* ibase_lt)
{
    _ibase_lt_init(ibase_lt);
    ibase_lt->sorted = false;
}

/** Destroys the ibase_lt.
 *
 * @param ibase_lt
 */
PUBLIC void
ibase_lt_destroy(ibase_lt_t* ibase_lt)
{
    while (!ibase_lt_is_empty(ibase_lt)) {
        ibase_lt_iter_remove(ibase_lt_head(ibase_lt), ibase_lt);
    }
}

/** (PRIVATE)
 *
 * @param self
 * @param ip
 * @param eq
 */
static inline tuple_lt_t*
ibase_lt_lookup(ibase_lt_t* self, tuple_lt_t* tuple, nu_bool_t* eq)
{
    *eq = false;
    FOREACH_LT(p, self) {
        int c;
        if ((c = tuple_lt_cmp(tuple, p)) <= 0) {
            *eq = (c == 0);
            return p;
        }
    }
    return ibase_lt_iter_end(self);
}

/** Sort the tuple.
 *
 * @param self
 * @param tuple_lt
 */
PUBLIC void
ibase_lt_resort(ibase_lt_t* self, tuple_lt_t* tuple_lt)
{
    assert(self->sorted);
    _ibase_lt_iter_cut(tuple_lt, self);
    nu_bool_t eq;
    return _ibase_lt_iter_insert_before(ibase_lt_lookup(self, tuple_lt, &eq),
            self, tuple_lt);
}

/** Sort all tuples
 *
 * @param self
 */
PUBLIC void
ibase_lt_sort(ibase_lt_t* self)
{
    size_t sz = ibase_lt_size(self);
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    std::vector<tuple_lt_t*> tl(sz);   //ScenSim-Port://
#else                                  //ScenSim-Port://
    tuple_lt_t* tl[sz];
#endif                                 //ScenSim-Port://
    tuple_lt_t* t;
    size_t i = 0;
    while ((t = _ibase_lt_shift(self)) != NULL) {
        tl[i++] = t;
    }
    for (i = 0; i < sz; ++i) {
        nu_bool_t eq;
        _ibase_lt_iter_insert_before(ibase_lt_lookup(self, tl[i], &eq),
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
static inline tuple_lt_t*
ibase_lt_lookup_by_final_ip(ibase_lt_t* self, tuple_lt_t* tuple, nu_bool_t* eq)
{
    *eq = false;
    FOREACH_LT(p, self) {
        int c;
        if ((c = tuple_lt_cmp_by_final_ip(tuple, p)) <= 0) {
            *eq = (c == 0);
            return p;
        }
    }
    return ibase_lt_iter_end(self);
}

/** Sort all tuples by final_ip
 *
 * @param self
 */
PUBLIC void
ibase_lt_sort_by_final_ip(ibase_lt_t* self)
{
    size_t sz = ibase_lt_size(self);
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    std::vector<tuple_lt_t*> tl(sz);   //ScenSim-Port://
#else                                  //ScenSim-Port://
    tuple_lt_t* tl[sz];
#endif                                 //ScenSim-Port://
    tuple_lt_t* t;
    size_t i = 0;
    while ((t = _ibase_lt_shift(self)) != NULL) {
        tl[i++] = t;
    }
    for (i = 0; i < sz; ++i) {
        nu_bool_t eq;
        _ibase_lt_iter_insert_before(ibase_lt_lookup_by_final_ip(self, tl[i], &eq),
                self, tl[i]);
    }
    self->sorted = true;
}

/** Adds the new tuple.
 *
 * @param self
 * @param next
 * @param last
 * @param final
 * @return the new tuple
 */
PUBLIC void
ibase_lt_add(ibase_lt_t* self,
        tuple_n_t* next, tuple_n_t* last, tuple_n2_t* final)
{
    FOREACH_LT(t, self) {
        if (t->next_id == next && t->last_id == last &&
            t->final_ip->hop2_ip == final->hop2_ip) {
            if (t->final_metric > final->in_metric)
                t->final_metric = final->in_metric;
            return;
        }
    }
    self->sorted = false;
    tuple_lt_t* tuple_lt = tuple_lt_create(next, last, final);
    _ibase_lt_insert_head(self, tuple_lt);
}

/** Outputs tuple_lt.
 *
 * @param tuple
 * @param logger
 */
PUBLIC void
tuple_lt_put_log(tuple_lt_t* tuple, nu_logger_t* logger)
{
    nu_logger_log(logger,
            "I_LT:[next=(%S) last:(%S) final:%I last_metric:%g final_metric:%g hop:%d]",
            &tuple->next_id->neighbor_ip_list,
            &tuple->last_id->neighbor_ip_list,
            tuple->final_ip->hop2_ip,
            tuple->last_metric, tuple->final_metric, tuple->number_hops);
}

/** Outputs ibase_lt.
 *
 * @param ibase_lt
 * @param logger
 */
PUBLIC void
ibase_lt_put_log(ibase_lt_t* ibase_lt, nu_logger_t* logger)
{
    nu_logger_log(logger, "I_LT:%d:--", ibase_lt_size(ibase_lt));
    FOREACH_LT(p, ibase_lt) {
        tuple_lt_put_log(p, logger);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
