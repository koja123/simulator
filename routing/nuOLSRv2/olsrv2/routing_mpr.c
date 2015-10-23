//
// Routing MPR calculator
//
#include "config.h"

#include "core/core.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup olsrv2
 * @{
 */
static inline void
add_routing_mpr(tuple_lt_t* tuple_lt)
{
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_log(NU_LOGGER, "add:%S",
                    &tuple_lt->next_id->neighbor_ip_list);
            );

    tuple_n_t* n = tuple_lt->next_id;
    tuple_n_set_routing_mpr(n, true);
}

static void
routing_mpr_phase0(ibase_lt_t* ibase_lt)
{
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_push_prefix(NU_LOGGER, "phase0_making:");
            );
    // Setup ibase_lt
    FOREACH_I(tuple_i) {
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            tuple_n_t* tuple_n = ibase_n_search_tuple_l(tuple_l);
            assert(tuple_n != NULL);
            if (!tuple_n->symmetric)
                continue;
            FOREACH_N2(tuple_n2, &tuple_l->ibase_n2) {
                ibase_lt_add(ibase_lt, tuple_n, tuple_n, tuple_n2);
            }
        }
    }
    OLSRV2_DO_LOG(calc_mpr,
            ibase_lt_put_log(ibase_lt, NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );
}

static void
routing_mpr_phase1(ibase_lt_t* ibase_lt)
{
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_push_prefix(NU_LOGGER, "phase1_updating:");
            );
    nu_bool_t changed = true;
    while (changed) {
        changed = false;
        FOREACH_LT(a, ibase_lt) {
            FOREACH_LT(b, ibase_lt) {
                if (a == b)
                    break;
                nu_link_metric_t a_metric = a->last_metric + a->final_metric;
                if (a_metric < b->last_metric &&
                    nu_ip_set_contain(&b->last_id->neighbor_ip_list,
                            a->final_ip->hop2_ip)) {
                    b->next_id = a->next_id;
                    b->last_metric = a_metric;
                    b->number_hops = a->number_hops + 1;
                    changed = true;
                    OLSRV2_DO_LOG(calc_mpr,
                            tuple_lt_put_log(b, NU_LOGGER);
                            );
                    continue;
                }
                nu_link_metric_t b_metric = b->last_metric + b->final_metric;
                if (b_metric < a->last_metric &&
                    nu_ip_set_contain(&a->last_id->neighbor_ip_list,
                            b->final_ip->hop2_ip)) {
                    a->next_id = b->next_id;
                    a->last_metric = b_metric;
                    a->number_hops = b->number_hops + 1;
                    changed = true;
                    OLSRV2_DO_LOG(calc_mpr,
                            tuple_lt_put_log(a, NU_LOGGER);
                            );
                    continue;
                }
            }
        }
    }
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_pop_prefix(NU_LOGGER);
            );
}

static void
routing_mpr_phase2(ibase_lt_t* ibase_lt)
{
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_push_prefix(NU_LOGGER, "phase2_removing:");
            );
    for (tuple_lt_t* tuple_lt = ibase_lt_iter(ibase_lt);
         !ibase_lt_iter_is_end(tuple_lt, ibase_lt);) {
        nu_bool_t do_remove = false;
        FOREACH_I(tuple_i) {
            tuple_l_t* tuple_l = ibase_l_search_neighbor_ip(&tuple_i->ibase_l,
                    tuple_lt->final_ip->hop2_ip);
            if (tuple_l != NULL && tuple_l->in_metric <= tuple_lt->final_metric) {
                do_remove = true;
                break;
            }
        }
        if (do_remove) {
            OLSRV2_DO_LOG(calc_mpr,
                    tuple_lt_put_log(tuple_lt, NU_LOGGER);
                    );
            tuple_lt = ibase_lt_iter_remove(tuple_lt, ibase_lt);
        } else {
            tuple_lt = ibase_lt_iter_next(tuple_lt, ibase_lt);
        }
    }

//    ibase_lt_sort_by_final_ip(ibase_lt); //ScenSim-Port://
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_log(NU_LOGGER, "Sorting_by_final_ip");
            ibase_lt_put_log(ibase_lt, NU_LOGGER);
            nu_logger_log(NU_LOGGER, "Sorted_by_final_ip");
            );
    for (tuple_lt_t* tuple_lt = ibase_lt_iter(ibase_lt);
         !ibase_lt_iter_is_end(tuple_lt, ibase_lt);) {
        nu_ip_t   final_ip  = tuple_lt->final_ip->hop2_ip;
        nu_bool_t do_remove = false;
        FOREACH_LT(t, ibase_lt) {
            if (t == tuple_lt)
                continue;
            int cmp = t->final_ip->hop2_ip - final_ip;
            if (cmp < 0)
                continue;
            if (cmp > 0)
                break;
            double t_m = t->last_metric + t->final_metric;
            double tuple_lt_m = tuple_lt->last_metric + tuple_lt->final_metric;
            if (t_m < tuple_lt_m ||
                (t_m == tuple_lt_m && t->number_hops < tuple_lt->number_hops)) {
                do_remove = true;
            }
        }
        if (do_remove) {
            OLSRV2_DO_LOG(calc_mpr,
                    tuple_lt_put_log(tuple_lt, NU_LOGGER);
                    );
            tuple_lt = ibase_lt_iter_remove(tuple_lt, ibase_lt);
        } else {
            tuple_lt = ibase_lt_iter_next(tuple_lt, ibase_lt);
        }
    }
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_pop_prefix(NU_LOGGER);
            );
}

/*
 */
static inline void
remove_tuple_lt(ibase_lt_t* ibase_lt, tuple_lt_t* tuple_lt)
{
    // Collect hop2 addresses from selected MPR
    tuple_n_t*  n = tuple_lt->next_id;
    tuple_lt_t* t = tuple_lt;
    nu_ip_set_t h2_set;
    nu_ip_set_init(&h2_set);
    do {
        nu_ip_set_add(&h2_set, t->final_ip->hop2_ip);
        t = ibase_lt_iter_next(t, ibase_lt);
    } while (!ibase_lt_iter_is_end(t, ibase_lt) && t->next_id == n);

    // tuple_lt assigned remove hop2 addresses in
    while (nu_ip_set_size(&h2_set) > 0) {
        nu_ip_t ip = nu_ip_set_remove_head(&h2_set);
        for (tuple_lt_t* t = ibase_lt_iter(ibase_lt);
             !ibase_lt_iter_is_end(t, ibase_lt);) {
            if (nu_ip_eq(ip, t->final_ip->hop2_ip))
                t = ibase_lt_iter_remove(t, ibase_lt);
            else
                t = ibase_lt_iter_next(t, ibase_lt);
        }
    }
    nu_ip_set_destroy(&h2_set);
}

/*
 */
static inline void
add_always(ibase_lt_t* ibase_lt)
{
    nu_bool_t has_always = false;
    FOREACH_LT(tuple_lt, ibase_lt) {
        if (tuple_lt->next_id->willingness == WILLINGNESS__ALWAYS) {
            add_routing_mpr(tuple_lt);
            has_always = true;
        }
    }
    if (!has_always)
        return;

    nu_bool_t changed = true;
    while (changed) {
        changed = false;
        FOREACH_LT(tuple_lt, ibase_lt) {
            if (tuple_lt->next_id->willingness == WILLINGNESS__ALWAYS) {
                remove_tuple_lt(ibase_lt, tuple_lt);
                changed = true;
                break;
            }
        }
    }
}

/**
 *
 */
static void
select_routing_mpr_by_metric_per_degree(ibase_lt_t* ibase_lt)
{
    add_always(ibase_lt);

//    ibase_lt_sort(ibase_lt); //ScenSim-Port://
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_log(NU_LOGGER, "Sorting_by_next");
            ibase_lt_put_log(ibase_lt, NU_LOGGER);
            nu_logger_log(NU_LOGGER, "Sorted_by_next");
            );
    while (ibase_lt_size(ibase_lt) > 0) {
        tuple_lt_t* selected = NULL;
        double min_metric_per_node = 1e100;
        for (tuple_lt_t* t = ibase_lt_iter(ibase_lt);
             !ibase_lt_iter_is_end(t, ibase_lt);) {
            int    d = 0;
            double m = 0.0;
            tuple_lt_t* candidate = t;
            for (tuple_n_t* prev = t->next_id;
                 prev == t->next_id;
                 t = ibase_lt_iter_next(t, ibase_lt)) {
                d += 1;
                m += t->last_metric + t->final_metric;
            }
            double metric_per_node = m / (double)d;
            if (metric_per_node < min_metric_per_node) {
                min_metric_per_node = metric_per_node;
                selected = candidate;
            }
        }
        add_routing_mpr(selected);
        remove_tuple_lt(ibase_lt, selected);
    }
}

/**
 *
 */
static void
select_routing_mpr_by_degree_or_metric(ibase_lt_t* ibase_lt)
{
    add_always(ibase_lt);

//    ibase_lt_sort(ibase_lt); //ScenSim-Port://
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_log(NU_LOGGER, "Sorting_by_next");
            ibase_lt_put_log(ibase_lt, NU_LOGGER);
            nu_logger_log(NU_LOGGER, "Sorted_by_next");
            );
    while (ibase_lt_size(ibase_lt) > 0) {
        tuple_lt_t* selected = NULL;
        int    max_degree = 0;
        double min_metric = 1e100;
        for (tuple_lt_t* t = ibase_lt_iter(ibase_lt);
             !ibase_lt_iter_is_end(t, ibase_lt);) {
            int    d = 0;
            double m = 0.0;
            tuple_lt_t* candidate = t;
            for (tuple_n_t* prev = t->next_id;
                 prev == t->next_id;
                 t = ibase_lt_iter_next(t, ibase_lt)) {
                d += 1;
                m += t->last_metric + t->final_metric;
            }
            if (max_degree < d) {
                max_degree = d;
                min_metric = m;
                selected = candidate;
            } else if (max_degree == d && min_metric > m) {
                min_metric = m;
                selected = candidate;
            }
        }
        add_routing_mpr(selected);
        remove_tuple_lt(ibase_lt, selected);
    }
}

/**
 *
 */
static void
select_routing_mpr_by_degree(ibase_lt_t* ibase_lt)
{
    add_always(ibase_lt);

//    ibase_lt_sort(ibase_lt); //ScenSim-Port://
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_log(NU_LOGGER, "Sorting_by_next");
            ibase_lt_put_log(ibase_lt, NU_LOGGER);
            nu_logger_log(NU_LOGGER, "Sorted_by_next");
            );
    while (ibase_lt_size(ibase_lt) > 0) {
        tuple_lt_t* selected = NULL;
        int max_degree = 0;
        for (tuple_lt_t* t = ibase_lt_iter(ibase_lt);
             !ibase_lt_iter_is_end(t, ibase_lt);) {
            int d = 0;
            tuple_lt_t* candidate = t;
            for (tuple_n_t* prev = t->next_id;
                 prev == t->next_id;
                 t = ibase_lt_iter_next(t, ibase_lt)) {
                d += 1;
            }
            if (max_degree < d) {
                max_degree = d;
                selected = candidate;
            }
        }
        add_routing_mpr(selected);
        remove_tuple_lt(ibase_lt, selected);
    }
}

/** Calculates routing MPR
 */
PUBLIC void
olsrv2_calc_routing_mpr(void)
{
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_push_prefix(NU_LOGGER, "CALC_RMPR:");
            );

    static ibase_lt_t ibase_lt;
    ibase_lt_init(&ibase_lt);

    routing_mpr_phase0(&ibase_lt);
    routing_mpr_phase1(&ibase_lt);
    routing_mpr_phase2(&ibase_lt);

    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_push_prefix(NU_LOGGER, "phase3_selecting:");
            );

    ibase_n_save_routing_mpr();
    if (RMPR_TYPE == MPR__BY_METRIC_PER_DEGREE)
        select_routing_mpr_by_metric_per_degree(&ibase_lt);
    else if (RMPR_TYPE == MPR__BY_DEGREE_OR_METRIC)
        select_routing_mpr_by_degree_or_metric(&ibase_lt);
    else
        select_routing_mpr_by_degree(&ibase_lt);
    ibase_n_commit_routing_mpr();

    OLSRV2_DO_LOG(calc_mpr,
            ibase_lt_put_log(&ibase_lt, NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );

    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_pop_prefix(NU_LOGGER);
            );

    ibase_lt_destroy(&ibase_lt);
}

/** @} */

}//namespace// //ScenSim-Port://
