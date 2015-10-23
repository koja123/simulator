//
// Flooding MPR calculator
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
add_flooding_mpr(tuple_l2_t* tuple_l2)
{
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_log(NU_LOGGER, "add:%S",
                    &tuple_l2->next_id->neighbor_ip_list);
            );
    tuple_n_t* n = tuple_l2->next_id;
    tuple_n_set_flooding_mpr(n, true);
}

/*
 */
static inline void
remove_tuple_l2(ibase_l2_t* ibase_l2, tuple_l2_t* tuple_l2)
{
    // Collect hop2 addresses from selected MPR
    tuple_n_t*  n = tuple_l2->next_id;
    tuple_l2_t* t = tuple_l2;
    nu_ip_set_t h2_set;
    nu_ip_set_init(&h2_set);
    do {
        nu_ip_set_add(&h2_set, t->final_ip->hop2_ip);
        t = ibase_l2_iter_next(t, ibase_l2);
    } while (!ibase_l2_iter_is_end(t, ibase_l2) && t->next_id == n);

    // tuple_l2 assigned remove hop2 addresses in
    while (nu_ip_set_size(&h2_set) > 0) {
        nu_ip_t ip = nu_ip_set_remove_head(&h2_set);
        for (tuple_l2_t* t = ibase_l2_iter(ibase_l2);
             !ibase_l2_iter_is_end(t, ibase_l2);) {
            if (nu_ip_eq(ip, t->final_ip->hop2_ip))
                t = ibase_l2_iter_remove(t, ibase_l2);
            else
                t = ibase_l2_iter_next(t, ibase_l2);
        }
    }
    nu_ip_set_destroy(&h2_set);
}

/*
 */
static inline void
add_always(ibase_l2_t* ibase_l2)
{
    nu_bool_t has_always = false;
    FOREACH_L2(tuple_l2, ibase_l2) {
        if (tuple_l2->next_id->willingness == WILLINGNESS__ALWAYS) {
            add_flooding_mpr(tuple_l2);
            has_always = true;
        }
    }
    if (!has_always)
        return;

    nu_bool_t changed = true;
    while (changed) {
        changed = false;
        FOREACH_L2(tuple_l2, ibase_l2) {
            if (tuple_l2->next_id->willingness == WILLINGNESS__ALWAYS) {
                remove_tuple_l2(ibase_l2, tuple_l2);
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
select_flooding_mpr_by_metric_per_degree(ibase_l2_t* ibase_l2)
{
    add_always(ibase_l2);

//    ibase_l2_sort(ibase_l2); //ScenSim-Port://
    while (ibase_l2_size(ibase_l2) > 0) {
        tuple_l2_t* selected = NULL;
        double min_metric_per_node = 1e100;
        for (tuple_l2_t* t = ibase_l2_iter(ibase_l2);
             !ibase_l2_iter_is_end(t, ibase_l2);) {
            int    d = 0;
            double m = 0.0;
            tuple_l2_t* candidate = t;
            for (tuple_n_t* prev = t->next_id;
                 prev == t->next_id;
                 t = ibase_l2_iter_next(t, ibase_l2)) {
                d += 1;
                m += t->next_id->_out_metric + t->final_ip->out_metric;
            }
            double metric_per_node = m / (double)d;
            if (metric_per_node < min_metric_per_node) {
                min_metric_per_node = metric_per_node;
                selected = candidate;
            }
        }
        add_flooding_mpr(selected);
        remove_tuple_l2(ibase_l2, selected);
    }
}

/**
 *
 */
static void
select_flooding_mpr_by_degree_or_metric(ibase_l2_t* ibase_l2)
{
    add_always(ibase_l2);

//    ibase_l2_sort(ibase_l2); //ScenSim-Port://
    while (ibase_l2_size(ibase_l2) > 0) {
        tuple_l2_t* selected = NULL;
        int    max_degree = 0;
        double min_metric = 1e100;
        for (tuple_l2_t* t = ibase_l2_iter(ibase_l2);
             !ibase_l2_iter_is_end(t, ibase_l2);) {
            int    d = 0;
            double m = 0.0;
            tuple_l2_t* candidate = t;
            for (tuple_n_t* prev = t->next_id;
                 prev == t->next_id;
                 t = ibase_l2_iter_next(t, ibase_l2)) {
                d += 1;
                m += t->next_id->_out_metric + t->final_ip->out_metric;
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
        add_flooding_mpr(selected);
        remove_tuple_l2(ibase_l2, selected);
    }
}

/**
 *
 */
static void
select_flooding_mpr_by_degree(ibase_l2_t* ibase_l2)
{
    add_always(ibase_l2);

//    ibase_l2_sort(ibase_l2); //ScenSim-Port://
    while (ibase_l2_size(ibase_l2) > 0) {
        tuple_l2_t* selected = NULL;
        int max_degree = 0;
        for (tuple_l2_t* t = ibase_l2_iter(ibase_l2);
             !ibase_l2_iter_is_end(t, ibase_l2);) {
            int d = 0;
            tuple_l2_t* candidate = t;
            for (tuple_n_t* prev = t->next_id;
                 prev == t->next_id;
                 t = ibase_l2_iter_next(t, ibase_l2)) {
                d += 1;
            }
            if (max_degree < d) {
                max_degree = d;
                selected = candidate;
            }
        }
        add_flooding_mpr(selected);
        remove_tuple_l2(ibase_l2, selected);
    }
}

/** Calculates flooding MPR
 */
PUBLIC void
olsrv2_calc_flooding_mpr(void)
{
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_push_prefix(NU_LOGGER, "CALC_FMPR:");
            );
    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_push_prefix(NU_LOGGER, "phase0_making:");
            );

    // Setup ibase_l2
    static ibase_l2_t ibase_l2;
    ibase_l2_init(&ibase_l2);

    FOREACH_I(tuple_i) {
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            tuple_n_t* tuple_n = ibase_n_search_tuple_l(tuple_l);
            assert(tuple_n != NULL);
            if (!tuple_n->symmetric)
                continue;
            FOREACH_N2(tuple_n2, &tuple_l->ibase_n2) {
                // Add only strict 2-hop
                if (!ibase_n_contain_symmetric(tuple_n2->hop2_ip))
                    ibase_l2_add(&ibase_l2, tuple_n, tuple_l, tuple_n2);
            }
        }
    }
    OLSRV2_DO_LOG(calc_mpr,
            ibase_l2_put_log(&ibase_l2, NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );

    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_push_prefix(NU_LOGGER, "phase1_selecting:");
            );

    ibase_n_save_flooding_mpr();
    if (FMPR_TYPE == MPR__BY_METRIC_PER_DEGREE)
        select_flooding_mpr_by_metric_per_degree(&ibase_l2);
    else if (FMPR_TYPE == MPR__BY_DEGREE_OR_METRIC)
        select_flooding_mpr_by_degree_or_metric(&ibase_l2);
    else
        select_flooding_mpr_by_degree(&ibase_l2);
    ibase_n_commit_flooding_mpr();

    OLSRV2_DO_LOG(calc_mpr,
            ibase_l2_put_log(&ibase_l2, NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );

    OLSRV2_DO_LOG(calc_mpr,
            nu_logger_pop_prefix(NU_LOGGER);
            );

    ibase_l2_destroy(&ibase_l2);
}

/** @} */

}//namespace// //ScenSim-Port://
