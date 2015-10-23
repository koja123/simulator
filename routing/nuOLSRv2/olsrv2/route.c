//
// Routing table calculation
//
#include "config.h"

#include "olsrv2/olsrv2.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup olsrv2
 * @{
 */

/*
 */
static void
route_add_neighbor_routers(void)
{
    OLSRV2_DO_LOG(calc_route,
            nu_logger_push_prefix(NU_LOGGER, "ADD_NR:");
            );
    FOREACH_N(tuple_n) {
        if (!tuple_n->symmetric)
            continue;
        tuple_l_t* tuple_l = ibase_l_search_tuple_n(tuple_n);
        ibase_r_add(tuple_n->orig_addr,
                nu_ip_set_head(&tuple_l->neighbor_ip_list),
                tuple_n->_out_metric, 1, tuple_l->tuple_i);
    }
    OLSRV2_DO_LOG(calc_route,
            nu_logger_pop_prefix(NU_LOGGER);
            );
}

/*
 */
static void
route_add_neighbor_addresses(void)
{
    if (ibase_n_size() == 0)
        return;
    OLSRV2_DO_LOG(calc_route,
            nu_logger_push_prefix(NU_LOGGER, "ADD_NA:");
            ibase_n_put_log(NU_LOGGER);
            );
    FOREACH_N(tuple_n) {
        if (!tuple_n->symmetric)
            continue;
        tuple_l_t* tuple_l = ibase_l_search_tuple_n(tuple_n);
        FOREACH_IP_SET(i, &tuple_n->neighbor_ip_list) {
            if (ibase_r_search_dest(i->ip) == NULL) {
                ibase_r_add(i->ip,
                        nu_ip_set_head(&tuple_l->neighbor_ip_list),
                        tuple_n->_out_metric, 1, tuple_l->tuple_i);
            }
        }
    }
    OLSRV2_DO_LOG(calc_route,
            nu_logger_pop_prefix(NU_LOGGER);
            );
}

#if 0

/*
 */
static int
route_add_ibase_tr(void)
{
    if (ibase_tr_size() == 0)
        return 1;
    OLSRV2_DO_LOG(calc_route,
            nu_logger_push_prefix(NU_LOGGER, "TR:");
            ibase_tr_put_log(NU_LOGGER);
            );
    tuple_tr_t* save_tr[ibase_tr_size()];
    size_t save_tr_size = 0;
    FOREACH_TR(tuple_tr) {
        save_tr[save_tr_size++] = tuple_tr;
    }
    int h = 1;
    nu_bool_t changed = true;
    while (changed && save_tr_size != 0) {
        changed = false;
        for (size_t i = 0; i < save_tr_size;) {
            tuple_tr_t* tuple_tr = save_tr[i];
            if (!ibase_r_contain_dest(tuple_tr->to_orig_addr)) {
                tuple_r_t* prev_r = ibase_r_search_dest_and_dist(
                        tuple_tr->from_orig_addr, h);
                if (prev_r == NULL) {
                    ++i;
                    continue;
                }
                ibase_r_add(tuple_tr->to_orig_addr, prev_r->next_ip,
                        tuple_tr->metric + prev_r->metric,
                        h + 1, prev_r->tuple_i);
                changed = true;
            }
            memmove(&save_tr[i], &save_tr[i + 1],
                    (save_tr_size - i - 1) * sizeof(tuple_tr_t*));
            --save_tr_size;
        }
        ++h;
    }
    OLSRV2_DO_LOG(calc_route,
            nu_logger_pop_prefix(NU_LOGGER);
            );

    return h;
}

#endif

/*
 */
static void
route_add_ibase_ta(void)
{
    if (ibase_ta_size() == 0)
        return;
    OLSRV2_DO_LOG(calc_route,
            nu_logger_push_prefix(NU_LOGGER, "ADD_TA:");
            ibase_ta_put_log(NU_LOGGER);
            );
    nu_bool_t changed = true;
    while (changed) {
        changed = false;
        FOREACH_TA(tuple_ta) {
            tuple_r_t* prev_r = ibase_r_search_dest(tuple_ta->from_orig_addr);
            if (prev_r == NULL)
                continue;
            tuple_r_t* tuple_r = ibase_r_search_dest(tuple_ta->dest_ip);
            if (tuple_r == NULL) {
                OLSRV2_DO_LOG(calc_route,
                        nu_logger_push_prefix(NU_LOGGER, "prev_r ");
                        tuple_r_put_log(prev_r, NU_LOGGER);
                        nu_logger_pop_prefix(NU_LOGGER);
                        nu_logger_push_prefix(NU_LOGGER, "tuple_ta ");
                        tuple_ta_put_log(tuple_ta, NU_LOGGER);
                        nu_logger_pop_prefix(NU_LOGGER);
                        );
                ibase_r_add(tuple_ta->dest_ip, prev_r->next_ip,
                        tuple_ta->metric + prev_r->metric,
                        prev_r->dist + 1, prev_r->tuple_i);
                changed = true;
            } else {
                if (tuple_ta->metric + prev_r->metric < tuple_r->metric) {
                    OLSRV2_DO_LOG(calc_route,
                            nu_logger_push_prefix(NU_LOGGER, "prev_r ");
                            tuple_r_put_log(prev_r, NU_LOGGER);
                            nu_logger_pop_prefix(NU_LOGGER);
                            nu_logger_push_prefix(NU_LOGGER, "tuple_ta ");
                            tuple_ta_put_log(tuple_ta, NU_LOGGER);
                            nu_logger_pop_prefix(NU_LOGGER);
                            nu_logger_push_prefix(NU_LOGGER, "tuple_r ");
                            tuple_r_put_log(tuple_r, NU_LOGGER);
                            nu_logger_pop_prefix(NU_LOGGER);
                            );
                    ibase_r_delete(tuple_r);
                    ibase_r_add(tuple_ta->dest_ip, prev_r->next_ip,
                            tuple_ta->metric + prev_r->metric,
                            prev_r->dist + 1, prev_r->tuple_i);
                    changed = true;
                }
            }
        }
    }
    OLSRV2_DO_LOG(calc_route,
            nu_logger_pop_prefix(NU_LOGGER);
            );
}

/*
 */
static void
route_add_ibase_n2()
{
    OLSRV2_DO_LOG(calc_route,
            nu_logger_push_prefix(NU_LOGGER, "ADD_N2:");
            );
    FOREACH_I(tuple_i) {
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            if (tuple_l->status != LINK_STATUS__SYMMETRIC)
                continue;
            OLSRV2_DO_LOG(calc_route,
                    tuple_l_put_log(tuple_l, NU_LOGGER);
                    );
            tuple_n_t* tuple_n = ibase_n_search_tuple_l(tuple_l);
            if (tuple_n->willingness == WILLINGNESS__NEVER)
                continue;
            OLSRV2_DO_LOG(calc_route,
                    ibase_n2_put_log(&tuple_l->ibase_n2, tuple_i,
                            tuple_l, NU_LOGGER);
                    );
            FOREACH_N2(tuple_n2, &tuple_l->ibase_n2) {
                tuple_r_t* prev_r = ibase_r_search_dest_ip_set(
                        &tuple_l->neighbor_ip_list);
                if (prev_r == NULL)
                    continue;
                nu_link_metric_t metric =
                    tuple_l->out_metric + tuple_n2->out_metric;
                tuple_r_t* tuple_r = ibase_r_search_dest(tuple_n2->hop2_ip);
                if (tuple_r == NULL) {
                    ibase_r_add(tuple_n2->hop2_ip, prev_r->next_ip,
                            metric, prev_r->dist + 1, prev_r->tuple_i);
                } else {
                    if (metric < tuple_r->metric) {
                        ibase_r_delete(tuple_r);
                        ibase_r_add(tuple_n2->hop2_ip, prev_r->next_ip,
                                metric, prev_r->dist + 1, prev_r->tuple_i);
                    }
                }
            }
        }
    }
    OLSRV2_DO_LOG(calc_route,
            nu_logger_pop_prefix(NU_LOGGER);
            );
}

/*
 */
static void
route_add_ibase_an(void)
{
    if (ibase_an_size() == 0)
        return;
    OLSRV2_DO_LOG(calc_route,
            nu_logger_push_prefix(NU_LOGGER, "ADD_AN:");
            ibase_an_put_log(NU_LOGGER);
            );

    FOREACH_AN(tuple_an) {
        tuple_r_t* prev_r = ibase_r_search_dest(tuple_an->orig_addr);
        if (prev_r == NULL)
            continue;
        tuple_r_t* tuple_r = ibase_r_search_dest(tuple_an->net_addr);
        if (tuple_r == NULL) {
            ibase_r_add(tuple_an->net_addr, prev_r->next_ip,
                    tuple_an->metric + prev_r->metric,
                    prev_r->dist + tuple_an->dist,
                    prev_r->tuple_i);
        } else if (tuple_r->dist > prev_r->dist + tuple_an->dist) {
            ibase_r_delete(tuple_r);
            ibase_r_add(tuple_an->net_addr, prev_r->next_ip,
                    tuple_an->metric + prev_r->metric,
                    prev_r->dist + tuple_an->dist,
                    prev_r->tuple_i);
        }
    }
    OLSRV2_DO_LOG(calc_route,
            nu_logger_pop_prefix(NU_LOGGER);
            );
}

/*
 */
static inline void
route_add_remote_routers(void)
{
    OLSRV2_DO_LOG(calc_route,
            nu_logger_push_prefix(NU_LOGGER, "ADD_TR:");
            ibase_an_put_log(NU_LOGGER);
            );
    int h = 1;
    nu_bool_t updated = true;
    while (updated) {
        updated = false;
        FOREACH_TR(tuple_tr) {
            tuple_r_t* prev =
                ibase_r_search_dest_and_dist(tuple_tr->from_orig_addr, static_cast<uint8_t>(h));
            if (prev == NULL)
                continue;
            tuple_r_t* dest =
                ibase_r_search_dest(tuple_tr->to_orig_addr);
            if (dest != NULL) {
                if (dest->metric > tuple_tr->metric + prev->metric) {
                    ibase_r_delete(dest);
                    ibase_r_add(tuple_tr->to_orig_addr, prev->next_ip,
                            tuple_tr->metric + prev->metric,
                            prev->dist + 1, prev->tuple_i);
                    updated = true;
                }
            } else {
                ibase_r_add(tuple_tr->to_orig_addr, prev->next_ip,
                        tuple_tr->metric + prev->metric,
                        prev->dist + 1, prev->tuple_i);
                updated = true;
            }
        }
        h += 1;
    }
    OLSRV2_DO_LOG(calc_route,
            nu_logger_pop_prefix(NU_LOGGER);
            );
}

/** Calculates the routing table.
 */
PUBLIC void
olsrv2_route_calc(void)
{
    OLSRV2_DO_LOG(calc_route,
            nu_logger_push_prefix(NU_LOGGER, "CALC_ROUTE:");
            );

    ibase_r_reset();
    route_add_neighbor_routers();   // C.2
    route_add_ibase_n2();
    route_add_remote_routers();     // C.3
    route_add_neighbor_addresses(); // C.4
    route_add_ibase_ta();           // C.5
    route_add_ibase_an();           // C.6

    OLSRV2_DO_LOG(calc_route,
            nu_logger_pop_prefix(NU_LOGGER);
            );
    OLSRV2_DO_LOG(calc_route,
            nu_logger_push_prefix(NU_LOGGER, "ROUTE_RESULT:");
            ibase_r_put_log(NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );

    ibase_r_update();
}

/** @} */

}//namespace// //ScenSim-Port://
