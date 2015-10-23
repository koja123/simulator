#include "config.h"

#include "olsrv2/olsrv2.h"
#include "olsrv2/lifo.h"
#include "olsrv2/debug_util.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup @olsrv2-etx
 * @{
 */

/** HELLO timeout event for ETX.
 *
 * @param ev
 */
PUBLIC void
etx_hello_timeout_event(nu_event_t* ev)
{
    tuple_l_t* tuple_l = (tuple_l_t*)ev->param;
#if defined(_WIN32) || defined(_WIN64)                           //ScenSim-Port://
    NU_DEBUG_PROCESS("etx_hello_timeout:%s:%S",                  //ScenSim-Port://
            tuple_l->tuple_i->name, &tuple_l->neighbor_ip_list); //ScenSim-Port://
#else                                                            //ScenSim-Port://
    DEBUG_PROCESS("etx_hello_timeout:%s:%S",
            tuple_l->tuple_i->name, &tuple_l->neighbor_ip_list);
#endif                                                           //ScenSim-Port://
    tuple_l->metric_lost_hellos += 1;
    tuple_l->metric_hello_timeout_event =
        nu_event_list_add(METRIC_EVENT_LIST,
                etx_hello_timeout_event, tuple_l,
                tuple_l->metric_hello_interval,
                NU_EVENT_PRIO__DEFAULT,
                "etx_hello_timeout_event");
}

/** Calculates link metrics, periodically.
 *
 * @param ev
 */
PUBLIC void
etx_calc_metric_event(nu_event_t* ev)
{
#if defined(_WIN32) || defined(_WIN64)    //ScenSim-Port://
    NU_DEBUG_PROCESS0("etx_calc_metric"); //ScenSim-Port://
#else                                     //ScenSim-Port://
    DEBUG_PROCESS0("etx_calc_metric");
#endif                                    //ScenSim-Port://
    OLSRV2_DO_LOG(link_metric,
            nu_logger_push_prefix(NU_LOGGER, "ETX:");
            );

    FOREACH_I(tuple_i) {
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            double sum_received = lifo_sum(&tuple_l->metric_received_lifo);
            double sum_total = lifo_sum(&tuple_l->metric_total_lifo);
            if (tuple_l->metric_hello_interval != UNDEF_INTERVAL &&
                tuple_l->metric_lost_hellos > 0) {
                double penalty = tuple_l->metric_hello_interval *
                                 tuple_l->metric_lost_hellos / ETX_MEMORY_LENGTH;
                sum_received = sum_received - sum_received * penalty;
            }
            if (sum_received < 1) {
                tuple_l->metric_r_etx = UNDEF_ETX;
                tuple_l_set_in_metric(tuple_l, MAXIMUM_METRIC);
            } else {
                tuple_l->metric_r_etx = sum_total / sum_received;
                if (tuple_l->metric_d_etx == UNDEF_ETX)
                    tuple_l_set_in_metric(tuple_l, DEFAULT_METRIC);
                else {
                    tuple_l_set_in_metric(tuple_l,
                            ETX_PERFECT_METRIC * tuple_l->metric_r_etx * tuple_l->metric_d_etx);
                }
            }
            OLSRV2_DO_LOG(link_metric,
                    nu_logger_log(NU_LOGGER,
                            "I:%s L:(%S) in:%g (%g/%g) d_etx=%g r_etx=%g", tuple_i->name,
                            &tuple_l->neighbor_ip_list, tuple_l->in_metric,
                            sum_received, sum_total, tuple_l->metric_d_etx, tuple_l->metric_r_etx);
                    //nu_logger_tagf(NU_LOGGER, "receved_lifo:\n");
                    //lifo_put_log(&tuple_l->metric_received_lifo, NU_LOGGER);
                    //nu_logger_tagf(NU_LOGGER, "total_lifo:\n");
                    //lifo_put_log(&tuple_l->metric_total_lifo, NU_LOGGER);
                    );
            lifo_push(&tuple_l->metric_total_lifo, 0);
            lifo_push(&tuple_l->metric_received_lifo, 0);
        }
    }

    OLSRV2_DO_LOG(link_metric,
            nu_logger_pop_prefix(NU_LOGGER);
            );
}

/** @} */

}//namespace// //ScenSim-Port://
