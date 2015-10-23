#ifndef OLSRV2_LM_TABLE_H_
#define OLSRV2_LM_TABLE_H_

#include "config.h" //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup  olsrv2-metric OLSRv2 static link metric :: Static link metric
 * @{
 */


/** */
struct metric;

/**
 * link metric list
 */
typedef struct metric_list {
#if defined(_WIN32) || defined(_WIN64)        //ScenSim-Port://
    struct _metric* next;   ///< next element //ScenSim-Port://
    struct _metric* prev;   ///< prev element //ScenSim-Port://
#else                                         //ScenSim-Port://
    struct metric* next;    ///< next element
    struct metric* prev;    ///< prev element
#endif                                        //ScenSim-Port://
    size_t         n;       ///< the number of elements.
} metric_list_t;

struct tuple_i;
PUBLIC void olsrv2_metric_list_init(void);
PUBLIC void olsrv2_metric_list_destroy(void);
PUBLIC void olsrv2_metric_list_load(const char*);
PUBLIC nu_link_metric_t olsrv2_metric_list_get(struct tuple_i*, const nu_ip_t);
PUBLIC void olsrv2_metric_list_put_log(nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
