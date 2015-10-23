//
// (AL) Local Attached Network Set
//
#ifndef OLSRV2_IBASE_AL_H_
#define OLSRV2_IBASE_AL_H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_al OLSRv2 :: (AL) Local Attached Network Tuple
 * @{
 */

/**
 * (AL) Local Attached Network Tuple
 */
typedef struct tuple_al {
    struct tuple_al* next;     ///< next
    struct tuple_al* prev;     ///< prev

    nu_ip_t          net_addr; ///< AL_net addr
    uint8_t          dist;     ///< AL_dist
    nu_link_metric_t metric;   ///< AL_metric
} tuple_al_t;

/**
 * (AL) Local Attached Network Set
 */
typedef struct ibase_al {
    tuple_al_t* next;     ///< first tuple
    tuple_al_t* prev;     ///< last tuple
    size_t      n;        ///< size
    nu_bool_t   change;   ///< change flag
} ibase_al_t;

////////////////////////////////////////////////////////////////
//
// ibase_al_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_al_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC void ibase_al_init(void);
PUBLIC void ibase_al_destroy(void);

PUBLIC tuple_al_t* ibase_al_add(const nu_ip_t, const int);
PUBLIC nu_bool_t ibase_al_contain(const nu_ip_t);

PUBLIC void ibase_al_put_log(nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
