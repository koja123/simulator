//
// (AR) Advertising Remote Router
//
#ifndef OLSRV2_IBASE_AR_H_
#define OLSRV2_IBASE_AR_H_

#include "olsrv2/ibase_time.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_ar OLSRv2 :: (AR) Advertising Remote Router Tuple
 * @{
 */

/**
 *  (AR) Advertising Remote Router Tuple
 */
typedef struct tuple_ar {
    struct tuple_ar*  next;         ///< next tuple
    struct tuple_ar*  prev;         ///< prev tuple

    tuple_time_t*     timeout_next; ///< next in timeout list
    tuple_time_t*     timeout_prev; ///< prev in timeout list
    tuple_time_proc_t timeout_proc; ///< timeout processor
    nu_time_t         time;         ///< timeout time

    nu_ip_t           orig_addr;    ///< AR_orig_addr
    uint16_t          seqnum;       ///< AR_seq_number
} tuple_ar_t;

/**
 *  (AR) Advertising Remote Router Set
 */
typedef struct ibase_ar {
    tuple_ar_t* next;   ///< first tuple
    tuple_ar_t* prev;   ///< last tuple
    size_t      n;      ///< size
    nu_bool_t   change; ///< change flag
} ibase_ar_t;

////////////////////////////////////////////////////////////////
//
// ibase_ar_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_ar_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC void ibase_ar_init(void);
PUBLIC void ibase_ar_destroy(void);

PUBLIC void ibase_ar_clear_change_flag(void);

PUBLIC nu_bool_t ibase_ar_is_changed(void);
PUBLIC tuple_ar_t* ibase_ar_add(const nu_ip_t, const uint16_t);
PUBLIC tuple_ar_t* ibase_ar_search_orig_addr(const nu_ip_t);
PUBLIC tuple_ar_t* ibase_ar_iter_remove(tuple_ar_t*);

PUBLIC void ibase_ar_put_log(nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
