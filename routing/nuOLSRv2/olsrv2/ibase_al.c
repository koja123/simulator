//
// (AL) Local Attached Network Set
//
#include "config.h"

#include "core/core.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_al
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tuple_al_t
//

/** Destroys and frees tuple_al. */
#define tuple_al_free    nu_mem_free

/*
 */
static inline tuple_al_t*
tuple_al_create(const nu_ip_t net_addr, const uint8_t dist)
{
    tuple_al_t* tuple = nu_mem_alloc(tuple_al_t);
    tuple->net_addr = net_addr;
    tuple->dist = dist;
    tuple->metric = UNDEF_METRIC;
    return tuple;
}

////////////////////////////////////////////////////////////////
//
// ibase_al_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_al_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Initialized ibase_al.
 */
PUBLIC void
ibase_al_init(void)
{
    _ibase_al_init(IBASE_AL);
}

/** Destroys ibase_al
 */
PUBLIC void
ibase_al_destroy(void)
{
    _ibase_al_destroy(IBASE_AL);
}

/** Adds the new tuple.
 *
 * @param net_addr
 * @param dist
 * @return the new tuple
 */
PUBLIC tuple_al_t*
ibase_al_add(const nu_ip_t net_addr, const int dist)
{
    tuple_al_t* tuple = tuple_al_create(net_addr, (uint8_t)dist);
    _ibase_al_insert_tail(IBASE_AL, tuple);
    return tuple;
}

/** Checks whether the ibase_al contains the net_addr.
 *
 * @param net_addr
 * @return true if ibase_al contains.
 */
PUBLIC nu_bool_t
ibase_al_contain(const nu_ip_t net_addr)
{
    FOREACH_AL(p) {
        if (nu_ip_eq(net_addr, p->net_addr))
            return true;
    }
    return false;
}

/** Removes the net_addr from the ibase_al.
 *
 * @param net_addr
 */
PUBLIC void
ibase_al_remove_net_addr(const nu_ip_t net_addr)
{
    FOREACH_AL(p) {
        if (nu_ip_eq(net_addr, p->net_addr)) {
            _ibase_al_iter_remove(p, IBASE_AL);
        }
    }
}

/** Outputs ibase_al.
 *
 * @param logger
 */
PUBLIC void
ibase_al_put_log(nu_logger_t* logger)
{
    nu_logger_log(logger, "I_AL:%d:--", ibase_al_size());
    FOREACH_AL(p) {
        nu_logger_log(logger,
                "I_AL:[net=%I,dist=%d]", p->net_addr, p->dist);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
