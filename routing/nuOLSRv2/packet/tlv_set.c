#include "config.h"

#include "packet/tlv_set.h"
#include "packet/debug_util.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_tlv_set
 * @{
 */

/** Compares tlv_sets.
 *
 * @param a
 * @param b
 * @retval true  if a == b
 * @retval false if a != b
 */
PUBLIC nu_bool_t
nu_tlv_set_eq(const nu_tlv_set_t* a, const nu_tlv_set_t* b)
{
    nu_tlv_t* pa = nu_tlv_set_iter(a);
    nu_tlv_t* pb = nu_tlv_set_iter(b);
    while (1) {
        if (nu_tlv_set_iter_is_end(pa, a) &&
            nu_tlv_set_iter_is_end(pb, b))
            return true;
        if (nu_tlv_set_iter_is_end(pa, a) ||
            nu_tlv_set_iter_is_end(pb, b))
            return false;
        if (!nu_tlv_eq(pa, pb))
            return false;
        pa = nu_tlv_set_iter_next(pa, a);
        pb = nu_tlv_set_iter_next(pb, b);
    }
    /* NOTREACH */
}

/** Outputs TLV Set.
 *
 * @param self
 * @param table
 * @param logger
 */
PUBLIC void
nu_tlv_set_put_log(const nu_tlv_set_t* self,
        manet_constant_t* table, nu_logger_t* logger)
{
    if (nu_tlv_set_is_empty(self))
        nu_logger_log(logger, "tlv_set:[]");
    else {
        nu_logger_log(logger, "tlv_set:[");
        nu_logger_push_prefix(logger, PACKET_LOG_INDENT);
        FOREACH_TLV_SET(tlv, self) {
            nu_tlv_put_log(tlv, table, logger);
        }
        nu_logger_pop_prefix(logger);
        nu_logger_log(logger, "]");
    }
}

/** @} */

}//namespace// //ScenSim-Port://
