#include "config.h"

#include "core/core.h"
#include "packet/atb.h"
#include "packet/debug_util.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_atb
 * @{
 */

/*
 */
static inline nu_atb_elt_t*
nu_atb_elt_create(const nu_ip_t ip)
{
    nu_atb_elt_t* r = nu_mem_alloc(nu_atb_elt_t);
    r->next = r->prev = NULL;
    r->ip = ip;
    nu_tlv_set_init(&r->tlv_set);
    return r;
}

/** Destroys and frees the atb_elt.
 *
 * @param self
 */
PUBLIC void
nu_atb_elt_free(nu_atb_elt_t* self)
{
    self->next = self->prev = NULL;
    self->ip = NU_IP_UNDEF;
    nu_tlv_set_destroy(&self->tlv_set);
    nu_mem_free(self);
}

/** Adds ip.
 *
 * @param self
 * @param ip
 * @return atb_iter
 */
PUBLIC nu_atb_iter_t
nu_atb_add_ip(nu_atb_t* self, const nu_ip_t ip)
{
    // Search IP
    FOREACH_ATB(p, self) {
        if (nu_ip_eq(ip, nu_atb_iter_ip(p)))
            return p;
    }
    nu_atb_elt_t* entry = nu_atb_elt_create(ip);
    nu_atb_iter_insert_before((nu_atb_elt_t*)self, self, entry);
    return (nu_atb_iter_t)entry;
}

/** Adds tlv to associated tlv_set.
 *
 * If target tlv_set contains same tlv,
 * this function frees given tlv and returns the existing tlv.
 *
 * @param self
 * @param ip
 * @param tlv
 * @return inserted tlv.
 */
PUBLIC nu_tlv_t*
nu_atb_add_ip_tlv(nu_atb_t* self, nu_ip_t ip, nu_tlv_t* tlv)
{
    // Search IP
    FOREACH_ATB(p, self) {
        if (nu_ip_eq(ip, nu_atb_iter_ip(p)))
            return nu_atb_iter_add_tlv(p, tlv);
    }
    nu_atb_elt_t* entry = nu_atb_elt_create(ip);
    nu_atb_insert_tail(self, entry);
    return nu_atb_iter_add_tlv(self->prev, tlv);
}

/** Adds tlv to associated tlv_set.
 *
 * If target tlv_set contains same tlv,
 * this function frees given tlv and returns the existing tlv.
 *
 * @param self
 * @param idx
 * @param tlv
 * @return inserted tlv.
 */
PUBLIC nu_tlv_t*
nu_atb_add_tlv_at(nu_atb_t* self, const size_t idx,
        nu_tlv_t* tlv)
{
    size_t i = 0;
    FOREACH_ATB(p, self) {
        if (i++ == idx)
            return nu_atb_iter_add_tlv(p, tlv);
    }
    nu_fatal("Out of bounds");
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    return NULL;                       //ScenSim-Port://
#endif                                 //ScenSim-Port://
}

/** Removes the ip from the atb.
 *
 * @param self
 * @param ip
 * @return true if atb is modified
 */
PUBLIC nu_bool_t
nu_atb_remove_ip(nu_atb_t* self, const nu_ip_t ip)
{
    // Search IP
    FOREACH_ATB(p, self) {
        if (nu_ip_eq(ip, nu_atb_iter_ip(p))) {
            nu_atb_iter_remove(p, self);
            return true;
        }
    }
    return false;
}

/** Removes tlv.
 *
 * @param self
 * @param ip
 * @param tlv_type
 * @return true if atb is modified
 */
PUBLIC nu_bool_t
nu_atb_remove_ip_tlv_type(nu_atb_t* self, nu_ip_t ip,
        const uint8_t tlv_type)
{
    // Search IP
    FOREACH_ATB(p, self) {
        if (nu_ip_eq(ip, nu_atb_iter_ip(p)))
            return nu_atb_iter_remove_tlv_type(p, tlv_type);
    }
    return false;
}

/** Removes tlv.
 *
 * @param self
 * @param idx
 * @param tlv_type
 * @return true if atb is modified
 */
PUBLIC nu_bool_t
nu_atb_remove_tlv_at(nu_atb_t* self, const size_t idx,
        const uint8_t tlv_type)
{
    size_t i = 0;
    FOREACH_ATB(p, self) {
        if (i++ == idx)
            return nu_atb_iter_remove_tlv_type(p, tlv_type);
    }
    nu_fatal("Out of bounds");
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    return false;                      //ScenSim-Port://
#endif                                 //ScenSim-Port://
}

/** Checks whether the atb contains the ip.
 *
 * @param self
 * @param ip
 * @return true if self contains the ip
 */
PUBLIC nu_bool_t
nu_atb_contain_ip(const nu_atb_t* self, const nu_ip_t ip)
{
    FOREACH_ATB(p, self) {
        if (ip == nu_atb_iter_ip(p))
            return true;
    }
    return false;
}

/** Searches atb by ip.
 *
 * @param self
 * @param ip
 * @return atb_iter or atb_iter_end.
 */
PUBLIC nu_atb_iter_t
nu_atb_search_ip(nu_atb_t* self, const nu_ip_t ip)
{
    FOREACH_ATB(p, self) {
        if (ip == nu_atb_iter_ip(p))
            return p;
    }
    return nu_atb_iter_end(self);
}

/** Searches tlv by ip and type.
 *
 * @param self
 * @param ip
 * @param tlv_type
 * @return tlv or NULL
 */
PUBLIC nu_tlv_t*
nu_atb_search_ip_tlv_type(const nu_atb_t* self, const nu_ip_t ip,
        const uint8_t tlv_type)
{
    FOREACH_ATB(p, self) {
        if (ip == nu_atb_iter_ip(p)) {
            return nu_tlv_set_search_type(nu_atb_iter_tlv_set(p),
                    tlv_type);
        }
    }
    return NULL;
}

/** Compare atbs.
 *
 * @param a
 * @param b
 * @return true if a == b
 */
PUBLIC nu_bool_t
nu_atb_eq(const nu_atb_t* a, const nu_atb_t* b)
{
    if (nu_atb_size(a) != nu_atb_size(b))
        return false;

    nu_atb_iter_t p = nu_atb_iter(a);
    nu_atb_iter_t q = nu_atb_iter(b);
    while (!nu_atb_iter_is_end(p, a)) {
        if (!nu_ip_eq(nu_atb_iter_ip(p),
                    nu_atb_iter_ip(q)))
            return false;
        if (!nu_tlv_set_eq(nu_atb_iter_tlv_set(p),
                    nu_atb_iter_tlv_set(q)))
            return false;
        p = nu_atb_iter_next(p, a);
        q = nu_atb_iter_next(q, b);
    }
    return true;
}

/** Output address tlv block.
 *
 * @param self
 * @param logger
 */
PUBLIC void
nu_atb_put_log(const nu_atb_t* self, nu_logger_t* logger)
{
    if (nu_atb_is_empty(self))
        nu_logger_log(logger, "atb:[]");
    else {
        nu_logger_log(logger, "atb:[");
        nu_logger_push_prefix(logger, PACKET_LOG_INDENT);
        FOREACH_ATB(p, self) {
            nu_logger_log(logger, "[ip:%I", nu_atb_iter_ip(p));
            nu_logger_push_prefix(logger, PACKET_LOG_INDENT);
            nu_tlv_set_put_log(nu_atb_iter_tlv_set(p), manet_addr_tlv, logger);
            nu_logger_pop_prefix(logger);
            nu_logger_log(logger, "]");
        }
        nu_logger_pop_prefix(logger);
        nu_logger_log(logger, "]");
    }
}

/** @} */

}//namespace// //ScenSim-Port://
