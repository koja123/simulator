#include "config.h"

#include <math.h>

#include "core/core.h"
#include "packet/msg.h"
#include "packet/msgflags.h"
#include "packet/debug_util.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_msg
 * @{
 */

/** Checks whether msg has originator address.
 *
 * @param self
 * @return true if msg has originator address
 */
PUBLIC nu_bool_t
nu_msg_has_orig_addr(const nu_msg_t* self)
{
    return bit_is_set(self->flags, M_ORIG);
}

/** Checks whether msg has hop limit.
 *
 * @param self
 * @return true if msg has hop limit
 */
PUBLIC nu_bool_t
nu_msg_has_hop_limit(const nu_msg_t* self)
{
    return bit_is_set(self->flags, M_HOP_LIMIT);
}

/** Checks whether msg has hop count.
 *
 * @param self
 * @return true if msg has hop count
 */
PUBLIC nu_bool_t
nu_msg_has_hop_count(const nu_msg_t* self)
{
    return bit_is_set(self->flags, M_HOP_COUNT);
}

/** Checks whether msg has sequence number.
 *
 * @param self
 * @return true if msg has sequence number
 */
PUBLIC nu_bool_t
nu_msg_has_seqnum(const nu_msg_t* self)
{
    return bit_is_set(self->flags, M_SEQNUM);
}

/** Initializes msg.
 *
 * @param self
 */
PUBLIC void
nu_msg_init(nu_msg_t* self)
{
    self->type = 0;
    self->addr_len  = (NU_IS_V4 ? NU_IP4_LEN : NU_IP6_LEN);
    self->hop_limit = 0;
    self->hop_count = 0;
    self->seqnum = 0;
    self->size = 0;

    self->flags = 0;
    nu_msgflags_set_no_orig_addr(self->flags);
    nu_msgflags_set_no_hop_limit(self->flags);
    nu_msgflags_set_no_hop_count(self->flags);
    nu_msgflags_set_no_seqnum(self->flags);

    self->body_decoded = false;
    self->body_ibuf = NULL;

    nu_tlv_set_init(&self->msg_tlv_set);
    nu_atb_list_init(&self->atb_list);
    self->cur_atb = NULL;
}

/** Creates msg.
 *
 * @return msg
 */
PUBLIC nu_msg_t*
nu_msg_create(void)
{
    nu_msg_t* self = nu_mem_alloc(nu_msg_t);
    nu_msg_init(self);
    return self;
}

/** Destroys msg.
 *
 * @param self
 */
PUBLIC void
nu_msg_destroy(nu_msg_t* self)
{
    nu_tlv_set_destroy(&self->msg_tlv_set);
    nu_atb_list_destroy(&self->atb_list);
    if (self->body_ibuf)
        nu_ibuf_free(self->body_ibuf);
}

/** Destroys and frees msg.
 *
 * @param self
 */
PUBLIC void
nu_msg_free(nu_msg_t* self)
{
    nu_msg_destroy(self);
    nu_mem_free(self);
}

/** Sets originator address.
 *
 * @param self
 * @param ip
 */
PUBLIC void
nu_msg_set_orig_addr(nu_msg_t* self, const nu_ip_t ip)
{
    nu_msgflags_set_orig_addr(self->flags);
    self->orig_addr = ip;
    self->addr_len  = (uint8_t)(nu_ip_len(ip));
}

/** Sets hop limit.
 *
 * @param self
 * @param limit
 */
PUBLIC void
nu_msg_set_hop_limit(nu_msg_t* self, const uint8_t limit)
{
    nu_msgflags_set_hop_limit(self->flags);
    self->hop_limit = limit;
}

/** Sets hop count.
 *
 * @param self
 * @param hc
 */
PUBLIC void
nu_msg_set_hop_count(nu_msg_t* self, const uint8_t hc)
{
    nu_msgflags_set_hop_count(self->flags);
    self->hop_count = hc;
}

/** Sets sequence number.
 *
 * @param self
 * @param sn
 */
PUBLIC void
nu_msg_set_seqnum(nu_msg_t* self, const uint16_t sn)
{
    nu_msgflags_set_seqnum(self->flags);
    self->seqnum = sn;
}

/** Adds message tlv.
 *
 * @param self
 * @param tlv
 * @return inserted tlv
 */
PUBLIC nu_tlv_t*
nu_msg_add_msg_tlv(nu_msg_t* self, nu_tlv_t* tlv)
{
    return nu_tlv_set_add(&self->msg_tlv_set, tlv);
}

/** Adds new atb.
 *
 * @param self
 * @return atb
 */
PUBLIC nu_atb_t*
nu_msg_add_atb(nu_msg_t* self)
{
    return(self->cur_atb = nu_atb_list_add(&self->atb_list));
}

/**
 * Add IP address to current address tlv block (atb).
 * If atb already has the IP address, do nothing.
 *
 * @param self
 * @param ip
 * @see nu_msg_add_ip
 */
PUBLIC void
nu_msg_add_ip(nu_msg_t* self, const nu_ip_t ip)
{
    nu_atb_add_ip(self->cur_atb, ip);
}

/**
 * Add IP address-TLV pair to the current address tlv block (atb).
 * If atb does not has IP address, add IP address to current atb, first.
 *
 * @param self
 * @param ip
 * @param tlv	this tlv is used for tlv_set. DO NOT CHANGE OR FREE this tlv.
 *
 * @see nu_atb_add_ip_tlv
 */
PUBLIC void
nu_msg_add_ip_tlv(nu_msg_t* self, const nu_ip_t ip, nu_tlv_t* tlv)
{
    nu_atb_add_ip_tlv(self->cur_atb, ip, tlv);
}

/** Removes the ip and assigned TLVs from the message.
 *
 * @param self
 * @param ip
 */
PUBLIC void
nu_msg_remove_ip(nu_msg_t* self, const nu_ip_t ip)
{
    nu_atb_list_t* atb_list = &self->atb_list;
    for (nu_atb_list_iter_t p = nu_atb_list_iter(atb_list);
         !nu_atb_list_iter_is_end(p, atb_list);) {
        nu_atb_t* atb = &p->atb;
        nu_atb_remove_ip(atb, ip);
        if (nu_atb_size(atb) == 0)
            p = nu_atb_list_iter_remove(p, atb_list);
        else
            p = nu_atb_list_iter_next(p, atb_list);
    }
}

/** Searches ip.
 *
 * @param self
 * @param ip
 * @return atb_elt or NULL
 */
PUBLIC nu_atb_elt_t*
nu_msg_search_ip(nu_msg_t* self, const nu_ip_t ip)
{
    FOREACH_ATB_LIST(p, self) {
        nu_atb_t*     atb = &p->atb;
        nu_atb_iter_t r = nu_atb_search_ip(atb, ip);
        if (!nu_atb_iter_is_end(r, atb))
            return r;
    }
    return NULL;
}

/** Searches tlv.
 *
 * @param self
 * @param ip
 * @param type
 * @return tlv or NULL
 */
PUBLIC nu_tlv_t*
nu_msg_search_ip_tlv(nu_msg_t* self, const nu_ip_t ip, const uint8_t type)
{
    FOREACH_ATB_LIST(p, self) {
        nu_atb_t* atb = &p->atb;
        nu_tlv_t* tlv = nu_atb_search_ip_tlv_type(atb, ip, type);
        if (tlv != NULL)
            return tlv;
    }
    return NULL;
}

/** Compare msgs.
 *
 * @param ma
 * @param mb
 * @return true if ma == mb
 */
PUBLIC nu_bool_t
nu_msg_eq(const nu_msg_t* ma, const nu_msg_t* mb)
{
    if (ma->type != mb->type)
        return false;
    if (ma->flags != mb->flags)
        return false;
    if (ma->addr_len != mb->addr_len)
        return false;
    if (nu_msg_has_orig_addr(ma) && ma->orig_addr != mb->orig_addr)
        return false;
    if (nu_msg_has_hop_limit(ma) && ma->hop_limit != mb->hop_limit)
        return false;
    if (nu_msg_has_hop_count(ma) && ma->hop_count != mb->hop_count)
        return false;
    if (nu_msg_has_seqnum(ma) && ma->seqnum != mb->seqnum)
        return false;
    if (!nu_tlv_set_eq(&ma->msg_tlv_set, &mb->msg_tlv_set))
        return false;
    if (!nu_atb_list_eq(&ma->atb_list, &mb->atb_list))
        return false;
    return true;
}

/** Outputs message.
 *
 * @param self
 * @param logger
 */
PUBLIC void
nu_msg_put_log(const nu_msg_t* self, nu_logger_t* logger)
{
    nu_logger_log(logger, "msg:[type:%d", self->type);
    nu_logger_push_prefix(logger, PACKET_LOG_INDENT);

    static char buf[80];
    msgflags_to_s(self->flags, buf, sizeof(buf));
    nu_logger_log(logger, "flags:%s", buf);
    nu_logger_log(logger, "addr_len:%d", self->addr_len);

    if (nu_msg_has_orig_addr(self))
        nu_logger_log(logger, "orig_addr:%I", self->orig_addr);
    if (nu_msg_has_hop_limit(self))
        nu_logger_log(logger, "hop_limit:%d", self->hop_limit);
    if (nu_msg_has_hop_count(self))
        nu_logger_log(logger, "hop_count:%d", self->hop_count);
    if (nu_msg_has_seqnum(self))
        nu_logger_log(logger, "seqnum:%hd", self->seqnum);

    nu_tlv_set_put_log(&self->msg_tlv_set, manet_msg_tlv, logger);

    FOREACH_ATB_LIST(p, self) {
        nu_atb_put_log(nu_atb_list_iter_atb(p), logger);
    }

    nu_logger_pop_prefix(logger);
}

/** @} */

}//namespace// //ScenSim-Port://
