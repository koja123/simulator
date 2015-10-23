#include "config.h"

#include "core/core.h"
#include "packet/packet.h"
#include "packet/encoder.h"

#include "packet/pktflags.h"
#include "packet/msgflags.h"
#include "packet/addrflags.h"
#include "packet/tlvflags.h"

#include "packet/debug_util.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_encoder_decoder
 * @{
 */

/** Appends data output buffer.
 *
 * @param name
 * @param type
 * @param var
 * @param args
 */
#if defined(_WIN32) || defined(_WIN64)                                    //ScenSim-Port://
#define write_buf(name, type, var, ...)                                 \
    do {                                                                \
        if (!nu_obuf_append_ ## type(self->buf, var, ## __VA_ARGS__)) { \
            nu_strbuf_append_cstr(&self->err,                           \
                    name " write error");                               \
            goto error;                                                 \
        }                                                               \
    }                                                                   \
    while (0)                                                             //ScenSim-Port://
#else                                                                     //ScenSim-Port://
#define write_buf(name, type, var, args ...)                     \
    do {                                                         \
        if (!nu_obuf_append_ ## type(self->buf, var, ## args)) { \
            nu_strbuf_append_cstr(&self->err,                    \
                    name " write error");                        \
            goto error;                                          \
        }                                                        \
    }                                                            \
    while (0)
#endif                                                                    //ScenSim-Port://

static nu_bool_t encode_tlv_block(nu_encoder_t*, const nu_tlv_set_t*);
static nu_bool_t encode_addr_tlv_block(nu_encoder_t*,
        const nu_atb_t*);
static nu_bool_t encode_addr_block(nu_encoder_t*, const nu_atb_t*);
static nu_bool_t encode_tlv_block_in_atb(nu_encoder_t*, const nu_atb_t*);
static nu_bool_t encode_tlv(nu_encoder_t*,
        const uint8_t type, const uint8_t type_ext,
        const uint8_t flags, const size_t start, const size_t stop,
        const uint8_t* value, const size_t value_len);

/** Initializes encoder.
 *
 * @param self
 */
PUBLIC void
nu_encoder_init(nu_encoder_t* self)
{
    nu_strbuf_init(&self->err);
}

/** Destroys encoder.
 *
 * @param self
 */
PUBLIC void
nu_encoder_destroy(nu_encoder_t* self)
{
    nu_strbuf_destroy(&self->err);
}

/** Encodes packet header.
 *
 * @param self
 * @param pkthdr
 * @return obuf
 */

/* <packet> :=
 *      {<pkt-header><pad-octet>*}?
 *      {<message><pad-octet>*}*
 *
 * <pkt-header> :=
 *      <version(4bits)>
 *      <pkg-flags(4bits)>
 *      <pkt-seq-num(16bit)>?
 *      <tlv-block>?
 */
PUBLIC nu_obuf_t*
nu_encoder_encode_pkthdr(nu_encoder_t* self, nu_pkthdr_t* pkthdr)
{
    nu_strbuf_clear(&self->err);
    self->buf = nu_obuf_create();

    DEBUG_MESSAGE_PACKING_PUSH_PREFIX("ENCODING_PKTHDR:");

    uint8_t flags = (pkthdr->version << 4) | (0x0f & pkthdr->flags);
    write_buf("<version><pkt-flags>", u8, flags);
    DEBUG_MESSAGE_PACKING_SEM(pktflags, flags);

    if (nu_pkthdr_has_seqnum(pkthdr)) {
        write_buf("<pkt-seq-num>", u16, pkthdr->seqnum);
        DEBUG_MESSAGE_PACKING_LOG("seqnum:0x%04x", pkthdr->seqnum);
    }
    if (nu_pkthdr_has_tlv(pkthdr)) {
        if (!encode_tlv_block(self, &pkthdr->tlv_set))
            goto error;
    }

    DEBUG_MESSAGE_PACKING_POP_PREFIX();

    return self->buf;

error:
    nu_logger_clear_prefix(NU_LOGGER);
    nu_warn("PacketHeader encode error:%s", nu_strbuf_cstr(&self->err));
    return self->buf;
}

/** Encodes message.
 *
 * @param self
 * @param msg
 * @return obuf
 */

/* <message> :=
 *      <msg-type>
 *      <msg-flags>
 *      <msg-version>?
 *      <msg-size>
 *      <msg-orig-addr>?
 *      <msg-hop-limit>?
 *      <msg-hop-count>?
 *      <msg-seq-num>?
 *      <tlv-block>
 *      <addr-tlv-block>*
 */
PUBLIC nu_obuf_t*
nu_encoder_encode_message(nu_encoder_t* self, nu_msg_t* msg)
{
    nu_strbuf_clear(&self->err);
    self->buf = nu_obuf_create();

    DEBUG_MESSAGE_PACKING_PUSH_PREFIX("ENCODING_MSG:");

    size_t  buf_top = nu_obuf_ptr(self->buf);
    uint8_t t;

    write_buf("<msg-type>", u8, msg->type);
    DEBUG_MESSAGE_PACKING_LOG("type:%d", msg->type);

    t = (msg->flags << 4) | (0x0f & (msg->addr_len - 1));
    write_buf("<msg-flags><msg-addr-length>", u8, t);
    DEBUG_MESSAGE_PACKING_SEM(msgflags, msg->flags);
    {
        size_t size_idx = nu_obuf_ptr(self->buf);
        write_buf("<msg-size>", u16, 0);

        if (nu_msg_has_orig_addr(msg)) {
            if (nu_ip_is_v4(msg->orig_addr))
                write_buf("<msg-orig-addr>", ip4, msg->orig_addr);
            else
                write_buf("<msg-orig-addr>", ip6, msg->orig_addr);
            DEBUG_MESSAGE_PACKING_LOG("orig:%I", msg->orig_addr);
        }
        if (nu_msg_has_hop_limit(msg)) {
            write_buf("<msg-hop-limit>", u8, msg->hop_limit);
            DEBUG_MESSAGE_PACKING_LOG("hop-limit:%d", msg->hop_limit);
        }
        if (nu_msg_has_hop_count(msg)) {
            write_buf("<msg-hop-count>", u8, msg->hop_count);
            DEBUG_MESSAGE_PACKING_LOG("hop-count:%d", msg->hop_count);
        }
        if (nu_msg_has_seqnum(msg)) {
            write_buf("<msg-seqnum>", u16, msg->seqnum);
            DEBUG_MESSAGE_PACKING_LOG("seqnum:0x%04x", msg->seqnum);
        }

        if (msg->body_ibuf == NULL) {
            if (!encode_tlv_block(self, &msg->msg_tlv_set))
                goto error;

            FOREACH_ATB_LIST(pATB, msg) {
                if (!encode_addr_tlv_block(self, nu_atb_list_iter_atb(pATB)))
                    goto error;
            }
        } else {
            if (!nu_obuf_append_ibuf(self->buf, msg->body_ibuf)) {
                nu_strbuf_append_cstr(&self->err, "body_buf append error");
                goto error;
            }
        }

        size_t size;
        if ((size = nu_obuf_ptr(self->buf) - buf_top) >= 0x10000) {
            nu_strbuf_append_cstr(&self->err, "message size is too long");
            goto error;
        }
        if (!nu_obuf_put_u16(self->buf, size_idx, (uint16_t)(size))) {
            nu_strbuf_append_cstr(&self->err, "<msg-size> write error");
            goto error;
        }
        DEBUG_MESSAGE_PACKING_LOG("msg-size:0x%04x", size);

        DEBUG_MESSAGE_PACKING_POP_PREFIX();
    }
    NU_PACKET_DO_LOG(message_hexdump,
            nu_logger_push_prefix(NU_LOGGER, "MSGDUMP:");
            nu_obuf_put_log(self->buf, NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );

    return self->buf;

error:
    nu_logger_clear_prefix(NU_LOGGER);
    nu_warn("Message encode error:%s", nu_strbuf_cstr(&self->err));
    return self->buf;
}

/*
 * <tlv-block> :=
 *      <tlv-length>
 *      <tlv>*
 */
static nu_bool_t
encode_tlv_block(nu_encoder_t* self, const nu_tlv_set_t* tlv_set)
{
    size_t buf_top = nu_obuf_ptr(self->buf);
    size_t size = 0;

    DEBUG_MESSAGE_PACKING_LOG("encoding tlv-block");

    write_buf("<tlv-length>", u16, 0);

    DEBUG_MESSAGE_PACKING_PUSH_PREFIX(PACKET_LOG_INDENT);
    FOREACH_TLV_SET(tlv, tlv_set) {
        uint8_t flags = 0;
        tlvflags_set_no_index(flags);
        if (!encode_tlv(self, tlv->type, tlv->type_ext, flags, 0, 0,
                    tlv->value, tlv->length))
            goto error;
    }

    size = nu_obuf_ptr(self->buf) - buf_top - 2;
    if (size >= 0x10000) {
        nu_strbuf_append_cstr(&self->err, "tlv-length is too long : %x");
        goto error;
    }
    if (!nu_obuf_put_u16(self->buf, buf_top, (uint16_t)(size))) {
        nu_strbuf_append_cstr(&self->err, "<tlv-length> write error");
        goto error;
    }

    DEBUG_MESSAGE_PACKING_LOG("tlv-length:0x%04x", size);

    DEBUG_MESSAGE_PACKING_POP_PREFIX();
    return true;

error:
    nu_strbuf_append_cstr(&self->err, ":<tlv-block>");
    return false;
}

/*
 * <addr-tlv-block> :=
 *      <addr-block>
 *      <tlv-block>
 */
static nu_bool_t
encode_addr_tlv_block(nu_encoder_t* self, const nu_atb_t* atb)
{
    if (nu_atb_size(atb) == 0)
        return true;

    if (!encode_addr_block(self, atb))
        goto error;
    if (!encode_tlv_block_in_atb(self, atb))
        goto error;
    return true;

error:
    nu_strbuf_append_cstr(&self->err, ":<addr-tlv-block>");
    return false;
}

/*
 * <addr-block> :=
 *      <num-addr>
 *      <addr-flags>
 *      (<head-length><head>?)?
 *      (<tail-length><tail>?)?
 *      <mid>*
 *      <prefix-length>*
 */
static nu_bool_t
encode_addr_block(nu_encoder_t* self, const nu_atb_t* atb)
{
    size_t  head_len = 0;
    size_t  tail_len = 0;
    size_t  mid_len  = 0;
    uint8_t flags = 0;

    nu_atb_iter_t p = nu_atb_iter(atb);
    if (nu_atb_iter_is_end(p, atb))
        return true;

    DEBUG_MESSAGE_PACKING_LOG("encoding addr-block");
    DEBUG_MESSAGE_PACKING_PUSH_PREFIX(PACKET_LOG_INDENT);

    nu_ip_t top_ip = p->ip;
    const nu_ip_addr_t* top_ipa = nu_ip_addr(top_ip);
    const size_t ip_len = (nu_ip_is_v4(top_ip) ? NU_IP4_LEN : NU_IP6_LEN);

    const int addr_num = (int)(nu_atb_size(atb));
    if (addr_num == 1) {
        head_len = 0;
        tail_len = 0;
        addrflags_set_no_head(flags);
        addrflags_set_no_full_tail(flags);
        addrflags_set_no_zero_tail(flags);
        if (!nu_ip_is_default_prefix(top_ip))
            addrflags_set_single_prelen(flags);
    } else {
        nu_bool_t single_prefix = true;
        head_len = ip_len;
        tail_len = ip_len;
        for (p = nu_atb_iter_next(p, atb);
             !nu_atb_iter_is_end(p, atb);
             p = nu_atb_iter_next(p, atb)) {
            const nu_ip_t ip = p->ip;
            assert(nu_ip_type(ip) == nu_ip_type(top_ip));
            const size_t h = nu_ip_get_head_length(top_ip, ip);
            const size_t t = nu_ip_get_tail_length(top_ip, ip);
            if (head_len > h)
                head_len = h;
            if (tail_len > t)
                tail_len = t;
            if (single_prefix &&
                nu_ip_prefix(top_ip) != nu_ip_prefix(ip))
                single_prefix = false;
        }

        // XXX
        if (head_len == ip_len) {
            head_len = 0;
            tail_len = 0;
            nu_warn("ATB contains duplicated IP address.");
        }

        if (head_len == 0)
            addrflags_set_no_head(flags);
        else
            addrflags_set_head(flags);

        if (tail_len == 0)
            addrflags_set_no_full_tail(flags);
        else if (nu_ip_is_zero_tail(top_ip, tail_len))
            addrflags_set_zero_tail(flags);
        else
            addrflags_set_full_tail(flags);

        if (single_prefix) {
            if (nu_ip_is_default_prefix(top_ip))
                addrflags_set_no_prelen(flags);
            else
                addrflags_set_single_prelen(flags);
        } else
            addrflags_set_multi_prelen(flags);
    }

    write_buf("<num-addr>", u8, (uint8_t)addr_num);
    DEBUG_MESSAGE_PACKING_LOG("num:%d", addr_num);

    write_buf("<addr-flags>", u8, flags);
    DEBUG_MESSAGE_PACKING_SEM(addrflags, flags);
    assert(!(bit_is_set(flags, A_FULL_TAIL) && bit_is_set(flags, A_ZERO_TAIL)));

    if (addrflags_has_head(flags)) {
        write_buf("<head-length>", u8, (uint8_t)(head_len));
        DEBUG_MESSAGE_PACKING_LOG("head-len:%d", head_len);

        write_buf("<head>", bytes, top_ipa->u8, head_len);
        DEBUG_MESSAGE_PACKING_LOG("head:0x%V", top_ipa->u8, head_len);
    }
    if (addrflags_has_tail(flags)) {
        write_buf("<tail-length>", u8, (uint8_t)(tail_len));
        DEBUG_MESSAGE_PACKING_LOG("tail-len:%d", tail_len);

        if (!addrflags_has_zero_tail(flags)) {
            const uint8_t* t = &top_ipa->u8[ip_len - tail_len];
            write_buf("<tail>", bytes, t, tail_len);
            DEBUG_MESSAGE_PACKING_LOG("tail:0x%V", t, tail_len);
        }
    }
    mid_len = ip_len - head_len - tail_len; // <mid>*
    FOREACH_ATB(p, atb) {
        const nu_ip_t  ip = p->ip;
        const uint8_t* m  = &nu_ip_addr(ip)->u8[head_len];
        write_buf("<mid>", bytes, m, mid_len);
        DEBUG_MESSAGE_PACKING_LOG("mid:0x%V", m, mid_len);
    }

    // <prefix-length>*
    if (addrflags_has_single_prelen(flags)) {
        write_buf("<prefix-length>", u8, nu_ip_prefix(top_ip));
        DEBUG_MESSAGE_PACKING_LOG("prefix-length:%d",
                nu_ip_prefix(top_ip));
    } else if (addrflags_has_multi_prelen(flags)) {
        FOREACH_ATB(p, atb) {
            const nu_ip_t ip = p->ip;
            write_buf("<prefix-length>", u8, nu_ip_prefix(ip));
            DEBUG_MESSAGE_PACKING_LOG("prefix-length:%d",
                    nu_ip_prefix(ip));
        }
    }

    DEBUG_MESSAGE_PACKING_POP_PREFIX();
    return true;

error:
    nu_strbuf_append_cstr(&self->err, ":<addr-block>");
    return false;
}

/*
 * Packing tlv-block in addr-tlv-block.
 *
 * <tlv-block> :=
 *      <tlv-length>
 *      <tlv>*
 */
static nu_bool_t
encode_tlv_block_in_atb(nu_encoder_t* self, const nu_atb_t* atb)
{
    DEBUG_MESSAGE_PACKING_LOG("encoding tlv-block in addr-tlv-block");
    DEBUG_MESSAGE_PACKING_PUSH_PREFIX(PACKET_LOG_INDENT);

    // Creates the set of tlv_set in the atb, and its iterator pair.
    const size_t addr_num = nu_atb_size(atb);
    struct tlv_set_iters {
        nu_tlv_set_t* tlv_set;
        nu_tlv_t*     p;
    }
    ;                                                       //ScenSim-Port://
    typedef struct tlv_set_iters tlv_set_iters_t;           //ScenSim-Port://
#if defined(_WIN32) || defined(_WIN64)                      //ScenSim-Port://
    tlv_set_iters_t* iters = new tlv_set_iters_t[addr_num]; //ScenSim-Port://
#else                                                       //ScenSim-Port://
    tlv_set_iters_t                                         //ScenSim-Port://
    iters[addr_num];
#endif                                                      //ScenSim-Port://
    size_t i = 0;
    FOREACH_ATB(p, atb) {
        iters[i].tlv_set = &p->tlv_set;
        iters[i].p = nu_tlv_set_iter(iters[i].tlv_set);
        ++i;
    }

    nu_obuf_t* value = nu_obuf_create();

    size_t size_idx = nu_obuf_ptr(self->buf);
    size_t buf_top  = 0;
    write_buf("<tlv-length>", u16, 0);
    buf_top = nu_obuf_ptr(self->buf);

    while (1) {
        size_t istart = 0;
        while (istart < addr_num &&
               nu_tlv_set_iter_is_end(iters[istart].p,
                       iters[istart].tlv_set))
            ++istart;

        if (istart == addr_num)
            break;

        nu_tlv_t* tlv = iters[istart].p;
        iters[istart].p =
            nu_tlv_set_iter_next(iters[istart].p, iters[istart].tlv_set);
        nu_bool_t multi_value = false;
        uint8_t   flags = 0;
        nu_obuf_clear(value);
        nu_obuf_append_bytes(value, tlv->value, tlv->length);
        size_t istop = istart + 1;
        while (istop < addr_num &&
               !nu_tlv_set_iter_is_end(iters[istop].p,
                       iters[istop].tlv_set) &&
               tlv->type == iters[istop].p->type &&
               tlv->type_ext == iters[istop].p->type_ext &&
               tlv->length == iters[istop].p->length) {
            nu_tlv_t* tt = iters[istop].p;
            if (memcmp(tlv->value, tt->value, tlv->length) != 0)
                multi_value = true;
            nu_obuf_append_bytes(value, tt->value, tlv->length);
            iters[istop].p = nu_tlv_set_iter_next(iters[istop].p,
                    iters[istop].tlv_set);
            ++istop;
        }
        --istop;

        // index?
        if (istart == 0 && istop == addr_num - 1)
            tlvflags_set_no_index(flags);
        else if (istop == istart)
            tlvflags_set_single_index(flags);
        else
            tlvflags_set_multi_index(flags);

        // value?
        if (multi_value) {
            tlvflags_set_multi_value(flags);
            if (!encode_tlv(self, tlv->type, tlv->type_ext,
                        flags, istart, istop, value->data, value->len))
                goto error;
        } else if (tlv->length != 0) {
            tlvflags_set_single_value(flags);
            if (!encode_tlv(self, tlv->type, tlv->type_ext,
                        flags, istart, istop, value->data, tlv->length))
                goto error;
        } else {
            tlvflags_set_no_value(flags);
            if (!encode_tlv(self, tlv->type, tlv->type_ext,
                        flags, istart, istop, 0, 0))
                goto error;
        }
    }

    {
        size_t size = nu_obuf_ptr(self->buf) - buf_top;
        if (size > 0x10000) {
            nu_strbuf_append_cstr(&self->err, "tlv-length is too long");
            goto error;
        }
        if (!nu_obuf_put_u16(self->buf, size_idx, (uint16_t)(size))) {
            nu_strbuf_append_cstr(&self->err, "<tlv-length> write error");
            goto error;
        }
        DEBUG_MESSAGE_PACKING_LOG("tlv-length:0x%04x", size);
    }
    nu_obuf_free(value);

    DEBUG_MESSAGE_PACKING_POP_PREFIX();
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    delete[] iters;                    //ScenSim-Port://
#endif                                 //ScenSim-Port://
    return true;

error:
    nu_obuf_free(value);
    nu_strbuf_append_cstr(&self->err, ":<tlv-block>");
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    delete[] iters;                    //ScenSim-Port://
#endif                                 //ScenSim-Port://
    return false;
}

/*
 * <tlv> :=
 *      <type>
 *      <tlv-flags>
 *      <type-ext>?
 *      <index-start>?
 *      <index-stop>?
 *      <length>?
 *      <value>?
 */
static nu_bool_t
encode_tlv(nu_encoder_t* self,
        const uint8_t type, const uint8_t type_ext, uint8_t flags,
        const size_t start, const size_t stop,
        const uint8_t* value, const size_t value_len)
{
    DEBUG_MESSAGE_PACKING_LOG("encoding tlv");
    DEBUG_MESSAGE_PACKING_PUSH_PREFIX(PACKET_LOG_INDENT);

    if (value_len == 0)
        tlvflags_set_no_value(flags);
    else if (value_len < 0x100)
        tlvflags_set_8bit_length(flags);
    else if (value_len < 0x10000)
        tlvflags_set_16bit_length(flags);
    else {
        nu_strbuf_append_cstr(&self->err, "tlv value is too long.");
        goto error;
    }

    if (type_ext != 0)
        tlvflags_set_type_ext(flags);

    write_buf("<tlv-type>", u8, type);
    DEBUG_MESSAGE_PACKING_LOG("type:%d", type);

    write_buf("<tlv-flags>", u8, flags);
    DEBUG_MESSAGE_PACKING_SEM(tlvflags, flags);

    if (tlvflags_has_type_ext(flags)) {
        write_buf("<tlv-type-ext>", u8, type_ext);
        DEBUG_MESSAGE_PACKING_LOG("type-ext:%d", type_ext);
    }

    if (tlvflags_has_no_index(flags))
        ;  // do nothing
    else if (tlvflags_has_single_index(flags)) {
        write_buf("<index-start>", u8, (uint8_t)(start));
        DEBUG_MESSAGE_PACKING_LOG("index-start:%d", start);
    } else if (tlvflags_has_multi_index(flags)) {
        write_buf("<index-start>", u8, (uint8_t)(start));
        write_buf("<index-stop>", u8, (uint8_t)(stop));
        DEBUG_MESSAGE_PACKING_LOG("index-start:%d index-stop:%d",
                start, stop);
    } else {
        nu_strbuf_append_cstr(&self->err, "Illegal <index-*> flags");
        goto error;
    }

    if (tlvflags_has_value(flags)) {
        if (tlvflags_has_8bit_length(flags)) {
            write_buf("<length>", u8, (uint8_t)value_len);
            DEBUG_MESSAGE_PACKING_LOG("length(8bit):%u", value_len);
        } else {
            write_buf("<length>", u16, (uint16_t)(value_len));
            DEBUG_MESSAGE_PACKING_LOG("length(16bit):%u", value_len);
        }
        write_buf("<value>", bytes, value, value_len);
        DEBUG_MESSAGE_PACKING_LOG("value:0x%V", value, value_len);
    }

    DEBUG_MESSAGE_PACKING_POP_PREFIX();
    return true;

error:
    nu_strbuf_append_cstr(&self->err, ":<tlv>");
    return false;
}

/** @} */

}//namespace// //ScenSim-Port://
