#include "config.h"

#include "core/core.h"
#include "packet/packet.h"
#include "packet/decoder.h"

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

/** Read data.
 *
 * @param name
 * @param type
 * @param var
 * @param args
 */
#if defined(_WIN32) || defined(_WIN64)                                 //ScenSim-Port://
#define read_buf(name, type, var, ...)                               \
    do {                                                             \
        if (!nu_ibuf_get_ ## type(self->buf, var, ## __VA_ARGS__)) { \
            nu_strbuf_append_cstr(&self->err, name " read error");   \
            goto error;                                              \
        }                                                            \
    }                                                                \
    while (0)                                                          //ScenSim-Port://
#else                                                                  //ScenSim-Port://
#define read_buf(name, type, var, args ...)                        \
    do {                                                           \
        if (!nu_ibuf_get_ ## type(self->buf, var, ## args)) {      \
            nu_strbuf_append_cstr(&self->err, name " read error"); \
            goto error;                                            \
        }                                                          \
    }                                                              \
    while (0)
#endif                                                                 //ScenSim-Port://

static nu_bool_t decode_tlv_block(nu_decoder_t*, nu_tlv_set_t*);
static nu_bool_t decode_tlv(nu_decoder_t*, nu_tlv_set_t*);

static nu_bool_t decode_atb(nu_decoder_t*, nu_atb_t*);
static nu_bool_t decode_addr_block(nu_decoder_t*, nu_atb_t*);
static nu_bool_t decode_tlv_block_in_atb(nu_decoder_t*, nu_atb_t*);
static nu_bool_t decode_tlv_in_atb(nu_decoder_t*, nu_atb_t*);

/** Initializes decoder.
 *
 * @param self
 */
PUBLIC void
nu_decoder_init(nu_decoder_t* self)
{
    nu_strbuf_init(&self->err);
}

/** Destroys and frees decoder.
 *
 * @param self
 */
PUBLIC void
nu_decoder_destroy(nu_decoder_t* self)
{
    nu_strbuf_destroy(&self->err);
}

/** Sets ip type.
 *
 * @param self
 * @param type
 */
PUBLIC void
nu_decoder_set_ip_type(nu_decoder_t* self, uint8_t type)
{
    self->type = type;
}

/** Decode packet header.
 *
 * @param self
 * @param buf
 * @return pkthdr or NULL
 */

/*
 * <packet> :=
 *		{<pkt-header><pad-octet>*}?
 *		{<message><pad-octet>*}*
 *
 * <pkt-header> :=
 *		<zero>
 *		<pkt-flags>
 *		<pkt-version>?
 *		<pkt-size>?
 *		<pkt-seq-num>?
 *		<tlv-block>?
 */
PUBLIC nu_pkthdr_t*
nu_decoder_decode_pkthdr(nu_decoder_t* self, nu_ibuf_t* buf)
{
    self->buf = buf;
    DEBUG_MESSAGE_PACKING_PUSH_PREFIX("DECODING_PKTHDR:");

    nu_pkthdr_t* hdr = NULL;

    nu_strbuf_clear(&self->err);

    hdr = nu_pkthdr_create();
    read_buf("<pkt-flags>", u8, &hdr->flags);
    hdr->version = (hdr->flags >> 4);
    hdr->flags = hdr->flags & 0x0f;

    DEBUG_MESSAGE_PACKING_SEM(pktflags, hdr->flags);

    if (nu_pkthdr_has_seqnum(hdr)) {
        read_buf("<pkt-seqnum>", u16, &hdr->seqnum);
        DEBUG_MESSAGE_PACKING_LOG("seqnum:%d", hdr->seqnum);
    }
    if (nu_pkthdr_has_tlv(hdr)) {
        if (!decode_tlv_block(self, &hdr->tlv_set))
            goto error;
    }

    DEBUG_MESSAGE_PACKING_POP_PREFIX();
    return hdr;

error:
    if (hdr)
        nu_pkthdr_free(hdr);
    nu_logger_clear_prefix(NU_LOGGER);
    nu_warn("PacketHeader decode error:%s", nu_strbuf_cstr(&self->err));
    return NULL;
}

/** Decode message.
 *
 * @param self
 * @param buf
 * @return msg or NULL
 */

/*
 * <message> :=
 *		<msg-header>
 *		<msg-body>
 */
PUBLIC nu_msg_t*
nu_decoder_decode_message(nu_decoder_t* self, nu_ibuf_t* buf)
{
    nu_msg_t* msg = nu_decoder_decode_msghdr(self, buf);
    if (!msg)
        return NULL;
    if (!nu_decoder_decode_msg_body(self, msg)) {
        nu_msg_free(msg);
        return NULL;
    }
    return msg;
}

/** Decode message body.
 *
 * @param self
 * @param msg
 * @return true if success
 */

/*
 * <msg-body> :=
 *		<tlv-block>
 *		<addr-tlv-block>*
 */
PUBLIC nu_bool_t
nu_decoder_decode_msg_body(nu_decoder_t* self, nu_msg_t* msg)
{
    assert(!msg->body_decoded);
    self->buf = msg->body_ibuf;

    DEBUG_MESSAGE_PACKING_PUSH_PREFIX("DECODING_MSGBODY:");

    if (!decode_tlv_block(self, &msg->msg_tlv_set))
        goto error;
    while (nu_ibuf_remain(self->buf)) {
        if (!decode_atb(self, nu_msg_add_atb(msg)))
            goto error;
    }

    msg->body_decoded = true;

    DEBUG_MESSAGE_PACKING_POP_PREFIX();
    return true;

error:
    nu_logger_clear_prefix(NU_LOGGER);
    nu_warn("Message body decode error:%s", nu_strbuf_cstr(&self->err));
    return false;
}

/** Decode message header.
 *
 * @param self
 * @param buf
 * @return msg or NULL
 */

/*
 * <msg-header> :=
 *		<msg-type>
 *		<msg-flags(4bits)>
 *		<msg-addr_length(4bits)>
 *		<msg-size>
 *		<msg-orig-addr>?
 *		<msg-hop-limit>?
 *		<msg-hop-count>?
 *		<msg-seq-num>?
 */
PUBLIC nu_msg_t*
nu_decoder_decode_msghdr(nu_decoder_t* self, nu_ibuf_t* buf)
{
    self->buf = buf;
    size_t   buf_top = nu_ibuf_ptr(self->buf);
    uint16_t size = 0;
    uint8_t  t = 0;

    if (nu_ibuf_remain(self->buf) == 0)
        return NULL;

    nu_msg_t* msg = nu_msg_create();

    DEBUG_MESSAGE_PACKING_PUSH_PREFIX("DECODING_MSGHDR:");

    read_buf("<msg-type>", u8, &msg->type);
    DEBUG_MESSAGE_PACKING_LOG("type:%d", msg->type);

    read_buf("<msg-flags><msg-addr-length>", u8, &t);
    msg->addr_len = (t & 0x0f) + 1;
    msg->flags = (t >> 4);
    DEBUG_MESSAGE_PACKING_SEM(msgflags, msg->flags);

    read_buf("<msg-size>", u16, &size);
    if (size == 0) {
        nu_strbuf_append_cstr(&self->err, "message size is zero");
        goto error;
    }
    msg->size = size;
    if (nu_msg_has_orig_addr(msg)) {
        if (NU_IS_V4)
            read_buf("<msg-orig-addr>", ip4, &msg->orig_addr);
        else
            read_buf("<msg-orig-addr>", ip6, &msg->orig_addr);
        nu_ip_set_default_prefix(&msg->orig_addr);
        DEBUG_MESSAGE_PACKING_LOG("orig:%I", msg->orig_addr);
    }
    if (nu_msg_has_hop_limit(msg)) {
        read_buf("<msg-hop-limit>", u8, &msg->hop_limit);
        DEBUG_MESSAGE_PACKING_LOG("hop-limit:%d", msg->hop_limit);
    }
    if (nu_msg_has_hop_count(msg)) {
        read_buf("<msg-hop-count>", u8, &msg->hop_count);
        DEBUG_MESSAGE_PACKING_LOG("hop-count:%d", msg->hop_count);
    }
    if (nu_msg_has_seqnum(msg)) {
        read_buf("<msg-seqnum>", u16, &msg->seqnum);
        DEBUG_MESSAGE_PACKING_LOG("seqnum:0x%04x", msg->seqnum);
    }

    msg->body_decoded = false;
    msg->body_ibuf = nu_ibuf_create_child(self->buf,
            size - (nu_ibuf_ptr(self->buf) - buf_top));

    DEBUG_MESSAGE_PACKING_POP_PREFIX();
    return msg;

error:
    nu_logger_clear_prefix(NU_LOGGER);
    nu_warn("Message header decode error:%s", nu_strbuf_cstr(&self->err));
    return NULL;
}

/*
 * <tlv-block> :=
 *		<tlv-length>
 *		<tlv>*
 */
static nu_bool_t
decode_tlv_block(nu_decoder_t* self, nu_tlv_set_t* tlvs)
{
    DEBUG_MESSAGE_PACKING_LOG("decoding tlv-block");
    DEBUG_MESSAGE_PACKING_PUSH_PREFIX(PACKET_LOG_INDENT);

    uint16_t tlv_length = 0;
    read_buf("<tlv-length>", u16, &tlv_length);
    DEBUG_MESSAGE_PACKING_LOG("tlv-length:0x%04x", tlv_length);

    if (tlv_length != 0) {
        size_t buf_top = nu_ibuf_ptr(self->buf);

        if (nu_ibuf_remain(self->buf) < tlv_length) {
            nu_strbuf_append_cstr(&self->err, "<tlv-length> is too big");
            goto error;
        }

        while (nu_ibuf_ptr(self->buf) - buf_top < tlv_length) {
            if (!decode_tlv(self, tlvs))
                goto error;
        }
    }

    DEBUG_MESSAGE_PACKING_POP_PREFIX();

    return true;

error:
    nu_strbuf_append_cstr(&self->err, ":<tlv-block>");
    return false;
}

/*
 * <tlv> :=
 *		<type>
 *		<tlv-flags>
 *		<type-ext>?
 *		<index-start>?
 *		<index-stop>?
 *		<length>?
 *		<value>?
 */
static nu_bool_t
decode_tlv(nu_decoder_t* self, nu_tlv_set_t* tlvs)
{
    uint8_t  type = 0;
    uint8_t  type_ext = 0;
    uint8_t  flags = 0;
    uint16_t len = 0;
    uint8_t* value = 0;

    DEBUG_MESSAGE_PACKING_LOG("decoding tlv");
    DEBUG_MESSAGE_PACKING_PUSH_PREFIX(PACKET_LOG_INDENT);

    read_buf("<tlv-type>", u8, &type);
    DEBUG_MESSAGE_PACKING_LOG("type:%d", type);

    read_buf("<tlv-flags>", u8, &flags);
    DEBUG_MESSAGE_PACKING_SEM(tlvflags, flags);

    if (tlvflags_has_type_ext(flags)) {
        read_buf("<type-ext>", u8, &type_ext);
        DEBUG_MESSAGE_PACKING_LOG("type-ext:%d", type_ext);
    }
    if (tlvflags_has_index(flags)) {
        nu_strbuf_append_cstr(&self->err,
                "(message|packet) tlv must not have index");
        goto error;
    }
    if (tlvflags_has_value(flags)) {
        if (tlvflags_has_8bit_length(flags)) {
            uint8_t len8 = 0;
            read_buf("<tlv-length> (8bit)", u8, &len8);
            len = len8;
            DEBUG_MESSAGE_PACKING_LOG("len(8bit):%d", len);
        } else {
            read_buf("<tlv-length> (16bit)", u16, &len);
            DEBUG_MESSAGE_PACKING_LOG("len(16bit):%d", len);
        }
    }
    if (len > 0) {
        value = (uint8_t*)nu_mem_calloc(len, sizeof(uint8_t));
        read_buf("<value>", bytes, value, len);
        DEBUG_MESSAGE_PACKING_LOG("value:0x%V", value, len);
    }

    nu_tlv_set_add(tlvs, nu_tlv_create(type, type_ext, value, len));
    if (value)
        nu_mem_free(value);

    DEBUG_MESSAGE_PACKING_POP_PREFIX();
    return true;

error:
    if (value)
        nu_mem_free(value);
    nu_strbuf_append_cstr(&self->err, ":<tlv>");
    return false;
}

/*
 * <addr-tlv-block> :=
 *		<addr-block> <tlv-block>
 */
static nu_bool_t
decode_atb(nu_decoder_t* self, nu_atb_t* atb)
{
    if (!decode_addr_block(self, atb))
        goto error;
    if (!decode_tlv_block_in_atb(self, atb))
        goto error;
    return true;

error:
    nu_strbuf_append_cstr(&self->err, ":<addr-tlv-block>");
    return false;
}

/*
 * <addr-block> :=
 *		<num-addr>
 *		<addr-flags>
 *		<head-length>?
 *		<head>?
 *		<tail-length>?
 *		<tail>?
 *		<mid>*
 *		<prefix-length>*
 */
static nu_bool_t
decode_addr_block(nu_decoder_t* self, nu_atb_t* atb)
{
    const size_t ip_len = (self->type == NU_IP_TYPE_V4)
                          ? NU_IP4_LEN : NU_IP6_LEN;
    uint8_t num_addr = 0;
    uint8_t flags = 0;
    uint8_t head_length = 0;
    uint8_t tail_length = 0;
    uint8_t mid_length  = 0;
#if defined(_WIN32) || defined(_WIN64)   //ScenSim-Port://
    uint8_t* head = new uint8_t[ip_len]; //ScenSim-Port://
    uint8_t* tail = new uint8_t[ip_len]; //ScenSim-Port://
    uint8_t* mid = new uint8_t[ip_len];  //ScenSim-Port://
#else                                    //ScenSim-Port://
    uint8_t head[ip_len];
    uint8_t tail[ip_len];
    uint8_t mid[ip_len];
#endif                                   //ScenSim-Port://

    DEBUG_MESSAGE_PACKING_LOG("decoding addr-block");
    DEBUG_MESSAGE_PACKING_PUSH_PREFIX(PACKET_LOG_INDENT);

    read_buf("<num-addr>", u8, &num_addr);
    DEBUG_MESSAGE_PACKING_LOG("num:%d", num_addr);

    read_buf("<addr-flags>", u8, &flags);
    DEBUG_MESSAGE_PACKING_SEM(addrflags, flags);

    // head
    if (addrflags_has_head(flags)) {
        read_buf("<head-length>", u8, &head_length);
        DEBUG_MESSAGE_PACKING_LOG("head-length:%d", head_length);

        read_buf("<head>", bytes, head, head_length);
        DEBUG_MESSAGE_PACKING_LOG("head:0x%V", head, head_length);
    }

    // tail
    if (addrflags_has_tail(flags)) {
        read_buf("<tail-length>", u8, &tail_length);
        DEBUG_MESSAGE_PACKING_LOG("tail-length:%d", tail_length);

        if (addrflags_has_zero_tail(flags)) {
            memset(tail, 0, tail_length);
            DEBUG_MESSAGE_PACKING_LOG("zerotail");
        } else {
            read_buf("<tail>", bytes, tail, tail_length);
            DEBUG_MESSAGE_PACKING_LOG("tail:0x%V", tail, tail_length);
        }
    }

#if defined(_WIN32) || defined(_WIN64)                 //ScenSim-Port://
    if ((size_t)head_length + tail_length >= ip_len) { //ScenSim-Port://
#else                                                  //ScenSim-Port://
    if (head_length + tail_length >= ip_len) {
#endif                                                 //ScenSim-Port://
        nu_strbuf_append_cstr(&self->err,
                "head_length + tail_length >= IP_LEN");
        goto error;
    }

    {                           // <mid>
        mid_length = uint8_t(ip_len - head_length - tail_length);
        for (size_t i = 0; i < num_addr; ++i) {
            read_buf("<mid>", bytes, mid, mid_length);
            nu_ip_t ip = nu_ip_create_with_hmt(head, head_length,
                    tail, tail_length, mid, mid_length);
            nu_ip_set_prefix(&ip, (uint8_t)(ip_len * 8));
            nu_atb_add_ip(atb, ip);
        }
    }

    // <prefix-length>
    if (addrflags_has_no_prelen(flags)) {
    } else if (addrflags_has_single_prelen(flags)) {
        uint8_t prefix = 0;
        read_buf("<prefix>", u8, &prefix);
        FOREACH_ATB(p, atb) {
            nu_ip_set_prefix(&p->ip, prefix);
        }
        DEBUG_MESSAGE_PACKING_LOG("single-prefix:%d", prefix);
    } else if (addrflags_has_multi_prelen(flags)) {
        uint8_t prefix = 0;
        FOREACH_ATB(p, atb) {
            read_buf("<prefix>", u8, &prefix);
            nu_ip_set_prefix(&p->ip, prefix);
            DEBUG_MESSAGE_PACKING_LOG("multi-prefix:%d", prefix);
        }
    } else {
        nu_strbuf_append_cstr(&self->err, "prelen flag combination error");
        goto error;
    }

    DEBUG_MESSAGE_PACKING(
            FOREACH_ATB(p, atb) {
                nu_logger_log(NU_LOGGER, "addr:%I", p->ip);
            }
            );

    DEBUG_MESSAGE_PACKING_POP_PREFIX();
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    delete[] head;                     //ScenSim-Port://
    delete[] tail;                     //ScenSim-Port://
    delete[] mid;                      //ScenSim-Port://
#endif                                 //ScenSim-Port://
    return true;

error:
    nu_strbuf_append_cstr(&self->err, ":<address-block>");
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    delete[] head;                     //ScenSim-Port://
    delete[] tail;                     //ScenSim-Port://
    delete[] mid;                      //ScenSim-Port://
#endif                                 //ScenSim-Port://
    return false;
}

/*
 * <tlv-block> :=
 *		<tlv-length> <tlv>*
 */
static nu_bool_t
decode_tlv_block_in_atb(nu_decoder_t* self, nu_atb_t* atb)
{
    DEBUG_MESSAGE_PACKING_LOG("decoding tlv-block in atb");
    DEBUG_MESSAGE_PACKING_PUSH_PREFIX(PACKET_LOG_INDENT);

    uint16_t tlv_length = 0;
    read_buf("<tlv-length>", u16, &tlv_length);
    DEBUG_MESSAGE_PACKING_LOG("tlv-length:0x%04x", tlv_length);

    if (tlv_length != 0) {
        size_t buf_top = nu_ibuf_ptr(self->buf);

        if (nu_ibuf_remain(self->buf) < tlv_length) {
            nu_strbuf_append_cstr(&self->err,
                    "<tlv-length> is too big");
            goto error;
        }

        while (nu_ibuf_ptr(self->buf) - buf_top < tlv_length) {
            if (!decode_tlv_in_atb(self, atb))
                goto error;
        }
    }

    DEBUG_MESSAGE_PACKING_POP_PREFIX();
    return true;

error:
    nu_strbuf_append_cstr(&self->err, ":<tlv-block>");
    return false;
}

/*
 * <tlv> :=
 *		<type>
 *		<tlv-flags>
 *		<ext-type>?
 *		<index-start>?
 *		<index-stop>?
 *		<length>?
 *		<value>?
 */
static nu_bool_t
decode_tlv_in_atb(nu_decoder_t* self, nu_atb_t* atb)
{
    uint8_t  type = 0;
    uint8_t  type_ext = 0;
    uint8_t  flags = 0;
    uint16_t len = 0;
    size_t   start = 0;
    size_t   stop  = nu_atb_size(atb) - 1;
    uint8_t* value = NULL;
    size_t   num = stop - start + 1;
    size_t   inc = 0;

    DEBUG_MESSAGE_PACKING_LOG("decoding tlv", type);
    DEBUG_MESSAGE_PACKING_PUSH_PREFIX(PACKET_LOG_INDENT);

    read_buf("<tlv-type>", u8, &type);
    DEBUG_MESSAGE_PACKING_LOG("type:%d", type);

    read_buf("<tlv-flags>", u8, &flags);
    DEBUG_MESSAGE_PACKING_SEM(tlvflags, flags);

    if (tlvflags_has_type_ext(flags)) {
        read_buf("<type-ext>", u8, &type_ext);
        DEBUG_MESSAGE_PACKING_LOG("type-ext:%d", type_ext);
    }

    if (tlvflags_has_no_index(flags)) {
    } else if (tlvflags_has_single_index(flags)) {
        uint8_t s = 0;
        read_buf("<index-start>", u8, &s);
        start = s;
        stop  = s;
        DEBUG_MESSAGE_PACKING_LOG("index-start:%d", start);
    } else if (tlvflags_has_multi_index(flags)) {
        uint8_t s = 0;
        read_buf("<index-start>", u8, &s);
        start = s;
        read_buf("<index-stop>", u8, &s);
        stop = s;
        DEBUG_MESSAGE_PACKING_LOG("index-start:%d index-stop:%d",
                start, stop);
    } else {
        nu_strbuf_append_cstr(&self->err,
                "<index-*> flags error");
        goto error;
    }

    if (start > stop) {
        nu_strbuf_append_cstr(&self->err,
                "index-start must be <= index-stop");
        goto error;
    }
    if (stop >= nu_atb_size(atb)) {
        nu_strbuf_append_cstr(&self->err,
                "index-stop must be <= the number of address");
        goto error;
    }

    num = stop - start + 1;

    if (tlvflags_has_value(flags)) {
        if (tlvflags_has_8bit_length(flags)) {
            uint8_t len8 = 0;
            read_buf("<length>", u8, &len8);
            len = len8;
            DEBUG_MESSAGE_PACKING_LOG("length(8bit):%d", len);
        } else {
            read_buf("<length>", u16, &len);
            DEBUG_MESSAGE_PACKING_LOG("length(16bit):%d", len);
        }
    }

    value = (uint8_t*)nu_mem_calloc(len, sizeof(uint8_t));
    if (len > 0) {
        read_buf("<value>", bytes, value, len);
        // Expand TLVs
        if (tlvflags_has_multi_value(flags)) {
            inc = len / num;
            len = (uint16_t)(len / num);
        }
        DEBUG_MESSAGE_PACKING_LOG("value:0x%V", value, len);
    }
    {
        nu_atb_iter_t p = nu_atb_iter(atb);
        for (size_t s = 0; s < start; ++s)
            p = nu_atb_iter_next(p, atb);
        for (size_t i = 0; i < num; ++i) {
            nu_tlv_set_add(&p->tlv_set,
                    nu_tlv_create(type, type_ext,
                            value + i * inc, len));
            p = nu_atb_iter_next(p, atb);
        }
    }
    if (value)
        nu_mem_free(value);

    DEBUG_MESSAGE_PACKING_POP_PREFIX();
    return true;

error:
    nu_strbuf_append_cstr(&self->err, ":<tlv-block>");
    return false;
}

/** @} */

}//namespace// //ScenSim-Port://
