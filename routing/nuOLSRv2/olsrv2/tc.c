//
// TC Message
//
#include "config.h"

#include <math.h>

#include "core/core.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"

#include "packet/msgtlv.h"
#include "packet/addrtlv.h"

#include "olsrv2/debug_util.h"

#if defined(_WIN32) || defined(_WIN64)                    //ScenSim-Port://
#include <limits>                                         //ScenSim-Port://
#ifndef NAN                                               //ScenSim-Port://
#define NAN      std::numeric_limits<double>::quiet_NaN() //ScenSim-Port://
#endif                                                    //ScenSim-Port://
#define isnan(n) _isnan(n)                                //ScenSim-Port://
#endif                                                    //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup olsrv2
 * @{
 */

// ip_info /////////////////////////////////////////////////

/**
 * Element of tc_ip_list.
 */
typedef struct tc_ip {
    struct tc_ip*    next;   ///< next
    struct tc_ip*    prev;   ///< prev
    nu_ip_t          ip;     ///< ip
    nu_link_metric_t metric; ///< metric
} tc_ip_t;

/*
 */
static inline tc_ip_t*
tc_ip_create(nu_ip_t ip)
{
    tc_ip_t* self = nu_mem_alloc(tc_ip_t);
    self->next = self->prev = NULL;
    self->ip = ip;
    self->metric = DEFAULT_METRIC;
    return self;
}

/*
 */
static inline void
tc_ip_free(tc_ip_t* self)
{
    nu_mem_free(self);
}

/**
 * IP address list in TC
 */
typedef struct tc_ip_list {
    tc_ip_t* next; ///< first element
    tc_ip_t* prev; ///< last element
    size_t   n;    ///< size
} tc_ip_list_t;

}//namespace// //ScenSim-Port://

#include "olsrv2/tc_ip_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

static inline tc_ip_t*
tc_ip_list_search_ip(tc_ip_list_t* list, const nu_ip_t ip)
{
    FOREACH_TC_IP_LIST(i, list) {
        if (nu_ip_eq(i->ip, ip))
            return i;
    }
    return NULL;
}

static void
tc_ip_list_put_log(tc_ip_list_t* list, nu_logger_t* logger)
{
    if (tc_ip_list_is_empty(list)) {
        nu_logger_log(logger, "TcIpList:[]");
    } else {
        nu_logger_log(logger, "TcIpList:[");
        nu_logger_push_prefix(logger, OLSRV2_LOG_INDENT);
        FOREACH_TC_IP_LIST(p, list) {
            nu_logger_log(logger, "[%I metric:%g]", p->ip, p->metric);
        }
        nu_logger_pop_prefix(logger);
        nu_logger_log(logger, "]");
    }
}

////////////////////////////////////////////////////////////////

/** Received TC message parameters
 */
typedef struct tc_param {
    nu_msg_t*    msg;           ///< message
    nu_ip_t      orig_addr;     ///< originator
    double       interval_time; ///< interval time
    double       validity_time; ///< validity time
    uint16_t     ansn;          ///< ansn
    nu_bool_t    has_ansn;      ///< has ansn flag
    uint8_t      cont_type_ext; ///< cont type ext

    nu_ip_set_t  nbr_addr_list; ///< nbr address list
    nu_ip_set_t  gw_list;       ///< gw list

    tc_ip_list_t ip_list;       ///< ip list
} tc_param_t;

#define NBR_ADDR_LIST       (&self->nbr_addr_list) ///< accessor
#define GW_LIST             (&self->gw_list)       ///< accessor
#define TC_IP_LIST          (&self->ip_list)       ///< accessor

/** Code for showing no cont type ext */
#define NO_CONT_TYPE_EXT    (0xff)
#if NO_CONT_TYPE_EXT == CONT_SEQNUM_EXT__COMPLETE || \
    NO_CONT_TYPE_EXT == CONT_SEQNUM_EXT__INCOMPLETE
#error "NO_CONT_TYPE_EXT value must be change."
#endif

static inline tc_param_t*
tc_param_create(nu_msg_t* msg)
{
    tc_param_t* self = nu_mem_alloc(tc_param_t);
    self->msg = msg;
    self->orig_addr = msg->orig_addr;
    self->validity_time = NAN;
    self->interval_time = NAN;
    self->has_ansn = false;
    self->ansn = 0;
    self->cont_type_ext = NO_CONT_TYPE_EXT;
    nu_ip_set_init(NBR_ADDR_LIST);
    nu_ip_set_init(GW_LIST);
    tc_ip_list_init(TC_IP_LIST);
    return self;
}

static inline void
tc_param_free(tc_param_t* self)
{
    tc_ip_list_destroy(TC_IP_LIST);
    nu_ip_set_destroy(GW_LIST);
    nu_ip_set_destroy(NBR_ADDR_LIST);
    nu_mem_free(self);
}

////////////////////////////////////////////////////////////////
//
// Processor
//

/** Prefix of invalid message log */
#define ERR_TC_TAG    "DISCARD TC: "

/** Outputs the invalid message information */
#define ERR_TC0(s)        nu_info(ERR_TC_TAG s)
/** Outputs the invalid message information */
#define ERR_TC(s, ...)    nu_info(ERR_TC_TAG s, __VA_ARGS__)

/*
 * Check the message tlvs in the TC message
 *
 * @param self
 *		TC message information
 * @return true if valid message
 *          false if invalid message
 */
static inline nu_bool_t
tc_read_msg_tlv(tc_param_t* self)
{
    self->has_ansn = false;
    double v;
    FOREACH_TLV_SET(tlv, &self->msg->msg_tlv_set) {
        switch (tlv->type) {
        case MSG_TLV__VALIDITY_TIME:
            if (!isnan(self->validity_time)) {
                ERR_TC0("TC has many VALIDITY_TIME tlvs");
                return false;
            }
            if (nu_msg_has_hop_count(self->msg)) {
                if (!nu_tlv_get_general_time(tlv, self->msg->hop_count, &v)) {
                    ERR_TC0("TC has illegal VALIDITY_TIME tlv");
                    return false;
                }
            } else {
                if (!nu_tlv_get_time(tlv, &v)) {
                    ERR_TC0("TC has illegal VALIDITY_TIME tlv length");
                    return false;
                }
            }
            self->validity_time = v;
            break;
        case MSG_TLV__INTERVAL_TIME:
            if (!isnan(self->interval_time)) {
                ERR_TC0("TC has many INTERVAL_TIME tlvs");
                return false;
            }
            if (!nu_tlv_get_time(tlv, &v)) {
                ERR_TC0("TC has illegal INTERVAL_TIME tlv length");
                return false;
            }
            self->interval_time = v;
            break;
        case MSG_TLV__CONT_SEQNUM:
            if (self->cont_type_ext != NO_CONT_TYPE_EXT) {
                ERR_TC0("TC has many CONT_SEQNUM tlvs");
                return false;
            }
            switch (tlv->type_ext) {
            case CONT_SEQNUM_EXT__COMPLETE:
            case CONT_SEQNUM_EXT__INCOMPLETE:
                break;
            default:
                ERR_TC0("Illegal type_ext in CONT_SEQNUM");
                return false;
            }
            if (!nu_tlv_get_u16(tlv, &self->ansn)) {
                ERR_TC0("TC has illegal CONT_SEQNUM tlv value");
                return false;
            }
            self->cont_type_ext = tlv->type_ext;
            self->has_ansn = true;
            break;
        default:
            nu_info("Ignore unknown message tlv (type=%d) in TC message.",
                    tlv->type);
            break;
        }
    }
    if (isnan(self->validity_time)) {
        ERR_TC0("TC has no VALIDITY_TIME tlv");
        return false;
    }
    if (!self->has_ansn &&
        (nu_ip_set_size(NBR_ADDR_LIST) > 0 || nu_ip_set_size(GW_LIST) > 0)) {
        ERR_TC0("TC has no CONT_SEQNUM tlv");
        return false;
    }
    if (isnan(self->interval_time)) {
        nu_debug("TC uses default interval time");
        self->interval_time = DEFAULT_TC_INTERVAL;
    }
    return true;
}

/* Check network address is my address
 *
 * @param ip	originator address or network address
 * @return true if ip is my address
 */
static inline nu_bool_t
tc_is_my_ip(nu_ip_t ip)
{
    nu_ip_t orig_ip = ip;
    nu_ip_set_default_prefix(&orig_ip);

    if (nu_orig_addr_eq(orig_ip, ORIGINATOR))
        return true;
    if (ibase_o_contain(orig_ip))
        return true;
    FOREACH_I(tuple_i) {
        if (nu_ip_set_contain(&tuple_i->local_ip_list, ip))
            return true;
    }
    if (ibase_ir_contain(ip))
        return true;
    return false;
}

/* Check nbr_addr_type tlv
 *
 * @param self
 * @param ip	associated network address
 * @param tlv	tlv information
 * @return true if valid, false if invalid
 */
static inline nu_bool_t
tc_check_nbr_addr_type(tc_param_t* self, nu_ip_t ip, nu_tlv_t* tlv)
{
    uint8_t v = 0;
    if (!nu_tlv_get_u8(tlv, &v)) {
        ERR_TC0("TC has illegal NBR_ADDR_TYPE tlv length");
        return false;
    }
    switch (v) {
    case NBR_ADDR_TYPE__ROUTABLE_ORIG:
        if (!nu_ip_is_default_prefix(ip)) {
            ERR_TC("%I is ROUTABLE_ORIG, but not default prefix", ip);
            return false;
        }
        if (!nu_ip_is_routable(ip)) {
            ERR_TC("%I is ROUTABLE_ORIG, but not routable", ip);
            return false;
        }
        break;
    case NBR_ADDR_TYPE__ORIGINATOR:
        if (!nu_ip_is_default_prefix(ip)) {
            ERR_TC("%I is ORIG, but not default prefix", ip);
            return false;
        }
        break;
    case NBR_ADDR_TYPE__ROUTABLE:
        if (!nu_ip_is_routable(ip)) {
            ERR_TC("%I is ROUTABLE, but not routable", ip);
            return false;
        }
        break;
    default:
        nu_warn("TC has illegal NBR_ADDR_TYPE's value(%d)", v);
        return false;
    }
    return true;
}

/*
 * Check LINK_METRIC tlv
 *
 * @param self
 * @param ip
 * @param tlv
 * @return true if valid
 */
static inline nu_bool_t
tc_check_link_metric_tlv(tc_param_t* self, nu_ip_t ip, nu_tlv_t* tlv)
{
    int type = nu_link_metric_type(tlv->type_ext);
    if (type != LINK_METRIC_TYPE) {              // ignore
        nu_info("Ignore different link metric type: %d (my type is %d)",
                type, LINK_METRIC_TYPE);
        return true;
    }

    uint8_t v;
    if (!nu_tlv_get_u8(tlv, &v)) {
        ERR_TC("%I is associated with invalid LINK_METRIC tlv", ip);
        return false;
    }
    int dir = nu_link_metric_direction(tlv->type_ext);
    if (dir != LM_TYPE_CODE__N_OUT) {
        ERR_TC("%I is associated with invalid LINK_METRIC direction code", ip);
        return false;
    }

    tc_ip_t* tip = tc_ip_list_search_ip(TC_IP_LIST, ip);
    if (tip == NULL)
        tc_ip_list_insert_head(TC_IP_LIST, tip = tc_ip_create(ip));
    tip->metric = nu_link_metric_unpack(v);
    return true;
}

/* Check network addresses in the TC message
 *
 * @param self
 * @retval true  if valid message
 * @retval false if invalid message
 */
static inline nu_bool_t
tc_read_addr_list(tc_param_t* self)
{
    FOREACH_ATB_LIST(p, self->msg) {
        FOREACH_ATB(atb, nu_atb_list_iter_atb(p)) {
            const nu_ip_t ip = atb->ip;
            const nu_tlv_set_t* tlvs = &atb->tlv_set;
            FOREACH_TLV_SET(tlv, tlvs) {
                if (tc_is_my_ip(ip))
                    continue;
                if (nu_ip_eq_without_prefix(self->orig_addr, ip)) {
                    ERR_TC("%I assoc w/ TLV is the msg-orig-addr", ip);
                    return false;
                }
                switch (tlv->type) {
                case ADDR_TLV__NBR_ADDR_TYPE:
                    if (nu_ip_set_contain(GW_LIST, ip)) {
                        ERR_TC("%I assoc w/ NBR_ADDR_TYPE and GATEWAY", ip);
                        return false;
                    }
                    if (!tc_check_nbr_addr_type(self, ip, tlv))
                        return false;
                    nu_ip_set_add(NBR_ADDR_LIST, ip);
                    break;

                case ADDR_TLV__GATEWAY:
                    if (ibase_al_contain(ip)) {
                        ERR_TC("%I is recorded in AL", ip);
                        return false;
                    }
                    if (nu_ip_set_contain(NBR_ADDR_LIST, ip)) {
                        ERR_TC("%I is assoc w/ GATEWAY and NBR_ADDR_TYPE", ip);
                        return false;
                    }
                    if (nu_ip_set_contain(GW_LIST, ip)) {
                        ERR_TC("%I has many GATEWAYs", ip);
                        return false;
                    }

                    nu_ip_set_add(GW_LIST, ip);
                    break;

                case ADDR_TLV__LINK_METRIC:
                    if (!tc_check_link_metric_tlv(self, ip, tlv))
                        return false;
                    break;

                default:
                    nu_info("Unknown ADDR_TLV(type=%d)", tlv->type);
                }
            }
        }
    }
    return true;
}

/* Populating the advertising remote router set
 *
 * @return true if this message must be discareded
 */
static inline nu_bool_t
tc_update_ibase_ar(tc_param_t* self)
{
    tuple_ar_t* tuple_ar = ibase_ar_search_orig_addr(self->orig_addr);
    if (tuple_ar != NULL) {
        if (nu_seqnum_gt(tuple_ar->seqnum, self->ansn))
            return true;  // No need to process
    } else
        tuple_ar = ibase_ar_add(self->orig_addr, self->ansn);

    tuple_ar->seqnum = self->ansn;
    tuple_ar_set_timeout(tuple_ar, self->validity_time);
    return false;
}

/* Populating the topology set and attached network set
 */
static inline void
tc_update_ibase(tc_param_t* self)
{
    FOREACH_ATB_LIST(p, self->msg) {
        FOREACH_ATB(atb, nu_atb_list_iter_atb(p)) {
            nu_ip_t advertised_ip = atb->ip;
            if (tc_is_my_ip(advertised_ip))
                continue;
            nu_tlv_set_t* tlvs = &atb->tlv_set;
            nu_tlv_t*     nbr_tlv = nu_tlv_set_search_type(tlvs,
                    ADDR_TLV__NBR_ADDR_TYPE);
            if (nbr_tlv) {
                uint8_t     v = 0;
                tuple_tr_t* tuple_tr = NULL;
                tuple_ta_t* tuple_ta = NULL;
                nu_tlv_get_u8(nbr_tlv, &v);
                if (v == NBR_ADDR_TYPE__ORIGINATOR ||
                    v == NBR_ADDR_TYPE__ROUTABLE_ORIG) {
                    tuple_tr = ibase_tr_get(self->orig_addr, advertised_ip);
                    tuple_tr->seqnum = self->ansn;
                    tuple_tr_set_timeout(tuple_tr, self->validity_time);
                }
                if (v == NBR_ADDR_TYPE__ROUTABLE ||
                    v == NBR_ADDR_TYPE__ROUTABLE_ORIG) {
                    tuple_ta = ibase_ta_get(self->orig_addr, advertised_ip);
                    tuple_ta->seqnum = self->ansn;
                    tuple_ta_set_timeout(tuple_ta, self->validity_time);
                }
                if (tuple_tr != NULL || tuple_ta != NULL) {
                    tc_ip_t* tcip = tc_ip_list_search_ip(TC_IP_LIST,
                            advertised_ip);
                    if (tcip != NULL && tuple_tr != NULL)
                        tuple_tr->metric = tcip->metric;
                    if (tcip != NULL && tuple_ta != NULL)
                        tuple_ta->metric = tcip->metric;
                }
            } else {
                nu_tlv_t* gw_tlv = nu_tlv_set_search_type(tlvs,
                        ADDR_TLV__GATEWAY);
                if (gw_tlv) {
#if 0
                    // XXX
                    if (ibase_al_contain(atb->ip))
                        continue;
#endif
                    uint8_t v = 0;
                    nu_tlv_get_u8(gw_tlv, &v);
                    tuple_an_t* tuple_an = ibase_an_get(self->orig_addr,
                            self->ansn, advertised_ip, v);
                    tuple_an_set_timeout(tuple_an, self->validity_time);
                }
            }
        }
    }
}

/*
 */
static inline void
tc_purge_ibase(tc_param_t* self)
{
    for (tuple_tr_t* p = ibase_tr_iter(); !ibase_tr_iter_is_end(p);) {
        if (nu_orig_addr_eq(p->from_orig_addr, self->orig_addr) &&
            nu_seqnum_gt(self->ansn, p->seqnum))
            p = ibase_tr_iter_remove(p);
        else
            p = ibase_tr_iter_next(p);
    }
    for (tuple_ta_t* p = ibase_ta_iter(); !ibase_ta_iter_is_end(p);) {
        if (nu_orig_addr_eq(p->from_orig_addr, self->orig_addr) &&
            nu_seqnum_gt(self->ansn, p->seqnum))
            p = ibase_ta_iter_remove(p);
        else
            p = ibase_ta_iter_next(p);
    }
    for (tuple_an_t* p = ibase_an_iter(); !ibase_an_iter_is_end(p);) {
        if (nu_orig_addr_eq(p->orig_addr, self->orig_addr) &&
            nu_seqnum_gt(self->ansn, p->seqnum))
            p = ibase_an_iter_remove(p);
        else
            p = ibase_an_iter_next(p);
    }
}

/** TC message processor
 *
 * @param msg		TC message
 * @param tuple_i	packet received interface
 * @param src		source address in IP packet
 * @return true if success
 */
PUBLIC nu_bool_t
olsrv2_tc_process(nu_msg_t* msg, tuple_i_t* tuple_i, const nu_ip_t src)
{
    tc_param_t* self = NULL;

#if defined(_WIN32) || defined(_WIN64)   //ScenSim-Port://
    NU_DEBUG_PROCESS("recv_tc:%I", src); //ScenSim-Port://
#else                                    //ScenSim-Port://
    DEBUG_PROCESS("recv_tc:%I", src);
#endif                                   //ScenSim-Port://
    OLSRV2_DO_LOG(recv_tc,
            nu_logger_push_prefix(NU_LOGGER, "RECV_TC:");
            nu_msg_put_log(msg, NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );

    olsrv2_stat_before_tc_process();

    if (!nu_msg_has_orig_addr(msg)) {
        ERR_TC0("TC has no originator");
        goto error;
    }
    if (tc_is_my_ip(msg->orig_addr)) {
        ERR_TC("msg-orig-addr %I is my address", msg->orig_addr);
        goto error;
    }
    if (!nu_msg_has_seqnum(msg)) {
        ERR_TC0("TC has no message-sequence-number");
        goto error;
    }
    if (!nu_msg_has_hop_limit(msg)) {
        ERR_TC0("TC has no hop-limit");
        goto error;
    }

    self = tc_param_create(msg);
    if (!tc_read_addr_list(self))
        goto error;
    if (!tc_read_msg_tlv(self))
        goto error;

    OLSRV2_DO_LOG(recv_tc,
            nu_logger_push_prefix(NU_LOGGER, "RECV_TC:");
            tc_ip_list_put_log(TC_IP_LIST, NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );

    if (tc_update_ibase_ar(self))
        goto finish;
    tc_update_ibase(self);

    if (self->cont_type_ext == CONT_SEQNUM_EXT__COMPLETE)
        tc_purge_ibase(self);

finish:
    if (self)
        tc_param_free(self);
    return true;

error:
    nu_debug("Ignore TC message");
    if (self)
        tc_param_free(self);
    return false;
}

////////////////////////////////////////////////////////////////
//
// Generator
//

/* Add addresses based on the neighbor information set.
 *
 * note: This code assumes that originator address has the default prefix.
 *
 * @param atb
 */
static inline void
tc_add_ibase_n(nu_atb_t* atb)
{
    FOREACH_N(tuple_n) {
        if (!tuple_n->advertised)
            continue;
        nu_bool_t add_orig = false;
        FOREACH_IP_SET(i, &tuple_n->neighbor_ip_list) {
            if (nu_ip_eq(i->ip, tuple_n->orig_addr)) {
                nu_atb_add_ip_tlv(atb, i->ip,
                        addrtlv_create_nbr_addr_type(NBR_ADDR_TYPE__ROUTABLE_ORIG));
                add_orig = true;
            } else {
                nu_atb_add_ip_tlv(atb, i->ip,
                        addrtlv_create_nbr_addr_type(NBR_ADDR_TYPE__ROUTABLE));
            }
            if (USE_LINK_METRIC && tuple_n->_out_metric != UNDEF_METRIC) {
                nu_atb_add_ip_tlv(atb, i->ip,
                        addrtlv_create_link_metric(
                                nu_link_metric_type_ext(LINK_METRIC_TYPE,
                                        LM_TYPE_CODE__N_OUT),
                                tuple_n->_out_metric));
            }
        }
        if (!add_orig) {
            nu_atb_add_ip_tlv(atb, tuple_n->orig_addr,
                    addrtlv_create_nbr_addr_type(NBR_ADDR_TYPE__ORIGINATOR));
        }
    }
}

/* Add network address
 *
 * @param atb
 */
static inline void
tc_add_ibase_al(nu_atb_t* atb)
{
    FOREACH_AL(tuple_al) {
        nu_atb_add_ip_tlv(atb, tuple_al->net_addr,
                addrtlv_create_gateway(tuple_al->dist));
        if (USE_LINK_METRIC && tuple_al->metric != UNDEF_METRIC) {
            nu_atb_add_ip_tlv(atb, tuple_al->net_addr,
                    addrtlv_create_link_metric(
                            nu_link_metric_type_ext(LINK_METRIC_TYPE,
                                    LM_TYPE_CODE__N_OUT),
                            tuple_al->metric));
        }
    }
}

/** TC message generator
 *
 * @return msg
 */
PUBLIC nu_msg_t*
olsrv2_tc_generate(void)
{
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    NU_DEBUG_PROCESS0("gen_tc");       //ScenSim-Port://
#else                                  //ScenSim-Port://
    DEBUG_PROCESS0("gen_tc");
#endif                                 //ScenSim-Port://

    // Incomplete TC generation has not been supported.
    const nu_bool_t complete = true;

    nu_msg_t* msg = nu_msg_create();
    msg->type = MSG_TYPE__TC;
    nu_msg_set_orig_addr(msg, OLSR->originator);
    nu_msg_set_hop_limit(msg, TC_HOP_LIMIT);
    nu_msg_set_hop_count(msg, 0);
    nu_msg_set_seqnum(msg, olsrv2_next_msg_seqnum());

    nu_msg_add_msg_tlv(msg,
            msgtlv_create_validity_time(T_HOLD_TIME));
    nu_msg_add_msg_tlv(msg,
            msgtlv_create_interval_time(OLSR->tc_interval));

    nu_atb_t* atb = nu_msg_add_atb(msg);
    tc_add_ibase_n(atb);
    if (!ibase_al_is_empty()) {
        atb = nu_msg_add_atb(msg);
        tc_add_ibase_al(atb);
    }

    nu_msg_add_msg_tlv(msg,
            msgtlv_create_cont_seqnum(
                    (complete
                     ? CONT_SEQNUM_EXT__COMPLETE
                     : CONT_SEQNUM_EXT__INCOMPLETE),
                    IBASE_N->ansn));

    OLSRV2_DO_LOG(send_tc,
            nu_logger_push_prefix(NU_LOGGER, "SEND_TC:");
            nu_msg_put_log(msg, NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );

    return msg;
}

/** @} */

}//namespace// //ScenSim-Port://
