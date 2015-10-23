#include "config.h"

#include "core/core.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"
#include "olsrv2/ibase_l.h"

#include "core/pvector.h"

#include "packet/msgtlv.h"
#include "packet/addrtlv.h"

#include "debug_util.h"

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
 * Element of hello_ip_list.
 */
typedef struct hello_ip {
    struct hello_ip* next;   ///< next
    struct hello_ip* prev;   ///< prev
    nu_ip_t          ip;     ///< ip
    nu_bool_t        has[4]; ///< link metric flags
    nu_bool_t        fmpr;   ///< true if ip selects src_ip as fmpr.
    nu_bool_t        rmpr;   ///< true if ip selects src_ip as rmpr.
    nu_link_metric_t l_in;   ///< l_in metric
    nu_link_metric_t n_out;  ///< n_out metric
    nu_link_metric_t n_in;   ///< n_in metric
} hello_ip_t;

/*
 */
static inline hello_ip_t*
hello_ip_create(nu_ip_t ip)
{
    hello_ip_t* self = nu_mem_alloc(hello_ip_t);
    self->next = self->prev = NULL;
    self->ip = ip;
    self->has[0] = self->has[1] = self->has[2] = self->has[3] = false;
    self->l_in  = DEFAULT_METRIC;
    self->n_out = DEFAULT_METRIC;
    self->n_in  = DEFAULT_METRIC;
    return self;
}

/*
 */
static inline void
hello_ip_free(hello_ip_t* self)
{
    nu_mem_free(self);
}

////////////////////////////////////////////////////////////////

/**
 * IP address list in HELLO
 */
typedef struct hello_ip_list {
    hello_ip_t* next; ///< first element
    hello_ip_t* prev; ///< last element
    size_t      n;    ///< size
} hello_ip_list_t;

}//namespace// //ScenSim-Port://

#include "olsrv2/hello_ip_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

static inline hello_ip_t*
hello_ip_list_search_ip(hello_ip_list_t* list, const nu_ip_t ip)
{
    FOREACH_HELLO_IP_LIST(i, list) {
        if (nu_ip_eq(i->ip, ip))
            return i;
    }
    return NULL;
}

static void
hello_ip_list_put_log(hello_ip_list_t* list, nu_logger_t* logger)
{
    if (hello_ip_list_is_empty(list)) {
        nu_logger_log(logger, "HelloIpList:[]");
    } else {
        nu_logger_log(logger, "HelloIpList:[");
        nu_logger_push_prefix(logger, OLSRV2_LOG_INDENT);
        FOREACH_HELLO_IP_LIST(p, list) {
            nu_logger_log(logger,
                    "[%I has:(%B,%B,%B,%B) l_in:%g n_out:%g n_in:%g]",
                    p->ip, p->has[0], p->has[1], p->has[2], p->has[3],
                    p->l_in, p->n_out, p->n_in);
        }
        nu_logger_pop_prefix(logger);
        nu_logger_log(logger, "]");
    }
}

////////////////////////////////////////////////////////////////

/** Received HELLO message's parameters
 */
typedef struct hello_param {
    nu_msg_t*       msg;                ///< message
    tuple_i_t*      tuple_i;            ///< tuple_i
    nu_ip_t         sending_ip;         ///< sending ip
    nu_ip_t         orig_addr;          ///< originator
    nu_bool_t       has_msg_seqnum;     ///< whether contains msg seqnum
    uint16_t        msg_seqnum;         ///< message seqnum

    nu_bool_t       has_interval_time;  ///< HELLO contains intaval time TLV
    double          interval_time;      ///< interval time
    double          validity_time;      ///< validity time
    uint8_t         willingness;        ///< willing ness
    uint8_t         metric_type;        ///< default metric type

    nu_ip_set_t     sending_addr_list;  ///< sending address list
    nu_ip_set_t     neighbor_addr_list; ///< neighbor address list
    nu_ip_set_t     removed_addr_list;  ///< removed address list
    nu_ip_set_t     lost_addr_list;     ///< lost address list

    /** Address list of neighbor whose link status is symmetric */
    nu_ip_set_t     link_sym_list;
    /** Address list of neighbor whose link status is heard */
    nu_ip_set_t     link_heard_list;
    /** Address list of neighbor whose link status is lost */
    nu_ip_set_t     link_lost_list;

    /** Address list of other_neighb status is symmetric */
    nu_ip_set_t     other_neighb_sym_list;
    /** Address list of other_neighb status is lost */
    nu_ip_set_t     other_neighb_lost_list;

    /** Address information */
    hello_ip_list_t ip_list;
} hello_param_t;

#define SENDING_ADDR_LIST         (&self->sending_addr_list)      ///< accessor
#define NEIGHBOR_ADDR_LIST        (&self->neighbor_addr_list)     ///< accessor
#define REMOVED_ADDR_LIST         (&self->removed_addr_list)      ///< accessor
#define LOST_ADDR_LIST            (&self->lost_addr_list)         ///< accessor
#define RECEIVING_ADDR_LIST       (&self->tuple_i->local_ip_list) ///< accessor

#define LINK_SYM_LIST             (&self->link_sym_list)          ///< accessor
#define LINK_HEARD_LIST           (&self->link_heard_list)        ///< accessor
#define LINK_LOST_LIST            (&self->link_lost_list)         ///< accessor

#define OTHER_NEIGHB_SYM_LIST     (&self->other_neighb_sym_list)  ///< accessor
#define OTHER_NEIGHB_LOST_LIST    (&self->other_neighb_lost_list) ///< accessor

#define HELLO_IP_LIST             (&self->ip_list)                ///< accessor

/** no willingness */
#define NO_WILL                   (0xff)

/*
 */
static hello_param_t*
hello_param_create(nu_msg_t* msg, tuple_i_t* tuple_i, nu_ip_t sending_ip)
{
    hello_param_t* self = nu_mem_alloc(hello_param_t);
    self->msg = msg;
    self->tuple_i = tuple_i;
    self->sending_ip = sending_ip;
    self->orig_addr  = NU_IP_UNDEF;
    self->has_msg_seqnum = false;
    self->msg_seqnum = 0;
    self->has_interval_time = false;
    self->interval_time = NAN;
    self->validity_time = NAN;
    self->willingness = NO_WILL;
    nu_ip_set_init(SENDING_ADDR_LIST);
    nu_ip_set_init(NEIGHBOR_ADDR_LIST);
    nu_ip_set_init(REMOVED_ADDR_LIST);
    nu_ip_set_init(LOST_ADDR_LIST);
    nu_ip_set_init(LINK_SYM_LIST);
    nu_ip_set_init(LINK_HEARD_LIST);
    nu_ip_set_init(LINK_LOST_LIST);
    nu_ip_set_init(OTHER_NEIGHB_SYM_LIST);
    nu_ip_set_init(OTHER_NEIGHB_LOST_LIST);
    hello_ip_list_init(HELLO_IP_LIST);
    return self;
}

/*
 */
static void
hello_param_free(hello_param_t* self)
{
    nu_ip_set_destroy(SENDING_ADDR_LIST);
    nu_ip_set_destroy(NEIGHBOR_ADDR_LIST);
    nu_ip_set_destroy(REMOVED_ADDR_LIST);
    nu_ip_set_destroy(LOST_ADDR_LIST);
    nu_ip_set_destroy(LINK_SYM_LIST);
    nu_ip_set_destroy(LINK_HEARD_LIST);
    nu_ip_set_destroy(LINK_LOST_LIST);
    nu_ip_set_destroy(OTHER_NEIGHB_SYM_LIST);
    nu_ip_set_destroy(OTHER_NEIGHB_LOST_LIST);
    hello_ip_list_destroy(HELLO_IP_LIST);
    nu_mem_free(self);
}

// Processor ///////////////////////////////////////////////////

/** Prefix of invalid message log */
#define ERR_HELLO_TAG    "DISCARD HELLO: "

/** Outputs the invalid message information */
#define ERR_HELLO0(s)        nu_info(ERR_HELLO_TAG s)
/** Outputs the invalid message information */
#define ERR_HELLO(s, ...)    nu_info(ERR_HELLO_TAG s, __VA_ARGS__)

/*
 * Check LOCAL_IF tlv
 *
 * @param self
 * @param ip
 * @param tlv
 * @return true if valid
 */
static inline nu_bool_t
hello_check_local_if_tlv(hello_param_t* self, nu_ip_t ip, nu_tlv_t* tlv)
{
    if (olsrv2_is_my_ip(ip))
        return false;
    FOREACH_I(tuple_i) {
        FOREACH_IP_SET(i, &tuple_i->local_ip_list) {
            if (nu_ip_is_overlap(ip, i->ip)) {
                ERR_HELLO("%I overlaps I_local_iface_addr_list(%I)",
                        ip, i->ip);
                return true;
            }
        }
    }
    FOREACH_IR(tuple_ir) {
        if (nu_ip_is_overlap(ip, tuple_ir->local_ip)) {
            ERR_HELLO("%I overlaps IR_local_iface_addr(%I)",
                    ip, tuple_ir->local_ip);
            return true;
        }
    }
    uint8_t v = 0;
    if (!nu_tlv_get_u8(tlv, &v)) {
        ERR_HELLO("%I has illegal LOCAL_IF tlv length", ip);
        return false;
    }
    switch (v) {
    case LOCAL_IF__THIS_IF:
        if (!nu_ip_set_contain(SENDING_ADDR_LIST, ip) &&
            nu_ip_set_contain(NEIGHBOR_ADDR_LIST, ip)) {
            ERR_HELLO("%I is associated with many LOCAL_IF values", ip);
            return false;
        }
        nu_ip_set_add(SENDING_ADDR_LIST, ip);
        break;
    case LOCAL_IF__OTHER_IF:
        if (nu_ip_set_contain(SENDING_ADDR_LIST, ip)) {
            ERR_HELLO("%I is associated with many LOCAL_IF values", ip);
            return false;
        }
        break;
    default:
        ERR_HELLO("%I has illegal LOCAL_IF's value, %d", ip, v);
        return false;
    }
    if (nu_ip_set_contain(OTHER_NEIGHB_SYM_LIST, ip) ||
        nu_ip_set_contain(OTHER_NEIGHB_LOST_LIST, ip)) {
        ERR_HELLO("%I is assiciated with LOCAL_IF and OTHER_NEIGHB", ip);
        return false;
    }
    if (nu_ip_set_contain(LINK_SYM_LIST, ip) ||
        nu_ip_set_contain(LINK_HEARD_LIST, ip) ||
        nu_ip_set_contain(LINK_LOST_LIST, ip)) {
        ERR_HELLO("%I is assiciated with LOCLA_IF and LINK_STATUS", ip);
        return false;
    }
    nu_ip_set_add(NEIGHBOR_ADDR_LIST, ip);
    return true;
}

/*
 * Check LINK_STATUS tlv
 *
 * @param self
 * @param ip
 * @param tlv
 * @return true if valid
 */
static inline nu_bool_t
check_link_status_tlv(hello_param_t* self, nu_ip_t ip, nu_tlv_t* tlv)
{
    if (nu_ip_set_contain(SENDING_ADDR_LIST, ip) ||
        nu_ip_set_contain(NEIGHBOR_ADDR_LIST, ip)) {
        ERR_HELLO("%I is assocaited with LOCAL_IF and LINK_STATUS", ip);
        return false;
    }
    uint8_t v = 0;
    if (!nu_tlv_get_u8(tlv, &v)) {
        ERR_HELLO("%I has illegal LINK_STATUS tlv length", ip);
        return false;
    }
    switch (v) {
    case LINK_STATUS__SYMMETRIC:
        if (nu_ip_set_contain(LINK_HEARD_LIST, ip) ||
            nu_ip_set_contain(LINK_LOST_LIST, ip)) {
            ERR_HELLO("%I is assocaited with many LINK_STATUS values", ip);
            return false;
        }
        nu_ip_set_add(LINK_SYM_LIST, ip);
        break;
    case LINK_STATUS__HEARD:
        if (nu_ip_set_contain(LINK_SYM_LIST, ip) ||
            nu_ip_set_contain(LINK_LOST_LIST, ip)) {
            ERR_HELLO("%I is assocaited with many LINK_STATUS values", ip);
            return false;
        }
        nu_ip_set_add(LINK_HEARD_LIST, ip);
        break;
    case LINK_STATUS__LOST:
        if (nu_ip_set_contain(LINK_SYM_LIST, ip) ||
            nu_ip_set_contain(LINK_HEARD_LIST, ip)) {
            ERR_HELLO("%I is assocaited with many LINK_STATUS values", ip);
            return false;
        }
        nu_ip_set_add(LINK_LOST_LIST, ip);
        break;
    default:
        ERR_HELLO("%I has illegal LINK_STATUS value", ip);
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
check_nu_link_metric_tlv(hello_param_t* self, nu_ip_t ip, nu_tlv_t* tlv)
{
    int type = nu_link_metric_type(tlv->type_ext);
    if (type != LINK_METRIC_TYPE) {      // ignore
        nu_info("Ignore different link metric type: %d (my type is %d)",
                type, LINK_METRIC_TYPE);
        return true;
    }

    uint8_t v;
    if (!nu_tlv_get_u8(tlv, &v)) {
        ERR_HELLO("%I is associated with invalid LINK_METRIC tlv", ip);
        return false;
    }
    nu_link_metric_t metric = nu_link_metric_unpack(v);
    int dir = nu_link_metric_direction(tlv->type_ext);

    hello_ip_t* hi = hello_ip_list_search_ip(HELLO_IP_LIST, ip);
    if (hi == NULL)
        hello_ip_list_insert_head(HELLO_IP_LIST, hi = hello_ip_create(ip));
    if (hi->has[dir]) {
        ERR_HELLO("%I is associated with duplicated LINK METRIC TLV", ip);
        return false;
    }
    if (hi->has[0] && (dir == 1 || dir == 2)) {
        ERR_HELLO("%I is associated with duplicated LINK METRIC TLV", ip);
        return false;
    }
    if (dir == 0 && (hi->has[1] || hi->has[2])) {
        ERR_HELLO("%I is associated with duplicated LINK METRIC TLV", ip);
        return false;
    }
    hi->has[dir] = true;
    switch (dir) {
    case 0: // L_in and N_out
        hi->l_in = hi->n_out = metric;
        break;
    case 1: // L_in
        hi->l_in = metric;
        break;
    case 2: // N_out
        hi->n_out = metric;
        break;
    case 3: // N_in
        hi->n_in = metric;
        break;
    default:
        nu_fatal("Internal error.");
        break;
    }
    return true;
}

/*
 * Check OTHER_NEIGHB tlv
 *
 * @param self
 * @param ip
 * @param tlv
 * @return true if valid
 */
static inline nu_bool_t
check_other_neighb_tlv(hello_param_t* self, nu_ip_t ip, nu_tlv_t* tlv)
{
    if (nu_ip_set_contain(NEIGHBOR_ADDR_LIST, ip)) {
        ERR_HELLO("%I is associated with OTHER_NEIGHB and LOCAL_IF", ip);
        return false;
    }
    if (nu_ip_set_contain(LINK_SYM_LIST, ip) ||
        nu_ip_set_contain(LINK_HEARD_LIST, ip) ||
        nu_ip_set_contain(LINK_LOST_LIST, ip)) {
        ERR_HELLO("%I is associated with OTHER_NEIGHB and LINK_STATUS", ip);
        return false;
    }
    uint8_t v = 0;
    if (!nu_tlv_get_u8(tlv, &v)) {
        ERR_HELLO("%I has illegal OTHER_NEIGHB tlv length", ip);
        return false;
    }
    switch (v) {
    case OTHER_NEIGHB__SYMMETRIC:
        if (nu_ip_set_contain(OTHER_NEIGHB_SYM_LIST, ip)) {
            ERR_HELLO("%I is assocaited with many OTHER_NEIGHB values", ip);
            return false;
        }
        nu_ip_set_add(OTHER_NEIGHB_SYM_LIST, ip);
        break;
    case OTHER_NEIGHB__LOST:
        if (nu_ip_set_contain(OTHER_NEIGHB_SYM_LIST, ip)) {
            ERR_HELLO("%I is assocaited with many OTHER_NEIGHB values", ip);
            return false;
        }
        nu_ip_set_add(OTHER_NEIGHB_LOST_LIST, ip);
        break;
    default:
        ERR_HELLO("%I has illegal OTHER_NEIGHB tlv value", ip);
        return false;
    }
    return true;
}

/*
 * Make SENDING_ADDR_LIST and NEIGHBOR_ADDR_LIST.
 * and check whether the HELLO message is valid or not.
 *
 * @param self
 *		HELLO message's parameters
 * @return true if success,
 *		false if this HELLO message must be discarded.
 */
static inline nu_bool_t
read_neighbor_addr_list(hello_param_t* self)
{
    FOREACH_ATB_LIST(p, self->msg) {
        FOREACH_ATB(atb, nu_atb_list_iter_atb(p)) {
            nu_ip_t ip = atb->ip;
            nu_tlv_set_t* tlvs = &atb->tlv_set;
            FOREACH_TLV_SET(tlv, tlvs) {
                // TLV that type_ext != 0
                switch (tlv->type) {
                case ADDR_TLV__LINK_METRIC:
                    if (!check_nu_link_metric_tlv(self, ip, tlv))
                        return false;
                    continue;
                default:
                    ; /* IGNORE */
                }

                if (tlv->type_ext != 0) {
                    nu_info("HELLO: Ignore TLV with type_ext != 0");
                    continue;
                }
                switch (tlv->type) {
                case ADDR_TLV__LOCAL_IF:
                    if (!hello_check_local_if_tlv(self, ip, tlv))
                        return false;
                    break;
                case ADDR_TLV__LINK_STATUS:
                    if (!check_link_status_tlv(self, ip, tlv))
                        return false;
                    break;
                case ADDR_TLV__OTHER_NEIGHB:
                    if (!check_other_neighb_tlv(self, ip, tlv))
                        return false;
                    break;
                default:
                    ; /* IGNORE */
                }
            }
        }
    }
    if (nu_ip_set_is_empty(SENDING_ADDR_LIST)) {
        if (olsrv2_is_my_ip(self->sending_ip))
            return false;
        nu_ip_set_add(SENDING_ADDR_LIST, self->sending_ip);
        nu_ip_set_add(NEIGHBOR_ADDR_LIST, self->sending_ip);
    }
    return true;
}

/*
 * Read message tlv in HELLO message.
 *
 * @param self
 *		HELLO message's	 parameters
 * @return true if success
 *		false if invalid message
 */
static inline nu_bool_t
hello_read_msg_tlv(hello_param_t* self)
{
    double  v = 0;
    uint8_t w = 0;
    uint8_t m = 0xff;
    FOREACH_TLV_SET(tlv, &self->msg->msg_tlv_set) {
        if (tlv->type_ext != 0) {
            nu_info("HELLO: Ignore TLV with type_ext != 0");
            continue;
        }
        switch (tlv->type) {
        case MSG_TLV__VALIDITY_TIME:
            if (!nu_tlv_get_time(tlv, &v)) {
                ERR_HELLO0("HELLO has illegal VALIDITY_TIME tlv length");
                return false;
            }
            if (!isnan(self->validity_time) && v != self->validity_time) {
                ERR_HELLO0("HELLO has many VALIDITY_TIME tlvs");
                return false;
            }
            self->validity_time = v;
            break;
        case MSG_TLV__INTERVAL_TIME:
            if (!nu_tlv_get_time(tlv, &v)) {
                ERR_HELLO0("HELLO has illegal INTERVAL_TIME tlv length");
                return false;
            }
            if (!isnan(self->interval_time) && v != self->interval_time) {
                ERR_HELLO0("HELLO has many INTERVAL_TIME tlvs");
                return false;
            }
            self->interval_time = v;
            self->has_interval_time = true;
            break;
        case MSG_TLV__MPR_WILLING:
            if (!nu_tlv_get_u8(tlv, &w)) {
                ERR_HELLO0("HELLO has illegal WILLINGNESS tlv length");
                return false;
            }
            if (self->willingness != NO_WILL && w != self->willingness) {
                ERR_HELLO0("HELLO has many WILLINGNESS tlvs");
                return false;
            }
            self->willingness = w;
            break;
        case MSG_TLV__LINK_METRIC_EXT:
            if (!nu_tlv_get_u8(tlv, &m)) {
                ERR_HELLO0("HELLO has illegal LINK_METRIC_EXT tlv length");
                return false;
            }
            self->metric_type = m;
            break;
        }
    }
    if (isnan(self->validity_time)) {
        ERR_HELLO0("HELLO has no VALIDITY_TIME tlv");
        return false;
    }
    if (isnan(self->interval_time)) {
        nu_info("HELLO uses global interval time");
        self->interval_time = GLOBAL_HELLO_INTERVAL;
    }
    if (self->willingness == NO_WILL) {
        nu_info("HELLO uses default WILLINGNESS");
        self->willingness = WILLINGNESS__DEFAULT;
    }
    return true;
}

/* Compare the matching tuple_n's neighbor_ip_list
 * and neighbor_addr_list for updating removed addr list
 * and lost addr list.
 */
static inline void
update_removed_and_lost_addr_list(hello_param_t* self, tuple_n_t* tuple_n)
{
    nu_ip_set_t* tuple_ip_set = &tuple_n->neighbor_ip_list;
    for (nu_ip_set_iter_t n = nu_ip_set_iter(tuple_ip_set);
         !nu_ip_set_iter_is_end(n, tuple_ip_set);) {
        if (nu_ip_set_contain(NEIGHBOR_ADDR_LIST, n->ip))
            n = nu_ip_set_iter_next(n, tuple_ip_set);
        else {
            nu_ip_set_add(REMOVED_ADDR_LIST, n->ip);
            if (tuple_n->symmetric)
                nu_ip_set_add(LOST_ADDR_LIST, n->ip);
            n = nu_ip_set_iter_remove(n, tuple_ip_set);
        }
    }
}

/*
 * Updating the Neighbor Set
 */
static inline tuple_n_t*
update_ibase_n(hello_param_t* self)
{
    nu_pvector_t tuples;
    nu_pvector_init(&tuples, 4, 4);
#if defined(_WIN32) || defined(_WIN64)                                 //ScenSim-Port://
    FOREACH_N(_tuple_n) {                                              //ScenSim-Port://
        if (nu_ip_list_is_overlap_ip_list(&_tuple_n->neighbor_ip_list, //ScenSim-Port://
                    NEIGHBOR_ADDR_LIST) ||                             //ScenSim-Port://
            nu_orig_addr_eq(_tuple_n->orig_addr, self->orig_addr))     //ScenSim-Port://
            nu_pvector_add(&tuples, _tuple_n);                         //ScenSim-Port://
    }                                                                  //ScenSim-Port://
#else                                                                  //ScenSim-Port://
    FOREACH_N(tuple_n) {
        if (nu_ip_list_is_overlap_ip_list(&tuple_n->neighbor_ip_list,
                    NEIGHBOR_ADDR_LIST) ||
            nu_orig_addr_eq(tuple_n->orig_addr, self->orig_addr))
            nu_pvector_add(&tuples, tuple_n);
    }
#endif                                                                 //ScenSim-Port://

    tuple_n_t* tuple_n = NULL;
    if (nu_pvector_size(&tuples) == 0) {
        tuple_n = ibase_n_add(NEIGHBOR_ADDR_LIST, self->orig_addr);
    } else if (nu_pvector_size(&tuples) == 1) {
        tuple_n = (tuple_n_t*)nu_pvector_get_at(&tuples, 0);
        tuple_n_set_orig_addr(tuple_n, self->orig_addr);
        update_removed_and_lost_addr_list(self, tuple_n);
        FOREACH_IP_SET(n, NEIGHBOR_ADDR_LIST) {
            tuple_n_add_neighbor_ip_list(tuple_n, n->ip);
        }
    } else {
        for (size_t i = 0; i < nu_pvector_size(&tuples); ++i) {
            tuple_n = (tuple_n_t*)nu_pvector_get_at(&tuples, i);
            update_removed_and_lost_addr_list(self, tuple_n);
            ibase_n_iter_remove(tuple_n);
        }
        tuple_n = ibase_n_add(NEIGHBOR_ADDR_LIST, self->orig_addr);
    }
    nu_pvector_destroy(&tuples);

    // Update willingness [olsrv2]
    tuple_n_set_willingness(tuple_n, self->willingness);
    return tuple_n;
}

/* Updating Lost Neighbor Set
 */
static inline void
update_ibase_nl(hello_param_t* self)
{
    FOREACH_IP_SET(i, LOST_ADDR_LIST) {
        if (!ibase_nl_contain(i->ip))
            ibase_nl_add(i->ip);
    }
}

static void
etx_hello_process(hello_param_t* self, tuple_l_t* tuple_l, nu_tlv_set_t* tlvs)
{
    tuple_l->metric_d_etx = UNDEF_ETX;
    nu_tlv_t* etx_tlv = nu_tlv_set_search_type(tlvs, ADDR_TLV__R_ETX);
    if (etx_tlv != NULL && etx_tlv->type_ext == 0) {
        uint8_t m;
        if (nu_tlv_get_u8(etx_tlv, &m)) {
            tuple_l->metric_d_etx = etx_unpack(m);
        } else {
            tuple_l->metric_d_etx = UNDEF_ETX;
            nu_info("HELLO: Ignore R_ETX with invalid data length");
        }
    }
    if (self->has_interval_time)
        tuple_l->metric_hello_interval = self->interval_time;
    if (tuple_l->metric_hello_interval != UNDEF_INTERVAL) {
        if (tuple_l->metric_hello_timeout_event != NULL) {
            nu_event_list_cancel(METRIC_EVENT_LIST,
                    tuple_l->metric_hello_timeout_event);
        }
        tuple_l->metric_hello_timeout_event =
            nu_event_list_add(METRIC_EVENT_LIST,
                    etx_hello_timeout_event, tuple_l,
                    tuple_l->metric_hello_interval * ETX_HELLO_TIMEOUT_FACTOR,
                    NU_EVENT_PRIO__DEFAULT,
                    "etx_hello_timeout_event");
    }
    tuple_l->metric_lost_hellos = 0;
}

/* Updating Link Set
 */
static inline tuple_l_t*
update_ibase_l(hello_param_t* self, tuple_n_t* tuple_n)
{
    ibase_l_t* ibase_l = &self->tuple_i->ibase_l;

    // 1. & 2. & 3.
    nu_pvector_t tuples;
    nu_pvector_init(&tuples, 4, 4);
    for (tuple_l_t* tuple_l = ibase_l_iter(ibase_l);
         !ibase_l_iter_is_end(tuple_l, ibase_l);) {
        tuple_l_remove_ip_set(tuple_l, REMOVED_ADDR_LIST);
        if (nu_ip_set_is_empty(&tuple_l->neighbor_ip_list)) {
            tuple_l = ibase_l_iter_remove(tuple_l, ibase_l);
            // See olsrv2_post_process() for Section 13.2 Link Tuple
            // Not Symmetric.
        } else {
            if (nu_ip_set_contain_at_least_one(
                        &tuple_l->neighbor_ip_list,
                        SENDING_ADDR_LIST))
                nu_pvector_add(&tuples, tuple_l);
            tuple_l = ibase_l_iter_next(tuple_l, ibase_l);
        }
    }

    // 4.
    if (nu_pvector_size(&tuples) > 1) {
        for (size_t i = 0; i < nu_pvector_size(&tuples); ++i) {
            ibase_l_iter_remove(
                    (tuple_l_t*)nu_pvector_get_at(&tuples, i), ibase_l);
            // See olsrv2_post_process() for Section 13.2 Link Tuple
            // Not Symmetric.
        }
        nu_pvector_remove_all(&tuples);
    }

    // 5.
    tuple_l_t* tuple_l = NULL;
    if (nu_pvector_size(&tuples) == 0) {
        tuple_l = ibase_l_add(ibase_l, self->tuple_i, tuple_n);
        tuple_l->tuple_n = tuple_n;
        tuple_l_set_timeout(tuple_l, self->validity_time);
        if (LQ_TYPE == LQ__HELLO) {
            double lq_interval = self->interval_time * LOSS_DETECT_SCALE;
            tuple_l->lq_event = nu_event_list_add_periodic(LQ_EVENT_LIST,
                    tuple_l_quality_down, tuple_l,
                    NU_EVENT_PRIO__DEFAULT,
                    lq_interval, lq_interval, 0, "tuple_l_quality_down");
        }
    } else if (nu_pvector_size(&tuples) == 1) {
        tuple_l = (tuple_l_t*)nu_pvector_get_at(&tuples, 0);
        tuple_l->tuple_n = tuple_n;
        if (LQ_TYPE == LQ__HELLO) {
            tuple_l_quality_up(tuple_l);
            double lq_interval = self->interval_time * LOSS_DETECT_SCALE;
            nu_event_list_cancel(LQ_EVENT_LIST, tuple_l->lq_event);
            tuple_l->lq_event = nu_event_list_add_periodic(LQ_EVENT_LIST,
                    tuple_l_quality_down, tuple_l,
                    NU_EVENT_PRIO__DEFAULT,
                    lq_interval, lq_interval, 0, "tuple_l_quality_down");
        }
    } else {
        nu_fatal("Internal error");
    }

    nu_pvector_destroy(&tuples);

    // 6.
    FOREACH_IP_SET(i, RECEIVING_ADDR_LIST) {
        nu_atb_iter_t atb = nu_msg_search_ip(self->msg, i->ip);
        if (atb == NULL)
            continue;
        nu_tlv_set_t* tlvs = nu_atb_iter_tlv_set(atb);
        nu_tlv_t*     tlv  = nu_tlv_set_search_type(tlvs,
                ADDR_TLV__LINK_STATUS);
        if (tlv == NULL) {
            nu_warn("Receiving address %I has no LINK_STATUS TLV.", i->ip);
            continue;
        }
        if (tlv->type_ext != 0) {
            nu_info("HELLO: Ignore LINK_STATUS TLV with type_ext != 0");
            continue;
        }
        uint8_t v = 0;
        nu_tlv_get_u8(tlv, &v);
        // v has been already checked at read_neighbor_addr_list()
        switch (v) {
        case LINK_STATUS__SYMMETRIC:
        case LINK_STATUS__HEARD:
            nu_time_set_timeout(&tuple_l->SYM_time, self->validity_time);
            {
                // LINK METRIC
                hello_ip_t* hi = hello_ip_list_search_ip(HELLO_IP_LIST, i->ip);
                if (hi == NULL)
                    tuple_l_set_out_metric(tuple_l, DEFAULT_METRIC);
                else
                    tuple_l_set_out_metric(tuple_l, hi->l_in);
            }
            break;
        case LINK_STATUS__LOST:
            tuple_l_set_out_metric(tuple_l, UNDEF_METRIC);
        /* GO THROUGH */
        default:
            if (!nu_time_is_expired(tuple_l->SYM_time)) {
                nu_time_set_timeout(&tuple_l->SYM_time, -1);
                if (tuple_l->status == LINK_STATUS__HEARD)
                    tuple_l_set_timeout(tuple_l, L_HOLD_TIME);
            }
            break;
        }
        if (LINK_METRIC_TYPE == LINK_METRIC_TYPE__ETX)
            etx_hello_process(self, tuple_l, tlvs);
    }

    nu_ip_set_copy(&tuple_l->neighbor_ip_list, SENDING_ADDR_LIST);

    tuple_l->HEARD_time = NU_NOW;
    nu_time_add_f(&tuple_l->HEARD_time, self->validity_time);
    if (nu_time_cmp(tuple_l->HEARD_time, tuple_l->SYM_time) < 0)
        tuple_l->HEARD_time = tuple_l->SYM_time;

    if (tuple_l->status == LINK_STATUS__PENDING) {
        tuple_l_set_time(tuple_l,
                nu_time_max(tuple_l->time, tuple_l->HEARD_time));
    } else {
        nu_time_t t = tuple_l->HEARD_time;
        nu_time_add_f(&t, L_HOLD_TIME);
        if (nu_time_cmp(tuple_l->time, t) < 0)
            tuple_l_set_time(tuple_l, t);
    }
    tuple_l_update_status(tuple_l);

    return tuple_l;
}

/* Updating the 2-Hop Set
 */
static inline void
update_ibase_n2(hello_param_t* self, tuple_l_t* tuple_l)
{
    ibase_n2_t* ibase_n2 = &tuple_l->ibase_n2;

    // 1.
    // no need to sync n2_neighbor_ip_list.
    // @see ibase_l.h and ibase_n2.h.

    // 2.
    if (tuple_l->status != LINK_STATUS__SYMMETRIC)
        return;

    FOREACH_ATB_LIST(p, self->msg) {
        FOREACH_ATB(atb, nu_atb_list_iter_atb(p)) {
            nu_ip_t hop2 = atb->ip;
            if (olsrv2_is_my_ip(hop2) ||
                nu_ip_set_contain(NEIGHBOR_ADDR_LIST, hop2))
                continue;

            nu_tlv_set_t* tlvs = &atb->tlv_set;
            nu_bool_t     is_sym = false;
            FOREACH_TLV_SET(tlv, tlvs) {
                if (tlv->type_ext != 0) {
                    nu_info("HELLO: Ignore TLV with type_ext != 0");
                    continue;
                }
                if (tlv->type == ADDR_TLV__LINK_STATUS) {
                    uint8_t v = 0;
                    nu_tlv_get_u8(tlv, &v);
                    if (v == LINK_STATUS__SYMMETRIC)
                        is_sym = true;
                    break;
                }
                if (tlv->type == ADDR_TLV__OTHER_NEIGHB) {
                    uint8_t v = 0;
                    nu_tlv_get_u8(tlv, &v);
                    if (v == OTHER_NEIGHB__SYMMETRIC)
                        is_sym = true;
                    break;
                }
            }

            tuple_n2_t* tuple_n2 = ibase_n2_search(ibase_n2, hop2);
            if (is_sym) {
                if (tuple_n2 == NULL)
                    tuple_n2 = ibase_n2_add(hop2, tuple_l);
                tuple_n2_set_timeout(tuple_n2, self->validity_time);

                // Link metric
                hello_ip_t* hi = hello_ip_list_search_ip(HELLO_IP_LIST, hop2);
                if (hi == NULL) {
                    tuple_n2_set_in_metric(tuple_n2, DEFAULT_METRIC);
                    tuple_n2_set_out_metric(tuple_n2, DEFAULT_METRIC);
                } else {
                    tuple_n2_set_in_metric(tuple_n2, hi->n_in);
                    tuple_n2_set_out_metric(tuple_n2, hi->n_out);
                }
            } else {
                if (tuple_n2 != NULL)
                    (void)ibase_n2_iter_remove(tuple_n2, ibase_n2);
            }
        }
    }
}

/* Update mpr_selector.
 */
static void
update_mpr_selector(hello_param_t* self, tuple_n_t* tuple_n, tuple_l_t* tuple_l)
{
    nu_bool_t rmprs = false;
    nu_bool_t fmprs = false;
    FOREACH_ATB_LIST(p, self->msg) {
        FOREACH_ATB(atb, nu_atb_list_iter_atb(p)) {
            if (!olsrv2_is_my_ip(atb->ip))
                continue;
            FOREACH_TLV_SET(tlv, &atb->tlv_set) {
                if (tlv->type == ADDR_TLV__MPR)
                    fmprs = true;
                else if (tlv->type == ADDR_TLV__FMPR)
                    fmprs = true;
                else if (tlv->type == ADDR_TLV__RMPR)
                    rmprs = true;
            }
        }
    }
    if (USE_LINK_METRIC) {
        tuple_l_set_mpr_selector(tuple_l, fmprs);
        tuple_n_set_mpr_selector(tuple_n, rmprs);
    } else {
        tuple_l_set_mpr_selector(tuple_l, fmprs);
        tuple_n_set_mpr_selector(tuple_n, fmprs);
    }
}

/** HELLO message processor.
 *
 * @param msg       HELLO message
 * @param tuple_i   packet received interface
 * @param src_ip    source address in the IP packet
 * @param metric    link metric
 * @return true if success
 */
PUBLIC tuple_l_t*
olsrv2_hello_process(nu_msg_t* msg, tuple_i_t* tuple_i,
        const nu_ip_t src_ip, const nu_link_metric_t metric)
{
    hello_param_t* self = NULL;
    tuple_l_t*     tuple_l = NULL;
    tuple_n_t*     tuple_n = NULL;
    nu_ip_t src = src_ip;

#if defined(_WIN32) || defined(_WIN64)         //ScenSim-Port://
    NU_DEBUG_PROCESS("recv_hello:%I", src_ip); //ScenSim-Port://
#else                                          //ScenSim-Port://
    DEBUG_PROCESS("recv_hello:%I", src_ip);
#endif                                         //ScenSim-Port://
    OLSRV2_DO_LOG(recv_hello,
            nu_logger_push_prefix(NU_LOGGER, "RECV_HELLO:");
            nu_msg_put_log(msg, NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );

    olsrv2_stat_before_hello_process();

    if (msg->addr_len != nu_ip_len(OLSR->originator)) {
        ERR_HELLO0("HELLO has different addr_len.");
        goto error;
    }
    if (nu_msg_has_hop_limit(msg) && msg->hop_limit != 1) {
        ERR_HELLO0("HELLO has hop-limit but it is not 1");
        goto error;
    }
    if (nu_msg_has_hop_count(msg) && msg->hop_count != 0) {
        ERR_HELLO0("HELLO has hop-count but it is not 0");
        goto error;
    }
    nu_ip_set_default_prefix(&src);
    self = hello_param_create(msg, tuple_i, src);
    self->has_msg_seqnum = nu_msg_has_seqnum(msg);
    if (self->has_msg_seqnum)
        self->msg_seqnum = msg->seqnum;

    // Verify message
    if (!hello_read_msg_tlv(self))
        goto error;
    if (!read_neighbor_addr_list(self))
        goto error;

    // Get originator address
    if (nu_msg_has_orig_addr(msg)) {
        nu_orig_addr_copy(&self->orig_addr, msg->orig_addr);
    } else if (nu_ip_set_size(NEIGHBOR_ADDR_LIST) == 1) {
        nu_orig_addr_copy(&self->orig_addr,
                nu_ip_set_head(NEIGHBOR_ADDR_LIST));
    } else {
        nu_orig_addr_copy(&self->orig_addr, src);
    }
    if (olsrv2_is_my_ip(self->orig_addr))
        return NULL;

    // Update link metric information
    FOREACH_HELLO_IP_LIST(i, HELLO_IP_LIST) {
        if (i->has[0] && !i->has[1] && !i->has[2] && !i->has[3])      // 0
            i->n_out = i->l_in;
        else if (!i->has[0] && i->has[1] && !i->has[2] && !i->has[3]) // 1
            i->n_out = DEFAULT_METRIC;
        else if (!i->has[0] && !i->has[1] && i->has[2] && !i->has[3]) // 2
            i->l_in = i->n_in = DEFAULT_METRIC;
        else if (!i->has[0] && i->has[1] && i->has[2] && !i->has[3])  // 1, 2
            i->l_in = i->n_in;
        else if (i->has[0] && !i->has[1] && !i->has[2] && i->has[3])  // 0, 3
            i->l_in = i->n_out;
        else if (!i->has[0] && i->has[1] && !i->has[2] && i->has[3])  // 1, 3
            i->n_out = DEFAULT_METRIC;
        else if (!i->has[0] && !i->has[1] && i->has[2] && i->has[3])  // 2, 3
            i->l_in = DEFAULT_METRIC;
        else if (!i->has[0] && i->has[1] && i->has[2] && i->has[3])   // 1, 2, 3
            ;
        else
            nu_fatal("Internal error Illegal conbination of link metric coding.");
    }
    OLSRV2_DO_LOG(recv_hello,
            nu_logger_push_prefix(NU_LOGGER, "RECV_HELLO:");
            hello_ip_list_put_log(HELLO_IP_LIST, NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );

    tuple_n = update_ibase_n(self);
    update_ibase_nl(self);
    tuple_l = update_ibase_l(self, tuple_n);
    if (LINK_METRIC_TYPE == LINK_METRIC_TYPE__STATIC ||
        LINK_METRIC_TYPE == LINK_METRIC_TYPE__TEST)
        tuple_l_set_in_metric(tuple_l, metric);
    update_ibase_n2(self, tuple_l);
    update_mpr_selector(self, tuple_n, tuple_l);

    olsrv2_stat_after_hello_process(tuple_i, src_ip, self->msg_seqnum);

    hello_param_free(self);
    return tuple_l;

error:
    if (self)
        hello_param_free(self);
    return NULL;
}

// Generator ///////////////////////////////////////////////////

/*
 */
static inline void
add_msg_tlv(nu_msg_t* msg, tuple_i_t* tuple_i)
{
    nu_msg_add_msg_tlv(msg,
            msgtlv_create_validity_time(H_HOLD_TIME));
    nu_msg_add_msg_tlv(msg,
            msgtlv_create_interval_time(tuple_i->hello_interval));
    nu_msg_add_msg_tlv(msg,
            msgtlv_create_mpr_willing(OLSR->willingness));
}

/*
 */
static inline void
add_local_interfaces(nu_atb_t* atb, tuple_i_t* target_tuple_i)
{
    FOREACH_I(tuple_i) {
        uint8_t value = 0;
        if (tuple_i == target_tuple_i)
            value = LOCAL_IF__THIS_IF;
        else
            value = LOCAL_IF__OTHER_IF;
        FOREACH_IP_SET(i, &tuple_i->local_ip_list) {
            nu_atb_add_ip_tlv(atb, i->ip,
                    addrtlv_create_local_if(value));
        }
    }
}

static inline void
hello_link_metric_add_ibase_l(nu_atb_t* atb, nu_ip_t ip,
        tuple_l_t* tuple_l, tuple_n_t* tuple_n)
{
    if (tuple_l->status == LINK_STATUS__SYMMETRIC ||
        tuple_l->status == LINK_STATUS__HEARD) {
        if (tuple_l->in_metric != UNDEF_METRIC) {
            nu_atb_add_ip_tlv(atb, ip,
                    addrtlv_create_link_metric(
                            nu_link_metric_type_ext(LINK_METRIC_TYPE,
                                    LM_TYPE_CODE__L_IN),
                            tuple_l->in_metric));
        }
        if (LINK_METRIC_TYPE == LINK_METRIC_TYPE__ETX &&
            tuple_l->metric_r_etx != UNDEF_ETX) {
            nu_atb_add_ip_tlv(atb, ip,
                    addrtlv_create_r_etx(tuple_l->metric_r_etx));
        }
        if (tuple_l->status == LINK_STATUS__SYMMETRIC) {
            if (tuple_n->flooding_mpr) {
                nu_atb_add_ip_tlv(atb, ip,
                        addrtlv_create_flooding_mpr());
            }
            if (tuple_n->routing_mpr) {
                nu_atb_add_ip_tlv(atb, ip,
                        addrtlv_create_routing_mpr());
            }
        }
    }
}

/*
 */
static inline void
hello_add_ibase_l(nu_atb_t* atb, tuple_i_t* tuple_i)
{
    FOREACH_L(tuple_l, &tuple_i->ibase_l) {
        if (tuple_l->status == LINK_STATUS__PENDING)
            continue;
        tuple_n_t* tuple_n = ibase_n_search_tuple_l(tuple_l);
        assert(tuple_n != NULL);
        FOREACH_IP_LIST(i, &tuple_l->neighbor_ip_list) {
            nu_atb_add_ip_tlv(atb, i->ip,
                    addrtlv_create_link_status(tuple_l->status));
            if (USE_LINK_METRIC) {
                hello_link_metric_add_ibase_l(atb, i->ip, tuple_l, tuple_n);
            } else {
                if (tuple_n->flooding_mpr &&
                    tuple_l->status == LINK_STATUS__SYMMETRIC)
                    nu_atb_add_ip_tlv(atb, i->ip, addrtlv_create_mpr());
            }
        }
    }
}

static inline void
hello_link_metric_add_ibase_n(nu_atb_t* atb, nu_ip_t ip, tuple_n_t* tuple_n)
{
    nu_atb_add_ip_tlv(atb, ip,
            addrtlv_create_link_metric(
                    nu_link_metric_type_ext(LINK_METRIC_TYPE,
                            LM_TYPE_CODE__N_IN),
                    tuple_n->_in_metric));
    nu_atb_add_ip_tlv(atb, ip,
            addrtlv_create_link_metric(
                    nu_link_metric_type_ext(LINK_METRIC_TYPE,
                            LM_TYPE_CODE__N_OUT),
                    tuple_n->_out_metric));
    if (tuple_n->routing_mpr &&
        nu_atb_search_ip_tlv_type(atb, ip, ADDR_TLV__RMPR) == NULL)
        nu_atb_add_ip_tlv(atb, ip, addrtlv_create_routing_mpr());
}

/*
 */
static inline void
hello_add_ibase_n(nu_atb_t* atb)
{
    FOREACH_N(tuple_n) {
        if (!tuple_n->symmetric)
            continue;
        FOREACH_IP_SET(i, &tuple_n->neighbor_ip_list) {
            if (!nu_atb_contain_ip(atb, i->ip)) {
                nu_atb_add_ip_tlv(atb, i->ip,
                        addrtlv_create_other_neighb(OTHER_NEIGHB__SYMMETRIC));
            }
            if (USE_LINK_METRIC)
                hello_link_metric_add_ibase_n(atb, i->ip, tuple_n);
            if (tuple_n->flooding_mpr &&
                nu_atb_search_ip_tlv_type(atb, i->ip, ADDR_TLV__MPR) == NULL)
                nu_atb_add_ip_tlv(atb, i->ip, addrtlv_create_mpr());
        }
    }
}

/*
 */
static inline void
add_ibase_nl(nu_atb_t* atb)
{
    FOREACH_NL(tuple_nl) {
        if (nu_atb_contain_ip(atb, tuple_nl->neighbor_ip))
            continue;
        nu_atb_add_ip_tlv(atb, tuple_nl->neighbor_ip,
                addrtlv_create_other_neighb(OTHER_NEIGHB__LOST));
    }
}

/*
 */
static inline void
helllo_add_ibase_al(nu_atb_t* atb)
{
    nu_fatal("NOT SUPPOERT");
}

/** HELLO message generator
 *
 * @param tuple_i
 *		target interface
 * @return msg
 */
PUBLIC nu_msg_t*
olsrv2_hello_generate(tuple_i_t* tuple_i)
{
#if defined(_WIN32) || defined(_WIN64)                                             //ScenSim-Port://
    NU_DEBUG_PROCESS("gen_hello:%s:%I", tuple_i->name, tuple_i_local_ip(tuple_i)); //ScenSim-Port://
#else                                                                              //ScenSim-Port://
    DEBUG_PROCESS("gen_hello:%s:%I", tuple_i->name, tuple_i_local_ip(tuple_i));
#endif                                                                             //ScenSim-Port://

    tuple_i_t* target_tuple_i = tuple_i;

    nu_msg_t* msg = nu_msg_create();
    msg->type = MSG_TYPE__HELLO;
    nu_msg_set_orig_addr(msg, OLSR->originator);
    nu_msg_set_hop_limit(msg, 1);
    nu_msg_set_hop_count(msg, 0);
    nu_msg_set_seqnum(msg, tuple_i_get_next_hello_msg_seqnum(tuple_i));

    add_msg_tlv(msg, target_tuple_i);

    nu_atb_t* local_atb = nu_msg_add_atb(msg);
    add_local_interfaces(local_atb, target_tuple_i);

    nu_atb_t* atb = nu_msg_add_atb(msg);
    hello_add_ibase_l(atb, target_tuple_i);
    hello_add_ibase_n(atb);
    add_ibase_nl(atb);

    OLSRV2_DO_LOG(send_hello,
            nu_logger_push_prefix(NU_LOGGER, "SEND_HELLO:");
            nu_msg_put_log(msg, NU_LOGGER);
            nu_logger_pop_prefix(NU_LOGGER);
            );

    return msg;
}

/** @} */

}//namespace// //ScenSim-Port://
