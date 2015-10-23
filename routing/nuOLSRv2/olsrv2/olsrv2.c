//
// OLSRv2 module
//
#include "config.h"

#include <stddef.h>

#include "packet/packet.h"
#include "olsrv2/olsrv2.h"
#include "olsrv2/pktq.h"
#include "olsrv2/debug_util.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup olsrv2
 */

#ifndef nuOLSRv2_ALL_STATIC
olsrv2_t* current_olsrv2 = NULL;
#endif

static void hello_gen_event(nu_event_t*);
static void tc_gen_event(nu_event_t*);

#ifndef NU_NDEBUG
static void ibase_put_log(nu_logger_t* logger);
#endif

/**
 * Log flags
 */
/* *INDENT-OFF* */
static nu_log_flag_t olsrv2_log_flags[] =
{
    { "process",	"Oi", offsetof(olsrv2_t, log_process),
      "processing" },
    { "recv_hello",     "OH", offsetof(olsrv2_t, log_recv_hello),
      "receiving HELLO message" },
    { "send_hello",     "Oh", offsetof(olsrv2_t, log_send_hello),
      "sending HELLO message" },
    { "recv_tc",        "OT", offsetof(olsrv2_t, log_recv_tc),
      "receiving TC message" },
    { "send_tc",        "Ot", offsetof(olsrv2_t, log_send_tc),
      "sending TC message" },
    { "calc_mpr",       "Om", offsetof(olsrv2_t, log_calc_mpr),
      "MPR calculation" },
    { "calc_route",     "Or", offsetof(olsrv2_t, log_calc_route),
      "routing calculation" },
    { "link_quality",   "Oq", offsetof(olsrv2_t, log_link_quality),
      "link quality" },
    { "link_metric",    "Ol", offsetof(olsrv2_t, log_link_metric),
      "link metric" },
    { "ibase_i",  "OIi",  offsetof(olsrv2_t, log_ibase_i),  "Local Interface Set" },
    { "ibase_ir", "OIir", offsetof(olsrv2_t, log_ibase_ir), "Removed Interface Address Set" },
    { "ibase_o",  "OIo",  offsetof(olsrv2_t, log_ibase_o),  "Originator Set" },
    { "ibase_al", "OIal", offsetof(olsrv2_t, log_ibase_al), "Local Attached Network Set" },
    { "ibase_l",  "OIl",  offsetof(olsrv2_t, log_ibase_l),  "Link Set" },
    { "ibase_n2", "OIn2", offsetof(olsrv2_t, log_ibase_n2), "2Hop Set" },
    { "ibase_n",  "OIn",  offsetof(olsrv2_t, log_ibase_n),  "Neighbor Set" },
    { "ibase_nl", "OInl", offsetof(olsrv2_t, log_ibase_nl), "Lost Neighbor Set" },
    { "ibase_ar", "OIar", offsetof(olsrv2_t, log_ibase_ar), "Advertising Remote Node Set" },
    { "ibase_ta", "OIta", offsetof(olsrv2_t, log_ibase_ta), "Routable Address Topology Set" },
    { "ibase_tr", "OItr", offsetof(olsrv2_t, log_ibase_tr), "Router Topology Set" },
    { "ibase_an", "OIan", offsetof(olsrv2_t, log_ibase_an), "Attached Network Set" },
    { "ibase_r",  "OIr",  offsetof(olsrv2_t, log_ibase_r),  "Routing Set" },
    { "ibase_p",  "OIp",  offsetof(olsrv2_t, log_ibase_p),  "Processed Set" },
    { "ibase_f",  "OIf",  offsetof(olsrv2_t, log_ibase_f),  "Forwarded Set" },
    { "ibase_rx", "OIrx", offsetof(olsrv2_t, log_ibase_rx), "Received Set" },
    { NULL },
};
/* *INDENT-ON* */

/** Initializes the olsrv2 module.
 *
 * @param self
 */
PUBLIC void
olsrv2_init(olsrv2_t* self)
{
    self->magic = OLSRv2_MAGIC;
    current_olsrv2 = self;
    current_core = self->core = nu_core_create();
#ifdef USE_NETIO
    current_netio = self->netio = nu_netio_create();
#endif
    current_packet = self->packet = nu_packet_create();
    PORT = DEFAULT_OLSRV2_PORT;
    NEXT_SEQNUM = 814;
    WILLINGNESS = WILLINGNESS__DEFAULT;
    ORIGINATOR  = NU_IP_UNDEF;

    olsrv2_ip_attr_init();

    nu_encoder_init(ENCODER);
    nu_decoder_init(DECODER);

    // Link metric parameters
    LINK_METRIC_TYPE  = LINK_METRIC_TYPE__NONE;
    ETX_MEMORY_LENGTH = DEFAULT_ETX_MEMORY_LENGTH;
    ETX_METRIC_INTERVAL = DEFAULT_ETX_METRIC_INTERVAL;
    ADVERTISE_TYPE = ADV_TYPE__MPRS;
    ADVERTISE_IN_METRIC  = 0;
    ADVERTISE_OUT_METRIC = 0;
    FMPR_TYPE  = MPR__BY_DEGREE;
    RMPR_TYPE  = MPR__BY_DEGREE_OR_METRIC;
    RELAY_TYPE = RELAY__RMPRS;
    olsrv2_metric_list_init();
    OLSR->update_by_metric = false;

    // Link quality default parameters
    LQ_TYPE = LQ__NO;
    HYST_SCALE  = LQ__DEFAULT_HYST_SCALE;
    HYST_ACCEPT = LQ__DEFAULT_HYST_ACCEPT;
    HYST_REJECT = LQ__DEFAULT_HYST_REJECT;
    INITIAL_QUALITY = LQ__DEFAULT_INITIAL_QUALITY;
    INITIAL_PENDING = (INITIAL_QUALITY < HYST_REJECT);
    LOSS_DETECT_SCALE = LQ__DEFAULT_LOSS_DETECT_SCALE;
    nu_event_list_init(LQ_EVENT_LIST);
    nu_event_list_init(METRIC_EVENT_LIST);

    // Forwarding queue
    olsrv2_pktq_init(&OLSR->forwardq);

    // Message intervals and jitter parameters
    GLOBAL_HELLO_INTERVAL = DEFAULT_HELLO_INTERVAL;
    GLOBAL_HELLO_MIN_INTERVAL = DEFAULT_HELLO_MIN_INTERVAL;
    GLOBAL_HP_MAXJITTER = DEFAULT_HP_MAXJITTER;
    GLOBAL_HT_MAXJITTER = DEFAULT_HT_MAXJITTER;
    START_HELLO = GLOBAL_HELLO_INTERVAL;
    TC_INTERVAL = DEFAULT_TC_INTERVAL;
    TC_MIN_INTERVAL = DEFAULT_TC_MIN_INTERVAL;
    TP_MAXJITTER = DEFAULT_TP_MAXJITTER;
    TT_MAXJITTER = DEFAULT_TT_MAXJITTER;
    START_TC = TC_INTERVAL;
    TC_HOP_LIMIT = DEFAULT_TC_HOP_LIMIT;
    F_MAXJITTER  = DEFAULT_F_MAXJITTER;

    // HOLD_TIME
    N_HOLD_TIME  = DEFAULT_N_HOLD_TIME;
    L_HOLD_TIME  = DEFAULT_L_HOLD_TIME;
    H_HOLD_TIME  = DEFAULT_H_HOLD_TIME;
    I_HOLD_TIME  = DEFAULT_I_HOLD_TIME;
    T_HOLD_TIME  = DEFAULT_T_HOLD_TIME;
    A_HOLD_TIME  = DEFAULT_A_HOLD_TIME;
    O_HOLD_TIME  = DEFAULT_O_HOLD_TIME;
    P_HOLD_TIME  = DEFAULT_P_HOLD_TIME;
    RX_HOLD_TIME = DEFAULT_RX_HOLD_TIME;
    F_HOLD_TIME  = DEFAULT_F_HOLD_TIME;

    // Initialize Information Bases
    ibase_i_init();
    ibase_ir_init();
    ibase_o_init();
    ibase_al_init();
    ibase_n_init();
    ibase_nl_init();
    ibase_ar_init();
    ibase_tr_init();
    ibase_ta_init();
    ibase_an_init();
    ibase_r_init();
    ibase_p_init();
    ibase_f_init();

    ibase_time_init(&OLSR->ibase_an_time_list);
    ibase_time_init(&OLSR->ibase_ar_time_list);
    ibase_time_init(&OLSR->ibase_ir_time_list);
    ibase_time_init(&OLSR->ibase_l_time_list);
    ibase_time_init(&OLSR->ibase_n2_time_list);
    ibase_time_init(&OLSR->ibase_nl_time_list);
    ibase_time_init(&OLSR->ibase_o_time_list);
    ibase_time_init(&OLSR->ibase_proc_time_list);
    ibase_time_init(&OLSR->ibase_ta_time_list);
    ibase_time_init(&OLSR->ibase_tr_time_list);

    nu_init_log_flags(olsrv2_log_flags, current_olsrv2);

#ifndef NUOLSRV2_NOSTAT
    olsrv2_stat_init();
#endif

    olsrv2_ip_attr_destroy();
}

/** Destroys the olsrv2 module.
 *
 * @param self
 */
PUBLIC void
olsrv2_destroy(olsrv2_t* self)
{
    current_olsrv2 = self;
    current_core = self->core;
#ifdef USE_NETIO
    current_netio = self->netio;
#endif
    current_packet = self->packet;

#ifndef NUOLSRV2_NOSTAT
    olsrv2_stat_put_log();
    olsrv2_stat_destroy();
#endif

    ibase_ir_destroy();
    ibase_o_destroy();
    ibase_al_destroy();
    ibase_n_destroy();
    ibase_nl_destroy();
    ibase_ar_destroy();
    ibase_tr_destroy();
    ibase_ta_destroy();
    ibase_an_destroy();
    ibase_r_destroy();
    ibase_p_destroy();
    ibase_f_destroy();
    ibase_i_destroy();

    olsrv2_pkt_t* pkt;
    while ((pkt = olsrv2_pktq_shift(&OLSR->forwardq)) != NULL) {
        nu_obuf_free((nu_obuf_t*)pkt->buf);
        olsrv2_pkt_free(pkt);
    }
    olsrv2_pktq_destroy(&OLSR->forwardq);

    olsrv2_metric_list_init();

    nu_event_list_destroy(LQ_EVENT_LIST);
    nu_event_list_destroy(METRIC_EVENT_LIST);

    nu_decoder_destroy(DECODER);
    nu_encoder_destroy(ENCODER);

    nu_packet_free(self->packet);
#ifdef USE_NETIO
    nu_netio_free(self->netio);
#endif
    nu_core_free(self->core);
}

/** Creates the olsrv2 module.
 *
 * @return the olsrv2 module
 */
PUBLIC olsrv2_t*
olsrv2_create(void)
{
    olsrv2_t* self = nu_mem_alloc(olsrv2_t);
    olsrv2_init(self);
    return self;
}

/** Destroys and frees the olsrv2 module.
 *
 * @param self
 */
PUBLIC void
olsrv2_free(olsrv2_t* self)
{
    olsrv2_destroy(self);
    nu_mem_free(self);
}

/** Checks whether the ip is my ip.
 *
 * @param ip
 * @return true if the ip is mine.
 */
PUBLIC nu_bool_t
olsrv2_is_my_ip(const nu_ip_t ip)
{
    FOREACH_I(tuple_i) {
        if (nu_ip_set_contain_without_prefix(&tuple_i->local_ip_list, ip))
            return true;
    }
    if (ibase_ir_contain(ip))
        return true;
    if (ibase_o_contain(ip))
        return true;
    if (ibase_al_contain(ip))
        return true;
    return false;
}

/** Sets the originator.
 */
PUBLIC void
olsrv2_set_originator(void)
{
    tuple_i_t* tuple_i;
    if ((tuple_i = ibase_i_head()) == NULL) {
        if (ORIGINATOR != NU_IP_UNDEF)
            ibase_o_add(ORIGINATOR);
        ORIGINATOR = NU_IP_UNDEF;
        return;
    }
    if (ORIGINATOR == NU_IP_UNDEF) {
        ORIGINATOR = tuple_i_local_ip(tuple_i);
        return;
    }
    if (!nu_ip_eq_without_prefix(ORIGINATOR, tuple_i_local_ip(tuple_i))) {
        ibase_o_add(ORIGINATOR);
        ORIGINATOR = tuple_i_local_ip(tuple_i);
    }
}

/** Gets the next sequence number.
 *
 * @return the sequence number
 */
PUBLIC uint16_t
olsrv2_next_msg_seqnum(void)
{
    return NEXT_SEQNUM++;
}

/*
 * HELLO Generator
 *
 * @see nu_event_func_t
 */
static void
hello_gen_event(nu_event_t* ev)
{
    tuple_i_t* tuple_i = (tuple_i_t*)ev->param;
    if (nu_time_sub(NU_NOW, tuple_i->last_hello_gen) < GLOBAL_HELLO_MIN_INTERVAL) {
        // TODO: check by sending time.
        return;
    }
    tuple_i->last_hello_gen = NU_NOW;

    nu_msg_t*  msg = olsrv2_hello_generate(tuple_i);
    nu_obuf_t* pkt = nu_encoder_encode_message(ENCODER, msg);
    tuple_i_sendq_enq(tuple_i, pkt);

    olsrv2_stat_after_hello_gen(pkt, tuple_i);

    nu_msg_free(msg);
}

/**
 * @return the number of advertised ip.
 */
PUBLIC size_t
olsrv2_update_advertise()
{
    size_t r = 0;

    // Update N_advertised.
    ibase_n_save_advertised();
    FOREACH_N(tuple_n) {
        if (!tuple_n->symmetric) {
            tuple_n_set_advertised(tuple_n, false);
            continue;
        }

        tuple_n_set_advertised(tuple_n, false);

        if (ADVERTISE_TYPE & ADV_TYPE__MPRS) {
            if (tuple_n->mpr_selector) {
                tuple_n_set_advertised(tuple_n, true);
            }
        }
        if (ADVERTISE_TYPE & ADV_TYPE__IN_METRIC) {
            if (tuple_n->_in_metric <= ADVERTISE_IN_METRIC) {
                tuple_n_set_advertised(tuple_n, true);
            }
        }
        if (ADVERTISE_TYPE & ADV_TYPE__OUT_METRIC) {
            if (tuple_n->_out_metric <= ADVERTISE_OUT_METRIC) {
                tuple_n_set_advertised(tuple_n, true);
            }
        }

        if (tuple_n->advertised)
            r += 1;
    }
    if (ibase_n_commit_advertised()) {
        IBASE_N->ansn += 1;
        olsrv2_stat_update_ansn();
#if defined(_WIN32) || defined(_WIN64)    //ScenSim-Port://
        NU_DEBUG_PROCESS0("update_ansn"); //ScenSim-Port://
#else                                     //ScenSim-Port://
        DEBUG_PROCESS0("update_ansn");
#endif                                    //ScenSim-Port://
    }

    r += ibase_al_size();

    if (r > 0) {
        LAST_NON_EMPTY_TC_TIME = NU_NOW;
        LAST_TC_IS_NON_EMPTY = true;
        return r;
    }
    if (LAST_TC_IS_NON_EMPTY) {
        LAST_TC_IS_NON_EMPTY = true;
        nu_time_t t = LAST_NON_EMPTY_TC_TIME;
        nu_time_add_f(&t, A_HOLD_TIME);
        return nu_time_cmp(NU_NOW, t) < 0;
    }
    return r;
}

/*
 * TC Generator
 *
 * @see nu_event_func_t
 */
static void
tc_gen_event(nu_event_t* ev)
{
    size_t adv_num = olsrv2_update_advertise();
    olsrv2_stat_advertise_num(adv_num);
    if (adv_num == 0)
        return;


    nu_obuf_t* buf = NULL;
    FOREACH_I(tuple_i) {
        if (buf == NULL) {
            nu_msg_t* msg = olsrv2_tc_generate();
            buf = nu_encoder_encode_message(ENCODER, msg);
            nu_msg_free(msg);
        }
        tuple_i_sendq_enq(tuple_i, nu_obuf_create_with_obuf(buf));

        olsrv2_stat_after_tc_gen(buf);
    }

    if (buf != NULL)
        nu_obuf_free(buf);
}

/*
 */
static void
ibase_clear_flags(void)
{
    ibase_l_clear_change_flag();
    ibase_l_clear_status();
    ibase_n2_clear_change_flag();
    ibase_ir_clear_change_flag();
    ibase_n_clear_change_flag();
    ibase_nl_clear_change_flag();
    ibase_ar_clear_change_flag();
    ibase_tr_clear_change_flag();
    ibase_ta_clear_change_flag();
    ibase_an_clear_change_flag();
    ibase_al_clear_change_flag();
    ibase_r_clear_change_flag();
}

/* [nhdp] Section 13.1 Link Tuple Symmetric
 */
static inline void
tuple_l_symmetric(tuple_l_t* tuple_l)
{
    tuple_n_t* tuple_n = ibase_n_search_tuple_l(tuple_l);
    assert(tuple_n != NULL);
    tuple_n_set_symmetric(tuple_n, true);
    FOREACH_IP_SET(i, &tuple_n->neighbor_ip_list) {
        ibase_nl_remove(i->ip);
    }
}

/* [nhdp] Section 13.2 Link Tuple Not Symmetric
 */
static inline void
tuple_l_not_symmetric(tuple_l_t* tuple_l)
{
    ibase_n2_remove_all(&tuple_l->ibase_n2);

    tuple_n_t* tuple_n = ibase_n_search_tuple_l(tuple_l);
    assert(tuple_n != NULL);
    if (nu_ip_set_size(&tuple_n->neighbor_ip_list) !=
        nu_ip_set_size(&tuple_l->neighbor_ip_list) &&
        tuple_n_has_symlink(tuple_n))
        return;
    tuple_n_set_symmetric(tuple_n, false);
    FOREACH_IP_SET(i, &tuple_n->neighbor_ip_list) {
        if (!ibase_nl_contain(i->ip))
            ibase_nl_add(i->ip);
    }
}

/* [nhdp] Section 13.3 Link Tuple Heard Timeout
 */
static inline void
tuple_l_heard_timeout(tuple_l_t* self)
{
    tuple_n_t* tuple_n = ibase_n_search_tuple_l(self);
    if (!tuple_n_has_link(tuple_n))
        (void)ibase_n_iter_remove(tuple_n);
}

/*
 * Update ibase_N based on ibase_L.
 */
static inline void
sync_ibases(void)
{
    // Save previous link metrics.
    FOREACH_N(tuple_n) {
        tuple_n->in_metric_save = tuple_n->_in_metric;
        tuple_n->_in_metric = UNDEF_METRIC;
        tuple_n->out_metric_save = tuple_n->_out_metric;
        tuple_n->_out_metric = UNDEF_METRIC;
        tuple_n->tuple_l_count = 0;
        tuple_n->flooding_mprs = false;
    }

    FOREACH_I(tuple_i) {
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            if (tuple_l->prev_status != tuple_l->status) {
                if (tuple_l->prev_status == LINK_STATUS__SYMMETRIC)
                    tuple_l_not_symmetric(tuple_l);
                else if (tuple_l->prev_status != LINK_STATUS__SYMMETRIC &&
                         tuple_l->status == LINK_STATUS__SYMMETRIC)
                    tuple_l_symmetric(tuple_l);
                else if (tuple_l->status == LINK_STATUS__LOST)
                    tuple_l_heard_timeout(tuple_l);
            }
            ++tuple_l->tuple_n->tuple_l_count;

            if (tuple_l->mpr_selector)
                tuple_l->tuple_n->flooding_mprs = true;

            if (tuple_l->tuple_n->_in_metric > tuple_l->in_metric)
                tuple_l->tuple_n->_in_metric = tuple_l->in_metric;
            if (tuple_l->tuple_n->_out_metric > tuple_l->out_metric)
                tuple_l->tuple_n->_out_metric = tuple_l->out_metric;
        }
    }

    for (tuple_n_t* tuple_n = ibase_n_iter();
         !ibase_n_iter_is_end(tuple_n);) {
        if (tuple_n->tuple_l_count == 0) {
            tuple_n = ibase_n_iter_remove(tuple_n);
            continue;
        }
        tuple_n_set_symmetric(tuple_n, tuple_n_has_symlink(tuple_n));
        if (tuple_n->in_metric_save != tuple_n->_in_metric ||
            tuple_n->out_metric_save != tuple_n->_out_metric) {
            IBASE_N->change = true;
            if (OLSR->update_by_metric && tuple_n->symmetric)
                IBASE_N->sym_change = true;
        }
        tuple_n = ibase_n_iter_next(tuple_n);
    }
}

/*
 */
static void
olsrv2_pre_process()
{
    ibase_clear_flags();

    ibase_time_remove(&OLSR->ibase_an_time_list);
    ibase_time_remove(&OLSR->ibase_ar_time_list);
    ibase_time_remove(&OLSR->ibase_ir_time_list);
    ibase_time_remove(&OLSR->ibase_l_time_list);
    ibase_time_remove(&OLSR->ibase_n2_time_list);
    ibase_time_remove(&OLSR->ibase_nl_time_list);
    ibase_time_remove(&OLSR->ibase_o_time_list);
    ibase_time_remove(&OLSR->ibase_proc_time_list);
    ibase_time_remove(&OLSR->ibase_ta_time_list);
    ibase_time_remove(&OLSR->ibase_tr_time_list);

    olsrv2_process_packets();

    nu_event_list_exec(LQ_EVENT_LIST);
    nu_event_list_exec(METRIC_EVENT_LIST);

    ibase_l_update_status();
    if (ibase_l_is_changed())
        sync_ibases();

    if (ibase_l_sym_is_changed() || IBASE_N->sym_change ||
        ibase_n2_is_changed()) {
#if defined(_WIN32) || defined(_WIN64)                   //ScenSim-Port://
        NU_DEBUG_PROCESS("calc_mpr:%s:%s:%s:",           //ScenSim-Port://
                (ibase_l_sym_is_changed() ? "L"  : "_"), //ScenSim-Port://
                (IBASE_N->sym_change ? "N"  : "_"),      //ScenSim-Port://
                (ibase_n2_is_changed() ? "N2" : "_"));   //ScenSim-Port://
#else                                                    //ScenSim-Port://
        DEBUG_PROCESS("calc_mpr:%s:%s:%s:",
                (ibase_l_sym_is_changed() ? "L"  : "_"),
                (IBASE_N->sym_change ? "N"  : "_"),
                (ibase_n2_is_changed() ? "N2" : "_"));
#endif                                                   //ScenSim-Port://
        olsrv2_calc_flooding_mpr();
        olsrv2_stat_after_fmpr_calc();
        if (USE_LINK_METRIC) {
            olsrv2_calc_routing_mpr();
            olsrv2_stat_after_rmpr_calc();
        }
    }
    if (ibase_l_sym_is_changed() || IBASE_N->sym_change ||
        ibase_n2_is_changed() || ibase_ar_is_changed() ||
        ibase_tr_is_changed() || ibase_ta_is_changed() ||
        ibase_an_is_changed()) {
#if defined(_WIN32) || defined(_WIN64)                    //ScenSim-Port://
        NU_DEBUG_PROCESS("calc_route:%s:%s:%s:%s:%s:%s:", //ScenSim-Port://
                (ibase_l_sym_is_changed()  ? "L"  : "_"), //ScenSim-Port://
                (IBASE_N->sym_change ? "N"  : "_"),       //ScenSim-Port://
                (ibase_n2_is_changed() ? "N2" : "_"),     //ScenSim-Port://
                (ibase_tr_is_changed() ? "TR"  : "_"),    //ScenSim-Port://
                (ibase_ta_is_changed() ? "TA"  : "_"),    //ScenSim-Port://
                (ibase_ar_is_changed() ? "AR" : "_"),     //ScenSim-Port://
                (ibase_an_is_changed() ? "AN" : "_"));    //ScenSim-Port://
#else                                                     //ScenSim-Port://
        DEBUG_PROCESS("calc_route:%s:%s:%s:%s:%s:%s:",
                (ibase_l_sym_is_changed()  ? "L"  : "_"),
                (IBASE_N->sym_change ? "N"  : "_"),
                (ibase_n2_is_changed() ? "N2" : "_"),
                (ibase_tr_is_changed() ? "TR"  : "_"),
                (ibase_ta_is_changed() ? "TA"  : "_"),
                (ibase_ar_is_changed() ? "AR" : "_"),
                (ibase_an_is_changed() ? "AN" : "_"));
#endif                                                    //ScenSim-Port://
        olsrv2_stat_before_route_calc();
        olsrv2_route_calc();
        olsrv2_stat_after_route_calc();
        if (ibase_r_is_changed()) {
#if defined(_WIN32) || defined(_WIN64)                        //ScenSim-Port://
            NU_DEBUG_PROCESS("flap_route:%s:%s:%s:%s:%s:%s:", //ScenSim-Port://
                    (ibase_l_sym_is_changed()  ? "L"  : "_"), //ScenSim-Port://
                    (IBASE_N->sym_change ? "N"  : "_"),       //ScenSim-Port://
                    (OLSR->ibase_n2_change ? "N2" : "_"),     //ScenSim-Port://
                    (ibase_tr_is_changed() ? "TR"  : "_"),    //ScenSim-Port://
                    (ibase_ta_is_changed() ? "TA"  : "_"),    //ScenSim-Port://
                    (ibase_ar_is_changed() ? "AR" : "_"),     //ScenSim-Port://
                    (ibase_an_is_changed() ? "AN" : "_"));    //ScenSim-Port://
#else                                                         //ScenSim-Port://
            DEBUG_PROCESS("flap_route:%s:%s:%s:%s:%s:%s:",
                    (ibase_l_sym_is_changed()  ? "L"  : "_"),
                    (IBASE_N->sym_change ? "N"  : "_"),
                    (OLSR->ibase_n2_change ? "N2" : "_"),
                    (ibase_tr_is_changed() ? "TR"  : "_"),
                    (ibase_ta_is_changed() ? "TA"  : "_"),
                    (ibase_ar_is_changed() ? "AR" : "_"),
                    (ibase_an_is_changed() ? "AN" : "_"));
#endif                                                        //ScenSim-Port://
        }
    }

#ifndef NU_NDEBUG             //ScenSim-Port://
    ibase_put_log(NU_LOGGER);
#endif                        //ScenSim-Port://
}

/*
 */
static void
olsrv2_post_process(void)
{
    olsrv2_send_packets();
}

/**  Verifys the parameters and initialize events.
 */
PUBLIC void
olsrv2_init_events(void)
{
    // Check willingness
    if (WILLINGNESS > 7) {
        nu_fatal("Willingness is %d, it must be 0 <= willingness <= 7.",
                WILLINGNESS);
    }

    // Check link quality parameters
    if (LQ_TYPE != LQ__NO) {
#define RANGE_CHECK(v, name)                              \
    if ((v) < 0.0 || (v) >= 1.0) {                        \
        nu_fatal(name " is %f, it must be [0, 1.0).", v); \
    }
        RANGE_CHECK(HYST_ACCEPT, "HYST_ACCEPT");
        RANGE_CHECK(HYST_REJECT, "HYST_REJECT");
        RANGE_CHECK(INITIAL_QUALITY, "INITIAL_QUALITY");

        if (HYST_ACCEPT < HYST_REJECT) {
            nu_fatal("HYST_ACCEPT(%f) < HYST_REJECT(%f).",
                    HYST_ACCEPT, HYST_REJECT);
        }
        if (INITIAL_QUALITY < HYST_REJECT) {
            if (INITIAL_PENDING != true) {
                nu_warn("if INITIAL_QUALITY < HYST_REJECT, INITIAL_PENDING must be true.");
                INITIAL_PENDING = true;
            }
        }
        if (INITIAL_QUALITY >= HYST_ACCEPT) {
            if (INITIAL_PENDING != false) {
                nu_warn("if INITIAL_QUALITY >= HYST_ACCEPT, INITIAL_PENDING must be true.");
                INITIAL_PENDING = false;
            }
        }
    }

    if (GLOBAL_HELLO_INTERVAL != DEFAULT_HELLO_INTERVAL) {
        H_HOLD_TIME = 3 * GLOBAL_HELLO_INTERVAL;
        L_HOLD_TIME = H_HOLD_TIME;
        N_HOLD_TIME = L_HOLD_TIME;
        I_HOLD_TIME = N_HOLD_TIME;
        GLOBAL_HELLO_MIN_INTERVAL = GLOBAL_HELLO_INTERVAL / 4.0;
    }
    if (TC_INTERVAL != DEFAULT_TC_INTERVAL) {
        T_HOLD_TIME = 3 * TC_INTERVAL;
        A_HOLD_TIME = T_HOLD_TIME;
        TC_MIN_INTERVAL = TC_INTERVAL / 4.0;
    }

    if (GLOBAL_HP_MAXJITTER != DEFAULT_HP_MAXJITTER) {
        GLOBAL_HT_MAXJITTER = GLOBAL_HP_MAXJITTER;
    }
    if (TP_MAXJITTER != DEFAULT_TP_MAXJITTER) {
        TT_MAXJITTER = TP_MAXJITTER;
        F_MAXJITTER  = TT_MAXJITTER;
    }

    if (GLOBAL_HELLO_INTERVAL - GLOBAL_HP_MAXJITTER < GLOBAL_HELLO_MIN_INTERVAL) {
        nu_fatal("HELLO_INTERVAL(%f) - HP_MAXJITTER(%f) must be greater than HELLO_MIN_INTERVAL(%f)",
                GLOBAL_HELLO_INTERVAL,
                GLOBAL_HP_MAXJITTER,
                GLOBAL_HELLO_MIN_INTERVAL);
    }

    if (TC_INTERVAL - TP_MAXJITTER < TC_MIN_INTERVAL) {
        nu_fatal("TC_INTERVAL(%f) - TP_MAXJITTER(%f) must be greater than TC_MIN_INTERVAL(%f)",
                TC_INTERVAL,
                TP_MAXJITTER,
                TC_MIN_INTERVAL);
    }

    FOREACH_I(tuple_i) {
        tuple_i_configure(tuple_i);
        if (tuple_i->manet) {
            if (!nu_ip_is_default_prefix(tuple_i_local_ip(tuple_i))) {
                nu_warn("MANET interface %s must have default prefix length.",
                        tuple_i->name);
            }
        }
    }

    olsrv2_set_originator();

    FOREACH_I(tuple_i) {
        if (!tuple_i->configured)
            continue;
        nu_scheduler_add_periodic_event(hello_gen_event, tuple_i,
                NU_EVENT_PRIO__DEFAULT,
                START_HELLO,
                tuple_i->hello_interval, tuple_i->hp_maxjitter,
                "hello_gen_event");
#ifdef USE_NETIO
        nu_socket_add_listener(tuple_i->sock,
                olsrv2_socket_listener, tuple_i);
#endif
    }
    nu_scheduler_add_periodic_event(tc_gen_event, NULL,
            NU_EVENT_PRIO__DEFAULT,
            START_TC,
            TC_INTERVAL, TP_MAXJITTER, "tc_gen_event");
    LAST_TC_IS_NON_EMPTY = false;
    nu_time_set_zero(&LAST_NON_EMPTY_TC_TIME);

    if (LINK_METRIC_TYPE == LINK_METRIC_TYPE__ETX) {
        nu_scheduler_add_periodic_event(etx_calc_metric_event, NULL,
                NU_EVENT_PRIO__DEFAULT,
                0,
                ETX_METRIC_INTERVAL, 0, "etx_calc_metric_event");
    }

    olsrv2_stat_after_init_event();

    nu_scheduler_set_pre_func(olsrv2_pre_process);
    nu_scheduler_set_post_func(olsrv2_post_process);
}

/** Shutdown the olsrv2 processing.
 */
PUBLIC void
olsrv2_shutdown(void)
{
    olsrv2_destroy(OLSR);
}

/** Outputs parameters for usage.
 *
 * @param prefix
 * @param fp
 */
PUBLIC void
olsrv2_put_params(const char* prefix, FILE* fp)
{
    fprintf(fp, "%s-Ov=(float) ... stat interval\n", prefix);
    nu_put_log_flags(prefix, olsrv2_log_flags, fp);
}

/** Sets the module parameter.
 *
 * @param opt
 * @return true if success
 */
PUBLIC nu_bool_t
olsrv2_set_param(const char* opt)
{
    if (nu_set_log_flag(opt, olsrv2_log_flags, current_olsrv2))
        return true;
#ifndef NUOLSRV2_NOSTAT
    const size_t Ov_len = strlen("-Ov=");
    if (strncmp("-Ov=", opt, Ov_len) == 0) {
        OLSR->stat.interval = atof(opt + Ov_len);
        return true;
    }
#endif
    return false;
}

#ifndef NU_NDEBUG

/**
 */
static void
ibase_put_log(nu_logger_t* logger)
{
    OLSRV2_DO_LOG(ibase_i,
            if (OLSR->ibase_i.change)
                ibase_i_put_log(logger);
            );

    OLSRV2_DO_LOG(ibase_ir,
            if (ibase_ir_is_changed())
                ibase_ir_put_log(logger);
            );
    OLSRV2_DO_LOG(ibase_o,
            if (ibase_o_is_changed())
                ibase_o_put_log(logger);
            );
    OLSRV2_DO_LOG(ibase_al,
            if (ibase_al_is_changed())
                ibase_al_put_log(logger);
            );
    OLSRV2_DO_LOG(ibase_l,
            if (ibase_l_is_changed()) {
                FOREACH_I(tuple_i) {
                    ibase_l_put_log(&tuple_i->ibase_l, tuple_i, logger);
                }
            }
            );
    OLSRV2_DO_LOG(ibase_n2,
            if (ibase_n2_is_changed()) {
                FOREACH_I(tuple_i) {
                    FOREACH_L(tuple_l, &tuple_i->ibase_l) {
                        ibase_n2_put_log(&tuple_l->ibase_n2,
                                tuple_i, tuple_l, logger);
                    }
                }
            }
            );
    OLSRV2_DO_LOG(ibase_n,
            if (ibase_n_is_changed())
                ibase_n_put_log(logger);
            );
    OLSRV2_DO_LOG(ibase_nl,
            if (ibase_nl_is_changed())
                ibase_nl_put_log(logger);
            );
    OLSRV2_DO_LOG(ibase_ar,
            if (ibase_ar_is_changed())
                ibase_ar_put_log(logger);
            );
    OLSRV2_DO_LOG(ibase_tr,
            if (ibase_tr_is_changed())
                ibase_tr_put_log(logger);
            );
    OLSRV2_DO_LOG(ibase_ta,
            if (ibase_ta_is_changed())
                ibase_ta_put_log(logger);
            );
    OLSRV2_DO_LOG(ibase_ar,
            if (ibase_an_is_changed())
                ibase_an_put_log(logger);
            );
    OLSRV2_DO_LOG(ibase_r,
            if (ibase_r_is_changed())
                ibase_r_put_log(logger);
            );
    OLSRV2_DO_LOG(ibase_p,
            ibase_p_put_log(logger);
            );
    OLSRV2_DO_LOG(ibase_rx,
            FOREACH_I(tuple_i) {
                ibase_rx_put_log(&tuple_i->ibase_rx, logger);
            }
            );
    OLSRV2_DO_LOG(ibase_f,
            ibase_f_put_log(logger);
            );
}

#endif

/** @} */

}//namespace// //ScenSim-Port://
