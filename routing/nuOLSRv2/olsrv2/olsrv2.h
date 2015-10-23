#ifndef OLSRV2_H_
#define OLSRV2_H_

#include "config.h" //ScenSim-Port://
#include "core/core.h"
#ifdef USE_NETIO
#include "netio/netio.h"
#endif
#include "packet/packet.h"
#include "packet/decoder.h"
#include "packet/encoder.h"
#include "packet/link_metric.h"

#include "olsrv2/protocol.h"

#include "olsrv2/etx.h"
#include "olsrv2/metric_list.h"

#include "olsrv2/ibase_ip.h"

#include "olsrv2/ibase_n2.h"
#include "olsrv2/ibase_l.h"
#include "olsrv2/ibase_proc.h"

#include "olsrv2/ibase_i.h"
#include "olsrv2/ibase_ir.h"
#include "olsrv2/ibase_o.h"
#include "olsrv2/ibase_al.h"

#include "olsrv2/ibase_n.h"
#include "olsrv2/ibase_nl.h"

#include "olsrv2/ibase_ar.h"
#include "olsrv2/ibase_ta.h"
#include "olsrv2/ibase_tr.h"
#include "olsrv2/ibase_an.h"
#include "olsrv2/ibase_r.h"

#include "olsrv2/ibase_lt.h"
#include "olsrv2/ibase_l2.h"

#include "olsrv2/ip_attr.h"
#include "olsrv2/stat.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup olsrv2 OLSRv2 :: OLSRv2 module
 * @{
 */

/** Magic code for debug */
#define OLSRv2_MAGIC    (0x84012103)

/**
 * OLSRv2 module
 */
typedef struct olsrv2 {
    uint32_t         magic;        ///< magic code
    nu_core_t*       core;         ///< core module
#ifdef USE_NETIO
    nu_netio_t*      netio;        ///< netio module
#endif
    nu_packet_t*     packet;       ///< packet module

    nu_ip_t          originator;   ///< originator
    short            port;         ///< port
    uint16_t         next_seqnum;  ///< next sequence number
    uint8_t          willingness;  ///< willingness

    // Link metric parameters
    int              link_metric_type;    ///< link metric type
    double           etx_metric_interval; ///< etx metric interval
    int              etx_memory_length;   ///< etx metric interval
    int              advertise_type;      ///< advertise node selection type
    nu_link_metric_t adv_in_metric;       ///< maximum metric for advertising
    nu_link_metric_t adv_out_metric;      ///< maximum metric for advertising
    int              fmpr_type;           ///< flooding mpr selection type
    int              rmpr_type;           ///< routing mpr selection type
    int              relay_type;          ///< relay TC type
    metric_list_t    metric_list;         ///< link metric list
    nu_event_list_t  metric_event_list;   ///< link metric event list
    nu_bool_t        update_by_metric;    ///< TODO:

    // Link quality parameters
    int              lq_type;           ///< link quality type
    double           hyst_scale;        ///< link hysteresis scale param.
    double           hyst_accept;       ///< link hysteresis accept param.
    double           hyst_reject;       ///< link hysteresis reject param.
    double           initial_quality;   ///< initial link quality
    nu_bool_t        initial_pending;   ///< initial pending
    double           loss_detect_scale; ///< loss detect scale
    nu_event_list_t  lq_event_list;     ///< link quality event list

    // HELLO Interval and jitter parameters
    double           global_hello_interval;      ///< HELLO interval
    double           global_hello_min_interval;  ///< min HELLO interval
    double           global_hp_maxjitter;        ///< HP max jitter
    double           global_ht_maxjitter;        ///< HT max jitter
    double           start_hello;                ///< HELLO start time

    // TC Interval and jitter parameters
    double           tc_interval;            ///< TC interval
    double           tc_min_interval;        ///< minimal TC interval
    double           tp_maxjitter;           ///< TP max jitter
    double           tt_maxjitter;           ///< TT max jitter
    double           start_tc;               ///< TC start time
    uint8_t          tc_hop_limit;           ///< TC hop limit
    nu_bool_t        last_tc_is_non_empty;   ///< true if last tc is non empty
    nu_time_t        last_non_empty_tc_time; ///< time when last non empty was sent

    // Forwarding jitter parameters
    double           f_maxjitter;       ///< F max jitter

    // Hold time
    double           n_hold_time;   ///< N_HOLD_TIME
    double           l_hold_time;   ///< L_HOLD_TIME
    double           h_hold_time;   ///< H_HOLD_TIME
    double           i_hold_time;   ///< I_HOLD_TIME
    double           t_hold_time;   ///< T_HOLD_TIME
    double           o_hold_time;   ///< O_HOLD_TIME
    double           a_hold_time;   ///< A_HOLD_TIME
    double           p_hold_time;   ///< P_HOLD_TIME
    double           rx_hold_time;  ///< RX_HOLD_TIME
    double           f_hold_time;   ///< F_HOLD_TIME

    // Message decoder & encoder
    nu_decoder_t     decoder;     ///< packet decoder
    nu_encoder_t     encoder;     ///< packet encoder

    // Local Information Base
    ibase_i_t        ibase_i;    ///< Local Interface Set [nhdp]
    ibase_ir_t       ibase_ir;   ///< Removed Interface Address Set [nhdp]

    ibase_o_t        ibase_o;    ///< Originator Set [olsrv2]
    ibase_al_t       ibase_al;   ///< Local Attached Network Set [olsrv2]

    // Neighbor Information Base
    ibase_n_t        ibase_n;    ///< Neighbor Sets [nhdp + olsrv2]
    ibase_nl_t       ibase_nl;   ///< Lost Neighbor Set [nhdp]

    // Topology Information Base
    ibase_ar_t       ibase_ar;   ///< Advertising Remote Router Set [olsrv2]
    ibase_ta_t       ibase_ta;   ///< Routable Address Topology Set [olsrv2]
    ibase_tr_t       ibase_tr;   ///< Router Topology Set [olsrv2]
    ibase_an_t       ibase_an;   ///< Attached Network Set [olsrv2]
    ibase_r_t        ibase_r;    ///< Routing Set (map:dest -> tuple_r) [olsrv2]

    // Processing and Forwarding Repositories
    ibase_p_t        ibase_p;               ///< Processed Set [olsrv2]
    ibase_f_t        ibase_f;               ///< Forwarded Set [olsrv2]

    ibase_time_t     ibase_an_time_list;    ///< timeout tuple list for AN
    ibase_time_t     ibase_ar_time_list;    ///< timeout tuple list for AR
    ibase_time_t     ibase_ir_time_list;    ///< timeout tuple list for IR
    ibase_time_t     ibase_l_time_list;     ///< timeout tuple list for L
    ibase_time_t     ibase_n2_time_list;    ///< timeout tuple list for N2
    ibase_time_t     ibase_nl_time_list;    ///< timeout tuple list for NL
    ibase_time_t     ibase_o_time_list;     ///< timeout tuple list for O
    ibase_time_t     ibase_proc_time_list;  ///< timeout tuple list for P,F
    ibase_time_t     ibase_ta_time_list;    ///< timeout tuple list for TA
    ibase_time_t     ibase_tr_time_list;    ///< timeout tuple list for TR

    //
    nu_bool_t        ibase_l_change;        ///< change flag for L
    nu_bool_t        ibase_l_sym_change;    ///< change flag for L(sym)
    nu_bool_t        ibase_n2_change;       ///< change flag for N2

    // Forwarding packet's queue
    olsrv2_pktq_t    forwardq;      ///< forwarding packet queue

    // Log flags
    nu_bool_t        log_recv_hello;      ///< log flag: recv hello
    nu_bool_t        log_send_hello;      ///< log flag: send hello
    nu_bool_t        log_recv_tc;         ///< log flag: recv tc
    nu_bool_t        log_send_tc;         ///< log flag: send tc
    nu_bool_t        log_process;         ///< log flag: processing
    nu_bool_t        log_calc_mpr;        ///< log flag: mpr
    nu_bool_t        log_calc_route;      ///< log flag: route
    nu_bool_t        log_link_quality;    ///< log flag: link quality
    nu_bool_t        log_link_metric;     ///< log flag: link metric

    //nu_bool_t       log_ibase_proc;      ///< log flag: ibase proc
    //nu_bool_t       log_ibase_iface;     ///< log flag: ibase interface
    //nu_bool_t       log_ibase_local;     ///< log flag: ibase local
    //nu_bool_t       log_ibase_node;      ///< log flag: ibase node
    //nu_bool_t       log_ibase_topology;  ///< log flag: ibase topology

    nu_bool_t        log_ibase_i;  ///< log flag: ibase_i
    nu_bool_t        log_ibase_ir; ///< log flag: ibase_ir
    nu_bool_t        log_ibase_o;  ///< log flag: ibase_o
    nu_bool_t        log_ibase_al; ///< log flag: ibase_al
    nu_bool_t        log_ibase_l;  ///< log flag: ibase_l
    nu_bool_t        log_ibase_n2; ///< log flag: ibase_n2
    nu_bool_t        log_ibase_n;  ///< log flag: ibase_n
    nu_bool_t        log_ibase_nl; ///< log flag: ibase_nl
    nu_bool_t        log_ibase_ar; ///< log flag: ibase_ar
    nu_bool_t        log_ibase_ta; ///< log flag: ibase_ta
    nu_bool_t        log_ibase_tr; ///< log flag: ibase_tr
    nu_bool_t        log_ibase_an; ///< log flag: ibase_an
    nu_bool_t        log_ibase_r;  ///< log flag: ibase_r
    nu_bool_t        log_ibase_p;  ///< log flag: ibase_p
    nu_bool_t        log_ibase_f;  ///< log flag: ibae_f
    nu_bool_t        log_ibase_rx; ///< log flag: ibae_rx

#ifndef NUOLSRV2_NOSTAT
    olsrv2_stat_t    stat;  ///< statistics
#endif

#ifdef ADDON_NUOLSRv2
    RandomSeed       seed; ///< Qualnet's random seed
    double           eot;  ///< Stop time
#endif
} olsrv2_t;

//
#define OLSR                         (current_olsrv2)                  ///< current olsrv2
#define ORIGINATOR                   (OLSR->originator)                ///< accessor
#define WILLINGNESS                  (OLSR->willingness)               ///< accessor
#define PORT                         (OLSR->port)                      ///< accessor

#define NEXT_SEQNUM                  (OLSR->next_seqnum)               ///< accessor
#define ENCODER                      (&OLSR->encoder)                  ///< accessor
#define DECODER                      (&OLSR->decoder)                  ///< accessor

#define LINK_METRIC_TYPE             (OLSR->link_metric_type)          ///< accessor
#define ETX_MEMORY_LENGTH            (OLSR->etx_memory_length)         ///< accessor
#define ETX_METRIC_INTERVAL          (OLSR->etx_metric_interval)       ///< accessor
#define ADVERTISE_TYPE               (OLSR->advertise_type)            ///< accessor
#define ADVERTISE_IN_METRIC          (OLSR->adv_in_metric)             ///< accessor
#define ADVERTISE_OUT_METRIC         (OLSR->adv_out_metric)            ///< accessor
#define FMPR_TYPE                    (OLSR->fmpr_type)                 ///< accessor
#define RMPR_TYPE                    (OLSR->rmpr_type)                 ///< accessor
#define METRIC_EVENT_LIST            (&OLSR->metric_event_list)        ///< accessor
#define RELAY_TYPE                   (OLSR->relay_type)                ///< accessor

#define LQ_TYPE                      (OLSR->lq_type)                   ///< accessor
#define HYST_SCALE                   (OLSR->hyst_scale)                ///< accessor
#define HYST_ACCEPT                  (OLSR->hyst_accept)               ///< accessor
#define HYST_REJECT                  (OLSR->hyst_reject)               ///< accessor
#define INITIAL_QUALITY              (OLSR->initial_quality)           ///< accessor
#define INITIAL_PENDING              (OLSR->initial_pending)           ///< accessor
#define LOSS_DETECT_SCALE            (OLSR->loss_detect_scale)         ///< accessor
#define LQ_EVENT_LIST                (&OLSR->lq_event_list)            ///< accessor

#define GLOBAL_HELLO_INTERVAL        (OLSR->global_hello_interval)     ///< accessor
#define GLOBAL_HELLO_MIN_INTERVAL    (OLSR->global_hello_min_interval) ///< accessor
#define GLOBAL_HP_MAXJITTER          (OLSR->global_hp_maxjitter)       ///< accessor
#define GLOBAL_HT_MAXJITTER          (OLSR->global_ht_maxjitter)       ///< accessor
#define START_HELLO                  (OLSR->start_hello)               ///< accessor

#define TC_INTERVAL                  (OLSR->tc_interval)               ///< accessor
#define TC_MIN_INTERVAL              (OLSR->tc_min_interval)           ///< accessor
#define TP_MAXJITTER                 (OLSR->tp_maxjitter)              ///< accessor
#define TT_MAXJITTER                 (OLSR->tt_maxjitter)              ///< accessor
#define TC_HOP_LIMIT                 (OLSR->tc_hop_limit)              ///< accessor
#define START_TC                     (OLSR->start_tc)                  ///< accessor
#define F_MAXJITTER                  (OLSR->f_maxjitter)               ///< accessor
#define LAST_TC_IS_NON_EMPTY         (OLSR->last_tc_is_non_empty)      ///< accessor
#define LAST_NON_EMPTY_TC_TIME       (OLSR->last_non_empty_tc_time)    ///< accessor

#define N_HOLD_TIME                  (OLSR->n_hold_time)               ///< accessor
#define L_HOLD_TIME                  (OLSR->l_hold_time)               ///< accessor
#define H_HOLD_TIME                  (OLSR->h_hold_time)               ///< accessor
#define I_HOLD_TIME                  (OLSR->i_hold_time)               ///< accessor
#define T_HOLD_TIME                  (OLSR->t_hold_time)               ///< accessor
#define O_HOLD_TIME                  (OLSR->o_hold_time)               ///< accessor
#define A_HOLD_TIME                  (OLSR->a_hold_time)               ///< accessor
#define P_HOLD_TIME                  (OLSR->p_hold_time)               ///< accessor
#define RX_HOLD_TIME                 (OLSR->rx_hold_time)              ///< accessor
#define F_HOLD_TIME                  (OLSR->f_hold_time)               ///< accessor

/**
 * Link metric
 */
#define USE_LINK_METRIC              (LINK_METRIC_TYPE != LINK_METRIC_TYPE__NONE)

//
// MPR Selection algorithm
//
#define MPR__BY_DEGREE               (0) ///< by degree
#define MPR__BY_METRIC_PER_DEGREE    (1) ///< by metric per degree
#define MPR__BY_DEGREE_OR_METRIC     (2) ///< by degree or metric

//
// Advertising node type
//
#define ADV_TYPE__MPRS          (1 << 1)      ///< mpr selector
#define ADV_TYPE__IN_METRIC     (1 << 2)      ///< in metric
#define ADV_TYPE__OUT_METRIC    (1 << 3)      ///< out metric

//
// Forwarding packet type
//
#define RELAY__FMPRS                 (0) ///< flooding mpr selector
#define RELAY__RMPRS                 (1) ///< routing mpr selector
#define RELAY__FMPRS_OR_RMPRS        (2) ///< flooding or routing mpr selector

/*
 * Link quality type
 */
#define LQ__NO                       (0)      ///< disable LQ
#define LQ__HELLO                    (1 << 0) ///< LQ loss detect by time

/*
 * Link Layer Notification
 */
#define LLN__NO_USE                  (false) ///< disable LLN
#define RESTORATION__USE             (true)  ///< LLN parameter
#define RESTORATION__DEFAULT_TYPE    (1)     ///< LLN parameter

/** @fn OLSRV2_DO_LOG(flag, x)
 *
 * Output debug log
 *
 * @param flag
 * @param x
 */
#ifndef NU_NDEBUG
#define OLSRV2_DO_LOG(flag, x)          \
    if (current_olsrv2->log_ ## flag && \
        nu_logger_set_prio(NU_LOGGER, LOG_DEBUG)) { x }
//#define OLSRV2_LOG_INDENT    "    "
#else
#define OLSRV2_DO_LOG(flag, x)
#endif

/** Current olsrv2 module */
#ifdef nuOLSRv2_ALL_STATIC
static olsrv2_t* current_olsrv2 = NULL;
#else
extern olsrv2_t* current_olsrv2;
#endif

PUBLIC void olsrv2_init(olsrv2_t*);
PUBLIC void olsrv2_destroy(olsrv2_t*);

PUBLIC olsrv2_t* olsrv2_create(void);
PUBLIC void olsrv2_free(olsrv2_t*);

PUBLIC void olsrv2_init_events(void);

PUBLIC void olsrv2_set_originator(void);
PUBLIC uint16_t olsrv2_next_msg_seqnum(void);
PUBLIC nu_bool_t olsrv2_is_my_ip(const nu_ip_t);

PUBLIC size_t olsrv2_update_advertise(void);

PUBLIC void olsrv2_put_params(const char*, FILE*);
PUBLIC nu_bool_t olsrv2_set_param(const char*);

PUBLIC size_t olsrv2_update_advertise();

//PUBLIC void olsrv2_ibase_put_log(nu_logger_t*);
//PUBLIC void olsrv2_debug_put_ibase_event(nu_event_t*);

// hello.c /////////////////////////////////////////////////////

PUBLIC nu_msg_t* olsrv2_hello_generate(tuple_i_t*);
PUBLIC tuple_l_t* olsrv2_hello_process(nu_msg_t*, tuple_i_t*, const nu_ip_t, const nu_link_metric_t);

// tc.c ////////////////////////////////////////////////////////

PUBLIC nu_msg_t* olsrv2_tc_generate(void);
PUBLIC nu_bool_t olsrv2_tc_process(nu_msg_t*, tuple_i_t*, const nu_ip_t);

// packet.c ////////////////////////////////////////////////////

PUBLIC void olsrv2_process_packets(void);

// flooding_mpr.c //////////////////////////////////////////////

PUBLIC void olsrv2_calc_flooding_mpr(void);

// routing_mpr.c ///////////////////////////////////////////////

PUBLIC void olsrv2_calc_routing_mpr(void);

// route.c /////////////////////////////////////////////////////

PUBLIC void olsrv2_route_calc(void);

//// archdep functions

PUBLIC nu_link_metric_t olsrv2_get_link_metric(tuple_i_t* tuple_i, nu_ip_t src);

#ifdef USE_NETIO
PUBLIC void olsrv2_socket_listener(nu_socket_entry_t*);
#endif

/** Sends all the message in the sending queue.
 */
PUBLIC void olsrv2_send_packets(void);

/** @} */

}//namespace// //ScenSim-Port://

#endif
