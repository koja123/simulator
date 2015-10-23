#include "config.h"

#include "olsrv2/olsrv2.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup olsrv2
 * @{
 */

#ifndef NUOLSRV2_NOSTAT

/** Current stat data */
#define STAT    (OLSR->stat)

/*
 */
static inline void
cum_int_init(olsrv2_stat_cum_t* self, uint32_t v)
{
    self->start_time = NU_NOW;
    self->time  = NU_NOW;
    self->value = v;
    self->total = 0.0;
}

static inline void
cum_int_add(olsrv2_stat_cum_t* self, uint32_t v)
{
    if (self->value != v) {
        self->total += (double)self->value * nu_time_sub(NU_NOW, self->time);
        self->value  = v;
        self->time   = NU_NOW;
    }
}

static inline double
cum_int_calc_avg(olsrv2_stat_cum_t* self)
{
    double total = self->total +
                   (double)self->value * nu_time_sub(NU_NOW, self->time);
    return total / nu_time_sub(NU_NOW, self->start_time);
}

/*
 */
static inline uint32_t
stat_calc_num_sym_link(void)
{
    uint32_t r = 0;
    FOREACH_I(tuple_i) {
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            if (tuple_l->status == LINK_STATUS__SYMMETRIC)
                r++;
        }
    }
    return r;
}

/*
 */
static inline uint32_t
stat_calc_num_adv(void)
{
    uint32_t r = 0;
    FOREACH_N(tuple_n) {
        if (tuple_n->advertised)
            r += 1;
    }
    return r;
}

/*
 */
static inline uint32_t
stat_calc_num_rmprs(void)
{
    uint32_t r = 0;
    FOREACH_N(tuple_n) {
        if (tuple_n->mpr_selector)
            r += 1;
    }
    return r;
}

/*
 */
static inline uint32_t
stat_calc_num_rmpr(void)
{
    uint32_t r = 0;
    FOREACH_N(tuple_n) {
        if (tuple_n->routing_mpr)
            r++;
    }
    return r;
}

/*
 */
static inline nu_bool_t
stat_calc_num_fmprs(void)
{
    uint32_t r = 0;
    FOREACH_I(tuple_i) {
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            if (tuple_l->mpr_selector)
                r += 1;
        }
    }
    return r;
}

/*
 */
static inline long
stat_calc_num_fmpr(void)
{
    unsigned int r = 0;
    FOREACH_N(tuple_n) {
        if (tuple_n->flooding_mpr)
            r++;
    }
    return r;
}

static inline void
stat_calc_hop(uint32_t table[MAX_HOPNUM])
{
    for (size_t i = 0; i < MAX_HOPNUM; ++i)
        table[i] = 0;
    FOREACH_R(tuple_r) {
        if (tuple_r->dist < MAX_HOPNUM)
            table[tuple_r->dist - 1]++;
        else
            table[MAX_HOPNUM - 1]++;
    }
}

/*
 */
static void
olsrv2_stat_clear(void)
{
    STAT.start_time = NU_NOW;

    FOREACH_IP_SET(i, &STAT.stat_ip_set) {
        olsrv2_ip_attr_t* si = olsrv2_ip_attr_get(nu_ip_set_iter_get(i));
        si->nhdpHelloMessageRecvd = 0;
        si->nhdpHelloMessageLost  = 0;
    }
    STAT.num_pkt_send  = 0;
    STAT.num_pkt_recv  = 0;
    STAT.send_pkt_size = 0;
    STAT.recv_pkt_size = 0;

    STAT.num_msg_send  = 0;
    STAT.num_msg_recv  = 0;
    STAT.send_msg_size = 0;
    STAT.recv_msg_size = 0;

    STAT.num_hello_recv = 0;
    STAT.hello_gen_num  = 0;
    STAT.hello_gen_size = 0;

    STAT.num_tc_recv = 0;
    STAT.tc_gen_num  = 0;
    STAT.tc_gen_size = 0;
    STAT.num_tc_relayed = 0;

    STAT.num_ansn_update = 0;

    STAT.num_calc_fmpr  = 0;
    STAT.num_calc_rmpr  = 0;
    STAT.num_calc_route = 0;
    STAT.num_flap_route = 0;

    STAT.fmpr_calc_L  = 0;
    STAT.fmpr_calc_N  = 0;
    STAT.fmpr_calc_N2 = 0;
    STAT.rmpr_calc_L  = 0;
    STAT.rmpr_calc_N  = 0;
    STAT.rmpr_calc_N2 = 0;

    STAT.route_calc_L = 0;
    STAT.route_calc_sym_N = 0;
    STAT.route_calc_N2 = 0;
    STAT.route_calc_TR = 0;
    STAT.route_calc_TA = 0;
    STAT.route_calc_AR = 0;
    STAT.route_calc_AN = 0;

    STAT.flap_L = 0;
    STAT.flap_sym_N = 0;
    STAT.flap_N2 = 0;
    STAT.flap_TR = 0;
    STAT.flap_TA = 0;
    STAT.flap_AR = 0;
    STAT.flap_AN = 0;

    cum_int_init(&STAT.num_advertised, stat_calc_num_adv());
    cum_int_init(&STAT.num_fmpr, stat_calc_num_fmpr());
    cum_int_init(&STAT.num_rmpr, stat_calc_num_rmpr());
    cum_int_init(&STAT.num_fmprs, stat_calc_num_fmprs());
    cum_int_init(&STAT.num_rmprs, stat_calc_num_rmprs());
    cum_int_init(&STAT.num_sym_link, stat_calc_num_sym_link());

    uint32_t hop_table[MAX_HOPNUM];
    stat_calc_hop(hop_table);
    for (int i = 0; i < MAX_HOPNUM; i++) {
        cum_int_init(&STAT.hops[i], hop_table[i]);
    }
}

static void
stat_periodic_event(nu_event_t* ev)
{
    olsrv2_stat_put_log();
    olsrv2_stat_clear();
    STAT.start_time = NU_NOW;
}

/** Destructor */
#define olsrv2_stat_ip_free    nu_mem_free

////////////////////////////////////////////////////////////////

/** Stat after initializing events.
 */
PUBLIC void
olsrv2_stat_after_init_event(void)
{
    olsrv2_stat_clear();
    if (STAT.interval > 0.0) {
        nu_scheduler_add_periodic_event(stat_periodic_event, NULL,
                NU_EVENT_PRIO__LOW,
                STAT.interval, STAT.interval, 0, "stat-periodic");
    }
}

/** Stat after generating HELLO.
 */
PUBLIC void
olsrv2_stat_after_hello_gen(const nu_obuf_t* pkt, tuple_i_t* tuple_i)
{
    tuple_i->stat.nhdpIfHelloMessageXmits += 1;
    tuple_i->stat.nhdpIfHelloMessagePeriodicXmits += 1;

    STAT.hello_gen_size += nu_obuf_size(pkt);
    STAT.hello_gen_num++;
}

/** Stat after generating TC.
 */
PUBLIC void
olsrv2_stat_after_tc_gen(const nu_obuf_t* pkt)
{
    STAT.tc_gen_size += nu_obuf_size(pkt);
    STAT.tc_gen_num++;
}

/** Stat before processing TC.
 */
PUBLIC void
olsrv2_stat_before_tc_process(void)
{
    STAT.num_tc_recv++;
}

/** Stat after calculating Flooding MPR.
 */
PUBLIC void
olsrv2_stat_after_fmpr_calc(void)
{
    STAT.num_calc_fmpr++;

    if (ibase_l_sym_is_changed())
        STAT.fmpr_calc_L++;
    if (IBASE_N->sym_change)
        STAT.fmpr_calc_N++;
    if (OLSR->ibase_n2_change)
        STAT.fmpr_calc_N2++;

    cum_int_add(&STAT.num_fmpr, stat_calc_num_fmpr());
}

/** Stat after calculating Routing MPR.
 */
PUBLIC void
olsrv2_stat_after_rmpr_calc(void)
{
    STAT.num_calc_rmpr++;

    if (ibase_l_sym_is_changed())
        STAT.rmpr_calc_L++;
    if (IBASE_N->sym_change)
        STAT.rmpr_calc_N++;
    if (OLSR->ibase_n2_change)
        STAT.rmpr_calc_N2++;

    cum_int_add(&STAT.num_rmpr, stat_calc_num_rmpr());
}

/** Stat after calculating routing table.
 */
PUBLIC void
olsrv2_stat_after_route_calc(void)
{
    STAT.num_calc_route++;

    if (ibase_l_sym_is_changed())
        STAT.route_calc_L++;
    if (IBASE_N->sym_change)
        STAT.route_calc_sym_N++;
    if (OLSR->ibase_n2_change)
        STAT.route_calc_N2++;
    if (ibase_tr_is_changed())
        STAT.route_calc_TR++;
    if (ibase_ta_is_changed())
        STAT.route_calc_TA++;
    if (ibase_ar_is_changed())
        STAT.route_calc_AR++;
    if (ibase_an_is_changed())
        STAT.route_calc_AN++;

    if (ibase_r_is_changed()) {
        if (ibase_l_sym_is_changed())
            STAT.flap_L++;
        if (IBASE_N->sym_change)
            STAT.flap_sym_N++;
        if (OLSR->ibase_n2_change)
            STAT.flap_N2++;
        if (ibase_tr_is_changed())
            STAT.flap_TR++;
        if (ibase_ta_is_changed())
            STAT.flap_TA++;
        if (ibase_ar_is_changed())
            STAT.flap_AR++;
        if (ibase_an_is_changed())
            STAT.flap_AN++;

        STAT.num_flap_route++;

        uint32_t hop_table[MAX_HOPNUM];
        stat_calc_hop(hop_table);
        for (int i = 0; i < MAX_HOPNUM; i++) {
            cum_int_add(&STAT.hops[i], hop_table[i]);
        }
    }

    cum_int_add(&STAT.num_sym_link, stat_calc_num_sym_link());
}

/** Stat before processing HELLO.
 */
PUBLIC void
olsrv2_stat_before_hello_process(void)
{
    STAT.num_hello_recv++;
}

/** Stat after processing HELLO.
 */
PUBLIC void
olsrv2_stat_after_hello_process(tuple_i_t* tuple_i,
        const nu_ip_t src, const uint16_t seqnum)
{
    tuple_i->stat.nhdpIfHelloMessageRecvd += 1;
    olsrv2_ip_attr_t* sip = olsrv2_ip_attr_get(src);
    if (!nu_ip_set_add(&STAT.stat_ip_set, src)) {
        sip->nhdpHelloMessageLost +=
            nu_seqnum_diff(seqnum, sip->last_seqnum) - 1;
    }
    sip->last_seqnum = seqnum;
    sip->nhdpHelloMessageRecvd += 1;

    cum_int_add(&STAT.num_fmprs, stat_calc_num_fmprs());
    cum_int_add(&STAT.num_rmprs, stat_calc_num_rmprs());
}

/** Stat after sending a message.
 */
PUBLIC void
olsrv2_stat_after_msg_send(const nu_obuf_t* buf)
{
    STAT.num_msg_send++;
    STAT.send_msg_size += (unsigned int)buf->len;
}

/** Stat before relaying TC.
 */
PUBLIC void
olsrv2_stat_before_tc_relay(void)
{
    STAT.num_tc_relayed++;
}

/** Stat after receving a message.
 */
PUBLIC void
olsrv2_stat_after_msg_recv(const nu_msg_t* msg)
{
    STAT.num_msg_recv++;
    STAT.recv_msg_size += (unsigned int)msg->size;
}

/** Stat after receiving a packet.
 */
PUBLIC void
olsrv2_stat_after_pkt_recv(const nu_ibuf_t* pkt)
{
    STAT.num_pkt_recv++;
    STAT.recv_pkt_size += pkt->len;
}

/** Stat before calculating routing table.
 */
PUBLIC void
olsrv2_stat_before_route_calc(void)
{
}

/** Stat after sending a packet.
 */
PUBLIC void
olsrv2_stat_after_pkt_send(const nu_obuf_t* pkt)
{
    STAT.num_pkt_send++;
    STAT.send_pkt_size += (unsigned int)pkt->len;
}

/** Stat when ansn is updated.
 */
PUBLIC void
olsrv2_stat_update_ansn(void)
{
    STAT.num_ansn_update += 1;
}

/** Stat the number of advertised ip.
 */
PUBLIC void
olsrv2_stat_advertise_num(size_t n)
{
    cum_int_add(&STAT.num_advertised, n);
}

////////////////////////////////////////////////////////////////

/**
 * Initializes the stat information of each interface.
 */
PUBLIC void
olsrv2_stat_iface_init(olsrv2_stat_iface_t* self)
{
    self->nhdpIfHelloMessageXmits = 0;
    self->nhdpIfHelloMessageRecvd = 0;
    self->nhdpIfHelloMessagePeriodicXmits  = 0;
    self->nhdpIfHelloMessageTriggeredXmits = 0;
}

/**
 * Destroys the stat information of each interface.
 */
PUBLIC void
olsrv2_stat_iface_destroy(olsrv2_stat_iface_t* self)
{
}

////////////////////////////////////////////////////////////////

/** Initializes the stat module.
 */
PUBLIC void
olsrv2_stat_init(void)
{
    STAT.interval = 0.0;
    nu_time_set_f(&STAT.start_time, 0);
    nu_ip_set_init(&STAT.stat_ip_set);
    olsrv2_stat_clear();
}

/** Destroys the stat module.
 */
PUBLIC void
olsrv2_stat_destroy(void)
{
    nu_ip_set_destroy(&STAT.stat_ip_set);
}

/** Outputs stat. information to default logger.
 */
PUBLIC void
olsrv2_stat_put_log(void)
{
#define STAT_LOG(s, ...)    nu_logger_log(NU_LOGGER, s, __VA_ARGS__)
#define STAT_LOG0(s)        nu_logger_log(NU_LOGGER, s)
#define STAT_LOG1(s, a)     nu_logger_log(NU_LOGGER, s, STAT.a)

    nu_logger_set_prio(NU_LOGGER, NU_LOGGER_INFO);
    nu_logger_push_prefix(NU_LOGGER, "STAT:");

    STAT_LOG("stat-period         %g", nu_time_sub(NU_NOW, STAT.start_time));

    switch (LINK_METRIC_TYPE) {
    case LINK_METRIC_TYPE__NONE:
        STAT_LOG0("Link_metric_type    no");
        break;
    case LINK_METRIC_TYPE__ETX:
        STAT_LOG0("Link_metric_type    etx");
        break;
    case LINK_METRIC_TYPE__STATIC:
        STAT_LOG0("Link_metric_type    static");
        break;
    case LINK_METRIC_TYPE__TEST:
        STAT_LOG0("Link_metric_type    test");
        break;
    default:
        nu_fatal("Unknown Link_metric_type:%d", LINK_METRIC_TYPE);
    }

    switch (LQ_TYPE) {
    case LQ__NO:
        STAT_LOG0("Link_quality_type   no");
        break;
    case LQ__HELLO:
        STAT_LOG0("Link_quality_type   hello");
        break;
    default:
        nu_fatal("Unknown Link_quality_type:%d", LQ_TYPE);
    }

    STAT_LOG("Advertised_node    %s%s%s",
            (ADVERTISE_TYPE & ADV_TYPE__MPRS) ? " mprs" : "",
            (ADVERTISE_TYPE & ADV_TYPE__IN_METRIC) ? " in-metric" : "",
            (ADVERTISE_TYPE & ADV_TYPE__OUT_METRIC) ? " out-metric" : "");

    switch (FMPR_TYPE) {
    case MPR__BY_DEGREE_OR_METRIC:
        STAT_LOG0("Fmpr_type           degree-or-metric");
        break;
    case MPR__BY_METRIC_PER_DEGREE:
        STAT_LOG0("Fmpr_type           metric-per-degree");
        break;
    case MPR__BY_DEGREE:
        STAT_LOG0("Fmpr_type           degree");
        break;
    default:
        nu_fatal("Unknown Fmpr_type");
    }

    switch (RMPR_TYPE) {
    case MPR__BY_DEGREE_OR_METRIC:
        STAT_LOG0("Rmpr_type           degree-or-metric");
        break;
    case MPR__BY_METRIC_PER_DEGREE:
        STAT_LOG0("Rmpr_type           metric-per-degree");
        break;
    case MPR__BY_DEGREE:
        STAT_LOG0("Rmpr_type           degree");
        break;
    default:
        nu_fatal("Unknown Rmpr_type");
    }

    switch (RELAY_TYPE) {
    case RELAY__FMPRS:
        STAT_LOG0("Relay_type          fmprs");
        break;
    case RELAY__RMPRS:
        STAT_LOG0("Relay_type          rmprs");
        break;
    case RELAY__FMPRS_OR_RMPRS:
        STAT_LOG0("Relay_type          fmprs-or-rmprs");
        break;
    default:
        nu_fatal("Unknown Realy_type");
    }

    STAT_LOG1("Hello_recv          %u", num_hello_recv);
    STAT_LOG1("Hello_gen           %u", hello_gen_num);
    STAT_LOG1("Hello_size          %u", hello_gen_size);
    FOREACH_IP_SET(i, &STAT.stat_ip_set) {
        const nu_ip_t     ip  = nu_ip_set_iter_get(i);
        olsrv2_ip_attr_t* sip = olsrv2_ip_attr_get(ip);
        STAT_LOG("Hello_recv_h         %I %u", ip,
                sip->nhdpHelloMessageRecvd);
        STAT_LOG("Hello_lost_h         %I %u", ip,
                sip->nhdpHelloMessageLost);
    }
    STAT_LOG1("TC_recv             %u", num_tc_recv);
    STAT_LOG1("TC_gen              %u", tc_gen_num);
    STAT_LOG1("TC_size             %u", tc_gen_size);
    STAT_LOG1("TC_relayed          %u", num_tc_relayed);

    STAT_LOG1("Message_recv_num    %u", num_msg_recv);
    STAT_LOG1("Message_send_num    %u", num_msg_send);
    STAT_LOG1("Message_recv_size   %u", recv_msg_size);
    STAT_LOG1("Message_send_size   %u", send_msg_size);
    STAT_LOG1("Packet_recv_num     %u", num_pkt_recv);
    STAT_LOG1("Packet_send_num     %u", num_pkt_send);
    STAT_LOG1("Packet_recv_size    %u", recv_pkt_size);
    STAT_LOG1("Packet_send_size    %u", send_pkt_size);

    STAT_LOG1("ANSN_update_num     %u", num_ansn_update);

    STAT_LOG1("Fmpr_calc_num       %u", num_calc_fmpr);
    STAT_LOG1("Fmpr_calc_by_L      %u", fmpr_calc_L);
    STAT_LOG1("Fmpr_calc_by_N      %u", fmpr_calc_N);
    STAT_LOG1("Fmpr_calc_by_N2     %u", fmpr_calc_N2);

    STAT_LOG1("Rmpr_calc_num       %u", num_calc_rmpr);
    STAT_LOG1("Rmpr_calc_by_L      %u", rmpr_calc_L);
    STAT_LOG1("Rmpr_calc_by_N      %u", rmpr_calc_N);
    STAT_LOG1("Rmpr_calc_by_N2     %u", rmpr_calc_N2);

    STAT_LOG1("Route_calc_num      %u", num_calc_route);
    STAT_LOG1("Route_calc_by_L     %u", route_calc_L);
    STAT_LOG1("Route_calc_by_sym_N %u", route_calc_sym_N);
    STAT_LOG1("Route_calc_by_N2    %u", route_calc_N2);
    STAT_LOG1("Route_calc_by_TR    %u", route_calc_TR);
    STAT_LOG1("Route_calc_by_TA    %u", route_calc_TA);
    STAT_LOG1("Route_calc_by_AR    %u", route_calc_AR);
    STAT_LOG1("Route_calc_by_AN    %u", route_calc_AN);

    STAT_LOG1("Route_flap_num      %u", num_flap_route);
    STAT_LOG1("Route_flap_by_L     %u", flap_L);
    STAT_LOG1("Route_flap_by_sym_N %u", flap_sym_N);
    STAT_LOG1("Route_flap_by_N2    %u", flap_N2);
    STAT_LOG1("Route_flap_by_TR    %u", flap_TR);
    STAT_LOG1("Route_flap_by_TA    %u", flap_TA);
    STAT_LOG1("Route_flap_by_AR    %u", flap_AR);
    STAT_LOG1("Route_flap_by_AN    %u", flap_AN);

    STAT_LOG("Avg_sym_link_num    %f", cum_int_calc_avg(&STAT.num_sym_link));
    STAT_LOG("Avg_fmpr_num        %f", cum_int_calc_avg(&STAT.num_fmpr));
    STAT_LOG("Avg_rmpr_num        %f", cum_int_calc_avg(&STAT.num_rmpr));
    STAT_LOG("Avg_fmprs_num       %f", cum_int_calc_avg(&STAT.num_fmprs));
    STAT_LOG("Avg_rmprs_num       %f", cum_int_calc_avg(&STAT.num_rmprs));
    STAT_LOG("Avg_advertised_num  %f", cum_int_calc_avg(&STAT.num_advertised));

    for (int i = 1; i < MAX_HOPNUM; i++) {
        STAT_LOG("Avg_%d_hop_num       %f", i,
                cum_int_calc_avg(&STAT.hops[i - 1]));
    }
    STAT_LOG("Avg_over_%d_hop_num  %f",
            MAX_HOPNUM, cum_int_calc_avg(&STAT.hops[MAX_HOPNUM - 1]));

    size_t total_l  = 0;
    size_t total_n2 = 0;
    size_t total_rx = 0;
    FOREACH_I(tuple_i) {
        total_l += ibase_l_size(&tuple_i->ibase_l);
        size_t n2 = 0;
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            n2 += ibase_n2_size(&tuple_l->ibase_n2);
        }
        total_n2 += n2;
        total_rx += ibase_rx_size(&tuple_i->ibase_rx);
    }

    STAT_LOG("Size_of_I           %u", ibase_i_size());
    STAT_LOG("Size_of_IR          %u", ibase_ir_size());
    STAT_LOG("Size_of_L           %u", total_l);
    STAT_LOG("Size_of_N2          %u", total_n2);
    STAT_LOG("Size_of_O           %u", ibase_o_size());
    STAT_LOG("Size_of_AL          %u", ibase_al_size());
    STAT_LOG("Size_of_N           %u", ibase_n_size());
    STAT_LOG("Size_of_NL          %u", ibase_nl_size());
    STAT_LOG("Size_of_AR          %u", ibase_ar_size());
    STAT_LOG("Size_of_TR          %u", ibase_tr_size());
    STAT_LOG("Size_of_TA          %u", ibase_ta_size());
    STAT_LOG("Size_of_AN          %u", ibase_an_size());
    STAT_LOG("Size_of_R           %u", ibase_r_size());
    STAT_LOG("Size_of_P           %u", ibase_p_size());
    STAT_LOG("Size_of_F           %u", ibase_f_size());
    STAT_LOG("Size_of_RX          %u", total_rx);

    nu_logger_pop_prefix(NU_LOGGER);
}

/** @} */

#endif

}//namespace// //ScenSim-Port://
