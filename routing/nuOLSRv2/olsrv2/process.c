#include "config.h"

#include "core/core.h"
#include "core/scheduler.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"
#include "olsrv2/pktq.h"
#include "olsrv2/debug_util.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup olsrv2
 * @{
 */

/*
 */
static void
forward_event(nu_event_t* ev)
{
    olsrv2_pkt_t* pkt;
    while ((pkt = olsrv2_pktq_shift(&OLSR->forwardq)) != NULL) {
        FOREACH_I(tuple_i) {
            if (!tuple_i->configured)
                continue;
            tuple_i_sendq_enq(tuple_i,
                    nu_obuf_dup((nu_obuf_t*)pkt->buf));
        }
        nu_obuf_free((nu_obuf_t*)pkt->buf);
        olsrv2_pkt_free(pkt);
    }
}

/*
 */
static void
process_message(nu_msg_t* msg, tuple_i_t* tuple_i,
        const nu_ip_t src_ip, const nu_link_metric_t metric,
        const int32_t pkt_seqnum)
{
    if (ibase_p_contain(msg))
        return;
    switch (msg->type) {
    case MSG_TYPE__HELLO:
        //ibase_p_add(msg); # Do not add to Proccessed Set case of HELLO MSG.
        if (nu_decoder_decode_msg_body(DECODER, msg)) {
            tuple_l_t* tuple_l =
                olsrv2_hello_process(msg, tuple_i, src_ip, metric);
            // ETX
            if (LINK_METRIC_TYPE == LINK_METRIC_TYPE__ETX &&
                pkt_seqnum != UNDEF_SEQNUM) {
                if (tuple_l->metric_last_pkt_seqnum == UNDEF_SEQNUM) {
                    lifo_set_top(&tuple_l->metric_received_lifo, 1);
                    lifo_set_top(&tuple_l->metric_total_lifo, 1);
                } else {
                    lifo_add_top(&tuple_l->metric_received_lifo, 1);
                    uint16_t diff =
                        nu_seqnum_diff(
                            static_cast<uint16_t>(pkt_seqnum),
                            static_cast<uint16_t>(tuple_l->metric_last_pkt_seqnum));
                    if (diff > ETX_SEQNUM_RESTART_DETECTION)
                        diff += 1;
                    lifo_add_top(&tuple_l->metric_total_lifo, diff);
                    tuple_l->metric_last_pkt_seqnum = pkt_seqnum;
                }
            }
        }
        break;
    case MSG_TYPE__TC:
        ibase_p_add(msg);
        if (nu_decoder_decode_msg_body(DECODER, msg))
            olsrv2_tc_process(msg, tuple_i, src_ip);
        break;
    default:
        ibase_p_add(msg);
        break;
    }
}

/*
 */
static void
forward_message(nu_msg_t* msg, tuple_i_t* tuple_i,
        const nu_ip_t src_ip)
{
    tuple_l_t* tuple_l = ibase_l_search_neighbor_ip_without_prefix(
            &tuple_i->ibase_l, src_ip);
    if (tuple_l == NULL || tuple_l->status != LINK_STATUS__SYMMETRIC)
        return;
    if (ibase_rx_contain(&tuple_i->ibase_rx, msg))
        return;
    ibase_rx_add(&tuple_i->ibase_rx, msg);

    if (ibase_f_contain(msg))
        return;
    ibase_f_add(msg);

    if (USE_LINK_METRIC) {
        if (RELAY_TYPE == RELAY__FMPRS) {
            if (!tuple_l->mpr_selector)
                return;
        } else if (RELAY_TYPE == RELAY__RMPRS) {
            tuple_n_t* tuple_n = ibase_n_search_tuple_l(tuple_l);
            if (!tuple_n->mpr_selector)
                return;
        } else if (RELAY_TYPE == RELAY__FMPRS_OR_RMPRS) {
            tuple_n_t* tuple_n = ibase_n_search_tuple_l(tuple_l);
            if (!tuple_l->mpr_selector && !tuple_n->mpr_selector)
                return;
        }
    } else {
        tuple_n_t* tuple_n = ibase_n_search_tuple_l(tuple_l);
        if (!tuple_n->mpr_selector)
            return;
    }

    if (nu_msg_has_hop_limit(msg))
        msg->hop_limit -= 1;
    if (nu_msg_has_hop_count(msg))
        msg->hop_count += 1;

#if defined(_WIN32) || defined(_WIN64)      //ScenSim-Port://
    NU_DEBUG_PROCESS0("packet_forwarding"); //ScenSim-Port://
#else                                       //ScenSim-Port://
    DEBUG_PROCESS0("packet_forwarding");
#endif                                      //ScenSim-Port://

    olsrv2_stat_before_tc_relay();

    olsrv2_pkt_t* pkt = olsrv2_pkt_create();
    pkt->buf = nu_encoder_encode_message(ENCODER, msg);
    if (pkt->buf != NULL) {
        olsrv2_pktq_insert_tail(&OLSR->forwardq, pkt);
        if (olsrv2_pktq_size(&OLSR->forwardq) == 1) {
            double d = F_MAXJITTER * nu_rand();
#if defined(_WIN32) || defined(_WIN64)                        //ScenSim-Port://
            NU_DEBUG_PROCESS("packet_enq_forwarding:%g:", d); //ScenSim-Port://
#else                                                         //ScenSim-Port://
            DEBUG_PROCESS("packet_enq_forwarding:%g:", d);
#endif                                                        //ScenSim-Port://
            nu_scheduler_add_event(forward_event, NULL,
                    d,
                    NU_EVENT_PRIO__DEFAULT, "forward");
        }
    } else {
        olsrv2_pkt_free(pkt);
    }
}

/*
 */
static void
process_packet(nu_ibuf_t* pkt, tuple_i_t* tuple_i, nu_ip_t src_ip)
{
    nu_decoder_set_ip_type(DECODER, nu_ip_type(src_ip));
    nu_pkthdr_t* hdr = nu_decoder_decode_pkthdr(DECODER, pkt);
    uint16_t pkt_seqnum = static_cast<uint16_t>(UNDEF_SEQNUM);
    if (LINK_METRIC_TYPE == LINK_METRIC_TYPE__ETX) {
        if (nu_pkthdr_has_seqnum(hdr))
            pkt_seqnum = hdr->seqnum;
    }
    nu_msg_t* msg = NULL;
    nu_link_metric_t metric = UNDEF_METRIC;
    if (LINK_METRIC_TYPE == LINK_METRIC_TYPE__STATIC)
        metric = olsrv2_metric_list_get(tuple_i, src_ip);
    else if (LINK_METRIC_TYPE == LINK_METRIC_TYPE__TEST)
        metric = olsrv2_get_link_metric(tuple_i, src_ip);
    while ((msg = nu_decoder_decode_msghdr(DECODER, pkt)) != NULL) {
        if (nu_msg_has_orig_addr(msg)) {
            if (olsrv2_is_my_ip(msg->orig_addr))
                goto cont;
        }
        olsrv2_stat_after_msg_recv(msg);
        switch (msg->type) {
        case MSG_TYPE__HELLO:
        case MSG_TYPE__TC:
            process_message(msg, tuple_i, src_ip, metric, pkt_seqnum);
            break;
        default:
#if defined(_WIN32) || defined(_WIN64)                     //ScenSim-Port://
            NU_DEBUG_PROCESS("unknown_packet:%I", src_ip); //ScenSim-Port://
#else                                                      //ScenSim-Port://
            DEBUG_PROCESS("unknown_packet:%I", src_ip);
#endif                                                     //ScenSim-Port://
        }
        if (nu_msg_has_hop_limit(msg) && msg->hop_limit > 1) {
            if (!nu_msg_has_hop_count(msg) || msg->hop_count < 255)
                forward_message(msg, tuple_i, src_ip);
        }
cont:
        nu_msg_free(msg);
    }
    if (hdr != NULL)
        nu_pkthdr_free(hdr);
}

/**
 * Received packets processor
 */
PUBLIC void
olsrv2_process_packets(void)
{
    FOREACH_I(tuple_i) {
        if (!tuple_i->configured)
            continue;
        nu_ibuf_t* pkt;
        while ((pkt = tuple_i_recvq_deq(tuple_i)) != NULL) {
            olsrv2_stat_after_pkt_recv(pkt);
            //if (!olsrv2_is_my_ip(pkt->src_ip))
            process_packet(pkt, tuple_i, pkt->src_ip);
            nu_ibuf_free(pkt);
        }
    }
}

/** @} */

}//namespace// //ScenSim-Port://
