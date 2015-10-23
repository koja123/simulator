//
// (I) Local Interface Set
//
#include "config.h"

#include "olsrv2/olsrv2.h"
#include "olsrv2/pktq.h"

#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
#define strdup(s) _strdup(s)           //ScenSim-Port://
#endif                                 //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_i
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tuple_i_t
//

/*
 */
static inline tuple_i_t*
tuple_i_create(const char* name)
{
    tuple_i_t* tuple = nu_mem_alloc(tuple_i_t);
    tuple->next = tuple->prev = NULL;
    tuple->name = strdup(name);
    tuple->iface_index = -1;
    nu_ip_set_init(&tuple->local_ip_list);
    tuple->manet = true;

    ibase_l_init(&tuple->ibase_l);
    ibase_rx_init(&tuple->ibase_rx);

    olsrv2_pktq_init(&tuple->recvq);
    olsrv2_pktq_init(&tuple->sendq);

    tuple->next_pkt_seqnum = (uint16_t)(nu_rand() * 0x10000);
    tuple->next_hello_msg_seqnum = 0;

    tuple->use_mcast = true;
    tuple->send_ip = NU_IP_UNDEF;
    tuple->recv_ip = NU_IP_UNDEF;
    tuple->ipv6_addr_type = 0;
    tuple->sock = -1;
    tuple->mtu  = 1000;
    tuple->configured = false;
    tuple->change = false;

    tuple->hello_interval = GLOBAL_HELLO_INTERVAL;
    tuple->hp_maxjitter = GLOBAL_HP_MAXJITTER;
    nu_time_set_zero(&tuple->last_hello_gen);

#ifndef NUOLSRV2_NOSTAT
    olsrv2_stat_iface_init(&tuple->stat);
#endif

    return tuple;
}

/*
 */
static inline void
tuple_i_free(tuple_i_t* tuple)
{
    tuple->next = tuple->prev = NULL;
    free(tuple->name);
    nu_ip_set_destroy(&tuple->local_ip_list);

    ibase_l_destroy(&tuple->ibase_l);
    ibase_rx_destroy(&tuple->ibase_rx);

    while (!olsrv2_pktq_is_empty(&tuple->recvq))
        nu_ibuf_free(tuple_i_recvq_deq(tuple));
    while (!olsrv2_pktq_is_empty(&tuple->sendq))
        nu_obuf_free(tuple_i_sendq_deq(tuple));

#ifndef NUOLSRV2_NOSTAT
    olsrv2_stat_iface_destroy(&tuple->stat);
#endif

    nu_mem_free(tuple);
}

/** Adds local ip.
 *
 * @param tuple
 * @param ip
 */
PUBLIC void
tuple_i_add_local_ip(tuple_i_t* tuple, nu_ip_t ip)
{
    nu_ip_set_add(&tuple->local_ip_list, ip);
    ibase_ir_remove(ip);
    // XXX
}

/** Appends the packet to the receive queue.
 *
 * @param tuple
 * @param buf
 */
PUBLIC void
tuple_i_recvq_enq(tuple_i_t* tuple, nu_ibuf_t* buf)
{
    olsrv2_pkt_t* pkt = olsrv2_pkt_create();
    pkt->buf = buf;
    olsrv2_pktq_insert_tail(&tuple->recvq, pkt);
}

/** Removes the top packet from the receive queue.
 *
 * @param tuple
 * @return the packet or NULL.
 */
PUBLIC nu_ibuf_t*
tuple_i_recvq_deq(tuple_i_t* tuple)
{
    olsrv2_pkt_t* pkt = olsrv2_pktq_shift(&tuple->recvq);
    if (pkt == NULL)
        return NULL;
    nu_ibuf_t* buf = (nu_ibuf_t*)pkt->buf;
    olsrv2_pkt_free(pkt);
    return buf;
}

/** Appends the packet to the send queue.
 *
 * @param tuple
 * @param buf
 */
PUBLIC void
tuple_i_sendq_enq(tuple_i_t* tuple, nu_obuf_t* buf)
{
    olsrv2_pkt_t* pkt = olsrv2_pkt_create();
    pkt->buf = buf;
    olsrv2_pktq_insert_tail(&tuple->sendq, pkt);
}

/** Remove the top packet from the send queue.
 *
 * @param tuple
 * @return the packet or NULL.
 */
PUBLIC nu_obuf_t*
tuple_i_sendq_deq(tuple_i_t* tuple)
{
    olsrv2_pkt_t* pkt = olsrv2_pktq_shift(&tuple->sendq);
    if (pkt == NULL)
        return NULL;
    nu_obuf_t* buf = (nu_obuf_t*)pkt->buf;
    olsrv2_pkt_free(pkt);
    return buf;
}

/** Builds packet.
 *
 * @param tuple
 * @return packed packet
 */
PUBLIC nu_obuf_t*
tuple_i_build_packet(tuple_i_t* tuple)
{
    if (olsrv2_pktq_is_empty(&tuple->sendq))
        return NULL;
    nu_pkthdr_t* pkthdr = nu_pkthdr_create();
    if (LINK_METRIC_TYPE == LINK_METRIC_TYPE__ETX)
        nu_pkthdr_set_seqnum(pkthdr, tuple_i_get_next_seqnum(tuple));
    nu_obuf_t* pkt = nu_encoder_encode_pkthdr(ENCODER, pkthdr);
    int msg_num = 0;
    while (!olsrv2_pktq_is_empty(&tuple->sendq)) {
        olsrv2_pkt_t* h = olsrv2_pktq_head(&tuple->sendq);
        if (msg_num != 0 &&
            nu_obuf_size((nu_obuf_t*)h->buf)
            + nu_obuf_size(pkt) > tuple->mtu)
            break;
        nu_obuf_t* p = tuple_i_sendq_deq(tuple);
        nu_obuf_append_obuf(pkt, p);

        olsrv2_stat_after_msg_send(p);

        nu_obuf_free(p);
        msg_num += 1;
    }
    nu_pkthdr_free(pkthdr);
    return pkt;
}

////////////////////////////////////////////////////////////////
//
// ibase_i_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_i_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Initializes the ibase_i.
 */
PUBLIC void
ibase_i_init(void)
{
    _ibase_i_init(IBASE_I);
}

/** Destroys the ibase_i.
 */
PUBLIC void
ibase_i_destroy(void)
{
    FOREACH_I(t) {
        tuple_i_restore_settings(t);
    }
    _ibase_i_destroy(IBASE_I);
}

/** Clears the change flag of the ibase_i.
 */
PUBLIC void
ibase_i_clear_change_flag(void)
{
    FOREACH_I(tuple_i)
    tuple_i->change = false;
    IBASE_I->change = false;
}

/** Checks whether interface has changed.
 *
 * @return ture if interface has changed.
 */
PUBLIC nu_bool_t
ibase_i_changed(void)
{
    if (IBASE_I->change)
        return true;
    FOREACH_I(tuple_i) {
        if (tuple_i->change)
            return(IBASE_I->change = true);
    }
    return false;
}

/** Adds tuple.
 *
 * @param ifname
 * @return new tuple
 */
PUBLIC tuple_i_t*
ibase_i_add(const char* ifname)
{
    tuple_i_t* new_tuple = tuple_i_create(ifname);
    _ibase_i_insert_tail(IBASE_I, new_tuple);
    IBASE_I->change = true;
    return new_tuple;
}

/** Gets tuple.
 *
 * @param ifname
 * @return tuple or NULL
 */
PUBLIC tuple_i_t*
ibase_i_search_name(const char* ifname)
{
    FOREACH_I(tuple_i) {
        if (strcmp(ifname, tuple_i->name) == 0)
            return tuple_i;
    }
    return NULL;
}

/** Gets the tuple at index.
 *
 * @param index
 * return tuple or NULL
 */
PUBLIC tuple_i_t*
ibase_i_search_index(int index)
{
    FOREACH_I(tuple_i) {
        if (tuple_i->iface_index == index)
            return tuple_i;
    }
    return NULL;
}

/** Removes the tuple.
 *
 * @param tuple
 * @return next tuple
 */
PUBLIC tuple_i_t*
ibase_i_iter_remove(tuple_i_t* tuple)
{
    nu_bool_t new_originator = false;
    FOREACH_IP_SET(p, &tuple->local_ip_list) {
        ibase_ir_add(p->ip);
        if (nu_ip_eq(p->ip, ORIGINATOR)) {
            ibase_o_add(p->ip);
            new_originator = true;
        }
    }
    tuple_i_t* next_tuple = _ibase_i_iter_remove(tuple, IBASE_I);
    if (new_originator) {
        if (ibase_i_iter_is_end(next_tuple))
            ORIGINATOR = NU_IP_UNDEF;
        else
            ORIGINATOR = tuple_i_local_ip(next_tuple);
    }
    IBASE_I->change = true;
    ibase_l_change();
    ibase_n2_change();
    return next_tuple;
}

/** Outputs tuple_i.
 *
 * @param tuple
 * @param logger
 */
PUBLIC void
tuple_i_put_log(tuple_i_t* tuple, nu_logger_t* logger)
{
    nu_logger_log(logger,
            "I_I:%s:[local:(%S)]",
            tuple->name, &tuple->local_ip_list);
}

/** Outputs ibase_i.
 *
 * @param logger
 */
PUBLIC void
ibase_i_put_log(nu_logger_t* logger)
{
    nu_logger_log(logger, "I_I:%d:--", ibase_i_size());
    FOREACH_I(p) {
        tuple_i_put_log(p, logger);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
