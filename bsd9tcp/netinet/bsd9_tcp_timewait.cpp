/*-
 * Copyright (c) 1982, 1986, 1988, 1990, 1993, 1995
 *  The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *  @(#)tcp_subr.c  8.2 (Berkeley) 5/24/95
 */

//ScenSim-Port//#include <sys/cdefs.h>
//ScenSim-Port//__FBSDID("$FreeBSD$");

//ScenSim-Port//#include "opt_inet.h"
//ScenSim-Port//#include "opt_inet6.h"
//ScenSim-Port//#include "opt_tcpdebug.h"

//ScenSim-Port//#include <sys/param.h>
//ScenSim-Port//#include <sys/systm.h>
//ScenSim-Port//#include <sys/callout.h>
//ScenSim-Port//#include <sys/kernel.h>
//ScenSim-Port//#include <sys/sysctl.h>
//ScenSim-Port//#include <sys/malloc.h>
//ScenSim-Port//#include <sys/mbuf.h>
//ScenSim-Port//#include <sys/priv.h>
//ScenSim-Port//#include <sys/proc.h>
//ScenSim-Port//#include <sys/socket.h>
//ScenSim-Port//#include <sys/socketvar.h>
//ScenSim-Port//#include <sys/protosw.h>
//ScenSim-Port//#include <sys/random.h>

//ScenSim-Port//#include <vm/uma.h>

//ScenSim-Port//#include <net/route.h>
//ScenSim-Port//#include <net/if.h>
//ScenSim-Port//#include <net/vnet.h>

//ScenSim-Port//#include <netinet/in.h>
//ScenSim-Port//#include <netinet/in_pcb.h>
//ScenSim-Port//#include <netinet/in_systm.h>
//ScenSim-Port//#include <netinet/in_var.h>
//ScenSim-Port//#include <netinet/ip.h>
//ScenSim-Port//#include <netinet/ip_icmp.h>
//ScenSim-Port//#include <netinet/ip_var.h>
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//#include <netinet/ip6.h>
//ScenSim-Port//#include <netinet6/in6_pcb.h>
//ScenSim-Port//#include <netinet6/ip6_var.h>
//ScenSim-Port//#include <netinet6/scope6_var.h>
//ScenSim-Port//#include <netinet6/nd6.h>
//ScenSim-Port//#endif
//ScenSim-Port//#include <netinet/tcp.h>
//ScenSim-Port//#include <netinet/tcp_fsm.h>
//ScenSim-Port//#include <netinet/tcp_seq.h>
//ScenSim-Port//#include <netinet/tcp_timer.h>
//ScenSim-Port//#include <netinet/tcp_var.h>
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//#include <netinet6/tcp6_var.h>
//ScenSim-Port//#endif
//ScenSim-Port//#include <netinet/tcpip.h>
//ScenSim-Port//#ifdef TCPDEBUG
//ScenSim-Port//#include <netinet/tcp_debug.h>
//ScenSim-Port//#endif
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//#include <netinet6/ip6protosw.h>
//ScenSim-Port//#endif

//ScenSim-Port//#include <machine/in_cksum.h>

//ScenSim-Port//#include <security/mac/mac_framework.h>
#include "tcp_porting.h"                        //ScenSim-Port//
namespace FreeBsd9Port {                                        //ScenSim-Port//

//ScenSim-Port//static VNET_DEFINE(uma_zone_t, tcptw_zone);
//ScenSim-Port//#define V_tcptw_zone            VNET(tcptw_zone)
//ScenSim-Port//static int  maxtcptw;

/*
 * The timed wait queue contains references to each of the TCP sessions
 * currently in the TIME_WAIT state.  The queue pointers, including the
 * queue pointers in each tcptw structure, are protected using the global
 * tcbinfo lock, which must be held over queue iteration and modification.
 */
//ScenSim-Port//static VNET_DEFINE(TAILQ_HEAD(, tcptw), twq_2msl);
//ScenSim-Port//#define V_twq_2msl          VNET(twq_2msl)

static void tcp_tw_2msl_reset(struct tcptw *, int);
static void tcp_tw_2msl_stop(struct tcptw *);

static int
tcptw_auto_size(void)
{
    int halfrange;

    /*
     * Max out at half the ephemeral port range so that TIME_WAIT
     * sockets don't tie up too many ephemeral ports.
     */
    if (V_ipport_lastauto > V_ipport_firstauto)
        halfrange = (V_ipport_lastauto - V_ipport_firstauto) / 2;
    else
        halfrange = (V_ipport_firstauto - V_ipport_lastauto) / 2;
    /* Protect against goofy port ranges smaller than 32. */
    return (imin(imax(halfrange, 32), maxsockets / 5));
}

//ScenSim-Port//static int
//ScenSim-Port//sysctl_maxtcptw(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    int error, new;
//ScenSim-Port//
//ScenSim-Port//    if (maxtcptw == 0)
//ScenSim-Port//        new = tcptw_auto_size();
//ScenSim-Port//    else
//ScenSim-Port//        new = maxtcptw;
//ScenSim-Port//    error = sysctl_handle_int(oidp, &new, 0, req);
//ScenSim-Port//    if (error == 0 && req->newptr)
//ScenSim-Port//        if (new >= 32) {
//ScenSim-Port//            maxtcptw = new;
//ScenSim-Port//            uma_zone_set_max(V_tcptw_zone, maxtcptw);
//ScenSim-Port//        }
//ScenSim-Port//    return (error);
//ScenSim-Port//}

//ScenSim-Port//SYSCTL_PROC(_net_inet_tcp, OID_AUTO, maxtcptw, CTLTYPE_INT|CTLFLAG_RW,
//ScenSim-Port//    &maxtcptw, 0, sysctl_maxtcptw, "IU",
//ScenSim-Port//    "Maximum number of compressed TCP TIME_WAIT entries");

//ScenSim-Port//VNET_DEFINE(int, nolocaltimewait) = 0;
//ScenSim-Port//#define V_nolocaltimewait   VNET(nolocaltimewait)
//ScenSim-Port//SYSCTL_VNET_INT(_net_inet_tcp, OID_AUTO, nolocaltimewait, CTLFLAG_RW,
//ScenSim-Port//    &VNET_NAME(nolocaltimewait), 0,
//ScenSim-Port//    "Do not create compressed TCP TIME_WAIT entries for local connections");

void
tcp_tw_zone_change(void)
{

    if (maxtcptw == 0)
        uma_zone_set_max(V_tcptw_zone, tcptw_auto_size());
}

void
tcp_tw_init(void)
{

    V_tcptw_zone = uma_zcreate("tcptw", sizeof(struct tcptw),
        NULL, NULL, NULL, NULL, UMA_ALIGN_PTR, UMA_ZONE_NOFREE);
//ScenSim-Port//    TUNABLE_INT_FETCH("net.inet.tcp.maxtcptw", &maxtcptw);
    if (maxtcptw == 0)
        uma_zone_set_max(V_tcptw_zone, tcptw_auto_size());
    else
        uma_zone_set_max(V_tcptw_zone, maxtcptw);
    TAILQ_INIT(&V_twq_2msl);
}

//ScenSim-Port//#ifdef VIMAGE
void
tcp_tw_destroy(void)
{
    struct tcptw *tw;

//ScenSim-Port//    INP_INFO_WLOCK(&V_tcbinfo);
    while((tw = TAILQ_FIRST(&V_twq_2msl)) != NULL)
        tcp_twclose(tw, 0);
//ScenSim-Port//    INP_INFO_WUNLOCK(&V_tcbinfo);

    uma_zdestroy(V_tcptw_zone);
}
//ScenSim-Port//#endif

/*
 * Move a TCP connection into TIME_WAIT state.
 *    tcbinfo is locked.
 *    inp is locked, and is unlocked before returning.
 */
void
tcp_twstart(struct tcpcb *tp)
{
    struct tcptw *tw;
    struct inpcb *inp = tp->t_inpcb;
    int acknow;
    struct socket *so;
#ifdef INET6
//ScenSim-Port//    int isipv6 = inp->inp_inc.inc_flags & INC_ISIPV6;
#endif

//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);  /* tcp_tw_2msl_reset(). */
//ScenSim-Port//    INP_WLOCK_ASSERT(inp);

//ScenSim-Port//    if (V_nolocaltimewait) {
//ScenSim-Port//        int error = 0;
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//        if (isipv6)
//ScenSim-Port//            error = in6_localaddr(&inp->in6p_faddr);
//ScenSim-Port//#endif
//ScenSim-Port//#if defined(INET6) && defined(INET)
//ScenSim-Port//        else
//ScenSim-Port//#endif
//ScenSim-Port//#ifdef INET
//ScenSim-Port//            error = in_localip(inp->inp_faddr);
//ScenSim-Port//#endif
//ScenSim-Port//        if (error) {
//ScenSim-Port//            tp = tcp_close(tp);
//ScenSim-Port//            if (tp != NULL)
//ScenSim-Port//                INP_WUNLOCK(inp);
//ScenSim-Port//            return;
//ScenSim-Port//        }
//ScenSim-Port//    }

//ScenSim-Port//    tw = uma_zalloc(V_tcptw_zone, M_NOWAIT);
    tw = (struct tcptw *)uma_zalloc(V_tcptw_zone, M_NOWAIT);    //ScenSim-Port//
    if (tw == NULL) {
        tw = tcp_tw_2msl_scan(1);
        if (tw == NULL) {
            tp = tcp_close(tp);
//ScenSim-Port//            if (tp != NULL)
//ScenSim-Port//                INP_WUNLOCK(inp);
            return;
        }
    }
    tw->tw_inpcb = inp;

    /*
     * Recover last window size sent.
     */
//ScenSim-Port//    KASSERT(SEQ_GEQ(tp->rcv_adv, tp->rcv_nxt),
//ScenSim-Port//        ("tcp_twstart negative window: tp %p rcv_nxt %u rcv_adv %u", tp,
//ScenSim-Port//        tp->rcv_nxt, tp->rcv_adv));
    tw->last_win = (tp->rcv_adv - tp->rcv_nxt) >> tp->rcv_scale;

    /*
     * Set t_recent if timestamps are used on the connection.
     */
    if ((tp->t_flags & (TF_REQ_TSTMP|TF_RCVD_TSTMP|TF_NOOPT)) ==
        (TF_REQ_TSTMP|TF_RCVD_TSTMP)) {
        tw->t_recent = tp->ts_recent;
        tw->ts_offset = tp->ts_offset;
    } else {
        tw->t_recent = 0;
        tw->ts_offset = 0;
    }

    tw->snd_nxt = tp->snd_nxt;
    tw->rcv_nxt = tp->rcv_nxt;
    tw->iss     = tp->iss;
    tw->irs     = tp->irs;
    tw->t_starttime = tp->t_starttime;
    tw->tw_time = 0;

/* XXX
 * If this code will
 * be used for fin-wait-2 state also, then we may need
 * a ts_recent from the last segment.
 */
    acknow = tp->t_flags & TF_ACKNOW;

    /*
     * First, discard tcpcb state, which includes stopping its timers and
     * freeing it.  tcp_discardcb() used to also release the inpcb, but
     * that work is now done in the caller.
     *
     * Note: soisdisconnected() call used to be made in tcp_discardcb(),
     * and might not be needed here any longer.
     */
    tcp_discardcb(tp);
    so = inp->inp_socket;
    soisdisconnected(so);
//ScenSim-Port//    tw->tw_cred = crhold(so->so_cred);
//ScenSim-Port//    SOCK_LOCK(so);
    tw->tw_so_options = so->so_options;
//ScenSim-Port//    SOCK_UNLOCK(so);
    if (acknow)
        tcp_twrespond(tw, TH_ACK);
    inp->inp_ppcb = tw;
    inp->inp_flags |= INP_TIMEWAIT;
    tcp_tw_2msl_reset(tw, 0);

    /*
     * If the inpcb owns the sole reference to the socket, then we can
     * detach and free the socket as it is not needed in time wait.
     */
    if (inp->inp_flags & INP_SOCKREF) {
//ScenSim-Port//        KASSERT(so->so_state & SS_PROTOREF,
//ScenSim-Port//            ("tcp_twstart: !SS_PROTOREF"));
        inp->inp_flags &= ~INP_SOCKREF;
//ScenSim-Port//        INP_WUNLOCK(inp);
//ScenSim-Port//        ACCEPT_LOCK();
//ScenSim-Port//        SOCK_LOCK(so);
        so->so_state &= ~SS_PROTOREF;
        sofree(so);
    } else
//ScenSim-Port//        INP_WUNLOCK(inp);
    ;                                                           //ScenSim-Port//
}

//ScenSim-Port//#if 0
//ScenSim-Port///*
//ScenSim-Port// * The appromixate rate of ISN increase of Microsoft TCP stacks;
//ScenSim-Port// * the actual rate is slightly higher due to the addition of
//ScenSim-Port// * random positive increments.
//ScenSim-Port// *
//ScenSim-Port// * Most other new OSes use semi-randomized ISN values, so we
//ScenSim-Port// * do not need to worry about them.
//ScenSim-Port// */
//ScenSim-Port//#define MS_ISN_BYTES_PER_SECOND     250000
//ScenSim-Port//
//ScenSim-Port///*
//ScenSim-Port// * Determine if the ISN we will generate has advanced beyond the last
//ScenSim-Port// * sequence number used by the previous connection.  If so, indicate
//ScenSim-Port// * that it is safe to recycle this tw socket by returning 1.
//ScenSim-Port// */
//ScenSim-Port//int
//ScenSim-Port//tcp_twrecycleable(struct tcptw *tw)
//ScenSim-Port//{
//ScenSim-Port//    tcp_seq new_iss = tw->iss;
//ScenSim-Port//    tcp_seq new_irs = tw->irs;
//ScenSim-Port//
//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
//ScenSim-Port//    new_iss += (ticks - tw->t_starttime) * (ISN_BYTES_PER_SECOND / hz);
//ScenSim-Port//    new_irs += (ticks - tw->t_starttime) * (MS_ISN_BYTES_PER_SECOND / hz);
//ScenSim-Port//
//ScenSim-Port//    if (SEQ_GT(new_iss, tw->snd_nxt) && SEQ_GT(new_irs, tw->rcv_nxt))
//ScenSim-Port//        return (1);
//ScenSim-Port//    else
//ScenSim-Port//        return (0);
//ScenSim-Port//}
//ScenSim-Port//#endif

/*
 * Returns 1 if the TIME_WAIT state was killed and we should start over,
 * looking for a pcb in the listen state.  Returns 0 otherwise.
 */
int
tcp_twcheck(struct inpcb *inp, struct tcpopt *to, struct tcphdr *th,
    struct mbuf *m, int tlen)
{
    struct tcptw *tw;
    int thflags;
    tcp_seq seq;

    /* tcbinfo lock required for tcp_twclose(), tcp_tw_2msl_reset(). */
//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK_ASSERT(inp);

    /*
     * XXXRW: Time wait state for inpcb has been recycled, but inpcb is
     * still present.  This is undesirable, but temporarily necessary
     * until we work out how to handle inpcb's who's timewait state has
     * been removed.
     */
    tw = intotw(inp);
    if (tw == NULL)
        goto drop;

    thflags = th->th_flags;

    /*
     * NOTE: for FIN_WAIT_2 (to be added later),
     * must validate sequence number before accepting RST
     */

    /*
     * If the segment contains RST:
     *  Drop the segment - see Stevens, vol. 2, p. 964 and
     *      RFC 1337.
     */
    if (thflags & TH_RST)
        goto drop;

//ScenSim-Port//#if 0
//ScenSim-Port///* PAWS not needed at the moment */
//ScenSim-Port//    /*
//ScenSim-Port//     * RFC 1323 PAWS: If we have a timestamp reply on this segment
//ScenSim-Port//     * and it's less than ts_recent, drop it.
//ScenSim-Port//     */
//ScenSim-Port//    if ((to.to_flags & TOF_TS) != 0 && tp->ts_recent &&
//ScenSim-Port//        TSTMP_LT(to.to_tsval, tp->ts_recent)) {
//ScenSim-Port//        if ((thflags & TH_ACK) == 0)
//ScenSim-Port//            goto drop;
//ScenSim-Port//        goto ack;
//ScenSim-Port//    }
//ScenSim-Port//    /*
//ScenSim-Port//     * ts_recent is never updated because we never accept new segments.
//ScenSim-Port//     */
//ScenSim-Port//#endif

    /*
     * If a new connection request is received
     * while in TIME_WAIT, drop the old connection
     * and start over if the sequence numbers
     * are above the previous ones.
     */
    if ((thflags & TH_SYN) && SEQ_GT(th->th_seq, tw->rcv_nxt)) {
        tcp_twclose(tw, 0);
        return (1);
    }

    /*
     * Drop the segment if it does not contain an ACK.
     */
    if ((thflags & TH_ACK) == 0)
        goto drop;

    /*
     * Reset the 2MSL timer if this is a duplicate FIN.
     */
    if (thflags & TH_FIN) {
        seq = th->th_seq + tlen + (thflags & TH_SYN ? 1 : 0);
        if (seq + 1 == tw->rcv_nxt)
            tcp_tw_2msl_reset(tw, 1);
    }

    /*
     * Acknowledge the segment if it has data or is not a duplicate ACK.
     */
    if (thflags != TH_ACK || tlen != 0 ||
        th->th_seq != tw->rcv_nxt || th->th_ack != tw->snd_nxt)
        tcp_twrespond(tw, TH_ACK);
drop:
//ScenSim-Port//    INP_WUNLOCK(inp);
    m_freem(m);
    return (0);
}

void
tcp_twclose(struct tcptw *tw, int reuse)
{
    struct socket *so;
    struct inpcb *inp;

    /*
     * At this point, we are in one of two situations:
     *
     * (1) We have no socket, just an inpcb<->twtcp pair.  We can free
     *     all state.
     *
     * (2) We have a socket -- if we own a reference, release it and
     *     notify the socket layer.
     */
    inp = tw->tw_inpcb;
//ScenSim-Port//    KASSERT((inp->inp_flags & INP_TIMEWAIT), ("tcp_twclose: !timewait"));
//ScenSim-Port//    KASSERT(intotw(inp) == tw, ("tcp_twclose: inp_ppcb != tw"));
//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);  /* tcp_tw_2msl_stop(). */
//ScenSim-Port//    INP_WLOCK_ASSERT(inp);

    tw->tw_inpcb = NULL;
    tcp_tw_2msl_stop(tw);
    inp->inp_ppcb = NULL;
    in_pcbdrop(inp);

    so = inp->inp_socket;
    if (so != NULL) {
        /*
         * If there's a socket, handle two cases: first, we own a
         * strong reference, which we will now release, or we don't
         * in which case another reference exists (XXXRW: think
         * about this more), and we don't need to take action.
         */
        if (inp->inp_flags & INP_SOCKREF) {
            inp->inp_flags &= ~INP_SOCKREF;
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            ACCEPT_LOCK();
//ScenSim-Port//            SOCK_LOCK(so);
//ScenSim-Port//            KASSERT(so->so_state & SS_PROTOREF,
//ScenSim-Port//                ("tcp_twclose: INP_SOCKREF && !SS_PROTOREF"));
            so->so_state &= ~SS_PROTOREF;
            sofree(so);
        } else {
            /*
             * If we don't own the only reference, the socket and
             * inpcb need to be left around to be handled by
             * tcp_usr_detach() later.
             */
//ScenSim-Port//            INP_WUNLOCK(inp);
        }
    } else
        in_pcbfree(inp);
    TCPSTAT_INC(tcps_closed);
//ScenSim-Port//    crfree(tw->tw_cred);
//ScenSim-Port//    tw->tw_cred = NULL;
    if (reuse)
        return;
    uma_zfree(V_tcptw_zone, tw);
}

int
tcp_twrespond(struct tcptw *tw, int flags)
{
    struct inpcb *inp = tw->tw_inpcb;
#if defined(INET6) || defined(INET)
    struct tcphdr *th = NULL;
#endif
    struct mbuf *m;
#ifdef INET
    struct ip *ip = NULL;
#endif
    u_int hdrlen, optlen;
    int error = 0;          /* Keep compiler happy */
    struct tcpopt to;
#ifdef INET6
    struct ip6_hdr *ip6 = NULL;
    int isipv6 = inp->inp_inc.inc_flags & INC_ISIPV6;
#endif

//ScenSim-Port//    INP_WLOCK_ASSERT(inp);

    m = m_gethdr(M_DONTWAIT, MT_DATA);
    if (m == NULL)
        return (ENOBUFS);
//ScenSim-Port//    m->m_data += max_linkhdr;

//ScenSim-Port//#ifdef MAC
//ScenSim-Port//    mac_inpcb_create_mbuf(inp, m);
//ScenSim-Port//#endif

#ifdef INET6
    if (isipv6) {
        hdrlen = sizeof(struct ip6_hdr) + sizeof(struct tcphdr);
        ip6 = mtod(m, struct ip6_hdr *);
        th = (struct tcphdr *)(ip6 + 1);
        tcpip_fillheaders(inp, ip6, th);
    }
#endif
#if defined(INET6) && defined(INET)
    else
#endif
#ifdef INET
    {
        hdrlen = sizeof(struct tcpiphdr);
        ip = mtod(m, struct ip *);
        th = (struct tcphdr *)(ip + 1);
        tcpip_fillheaders(inp, ip, th);
    }
#endif
    to.to_flags = 0;

    /*
     * Send a timestamp and echo-reply if both our side and our peer
     * have sent timestamps in our SYN's and this is not a RST.
     */
    if (tw->t_recent && flags == TH_ACK) {
        to.to_flags |= TOF_TS;
        to.to_tsval = ticks + tw->ts_offset;
        to.to_tsecr = tw->t_recent;
    }
    optlen = tcp_addoptions(&to, (u_char *)(th + 1));

    m->m_len = hdrlen + optlen;
    m->m_pkthdr.len = m->m_len;

//ScenSim-Port//    KASSERT(max_linkhdr + m->m_len <= MHLEN, ("tcptw: mbuf too small"));

    th->th_seq = htonl(tw->snd_nxt);
    th->th_ack = htonl(tw->rcv_nxt);
    th->th_off = (sizeof(struct tcphdr) + optlen) >> 2;
    th->th_flags = flags;
    th->th_win = htons(tw->last_win);

#ifdef INET6
    if (isipv6) {
        th->th_sum = in6_cksum(m, IPPROTO_TCP, sizeof(struct ip6_hdr),
            sizeof(struct tcphdr) + optlen);
        ip6->ip6_hlim = in6_selecthlim(inp, NULL);
//ScenSim-Port//        error = ip6_output(m, inp->in6p_outputopts, NULL,
//ScenSim-Port//            (tw->tw_so_options & SO_DONTROUTE), NULL, NULL, inp);
        error = ip_output(m, NULL, NULL,                        //ScenSim-Port//
            ((tw->tw_so_options & SO_DONTROUTE) ?               //ScenSim-Port//
            IP_ROUTETOIF : 0), NULL, inp, 0, 0);                //ScenSim-Port//
    }
#endif
#if defined(INET6) && defined(INET)
    else
#endif
#ifdef INET
    {
        th->th_sum = in_pseudo(ip->ip_src.s_addr, ip->ip_dst.s_addr,
            htons(sizeof(struct tcphdr) + optlen + IPPROTO_TCP));
        m->m_pkthdr.csum_flags = CSUM_TCP;
        m->m_pkthdr.csum_data = offsetof(struct tcphdr, th_sum);
        ip->ip_len = m->m_pkthdr.len;
        if (V_path_mtu_discovery)
            ip->ip_off |= IP_DF;
//ScenSim-Port//        error = ip_output(m, inp->inp_options, NULL,
//ScenSim-Port//            ((tw->tw_so_options & SO_DONTROUTE) ? IP_ROUTETOIF : 0),
//ScenSim-Port//            NULL, inp);
        error = ip_output(m, inp->inp_options, NULL,            //ScenSim-Port//
            ((tw->tw_so_options & SO_DONTROUTE)                 //ScenSim-Port//
                ? IP_ROUTETOIF : 0),                            //ScenSim-Port//
            NULL, inp, 0, 0);                                   //ScenSim-Port//
    }
#endif
    if (flags & TH_ACK)
        TCPSTAT_INC(tcps_sndacks);
    else
        TCPSTAT_INC(tcps_sndctrl);
    TCPSTAT_INC(tcps_sndtotal);
    return (error);
}

static void
tcp_tw_2msl_reset(struct tcptw *tw, int rearm)
{

//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK_ASSERT(tw->tw_inpcb);
    if (TAILQ_EMPTY(&V_twq_2msl))                               //ScenSim-Port//
        callout_reset(&tcp_slowtimo_callout,                    //ScenSim-Port//
            tcp_slowtimo_timeout, tcp_slowtimo, NULL);          //ScenSim-Port//
    if (rearm)
        TAILQ_REMOVE(&V_twq_2msl, tw, tw_2msl);
    tw->tw_time = ticks + 2 * tcp_msl;
    TAILQ_INSERT_TAIL(&V_twq_2msl, tw, tw_2msl);
}

static void
tcp_tw_2msl_stop(struct tcptw *tw)
{

//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
    TAILQ_REMOVE(&V_twq_2msl, tw, tw_2msl);
    if (TAILQ_EMPTY(&V_twq_2msl))                               //ScenSim-Port//
        callout_stop(&tcp_slowtimo_callout);                    //ScenSim-Port//
}

struct tcptw *
tcp_tw_2msl_scan(int reuse)
{
    struct tcptw *tw;

//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
    for (;;) {
        tw = TAILQ_FIRST(&V_twq_2msl);
        if (tw == NULL || (!reuse && (tw->tw_time - ticks) > 0))
            break;
//ScenSim-Port//        INP_WLOCK(tw->tw_inpcb);
        tcp_twclose(tw, reuse);
        if (reuse)
            return (tw);
    }
    return (NULL);
}
}//namespace//                                                  //ScenSim-Port//
