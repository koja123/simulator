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

//ScenSim-Port//#include "opt_compat.h"
//ScenSim-Port//#include "opt_inet.h"
//ScenSim-Port//#include "opt_inet6.h"
//ScenSim-Port//#include "opt_ipsec.h"
//ScenSim-Port//#include "opt_tcpdebug.h"

//ScenSim-Port//#include <sys/param.h>
//ScenSim-Port//#include <sys/systm.h>
//ScenSim-Port//#include <sys/callout.h>
//ScenSim-Port//#include <sys/hhook.h>
//ScenSim-Port//#include <sys/kernel.h>
//ScenSim-Port//#include <sys/khelp.h>
//ScenSim-Port//#include <sys/sysctl.h>
//ScenSim-Port//#include <sys/jail.h>
//ScenSim-Port//#include <sys/malloc.h>
//ScenSim-Port//#include <sys/mbuf.h>
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//#include <sys/domain.h>
//ScenSim-Port//#endif
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

//ScenSim-Port//#include <netinet/cc.h>
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

//ScenSim-Port//#include <netinet/tcp_fsm.h>
//ScenSim-Port//#include <netinet/tcp_seq.h>
//ScenSim-Port//#include <netinet/tcp_timer.h>
//ScenSim-Port//#include <netinet/tcp_var.h>
//ScenSim-Port//#include <netinet/tcp_syncache.h>
//ScenSim-Port//#include <netinet/tcp_offload.h>
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

//ScenSim-Port//#ifdef IPSEC
//ScenSim-Port//#include <netipsec/ipsec.h>
//ScenSim-Port//#include <netipsec/xform.h>
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//#include <netipsec/ipsec6.h>
//ScenSim-Port//#endif
//ScenSim-Port//#include <netipsec/key.h>
//ScenSim-Port//#include <sys/syslog.h>
//ScenSim-Port//#endif /*IPSEC*/

//ScenSim-Port//#include <machine/in_cksum.h>
//ScenSim-Port//#include <sys/bsd9_md5.h>

//ScenSim-Port//#include <security/mac/mac_framework.h>
#include "tcp_porting.h"                        //ScenSim-Port//
namespace FreeBsd9Port {                                        //ScenSim-Port//

//ScenSim-Port//VNET_DEFINE(int, tcp_mssdflt) = TCP_MSS;
#ifdef INET6
//ScenSim-Port//VNET_DEFINE(int, tcp_v6mssdflt) = TCP6_MSS;
#endif

//ScenSim-Port//static int
//ScenSim-Port//sysctl_net_inet_tcp_mss_check(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    int error, new;
//ScenSim-Port//
//ScenSim-Port//    new = V_tcp_mssdflt;
//ScenSim-Port//    error = sysctl_handle_int(oidp, &new, 0, req);
//ScenSim-Port//    if (error == 0 && req->newptr) {
//ScenSim-Port//        if (new < TCP_MINMSS)
//ScenSim-Port//            error = EINVAL;
//ScenSim-Port//        else
//ScenSim-Port//            V_tcp_mssdflt = new;
//ScenSim-Port//    }
//ScenSim-Port//    return (error);
//ScenSim-Port//}

//ScenSim-Port//SYSCTL_VNET_PROC(_net_inet_tcp, TCPCTL_MSSDFLT, mssdflt,
//ScenSim-Port//    CTLTYPE_INT|CTLFLAG_RW, &VNET_NAME(tcp_mssdflt), 0,
//ScenSim-Port//    &sysctl_net_inet_tcp_mss_check, "I",
//ScenSim-Port//    "Default TCP Maximum Segment Size");

#ifdef INET6
//ScenSim-Port//static int
//ScenSim-Port//sysctl_net_inet_tcp_mss_v6_check(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    int error, new;
//ScenSim-Port//
//ScenSim-Port//    new = V_tcp_v6mssdflt;
//ScenSim-Port//    error = sysctl_handle_int(oidp, &new, 0, req);
//ScenSim-Port//    if (error == 0 && req->newptr) {
//ScenSim-Port//        if (new < TCP_MINMSS)
//ScenSim-Port//            error = EINVAL;
//ScenSim-Port//        else
//ScenSim-Port//            V_tcp_v6mssdflt = new;
//ScenSim-Port//    }
//ScenSim-Port//    return (error);
//ScenSim-Port//}

//ScenSim-Port//SYSCTL_VNET_PROC(_net_inet_tcp, TCPCTL_V6MSSDFLT, v6mssdflt,
//ScenSim-Port//    CTLTYPE_INT|CTLFLAG_RW, &VNET_NAME(tcp_v6mssdflt), 0,
//ScenSim-Port//    &sysctl_net_inet_tcp_mss_v6_check, "I",
//ScenSim-Port//   "Default TCP Maximum Segment Size for IPv6");
#endif /* INET6 */

/*
 * Minimum MSS we accept and use. This prevents DoS attacks where
 * we are forced to a ridiculous low MSS like 20 and send hundreds
 * of packets instead of one. The effect scales with the available
 * bandwidth and quickly saturates the CPU and network interface
 * with packet generation and sending. Set to zero to disable MINMSS
 * checking. This setting prevents us from sending too small packets.
 */
//ScenSim-Port//VNET_DEFINE(int, tcp_minmss) = TCP_MINMSS;
//ScenSim-Port//SYSCTL_VNET_INT(_net_inet_tcp, OID_AUTO, minmss, CTLFLAG_RW,
//ScenSim-Port//     &VNET_NAME(tcp_minmss), 0,
//ScenSim-Port//    "Minmum TCP Maximum Segment Size");

//ScenSim-Port//VNET_DEFINE(int, tcp_do_rfc1323) = 1;
//ScenSim-Port//SYSCTL_VNET_INT(_net_inet_tcp, TCPCTL_DO_RFC1323, rfc1323, CTLFLAG_RW,
//ScenSim-Port//    &VNET_NAME(tcp_do_rfc1323), 0,
//ScenSim-Port//    "Enable rfc1323 (high performance TCP) extensions");

//ScenSim-Port//static int  tcp_log_debug = 0;
#define tcp_log_debug SCENSIMGLOBAL(tcp_log_debug)              //ScenSim-Port//
//ScenSim-Port//SYSCTL_INT(_net_inet_tcp, OID_AUTO, log_debug, CTLFLAG_RW,
//ScenSim-Port//    &tcp_log_debug, 0, "Log errors caused by incoming TCP segments");

//ScenSim-Port//static int  tcp_tcbhashsize = 0;
#define tcp_tcbhashsize SCENSIMGLOBAL(tcp_tcbhashsize)          //ScenSim-Port//
//ScenSim-Port//SYSCTL_INT(_net_inet_tcp, OID_AUTO, tcbhashsize, CTLFLAG_RDTUN,
//ScenSim-Port//    &tcp_tcbhashsize, 0, "Size of TCP control-block hashtable");

//ScenSim-Port//static int  do_tcpdrain = 1;
//ScenSim-Port//SYSCTL_INT(_net_inet_tcp, OID_AUTO, do_tcpdrain, CTLFLAG_RW, &do_tcpdrain, 0,
//ScenSim-Port//    "Enable tcp_drain routine for extra help when low on mbufs");

//ScenSim-Port//SYSCTL_VNET_UINT(_net_inet_tcp, OID_AUTO, pcbcount, CTLFLAG_RD,
//ScenSim-Port//    &VNET_NAME(tcbinfo.ipi_count), 0, "Number of active PCBs");

//ScenSim-Port//static VNET_DEFINE(int, icmp_may_rst) = 1;
//ScenSim-Port//#define V_icmp_may_rst          VNET(icmp_may_rst)
//ScenSim-Port//SYSCTL_VNET_INT(_net_inet_tcp, OID_AUTO, icmp_may_rst, CTLFLAG_RW,
//ScenSim-Port//    &VNET_NAME(icmp_may_rst), 0,
//ScenSim-Port//    "Certain ICMP unreachable messages may abort connections in SYN_SENT");

//ScenSim-Port//static VNET_DEFINE(int, tcp_isn_reseed_interval) = 0;
//ScenSim-Port//#define V_tcp_isn_reseed_interval   VNET(tcp_isn_reseed_interval)
//ScenSim-Port//SYSCTL_VNET_INT(_net_inet_tcp, OID_AUTO, isn_reseed_interval, CTLFLAG_RW,
//ScenSim-Port//    &VNET_NAME(tcp_isn_reseed_interval), 0,
//ScenSim-Port//    "Seconds between reseeding of ISN secret");

//ScenSim-Port//static int  tcp_soreceive_stream = 0;
//ScenSim-Port//SYSCTL_INT(_net_inet_tcp, OID_AUTO, soreceive_stream, CTLFLAG_RDTUN,
//ScenSim-Port//    &tcp_soreceive_stream, 0, "Using soreceive_stream for TCP sockets");

//ScenSim-Port//#ifdef TCP_SIGNATURE
//ScenSim-Port//static int  tcp_sig_checksigs = 1;
//ScenSim-Port//SYSCTL_INT(_net_inet_tcp, OID_AUTO, signature_verify_input, CTLFLAG_RW,
//ScenSim-Port//    &tcp_sig_checksigs, 0, "Verify RFC2385 digests on inbound traffic");
//ScenSim-Port//#endif

//ScenSim-Port//VNET_DEFINE(uma_zone_t, sack_hole_zone);
//ScenSim-Port//#define V_sack_hole_zone        VNET(sack_hole_zone)

//ScenSim-Port//VNET_DEFINE(struct hhook_head *, tcp_hhh[HHOOK_TCP_LAST+1]);

static struct inpcb *tcp_notify(struct inpcb *, int);
static char *   tcp_log_addr(struct in_conninfo *inc, struct tcphdr *th,
            void *ip4hdr, const void *ip6hdr);

/*
 * Target size of TCP PCB hash tables. Must be a power of two.
 *
 * Note that this can be overridden by the kernel environment
 * variable net.inet.tcp.tcbhashsize
 */
//ScenSim-Port//#ifndef TCBHASHSIZE
//ScenSim-Port//#define TCBHASHSIZE 512
//ScenSim-Port//#endif

/*
 * XXX
 * Callouts should be moved into struct tcp directly.  They are currently
 * separate because the tcpcb structure is exported to userland for sysctl
 * parsing purposes, which do not know about callouts.
 */
struct tcpcb_mem {
    struct  tcpcb       tcb;
    struct  tcp_timer   tt;
    struct  cc_var      ccv;
//ScenSim-Port//    struct  osd     osd;
};

//ScenSim-Port//static VNET_DEFINE(uma_zone_t, tcpcb_zone);
//ScenSim-Port//#define V_tcpcb_zone            VNET(tcpcb_zone)

//ScenSim-Port//MALLOC_DEFINE(M_TCPLOG, "tcplog", "TCP address and flags print buffers");
//ScenSim-Port//static struct mtx isn_mtx;

//ScenSim-Port//#define ISN_LOCK_INIT() mtx_init(&isn_mtx, "isn_mtx", NULL, MTX_DEF)
//ScenSim-Port//#define ISN_LOCK()  mtx_lock(&isn_mtx)
//ScenSim-Port//#define ISN_UNLOCK()    mtx_unlock(&isn_mtx)

/*
 * TCP initialization.
 */
//ScenSim-Port//static void
//ScenSim-Port//tcp_zone_change(void *tag)
//ScenSim-Port//{
//ScenSim-Port//
//ScenSim-Port//    uma_zone_set_max(V_tcbinfo.ipi_zone, maxsockets);
//ScenSim-Port//    uma_zone_set_max(V_tcpcb_zone, maxsockets);
//ScenSim-Port//    tcp_tw_zone_change();
//ScenSim-Port//}

//ScenSim-Port//static int
//ScenSim-Port//tcp_inpcb_init(void *mem, int size, int flags)
//ScenSim-Port//{
//ScenSim-Port//    struct inpcb *inp = mem;
//ScenSim-Port//
//ScenSim-Port//    INP_LOCK_INIT(inp, "inp", "tcpinp");
//ScenSim-Port//    return (0);
//ScenSim-Port//}

void
tcp_init(void)
{
//ScenSim-Port//    int hashsize;

//ScenSim-Port//    if (hhook_head_register(HHOOK_TYPE_TCP, HHOOK_TCP_EST_IN,
//ScenSim-Port//        &V_tcp_hhh[HHOOK_TCP_EST_IN], HHOOK_NOWAIT|HHOOK_HEADISINVNET) != 0)
//ScenSim-Port//        printf("%s: WARNING: unable to register helper hook\n", __func__);
//ScenSim-Port//    if (hhook_head_register(HHOOK_TYPE_TCP, HHOOK_TCP_EST_OUT,
//ScenSim-Port//        &V_tcp_hhh[HHOOK_TCP_EST_OUT], HHOOK_NOWAIT|HHOOK_HEADISINVNET) != 0)
//ScenSim-Port//        printf("%s: WARNING: unable to register helper hook\n", __func__);

//ScenSim-Port//    hashsize = TCBHASHSIZE;
//ScenSim-Port//    TUNABLE_INT_FETCH("net.inet.tcp.tcbhashsize", &hashsize);
//ScenSim-Port//    if (!powerof2(hashsize)) {
//ScenSim-Port//        printf("WARNING: TCB hash size not a power of 2\n");
//ScenSim-Port//        hashsize = 512; /* safe default */
//ScenSim-Port//    }
    in_pcbinfo_init(&V_tcbinfo, "tcp", &V_tcb, hashsize, hashsize,
        "tcp_inpcb", tcp_inpcb_init, NULL, UMA_ZONE_NOFREE,
        IPI_HASHFIELDS_4TUPLE);

    /*
     * These have to be type stable for the benefit of the timers.
     */
    V_tcpcb_zone = uma_zcreate("tcpcb", sizeof(struct tcpcb_mem),
        NULL, NULL, NULL, NULL, UMA_ALIGN_PTR, UMA_ZONE_NOFREE);
    uma_zone_set_max(V_tcpcb_zone, maxsockets);

    tcp_tw_init();
    syncache_init();
    tcp_hc_init();
    tcp_reass_init();
    ertt_mod_init();                                            //ScenSim-Port//
    if (V_default_cc_ptr->mod_init) {                           //ScenSim-Port//
        V_default_cc_ptr->mod_init();                           //ScenSim-Port//
    }                                                           //ScenSim-Port//
    callout_init_porting(&tcp_slowtimo_callout);                //ScenSim-Port//

//ScenSim-Port//    TUNABLE_INT_FETCH("net.inet.tcp.sack.enable", &V_tcp_do_sack);
    V_sack_hole_zone = uma_zcreate("sackhole", sizeof(struct sackhole),
        NULL, NULL, NULL, NULL, UMA_ALIGN_PTR, UMA_ZONE_NOFREE);

    /* Skip initialization of globals for non-default instances. */
//ScenSim-Port//    if (!IS_DEFAULT_VNET(curvnet))
//ScenSim-Port//        return;

    /* XXX virtualize those bellow? */
//ScenSim-Port//    tcp_delacktime = TCPTV_DELACK;
//ScenSim-Port//    tcp_keepinit = TCPTV_KEEP_INIT;
//ScenSim-Port//    tcp_keepidle = TCPTV_KEEP_IDLE;
//ScenSim-Port//    tcp_keepintvl = TCPTV_KEEPINTVL;
//ScenSim-Port//    tcp_maxpersistidle = TCPTV_KEEP_IDLE;
//ScenSim-Port//    tcp_msl = TCPTV_MSL;
//ScenSim-Port//    tcp_rexmit_min = TCPTV_MIN;
    if (tcp_rexmit_min < 1)
        tcp_rexmit_min = 1;
//ScenSim-Port//    tcp_rexmit_slop = TCPTV_CPU_VAR;
//ScenSim-Port//    tcp_finwait2_timeout = TCPTV_FINWAIT2_TIMEOUT;
//ScenSim-Port//    tcp_tcbhashsize = hashsize;

//ScenSim-Port//    TUNABLE_INT_FETCH("net.inet.tcp.soreceive_stream", &tcp_soreceive_stream);
//ScenSim-Port//    if (tcp_soreceive_stream) {
//ScenSim-Port//#ifdef INET
//ScenSim-Port//        tcp_usrreqs.pru_soreceive = soreceive_stream;
//ScenSim-Port//#endif
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//        tcp6_usrreqs.pru_soreceive = soreceive_stream;
//ScenSim-Port//#endif /* INET6 */
//ScenSim-Port//    }

#ifdef INET6
#define TCP_MINPROTOHDR (sizeof(struct ip6_hdr) + sizeof(struct tcphdr))
#else /* INET6 */
#define TCP_MINPROTOHDR (sizeof(struct tcpiphdr))
#endif /* INET6 */
//ScenSim-Port//    if (max_protohdr < TCP_MINPROTOHDR)
//ScenSim-Port//        max_protohdr = TCP_MINPROTOHDR;
//ScenSim-Port//    if (max_linkhdr + TCP_MINPROTOHDR > MHLEN)
    if (TCP_MINPROTOHDR > MHLEN)                                //ScenSim-Port//
        panic("tcp_init");
#undef TCP_MINPROTOHDR

//ScenSim-Port//    ISN_LOCK_INIT();
//ScenSim-Port//    EVENTHANDLER_REGISTER(shutdown_pre_sync, tcp_fini, NULL,
//ScenSim-Port//        SHUTDOWN_PRI_DEFAULT);
//ScenSim-Port//    EVENTHANDLER_REGISTER(maxsockets_change, tcp_zone_change, NULL,
//ScenSim-Port//        EVENTHANDLER_PRI_ANY);
}

//ScenSim-Port//#ifdef VIMAGE
void
tcp_destroy(void)
{

    in_pcbinfo_destroy(&V_tcbinfo);                             //ScenSim-Port//
    callout_stop(&tcp_slowtimo_callout);                        //ScenSim-Port//
    if (V_default_cc_ptr->mod_destroy) {                        //ScenSim-Port//
        V_default_cc_ptr->mod_destroy();                        //ScenSim-Port//
    }                                                           //ScenSim-Port//
    ertt_mod_destroy();                                         //ScenSim-Port//
    tcp_reass_destroy();
    tcp_hc_destroy();
    syncache_destroy();
    tcp_tw_destroy();
//ScenSim-Port//    in_pcbinfo_destroy(&V_tcbinfo);
    uma_zdestroy(V_sack_hole_zone);
    uma_zdestroy(V_tcpcb_zone);
}
//ScenSim-Port//#endif

//ScenSim-Port//void
//ScenSim-Port//tcp_fini(void *xtp)
//ScenSim-Port//{
//ScenSim-Port//
//ScenSim-Port//}

/*
 * Fill in the IP and TCP headers for an outgoing packet, given the tcpcb.
 * tcp_template used to store this data in mbufs, but we now recopy it out
 * of the tcpcb each time to conserve mbufs.
 */
void
tcpip_fillheaders(struct inpcb *inp, void *ip_ptr, void *tcp_ptr)
{
    struct tcphdr *th = (struct tcphdr *)tcp_ptr;

//ScenSim-Port//    INP_WLOCK_ASSERT(inp);

#ifdef INET6
    if ((inp->inp_vflag & INP_IPV6) != 0) {
        struct ip6_hdr *ip6;

        ip6 = (struct ip6_hdr *)ip_ptr;
        ip6->ip6_flow = (ip6->ip6_flow & ~IPV6_FLOWINFO_MASK) |
            (inp->inp_flow & IPV6_FLOWINFO_MASK);
        ip6->ip6_vfc = (ip6->ip6_vfc & ~IPV6_VERSION_MASK) |
            (IPV6_VERSION & IPV6_VERSION_MASK);
        ip6->ip6_nxt = IPPROTO_TCP;
        ip6->ip6_plen = htons(sizeof(struct tcphdr));
        ip6->ip6_src = inp->in6p_laddr;
        ip6->ip6_dst = inp->in6p_faddr;
    }
#endif /* INET6 */
#if defined(INET6) && defined(INET)
    else
#endif
#ifdef INET
    {
        struct ip *ip;

        ip = (struct ip *)ip_ptr;
        ip->ip_v = IPVERSION;
        ip->ip_hl = 5;
        ip->ip_tos = inp->inp_ip_tos;
        ip->ip_len = 0;
        ip->ip_id = 0;
        ip->ip_off = 0;
        ip->ip_ttl = inp->inp_ip_ttl;
        ip->ip_sum = 0;
        ip->ip_p = IPPROTO_TCP;
        ip->ip_src = inp->inp_laddr;
        ip->ip_dst = inp->inp_faddr;
    }
#endif /* INET */
    th->th_sport = inp->inp_lport;
    th->th_dport = inp->inp_fport;
    th->th_seq = 0;
    th->th_ack = 0;
    th->th_x2 = 0;
    th->th_off = 5;
    th->th_flags = 0;
    th->th_win = 0;
    th->th_urp = 0;
    th->th_sum = 0;     /* in_pseudo() is called later for ipv4 */
}

/*
 * Create template to be used to send tcp packets on a connection.
 * Allocates an mbuf and fills in a skeletal tcp/ip header.  The only
 * use for this function is in keepalives, which use tcp_respond.
 */
struct tcptemp *
tcpip_maketemplate(struct inpcb *inp)
{
    struct tcptemp *t;

//ScenSim-Port//    t = malloc(sizeof(*t), M_TEMP, M_NOWAIT);
    t = (struct tcptemp *)malloc(sizeof(*t), M_TEMP, M_NOWAIT); //ScenSim-Port//
    if (t == NULL)
        return (NULL);
    tcpip_fillheaders(inp, (void *)&t->tt_ipgen, (void *)&t->tt_t);
    return (t);
}

/*
 * Send a single message to the TCP at address specified by
 * the given TCP/IP header.  If m == NULL, then we make a copy
 * of the tcpiphdr at ti and send directly to the addressed host.
 * This is used to force keep alive messages out using the TCP
 * template for a connection.  If flags are given then we send
 * a message back to the TCP which originated the * segment ti,
 * and discard the mbuf containing it and any other attached mbufs.
 *
 * In any case the ack and sequence number of the transmitted
 * segment are as specified by the parameters.
 *
 * NOTE: If m != NULL, then ti must point to *inside* the mbuf.
 */
void
tcp_respond(struct tcpcb *tp, void *ipgen, struct tcphdr *th, struct mbuf *m,
    tcp_seq ack, tcp_seq seq, int flags)
{
    int tlen;
    int win = 0;
    struct ip *ip;
    struct tcphdr *nth;
#ifdef INET6
    struct ip6_hdr *ip6;
    int isipv6;
#endif /* INET6 */
    int ipflags = 0;
    struct inpcb *inp;

//ScenSim-Port//    KASSERT(tp != NULL || m != NULL, ("tcp_respond: tp and m both NULL"));

#ifdef INET6
    isipv6 = ((struct ip *)ipgen)->ip_v == (IPV6_VERSION >> 4);
//ScenSim-Port//    ip6 = ipgen;
    ip6 = (struct ip6_hdr *)ipgen;                              //ScenSim-Port//
#endif /* INET6 */
//ScenSim-Port//    ip = ipgen;
    ip = (struct ip *)ipgen;                                    //ScenSim-Port//

    if (tp != NULL) {
        inp = tp->t_inpcb;
//ScenSim-Port//        KASSERT(inp != NULL, ("tcp control block w/o inpcb"));
//ScenSim-Port//        INP_WLOCK_ASSERT(inp);
    } else
        inp = NULL;

    if (tp != NULL) {
        if (!(flags & TH_RST)) {
            win = sbspace(&inp->inp_socket->so_rcv);
            if (win > (long)TCP_MAXWIN << tp->rcv_scale)
                win = (long)TCP_MAXWIN << tp->rcv_scale;
        }
    }
    if (m == NULL) {
        m = m_gethdr(M_DONTWAIT, MT_DATA);
        if (m == NULL)
            return;
        tlen = 0;
//ScenSim-Port//        m->m_data += max_linkhdr;
#ifdef INET6
        if (isipv6) {
            bcopy((caddr_t)ip6, mtod(m, caddr_t),
                  sizeof(struct ip6_hdr));
            ip6 = mtod(m, struct ip6_hdr *);
            nth = (struct tcphdr *)(ip6 + 1);
        } else
#endif /* INET6 */
          {
        bcopy((caddr_t)ip, mtod(m, caddr_t), sizeof(struct ip));
        ip = mtod(m, struct ip *);
        nth = (struct tcphdr *)(ip + 1);
          }
        bcopy((caddr_t)th, (caddr_t)nth, sizeof(struct tcphdr));
        flags = TH_ACK;
    } else {
        /*
         *  reuse the mbuf.
         * XXX MRT We inherrit the FIB, which is lucky.
         */
        m_freem(m->m_next);
        m->m_next = NULL;
        m->m_data = (caddr_t)ipgen;
//ScenSim-Port//        m_addr_changed(m);
        /* m_len is set later */
        tlen = 0;
#define xchg(a,b,type) { type t; t=a; a=b; b=t; }
#ifdef INET6
        if (isipv6) {
            xchg(ip6->ip6_dst, ip6->ip6_src, struct in6_addr);
            nth = (struct tcphdr *)(ip6 + 1);
        } else
#endif /* INET6 */
          {
        xchg(ip->ip_dst.s_addr, ip->ip_src.s_addr, uint32_t);
        nth = (struct tcphdr *)(ip + 1);
          }
        if (th != nth) {
            /*
             * this is usually a case when an extension header
             * exists between the IPv6 header and the
             * TCP header.
             */
            nth->th_sport = th->th_sport;
            nth->th_dport = th->th_dport;
        }
        xchg(nth->th_dport, nth->th_sport, uint16_t);
#undef xchg
    }
#ifdef INET6
    if (isipv6) {
        ip6->ip6_flow = 0;
        ip6->ip6_vfc = IPV6_VERSION;
        ip6->ip6_nxt = IPPROTO_TCP;
        ip6->ip6_plen = htons((u_short)(sizeof (struct tcphdr) +
                        tlen));
        tlen += sizeof (struct ip6_hdr) + sizeof (struct tcphdr);
    }
#endif
#if defined(INET) && defined(INET6)
    else
#endif
#ifdef INET
    {
        tlen += sizeof (struct tcpiphdr);
        ip->ip_len = tlen;
        ip->ip_ttl = V_ip_defttl;
        if (V_path_mtu_discovery)
            ip->ip_off |= IP_DF;
    }
#endif
    m->m_len = tlen;
    m->m_pkthdr.len = tlen;
//ScenSim-Port//    m->m_pkthdr.rcvif = NULL;
//ScenSim-Port//#ifdef MAC
//ScenSim-Port//    if (inp != NULL) {
//ScenSim-Port//        /*
//ScenSim-Port//         * Packet is associated with a socket, so allow the
//ScenSim-Port//         * label of the response to reflect the socket label.
//ScenSim-Port//         */
//ScenSim-Port//        INP_WLOCK_ASSERT(inp);
//ScenSim-Port//        mac_inpcb_create_mbuf(inp, m);
//ScenSim-Port//    } else {
//ScenSim-Port//        /*
//ScenSim-Port//         * Packet is not associated with a socket, so possibly
//ScenSim-Port//         * update the label in place.
//ScenSim-Port//         */
//ScenSim-Port//        mac_netinet_tcp_reply(m);
//ScenSim-Port//    }
//ScenSim-Port//#endif
    nth->th_seq = htonl(seq);
    nth->th_ack = htonl(ack);
    nth->th_x2 = 0;
    nth->th_off = sizeof (struct tcphdr) >> 2;
    nth->th_flags = flags;
    if (tp != NULL)
        nth->th_win = htons((u_short) (win >> tp->rcv_scale));
    else
        nth->th_win = htons((u_short)win);
    nth->th_urp = 0;
#ifdef INET6
    if (isipv6) {
        nth->th_sum = 0;
        nth->th_sum = in6_cksum(m, IPPROTO_TCP,
                    sizeof(struct ip6_hdr),
                    tlen - sizeof(struct ip6_hdr));
        ip6->ip6_hlim = in6_selecthlim(tp != NULL ? tp->t_inpcb :
            NULL, NULL);
    }
#endif /* INET6 */
#if defined(INET6) && defined(INET)
    else
#endif
#ifdef INET
    {
        nth->th_sum = in_pseudo(ip->ip_src.s_addr, ip->ip_dst.s_addr,
            htons((u_short)(tlen - sizeof(struct ip) + ip->ip_p)));
        m->m_pkthdr.csum_flags = CSUM_TCP;
        m->m_pkthdr.csum_data = offsetof(struct tcphdr, th_sum);
    }
#endif /* INET */
//ScenSim-Port//#ifdef TCPDEBUG
//ScenSim-Port//    if (tp == NULL || (inp->inp_socket->so_options & SO_DEBUG))
//ScenSim-Port//        tcp_trace(TA_OUTPUT, 0, tp, mtod(m, void *), th, 0);
//ScenSim-Port//#endif
#ifdef INET6
    if (isipv6)
//ScenSim-Port//        (void) ip6_output(m, NULL, NULL, ipflags, NULL, NULL, inp);
    (void)ip_output(m, NULL, NULL, ipflags, NULL, inp, 0, 0);//ScenSim-Port//
#endif /* INET6 */
#if defined(INET) && defined(INET6)
    else
#endif
#ifdef INET
//ScenSim-Port//        (void) ip_output(m, NULL, NULL, ipflags, NULL, inp);
    (void)ip_output(m, NULL, NULL, ipflags, NULL, inp, 0, 0);//ScenSim-Port//
#endif
}

/*
 * Create a new TCP control block, making an
 * empty reassembly queue and hooking it to the argument
 * protocol control block.  The `inp' parameter must have
 * come from the zone allocator set up in tcp_init().
 */
struct tcpcb *
tcp_newtcpcb(struct inpcb *inp)
{
    struct tcpcb_mem *tm;
    struct tcpcb *tp;
#ifdef INET6
    int isipv6 = (inp->inp_vflag & INP_IPV6) != 0;
#endif /* INET6 */

//ScenSim-Port//    tm = uma_zalloc(V_tcpcb_zone, M_NOWAIT | M_ZERO);
    tm = (struct tcpcb_mem *)uma_zalloc(                        //ScenSim-Port//
        V_tcpcb_zone, M_NOWAIT | M_ZERO);                       //ScenSim-Port//
    if (tm == NULL)
        return (NULL);
    tp = &tm->tcb;

    /* Initialise cc_var struct for this tcpcb. */
    tp->ccv = &tm->ccv;
    tp->ccv->type = IPPROTO_TCP;
    tp->ccv->ccvc.tcp = tp;
    ertt_uma_ctor(&tp->ccv->ertt_porting);                      //ScenSim-Port//

    /*
     * Use the current system default CC algorithm.
     */
//ScenSim-Port//    CC_LIST_RLOCK();
//ScenSim-Port//    KASSERT(!STAILQ_EMPTY(&cc_list), ("cc_list is empty!"));
    CC_ALGO(tp) = CC_DEFAULT();
//ScenSim-Port//    CC_LIST_RUNLOCK();

    if (CC_ALGO(tp)->cb_init != NULL)
        if (CC_ALGO(tp)->cb_init(tp->ccv) > 0) {
            uma_zfree(V_tcpcb_zone, tm);
            return (NULL);
        }

//ScenSim-Port//    tp->osd = &tm->osd;
//ScenSim-Port//    if (khelp_init_osd(HELPER_CLASS_TCP, tp->osd)) {
//ScenSim-Port//        uma_zfree(V_tcpcb_zone, tm);
//ScenSim-Port//        return (NULL);
//ScenSim-Port//    }

//ScenSim-Port//#ifdef VIMAGE
    tp->t_vnet = inp->inp_vnet;
//ScenSim-Port//#endif
    tp->t_timers = &tm->tt;
    /*  LIST_INIT(&tp->t_segq); */  /* XXX covered by M_ZERO */
    tp->t_maxseg = tp->t_maxopd =
#ifdef INET6
        isipv6 ? V_tcp_v6mssdflt :
#endif /* INET6 */
        V_tcp_mssdflt;

    /* Set up our timeouts. */
    callout_init(&tp->t_timers->tt_rexmt, CALLOUT_MPSAFE);
    callout_init(&tp->t_timers->tt_persist, CALLOUT_MPSAFE);
    callout_init(&tp->t_timers->tt_keep, CALLOUT_MPSAFE);
    callout_init(&tp->t_timers->tt_2msl, CALLOUT_MPSAFE);
    callout_init(&tp->t_timers->tt_delack, CALLOUT_MPSAFE);

    if (V_tcp_do_rfc1323)
        tp->t_flags = (TF_REQ_SCALE|TF_REQ_TSTMP);
    if (V_tcp_do_sack)
        tp->t_flags |= TF_SACK_PERMIT;
    TAILQ_INIT(&tp->snd_holes);
    tp->t_inpcb = inp;  /* XXX */
    /*
     * Init srtt to TCPTV_SRTTBASE (0), so we can tell that we have no
     * rtt estimate.  Set rttvar so that srtt + 4 * rttvar gives
     * reasonable initial retransmit time.
     */
    tp->t_srtt = TCPTV_SRTTBASE;
    tp->t_rttvar = ((TCPTV_RTOBASE - TCPTV_SRTTBASE) << TCP_RTTVAR_SHIFT) / 4;
    tp->t_rttmin = tcp_rexmit_min;
    tp->t_rxtcur = TCPTV_RTOBASE;
    tp->snd_cwnd = TCP_MAXWIN << TCP_MAX_WINSHIFT;
    tp->snd_ssthresh = TCP_MAXWIN << TCP_MAX_WINSHIFT;
    tp->t_rcvtime = ticks;
    /*
     * IPv4 TTL initialization is necessary for an IPv6 socket as well,
     * because the socket may be bound to an IPv6 wildcard address,
     * which may match an IPv4-mapped IPv6 address.
     */
    inp->inp_ip_ttl = V_ip_defttl;
    inp->inp_ppcb = tp;
    return (tp);        /* XXX */
}

/*
 * Switch the congestion control algorithm back to NewReno for any active
 * control blocks using an algorithm which is about to go away.
 * This ensures the CC framework can allow the unload to proceed without leaving
 * any dangling pointers which would trigger a panic.
 * Returning non-zero would inform the CC framework that something went wrong
 * and it would be unsafe to allow the unload to proceed. However, there is no
 * way for this to occur with this implementation so we always return zero.
 */
//ScenSim-Port//int
//ScenSim-Port//tcp_ccalgounload(struct cc_algo *unload_algo)
//ScenSim-Port//{
//ScenSim-Port//    struct cc_algo *tmpalgo;
//ScenSim-Port//    struct inpcb *inp;
//ScenSim-Port//    struct tcpcb *tp;
//ScenSim-Port//    VNET_ITERATOR_DECL(vnet_iter);
//ScenSim-Port//
//ScenSim-Port//    /*
//ScenSim-Port//     * Check all active control blocks across all network stacks and change
//ScenSim-Port//     * any that are using "unload_algo" back to NewReno. If "unload_algo"
//ScenSim-Port//     * requires cleanup code to be run, call it.
//ScenSim-Port//     */
//ScenSim-Port//    VNET_LIST_RLOCK();
//ScenSim-Port//    VNET_FOREACH(vnet_iter) {
//ScenSim-Port//        CURVNET_SET(vnet_iter);
//ScenSim-Port//        INP_INFO_RLOCK(&V_tcbinfo);
//ScenSim-Port//        /*
//ScenSim-Port//         * New connections already part way through being initialised
//ScenSim-Port//         * with the CC algo we're removing will not race with this code
//ScenSim-Port//         * because the INP_INFO_WLOCK is held during initialisation. We
//ScenSim-Port//         * therefore don't enter the loop below until the connection
//ScenSim-Port//         * list has stabilised.
//ScenSim-Port//         */
//ScenSim-Port//        LIST_FOREACH(inp, &V_tcb, inp_list) {
//ScenSim-Port//            INP_WLOCK(inp);
//ScenSim-Port//            /* Important to skip tcptw structs. */
//ScenSim-Port//            if (!(inp->inp_flags & INP_TIMEWAIT) &&
//ScenSim-Port//                (tp = intotcpcb(inp)) != NULL) {
//ScenSim-Port//                /*
//ScenSim-Port//                 * By holding INP_WLOCK here, we are assured
//ScenSim-Port//                 * that the connection is not currently
//ScenSim-Port//                 * executing inside the CC module's functions
//ScenSim-Port//                 * i.e. it is safe to make the switch back to
//ScenSim-Port//                 * NewReno.
//ScenSim-Port//                 */
//ScenSim-Port//                if (CC_ALGO(tp) == unload_algo) {
//ScenSim-Port//                    tmpalgo = CC_ALGO(tp);
//ScenSim-Port//                    /* NewReno does not require any init. */
//ScenSim-Port//                    CC_ALGO(tp) = &newreno_cc_algo;
//ScenSim-Port//                    if (tmpalgo->cb_destroy != NULL)
//ScenSim-Port//                        tmpalgo->cb_destroy(tp->ccv);
//ScenSim-Port//                }
//ScenSim-Port//            }
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//        }
//ScenSim-Port//        INP_INFO_RUNLOCK(&V_tcbinfo);
//ScenSim-Port//        CURVNET_RESTORE();
//ScenSim-Port//    }
//ScenSim-Port//    VNET_LIST_RUNLOCK();
//ScenSim-Port//
//ScenSim-Port//    return (0);
//ScenSim-Port//}

/*
 * Drop a TCP connection, reporting
 * the specified error.  If connection is synchronized,
 * then send a RST to peer.
 */
struct tcpcb *
tcp_drop(struct tcpcb *tp, int errnoVal)
{
    struct socket *so = tp->t_inpcb->inp_socket;

//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK_ASSERT(tp->t_inpcb);

    if (TCPS_HAVERCVDSYN(tp->t_state)) {
        tp->t_state = TCPS_CLOSED;
        (void) tcp_output_reset(tp);
        TCPSTAT_INC(tcps_drops);
    } else
        TCPSTAT_INC(tcps_conndrops);
    if (errnoVal == ETIMEDOUT && tp->t_softerror)
        errnoVal = tp->t_softerror;
    so->so_error = errnoVal;
    return (tcp_close(tp));
}

void
tcp_discardcb(struct tcpcb *tp)
{
    struct inpcb *inp = tp->t_inpcb;
    struct socket *so = inp->inp_socket;
#ifdef INET6
    int isipv6 = (inp->inp_vflag & INP_IPV6) != 0;
#endif /* INET6 */

//ScenSim-Port//    INP_WLOCK_ASSERT(inp);

    /*
     * Make sure that all of our timers are stopped before we delete the
     * PCB.
     *
     * XXXRW: Really, we would like to use callout_drain() here in order
     * to avoid races experienced in tcp_timer.c where a timer is already
     * executing at this point.  However, we can't, both because we're
     * running in a context where we can't sleep, and also because we
     * hold locks required by the timers.  What we instead need to do is
     * test to see if callout_drain() is required, and if so, defer some
     * portion of the remainder of tcp_discardcb() to an asynchronous
     * context that can callout_drain() and then continue.  Some care
     * will be required to ensure that no further processing takes place
     * on the tcpcb, even though it hasn't been freed (a flag?).
     */
    callout_stop(&tp->t_timers->tt_rexmt);
    callout_stop(&tp->t_timers->tt_persist);
    callout_stop(&tp->t_timers->tt_keep);
    callout_stop(&tp->t_timers->tt_2msl);
    callout_stop(&tp->t_timers->tt_delack);

    /*
     * If we got enough samples through the srtt filter,
     * save the rtt and rttvar in the routing entry.
     * 'Enough' is arbitrarily defined as 4 rtt samples.
     * 4 samples is enough for the srtt filter to converge
     * to within enough % of the correct value; fewer samples
     * and we could save a bogus rtt. The danger is not high
     * as tcp quickly recovers from everything.
     * XXX: Works very well but needs some more statistics!
     */
    if (tp->t_rttupdated >= 4) {
        struct hc_metrics_lite metrics;
        u_long ssthresh;

        bzero(&metrics, sizeof(metrics));
        /*
         * Update the ssthresh always when the conditions below
         * are satisfied. This gives us better new start value
         * for the congestion avoidance for new connections.
         * ssthresh is only set if packet loss occured on a session.
         *
         * XXXRW: 'so' may be NULL here, and/or socket buffer may be
         * being torn down.  Ideally this code would not use 'so'.
         */
        ssthresh = tp->snd_ssthresh;
        if (ssthresh != 0 && ssthresh < so->so_snd.sb_hiwat / 2) {
            /*
             * convert the limit from user data bytes to
             * packets then to packet data bytes.
             */
            ssthresh = (ssthresh + tp->t_maxseg / 2) / tp->t_maxseg;
            if (ssthresh < 2)
                ssthresh = 2;
            ssthresh *= (u_long)(tp->t_maxseg +
#ifdef INET6
                      (isipv6 ? sizeof (struct ip6_hdr) +
                           sizeof (struct tcphdr) :
#endif
                       sizeof (struct tcpiphdr)
#ifdef INET6
                       )
#endif
                      );
        } else
            ssthresh = 0;
        metrics.rmx_ssthresh = ssthresh;

        metrics.rmx_rtt = tp->t_srtt;
        metrics.rmx_rttvar = tp->t_rttvar;
        metrics.rmx_cwnd = tp->snd_cwnd;
        metrics.rmx_sendpipe = 0;
        metrics.rmx_recvpipe = 0;

        tcp_hc_update(&inp->inp_inc, &metrics);
    }

    /* free the reassembly queue, if any */
    tcp_reass_flush(tp);
    /* Disconnect offload device, if any. */
//ScenSim-Port//    tcp_offload_detach(tp);

    tcp_free_sackholes(tp);

    /* Allow the CC algorithm to clean up after itself. */
    if (CC_ALGO(tp)->cb_destroy != NULL)
        CC_ALGO(tp)->cb_destroy(tp->ccv);
    ertt_uma_dtor(&tp->ccv->ertt_porting);                      //ScenSim-Port//

//ScenSim-Port//    khelp_destroy_osd(tp->osd);

    CC_ALGO(tp) = NULL;
    inp->inp_ppcb = NULL;
    tp->t_inpcb = NULL;
    uma_zfree(V_tcpcb_zone, tp);
}

/*
 * Attempt to close a TCP control block, marking it as dropped, and freeing
 * the socket if we hold the only reference.
 */
struct tcpcb *
tcp_close(struct tcpcb *tp)
{
    struct inpcb *inp = tp->t_inpcb;
    struct socket *so;

//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK_ASSERT(inp);

    /* Notify any offload devices of listener close */
//ScenSim-Port//    if (tp->t_state == TCPS_LISTEN)
//ScenSim-Port//        tcp_offload_listen_close(tp);
    in_pcbdrop(inp);
    TCPSTAT_INC(tcps_closed);
//ScenSim-Port//    KASSERT(inp->inp_socket != NULL, ("tcp_close: inp_socket NULL"));
    so = inp->inp_socket;
    soisdisconnected(so);
    if (inp->inp_flags & INP_SOCKREF) {
//ScenSim-Port//        KASSERT(so->so_state & SS_PROTOREF,
//ScenSim-Port//            ("tcp_close: !SS_PROTOREF"));
        inp->inp_flags &= ~INP_SOCKREF;
//ScenSim-Port//        INP_WUNLOCK(inp);
//ScenSim-Port//        ACCEPT_LOCK();
//ScenSim-Port//        SOCK_LOCK(so);
        so->so_state &= ~SS_PROTOREF;
        sofree(so);
        return (NULL);
    }
    return (tp);
}

//ScenSim-Port//void
//ScenSim-Port//tcp_drain(void)
//ScenSim-Port//{
//ScenSim-Port//    VNET_ITERATOR_DECL(vnet_iter);
//ScenSim-Port//
//ScenSim-Port//    if (!do_tcpdrain)
//ScenSim-Port//        return;
//ScenSim-Port//
//ScenSim-Port//    VNET_LIST_RLOCK_NOSLEEP();
//ScenSim-Port//    VNET_FOREACH(vnet_iter) {
//ScenSim-Port//        CURVNET_SET(vnet_iter);
//ScenSim-Port//        struct inpcb *inpb;
//ScenSim-Port//        struct tcpcb *tcpb;
//ScenSim-Port//
//ScenSim-Port//    /*
//ScenSim-Port//     * Walk the tcpbs, if existing, and flush the reassembly queue,
//ScenSim-Port//     * if there is one...
//ScenSim-Port//     * XXX: The "Net/3" implementation doesn't imply that the TCP
//ScenSim-Port//     *      reassembly queue should be flushed, but in a situation
//ScenSim-Port//     *  where we're really low on mbufs, this is potentially
//ScenSim-Port//     *  usefull.
//ScenSim-Port//     */
//ScenSim-Port//        INP_INFO_RLOCK(&V_tcbinfo);
//ScenSim-Port//        LIST_FOREACH(inpb, V_tcbinfo.ipi_listhead, inp_list) {
//ScenSim-Port//            if (inpb->inp_flags & INP_TIMEWAIT)
//ScenSim-Port//                continue;
//ScenSim-Port//            INP_WLOCK(inpb);
//ScenSim-Port//            if ((tcpb = intotcpcb(inpb)) != NULL) {
//ScenSim-Port//                tcp_reass_flush(tcpb);
//ScenSim-Port//                tcp_clean_sackreport(tcpb);
//ScenSim-Port//            }
//ScenSim-Port//            INP_WUNLOCK(inpb);
//ScenSim-Port//        }
//ScenSim-Port//        INP_INFO_RUNLOCK(&V_tcbinfo);
//ScenSim-Port//        CURVNET_RESTORE();
//ScenSim-Port//    }
//ScenSim-Port//    VNET_LIST_RUNLOCK_NOSLEEP();
//ScenSim-Port//}

/*
 * Notify a tcp user of an asynchronous error;
 * store error as soft error, but wake up user
 * (for now, won't do anything until can select for soft error).
 *
 * Do not wake up user since there currently is no mechanism for
 * reporting soft errors (yet - a kqueue filter may be added).
 */
static struct inpcb *
tcp_notify(struct inpcb *inp, int error)
{
    struct tcpcb *tp;

//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK_ASSERT(inp);

    if ((inp->inp_flags & INP_TIMEWAIT) ||
        (inp->inp_flags & INP_DROPPED))
        return (inp);

    tp = intotcpcb(inp);
//ScenSim-Port//    KASSERT(tp != NULL, ("tcp_notify: tp == NULL"));

    /*
     * Ignore some errors if we are hooked up.
     * If connection hasn't completed, has retransmitted several times,
     * and receives a second error, give up now.  This is better
     * than waiting a long time to establish a connection that
     * can never complete.
     */
    if (tp->t_state == TCPS_ESTABLISHED &&
        (error == EHOSTUNREACH || error == ENETUNREACH ||
         error == EHOSTDOWN)) {
        return (inp);
    } else if (tp->t_state < TCPS_ESTABLISHED && tp->t_rxtshift > 3 &&
        tp->t_softerror) {
        tp = tcp_drop(tp, error);
        if (tp != NULL)
            return (inp);
        else
            return (NULL);
    } else {
        tp->t_softerror = error;
        return (inp);
    }
//ScenSim-Port//#if 0
//ScenSim-Port//    wakeup( &so->so_timeo);
//ScenSim-Port//    sorwakeup(so);
//ScenSim-Port//    sowwakeup(so);
//ScenSim-Port//#endif
}

//ScenSim-Port//static int
//ScenSim-Port//tcp_pcblist(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    int error, i, m, n, pcb_count;
//ScenSim-Port//    struct inpcb *inp, **inp_list;
//ScenSim-Port//    inp_gen_t gencnt;
//ScenSim-Port//    struct xinpgen xig;
//ScenSim-Port//
//ScenSim-Port//    /*
//ScenSim-Port//     * The process of preparing the TCB list is too time-consuming and
//ScenSim-Port//     * resource-intensive to repeat twice on every request.
//ScenSim-Port//     */
//ScenSim-Port//    if (req->oldptr == NULL) {
//ScenSim-Port//        n = V_tcbinfo.ipi_count + syncache_pcbcount();
//ScenSim-Port//        n += imax(n / 8, 10);
//ScenSim-Port//        req->oldidx = 2 * (sizeof xig) + n * sizeof(struct xtcpcb);
//ScenSim-Port//        return (0);
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//    if (req->newptr != NULL)
//ScenSim-Port//        return (EPERM);
//ScenSim-Port//
//ScenSim-Port//    /*
//ScenSim-Port//     * OK, now we're committed to doing something.
//ScenSim-Port//     */
//ScenSim-Port//    INP_INFO_RLOCK(&V_tcbinfo);
//ScenSim-Port//    gencnt = V_tcbinfo.ipi_gencnt;
//ScenSim-Port//    n = V_tcbinfo.ipi_count;
//ScenSim-Port//    INP_INFO_RUNLOCK(&V_tcbinfo);
//ScenSim-Port//
//ScenSim-Port//    m = syncache_pcbcount();
//ScenSim-Port//
//ScenSim-Port//    error = sysctl_wire_old_buffer(req, 2 * (sizeof xig)
//ScenSim-Port//        + (n + m) * sizeof(struct xtcpcb));
//ScenSim-Port//    if (error != 0)
//ScenSim-Port//        return (error);
//ScenSim-Port//
//ScenSim-Port//    xig.xig_len = sizeof xig;
//ScenSim-Port//    xig.xig_count = n + m;
//ScenSim-Port//    xig.xig_gen = gencnt;
//ScenSim-Port//    xig.xig_sogen = so_gencnt;
//ScenSim-Port//    error = SYSCTL_OUT(req, &xig, sizeof xig);
//ScenSim-Port//    if (error)
//ScenSim-Port//        return (error);
//ScenSim-Port//
//ScenSim-Port//    error = syncache_pcblist(req, m, &pcb_count);
//ScenSim-Port//    if (error)
//ScenSim-Port//        return (error);
//ScenSim-Port//
//ScenSim-Port//    inp_list = malloc(n * sizeof *inp_list, M_TEMP, M_WAITOK);
//ScenSim-Port//    if (inp_list == NULL)
//ScenSim-Port//        return (ENOMEM);
//ScenSim-Port//
//ScenSim-Port//    INP_INFO_RLOCK(&V_tcbinfo);
//ScenSim-Port//    for (inp = LIST_FIRST(V_tcbinfo.ipi_listhead), i = 0;
//ScenSim-Port//        inp != NULL && i < n; inp = LIST_NEXT(inp, inp_list)) {
//ScenSim-Port//        INP_WLOCK(inp);
//ScenSim-Port//        if (inp->inp_gencnt <= gencnt) {
//ScenSim-Port//            /*
//ScenSim-Port//             * XXX: This use of cr_cansee(), introduced with
//ScenSim-Port//             * TCP state changes, is not quite right, but for
//ScenSim-Port//             * now, better than nothing.
//ScenSim-Port//             */
//ScenSim-Port//            if (inp->inp_flags & INP_TIMEWAIT) {
//ScenSim-Port//                if (intotw(inp) != NULL)
//ScenSim-Port//                    error = cr_cansee(req->td->td_ucred,
//ScenSim-Port//                        intotw(inp)->tw_cred);
//ScenSim-Port//                else
//ScenSim-Port//                    error = EINVAL; /* Skip this inp. */
//ScenSim-Port//            } else
//ScenSim-Port//                error = cr_canseeinpcb(req->td->td_ucred, inp);
//ScenSim-Port//            if (error == 0) {
//ScenSim-Port//                in_pcbref(inp);
//ScenSim-Port//                inp_list[i++] = inp;
//ScenSim-Port//            }
//ScenSim-Port//        }
//ScenSim-Port//        INP_WUNLOCK(inp);
//ScenSim-Port//    }
//ScenSim-Port//    INP_INFO_RUNLOCK(&V_tcbinfo);
//ScenSim-Port//    n = i;
//ScenSim-Port//
//ScenSim-Port//    error = 0;
//ScenSim-Port//    for (i = 0; i < n; i++) {
//ScenSim-Port//        inp = inp_list[i];
//ScenSim-Port//        INP_RLOCK(inp);
//ScenSim-Port//        if (inp->inp_gencnt <= gencnt) {
//ScenSim-Port//            struct xtcpcb xt;
//ScenSim-Port//            void *inp_ppcb;
//ScenSim-Port//
//ScenSim-Port//            bzero(&xt, sizeof(xt));
//ScenSim-Port//            xt.xt_len = sizeof xt;
//ScenSim-Port//            /* XXX should avoid extra copy */
//ScenSim-Port//            bcopy(inp, &xt.xt_inp, sizeof *inp);
//ScenSim-Port//            inp_ppcb = inp->inp_ppcb;
//ScenSim-Port//            if (inp_ppcb == NULL)
//ScenSim-Port//                bzero((char *) &xt.xt_tp, sizeof xt.xt_tp);
//ScenSim-Port//            else if (inp->inp_flags & INP_TIMEWAIT) {
//ScenSim-Port//                bzero((char *) &xt.xt_tp, sizeof xt.xt_tp);
//ScenSim-Port//                xt.xt_tp.t_state = TCPS_TIME_WAIT;
//ScenSim-Port//            } else {
//ScenSim-Port//                bcopy(inp_ppcb, &xt.xt_tp, sizeof xt.xt_tp);
//ScenSim-Port//                if (xt.xt_tp.t_timers)
//ScenSim-Port//                    tcp_timer_to_xtimer(&xt.xt_tp, xt.xt_tp.t_timers, &xt.xt_timer);
//ScenSim-Port//            }
//ScenSim-Port//            if (inp->inp_socket != NULL)
//ScenSim-Port//                sotoxsocket(inp->inp_socket, &xt.xt_socket);
//ScenSim-Port//            else {
//ScenSim-Port//                bzero(&xt.xt_socket, sizeof xt.xt_socket);
//ScenSim-Port//                xt.xt_socket.xso_protocol = IPPROTO_TCP;
//ScenSim-Port//            }
//ScenSim-Port//            xt.xt_inp.inp_gencnt = inp->inp_gencnt;
//ScenSim-Port//            INP_RUNLOCK(inp);
//ScenSim-Port//            error = SYSCTL_OUT(req, &xt, sizeof xt);
//ScenSim-Port//        } else
//ScenSim-Port//            INP_RUNLOCK(inp);
//ScenSim-Port//    }
//ScenSim-Port//    INP_INFO_WLOCK(&V_tcbinfo);
//ScenSim-Port//    for (i = 0; i < n; i++) {
//ScenSim-Port//        inp = inp_list[i];
//ScenSim-Port//        INP_RLOCK(inp);
//ScenSim-Port//        if (!in_pcbrele_rlocked(inp))
//ScenSim-Port//            INP_RUNLOCK(inp);
//ScenSim-Port//    }
//ScenSim-Port//    INP_INFO_WUNLOCK(&V_tcbinfo);
//ScenSim-Port//
//ScenSim-Port//    if (!error) {
//ScenSim-Port//        /*
//ScenSim-Port//         * Give the user an updated idea of our state.
//ScenSim-Port//         * If the generation differs from what we told
//ScenSim-Port//         * her before, she knows that something happened
//ScenSim-Port//         * while we were processing this request, and it
//ScenSim-Port//         * might be necessary to retry.
//ScenSim-Port//         */
//ScenSim-Port//        INP_INFO_RLOCK(&V_tcbinfo);
//ScenSim-Port//        xig.xig_gen = V_tcbinfo.ipi_gencnt;
//ScenSim-Port//        xig.xig_sogen = so_gencnt;
//ScenSim-Port//        xig.xig_count = V_tcbinfo.ipi_count + pcb_count;
//ScenSim-Port//        INP_INFO_RUNLOCK(&V_tcbinfo);
//ScenSim-Port//        error = SYSCTL_OUT(req, &xig, sizeof xig);
//ScenSim-Port//    }
//ScenSim-Port//    free(inp_list, M_TEMP);
//ScenSim-Port//    return (error);
//ScenSim-Port//}

//ScenSim-Port//SYSCTL_PROC(_net_inet_tcp, TCPCTL_PCBLIST, pcblist,
//ScenSim-Port//    CTLTYPE_OPAQUE | CTLFLAG_RD, NULL, 0,
//ScenSim-Port//    tcp_pcblist, "S,xtcpcb", "List of active TCP connections");

#ifdef INET
//ScenSim-Port//static int
//ScenSim-Port//tcp_getcred(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    struct xucred xuc;
//ScenSim-Port//    struct sockaddr_in addrs[2];
//ScenSim-Port//    struct inpcb *inp;
//ScenSim-Port//    int error;
//ScenSim-Port//
//ScenSim-Port//    error = priv_check(req->td, PRIV_NETINET_GETCRED);
//ScenSim-Port//    if (error)
//ScenSim-Port//        return (error);
//ScenSim-Port//    error = SYSCTL_IN(req, addrs, sizeof(addrs));
//ScenSim-Port//    if (error)
//ScenSim-Port//        return (error);
//ScenSim-Port//    inp = in_pcblookup(&V_tcbinfo, addrs[1].sin_addr, addrs[1].sin_port,
//ScenSim-Port//        addrs[0].sin_addr, addrs[0].sin_port, INPLOOKUP_RLOCKPCB, NULL);
//ScenSim-Port//    if (inp != NULL) {
//ScenSim-Port//        if (inp->inp_socket == NULL)
//ScenSim-Port//            error = ENOENT;
//ScenSim-Port//        if (error == 0)
//ScenSim-Port//            error = cr_canseeinpcb(req->td->td_ucred, inp);
//ScenSim-Port//        if (error == 0)
//ScenSim-Port//            cru2x(inp->inp_cred, &xuc);
//ScenSim-Port//        INP_RUNLOCK(inp);
//ScenSim-Port//    } else
//ScenSim-Port//        error = ENOENT;
//ScenSim-Port//    if (error == 0)
//ScenSim-Port//        error = SYSCTL_OUT(req, &xuc, sizeof(struct xucred));
//ScenSim-Port//    return (error);
//ScenSim-Port//}

//ScenSim-Port//SYSCTL_PROC(_net_inet_tcp, OID_AUTO, getcred,
//ScenSim-Port//    CTLTYPE_OPAQUE|CTLFLAG_RW|CTLFLAG_PRISON, 0, 0,
//ScenSim-Port//    tcp_getcred, "S,xucred", "Get the xucred of a TCP connection");
#endif /* INET */

#ifdef INET6
//ScenSim-Port//static int
//ScenSim-Port//tcp6_getcred(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    struct xucred xuc;
//ScenSim-Port//    struct sockaddr_in6 addrs[2];
//ScenSim-Port//    struct inpcb *inp;
//ScenSim-Port//    int error;
//ScenSim-Port//#ifdef INET
//ScenSim-Port//    int mapped = 0;
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//    error = priv_check(req->td, PRIV_NETINET_GETCRED);
//ScenSim-Port//    if (error)
//ScenSim-Port//        return (error);
//ScenSim-Port//    error = SYSCTL_IN(req, addrs, sizeof(addrs));
//ScenSim-Port//    if (error)
//ScenSim-Port//        return (error);
//ScenSim-Port//    if ((error = sa6_embedscope(&addrs[0], V_ip6_use_defzone)) != 0 ||
//ScenSim-Port//        (error = sa6_embedscope(&addrs[1], V_ip6_use_defzone)) != 0) {
//ScenSim-Port//        return (error);
//ScenSim-Port//    }
//ScenSim-Port//    if (IN6_IS_ADDR_V4MAPPED(&addrs[0].sin6_addr)) {
//ScenSim-Port//#ifdef INET
//ScenSim-Port//        if (IN6_IS_ADDR_V4MAPPED(&addrs[1].sin6_addr))
//ScenSim-Port//            mapped = 1;
//ScenSim-Port//        else
//ScenSim-Port//#endif
//ScenSim-Port//            return (EINVAL);
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//#ifdef INET
//ScenSim-Port//    if (mapped == 1)
//ScenSim-Port//        inp = in_pcblookup(&V_tcbinfo,
//ScenSim-Port//            *(struct in_addr *)&addrs[1].sin6_addr.s6_addr[12],
//ScenSim-Port//            addrs[1].sin6_port,
//ScenSim-Port//            *(struct in_addr *)&addrs[0].sin6_addr.s6_addr[12],
//ScenSim-Port//            addrs[0].sin6_port, INPLOOKUP_RLOCKPCB, NULL);
//ScenSim-Port//    else
//ScenSim-Port//#endif
//ScenSim-Port//        inp = in6_pcblookup(&V_tcbinfo,
//ScenSim-Port//            &addrs[1].sin6_addr, addrs[1].sin6_port,
//ScenSim-Port//            &addrs[0].sin6_addr, addrs[0].sin6_port,
//ScenSim-Port//            INPLOOKUP_RLOCKPCB, NULL);
//ScenSim-Port//    if (inp != NULL) {
//ScenSim-Port//        if (inp->inp_socket == NULL)
//ScenSim-Port//            error = ENOENT;
//ScenSim-Port//        if (error == 0)
//ScenSim-Port//            error = cr_canseeinpcb(req->td->td_ucred, inp);
//ScenSim-Port//        if (error == 0)
//ScenSim-Port//            cru2x(inp->inp_cred, &xuc);
//ScenSim-Port//        INP_RUNLOCK(inp);
//ScenSim-Port//    } else
//ScenSim-Port//        error = ENOENT;
//ScenSim-Port//    if (error == 0)
//ScenSim-Port//        error = SYSCTL_OUT(req, &xuc, sizeof(struct xucred));
//ScenSim-Port//    return (error);
//ScenSim-Port//}

//ScenSim-Port//SYSCTL_PROC(_net_inet6_tcp6, OID_AUTO, getcred,
//ScenSim-Port//    CTLTYPE_OPAQUE|CTLFLAG_RW|CTLFLAG_PRISON, 0, 0,
//ScenSim-Port//    tcp6_getcred, "S,xucred", "Get the xucred of a TCP6 connection");
#endif /* INET6 */


#ifdef INET
void
tcp_ctlinput(int cmd, struct sockaddr *sa, void *vip)
{
//ScenSim-Port//    struct ip *ip = vip;
    struct ip *ip = (struct ip *)vip;                           //ScenSim-Port//
    struct tcphdr *th;
    struct in_addr faddr;
    struct inpcb *inp;
    struct tcpcb *tp;
    struct inpcb *(*notify)(struct inpcb *, int) = tcp_notify;
    struct icmp *icp;
    struct in_conninfo inc;
    tcp_seq icmp_tcp_seq;
    int mtu;

    faddr = ((struct sockaddr_in *)sa)->sin_addr;
    if (sa->sa_family != AF_INET || faddr.s_addr == INADDR_ANY)
        return;

    if (cmd == PRC_MSGSIZE)
        notify = tcp_mtudisc;
    else if (V_icmp_may_rst && (cmd == PRC_UNREACH_ADMIN_PROHIB ||
        cmd == PRC_UNREACH_PORT || cmd == PRC_TIMXCEED_INTRANS) && ip)
        notify = tcp_drop_syn_sent;
    /*
     * Redirects don't need to be handled up here.
     */
    else if (PRC_IS_REDIRECT(cmd))
        return;
    /*
     * Source quench is depreciated.
     */
    else if (cmd == PRC_QUENCH)
        return;
    /*
     * Hostdead is ugly because it goes linearly through all PCBs.
     * XXX: We never get this from ICMP, otherwise it makes an
     * excellent DoS attack on machines with many connections.
     */
    else if (cmd == PRC_HOSTDEAD)
        ip = NULL;
    else if ((unsigned)cmd >= PRC_NCMDS || inetctlerrmap[cmd] == 0)
        return;
    if (ip != NULL) {
        icp = (struct icmp *)((caddr_t)ip
                      - offsetof(struct icmp, icmp_ip));
        th = (struct tcphdr *)((caddr_t)ip
                       + (ip->ip_hl << 2));
//ScenSim-Port//        INP_INFO_WLOCK(&V_tcbinfo);
        inp = in_pcblookup(&V_tcbinfo, faddr, th->th_dport,
            ip->ip_src, th->th_sport, INPLOOKUP_WLOCKPCB, NULL);
        if (inp != NULL)  {
            if (!(inp->inp_flags & INP_TIMEWAIT) &&
                !(inp->inp_flags & INP_DROPPED) &&
                !(inp->inp_socket == NULL)) {
                icmp_tcp_seq = htonl(th->th_seq);
                tp = intotcpcb(inp);
                if (SEQ_GEQ(icmp_tcp_seq, tp->snd_una) &&
                    SEQ_LT(icmp_tcp_seq, tp->snd_max)) {
                    if (cmd == PRC_MSGSIZE) {
                        /*
                         * MTU discovery:
                         * If we got a needfrag set the MTU
                         * in the route to the suggested new
                         * value (if given) and then notify.
                         */
                        bzero(&inc, sizeof(inc));
                        inc.inc_faddr = faddr;
//ScenSim-Port//                        inc.inc_fibnum =
//ScenSim-Port//                        inp->inp_inc.inc_fibnum;

                        mtu = ntohs(icp->icmp_nextmtu);
                        /*
                         * If no alternative MTU was
                         * proposed, try the next smaller
                         * one.  ip->ip_len has already
                         * been swapped in icmp_input().
                         */
                        if (!mtu)
                        mtu = ip_next_mtu(ip->ip_len,
                         1);
                        if (mtu < V_tcp_minmss
                         + sizeof(struct tcpiphdr))
                        mtu = V_tcp_minmss
                         + sizeof(struct tcpiphdr);
                        /*
                         * Only cache the MTU if it
                         * is smaller than the interface
                         * or route MTU.  tcp_mtudisc()
                         * will do right thing by itself.
                         */
                        if (mtu <= tcp_maxmtu(&inc, NULL))
                        tcp_hc_updatemtu(&inc, mtu);
                    }

                    inp = (*notify)(inp, inetctlerrmap[cmd]);
                }
            }
//ScenSim-Port//            if (inp != NULL)
//ScenSim-Port//                INP_WUNLOCK(inp);
        } else {
            bzero(&inc, sizeof(inc));
            inc.inc_fport = th->th_dport;
            inc.inc_lport = th->th_sport;
            inc.inc_faddr = faddr;
            inc.inc_laddr = ip->ip_src;
            syncache_unreach(&inc, th);
        }
//ScenSim-Port//        INP_INFO_WUNLOCK(&V_tcbinfo);
    } else
        in_pcbnotifyall(&V_tcbinfo, faddr, inetctlerrmap[cmd], notify);
}
#endif /* INET */

#ifdef INET6
void
tcp6_ctlinput(int cmd, struct sockaddr *sa, void *d)
{
    struct tcphdr th;
    struct inpcb *(*notify)(struct inpcb *, int) = tcp_notify;
    struct ip6_hdr *ip6;
    struct mbuf *m;
    struct ip6ctlparam *ip6cp = NULL;
    const struct sockaddr_in6 *sa6_src = NULL;
    int off;
    struct tcp_portonly {
        u_int16_t th_sport;
        u_int16_t th_dport;
    }; //NotUsed: *thp;

    if (sa->sa_family != AF_INET6 ||
        sa->sa_len != sizeof(struct sockaddr_in6))
        return;

    if (cmd == PRC_MSGSIZE)
        notify = tcp_mtudisc;
    else if (!PRC_IS_REDIRECT(cmd) &&
         ((unsigned)cmd >= PRC_NCMDS || inet6ctlerrmap[cmd] == 0))
        return;
    /* Source quench is depreciated. */
    else if (cmd == PRC_QUENCH)
        return;

    /* if the parameter is from icmp6, decode it. */
    if (d != NULL) {
        ip6cp = (struct ip6ctlparam *)d;
        m = ip6cp->ip6c_m;
        ip6 = ip6cp->ip6c_ip6;
        off = ip6cp->ip6c_off;
        sa6_src = ip6cp->ip6c_src;
    } else {
        m = NULL;
        ip6 = NULL;
        off = 0;    /* fool gcc */
        sa6_src = &sa6_any;
    }

    if (ip6 != NULL) {
        struct in_conninfo inc;
        /*
         * XXX: We assume that when IPV6 is non NULL,
         * M and OFF are valid.
         */

        /* check if we can safely examine src and dst ports */
        if (m->m_pkthdr.len < off + sizeof(tcp_portonly/**thp*/))
            return;

        bzero(&th, sizeof(th));
        m_copydata(m, off, sizeof(tcp_portonly/**thp*/), (caddr_t)&th);

        in6_pcbnotify(&V_tcbinfo, sa, th.th_dport,
            (struct sockaddr *)ip6cp->ip6c_src,
            th.th_sport, cmd, NULL, notify);

        bzero(&inc, sizeof(inc));
        inc.inc_fport = th.th_dport;
        inc.inc_lport = th.th_sport;
        inc.inc6_faddr = ((struct sockaddr_in6 *)sa)->sin6_addr;
        inc.inc6_laddr = ip6cp->ip6c_src->sin6_addr;
        inc.inc_flags |= INC_ISIPV6;
//ScenSim-Port//        INP_INFO_WLOCK(&V_tcbinfo);
        syncache_unreach(&inc, &th);
//ScenSim-Port//        INP_INFO_WUNLOCK(&V_tcbinfo);
    } else
        in6_pcbnotify(&V_tcbinfo, sa, 0, (const struct sockaddr *)sa6_src,
                  0, cmd, NULL, notify);
}
#endif /* INET6 */


/*
 * Following is where TCP initial sequence number generation occurs.
 *
 * There are two places where we must use initial sequence numbers:
 * 1.  In SYN-ACK packets.
 * 2.  In SYN packets.
 *
 * All ISNs for SYN-ACK packets are generated by the syncache.  See
 * tcp_syncache.c for details.
 *
 * The ISNs in SYN packets must be monotonic; TIME_WAIT recycling
 * depends on this property.  In addition, these ISNs should be
 * unguessable so as to prevent connection hijacking.  To satisfy
 * the requirements of this situation, the algorithm outlined in
 * RFC 1948 is used, with only small modifications.
 *
 * Implementation details:
 *
 * Time is based off the system timer, and is corrected so that it
 * increases by one megabyte per second.  This allows for proper
 * recycling on high speed LANs while still leaving over an hour
 * before rollover.
 *
 * As reading the *exact* system time is too expensive to be done
 * whenever setting up a TCP connection, we increment the time
 * offset in two ways.  First, a small random positive increment
 * is added to isn_offset for each connection that is set up.
 * Second, the function tcp_isn_tick fires once per clock tick
 * and increments isn_offset as necessary so that sequence numbers
 * are incremented at approximately ISN_BYTES_PER_SECOND.  The
 * random positive increments serve only to ensure that the same
 * exact sequence number is never sent out twice (as could otherwise
 * happen when a port is recycled in less than the system tick
 * interval.)
 *
 * net.inet.tcp.isn_reseed_interval controls the number of seconds
 * between seeding of isn_secret.  This is normally set to zero,
 * as reseeding should not be necessary.
 *
 * Locking of the global variables isn_secret, isn_last_reseed, isn_offset,
 * isn_offset_old, and isn_ctx is performed using the TCP pcbinfo lock.  In
 * general, this means holding an exclusive (write) lock.
 */

#define ISN_BYTES_PER_SECOND 1048576
#define ISN_STATIC_INCREMENT 4096
#define ISN_RANDOM_INCREMENT (4096 - 1)

//ScenSim-Port//static VNET_DEFINE(u_char, isn_secret[32]);
//ScenSim-Port//static VNET_DEFINE(int, isn_last);
//ScenSim-Port//static VNET_DEFINE(int, isn_last_reseed);
//ScenSim-Port//static VNET_DEFINE(u_int32_t, isn_offset);
//ScenSim-Port//static VNET_DEFINE(u_int32_t, isn_offset_old);

//ScenSim-Port//#define V_isn_secret            VNET(isn_secret)
//ScenSim-Port//#define V_isn_last          VNET(isn_last)
//ScenSim-Port//#define V_isn_last_reseed       VNET(isn_last_reseed)
//ScenSim-Port//#define V_isn_offset            VNET(isn_offset)
//ScenSim-Port//#define V_isn_offset_old        VNET(isn_offset_old)

tcp_seq
tcp_new_isn(struct tcpcb *tp)
{
    MD5_CTX isn_ctx;
    u_int32_t md5_buffer[4];
    tcp_seq new_isn;
    u_int32_t projected_offset;

//ScenSim-Port//    INP_WLOCK_ASSERT(tp->t_inpcb);

//ScenSim-Port//    ISN_LOCK();
    /* Seed if this is the first use, reseed if requested. */
    if ((V_isn_last_reseed == 0) || ((V_tcp_isn_reseed_interval > 0) &&
//ScenSim-Port//         (((u_int)V_isn_last_reseed + (u_int)V_tcp_isn_reseed_interval*hz)
        (((u_int)V_isn_last_reseed +                            //ScenSim-Port//
            (u_int)V_tcp_isn_reseed_interval)                   //ScenSim-Port//
        < (u_int)ticks))) {
        read_random(&V_isn_secret, sizeof(V_isn_secret));
        V_isn_last_reseed = ticks;
    }

    /* Compute the md5 hash and return the ISN. */
    MD5Init(&isn_ctx);
    MD5Update(&isn_ctx, (u_char *) &tp->t_inpcb->inp_fport, sizeof(u_short));
    MD5Update(&isn_ctx, (u_char *) &tp->t_inpcb->inp_lport, sizeof(u_short));
#ifdef INET6
    if ((tp->t_inpcb->inp_vflag & INP_IPV6) != 0) {
        MD5Update(&isn_ctx, (u_char *) &tp->t_inpcb->in6p_faddr,
              sizeof(struct in6_addr));
        MD5Update(&isn_ctx, (u_char *) &tp->t_inpcb->in6p_laddr,
              sizeof(struct in6_addr));
    } else
#endif
    {
        MD5Update(&isn_ctx, (u_char *) &tp->t_inpcb->inp_faddr,
              sizeof(struct in_addr));
        MD5Update(&isn_ctx, (u_char *) &tp->t_inpcb->inp_laddr,
              sizeof(struct in_addr));
    }
    MD5Update(&isn_ctx, (u_char *) &V_isn_secret, sizeof(V_isn_secret));
    MD5Final((u_char *) &md5_buffer, &isn_ctx);
    new_isn = (tcp_seq) md5_buffer[0];
    V_isn_offset += ISN_STATIC_INCREMENT +
        (arc4random() & ISN_RANDOM_INCREMENT);
    if (ticks != V_isn_last) {
        projected_offset = V_isn_offset_old +
            ISN_BYTES_PER_SECOND / hz * (ticks - V_isn_last);
        if (SEQ_GT(projected_offset, V_isn_offset))
            V_isn_offset = projected_offset;
        V_isn_offset_old = V_isn_offset;
        V_isn_last = ticks;
    }
    new_isn += V_isn_offset;
//ScenSim-Port//    ISN_UNLOCK();
    return (new_isn);
}

/*
 * When a specific ICMP unreachable message is received and the
 * connection state is SYN-SENT, drop the connection.  This behavior
 * is controlled by the icmp_may_rst sysctl.
 */
struct inpcb *
tcp_drop_syn_sent(struct inpcb *inp, int errnoVal)
{
    struct tcpcb *tp;

//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK_ASSERT(inp);

    if ((inp->inp_flags & INP_TIMEWAIT) ||
        (inp->inp_flags & INP_DROPPED))
        return (inp);

    tp = intotcpcb(inp);
    if (tp->t_state != TCPS_SYN_SENT)
        return (inp);

    tp = tcp_drop(tp, errnoVal);
    if (tp != NULL)
        return (inp);
    else
        return (NULL);
}

/*
 * When `need fragmentation' ICMP is received, update our idea of the MSS
 * based on the new value in the route.  Also nudge TCP to send something,
 * since we know the packet we just sent was dropped.
 * This duplicates some code in the tcp_mss() function in tcp_input.c.
 */
struct inpcb *
tcp_mtudisc(struct inpcb *inp, int errnoVal)
{
    struct tcpcb *tp;
    struct socket *so;

//ScenSim-Port//    INP_WLOCK_ASSERT(inp);
    if ((inp->inp_flags & INP_TIMEWAIT) ||
        (inp->inp_flags & INP_DROPPED))
        return (inp);

    tp = intotcpcb(inp);
//ScenSim-Port//    KASSERT(tp != NULL, ("tcp_mtudisc: tp == NULL"));

    tcp_mss_update(tp, -1, NULL, NULL);

    so = inp->inp_socket;
//ScenSim-Port//    SOCKBUF_LOCK(&so->so_snd);
    /* If the mss is larger than the socket buffer, decrease the mss. */
    if (so->so_snd.sb_hiwat < tp->t_maxseg)
        tp->t_maxseg = so->so_snd.sb_hiwat;
//ScenSim-Port//    SOCKBUF_UNLOCK(&so->so_snd);

    TCPSTAT_INC(tcps_mturesent);
    tp->t_rtttime = 0;
    tp->snd_nxt = tp->snd_una;
    tcp_free_sackholes(tp);
    tp->snd_recover = tp->snd_max;
    if (tp->t_flags & TF_SACK_PERMIT)
        EXIT_FASTRECOVERY(tp->t_flags);
    tcp_output_send(tp);
    return (inp);
}

#ifdef INET
/*
 * Look-up the routing entry to the peer of this inpcb.  If no route
 * is found and it cannot be allocated, then return 0.  This routine
 * is called by TCP routines that access the rmx structure and by
 * tcp_mss_update to get the peer/interface MTU.
 */
u_long
tcp_maxmtu(struct in_conninfo *inc, int *flags)
{
//ScenSim-Port//    struct route sro;
//ScenSim-Port//    struct sockaddr_in *dst;
//ScenSim-Port//    struct ifnet *ifp;
    u_long maxmtu = 0;

//ScenSim-Port//    KASSERT(inc != NULL, ("tcp_maxmtu with NULL in_conninfo pointer"));

//ScenSim-Port//    bzero(&sro, sizeof(sro));
//ScenSim-Port//    if (inc->inc_faddr.s_addr != INADDR_ANY) {
//ScenSim-Port//            dst = (struct sockaddr_in *)&sro.ro_dst;
//ScenSim-Port//        dst->sin_family = AF_INET;
//ScenSim-Port//        dst->sin_len = sizeof(*dst);
//ScenSim-Port//        dst->sin_addr = inc->inc_faddr;
//ScenSim-Port//        in_rtalloc_ign(&sro, 0, inc->inc_fibnum);
//ScenSim-Port//    }
//ScenSim-Port//    if (sro.ro_rt != NULL) {
//ScenSim-Port//        ifp = sro.ro_rt->rt_ifp;
//ScenSim-Port//        if (sro.ro_rt->rt_rmx.rmx_mtu == 0)
//ScenSim-Port//            maxmtu = ifp->if_mtu;
//ScenSim-Port//        else
//ScenSim-Port//            maxmtu = min(sro.ro_rt->rt_rmx.rmx_mtu, ifp->if_mtu);

        /* Report additional interface capabilities. */
//ScenSim-Port//        if (flags != NULL) {
//ScenSim-Port//            if (ifp->if_capenable & IFCAP_TSO4 &&
//ScenSim-Port//                ifp->if_hwassist & CSUM_TSO)
//ScenSim-Port//                *flags |= CSUM_TSO;
//ScenSim-Port//        }
//ScenSim-Port//        RTFREE(sro.ro_rt);
//ScenSim-Port//    }
    return (maxmtu);
}
#endif /* INET */

#ifdef INET6
u_long
tcp_maxmtu6(struct in_conninfo *inc, int *flags)
{
//ScenSim-Port//    struct route_in6 sro6;
//ScenSim-Port//    struct ifnet *ifp;
    u_long maxmtu = 0;

//ScenSim-Port//    KASSERT(inc != NULL, ("tcp_maxmtu6 with NULL in_conninfo pointer"));

//ScenSim-Port//    bzero(&sro6, sizeof(sro6));
//ScenSim-Port//    if (!IN6_IS_ADDR_UNSPECIFIED(&inc->inc6_faddr)) {
//ScenSim-Port//        sro6.ro_dst.sin6_family = AF_INET6;
//ScenSim-Port//        sro6.ro_dst.sin6_len = sizeof(struct sockaddr_in6);
//ScenSim-Port//        sro6.ro_dst.sin6_addr = inc->inc6_faddr;
//ScenSim-Port//        rtalloc_ign((struct route *)&sro6, 0);
//ScenSim-Port//    }
//ScenSim-Port//    if (sro6.ro_rt != NULL) {
//ScenSim-Port//        ifp = sro6.ro_rt->rt_ifp;
//ScenSim-Port//        if (sro6.ro_rt->rt_rmx.rmx_mtu == 0)
//ScenSim-Port//            maxmtu = IN6_LINKMTU(sro6.ro_rt->rt_ifp);
//ScenSim-Port//        else
//ScenSim-Port//            maxmtu = min(sro6.ro_rt->rt_rmx.rmx_mtu,
//ScenSim-Port//                     IN6_LINKMTU(sro6.ro_rt->rt_ifp));

        /* Report additional interface capabilities. */
//ScenSim-Port//        if (flags != NULL) {
//ScenSim-Port//            if (ifp->if_capenable & IFCAP_TSO6 &&
//ScenSim-Port//                ifp->if_hwassist & CSUM_TSO)
//ScenSim-Port//                *flags |= CSUM_TSO;
//ScenSim-Port//        }
//ScenSim-Port//        RTFREE(sro6.ro_rt);
//ScenSim-Port//    }

    return (maxmtu);
}
#endif /* INET6 */

//ScenSim-Port//#ifdef IPSEC
//ScenSim-Port///* compute ESP/AH header size for TCP, including outer IP header. */
//ScenSim-Port//size_t
//ScenSim-Port//ipsec_hdrsiz_tcp(struct tcpcb *tp)
//ScenSim-Port//{
//ScenSim-Port//    struct inpcb *inp;
//ScenSim-Port//    struct mbuf *m;
//ScenSim-Port//    size_t hdrsiz;
//ScenSim-Port//    struct ip *ip;
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//    struct ip6_hdr *ip6;
//ScenSim-Port//#endif
//ScenSim-Port//    struct tcphdr *th;
//ScenSim-Port//
//ScenSim-Port//    if ((tp == NULL) || ((inp = tp->t_inpcb) == NULL))
//ScenSim-Port//        return (0);
//ScenSim-Port//    MGETHDR(m, M_DONTWAIT, MT_DATA);
//ScenSim-Port//    if (!m)
//ScenSim-Port//        return (0);
//ScenSim-Port//
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//    if ((inp->inp_vflag & INP_IPV6) != 0) {
//ScenSim-Port//        ip6 = mtod(m, struct ip6_hdr *);
//ScenSim-Port//        th = (struct tcphdr *)(ip6 + 1);
//ScenSim-Port//        m->m_pkthdr.len = m->m_len =
//ScenSim-Port//            sizeof(struct ip6_hdr) + sizeof(struct tcphdr);
//ScenSim-Port//        tcpip_fillheaders(inp, ip6, th);
//ScenSim-Port//        hdrsiz = ipsec_hdrsiz(m, IPSEC_DIR_OUTBOUND, inp);
//ScenSim-Port//    } else
//ScenSim-Port//#endif /* INET6 */
//ScenSim-Port//    {
//ScenSim-Port//        ip = mtod(m, struct ip *);
//ScenSim-Port//        th = (struct tcphdr *)(ip + 1);
//ScenSim-Port//        m->m_pkthdr.len = m->m_len = sizeof(struct tcpiphdr);
//ScenSim-Port//        tcpip_fillheaders(inp, ip, th);
//ScenSim-Port//        hdrsiz = ipsec_hdrsiz(m, IPSEC_DIR_OUTBOUND, inp);
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//    m_free(m);
//ScenSim-Port//    return (hdrsiz);
//ScenSim-Port//}
//ScenSim-Port//#endif /* IPSEC */

//ScenSim-Port//#ifdef TCP_SIGNATURE
//ScenSim-Port///*
//ScenSim-Port// * Callback function invoked by m_apply() to digest TCP segment data
//ScenSim-Port// * contained within an mbuf chain.
//ScenSim-Port// */
//ScenSim-Port//static int
//ScenSim-Port//tcp_signature_apply(void *fstate, void *data, u_int len)
//ScenSim-Port//{
//ScenSim-Port//
//ScenSim-Port//    MD5Update(fstate, (u_char *)data, len);
//ScenSim-Port//    return (0);
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port///*
//ScenSim-Port// * Compute TCP-MD5 hash of a TCP segment. (RFC2385)
//ScenSim-Port// *
//ScenSim-Port// * Parameters:
//ScenSim-Port// * m        pointer to head of mbuf chain
//ScenSim-Port// * _unused
//ScenSim-Port// * len      length of TCP segment data, excluding options
//ScenSim-Port// * optlen   length of TCP segment options
//ScenSim-Port// * buf      pointer to storage for computed MD5 digest
//ScenSim-Port// * direction    direction of flow (IPSEC_DIR_INBOUND or OUTBOUND)
//ScenSim-Port// *
//ScenSim-Port// * We do this over ip, tcphdr, segment data, and the key in the SADB.
//ScenSim-Port// * When called from tcp_input(), we can be sure that th_sum has been
//ScenSim-Port// * zeroed out and verified already.
//ScenSim-Port// *
//ScenSim-Port// * Return 0 if successful, otherwise return -1.
//ScenSim-Port// *
//ScenSim-Port// * XXX The key is retrieved from the system's PF_KEY SADB, by keying a
//ScenSim-Port// * search with the destination IP address, and a 'magic SPI' to be
//ScenSim-Port// * determined by the application. This is hardcoded elsewhere to 1179
//ScenSim-Port// * right now. Another branch of this code exists which uses the SPD to
//ScenSim-Port// * specify per-application flows but it is unstable.
//ScenSim-Port// */
//ScenSim-Port//int
//ScenSim-Port//tcp_signature_compute(struct mbuf *m, int _unused, int len, int optlen,
//ScenSim-Port//    u_char *buf, u_int direction)
//ScenSim-Port//{
//ScenSim-Port//    union sockaddr_union dst;
//ScenSim-Port//#ifdef INET
//ScenSim-Port//    struct ippseudo ippseudo;
//ScenSim-Port//#endif
//ScenSim-Port//    MD5_CTX ctx;
//ScenSim-Port//    int doff;
//ScenSim-Port//    struct ip *ip;
//ScenSim-Port//#ifdef INET
//ScenSim-Port//    struct ipovly *ipovly;
//ScenSim-Port//#endif
//ScenSim-Port//    struct secasvar *sav;
//ScenSim-Port//    struct tcphdr *th;
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//    struct ip6_hdr *ip6;
//ScenSim-Port//    struct in6_addr in6;
//ScenSim-Port//    char ip6buf[INET6_ADDRSTRLEN];
//ScenSim-Port//    uint32_t plen;
//ScenSim-Port//    uint16_t nhdr;
//ScenSim-Port//#endif
//ScenSim-Port//    u_short savecsum;
//ScenSim-Port//
//ScenSim-Port//    KASSERT(m != NULL, ("NULL mbuf chain"));
//ScenSim-Port//    KASSERT(buf != NULL, ("NULL signature pointer"));
//ScenSim-Port//
//ScenSim-Port//    /* Extract the destination from the IP header in the mbuf. */
//ScenSim-Port//    bzero(&dst, sizeof(union sockaddr_union));
//ScenSim-Port//    ip = mtod(m, struct ip *);
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//    ip6 = NULL; /* Make the compiler happy. */
//ScenSim-Port//#endif
//ScenSim-Port//    switch (ip->ip_v) {
//ScenSim-Port//#ifdef INET
//ScenSim-Port//    case IPVERSION:
//ScenSim-Port//        dst.sa.sa_len = sizeof(struct sockaddr_in);
//ScenSim-Port//        dst.sa.sa_family = AF_INET;
//ScenSim-Port//        dst.sin.sin_addr = (direction == IPSEC_DIR_INBOUND) ?
//ScenSim-Port//            ip->ip_src : ip->ip_dst;
//ScenSim-Port//        break;
//ScenSim-Port//#endif
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//    case (IPV6_VERSION >> 4):
//ScenSim-Port//        ip6 = mtod(m, struct ip6_hdr *);
//ScenSim-Port//        dst.sa.sa_len = sizeof(struct sockaddr_in6);
//ScenSim-Port//        dst.sa.sa_family = AF_INET6;
//ScenSim-Port//        dst.sin6.sin6_addr = (direction == IPSEC_DIR_INBOUND) ?
//ScenSim-Port//            ip6->ip6_src : ip6->ip6_dst;
//ScenSim-Port//        break;
//ScenSim-Port//#endif
//ScenSim-Port//    default:
//ScenSim-Port//        return (EINVAL);
//ScenSim-Port//        /* NOTREACHED */
//ScenSim-Port//        break;
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//    /* Look up an SADB entry which matches the address of the peer. */
//ScenSim-Port//    sav = KEY_ALLOCSA(&dst, IPPROTO_TCP, htonl(TCP_SIG_SPI));
//ScenSim-Port//    if (sav == NULL) {
//ScenSim-Port//        ipseclog((LOG_ERR, "%s: SADB lookup failed for %s\n", __func__,
//ScenSim-Port//            (ip->ip_v == IPVERSION) ? inet_ntoa(dst.sin.sin_addr) :
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//            (ip->ip_v == (IPV6_VERSION >> 4)) ?
//ScenSim-Port//                ip6_sprintf(ip6buf, &dst.sin6.sin6_addr) :
//ScenSim-Port//#endif
//ScenSim-Port//            "(unsupported)"));
//ScenSim-Port//        return (EINVAL);
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//    MD5Init(&ctx);
//ScenSim-Port//    /*
//ScenSim-Port//     * Step 1: Update MD5 hash with IP(v6) pseudo-header.
//ScenSim-Port//     *
//ScenSim-Port//     * XXX The ippseudo header MUST be digested in network byte order,
//ScenSim-Port//     * or else we'll fail the regression test. Assume all fields we've
//ScenSim-Port//     * been doing arithmetic on have been in host byte order.
//ScenSim-Port//     * XXX One cannot depend on ipovly->ih_len here. When called from
//ScenSim-Port//     * tcp_output(), the underlying ip_len member has not yet been set.
//ScenSim-Port//     */
//ScenSim-Port//    switch (ip->ip_v) {
//ScenSim-Port//#ifdef INET
//ScenSim-Port//    case IPVERSION:
//ScenSim-Port//        ipovly = (struct ipovly *)ip;
//ScenSim-Port//        ippseudo.ippseudo_src = ipovly->ih_src;
//ScenSim-Port//        ippseudo.ippseudo_dst = ipovly->ih_dst;
//ScenSim-Port//        ippseudo.ippseudo_pad = 0;
//ScenSim-Port//        ippseudo.ippseudo_p = IPPROTO_TCP;
//ScenSim-Port//        ippseudo.ippseudo_len = htons(len + sizeof(struct tcphdr) +
//ScenSim-Port//            optlen);
//ScenSim-Port//        MD5Update(&ctx, (char *)&ippseudo, sizeof(struct ippseudo));
//ScenSim-Port//
//ScenSim-Port//        th = (struct tcphdr *)((u_char *)ip + sizeof(struct ip));
//ScenSim-Port//        doff = sizeof(struct ip) + sizeof(struct tcphdr) + optlen;
//ScenSim-Port//        break;
//ScenSim-Port//#endif
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//    /*
//ScenSim-Port//     * RFC 2385, 2.0  Proposal
//ScenSim-Port//     * For IPv6, the pseudo-header is as described in RFC 2460, namely the
//ScenSim-Port//     * 128-bit source IPv6 address, 128-bit destination IPv6 address, zero-
//ScenSim-Port//     * extended next header value (to form 32 bits), and 32-bit segment
//ScenSim-Port//     * length.
//ScenSim-Port//     * Note: Upper-Layer Packet Length comes before Next Header.
//ScenSim-Port//     */
//ScenSim-Port//    case (IPV6_VERSION >> 4):
//ScenSim-Port//        in6 = ip6->ip6_src;
//ScenSim-Port//        in6_clearscope(&in6);
//ScenSim-Port//        MD5Update(&ctx, (char *)&in6, sizeof(struct in6_addr));
//ScenSim-Port//        in6 = ip6->ip6_dst;
//ScenSim-Port//        in6_clearscope(&in6);
//ScenSim-Port//        MD5Update(&ctx, (char *)&in6, sizeof(struct in6_addr));
//ScenSim-Port//        plen = htonl(len + sizeof(struct tcphdr) + optlen);
//ScenSim-Port//        MD5Update(&ctx, (char *)&plen, sizeof(uint32_t));
//ScenSim-Port//        nhdr = 0;
//ScenSim-Port//        MD5Update(&ctx, (char *)&nhdr, sizeof(uint8_t));
//ScenSim-Port//        MD5Update(&ctx, (char *)&nhdr, sizeof(uint8_t));
//ScenSim-Port//        MD5Update(&ctx, (char *)&nhdr, sizeof(uint8_t));
//ScenSim-Port//        nhdr = IPPROTO_TCP;
//ScenSim-Port//        MD5Update(&ctx, (char *)&nhdr, sizeof(uint8_t));
//ScenSim-Port//
//ScenSim-Port//        th = (struct tcphdr *)((u_char *)ip6 + sizeof(struct ip6_hdr));
//ScenSim-Port//        doff = sizeof(struct ip6_hdr) + sizeof(struct tcphdr) + optlen;
//ScenSim-Port//        break;
//ScenSim-Port//#endif
//ScenSim-Port//    default:
//ScenSim-Port//        return (EINVAL);
//ScenSim-Port//        /* NOTREACHED */
//ScenSim-Port//        break;
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//
//ScenSim-Port//    /*
//ScenSim-Port//     * Step 2: Update MD5 hash with TCP header, excluding options.
//ScenSim-Port//     * The TCP checksum must be set to zero.
//ScenSim-Port//     */
//ScenSim-Port//    savecsum = th->th_sum;
//ScenSim-Port//    th->th_sum = 0;
//ScenSim-Port//    MD5Update(&ctx, (char *)th, sizeof(struct tcphdr));
//ScenSim-Port//    th->th_sum = savecsum;
//ScenSim-Port//
//ScenSim-Port//    /*
//ScenSim-Port//     * Step 3: Update MD5 hash with TCP segment data.
//ScenSim-Port//     *         Use m_apply() to avoid an early m_pullup().
//ScenSim-Port//     */
//ScenSim-Port//    if (len > 0)
//ScenSim-Port//        m_apply(m, doff, len, tcp_signature_apply, &ctx);
//ScenSim-Port//
//ScenSim-Port//    /*
//ScenSim-Port//     * Step 4: Update MD5 hash with shared secret.
//ScenSim-Port//     */
//ScenSim-Port//    MD5Update(&ctx, sav->key_auth->key_data, _KEYLEN(sav->key_auth));
//ScenSim-Port//    MD5Final(buf, &ctx);
//ScenSim-Port//
//ScenSim-Port//    key_sa_recordxfer(sav, m);
//ScenSim-Port//    KEY_FREESAV(&sav);
//ScenSim-Port//    return (0);
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port///*
//ScenSim-Port// * Verify the TCP-MD5 hash of a TCP segment. (RFC2385)
//ScenSim-Port// *
//ScenSim-Port// * Parameters:
//ScenSim-Port// * m        pointer to head of mbuf chain
//ScenSim-Port// * len      length of TCP segment data, excluding options
//ScenSim-Port// * optlen   length of TCP segment options
//ScenSim-Port// * buf      pointer to storage for computed MD5 digest
//ScenSim-Port// * direction    direction of flow (IPSEC_DIR_INBOUND or OUTBOUND)
//ScenSim-Port// *
//ScenSim-Port// * Return 1 if successful, otherwise return 0.
//ScenSim-Port// */
//ScenSim-Port//int
//ScenSim-Port//tcp_signature_verify(struct mbuf *m, int off0, int tlen, int optlen,
//ScenSim-Port//    struct tcpopt *to, struct tcphdr *th, u_int tcpbflag)
//ScenSim-Port//{
//ScenSim-Port//    char tmpdigest[TCP_SIGLEN];
//ScenSim-Port//
//ScenSim-Port//    if (tcp_sig_checksigs == 0)
//ScenSim-Port//        return (1);
//ScenSim-Port//    if ((tcpbflag & TF_SIGNATURE) == 0) {
//ScenSim-Port//        if ((to->to_flags & TOF_SIGNATURE) != 0) {
//ScenSim-Port//
//ScenSim-Port//            /*
//ScenSim-Port//             * If this socket is not expecting signature but
//ScenSim-Port//             * the segment contains signature just fail.
//ScenSim-Port//             */
//ScenSim-Port//            TCPSTAT_INC(tcps_sig_err_sigopt);
//ScenSim-Port//            TCPSTAT_INC(tcps_sig_rcvbadsig);
//ScenSim-Port//            return (0);
//ScenSim-Port//        }
//ScenSim-Port//
//ScenSim-Port//        /* Signature is not expected, and not present in segment. */
//ScenSim-Port//        return (1);
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//    /*
//ScenSim-Port//     * If this socket is expecting signature but the segment does not
//ScenSim-Port//     * contain any just fail.
//ScenSim-Port//     */
//ScenSim-Port//    if ((to->to_flags & TOF_SIGNATURE) == 0) {
//ScenSim-Port//        TCPSTAT_INC(tcps_sig_err_nosigopt);
//ScenSim-Port//        TCPSTAT_INC(tcps_sig_rcvbadsig);
//ScenSim-Port//        return (0);
//ScenSim-Port//    }
//ScenSim-Port//    if (tcp_signature_compute(m, off0, tlen, optlen, &tmpdigest[0],
//ScenSim-Port//        IPSEC_DIR_INBOUND) == -1) {
//ScenSim-Port//        TCPSTAT_INC(tcps_sig_err_buildsig);
//ScenSim-Port//        TCPSTAT_INC(tcps_sig_rcvbadsig);
//ScenSim-Port//        return (0);
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//    if (bcmp(to->to_signature, &tmpdigest[0], TCP_SIGLEN) != 0) {
//ScenSim-Port//        TCPSTAT_INC(tcps_sig_rcvbadsig);
//ScenSim-Port//        return (0);
//ScenSim-Port//    }
//ScenSim-Port//    TCPSTAT_INC(tcps_sig_rcvgoodsig);
//ScenSim-Port//    return (1);
//ScenSim-Port//}
//ScenSim-Port//#endif /* TCP_SIGNATURE */

//ScenSim-Port//static int
//ScenSim-Port//sysctl_drop(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    /* addrs[0] is a foreign socket, addrs[1] is a local one. */
//ScenSim-Port//    struct sockaddr_storage addrs[2];
//ScenSim-Port//    struct inpcb *inp;
//ScenSim-Port//    struct tcpcb *tp;
//ScenSim-Port//    struct tcptw *tw;
//ScenSim-Port//    struct sockaddr_in *fin, *lin;
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//    struct sockaddr_in6 *fin6, *lin6;
//ScenSim-Port//#endif
//ScenSim-Port//    int error;
//ScenSim-Port//
//ScenSim-Port//    inp = NULL;
//ScenSim-Port//    fin = lin = NULL;
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//    fin6 = lin6 = NULL;
//ScenSim-Port//#endif
//ScenSim-Port//    error = 0;
//ScenSim-Port//
//ScenSim-Port//    if (req->oldptr != NULL || req->oldlen != 0)
//ScenSim-Port//        return (EINVAL);
//ScenSim-Port//    if (req->newptr == NULL)
//ScenSim-Port//        return (EPERM);
//ScenSim-Port//    if (req->newlen < sizeof(addrs))
//ScenSim-Port//        return (ENOMEM);
//ScenSim-Port//    error = SYSCTL_IN(req, &addrs, sizeof(addrs));
//ScenSim-Port//    if (error)
//ScenSim-Port//        return (error);
//ScenSim-Port//
//ScenSim-Port//    switch (addrs[0].ss_family) {
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//    case AF_INET6:
//ScenSim-Port//        fin6 = (struct sockaddr_in6 *)&addrs[0];
//ScenSim-Port//        lin6 = (struct sockaddr_in6 *)&addrs[1];
//ScenSim-Port//        if (fin6->sin6_len != sizeof(struct sockaddr_in6) ||
//ScenSim-Port//            lin6->sin6_len != sizeof(struct sockaddr_in6))
//ScenSim-Port//            return (EINVAL);
//ScenSim-Port//        if (IN6_IS_ADDR_V4MAPPED(&fin6->sin6_addr)) {
//ScenSim-Port//            if (!IN6_IS_ADDR_V4MAPPED(&lin6->sin6_addr))
//ScenSim-Port//                return (EINVAL);
//ScenSim-Port//            in6_sin6_2_sin_in_sock((struct sockaddr *)&addrs[0]);
//ScenSim-Port//            in6_sin6_2_sin_in_sock((struct sockaddr *)&addrs[1]);
//ScenSim-Port//            fin = (struct sockaddr_in *)&addrs[0];
//ScenSim-Port//            lin = (struct sockaddr_in *)&addrs[1];
//ScenSim-Port//            break;
//ScenSim-Port//        }
//ScenSim-Port//        error = sa6_embedscope(fin6, V_ip6_use_defzone);
//ScenSim-Port//        if (error)
//ScenSim-Port//            return (error);
//ScenSim-Port//        error = sa6_embedscope(lin6, V_ip6_use_defzone);
//ScenSim-Port//        if (error)
//ScenSim-Port//            return (error);
//ScenSim-Port//        break;
//ScenSim-Port//#endif
//ScenSim-Port//#ifdef INET
//ScenSim-Port//    case AF_INET:
//ScenSim-Port//        fin = (struct sockaddr_in *)&addrs[0];
//ScenSim-Port//        lin = (struct sockaddr_in *)&addrs[1];
//ScenSim-Port//        if (fin->sin_len != sizeof(struct sockaddr_in) ||
//ScenSim-Port//            lin->sin_len != sizeof(struct sockaddr_in))
//ScenSim-Port//            return (EINVAL);
//ScenSim-Port//        break;
//ScenSim-Port//#endif
//ScenSim-Port//    default:
//ScenSim-Port//        return (EINVAL);
//ScenSim-Port//    }
//ScenSim-Port//    INP_INFO_WLOCK(&V_tcbinfo);
//ScenSim-Port//    switch (addrs[0].ss_family) {
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//    case AF_INET6:
//ScenSim-Port//        inp = in6_pcblookup(&V_tcbinfo, &fin6->sin6_addr,
//ScenSim-Port//            fin6->sin6_port, &lin6->sin6_addr, lin6->sin6_port,
//ScenSim-Port//            INPLOOKUP_WLOCKPCB, NULL);
//ScenSim-Port//        break;
//ScenSim-Port//#endif
//ScenSim-Port//#ifdef INET
//ScenSim-Port//    case AF_INET:
//ScenSim-Port//        inp = in_pcblookup(&V_tcbinfo, fin->sin_addr, fin->sin_port,
//ScenSim-Port//            lin->sin_addr, lin->sin_port, INPLOOKUP_WLOCKPCB, NULL);
//ScenSim-Port//        break;
//ScenSim-Port//#endif
//ScenSim-Port//    }
//ScenSim-Port//    if (inp != NULL) {
//ScenSim-Port//        if (inp->inp_flags & INP_TIMEWAIT) {
//ScenSim-Port//            /*
//ScenSim-Port//             * XXXRW: There currently exists a state where an
//ScenSim-Port//             * inpcb is present, but its timewait state has been
//ScenSim-Port//             * discarded.  For now, don't allow dropping of this
//ScenSim-Port//             * type of inpcb.
//ScenSim-Port//             */
//ScenSim-Port//            tw = intotw(inp);
//ScenSim-Port//            if (tw != NULL)
//ScenSim-Port//                tcp_twclose(tw, 0);
//ScenSim-Port//            else
//ScenSim-Port//                INP_WUNLOCK(inp);
//ScenSim-Port//        } else if (!(inp->inp_flags & INP_DROPPED) &&
//ScenSim-Port//               !(inp->inp_socket->so_options & SO_ACCEPTCONN)) {
//ScenSim-Port//            tp = intotcpcb(inp);
//ScenSim-Port//            tp = tcp_drop(tp, ECONNABORTED);
//ScenSim-Port//            if (tp != NULL)
//ScenSim-Port//                INP_WUNLOCK(inp);
//ScenSim-Port//        } else
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//    } else
//ScenSim-Port//        error = ESRCH;
//ScenSim-Port//    INP_INFO_WUNLOCK(&V_tcbinfo);
//ScenSim-Port//    return (error);
//ScenSim-Port//}

//ScenSim-Port//SYSCTL_PROC(_net_inet_tcp, TCPCTL_DROP, drop,
//ScenSim-Port//    CTLTYPE_STRUCT|CTLFLAG_WR|CTLFLAG_SKIP, NULL,
//ScenSim-Port//    0, sysctl_drop, "", "Drop TCP connection");

/*
 * Generate a standardized TCP log line for use throughout the
 * tcp subsystem.  Memory allocation is done with M_NOWAIT to
 * allow use in the interrupt context.
 *
 * NB: The caller MUST free(s, M_TCPLOG) the returned string.
 * NB: The function may return NULL if memory allocation failed.
 *
 * Due to header inclusion and ordering limitations the struct ip
 * and ip6_hdr pointers have to be passed as void pointers.
 */
char *
tcp_log_vain(struct in_conninfo *inc, struct tcphdr *th, void *ip4hdr,
    const void *ip6hdr)
{

    /* Is logging enabled? */
    if (tcp_log_in_vain == 0)
        return (NULL);

    return (tcp_log_addr(inc, th, ip4hdr, ip6hdr));
}

char *
tcp_log_addrs(struct in_conninfo *inc, struct tcphdr *th, void *ip4hdr,
    const void *ip6hdr)
{

    /* Is logging enabled? */
    if (tcp_log_debug == 0)
        return (NULL);

    return (tcp_log_addr(inc, th, ip4hdr, ip6hdr));
}

static char *
tcp_log_addr(struct in_conninfo *inc, struct tcphdr *th, void *ip4hdr,
    const void *ip6hdr)
{
    char *s, *sp;
    size_t size;
    struct ip *ip;
#ifdef INET6
    const struct ip6_hdr *ip6;

    ip6 = (const struct ip6_hdr *)ip6hdr;
#endif /* INET6 */
    ip = (struct ip *)ip4hdr;

    /*
     * The log line looks like this:
     * "TCP: [1.2.3.4]:50332 to [1.2.3.4]:80 tcpflags 0x2<SYN>"
     */
    size = sizeof("TCP: []:12345 to []:12345 tcpflags 0x2<>") +
        sizeof(PRINT_TH_FLAGS) + 1 +
#ifdef INET6
        2 * INET6_ADDRSTRLEN;
#else
        2 * INET_ADDRSTRLEN;
#endif /* INET6 */

//ScenSim-Port//    s = malloc(size, M_TCPLOG, M_ZERO|M_NOWAIT);
    s = (char *)malloc(size, M_TCPLOG, M_ZERO|M_NOWAIT);        //ScenSim-Port//
    if (s == NULL)
        return (NULL);

    strcat(s, "TCP: [");
    sp = s + strlen(s);

    if (inc && ((inc->inc_flags & INC_ISIPV6) == 0)) {
        inet_ntoa_r(inc->inc_faddr, sp);
        sp = s + strlen(s);
        sprintf(sp, "]:%i to [", ntohs(inc->inc_fport));
        sp = s + strlen(s);
        inet_ntoa_r(inc->inc_laddr, sp);
        sp = s + strlen(s);
        sprintf(sp, "]:%i", ntohs(inc->inc_lport));
#ifdef INET6
    } else if (inc) {
        ip6_sprintf(sp, &inc->inc6_faddr);
        sp = s + strlen(s);
        sprintf(sp, "]:%i to [", ntohs(inc->inc_fport));
        sp = s + strlen(s);
        ip6_sprintf(sp, &inc->inc6_laddr);
        sp = s + strlen(s);
        sprintf(sp, "]:%i", ntohs(inc->inc_lport));
    } else if (ip6 && th) {
        ip6_sprintf(sp, &ip6->ip6_src);
        sp = s + strlen(s);
        sprintf(sp, "]:%i to [", ntohs(th->th_sport));
        sp = s + strlen(s);
        ip6_sprintf(sp, &ip6->ip6_dst);
        sp = s + strlen(s);
        sprintf(sp, "]:%i", ntohs(th->th_dport));
#endif /* INET6 */
#ifdef INET
    } else if (ip && th) {
        inet_ntoa_r(ip->ip_src, sp);
        sp = s + strlen(s);
        sprintf(sp, "]:%i to [", ntohs(th->th_sport));
        sp = s + strlen(s);
        inet_ntoa_r(ip->ip_dst, sp);
        sp = s + strlen(s);
        sprintf(sp, "]:%i", ntohs(th->th_dport));
#endif /* INET */
    } else {
        free(s, M_TCPLOG);
        return (NULL);
    }
    sp = s + strlen(s);
    if (th)
//ScenSim-Port//        sprintf(sp, " tcpflags 0x%b", th->th_flags, PRINT_TH_FLAGS);
#pragma clang diagnostic push //ScenSim-Port//
#pragma clang diagnostic ignored "-Wformat-extra-args" //ScenSim-Port//
        sprintf(sp, " tcpflags 0x%i",                           //ScenSim-Port//
            th->th_flags, PRINT_TH_FLAGS);                      //ScenSim-Port//
#pragma clang diagnostic pop //ScenSim-Port//
    if (*(s + size - 1) != '\0')
        panic("%s: string too long", __func__);
    return (s);
}
}//namespace//                                                  //ScenSim-Port//
