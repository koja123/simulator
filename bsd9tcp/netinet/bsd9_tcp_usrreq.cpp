/*-
 * Copyright (c) 1982, 1986, 1988, 1993
 *  The Regents of the University of California.
 * Copyright (c) 2006-2007 Robert N. M. Watson
 * Copyright (c) 2010-2011 Juniper Networks, Inc.
 * All rights reserved.
 *
 * Portions of this software were developed by Robert N. M. Watson under
 * contract to Juniper Networks, Inc.
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
 *  From: @(#)tcp_usrreq.c  8.2 (Berkeley) 1/3/94
 */

//ScenSim-Port//#include <sys/cdefs.h>
//ScenSim-Port//__FBSDID("$FreeBSD$");

//ScenSim-Port//#include "opt_ddb.h"
//ScenSim-Port//#include "opt_inet.h"
//ScenSim-Port//#include "opt_inet6.h"
//ScenSim-Port//#include "opt_tcpdebug.h"

//ScenSim-Port//#include <sys/param.h>
//ScenSim-Port//#include <sys/systm.h>
//ScenSim-Port//#include <sys/malloc.h>
//ScenSim-Port//#include <sys/kernel.h>
//ScenSim-Port//#include <sys/sysctl.h>
//ScenSim-Port//#include <sys/mbuf.h>
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//#include <sys/domain.h>
//ScenSim-Port//#endif /* INET6 */
//ScenSim-Port//#include <sys/socket.h>
//ScenSim-Port//#include <sys/socketvar.h>
//ScenSim-Port//#include <sys/protosw.h>
//ScenSim-Port//#include <sys/proc.h>
//ScenSim-Port//#include <sys/jail.h>

//ScenSim-Port//#ifdef DDB
//ScenSim-Port//#include <ddb/ddb.h>
//ScenSim-Port//#endif

//ScenSim-Port//#include <net/if.h>
//ScenSim-Port//#include <net/route.h>
//ScenSim-Port//#include <net/vnet.h>

//ScenSim-Port//#include <netinet/cc.h>
//ScenSim-Port//#include <netinet/in.h>
//ScenSim-Port//#include <netinet/in_pcb.h>
//ScenSim-Port//#include <netinet/in_systm.h>
//ScenSim-Port//#include <netinet/in_var.h>
//ScenSim-Port//#include <netinet/ip_var.h>
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//#include <netinet/ip6.h>
//ScenSim-Port//#include <netinet6/in6_pcb.h>
//ScenSim-Port//#include <netinet6/ip6_var.h>
//ScenSim-Port//#include <netinet6/scope6_var.h>
//ScenSim-Port//#endif
//ScenSim-Port//#include <netinet/tcp_fsm.h>
//ScenSim-Port//#include <netinet/tcp_seq.h>
//ScenSim-Port//#include <netinet/tcp_timer.h>
//ScenSim-Port//#include <netinet/tcp_var.h>
//ScenSim-Port//#include <netinet/tcpip.h>
//ScenSim-Port//#ifdef TCPDEBUG
//ScenSim-Port//#include <netinet/tcp_debug.h>
//ScenSim-Port//#endif
//ScenSim-Port//#include <netinet/tcp_offload.h>
#include "tcp_porting.h"                        //ScenSim-Port//
namespace FreeBsd9Port {                                        //ScenSim-Port//

/*
 * TCP protocol interface to socket abstraction.
 */
static int  tcp_attach(struct socket *);
#ifdef INET
static int  tcp_connect(struct tcpcb *, struct sockaddr *,
            struct thread *td);
#endif /* INET */
#ifdef INET6
static int  tcp6_connect(struct tcpcb *, struct sockaddr *,
            struct thread *td);
#endif /* INET6 */
static void tcp_disconnect(struct tcpcb *);
static void tcp_usrclosed(struct tcpcb *);
//ScenSim-Port//static void tcp_fill_info(struct tcpcb *, struct tcp_info *);

//ScenSim-Port//#ifdef TCPDEBUG
//ScenSim-Port//#define TCPDEBUG0   int ostate = 0
//ScenSim-Port//#define TCPDEBUG1() ostate = tp ? tp->t_state : 0
//ScenSim-Port//#define TCPDEBUG2(req)  if (tp && (so->so_options & SO_DEBUG)) \
//ScenSim-Port//                tcp_trace(TA_USER, ostate, tp, 0, 0, req)
//ScenSim-Port//#else
//ScenSim-Port//#define TCPDEBUG0
//ScenSim-Port//#define TCPDEBUG1()
//ScenSim-Port//#define TCPDEBUG2(req)
//ScenSim-Port//#endif

/*
 * TCP attaches to socket via pru_attach(), reserving space,
 * and an internet control block.
 */
static int
tcp_usr_attach(struct socket *so, int proto, struct thread *td)
{
//ScenSim-Port//    struct inpcb *inp;
//ScenSim-Port//    struct tcpcb *tp = NULL;
    int error;
//ScenSim-Port//    TCPDEBUG0;

//ScenSim-Port//    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp == NULL, ("tcp_usr_attach: inp != NULL"));
//ScenSim-Port//    TCPDEBUG1();

    error = tcp_attach(so);
//ScenSim-Port//    if (error)
//ScenSim-Port//        goto out;

//ScenSim-Port//    if ((so->so_options & SO_LINGER) && so->so_linger == 0)
//ScenSim-Port//        so->so_linger = TCP_LINGERTIME;

//ScenSim-Port//    inp = sotoinpcb(so);
//ScenSim-Port//    tp = intotcpcb(inp);
//ScenSim-Port//out:
//ScenSim-Port//    TCPDEBUG2(PRU_ATTACH);
    return error;
}

/*
 * tcp_detach is called when the socket layer loses its final reference
 * to the socket, be it a file descriptor reference, a reference from TCP,
 * etc.  At this point, there is only one case in which we will keep around
 * inpcb state: time wait.
 *
 * This function can probably be re-absorbed back into tcp_usr_detach() now
 * that there is a single detach path.
 */
static void
tcp_detach(struct socket *so, struct inpcb *inp)
{
    struct tcpcb *tp;

//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK_ASSERT(inp);

//ScenSim-Port//    KASSERT(so->so_pcb == inp, ("tcp_detach: so_pcb != inp"));
//ScenSim-Port//    KASSERT(inp->inp_socket == so, ("tcp_detach: inp_socket != so"));

    tp = intotcpcb(inp);

    if (inp->inp_flags & INP_TIMEWAIT) {
        /*
         * There are two cases to handle: one in which the time wait
         * state is being discarded (INP_DROPPED), and one in which
         * this connection will remain in timewait.  In the former,
         * it is time to discard all state (except tcptw, which has
         * already been discarded by the timewait close code, which
         * should be further up the call stack somewhere).  In the
         * latter case, we detach from the socket, but leave the pcb
         * present until timewait ends.
         *
         * XXXRW: Would it be cleaner to free the tcptw here?
         */
        if (inp->inp_flags & INP_DROPPED) {
//ScenSim-Port//            KASSERT(tp == NULL, ("tcp_detach: INP_TIMEWAIT && "
//ScenSim-Port//                "INP_DROPPED && tp != NULL"));
            in_pcbdetach(inp);
            in_pcbfree(inp);
        } else {
            in_pcbdetach(inp);
//ScenSim-Port//            INP_WUNLOCK(inp);
        }
    } else {
        /*
         * If the connection is not in timewait, we consider two
         * two conditions: one in which no further processing is
         * necessary (dropped || embryonic), and one in which TCP is
         * not yet done, but no longer requires the socket, so the
         * pcb will persist for the time being.
         *
         * XXXRW: Does the second case still occur?
         */
        if (inp->inp_flags & INP_DROPPED ||
            tp->t_state < TCPS_SYN_SENT) {
            tcp_discardcb(tp);
            in_pcbdetach(inp);
            in_pcbfree(inp);
        } else
            in_pcbdetach(inp);
    }
}

/*
 * pru_detach() detaches the TCP protocol from the socket.
 * If the protocol state is non-embryonic, then can't
 * do this directly: have to initiate a pru_disconnect(),
 * which may finish later; embryonic TCB's can just
 * be discarded here.
 */
static void
tcp_usr_detach(struct socket *so)
{
    struct inpcb *inp;

    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp_usr_detach: inp == NULL"));
//ScenSim-Port//    INP_INFO_WLOCK(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK(inp);
//ScenSim-Port//    KASSERT(inp->inp_socket != NULL,
//ScenSim-Port//        ("tcp_usr_detach: inp_socket == NULL"));
    tcp_detach(so, inp);
//ScenSim-Port//    INP_INFO_WUNLOCK(&V_tcbinfo);
}

#ifdef INET
/*
 * Give the socket an address.
 */
static int
tcp_usr_bind(struct socket *so, struct sockaddr *nam, struct thread *td)
{
    int error = 0;
    struct inpcb *inp;
//ScenSim-Port//    struct tcpcb *tp = NULL;
    struct sockaddr_in *sinp;

    sinp = (struct sockaddr_in *)nam;
    if (nam->sa_len != sizeof (*sinp))
        return (EINVAL);
    /*
     * Must check for multicast addresses and disallow binding
     * to them.
     */
    if (sinp->sin_family == AF_INET &&
        IN_MULTICAST(ntohl(sinp->sin_addr.s_addr)))
        return (EAFNOSUPPORT);

//ScenSim-Port//    TCPDEBUG0;
    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp_usr_bind: inp == NULL"));
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        error = EINVAL;
        goto out;
    }
//ScenSim-Port//    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();
//ScenSim-Port//    INP_HASH_WLOCK(&V_tcbinfo);
    error = in_pcbbind(inp, nam, td->td_ucred);
//ScenSim-Port//    INP_HASH_WUNLOCK(&V_tcbinfo);
out:
//ScenSim-Port//    TCPDEBUG2(PRU_BIND);
//ScenSim-Port//    INP_WUNLOCK(inp);

    return (error);
}
#endif /* INET */

#ifdef INET6
static int
tcp6_usr_bind(struct socket *so, struct sockaddr *nam, struct thread *td)
{
    int error = 0;
    struct inpcb *inp;
//ScenSim-Port//    struct tcpcb *tp = NULL;
    struct sockaddr_in6 *sin6p;

    sin6p = (struct sockaddr_in6 *)nam;
    if (nam->sa_len != sizeof (*sin6p))
        return (EINVAL);
    /*
     * Must check for multicast addresses and disallow binding
     * to them.
     */
    if (sin6p->sin6_family == AF_INET6 &&
        IN6_IS_ADDR_MULTICAST(&sin6p->sin6_addr))
        return (EAFNOSUPPORT);

//ScenSim-Port//    TCPDEBUG0;
    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp6_usr_bind: inp == NULL"));
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        error = EINVAL;
        goto out;
    }
//ScenSim-Port//    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();
//ScenSim-Port//    INP_HASH_WLOCK(&V_tcbinfo);
    inp->inp_vflag &= ~INP_IPV4;
    inp->inp_vflag |= INP_IPV6;
#ifdef INET
    if ((inp->inp_flags & IN6P_IPV6_V6ONLY) == 0) {
        if (IN6_IS_ADDR_UNSPECIFIED(&sin6p->sin6_addr))
            inp->inp_vflag |= INP_IPV4;
        else if (IN6_IS_ADDR_V4MAPPED(&sin6p->sin6_addr)) {
            struct sockaddr_in sin;

            in6_sin6_2_sin(&sin, sin6p);
            inp->inp_vflag |= INP_IPV4;
            inp->inp_vflag &= ~INP_IPV6;
            error = in_pcbbind(inp, (struct sockaddr *)&sin,
                td->td_ucred);
//ScenSim-Port//            INP_HASH_WUNLOCK(&V_tcbinfo);
            goto out;
        }
    }
#endif
    error = in6_pcbbind(inp, nam, td->td_ucred);
//ScenSim-Port//    INP_HASH_WUNLOCK(&V_tcbinfo);
out:
//ScenSim-Port//    TCPDEBUG2(PRU_BIND);
//ScenSim-Port//    INP_WUNLOCK(inp);
    return (error);
}
#endif /* INET6 */

#ifdef INET
/*
 * Prepare to accept connections.
 */
static int
tcp_usr_listen(struct socket *so, int backlog, struct thread *td)
{
    int error = 0;
    struct inpcb *inp;
    struct tcpcb *tp = NULL;

//ScenSim-Port//    TCPDEBUG0;
    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp_usr_listen: inp == NULL"));
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        error = EINVAL;
        goto out;
    }
    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();
//ScenSim-Port//    SOCK_LOCK(so);
    error = solisten_proto_check(so);
//ScenSim-Port//    INP_HASH_WLOCK(&V_tcbinfo);
    if (error == 0 && inp->inp_lport == 0)
        error = in_pcbbind(inp, (struct sockaddr *)0, td->td_ucred);
//ScenSim-Port//    INP_HASH_WUNLOCK(&V_tcbinfo);
    if (error == 0) {
        tp->t_state = TCPS_LISTEN;
        solisten_proto(so, backlog);
//ScenSim-Port//        tcp_offload_listen_open(tp);
    }
//ScenSim-Port//    SOCK_UNLOCK(so);

out:
//ScenSim-Port//    TCPDEBUG2(PRU_LISTEN);
//ScenSim-Port//    INP_WUNLOCK(inp);
    return (error);
}
#endif /* INET */

#ifdef INET6
static int
tcp6_usr_listen(struct socket *so, int backlog, struct thread *td)
{
    int error = 0;
    struct inpcb *inp;
    struct tcpcb *tp = NULL;

//ScenSim-Port//    TCPDEBUG0;
    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp6_usr_listen: inp == NULL"));
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        error = EINVAL;
        goto out;
    }
    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();
//ScenSim-Port//    SOCK_LOCK(so);
    error = solisten_proto_check(so);
//ScenSim-Port//    INP_HASH_WLOCK(&V_tcbinfo);
    if (error == 0 && inp->inp_lport == 0) {
        inp->inp_vflag &= ~INP_IPV4;
        if ((inp->inp_flags & IN6P_IPV6_V6ONLY) == 0)
            inp->inp_vflag |= INP_IPV4;
        error = in6_pcbbind(inp, (struct sockaddr *)0, td->td_ucred);
    }
//ScenSim-Port//    INP_HASH_WUNLOCK(&V_tcbinfo);
    if (error == 0) {
        tp->t_state = TCPS_LISTEN;
        solisten_proto(so, backlog);
    }
//ScenSim-Port//    SOCK_UNLOCK(so);

out:
//ScenSim-Port//    TCPDEBUG2(PRU_LISTEN);
//ScenSim-Port//    INP_WUNLOCK(inp);
    return (error);
}
#endif /* INET6 */

#ifdef INET
/*
 * Initiate connection to peer.
 * Create a template for use in transmissions on this connection.
 * Enter SYN_SENT state, and mark socket as connecting.
 * Start keep-alive timer, and seed output sequence space.
 * Send initial segment on connection.
 */
static int
tcp_usr_connect(struct socket *so, struct sockaddr *nam, struct thread *td)
{
    int error = 0;
    struct inpcb *inp;
    struct tcpcb *tp = NULL;
    struct sockaddr_in *sinp;

    sinp = (struct sockaddr_in *)nam;
    if (nam->sa_len != sizeof (*sinp))
        return (EINVAL);
    /*
     * Must disallow TCP ``connections'' to multicast addresses.
     */
    if (sinp->sin_family == AF_INET
        && IN_MULTICAST(ntohl(sinp->sin_addr.s_addr)))
        return (EAFNOSUPPORT);
//ScenSim-Port//    if ((error = prison_remote_ip4(td->td_ucred, &sinp->sin_addr)) != 0)
//ScenSim-Port//        return (error);

//ScenSim-Port//    TCPDEBUG0;
    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp_usr_connect: inp == NULL"));
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        error = EINVAL;
        goto out;
    }
    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();
    if ((error = tcp_connect(tp, nam, td)) != 0)
        goto out;
    error = tcp_output_connect(so, nam);
out:
//ScenSim-Port//    TCPDEBUG2(PRU_CONNECT);
//ScenSim-Port//    INP_WUNLOCK(inp);
    return (error);
}
#endif /* INET */

#ifdef INET6
static int
tcp6_usr_connect(struct socket *so, struct sockaddr *nam, struct thread *td)
{
    int error = 0;
    struct inpcb *inp;
    struct tcpcb *tp = NULL;
    struct sockaddr_in6 *sin6p;

//ScenSim-Port//    TCPDEBUG0;

    sin6p = (struct sockaddr_in6 *)nam;
    if (nam->sa_len != sizeof (*sin6p))
        return (EINVAL);
    /*
     * Must disallow TCP ``connections'' to multicast addresses.
     */
    if (sin6p->sin6_family == AF_INET6
        && IN6_IS_ADDR_MULTICAST(&sin6p->sin6_addr))
        return (EAFNOSUPPORT);

    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp6_usr_connect: inp == NULL"));
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        error = EINVAL;
        goto out;
    }
    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();
#ifdef INET
    /*
     * XXXRW: Some confusion: V4/V6 flags relate to binding, and
     * therefore probably require the hash lock, which isn't held here.
     * Is this a significant problem?
     */
    if (IN6_IS_ADDR_V4MAPPED(&sin6p->sin6_addr)) {
        struct sockaddr_in sin;

        if ((inp->inp_flags & IN6P_IPV6_V6ONLY) != 0) {
            error = EINVAL;
            goto out;
        }

        in6_sin6_2_sin(&sin, sin6p);
        inp->inp_vflag |= INP_IPV4;
        inp->inp_vflag &= ~INP_IPV6;
//ScenSim-Port//        if ((error = prison_remote_ip4(td->td_ucred,
//ScenSim-Port//            &sin.sin_addr)) != 0)
//ScenSim-Port//            goto out;
        if ((error = tcp_connect(tp, (struct sockaddr *)&sin, td)) != 0)
            goto out;
        error = tcp_output_connect(so, nam);
        goto out;
    }
#endif
    inp->inp_vflag &= ~INP_IPV4;
    inp->inp_vflag |= INP_IPV6;
    inp->inp_inc.inc_flags |= INC_ISIPV6;
//ScenSim-Port//    if ((error = prison_remote_ip6(td->td_ucred, &sin6p->sin6_addr)) != 0)
//ScenSim-Port//        goto out;
    if ((error = tcp6_connect(tp, nam, td)) != 0)
        goto out;
    error = tcp_output_connect(so, nam);

out:
//ScenSim-Port//    TCPDEBUG2(PRU_CONNECT);
//ScenSim-Port//    INP_WUNLOCK(inp);
    return (error);
}
#endif /* INET6 */

/*
 * Initiate disconnect from peer.
 * If connection never passed embryonic stage, just drop;
 * else if don't need to let data drain, then can just drop anyways,
 * else have to begin TCP shutdown process: mark socket disconnecting,
 * drain unread data, state switch to reflect user close, and
 * send segment (e.g. FIN) to peer.  Socket will be really disconnected
 * when peer sends FIN and acks ours.
 *
 * SHOULD IMPLEMENT LATER PRU_CONNECT VIA REALLOC TCPCB.
 */
static int
tcp_usr_disconnect(struct socket *so)
{
    struct inpcb *inp;
    struct tcpcb *tp = NULL;
    int error = 0;

//ScenSim-Port//    TCPDEBUG0;
//ScenSim-Port//    INP_INFO_WLOCK(&V_tcbinfo);
    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp_usr_disconnect: inp == NULL"));
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        error = ECONNRESET;
        goto out;
    }
    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();
    tcp_disconnect(tp);
out:
//ScenSim-Port//    TCPDEBUG2(PRU_DISCONNECT);
//ScenSim-Port//    INP_WUNLOCK(inp);
//ScenSim-Port//    INP_INFO_WUNLOCK(&V_tcbinfo);
    return (error);
}

#ifdef INET
/*
 * Accept a connection.  Essentially all the work is done at higher levels;
 * just return the address of the peer, storing through addr.
 *
 * The rationale for acquiring the tcbinfo lock here is somewhat complicated,
 * and is described in detail in the commit log entry for r175612.  Acquiring
 * it delays an accept(2) racing with sonewconn(), which inserts the socket
 * before the inpcb address/port fields are initialized.  A better fix would
 * prevent the socket from being placed in the listen queue until all fields
 * are fully initialized.
 */
static int
tcp_usr_accept(struct socket *so, struct sockaddr **nam)
{
    int error = 0;
    struct inpcb *inp = NULL;
//ScenSim-Port//    struct tcpcb *tp = NULL;
//ScenSim-Port//    struct in_addr addr;
//ScenSim-Port//    in_port_t port = 0;
//ScenSim-Port//    TCPDEBUG0;

    if (so->so_state & SS_ISDISCONNECTED)
        return (ECONNABORTED);

    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp_usr_accept: inp == NULL"));
//ScenSim-Port//    INP_INFO_RLOCK(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        error = ECONNABORTED;
        goto out;
    }
//ScenSim-Port//    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();

    /*
     * We inline in_getpeeraddr and COMMON_END here, so that we can
     * copy the data of interest and defer the malloc until after we
     * release the lock.
     */
//ScenSim-Port//    port = inp->inp_fport;
//ScenSim-Port//    addr = inp->inp_faddr;

out:
//ScenSim-Port//    TCPDEBUG2(PRU_ACCEPT);
//ScenSim-Port//    INP_WUNLOCK(inp);
//ScenSim-Port//    INP_INFO_RUNLOCK(&V_tcbinfo);
//ScenSim-Port//    if (error == 0)
//ScenSim-Port//        *nam = in_sockaddr(port, &addr);
    return error;
}
#endif /* INET */

#ifdef INET6
static int
tcp6_usr_accept(struct socket *so, struct sockaddr **nam)
{
    struct inpcb *inp = NULL;
    int error = 0;
//ScenSim-Port//    struct tcpcb *tp = NULL;
//ScenSim-Port//    struct in_addr addr;
//ScenSim-Port//    struct in6_addr addr6;
//ScenSim-Port//    in_port_t port = 0;
//ScenSim-Port//    int v4 = 0;
//ScenSim-Port//    TCPDEBUG0;

    if (so->so_state & SS_ISDISCONNECTED)
        return (ECONNABORTED);

    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp6_usr_accept: inp == NULL"));
//ScenSim-Port//    INP_INFO_RLOCK(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        error = ECONNABORTED;
        goto out;
    }
//ScenSim-Port//    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();

    /*
     * We inline in6_mapped_peeraddr and COMMON_END here, so that we can
     * copy the data of interest and defer the malloc until after we
     * release the lock.
     */
//ScenSim-Port//    if (inp->inp_vflag & INP_IPV4) {
//ScenSim-Port//        v4 = 1;
//ScenSim-Port//        port = inp->inp_fport;
//ScenSim-Port//        addr = inp->inp_faddr;
//ScenSim-Port//    } else {
//ScenSim-Port//        port = inp->inp_fport;
//ScenSim-Port//        addr6 = inp->in6p_faddr;
//ScenSim-Port//    }

out:
//ScenSim-Port//    TCPDEBUG2(PRU_ACCEPT);
//ScenSim-Port//    INP_WUNLOCK(inp);
//ScenSim-Port//    INP_INFO_RUNLOCK(&V_tcbinfo);
//ScenSim-Port//    if (error == 0) {
//ScenSim-Port//        if (v4)
//ScenSim-Port//            *nam = in6_v4mapsin6_sockaddr(port, &addr);
//ScenSim-Port//        else
//ScenSim-Port//            *nam = in6_sockaddr(port, &addr6);
//ScenSim-Port//    }
    return error;
}
#endif /* INET6 */

/*
 * Mark the connection as being incapable of further output.
 */
static int
tcp_usr_shutdown(struct socket *so)
{
    int error = 0;
    struct inpcb *inp;
    struct tcpcb *tp = NULL;

//ScenSim-Port//    TCPDEBUG0;
//ScenSim-Port//    INP_INFO_WLOCK(&V_tcbinfo);
    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("inp == NULL"));
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        error = ECONNRESET;
        goto out;
    }
    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();
    socantsendmore(so);
    tcp_usrclosed(tp);
    if (!(inp->inp_flags & INP_DROPPED))
        error = tcp_output_disconnect(tp);

out:
//ScenSim-Port//    TCPDEBUG2(PRU_SHUTDOWN);
//ScenSim-Port//    INP_WUNLOCK(inp);
//ScenSim-Port//    INP_INFO_WUNLOCK(&V_tcbinfo);

    return (error);
}

/*
 * After a receive, possibly send window update to peer.
 */
static int
tcp_usr_rcvd(struct socket *so, int flags)
{
    struct inpcb *inp;
    struct tcpcb *tp = NULL;
    int error = 0;

//ScenSim-Port//    TCPDEBUG0;
    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp_usr_rcvd: inp == NULL"));
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        error = ECONNRESET;
        goto out;
    }
    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();
    tcp_output_rcvd(tp);

out:
//ScenSim-Port//    TCPDEBUG2(PRU_RCVD);
//ScenSim-Port//    INP_WUNLOCK(inp);
    return (error);
}

/*
 * Do a send by putting data in output queue and updating urgent
 * marker if URG set.  Possibly send more data.  Unlike the other
 * pru_*() routines, the mbuf chains are our responsibility.  We
 * must either enqueue them or free them.  The other pru_* routines
 * generally are caller-frees.
 */
static int
tcp_usr_send(struct socket *so, int flags, struct mbuf *m,
    struct sockaddr *nam, struct mbuf *control, struct thread *td)
{
    int error = 0;
    struct inpcb *inp;
    struct tcpcb *tp = NULL;
#ifdef INET6
    int isipv6;
#endif
//ScenSim-Port//    TCPDEBUG0;

    /*
     * We require the pcbinfo lock if we will close the socket as part of
     * this call.
     */
//ScenSim-Port//    if (flags & PRUS_EOF)
//ScenSim-Port//        INP_INFO_WLOCK(&V_tcbinfo);
    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp_usr_send: inp == NULL"));
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        if (control)
            m_freem(control);
        if (m)
            m_freem(m);
        error = ECONNRESET;
        goto out;
    }
#ifdef INET6
    isipv6 = nam && nam->sa_family == AF_INET6;
#endif /* INET6 */
    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();
    if (control) {
        /* TCP doesn't do control messages (rights, creds, etc) */
        if (control->m_len) {
            m_freem(control);
            if (m)
                m_freem(m);
            error = EINVAL;
            goto out;
        }
        m_freem(control);   /* empty control, just free it */
    }
    if (!(flags & PRUS_OOB)) {
        sbappendstream(&so->so_snd, m);
        if (nam && tp->t_state < TCPS_SYN_SENT) {
            /*
             * Do implied connect if not yet connected,
             * initialize window to default value, and
             * initialize maxseg/maxopd using peer's cached
             * MSS.
             */
#ifdef INET6
            if (isipv6)
                error = tcp6_connect(tp, nam, td);
#endif /* INET6 */
#if defined(INET6) && defined(INET)
            else
#endif
#ifdef INET
                error = tcp_connect(tp, nam, td);
#endif
            if (error)
                goto out;
            tp->snd_wnd = TTCP_CLIENT_SND_WND;
            tcp_mss(tp, -1);
        }
        if (flags & PRUS_EOF) {
            /*
             * Close the send side of the connection after
             * the data is sent.
             */
//ScenSim-Port//            INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
            socantsendmore(so);
            tcp_usrclosed(tp);
        }
        if (!(inp->inp_flags & INP_DROPPED)) {
            if (flags & PRUS_MORETOCOME)
                tp->t_flags |= TF_MORETOCOME;
            error = tcp_output_send(tp);
            if (flags & PRUS_MORETOCOME)
                tp->t_flags &= ~TF_MORETOCOME;
        }
    } else {
        /*
         * XXXRW: PRUS_EOF not implemented with PRUS_OOB?
         */
//ScenSim-Port//        SOCKBUF_LOCK(&so->so_snd);
        if (sbspace(&so->so_snd) < -512) {
//ScenSim-Port//            SOCKBUF_UNLOCK(&so->so_snd);
            m_freem(m);
            error = ENOBUFS;
            goto out;
        }
        /*
         * According to RFC961 (Assigned Protocols),
         * the urgent pointer points to the last octet
         * of urgent data.  We continue, however,
         * to consider it to indicate the first octet
         * of data past the urgent section.
         * Otherwise, snd_up should be one lower.
         */
        sbappendstream_locked(&so->so_snd, m);
//ScenSim-Port//        SOCKBUF_UNLOCK(&so->so_snd);
        if (nam && tp->t_state < TCPS_SYN_SENT) {
            /*
             * Do implied connect if not yet connected,
             * initialize window to default value, and
             * initialize maxseg/maxopd using peer's cached
             * MSS.
             */
#ifdef INET6
            if (isipv6)
                error = tcp6_connect(tp, nam, td);
#endif /* INET6 */
#if defined(INET6) && defined(INET)
            else
#endif
#ifdef INET
                error = tcp_connect(tp, nam, td);
#endif
            if (error)
                goto out;
            tp->snd_wnd = TTCP_CLIENT_SND_WND;
            tcp_mss(tp, -1);
        }
        tp->snd_up = tp->snd_una + so->so_snd.sb_cc;
        tp->t_flags |= TF_FORCEDATA;
        error = tcp_output_send(tp);
        tp->t_flags &= ~TF_FORCEDATA;
    }
out:
//ScenSim-Port//    TCPDEBUG2((flags & PRUS_OOB) ? PRU_SENDOOB :
//ScenSim-Port//          ((flags & PRUS_EOF) ? PRU_SEND_EOF : PRU_SEND));
//ScenSim-Port//    INP_WUNLOCK(inp);
//ScenSim-Port//    if (flags & PRUS_EOF)
//ScenSim-Port//        INP_INFO_WUNLOCK(&V_tcbinfo);
    return (error);
}

/*
 * Abort the TCP.  Drop the connection abruptly.
 */
static void
tcp_usr_abort(struct socket *so)
{
    struct inpcb *inp;
    struct tcpcb *tp = NULL;
//ScenSim-Port//    TCPDEBUG0;

    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp_usr_abort: inp == NULL"));

//ScenSim-Port//    INP_INFO_WLOCK(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK(inp);
//ScenSim-Port//    KASSERT(inp->inp_socket != NULL,
//ScenSim-Port//        ("tcp_usr_abort: inp_socket == NULL"));

    /*
     * If we still have full TCP state, and we're not dropped, drop.
     */
    if (!(inp->inp_flags & INP_TIMEWAIT) &&
        !(inp->inp_flags & INP_DROPPED)) {
        tp = intotcpcb(inp);
//ScenSim-Port//        TCPDEBUG1();
        tcp_drop(tp, ECONNABORTED);
//ScenSim-Port//        TCPDEBUG2(PRU_ABORT);
    }
    if (!(inp->inp_flags & INP_DROPPED)) {
//ScenSim-Port//        SOCK_LOCK(so);
        so->so_state |= SS_PROTOREF;
//ScenSim-Port//        SOCK_UNLOCK(so);
        inp->inp_flags |= INP_SOCKREF;
    }
//ScenSim-Port//    INP_WUNLOCK(inp);
//ScenSim-Port//    INP_INFO_WUNLOCK(&V_tcbinfo);
}

/*
 * TCP socket is closed.  Start friendly disconnect.
 */
static void
tcp_usr_close(struct socket *so)
{
    struct inpcb *inp;
    struct tcpcb *tp = NULL;
//ScenSim-Port//    TCPDEBUG0;

    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp_usr_close: inp == NULL"));

//ScenSim-Port//    INP_INFO_WLOCK(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK(inp);
//ScenSim-Port//    KASSERT(inp->inp_socket != NULL,
//ScenSim-Port//        ("tcp_usr_close: inp_socket == NULL"));

    /*
     * If we still have full TCP state, and we're not dropped, initiate
     * a disconnect.
     */
    if (!(inp->inp_flags & INP_TIMEWAIT) &&
        !(inp->inp_flags & INP_DROPPED)) {
        tp = intotcpcb(inp);
//ScenSim-Port//        TCPDEBUG1();
        tcp_disconnect(tp);
//ScenSim-Port//        TCPDEBUG2(PRU_CLOSE);
    }
    if (!(inp->inp_flags & INP_DROPPED)) {
//ScenSim-Port//        SOCK_LOCK(so);
        so->so_state |= SS_PROTOREF;
//ScenSim-Port//        SOCK_UNLOCK(so);
        inp->inp_flags |= INP_SOCKREF;
    }
//ScenSim-Port//    INP_WUNLOCK(inp);
//ScenSim-Port//    INP_INFO_WUNLOCK(&V_tcbinfo);
}

/*
 * Receive out-of-band data.
 */
static int
tcp_usr_rcvoob(struct socket *so, struct mbuf *m, int flags)
{
    int error = 0;
    struct inpcb *inp;
    struct tcpcb *tp = NULL;

//ScenSim-Port//    TCPDEBUG0;
    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp_usr_rcvoob: inp == NULL"));
//ScenSim-Port//    INP_WLOCK(inp);
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
        error = ECONNRESET;
        goto out;
    }
    tp = intotcpcb(inp);
//ScenSim-Port//    TCPDEBUG1();
    if ((so->so_oobmark == 0 &&
         (so->so_rcv.sb_state & SBS_RCVATMARK) == 0) ||
        so->so_options & SO_OOBINLINE ||
        tp->t_oobflags & TCPOOB_HADDATA) {
        error = EINVAL;
        goto out;
    }
    if ((tp->t_oobflags & TCPOOB_HAVEDATA) == 0) {
        error = EWOULDBLOCK;
        goto out;
    }
    m->m_len = 1;
    *mtod(m, caddr_t) = tp->t_iobc;
    if ((flags & MSG_PEEK) == 0)
        tp->t_oobflags ^= (TCPOOB_HAVEDATA | TCPOOB_HADDATA);

out:
//ScenSim-Port//    TCPDEBUG2(PRU_RCVOOB);
//ScenSim-Port//    INP_WUNLOCK(inp);
    return (error);
}

#ifdef INET
struct pr_usrreqs tcp_usrreqs = {
//ScenSim-Port//    .pru_abort =        tcp_usr_abort,
//ScenSim-Port//    .pru_accept =       tcp_usr_accept,
//ScenSim-Port//    .pru_attach =       tcp_usr_attach,
//ScenSim-Port//    .pru_bind =     tcp_usr_bind,
//ScenSim-Port//    .pru_connect =      tcp_usr_connect,
//ScenSim-Port//    .pru_control =      in_control,
//ScenSim-Port//    .pru_detach =       tcp_usr_detach,
//ScenSim-Port//    .pru_disconnect =   tcp_usr_disconnect,
//ScenSim-Port//    .pru_listen =       tcp_usr_listen,
//ScenSim-Port//    .pru_peeraddr =     in_getpeeraddr,
//ScenSim-Port//    .pru_rcvd =     tcp_usr_rcvd,
//ScenSim-Port//    .pru_rcvoob =       tcp_usr_rcvoob,
//ScenSim-Port//    .pru_send =     tcp_usr_send,
//ScenSim-Port//    .pru_shutdown =     tcp_usr_shutdown,
//ScenSim-Port//    .pru_sockaddr =     in_getsockaddr,
//ScenSim-Port//    .pru_sosetlabel =   in_pcbsosetlabel,
//ScenSim-Port//    .pru_close =        tcp_usr_close,
    tcp_usr_abort,                                              //ScenSim-Port//
    tcp_usr_accept,                                             //ScenSim-Port//
    tcp_usr_attach,                                             //ScenSim-Port//
    tcp_usr_bind,                                               //ScenSim-Port//
    tcp_usr_connect,                                            //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    tcp_usr_detach,                                             //ScenSim-Port//
    tcp_usr_disconnect,                                         //ScenSim-Port//
    tcp_usr_listen,                                             //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    tcp_usr_rcvd,                                               //ScenSim-Port//
    tcp_usr_rcvoob,                                             //ScenSim-Port//
    tcp_usr_send,                                               //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    tcp_usr_shutdown,                                           //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    tcp_usr_close                                               //ScenSim-Port//
};
#endif /* INET */

#ifdef INET6
struct pr_usrreqs tcp6_usrreqs = {
//ScenSim-Port//    .pru_abort =        tcp_usr_abort,
//ScenSim-Port//    .pru_accept =       tcp6_usr_accept,
//ScenSim-Port//    .pru_attach =       tcp_usr_attach,
//ScenSim-Port//    .pru_bind =     tcp6_usr_bind,
//ScenSim-Port//    .pru_connect =      tcp6_usr_connect,
//ScenSim-Port//    .pru_control =      in6_control,
//ScenSim-Port//    .pru_detach =       tcp_usr_detach,
//ScenSim-Port//    .pru_disconnect =   tcp_usr_disconnect,
//ScenSim-Port//    .pru_listen =       tcp6_usr_listen,
//ScenSim-Port//    .pru_peeraddr =     in6_mapped_peeraddr,
//ScenSim-Port//    .pru_rcvd =     tcp_usr_rcvd,
//ScenSim-Port//    .pru_rcvoob =       tcp_usr_rcvoob,
//ScenSim-Port//    .pru_send =     tcp_usr_send,
//ScenSim-Port//    .pru_shutdown =     tcp_usr_shutdown,
//ScenSim-Port//    .pru_sockaddr =     in6_mapped_sockaddr,
//ScenSim-Port//    .pru_sosetlabel =   in_pcbsosetlabel,
//ScenSim-Port//    .pru_close =        tcp_usr_close,
    tcp_usr_abort,                                              //ScenSim-Port//
    tcp6_usr_accept,                                            //ScenSim-Port//
    tcp_usr_attach,                                             //ScenSim-Port//
    tcp6_usr_bind,                                              //ScenSim-Port//
    tcp6_usr_connect,                                           //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    tcp_usr_detach,                                             //ScenSim-Port//
    tcp_usr_disconnect,                                         //ScenSim-Port//
    tcp6_usr_listen,                                            //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    tcp_usr_rcvd,                                               //ScenSim-Port//
    tcp_usr_rcvoob,                                             //ScenSim-Port//
    tcp_usr_send,                                               //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    tcp_usr_shutdown,                                           //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    tcp_usr_close                                               //ScenSim-Port//
};
#endif /* INET6 */

#ifdef INET
/*
 * Common subroutine to open a TCP connection to remote host specified
 * by struct sockaddr_in in mbuf *nam.  Call in_pcbbind to assign a local
 * port number if needed.  Call in_pcbconnect_setup to do the routing and
 * to choose a local host address (interface).  If there is an existing
 * incarnation of the same connection in TIME-WAIT state and if the remote
 * host was sending CC options and if the connection duration was < MSL, then
 * truncate the previous TIME-WAIT state and proceed.
 * Initialize connection parameters and enter SYN-SENT state.
 */
static int
tcp_connect(struct tcpcb *tp, struct sockaddr *nam, struct thread *td)
{
    struct inpcb *inp = tp->t_inpcb, *oinp;
    struct socket *so = inp->inp_socket;
    struct in_addr laddr;
    u_short lport;
    int error;

//ScenSim-Port//    INP_WLOCK_ASSERT(inp);
//ScenSim-Port//    INP_HASH_WLOCK(&V_tcbinfo);

    if (inp->inp_lport == 0) {
        error = in_pcbbind(inp, (struct sockaddr *)0, td->td_ucred);
        if (error)
            goto out;
    }

    /*
     * Cannot simply call in_pcbconnect, because there might be an
     * earlier incarnation of this same connection still in
     * TIME_WAIT state, creating an ADDRINUSE error.
     */
    laddr = inp->inp_laddr;
    lport = inp->inp_lport;
    error = in_pcbconnect_setup(inp, nam, &laddr.s_addr, &lport,
        &inp->inp_faddr.s_addr, &inp->inp_fport, &oinp, td->td_ucred);
    if (error && oinp == NULL)
        goto out;
    if (oinp) {
        error = EADDRINUSE;
        goto out;
    }
    inp->inp_laddr = laddr;
//ScenSim-Port//    in_pcbrehash(inp);
//ScenSim-Port//    INP_HASH_WUNLOCK(&V_tcbinfo);

    /*
     * Compute window scaling to request:
     * Scale to fit into sweet spot.  See tcp_syncache.c.
     * XXX: This should move to tcp_output().
     */
    while (tp->request_r_scale < TCP_MAX_WINSHIFT &&
        (TCP_MAXWIN << tp->request_r_scale) < sb_max)
        tp->request_r_scale++;

    soisconnecting(so);
    TCPSTAT_INC(tcps_connattempt);
    tp->t_state = TCPS_SYN_SENT;
    tcp_timer_activate(tp, TT_KEEP, tcp_keepinit);
    tp->iss = tcp_new_isn(tp);
    if (debug_iss_zero) {                                       //ScenSim-Port//
        tp->iss = 0;                                            //ScenSim-Port//
    }                                                           //ScenSim-Port//
    tcp_sendseqinit(tp);

    return 0;

out:
//ScenSim-Port//    INP_HASH_WUNLOCK(&V_tcbinfo);
    return (error);
}
#endif /* INET */

#ifdef INET6
static int
tcp6_connect(struct tcpcb *tp, struct sockaddr *nam, struct thread *td)
{
    struct inpcb *inp = tp->t_inpcb, *oinp;
    struct socket *so = inp->inp_socket;
    struct sockaddr_in6 *sin6 = (struct sockaddr_in6 *)nam;
    struct in6_addr addr6;
    int error;

//ScenSim-Port//    INP_WLOCK_ASSERT(inp);
//ScenSim-Port//    INP_HASH_WLOCK(&V_tcbinfo);

    if (inp->inp_lport == 0) {
        error = in6_pcbbind(inp, (struct sockaddr *)0, td->td_ucred);
        if (error)
            goto out;
    }

    /*
     * Cannot simply call in_pcbconnect, because there might be an
     * earlier incarnation of this same connection still in
     * TIME_WAIT state, creating an ADDRINUSE error.
     * in6_pcbladdr() also handles scope zone IDs.
     *
     * XXXRW: We wouldn't need to expose in6_pcblookup_hash_locked()
     * outside of in6_pcb.c if there were an in6_pcbconnect_setup().
     */
    error = in6_pcbladdr(inp, nam, &addr6);
    if (error)
        goto out;
    oinp = in6_pcblookup_hash_locked(inp->inp_pcbinfo,
                  &sin6->sin6_addr, sin6->sin6_port,
                  IN6_IS_ADDR_UNSPECIFIED(&inp->in6p_laddr)
                  ? &addr6
                  : &inp->in6p_laddr,
                  inp->inp_lport,  0, NULL);
    if (oinp) {
        error = EADDRINUSE;
        goto out;
    }
    if (IN6_IS_ADDR_UNSPECIFIED(&inp->in6p_laddr))
        inp->in6p_laddr = addr6;
    inp->in6p_faddr = sin6->sin6_addr;
    inp->inp_fport = sin6->sin6_port;
    /* update flowinfo - draft-itojun-ipv6-flowlabel-api-00 */
    inp->inp_flow &= ~IPV6_FLOWLABEL_MASK;
    if (inp->inp_flags & IN6P_AUTOFLOWLABEL)
        inp->inp_flow |=
            (htonl(ip6_randomflowlabel()) & IPV6_FLOWLABEL_MASK);
//ScenSim-Port//    in_pcbrehash(inp);
//ScenSim-Port//    INP_HASH_WUNLOCK(&V_tcbinfo);

    /* Compute window scaling to request.  */
    while (tp->request_r_scale < TCP_MAX_WINSHIFT &&
        (TCP_MAXWIN << tp->request_r_scale) < sb_max)
        tp->request_r_scale++;

    soisconnecting(so);
    TCPSTAT_INC(tcps_connattempt);
    tp->t_state = TCPS_SYN_SENT;
    tcp_timer_activate(tp, TT_KEEP, tcp_keepinit);
    tp->iss = tcp_new_isn(tp);
    if (debug_iss_zero) {                                       //ScenSim-Port//
        tp->iss = 0;                                            //ScenSim-Port//
    }                                                           //ScenSim-Port//
    tcp_sendseqinit(tp);

    return 0;

out:
//ScenSim-Port//    INP_HASH_WUNLOCK(&V_tcbinfo);
    return error;
}
#endif /* INET6 */

/*
 * Export TCP internal state information via a struct tcp_info, based on the
 * Linux 2.6 API.  Not ABI compatible as our constants are mapped differently
 * (TCP state machine, etc).  We export all information using FreeBSD-native
 * constants -- for example, the numeric values for tcpi_state will differ
 * from Linux.
 */
//ScenSim-Port//static void
//ScenSim-Port//tcp_fill_info(struct tcpcb *tp, struct tcp_info *ti)
//ScenSim-Port//{
//ScenSim-Port//
//ScenSim-Port//    INP_WLOCK_ASSERT(tp->t_inpcb);
//ScenSim-Port//    bzero(ti, sizeof(*ti));
//ScenSim-Port//
//ScenSim-Port//    ti->tcpi_state = tp->t_state;
//ScenSim-Port//    if ((tp->t_flags & TF_REQ_TSTMP) && (tp->t_flags & TF_RCVD_TSTMP))
//ScenSim-Port//        ti->tcpi_options |= TCPI_OPT_TIMESTAMPS;
//ScenSim-Port//    if (tp->t_flags & TF_SACK_PERMIT)
//ScenSim-Port//        ti->tcpi_options |= TCPI_OPT_SACK;
//ScenSim-Port//    if ((tp->t_flags & TF_REQ_SCALE) && (tp->t_flags & TF_RCVD_SCALE)) {
//ScenSim-Port//        ti->tcpi_options |= TCPI_OPT_WSCALE;
//ScenSim-Port//        ti->tcpi_snd_wscale = tp->snd_scale;
//ScenSim-Port//        ti->tcpi_rcv_wscale = tp->rcv_scale;
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//    ti->tcpi_rto = tp->t_rxtcur * tick;
//ScenSim-Port//    ti->tcpi_last_data_recv = (long)(ticks - (int)tp->t_rcvtime) * tick;
//ScenSim-Port//    ti->tcpi_rtt = ((u_int64_t)tp->t_srtt * tick) >> TCP_RTT_SHIFT;
//ScenSim-Port//    ti->tcpi_rttvar = ((u_int64_t)tp->t_rttvar * tick) >> TCP_RTTVAR_SHIFT;
//ScenSim-Port//
//ScenSim-Port//    ti->tcpi_snd_ssthresh = tp->snd_ssthresh;
//ScenSim-Port//    ti->tcpi_snd_cwnd = tp->snd_cwnd;
//ScenSim-Port//
//ScenSim-Port//    /*
//ScenSim-Port//     * FreeBSD-specific extension fields for tcp_info.
//ScenSim-Port//     */
//ScenSim-Port//    ti->tcpi_rcv_space = tp->rcv_wnd;
//ScenSim-Port//    ti->tcpi_rcv_nxt = tp->rcv_nxt;
//ScenSim-Port//    ti->tcpi_snd_wnd = tp->snd_wnd;
//ScenSim-Port//    ti->tcpi_snd_bwnd = 0;      /* Unused, kept for compat. */
//ScenSim-Port//    ti->tcpi_snd_nxt = tp->snd_nxt;
//ScenSim-Port//    ti->tcpi_snd_mss = tp->t_maxseg;
//ScenSim-Port//    ti->tcpi_rcv_mss = tp->t_maxseg;
//ScenSim-Port//    if (tp->t_flags & TF_TOE)
//ScenSim-Port//        ti->tcpi_options |= TCPI_OPT_TOE;
//ScenSim-Port//    ti->tcpi_snd_rexmitpack = tp->t_sndrexmitpack;
//ScenSim-Port//    ti->tcpi_rcv_ooopack = tp->t_rcvoopack;
//ScenSim-Port//    ti->tcpi_snd_zerowin = tp->t_sndzerowin;
//ScenSim-Port//}

/*
 * tcp_ctloutput() must drop the inpcb lock before performing copyin on
 * socket option arguments.  When it re-acquires the lock after the copy, it
 * has to revalidate that the connection is still valid for the socket
 * option.
 */
#if 0                                                           //ScenSim-Port//
#define INP_WLOCK_RECHECK(inp) do {                 \
    INP_WLOCK(inp);                         \
    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {        \
        INP_WUNLOCK(inp);                   \
        return (ECONNRESET);                    \
    }                               \
    tp = intotcpcb(inp);                        \
} while(0)
#endif                                                          //ScenSim-Port//

//ScenSim-Port//int
//ScenSim-Port//tcp_ctloutput(struct socket *so, struct sockopt *sopt)
//ScenSim-Port//{
//ScenSim-Port//    int error, opt, optval;
//ScenSim-Port//    struct  inpcb *inp;
//ScenSim-Port//    struct  tcpcb *tp;
//ScenSim-Port//    struct  tcp_info ti;
//ScenSim-Port//    char buf[TCP_CA_NAME_MAX];
//ScenSim-Port//    struct cc_algo *algo;
//ScenSim-Port//
//ScenSim-Port//    error = 0;
//ScenSim-Port//    inp = sotoinpcb(so);
//ScenSim-Port//    KASSERT(inp != NULL, ("tcp_ctloutput: inp == NULL"));
//ScenSim-Port//    INP_WLOCK(inp);
//ScenSim-Port//    if (sopt->sopt_level != IPPROTO_TCP) {
//ScenSim-Port//#ifdef INET6
//ScenSim-Port//        if (inp->inp_vflag & INP_IPV6PROTO) {
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = ip6_ctloutput(so, sopt);
//ScenSim-Port//        }
//ScenSim-Port//#endif /* INET6 */
//ScenSim-Port//#if defined(INET6) && defined(INET)
//ScenSim-Port//        else
//ScenSim-Port//#endif
//ScenSim-Port//#ifdef INET
//ScenSim-Port//        {
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = ip_ctloutput(so, sopt);
//ScenSim-Port//        }
//ScenSim-Port//#endif
//ScenSim-Port//        return (error);
//ScenSim-Port//    }
//ScenSim-Port//    if (inp->inp_flags & (INP_TIMEWAIT | INP_DROPPED)) {
//ScenSim-Port//        INP_WUNLOCK(inp);
//ScenSim-Port//        return (ECONNRESET);
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//    switch (sopt->sopt_dir) {
//ScenSim-Port//    case SOPT_SET:
//ScenSim-Port//        switch (sopt->sopt_name) {
//ScenSim-Port//#ifdef TCP_SIGNATURE
//ScenSim-Port//        case TCP_MD5SIG:
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = sooptcopyin(sopt, &optval, sizeof optval,
//ScenSim-Port//                sizeof optval);
//ScenSim-Port//            if (error)
//ScenSim-Port//                return (error);
//ScenSim-Port//
//ScenSim-Port//            INP_WLOCK_RECHECK(inp);
//ScenSim-Port//            if (optval > 0)
//ScenSim-Port//                tp->t_flags |= TF_SIGNATURE;
//ScenSim-Port//            else
//ScenSim-Port//                tp->t_flags &= ~TF_SIGNATURE;
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            break;
//ScenSim-Port//#endif /* TCP_SIGNATURE */
//ScenSim-Port//        case TCP_NODELAY:
//ScenSim-Port//        case TCP_NOOPT:
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = sooptcopyin(sopt, &optval, sizeof optval,
//ScenSim-Port//                sizeof optval);
//ScenSim-Port//            if (error)
//ScenSim-Port//                return (error);
//ScenSim-Port//
//ScenSim-Port//            INP_WLOCK_RECHECK(inp);
//ScenSim-Port//            switch (sopt->sopt_name) {
//ScenSim-Port//            case TCP_NODELAY:
//ScenSim-Port//                opt = TF_NODELAY;
//ScenSim-Port//                break;
//ScenSim-Port//            case TCP_NOOPT:
//ScenSim-Port//                opt = TF_NOOPT;
//ScenSim-Port//                break;
//ScenSim-Port//            default:
//ScenSim-Port//                opt = 0; /* dead code to fool gcc */
//ScenSim-Port//                break;
//ScenSim-Port//            }
//ScenSim-Port//
//ScenSim-Port//            if (optval)
//ScenSim-Port//                tp->t_flags |= opt;
//ScenSim-Port//            else
//ScenSim-Port//                tp->t_flags &= ~opt;
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            break;
//ScenSim-Port//
//ScenSim-Port//        case TCP_NOPUSH:
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = sooptcopyin(sopt, &optval, sizeof optval,
//ScenSim-Port//                sizeof optval);
//ScenSim-Port//            if (error)
//ScenSim-Port//                return (error);
//ScenSim-Port//
//ScenSim-Port//            INP_WLOCK_RECHECK(inp);
//ScenSim-Port//            if (optval)
//ScenSim-Port//                tp->t_flags |= TF_NOPUSH;
//ScenSim-Port//            else if (tp->t_flags & TF_NOPUSH) {
//ScenSim-Port//                tp->t_flags &= ~TF_NOPUSH;
//ScenSim-Port//                if (TCPS_HAVEESTABLISHED(tp->t_state))
//ScenSim-Port//                    error = tcp_output(tp);
//ScenSim-Port//            }
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            break;
//ScenSim-Port//
//ScenSim-Port//        case TCP_MAXSEG:
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = sooptcopyin(sopt, &optval, sizeof optval,
//ScenSim-Port//                sizeof optval);
//ScenSim-Port//            if (error)
//ScenSim-Port//                return (error);
//ScenSim-Port//
//ScenSim-Port//            INP_WLOCK_RECHECK(inp);
//ScenSim-Port//            if (optval > 0 && optval <= tp->t_maxseg &&
//ScenSim-Port//                optval + 40 >= V_tcp_minmss)
//ScenSim-Port//                tp->t_maxseg = optval;
//ScenSim-Port//            else
//ScenSim-Port//                error = EINVAL;
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            break;
//ScenSim-Port//
//ScenSim-Port//        case TCP_INFO:
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = EINVAL;
//ScenSim-Port//            break;
//ScenSim-Port//
//ScenSim-Port//        case TCP_CONGESTION:
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            bzero(buf, sizeof(buf));
//ScenSim-Port//            error = sooptcopyin(sopt, &buf, sizeof(buf), 1);
//ScenSim-Port//            if (error)
//ScenSim-Port//                break;
//ScenSim-Port//            INP_WLOCK_RECHECK(inp);
//ScenSim-Port//            /*
//ScenSim-Port//             * Return EINVAL if we can't find the requested cc algo.
//ScenSim-Port//             */
//ScenSim-Port//            error = EINVAL;
//ScenSim-Port//            CC_LIST_RLOCK();
//ScenSim-Port//            STAILQ_FOREACH(algo, &cc_list, entries) {
//ScenSim-Port//                if (strncmp(buf, algo->name, TCP_CA_NAME_MAX)
//ScenSim-Port//                    == 0) {
//ScenSim-Port//                    /* We've found the requested algo. */
//ScenSim-Port//                    error = 0;
//ScenSim-Port//                    /*
//ScenSim-Port//                     * We hold a write lock over the tcb
//ScenSim-Port//                     * so it's safe to do these things
//ScenSim-Port//                     * without ordering concerns.
//ScenSim-Port//                     */
//ScenSim-Port//                    if (CC_ALGO(tp)->cb_destroy != NULL)
//ScenSim-Port//                        CC_ALGO(tp)->cb_destroy(tp->ccv);
//ScenSim-Port//                    CC_ALGO(tp) = algo;
//ScenSim-Port//                    /*
//ScenSim-Port//                     * If something goes pear shaped
//ScenSim-Port//                     * initialising the new algo,
//ScenSim-Port//                     * fall back to newreno (which
//ScenSim-Port//                     * does not require initialisation).
//ScenSim-Port//                     */
//ScenSim-Port//                    if (algo->cb_init != NULL)
//ScenSim-Port//                        if (algo->cb_init(tp->ccv) > 0) {
//ScenSim-Port//                            CC_ALGO(tp) = &newreno_cc_algo;
//ScenSim-Port//                            /*
//ScenSim-Port//                             * The only reason init
//ScenSim-Port//                             * should fail is
//ScenSim-Port//                             * because of malloc.
//ScenSim-Port//                             */
//ScenSim-Port//                            error = ENOMEM;
//ScenSim-Port//                        }
//ScenSim-Port//                    break; /* Break the STAILQ_FOREACH. */
//ScenSim-Port//                }
//ScenSim-Port//            }
//ScenSim-Port//            CC_LIST_RUNLOCK();
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            break;
//ScenSim-Port//
//ScenSim-Port//        default:
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = ENOPROTOOPT;
//ScenSim-Port//            break;
//ScenSim-Port//        }
//ScenSim-Port//        break;
//ScenSim-Port//
//ScenSim-Port//    case SOPT_GET:
//ScenSim-Port//        tp = intotcpcb(inp);
//ScenSim-Port//        switch (sopt->sopt_name) {
//ScenSim-Port//#ifdef TCP_SIGNATURE
//ScenSim-Port//        case TCP_MD5SIG:
//ScenSim-Port//            optval = (tp->t_flags & TF_SIGNATURE) ? 1 : 0;
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = sooptcopyout(sopt, &optval, sizeof optval);
//ScenSim-Port//            break;
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//        case TCP_NODELAY:
//ScenSim-Port//            optval = tp->t_flags & TF_NODELAY;
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = sooptcopyout(sopt, &optval, sizeof optval);
//ScenSim-Port//            break;
//ScenSim-Port//        case TCP_MAXSEG:
//ScenSim-Port//            optval = tp->t_maxseg;
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = sooptcopyout(sopt, &optval, sizeof optval);
//ScenSim-Port//            break;
//ScenSim-Port//        case TCP_NOOPT:
//ScenSim-Port//            optval = tp->t_flags & TF_NOOPT;
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = sooptcopyout(sopt, &optval, sizeof optval);
//ScenSim-Port//            break;
//ScenSim-Port//        case TCP_NOPUSH:
//ScenSim-Port//            optval = tp->t_flags & TF_NOPUSH;
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = sooptcopyout(sopt, &optval, sizeof optval);
//ScenSim-Port//            break;
//ScenSim-Port//        case TCP_INFO:
//ScenSim-Port//            tcp_fill_info(tp, &ti);
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = sooptcopyout(sopt, &ti, sizeof ti);
//ScenSim-Port//            break;
//ScenSim-Port//        case TCP_CONGESTION:
//ScenSim-Port//            bzero(buf, sizeof(buf));
//ScenSim-Port//            strlcpy(buf, CC_ALGO(tp)->name, TCP_CA_NAME_MAX);
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = sooptcopyout(sopt, buf, TCP_CA_NAME_MAX);
//ScenSim-Port//            break;
//ScenSim-Port//        default:
//ScenSim-Port//            INP_WUNLOCK(inp);
//ScenSim-Port//            error = ENOPROTOOPT;
//ScenSim-Port//            break;
//ScenSim-Port//        }
//ScenSim-Port//        break;
//ScenSim-Port//    }
//ScenSim-Port//    return (error);
//ScenSim-Port//}
//ScenSim-Port//#undef INP_WLOCK_RECHECK

/*
 * tcp_sendspace and tcp_recvspace are the default send and receive window
 * sizes, respectively.  These are obsolescent (this information should
 * be set by the route).
 */
//ScenSim-Port//u_long  tcp_sendspace = 1024*32;
//ScenSim-Port//SYSCTL_ULONG(_net_inet_tcp, TCPCTL_SENDSPACE, sendspace, CTLFLAG_RW,
//ScenSim-Port//    &tcp_sendspace , 0, "Maximum outgoing TCP datagram size");
//ScenSim-Port//u_long  tcp_recvspace = 1024*64;
//ScenSim-Port//SYSCTL_ULONG(_net_inet_tcp, TCPCTL_RECVSPACE, recvspace, CTLFLAG_RW,
//ScenSim-Port//    &tcp_recvspace , 0, "Maximum incoming TCP datagram size");

/*
 * Attach TCP protocol to socket, allocating
 * internet protocol control block, tcp control block,
 * bufer space, and entering LISTEN state if to accept connections.
 */
static int
tcp_attach(struct socket *so)
{
    struct tcpcb *tp;
    struct inpcb *inp;
    int error;

    if (so->so_snd.sb_hiwat == 0 || so->so_rcv.sb_hiwat == 0) {
        error = soreserve(so, tcp_sendspace, tcp_recvspace);
        if (error)
            return (error);
    }
    so->so_rcv.sb_flags |= SB_AUTOSIZE;
    so->so_snd.sb_flags |= SB_AUTOSIZE;
//ScenSim-Port//    INP_INFO_WLOCK(&V_tcbinfo);
    error = in_pcballoc(so, &V_tcbinfo);
    if (error) {
//ScenSim-Port//        INP_INFO_WUNLOCK(&V_tcbinfo);
        return (error);
    }
    inp = sotoinpcb(so);
#ifdef INET6
    if (inp->inp_vflag & INP_IPV6PROTO) {
        inp->inp_vflag |= INP_IPV6;
        inp->in6p_hops = -1;    /* use kernel default */
    }
    else
#endif
    inp->inp_vflag |= INP_IPV4;
    tp = tcp_newtcpcb(inp);
    if (tp == NULL) {
        in_pcbdetach(inp);
        in_pcbfree(inp);
//ScenSim-Port//        INP_INFO_WUNLOCK(&V_tcbinfo);
        return (ENOBUFS);
    }
    tp->t_state = TCPS_CLOSED;
//ScenSim-Port//    INP_WUNLOCK(inp);
//ScenSim-Port//    INP_INFO_WUNLOCK(&V_tcbinfo);
    return (0);
}

/*
 * Initiate (or continue) disconnect.
 * If embryonic state, just send reset (once).
 * If in ``let data drain'' option and linger null, just drop.
 * Otherwise (hard), mark socket disconnecting and drop
 * current input data; switch states based on user close, and
 * send segment to peer (with FIN).
 */
static void
tcp_disconnect(struct tcpcb *tp)
{
    struct inpcb *inp = tp->t_inpcb;
    struct socket *so = inp->inp_socket;

//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK_ASSERT(inp);

    /*
     * Neither tcp_close() nor tcp_drop() should return NULL, as the
     * socket is still open.
     */
    if (tp->t_state < TCPS_ESTABLISHED) {
        tp = tcp_close(tp);
//ScenSim-Port//        KASSERT(tp != NULL,
//ScenSim-Port//            ("tcp_disconnect: tcp_close() returned NULL"));
//ScenSim-Port//    } else if ((so->so_options & SO_LINGER) && so->so_linger == 0) {
//ScenSim-Port//        tp = tcp_drop(tp, 0);
//ScenSim-Port//        KASSERT(tp != NULL,
//ScenSim-Port//            ("tcp_disconnect: tcp_drop() returned NULL"));
    } else {
        soisdisconnecting(so);
        sbflush(&so->so_rcv);
        tcp_usrclosed(tp);
        if (!(inp->inp_flags & INP_DROPPED))
            tcp_output_disconnect(tp);
    }
}

/*
 * User issued close, and wish to trail through shutdown states:
 * if never received SYN, just forget it.  If got a SYN from peer,
 * but haven't sent FIN, then go to FIN_WAIT_1 state to send peer a FIN.
 * If already got a FIN from peer, then almost done; go to LAST_ACK
 * state.  In all other cases, have already sent FIN to peer (e.g.
 * after PRU_SHUTDOWN), and just have to play tedious game waiting
 * for peer to send FIN or not respond to keep-alives, etc.
 * We can let the user exit from the close as soon as the FIN is acked.
 */
static void
tcp_usrclosed(struct tcpcb *tp)
{

//ScenSim-Port//    INP_INFO_WLOCK_ASSERT(&V_tcbinfo);
//ScenSim-Port//    INP_WLOCK_ASSERT(tp->t_inpcb);

    switch (tp->t_state) {
    case TCPS_LISTEN:
//ScenSim-Port//        tcp_offload_listen_close(tp);
        /* FALLTHROUGH */
    case TCPS_CLOSED:
        tp->t_state = TCPS_CLOSED;
        tp = tcp_close(tp);
        /*
         * tcp_close() should never return NULL here as the socket is
         * still open.
         */
//ScenSim-Port//        KASSERT(tp != NULL,
//ScenSim-Port//            ("tcp_usrclosed: tcp_close() returned NULL"));
        break;

    case TCPS_SYN_SENT:
    case TCPS_SYN_RECEIVED:
        tp->t_flags |= TF_NEEDFIN;
        break;

    case TCPS_ESTABLISHED:
        tp->t_state = TCPS_FIN_WAIT_1;
        break;

    case TCPS_CLOSE_WAIT:
        tp->t_state = TCPS_LAST_ACK;
        break;
    }
    if (tp->t_state >= TCPS_FIN_WAIT_2) {
        soisdisconnected(tp->t_inpcb->inp_socket);
        /* Prevent the connection hanging in FIN_WAIT_2 forever. */
        if (tp->t_state == TCPS_FIN_WAIT_2) {
            int timeout;

            timeout = (tcp_fast_finwait2_recycle) ?
                tcp_finwait2_timeout : tcp_maxidle;
            tcp_timer_activate(tp, TT_2MSL, timeout);
        }
    }
}

//ScenSim-Port//#ifdef DDB
//ScenSim-Port//static void
//ScenSim-Port//db_print_indent(int indent)
//ScenSim-Port//{
//ScenSim-Port//    int i;
//ScenSim-Port//
//ScenSim-Port//    for (i = 0; i < indent; i++)
//ScenSim-Port//        db_printf(" ");
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//static void
//ScenSim-Port//db_print_tstate(int t_state)
//ScenSim-Port//{
//ScenSim-Port//
//ScenSim-Port//    switch (t_state) {
//ScenSim-Port//    case TCPS_CLOSED:
//ScenSim-Port//        db_printf("TCPS_CLOSED");
//ScenSim-Port//        return;
//ScenSim-Port//
//ScenSim-Port//    case TCPS_LISTEN:
//ScenSim-Port//        db_printf("TCPS_LISTEN");
//ScenSim-Port//        return;
//ScenSim-Port//
//ScenSim-Port//    case TCPS_SYN_SENT:
//ScenSim-Port//        db_printf("TCPS_SYN_SENT");
//ScenSim-Port//        return;
//ScenSim-Port//
//ScenSim-Port//    case TCPS_SYN_RECEIVED:
//ScenSim-Port//        db_printf("TCPS_SYN_RECEIVED");
//ScenSim-Port//        return;
//ScenSim-Port//
//ScenSim-Port//    case TCPS_ESTABLISHED:
//ScenSim-Port//        db_printf("TCPS_ESTABLISHED");
//ScenSim-Port//        return;
//ScenSim-Port//
//ScenSim-Port//    case TCPS_CLOSE_WAIT:
//ScenSim-Port//        db_printf("TCPS_CLOSE_WAIT");
//ScenSim-Port//        return;
//ScenSim-Port//
//ScenSim-Port//    case TCPS_FIN_WAIT_1:
//ScenSim-Port//        db_printf("TCPS_FIN_WAIT_1");
//ScenSim-Port//        return;
//ScenSim-Port//
//ScenSim-Port//    case TCPS_CLOSING:
//ScenSim-Port//        db_printf("TCPS_CLOSING");
//ScenSim-Port//        return;
//ScenSim-Port//
//ScenSim-Port//    case TCPS_LAST_ACK:
//ScenSim-Port//        db_printf("TCPS_LAST_ACK");
//ScenSim-Port//        return;
//ScenSim-Port//
//ScenSim-Port//    case TCPS_FIN_WAIT_2:
//ScenSim-Port//        db_printf("TCPS_FIN_WAIT_2");
//ScenSim-Port//        return;
//ScenSim-Port//
//ScenSim-Port//    case TCPS_TIME_WAIT:
//ScenSim-Port//        db_printf("TCPS_TIME_WAIT");
//ScenSim-Port//        return;
//ScenSim-Port//
//ScenSim-Port//    default:
//ScenSim-Port//        db_printf("unknown");
//ScenSim-Port//        return;
//ScenSim-Port//    }
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//static void
//ScenSim-Port//db_print_tflags(u_int t_flags)
//ScenSim-Port//{
//ScenSim-Port//    int comma;
//ScenSim-Port//
//ScenSim-Port//    comma = 0;
//ScenSim-Port//    if (t_flags & TF_ACKNOW) {
//ScenSim-Port//        db_printf("%sTF_ACKNOW", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_DELACK) {
//ScenSim-Port//        db_printf("%sTF_DELACK", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_NODELAY) {
//ScenSim-Port//        db_printf("%sTF_NODELAY", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_NOOPT) {
//ScenSim-Port//        db_printf("%sTF_NOOPT", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_SENTFIN) {
//ScenSim-Port//        db_printf("%sTF_SENTFIN", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_REQ_SCALE) {
//ScenSim-Port//        db_printf("%sTF_REQ_SCALE", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_RCVD_SCALE) {
//ScenSim-Port//        db_printf("%sTF_RECVD_SCALE", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_REQ_TSTMP) {
//ScenSim-Port//        db_printf("%sTF_REQ_TSTMP", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_RCVD_TSTMP) {
//ScenSim-Port//        db_printf("%sTF_RCVD_TSTMP", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_SACK_PERMIT) {
//ScenSim-Port//        db_printf("%sTF_SACK_PERMIT", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_NEEDSYN) {
//ScenSim-Port//        db_printf("%sTF_NEEDSYN", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_NEEDFIN) {
//ScenSim-Port//        db_printf("%sTF_NEEDFIN", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_NOPUSH) {
//ScenSim-Port//        db_printf("%sTF_NOPUSH", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_MORETOCOME) {
//ScenSim-Port//        db_printf("%sTF_MORETOCOME", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_LQ_OVERFLOW) {
//ScenSim-Port//        db_printf("%sTF_LQ_OVERFLOW", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_LASTIDLE) {
//ScenSim-Port//        db_printf("%sTF_LASTIDLE", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_RXWIN0SENT) {
//ScenSim-Port//        db_printf("%sTF_RXWIN0SENT", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_FASTRECOVERY) {
//ScenSim-Port//        db_printf("%sTF_FASTRECOVERY", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_CONGRECOVERY) {
//ScenSim-Port//        db_printf("%sTF_CONGRECOVERY", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_WASFRECOVERY) {
//ScenSim-Port//        db_printf("%sTF_WASFRECOVERY", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_SIGNATURE) {
//ScenSim-Port//        db_printf("%sTF_SIGNATURE", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_FORCEDATA) {
//ScenSim-Port//        db_printf("%sTF_FORCEDATA", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_TSO) {
//ScenSim-Port//        db_printf("%sTF_TSO", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_flags & TF_ECN_PERMIT) {
//ScenSim-Port//        db_printf("%sTF_ECN_PERMIT", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//static void
//ScenSim-Port//db_print_toobflags(char t_oobflags)
//ScenSim-Port//{
//ScenSim-Port//    int comma;
//ScenSim-Port//
//ScenSim-Port//    comma = 0;
//ScenSim-Port//    if (t_oobflags & TCPOOB_HAVEDATA) {
//ScenSim-Port//        db_printf("%sTCPOOB_HAVEDATA", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//    if (t_oobflags & TCPOOB_HADDATA) {
//ScenSim-Port//        db_printf("%sTCPOOB_HADDATA", comma ? ", " : "");
//ScenSim-Port//        comma = 1;
//ScenSim-Port//    }
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//static void
//ScenSim-Port//db_print_tcpcb(struct tcpcb *tp, const char *name, int indent)
//ScenSim-Port//{
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("%s at %p\n", name, tp);
//ScenSim-Port//
//ScenSim-Port//    indent += 2;
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("t_segq first: %p   t_segqlen: %d   t_dupacks: %d\n",
//ScenSim-Port//       LIST_FIRST(&tp->t_segq), tp->t_segqlen, tp->t_dupacks);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("tt_rexmt: %p   tt_persist: %p   tt_keep: %p\n",
//ScenSim-Port//        &tp->t_timers->tt_rexmt, &tp->t_timers->tt_persist, &tp->t_timers->tt_keep);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("tt_2msl: %p   tt_delack: %p   t_inpcb: %p\n", &tp->t_timers->tt_2msl,
//ScenSim-Port//        &tp->t_timers->tt_delack, tp->t_inpcb);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("t_state: %d (", tp->t_state);
//ScenSim-Port//    db_print_tstate(tp->t_state);
//ScenSim-Port//    db_printf(")\n");
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("t_flags: 0x%x (", tp->t_flags);
//ScenSim-Port//    db_print_tflags(tp->t_flags);
//ScenSim-Port//    db_printf(")\n");
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("snd_una: 0x%08x   snd_max: 0x%08x   snd_nxt: x0%08x\n",
//ScenSim-Port//        tp->snd_una, tp->snd_max, tp->snd_nxt);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("snd_up: 0x%08x   snd_wl1: 0x%08x   snd_wl2: 0x%08x\n",
//ScenSim-Port//       tp->snd_up, tp->snd_wl1, tp->snd_wl2);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("iss: 0x%08x   irs: 0x%08x   rcv_nxt: 0x%08x\n",
//ScenSim-Port//        tp->iss, tp->irs, tp->rcv_nxt);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("rcv_adv: 0x%08x   rcv_wnd: %lu   rcv_up: 0x%08x\n",
//ScenSim-Port//        tp->rcv_adv, tp->rcv_wnd, tp->rcv_up);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("snd_wnd: %lu   snd_cwnd: %lu\n",
//ScenSim-Port//       tp->snd_wnd, tp->snd_cwnd);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("snd_ssthresh: %lu   snd_recover: "
//ScenSim-Port//        "0x%08x\n", tp->snd_ssthresh, tp->snd_recover);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("t_maxopd: %u   t_rcvtime: %u   t_startime: %u\n",
//ScenSim-Port//        tp->t_maxopd, tp->t_rcvtime, tp->t_starttime);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("t_rttime: %u   t_rtsq: 0x%08x\n",
//ScenSim-Port//        tp->t_rtttime, tp->t_rtseq);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("t_rxtcur: %d   t_maxseg: %u   t_srtt: %d\n",
//ScenSim-Port//        tp->t_rxtcur, tp->t_maxseg, tp->t_srtt);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("t_rttvar: %d   t_rxtshift: %d   t_rttmin: %u   "
//ScenSim-Port//        "t_rttbest: %u\n", tp->t_rttvar, tp->t_rxtshift, tp->t_rttmin,
//ScenSim-Port//        tp->t_rttbest);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("t_rttupdated: %lu   max_sndwnd: %lu   t_softerror: %d\n",
//ScenSim-Port//        tp->t_rttupdated, tp->max_sndwnd, tp->t_softerror);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("t_oobflags: 0x%x (", tp->t_oobflags);
//ScenSim-Port//    db_print_toobflags(tp->t_oobflags);
//ScenSim-Port//    db_printf(")   t_iobc: 0x%02x\n", tp->t_iobc);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("snd_scale: %u   rcv_scale: %u   request_r_scale: %u\n",
//ScenSim-Port//        tp->snd_scale, tp->rcv_scale, tp->request_r_scale);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("ts_recent: %u   ts_recent_age: %u\n",
//ScenSim-Port//        tp->ts_recent, tp->ts_recent_age);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("ts_offset: %u   last_ack_sent: 0x%08x   snd_cwnd_prev: "
//ScenSim-Port//        "%lu\n", tp->ts_offset, tp->last_ack_sent, tp->snd_cwnd_prev);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("snd_ssthresh_prev: %lu   snd_recover_prev: 0x%08x   "
//ScenSim-Port//        "t_badrxtwin: %u\n", tp->snd_ssthresh_prev,
//ScenSim-Port//        tp->snd_recover_prev, tp->t_badrxtwin);
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("snd_numholes: %d  snd_holes first: %p\n",
//ScenSim-Port//        tp->snd_numholes, TAILQ_FIRST(&tp->snd_holes));
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("snd_fack: 0x%08x   rcv_numsacks: %d   sack_newdata: "
//ScenSim-Port//        "0x%08x\n", tp->snd_fack, tp->rcv_numsacks, tp->sack_newdata);
//ScenSim-Port//
//ScenSim-Port//    /* Skip sackblks, sackhint. */
//ScenSim-Port//
//ScenSim-Port//    db_print_indent(indent);
//ScenSim-Port//    db_printf("t_rttlow: %d   rfbuf_ts: %u   rfbuf_cnt: %d\n",
//ScenSim-Port//        tp->t_rttlow, tp->rfbuf_ts, tp->rfbuf_cnt);
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//DB_SHOW_COMMAND(tcpcb, db_show_tcpcb)
//ScenSim-Port//{
//ScenSim-Port//    struct tcpcb *tp;
//ScenSim-Port//
//ScenSim-Port//    if (!have_addr) {
//ScenSim-Port//        db_printf("usage: show tcpcb <addr>\n");
//ScenSim-Port//        return;
//ScenSim-Port//    }
//ScenSim-Port//    tp = (struct tcpcb *)addr;
//ScenSim-Port//
//ScenSim-Port//    db_print_tcpcb(tp, "tcpcb", 0);
//ScenSim-Port//}
//ScenSim-Port//#endif
}//namespace//                                                  //ScenSim-Port//
