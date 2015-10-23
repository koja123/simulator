/*-
 * Copyright (c) 2009-2010
 *  Swinburne University of Technology, Melbourne, Australia
 * Copyright (c) 2010 Lawrence Stewart <lstewart@freebsd.org>
 * Copyright (c) 2010-2011 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed at the Centre for Advanced Internet
 * Architectures, Swinburne University of Technology, by David Hayes and
 * Lawrence Stewart, made possible in part by a grant from the Cisco University
 * Research Program Fund at Community Foundation Silicon Valley.
 *
 * Portions of this software were developed at the Centre for Advanced Internet
 * Architectures, Swinburne University of Technology, Melbourne, Australia by
 * David Hayes under sponsorship from the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * An implementation of the Hamilton Institute's delay-based congestion control
 * algorithm for FreeBSD, based on "A strategy for fair coexistence of loss and
 * delay-based congestion control algorithms," by L. Budzisz, R. Stanojevic, R.
 * Shorten, and F. Baker, IEEE Commun. Lett., vol. 13, no. 7, pp. 555--557, Jul.
 * 2009.
 *
 * Originally released as part of the NewTCP research project at Swinburne
 * University of Technology's Centre for Advanced Internet Architectures,
 * Melbourne, Australia, which was made possible in part by a grant from the
 * Cisco University Research Program Fund at Community Foundation Silicon
 * Valley. More details are available at:
 *   http://caia.swin.edu.au/urp/newtcp/
 */

//ScenSim-Port//#include <sys/cdefs.h>
//ScenSim-Port//__FBSDID("$FreeBSD$");

//ScenSim-Port//#include <sys/param.h>
//ScenSim-Port//#include <sys/kernel.h>
//ScenSim-Port//#include <sys/khelp.h>
//ScenSim-Port//#include <sys/limits.h>
//ScenSim-Port//#include <sys/malloc.h>
//ScenSim-Port//#include <sys/module.h>
//ScenSim-Port//#include <sys/queue.h>
//ScenSim-Port//#include <sys/socket.h>
//ScenSim-Port//#include <sys/socketvar.h>
//ScenSim-Port//#include <sys/sysctl.h>
//ScenSim-Port//#include <sys/systm.h>

//ScenSim-Port//#include <net/if.h>
//ScenSim-Port//#include <net/vnet.h>

//ScenSim-Port//#include <netinet/cc.h>
//ScenSim-Port//#include <netinet/tcp_seq.h>
//ScenSim-Port//#include <netinet/tcp_timer.h>
//ScenSim-Port//#include <netinet/tcp_var.h>

//ScenSim-Port//#include <netinet/cc/cc_module.h>

//ScenSim-Port//#include <netinet/khelp/h_ertt.h>
#include "tcp_porting.h"                     //ScenSim-Port//
namespace FreeBsd9Port {                                        //ScenSim-Port//

//ScenSim-Port//#define CAST_PTR_INT(X) (*((int*)(X)))

/* Largest possible number returned by random(). */
//ScenSim-Port//#define RANDOM_MAX  INT_MAX

static void hd_ack_received(struct cc_var *ccv, uint16_t ack_type);
static int  hd_mod_init(void);

//ScenSim-Port//static int ertt_id;

//ScenSim-Port//static VNET_DEFINE(uint32_t, hd_qthresh) = 20;
//ScenSim-Port//static VNET_DEFINE(uint32_t, hd_qmin) = 5;
//ScenSim-Port//static VNET_DEFINE(uint32_t, hd_pmax) = 5;
//ScenSim-Port//#define V_hd_qthresh    VNET(hd_qthresh)
//ScenSim-Port//#define V_hd_qmin   VNET(hd_qmin)
//ScenSim-Port//#define V_hd_pmax   VNET(hd_pmax)

struct cc_algo hd_cc_algo = {
//ScenSim-Port//    .name = "hd",
//ScenSim-Port//    .ack_received = hd_ack_received,
//ScenSim-Port//    .mod_init = hd_mod_init
    "hd",                                                       //ScenSim-Port//
    hd_mod_init,                                                //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    hd_ack_received,                                            //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
};

/*
 * Hamilton backoff function. Returns 1 if we should backoff or 0 otherwise.
 */
static __inline int
should_backoff(int qdly, int maxqdly)
{
    unsigned long p;

    if (qdly < V_hd_qthresh) {
        p = (((RANDOM_MAX / 100) * V_hd_pmax) /
            (V_hd_qthresh - V_hd_qmin)) * (qdly - V_hd_qmin);
    } else {
        if (qdly > V_hd_qthresh)
            p = (((RANDOM_MAX / 100) * V_hd_pmax) /
                (maxqdly - V_hd_qthresh)) * (maxqdly - qdly);
        else
            p = (RANDOM_MAX / 100) * V_hd_pmax;
    }

    return (random() < p);
}

/*
 * If the ack type is CC_ACK, and the inferred queueing delay is greater than
 * the Qmin threshold, cwnd is reduced probabilistically. When backing off due
 * to delay, HD behaves like NewReno when an ECN signal is received. HD behaves
 * as NewReno in all other circumstances.
 */
static void
hd_ack_received(struct cc_var *ccv, uint16_t ack_type)
{
    struct ertt *e_t;
    int qdly;

    if (ack_type == CC_ACK) {
//ScenSim-Port//        e_t = khelp_get_osd(CCV(ccv, osd), ertt_id);
        e_t = &ccv->ertt_porting;                               //ScenSim-Port//

        if (e_t->rtt && e_t->minrtt && V_hd_qthresh > 0) {
            qdly = e_t->rtt - e_t->minrtt;

            if (qdly > V_hd_qmin &&
                !IN_RECOVERY(CCV(ccv, t_flags))) {
                /* Probabilistic backoff of cwnd. */
                if (should_backoff(qdly,
                    e_t->maxrtt - e_t->minrtt)) {
                    /*
                     * Update cwnd and ssthresh update to
                     * half cwnd and behave like an ECN (ie
                     * not a packet loss).
                     */
                    newreno_cc_algo.cong_signal(ccv,
                        CC_ECN);
                    return;
                }
            }
        }
    }
    newreno_cc_algo.ack_received(ccv, ack_type); /* As for NewReno. */
}

static int
hd_mod_init(void)
{

//ScenSim-Port//    ertt_id = khelp_get_id("ertt");
//ScenSim-Port//    if (ertt_id <= 0) {
//ScenSim-Port//        printf("%s: h_ertt module not found\n", __func__);
//ScenSim-Port//        return (ENOENT);
//ScenSim-Port//    }

    hd_cc_algo.after_idle = newreno_cc_algo.after_idle;
    hd_cc_algo.cong_signal = newreno_cc_algo.cong_signal;
    hd_cc_algo.post_recovery = newreno_cc_algo.post_recovery;

    return (0);
}

//ScenSim-Port//static int
//ScenSim-Port//hd_pmax_handler(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    int error;
//ScenSim-Port//    uint32_t new;
//ScenSim-Port//
//ScenSim-Port//    new = V_hd_pmax;
//ScenSim-Port//    error = sysctl_handle_int(oidp, &new, 0, req);
//ScenSim-Port//    if (error == 0 && req->newptr != NULL) {
//ScenSim-Port//        if (CAST_PTR_INT(req->newptr) == 0 ||
//ScenSim-Port//            CAST_PTR_INT(req->newptr) > 100)
//ScenSim-Port//            error = EINVAL;
//ScenSim-Port//        else
//ScenSim-Port//            V_hd_pmax = new;
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//    return (error);
//ScenSim-Port//}

//ScenSim-Port//static int
//ScenSim-Port//hd_qmin_handler(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    int error;
//ScenSim-Port//    uint32_t new;
//ScenSim-Port//
//ScenSim-Port//    new = V_hd_qmin;
//ScenSim-Port//    error = sysctl_handle_int(oidp, &new, 0, req);
//ScenSim-Port//    if (error == 0 && req->newptr != NULL) {
//ScenSim-Port//        if (CAST_PTR_INT(req->newptr) > V_hd_qthresh)
//ScenSim-Port//            error = EINVAL;
//ScenSim-Port//        else
//ScenSim-Port//            V_hd_qmin = new;
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//    return (error);
//ScenSim-Port//}

//ScenSim-Port//static int
//ScenSim-Port//hd_qthresh_handler(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    int error;
//ScenSim-Port//    uint32_t new;
//ScenSim-Port//
//ScenSim-Port//    new = V_hd_qthresh;
//ScenSim-Port//    error = sysctl_handle_int(oidp, &new, 0, req);
//ScenSim-Port//    if (error == 0 && req->newptr != NULL) {
//ScenSim-Port//        if (CAST_PTR_INT(req->newptr) < 1 ||
//ScenSim-Port//            CAST_PTR_INT(req->newptr) < V_hd_qmin)
//ScenSim-Port//            error = EINVAL;
//ScenSim-Port//        else
//ScenSim-Port//            V_hd_qthresh = new;
//ScenSim-Port//    }
//ScenSim-Port//
//ScenSim-Port//    return (error);
//ScenSim-Port//}

//ScenSim-Port//SYSCTL_DECL(_net_inet_tcp_cc_hd);
//ScenSim-Port//SYSCTL_NODE(_net_inet_tcp_cc, OID_AUTO, hd, CTLFLAG_RW, NULL,
//ScenSim-Port//    "Hamilton delay-based congestion control related settings");

//ScenSim-Port//SYSCTL_VNET_PROC(_net_inet_tcp_cc_hd, OID_AUTO, queue_threshold,
//ScenSim-Port//    CTLTYPE_UINT|CTLFLAG_RW, &VNET_NAME(hd_qthresh), 20, &hd_qthresh_handler,
//ScenSim-Port//    "IU", "queueing congestion threshold (qth) in ticks");

//ScenSim-Port//SYSCTL_VNET_PROC(_net_inet_tcp_cc_hd, OID_AUTO, pmax,
//ScenSim-Port//    CTLTYPE_UINT|CTLFLAG_RW, &VNET_NAME(hd_pmax), 5, &hd_pmax_handler,
//ScenSim-Port//    "IU", "per packet maximum backoff probability as a percentage");

//ScenSim-Port//SYSCTL_VNET_PROC(_net_inet_tcp_cc_hd, OID_AUTO, queue_min,
//ScenSim-Port//    CTLTYPE_UINT|CTLFLAG_RW, &VNET_NAME(hd_qmin), 5, &hd_qmin_handler,
//ScenSim-Port//    "IU", "minimum queueing delay threshold (qmin) in ticks");

//ScenSim-Port//DECLARE_CC_MODULE(hd, &hd_cc_algo);
//ScenSim-Port//MODULE_DEPEND(hd, ertt, 1, 1, 1);
}//namespace//                                                  //ScenSim-Port//
