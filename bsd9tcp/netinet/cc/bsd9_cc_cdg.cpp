/*-
 * Copyright (c) 2009-2011
 *  Swinburne University of Technology, Melbourne, Australia
 * All rights reserved.
 *
 * This software was developed at the Centre for Advanced Internet
 * Architectures, Swinburne University, by David Hayes
 *
 * This project has been made possible in part by a grant from the Cisco University
 * Research Program Fund at Community Foundation Silicon Valley. Testing and
 * development was further assisted by a grant from the FreeBSD Foundation.
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
 * CAIA Gradient Delay-Based CC
 *
 * Development is part of the CAIA NEWTCP project,
 * http://caia.swin.edu.au/urp/newtcp/
 *
 */

//ScenSim-Port//#include <sys/cdefs.h>
//ScenSim-Port//__FBSDID("$FreeBSD$");

//ScenSim-Port//#include <sys/param.h>
//ScenSim-Port//#include <sys/kernel.h>
//ScenSim-Port//#include <sys/hhook.h>
//ScenSim-Port//#include <sys/khelp.h>
//ScenSim-Port//#include <sys/limits.h>
//ScenSim-Port//#include <sys/lock.h>
//ScenSim-Port//#include <sys/malloc.h>
//ScenSim-Port//#include <sys/module.h>
//ScenSim-Port//#include <sys/queue.h>
//ScenSim-Port//#include <sys/socket.h>
//ScenSim-Port//#include <sys/socketvar.h>
//ScenSim-Port//#include <sys/sysctl.h>
//ScenSim-Port//#include <sys/systm.h>
//ScenSim-Port//#include <sys/limits.h>

//ScenSim-Port//#include <net/if.h>
//ScenSim-Port//#include <net/vnet.h>

//ScenSim-Port//#include <netinet/cc.h>
//ScenSim-Port//#include <netinet/khelp/h_ertt.h>
//ScenSim-Port//#include <netinet/tcp_seq.h>
//ScenSim-Port//#include <netinet/tcp_timer.h>
//ScenSim-Port//#include <netinet/tcp_var.h>

//ScenSim-Port//#include <netinet/cc/cc_module.h>

//ScenSim-Port//#include <vm/uma.h>
#include "tcp_porting.h"                     //ScenSim-Port//
namespace FreeBsd9Port {                                        //ScenSim-Port//

//ScenSim-Port//static VNET_DEFINE(uma_zone_t, qdiffsample_zone);
//ScenSim-Port//#define V_qdiffsample_zone  VNET(qdiffsample_zone)

//ScenSim-Port//#define CAST_PTR_INT(X) (*((int*)(X)))

/*
 * lookup up table for:
 *   (1-exp(-x))<<15, where x=0:2^-7:MAXGRAD
 */
#define EXP_PREC 15
#define EXP_INC_PREC 7
#define MAXGRAD 5
/* Note probexp[0] is set to 10 instead of 0
   as a safety for very low increase gradients */
const int probexp[641] = {
   10,255,508,759,1008,1255,1501,1744,1985,2225,2463,2698,2932,3165,3395,3624,
   3850,4075,4299,4520,4740,4958,5175,5389,5602,5814,6024,6232,6438,6643,6846,
   7048,7248,7447,7644,7839,8033,8226,8417,8606,8794,8981,9166,9350,9532,9713,
   9892,10070,10247,10422,10596,10769,10940,11110,11278,11445,11611,11776,11939,
   12101,12262,12422,12580,12737,12893,13048,13201,13354,13505,13655,13803,13951,
   14097,14243,14387,14530,14672,14813,14952,15091,15229,15365,15500,15635,15768,
   15900,16032,16162,16291,16419,16547,16673,16798,16922,17046,17168,17289,17410,
   17529,17648,17766,17882,17998,18113,18227,18340,18453,18564,18675,18784,18893,
   19001,19108,19215,19320,19425,19529,19632,19734,19835,19936,20036,20135,20233,
   20331,20427,20523,20619,20713,20807,20900,20993,21084,21175,21265,21355,21444,
   21532,21619,21706,21792,21878,21962,22046,22130,22213,22295,22376,22457,22537,
   22617,22696,22774,22852,22929,23006,23082,23157,23232,23306,23380,23453,23525,
   23597,23669,23739,23810,23879,23949,24017,24085,24153,24220,24286,24352,24418,
   24483,24547,24611,24675,24738,24800,24862,24924,24985,25045,25106,25165,25224,
   25283,25341,25399,25456,25513,25570,25626,25681,25737,25791,25846,25899,25953,
   26006,26059,26111,26163,26214,26265,26316,26366,26416,26465,26514,26563,26611,
   26659,26707,26754,26801,26847,26893,26939,26984,27029,27074,27118,27162,27206,
   27249,27292,27335,27377,27419,27460,27502,27543,27583,27624,27664,27703,27743,
   27782,27821,27859,27897,27935,27973,28010,28047,28084,28121,28157,28193,28228,
   28263,28299,28333,28368,28402,28436,28470,28503,28536,28569,28602,28634,28667,
   28699,28730,28762,28793,28824,28854,28885,28915,28945,28975,29004,29034,29063,
   29092,29120,29149,29177,29205,29232,29260,29287,29314,29341,29368,29394,29421,
   29447,29472,29498,29524,29549,29574,29599,29623,29648,29672,29696,29720,29744,
   29767,29791,29814,29837,29860,29882,29905,29927,29949,29971,29993,30014,30036,
   30057,30078,30099,30120,30141,30161,30181,30201,30221,30241,30261,30280,30300,
   30319,30338,30357,30376,30394,30413,30431,30449,30467,30485,30503,30521,30538,
   30555,30573,30590,30607,30624,30640,30657,30673,30690,30706,30722,30738,30753,
   30769,30785,30800,30815,30831,30846,30861,30876,30890,30905,30919,30934,30948,
   30962,30976,30990,31004,31018,31031,31045,31058,31072,31085,31098,31111,31124,
   31137,31149,31162,31174,31187,31199,31211,31223,31235,31247,31259,31271,31283,
   31294,31306,31317,31328,31339,31351,31362,31373,31383,31394,31405,31416,31426,
   31436,31447,31457,31467,31477,31487,31497,31507,31517,31527,31537,31546,31556,
   31565,31574,31584,31593,31602,31611,31620,31629,31638,31647,31655,31664,31673,
   31681,31690,31698,31706,31715,31723,31731,31739,31747,31755,31763,31771,31778,
   31786,31794,31801,31809,31816,31824,31831,31838,31846,31853,31860,31867,31874,
   31881,31888,31895,31902,31908,31915,31922,31928,31935,31941,31948,31954,31960,
   31967,31973,31979,31985,31991,31997,32003,32009,32015,32021,32027,32033,32038,
   32044,32050,32055,32061,32066,32072,32077,32083,32088,32093,32098,32104,32109,
   32114,32119,32124,32129,32134,32139,32144,32149,32154,32158,32163,32168,32173,
   32177,32182,32186,32191,32195,32200,32204,32209,32213,32217,32222,32226,32230,
   32234,32238,32242,32247,32251,32255,32259,32263,32267,32270,32274,32278,32282,
   32286,32290,32293,32297,32301,32304,32308,32311,32315,32318,32322,32325,32329,
   32332,32336,32339,32342,32346,32349,32352,32356,32359,32362,32365,32368,32371,
   32374,32377,32381,32384,32387,32389,32392,32395,32398,32401,32404,32407,32410,
   32412,32415,32418,32421,32423,32426,32429,32431,32434,32437,32439,32442,32444,
   32447,32449,32452,32454,32457,32459,32461,32464,32466,32469,32471,32473,32476,
   32478,32480,32482,32485,32487,32489,32491,32493,32495,32497,32500,32502,32504,
   32506,32508,32510,32512,32514,32516,32518,32520,32522,32524,32526,32527,32529,
   32531,32533,32535,32537,32538,32540,32542,32544,32545,32547};



//ScenSim-Port//#ifdef VIMAGE
//ScenSim-Port//#define HANDLE_UINT(oidp, arg1, arg2, req) \
//ScenSim-Port//    vnet_sysctl_handle_uint(oidp, arg1, arg2, req)
//ScenSim-Port//#else
//ScenSim-Port//#define HANDLE_UINT(oidp, arg1, arg2, req) \
//ScenSim-Port//    sysctl_handle_int(oidp, arg1, arg2, req)
//ScenSim-Port//#endif

#define D_P_E  EXP_INC_PREC /* Delay precision enhance - added bits for integer arithmetic precision */

struct qdiff_sample {
    STAILQ_ENTRY(qdiff_sample) qdiff_lnk;
    long qdiff;
};


struct cdg {
    long max_qtrend;
    long min_qtrend;
    STAILQ_HEAD(minrtts_head, qdiff_sample) qdiffmin_q;
    STAILQ_HEAD(maxrtts_head, qdiff_sample) qdiffmax_q;
    long window_incr;
    long rtt_count; /* rttcount for window increase when in congestion avoidance */
    int maxrtt_in_rtt; /* maximum measured rtt within an rtt period */
    int maxrtt_in_prevrtt; /* maximum measured rtt within prev rtt period */
    int minrtt_in_rtt; /* minimum measured rtt within an rtt period */
    int minrtt_in_prevrtt; /* minimum measured rtt within prev rtt period */
    u_int consec_cong_cnt; /* consecutive congestion episode counter */
    u_long shadow_w; /* when tracking a new reno type loss window */
    int sample_q_size; /* maximum number of samples in the moving average queue */
    int num_samples; /* number of samples in the moving average queue */
    int queue_state; /* estimate of the queue state of the path */
};

/* Queue state definitions */
#define CDG_Q_UNKNOWN   9999
#define CDG_Q_EMPTY 1
#define CDG_Q_RISING    2
#define CDG_Q_FALLING   3
#define CDG_Q_FULL  4

static int cdg_mod_init(void);
static void cdg_conn_init(struct cc_var *ccv);
static int cdg_cb_init(struct cc_var *ccv);
static void cdg_cb_destroy(struct cc_var *ccv);
static void cdg_cong_signal(struct cc_var *ccv, uint32_t signal_type);
static void cdg_ack_received(struct cc_var *ccv, uint16_t ack_type);

//ScenSim-Port//static int cdg_wdf_handler(SYSCTL_HANDLER_ARGS);
//ScenSim-Port//static int cdg_loss_wdf_handler(SYSCTL_HANDLER_ARGS);
//ScenSim-Port//static int cdg_exp_backoff_scale_handler(SYSCTL_HANDLER_ARGS);

//ScenSim-Port//MALLOC_DECLARE(M_CDG);
//ScenSim-Port//MALLOC_DEFINE(M_CDG, "cdg data",
//ScenSim-Port//  "Per connection data required for the CDG congestion algorithm");



struct cc_algo cdg_cc_algo = {
//ScenSim-Port//    .name = "cdg",
//ScenSim-Port//    .mod_init = cdg_mod_init,
//ScenSim-Port//    .ack_received = cdg_ack_received,
//ScenSim-Port//    .cb_destroy = cdg_cb_destroy,
//ScenSim-Port//    .cb_init = cdg_cb_init,
//ScenSim-Port//    .conn_init = cdg_conn_init,
//ScenSim-Port//    .cong_signal = cdg_cong_signal
    "cdg",                                                      //ScenSim-Port//
    cdg_mod_init,                                               //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    cdg_cb_init,                                                //ScenSim-Port//
    cdg_cb_destroy,                                             //ScenSim-Port//
    cdg_conn_init,                                              //ScenSim-Port//
    cdg_ack_received,                                           //ScenSim-Port//
    cdg_cong_signal,                                            //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
    NULL,                                                       //ScenSim-Port//
};

//ScenSim-Port//static int ertt_id;

//ScenSim-Port//static VNET_DEFINE(u_int, cdg_wif) = 0;
//ScenSim-Port//static VNET_DEFINE(u_int, cdg_wdf) = 50;
//ScenSim-Port//static VNET_DEFINE(u_int, cdg_loss_wdf) = 50;
//ScenSim-Port//static VNET_DEFINE(u_int, cdg_smoothing_factor) = 8;
//ScenSim-Port//static VNET_DEFINE(u_int, cdg_exp_backoff_scale) = 3;
//ScenSim-Port//static VNET_DEFINE(u_int, cdg_consec_cong) = 5;
//ScenSim-Port//static VNET_DEFINE(u_int, cdg_hold_backoff) = 5;

//ScenSim-Port//#define V_cdg_wif       VNET(cdg_wif)
//ScenSim-Port//#define V_cdg_wdf       VNET(cdg_wdf)
//ScenSim-Port//#define V_cdg_loss_wdf      VNET(cdg_loss_wdf)
//ScenSim-Port//#define V_cdg_smoothing_factor  VNET(cdg_smoothing_factor)
//ScenSim-Port//#define V_cdg_exp_backoff_scale     VNET(cdg_exp_backoff_scale)
//ScenSim-Port//#define V_cdg_consec_cong   VNET(cdg_consec_cong)
//ScenSim-Port//#define V_cdg_hold_backoff  VNET(cdg_hold_backoff)

#define WDF_RENO 50

static int
cdg_mod_init(void)
{
//ScenSim-Port//    ertt_id = khelp_get_id("ertt");
//ScenSim-Port//    if (ertt_id <= 0)
//ScenSim-Port//        return (EINVAL);

//ScenSim-Port//    V_cdg_wif = 0;
//ScenSim-Port//    V_cdg_wdf = 50;
//ScenSim-Port//    V_cdg_loss_wdf = 50;
//ScenSim-Port//    V_cdg_smoothing_factor = 8;
//ScenSim-Port//    V_cdg_exp_backoff_scale = 3;
//ScenSim-Port//    V_cdg_consec_cong = 5;
//ScenSim-Port//    V_cdg_hold_backoff = 5;
    V_qdiffsample_zone = uma_zcreate("qdiffsample", sizeof(struct qdiff_sample),
                     NULL, NULL, NULL, NULL, 0, 0);

    cdg_cc_algo.post_recovery = newreno_cc_algo.post_recovery;
    cdg_cc_algo.after_idle = newreno_cc_algo.after_idle;
    return (0);
}

/* Create struct to store CDG specific data */
static int
cdg_cb_init(struct cc_var *ccv)
{
    struct cdg *cdg_data;

//ScenSim-Port//    cdg_data = malloc(sizeof(struct cdg), M_CDG, M_NOWAIT);
    cdg_data = (struct cdg *)malloc(                            //ScenSim-Port//
        sizeof(struct cdg), M_CDG, M_NOWAIT);                   //ScenSim-Port//


    if (cdg_data == NULL)
        return (ENOMEM);

    cdg_data->shadow_w = 0;
    cdg_data->max_qtrend = 0;
    cdg_data->min_qtrend = 0;
    cdg_data->queue_state = CDG_Q_UNKNOWN;
    cdg_data->maxrtt_in_rtt = 0;
    cdg_data->maxrtt_in_prevrtt = 0;
    cdg_data->minrtt_in_rtt = INT_MAX;
    cdg_data->minrtt_in_prevrtt = 0;
    cdg_data->window_incr = 0;
    cdg_data->rtt_count = 0;
    cdg_data->consec_cong_cnt = 0;

    cdg_data->sample_q_size = V_cdg_smoothing_factor;
    cdg_data->num_samples = 0;

    STAILQ_INIT(&cdg_data->qdiffmin_q);
    STAILQ_INIT(&cdg_data->qdiffmax_q);

    ccv->cc_data = cdg_data;

    return (0);
}

static void
cdg_conn_init(struct cc_var *ccv)
{
//ScenSim-Port//    struct cdg *cdg_data = ccv->cc_data;
    struct cdg *cdg_data = (struct cdg *)ccv->cc_data;          //ScenSim-Port//

    /*
     * Initialise the shadow_cwnd in case we are competing
     * with loss based flows from the start
     */
    cdg_data->shadow_w = CCV(ccv,snd_cwnd);
}

/*
 * Free the struct used to store CDG specific data for the specified
 * TCP control block.
 */
static void
cdg_cb_destroy(struct cc_var *ccv)
{
//ScenSim-Port//    struct cdg *cdg_data = ccv->cc_data;
    struct cdg *cdg_data = (struct cdg *)ccv->cc_data;          //ScenSim-Port//
    struct qdiff_sample *qds, *qds_n;

    qds = STAILQ_FIRST(&cdg_data->qdiffmin_q);
    while (qds != NULL) {
        qds_n = STAILQ_NEXT(qds, qdiff_lnk);
        uma_zfree(V_qdiffsample_zone,qds);
        qds = qds_n;
    }
    qds = STAILQ_FIRST(&cdg_data->qdiffmax_q);
    while (qds != NULL) {
        qds_n = STAILQ_NEXT(qds, qdiff_lnk);
        uma_zfree(V_qdiffsample_zone,qds);
        qds = qds_n;
    }

    if (ccv->cc_data != NULL)
        free(ccv->cc_data, M_CDG);
}

//ScenSim-Port//static int
//ScenSim-Port//cdg_wdf_handler(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    if (req->newptr != NULL) {
//ScenSim-Port//        if(CAST_PTR_INT(req->newptr) < 1)
//ScenSim-Port//            return (EINVAL);
//ScenSim-Port//        else if (CAST_PTR_INT(req->newptr) > 99)
//ScenSim-Port//            return (EINVAL);
//ScenSim-Port//    }
//ScenSim-Port//    return (HANDLE_UINT(oidp, arg1, arg2, req));
//ScenSim-Port//}

//ScenSim-Port//static int
//ScenSim-Port//cdg_loss_wdf_handler(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    if (req->newptr != NULL) {
//ScenSim-Port//        if(CAST_PTR_INT(req->newptr) < 0)
//ScenSim-Port//            return (EINVAL);
//ScenSim-Port//        else if (CAST_PTR_INT(req->newptr) > 99)
//ScenSim-Port//            return (EINVAL);
//ScenSim-Port//    }
//ScenSim-Port//    return (HANDLE_UINT(oidp, arg1, arg2, req));
//ScenSim-Port//}


//ScenSim-Port//static int
//ScenSim-Port//cdg_exp_backoff_scale_handler(SYSCTL_HANDLER_ARGS)
//ScenSim-Port//{
//ScenSim-Port//    if (req->newptr != NULL) {
//ScenSim-Port//        if (CAST_PTR_INT(req->newptr) < 1)
//ScenSim-Port//            return (EINVAL);
//ScenSim-Port//    }
//ScenSim-Port//    return (HANDLE_UINT(oidp, arg1, arg2, req));
//ScenSim-Port//}

/*
 * Window decrease function
 * This allows multiplicative decreases of 1-w/2^wdf, where
 * wdf=1 is the window halving as in Reno. Wdf > 1 can be of benifit
 * to delay based algorithms which detect congestion much earlier than
 * loss based algorithms, which often have false positives, and for use
 * in high BDP paths.
 */
static u_long inline
cdg_window_decrease(struct cc_var *ccv, u_long owin, u_int wdf)
{
    u_long win;
    if (wdf > 0) {
        win = min(CCV(ccv,snd_wnd), owin) / CCV(ccv,t_maxseg);
        win -= max(((win*wdf)/100),1);
        return(max(win,2) * CCV(ccv,t_maxseg));
    } else
        return(owin);
}

/*
 * Window increase function
 * This window increase function is independent of the initial window size
 * to ensure small window flows are not descriminated against (ie fairness).
 * It increases at 1pkt/rtt like Reno for wif rtts, and then 2pkts/rtt for
 * the next wif rtts, and then 3pkts/rtt for the next wif rtts, etc.
 */
static void
cdg_window_increase(struct cc_var *ccv, struct cdg *cdg_data, int new_measurement)
{
    int incr = 0;
    int s_w_incr = 0;
    if (CCV(ccv,snd_cwnd) <= CCV(ccv,snd_ssthresh)) { /* normal Reno slow start */
        incr =  CCV(ccv,t_maxseg);
        s_w_incr = incr;
        cdg_data->window_incr=0;
        cdg_data->rtt_count=0;
    } else { /* congestion avoidance */
        if (new_measurement) {
            s_w_incr = CCV(ccv,t_maxseg);
            if (V_cdg_wif == 0) {
                incr = CCV(ccv,t_maxseg);
            } else {
                if (++cdg_data->rtt_count >= V_cdg_wif) {
                    cdg_data->window_incr++;
                    cdg_data->rtt_count=0;
                }
                incr = CCV(ccv,t_maxseg) * cdg_data->window_incr;
            }
        }
    }

    if (cdg_data->shadow_w > 0)
        cdg_data->shadow_w =
            min(cdg_data->shadow_w+s_w_incr,
            TCP_MAXWIN<<CCV(ccv,snd_scale));
    CCV(ccv,snd_cwnd) =
        min(CCV(ccv,snd_cwnd)+incr, TCP_MAXWIN<<CCV(ccv,snd_scale));
    assert(curvnet->ctx != 0);                                  //ScenSim-Port//
    UpdateCwndForStatistics(curvnet->ctx, CCV(ccv, snd_cwnd));  //ScenSim-Port//
}

#define CDG_DELAY 0x1000 /* private flag for a delay based congestion signal */

/* when congestion is detected */
static void
cdg_cong_signal(struct cc_var *ccv, uint32_t signal_type)
{
//ScenSim-Port//    struct cdg *cdg_data = ccv->cc_data;
    struct cdg *cdg_data = (struct cdg *)ccv->cc_data;          //ScenSim-Port//

    switch(signal_type) {
    case CDG_DELAY:
        CCV(ccv,snd_ssthresh) =
            cdg_window_decrease(ccv,CCV(ccv,snd_cwnd),V_cdg_wdf);
        CCV(ccv,snd_cwnd) = CCV(ccv,snd_ssthresh);
        assert(curvnet->ctx != 0);                              //ScenSim-Port//
        UpdateCwndForStatistics(                                //ScenSim-Port//
            curvnet->ctx, CCV(ccv, snd_cwnd));                  //ScenSim-Port//
        CCV(ccv,snd_recover) = CCV(ccv,snd_max);
        cdg_data->window_incr = 0; /* reset window increment */
        cdg_data->rtt_count = 0; /* reset rtt count */
        ENTER_CONGRECOVERY(CCV(ccv,t_flags));
        break;
    case CC_NDUPACK: /* ie packet loss */
        /*
         * If already responding to congestion OR
         * we have guessed no queue in the path is full
         */
        if (IN_RECOVERY(CCV(ccv,t_flags)) ||
            cdg_data->queue_state < CDG_Q_FULL) {
            CCV(ccv,snd_ssthresh) = CCV(ccv,snd_cwnd);
            CCV(ccv,snd_recover) = CCV(ccv,snd_max);
        } else {
            /* Loss is likely to be congestion related.  We have
             * inferred a queue full state, so have shaddow window
             * react to loss as New Reno would */
            if (cdg_data->shadow_w > 0)
                cdg_data->shadow_w =
                    cdg_window_decrease(ccv, cdg_data->shadow_w, WDF_RENO);

            CCV(ccv,snd_ssthresh) = max(cdg_data->shadow_w,
                cdg_window_decrease(ccv,
                CCV(ccv,snd_cwnd), V_cdg_loss_wdf));

            cdg_data->window_incr = 0; /* reset window increment */
            cdg_data->rtt_count = 0; /* reset rtt count */
        }
        ENTER_FASTRECOVERY(CCV(ccv,t_flags));
        break;
    default:
        newreno_cc_algo.cong_signal(ccv,signal_type);
    }
}


/*
 * Probabilistic backoff
 *
 * Using a negetaive exponential probabilistic backoff
 * so that sources with varying RTTs which share the
 * same link will, on average, have the same probability
 * of backoff over time.
 *
 * Prob_backoff = 1 - exp(-qtrend/V_cdg_exp_backoff_scale)
 * where V_cdg_exp_backoff_scale is the average qtrend for
 * the exponential backoff
 */
static int inline
prob_backoff(long qtrend)
{
    if (qtrend > ((MAXGRAD*V_cdg_exp_backoff_scale) << D_P_E)) {
        return (1); /* back off */
    } else {
        /* probability of backoff proportional to rate of queue growth */
        int p,idx;

        if (V_cdg_exp_backoff_scale > 1)
            idx = (qtrend + V_cdg_exp_backoff_scale/2)/V_cdg_exp_backoff_scale;
        else
            idx = qtrend;
        p = (INT_MAX / (1<<EXP_PREC)) * probexp[idx];
        return(random() < p);
    }
}

/* incremental moving average calculation */
static void inline
calc_moving_average(struct cdg *cdg_data, long qdiff_max, long qdiff_min)
{
    struct qdiff_sample *qds;

    ++cdg_data->num_samples;
    if (cdg_data->num_samples > cdg_data->sample_q_size) {
        /* minrtt */
        qds = STAILQ_FIRST(&cdg_data->qdiffmin_q);
        cdg_data->min_qtrend =  cdg_data->min_qtrend
            + (qdiff_min - qds->qdiff) / cdg_data->sample_q_size;
        STAILQ_REMOVE_HEAD(&cdg_data->qdiffmin_q, qdiff_lnk);
        qds->qdiff = qdiff_min;
        STAILQ_INSERT_TAIL(&cdg_data->qdiffmin_q, qds, qdiff_lnk);
        /* maxrtt */
        qds = STAILQ_FIRST(&cdg_data->qdiffmax_q);
        cdg_data->max_qtrend =  cdg_data->max_qtrend
            + (qdiff_max - qds->qdiff) / cdg_data->sample_q_size;
        STAILQ_REMOVE_HEAD(&cdg_data->qdiffmax_q, qdiff_lnk);
        qds->qdiff = qdiff_max;
        STAILQ_INSERT_TAIL(&cdg_data->qdiffmax_q, qds, qdiff_lnk);
        --cdg_data->num_samples;
    } else {
        qds =  (struct qdiff_sample *)
            uma_zalloc(V_qdiffsample_zone, M_NOWAIT);
        if (qds) {
            cdg_data->min_qtrend =
                cdg_data->min_qtrend +
                qdiff_min / cdg_data->sample_q_size;
            qds->qdiff = qdiff_min;
            STAILQ_INSERT_TAIL(&cdg_data->qdiffmin_q, qds, qdiff_lnk);
        }
        qds =  (struct qdiff_sample *)
            uma_zalloc(V_qdiffsample_zone, M_NOWAIT);
        if (qds) {
            cdg_data->max_qtrend =
                cdg_data->max_qtrend +
                qdiff_max / cdg_data->sample_q_size;
            qds->qdiff = qdiff_max;
            STAILQ_INSERT_TAIL(&cdg_data->qdiffmax_q, qds, qdiff_lnk);
        }
    }

}

/* dynamic threshold probabilistic congestion control detection and response */
static void
cdg_ack_received(struct cc_var *ccv, uint16_t ack_type)
{
//ScenSim-Port//    struct ertt *e_t = (struct ertt *)khelp_get_osd(CCV(ccv,osd),ertt_id);
    struct ertt *e_t = &ccv->ertt_porting;                      //ScenSim-Port//
    int congestion = 0;
//ScenSim-Port//    struct cdg *cdg_data = ccv->cc_data;
    struct cdg *cdg_data = (struct cdg *)ccv->cc_data;          //ScenSim-Port//
    int new_measurement = e_t->flags & ERTT_NEW_MEASUREMENT;

    cdg_data->maxrtt_in_rtt = imax(e_t->rtt, cdg_data->maxrtt_in_rtt);
    cdg_data->minrtt_in_rtt = imin(e_t->rtt, cdg_data->minrtt_in_rtt);

    if (new_measurement) {
        long qdiff_max, qdiff_min;
        int slowstart;
        slowstart = (CCV(ccv,snd_cwnd) <= CCV(ccv,snd_ssthresh));
        /*
         * Update smoothed gradient measurements.
         * Since we are only using one measurement per RTT use max or min rtt_in_rtt.
         * This is also less noisy than a sample rtt measurement.
         * Max rtt measurements can have trouble due to OS issues.
         */
        if (cdg_data->maxrtt_in_prevrtt) {
            qdiff_max =  ( (long) (cdg_data->maxrtt_in_rtt
                - cdg_data->maxrtt_in_prevrtt) << D_P_E );
            qdiff_min =  ( (long) (cdg_data->minrtt_in_rtt
                - cdg_data->minrtt_in_prevrtt) << D_P_E );

            calc_moving_average(cdg_data, qdiff_max, qdiff_min);

            /* probabilistic backoff with respect to gradient */
            if (slowstart && qdiff_min > 0)
                congestion = prob_backoff(qdiff_min);
            else if (cdg_data->min_qtrend > 0)
                congestion = prob_backoff(cdg_data->min_qtrend);
            else if (slowstart && qdiff_max > 0)
                congestion = prob_backoff(qdiff_max);
            else if (cdg_data->max_qtrend > 0)
                congestion = prob_backoff(cdg_data->max_qtrend);

            /* Update estimate of queue state */
            if (cdg_data->min_qtrend > 0 && cdg_data->max_qtrend <= 0) {
                cdg_data->queue_state = CDG_Q_FULL;
            } else if (cdg_data->min_qtrend >= 0 && cdg_data->max_qtrend < 0) {
                cdg_data->queue_state = CDG_Q_EMPTY;
                cdg_data->shadow_w = 0;
            } else if  (cdg_data->min_qtrend > 0 && cdg_data->max_qtrend > 0) {
                cdg_data->queue_state = CDG_Q_RISING;
            } else if  (cdg_data->min_qtrend < 0 && cdg_data->max_qtrend < 0) {
                cdg_data->queue_state = CDG_Q_FALLING;
            }

            if (cdg_data->min_qtrend < 0 || cdg_data->max_qtrend < 0)
                cdg_data->consec_cong_cnt = 0;

        }

        cdg_data->minrtt_in_prevrtt = cdg_data->minrtt_in_rtt;
        cdg_data->minrtt_in_rtt = INT_MAX;
        cdg_data->maxrtt_in_prevrtt = cdg_data->maxrtt_in_rtt;
        cdg_data->maxrtt_in_rtt = 0;

        e_t->flags &= ~ERTT_NEW_MEASUREMENT; /* reset measurement flag */
    }

    if (congestion) {
        cdg_data->consec_cong_cnt++;
        if (!IN_RECOVERY(CCV(ccv,t_flags))) {
            if (cdg_data->consec_cong_cnt < V_cdg_consec_cong)
                cdg_cong_signal(ccv, CDG_DELAY);
            else
                /* We have been backing off, but it is having no
                 * effect - the queue is not falling.  Assume we
                 * are competing with loss based flows, and
                 * don't back off for the next
                 * V_cdg_hold_backoff
                 */
                if (cdg_data->consec_cong_cnt >=
                    V_cdg_consec_cong + V_cdg_hold_backoff)
                    cdg_data->consec_cong_cnt = 0;

            cdg_data->maxrtt_in_prevrtt = 0; /* won't see effect until 2nd rtt */
            /* resync shadow_w, as we might be competing with a loss based flow */
            cdg_data->shadow_w = max(CCV(ccv,snd_cwnd),cdg_data->shadow_w);
        }
    } else if (ack_type == CC_ACK)
        cdg_window_increase(ccv,cdg_data,new_measurement);
}

//ScenSim-Port//SYSCTL_DECL(_net_inet_tcp_cc_cdg);
//ScenSim-Port//SYSCTL_NODE(_net_inet_tcp_cc, OID_AUTO, cdg, CTLFLAG_RW, NULL,
//ScenSim-Port//    "CAIA gradient-delay based congestion control related settings");

//ScenSim-Port//SYSCTL_VNET_PROC(_net_inet_tcp_cc_cdg, OID_AUTO, wdf,
//ScenSim-Port//    CTLTYPE_UINT|CTLFLAG_RW, &VNET_NAME(cdg_wdf), 1,
//ScenSim-Port//    &cdg_wdf_handler, "IU",
//ScenSim-Port//    "Window decrease factor (w = w - wdf*w/100)\n NB wdf >= 1");

//ScenSim-Port//SYSCTL_VNET_PROC(_net_inet_tcp_cc_cdg, OID_AUTO, loss_wdf,
//ScenSim-Port//    CTLTYPE_UINT|CTLFLAG_RW, &VNET_NAME(cdg_loss_wdf), 50,
//ScenSim-Port//    &cdg_loss_wdf_handler, "IU",
//ScenSim-Port//    "Percentage window decrease for packet loss");

//ScenSim-Port//SYSCTL_VNET_PROC(_net_inet_tcp_cc_cdg, OID_AUTO, exp_backoff_scale,
//ScenSim-Port//    CTLTYPE_UINT|CTLFLAG_RW, &VNET_NAME(cdg_exp_backoff_scale), 2,
//ScenSim-Port//    &cdg_exp_backoff_scale_handler, "IU",
//ScenSim-Port//    "Scaling parameter for the probabilistic exponential backoff");

//ScenSim-Port//SYSCTL_VNET_UINT(_net_inet_tcp_cc_cdg,  OID_AUTO, smoothing_factor,
//ScenSim-Port//    CTLFLAG_RW, &VNET_NAME(cdg_smoothing_factor), 4,
//ScenSim-Port//    "Number of samples used in the moving average smoothing (0 - no smoothing)");

//ScenSim-Port//SYSCTL_VNET_UINT(_net_inet_tcp_cc_cdg, OID_AUTO, wif,
//ScenSim-Port//    CTLFLAG_RW, &VNET_NAME(cdg_wif), 0,
//ScenSim-Port//    "Per RTT window increase factor (w = w + i, where i is incremented every wif rtts or i=1 if wif = 0)");

//ScenSim-Port//SYSCTL_VNET_UINT(_net_inet_tcp_cc_cdg, OID_AUTO, loss_compete_consec_cong,
//ScenSim-Port//    CTLFLAG_RW, &VNET_NAME(cdg_consec_cong), 3,
//ScenSim-Port//    "Number of consecutive delay gradient based congestion episodes which will trigger loss based CC compatibility");

//ScenSim-Port//SYSCTL_VNET_UINT(_net_inet_tcp_cc_cdg, OID_AUTO, loss_compete_hold_backoff,
//ScenSim-Port//    CTLFLAG_RW, &VNET_NAME(cdg_hold_backoff), 3,
//ScenSim-Port//    "Number of consecutive delay gradient based congestion episodes to hold the window backoff for loss based CC compatibility");

//ScenSim-Port//DECLARE_CC_MODULE(cdg, &cdg_cc_algo);
//ScenSim-Port//MODULE_DEPEND(cdg, ertt, 1, 1, 1);
}//namespace//                                                  //ScenSim-Port//
