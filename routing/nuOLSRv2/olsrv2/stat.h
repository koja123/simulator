#ifndef OLSRV2_STAT_H_
#define OLSRV2_STAT_H_

#include "config.h" //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

#ifndef NUOLSRV2_NOSTAT

/************************************************************//**
 * @addtogroup olsrv2
 * @{
 */

/** Maximum HOP count for logging */
#define MAX_HOPNUM    (5)

/**
 * Statistics information of each interface
 */
typedef struct olsrv2_stat_iface {
    uint32_t nhdpIfHelloMessageXmits;          ///< nhdpIfHelloMessageXmits
    uint32_t nhdpIfHelloMessageRecvd;          ///< nhdpIfHelloMessageRecvd
    uint32_t nhdpIfHelloMessagePeriodicXmits;  ///< nhdpIfHelloMessagePeriodicXmits
    uint32_t nhdpIfHelloMessageTriggeredXmits; ///< nhdpIfHelloMessageTriggeredXmits
} olsrv2_stat_iface_t;

PUBLIC void olsrv2_stat_iface_init(olsrv2_stat_iface_t*);
PUBLIC void olsrv2_stat_iface_destroy(olsrv2_stat_iface_t*);

/**
 * Cumulative time counter
 */
typedef struct olsrv2_stat_cum {
    nu_time_t start_time; ///< start time
    nu_time_t time;       ///< current time
    uint32_t  value;      ///< current value
    double    total;      ///< cummurative value
} olsrv2_stat_cum_t;

/**
 * Statistics information
 */
typedef struct olsrv2_stat {
    nu_time_t         start_time;       ///< stat start time
    double            interval;         ///< stat interval

    nu_ip_set_t       stat_ip_set;      ///< ip addresses that have olsrv2_stat_ip.

    uint32_t          num_pkt_send;     ///< sent packet counter
    uint32_t          num_pkt_recv;     ///< received packet counter
    uint32_t          send_pkt_size;    ///< sent packet size counter
    uint32_t          recv_pkt_size;    ///< received packet size counter

    uint32_t          num_msg_send;     ///< sent message counter
    uint32_t          num_msg_recv;     ///< received message counter
    uint32_t          send_msg_size;    ///< sent message size counter
    uint32_t          recv_msg_size;    ///< received message size counter

    uint32_t          hello_gen_num;    ///< sent HELLO message counter
    uint32_t          hello_gen_size;   ///< sent HELLO message size
    uint32_t          num_hello_recv;   ///< received HELLO message counter

    uint32_t          tc_gen_num;       ///< sent TC message counter
    uint32_t          tc_gen_size;      ///< sent TC message size
    uint32_t          num_tc_recv;      ///< recevied TC message counter
    uint32_t          num_tc_relayed;   ///< relayed TC message counter

    uint32_t          num_ansn_update;  ///< update count of my ansn

    uint32_t          num_calc_fmpr;    ///< flooding mpr calc. counter
    uint32_t          num_calc_rmpr;    ///< routing mpr calc. counter
    uint32_t          num_calc_route;   ///< route calc. counter
    uint32_t          num_flap_route;   ///< route flapping counter

    uint32_t          fmpr_calc_L;      ///< flooding mpr calc. by L counter
    uint32_t          fmpr_calc_N;      ///< flooding mpr calc. by N counter
    uint32_t          fmpr_calc_N2;     ///< flooding mpr calc. by N2 counter
    uint32_t          rmpr_calc_L;      ///< routing mpr calc. by L counter
    uint32_t          rmpr_calc_N;      ///< routing mpr calc. by N counter
    uint32_t          rmpr_calc_N2;     ///< routing mpr calc. by N2 counter

    uint32_t          route_calc_L;     ///< route calc. by L
    uint32_t          route_calc_sym_N; ///< route calc. by N
    uint32_t          route_calc_N2;    ///< route calc. by N2
    uint32_t          route_calc_TR;    ///< route calc. by TR
    uint32_t          route_calc_TA;    ///< route calc. by TA
    uint32_t          route_calc_AR;    ///< route calc. by AR
    uint32_t          route_calc_AN;    ///< route calc. by AN

    uint32_t          flap_L;           ///< route flapping by L
    uint32_t          flap_sym_N;       ///< route flapping by N
    uint32_t          flap_N2;          ///< route flapping by N2
    uint32_t          flap_TR;          ///< route flapping by TR
    uint32_t          flap_TA;          ///< route flapping by TA
    uint32_t          flap_AR;          ///< route flapping by AR
    uint32_t          flap_AN;          ///< route flapping by AN

    olsrv2_stat_cum_t num_advertised;   ///< advertised ip counter
    olsrv2_stat_cum_t num_fmpr;         ///< flooding mpr
    olsrv2_stat_cum_t num_rmpr;         ///< routing mpr
    olsrv2_stat_cum_t num_fmprs;        ///< flooding mpr selector
    olsrv2_stat_cum_t num_rmprs;        ///< routing mpr selector
    olsrv2_stat_cum_t num_sym_link;     ///< sym link
    olsrv2_stat_cum_t hops[MAX_HOPNUM]; ///< hop count
} olsrv2_stat_t;

struct tuple_i;
PUBLIC void olsrv2_stat_after_init_event(void);
PUBLIC void olsrv2_stat_after_hello_gen(const nu_obuf_t*, struct tuple_i*);
PUBLIC void olsrv2_stat_after_tc_gen(const nu_obuf_t*);
PUBLIC void olsrv2_stat_after_fmpr_calc(void);
PUBLIC void olsrv2_stat_after_rmpr_calc(void);
PUBLIC void olsrv2_stat_before_hello_process(void);
PUBLIC void olsrv2_stat_after_hello_process(struct tuple_i*, const nu_ip_t, uint16_t);
PUBLIC void olsrv2_stat_before_tc_relay(void);
PUBLIC void olsrv2_stat_before_tc_process(void);
PUBLIC void olsrv2_stat_before_route_calc(void);
PUBLIC void olsrv2_stat_after_route_calc(void);
PUBLIC void olsrv2_stat_after_msg_send(const nu_obuf_t*);
PUBLIC void olsrv2_stat_after_msg_recv(const nu_msg_t*);
PUBLIC void olsrv2_stat_after_pkt_recv(const nu_ibuf_t*);
PUBLIC void olsrv2_stat_after_pkt_send(const nu_obuf_t*);
PUBLIC void olsrv2_stat_update_ansn(void);
PUBLIC void olsrv2_stat_advertise_num(size_t);
PUBLIC void olsrv2_stat_init(void);
PUBLIC void olsrv2_stat_destroy(void);
PUBLIC void olsrv2_stat_put_log(void);

#else

#define olsrv2_stat_after_init_event()
#define olsrv2_stat_after_hello_gen(a, b)
#define olsrv2_stat_after_tc_gen(a)
#define olsrv2_stat_after_fmpr_calc()
#define olsrv2_stat_after_rmpr_calc()
#define olsrv2_stat_before_hello_process()
#define olsrv2_stat_after_hello_process(a, b, c)
#define olsrv2_stat_before_tc_relay()
#define olsrv2_stat_before_tc_process()
#define olsrv2_stat_before_route_calc()
#define olsrv2_stat_after_route_calc()
#define olsrv2_stat_after_msg_send(a)
#define olsrv2_stat_after_msg_recv(a)
#define olsrv2_stat_after_pkt_recv(a)
#define olsrv2_stat_after_pkt_send(a)
#define olsrv2_stat_update_ansn()
#define olsrv2_stat_advertise_num(s)    ;
#define olsrv2_stat_init()
#define olsrv2_stat_destroy()
#define olsrv2_stat_put_log()

#endif // NUOLSRV2_NOSTAT

/** @} */

}//namespace// //ScenSim-Port://

#endif
