//
// Default protocol patameters
//
#ifndef OLSRV2_PROTOCOLS_H_
#define OLSRV2_PROTOCOLS_H_

/*
 * OLSR Port
 */
#define DEFAULT_OLSRV1_PORT              (698)
#define DEFAULT_OLSRV2_PORT              ((unsigned short)(10698))

/*
 * @see rfc5498
 */
#define DEFAULT_MANET_UDP_PORT           (269)

/*
 * Link-local multicast address for MANET protocol packets.
 *
 * @see rfc5498
 */
#define LL_MANET_ROUTERS4                "224.0.0.109"
#define LL_MANET_ROUTERS6                "FF02:0:0:0:0:0:0:6D"

/*
 * Message Intervals
 */
#define DEFAULT_HELLO_INTERVAL           (2.0)                                          // [nhdp]
#define DEFAULT_REFRESH_INTERVAL         DEFAULT_HELLO_INTERVAL                         // [nhdp]
#define DEFAULT_HELLO_MIN_INTERVAL       (DEFAULT_HELLO_INTERVAL / 4)                   // [nhdp]
#define DEFAULT_HP_MAXJITTER             (DEFAULT_HELLO_INTERVAL / 4)                   // [nhdp]
#define DEFAULT_HT_MAXJITTER             (DEFAULT_HELLO_INTERVAL / 4)                   // [nhdp]

#define DEFAULT_TC_INTERVAL              (5.0)                                          // [olsrv2]
#define DEFAULT_TC_MIN_INTERVAL          (DEFAULT_TC_INTERVAL / 4)                      // [olsrv2]
#define DEFAULT_TP_MAXJITTER             DEFAULT_HP_MAXJITTER                           // [olsrv2]
#define DEFAULT_TT_MAXJITTER             DEFAULT_HT_MAXJITTER                           // [olsrv2]
#define DEFAULT_F_MAXJITTER              DEFAULT_TT_MAXJITTER                           // [olsrv2]
#define DEFAULT_TC_HOP_LIMIT             (10)                                           // [olsrv2]

/*
 * Holding Times
 */
#define DEFAULT_H_HOLD_TIME              (3 * DEFAULT_REFRESH_INTERVAL)              // [nhdp]
#define DEFAULT_L_HOLD_TIME              DEFAULT_H_HOLD_TIME                         // [nhdp]
#define DEFAULT_N_HOLD_TIME              DEFAULT_L_HOLD_TIME                         // [nhdp]
#define DEFAULT_I_HOLD_TIME              DEFAULT_N_HOLD_TIME                         // [nhdp]

#define DEFAULT_T_HOLD_TIME              (3 * DEFAULT_TC_INTERVAL)                   // [olsrv2]
#define DEFAULT_A_HOLD_TIME              DEFAULT_T_HOLD_TIME                         // [olsrv2]
#define DEFAULT_RX_HOLD_TIME             (30.0)                                      // [olsrv2]
#define DEFAULT_F_HOLD_TIME              (30.0)                                      // [olsrv2]
#define DEFAULT_P_HOLD_TIME              (30.0)                                      // [olsrv2]
#define DEFAULT_O_HOLD_TIME              (30.0)                                      // [olsrv2]

/*
 * Willingness
 */
#define WILLINGNESS__DEFAULT             (3)      // [olsrv2]
#define WILLINGNESS__NEVER               (0)      // [olsrv2]
#define WILLINGNESS__ALWAYS              (7)      // [olsrv2]

/*
 * Link Quality Parameters
 */
#define LQ__DEFAULT_HYST_SCALE           (0.5)
#define LQ__DEFAULT_HYST_ACCEPT          (0.8)
#define LQ__DEFAULT_HYST_REJECT          (0.3)
#define LQ__DEFAULT_INITIAL_QUALITY      (0.5)
#define LQ__DEFAULT_LOSS_DETECT_SCALE    (1.5)

#endif
