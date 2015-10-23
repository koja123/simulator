//
// (I) Local Interface
//
#ifndef OLSRV2_IBASE_I_H_
#define OLSRV2_IBASE_I_H_

#include "config.h" //ScenSim-Port://
#include "core/buf.h"
#include "olsrv2/pktq.h"
#ifdef USE_NETIO
#include "netio/sockaddr.h"
#endif
#include "olsrv2/stat.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_i OLSRv2 :: (I) Local Interface Tuple
 * @{
 */

/** */
struct olsrv2_stat_iface;

/**
 * (I) Local Interface Tuple
 */
typedef struct tuple_i {
    struct tuple_i*     next;          ///< next tuple
    struct tuple_i*     prev;          ///< prev tuple

    char*               name;          ///< iface name
    int                 iface_index;   ///< iface index

    nu_ip_set_t         local_ip_list; ///< I_local_iface_addr_list
    nu_bool_t           manet;         ///< I_manet

    //// Interface Information Base
    ibase_l_t           ibase_l;        ///< (L) Link Set
    ibase_rx_t          ibase_rx;       ///< (RX) Received Set

    double              hello_interval; ///< interface local hello interval
    double              hp_maxjitter;   ///< interface local hp maxjitter
    nu_time_t           last_hello_gen; ///< last HELLO send time

    //// Packet Queue
    olsrv2_pktq_t       recvq;                 ///< received msg queue
    olsrv2_pktq_t       sendq;                 ///< sending msg queue

    uint16_t            next_pkt_seqnum;       ///< Packet seqnum
    uint16_t            next_hello_msg_seqnum; ///< HELLO message seqnum

    ////
    nu_bool_t           configured; ///< configured flag
    nu_bool_t           change;     ///< change flag

    //// archdep parameters
    nu_bool_t           use_mcast;             ///< use mcast
    nu_ip_t             send_ip;               ///< send ip
    nu_ip_t             recv_ip;               ///< recv ip
    int                 ipv6_addr_type;        ///< IPv6 addr type
#ifdef USE_NETIO
    nu_sockaddr_t       send_sockaddr;         ///< sockaddr for sending packet
    nu_sockaddr_t       recv_sockaddr;         ///< sockaddr of receiving  packet
#ifdef linux
    int                 orig_send_redirects;   ///< original send redirects status
    int                 orig_accept_redirects; ///< original accept redirects status
    int                 orig_rp_filter;        ///< original spoof status
#endif
#endif
    int                 sock;               ///< socket
    size_t              mtu;                ///< mtu

#ifndef NUOLSRV2_NOSTAT
    olsrv2_stat_iface_t stat;               ///< stat information
#endif
} tuple_i_t;

/**
 * (I) Local Interface Set
 */
typedef struct ibase_i {
    tuple_i_t* next;   ///< first tuple
    tuple_i_t* prev;   ///< last tuple
    size_t     n;      ///< size
    nu_bool_t  change; ///< change flag
} ibase_i_t;

////////////////////////////////////////////////////////////////
//
// tuple_i_t
//

PUBLIC void tuple_i_add_local_ip(tuple_i_t*, nu_ip_t);

PUBLIC void tuple_i_recvq_enq(tuple_i_t*, nu_ibuf_t*);
PUBLIC nu_ibuf_t* tuple_i_recvq_deq(tuple_i_t*);
PUBLIC void tuple_i_sendq_enq(tuple_i_t*, nu_obuf_t*);
PUBLIC nu_obuf_t* tuple_i_sendq_deq(tuple_i_t*);

PUBLIC nu_obuf_t* tuple_i_build_packet(tuple_i_t*);
PUBLIC nu_obuf_t* tuple_i_send_packet(tuple_i_t*);
PUBLIC nu_bool_t tuple_i_configure(tuple_i_t*);        // archdep
PUBLIC nu_bool_t tuple_i_restore_settings(tuple_i_t*); // archdep

PUBLIC void tuple_i_put_log(tuple_i_t*, nu_logger_t*);

/** Gets local ip
 *
 * @param  tuple_i
 * @return the first ip address of the tuple_i's local_ip_list.
 */
PUBLIC_INLINE nu_ip_t
tuple_i_local_ip(const tuple_i_t* tuple_i)
{
    return nu_ip_set_head(&tuple_i->local_ip_list);
}

/** Adds ip.
 *
 * @param tuple_i
 * @param ip   the network address to add
 */
PUBLIC_INLINE void
tuple_i_add_ip(tuple_i_t* tuple_i, const nu_ip_t ip)
{
    tuple_i->change = true;
    nu_ip_set_add(&tuple_i->local_ip_list, ip);
}

/** Gets next sequence number.
 *
 * @param tuple_i
 * @return seqnum
 */
PUBLIC_INLINE uint16_t
tuple_i_get_next_seqnum(tuple_i_t* tuple_i)
{
    return tuple_i->next_pkt_seqnum++;
}

/** Gets next hello message's sequence number.
 *
 * @param tuple_i
 * @return seqnum
 */
PUBLIC_INLINE uint16_t
tuple_i_get_next_hello_msg_seqnum(tuple_i_t* tuple_i)
{
    return tuple_i->next_hello_msg_seqnum++;
}

////////////////////////////////////////////////////////////////
//
// ibase_i_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_i_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC void ibase_i_init(void);
PUBLIC void ibase_i_destroy(void);

PUBLIC tuple_i_t* ibase_i_add(const char*);
PUBLIC tuple_i_t* ibase_i_search_name(const char*);
PUBLIC tuple_i_t* ibase_i_search_index(int);
PUBLIC tuple_i_t* ibase_i_iter_remove(tuple_i_t*);

PUBLIC void ibase_i_put_log(nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
