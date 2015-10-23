#ifndef TCP_PORTING_H
#define TCP_PORTING_H

// Note: Tcp_Porting.h DOES NOT include Scenargie (ScenSim) headers and ScenSim types are only
//       from forwarding definitions.  "Bridge Code", i.e. code that has both Scenargie and
//       FreeBSD headers included is in the file: bsd9tcpglue.cpp.


#if defined(_WIN32) || defined(_WIN64)
#pragma warning(disable:4005) // 'identifier' : macro redefinition
#pragma warning(disable:4018) // 'expression' : signed/unsigned mismatch (level 3)
#pragma warning(disable:4244) // 'conversion' conversion from 'type1' to 'type2', possible loss of data (levels 3 and 4)
#pragma warning(disable:4267) // 'var' : conversion from 'size_t' to 'type', possible loss of data
#pragma warning(disable:4351) // new behavior: elements of array 'array' will be default initialized (level 1)
#endif

#define INET
//TBD: IPv6 real packet format is not supported
#define INET6

//Jay// #include <scensim_network.h>
//Jay// #include <scensim_transport.h>

#include <stddef.h>
#include <limits.h>
#include <memory.h>
#include <stdio.h>
#include <assert.h>

//Jay Note: Mixing Free BSD C code and C++ STL and Boost is scary.

#include <vector>
#include <memory>

#include <stdint.h>

namespace ScenSim {
    class Packet;
    class TcpProtocolImplementation;
    class TcpConnectionImplementation;
};

extern uint16_t NetToHost16(uint16_t net16);
extern uint32_t NetToHost32(uint32_t net32);
extern uint16_t HostToNet16(uint16_t host16);
extern uint32_t HostToNet32(uint32_t host32);

#if defined(_WIN32) || defined(_WIN64)
#define __func__ __FUNCTION__
#undef errno
#endif

#define __packed
#define __aligned(x)

#if 0 //for debug
#undef assert
#if defined(_WIN32) || defined(_WIN64)
#define assert(exp) do { if (!(exp)) { __asm int 3 } } while(0)
#else
#define assert(exp) do { if (!(exp)) { asm("int $3"); } } while(0)
#endif
#endif

//------------------------------------------------------------------------------
// errno
//------------------------------------------------------------------------------
#include <errno.h>


namespace FreeBsd9Port {

//------------------------------------------------------------------------------
// common types
//------------------------------------------------------------------------------

typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef char *caddr_t;
typedef uint8_t sa_family_t;
typedef uint16_t in_port_t;
typedef uint32_t in_addr_t;

//------------------------------------------------------------------------------
// system headers
//------------------------------------------------------------------------------

#include "sys/bsd9_queue.h"
#include "sys/bsd9_md5.h"

//------------------------------------------------------------------------------
// byte order
//------------------------------------------------------------------------------

#if defined(_WIN32) || defined(_WIN64)
#define LITTLE_ENDIAN 1234
#define BIG_ENDIAN 4321
#define BYTE_ORDER LITTLE_ENDIAN
#endif

#undef htonl
#undef htons
#undef ntohl
#undef ntohs

#define htonl(x) HostToNet32(x)
#define htons(x) HostToNet16(x)
#define ntohl(x) NetToHost32(x)
#define ntohs(x) NetToHost16(x)

// Note: Definitions only for Windows msvs2008 except "EHOSTDOWN" needed for msvs2010.
//       Added 100 to values to prevent conflicts.


#ifndef EPERM
#define EPERM         101     /* Operation not permitted */
#endif

#ifndef ENOENT
#define ENOENT        102     /* No such file or directory */
#endif

#ifndef ENOMEM
#define ENOMEM        112     /* Cannot allocate memory */
#endif

#ifndef EINVAL
#define EINVAL        122     /* Invalid argument */
#endif

#ifndef EAGAIN
#define EAGAIN        135     /* Resource temporarily unavailable */
#endif

#ifndef EWOULDBLOCK
#define EWOULDBLOCK  EAGAIN /* Operation would block */
#endif

#ifndef EALREADY
#define EALREADY     137     /* Operation already in progress */
#endif

#ifndef EMSGSIZE
#define EMSGSIZE     140     /* Message too long */
#endif

#ifndef ENOPROTOOPT
#define ENOPROTOOPT  142     /* Protocol not available */
#endif

#ifndef EOPNOTSUPP
#define EOPNOTSUPP   145     /* Operation not supported */
#endif

#ifndef EAFNOSUPPORT
#define EAFNOSUPPORT 147     /* Address family not supported by protocol family */
#endif

#ifndef EADDRINUSE
#define EADDRINUSE   148     /* Address already in use */
#endif

#ifndef ENETDOWN
#define ENETDOWN      150     /* Network is down */
#endif

#ifndef ENETUNREACH
#define ENETUNREACH   151     /* Network is unreachable */
#endif

#ifndef ECONNABORTED
#define ECONNABORTED  153     /* Software caused connection abort */
#endif

#ifndef ECONNRESET
#define ECONNRESET    154     /* Connection reset by peer */
#endif

#ifndef ENOBUFS
#define ENOBUFS      155     /* No buffer space available */
#endif

#ifndef EISCONN
#define EISCONN      156     /* Socket is already connected */
#endif

#ifndef ENOTCONN
#define ENOTCONN     157     /* Socket is not connected */
#endif

#ifndef ETIMEDOUT
#define ETIMEDOUT     160     /* Operation timed out */
#endif

#ifndef ECONNREFUSED
#define ECONNREFUSED  161     /* Connection refused */
#endif

#ifndef EHOSTDOWN
#define EHOSTDOWN     164     /* Host is down */   //msvc2010
#endif

#ifndef EHOSTUNREACH
#define EHOSTUNREACH  165     /* No route to host */
#endif



//------------------------------------------------------------------------------
// common macros
//------------------------------------------------------------------------------

#define roundup(x, y) ((((x)+((y)-1))/(y))*(y))
#define powerof2(x) ((((x)-1)&(x))==0)

//------------------------------------------------------------------------------
// atomic operations
//------------------------------------------------------------------------------

#define atomic_add_int(P, V) (*(u_int *)(P) += (V))
#define atomic_subtract_int(P, V) (*(u_int *)(P) -= (V))

//------------------------------------------------------------------------------
// comparison functions
//------------------------------------------------------------------------------

static inline u_int max(u_int a, u_int b) { return (a > b ? a : b); }
static inline u_int min(u_int a, u_int b) { return (a < b ? a : b); }
static inline int imax(int a, int b) { return (a > b ? a : b); }
static inline int imin(int a, int b) { return (a < b ? a : b); }
static inline long lmin(long a, long b) { return (a < b ? a : b); }
static inline u_long ulmin(u_long a, u_long b) { return (a < b ? a : b); }

//------------------------------------------------------------------------------
// external variables
//------------------------------------------------------------------------------

extern const int hz;

extern int ticks;
extern time_t time_uptime;

extern u_char inetctlerrmap[];

#ifdef INET6
extern const struct in6_addr in6addr_any;
extern const struct sockaddr_in6 sa6_any;
extern u_char inet6ctlerrmap[];
extern struct pr_usrreqs tcp6_usrreqs;
#endif

extern struct vnet *curvnet;
extern struct vnet *saved_vnet;


//------------------------------------------------------------------------------
// memory allocation
//------------------------------------------------------------------------------

#define M_CDG       NULL //Not Used//
#define M_CHD       NULL //Not Used//
#define M_CUBIC     NULL //Not Used//
#define M_HOSTCACHE NULL //Not Used//
#define M_HTCP      NULL //Not Used//
#define M_SYNCACHE  NULL //Not Used//
#define M_TCPLOG    NULL //Not Used//
#define M_TEMP      NULL //Not Used//
#define M_VEGAS     NULL //Not Used//

#define M_NOWAIT 0x0001 //Not Used//
#define M_WAITOK 0x0002 //Not Used//
#define M_ZERO   0x0100 //Not Used//

void *malloc(unsigned long size, struct malloc_type *type, int flags);
void free(void *addr, struct malloc_type *type);

//------------------------------------------------------------------------------
// uma
//------------------------------------------------------------------------------

struct uma_zone {
    size_t size;
    int count;
    int limit;
};

typedef struct uma_zone *uma_zone_t;

#define uma_zcreate(name, size, ctor, dtor, uminit, fini, align, flags) \
    uma_zcreate_porting(size)
uma_zone_t uma_zcreate_porting(size_t size);
void uma_zdestroy(uma_zone_t zone);
#define uma_zalloc(zone, flags) uma_zalloc_porting(zone)
void *uma_zalloc_porting(uma_zone_t zone);
void uma_zfree(uma_zone_t zone, void *item);

static inline int uma_zone_set_max(uma_zone_t zone, int nitems)
{
    zone->limit = nitems;
    return zone->limit;
}

//------------------------------------------------------------------------------
// mbuf
//------------------------------------------------------------------------------

#define CSUM_DATA_VALID 0x0400 /* csum_data field is valid */
#define CSUM_PSEUDO_HDR 0x0800 /* csum_data has pseudo hdr */

//ScenSim-Port//#define M_EXT    0x00000001 /* has associated external storage */
#define M_PKTHDR 0x00000002 /* start of record */
//ScenSim-Port//#define M_FLOWID 0x00400000 /* deprecated: flowid is valid */
#define M_BCAST  0x00000200 /* send/received as link-level broadcast */
#define M_MCAST  0x00000400 /* send/received as link-level multicast */

void m_adj(struct mbuf *mp, int req_len);
struct mbuf *m_pullup(struct mbuf *, int);
void m_copydata(const struct mbuf *, int, int, caddr_t);
#define m_copy(m, o, l) m_copy_porting((m), (o), (l))
struct mbuf *m_copy_porting(struct mbuf *, int, int);

#define CSUM_TCP 0x0002 /* will csum TCP */

#define MT_DATA   1       /* dynamic (data) allocation */
#define MT_HEADER MT_DATA /* packet header, use M_PKTHDR instead */

class mbuf_porting {
public:
    mbuf_porting(
        std::shared_ptr<std::vector<unsigned char> >& vector, int vlen)
        : m_vector(vector), m_packet(NULL), m_vlen(vlen) {}
    mbuf_porting(ScenSim::Packet* packet, int off)
        : m_vector(), m_packet(packet), m_off(off) {}

    ~mbuf_porting();
    //Jay// {
    //Jay//     if (m_packet != NULL) {
    //Jay//         assert(m_vector.get() == NULL);
    //Jay//         delete m_packet;
    //Jay//     }
    //Jay//     else {
    //Jay//         assert(m_vector.get() != NULL);
    //Jay//     }
    //Jay// }

    int datalen();
    //Jay// {
    //Jay//     if (m_packet != NULL) {
    //Jay//         assert(m_vector.get() == NULL);
    //Jay//         return m_packet->LengthBytes() - m_off;
    //Jay//     }
    //Jay//     else {
    //Jay//         assert(m_vector.get() != NULL);
    //Jay//         return m_vector->size();
    //Jay//     }
    //Jay// }

    int vdatalen();

    caddr_t databuf();
    //Jay// {
    //Jay//     if (m_packet != NULL) {
    //Jay//         assert(m_vector.get() == NULL);
    //Jay//         return (caddr_t)m_packet->GetRawPayloadData() + m_off;
    //Jay//     }
    //Jay//     else {
    //Jay//         assert(m_vector.get() != NULL);
    //Jay//         return (caddr_t)&(*m_vector)[0];
    //Jay//     }
    //Jay// }

    void get_data(
        const unsigned int dataBegin,
        const unsigned int dataEnd,
        unsigned int& length,
        const unsigned char*& rawFragmentDataPtr,
        unsigned int& nextOffset) const;

private:
    std::shared_ptr<std::vector<unsigned char> > m_vector;
    ScenSim::Packet* m_packet;
    int m_off;
    int m_vlen;
};

struct pkthdr {
    int len;
    int csum_flags;
    int csum_data;
};

struct m_hdr {
    struct mbuf *mh_next;
//ScenSim-Port//    struct mbuf *mh_nextpkt;
    caddr_t mh_data;
    int mh_len;
    int mh_flags;
//ScenSim-Port//    short mh_type;
//ScenSim-Port//    uint8_t pad[M_HDR_PAD];
};

struct mbuf {
    struct m_hdr m_hdr;
//ScenSim-Port//    union {
//ScenSim-Port//        struct {
    struct pkthdr m_pkthdr;
//ScenSim-Port//            union {
//ScenSim-Port//                struct m_ext MH_ext;
//ScenSim-Port//                char MH_databuf[MHLEN];
//ScenSim-Port//            } MH_dat;
//ScenSim-Port//        } MH;
//ScenSim-Port//        char M_databuf[MLEN];
//ScenSim-Port//    } M_dat;
    bool m_extptr;
    caddr_t m_databuf;
    int m_datalen;
    int m_vdatalen;
    mbuf_porting *m_datasource;
};

#define m_next    m_hdr.mh_next
#define m_len     m_hdr.mh_len
#define m_data    m_hdr.mh_data
//ScenSim-Port//#define m_type    m_hdr.mh_type
#define m_flags   m_hdr.mh_flags
//ScenSim-Port//#define m_nextpkt m_hdr.mh_nextpkt
//ScenSim-Port//#define m_act     m_nextpkt
//ScenSim-Port//#define m_pkthdr  M_dat.MH.MH_pkthdr
//ScenSim-Port//#define m_ext     M_dat.MH.MH_dat.MH_ext
//ScenSim-Port//#define m_pktdat  M_dat.MH.MH_dat.MH_databuf
//ScenSim-Port//#define m_dat     M_dat.M_databuf

struct mbuf *m_get(mbuf_porting *data, short type);
#define MGETHDR(m, how, type) ((m) = m_gethdr((how), (type)))
#define m_gethdr(how, type) m_gethdr_porting(type)
struct mbuf *m_gethdr_porting(short type);
struct mbuf *m_free(struct mbuf *m);
void m_freem(struct mbuf *m);
#define mtod(m, t) ((t)((m)->m_data))

#ifdef INET6
#define MHLEN 100 // IPv6 header (max:40 bytes, IP options are not supported) +
                  // TCP header (max:60 bytes)
#else
#define MHLEN 80 // IP header (max:20 bytes, IP options are not supported) +
                 // TCP header (max:60 bytes)
#endif

//------------------------------------------------------------------------------
// checksum
//------------------------------------------------------------------------------

u_short in_pseudo(u_int32_t a, u_int32_t b, u_int32_t c);
#define in_cksum(m, len) in_cksum_skip(m, len, 0)
u_short in_cksum_skip(struct mbuf *m, int len, int skip);
#ifdef INET6
int in6_cksum(struct mbuf *m, u_int8_t nxt, u_int32_t off, u_int32_t len);
#endif

//------------------------------------------------------------------------------
// protosw
//------------------------------------------------------------------------------

#define PRC_IFDOWN            0 /* interface transition */
#define PRC_ROUTEDEAD         1 /* select new route if possible ??? */
#define PRC_IFUP              2 /* interface has come back up */
#define PRC_QUENCH2           3 /* DEC congestion bit says slow down */
#define PRC_QUENCH            4 /* some one said to slow down */
#define PRC_MSGSIZE           5 /* message size forced drop */
#define PRC_HOSTDEAD          6 /* host appears to be down */
#define PRC_HOSTUNREACH       7 /* deprecated (use PRC_UNREACH_HOST) */
#define PRC_UNREACH_NET       8 /* no route to network */
#define PRC_UNREACH_HOST      9 /* no route to host */
#define PRC_UNREACH_PROTOCOL 10 /* dst says bad protocol */
#define PRC_UNREACH_PORT     11 /* bad port # */
/* was PRC_UNREACH_NEEDFRAG 12 (use PRC_MSGSIZE) */
#define PRC_UNREACH_SRCFAIL  13 /* source route failed */
#define PRC_REDIRECT_NET     14 /* net routing redirect */
#define PRC_REDIRECT_HOST    15 /* host routing redirect */
#define PRC_REDIRECT_TOSNET  16 /* redirect for type of service & net */
#define PRC_REDIRECT_TOSHOST 17 /* redirect for tos & host */
#define PRC_TIMXCEED_INTRANS 18 /* packet lifetime expired in transit */
#define PRC_TIMXCEED_REASS   19 /* lifetime expired on reass q */
#define PRC_PARAMPROB        20 /* header incorrect */
#define PRC_UNREACH_ADMIN_PROHIB 21 /* packet administrativly prohibited */
#define PRC_NCMDS            22

#define PRC_IS_REDIRECT(cmd) \
    ((cmd) >= PRC_REDIRECT_NET && (cmd) <= PRC_REDIRECT_TOSHOST)

#define PRU_ATTACH     0
#define PRU_DETACH     1
#define PRU_BIND       2
#define PRU_LISTEN     3
#define PRU_CONNECT    4
#define PRU_ACCEPT     5
#define PRU_DISCONNECT 6
#define PRU_SHUTDOWN   7
#define PRU_RCVD       8
#define PRU_SEND       9
#define PRU_ABORT      10
#define PRU_CONTROL    11
#define PRU_SENSE      12
#define PRU_RCVOOB     13
#define PRU_SENDOOB    14
#define PRU_SOCKADDR   15
#define PRU_PEERADDR   16
#define PRU_CONNECT2   17
#define PRU_FASTTIMO   18
#define PRU_SLOWTIMO   19
#define PRU_PROTORCV   20
#define PRU_PROTOSEND  21
#define PRU_SEND_EOF   22
#define PRU_SOSETLABEL 23
#define PRU_CLOSE      24
#define PRU_FLUSH      25
#define PRU_NREQ       25

#define PR_FASTHZ 5 /* 5 fast timeouts per second */

struct pr_usrreqs {
    void (*pru_abort)(struct socket *so);
    int (*pru_accept)(struct socket *so, struct sockaddr **nam);
    int (*pru_attach)(struct socket *so, int proto, struct thread *td);
    int (*pru_bind)(struct socket *so, struct sockaddr *nam,
        struct thread *td);
    int (*pru_connect)(struct socket *so, struct sockaddr *nam,
        struct thread *td);
    int (*pru_connect2)(struct socket *so1, struct socket *so2);
    int (*pru_control)(struct socket *so, u_long cmd, caddr_t data);
    void (*pru_detach)(struct socket *so);
    int (*pru_disconnect)(struct socket *so);
    int (*pru_listen)(struct socket *so, int backlog,
        struct thread *td);
    int (*pru_peeraddr)(struct socket *so, struct sockaddr **nam);
    int (*pru_rcvd)(struct socket *so, int flags);
    int (*pru_rcvoob)(struct socket *so, struct mbuf *m, int flags);
    int (*pru_send)(struct socket *so, int flags, struct mbuf *m,
        struct sockaddr *addr, struct mbuf *control, struct thread *td);
#define PRUS_OOB        0x1
#define PRUS_EOF        0x2
#define PRUS_MORETOCOME 0x4
    int (*pru_sense)(struct socket *so, struct stat *sb);
    int (*pru_shutdown)(struct socket *so);
    int (*pru_flush)(struct socket *so, int direction);
    int (*pru_sockaddr)(struct socket *so, struct sockaddr **nam);
    int (*pru_sosend)(struct socket *so, struct sockaddr *addr,
        struct uio *uio, struct mbuf *top, struct mbuf *control,
        int flags, struct thread *td);
    int (*pru_soreceive)(struct socket *so, struct sockaddr **paddr,
        struct uio *uio, struct mbuf **mp0, struct mbuf **controlp,
        int *flagsp);
    int (*pru_sopoll)(struct socket *so, int events,
        struct ucred *cred, struct thread *td);
    void (*pru_sosetlabel)(struct socket *so);
    void (*pru_close)(struct socket *so);
};

//------------------------------------------------------------------------------
// sockbuf
//------------------------------------------------------------------------------

#define SBS_CANTSENDMORE 0x0010 /* can't send more data to peer */
#define SBS_CANTRCVMORE  0x0020 /* can't receive more data from peer */
#define SBS_RCVATMARK    0x0040 /* at mark on input */

#define SB_AUTOSIZE 0x800 /* automatically size socket buffer */

struct sockbuf {
//ScenSim-Port//    struct selinfo sb_sel; /* process selecting read/write */
//ScenSim-Port//    struct mtx sb_mtx; /* sockbuf lock */
//ScenSim-Port//    struct sx sb_sx; /* prevent I/O interlacing */
    short sb_state;
    struct mbuf *sb_mb; /* (c/d) the mbuf chain */
    struct mbuf *sb_mbtail; /* (c/d) the last mbuf in the chain */
//ScenSim-Port//    struct mbuf *sb_lastrecord; /* (c/d) first mbuf of last
//ScenSim-Port//                                 * record in socket buffer */
//ScenSim-Port//    struct mbuf *sb_sndptr; /* (c/d) pointer into mbuf chain */
//ScenSim-Port//    u_int sb_sndptroff; /* (c/d) byte offset of ptr into chain */
    u_int sb_cc; /* (c/d) actual chars in buffer */
    u_int sb_hiwat;
//ScenSim-Port//    u_int sb_mbcnt; /* (c/d) chars of mbufs used */
//ScenSim-Port//    u_int sb_mcnt; /* (c/d) number of mbufs in buffer */
//ScenSim-Port//    u_int sb_ccnt; /* (c/d) number of clusters in buffer */
//ScenSim-Port//    u_int sb_mbmax; /* (c/d) max chars of mbufs to use */
//ScenSim-Port//    u_int sb_ctl; /* (c/d) non-data chars in buffer */
//ScenSim-Port//    int sb_lowat; /* (c/d) low water mark */
//ScenSim-Port//    int sb_timeo; /* (c/d) timeout for read/write */
    short sb_flags; /* (c/d) flags, see below */
//ScenSim-Port//    int (*sb_upcall)(struct socket *, void *, int); /* (c/d) */
//ScenSim-Port//    void *sb_upcallarg; /* (c/d) */
};

#define sbspace(sb) ((long)(sb)->sb_hiwat - (sb)->sb_cc)
#define sbdrop(sb, len) sbdrop_locked(sb, len)
#define sbflush(sb) sbdrop_locked((sb), (sb)->sb_cc)
void sbdrop_locked(struct sockbuf *sb, int len);
#define sbreserve_locked(sb, cc, so, td) sbreserve_locked_porting(sb, cc)
int sbreserve_locked_porting(struct sockbuf *sb, u_long cc);
#define sbappendstream(sb, m) sbappendstream_locked(sb, m)
void sbappendstream_locked(struct sockbuf *sb, struct mbuf *m);

//------------------------------------------------------------------------------
// socket
//------------------------------------------------------------------------------

#define MSG_PEEK 0x2 /* peek at incoming message */

struct sockaddr {
    unsigned char sa_len;
    sa_family_t sa_family;
//ScenSim-Port//    char sa_data[14]; /* actually longer; address value */
};

//ScenSim-Port//#define SO_DEBUG        0x0001 /* turn on debugging info recording */
#define SO_ACCEPTCONN   0x0002 /* socket has had listen() */
//ScenSim-Port//#define SO_REUSEADDR    0x0004 /* allow local address reuse */
#define SO_KEEPALIVE    0x0008 /* keep connections alive */
#define SO_DONTROUTE    0x0010 /* just use interface addresses */
//ScenSim-Port//#define SO_BROADCAST    0x0020 /* permit sending of broadcast msgs */
//ScenSim-Port//#define SO_USELOOPBACK  0x0040 /* bypass hardware when possible */
//ScenSim-Port//#define SO_LINGER       0x0080 /* linger on close if data present */
#define SO_OOBINLINE    0x0100 /* leave received OOB data in line */
//ScenSim-Port//#define SO_REUSEPORT    0x0200 /* allow local address & port reuse */
//ScenSim-Port//#define SO_TIMESTAMP    0x0400 /* timestamp received dgram traffic */
//ScenSim-Port//#define SO_NOSIGPIPE    0x0800 /* no SIGPIPE from EPIPE */
//ScenSim-Port//#define SO_ACCEPTFILTER 0x1000 /* there is an accept filter */
//ScenSim-Port//#define SO_BINTIME      0x2000 /* timestamp received dgram traffic */
//ScenSim-Port//#define SO_NO_OFFLOAD   0x4000 /* socket cannot be offloaded */
//ScenSim-Port//#define SO_NO_DDP       0x8000 /* disable direct data placement */

#define SS_NOFDREF         0x0001 /* no file table ref any more */
#define SS_ISCONNECTED     0x0002 /* socket connected to a peer */
#define SS_ISCONNECTING    0x0004 /* in process of connecting to peer */
#define SS_ISDISCONNECTING 0x0008 /* in process of disconnecting */
//ScenSim-Port//#define SS_NBIO            0x0100 /* non-blocking ops */
//ScenSim-Port//#define SS_ASYNC           0x0200 /* async i/o notify */
//ScenSim-Port//#define SS_ISCONFIRMING    0x0400 /* deciding to accept connection req */
#define SS_ISDISCONNECTED  0x2000 /* socket disconnected from peer */
#define SS_PROTOREF        0x4000 /* strong protocol reference */

#define SQ_INCOMP 0x0800
#define SQ_COMP   0x1000

struct socket {
//ScenSim-Port//    int so_count; /* (b) reference count */
//ScenSim-Port//    short so_type; /* (a) generic type, see socket.h */
    short so_options;
//ScenSim-Port//    short so_linger; /* time to linger while closing */
    short so_state;
    int so_qstate; /* (e) internal state flags SQ_* */
    void *so_pcb; /* protocol control block */
    struct vnet *so_vnet; /* network stack instance */
//ScenSim-Port//    struct protosw *so_proto; /* (a) protocol handle */
    struct pr_usrreqs *so_usrreqs;
    struct socket *so_head; /* (e) back pointer to listen socket */
    TAILQ_HEAD(, socket) so_incomp; /* (e) queue of partial unaccepted connections */
    TAILQ_HEAD(, socket) so_comp; /* (e) queue of complete unaccepted connections */
    TAILQ_ENTRY(socket) so_list; /* (e) list of unaccepted connections */
    u_short so_qlen; /* (e) number of unaccepted connections */
    u_short so_incqlen; /* (e) number of unaccepted incomplete
                           connections */
    u_short so_qlimit; /* (e) max number queued connections */
//ScenSim-Port//    short so_timeo; /* (g) connection timeout */
    u_short so_error; /* (f) error affecting connection */
//ScenSim-Port//     struct sigio *so_sigio; /* [sg] information for async I/O or
//ScenSim-Port//                                out of band data (SIGURG) */
    u_long so_oobmark; /* (c) chars to oob mark */
//ScenSim-Port//    TAILQ_HEAD(, aiocblist) so_aiojobq; /* AIO ops waiting on socket */
    struct sockbuf so_rcv, so_snd;
//ScenSim-Port//    struct ucred *so_cred; /* (a) user credentials */
//ScenSim-Port//    struct label *so_label; /* (b) MAC label for socket */
//ScenSim-Port//    struct label *so_peerlabel; /* (b) cached MAC label for peer */
//ScenSim-Port//    so_gen_t so_gencnt; /* (h) generation count */
//ScenSim-Port//    void *so_emuldata; /* (b) private data for emulators */
//ScenSim-Port//    struct so_accf {
//ScenSim-Port//        struct accept_filter *so_accept_filter;
//ScenSim-Port//        void *so_accept_filter_arg; /* saved filter args */
//ScenSim-Port//        char *so_accept_filter_str; /* saved user args */
//ScenSim-Port//    } *so_accf;
//ScenSim-Port//    int so_fibnum; /* routing domain for this socket */
//ScenSim-Port//    uint32_t so_user_cookie;
    int so_acked;
    ScenSim::TcpConnectionImplementation *so_ctx;
};

#define AF_INET 2
#ifdef INET6
#define AF_INET6 28
#endif

int socreate(int dom, struct socket **aso);
struct socket *sonewconn(struct socket *head, int connstatus);
int sobind(struct socket *so, struct sockaddr *nam);
int solisten(struct socket *so, int backlog);
int solisten_proto_check(struct socket *so);
void solisten_proto(struct socket *so, int backlog);
void sofree(struct socket *so);
int soclose(struct socket *so);
void soabort(struct socket *so);
int soaccept(struct socket *so);
int soconnect(struct socket *so, struct sockaddr *nam);
int sodisconnect(struct socket *so);
void soisconnecting(struct socket *so);
void soisconnected(struct socket *so);
void soisdisconnecting(struct socket *so);
void soisdisconnected(struct socket *so);
void socantrcvmore(struct socket *so);
void socantsendmore(struct socket *so);
#define sowwakeup(so) sowwakeup_locked(so)
void sorwakeup_locked(struct socket *so);
void sowwakeup_locked(struct socket *so);
int soreserve(struct socket *so, u_long sndcc, u_long rcvcc);
void sohasoutofband(struct socket *so);
int sosend(struct socket *so, int flags, struct mbuf *m);

//------------------------------------------------------------------------------
// address
//------------------------------------------------------------------------------

#define INET_ADDRSTRLEN 16
#define IPPROTO_TCP 6

#define INADDR_ANY (u_int32_t)0x00000000
#define INADDR_BROADCAST (u_int32_t)0xffffffff /* must be masked */

#define IN_CLASSD(i) (((u_int32_t)(i) & 0xf0000000) == 0xe0000000)
#define IN_MULTICAST(i) IN_CLASSD(i)

#define IPPORT_EPHEMERALFIRST 10000
#define IPPORT_EPHEMERALLAST  65535

struct in_addr {
    in_addr_t s_addr;
};

struct sockaddr_in {
    uint8_t sin_len;
    sa_family_t sin_family;
    in_port_t sin_port;
    struct in_addr sin_addr;
//ScenSim-Port//    char sin_zero[8];
};

struct in6_addr {
    union {
        uint8_t __u6_addr8[16];
        uint16_t __u6_addr16[8];
        uint32_t __u6_addr32[4];
    } __u6_addr;
};

#define s6_addr   __u6_addr.__u6_addr8
#define s6_addr8  __u6_addr.__u6_addr8
#define s6_addr16 __u6_addr.__u6_addr16
#define s6_addr32 __u6_addr.__u6_addr32

#ifdef INET6
#define INET6_ADDRSTRLEN 46

#define IN6_IS_ADDR_V4MAPPED(a)        \
    ((*(const u_int32_t *)(const void *)(&(a)->s6_addr[0]) == 0) && \
     (*(const u_int32_t *)(const void *)(&(a)->s6_addr[4]) == 0) && \
     (*(const u_int32_t *)(const void *)(&(a)->s6_addr[8]) == ntohl(0x0000ffff)))

#define IN6ADDR_ANY_INIT \
    {{{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }}}

#define IN6_IS_ADDR_UNSPECIFIED(a) \
    ((*(const u_int32_t *)(const void *)(&(a)->s6_addr[0]) == 0) && \
     (*(const u_int32_t *)(const void *)(&(a)->s6_addr[4]) == 0) && \
     (*(const u_int32_t *)(const void *)(&(a)->s6_addr[8]) == 0) && \
     (*(const u_int32_t *)(const void *)(&(a)->s6_addr[12]) == 0))

#define IN6_ARE_ADDR_EQUAL(a, b)   \
    (memcmp(&(a)->s6_addr[0], &(b)->s6_addr[0], sizeof(struct in6_addr)) == 0)

#define IN6_IS_ADDR_MULTICAST(a) ((a)->s6_addr[0] == 0xff)

struct sockaddr_in6 {
    uint8_t sin6_len;
    sa_family_t sin6_family;
    in_port_t sin6_port;
    uint32_t sin6_flowinfo;
    struct in6_addr sin6_addr;
    uint32_t sin6_scope_id;
};
#endif

int in_localaddr(struct in_addr);
#define in_broadcast(in, ifp) in_broadcast_porting(in)
int in_broadcast_porting(struct in_addr in);
char *inet_ntoa_r(struct in_addr ina, char *buf); /* in libkern */
#ifdef INET6
void in6_sin6_2_sin(struct sockaddr_in *sin, struct sockaddr_in6 *sin6);
int in6_localaddr(struct in6_addr *);
#endif

//------------------------------------------------------------------------------
// inpcb
//------------------------------------------------------------------------------

#define INC_ISIPV6    0x01

#define INP_IPV4      0x1
#define INP_IPV6      0x2
#define INP_IPV6PROTO 0x4

#define INPLOOKUP_WILDCARD 0x00000001 /* Allow wildcard sockets. */
//ScenSim-Port//#define INPLOOKUP_RLOCKPCB 0x00000002 /* Return inpcb read-locked. */
#define INPLOOKUP_WLOCKPCB 0x00000004 /* Return inpcb write-locked. */

#define INP_RECVOPTS       0x00000001 /* receive incoming IP options */
#define INP_RECVRETOPTS    0x00000002 /* receive IP options for reply */
#define INP_RECVDSTADDR    0x00000004 /* receive IP dst address */
#define INP_HDRINCL        0x00000008 /* user supplies entire IP header */
#define INP_HIGHPORT       0x00000010 /* user wants "high" port binding */
#define INP_LOWPORT        0x00000020 /* user wants "low" port binding */
#define INP_ANONPORT       0x00000040 /* port chosen for user */
#define INP_RECVIF         0x00000080 /* receive incoming interface */
#define INP_MTUDISC        0x00000100 /* user can do MTU discovery */
#define INP_FAITH          0x00000200 /* accept FAITH'ed connections */
#define INP_RECVTTL        0x00000400 /* receive incoming IP TTL */
#define INP_DONTFRAG       0x00000800 /* don't fragment packet */
#define INP_BINDANY        0x00001000 /* allow bind to any address */
#define INP_INHASHLIST     0x00002000 /* in_pcbinshash() has been called */
#define IN6P_IPV6_V6ONLY   0x00008000 /* restrict AF_INET6 socket for v6 */
#define IN6P_PKTINFO       0x00010000 /* receive IP6 dst and I/F */
#define IN6P_HOPLIMIT      0x00020000 /* receive hoplimit */
#define IN6P_HOPOPTS       0x00040000 /* receive hop-by-hop options */
#define IN6P_DSTOPTS       0x00080000 /* receive dst options after rthdr */
#define IN6P_RTHDR         0x00100000 /* receive routing header */
#define IN6P_RTHDRDSTOPTS  0x00200000 /* receive dstoptions before rthdr */
#define IN6P_TCLASS        0x00400000 /* receive traffic class value */
#define IN6P_AUTOFLOWLABEL 0x00800000 /* attach flowlabel automatically */
#define INP_TIMEWAIT       0x01000000 /* in TIMEWAIT, ppcb is tcptw */
#define INP_ONESBCAST      0x02000000 /* send all-ones broadcast */
#define INP_DROPPED        0x04000000 /* protocol drop flag */
#define INP_SOCKREF        0x08000000 /* strong socket reference */
#define INP_SW_FLOWID      0x10000000 /* software generated flow id */
#define INP_HW_FLOWID      0x20000000 /* hardware generated flow id */
#define IN6P_RFC2292       0x40000000 /* used RFC2292 API on the socket */
#define IN6P_MTU           0x80000000 /* receive path MTU */

#define INP_CONTROLOPTS \
    (INP_RECVOPTS|INP_RECVRETOPTS|INP_RECVDSTADDR|INP_RECVIF|INP_RECVTTL| \
    IN6P_PKTINFO|IN6P_HOPLIMIT|IN6P_HOPOPTS|IN6P_DSTOPTS|IN6P_RTHDR| \
    IN6P_RTHDRDSTOPTS|IN6P_TCLASS|IN6P_AUTOFLOWLABEL|IN6P_RFC2292|IN6P_MTU)

#define in6pcb inpcb

struct in_addr_4in6 {
    u_int32_t ia46_pad32[3];
    struct in_addr ia46_addr4;
};

struct in_endpoints {
    u_int16_t ie_fport;
    u_int16_t ie_lport;
    union {
        struct in_addr_4in6 ie46_foreign;
        struct in6_addr ie6_foreign;
    } ie_dependfaddr;
    union {
        struct in_addr_4in6 ie46_local;
        struct in6_addr ie6_local;
    } ie_dependladdr;
};
#define ie_faddr   ie_dependfaddr.ie46_foreign.ia46_addr4
#define ie_laddr   ie_dependladdr.ie46_local.ia46_addr4
#define ie6_faddr  ie_dependfaddr.ie6_foreign
#define ie6_laddr  ie_dependladdr.ie6_local

struct in_conninfo {
    u_int8_t inc_flags;
//ScenSim-Port//    u_int8_t inc_len;
//ScenSim-Port//    u_int16_t inc_fibnum;
    struct  in_endpoints inc_ie;
};
#define inc_fport  inc_ie.ie_fport
#define inc_lport  inc_ie.ie_lport
#define inc_faddr  inc_ie.ie_faddr
#define inc_laddr  inc_ie.ie_laddr
#define inc6_faddr inc_ie.ie6_faddr
#define inc6_laddr inc_ie.ie6_laddr

typedef LIST_ENTRY(inpcb) inpcb_list_entry;

typedef struct {
    u_char inp4_ip_tos;
    struct mbuf *inp4_options;
//ScenSim-Port//    struct ip_moptions *inp4_moptions; /* (i) IP mcast options */
} inp_depend4_t;

typedef struct {
    struct mbuf *inp6_options;
    struct ip6_pktopts *inp6_outputopts;
//ScenSim-Port//    struct ip6_moptions *inp6_moptions;
//ScenSim-Port//    struct icmp6_filter *inp6_icmp6filt;
//ScenSim-Port//    int inp6_cksum;
    short inp6_hops;
} inp_depend6_t;

struct inpcb {
//ScenSim-Port//    LIST_ENTRY(inpcb) inp_hash; /* (i/p) hash list */
//ScenSim-Port//    LIST_ENTRY(inpcb) inp_pcbgrouphash; /* (g/i) hash list */
    inpcb_list_entry inp_list; /* (i/p) list for all PCBs for proto */
    void *inp_ppcb;
    struct inpcbinfo *inp_pcbinfo; /* (c) PCB list info */
//ScenSim-Port//    struct inpcbgroup *inp_pcbgroup; /* (g/i) PCB group list */
//ScenSim-Port//    LIST_ENTRY(inpcb) inp_pcbgroup_wild; /* (g/i/p) group wildcard entry */
    struct socket *inp_socket;
//ScenSim-Port//    struct ucred *inp_cred; /* (c) cache of socket cred */
    u_int32_t inp_flow;
    int inp_flags;
//ScenSim-Port//    int inp_flags2;  /* (i) generic IP/datagram flags #2*/
    u_char inp_vflag;
    u_char inp_ip_ttl;
//ScenSim-Port//    u_char inp_ip_p;  /* (c) protocol proto */
    u_char inp_ip_minttl;  /* (i) minimum TTL or drop */
//ScenSim-Port//    uint32_t inp_flowid;  /* (x) flow id / queue id */
//ScenSim-Port//    u_int inp_refcount;  /* (i) refcount */
//ScenSim-Port//    void *inp_pspare[5];  /* (x) route caching / general use */
//ScenSim-Port//    u_int inp_ispare[6];  /* (x) route caching / user cookie /
    struct in_conninfo inp_inc;
//ScenSim-Port//    struct label *inp_label; /* (i) MAC label */
//ScenSim-Port//    struct inpcbpolicy *inp_sp;    /* (s) for IPSEC */
//ScenSim-Port//    struct {
//ScenSim-Port//        u_char inp4_ip_tos;
//ScenSim-Port//        struct mbuf *inp4_options;
//ScenSim-Port//     struct ip_moptions *inp4_moptions; /* (i) IP mcast options */
//ScenSim-Port//    } inp_depend4;
    inp_depend4_t inp_depend4;
//ScenSim-Port//    struct {
//ScenSim-Port//        struct mbuf *inp6_options;
//ScenSim-Port//        struct ip6_pktopts *inp6_outputopts;
//ScenSim-Port//     struct ip6_moptions *inp6_moptions;
//ScenSim-Port//     struct icmp6_filter *inp6_icmp6filt;
//ScenSim-Port//     int inp6_cksum;
//ScenSim-Port//     short inp6_hops;
//ScenSim-Port//    } inp_depend6;
    inp_depend6_t inp_depend6;
//ScenSim-Port//    LIST_ENTRY(inpcb) inp_portlist; /* (i/p) */
//ScenSim-Port//    struct inpcbport *inp_phd; /* (i/p) head of this list */
//ScenSim-Port//#define inp_zero_size offsetof(struct inpcb, inp_gencnt)
//ScenSim-Port//    inp_gen_t inp_gencnt; /* (c) generation count */
//ScenSim-Port//    struct llentry *inp_lle; /* cached L2 information */
//ScenSim-Port//    struct rtentry *inp_rt; /* cached L3 information */
//ScenSim-Port//    struct rwlock inp_lock;
};

#define inp_fport       inp_inc.inc_fport
#define inp_lport       inp_inc.inc_lport
#define inp_faddr       inp_inc.inc_faddr
#define inp_laddr       inp_inc.inc_laddr
#define inp_ip_tos      inp_depend4.inp4_ip_tos
#define inp_options     inp_depend4.inp4_options
#define inp_moptions    inp_depend4.inp4_moptions

#define in6p_faddr      inp_inc.inc6_faddr
#define in6p_laddr      inp_inc.inc6_laddr
#define in6p_hops       inp_depend6.inp6_hops
#define in6p_flowinfo   inp_flow
#define in6p_options    inp_depend6.inp6_options
#define in6p_outputopts inp_depend6.inp6_outputopts
#define in6p_moptions   inp_depend6.inp6_moptions
#define in6p_icmp6filt  inp_depend6.inp6_icmp6filt
#define in6p_cksum      inp_depend6.inp6_cksum

#define inp_vnet        inp_pcbinfo->ipi_vnet

struct inpcbinfo {
    struct inpcbhead *ipi_listhead;
    struct vnet *ipi_vnet;
};

LIST_HEAD(inpcbhead, inpcb);

#define sotoinpcb(so) ((struct inpcb *)(so)->so_pcb)
#define in_pcbinfo_init( \
    pcbinfo, name, listhead, hash_nelements, porthash_nelements, inpcbzone_name, \
    inpcbzone_init, inpcbzone_fini, inpcbzone_flags, hashfields) \
    in_pcbinfo_init_porting((pcbinfo), (listhead))
void in_pcbinfo_init_porting(
    struct inpcbinfo *pcbinfo, struct inpcbhead *listhead);
void in_pcbinfo_destroy(struct inpcbinfo *pcbinfo);
#define in_pcbbind(inp, nam, cred) in_pcbbind_porting(inp, nam)
int in_pcbbind_porting(struct inpcb *inp, struct sockaddr *nam);
#define in_pcbconnect_setup(inp, nam, laddrp, lportp, faddrp, fportp, oinpp, cred) \
    in_pcbconnect_setup_porting(inp, nam, laddrp, lportp, faddrp, fportp, oinpp)
int in_pcbconnect_setup_porting(
    struct inpcb *inp, struct sockaddr *nam, in_addr_t *laddrp, u_short *lportp,
    in_addr_t *faddrp, u_short *fportp, struct inpcb **oinpp);
#define in_pcbconnect_mbuf(inp, nam, cred, m) \
    in_pcbconnect_mbuf_porting(inp, nam, m)
int in_pcbconnect_mbuf_porting(
    struct inpcb *inp, struct sockaddr *nam, struct mbuf *m);
#define in_pcblookup_mbuf(pcbinfo, faddr, fport, laddr, lport, lookupflags, ifp, m) \
    in_pcblookup_porting(pcbinfo, faddr, fport, laddr, lport, lookupflags)
#define in_pcblookup(pcbinfo, faddf, fport, laddr, lport, lookupflags, ifp) \
    in_pcblookup_porting(pcbinfo, faddf, fport, laddr, lport, lookupflags)
struct inpcb *in_pcblookup_porting(
    struct inpcbinfo *pcbinfo, struct in_addr faddr, u_int fport,
    struct in_addr laddr, u_int lport, int lookupflags);
void in_pcbdrop(struct inpcb *inp);
void in_pcbfree(struct inpcb *inp);
void in_pcbnotifyall(
    struct inpcbinfo *pcbinfo, struct in_addr faddr, int errnoVal,
    struct inpcb *(*notify)(struct inpcb *, int));
void in_pcbdetach(struct inpcb *inp);
struct sockaddr *in_sockaddr(in_port_t port, struct in_addr *addr);
int in_pcballoc(struct socket *so, struct inpcbinfo *pcbinfo);

#ifdef INET6
#define in6_pcbconnect_mbuf(inp, nam, cred, m) \
    in6_pcbconnect_mbuf_porting(inp, nam, m)
int in6_pcbconnect_mbuf_porting(
    struct inpcb *inp, struct sockaddr *nam, struct mbuf *m);
#define in6_pcblookup_mbuf(pcbinfo, faddr, fport, laddr, lport, lookupflags, ifp, m) \
    in6_pcblookup_mbuf_porting(pcbinfo, faddr, fport, laddr, lport, lookupflags)
#define in6_pcblookup_hash_locked(pcbinfo, faddr, fport, laddr, lport, lookupflags, ifp) \
    in6_pcblookup_mbuf_porting(pcbinfo, faddr, fport, laddr, lport, lookupflags)
struct inpcb *in6_pcblookup_mbuf_porting(
    struct inpcbinfo *pcbinfo, struct in6_addr *faddr, u_int fport,
    struct in6_addr *laddr, u_int lport, int lookupflags);
void in6_pcbnotify(struct inpcbinfo *, struct sockaddr *,
      u_int, const struct sockaddr *, u_int, int, void *,
      struct inpcb *(*)(struct inpcb *, int));
#define in6_pcbbind(inp, nam, cred) in6_pcbbind_porting(inp, nam)
int in6_pcbbind_porting(struct inpcb *inp, struct sockaddr *nam);
int in6_pcbladdr(struct inpcb *, struct sockaddr *, struct in6_addr *);

#endif

//------------------------------------------------------------------------------
// random
//------------------------------------------------------------------------------

#define RANDOM_MAX (INT_MAX - 1)

u_long random(void);
int read_random(void *buf, int count);
uint32_t arc4random(void);

//------------------------------------------------------------------------------
// callout
//------------------------------------------------------------------------------

#define CALLOUT_ACTIVE  0x0002
#define CALLOUT_PENDING 0x0004

struct callout {
    int c_time;
    void *c_arg;
    void (*c_func)(void *);
    int c_flags;
    int c_index;
};

#define callout_init(c, flags) callout_init_porting(c)
#define callout_init_mtx(c, mtx, flags) callout_init_porting(c)
void callout_init_porting(struct callout *c);
#define callout_reset_on(c, on_tick, fn, arg, cpu) \
    callout_reset(c, on_tick, fn, arg)
int callout_reset(struct callout *c, int on_tick, void (*fn)(void *), void *arg);
int callout_stop(struct callout *c);
#define callout_drain(c) callout_stop(c)
#define callout_active(c) ((c)->c_flags & CALLOUT_ACTIVE)
#define callout_deactivate(c) ((c)->c_flags &= ~CALLOUT_ACTIVE)
#define callout_pending(c) ((c)->c_flags & CALLOUT_PENDING)

//------------------------------------------------------------------------------
// system functions
//------------------------------------------------------------------------------

#define LOG_INFO  6
#define LOG_DEBUG 7

void log(int level, const char *format, ...);
void bcopy(const void *src, void *dst, size_t len);
void bzero(void *buf, size_t len);
void panic(const char *fmt, ...);

//------------------------------------------------------------------------------
// output
//------------------------------------------------------------------------------

int ip_output(
    struct mbuf *m, struct mbuf *opt, struct route *ro, int flags,
    struct ip_moptions *imo, struct inpcb *inp, int off, long len);

//------------------------------------------------------------------------------
// ip
//------------------------------------------------------------------------------

#define IP_ROUTETOIF SO_DONTROUTE

struct ipovly {
    u_char ih_x1[9];
    u_char ih_pr;
    u_short ih_len;
    struct in_addr ih_src;
    struct in_addr ih_dst;
};

#define MAX_IPOPTLEN 40
struct ipoption {
//ScenSim-Port//    struct in_addr ipopt_dst; /* first-hop dst if source routed */
    char ipopt_list[MAX_IPOPTLEN]; /* options proper */
};

#ifdef INET6
struct ip6ctlparam {
    struct mbuf *ip6c_m;            /* start of mbuf chain */
//ScenSim-Port//    struct icmp6_hdr *ip6c_icmp6;   /* icmp6 header of target packet */
    struct ip6_hdr *ip6c_ip6;       /* ip6 header of target packet */
    int ip6c_off;                   /* offset of the target proto header */
    struct sockaddr_in6 *ip6c_src;  /* srcaddr w/ additional info */
//ScenSim-Port//    struct sockaddr_in6 *ip6c_dst;  /* (final) dstaddr w/ additional info */
//ScenSim-Port//    struct in6_addr *ip6c_finaldst; /* final destination address */
//ScenSim-Port//    void *ip6c_cmdarg;              /* control command dependent data */
//ScenSim-Port//    u_int8_t ip6c_nxt;              /* final next header field */
};
#endif

struct mbuf *ip_srcroute(struct mbuf *);
void ip_stripoptions(struct mbuf *, struct mbuf *);
#ifdef INET6
#define ip6_copypktopts(src, canwait) ip6_copypktopts_porting(src)
struct ip6_pktopts *ip6_copypktopts_porting(struct ip6_pktopts *src);
u_int32_t ip6_randomflowlabel(void);
int ip6_optlen(struct inpcb *);
char *ip6_sprintf(char *, const struct in6_addr *);
void in6_losing(struct inpcb *inp);
int in6_selecthlim(struct in6pcb *in6p, struct ifnet *ifp);
void nd6_nud_hint(struct rtentry *rt, struct in6_addr *dst6, int force);
#endif

//------------------------------------------------------------------------------
// icmp
//------------------------------------------------------------------------------

#define BANDLIM_UNLIMITED      -1
#define BANDLIM_RST_CLOSEDPORT  3 /* No connection, and no listeners */
#define BANDLIM_RST_OPENPORT    4 /* No connection, listener */

int badport_bandlim(int);

//------------------------------------------------------------------------------
// included headers
//------------------------------------------------------------------------------

#include <netinet/bsd9_ip.h>
#include <netinet/bsd9_ip6.h>
#include <netinet/bsd9_ip_icmp.h>
#include <netinet/khelp/bsd9_h_ertt.h>
#include <netinet/bsd9_tcp.h>
#include <netinet/bsd9_cc.h>
#include <netinet/cc/bsd9_cc_module.h>
#include <netinet/cc/bsd9_cc_cubic.h>
#include <netinet/bsd9_tcpip.h>
#include <netinet/bsd9_tcp_var.h>
#include <netinet/bsd9_tcp_fsm.h>
#include <netinet/bsd9_tcp_hostcache.h>
#include <netinet/bsd9_tcp_offload.h>
#include <netinet/bsd9_tcp_seq.h>
#include <netinet/bsd9_tcp_syncache.h>
#include <netinet/bsd9_tcp_timer.h>
#include <netinet/bsd9_tcp_debug.h>

//------------------------------------------------------------------------------
// debug
//------------------------------------------------------------------------------

#define DEBUG_LOG_APPEND_SB(sb, m) \
    do { if (debug_sockbuf) { assert(curvnet->ctx); \
         curvnet->ctx->OutputDebugLogForAppendSockBuf(sb, m); } } while(0)
#define DEBUG_ASSERT_SB_CC(sb) \
    do { if (debug_sockbuf) assert_sb_cc(sb); } while(0)

//------------------------------------------------------------------------------
// vnet
//------------------------------------------------------------------------------

#define CURVNET_SET(arg) \
    do { saved_vnet = curvnet; curvnet = (arg); } while (0)
#define CURVNET_RESTORE() \
    do { curvnet = saved_vnet; saved_vnet = NULL; } while (0)

struct vnet {

// sys/netinet/cc/cc.c
    struct cc_algo *default_cc_ptr;

// sys/netinet/cc/cc_cdg.c
    uma_zone_t qdiffsample_zone;
    int cdg_wif;
    int cdg_wdf;
    int cdg_loss_wdf;
    int cdg_smoothing_factor;
    int cdg_exp_backoff_scale;
    int cdg_consec_cong;
    int cdg_hold_backoff;

// sys/netinet/cc/cc_chd.c
    int chd_qmin;
    int chd_pmax;
    int chd_loss_fair;
    int chd_use_max;
    int chd_qthresh;

// sys/netinet/cc/cc_hd.c
    int hd_qthresh;
    int hd_qmin;
    int hd_pmax;

// sys/netinet/cc/cc_htcp.c
    int htcp_adaptive_backoff;
    int htcp_rtt_scaling;

// sys/netinet/cc/cc_vegas.c
    int vegas_alpha;
    int vegas_beta;

// sys/netinet/khelp/h_ertt.c
    uma_zone_t txseginfo_zone;

// sys/netinet/tcp_hostcache.c
#define TCP_HOSTCACHE_HASHSIZE 1
#define TCP_HOSTCACHE_BUCKETLIMIT 30
    struct tcp_hostcache tcp_hostcache;
    struct callout tcp_hc_callout;

// sys/netinet/tcp_input.c
    struct tcpstat *tcpstat;
    int tcp_log_in_vain;
    int blackhole;
    int tcp_delack_enabled;
    int drop_synfin;
    int tcp_do_rfc3042;
    int tcp_do_rfc3390;
    int tcp_do_rfc3465;
    int tcp_abc_l_var;
    int tcp_do_ecn;
    int tcp_ecn_maxretries;
    int tcp_insecure_rst;
    int tcp_do_autorcvbuf;
    int tcp_autorcvbuf_inc;
    int tcp_autorcvbuf_max;
    struct inpcbhead tcb;
    struct inpcbinfo tcbinfo;

// sys/netinet/tcp_output.c
    int path_mtu_discovery;
    int ss_fltsz;
    int ss_fltsz_local;
    int tcp_do_autosndbuf;
    int tcp_autosndbuf_inc;
    int tcp_autosndbuf_max;

// sys/netinet/tcp_timer.c
    int tcp_keepinit;
    int tcp_keepidle;
    int tcp_keepintvl;
    int tcp_delacktime;
    int tcp_msl;
    int tcp_rexmit_min;
    int tcp_rexmit_slop;
    int always_keepalive;
    int tcp_fast_finwait2_recycle;
    int tcp_finwait2_timeout;
    int tcp_keepcnt;
    int tcp_maxpersistidle;
    int tcp_maxidle;

// sys/netinet/tcp_reass.c
    int tcp_reass_maxseg;
    uma_zone_t tcp_reass_zone;

// sys/netinet/tcp_sack.c
    int tcp_do_sack;
    int tcp_sack_maxholes;
    int tcp_sack_globalmaxholes;
    int tcp_sack_globalholes;

// sys/netinet/tcp_timewait.c
    uma_zone_t tcptw_zone;
    int maxtcptw;
    TAILQ_HEAD(, tcptw) twq_2msl;

// sys/netinet/tcp_subr.c
    int tcp_mssdflt;
    int tcp_v6mssdflt;
    int tcp_minmss;
    int tcp_do_rfc1323;
    int tcp_log_debug;
    int icmp_may_rst;
    int tcp_isn_reseed_interval;
    uma_zone_t sack_hole_zone;
    uma_zone_t tcpcb_zone;
    u_char isn_secret[32];
    int isn_last;
    int isn_last_reseed;
    u_int32_t isn_offset;
    u_int32_t isn_offset_old;

// sys/netinet/tcp_syncache.c
#define TCP_SYNCACHE_HASHSIZE 1
#define TCP_SYNCACHE_BUCKETLIMIT 30
    int tcp_syncookies;
    int tcp_syncookiesonly;
    struct tcp_syncache tcp_syncache;
    int tcp_sc_rst_sock_fail;

// sys/netinet/tcp_usrreq.c
    int tcp_sendspace;
    int tcp_recvspace;

// sys/kern/uipc_socket.c
    int maxsockets;

// sys/kern/uipc_sockbuf.c
    int sb_max;

// newly added for porting
    struct callout tcp_slowtimo_callout;
    int tcp_slowtimo_timeout;
    int tcp_nagle;
    int tcp_options;
    int debug_iss_zero;
    int debug_packet;
    int debug_packet_option;
    int debug_packet_window;
    int debug_packet_file;
    int debug_reass;
    int debug_reass_file;
    int debug_sockbuf;
    int debug_sockbuf_file;
    int debug_tcpstat;
    ScenSim::TcpProtocolImplementation *ctx;

    vnet()
        :
        default_cc_ptr(&newreno_cc_algo),
        qdiffsample_zone(),                      //cdg_mod_init()
        cdg_wif(0),
        cdg_wdf(50),
        cdg_loss_wdf(50),
        cdg_smoothing_factor(8),
        cdg_exp_backoff_scale(3),
        cdg_consec_cong(5),
        cdg_hold_backoff(5),
        chd_qmin(5),
        chd_pmax(50),
        chd_loss_fair(1),
        chd_use_max(1),
        chd_qthresh(20),
        hd_qthresh(20),
        hd_qmin(5),
        hd_pmax(5),
        htcp_adaptive_backoff(0),
        htcp_rtt_scaling(0),
        vegas_alpha(1),
        vegas_beta(3),
        txseginfo_zone(),                        //ertt_mod_init()
        tcp_hostcache(),                         //tcp_hc_init()
        tcp_hc_callout(),                        //tcp_hc_init()
        tcpstat(NULL),
        tcp_log_in_vain(0),
        blackhole(0),
        tcp_delack_enabled(1),
        drop_synfin(0),
        tcp_do_rfc3042(1),
        tcp_do_rfc3390(1),
        tcp_do_rfc3465(1),
        tcp_abc_l_var(2),
        tcp_do_ecn(0),
        tcp_ecn_maxretries(1),
        tcp_insecure_rst(0),
        tcp_do_autorcvbuf(1),
        tcp_autorcvbuf_inc(16*1024),
        tcp_autorcvbuf_max(2*1024*1024),
        tcb(),                                   //tcp_init()
        tcbinfo(),                               //tcp_init()
        path_mtu_discovery(0),
        ss_fltsz(1),
        ss_fltsz_local(4),
        tcp_do_autosndbuf(1),
        tcp_autosndbuf_inc(8*1024),
        tcp_autosndbuf_max(2*1024*1024),
        tcp_keepinit(TCPTV_KEEP_INIT),
        tcp_keepidle(TCPTV_KEEP_IDLE),
        tcp_keepintvl(TCPTV_KEEPINTVL),
        tcp_delacktime(TCPTV_DELACK),
        tcp_msl(TCPTV_MSL),
        tcp_rexmit_min(TCPTV_MIN),
        tcp_rexmit_slop(TCPTV_CPU_VAR),
        always_keepalive(1),
        tcp_fast_finwait2_recycle(0),
        tcp_finwait2_timeout(TCPTV_FINWAIT2_TIMEOUT),
        tcp_keepcnt(TCPTV_KEEPCNT),
        tcp_maxpersistidle(TCPTV_KEEP_IDLE),
        tcp_maxidle(),                           //tcp_slowtimo()
        tcp_reass_maxseg(128),
        tcp_reass_zone(),                        //tcp_reass_init()
        tcp_do_sack(1),
        tcp_sack_maxholes(128),
        tcp_sack_globalmaxholes(65536),
        tcp_sack_globalholes(0),
        tcptw_zone(),                            //tcp_tw_init()
        maxtcptw(0),
        twq_2msl(),                              //tcp_tw_init()
        tcp_mssdflt(TCP_MSS),
        tcp_v6mssdflt(TCP6_MSS),
        tcp_minmss(TCP_MINMSS),
        tcp_do_rfc1323(1),
        tcp_log_debug(0),
        icmp_may_rst(0),
        tcp_isn_reseed_interval(0),
        sack_hole_zone(),                        //tcp_init()
        tcpcb_zone(),                            //tcp_init()
        isn_secret(),                            //tcp_new_isn()
        isn_last(),                              //tcp_new_isn()
        isn_last_reseed(),                       //tcp_new_isn()
        isn_offset(),                            //tcp_new_isn()
        isn_offset_old(),                        //tcp_new_isn()
        tcp_syncookies(1),
        tcp_syncookiesonly(0),
        tcp_syncache(),                          //syncache_init()
        tcp_sc_rst_sock_fail(1),
        tcp_sendspace(1024*32),
        tcp_recvspace(1024*64),
        maxsockets(65535),
        sb_max(2*1024*1024),
        tcp_slowtimo_callout(),                  //tcp_init()
        tcp_slowtimo_timeout(hz/2),
        tcp_nagle(1),
        tcp_options(1),
        debug_iss_zero(0),
        debug_packet(0),
        debug_packet_option(0),
        debug_packet_window(0),
        debug_packet_file(0),
        debug_reass(0),
        debug_reass_file(0),
        debug_sockbuf(0),
        debug_sockbuf_file(0),
        debug_tcpstat(0),
        ctx()                                    //TcpProtocolImplementation::InitializeOnDemand()
    {
        tcp_hostcache.hashsize = TCP_HOSTCACHE_HASHSIZE;
        tcp_hostcache.bucket_limit = TCP_HOSTCACHE_BUCKETLIMIT;
        tcp_syncache.hashsize = TCP_SYNCACHE_HASHSIZE;
        tcp_syncache.bucket_limit = TCP_SYNCACHE_BUCKETLIMIT;

    }//vnet//

};//vnet//

//------------------------------------------------------------------------------
// replacement macros for global variable declarations
//------------------------------------------------------------------------------

#define SCENSIMGLOBAL(n)          (curvnet->n)

// sys/netinet/cc.h
#define V_default_cc_ptr          SCENSIMGLOBAL(default_cc_ptr)

// sys/netinet/cc/cc_cdg.c
#define V_qdiffsample_zone        SCENSIMGLOBAL(qdiffsample_zone)
#define V_cdg_wif                 SCENSIMGLOBAL(cdg_wif)
#define V_cdg_wdf                 SCENSIMGLOBAL(cdg_wdf)
#define V_cdg_loss_wdf            SCENSIMGLOBAL(cdg_loss_wdf)
#define V_cdg_smoothing_factor    SCENSIMGLOBAL(cdg_smoothing_factor)
#define V_cdg_exp_backoff_scale   SCENSIMGLOBAL(cdg_exp_backoff_scale)
#define V_cdg_consec_cong         SCENSIMGLOBAL(cdg_consec_cong)
#define V_cdg_hold_backoff        SCENSIMGLOBAL(cdg_hold_backoff)

// sys/netinet/cc/cc_chd.c
#define V_chd_qthresh             SCENSIMGLOBAL(chd_qthresh)
#define V_chd_qmin                SCENSIMGLOBAL(chd_qmin)
#define V_chd_pmax                SCENSIMGLOBAL(chd_pmax)
#define V_chd_loss_fair           SCENSIMGLOBAL(chd_loss_fair)
#define V_chd_use_max             SCENSIMGLOBAL(chd_use_max)

// sys/netinet/cc/cc_hd.c
#define V_hd_qthresh              SCENSIMGLOBAL(hd_qthresh)
#define V_hd_qmin                 SCENSIMGLOBAL(hd_qmin)
#define V_hd_pmax                 SCENSIMGLOBAL(hd_pmax)

// sys/netinet/cc/cc_htcp.c
#define V_htcp_adaptive_backoff   SCENSIMGLOBAL(htcp_adaptive_backoff)
#define V_htcp_rtt_scaling        SCENSIMGLOBAL(htcp_rtt_scaling)

// sys/netinet/cc/cc_vegas.c
#define V_vegas_alpha             SCENSIMGLOBAL(vegas_alpha)
#define V_vegas_beta              SCENSIMGLOBAL(vegas_beta)

// sys/netinet/tcp_hostcache.c
#define V_tcp_hostcache           SCENSIMGLOBAL(tcp_hostcache)
#define V_tcp_hc_callout          SCENSIMGLOBAL(tcp_hc_callout)

// sys/netinet/tcp_input.c
#define V_blackhole               SCENSIMGLOBAL(blackhole)
#define V_drop_synfin             SCENSIMGLOBAL(drop_synfin)
#define V_tcp_do_rfc3042          SCENSIMGLOBAL(tcp_do_rfc3042)
#define V_tcp_insecure_rst        SCENSIMGLOBAL(tcp_insecure_rst)
#define V_tcp_do_autorcvbuf       SCENSIMGLOBAL(tcp_do_autorcvbuf)
#define V_tcp_autorcvbuf_inc      SCENSIMGLOBAL(tcp_autorcvbuf_inc)
#define V_tcp_autorcvbuf_max      SCENSIMGLOBAL(tcp_autorcvbuf_max)

// sys/netinet/tcp_output.c
#define V_tcp_do_autosndbuf       SCENSIMGLOBAL(tcp_do_autosndbuf)
#define V_tcp_autosndbuf_inc      SCENSIMGLOBAL(tcp_autosndbuf_inc)
#define V_tcp_autosndbuf_max      SCENSIMGLOBAL(tcp_autosndbuf_max)

// sys/netinet/tcp_reass.c
#define V_tcp_reass_maxseg        SCENSIMGLOBAL(tcp_reass_maxseg)
#define V_tcp_reass_zone          SCENSIMGLOBAL(tcp_reass_zone)

// sys/netinet/tcp_subr.c
#define V_icmp_may_rst            SCENSIMGLOBAL(icmp_may_rst)
#define V_tcp_isn_reseed_interval SCENSIMGLOBAL(tcp_isn_reseed_interval)
#define V_tcpcb_zone              SCENSIMGLOBAL(tcpcb_zone)
#define V_isn_secret              SCENSIMGLOBAL(isn_secret)
#define V_isn_last                SCENSIMGLOBAL(isn_last)
#define V_isn_last_reseed         SCENSIMGLOBAL(isn_last_reseed)
#define V_isn_offset              SCENSIMGLOBAL(isn_offset)
#define V_isn_offset_old          SCENSIMGLOBAL(isn_offset_old)

// sys/netinet/tcp_sack.c
#define V_sack_hole_zone          SCENSIMGLOBAL(sack_hole_zone)
#define V_tcp_do_sack             SCENSIMGLOBAL(tcp_do_sack)
#define V_tcp_sack_maxholes       SCENSIMGLOBAL(tcp_sack_maxholes)
#define V_tcp_sack_globalmaxholes SCENSIMGLOBAL(tcp_sack_globalmaxholes)
#define V_tcp_sack_globalholes    SCENSIMGLOBAL(tcp_sack_globalholes)

// sys/netinet/tcp_syncache.c
#define V_tcp_syncookies          SCENSIMGLOBAL(tcp_syncookies)
#define V_tcp_syncookiesonly      SCENSIMGLOBAL(tcp_syncookiesonly)
#define V_tcp_syncache            SCENSIMGLOBAL(tcp_syncache)

// sys/netinet/tcp_timer.c
#define always_keepalive          SCENSIMGLOBAL(always_keepalive)
#define tcp_fast_finwait2_recycle SCENSIMGLOBAL(tcp_fast_finwait2_recycle)
#define tcp_finwait2_timeout      SCENSIMGLOBAL(tcp_finwait2_timeout)
#define tcp_keepcnt               SCENSIMGLOBAL(tcp_keepcnt)
#define tcp_maxpersistidle        SCENSIMGLOBAL(tcp_maxpersistidle)

// sys/netinet/tcp_timer.h
#define tcp_keepinit              SCENSIMGLOBAL(tcp_keepinit)
#define tcp_keepidle              SCENSIMGLOBAL(tcp_keepidle)
#define tcp_keepintvl             SCENSIMGLOBAL(tcp_keepintvl)
#define tcp_maxidle               SCENSIMGLOBAL(tcp_maxidle)
#define tcp_delacktime            SCENSIMGLOBAL(tcp_delacktime)
#define tcp_rexmit_min            SCENSIMGLOBAL(tcp_rexmit_min)
#define tcp_rexmit_slop           SCENSIMGLOBAL(tcp_rexmit_slop)
#define tcp_msl                   SCENSIMGLOBAL(tcp_msl)

// sys/netinet/tcp_timewait.c
#define V_tcptw_zone              SCENSIMGLOBAL(tcptw_zone)
#define maxtcptw                  SCENSIMGLOBAL(maxtcptw)
#define V_twq_2msl                SCENSIMGLOBAL(twq_2msl)

// sys/netinet/tcp_var.h
#define V_tcp_do_rfc1323          SCENSIMGLOBAL(tcp_do_rfc1323)
#define tcp_log_in_vain           SCENSIMGLOBAL(tcp_log_in_vain)
#define V_tcb                     SCENSIMGLOBAL(tcb)
#define V_tcbinfo                 SCENSIMGLOBAL(tcbinfo)
#define V_tcpstat                 SCENSIMGLOBAL(tcpstat)
#define V_tcp_mssdflt             SCENSIMGLOBAL(tcp_mssdflt)
#define V_tcp_minmss              SCENSIMGLOBAL(tcp_minmss)
#define V_tcp_delack_enabled      SCENSIMGLOBAL(tcp_delack_enabled)
#define V_tcp_do_rfc3390          SCENSIMGLOBAL(tcp_do_rfc3390)
#define V_path_mtu_discovery      SCENSIMGLOBAL(path_mtu_discovery)
#define V_ss_fltsz                SCENSIMGLOBAL(ss_fltsz)
#define V_ss_fltsz_local          SCENSIMGLOBAL(ss_fltsz_local)
#define V_tcp_do_rfc3465          SCENSIMGLOBAL(tcp_do_rfc3465)
#define V_tcp_abc_l_var           SCENSIMGLOBAL(tcp_abc_l_var)
#define V_tcp_sc_rst_sock_fail    SCENSIMGLOBAL(tcp_sc_rst_sock_fail)
#define V_tcp_do_ecn              SCENSIMGLOBAL(tcp_do_ecn)
#define V_tcp_ecn_maxretries      SCENSIMGLOBAL(tcp_ecn_maxretries)
#define tcp_sendspace             SCENSIMGLOBAL(tcp_sendspace)
#define tcp_recvspace             SCENSIMGLOBAL(tcp_recvspace)

// sys/netinet6/tcp6_var.h
#define V_tcp_v6mssdflt           SCENSIMGLOBAL(tcp_v6mssdflt)

// sys/netinet/in_pcb.h
#define V_ipport_firstauto        IPPORT_EPHEMERALFIRST
#define V_ipport_lastauto         IPPORT_EPHEMERALLAST

// sys/netinet/ip_var.h
#define V_ip_defttl               IPDEFTTL

// sys/sys/socketvar.h
#define maxsockets                SCENSIMGLOBAL(maxsockets)
#define sb_max                    SCENSIMGLOBAL(sb_max)

// newly added for porting
#define tcp_slowtimo_callout      SCENSIMGLOBAL(tcp_slowtimo_callout)
#define tcp_slowtimo_timeout      SCENSIMGLOBAL(tcp_slowtimo_timeout)
#define tcp_nagle                 SCENSIMGLOBAL(tcp_nagle)
#define tcp_options               SCENSIMGLOBAL(tcp_options)
#define debug_iss_zero            SCENSIMGLOBAL(debug_iss_zero)
#define debug_packet              SCENSIMGLOBAL(debug_packet)
#define debug_packet_option       SCENSIMGLOBAL(debug_packet_option)
#define debug_packet_window       SCENSIMGLOBAL(debug_packet_window)
#define debug_packet_file         SCENSIMGLOBAL(debug_packet_file)
#define debug_reass               SCENSIMGLOBAL(debug_reass)
#define debug_reass_file          SCENSIMGLOBAL(debug_reass_file)
#define debug_sockbuf             SCENSIMGLOBAL(debug_sockbuf)
#define debug_sockbuf_file        SCENSIMGLOBAL(debug_sockbuf_file)
#define debug_tcpstat             SCENSIMGLOBAL(debug_tcpstat)


// Glue functions TcpProtocolImplementation and Scenargie types do not need to be defined
// inside FreeBsd Code.


int32_t GenerateRandomInt(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr);

void RegisterCallout(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::callout *c);

void ScheduleCallout(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::callout *c);

void CreatePacket(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so,
    unsigned int totalPayloadLength,
    ScenSim::Packet*& packetPtr,
    unsigned char*& rawPacketDataPtr);

void AddVirtualFragmentData(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    ScenSim::Packet& packet,
    const unsigned int offset,
    const unsigned int length,
    const char* rawFragmentDataPtr);

void GetVirtualFragmentData(
    const ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const ScenSim::Packet& packet,
    const unsigned int dataBegin,
    const unsigned int dataEnd,
    unsigned int& length,
    const unsigned char*& rawFragmentDataPtr,
    unsigned int& nextOffset);

void SendToNetworkLayer(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const struct FreeBsd9Port::tcpcb *tp,
    const struct FreeBsd9Port::mbuf *m,
    ScenSim::Packet*& packetPtr);

void HandleNewConnection(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *lso,
    struct FreeBsd9Port::socket **so);

void NotifyIsConnected(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so);

void NotifyBufferAvailable(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so);

void NotifyDataArrival(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so);

void NotifyIsFinReceived(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so);

void NotifyIsSocketDisconnected(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so);

void NotifyIsSocketClosing(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    struct FreeBsd9Port::socket *so);

void UpdateRttForStatistics(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    int rtt);

void UpdateCwndForStatistics(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    u_long cwnd);

void CountRetransmissionForStatistics(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr);

void OutputDebugLogForPacket(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const struct FreeBsd9Port::tcpcb *tp,
    const struct FreeBsd9Port::mbuf *m,
    const ScenSim::Packet& aPacket,
    const bool isSent);

void OutputDebugLogForReassembleQueue(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const struct FreeBsd9Port::tcpcb *tp);

void OutputDebugLogForAppendSockBuf(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const struct FreeBsd9Port::sockbuf *sb,
    const struct FreeBsd9Port::mbuf *m);

void OutputDebugLogForDropSockBuf(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const struct FreeBsd9Port::sockbuf *sb,
    int len);

void OutputDebugLogForStat(
    ScenSim::TcpProtocolImplementation* tcpProtocolPtr,
    const struct FreeBsd9Port::tcpstat *stat);



}//namespace//


#endif
