// Note: Tcp_Porting.cpp DOES NOT include Scenargie (ScenSim) headers and ScenSim types are only
//       from forwarding definitions.  "Bridge Code", i.e. code that has both Scenargie and
//       FreeBSD headers included is in the file: bsd9tcpglue.cpp.


#include <stdarg.h>
#include "tcp_porting.h"
#include <iostream>

namespace FreeBsd9Port {

using std::cerr;
using std::endl;

//------------------------------------------------------------------------------
// global variables
//------------------------------------------------------------------------------

const int hz = 100;

int ticks;
time_t time_uptime;

u_char inetctlerrmap[PRC_NCMDS] = {
    0,            0,            0,            0,
    0,            EMSGSIZE,     EHOSTDOWN,    EHOSTUNREACH,
    EHOSTUNREACH, EHOSTUNREACH, ECONNREFUSED, ECONNREFUSED,
    EMSGSIZE,     EHOSTUNREACH, 0,            0,
    0,            0,            EHOSTUNREACH, 0,
    ENOPROTOOPT,  ECONNREFUSED
};

#ifdef INET6
const struct in6_addr in6addr_any = IN6ADDR_ANY_INIT;

const struct sockaddr_in6 sa6_any =
    { sizeof(sa6_any), AF_INET6, 0, 0, IN6ADDR_ANY_INIT, 0 };

u_char inet6ctlerrmap[PRC_NCMDS] = {
    0,            0,            0,            0,
    0,            EMSGSIZE,     EHOSTDOWN,    EHOSTUNREACH,
    EHOSTUNREACH, EHOSTUNREACH, ECONNREFUSED, ECONNREFUSED,
    EMSGSIZE,     EHOSTUNREACH, 0,            0,
    0,            0,            0,            0,
    ENOPROTOOPT
};
#endif

struct vnet *curvnet;
struct vnet *saved_vnet;

//------------------------------------------------------------------------------
// memory allocation
//------------------------------------------------------------------------------

void *malloc(unsigned long size, struct malloc_type *type, int flags)
{
    void *p;

    p = ::malloc(size);
    if (p == NULL) {
        return NULL;
    }

    bzero(p, size);

    return p;
}

void free(void *addr, struct malloc_type *type)
{
    ::free(addr);
}

//------------------------------------------------------------------------------
// uma
//------------------------------------------------------------------------------

uma_zone_t uma_zcreate_porting(size_t size)
{
    uma_zone_t zone;

    zone = (uma_zone_t)malloc(sizeof(uma_zone), NULL, 0);
    if (zone) {
        zone->size = size;
        zone->count = 0;
        zone->limit = INT_MAX;
    }

    return zone;
}

void uma_zdestroy(uma_zone_t zone)
{
    free(zone, NULL);
}

void *uma_zalloc_porting(uma_zone_t zone)
{
    if (zone->count >= zone->limit) {
        return NULL;
    }
    else {
        void *p = malloc(zone->size, NULL, 0);
        if (p) {
            ++(zone->count);
        }
        return p;
    }
}

void uma_zfree(uma_zone_t zone, void *item)
{
    --(zone->count);
    free(item, NULL);
}

//------------------------------------------------------------------------------
// mbuf
//------------------------------------------------------------------------------

void m_adj(struct mbuf *mp, int req_len)
{
    if (req_len >= 0) {
        mp->m_data += req_len;
        mp->m_len -= req_len;
        mp->m_pkthdr.len -= req_len;
        assert(mp->m_data >= mp->m_databuf);
        assert(mp->m_data <= mp->m_databuf + mp->m_vdatalen);
        assert(mp->m_len >= 0);
        assert(mp->m_pkthdr.len >= 0);
    }
    else {
        mp->m_len += req_len;
        mp->m_pkthdr.len += req_len;
        assert(mp->m_len >= 0);
        assert(mp->m_pkthdr.len >= 0);
    }
}

struct mbuf *m_pullup(struct mbuf *, int)
{
    assert(false);
    return NULL;
}

void m_copydata(const struct mbuf *, int, int, caddr_t)
{
    assert(false);
}

struct mbuf *m_copym_porting(struct mbuf *, int, int)
{
    assert(false);
    return NULL;
}

struct mbuf *m_get(mbuf_porting *data, short type)
{
    struct mbuf *m;

    m = (struct mbuf *)malloc(sizeof(struct mbuf), NULL, 0);
    if (m == NULL) {
        return NULL;
    }

    m->m_hdr.mh_flags = type;

    m->m_extptr = true;
    m->m_datasource = data;
    m->m_databuf = data->databuf();
    m->m_datalen = data->datalen();
    m->m_vdatalen = data->vdatalen();
    assert(m->m_databuf || (!m->m_databuf && m->m_datalen == 0));

    m->m_data = m->m_databuf;

    return m;
}

struct mbuf *m_gethdr_porting(short type)
{
    struct mbuf *m;

    m = (struct mbuf *)malloc(sizeof(struct mbuf), NULL, 0);
    if (m == NULL) {
        return NULL;
    }

    m->m_hdr.mh_flags = type;

    m->m_extptr = false;
    m->m_databuf = (caddr_t)malloc(MHLEN, NULL, 0);
    if (m->m_databuf == NULL) {
        free(m, NULL);
        return NULL;
    }
    m->m_datalen = MHLEN;
    m->m_vdatalen = m->m_datalen;

    m->m_data = m->m_databuf;

    return m;
}

struct mbuf *m_free(struct mbuf *m)
{
    mbuf *n;

    n = m->m_next;
    if (m->m_extptr) {
        delete m->m_datasource;
    }
    else {
        free(m->m_databuf, NULL);
    }
    free(m, NULL);

    return n;
}

void m_freem(struct mbuf *m)
{
    while (m != NULL) {
        m = m_free(m);
    }
}

//------------------------------------------------------------------------------
// checksum
//------------------------------------------------------------------------------

u_short in_pseudo(u_int32_t a, u_int32_t b, u_int32_t c)
{
//TBD: checksum is not supported
    u_int64_t sum = a + b + c;
    sum  = (sum & 0xffff) + (sum >> 16);
    sum  = (sum & 0xffff) + (sum >> 16);
    return sum;
}

u_short in_cksum_skip(struct mbuf *m, int len, int skip)
{
//TBD: checksum is not supported
    return 0;
}

#ifdef INET6
int in6_cksum(struct mbuf *m, u_int8_t nxt, u_int32_t off, u_int32_t len)
{
//TBD: checksum is not supported
    return 0;
}
#endif

//------------------------------------------------------------------------------
// sockbuf
//------------------------------------------------------------------------------

void assert_sb_cc(struct sockbuf *sb)
{
//    std::ofstream ofs;
//    ofs.open("sb.bin", std::ios::binary|std::ios::trunc);

    struct mbuf *m = sb->sb_mb;
    unsigned int len = 0;

    while (m) {
//        ofs.write((const char *)m->m_data, m->m_len);
        len += m->m_len;
        m = m->m_next;
    }

    assert(len == sb->sb_cc);
}

void sbdrop_locked(struct sockbuf *sb, int len)
{
    if (debug_sockbuf) {
        assert(curvnet->ctx != 0);
        OutputDebugLogForDropSockBuf(curvnet->ctx, sb, len);
    }//if//

    assert(sb->sb_cc >= len);
    sb->sb_cc -= len;

    struct mbuf *m = sb->sb_mb;

    while (m && len > 0) {
        if (m->m_len > len) {
            m->m_data += len;
            m->m_len -= len;
            m->m_pkthdr.len -= len;
            assert(m->m_data >= m->m_databuf);
            assert(m->m_data <= m->m_databuf + m->m_vdatalen);
            assert(m->m_len >= 0);
            assert(m->m_pkthdr.len >= 0);
            len -= len;
        }
        else {
            len -= m->m_len;
            m = m_free(m);
        }
    }
    assert(len == 0);

    sb->sb_mb = m;
    if (sb->sb_mb == NULL) {
        sb->sb_mbtail = NULL;
    }

    DEBUG_ASSERT_SB_CC(sb);
}

int sbreserve_locked_porting(struct sockbuf *sb, u_long cc)
{
    sb->sb_hiwat = ulmin(cc, sb_max);

    return 0;
}

void sbappendstream_locked(struct sockbuf *sb, struct mbuf *m)
{
    if (debug_sockbuf) {
        assert(curvnet->ctx !=0);
        OutputDebugLogForAppendSockBuf(curvnet->ctx, sb, m);
    }//if//

    if (m->m_len == 0) {
        m_freem(m);
        return;
    }

    if (sb->sb_mb == NULL) {
        assert(sb->sb_mbtail == NULL);
        sb->sb_mb = m;
    }
    else {
        assert(sb->sb_mbtail);
        sb->sb_mbtail->m_next = m;
    }
    sb->sb_mbtail = m;

    sb->sb_cc += m->m_len;
    assert(sb->sb_cc <= sb->sb_hiwat);

    DEBUG_ASSERT_SB_CC(sb);
}

//------------------------------------------------------------------------------
// socket
//------------------------------------------------------------------------------

static struct socket *soalloc(struct vnet *vnet)
{
    struct socket *so;

    so = (struct socket *)malloc(sizeof(struct socket), NULL, 0);
    if (so == NULL) {
        return NULL;
    }

    TAILQ_INIT(&so->so_incomp);
    TAILQ_INIT(&so->so_comp);

    so->so_vnet = vnet;
    assert(so->so_vnet);

    return so;
}

static void sodealloc(struct socket *so)
{
    assert(TAILQ_EMPTY(&so->so_incomp));
    assert(TAILQ_EMPTY(&so->so_comp));

    free(so, NULL);
}

int socreate(int dom, struct socket **aso)
{
    struct socket *so;
    int error;

    so = soalloc(curvnet);
    if (so == NULL) {
        return ENOBUFS;
    }

    switch (dom) {
    case AF_INET:
        so->so_usrreqs = &tcp_usrreqs;
        break;
#ifdef INET6
    case AF_INET6:
        so->so_usrreqs = &tcp6_usrreqs;
        break;
#endif
    default:
        assert(false);
    }

    error = (*so->so_usrreqs->pru_attach)(so, 0, NULL);
    if (error) {
        sodealloc(so);
        return error;
    }

    struct inpcb *inp = sotoinpcb(so);
    assert(inp);

    struct tcpcb *tp = intotcpcb(inp);
    assert(tp);

    if (tcp_nagle) {
        tp->t_flags |= TF_NODELAY;
    }//if//

    if (!tcp_options) {
        tp->t_flags |= TF_NOOPT;
    }//if//

    *aso = so;

    return 0;
}

struct socket *sonewconn(struct socket *head, int connstatus)
{
    struct socket *so;

    if (head->so_qlen > 3 * head->so_qlimit / 2) {
        return NULL;
    }

    assert(curvnet->ctx);
    HandleNewConnection(curvnet->ctx, head, &so);

    so->so_head = head;
    so->so_options = head->so_options &~ SO_ACCEPTCONN;
    so->so_state = head->so_state | SS_NOFDREF;
    so->so_usrreqs = head->so_usrreqs;

    so->so_rcv.sb_flags |= head->so_rcv.sb_flags & SB_AUTOSIZE;
    so->so_snd.sb_flags |= head->so_snd.sb_flags & SB_AUTOSIZE;

    if (soreserve(so, head->so_snd.sb_hiwat, head->so_rcv.sb_hiwat) != 0) {
        sodealloc(so);
        return NULL;
    }

    while (head->so_incqlen > head->so_qlimit) {
        struct socket *sp;
        sp = TAILQ_FIRST(&head->so_incomp);
        TAILQ_REMOVE(&head->so_incomp, sp, so_list);
        head->so_incqlen--;
        sp->so_qstate &= ~SQ_INCOMP;
        sp->so_head = NULL;
        soabort(sp);
    }
    TAILQ_INSERT_TAIL(&head->so_incomp, so, so_list);
    so->so_qstate |= SQ_INCOMP;
    head->so_incqlen++;

    return so;
}

int sobind(struct socket *so, struct sockaddr *nam)
{
    int error;

    error = (*so->so_usrreqs->pru_bind)(so, nam, NULL);

    return error;
}

int solisten(struct socket *so, int backlog)
{
    int error;

    error = (*so->so_usrreqs->pru_listen)(so, backlog, NULL);

    return error;
}

int solisten_proto_check(struct socket *so)
{
    if (so->so_state &
        (SS_ISCONNECTED | SS_ISCONNECTING | SS_ISDISCONNECTING)) {

        return EINVAL;
    }

    return 0;
}

static const int somaxconn = 128;
void solisten_proto(struct socket *so, int backlog)
{
    if (backlog < 0 || backlog > somaxconn) {
        backlog = somaxconn;
    }
    so->so_qlimit = backlog;
    so->so_options |= SO_ACCEPTCONN;
}

void sofree(struct socket *so)
{
    struct socket *head;

    if ((so->so_state & SS_NOFDREF) == 0 ||
        (so->so_state & SS_PROTOREF) ||
        (so->so_qstate & SQ_COMP)) {

        return;
    }

    head = so->so_head;
    if (head != NULL) {
        assert(so->so_qstate & SQ_INCOMP);
        TAILQ_REMOVE(&head->so_incomp, so, so_list);
        head->so_incqlen--;
        so->so_qstate &= ~SQ_INCOMP;
        so->so_head = NULL;
    }

    assert((so->so_qstate & SQ_COMP) == 0);
    assert((so->so_qstate & SQ_INCOMP) == 0);
    assert(TAILQ_EMPTY(&so->so_comp));
    assert(TAILQ_EMPTY(&so->so_incomp));

    (*so->so_usrreqs->pru_detach)(so);

    sbdrop_locked(&so->so_snd, so->so_snd.sb_cc);
    assert(so->so_snd.sb_cc == 0);

    sbdrop_locked(&so->so_rcv, so->so_rcv.sb_cc);
    assert(so->so_rcv.sb_cc == 0);

    assert(curvnet->ctx);
    NotifyIsSocketClosing(curvnet->ctx, so);
    sodealloc(so);
}

int soclose(struct socket *so)
{
    int error = 0;

    assert((so->so_state & SS_NOFDREF) == 0);

    if (so->so_state & SS_ISCONNECTED) {
        if ((so->so_state & SS_ISDISCONNECTING) == 0) {
            error = sodisconnect(so);
            if (error) {
                if (error == ENOTCONN)
                    error = 0;
                goto drop;
            }
        }
    }

drop:
    (*so->so_usrreqs->pru_close)(so);

    if (so->so_options & SO_ACCEPTCONN) {
        struct socket *sp;
        while ((sp = TAILQ_FIRST(&so->so_incomp)) != NULL) {
            TAILQ_REMOVE(&so->so_incomp, sp, so_list);
            so->so_incqlen--;
            sp->so_qstate &= ~SQ_INCOMP;
            sp->so_head = NULL;
            soabort(sp);
        }
        while ((sp = TAILQ_FIRST(&so->so_comp)) != NULL) {
            TAILQ_REMOVE(&so->so_comp, sp, so_list);
            so->so_qlen--;
            sp->so_qstate &= ~SQ_COMP;
            sp->so_head = NULL;
            soabort(sp);
        }
    }

    assert((so->so_state & SS_NOFDREF) == 0);
    so->so_state |= SS_NOFDREF;
    sofree(so);

    return error;
}

void soabort(struct socket *so)
{
    assert((so->so_state & SS_PROTOREF) == 0);
    assert(so->so_state & SS_NOFDREF);
    assert((so->so_state & SQ_COMP) == 0);
    assert((so->so_state & SQ_INCOMP) == 0);

    (*so->so_usrreqs->pru_abort)(so);

    sofree(so);
}

int soaccept(struct socket *so)
{
    int error;

    assert(so->so_state & SS_NOFDREF);
    so->so_state &= ~SS_NOFDREF;

    error = (*so->so_usrreqs->pru_accept)(so, NULL);

    return error;
}

int soconnect(struct socket *so, struct sockaddr *nam)
{
    int error;

    if (so->so_options & SO_ACCEPTCONN) {
        return EOPNOTSUPP;
    }
    if (so->so_state & (SS_ISCONNECTED|SS_ISCONNECTING)) {
        error = EISCONN;
    }
    else {
        so->so_error = 0;
        error = (*so->so_usrreqs->pru_connect)(so, nam, NULL);
    }

    return error;
}

int sodisconnect(struct socket *so)
{
    int error;

    if ((so->so_state & SS_ISCONNECTED) == 0) {
        return ENOTCONN;
    }
    if (so->so_state & SS_ISDISCONNECTING) {
        return EALREADY;
    }

    error = (*so->so_usrreqs->pru_disconnect)(so);

    return error;
}

void soisconnecting(struct socket *so)
{
    so->so_state &= ~(SS_ISCONNECTED|SS_ISDISCONNECTING);
    so->so_state |= SS_ISCONNECTING;
}

void soisconnected(struct socket *so)
{
    struct socket *head;

    so->so_state &= ~(SS_ISCONNECTING|SS_ISDISCONNECTING);
    so->so_state |= SS_ISCONNECTED;

    head = so->so_head;
    if (head != NULL && (so->so_qstate & SQ_INCOMP)) {
        TAILQ_REMOVE(&head->so_incomp, so, so_list);
        head->so_incqlen--;
        so->so_qstate &= ~SQ_INCOMP;
        TAILQ_INSERT_TAIL(&head->so_comp, so, so_list);
        head->so_qlen++;
        so->so_qstate |= SQ_COMP;
    }

    assert(curvnet->ctx);
    NotifyIsConnected(curvnet->ctx, so);
}

void soisdisconnecting(struct socket *so)
{
    so->so_state &= ~SS_ISCONNECTING;
    so->so_state |= SS_ISDISCONNECTING;
    so->so_rcv.sb_state |= SBS_CANTRCVMORE;
    so->so_snd.sb_state |= SBS_CANTSENDMORE;
}

void soisdisconnected(struct socket *so)
{
    so->so_state &= ~(SS_ISCONNECTING|SS_ISCONNECTED|SS_ISDISCONNECTING);
    so->so_state |= SS_ISDISCONNECTED;
    so->so_rcv.sb_state |= SBS_CANTRCVMORE;
    so->so_snd.sb_state |= SBS_CANTSENDMORE;

    sbdrop_locked(&so->so_snd, so->so_snd.sb_cc);
    assert(so->so_snd.sb_cc == 0);

    assert(curvnet->ctx);
    NotifyIsSocketDisconnected(curvnet->ctx, so);
}

void socantrcvmore(struct socket *so)
{
    so->so_rcv.sb_state |= SBS_CANTRCVMORE;
}

void socantsendmore(struct socket *so)
{
    so->so_snd.sb_state |= SBS_CANTSENDMORE;
}

void sorwakeup_locked(struct socket *so)
{
//TBD: so->so_rcv.sb_lowat is not supported
    if (so->so_rcv.sb_cc > 0) {
        assert(curvnet->ctx);
        NotifyDataArrival(curvnet->ctx, so);
    }
}

void sowwakeup_locked(struct socket *so)
{
//TBD: so->so_snd.sb_lowat is not supported
    if (sbspace(&so->so_snd) > 0) {
        assert(curvnet->ctx);
        NotifyBufferAvailable(curvnet->ctx, so);
    }
}

int soreserve(struct socket *so, u_long sndcc, u_long rcvcc)
{
    sbreserve_locked(&so->so_snd, sndcc, NULL, NULL);
    sbreserve_locked(&so->so_rcv, rcvcc, NULL, NULL);

    return 0;
}

void sohasoutofband(struct socket *so)
{
    assert(false);
}

int sosend(struct socket *so, int flags, struct mbuf *m)
{
    if (so->so_snd.sb_state & SBS_CANTSENDMORE) {
        m_free(m);
        return 0;
    }

    int error;

    error = (*so->so_usrreqs->pru_send)(so, flags, m, NULL, NULL, NULL);

    return (error);
}

//------------------------------------------------------------------------------
// address
//------------------------------------------------------------------------------

int in_localaddr(struct in_addr)
{
//TBD: always not local
    return 0;
}

int in_broadcast_porting(struct in_addr in)
{
    if (in.s_addr == INADDR_BROADCAST ||
        in.s_addr == INADDR_ANY) {
        return 1;
    }
    else {
        return 0;
    }
}

char *inet_ntoa_r(struct in_addr ina, char *buf)
{
    assert(false);
    return NULL;
}

#ifdef INET6
void in6_sin6_2_sin(
    struct sockaddr_in *sin, struct sockaddr_in6 *sin6)
{
    assert(false);
}

int in6_localaddr(struct in6_addr *)
{
    assert(false);
    return 0;
}
#endif

//------------------------------------------------------------------------------
// inpcb
//------------------------------------------------------------------------------

void in_pcbinfo_init_porting(
    struct inpcbinfo *pcbinfo, struct inpcbhead *listhead)
{
    pcbinfo->ipi_vnet = curvnet;
    pcbinfo->ipi_listhead = listhead;
    LIST_INIT(pcbinfo->ipi_listhead);
}

void in_pcbinfo_destroy(struct inpcbinfo *pcbinfo)
{
    struct inpcb *inpb;

    inpb = LIST_FIRST(V_tcbinfo.ipi_listhead);
    while (inpb != NULL) {
        struct inpcb *inpb_next = LIST_NEXT(inpb, inp_list);
        struct socket *so = inpb->inp_socket;
        struct tcpcb *tp = (struct tcpcb *)inpb->inp_ppcb;

        if ((inpb->inp_flags & INP_TIMEWAIT) == 0) {
            if (tp) {
                tcp_discardcb(tp);
            }
            if (so) {
                sbdrop_locked(&so->so_snd, so->so_snd.sb_cc);
                assert(so->so_snd.sb_cc == 0);
                sbdrop_locked(&so->so_rcv, so->so_rcv.sb_cc);
                assert(so->so_rcv.sb_cc == 0);
                assert(curvnet->ctx == NULL);
                sodealloc(so);
            }
            in_pcbfree(inpb);
        }
        inpb = inpb_next;
    }
}

int in_pcbbind_porting(struct inpcb *inp, struct sockaddr *nam)
{
    struct sockaddr_in *sin;
    struct inpcb *inpb;

    if (inp->inp_lport != 0 || inp->inp_laddr.s_addr != INADDR_ANY) {
        return EINVAL;
    }

    sin = (struct sockaddr_in *)nam;
//TBD: address and port should be automatically set if (sin == NULL)
    assert(sin != NULL);
    assert(sin->sin_port != 0);

    LIST_FOREACH(inpb, V_tcbinfo.ipi_listhead, inp_list) {
        if (inpb->inp_lport == sin->sin_port &&
            inpb->inp_laddr.s_addr == sin->sin_addr.s_addr) {

            return EADDRINUSE;
        }
    }

    inp->inp_lport = sin->sin_port;
    inp->inp_laddr.s_addr = sin->sin_addr.s_addr;

    return 0;
}

int in_pcbconnect_setup_porting(
    struct inpcb *inp, struct sockaddr *nam, in_addr_t *laddrp, u_short *lportp,
    in_addr_t *faddrp, u_short *fportp, struct inpcb **oinpp)
{
    struct sockaddr_in *sin;
    struct inpcb *inpb;

    if (oinpp) {
        *oinpp = NULL;
    }

    sin = (struct sockaddr_in *)nam;
    assert(sin != NULL);
    assert(sin->sin_port != 0);
    assert(*lportp != 0);

    LIST_FOREACH(inpb, V_tcbinfo.ipi_listhead, inp_list) {
        if (inpb->inp_lport == *lportp &&
            inpb->inp_laddr.s_addr == *laddrp &&
            inpb->inp_fport == sin->sin_port &&
            inpb->inp_faddr.s_addr == sin->sin_addr.s_addr) {

            if (oinpp) {
                *oinpp = inp;
            }

            break;
        }
    }

    *fportp = sin->sin_port;
    *faddrp = sin->sin_addr.s_addr;

    return 0;
}

int in_pcbconnect_mbuf_porting(
    struct inpcb *inp, struct sockaddr *nam, struct mbuf *m)
{
    in_addr_t laddr, faddr;
    u_short lport, fport;
    int error;

    lport = inp->inp_lport;
    laddr = inp->inp_laddr.s_addr;

    error = in_pcbconnect_setup_porting(
        inp, nam, &laddr, &lport, &faddr, &fport, NULL);
    if (error) {
        return error;
    }

    inp->inp_laddr.s_addr = laddr;
    inp->inp_lport = lport;
    inp->inp_faddr.s_addr = faddr;
    inp->inp_fport = fport;

    return 0;
}

struct inpcb *in_pcblookup_porting(
    struct inpcbinfo *pcbinfo, struct in_addr faddr, u_int fport,
    struct in_addr laddr, u_int lport, int lookupflags)
{
    struct inpcb *inpb, *match = NULL;
    int matchwild = 3, wildcard;

    LIST_FOREACH(inpb, V_tcbinfo.ipi_listhead, inp_list) {
        if (inpb->inp_lport != lport) {
            continue;
        }

        wildcard = 0;

        if (inpb->inp_laddr.s_addr != INADDR_ANY) {
            if (laddr.s_addr == INADDR_ANY) {
                ++wildcard;
            }
            else if (inpb->inp_laddr.s_addr != laddr.s_addr) {
                continue;
            }
        }
        else {
            if (laddr.s_addr != INADDR_ANY) {
                ++wildcard;
            }
        }

        if (inpb->inp_faddr.s_addr != INADDR_ANY) {
            if (faddr.s_addr == INADDR_ANY) {
                ++wildcard;
            }
            else if (
                (inpb->inp_faddr.s_addr != faddr.s_addr) ||
                (inpb->inp_fport != fport)) {

                continue;
            }
        }
        else {
            if (faddr.s_addr != INADDR_ANY) {
                ++wildcard;
            }
        }

        if (wildcard && (lookupflags & INPLOOKUP_WILDCARD) == 0) {
            continue;
        }

        if (wildcard < matchwild) {
            match = inpb;
            matchwild = wildcard;

            if (matchwild == 0) {
                break;
            }
        }
    }

    return match;
}

void in_pcbdrop(struct inpcb *inp)
{
    inp->inp_flags |= INP_DROPPED;
}

void in_pcbfree(struct inpcb *inp)
{
    LIST_REMOVE(inp, inp_list);
    if (inp->inp_options)
        (void)m_free(inp->inp_options);
    inp->inp_vflag = 0;
    free(inp, NULL);
}

void in_pcbnotifyall(
    struct inpcbinfo *pcbinfo, struct in_addr faddr, int errnoVal,
    struct inpcb *(*notify)(struct inpcb *, int))
{
    assert(false);
}

void in_pcbdetach(struct inpcb *inp)
{
    inp->inp_socket->so_pcb = NULL;
    inp->inp_socket = NULL;
}

int in_pcballoc(struct socket *so, struct inpcbinfo *pcbinfo)
{
    struct inpcb *inp;
    int error;

    error = 0;
    inp = (struct inpcb *)malloc(sizeof(struct inpcb), NULL, 0);
    if (inp == NULL)
        return (ENOBUFS);
    inp->inp_pcbinfo = pcbinfo;
    inp->inp_socket = so;
#ifdef INET6
    if (so->so_usrreqs == &tcp6_usrreqs) {
        inp->inp_vflag |= INP_IPV6PROTO;
        inp->inp_flags |= IN6P_IPV6_V6ONLY;
        inp->inp_flags |= IN6P_AUTOFLOWLABEL;
    }
#endif
    LIST_INSERT_HEAD(pcbinfo->ipi_listhead, inp, inp_list);
    so->so_pcb = (caddr_t)inp;

    return (error);
}

#ifdef INET6
int in6_pcbconnect_mbuf_porting(
    struct inpcb *inp, struct sockaddr *nam, struct mbuf *m)
{
    struct sockaddr_in6 *sin6;
    struct inpcb *inpb;

    sin6 = (struct sockaddr_in6 *)nam;
    assert(sin6 != NULL);
    assert(sin6->sin6_port != 0);
    assert(inp != NULL);
    assert(inp->inp_lport != 0);

    LIST_FOREACH(inpb, V_tcbinfo.ipi_listhead, inp_list) {
        if (inpb->inp_lport == inp->inp_lport &&
            IN6_ARE_ADDR_EQUAL(&inpb->in6p_laddr, &inp->in6p_laddr) &&
            (inpb->inp_fport == sin6->sin6_port || inpb->inp_fport == 0) &&
            (IN6_ARE_ADDR_EQUAL(&inpb->in6p_faddr, &sin6->sin6_addr) ||
            IN6_ARE_ADDR_EQUAL(&inpb->in6p_faddr, &in6addr_any))) {

            inp->in6p_faddr = sin6->sin6_addr;
            inp->inp_fport = sin6->sin6_port;

            return 0;
        }
    }

    return ENOENT;
}

struct inpcb *in6_pcblookup_mbuf_porting(
    struct inpcbinfo *pcbinfo, struct in6_addr *faddr, u_int fport,
    struct in6_addr *laddr, u_int lport, int lookupflags)
{
    struct inpcb *inpb, *match = NULL;
    int matchwild = 3, wildcard;

    LIST_FOREACH(inpb, V_tcbinfo.ipi_listhead, inp_list) {
        if (inpb->inp_lport != lport) {
            continue;
        }

        wildcard = 0;

        if(!IN6_ARE_ADDR_EQUAL(&inpb->in6p_laddr, &in6addr_any)) {
            if(IN6_ARE_ADDR_EQUAL(laddr, &in6addr_any)) {
                ++wildcard;
            }
            else if(!IN6_ARE_ADDR_EQUAL(&inpb->in6p_laddr, laddr)) {
                continue;
            }
        }
        else {
            if(!IN6_ARE_ADDR_EQUAL(laddr, &in6addr_any)) {
                ++wildcard;
            }
        }

        if(!IN6_ARE_ADDR_EQUAL(&inpb->in6p_faddr, &in6addr_any)) {
            if(IN6_ARE_ADDR_EQUAL(faddr, &in6addr_any)) {
                ++wildcard;
            }
            else if (
                !IN6_ARE_ADDR_EQUAL(&inpb->in6p_faddr, faddr) ||
                (inpb->inp_fport != fport)) {

                continue;
            }
        }
        else {
            if(!IN6_ARE_ADDR_EQUAL(faddr, &in6addr_any)) {
                ++wildcard;
            }
        }

        if (wildcard && (lookupflags & INPLOOKUP_WILDCARD) == 0) {
            continue;
        }

        if (wildcard < matchwild) {
            match = inpb;
            matchwild = wildcard;

            if (matchwild == 0) {
                break;
            }
        }
    }

    return match;
}

void in6_pcbnotify(struct inpcbinfo *, struct sockaddr *,
      u_int, const struct sockaddr *, u_int, int, void *,
      struct inpcb *(*)(struct inpcb *, int))
{
    assert(false);
}

int in6_pcbbind_porting(struct inpcb *inp, struct sockaddr *nam)
{
    struct sockaddr_in6 *sin;
    struct inpcb *inpb;

    if (inp->inp_lport != 0 || !IN6_ARE_ADDR_EQUAL(&inp->in6p_laddr, &in6addr_any)) {
        return EINVAL;
    }

    sin = (struct sockaddr_in6 *)nam;
//TBD: address and port should be automatically set if (sin == NULL)
    assert(sin != NULL);
    assert(sin->sin6_port != 0);

    LIST_FOREACH(inpb, V_tcbinfo.ipi_listhead, inp_list) {
        if (inpb->inp_lport == sin->sin6_port &&
            IN6_ARE_ADDR_EQUAL(&inpb->in6p_laddr, &sin->sin6_addr)) {

            return EADDRINUSE;
        }
    }

    inp->inp_lport = sin->sin6_port;
    inp->in6p_laddr = sin->sin6_addr;

    return 0;
}

int in6_pcbladdr(struct inpcb *inp, struct sockaddr *nam, struct in6_addr *plocal_addr6)
{
    assert(plocal_addr6);
    *plocal_addr6 = inp->in6p_laddr;
    return 0;
}
#endif

//------------------------------------------------------------------------------
// random
//------------------------------------------------------------------------------

u_long random(void)
{
    u_long randval;

    assert(curvnet->ctx);
    randval = (u_long)GenerateRandomInt(curvnet->ctx);

    return randval;
}

int read_random(void *buf, int count)
{
    int32_t randval;
    int size, i;

    for (i = 0; i < count; i += (int)sizeof(int32_t)) {
        assert(curvnet->ctx);
        randval = GenerateRandomInt(curvnet->ctx);//TBD//
        size = min(count - i, sizeof(int32_t));
        bcopy(&randval, &((char *)buf)[i], (size_t)size);
    }

    return count;
}

uint32_t arc4random(void)
{
    uint32_t randval;

    read_random(&randval, sizeof(uint32_t));

    return randval;
}

//------------------------------------------------------------------------------
// callout
//------------------------------------------------------------------------------

void callout_init_porting(struct callout *c)
{
    bzero(c, sizeof(struct callout));
    assert(curvnet->ctx);
    RegisterCallout(curvnet->ctx, c);
}

int callout_reset(
    struct callout *c, int on_tick, void (*fn)(void *), void *arg)
{
    assert(on_tick > 0);

    c->c_time = on_tick;
    c->c_arg = arg;
    c->c_func = fn;

    assert(curvnet->ctx);
    ScheduleCallout(curvnet->ctx, c);

    return 0;
}

int callout_stop(struct callout *c)
{
    c->c_time = 0;
    c->c_arg = NULL;
    c->c_func = NULL;

    if (curvnet->ctx) {
        ScheduleCallout(curvnet->ctx, c);
    }

    return 0;
}

//------------------------------------------------------------------------------
// system functions
//------------------------------------------------------------------------------

static void log_internal(const char *fmt, va_list ap)
{
    int len;
    char *buf;

    len = vsnprintf(NULL, 0, fmt, ap);
    buf = (char *)malloc(len + 1, NULL, 0);
    assert(buf);
    vsnprintf(buf, len + 1, fmt, ap);
    std::cout << buf << std::endl;
    free(buf, NULL);
}

void log(int level, const char *format, ...)
{
    va_list ap;

    va_start(ap, format);
    log_internal(format, ap);
    va_end(ap);
}

void bcopy(const void *src, void *dst, size_t len)
{
    memcpy(dst, src, len);
}

void bzero(void *buf, size_t len)
{
    char *p = static_cast<char *>(buf);

    while (len-- != 0) {
        *p++ = 0;
    }
}

void panic(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    log_internal(fmt, ap);
    va_end(ap);

    assert(false);
    abort();
}

//------------------------------------------------------------------------------
// output
//------------------------------------------------------------------------------

int ip_output(
    struct mbuf *m, struct mbuf *opt, struct route *ro, int flags,
    struct ip_moptions *imo, struct inpcb *inp, int off, long len)
{
    assert(opt == NULL); //Not Used//
    assert(ro == NULL);  //Not Used//
    assert(flags == 0);  //Not Used//
    assert(imo == NULL); //Not Used//

    assert(off >= 0);
    assert(len >= 0);

    assert(curvnet->ctx);
    unsigned char* datap = 0;
    ScenSim::Packet* packetPtr = 0;
    const long orig_len = len;

    CreatePacket(curvnet->ctx, inp ? inp->inp_socket : NULL, len, packetPtr, datap);

    if (len > 0) {
        struct mbuf *mb = inp->inp_socket->so_snd.sb_mb;
        while (mb && off > 0) {
            if (mb->m_len > off) {
                break;
            }
            else {
                off -= mb->m_len;
            }
            mb = mb->m_next;
        }
        while (mb && len > 0) {
            long copylen;
            if (mb->m_len > off + len) {
                copylen = len;
            }
            else {
                copylen = mb->m_len - off;
            }
            const char* copybegin = mb->m_data + off;
            const char* copyend = mb->m_data + off + copylen;
            if (datap) {
                std::copy(copybegin, copyend, datap);
                datap += copylen;
            }
            else {
                const char* dataend = mb->m_databuf + mb->m_datalen;
                if ((copybegin < dataend) && (dataend <= copyend)) {
                    AddVirtualFragmentData(
                        curvnet->ctx, *packetPtr, orig_len - len, dataend - copybegin, copybegin);
                }
                else if (copyend < dataend) {
                    AddVirtualFragmentData(
                        curvnet->ctx, *packetPtr, orig_len - len, copylen, copybegin);
                }
            }
            len -= copylen;
            off = 0;
            mb = mb->m_next;
        }
        assert(len == 0);
    }

    assert(curvnet->ctx);
    SendToNetworkLayer(curvnet->ctx,
        (inp && !(inp->inp_flags & INP_TIMEWAIT)) ? (struct tcpcb *)inp->inp_ppcb : NULL, m, packetPtr);

    m_free(m);

    return 0;
}

//------------------------------------------------------------------------------
// ip
//------------------------------------------------------------------------------

struct mbuf *ip_srcroute(struct mbuf *)
{
//TBD: IP options are not supported
    return NULL;
}

void ip_stripoptions(struct mbuf *, struct mbuf *)
{
//TBD: never reached here because IP options is not supported
    assert(false);
}

#ifdef INET6
struct ip6_pktopts *ip6_copypktopts_porting(
    struct ip6_pktopts *src)
{
    assert(false);
    return NULL;
}

u_int32_t ip6_randomflowlabel(void)
{
    u_int32_t randval;
    randval = (u_int32_t)GenerateRandomInt(curvnet->ctx);
    return randval & 0xfffff;
}

int ip6_optlen(struct inpcb *)
{
    return 0;
}

char *ip6_sprintf(char *, const struct in6_addr *)
{
    assert(false);
    return NULL;
}

void in6_losing(struct inpcb *inp)
{
}

int in6_selecthlim(struct in6pcb *in6p, struct ifnet *ifp)
{
    return 64;
}

void nd6_nud_hint(struct rtentry *rt, struct in6_addr *dst6, int force)
{
}
#endif

//------------------------------------------------------------------------------
// icmp
//------------------------------------------------------------------------------

int badport_bandlim(int)
{
//TBD: ICMP is not supported
    return 0;
}

int ip_next_mtu(int, int)
{
//TBD: ICMP is not supported
    assert(false);
    return 0;
}

}//namespace//
