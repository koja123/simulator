#ifndef NU_CORE_IP_H_
#define NU_CORE_IP_H_

#include "config.h" //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_ip Core :: IP address and originator address
 * @{
 */

#define NU_IP4_LEN    sizeof(nu_in_addr_t)  ///< size of IPv4 address
#define NU_IP6_LEN    sizeof(nu_in6_addr_t) ///< size of IPv6 address

#ifdef ADDON_NUOLSRv2
//// IP address for qualnet
typedef uint32_t          nu_in_addr_t;   ///< IPv4 address
typedef in6_addr          nu_in6_addr_t;  ///< IPv6 address
#else
typedef struct in_addr    nu_in_addr_t;   ///< IPv4 address
typedef struct in6_addr   nu_in6_addr_t;  ///< IPv6 address
#endif

/**
 * IP address bit image.
 */
typedef union {
    uint8_t       u8[16]; ///< for uint8_t access
    int16_t       i16[8]; ///< for int16_t access
    uint32_t      u32[4]; ///< for uint32_t access
    nu_in_addr_t  v4;     ///< IPv4 address
    nu_in6_addr_t v6;     ///< IPv6 address
} nu_ip_addr_t;

/**
 * IP address with prefix length.
 *
 * IP address is declared as uint32_t (uint8_t x 4):
 *
 * - u[0] ... type (2:IPv4 / 30:IPv6 / 0xff:undefined)
 * - u[1] ... prefix (0xff:undefined / 0-128:prefix length)
 * - u[2] ... chunk number
 * - u[3] ... index in the chunk
 *
 * @see ip_heap.h, ip_heap.c
 */
typedef uint32_t   nu_ip_t;

#define NU_IP_TYPE_UNDEF    ((uint8_t)0xff)       ///< ip type is undefined
#define NU_IP_TYPE_V4       ((uint8_t)2)          ///< ip type is v4
#define NU_IP_TYPE_V6       ((uint8_t)30)         ///< ip type is v6

#define NU_IP_UNDEF         ((nu_ip_t)0xffffffff) ///< undefined ip

/**
 * Utility for IP address
 */
typedef union {
    uint8_t  b[4];  ///< for byte access
    uint32_t v;     ///< for 32bits unsigned int access
} nu_ip_util_t;

typedef char   nu_ip_str_t[128];                  ///< IP address for text form

/**
 * nu_ip_addr_t storage (PRIVATE TYPE, DO NOT USE THIS TYPE).
 */
typedef struct nu_ip_chunk {
    short        last;          ///< last
    short        remain;        ///< remain
    uint8_t      status[0x100]; ///< status
    nu_ip_addr_t addr[0x100];   ///< addr
    void*        attr[0x100];   ///< pointer to attribute of ip
} nu_ip_chunk_t;

/**
 * nu_ip_addr_t storage table (PRIVATE TYPE, DO NOT USE THIS TYPE).
 */
typedef struct nu_ip_heap {
    short          last;          ///< last
    short          remain;        ///< remain
    uint8_t        status[0x100]; ///< status
    nu_ip_chunk_t* chunk[0x100];  ///< chunk
} nu_ip_heap_t;

PUBLIC nu_ip_t nu_ip_create_with_str(const char*, short af);
PUBLIC nu_ip_t nu_ip4_create_with_str(const char*);
PUBLIC nu_ip_t nu_ip6_create_with_str(const char*);
PUBLIC nu_ip_t nu_ip_create_with_hmt(
        const uint8_t* head, size_t head_length,
        const uint8_t* tail, size_t tail_length,
        const uint8_t* mid, size_t mid_length);

PUBLIC const char* nu_ip_to_s(const nu_ip_t, nu_ip_str_t);

PUBLIC nu_bool_t nu_ip_is_overlap(const nu_ip_t, const nu_ip_t);

PUBLIC size_t nu_ip_get_head_length(const nu_ip_t, const nu_ip_t);
PUBLIC size_t nu_ip_get_tail_length(const nu_ip_t, const nu_ip_t);
PUBLIC nu_bool_t nu_ip_is_zero_tail(const nu_ip_t, const size_t tail_len);

PUBLIC void nu_ip_heap_init(nu_ip_heap_t*);
PUBLIC void nu_ip_heap_destroy(nu_ip_heap_t*);

PUBLIC nu_ip_t nu_ip4_heap_intern_(const nu_ip_addr_t*);
PUBLIC nu_ip_t nu_ip6_heap_intern_(const nu_ip_addr_t*);
//PUBLIC const nu_ip_addr_t* nu_ip_addr(const nu_ip_t);

/* *INDENT-OFF* */
/** type of constructor of ip_attr. */
typedef void*   (*nu_ip_attr_create_func_t)(const nu_ip_t);

/** type of destructor of ip_attr. */
typedef void    (*nu_ip_attr_free_func_t)(void*);
/* *INDENT-ON* */

PUBLIC void nu_ip_attr_set_constructor(nu_ip_attr_create_func_t);
PUBLIC void nu_ip_attr_set_destructor(nu_ip_attr_free_func_t);

/** Converts ip_addr to ip (IPv2).
 *
 * @param ip_addr
 * @return ip
 */
PUBLIC_INLINE nu_ip_t
nu_ip4(const nu_ip_addr_t* ip_addr)
{
    return nu_ip4_heap_intern_(ip_addr);
}

/** Converts ip_addr to ip (IPv6).
 *
 * @param ip_addr
 * @return ip
 */
PUBLIC_INLINE nu_ip_t
nu_ip6(const nu_ip_addr_t* ip_addr)
{
    return nu_ip6_heap_intern_(ip_addr);
}

/** Compares ip_addrs (IPv4).
 *
 * @param x
 * @param y
 * @retval <0 if x < y
 * @retval =0 if x == y
 * @retval >0 if x > y
 */
PUBLIC_INLINE int
nu_ip4_addr_cmp(const nu_ip_addr_t* x, const nu_ip_addr_t* y)
{
    int r;
    if ((r = nu_ntohs(x->i16[0]) - nu_ntohs(y->i16[0])) != 0)
        return r;
    return nu_ntohs(x->i16[1]) - nu_ntohs(y->i16[1]);
}

/** Compares ip_addrs (IPv6).
 *
 * @param x
 * @param y
 * @retval <0 if x < y
 * @retval =0 if x == y
 * @retval >0 if x > y
 */
PUBLIC_INLINE int
nu_ip6_addr_cmp(const nu_ip_addr_t* x, const nu_ip_addr_t* y)
{
    int r;
    if ((r = nu_ntohs(x->i16[0]) - nu_ntohs(y->i16[0])) != 0)
        return r;
    if ((r = nu_ntohs(x->i16[1]) - nu_ntohs(y->i16[1])) != 0)
        return r;
    if ((r = nu_ntohs(x->i16[2]) - nu_ntohs(y->i16[2])) != 0)
        return r;
    if ((r = nu_ntohs(x->i16[3]) - nu_ntohs(y->i16[3])) != 0)
        return r;
    if ((r = nu_ntohs(x->i16[4]) - nu_ntohs(y->i16[4])) != 0)
        return r;
    if ((r = nu_ntohs(x->i16[5]) - nu_ntohs(y->i16[5])) != 0)
        return r;
    if ((r = nu_ntohs(x->i16[6]) - nu_ntohs(y->i16[6])) != 0)
        return r;
    return nu_ntohs(x->i16[7]) - nu_ntohs(y->i16[7]);
}

/** Compares ip_addrs (IPv4).
 *
 * @param x
 * @param y
 * @retval true  if x == y
 * @retval false if x != y
 */
PUBLIC_INLINE nu_bool_t
nu_ip4_addr_eq(const nu_ip_addr_t* x, const nu_ip_addr_t* y)
{
    return x->u32[0] == y->u32[0];
}

/** Compares ip_addrs (IPv6).
 *
 * @param x
 * @param y
 * @retval true  if x == y
 * @retval false if x != y
 */
PUBLIC_INLINE int
nu_ip6_addr_eq(const nu_ip_addr_t* x, const nu_ip_addr_t* y)
{
    return x->u32[0] == y->u32[0] &&
           x->u32[1] == y->u32[1] &&
           x->u32[2] == y->u32[2] &&
           x->u32[3] == y->u32[3];
}

/* (PRIVATE INLINE FUNCTION) Intern ip address
 *
 * @param addr
 * @param af
 * @return nu_ip_t
 */
static inline nu_ip_t
nu_ip_heap_intern_(const nu_ip_addr_t* addr, const short af)
{
    switch (af) {
    case AF_INET:
        return nu_ip4_heap_intern_(addr);
    case AF_INET6:
        return nu_ip6_heap_intern_(addr);
    default:
        abort();
    }
}

/** Converts ip_addr to ip.
 *
 * @param ip_addr
 * @param af
 * @return nu_ip_t
 */
PUBLIC_INLINE nu_ip_t
nu_ip(const nu_ip_addr_t* ip_addr, const short af)
{
    return nu_ip_heap_intern_(ip_addr, af);
}

/** Gets ip type.
 *
 * @param ip
 * @return the ip version
 */
PUBLIC_INLINE uint8_t
nu_ip_type(const nu_ip_t ip)
{
    nu_ip_util_t u;
    u.v = ip;
    return u.b[0];
}

/** Gets address family.
 *
 * @param ip
 * @return the address family of the ip
 */
PUBLIC_INLINE short
nu_ip_af(const nu_ip_t ip)
{
    switch (nu_ip_type(ip)) {
    case NU_IP_TYPE_V4:
        return AF_INET;
    case NU_IP_TYPE_V6:
        return AF_INET6;
    default:
        abort();
    }
}

/** Checks whether ip refers IPv4 address.
 *
 * @param ip
 * @retval true  if the ip is IPv4 address
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_is_v4(const nu_ip_t ip)
{
    nu_ip_util_t u;
    u.v = ip;
    return NU_IP_TYPE_V4 == u.b[0];
}

/** Checks whether ip refers IPv6 address.
 *
 * @param ip
 * @retval true  if the ip is IPv6 address
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_is_v6(const nu_ip_t ip)
{
    nu_ip_util_t u;
    u.v = ip;
    return NU_IP_TYPE_V6 == u.b[0];
}

/** Checks whether ip refers undefined ip.
 *
 * @param ip
 * @retval true  if the ip is undefined
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_is_undef(const nu_ip_t ip)
{
    nu_ip_util_t u;
    u.v = ip;
    return NU_IP_TYPE_UNDEF == u.b[0];
}

/** XXX
 * @param ip
 * @retval true  if ip is routable address
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_is_routable(const nu_ip_t ip)
{
    return true;
}

/** Gets the prefix length of the ip.
 *
 * @param ip
 * @return the prefix length of the ip
 */
PUBLIC_INLINE uint8_t
nu_ip_prefix(const nu_ip_t ip)
{
    nu_ip_util_t u;
    u.v = ip;
    return u.b[1];
}

/** Checks whether ip has default prefix length.
 *
 * @param ip
 * @retval true  if the ip has the default prefix length
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_is_default_prefix(const nu_ip_t ip)
{
    if (nu_ip_is_v4(ip))
        return nu_ip_prefix(ip) == NU_IP4_LEN * 8;
    else if (nu_ip_is_v6(ip))
        return nu_ip_prefix(ip) == NU_IP6_LEN * 8;
    else
        abort();
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    return false;                      //ScenSim-Port://
#endif                                 //ScenSim-Port://
}

/** Sets the prefix length to ip.
 *
 * @param ip
 * @param prefix
 */
PUBLIC_INLINE void
nu_ip_set_prefix(nu_ip_t* ip, const uint8_t prefix)
{
    nu_ip_util_t u;
    u.v = *ip;
    u.b[1] = prefix;
    *ip = u.v;
}

/** Sets the default prefix length to ip.
 *
 * @param ip
 */
PUBLIC_INLINE void
nu_ip_set_default_prefix(nu_ip_t* ip)
{
    nu_ip_util_t u;
    u.v = *ip;
    if (nu_ip_is_v4(*ip))
        u.b[1] = NU_IP4_LEN * 8;
    else if (nu_ip_is_v6(*ip))
        u.b[1] = NU_IP6_LEN * 8;
    else
        abort();
    *ip = u.v;
}

/** Copy ip.
 *
 * @param dst
 * @param src
 */
PUBLIC_INLINE void
nu_ip_copy(nu_ip_t* dst, const nu_ip_t* src)
{
    if (dst != src)
        *dst = *src;
}

/** Compares ips with prefix.
 *
 * @param a
 * @param b
 * @retval <0 if a < b
 * @retval =0 if a == b
 * @retval >0 if a > b
 */
#if 0
PUBLIC_INLINE nu_bool_t
nu_ip_eq(const nu_ip_t a, const nu_ip_t b)
{
    assert(a != NU_IP_UNDEF);
    assert(b != NU_IP_UNDEF);
    return a == b;
}

#else
#define nu_ip_eq(a, b)    ((a) == (b))
#endif

/** Compares ips without prefix.
 *
 * @param a
 * @param b
 * @retval true  if a == b
 * @retval false if a != b
 */
PUBLIC_INLINE nu_bool_t
nu_ip_eq_without_prefix(const nu_ip_t a, const nu_ip_t b)
{
    assert(a != NU_IP_UNDEF);
    assert(b != NU_IP_UNDEF);
    nu_ip_util_t x;
    nu_ip_util_t y;
    x.v = a;
    y.v = b;
    return x.b[3] == y.b[3] && x.b[2] == y.b[2] && x.b[0] == y.b[0];
}

/** Compares originator addresses.
 *
 * @param a
 * @param b
 * @retval true  if a == b
 * @retval false if a != b
 */
PUBLIC_INLINE nu_bool_t
nu_orig_addr_eq(const nu_ip_t a, const nu_ip_t b)
{
    assert(a != NU_IP_UNDEF);
    assert(b != NU_IP_UNDEF);
    assert(nu_ip_is_default_prefix(a));
    assert(nu_ip_is_default_prefix(b));
    return a == b;
}

/* (PRIVATE)
 */
static inline size_t
nu_ip_hash(const nu_ip_t ip)
{
    assert(ip != NU_IP_UNDEF);
    nu_ip_util_t u;
    u.v = ip;
    return u.b[3];
}

/** Gets the length of the IP address in octet.
 *
 * @param ip
 * @return the length of ip
 */
PUBLIC_INLINE size_t
nu_ip_len(const nu_ip_t ip)
{
    assert(ip != NU_IP_UNDEF);
    return nu_ip_is_v4(ip) ? NU_IP4_LEN : NU_IP6_LEN;
}

/** Gets netmask image.
 *
 * @param ip
 * @return netmask
 */
PUBLIC_INLINE uint32_t
nu_ip4_netmask(const nu_ip_t ip)
{
    assert(nu_ip_is_v4(ip));
    return nu_htonl(0xffffffffU << (32 - nu_ip_prefix(ip)));
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
