#include "config.h"

#include "core/core.h"
#include "core/ip_inline.h"

#if defined(_WIN32) && !defined(_WIN64) //ScenSim-Port://
#include <sstream>                      //ScenSim-Port://
#endif                                  //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

#if defined(_WIN32) && !defined(_WIN64)                                //ScenSim-Port://
static const char*                                                     //ScenSim-Port://
inet_ntop(int af, const nu_ip_addr_t* ip, nu_ip_str_t buf, size_t len) //ScenSim-Port://
{                                                                      //ScenSim-Port://
    uint32_t ipAddress;                                                //ScenSim-Port://
    uint64_t ipAddressHighBits;                                        //ScenSim-Port://
    uint64_t ipAddressLowBits;                                         //ScenSim-Port://
    std::ostringstream outStream;                                      //ScenSim-Port://
    switch (af) {                                                      //ScenSim-Port://
    case AF_INET:                                                      //ScenSim-Port://
        ipAddress = nu_ntohl(ip->u32[0]);                              //ScenSim-Port://
        outStream                                                      //ScenSim-Port://
            << (ipAddress / 0x01000000) << '.'                         //ScenSim-Port://
            << ((ipAddress / 0x00010000) % 256) << '.'                 //ScenSim-Port://
            << ((ipAddress / 0x00000100) % 256) << '.'                 //ScenSim-Port://
            << (ipAddress % 256);                                      //ScenSim-Port://
        break;                                                         //ScenSim-Port://
    case AF_INET6:                                                     //ScenSim-Port://
        ConvertNet128ToTwoHost64(                                      //ScenSim-Port://
            ip->u8, ipAddressHighBits, ipAddressLowBits);              //ScenSim-Port://
        outStream                                                      //ScenSim-Port://
            << (ipAddressHighBits / 0x0001000000000000) << ':'         //ScenSim-Port://
            << ((ipAddressHighBits / 0x0000000100000000) & 0xFFFF) << ':' //ScenSim-Port://
            << ((ipAddressHighBits / 0x0000000000001000) & 0xFFFF) << ':' //ScenSim-Port://
            << (ipAddressHighBits & 0xFFFF) << ':'                     //ScenSim-Port://
            << (ipAddressLowBits / 0x0001000000000000) << ':'          //ScenSim-Port://
            << ((ipAddressLowBits / 0x0000000100000000) & 0xFFFF) << ':' //ScenSim-Port://
            << ((ipAddressLowBits / 0x0000000000001000) & 0xFFFF) << ':' //ScenSim-Port://
            << (ipAddressLowBits & 0xFFFF);                            //ScenSim-Port://
        break;                                                         //ScenSim-Port://
    default:                                                           //ScenSim-Port://
        assert(0);                                                     //ScenSim-Port://
    }                                                                  //ScenSim-Port://
    strcpy_s(buf, len, outStream.str().c_str());                       //ScenSim-Port://
    return buf;                                                        //ScenSim-Port://
}                                                                      //ScenSim-Port://
#endif                                                                 //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_ip
 * @{
 */

/* @see nu_ip4_create_with_str
 */
static nu_bool_t
nu_ip4_addr_set_with_str(nu_ip_addr_t* data, const char* str)
{
#if defined(_WIN32) && !defined(_WIN64)       //ScenSim-Port://
        assert(false);                        //ScenSim-Port://
        exit(1);                              //ScenSim-Port://
#else                                         //ScenSim-Port://
    if (inet_pton(AF_INET, str, data) != 1) {
        perror("inet_pton");
        return false;
    }
#endif                                        //ScenSim-Port://
    return true;
}

/* @see nu_ip6_create_with_str
 */
static nu_bool_t
nu_ip6_addr_set_with_str(nu_ip_addr_t* data, const char* str)
{
#if defined(_WIN32) && !defined(_WIN64)       //ScenSim-Port://
        assert(false);                        //ScenSim-Port://
        exit(1);                              //ScenSim-Port://
#else                                         //ScenSim-Port://
    if (inet_pton(AF_INET6, str, data) != 1) {
        perror("inet_pton");
        return false;
    }
#endif                                        //ScenSim-Port://
    return true;
}

#if 0
static void
nu_ip4_addr_copy(nu_ip_addr_t* dst, const nu_ip_addr_t* src)
{
    dst->u32[0] = src->u32[0];
}

static void
nu_ip6_addr_copy(nu_ip_addr_t* dst, const nu_ip_addr_t* src)
{
    dst->u32[0] = src->u32[0];
    dst->u32[1] = src->u32[1];
    dst->u32[2] = src->u32[2];
    dst->u32[3] = src->u32[3];
}

#endif

/** Compares ip addresses.
 *
 * @param a
 * @param b
 * @param len   prefix length in bytes.
 * @return true if a and b have same prefix (in bytes)
 */
PUBLIC nu_bool_t
nu_ip_addr_eq_prefix(const nu_ip_addr_t* a, const nu_ip_addr_t* b,
        const size_t len)
{
    return memcmp(a, b, len) == 0;
}

/* @see nu_ip_get_head_length
 */
static size_t
nu_ip4_addr_get_head_length(const nu_ip_addr_t* x, const nu_ip_addr_t* y)
{
    size_t i;
    for (i = 0; i < 4 && x->u8[i] == y->u8[i]; ++i)
        ;
    return i;
}

/* @see nu_ip_get_head_length
 */
static size_t
nu_ip6_addr_get_head_length(const nu_ip_addr_t* x, const nu_ip_addr_t* y)
{
    size_t i;
    for (i = 0; i < 16 && x->u8[i] == y->u8[i]; ++i)
        ;
    return i;
}

/** Gets the head length of ip.
 *
 * @param x
 * @param y
 * @return the head length of x and y
 */
PUBLIC size_t
nu_ip_get_head_length(const nu_ip_t x, const nu_ip_t y)
{
    assert(nu_ip_type(x) == nu_ip_type(y));
    if (nu_ip_is_v4(x))
        return nu_ip4_addr_get_head_length(nu_ip_addr(x), nu_ip_addr(y));
    else if (nu_ip_is_v6(x))
        return nu_ip6_addr_get_head_length(nu_ip_addr(x), nu_ip_addr(y));
    else
        nu_fatal("Internal Error");
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    return 0;                          //ScenSim-Port://
#endif                                 //ScenSim-Port://
}

/* @see nu_ip_get_tail_length
 */
static size_t
nu_ip4_addr_get_tail_length(const nu_ip_addr_t* x, const nu_ip_addr_t* y)
{
    if (x->u8[3] != y->u8[3])
        return 0;
    if (x->u8[2] != y->u8[2])
        return 1;
    if (x->u8[1] != y->u8[1])
        return 2;
    if (x->u8[0] != y->u8[0])
        return 3;
    return 4;
}

/* @see nu_ip_get_tail_length
 */
static size_t
nu_ip6_addr_get_tail_length(const nu_ip_addr_t* x, const nu_ip_addr_t* y)
{
    int i;
    for (i = 16 - 1; i >= 0 && x->u8[i] == y->u8[i]; --i)
        ;
    return (size_t)(16 - i - 1);
}

/** Gets the tail length of ip.
 *
 * @param x
 * @param y
 * @return the tail length of x and y
 */
PUBLIC size_t
nu_ip_get_tail_length(const nu_ip_t x, const nu_ip_t y)
{
    assert(nu_ip_type(x) == nu_ip_type(y));
    if (nu_ip_is_v4(x))
        return nu_ip4_addr_get_tail_length(nu_ip_addr(x), nu_ip_addr(y));
    else if (nu_ip_is_v6(x))
        return nu_ip6_addr_get_tail_length(nu_ip_addr(x), nu_ip_addr(y));
    else
        nu_fatal("Internal Error");
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    return 0;                          //ScenSim-Port://
#endif                                 //ScenSim-Port://
}

/* @see nu_ip_is_zero_tail
 */
static nu_bool_t
nu_ip4_addr_is_zero_tail(const nu_ip_addr_t* a, const size_t tail_len)
{
    const uint8_t* x = (uint8_t*)a;
    for (unsigned int i = 4 - 1; i >= 4 - tail_len; --i) {
        if (x[i] != 0)
            return false;
    }
    return true;
}

/* @see nu_ip_is_zero_tail
 */
static nu_bool_t
nu_ip6_addr_is_zero_tail(const nu_ip_addr_t* a, const size_t tail_len)
{
    const uint8_t* x = (uint8_t*)a;
    for (unsigned int i = 16 - 1; i >= 16 - tail_len; --i) {
        if (x[i] != 0)
            return false;
    }
    return true;
}

/** Checks whether ip has zero tail.
 *
 * @param ip
 * @param tail_len
 * @return true if ip has zero tail.
 */
PUBLIC nu_bool_t
nu_ip_is_zero_tail(const nu_ip_t ip, const size_t tail_len)
{
    if (nu_ip_is_v4(ip))
        return nu_ip4_addr_is_zero_tail(nu_ip_addr(ip), tail_len);
    else if (nu_ip_is_v6(ip))
        return nu_ip6_addr_is_zero_tail(nu_ip_addr(ip), tail_len);
    else {
        nu_fatal("Internal Error");
        /* NOTREACH */
    }
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    return 0;                          //ScenSim-Port://
#endif                                 //ScenSim-Port://
}

/**
 * Converts ip_addr to string.
 *
 * @param ip
 * @param buf
 * @return
 */
PUBLIC const char*
nu_ip_addr_to_s(const nu_ip_addr_t* ip, nu_ip_str_t buf)
{
#if defined(_WIN64)                                               //ScenSim-Port://
    return inet_ntop(NU_AF, (void*)ip, buf, sizeof(nu_ip_str_t)); //ScenSim-Port://
#else                                                             //ScenSim-Port://
    return inet_ntop(NU_AF, ip, buf, sizeof(nu_ip_str_t));
#endif                                                            //ScenSim-Port://
}

////////////////////////////////////////////////////////////////
//
// ip
//

/**
 * Creates ip from str and address family.
 *
 * @param str
 * @param af
 * @return ip
 */
PUBLIC nu_ip_t
nu_ip_create_with_str(const char* str, const short af)
{
    if (af == AF_INET)
        return nu_ip4_create_with_str(str);
    else if (af == AF_INET6)
        return nu_ip6_create_with_str(str);
    else
        abort();
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    return 0;                          //ScenSim-Port://
#endif                                 //ScenSim-Port://
}

/** Creates ip from str (IPv4)
 *
 * @param str
 * @return ip
 */
PUBLIC nu_ip_t
nu_ip4_create_with_str(const char* str)
{
    nu_ip_addr_t data;
    int  prefix;
#if defined(_WIN32) || defined(_WIN64)     //ScenSim-Port://
    char* buf = new char[strlen(str) + 1]; //ScenSim-Port://
#else                                      //ScenSim-Port://
    char buf[strlen(str) + 1];
#endif                                     //ScenSim-Port://
    memcpy(buf, str, strlen(str) + 1);
    char* p;
    for (p = buf; *p != '\0' && *p != '/'; ++p)
        ;
    if (*p != '/')
        prefix = 32;
    else {
        *p = '\0';
        prefix = strtoul(++p, NULL, 10);
    }
    if (!nu_ip4_addr_set_with_str(&data, buf))
        goto error;
    else {
        nu_ip_t self = nu_ip4_heap_intern_(&data);
        nu_ip_set_prefix(&self, static_cast<uint8_t>(prefix));
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
        delete[] buf;                  //ScenSim-Port://
#endif                                 //ScenSim-Port://
        return self;
    }

error:
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    delete[] buf;                      //ScenSim-Port://
#endif                                 //ScenSim-Port://
    return NU_IP_UNDEF;
}

/** Creates ip from str (IPv6).
 *
 * @param str
 * @return ip
 */
PUBLIC nu_ip_t
nu_ip6_create_with_str(const char* str)
{
    nu_ip_addr_t data;
    int  prefix;
#if defined(_WIN32) || defined(_WIN64)     //ScenSim-Port://
    char* buf = new char[strlen(str) + 1]; //ScenSim-Port://
#else                                      //ScenSim-Port://
    char buf[strlen(str) + 1];
#endif                                     //ScenSim-Port://
    memcpy(buf, str, strlen(str) + 1);
    char* p;
    for (p = buf; *p != '\0' && *p != '/'; ++p)
        ;
    if (*p != '/')
        prefix = 128;
    else {
        *p = '\0';
        prefix = strtoul(++p, NULL, 10);
    }
    if (!nu_ip6_addr_set_with_str(&data, buf))
        goto error;
    else {
        nu_ip_t self = nu_ip6_heap_intern_(&data);
        nu_ip_set_prefix(&self, static_cast<uint8_t>(prefix));
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
        delete[] buf;                  //ScenSim-Port://
#endif                                 //ScenSim-Port://
        return self;
    }

error:
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    delete[] buf;                      //ScenSim-Port://
#endif                                 //ScenSim-Port://
    return NU_IP_UNDEF;
}

/** Creates ip from head, tail, and mid.
 *
 * @param head      prefix of IP address
 * @param head_length   length of the head [bytes]
 * @param tail      tail of IP address
 * @param tail_length   length of the tail [bytes]
 * @param mid       mid of IP address
 * @param mid_length    length of the mid [bytes]
 * @return ip
 */
PUBLIC nu_ip_t
nu_ip_create_with_hmt(
        const uint8_t* head, size_t head_length,
        const uint8_t* tail, size_t tail_length,
        const uint8_t* mid, size_t mid_length)
{
    nu_ip_addr_t ipa;
    uint8_t*     p = ipa.u8;
    memcpy(p, head, head_length);
    memcpy(p + head_length, mid, mid_length);
    memcpy(p + head_length + mid_length, tail, tail_length);

    if (head_length + mid_length + tail_length == NU_IP4_LEN)
        return nu_ip4_heap_intern_(&ipa);
    else if (head_length + mid_length + tail_length == NU_IP6_LEN)
        return nu_ip6_heap_intern_(&ipa);
    else {
        nu_fatal("Internal error");
        /* NOTREACH */
    }
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    return 0;                          //ScenSim-Port://
#endif                                 //ScenSim-Port://
}

/** Converts ip to string.
 *
 * @param ip
 * @param buf
 * @return buf
 */
PUBLIC const char*
nu_ip_to_s(const nu_ip_t ip, nu_ip_str_t buf)
{
    if (nu_ip_is_v4(ip)) {
#if defined(_WIN64)                                                          //ScenSim-Port://
        inet_ntop(AF_INET, (void*)nu_ip_addr(ip), buf, sizeof(nu_ip_str_t)); //ScenSim-Port://
#else                                                                        //ScenSim-Port://
        inet_ntop(AF_INET, nu_ip_addr(ip), buf, sizeof(nu_ip_str_t));
#endif                                                                       //ScenSim-Port://
        if (nu_ip_prefix(ip) == 32)
            return buf;
    } else if (nu_ip_is_v6(ip)) {
#if defined(_WIN64)                                                           //ScenSim-Port://
        inet_ntop(AF_INET6, (void*)nu_ip_addr(ip), buf, sizeof(nu_ip_str_t)); //ScenSim-Port://
#else                                                                         //ScenSim-Port://
        inet_ntop(AF_INET6, nu_ip_addr(ip), buf, sizeof(nu_ip_str_t));
#endif                                                                        //ScenSim-Port://
        if (nu_ip_prefix(ip) == 128)
            return buf;
    } else
        assert(0);

    char pbuf[5];
    snprintf(pbuf, 5, "/%d", nu_ip_prefix(ip));
    strncat(buf, pbuf, sizeof(nu_ip_str_t));
    return buf;
}

/** Checks whether ips are overlapped.
 *
 * @param a
 * @param b
 * @return true if a and b is overlapped.
 */
PUBLIC nu_bool_t
nu_ip_is_overlap(const nu_ip_t a, const nu_ip_t b)
{
#if 0
    return nu_ip_eq(a, b);
#else
    if (a == b)
        return true;
    if (nu_ip_is_v4(a) && nu_ip_is_v4(b)) {
        if (nu_ip_is_default_prefix(a) && nu_ip_is_default_prefix(b))
            return false;
        const nu_ip_addr_t* x = nu_ip_addr(a);
        const nu_ip_addr_t* y = nu_ip_addr(b);
        const uint32_t mask = (nu_ip_prefix(a) < nu_ip_prefix(b))
                              ? nu_ip4_netmask(a) : nu_ip4_netmask(b);
        return (x->u32[0] & mask) == (y->u32[0] & mask);
    } else if (nu_ip_is_v6(a) && nu_ip_is_v6(b)) {
        if (nu_ip_is_default_prefix(a) && nu_ip_is_default_prefix(b))
            return false;
        const nu_ip_addr_t* x = nu_ip_addr(a);
        const nu_ip_addr_t* y = nu_ip_addr(b);
        size_t idx = 0;
        size_t prefix = (nu_ip_prefix(a) < nu_ip_prefix(b))
                        ? nu_ip_prefix(a) : nu_ip_prefix(b);
        while (prefix >= 32) {
            if (x->u32[idx] != y->u32[idx])
                return false;
            prefix -= 32;
            idx += 1;
        }
        if (prefix == 0)
            return true;
        const uint32_t mask = nu_htonl(0xffffffffU << (32 - prefix));
        return (x->u32[idx] & mask) == (y->u32[idx] & mask);
    } else
        return false;
#endif
}

/** @} */

}//namespace// //ScenSim-Port://
