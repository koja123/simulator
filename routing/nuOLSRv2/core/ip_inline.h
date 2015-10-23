#ifndef NU_IP_INLINE_H__
#define NU_IP_INLINE_H__

#include "config.h" //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_ip
 * @{
 */

/** Gets ip_addr from the ip.
 *
 * DON'T MODIFY RETURNED IP ADDRESS.
 * If you want to modify it, use nu_ip_addr_copy().
 *
 * @param ip
 * @return ip address
 */
PUBLIC_INLINE const nu_ip_addr_t*
nu_ip_addr(const nu_ip_t ip)
{
    nu_ip_util_t u;
    u.v = ip;
    //assert(!nu_ip_is_undef(ip));
    //assert(u.b[2] <= NU_IP_HEAP.last);
    return &NU_IP_HEAP.chunk[u.b[2]]->addr[u.b[3]];
}

/** Compares ips with prefix.
 *
 * @param a
 * @param b
 * @retval <0 if a < b
 * @retval =0 if a == b
 * @retval >0 if a > b
 */
PUBLIC_INLINE int
nu_ip_cmp(const nu_ip_t a, const nu_ip_t b)
{
    assert(a != NU_IP_UNDEF);
    assert(b != NU_IP_UNDEF);
    if (a == b)
        return 0;
    int r = 0;
    if ((r = nu_ip_type(a) - nu_ip_type(b)) != 0)
        return r;
    if (nu_ip_is_v4(a)) {
        if ((r = nu_ip4_addr_cmp(nu_ip_addr(a), nu_ip_addr(b))) != 0)
            return r;
    } else if (nu_ip_is_v6(a)) {
        if ((r = nu_ip6_addr_cmp(nu_ip_addr(a), nu_ip_addr(b))) != 0)
            return r;
    } else
        abort();
    return nu_ip_prefix(a) - nu_ip_prefix(b);
}

/** Compares ips without prefix.
 *
 * @param a
 * @param b
 * @retval <0 if a < b
 * @retval =0 if a == b
 * @retval >0 if a > b
 */
PUBLIC_INLINE int
nu_ip_cmp_without_prefix(const nu_ip_t a, const nu_ip_t b)
{
    assert(a != NU_IP_UNDEF);
    assert(b != NU_IP_UNDEF);
    if (a == b)
        return 0;
    int r = 0;
    if ((r = nu_ip_type(a) - nu_ip_type(b)) != 0)
        return r;
    if (nu_ip_is_v4(a)) {
        r = nu_ip4_addr_cmp(nu_ip_addr(a), nu_ip_addr(b));
        return r;
    } else if (nu_ip_is_v6(a)) {
        return nu_ip6_addr_cmp(nu_ip_addr(a), nu_ip_addr(b));
    } else {
        abort();
    }
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    return 0;                          //ScenSim-Port://
#endif                                 //ScenSim-Port://
}

/** Compares originator addresses.
 *
 * @param a
 * @param b
 * @retval <0 if a < b
 * @retval =0 if a == b
 * @retval >0 if a > b
 */
PUBLIC_INLINE int
nu_orig_addr_cmp(const nu_ip_t a, const nu_ip_t b)
{
    assert(a != NU_IP_UNDEF);
    assert(b != NU_IP_UNDEF);
    assert(nu_ip_is_default_prefix(a));
    assert(nu_ip_is_default_prefix(b));
    return nu_ip_cmp_without_prefix(a, b);
}

/** Copies originator address.
 *
 * @param[out] dst
 * @param src
 */
PUBLIC_INLINE void
nu_orig_addr_copy(nu_ip_t* dst, const nu_ip_t src)
{
    *dst = src;
    nu_ip_set_default_prefix(dst);
}

/** Copies ip_addr.
 *
 * @param dst	destination address (nu_ip_addr_t)
 * @param ip
 */
PUBLIC_INLINE void
nu_ip_addr_copy(void* dst, const nu_ip_t ip)
{
    assert(ip != NU_IP_UNDEF);
    if (nu_ip_is_v4(ip))
        memcpy(dst, nu_ip_addr(ip), NU_IP4_LEN);
    else if (nu_ip_is_v6(ip))
        memcpy(dst, nu_ip_addr(ip), NU_IP6_LEN);
    else
        abort();
}

/** Search attribute of ip_addr from the ip.
 *
 * @param ip
 * @return pointer to attribute or NULL.
 */
PRIVATE_INLINE void*
nu_ip_attr_search(const nu_ip_t ip)
{
    nu_ip_util_t u;
    u.v = ip;
    assert(!nu_ip_is_undef(ip));
    assert(u.b[2] <= NU_IP_HEAP.last);
    return NU_IP_HEAP.chunk[u.b[2]]->attr[u.b[3]];
}

/** Gets attribute of ip_addr from the ip.
 *
 * @param ip
 * @param attr
 * @return point to old attribute or NULL.
 */
PUBLIC_INLINE void*
nu_ip_attr_set(const nu_ip_t ip, void* attr)
{
    nu_ip_util_t u;
    u.v = ip;
    assert(!nu_ip_is_undef(ip));
    assert(u.b[2] <= NU_IP_HEAP.last);
    void* old_attr = NU_IP_HEAP.chunk[u.b[2]]->attr[u.b[3]];
    NU_IP_HEAP.chunk[u.b[2]]->attr[u.b[3]] = attr;
    return old_attr;
}

/** Gets attribute of ip_addr from the ip.
 *
 * @param ip
 * @return pointer to attribute or NULL.
 */
PUBLIC_INLINE void*
nu_ip_attr_get(const nu_ip_t ip)
{
    void* r = nu_ip_attr_search(ip);
    if (r != NULL)
        return r;
    r = NU_IP_ATTR_CONSTRUCTOR(ip);
    nu_ip_attr_set(ip, r);
    return r;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
