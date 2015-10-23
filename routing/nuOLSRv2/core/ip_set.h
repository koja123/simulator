#ifndef NU_CORE_IP_SET_H_
#define NU_CORE_IP_SET_H_

#include "core/ip_list.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_ip_container
 * @{
 */

/**
 * ip set.
 */
typedef nu_ip_list_t        nu_ip_set_t;

/**
 * iterator of ip_set.
 */
typedef nu_ip_list_iter_t   nu_ip_set_iter_t;

/**
 * accessor
 */
#define nu_ip_set_iter_get(p)    ((p)->ip)

////////////////////////////////////////////////////////////////
//
// ip_set global functions
//

PUBLIC nu_bool_t nu_ip_set_eq(nu_ip_set_t*, nu_ip_set_t*);
PUBLIC nu_bool_t nu_ip_set_copy(nu_ip_set_t* dst,
        const nu_ip_set_t* src);

}//namespace// //ScenSim-Port://

#include "core/ip_set_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

////////////////////////////////////////////////////////////////
//
// ip_set inline functions
//

/** Creates ip_set.
 *
 * @return ip_set
 */
PUBLIC_INLINE nu_ip_set_t*
nu_ip_set_create(void)
{
    nu_ip_set_t* ip_set = nu_mem_alloc(nu_ip_set_t);
    nu_ip_set_init(ip_set);
    return ip_set;
}

/** Destroys and frees the ip_set.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_ip_set_free(nu_ip_set_t* self)
{
    nu_ip_set_destroy(self);
    nu_mem_free(self);
}

/**
 * @param self
 * @return the top of the elements
 */
PUBLIC_INLINE nu_ip_t
nu_ip_set_head(const nu_ip_set_t* self)
{
    if (nu_ip_set_is_empty(self))
        return NU_IP_UNDEF;
    return self->next->ip;
}

/** Inserts the ip to the ip_set.
 *
 * @param self
 * @param ip
 * @retval true  if ip_set is changed.
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_set_add(nu_ip_set_t* self, nu_ip_t ip)
{
    FOREACH_IP_SET(p, self) {
        if (nu_ip_eq(ip, p->ip))
            return false;
    }
    nu_ip_list_insert_head(self, ip);
    return true;
}

/** Checks whether the ip_set contains the ip.
 *
 * ips are compared by nu_ip_eq_without_prefix().
 *
 * @param self
 * @param ip
 * @retval true  if self contains the ip
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_set_contain_without_prefix(const nu_ip_set_t* self, nu_ip_t ip)
{
    FOREACH_IP_SET(p, self) {
        if (nu_ip_eq_without_prefix(ip, p->ip))
            return true;
    }
    return false;
}

/** Checks whether the ip_set contains the ip.
 *
 * @param self
 * @param ip
 * @retval true  if self contains ip
 * @retval flase otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_set_contain(const nu_ip_set_t* self, nu_ip_t ip)
{
    FOREACH_IP_SET(p, self) {
        if (nu_ip_eq(ip, p->ip))
            return true;
    }
    return false;
}

/** Checks whether the ip_set contains the ip in the ip_set.
 *
 * @param self
 * @param target
 * @retval true  if self contains at least one ip in target
 * @retval flase otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_set_contain_at_least_one(const nu_ip_set_t* self,
        const nu_ip_set_t* target)
{
    return nu_ip_list_contain_at_least_one(self, target);
}

/** Removes the ip address from the ip_set.
 *
 * @param self
 * @return ip
 */
PUBLIC_INLINE nu_ip_t
nu_ip_set_remove_head(nu_ip_set_t* self)
{
    assert(self->n > 0);
    nu_ip_t r = self->prev->ip;
    nu_ip_set_iter_remove(self->prev, self);
    return r;
}

/** Removes the specified ip from the ip_set.
 *
 * @param self
 * @param ip
 */
PUBLIC_INLINE void
nu_ip_set_remove_ip(nu_ip_set_t* self, nu_ip_t ip)
{
    FOREACH_IP_SET(p, self) {
        if (nu_ip_eq(ip, p->ip)) {
            nu_ip_list_iter_remove(p, self);
            return;
        }
    }
}

/** Removes the ips in the ip_set from the ip_set.
 *
 * @param self
 * @param ip_set
 */
PUBLIC_INLINE void
nu_ip_set_remove_ip_set(nu_ip_set_t* self, const nu_ip_set_t* ip_set)
{
    FOREACH_IP_SET(p, ip_set) {
        nu_ip_set_remove_ip(self, p->ip);
    }
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
