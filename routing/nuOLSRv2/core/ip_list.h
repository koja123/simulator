#ifndef NU_CORE_IP_LIST_H_
#define NU_CORE_IP_LIST_H_

#include "core/ip.h"
#include "core/mem.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_ip_container Core :: IP address containers
 * @{
 */

/**
 * Element of ip containers
 */
typedef struct nu_ip_elt {
    struct nu_ip_elt* next;   ///< next element
    struct nu_ip_elt* prev;   ///< prev element
    nu_ip_t           ip;     ///< ip
} nu_ip_elt_t;

/**
 * Iterator of the ip_list.
 */
typedef nu_ip_elt_t*  nu_ip_list_iter_t;

/**
 * Accessor
 */
#define nu_ip_list_iter_get(p)    ((p)->ip)

/**
 * ip containers
 */
typedef struct nu_ip_list {
    nu_ip_elt_t* next;    ///< the first element
    nu_ip_elt_t* prev;    ///< the last element
    size_t       n;       ///< size
} nu_ip_list_t;

////////////////////////////////////////////////////////////////

/**
 * Destructor of nu_ip_elt
 */
#define nu_ip_elt_free    nu_mem_free

}//namespace// //ScenSim-Port://

#include "core/ip_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Gets the first element
 *
 * @param self
 * @return the first ip in the list or,
 *         NU_IP_UNDEF if list is empty
 */
PUBLIC_INLINE nu_ip_t
nu_ip_list_head(const nu_ip_list_t* self)
{
    if (nu_ip_list_size(self) == 0)
        return NU_IP_UNDEF;
    return self->next->ip;
}

/** Inserts the new ip after the ip that is pointed by the iterator.
 *
 * @param self
 * @param ip
 * @param iter
 */
PUBLIC_INLINE void
nu_ip_list_insert_after(nu_ip_list_t* self,
        nu_ip_t ip, nu_ip_list_iter_t iter)
{
    nu_ip_elt_t* elt = nu_mem_alloc(nu_ip_elt_t);
    elt->ip = ip;
    nu_ip_list_iter_insert_after(iter, self, elt);
}

/** Inserts the new ip before the ip that is pointed by the iterator.
 *
 * @param self
 * @param ip
 * @param iter
 */
PUBLIC_INLINE void
nu_ip_list_insert_before(nu_ip_list_t* self,
        nu_ip_t ip, nu_ip_list_iter_t iter)
{
    nu_ip_elt_t* elt = nu_mem_alloc(nu_ip_elt_t);
    elt->ip = ip;
    nu_ip_list_iter_insert_before(iter, self, elt);
}

/** Inserts the ip into the top of the list.
 *
 * @param self
 * @param ip
 */
PUBLIC_INLINE void
nu_ip_list_insert_head(nu_ip_list_t* self, nu_ip_t ip)
{
    nu_ip_list_insert_after(self, ip, (nu_ip_list_iter_t)self);
}

/** Appends the ip to the last of the list.
 *
 * @param self
 * @param ip
 */
PUBLIC_INLINE void
nu_ip_list_insert_tail(nu_ip_list_t* self, nu_ip_t ip)
{
    nu_ip_list_insert_before(self, ip, (nu_ip_list_iter_t)self);
}

/** Removes the top ip of the list.
 *
 * @param self
 * @return removed ip or,
 *	   NU_IP_UNDEF if list is empty
 */
PUBLIC_INLINE nu_ip_t
nu_ip_list_remove_head(nu_ip_list_t* self)
{
    if (nu_ip_list_is_empty(self))
        return NU_IP_UNDEF;
    nu_ip_t r = nu_ip_list_head(self);
    nu_ip_list_iter_remove(self->next, self);
    return r;
}

/** Removes all the ip from the list.
 *
 * @param self
 * @param ip
 */
PUBLIC_INLINE void
nu_ip_list_remove_ip(nu_ip_list_t* self, const nu_ip_t ip)
{
    if (nu_ip_list_is_empty(self))
        return;
    nu_ip_list_iter_t p;
    for (p = nu_ip_list_iter(self);
         !nu_ip_list_iter_is_end(p, self);) {
        if (nu_ip_eq(ip, p->ip))
            p = nu_ip_list_iter_remove(p, self);
        else
            p = nu_ip_list_iter_next(p, self);
    }
}

/** Searches the ip.
 *
 * @param self
 * @param ip
 * @return the iterator pointing ip.
 */
PUBLIC_INLINE nu_ip_list_iter_t
nu_ip_list_search(const nu_ip_list_t* self, const nu_ip_t ip)
{
    FOREACH_IP_LIST(p, self) {
        if (nu_ip_eq(ip, p->ip))
            return p;
    }
    return nu_ip_list_iter_end(self);
}

/** Searches the ip which overlapped with the ip.
 *
 * @param self
 * @param ip
 * @return the iterator for ip that overlapped with the ip if it is found,
 *	   otherwise return the iter that points end of the ip_list.
 */
PUBLIC_INLINE nu_ip_list_iter_t
nu_ip_list_search_overlap(const nu_ip_list_t* self, const nu_ip_t ip)
{
    FOREACH_IP_LIST(p, self) {
        if (nu_ip_is_overlap(ip, p->ip))
            return p;
    }
    return nu_ip_list_iter_end(self);
}

/** Checks whether the list contains the ip.
 *
 * @param self
 * @param ip
 * @retval true  if self contains the ip
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_list_contain(const nu_ip_list_t* self, const nu_ip_t ip)
{
    return !nu_ip_list_iter_is_end(
            nu_ip_list_search(self, ip), self);
}

/** Checks whether the list contains ip which overlapped the ip.
 *
 * @param self
 * @param ip
 * @retval true  if self overlaps ip
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_list_is_overlap_ip(const nu_ip_list_t* self, const nu_ip_t ip)
{
    return !nu_ip_list_iter_is_end(
            nu_ip_list_search_overlap(self, ip), self);
}

/** Compares the two ip_lists.
 *
 * @param self
 * @param target
 * @retval ture if self contains at least one ip address in target.
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_list_contain_at_least_one(const nu_ip_list_t* self,
        const nu_ip_list_t* target)
{
    FOREACH_IP_LIST(p, target) {
        if (nu_ip_list_contain(self, p->ip))
            return true;
    }
    return false;
}

/** Compares the two ip_lists.
 *
 * @param self
 * @param target
 * @retval ture  if self overlaps target.
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_list_is_overlap_ip_list(const nu_ip_list_t* self,
        const nu_ip_list_t* target)
{
    FOREACH_IP_LIST(p, target) {
        if (nu_ip_list_is_overlap_ip(self, p->ip))
            return true;
    }
    return false;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
