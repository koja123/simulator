#include "config.h"

#include "core/ip_set.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_ip_container
 * @{
 */

/** Compares the two ip_sets.
 *
 * @param a
 * @param b
 * @return true if a is equal to b.
 */
PUBLIC nu_bool_t
nu_ip_set_eq(nu_ip_set_t* a, nu_ip_set_t* b)
{
    if (nu_ip_set_size(a) != nu_ip_set_size(b))
        return false;
    FOREACH_IP_SET(i, a) {
        if (!nu_ip_set_contain(b, i->ip))
            return false;
    }
    return true;
}

/** Syncs ip_set.
 *
 * @param dst
 * @param src
 * @retval true  if dst is modified.
 * @retval false if dst is not modified.
 */
PUBLIC nu_bool_t
nu_ip_set_copy(nu_ip_set_t* dst, const nu_ip_set_t* src)
{
    // XXX: Optimize
    nu_bool_t change = false;
    FOREACH_IP_SET(p, src) {
        if (!nu_ip_set_contain(dst, p->ip)) {
            nu_ip_set_add(dst, p->ip);
            change = true;
        }
    }
    if (nu_ip_set_size(dst) == nu_ip_set_size(src))
        return change;
    for (nu_ip_set_iter_t p = nu_ip_set_iter(dst);
         !nu_ip_set_iter_is_end(p, dst);) {
        if (nu_ip_set_contain(src, p->ip))
            p = nu_ip_set_iter_next(p, dst);
        else {
            p = nu_ip_set_iter_remove(p, dst);
            change = true;
        }
    }
    return change;
}

/** @} */

}//namespace// //ScenSim-Port://
