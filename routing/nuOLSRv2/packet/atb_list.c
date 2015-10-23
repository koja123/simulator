#include "config.h"

#include "packet/atb_list.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_atb_list
 * @{
 */

/** Frees atb_list_elt.
 *
 * @param self
 */
PUBLIC void
nu_atb_list_elt_free(nu_atb_list_iter_t self)
{
    nu_atb_destroy(&self->atb);
    nu_mem_free(self);
}

/** Returns atb.
 *
 * @param p
 */
PUBLIC nu_atb_t*
nu_atb_list_iter_atb(const nu_atb_list_iter_t p)
{
    return &p->atb;
}

/** Adds atb.
 *
 * @param self
 * @return atb
 */
PUBLIC nu_atb_t*
nu_atb_list_add(nu_atb_list_t* self)
{
    nu_atb_list_elt_t* entry = nu_mem_alloc(nu_atb_list_elt_t);
    nu_atb_init(&entry->atb);
    nu_atb_list_iter_insert_before((nu_atb_list_elt_t*)self, self, entry);
    return (nu_atb_t*)&entry->atb;
}

/** Compares atb_list.
 *
 * @param a
 * @param b
 * @return true if a == b
 */
PUBLIC nu_bool_t
nu_atb_list_eq(const nu_atb_list_t* a, const nu_atb_list_t* b)
{
    if (nu_atb_list_size(a) != nu_atb_list_size(b))
        return false;
    nu_atb_list_iter_t pa = nu_atb_list_iter(a);
    nu_atb_list_iter_t pb = nu_atb_list_iter(b);
    while (!nu_atb_list_iter_is_end(pa, a)) {
        if (!nu_atb_eq(&pa->atb, &pb->atb))
            return false;
        pa = nu_atb_list_iter_next(pa, a);
        pb = nu_atb_list_iter_next(pb, b);
    }
    return true;
}

/** @} */

}//namespace// //ScenSim-Port://
