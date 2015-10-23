//
// TLV set
//
// The elements are sorted by tlv type value.
// @see nu_tlv_cmp
//
#ifndef NU_PACKET_TLV_SET_H_
#define NU_PACKET_TLV_SET_H_

/************************************************************//**
 * @defgroup nu_tlv_set Packet :: TLV set
 * @{
 */

#include "packet/tlv.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/**
 * TLV Set
 */
typedef struct nu_tlv_set {
    nu_tlv_t* next;       ///< pointer to the first element.
    nu_tlv_t* prev;       ///< pointer to the last element.
    size_t    n;          ///< the size of tlv_set.
} nu_tlv_set_t;

////////////////////////////////////////////////////////////////

}//namespace// //ScenSim-Port://

#include "packet/tlv_set_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC nu_bool_t nu_tlv_set_eq(
        const nu_tlv_set_t*, const nu_tlv_set_t*);
PUBLIC void nu_tlv_set_put_log(
        const nu_tlv_set_t*, manet_constant_t*, nu_logger_t*);

/*
 */
static inline nu_tlv_t*
nu_tlv_set_lookup(nu_tlv_set_t* self,
        const nu_tlv_t* tlv, nu_bool_t* eq)
{
    *eq = false;
    FOREACH_TLV_SET(p, self) {
        int c;
        if ((c = nu_tlv_cmp(tlv, p)) <= 0) {
            *eq = (c == 0);
            return p;
        }
    }
    return nu_tlv_set_iter_end(self);
}

/*
 */
static inline nu_tlv_t*
nu_tlv_set_lookup_type(nu_tlv_set_t* self,
        const uint8_t type, nu_bool_t* eq)
{
    *eq = false;
    FOREACH_TLV_SET(p, self) {
        int c;
        if ((c = type - p->type) <= 0) {
            *eq = (c == 0);
            return p;
        }
    }
    return nu_tlv_set_iter_end(self);
}

/** Inserts TLV.
 *
 * @param self
 * @param tlv
 * @return inserted TLV.
 */
PUBLIC_INLINE nu_tlv_t*
nu_tlv_set_add(nu_tlv_set_t* self, nu_tlv_t* tlv)
{
    nu_bool_t eq;
    nu_tlv_t* p = nu_tlv_set_lookup(self, tlv, &eq);
    if (eq) {
        nu_tlv_free(tlv);
        return p;
    }
    nu_tlv_set_iter_insert_before(p, self, tlv);
    return tlv;
}

/** Checks whether TLV set contains the tlv or not.
 *
 * @param self
 * @param tlv
 * @return true if <code>self</code> contains the same tlv.
 */
PUBLIC_INLINE nu_bool_t
nu_tlv_set_contain(nu_tlv_set_t* self, nu_tlv_t* tlv)
{
    nu_bool_t eq;
    nu_tlv_set_lookup(self, tlv, &eq);
    return eq;
}

/** Searches tlv by the type.
 *
 * @param self
 * @param type
 * @return true if contains.
 */
PUBLIC_INLINE nu_bool_t
nu_tlv_set_contain_type(nu_tlv_set_t* self, uint8_t type)
{
    nu_bool_t eq;
    nu_tlv_set_lookup_type(self, type, &eq);
    return eq;
}

/** Searches tlv by the type.
 *
 * @param self
 * @param type
 * @return tlv if exists or NULL.
 */
PUBLIC_INLINE nu_tlv_t*
nu_tlv_set_search_type(nu_tlv_set_t* self, uint8_t type)
{
    nu_bool_t eq;
    nu_tlv_t* p = nu_tlv_set_lookup_type(self, type, &eq);
    if (eq)
        return p;
    else
        return NULL;
}

/** Removes tlv which has same type.
 *
 * @param self
 * @param type
 * @return true	  if a tlv is removed.
 *         false  if no tlv is removed.
 */
PUBLIC_INLINE nu_bool_t
nu_tlv_set_remove_type(nu_tlv_set_t* self, const uint8_t type)
{
    if (nu_tlv_set_size(self) == 0)
        return false;
    nu_bool_t r = false;
    for (nu_tlv_t* p = nu_tlv_set_iter(self);
         !nu_tlv_set_iter_is_end(p, self);) {
        if (type == p->type) {
            p = nu_tlv_set_iter_remove(p, self);
            r = true;
        } else
            p = nu_tlv_set_iter_next(p, self);
    }
    return r;
}

/** Gets tlv.
 *
 * @param self
 * @param index
 * @retval tlv if index < size.
 * @retval NULL if index < 0 or index >= size.
 */
PUBLIC_INLINE nu_tlv_t*
nu_tlv_set_get_at_index(nu_tlv_set_t* self, const unsigned index)
{
    if (index < 0 || index >= self->n)
        return NULL;
    nu_tlv_t* p;
    unsigned  i;
    for (i = 0, p = self->next;
         i < index && p != (nu_tlv_t*)self;
         ++i, p = p->next)
        ;
    return p;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
