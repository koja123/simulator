#ifndef NU_PACKET_ATB_H_
#define NU_PACKET_ATB_H_

#include "core/ip.h"
#include "packet/tlv_set.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_atb Packet :: Address TLV Block
 * @{
 */

/**
 * Element of Address TLV Block
 */
typedef struct nu_atb_elt {
    struct nu_atb_elt* next;    ///< pointer to next element
    struct nu_atb_elt* prev;    ///< pointer to prev element

    nu_ip_t            ip;      ///< IP address
    nu_tlv_set_t       tlv_set; ///< the set of TLVs assigned to IP address
} nu_atb_elt_t;

/**
 * Address TLV Block (list of nu_atb_elt)
 */
typedef struct nu_atb {
    nu_atb_elt_t* next;    ///< pointer to first element
    nu_atb_elt_t* prev;    ///< pointer to last element
    size_t        n;       ///< the number of IP and the set of TLVs pair
} nu_atb_t;

/**
 * Iterator to nu_atb_elt.
 */
typedef nu_atb_elt_t*  nu_atb_iter_t;

PUBLIC void nu_atb_elt_free(nu_atb_elt_t*);

/** Frees the atb_elt. */
#define nu_atb_iter_free    nu_atb_elt_free

}//namespace// //ScenSim-Port://

#include "packet/atb_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

PUBLIC void nu_atb_destroy(nu_atb_t*);

PUBLIC nu_atb_iter_t nu_atb_add_ip(nu_atb_t*, nu_ip_t);
PUBLIC nu_tlv_t* nu_atb_add_ip_tlv(nu_atb_t*,
        const nu_ip_t, nu_tlv_t*);
PUBLIC nu_tlv_t* nu_atb_add_tlv_at(nu_atb_t*,
        const size_t, nu_tlv_t*);

PUBLIC nu_bool_t nu_atb_remove_ip(nu_atb_t*, const nu_ip_t);
PUBLIC nu_bool_t nu_atb_remove_ip_tlv_type(nu_atb_t*,
        const nu_ip_t, const uint8_t tlv_type);

PUBLIC nu_bool_t nu_atb_contain_ip(const nu_atb_t*, const nu_ip_t);

PUBLIC nu_atb_iter_t nu_atb_search_ip(nu_atb_t*, const nu_ip_t);
PUBLIC nu_tlv_t* nu_atb_search_ip_tlv_type(const nu_atb_t*,
        const nu_ip_t, const uint8_t);

PUBLIC nu_bool_t nu_atb_eq(const nu_atb_t*, const nu_atb_t*);

PUBLIC void nu_atb_put_log(const nu_atb_t*, nu_logger_t*);

/**
 * @param p	the iterator which point IP and the set of TLVs pair.
 * @return ip
 */
PUBLIC_INLINE nu_ip_t
nu_atb_iter_ip(const nu_atb_iter_t p)
{
    return p->ip;
}

/**
 * @param p	the iterator which point IP and the set of TLVs pair.
 * @return the set of TLVs
 */
PUBLIC_INLINE nu_tlv_set_t*
nu_atb_iter_tlv_set(const nu_atb_iter_t p)
{
    return &p->tlv_set;
}

/**
 * @param p	the iterator which point IP and the set of TLVs pair.
 * @param tlv
 * @return the set of TLVs
 */
PUBLIC_INLINE nu_tlv_t*
nu_atb_iter_add_tlv(const nu_atb_iter_t p, nu_tlv_t* tlv)
{
    return nu_tlv_set_add(&p->tlv_set, tlv);
}

/**
 * Removes the tlv of target type from the atb.
 *
 * @param p
 * @param type
 */
PUBLIC_INLINE nu_bool_t
nu_atb_iter_remove_tlv_type(const nu_atb_iter_t p, const uint8_t type)
{
    return nu_tlv_set_remove_type(&p->tlv_set, type);
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
