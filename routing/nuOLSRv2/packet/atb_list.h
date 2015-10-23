#ifndef NU_PACKET_ATB_LIST_H_
#define NU_PACKET_ATB_LIST_H_

#include "packet/atb.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_atb_list Packet :: List of Address TLV Block
 * @{
 */

/**
 * Element of ATB list
 */
typedef struct nu_atb_list_elt {
    struct nu_atb_list_elt* next; ///< pointer to next element
    struct nu_atb_list_elt* prev; ///< pointer to prev element

    nu_atb_t                atb;  ///< address TLV block
} nu_atb_list_elt_t;

/**
 * ATB list
 */
typedef struct nu_atb_list {
    nu_atb_list_elt_t* next;  ///< pointer to first element
    nu_atb_list_elt_t* prev;  ///< pointer to last element
    size_t             n;     ///< element eize
} nu_atb_list_t;

/**
 * Iterator to atb_list_elt.
 */
typedef nu_atb_list_elt_t*  nu_atb_list_iter_t;

////////////////////////////////////////////////////////////////

PUBLIC void nu_atb_list_elt_free(nu_atb_list_iter_t self);

}//namespace// //ScenSim-Port://

#include "packet/atb_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Traverse atb in msg.
 *
 * @param p
 * @param msg
 */
// Override FOREACH_ATB_LIST in atb_list_.h
#undef FOREACH_ATB_LIST
#define FOREACH_ATB_LIST(p, msg)                                  \
    for (nu_atb_list_iter_t p = nu_atb_list_iter(&msg->atb_list); \
         !nu_atb_list_iter_is_end(p, &msg->atb_list);             \
         p = nu_atb_list_iter_next(p, &msg->atb_list))

PUBLIC nu_atb_t* nu_atb_list_iter_atb(nu_atb_list_iter_t);

PUBLIC nu_atb_t* nu_atb_list_add(nu_atb_list_t*);

PUBLIC nu_bool_t nu_atb_list_eq(const nu_atb_list_t*,
        const nu_atb_list_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
