//
// (N) Neighbor Set
//
#include "config.h"

#include "core/core.h"
#include "packet/packet.h"
#include "olsrv2/olsrv2.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_n
 */

////////////////////////////////////////////////////////////////
//
// tuple_n
//

/*
 */
static inline tuple_n_t*
tuple_n_create(const nu_ip_set_t* neighbor_ip_list, const nu_ip_t orig_addr)
{
    tuple_n_t* tuple = nu_mem_alloc(tuple_n_t);
    tuple->next = tuple->prev = NULL;
    nu_ip_set_init(&tuple->neighbor_ip_list);
    nu_ip_set_copy(&tuple->neighbor_ip_list, neighbor_ip_list);
    tuple->symmetric = false;
    tuple->orig_addr = orig_addr;
    tuple->willingness  = WILLINGNESS__NEVER;
    tuple->flooding_mpr = false;
    tuple->routing_mpr  = false;
    tuple->mpr_selector = false;
    tuple->advertised = false;
    tuple->_in_metric = tuple->_out_metric = UNDEF_METRIC;
    return tuple;
}

/*
 */
static inline void
tuple_n_free(tuple_n_t* tuple_n)
{
    nu_ip_set_destroy(&tuple_n->neighbor_ip_list);
    nu_mem_free(tuple_n);
}

/** Checks whether the neighbor has symlink.
 *
 * @param tuple_n
 * @return true if the neighbor has symlink.
 */
PUBLIC nu_bool_t
tuple_n_has_symlink(const tuple_n_t* tuple_n)
{
    FOREACH_I(tuple_i) {
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            if (nu_ip_set_contain_at_least_one(
                        &tuple_n->neighbor_ip_list,
                        &tuple_l->neighbor_ip_list) &&
                tuple_l->status == LINK_STATUS__SYMMETRIC)
                return true;
        }
    }
    return false;
}

/** Checks whether the neighbor has at least one link.
 *
 * @param tuple_n
 * @return true if the neighbor has at least one link.
 */
PUBLIC nu_bool_t
tuple_n_has_link(const tuple_n_t* tuple_n)
{
    FOREACH_I(tuple_i) {
        FOREACH_L(tuple_l, &tuple_i->ibase_l) {
            if (nu_ip_set_contain_at_least_one(
                        &tuple_n->neighbor_ip_list,
                        &tuple_l->neighbor_ip_list))
                return true;
        }
    }
    return false;
}

/** Adds the ip to the N_neighbor_ifaddr_list.
 *
 * @param tuple_n
 * @param ip
 */
PUBLIC void
tuple_n_add_neighbor_ip_list(tuple_n_t* tuple_n, nu_ip_t ip)
{
    if (nu_ip_set_add(&tuple_n->neighbor_ip_list, ip)) {
        IBASE_N->change = true;
        if (tuple_n->symmetric) {
            IBASE_N->sym_change = true;
            //IBASE_N->ansn_change = true;
        }
    }
}

/** Sets the N_orig_addr.
 *
 * @param tuple_n
 * @param orig_addr
 */
PUBLIC void
tuple_n_set_orig_addr(tuple_n_t* tuple_n, nu_ip_t orig_addr)
{
    if (!nu_orig_addr_eq(tuple_n->orig_addr, orig_addr)) {
        nu_orig_addr_copy(&tuple_n->orig_addr, orig_addr);
        if (tuple_n->symmetric)
            IBASE_N->sym_change = true;
    }
}

/** Sets the N_symmetric.
 *
 * @param tuple_n
 * @param sym
 */
PUBLIC void
tuple_n_set_symmetric(tuple_n_t* tuple_n, nu_bool_t sym)
{
    if (tuple_n->symmetric != sym) {
        tuple_n->symmetric = sym;
        IBASE_N->change = true;
        IBASE_N->sym_change = true;
        if (!tuple_n->symmetric) {
            tuple_n_set_mpr_selector(tuple_n, false);
        }
    }
}

/** Sets the N_willingness.
 *
 * @param tuple_n
 * @param wil
 */
PUBLIC void
tuple_n_set_willingness(tuple_n_t* tuple_n, nu_bool_t wil)
{
    if (tuple_n->willingness != wil) {
        tuple_n->willingness = wil;
        if (tuple_n->symmetric)
            IBASE_N->sym_change = true;
    }
}

/** Sets the N_fooding_mpr.
 *
 * @param tuple_n
 * @param mpr
 * @see ibase_n_save_mpr(), ibase_n_commit_mpr()
 */
PUBLIC void
tuple_n_set_flooding_mpr(tuple_n_t* tuple_n, nu_bool_t mpr)
{
    if (tuple_n->flooding_mpr != mpr) {
        tuple_n->flooding_mpr = mpr;
        IBASE_N->change = true;
#if 0
        if (tuple_n->symmetric)
            IBASE_N->sym_change = true;
#endif
    }
}

/** Sets the N_routing_mpr.
 *
 * @param tuple_n
 * @param mpr
 * @see ibase_n_save_mpr(), ibase_n_commit_mpr()
 */
PUBLIC void
tuple_n_set_routing_mpr(tuple_n_t* tuple_n, nu_bool_t mpr)
{
    if (tuple_n->routing_mpr != mpr) {
        tuple_n->routing_mpr = mpr;
        IBASE_N->change = true;
#if 0
        if (tuple_n->symmetric)
            IBASE_N->sym_change = true;
#endif
    }
}

/** Sets the N_mpr_selector.
 *
 * @param tuple_n
 * @param mprs
 */
PUBLIC void
tuple_n_set_mpr_selector(tuple_n_t* tuple_n, nu_bool_t mprs)
{
    if (tuple_n->mpr_selector != mprs) {
        tuple_n->mpr_selector = mprs;
        IBASE_N->change = true;
    }
}

/** Sets the N_advertised.
 *
 * @param tuple_n
 * @param advertised
 */
PUBLIC void
tuple_n_set_advertised(tuple_n_t* tuple_n, nu_bool_t advertised)
{
    if (tuple_n->advertised != advertised) {
        tuple_n->advertised = advertised;
        IBASE_N->change = true;
    }
}

////////////////////////////////////////////////////////////////
//
// ibase_n_t
//

}//namespace// //ScenSim-Port://

#include "olsrv2/ibase_n_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/** Initializes the ibase_n.
 */
PUBLIC void
ibase_n_init(void)
{
    _ibase_n_init(IBASE_N);
    IBASE_N->ansn = (uint16_t)(nu_rand() * 0x10000);
}

/** Destroys the ibase_n.
 */
PUBLIC void
ibase_n_destroy(void)
{
    _ibase_n_destroy(IBASE_N);
}

/** Adds new tuple.
 *
 * @param neighbor_ip_list
 * @param orig_addr
 * @return the new tuple
 */
PUBLIC tuple_n_t*
ibase_n_add(const nu_ip_set_t* neighbor_ip_list, const nu_ip_t orig_addr)
{
    tuple_n_t* new_tuple = tuple_n_create(neighbor_ip_list, orig_addr);
    _ibase_n_insert_tail(IBASE_N, new_tuple);
    assert(!new_tuple->symmetric);
    IBASE_N->change = true;
    return new_tuple;
}

/** Gets the related tuple_n of the tuple_l.
 *
 * @param tuple_l
 * @return the related tuple_n or NULL.
 */
PUBLIC tuple_n_t*
ibase_n_search_tuple_l(tuple_l_t* tuple_l)
{
    if (tuple_l->tuple_n != NULL)
        return tuple_l->tuple_n;

    FOREACH_N(tuple_n) {
        if (nu_ip_set_contain_at_least_one(
                    &tuple_n->neighbor_ip_list, &tuple_l->neighbor_ip_list))
            return(tuple_l->tuple_n = tuple_n);
    }
    return NULL;
}

/** Checks whether the ibase_n contains the symmetric link between the ip.
 *
 * @param ip
 * @return true if the ibae_n contains the symmetric link.
 */
PUBLIC nu_bool_t
ibase_n_contain_symmetric(const nu_ip_t ip)
{
    FOREACH_N(tuple_n) {
        if (nu_ip_set_contain(&tuple_n->neighbor_ip_list, ip))
            return tuple_n->symmetric;
    }
    return false;
}

/** Checks whether the ibase_n contains the symmetric link between the ip.
 *
 * @param ip
 * @return true if the ibae_n contains the symmetric link.
 */
PUBLIC nu_bool_t
ibase_n_contain_symmetric_without_prefix(const nu_ip_t ip)
{
    FOREACH_N(tuple_n) {
        if (nu_ip_set_contain_without_prefix(&tuple_n->neighbor_ip_list, ip))
            return tuple_n->symmetric;
    }
    return false;
}

/** Removes the tuple.
 *
 * @param tuple_n
 * @return the next tuple
 */
PUBLIC tuple_n_t*
ibase_n_iter_remove(tuple_n_t* tuple_n)
{
    assert(!tuple_n_has_link(tuple_n));
    IBASE_N->change = true;
    if (tuple_n->symmetric)
        IBASE_N->sym_change = true;
    return _ibase_n_iter_remove(tuple_n, IBASE_N);
}

/** Saves current advertised status.
 */
PUBLIC void
ibase_n_save_advertised(void)
{
    FOREACH_N(tuple_n) {
        tuple_n->advertised_save = tuple_n->advertised;
    }
}

/** Saves current flooding mpr status.
 */
PUBLIC void
ibase_n_save_flooding_mpr(void)
{
    FOREACH_N(tuple_n) {
        tuple_n->flooding_mpr_save = tuple_n->flooding_mpr;
        tuple_n->flooding_mpr = false;
    }
}

/** Saves current routing mpr status.
 */
PUBLIC void
ibase_n_save_routing_mpr(void)
{
    FOREACH_N(tuple_n) {
        tuple_n->routing_mpr_save = tuple_n->routing_mpr;
        tuple_n->routing_mpr = false;
    }
}

/** Commits advertised status
 *
 * @retval true if advertised status is changed.
 */
PUBLIC nu_bool_t
ibase_n_commit_advertised(void)
{
    FOREACH_N(tuple_n) {
        if (tuple_n->advertised != tuple_n->advertised_save)
            return true;
    }
    return false;
}

/** Commits flooding mpr status.
 */
PUBLIC void
ibase_n_commit_flooding_mpr(void)
{
    FOREACH_N(tuple_n) {
        if (tuple_n->flooding_mpr != tuple_n->flooding_mpr_save)
            IBASE_N->change = true;
    }
}

/** Commits routing mpr status.
 */
PUBLIC void
ibase_n_commit_routing_mpr(void)
{
    FOREACH_N(tuple_n) {
        if (tuple_n->routing_mpr != tuple_n->routing_mpr_save)
            IBASE_N->change = true;
    }
}

/** Outputs tuple_n.
 *
 * @param tuple_n
 * @param logger
 */
PUBLIC void
tuple_n_put_log(tuple_n_t* tuple_n, nu_logger_t* logger)
{
    nu_logger_log(logger,
            "I_N:[sym:%B will:%d fmpr:%B rmpr:%B mprs:%B adv:%B orig:%I in:%g out:%g neighb:(%S)]",
            tuple_n->symmetric, tuple_n->willingness,
            tuple_n->flooding_mpr, tuple_n->routing_mpr, tuple_n->mpr_selector, tuple_n->advertised,
            tuple_n->orig_addr, tuple_n->_in_metric, tuple_n->_out_metric,
            &tuple_n->neighbor_ip_list);
}

/** Outputs ibase_n.
 *
 * @param logger
 */
PUBLIC void
ibase_n_put_log(nu_logger_t* logger)
{
    nu_logger_log(logger, "I_N:%d:--", ibase_n_size());
    FOREACH_N(p) {
        tuple_n_put_log(p, logger);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
