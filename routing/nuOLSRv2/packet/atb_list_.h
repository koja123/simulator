#ifndef NU_ATB_LIST__H_
#define NU_ATB_LIST__H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_atb
 * @{
 */

////////////////////////////////////////////////////////////////
//
// nu_atb_list_elt_t
//

/**
 * Gets the iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE nu_atb_list_elt_t*
nu_atb_list_iter(const nu_atb_list_t* list)
{
    return (nu_atb_list_elt_t*)list->next;
}

/**
 * Get the reverse iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE nu_atb_list_elt_t*
nu_atb_list_iter_r(const nu_atb_list_t* list)
{
    return (nu_atb_list_elt_t*)list->prev;
}

/**
 * Gets the iterator which points the last(NOT the last element).
 *
 * @param list
 * @return the iterator which points the end of the list.
 */
PUBLIC_INLINE nu_atb_list_elt_t*
nu_atb_list_iter_end(const nu_atb_list_t* list)
{
    return (nu_atb_list_elt_t*)list;
}

/**
 * Checks whether the iterator points the last or not.
 *
 * @param iter
 * @param list
 * @return true if p is the iterator which points to the end of the list
 */
PUBLIC_INLINE nu_bool_t
nu_atb_list_iter_is_end(const nu_atb_list_elt_t* iter,
        const nu_atb_list_t* list)
{
    return((void*)iter == (void*)list);
}

/**
 * Gets the next element
 *
 * @param iter
 * @param list
 * @return the iterator which points to the next element
 */
PUBLIC_INLINE nu_atb_list_elt_t*
nu_atb_list_iter_next(const nu_atb_list_elt_t* iter, const nu_atb_list_t* list)
{
    return iter->next;
}

/**
 * Gets the next element.
 *
 * @param iter
 * @param list
 * @return the iterator which points to the next element
 */
PUBLIC_INLINE nu_atb_list_elt_t*
nu_atb_list_iter_next_r(const nu_atb_list_elt_t* iter, const nu_atb_list_t* list)
{
    return iter->prev;
}

/**
 * Inserts the element before the iterator.
 *
 * @param iter
 * @param list
 * @param elt
 */
PUBLIC_INLINE void
nu_atb_list_iter_insert_before(nu_atb_list_elt_t* iter,
        nu_atb_list_t* list, nu_atb_list_elt_t* elt)
{
    assert(elt);

    elt->prev = iter->prev;
    elt->next = iter;
    iter->prev->next = elt;
    iter->prev = elt;

    ++list->n;
}

/**
 * Inserts the element after the iterator.
 *
 * @param iter
 * @param list
 * @param elt
 */
PUBLIC_INLINE void
nu_atb_list_iter_insert_after(nu_atb_list_elt_t* iter,
        nu_atb_list_t* list, nu_atb_list_elt_t* elt)
{
    assert(elt);

    elt->prev = iter;
    elt->next = iter->next;
    iter->next->prev = elt;
    iter->next = elt;

    ++list->n;
}

/**
 * Removes the element which is pointed by the iterator.
 *
 * @param iter
 * @param list
 * @return the iterator which points next element
 */
PUBLIC_INLINE nu_atb_list_elt_t*
nu_atb_list_iter_remove(nu_atb_list_elt_t* iter, nu_atb_list_t* list)
{
    assert(iter != (nu_atb_list_elt_t*)list);

    nu_atb_list_elt_t* next_p = iter->next;
    iter->prev->next = iter->next;
    iter->next->prev = iter->prev;
    --list->n;
    //iter->next = iter->prev = NULL;
    nu_atb_list_elt_free(iter);
    return next_p;
}

/**
 * Removes the element which is pointed by the iter.
 *
 * The memory of removed element is NOT freed.
 *
 * @param iter
 * @param list
 * @return the iterator which points next element
 */
PUBLIC_INLINE nu_atb_list_elt_t*
nu_atb_list_iter_cut(nu_atb_list_elt_t* iter, nu_atb_list_t* list)
{
    assert(iter != (nu_atb_list_elt_t*)list);
    nu_atb_list_elt_t* next_p = iter->next;
    iter->prev->next = iter->next;
    iter->next->prev = iter->prev;
    --list->n;
    return next_p;
}

/** Traverses all the element of the list.
 *
 * @param iter
 * @param list
 */
#define FOREACH_ATB_LIST(iter, list)                       \
    for (nu_atb_list_elt_t* iter = nu_atb_list_iter(list); \
         !nu_atb_list_iter_is_end(iter, list);             \
         iter = nu_atb_list_iter_next(iter, list))

/**
 * Traverse all the element of the list (reverse order).
 *
 * @param iter
 * @param list
 */
#define FOREACH_ATB_LIST_R(iter, list)                       \
    for (nu_atb_list_elt_t* iter = nu_atb_list_iter_r(list); \
         !nu_atb_list_iter_is_end(iter, list);               \
         iter = nu_atb_list_iter_next_r(iter, list))

////////////////////////////////////////////////////////////////
//
// nu_atb_list_t
//

/**
 * @param self
 * @return the size of the list
 */
PUBLIC_INLINE size_t
nu_atb_list_size(const nu_atb_list_t* self)
{
    return self->n;
}

/**
 * @param self
 * @retval true  if the list is empty
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_atb_list_is_empty(const nu_atb_list_t* self)
{
    return self->next == (nu_atb_list_elt_t*)self;
}

/** Removes the top element.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_atb_list_remove_head(nu_atb_list_t* self)
{
    if (!nu_atb_list_is_empty(self))
        (void)nu_atb_list_iter_remove(self->next, self);
}

/** Removes the last element.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_atb_list_remove_tail(nu_atb_list_t* self)
{
    if (!nu_atb_list_is_empty(self))
        (void)nu_atb_list_iter_remove(self->prev, self);
}

/** Remove sthe first element of the list, and return it.
 *
 * @param self
 * @return the removed element
 */
PUBLIC_INLINE nu_atb_list_elt_t*
nu_atb_list_shift(nu_atb_list_t* self)
{
    if (nu_atb_list_is_empty(self))
        return NULL;
    nu_atb_list_elt_t* result = self->next;
    nu_atb_list_iter_cut(self->next, self);
    return result;
}

/** Prepends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
nu_atb_list_insert_head(nu_atb_list_t* self, nu_atb_list_elt_t* elt)
{
    nu_atb_list_iter_insert_after((nu_atb_list_elt_t*)self, self, elt);
}

/** Appends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
nu_atb_list_insert_tail(nu_atb_list_t* self, nu_atb_list_elt_t* elt)
{
    nu_atb_list_iter_insert_before((nu_atb_list_elt_t*)self, self, elt);
}

/** Gets the top element.
 *
 * @param self
 * @return the top element of self
 */
PUBLIC_INLINE nu_atb_list_elt_t*
nu_atb_list_head(nu_atb_list_t* self)
{
    if (nu_atb_list_is_empty(self))
        return NULL;
    return self->next;
}

/** Removes all the elements
 *
 * @param self
 */
PUBLIC_INLINE void
nu_atb_list_remove_all(nu_atb_list_t* self)
{
    while (!nu_atb_list_is_empty(self))
        (void)nu_atb_list_iter_remove(self->prev, self);
}

/** Initializes list.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_atb_list_init(nu_atb_list_t* self)
{
    self->prev = self->next = (nu_atb_list_elt_t*)self;
    self->n = 0;
}

/** Destroys the list.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_atb_list_destroy(nu_atb_list_t* self)
{
    nu_atb_list_remove_all(self);
    assert((nu_atb_list_elt_t*)self == self->next);
    self->next = self->prev = NULL;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
