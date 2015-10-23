#ifndef NU_EVENT_LIST__H_
#define NU_EVENT_LIST__H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_ip_container
 * @{
 */

////////////////////////////////////////////////////////////////
//
// nu_event_t
//

/**
 * Gets the iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE nu_event_t*
nu_event_list_iter(const nu_event_list_t* list)
{
    return (nu_event_t*)list->next;
}

/**
 * Get the reverse iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE nu_event_t*
nu_event_list_iter_r(const nu_event_list_t* list)
{
    return (nu_event_t*)list->prev;
}

/**
 * Gets the iterator which points the last(NOT the last element).
 *
 * @param list
 * @return the iterator which points the end of the list.
 */
PUBLIC_INLINE nu_event_t*
nu_event_list_iter_end(const nu_event_list_t* list)
{
    return (nu_event_t*)list;
}

/**
 * Checks whether the iterator points the last or not.
 *
 * @param iter
 * @param list
 * @return true if p is the iterator which points to the end of the list
 */
PUBLIC_INLINE nu_bool_t
nu_event_list_iter_is_end(const nu_event_t* iter,
        const nu_event_list_t* list)
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
PUBLIC_INLINE nu_event_t*
nu_event_list_iter_next(const nu_event_t* iter, const nu_event_list_t* list)
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
PUBLIC_INLINE nu_event_t*
nu_event_list_iter_next_r(const nu_event_t* iter, const nu_event_list_t* list)
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
nu_event_list_iter_insert_before(nu_event_t* iter,
        nu_event_list_t* list, nu_event_t* elt)
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
nu_event_list_iter_insert_after(nu_event_t* iter,
        nu_event_list_t* list, nu_event_t* elt)
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
PUBLIC_INLINE nu_event_t*
nu_event_list_iter_remove(nu_event_t* iter, nu_event_list_t* list)
{
    assert(iter != (nu_event_t*)list);

    nu_event_t* next_p = iter->next;
    iter->prev->next = iter->next;
    iter->next->prev = iter->prev;
    --list->n;
    //iter->next = iter->prev = NULL;
    nu_event_free(iter);
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
PUBLIC_INLINE nu_event_t*
nu_event_list_iter_cut(nu_event_t* iter, nu_event_list_t* list)
{
    assert(iter != (nu_event_t*)list);
    nu_event_t* next_p = iter->next;
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
#define FOREACH_EVENT_LIST(iter, list)                \
    for (nu_event_t* iter = nu_event_list_iter(list); \
         !nu_event_list_iter_is_end(iter, list);      \
         iter = nu_event_list_iter_next(iter, list))

/**
 * Traverse all the element of the list (reverse order).
 *
 * @param iter
 * @param list
 */
#define FOREACH_EVENT_LIST_R(iter, list)                \
    for (nu_event_t* iter = nu_event_list_iter_r(list); \
         !nu_event_list_iter_is_end(iter, list);        \
         iter = nu_event_list_iter_next_r(iter, list))

////////////////////////////////////////////////////////////////
//
// nu_event_list_t
//

/**
 * @param self
 * @return the size of the list
 */
PUBLIC_INLINE size_t
nu_event_list_size(const nu_event_list_t* self)
{
    return self->n;
}

/**
 * @param self
 * @retval true  if the list is empty
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_event_list_is_empty(const nu_event_list_t* self)
{
    return self->next == (nu_event_t*)self;
}

/** Removes the top element.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_event_list_remove_head(nu_event_list_t* self)
{
    if (!nu_event_list_is_empty(self))
        (void)nu_event_list_iter_remove(self->next, self);
}

/** Removes the last element.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_event_list_remove_tail(nu_event_list_t* self)
{
    if (!nu_event_list_is_empty(self))
        (void)nu_event_list_iter_remove(self->prev, self);
}

/** Remove sthe first element of the list, and return it.
 *
 * @param self
 * @return the removed element
 */
PUBLIC_INLINE nu_event_t*
nu_event_list_shift(nu_event_list_t* self)
{
    if (nu_event_list_is_empty(self))
        return NULL;
    nu_event_t* result = self->next;
    nu_event_list_iter_cut(self->next, self);
    return result;
}

/** Prepends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
nu_event_list_insert_head(nu_event_list_t* self, nu_event_t* elt)
{
    nu_event_list_iter_insert_after((nu_event_t*)self, self, elt);
}

/** Appends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
nu_event_list_insert_tail(nu_event_list_t* self, nu_event_t* elt)
{
    nu_event_list_iter_insert_before((nu_event_t*)self, self, elt);
}

/** Gets the top element.
 *
 * @param self
 * @return the top element of self
 */
PUBLIC_INLINE nu_event_t*
nu_event_list_head(nu_event_list_t* self)
{
    if (nu_event_list_is_empty(self))
        return NULL;
    return self->next;
}

/** Removes all the elements
 *
 * @param self
 */
PUBLIC_INLINE void
nu_event_list_remove_all(nu_event_list_t* self)
{
    while (!nu_event_list_is_empty(self))
        (void)nu_event_list_iter_remove(self->prev, self);
}

/** Initializes list.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_event_list_init(nu_event_list_t* self)
{
    self->prev = self->next = (nu_event_t*)self;
    self->n = 0;
}

/** Destroys the list.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_event_list_destroy(nu_event_list_t* self)
{
    nu_event_list_remove_all(self);
    assert((nu_event_t*)self == self->next);
    self->next = self->prev = NULL;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
