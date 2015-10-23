#ifndef TC_IP_LIST__H_
#define TC_IP_LIST__H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup olsrv2
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tc_ip_t
//

/**
 * Gets the iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE tc_ip_t*
tc_ip_list_iter(const tc_ip_list_t* list)
{
    return (tc_ip_t*)list->next;
}

/**
 * Get the reverse iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE tc_ip_t*
tc_ip_list_iter_r(const tc_ip_list_t* list)
{
    return (tc_ip_t*)list->prev;
}

/**
 * Gets the iterator which points the last(NOT the last element).
 *
 * @param list
 * @return the iterator which points the end of the list.
 */
PUBLIC_INLINE tc_ip_t*
tc_ip_list_iter_end(const tc_ip_list_t* list)
{
    return (tc_ip_t*)list;
}

/**
 * Checks whether the iterator points the last or not.
 *
 * @param iter
 * @param list
 * @return true if p is the iterator which points to the end of the list
 */
PUBLIC_INLINE nu_bool_t
tc_ip_list_iter_is_end(const tc_ip_t* iter,
        const tc_ip_list_t* list)
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
PUBLIC_INLINE tc_ip_t*
tc_ip_list_iter_next(const tc_ip_t* iter, const tc_ip_list_t* list)
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
PUBLIC_INLINE tc_ip_t*
tc_ip_list_iter_next_r(const tc_ip_t* iter, const tc_ip_list_t* list)
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
tc_ip_list_iter_insert_before(tc_ip_t* iter,
        tc_ip_list_t* list, tc_ip_t* elt)
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
tc_ip_list_iter_insert_after(tc_ip_t* iter,
        tc_ip_list_t* list, tc_ip_t* elt)
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
PUBLIC_INLINE tc_ip_t*
tc_ip_list_iter_remove(tc_ip_t* iter, tc_ip_list_t* list)
{
    assert(iter != (tc_ip_t*)list);

    tc_ip_t* next_p = iter->next;
    iter->prev->next = iter->next;
    iter->next->prev = iter->prev;
    --list->n;
    //iter->next = iter->prev = NULL;
    tc_ip_free(iter);
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
PUBLIC_INLINE tc_ip_t*
tc_ip_list_iter_cut(tc_ip_t* iter, tc_ip_list_t* list)
{
    assert(iter != (tc_ip_t*)list);
    tc_ip_t* next_p = iter->next;
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
#define FOREACH_TC_IP_LIST(iter, list)          \
    for (tc_ip_t* iter = tc_ip_list_iter(list); \
         !tc_ip_list_iter_is_end(iter, list);   \
         iter = tc_ip_list_iter_next(iter, list))

/**
 * Traverse all the element of the list (reverse order).
 *
 * @param iter
 * @param list
 */
#define FOREACH_TC_IP_LIST_R(iter, list)          \
    for (tc_ip_t* iter = tc_ip_list_iter_r(list); \
         !tc_ip_list_iter_is_end(iter, list);     \
         iter = tc_ip_list_iter_next_r(iter, list))

////////////////////////////////////////////////////////////////
//
// tc_ip_list_t
//

/**
 * @param self
 * @return the size of the list
 */
PUBLIC_INLINE size_t
tc_ip_list_size(const tc_ip_list_t* self)
{
    return self->n;
}

/**
 * @param self
 * @retval true  if the list is empty
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
tc_ip_list_is_empty(const tc_ip_list_t* self)
{
    return self->next == (tc_ip_t*)self;
}

/** Removes the top element.
 *
 * @param self
 */
PUBLIC_INLINE void
tc_ip_list_remove_head(tc_ip_list_t* self)
{
    if (!tc_ip_list_is_empty(self))
        (void)tc_ip_list_iter_remove(self->next, self);
}

/** Removes the last element.
 *
 * @param self
 */
PUBLIC_INLINE void
tc_ip_list_remove_tail(tc_ip_list_t* self)
{
    if (!tc_ip_list_is_empty(self))
        (void)tc_ip_list_iter_remove(self->prev, self);
}

/** Remove sthe first element of the list, and return it.
 *
 * @param self
 * @return the removed element
 */
PUBLIC_INLINE tc_ip_t*
tc_ip_list_shift(tc_ip_list_t* self)
{
    if (tc_ip_list_is_empty(self))
        return NULL;
    tc_ip_t* result = self->next;
    tc_ip_list_iter_cut(self->next, self);
    return result;
}

/** Prepends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
tc_ip_list_insert_head(tc_ip_list_t* self, tc_ip_t* elt)
{
    tc_ip_list_iter_insert_after((tc_ip_t*)self, self, elt);
}

/** Appends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
tc_ip_list_insert_tail(tc_ip_list_t* self, tc_ip_t* elt)
{
    tc_ip_list_iter_insert_before((tc_ip_t*)self, self, elt);
}

/** Gets the top element.
 *
 * @param self
 * @return the top element of self
 */
PUBLIC_INLINE tc_ip_t*
tc_ip_list_head(tc_ip_list_t* self)
{
    if (tc_ip_list_is_empty(self))
        return NULL;
    return self->next;
}

/** Removes all the elements
 *
 * @param self
 */
PUBLIC_INLINE void
tc_ip_list_remove_all(tc_ip_list_t* self)
{
    while (!tc_ip_list_is_empty(self))
        (void)tc_ip_list_iter_remove(self->prev, self);
}

/** Initializes list.
 *
 * @param self
 */
PUBLIC_INLINE void
tc_ip_list_init(tc_ip_list_t* self)
{
    self->prev = self->next = (tc_ip_t*)self;
    self->n = 0;
}

/** Destroys the list.
 *
 * @param self
 */
PUBLIC_INLINE void
tc_ip_list_destroy(tc_ip_list_t* self)
{
    tc_ip_list_remove_all(self);
    assert((tc_ip_t*)self == self->next);
    self->next = self->prev = NULL;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
