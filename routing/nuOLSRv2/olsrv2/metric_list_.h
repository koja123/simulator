#ifndef METRIC_LIST__H_
#define METRIC_LIST__H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup olsrv2
 * @{
 */

////////////////////////////////////////////////////////////////
//
// metric_t
//

/**
 * Gets the iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE metric_t*
metric_list_iter(const metric_list_t* list)
{
    return (metric_t*)list->next;
}

/**
 * Get the reverse iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE metric_t*
metric_list_iter_r(const metric_list_t* list)
{
    return (metric_t*)list->prev;
}

/**
 * Gets the iterator which points the last(NOT the last element).
 *
 * @param list
 * @return the iterator which points the end of the list.
 */
PUBLIC_INLINE metric_t*
metric_list_iter_end(const metric_list_t* list)
{
    return (metric_t*)list;
}

/**
 * Checks whether the iterator points the last or not.
 *
 * @param iter
 * @param list
 * @return true if p is the iterator which points to the end of the list
 */
PUBLIC_INLINE nu_bool_t
metric_list_iter_is_end(const metric_t* iter,
        const metric_list_t* list)
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
PUBLIC_INLINE metric_t*
metric_list_iter_next(const metric_t* iter, const metric_list_t* list)
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
PUBLIC_INLINE metric_t*
metric_list_iter_next_r(const metric_t* iter, const metric_list_t* list)
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
metric_list_iter_insert_before(metric_t* iter,
        metric_list_t* list, metric_t* elt)
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
metric_list_iter_insert_after(metric_t* iter,
        metric_list_t* list, metric_t* elt)
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
PUBLIC_INLINE metric_t*
metric_list_iter_remove(metric_t* iter, metric_list_t* list)
{
    assert(iter != (metric_t*)list);

    metric_t* next_p = iter->next;
    iter->prev->next = iter->next;
    iter->next->prev = iter->prev;
    --list->n;
    //iter->next = iter->prev = NULL;
    metric_free(iter);
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
PUBLIC_INLINE metric_t*
metric_list_iter_cut(metric_t* iter, metric_list_t* list)
{
    assert(iter != (metric_t*)list);
    metric_t* next_p = iter->next;
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
#define FOREACH_METRIC_LIST(iter, list)           \
    for (metric_t* iter = metric_list_iter(list); \
         !metric_list_iter_is_end(iter, list);    \
         iter = metric_list_iter_next(iter, list))

/**
 * Traverse all the element of the list (reverse order).
 *
 * @param iter
 * @param list
 */
#define FOREACH_METRIC_LIST_R(iter, list)           \
    for (metric_t* iter = metric_list_iter_r(list); \
         !metric_list_iter_is_end(iter, list);      \
         iter = metric_list_iter_next_r(iter, list))

////////////////////////////////////////////////////////////////
//
// metric_list_t
//

/**
 * @param self
 * @return the size of the list
 */
PUBLIC_INLINE size_t
metric_list_size(const metric_list_t* self)
{
    return self->n;
}

/**
 * @param self
 * @retval true  if the list is empty
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
metric_list_is_empty(const metric_list_t* self)
{
    return self->next == (metric_t*)self;
}

/** Removes the top element.
 *
 * @param self
 */
PUBLIC_INLINE void
metric_list_remove_head(metric_list_t* self)
{
    if (!metric_list_is_empty(self))
        (void)metric_list_iter_remove(self->next, self);
}

/** Removes the last element.
 *
 * @param self
 */
PUBLIC_INLINE void
metric_list_remove_tail(metric_list_t* self)
{
    if (!metric_list_is_empty(self))
        (void)metric_list_iter_remove(self->prev, self);
}

/** Remove sthe first element of the list, and return it.
 *
 * @param self
 * @return the removed element
 */
PUBLIC_INLINE metric_t*
metric_list_shift(metric_list_t* self)
{
    if (metric_list_is_empty(self))
        return NULL;
    metric_t* result = self->next;
    metric_list_iter_cut(self->next, self);
    return result;
}

/** Prepends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
metric_list_insert_head(metric_list_t* self, metric_t* elt)
{
    metric_list_iter_insert_after((metric_t*)self, self, elt);
}

/** Appends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
metric_list_insert_tail(metric_list_t* self, metric_t* elt)
{
    metric_list_iter_insert_before((metric_t*)self, self, elt);
}

/** Gets the top element.
 *
 * @param self
 * @return the top element of self
 */
PUBLIC_INLINE metric_t*
metric_list_head(metric_list_t* self)
{
    if (metric_list_is_empty(self))
        return NULL;
    return self->next;
}

/** Removes all the elements
 *
 * @param self
 */
PUBLIC_INLINE void
metric_list_remove_all(metric_list_t* self)
{
    while (!metric_list_is_empty(self))
        (void)metric_list_iter_remove(self->prev, self);
}

/** Initializes list.
 *
 * @param self
 */
PUBLIC_INLINE void
metric_list_init(metric_list_t* self)
{
    self->prev = self->next = (metric_t*)self;
    self->n = 0;
}

/** Destroys the list.
 *
 * @param self
 */
PUBLIC_INLINE void
metric_list_destroy(metric_list_t* self)
{
    metric_list_remove_all(self);
    assert((metric_t*)self == self->next);
    self->next = self->prev = NULL;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
