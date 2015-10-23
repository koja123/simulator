#ifndef TUPLE_PROC_LIST__H_
#define TUPLE_PROC_LIST__H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup olsrv2
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tuple_proc_t
//

/**
 * Gets the iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE tuple_proc_t*
tuple_proc_list_iter(const tuple_proc_list_t* list)
{
    return (tuple_proc_t*)list->next;
}

/**
 * Get the reverse iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE tuple_proc_t*
tuple_proc_list_iter_r(const tuple_proc_list_t* list)
{
    return (tuple_proc_t*)list->prev;
}

/**
 * Gets the iterator which points the last(NOT the last element).
 *
 * @param list
 * @return the iterator which points the end of the list.
 */
PUBLIC_INLINE tuple_proc_t*
tuple_proc_list_iter_end(const tuple_proc_list_t* list)
{
    return (tuple_proc_t*)list;
}

/**
 * Checks whether the iterator points the last or not.
 *
 * @param iter
 * @param list
 * @return true if p is the iterator which points to the end of the list
 */
PUBLIC_INLINE nu_bool_t
tuple_proc_list_iter_is_end(const tuple_proc_t* iter,
        const tuple_proc_list_t* list)
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
PUBLIC_INLINE tuple_proc_t*
tuple_proc_list_iter_next(const tuple_proc_t* iter, const tuple_proc_list_t* list)
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
PUBLIC_INLINE tuple_proc_t*
tuple_proc_list_iter_next_r(const tuple_proc_t* iter, const tuple_proc_list_t* list)
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
tuple_proc_list_iter_insert_before(tuple_proc_t* iter,
        tuple_proc_list_t* list, tuple_proc_t* elt)
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
tuple_proc_list_iter_insert_after(tuple_proc_t* iter,
        tuple_proc_list_t* list, tuple_proc_t* elt)
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
PUBLIC_INLINE tuple_proc_t*
tuple_proc_list_iter_remove(tuple_proc_t* iter, tuple_proc_list_t* list)
{
    assert(iter != (tuple_proc_t*)list);

    tuple_proc_t* next_p = iter->next;
    iter->prev->next = iter->next;
    iter->next->prev = iter->prev;
    --list->n;
    //iter->next = iter->prev = NULL;
    tuple_proc_free(iter);
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
PUBLIC_INLINE tuple_proc_t*
tuple_proc_list_iter_cut(tuple_proc_t* iter, tuple_proc_list_t* list)
{
    assert(iter != (tuple_proc_t*)list);
    tuple_proc_t* next_p = iter->next;
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
#define FOREACH_TUPLE_PROC_LIST(iter, list)               \
    for (tuple_proc_t* iter = tuple_proc_list_iter(list); \
         !tuple_proc_list_iter_is_end(iter, list);        \
         iter = tuple_proc_list_iter_next(iter, list))

/**
 * Traverse all the element of the list (reverse order).
 *
 * @param iter
 * @param list
 */
#define FOREACH_TUPLE_PROC_LIST_R(iter, list)               \
    for (tuple_proc_t* iter = tuple_proc_list_iter_r(list); \
         !tuple_proc_list_iter_is_end(iter, list);          \
         iter = tuple_proc_list_iter_next_r(iter, list))

////////////////////////////////////////////////////////////////
//
// tuple_proc_list_t
//

/**
 * @param self
 * @return the size of the list
 */
PUBLIC_INLINE size_t
tuple_proc_list_size(const tuple_proc_list_t* self)
{
    return self->n;
}

/**
 * @param self
 * @retval true  if the list is empty
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
tuple_proc_list_is_empty(const tuple_proc_list_t* self)
{
    return self->next == (tuple_proc_t*)self;
}

/** Removes the top element.
 *
 * @param self
 */
PUBLIC_INLINE void
tuple_proc_list_remove_head(tuple_proc_list_t* self)
{
    if (!tuple_proc_list_is_empty(self))
        (void)tuple_proc_list_iter_remove(self->next, self);
}

/** Removes the last element.
 *
 * @param self
 */
PUBLIC_INLINE void
tuple_proc_list_remove_tail(tuple_proc_list_t* self)
{
    if (!tuple_proc_list_is_empty(self))
        (void)tuple_proc_list_iter_remove(self->prev, self);
}

/** Remove sthe first element of the list, and return it.
 *
 * @param self
 * @return the removed element
 */
PUBLIC_INLINE tuple_proc_t*
tuple_proc_list_shift(tuple_proc_list_t* self)
{
    if (tuple_proc_list_is_empty(self))
        return NULL;
    tuple_proc_t* result = self->next;
    tuple_proc_list_iter_cut(self->next, self);
    return result;
}

/** Prepends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
tuple_proc_list_insert_head(tuple_proc_list_t* self, tuple_proc_t* elt)
{
    tuple_proc_list_iter_insert_after((tuple_proc_t*)self, self, elt);
}

/** Appends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
tuple_proc_list_insert_tail(tuple_proc_list_t* self, tuple_proc_t* elt)
{
    tuple_proc_list_iter_insert_before((tuple_proc_t*)self, self, elt);
}

/** Gets the top element.
 *
 * @param self
 * @return the top element of self
 */
PUBLIC_INLINE tuple_proc_t*
tuple_proc_list_head(tuple_proc_list_t* self)
{
    if (tuple_proc_list_is_empty(self))
        return NULL;
    return self->next;
}

/** Removes all the elements
 *
 * @param self
 */
PUBLIC_INLINE void
tuple_proc_list_remove_all(tuple_proc_list_t* self)
{
    while (!tuple_proc_list_is_empty(self))
        (void)tuple_proc_list_iter_remove(self->prev, self);
}

/** Initializes list.
 *
 * @param self
 */
PUBLIC_INLINE void
tuple_proc_list_init(tuple_proc_list_t* self)
{
    self->prev = self->next = (tuple_proc_t*)self;
    self->n = 0;
}

/** Destroys the list.
 *
 * @param self
 */
PUBLIC_INLINE void
tuple_proc_list_destroy(tuple_proc_list_t* self)
{
    tuple_proc_list_remove_all(self);
    assert((tuple_proc_t*)self == self->next);
    self->next = self->prev = NULL;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
