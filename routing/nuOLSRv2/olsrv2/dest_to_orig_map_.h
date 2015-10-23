#ifndef DEST_TO_ORIG_MAP__H_
#define DEST_TO_ORIG_MAP__H_

/************************************************************//**
 * @addtogroup olsrv2
 * @{
 */

////////////////////////////////////////////////////////////////
//
// dest_to_orig_t
//

/**
 * Gets the iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE dest_to_orig_t*
dest_to_orig_map_iter(const dest_to_orig_map_t* list)
{
    return (dest_to_orig_t*)list->next;
}

/**
 * Get the reverse iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE dest_to_orig_t*
dest_to_orig_map_iter_r(const dest_to_orig_map_t* list)
{
    return (dest_to_orig_t*)list->prev;
}

/**
 * Gets the iterator which points the last(NOT the last element).
 *
 * @param list
 * @return the iterator which points the end of the list.
 */
PUBLIC_INLINE dest_to_orig_t*
dest_to_orig_map_iter_end(const dest_to_orig_map_t* list)
{
    return (dest_to_orig_t*)list;
}

/**
 * Checks whether the iterator points the last or not.
 *
 * @param iter
 * @param list
 * @return true if p is the iterator which points to the end of the list
 */
PUBLIC_INLINE nu_bool_t
dest_to_orig_map_iter_is_end(const dest_to_orig_t* iter,
        const dest_to_orig_map_t* list)
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
PUBLIC_INLINE dest_to_orig_t*
dest_to_orig_map_iter_next(const dest_to_orig_t* iter, const dest_to_orig_map_t* list)
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
PUBLIC_INLINE dest_to_orig_t*
dest_to_orig_map_iter_next_r(const dest_to_orig_t* iter, const dest_to_orig_map_t* list)
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
dest_to_orig_map_iter_insert_before(dest_to_orig_t* iter,
        dest_to_orig_map_t* list, dest_to_orig_t* elt)
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
dest_to_orig_map_iter_insert_after(dest_to_orig_t* iter,
        dest_to_orig_map_t* list, dest_to_orig_t* elt)
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
PUBLIC_INLINE dest_to_orig_t*
dest_to_orig_map_iter_remove(dest_to_orig_t* iter, dest_to_orig_map_t* list)
{
    assert(iter != (dest_to_orig_t*)list);

    dest_to_orig_t* next_p = iter->next;
    iter->prev->next = iter->next;
    iter->next->prev = iter->prev;
    --list->n;
    //iter->next = iter->prev = NULL;
    dest_to_orig_free(iter);
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
PUBLIC_INLINE dest_to_orig_t*
dest_to_orig_map_iter_cut(dest_to_orig_t* iter, dest_to_orig_map_t* list)
{
    assert(iter != (dest_to_orig_t*)list);
    dest_to_orig_t* next_p = iter->next;
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
#define FOREACH_DEST_TO_ORIG_MAP(iter, list)                 \
    for (dest_to_orig_t* iter = dest_to_orig_map_iter(list); \
         !dest_to_orig_map_iter_is_end(iter, list);          \
         iter = dest_to_orig_map_iter_next(iter, list))

/**
 * Traverse all the element of the list (reverse order).
 *
 * @param iter
 * @param list
 */
#define FOREACH_DEST_TO_ORIG_MAP_R(iter, list)                 \
    for (dest_to_orig_t* iter = dest_to_orig_map_iter_r(list); \
         !dest_to_orig_map_iter_is_end(iter, list);            \
         iter = dest_to_orig_map_iter_next_r(iter, list))

////////////////////////////////////////////////////////////////
//
// dest_to_orig_map_t
//

/**
 * @param self
 * @return the size of the list
 */
PUBLIC_INLINE size_t
dest_to_orig_map_size(const dest_to_orig_map_t* self)
{
    return self->n;
}

/**
 * @param self
 * @retval true  if the list is empty
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
dest_to_orig_map_is_empty(const dest_to_orig_map_t* self)
{
    return self->next == (dest_to_orig_t*)self;
}

/** Removes the top element.
 *
 * @param self
 */
PUBLIC_INLINE void
dest_to_orig_map_remove_head(dest_to_orig_map_t* self)
{
    if (!dest_to_orig_map_is_empty(self))
        (void)dest_to_orig_map_iter_remove(self->next, self);
}

/** Removes the last element.
 *
 * @param self
 */
PUBLIC_INLINE void
dest_to_orig_map_remove_tail(dest_to_orig_map_t* self)
{
    if (!dest_to_orig_map_is_empty(self))
        (void)dest_to_orig_map_iter_remove(self->prev, self);
}

/** Remove sthe first element of the list, and return it.
 *
 * @param self
 * @return the removed element
 */
PUBLIC_INLINE dest_to_orig_t*
dest_to_orig_map_shift(dest_to_orig_map_t* self)
{
    if (dest_to_orig_map_is_empty(self))
        return NULL;
    dest_to_orig_t* result = self->next;
    dest_to_orig_map_iter_cut(self->next, self);
    return result;
}

/** Prepends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
dest_to_orig_map_insert_head(dest_to_orig_map_t* self, dest_to_orig_t* elt)
{
    dest_to_orig_map_iter_insert_after((dest_to_orig_t*)self, self, elt);
}

/** Appends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
dest_to_orig_map_insert_tail(dest_to_orig_map_t* self, dest_to_orig_t* elt)
{
    dest_to_orig_map_iter_insert_before((dest_to_orig_t*)self, self, elt);
}

/** Gets the top element.
 *
 * @param self
 * @return the top element of self
 */
PUBLIC_INLINE dest_to_orig_t*
dest_to_orig_map_head(dest_to_orig_map_t* self)
{
    if (dest_to_orig_map_is_empty(self))
        return NULL;
    return self->next;
}

/** Removes all the elements
 *
 * @param self
 */
PUBLIC_INLINE void
dest_to_orig_map_remove_all(dest_to_orig_map_t* self)
{
    while (!dest_to_orig_map_is_empty(self))
        (void)dest_to_orig_map_iter_remove(self->prev, self);
}

/** Initializes list.
 *
 * @param self
 */
PUBLIC_INLINE void
dest_to_orig_map_init(dest_to_orig_map_t* self)
{
    self->prev = self->next = (dest_to_orig_t*)self;
    self->n = 0;
}

/** Destroys the list.
 *
 * @param self
 */
PUBLIC_INLINE void
dest_to_orig_map_destroy(dest_to_orig_map_t* self)
{
    dest_to_orig_map_remove_all(self);
    assert((dest_to_orig_t*)self == self->next);
    self->next = self->prev = NULL;
}

/** @} */

#endif
