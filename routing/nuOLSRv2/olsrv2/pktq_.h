#ifndef OLSRV2_PKTQ__H_
#define OLSRV2_PKTQ__H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup olsrv2
 * @{
 */

////////////////////////////////////////////////////////////////
//
// olsrv2_pkt_t
//

/**
 * Gets the iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE olsrv2_pkt_t*
olsrv2_pktq_iter(const olsrv2_pktq_t* list)
{
    return (olsrv2_pkt_t*)list->next;
}

/**
 * Get the reverse iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE olsrv2_pkt_t*
olsrv2_pktq_iter_r(const olsrv2_pktq_t* list)
{
    return (olsrv2_pkt_t*)list->prev;
}

/**
 * Gets the iterator which points the last(NOT the last element).
 *
 * @param list
 * @return the iterator which points the end of the list.
 */
PUBLIC_INLINE olsrv2_pkt_t*
olsrv2_pktq_iter_end(const olsrv2_pktq_t* list)
{
    return (olsrv2_pkt_t*)list;
}

/**
 * Checks whether the iterator points the last or not.
 *
 * @param iter
 * @param list
 * @return true if p is the iterator which points to the end of the list
 */
PUBLIC_INLINE nu_bool_t
olsrv2_pktq_iter_is_end(const olsrv2_pkt_t* iter,
        const olsrv2_pktq_t* list)
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
PUBLIC_INLINE olsrv2_pkt_t*
olsrv2_pktq_iter_next(const olsrv2_pkt_t* iter, const olsrv2_pktq_t* list)
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
PUBLIC_INLINE olsrv2_pkt_t*
olsrv2_pktq_iter_next_r(const olsrv2_pkt_t* iter, const olsrv2_pktq_t* list)
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
olsrv2_pktq_iter_insert_before(olsrv2_pkt_t* iter,
        olsrv2_pktq_t* list, olsrv2_pkt_t* elt)
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
olsrv2_pktq_iter_insert_after(olsrv2_pkt_t* iter,
        olsrv2_pktq_t* list, olsrv2_pkt_t* elt)
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
PUBLIC_INLINE olsrv2_pkt_t*
olsrv2_pktq_iter_remove(olsrv2_pkt_t* iter, olsrv2_pktq_t* list)
{
    assert(iter != (olsrv2_pkt_t*)list);

    olsrv2_pkt_t* next_p = iter->next;
    iter->prev->next = iter->next;
    iter->next->prev = iter->prev;
    --list->n;
    //iter->next = iter->prev = NULL;
    olsrv2_pkt_free(iter);
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
PUBLIC_INLINE olsrv2_pkt_t*
olsrv2_pktq_iter_cut(olsrv2_pkt_t* iter, olsrv2_pktq_t* list)
{
    assert(iter != (olsrv2_pkt_t*)list);
    olsrv2_pkt_t* next_p = iter->next;
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
#define FOREACH_PKTQ(iter, list)                      \
    for (olsrv2_pkt_t* iter = olsrv2_pktq_iter(list); \
         !olsrv2_pktq_iter_is_end(iter, list);        \
         iter = olsrv2_pktq_iter_next(iter, list))

/**
 * Traverse all the element of the list (reverse order).
 *
 * @param iter
 * @param list
 */
#define FOREACH_PKTQ_R(iter, list)                      \
    for (olsrv2_pkt_t* iter = olsrv2_pktq_iter_r(list); \
         !olsrv2_pktq_iter_is_end(iter, list);          \
         iter = olsrv2_pktq_iter_next_r(iter, list))

////////////////////////////////////////////////////////////////
//
// olsrv2_pktq_t
//

/**
 * @param self
 * @return the size of the list
 */
PUBLIC_INLINE size_t
olsrv2_pktq_size(const olsrv2_pktq_t* self)
{
    return self->n;
}

/**
 * @param self
 * @retval true  if the list is empty
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
olsrv2_pktq_is_empty(const olsrv2_pktq_t* self)
{
    return self->next == (olsrv2_pkt_t*)self;
}

/** Removes the top element.
 *
 * @param self
 */
PUBLIC_INLINE void
olsrv2_pktq_remove_head(olsrv2_pktq_t* self)
{
    if (!olsrv2_pktq_is_empty(self))
        (void)olsrv2_pktq_iter_remove(self->next, self);
}

/** Removes the last element.
 *
 * @param self
 */
PUBLIC_INLINE void
olsrv2_pktq_remove_tail(olsrv2_pktq_t* self)
{
    if (!olsrv2_pktq_is_empty(self))
        (void)olsrv2_pktq_iter_remove(self->prev, self);
}

/** Remove sthe first element of the list, and return it.
 *
 * @param self
 * @return the removed element
 */
PUBLIC_INLINE olsrv2_pkt_t*
olsrv2_pktq_shift(olsrv2_pktq_t* self)
{
    if (olsrv2_pktq_is_empty(self))
        return NULL;
    olsrv2_pkt_t* result = self->next;
    olsrv2_pktq_iter_cut(self->next, self);
    return result;
}

/** Prepends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
olsrv2_pktq_insert_head(olsrv2_pktq_t* self, olsrv2_pkt_t* elt)
{
    olsrv2_pktq_iter_insert_after((olsrv2_pkt_t*)self, self, elt);
}

/** Appends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
olsrv2_pktq_insert_tail(olsrv2_pktq_t* self, olsrv2_pkt_t* elt)
{
    olsrv2_pktq_iter_insert_before((olsrv2_pkt_t*)self, self, elt);
}

/** Gets the top element.
 *
 * @param self
 * @return the top element of self
 */
PUBLIC_INLINE olsrv2_pkt_t*
olsrv2_pktq_head(olsrv2_pktq_t* self)
{
    if (olsrv2_pktq_is_empty(self))
        return NULL;
    return self->next;
}

/** Removes all the elements
 *
 * @param self
 */
PUBLIC_INLINE void
olsrv2_pktq_remove_all(olsrv2_pktq_t* self)
{
    while (!olsrv2_pktq_is_empty(self))
        (void)olsrv2_pktq_iter_remove(self->prev, self);
}

/** Initializes list.
 *
 * @param self
 */
PUBLIC_INLINE void
olsrv2_pktq_init(olsrv2_pktq_t* self)
{
    self->prev = self->next = (olsrv2_pkt_t*)self;
    self->n = 0;
}

/** Destroys the list.
 *
 * @param self
 */
PUBLIC_INLINE void
olsrv2_pktq_destroy(olsrv2_pktq_t* self)
{
    olsrv2_pktq_remove_all(self);
    assert((olsrv2_pkt_t*)self == self->next);
    self->next = self->prev = NULL;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
