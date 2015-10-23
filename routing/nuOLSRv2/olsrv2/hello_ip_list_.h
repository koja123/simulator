#ifndef HELLO_IP_LIST__H_
#define HELLO_IP_LIST__H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup olsrv2
 * @{
 */

////////////////////////////////////////////////////////////////
//
// hello_ip_t
//

/**
 * Gets the iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE hello_ip_t*
hello_ip_list_iter(const hello_ip_list_t* list)
{
    return (hello_ip_t*)list->next;
}

/**
 * Get the reverse iterator which points the first element.
 *
 * @return the iterator of the list
 */
PUBLIC_INLINE hello_ip_t*
hello_ip_list_iter_r(const hello_ip_list_t* list)
{
    return (hello_ip_t*)list->prev;
}

/**
 * Gets the iterator which points the last(NOT the last element).
 *
 * @param list
 * @return the iterator which points the end of the list.
 */
PUBLIC_INLINE hello_ip_t*
hello_ip_list_iter_end(const hello_ip_list_t* list)
{
    return (hello_ip_t*)list;
}

/**
 * Checks whether the iterator points the last or not.
 *
 * @param iter
 * @param list
 * @return true if p is the iterator which points to the end of the list
 */
PUBLIC_INLINE nu_bool_t
hello_ip_list_iter_is_end(const hello_ip_t* iter,
        const hello_ip_list_t* list)
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
PUBLIC_INLINE hello_ip_t*
hello_ip_list_iter_next(const hello_ip_t* iter, const hello_ip_list_t* list)
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
PUBLIC_INLINE hello_ip_t*
hello_ip_list_iter_next_r(const hello_ip_t* iter, const hello_ip_list_t* list)
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
hello_ip_list_iter_insert_before(hello_ip_t* iter,
        hello_ip_list_t* list, hello_ip_t* elt)
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
hello_ip_list_iter_insert_after(hello_ip_t* iter,
        hello_ip_list_t* list, hello_ip_t* elt)
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
PUBLIC_INLINE hello_ip_t*
hello_ip_list_iter_remove(hello_ip_t* iter, hello_ip_list_t* list)
{
    assert(iter != (hello_ip_t*)list);

    hello_ip_t* next_p = iter->next;
    iter->prev->next = iter->next;
    iter->next->prev = iter->prev;
    --list->n;
    //iter->next = iter->prev = NULL;
    hello_ip_free(iter);
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
PUBLIC_INLINE hello_ip_t*
hello_ip_list_iter_cut(hello_ip_t* iter, hello_ip_list_t* list)
{
    assert(iter != (hello_ip_t*)list);
    hello_ip_t* next_p = iter->next;
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
#define FOREACH_HELLO_IP_LIST(iter, list)             \
    for (hello_ip_t* iter = hello_ip_list_iter(list); \
         !hello_ip_list_iter_is_end(iter, list);      \
         iter = hello_ip_list_iter_next(iter, list))

/**
 * Traverse all the element of the list (reverse order).
 *
 * @param iter
 * @param list
 */
#define FOREACH_HELLO_IP_LIST_R(iter, list)             \
    for (hello_ip_t* iter = hello_ip_list_iter_r(list); \
         !hello_ip_list_iter_is_end(iter, list);        \
         iter = hello_ip_list_iter_next_r(iter, list))

////////////////////////////////////////////////////////////////
//
// hello_ip_list_t
//

/**
 * @param self
 * @return the size of the list
 */
PUBLIC_INLINE size_t
hello_ip_list_size(const hello_ip_list_t* self)
{
    return self->n;
}

/**
 * @param self
 * @retval true  if the list is empty
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
hello_ip_list_is_empty(const hello_ip_list_t* self)
{
    return self->next == (hello_ip_t*)self;
}

/** Removes the top element.
 *
 * @param self
 */
PUBLIC_INLINE void
hello_ip_list_remove_head(hello_ip_list_t* self)
{
    if (!hello_ip_list_is_empty(self))
        (void)hello_ip_list_iter_remove(self->next, self);
}

/** Removes the last element.
 *
 * @param self
 */
PUBLIC_INLINE void
hello_ip_list_remove_tail(hello_ip_list_t* self)
{
    if (!hello_ip_list_is_empty(self))
        (void)hello_ip_list_iter_remove(self->prev, self);
}

/** Remove sthe first element of the list, and return it.
 *
 * @param self
 * @return the removed element
 */
PUBLIC_INLINE hello_ip_t*
hello_ip_list_shift(hello_ip_list_t* self)
{
    if (hello_ip_list_is_empty(self))
        return NULL;
    hello_ip_t* result = self->next;
    hello_ip_list_iter_cut(self->next, self);
    return result;
}

/** Prepends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
hello_ip_list_insert_head(hello_ip_list_t* self, hello_ip_t* elt)
{
    hello_ip_list_iter_insert_after((hello_ip_t*)self, self, elt);
}

/** Appends the element.
 *
 * @param self
 * @param elt
 */
PUBLIC_INLINE void
hello_ip_list_insert_tail(hello_ip_list_t* self, hello_ip_t* elt)
{
    hello_ip_list_iter_insert_before((hello_ip_t*)self, self, elt);
}

/** Gets the top element.
 *
 * @param self
 * @return the top element of self
 */
PUBLIC_INLINE hello_ip_t*
hello_ip_list_head(hello_ip_list_t* self)
{
    if (hello_ip_list_is_empty(self))
        return NULL;
    return self->next;
}

/** Removes all the elements
 *
 * @param self
 */
PUBLIC_INLINE void
hello_ip_list_remove_all(hello_ip_list_t* self)
{
    while (!hello_ip_list_is_empty(self))
        (void)hello_ip_list_iter_remove(self->prev, self);
}

/** Initializes list.
 *
 * @param self
 */
PUBLIC_INLINE void
hello_ip_list_init(hello_ip_list_t* self)
{
    self->prev = self->next = (hello_ip_t*)self;
    self->n = 0;
}

/** Destroys the list.
 *
 * @param self
 */
PUBLIC_INLINE void
hello_ip_list_destroy(hello_ip_list_t* self)
{
    hello_ip_list_remove_all(self);
    assert((hello_ip_t*)self == self->next);
    self->next = self->prev = NULL;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
