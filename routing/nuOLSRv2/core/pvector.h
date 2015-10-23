//
// Vector of pointers
//
#ifndef NU_CORE_PVECTOR_H_
#define NU_CORE_PVECTOR_H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_pvector Core :: Vector of pointers
 * @{
 */

/**
 * Pointer vector
 */
typedef struct pvector {
    size_t grow;    ///< growing size
    size_t capa;    ///< current capcacity
    size_t size;    ///< current size
    void** data;    ///< data
} nu_pvector_t;

/** Iterator */
typedef void** nu_pvector_iter_t;

/** Traverses all the element of the vector.
 *
 * @param p
 * @param vector
 */
#define FOREACH_PVECTOR(p, vector)                      \
    for (nu_pvector_iter_t p = nu_pvector_iter(vector); \
         !nu_pvector_iter_is_end(p, vector);            \
         p = nu_pvector_iter_next(p, vector))

/**
 * Gets the iterator which points the first element.
 */
#define nu_pvector_iter(vector)    (&((vector)->data[0]))

/**
 * Gets the ip which is pointed by the iterator.
 */
#define nu_pvector_iter_get(p)     (*(p))

/**
 * Gets the iterator which points the last(NOT the last element).
 *
 * @param vector
 * @return the iterator which points the end of the list.
 */
#define nu_pvector_iter_end(vector) \
    (&((vector)->data[nu_pvector_size(vector)]))

/**
 * Gets the next element
 *
 * @param p
 * @param vector
 * @return the iterator which points to the next element
 */
#define nu_pvector_iter_next(p, vector)    ((p) + 1)

/**
 * Checks whether the iterator points the last or not.
 *
 * @param p
 * @param vector
 * @return true if p is the iterator which points to the end of the list
 */
#define nu_pvector_iter_is_end(p, vector) \
    ((p) == nu_pvector_iter_end(vector))

PUBLIC void nu_pvector_init(nu_pvector_t*, size_t, size_t);
PUBLIC void nu_pvector_destroy(nu_pvector_t*);

PUBLIC void nu_pvector_add(nu_pvector_t*, void*);

/** Returns the number of elements in the list.
 *
 * @param self
 * @return the size
 */
PUBLIC_INLINE size_t
nu_pvector_size(nu_pvector_t* self)
{
    return self->size;
}

/** Returns the element at the specified position in the list.
 *
 * @param self
 * @param index
 * @return the element at the index in self
 */
PUBLIC_INLINE void*
nu_pvector_get_at(nu_pvector_t* self, const size_t index)
{
    assert(index < self->size);
    return self->data[index];
}

/** Checks whether the pvector contains the specified element.
 *
 * @param self
 * @param p
 * @retval true	 if self contains p
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_pvector_contain(nu_pvector_t* self, void* p)
{
    for (size_t i = 0; i < self->size; ++i) {
        if (p == self->data[i])
            return true;
    }
    return false;
}

/** Adds the specified element if the pvector does not contained it.
 *
 * @param self
 * @param p
 */
PUBLIC_INLINE void
nu_pvector_add_if_noexist(nu_pvector_t* self, void* p)
{
    if (!nu_pvector_contain(self, p))
        nu_pvector_add(self, p);
}

/** Removes the element at the specified position in the pvector.
 *
 * @param self
 * @param index
 */
PUBLIC_INLINE void
nu_pvector_remove(nu_pvector_t* self, size_t index)
{
    for (; index < self->size - 1; ++index)
        self->data[index] = self->data[index + 1];
    --self->size;
}

/** Removes the element at the iterator.
 *
 * @param iter
 * @param pvector
 */
PUBLIC_INLINE nu_pvector_iter_t
nu_pvector_iter_remove(nu_pvector_iter_t iter, nu_pvector_t* pvector)
{
    assert(!nu_pvector_iter_is_end(iter, pvector));
    for (nu_pvector_iter_t p = iter + 1, q = iter;
         nu_pvector_iter_is_end(p, pvector); ++p, ++q)
        *q = *p;
    --pvector->size;
    return iter + 1;
}

/** Removes the first element that is equal to the specified element.
 *
 * @param self
 * @param p
 */
PUBLIC_INLINE void
nu_pvector_remove_ptr(nu_pvector_t* self, const void* p)
{
    for (size_t i = 0; i < self->size; ++i) {
        if (p == self->data[i]) {
            nu_pvector_remove(self, i);
            return;
        }
    }
}

/** Removes all the elements in the pvector.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_pvector_remove_all(nu_pvector_t* self)
{
    self->size = 0;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
