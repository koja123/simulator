//
// Vector of pointers
//
#ifndef NU_CORE_PVECTOR_H_
#define NU_CORE_PVECTOR_H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_ip_vector Core :: Vector of ip.
 * @{
 */

/**
 * Pointer vector
 */
typedef struct ip_vector {
    size_t   grow; ///< growing size
    size_t   capa; ///< current capcacity
    size_t   size; ///< current size
    nu_ip_t* data; ///< data
} nu_ip_vector_t;

/** Iterator */
typedef nu_ip_t*  nu_ip_vector_iter_t;

/** Traverses all the element of the vector.
 *
 * @param p
 * @param vector
 */
#define FOREACH_IP_VECTOR(p, vector)                        \
    for (nu_ip_vector_iter_t p = nu_ip_vector_iter(vector); \
         !nu_ip_vector_iter_is_end(p, vector);              \
         p = nu_ip_vector_iter_next(p, vector))

/**
 * Gets the iterator which points the first element.
 */
#define nu_ip_vector_iter(vector)    (&((vector)->data[0]))

/**
 * Gets the ip which is pointed by the iterator.
 */
#define nu_ip_vector_iter_get(p)     (*(p))

/**
 * Gets the iterator which points the last(NOT the last element).
 *
 * @param vector
 * @return the iterator which points the end of the list.
 */
#define nu_ip_vector_iter_end(vector) \
    (&((vector)->data[nu_ip_vector_size(vector)]))

/**
 * Gets the next element
 *
 * @param p
 * @param vector
 * @return the iterator which points to the next element
 */
#define nu_ip_vector_iter_next(p, vector)    ((p) + 1)

/**
 * Checks whether the iterator points the last or not.
 *
 * @param p
 * @param vector
 * @return true if p is the iterator which points to the end of the list
 */
#define nu_ip_vector_iter_is_end(p, vector) \
    ((p) == nu_ip_vector_iter_end(vector))

PUBLIC void nu_ip_vector_init(nu_ip_vector_t*);
PUBLIC void nu_ip_vector_destroy(nu_ip_vector_t*);

PUBLIC void nu_ip_vector_add(nu_ip_vector_t*, const nu_ip_t);

/** Returns the number of elements in the list.
 *
 * @param self
 * @return the size
 */
PUBLIC_INLINE size_t
nu_ip_vector_size(nu_ip_vector_t* self)
{
    return self->size;
}

/** Returns the element at the specified position in the list.
 *
 * @param self
 * @param index
 * @return the element at the index in self
 */
PUBLIC_INLINE nu_ip_t
nu_ip_vector_get_at(nu_ip_vector_t* self, const size_t index)
{
    assert(index < self->size);
    return self->data[index];
}

/** Checks whether the ip_vector contains the specified element.
 *
 * @param self
 * @param p
 * @retval true	 if self contains p
 * @retval false otherwise
 */
PUBLIC_INLINE nu_bool_t
nu_ip_vector_contain(nu_ip_vector_t* self, const nu_ip_t p)
{
    for (size_t i = 0; i < self->size; ++i) {
        if (nu_ip_eq(p, self->data[i]))
            return true;
    }
    return false;
}

/** Adds the specified element if the ip_vector does not contained it.
 *
 * @param self
 * @param p
 */
PUBLIC_INLINE void
nu_ip_vector_add_if_noexist(nu_ip_vector_t* self, const nu_ip_t p)
{
    if (!nu_ip_vector_contain(self, p))
        nu_ip_vector_add(self, p);
}

/** Removes the element at the specified position in the ip_vector.
 *
 * @param self
 * @param index
 */
PUBLIC_INLINE void
nu_ip_vector_remove_at(nu_ip_vector_t* self, size_t index)
{
    for (; index < self->size - 1; ++index)
        self->data[index] = self->data[index + 1];
    --self->size;
}

/** Removes the element at the iterator.
 *
 * @param iter
 * @param ip_vector
 */
PUBLIC_INLINE nu_ip_vector_iter_t
nu_ip_vector_iter_remove(nu_ip_vector_iter_t iter, nu_ip_vector_t* ip_vector)
{
    assert(!nu_ip_vector_iter_is_end(iter, ip_vector));
    for (nu_ip_vector_iter_t p = iter + 1, q = iter;
         nu_ip_vector_iter_is_end(p, ip_vector); ++p, ++q)
        *q = *p;
    --ip_vector->size;
    return iter + 1;
}

/** Removes the first element that is equal to the specified element.
 *
 * @param self
 * @param p
 */
PUBLIC_INLINE void
nu_ip_vector_remove(nu_ip_vector_t* self, const nu_ip_t p)
{
    for (size_t i = 0; i < self->size; ++i) {
        if (nu_ip_eq(p, self->data[i])) {
            nu_ip_vector_remove(self, (nu_ip_t)(i));
            return;
        }
    }
}

/** Removes all the elements in the ip_vector.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_ip_vector_remove_all(nu_ip_vector_t* self)
{
    self->size = 0;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
