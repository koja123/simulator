#ifndef OLSRV2_LIFO_H_
#define OLSRV2_LIFO_H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup  olsrv2-etx OLSRv2-ETX :: ETX
 * @{
 */

/**
 * LIFO
 */
typedef struct lifo {
    size_t len;     ///< maximum length
    size_t p;       ///< current top
    int*   data;    ///< data
} lifo_t;

/** Initializes the lifo
 *
 * @param self
 * @param len
 */
PUBLIC_INLINE void
lifo_init(lifo_t* self, size_t len)
{
    self->len = len;
    self->p = 0;
    self->data = (int*)nu_mem_calloc(len, sizeof(int));
}

/** Destroys the lifo.
 *
 * @param self
 */
PUBLIC_INLINE void
lifo_destroy(lifo_t* self)
{
    nu_mem_free(self->data);
}

/** Pushes the element.
 *
 * @param self
 * @param v
 */
PUBLIC_INLINE void
lifo_push(lifo_t* self, int v)
{
    self->p = (self->p + 1) % self->len;
    self->data[self->p] = v;
}

/** Sets the top element.
 *
 * @param self
 * @param v
 */
PUBLIC_INLINE void
lifo_set_top(lifo_t* self, int v)
{
    self->data[self->p] = v;
}

/** Adds v to the top element.
 *
 * @param self
 * @param v
 */
PUBLIC_INLINE void
lifo_add_top(lifo_t* self, int v)
{
    self->data[self->p] += v;
}

/** Returns the top element.
 *
 * @param self
 * @return the top element.
 */
PUBLIC_INLINE int
lifo_top(lifo_t* self)
{
    return self->data[self->p];
}

/** Returns the sum of elements.
 *
 * @param self
 * @return the sum of elements
 */
PUBLIC_INLINE double
lifo_sum(lifo_t* self)
{
    double sum = 0;
    for (unsigned i = 0; i < self->len; ++i)
        sum += (double)self->data[i];
    return sum;
}

/** Outputs log.
 *
 * @param self
 * @param logger
 */
PUBLIC_INLINE void
lifo_put_log(lifo_t* self, nu_logger_t* logger)
{
    nu_strbuf_t buf;
    nu_strbuf_init(&buf);
    nu_strbuf_append_cstr(&buf, "lifo:[");
    for (size_t i = 0; i < self->len; ++i) {
        if (self->p == i)
            nu_strbuf_appendf(&buf, " *%d", self->data[i]);
        else
            nu_strbuf_appendf(&buf, " %d", self->data[i]);
    }
    nu_strbuf_append_char(&buf, ']');
    nu_logger_log(logger, nu_strbuf_cstr(&buf));
    nu_strbuf_destroy(&buf);
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
