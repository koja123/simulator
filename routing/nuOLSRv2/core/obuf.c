#include "config.h"

#include "core/core.h"
#include "core/mem.h"
#include "core/buf.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_buf
 * @{
 */

#define BUF_INIT_SIZE    1024       ///< default initial buffer size
#define BUF_GROW_SIZE    256        ///< default growing buffer size

static size_t GROW = BUF_GROW_SIZE;

/** Reallocates buffer.
 *
 * @param self
 */
PUBLIC void
nu_obuf_grow(nu_obuf_t* self)
{
    if (self->capa == 0)
        nu_fatal("Internal Error:Try to modify read only obuf.");
    size_t new_capa = self->capa + GROW;
    self->data = (uint8_t*)nu_mem_realloc(self->data, new_capa);
    self->capa = new_capa;
}

/** Constructs obuf with the default size and
 * the default grow size.
 *
 * @return obuf
 */
PUBLIC nu_obuf_t*
nu_obuf_create(void)
{
    return nu_obuf_create_with_capa_and_grow(BUF_INIT_SIZE,
            BUF_GROW_SIZE);
}

/** Constructs obuf and copy data from src.
 *
 * @param src
 * @return obuf object
 */
PUBLIC nu_obuf_t*
nu_obuf_create_with_obuf(const nu_obuf_t* src)
{
    nu_obuf_t* self = nu_obuf_create_with_capa(src->len);
    nu_obuf_append_bytes(self, src->data, src->len);
    return self;
}

/** Constructs obuf with the initial size size.
 *
 * @param capa  the initial capacity  of buffer
 * @return obuf
 */
PUBLIC nu_obuf_t*
nu_obuf_create_with_capa(const size_t capa)
{
    return nu_obuf_create_with_capa_and_grow(capa, BUF_GROW_SIZE);
}

/**
 * Constructs obuf with the initial size and grow size.
 *
 * @param capa  the initial capacity of buffer
 * @param grow  the growing size of buffer
 * @return obuf
 */
PUBLIC nu_obuf_t*
nu_obuf_create_with_capa_and_grow(const size_t capa,
        const size_t grow)
{
    nu_obuf_t* r = nu_mem_alloc(nu_obuf_t);
    assert(capa > 0);
    assert(grow > 0);
    r->capa = capa;
    r->data = (uint8_t*)nu_mem_calloc(capa, sizeof(uint8_t));
    r->len  = 0;
    return r;
}

/** Constructs obuf with the byte array.
 *
 * @param data
 * @param len
 * @return obuf
 */
PUBLIC nu_obuf_t*
nu_obuf_create_with_bytes(const uint8_t* data, const size_t len)
{
    nu_obuf_t* r = nu_obuf_create_with_capa(len);
    memcpy(r->data, data, len);
    r->len = len;
    return r;
}

/** Duplicates obuf.
 *
 * @param src
 * @return obuf
 */
PUBLIC nu_obuf_t*
nu_obuf_dup(const nu_obuf_t* src)
{
    return nu_obuf_create_with_bytes(src->data, src->len);
}

/** Compares obuf.
 *
 * @param a
 * @param b
 * @retval 0  if equals,
 * @retval <0 if the length of a < the length of b || a < b,
 * @retval >0 if the length of a > the length of b || a > b.
 */
PUBLIC int
nu_obuf_cmp(const nu_obuf_t* a, const nu_obuf_t* b)
{
    int r;
    if ((r = (int)(a->len - b->len)) != 0)
        return r;
    return memcmp(a->data, b->data, a->len);
}

/** Frees obuf.
 *
 * @param self
 */
PUBLIC void
nu_obuf_free(nu_obuf_t* self)
{
    if (self->capa > 0)
        nu_mem_free(self->data);
    nu_mem_free(self);
}

/*
 */
static void
obuf_put(const nu_obuf_t* self, size_t p, nu_logger_t* logger)
{
    nu_logger_log(logger,
            "capa:%lu len:%lu(0x%lx)",
            (long unsigned int)self->capa,
            (long unsigned int)self->len, (long unsigned int)self->len);

    while (p < self->len) {
        size_t sz = (self->len - p < 0x10) ? (self->len - p) : 0x10;
        nu_logger_log(logger, "%08x:%V", (unsigned)p,
                &self->data[p], sz);
        p += sz;
    }
}

/** Outpus obuf to the logger.
 *
 * @param self
 * @param logger
 */
PUBLIC void
nu_obuf_put_log(const nu_obuf_t* self, nu_logger_t* logger)
{
    obuf_put(self, 0, logger);
}

/** Outpus obuf to the file.
 *
 * @param self
 * @param fp
 */
PUBLIC nu_bool_t
nu_obuf_dump(nu_obuf_t* self, FILE* fp)
{
    return fwrite(self->data, 1, self->len, fp) == self->len;
}

/**
 * Restores obuf data from the file.
 *
 * @param self
 * @param fp
 * @retval true  if success
 * @retval false otherwise
 */
PUBLIC nu_bool_t
nu_obuf_restore(nu_obuf_t* self, FILE* fp)
{
    nu_obuf_clear(self);
    int c;
    while ((c = fgetc(fp)) != EOF) {
        if (!nu_obuf_append_u8(self, (uint8_t)c))
            return false;
    }
    return true;
}

/** @} */

}//namespace// //ScenSim-Port://
