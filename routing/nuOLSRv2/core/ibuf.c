//
// Input buffer
//
#include "config.h"

#include "core/core.h"
#include "core/mem.h"
#include "core/buf.h"

#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
#define __func__ __FUNCTION__          //ScenSim-Port://
#endif                                 //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_buf
 * @{
 */

/** Creates ibuf object.
 *
 * @param data
 * @param len
 * @return ibuf object
 */
PUBLIC nu_ibuf_t*
nu_ibuf_create(const uint8_t* data, const size_t len)
{
    nu_ibuf_t* r = nu_mem_alloc(nu_ibuf_t);
    r->data = (uint8_t*)nu_mem_calloc(len, sizeof(uint8_t));
    r->head = r->data;
    memcpy(r->data, data, len);
    r->len = (ssize_t)(len);
    r->is_child = false;
    return r;
}

/** Creates ibuf object.
 *
 * @param src
 * @return ibuf object
 */
PUBLIC nu_ibuf_t*
nu_ibuf_create_with_obuf(const nu_obuf_t* src)
{
    return nu_ibuf_create(src->data, src->len);
}

/** Creates ibuf object.
 *
 * @param src
 * @param len
 * @return ibuf object
 */
PUBLIC nu_ibuf_t*
nu_ibuf_create_child(nu_ibuf_t* src, size_t len)
{
    if (nu_ibuf_remain(src) < len) {
        nu_err("%s:packet size is too short to read %d bytes(remain = %d)",
                __func__, len, nu_ibuf_remain(src));
        return NULL;
    }
    nu_ibuf_t* r = nu_mem_alloc(nu_ibuf_t);
    r->len = (ssize_t)(len);
    r->is_child = true;
    r->data = r->head = src->head;
    src->head += len;   // increments the pointer of the parent buffer
    return r;
}

/** Duplicates the ibuf object.
 *
 * @param src
 * @return copied ibuf object
 */
PUBLIC nu_ibuf_t*
nu_ibuf_dup(const nu_ibuf_t* src)
{
    return nu_ibuf_create(src->data, src->len);
}

/** Destroys and frees the ibuf object.
 *
 * @param self
 */
PUBLIC void
nu_ibuf_free(nu_ibuf_t* self)
{
    if (!self->is_child)
        nu_mem_free(self->data);
    nu_mem_free(self);
}

/*
 */
static void
ibuf_put(const nu_ibuf_t* self, ssize_t p, nu_logger_t* logger)
{
    nu_logger_log(logger,
            "len:%lu, head:0x%8p, data:0x%8p",
            (long unsigned int)self->len, self->head, self->data);

    nu_strbuf_t buf;
    nu_strbuf_init(&buf);
    while (p < self->len) {
        nu_strbuf_appendf(&buf, "%08x", (unsigned)p);
        for (int i = 0; i < 0x10; ++i, ++p) {
            if (p < self->len)
                nu_strbuf_appendf(&buf, " %02x", self->data[p]);
        }
        nu_logger_log_strbuf(logger, &buf);
    }
    nu_strbuf_destroy(&buf);
}

/** Outputs ibuf to the logger.
 *
 * @param self
 * @param logger
 */
PUBLIC void
nu_ibuf_put_log(const nu_ibuf_t* self, nu_logger_t* logger)
{
    ibuf_put(self, 0, logger);
}

/** Outputs ibuf to the logger.
 *
 * @param self
 * @param logger
 */
PUBLIC void
nu_ibuf_put_log_from_ptr(nu_ibuf_t* self, nu_logger_t* logger)
{
    ibuf_put(self, (ssize_t)(self->head - self->data), logger);
}

/** Write buffer to the file.
 *
 * @param self
 * @param fp
 */
PUBLIC nu_bool_t
nu_ibuf_dump(nu_ibuf_t* self, FILE* fp)
{
    return fwrite(self->data, 1, self->len, fp) == (size_t)self->len;
}

/** Creates ibuf object from the file.
 *
 * @param fp
 * @return ibuf object
 */
PUBLIC nu_ibuf_t*
nu_ibuf_create_from_fp(FILE* fp)
{
    nu_ibuf_t* self = NULL;
    nu_obuf_t* tmp  = nu_obuf_create();
    int c;
    while ((c = fgetc(fp)) != EOF) {
        if (!nu_obuf_append_u8(tmp, (uint8_t)c))
            goto finish;
    }
    self = nu_ibuf_create(tmp->data, tmp->len);

finish:
    nu_obuf_free(tmp);
    return self;
}

/** @} */

}//namespace// //ScenSim-Port://
