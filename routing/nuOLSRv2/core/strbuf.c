#include "config.h"

#include <assert.h>
#include <stdio.h>
#if 0               //ScenSim-Port://
#include <stdint.h>
#endif              //ScenSim-Port://
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <limits.h> //ScenSim-Port://

#include "core/core.h"
#include "core/mem.h"
#include "core/strbuf.h"

#include "core/ip_set.h"

// Insert verify()
#define USE_VERIFY    0

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_buf
 * @{
 */

#define STR_INIT_SIZE    80     ///< Default initial size of strbuf.
#define STR_GROW_SIZE    80     ///< Default growing size of strbuf.

/*
 */
static inline int
verify(nu_strbuf_t* self)
{
#if USE_VERIFY
    int i;
    assert(self->capa > self->len);
    for (i = 0; i < self->len; ++i)
        assert(self->data[i] != '\0');
    assert(self->data[self->len] == '\0');
#endif
    return 1;
}

static void
strbuf_grow(nu_strbuf_t* self)
{
    size_t new_capa = self->capa + STR_GROW_SIZE;
    self->data = (char*)nu_mem_realloc(self->data,
            sizeof(nu_strbuf_t) + new_capa);
    self->capa = new_capa;
}

/** Initializes strbuf.
 *
 * @param self
 */
PUBLIC void
nu_strbuf_init(nu_strbuf_t* self)
{
    self->capa = STR_INIT_SIZE;
    self->data = (char*)nu_mem_calloc(self->capa, sizeof(char));
    self->data[0] = '\0';
    self->len = 0;
}

/** Allocates and initializes strbuf.
 *
 * @return strbuf object
 */
PUBLIC nu_strbuf_t*
nu_strbuf_create(void)
{
    nu_strbuf_t* r = nu_mem_alloc(nu_strbuf_t);
    nu_strbuf_init(r);
    return r;
}

/** Destroys strbuf.
 *
 * @param self
 */
PUBLIC void
nu_strbuf_destroy(nu_strbuf_t* self)
{
    nu_mem_free(self->data);
}

/** Destroys and frees the strbuf.
 *
 * @param self
 */
PUBLIC void
nu_strbuf_free(nu_strbuf_t* self)
{
    nu_strbuf_destroy(self);
    nu_mem_free(self);
}

/** Clears the strbuf.
 *
 * @param self
 */
PUBLIC void
nu_strbuf_clear(nu_strbuf_t* self)
{
    self->len = 0;
    self->data[0] = '\0';
}

/** Appends the character to the strbuf.
 *
 * @param self
 * @param c
 */
PUBLIC void
nu_strbuf_append_char(nu_strbuf_t* self, const char c)
{
    while (self->len + 1 >= self->capa)
        strbuf_grow(self);
    self->data[self->len++] = c;
    self->data[self->len] = '\0';
}

/** Appends the string to the strbuf.
 *
 * @param self
 * @param c
 */
PUBLIC void
nu_strbuf_append_cstr(nu_strbuf_t* self, const char* c)
{
    size_t alen = strlen(c);
    while (self->len + alen + 1 >= self->capa)
        strbuf_grow(self);
    strcpy(&self->data[self->len], c);
    self->len += alen;
    assert(self->len == strlen(self->data));
    verify(self);
}

/** Appends the ip to the strbuf.
 *
 * @param self
 * @param ip
 */
PUBLIC void
nu_strbuf_append_ip(nu_strbuf_t* self, const nu_ip_t ip)
{
    nu_ip_str_t buf;
    nu_ip_to_s(ip, buf);
    nu_strbuf_append_cstr(self, buf);
    verify(self);
}

/** Appends the byte array to the strbuf.
 *
 * @param self
 * @param a
 * @param alen
 */
PUBLIC void
nu_strbuf_append_bytes(nu_strbuf_t* self,
        const uint8_t* a, const size_t alen)
{
    char buf[4];
    nu_bool_t first = true;
    for (size_t i = 0; i < alen; ++i) {
        if (!first)
            nu_strbuf_append_char(self, ':');
        first = false;
        snprintf(buf, sizeof(buf), "%02x", a[i]);
        nu_strbuf_append_cstr(self, buf);
    }
    verify(self);
}

/** Appends the printable character in the byte array to strbuf.
 *
 * @param self
 * @param a
 * @param alen
 */
PUBLIC void
nu_strbuf_append_printable(nu_strbuf_t* self,
        const uint8_t* a, const size_t alen)
{
    verify(self);
    for (size_t i = 0; i < alen; ++i) {
        if (isprint(a[i]))
            nu_strbuf_append_char(self, a[i]);
        else
            nu_strbuf_append_char(self, '?');
    }
    verify(self);
}

/** Outputs the strbuf.
 *
 * @param self
 * @param fp
 */
PUBLIC void
nu_strbuf_put(nu_strbuf_t* self, FILE* fp)
{
    fputs(self->data, fp);
    verify(self);
}

/** Subset and some extension of snprintf().
 *
 * SUPPORT FORMAT
 *  %%
 *  %[-0-9]*s
 *  %[-0-9]*c
 *  %(-?[0-9]+)d
 *  %(-?[0-9]+)ld
 *  %(-?[0-9]+)x
 *  %(-?[0-9]+)lx
 *  %(-?[0-9]+)u
 *  %(-?[0-9]+)lu
 *  %(-?[0-9]+)I: nu_ip_t
 *  %S: nu_ip_set_t/nu_ip_list_t
 *  %B: nu_bool_t
 *      %V: uint8_t*, size_t
 *  %(-?[0-9]+)T: time - NOW
 */
PUBLIC void
nu_strbuf_vappendf(nu_strbuf_t* self, const char* format, va_list ap)
{
#define APPENDF(...)                                      \
    do {                                                  \
        size_t n;                                         \
        while (1) {                                       \
            n = snprintf(&self->data[self->len],          \
                    self->capa - self->len, __VA_ARGS__); \
            if (n < self->capa - self->len)               \
                break;                                    \
            strbuf_grow(self);                            \
        }                                                 \
        self->len += n;                                   \
    }                                                     \
    while (0)

#if defined(_WIN32) || defined(_WIN64)        //ScenSim-Port://
    char* buf = new char[strlen(format) + 1]; //ScenSim-Port://
#else                                         //ScenSim-Port://
    char  buf[strlen(format) + 1];
#endif                                        //ScenSim-Port://
    char* bp = &buf[0];
    for (const char* p = format; *p != '\0';) {
        int long_type = 0;
        switch (*p) {
        case '%':
            *bp++ = *p++;
            if (*p == '%' || *p == '\0') {
                *bp++ = *p++;
                continue;
            }
            long_type = 0;
            // copy format string
            while (*p != '\0' && strchr("scdiouxXfegpISBTV%", *p) == NULL) {
                if (*p == 'l')
                    ++long_type;
                *bp++ = *p++;
            }
            if (*p == '\0' || *p == '%') {
                fputs("FORMAT_ERROR:'", stderr);
                fputs(format, stderr);
                fputs("'\n'", stderr);
                fputs(format, stderr);
                fputs("'\n", stderr);
                abort();
            }
            *bp++ = *p++;
            *bp = '\0';

            switch (*(bp - 1)) {
            case 'I': {
                nu_ip_t     ip = va_arg(ap, uint32_t);
                nu_ip_str_t ips;
                nu_ip_to_s(ip, ips);
                *(bp - 1) = 's';
                APPENDF(buf, (char*)ips);
                break;
            }
            case 'T': {
                nu_time_t* t = va_arg(ap, nu_time_t*);
                double     v = nu_time_calc_timeout(*t);
                *(bp - 1) = 'g';
                APPENDF(buf, v);
                break;
            }
            case 'S': {
                nu_ip_set_t* set = va_arg(ap, nu_ip_set_t*);
                nu_bool_t    first = true;
                *(bp - 2) = '\0'; // overwrite '%'
                nu_strbuf_append_cstr(self, buf);
                FOREACH_IP_SET(i, set) {
                    if (!first)
                        nu_strbuf_append_char(self, ' ');
                    nu_strbuf_append_ip(self, i->ip);
                    first = false;
                }
                break;
            }
            case 'B': {
                int b = va_arg(ap, int);
                *(bp - 1) = 's';
                APPENDF(buf, (b ? "yes" : "no"));
                break;
            }
            case 'V': {
                const uint8_t* v = va_arg(ap, const uint8_t*);
                size_t sz = va_arg(ap, size_t);
                *(bp - 2) = '\0'; // overwrite '%'
                nu_strbuf_append_cstr(self, buf);
                for (size_t i = 0; i < sz; ++i) {
                    APPENDF("%02x", v[i]);
                }
                break;
            }

            case 'd':
            case 'i':
            case 'o':
                if (long_type == 0) {
                    int v = va_arg(ap, int);
                    APPENDF(buf, v);
                } else if (long_type == 1) {
                    long v = va_arg(ap, long);
                    APPENDF(buf, v);
                } else if (long_type == 2) {
                    long long v = va_arg(ap, long long);
                    APPENDF(buf, v);
                } else {
                    nu_fatal("Unsupported format '%s'.", buf);
                }
                break;

            case 'u':
            case 'x':
            case 'X':
                if (long_type == 0) {
                    unsigned int v = va_arg(ap, unsigned int);
                    APPENDF(buf, v);
                } else if (long_type == 1) {
                    unsigned long v = va_arg(ap, unsigned long);
                    APPENDF(buf, v);
                } else if (long_type == 2) {
                    unsigned long long v = va_arg(ap, unsigned long long);
                    APPENDF(buf, v);
                } else {
                    nu_fatal("Unsupported format '%s'.", buf);
                }
                break;

            case 'f':
            case 'g':
            case 'e': {
                double v = va_arg(ap, double);
                APPENDF(buf, v);
                break;
            }

            case 'c': {
                int iv = va_arg(ap, int);
                assert((iv >= CHAR_MIN) && (iv <= CHAR_MAX));
                char v = (char)iv;
                APPENDF(buf, v);
                break;
            }

            case 's': {
                char* v = va_arg(ap, char*);
                APPENDF(buf, v);
                break;
            }

            case 'p': {
                void* v = va_arg(ap, void*);
                APPENDF(buf, v);
                break;
            }

            default:
                fputs("UNKNOWN_FORMAT_ERROR:'", stderr);
                fputs(buf, stderr);
                fputs("'\n", stderr);
                abort();
            }
            bp = &buf[0];
            break;              /* case '%' */

        default:
            *bp++ = *p++;
        }
    }
    *bp = '\0';
    nu_strbuf_append_cstr(self, buf);
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    delete[] buf;                      //ScenSim-Port://
#endif                                 //ScenSim-Port://
}

/** Appends the formated string to strbuf.
 *
 * @param self
 * @param format
 */
PUBLIC void
nu_strbuf_appendf(nu_strbuf_t* self, const char* format, ...)
{
    va_list ap;
    va_start(ap, format);
    nu_strbuf_vappendf(self, format, ap);
    va_end(ap);
}

/** @} */

}//namespace// //ScenSim-Port://
