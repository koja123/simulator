#ifndef NU_CORE_STRBUF_H_
#define NU_CORE_STRBUF_H_

#include <stdarg.h>

#include "core/ip.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_buf
 * @{
 */

/**
 * Variable length string buffer.
 */
typedef struct nu_strbuf {
    size_t capa;                ///< capacity
    size_t len;                 ///< data length
    char*  data;                ///< data buffer
} nu_strbuf_t;

/**
 * @return the string pointer of the strbuf.
 */
#define nu_strbuf_cstr(str)    ((str)->data)

/**
 * @return the length of string.
 */
#define nu_strbuf_len(str)     ((str)->len)

PUBLIC nu_strbuf_t* nu_strbuf_create(void);
PUBLIC void nu_strbuf_init(nu_strbuf_t*);
PUBLIC void nu_strbuf_destroy(nu_strbuf_t*);
PUBLIC void nu_strbuf_free(nu_strbuf_t*);

PUBLIC void nu_strbuf_clear(nu_strbuf_t*);
//PUBLIC void nu_strbuf_append_d(nu_strbuf_t*, int);
//PUBLIC void nu_strbuf_append_x(nu_strbuf_t*, int);
PUBLIC void nu_strbuf_append_char(nu_strbuf_t*, const char);
PUBLIC void nu_strbuf_append_cstr(nu_strbuf_t*, const char*);
PUBLIC void nu_strbuf_append_ip(nu_strbuf_t*, nu_ip_t);
PUBLIC void nu_strbuf_append_bytes(nu_strbuf_t*,
        const uint8_t*, const size_t);
PUBLIC void nu_strbuf_append_printable(nu_strbuf_t*,
        const uint8_t*, const size_t);
PUBLIC void nu_strbuf_appendf(nu_strbuf_t*, const char*, ...);
PUBLIC void nu_strbuf_vappendf(nu_strbuf_t*, const char*, va_list);
PUBLIC void nu_strbuf_put(nu_strbuf_t*, FILE*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
