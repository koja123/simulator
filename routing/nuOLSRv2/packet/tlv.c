#include "config.h"

#include <math.h>

#include "core/core.h"
#include "packet/tlv.h"
#include "packet/link_metric.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_tlv
 * @{
 */

/** Creates general time tlv.
 *
 * @param type
 * @param type_ext
 * @param n
 * @param values
 * @param hop
 * @param defaultValue
 * @return tlv
 */
nu_tlv_t*
nu_tlv_create_general_time(const uint8_t type, const uint8_t type_ext,
        const size_t n, const double* values, const uint8_t* hop,
        const double defaultValue)
{
    nu_tlv_t* self = (nu_tlv_t*)nu_mem_malloc(sizeof(nu_tlv_t) +
            n * 2 + 1);
    self->next = self->prev = NULL;
    self->type = type;
    self->type_ext = type_ext;
    self->length = (uint16_t)(n * 2 + 1);
    size_t p = 0;
    for (size_t i = 0; i < n; ++i) {
        self->value[p++] = nu_time_pack(values[i]);
        self->value[p++] = hop[i];
    }
    self->value[p] = nu_time_pack(defaultValue);
    return self;
}

/** Gets general time.
 *
 * @param self
 * @param hop
 * @param[out] v
 * @return true if success.
 */
PUBLIC nu_bool_t
nu_tlv_get_general_time(const nu_tlv_t* self, const uint8_t hop, double* v)
{
    if (self->length < 1)
        return false;
    if (self->length == 1) {
        *v = nu_time_unpack(self->value[0]);
        return true;
    } else {
        if (self->length % 2 == 0) {
            nu_warn("Illegal General Time TLV structure");
            return false;
        }
        for (size_t i = 1; i < self->length && hop <= self->value[i]; i += 2) {
            *v = nu_time_unpack(self->value[i - 1]);
            return true;
        }
        *v = nu_time_unpack(self->value[self->length - 1]);
        return true;
    }
}

/** Outputs TLV.
 *
 * @param tlv
 * @param table
 * @param logger
 */
PUBLIC void
nu_tlv_put_log(const nu_tlv_t* tlv, manet_constant_t* table,
        nu_logger_t* logger)
{
    const char* name;
    const manet_constant_t* tbl = table;
    nu_strbuf_t buf;
    nu_strbuf_init(&buf);
    nu_strbuf_append_cstr(&buf, "tlv[");
    name = NULL;
    while (tbl != NULL && tbl->name != NULL) {
        if (tlv->type == tbl->code) {
            name = tbl->name;
            tbl  = tbl->value;
            break;
        }
        ++tbl;
    }
    if (name) {
        nu_strbuf_appendf(&buf, "type:%s type-ext:%d",
                name, tlv->type_ext);
    } else {
        nu_strbuf_appendf(&buf, "type:%d type-ext:%d",
                tlv->type, tlv->type_ext);
    }
    if (tbl == manet_tlv_link_metric) {
        nu_strbuf_appendf(&buf, " metric:%g",
                nu_link_metric_unpack(tlv->value[0]));
    } else if (tbl == manet_tlv_general_time) {
        nu_strbuf_append_cstr(&buf, " time:");
        for (int i = 0; i < tlv->length - 1; i += 2) {
            nu_strbuf_appendf(&buf, "(%d:%g)",
                    nu_time_unpack(tlv->value[i]),
                    nu_time_unpack(tlv->value[i + 1]));
        }
        nu_strbuf_appendf(&buf, "%g", nu_time_unpack(tlv->value[0]));
    } else {
        nu_strbuf_appendf(&buf, " length:%d", tlv->length);
        if (tlv->length == 1) {
            name = NULL;
            while (tbl != NULL && tbl->name != 0) {
                if (tlv->value[0] == tbl->code) {
                    name = tbl->name;
                    break;
                }
                ++tbl;
            }
            if (name) {
                nu_strbuf_appendf(&buf,
                        " value:%s(0x%02x)", name, tlv->value[0]);
            }
        }
        if (tlv->length > 1) {
            nu_strbuf_appendf(&buf, " value:0x%V", tlv->value, (size_t)tlv->length);
        }
    }
    nu_strbuf_append_char(&buf, ']');
    nu_logger_log(logger, nu_strbuf_cstr(&buf));
    nu_strbuf_destroy(&buf);
}

/** @} */

}//namespace// //ScenSim-Port://
