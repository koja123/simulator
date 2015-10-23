//
// TLV
//
#ifndef NU_PACKET_TLV_H_
#define NU_PACKET_TLV_H_

#include "config.h" //ScenSim-Port://
#include "core/core.h" //ScenSim-Port://
#include "core/time.h"
#include "core/logger.h"
#include "core/mem.h"
#include "packet/constant.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_tlv Packet :: TLV (Type Length Variable)
 * @{
 */

/**
 * TLV (type length value)
 */
typedef struct nu_tlv {
    struct nu_tlv* next;          ///< pointer to the next element.
    struct nu_tlv* prev;          ///< pointer to the prev element.
    uint8_t        type;          ///< type
    uint8_t        type_ext;      ///< type extension
    uint16_t       length;        ///< length of value
    uint8_t        value[0];      ///< value
} nu_tlv_t;

PUBLIC nu_tlv_t* nu_tlv_create_general_time(
        const uint8_t type, const uint8_t type_ext,
        const size_t n, const double* values, const uint8_t* hop,
        const double defaultValue);

PUBLIC nu_bool_t nu_tlv_get_general_time(const nu_tlv_t* self,
        const uint8_t hop, double* v);

PUBLIC void nu_tlv_put_log(const nu_tlv_t* tlv, manet_constant_t*,
        nu_logger_t*);

/** Creates TLV.
 *
 * @param type
 * @param type_ext
 * @param value
 * @param len
 * @return tlv
 */
PUBLIC_INLINE nu_tlv_t*
nu_tlv_create(const uint8_t type, const uint8_t type_ext,
        const uint8_t* value, const uint16_t len)
{
    nu_tlv_t* self = (nu_tlv_t*)nu_mem_malloc(sizeof(nu_tlv_t) + len);
    self->next = self->prev = NULL;
    self->type = type;
    self->type_ext = type_ext;
    self->length = len;
    if (self->length != 0)
        memcpy(self->value, value, self->length);
    return self;
}

/** Creates TLV with 8bits value.
 *
 * @param type
 * @param type_ext
 * @param value
 * @return tlv
 */
PUBLIC_INLINE nu_tlv_t*
nu_tlv_create_u8(const uint8_t type, const uint8_t type_ext,
        const uint8_t value)
{
    nu_tlv_t* self = (nu_tlv_t*)nu_mem_malloc(sizeof(nu_tlv_t) + 1);
    self->next = self->prev = NULL;
    self->type = type;
    self->type_ext = type_ext;
    self->length = 1;
    self->value[0] = value;
    return self;
}

/** Creates TLV with 16bits value.
 *
 * @param type
 * @param type_ext
 * @param value
 * @return TLV object
 */
PUBLIC_INLINE nu_tlv_t*
nu_tlv_create_u16(const uint8_t type, const uint8_t type_ext,
        const uint16_t value)
{
    uint16_t  v = nu_htons(value);
    uint8_t*  p = (uint8_t*)&v;
    nu_tlv_t* self = (nu_tlv_t*)nu_mem_malloc(sizeof(nu_tlv_t) + 2);
    self->next = self->prev = NULL;
    self->type = type;
    self->type_ext = type_ext;
    self->length = 2;
    self->value[0] = *p++;
    self->value[1] = *p;
    return self;
}

/**
 * Creates TLV with 32bits value.
 *
 * @param type
 * @param type_ext
 * @param value
 * @return TLV object
 */
PUBLIC_INLINE nu_tlv_t*
nu_tlv_create_u32(const uint8_t type, const uint8_t type_ext,
        const uint32_t value)
{
    uint32_t  v = nu_htonl(value);
    uint8_t*  p = (uint8_t*)&v;
    nu_tlv_t* self = (nu_tlv_t*)nu_mem_malloc(sizeof(nu_tlv_t) + 4);
    self->next = self->prev = NULL;
    self->type = type;
    self->type_ext = type_ext;
    self->length = 4;
    self->value[0] = *p++;
    self->value[1] = *p++;
    self->value[2] = *p++;
    self->value[3] = *p;
    return self;
}

/** Creates TLV without value.
 *
 * @param type
 * @param type_ext
 * @return TLV object
 */
PUBLIC_INLINE nu_tlv_t*
nu_tlv_create_novalue(const uint8_t type, const uint8_t type_ext)
{
    nu_tlv_t* self = nu_mem_alloc(nu_tlv_t);
    self->next = self->prev = NULL;
    self->type = type;
    self->type_ext = type_ext;
    self->length = 0;
    return self;
}

/** Duplicates TLV.
 *
 * @param src
 * @return new tlv.
 */
PUBLIC_INLINE nu_tlv_t*
nu_tlv_dup(const nu_tlv_t* src)
{
    nu_tlv_t* tlv = (nu_tlv_t*)nu_mem_malloc(sizeof(nu_tlv_t) + src->length);
    memcpy(tlv, src, sizeof(nu_tlv_t) + src->length);
    return tlv;
}

/** Compares tlvs.
 *
 * @param tlv1
 * @param tlv2
 * @return true if tlv1 == tlv2
 */
PUBLIC_INLINE nu_bool_t
nu_tlv_eq(const nu_tlv_t* tlv1, const nu_tlv_t* tlv2)
{
    if (tlv1 == tlv2)
        return true;
    return tlv1->type == tlv2->type &&
           tlv1->type_ext == tlv2->type_ext &&
           tlv1->length == tlv2->length &&
           memcmp(tlv1->value, tlv2->value, tlv1->length) == 0;
}

/** Compares tlvs.
 *
 * @param tlv1
 * @param tlv2
 * @return tlv1 - tlv2
 */
PUBLIC_INLINE int
nu_tlv_cmp(const nu_tlv_t* tlv1, const nu_tlv_t* tlv2)
{
    if (tlv1 == tlv2)
        return 0;
    if (tlv1->type != tlv2->type)
        return (int)tlv1->type - (int)tlv2->type;
    if (tlv1->type_ext != tlv2->type_ext)
        return (int)tlv1->type_ext - (int)tlv2->type_ext;
    if (tlv1->length != tlv2->length)
        return (int)tlv1->length - (int)tlv2->length;
    return memcmp(tlv1->value, tlv2->value, tlv1->length);
}

/** Destroys tlv.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_tlv_free(nu_tlv_t* self)
{
    nu_mem_free(self);
}

/** Gets 8bits value.
 *
 * @param self
 * @param[out] v
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_tlv_get_u8(const nu_tlv_t* self, uint8_t* v)
{
    if (self->length != 1)
        return false;
    *v = self->value[0];
    return true;
}

/** Gets 16bits value.
 *
 * @param self
 * @param[out] v
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_tlv_get_u16(const nu_tlv_t* self, uint16_t* v)
{
    if (self->length != 2)
        return false;
    uint8_t* p = (uint8_t*)v;
    *p++ = self->value[0];
    *p = self->value[1];
    *v = nu_ntohs(*v);
    return true;
}

/** Gets 32bits value.
 *
 * @param self
 * @param[out] v
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_tlv_get_u32(const nu_tlv_t* self, uint32_t* v)
{
    if (self->length != 4)
        return false;
    uint8_t* p = (uint8_t*)v;
    *p++ = self->value[0];
    *p++ = self->value[1];
    *p++ = self->value[2];
    *p = self->value[3];
    *v = nu_ntohl(*v);
    return true;
}

/** Gets time values from 8bits value.
 *
 * @param self
 * @param[out] v
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_tlv_get_time(const nu_tlv_t* self, double* v)
{
    if (self->length != 1)
        return false;
    *v = nu_time_unpack(self->value[self->length - 1]);
    return true;
}

#if 0

/** Outputs values to logger
 *
 * @param tlv
 * @param max_len
 * @param logger
 */
PUBLIC_INLINE void
nu_tlv_put_value(const nu_tlv_t* tlv, const int max_len,
        nu_logger_t* logger)
{
    nu_logger_fmt(logger, "0x");
    for (ssize_t i = 0; i < tlv->length && i < max_len; ++i)
        nu_logger_fmt(logger, "%02x", tlv->value[i]);
}

#endif

/** @} */

}//namespace// //ScenSim-Port://

#endif
