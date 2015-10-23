#ifndef NU_CORE_BUF_H_
#define NU_CORE_BUF_H_

#include "config.h" //ScenSim-Port://
#include "core/core.h" //ScenSim-Port://
#include "core/logger.h"
#include "core/ip_inline.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_buf Core :: Input / Output Buffer
 * @brief input/output buffer
 * @{
 */

/**
 * Input buffer.
 */
typedef struct nu_ibuf {
    nu_ip_t   src_ip;         ///< source IP in UDP packet
    ssize_t   len;            ///< data length
    uint8_t*  head;           ///< pointer for reading
    nu_bool_t is_child;       ///< child buffer flag
    uint8_t*  data;           ///< data
} nu_ibuf_t;

/**
 * Output buffer.
 */
typedef struct nu_obuf {
    size_t   len;               ///< data length
    size_t   capa;              ///< >0:capacity / ==0:child buffer
    uint8_t* data;              ///< data
} nu_obuf_t;

////////////////////////////////////////////////////////////////
//
// ibuf
//

PUBLIC nu_ibuf_t* nu_ibuf_create(const uint8_t* data, const size_t len);
PUBLIC nu_ibuf_t* nu_ibuf_create_with_obuf(const nu_obuf_t* src);
PUBLIC nu_ibuf_t* nu_ibuf_create_from_fp(FILE* fp);
PUBLIC nu_ibuf_t* nu_ibuf_create_child(nu_ibuf_t*, size_t len);
PUBLIC nu_ibuf_t* nu_ibuf_dup(const nu_ibuf_t*);
PUBLIC void nu_ibuf_free(nu_ibuf_t*);

PUBLIC void nu_ibuf_put_log(const nu_ibuf_t*, nu_logger_t*);
PUBLIC void nu_ibuf_put_log_from_ptr(nu_ibuf_t*, nu_logger_t*);
PUBLIC nu_bool_t nu_ibuf_dump(nu_ibuf_t*, FILE*);

////////////////////////////////////////////////////////////////
//
// obuf
//

PUBLIC nu_obuf_t* nu_obuf_create(void);
PUBLIC nu_obuf_t* nu_obuf_create_with_obuf(const nu_obuf_t* src);
PUBLIC nu_obuf_t* nu_obuf_create_with_bytes(
        const uint8_t* data, const size_t len);
PUBLIC nu_obuf_t* nu_obuf_create_with_capa(const size_t capa);
PUBLIC nu_obuf_t* nu_obuf_create_with_capa_and_grow(
        const size_t capa, const size_t grow);
PUBLIC nu_obuf_t* nu_obuf_dup(const nu_obuf_t*);
PUBLIC int nu_obuf_cmp(const nu_obuf_t*, const nu_obuf_t*);
PUBLIC void nu_obuf_free(nu_obuf_t*);

PUBLIC void nu_obuf_grow(nu_obuf_t*);

PUBLIC void nu_obuf_put_log(const nu_obuf_t*, nu_logger_t*);
PUBLIC nu_bool_t nu_obuf_dump(nu_obuf_t*, FILE*);
PUBLIC nu_bool_t nu_obuf_restore(nu_obuf_t*, FILE*);

////////////////////////////////////////////////////////////////

/** Returns the remaining data size.
 *
 * @param self
 * @return the remaining data size
 */
PUBLIC_INLINE size_t
nu_ibuf_remain(const nu_ibuf_t* self)
{
    if (self->len < self->head - self->data)
        return 0;
    return self->len - (self->head - self->data);
}

/** Returns the current pointer.
 *
 * @param self
 * @return the current pointer.
 */
PUBLIC_INLINE size_t
nu_ibuf_ptr(const nu_ibuf_t* self)
{
    return self->head - self->data;
}

/** Resets the pointer.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_ibuf_reset_ptr(nu_ibuf_t* self)
{
    self->head = self->data;
}

/** Skips the padding.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_ibuf_skip_padding(nu_ibuf_t* self)
{
    uintptr_t p = (uintptr_t)self->head;
    if (p % 4 != 0)
        self->head += 4 - (p % 4);
}

/** Reads 1byte unsigned int.
 *
 * @param self
 * @param[out] r    result
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_ibuf_get_u8(nu_ibuf_t* self, uint8_t* r)
{
    if (sizeof(uint8_t) > nu_ibuf_remain(self))
        return false;
    *r = *(uint8_t*)(self->head);
    self->head += sizeof(uint8_t);
    return true;
}

/** Reads 2bytes unsigned int.
 *
 * @param self
 * @param[out] r    result
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_ibuf_get_u16(nu_ibuf_t* self, uint16_t* r)
{
    uint16_t ndt;
    uint8_t* pdt = (uint8_t*)&ndt;
    if (sizeof(uint16_t) > nu_ibuf_remain(self))
        return false;
    *pdt++ = *self->head++;
    *pdt = *self->head++;
    *r = nu_ntohs(ndt);
    return true;
}

/** Reads 4bytes unsigned int.
 *
 * @param self
 * @param[out] r    result
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_ibuf_get_u32(nu_ibuf_t* self, uint32_t* r)
{
    uint32_t ndt;
    uint8_t* pdt = (uint8_t*)&ndt;
    if (sizeof(uint32_t) > nu_ibuf_remain(self))
        return false;
    *pdt++ = *self->head++;
    *pdt++ = *self->head++;
    *pdt++ = *self->head++;
    *pdt = *self->head++;
    *r = nu_ntohl(ndt);
    return true;
}

/** Reads byte array.
 *
 * @param self
 * @param[out] r    result
 * @param len       length of byte array
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_ibuf_get_bytes(nu_ibuf_t* self, uint8_t* r, size_t len)
{
    if (nu_ibuf_remain(self) < len)
        return false;
    memcpy(r, self->head, len);
    self->head += len;
    return true;
}

/** Reads IPv4 address.
 *
 * @param self
 * @param[out] r    result
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_ibuf_get_ip4(nu_ibuf_t* self, nu_ip_t* r)
{
    nu_ip_addr_t data;
    int ret = nu_ibuf_get_bytes(self, (uint8_t*)&data, NU_IP4_LEN);
    *r = nu_ip4(&data);
    return static_cast<nu_bool_t>(ret);
}

/** Reads IPv6 address.
 *
 * @param self
 * @param[out] r    result
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_ibuf_get_ip6(nu_ibuf_t* self, nu_ip_t* r)
{
    nu_ip_addr_t data;
    int ret = nu_ibuf_get_bytes(self, (uint8_t*)&data, NU_IP6_LEN);
    *r = nu_ip6(&data);
    return static_cast<nu_bool_t>(ret);
}

/** Peeks 1byte unsigned int.
 *
 * This method reads 1byte unsigned integer without incrementing
 * the read pointer.
 *
 * @param self
 * @param[out] r    result
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_ibuf_peek_u8(nu_ibuf_t* self, uint8_t* r)
{
    if (sizeof(uint8_t) > nu_ibuf_remain(self))
        return false;
    *r = *(uint8_t*)(self->head);
    return true;
}

/** Peeks 2byte unsigned int.
 *
 * This method reads 2byte unsigned integer without incrementing
 * the read pointer.
 *
 * @param self
 * @param[out] r    result
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_ibuf_peek_u16(nu_ibuf_t* self, uint16_t* r)
{
    uint16_t ndt;
    uint8_t* pdt = (uint8_t*)&ndt;

    if (sizeof(uint16_t) > nu_ibuf_remain(self))
        return false;
    *pdt++ = *self->head;
    *pdt = *self->head + 1;
    *r = nu_ntohs(ndt);
    return true;
}

/** Peeks 4bytes unsigned int.
 *
 * This method reads 4byte unsigned integer without incrementing
 * the read pointer.
 *
 * @param self
 * @param[out] r    result
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_ibuf_peek_u32(nu_ibuf_t* self, uint32_t* r)
{
    uint32_t ndt;
    uint8_t* pdt = (uint8_t*)&ndt;
    if (sizeof(uint32_t) > nu_ibuf_remain(self))
        return false;
    *pdt++ = *self->head;
    *pdt++ = *self->head + 1;
    *pdt++ = *self->head + 2;
    *pdt = *self->head + 3;
    *r = nu_ntohl(ndt);
    return true;
}

/** Peeks byte array.
 *
 * This method reads byte array without incrementing
 * the read pointer.
 *
 * @param self
 * @param[out] r    result
 * @param len       length
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_ibuf_peek_bytes(nu_ibuf_t* self, uint8_t* r, size_t len)
{
    if (nu_ibuf_remain(self) < len)
        return false;
    memcpy(r, self->head, len);
    return true;
}

/** Peeks IPv4 address.
 *
 * This method reads IPv4 address without incrementing
 * the read pointer.
 *
 * @param self
 * @param[out] r    result
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_ibuf_peek_ip4(nu_ibuf_t* self, nu_ip_t* r)
{
    nu_ip_addr_t data;
    int ret = nu_ibuf_peek_bytes(self, (uint8_t*)&data, NU_IP4_LEN);
    *r = nu_ip4(&data);
    return static_cast<nu_bool_t>(ret);
}

/** Peeks IPv6 address.
 *
 * This method reads IPv6 address without incrementing
 * the read pointer.
 *
 * @param self
 * @param[out] r    result
 * @return true if success
 */
PUBLIC_INLINE nu_bool_t
nu_ibuf_peek_ip6(nu_ibuf_t* self, nu_ip_t* r)
{
    nu_ip_addr_t data;
    int ret = nu_ibuf_peek_bytes(self, (uint8_t*)&data, NU_IP6_LEN);
    *r = nu_ip6(&data);
    return static_cast<nu_bool_t>(ret);
}

/** Clears buffer.
 *
 * @param self
 */
PUBLIC_INLINE void
nu_obuf_clear(nu_obuf_t* self)
{
    self->len = 0;
}

/** Gets the current pointer.
 *
 * @param self
 * @return the pointer
 */
PUBLIC_INLINE size_t
nu_obuf_ptr(const nu_obuf_t* self)
{
    return self->len;
}

/** Gets the size.
 *
 * @param self
 * @return the size of data.
 */
PUBLIC_INLINE size_t
nu_obuf_size(const nu_obuf_t* self)
{
    return self->len;
}

/** Appends 1byte unsigned int.
 *
 * @param self
 * @param dt    data for appending
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_append_u8(nu_obuf_t* self, const uint8_t dt)
{
    while (self->len + 1 > self->capa)
        nu_obuf_grow(self);
    self->data[self->len++] = dt;
    return true;
}

/** Appends 2byte unsigned int.
 *
 * @param self
 * @param dt    data for appending
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_append_u16(nu_obuf_t* self, const uint16_t dt)
{
    uint16_t ndt = nu_htons(dt);
    uint8_t* pdt = (uint8_t*)&ndt;
    while (self->len + 2 > self->capa)
        nu_obuf_grow(self);
    self->data[self->len++] = *pdt++;
    self->data[self->len++] = *pdt;
    return true;
}

/** Appends 4byte unsigned int.
 *
 * @param self
 * @param dt    data for appending
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_append_u32(nu_obuf_t* self, const uint32_t dt)
{
    uint32_t ndt = nu_htonl(dt);
    uint8_t* pdt = (uint8_t*)&ndt;
    while (self->len + 4 > self->capa)
        nu_obuf_grow(self);
    self->data[self->len++] = *pdt++;
    self->data[self->len++] = *pdt++;
    self->data[self->len++] = *pdt++;
    self->data[self->len++] = *pdt;
    return true;
}

/** Appends byte array.
 *
 * @param self
 * @param dt    data for appending
 * @param sz    size of dt
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_append_bytes(nu_obuf_t* self,
        const uint8_t* dt, const size_t sz)
{
    while (self->len + sz > self->capa)
        nu_obuf_grow(self);
    memcpy(&self->data[self->len], dt, sz);
    self->len += sz;
    return true;
}

/** Appends IPv4 address.
 *
 * @param self
 * @param ip    ip for appending
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_append_ip4(nu_obuf_t* self, const nu_ip_t ip)
{
    assert(nu_ip_is_v4(ip));
    return nu_obuf_append_bytes(self,
            (const uint8_t*)nu_ip_addr(ip), NU_IP4_LEN);
}

/**
 * Appends IPv6 address.
 *
 * @param self
 * @param ip    ip for appending
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_append_ip6(nu_obuf_t* self, const nu_ip_t ip)
{
    assert(nu_ip_is_v6(ip));
    return nu_obuf_append_bytes(self,
            (const uint8_t*)nu_ip_addr(ip), NU_IP6_LEN);
}

/** Appends obuf.
 *
 * @param self
 * @param src   obuf for appending
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_append_obuf(nu_obuf_t* self, const nu_obuf_t* src)
{
    return nu_obuf_append_bytes(self, src->data, src->len);
}

/** Appends ibuf.
 *
 * @param self
 * @param src   ibuf for appending
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_append_ibuf(nu_obuf_t* self, const nu_ibuf_t* src)
{
    return nu_obuf_append_bytes(self, src->data, src->len);
}

/** Appends padding.
 *
 * @param self
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_append_padding(nu_obuf_t* self)
{
    while (self->len % 4 != 0) {
        if (!nu_obuf_append_u8(self, 0))
            return false;
    }
    return true;
}

/** Puts 1byte unsigned int.
 *
 * @param self
 * @param idx   point to put
 * @param dt    data
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_put_u8(nu_obuf_t* self, const size_t idx, const uint8_t dt)
{
    while (idx + sizeof(uint8_t) > self->capa)
        nu_obuf_grow(self);
    self->data[idx] = dt;
    return true;
}

/** Puts 2bytes unsigned int
 *
 * @param self
 * @param idx   point to put
 * @param dt    data
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_put_u16(nu_obuf_t* self, const size_t idx, const uint16_t dt)
{
    while (idx + sizeof(uint16_t) > self->capa)
        nu_obuf_grow(self);
    uint16_t ndt = nu_htons(dt);
    uint8_t* pdt = (uint8_t*)&ndt;
    self->data[idx] = *pdt++;
    self->data[idx + 1] = *pdt;
    return true;
}

/** Puts 4bytes unsigned int
 *
 * @param self
 * @param idx   point to put
 * @param dt    data
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_put_u32(nu_obuf_t* self, const size_t idx, const uint32_t dt)
{
    while (idx + sizeof(uint32_t) > self->capa)
        nu_obuf_grow(self);
    uint32_t ndt = nu_htonl(dt);
    uint8_t* pdt = (uint8_t*)&ndt;
    self->data[idx] = *pdt++;
    self->data[idx + 1] = *pdt++;
    self->data[idx + 2] = *pdt++;
    self->data[idx + 3] = *pdt;
    return true;
}

/** Puts byte array
 *
 * @param self
 * @param idx   point to put
 * @param dt    data
 * @param sz    size of dt
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_put_bytes(nu_obuf_t* self, const size_t idx,
        const uint8_t* dt, const size_t sz)
{
    while (idx + sz > self->capa)
        nu_obuf_grow(self);
    memcpy(&self->data[idx], dt, sz);
    self->len += sz;
    return true;
}

/** Puts IPv4 address
 *
 * @param self
 * @param idx   point to put
 * @param ip    ip address
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_put_ip4(nu_obuf_t* self, size_t idx, const nu_ip_t ip)
{
    assert(nu_ip_is_v4(ip));
    return nu_obuf_put_bytes(self, idx,
            (const uint8_t*)nu_ip_addr(ip), NU_IP4_LEN);
}

/** Puts IPv6 address
 *
 * @param self
 * @param idx   point to put
 * @param ip    ip address
 * @return ture if success
 */
PUBLIC_INLINE nu_bool_t
nu_obuf_put_ip6(nu_obuf_t* self, size_t idx, const nu_ip_t ip)
{
    assert(nu_ip_is_v6(ip));
    return nu_obuf_put_bytes(self, idx,
            (const uint8_t*)nu_ip_addr(ip), NU_IP6_LEN);
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
