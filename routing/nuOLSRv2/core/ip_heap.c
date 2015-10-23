#include "config.h"

#include "core/core.h"
#include "core/ip_inline.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_ip
 * @{
 */

#if 0

/** Gets ip_addr from the ip.
 *
 * DON'T MODIFY RETURNED IP ADDRESS.
 * If you want to modify it, use nu_ip_addr_copy().
 *
 * @param ip
 * @return ip address
 */
PUBLIC const nu_ip_addr_t*
nu_ip_addr(const nu_ip_t ip)
{
    nu_ip_util_t u;
    u.v = ip;
    //assert(!nu_ip_is_undef(ip));
    //assert(u.b[2] <= NU_IP_HEAP.last);
    return &NU_IP_HEAP.chunk[u.b[2]]->addr[u.b[3]];
}

#endif

/** Set ip attribute constructor
 *
 * @param func
 */
PUBLIC void
nu_ip_attr_set_constructor(nu_ip_attr_create_func_t func)
{
    assert(NU_IP_ATTR_CONSTRUCTOR == NULL);
    NU_IP_ATTR_CONSTRUCTOR = func;
}

/** Set ip attribute destructor
 *
 * @param func
 */
PUBLIC void
nu_ip_attr_set_destructor(nu_ip_attr_free_func_t func)
{
    assert(NU_IP_ATTR_DESTRUCTOR == NULL);
    NU_IP_ATTR_DESTRUCTOR = func;
}

/*
 */
static inline nu_ip_chunk_t*
ip_chunk_create(void)
{
    nu_ip_chunk_t* self = (nu_ip_chunk_t*)malloc(sizeof(nu_ip_chunk_t));
    self->last = -1;
    self->remain = 0x100;
    memset(self->addr, 0, sizeof(nu_ip_addr_t) * 0x100);
    memset(self->attr, 0, sizeof(void*) * 0x100);
    memset(self->status, NU_IP_TYPE_UNDEF, 0x100);
    return self;
}

/*
 */
static inline void
nu_ip_chunk_free(nu_ip_chunk_t* self)
{
    memset(self, 0xfe, sizeof(nu_ip_chunk_t));
    free(self);
}

/*
 */
static inline nu_bool_t
ip_chunk_get_empty(nu_ip_chunk_t* self)
{
    if (self->remain == 0)
        return false;
    --self->remain;
    for (self->last = self->last + 1; self->last < 0x100; ++self->last) {
        if (self->status[self->last] == NU_IP_TYPE_UNDEF)
            return true;
    }
    return false;
}

/*
 */
static inline nu_ip_addr_t*
ip_chunk_set_addr(nu_ip_chunk_t* self, const nu_ip_addr_t* addr,
        uint8_t type)
{
    assert(self->status[self->last] == NU_IP_TYPE_UNDEF);
    self->status[self->last] = type;
    nu_ip_addr_t* new_addr = &self->addr[self->last];
    memcpy(new_addr, addr, sizeof(nu_ip_addr_t));
    return new_addr;
}

/** Initializes the ip_heap.
 *
 * @param self
 */
PUBLIC void
nu_ip_heap_init(nu_ip_heap_t* self)
{
    memset(self->chunk, 0, 0x100 * sizeof(nu_ip_chunk_t*));
    self->chunk[0] = ip_chunk_create();
    self->remain = 0xff;
    self->last = 0;
    memset(self->status, 0, 0x100);
}

/** Destroys the ip_heap.
 *
 * @param self
 */
PUBLIC void
nu_ip_heap_destroy(nu_ip_heap_t* self)
{
    NU_CORE_DO_LOG(ip_heap,
            nu_logger_log(NU_LOGGER,
                    "IP_HEAP:totaly allocated chunk:%d", self->last + 1);
            );

    for (ssize_t i = 0; i <= self->last; ++i) {
        nu_ip_chunk_t* c = self->chunk[i];
        int v4 = 0;
        int v6 = 0;
        int f  = 0;
        for (size_t j = 0; j < 0x100; ++j) {
            switch (c->status[j]) {
            case NU_IP_TYPE_UNDEF:
                f += 1;
                break;
            case NU_IP_TYPE_V4:
                v4 += 1;
                break;
            case NU_IP_TYPE_V6:
                v6 += 1;
                break;
            default:
                nu_fatal("ip_chunk:unknown status == (%d)0x%02x",
                        c->status[j], c->status[j]);
            }
            if (c->attr[j] != NULL)
                NU_IP_ATTR_DESTRUCTOR(c->attr[j]);
        }
        NU_CORE_DO_LOG(ip_heap,
                nu_logger_log(NU_LOGGER,
                        "IP_HEAP:ip_chunk:%lu/ipv4:%d/ipv6:%d/free:%d",
                        (unsigned long)i, v4, v6, f);
                );
        nu_ip_chunk_free(c);
    }
}

static inline nu_ip_t
ip4_search(const nu_ip_addr_t* addr)
{
    for (int i = 0; i <= NU_IP_HEAP.last; ++i) {
        for (int j = 0; j <= NU_IP_HEAP.chunk[i]->last; ++j) {
            if (NU_IP_HEAP.chunk[i]->status[j] != NU_IP_TYPE_V4)
                continue;
            const nu_ip_addr_t* e = &NU_IP_HEAP.chunk[i]->addr[j];
            if (nu_ip4_addr_eq(e, addr)) {
                nu_ip_util_t u;
                u.b[0] = NU_IP_TYPE_V4;
                u.b[1] = 0;
                u.b[2] = (uint8_t)i;
                u.b[3] = (uint8_t)j;
                return u.v;
            }
        }
    }
    return NU_IP_UNDEF;
}

static inline nu_ip_t
ip6_search(const nu_ip_addr_t* addr)
{
    for (int i = 0; i <= NU_IP_HEAP.last; ++i) {
        for (int j = 0; j <= NU_IP_HEAP.chunk[i]->last; ++j) {
            if (NU_IP_HEAP.chunk[i]->status[j] != NU_IP_TYPE_V6)
                continue;
            const nu_ip_addr_t* e = &NU_IP_HEAP.chunk[i]->addr[j];
            if (nu_ip6_addr_eq(e, addr)) {
                nu_ip_util_t u;
                u.b[0] = NU_IP_TYPE_V6;
                u.b[1] = 0;
                u.b[2] = (uint8_t)i;
                u.b[3] = (uint8_t)j;
                return u.v;
            }
        }
    }
    return NU_IP_UNDEF;
}

/* Registers IP address.
 *
 * @param addr  IP address
 * @param af    Address family
 * @return      nu_ip_t
 */
static nu_ip_t
ip_heap_intern(const nu_ip_addr_t* addr, const short af)
{
    nu_ip_t r = (af == AF_INET) ? ip4_search(addr) : ip6_search(addr);
    if (r != NU_IP_UNDEF)
        return r;

    // Search free space
    for (; NU_IP_HEAP.last < 0xff; ++NU_IP_HEAP.last) {
        if (NU_IP_HEAP.chunk[NU_IP_HEAP.last] == NULL)
            NU_IP_HEAP.chunk[NU_IP_HEAP.last] = ip_chunk_create();
        if (ip_chunk_get_empty(NU_IP_HEAP.chunk[NU_IP_HEAP.last]))
            break;
    }
    assert(NU_IP_HEAP.last < 0xff);

    uint8_t type = (af == AF_INET) ? NU_IP_TYPE_V4 : NU_IP_TYPE_V6;
    nu_ip_chunk_t* chunk = NU_IP_HEAP.chunk[NU_IP_HEAP.last];
    ip_chunk_set_addr(chunk, addr, type);
    nu_ip_util_t u;
    u.b[0] = type;
    u.b[1] = 0;
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
    u.b[2] = (uint8_t)NU_IP_HEAP.last; //ScenSim-Port://
    u.b[3] = (uint8_t)chunk->last;     //ScenSim-Port://
#else                                  //ScenSim-Port://
    u.b[2] = NU_IP_HEAP.last;
    u.b[3] = chunk->last;
#endif                                 //ScenSim-Port://
    return u.v;
}

/**
 * (PRIVATE: DO NOT USE THIS FUNCTION)
 *
 * @param data
 * @return ip
 */
PRIVATE nu_ip_t
nu_ip4_heap_intern_(const nu_ip_addr_t* data)
{
    return ip_heap_intern(data, AF_INET);
}

/**
 * (PRIVATE: DO NOT USE THIS FUNCTION)
 *
 * @param data
 * @return ip
 */
PRIVATE nu_ip_t
nu_ip6_heap_intern_(const nu_ip_addr_t* data)
{
    return ip_heap_intern(data, AF_INET6);
}

/** @} */

}//namespace// //ScenSim-Port://
