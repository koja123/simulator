#ifndef NU_PACKET_ADDRFLAGS_H_
#define NU_PACKET_ADDRFLAGS_H_

#include "packet/bit.h"


namespace NuOLSRv2Port { //ScenSim-Port://

enum addrflags {
    A_HEAD          = 7,
    A_FULL_TAIL     = 6,
    A_ZERO_TAIL     = 5,
    A_SINGLE_PRELEN = 4,
    A_MULTI_PRELEN  = 3
};

#define addrflags_set_head(s)            bit_set(s, A_HEAD)
#define addrflags_set_no_head(s)         bit_clr(s, A_HEAD)
#define addrflags_set_full_tail(s)       bit_set(s, A_FULL_TAIL)
#define addrflags_set_no_full_tail(s)    bit_clr(s, A_FULL_TAIL)
#define addrflags_set_zero_tail(s)       bit_set(s, A_ZERO_TAIL)
#define addrflags_set_no_zero_tail(s)    bit_clr(s, A_ZERO_TAIL)

#define addrflags_set_no_prelen(s)    \
    do {                              \
        bit_clr(s, A_SINGLE_PRELEN);  \
        bit_clr(s, A_MULTI_PRELEN); } \
    while (0)

#define addrflags_set_single_prelen(s) \
    do {                               \
        bit_set(s, A_SINGLE_PRELEN);   \
        bit_clr(s, A_MULTI_PRELEN); }  \
    while (0)

#define addrflags_set_multi_prelen(s) \
    do {                              \
        bit_clr(s, A_SINGLE_PRELEN);  \
        bit_set(s, A_MULTI_PRELEN); } \
    while (0)

#define addrflags_has_no_head(s)    bit_is_clr(s, A_HEAD)
#define addrflags_has_head(s)       bit_is_set(s, A_HEAD)

#define addrflags_has_no_tail(s) \
    (bit_is_clr(s, A_FULL_TAIL) && bit_is_clr(s, A_ZERO_TAIL))
#define addrflags_has_tail(s) \
    (bit_is_set(s, A_FULL_TAIL) || bit_is_set(s, A_ZERO_TAIL))
#define addrflags_has_full_tail(s) \
    (bit_is_set(s, A_FULL_TAIL) && bit_is_clr(s, A_ZERO_TAIL))
#define addrflags_has_zero_tail(s) \
    (bit_is_clr(s, A_FULL_TAIL) && bit_is_set(s, A_ZERO_TAIL))

#define addrflags_has_no_prelen(s) \
    (bit_is_clr(s, A_SINGLE_PRELEN) && bit_is_clr(s, A_MULTI_PRELEN))
#define addrflags_has_single_prelen(s) \
    (bit_is_set(s, A_SINGLE_PRELEN) && bit_is_clr(s, A_MULTI_PRELEN))
#define addrflags_has_multi_prelen(s) \
    (bit_is_clr(s, A_SINGLE_PRELEN) && bit_is_set(s, A_MULTI_PRELEN))

/**
 */
static inline void
addrflags_to_s(const uint8_t s, char* buf, size_t bufsz)
{
    snprintf(buf, bufsz,
            "0x%02x(%s:%s:%s:%s:%s)",
            s,
            (bit_is_set(s, A_HEAD)) ? "head" : "",
            (bit_is_set(s, A_FULL_TAIL)) ? "full-tail" : "",
            (bit_is_set(s, A_ZERO_TAIL)) ? "zero-tail" : "",
            (bit_is_set(s, A_SINGLE_PRELEN)) ? "single-prelen" : "",
            (bit_is_set(s, A_MULTI_PRELEN)) ? "multi-prelen" : "");
}

}//namespace// //ScenSim-Port://

#endif
