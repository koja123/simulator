#ifndef NU_PACKET_TLVFLAGS_H_
#define NU_PACKET_TLVFLAGS_H_

#include "packet/bit.h"


namespace NuOLSRv2Port { //ScenSim-Port://

enum tlvflags {
    T_TYPE_EXT     = 7,
    T_SINGLE_INDEX = 6,
    T_MULTI_INDEX  = 5,
    T_VALUE        = 4,
    T_EXT_LEN      = 3,
    T_MULTI_VALUE  = 2
};

#define tlvflags_set_type_ext(s)       bit_set(s, T_TYPE_EXT)
#define tlvflags_set_no_type_ext(s)    bit_clr(s, T_TYPE_EXT)

#define tlvflags_set_8bit_length(s) \
    do {                            \
        bit_clr(s, T_EXT_LEN);      \
        bit_set(s, T_VALUE); }      \
    while (0)
#define tlvflags_set_16bit_length(s) \
    do {                             \
        bit_set(s, T_EXT_LEN);       \
        bit_set(s, T_VALUE); }       \
    while (0)
#define tlvflags_set_no_value(s) \
    do {                         \
        bit_clr(s, T_EXT_LEN);   \
        bit_clr(s, T_VALUE); }   \
    while (0)

#define tlvflags_set_no_index(s)     \
    do {                             \
        bit_clr(s, T_SINGLE_INDEX);  \
        bit_clr(s, T_MULTI_INDEX); } \
    while (0)
#define tlvflags_set_single_index(s) \
    do {                             \
        bit_set(s, T_SINGLE_INDEX);  \
        bit_clr(s, T_MULTI_INDEX); } \
    while (0)
#define tlvflags_set_multi_index(s)  \
    do {                             \
        bit_clr(s, T_SINGLE_INDEX);  \
        bit_set(s, T_MULTI_INDEX); } \
    while (0)

#define tlvflags_set_single_value(s) \
    do {                             \
        bit_set(s, T_VALUE);         \
        bit_clr(s, T_MULTI_VALUE); } \
    while (0)
#define tlvflags_set_multi_value(s)  \
    do {                             \
        bit_set(s, T_VALUE);         \
        bit_set(s, T_MULTI_VALUE); } \
    while (0)

#define tlvflags_has_type_ext(s)        (bit_is_set(s, T_TYPE_EXT))
#define tlvflags_has_8bit_length(s)     (bit_is_clr(s, T_EXT_LEN))
#define tlvflags_has_16bit_length(s)    (bit_is_set(s, T_EXT_LEN))
#define tlvflags_has_no_value(s)        (bit_is_clr(s, T_VALUE))
#define tlvflags_has_value(s)           (bit_is_set(s, T_VALUE))

#define tlvflags_has_no_index(s) \
    (bit_is_clr(s, T_SINGLE_INDEX) && bit_is_clr(s, T_MULTI_INDEX))
#define tlvflags_has_index(s) \
    (bit_is_set(s, T_SINGLE_INDEX) || bit_is_set(s, T_MULTI_INDEX))
#define tlvflags_has_single_index(s) \
    (bit_is_set(s, T_SINGLE_INDEX) && bit_is_clr(s, T_MULTI_INDEX))
#define tlvflags_has_multi_index(s) \
    (bit_is_clr(s, T_SINGLE_INDEX) && bit_is_set(s, T_MULTI_INDEX))

#define tlvflags_has_single_value(s) \
    (bit_is_set(s, T_VALUE) && bit_is_clr(s, T_MULTI_VALUE))
#define tlvflags_has_multi_value(s) \
    (bit_is_set(s, T_VALUE) && bit_is_set(s, T_MULTI_VALUE))

/**
 */
static inline void
tlvflags_to_s(const uint8_t s, char* buf, size_t bufsz)
{
    snprintf(buf, bufsz, "0x%02x(%s:%s:%s:%s:%s:%s)",
            s,
            (bit_is_set(s, T_TYPE_EXT)) ? "type-ext" : "",
            (bit_is_set(s, T_SINGLE_INDEX)) ? "single-index" : "",
            (bit_is_set(s, T_MULTI_INDEX)) ? "multi-index" : "",
            (bit_is_set(s, T_VALUE)) ? "value" : "",
            (bit_is_set(s, T_EXT_LEN)) ? "ext-len" : "",
            (bit_is_set(s, T_MULTI_VALUE)) ? "multi-value" : "");
}

}//namespace// //ScenSim-Port://

#endif
