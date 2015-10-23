#ifndef NU_PACKET_MSGFLAGS_H_
#define NU_PACKET_MSGFLAGS_H_

#include "packet/bit.h"


namespace NuOLSRv2Port { //ScenSim-Port://

enum nu_msgflags {
    M_ORIG      = 3,
    M_HOP_LIMIT = 2,
    M_HOP_COUNT = 1,
    M_SEQNUM    = 0,
};

#define nu_msgflags_set_orig_addr(s)       bit_set(s, M_ORIG)
#define nu_msgflags_set_no_orig_addr(s)    bit_clr(s, M_ORIG)
#define nu_msgflags_set_hop_limit(s)       bit_set(s, M_HOP_LIMIT)
#define nu_msgflags_set_no_hop_limit(s)    bit_clr(s, M_HOP_LIMIT)
#define nu_msgflags_set_hop_count(s)       bit_set(s, M_HOP_COUNT)
#define nu_msgflags_set_no_hop_count(s)    bit_clr(s, M_HOP_COUNT)
#define nu_msgflags_set_seqnum(s)          bit_set(s, M_SEQNUM)
#define nu_msgflags_set_no_seqnum(s)       bit_clr(s, M_SEQNUM)

/**
 */
static inline void
msgflags_to_s(const uint8_t s, char* buf, size_t bufsz)
{
    snprintf(buf, bufsz, "0x%02x(%s:%s:%s:%s)", s,
            (bit_is_set(s, M_ORIG)) ? "orig" : "",
            (bit_is_set(s, M_HOP_LIMIT)) ? "hop-limit" : "",
            (bit_is_set(s, M_HOP_COUNT)) ? "hop-count" : "",
            (bit_is_set(s, M_SEQNUM)) ? "seqnum" : ""
            );
}

}//namespace// //ScenSim-Port://

#endif
