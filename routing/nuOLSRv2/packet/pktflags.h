#ifndef NU_PACKET_PKTFLAGS_H_
#define NU_PACKET_PKTFLAGS_H_

#include "packet/bit.h"


namespace NuOLSRv2Port { //ScenSim-Port://

enum pktflags {
    P_SEQNUM = 3,
    P_TLV    = 2,
};

#define nu_pktflags_set_seqnum(s)       bit_set(s, P_SEQNUM)
#define nu_pktflags_set_no_seqnum(s)    bit_clr(s, P_SEQNUM)
#define nu_pktflags_set_tlv(s)          bit_set(s, P_TLV)
#define nu_pktflags_set_no_tlv(s)       bit_clr(s, P_TLV)

/**
 */
static inline void
pktflags_to_s(const uint8_t s, char* buf, size_t bufsz)
{
    snprintf(buf, bufsz, "0x%02x(%s:%s)",
            s,
            (bit_is_set(s, P_SEQNUM)) ? "seqnum" : "",
            (bit_is_set(s, P_TLV)) ? "tlv" : "");
}

}//namespace// //ScenSim-Port://

#endif
