#ifndef OLSRV2_PKT_H_
#define OLSRV2_PKT_H_

#include "core/mem.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup olsrv2_pktq OLSRv2 :: Packet queue
 * @{
 */

/**
 * Packet for send/recv queue
 */
typedef struct olsrv2_pkt {
    struct olsrv2_pkt* next; ///< next
    struct olsrv2_pkt* prev; ///< prev
    void*              buf;  ///< packet data
} olsrv2_pkt_t;

/**
 * Packet queue
 */
typedef struct olsrv2_pktq {
    olsrv2_pkt_t* next; ///< first packet
    olsrv2_pkt_t* prev; ///< last packet
    size_t        n;    ///< size
} olsrv2_pktq_t;

/**
 * Creates packet for packet queue
 *
 * @return pkt
 */
PUBLIC_INLINE olsrv2_pkt_t*
olsrv2_pkt_create(void)
{
    return nu_mem_alloc(olsrv2_pkt_t);
}

/** Destroys packet in packet queue
 *
 * @param pkt
 */
PUBLIC_INLINE void
olsrv2_pkt_free(olsrv2_pkt_t* pkt)
{
    nu_mem_free(pkt);
}

}//namespace// //ScenSim-Port://

#include "olsrv2/pktq_.h"

/** @} */

#endif
