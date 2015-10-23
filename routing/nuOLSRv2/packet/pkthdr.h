//
// MANET packet header
//
#ifndef NU_PACKET_PKTHDR_H_
#define NU_PACKET_PKTHDR_H_

#include "packet/tlv_set.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_pkthdr Packet :: MANET packet header
 * @{
 */

/**
 * MANET packet header
 */
typedef struct nu_pkthdr {
    uint8_t      flags;   ///< packet flag
    uint8_t      version; ///< packet format version
    uint16_t     seqnum;  ///< packet sequence number
    nu_tlv_set_t tlv_set; ///< packet tlv set
} nu_pkthdr_t;

PUBLIC void nu_pkthdr_init(nu_pkthdr_t*);
PUBLIC void nu_pkthdr_destroy(nu_pkthdr_t*);

PUBLIC nu_pkthdr_t* nu_pkthdr_create(void);
PUBLIC void nu_pkthdr_free(nu_pkthdr_t*);

PUBLIC void nu_pkthdr_set_version(nu_pkthdr_t*, uint8_t);
PUBLIC void nu_pkthdr_set_seqnum(nu_pkthdr_t*, uint16_t);
PUBLIC void nu_pkthdr_add_tlv(nu_pkthdr_t*, nu_tlv_t*);

PUBLIC nu_bool_t nu_pkthdr_has_seqnum(nu_pkthdr_t*);
PUBLIC nu_bool_t nu_pkthdr_has_tlv(nu_pkthdr_t*);

PUBLIC void nu_pkthdr_put_log(nu_pkthdr_t*, nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
