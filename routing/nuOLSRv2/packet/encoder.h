//
// MANET packet encoder
//
#ifndef NU_PACKET_ENCODER_H_
#define NU_PACKET_ENCODER_H_

#include "core/buf.h"
#include "core/strbuf.h"
#include "packet/pkthdr.h"
#include "packet/msg.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_encoder_decoder
 * @{
 */

/**
 * MANET message encoder
 */
typedef struct nu_encoder {
    nu_obuf_t*  buf;  ///< encoded message buffer
    nu_strbuf_t err;  ///< error message
} nu_encoder_t;

PUBLIC void nu_encoder_init(nu_encoder_t*);
PUBLIC void nu_encoder_destroy(nu_encoder_t*);

PUBLIC nu_obuf_t* nu_encoder_encode_pkthdr(nu_encoder_t*,
        nu_pkthdr_t*);
PUBLIC nu_obuf_t* nu_encoder_encode_message(nu_encoder_t*,
        nu_msg_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
