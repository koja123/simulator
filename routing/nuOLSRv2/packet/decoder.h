//
// MANET packet decoder
//
#ifndef NU_PACKET_DECODER_H_
#define NU_PACKET_DECODER_H_

#include "core/strbuf.h"
#include "core/buf.h"
#include "packet/pkthdr.h"
#include "packet/msg.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_encoder_decoder Packet :: MANET message encoder / decoder
 * @{
 */

/**
 * MANET message decoder
 */
typedef struct nu_decoder {
    uint8_t     type; ///< source packet's ip type (see nu_ip_t)
    nu_strbuf_t err;  ///< error message
    nu_ibuf_t*  buf;  ///< encoded message
} nu_decoder_t;

PUBLIC void nu_decoder_init(nu_decoder_t*);
PUBLIC void nu_decoder_destroy(nu_decoder_t*);

PUBLIC void nu_decoder_set_ip_type(nu_decoder_t*, uint8_t type);

PUBLIC nu_pkthdr_t* nu_decoder_decode_pkthdr(nu_decoder_t*, nu_ibuf_t*);
PUBLIC nu_msg_t* nu_decoder_decode_message(nu_decoder_t*, nu_ibuf_t*);
PUBLIC nu_msg_t* nu_decoder_decode_msghdr(nu_decoder_t*, nu_ibuf_t*);
PUBLIC nu_bool_t nu_decoder_decode_msg_body(nu_decoder_t*, nu_msg_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
