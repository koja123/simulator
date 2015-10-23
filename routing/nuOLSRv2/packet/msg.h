//
// MANET message
//
#ifndef NU_PACKET_MSG_H_
#define NU_PACKET_MSG_H_

#include "core/buf.h"
#include "packet/tlv_set.h"
#include "packet/atb_list.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_msg Packet :: MANET message
 * @{
 */

/**
 * MANET Message
 */
typedef struct nu_msg {
    uint8_t       type;          ///< message type
    uint8_t       flags;         ///< message flags
    uint8_t       addr_len;      ///< (IPv4:4 / IPv6:16)
    uint16_t      size;          ///< message size
    nu_ip_t       orig_addr;     ///< originator address
    uint8_t       hop_limit;     ///< hop limit
    uint8_t       hop_count;     ///< hop count
    uint16_t      seqnum;        ///< sequence number

    uint16_t      body_size;     ///< message body size
    nu_bool_t     body_decoded;  ///< true:message body is decoded
    nu_ibuf_t*    body_ibuf;     ///< encoded message body

    nu_tlv_set_t  msg_tlv_set;   ///< message tlvs
    nu_atb_list_t atb_list;      ///< list of address tlv block
    nu_atb_t*     cur_atb;       ///< pointer to address tlv block
} nu_msg_t;

PUBLIC void nu_msg_init(nu_msg_t*);
PUBLIC void nu_msg_destroy(nu_msg_t*);

PUBLIC nu_msg_t* nu_msg_create(void);
PUBLIC void nu_msg_free(nu_msg_t*);

PUBLIC void nu_msg_set_orig_addr(nu_msg_t*, const nu_ip_t);
PUBLIC void nu_msg_set_hop_limit(nu_msg_t*, const uint8_t);
PUBLIC void nu_msg_set_hop_count(nu_msg_t*, const uint8_t);
PUBLIC void nu_msg_set_seqnum(nu_msg_t*, const uint16_t);

PUBLIC nu_bool_t nu_msg_has_orig_addr(const nu_msg_t*);
PUBLIC nu_bool_t nu_msg_has_hop_limit(const nu_msg_t*);
PUBLIC nu_bool_t nu_msg_has_hop_count(const nu_msg_t*);
PUBLIC nu_bool_t nu_msg_has_seqnum(const nu_msg_t*);

PUBLIC nu_tlv_t* nu_msg_add_msg_tlv(nu_msg_t*, nu_tlv_t*);

PUBLIC nu_atb_t* nu_msg_add_atb(nu_msg_t*);

PUBLIC void nu_msg_add_ip(nu_msg_t*, const nu_ip_t);
PUBLIC void nu_msg_add_ip_tlv(nu_msg_t*, const nu_ip_t, nu_tlv_t*);
PUBLIC void nu_msg_remove_ip(nu_msg_t*, const nu_ip_t);

PUBLIC nu_atb_elt_t* nu_msg_search_ip(nu_msg_t*,
        const nu_ip_t);
PUBLIC nu_tlv_t* nu_msg_search_ip_tlv(nu_msg_t*,
        const nu_ip_t, const uint8_t type);

PUBLIC nu_bool_t nu_msg_eq(const nu_msg_t*, const nu_msg_t*);

PUBLIC void nu_msg_put_log(const nu_msg_t*, nu_logger_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
