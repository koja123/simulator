#include "config.h"

#include <stddef.h>

#include "core/core.h"
#include "packet/packet.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_packet
 * @{
 */

#ifndef nuOLSRv2_ALL_STATIC
nu_packet_t* current_packet = NULL;
#endif

/**
 * Log flags
 */
/* *INDENT-OFF* */
static nu_log_flag_t nu_packet_log_flags[] =
{
    { "message_hexdump", "Pm", offsetof(nu_packet_t, log_message_hexdump),
      "Message hexdump" },
    { "message_packing", "Pa", offsetof(nu_packet_t, log_message_packing),
      "Message packing/unpacking algorithm" },
    { NULL },
};
/* *INDENT-ON* */

/** Creates packet module.
 *
 * @return packet
 */
PUBLIC nu_packet_t*
nu_packet_create(void)
{
    current_packet = nu_mem_alloc(nu_packet_t);
    current_packet->use_packet_seqnum  = false;
    current_packet->next_packet_seqnum = (uint16_t)(nu_rand() * 0x10000);

    current_packet->log_message_hexdump = false;
    current_packet->log_message_packing = false;

    return current_packet;
}

/** Destroys and frees the packet module.
 *
 * @param self
 */
PUBLIC void
nu_packet_free(nu_packet_t* self)
{
    nu_mem_free(self);
}

/** Gets next sequence number.
 *
 * @return sequence number
 */
PUBLIC uint16_t
nu_packet_next_seqnum(void)
{
    return current_packet->next_packet_seqnum++;
}

/** Outpus parameter list.
 *
 * @param prefix
 * @param fp
 */
PUBLIC void
nu_packet_put_params(const char* prefix, FILE* fp)
{
    nu_put_log_flags(prefix, nu_packet_log_flags, fp);
}

/** Sets parameter to packet module.
 *
 * @param opt
 */
PUBLIC nu_bool_t
nu_packet_set_param(const char* opt)
{
    return nu_set_log_flag(opt, nu_packet_log_flags, current_packet);
}

/** @} */

}//namespace// //ScenSim-Port://
