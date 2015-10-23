//
// Packet module variables
//
#ifndef NU_PACKET_H_
#define NU_PACKET_H_

#include "config.h" //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_packet Packet :: Packet module
 * @{
 */

/**
 * Packet module variables
 */
typedef struct nu_packet {
    nu_bool_t use_packet_seqnum;     ///< true:add packet seqnum
    uint16_t  next_packet_seqnum;    ///< next packet sequence number

    nu_bool_t log_message_hexdump;   ///< log flag for dumping message
    nu_bool_t log_message_packing;   ///< log flag for packing algorithm
} nu_packet_t;

#ifndef NU_NDEBUG

/** @fn NU_PACKET_DO_LOG(flag, x)
 *
 * Output debug log
 *
 * @param flag
 * @param x
 */
#define NU_PACKET_DO_LOG(flag, x)       \
    if (current_packet->log_ ## flag && \
        nu_logger_set_prio(NU_LOGGER, LOG_DEBUG)) { x }
/** LOG INDENT */
#define PACKET_LOG_INDENT    "    "
#else
#define NU_PACKET_DO_LOG(flag, x)
#define PACKET_LOG_INDENT "" //ScenSim-Port://
#endif

/**
 * Current packet module
 */
#ifdef nuOLSRv2_ALL_STATIC
static nu_packet_t* current_packet = NULL;
#else
extern nu_packet_t* current_packet;
#endif

PUBLIC nu_packet_t* nu_packet_create(void);
PUBLIC void nu_packet_free(nu_packet_t*);

PUBLIC uint16_t nu_packet_next_seqnum(void);

PUBLIC void nu_packet_put_params(const char*, FILE*);
PUBLIC nu_bool_t nu_packet_set_param(const char*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
