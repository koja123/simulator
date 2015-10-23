#include "config.h"

#include <math.h>

#include "packet/pkthdr.h"
#include "packet/pktflags.h"
#include "packet/packet.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_pkthdr
 * @{
 */

/** Checks whether pkthdr contains sequence number.
 *
 * @param self
 */
PUBLIC nu_bool_t
nu_pkthdr_has_seqnum(nu_pkthdr_t* self)
{
    return bit_is_set(self->flags, P_SEQNUM);
}

/** Checks whether pkthdr contains tlv.
 *
 * @return true if pkthdr contains tlv.
 */
PUBLIC nu_bool_t
nu_pkthdr_has_tlv(nu_pkthdr_t* self)
{
    return bit_is_set(self->flags, P_TLV);
}

/** Initializes pkthdr.
 *
 * @param self
 */
PUBLIC void
nu_pkthdr_init(nu_pkthdr_t* self)
{
    self->version = 0;
    self->flags = 0;
    if (current_packet->use_packet_seqnum) {
        nu_pkthdr_set_seqnum(self, nu_packet_next_seqnum());
    } else {
        nu_pktflags_set_no_seqnum(self->flags);
        self->seqnum = 0;
    }
    nu_pktflags_set_no_tlv(self->flags);
    nu_tlv_set_init(&self->tlv_set);
}

/** Creates pkthdr.
 *
 * @return pkthdr
 */
PUBLIC nu_pkthdr_t*
nu_pkthdr_create(void)
{
    nu_pkthdr_t* self = nu_mem_alloc(nu_pkthdr_t);
    nu_pkthdr_init(self);
    return self;
}

/** Destroys pkthdr.
 *
 * @param self
 */
PUBLIC void
nu_pkthdr_destroy(nu_pkthdr_t* self)
{
    nu_tlv_set_remove_all(&self->tlv_set);
}

/** Destroys and frees pkthdr.
 *
 * @param self
 */
PUBLIC void
nu_pkthdr_free(nu_pkthdr_t* self)
{
    nu_pkthdr_destroy(self);
    nu_mem_free(self);
}

/** Sets version
 *
 * @param self
 * @param ver
 */
PUBLIC void
nu_pkthdr_set_version(nu_pkthdr_t* self, uint8_t ver)
{
    self->version = ver;
}

/** Sets sequence number
 *
 * @param self
 * @param seqnum
 */
PUBLIC void
nu_pkthdr_set_seqnum(nu_pkthdr_t* self, uint16_t seqnum)
{
    self->seqnum = seqnum;
    nu_pktflags_set_seqnum(self->flags);
}

/** Adds tlv.
 *
 * @param self
 * @param tlv
 */
PUBLIC void
nu_pkthdr_add_tlv(nu_pkthdr_t* self, nu_tlv_t* tlv)
{
    nu_tlv_set_add(&self->tlv_set, tlv);
    nu_pktflags_set_tlv(self->flags);
}

/** Outputs packet header.
 *
 * @param self
 * @param logger
 */
PUBLIC void
nu_pkthdr_put_log(nu_pkthdr_t* self, nu_logger_t* logger)
{
    nu_logger_log(logger, "pkthdr:[ver:%d", self->version);
    nu_logger_push_prefix(logger, PACKET_LOG_INDENT);
    if (nu_pkthdr_has_seqnum(self))
        nu_logger_log(logger, "seqnum:%hd", self->seqnum);
    if (nu_pkthdr_has_tlv(self))
        nu_tlv_set_put_log(&self->tlv_set, manet_pkt_tlv, logger);
    nu_logger_log(logger, "]");
    nu_logger_pop_prefix(logger);
}

/** @} */

}//namespace// //ScenSim-Port://
