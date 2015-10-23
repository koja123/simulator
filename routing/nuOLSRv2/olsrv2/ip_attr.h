#ifndef OLSRV2_IP_ATTR_H__
#define OLSRV2_IP_ATTR_H__

#include "config.h" //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/**
 * Additional information of ip.
 */
typedef struct olsrv2_ip_attr {
    tuple_r_t* tuple_r_dest;          ///< dest tuple in ibase_r.
#ifndef NUOLSRV2_NOSTAT
    uint16_t   last_seqnum;           ///< last seqnum of packet from ip
    uint32_t   nhdpHelloMessageRecvd; ///< nhdpHelloMessageRecvd
    uint32_t   nhdpHelloMessageLost;  ///< nhdpHelloMessageLost
#endif
} olsrv2_ip_attr_t;

PUBLIC void olsrv2_ip_attr_init(void);
PUBLIC void olsrv2_ip_attr_destroy(void);
PUBLIC olsrv2_ip_attr_t* olsrv2_ip_attr_get(const nu_ip_t);

}//namespace// //ScenSim-Port://

#endif
