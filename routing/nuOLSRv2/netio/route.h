//
// Routing table interface
//
#ifndef NU_NETIO_ROUTE_H_
#define NU_NETIO_ROUTE_H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_netio
 * @{
 */

PUBLIC nu_bool_t nu_route_add(const nu_ip_t dest, const nu_ip_t next,
        const nu_ip_t local, const uint8_t hop, const char* ifname);
PUBLIC nu_bool_t nu_route_delete(const nu_ip_t dest, const nu_ip_t next,
        const nu_ip_t local, const uint8_t hop, const char* ifname);

/** @} */

}//namespace// //ScenSim-Port://

#endif
