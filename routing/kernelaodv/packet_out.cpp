/***************************************************************************
                          packet_out.c  -  description
                             -------------------
    begin                : Mon Aug 4 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#include "kernelaodvglue.h"
#include "packet_out.h"

namespace KernelAodvPort {



//ScenSim-Port://extern unsigned int g_broadcast_ip;
//ScenSim-Port://extern unsigned int g_my_ip;
extern unsigned char g_aodv_gateway;


AodvAction output_handler(
    //ScenSim-Port://unsigned int hooknum,
    const shared_ptr<ScenSim::SimulationEngineInterface>& simulationEngineInterfacePtr,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    AodvScheduleTimer* aodvScheduleTimerPtr,
    AodvFloodId* aodvFloodIdPtr,
    //ScenSim-Port://struct sk_buff **skb,
    const unsigned int sourceIpAddress,
    const unsigned int destinationIpAddress
    //ScenSim-Port://,
    //ScenSim-Port://const struct net_device *in,
    //ScenSim-Port://const struct net_device *out,
    //ScenSim-Port://int (*okfn) (struct sk_buff *)
    )
{
//ScenSim-Port://    struct iphdr *ip= (*skb)->nh.iph;
//ScenSim-Port://    struct net_device *dev= (*skb)->dev;

    aodv_route *tmp_route = new aodv_route();

//ScenSim-Port://    aodv_neigh *tmp_neigh;
//ScenSim-Port://    void *p = (uint32_t *) ip + ip->ihl;
//ScenSim-Port://    struct udphdr *udp = p; //(struct udphdr *) ip + ip->ihl;
//ScenSim-Port://    struct ethhdr *mac = (*skb)->mac.ethernet;  //Thanks to Randy Pitz for adding this extra check...
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    if (ip->daddr == g_broadcast_ip)
//ScenSim-Port://    {
//ScenSim-Port://        return NF_ACCEPT;
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    //Need this additional check, otherwise users on the
//ScenSim-Port://    //gateway will not be able to access the externel 
//ScenSim-Port://    //network. Remote user located outside the aodv_subnet
//ScenSim-Port://    //also require this check if they are access services on
//ScenSim-Port://    //the gateway node. 28 April 2004 pb 
//ScenSim-Port://    if(g_aodv_gateway && !aodv_subnet_test(ip->daddr))
//ScenSim-Port://    {
//ScenSim-Port://        return NF_ACCEPT;
//ScenSim-Port://    }
//ScenSim-Port://

    //Try to get a route to the destination

    tmp_route = aodvRoutingTablePtr->find_aodv_route(destinationIpAddress);

    if ((tmp_route == NULL) || !(tmp_route->route_valid)) {


        gen_rreq(
            simulationEngineInterfacePtr,
            kernelAodvProtocolPtr,
            aodvRoutingTablePtr,
            aodvScheduleTimerPtr,
            aodvFloodIdPtr,
            sourceIpAddress,
            destinationIpAddress);

        assert((sourceIpAddress == kernelAodvProtocolPtr->GetIpAddressLow32Bits()) &&
               "Source IP address must be itself.");

        //ScenSim-Port://return NF_QUEUE;
        return AODV_RREQ_SEND;
    }

    if ((tmp_route != NULL) && (tmp_route->route_valid)) {
        //Found after an LONG search by Jon Anderson
        if (!tmp_route->self_route) {
            tmp_route->lifetime =
                simulationEngineInterfacePtr->CurrentTime()
                    + kernelAodvProtocolPtr->GetAodvActiveRouteTimeout();
        }
    }

    return AODV_NO_ACTION;
}

}//namespace//
