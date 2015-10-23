/***************************************************************************
                          rrep.c  -  description
                             -------------------
    begin                : Wed Aug 6 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#include "kernelaodvglue.h"
#include "rrep.h"

namespace KernelAodvPort {

void convert_rrep_to_host(rrep * tmp_rrep)
{
    tmp_rrep->dst_seq = ntohl(tmp_rrep->dst_seq);
    tmp_rrep->lifetime = ntohl(tmp_rrep->lifetime);
}


void convert_rrep_to_network(rrep * tmp_rrep)
{
    tmp_rrep->dst_seq = htonl(tmp_rrep->dst_seq);
    tmp_rrep->lifetime = htonl(tmp_rrep->lifetime);
}


int check_rrep(
    AodvScheduleTimer* aodvScheduleTimerPtr,
    AodvNeighbor* aodvNeighborPtr,
    AodvHello* aodvHelloPtr,
    rrep * tmp_rrep,
    task * tmp_packet)
{
    //ScenSim-Port://char dst_ip[16];
    //ScenSim-Port://char src_ip[16];
    //ScenSim-Port://u_int32_t tmp;
    //ScenSim-Port://u_int64_t timer;


    if (tmp_rrep->src_ip == tmp_rrep->dst_ip)
    {
        //its a hello messages! HELLO WORLD!
        aodvHelloPtr->recv_hello(
            aodvNeighborPtr,
            tmp_rrep,
            tmp_packet);
        return 0;
    }

    if (!aodvNeighborPtr->valid_aodv_neigh(tmp_packet->src_ip))
    {
        //ScenSim-Port://printk(KERN_INFO "AODV: Not processing RREP from %s, not a valid neighbor\n", inet_ntoa(tmp_packet->src_ip));
        return 0;
    }

    //ScenSim-Port://delete_timer(tmp_rrep->dst_ip, TASK_RESEND_RREQ);
    aodvScheduleTimerPtr->delete_timer(TASK_RESEND_RREQ, tmp_rrep->dst_ip);
    //ScenSim-Port://update_timer_queue();
    //ScenSim-Port://strcpy(src_ip, inet_ntoa(tmp_packet->src_ip));
    //ScenSim-Port://strcpy(dst_ip, inet_ntoa(tmp_rrep->dst_ip));
    //ScenSim-Port://printk(KERN_INFO "AODV: recv a route to: %s next hop: %s \n", dst_ip, src_ip);

    return 1;

}

int recv_rrep(
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    AodvScheduleTimer* aodvScheduleTimerPtr,
    AodvNeighbor* aodvNeighborPtr,
    AodvHello* aodvHelloPtr,
    task * tmp_packet)
{
    aodv_route *send_route;
    //ScenSim-Port://aodv_route *recv_route;
    //ScenSim-Port://char dst_ip[16];
    //ScenSim-Port://char src_ip[16];

    rrep *tmp_rrep;


    tmp_rrep = static_cast<rrep*>(tmp_packet->data);
    convert_rrep_to_host(tmp_rrep);


    if (!check_rrep(
             aodvScheduleTimerPtr,
             aodvNeighborPtr,
             aodvHelloPtr,
             tmp_rrep,
             tmp_packet))
    {
        return 0;
    }


    tmp_rrep->metric++;

    aodvRoutingTablePtr->update_aodv_route(
        tmp_rrep->dst_ip,
        tmp_packet->src_ip,
        tmp_rrep->metric,
        tmp_rrep->dst_seq,
        tmp_packet->dev);

    send_route = aodvRoutingTablePtr->find_aodv_route(tmp_rrep->src_ip);


    if (!send_route)
    {
        //ScenSim-Port://strcpy(src_ip, inet_ntoa(tmp_rrep->src_ip));
        //ScenSim-Port://strcpy(dst_ip, inet_ntoa(tmp_rrep->dst_ip));
        //ScenSim-Port://printk(KERN_WARNING "AODV: No reverse-route for RREP from: %s to: %s", dst_ip, src_ip);
        return 0;
    }

    if (!send_route->self_route)
    {
        //ScenSim-Port://strcpy(src_ip, inet_ntoa(tmp_rrep->src_ip));
        //ScenSim-Port://strcpy(dst_ip, inet_ntoa(tmp_rrep->dst_ip));
        //ScenSim-Port://printk(KERN_INFO "AODV: Forwarding a route to: %s from node: %s \n", dst_ip, src_ip);

        convert_rrep_to_network(tmp_rrep);
        //ScenSim-Port://send_message(send_route->next_hop, NET_DIAMETER, tmp_rrep, sizeof(rrep), NULL);
        kernelAodvProtocolPtr->SendUnicastAodvControlPacket(
            send_route->next_hop,
            kernelAodvProtocolPtr->GetAodvNetDiameter(),
            *tmp_rrep);
    }



    /* If I'm not the destination of the RREP I forward it */

    return 0;
}



int send_rrep(
    const shared_ptr<ScenSim::SimulationEngineInterface>& simulationEngineInterfacePtr,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    aodv_route * src_route,
    aodv_route * dst_route,
    rreq * tmp_rreq)
{

    rrep *tmp_rrep = new rrep();

//ScenSim-Port://    if ((tmp_rrep = (rrep *) kmalloc(sizeof(rrep), GFP_ATOMIC)) == NULL)
//ScenSim-Port://    {
//ScenSim-Port://        printk(KERN_WARNING "AODV: Can't allocate new rreq\n");
//ScenSim-Port://        return 0;
//ScenSim-Port://    }

    tmp_rrep->type = RREP_MESSAGE;

    tmp_rrep->reserved1 = 0;
    tmp_rrep->reserved2 = 0;
    tmp_rrep->src_ip = src_route->ip;
    tmp_rrep->dst_ip = dst_route->ip;
    tmp_rrep->dst_seq = htonl(dst_route->seq);
    tmp_rrep->metric = dst_route->metric;
    if (dst_route->self_route)
    {
        tmp_rrep->lifetime = htonl( static_cast<unsigned int>(kernelAodvProtocolPtr->GetAodvMyRouteTimeout() / ScenSim::MILLI_SECOND) );
    } else
    {
        //ScenSim-Port://tmp_rrep->lifetime = htonl(dst_route->lifetime - getcurrtime());
        tmp_rrep->lifetime =
            htonl( static_cast<unsigned int>((dst_route->lifetime - simulationEngineInterfacePtr->CurrentTime()) / ScenSim::MILLI_SECOND) );
    }

    //ScenSim-Port://send_message(src_route->next_hop, NET_DIAMETER, tmp_rrep, sizeof(rrep) , NULL);
    kernelAodvProtocolPtr->SendUnicastAodvControlPacket(
        src_route->next_hop,
        kernelAodvProtocolPtr->GetAodvNetDiameter(),
        *tmp_rrep);
    //ScenSim-Port://kfree(tmp_rrep);
    return 0;
}





int gen_rrep(
    const shared_ptr<ScenSim::SimulationEngineInterface>& simulationEngineInterfacePtr,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    const unsigned int src_ip,
    const unsigned int dst_ip,
    rreq *tmp_rreq)
{
    //ScenSim-Port://rrep tmp_rrep;
    aodv_route *src_route;
    aodv_route *dst_route;
    //struct interface_list_entry *tmp_interface=NULL;


    //ScenSim-Port://char src_ip_str[16];
    //ScenSim-Port://char dst_ip_str[16];

    /* Get the source and destination IP address from the RREQ */
    src_route = aodvRoutingTablePtr->find_aodv_route(src_ip);
    if (src_route == nullptr)
    {
        //ScenSim-Port://printk(KERN_WARNING "AODV: RREP  No route to Source! src: %s\n", inet_ntoa(src_ip));
        printf("AODV Warning: RREP has no route to source.\n");
        //ScenSim-Port://return -EHOSTUNREACH;
        return -1;
    }

    dst_route = aodvRoutingTablePtr->find_aodv_route(dst_ip);

    if (dst_route == nullptr)
    {
        //ScenSim-Port://printk(KERN_WARNING "AODV: RREP  No route to Dest! dst: %s\n", inet_ntoa(dst_ip));
        printf("AODV Warning: RREP has no route to destination.\n");
        //ScenSim-Port://return -EHOSTUNREACH;
        return -1;
    }

    send_rrep(
        simulationEngineInterfacePtr,
        kernelAodvProtocolPtr,
        src_route,
        dst_route,
        tmp_rreq);

    return 0;
}

}//namespace//
