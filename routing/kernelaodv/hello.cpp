/***************************************************************************
                          hello.c  -  description
                             -------------------
    begin                : Wed Aug 13 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#include "kernelaodvglue.h"
#include "hello.h"


namespace KernelAodvPort {

//ScenSim-Port://extern aodv_route *g_my_route;
//ScenSim-Port://extern u_int32_t   g_my_ip;
//ScenSim-Port://#ifdef LINK_LIMIT
//ScenSim-Port://extern int g_link_limit;
//ScenSim-Port://#endif
//ScenSim-Port://rrep * create_hello()
//ScenSim-Port://{
//ScenSim-Port://    rrep *new_rrep;
//ScenSim-Port://
//ScenSim-Port://    return new_rrep;
//ScenSim-Port://}


int AodvHello::send_hello()
{
    rrep *tmp_rrep = new rrep();

    //ScenSim-Port://int i;
    //ScenSim-Port://aodv_dst * tmp_dst;
    //ScenSim-Port://struct interface_list_entry *tmp_interface;


    //ScenSim-Port://if ((tmp_rrep = (rrep *) kmalloc(sizeof(rrep) , GFP_ATOMIC)) == NULL)
    //ScenSim-Port://{
    //ScenSim-Port://    printk(KERN_WARNING "AODV: Can't allocate new rreq\n");
    //ScenSim-Port://    neigh_read_unlock();
    //ScenSim-Port://    return 0;
    //ScenSim-Port://}


    tmp_rrep->type = RREP_MESSAGE;
    tmp_rrep->reserved1 = 0;


    //ScenSim-Port://tmp_rrep->src_ip = g_my_route->ip;
    //ScenSim-Port://tmp_rrep->dst_ip = g_my_route->ip;
    //ScenSim-Port://tmp_rrep->dst_seq = htonl(g_my_route->seq);

    tmp_rrep->src_ip = kernelAodvProtocolPtr->GetMyRoute()->ip;
    tmp_rrep->dst_ip = kernelAodvProtocolPtr->GetMyRoute()->ip;
    tmp_rrep->dst_seq = htonl(kernelAodvProtocolPtr->GetMyRoute()->seq);
    tmp_rrep->lifetime = htonl( static_cast<unsigned int>(kernelAodvProtocolPtr->GetAodvMyRouteTimeout() / ScenSim::MILLI_SECOND) );

    //ScenSim-Port://local_broadcast(1, tmp_rrep, sizeof(rrep) );
    kernelAodvProtocolPtr->SendBroadcastAodvControlPacket(1, *tmp_rrep);

    //ScenSim-Port://kfree (tmp_rrep);
    //ScenSim-Port://insert_timer(TASK_HELLO, HELLO_INTERVAL, g_my_ip);

    aodvScheduleTimerPtr->insert_timer(
        TASK_HELLO,
        static_cast<unsigned int>(kernelAodvProtocolPtr->GetAodvHelloInterval() / ScenSim::MILLI_SECOND),
        kernelAodvProtocolPtr->GetIpAddressLow32Bits());


    //ScenSim-Port://update_timer_queue();
    return 0;
}


int AodvHello::recv_hello(
    AodvNeighbor* aodvNeighborPtr,
    rrep * tmp_rrep,
    task * tmp_packet)
{
    //ScenSim-Port://aodv_route *recv_route;
    aodv_neigh *tmp_neigh;

    tmp_neigh = aodvNeighborPtr->find_aodv_neigh(tmp_rrep->dst_ip);
    if (tmp_neigh == NULL)
    {

       tmp_neigh = aodvNeighborPtr->create_aodv_neigh(tmp_rrep->dst_ip);
       if (!tmp_neigh)
       {
            //ScenSim-Port://printk(KERN_WARNING "AODV: Error creating neighbor: %s\n", inet_ntoa(tmp_rrep->dst_ip));
            printf("Error: Creating Neighbor.\n");
            return -1;
       }
       //ScenSim-Port://memcpy(&(tmp_neigh->hw_addr), &(tmp_packet->src_hw_addr), sizeof(unsigned char) * ETH_ALEN);
       //ScenSim-Port://tmp_neigh->dev = tmp_packet->dev;

#ifdef AODV_SIGNAL
       set_spy();
#endif

    }

    //ScenSim-Port://delete_timer(tmp_neigh->ip, TASK_NEIGHBOR);
    aodvScheduleTimerPtr->delete_timer(
        TASK_NEIGHBOR,
        tmp_neigh->ip);

    aodvScheduleTimerPtr->insert_timer(
        TASK_NEIGHBOR,
        static_cast<unsigned int>(kernelAodvProtocolPtr->GetAodvHelloInterval() / ScenSim::MILLI_SECOND) * (1 + kernelAodvProtocolPtr->GetAodvAllowedHelloLoss()) + 100,
        tmp_neigh->ip);

    //ScenSim-Port://update_timer_queue();
    //ScenSim-Port://tmp_neigh->lifetime = tmp_rrep->lifetime + getcurrtime() + 20;

    tmp_neigh->lifetime = tmp_rrep->lifetime * ScenSim::MILLI_SECOND
                          + simulationEngineInterfacePtr->CurrentTime()
                          + 20 * ScenSim::MILLI_SECOND;

    aodvNeighborPtr->update_aodv_neigh(
        aodvRoutingTablePtr,
        tmp_neigh,
        tmp_rrep);


    return 0;
}

}//namespace//
