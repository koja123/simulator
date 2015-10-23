/***************************************************************************
                          rreq.c  -  description
                             -------------------
    begin                : Mon Aug 4 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#include "kernelaodvglue.h"
#include "rreq.h"
#include "aodv_utils.h"
#include "rrep.h"

namespace KernelAodvPort {

//ScenSim-Port://extern aodv_route * g_my_route;
//ScenSim-Port://extern u_int8_t        g_aodv_gateway;
//ScenSim-Port://extern u_int32_t        g_broadcast_ip;

using std::shared_ptr;//ScenSim-Port://
using ScenSim::KernelAodvProtocol;//ScenSim-Port://
using ScenSim::SimulationEngineInterface;//ScenSim-Port://

void convert_rreq_to_host(rreq * tmp_rreq)
{
    tmp_rreq->rreq_id = ntohl(tmp_rreq->rreq_id);
    tmp_rreq->dst_seq = ntohl(tmp_rreq->dst_seq);
    tmp_rreq->src_seq = ntohl(tmp_rreq->src_seq);
}

void convert_rreq_to_network(rreq * tmp_rreq)
{
    tmp_rreq->rreq_id = htonl(tmp_rreq->rreq_id);
    tmp_rreq->dst_seq = htonl(tmp_rreq->dst_seq);
    tmp_rreq->src_seq = htonl(tmp_rreq->src_seq);
}


/****************************************************

   recv_rreq
----------------------------------------------------
Handles the recieving of RREQs
****************************************************/

int check_rreq(
    AodvRoutingTable* aodvRoutingTablePtr,
    AodvFloodId* aodvFloodIdPtr,
    AodvNeighbor* aodvNeighborPtr,
    rreq * tmp_rreq,
    const unsigned int last_hop_ip)
{
    aodv_neigh *tmp_neigh = new aodv_neigh();
    aodv_route *src_route = new aodv_route();

    //ScenSim-Port://char src_ip[16];
    //ScenSim-Port://char dst_ip[16];
    //ScenSim-Port://char pkt_ip[16];


    if (aodvFloodIdPtr->find_flood_id(tmp_rreq->src_ip, tmp_rreq->rreq_id))
    {
        return 0;
    }

    src_route = aodvRoutingTablePtr->find_aodv_route(tmp_rreq->src_ip);

    if (!aodvNeighborPtr->valid_aodv_neigh(last_hop_ip))
    {
        return 0;
    }

    if ((src_route != NULL) && (src_route->next_hop != last_hop_ip) && (src_route->next_hop == src_route->ip))

    {
        //ScenSim-Port://strcpy(src_ip, inet_ntoa(tmp_rreq->src_ip));
        //ScenSim-Port://strcpy(dst_ip, inet_ntoa(tmp_rreq->dst_ip));
        //ScenSim-Port://strcpy(pkt_ip, inet_ntoa(last_hop_ip));
        //ScenSim-Port://printk
        //ScenSim-Port://    ("Last Hop of: %s Doesn't match route: %s dst: %s RREQ_ID: %u, expires: %d\n",
        //ScenSim-Port://     pkt_ip, src_ip, dst_ip, tmp_rreq->rreq_id, src_route->lifetime - getcurrtime());
    }




    return 1;

}


int reply_to_rreq(
    AodvRoutingTable* aodvRoutingTablePtr,
    rreq * tmp_rreq)
{
    //ScenSim-Port://char src_ip[16];
    //ScenSim-Port://char dst_ip[16];
    //ScenSim-Port://char pkt_ip[16];
    aodv_route *dst_route;

    dst_route = aodvRoutingTablePtr->find_aodv_route(tmp_rreq->dst_ip);

//This "if" should be modified to just check this packet for this node or not.
//We do not assume this situation.
//ScenSim-Port://   if (!aodv_subnet_test(tmp_rreq->dst_ip)) {
//ScenSim-Port://
//ScenSim-Port://        if (g_aodv_gateway) {
//ScenSim-Port://
//ScenSim-Port://            //ScenSim-Port://printk("Gatewaying for address: %s, ", inet_ntoa( tmp_rreq->dst_ip ));
//ScenSim-Port://            if (dst_route == NULL) {
//ScenSim-Port://                //ScenSim-Port://printk("creating route for: %s \n",inet_ntoa( tmp_rreq->dst_ip ));
//ScenSim-Port://                dst_route = aodvRoutingTable->create_aodv_route(tmp_rreq->dst_ip);
//ScenSim-Port://                dst_route->seq = g_my_route->seq;
//ScenSim-Port://                dst_route->next_hop = tmp_rreq->dst_ip;
//ScenSim-Port://                dst_route->metric = 1;
//ScenSim-Port://                dst_route->dev =   g_my_route->dev;
//ScenSim-Port://
//ScenSim-Port://            }
//ScenSim-Port://            else {
//ScenSim-Port://
//ScenSim-Port://                //ScenSim-Port://printk("using route: %s \n",inet_ntoa( dst_route->ip ));
//ScenSim-Port://            }
//ScenSim-Port://
//ScenSim-Port://            dst_route->lifetime =  getcurrtime() +  ACTIVE_ROUTE_TIMEOUT;
//ScenSim-Port://            dst_route->route_valid = TRUE;
//ScenSim-Port://            dst_route->route_seq_valid = TRUE;
//ScenSim-Port://
//ScenSim-Port://            return 1;
//ScenSim-Port://        }
//ScenSim-Port://
//ScenSim-Port://        //it is a local subnet and we need to create a route for it before we can reply...
//ScenSim-Port://        if ((dst_route!=NULL) && (dst_route->netmask!=g_broadcast_ip)) {
//ScenSim-Port://
//ScenSim-Port://            //ScenSim-Port://printk("creating route for local address: %s \n",inet_ntoa( tmp_rreq->dst_ip ));
//ScenSim-Port://            dst_route = create_aodv_route(tmp_rreq->dst_ip);
//ScenSim-Port://            dst_route->seq = g_my_route->seq;
//ScenSim-Port://            dst_route->next_hop = tmp_rreq->dst_ip;
//ScenSim-Port://            dst_route->metric = 1;
//ScenSim-Port://            dst_route->dev =   g_my_route->dev;
//ScenSim-Port://            dst_route->lifetime =  getcurrtime() +  ACTIVE_ROUTE_TIMEOUT;
//ScenSim-Port://            dst_route->route_valid = TRUE;
//ScenSim-Port://            dst_route->route_seq_valid = TRUE;
//ScenSim-Port://
//ScenSim-Port://            return 1;
//ScenSim-Port://        }
//ScenSim-Port://
//ScenSim-Port://    }
//ScenSim-Port://    else {

        // if it is not outside of the AODV subnet and I am the dst...
        if (dst_route && dst_route->self_route) {
            if (seq_less_or_equal(dst_route->seq, tmp_rreq->dst_seq)) {
                dst_route->seq = tmp_rreq->dst_seq + 1;
            }

            //ScenSim-Port://strcpy(src_ip, inet_ntoa(tmp_rreq->src_ip));
            //ScenSim-Port://strcpy(dst_ip, inet_ntoa(tmp_rreq->dst_ip));
            //ScenSim-Port://printk(KERN_INFO "AODV: Destination, Generating RREP -  src: %s dst: %s \n", src_ip, dst_ip);
            return 1;
        }
//ScenSim-Port://    }


    /* Test to see if we should send a RREP AKA we have or are the desired route */






    return 0;
}

int forward_rreq(
    KernelAodvProtocol* kernelAodvProtocolPtr,
    rreq * tmp_rreq,
    int ttl)
{
    convert_rreq_to_network(tmp_rreq);

    /* Call send_datagram to send and forward the RREQ */
    //ScenSim-Port://local_broadcast(ttl - 1, tmp_rreq, sizeof(rreq));
    kernelAodvProtocolPtr->SendBroadcastAodvControlPacket(
        static_cast<unsigned char>(ttl - 1),
        *tmp_rreq);

    return 0;
}


int recv_rreq(
    const shared_ptr<ScenSim::SimulationEngineInterface>& simulationEngineInterfacePtr,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    AodvFloodId* aodvFloodIdPtr,
    AodvNeighbor* aodvNeighborPtr,
    task * tmp_packet)
{
    rreq *tmp_rreq;
    //ScenSim-Port://aodv_route *src_route;      /* Routing table entry */
    //ScenSim-Port://u_int64_t current_time;     /* Current time */
    long long int current_time = simulationEngineInterfacePtr->CurrentTime();
    //ScenSim-Port://int size_out;

    tmp_rreq = static_cast<rreq*>(tmp_packet->data);
    convert_rreq_to_host(tmp_rreq);

    //ScenSim-Port://current_time = getcurrtime();       /* Get the current time */


    /* Look in the route request list to see if the node has
       already received this request. */

    if (tmp_packet->ttl <= 1)
    {
//ScenSim-Port://        printk(KERN_INFO "AODV TTL for RREQ from: %s expired\n", inet_ntoa(tmp_rreq->src_ip));
//ScenSim-Port://        return -ETIMEDOUT;
        return 0;
    }


    if (!check_rreq(
             aodvRoutingTablePtr,
             aodvFloodIdPtr,
             aodvNeighborPtr,
             tmp_rreq,
             tmp_packet->src_ip))
    {
        return 1;
    }


    tmp_rreq->metric++;


    /* Have not received this RREQ within BCAST_ID_SAVE time */
    /* Add this RREQ to the list for further checks */


    /* UPDATE REVERSE */
    aodvRoutingTablePtr->update_aodv_route(
        tmp_rreq->src_ip,
        tmp_packet->src_ip,
        tmp_rreq->metric,
        tmp_rreq->src_seq,
        tmp_packet->dev);

    aodvFloodIdPtr->insert_flood_id(
        tmp_rreq->src_ip,
        tmp_rreq->dst_ip,
        tmp_rreq->rreq_id,
        current_time + kernelAodvProtocolPtr->GetAodvPathDiscoveryTime());

    switch (reply_to_rreq(aodvRoutingTablePtr, tmp_rreq))
    {
    case 1:
        gen_rrep(
            simulationEngineInterfacePtr,
            kernelAodvProtocolPtr,
            aodvRoutingTablePtr,
            tmp_rreq->src_ip,
            tmp_rreq->dst_ip,
            tmp_rreq);
        return 0;
        break;
    }

    forward_rreq(
        kernelAodvProtocolPtr,
        tmp_rreq,
        tmp_packet->ttl);

    return 0;
}


int resend_rreq(
    const shared_ptr<ScenSim::SimulationEngineInterface>& simulationEngineInterfacePtr,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    AodvScheduleTimer* aodvScheduleTimerPtr,
    AodvFloodId* aodvFloodIdPtr,
    task * tmp_packet)
{
    aodv_route *tmp_route;
    rreq *out_rreq = new rreq();
    out_rreq->type = RREQ_MESSAGE;
    //ScenSim-Port://u_int8_t out_ttl;
    unsigned char out_ttl;

    if (tmp_packet->retries <= 0)
    {
        //ScenSim-Port://ipq_drop_ip(tmp_packet->dst_ip);

        return 0;
    }


    //ScenSim-Port://if ((out_rreq = (rreq *) kmalloc(sizeof(rreq), GFP_ATOMIC)) == NULL)
    //ScenSim-Port://{
    //ScenSim-Port://    printk(KERN_WARNING "AODV: Can't allocate new rreq\n");
    //ScenSim-Port://    return 0;
    //ScenSim-Port://}





    /* Get routing table entry for destination */
    tmp_route = aodvRoutingTablePtr->find_aodv_route(tmp_packet->dst_ip);


    if (tmp_route == NULL)
    {
        /* Entry does not exist -> set to initial values */
        out_rreq->dst_seq = htonl(0);
        out_rreq->u = TRUE;
        //out_ttl = TTL_START;
    } else
    {
        /* Entry does exist -> get value from rt */
        out_rreq->dst_seq = htonl(tmp_route->seq);
        out_rreq->u = FALSE;
        //out_ttl = tmp_route->metric + TTL_INCREMENT;
    }

    out_ttl = kernelAodvProtocolPtr->GetAodvNetDiameter();

    /* Get routing table entry for source, when this is ourself this one should always exist */

    tmp_route = aodvRoutingTablePtr->find_aodv_route(tmp_packet->src_ip);

    if (tmp_route == NULL)
    {
        assert(false && "Get routing table entry for source, when this is ourself this one should always exist.");
        //ScenSim-Port://printk(KERN_WARNING "AODV: Can't get route to source: %s\n", inet_ntoa(tmp_packet->src_ip));
        //ScenSim-Port://kfree(out_rreq);
        //ScenSim-Port://return -EHOSTUNREACH;
    }

    /* Get our own sequence number */
    tmp_route->rreq_id = tmp_route->rreq_id + 1;
    tmp_route->seq = tmp_route->seq + 1;
    out_rreq->src_seq = htonl(tmp_route->seq);
    out_rreq->rreq_id = htonl(tmp_route->rreq_id);

    /* Fill in the package */
    out_rreq->dst_ip = tmp_packet->dst_ip;
    out_rreq->src_ip = tmp_packet->src_ip;
    //out_rreq->type = RREQ_MESSAGE;
    out_rreq->metric = htonl(0);
    out_rreq->j = 0;
    out_rreq->r = 0;
    out_rreq->d = 0;
    out_rreq->reserved = 0;
    out_rreq->second_reserved = 0;
    out_rreq->g = 1;

    /* Get the broadcast address and ttl right */

    if (aodvFloodIdPtr->insert_flood_id(
            tmp_packet->src_ip,
            tmp_packet->dst_ip,
            tmp_route->rreq_id,
            simulationEngineInterfacePtr->CurrentTime() + kernelAodvProtocolPtr->GetAodvPathDiscoveryTime()) < 0)
    {
        printf("Duplicated control management packet is detected.\n");
        //ScenSim-Port://kfree(out_rreq);
        //ScenSim-Port://printk(KERN_WARNING "AODV: Can't add to broadcast list\n");
        return -ENOMEM;
    }




    //    insert_timer_queue_entry(getcurrtime() + NET_TRAVERSAL_TIME,timer_rreq, sizeof(struct rreq),out_rreq->dst_ip,0,out_ttl, EVENT_RREQ);
    //insert_timer_queue_entry(getcurrtime() + NET_TRAVERSAL_TIME,timer_rreq, sizeof(struct rreq),out_rreq->dst_ip,RREQ_RETRIES,out_ttl, EVENT_RREQ);
    aodvScheduleTimerPtr->insert_rreq_timer(
        out_rreq,
        static_cast<unsigned char>(tmp_packet->retries - 1));
    //ScenSim-Port://update_timer_queue();

    //ScenSim-Port://local_broadcast(out_ttl,out_rreq,sizeof(struct rreq));

    kernelAodvProtocolPtr->SendBroadcastAodvControlPacket(
        out_ttl,
        *out_rreq);

    //ScenSim-Port://kfree(out_rreq);

    return 0;



}

/****************************************************

   gen_rreq
----------------------------------------------------
Generates a RREQ! wahhooo!
****************************************************/
int gen_rreq(
    const shared_ptr<ScenSim::SimulationEngineInterface>& simulationEngineInterfacePtr,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    AodvScheduleTimer* aodvScheduleTimerPtr,
    AodvFloodId* aodvFloodIdPtr,
    const unsigned int src_ip,
    const unsigned int dst_ip)
{
    aodv_route *tmp_route = new aodv_route();
    rreq *out_rreq = new rreq();
    unsigned char out_ttl;


    //ScenSim-Port://if (find_timer(dst_ip, TASK_RESEND_RREQ) != NULL)
    if (aodvScheduleTimerPtr->find_timer(dst_ip, TASK_RESEND_RREQ))
    {
        //try to find duplicated events. If it is detected, no event happen.
        return 0;
    }

//ScenSim-Port://    /* Allocate memory for the rreq message */
//ScenSim-Port://
//ScenSim-Port://    if ((out_rreq = (rreq *) kmalloc(sizeof(rreq), GFP_ATOMIC)) == NULL)
//ScenSim-Port://    {
//ScenSim-Port://        printk(KERN_WARNING "AODV: Can't allocate new rreq\n");
//ScenSim-Port://        return 0;
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://        printk(KERN_INFO "Generating a RREQ for: %s\n", inet_ntoa(dst_ip));
//ScenSim-Port://      if ((out_rreq = (rreq *) kmalloc(sizeof(rreq), GFP_ATOMIC)) == NULL)
//ScenSim-Port://    {
//ScenSim-Port://        printk(KERN_WARNING "AODV: Can't allocate new rreq\n");
//ScenSim-Port://        return -ENOMEM;
//ScenSim-Port://    }
//ScenSim-Port://
    /* Get routing table entry for destination */
    tmp_route = aodvRoutingTablePtr->find_aodv_route(dst_ip);


    if (tmp_route == NULL) {
        /* Entry does not exist -> set to initial values */
        out_rreq->dst_seq = htonl(0);
        out_rreq->u = TRUE;
        //out_ttl = TTL_START;
    }
    else {
        /* Entry does exist -> get value from rt */
        out_rreq->dst_seq = htonl(tmp_route->seq);
        out_rreq->u = FALSE;
        //out_ttl = tmp_route->metric + TTL_INCREMENT;
    }

    out_ttl = kernelAodvProtocolPtr->GetAodvNetDiameter();

    /* Get routing table entry for source, when this is ourself this one should always exist */

    tmp_route = aodvRoutingTablePtr->find_aodv_route(src_ip);

    if (tmp_route == NULL) {
        assert(false && "Get routing table entry for source, when this is ourself this one should always exist.");
//ScenSim-Port://        printk(KERN_WARNING "AODV: Can't get route to source: %s\n", inet_ntoa(src_ip));
//ScenSim-Port://        kfree(out_rreq);
//ScenSim-Port://        return -EHOSTUNREACH;
    }

    /* Get our own sequence number */
    tmp_route->rreq_id = tmp_route->rreq_id + 1;
    tmp_route->seq = tmp_route->seq + 1;
    out_rreq->src_seq = htonl(tmp_route->seq);
    out_rreq->rreq_id = htonl(tmp_route->rreq_id);

    /* Fill in the package */
    out_rreq->dst_ip = dst_ip;
    out_rreq->src_ip = src_ip;
    out_rreq->type = RREQ_MESSAGE;
    out_rreq->metric = htonl(0);
    out_rreq->j = 0;
    out_rreq->r = 0;
    out_rreq->d = 1;
    out_rreq->reserved = 0;
    out_rreq->second_reserved = 0;
    out_rreq->g = 1;

    if (aodvFloodIdPtr->insert_flood_id(
            src_ip,
            dst_ip,
            tmp_route->rreq_id,
            simulationEngineInterfacePtr->CurrentTime() + kernelAodvProtocolPtr->GetAodvPathDiscoveryTime()) < 0)
    {
        printf("Duplicated control management packet is detected.\n");
        //ScenSim-Port://kfree(out_rreq);
        //ScenSim-Port://printk(KERN_WARNING "AODV: Can't add to broadcast list\n");
        return -ENOMEM;
    }

    aodvScheduleTimerPtr->insert_rreq_timer(
        out_rreq,
        kernelAodvProtocolPtr->GetAodvRreqRetries());
//ScenSim-Port://    update_timer_queue();
//ScenSim-Port://
//ScenSim-Port://    local_broadcast(out_ttl, out_rreq, sizeof(rreq));
    kernelAodvProtocolPtr->SendBroadcastAodvControlPacket(
        out_ttl,
        *out_rreq);
//ScenSim-Port://
//ScenSim-Port://    kfree(out_rreq);

    return 0;

}

}//namespace//
