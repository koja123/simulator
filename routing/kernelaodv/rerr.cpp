/***************************************************************************
                          rerr.c  -  description
                             -------------------
    begin                : Mon Aug 11 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#include "kernelaodvglue.h"
#include "rerr.h"

namespace KernelAodvPort {
using std::cerr;//ScenSim-Port://
using std::endl;//ScenSim-Port://

int send_rerr(
    KernelAodvProtocol* kernelAodvProtocolPtr,
    rerr_route * bad_routes,
    int num_routes)
{
    rerr *tmp_rerr;
    unsigned int rerr_size;
    rerr_route *tmp_rerr_route = bad_routes;
    rerr_route *dead_rerr_route = NULL;
    //ScenSim-Port://aodv_dst *tmp_rerr_dst = NULL;
    aodv_dst *tmp_rerr_dst = new aodv_dst();

    void *data;

    //ScenSim-Port://int datalen, i;

    rerr_size = sizeof(rerr) + (sizeof(aodv_dst) * num_routes);


    //ScenSim-Port://if ((data = kmalloc(rerr_size, GFP_ATOMIC)) == NULL)
    if ((data = malloc(rerr_size)) == NULL)
    {
        //ScenSim-Port://printk(KERN_WARNING "AODV: Error creating packet to send RERR\n");
        cerr << "Error: Creating RRER packet has errors." <<endl;
        exit(1);
        return -ENOMEM;
    }


    tmp_rerr = (rerr *) data;

    tmp_rerr->type = 3;
    tmp_rerr->dst_count = static_cast<unsigned char>(num_routes);
    tmp_rerr->reserved = 0;
    tmp_rerr->n = 0;


    //add in the routes that have broken...
    //ScenSim-Port://tmp_rerr_dst = (aodv_dst *) ((void *)data + sizeof(rerr));
    tmp_rerr_dst = (aodv_dst *) (static_cast<char *>(data) + sizeof(rerr));
    while (tmp_rerr_route)
    {
        tmp_rerr_dst->ip = tmp_rerr_route->ip;
        tmp_rerr_dst->seq = tmp_rerr_route->seq;
        dead_rerr_route = tmp_rerr_route;
        tmp_rerr_route = tmp_rerr_route->next;
        //ScenSim-Port://kfree(dead_rerr_route);
        //dead_rerr_route = NULL;
        //ScenSim-Port://tmp_rerr_dst = (void *)tmp_rerr_dst + sizeof(aodv_dst);
        tmp_rerr_dst = (aodv_dst *)(tmp_rerr_dst + 1);  //Questionable. static_cast<char *> for shifting address.
    }

    //ScenSim-Port://local_broadcast(NET_DIAMETER, data, rerr_size);

    unsigned char* rerrData = static_cast<unsigned char*>(data);
    kernelAodvProtocolPtr->SendBroadcastAodvControlPacket(
        kernelAodvProtocolPtr->GetAodvNetDiameter(),
        rerrData,
        rerr_size);
    //ScenSim-Port://kfree(data);
    return 0;
}



int gen_rerr(
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    const unsigned int brk_dst_ip)
{
    aodv_route *tmp_route;
    rerr_route *bad_routes = NULL;
    rerr_route *tmp_rerr_route = NULL;

    int num_routes = 0;

    int rerrhdr_created = 0;

    tmp_route = aodvRoutingTablePtr->first_aodv_route();

    //go through list
    while (tmp_route != NULL)
    {
        if ((tmp_route->next_hop == brk_dst_ip)  && !tmp_route->self_route) //&& valid_aodv_route(tmp_route)
        {
            //ScenSim-Port://if ((tmp_rerr_route = (rerr_route *) kmalloc(sizeof(rerr_route), GFP_ATOMIC)) == NULL)
            if ((tmp_rerr_route = (rerr_route *) malloc(sizeof(rerr_route))) == NULL)
            {
                //ScenSim-Port://printk(KERN_WARNING "AODV: RERR Can't allocate new entry\n");
                cerr << "Error: RERR cannot allocate new entry."<< endl;
                exit(1);
                return 0;
            }
            tmp_rerr_route->next = bad_routes;
            bad_routes = tmp_rerr_route;
            tmp_rerr_route->ip = tmp_route->ip;
            tmp_rerr_route->seq = htonl(tmp_route->seq);
            num_routes++;
            if (tmp_route->route_valid)
                aodvRoutingTablePtr->expire_aodv_route(tmp_route);
            //ScenSim-Port://printk(KERN_INFO "RERR: Broken link as next hop for - %s \n", inet_ntoa(tmp_route->ip));
        }
        //move on to the next entry
        tmp_route = tmp_route->next;
    }
    if (num_routes)
    {
        send_rerr(
            kernelAodvProtocolPtr,
            bad_routes,
            num_routes);
    }

    return 0;
}

int recv_rerr(
    KernelAodvProtocol* kernelAodvProtocolPtr,
    AodvRoutingTable* aodvRoutingTablePtr,
    task * tmp_packet)
{
    aodv_route *tmp_route;
    rerr *tmp_rerr;
    aodv_dst *tmp_rerr_dst;
    rerr_route *outgoing_rerr_routes = NULL;
    rerr_route *tmp_rerr_route = NULL;
    int num_outgoing_routes = 0;
    int i;


    //ScenSim-Port://tmp_rerr = (rerr *) tmp_packet->data;
    tmp_rerr = static_cast<rerr *> (tmp_packet->data);
    //ScenSim-Port://tmp_rerr_dst = (aodv_dst *) (&tmp_packet->data + sizeof(rerr));
    tmp_rerr_dst = (aodv_dst *) (static_cast<char *>(tmp_packet->data) + sizeof(rerr));
    //ScenSim-Port://printk(KERN_INFO "AODV: recieved a route error from %s, count= %u\n", inet_ntoa(tmp_packet->src_ip), tmp_rerr->dst_count);

    for (i = 0; i < tmp_rerr->dst_count; i++)
    {
        //go through all the unr dst in the rerr

        //ScenSim-Port://printk(KERN_INFO "       -> %s", inet_ntoa(tmp_rerr_dst->ip));
        tmp_route = aodvRoutingTablePtr->find_aodv_route(tmp_rerr_dst->ip);
        if (tmp_route && !tmp_route->self_route)
        {
            // if the route is valid and not a self route
            if (tmp_route->next_hop == tmp_packet->src_ip)
            {
                //if we are using the person who sent the rerr as a next hop

                //ScenSim-Port://printk(KERN_INFO " Removing route");
                tmp_route->seq = ntohl(tmp_rerr_dst->seq); //convert to host format for local storage

                //create a temporary rerr route to use for the outgoing RERR
                //ScenSim-Port://if ((tmp_rerr_route = (rerr_route *) kmalloc(sizeof(rerr_route), GFP_ATOMIC)) == NULL)
                if ((tmp_rerr_route = (rerr_route *) malloc(sizeof(rerr_route))) == NULL)
                {
                    //ScenSim-Port://printk(KERN_WARNING "AODV: RERR Can't allocate new entry\n");
                    cerr << "Error: RERR cannot allocate new entry."<< endl;
                    exit(1);
                    return 0;
                }


                tmp_rerr_route->next = outgoing_rerr_routes;
                outgoing_rerr_routes = tmp_rerr_route;
                tmp_rerr_route->ip = tmp_rerr_dst->ip;
                tmp_rerr_route->seq = tmp_rerr_dst->seq; //Already in network format...
                num_outgoing_routes++;
                if (tmp_route->route_valid)
                    aodvRoutingTablePtr->expire_aodv_route(tmp_route);
            }
        }
        //ScenSim-Port://tmp_rerr_dst = (aodv_dst *) ((void*)tmp_rerr_dst + sizeof(aodv_dst));
        tmp_rerr_dst = (aodv_dst *) (&tmp_rerr_dst + sizeof(aodv_dst));
        //ScenSim-Port://printk(KERN_INFO "\n");
    }
    if (num_outgoing_routes)
    {
        send_rerr(
            kernelAodvProtocolPtr,
            outgoing_rerr_routes,
            num_outgoing_routes);
    }
    return 0;
}

}//namespace//
