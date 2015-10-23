/***************************************************************************
                          packet_in.c  -  description
                             -------------------
    begin                : Mon Jul 29 2002
    copyright            : (C) 2002 by Luke Klein-Berndt
    email                : kleinb@nist.gov
***************************************************************************/

#include "packet_in.h"

namespace KernelAodvPort {


//ScenSim-Port://int valid_aodv_packet(int numbytes, int type, char *data)
//ScenSim-Port://{
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    rerr *tmp_rerr;
//ScenSim-Port://    rreq *tmp_rreq;
//ScenSim-Port://    rrep *tmp_rrep;
//ScenSim-Port://    switch (type)
//ScenSim-Port://    {
//ScenSim-Port://        //RREQ
//ScenSim-Port://    case 1:
//ScenSim-Port://               tmp_rreq = (rreq *) data;
//ScenSim-Port://        //If it is a normal route rreq
//ScenSim-Port://               if (numbytes == sizeof(rreq))
//ScenSim-Port://                       return 1;
//ScenSim-Port:// 
//ScenSim-Port://        break;
//ScenSim-Port://
//ScenSim-Port://        //RREP
//ScenSim-Port://    case 2:
//ScenSim-Port://               tmp_rrep = (rrep *) data;
//ScenSim-Port://
//ScenSim-Port://               if (numbytes == sizeof(rrep))
//ScenSim-Port://                       return 1;
//ScenSim-Port://               
//ScenSim-Port://               break;
//ScenSim-Port://        //RERR
//ScenSim-Port://    case 3:
//ScenSim-Port://
//ScenSim-Port://        // Normal RERR
//ScenSim-Port://        tmp_rerr = (rerr *) data;
//ScenSim-Port://        if (numbytes == (sizeof(rerr) + (sizeof(aodv_dst) * tmp_rerr->dst_count)))
//ScenSim-Port://        {
//ScenSim-Port://            return 1;
//ScenSim-Port://        }
//ScenSim-Port://        break;
//ScenSim-Port://
//ScenSim-Port://    case 4:                    //Normal RREP-ACK
//ScenSim-Port://        if (numbytes == sizeof(rrep_ack))
//ScenSim-Port://           return 1;
//ScenSim-Port://        break;
//ScenSim-Port://
//ScenSim-Port://    default:
//ScenSim-Port://        break;
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    return 0;
//ScenSim-Port://}




int packet_in(
    KernelAodvProtocol* kernelAodvProtocolPtr,
    const unsigned char aodvControlPacketType,
    const unsigned int lastHopIp,
    const unsigned int destinationIp,
    const unsigned char ttl,
    struct sk_buff *packet)
{

    //ScenSim-Port://    struct net_device *dev;
    //ScenSim-Port://    struct iphdr *ip;
    //ScenSim-Port://aodv_route *tmp_route;
    //ScenSim-Port://aodv_neigh *tmp_neigh;
    //ScenSim-Port://unsigned int tmp_ip;



    // Create aodv message types


    //The packets which come in still have their headers from the IP and UDP
    //ScenSim-Port://    int start_point = sizeof(struct udphdr) + sizeof(struct iphdr);


    //get pointers to the important parts of the message
    //ScenSim-Port://ip = packet->nh.iph;
    //ScenSim-Port://    dev = packet->dev;


    //ScenSim-Port://    if (strcmp(dev->name, "lo") == 0) {
    //ScenSim-Port://        return NF_DROP;
    //ScenSim-Port://    }

    //For all AODV packets the type is the first byte.
    //ScenSim-Port://    aodv_type = (int) packet->data[start_point];
    //ScenSim-Port://
    //ScenSim-Port://    if (!valid_aodv_packet(packet->len - start_point, aodv_type, packet->data + start_point))
    //ScenSim-Port://    {
    //ScenSim-Port://        //ScenSim-Port://printk(KERN_NOTICE
    //ScenSim-Port://        //ScenSim-Port://       "AODV: Packet of type: %d and of size %u from: %s failed packet check!\n", aodv_type, packet->len - start_point, inet_ntoa(ip->saddr));
    //ScenSim-Port://        printf("AODV: failed check!/n");
    //ScenSim-Port://        return NF_DROP;
    //ScenSim-Port://
    //ScenSim-Port://    }


    //tmp_neigh = find_aodv_neigh_by_hw(&(packet->mac.ethernet->h_source));
    //
    //if (tmp_neigh != NULL) {
    //    delete_timer(tmp_neigh->ip, TASK_NEIGHBOR);
    //    insert_timer(TASK_NEIGHBOR, HELLO_INTERVAL * (1 + ALLOWED_HELLO_LOSS) + 100, tmp_neigh->ip);
    //  update_timer_queue();
    //}


    //place packet in the event queue!
    insert_task(
        kernelAodvProtocolPtr,
        lastHopIp,
        destinationIp,
        aodvControlPacketType,
        ttl,
        packet);


    return NF_ACCEPT;
}



unsigned int input_handler(
    //ScenSim-Port://unsigned int hooknum,
    KernelAodvProtocol* kernelAodvProtocolPtr,
    const unsigned char aodvControlPacketType,
    const unsigned int lastHopIp,
    const unsigned int destinationIp,
    const unsigned char ttl,
    struct sk_buff **skb
    //ScenSim-Port://,
    //ScenSim-Port://const struct net_device *in,
    //ScenSim-Port://const struct net_device *out,
    //ScenSim-Port://int (*okfn) (struct sk_buff *)
    )
{
//ScenSim-Port://    struct iphdr *ip = (*skb)->nh.iph;
//ScenSim-Port://    struct iphdr *dev_ip = in->ip_ptr;
//ScenSim-Port://    void *p = (uint32_t *) ip + ip->ihl;
//ScenSim-Port://    struct udphdr *udp = p; //(struct udphdr *) ip + ip->ihl;
//ScenSim-Port://    struct ethhdr *mac = (*skb)->mac.ethernet;  //Thanks to Randy Pitz for adding this extra check...
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    if ((*skb)->h.uh != NULL) {
//ScenSim-Port://
//ScenSim-Port://        if ((udp->dest == htons(AODVPORT)) //udp->dest means udp required port number for AODV. 654 is its port.
//ScenSim-Port://            && (mac->h_proto == htons(ETH_P_IP))) {  //this checks if protocol number is equal to IP protocol(0800). (e.g.ARP is 0806.RARP is 8036.)
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://            if (dev_ip->saddr != ip->saddr) {
//ScenSim-Port://
//ScenSim-Port://                return packet_in(*(skb));
//ScenSim-Port://
//ScenSim-Port://            }
//ScenSim-Port://            else {
//ScenSim-Port://
//ScenSim-Port://                printf("dropping packet from: %s\n",inet_ntoa(ip->saddr));
//ScenSim-Port://                return NF_DROP;
//ScenSim-Port://            }
//ScenSim-Port://        }
//ScenSim-Port://    }
//ScenSim-Port://    return NF_ACCEPT;

    return packet_in(
               kernelAodvProtocolPtr,
               aodvControlPacketType,
               lastHopIp,
               destinationIp,
               ttl,
               *(skb));
}

}//namespace//

