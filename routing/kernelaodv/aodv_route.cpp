
/***************************************************************************
                          aodv_route.c  -  description
                             -------------------
    begin                : Mon Jul 29 2002
    copyright            : (C) 2002 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#include "kernelaodvglue.h"
#include "aodv_route.h"
#include "aodv_utils.h"

namespace KernelAodvPort {


//ScenSim-Port://aodv_route *aodv_route_table;
//ScenSim-Port://extern u_int32_t g_broadcast_ip;
//ScenSim-Port://rwlock_t route_lock = RW_LOCK_UNLOCKED;


//ScenSim-Port://inline void route_read_lock()
//ScenSim-Port://{
//ScenSim-Port://    read_lock_bh(&route_lock);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://inline void route_read_unlock()
//ScenSim-Port://{
//ScenSim-Port://    read_unlock_bh(&route_lock);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://inline void route_write_lock()
//ScenSim-Port://{
//ScenSim-Port://    write_lock_bh(&route_lock);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://inline void route_write_unlock()
//ScenSim-Port://{
//ScenSim-Port://   
//ScenSim-Port://    write_unlock_bh(&route_lock);
//ScenSim-Port://}


void AodvRoutingTable::init_aodv_route_table(const unsigned int initInterfaceIp)
{
    aodv_route *tmp_route;
    unsigned int lo;
    aodv_route_table = NULL;

    //ScenSim-Port://inet_aton("127.0.0.1",&lo);
    //ScenSim-Port://lo = 2130706433;
    lo = initInterfaceIp;

    tmp_route = create_aodv_route(lo);
    tmp_route->next_hop = lo;
    tmp_route->metric = 0;
    tmp_route->self_route = TRUE;
    tmp_route->route_valid = TRUE;
    tmp_route->seq = 0;
}


aodv_route* AodvRoutingTable::first_aodv_route()
{
    return aodv_route_table;
}


//ScenSim-Port://int valid_aodv_route(aodv_route * tmp_route)
//ScenSim-Port://{
//ScenSim-Port://    if (time_after(tmp_route->lifetime,  getcurrtime()) && tmp_route->route_valid)
//ScenSim-Port://    {
//ScenSim-Port://        return 1;
//ScenSim-Port://    } else
//ScenSim-Port://    {
//ScenSim-Port://        return 0;
//ScenSim-Port://    }
//ScenSim-Port://}


void AodvRoutingTable::expire_aodv_route(aodv_route * tmp_route)
{
    //marks a route as expired

    tmp_route->lifetime = (simulationEngineInterfacePtr->CurrentTime()
                           + kernelAodvProtocolPtr->GetAodvDeletePeriod());
    tmp_route->seq++;
    tmp_route->route_valid = FALSE;

    //ScenSim-Port://
    kernelAodvProtocolPtr->DeleteRoutingTableEntry(
        tmp_route->ip,
        tmp_route->next_hop,
        tmp_route->netmask);
}



void AodvRoutingTable::remove_aodv_route(aodv_route * dead_route)
{

    //ScenSim-Port://route_write_lock();

    if (aodv_route_table == dead_route)
    {
        aodv_route_table = dead_route->next;
    }
    if (dead_route->prev != NULL)
    {
        dead_route->prev->next = dead_route->next;
    }
    if (dead_route->next!=NULL)
    {
        dead_route->next->prev = dead_route->prev;
    }

    //ScenSim-Port://route_write_unlock();

    //ScenSim-Port://kfree(dead_route);
}


//ScenSim-Port://int cleanup_aodv_route_table()
//ScenSim-Port://{
//ScenSim-Port://    aodv_route *dead_route, *tmp_route;
//ScenSim-Port://    //route_write_lock();
//ScenSim-Port://
//ScenSim-Port://    tmp_route = aodv_route_table;
//ScenSim-Port://    while (tmp_route!=NULL) 
//ScenSim-Port://   {
//ScenSim-Port://       delete_kernel_route_entry(tmp_route->ip, tmp_route->next_hop, tmp_route->netmask);
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://           dead_route = tmp_route;
//ScenSim-Port://      tmp_route = tmp_route->next;
//ScenSim-Port://           kfree(dead_route);
//ScenSim-Port://    }
//ScenSim-Port://    aodv_route_table = NULL;
//ScenSim-Port://    //route_write_unlock();
//ScenSim-Port://    return 0;
//ScenSim-Port://}


int AodvRoutingTable::flush_aodv_route_table()
{
    //ScenSim-Port://u_int64_t currtime = getcurrtime();
    long long int currtime = simulationEngineInterfacePtr->CurrentTime();
    aodv_route *dead_route, *tmp_route, *prev_route=NULL;
    //route_write_lock();

    tmp_route = aodv_route_table;
    while (tmp_route!=NULL)
   {
            if (prev_route != tmp_route->prev)
            {
                //ScenSim-Port://printk("AODV: Routing table error! %s prev is wrong!\n",inet_ntoa(tmp_route->ip));
                printf("Error: Routing table error! ip(%u)\n", tmp_route->ip);
                }
    prev_route = tmp_route;


             if (time_before(tmp_route->lifetime, currtime) && (!tmp_route->self_route))
        {

            //printk("looking at route: %s\n",inet_ntoa(tmp_route->ip));

                if (tmp_route->route_valid )
            {
                expire_aodv_route(tmp_route);
                tmp_route = tmp_route->next;
            } else
            {
                //ScenSim-Port://delete_kernel_route_entry(tmp_route->ip, tmp_route->next_hop, tmp_route->netmask);
                kernelAodvProtocolPtr->DeleteRoutingTableEntry(
                    tmp_route->ip,
                    tmp_route->next_hop,
                    tmp_route->netmask);

                dead_route = tmp_route;
                prev_route = tmp_route->prev;
                tmp_route = tmp_route->next;
                //ScenSim-Port://route_read_unlock();
                remove_aodv_route(dead_route);
                //ScenSim-Port://route_read_lock();
            }
                }
            else
            {
            tmp_route = tmp_route->next;
          }

    }

  //  route_write_unlock();
    return 0;
}


//ScenSim-Port://int delete_aodv_route(u_int32_t target_ip)
//ScenSim-Port://{
//ScenSim-Port://    aodv_route *dead_route;
//ScenSim-Port://    route_read_lock();
//ScenSim-Port://
//ScenSim-Port://    dead_route = aodv_route_table;
//ScenSim-Port://    while ((dead_route != NULL) && (dead_route->ip <= target_ip))
//ScenSim-Port://    {
//ScenSim-Port://        if (dead_route->ip == target_ip)
//ScenSim-Port://        {
//ScenSim-Port://
//ScenSim-Port://            route_read_unlock();
//ScenSim-Port://            remove_aodv_route(dead_route);
//ScenSim-Port://            return 1;
//ScenSim-Port://        }
//ScenSim-Port://        dead_route = dead_route->next;
//ScenSim-Port://
//ScenSim-Port://    }
//ScenSim-Port://    route_read_unlock();
//ScenSim-Port://    return 0;
//ScenSim-Port://}



void AodvRoutingTable::insert_aodv_route(aodv_route* new_route)
{
    aodv_route *tmp_route;
    aodv_route *prev_route = NULL;

    //ScenSim-Port://route_write_lock();

    tmp_route = aodv_route_table;

    while ((tmp_route != NULL) && (tmp_route->ip < new_route->ip))
    {
        prev_route = tmp_route;
        tmp_route = tmp_route->next;
    }


    if (aodv_route_table && (tmp_route == aodv_route_table))   // if it goes in the first spot in the table
    {
        aodv_route_table->prev = new_route;
        new_route->next = aodv_route_table;
        aodv_route_table = new_route;
    }

    if (aodv_route_table == NULL)       // if the routing table is empty
    {
        aodv_route_table = new_route;
    }


    if (prev_route!=NULL)
    {
        if (prev_route->next)
        {
            prev_route->next->prev = new_route;
        }

        new_route->next = prev_route->next;
        new_route->prev = prev_route;
        prev_route->next = new_route;
    }

    //ScenSim-Port://route_write_unlock();
    return;
}


aodv_route* AodvRoutingTable::create_aodv_route(const unsigned int ip)
{
    aodv_route *tmp_entry = new aodv_route();

    /* Allocate memory for new entry */

    //ScenSim-Port://if ((tmp_entry = (aodv_route *) kmalloc(sizeof(aodv_route), GFP_ATOMIC)) == NULL)
    //ScenSim-Port://{
    //ScenSim-Port://    printk(KERN_WARNING "AODV: Error getting memory for new route\n");
    //ScenSim-Port://    return NULL;
    //ScenSim-Port://}

    tmp_entry->self_route = FALSE;
    tmp_entry->rreq_id = 0;
    tmp_entry->metric = 0;
    tmp_entry->seq = 0;
    tmp_entry->ip = ip;
    tmp_entry->next_hop = ip;
    tmp_entry->dev = NULL;
    tmp_entry->route_valid = FALSE;
    tmp_entry->netmask = g_broadcast_ip;
    tmp_entry->route_seq_valid = FALSE;
    tmp_entry->prev = NULL;
    tmp_entry->next = NULL;

    if (ip)
    {
        insert_aodv_route(tmp_entry);
    }

    return tmp_entry;
}


int AodvRoutingTable::update_aodv_route(
    const unsigned int ip,
    const unsigned int next_hop_ip,
    const unsigned char metric,
    const unsigned int seq,
    struct net_device *dev)
{

    aodv_route *tmp_route;
    //ScenSim-Port://u_int64_t curr_time;
    long long int curr_time;

    /*lock table */


    tmp_route = (*this).find_aodv_route(ip);    /* Get eprev_route->next->prev = new_route;ntry from RT if there is one */

    if (!aodvNeighborPtr->valid_aodv_neigh(next_hop_ip))
    {
        //ScenSim-Port://printk(KERN_INFO "AODV: Failed to update route: %s \n", inet_ntoa(ip));
        return 0;
    }

    if (tmp_route && seq_greater(tmp_route->seq, seq))
    {
        return 0;
    }

    if (tmp_route && tmp_route->route_valid)
    {
        if ((seq == tmp_route->seq) && (metric >= tmp_route->metric))
        {
            return 0;
        }
    }

    if (tmp_route == NULL)
    {
        tmp_route = (*this).create_aodv_route(ip);

        if (tmp_route == NULL)
            return -ENOMEM;
        tmp_route->ip = ip;

    }



    if (tmp_route->self_route)
    {
        //ScenSim-Port://printk("updating a SELF-ROUTE!!! %s hopcount %d\n", inet_ntoa(next_hop_ip), metric);
        if (!tmp_route->route_valid)
            printf("because route was invalid!\n");

        if (!tmp_route->route_seq_valid)
            printf("because seq was invalid!\n");

        if (seq_greater(seq, tmp_route->seq))
            printf("because seq of route was lower!\n");
        if ((seq == tmp_route->seq) && (metric < tmp_route->metric))
            printf("becase seq same but hop lower!\n");
    }

    /* Update values in the RT entry */
    tmp_route->seq = seq;
    tmp_route->next_hop = next_hop_ip;
    tmp_route->metric = metric;
    tmp_route->dev =   dev;
    tmp_route->route_valid = TRUE;
    tmp_route->route_seq_valid = TRUE;

    //ScenSim-Port://delete_kernel_route_entry(tmp_route->ip, tmp_route->next_hop, tmp_route->netmask);  
    //ScenSim-Port://insert_kernel_route_entry(tmp_route->ip, tmp_route->next_hop, tmp_route->netmask, tmp_route->dev->name);
    kernelAodvProtocolPtr->DeleteRoutingTableEntry(
        tmp_route->ip,
        tmp_route->next_hop,
        tmp_route->netmask);
    kernelAodvProtocolPtr->AddRoutingTableEntry(
        tmp_route->ip,
        tmp_route->next_hop,
        tmp_route->netmask,
        tmp_route->dev->name);
    //ScenSim-Port://ipq_send_ip(ip);
    //We do not assume packe queueing before no existing route.

    //ScenSim-Port://curr_time = getcurrtime();  /* Get current time */
    curr_time = simulationEngineInterfacePtr->CurrentTime();
    /* Check if the lifetime in RT is valid, if not update it */

    tmp_route->lifetime =  curr_time + kernelAodvProtocolPtr->GetAodvActiveRouteTimeout();


    return 0;
}


int AodvRoutingTable::compare_aodv_route(
    aodv_route * tmp_route,
    const unsigned int target_ip)
{

    if ((tmp_route->ip & tmp_route->netmask)
        == (target_ip & tmp_route->netmask)) {
        return 1;
    }
    else {

        return 0;

    }

}


aodv_route* AodvRoutingTable::find_aodv_route(const unsigned int target_ip)
{
    //ScenSim-Port://aodv_route *tmp_route, *dead_route;
    aodv_route *tmp_route;
    aodv_route *possible_route = NULL;
    //ScenSim-Port://u_int64_t curr_time = getcurrtime();
    long long int curr_time = simulationEngineInterfacePtr->CurrentTime();//temporarily//

    /*lock table */
    //ScenSim-Port://route_read_lock();

    tmp_route = aodv_route_table;


    while ((tmp_route != NULL) && (tmp_route->ip <= target_ip))
    {
        if ((time_before(tmp_route->lifetime, curr_time))
            &&(!tmp_route->self_route)
            && (tmp_route->route_valid))
        {

            expire_aodv_route(tmp_route);

        }

        if (compare_aodv_route(tmp_route, target_ip))
        {
            //ScenSim-Port://printk("it looks like the route %s",inet_ntoa(tmp_route->ip));
            //ScenSim-Port://printk("is equal to: %s\n",inet_ntoa(target_ip));
            possible_route = tmp_route;
        }
        tmp_route = tmp_route->next;

    }
    /*unlock table */
    //ScenSim-Port://route_read_unlock();

    return possible_route;
}


int AodvRoutingTable::find_metric(const unsigned int tmp_ip)
{

    return 1;
}


//ScenSim-Port://int read_route_table_proc(char *buffer, char **buffer_location, off_t offset, int buffer_length,int *eof,void *data)
//ScenSim-Port://{
//ScenSim-Port://
//ScenSim-Port://    static char *my_buffer;
//ScenSim-Port://    char temp_buffer[200];
//ScenSim-Port://    char temp[100];
//ScenSim-Port://    aodv_route *tmp_entry;
//ScenSim-Port://    u_int64_t remainder, numerator;
//ScenSim-Port://    u_int64_t tmp_time;
//ScenSim-Port://    int len,i;
//ScenSim-Port://    u_int64_t currtime;
//ScenSim-Port://    char dst[16];
//ScenSim-Port://    char hop[16];
//ScenSim-Port://
//ScenSim-Port://    currtime=getcurrtime();
//ScenSim-Port://
//ScenSim-Port://    /*lock table*/
//ScenSim-Port://    route_read_lock();
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    tmp_entry = aodv_route_table;
//ScenSim-Port://
//ScenSim-Port://    my_buffer=buffer;
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    sprintf(my_buffer,"\nRoute Table \n---------------------------------------------------------------------------------\n");
//ScenSim-Port://    sprintf(temp_buffer,"        IP        |    Seq    |   Hop Count  |     Next Hop  \n");
//ScenSim-Port://    strcat(my_buffer,temp_buffer);
//ScenSim-Port://    sprintf(temp_buffer,"---------------------------------------------------------------------------------\n");
//ScenSim-Port://    strcat(my_buffer,temp_buffer);
//ScenSim-Port://
//ScenSim-Port://    while (tmp_entry!=NULL)
//ScenSim-Port://    {
//ScenSim-Port://        strcpy(hop,inet_ntoa(tmp_entry->next_hop));
//ScenSim-Port://        strcpy(dst,inet_ntoa(tmp_entry->ip));
//ScenSim-Port://        sprintf(temp_buffer,"  %-16s     %5u       %3d         %-16s ",dst ,tmp_entry->seq,tmp_entry->metric,hop);
//ScenSim-Port://        strcat(my_buffer,temp_buffer);
//ScenSim-Port://
//ScenSim-Port://   if (tmp_entry->self_route)
//ScenSim-Port://     {
//ScenSim-Port://       strcat( my_buffer, " Self Route \n");
//ScenSim-Port://     }
//ScenSim-Port://   else
//ScenSim-Port://     {
//ScenSim-Port://       if (tmp_entry->route_valid)
//ScenSim-Port://         strcat(my_buffer, " Valid ");
//ScenSim-Port://
//ScenSim-Port://       tmp_time=tmp_entry->lifetime-currtime;
//ScenSim-Port://       numerator = (tmp_time);
//ScenSim-Port://       remainder = do_div( numerator, 1000 );
//ScenSim-Port://       if (time_before(tmp_entry->lifetime, currtime) )
//ScenSim-Port://         {
//ScenSim-Port://       sprintf(temp," Expired!\n");
//ScenSim-Port://         }
//ScenSim-Port://       else
//ScenSim-Port://         {
//ScenSim-Port://       sprintf(temp," sec/msec: %lu/%lu \n", (unsigned long)numerator, (unsigned long)remainder);
//ScenSim-Port://         }
//ScenSim-Port://           strcat(my_buffer,temp);
//ScenSim-Port://     }
//ScenSim-Port://
//ScenSim-Port://   tmp_entry=tmp_entry->next;
//ScenSim-Port://
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port:///*unlock table*/
//ScenSim-Port://route_read_unlock();
//ScenSim-Port://
//ScenSim-Port://    strcat(my_buffer,"---------------------------------------------------------------------------------\n\n");
//ScenSim-Port://
//ScenSim-Port://    len = strlen(my_buffer);
//ScenSim-Port://    *buffer_location = my_buffer + offset;
//ScenSim-Port://    len -= offset;
//ScenSim-Port://    if (len > buffer_length)
//ScenSim-Port://        len = buffer_length;
//ScenSim-Port://    else if (len < 0)
//ScenSim-Port://        len = 0;
//ScenSim-Port://    return len;
//ScenSim-Port://}

}//namespace//
