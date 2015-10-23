/***************************************************************************
                          aodv_neigh.c  -  description
                             -------------------
    begin                : Thu Jul 31 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#include "kernelaodvglue.h"
#include "aodv_neigh.h"
#include "aodv_route.h"

//ScenSim-Port://aodv_neigh *aodv_neigh_list;
//ScenSim-Port://#ifdef LINK_LIMIT
//ScenSim-Port://extern int g_link_limit;
//ScenSim-Port://#endif
//ScenSim-Port://rwlock_t neigh_lock = RW_LOCK_UNLOCKED;

namespace KernelAodvPort {


//ScenSim-Port://inline void neigh_read_lock()
//ScenSim-Port://{
//ScenSim-Port://    read_lock_bh(&neigh_lock);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://inline void neigh_read_unlock()
//ScenSim-Port://{
//ScenSim-Port://    read_unlock_bh(&neigh_lock);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://inline void neigh_write_lock()
//ScenSim-Port://{
//ScenSim-Port://    write_lock_bh(&neigh_lock);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://inline void neigh_write_unlock()
//ScenSim-Port://{
//ScenSim-Port://
//ScenSim-Port://    write_unlock_bh(&neigh_lock);
//ScenSim-Port://}



void AodvNeighbor::init_aodv_neigh_list()
{
    aodv_neigh_list = NULL;
}


aodv_neigh* AodvNeighbor::find_aodv_neigh(const unsigned int target_ip)
{
    aodv_neigh *tmp_neigh;

    //ScenSim-Port://neigh_read_lock();
    tmp_neigh = aodv_neigh_list;

    while ((tmp_neigh != NULL) && (tmp_neigh->ip <= target_ip))
    {
        if (tmp_neigh->ip == target_ip)
        {
            //ScenSim-Port://neigh_read_unlock();
            return tmp_neigh;
        }
        tmp_neigh = tmp_neigh->next;
    }
    //ScenSim-Port://neigh_read_unlock();
    return NULL;
}


int AodvNeighbor::valid_aodv_neigh(const unsigned int target_ip)
{
    aodv_neigh *tmp_neigh;

    tmp_neigh = find_aodv_neigh(target_ip);

    if (tmp_neigh)
    {
        if (tmp_neigh->valid_link &&
            time_before(
                simulationEngineInterfacePtr->CurrentTime(),
                tmp_neigh->lifetime)) {
            return 1;
        }
        else {
            return 0;

        }
    }
    else {
        return 0;
    }
}


//ScenSim-Port://aodv_neigh *find_aodv_neigh_by_hw(char *hw_addr)
//ScenSim-Port://{
//ScenSim-Port://    aodv_neigh *tmp_neigh = aodv_neigh_list;
//ScenSim-Port://
//ScenSim-Port://    while (tmp_neigh != NULL)
//ScenSim-Port://    {
//ScenSim-Port://        if (!memcmp(&(tmp_neigh->hw_addr), hw_addr, ETH_ALEN))
//ScenSim-Port://            return tmp_neigh;
//ScenSim-Port://        tmp_neigh = tmp_neigh->next;
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    return NULL;
//ScenSim-Port://}


//ScenSim-Port://void update_aodv_neigh_link(char *hw_addr, u_int8_t link)
//ScenSim-Port://{
//ScenSim-Port://
//ScenSim-Port://    aodv_neigh *tmp_neigh;
//ScenSim-Port://    u_int8_t link_temp;
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://       neigh_write_lock();
//ScenSim-Port://
//ScenSim-Port://    tmp_neigh = aodv_neigh_list;
//ScenSim-Port://
//ScenSim-Port://    //search the interface list for a device with the same ip
//ScenSim-Port://    while (tmp_neigh)
//ScenSim-Port://    {
//ScenSim-Port://        if (!memcmp(&(tmp_neigh->hw_addr), hw_addr, ETH_ALEN))
//ScenSim-Port://        {
//ScenSim-Port://           if (link)
//ScenSim-Port://            tmp_neigh->link = 0x100 - link;
//ScenSim-Port://          else
//ScenSim-Port://            tmp_neigh->link = 0;
//ScenSim-Port://           break;
//ScenSim-Port://        }
//ScenSim-Port://        tmp_neigh = tmp_neigh->next;
//ScenSim-Port://    }
//ScenSim-Port://       neigh_write_unlock();
//ScenSim-Port://}


int AodvNeighbor::delete_aodv_neigh(
    AodvRoutingTable* aodvRoutingTablePtr,
    const unsigned int ip)
{
    aodv_neigh *tmp_neigh;
    aodv_neigh *prev_neigh = NULL;
    aodv_route *tmp_route;

    //ScenSim-Port://neigh_write_lock();
    tmp_neigh = aodv_neigh_list;

    while (tmp_neigh != NULL)
    {
        if (tmp_neigh->ip == ip)
        {

            if (prev_neigh != NULL)
            {
                prev_neigh->next = tmp_neigh->next;
            } else
            {
                aodv_neigh_list = tmp_neigh->next;
            }

            //ScenSim-Port://kfree(tmp_neigh);

            //ScenSim-Port://delete_timer(ip, TASK_NEIGHBOR);
            aodvScheduleTimerPtr->delete_timer(TASK_NEIGHBOR, ip);


            //ScenSim-Port://update_timer_queue();
            //ScenSim-Port://neigh_write_unlock();

            tmp_route = aodvRoutingTablePtr->find_aodv_route(ip);
            if (tmp_route && (tmp_route->next_hop == tmp_route->ip))
            {
                //ScenSim-Port://delete_kernel_route_entry(tmp_route->ip, tmp_route->next_hop, tmp_route->netmask);
                gen_rerr(
                    kernelAodvProtocolPtr,
                    aodvRoutingTablePtr,
                    tmp_route->ip);

                if (tmp_route->route_valid)
                {
                    aodvRoutingTablePtr->expire_aodv_route(tmp_route);
                }
                else
                {
                    aodvRoutingTablePtr->remove_aodv_route(tmp_route);
                }
            }

            return 0;
        }
        prev_neigh = tmp_neigh;
        tmp_neigh = tmp_neigh->next;
    }
    //ScenSim-Port://neigh_write_unlock();

    //ScenSim-Port://return -ENODATA;
    return -1;
}


aodv_neigh* AodvNeighbor::create_aodv_neigh(const unsigned int ip)
{
    aodv_neigh *new_neigh = new aodv_neigh();
    aodv_neigh *prev_neigh = NULL;
    aodv_neigh *tmp_neigh = NULL;
    int i = 0;

    //ScenSim-Port://neigh_write_lock();

    tmp_neigh = aodv_neigh_list;
    while ((tmp_neigh != NULL) && (tmp_neigh->ip < ip))
    {
        prev_neigh = tmp_neigh;
        tmp_neigh = tmp_neigh->next;
    }

    if (tmp_neigh && (tmp_neigh->ip == ip))
    {
        //printk(KERN_WARNING "AODV_NEIGH: Creating a duplicate neighbor\n");
        printf("AODV Warning: Now(%lld) my_ip(%u) create_aodv_neigh():AODV_NEIGH Creating a duplicate neighbor.\n",
               simulationEngineInterfacePtr->CurrentTime(),
               kernelAodvProtocolPtr->GetIpAddressLow32Bits());
        return NULL;
    }

    //ScenSim-Port://if ((new_neigh = kmalloc(sizeof(aodv_neigh), GFP_ATOMIC)) == NULL)
    //ScenSim-Port://{
    //ScenSim-Port://    printk(KERN_WARNING "NEIGHBOR_LIST: Can't allocate new entry\n");
    //ScenSim-Port://    return NULL;
    //ScenSim-Port://}

    new_neigh->ip = ip;
    new_neigh->lifetime = 0;
    new_neigh->route_entry = NULL;
    new_neigh->link = 0;

#ifdef LINK_LIMIT
    new_neigh->valid_link = FALSE;
#else
    new_neigh->valid_link = TRUE;
#endif

    new_neigh->next = NULL;

    if (prev_neigh == NULL)
    {

        new_neigh->next = aodv_neigh_list;
        aodv_neigh_list = new_neigh;
    } else
    {
        new_neigh->next = prev_neigh->next;
        prev_neigh->next = new_neigh;
    }


    //ScenSim-Port://neigh_write_unlock();

    return new_neigh;
}


void AodvNeighbor::update_aodv_neigh_route(
    AodvRoutingTable* aodvRoutingTablePtr,
    aodv_neigh * tmp_neigh,
    rrep *tmp_rrep)
{
    aodv_route *tmp_route;

    tmp_route = aodvRoutingTablePtr->find_aodv_route(tmp_neigh->ip);


    if (tmp_route)
    {
/*
        if (!tmp_route->route_valid)
        {
            insert_kernel_route_entry(tmp_route->ip, tmp_route->next_hop, tmp_route->netmask,tmp_route->dev->name);
        }

        if (tmp_route->next_hop!=tmp_route->ip)
        {
*/

//ScenSim-Port://        delete_kernel_route_entry(tmp_route->ip, tmp_route->next_hop, tmp_route->netmask);
//ScenSim-Port://        insert_kernel_route_entry(tmp_route->ip, tmp_route->next_hop, tmp_route->netmask,tmp_route->dev->name);
        kernelAodvProtocolPtr->DeleteRoutingTableEntry(
            tmp_route->ip,
            tmp_route->next_hop,
            tmp_route->netmask);
        kernelAodvProtocolPtr->AddRoutingTableEntry(
            tmp_route->ip,
            tmp_route->next_hop,
            tmp_route->netmask,
            tmp_route->dev->name);

        //}
    }
    else
    {
            //ScenSim-Port://printk(KERN_INFO "AODV: Creating route for neighbor: %s Link: %d\n", inet_ntoa(tmp_neigh->ip), tmp_neigh->link);
        tmp_route = aodvRoutingTablePtr->create_aodv_route(tmp_neigh->ip);

//ScenSim-Port://           delete_kernel_route_entry(tmp_route->ip, tmp_route->next_hop, tmp_route->netmask);
//ScenSim-Port://            insert_kernel_route_entry(tmp_route->ip, tmp_route->next_hop, tmp_route->netmask,tmp_route->dev->name);
        kernelAodvProtocolPtr->DeleteRoutingTableEntry(
            tmp_route->ip,
            tmp_route->next_hop,
            tmp_route->netmask);
        kernelAodvProtocolPtr->AddRoutingTableEntry(
            tmp_route->ip,
            tmp_route->next_hop,
            tmp_route->netmask,
            tmp_route->dev->name);
    }


    //ScenSim-Port://tmp_route->lifetime = getcurrtime() + tmp_rrep->lifetime
    tmp_route->lifetime = simulationEngineInterfacePtr->CurrentTime() + tmp_rrep->lifetime * ScenSim::MILLI_SECOND;
    tmp_route->dev = tmp_neigh->dev;
    tmp_route->next_hop = tmp_neigh->ip;
    tmp_route->route_seq_valid = TRUE;
    tmp_route->route_valid = TRUE;
    tmp_route->seq = tmp_rrep->dst_seq;
    tmp_neigh->route_entry = tmp_route;
    tmp_route->metric = static_cast<unsigned char>(aodvRoutingTablePtr->find_metric(tmp_route->ip));
}



int AodvNeighbor::update_aodv_neigh(
    AodvRoutingTable* aodvRoutingTablePtr,
    aodv_neigh *tmp_neigh,
    rrep *tmp_rrep)
{
    if (tmp_neigh==NULL)
    {
        //printk(KERN_WARNING "AODV: NULL neighbor passed!\n");
        printf("AODV Warning: AODV: NULL neighbor passed!\n");
        return 0;
    }

#ifdef LINK_LIMIT
    if (tmp_neigh->link > (g_link_limit))
    {
        tmp_neigh->valid_link = TRUE;
    }
    /*if (tmp_neigh->link < (g_link_limit - 5))
    {
        tmp_neigh->valid_link = FALSE;
    }*/
#endif

    if (tmp_neigh->valid_link)
    {
        (*this).update_aodv_neigh_route(
            aodvRoutingTablePtr,
            tmp_neigh,
            tmp_rrep);
    }

    return 1;
}


//ScenSim-Port://aodv_neigh *first_aodv_neigh()
//ScenSim-Port://{
//ScenSim-Port://    return aodv_neigh_list;
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://int read_neigh_proc(char *buffer, char **buffer_location, off_t offset, int buffer_length,int *eof,void *data)
//ScenSim-Port://{
//ScenSim-Port://    char tmp_buffer[200];
//ScenSim-Port://    aodv_neigh *tmp_neigh;
//ScenSim-Port://    u_int64_t remainder, numerator;
//ScenSim-Port://    u_int64_t tmp_time;
//ScenSim-Port://    int len;
//ScenSim-Port://    u_int64_t currtime;
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    currtime=getcurrtime();
//ScenSim-Port://
//ScenSim-Port://       neigh_read_lock();
//ScenSim-Port://
//ScenSim-Port://    tmp_neigh = aodv_neigh_list;
//ScenSim-Port://
//ScenSim-Port://    sprintf(buffer,"\nAODV Neighbors \n---------------------------------------------------------------------------------\n");
//ScenSim-Port://    sprintf(tmp_buffer,"        IP        |   Link    |    Valid    |  Lifetime \n");
//ScenSim-Port://    strcat( buffer,tmp_buffer);
//ScenSim-Port://    sprintf(tmp_buffer,"---------------------------------------------------------------------------------\n");
//ScenSim-Port://    strcat( buffer,tmp_buffer);
//ScenSim-Port://    while (tmp_neigh!=NULL)
//ScenSim-Port://    {
//ScenSim-Port://         sprintf(tmp_buffer,"   %-16s     %d        ",inet_ntoa(tmp_neigh->ip) ,tmp_neigh->link );
//ScenSim-Port://        strcat(buffer,tmp_buffer);
//ScenSim-Port://
//ScenSim-Port://   if (tmp_neigh->valid_link)
//ScenSim-Port://     {
//ScenSim-Port://       strcat( buffer, "+       ");
//ScenSim-Port://     }
//ScenSim-Port://   else
//ScenSim-Port://     {
//ScenSim-Port://       strcat( buffer, "-       ");
//ScenSim-Port://       }
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://       tmp_time=tmp_neigh->lifetime-currtime;
//ScenSim-Port://       numerator = (tmp_time);
//ScenSim-Port://       remainder = do_div( numerator, 1000 );
//ScenSim-Port://       if (time_before(tmp_neigh->lifetime, currtime) )
//ScenSim-Port://         {
//ScenSim-Port://                   sprintf(tmp_buffer," Expired!\n");
//ScenSim-Port://         }
//ScenSim-Port://       else
//ScenSim-Port://         {
//ScenSim-Port://                   sprintf(tmp_buffer," sec/msec: %lu/%lu \n", (unsigned long)numerator, (unsigned long)remainder );
//ScenSim-Port://            }
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://   strcat(buffer,tmp_buffer);
//ScenSim-Port://   tmp_neigh=tmp_neigh->next;
//ScenSim-Port://
//ScenSim-Port://    }
//ScenSim-Port://  strcat(buffer,"---------------------------------------------------------------------------------\n\n");
//ScenSim-Port://
//ScenSim-Port://    len = strlen(buffer);
//ScenSim-Port://    *buffer_location = buffer + offset;
//ScenSim-Port://    len -= offset;
//ScenSim-Port://    if (len > buffer_length)
//ScenSim-Port://        len = buffer_length;
//ScenSim-Port://    else if (len < 0)
//ScenSim-Port://        len = 0;
//ScenSim-Port://
//ScenSim-Port://   neigh_read_unlock();
//ScenSim-Port://
//ScenSim-Port://    return len;
//ScenSim-Port://}

}//namespace//
