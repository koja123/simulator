/***************************************************************************
                          flood_id.c  -  description
                             -------------------
    begin                : Mon Aug 4 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#include "kernelaodvglue.h"
#include "flood_id.h"

/****************************************************

   flood_id_queue
----------------------------------------------------
This is used to keep track of messages which
are flooded to prevent rebroadcast of messages
****************************************************/

//ScenSim-Port://flood_id *flood_id_queue;
//ScenSim-Port://rwlock_t flood_id_lock = RW_LOCK_UNLOCKED;
//ScenSim-Port://extern u_int32_t g_my_ip;

/****************************************************

   init_flood_id_queue
----------------------------------------------------
Gets the ball rolling!
****************************************************/

namespace KernelAodvPort {


void AodvFloodId::init_flood_id_queue()
{
    flood_id_queue = NULL;
    //ScenSim-Port://return 0;
}

//ScenSim-Port://void flood_id_read_lock()
//ScenSim-Port://{
//ScenSim-Port://    read_lock_bh(&flood_id_lock);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://void flood_id_read_unlock()
//ScenSim-Port://{
//ScenSim-Port://    read_unlock_bh(&flood_id_lock);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://void flood_id_write_lock()
//ScenSim-Port://{
//ScenSim-Port://    write_lock_bh(&flood_id_lock);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://void flood_id_write_unlock()
//ScenSim-Port://{
//ScenSim-Port://    write_unlock_bh(&flood_id_lock);
//ScenSim-Port://}


/****************************************************

   find_flood_id_queue_entry
----------------------------------------------------
will search the queue for an entry with the
matching ID and src_ip
****************************************************/
flood_id* AodvFloodId::find_flood_id(
    const unsigned int src_ip,
    const unsigned int id)
{
    flood_id *tmp_flood_id;     /* Working entry in the RREQ list */

    //ScenSim-Port://u_int64_t curr = getcurrtime();
    long long int curr = simulationEngineInterfacePtr->CurrentTime();

    /*lock table */
    //ScenSim-Port://flood_id_read_lock();

    tmp_flood_id = flood_id_queue;      /* Start at the header */

    //go through the whole queue
    while (tmp_flood_id != NULL)
    {
        if (time_before(tmp_flood_id->lifetime, curr))
        {
            /*unlock table */


            //ScenSim-Port://flood_id_read_unlock();

            return NULL;
        }
        //if there is a match and it is still valid
        if ((src_ip == tmp_flood_id->src_ip) && (id == tmp_flood_id->id))
        {
            /*unlock table */


            //ScenSim-Port://flood_id_read_unlock();

            return tmp_flood_id;
        }
        //continue on to the next entry
        tmp_flood_id = tmp_flood_id->next;



    }
    /*unlock table */


    //ScenSim-Port://flood_id_read_unlock();

    return NULL;
}


/****************************************************

   read_flood_id_proc
----------------------------------------------------
prints out the flood id queue when the proc
file is read
****************************************************/
//ScenSim-Port://int read_flood_id_proc(char *buffer, char **buffer_location, off_t offset, int buffer_length,int *eof,void *data)
//ScenSim-Port://{
//ScenSim-Port://  int len;
//ScenSim-Port://  static char *my_buffer;
//ScenSim-Port://  char temp_buffer[200];
//ScenSim-Port://  flood_id *tmp_entry;
//ScenSim-Port://  char temp_ip[16];
//ScenSim-Port://  u_int64_t remainder, numerator;
//ScenSim-Port://  u_int64_t tmp_time, currtime;
//ScenSim-Port://  
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    //lock table
//ScenSim-Port://    flood_id_read_lock();
//ScenSim-Port://
//ScenSim-Port://  tmp_entry=flood_id_queue;
//ScenSim-Port://
//ScenSim-Port://  my_buffer=buffer;
//ScenSim-Port://  currtime = getcurrtime();
//ScenSim-Port://  
//ScenSim-Port://  sprintf(my_buffer,"\nFlood Id Queue\n---------------------------------\n");
//ScenSim-Port://
//ScenSim-Port://  while (tmp_entry!=NULL)
//ScenSim-Port://    {
//ScenSim-Port://      strcpy(temp_ip,inet_ntoa(tmp_entry->dst_ip));
//ScenSim-Port://      sprintf(temp_buffer,"Src IP: %-16s  Dst IP: %-16s Flood ID: %-10u ", inet_ntoa(tmp_entry->src_ip),temp_ip,tmp_entry->id);
//ScenSim-Port://      strcat(my_buffer,temp_buffer);
//ScenSim-Port://
//ScenSim-Port://       tmp_time=tmp_entry->lifetime - currtime;
//ScenSim-Port://       numerator = (tmp_time);
//ScenSim-Port://       remainder = do_div( numerator, 1000 );
//ScenSim-Port://       if (time_before(tmp_entry->lifetime, currtime) )
//ScenSim-Port://         {
//ScenSim-Port://       sprintf(temp_buffer," Expired!\n");
//ScenSim-Port://         }
//ScenSim-Port://       else
//ScenSim-Port://         {
//ScenSim-Port://       sprintf(temp_buffer," sec/msec: %lu/%lu \n", (unsigned long)numerator, (unsigned long)remainder);
//ScenSim-Port://         }
//ScenSim-Port://      strcat(my_buffer,temp_buffer);
//ScenSim-Port://
//ScenSim-Port://      tmp_entry=tmp_entry->next;
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    //unlock table
//ScenSim-Port://    flood_id_read_unlock();
//ScenSim-Port://
//ScenSim-Port://    sprintf(temp_buffer,"\n---------------------------------\n");
//ScenSim-Port://    strcat(my_buffer,temp_buffer);
//ScenSim-Port://
//ScenSim-Port://    *buffer_location=my_buffer;
//ScenSim-Port://    len = strlen(my_buffer);
//ScenSim-Port://    if (len <= offset+buffer_length) *eof = 1;
//ScenSim-Port://    *buffer_location = my_buffer + offset;
//ScenSim-Port://    len -= offset;
//ScenSim-Port://    if (len>buffer_length) len = buffer_length;
//ScenSim-Port://    if (len<0) len = 0;
//ScenSim-Port://    return len;
//ScenSim-Port://
//ScenSim-Port://}




/****************************************************

   clean_up_flood_id_queue
----------------------------------------------------
Deletes everything in the flood id queue
****************************************************/
//ScenSim-Port://void cleanup_flood_id_queue()
//ScenSim-Port://{
//ScenSim-Port://    flood_id *tmp_flood_id, *dead_flood_id;
//ScenSim-Port://    int count = 0;
//ScenSim-Port://
//ScenSim-Port://    tmp_flood_id = flood_id_queue;
//ScenSim-Port://
//ScenSim-Port://    // print_flood_id_queue();
//ScenSim-Port://
//ScenSim-Port://    while (tmp_flood_id != NULL)
//ScenSim-Port://    {
//ScenSim-Port://        dead_flood_id = tmp_flood_id;
//ScenSim-Port://        tmp_flood_id = tmp_flood_id->next;
//ScenSim-Port://        kfree(dead_flood_id);
//ScenSim-Port://        count++;
//ScenSim-Port://    }
//ScenSim-Port://    flood_id_queue = NULL;
//ScenSim-Port://
//ScenSim-Port://    printk(KERN_INFO "Removed %d Flood ID entries! \n", count);
//ScenSim-Port://    printk(KERN_INFO "---------------------------------------------\n");
//ScenSim-Port://
//ScenSim-Port://}

/****************************************************

   insert_flood_id_queue_entry
----------------------------------------------------
Inserts an entry into the flood ID queue
****************************************************/

int AodvFloodId::insert_flood_id(
    const unsigned int src_ip,
    const unsigned int dst_ip,
    const unsigned int id,
    const long long int& lt)
{
    flood_id *new_flood_id = new flood_id();     /* Pointer to the working entry */


//ScenSim-Port://    /* Allocate memory for the new entry */
//ScenSim-Port://    if ((new_flood_id = (flood_id *) kmalloc(sizeof(flood_id), GFP_ATOMIC)) == NULL)
//ScenSim-Port://    {
//ScenSim-Port://        printk(KERN_WARNING "AODV: Not enough memory to create Flood ID queue\n");
//ScenSim-Port://
//ScenSim-Port://        /* Failed to allocate memory for new Flood ID queue */
//ScenSim-Port://        return -ENOMEM;
//ScenSim-Port://    }

    /* Fill in the information in the new entry */
    new_flood_id->src_ip = src_ip;
    new_flood_id->dst_ip = dst_ip;
    new_flood_id->id = id;
    new_flood_id->lifetime = lt;

//ScenSim-Port://    /*lock table */
//ScenSim-Port://    flood_id_write_lock();


    new_flood_id->next = flood_id_queue;
    /* Put the new entry in the list */
    flood_id_queue = new_flood_id;

//ScenSim-Port://    /*unlock table */
//ScenSim-Port://    flood_id_write_unlock();

    return 0;
}


int AodvFloodId::flush_flood_id_queue()
{
    flood_id *tmp_flood_id, *prev_flood_id, *dead_flood_id;
    //ScenSim-Port://u_int64_t curr_time = getcurrtime();        /* Current time */
    long long int curr_time = simulationEngineInterfacePtr->CurrentTime();
    int id_count = 0;


    /*lock table */
    //ScenSim-Port://flood_id_write_lock();


    tmp_flood_id = flood_id_queue;
    prev_flood_id = NULL;

    //go through the entire queue
    while (tmp_flood_id)
    {
        //if the entry has expired
        if (time_before(tmp_flood_id->lifetime, curr_time))
        {


            //if it is the first entry
            if (prev_flood_id == NULL)
                flood_id_queue = tmp_flood_id->next;
            else
                prev_flood_id->next = tmp_flood_id->next;

            //kill it!
            dead_flood_id = tmp_flood_id;
            tmp_flood_id = tmp_flood_id->next;
            //ScenSim-Port://kfree(dead_flood_id);
            id_count++;

        } else
        {


            //next entry
            prev_flood_id = tmp_flood_id;
            tmp_flood_id = tmp_flood_id->next;


        }
    }

    /*unlock table */
    //ScenSim-Port://flood_id_write_unlock();
    //ScenSim-Port://insert_timer( TASK_CLEANUP, HELLO_INTERVAL, g_my_ip);

    unsigned int g_my_ip = kernelAodvProtocolPtr->GetIpAddressLow32Bits();
    aodvScheduleTimerPtr->insert_timer(
        TASK_CLEANUP,
        static_cast<unsigned int>(kernelAodvProtocolPtr->GetAodvHelloInterval() / ScenSim::MILLI_SECOND),
        g_my_ip);
    //ScenSim-Port://update_timer_queue();

    return id_count;
}

}//namespace//
