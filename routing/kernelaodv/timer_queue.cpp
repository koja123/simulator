/***************************************************************************
                          timer_queue.c  -  description
                             -------------------
    begin                : Mon Jul 14 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#include "kernelaodvglue.h"
#include "timer_queue.h"

namespace KernelAodvPort {
using std::cerr;//ScenSim-Port://
using std::endl;//ScenSim-Port://


//ScenSim-Port://struct timer_list aodv_timer;
//ScenSim-Port://rwlock_t timer_lock = RW_LOCK_UNLOCKED;
//ScenSim-Port://task *timer_queue;
//ScenSim-Port://unsigned long flags;



//ScenSim-Port://inline void timer_read_lock()
//ScenSim-Port://{
//ScenSim-Port://    read_lock_irqsave(&timer_lock, flags);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://inline void timer_read_unlock()
//ScenSim-Port://{
//ScenSim-Port://    read_unlock_irqrestore(&timer_lock, flags);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://inline void timer_write_lock()
//ScenSim-Port://{
//ScenSim-Port://    write_lock_irqsave(&timer_lock, flags);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://inline void timer_write_unlock()
//ScenSim-Port://{
//ScenSim-Port://    write_unlock_irqrestore(&timer_lock, flags);
//ScenSim-Port://}








//ScenSim-Port://int init_timer_queue()
//ScenSim-Port://{
//ScenSim-Port://    //ScenSim-Port://init_timer(&aodv_timer);
//ScenSim-Port://    timer_queue = NULL;
//ScenSim-Port://    return 0;
//ScenSim-Port://}


//ScenSim-Port://static unsigned long tvtojiffies(struct timeval *value)
//ScenSim-Port://{
//ScenSim-Port://    unsigned long sec = (unsigned) value->tv_sec;
//ScenSim-Port://    unsigned long usec = (unsigned) value->tv_usec;
//ScenSim-Port://
//ScenSim-Port://    if (sec > (ULONG_MAX / HZ))
//ScenSim-Port://        return ULONG_MAX;
//ScenSim-Port://    usec += 1000000 / HZ - 1;
//ScenSim-Port://    usec /= 1000000 / HZ;
//ScenSim-Port://    return HZ * sec + usec;
//ScenSim-Port://}


//ScenSim-Port://task *first_timer_due(u_int64_t currtime)
//ScenSim-Port://{
//ScenSim-Port://    task *tmp_task;
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    // lock Read
//ScenSim-Port://    timer_write_lock();
//ScenSim-Port://    if (timer_queue != NULL)
//ScenSim-Port://    {
//ScenSim-Port://
//ScenSim-Port://        /* If pqe's time is in teh interval */
//ScenSim-Port://        if (time_before_eq(timer_queue->time, currtime))
//ScenSim-Port://        {
//ScenSim-Port://               tmp_task = timer_queue;
//ScenSim-Port://            timer_queue = timer_queue->next;
//ScenSim-Port://            timer_write_unlock();
//ScenSim-Port://            return tmp_task;
//ScenSim-Port://        }
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    /* unlock read */
//ScenSim-Port://    timer_write_unlock();
//ScenSim-Port://    return NULL;
//ScenSim-Port://}


//ScenSim-Port://void timer_queue_signal()
//ScenSim-Port://{
//ScenSim-Port://    task *tmp_task;
//ScenSim-Port://    u_int64_t currtime;
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    // Get the first due entry in the queue /
//ScenSim-Port://    currtime = getcurrtime();
//ScenSim-Port://    tmp_task = first_timer_due(currtime);
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    // While there is still events that has timed out
//ScenSim-Port://    while (tmp_task != NULL)
//ScenSim-Port://    {
//ScenSim-Port://        insert_task_from_timer(tmp_task);
//ScenSim-Port://        tmp_task = first_timer_due(currtime);
//ScenSim-Port://    }
//ScenSim-Port://    update_timer_queue();
//ScenSim-Port://}

//ScenSim-Port://void update_timer_queue()
//ScenSim-Port://{
//ScenSim-Port://    struct timeval delay_time;
//ScenSim-Port://    u_int64_t currtime;
//ScenSim-Port://    u_int64_t tv;
//ScenSim-Port://    u_int64_t remainder, numerator;
//ScenSim-Port://
//ScenSim-Port://    delay_time.tv_sec = 0;
//ScenSim-Port://    delay_time.tv_usec = 0;
//ScenSim-Port://
//ScenSim-Port://    /* lock Read */
//ScenSim-Port://    timer_read_lock();
//ScenSim-Port://
//ScenSim-Port://    if (timer_queue == NULL)
//ScenSim-Port://    {
//ScenSim-Port://        // No event to set timer for
//ScenSim-Port://        delay_time.tv_sec = 0;
//ScenSim-Port://        delay_time.tv_usec = 0;
//ScenSim-Port://        del_timer(&aodv_timer);
//ScenSim-Port://        timer_read_unlock();
//ScenSim-Port://        return;
//ScenSim-Port://    } else
//ScenSim-Port://    {
//ScenSim-Port://        //* Get the first time value
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://        currtime = getcurrtime();
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://        if (time_before( timer_queue->time, currtime))
//ScenSim-Port://        {
//ScenSim-Port://            // If the event has allready happend, set the timeout to              1 microsecond :-)
//ScenSim-Port://
//ScenSim-Port://            delay_time.tv_sec = 0;
//ScenSim-Port://            delay_time.tv_usec = 1;
//ScenSim-Port://        } else
//ScenSim-Port://        {
//ScenSim-Port://            // Set the timer to the actual seconds / microseconds from now
//ScenSim-Port://
//ScenSim-Port://            //This is a fix for an error that occurs on ARM Linux Kernels because they do 64bits differently
//ScenSim-Port://            //Thanks to S. Peter Li for coming up with this fix!
//ScenSim-Port://
//ScenSim-Port://            numerator = (timer_queue->time - currtime);
//ScenSim-Port://            remainder = do_div(numerator, 1000);
//ScenSim-Port://
//ScenSim-Port://            delay_time.tv_sec = numerator;
//ScenSim-Port://            delay_time.tv_usec = remainder * 1000;
//ScenSim-Port://
//ScenSim-Port://        }
//ScenSim-Port://
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    if (!timer_pending(&aodv_timer))
//ScenSim-Port://    {
//ScenSim-Port://        aodv_timer.function = &timer_queue_signal;
//ScenSim-Port://        aodv_timer.expires = jiffies + tvtojiffies(&delay_time);
//ScenSim-Port://        //printk("timer sched in %u sec and %u milisec delay %u\n",delay_time.tv_sec, delay_time.tv_usec,aodv_timer.expires);
//ScenSim-Port://
//ScenSim-Port://        add_timer(&aodv_timer);
//ScenSim-Port://    } else
//ScenSim-Port://    {
//ScenSim-Port://        mod_timer(&aodv_timer, jiffies + tvtojiffies(&delay_time));
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    /* lock Read */
//ScenSim-Port://    timer_read_unlock();
//ScenSim-Port://
//ScenSim-Port://    // Set the timer (in real time)
//ScenSim-Port://    return;
//ScenSim-Port://}







//ScenSim-Port://void queue_timer(task * new_timer)
//ScenSim-Port://{
//ScenSim-Port://
//ScenSim-Port://    task *prev_timer = NULL;
//ScenSim-Port://    task *tmp_timer = NULL;
//ScenSim-Port://    //ScenSim-Port://u_int64_t currtime = getcurrtime();
//ScenSim-Port://
//ScenSim-Port://    /* lock Write */
//ScenSim-Port://    //ScenSim-Port://timer_write_lock();
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    tmp_timer = timer_queue;
//ScenSim-Port://
//ScenSim-Port://    while (tmp_timer != NULL && (time_after(new_timer->time,tmp_timer->time)))
//ScenSim-Port://    {
//ScenSim-Port://       //printk("%d is larger than %s type: %d time dif of %d \n", new_timer->type,inet_ntoa(tmp_timer->id),tmp_timer->type, new_timer->time-tmp_timer->time);
//ScenSim-Port://        prev_timer = tmp_timer;
//ScenSim-Port://        tmp_timer = tmp_timer->next;
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    if ((timer_queue == NULL) || (timer_queue == tmp_timer))
//ScenSim-Port://    {
//ScenSim-Port://        new_timer->next = timer_queue;
//ScenSim-Port://        timer_queue = new_timer;
//ScenSim-Port://    }
//ScenSim-Port://    else {
//ScenSim-Port://        if (tmp_timer == NULL)  // If at the end of the List
//ScenSim-Port://        {
//ScenSim-Port://            new_timer->next = NULL;
//ScenSim-Port://            prev_timer->next = new_timer;
//ScenSim-Port://        }
//ScenSim-Port://        else                  // Inserting in to the middle of the list somewhere
//ScenSim-Port://        {
//ScenSim-Port://            new_timer->next = prev_timer->next;
//ScenSim-Port://            prev_timer->next = new_timer;
//ScenSim-Port://        }
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    /* unlock Write */
//ScenSim-Port://    //ScenSim-Port://timer_write_unlock();
//ScenSim-Port://}

/****************************************************

   insert_timer_queue_entry
----------------------------------------------------
Insert an event into the queue. Also allocates
enough room for the data and copies that too
****************************************************/
int AodvScheduleTimer::insert_timer(
    const unsigned char task_type,
    const unsigned int delay,
    const unsigned int ip)
{
    task *new_entry;

    new_entry = create_task(task_type);

    // get memory
    if (new_entry == NULL)
    {
        //ScenSim-Port://printk(KERN_WARNING "AODV: Error allocating timer!\n");
        //ScenSim-Port://return -ENOMEM;
        cerr << "Error: AodvScheduleTimer did not create a task." << endl;
        exit(1);
    }

    new_entry->src_ip = ip;
    new_entry->dst_ip = ip;
    new_entry->id = ip;

    new_entry->time = simulationEngineInterfacePtr->CurrentTime() + delay * ScenSim::MILLI_SECOND;
    //ScenSim-Port://queue_timer(new_entry);
    kernelAodvProtocolPtr->ScheduleEventForAodvRouting(new_entry);
    return 0;
}


int AodvScheduleTimer::insert_rreq_timer(
    rreq * tmp_rreq,
    const unsigned char retries)
{
//ScenSim-Port://    task *new_timer = new task();
//ScenSim-Port://    // get memory
//ScenSim-Port://    if (!(new_timer = create_task(TASK_RESEND_RREQ)))
//ScenSim-Port://    {
//ScenSim-Port://        //ScenSim-Potr://printk(KERN_WARNING "AODV: Error allocating timer!\n");
//ScenSim-Port://
//ScenSim-Port://        return -ENOMEM;
//ScenSim-Port://    }
    task* new_timer = create_task(TASK_RESEND_RREQ);

    new_timer->src_ip = tmp_rreq->src_ip;
    new_timer->dst_ip = tmp_rreq->dst_ip;
    new_timer->id = tmp_rreq->dst_ip;
    new_timer->retries = retries;
    new_timer->ttl = 30;
    new_timer->time = simulationEngineInterfacePtr->CurrentTime()
      + ((2 ^ (kernelAodvProtocolPtr->GetAodvRreqRetries() - retries)) * kernelAodvProtocolPtr->GetAodvNetTraversalTime());
    //ScenSim-Port://queue_timer(new_timer);

    if (new_timer->time > simulationEngineInterfacePtr->CurrentTime()) {
        kernelAodvProtocolPtr->ScheduleEventForAodvRouting(new_timer);
    }

    return 0;
}



//ScenSim-Port:///****************************************************
//ScenSim-Port://
//ScenSim-Port://   find_first_timer_qu2 ^ (RREQ_RETRIES - retries)eue_entry
//ScenSim-Port://----------------------------------------------------
//ScenSim-Port://Returns the first entry in the timer queue
//ScenSim-Port://****************************************************/
//ScenSim-Port://task *find_first_timer_queue_entry()
//ScenSim-Port://{
//ScenSim-Port://    return timer_queue;
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port:///****************************************************
//ScenSim-Port://
//ScenSim-Port://   find_first_timer_queue_entry_of_id
//ScenSim-Port://----------------------------------------------------
//ScenSim-Port://Returns the first timer queue entry with a matching
//ScenSim-Port://ID
//ScenSim-Port://****************************************************/
//ScenSim-Port:///*
//ScenSim-Port://struct timer_queue_entry * find_first_timer_queue_entry_of_id(u_int32_t id)
//ScenSim-Port://{
//ScenSim-Port://    struct timer_queue_entry *tmp_entry;
//ScenSim-Port://
//ScenSim-Port://    // lock Read 
//ScenSim-Port://    timer_read_lock();
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    tmp_entry=timer_queue;
//ScenSim-Port://
//ScenSim-Port://    while (tmp_entry != NULL && tmp_entry->id != id)
//ScenSim-Port://        tmp_entry=tmp_entry->next;
//ScenSim-Port://
//ScenSim-Port://    // unlock Read 
//ScenSim-Port://    timer_read_unlock();
//ScenSim-Port://
//ScenSim-Port://    return tmp_entry;
//ScenSim-Port://}*/


//ScenSim-Port://task* AodvScheduleTimer::find_timer(const unsigned int& id,
//ScenSim-Port://                                    const unsigned char& type)

bool AodvScheduleTimer::find_timer(
    const unsigned int id,
    const unsigned char type)
{
    return kernelAodvProtocolPtr->SearchEventTicket(type, id);

//ScenSim-Port://    task *tmp_task;
//ScenSim-Port://    // lock Read
//ScenSim-Port://
//ScenSim-Port://    //ScenSim-Port://timer_read_lock();
//ScenSim-Port://    tmp_task = timer_queue;
//ScenSim-Port://    while (tmp_task != NULL)
//ScenSim-Port://    {
//ScenSim-Port://        if ((tmp_task->id == id) && (tmp_task->type == type))
//ScenSim-Port://        {
//ScenSim-Port://            timer_read_unlock();
//ScenSim-Port://            return tmp_task;
//ScenSim-Port://        }
//ScenSim-Port://        tmp_task = tmp_task->next;
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    // unlock Read 
//ScenSim-Port://    //ScenSim-Port://timer_read_unlock();
//ScenSim-Port://    return NULL;
}

/****************************************************

   delete_timer_queue_entry_of_id
----------------------------------------------------
Deletes the first entry with a matching id
****************************************************/

void AodvScheduleTimer::delete_timer(
    const unsigned char task_type,
    const unsigned int id)
{
    kernelAodvProtocolPtr->CancelReservedEvents(
        task_type,
        id);

//ScenSim-Port://    task *tmp_task;
//ScenSim-Port://    task *prev_task;
//ScenSim-Port://    task *dead_task;
//ScenSim-Port://    /* lock Write */
//ScenSim-Port://
//ScenSim-Port://    //printk("deleting timer: %s  type: %u", inet_ntoa(id), type);
//ScenSim-Port://    //ScenSim-Port://timer_write_lock();
//ScenSim-Port://    tmp_task = timer_queue;
//ScenSim-Port://    prev_task = NULL;
//ScenSim-Port://    while (tmp_task != NULL)
//ScenSim-Port://    {
//ScenSim-Port://        if ((tmp_task->id == id) && (tmp_task->type == type))
//ScenSim-Port://        {
//ScenSim-Port://            if (prev_task == NULL)
//ScenSim-Port://            {
//ScenSim-Port://                timer_queue = tmp_task->next;
//ScenSim-Port://            } else
//ScenSim-Port://            {
//ScenSim-Port://                prev_task->next = tmp_task->next;
//ScenSim-Port://            }
//ScenSim-Port://
//ScenSim-Port://            dead_task = tmp_task;
//ScenSim-Port://            tmp_task = tmp_task->next;
//ScenSim-Port://            //ScenSim-Port://kfree(dead_task->data);
//ScenSim-Port://            //ScenSim-Port://kfree(dead_task);
//ScenSim-Port://        } else
//ScenSim-Port://        {
//ScenSim-Port://            prev_task = tmp_task;
//ScenSim-Port://            tmp_task = tmp_task->next;
//ScenSim-Port://        }
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    /* unlock Write */
//ScenSim-Port://    //ScenSim-Port://timer_write_unlock();
//ScenSim-Port://    //update_timer_queue();
}

//ScenSim-Port://int read_timer_queue_proc(char *buffer, char **buffer_location, off_t offset, int buffer_length,int *eof,void *data)
//ScenSim-Port://{
//ScenSim-Port://    task *tmp_task;
//ScenSim-Port://       u_int64_t remainder, numerator;
//ScenSim-Port://    u_int64_t tmp_time;
//ScenSim-Port://    u_int64_t currtime = getcurrtime();
//ScenSim-Port://    int len;
//ScenSim-Port://    char tmp_buffer[200];
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    /* lock Read */
//ScenSim-Port://    timer_read_lock();
//ScenSim-Port://
//ScenSim-Port://    sprintf(buffer,"\nTimer Queue\n-------------------------------------------------------\n");
//ScenSim-Port://       strcat(buffer,"       ID      |     Type     |   sec/msec   |   Retries\n");
//ScenSim-Port://       strcat(buffer,"-------------------------------------------------------\n");
//ScenSim-Port://    tmp_task = timer_queue;
//ScenSim-Port://
//ScenSim-Port://    while (tmp_task != NULL)
//ScenSim-Port://    {
//ScenSim-Port://
//ScenSim-Port://   //This is a fix for an error that occurs on ARM Linux Kernels because they do 64bits differently
//ScenSim-Port://   //Thanks printk(" timer has %d left on it\n", timer_queue->time - currtime);
//ScenSim-Port://   // to S. Peter Li for coming up with this fix!
//ScenSim-Port://   sprintf( tmp_buffer,"    %-16s  ", inet_ntoa(tmp_task->dst_ip));
//ScenSim-Port://   strcat(buffer, tmp_buffer);
//ScenSim-Port://
//ScenSim-Port://   switch (tmp_task->type)
//ScenSim-Port://            {
//ScenSim-Port://
//ScenSim-Port://             case TASK_RESEND_RREQ:
//ScenSim-Port://                strcat(buffer, "RREQ    ");
//ScenSim-Port://                break;
//ScenSim-Port://
//ScenSim-Port://             case TASK_CLEANUP:
//ScenSim-Port://                strcat(buffer, "Cleanup ");
//ScenSim-Port://                break;
//ScenSim-Port://
//ScenSim-Port://             case TASK_HELLO:
//ScenSim-Port://                strcat(buffer, "Hello   ");
//ScenSim-Port://                break;
//ScenSim-Port://
//ScenSim-Port://             case TASK_NEIGHBOR:
//ScenSim-Port://                strcat(buffer, "Neighbor");
//ScenSim-Port://                break;
//ScenSim-Port://
//ScenSim-Port://       }
//ScenSim-Port://      tmp_time=tmp_task->time - currtime;
//ScenSim-Port://
//ScenSim-Port://           numerator = (tmp_time);
//ScenSim-Port://           remainder = do_div( numerator, 1000 );  
//ScenSim-Port://           sprintf(tmp_buffer,"    %lu/%lu       %u\n", (unsigned long)numerator, (unsigned long)remainder, tmp_task->retries);
//ScenSim-Port://           strcat(buffer, tmp_buffer);
//ScenSim-Port://       tmp_task = tmp_task->next;
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    strcat(buffer,"-------------------------------------------------------\n");
//ScenSim-Port://
//ScenSim-Port://    /* unlock Read */
//ScenSim-Port://    timer_read_unlock();
//ScenSim-Port://
//ScenSim-Port://    len = strlen(buffer);
//ScenSim-Port://    if (len <= offset+buffer_length) *eof = 1;
//ScenSim-Port://    *buffer_location = buffer + offset;
//ScenSim-Port://    len -= offset;
//ScenSim-Port://    if (len>buffer_length) len = buffer_length;
//ScenSim-Port://    if (len<0) len = 0;
//ScenSim-Port://    return len;
//ScenSim-Port://}

}//namespace//
