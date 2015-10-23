/***************************************************************************
                          task_queue.c  -  description
                             -------------------
    begin                : Tue Jul 8 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#include "kernelaodvglue.h"
#include "task_queue.h"

namespace KernelAodvPort {

//ScenSim-Port://spinlock_t task_queue_lock = SPIN_LOCK_UNLOCKED;
//ScenSim-Port://task *task_q;
//ScenSim-Port://task *task_end;

//ScenSim-Port://void queue_lock()
//ScenSim-Port://{
//ScenSim-Port://    spin_lock_bh(&task_queue_lock);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://void queue_unlock()
//ScenSim-Port://{
//ScenSim-Port://    spin_unlock_bh(&task_queue_lock);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://void init_task_queue()
//ScenSim-Port://{
//ScenSim-Port://   task_q=NULL;
//ScenSim-Port://   task_end=NULL;
//ScenSim-Port://}

//ScenSim-Port://void cleanup_task_queue()
//ScenSim-Port://{
//ScenSim-Port://task *dead_task, *tmp_task;
//ScenSim-Port://
//ScenSim-Port://queue_lock();
//ScenSim-Port:// tmp_task = task_q;
//ScenSim-Port:// task_q=NULL;
//ScenSim-Port://
//ScenSim-Port://while(tmp_task)
//ScenSim-Port://{
//ScenSim-Port://  dead_task = tmp_task;
//ScenSim-Port://tmp_task = tmp_task->next;
//ScenSim-Port://kfree(dead_task->data);
//ScenSim-Port://kfree(dead_task);
//ScenSim-Port://}
//ScenSim-Port:// queue_unlock();
//ScenSim-Port://}



task *create_task(const int type)
{
    task *new_task = new task();

//ScenSim-Port://    new_task = (task *) kmalloc(sizeof(task), GFP_ATOMIC);
//ScenSim-Port://
//ScenSim-Port://    if ( new_task == NULL)
//ScenSim-Port://    {
//ScenSim-Port://        printk(KERN_WARNING "AODV: Not enough memory to create Event Queue Entry\n");
//ScenSim-Port://        return NULL;
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    new_task->time = getcurrtime();
    new_task->type = type;
    new_task->src_ip = 0;
    new_task->dst_ip = 0;
    new_task->ttl = 0;
    new_task->retries = 0;
    new_task->data = NULL;
    new_task->data_len = 0;
    new_task->next = NULL;
    new_task->prev = NULL;
    new_task->dev = NULL;

    return new_task;

}


//ScenSim-Port://int queue_aodv_task(task * new_entry)
//ScenSim-Port://{
//ScenSim-Port://
//ScenSim-Port://    /*lock table */
//ScenSim-Port://    queue_lock();
//ScenSim-Port://
//ScenSim-Port://    //Set all the variables
//ScenSim-Port://    new_entry->next = task_q;
//ScenSim-Port://       new_entry->prev = NULL;
//ScenSim-Port://
//ScenSim-Port://    if (task_q != NULL)
//ScenSim-Port://    {
//ScenSim-Port://       task_q->prev = new_entry; 
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    if (task_end == NULL)
//ScenSim-Port://    {
//ScenSim-Port://      task_end = new_entry; 
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://       task_q = new_entry;
//ScenSim-Port://
//ScenSim-Port://    //unlock table 
//ScenSim-Port://    queue_unlock();
//ScenSim-Port://
//ScenSim-Port://    //wake up the AODV thread
//ScenSim-Port://    kick_aodv();
//ScenSim-Port://
//ScenSim-Port://    return 0;
//ScenSim-Port://}

//ScenSim-Port://task *get_task()
//ScenSim-Port://{
//ScenSim-Port://    task *tmp_task = NULL;
//ScenSim-Port://
//ScenSim-Port://    queue_lock();
//ScenSim-Port://   if (task_end)
//ScenSim-Port://    {
//ScenSim-Port://        tmp_task = task_end;
//ScenSim-Port://        if (task_end == task_q)
//ScenSim-Port://        {
//ScenSim-Port://            task_q = NULL;
//ScenSim-Port://            task_end = NULL;
//ScenSim-Port://        } else
//ScenSim-Port://        {
//ScenSim-Port://            task_end = task_end->prev;
//ScenSim-Port://        }
//ScenSim-Port://        queue_unlock();
//ScenSim-Port://        return tmp_task;
//ScenSim-Port://    }
//ScenSim-Port://        if (task_q != NULL)
//ScenSim-Port://        {
//ScenSim-Port://            printk(KERN_ERR "TASK_QUEUE: Error with task queue\n");
//ScenSim-Port://         }
//ScenSim-Port://        queue_unlock();
//ScenSim-Port://        return NULL;
//ScenSim-Port://
//ScenSim-Port://}


int insert_task(
    KernelAodvProtocol* kernelAodvProtocolPtr,
    const unsigned int lastHopIp,
    const unsigned int destinationIp,
    const int type,
    const unsigned char ttl,
    struct sk_buff *packet)
{
    task *new_task = new task();
    //ScenSim-Port://struct iphdr *ip;

    //ScenSim-Port://int start_point = sizeof(struct udphdr) + sizeof(struct iphdr);


    new_task = create_task(type);


    if (!new_task)
    {
        //ScenSim-Port://printk(KERN_WARNING "AODV: Not enough memory to create Task\n");
        printf("Error: No data is detected.\n");

        return -ENOMEM;
    }

    if (type < 100)
    {
        new_task->src_ip = lastHopIp;
        new_task->dst_ip = destinationIp;
        //ScenSim-Port://        ip = packet->nh.iph;
        //ScenSim-Port://        new_task->src_ip = ip->saddr;
        //ScenSim-Port://        new_task->dst_ip = ip->daddr;
        //ScenSim-Port://        new_task->ttl = ip->ttl;
        new_task->ttl = ttl;
        //ScenSim-Port:////        new_task->dev = packet->dev;
        //ScenSim-Port://new_task->data_len = packet->len - start_point;

        //create space for the data and copy it there
        //ScenSim-Port://new_task->data = kmalloc(new_task->data_len, GFP_ATOMIC);
        new_task->data = packet->data;
        if (!new_task->data)
        {
            //ScenSim-Port://kfree(new_task);
            //ScenSim-Port://printk(KERN_WARNING "AODV: Not enough memory to create Event Queue Data Entry\n");
            printf("Error: No data is detected.\n");
            return -ENOMEM;
        }

        //ScenSim-Port://memcpy(new_task->data, packet->data + start_point, new_task->data_len);
    }



    switch (type)
    {
        //RREP
    case TASK_RREP:
    //ScenSim-Port://        memcpy(&(new_task->src_hw_addr), &(packet->mac.ethernet->h_source), sizeof(unsigned char) * ETH_ALEN);
        break;

    default:
        break;
    }

    kernelAodvProtocolPtr->ProcessTaskForAodvRouting(new_task);
    //ScenSim-Port://queue_aodv_task(new_task);

    return 0;
}

//ScenSim-Port://int insert_task_from_timer(task * timer_task)
//ScenSim-Port://{
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    if (!timer_task)
//ScenSim-Port://    {
//ScenSim-Port://        printk(KERN_WARNING "AODV: Passed a Null task Task\n");
//ScenSim-Port://        return -ENOMEM;
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    queue_aodv_task(timer_task);
//ScenSim-Port://
//ScenSim-Port://}


}//namespace//
