/***************************************************************************
                          aodv.h  -  description
                             -------------------
    begin                : Tue Jul 1 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/


#ifndef AODV_H
#define AODV_H

#include <cstdlib>
#include <assert.h>
#include "fakeout_aodv.h"


namespace KernelAodvPort {
//#include <linux/netdevice.h>

#define AODVPORT        654
#define TRUE            1
#define FALSE           0


// See section 10 of the AODV draft
// Times in milliseconds

//ScenSim-Port//#define ACTIVE_ROUTE_TIMEOUT  3000
//ScenSim-Port//#define ALLOWED_HELLO_LOSS    2
//ScenSim-Port//#define BLACKLIST_TIMEOUT     RREQ_RETRIES * NET_TRAVERSAL_TIME
//ScenSim-Port//#define DELETE_PERIOD         ALLOWED_HELLO_LOSS * HELLO_INTERVAL
//ScenSim-Port//#define HELLO_INTERVAL        1000
//ScenSim-Port//#define LOCAL_ADD_TTL         2
//ScenSim-Port//#define MAX_REPAIR_TTL        0.3 * NET_DIAMETER
//ScenSim-Port//#define MY_ROUTE_TIMEOUT      ACTIVE_ROUTE_TIMEOUT
//ScenSim-Port//#define NET_DIAMETER          10
//ScenSim-Port//#define NODE_TRAVERSAL_TIME   40
//ScenSim-Port//#define NET_TRAVERSAL_TIME    2 * NODE_TRAVERSAL_TIME * NET_DIAMETER
//ScenSim-Port//#define NEXT_HOP_WAIT         NODE_TRAVERSAL_TIME + 10
//ScenSim-Port//#define PATH_DISCOVERY_TIME   2 * NET_TRAVERSAL_TIME
//ScenSim-Port//#define RERR_RATELIMIT        10
//ScenSim-Port//#define RING_TRAVERSAL_TIME   2 * NODE_TRAVERSAL_TIME * ( TTL_VALUE + TIMEOUT_BUFFER)
//ScenSim-Port//#define RREQ_RETRIES          2
//ScenSim-Port//#define RREQ_RATELIMIT        10
//ScenSim-Port//#define TIMEOUT_BUFFER        2
//ScenSim-Port//#define TTL_START             2
//ScenSim-Port//#define TTL_INCREMENT         2
//ScenSim-Port//#define TTL_THRESHOLD         7
//ScenSim-Port//#define TTL_VALUE             3


// Message Types

#define RREQ_MESSAGE        1
#define RREP_MESSAGE        2
#define RERR_MESSAGE        3
#define RREP_ACK_MESSAGE    4

// Tasks

#define TASK_RREQ           1
#define TASK_RREP           2
#define TASK_RERR           3
#define TASK_RREP_ACK       4
#define TASK_RESEND_RREQ    101
#define TASK_HELLO          102
#define TASK_NEIGHBOR       103
#define TASK_CLEANUP        104
#define TASK_ROUTE_CLEANUP  105


// Structures

// Route table

struct _flood_id {
    unsigned int src_ip;
    unsigned int dst_ip;
    unsigned int id;
    //ScenSim-Port://unsigned long lifetime;
    long long int lifetime;
    struct _flood_id *next;
};

typedef struct _flood_id flood_id;



struct _aodv_route {
    unsigned int ip;
    unsigned int netmask;
    unsigned int seq;
    unsigned int old_seq;
    unsigned char  metric;
    unsigned int next_hop;
    unsigned int rreq_id;
    //ScenSim-Port://unsigned long lifetime;
    long long int lifetime;

    struct net_device *dev;
    //ScenSim-Port://unsigned char route_valid:1;
    //ScenSim-Port://unsigned char route_seq_valid:1;
    //ScenSim-Port://unsigned char self_route:1;
    unsigned char route_valid;
    unsigned char route_seq_valid;
    unsigned char self_route;

    struct _aodv_route *next;
    struct _aodv_route *prev;
};

typedef struct _aodv_route aodv_route;


struct _aodv_dev {
    struct net_device *dev;
    aodv_route *route_entry;
    int index;
    unsigned int ip;
    unsigned int netmask;
    char name[IFNAMSIZ];
    struct _aodv_dev *next;
    struct socket *sock;
};

typedef struct _aodv_dev aodv_dev;



struct _aodv_neigh {
    unsigned int ip;
    unsigned int seq;
    //ScenSim-Port://unsigned long lifetime;
    long long int lifetime;
    unsigned char hw_addr[ETH_ALEN];
    struct net_device *dev;
    aodv_route *route_entry;
    int link;
    unsigned char valid_link;

    struct _aodv_neigh *next;
};

typedef struct _aodv_neigh aodv_neigh;



struct _task {

    int type;
    unsigned int id;
    //ScenSim-Port://unsigned long time;
    long long int time;
    unsigned int dst_ip;
    unsigned int src_ip;
    struct net_device *dev;
    unsigned char ttl;
    unsigned short retries;

    unsigned char src_hw_addr[ETH_ALEN];

    unsigned int data_len;
    void *data;

    struct _task *next;
    struct _task *prev;

};

typedef struct _task task;



//Route reply message type
typedef struct {
    unsigned char type;

} rrep_ack;

typedef struct {
    unsigned char type;

//ScenSim-Port://#if defined(__BIG_ENDIAN_BITFIELD)
//ScenSim-Port://    unsigned int a:1;
//ScenSim-Port://    unsigned int reserved1:7;
//ScenSim-Port://#elif defined(__LITTLE_ENDIAN_BITFIELD)
//ScenSim-Port://    unsigned int reserved1:7;
//ScenSim-Port://    unsigned int a:1;
//ScenSim-Port://#else
//ScenSim-Port:////#error "Please fix <asm/byteorder.h>"
//ScenSim-Port://#endif

    unsigned char reserved1;
    //This declaration is for windows compliling. There are padding problems concerning with bit field.

    unsigned char reserved2;
    unsigned char metric;
    unsigned int dst_ip;
    unsigned int dst_seq;
    unsigned int src_ip;
    unsigned int lifetime;
} rrep;



//Endian handling based on DSR implemetation by Alex Song s369677@student.uq.edu.au

typedef struct {
    unsigned char type;

#if defined(__BIG_ENDIAN_BITFIELD)
    unsigned char j:1;
    unsigned char r:1;
    unsigned char g:1;
    unsigned char d:1;
    unsigned char u:1;
    unsigned char reserved:3;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
    unsigned char reserved:3;
    unsigned char u:1;
    unsigned char d:1;
    unsigned char g:1;
    unsigned char r:1;
    unsigned char j:1;
#else
//#error "Please fix <asm/byteorder.h>"
#endif
    unsigned char second_reserved;
    unsigned char metric;
    unsigned int rreq_id;
    unsigned int dst_ip;
    unsigned int dst_seq;
    unsigned int src_ip;
    unsigned int src_seq;
} rreq;



typedef struct {
    unsigned char type;

//ScenSim-Port://#if defined(__BIG_ENDIAN_BITFIELD)
//ScenSim-Port://    unsigned int n:1;
//ScenSim-Port://    unsigned int reserved:15;
//ScenSim-Port://#elif defined(__LITTLE_ENDIAN_BITFIELD)
//ScenSim-Port://    unsigned int reserved:15;
//ScenSim-Port://    unsigned int n:1;
//ScenSim-Port://#else
//ScenSim-Port:////#error "Please fix <asm/byteorder.h>"
//ScenSim-Port://#endif
//ScenSim-Port://    unsigned int dst_count:8;
    unsigned char n;
    unsigned char reserved;
    unsigned char dst_count;
    //These declarations are for windows compliling. There are padding problems concerning with bit field.

} rerr;



typedef struct {
    unsigned int ip;
    unsigned int seq;
} aodv_dst;



struct _rerr_route {
    unsigned int ip;
    unsigned int seq;
    struct _rerr_route *next;
};

typedef struct _rerr_route rerr_route;



}//namespace//

#endif
