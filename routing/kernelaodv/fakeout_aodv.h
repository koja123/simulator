#ifndef FAKEOUT_AODV_H
#define FAKEOUT_AODV_H

#include <limits.h>

namespace KernelAodvPort {


const unsigned int g_broadcast_ip = UINT_MAX; //This means broad cast address (255.255.255.255) in bytes order.

//ScenSim-Port://#define EHOSTUNREACH 1
//ScenSim-Port://#define ENODATA 1

#define IFNAMSIZ        16
#define ETH_ALEN        6
#define ETH_P_IP        0x0800
#define NF_DROP         0
#define NF_ACCEPT       1
#define NF_STOLEN       2
#define NF_QUEUE        3
#define NF_REPEAT       4
#define NF_STOP         5

#define ENOMEM 12 /* Out of memory */

enum AodvAction {
    AODV_RREQ_SEND,
    AODV_NO_ACTION
};


#define time_after(x,y) ((long long int)(y) - (long long int)(x) < 0)
//#define time_before(x,y) ((long long int)(x) - (long long int)(y) <= 0)
#define time_before(x,y) time_after(y,x)
//Assume x == y in the simulator.//

#ifndef htons
#define htons(x) (x)
#endif
#ifndef ntohl
#define ntohl(x) (x)
#endif
#ifndef ntohs
#define ntohs(x) (x)
#endif
#ifndef htonl
#define htonl(x) (x)
#endif


#ifndef __BIG_ENDIAN_BITFIELD
#define __BIG_ENDIAN_BITFIELD 0
#endif
#ifndef __LITTLE_ENDIAN_BITFIELD
#define __LITTLE_ENDIAN_BITFIELD 1
#endif



struct sk_buff {
    /* These two members must be first. */
    struct sk_buff  * next;                 /* Next buffer in list                          */
    struct sk_buff  * prev;                 /* Previous buffer in list                      */

//ScenSim-Port://    struct sk_buff_head * list;             /* List we are on                               */
//ScenSim-Port://    struct sock     *sk;                    /* Socket we are owned by                       */
//ScenSim-Port://    struct timeval  stamp;                  /* Time we arrived                              */
    struct net_device       *dev;           /* Device we arrived on/are leaving by          */
    struct net_device       *real_dev;      /* For support of point to point protocols 
                                               (e.g. 802.3ad) over bonding, we must save the
                                               physical device that got the packet before
                                               replacing skb->dev with the virtual device.  */

    /* Transport layer header */
    union
    {
//ScenSim-Port://        struct tcphdr   *th;
//ScenSim-Port://        struct udphdr   *uh;
//ScenSim-Port://        struct icmphdr  *icmph;
//ScenSim-Port://        struct igmphdr  *igmph;
//ScenSim-Port://        struct iphdr    *ipiph;
//ScenSim-Port://        struct spxhdr   *spxh;
//ScenSim-Port://        unsigned char   *raw;
    } h;

    /* Network layer header */
    union
    {
//ScenSim-Port://        struct iphdr    *iph;
//ScenSim-Port://        struct ipv6hdr  *ipv6h;
//ScenSim-Port://        struct arphdr   *arph;
//ScenSim-Port://        struct ipxhdr   *ipxh;
//ScenSim-Port://        unsigned char   *raw;
    } nh;

    /* Link layer header */
    union 
    {       
//ScenSim-Port://        struct ethhdr   *ethernet;
//ScenSim-Port://        unsigned char   *raw;
    } mac;

//ScenSim-Port://    struct  dst_entry *dst;

    /* 
     * This is the control buffer. It is free to use for every
     * layer. Please put your private variables there. If you
     * want to keep them across layers you have to do a skb_clone()
     * first. This is owned by whoever has the skb queued ATM.
     */ 
    char            cb[48];  

    unsigned int    len;                    /* Length of actual data                        */
    unsigned int    data_len;
    unsigned int    csum;                   /* Checksum                                     */
//ScenSim-Port://unsigned char   __unused,               /* Dead field, may be reused                    */
    unsigned char   cloned,                 /* head may be cloned (check refcnt to be sure).*/
                    pkt_type,               /* Packet class                                 */
                    ip_summed;              /* Driver fed us an IP checksum                 */
    unsigned int    priority;               /* Packet queueing priority                     */
//ScenSim-Port://atomic_t        users;                  /* User count - see datagram.c,tcp.c            */
    unsigned short  protocol;               /* Packet protocol from driver.                 */
    unsigned short  security;               /* Security level of packet                     */
    unsigned int    truesize;               /* Buffer size                                  */

    unsigned char   *head;                  /* Head of buffer                               */
    unsigned char   *data;                  /* Data head pointer                            */
    unsigned char   *tail;                  /* Tail pointer                                 */
    unsigned char   *end;                   /* End pointer                                  */

    void            (*destructor)(struct sk_buff *);        /* Destruct function            */
#ifdef CONFIG_NETFILTER
    /* Can be used for communication between hooks. */
    unsigned long   nfmark;
    /* Cache info */
    unsigned int           nfcache;
    /* Associated connection, if any */
//ScenSim-Port://    struct nf_ct_info *nfct;
#ifdef CONFIG_NETFILTER_DEBUG
    unsigned int nf_debug;
#endif
#endif /*CONFIG_NETFILTER*/

#if defined(CONFIG_HIPPI)
    union{
        unsigned int   ifield;
    } private;
#endif

#ifdef CONFIG_NET_SCHED
   unsigned int           tc_index;               /* traffic control index */
#endif
};


struct net_device
{

    /*
     * This is the first field of the "visible" part of this structure
     * (i.e. as seen by users in the "Space.c" file).  It is the name
     * the interface.
     */
    char                    name[IFNAMSIZ];

    /*
     *      I/O specific fields
     *      FIXME: Merge these and struct ifmap into one
     */
    unsigned long           rmem_end;       /* shmem "recv" end     */
    unsigned long           rmem_start;     /* shmem "recv" start   */
    unsigned long           mem_end;        /* shared mem end       */
    unsigned long           mem_start;      /* shared mem start     */
    unsigned long           base_addr;      /* device I/O address   */
    unsigned int            irq;            /* device IRQ number    */

    /*
     *      Some hardware also needs these fields, but they are not
     *      part of the usual set specified in Space.c.
     */

    unsigned char           if_port;        /* Selectable AUI, TP,..*/
    unsigned char           dma;            /* DMA channel          */

    unsigned long           state;

    struct net_device       *next;
    
    /* The device initialization function. Called only once. */
    int                     (*init)(struct net_device *dev);

    /* ------- Fields preinitialized in Space.c finish here ------- */

    struct net_device       *next_sched;

    /* Interface index. Unique device identifier    */
    int                     ifindex;
    int                     iflink;


//ScenSim-Port://    struct net_device_stats* (*get_stats)(struct net_device *dev);
//ScenSim-Port://    struct iw_statistics*   (*get_wireless_stats)(struct net_device *dev);

    /* List of functions to handle Wireless Extensions (instead of ioctl).
     * See <net/iw_handler.h> for details. Jean II */
//ScenSim-Port://    struct iw_handler_def * wireless_handlers;

//ScenSim-Port://    struct ethtool_ops *ethtool_ops;

    /*
     * This marks the end of the "visible" part of the structure. All
     * fields hereafter are internal to the system, and may change at
     * will (read: may be cleaned up at will).
     */

    /* These may be needed for future network-power-down code. */
    unsigned long           trans_start;    /* Time (in jiffies) of last Tx */
    unsigned long           last_rx;        /* Time of last Rx      */

    unsigned short          flags;  /* interface flags (a la BSD)   */
    unsigned short          gflags;
    unsigned short          priv_flags; /* Like 'flags' but invisible to userspace. */
    unsigned short          unused_alignment_fixer; /* Because we need priv_flags,
                                                     * and we want to be 32-bit aligned.
                                                     */

    unsigned                mtu;    /* interface MTU value          */
    unsigned short          type;   /* interface hardware type      */
    unsigned short          hard_header_len;        /* hardware hdr length  */
    void                    *priv;  /* pointer to private data      */

    struct net_device       *master; /* Pointer to master device of a group,
                                      * which this device is member of.
                                      */

    /* Interface address info. */
//ScenSim-Port://    unsigned char           broadcast[MAX_ADDR_LEN];        /* hw bcast add */
//ScenSim-Port://    unsigned char           dev_addr[MAX_ADDR_LEN]; /* hw address   */
    unsigned char           addr_len;       /* hardware address length      */

//ScenSim-Port://    struct dev_mc_list      *mc_list;       /* Multicast mac addresses      */
    int                     mc_count;       /* Number of installed mcasts   */
    int                     promiscuity;
    int                     allmulti;

    int                     watchdog_timeo;
//ScenSim-Port://    struct timer_list       watchdog_timer;

    /* Protocol specific pointers */
    
    void                    *atalk_ptr;     /* AppleTalk link       */
    void                    *ip_ptr;        /* IPv4 specific data   */  
    void                    *dn_ptr;        /* DECnet specific data */
    void                    *ip6_ptr;       /* IPv6 specific data */
    void                    *ec_ptr;        /* Econet specific data */

//ScenSim-Port://    struct list_head        poll_list;      /* Link to poll list    */
    int                     quota;
    int                     weight;

//ScenSim-Port://    struct Qdisc            *qdisc;
//ScenSim-Port://    struct Qdisc            *qdisc_sleeping;
//ScenSim-Port://    struct Qdisc            *qdisc_list;
//ScenSim-Port://    struct Qdisc            *qdisc_ingress;
    unsigned long           tx_queue_len;   /* Max frames per queue allowed */

    /* hard_start_xmit synchronizer */
//ScenSim-Port://    spinlock_t              xmit_lock;
    /* cpu id of processor entered to hard_start_xmit or -1,
       if nobody entered there.
     */
    int                     xmit_lock_owner;
    /* device queue lock */
//ScenSim-Port://    spinlock_t              queue_lock;
    /* Number of references to this device */
//ScenSim-Port://    atomic_t                refcnt;
    /* The flag marking that device is unregistered, but held by an user */
    int                     deadbeaf;

    /* Net device features */
    int                     features;
#define NETIF_F_SG              1       /* Scatter/gather IO. */
#define NETIF_F_IP_CSUM         2       /* Can checksum only TCP/UDP over IPv4. */
#define NETIF_F_NO_CSUM         4       /* Does not require checksum. F.e. loopack. */
#define NETIF_F_HW_CSUM         8       /* Can checksum all the packets. */
#define NETIF_F_DYNALLOC        16      /* Self-dectructable device. */
#define NETIF_F_HIGHDMA         32      /* Can DMA to high memory. */
#define NETIF_F_FRAGLIST        64      /* Scatter/gather IO. */
#define NETIF_F_HW_VLAN_TX      128     /* Transmit VLAN hw acceleration */
#define NETIF_F_HW_VLAN_RX      256     /* Receive VLAN hw acceleration */
#define NETIF_F_HW_VLAN_FILTER  512     /* Receive filtering on VLAN */
#define NETIF_F_VLAN_CHALLENGED 1024    /* Device cannot handle VLAN packets */

//ScenSim-Port://    /* Called after device is detached from network. */
//ScenSim-Port://    void                    (*uninit)(struct net_device *dev);
//ScenSim-Port://    /* Called after last user reference disappears. */
//ScenSim-Port://    void                    (*destructor)(struct net_device *dev);
//ScenSim-Port://
//ScenSim-Port://    /* Pointers to interface service routines.      */
//ScenSim-Port://    int                     (*open)(struct net_device *dev);
//ScenSim-Port://    int                     (*stop)(struct net_device *dev);
//ScenSim-Port://    int                     (*hard_start_xmit) (struct sk_buff *skb,
//ScenSim-Port://                                                struct net_device *dev);
//ScenSim-Port://#define HAVE_NETDEV_POLL
//ScenSim-Port://    int                     (*poll) (struct net_device *dev, int *quota);
//ScenSim-Port://    int                     (*hard_header) (struct sk_buff *skb,
//ScenSim-Port://                                            struct net_device *dev,
//ScenSim-Port://                                            unsigned short type,
//ScenSim-Port://                                            void *daddr,
//ScenSim-Port://                                            void *saddr,
//ScenSim-Port://                                            unsigned len);
//ScenSim-Port://    int                     (*rebuild_header)(struct sk_buff *skb);
//ScenSim-Port://#define HAVE_MULTICAST                   
//ScenSim-Port://    void                    (*set_multicast_list)(struct net_device *dev);
//ScenSim-Port://#define HAVE_SET_MAC_ADDR                
//ScenSim-Port://    int                     (*set_mac_address)(struct net_device *dev,
//ScenSim-Port://                                               void *addr);
//ScenSim-Port://#define HAVE_PRIVATE_IOCTL
//ScenSim-Port://    int                     (*do_ioctl)(struct net_device *dev,
//ScenSim-Port://                                        struct ifreq *ifr, int cmd);
//ScenSim-Port://#define HAVE_SET_CONFIG
//ScenSim-Port://    int                     (*set_config)(struct net_device *dev,
//ScenSim-Port://                                              struct ifmap *map);
//ScenSim-Port://#define HAVE_HEADER_CACHE
//ScenSim-Port://    int                     (*hard_header_cache)(struct neighbour *neigh,
//ScenSim-Port://                                                 struct hh_cache *hh);
//ScenSim-Port://    void                    (*header_cache_update)(struct hh_cache *hh,
//ScenSim-Port://                                                   struct net_device *dev,
//ScenSim-Port://                                                   unsigned char *  haddr);
//ScenSim-Port://#define HAVE_CHANGE_MTU
//ScenSim-Port://    int                     (*change_mtu)(struct net_device *dev, int new_mtu);
//ScenSim-Port://
//ScenSim-Port://#define HAVE_TX_TIMEOUT
//ScenSim-Port://    void                    (*tx_timeout) (struct net_device *dev);
//ScenSim-Port://
//ScenSim-Port://    void                    (*vlan_rx_register)(struct net_device *dev,
//ScenSim-Port://                                                struct vlan_group *grp);
//ScenSim-Port://    void                    (*vlan_rx_add_vid)(struct net_device *dev,
//ScenSim-Port://                                               unsigned short vid);
//ScenSim-Port://    void                    (*vlan_rx_kill_vid)(struct net_device *dev,
//ScenSim-Port://                                                unsigned short vid);
//ScenSim-Port://
//ScenSim-Port://    int                     (*hard_header_parse)(struct sk_buff *skb,
//ScenSim-Port://                                                 unsigned char *haddr);
//ScenSim-Port://    int                     (*neigh_setup)(struct net_device *dev, struct neigh_parms *);
//ScenSim-Port://    int                     (*accept_fastpath)(struct net_device *, struct dst_entry*);

    /* open/release and usage marking */
//ScenSim-Port://    struct module *owner;

    /* bridge stuff */
//ScenSim-Port://    struct net_bridge_port  *br_port;

#ifdef CONFIG_NET_FASTROUTE
#define NETDEV_FASTROUTE_HMASK 0xF
    /* Semi-private data. Keep it at the end of device struct. */
//ScenSim-Port://    rwlock_t                fastpath_lock;
//ScenSim-Port://    struct dst_entry        *fastpath[NETDEV_FASTROUTE_HMASK+1];
#endif
#ifdef CONFIG_NET_DIVERT
    /* this will get initialized at each interface type init routine */
//ScenSim-Port://    struct divert_blk       *divert;
#endif /* CONFIG_NET_DIVERT */
};

}//namespace//

#endif
