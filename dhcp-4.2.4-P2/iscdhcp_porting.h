#ifndef ISCDHCP_PORTING_H
#define ISCDHCP_PORTING_H

#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <iostream>

#if !defined(_WIN32) && !defined(_WIN64)
#include <unistd.h>
#include <fcntl.h>
#include <netdb.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#endif

#include <sysstuff.h>

namespace ScenSim {
    class IscDhcpImplementation;
};//namespace ScenSim//

namespace IscDhcpPort {

#define LOG_ERR   1
#define LOG_INFO  2
#define LOG_DEBUG 3
#define syslog(type, fmt, bufp) do { if (type <= LOG_ERR) { std::cout << bufp << std::endl; } } while(0)

long int random(void);
void receive_packet(unsigned int index, unsigned char *packet, unsigned int len, unsigned int from_port, struct iaddr *from, bool was_unicast);

#undef htonl
#undef htons
#undef ntohl
#undef ntohs

uint32_t htonl(uint32_t hostlong);
uint16_t htons(uint16_t hostshort);
uint32_t ntohl(uint32_t netlong);
uint16_t ntohs(uint16_t netshort);

typedef uint64_t isc_uint64_t;
typedef enum { isc_boolean_false = 0, isc_boolean_true = 1 } isc_boolean_t;
typedef unsigned int isc_result_t;

#define ISC_FALSE isc_boolean_false
#define ISC_TRUE  isc_boolean_true

#define ISC_R_SUCCESS        0
#define ISC_R_ADDRNOTAVAIL   4
#define ISC_R_ADDRINUSE      5
#define ISC_R_NOPERM         6
#define ISC_R_NOCONN         7
#define ISC_R_NETUNREACH     8
#define ISC_R_CONNREFUSED    12
#define ISC_R_NORESOURCES    13
#define ISC_R_EXISTS         18
#define ISC_R_NOSPACE        19
#define ISC_R_CANCELED       20
#define ISC_R_SHUTTINGDOWN   22
#define ISC_R_NOTFOUND       23
#define ISC_R_FAILURE        25
#define ISC_R_NOTIMPLEMENTED 27
#define ISC_R_NOMORE         29
#define ISC_R_UNEXPECTED     34
#define ISC_R_NOTCONNECTED   40
#define ISC_R_INPROGRESS     53

#define TIME_MAX 2147483647
#define ISC_RESULTCLASS_FROMNUM(num) ((num) << 16)
#define ISC_RESULTCLASS_DHCP ISC_RESULTCLASS_FROMNUM(6)
#define ISC_R_NOMEMORY 1
#define ISC_R_IOERROR 26
#define USE_SOCKET_SEND
#define USE_SOCKET_RECEIVE
#define FLEXIBLE_ARRAY_MEMBER
#define ISC_DHCP_NORETURN
#define NO_H_ERRNO
#define DHCPv6
#define INET6_ADDRSTRLEN 46

#include <isc/md5.h>
#include <isc/heap.h>
#include "dhcpd.h"

// common/tables.cpp
extern struct option dhcp_options_template[];
extern struct option nwip_options_template[];
extern struct option fqdn_options_template[];
extern struct option vendor_class_options_template[];
extern struct option vendor_options_template[];
extern struct option isc_options_template [];
extern struct option dhcpv6_options_template[];
extern struct option vsio_options_template[];
extern struct option isc6_options_template[];

// server/mdb.cpp
typedef struct host_id_info {
    struct option *option;
    host_hash_t *values_hash;
    struct host_id_info *next;
} host_id_info_t;

// server/stables.cpp
extern struct option agent_options_template[];
extern struct option server_options_template[];

struct context {
// omapip/dispatch.cpp
    struct timeval cur_tv;
// omapip/errwarn.cpp
    int log_priority;
// omapip/handle.cpp
    omapi_handle_table_t *omapi_handle_table;
    omapi_handle_t omapi_next_handle;
// omapip/support.cpp
    omapi_object_type_t *omapi_type_connection;
    omapi_object_type_t *omapi_type_generic;
    omapi_object_type_t *omapi_object_types;
// common/comapi.cpp
    omapi_object_type_t *dhcp_type_group;
    omapi_object_type_t *dhcp_type_shared_network;
    omapi_object_type_t *dhcp_type_subnet;
    omapi_object_type_t *dhcp_type_control;
    dhcp_control_object_t *dhcp_control_object;
// common/discover.cpp
    struct interface_info *interfaces;
    int local_family;
// common/memory.cpp
    struct group *root_group;
    group_hash_t *group_name_hash;
    int (*group_write_hook) (struct group_object *);
// common/options.cpp
    struct option *vendor_cfg_option;
// common/parse.cpp
    struct enumeration *enumerations;
// common/print.cpp
    int db_time_format;
// common/tables.cpp
    struct universe dhcp_universe;
#define DHCP_OPTIONS_NUM 97
    struct option dhcp_options[DHCP_OPTIONS_NUM];
    struct universe nwip_universe;
#define NWIP_OPTIONS_NUM 12
    struct option nwip_options[NWIP_OPTIONS_NUM];
    struct universe fqdn_universe;
    struct universe fqdn6_universe;
#define FQDN_OPTIONS_NUM 9
    struct option fqdn_options[FQDN_OPTIONS_NUM];
    struct universe vendor_class_universe;
#define VENDOR_CLASS_OPTIONS_NUM 2
    struct option vendor_class_options[VENDOR_CLASS_OPTIONS_NUM];
    struct universe vendor_universe;
#define VENDOR_OPTIONS_NUM 2
    struct option vendor_options[VENDOR_OPTIONS_NUM];
    struct universe isc_universe;
#define ISC_OPTIONS_NUM 3
    struct option isc_options[ISC_OPTIONS_NUM];
    struct universe dhcpv6_universe;
#define DHCPV6_OPTIONS_NUM 39
    struct option dhcpv6_options[DHCPV6_OPTIONS_NUM];
    struct universe vsio_universe;
#define VSIO_OPTIONS_NUM 2
    struct option vsio_options[VSIO_OPTIONS_NUM];
    struct universe isc6_universe;
#define ISC6_OPTIONS_NUM 3
    struct option isc6_options[ISC6_OPTIONS_NUM];
    universe_hash_t *universe_hash;
    struct universe **universes;
    int universe_count;
    int universe_max;
    struct universe *config_universe;
// common/tree.cpp
    struct binding_scope *global_scope;
// server/class.cpp
    struct executable_statement *default_classification_rules;
    int have_billing_classes;
// server/confpars.cpp
    unsigned char global_host_once;
    unsigned char dhcpv6_class_once;
// server/db.cpp
    FILE *db_file;
    int counting;
    int count;
    TIME write_time;
    int lease_file_is_corrupt;
// server/dhcp.cpp
    int outstanding_pings;
    struct leasequeue *ackqueue_head;
    struct leasequeue *ackqueue_tail;
    struct leasequeue *free_ackqueue;
    struct timeval max_fsync;
    int outstanding_acks;
    int max_outstanding_acks;
    int max_ack_delay_secs;
    int max_ack_delay_usecs;
    int min_ack_delay_usecs;
    int site_code_min;
// server/dhcpd.cpp
    int ddns_update_style;
    const char *path_dhcpd_conf;
    const char *path_dhcpd_db;
// server/md6.cpp
    ia_hash_t *ia_na_active;
    ia_hash_t *ia_ta_active;
    ia_hash_t *ia_pd_active;
    struct ipv6_pool **pools;
    int num_pools;
// server/mdb.cpp
    struct subnet *subnets;
    struct shared_network *shared_networks;
    host_hash_t *host_hw_addr_hash;
    host_hash_t *host_uid_hash;
    host_hash_t *host_name_hash;
    lease_id_hash_t *lease_uid_hash;
    lease_ip_hash_t *lease_ip_addr_hash;
    lease_id_hash_t *lease_hw_addr_hash;
    host_id_info_t *host_id_info;
    int numclasseswritten;
// server/omapi.cpp
    omapi_object_type_t *dhcp_type_lease;
    omapi_object_type_t *dhcp_type_pool;
    omapi_object_type_t *dhcp_type_class;
    omapi_object_type_t *dhcp_type_subclass;
    omapi_object_type_t *dhcp_type_host;
// server/stables.cpp
    struct universe agent_universe;
#define AGENT_OPTIONS_NUM 6
    struct option agent_options[AGENT_OPTIONS_NUM];
    struct universe server_universe;
#define SERVER_OPTIONS_NUM 58
    struct option server_options[SERVER_OPTIONS_NUM];
// client/clparse.cpp
    struct client_config top_level_config;
#define NUM_DEFAULT_REQUESTED_OPTS 9
    struct option *default_requested_options[NUM_DEFAULT_REQUESTED_OPTS + 1];
// client/dhclient.cpp
    TIME default_lease_time;
    TIME max_lease_time;
    const char *path_dhclient_conf;
    const char *path_dhclient_db;
    char *path_dhclient_script;
    int interfaces_requested;
    struct sockaddr_in sockaddr_broadcast;
    struct in_addr giaddr;
    struct data_string default_duid;
    int duid_type;
    u_int16_t local_port;
    u_int16_t remote_port;
    int no_daemon;
    struct string_list *client_env;
    int client_env_count;
    int stateless;
    int wanted_ia_na;
    int wanted_ia_ta;
    int wanted_ia_pd;
    FILE *leaseFile;
    int leases_written;
// client/dhc6.cpp
    struct option *clientid_option;
    struct option *elapsed_option;
    struct option *ia_na_option;
    struct option *ia_ta_option;
    struct option *ia_pd_option;
    struct option *iaaddr_option;
    struct option *iaprefix_option;
    struct option *oro_option;
    struct option *irt_option;
// includes/dhcpd.h
    struct collection *collections;
// others
    void (*bootp)(struct packet*);
    isc_result_t (*find_class)(struct class_type**, const char*, const char*, int);
    void (*classify)(struct packet*, struct class_type*);
    int (*check_collection)(struct packet*, struct lease*, struct collection*);
    int (*parse_allow_deny)(struct option_cache**, struct parse*, int);
    void (*dhcp)(struct packet*);
    void (*dhcpv6)(struct packet*);
    ScenSim::IscDhcpImplementation *glue;

    context(bool use_ipv6) :
// omapip/dispatch.cpp
        cur_tv(),
// omapip/errwarn.cpp
        log_priority(0),
// omapip/handle.cpp
        omapi_handle_table(NULL),
        omapi_next_handle(1),
// omapip/support.cpp
        omapi_type_connection(NULL),
        omapi_type_generic(NULL),
        omapi_object_types(NULL),
// common/comapi.cpp
        dhcp_type_group(NULL),
        dhcp_type_shared_network(NULL),
        dhcp_type_subnet(NULL),
        dhcp_type_control(NULL),
        dhcp_control_object(NULL),
// common/discover.cpp
        interfaces(NULL),
        local_family(AF_INET),
// common/memory.cpp
        root_group(NULL),
        group_name_hash(NULL),
        group_write_hook(NULL),
// common/options.cpp
        vendor_cfg_option(NULL),
// common/parse.cpp
        enumerations(NULL),
// common/print.cpp
        db_time_format(DEFAULT_TIME_FORMAT),
// common/tables.cpp
        dhcp_universe(),
        dhcp_options(),
        nwip_universe(),
        nwip_options(),
        fqdn_universe(),
        fqdn6_universe(),
        fqdn_options(),
        vendor_class_universe(),
        vendor_class_options(),
        vendor_universe(),
        vendor_options(),
        isc_universe(),
        isc_options(),
        dhcpv6_universe(),
        dhcpv6_options(),
        vsio_universe(),
        vsio_options(),
        isc6_universe(),
        isc6_options(),
        universe_hash(NULL),
        universes(NULL),
        universe_count(0),
        universe_max(0),
        config_universe(NULL),
// common/tree.cpp
        global_scope(NULL),
// server/class.cpp
        default_classification_rules(NULL),
        have_billing_classes(0),
// server/confpars.cpp
        global_host_once(1),
        dhcpv6_class_once(1),
// server/db.cpp
        db_file(NULL),
        counting(0),
        count(0),
        write_time(0),
        lease_file_is_corrupt(0),
// server/dhcp.cpp
        outstanding_pings(0),
        ackqueue_head(NULL),
        ackqueue_tail(NULL),
        free_ackqueue(NULL),
        max_fsync(),
        outstanding_acks(0),
        max_outstanding_acks(DEFAULT_DELAYED_ACK),
        max_ack_delay_secs(DEFAULT_ACK_DELAY_SECS),
        max_ack_delay_usecs(DEFAULT_ACK_DELAY_USECS),
        min_ack_delay_usecs(DEFAULT_MIN_ACK_DELAY_USECS),
        site_code_min(0),
// server/dhcpd.cpp
        ddns_update_style(0),
        path_dhcpd_conf(NULL),
        path_dhcpd_db(NULL),
// server/md6.cpp
        ia_na_active(NULL),
        ia_ta_active(NULL),
        ia_pd_active(NULL),
        pools(NULL),
        num_pools(0),
// server/mdb.cpp
        subnets(NULL),
        shared_networks(NULL),
        host_hw_addr_hash(NULL),
        host_uid_hash(NULL),
        host_name_hash(NULL),
        lease_uid_hash(NULL),
        lease_ip_addr_hash(NULL),
        lease_hw_addr_hash(NULL),
        host_id_info(NULL),
        numclasseswritten(0),
// server/omapi.cpp
        dhcp_type_lease(NULL),
        dhcp_type_pool(NULL),
        dhcp_type_class(NULL),
        dhcp_type_subclass(NULL),
        dhcp_type_host(NULL),
// server/stables.cpp
        agent_universe(),
        agent_options(),
        server_universe(),
        server_options(),
// client/clparse.cpp
        top_level_config(),
        default_requested_options(),
// client/dhclient.cpp
        default_lease_time(43200),
        max_lease_time(86400),
        path_dhclient_conf(NULL),
        path_dhclient_db(NULL),
        path_dhclient_script(NULL),
        interfaces_requested(0),
        sockaddr_broadcast(),
        giaddr(),
        default_duid(),
        duid_type(0),
        local_port(0),
        remote_port(0),
        no_daemon(0),
        client_env(NULL),
        client_env_count(0),
        stateless(0),
        wanted_ia_na(-1),
        wanted_ia_ta(0),
        wanted_ia_pd(0),
        leaseFile(NULL),
        leases_written(0),
// client/dhc6.cpp
        clientid_option(NULL),
        elapsed_option(NULL),
        ia_na_option(NULL),
        ia_ta_option(NULL),
        ia_pd_option(NULL),
        iaaddr_option(NULL),
        iaprefix_option(NULL),
        oro_option(NULL),
        irt_option(NULL),
// includes/dhcpd.h
        collections(NULL),
// others
        bootp(NULL),
        find_class(NULL),
        classify(NULL),
        check_collection(NULL),
        parse_allow_deny(NULL),
        dhcp(NULL),
        dhcpv6(NULL),
        glue(NULL)
    {
        for (int i = 0; i < DHCP_OPTIONS_NUM; ++i) {
            dhcp_options[i] = dhcp_options_template[i];
            dhcp_options[i].universe = &dhcp_universe;
        }
        for (int i = 0; i < NWIP_OPTIONS_NUM; ++i) {
            nwip_options[i] = nwip_options_template[i];
            nwip_options[i].universe = &nwip_universe;
        }
        for (int i = 0; i < FQDN_OPTIONS_NUM; ++i) {
            fqdn_options[i] = fqdn_options_template[i];
            fqdn_options[i].universe = &fqdn_universe;
        }
        for (int i = 0; i < VENDOR_CLASS_OPTIONS_NUM; ++i) {
            vendor_class_options[i] = vendor_class_options_template[i];
            vendor_class_options[i].universe = &vendor_class_universe;
        }
        for (int i = 0; i < VENDOR_OPTIONS_NUM; ++i) {
            vendor_options[i] = vendor_options_template[i];
            vendor_options[i].universe = &vendor_universe;
        }
        for (int i = 0; i < ISC_OPTIONS_NUM; ++i) {
            isc_options[i] = isc_options_template[i];
            isc_options[i].universe = &isc_universe;
        }
        for (int i = 0; i < DHCPV6_OPTIONS_NUM; ++i) {
            dhcpv6_options[i] = dhcpv6_options_template[i];
            dhcpv6_options[i].universe = &dhcpv6_universe;
        }
        for (int i = 0; i < VSIO_OPTIONS_NUM; ++i) {
            vsio_options[i] = vsio_options_template[i];
            vsio_options[i].universe = &vsio_universe;
        }
        for (int i = 0; i < ISC6_OPTIONS_NUM; ++i) {
            isc6_options[i] = isc6_options_template[i];
            isc6_options[i].universe = &isc6_universe;
        }
        for (int i = 0; i < AGENT_OPTIONS_NUM; ++i) {
            agent_options[i] = agent_options_template[i];
            agent_options[i].universe = &agent_universe;
        }
        for (int i = 0; i < SERVER_OPTIONS_NUM; ++i) {
            server_options[i] = server_options_template[i];
            server_options[i].universe = &server_universe;
        }
        if (use_ipv6) {
            local_family = AF_INET6;
        }
    }

};//context//

extern struct context* curctx;
#define SCENSIMGLOBAL(n) (curctx->n)
// omapip/dispatch.cpp
#define cur_tv SCENSIMGLOBAL(cur_tv)
// omapip/errwarn.cpp
#define log_priority SCENSIMGLOBAL(log_priority)
// omapip/handle.cpp
#define omapi_handle_table SCENSIMGLOBAL(omapi_handle_table)
#define omapi_next_handle SCENSIMGLOBAL(omapi_next_handle)
// omapip/support.cpp
#define omapi_type_connection SCENSIMGLOBAL(omapi_type_connection)
#define omapi_type_generic SCENSIMGLOBAL(omapi_type_generic)
#define omapi_object_types SCENSIMGLOBAL(omapi_object_types)
// common/comapi.cpp
#define dhcp_type_group SCENSIMGLOBAL(dhcp_type_group)
#define dhcp_type_shared_network SCENSIMGLOBAL(dhcp_type_shared_network)
#define dhcp_type_subnet SCENSIMGLOBAL(dhcp_type_subnet)
#define dhcp_type_control SCENSIMGLOBAL(dhcp_type_control)
#define dhcp_control_object SCENSIMGLOBAL(dhcp_control_object)
// common/discover.cpp
#define interfaces SCENSIMGLOBAL(interfaces)
#define local_family SCENSIMGLOBAL(local_family)
// common/memory.cpp
#define root_group SCENSIMGLOBAL(root_group)
#define group_name_hash SCENSIMGLOBAL(group_name_hash)
#define group_write_hook SCENSIMGLOBAL(group_write_hook)
// common/options.cpp
#define vendor_cfg_option SCENSIMGLOBAL(vendor_cfg_option)
// common/parse.cpp
#define enumerations SCENSIMGLOBAL(enumerations)
// common/print.cpp
#define db_time_format SCENSIMGLOBAL(db_time_format)
// common/tables.cpp
#define dhcp_universe SCENSIMGLOBAL(dhcp_universe)
#define dhcp_options SCENSIMGLOBAL(dhcp_options)
#define nwip_universe SCENSIMGLOBAL(nwip_universe)
#define nwip_options SCENSIMGLOBAL(nwip_options)
#define fqdn_universe SCENSIMGLOBAL(fqdn_universe)
#define fqdn6_universe SCENSIMGLOBAL(fqdn6_universe)
#define fqdn_options SCENSIMGLOBAL(fqdn_options)
#define vendor_class_universe SCENSIMGLOBAL(vendor_class_universe)
#define vendor_class_options SCENSIMGLOBAL(vendor_class_options)
#define vendor_universe SCENSIMGLOBAL(vendor_universe)
#define vendor_options SCENSIMGLOBAL(vendor_options)
#define isc_universe SCENSIMGLOBAL(isc_universe)
#define isc_options SCENSIMGLOBAL(isc_options)
#define dhcpv6_universe SCENSIMGLOBAL(dhcpv6_universe)
#define dhcpv6_options SCENSIMGLOBAL(dhcpv6_options)
#define vsio_universe SCENSIMGLOBAL(vsio_universe)
#define vsio_options SCENSIMGLOBAL(vsio_options)
#define isc6_universe SCENSIMGLOBAL(isc6_universe)
#define isc6_options SCENSIMGLOBAL(isc6_options)
#define universe_hash SCENSIMGLOBAL(universe_hash)
#define universes_ SCENSIMGLOBAL(universes)
#define universe_count_ SCENSIMGLOBAL(universe_count)
#define universe_max SCENSIMGLOBAL(universe_max)
#define config_universe SCENSIMGLOBAL(config_universe)
// common/tree.cpp
#define global_scope SCENSIMGLOBAL(global_scope)
// server/class.cpp
#define default_classification_rules SCENSIMGLOBAL(default_classification_rules)
#define have_billing_classes SCENSIMGLOBAL(have_billing_classes)
// server/confpars.cpp
#define global_host_once SCENSIMGLOBAL(global_host_once)
#define dhcpv6_class_once SCENSIMGLOBAL(dhcpv6_class_once)
// server/db.cpp
#define db_file SCENSIMGLOBAL(db_file)
#define write_time SCENSIMGLOBAL(write_time)
#define lease_file_is_corrupt SCENSIMGLOBAL(lease_file_is_corrupt)
// server/dhcp.cpp
#define outstanding_pings SCENSIMGLOBAL(outstanding_pings)
#define ackqueue_head SCENSIMGLOBAL(ackqueue_head)
#define ackqueue_tail SCENSIMGLOBAL(ackqueue_tail)
#define free_ackqueue SCENSIMGLOBAL(free_ackqueue)
#define max_fsync SCENSIMGLOBAL(max_fsync)
#define outstanding_acks SCENSIMGLOBAL(outstanding_acks)
#define max_outstanding_acks SCENSIMGLOBAL(max_outstanding_acks)
#define max_ack_delay_secs SCENSIMGLOBAL(max_ack_delay_secs)
#define max_ack_delay_usecs SCENSIMGLOBAL(max_ack_delay_usecs)
#define min_ack_delay_usecs SCENSIMGLOBAL(min_ack_delay_usecs)
#define site_code_min_ SCENSIMGLOBAL(site_code_min)//ScenSim-Port//
// server/dhcpd.cpp
#define ddns_update_style SCENSIMGLOBAL(ddns_update_style)
#define path_dhcpd_conf SCENSIMGLOBAL(path_dhcpd_conf)
#define path_dhcpd_db SCENSIMGLOBAL(path_dhcpd_db)
// server/md6.cpp
#define ia_na_active SCENSIMGLOBAL(ia_na_active)
#define ia_ta_active SCENSIMGLOBAL(ia_ta_active)
#define ia_pd_active SCENSIMGLOBAL(ia_pd_active)
#define num_pools SCENSIMGLOBAL(num_pools)
// server/mdb.cpp
#define shared_networks SCENSIMGLOBAL(shared_networks)
#define host_hw_addr_hash SCENSIMGLOBAL(host_hw_addr_hash)
#define host_uid_hash SCENSIMGLOBAL(host_uid_hash)
#define host_name_hash SCENSIMGLOBAL(host_name_hash)
#define lease_uid_hash SCENSIMGLOBAL(lease_uid_hash)
#define lease_ip_addr_hash SCENSIMGLOBAL(lease_ip_addr_hash)
#define lease_hw_addr_hash SCENSIMGLOBAL(lease_hw_addr_hash)
#define host_id_info SCENSIMGLOBAL(host_id_info)
#define numclasseswritten SCENSIMGLOBAL(numclasseswritten)
// server/omapi.cpp
#define dhcp_type_lease SCENSIMGLOBAL(dhcp_type_lease)
#define dhcp_type_pool SCENSIMGLOBAL(dhcp_type_pool)
#define dhcp_type_class SCENSIMGLOBAL(dhcp_type_class)
#define dhcp_type_subclass SCENSIMGLOBAL(dhcp_type_subclass)
#define dhcp_type_host SCENSIMGLOBAL(dhcp_type_host)
// server/stables.cpp
#define agent_universe SCENSIMGLOBAL(agent_universe)
#define server_universe SCENSIMGLOBAL(server_universe)
#define server_options SCENSIMGLOBAL(server_options)
// client/clparse.cpp
#define top_level_config SCENSIMGLOBAL(top_level_config)
#define default_requested_options SCENSIMGLOBAL(default_requested_options)
// client/dhclient.cpp
#define default_lease_time SCENSIMGLOBAL(default_lease_time)
#define max_lease_time SCENSIMGLOBAL(max_lease_time)
#define path_dhclient_conf SCENSIMGLOBAL(path_dhclient_conf)
#define path_dhclient_db SCENSIMGLOBAL(path_dhclient_db)
#define path_dhclient_script SCENSIMGLOBAL(path_dhclient_script)
#define interfaces_requested SCENSIMGLOBAL(interfaces_requested)
#define sockaddr_broadcast SCENSIMGLOBAL(sockaddr_broadcast)
#define giaddr_porting SCENSIMGLOBAL(giaddr)
#define default_duid SCENSIMGLOBAL(default_duid)
#define duid_type SCENSIMGLOBAL(duid_type)
#define local_port SCENSIMGLOBAL(local_port)
#define remote_port SCENSIMGLOBAL(remote_port)
#define no_daemon SCENSIMGLOBAL(no_daemon)
#define client_env SCENSIMGLOBAL(client_env)
#define client_env_count SCENSIMGLOBAL(client_env_count)
#define stateless SCENSIMGLOBAL(stateless)
#define wanted_ia_na SCENSIMGLOBAL(wanted_ia_na)
#define wanted_ia_ta SCENSIMGLOBAL(wanted_ia_ta)
#define wanted_ia_pd SCENSIMGLOBAL(wanted_ia_pd)
#define leaseFile SCENSIMGLOBAL(leaseFile)
#define leases_written SCENSIMGLOBAL(leases_written)
// client/dhc6.cpp
#define clientid_option SCENSIMGLOBAL(clientid_option)
#define elapsed_option SCENSIMGLOBAL(elapsed_option)
#define ia_na_option SCENSIMGLOBAL(ia_na_option)
#define ia_ta_option SCENSIMGLOBAL(ia_ta_option)
#define ia_pd_option SCENSIMGLOBAL(ia_pd_option)
#define iaaddr_option SCENSIMGLOBAL(iaaddr_option)
#define iaprefix_option SCENSIMGLOBAL(iaprefix_option)
#define oro_option SCENSIMGLOBAL(oro_option)
#define irt_option SCENSIMGLOBAL(irt_option)
// includes/dhcpd.h
#define collections SCENSIMGLOBAL(collections)
// others

#define CURCTX_SET(arg) \
    do { assert(curctx == NULL); curctx = (arg); } while(0)
#define CURCTX_CLEAR() \
    do { assert(curctx != NULL); curctx = NULL; } while(0)

int start_client();
int start_server();

int32_t GenerateRandomInt(ScenSim::IscDhcpImplementation *iscDhcpImplPtr);

unsigned int NumberOfInterfaces(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr);

const char *GetInterfaceId(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    unsigned int interfaceIndex);

void GetIpAddress(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    unsigned int interfaceIndex,
    struct iaddr *ia);

void SetIpAddress(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    unsigned int interfaceIndex,
    struct iaddr *ia,
    struct iaddr *mask);

void SetGatewayAddress(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    unsigned int interfaceIndex,
    struct iaddr *ia);

void GetHardwareAddress(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    unsigned int interfaceIndex,
    uint8_t *hardwareAddress,
    uint8_t *hardwareAddressLength);

void AddTimer(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    const timeval *event_time,
    void (*func)(void *),
    void *arg);

void CancelTimer(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    void (*func)(void *),
    void *arg);

void SendPacket(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    const unsigned char *dataPtr,
    const size_t dataLength,
    const uint32_t source_address,
    const uint16_t source_port,
    const uint32_t destination_address,
    const uint16_t destination_port);

void SendPacket6(
    ScenSim::IscDhcpImplementation *iscDhcpImplPtr,
    const unsigned char *dataPtr,
    const size_t dataLength,
    const uint8_t *source_address,
    const uint16_t source_port,
    const uint8_t *destination_address,
    const uint16_t destination_port);

}//namespace IscDhcpPort//

#endif //ISCDHCP_PORTING_H//
