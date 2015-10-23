/* dhclient.c

   DHCP Client. */

/*
 * Copyright (c) 2004-2012 by Internet Systems Consortium, Inc. ("ISC")
 * Copyright (c) 1995-2003 by Internet Software Consortium
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS.  IN NO EVENT SHALL ISC BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *   Internet Systems Consortium, Inc.
 *   950 Charter Street
 *   Redwood City, CA 94063
 *   <info@isc.org>
 *   https://www.isc.org/
 *
 * This code is based on the original client state machine that was
 * written by Elliot Poger.  The code has been extensively hacked on
 * by Ted Lemon since then, so any mistakes you find are probably his
 * fault and not Elliot's.
 */
#include <iscdhcp_porting.h>//ScenSim-Port//
namespace IscDhcpPort {//ScenSim-Port//
#define new new_//ScenSim-Port//

//ScenSim-Port//#include "dhcpd.h"
//ScenSim-Port//#include <syslog.h>
//ScenSim-Port//#include <signal.h>
//ScenSim-Port//#include <errno.h>
//ScenSim-Port//#include <sys/time.h>
//ScenSim-Port//#include <sys/wait.h>
//ScenSim-Port//#include <limits.h>
//ScenSim-Port//#include <dns/result.h>

//ScenSim-Port//TIME default_lease_time = 43200; /* 12 hours... */
//ScenSim-Port//TIME max_lease_time = 86400; /* 24 hours... */

//ScenSim-Port//const char *path_dhclient_conf = _PATH_DHCLIENT_CONF;
//ScenSim-Port//const char *path_dhclient_db = NULL;
//ScenSim-Port//const char *path_dhclient_pid = NULL;
//ScenSim-Port//static char path_dhclient_script_array[] = _PATH_DHCLIENT_SCRIPT;
//ScenSim-Port//char *path_dhclient_script = path_dhclient_script_array;

/* False (default) => we write and use a pid file */
//ScenSim-Port//isc_boolean_t no_pid_file = ISC_FALSE;

//ScenSim-Port//int dhcp_max_agent_option_packet_length = 0;

//ScenSim-Port//int interfaces_requested = 0;

//ScenSim-Port//struct iaddr iaddr_broadcast = { 4, { 255, 255, 255, 255 } };
static const struct iaddr iaddr_broadcast = { 4, { 255, 255, 255, 255 } };//ScenSim-Port//
//ScenSim-Port//struct iaddr iaddr_any = { 4, { 0, 0, 0, 0 } };
//ScenSim-Port//struct in_addr inaddr_any;
static const struct in_addr inaddr_any = { INADDR_ANY };//ScenSim-Port//
//ScenSim-Port//struct sockaddr_in sockaddr_broadcast;
//ScenSim-Port//struct in_addr giaddr;
//ScenSim-Port//struct data_string default_duid;
//ScenSim-Port//int duid_type = 0;

/* ASSERT_STATE() does nothing now; it used to be
   assert (state_is == state_shouldbe). */
//ScenSim-Port//#define ASSERT_STATE(state_is, state_shouldbe) {}

//ScenSim-Port//static const char copyright[] =
//ScenSim-Port//"Copyright 2004-2012 Internet Systems Consortium.";
//ScenSim-Port//static const char arr [] = "All rights reserved.";
//ScenSim-Port//static const char message [] = "Internet Systems Consortium DHCP Client";
//ScenSim-Port//static const char url [] = 
//ScenSim-Port//"For info, please visit https://www.isc.org/software/dhcp/";

//ScenSim-Port//u_int16_t local_port = 0;
//ScenSim-Port//u_int16_t remote_port = 0;
//ScenSim-Port//int no_daemon = 0;
//ScenSim-Port//struct string_list *client_env = NULL;
//ScenSim-Port//int client_env_count = 0;
//ScenSim-Port//int onetry = 0;
//ScenSim-Port//int quiet = 1;
//ScenSim-Port//int nowait = 0;
//ScenSim-Port//int stateless = 0;
//ScenSim-Port//int wanted_ia_na = -1;		/* the absolute value is the real one. */
//ScenSim-Port//int wanted_ia_ta = 0;
//ScenSim-Port//int wanted_ia_pd = 0;
//ScenSim-Port//char *mockup_relay = NULL;

//ScenSim-Port//void run_stateless(int exit_mode);
static void run_stateless(int exit_mode);//ScenSim-Port//

//ScenSim-Port//static void usage(void);

static isc_result_t write_duid(struct data_string *duid);
static void add_reject(struct packet *packet);

//ScenSim-Port//static int check_domain_name(const char *ptr, size_t len, int dots);
//ScenSim-Port//static int check_domain_name_list(const char *ptr, size_t len, int dots);
static int check_option_values(struct universe *universe, unsigned int opt,
			       const char *ptr, size_t len);

int
//ScenSim-Port//main(int argc, char **argv) {
start_client() {//ScenSim-Port//
    curctx->bootp = client_bootp;//ScenSim-Port//
    curctx->find_class = client_find_class;//ScenSim-Port//
    curctx->classify = client_classify;//ScenSim-Port//
    curctx->check_collection = client_check_collection;//ScenSim-Port//
    curctx->parse_allow_deny = client_parse_allow_deny;//ScenSim-Port//
    curctx->dhcp = client_dhcp;//ScenSim-Port//
    curctx->dhcpv6 = client_dhcpv6;//ScenSim-Port//
//ScenSim-Port//	int fd;
//ScenSim-Port//	int i;
	struct interface_info *ip;
	struct client_state *client;
//ScenSim-Port//	unsigned seed;
//ScenSim-Port//	char *server = NULL;
//ScenSim-Port//	isc_result_t status;
	int exit_mode = 0;
	int release_mode = 0;
	struct timeval tv;
//ScenSim-Port//	omapi_object_t *listener;
//ScenSim-Port//	isc_result_t result;
//ScenSim-Port//	int persist = 0;
//ScenSim-Port//	int no_dhclient_conf = 0;
//ScenSim-Port//	int no_dhclient_db = 0;
//ScenSim-Port//	int no_dhclient_pid = 0;
//ScenSim-Port//	int no_dhclient_script = 0;
//ScenSim-Port//#ifdef DHCPv6
//ScenSim-Port//	int local_family_set = 0;
//ScenSim-Port//#endif /* DHCPv6 */
//ScenSim-Port//	char *s;

	/* Initialize client globals. */
	memset(&default_duid, 0, sizeof(default_duid));

	/* Make sure that file descriptors 0 (stdin), 1, (stdout), and
	   2 (stderr) are open. To do this, we assume that when we
	   open a file the lowest available file descriptor is used. */
//ScenSim-Port//	fd = open("/dev/null", O_RDWR);
//ScenSim-Port//	if (fd == 0)
//ScenSim-Port//		fd = open("/dev/null", O_RDWR);
//ScenSim-Port//	if (fd == 1)
//ScenSim-Port//		fd = open("/dev/null", O_RDWR);
//ScenSim-Port//	if (fd == 2)
//ScenSim-Port//		log_perror = 0; /* No sense logging to /dev/null. */
//ScenSim-Port//	else if (fd != -1)
//ScenSim-Port//		close(fd);

//ScenSim-Port//	openlog("dhclient", LOG_NDELAY, LOG_DAEMON);

//ScenSim-Port//#if !(defined(DEBUG) || defined(__CYGWIN32__))
//ScenSim-Port//	setlogmask(LOG_UPTO(LOG_INFO));
//ScenSim-Port//#endif

	/* Set up the isc and dns library managers */
//ScenSim-Port//	status = dhcp_context_create();
//ScenSim-Port//	if (status != ISC_R_SUCCESS)
//ScenSim-Port//		log_fatal("Can't initialize context: %s",
//ScenSim-Port//			  isc_result_totext(status));

	/* Set up the OMAPI. */
//ScenSim-Port//	status = omapi_init();
//ScenSim-Port//	if (status != ISC_R_SUCCESS)
//ScenSim-Port//		log_fatal("Can't initialize OMAPI: %s",
//ScenSim-Port//			  isc_result_totext(status));

	/* Set up the OMAPI wrappers for various server database internal
	   objects. */
	dhcp_common_objects_setup();

//ScenSim-Port//	dhcp_interface_discovery_hook = dhclient_interface_discovery_hook;
//ScenSim-Port//	dhcp_interface_shutdown_hook = dhclient_interface_shutdown_hook;
//ScenSim-Port//	dhcp_interface_startup_hook = dhclient_interface_startup_hook;

//ScenSim-Port//	for (i = 1; i < argc; i++) {
//ScenSim-Port//		if (!strcmp(argv[i], "-r")) {
//ScenSim-Port//			release_mode = 1;
//ScenSim-Port//			no_daemon = 1;
//ScenSim-Port//#ifdef DHCPv6
//ScenSim-Port//		} else if (!strcmp(argv[i], "-4")) {
//ScenSim-Port//			if (local_family_set && local_family != AF_INET)
//ScenSim-Port//				log_fatal("Client can only do v4 or v6, not "
//ScenSim-Port//					  "both.");
//ScenSim-Port//			local_family_set = 1;
//ScenSim-Port//			local_family = AF_INET;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-6")) {
//ScenSim-Port//			if (local_family_set && local_family != AF_INET6)
//ScenSim-Port//				log_fatal("Client can only do v4 or v6, not "
//ScenSim-Port//					  "both.");
//ScenSim-Port//			local_family_set = 1;
//ScenSim-Port//			local_family = AF_INET6;
//ScenSim-Port//#endif /* DHCPv6 */
//ScenSim-Port//		} else if (!strcmp(argv[i], "-x")) { /* eXit, no release */
//ScenSim-Port//			release_mode = 0;
//ScenSim-Port//			no_daemon = 0;
//ScenSim-Port//			exit_mode = 1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-p")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage();
//ScenSim-Port//			local_port = validate_port(argv[i]);
//ScenSim-Port//			log_debug("binding to user-specified port %d",
//ScenSim-Port//				  ntohs(local_port));
//ScenSim-Port//		} else if (!strcmp(argv[i], "-d")) {
//ScenSim-Port//			no_daemon = 1;
//ScenSim-Port//			quiet = 0;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-pf")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage();
//ScenSim-Port//			path_dhclient_pid = argv[i];
//ScenSim-Port//			no_dhclient_pid = 1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "--no-pid")) {
//ScenSim-Port//			no_pid_file = ISC_TRUE;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-cf")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage();
//ScenSim-Port//			path_dhclient_conf = argv[i];
//ScenSim-Port//			no_dhclient_conf = 1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-lf")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage();
//ScenSim-Port//			path_dhclient_db = argv[i];
//ScenSim-Port//			no_dhclient_db = 1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-sf")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage();
//ScenSim-Port//			path_dhclient_script = argv[i];
//ScenSim-Port//			no_dhclient_script = 1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-1")) {
//ScenSim-Port//			onetry = 1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-q")) {
//ScenSim-Port//			quiet = 1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-s")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage();
//ScenSim-Port//			server = argv[i];
//ScenSim-Port//		} else if (!strcmp(argv[i], "-g")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage();
//ScenSim-Port//			mockup_relay = argv[i];
//ScenSim-Port//		} else if (!strcmp(argv[i], "-nw")) {
//ScenSim-Port//			nowait = 1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-n")) {
//ScenSim-Port//			/* do not start up any interfaces */
//ScenSim-Port//			interfaces_requested = -1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-w")) {
//ScenSim-Port//			/* do not exit if there are no broadcast interfaces. */
//ScenSim-Port//			persist = 1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-e")) {
//ScenSim-Port//			struct string_list *tmp;
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage();
//ScenSim-Port//			tmp = dmalloc(strlen(argv[i]) + sizeof *tmp, MDL);
//ScenSim-Port//			if (!tmp)
//ScenSim-Port//				log_fatal("No memory for %s", argv[i]);
//ScenSim-Port//			strcpy(tmp->string, argv[i]);
//ScenSim-Port//			tmp->next = client_env;
//ScenSim-Port//			client_env = tmp;
//ScenSim-Port//			client_env_count++;
//ScenSim-Port//#ifdef DHCPv6
//ScenSim-Port//		} else if (!strcmp(argv[i], "-S")) {
//ScenSim-Port//			if (local_family_set && (local_family == AF_INET)) {
//ScenSim-Port//				usage();
//ScenSim-Port//			}
//ScenSim-Port//			local_family_set = 1;
//ScenSim-Port//			local_family = AF_INET6;
//ScenSim-Port//			wanted_ia_na = 0;
//ScenSim-Port//			stateless = 1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-N")) {
//ScenSim-Port//			if (local_family_set && (local_family == AF_INET)) {
//ScenSim-Port//				usage();
//ScenSim-Port//			}
//ScenSim-Port//			local_family_set = 1;
//ScenSim-Port//			local_family = AF_INET6;
//ScenSim-Port//			if (wanted_ia_na < 0) {
//ScenSim-Port//				wanted_ia_na = 0;
//ScenSim-Port//			}
//ScenSim-Port//			wanted_ia_na++;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-T")) {
//ScenSim-Port//			if (local_family_set && (local_family == AF_INET)) {
//ScenSim-Port//				usage();
//ScenSim-Port//			}
//ScenSim-Port//			local_family_set = 1;
//ScenSim-Port//			local_family = AF_INET6;
//ScenSim-Port//			if (wanted_ia_na < 0) {
//ScenSim-Port//				wanted_ia_na = 0;
//ScenSim-Port//			}
//ScenSim-Port//			wanted_ia_ta++;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-P")) {
//ScenSim-Port//			if (local_family_set && (local_family == AF_INET)) {
//ScenSim-Port//				usage();
//ScenSim-Port//			}
//ScenSim-Port//			local_family_set = 1;
//ScenSim-Port//			local_family = AF_INET6;
//ScenSim-Port//			if (wanted_ia_na < 0) {
//ScenSim-Port//				wanted_ia_na = 0;
//ScenSim-Port//			}
//ScenSim-Port//			wanted_ia_pd++;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-D")) {
//ScenSim-Port//			if (local_family_set && (local_family == AF_INET)) {
//ScenSim-Port//				usage();
//ScenSim-Port//			}
//ScenSim-Port//			local_family_set = 1;
//ScenSim-Port//			local_family = AF_INET6;
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage();
//ScenSim-Port//			if (!strcasecmp(argv[i], "LL")) {
//ScenSim-Port//				duid_type = DUID_LL;
//ScenSim-Port//			} else if (!strcasecmp(argv[i], "LLT")) {
//ScenSim-Port//				duid_type = DUID_LLT;
//ScenSim-Port//			} else {
//ScenSim-Port//				usage();
//ScenSim-Port//			}
//ScenSim-Port//#endif /* DHCPv6 */
//ScenSim-Port//		} else if (!strcmp(argv[i], "-v")) {
//ScenSim-Port//			quiet = 0;
//ScenSim-Port//		} else if (!strcmp(argv[i], "--version")) {
//ScenSim-Port//			log_info("isc-dhclient-%s", PACKAGE_VERSION);
//ScenSim-Port//			exit(0);
//ScenSim-Port//		} else if (argv[i][0] == '-') {
//ScenSim-Port//		    usage();
//ScenSim-Port//		} else if (interfaces_requested < 0) {
//ScenSim-Port//		    usage();
//ScenSim-Port//		} else {
//ScenSim-Port//		    struct interface_info *tmp = NULL;
//ScenSim-Port//
//ScenSim-Port//		    status = interface_allocate(&tmp, MDL);
//ScenSim-Port//		    if (status != ISC_R_SUCCESS)
//ScenSim-Port//			log_fatal("Can't record interface %s:%s",
//ScenSim-Port//				  argv[i], isc_result_totext(status));
//ScenSim-Port//		    if (strlen(argv[i]) >= sizeof(tmp->name))
//ScenSim-Port//			    log_fatal("%s: interface name too long (is %ld)",
//ScenSim-Port//				      argv[i], (long)strlen(argv[i]));
//ScenSim-Port//		    strcpy(tmp->name, argv[i]);
//ScenSim-Port//		    if (interfaces) {
//ScenSim-Port//			    interface_reference(&tmp->next,
//ScenSim-Port//						interfaces, MDL);
//ScenSim-Port//			    interface_dereference(&interfaces, MDL);
//ScenSim-Port//		    }
//ScenSim-Port//		    interface_reference(&interfaces, tmp, MDL);
//ScenSim-Port//		    tmp->flags = INTERFACE_REQUESTED;
//ScenSim-Port//		    interfaces_requested++;
//ScenSim-Port//		}
//ScenSim-Port//	}
    unsigned int num_of_interfaces = NumberOfInterfaces(curctx->glue);//ScenSim-Port//
    for (int i = 0; i < num_of_interfaces; ++i) {//ScenSim-Port//
        struct interface_info *new_interface = NULL;//ScenSim-Port//
        isc_result_t status = interface_allocate(&new_interface, MDL);//ScenSim-Port//
        assert(status == ISC_R_SUCCESS);//ScenSim-Port//
        strcpy(new_interface->name, GetInterfaceId(curctx->glue, i));//ScenSim-Port//
        GetHardwareAddress(curctx->glue, i, new_interface->hw_address.hbuf, &new_interface->hw_address.hlen);//ScenSim-Port//
        if (local_family == AF_INET6) {//ScenSim-Port//
            struct iaddr ia;//ScenSim-Port//
            GetIpAddress(curctx->glue, i, &ia);//ScenSim-Port//
            assert(ia.len == 16);//ScenSim-Port//
            assert(new_interface->v6addresses == NULL);//ScenSim-Port//
            new_interface->v6addresses = (in6_addr*)dmalloc(sizeof(struct in6_addr), MDL);//ScenSim-Port//
            if (new_interface->v6addresses == NULL) {//ScenSim-Port//
                log_fatal("Out of memory saving IPv6 address on interface.");//ScenSim-Port//
            }//ScenSim-Port//
            memcpy(new_interface->v6addresses[0].s6_addr, &ia.iabuf, ia.len);//ScenSim-Port//
            new_interface->v6address_count = 1;//ScenSim-Port//
            new_interface->v6address_max = 1;//ScenSim-Port//
        }//ScenSim-Port//
        if (interfaces) {//ScenSim-Port//
            interface_reference(&new_interface->next, interfaces, MDL);//ScenSim-Port//
            interface_dereference(&interfaces, MDL);//ScenSim-Port//
        }//ScenSim-Port//
        interface_reference(&interfaces, new_interface, MDL);//ScenSim-Port//
        new_interface->flags = INTERFACE_REQUESTED;//ScenSim-Port//
        interfaces_requested++;//ScenSim-Port//
    }//ScenSim-Port//

	if (wanted_ia_na < 0) {
		wanted_ia_na = 1;
	}

	/* Support only one (requested) interface for Prefix Delegation. */
	if (wanted_ia_pd && (interfaces_requested != 1)) {
//ScenSim-Port//		usage();
		assert(false); abort();//ScenSim-Port//
	}

//ScenSim-Port//	if (!no_dhclient_conf && (s = getenv("PATH_DHCLIENT_CONF"))) {
//ScenSim-Port//		path_dhclient_conf = s;
//ScenSim-Port//	}
//ScenSim-Port//	if (!no_dhclient_db && (s = getenv("PATH_DHCLIENT_DB"))) {
//ScenSim-Port//		path_dhclient_db = s;
//ScenSim-Port//	}
//ScenSim-Port//	if (!no_dhclient_pid && (s = getenv("PATH_DHCLIENT_PID"))) {
//ScenSim-Port//		path_dhclient_pid = s;
//ScenSim-Port//	}
//ScenSim-Port//	if (!no_dhclient_script && (s = getenv("PATH_DHCLIENT_SCRIPT"))) {
//ScenSim-Port//		path_dhclient_script = s;
//ScenSim-Port//	}

	/* Set up the initial dhcp option universe. */
	initialize_common_option_spaces();

	/* Assign v4 or v6 specific running parameters. */
	if (local_family == AF_INET)
		dhcpv4_client_assignments();
#ifdef DHCPv6
	else if (local_family == AF_INET6)
		dhcpv6_client_assignments();
#endif /* DHCPv6 */
	else
		log_fatal("Impossible condition at %s:%d.", MDL);

	/*
	 * convert relative path names to absolute, for files that need
	 * to be reopened after chdir() has been called
	 */
//ScenSim-Port//	if (path_dhclient_db[0] != '/') {
//ScenSim-Port//		char *path = dmalloc(PATH_MAX, MDL);
//ScenSim-Port//		if (path == NULL)
//ScenSim-Port//			log_fatal("No memory for filename\n");
//ScenSim-Port//		path_dhclient_db = realpath(path_dhclient_db, path);
//ScenSim-Port//		if (path_dhclient_db == NULL)
//ScenSim-Port//			log_fatal("%s: %s", path, strerror(errno));
//ScenSim-Port//	}

//ScenSim-Port//	if (path_dhclient_script[0] != '/') {
//ScenSim-Port//		char *path = dmalloc(PATH_MAX, MDL);
//ScenSim-Port//		if (path == NULL)
//ScenSim-Port//			log_fatal("No memory for filename\n");
//ScenSim-Port//		path_dhclient_script = realpath(path_dhclient_script, path);
//ScenSim-Port//		if (path_dhclient_script == NULL)
//ScenSim-Port//			log_fatal("%s: %s", path, strerror(errno));
//ScenSim-Port//	}

	/*
	 * See if we should  kill off any currently running client
	 * we don't try to kill it off if the user told us not
	 * to write a pid file - we assume they are controlling
	 * the process in some other fashion.
	 */
//ScenSim-Port//	if ((release_mode || exit_mode) && (no_pid_file == ISC_FALSE)) {
//ScenSim-Port//		FILE *pidfd;
//ScenSim-Port//		pid_t oldpid;
//ScenSim-Port//		long temp;
//ScenSim-Port//		int e;
//ScenSim-Port//
//ScenSim-Port//		oldpid = 0;
//ScenSim-Port//		if ((pidfd = fopen(path_dhclient_pid, "r")) != NULL) {
//ScenSim-Port//			e = fscanf(pidfd, "%ld\n", &temp);
//ScenSim-Port//			oldpid = (pid_t)temp;
//ScenSim-Port//
//ScenSim-Port//			if (e != 0 && e != EOF) {
//ScenSim-Port//				if (oldpid)
//ScenSim-Port//					kill(oldpid, SIGTERM);
//ScenSim-Port//			}
//ScenSim-Port//			fclose(pidfd);
//ScenSim-Port//		}
//ScenSim-Port//	}

//ScenSim-Port//	if (!quiet) {
//ScenSim-Port//		log_info("%s %s", message, PACKAGE_VERSION);
//ScenSim-Port//		log_info(copyright);
//ScenSim-Port//		log_info(arr);
//ScenSim-Port//		log_info(url);
//ScenSim-Port//		log_info("%s", "");
//ScenSim-Port//	} else {
//ScenSim-Port//		log_perror = 0;
//ScenSim-Port//		quiet_interface_discovery = 1;
//ScenSim-Port//	}

	/* If we're given a relay agent address to insert, for testing
	   purposes, figure out what it is. */
//ScenSim-Port//	if (mockup_relay) {
//ScenSim-Port//		if (!inet_aton(mockup_relay, &giaddr)) {
//ScenSim-Port//			struct hostent *he;
//ScenSim-Port//			he = gethostbyname(mockup_relay);
//ScenSim-Port//			if (he) {
//ScenSim-Port//				memcpy(&giaddr, he->h_addr_list[0],
//ScenSim-Port//				       sizeof giaddr);
//ScenSim-Port//			} else {
//ScenSim-Port//				log_fatal("%s: no such host", mockup_relay);
//ScenSim-Port//			}
//ScenSim-Port//		}
//ScenSim-Port//	}

	/* Get the current time... */
//ScenSim-Port//	gettimeofday(&cur_tv, NULL);

	sockaddr_broadcast.sin_family = AF_INET;
	sockaddr_broadcast.sin_port = remote_port;
//ScenSim-Port//	if (server) {
//ScenSim-Port//		if (!inet_aton(server, &sockaddr_broadcast.sin_addr)) {
//ScenSim-Port//			struct hostent *he;
//ScenSim-Port//			he = gethostbyname(server);
//ScenSim-Port//			if (he) {
//ScenSim-Port//				memcpy(&sockaddr_broadcast.sin_addr,
//ScenSim-Port//				       he->h_addr_list[0],
//ScenSim-Port//				       sizeof sockaddr_broadcast.sin_addr);
//ScenSim-Port//			} else
//ScenSim-Port//				sockaddr_broadcast.sin_addr.s_addr =
//ScenSim-Port//					INADDR_BROADCAST;
//ScenSim-Port//		}
//ScenSim-Port//	} else {
		sockaddr_broadcast.sin_addr.s_addr = INADDR_BROADCAST;
//ScenSim-Port//	}

//ScenSim-Port//	inaddr_any.s_addr = INADDR_ANY;

	/* Stateless special case. */
	if (stateless) {
		if (release_mode || (wanted_ia_na > 0) ||
		    wanted_ia_ta || wanted_ia_pd ||
		    (interfaces_requested != 1)) {
//ScenSim-Port//			usage();
			assert(false); abort();//ScenSim-Port//
		}
		run_stateless(exit_mode);
		return 0;
	}

	/* Discover all the network interfaces. */
//ScenSim-Port//	discover_interfaces(DISCOVER_UNCONFIGURED);

	/* Parse the dhclient.conf file. */
	read_client_conf();

	/* Parse the lease database. */
	read_client_leases();

	/* Rewrite the lease database... */
//ScenSim-Port//	rewrite_client_leases();

	/* XXX */
/* 	config_counter(&snd_counter, &rcv_counter); */

	/*
	 * If no broadcast interfaces were discovered, call the script
	 * and tell it so.
	 */
//ScenSim-Port//	if (!interfaces) {
//ScenSim-Port//		/*
//ScenSim-Port//		 * Call dhclient-script with the NBI flag,
//ScenSim-Port//		 * in case somebody cares.
//ScenSim-Port//		 */
//ScenSim-Port//		script_init(NULL, "NBI", NULL);
//ScenSim-Port//		script_go(NULL);
//ScenSim-Port//
//ScenSim-Port//		/*
//ScenSim-Port//		 * If we haven't been asked to persist, waiting for new
//ScenSim-Port//		 * interfaces, then just exit.
//ScenSim-Port//		 */
//ScenSim-Port//		if (!persist) {
//ScenSim-Port//			/* Nothing more to do. */
//ScenSim-Port//			log_info("No broadcast interfaces found - exiting.");
//ScenSim-Port//			exit(0);
//ScenSim-Port//		}
//ScenSim-Port//	} else if (!release_mode && !exit_mode) {
//ScenSim-Port//		/* Call the script with the list of interfaces. */
//ScenSim-Port//		for (ip = interfaces; ip; ip = ip->next) {
//ScenSim-Port//			/*
//ScenSim-Port//			 * If interfaces were specified, don't configure
//ScenSim-Port//			 * interfaces that weren't specified!
//ScenSim-Port//			 */
//ScenSim-Port//			if ((interfaces_requested > 0) &&
//ScenSim-Port//			    ((ip->flags & (INTERFACE_REQUESTED |
//ScenSim-Port//					   INTERFACE_AUTOMATIC)) !=
//ScenSim-Port//			     INTERFACE_REQUESTED))
//ScenSim-Port//				continue;
//ScenSim-Port//
//ScenSim-Port//			if (local_family == AF_INET6) {
//ScenSim-Port//				script_init(ip->client, "PREINIT6", NULL);
//ScenSim-Port//			} else {
//ScenSim-Port//				script_init(ip->client, "PREINIT", NULL);
//ScenSim-Port//			    	if (ip->client->alias != NULL)
//ScenSim-Port//					script_write_params(ip->client,
//ScenSim-Port//							    "alias_",
//ScenSim-Port//							    ip->client->alias);
//ScenSim-Port//			}
//ScenSim-Port//			script_go(ip->client);
//ScenSim-Port//		}
//ScenSim-Port//	}

	/* At this point, all the interfaces that the script thinks
	   are relevant should be running, so now we once again call
	   discover_interfaces(), and this time ask it to actually set
	   up the interfaces. */
//ScenSim-Port//	discover_interfaces(interfaces_requested != 0
//ScenSim-Port//			    ? DISCOVER_REQUESTED
//ScenSim-Port//			    : DISCOVER_RUNNING);

	/* Make up a seed for the random number generator from current
	   time plus the sum of the last four bytes of each
	   interface's hardware address interpreted as an integer.
	   Not much entropy, but we're booting, so we're not likely to
	   find anything better. */
//ScenSim-Port//	seed = 0;
//ScenSim-Port//	for (ip = interfaces; ip; ip = ip->next) {
//ScenSim-Port//		int junk;
//ScenSim-Port//		memcpy(&junk,
//ScenSim-Port//		       &ip->hw_address.hbuf[ip->hw_address.hlen -
//ScenSim-Port//					    sizeof seed], sizeof seed);
//ScenSim-Port//		seed += junk;
//ScenSim-Port//	}
//ScenSim-Port//	srandom(seed + cur_time + (unsigned)getpid());

	/* Start a configuration state machine for each interface. */
#ifdef DHCPv6
	if (local_family == AF_INET6) {
		/* Establish a default DUID.  This may be moved to the
		 * DHCPv4 area later.
		 */
		if (default_duid.len == 0) {
			if (default_duid.buffer != NULL)
				data_string_forget(&default_duid, MDL);

			form_duid(&default_duid, MDL);
			write_duid(&default_duid);
		}

		for (ip = interfaces ; ip != NULL ; ip = ip->next) {
			for (client = ip->client ; client != NULL ;
			     client = client->next) {
				if (release_mode) {
					start_release6(client);
					continue;
				} else if (exit_mode) {
					unconfigure6(client, "STOP6");
					continue;
				}

				/* If we have a previous binding, Confirm
				 * that we can (or can't) still use it.
				 */
				if ((client->active_lease != NULL) &&
				    !client->active_lease->released)
					start_confirm6(client);
				else
					start_init6(client);
			}
		}
	} else
#endif /* DHCPv6 */
	{
		for (ip = interfaces ; ip ; ip = ip->next) {
			ip->flags |= INTERFACE_RUNNING;
			for (client = ip->client ; client ;
			     client = client->next) {
				if (exit_mode)
					state_stop(client);
				else if (release_mode)
					do_release(client);
				else {
					client->state = S_INIT;

					if (top_level_config.initial_delay>0)
					{
						tv.tv_sec = 0;
						if (top_level_config.
						    initial_delay>1)
							tv.tv_sec = cur_time
							+ random()
							% (top_level_config.
							   initial_delay-1);
						tv.tv_usec = random()
							% 1000000;
						if (tv.tv_usec < cur_tv.tv_usec) {//ScenSim-Port//
							tv.tv_usec = cur_tv.tv_usec + 1;//ScenSim-Port//
						}//ScenSim-Port//
						/*
						 * this gives better
						 * distribution than just
						 *whole seconds
						 */
						add_timeout(&tv, state_reboot,
						            client, 0, 0);
					} else {
						state_reboot(client);
					}
				}
			}
		}
	}

	if (exit_mode)
		return 0;
	if (release_mode) {
#ifndef DHCPv6
		return 0;
#else
		if (local_family == AF_INET6) {
//ScenSim-Port//			if (onetry)
//ScenSim-Port//				return 0;
		} else
			return 0;
#endif /* DHCPv6 */
	}

	/* Start up a listener for the object management API protocol. */
//ScenSim-Port//	if (top_level_config.omapi_port != -1) {
//ScenSim-Port//		listener = NULL;
//ScenSim-Port//		result = omapi_generic_new(&listener, MDL);
//ScenSim-Port//		if (result != ISC_R_SUCCESS)
//ScenSim-Port//			log_fatal("Can't allocate new generic object: %s\n",
//ScenSim-Port//				  isc_result_totext(result));
//ScenSim-Port//		result = omapi_protocol_listen(listener,
//ScenSim-Port//					       (unsigned)
//ScenSim-Port//					       top_level_config.omapi_port,
//ScenSim-Port//					       1);
//ScenSim-Port//		if (result != ISC_R_SUCCESS)
//ScenSim-Port//			log_fatal("Can't start OMAPI protocol: %s",
//ScenSim-Port//				  isc_result_totext (result));
//ScenSim-Port//	}

	/* Set up the bootp packet handler... */
//ScenSim-Port//	bootp_packet_handler = do_packet;
//ScenSim-Port//#ifdef DHCPv6
//ScenSim-Port//	dhcpv6_packet_handler = do_packet6;
//ScenSim-Port//#endif /* DHCPv6 */

//ScenSim-Port//#if defined(DEBUG_MEMORY_LEAKAGE) || defined(DEBUG_MALLOC_POOL) || \
//ScenSim-Port//		defined(DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//	dmalloc_cutoff_generation = dmalloc_generation;
//ScenSim-Port//	dmalloc_longterm = dmalloc_outstanding;
//ScenSim-Port//	dmalloc_outstanding = 0;
//ScenSim-Port//#endif

	/* If we're not supposed to wait before getting the address,
	   don't. */
//ScenSim-Port//	if (nowait)
//ScenSim-Port//		go_daemon();

	/* If we're not going to daemonize, write the pid file
	   now. */
//ScenSim-Port//	if (no_daemon || nowait)
//ScenSim-Port//		write_client_pid_file();

	/* Start dispatching packets and timeouts... */
//ScenSim-Port//	dispatch();

	/*NOTREACHED*/
	return 0;
}

//ScenSim-Port//static void usage()
//ScenSim-Port//{
//ScenSim-Port//	log_info("%s %s", message, PACKAGE_VERSION);
//ScenSim-Port//	log_info(copyright);
//ScenSim-Port//	log_info(arr);
//ScenSim-Port//	log_info(url);
//ScenSim-Port//
//ScenSim-Port//
//ScenSim-Port//	log_fatal("Usage: dhclient "
//ScenSim-Port//#ifdef DHCPv6
//ScenSim-Port//		  "[-4|-6] [-SNTP1dvrx] [-nw] [-p <port>] [-D LL|LLT]\n"
//ScenSim-Port//#else /* DHCPv6 */
//ScenSim-Port//		  "[-1dvrx] [-nw] [-p <port>]\n"
//ScenSim-Port//#endif /* DHCPv6 */
//ScenSim-Port//		  "                [-s server-addr] [-cf config-file] "
//ScenSim-Port//		  "[-lf lease-file]\n"
//ScenSim-Port//		  "                [-pf pid-file] [--no-pid] [-e VAR=val]\n"
//ScenSim-Port//		  "                [-sf script-file] [interface]");
//ScenSim-Port//}

void run_stateless(int exit_mode)
{
#ifdef DHCPv6
	struct client_state *client;
//ScenSim-Port//	omapi_object_t *listener;
	isc_result_t result;

	/* Discover the network interface. */
//ScenSim-Port//	discover_interfaces(DISCOVER_REQUESTED);

	if (!interfaces)
//ScenSim-Port//		usage();
	{ assert(false); abort(); }//ScenSim-Port//

	/* Parse the dhclient.conf file. */
	read_client_conf();

	/* Parse the lease database. */
	read_client_leases();

	/* Establish a default DUID. */
	if (default_duid.len == 0) {
		if (default_duid.buffer != NULL)
			data_string_forget(&default_duid, MDL);

		form_duid(&default_duid, MDL);
	}

	/* Start a configuration state machine. */
	for (client = interfaces->client ;
	     client != NULL ;
	     client = client->next) {
		if (exit_mode) {
			unconfigure6(client, "STOP6");
			continue;
		}
		start_info_request6(client);
	}
	if (exit_mode)
		return;

	/* Start up a listener for the object management API protocol. */
//ScenSim-Port//	if (top_level_config.omapi_port != -1) {
//ScenSim-Port//		listener = NULL;
//ScenSim-Port//		result = omapi_generic_new(&listener, MDL);
//ScenSim-Port//		if (result != ISC_R_SUCCESS)
//ScenSim-Port//			log_fatal("Can't allocate new generic object: %s\n",
//ScenSim-Port//				  isc_result_totext(result));
//ScenSim-Port//		result = omapi_protocol_listen(listener,
//ScenSim-Port//					       (unsigned)
//ScenSim-Port//					       top_level_config.omapi_port,
//ScenSim-Port//					       1);
//ScenSim-Port//		if (result != ISC_R_SUCCESS)
//ScenSim-Port//			log_fatal("Can't start OMAPI protocol: %s",
//ScenSim-Port//				  isc_result_totext(result));
//ScenSim-Port//	}

	/* Set up the packet handler... */
//ScenSim-Port//	dhcpv6_packet_handler = do_packet6;

//ScenSim-Port//#if defined(DEBUG_MEMORY_LEAKAGE) || defined(DEBUG_MALLOC_POOL) || \
//ScenSim-Port//		defined(DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//	dmalloc_cutoff_generation = dmalloc_generation;
//ScenSim-Port//	dmalloc_longterm = dmalloc_outstanding;
//ScenSim-Port//	dmalloc_outstanding = 0;
//ScenSim-Port//#endif

	/* If we're not supposed to wait before getting the address,
	   don't. */
//ScenSim-Port//	if (nowait)
//ScenSim-Port//		go_daemon();

	/* If we're not going to daemonize, write the pid file
	   now. */
//ScenSim-Port//	if (no_daemon || nowait)
//ScenSim-Port//		write_client_pid_file();

	/* Start dispatching packets and timeouts... */
//ScenSim-Port//	dispatch();

	/*NOTREACHED*/
#endif /* DHCPv6 */
	return;
}

//ScenSim-Port//isc_result_t find_class (struct class **c,
isc_result_t client_find_class (struct class_type **c,//ScenSim-Port//
		const char *s, const char *file, int line)
{
	return 0;
}

//ScenSim-Port//int check_collection (packet, lease, collection)
//ScenSim-Port//	struct packet *packet;
//ScenSim-Port//	struct lease *lease;
//ScenSim-Port//	struct collection *collection;
int client_check_collection (struct packet *packet, struct lease *lease, struct collection *collection)//ScenSim-Port//
{
	return 0;
}

//ScenSim-Port//void classify (packet, class)
//ScenSim-Port//	struct packet *packet;
//ScenSim-Port//	struct class *class;
void client_classify (struct packet *packet, struct class_type *class_arg)//ScenSim-Port//
{
}

//ScenSim-Port//int unbill_class (lease, class)
//ScenSim-Port//	struct lease *lease;
//ScenSim-Port//	struct class *class;
//ScenSim-Port//{
//ScenSim-Port//	return 0;
//ScenSim-Port//}

//ScenSim-Port//int find_subnet (struct subnet **sp,
//ScenSim-Port//		 struct iaddr addr, const char *file, int line)
//ScenSim-Port//{
//ScenSim-Port//	return 0;
//ScenSim-Port//}

/* Individual States:
 *
 * Each routine is called from the dhclient_state_machine() in one of
 * these conditions:
 * -> entering INIT state
 * -> recvpacket_flag == 0: timeout in this state
 * -> otherwise: received a packet in this state
 *
 * Return conditions as handled by dhclient_state_machine():
 * Returns 1, sendpacket_flag = 1: send packet, reset timer.
 * Returns 1, sendpacket_flag = 0: just reset the timer (wait for a milestone).
 * Returns 0: finish the nap which was interrupted for no good reason.
 *
 * Several per-interface variables are used to keep track of the process:
 *   active_lease: the lease that is being used on the interface
 *                 (null pointer if not configured yet).
 *   offered_leases: leases corresponding to DHCPOFFER messages that have
 *		     been sent to us by DHCP servers.
 *   acked_leases: leases corresponding to DHCPACK messages that have been
 *		   sent to us by DHCP servers.
 *   sendpacket: DHCP packet we're trying to send.
 *   destination: IP address to send sendpacket to
 * In addition, there are several relevant per-lease variables.
 *   T1_expiry, T2_expiry, lease_expiry: lease milestones
 * In the active lease, these control the process of renewing the lease;
 * In leases on the acked_leases list, this simply determines when we
 * can no longer legitimately use the lease.
 */

//ScenSim-Port//void state_reboot (cpp)
//ScenSim-Port//	void *cpp;
void state_reboot (void *cpp)//ScenSim-Port//
{
//ScenSim-Port//	struct client_state *client = cpp;
	struct client_state *client = (client_state*)cpp;//ScenSim-Port//

	/* If we don't remember an active lease, go straight to INIT. */
	if (!client -> active ||
	    client -> active -> is_bootp ||
	    client -> active -> expiry <= cur_time) {
		state_init (client);
		return;
	}

	/* We are in the rebooting state. */
	client -> state = S_REBOOTING;

	/*
	 * make_request doesn't initialize xid because it normally comes
	 * from the DHCPDISCOVER, but we haven't sent a DHCPDISCOVER,
	 * so pick an xid now.
	 */
	client -> xid = random ();

	/*
	 * Make a DHCPREQUEST packet, and set
	 * appropriate per-interface flags.
	 */
	make_request (client, client -> active);
	client -> destination = iaddr_broadcast;
	client -> first_sending = cur_time;
	client -> interval = client -> config -> initial_interval;

	/* Zap the medium list... */
	client -> medium = NULL;

	/* Send out the first DHCPREQUEST packet. */
	send_request (client);
}

/* Called when a lease has completely expired and we've been unable to
   renew it. */

//ScenSim-Port//void state_init (cpp)
//ScenSim-Port//	void *cpp;
void state_init (void *cpp)//ScenSim-Port//
{
//ScenSim-Port//	struct client_state *client = cpp;
	struct client_state *client = (client_state*)cpp;//ScenSim-Port//

//ScenSim-Port//	ASSERT_STATE(state, S_INIT);

	/* Make a DHCPDISCOVER packet, and set appropriate per-interface
	   flags. */
	make_discover (client, client -> active);
	client -> xid = client -> packet.xid;
	client -> destination = iaddr_broadcast;
	client -> state = S_SELECTING;
	client -> first_sending = cur_time;
	client -> interval = client -> config -> initial_interval;

	/* Add an immediate timeout to cause the first DHCPDISCOVER packet
	   to go out. */
	send_discover (client);
}

/*
 * state_selecting is called when one or more DHCPOFFER packets have been
 * received and a configurable period of time has passed.
 */

//ScenSim-Port//void state_selecting (cpp)
//ScenSim-Port//	void *cpp;
void state_selecting (void *cpp)//ScenSim-Port//
{
//ScenSim-Port//	struct client_state *client = cpp;
	struct client_state *client = (client_state*)cpp;//ScenSim-Port//
	struct client_lease *lp, *next, *picked;


//ScenSim-Port//	ASSERT_STATE(state, S_SELECTING);

	/*
	 * Cancel state_selecting and send_discover timeouts, since either
	 * one could have got us here.
	 */
	cancel_timeout (state_selecting, client);
	cancel_timeout (send_discover, client);

	/*
	 * We have received one or more DHCPOFFER packets.   Currently,
	 * the only criterion by which we judge leases is whether or
	 * not we get a response when we arp for them.
	 */
	picked = NULL;
	for (lp = client -> offered_leases; lp; lp = next) {
		next = lp -> next;

		/*
		 * Check to see if we got an ARPREPLY for the address
		 * in this particular lease.
		 */
		if (!picked) {
			picked = lp;
			picked -> next = NULL;
		} else {
			destroy_client_lease (lp);
		}
	}
	client -> offered_leases = NULL;

	/*
	 * If we just tossed all the leases we were offered, go back
	 * to square one.
	 */
	if (!picked) {
		client -> state = S_INIT;
		state_init (client);
		return;
	}

	/* If it was a BOOTREPLY, we can just take the address right now. */
	if (picked -> is_bootp) {
		client -> new = picked;

		/* Make up some lease expiry times
		   XXX these should be configurable. */
		client -> new -> expiry = cur_time + 12000;
		client -> new -> renewal += cur_time + 8000;
		client -> new -> rebind += cur_time + 10000;

		client -> state = S_REQUESTING;

		/* Bind to the address we received. */
		bind_lease (client);
		return;
	}

	/* Go to the REQUESTING state. */
	client -> destination = iaddr_broadcast;
	client -> state = S_REQUESTING;
	client -> first_sending = cur_time;
	client -> interval = client -> config -> initial_interval;

	/* Make a DHCPREQUEST packet from the lease we picked. */
	make_request (client, picked);
	client -> xid = client -> packet.xid;

	/* Toss the lease we picked - we'll get it back in a DHCPACK. */
	destroy_client_lease (picked);

	/* Add an immediate timeout to send the first DHCPREQUEST packet. */
	send_request (client);
}

/* state_requesting is called when we receive a DHCPACK message after
   having sent out one or more DHCPREQUEST packets. */

//ScenSim-Port//void dhcpack (packet)
//ScenSim-Port//	struct packet *packet;
void dhcpack (struct packet *packet)//ScenSim-Port//
{
	struct interface_info *ip = packet -> interface;
	struct client_state *client;
	struct client_lease *lease;
	struct option_cache *oc;
	struct data_string ds;

	/* If we're not receptive to an offer right now, or if the offer
	   has an unrecognizable transaction id, then just drop it. */
	for (client = ip -> client; client; client = client -> next) {
		if (client -> xid == packet -> raw -> xid)
			break;
	}
	if (!client ||
	    (packet -> interface -> hw_address.hlen - 1 !=
	     packet -> raw -> hlen) ||
	    (memcmp (&packet -> interface -> hw_address.hbuf [1],
		     packet -> raw -> chaddr, packet -> raw -> hlen))) {
#if defined (DEBUG)
		log_debug ("DHCPACK in wrong transaction.");
#endif
		return;
	}

	if (client -> state != S_REBOOTING &&
	    client -> state != S_REQUESTING &&
	    client -> state != S_RENEWING &&
	    client -> state != S_REBINDING) {
#if defined (DEBUG)
		log_debug ("DHCPACK in wrong state.");
#endif
		return;
	}

	log_info ("DHCPACK from %s", piaddr (packet -> client_addr));

	lease = packet_to_lease (packet, client);
	if (!lease) {
		log_info ("packet_to_lease failed.");
		return;
	}

	client -> new = lease;

	/* Stop resending DHCPREQUEST. */
	cancel_timeout (send_request, client);

	/* Figure out the lease time. */
	oc = lookup_option (&dhcp_universe, client -> new -> options,
			    DHO_DHCP_LEASE_TIME);
	memset (&ds, 0, sizeof ds);
	if (oc &&
	    evaluate_option_cache (&ds, packet, (struct lease *)0, client,
				   packet -> options, client -> new -> options,
				   &global_scope, oc, MDL)) {
		if (ds.len > 3)
			client -> new -> expiry = getULong (ds.data);
		else
			client -> new -> expiry = 0;
		data_string_forget (&ds, MDL);
	} else
			client -> new -> expiry = 0;

	if (client->new->expiry == 0) {
		struct timeval tv;

		log_error ("no expiry time on offered lease.");

		/* Quench this (broken) server.  Return to INIT to reselect. */
		add_reject(packet);

		/* 1/2 second delay to restart at INIT. */
		tv.tv_sec = cur_tv.tv_sec;
		tv.tv_usec = cur_tv.tv_usec + 500000;

		if (tv.tv_usec >= 1000000) {
			tv.tv_sec++;
			tv.tv_usec -= 1000000;
		}

		add_timeout(&tv, state_init, client, 0, 0);
		return;
	}

	/*
	 * A number that looks negative here is really just very large,
	 * because the lease expiry offset is unsigned.
	 */
	if (client->new->expiry < 0)
		client->new->expiry = TIME_MAX;

	/* Take the server-provided renewal time if there is one. */
	oc = lookup_option (&dhcp_universe, client -> new -> options,
			    DHO_DHCP_RENEWAL_TIME);
	if (oc &&
	    evaluate_option_cache (&ds, packet, (struct lease *)0, client,
				   packet -> options, client -> new -> options,
				   &global_scope, oc, MDL)) {
		if (ds.len > 3)
			client -> new -> renewal = getULong (ds.data);
		else
			client -> new -> renewal = 0;
		data_string_forget (&ds, MDL);
	} else
			client -> new -> renewal = 0;

	/* If it wasn't specified by the server, calculate it. */
	if (!client -> new -> renewal)
		client -> new -> renewal = client -> new -> expiry / 2 + 1;

	if (client -> new -> renewal <= 0)
		client -> new -> renewal = TIME_MAX;

	/* Now introduce some randomness to the renewal time: */
	if (client->new->renewal <= ((TIME_MAX / 3) - 3))
		client->new->renewal = (((client->new->renewal * 3) + 3) / 4) +
				(((random() % client->new->renewal) + 3) / 4);

	/* Same deal with the rebind time. */
	oc = lookup_option (&dhcp_universe, client -> new -> options,
			    DHO_DHCP_REBINDING_TIME);
	if (oc &&
	    evaluate_option_cache (&ds, packet, (struct lease *)0, client,
				   packet -> options, client -> new -> options,
				   &global_scope, oc, MDL)) {
		if (ds.len > 3)
			client -> new -> rebind = getULong (ds.data);
		else
			client -> new -> rebind = 0;
		data_string_forget (&ds, MDL);
	} else
			client -> new -> rebind = 0;

	if (client -> new -> rebind <= 0) {
		if (client -> new -> expiry <= TIME_MAX / 7)
			client -> new -> rebind =
					client -> new -> expiry * 7 / 8;
		else
			client -> new -> rebind =
					client -> new -> expiry / 8 * 7;
	}

	/* Make sure our randomness didn't run the renewal time past the
	   rebind time. */
	if (client -> new -> renewal > client -> new -> rebind) {
		if (client -> new -> rebind <= TIME_MAX / 3)
			client -> new -> renewal =
					client -> new -> rebind * 3 / 4;
		else
			client -> new -> renewal =
					client -> new -> rebind / 4 * 3;
	}

	client -> new -> expiry += cur_time;
	/* Lease lengths can never be negative. */
	if (client -> new -> expiry < cur_time)
		client -> new -> expiry = TIME_MAX;
	client -> new -> renewal += cur_time;
	if (client -> new -> renewal < cur_time)
		client -> new -> renewal = TIME_MAX;
	client -> new -> rebind += cur_time;
	if (client -> new -> rebind < cur_time)
		client -> new -> rebind = TIME_MAX;

	bind_lease (client);
}

//ScenSim-Port//void bind_lease (client)
//ScenSim-Port//	struct client_state *client;
void bind_lease (struct client_state *client)//ScenSim-Port//
{
	struct timeval tv;

	/* Remember the medium. */
	client -> new -> medium = client -> medium;

	/* Run the client script with the new parameters. */
//ScenSim-Port//	script_init (client, (client -> state == S_REQUESTING
//ScenSim-Port//			  ? "BOUND"
//ScenSim-Port//			  : (client -> state == S_RENEWING
//ScenSim-Port//			     ? "RENEW"
//ScenSim-Port//			     : (client -> state == S_REBOOTING
//ScenSim-Port//				? "REBOOT" : "REBIND"))),
//ScenSim-Port//		     client -> new -> medium);
//ScenSim-Port//	if (client -> active && client -> state != S_REBOOTING)
//ScenSim-Port//		script_write_params (client, "old_", client -> active);
//ScenSim-Port//	script_write_params (client, "new_", client -> new);
//ScenSim-Port//	if (client -> alias)
//ScenSim-Port//		script_write_params (client, "alias_", client -> alias);

	/* If the BOUND/RENEW code detects another machine using the
	   offered address, it exits nonzero.  We need to send a
	   DHCPDECLINE and toss the lease. */
//ScenSim-Port//	if (script_go (client)) {
//ScenSim-Port//		make_decline (client, client -> new);
//ScenSim-Port//		send_decline (client);
//ScenSim-Port//		destroy_client_lease (client -> new);
//ScenSim-Port//		client -> new = (struct client_lease *)0;
//ScenSim-Port//		state_init (client);
//ScenSim-Port//		return;
//ScenSim-Port//	}
    struct iaddr subnet_mask, gateway;//ScenSim-Port//
    struct option_cache *op;//ScenSim-Port//
    struct data_string dp;//ScenSim-Port//
    op = lookup_option(&dhcp_universe, client->new->options, DHO_SUBNET_MASK);//ScenSim-Port//
    if (op) {//ScenSim-Port//
        memset(&dp, 0, sizeof dp);//ScenSim-Port//
        evaluate_option_cache (&dp, NULL, NULL, NULL, client->new->options, NULL, NULL, op, MDL);//ScenSim-Port//
        subnet_mask.len = dp.len;//ScenSim-Port//
        memcpy(subnet_mask.iabuf, dp.data, dp.len);//ScenSim-Port//
        data_string_forget (&dp, MDL);//ScenSim-Port//
    }//ScenSim-Port//
    op = lookup_option(&dhcp_universe, client->new->options, DHO_ROUTERS);//ScenSim-Port//
    if (op) {//ScenSim-Port//
        memset(&dp, 0, sizeof dp);//ScenSim-Port//
        evaluate_option_cache (&dp, NULL, NULL, NULL, client->new->options, NULL, NULL, op, MDL);//ScenSim-Port//
        gateway.len = dp.len;//ScenSim-Port//
        memcpy(gateway.iabuf, dp.data, dp.len);//ScenSim-Port//
        data_string_forget (&dp, MDL);//ScenSim-Port//
    }//ScenSim-Port//
    SetIpAddress(curctx->glue, 0, &client->new->address, &subnet_mask);//ScenSim-Port//
    SetGatewayAddress(curctx->glue, 0, &gateway);//ScenSim-Port//

	/* Write out the new lease if it has been long enough. */
	if (!client->last_write ||
	    (cur_time - client->last_write) >= MIN_LEASE_WRITE)
		write_client_lease(client, client->new, 0, 0);

	/* Replace the old active lease with the new one. */
	if (client -> active)
		destroy_client_lease (client -> active);
	client -> active = client -> new;
	client -> new = (struct client_lease *)0;

	/* Set up a timeout to start the renewal process. */
	tv.tv_sec = client->active->renewal;
	tv.tv_usec = ((client->active->renewal - cur_tv.tv_sec) > 1) ?
			random() % 1000000 : cur_tv.tv_usec;
	add_timeout(&tv, state_bound, client, 0, 0);

	log_info ("bound to %s -- renewal in %ld seconds.",
	      piaddr (client -> active -> address),
	      (long)(client -> active -> renewal - cur_time));
	client -> state = S_BOUND;
//ScenSim-Port//	reinitialize_interfaces ();
//ScenSim-Port//	go_daemon ();
//ScenSim-Port//#if defined (NSUPDATE)
//ScenSim-Port//	if (client->config->do_forward_update)
//ScenSim-Port//		dhclient_schedule_updates(client, &client->active->address, 1);
//ScenSim-Port//#endif
}

/* state_bound is called when we've successfully bound to a particular
   lease, but the renewal time on that lease has expired.   We are
   expected to unicast a DHCPREQUEST to the server that gave us our
   original lease. */

//ScenSim-Port//void state_bound (cpp)
//ScenSim-Port//	void *cpp;
void state_bound (void *cpp)//ScenSim-Port//
{
//ScenSim-Port//	struct client_state *client = cpp;
	struct client_state *client = (client_state*)cpp;//ScenSim-Port//
	struct option_cache *oc;
	struct data_string ds;

//ScenSim-Port//	ASSERT_STATE(state, S_BOUND);

	/* T1 has expired. */
	make_request (client, client -> active);
	client -> xid = client -> packet.xid;

	memset (&ds, 0, sizeof ds);
	oc = lookup_option (&dhcp_universe, client -> active -> options,
			    DHO_DHCP_SERVER_IDENTIFIER);
	if (oc &&
	    evaluate_option_cache (&ds, (struct packet *)0, (struct lease *)0,
				   client, (struct option_state *)0,
				   client -> active -> options,
				   &global_scope, oc, MDL)) {
		if (ds.len > 3) {
			memcpy (client -> destination.iabuf, ds.data, 4);
			client -> destination.len = 4;
		} else
			client -> destination = iaddr_broadcast;

		data_string_forget (&ds, MDL);
	} else
		client -> destination = iaddr_broadcast;

	client -> first_sending = cur_time;
	client -> interval = client -> config -> initial_interval;
	client -> state = S_RENEWING;

	/* Send the first packet immediately. */
	send_request (client);
}

/* state_stop is called when we've been told to shut down.   We unconfigure
   the interfaces, and then stop operating until told otherwise. */

//ScenSim-Port//void state_stop (cpp)
//ScenSim-Port//	void *cpp;
void state_stop (void *cpp)//ScenSim-Port//
{
//ScenSim-Port//	struct client_state *client = cpp;
	struct client_state *client = (client_state*)cpp;//ScenSim-Port//

	/* Cancel all timeouts. */
	cancel_timeout(state_selecting, client);
	cancel_timeout(send_discover, client);
	cancel_timeout(send_request, client);
	cancel_timeout(state_bound, client);

	/* If we have an address, unconfigure it. */
//ScenSim-Port//	if (client->active) {
//ScenSim-Port//		script_init(client, "STOP", client->active->medium);
//ScenSim-Port//		script_write_params(client, "old_", client->active);
//ScenSim-Port//		if (client->alias)
//ScenSim-Port//			script_write_params(client, "alias_", client->alias);
//ScenSim-Port//		script_go(client);
//ScenSim-Port//	}
}

//ScenSim-Port//int commit_leases ()
//ScenSim-Port//{
//ScenSim-Port//	return 0;
//ScenSim-Port//}

//ScenSim-Port//int write_lease (lease)
//ScenSim-Port//	struct lease *lease;
//ScenSim-Port//{
//ScenSim-Port//	return 0;
//ScenSim-Port//}

//ScenSim-Port//int write_host (host)
//ScenSim-Port//	struct host_decl *host;
//ScenSim-Port//{
//ScenSim-Port//	return 0;
//ScenSim-Port//}

//ScenSim-Port//void db_startup (testp)
//ScenSim-Port//	int testp;
//ScenSim-Port//{
//ScenSim-Port//}

//ScenSim-Port//void bootp (packet)
//ScenSim-Port//	struct packet *packet;
void client_bootp (struct packet *packet)//ScenSim-Port//
{
	struct iaddrmatchlist *ap;
	char addrbuf[4*16];
	char maskbuf[4*16];

	if (packet -> raw -> op != BOOTREPLY)
		return;

	/* If there's a reject list, make sure this packet's sender isn't
	   on it. */
	for (ap = packet -> interface -> client -> config -> reject_list;
	     ap; ap = ap -> next) {
		if (addr_match(&packet->client_addr, &ap->match)) {

		        /* piaddr() returns its result in a static
			   buffer sized 4*16 (see common/inet.c). */

		        strcpy(addrbuf, piaddr(ap->match.addr));
		        strcpy(maskbuf, piaddr(ap->match.mask));

			log_info("BOOTREPLY from %s rejected by rule %s "
				 "mask %s.", piaddr(packet->client_addr),
				 addrbuf, maskbuf);
			return;
		}
	}

	dhcpoffer (packet);

}

//ScenSim-Port//void dhcp (packet)
//ScenSim-Port//	struct packet *packet;
void client_dhcp (struct packet *packet)//ScenSim-Port//
{
	struct iaddrmatchlist *ap;
	void (*handler) (struct packet *);
	const char *type;
	char addrbuf[4*16];
	char maskbuf[4*16];

	switch (packet -> packet_type) {
	      case DHCPOFFER:
		handler = dhcpoffer;
		type = "DHCPOFFER";
		break;

	      case DHCPNAK:
		handler = dhcpnak;
		type = "DHCPNACK";
		break;

	      case DHCPACK:
		handler = dhcpack;
		type = "DHCPACK";
		break;

	      default:
		return;
	}

	/* If there's a reject list, make sure this packet's sender isn't
	   on it. */
	for (ap = packet -> interface -> client -> config -> reject_list;
	     ap; ap = ap -> next) {
		if (addr_match(&packet->client_addr, &ap->match)) {

		        /* piaddr() returns its result in a static
			   buffer sized 4*16 (see common/inet.c). */

		        strcpy(addrbuf, piaddr(ap->match.addr));
		        strcpy(maskbuf, piaddr(ap->match.mask));

			log_info("%s from %s rejected by rule %s mask %s.",
				 type, piaddr(packet->client_addr),
				 addrbuf, maskbuf);
			return;
		}
	}
	(*handler) (packet);
}

#ifdef DHCPv6
void
//ScenSim-Port//dhcpv6(struct packet *packet) {
client_dhcpv6(struct packet *packet) {//ScenSim-Port//
	struct iaddrmatchlist *ap;
	struct client_state *client;
	char addrbuf[sizeof("ffff:ffff:ffff:ffff:ffff:ffff:ffff:ffff")];

	/* Silently drop bogus messages. */
	if (packet->dhcpv6_msg_type >= dhcpv6_type_name_max)
		return;

	/* Discard, with log, packets from quenched sources. */
	for (ap = packet->interface->client->config->reject_list ;
	     ap ; ap = ap->next) {
		if (addr_match(&packet->client_addr, &ap->match)) {
			strcpy(addrbuf, piaddr(packet->client_addr));
			log_info("%s from %s rejected by rule %s",
				 dhcpv6_type_names[packet->dhcpv6_msg_type],
				 addrbuf,
				 piaddrmask(&ap->match.addr, &ap->match.mask));
			return;
		}
	}

	/* Screen out nonsensical messages. */
	switch(packet->dhcpv6_msg_type) {
	      case DHCPV6_ADVERTISE:
	      case DHCPV6_RECONFIGURE:
		if (stateless)
		  return;
	      /* Falls through */
	      case DHCPV6_REPLY:
		log_info("RCV: %s message on %s from %s.",
			 dhcpv6_type_names[packet->dhcpv6_msg_type],
			 packet->interface->name, piaddr(packet->client_addr));
		break;

	      default:
		return;
	}

	/* Find a client state that matches the incoming XID. */
	for (client = packet->interface->client ; client ;
	     client = client->next) {
		if (memcmp(&client->dhcpv6_transaction_id,
			   packet->dhcpv6_transaction_id, 3) == 0) {
			client->v6_handler(packet, client);
			return;
		}
	}

	/* XXX: temporary log for debugging */
	log_info("Packet received, but nothing done with it.");
}
#endif /* DHCPv6 */

//ScenSim-Port//void dhcpoffer (packet)
//ScenSim-Port//	struct packet *packet;
void dhcpoffer (struct packet *packet)//ScenSim-Port//
{
	struct interface_info *ip = packet -> interface;
	struct client_state *client;
	struct client_lease *lease, *lp;
	struct option **req;
	int i;
	int stop_selecting;
	const char *name = packet -> packet_type ? "DHCPOFFER" : "BOOTREPLY";
	char obuf [1024];
	struct timeval tv;

#ifdef DEBUG_PACKET
	dump_packet (packet);
#endif

	/* Find a client state that matches the xid... */
	for (client = ip -> client; client; client = client -> next)
		if (client -> xid == packet -> raw -> xid)
			break;

	/* If we're not receptive to an offer right now, or if the offer
	   has an unrecognizable transaction id, then just drop it. */
	if (!client ||
	    client -> state != S_SELECTING ||
	    (packet -> interface -> hw_address.hlen - 1 !=
	     packet -> raw -> hlen) ||
	    (memcmp (&packet -> interface -> hw_address.hbuf [1],
		     packet -> raw -> chaddr, packet -> raw -> hlen))) {
#if defined (DEBUG)
		log_debug ("%s in wrong transaction.", name);
#endif
		return;
	}

	sprintf (obuf, "%s from %s", name, piaddr (packet -> client_addr));


	/* If this lease doesn't supply the minimum required DHCPv4 parameters,
	 * ignore it.
	 */
	req = client->config->required_options;
	if (req != NULL) {
		for (i = 0 ; req[i] != NULL ; i++) {
			if ((req[i]->universe == &dhcp_universe) &&
			    !lookup_option(&dhcp_universe, packet->options,
					   req[i]->code)) {
				struct option *option = NULL;
				unsigned code = req[i]->code;

				option_code_hash_lookup(&option,
							dhcp_universe.code_hash,
							&code, 0, MDL);

				if (option)
					log_info("%s: no %s option.", obuf,
						 option->name);
				else
					log_info("%s: no unknown-%u option.",
						 obuf, code);

				option_dereference(&option, MDL);

				return;
			}
		}
	}

	/* If we've already seen this lease, don't record it again. */
	for (lease = client -> offered_leases; lease; lease = lease -> next) {
		if (lease -> address.len == sizeof packet -> raw -> yiaddr &&
		    !memcmp (lease -> address.iabuf,
			     &packet -> raw -> yiaddr, lease -> address.len)) {
			log_debug ("%s: already seen.", obuf);
			return;
		}
	}

	lease = packet_to_lease (packet, client);
	if (!lease) {
		log_info ("%s: packet_to_lease failed.", obuf);
		return;
	}

	/* If this lease was acquired through a BOOTREPLY, record that
	   fact. */
	if (!packet -> options_valid || !packet -> packet_type)
		lease -> is_bootp = 1;

	/* Record the medium under which this lease was offered. */
	lease -> medium = client -> medium;

	/* Figure out when we're supposed to stop selecting. */
	stop_selecting = (client -> first_sending +
			  client -> config -> select_interval);

	/* If this is the lease we asked for, put it at the head of the
	   list, and don't mess with the arp request timeout. */
	if (lease -> address.len == client -> requested_address.len &&
	    !memcmp (lease -> address.iabuf,
		     client -> requested_address.iabuf,
		     client -> requested_address.len)) {
		lease -> next = client -> offered_leases;
		client -> offered_leases = lease;
	} else {
		/* Put the lease at the end of the list. */
		lease -> next = (struct client_lease *)0;
		if (!client -> offered_leases)
			client -> offered_leases = lease;
		else {
			for (lp = client -> offered_leases; lp -> next;
			     lp = lp -> next)
				;
			lp -> next = lease;
		}
	}

	/* If the selecting interval has expired, go immediately to
	   state_selecting().  Otherwise, time out into
	   state_selecting at the select interval. */
	if (stop_selecting <= cur_tv.tv_sec)
		state_selecting (client);
	else {
		tv.tv_sec = stop_selecting;
		tv.tv_usec = cur_tv.tv_usec;
		add_timeout(&tv, state_selecting, client, 0, 0);
		cancel_timeout(send_discover, client);
	}
	log_info("%s", obuf);
}

/* Allocate a client_lease structure and initialize it from the parameters
   in the specified packet. */

//ScenSim-Port//struct client_lease *packet_to_lease (packet, client)
//ScenSim-Port//	struct packet *packet;
//ScenSim-Port//	struct client_state *client;
struct client_lease *packet_to_lease (struct packet *packet, struct client_state *client)//ScenSim-Port//
{
	struct client_lease *lease;
	unsigned i;
	struct option_cache *oc;
	struct option *option = NULL;
	struct data_string data;

	lease = (struct client_lease *)new_client_lease (MDL);

	if (!lease) {
		log_error ("packet_to_lease: no memory to record lease.\n");
		return (struct client_lease *)0;
	}

	memset (lease, 0, sizeof *lease);

	/* Copy the lease options. */
	option_state_reference (&lease -> options, packet -> options, MDL);

	lease -> address.len = sizeof (packet -> raw -> yiaddr);
	memcpy (lease -> address.iabuf, &packet -> raw -> yiaddr,
		lease -> address.len);

	memset (&data, 0, sizeof data);

	if (client -> config -> vendor_space_name) {
		i = DHO_VENDOR_ENCAPSULATED_OPTIONS;

		/* See if there was a vendor encapsulation option. */
		oc = lookup_option (&dhcp_universe, lease -> options, i);
		if (oc &&
		    client -> config -> vendor_space_name &&
		    evaluate_option_cache (&data, packet,
					   (struct lease *)0, client,
					   packet -> options, lease -> options,
					   &global_scope, oc, MDL)) {
			if (data.len) {
				if (!option_code_hash_lookup(&option,
						dhcp_universe.code_hash,
						&i, 0, MDL))
					log_fatal("Unable to find VENDOR "
						  "option (%s:%d).", MDL);
				parse_encapsulated_suboptions
					(packet -> options, option,
					 data.data, data.len, &dhcp_universe,
					 client -> config -> vendor_space_name
						);

				option_dereference(&option, MDL);
			}
			data_string_forget (&data, MDL);
		}
	} else
		i = 0;

	/* Figure out the overload flag. */
	oc = lookup_option (&dhcp_universe, lease -> options,
			    DHO_DHCP_OPTION_OVERLOAD);
	if (oc &&
	    evaluate_option_cache (&data, packet, (struct lease *)0, client,
				   packet -> options, lease -> options,
				   &global_scope, oc, MDL)) {
		if (data.len > 0)
			i = data.data [0];
		else
			i = 0;
		data_string_forget (&data, MDL);
	} else
		i = 0;

	/* If the server name was filled out, copy it. */
	if (!(i & 2) && packet -> raw -> sname [0]) {
		unsigned len;
		/* Don't count on the NUL terminator. */
		for (len = 0; len < DHCP_SNAME_LEN; len++)
			if (!packet -> raw -> sname [len])
				break;
//ScenSim-Port//		lease -> server_name = dmalloc (len + 1, MDL);
		lease -> server_name = (char*)dmalloc (len + 1, MDL);//ScenSim-Port//
		if (!lease -> server_name) {
			log_error ("dhcpoffer: no memory for server name.\n");
			destroy_client_lease (lease);
			return (struct client_lease *)0;
		} else {
			memcpy (lease -> server_name,
				packet -> raw -> sname, len);
			lease -> server_name [len] = 0;
		}
	}

	/* Ditto for the filename. */
	if (!(i & 1) && packet -> raw -> file [0]) {
		unsigned len;
		/* Don't count on the NUL terminator. */
		for (len = 0; len < DHCP_FILE_LEN; len++)
			if (!packet -> raw -> file [len])
				break;
//ScenSim-Port//		lease -> filename = dmalloc (len + 1, MDL);
		lease -> filename = (char*)dmalloc (len + 1, MDL);//ScenSim-Port//
		if (!lease -> filename) {
			log_error ("dhcpoffer: no memory for filename.\n");
			destroy_client_lease (lease);
			return (struct client_lease *)0;
		} else {
			memcpy (lease -> filename,
				packet -> raw -> file, len);
			lease -> filename [len] = 0;
		}
	}

	execute_statements_in_scope ((struct binding_value **)0,
				     (struct packet *)packet,
				     (struct lease *)0, client,
				     lease -> options, lease -> options,
				     &global_scope,
				     client -> config -> on_receipt,
				     (struct group *)0);

	return lease;
}

//ScenSim-Port//void dhcpnak (packet)
//ScenSim-Port//	struct packet *packet;
void dhcpnak (struct packet *packet)//ScenSim-Port//
{
	struct interface_info *ip = packet -> interface;
	struct client_state *client;

	/* Find a client state that matches the xid... */
	for (client = ip -> client; client; client = client -> next)
		if (client -> xid == packet -> raw -> xid)
			break;

	/* If we're not receptive to an offer right now, or if the offer
	   has an unrecognizable transaction id, then just drop it. */
	if (!client ||
	    (packet -> interface -> hw_address.hlen - 1 !=
	     packet -> raw -> hlen) ||
	    (memcmp (&packet -> interface -> hw_address.hbuf [1],
		     packet -> raw -> chaddr, packet -> raw -> hlen))) {
#if defined (DEBUG)
		log_debug ("DHCPNAK in wrong transaction.");
#endif
		return;
	}

	if (client -> state != S_REBOOTING &&
	    client -> state != S_REQUESTING &&
	    client -> state != S_RENEWING &&
	    client -> state != S_REBINDING) {
#if defined (DEBUG)
		log_debug ("DHCPNAK in wrong state.");
#endif
		return;
	}

	log_info ("DHCPNAK from %s", piaddr (packet -> client_addr));

	if (!client -> active) {
#if defined (DEBUG)
		log_info ("DHCPNAK with no active lease.\n");
#endif
		return;
	}

	/* If we get a DHCPNAK, we use the EXPIRE dhclient-script state
	 * to indicate that we want all old bindings to be removed.  (It
	 * is possible that we may get a NAK while in the RENEW state,
	 * so we might have bindings active at that time)
	 */
//ScenSim-Port//	script_init(client, "EXPIRE", NULL);
//ScenSim-Port//	script_write_params(client, "old_", client->active);
//ScenSim-Port//	if (client->alias)
//ScenSim-Port//		script_write_params(client, "alias_", client->alias);
//ScenSim-Port//	script_go(client);

	destroy_client_lease (client -> active);
	client -> active = (struct client_lease *)0;

	/* Stop sending DHCPREQUEST packets... */
	cancel_timeout (send_request, client);

	/* On some scripts, 'EXPIRE' causes the interface to be ifconfig'd
	 * down (this expunges any routes and arp cache).  This makes the
	 * interface unusable by state_init(), which we call next.  So, we
	 * need to 'PREINIT' the interface to bring it back up.
	 */
//ScenSim-Port//	script_init(client, "PREINIT", NULL);
//ScenSim-Port//	if (client->alias)
//ScenSim-Port//		script_write_params(client, "alias_", client->alias);
//ScenSim-Port//	script_go(client);

	client -> state = S_INIT;
	state_init (client);
}

/* Send out a DHCPDISCOVER packet, and set a timeout to send out another
   one after the right interval has expired.  If we don't get an offer by
   the time we reach the panic interval, call the panic function. */

//ScenSim-Port//void send_discover (cpp)
//ScenSim-Port//	void *cpp;
void send_discover (void *cpp)//ScenSim-Port//
{
//ScenSim-Port//	struct client_state *client = cpp;
	struct client_state *client = (client_state*)cpp;//ScenSim-Port//

	int result;
	int interval;
	int increase = 1;
	struct timeval tv;

	/* Figure out how long it's been since we started transmitting. */
	interval = cur_time - client -> first_sending;

	/* If we're past the panic timeout, call the script and tell it
	   we haven't found anything for this interface yet. */
	if (interval > client -> config -> timeout) {
		state_panic (client);
		return;
	}

	/* If we're selecting media, try the whole list before doing
	   the exponential backoff, but if we've already received an
	   offer, stop looping, because we obviously have it right. */
//ScenSim-Port//	if (!client -> offered_leases &&
//ScenSim-Port//	    client -> config -> media) {
//ScenSim-Port//		int fail = 0;
//ScenSim-Port//	      again:
//ScenSim-Port//		if (client -> medium) {
//ScenSim-Port//			client -> medium = client -> medium -> next;
//ScenSim-Port//			increase = 0;
//ScenSim-Port//		}
//ScenSim-Port//		if (!client -> medium) {
//ScenSim-Port//			if (fail)
//ScenSim-Port//				log_fatal ("No valid media types for %s!",
//ScenSim-Port//				       client -> interface -> name);
//ScenSim-Port//			client -> medium =
//ScenSim-Port//				client -> config -> media;
//ScenSim-Port//			increase = 1;
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//		log_info ("Trying medium \"%s\" %d",
//ScenSim-Port//			  client -> medium -> string, increase);
//ScenSim-Port//		script_init (client, "MEDIUM", client -> medium);
//ScenSim-Port//		if (script_go (client)) {
//ScenSim-Port//			fail = 1;
//ScenSim-Port//			goto again;
//ScenSim-Port//		}
//ScenSim-Port//	}

	/* If we're supposed to increase the interval, do so.  If it's
	   currently zero (i.e., we haven't sent any packets yet), set
	   it to initial_interval; otherwise, add to it a random number
	   between zero and two times itself.  On average, this means
	   that it will double with every transmission. */
	if (increase) {
		if (!client->interval)
			client->interval = client->config->initial_interval;
		else
			client->interval += random() % (2 * client->interval);

		/* Don't backoff past cutoff. */
		if (client->interval > client->config->backoff_cutoff)
			client->interval = (client->config->backoff_cutoff / 2)
				 + (random() % client->config->backoff_cutoff);
	} else if (!client->interval)
		client->interval = client->config->initial_interval;

	/* If the backoff would take us to the panic timeout, just use that
	   as the interval. */
	if (cur_time + client -> interval >
	    client -> first_sending + client -> config -> timeout)
		client -> interval =
			(client -> first_sending +
			 client -> config -> timeout) - cur_time + 1;

	/* Record the number of seconds since we started sending. */
	if (interval < 65536)
		client -> packet.secs = htons (interval);
	else
		client -> packet.secs = htons (65535);
	client -> secs = client -> packet.secs;

	log_info ("DHCPDISCOVER on %s to %s port %d interval %ld",
	      client -> name ? client -> name : client -> interface -> name,
	      inet_ntoa (sockaddr_broadcast.sin_addr),
	      ntohs (sockaddr_broadcast.sin_port), (long)(client -> interval));

	/* Send out a packet. */
	result = send_packet(client->interface, NULL, &client->packet,
			     client->packet_length, inaddr_any,
                             &sockaddr_broadcast, NULL);
        if (result < 0) {
		log_error("%s:%d: Failed to send %d byte long packet over %s "
			  "interface.", MDL, client->packet_length,
			  client->interface->name);
	}

	/*
	 * If we used 0 microseconds here, and there were other clients on the
	 * same network with a synchronized local clock (ntp), and a similar
	 * zero-microsecond-scheduler behavior, then we could be participating
	 * in a sub-second DOS ttck.
	 */
	tv.tv_sec = cur_tv.tv_sec + client->interval;
	tv.tv_usec = client->interval > 1 ? random() % 1000000 : cur_tv.tv_usec;
	add_timeout(&tv, send_discover, client, 0, 0);
}

/* state_panic gets called if we haven't received any offers in a preset
   amount of time.   When this happens, we try to use existing leases that
   haven't yet expired, and failing that, we call the client script and
   hope it can do something. */

//ScenSim-Port//void state_panic (cpp)
//ScenSim-Port//	void *cpp;
void state_panic (void *cpp)//ScenSim-Port//
{
//ScenSim-Port//	struct client_state *client = cpp;
	struct client_state *client = (client_state*)cpp;//ScenSim-Port//
	struct client_lease *loop;
	struct client_lease *lp;
	struct timeval tv;

	loop = lp = client -> active;

	log_info ("No DHCPOFFERS received.");

	/* We may not have an active lease, but we may have some
	   predefined leases that we can try. */
	if (!client -> active && client -> leases)
		goto activate_next;

	/* Run through the list of leases and see if one can be used. */
	while (client -> active) {
		if (client -> active -> expiry > cur_time) {
			log_info ("Trying recorded lease %s",
			      piaddr (client -> active -> address));
			/* Run the client script with the existing
			   parameters. */
//ScenSim-Port//			script_init (client, "TIMEOUT",
//ScenSim-Port//				     client -> active -> medium);
//ScenSim-Port//			script_write_params (client, "new_", client -> active);
//ScenSim-Port//			if (client -> alias)
//ScenSim-Port//				script_write_params (client, "alias_",
//ScenSim-Port//						     client -> alias);

			/* If the old lease is still good and doesn't
			   yet need renewal, go into BOUND state and
			   timeout at the renewal time. */
//ScenSim-Port//			if (!script_go (client)) {
//ScenSim-Port//			    if (cur_time < client -> active -> renewal) {
//ScenSim-Port//				client -> state = S_BOUND;
//ScenSim-Port//				log_info ("bound: renewal in %ld %s.",
//ScenSim-Port//					  (long)(client -> active -> renewal -
//ScenSim-Port//						 cur_time), "seconds");
//ScenSim-Port//				tv.tv_sec = client->active->renewal;
//ScenSim-Port//				tv.tv_usec = ((client->active->renewal -
//ScenSim-Port//						    cur_time) > 1) ?
//ScenSim-Port//						random() % 1000000 :
//ScenSim-Port//						cur_tv.tv_usec;
//ScenSim-Port//				add_timeout(&tv, state_bound, client, 0, 0);
//ScenSim-Port//			    } else {
//ScenSim-Port//				client -> state = S_BOUND;
//ScenSim-Port//				log_info ("bound: immediate renewal.");
//ScenSim-Port//				state_bound (client);
//ScenSim-Port//			    }
//ScenSim-Port//			    reinitialize_interfaces ();
//ScenSim-Port//			    go_daemon ();
//ScenSim-Port//			    return;
//ScenSim-Port//			}
		}

		/* If there are no other leases, give up. */
		if (!client -> leases) {
			client -> leases = client -> active;
			client -> active = (struct client_lease *)0;
			break;
		}

	activate_next:
		/* Otherwise, put the active lease at the end of the
		   lease list, and try another lease.. */
		for (lp = client -> leases; lp -> next; lp = lp -> next)
			;
		lp -> next = client -> active;
		if (lp -> next) {
			lp -> next -> next = (struct client_lease *)0;
		}
		client -> active = client -> leases;
		client -> leases = client -> leases -> next;

		/* If we already tried this lease, we've exhausted the
		   set of leases, so we might as well give up for
		   now. */
		if (client -> active == loop)
			break;
		else if (!loop)
			loop = client -> active;
	}

	/* No leases were available, or what was available didn't work, so
	   tell the shell script that we failed to allocate an address,
	   and try again later. */
//ScenSim-Port//	if (onetry) {
//ScenSim-Port//		if (!quiet)
//ScenSim-Port//			log_info ("Unable to obtain a lease on first try.%s",
//ScenSim-Port//				  "  Exiting.");
//ScenSim-Port//		exit (2);
//ScenSim-Port//	}

	log_info ("No working leases in persistent database - sleeping.");
//ScenSim-Port//	script_init (client, "FAIL", (struct string_list *)0);
//ScenSim-Port//	if (client -> alias)
//ScenSim-Port//		script_write_params (client, "alias_", client -> alias);
//ScenSim-Port//	script_go (client);
	client -> state = S_INIT;
	tv.tv_sec = cur_tv.tv_sec + ((client->config->retry_interval + 1) / 2 +
		    (random() % client->config->retry_interval));
	tv.tv_usec = ((tv.tv_sec - cur_tv.tv_sec) > 1) ?
			random() % 1000000 : cur_tv.tv_usec;
	add_timeout(&tv, state_init, client, 0, 0);
//ScenSim-Port//	go_daemon ();
}

//ScenSim-Port//void send_request (cpp)
//ScenSim-Port//	void *cpp;
void send_request (void *cpp)//ScenSim-Port//
{
//ScenSim-Port//	struct client_state *client = cpp;
	struct client_state *client = (client_state*)cpp;//ScenSim-Port//

	int result;
	int interval;
	struct sockaddr_in destination;
	struct in_addr from;
	struct timeval tv;

	/* Figure out how long it's been since we started transmitting. */
	interval = cur_time - client -> first_sending;

	/* If we're in the INIT-REBOOT or REQUESTING state and we're
	   past the reboot timeout, go to INIT and see if we can
	   DISCOVER an address... */
	/* XXX In the INIT-REBOOT state, if we don't get an ACK, it
	   means either that we're on a network with no DHCP server,
	   or that our server is down.  In the latter case, assuming
	   that there is a backup DHCP server, DHCPDISCOVER will get
	   us a new address, but we could also have successfully
	   reused our old address.  In the former case, we're hosed
	   anyway.  This is not a win-prone situation. */
	if ((client -> state == S_REBOOTING ||
	     client -> state == S_REQUESTING) &&
	    interval > client -> config -> reboot_timeout) {
	cancel:
		client -> state = S_INIT;
		cancel_timeout (send_request, client);
		state_init (client);
		return;
	}

	/* If we're in the reboot state, make sure the media is set up
	   correctly. */
	if (client -> state == S_REBOOTING &&
	    !client -> medium &&
	    client -> active -> medium ) {
//ScenSim-Port//		script_init (client, "MEDIUM", client -> active -> medium);

		/* If the medium we chose won't fly, go to INIT state. */
//ScenSim-Port//		if (script_go (client))
//ScenSim-Port//			goto cancel;

		/* Record the medium. */
		client -> medium = client -> active -> medium;
	}

	/* If the lease has expired, relinquish the address and go back
	   to the INIT state. */
	if (client -> state != S_REQUESTING &&
	    cur_time > client -> active -> expiry) {
		/* Run the client script with the new parameters. */
//ScenSim-Port//		script_init (client, "EXPIRE", (struct string_list *)0);
//ScenSim-Port//		script_write_params (client, "old_", client -> active);
//ScenSim-Port//		if (client -> alias)
//ScenSim-Port//			script_write_params (client, "alias_",
//ScenSim-Port//					     client -> alias);
//ScenSim-Port//		script_go (client);

		/* Now do a preinit on the interface so that we can
		   discover a new address. */
//ScenSim-Port//		script_init (client, "PREINIT", (struct string_list *)0);
//ScenSim-Port//		if (client -> alias)
//ScenSim-Port//			script_write_params (client, "alias_",
//ScenSim-Port//					     client -> alias);
//ScenSim-Port//		script_go (client);

		client -> state = S_INIT;
		state_init (client);
		return;
	}

	/* Do the exponential backoff... */
	if (!client -> interval)
		client -> interval = client -> config -> initial_interval;
	else {
		client -> interval += ((random () >> 2) %
				       (2 * client -> interval));
	}

	/* Don't backoff past cutoff. */
	if (client -> interval >
	    client -> config -> backoff_cutoff)
		client -> interval =
			((client -> config -> backoff_cutoff / 2)
			 + ((random () >> 2) %
					client -> config -> backoff_cutoff));

	/* If the backoff would take us to the expiry time, just set the
	   timeout to the expiry time. */
	if (client -> state != S_REQUESTING &&
	    cur_time + client -> interval > client -> active -> expiry)
		client -> interval =
			client -> active -> expiry - cur_time + 1;

	/* If the lease T2 time has elapsed, or if we're not yet bound,
	   broadcast the DHCPREQUEST rather than unicasting. */
	if (client -> state == S_REQUESTING ||
	    client -> state == S_REBOOTING ||
	    cur_time > client -> active -> rebind)
		destination.sin_addr = sockaddr_broadcast.sin_addr;
	else
		memcpy (&destination.sin_addr.s_addr,
			client -> destination.iabuf,
			sizeof destination.sin_addr.s_addr);
	destination.sin_port = remote_port;
	destination.sin_family = AF_INET;
#ifdef HAVE_SA_LEN
	destination.sin_len = sizeof destination;
#endif

	if (client -> state == S_RENEWING ||
	    client -> state == S_REBINDING)
		memcpy (&from, client -> active -> address.iabuf,
			sizeof from);
	else
		from.s_addr = INADDR_ANY;

	/* Record the number of seconds since we started sending. */
	if (client -> state == S_REQUESTING)
		client -> packet.secs = client -> secs;
	else {
		if (interval < 65536)
			client -> packet.secs = htons (interval);
		else
			client -> packet.secs = htons (65535);
	}

	log_info ("DHCPREQUEST on %s to %s port %d",
	      client -> name ? client -> name : client -> interface -> name,
	      inet_ntoa (destination.sin_addr),
	      ntohs (destination.sin_port));

	if (destination.sin_addr.s_addr != INADDR_BROADCAST &&
	    fallback_interface) {
		result = send_packet(fallback_interface, NULL, &client->packet,
				     client->packet_length, from, &destination,
				     NULL);
		if (result < 0) {
			log_error("%s:%d: Failed to send %d byte long packet "
				  "over %s interface.", MDL,
				  client->packet_length,
				  fallback_interface->name);
		}
        }
	else {
		/* Send out a packet. */
		result = send_packet(client->interface, NULL, &client->packet,
				     client->packet_length, from, &destination,
				     NULL);
		if (result < 0) {
			log_error("%s:%d: Failed to send %d byte long packet"
				  " over %s interface.", MDL,
				  client->packet_length,
				  client->interface->name);
		}
        }

	tv.tv_sec = cur_tv.tv_sec + client->interval;
	tv.tv_usec = ((tv.tv_sec - cur_tv.tv_sec) > 1) ?
			random() % 1000000 : cur_tv.tv_usec;
	add_timeout(&tv, send_request, client, 0, 0);
}

//ScenSim-Port//void send_decline (cpp)
//ScenSim-Port//	void *cpp;
void send_decline (void *cpp)//ScenSim-Port//
{
//ScenSim-Port//	struct client_state *client = cpp;
	struct client_state *client = (client_state*)cpp;//ScenSim-Port//

	int result;

	log_info ("DHCPDECLINE on %s to %s port %d",
	      client->name ? client->name : client->interface->name,
	      inet_ntoa(sockaddr_broadcast.sin_addr),
	      ntohs(sockaddr_broadcast.sin_port));

	/* Send out a packet. */
	result = send_packet(client->interface, NULL, &client->packet,
			     client->packet_length, inaddr_any,
			     &sockaddr_broadcast, NULL);
	if (result < 0) {
		log_error("%s:%d: Failed to send %d byte long packet over %s"
			  " interface.", MDL, client->packet_length,
			  client->interface->name);
	}
}

//ScenSim-Port//void send_release (cpp)
//ScenSim-Port//	void *cpp;
void send_release (void *cpp)//ScenSim-Port//
{
//ScenSim-Port//	struct client_state *client = cpp;
	struct client_state *client = (client_state*)cpp;//ScenSim-Port//

	int result;
	struct sockaddr_in destination;
	struct in_addr from;

	memcpy (&from, client -> active -> address.iabuf,
		sizeof from);
	memcpy (&destination.sin_addr.s_addr,
		client -> destination.iabuf,
		sizeof destination.sin_addr.s_addr);
	destination.sin_port = remote_port;
	destination.sin_family = AF_INET;
#ifdef HAVE_SA_LEN
	destination.sin_len = sizeof destination;
#endif

	/* Set the lease to end now, so that we don't accidentally
	   reuse it if we restart before the old expiry time. */
	client -> active -> expiry =
		client -> active -> renewal =
		client -> active -> rebind = cur_time;
	if (!write_client_lease (client, client -> active, 1, 1)) {
		log_error ("Can't release lease: lease write failed.");
		return;
	}

	log_info ("DHCPRELEASE on %s to %s port %d",
	      client -> name ? client -> name : client -> interface -> name,
	      inet_ntoa (destination.sin_addr),
	      ntohs (destination.sin_port));

	if (fallback_interface) {
		result = send_packet(fallback_interface, NULL, &client->packet,
				      client->packet_length, from, &destination,
				      NULL);
		if (result < 0) {
			log_error("%s:%d: Failed to send %d byte long packet"
				  " over %s interface.", MDL,
				  client->packet_length,
				  fallback_interface->name);
		}
        } else {
		/* Send out a packet. */
		result = send_packet(client->interface, NULL, &client->packet,
				      client->packet_length, from, &destination,
				      NULL);
		if (result < 0) {
			log_error ("%s:%d: Failed to send %d byte long packet"
				   " over %s interface.", MDL,
				   client->packet_length,
				   client->interface->name);
		}

        }
}

void
make_client_options(struct client_state *client, struct client_lease *lease,
		    u_int8_t *type, struct option_cache *sid,
		    struct iaddr *rip, struct option **prl,
		    struct option_state **op)
{
	unsigned i;
	struct option_cache *oc;
	struct option *option = NULL;
	struct buffer *bp = (struct buffer *)0;

	/* If there are any leftover options, get rid of them. */
	if (*op)
		option_state_dereference (op, MDL);

	/* Allocate space for options. */
	option_state_allocate (op, MDL);

	/* Send the server identifier if provided. */
	if (sid)
		save_option (&dhcp_universe, *op, sid);

	oc = (struct option_cache *)0;

	/* Send the requested address if provided. */
	if (rip) {
		client -> requested_address = *rip;
		i = DHO_DHCP_REQUESTED_ADDRESS;
		if (!(option_code_hash_lookup(&option, dhcp_universe.code_hash,
					      &i, 0, MDL) &&
		      make_const_option_cache(&oc, NULL, rip->iabuf, rip->len,
					      option, MDL)))
			log_error ("can't make requested address cache.");
		else {
			save_option (&dhcp_universe, *op, oc);
			option_cache_dereference (&oc, MDL);
		}
		option_dereference(&option, MDL);
	} else {
		client -> requested_address.len = 0;
	}

	i = DHO_DHCP_MESSAGE_TYPE;
	if (!(option_code_hash_lookup(&option, dhcp_universe.code_hash, &i, 0,
				      MDL) &&
	      make_const_option_cache(&oc, NULL, type, 1, option, MDL)))
		log_error ("can't make message type.");
	else {
		save_option (&dhcp_universe, *op, oc);
		option_cache_dereference (&oc, MDL);
	}
	option_dereference(&option, MDL);

	if (prl) {
		int len;

		/* Probe the length of the list. */
		len = 0;
		for (i = 0 ; prl[i] != NULL ; i++)
			if (prl[i]->universe == &dhcp_universe)
				len++;

		if (!buffer_allocate (&bp, len, MDL))
			log_error ("can't make parameter list buffer.");
		else {
			unsigned code = DHO_DHCP_PARAMETER_REQUEST_LIST;

			len = 0;
			for (i = 0 ; prl[i] != NULL ; i++)
				if (prl[i]->universe == &dhcp_universe)
					bp->data[len++] = prl[i]->code;

			if (!(option_code_hash_lookup(&option,
						      dhcp_universe.code_hash,
						      &code, 0, MDL) &&
			      make_const_option_cache(&oc, &bp, NULL, len,
						      option, MDL)))
				log_error ("can't make option cache");
			else {
				save_option (&dhcp_universe, *op, oc);
				option_cache_dereference (&oc, MDL);
			}
			option_dereference(&option, MDL);
		}
	}

	/* Run statements that need to be run on transmission. */
	if (client -> config -> on_transmission)
		execute_statements_in_scope
			((struct binding_value **)0,
			 (struct packet *)0, (struct lease *)0, client,
			 (lease ? lease -> options : (struct option_state *)0),
			 *op, &global_scope,
			 client -> config -> on_transmission,
			 (struct group *)0);
}

//ScenSim-Port//void make_discover (client, lease)
//ScenSim-Port//	struct client_state *client;
//ScenSim-Port//	struct client_lease *lease;
void make_discover (struct client_state *client, struct client_lease *lease)//ScenSim-Port//
{
	unsigned char discover = DHCPDISCOVER;
	struct option_state *options = (struct option_state *)0;

	memset (&client -> packet, 0, sizeof (client -> packet));

	make_client_options (client,
			     lease, &discover, (struct option_cache *)0,
			     lease ? &lease -> address : (struct iaddr *)0,
			     client -> config -> requested_options,
			     &options);

	/* Set up the option buffer... */
	client -> packet_length =
		cons_options ((struct packet *)0, &client -> packet,
			      (struct lease *)0, client,
			      /* maximum packet size */1500,
			      (struct option_state *)0,
			      options,
			      /* scope */ &global_scope,
			      /* overload */ 0,
			      /* terminate */0,
			      /* bootpp    */0,
			      (struct data_string *)0,
			      client -> config -> vendor_space_name);

	option_state_dereference (&options, MDL);
	if (client -> packet_length < BOOTP_MIN_LEN)
		client -> packet_length = BOOTP_MIN_LEN;

	client -> packet.op = BOOTREQUEST;
	client -> packet.htype = client -> interface -> hw_address.hbuf [0];
	client -> packet.hlen = client -> interface -> hw_address.hlen - 1;
	client -> packet.hops = 0;
	client -> packet.xid = random ();
	client -> packet.secs = 0; /* filled in by send_discover. */

	if (can_receive_unicast_unconfigured (client -> interface))
		client -> packet.flags = 0;
	else
		client -> packet.flags = htons (BOOTP_BROADCAST);

	memset (&(client -> packet.ciaddr),
		0, sizeof client -> packet.ciaddr);
	memset (&(client -> packet.yiaddr),
		0, sizeof client -> packet.yiaddr);
	memset (&(client -> packet.siaddr),
		0, sizeof client -> packet.siaddr);
//ScenSim-Port//	client -> packet.giaddr = giaddr;
	client -> packet.giaddr = giaddr_porting;//ScenSim-Port//
	if (client -> interface -> hw_address.hlen > 0)
	    memcpy (client -> packet.chaddr,
		    &client -> interface -> hw_address.hbuf [1],
		    (unsigned)(client -> interface -> hw_address.hlen - 1));

#ifdef DEBUG_PACKET
	dump_raw ((unsigned char *)&client -> packet, client -> packet_length);
#endif
}


//ScenSim-Port//void make_request (client, lease)
//ScenSim-Port//	struct client_state *client;
//ScenSim-Port//	struct client_lease *lease;
void make_request (struct client_state *client, struct client_lease *lease)//ScenSim-Port//
{
	unsigned char request = DHCPREQUEST;
	struct option_cache *oc;

	memset (&client -> packet, 0, sizeof (client -> packet));

	if (client -> state == S_REQUESTING)
		oc = lookup_option (&dhcp_universe, lease -> options,
				    DHO_DHCP_SERVER_IDENTIFIER);
	else
		oc = (struct option_cache *)0;

	if (client -> sent_options)
		option_state_dereference (&client -> sent_options, MDL);

	make_client_options (client, lease, &request, oc,
			     ((client -> state == S_REQUESTING ||
			       client -> state == S_REBOOTING)
			      ? &lease -> address
			      : (struct iaddr *)0),
			     client -> config -> requested_options,
			     &client -> sent_options);

	/* Set up the option buffer... */
	client -> packet_length =
		cons_options ((struct packet *)0, &client -> packet,
			      (struct lease *)0, client,
			      /* maximum packet size */1500,
			      (struct option_state *)0,
			      client -> sent_options,
			      /* scope */ &global_scope,
			      /* overload */ 0,
			      /* terminate */0,
			      /* bootpp    */0,
			      (struct data_string *)0,
			      client -> config -> vendor_space_name);

	if (client -> packet_length < BOOTP_MIN_LEN)
		client -> packet_length = BOOTP_MIN_LEN;

	client -> packet.op = BOOTREQUEST;
	client -> packet.htype = client -> interface -> hw_address.hbuf [0];
	client -> packet.hlen = client -> interface -> hw_address.hlen - 1;
	client -> packet.hops = 0;
	client -> packet.xid = client -> xid;
	client -> packet.secs = 0; /* Filled in by send_request. */

	/* If we own the address we're requesting, put it in ciaddr;
	   otherwise set ciaddr to zero. */
	if (client -> state == S_BOUND ||
	    client -> state == S_RENEWING ||
	    client -> state == S_REBINDING) {
		memcpy (&client -> packet.ciaddr,
			lease -> address.iabuf, lease -> address.len);
		client -> packet.flags = 0;
	} else {
		memset (&client -> packet.ciaddr, 0,
			sizeof client -> packet.ciaddr);
		if (can_receive_unicast_unconfigured (client -> interface))
			client -> packet.flags = 0;
		else
			client -> packet.flags = htons (BOOTP_BROADCAST);
	}

	memset (&client -> packet.yiaddr, 0,
		sizeof client -> packet.yiaddr);
	memset (&client -> packet.siaddr, 0,
		sizeof client -> packet.siaddr);
	if (client -> state != S_BOUND &&
	    client -> state != S_RENEWING)
//ScenSim-Port//		client -> packet.giaddr = giaddr;
		client -> packet.giaddr = giaddr_porting;//ScenSim-Port//
	else
		memset (&client -> packet.giaddr, 0,
			sizeof client -> packet.giaddr);
	if (client -> interface -> hw_address.hlen > 0)
	    memcpy (client -> packet.chaddr,
		    &client -> interface -> hw_address.hbuf [1],
		    (unsigned)(client -> interface -> hw_address.hlen - 1));

#ifdef DEBUG_PACKET
	dump_raw ((unsigned char *)&client -> packet, client -> packet_length);
#endif
}

//ScenSim-Port//void make_decline (client, lease)
//ScenSim-Port//	struct client_state *client;
//ScenSim-Port//	struct client_lease *lease;
void make_decline (struct client_state *client, struct client_lease *lease)//ScenSim-Port//
{
	unsigned char decline = DHCPDECLINE;
	struct option_cache *oc;

	struct option_state *options = (struct option_state *)0;

	/* Create the options cache. */
	oc = lookup_option (&dhcp_universe, lease -> options,
			    DHO_DHCP_SERVER_IDENTIFIER);
	make_client_options(client, lease, &decline, oc, &lease->address,
			    NULL, &options);

	/* Consume the options cache into the option buffer. */
	memset (&client -> packet, 0, sizeof (client -> packet));
	client -> packet_length =
		cons_options ((struct packet *)0, &client -> packet,
			      (struct lease *)0, client, 0,
			      (struct option_state *)0, options,
			      &global_scope, 0, 0, 0, (struct data_string *)0,
			      client -> config -> vendor_space_name);

	/* Destroy the options cache. */
	option_state_dereference (&options, MDL);

	if (client -> packet_length < BOOTP_MIN_LEN)
		client -> packet_length = BOOTP_MIN_LEN;

	client -> packet.op = BOOTREQUEST;
	client -> packet.htype = client -> interface -> hw_address.hbuf [0];
	client -> packet.hlen = client -> interface -> hw_address.hlen - 1;
	client -> packet.hops = 0;
	client -> packet.xid = client -> xid;
	client -> packet.secs = 0; /* Filled in by send_request. */
	if (can_receive_unicast_unconfigured (client -> interface))
		client -> packet.flags = 0;
	else
		client -> packet.flags = htons (BOOTP_BROADCAST);

	/* ciaddr must always be zero. */
	memset (&client -> packet.ciaddr, 0,
		sizeof client -> packet.ciaddr);
	memset (&client -> packet.yiaddr, 0,
		sizeof client -> packet.yiaddr);
	memset (&client -> packet.siaddr, 0,
		sizeof client -> packet.siaddr);
//ScenSim-Port//	client -> packet.giaddr = giaddr;
	client -> packet.giaddr = giaddr_porting;//ScenSim-Port//
	memcpy (client -> packet.chaddr,
		&client -> interface -> hw_address.hbuf [1],
		client -> interface -> hw_address.hlen);

#ifdef DEBUG_PACKET
	dump_raw ((unsigned char *)&client -> packet, client -> packet_length);
#endif
}

//ScenSim-Port//void make_release (client, lease)
//ScenSim-Port//	struct client_state *client;
//ScenSim-Port//	struct client_lease *lease;
void make_release (struct client_state *client, struct client_lease *lease)//ScenSim-Port//
{
	unsigned char request = DHCPRELEASE;
	struct option_cache *oc;

	struct option_state *options = (struct option_state *)0;

	memset (&client -> packet, 0, sizeof (client -> packet));

	oc = lookup_option (&dhcp_universe, lease -> options,
			    DHO_DHCP_SERVER_IDENTIFIER);
	make_client_options(client, lease, &request, oc, NULL, NULL, &options);

	/* Set up the option buffer... */
	client -> packet_length =
		cons_options ((struct packet *)0, &client -> packet,
			      (struct lease *)0, client,
			      /* maximum packet size */1500,
			      (struct option_state *)0,
			      options,
			      /* scope */ &global_scope,
			      /* overload */ 0,
			      /* terminate */0,
			      /* bootpp    */0,
			      (struct data_string *)0,
			      client -> config -> vendor_space_name);

	if (client -> packet_length < BOOTP_MIN_LEN)
		client -> packet_length = BOOTP_MIN_LEN;
	option_state_dereference (&options, MDL);

	client -> packet.op = BOOTREQUEST;
	client -> packet.htype = client -> interface -> hw_address.hbuf [0];
	client -> packet.hlen = client -> interface -> hw_address.hlen - 1;
	client -> packet.hops = 0;
	client -> packet.xid = random ();
	client -> packet.secs = 0;
	client -> packet.flags = 0;
	memcpy (&client -> packet.ciaddr,
		lease -> address.iabuf, lease -> address.len);
	memset (&client -> packet.yiaddr, 0,
		sizeof client -> packet.yiaddr);
	memset (&client -> packet.siaddr, 0,
		sizeof client -> packet.siaddr);
//ScenSim-Port//	client -> packet.giaddr = giaddr;
	client -> packet.giaddr = giaddr_porting;//ScenSim-Port//
	memcpy (client -> packet.chaddr,
		&client -> interface -> hw_address.hbuf [1],
		client -> interface -> hw_address.hlen);

#ifdef DEBUG_PACKET
	dump_raw ((unsigned char *)&client -> packet, client -> packet_length);
#endif
}

//ScenSim-Port//void destroy_client_lease (lease)
//ScenSim-Port//	struct client_lease *lease;
void destroy_client_lease (struct client_lease *lease)//ScenSim-Port//
{
	if (lease -> server_name)
		dfree (lease -> server_name, MDL);
	if (lease -> filename)
		dfree (lease -> filename, MDL);
	option_state_dereference (&lease -> options, MDL);
	free_client_lease (lease, MDL);
}

//ScenSim-Port//FILE *leaseFile = NULL;
//ScenSim-Port//int leases_written = 0;

void rewrite_client_leases ()
{
	struct interface_info *ip;
	struct client_state *client;
	struct client_lease *lp;

	if (leaseFile != NULL)
		fclose (leaseFile);
	leaseFile = fopen (path_dhclient_db, "w");
	if (leaseFile == NULL) {
		log_error ("can't create %s: %m", path_dhclient_db);
		return;
	}

	/* If there is a default duid, write it out. */
	if (default_duid.len != 0)
		write_duid(&default_duid);

	/* Write out all the leases attached to configured interfaces that
	   we know about. */
	for (ip = interfaces; ip; ip = ip -> next) {
		for (client = ip -> client; client; client = client -> next) {
			for (lp = client -> leases; lp; lp = lp -> next) {
				write_client_lease (client, lp, 1, 0);
			}
			if (client -> active)
				write_client_lease (client,
						    client -> active, 1, 0);

			if (client->active_lease != NULL)
				write_client6_lease(client,
						    client->active_lease,
						    1, 0);

			/* Reset last_write after rewrites. */
			client->last_write = 0;
		}
	}

	/* Write out any leases that are attached to interfaces that aren't
	   currently configured. */
	for (ip = dummy_interfaces; ip; ip = ip -> next) {
		for (client = ip -> client; client; client = client -> next) {
			for (lp = client -> leases; lp; lp = lp -> next) {
				write_client_lease (client, lp, 1, 0);
			}
			if (client -> active)
				write_client_lease (client,
						    client -> active, 1, 0);

			if (client->active_lease != NULL)
				write_client6_lease(client,
						    client->active_lease,
						    1, 0);

			/* Reset last_write after rewrites. */
			client->last_write = 0;
		}
	}
	fflush (leaseFile);
}

void write_lease_option (struct option_cache *oc,
			 struct packet *packet, struct lease *lease,
			 struct client_state *client_state,
			 struct option_state *in_options,
			 struct option_state *cfg_options,
			 struct binding_scope **scope,
			 struct universe *u, void *stuff)
{
	const char *name, *dot;
	struct data_string ds;
//ScenSim-Port//	char *preamble = stuff;
	char *preamble = (char*)stuff;//ScenSim-Port//

	memset (&ds, 0, sizeof ds);

	if (u != &dhcp_universe) {
		name = u -> name;
		dot = ".";
	} else {
		name = "";
		dot = "";
	}
	if (evaluate_option_cache (&ds, packet, lease, client_state,
				   in_options, cfg_options, scope, oc, MDL)) {
		fprintf(leaseFile, "%soption %s%s%s %s;\n", preamble,
			name, dot, oc->option->name,
			pretty_print_option(oc->option, ds.data, ds.len,
					    1, 1));
		data_string_forget (&ds, MDL);
	}
}

/* Write an option cache to the lease store. */
static void
write_options(struct client_state *client, struct option_state *options,
	      const char *preamble)
{
	int i;

	for (i = 0; i < options->universe_count; i++) {
		option_space_foreach(NULL, NULL, client, NULL, options,
//ScenSim-Port//				     &global_scope, universes[i],
				     &global_scope, universes_[i],//ScenSim-Port//
				     (char *)preamble, write_lease_option);
	}
}

/* Write the default DUID to the lease store. */
static isc_result_t
write_duid(struct data_string *duid)
{
	char *str;
	int stat;

	if ((duid == NULL) || (duid->len <= 2))
		return DHCP_R_INVALIDARG;

	if (leaseFile == NULL) {	/* XXX? */
		leaseFile = fopen(path_dhclient_db, "w");
		if (leaseFile == NULL) {
			log_error("can't create %s: %m", path_dhclient_db);
			return ISC_R_IOERROR;
		}
	}

	/* It would make more sense to write this as a hex string,
	 * but our function to do that (print_hex_n) uses a fixed
	 * length buffer...and we can't guarantee a duid would be
	 * less than the fixed length.
	 */
	str = quotify_buf(duid->data, duid->len, MDL);
	if (str == NULL)
		return ISC_R_NOMEMORY;

	stat = fprintf(leaseFile, "default-duid \"%s\";\n", str);
	dfree(str, MDL);
	if (stat <= 0)
		return ISC_R_IOERROR;

	if (fflush(leaseFile) != 0)
		return ISC_R_IOERROR;

	return ISC_R_SUCCESS;
}

/* Write a DHCPv6 lease to the store. */
isc_result_t
write_client6_lease(struct client_state *client, struct dhc6_lease *lease,
		    int rewrite, int sync)
{
	struct dhc6_ia *ia;
	struct dhc6_addr *addr;
	int stat;
	const char *ianame;

	/* This should include the current lease. */
	if (!rewrite && (leases_written++ > 20)) {
		rewrite_client_leases();
		leases_written = 0;
		return ISC_R_SUCCESS;
	}

	if (client == NULL || lease == NULL)
		return DHCP_R_INVALIDARG;

	if (leaseFile == NULL) {	/* XXX? */
		leaseFile = fopen(path_dhclient_db, "w");
		if (leaseFile == NULL) {
			log_error("can't create %s: %m", path_dhclient_db);
			return ISC_R_IOERROR;
		}
	}

	stat = fprintf(leaseFile, "lease6 {\n");
	if (stat <= 0)
		return ISC_R_IOERROR;

	stat = fprintf(leaseFile, "  interface \"%s\";\n",
		       client->interface->name);
	if (stat <= 0)
		return ISC_R_IOERROR;

	for (ia = lease->bindings ; ia != NULL ; ia = ia->next) {
		switch (ia->ia_type) {
			case D6O_IA_NA:
			default:
				ianame = "ia-na";
				break;
			case D6O_IA_TA:
				ianame = "ia-ta";
				break;
			case D6O_IA_PD:
				ianame = "ia-pd";
				break;
		}
		stat = fprintf(leaseFile, "  %s %s {\n",
			       ianame, print_hex_1(4, ia->iaid, 12));
		if (stat <= 0)
			return ISC_R_IOERROR;

		if (ia->ia_type != D6O_IA_TA)
			stat = fprintf(leaseFile, "    starts %d;\n"
						  "    renew %u;\n"
						  "    rebind %u;\n",
				       (int)ia->starts, ia->renew, ia->rebind);
		else
			stat = fprintf(leaseFile, "    starts %d;\n",
				       (int)ia->starts);
		if (stat <= 0)
			return ISC_R_IOERROR;

		for (addr = ia->addrs ; addr != NULL ; addr = addr->next) {
			if (ia->ia_type != D6O_IA_PD)
				stat = fprintf(leaseFile,
					       "    iaaddr %s {\n",
					       piaddr(addr->address));
			else
				stat = fprintf(leaseFile,
					       "    iaprefix %s/%d {\n",
					       piaddr(addr->address),
					       (int)addr->plen);
			if (stat <= 0)
				return ISC_R_IOERROR;

			stat = fprintf(leaseFile, "      starts %d;\n"
						  "      preferred-life %u;\n"
						  "      max-life %u;\n",
				       (int)addr->starts, addr->preferred_life,
				       addr->max_life);
			if (stat <= 0)
				return ISC_R_IOERROR;

			if (addr->options != NULL)
				write_options(client, addr->options, "      ");

			stat = fprintf(leaseFile, "    }\n");
			if (stat <= 0)
				return ISC_R_IOERROR;
		}

		if (ia->options != NULL)
			write_options(client, ia->options, "    ");

		stat = fprintf(leaseFile, "  }\n");
		if (stat <= 0)
			return ISC_R_IOERROR;
	}

	if (lease->released) {
		stat = fprintf(leaseFile, "  released;\n");
		if (stat <= 0)
			return ISC_R_IOERROR;
	}

	if (lease->options != NULL)
		write_options(client, lease->options, "  ");

	stat = fprintf(leaseFile, "}\n");
	if (stat <= 0)
		return ISC_R_IOERROR;

	if (fflush(leaseFile) != 0)
		return ISC_R_IOERROR;

//ScenSim-Port//	if (sync) {
//ScenSim-Port//		if (fsync(fileno(leaseFile)) < 0) {
//ScenSim-Port//			log_error("write_client_lease: fsync(): %m");
//ScenSim-Port//			return ISC_R_IOERROR;
//ScenSim-Port//		}
//ScenSim-Port//	}

	return ISC_R_SUCCESS;
}

//ScenSim-Port//int write_client_lease (client, lease, rewrite, makesure)
//ScenSim-Port//	struct client_state *client;
//ScenSim-Port//	struct client_lease *lease;
//ScenSim-Port//	int rewrite;
//ScenSim-Port//	int makesure;
int write_client_lease (struct client_state *client, struct client_lease *lease, int rewrite, int makesure)//ScenSim-Port//
{
	struct data_string ds;
	int errors = 0;
	char *s;
	const char *tval;

	if (!rewrite) {
		if (leases_written++ > 20) {
			rewrite_client_leases ();
			leases_written = 0;
		}
	}

	/* If the lease came from the config file, we don't need to stash
	   a copy in the lease database. */
	if (lease -> is_static)
		return 1;

	if (leaseFile == NULL) {	/* XXX */
		leaseFile = fopen (path_dhclient_db, "w");
		if (leaseFile == NULL) {
			log_error ("can't create %s: %m", path_dhclient_db);
			return 0;
		}
	}

	errno = 0;
	fprintf (leaseFile, "lease {\n");
	if (lease -> is_bootp) {
		fprintf (leaseFile, "  bootp;\n");
		if (errno) {
			++errors;
			errno = 0;
		}
	}
	fprintf (leaseFile, "  interface \"%s\";\n",
		 client -> interface -> name);
	if (errno) {
		++errors;
		errno = 0;
	}
	if (client -> name) {
		fprintf (leaseFile, "  name \"%s\";\n", client -> name);
		if (errno) {
			++errors;
			errno = 0;
		}
	}
	fprintf (leaseFile, "  fixed-address %s;\n",
		 piaddr (lease -> address));
	if (errno) {
		++errors;
		errno = 0;
	}
	if (lease -> filename) {
		s = quotify_string (lease -> filename, MDL);
		if (s) {
			fprintf (leaseFile, "  filename \"%s\";\n", s);
			if (errno) {
				++errors;
				errno = 0;
			}
			dfree (s, MDL);
		} else
			errors++;

	}
	if (lease->server_name != NULL) {
		s = quotify_string(lease->server_name, MDL);
		if (s != NULL) {
			fprintf(leaseFile, "  server-name \"%s\";\n", s);
			if (errno) {
				++errors;
				errno = 0;
			}
			dfree(s, MDL);
		} else
			++errors;
	}
	if (lease -> medium) {
		s = quotify_string (lease -> medium -> string, MDL);
		if (s) {
			fprintf (leaseFile, "  medium \"%s\";\n", s);
			if (errno) {
				++errors;
				errno = 0;
			}
			dfree (s, MDL);
		} else
			errors++;
	}
	if (errno != 0) {
		errors++;
		errno = 0;
	}

	memset (&ds, 0, sizeof ds);

	write_options(client, lease->options, "  ");

	tval = print_time(lease->renewal);
	if (tval == NULL ||
	    fprintf(leaseFile, "  renew %s\n", tval) < 0)
		errors++;

	tval = print_time(lease->rebind);
	if (tval == NULL ||
	    fprintf(leaseFile, "  rebind %s\n", tval) < 0)
		errors++;

	tval = print_time(lease->expiry);
	if (tval == NULL ||
	    fprintf(leaseFile, "  expire %s\n", tval) < 0)
		errors++;

	if (fprintf(leaseFile, "}\n") < 0)
		errors++;

	if (fflush(leaseFile) != 0)
		errors++;

	client->last_write = cur_time;

//ScenSim-Port//	if (!errors && makesure) {
//ScenSim-Port//		if (fsync (fileno (leaseFile)) < 0) {
//ScenSim-Port//			log_info ("write_client_lease: %m");
//ScenSim-Port//			return 0;
//ScenSim-Port//		}
//ScenSim-Port//	}

	return errors ? 0 : 1;
}

/* Variables holding name of script and file pointer for writing to
   script.   Needless to say, this is not reentrant - only one script
   can be invoked at a time. */
char scriptName [256];
FILE *scriptFile;

//ScenSim-Port//void script_init (client, reason, medium)
//ScenSim-Port//	struct client_state *client;
//ScenSim-Port//	const char *reason;
//ScenSim-Port//	struct string_list *medium;
void script_init (struct client_state *client, const char *reason, struct string_list *medium)//ScenSim-Port//
{
	struct string_list *sl, *next;

	if (client) {
		for (sl = client -> env; sl; sl = next) {
			next = sl -> next;
			dfree (sl, MDL);
		}
		client -> env = (struct string_list *)0;
		client -> envc = 0;

		if (client -> interface) {
			client_envadd (client, "", "interface", "%s",
				       client -> interface -> name);
		}
		if (client -> name)
			client_envadd (client,
				       "", "client", "%s", client -> name);
		if (medium)
			client_envadd (client,
				       "", "medium", "%s", medium -> string);

		client_envadd (client, "", "reason", "%s", reason);
//ScenSim-Port//		client_envadd (client, "", "pid", "%ld", (long int)getpid ());
	}
}

void client_option_envadd (struct option_cache *oc,
			   struct packet *packet, struct lease *lease,
			   struct client_state *client_state,
			   struct option_state *in_options,
			   struct option_state *cfg_options,
			   struct binding_scope **scope,
			   struct universe *u, void *stuff)
{
//ScenSim-Port//	struct envadd_state *es = stuff;
	struct envadd_state *es = (envadd_state*)stuff;//ScenSim-Port//
	struct data_string data;
	memset (&data, 0, sizeof data);

	if (evaluate_option_cache (&data, packet, lease, client_state,
				   in_options, cfg_options, scope, oc, MDL)) {
		if (data.len) {
			char name [256];
			if (dhcp_option_ev_name (name, sizeof name,
						 oc->option)) {
				const char *value;
				size_t length;
				value = pretty_print_option(oc->option,
							    data.data,
							    data.len, 0, 0);
				length = strlen(value);

				if (check_option_values(oc->option->universe,
							oc->option->code,
							value, length) == 0) {
					client_envadd(es->client, es->prefix,
						      name, "%s", value);
				} else {
					log_error("suspect value in %s "
						  "option - discarded",
						  name);
				}
				data_string_forget (&data, MDL);
			}
		}
	}
}

//ScenSim-Port//void script_write_params (client, prefix, lease)
//ScenSim-Port//	struct client_state *client;
//ScenSim-Port//	const char *prefix;
//ScenSim-Port//	struct client_lease *lease;
void script_write_params (struct client_state *client, const char *prefix, struct client_lease *lease)//ScenSim-Port//
{
	int i;
	struct data_string data;
	struct option_cache *oc;
	struct envadd_state es;

	es.client = client;
	es.prefix = prefix;

	client_envadd (client,
		       prefix, "ip_address", "%s", piaddr (lease -> address));

	/* For the benefit of Linux (and operating systems which may
	   have similar needs), compute the network address based on
	   the supplied ip address and netmask, if provided.  Also
	   compute the broadcast address (the host address all ones
	   broadcast address, not the host address all zeroes
	   broadcast address). */

	memset (&data, 0, sizeof data);
	oc = lookup_option (&dhcp_universe, lease -> options, DHO_SUBNET_MASK);
	if (oc && evaluate_option_cache (&data, (struct packet *)0,
					 (struct lease *)0, client,
					 (struct option_state *)0,
					 lease -> options,
					 &global_scope, oc, MDL)) {
		if (data.len > 3) {
			struct iaddr netmask, subnet, broadcast;

			/*
			 * No matter the length of the subnet-mask option,
			 * use only the first four octets.  Note that
			 * subnet-mask options longer than 4 octets are not
			 * in conformance with RFC 2132, but servers with this
			 * flaw do exist.
			 */
			memcpy(netmask.iabuf, data.data, 4);
			netmask.len = 4;
			data_string_forget (&data, MDL);

			subnet = subnet_number (lease -> address, netmask);
			if (subnet.len) {
			    client_envadd (client, prefix, "network_number",
					   "%s", piaddr (subnet));

			    oc = lookup_option (&dhcp_universe,
						lease -> options,
						DHO_BROADCAST_ADDRESS);
			    if (!oc ||
				!(evaluate_option_cache
				  (&data, (struct packet *)0,
				   (struct lease *)0, client,
				   (struct option_state *)0,
				   lease -> options,
				   &global_scope, oc, MDL))) {
				broadcast = broadcast_addr (subnet, netmask);
				if (broadcast.len) {
				    client_envadd (client,
						   prefix, "broadcast_address",
						   "%s", piaddr (broadcast));
				}
			    }
			}
		}
		data_string_forget (&data, MDL);
	}

	if (lease->filename) {
		if (check_option_values(NULL, DHO_ROOT_PATH,
					lease->filename,
					strlen(lease->filename)) == 0) {
			client_envadd(client, prefix, "filename",
				      "%s", lease->filename);
		} else {
			log_error("suspect value in %s "
				  "option - discarded",
				  lease->filename);
		}
	}

	if (lease->server_name) {
		if (check_option_values(NULL, DHO_HOST_NAME,
					lease->server_name,
					strlen(lease->server_name)) == 0 ) {
			client_envadd (client, prefix, "server_name",
				       "%s", lease->server_name);
		} else {
			log_error("suspect value in %s "
				  "option - discarded",
				  lease->server_name);
		}
	}

	for (i = 0; i < lease -> options -> universe_count; i++) {
		option_space_foreach ((struct packet *)0, (struct lease *)0,
				      client, (struct option_state *)0,
				      lease -> options, &global_scope,
//ScenSim-Port//				      universes [i],
				      universes_ [i],//ScenSim-Port//
				      &es, client_option_envadd);
	}
	client_envadd (client, prefix, "expiry", "%d", (int)(lease -> expiry));
}

//ScenSim-Port//int script_go (client)
//ScenSim-Port//	struct client_state *client;
//ScenSim-Port//{
//ScenSim-Port//	char *scriptName;
//ScenSim-Port//	char *argv [2];
//ScenSim-Port//	char **envp;
//ScenSim-Port//	char reason [] = "REASON=NBI";
//ScenSim-Port//	static char client_path [] = CLIENT_PATH;
//ScenSim-Port//	int i;
//ScenSim-Port//	struct string_list *sp, *next;
//ScenSim-Port//	int pid, wpid, wstatus;
//ScenSim-Port//
//ScenSim-Port//	if (client)
//ScenSim-Port//		scriptName = client -> config -> script_name;
//ScenSim-Port//	else
//ScenSim-Port//		scriptName = top_level_config.script_name;
//ScenSim-Port//
//ScenSim-Port//	envp = dmalloc (((client ? client -> envc : 2) +
//ScenSim-Port//			 client_env_count + 2) * sizeof (char *), MDL);
//ScenSim-Port//	if (!envp) {
//ScenSim-Port//		log_error ("No memory for client script environment.");
//ScenSim-Port//		return 0;
//ScenSim-Port//	}
//ScenSim-Port//	i = 0;
//ScenSim-Port//	/* Copy out the environment specified on the command line,
//ScenSim-Port//	   if any. */
//ScenSim-Port//	for (sp = client_env; sp; sp = sp -> next) {
//ScenSim-Port//		envp [i++] = sp -> string;
//ScenSim-Port//	}
//ScenSim-Port//	/* Copy out the environment specified by dhclient. */
//ScenSim-Port//	if (client) {
//ScenSim-Port//		for (sp = client -> env; sp; sp = sp -> next) {
//ScenSim-Port//			envp [i++] = sp -> string;
//ScenSim-Port//		}
//ScenSim-Port//	} else {
//ScenSim-Port//		envp [i++] = reason;
//ScenSim-Port//	}
//ScenSim-Port//	/* Set $PATH. */
//ScenSim-Port//	envp [i++] = client_path;
//ScenSim-Port//	envp [i] = (char *)0;
//ScenSim-Port//
//ScenSim-Port//	argv [0] = scriptName;
//ScenSim-Port//	argv [1] = (char *)0;
//ScenSim-Port//
//ScenSim-Port//	pid = fork ();
//ScenSim-Port//	if (pid < 0) {
//ScenSim-Port//		log_error ("fork: %m");
//ScenSim-Port//		wstatus = 0;
//ScenSim-Port//	} else if (pid) {
//ScenSim-Port//		do {
//ScenSim-Port//			wpid = wait (&wstatus);
//ScenSim-Port//		} while (wpid != pid && wpid > 0);
//ScenSim-Port//		if (wpid < 0) {
//ScenSim-Port//			log_error ("wait: %m");
//ScenSim-Port//			wstatus = 0;
//ScenSim-Port//		}
//ScenSim-Port//	} else {
//ScenSim-Port//		/* We don't want to pass an open file descriptor for
//ScenSim-Port//		 * dhclient.leases when executing dhclient-script.
//ScenSim-Port//		 */
//ScenSim-Port//		if (leaseFile != NULL)
//ScenSim-Port//			fclose(leaseFile);
//ScenSim-Port//		execve (scriptName, argv, envp);
//ScenSim-Port//		log_error ("execve (%s, ...): %m", scriptName);
//ScenSim-Port//		exit (0);
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (client) {
//ScenSim-Port//		for (sp = client -> env; sp; sp = next) {
//ScenSim-Port//			next = sp -> next;
//ScenSim-Port//			dfree (sp, MDL);
//ScenSim-Port//		}
//ScenSim-Port//		client -> env = (struct string_list *)0;
//ScenSim-Port//		client -> envc = 0;
//ScenSim-Port//	}
//ScenSim-Port//	dfree (envp, MDL);
//ScenSim-Port//	gettimeofday(&cur_tv, NULL);
//ScenSim-Port//	return (WIFEXITED (wstatus) ?
//ScenSim-Port//		WEXITSTATUS (wstatus) : -WTERMSIG (wstatus));
//ScenSim-Port//}

void client_envadd (struct client_state *client,
		    const char *prefix, const char *name, const char *fmt, ...)
{
	char spbuf [1024];
	char *s;
	unsigned len;
	struct string_list *val;
	va_list list;

	va_start (list, fmt);
	len = vsnprintf (spbuf, sizeof spbuf, fmt, list);
	va_end (list);

//ScenSim-Port//	val = dmalloc (strlen (prefix) + strlen (name) + 1 /* = */ +
	val = (string_list*)dmalloc (strlen (prefix) + strlen (name) + 1 /* = */ +//ScenSim-Port//
		       len + sizeof *val, MDL);
	if (!val)
		return;
	s = val -> string;
	strcpy (s, prefix);
	strcat (s, name);
	s += strlen (s);
	*s++ = '=';
	if (len >= sizeof spbuf) {
		va_start (list, fmt);
		vsnprintf (s, len + 1, fmt, list);
		va_end (list);
	} else
		strcpy (s, spbuf);
	val -> next = client -> env;
	client -> env = val;
	client -> envc++;
}

//ScenSim-Port//int dhcp_option_ev_name (buf, buflen, option)
//ScenSim-Port//	char *buf;
//ScenSim-Port//	size_t buflen;
//ScenSim-Port//	struct option *option;
int dhcp_option_ev_name (char *buf, size_t buflen, struct option *option)//ScenSim-Port//
{
	int i, j;
	const char *s;

	j = 0;
	if (option -> universe != &dhcp_universe) {
		s = option -> universe -> name;
		i = 0;
	} else {
		s = option -> name;
		i = 1;
	}

	do {
		while (*s) {
			if (j + 1 == buflen)
				return 0;
			if (*s == '-')
				buf [j++] = '_';
			else
				buf [j++] = *s;
			++s;
		}
		if (!i) {
			s = option -> name;
			if (j + 1 == buflen)
				return 0;
			buf [j++] = '_';
		}
		++i;
	} while (i != 2);

	buf [j] = 0;
	return 1;
}

//ScenSim-Port//void go_daemon ()
//ScenSim-Port//{
//ScenSim-Port//	static int state = 0;
//ScenSim-Port//	int pid;
//ScenSim-Port//
//ScenSim-Port//	/* Don't become a daemon if the user requested otherwise. */
//ScenSim-Port//	if (no_daemon) {
//ScenSim-Port//		write_client_pid_file ();
//ScenSim-Port//		return;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/* Only do it once. */
//ScenSim-Port//	if (state)
//ScenSim-Port//		return;
//ScenSim-Port//	state = 1;
//ScenSim-Port//
//ScenSim-Port//	/* Stop logging to stderr... */
//ScenSim-Port//	log_perror = 0;
//ScenSim-Port//
//ScenSim-Port//	/* Become a daemon... */
//ScenSim-Port//	if ((pid = fork ()) < 0)
//ScenSim-Port//		log_fatal ("Can't fork daemon: %m");
//ScenSim-Port//	else if (pid)
//ScenSim-Port//		exit (0);
//ScenSim-Port//	/* Become session leader and get pid... */
//ScenSim-Port//	pid = setsid ();
//ScenSim-Port//
//ScenSim-Port//	/* Close standard I/O descriptors. */
//ScenSim-Port//	close(0);
//ScenSim-Port//	close(1);
//ScenSim-Port//	close(2);
//ScenSim-Port//
//ScenSim-Port//	/* Reopen them on /dev/null. */
//ScenSim-Port//	open("/dev/null", O_RDWR);
//ScenSim-Port//	open("/dev/null", O_RDWR);
//ScenSim-Port//	open("/dev/null", O_RDWR);
//ScenSim-Port//
//ScenSim-Port//	write_client_pid_file ();
//ScenSim-Port//
//ScenSim-Port//	IGNORE_RET (chdir("/"));
//ScenSim-Port//}

//ScenSim-Port//void write_client_pid_file ()
//ScenSim-Port//{
//ScenSim-Port//	FILE *pf;
//ScenSim-Port//	int pfdesc;
//ScenSim-Port//
//ScenSim-Port//	/* nothing to do if the user doesn't want a pid file */
//ScenSim-Port//	if (no_pid_file == ISC_TRUE) {
//ScenSim-Port//		return;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	pfdesc = open (path_dhclient_pid, O_CREAT | O_TRUNC | O_WRONLY, 0644);
//ScenSim-Port//
//ScenSim-Port//	if (pfdesc < 0) {
//ScenSim-Port//		log_error ("Can't create %s: %m", path_dhclient_pid);
//ScenSim-Port//		return;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	pf = fdopen (pfdesc, "w");
//ScenSim-Port//	if (!pf) {
//ScenSim-Port//		close(pfdesc);
//ScenSim-Port//		log_error ("Can't fdopen %s: %m", path_dhclient_pid);
//ScenSim-Port//	} else {
//ScenSim-Port//		fprintf (pf, "%ld\n", (long)getpid ());
//ScenSim-Port//		fclose (pf);
//ScenSim-Port//	}
//ScenSim-Port//}

void client_location_changed ()
{
	struct interface_info *ip;
	struct client_state *client;

	for (ip = interfaces; ip; ip = ip -> next) {
		for (client = ip -> client; client; client = client -> next) {
			switch (client -> state) {
			      case S_SELECTING:
				cancel_timeout (send_discover, client);
				break;

			      case S_BOUND:
				cancel_timeout (state_bound, client);
				break;

			      case S_REBOOTING:
			      case S_REQUESTING:
			      case S_RENEWING:
				cancel_timeout (send_request, client);
				break;

			      case S_INIT:
			      case S_REBINDING:
			      case S_STOPPED:
				break;
			}
			client -> state = S_INIT;
			state_reboot (client);
		}
	}
}

//ScenSim-Port//void do_release(client)
//ScenSim-Port//	struct client_state *client;
void do_release(struct client_state *client)//ScenSim-Port//
{
	struct data_string ds;
	struct option_cache *oc;

	/* Pick a random xid. */
	client -> xid = random ();

	/* is there even a lease to release? */
	if (client -> active) {
		/* Make a DHCPRELEASE packet, and set appropriate per-interface
		   flags. */
		make_release (client, client -> active);

		memset (&ds, 0, sizeof ds);
		oc = lookup_option (&dhcp_universe,
				    client -> active -> options,
				    DHO_DHCP_SERVER_IDENTIFIER);
		if (oc &&
		    evaluate_option_cache (&ds, (struct packet *)0,
					   (struct lease *)0, client,
					   (struct option_state *)0,
					   client -> active -> options,
					   &global_scope, oc, MDL)) {
			if (ds.len > 3) {
				memcpy (client -> destination.iabuf,
					ds.data, 4);
				client -> destination.len = 4;
			} else
				client -> destination = iaddr_broadcast;

			data_string_forget (&ds, MDL);
		} else
			client -> destination = iaddr_broadcast;
		client -> first_sending = cur_time;
		client -> interval = client -> config -> initial_interval;

		/* Zap the medium list... */
		client -> medium = (struct string_list *)0;

		/* Send out the first and only DHCPRELEASE packet. */
		send_release (client);

		/* Do the client script RELEASE operation. */
//ScenSim-Port//		script_init (client,
//ScenSim-Port//			     "RELEASE", (struct string_list *)0);
//ScenSim-Port//		if (client -> alias)
//ScenSim-Port//			script_write_params (client, "alias_",
//ScenSim-Port//					     client -> alias);
//ScenSim-Port//		script_write_params (client, "old_", client -> active);
//ScenSim-Port//		script_go (client);
	}

	/* Cancel any timeouts. */
	cancel_timeout (state_bound, client);
	cancel_timeout (send_discover, client);
	cancel_timeout (state_init, client);
	cancel_timeout (send_request, client);
	cancel_timeout (state_reboot, client);
	client -> state = S_STOPPED;
}

//ScenSim-Port//int dhclient_interface_shutdown_hook (struct interface_info *interface)
//ScenSim-Port//{
//ScenSim-Port//	do_release (interface -> client);
//ScenSim-Port//
//ScenSim-Port//	return 1;
//ScenSim-Port//}

//ScenSim-Port//int dhclient_interface_discovery_hook (struct interface_info *tmp)
//ScenSim-Port//{
//ScenSim-Port//	struct interface_info *last, *ip;
//ScenSim-Port//	/* See if we can find the client from dummy_interfaces */
//ScenSim-Port//	last = 0;
//ScenSim-Port//	for (ip = dummy_interfaces; ip; ip = ip -> next) {
//ScenSim-Port//		if (!strcmp (ip -> name, tmp -> name)) {
//ScenSim-Port//			/* Remove from dummy_interfaces */
//ScenSim-Port//			if (last) {
//ScenSim-Port//				ip = (struct interface_info *)0;
//ScenSim-Port//				interface_reference (&ip, last -> next, MDL);
//ScenSim-Port//				interface_dereference (&last -> next, MDL);
//ScenSim-Port//				if (ip -> next) {
//ScenSim-Port//					interface_reference (&last -> next,
//ScenSim-Port//							     ip -> next, MDL);
//ScenSim-Port//					interface_dereference (&ip -> next,
//ScenSim-Port//							       MDL);
//ScenSim-Port//				}
//ScenSim-Port//			} else {
//ScenSim-Port//				ip = (struct interface_info *)0;
//ScenSim-Port//				interface_reference (&ip,
//ScenSim-Port//						     dummy_interfaces, MDL);
//ScenSim-Port//				interface_dereference (&dummy_interfaces, MDL);
//ScenSim-Port//				if (ip -> next) {
//ScenSim-Port//					interface_reference (&dummy_interfaces,
//ScenSim-Port//							     ip -> next, MDL);
//ScenSim-Port//					interface_dereference (&ip -> next,
//ScenSim-Port//							       MDL);
//ScenSim-Port//				}
//ScenSim-Port//			}
//ScenSim-Port//			/* Copy "client" to tmp */
//ScenSim-Port//			if (ip -> client) {
//ScenSim-Port//				tmp -> client = ip -> client;
//ScenSim-Port//				tmp -> client -> interface = tmp;
//ScenSim-Port//			}
//ScenSim-Port//			interface_dereference (&ip, MDL);
//ScenSim-Port//			break;
//ScenSim-Port//		}
//ScenSim-Port//		last = ip;
//ScenSim-Port//	}
//ScenSim-Port//	return 1;
//ScenSim-Port//}

//ScenSim-Port//isc_result_t dhclient_interface_startup_hook (struct interface_info *interface)
//ScenSim-Port//{
//ScenSim-Port//	struct interface_info *ip;
//ScenSim-Port//	struct client_state *client;
//ScenSim-Port//
//ScenSim-Port//	/* This code needs some rethinking.   It doesn't test against
//ScenSim-Port//	   a signal name, and it just kind of bulls into doing something
//ScenSim-Port//	   that may or may not be appropriate. */
//ScenSim-Port//
//ScenSim-Port//	if (interfaces) {
//ScenSim-Port//		interface_reference (&interface -> next, interfaces, MDL);
//ScenSim-Port//		interface_dereference (&interfaces, MDL);
//ScenSim-Port//	}
//ScenSim-Port//	interface_reference (&interfaces, interface, MDL);
//ScenSim-Port//
//ScenSim-Port//	discover_interfaces (DISCOVER_UNCONFIGURED);
//ScenSim-Port//
//ScenSim-Port//	for (ip = interfaces; ip; ip = ip -> next) {
//ScenSim-Port//		/* If interfaces were specified, don't configure
//ScenSim-Port//		   interfaces that weren't specified! */
//ScenSim-Port//		if (ip -> flags & INTERFACE_RUNNING ||
//ScenSim-Port//		   (ip -> flags & (INTERFACE_REQUESTED |
//ScenSim-Port//				     INTERFACE_AUTOMATIC)) !=
//ScenSim-Port//		     INTERFACE_REQUESTED)
//ScenSim-Port//			continue;
//ScenSim-Port//		script_init (ip -> client,
//ScenSim-Port//			     "PREINIT", (struct string_list *)0);
//ScenSim-Port//		if (ip -> client -> alias)
//ScenSim-Port//			script_write_params (ip -> client, "alias_",
//ScenSim-Port//					     ip -> client -> alias);
//ScenSim-Port//		script_go (ip -> client);
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	discover_interfaces (interfaces_requested != 0
//ScenSim-Port//			     ? DISCOVER_REQUESTED
//ScenSim-Port//			     : DISCOVER_RUNNING);
//ScenSim-Port//
//ScenSim-Port//	for (ip = interfaces; ip; ip = ip -> next) {
//ScenSim-Port//		if (ip -> flags & INTERFACE_RUNNING)
//ScenSim-Port//			continue;
//ScenSim-Port//		ip -> flags |= INTERFACE_RUNNING;
//ScenSim-Port//		for (client = ip->client ; client ; client = client->next) {
//ScenSim-Port//			client->state = S_INIT;
//ScenSim-Port//			state_reboot(client);
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//	return ISC_R_SUCCESS;
//ScenSim-Port//}

/* The client should never receive a relay agent information option,
   so if it does, log it and discard it. */

//ScenSim-Port//int parse_agent_information_option (packet, len, data)
//ScenSim-Port//	struct packet *packet;
//ScenSim-Port//	int len;
//ScenSim-Port//	u_int8_t *data;
int parse_agent_information_option (struct packet *packet, int len, u_int8_t *data)//ScenSim-Port//
{
	return 1;
}

/* The client never sends relay agent information options. */

//ScenSim-Port//unsigned cons_agent_information_options (cfg_options, outpacket,
//ScenSim-Port//					 agentix, length)
//ScenSim-Port//	struct option_state *cfg_options;
//ScenSim-Port//	struct dhcp_packet *outpacket;
//ScenSim-Port//	unsigned agentix;
//ScenSim-Port//	unsigned length;
unsigned cons_agent_information_options (struct option_state *cfg_options, struct dhcp_packet *outpacket, unsigned agentix, unsigned length)//ScenSim-Port//
{
	return length;
}

static void shutdown_exit (void *foo)
{
	exit (0);
}

#if defined (NSUPDATE)
/*
 * If the first query fails, the updater MUST NOT delete the DNS name.  It
 * may be that the host whose lease on the server has expired has moved
 * to another network and obtained a lease from a different server,
 * which has caused the client's A RR to be replaced. It may also be
 * that some other client has been configured with a name that matches
 * the name of the DHCP client, and the policy was that the last client
 * to specify the name would get the name.  In this case, the DHCID RR
 * will no longer match the updater's notion of the client-identity of
 * the host pointed to by the DNS name.
 *   -- "Interaction between DHCP and DNS"
 */

/* The first and second stages are pretty similar so we combine them */
void
client_dns_remove_action(dhcp_ddns_cb_t *ddns_cb,
			 isc_result_t    eresult)
{

	isc_result_t result;

	if ((eresult == ISC_R_SUCCESS) &&
	    (ddns_cb->state == DDNS_STATE_REM_FW_YXDHCID)) {
		/* Do the second stage of the FWD removal */
		ddns_cb->state = DDNS_STATE_REM_FW_NXRR;

		result = ddns_modify_fwd(ddns_cb, MDL);
		if (result == ISC_R_SUCCESS) {
			return;
		}
	}

	/* If we are done or have an error clean up */
	ddns_cb_free(ddns_cb, MDL);
	return;
}

void
client_dns_remove(struct client_state *client,
		  struct iaddr        *addr)
{
	dhcp_ddns_cb_t *ddns_cb;
	isc_result_t result;

	/* if we have an old ddns request for this client, cancel it */
	if (client->ddns_cb != NULL) {
		ddns_cancel(client->ddns_cb, MDL);
		client->ddns_cb = NULL;
	}
	
	ddns_cb = ddns_cb_alloc(MDL);
	if (ddns_cb != NULL) {
		ddns_cb->address = *addr;
		ddns_cb->timeout = 0;

		ddns_cb->state = DDNS_STATE_REM_FW_YXDHCID;
		ddns_cb->flags = DDNS_UPDATE_ADDR;
		ddns_cb->cur_func = client_dns_remove_action;

		result = client_dns_update(client, ddns_cb);

		if (result != ISC_R_TIMEDOUT) {
			ddns_cb_free(ddns_cb, MDL);
		}
	}
}
#endif

isc_result_t dhcp_set_control_state (control_object_state_t oldstate,
				     control_object_state_t newstate)
{
	struct interface_info *ip;
	struct client_state *client;
	struct timeval tv;

	/* Do the right thing for each interface. */
	for (ip = interfaces; ip; ip = ip -> next) {
	    for (client = ip -> client; client; client = client -> next) {
		switch (newstate) {
		  case server_startup:
		    return ISC_R_SUCCESS;

		  case server_running:
		    return ISC_R_SUCCESS;

		  case server_shutdown:
		    if (client -> active &&
			client -> active -> expiry > cur_time) {
#if defined (NSUPDATE)
			    if (client->config->do_forward_update) {
				    client_dns_remove(client,
						      &client->active->address);
			    }
#endif
			    do_release (client);
		    }
		    break;

		  case server_hibernate:
		    state_stop (client);
		    break;

		  case server_awaken:
		    state_reboot (client);
		    break;
		}
	    }
	}

	if (newstate == server_shutdown) {
		tv.tv_sec = cur_tv.tv_sec;
		tv.tv_usec = cur_tv.tv_usec + 1;
		add_timeout(&tv, shutdown_exit, 0, 0, 0);
	}
	return ISC_R_SUCCESS;
}

#if defined (NSUPDATE)
/*
 * Called after a timeout if the DNS update failed on the previous try.
 * Starts the retry process.  If the retry times out it will schedule
 * this routine to run again after a 10x wait.
 */
void
client_dns_update_timeout (void *cp)
{
	dhcp_ddns_cb_t *ddns_cb = (dhcp_ddns_cb_t *)cp;
	struct client_state *client = (struct client_state *)ddns_cb->lease;
	isc_result_t status = ISC_R_FAILURE;

	if ((client != NULL) &&
	    ((client->active != NULL) ||
	     (client->active_lease != NULL)))
		status = client_dns_update(client, ddns_cb);

	/*
	 * A status of timedout indicates that we started the update and
	 * have released control of the control block.  Any other status
	 * indicates that we should clean up the control block.  We either
	 * got a success which indicates that we didn't really need to
	 * send an update or some other error in which case we weren't able
	 * to start the update process.  In both cases we still own
	 * the control block and should free it.
	 */
	if (status != ISC_R_TIMEDOUT) {
		if (client != NULL) {
			client->ddns_cb = NULL;
		}
		ddns_cb_free(ddns_cb, MDL);
	}
}

/*
 * If the first query succeeds, the updater can conclude that it
 * has added a new name whose only RRs are the A and DHCID RR records.
 * The A RR update is now complete (and a client updater is finished,
 * while a server might proceed to perform a PTR RR update).
 *   -- "Interaction between DHCP and DNS"
 *
 * If the second query succeeds, the updater can conclude that the current
 * client was the last client associated with the domain name, and that
 * the name now contains the updated A RR. The A RR update is now
 * complete (and a client updater is finished, while a server would
 * then proceed to perform a PTR RR update).
 *   -- "Interaction between DHCP and DNS"
 *
 * If the second query fails with NXRRSET, the updater must conclude
 * that the client's desired name is in use by another host.  At this
 * juncture, the updater can decide (based on some administrative
 * configuration outside of the scope of this document) whether to let
 * the existing owner of the name keep that name, and to (possibly)
 * perform some name disambiguation operation on behalf of the current
 * client, or to replace the RRs on the name with RRs that represent
 * the current client. If the configured policy allows replacement of
 * existing records, the updater submits a query that deletes the
 * existing A RR and the existing DHCID RR, adding A and DHCID RRs that
 * represent the IP address and client-identity of the new client.
 *   -- "Interaction between DHCP and DNS"
 */

/* The first and second stages are pretty similar so we combine them */
void
client_dns_update_action(dhcp_ddns_cb_t *ddns_cb,
			 isc_result_t    eresult)
{
	isc_result_t result;
	struct timeval tv;

	switch(eresult) {
	case ISC_R_SUCCESS:
	default:
		/* Either we succeeded or broke in a bad way, clean up */
		break;

	case DNS_R_YXRRSET:
		/*
		 * This is the only difference between the two stages,
		 * check to see if it is the first stage, in which case
		 * start the second stage
		 */
		if (ddns_cb->state == DDNS_STATE_ADD_FW_NXDOMAIN) {
			ddns_cb->state = DDNS_STATE_ADD_FW_YXDHCID;
			ddns_cb->cur_func = client_dns_update_action;

			result = ddns_modify_fwd(ddns_cb, MDL);
			if (result == ISC_R_SUCCESS) {
				return;
			}
		}
		break;

	case ISC_R_TIMEDOUT:
		/*
		 * We got a timeout response from the DNS module.  Schedule
		 * another attempt for later.  We forget the name, dhcid and
		 * zone so if it gets changed we will get the new information.
		 */
		data_string_forget(&ddns_cb->fwd_name, MDL);
		data_string_forget(&ddns_cb->dhcid, MDL);
		if (ddns_cb->zone != NULL) {
			forget_zone((struct dns_zone **)&ddns_cb->zone);
		}

		/* Reset to doing the first stage */
		ddns_cb->state    = DDNS_STATE_ADD_FW_NXDOMAIN;
		ddns_cb->cur_func = client_dns_update_action;

		/* and update our timer */
		if (ddns_cb->timeout < 3600)
			ddns_cb->timeout *= 10;
		tv.tv_sec = cur_tv.tv_sec + ddns_cb->timeout;
		tv.tv_usec = cur_tv.tv_usec;
		add_timeout(&tv, client_dns_update_timeout,
			    ddns_cb, NULL, NULL);
		return;
	}

	ddns_cb_free(ddns_cb, MDL);
	return;
}

/* See if we should do a DNS update, and if so, do it. */

isc_result_t
client_dns_update(struct client_state *client, dhcp_ddns_cb_t *ddns_cb)
{
	struct data_string client_identifier;
	struct option_cache *oc;
	int ignorep;
	int result;
	isc_result_t rcode;

	/* If we didn't send an FQDN option, we certainly aren't going to
	   be doing an update. */
	if (!client -> sent_options)
		return ISC_R_SUCCESS;

	/* If we don't have a lease, we can't do an update. */
	if ((client->active == NULL) && (client->active_lease == NULL))
		return ISC_R_SUCCESS;

	/* If we set the no client update flag, don't do the update. */
	if ((oc = lookup_option (&fqdn_universe, client -> sent_options,
				  FQDN_NO_CLIENT_UPDATE)) &&
	    evaluate_boolean_option_cache (&ignorep, (struct packet *)0,
					   (struct lease *)0, client,
					   client -> sent_options,
					   (struct option_state *)0,
					   &global_scope, oc, MDL))
		return ISC_R_SUCCESS;

	/* If we set the "server, please update" flag, or didn't set it
	   to false, don't do the update. */
	if (!(oc = lookup_option (&fqdn_universe, client -> sent_options,
				  FQDN_SERVER_UPDATE)) ||
	    evaluate_boolean_option_cache (&ignorep, (struct packet *)0,
					   (struct lease *)0, client,
					   client -> sent_options,
					   (struct option_state *)0,
					   &global_scope, oc, MDL))
		return ISC_R_SUCCESS;

	/* If no FQDN option was supplied, don't do the update. */
	if (!(oc = lookup_option (&fqdn_universe, client -> sent_options,
				  FQDN_FQDN)) ||
	    !evaluate_option_cache (&ddns_cb->fwd_name, (struct packet *)0,
				    (struct lease *)0, client,
				    client -> sent_options,
				    (struct option_state *)0,
				    &global_scope, oc, MDL))
		return ISC_R_SUCCESS;

	/* If this is a DHCPv6 client update, make a dhcid string out of
	 * the DUID.  If this is a DHCPv4 client update, choose either
	 * the client identifier, if there is one, or the interface's
	 * MAC address.
	 */
	result = 0;
	memset(&client_identifier, 0, sizeof(client_identifier));
	if (client->active_lease != NULL) {
		if (((oc =
		      lookup_option(&dhcpv6_universe, client->sent_options,
				    D6O_CLIENTID)) != NULL) &&
		    evaluate_option_cache(&client_identifier, NULL, NULL,
					  client, client->sent_options, NULL,
					  &global_scope, oc, MDL)) {
			/* RFC4701 defines type '2' as being for the DUID
			 * field.  We aren't using RFC4701 DHCID RR's yet,
			 * but this is as good a value as any.
			 */
			result = get_dhcid(&ddns_cb->dhcid, 2,
					   client_identifier.data,
					   client_identifier.len);
			data_string_forget(&client_identifier, MDL);
		} else
			log_fatal("Impossible condition at %s:%d.", MDL);
	} else {
		if (((oc =
		      lookup_option(&dhcp_universe, client->sent_options,
				    DHO_DHCP_CLIENT_IDENTIFIER)) != NULL) &&
		    evaluate_option_cache(&client_identifier, NULL, NULL,
					  client, client->sent_options, NULL,
					  &global_scope, oc, MDL)) {
			result = get_dhcid(&ddns_cb->dhcid,
					   DHO_DHCP_CLIENT_IDENTIFIER,
					   client_identifier.data,
					   client_identifier.len);
			data_string_forget(&client_identifier, MDL);
		} else
			result = get_dhcid(&ddns_cb->dhcid, 0,
					   client->interface->hw_address.hbuf,
					   client->interface->hw_address.hlen);
	}
	if (!result) {
		return ISC_R_SUCCESS;
	}

	/*
	 * Perform updates.
	 */
	if (ddns_cb->fwd_name.len && ddns_cb->dhcid.len) {
		rcode = ddns_modify_fwd(ddns_cb, MDL);
	} else
		rcode = ISC_R_FAILURE;

	/*
	 * A success from the modify routine means we are performing
	 * async processing, for which we use the timedout error message.
	 */
	if (rcode == ISC_R_SUCCESS) {
		rcode = ISC_R_TIMEDOUT;
	}

	return rcode;
}


/*
 * Schedule the first update.  They will continue to retry occasionally
 * until they no longer time out (or fail).
 */
void
dhclient_schedule_updates(struct client_state *client,
			  struct iaddr        *addr,
			  int                  offset)
{
	dhcp_ddns_cb_t *ddns_cb;
	struct timeval tv;

	if (!client->config->do_forward_update)
		return;

	/* cancel any outstanding ddns requests */
	if (client->ddns_cb != NULL) {
		ddns_cancel(client->ddns_cb, MDL);
		client->ddns_cb = NULL;
	}

	ddns_cb = ddns_cb_alloc(MDL);

	if (ddns_cb != NULL) {
		ddns_cb->lease = (void *)client;
		ddns_cb->address = *addr;
		ddns_cb->timeout = 1;

		/*
		 * XXX: DNS TTL is a problem we need to solve properly.
		 * Until that time, 300 is a placeholder default for
		 * something that is less insane than a value scaled
		 * by lease timeout.
		 */
		ddns_cb->ttl = 300;

		ddns_cb->state = DDNS_STATE_ADD_FW_NXDOMAIN;
		ddns_cb->cur_func = client_dns_update_action;
		ddns_cb->flags = DDNS_UPDATE_ADDR | DDNS_INCLUDE_RRSET;

		client->ddns_cb = ddns_cb;

		tv.tv_sec = cur_tv.tv_sec + offset;
		tv.tv_usec = cur_tv.tv_usec;
		add_timeout(&tv, client_dns_update_timeout,
			    ddns_cb, NULL, NULL);
	} else {
		log_error("Unable to allocate dns update state for %s",
			  piaddr(*addr));
	}
}
#endif

void
dhcpv4_client_assignments(void)
{
//ScenSim-Port//	struct servent *ent;

//ScenSim-Port//	if (path_dhclient_pid == NULL)
//ScenSim-Port//		path_dhclient_pid = _PATH_DHCLIENT_PID;
//ScenSim-Port//	if (path_dhclient_db == NULL)
//ScenSim-Port//		path_dhclient_db = _PATH_DHCLIENT_DB;

	/* Default to the DHCP/BOOTP port. */
	if (!local_port) {
		/* If we're faking a relay agent, and we're not using loopback,
		   use the server port, not the client port. */
//ScenSim-Port//		if (mockup_relay && giaddr.s_addr != htonl (INADDR_LOOPBACK)) {
//ScenSim-Port//			local_port = htons(67);
//ScenSim-Port//		} else {
//ScenSim-Port//			ent = getservbyname ("dhcpc", "udp");
//ScenSim-Port//			if (!ent)
				local_port = htons (68);
//ScenSim-Port//			else
//ScenSim-Port//				local_port = ent -> s_port;
//ScenSim-Port//#ifndef __CYGWIN32__
//ScenSim-Port//			endservent ();
//ScenSim-Port//#endif
//ScenSim-Port//		}
	}

	/* If we're faking a relay agent, and we're not using loopback,
	   we're using the server port, not the client port. */
//ScenSim-Port//	if (mockup_relay && giaddr.s_addr != htonl (INADDR_LOOPBACK)) {
//ScenSim-Port//		remote_port = local_port;
//ScenSim-Port//	} else
		remote_port = htons (ntohs (local_port) - 1);   /* XXX */
}

/*
 * The following routines are used to check that certain
 * strings are reasonable before we pass them to the scripts.
 * This avoids some problems with scripts treating the strings
 * as commands - see ticket 23722
 * The domain checking code should be done as part of assembling
 * the string but we are doing it here for now due to time
 * constraints.
 */

static int check_domain_name(const char *ptr, size_t len, int dots)
{
	const char *p;

	/* not empty or complete length not over 255 characters   */
	if ((len == 0) || (len > 256))
		return(-1);

	/* consists of [[:alnum:]-]+ labels separated by [.]      */
	/* a [_] is against RFC but seems to be "widely used"...  */
	for (p=ptr; (*p != 0) && (len-- > 0); p++) {
		if ((*p == '-') || (*p == '_')) {
			/* not allowed at begin or end of a label */
			if (((p - ptr) == 0) || (len == 0) || (p[1] == '.'))
				return(-1);
		} else if (*p == '.') {
			/* each label has to be 1-63 characters;
			   we allow [.] at the end ('foo.bar.')   */
			size_t d = p - ptr;
			if ((d <= 0) || (d >= 64))
				return(-1);
			ptr = p + 1; /* jump to the next label    */
			if ((dots > 0) && (len > 0))
				dots--;
		} else if (isalnum((unsigned char)*p) == 0) {
			/* also numbers at the begin are fine     */
			return(-1);
		}
	}
	return(dots ? -1 : 0);
}

static int check_domain_name_list(const char *ptr, size_t len, int dots)
{
	const char *p;
	int ret = -1; /* at least one needed */

	if ((ptr == NULL) || (len == 0))
		return(-1);

	for (p=ptr; (*p != 0) && (len > 0); p++, len--) {
		if (*p != ' ')
			continue;
		if (p > ptr) {
			if (check_domain_name(ptr, p - ptr, dots) != 0)
				return(-1);
			ret = 0;
		}
		ptr = p + 1;
	}
	if (p > ptr)
		return(check_domain_name(ptr, p - ptr, dots));
	else
		return(ret);
}

static int check_option_values(struct universe *universe,
			       unsigned int opt,
			       const char *ptr,
			       size_t len)
{
	if (ptr == NULL)
		return(-1);

	/* just reject options we want to protect, will be escaped anyway */
	if ((universe == NULL) || (universe == &dhcp_universe)) {
		switch(opt) {
		      case DHO_DOMAIN_NAME:
#ifdef ACCEPT_LIST_IN_DOMAIN_NAME
			      return check_domain_name_list(ptr, len, 0);
#else
			      return check_domain_name(ptr, len, 0);
#endif
		      case DHO_HOST_NAME:
		      case DHO_NIS_DOMAIN:
		      case DHO_NETBIOS_SCOPE:
			return check_domain_name(ptr, len, 0);
			break;
		      case DHO_DOMAIN_SEARCH:
			return check_domain_name_list(ptr, len, 0);
			break;
		      case DHO_ROOT_PATH:
			if (len == 0)
				return(-1);
			for (; (*ptr != 0) && (len-- > 0); ptr++) {
				if(!(isalnum((unsigned char)*ptr) ||
				     *ptr == '#'  || *ptr == '%' ||
				     *ptr == '+'  || *ptr == '-' ||
				     *ptr == '_'  || *ptr == ':' ||
				     *ptr == '.'  || *ptr == ',' ||
				     *ptr == '@'  || *ptr == '~' ||
				     *ptr == '\\' || *ptr == '/' ||
				     *ptr == '['  || *ptr == ']' ||
				     *ptr == '='  || *ptr == ' '))
					return(-1);
			}
			return(0);
			break;
		}
	}

#ifdef DHCPv6
	if (universe == &dhcpv6_universe) {
		switch(opt) {
		      case D6O_SIP_SERVERS_DNS:
		      case D6O_DOMAIN_SEARCH:
		      case D6O_NIS_DOMAIN_NAME:
		      case D6O_NISP_DOMAIN_NAME:
			return check_domain_name_list(ptr, len, 0);
			break;
		}
	}
#endif

	return(0);
}

static void
add_reject(struct packet *packet) {
	struct iaddrmatchlist *list;
	
//ScenSim-Port//	list = dmalloc(sizeof(struct iaddrmatchlist), MDL);
	list = (iaddrmatchlist*)dmalloc(sizeof(struct iaddrmatchlist), MDL);//ScenSim-Port//
	if (!list)
		log_fatal ("no memory for reject list!");

	/*
	 * client_addr is misleading - it is set to source address in common
	 * code.
	 */
	list->match.addr = packet->client_addr;
	/* Set mask to indicate host address. */
	list->match.mask.len = list->match.addr.len;
	memset(list->match.mask.iabuf, 0xff, sizeof(list->match.mask.iabuf));

	/* Append to reject list for the source interface. */
	list->next = packet->interface->client->config->reject_list;
	packet->interface->client->config->reject_list = list;

	/*
	 * We should inform user that we won't be accepting this server
	 * anymore.
	 */
	log_info("Server added to list of rejected servers.");
}
}//namespace IscDhcpPort////ScenSim-Port//
