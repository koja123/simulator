/* dhcpd.c

   DHCP Server Daemon. */

/*
 * Copyright (c) 2004-2012 by Internet Systems Consortium, Inc. ("ISC")
 * Copyright (c) 1996-2003 by Internet Software Consortium
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
 * This software has been written for Internet Systems Consortium
 * by Ted Lemon in cooperation with Vixie Enterprises and Nominum, Inc.
 * To learn more about Internet Systems Consortium, see
 * ``https://www.isc.org/''.  To learn more about Vixie Enterprises,
 * see ``http://www.vix.com''.   To learn more about Nominum, Inc., see
 * ``http://www.nominum.com''.
 */
#include <iscdhcp_porting.h>//ScenSim-Port//
namespace IscDhcpPort {//ScenSim-Port//

//ScenSim-Port//static const char copyright[] =
//ScenSim-Port//"Copyright 2004-2012 Internet Systems Consortium.";
//ScenSim-Port//static const char arr [] = "All rights reserved.";
//ScenSim-Port//static const char message [] = "Internet Systems Consortium DHCP Server";
//ScenSim-Port//static const char url [] =
//ScenSim-Port//"For info, please visit https://www.isc.org/software/dhcp/";

//ScenSim-Port//#include "dhcpd.h"
//ScenSim-Port//#include <omapip/omapip_p.h>
//ScenSim-Port//#include <syslog.h>
//ScenSim-Port//#include <errno.h>
//ScenSim-Port//#include <limits.h>
//ScenSim-Port//#include <sys/types.h>
//ScenSim-Port//#include <sys/time.h>
//ScenSim-Port//#include <signal.h>

//ScenSim-Port//#if defined (PARANOIA)
//ScenSim-Port//#  include <sys/types.h>
//ScenSim-Port//#  include <unistd.h>
//ScenSim-Port//#  include <pwd.h>
//ScenSim-Port///* get around the ISC declaration of group */
//ScenSim-Port//#  define group real_group 
//ScenSim-Port//#    include <grp.h>
//ScenSim-Port//#  undef group
//ScenSim-Port//#endif /* PARANOIA */

//ScenSim-Port//static void usage(void);

//ScenSim-Port//struct iaddr server_identifier;
//ScenSim-Port//int server_identifier_matched;

//ScenSim-Port//#if defined (NSUPDATE)
//ScenSim-Port//
//ScenSim-Port///* This stuff is always executed to figure the default values for certain
//ScenSim-Port//   ddns variables. */
//ScenSim-Port//
//ScenSim-Port//char std_nsupdate [] = "						    \n\
//ScenSim-Port//option server.ddns-hostname =						    \n\
//ScenSim-Port//  pick (option fqdn.hostname, option host-name);			    \n\
//ScenSim-Port//option server.ddns-domainname =	config-option domain-name;		    \n\
//ScenSim-Port//option server.ddns-rev-domainname = \"in-addr.arpa.\";";
//ScenSim-Port//
//ScenSim-Port///* This is the old-style name service updater that is executed
//ScenSim-Port//   whenever a lease is committed.  It does not follow the DHCP-DNS
//ScenSim-Port//   draft at all. */
//ScenSim-Port//
//ScenSim-Port//char old_nsupdate [] = "						    \n\
//ScenSim-Port//on commit {								    \n\
//ScenSim-Port//  if (not static and							    \n\
//ScenSim-Port//      ((config-option server.ddns-updates = null) or			    \n\
//ScenSim-Port//       (config-option server.ddns-updates != 0))) {			    \n\
//ScenSim-Port//    set new-ddns-fwd-name =						    \n\
//ScenSim-Port//      concat (pick (config-option server.ddns-hostname,			    \n\
//ScenSim-Port//		    option host-name), \".\",				    \n\
//ScenSim-Port//	      pick (config-option server.ddns-domainname,		    \n\
//ScenSim-Port//		    config-option domain-name));			    \n\
//ScenSim-Port//    if (defined (ddns-fwd-name) and ddns-fwd-name != new-ddns-fwd-name) {   \n\
//ScenSim-Port//      switch (ns-update (delete (IN, A, ddns-fwd-name, leased-address))) {  \n\
//ScenSim-Port//      case NOERROR:							    \n\
//ScenSim-Port//	unset ddns-fwd-name;						    \n\
//ScenSim-Port//	on expiry or release {						    \n\
//ScenSim-Port//	}								    \n\
//ScenSim-Port//      }									    \n\
//ScenSim-Port//    }									    \n\
//ScenSim-Port//									    \n\
//ScenSim-Port//    if (not defined (ddns-fwd-name)) {					    \n\
//ScenSim-Port//      set ddns-fwd-name = new-ddns-fwd-name;				    \n\
//ScenSim-Port//      if defined (ddns-fwd-name) {					    \n\
//ScenSim-Port//	switch (ns-update (not exists (IN, A, ddns-fwd-name, null),	    \n\
//ScenSim-Port//			   add (IN, A, ddns-fwd-name, leased-address,	    \n\
//ScenSim-Port//				lease-time / 2))) {			    \n\
//ScenSim-Port//	default:							    \n\
//ScenSim-Port//	  unset ddns-fwd-name;						    \n\
//ScenSim-Port//	  break;							    \n\
//ScenSim-Port//									    \n\
//ScenSim-Port//	case NOERROR:							    \n\
//ScenSim-Port//	  set ddns-rev-name =						    \n\
//ScenSim-Port//	    concat (binary-to-ascii (10, 8, \".\",			    \n\
//ScenSim-Port//				     reverse (1,			    \n\
//ScenSim-Port//					      leased-address)), \".\",	    \n\
//ScenSim-Port//		    pick (config-option server.ddns-rev-domainname,	    \n\
//ScenSim-Port//			  \"in-addr.arpa.\"));				    \n\
//ScenSim-Port//	  switch (ns-update (delete (IN, PTR, ddns-rev-name, null),	    \n\
//ScenSim-Port//			     add (IN, PTR, ddns-rev-name, ddns-fwd-name,    \n\
//ScenSim-Port//				  lease-time / 2)))			    \n\
//ScenSim-Port//	    {								    \n\
//ScenSim-Port//	    default:							    \n\
//ScenSim-Port//	      unset ddns-rev-name;					    \n\
//ScenSim-Port//	      on release or expiry {					    \n\
//ScenSim-Port//		switch (ns-update (delete (IN, A, ddns-fwd-name,	    \n\
//ScenSim-Port//					   leased-address))) {		    \n\
//ScenSim-Port//		case NOERROR:						    \n\
//ScenSim-Port//		  unset ddns-fwd-name;					    \n\
//ScenSim-Port//		  break;						    \n\
//ScenSim-Port//		}							    \n\
//ScenSim-Port//		on release or expiry;					    \n\
//ScenSim-Port//	      }								    \n\
//ScenSim-Port//	      break;							    \n\
//ScenSim-Port//									    \n\
//ScenSim-Port//	    case NOERROR:						    \n\
//ScenSim-Port//	      on release or expiry {					    \n\
//ScenSim-Port//		switch (ns-update (delete (IN, PTR, ddns-rev-name, null))) {\n\
//ScenSim-Port//		case NOERROR:						    \n\
//ScenSim-Port//		  unset ddns-rev-name;					    \n\
//ScenSim-Port//		  break;						    \n\
//ScenSim-Port//		}							    \n\
//ScenSim-Port//		switch (ns-update (delete (IN, A, ddns-fwd-name,	    \n\
//ScenSim-Port//					   leased-address))) {		    \n\
//ScenSim-Port//		case NOERROR:						    \n\
//ScenSim-Port//		  unset ddns-fwd-name;					    \n\
//ScenSim-Port//		  break;						    \n\
//ScenSim-Port//		}							    \n\
//ScenSim-Port//		on release or expiry;					    \n\
//ScenSim-Port//	      }								    \n\
//ScenSim-Port//	    }								    \n\
//ScenSim-Port//	}								    \n\
//ScenSim-Port//      }									    \n\
//ScenSim-Port//    }									    \n\
//ScenSim-Port//    unset new-ddns-fwd-name;						    \n\
//ScenSim-Port//  }									    \n\
//ScenSim-Port//}";
//ScenSim-Port//
//ScenSim-Port//#endif /* NSUPDATE */
//ScenSim-Port//int ddns_update_style;

//ScenSim-Port//const char *path_dhcpd_conf = _PATH_DHCPD_CONF;
//ScenSim-Port//const char *path_dhcpd_db = _PATH_DHCPD_DB;
//ScenSim-Port//const char *path_dhcpd_pid = _PATH_DHCPD_PID;
/* False (default) => we write and use a pid file */
//ScenSim-Port//isc_boolean_t no_pid_file = ISC_FALSE;

//ScenSim-Port//int dhcp_max_agent_option_packet_length = DHCP_MTU_MAX;

//ScenSim-Port//static omapi_auth_key_t *omapi_key = (omapi_auth_key_t *)0;
//ScenSim-Port//int omapi_port;

//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//trace_type_t *trace_srandom;
//ScenSim-Port//#endif

//ScenSim-Port//static isc_result_t verify_addr (omapi_object_t *l, omapi_addr_t *addr) {
//ScenSim-Port//	return ISC_R_SUCCESS;
//ScenSim-Port//}

//ScenSim-Port//static isc_result_t verify_auth (omapi_object_t *p, omapi_auth_key_t *a) {
//ScenSim-Port//	if (a != omapi_key)
//ScenSim-Port//		return DHCP_R_INVALIDKEY;
//ScenSim-Port//	return ISC_R_SUCCESS;
//ScenSim-Port//}

//ScenSim-Port//static void omapi_listener_start (void *foo)
//ScenSim-Port//{
//ScenSim-Port//	omapi_object_t *listener;
//ScenSim-Port//	isc_result_t result;
//ScenSim-Port//	struct timeval tv;
//ScenSim-Port//
//ScenSim-Port//	listener = (omapi_object_t *)0;
//ScenSim-Port//	result = omapi_generic_new (&listener, MDL);
//ScenSim-Port//	if (result != ISC_R_SUCCESS)
//ScenSim-Port//		log_fatal ("Can't allocate new generic object: %s",
//ScenSim-Port//			   isc_result_totext (result));
//ScenSim-Port//	result = omapi_protocol_listen (listener,
//ScenSim-Port//					(unsigned)omapi_port, 1);
//ScenSim-Port//	if (result == ISC_R_SUCCESS && omapi_key)
//ScenSim-Port//		result = omapi_protocol_configure_security
//ScenSim-Port//			(listener, verify_addr, verify_auth);
//ScenSim-Port//	if (result != ISC_R_SUCCESS) {
//ScenSim-Port//		log_error ("Can't start OMAPI protocol: %s",
//ScenSim-Port//			   isc_result_totext (result));
//ScenSim-Port//		tv.tv_sec = cur_tv.tv_sec + 5;
//ScenSim-Port//		tv.tv_usec = cur_tv.tv_usec;
//ScenSim-Port//		add_timeout (&tv, omapi_listener_start, 0, 0, 0);
//ScenSim-Port//	}
//ScenSim-Port//	omapi_object_dereference (&listener, MDL);
//ScenSim-Port//}

//ScenSim-Port//#if defined (PARANOIA)
//ScenSim-Port///* to be used in one of two possible scenarios */
//ScenSim-Port//static void setup_chroot (char *chroot_dir) {
//ScenSim-Port//  if (geteuid())
//ScenSim-Port//    log_fatal ("you must be root to use chroot");
//ScenSim-Port//
//ScenSim-Port//  if (chroot(chroot_dir)) {
//ScenSim-Port//    log_fatal ("chroot(\"%s\"): %m", chroot_dir);
//ScenSim-Port//  }
//ScenSim-Port//  if (chdir ("/")) {
//ScenSim-Port//    /* probably permission denied */
//ScenSim-Port//    log_fatal ("chdir(\"/\"): %m");
//ScenSim-Port//  }
//ScenSim-Port//}
//ScenSim-Port//#endif /* PARANOIA */

//ScenSim-Port//#ifndef UNIT_TEST
int 
//ScenSim-Port//main(int argc, char **argv) {
start_server() {//ScenSim-Port//
    curctx->bootp = bootp;//ScenSim-Port//
    curctx->find_class = find_class;//ScenSim-Port//
    curctx->classify = classify;//ScenSim-Port//
    curctx->check_collection = check_collection;//ScenSim-Port//
    curctx->parse_allow_deny = parse_allow_deny;//ScenSim-Port//
    curctx->dhcp = dhcp;//ScenSim-Port//
    curctx->dhcpv6 = dhcpv6;//ScenSim-Port//
//ScenSim-Port//	int fd;
	int i, status;
	struct servent *ent;
	char *s;
	int cftest = 0;
	int lftest = 0;
#ifndef DEBUG
	int pid;
	char pbuf [20];
	int daemon = 1;
#endif
	int quiet = 0;
	char *server = (char *)0;
	isc_result_t result;
	unsigned seed;
	struct interface_info *ip;
//ScenSim-Port//#if defined (NSUPDATE)
//ScenSim-Port//	struct parse *parse;
//ScenSim-Port//	int lose;
//ScenSim-Port//#endif
	int no_dhcpd_conf = 0;
	int no_dhcpd_db = 0;
	int no_dhcpd_pid = 0;
#ifdef DHCPv6
	int local_family_set = 0;
#endif /* DHCPv6 */
//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//	char *traceinfile = (char *)0;
//ScenSim-Port//	char *traceoutfile = (char *)0;
//ScenSim-Port//#endif

//ScenSim-Port//#if defined (PARANOIA)
//ScenSim-Port//	char *set_user   = 0;
//ScenSim-Port//	char *set_group  = 0;
//ScenSim-Port//	char *set_chroot = 0;
//ScenSim-Port//
//ScenSim-Port//	uid_t set_uid = 0;
//ScenSim-Port//	gid_t set_gid = 0;
//ScenSim-Port//#endif /* PARANOIA */

        /* Make sure that file descriptors 0 (stdin), 1, (stdout), and
           2 (stderr) are open. To do this, we assume that when we
           open a file the lowest available file descriptor is used. */
//ScenSim-Port//        fd = open("/dev/null", O_RDWR);
//ScenSim-Port//        if (fd == 0)
//ScenSim-Port//                fd = open("/dev/null", O_RDWR);
//ScenSim-Port//        if (fd == 1)
//ScenSim-Port//                fd = open("/dev/null", O_RDWR);
//ScenSim-Port//        if (fd == 2)
//ScenSim-Port//                log_perror = 0; /* No sense logging to /dev/null. */
//ScenSim-Port//        else if (fd != -1)
//ScenSim-Port//                close(fd);

	/* Set up the isc and dns library managers */
//ScenSim-Port//	status = dhcp_context_create();
//ScenSim-Port//	if (status != ISC_R_SUCCESS)
//ScenSim-Port//		log_fatal("Can't initialize context: %s",
//ScenSim-Port//			  isc_result_totext(status));

	/* Set up the client classification system. */
	classification_setup ();

	/* Initialize the omapi system. */
//ScenSim-Port//	result = omapi_init ();
//ScenSim-Port//	if (result != ISC_R_SUCCESS)
//ScenSim-Port//		log_fatal ("Can't initialize OMAPI: %s",
//ScenSim-Port//			   isc_result_totext (result));

	/* Set up the OMAPI wrappers for common objects. */
	dhcp_db_objects_setup ();
	/* Set up the OMAPI wrappers for various server database internal
	   objects. */
	dhcp_common_objects_setup ();

	/* Initially, log errors to stderr as well as to syslogd. */
//ScenSim-Port//	openlog ("dhcpd", LOG_NDELAY, DHCPD_LOG_FACILITY);

//ScenSim-Port//	for (i = 1; i < argc; i++) {
//ScenSim-Port//		if (!strcmp (argv [i], "-p")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage ();
//ScenSim-Port//			local_port = validate_port (argv [i]);
//ScenSim-Port//			log_debug ("binding to user-specified port %d",
//ScenSim-Port//			       ntohs (local_port));
//ScenSim-Port//		} else if (!strcmp (argv [i], "-f")) {
//ScenSim-Port//#ifndef DEBUG
//ScenSim-Port//			daemon = 0;
//ScenSim-Port//#endif
//ScenSim-Port//		} else if (!strcmp (argv [i], "-d")) {
//ScenSim-Port//#ifndef DEBUG
//ScenSim-Port//			daemon = 0;
//ScenSim-Port//#endif
//ScenSim-Port//			log_perror = -1;
//ScenSim-Port//		} else if (!strcmp (argv [i], "-s")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage ();
//ScenSim-Port//			server = argv [i];
//ScenSim-Port//#if defined (PARANOIA)
//ScenSim-Port//		} else if (!strcmp (argv [i], "-user")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage ();
//ScenSim-Port//			set_user = argv [i];
//ScenSim-Port//		} else if (!strcmp (argv [i], "-group")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage ();
//ScenSim-Port//			set_group = argv [i];
//ScenSim-Port//		} else if (!strcmp (argv [i], "-chroot")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage ();
//ScenSim-Port//			set_chroot = argv [i];
//ScenSim-Port//#endif /* PARANOIA */
//ScenSim-Port//		} else if (!strcmp (argv [i], "-cf")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage ();
//ScenSim-Port//			path_dhcpd_conf = argv [i];
//ScenSim-Port//			no_dhcpd_conf = 1;
//ScenSim-Port//		} else if (!strcmp (argv [i], "-lf")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage ();
//ScenSim-Port//			path_dhcpd_db = argv [i];
//ScenSim-Port//			no_dhcpd_db = 1;
//ScenSim-Port//		} else if (!strcmp (argv [i], "-pf")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage ();
//ScenSim-Port//			path_dhcpd_pid = argv [i];
//ScenSim-Port//			no_dhcpd_pid = 1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "--no-pid")) {
//ScenSim-Port//			no_pid_file = ISC_TRUE;
//ScenSim-Port//                } else if (!strcmp (argv [i], "-t")) {
//ScenSim-Port//			/* test configurations only */
//ScenSim-Port//#ifndef DEBUG
//ScenSim-Port//			daemon = 0;
//ScenSim-Port//#endif
//ScenSim-Port//			cftest = 1;
//ScenSim-Port//			log_perror = -1;
//ScenSim-Port//                } else if (!strcmp (argv [i], "-T")) {
//ScenSim-Port//			/* test configurations and lease file only */
//ScenSim-Port//#ifndef DEBUG
//ScenSim-Port//			daemon = 0;
//ScenSim-Port//#endif
//ScenSim-Port//			cftest = 1;
//ScenSim-Port//			lftest = 1;
//ScenSim-Port//			log_perror = -1;
//ScenSim-Port//		} else if (!strcmp (argv [i], "-q")) {
//ScenSim-Port//			quiet = 1;
//ScenSim-Port//			quiet_interface_discovery = 1;
//ScenSim-Port//#ifdef DHCPv6
//ScenSim-Port//		} else if (!strcmp(argv[i], "-4")) {
//ScenSim-Port//			if (local_family_set && (local_family != AF_INET)) {
//ScenSim-Port//				log_fatal("Server cannot run in both IPv4 and "
//ScenSim-Port//					  "IPv6 mode at the same time.");
//ScenSim-Port//			}
//ScenSim-Port//			local_family = AF_INET;
//ScenSim-Port//			local_family_set = 1;
//ScenSim-Port//		} else if (!strcmp(argv[i], "-6")) {
//ScenSim-Port//			if (local_family_set && (local_family != AF_INET6)) {
//ScenSim-Port//				log_fatal("Server cannot run in both IPv4 and "
//ScenSim-Port//					  "IPv6 mode at the same time.");
//ScenSim-Port//			}
//ScenSim-Port//			local_family = AF_INET6;
//ScenSim-Port//			local_family_set = 1;
//ScenSim-Port//#endif /* DHCPv6 */
//ScenSim-Port//		} else if (!strcmp (argv [i], "--version")) {
//ScenSim-Port//			log_info("isc-dhcpd-%s", PACKAGE_VERSION);
//ScenSim-Port//			exit (0);
//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//		} else if (!strcmp (argv [i], "-tf")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage ();
//ScenSim-Port//			traceoutfile = argv [i];
//ScenSim-Port//		} else if (!strcmp (argv [i], "-play")) {
//ScenSim-Port//			if (++i == argc)
//ScenSim-Port//				usage ();
//ScenSim-Port//			traceinfile = argv [i];
//ScenSim-Port//			trace_replay_init ();
//ScenSim-Port//#endif /* TRACING */
//ScenSim-Port//		} else if (argv [i][0] == '-') {
//ScenSim-Port//			usage ();
//ScenSim-Port//		} else {
//ScenSim-Port//			struct interface_info *tmp =
//ScenSim-Port//				(struct interface_info *)0;
//ScenSim-Port//			if (strlen(argv[i]) >= sizeof(tmp->name))
//ScenSim-Port//				log_fatal("%s: interface name too long "
//ScenSim-Port//					  "(is %ld)",
//ScenSim-Port//					  argv[i], (long)strlen(argv[i]));
//ScenSim-Port//			result = interface_allocate (&tmp, MDL);
//ScenSim-Port//			if (result != ISC_R_SUCCESS)
//ScenSim-Port//				log_fatal ("Insufficient memory to %s %s: %s",
//ScenSim-Port//					   "record interface", argv [i],
//ScenSim-Port//					   isc_result_totext (result));
//ScenSim-Port//			strcpy (tmp -> name, argv [i]);
//ScenSim-Port//			if (interfaces) {
//ScenSim-Port//				interface_reference (&tmp -> next,
//ScenSim-Port//						     interfaces, MDL);
//ScenSim-Port//				interface_dereference (&interfaces, MDL);
//ScenSim-Port//			}
//ScenSim-Port//			interface_reference (&interfaces, tmp, MDL);
//ScenSim-Port//			tmp -> flags = INTERFACE_REQUESTED;
//ScenSim-Port//		}
//ScenSim-Port//	}

//ScenSim-Port//	if (!no_dhcpd_conf && (s = getenv ("PATH_DHCPD_CONF"))) {
//ScenSim-Port//		path_dhcpd_conf = s;
//ScenSim-Port//	}

//ScenSim-Port//#ifdef DHCPv6
//ScenSim-Port//        if (local_family == AF_INET6) {
//ScenSim-Port//                /* DHCPv6: override DHCPv4 lease and pid filenames */
//ScenSim-Port//	        if (!no_dhcpd_db) {
//ScenSim-Port//                        if ((s = getenv ("PATH_DHCPD6_DB")))
//ScenSim-Port//		                path_dhcpd_db = s;
//ScenSim-Port//                        else
//ScenSim-Port//		                path_dhcpd_db = _PATH_DHCPD6_DB;
//ScenSim-Port//	        }
//ScenSim-Port//	        if (!no_dhcpd_pid) {
//ScenSim-Port//                        if ((s = getenv ("PATH_DHCPD6_PID")))
//ScenSim-Port//		                path_dhcpd_pid = s;
//ScenSim-Port//                        else
//ScenSim-Port//		                path_dhcpd_pid = _PATH_DHCPD6_PID;
//ScenSim-Port//	        }
//ScenSim-Port//        } else
//ScenSim-Port//#else /* !DHCPv6 */
//ScenSim-Port//        {
//ScenSim-Port//	        if (!no_dhcpd_db && (s = getenv ("PATH_DHCPD_DB"))) {
//ScenSim-Port//		        path_dhcpd_db = s;
//ScenSim-Port//	        }
//ScenSim-Port//	        if (!no_dhcpd_pid && (s = getenv ("PATH_DHCPD_PID"))) {
//ScenSim-Port//		        path_dhcpd_pid = s;
//ScenSim-Port//	        }
//ScenSim-Port//        }
//ScenSim-Port//#endif /* DHCPv6 */

        /*
         * convert relative path names to absolute, for files that need
         * to be reopened after chdir() has been called
         */
//ScenSim-Port//        if (path_dhcpd_db[0] != '/') {
//ScenSim-Port//                char *path = dmalloc(PATH_MAX, MDL);
//ScenSim-Port//                if (path == NULL)
//ScenSim-Port//                        log_fatal("No memory for filename\n");
//ScenSim-Port//                path_dhcpd_db = realpath(path_dhcpd_db,  path);
//ScenSim-Port//                if (path_dhcpd_db == NULL)
//ScenSim-Port//                        log_fatal("%s: %s", path, strerror(errno));
//ScenSim-Port//        }

//ScenSim-Port//	if (!quiet) {
//ScenSim-Port//		log_info("%s %s", message, PACKAGE_VERSION);
//ScenSim-Port//		log_info (copyright);
//ScenSim-Port//		log_info (arr);
//ScenSim-Port//		log_info (url);
//ScenSim-Port//	} else {
//ScenSim-Port//		quiet = 0;
//ScenSim-Port//		log_perror = 0;
//ScenSim-Port//	}

//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//	trace_init (set_time, MDL);
//ScenSim-Port//	if (traceoutfile) {
//ScenSim-Port//		result = trace_begin (traceoutfile, MDL);
//ScenSim-Port//		if (result != ISC_R_SUCCESS)
//ScenSim-Port//			log_fatal ("Unable to begin trace: %s",
//ScenSim-Port//				isc_result_totext (result));
//ScenSim-Port//	}
//ScenSim-Port//	interface_trace_setup ();
//ScenSim-Port//	parse_trace_setup ();
//ScenSim-Port//	trace_srandom = trace_type_register ("random-seed", (void *)0,
//ScenSim-Port//					     trace_seed_input,
//ScenSim-Port//					     trace_seed_stop, MDL);
//ScenSim-Port//	trace_ddns_init();
//ScenSim-Port//#endif

//ScenSim-Port//#if defined (PARANOIA)
//ScenSim-Port//	/* get user and group info if those options were given */
//ScenSim-Port//	if (set_user) {
//ScenSim-Port//		struct passwd *tmp_pwd;
//ScenSim-Port//
//ScenSim-Port//		if (geteuid())
//ScenSim-Port//			log_fatal ("you must be root to set user");
//ScenSim-Port//
//ScenSim-Port//		if (!(tmp_pwd = getpwnam(set_user)))
//ScenSim-Port//			log_fatal ("no such user: %s", set_user);
//ScenSim-Port//
//ScenSim-Port//		set_uid = tmp_pwd->pw_uid;
//ScenSim-Port//
//ScenSim-Port//		/* use the user's group as the default gid */
//ScenSim-Port//		if (!set_group)
//ScenSim-Port//			set_gid = tmp_pwd->pw_gid;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (set_group) {
//ScenSim-Port///* get around the ISC declaration of group */
//ScenSim-Port//#define group real_group
//ScenSim-Port//		struct group *tmp_grp;
//ScenSim-Port//
//ScenSim-Port//		if (geteuid())
//ScenSim-Port//			log_fatal ("you must be root to set group");
//ScenSim-Port//
//ScenSim-Port//		if (!(tmp_grp = getgrnam(set_group)))
//ScenSim-Port//			log_fatal ("no such group: %s", set_group);
//ScenSim-Port//
//ScenSim-Port//		set_gid = tmp_grp->gr_gid;
//ScenSim-Port//#undef group
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//#  if defined (EARLY_CHROOT)
//ScenSim-Port//	if (set_chroot) setup_chroot (set_chroot);
//ScenSim-Port//#  endif /* EARLY_CHROOT */
//ScenSim-Port//#endif /* PARANOIA */

	/* Default to the DHCP/BOOTP port. */
	if (!local_port)
	{
//ScenSim-Port//		if ((s = getenv ("DHCPD_PORT"))) {
//ScenSim-Port//			local_port = validate_port (s);
//ScenSim-Port//			log_debug ("binding to environment-specified port %d",
//ScenSim-Port//				   ntohs (local_port));
//ScenSim-Port//		} else {
			if (local_family == AF_INET) {
//ScenSim-Port//				ent = getservbyname("dhcp", "udp");
//ScenSim-Port//				if (ent == NULL) {
					local_port = htons(67);
//ScenSim-Port//				} else {
//ScenSim-Port//					local_port = ent->s_port;
//ScenSim-Port//				}
			} else {
				/* INSIST(local_family == AF_INET6); */
//ScenSim-Port//				ent = getservbyname("dhcpv6-server", "udp");
//ScenSim-Port//				if (ent == NULL) {
					local_port = htons(547);
//ScenSim-Port//				} else {
//ScenSim-Port//					local_port = ent->s_port;
//ScenSim-Port//				}
			}
//ScenSim-Port//#ifndef __CYGWIN32__ /* XXX */
//ScenSim-Port//			endservent ();
//ScenSim-Port//#endif
//ScenSim-Port//		}
	}
  
  	if (local_family == AF_INET) {
		remote_port = htons(ntohs(local_port) + 1);
	} else {
		/* INSIST(local_family == AF_INET6); */
//ScenSim-Port//		ent = getservbyname("dhcpv6-client", "udp");
//ScenSim-Port//		if (ent == NULL) {
			remote_port = htons(546);
//ScenSim-Port//		} else {
//ScenSim-Port//			remote_port = ent->s_port;
//ScenSim-Port//		}
	}

//ScenSim-Port//	if (server) {
//ScenSim-Port//		if (local_family != AF_INET) {
//ScenSim-Port//			log_fatal("You can only specify address to send "
//ScenSim-Port//			          "replies to when running an IPv4 server.");
//ScenSim-Port//		}
//ScenSim-Port//		if (!inet_aton (server, &limited_broadcast)) {
//ScenSim-Port//			struct hostent *he;
//ScenSim-Port//			he = gethostbyname (server);
//ScenSim-Port//			if (he) {
//ScenSim-Port//				memcpy (&limited_broadcast,
//ScenSim-Port//					he -> h_addr_list [0],
//ScenSim-Port//					sizeof limited_broadcast);
//ScenSim-Port//			} else
//ScenSim-Port//				limited_broadcast.s_addr = INADDR_BROADCAST;
//ScenSim-Port//		}
//ScenSim-Port//	} else {
		limited_broadcast.s_addr = INADDR_BROADCAST;
//ScenSim-Port//	}

	/* Get the current time... */
//ScenSim-Port//	gettimeofday(&cur_tv, NULL);

	/* Set up the initial dhcp option universe. */
	initialize_common_option_spaces ();
	initialize_server_option_spaces ();

	/* Add the ddns update style enumeration prior to parsing. */
	add_enumeration (&ddns_styles);
	add_enumeration (&syslog_enum);
//ScenSim-Port//#if defined (LDAP_CONFIGURATION)
//ScenSim-Port//	add_enumeration (&ldap_methods);
//ScenSim-Port//#if defined (LDAP_USE_SSL)
//ScenSim-Port//	add_enumeration (&ldap_ssl_usage_enum);
//ScenSim-Port//	add_enumeration (&ldap_tls_reqcert_enum);
//ScenSim-Port//	add_enumeration (&ldap_tls_crlcheck_enum);
//ScenSim-Port//#endif
//ScenSim-Port//#endif

	if (!group_allocate (&root_group, MDL))
		log_fatal ("Can't allocate root group!");
	root_group -> authoritative = 0;

	/* Set up various hooks. */
//ScenSim-Port//	dhcp_interface_setup_hook = dhcpd_interface_setup_hook;
//ScenSim-Port//	bootp_packet_handler = do_packet;
//ScenSim-Port//#ifdef DHCPv6
//ScenSim-Port//	dhcpv6_packet_handler = do_packet6;
//ScenSim-Port//#endif /* DHCPv6 */

//ScenSim-Port//#if defined (NSUPDATE)
//ScenSim-Port//	/* Set up the standard name service updater routine. */
//ScenSim-Port//	parse = NULL;
//ScenSim-Port//	status = new_parse(&parse, -1, std_nsupdate, sizeof(std_nsupdate) - 1,
//ScenSim-Port//			    "standard name service update routine", 0);
//ScenSim-Port//	if (status != ISC_R_SUCCESS)
//ScenSim-Port//		log_fatal ("can't begin parsing name service updater!");
//ScenSim-Port//
//ScenSim-Port//	if (parse != NULL) {
//ScenSim-Port//		lose = 0;
//ScenSim-Port//		if (!(parse_executable_statements(&root_group->statements,
//ScenSim-Port//						  parse, &lose, context_any))) {
//ScenSim-Port//			end_parse(&parse);
//ScenSim-Port//			log_fatal("can't parse standard name service updater!");
//ScenSim-Port//		}
//ScenSim-Port//		end_parse(&parse);
//ScenSim-Port//	}
//ScenSim-Port//#endif

	/* Initialize icmp support... */
//ScenSim-Port//	if (!cftest && !lftest)
//ScenSim-Port//		icmp_startup (1, lease_pinged);

//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//	if (traceinfile) {
//ScenSim-Port//	    if (!no_dhcpd_db) {
//ScenSim-Port//		    log_error ("%s", "");
//ScenSim-Port//		    log_error ("** You must specify a lease file with -lf.");
//ScenSim-Port//		    log_error ("   Dhcpd will not overwrite your default");
//ScenSim-Port//		    log_fatal ("   lease file when playing back a trace. **");
//ScenSim-Port//	    }		
//ScenSim-Port//	    trace_file_replay (traceinfile);
//ScenSim-Port//
//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) && \
//ScenSim-Port//                defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//            free_everything ();
//ScenSim-Port//            omapi_print_dmalloc_usage_by_caller (); 
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//	    exit (0);
//ScenSim-Port//	}
//ScenSim-Port//#endif

#ifdef DHCPv6
	/* set up DHCPv6 hashes */
	if (!ia_new_hash(&ia_na_active, DEFAULT_HASH_SIZE, MDL)) {
		log_fatal("Out of memory creating hash for active IA_NA.");
	}
	if (!ia_new_hash(&ia_ta_active, DEFAULT_HASH_SIZE, MDL)) {
		log_fatal("Out of memory creating hash for active IA_TA.");
	}
	if (!ia_new_hash(&ia_pd_active, DEFAULT_HASH_SIZE, MDL)) {
		log_fatal("Out of memory creating hash for active IA_PD.");
	}
#endif /* DHCPv6 */

	/* Read the dhcpd.conf file... */
	if (readconf () != ISC_R_SUCCESS)
		log_fatal ("Configuration file errors encountered -- exiting");

	postconf_initialization (quiet);
 
//ScenSim-Port//#if defined (PARANOIA) && !defined (EARLY_CHROOT)
//ScenSim-Port//	if (set_chroot) setup_chroot (set_chroot);
//ScenSim-Port//#endif /* PARANOIA && !EARLY_CHROOT */

        /* test option should cause an early exit */
 	if (cftest && !lftest) 
 		exit(0);

	group_write_hook = group_writer;

	/* Start up the database... */
	db_startup (lftest);

	if (lftest)
		exit (0);

	/* Discover all the network interfaces and initialize them. */
//ScenSim-Port//	discover_interfaces(DISCOVER_SERVER);
    unsigned int num_of_interfaces = NumberOfInterfaces(curctx->glue);//ScenSim-Port//
    for (int i = 0; i < num_of_interfaces; ++i) {//ScenSim-Port//
        struct interface_info *new_interface = NULL;//ScenSim-Port//
        isc_result_t status = interface_allocate(&new_interface, MDL);//ScenSim-Port//
        assert(status == ISC_R_SUCCESS);//ScenSim-Port//
        strcpy(new_interface->name, GetInterfaceId(curctx->glue, i));//ScenSim-Port//
        GetHardwareAddress(curctx->glue, i, new_interface->hw_address.hbuf, &new_interface->hw_address.hlen);//ScenSim-Port//
        struct iaddr ia;//ScenSim-Port//
        GetIpAddress(curctx->glue, i, &ia);//ScenSim-Port//
        if (local_family == AF_INET6) {//ScenSim-Port//
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
        dhcpd_interface_setup_hook(new_interface, &ia);//ScenSim-Port//
        if (interfaces) {//ScenSim-Port//
            interface_reference(&new_interface->next, interfaces, MDL);//ScenSim-Port//
            interface_dereference(&interfaces, MDL);//ScenSim-Port//
        }//ScenSim-Port//
        interface_reference(&interfaces, new_interface, MDL);//ScenSim-Port//
    }//ScenSim-Port//

#ifdef DHCPv6
	/*
	 * Remove addresses from our pools that we should not issue
	 * to clients.
	 *
	 * We currently have no support for this in IPv4. It is not 
	 * as important in IPv4, as making pools with ranges that 
	 * leave out interfaces and hosts is fairly straightforward
	 * using range notation, but not so handy with CIDR notation.
	 */
	if (local_family == AF_INET6) {
		mark_hosts_unavailable();
		mark_phosts_unavailable();
		mark_interfaces_unavailable();
	}
#endif /* DHCPv6 */


	/* Make up a seed for the random number generator from current
	   time plus the sum of the last four bytes of each
	   interface's hardware address interpreted as an integer.
	   Not much entropy, but we're booting, so we're not likely to
	   find anything better. */
//ScenSim-Port//	seed = 0;
//ScenSim-Port//	for (ip = interfaces; ip; ip = ip -> next) {
//ScenSim-Port//		int junk;
//ScenSim-Port//		memcpy (&junk,
//ScenSim-Port//			&ip -> hw_address.hbuf [ip -> hw_address.hlen -
//ScenSim-Port//					       sizeof seed], sizeof seed);
//ScenSim-Port//		seed += junk;
//ScenSim-Port//	}
//ScenSim-Port//	srandom (seed + cur_time);
//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//	trace_seed_stash (trace_srandom, seed + cur_time);
//ScenSim-Port//#endif
	postdb_startup ();

#ifdef DHCPv6
	/*
	 * Set server DHCPv6 identifier.
	 * See dhcpv6.c for discussion of setting DUID.
	 */
	if (set_server_duid_from_option() == ISC_R_SUCCESS) {
		write_server_duid();
	} else {
		if (!server_duid_isset()) {
			if (generate_new_server_duid() != ISC_R_SUCCESS) {
				log_fatal("Unable to set server identifier.");
			}
			write_server_duid();
		}
	}
#endif /* DHCPv6 */

//ScenSim-Port//#ifndef DEBUG
//ScenSim-Port//	if (daemon) {
//ScenSim-Port//		/* First part of becoming a daemon... */
//ScenSim-Port//		if ((pid = fork ()) < 0)
//ScenSim-Port//			log_fatal ("Can't fork daemon: %m");
//ScenSim-Port//		else if (pid)
//ScenSim-Port//			exit (0);
//ScenSim-Port//	}
//ScenSim-Port// 
//ScenSim-Port//#if defined (PARANOIA)
//ScenSim-Port//	/* change uid to the specified one */
//ScenSim-Port//
//ScenSim-Port//	if (set_gid) {
//ScenSim-Port//		if (setgroups (0, (void *)0))
//ScenSim-Port//			log_fatal ("setgroups: %m");
//ScenSim-Port//		if (setgid (set_gid))
//ScenSim-Port//			log_fatal ("setgid(%d): %m", (int) set_gid);
//ScenSim-Port//	}	
//ScenSim-Port//
//ScenSim-Port//	if (set_uid) {
//ScenSim-Port//		if (setuid (set_uid))
//ScenSim-Port//			log_fatal ("setuid(%d): %m", (int) set_uid);
//ScenSim-Port//	}
//ScenSim-Port//#endif /* PARANOIA */
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * Deal with pid files.  If the user told us
//ScenSim-Port//	 * not to write a file we don't read one either
//ScenSim-Port//	 */
//ScenSim-Port//	if (no_pid_file == ISC_FALSE) {
//ScenSim-Port//		/*Read previous pid file. */
//ScenSim-Port//		if ((i = open (path_dhcpd_pid, O_RDONLY)) >= 0) {
//ScenSim-Port//			status = read(i, pbuf, (sizeof pbuf) - 1);
//ScenSim-Port//			close (i);
//ScenSim-Port//			if (status > 0) {
//ScenSim-Port//				pbuf[status] = 0;
//ScenSim-Port//				pid = atoi(pbuf);
//ScenSim-Port//
//ScenSim-Port//				/*
//ScenSim-Port//				 * If there was a previous server process and
//ScenSim-Port//				 * it is still running, abort
//ScenSim-Port//				 */
//ScenSim-Port//				if (!pid ||
//ScenSim-Port//				    (pid != getpid() && kill(pid, 0) == 0))
//ScenSim-Port//					log_fatal("There's already a "
//ScenSim-Port//						  "DHCP server running.");
//ScenSim-Port//			}
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//		/* Write new pid file. */
//ScenSim-Port//		i = open(path_dhcpd_pid, O_WRONLY|O_CREAT|O_TRUNC, 0644);
//ScenSim-Port//		if (i >= 0) {
//ScenSim-Port//			sprintf(pbuf, "%d\n", (int) getpid());
//ScenSim-Port//			IGNORE_RET (write(i, pbuf, strlen(pbuf)));
//ScenSim-Port//			close(i);
//ScenSim-Port//		} else {
//ScenSim-Port//			log_error("Can't create PID file %s: %m.",
//ScenSim-Port//				  path_dhcpd_pid);
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/* If we were requested to log to stdout on the command line,
//ScenSim-Port//	   keep doing so; otherwise, stop. */
//ScenSim-Port//	if (log_perror == -1)
//ScenSim-Port//		log_perror = 1;
//ScenSim-Port//	else
//ScenSim-Port//		log_perror = 0;
//ScenSim-Port//
//ScenSim-Port//	if (daemon) {
//ScenSim-Port//		/* Become session leader and get pid... */
//ScenSim-Port//		pid = setsid();
//ScenSim-Port//
//ScenSim-Port//                /* Close standard I/O descriptors. */
//ScenSim-Port//                close(0);
//ScenSim-Port//                close(1);
//ScenSim-Port//                close(2);
//ScenSim-Port//
//ScenSim-Port//                /* Reopen them on /dev/null. */
//ScenSim-Port//                open("/dev/null", O_RDWR);
//ScenSim-Port//                open("/dev/null", O_RDWR);
//ScenSim-Port//                open("/dev/null", O_RDWR);
//ScenSim-Port//                log_perror = 0; /* No sense logging to /dev/null. */
//ScenSim-Port//
//ScenSim-Port//       		IGNORE_RET (chdir("/"));
//ScenSim-Port//	}
//ScenSim-Port//#endif /* !DEBUG */

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || defined (DEBUG_MALLOC_POOL) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//	dmalloc_cutoff_generation = dmalloc_generation;
//ScenSim-Port//	dmalloc_longterm = dmalloc_outstanding;
//ScenSim-Port//	dmalloc_outstanding = 0;
//ScenSim-Port//#endif

//ScenSim-Port//	omapi_set_int_value ((omapi_object_t *)dhcp_control_object,
//ScenSim-Port//			     (omapi_object_t *)0, "state", server_running);

	/* Receive packets and dispatch them... */
//ScenSim-Port//	dispatch ();

	/* Not reached */
	return 0;
}
//ScenSim-Port//#endif /* !UNIT_TEST */

void postconf_initialization (int quiet)
{
	struct option_state *options = (struct option_state *)0;
	struct data_string db;
	struct option_cache *oc;
	char *s;
	isc_result_t result;
//ScenSim-Port//#if defined (NSUPDATE)
//ScenSim-Port//	struct parse *parse;
//ScenSim-Port//#endif
	int tmp;

	/* Now try to get the lease file name. */
	option_state_allocate (&options, MDL);

	execute_statements_in_scope ((struct binding_value **)0,
				     (struct packet *)0,
				     (struct lease *)0,
				     (struct client_state *)0,
				     (struct option_state *)0,
				     options, &global_scope,
				     root_group,
				     (struct group *)0);
	memset (&db, 0, sizeof db);
	oc = lookup_option (&server_universe, options, SV_LEASE_FILE_NAME);
	if (oc &&
	    evaluate_option_cache (&db, (struct packet *)0,
				   (struct lease *)0, (struct client_state *)0,
				   options, (struct option_state *)0,
				   &global_scope, oc, MDL)) {
//ScenSim-Port//		s = dmalloc (db.len + 1, MDL);
		s = (char*)dmalloc (db.len + 1, MDL);//ScenSim-Port//
		if (!s)
			log_fatal ("no memory for lease db filename.");
		memcpy (s, db.data, db.len);
		s [db.len] = 0;
		data_string_forget (&db, MDL);
		path_dhcpd_db = s;
	}

//ScenSim-Port//	oc = lookup_option (&server_universe, options, SV_PID_FILE_NAME);
//ScenSim-Port//	if (oc &&
//ScenSim-Port//	    evaluate_option_cache (&db, (struct packet *)0,
//ScenSim-Port//				   (struct lease *)0, (struct client_state *)0,
//ScenSim-Port//				   options, (struct option_state *)0,
//ScenSim-Port//				   &global_scope, oc, MDL)) {
//ScenSim-Port//		s = dmalloc (db.len + 1, MDL);
//ScenSim-Port//		if (!s)
//ScenSim-Port//			log_fatal ("no memory for pid filename.");
//ScenSim-Port//		memcpy (s, db.data, db.len);
//ScenSim-Port//		s [db.len] = 0;
//ScenSim-Port//		data_string_forget (&db, MDL);
//ScenSim-Port//		path_dhcpd_pid = s;
//ScenSim-Port//	}

#ifdef DHCPv6
        if (local_family == AF_INET6) {
                /*
                 * Override lease file name with dhcpv6 lease file name,
                 * if it was set; then, do the same with the pid file name
                 */
                oc = lookup_option(&server_universe, options,
                                   SV_DHCPV6_LEASE_FILE_NAME);
                if (oc &&
                    evaluate_option_cache(&db, NULL, NULL, NULL,
				          options, NULL, &global_scope,
                                          oc, MDL)) {
//ScenSim-Port//                        s = dmalloc (db.len + 1, MDL);
                        s = (char*)dmalloc (db.len + 1, MDL);//ScenSim-Port//
                        if (!s)
                                log_fatal ("no memory for lease db filename.");
                        memcpy (s, db.data, db.len);
                        s [db.len] = 0;
                        data_string_forget (&db, MDL);
                        path_dhcpd_db = s;
                }

//ScenSim-Port//                oc = lookup_option(&server_universe, options,
//ScenSim-Port//                                   SV_DHCPV6_PID_FILE_NAME);
//ScenSim-Port//                if (oc &&
//ScenSim-Port//                    evaluate_option_cache(&db, NULL, NULL, NULL,
//ScenSim-Port//				          options, NULL, &global_scope,
//ScenSim-Port//                                          oc, MDL)) {
//ScenSim-Port//                        s = dmalloc (db.len + 1, MDL);
//ScenSim-Port//                        if (!s)
//ScenSim-Port//                                log_fatal ("no memory for pid filename.");
//ScenSim-Port//                        memcpy (s, db.data, db.len);
//ScenSim-Port//                        s [db.len] = 0;
//ScenSim-Port//                        data_string_forget (&db, MDL);
//ScenSim-Port//                        path_dhcpd_pid = s;
//ScenSim-Port//                }
        }
#endif /* DHCPv6 */

//ScenSim-Port//	omapi_port = -1;
//ScenSim-Port//	oc = lookup_option (&server_universe, options, SV_OMAPI_PORT);
//ScenSim-Port//	if (oc &&
//ScenSim-Port//	    evaluate_option_cache (&db, (struct packet *)0,
//ScenSim-Port//				   (struct lease *)0, (struct client_state *)0,
//ScenSim-Port//				   options, (struct option_state *)0,
//ScenSim-Port//				   &global_scope, oc, MDL)) {
//ScenSim-Port//		if (db.len == 2) {
//ScenSim-Port//			omapi_port = getUShort (db.data);
//ScenSim-Port//		} else
//ScenSim-Port//			log_fatal ("invalid omapi port data length");
//ScenSim-Port//		data_string_forget (&db, MDL);
//ScenSim-Port//	}

//ScenSim-Port//	oc = lookup_option (&server_universe, options, SV_OMAPI_KEY);
//ScenSim-Port//	if (oc &&
//ScenSim-Port//	    evaluate_option_cache (&db, (struct packet *)0,
//ScenSim-Port//				   (struct lease *)0, (struct client_state *)0,
//ScenSim-Port//				   options,
//ScenSim-Port//				   (struct option_state *)0,
//ScenSim-Port//				   &global_scope, oc, MDL)) {
//ScenSim-Port//		s = dmalloc (db.len + 1, MDL);
//ScenSim-Port//		if (!s)
//ScenSim-Port//			log_fatal ("no memory for OMAPI key filename.");
//ScenSim-Port//		memcpy (s, db.data, db.len);
//ScenSim-Port//		s [db.len] = 0;
//ScenSim-Port//		data_string_forget (&db, MDL);
//ScenSim-Port//		result = omapi_auth_key_lookup_name (&omapi_key, s);
//ScenSim-Port//		dfree (s, MDL);
//ScenSim-Port//		if (result != ISC_R_SUCCESS)
//ScenSim-Port//			log_fatal ("OMAPI key %s: %s",
//ScenSim-Port//				   s, isc_result_totext (result));
//ScenSim-Port//	}

	oc = lookup_option (&server_universe, options, SV_LOCAL_PORT);
	if (oc &&
	    evaluate_option_cache (&db, (struct packet *)0,
				   (struct lease *)0, (struct client_state *)0,
				   options,
				   (struct option_state *)0,
				   &global_scope, oc, MDL)) {
		if (db.len == 2) {
			local_port = htons (getUShort (db.data));
		} else
			log_fatal ("invalid local port data length");
		data_string_forget (&db, MDL);
	}

	oc = lookup_option (&server_universe, options, SV_REMOTE_PORT);
	if (oc &&
	    evaluate_option_cache (&db, (struct packet *)0,
				   (struct lease *)0, (struct client_state *)0,
				   options, (struct option_state *)0,
				   &global_scope, oc, MDL)) {
		if (db.len == 2) {
			remote_port = htons (getUShort (db.data));
		} else
			log_fatal ("invalid remote port data length");
		data_string_forget (&db, MDL);
	}

	oc = lookup_option (&server_universe, options,
			    SV_LIMITED_BROADCAST_ADDRESS);
	if (oc &&
	    evaluate_option_cache (&db, (struct packet *)0,
				   (struct lease *)0, (struct client_state *)0,
				   options, (struct option_state *)0,
				   &global_scope, oc, MDL)) {
		if (db.len == 4) {
			memcpy (&limited_broadcast, db.data, 4);
		} else
			log_fatal ("invalid broadcast address data length");
		data_string_forget (&db, MDL);
	}

	oc = lookup_option (&server_universe, options,
			    SV_LOCAL_ADDRESS);
	if (oc &&
	    evaluate_option_cache (&db, (struct packet *)0,
				   (struct lease *)0, (struct client_state *)0,
				   options, (struct option_state *)0,
				   &global_scope, oc, MDL)) {
		if (db.len == 4) {
			memcpy (&local_address, db.data, 4);
		} else
			log_fatal ("invalid local address data length");
		data_string_forget (&db, MDL);
	}

	oc = lookup_option (&server_universe, options, SV_DDNS_UPDATE_STYLE);
	if (oc) {
		if (evaluate_option_cache (&db, (struct packet *)0,
					   (struct lease *)0,
					   (struct client_state *)0,
					   options,
					   (struct option_state *)0,
					   &global_scope, oc, MDL)) {
			if (db.len == 1) {
				ddns_update_style = db.data [0];
			} else
				log_fatal ("invalid dns update type");
			data_string_forget (&db, MDL);
		}
	} else {
		ddns_update_style = DDNS_UPDATE_STYLE_NONE;
	}
//ScenSim-Port//#if defined (NSUPDATE)
//ScenSim-Port//	/* We no longer support ad_hoc, tell the user */
//ScenSim-Port//	if (ddns_update_style == DDNS_UPDATE_STYLE_AD_HOC) {
//ScenSim-Port//		log_fatal("ddns-update-style ad_hoc no longer supported");
//ScenSim-Port//	}
//ScenSim-Port//#else
	/* If we don't have support for updates compiled in tell the user */
	if (ddns_update_style != DDNS_UPDATE_STYLE_NONE) {
		log_fatal("Support for ddns-update-style not compiled in");
	}
//ScenSim-Port//#endif

//ScenSim-Port//	oc = lookup_option (&server_universe, options, SV_LOG_FACILITY);
//ScenSim-Port//	if (oc) {
//ScenSim-Port//		if (evaluate_option_cache (&db, (struct packet *)0,
//ScenSim-Port//					   (struct lease *)0,
//ScenSim-Port//					   (struct client_state *)0,
//ScenSim-Port//					   options,
//ScenSim-Port//					   (struct option_state *)0,
//ScenSim-Port//					   &global_scope, oc, MDL)) {
//ScenSim-Port//			if (db.len == 1) {
//ScenSim-Port//				closelog ();
//ScenSim-Port//				openlog ("dhcpd", LOG_NDELAY, db.data[0]);
//ScenSim-Port//				/* Log the startup banner into the new
//ScenSim-Port//				   log file. */
//ScenSim-Port//				if (!quiet) {
//ScenSim-Port//					/* Don't log to stderr twice. */
//ScenSim-Port//					tmp = log_perror;
//ScenSim-Port//					log_perror = 0;
//ScenSim-Port//					log_info("%s %s",
//ScenSim-Port//						 message, PACKAGE_VERSION);
//ScenSim-Port//					log_info (copyright);
//ScenSim-Port//					log_info (arr);
//ScenSim-Port//					log_info (url);
//ScenSim-Port//					log_perror = tmp;
//ScenSim-Port//				}
//ScenSim-Port//			} else
//ScenSim-Port//				log_fatal ("invalid log facility");
//ScenSim-Port//			data_string_forget (&db, MDL);
//ScenSim-Port//		}
//ScenSim-Port//	}
	
	oc = lookup_option(&server_universe, options, SV_DELAYED_ACK);
	if (oc &&
	    evaluate_option_cache(&db, NULL, NULL, NULL, options, NULL,
				  &global_scope, oc, MDL)) {
		if (db.len == 2) {
			max_outstanding_acks = htons(getUShort(db.data));
		} else {
			log_fatal("invalid max delayed ACK count ");
		}
		data_string_forget(&db, MDL);
	}

	oc = lookup_option(&server_universe, options, SV_MAX_ACK_DELAY);
	if (oc &&
	    evaluate_option_cache(&db, NULL, NULL, NULL, options, NULL,
				  &global_scope, oc, MDL)) {
		u_int32_t timeval;

		if (db.len != 4)
			log_fatal("invalid max ack delay configuration");

		timeval = getULong(db.data);
		max_ack_delay_secs  = timeval / 1000000;
		max_ack_delay_usecs = timeval % 1000000;

		data_string_forget(&db, MDL);
	}

	/* Don't need the options anymore. */
	option_state_dereference (&options, MDL);
	
//ScenSim-Port//#if defined (NSUPDATE)
//ScenSim-Port//	/* If old-style ddns updates have been requested, parse the
//ScenSim-Port//	   old-style ddns updater. */
//ScenSim-Port//	if (ddns_update_style == 1) {
//ScenSim-Port//		struct executable_statement **e, *s;
//ScenSim-Port//
//ScenSim-Port//		if (root_group -> statements) {
//ScenSim-Port//			s = (struct executable_statement *)0;
//ScenSim-Port//			if (!executable_statement_allocate (&s, MDL))
//ScenSim-Port//				log_fatal ("no memory for ddns updater");
//ScenSim-Port//			executable_statement_reference
//ScenSim-Port//				(&s -> next, root_group -> statements, MDL);
//ScenSim-Port//			executable_statement_dereference
//ScenSim-Port//				(&root_group -> statements, MDL);
//ScenSim-Port//			executable_statement_reference
//ScenSim-Port//				(&root_group -> statements, s, MDL);
//ScenSim-Port//			s -> op = statements_statement;
//ScenSim-Port//			e = &s -> data.statements;
//ScenSim-Port//			executable_statement_dereference (&s, MDL);
//ScenSim-Port//		} else {
//ScenSim-Port//			e = &root_group -> statements;
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//		/* Set up the standard name service updater routine. */
//ScenSim-Port//		parse = NULL;
//ScenSim-Port//		result = new_parse(&parse, -1, old_nsupdate,
//ScenSim-Port//				   sizeof(old_nsupdate) - 1,
//ScenSim-Port//				   "old name service update routine", 0);
//ScenSim-Port//		if (result != ISC_R_SUCCESS)
//ScenSim-Port//			log_fatal ("can't begin parsing old ddns updater!");
//ScenSim-Port//
//ScenSim-Port//		if (parse != NULL) {
//ScenSim-Port//			tmp = 0;
//ScenSim-Port//			if (!(parse_executable_statements(e, parse, &tmp,
//ScenSim-Port//							  context_any))) {
//ScenSim-Port//				end_parse(&parse);
//ScenSim-Port//				log_fatal("can't parse standard ddns updater!");
//ScenSim-Port//			}
//ScenSim-Port//		}
//ScenSim-Port//		end_parse(&parse);
//ScenSim-Port//	}
//ScenSim-Port//#endif
}

void postdb_startup (void)
{
	/* Initialize the omapi listener state. */
//ScenSim-Port//	if (omapi_port != -1) {
//ScenSim-Port//		omapi_listener_start (0);
//ScenSim-Port//	}

#if defined (FAILOVER_PROTOCOL)
	/* Initialize the failover listener state. */
	dhcp_failover_startup ();
#endif

	/*
	 * Begin our lease timeout background task.
	 */
//ScenSim-Port//	schedule_all_ipv6_lease_timeouts();
}

/* Print usage message. */

//ScenSim-Port//static void
//ScenSim-Port//usage(void) {
//ScenSim-Port//	log_info("%s %s", message, PACKAGE_VERSION);
//ScenSim-Port//	log_info(copyright);
//ScenSim-Port//	log_info(arr);
//ScenSim-Port//
//ScenSim-Port//	log_fatal("Usage: dhcpd [-p <UDP port #>] [-f] [-d] [-q] [-t|-T]\n"
//ScenSim-Port//#ifdef DHCPv6
//ScenSim-Port//		  "             [-4|-6] [-cf config-file] [-lf lease-file]\n"
//ScenSim-Port//#else /* !DHCPv6 */
//ScenSim-Port//		  "             [-cf config-file] [-lf lease-file]\n"
//ScenSim-Port//#endif /* DHCPv6 */
//ScenSim-Port//#if defined (PARANOIA)
//ScenSim-Port//		   /* meld into the following string */
//ScenSim-Port//		  "             [-user user] [-group group] [-chroot dir]\n"
//ScenSim-Port//#endif /* PARANOIA */
//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//		  "             [-tf trace-output-file]\n"
//ScenSim-Port//		  "             [-play trace-input-file]\n"
//ScenSim-Port//#endif /* TRACING */
//ScenSim-Port//		  "             [-pf pid-file] [--no-pid] [-s server]\n"
//ScenSim-Port//		  "             [if0 [...ifN]]");
//ScenSim-Port//}

//ScenSim-Port//void lease_pinged (from, packet, length)
//ScenSim-Port//	struct iaddr from;
//ScenSim-Port//	u_int8_t *packet;
//ScenSim-Port//	int length;
void lease_pinged (struct iaddr from, u_int8_t *packet, int length)//ScenSim-Port//
{
	struct lease *lp;

	/* Don't try to look up a pinged lease if we aren't trying to
	   ping one - otherwise somebody could easily make us churn by
	   just forging repeated ICMP EchoReply packets for us to look
	   up. */
	if (!outstanding_pings)
		return;

	lp = (struct lease *)0;
	if (!find_lease_by_ip_addr (&lp, from, MDL)) {
		log_debug ("unexpected ICMP Echo Reply from %s",
			   piaddr (from));
		return;
	}

	if (!lp -> state) {
#if defined (FAILOVER_PROTOCOL)
		if (!lp -> pool ||
		    !lp -> pool -> failover_peer)
#endif
			log_debug ("ICMP Echo Reply for %s late or spurious.",
				   piaddr (from));
		goto out;
	}

	if (lp -> ends > cur_time) {
		log_debug ("ICMP Echo reply while lease %s valid.",
			   piaddr (from));
	}

	/* At this point it looks like we pinged a lease and got a
	   response, which shouldn't have happened. */
	data_string_forget (&lp -> state -> parameter_request_list, MDL);
	free_lease_state (lp -> state, MDL);
	lp -> state = (struct lease_state *)0;

	abandon_lease (lp, "pinged before offer");
	cancel_timeout (lease_ping_timeout, lp);
	--outstanding_pings;
      out:
	lease_dereference (&lp, MDL);
}

//ScenSim-Port//void lease_ping_timeout (vlp)
//ScenSim-Port//	void *vlp;
void lease_ping_timeout (void *vlp)//ScenSim-Port//
{
//ScenSim-Port//	struct lease *lp = vlp;
	struct lease *lp = (struct lease*)vlp;//ScenSim-Port//

#if defined (DEBUG_MEMORY_LEAKAGE)
	unsigned long previous_outstanding = dmalloc_outstanding;
#endif

	--outstanding_pings;
	dhcp_reply (lp);

#if defined (DEBUG_MEMORY_LEAKAGE)
	log_info ("generation %ld: %ld new, %ld outstanding, %ld long-term",
		  dmalloc_generation,
		  dmalloc_outstanding - previous_outstanding,
		  dmalloc_outstanding, dmalloc_longterm);
#endif
#if defined (DEBUG_MEMORY_LEAKAGE)
	dmalloc_dump_outstanding ();
#endif
}

int dhcpd_interface_setup_hook (struct interface_info *ip, struct iaddr *ia)
{
	struct subnet *subnet;
	struct shared_network *share;
//ScenSim-Port//	isc_result_t status;

	/* Special case for fallback network - not sure why this is
	   necessary. */
//ScenSim-Port//	if (!ia) {
//ScenSim-Port//		const char *fnn = "fallback-net";
//ScenSim-Port//		status = shared_network_allocate (&ip -> shared_network, MDL);
//ScenSim-Port//		if (status != ISC_R_SUCCESS)
//ScenSim-Port//			log_fatal ("No memory for shared subnet: %s",
//ScenSim-Port//				   isc_result_totext (status));
//ScenSim-Port//		ip -> shared_network -> name = dmalloc (strlen (fnn) + 1, MDL);
//ScenSim-Port//		strcpy (ip -> shared_network -> name, fnn);
//ScenSim-Port//		return 1;
//ScenSim-Port//	}

	/* If there's a registered subnet for this address,
	   connect it together... */
	subnet = (struct subnet *)0;
	if (find_subnet (&subnet, *ia, MDL)) {
		/* If this interface has multiple aliases on the same
		   subnet, ignore all but the first we encounter. */
		if (!subnet -> interface) {
			interface_reference (&subnet -> interface, ip, MDL);
			subnet -> interface_address = *ia;
		} else if (subnet -> interface != ip) {
			log_error ("Multiple interfaces match the %s: %s %s", 
				   "same subnet",
				   subnet -> interface -> name, ip -> name);
		}
		share = subnet -> shared_network;
		if (ip -> shared_network &&
		    ip -> shared_network != share) {
			log_fatal ("Interface %s matches multiple shared %s",
				   ip -> name, "networks");
		} else {
			if (!ip -> shared_network)
				shared_network_reference
					(&ip -> shared_network, share, MDL);
		}
		
		if (!share -> interface) {
			interface_reference (&share -> interface, ip, MDL);
		} else if (share -> interface != ip) {
			log_error ("Multiple interfaces match the %s: %s %s", 
				   "same shared network",
				   share -> interface -> name, ip -> name);
		}
		subnet_dereference (&subnet, MDL);
	}
	return 1;
}

static TIME shutdown_time;
static int omapi_connection_count;
enum dhcp_shutdown_state shutdown_state;

isc_result_t dhcp_io_shutdown (omapi_object_t *obj, void *foo)
{
	/* Shut down all listeners. */
//ScenSim-Port//	if (shutdown_state == shutdown_listeners &&
//ScenSim-Port//	    obj -> type == omapi_type_listener &&
//ScenSim-Port//	    obj -> inner &&
//ScenSim-Port//	    obj -> inner -> type == omapi_type_protocol_listener) {
//ScenSim-Port//		omapi_listener_destroy (obj, MDL);
//ScenSim-Port//		return ISC_R_SUCCESS;
//ScenSim-Port//	}

	/* Shut down all existing omapi connections. */
//ScenSim-Port//	if (obj -> type == omapi_type_connection &&
//ScenSim-Port//	    obj -> inner &&
//ScenSim-Port//	    obj -> inner -> type == omapi_type_protocol) {
//ScenSim-Port//		if (shutdown_state == shutdown_drop_omapi_connections) {
//ScenSim-Port//			omapi_disconnect (obj, 1);
//ScenSim-Port//		}
//ScenSim-Port//		omapi_connection_count++;
//ScenSim-Port//		if (shutdown_state == shutdown_omapi_connections) {
//ScenSim-Port//			omapi_disconnect (obj, 0);
//ScenSim-Port//			return ISC_R_SUCCESS;
//ScenSim-Port//		}
//ScenSim-Port//	}

	/* Shutdown all DHCP interfaces. */
	if (obj -> type == dhcp_type_interface &&
	    shutdown_state == shutdown_dhcp) {
		dhcp_interface_remove (obj, (omapi_object_t *)0);
		return ISC_R_SUCCESS;
	}
	return ISC_R_SUCCESS;
}

//ScenSim-Port//static isc_result_t dhcp_io_shutdown_countdown (void *vlp)
//ScenSim-Port//{
//ScenSim-Port//#if defined (FAILOVER_PROTOCOL)
//ScenSim-Port//	dhcp_failover_state_t *state;
//ScenSim-Port//	int failover_connection_count = 0;
//ScenSim-Port//#endif
//ScenSim-Port//	struct timeval tv;
//ScenSim-Port//
//ScenSim-Port//      oncemore:
//ScenSim-Port//	if (shutdown_state == shutdown_listeners ||
//ScenSim-Port//	    shutdown_state == shutdown_omapi_connections ||
//ScenSim-Port//	    shutdown_state == shutdown_drop_omapi_connections ||
//ScenSim-Port//	    shutdown_state == shutdown_dhcp) {
//ScenSim-Port//		omapi_connection_count = 0;
//ScenSim-Port//		omapi_io_state_foreach (dhcp_io_shutdown, 0);
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if ((shutdown_state == shutdown_listeners ||
//ScenSim-Port//	     shutdown_state == shutdown_omapi_connections ||
//ScenSim-Port//	     shutdown_state == shutdown_drop_omapi_connections) &&
//ScenSim-Port//	    omapi_connection_count == 0) {
//ScenSim-Port//		shutdown_state = shutdown_dhcp;
//ScenSim-Port//		shutdown_time = cur_time;
//ScenSim-Port//		goto oncemore;
//ScenSim-Port//	} else if (shutdown_state == shutdown_listeners &&
//ScenSim-Port//		   cur_time - shutdown_time > 4) {
//ScenSim-Port//		shutdown_state = shutdown_omapi_connections;
//ScenSim-Port//		shutdown_time = cur_time;
//ScenSim-Port//	} else if (shutdown_state == shutdown_omapi_connections &&
//ScenSim-Port//		   cur_time - shutdown_time > 4) {
//ScenSim-Port//		shutdown_state = shutdown_drop_omapi_connections;
//ScenSim-Port//		shutdown_time = cur_time;
//ScenSim-Port//	} else if (shutdown_state == shutdown_drop_omapi_connections &&
//ScenSim-Port//		   cur_time - shutdown_time > 4) {
//ScenSim-Port//		shutdown_state = shutdown_dhcp;
//ScenSim-Port//		shutdown_time = cur_time;
//ScenSim-Port//		goto oncemore;
//ScenSim-Port//	} else if (shutdown_state == shutdown_dhcp &&
//ScenSim-Port//		   cur_time - shutdown_time > 4) {
//ScenSim-Port//		shutdown_state = shutdown_done;
//ScenSim-Port//		shutdown_time = cur_time;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//#if defined (FAILOVER_PROTOCOL)
//ScenSim-Port//	/* Set all failover peers into the shutdown state. */
//ScenSim-Port//	if (shutdown_state == shutdown_dhcp) {
//ScenSim-Port//	    for (state = failover_states; state; state = state -> next) {
//ScenSim-Port//		if (state -> me.state == normal) {
//ScenSim-Port//		    dhcp_failover_set_state (state, shut_down);
//ScenSim-Port//		    failover_connection_count++;
//ScenSim-Port//		}
//ScenSim-Port//		if (state -> me.state == shut_down &&
//ScenSim-Port//		    state -> partner.state != partner_down)
//ScenSim-Port//			failover_connection_count++;
//ScenSim-Port//	    }
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (shutdown_state == shutdown_done) {
//ScenSim-Port//	    for (state = failover_states; state; state = state -> next) {
//ScenSim-Port//		if (state -> me.state == shut_down) {
//ScenSim-Port//		    if (state -> link_to_peer)
//ScenSim-Port//			dhcp_failover_link_dereference (&state -> link_to_peer,
//ScenSim-Port//							MDL);
//ScenSim-Port//		    dhcp_failover_set_state (state, recover);
//ScenSim-Port//		}
//ScenSim-Port//	    }
//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) && \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//	    free_everything ();
//ScenSim-Port//	    omapi_print_dmalloc_usage_by_caller ();
//ScenSim-Port//#endif
//ScenSim-Port//	    exit (0);
//ScenSim-Port//	}		
//ScenSim-Port//#else
//ScenSim-Port//	if (shutdown_state == shutdown_done) {
//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) && \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//		free_everything ();
//ScenSim-Port//		omapi_print_dmalloc_usage_by_caller (); 
//ScenSim-Port//#endif
//ScenSim-Port//		exit (0);
//ScenSim-Port//	}
//ScenSim-Port//#endif
//ScenSim-Port//	if (shutdown_state == shutdown_dhcp &&
//ScenSim-Port//#if defined(FAILOVER_PROTOCOL)
//ScenSim-Port//	    !failover_connection_count &&
//ScenSim-Port//#endif
//ScenSim-Port//	    ISC_TRUE) {
//ScenSim-Port//		shutdown_state = shutdown_done;
//ScenSim-Port//		shutdown_time = cur_time;
//ScenSim-Port//		goto oncemore;
//ScenSim-Port//	}
//ScenSim-Port//	tv.tv_sec = cur_tv.tv_sec + 1;
//ScenSim-Port//	tv.tv_usec = cur_tv.tv_usec;
//ScenSim-Port//	add_timeout (&tv,
//ScenSim-Port//		     (void (*)(void *))dhcp_io_shutdown_countdown, 0, 0, 0);
//ScenSim-Port//	return ISC_R_SUCCESS;
//ScenSim-Port//}

//ScenSim-Port//isc_result_t dhcp_set_control_state (control_object_state_t oldstate,
//ScenSim-Port//				     control_object_state_t newstate)
//ScenSim-Port//{
//ScenSim-Port//	if (newstate == server_shutdown) {
//ScenSim-Port//		shutdown_time = cur_time;
//ScenSim-Port//		shutdown_state = shutdown_listeners;
//ScenSim-Port//		dhcp_io_shutdown_countdown (0);
//ScenSim-Port//		return ISC_R_SUCCESS;
//ScenSim-Port//	}
//ScenSim-Port//	return DHCP_R_INVALIDARG;
//ScenSim-Port//}
}//namespace IscDhcpPort////ScenSim-Port//
