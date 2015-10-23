/* socket.c

   BSD socket interface code... */

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
 * This software has been written for Internet Systems Consortium
 * by Ted Lemon in cooperation with Vixie Enterprises and Nominum, Inc.
 * To learn more about Internet Systems Consortium, see
 * ``https://www.isc.org/''.  To learn more about Vixie Enterprises,
 * see ``http://www.vix.com''.   To learn more about Nominum, Inc., see
 * ``http://www.nominum.com''.
 */
#include <iscdhcp_porting.h>//ScenSim-Port//
namespace IscDhcpPort {//ScenSim-Port//

/* SO_BINDTODEVICE support added by Elliot Poger (poger@leland.stanford.edu).
 * This sockopt allows a socket to be bound to a particular interface,
 * thus enabling the use of DHCPD on a multihomed host.
 * If SO_BINDTODEVICE is defined in your system header files, the use of
 * this sockopt will be automatically enabled. 
 * I have implemented it under Linux; other systems should be doable also.
 */

//ScenSim-Port//#include "dhcpd.h"
//ScenSim-Port//#include <errno.h>
//ScenSim-Port//#include <sys/ioctl.h>
//ScenSim-Port//#include <sys/uio.h>
//ScenSim-Port//#include <sys/uio.h>

//ScenSim-Port//#if defined(sun) && defined(USE_V4_PKTINFO)
//ScenSim-Port//#include <sys/sysmacros.h>
//ScenSim-Port//#include <net/if.h>
//ScenSim-Port//#include <sys/sockio.h>
//ScenSim-Port//#include <net/if_dl.h>
//ScenSim-Port//#include <sys/dlpi.h>
//ScenSim-Port//#endif

//ScenSim-Port//#ifdef USE_SOCKET_FALLBACK
//ScenSim-Port//# if !defined (USE_SOCKET_SEND)
//ScenSim-Port//#  define if_register_send if_register_fallback
//ScenSim-Port//#  define send_packet send_fallback
//ScenSim-Port//#  define if_reinitialize_send if_reinitialize_fallback
//ScenSim-Port//# endif
//ScenSim-Port//#endif

//ScenSim-Port//#if defined(DHCPv6)
//ScenSim-Port///*
//ScenSim-Port// * XXX: this is gross.  we need to go back and overhaul the API for socket
//ScenSim-Port// * handling.
//ScenSim-Port// */
//ScenSim-Port//static unsigned int global_v6_socket_references = 0;
//ScenSim-Port//static int global_v6_socket = -1;
//ScenSim-Port//
//ScenSim-Port//static void if_register_multicast(struct interface_info *info);
//ScenSim-Port//#endif

/*
 * We can use a single socket for AF_INET (similar to AF_INET6) on all
 * interfaces configured for DHCP if the system has support for IP_PKTINFO
 * and IP_RECVPKTINFO (for example Solaris 11).
 */
//ScenSim-Port//#if defined(IP_PKTINFO) && defined(IP_RECVPKTINFO) && defined(USE_V4_PKTINFO)
//ScenSim-Port//static unsigned int global_v4_socket_references = 0;
//ScenSim-Port//static int global_v4_socket = -1;
//ScenSim-Port//#endif

/*
 * If we can't bind() to a specific interface, then we can only have
 * a single socket. This variable insures that we don't try to listen
 * on two sockets.
 */
//ScenSim-Port//#if !defined(SO_BINDTODEVICE) && !defined(USE_FALLBACK)
//ScenSim-Port//static int once = 0;
//ScenSim-Port//#endif /* !defined(SO_BINDTODEVICE) && !defined(USE_FALLBACK) */

/* Reinitializes the specified interface after an address change.   This
   is not required for packet-filter APIs. */

#if defined (USE_SOCKET_SEND) || defined (USE_SOCKET_FALLBACK)
//ScenSim-Port//void if_reinitialize_send (info)
//ScenSim-Port//	struct interface_info *info;
void if_reinitialize_send (struct interface_info *info)//ScenSim-Port//
{
#if 0
#ifndef USE_SOCKET_RECEIVE
	once = 0;
	close (info -> wfdesc);
#endif
	if_register_send (info);
#endif
}
#endif

#ifdef USE_SOCKET_RECEIVE
//ScenSim-Port//void if_reinitialize_receive (info)
//ScenSim-Port//	struct interface_info *info;
void if_reinitialize_receive (struct interface_info *info)//ScenSim-Port//
{
#if 0
	once = 0;
	close (info -> rfdesc);
	if_register_receive (info);
#endif
}
#endif

#if defined (USE_SOCKET_SEND) || \
	defined (USE_SOCKET_RECEIVE) || \
		defined (USE_SOCKET_FALLBACK)
/* Generic interface registration routine... */
int
if_register_socket(struct interface_info *info, int family,
		   int *do_multicast)
{
	struct sockaddr_storage name;
	int name_len;
	int sock;
	int flag;
	int domain;
#ifdef DHCPv6
	struct sockaddr_in6 *addr6;
#endif
	struct sockaddr_in *addr;

	/* INSIST((family == AF_INET) || (family == AF_INET6)); */

//ScenSim-Port//#if !defined(SO_BINDTODEVICE) && !defined(USE_FALLBACK)
//ScenSim-Port//	/* Make sure only one interface is registered. */
//ScenSim-Port//	if (once) {
//ScenSim-Port//		log_fatal ("The standard socket API can only support %s",
//ScenSim-Port//		       "hosts with a single network interface.");
//ScenSim-Port//	}
//ScenSim-Port//	once = 1;
//ScenSim-Port//#endif

	/* 
	 * Set up the address we're going to bind to, depending on the
	 * address family. 
	 */ 
	memset(&name, 0, sizeof(name));
	switch (family) {
#ifdef DHCPv6
	case AF_INET6:
		addr6 = (struct sockaddr_in6 *)&name; 
		addr6->sin6_family = AF_INET6;
		addr6->sin6_port = local_port;
		/* XXX: What will happen to multicasts if this is nonzero? */
		memcpy(&addr6->sin6_addr,
		       &local_address6, 
		       sizeof(addr6->sin6_addr));
#ifdef HAVE_SA_LEN
		addr6->sin6_len = sizeof(*addr6);
#endif
		name_len = sizeof(*addr6);
		domain = PF_INET6;
		if ((info->flags & INTERFACE_STREAMS) == INTERFACE_UPSTREAM) {
			*do_multicast = 0;
		}
		break;
#endif /* DHCPv6 */

	case AF_INET:
	default:
		addr = (struct sockaddr_in *)&name; 
		addr->sin_family = AF_INET;
		addr->sin_port = local_port;
		memcpy(&addr->sin_addr,
		       &local_address,
		       sizeof(addr->sin_addr));
#ifdef HAVE_SA_LEN
		addr->sin_len = sizeof(*addr);
#endif
		name_len = sizeof(*addr);
		domain = PF_INET;
		break;
	}

	/* Make a socket... */
//ScenSim-Port//	sock = socket(domain, SOCK_DGRAM, IPPROTO_UDP);
//ScenSim-Port//	if (sock < 0) {
//ScenSim-Port//		log_fatal("Can't create dhcp socket: %m");
//ScenSim-Port//	}

	/* Set the REUSEADDR option so that we don't fail to start if
	   we're being restarted. */
	flag = 1;
//ScenSim-Port//	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR,
//ScenSim-Port//			(char *)&flag, sizeof(flag)) < 0) {
//ScenSim-Port//		log_fatal("Can't set SO_REUSEADDR option on dhcp socket: %m");
//ScenSim-Port//	}

	/* Set the BROADCAST option so that we can broadcast DHCP responses.
	   We shouldn't do this for fallback devices, and we can detect that
	   a device is a fallback because it has no ifp structure. */
//ScenSim-Port//	if (info->ifp &&
//ScenSim-Port//	    (setsockopt(sock, SOL_SOCKET, SO_BROADCAST,
//ScenSim-Port//			 (char *)&flag, sizeof(flag)) < 0)) {
//ScenSim-Port//		log_fatal("Can't set SO_BROADCAST option on dhcp socket: %m");
//ScenSim-Port//	}

#if defined(DHCPv6) && defined(SO_REUSEPORT)
	/*
	 * We only set SO_REUSEPORT on AF_INET6 sockets, so that multiple
	 * daemons can bind to their own sockets and get data for their
	 * respective interfaces.  This does not (and should not) affect
	 * DHCPv4 sockets; we can't yet support BSD sockets well, much
	 * less multiple sockets.
	 */
	if (local_family == AF_INET6) {
		flag = 1;
		if (setsockopt(sock, SOL_SOCKET, SO_REUSEPORT,
			       (char *)&flag, sizeof(flag)) < 0) {
			log_fatal("Can't set SO_REUSEPORT option on dhcp "
				  "socket: %m");
		}
	}
#endif

	/* Bind the socket to this interface's IP address. */
//ScenSim-Port//	if (bind(sock, (struct sockaddr *)&name, name_len) < 0) {
//ScenSim-Port//		log_error("Can't bind to dhcp address: %m");
//ScenSim-Port//		log_error("Please make sure there is no other dhcp server");
//ScenSim-Port//		log_error("running and that there's no entry for dhcp or");
//ScenSim-Port//		log_error("bootp in /etc/inetd.conf.   Also make sure you");
//ScenSim-Port//		log_error("are not running HP JetAdmin software, which");
//ScenSim-Port//		log_fatal("includes a bootp server.");
//ScenSim-Port//	}

#if defined(SO_BINDTODEVICE)
	/* Bind this socket to this interface. */
	if ((local_family != AF_INET6) && (info->ifp != NULL) &&
	    setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE,
			(char *)(info -> ifp), sizeof(*(info -> ifp))) < 0) {
		log_fatal("setsockopt: SO_BINDTODEVICE: %m");
	}
#endif

	/* IP_BROADCAST_IF instructs the kernel which interface to send
	 * IP packets whose destination address is 255.255.255.255.  These
	 * will be treated as subnet broadcasts on the interface identified
	 * by ip address (info -> primary_address).  This is only known to
	 * be defined in SCO system headers, and may not be defined in all
	 * releases.
	 */
#if defined(SCO) && defined(IP_BROADCAST_IF)
        if (info->address_count &&
	    setsockopt(sock, IPPROTO_IP, IP_BROADCAST_IF, &info->addresses[0],
		       sizeof(info->addresses[0])) < 0)
		log_fatal("Can't set IP_BROADCAST_IF on dhcp socket: %m");
#endif

#if defined(IP_PKTINFO) && defined(IP_RECVPKTINFO)  && defined(USE_V4_PKTINFO)
	/*
	 * If we turn on IP_RECVPKTINFO we will be able to receive
	 * the interface index information of the received packet.
	 */
	if (family == AF_INET) {
		int on = 1;
		if (setsockopt(sock, IPPROTO_IP, IP_RECVPKTINFO, 
		               &on, sizeof(on)) != 0) {
			log_fatal("setsockopt: IPV_RECVPKTINFO: %m");
		}
	}
#endif

#ifdef DHCPv6
	/*
	 * If we turn on IPV6_PKTINFO, we will be able to receive 
	 * additional information, such as the destination IP address.
	 * We need this to spot unicast packets.
	 */
	if (family == AF_INET6) {
		int on = 1;
#ifdef IPV6_RECVPKTINFO
		/* RFC3542 */
		if (setsockopt(sock, IPPROTO_IPV6, IPV6_RECVPKTINFO, 
		               &on, sizeof(on)) != 0) {
			log_fatal("setsockopt: IPV6_RECVPKTINFO: %m");
		}
#else
		/* RFC2292 */
//ScenSim-Port//		if (setsockopt(sock, IPPROTO_IPV6, IPV6_PKTINFO, 
//ScenSim-Port//		               &on, sizeof(on)) != 0) {
//ScenSim-Port//			log_fatal("setsockopt: IPV6_PKTINFO: %m");
//ScenSim-Port//		}
#endif
	}

	if ((family == AF_INET6) &&
	    ((info->flags & INTERFACE_UPSTREAM) != 0)) {
		int hop_limit = 32;
//ScenSim-Port//		if (setsockopt(sock, IPPROTO_IPV6, IPV6_MULTICAST_HOPS,
//ScenSim-Port//			       &hop_limit, sizeof(int)) < 0) {
//ScenSim-Port//			log_fatal("setsockopt: IPV6_MULTICAST_HOPS: %m");
//ScenSim-Port//		}
	}
#endif /* DHCPv6 */

	return sock;
}
#endif /* USE_SOCKET_SEND || USE_SOCKET_RECEIVE || USE_SOCKET_FALLBACK */

#if defined (USE_SOCKET_SEND) || defined (USE_SOCKET_FALLBACK)
//ScenSim-Port//void if_register_send (info)
//ScenSim-Port//	struct interface_info *info;
void if_register_send (struct interface_info *info)//ScenSim-Port//
{
#ifndef USE_SOCKET_RECEIVE
	info->wfdesc = if_register_socket(info, AF_INET, 0);
	/* If this is a normal IPv4 address, get the hardware address. */
	if (strcmp(info->name, "fallback") != 0)
		get_hw_addr(info->name, &info->hw_address);
#if defined (USE_SOCKET_FALLBACK)
	/* Fallback only registers for send, but may need to receive as
	   well. */
	info->rfdesc = info->wfdesc;
#endif
#else
	info->wfdesc = info->rfdesc;
#endif
	if (!quiet_interface_discovery)
		log_info ("Sending on   Socket/%s%s%s",
		      info->name,
		      (info->shared_network ? "/" : ""),
		      (info->shared_network ?
		       info->shared_network->name : ""));
}

#if defined (USE_SOCKET_SEND)
//ScenSim-Port//void if_deregister_send (info)
//ScenSim-Port//	struct interface_info *info;
void if_deregister_send (struct interface_info *info)//ScenSim-Port//
{
#ifndef USE_SOCKET_RECEIVE
	close (info -> wfdesc);
#endif
	info -> wfdesc = -1;

	if (!quiet_interface_discovery)
		log_info ("Disabling output on Socket/%s%s%s",
		      info -> name,
		      (info -> shared_network ? "/" : ""),
		      (info -> shared_network ?
		       info -> shared_network -> name : ""));
}
#endif /* USE_SOCKET_SEND */
#endif /* USE_SOCKET_SEND || USE_SOCKET_FALLBACK */

#ifdef USE_SOCKET_RECEIVE
//ScenSim-Port//void if_register_receive (info)
//ScenSim-Port//	struct interface_info *info;
void if_register_receive (struct interface_info *info)//ScenSim-Port//
{

#if defined(IP_PKTINFO) && defined(IP_RECVPKTINFO) && defined(USE_V4_PKTINFO)
	if (global_v4_socket_references == 0) {
		global_v4_socket = if_register_socket(info, AF_INET, 0);
		if (global_v4_socket < 0) {
			/*
			 * if_register_socket() fatally logs if it fails to
			 * create a socket, this is just a sanity check.
			 */
			log_fatal("Failed to create AF_INET socket %s:%d",
				  MDL);
		}
	}
		
	info->rfdesc = global_v4_socket;
	global_v4_socket_references++;
#else
	/* If we're using the socket API for sending and receiving,
	   we don't need to register this interface twice. */
	info->rfdesc = if_register_socket(info, AF_INET, 0);
#endif /* IP_PKTINFO... */
	/* If this is a normal IPv4 address, get the hardware address. */
	if (strcmp(info->name, "fallback") != 0)
		get_hw_addr(info->name, &info->hw_address);

	if (!quiet_interface_discovery)
		log_info ("Listening on Socket/%s%s%s",
		      info->name,
		      (info->shared_network ? "/" : ""),
		      (info->shared_network ?
		       info->shared_network->name : ""));
}

//ScenSim-Port//void if_deregister_receive (info)
//ScenSim-Port//	struct interface_info *info;
void if_deregister_receive (struct interface_info *info)//ScenSim-Port//
{
#if defined(IP_PKTINFO) && defined(IP_RECVPKTINFO) && defined(USE_V4_PKTINFO)
	/* Dereference the global v4 socket. */
	if ((info->rfdesc == global_v4_socket) &&
	    (info->wfdesc == global_v4_socket) &&
	    (global_v4_socket_references > 0)) {
		global_v4_socket_references--;
		info->rfdesc = -1;
	} else {
		log_fatal("Impossible condition at %s:%d", MDL);
	}

	if (global_v4_socket_references == 0) {
		close(global_v4_socket);
		global_v4_socket = -1;
	}
#else
	close(info->rfdesc);
	info->rfdesc = -1;
#endif /* IP_PKTINFO... */
	if (!quiet_interface_discovery)
		log_info ("Disabling input on Socket/%s%s%s",
		      info -> name,
		      (info -> shared_network ? "/" : ""),
		      (info -> shared_network ?
		       info -> shared_network -> name : ""));
}
#endif /* USE_SOCKET_RECEIVE */


#ifdef DHCPv6 
/*
 * This function joins the interface to DHCPv6 multicast groups so we will
 * receive multicast messages.
 */
static void
if_register_multicast(struct interface_info *info) {
//ScenSim-Port//	int sock = info->rfdesc;
//ScenSim-Port//	struct ipv6_mreq mreq;
//ScenSim-Port//
//ScenSim-Port//	if (inet_pton(AF_INET6, All_DHCP_Relay_Agents_and_Servers,
//ScenSim-Port//		      &mreq.ipv6mr_multiaddr) <= 0) {
//ScenSim-Port//		log_fatal("inet_pton: unable to convert '%s'", 
//ScenSim-Port//			  All_DHCP_Relay_Agents_and_Servers);
//ScenSim-Port//	}
//ScenSim-Port//	mreq.ipv6mr_interface = if_nametoindex(info->name);
//ScenSim-Port//	if (setsockopt(sock, IPPROTO_IPV6, IPV6_JOIN_GROUP, 
//ScenSim-Port//		       &mreq, sizeof(mreq)) < 0) {
//ScenSim-Port//		log_fatal("setsockopt: IPV6_JOIN_GROUP: %m");
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * The relay agent code sets the streams so you know which way
//ScenSim-Port//	 * is up and down.  But a relay agent shouldn't join to the
//ScenSim-Port//	 * Server address, or else you get fun loops.  So up or down
//ScenSim-Port//	 * doesn't matter, we're just using that config to sense this is
//ScenSim-Port//	 * a relay agent.
//ScenSim-Port//	 */
//ScenSim-Port//	if ((info->flags & INTERFACE_STREAMS) == 0) {
//ScenSim-Port//		if (inet_pton(AF_INET6, All_DHCP_Servers,
//ScenSim-Port//			      &mreq.ipv6mr_multiaddr) <= 0) {
//ScenSim-Port//			log_fatal("inet_pton: unable to convert '%s'", 
//ScenSim-Port//				  All_DHCP_Servers);
//ScenSim-Port//		}
//ScenSim-Port//		mreq.ipv6mr_interface = if_nametoindex(info->name);
//ScenSim-Port//		if (setsockopt(sock, IPPROTO_IPV6, IPV6_JOIN_GROUP, 
//ScenSim-Port//			       &mreq, sizeof(mreq)) < 0) {
//ScenSim-Port//			log_fatal("setsockopt: IPV6_JOIN_GROUP: %m");
//ScenSim-Port//		}
//ScenSim-Port//	}
}

void
if_register6(struct interface_info *info, int do_multicast) {
//ScenSim-Port//	/* Bounce do_multicast to a stack variable because we may change it. */
//ScenSim-Port//	int req_multi = do_multicast;
//ScenSim-Port//
//ScenSim-Port//	if (global_v6_socket_references == 0) {
//ScenSim-Port//		global_v6_socket = if_register_socket(info, AF_INET6,
//ScenSim-Port//						      &req_multi);
//ScenSim-Port//		if (global_v6_socket < 0) {
//ScenSim-Port//			/*
//ScenSim-Port//			 * if_register_socket() fatally logs if it fails to
//ScenSim-Port//			 * create a socket, this is just a sanity check.
//ScenSim-Port//			 */
//ScenSim-Port//			log_fatal("Impossible condition at %s:%d", MDL);
//ScenSim-Port//		} else {
//ScenSim-Port//			log_info("Bound to *:%d", ntohs(local_port));
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//		
//ScenSim-Port//	info->rfdesc = global_v6_socket;
//ScenSim-Port//	info->wfdesc = global_v6_socket;
//ScenSim-Port//	global_v6_socket_references++;
//ScenSim-Port//
//ScenSim-Port//	if (req_multi)
//ScenSim-Port//		if_register_multicast(info);
//ScenSim-Port//
//ScenSim-Port//	get_hw_addr(info->name, &info->hw_address);
//ScenSim-Port//
//ScenSim-Port//	if (!quiet_interface_discovery) {
//ScenSim-Port//		if (info->shared_network != NULL) {
//ScenSim-Port//			log_info("Listening on Socket/%d/%s/%s",
//ScenSim-Port//				 global_v6_socket, info->name, 
//ScenSim-Port//				 info->shared_network->name);
//ScenSim-Port//			log_info("Sending on   Socket/%d/%s/%s",
//ScenSim-Port//				 global_v6_socket, info->name,
//ScenSim-Port//				 info->shared_network->name);
//ScenSim-Port//		} else {
//ScenSim-Port//			log_info("Listening on Socket/%s", info->name);
//ScenSim-Port//			log_info("Sending on   Socket/%s", info->name);
//ScenSim-Port//		}
//ScenSim-Port//	}
}

void 
if_deregister6(struct interface_info *info) {
//ScenSim-Port//	/* Dereference the global v6 socket. */
//ScenSim-Port//	if ((info->rfdesc == global_v6_socket) &&
//ScenSim-Port//	    (info->wfdesc == global_v6_socket) &&
//ScenSim-Port//	    (global_v6_socket_references > 0)) {
//ScenSim-Port//		global_v6_socket_references--;
//ScenSim-Port//		info->rfdesc = -1;
//ScenSim-Port//		info->wfdesc = -1;
//ScenSim-Port//	} else {
//ScenSim-Port//		log_fatal("Impossible condition at %s:%d", MDL);
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (!quiet_interface_discovery) {
//ScenSim-Port//		if (info->shared_network != NULL) {
//ScenSim-Port//			log_info("Disabling input on  Socket/%s/%s", info->name,
//ScenSim-Port//		       		 info->shared_network->name);
//ScenSim-Port//			log_info("Disabling output on Socket/%s/%s", info->name,
//ScenSim-Port//		       		 info->shared_network->name);
//ScenSim-Port//		} else {
//ScenSim-Port//			log_info("Disabling input on  Socket/%s", info->name);
//ScenSim-Port//			log_info("Disabling output on Socket/%s", info->name);
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (global_v6_socket_references == 0) {
//ScenSim-Port//		close(global_v6_socket);
//ScenSim-Port//		global_v6_socket = -1;
//ScenSim-Port//
//ScenSim-Port//		log_info("Unbound from *:%d", ntohs(local_port));
//ScenSim-Port//	}
}
#endif /* DHCPv6 */

//ScenSim-Port//#if defined (USE_SOCKET_SEND) || defined (USE_SOCKET_FALLBACK)
//ScenSim-Port//ssize_t send_packet (interface, packet, raw, len, from, to, hto)
//ScenSim-Port//	struct interface_info *interface;
//ScenSim-Port//	struct packet *packet;
//ScenSim-Port//	struct dhcp_packet *raw;
//ScenSim-Port//	size_t len;
//ScenSim-Port//	struct in_addr from;
//ScenSim-Port//	struct sockaddr_in *to;
//ScenSim-Port//	struct hardware *hto;
ssize_t send_packet (struct interface_info *interface, struct packet *packet, struct dhcp_packet *raw, size_t len, struct in_addr from, struct sockaddr_in *to, struct hardware *hto)//ScenSim-Port//
{
//ScenSim-Port//	int result;
//ScenSim-Port//#ifdef IGNORE_HOSTUNREACH
//ScenSim-Port//	int retry = 0;
//ScenSim-Port//	do {
//ScenSim-Port//#endif
//ScenSim-Port//#if defined(IP_PKTINFO) && defined(IP_RECVPKTINFO) && defined(USE_V4_PKTINFO)
//ScenSim-Port//		struct in_pktinfo pktinfo;
//ScenSim-Port//
//ScenSim-Port//		if (interface->ifp != NULL) {
//ScenSim-Port//			memset(&pktinfo, 0, sizeof (pktinfo));
//ScenSim-Port//			pktinfo.ipi_ifindex = interface->ifp->ifr_index;
//ScenSim-Port//			if (setsockopt(interface->wfdesc, IPPROTO_IP,
//ScenSim-Port//				       IP_PKTINFO, (char *)&pktinfo,
//ScenSim-Port//				       sizeof(pktinfo)) < 0) 
//ScenSim-Port//				log_fatal("setsockopt: IP_PKTINFO: %m");
//ScenSim-Port//		}
//ScenSim-Port//#endif
//ScenSim-Port//		result = sendto (interface -> wfdesc, (char *)raw, len, 0,
//ScenSim-Port//				 (struct sockaddr *)to, sizeof *to);
//ScenSim-Port//#ifdef IGNORE_HOSTUNREACH
//ScenSim-Port//	} while (to -> sin_addr.s_addr == htonl (INADDR_BROADCAST) &&
//ScenSim-Port//		 result < 0 &&
//ScenSim-Port//		 (errno == EHOSTUNREACH ||
//ScenSim-Port//		  errno == ECONNREFUSED) &&
//ScenSim-Port//		 retry++ < 10);
//ScenSim-Port//#endif
//ScenSim-Port//	if (result < 0) {
//ScenSim-Port//		log_error ("send_packet: %m");
//ScenSim-Port//		if (errno == ENETUNREACH)
//ScenSim-Port//			log_error ("send_packet: please consult README file%s",
//ScenSim-Port//				   " regarding broadcast address.");
//ScenSim-Port//	}
//ScenSim-Port//	return result;
    assert(to->sin_family == AF_INET);//ScenSim-Port//
    uint32_t to_s_addr = to->sin_addr.s_addr;//ScenSim-Port//
    assert(remote_port == to->sin_port);//ScenSim-Port//
    SendPacket(curctx->glue, (unsigned char *)raw, len, from.s_addr, local_port, to_s_addr, remote_port);//ScenSim-Port//
    return len;//ScenSim-Port//
}

//ScenSim-Port//#endif /* USE_SOCKET_SEND || USE_SOCKET_FALLBACK */

//ScenSim-Port//#ifdef DHCPv6
//ScenSim-Port///*
//ScenSim-Port// * Solaris 9 is missing the CMSG_LEN and CMSG_SPACE macros, so we will 
//ScenSim-Port// * synthesize them (based on the BIND 9 technique).
//ScenSim-Port// */
//ScenSim-Port//
//ScenSim-Port//#ifndef CMSG_LEN
//ScenSim-Port//static size_t CMSG_LEN(size_t len) {
//ScenSim-Port//	size_t hdrlen;
//ScenSim-Port//	/*
//ScenSim-Port//	 * Cast NULL so that any pointer arithmetic performed by CMSG_DATA
//ScenSim-Port//	 * is correct.
//ScenSim-Port//	 */
//ScenSim-Port//	hdrlen = (size_t)CMSG_DATA(((struct cmsghdr *)NULL));
//ScenSim-Port//	return hdrlen + len;
//ScenSim-Port//}
//ScenSim-Port//#endif /* !CMSG_LEN */
//ScenSim-Port//
//ScenSim-Port//#ifndef CMSG_SPACE
//ScenSim-Port//static size_t CMSG_SPACE(size_t len) {
//ScenSim-Port//	struct msghdr msg;
//ScenSim-Port//	struct cmsghdr *cmsgp;
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * XXX: The buffer length is an ad-hoc value, but should be enough
//ScenSim-Port//	 * in a practical sense.
//ScenSim-Port//	 */
//ScenSim-Port//	union {
//ScenSim-Port//		struct cmsghdr cmsg_sizer;
//ScenSim-Port//		u_int8_t pktinfo_sizer[sizeof(struct cmsghdr) + 1024];
//ScenSim-Port//	} dummybuf;
//ScenSim-Port//
//ScenSim-Port//	memset(&msg, 0, sizeof(msg));
//ScenSim-Port//	msg.msg_control = &dummybuf;
//ScenSim-Port//	msg.msg_controllen = sizeof(dummybuf);
//ScenSim-Port//
//ScenSim-Port//	cmsgp = (struct cmsghdr *)&dummybuf;
//ScenSim-Port//	cmsgp->cmsg_len = CMSG_LEN(len);
//ScenSim-Port//
//ScenSim-Port//	cmsgp = CMSG_NXTHDR(&msg, cmsgp);
//ScenSim-Port//	if (cmsgp != NULL) {
//ScenSim-Port//		return (char *)cmsgp - (char *)msg.msg_control;
//ScenSim-Port//	} else {
//ScenSim-Port//		return 0;
//ScenSim-Port//	}
//ScenSim-Port//}
//ScenSim-Port//#endif /* !CMSG_SPACE */
//ScenSim-Port//
//ScenSim-Port//#endif /* DHCPv6 */

//ScenSim-Port//#if defined(DHCPv6) || \
//ScenSim-Port//	(defined(IP_PKTINFO) && defined(IP_RECVPKTINFO) && \
//ScenSim-Port//	 defined(USE_V4_PKTINFO))
//ScenSim-Port///*
//ScenSim-Port// * For both send_packet6() and receive_packet6() we need to allocate
//ScenSim-Port// * space for the cmsg header information.  We do this once and reuse
//ScenSim-Port// * the buffer.  We also need the control buf for send_packet() and
//ScenSim-Port// * receive_packet() when we use a single socket and IP_PKTINFO to
//ScenSim-Port// * send the packet out the correct interface.
//ScenSim-Port// */
//ScenSim-Port//static void   *control_buf = NULL;
//ScenSim-Port//static size_t  control_buf_len = 0;
//ScenSim-Port//
//ScenSim-Port//static void
//ScenSim-Port//allocate_cmsg_cbuf(void) {
//ScenSim-Port//	control_buf_len = CMSG_SPACE(sizeof(struct in6_pktinfo));
//ScenSim-Port//	control_buf = dmalloc(control_buf_len, MDL);
//ScenSim-Port//	return;
//ScenSim-Port//}
//ScenSim-Port//#endif /* DHCPv6, IP_PKTINFO ... */

#ifdef DHCPv6
/* 
 * For both send_packet6() and receive_packet6() we need to use the 
 * sendmsg()/recvmsg() functions rather than the simpler send()/recv()
 * functions.
 *
 * In the case of send_packet6(), we need to do this in order to insure
 * that the reply packet leaves on the same interface that it arrived 
 * on. 
 *
 * In the case of receive_packet6(), we need to do this in order to 
 * get the IP address the packet was sent to. This is used to identify
 * whether a packet is multicast or unicast.
 *
 * Helpful man pages: recvmsg, readv (talks about the iovec stuff), cmsg.
 *
 * Also see the sections in RFC 3542 about IPV6_PKTINFO.
 */

/* Send an IPv6 packet */
ssize_t send_packet6(struct interface_info *interface,
		     const unsigned char *raw, size_t len,
		     struct sockaddr_in6 *to) {
//ScenSim-Port//	struct msghdr m;
//ScenSim-Port//	struct iovec v;
//ScenSim-Port//	int result;
//ScenSim-Port//	struct in6_pktinfo *pktinfo;
//ScenSim-Port//	struct cmsghdr *cmsg;
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * If necessary allocate space for the control message header.
//ScenSim-Port//	 * The space is common between send and receive.
//ScenSim-Port//	 */
//ScenSim-Port//
//ScenSim-Port//	if (control_buf == NULL) {
//ScenSim-Port//		allocate_cmsg_cbuf();
//ScenSim-Port//		if (control_buf == NULL) {
//ScenSim-Port//			log_error("send_packet6: unable to allocate cmsg header");
//ScenSim-Port//			return(ENOMEM);
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//	memset(control_buf, 0, control_buf_len);
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * Initialize our message header structure.
//ScenSim-Port//	 */
//ScenSim-Port//	memset(&m, 0, sizeof(m));
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * Set the target address we're sending to.
//ScenSim-Port//	 */
//ScenSim-Port//	m.msg_name = to;
//ScenSim-Port//	m.msg_namelen = sizeof(*to);
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * Set the data buffer we're sending. (Using this wacky 
//ScenSim-Port//	 * "scatter-gather" stuff... we only have a single chunk 
//ScenSim-Port//	 * of data to send, so we declare a single vector entry.)
//ScenSim-Port//	 */
//ScenSim-Port//	v.iov_base = (char *)raw;
//ScenSim-Port//	v.iov_len = len;
//ScenSim-Port//	m.msg_iov = &v;
//ScenSim-Port//	m.msg_iovlen = 1;
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * Setting the interface is a bit more involved.
//ScenSim-Port//	 * 
//ScenSim-Port//	 * We have to create a "control message", and set that to 
//ScenSim-Port//	 * define the IPv6 packet information. We could set the
//ScenSim-Port//	 * source address if we wanted, but we can safely let the
//ScenSim-Port//	 * kernel decide what that should be. 
//ScenSim-Port//	 */
//ScenSim-Port//	m.msg_control = control_buf;
//ScenSim-Port//	m.msg_controllen = control_buf_len;
//ScenSim-Port//	cmsg = CMSG_FIRSTHDR(&m);
//ScenSim-Port//	cmsg->cmsg_level = IPPROTO_IPV6;
//ScenSim-Port//	cmsg->cmsg_type = IPV6_PKTINFO;
//ScenSim-Port//	cmsg->cmsg_len = CMSG_LEN(sizeof(*pktinfo));
//ScenSim-Port//	pktinfo = (struct in6_pktinfo *)CMSG_DATA(cmsg);
//ScenSim-Port//	memset(pktinfo, 0, sizeof(*pktinfo));
//ScenSim-Port//	pktinfo->ipi6_ifindex = if_nametoindex(interface->name);
//ScenSim-Port//	m.msg_controllen = cmsg->cmsg_len;
//ScenSim-Port//
//ScenSim-Port//	result = sendmsg(interface->wfdesc, &m, 0);
//ScenSim-Port//	if (result < 0) {
//ScenSim-Port//		log_error("send_packet6: %m");
//ScenSim-Port//	}
//ScenSim-Port//	return result;
    assert(to->sin6_family == AF_INET6);//ScenSim-Port//
    uint8_t *to_s_addr = to->sin6_addr.s6_addr;//ScenSim-Port//
    assert(remote_port == to->sin6_port);//ScenSim-Port//
    SendPacket6(curctx->glue, (unsigned char *)raw, len, interface->v6addresses[0].s6_addr, local_port, to_s_addr, remote_port);//ScenSim-Port//
    return len;//ScenSim-Port//
}
#endif /* DHCPv6 */

#ifdef USE_SOCKET_RECEIVE
//ScenSim-Port//ssize_t receive_packet (interface, buf, len, from, hfrom)
//ScenSim-Port//	struct interface_info *interface;
//ScenSim-Port//	unsigned char *buf;
//ScenSim-Port//	size_t len;
//ScenSim-Port//	struct sockaddr_in *from;
//ScenSim-Port//	struct hardware *hfrom;
ssize_t receive_packet (struct interface_info *interface, unsigned char *buf, size_t len, struct sockaddr_in *from, struct hardware *hfrom)//ScenSim-Port//
{
#if !(defined(IP_PKTINFO) && defined(IP_RECVPKTINFO) && defined(USE_V4_PKTINFO))
//ScenSim-Port//	SOCKLEN_T flen = sizeof *from;
	size_t flen = sizeof *from;//ScenSim-Port//
#endif
	int result;

	/*
	 * The normal Berkeley socket interface doesn't give us any way
	 * to know what hardware interface we received the message on,
	 * but we should at least make sure the structure is emptied.
	 */
	memset(hfrom, 0, sizeof(*hfrom));

#ifdef IGNORE_HOSTUNREACH
	int retry = 0;
	do {
#endif

#if defined(IP_PKTINFO) && defined(IP_RECVPKTINFO) && defined(USE_V4_PKTINFO)
	struct msghdr m;
	struct iovec v;
	struct cmsghdr *cmsg;
	struct in_pktinfo *pktinfo;
	unsigned int ifindex;

	/*
	 * If necessary allocate space for the control message header.
	 * The space is common between send and receive.
	 */
	if (control_buf == NULL) {
		allocate_cmsg_cbuf();
		if (control_buf == NULL) {
			log_error("receive_packet: unable to allocate cmsg "
				  "header");
			return(ENOMEM);
		}
	}
	memset(control_buf, 0, control_buf_len);

	/*
	 * Initialize our message header structure.
	 */
	memset(&m, 0, sizeof(m));

	/*
	 * Point so we can get the from address.
	 */
	m.msg_name = from;
	m.msg_namelen = sizeof(*from);

	/*
	 * Set the data buffer we're receiving. (Using this wacky 
	 * "scatter-gather" stuff... but we that doesn't really make
	 * sense for us, so we use a single vector entry.)
	 */
	v.iov_base = buf;
	v.iov_len = len;
	m.msg_iov = &v;
	m.msg_iovlen = 1;

	/*
	 * Getting the interface is a bit more involved.
	 *
	 * We set up some space for a "control message". We have 
	 * previously asked the kernel to give us packet 
	 * information (when we initialized the interface), so we
	 * should get the interface index from that.
	 */
	m.msg_control = control_buf;
	m.msg_controllen = control_buf_len;

	result = recvmsg(interface->rfdesc, &m, 0);

	if (result >= 0) {
		/*
		 * If we did read successfully, then we need to loop
		 * through the control messages we received and 
		 * find the one with our inteface index.
		 */
		cmsg = CMSG_FIRSTHDR(&m);
		while (cmsg != NULL) {
			if ((cmsg->cmsg_level == IPPROTO_IP) && 
			    (cmsg->cmsg_type == IP_PKTINFO)) {
				pktinfo = (struct in_pktinfo *)CMSG_DATA(cmsg);
				ifindex = pktinfo->ipi_ifindex;
				/*
				 * We pass the ifindex back to the caller 
				 * using the unused hfrom parameter avoiding
				 * interface changes between sockets and 
				 * the discover code.
				 */
				memcpy(hfrom->hbuf, &ifindex, sizeof(ifindex));
				return (result);
			}
			cmsg = CMSG_NXTHDR(&m, cmsg);
		}

		/*
		 * We didn't find the necessary control message
		 * flag it as an error
		 */
		result = -1;
		errno = EIO;
	}
#else
//ScenSim-Port//		result = recvfrom(interface -> rfdesc, (char *)buf, len, 0,
//ScenSim-Port//				  (struct sockaddr *)from, &flen);
#endif /* IP_PKTINFO ... */
#ifdef IGNORE_HOSTUNREACH
	} while (result < 0 &&
		 (errno == EHOSTUNREACH ||
		  errno == ECONNREFUSED) &&
		 retry++ < 10);
#endif
	return (result);
}

#endif /* USE_SOCKET_RECEIVE */

#ifdef DHCPv6
ssize_t 
receive_packet6(struct interface_info *interface, 
		unsigned char *buf, size_t len, 
		struct sockaddr_in6 *from, struct in6_addr *to_addr,
		unsigned int *if_idx)
{
//ScenSim-Port//	struct msghdr m;
//ScenSim-Port//	struct iovec v;
//ScenSim-Port//	int result;
//ScenSim-Port//	struct cmsghdr *cmsg;
//ScenSim-Port//	struct in6_pktinfo *pktinfo;
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * If necessary allocate space for the control message header.
//ScenSim-Port//	 * The space is common between send and receive.
//ScenSim-Port//	 */
//ScenSim-Port//	if (control_buf == NULL) {
//ScenSim-Port//		allocate_cmsg_cbuf();
//ScenSim-Port//		if (control_buf == NULL) {
//ScenSim-Port//			log_error("receive_packet6: unable to allocate cmsg "
//ScenSim-Port//				  "header");
//ScenSim-Port//			return(ENOMEM);
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//	memset(control_buf, 0, control_buf_len);
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * Initialize our message header structure.
//ScenSim-Port//	 */
//ScenSim-Port//	memset(&m, 0, sizeof(m));
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * Point so we can get the from address.
//ScenSim-Port//	 */
//ScenSim-Port//	m.msg_name = from;
//ScenSim-Port//	m.msg_namelen = sizeof(*from);
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * Set the data buffer we're receiving. (Using this wacky 
//ScenSim-Port//	 * "scatter-gather" stuff... but we that doesn't really make
//ScenSim-Port//	 * sense for us, so we use a single vector entry.)
//ScenSim-Port//	 */
//ScenSim-Port//	v.iov_base = buf;
//ScenSim-Port//	v.iov_len = len;
//ScenSim-Port//	m.msg_iov = &v;
//ScenSim-Port//	m.msg_iovlen = 1;
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * Getting the interface is a bit more involved.
//ScenSim-Port//	 *
//ScenSim-Port//	 * We set up some space for a "control message". We have 
//ScenSim-Port//	 * previously asked the kernel to give us packet 
//ScenSim-Port//	 * information (when we initialized the interface), so we
//ScenSim-Port//	 * should get the destination address from that.
//ScenSim-Port//	 */
//ScenSim-Port//	m.msg_control = control_buf;
//ScenSim-Port//	m.msg_controllen = control_buf_len;
//ScenSim-Port//
//ScenSim-Port//	result = recvmsg(interface->rfdesc, &m, 0);
//ScenSim-Port//
//ScenSim-Port//	if (result >= 0) {
//ScenSim-Port//		/*
//ScenSim-Port//		 * If we did read successfully, then we need to loop
//ScenSim-Port//		 * through the control messages we received and 
//ScenSim-Port//		 * find the one with our destination address.
//ScenSim-Port//		 */
//ScenSim-Port//		cmsg = CMSG_FIRSTHDR(&m);
//ScenSim-Port//		while (cmsg != NULL) {
//ScenSim-Port//			if ((cmsg->cmsg_level == IPPROTO_IPV6) && 
//ScenSim-Port//			    (cmsg->cmsg_type == IPV6_PKTINFO)) {
//ScenSim-Port//				pktinfo = (struct in6_pktinfo *)CMSG_DATA(cmsg);
//ScenSim-Port//				*to_addr = pktinfo->ipi6_addr;
//ScenSim-Port//				*if_idx = pktinfo->ipi6_ifindex;
//ScenSim-Port//
//ScenSim-Port//				return (result);
//ScenSim-Port//			}
//ScenSim-Port//			cmsg = CMSG_NXTHDR(&m, cmsg);
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//		/*
//ScenSim-Port//		 * We didn't find the necessary control message
//ScenSim-Port//		 * flag is as an error
//ScenSim-Port//		 */
//ScenSim-Port//		result = -1;
//ScenSim-Port//		errno = EIO;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	return (result);
    return ISC_R_SUCCESS;//ScenSim-Port//
}
#endif /* DHCPv6 */

//ScenSim-Port//#if defined (USE_SOCKET_FALLBACK)
//ScenSim-Port///* This just reads in a packet and silently discards it. */
//ScenSim-Port//
//ScenSim-Port//isc_result_t fallback_discard (object)
//ScenSim-Port//	omapi_object_t *object;
//ScenSim-Port//{
//ScenSim-Port//	char buf [1540];
//ScenSim-Port//	struct sockaddr_in from;
//ScenSim-Port//	SOCKLEN_T flen = sizeof from;
//ScenSim-Port//	int status;
//ScenSim-Port//	struct interface_info *interface;
//ScenSim-Port//
//ScenSim-Port//	if (object -> type != dhcp_type_interface)
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//	interface = (struct interface_info *)object;
//ScenSim-Port//
//ScenSim-Port//	status = recvfrom (interface -> wfdesc, buf, sizeof buf, 0,
//ScenSim-Port//			   (struct sockaddr *)&from, &flen);
//ScenSim-Port//#if defined (DEBUG)
//ScenSim-Port//	/* Only report fallback discard errors if we're debugging. */
//ScenSim-Port//	if (status < 0) {
//ScenSim-Port//		log_error ("fallback_discard: %m");
//ScenSim-Port//		return ISC_R_UNEXPECTED;
//ScenSim-Port//	}
//ScenSim-Port//#else
//ScenSim-Port//        /* ignore the fact that status value is never used */
//ScenSim-Port//        IGNORE_UNUSED(status);
//ScenSim-Port//#endif
//ScenSim-Port//	return ISC_R_SUCCESS;
//ScenSim-Port//}
//ScenSim-Port//#endif /* USE_SOCKET_FALLBACK */

//ScenSim-Port//#if defined (USE_SOCKET_SEND)
//ScenSim-Port//int can_unicast_without_arp (ip)
//ScenSim-Port//	struct interface_info *ip;
int can_unicast_without_arp (struct interface_info *ip)//ScenSim-Port//
{
	return 0;
}

//ScenSim-Port//int can_receive_unicast_unconfigured (ip)
//ScenSim-Port//	struct interface_info *ip;
int can_receive_unicast_unconfigured (struct interface_info *ip)//ScenSim-Port//
{
//ScenSim-Port//#if defined (SOCKET_CAN_RECEIVE_UNICAST_UNCONFIGURED)
//ScenSim-Port//	return 1;
//ScenSim-Port//#else
	return 0;
//ScenSim-Port//#endif
}

//ScenSim-Port//int supports_multiple_interfaces (ip)
//ScenSim-Port//	struct interface_info *ip;
int supports_multiple_interfaces (struct interface_info *ip)//ScenSim-Port//
{
#if defined(SO_BINDTODEVICE) || \
	(defined(IP_PKTINFO) && defined(IP_RECVPKTINFO) && \
	 defined(USE_V4_PKTINFO))
	return(1);
#else
	return(0);
#endif
}

/* If we have SO_BINDTODEVICE, set up a fallback interface; otherwise,
   do not. */

void maybe_setup_fallback ()
{
#if defined (USE_SOCKET_FALLBACK)
	isc_result_t status;
	struct interface_info *fbi = (struct interface_info *)0;
	if (setup_fallback (&fbi, MDL)) {
		fbi -> wfdesc = if_register_socket (fbi, AF_INET, 0);
		fbi -> rfdesc = fbi -> wfdesc;
		log_info ("Sending on   Socket/%s%s%s",
		      fbi -> name,
		      (fbi -> shared_network ? "/" : ""),
		      (fbi -> shared_network ?
		       fbi -> shared_network -> name : ""));
	
		status = omapi_register_io_object ((omapi_object_t *)fbi,
						   if_readsocket, 0,
						   fallback_discard, 0, 0);
		if (status != ISC_R_SUCCESS)
			log_fatal ("Can't register I/O handle for %s: %s",
				   fbi -> name, isc_result_totext (status));
		interface_dereference (&fbi, MDL);
	}
#endif
}


//ScenSim-Port//#if defined(sun) && defined(USE_V4_PKTINFO)
//ScenSim-Port///* This code assumes the existence of SIOCGLIFHWADDR */
void
get_hw_addr(const char *name, struct hardware *hw) {
//ScenSim-Port//	struct sockaddr_dl *dladdrp;
//ScenSim-Port//	int sock, i;
//ScenSim-Port//	struct lifreq lifr;
//ScenSim-Port//
//ScenSim-Port//	memset(&lifr, 0, sizeof (lifr));
//ScenSim-Port//	(void) strlcpy(lifr.lifr_name, name, sizeof (lifr.lifr_name));
//ScenSim-Port//	/*
//ScenSim-Port//	 * Check if the interface is a virtual or IPMP interface - in those
//ScenSim-Port//	 * cases it has no hw address, so generate a random one.
//ScenSim-Port//	 */
//ScenSim-Port//	if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ||
//ScenSim-Port//	    ioctl(sock, SIOCGLIFFLAGS, &lifr) < 0) {
//ScenSim-Port//		if (sock != -1)
//ScenSim-Port//			(void) close(sock);
//ScenSim-Port//
//ScenSim-Port//#ifdef DHCPv6
//ScenSim-Port//		/*
//ScenSim-Port//		 * If approrpriate try this with an IPv6 socket
//ScenSim-Port//		 */
//ScenSim-Port//		if ((sock = socket(AF_INET6, SOCK_DGRAM, 0)) >= 0 &&
//ScenSim-Port//		    ioctl(sock, SIOCGLIFFLAGS, &lifr) >= 0) {
//ScenSim-Port//			goto flag_check;
//ScenSim-Port//		}
//ScenSim-Port//		if (sock != -1)
//ScenSim-Port//			(void) close(sock);
//ScenSim-Port//#endif
//ScenSim-Port//		log_fatal("Couldn't get interface flags for %s: %m", name);
//ScenSim-Port//
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port// flag_check:
//ScenSim-Port//	if (lifr.lifr_flags & (IFF_VIRTUAL|IFF_IPMP)) {
//ScenSim-Port//		hw->hlen = sizeof (hw->hbuf);
//ScenSim-Port//		srandom((long)gethrtime());
//ScenSim-Port//
//ScenSim-Port//		hw->hbuf[0] = HTYPE_IPMP;
//ScenSim-Port//		for (i = 1; i < hw->hlen; ++i) {
//ScenSim-Port//			hw->hbuf[i] = random() % 256;
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//		if (sock != -1)
//ScenSim-Port//			(void) close(sock);
//ScenSim-Port//		return;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (ioctl(sock, SIOCGLIFHWADDR, &lifr) < 0)
//ScenSim-Port//		log_fatal("Couldn't get interface hardware address for %s: %m",
//ScenSim-Port//			  name);
//ScenSim-Port//	dladdrp = (struct sockaddr_dl *)&lifr.lifr_addr;
//ScenSim-Port//	hw->hlen = dladdrp->sdl_alen+1;
//ScenSim-Port//	switch (dladdrp->sdl_type) {
//ScenSim-Port//		case DL_CSMACD: /* IEEE 802.3 */
//ScenSim-Port//		case DL_ETHER:
//ScenSim-Port//			hw->hbuf[0] = HTYPE_ETHER;
//ScenSim-Port//			break;
//ScenSim-Port//		case DL_TPR:
//ScenSim-Port//			hw->hbuf[0] = HTYPE_IEEE802;
//ScenSim-Port//			break;
//ScenSim-Port//		case DL_FDDI:
//ScenSim-Port//			hw->hbuf[0] = HTYPE_FDDI;
//ScenSim-Port//			break;
//ScenSim-Port//		case DL_IB:
//ScenSim-Port//			hw->hbuf[0] = HTYPE_INFINIBAND;
//ScenSim-Port//			break;
//ScenSim-Port//		default:
//ScenSim-Port//			log_fatal("%s: unsupported DLPI MAC type %lu", name,
//ScenSim-Port//				  (unsigned long)dladdrp->sdl_type);
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	memcpy(hw->hbuf+1, LLADDR(dladdrp), hw->hlen-1);
//ScenSim-Port//
//ScenSim-Port//	if (sock != -1)
//ScenSim-Port//		(void) close(sock);
}
//ScenSim-Port//#endif /* defined(sun) */
//ScenSim-Port//
//ScenSim-Port//#endif /* USE_SOCKET_SEND */
}//namespace IscDhcpPort////ScenSim-Port//
