/* connection.c

   Subroutines for dealing with connections. */

/*
 * Copyright (c) 2009-2011 by Internet Systems Consortium, Inc. ("ISC")
 * Copyright (c) 2004,2007 by Internet Systems Consortium, Inc. ("ISC")
 * Copyright (c) 1999-2003 by Internet Software Consortium
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

//ScenSim-Port//#include "dhcpd.h"

//ScenSim-Port//#include <omapip/omapip_p.h>
//ScenSim-Port//#include <arpa/inet.h>
//ScenSim-Port//#include <arpa/nameser.h>
//ScenSim-Port//#include <errno.h>

//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//static void trace_connect_input (trace_type_t *, unsigned, char *);
//ScenSim-Port//static void trace_connect_stop (trace_type_t *);
//ScenSim-Port//static void trace_disconnect_input (trace_type_t *, unsigned, char *);
//ScenSim-Port//static void trace_disconnect_stop (trace_type_t *);
//ScenSim-Port//trace_type_t *trace_connect;
//ScenSim-Port//trace_type_t *trace_disconnect;
//ScenSim-Port//extern omapi_array_t *trace_listeners;
//ScenSim-Port//#endif
static isc_result_t omapi_connection_connect_internal (omapi_object_t *);

OMAPI_OBJECT_ALLOC (omapi_connection,
		    omapi_connection_object_t, omapi_type_connection)

isc_result_t omapi_connect (omapi_object_t *c,
			    const char *server_name,
			    unsigned port)
{
	struct hostent *he;
	unsigned i, hix;
	omapi_addr_list_t *addrs = (omapi_addr_list_t *)0;
	struct in_addr foo;
	isc_result_t status;

//ScenSim-Port//#ifdef DEBUG_PROTOCOL
//ScenSim-Port//	log_debug ("omapi_connect(%s, port=%d)", server_name, port);
//ScenSim-Port//#endif

	if (!inet_aton (server_name, &foo)) {
		/* If we didn't get a numeric address, try for a domain
		   name.  It's okay for this call to block. */
		he = gethostbyname (server_name);
		if (!he)
			return DHCP_R_HOSTUNKNOWN;
		for (i = 0; he -> h_addr_list [i]; i++)
			;
		if (i == 0)
			return DHCP_R_HOSTUNKNOWN;
		hix = i;

		status = omapi_addr_list_new (&addrs, hix, MDL);
		if (status != ISC_R_SUCCESS)
			return status;
		for (i = 0; i < hix; i++) {
			addrs -> addresses [i].addrtype = he -> h_addrtype;
			addrs -> addresses [i].addrlen = he -> h_length;
			memcpy (addrs -> addresses [i].address,
				he -> h_addr_list [i],
				(unsigned)he -> h_length);
			addrs -> addresses [i].port = port;
		}
	} else {
		status = omapi_addr_list_new (&addrs, 1, MDL);
		if (status != ISC_R_SUCCESS)
			return status;
		addrs -> addresses [0].addrtype = AF_INET;
		addrs -> addresses [0].addrlen = sizeof foo;
		memcpy (addrs -> addresses [0].address, &foo, sizeof foo);
		addrs -> addresses [0].port = port;
		hix = 1;
	}
	status = omapi_connect_list (c, addrs, (omapi_addr_t *)0);
	omapi_addr_list_dereference (&addrs, MDL);
	return status;
}

isc_result_t omapi_connect_list (omapi_object_t *c,
				 omapi_addr_list_t *remote_addrs,
				 omapi_addr_t *local_addr)
{
	isc_result_t status;
	omapi_connection_object_t *obj;
	int flag;
	struct sockaddr_in local_sin;

	obj = (omapi_connection_object_t *)0;
	status = omapi_connection_allocate (&obj, MDL);
	if (status != ISC_R_SUCCESS)
		return status;

	status = omapi_object_reference (&c -> outer, (omapi_object_t *)obj,
					 MDL);
	if (status != ISC_R_SUCCESS) {
		omapi_connection_dereference (&obj, MDL);
		return status;
	}
	status = omapi_object_reference (&obj -> inner, c, MDL);
	if (status != ISC_R_SUCCESS) {
		omapi_connection_dereference (&obj, MDL);
		return status;
	}

	/* Store the address list on the object. */
	omapi_addr_list_reference (&obj -> connect_list, remote_addrs, MDL);
	obj -> cptr = 0;
	obj -> state = omapi_connection_unconnected;

//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//	/* If we're playing back, don't actually try to connect - just leave
//ScenSim-Port//	   the object available for a subsequent connect or disconnect. */
//ScenSim-Port//	if (!trace_playback ()) {
//ScenSim-Port//#endif
		/* Create a socket on which to communicate. */
//ScenSim-Port//		obj -> socket =
//ScenSim-Port//			socket (PF_INET, SOCK_STREAM, IPPROTO_TCP);
//ScenSim-Port//		if (obj -> socket < 0) {
//ScenSim-Port//			omapi_connection_dereference (&obj, MDL);
//ScenSim-Port//			if (errno == EMFILE || errno == ENFILE
//ScenSim-Port//			    || errno == ENOBUFS)
//ScenSim-Port//				return ISC_R_NORESOURCES;
//ScenSim-Port//			return ISC_R_UNEXPECTED;
//ScenSim-Port//		}

		/* Set up the local address, if any. */
		if (local_addr) {
			/* Only do TCPv4 so far. */
			if (local_addr -> addrtype != AF_INET) {
				omapi_connection_dereference (&obj, MDL);
				return DHCP_R_INVALIDARG;
			}
			local_sin.sin_port = htons (local_addr -> port);
			memcpy (&local_sin.sin_addr,
				local_addr -> address,
				local_addr -> addrlen);
//ScenSim-Port//#if defined (HAVE_SA_LEN)
//ScenSim-Port//			local_sin.sin_len = sizeof local_addr;
//ScenSim-Port//#endif
			local_sin.sin_family = AF_INET;
			memset (&local_sin.sin_zero, 0,
				sizeof local_sin.sin_zero);
			
//ScenSim-Port//			if (bind (obj -> socket, (struct sockaddr *)&local_sin,
//ScenSim-Port//				  sizeof local_sin) < 0) {
//ScenSim-Port//				omapi_connection_object_t **objp = &obj;
//ScenSim-Port//				omapi_object_t **o = (omapi_object_t **)objp;
//ScenSim-Port//				omapi_object_dereference(o, MDL);
//ScenSim-Port//				if (errno == EADDRINUSE)
//ScenSim-Port//					return ISC_R_ADDRINUSE;
//ScenSim-Port//				if (errno == EADDRNOTAVAIL)
//ScenSim-Port//					return ISC_R_ADDRNOTAVAIL;
//ScenSim-Port//				if (errno == EACCES)
//ScenSim-Port//					return ISC_R_NOPERM;
//ScenSim-Port//				return ISC_R_UNEXPECTED;
//ScenSim-Port//			}
			obj -> local_addr = local_sin;
		}

//ScenSim-Port//#if defined(F_SETFD)
//ScenSim-Port//		if (fcntl (obj -> socket, F_SETFD, 1) < 0) {
//ScenSim-Port//			close (obj -> socket);
//ScenSim-Port//			omapi_connection_dereference (&obj, MDL);
//ScenSim-Port//			return ISC_R_UNEXPECTED;
//ScenSim-Port//		}
//ScenSim-Port//#endif

		/* Set the SO_REUSEADDR flag (this should not fail). */
		flag = 1;
//ScenSim-Port//		if (setsockopt (obj -> socket, SOL_SOCKET, SO_REUSEADDR,
//ScenSim-Port//				(char *)&flag, sizeof flag) < 0) {
//ScenSim-Port//			omapi_connection_dereference (&obj, MDL);
//ScenSim-Port//			return ISC_R_UNEXPECTED;
//ScenSim-Port//		}
	
		/* Set the file to nonblocking mode. */
//ScenSim-Port//		if (fcntl (obj -> socket, F_SETFL, O_NONBLOCK) < 0) {
//ScenSim-Port//			omapi_connection_dereference (&obj, MDL);
//ScenSim-Port//			return ISC_R_UNEXPECTED;
//ScenSim-Port//		}

//ScenSim-Port//#ifdef SO_NOSIGPIPE
//ScenSim-Port//		/*
//ScenSim-Port//		 * If available stop the OS from killing our
//ScenSim-Port//		 * program on a SIGPIPE failure
//ScenSim-Port//		 */
//ScenSim-Port//		flag = 1;
//ScenSim-Port//		if (setsockopt(obj->socket, SOL_SOCKET, SO_NOSIGPIPE,
//ScenSim-Port//			       (char *)&flag, sizeof(flag)) < 0) {
//ScenSim-Port//			omapi_connection_dereference (&obj, MDL);
//ScenSim-Port//			return ISC_R_UNEXPECTED;
//ScenSim-Port//		}			
//ScenSim-Port//#endif

		status = (omapi_register_io_object
			  ((omapi_object_t *)obj,
			   0, omapi_connection_writefd,
			   0, omapi_connection_connect,
			   omapi_connection_reaper));
		if (status != ISC_R_SUCCESS)
			goto out;
		status = omapi_connection_connect_internal ((omapi_object_t *)
							    obj);
		/*
		 * inprogress is the same as success but used
		 * to indicate to the dispatch code that we should
		 * mark the socket as requiring more attention.
		 * Routines calling this function should handle
		 * success properly.
		 */
		if (status == ISC_R_INPROGRESS) {
			status = ISC_R_SUCCESS;
		}
//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//	}
//ScenSim-Port//	omapi_connection_register (obj, MDL);
//ScenSim-Port//#endif

      out:
	omapi_connection_dereference (&obj, MDL);
	return status;
}

//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//omapi_array_t *omapi_connections;
//ScenSim-Port//
//ScenSim-Port//OMAPI_ARRAY_TYPE(omapi_connection, omapi_connection_object_t)
//ScenSim-Port//
//ScenSim-Port//void omapi_connection_trace_setup (void) {
//ScenSim-Port//	trace_connect = trace_type_register ("connect", (void *)0,
//ScenSim-Port//					     trace_connect_input,
//ScenSim-Port//					     trace_connect_stop, MDL);
//ScenSim-Port//	trace_disconnect = trace_type_register ("disconnect", (void *)0,
//ScenSim-Port//						trace_disconnect_input,
//ScenSim-Port//						trace_disconnect_stop, MDL);
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//void omapi_connection_register (omapi_connection_object_t *obj,
//ScenSim-Port//				const char *file, int line)
//ScenSim-Port//{
//ScenSim-Port//	isc_result_t status;
//ScenSim-Port//	trace_iov_t iov [6];
//ScenSim-Port//	int iov_count = 0;
//ScenSim-Port//	int32_t connect_index, listener_index;
//ScenSim-Port//	static int32_t index;
//ScenSim-Port//
//ScenSim-Port//	if (!omapi_connections) {
//ScenSim-Port//		status = omapi_connection_array_allocate (&omapi_connections,
//ScenSim-Port//							  file, line);
//ScenSim-Port//		if (status != ISC_R_SUCCESS)
//ScenSim-Port//			return;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	status = omapi_connection_array_extend (omapi_connections, obj,
//ScenSim-Port//						(int *)0, file, line);
//ScenSim-Port//	if (status != ISC_R_SUCCESS) {
//ScenSim-Port//		obj -> index = -1;
//ScenSim-Port//		return;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//	if (trace_record ()) {
//ScenSim-Port//		/* Connection registration packet:
//ScenSim-Port//		   
//ScenSim-Port//		     int32_t index
//ScenSim-Port//		     int32_t listener_index [-1 means no listener]
//ScenSim-Port//		   u_int16_t remote_port
//ScenSim-Port//		   u_int16_t local_port
//ScenSim-Port//		   u_int32_t remote_addr
//ScenSim-Port//		   u_int32_t local_addr */
//ScenSim-Port//
//ScenSim-Port//		connect_index = htonl (index);
//ScenSim-Port//		index++;
//ScenSim-Port//		if (obj -> listener)
//ScenSim-Port//			listener_index = htonl (obj -> listener -> index);
//ScenSim-Port//		else
//ScenSim-Port//			listener_index = htonl (-1);
//ScenSim-Port//		iov [iov_count].buf = (char *)&connect_index;
//ScenSim-Port//		iov [iov_count++].len = sizeof connect_index;
//ScenSim-Port//		iov [iov_count].buf = (char *)&listener_index;
//ScenSim-Port//		iov [iov_count++].len = sizeof listener_index;
//ScenSim-Port//		iov [iov_count].buf = (char *)&obj -> remote_addr.sin_port;
//ScenSim-Port//		iov [iov_count++].len = sizeof obj -> remote_addr.sin_port;
//ScenSim-Port//		iov [iov_count].buf = (char *)&obj -> local_addr.sin_port;
//ScenSim-Port//		iov [iov_count++].len = sizeof obj -> local_addr.sin_port;
//ScenSim-Port//		iov [iov_count].buf = (char *)&obj -> remote_addr.sin_addr;
//ScenSim-Port//		iov [iov_count++].len = sizeof obj -> remote_addr.sin_addr;
//ScenSim-Port//		iov [iov_count].buf = (char *)&obj -> local_addr.sin_addr;
//ScenSim-Port//		iov [iov_count++].len = sizeof obj -> local_addr.sin_addr;
//ScenSim-Port//
//ScenSim-Port//		status = trace_write_packet_iov (trace_connect,
//ScenSim-Port//						 iov_count, iov, file, line);
//ScenSim-Port//	}
//ScenSim-Port//#endif
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//static void trace_connect_input (trace_type_t *ttype,
//ScenSim-Port//				 unsigned length, char *buf)
//ScenSim-Port//{
//ScenSim-Port//	struct sockaddr_in remote, local;
//ScenSim-Port//	int32_t connect_index, listener_index;
//ScenSim-Port//	char *s = buf;
//ScenSim-Port//	omapi_connection_object_t *obj;
//ScenSim-Port//	isc_result_t status;
//ScenSim-Port//	int i;
//ScenSim-Port//
//ScenSim-Port//	if (length != ((sizeof connect_index) +
//ScenSim-Port//		       (sizeof remote.sin_port) +
//ScenSim-Port//		       (sizeof remote.sin_addr)) * 2) {
//ScenSim-Port//		log_error ("Trace connect: invalid length %d", length);
//ScenSim-Port//		return;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	memset (&remote, 0, sizeof remote);
//ScenSim-Port//	memset (&local, 0, sizeof local);
//ScenSim-Port//	memcpy (&connect_index, s, sizeof connect_index);
//ScenSim-Port//	s += sizeof connect_index;
//ScenSim-Port//	memcpy (&listener_index, s, sizeof listener_index);
//ScenSim-Port//	s += sizeof listener_index;
//ScenSim-Port//	memcpy (&remote.sin_port, s, sizeof remote.sin_port);
//ScenSim-Port//	s += sizeof remote.sin_port;
//ScenSim-Port//	memcpy (&local.sin_port, s, sizeof local.sin_port);
//ScenSim-Port//	s += sizeof local.sin_port;
//ScenSim-Port//	memcpy (&remote.sin_addr, s, sizeof remote.sin_addr);
//ScenSim-Port//	s += sizeof remote.sin_addr;
//ScenSim-Port//	memcpy (&local.sin_addr, s, sizeof local.sin_addr);
//ScenSim-Port//	s += sizeof local.sin_addr;
//ScenSim-Port//
//ScenSim-Port//	connect_index = ntohl (connect_index);
//ScenSim-Port//	listener_index = ntohl (listener_index);
//ScenSim-Port//
//ScenSim-Port//	/* If this was a connect to a listener, then we just slap together
//ScenSim-Port//	   a new connection. */
//ScenSim-Port//	if (listener_index != -1) {
//ScenSim-Port//		omapi_listener_object_t *listener;
//ScenSim-Port//		listener = (omapi_listener_object_t *)0;
//ScenSim-Port//		omapi_array_foreach_begin (trace_listeners,
//ScenSim-Port//					   omapi_listener_object_t, lp) {
//ScenSim-Port//			if (lp -> address.sin_port == local.sin_port) {
//ScenSim-Port//				omapi_listener_reference (&listener, lp, MDL);
//ScenSim-Port//				omapi_listener_dereference (&lp, MDL);
//ScenSim-Port//				break;
//ScenSim-Port//			} 
//ScenSim-Port//		} omapi_array_foreach_end (trace_listeners,
//ScenSim-Port//					   omapi_listener_object_t, lp);
//ScenSim-Port//		if (!listener) {
//ScenSim-Port//			log_error ("%s%ld, addr %s, port %d",
//ScenSim-Port//				   "Spurious traced listener connect - index ",
//ScenSim-Port//				   (long int)listener_index,
//ScenSim-Port//				   inet_ntoa (local.sin_addr),
//ScenSim-Port//				   ntohs (local.sin_port));
//ScenSim-Port//			return;
//ScenSim-Port//		}
//ScenSim-Port//		obj = (omapi_connection_object_t *)0;
//ScenSim-Port//		status = omapi_listener_connect (&obj, listener, -1, &remote);
//ScenSim-Port//		if (status != ISC_R_SUCCESS) {
//ScenSim-Port//			log_error ("traced listener connect: %s",
//ScenSim-Port//				   isc_result_totext (status));
//ScenSim-Port//		}
//ScenSim-Port//		if (obj)
//ScenSim-Port//			omapi_connection_dereference (&obj, MDL);
//ScenSim-Port//		omapi_listener_dereference (&listener, MDL);
//ScenSim-Port//		return;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/* Find the matching connect object, if there is one. */
//ScenSim-Port//	omapi_array_foreach_begin (omapi_connections,
//ScenSim-Port//				   omapi_connection_object_t, lp) {
//ScenSim-Port//	    for (i = 0; (lp -> connect_list &&
//ScenSim-Port//			 i < lp -> connect_list -> count); i++) {
//ScenSim-Port//		    if (!memcmp (&remote.sin_addr,
//ScenSim-Port//				 &lp -> connect_list -> addresses [i].address,
//ScenSim-Port//				 sizeof remote.sin_addr) &&
//ScenSim-Port//			(ntohs (remote.sin_port) ==
//ScenSim-Port//			 lp -> connect_list -> addresses [i].port))
//ScenSim-Port//			lp -> state = omapi_connection_connected;
//ScenSim-Port//			lp -> remote_addr = remote;
//ScenSim-Port//			lp -> remote_addr.sin_family = AF_INET;
//ScenSim-Port//			omapi_addr_list_dereference (&lp -> connect_list, MDL);
//ScenSim-Port//			lp -> index = connect_index;
//ScenSim-Port//			status = omapi_signal_in ((omapi_object_t *)lp,
//ScenSim-Port//						  "connect");
//ScenSim-Port//			omapi_connection_dereference (&lp, MDL);
//ScenSim-Port//			return;
//ScenSim-Port//		}
//ScenSim-Port//	} omapi_array_foreach_end (omapi_connections,
//ScenSim-Port//				   omapi_connection_object_t, lp);
//ScenSim-Port//						 
//ScenSim-Port//	log_error ("Spurious traced connect - index %ld, addr %s, port %d",
//ScenSim-Port//		   (long int)connect_index, inet_ntoa (remote.sin_addr),
//ScenSim-Port//		   ntohs (remote.sin_port));
//ScenSim-Port//	return;
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//static void trace_connect_stop (trace_type_t *ttype) { }
//ScenSim-Port//
//ScenSim-Port//static void trace_disconnect_input (trace_type_t *ttype,
//ScenSim-Port//				    unsigned length, char *buf)
//ScenSim-Port//{
//ScenSim-Port//	int32_t *index;
//ScenSim-Port//	if (length != sizeof *index) {
//ScenSim-Port//		log_error ("trace disconnect: wrong length %d", length);
//ScenSim-Port//		return;
//ScenSim-Port//	}
//ScenSim-Port//	
//ScenSim-Port//	index = (int32_t *)buf;
//ScenSim-Port//
//ScenSim-Port//	omapi_array_foreach_begin (omapi_connections,
//ScenSim-Port//				   omapi_connection_object_t, lp) {
//ScenSim-Port//		if (lp -> index == ntohl (*index)) {
//ScenSim-Port//			omapi_disconnect ((omapi_object_t *)lp, 1);
//ScenSim-Port//			omapi_connection_dereference (&lp, MDL);
//ScenSim-Port//			return;
//ScenSim-Port//		}
//ScenSim-Port//	} omapi_array_foreach_end (omapi_connections,
//ScenSim-Port//				   omapi_connection_object_t, lp);
//ScenSim-Port//
//ScenSim-Port//	log_error ("trace disconnect: no connection matching index %ld",
//ScenSim-Port//		   (long int)ntohl (*index));
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//static void trace_disconnect_stop (trace_type_t *ttype) { }
//ScenSim-Port//#endif

/* Disconnect a connection object from the remote end.   If force is nonzero,
   close the connection immediately.   Otherwise, shut down the receiving end
   but allow any unsent data to be sent before actually closing the socket. */

isc_result_t omapi_disconnect (omapi_object_t *h,
			       int force)
{
	omapi_connection_object_t *c;

//ScenSim-Port//#ifdef DEBUG_PROTOCOL
//ScenSim-Port//	log_debug ("omapi_disconnect(%s)", force ? "force" : "");
//ScenSim-Port//#endif

	c = (omapi_connection_object_t *)h;
	if (c -> type != omapi_type_connection)
		return DHCP_R_INVALIDARG;

//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//	if (trace_record ()) {
//ScenSim-Port//		isc_result_t status;
//ScenSim-Port//		int32_t index;
//ScenSim-Port//
//ScenSim-Port//		index = htonl (c -> index);
//ScenSim-Port//		status = trace_write_packet (trace_disconnect,
//ScenSim-Port//					     sizeof index, (char *)&index,
//ScenSim-Port//					     MDL);
//ScenSim-Port//		if (status != ISC_R_SUCCESS) {
//ScenSim-Port//			trace_stop ();
//ScenSim-Port//			log_error ("trace_write_packet: %s",
//ScenSim-Port//				   isc_result_totext (status));
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//	if (!trace_playback ()) {
//ScenSim-Port//#endif
		if (!force) {
			/* If we're already disconnecting, we don't have to do
			   anything. */
			if (c -> state == omapi_connection_disconnecting)
				return ISC_R_SUCCESS;

			/* Try to shut down the socket - this sends a FIN to
			   the remote end, so that it won't send us any more
			   data.   If the shutdown succeeds, and we still
			   have bytes left to write, defer closing the socket
			   until that's done. */
//ScenSim-Port//			if (!shutdown (c -> socket, SHUT_RD)) {
//ScenSim-Port//				if (c -> out_bytes > 0) {
//ScenSim-Port//					c -> state =
//ScenSim-Port//						omapi_connection_disconnecting;
//ScenSim-Port//					return ISC_R_SUCCESS;
//ScenSim-Port//				}
//ScenSim-Port//			}
		}
		close (c -> socket);
//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//	}
//ScenSim-Port//#endif
	c -> state = omapi_connection_closed;

//ScenSim-Port//#if 0
//ScenSim-Port//	/*
//ScenSim-Port//	 * Disconnecting from the I/O object seems incorrect as it doesn't
//ScenSim-Port//	 * cause the I/O object to be cleaned and released.  Previous to
//ScenSim-Port//	 * using the isc socket library this wouldn't have caused a problem
//ScenSim-Port//	 * with the socket library we would have a reference to a closed
//ScenSim-Port//	 * socket.  Instead we now do an unregister to properly free the
//ScenSim-Port//	 * I/O object.
//ScenSim-Port//	 */
//ScenSim-Port//
//ScenSim-Port//	/* Disconnect from I/O object, if any. */
//ScenSim-Port//	if (h -> outer) {
//ScenSim-Port//		if (h -> outer -> inner)
//ScenSim-Port//			omapi_object_dereference (&h -> outer -> inner, MDL);
//ScenSim-Port//		omapi_object_dereference (&h -> outer, MDL);
//ScenSim-Port//	}
//ScenSim-Port//#else
	if (h->outer) {
		omapi_unregister_io_object(h);
	}
//ScenSim-Port//#endif

	/* If whatever created us registered a signal handler, send it
	   a disconnect signal. */
	omapi_signal (h, "disconnect", h);

	/* Disconnect from protocol object, if any. */
	if (h->inner != NULL) {
		if (h->inner->outer != NULL) {
			omapi_object_dereference(&h->inner->outer, MDL);
		}
		omapi_object_dereference(&h->inner, MDL);
	}

	/* XXX: the code to free buffers should be in the dereference
		function, but there is no special-purpose function to
		dereference connections, so these just get leaked */
	/* Free any buffers */
	if (c->inbufs != NULL) {
		omapi_buffer_dereference(&c->inbufs, MDL);
	}
	c->in_bytes = 0;
	if (c->outbufs != NULL) {
		omapi_buffer_dereference(&c->outbufs, MDL);
	}
	c->out_bytes = 0;

	return ISC_R_SUCCESS;
}

isc_result_t omapi_connection_require (omapi_object_t *h, unsigned bytes)
{
	omapi_connection_object_t *c;

	if (h -> type != omapi_type_connection)
		return DHCP_R_INVALIDARG;
	c = (omapi_connection_object_t *)h;

	c -> bytes_needed = bytes;
	if (c -> bytes_needed <= c -> in_bytes) {
		return ISC_R_SUCCESS;
	}
	return DHCP_R_NOTYET;
}

/* Return the socket on which the dispatcher should wait for readiness
   to read, for a connection object.  */
int omapi_connection_readfd (omapi_object_t *h)
{
	omapi_connection_object_t *c;
	if (h -> type != omapi_type_connection)
		return -1;
	c = (omapi_connection_object_t *)h;
	if (c -> state != omapi_connection_connected)
		return -1;
	return c -> socket;
}

/*
 * Return the socket on which the dispatcher should wait for readiness
 * to write, for a connection object.  When bytes are buffered we should
 * also poke the dispatcher to tell it to start or re-start watching the
 * socket.
 */
int omapi_connection_writefd (omapi_object_t *h)
{
	omapi_connection_object_t *c;
	if (h -> type != omapi_type_connection)
		return -1;
	c = (omapi_connection_object_t *)h;
	return c->socket;
}

isc_result_t omapi_connection_connect (omapi_object_t *h)
{
	isc_result_t status;

	/*
	 * We use the INPROGRESS status to indicate that
	 * we want more from the socket.  In this case we
	 * have now connected and are trying to write to
	 * the socket for the first time.  For the signaling
	 * code this is the same as a SUCCESS so we don't
	 * pass it on as a signal.
	 */
	status = omapi_connection_connect_internal (h);
	if (status == ISC_R_INPROGRESS) 
		return ISC_R_INPROGRESS;

	if (status != ISC_R_SUCCESS)
		omapi_signal (h, "status", status);

	return ISC_R_SUCCESS;
}

static isc_result_t omapi_connection_connect_internal (omapi_object_t *h)
{
	int error;
	omapi_connection_object_t *c;
	socklen_t sl;
	isc_result_t status;

	if (h -> type != omapi_type_connection)
		return DHCP_R_INVALIDARG;
	c = (omapi_connection_object_t *)h;

	if (c -> state == omapi_connection_connecting) {
		sl = sizeof error;
//ScenSim-Port//		if (getsockopt (c -> socket, SOL_SOCKET, SO_ERROR,
//ScenSim-Port//				(char *)&error, &sl) < 0) {
//ScenSim-Port//			omapi_disconnect (h, 1);
//ScenSim-Port//			return ISC_R_SUCCESS;
//ScenSim-Port//		}
		if (!error)
			c -> state = omapi_connection_connected;
	}
	if (c -> state == omapi_connection_connecting ||
	    c -> state == omapi_connection_unconnected) {
		if (c -> cptr >= c -> connect_list -> count) {
			switch (error) {
			      case ECONNREFUSED:
				status = ISC_R_CONNREFUSED;
				break;
			      case ENETUNREACH:
				status = ISC_R_NETUNREACH;
				break;
			      default:
				status = uerr2isc (error);
				break;
			}
			omapi_disconnect (h, 1);
			return status;
		}

		if (c -> connect_list -> addresses [c -> cptr].addrtype !=
		    AF_INET) {
			omapi_disconnect (h, 1);
			return DHCP_R_INVALIDARG;
		}

		memcpy (&c -> remote_addr.sin_addr,
			&c -> connect_list -> addresses [c -> cptr].address,
			sizeof c -> remote_addr.sin_addr);
		c -> remote_addr.sin_family = AF_INET;
		c -> remote_addr.sin_port =
		       htons (c -> connect_list -> addresses [c -> cptr].port);
//ScenSim-Port//#if defined (HAVE_SA_LEN)
//ScenSim-Port//		c -> remote_addr.sin_len = sizeof c -> remote_addr;
//ScenSim-Port//#endif
		memset (&c -> remote_addr.sin_zero, 0,
			sizeof c -> remote_addr.sin_zero);
		++c -> cptr;

//ScenSim-Port//		error = connect (c -> socket,
//ScenSim-Port//				 (struct sockaddr *)&c -> remote_addr,
//ScenSim-Port//				 sizeof c -> remote_addr);
		if (error < 0) {
			error = errno;
			if (error != EINPROGRESS) {
				omapi_disconnect (h, 1);
				switch (error) {
				      case ECONNREFUSED:
					status = ISC_R_CONNREFUSED;
					break;
				      case ENETUNREACH:
					status = ISC_R_NETUNREACH;
					break;
				      default:
					status = uerr2isc (error);
					break;
				}
				return status;
			}
			c -> state = omapi_connection_connecting;
			return DHCP_R_INCOMPLETE;
		}
		c -> state = omapi_connection_connected;
	}
	
	/* I don't know why this would fail, so I'm tempted not to test
	   the return value. */
	sl = sizeof (c -> local_addr);
//ScenSim-Port//	if (getsockname (c -> socket,
//ScenSim-Port//			 (struct sockaddr *)&c -> local_addr, &sl) < 0) {
//ScenSim-Port//	}

	/* Reregister with the I/O object.  If we don't already have an
	   I/O object this turns into a register call, otherwise we simply
	   modify the pointers in the I/O object. */

	status = omapi_reregister_io_object (h,
					     omapi_connection_readfd,
					     omapi_connection_writefd,
					     omapi_connection_reader,
					     omapi_connection_writer,
					     omapi_connection_reaper);

	if (status != ISC_R_SUCCESS) {
		omapi_disconnect (h, 1);
		return status;
	}

	omapi_signal_in (h, "connect");
	omapi_addr_list_dereference (&c -> connect_list, MDL);
	return ISC_R_INPROGRESS;
}

/* Reaper function for connection - if the connection is completely closed,
   reap it.   If it's in the disconnecting state, there were bytes left
   to write when the user closed it, so if there are now no bytes left to
   write, we can close it. */
isc_result_t omapi_connection_reaper (omapi_object_t *h)
{
	omapi_connection_object_t *c;

	if (h -> type != omapi_type_connection)
		return DHCP_R_INVALIDARG;

	c = (omapi_connection_object_t *)h;
	if (c -> state == omapi_connection_disconnecting &&
	    c -> out_bytes == 0) {
//ScenSim-Port//#ifdef DEBUG_PROTOCOL
//ScenSim-Port//		log_debug ("omapi_connection_reaper(): disconnect");
//ScenSim-Port//#endif
		omapi_disconnect (h, 1);
	}
	if (c -> state == omapi_connection_closed) {
//ScenSim-Port//#ifdef DEBUG_PROTOCOL
//ScenSim-Port//		log_debug ("omapi_connection_reaper(): closed");
//ScenSim-Port//#endif
		return ISC_R_NOTCONNECTED;
	}
	return ISC_R_SUCCESS;
}

//ScenSim-Port//static isc_result_t make_dst_key (dst_key_t **dst_key, omapi_object_t *a) {
//ScenSim-Port//	omapi_value_t *name      = (omapi_value_t *)0;
//ScenSim-Port//	omapi_value_t *algorithm = (omapi_value_t *)0;
//ScenSim-Port//	omapi_value_t *key       = (omapi_value_t *)0;
//ScenSim-Port//	char *name_str = NULL;
//ScenSim-Port//	isc_result_t status = ISC_R_SUCCESS;
//ScenSim-Port//
//ScenSim-Port//	if (status == ISC_R_SUCCESS)
//ScenSim-Port//		status = omapi_get_value_str
//ScenSim-Port//			(a, (omapi_object_t *)0, "name", &name);
//ScenSim-Port//
//ScenSim-Port//	if (status == ISC_R_SUCCESS)
//ScenSim-Port//		status = omapi_get_value_str
//ScenSim-Port//			(a, (omapi_object_t *)0, "algorithm", &algorithm);
//ScenSim-Port//
//ScenSim-Port//	if (status == ISC_R_SUCCESS)
//ScenSim-Port//		status = omapi_get_value_str
//ScenSim-Port//			(a, (omapi_object_t *)0, "key", &key);
//ScenSim-Port//
//ScenSim-Port//	if (status == ISC_R_SUCCESS) {
//ScenSim-Port//		if ((algorithm->value->type != omapi_datatype_data &&
//ScenSim-Port//		     algorithm->value->type != omapi_datatype_string) ||
//ScenSim-Port//		    strncasecmp((char *)algorithm->value->u.buffer.value,
//ScenSim-Port//				NS_TSIG_ALG_HMAC_MD5 ".",
//ScenSim-Port//				algorithm->value->u.buffer.len) != 0) {
//ScenSim-Port//			status = DHCP_R_INVALIDARG;
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (status == ISC_R_SUCCESS) {
//ScenSim-Port//		name_str = dmalloc (name -> value -> u.buffer.len + 1, MDL);
//ScenSim-Port//		if (!name_str)
//ScenSim-Port//			status = ISC_R_NOMEMORY;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (status == ISC_R_SUCCESS) {
//ScenSim-Port//		memcpy (name_str,
//ScenSim-Port//			name -> value -> u.buffer.value,
//ScenSim-Port//			name -> value -> u.buffer.len);
//ScenSim-Port//		name_str [name -> value -> u.buffer.len] = 0;
//ScenSim-Port//
//ScenSim-Port//		status = isclib_make_dst_key(name_str,
//ScenSim-Port//					     DHCP_HMAC_MD5_NAME,
//ScenSim-Port//					     key->value->u.buffer.value,
//ScenSim-Port//					     key->value->u.buffer.len,
//ScenSim-Port//					     dst_key);
//ScenSim-Port//
//ScenSim-Port//		if (*dst_key == NULL)
//ScenSim-Port//			status = ISC_R_NOMEMORY;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (name_str)
//ScenSim-Port//		dfree (name_str, MDL);
//ScenSim-Port//	if (key)
//ScenSim-Port//		omapi_value_dereference (&key, MDL);
//ScenSim-Port//	if (algorithm)
//ScenSim-Port//		omapi_value_dereference (&algorithm, MDL);
//ScenSim-Port//	if (name)
//ScenSim-Port//		omapi_value_dereference (&name, MDL);
//ScenSim-Port//
//ScenSim-Port//	return status;
//ScenSim-Port//}

//ScenSim-Port//isc_result_t omapi_connection_sign_data (int mode,
//ScenSim-Port//					 dst_key_t *key,
//ScenSim-Port//					 void **context,
//ScenSim-Port//					 const unsigned char *data,
//ScenSim-Port//					 const unsigned len,
//ScenSim-Port//					 omapi_typed_data_t **result)
//ScenSim-Port//{
//ScenSim-Port//	omapi_typed_data_t *td = (omapi_typed_data_t *)0;
//ScenSim-Port//	isc_result_t status;
//ScenSim-Port//	dst_context_t **dctx = (dst_context_t **)context;
//ScenSim-Port//
//ScenSim-Port//	/* Create the context for the dst module */
//ScenSim-Port//	if (mode & SIG_MODE_INIT) {
//ScenSim-Port//		status = dst_context_create(key, dhcp_gbl_ctx.mctx, dctx);
//ScenSim-Port//		if (status != ISC_R_SUCCESS) {
//ScenSim-Port//			return status;
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/* If we have any data add it to the context */
//ScenSim-Port//	if (len != 0) {
//ScenSim-Port//		isc_region_t region;
//ScenSim-Port//		region.base   = (unsigned char *)data;
//ScenSim-Port//		region.length = len;
//ScenSim-Port//		dst_context_adddata(*dctx, &region);
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/* Finish the signature and clean up the context */
//ScenSim-Port//	if (mode & SIG_MODE_FINAL) {
//ScenSim-Port//		unsigned int sigsize;
//ScenSim-Port//		isc_buffer_t sigbuf;
//ScenSim-Port//
//ScenSim-Port//		status = dst_key_sigsize(key, &sigsize);
//ScenSim-Port//		if (status != ISC_R_SUCCESS) {
//ScenSim-Port//			goto cleanup;
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//		status = omapi_typed_data_new (MDL, &td,
//ScenSim-Port//					       omapi_datatype_data,
//ScenSim-Port//					       sigsize);
//ScenSim-Port//		if (status != ISC_R_SUCCESS) {
//ScenSim-Port//			goto cleanup;
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//		isc_buffer_init(&sigbuf, td->u.buffer.value, td->u.buffer.len);
//ScenSim-Port//		status = dst_context_sign(*dctx, &sigbuf);
//ScenSim-Port//		if (status != ISC_R_SUCCESS) {
//ScenSim-Port//			goto cleanup;
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//		if (result) {
//ScenSim-Port//			omapi_typed_data_reference (result, td, MDL);
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//	cleanup:
//ScenSim-Port//		/* We are done with the context and the td.  On success
//ScenSim-Port//		 * the td is now referenced from result, on failure we
//ScenSim-Port//		 * don't need it any more */
//ScenSim-Port//		if (td) {
//ScenSim-Port//			omapi_typed_data_dereference (&td, MDL);
//ScenSim-Port//		}
//ScenSim-Port//		dst_context_destroy(dctx);
//ScenSim-Port//		return status;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	return ISC_R_SUCCESS;
//ScenSim-Port//}

//ScenSim-Port//isc_result_t omapi_connection_output_auth_length (omapi_object_t *h,
//ScenSim-Port//						  unsigned *l)
//ScenSim-Port//{
//ScenSim-Port//	omapi_connection_object_t *c;
//ScenSim-Port//
//ScenSim-Port//	if (h->type != omapi_type_connection)
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//	c = (omapi_connection_object_t *)h;
//ScenSim-Port//
//ScenSim-Port//	if (c->out_key == NULL)
//ScenSim-Port//		return ISC_R_NOTFOUND;
//ScenSim-Port//
//ScenSim-Port//	return(dst_key_sigsize(c->out_key, l));
//ScenSim-Port//}

isc_result_t omapi_connection_set_value (omapi_object_t *h,
					 omapi_object_t *id,
					 omapi_data_string_t *name,
					 omapi_typed_data_t *value)
{
	omapi_connection_object_t *c;
	isc_result_t status;

	if (h -> type != omapi_type_connection)
		return DHCP_R_INVALIDARG;
	c = (omapi_connection_object_t *)h;

	if (omapi_ds_strcmp (name, "input-authenticator") == 0) {
		if (value && value -> type != omapi_datatype_object)
			return DHCP_R_INVALIDARG;

//ScenSim-Port//		if (c -> in_context) {
//ScenSim-Port//			omapi_connection_sign_data (SIG_MODE_FINAL,
//ScenSim-Port//						    c -> in_key,
//ScenSim-Port//						    &c -> in_context,
//ScenSim-Port//						    0, 0,
//ScenSim-Port//						    (omapi_typed_data_t **) 0);
//ScenSim-Port//		}

//ScenSim-Port//		if (c->in_key != NULL) {
//ScenSim-Port//			dst_key_free(&c->in_key);
//ScenSim-Port//		}

//ScenSim-Port//		if (value) {
//ScenSim-Port//			status = make_dst_key (&c -> in_key,
//ScenSim-Port//					       value -> u.object);
//ScenSim-Port//			if (status != ISC_R_SUCCESS)
//ScenSim-Port//				return status;
//ScenSim-Port//		}

		return ISC_R_SUCCESS;
	}
	else if (omapi_ds_strcmp (name, "output-authenticator") == 0) {
		if (value && value -> type != omapi_datatype_object)
			return DHCP_R_INVALIDARG;

//ScenSim-Port//		if (c -> out_context) {
//ScenSim-Port//			omapi_connection_sign_data (SIG_MODE_FINAL,
//ScenSim-Port//						    c -> out_key,
//ScenSim-Port//						    &c -> out_context,
//ScenSim-Port//						    0, 0,
//ScenSim-Port//						    (omapi_typed_data_t **) 0);
//ScenSim-Port//		}

//ScenSim-Port//		if (c->out_key != NULL) {
//ScenSim-Port//			dst_key_free(&c->out_key);
//ScenSim-Port//		}

//ScenSim-Port//		if (value) {
//ScenSim-Port//			status = make_dst_key (&c -> out_key,
//ScenSim-Port//					       value -> u.object);
//ScenSim-Port//			if (status != ISC_R_SUCCESS)
//ScenSim-Port//				return status;
//ScenSim-Port//		}

		return ISC_R_SUCCESS;
	}
	
	if (h -> inner && h -> inner -> type -> set_value)
		return (*(h -> inner -> type -> set_value))
			(h -> inner, id, name, value);
	return ISC_R_NOTFOUND;
}

isc_result_t omapi_connection_get_value (omapi_object_t *h,
					 omapi_object_t *id,
					 omapi_data_string_t *name,
					 omapi_value_t **value)
{
	omapi_connection_object_t *c;
	omapi_typed_data_t *td = (omapi_typed_data_t *)0;
	isc_result_t status;
	unsigned int sigsize;

	if (h -> type != omapi_type_connection)
		return DHCP_R_INVALIDARG;
	c = (omapi_connection_object_t *)h;

	if (omapi_ds_strcmp (name, "input-signature") == 0) {
//ScenSim-Port//		if (!c -> in_key || !c -> in_context)
//ScenSim-Port//			return ISC_R_NOTFOUND;

//ScenSim-Port//		status = omapi_connection_sign_data (SIG_MODE_FINAL,
//ScenSim-Port//						     c -> in_key,
//ScenSim-Port//						     &c -> in_context,
//ScenSim-Port//						     0, 0, &td);
//ScenSim-Port//		if (status != ISC_R_SUCCESS)
//ScenSim-Port//			return status;

		status = omapi_make_value (value, name, td, MDL);
		omapi_typed_data_dereference (&td, MDL);
		return status;

	} else if (omapi_ds_strcmp (name, "input-signature-size") == 0) {
//ScenSim-Port//		if (c->in_key == NULL)
//ScenSim-Port//			return ISC_R_NOTFOUND;

//ScenSim-Port//		status = dst_key_sigsize(c->in_key, &sigsize);
//ScenSim-Port//		if (status != ISC_R_SUCCESS) {
//ScenSim-Port//			return(status);
//ScenSim-Port//		}		

		return omapi_make_int_value(value, name, sigsize, MDL);

	} else if (omapi_ds_strcmp (name, "output-signature") == 0) {
//ScenSim-Port//		if (!c -> out_key || !c -> out_context)
//ScenSim-Port//			return ISC_R_NOTFOUND;

//ScenSim-Port//		status = omapi_connection_sign_data (SIG_MODE_FINAL,
//ScenSim-Port//						     c -> out_key,
//ScenSim-Port//						     &c -> out_context,
//ScenSim-Port//						     0, 0, &td);
//ScenSim-Port//		if (status != ISC_R_SUCCESS)
//ScenSim-Port//			return status;

		status = omapi_make_value (value, name, td, MDL);
		omapi_typed_data_dereference (&td, MDL);
		return status;

	} else if (omapi_ds_strcmp (name, "output-signature-size") == 0) {
//ScenSim-Port//		if (c->out_key == NULL)
//ScenSim-Port//			return ISC_R_NOTFOUND;


//ScenSim-Port//		status = dst_key_sigsize(c->out_key, &sigsize);
//ScenSim-Port//		if (status != ISC_R_SUCCESS) {
//ScenSim-Port//			return(status);
//ScenSim-Port//		}		

		return omapi_make_int_value(value, name, sigsize, MDL);
	}
	
	if (h -> inner && h -> inner -> type -> get_value)
		return (*(h -> inner -> type -> get_value))
			(h -> inner, id, name, value);
	return ISC_R_NOTFOUND;
}

isc_result_t omapi_connection_destroy (omapi_object_t *h,
				       const char *file, int line)
{
	omapi_connection_object_t *c;

//ScenSim-Port//#ifdef DEBUG_PROTOCOL
//ScenSim-Port//	log_debug ("omapi_connection_destroy()");
//ScenSim-Port//#endif

	if (h -> type != omapi_type_connection)
		return ISC_R_UNEXPECTED;
	c = (omapi_connection_object_t *)(h);
	if (c -> state == omapi_connection_connected)
		omapi_disconnect (h, 1);
//ScenSim-Port//	if (c -> listener)
//ScenSim-Port//		omapi_listener_dereference (&c -> listener, file, line);
	if (c -> connect_list)
		omapi_addr_list_dereference (&c -> connect_list, file, line);
	return ISC_R_SUCCESS;
}

isc_result_t omapi_connection_signal_handler (omapi_object_t *h,
					      const char *name, va_list ap)
{
	if (h -> type != omapi_type_connection)
		return DHCP_R_INVALIDARG;

//ScenSim-Port//#ifdef DEBUG_PROTOCOL
//ScenSim-Port//	log_debug ("omapi_connection_signal_handler(%s)", name);
//ScenSim-Port//#endif
	
	if (h -> inner && h -> inner -> type -> signal_handler)
		return (*(h -> inner -> type -> signal_handler)) (h -> inner,
								  name, ap);
	return ISC_R_NOTFOUND;
}

/* Write all the published values associated with the object through the
   specified connection. */

isc_result_t omapi_connection_stuff_values (omapi_object_t *c,
					    omapi_object_t *id,
					    omapi_object_t *m)
{
	if (m -> type != omapi_type_connection)
		return DHCP_R_INVALIDARG;

	if (m -> inner && m -> inner -> type -> stuff_values)
		return (*(m -> inner -> type -> stuff_values)) (c, id,
								m -> inner);
	return ISC_R_SUCCESS;
}
}//namespace IscDhcpPort////ScenSim-Port//
