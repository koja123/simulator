/* toisc.c

   Convert non-ISC result codes to ISC result codes. */

/*
 * Copyright (c) 2004,2007,2009 by Internet Systems Consortium, Inc. ("ISC")
 * Copyright (c) 2001-2003 by Internet Software Consortium
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
//ScenSim-Port//#include "arpa/nameser.h"
//ScenSim-Port//#include "minires.h"

//ScenSim-Port//#include <errno.h>

isc_result_t uerr2isc (int err)
{
//ScenSim-Port//	switch (err) {
//ScenSim-Port//	      case EPERM:
//ScenSim-Port//		return ISC_R_NOPERM;
//ScenSim-Port//
//ScenSim-Port//	      case ENOENT:
//ScenSim-Port//		return ISC_R_NOTFOUND;
//ScenSim-Port//
//ScenSim-Port//	      case ESRCH:
//ScenSim-Port//		return ISC_R_NOTFOUND;
//ScenSim-Port//
//ScenSim-Port//	      case EIO:
//ScenSim-Port//		return ISC_R_IOERROR;
//ScenSim-Port//
//ScenSim-Port//	      case ENXIO:
//ScenSim-Port//		return ISC_R_NOTFOUND;
//ScenSim-Port//
//ScenSim-Port//	      case E2BIG:
//ScenSim-Port//		return ISC_R_NOSPACE;
//ScenSim-Port//
//ScenSim-Port//	      case ENOEXEC:
//ScenSim-Port//		return DHCP_R_FORMERR;
//ScenSim-Port//
//ScenSim-Port//	      case ECHILD:
//ScenSim-Port//		return ISC_R_NOTFOUND;
//ScenSim-Port//
//ScenSim-Port//	      case ENOMEM:
//ScenSim-Port//		return ISC_R_NOMEMORY;
//ScenSim-Port//
//ScenSim-Port//	      case EACCES:
//ScenSim-Port//		return ISC_R_NOPERM;
//ScenSim-Port//
//ScenSim-Port//	      case EFAULT:
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//
//ScenSim-Port//	      case EEXIST:
//ScenSim-Port//		return ISC_R_EXISTS;
//ScenSim-Port//
//ScenSim-Port//	      case EINVAL:
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//
//ScenSim-Port//	      case ENOTTY:
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//
//ScenSim-Port//	      case EFBIG:
//ScenSim-Port//		return ISC_R_NOSPACE;
//ScenSim-Port//
//ScenSim-Port//	      case ENOSPC:
//ScenSim-Port//		return ISC_R_NOSPACE;
//ScenSim-Port//
//ScenSim-Port//	      case EROFS:
//ScenSim-Port//		return ISC_R_NOPERM;
//ScenSim-Port//
//ScenSim-Port//	      case EMLINK:
//ScenSim-Port//		return ISC_R_NOSPACE;
//ScenSim-Port//
//ScenSim-Port//	      case EPIPE:
//ScenSim-Port//		return ISC_R_NOTCONNECTED;
//ScenSim-Port//
//ScenSim-Port//	      case EINPROGRESS:
//ScenSim-Port//		return ISC_R_ALREADYRUNNING;
//ScenSim-Port//
//ScenSim-Port//	      case EALREADY:
//ScenSim-Port//		return ISC_R_ALREADYRUNNING;
//ScenSim-Port//
//ScenSim-Port//	      case ENOTSOCK:
//ScenSim-Port//		return ISC_R_INVALIDFILE;
//ScenSim-Port//
//ScenSim-Port//	      case EDESTADDRREQ:
//ScenSim-Port//		return DHCP_R_DESTADDRREQ;
//ScenSim-Port//
//ScenSim-Port//	      case EMSGSIZE:
//ScenSim-Port//		return ISC_R_NOSPACE;
//ScenSim-Port//
//ScenSim-Port//	      case EPROTOTYPE:
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//
//ScenSim-Port//	      case ENOPROTOOPT:
//ScenSim-Port//		return ISC_R_NOTIMPLEMENTED;
//ScenSim-Port//
//ScenSim-Port//	      case EPROTONOSUPPORT:
//ScenSim-Port//		return ISC_R_NOTIMPLEMENTED;
//ScenSim-Port//
//ScenSim-Port//	      case ESOCKTNOSUPPORT:
//ScenSim-Port//		return ISC_R_NOTIMPLEMENTED;
//ScenSim-Port//
//ScenSim-Port//	      case EOPNOTSUPP:
//ScenSim-Port//		return ISC_R_NOTIMPLEMENTED;
//ScenSim-Port//
//ScenSim-Port//	      case EPFNOSUPPORT:
//ScenSim-Port//		return ISC_R_NOTIMPLEMENTED;
//ScenSim-Port//
//ScenSim-Port//	      case EAFNOSUPPORT:
//ScenSim-Port//		return ISC_R_NOTIMPLEMENTED;
//ScenSim-Port//
//ScenSim-Port//	      case EADDRINUSE:
//ScenSim-Port//		return ISC_R_ADDRINUSE;
//ScenSim-Port//
//ScenSim-Port//	      case EADDRNOTAVAIL:
//ScenSim-Port//		return ISC_R_ADDRNOTAVAIL;
//ScenSim-Port//
//ScenSim-Port//	      case ENETDOWN:
//ScenSim-Port//		return ISC_R_NETDOWN;
//ScenSim-Port//
//ScenSim-Port//	      case ENETUNREACH:
//ScenSim-Port//		return ISC_R_NETUNREACH;
//ScenSim-Port//
//ScenSim-Port//	      case ECONNABORTED:
//ScenSim-Port//		return ISC_R_TIMEDOUT;
//ScenSim-Port//
//ScenSim-Port//	      case ECONNRESET:
//ScenSim-Port//		return DHCP_R_CONNRESET;
//ScenSim-Port//
//ScenSim-Port//	      case ENOBUFS:
//ScenSim-Port//		return ISC_R_NOSPACE;
//ScenSim-Port//
//ScenSim-Port//	      case EISCONN:
//ScenSim-Port//		return ISC_R_ALREADYRUNNING;
//ScenSim-Port//
//ScenSim-Port//	      case ENOTCONN:
//ScenSim-Port//		return ISC_R_NOTCONNECTED;
//ScenSim-Port//
//ScenSim-Port//	      case ESHUTDOWN:
//ScenSim-Port//		return ISC_R_SHUTTINGDOWN;
//ScenSim-Port//
//ScenSim-Port//	      case ETIMEDOUT:
//ScenSim-Port//		return ISC_R_TIMEDOUT;
//ScenSim-Port//
//ScenSim-Port//	      case ECONNREFUSED:
//ScenSim-Port//		return ISC_R_CONNREFUSED;
//ScenSim-Port//
//ScenSim-Port//	      case EHOSTDOWN:
//ScenSim-Port//		return ISC_R_HOSTDOWN;
//ScenSim-Port//
//ScenSim-Port//	      case EHOSTUNREACH:
//ScenSim-Port//		return ISC_R_HOSTUNREACH;
//ScenSim-Port//
//ScenSim-Port//#ifdef EDQUOT
//ScenSim-Port//	      case EDQUOT:
//ScenSim-Port//		return ISC_R_QUOTA;
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//#ifdef EBADRPC
//ScenSim-Port//	      case EBADRPC:
//ScenSim-Port//		return ISC_R_NOTIMPLEMENTED;
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//#ifdef ERPCMISMATCH
//ScenSim-Port//	      case ERPCMISMATCH:
//ScenSim-Port//		return DHCP_R_VERSIONMISMATCH;
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//#ifdef EPROGMISMATCH
//ScenSim-Port//	      case EPROGMISMATCH:
//ScenSim-Port//		return DHCP_R_VERSIONMISMATCH;
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//#ifdef EAUTH
//ScenSim-Port//	      case EAUTH:
//ScenSim-Port//		return DHCP_R_NOTAUTH;
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//#ifdef ENEEDAUTH
//ScenSim-Port//	      case ENEEDAUTH:
//ScenSim-Port//		return DHCP_R_NOTAUTH;
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//#ifdef EOVERFLOW
//ScenSim-Port//	      case EOVERFLOW:
//ScenSim-Port//		return ISC_R_NOSPACE;
//ScenSim-Port//#endif
//ScenSim-Port//	}
	return ISC_R_UNEXPECTED;
}
}//namespace IscDhcpPort////ScenSim-Port//
