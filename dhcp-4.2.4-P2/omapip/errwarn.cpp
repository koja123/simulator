/* errwarn.c

   Errors and warnings... */

/*
 * Copyright (c) 1995 RadioMail Corporation.
 * Copyright (c) 2004,2007,2009 by Internet Systems Consortium, Inc. ("ISC")
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
 * This software was written for RadioMail Corporation by Ted Lemon
 * under a contract with Vixie Enterprises.   Further modifications have
 * been made for Internet Systems Consortium under a contract
 * with Vixie Laboratories.
 */
#include <iscdhcp_porting.h>//ScenSim-Port//
namespace IscDhcpPort {//ScenSim-Port//

//ScenSim-Port//#include "dhcpd.h"

//ScenSim-Port//#include <omapip/omapip_p.h>
//ScenSim-Port//#include <errno.h>
//ScenSim-Port//#include <syslog.h>

//ScenSim-Port//#ifdef DEBUG
//ScenSim-Port//int log_perror = -1;
//ScenSim-Port//#else
//ScenSim-Port//int log_perror = 1;
//ScenSim-Port//#endif
//ScenSim-Port//int log_priority;
//ScenSim-Port//void (*log_cleanup) (void);

#define CVT_BUF_MAX 1023
static char mbuf [CVT_BUF_MAX + 1];
static char fbuf [CVT_BUF_MAX + 1];

/* Log an error message, then exit... */

void log_fatal (const char * fmt, ... )
{
  va_list list;

  do_percentm (fbuf, fmt);

  /* %Audit% This is log output. %2004.06.17,Safe%
   * If we truncate we hope the user can get a hint from the log.
   */
  va_start (list, fmt);
  vsnprintf (mbuf, sizeof mbuf, fbuf, list);
  va_end (list);

//ScenSim-Port//#ifndef DEBUG
//ScenSim-Port//  syslog (log_priority | LOG_ERR, "%s", mbuf);
  syslog ((log_priority | LOG_ERR), "%s", mbuf); //ScenSim-Port//
//ScenSim-Port//#endif

  /* Also log it to stderr? */
//ScenSim-Port//  if (log_perror) {
//ScenSim-Port//	  IGNORE_RET (write (STDERR_FILENO, mbuf, strlen (mbuf)));
//ScenSim-Port//	  IGNORE_RET (write (STDERR_FILENO, "\n", 1));
//ScenSim-Port//  }

//ScenSim-Port//#if !defined (NOMINUM)
//ScenSim-Port//  log_error ("%s", "");
//ScenSim-Port//  log_error ("If you did not get this software from ftp.isc.org, please");
//ScenSim-Port//  log_error ("get the latest from ftp.isc.org and install that before");
//ScenSim-Port//  log_error ("requesting help.");
//ScenSim-Port//  log_error ("%s", "");
//ScenSim-Port//  log_error ("If you did get this software from ftp.isc.org and have not");
//ScenSim-Port//  log_error ("yet read the README, please read it before requesting help.");
//ScenSim-Port//  log_error ("If you intend to request help from the dhcp-server@isc.org");
//ScenSim-Port//  log_error ("mailing list, please read the section on the README about");
//ScenSim-Port//  log_error ("submitting bug reports and requests for help.");
//ScenSim-Port//  log_error ("%s", "");
//ScenSim-Port//  log_error ("Please do not under any circumstances send requests for");
//ScenSim-Port//  log_error ("help directly to the authors of this software - please");
//ScenSim-Port//  log_error ("send them to the appropriate mailing list as described in");
//ScenSim-Port//  log_error ("the README file.");
//ScenSim-Port//  log_error ("%s", "");
//ScenSim-Port//  log_error ("exiting.");
//ScenSim-Port//#endif
//ScenSim-Port//  if (log_cleanup)
//ScenSim-Port//	  (*log_cleanup) ();
  exit (1);
}

/* Log an error message... */

int log_error (const char * fmt, ...)
{
  va_list list;

  do_percentm (fbuf, fmt);

  /* %Audit% This is log output. %2004.06.17,Safe%
   * If we truncate we hope the user can get a hint from the log.
   */
  va_start (list, fmt);
  vsnprintf (mbuf, sizeof mbuf, fbuf, list);
  va_end (list);

//ScenSim-Port//#ifndef DEBUG
//ScenSim-Port//  syslog (log_priority | LOG_ERR, "%s", mbuf);
  syslog ((log_priority | LOG_ERR), "%s", mbuf); //ScenSim-Port//
//ScenSim-Port//#endif

//ScenSim-Port//  if (log_perror) {
//ScenSim-Port//	  IGNORE_RET (write (STDERR_FILENO, mbuf, strlen (mbuf)));
//ScenSim-Port//	  IGNORE_RET (write (STDERR_FILENO, "\n", 1));
//ScenSim-Port//  }

  return 0;
}

/* Log a note... */

int log_info (const char *fmt, ...)
{
  va_list list;

  do_percentm (fbuf, fmt);

  /* %Audit% This is log output. %2004.06.17,Safe%
   * If we truncate we hope the user can get a hint from the log.
   */
  va_start (list, fmt);
  vsnprintf (mbuf, sizeof mbuf, fbuf, list);
  va_end (list);

//ScenSim-Port//#ifndef DEBUG
//ScenSim-Port//  syslog (log_priority | LOG_INFO, "%s", mbuf);
  syslog ((log_priority | LOG_INFO), "%s", mbuf); //ScenSim-Port//
//ScenSim-Port//#endif

//ScenSim-Port//  if (log_perror) {
//ScenSim-Port//	  IGNORE_RET (write (STDERR_FILENO, mbuf, strlen (mbuf)));
//ScenSim-Port//	  IGNORE_RET (write (STDERR_FILENO, "\n", 1));
//ScenSim-Port//  }

  return 0;
}

/* Log a debug message... */

int log_debug (const char *fmt, ...)
{
  va_list list;

  do_percentm (fbuf, fmt);

  /* %Audit% This is log output. %2004.06.17,Safe%
   * If we truncate we hope the user can get a hint from the log.
   */
  va_start (list, fmt);
  vsnprintf (mbuf, sizeof mbuf, fbuf, list);
  va_end (list);

//ScenSim-Port//#ifndef DEBUG
//ScenSim-Port//  syslog (log_priority | LOG_DEBUG, "%s", mbuf);
  syslog ((log_priority | LOG_DEBUG), "%s", mbuf); //ScenSim-Port//
//ScenSim-Port//#endif

//ScenSim-Port//  if (log_perror) {
//ScenSim-Port//	  IGNORE_RET (write (STDERR_FILENO, mbuf, strlen (mbuf)));
//ScenSim-Port//	  IGNORE_RET (write (STDERR_FILENO, "\n", 1));
//ScenSim-Port//  }

  return 0;
}

/* Find %m in the input string and substitute an error message string. */

//ScenSim-Port//void do_percentm (obuf, ibuf)
//ScenSim-Port//     char *obuf;
//ScenSim-Port//     const char *ibuf;
void do_percentm (char *obuf, const char *ibuf)//ScenSim-Port//
{
	const char *s = ibuf;
	char *p = obuf;
	int infmt = 0;
	const char *m;
	int len = 0;

	while (*s) {
		if (infmt) {
			if (*s == 'm') {
//ScenSim-Port//#ifndef __CYGWIN32__
				m = strerror (errno);
//ScenSim-Port//#else
//ScenSim-Port//				m = pWSAError ();
//ScenSim-Port//#endif
				if (!m)
					m = "<unknown error>";
				len += strlen (m);
				if (len > CVT_BUF_MAX)
					goto out;
				strcpy (p - 1, m);
				p += strlen (p);
				++s;
			} else {
				if (++len > CVT_BUF_MAX)
					goto out;
				*p++ = *s++;
			}
			infmt = 0;
		} else {
			if (*s == '%')
				infmt = 1;
			if (++len > CVT_BUF_MAX)
				goto out;
			*p++ = *s++;
		}
	}
      out:
	*p = 0;
}

//ScenSim-Port//#ifdef NO_STRERROR
//ScenSim-Port//char *strerror (err)
//ScenSim-Port//	int err;
//ScenSim-Port//{
//ScenSim-Port//	extern char *sys_errlist [];
//ScenSim-Port//	extern int sys_nerr;
//ScenSim-Port//	static char errbuf [128];
//ScenSim-Port//
//ScenSim-Port//	if (err < 0 || err >= sys_nerr) {
//ScenSim-Port//		sprintf (errbuf, "Error %d", err);
//ScenSim-Port//		return errbuf;
//ScenSim-Port//	}
//ScenSim-Port//	return sys_errlist [err];
//ScenSim-Port//}
//ScenSim-Port//#endif /* NO_STRERROR */

//ScenSim-Port//#ifdef _WIN32
//ScenSim-Port//char *pWSAError ()
//ScenSim-Port//{
//ScenSim-Port//  int err = WSAGetLastError ();
//ScenSim-Port//
//ScenSim-Port//  switch (err)
//ScenSim-Port//    {
//ScenSim-Port//    case WSAEACCES:
//ScenSim-Port//      return "Permission denied";
//ScenSim-Port//    case WSAEADDRINUSE:
//ScenSim-Port//      return "Address already in use";
//ScenSim-Port//    case WSAEADDRNOTAVAIL:
//ScenSim-Port//      return "Cannot assign requested address";
//ScenSim-Port//    case WSAEAFNOSUPPORT:
//ScenSim-Port//      return "Address family not supported by protocol family";
//ScenSim-Port//    case WSAEALREADY:
//ScenSim-Port//      return "Operation already in progress";
//ScenSim-Port//    case WSAECONNABORTED:
//ScenSim-Port//      return "Software caused connection abort";
//ScenSim-Port//    case WSAECONNREFUSED:
//ScenSim-Port//      return "Connection refused";
//ScenSim-Port//    case WSAECONNRESET:
//ScenSim-Port//      return "Connection reset by peer";
//ScenSim-Port//    case WSAEDESTADDRREQ:
//ScenSim-Port//      return "Destination address required";
//ScenSim-Port//    case WSAEFAULT:
//ScenSim-Port//      return "Bad address";
//ScenSim-Port//    case WSAEHOSTDOWN:
//ScenSim-Port//      return "Host is down";
//ScenSim-Port//    case WSAEHOSTUNREACH:
//ScenSim-Port//      return "No route to host";
//ScenSim-Port//    case WSAEINPROGRESS:
//ScenSim-Port//      return "Operation now in progress";
//ScenSim-Port//    case WSAEINTR:
//ScenSim-Port//      return "Interrupted function call";
//ScenSim-Port//    case WSAEINVAL:
//ScenSim-Port//      return "Invalid argument";
//ScenSim-Port//    case WSAEISCONN:
//ScenSim-Port//      return "Socket is already connected";
//ScenSim-Port//    case WSAEMFILE:
//ScenSim-Port//      return "Too many open files";
//ScenSim-Port//    case WSAEMSGSIZE:
//ScenSim-Port//      return "Message too long";
//ScenSim-Port//    case WSAENETDOWN:
//ScenSim-Port//      return "Network is down";
//ScenSim-Port//    case WSAENETRESET:
//ScenSim-Port//      return "Network dropped connection on reset";
//ScenSim-Port//    case WSAENETUNREACH:
//ScenSim-Port//      return "Network is unreachable";
//ScenSim-Port//    case WSAENOBUFS:
//ScenSim-Port//      return "No buffer space available";
//ScenSim-Port//    case WSAENOPROTOOPT:
//ScenSim-Port//      return "Bad protocol option";
//ScenSim-Port//    case WSAENOTCONN:
//ScenSim-Port//      return "Socket is not connected";
//ScenSim-Port//    case WSAENOTSOCK:
//ScenSim-Port//      return "Socket operation on non-socket";
//ScenSim-Port//    case WSAEOPNOTSUPP:
//ScenSim-Port//      return "Operation not supported";
//ScenSim-Port//    case WSAEPFNOSUPPORT:
//ScenSim-Port//      return "Protocol family not supported";
//ScenSim-Port//    case WSAEPROCLIM:
//ScenSim-Port//      return "Too many processes";
//ScenSim-Port//    case WSAEPROTONOSUPPORT:
//ScenSim-Port//      return "Protocol not supported";
//ScenSim-Port//    case WSAEPROTOTYPE:
//ScenSim-Port//      return "Protocol wrong type for socket";
//ScenSim-Port//    case WSAESHUTDOWN:
//ScenSim-Port//      return "Cannot send after socket shutdown";
//ScenSim-Port//    case WSAESOCKTNOSUPPORT:
//ScenSim-Port//      return "Socket type not supported";
//ScenSim-Port//    case WSAETIMEDOUT:
//ScenSim-Port//      return "Connection timed out";
//ScenSim-Port//    case WSAEWOULDBLOCK:
//ScenSim-Port//      return "Resource temporarily unavailable";
//ScenSim-Port//    case WSAHOST_NOT_FOUND:
//ScenSim-Port//      return "Host not found";
//ScenSim-Port//#if 0
//ScenSim-Port//    case WSA_INVALID_HANDLE:
//ScenSim-Port//      return "Specified event object handle is invalid";
//ScenSim-Port//    case WSA_INVALID_PARAMETER:
//ScenSim-Port//      return "One or more parameters are invalid";
//ScenSim-Port//    case WSAINVALIDPROCTABLE:
//ScenSim-Port//      return "Invalid procedure table from service provider";
//ScenSim-Port//    case WSAINVALIDPROVIDER:
//ScenSim-Port//      return "Invalid service provider version number";
//ScenSim-Port//    case WSA_IO_PENDING:
//ScenSim-Port//      return "Overlapped operations will complete later";
//ScenSim-Port//    case WSA_IO_INCOMPLETE:
//ScenSim-Port//      return "Overlapped I/O event object not in signaled state";
//ScenSim-Port//    case WSA_NOT_ENOUGH_MEMORY:
//ScenSim-Port//      return "Insufficient memory available";
//ScenSim-Port//#endif
//ScenSim-Port//    case WSANOTINITIALISED:
//ScenSim-Port//      return "Successful WSAStartup not yet performer";
//ScenSim-Port//    case WSANO_DATA:
//ScenSim-Port//      return "Valid name, no data record of requested type";
//ScenSim-Port//    case WSANO_RECOVERY:
//ScenSim-Port//      return "This is a non-recoverable error";
//ScenSim-Port//#if 0
//ScenSim-Port//    case WSAPROVIDERFAILEDINIT:
//ScenSim-Port//      return "Unable to initialize a service provider";
//ScenSim-Port//    case WSASYSCALLFAILURE:
//ScenSim-Port//      return "System call failure";
//ScenSim-Port//#endif
//ScenSim-Port//    case WSASYSNOTREADY:
//ScenSim-Port//      return "Network subsystem is unavailable";
//ScenSim-Port//    case WSATRY_AGAIN:
//ScenSim-Port//      return "Non-authoritative host not found";
//ScenSim-Port//    case WSAVERNOTSUPPORTED:
//ScenSim-Port//      return "WINSOCK.DLL version out of range";
//ScenSim-Port//    case WSAEDISCON:
//ScenSim-Port//      return "Graceful shutdown in progress";
//ScenSim-Port//#if 0
//ScenSim-Port//    case WSA_OPERATION_ABORTED:
//ScenSim-Port//      return "Overlapped operation aborted";
//ScenSim-Port//#endif
//ScenSim-Port//    }
//ScenSim-Port//  return "Unknown WinSock error";
//ScenSim-Port//}
//ScenSim-Port//#endif /* _WIN32 */
}//namespace IscDhcpPort////ScenSim-Port//
