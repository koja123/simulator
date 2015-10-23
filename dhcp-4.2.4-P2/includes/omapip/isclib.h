/* isclib.h

   connections to the isc and dns libraries */

/*
 * Copyright (c) 2009 by Internet Systems Consortium, Inc. ("ISC")
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
 *   http://www.isc.org/
 *
 */

#ifndef ISCLIB_H
#define ISCLIB_H

//ScenSim-Port//#include "config.h"

//ScenSim-Port//#include <syslog.h>

#define MAXWIRE 256

//ScenSim-Port//#include <sys/types.h>
//ScenSim-Port//#include <sys/socket.h>

//ScenSim-Port//#include <netinet/in.h>

//ScenSim-Port//#include <arpa/inet.h>

//ScenSim-Port//#include <unistd.h>
//ScenSim-Port//#include <ctype.h>
//ScenSim-Port//#include <stdio.h>
//ScenSim-Port//#include <stdlib.h>
//ScenSim-Port//#include <string.h>
//ScenSim-Port//#include <netdb.h>

//ScenSim-Port//#include <isc/buffer.h>
//ScenSim-Port//#include <isc/lex.h>
//ScenSim-Port//#include <isc/lib.h>
//ScenSim-Port//#include <isc/app.h>
//ScenSim-Port//#include <isc/mem.h>
//ScenSim-Port//#include <isc/parseint.h>
//ScenSim-Port//#include <isc/socket.h>
//ScenSim-Port//#include <isc/sockaddr.h>
//ScenSim-Port//#include <isc/task.h>
//ScenSim-Port//#include <isc/timer.h>
//ScenSim-Port//#include <isc/heap.h>
//ScenSim-Port//#include <isc/random.h>

//ScenSim-Port//#include <dns/client.h>
//ScenSim-Port//#include <dns/fixedname.h>
//ScenSim-Port//#include <dns/keyvalues.h>
//ScenSim-Port//#include <dns/lib.h>
//ScenSim-Port//#include <dns/name.h>
//ScenSim-Port//#include <dns/rdata.h>
//ScenSim-Port//#include <dns/rdataclass.h>
//ScenSim-Port//#include <dns/rdatalist.h>
//ScenSim-Port//#include <dns/rdataset.h>
//ScenSim-Port//#include <dns/rdatastruct.h>
//ScenSim-Port//#include <dns/rdatatype.h>
//ScenSim-Port//#include <dns/result.h>
//ScenSim-Port//#include <dns/secalg.h>
//ScenSim-Port//#include <dns/tsec.h>

//ScenSim-Port//#include <dst/dst.h>

//ScenSim-Port//#include "result.h"


/*
 * DHCP context structure
 * This holds the libisc information for a dhcp entity
 */

typedef struct dhcp_context {
//ScenSim-Port//	isc_mem_t	*mctx;
//ScenSim-Port//	isc_appctx_t	*actx;
	int              actx_started;
//ScenSim-Port//	isc_taskmgr_t	*taskmgr;
//ScenSim-Port//	isc_task_t	*task;
//ScenSim-Port//	isc_socketmgr_t *socketmgr;
//ScenSim-Port//	isc_timermgr_t	*timermgr;
#if defined (NSUPDATE)
  	dns_client_t    *dnsclient;
#endif
} dhcp_context_t;

extern dhcp_context_t dhcp_gbl_ctx;

#define DHCP_MAXDNS_WIRE 256
#define DHCP_MAXNS         3
#define DHCP_HMAC_MD5_NAME "HMAC-MD5.SIG-ALG.REG.INT."

//ScenSim-Port//isc_result_t dhcp_isc_name(unsigned char    *namestr,
//ScenSim-Port//			   dns_fixedname_t  *namefix,
//ScenSim-Port//			   dns_name_t      **name);

//ScenSim-Port//isc_result_t
//ScenSim-Port//isclib_make_dst_key(char          *inname,
//ScenSim-Port//		    char          *algorithm,
//ScenSim-Port//		    unsigned char *secret,
//ScenSim-Port//		    int            length,
//ScenSim-Port//		    dst_key_t    **dstkey);

isc_result_t dhcp_context_create(void);
void isclib_cleanup(void);

#endif /* ISCLIB_H */
