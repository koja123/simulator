/* alloc.h

   Definitions for the object management API protocol memory allocation... */

/*
 * Copyright (c) 2004,2009 by Internet Systems Consortium, Inc. ("ISC")
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

isc_result_t omapi_buffer_new (omapi_buffer_t **, const char *, int);
isc_result_t omapi_buffer_reference (omapi_buffer_t **,
				     omapi_buffer_t *, const char *, int);
isc_result_t omapi_buffer_dereference (omapi_buffer_t **, const char *, int);

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || defined (DEBUG_MALLOC_POOL) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//#define DMDOFFSET (sizeof (struct dmalloc_preamble))
//ScenSim-Port//#define DMLFSIZE 16
//ScenSim-Port//#define DMUFSIZE 16
//ScenSim-Port//#define DMDSIZE (DMDOFFSET + DMLFSIZE + DMUFSIZE)
//ScenSim-Port//
//ScenSim-Port//struct dmalloc_preamble {
//ScenSim-Port//	struct dmalloc_preamble *prev, *next;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
//ScenSim-Port//	size_t size;
//ScenSim-Port//	unsigned long generation;
//ScenSim-Port//	unsigned char low_fence [DMLFSIZE];
//ScenSim-Port//};
//ScenSim-Port//#else
#define DMDOFFSET 0
#define DMDSIZE 0
//ScenSim-Port//#endif

/* rc_history flags... */
#define RC_LEASE	1
#define RC_MISC		2

//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//#if !defined (RC_HISTORY_MAX)
//ScenSim-Port//# define RC_HISTORY_MAX 256
//ScenSim-Port//#endif

//ScenSim-Port//#if !defined (RC_HISTORY_FLAGS)
//ScenSim-Port//# define RC_HISTORY_FLAGS (RC_LEASE | RC_MISC)
//ScenSim-Port//#endif

//ScenSim-Port//struct rc_history_entry {
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
//ScenSim-Port//	void *reference;
//ScenSim-Port//	void *addr;
//ScenSim-Port//	int refcnt;
//ScenSim-Port//};

//ScenSim-Port//#define rc_register(x, l, r, y, z, d, f) do { \
//ScenSim-Port//		if (RC_HISTORY_FLAGS & ~(f)) { \
//ScenSim-Port//			rc_history [rc_history_index].file = (x); \
//ScenSim-Port//			rc_history [rc_history_index].line = (l); \
//ScenSim-Port//			rc_history [rc_history_index].reference = (r); \
//ScenSim-Port//			rc_history [rc_history_index].addr = (y); \
//ScenSim-Port//			rc_history [rc_history_index].refcnt = (z); \
//ScenSim-Port//			rc_history_next (d); \
//ScenSim-Port//		} \
//ScenSim-Port//	} while (0)
//ScenSim-Port//#define rc_register_mdl(r, y, z, d, f) \
//ScenSim-Port//	rc_register (__FILE__, __LINE__, r, y, z, d, f)
//ScenSim-Port//#else
#define rc_register(file, line, reference, addr, refcnt, d, f)
#define rc_register_mdl(reference, addr, refcnt, d, f)
//ScenSim-Port//#endif

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || defined (DEBUG_MALLOC_POOL) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//extern struct dmalloc_preamble *dmalloc_list;
//ScenSim-Port//extern unsigned long dmalloc_outstanding;
//ScenSim-Port//extern unsigned long dmalloc_longterm;
//ScenSim-Port//extern unsigned long dmalloc_generation;
//ScenSim-Port//extern unsigned long dmalloc_cutoff_generation;
//ScenSim-Port//#endif

//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//extern struct rc_history_entry rc_history [RC_HISTORY_MAX];
//ScenSim-Port//extern int rc_history_index;
//ScenSim-Port//extern int rc_history_count;
//ScenSim-Port//#endif
