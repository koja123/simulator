/* alloc.c

   Functions supporting memory allocation for the object management
   protocol... */

/*
 * Copyright (c) 2009-2010 by Internet Systems Consortium, Inc. ("ISC")
 * Copyright (c) 2004-2007 by Internet Systems Consortium, Inc. ("ISC")
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
#define new new_ptr//ScenSim-Port//

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || defined (DEBUG_MALLOC_POOL) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//struct dmalloc_preamble *dmalloc_list;
//ScenSim-Port//unsigned long dmalloc_outstanding;
//ScenSim-Port//unsigned long dmalloc_longterm;
//ScenSim-Port//unsigned long dmalloc_generation;
//ScenSim-Port//unsigned long dmalloc_cutoff_generation;
//ScenSim-Port//#endif

//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//struct rc_history_entry rc_history [RC_HISTORY_MAX];
//ScenSim-Port//int rc_history_index;
//ScenSim-Port//int rc_history_count;
//ScenSim-Port//#endif

//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//static void print_rc_hist_entry (int);
//ScenSim-Port//#endif

void *
dmalloc(unsigned size, const char *file, int line) {
	unsigned char *foo;
	unsigned len;
//ScenSim-Port//	void **bar;
	void *bar;//ScenSim-Port//
//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || defined (DEBUG_MALLOC_POOL) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//	int i;
//ScenSim-Port//	struct dmalloc_preamble *dp;
//ScenSim-Port//#endif

	len = size + DMDSIZE;
	if (len < size)
		return NULL;

//ScenSim-Port//	foo = malloc(len);
	foo = (unsigned char*)malloc(len);//ScenSim-Port//

	if (!foo)
		return NULL;
	bar = (void *)(foo + DMDOFFSET);
	memset (bar, 0, size);

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || defined (DEBUG_MALLOC_POOL) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//	dp = (struct dmalloc_preamble *)foo;
//ScenSim-Port//	dp -> prev = dmalloc_list;
//ScenSim-Port//	if (dmalloc_list)
//ScenSim-Port//		dmalloc_list -> next = dp;
//ScenSim-Port//	dmalloc_list = dp;
//ScenSim-Port//	dp -> next = (struct dmalloc_preamble *)0;
//ScenSim-Port//	dp -> size = size;
//ScenSim-Port//	dp -> file = file;
//ScenSim-Port//	dp -> line = line;
//ScenSim-Port//	dp -> generation = dmalloc_generation++;
//ScenSim-Port//	dmalloc_outstanding += size;
//ScenSim-Port//	for (i = 0; i < DMLFSIZE; i++)
//ScenSim-Port//		dp -> low_fence [i] =
//ScenSim-Port//			(((unsigned long)
//ScenSim-Port//			  (&dp -> low_fence [i])) % 143) + 113;
//ScenSim-Port//	for (i = DMDOFFSET; i < DMDSIZE; i++)
//ScenSim-Port//		foo [i + size] =
//ScenSim-Port//			(((unsigned long)
//ScenSim-Port//			  (&foo [i + size])) % 143) + 113;
//ScenSim-Port//#if defined (DEBUG_MALLOC_POOL_EXHAUSTIVELY)
//ScenSim-Port//	/* Check _every_ entry in the pool!   Very expensive. */
//ScenSim-Port//	for (dp = dmalloc_list; dp; dp = dp -> prev) {
//ScenSim-Port//		for (i = 0; i < DMLFSIZE; i++) {
//ScenSim-Port//			if (dp -> low_fence [i] !=
//ScenSim-Port//				(((unsigned long)
//ScenSim-Port//				  (&dp -> low_fence [i])) % 143) + 113)
//ScenSim-Port//			{
//ScenSim-Port//				log_error ("malloc fence modified: %s(%d)",
//ScenSim-Port//					   dp -> file, dp -> line);
//ScenSim-Port//				abort ();
//ScenSim-Port//			}
//ScenSim-Port//		}
//ScenSim-Port//		foo = (unsigned char *)dp;
//ScenSim-Port//		for (i = DMDOFFSET; i < DMDSIZE; i++) {
//ScenSim-Port//			if (foo [i + dp -> size] !=
//ScenSim-Port//				(((unsigned long)
//ScenSim-Port//				  (&foo [i + dp -> size])) % 143) + 113) {
//ScenSim-Port//				log_error ("malloc fence modified: %s(%d)",
//ScenSim-Port//					   dp -> file, dp -> line);
//ScenSim-Port//				abort ();
//ScenSim-Port//			}
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//#endif
//ScenSim-Port//#endif
//ScenSim-Port//#ifdef DEBUG_REFCNT_DMALLOC_FREE
//ScenSim-Port//	rc_register (file, line, 0, foo + DMDOFFSET, 1, 0, RC_MALLOC);
//ScenSim-Port//#endif
	return bar;
}

void 
dfree(void *ptr, const char *file, int line) {
	if (!ptr) {
		log_error ("dfree %s(%d): free on null pointer.", file, line);
		return;
	}
//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || defined (DEBUG_MALLOC_POOL) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//	{
//ScenSim-Port//		unsigned char *bar = ptr;
//ScenSim-Port//		struct dmalloc_preamble *dp, *cur;
//ScenSim-Port//		int i;
//ScenSim-Port//		bar -= DMDOFFSET;
//ScenSim-Port//		cur = (struct dmalloc_preamble *)bar;
//ScenSim-Port//		for (dp = dmalloc_list; dp; dp = dp -> prev)
//ScenSim-Port//			if (dp == cur)
//ScenSim-Port//				break;
//ScenSim-Port//		if (!dp) {
//ScenSim-Port//			log_error ("%s(%d): freeing unknown memory: %lx",
//ScenSim-Port//				   file, line, (unsigned long)cur);
//ScenSim-Port//			abort ();
//ScenSim-Port//		}
//ScenSim-Port//		if (dp -> prev)
//ScenSim-Port//			dp -> prev -> next = dp -> next;
//ScenSim-Port//		if (dp -> next)
//ScenSim-Port//			dp -> next -> prev = dp -> prev;
//ScenSim-Port//		if (dp == dmalloc_list)
//ScenSim-Port//			dmalloc_list = dp -> prev;
//ScenSim-Port//		if (dp -> generation >= dmalloc_cutoff_generation)
//ScenSim-Port//			dmalloc_outstanding -= dp -> size;
//ScenSim-Port//		else
//ScenSim-Port//			dmalloc_longterm -= dp -> size;
//ScenSim-Port//
//ScenSim-Port//		for (i = 0; i < DMLFSIZE; i++) {
//ScenSim-Port//			if (dp -> low_fence [i] !=
//ScenSim-Port//				(((unsigned long)
//ScenSim-Port//				  (&dp -> low_fence [i])) % 143) + 113)
//ScenSim-Port//			{
//ScenSim-Port//				log_error ("malloc fence modified: %s(%d)",
//ScenSim-Port//					   dp -> file, dp -> line);
//ScenSim-Port//				abort ();
//ScenSim-Port//			}
//ScenSim-Port//		}
//ScenSim-Port//		for (i = DMDOFFSET; i < DMDSIZE; i++) {
//ScenSim-Port//			if (bar [i + dp -> size] !=
//ScenSim-Port//				(((unsigned long)
//ScenSim-Port//				  (&bar [i + dp -> size])) % 143) + 113) {
//ScenSim-Port//				log_error ("malloc fence modified: %s(%d)",
//ScenSim-Port//					   dp -> file, dp -> line);
//ScenSim-Port//				abort ();
//ScenSim-Port//			}
//ScenSim-Port//		}
//ScenSim-Port//		ptr = bar;
//ScenSim-Port//	}
//ScenSim-Port//#endif
//ScenSim-Port//#ifdef DEBUG_REFCNT_DMALLOC_FREE
//ScenSim-Port//	rc_register (file, line,
//ScenSim-Port//		     0, (unsigned char *)ptr + DMDOFFSET, 0, 1, RC_MALLOC);
//ScenSim-Port//#endif
	free (ptr);
}

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || defined (DEBUG_MALLOC_POOL) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port///* For allocation functions that keep their own free lists, we want to
//ScenSim-Port//   account for the reuse of the memory. */
//ScenSim-Port//
//ScenSim-Port//void 
//ScenSim-Port//dmalloc_reuse(void *foo, const char *file, int line, int justref) {
//ScenSim-Port//	struct dmalloc_preamble *dp;
//ScenSim-Port//
//ScenSim-Port//	/* Get the pointer to the dmalloc header. */
//ScenSim-Port//	dp = foo;
//ScenSim-Port//	dp--;
//ScenSim-Port//
//ScenSim-Port//	/* If we just allocated this and are now referencing it, this
//ScenSim-Port//	   function would almost be a no-op, except that it would
//ScenSim-Port//	   increment the generation count needlessly.  So just return
//ScenSim-Port//	   in this case. */
//ScenSim-Port//	if (dp -> generation == dmalloc_generation)
//ScenSim-Port//		return;
//ScenSim-Port//
//ScenSim-Port//	/* If this is longterm data, and we just made reference to it,
//ScenSim-Port//	   don't put it on the short-term list or change its name -
//ScenSim-Port//	   we don't need to know about this. */
//ScenSim-Port//	if (dp -> generation < dmalloc_cutoff_generation && justref)
//ScenSim-Port//		return;
//ScenSim-Port//
//ScenSim-Port//	/* Take it out of the place in the allocated list where it was. */
//ScenSim-Port//	if (dp -> prev)
//ScenSim-Port//		dp -> prev -> next = dp -> next;
//ScenSim-Port//	if (dp -> next)
//ScenSim-Port//		dp -> next -> prev = dp -> prev;
//ScenSim-Port//	if (dp == dmalloc_list)
//ScenSim-Port//		dmalloc_list = dp -> prev;
//ScenSim-Port//
//ScenSim-Port//	/* Account for its removal. */
//ScenSim-Port//	if (dp -> generation >= dmalloc_cutoff_generation)
//ScenSim-Port//		dmalloc_outstanding -= dp -> size;
//ScenSim-Port//	else
//ScenSim-Port//		dmalloc_longterm -= dp -> size;
//ScenSim-Port//
//ScenSim-Port//	/* Now put it at the head of the list. */
//ScenSim-Port//	dp -> prev = dmalloc_list;
//ScenSim-Port//	if (dmalloc_list)
//ScenSim-Port//		dmalloc_list -> next = dp;
//ScenSim-Port//	dmalloc_list = dp;
//ScenSim-Port//	dp -> next = (struct dmalloc_preamble *)0;
//ScenSim-Port//
//ScenSim-Port//	/* Change the reference location information. */
//ScenSim-Port//	dp -> file = file;
//ScenSim-Port//	dp -> line = line;
//ScenSim-Port//
//ScenSim-Port//	/* Increment the generation. */
//ScenSim-Port//	dp -> generation = dmalloc_generation++;
//ScenSim-Port//
//ScenSim-Port//	/* Account for it. */
//ScenSim-Port//	dmalloc_outstanding += dp -> size;
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//void dmalloc_dump_outstanding ()
//ScenSim-Port//{
//ScenSim-Port//	static unsigned long dmalloc_cutoff_point;
//ScenSim-Port//	struct dmalloc_preamble *dp;
//ScenSim-Port//#if defined(DEBUG_MALLOC_POOL)
//ScenSim-Port//	unsigned char *foo;
//ScenSim-Port//	int i;
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//	if (!dmalloc_cutoff_point)
//ScenSim-Port//		dmalloc_cutoff_point = dmalloc_cutoff_generation;
//ScenSim-Port//	for (dp = dmalloc_list; dp; dp = dp -> prev) {
//ScenSim-Port//		if (dp -> generation <= dmalloc_cutoff_point)
//ScenSim-Port//			break;
//ScenSim-Port//#if defined (DEBUG_MALLOC_POOL)
//ScenSim-Port//		for (i = 0; i < DMLFSIZE; i++) {
//ScenSim-Port//			if (dp -> low_fence [i] !=
//ScenSim-Port//				(((unsigned long)
//ScenSim-Port//				  (&dp -> low_fence [i])) % 143) + 113)
//ScenSim-Port//			{
//ScenSim-Port//				log_error ("malloc fence modified: %s(%d)",
//ScenSim-Port//					   dp -> file, dp -> line);
//ScenSim-Port//				abort ();
//ScenSim-Port//			}
//ScenSim-Port//		}
//ScenSim-Port//		foo = (unsigned char *)dp;
//ScenSim-Port//		for (i = DMDOFFSET; i < DMDSIZE; i++) {
//ScenSim-Port//			if (foo [i + dp -> size] !=
//ScenSim-Port//				(((unsigned long)
//ScenSim-Port//				  (&foo [i + dp -> size])) % 143) + 113) {
//ScenSim-Port//				log_error ("malloc fence modified: %s(%d)",
//ScenSim-Port//					   dp -> file, dp -> line);
//ScenSim-Port//				abort ();
//ScenSim-Port//			}
//ScenSim-Port//		}
//ScenSim-Port//#endif
//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//		/* Don't count data that's actually on a free list
//ScenSim-Port//                   somewhere. */
//ScenSim-Port//		if (dp -> file) {
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//			int i, count, inhistory = 0, noted = 0;
//ScenSim-Port//
//ScenSim-Port//			/* If we have the info, see if this is actually
//ScenSim-Port//			   new garbage. */
//ScenSim-Port//			if (rc_history_count < RC_HISTORY_MAX) {
//ScenSim-Port//			    count = rc_history_count;
//ScenSim-Port//			} else
//ScenSim-Port//			    count = RC_HISTORY_MAX;
//ScenSim-Port//			i = rc_history_index - 1;
//ScenSim-Port//			if (i < 0)
//ScenSim-Port//				i += RC_HISTORY_MAX;
//ScenSim-Port//
//ScenSim-Port//			do {
//ScenSim-Port//			    if (rc_history [i].addr == dp + 1) {
//ScenSim-Port//				inhistory = 1;
//ScenSim-Port//				if (!noted) {
//ScenSim-Port//				    log_info ("  %s(%d): %ld", dp -> file,
//ScenSim-Port//					      dp -> line, dp -> size);
//ScenSim-Port//				    noted = 1;
//ScenSim-Port//				}
//ScenSim-Port//				print_rc_hist_entry (i);
//ScenSim-Port//				if (!rc_history [i].refcnt)
//ScenSim-Port//				    break;
//ScenSim-Port//			    }
//ScenSim-Port//			    if (--i < 0)
//ScenSim-Port//				i = RC_HISTORY_MAX - 1;
//ScenSim-Port//			} while (count--);
//ScenSim-Port//			if (!inhistory)
//ScenSim-Port//#endif
//ScenSim-Port//				log_info ("  %s(%d): %ld",
//ScenSim-Port//					  dp -> file, dp -> line, dp -> size);
//ScenSim-Port//		}
//ScenSim-Port//#endif
//ScenSim-Port//	}
//ScenSim-Port//	if (dmalloc_list)
//ScenSim-Port//		dmalloc_cutoff_point = dmalloc_list -> generation;
//ScenSim-Port//}
//ScenSim-Port//#endif /* DEBUG_MEMORY_LEAKAGE || DEBUG_MALLOC_POOL */

//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//static void print_rc_hist_entry (int i)
//ScenSim-Port//{
//ScenSim-Port//	log_info ("   referenced by %s(%d)[%lx]: addr = %lx  refcnt = %x",
//ScenSim-Port//		  rc_history [i].file, rc_history [i].line,
//ScenSim-Port//		  (unsigned long)rc_history [i].reference,
//ScenSim-Port//		  (unsigned long)rc_history [i].addr,
//ScenSim-Port//		  rc_history [i].refcnt);
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//void dump_rc_history (void *addr)
//ScenSim-Port//{
//ScenSim-Port//	int i;
//ScenSim-Port//
//ScenSim-Port//	i = rc_history_index;
//ScenSim-Port//	if (!rc_history [i].file)
//ScenSim-Port//		i = 0;
//ScenSim-Port//	else if (rc_history_count < RC_HISTORY_MAX) {
//ScenSim-Port//		i -= rc_history_count;
//ScenSim-Port//		if (i < 0)
//ScenSim-Port//			i += RC_HISTORY_MAX;
//ScenSim-Port//	}
//ScenSim-Port//	rc_history_count = 0;
//ScenSim-Port//
//ScenSim-Port//	while (rc_history [i].file) {
//ScenSim-Port//		if (!addr || addr == rc_history [i].addr)
//ScenSim-Port//			print_rc_hist_entry (i);
//ScenSim-Port//		++i;
//ScenSim-Port//		if (i == RC_HISTORY_MAX)
//ScenSim-Port//			i = 0;
//ScenSim-Port//		if (i == rc_history_index)
//ScenSim-Port//			break;
//ScenSim-Port//	}
//ScenSim-Port//}
//ScenSim-Port//void rc_history_next (int d)
//ScenSim-Port//{
//ScenSim-Port//#if defined (RC_HISTORY_COMPRESSION)
//ScenSim-Port//	int i, j = 0, m, n = 0;
//ScenSim-Port//	void *ap, *rp;
//ScenSim-Port//
//ScenSim-Port//	/* If we are decreasing the reference count, try to find the
//ScenSim-Port//	   entry where the reference was made and eliminate it; then
//ScenSim-Port//	   we can also eliminate this reference. */
//ScenSim-Port//	if (d) {
//ScenSim-Port//	    m = rc_history_index - 1000;
//ScenSim-Port//	    if (m < -1)
//ScenSim-Port//		m = -1;
//ScenSim-Port//	    ap = rc_history [rc_history_index].addr;
//ScenSim-Port//	    rp = rc_history [rc_history_index].reference;
//ScenSim-Port//	    for (i = rc_history_index - 1; i > m; i--) {
//ScenSim-Port//		if (rc_history [i].addr == ap) {
//ScenSim-Port//		    if (rc_history [i].reference == rp) {
//ScenSim-Port//			if (n > 10) {
//ScenSim-Port//			    for (n = i; n <= rc_history_index; n++)
//ScenSim-Port//				    print_rc_hist_entry (n);
//ScenSim-Port//			    n = 11;
//ScenSim-Port//			}
//ScenSim-Port//			memmove (&rc_history [i],
//ScenSim-Port//				 &rc_history [i + 1],
//ScenSim-Port//				 (unsigned)((rc_history_index - i) *
//ScenSim-Port//					    sizeof (struct rc_history_entry)));
//ScenSim-Port//			--rc_history_count;
//ScenSim-Port//			--rc_history_index;
//ScenSim-Port//			for (j = i; j < rc_history_count; j++) {
//ScenSim-Port//			    if (rc_history [j].addr == ap)
//ScenSim-Port//				--rc_history [j].refcnt;
//ScenSim-Port//			}
//ScenSim-Port//			if (n > 10) {
//ScenSim-Port//			    for (n = i; n <= rc_history_index; n++)
//ScenSim-Port//				    print_rc_hist_entry (n);
//ScenSim-Port//			    n = 11;
//ScenSim-Port//			    exit (0);
//ScenSim-Port//			}
//ScenSim-Port//			return;
//ScenSim-Port//		    }
//ScenSim-Port//		}
//ScenSim-Port//	    }
//ScenSim-Port//	}
//ScenSim-Port//#endif
//ScenSim-Port//	if (++rc_history_index == RC_HISTORY_MAX)
//ScenSim-Port//		rc_history_index = 0;
//ScenSim-Port//	++rc_history_count;
//ScenSim-Port//}
//ScenSim-Port//#endif /* DEBUG_RC_HISTORY */

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || defined (DEBUG_MALLOC_POOL) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//struct caller {
//ScenSim-Port//	struct dmalloc_preamble *dp;
//ScenSim-Port//	int count;
//ScenSim-Port//};
//ScenSim-Port//
//ScenSim-Port//static int dmalloc_find_entry (struct dmalloc_preamble *dp,
//ScenSim-Port//			       struct caller *array,
//ScenSim-Port//			       int min, int max)
//ScenSim-Port//{
//ScenSim-Port//	int middle;
//ScenSim-Port//
//ScenSim-Port//	middle = (min + max) / 2;
//ScenSim-Port//	if (middle == min)
//ScenSim-Port//		return middle;
//ScenSim-Port//	if (array [middle].dp -> file == dp -> file) {
//ScenSim-Port//		if (array [middle].dp -> line == dp -> line)
//ScenSim-Port//			return middle;
//ScenSim-Port//		else if (array [middle].dp -> line < dp -> line)
//ScenSim-Port//			return dmalloc_find_entry (dp, array, middle, max);
//ScenSim-Port//		else
//ScenSim-Port//			return dmalloc_find_entry (dp, array, 0, middle);
//ScenSim-Port//	} else if (array [middle].dp -> file < dp -> file)
//ScenSim-Port//		return dmalloc_find_entry (dp, array, middle, max);
//ScenSim-Port//	else
//ScenSim-Port//		return dmalloc_find_entry (dp, array, 0, middle);
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//void omapi_print_dmalloc_usage_by_caller ()
//ScenSim-Port//{
//ScenSim-Port//	struct dmalloc_preamble *dp;
//ScenSim-Port//	int ccur, cmax, i;
//ScenSim-Port//	struct caller cp [1024];
//ScenSim-Port//
//ScenSim-Port//	cmax = 1024;
//ScenSim-Port//	ccur = 0;
//ScenSim-Port//
//ScenSim-Port//	memset (cp, 0, sizeof cp);
//ScenSim-Port//	for (dp = dmalloc_list; dp; dp = dp -> prev) {
//ScenSim-Port//		i = dmalloc_find_entry (dp, cp, 0, ccur);
//ScenSim-Port//		if ((i == ccur ||
//ScenSim-Port//		     cp [i].dp -> file != dp -> file ||
//ScenSim-Port//		     cp [i].dp -> line != dp -> line) &&
//ScenSim-Port//		    ccur == cmax) {
//ScenSim-Port//			log_error ("no space for memory usage summary.");
//ScenSim-Port//			return;
//ScenSim-Port//		}
//ScenSim-Port//		if (i == ccur) {
//ScenSim-Port//			cp [ccur++].dp = dp;
//ScenSim-Port//			cp [i].count = 1;
//ScenSim-Port//		} else if (cp [i].dp -> file < dp -> file ||
//ScenSim-Port//			   (cp [i].dp -> file == dp -> file &&
//ScenSim-Port//			    cp [i].dp -> line < dp -> line)) {
//ScenSim-Port//			if (i + 1 != ccur)
//ScenSim-Port//				memmove (cp + i + 2, cp + i + 1,
//ScenSim-Port//					 (ccur - i) * sizeof *cp);
//ScenSim-Port//			cp [i + 1].dp = dp;
//ScenSim-Port//			cp [i + 1].count = 1;
//ScenSim-Port//			ccur++;
//ScenSim-Port//		} else if (cp [i].dp -> file != dp -> file ||
//ScenSim-Port//			   cp [i].dp -> line != dp -> line) {
//ScenSim-Port//			memmove (cp + i + 1,
//ScenSim-Port//				 cp + i, (ccur - i) * sizeof *cp);
//ScenSim-Port//			cp [i].dp = dp;
//ScenSim-Port//			cp [i].count = 1;
//ScenSim-Port//			ccur++;
//ScenSim-Port//		} else
//ScenSim-Port//			cp [i].count++;
//ScenSim-Port//#if 0
//ScenSim-Port//		printf ("%d\t%s:%d\n", i, dp -> file, dp -> line);
//ScenSim-Port//		dump_rc_history (dp + 1);
//ScenSim-Port//#endif
//ScenSim-Port//	}
//ScenSim-Port//	for (i = 0; i < ccur; i++) {
//ScenSim-Port//		printf ("%d\t%s:%d\t%d\n", i,
//ScenSim-Port//			cp [i].dp -> file, cp [i].dp -> line, cp [i].count);
//ScenSim-Port//#if defined(DUMP_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (cp [i].dp + 1);
//ScenSim-Port//#endif
//ScenSim-Port//	}
//ScenSim-Port//}
//ScenSim-Port//#endif /* DEBUG_MEMORY_LEAKAGE || DEBUG_MALLOC_POOL */

isc_result_t omapi_object_allocate (omapi_object_t **o,
				    omapi_object_type_t *type,
				    size_t size,
				    const char *file, int line)
{
	size_t tsize;
	omapi_object_t *foo;
	isc_result_t status;

	if (type -> allocator) {
		foo = (omapi_object_t *)0;
		status = (*type -> allocator) (&foo, file, line);
		tsize = type -> size;
	} else {
		status = ISC_R_NOMEMORY;
		tsize = 0;
	}

	if (status == ISC_R_NOMEMORY) {
		if (type -> sizer)
			tsize = (*type -> sizer) (size);
		else
			tsize = type -> size;
		
		/* Sanity check. */
		if (tsize < sizeof (omapi_object_t))
			return DHCP_R_INVALIDARG;
		
//ScenSim-Port//		foo = dmalloc (tsize, file, line);
		foo = (omapi_object_t *)dmalloc (tsize, file, line);//ScenSim-Port//
		if (!foo)
			return ISC_R_NOMEMORY;
	}

	status = omapi_object_initialize (foo, type, size, tsize, file, line);
	if (status != ISC_R_SUCCESS) {
		if (type -> freer)
			(*type -> freer) (foo, file, line);
		else
			dfree (foo, file, line);
		return status;
	}
	return omapi_object_reference (o, foo, file, line);
}

isc_result_t omapi_object_initialize (omapi_object_t *o,
				      omapi_object_type_t *type,
				      size_t usize, size_t psize,
				      const char *file, int line)
{
	memset (o, 0, psize);
	o -> type = type;
	if (type -> initialize)
		(*type -> initialize) (o, file, line);
	return ISC_R_SUCCESS;
}

isc_result_t omapi_object_reference (omapi_object_t **r,
				     omapi_object_t *h,
				     const char *file, int line)
{
	if (!h || !r)
		return DHCP_R_INVALIDARG;

	if (*r) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): reference store into non-null pointer!",
//ScenSim-Port//			   file, line);
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	*r = h;
	h -> refcnt++;
	rc_register (file, line, r, h, h -> refcnt, 0, h -> type -> rc_flag);
	return ISC_R_SUCCESS;
}

isc_result_t omapi_object_dereference (omapi_object_t **h,
				       const char *file, int line)
{
	int outer_reference = 0;
	int inner_reference = 0;
	int handle_reference = 0;
	int extra_references;
	omapi_object_t *p, *hp;

	if (!h)
		return DHCP_R_INVALIDARG;

	if (!*h) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): dereference of null pointer!", file, line);
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	
	if ((*h) -> refcnt <= 0) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): dereference of pointer with refcnt of zero!",
//ScenSim-Port//			   file, line);
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (*h);
//ScenSim-Port//#endif
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*h = 0;
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	
	/* See if this object's inner object refers to it, but don't
	   count this as a reference if we're being asked to free the
	   reference from the inner object. */
	if ((*h) -> inner && (*h) -> inner -> outer &&
	    h != &((*h) -> inner -> outer))
		inner_reference = 1;

	/* Ditto for the outer object. */
	if ((*h) -> outer && (*h) -> outer -> inner &&
	    h != &((*h) -> outer -> inner))
		outer_reference = 1;

	/* Ditto for the outer object.  The code below assumes that
	   the only reason we'd get a dereference from the handle
	   table is if this function does it - otherwise we'd have to
	   traverse the handle table to find the address where the
	   reference is stored and compare against that, and we don't
	   want to do that if we can avoid it. */
	if ((*h) -> handle)
		handle_reference = 1;

	/* If we are getting rid of the last reference other than
	   references to inner and outer objects, or from the handle
	   table, then we must examine all the objects in either
	   direction to see if they hold any non-inner, non-outer,
	   non-handle-table references.  If not, we need to free the
	   entire chain of objects. */
	if ((*h) -> refcnt ==
	    inner_reference + outer_reference + handle_reference + 1) {
		if (inner_reference || outer_reference || handle_reference) {
			/* XXX we could check for a reference from the
                           handle table here. */
			extra_references = 0;
			for (p = (*h) -> inner;
			     p && !extra_references; p = p -> inner) {
				extra_references += p -> refcnt;
				if (p -> inner && p -> inner -> outer == p)
					--extra_references;
				if (p -> outer)
					--extra_references;
				if (p -> handle)
					--extra_references;
			}
			for (p = (*h) -> outer;
			     p && !extra_references; p = p -> outer) {
				extra_references += p -> refcnt;
				if (p -> outer && p -> outer -> inner == p)
					--extra_references;
				if (p -> inner)
					--extra_references;
				if (p -> handle)
					--extra_references;
			}
		} else
			extra_references = 0;

		if (!extra_references) {
			hp = *h;
			*h = 0;
			hp -> refcnt--;
			if (inner_reference)
				omapi_object_dereference
					(&hp -> inner, file, line);
			if (outer_reference)
				omapi_object_dereference
					(&hp -> outer, file, line);
/*			if (!hp -> type -> freer) */
				rc_register (file, line, h, hp,
					     0, 1, hp -> type -> rc_flag);
			if (handle_reference) {
				if (omapi_handle_clear(hp->handle) != 
				    ISC_R_SUCCESS) {
					log_debug("Attempt to clear null "
						  "handle pointer");
				}
			}
			if (hp -> type -> destroy)
				(*(hp -> type -> destroy)) (hp, file, line);
			if (hp -> type -> freer)
				(hp -> type -> freer (hp, file, line));
			else
				dfree (hp, file, line);
		} else {
			(*h) -> refcnt--;
/*			if (!(*h) -> type -> freer) */
				rc_register (file, line,
					     h, *h, (*h) -> refcnt, 1,
					     (*h) -> type -> rc_flag);
		}
	} else {
		(*h) -> refcnt--;
/*		if (!(*h) -> type -> freer) */
			rc_register (file, line, h, *h, (*h) -> refcnt, 1,
				     (*h) -> type -> rc_flag);
	}
	*h = 0;
	return ISC_R_SUCCESS;
}

isc_result_t omapi_buffer_new (omapi_buffer_t **h,
			       const char *file, int line)
{
	omapi_buffer_t *t;
	isc_result_t status;
	
	t = (omapi_buffer_t *)dmalloc (sizeof *t, file, line);
	if (!t)
		return ISC_R_NOMEMORY;
	memset (t, 0, sizeof *t);
	status = omapi_buffer_reference (h, t, file, line);
	if (status != ISC_R_SUCCESS)
		dfree (t, file, line);
	(*h) -> head = sizeof ((*h) -> buf) - 1;
	return status;
}

isc_result_t omapi_buffer_reference (omapi_buffer_t **r,
				     omapi_buffer_t *h,
				     const char *file, int line)
{
	if (!h || !r)
		return DHCP_R_INVALIDARG;

	if (*r) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): reference store into non-null pointer!",
//ScenSim-Port//			   file, line);
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	*r = h;
	h -> refcnt++;
	rc_register (file, line, r, h, h -> refcnt, 0, RC_MISC);
	return ISC_R_SUCCESS;
}

isc_result_t omapi_buffer_dereference (omapi_buffer_t **h,
				       const char *file, int line)
{
	if (!h)
		return DHCP_R_INVALIDARG;

	if (!*h) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): dereference of null pointer!", file, line);
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	
	if ((*h) -> refcnt <= 0) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): dereference of pointer with refcnt of zero!",
//ScenSim-Port//			   file, line);
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (*h);
//ScenSim-Port//#endif
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*h = 0;
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}

	--(*h) -> refcnt;
	rc_register (file, line, h, *h, (*h) -> refcnt, 1, RC_MISC);
	if ((*h) -> refcnt == 0)
		dfree (*h, file, line);
	*h = 0;
	return ISC_R_SUCCESS;
}

isc_result_t omapi_typed_data_new (const char *file, int line,
				   omapi_typed_data_t **t,
				   omapi_datatype_t type, ...)
{
	va_list l;
	omapi_typed_data_t *new;
	unsigned len;
	unsigned val = 0;
	int intval = 0;
	char *s = NULL;
	isc_result_t status;
	omapi_object_t *obj = NULL;

	va_start (l, type);

	switch (type) {
	      case omapi_datatype_int:
		len = OMAPI_TYPED_DATA_INT_LEN;
		intval = va_arg (l, int);
		break;
	      case omapi_datatype_string:
		s = va_arg (l, char *);
		val = strlen (s);
		len = OMAPI_TYPED_DATA_NOBUFFER_LEN + val;
		if (len < val) {
			va_end(l);
			return DHCP_R_INVALIDARG;
		}
		break;
	      case omapi_datatype_data:
		val = va_arg (l, unsigned);
		len = OMAPI_TYPED_DATA_NOBUFFER_LEN + val;
		if (len < val) {
			va_end(l);
			return DHCP_R_INVALIDARG;
		}
		break;
	      case omapi_datatype_object:
		len = OMAPI_TYPED_DATA_OBJECT_LEN;
		obj = va_arg (l, omapi_object_t *);
		break;
	      default:
		va_end (l);
		return DHCP_R_INVALIDARG;
	}
	va_end (l);

//ScenSim-Port//	new = dmalloc (len, file, line);
	new = (omapi_typed_data_t*)dmalloc (len, file, line);//ScenSim-Port//
	if (!new)
		return ISC_R_NOMEMORY;
	memset (new, 0, len);

	switch (type) {
	      case omapi_datatype_int:
		new -> u.integer = intval;
		break;
	      case omapi_datatype_string:
		memcpy (new -> u.buffer.value, s, val);
		new -> u.buffer.len = val;
		break;
	      case omapi_datatype_data:
		new -> u.buffer.len = val;
		break;
	      case omapi_datatype_object:
		status = omapi_object_reference (&new -> u.object, obj,
						 file, line);
		if (status != ISC_R_SUCCESS) {
			dfree (new, file, line);
			return status;
		}
		break;
	}
	new -> type = type;

	return omapi_typed_data_reference (t, new, file, line);
}

isc_result_t omapi_typed_data_reference (omapi_typed_data_t **r,
					 omapi_typed_data_t *h,
					 const char *file, int line)
{
	if (!h || !r)
		return DHCP_R_INVALIDARG;

	if (*r) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): reference store into non-null pointer!", file, line);
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	*r = h;
	h -> refcnt++;
	rc_register (file, line, r, h, h -> refcnt, 0, RC_MISC);
	return ISC_R_SUCCESS;
}

isc_result_t omapi_typed_data_dereference (omapi_typed_data_t **h,
					   const char *file, int line)
{
	if (!h)
		return DHCP_R_INVALIDARG;

	if (!*h) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): dereference of null pointer!", file, line);
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	
	if ((*h) -> refcnt <= 0) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): dereference of pointer with refcnt of zero!",
//ScenSim-Port//			   file, line);
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (*h);
//ScenSim-Port//#endif
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*h = 0;
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	
	--((*h) -> refcnt);
	rc_register (file, line, h, *h, (*h) -> refcnt, 1, RC_MISC);
	if ((*h) -> refcnt <= 0 ) {
		switch ((*h) -> type) {
		      case omapi_datatype_int:
		      case omapi_datatype_string:
		      case omapi_datatype_data:
		      default:
			break;
		      case omapi_datatype_object:
			omapi_object_dereference (&(*h) -> u.object,
						  file, line);
			break;
		}
		dfree (*h, file, line);
	}
	*h = 0;
	return ISC_R_SUCCESS;
}

isc_result_t omapi_data_string_new (omapi_data_string_t **d, unsigned len,
				    const char *file, int line)
{
	omapi_data_string_t *new;
	unsigned nlen;

	nlen = OMAPI_DATA_STRING_EMPTY_SIZE + len;
	if (nlen < len)
		return DHCP_R_INVALIDARG;
//ScenSim-Port//	new = dmalloc (nlen, file, line);
	new = (omapi_data_string_t*)dmalloc (nlen, file, line);//ScenSim-Port//
	if (!new)
		return ISC_R_NOMEMORY;
	memset (new, 0, OMAPI_DATA_STRING_EMPTY_SIZE);
	new -> len = len;
	return omapi_data_string_reference (d, new, file, line);
}

isc_result_t omapi_data_string_reference (omapi_data_string_t **r,
					  omapi_data_string_t *h,
					  const char *file, int line)
{
	if (!h || !r)
		return DHCP_R_INVALIDARG;

	if (*r) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): reference store into non-null pointer!", file, line);
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	*r = h;
	h -> refcnt++;
	rc_register (file, line, r, h, h -> refcnt, 0, RC_MISC);
	return ISC_R_SUCCESS;
}

isc_result_t omapi_data_string_dereference (omapi_data_string_t **h,
					    const char *file, int line)
{
	if (!h)
		return DHCP_R_INVALIDARG;

	if (!*h) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): dereference of null pointer!", file, line);
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	
	if ((*h) -> refcnt <= 0) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): dereference of pointer with refcnt of zero!",
//ScenSim-Port//			   file, line);
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (*h);
//ScenSim-Port//#endif
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*h = 0;
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}

	--((*h) -> refcnt);
	rc_register (file, line, h, *h, (*h) -> refcnt, 1, RC_MISC);
	if ((*h) -> refcnt <= 0 ) {
		dfree (*h, file, line);
	}
	*h = 0;
	return ISC_R_SUCCESS;
}

isc_result_t omapi_value_new (omapi_value_t **d,
			      const char *file, int line)
{
	omapi_value_t *new;

//ScenSim-Port//	new = dmalloc (sizeof *new, file, line);
	new = (omapi_value_t*)dmalloc (sizeof *new, file, line);//ScenSim-Port//
	if (!new)
		return ISC_R_NOMEMORY;
	memset (new, 0, sizeof *new);
	return omapi_value_reference (d, new, file, line);
}

isc_result_t omapi_value_reference (omapi_value_t **r,
				    omapi_value_t *h,
				    const char *file, int line)
{
	if (!h || !r)
		return DHCP_R_INVALIDARG;

	if (*r) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): reference store into non-null pointer!",
//ScenSim-Port//			   file, line);
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	*r = h;
	h -> refcnt++;
	rc_register (file, line, r, h, h -> refcnt, 0, RC_MISC);
	return ISC_R_SUCCESS;
}

isc_result_t omapi_value_dereference (omapi_value_t **h,
				      const char *file, int line)
{
	if (!h)
		return DHCP_R_INVALIDARG;

	if (!*h) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): dereference of null pointer!", file, line);
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	
	if ((*h) -> refcnt <= 0) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): dereference of pointer with refcnt of zero!",
//ScenSim-Port//			   file, line);
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (*h);
//ScenSim-Port//#endif
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*h = 0;
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	
	--((*h) -> refcnt);
	rc_register (file, line, h, *h, (*h) -> refcnt, 1, RC_MISC);
	if ((*h) -> refcnt == 0) {
		if ((*h) -> name)
			omapi_data_string_dereference (&(*h) -> name,
						       file, line);
		if ((*h) -> value)
			omapi_typed_data_dereference (&(*h) -> value,
						      file, line);
		dfree (*h, file, line);
	}
	*h = 0;
	return ISC_R_SUCCESS;
}

isc_result_t omapi_addr_list_new (omapi_addr_list_t **d, unsigned count,
				  const char *file, int line)
{
	omapi_addr_list_t *new;

//ScenSim-Port//	new = dmalloc ((count * sizeof (omapi_addr_t)) +
	new = (omapi_addr_list_t*)dmalloc ((count * sizeof (omapi_addr_t)) +//ScenSim-Port//
		       sizeof (omapi_addr_list_t), file, line);
	if (!new)
		return ISC_R_NOMEMORY;
	memset (new, 0, ((count * sizeof (omapi_addr_t)) +
			 sizeof (omapi_addr_list_t)));
	new -> count = count;
	new -> addresses = (omapi_addr_t *)(new + 1);
	return omapi_addr_list_reference (d, new, file, line);
}

isc_result_t omapi_addr_list_reference (omapi_addr_list_t **r,
					  omapi_addr_list_t *h,
					  const char *file, int line)
{
	if (!h || !r)
		return DHCP_R_INVALIDARG;

	if (*r) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): reference store into non-null pointer!",
//ScenSim-Port//			   file, line);
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	*r = h;
	h -> refcnt++;
	rc_register (file, line, r, h, h -> refcnt, 0, RC_MISC);
	return ISC_R_SUCCESS;
}

isc_result_t omapi_addr_list_dereference (omapi_addr_list_t **h,
					    const char *file, int line)
{
	if (!h)
		return DHCP_R_INVALIDARG;

	if (!*h) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): dereference of null pointer!", file, line);
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}
	
	if ((*h) -> refcnt <= 0) {
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		log_error ("%s(%d): dereference of pointer with zero refcnt!",
//ScenSim-Port//			   file, line);
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (*h);
//ScenSim-Port//#endif
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*h = 0;
		return DHCP_R_INVALIDARG;
//ScenSim-Port//#endif
	}

	--((*h) -> refcnt);
	rc_register (file, line, h, *h, (*h) -> refcnt, 1, RC_MISC);
	if ((*h) -> refcnt <= 0 ) {
		dfree (*h, file, line);
	}
	*h = 0;
	return ISC_R_SUCCESS;
}

}//namespace IscDhcpPort////ScenSim-Port//
