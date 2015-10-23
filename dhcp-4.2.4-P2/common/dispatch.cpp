/* dispatch.c

   Network input dispatcher... */

/*
 * Copyright (c) 2004-2011 by Internet Systems Consortium, Inc. ("ISC")
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
 */
#include <iscdhcp_porting.h>//ScenSim-Port//
namespace IscDhcpPort {//ScenSim-Port//

//ScenSim-Port//#include "dhcpd.h"

//ScenSim-Port//#include <sys/time.h>

//ScenSim-Port//struct timeout *timeouts;
//ScenSim-Port//static struct timeout *free_timeouts;

//ScenSim-Port//void set_time(TIME t)
//ScenSim-Port//{
//ScenSim-Port//	/* Do any outstanding timeouts. */
//ScenSim-Port//	if (cur_tv . tv_sec != t) {
//ScenSim-Port//		cur_tv . tv_sec = t;
//ScenSim-Port//		cur_tv . tv_usec = 0;
//ScenSim-Port//		process_outstanding_timeouts ((struct timeval *)0);
//ScenSim-Port//	}
//ScenSim-Port//}

//ScenSim-Port//struct timeval *process_outstanding_timeouts (struct timeval *tvp)
//ScenSim-Port//{
//ScenSim-Port//	/* Call any expired timeouts, and then if there's
//ScenSim-Port//	   still a timeout registered, time out the select
//ScenSim-Port//	   call then. */
//ScenSim-Port//      another:
//ScenSim-Port//	if (timeouts) {
//ScenSim-Port//		struct timeout *t;
//ScenSim-Port//		if ((timeouts -> when . tv_sec < cur_tv . tv_sec) ||
//ScenSim-Port//		    ((timeouts -> when . tv_sec == cur_tv . tv_sec) &&
//ScenSim-Port//		     (timeouts -> when . tv_usec <= cur_tv . tv_usec))) {
//ScenSim-Port//			t = timeouts;
//ScenSim-Port//			timeouts = timeouts -> next;
//ScenSim-Port//			(*(t -> func)) (t -> what);
//ScenSim-Port//			if (t -> unref)
//ScenSim-Port//				(*t -> unref) (&t -> what, MDL);
//ScenSim-Port//			t -> next = free_timeouts;
//ScenSim-Port//			free_timeouts = t;
//ScenSim-Port//			goto another;
//ScenSim-Port//		}
//ScenSim-Port//		if (tvp) {
//ScenSim-Port//			tvp -> tv_sec = timeouts -> when . tv_sec;
//ScenSim-Port//			tvp -> tv_usec = timeouts -> when . tv_usec;
//ScenSim-Port//		}
//ScenSim-Port//		return tvp;
//ScenSim-Port//	} else
//ScenSim-Port//		return (struct timeval *)0;
//ScenSim-Port//}

/* Wait for packets to come in using select().   When one does, call
   receive_packet to receive the packet and possibly strip hardware
   addressing information from it, and then call through the
   bootp_packet_handler hook to try to do something with it. */

/*
 * Use the DHCP timeout list as a place to store DHCP specific
 * information, but use the ISC timer system to actually dispatch
 * the events.
 *
 * There are several things that the DHCP timer code does that the
 * ISC code doesn't:
 * 1) It allows for negative times
 * 2) The cancel arguments are different.  The DHCP code uses the
 * function and data to find the proper timer to cancel while the
 * ISC code uses a pointer to the timer.
 * 3) The DHCP code includes provision for incrementing and decrementing
 * a reference counter associated with the data.
 * The first one is fairly easy to fix but will take some time to go throuh
 * the callers and update them.  The second is also not all that difficult
 * in concept - add a pointer to the appropriate structures to hold a pointer
 * to the timer and use that.  The complications arise in trying to ensure
 * that all of the corner cases are covered.  The last one is potentially
 * more painful and requires more investigation.
 * 
 * The plan is continue with the older DHCP calls and timer list.  The
 * calls will continue to manipulate the list but will also pass a
 * timer to the ISC timer code for the actual dispatch.  Later, if desired,
 * we can go back and modify the underlying calls to use the ISC
 * timer functions directly without requiring all of the code to change
 * at the same time.
 */

void
dispatch(void)
{
//ScenSim-Port//	isc_result_t status;
//ScenSim-Port//
//ScenSim-Port//	status = isc_app_ctxrun(dhcp_gbl_ctx.actx);
//ScenSim-Port//
//ScenSim-Port//	log_fatal ("Dispatch routine failed: %s -- exiting",
//ScenSim-Port//		   isc_result_totext (status));
}

//ScenSim-Port//void
//ScenSim-Port//isclib_timer_callback(isc_task_t  *taskp,
//ScenSim-Port//		      isc_event_t *eventp)
//ScenSim-Port//{
//ScenSim-Port//	struct timeout *t = (struct timeout *)eventp->ev_arg;
//ScenSim-Port//	struct timeout *q, *r;
//ScenSim-Port//
//ScenSim-Port//	/* Get the current time... */
//ScenSim-Port//	gettimeofday (&cur_tv, (struct timezone *)0);
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * Find the timeout on the dhcp list and remove it.
//ScenSim-Port//	 * As the list isn't ordered we search the entire list
//ScenSim-Port//	 */
//ScenSim-Port//
//ScenSim-Port//	r = NULL;
//ScenSim-Port//	for (q = timeouts; q; q = q->next) {
//ScenSim-Port//		if (q == t) {
//ScenSim-Port//			if (r)
//ScenSim-Port//				r->next = q->next;
//ScenSim-Port//			else
//ScenSim-Port//				timeouts = q->next;
//ScenSim-Port//			break;
//ScenSim-Port//		}
//ScenSim-Port//		r = q;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * The timer should always be on the list.  If it is we do
//ScenSim-Port//	 * the work and detach the timer block, if not we log an error.
//ScenSim-Port//	 * In both cases we attempt free the ISC event and continue
//ScenSim-Port//	 * processing.
//ScenSim-Port//	 */
//ScenSim-Port//
//ScenSim-Port//	if (q != NULL) {
//ScenSim-Port//		/* call the callback function */
//ScenSim-Port//		(*(q->func)) (q->what);
//ScenSim-Port//		if (q->unref) {
//ScenSim-Port//			(*q->unref) (&q->what, MDL);
//ScenSim-Port//		}
//ScenSim-Port//		q->next = free_timeouts;
//ScenSim-Port//		isc_timer_detach(&q->isc_timeout);
//ScenSim-Port//		free_timeouts = q;
//ScenSim-Port//	} else {
//ScenSim-Port//		/*
//ScenSim-Port//		 * Hmm, we should clean up the timer structure but aren't
//ScenSim-Port//		 * sure about the pointer to the timer block we got so
//ScenSim-Port//		 * don't try to - may change this to a log_fatal
//ScenSim-Port//		 */
//ScenSim-Port//		log_error("Error finding timer structure");
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	isc_event_free(&eventp);
//ScenSim-Port//	return;
//ScenSim-Port//}

/* maximum value for usec */
//ScenSim-Port//#define USEC_MAX 1000000
//ScenSim-Port//#define DHCP_SEC_MAX  0xFFFFFFFF

//ScenSim-Port//void add_timeout (when, where, what, ref, unref)
//ScenSim-Port//	struct timeval *when;
//ScenSim-Port//	void (*where) (void *);
//ScenSim-Port//	void *what;
//ScenSim-Port//	tvref_t ref;
//ScenSim-Port//	tvunref_t unref;
void add_timeout (struct timeval *when, void (*where) (void *), void *what, tvref_t ref, tvunref_t unref)//ScenSim-Port//
{
//ScenSim-Port//	struct timeout *t, *q;
//ScenSim-Port//	int usereset = 0;
//ScenSim-Port//	isc_result_t status;
//ScenSim-Port//	int64_t sec;
//ScenSim-Port//	int usec;
//ScenSim-Port//	isc_interval_t interval;
//ScenSim-Port//	isc_time_t expires;
//ScenSim-Port//
//ScenSim-Port//	/* See if this timeout supersedes an existing timeout. */
//ScenSim-Port//	t = (struct timeout *)0;
//ScenSim-Port//	for (q = timeouts; q; q = q->next) {
//ScenSim-Port//		if ((where == NULL || q->func == where) &&
//ScenSim-Port//		    q->what == what) {
//ScenSim-Port//			if (t)
//ScenSim-Port//				t->next = q->next;
//ScenSim-Port//			else
//ScenSim-Port//				timeouts = q->next;
//ScenSim-Port//			usereset = 1;
//ScenSim-Port//			break;
//ScenSim-Port//		}
//ScenSim-Port//		t = q;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/* If we didn't supersede a timeout, allocate a timeout
//ScenSim-Port//	   structure now. */
//ScenSim-Port//	if (!q) {
//ScenSim-Port//		if (free_timeouts) {
//ScenSim-Port//			q = free_timeouts;
//ScenSim-Port//			free_timeouts = q->next;
//ScenSim-Port//		} else {
//ScenSim-Port//			q = ((struct timeout *)
//ScenSim-Port//			     dmalloc(sizeof(struct timeout), MDL));
//ScenSim-Port//			if (!q) {
//ScenSim-Port//				log_fatal("add_timeout: no memory!");
//ScenSim-Port//			}
//ScenSim-Port//		}
//ScenSim-Port//		memset(q, 0, sizeof *q);
//ScenSim-Port//		q->func = where;
//ScenSim-Port//		q->ref = ref;
//ScenSim-Port//		q->unref = unref;
//ScenSim-Port//		if (q->ref)
//ScenSim-Port//			(*q->ref)(&q->what, what, MDL);
//ScenSim-Port//		else
//ScenSim-Port//			q->what = what;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * The value passed in is a time from an epoch but we need a relative
//ScenSim-Port//	 * time so we need to do some math to try and recover the period.
//ScenSim-Port//	 * This is complicated by the fact that not all of the calls cared
//ScenSim-Port//	 * about the usec value, if it's zero we assume the caller didn't care.
//ScenSim-Port//	 *
//ScenSim-Port//	 * The ISC timer library doesn't seem to like negative values
//ScenSim-Port//	 * and can't accept any values above 4G-1 seconds so we limit
//ScenSim-Port//	 * the values to 0 <= value < 4G-1.  We do it before
//ScenSim-Port//	 * checking the trace option so that both the trace code and
//ScenSim-Port//	 * the working code use the same values.
//ScenSim-Port//	 */
//ScenSim-Port//
//ScenSim-Port//	sec  = when->tv_sec - cur_tv.tv_sec;
//ScenSim-Port//	usec = when->tv_usec - cur_tv.tv_usec;
//ScenSim-Port//	
//ScenSim-Port//	if ((when->tv_usec != 0) && (usec < 0)) {
//ScenSim-Port//		sec--;
//ScenSim-Port//		usec += USEC_MAX;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (sec < 0) {
//ScenSim-Port//		sec  = 0;
//ScenSim-Port//		usec = 0;
//ScenSim-Port//	} else if (sec > DHCP_SEC_MAX) {
//ScenSim-Port//		log_error("Timeout requested too large "
//ScenSim-Port//			  "reducing to 2^^32-1");
//ScenSim-Port//		sec = DHCP_SEC_MAX;
//ScenSim-Port//		usec = 0;
//ScenSim-Port//	} else if (usec < 0) {
//ScenSim-Port//		usec = 0;
//ScenSim-Port//	} else if (usec >= USEC_MAX) {
//ScenSim-Port//		usec = USEC_MAX - 1;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/* 
//ScenSim-Port//	 * This is necessary for the tracing code but we put it
//ScenSim-Port//	 * here in case we want to compare timing information
//ScenSim-Port//	 * for some reason, like debugging.
//ScenSim-Port//	 */
//ScenSim-Port//	q->when.tv_sec  = cur_tv.tv_sec + (sec & DHCP_SEC_MAX);
//ScenSim-Port//	q->when.tv_usec = usec;
//ScenSim-Port//
//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//	if (trace_playback()) {
//ScenSim-Port//		/*
//ScenSim-Port//		 * If we are doing playback we need to handle the timers
//ScenSim-Port//		 * within this code rather than having the isclib handle
//ScenSim-Port//		 * them for us.  We need to keep the timer list in order
//ScenSim-Port//		 * to allow us to find the ones to timeout.
//ScenSim-Port//		 *
//ScenSim-Port//		 * By using a different timer setup in the playback we may
//ScenSim-Port//		 * have variations between the orginal and the playback but
//ScenSim-Port//		 * it's the best we can do for now.
//ScenSim-Port//		 */
//ScenSim-Port//
//ScenSim-Port//		/* Beginning of list? */
//ScenSim-Port//		if (!timeouts || (timeouts->when.tv_sec > q-> when.tv_sec) ||
//ScenSim-Port//		    ((timeouts->when.tv_sec == q->when.tv_sec) &&
//ScenSim-Port//		     (timeouts->when.tv_usec > q->when.tv_usec))) {
//ScenSim-Port//			q->next = timeouts;
//ScenSim-Port//			timeouts = q;
//ScenSim-Port//			return;
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//		/* Middle of list? */
//ScenSim-Port//		for (t = timeouts; t->next; t = t->next) {
//ScenSim-Port//			if ((t->next->when.tv_sec > q->when.tv_sec) ||
//ScenSim-Port//			    ((t->next->when.tv_sec == q->when.tv_sec) &&
//ScenSim-Port//			     (t->next->when.tv_usec > q->when.tv_usec))) {
//ScenSim-Port//				q->next = t->next;
//ScenSim-Port//				t->next = q;
//ScenSim-Port//				return;
//ScenSim-Port//			}
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//		/* End of list. */
//ScenSim-Port//		t->next = q;
//ScenSim-Port//		q->next = (struct timeout *)0;
//ScenSim-Port//		return;
//ScenSim-Port//	}
//ScenSim-Port//#endif
//ScenSim-Port//	/*
//ScenSim-Port//	 * Don't bother sorting the DHCP list, just add it to the front.
//ScenSim-Port//	 * Eventually the list should be removed as we migrate the callers
//ScenSim-Port//	 * to the native ISC timer functions, if it becomes a performance
//ScenSim-Port//	 * problem before then we may need to order the list.
//ScenSim-Port//	 */
//ScenSim-Port//	q->next  = timeouts;
//ScenSim-Port//	timeouts = q;
//ScenSim-Port//
//ScenSim-Port//	isc_interval_set(&interval, sec & DHCP_SEC_MAX, usec * 1000);
//ScenSim-Port//	status = isc_time_nowplusinterval(&expires, &interval);
//ScenSim-Port//	if (status != ISC_R_SUCCESS) {
//ScenSim-Port//		/*
//ScenSim-Port//		 * The system time function isn't happy or returned
//ScenSim-Port//		 * a value larger than isc_time_t can hold.
//ScenSim-Port//		 */
//ScenSim-Port//		log_fatal("Unable to set up timer: %s",
//ScenSim-Port//			  isc_result_totext(status));
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (usereset == 0) {
//ScenSim-Port//		status = isc_timer_create(dhcp_gbl_ctx.timermgr,
//ScenSim-Port//					  isc_timertype_once, &expires,
//ScenSim-Port//					  NULL, dhcp_gbl_ctx.task,
//ScenSim-Port//					  isclib_timer_callback,
//ScenSim-Port//					  (void *)q, &q->isc_timeout);
//ScenSim-Port//	} else {
//ScenSim-Port//		status = isc_timer_reset(q->isc_timeout,
//ScenSim-Port//					 isc_timertype_once, &expires,
//ScenSim-Port//					 NULL, 0);
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/* If it fails log an error and die */
//ScenSim-Port//	if (status != ISC_R_SUCCESS) {
//ScenSim-Port//		log_fatal("Unable to add timeout to isclib\n");
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	return;
    AddTimer(curctx->glue, when, where, what);//ScenSim-Port//
}

//ScenSim-Port//void cancel_timeout (where, what)
//ScenSim-Port//	void (*where) (void *);
//ScenSim-Port//	void *what;
void cancel_timeout (void (*where) (void *), void *what)//ScenSim-Port//
{
//ScenSim-Port//	struct timeout *t, *q;
//ScenSim-Port//
//ScenSim-Port//	/* Look for this timeout on the list, and unlink it if we find it. */
//ScenSim-Port//	t = (struct timeout *)0;
//ScenSim-Port//	for (q = timeouts; q; q = q -> next) {
//ScenSim-Port//		if (q->func == where && q->what == what) {
//ScenSim-Port//			if (t)
//ScenSim-Port//				t->next = q->next;
//ScenSim-Port//			else
//ScenSim-Port//				timeouts = q->next;
//ScenSim-Port//			break;
//ScenSim-Port//		}
//ScenSim-Port//		t = q;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * If we found the timeout, cancel it and put it on the free list.
//ScenSim-Port//	 * The TRACING stuff is ugly but we don't add a timer when doing
//ScenSim-Port//	 * playback so we don't want to remove them then either.
//ScenSim-Port//	 */
//ScenSim-Port//	if (q) {
//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//		if (!trace_playback()) {
//ScenSim-Port//#endif
//ScenSim-Port//			isc_timer_detach(&q->isc_timeout);
//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//		}
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//		if (q->unref)
//ScenSim-Port//			(*q->unref) (&q->what, MDL);
//ScenSim-Port//		q->next = free_timeouts;
//ScenSim-Port//		free_timeouts = q;
//ScenSim-Port//	}
    CancelTimer(curctx->glue, where, what);//ScenSim-Port//
}

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
//ScenSim-Port//void cancel_all_timeouts ()
//ScenSim-Port//{
//ScenSim-Port//	struct timeout *t, *n;
//ScenSim-Port//	for (t = timeouts; t; t = n) {
//ScenSim-Port//		n = t->next;
//ScenSim-Port//		isc_timer_detach(&t->isc_timeout);
//ScenSim-Port//		if (t->unref && t->what)
//ScenSim-Port//			(*t->unref) (&t->what, MDL);
//ScenSim-Port//		t->next = free_timeouts;
//ScenSim-Port//		free_timeouts = t;
//ScenSim-Port//	}
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//void relinquish_timeouts ()
//ScenSim-Port//{
//ScenSim-Port//	struct timeout *t, *n;
//ScenSim-Port//	for (t = free_timeouts; t; t = n) {
//ScenSim-Port//		n = t->next;
//ScenSim-Port//		dfree(t, MDL);
//ScenSim-Port//	}
//ScenSim-Port//}
//ScenSim-Port//#endif
}//namespace IscDhcpPort////ScenSim-Port//
