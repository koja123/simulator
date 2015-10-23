/* dispatch.c

   I/O dispatcher. */

/*
 * Copyright (c) 2004,2007-2009 by Internet Systems Consortium, Inc. ("ISC")
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
//ScenSim-Port//#include <sys/time.h>

//ScenSim-Port//static omapi_io_object_t omapi_io_states;
//ScenSim-Port//struct timeval cur_tv;

//ScenSim-Port//struct eventqueue *rw_queue_empty;

//ScenSim-Port//OMAPI_OBJECT_ALLOC (omapi_io,
//ScenSim-Port//		    omapi_io_object_t, omapi_type_io_object)
//ScenSim-Port//OMAPI_OBJECT_ALLOC (omapi_waiter,
//ScenSim-Port//		    omapi_waiter_object_t, omapi_type_waiter)

//ScenSim-Port//void
//ScenSim-Port//register_eventhandler(struct eventqueue **queue, void (*handler)(void *))
//ScenSim-Port//{
//ScenSim-Port//	struct eventqueue *t, *q;
//ScenSim-Port//
//ScenSim-Port//	/* traverse to end of list */
//ScenSim-Port//	t = NULL;
//ScenSim-Port//	for (q = *queue ; q ; q = q->next) {
//ScenSim-Port//		if (q->handler == handler)
//ScenSim-Port//			return; /* handler already registered */
//ScenSim-Port//		t = q;
//ScenSim-Port//	}
//ScenSim-Port//		
//ScenSim-Port//	q = ((struct eventqueue *)dmalloc(sizeof(struct eventqueue), MDL));
//ScenSim-Port//	if (!q)
//ScenSim-Port//		log_fatal("register_eventhandler: no memory!");
//ScenSim-Port//	memset(q, 0, sizeof *q);
//ScenSim-Port//	if (t)
//ScenSim-Port//		t->next = q;
//ScenSim-Port//	else 
//ScenSim-Port//		*queue	= q;
//ScenSim-Port//	q->handler = handler;
//ScenSim-Port//	return;
//ScenSim-Port//}

//ScenSim-Port//void
//ScenSim-Port//unregister_eventhandler(struct eventqueue **queue, void (*handler)(void *))
//ScenSim-Port//{
//ScenSim-Port//	struct eventqueue *t, *q;
//ScenSim-Port//	
//ScenSim-Port//	/* traverse to end of list */
//ScenSim-Port//	t= NULL;
//ScenSim-Port//	for (q = *queue ; q ; q = q->next) {
//ScenSim-Port//		if (q->handler == handler) {
//ScenSim-Port//			if (t)
//ScenSim-Port//				t->next = q->next;
//ScenSim-Port//			else
//ScenSim-Port//				*queue = q->next;
//ScenSim-Port//			dfree(q, MDL); /* Don't access q after this!*/
//ScenSim-Port//			break;
//ScenSim-Port//		}
//ScenSim-Port//		t = q;
//ScenSim-Port//	}
//ScenSim-Port//	return;
//ScenSim-Port//}

//ScenSim-Port//void
//ScenSim-Port//trigger_event(struct eventqueue **queue)
//ScenSim-Port//{
//ScenSim-Port//	struct eventqueue *q;
//ScenSim-Port//
//ScenSim-Port//	for (q=*queue ; q ; q=q->next) {
//ScenSim-Port//		if (q->handler) 
//ScenSim-Port//			(*q->handler)(NULL);
//ScenSim-Port//	}
//ScenSim-Port//}

/*
 * Callback routine to connect the omapi I/O object and socket with
 * the isc socket code.  The isc socket code will call this routine
 * which will then call the correct local routine to process the bytes.
 * 
 * Currently we are always willing to read more data, this should be modified
 * so that on connections we don't read more if we already have enough.
 *
 * If we have more bytes to write we ask the library to call us when
 * we can write more.  If we indicate we don't have more to write we need
 * to poke the library via isc_socket_fdwatchpoke.
 */

/*
 * sockdelete indicates if we are deleting the socket or leaving it in place
 * 1 is delete, 0 is leave in place
 */
//ScenSim-Port//#define SOCKDELETE 1
//ScenSim-Port//int
//ScenSim-Port//omapi_iscsock_cb(isc_task_t   *task,
//ScenSim-Port//		 isc_socket_t *socket,
//ScenSim-Port//		 void         *cbarg,
//ScenSim-Port//		 int           flags)
//ScenSim-Port//{
//ScenSim-Port//	omapi_io_object_t *obj;
//ScenSim-Port//	isc_result_t status;
//ScenSim-Port//
//ScenSim-Port//	/* Get the current time... */
//ScenSim-Port//	gettimeofday (&cur_tv, (struct timezone *)0);
//ScenSim-Port//
//ScenSim-Port//	/* isc socket stuff */
//ScenSim-Port//#if SOCKDELETE
//ScenSim-Port//	/*
//ScenSim-Port//	 * walk through the io states list, if our object is on there
//ScenSim-Port//	 * service it.  if not ignore it.
//ScenSim-Port//	 */
//ScenSim-Port//	for (obj = omapi_io_states.next;
//ScenSim-Port//	     (obj != NULL) && (obj->next != NULL);
//ScenSim-Port//	     obj = obj->next) {
//ScenSim-Port//		if (obj == cbarg)
//ScenSim-Port//			break;
//ScenSim-Port//	}
//ScenSim-Port//	if (obj == NULL) {
//ScenSim-Port//		return(0);
//ScenSim-Port//	}
//ScenSim-Port//#else
//ScenSim-Port//	/* Not much to be done if we have the wrong type of object. */
//ScenSim-Port//	if (((omapi_object_t *)cbarg) -> type != omapi_type_io_object) {
//ScenSim-Port//		log_fatal ("Incorrect object type, must be of type io_object");
//ScenSim-Port//	}
//ScenSim-Port//	obj = (omapi_io_object_t *)cbarg;
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * If the object is marked as closed don't try and process
//ScenSim-Port//	 * anything just indicate that we don't want any more.
//ScenSim-Port//	 *
//ScenSim-Port//	 * This should be a temporary fix until we arrange to properly
//ScenSim-Port//	 * close the socket.
//ScenSim-Port//	 */
//ScenSim-Port//	if (obj->closed == ISC_TRUE) {
//ScenSim-Port//		return(0);
//ScenSim-Port//	}
//ScenSim-Port//#endif	  
//ScenSim-Port//
//ScenSim-Port//	if ((flags == ISC_SOCKFDWATCH_READ) &&
//ScenSim-Port//	    (obj->reader != NULL) &&
//ScenSim-Port//	    (obj->inner != NULL)) {
//ScenSim-Port//		obj->reader(obj->inner);
//ScenSim-Port//		/* We always ask for more when reading */
//ScenSim-Port//		return (1);
//ScenSim-Port//	} else if ((flags == ISC_SOCKFDWATCH_WRITE) &&
//ScenSim-Port//		 (obj->writer != NULL) &&
//ScenSim-Port//		 (obj->inner != NULL)) {
//ScenSim-Port//		status = obj->writer(obj->inner);
//ScenSim-Port//		/* If the writer has more to write they should return
//ScenSim-Port//		 * ISC_R_INPROGRESS */
//ScenSim-Port//		if (status == ISC_R_INPROGRESS) {
//ScenSim-Port//			return (1);
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * We get here if we either had an error (inconsistent
//ScenSim-Port//	 * structures etc) or no more to write, tell the socket
//ScenSim-Port//	 * lib we don't have more to do right now.
//ScenSim-Port//	 */
//ScenSim-Port//	return (0);
//ScenSim-Port//}

/* Register an I/O handle so that we can do asynchronous I/O on it. */

isc_result_t omapi_register_io_object (omapi_object_t *h,
				       int (*readfd) (omapi_object_t *),
				       int (*writefd) (omapi_object_t *),
				       isc_result_t (*reader)
						(omapi_object_t *),
				       isc_result_t (*writer)
						(omapi_object_t *),
				       isc_result_t (*reaper)
						(omapi_object_t *))
{
//ScenSim-Port//	isc_result_t status;
//ScenSim-Port//	omapi_io_object_t *obj, *p;
//ScenSim-Port//	int fd_flags = 0, fd = 0;
//ScenSim-Port//
//ScenSim-Port//	/* omapi_io_states is a static object.   If its reference count
//ScenSim-Port//	   is zero, this is the first I/O handle to be registered, so
//ScenSim-Port//	   we need to initialize it.   Because there is no inner or outer
//ScenSim-Port//	   pointer on this object, and we're setting its refcnt to 1, it
//ScenSim-Port//	   will never be freed. */
//ScenSim-Port//	if (!omapi_io_states.refcnt) {
//ScenSim-Port//		omapi_io_states.refcnt = 1;
//ScenSim-Port//		omapi_io_states.type = omapi_type_io_object;
//ScenSim-Port//	}
//ScenSim-Port//		
//ScenSim-Port//	obj = (omapi_io_object_t *)0;
//ScenSim-Port//	status = omapi_io_allocate (&obj, MDL);
//ScenSim-Port//	if (status != ISC_R_SUCCESS)
//ScenSim-Port//		return status;
//ScenSim-Port//	obj->closed = ISC_FALSE;  /* mark as open */
//ScenSim-Port//
//ScenSim-Port//	status = omapi_object_reference (&obj -> inner, h, MDL);
//ScenSim-Port//	if (status != ISC_R_SUCCESS) {
//ScenSim-Port//		omapi_io_dereference (&obj, MDL);
//ScenSim-Port//		return status;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	status = omapi_object_reference (&h -> outer,
//ScenSim-Port//					 (omapi_object_t *)obj, MDL);
//ScenSim-Port//	if (status != ISC_R_SUCCESS) {
//ScenSim-Port//		omapi_io_dereference (&obj, MDL);
//ScenSim-Port//		return status;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/*
//ScenSim-Port//	 * Attach the I/O object to the isc socket library via the 
//ScenSim-Port//	 * fdwatch function.  This allows the socket library to watch
//ScenSim-Port//	 * over a socket that we built.  If there are both a read and
//ScenSim-Port//	 * a write socket we asssume they are the same socket.
//ScenSim-Port//	 */
//ScenSim-Port//
//ScenSim-Port//	if (readfd) {
//ScenSim-Port//		fd_flags |= ISC_SOCKFDWATCH_READ;
//ScenSim-Port//		fd = readfd(h);
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (writefd) {
//ScenSim-Port//		fd_flags |= ISC_SOCKFDWATCH_WRITE;
//ScenSim-Port//		fd = writefd(h);
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (fd_flags != 0) {
//ScenSim-Port//		status = isc_socket_fdwatchcreate(dhcp_gbl_ctx.socketmgr,
//ScenSim-Port//						  fd, fd_flags,
//ScenSim-Port//						  omapi_iscsock_cb,
//ScenSim-Port//						  obj,
//ScenSim-Port//						  dhcp_gbl_ctx.task,
//ScenSim-Port//						  &obj->fd);
//ScenSim-Port//		if (status != ISC_R_SUCCESS) {
//ScenSim-Port//			log_error("Unable to register fd with library %s",
//ScenSim-Port//				   isc_result_totext(status));
//ScenSim-Port//
//ScenSim-Port//			/*sar*/
//ScenSim-Port//			/* is this the cleanup we need? */
//ScenSim-Port//			omapi_object_dereference(&h->outer, MDL);
//ScenSim-Port//			omapi_io_dereference (&obj, MDL);
//ScenSim-Port//			return (status);
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//
//ScenSim-Port//	/* Find the last I/O state, if there are any. */
//ScenSim-Port//	for (p = omapi_io_states.next;
//ScenSim-Port//	     p && p -> next; p = p -> next)
//ScenSim-Port//		;
//ScenSim-Port//	if (p)
//ScenSim-Port//		omapi_io_reference (&p -> next, obj, MDL);
//ScenSim-Port//	else
//ScenSim-Port//		omapi_io_reference (&omapi_io_states.next, obj, MDL);
//ScenSim-Port//
//ScenSim-Port//	obj -> readfd = readfd;
//ScenSim-Port//	obj -> writefd = writefd;
//ScenSim-Port//	obj -> reader = reader;
//ScenSim-Port//	obj -> writer = writer;
//ScenSim-Port//	obj -> reaper = reaper;
//ScenSim-Port//
//ScenSim-Port//	omapi_io_dereference(&obj, MDL);
	return ISC_R_SUCCESS;
}

/*
 * ReRegister an I/O handle so that we can do asynchronous I/O on it.
 * If the handle doesn't exist we call the register routine to build it.
 * If it does exist we change the functions associated with it, and
 * repoke the fd code to make it happy.  Neither the objects nor the
 * fd are allowed to have changed.
 */

isc_result_t omapi_reregister_io_object (omapi_object_t *h,
					 int (*readfd) (omapi_object_t *),
					 int (*writefd) (omapi_object_t *),
					 isc_result_t (*reader)
					 	(omapi_object_t *),
					 isc_result_t (*writer)
					 	(omapi_object_t *),
					 isc_result_t (*reaper)
					 	(omapi_object_t *))
{
//ScenSim-Port//	omapi_io_object_t *obj;
//ScenSim-Port//	int fd_flags = 0;
//ScenSim-Port//
//ScenSim-Port//	if ((!h -> outer) || (h -> outer -> type != omapi_type_io_object)) {
//ScenSim-Port//		/*
//ScenSim-Port//		 * If we don't have an object or if the type isn't what 
//ScenSim-Port//		 * we expect do the normal registration (which will overwrite
//ScenSim-Port//		 * an incorrect type, that's what we did historically, may
//ScenSim-Port//		 * want to change that)
//ScenSim-Port//		 */
//ScenSim-Port//		return (omapi_register_io_object (h, readfd, writefd,
//ScenSim-Port//						  reader, writer, reaper));
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/* We have an io object of the correct type, try to update it */
//ScenSim-Port//	/*sar*/
//ScenSim-Port//	/* Should we validate that the fd matches the previous one?
//ScenSim-Port//	 * It's suppossed to, that's a requirement, don't bother yet */
//ScenSim-Port//
//ScenSim-Port//	obj = (omapi_io_object_t *)h->outer;
//ScenSim-Port//
//ScenSim-Port//	obj->readfd = readfd;
//ScenSim-Port//	obj->writefd = writefd;
//ScenSim-Port//	obj->reader = reader;
//ScenSim-Port//	obj->writer = writer;
//ScenSim-Port//	obj->reaper = reaper;
//ScenSim-Port//
//ScenSim-Port//	if (readfd) {
//ScenSim-Port//		fd_flags |= ISC_SOCKFDWATCH_READ;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (writefd) {
//ScenSim-Port//		fd_flags |= ISC_SOCKFDWATCH_WRITE;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	isc_socket_fdwatchpoke(obj->fd, fd_flags);
//ScenSim-Port//	
	return (ISC_R_SUCCESS);
}

isc_result_t omapi_unregister_io_object (omapi_object_t *h)
{
//ScenSim-Port//	omapi_io_object_t *obj, *ph;
//ScenSim-Port//#if SOCKDELETE
//ScenSim-Port//	omapi_io_object_t *p, *last; 
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//	if (!h -> outer || h -> outer -> type != omapi_type_io_object)
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//	obj = (omapi_io_object_t *)h -> outer;
//ScenSim-Port//	ph = (omapi_io_object_t *)0;
//ScenSim-Port//	omapi_io_reference (&ph, obj, MDL);
//ScenSim-Port//
//ScenSim-Port//#if SOCKDELETE
//ScenSim-Port//	/*
//ScenSim-Port//	 * For now we leave this out.  We can't clean up the isc socket
//ScenSim-Port//	 * structure cleanly yet so we need to leave the io object in place.
//ScenSim-Port//	 * By leaving it on the io states list we avoid it being freed.
//ScenSim-Port//	 * We also mark it as closed to avoid using it.
//ScenSim-Port//	 */
//ScenSim-Port//
//ScenSim-Port//	/* remove from the list of I/O states */
//ScenSim-Port//        last = &omapi_io_states;
//ScenSim-Port//	for (p = omapi_io_states.next; p; p = p -> next) {
//ScenSim-Port//		if (p == obj) {
//ScenSim-Port//			omapi_io_dereference (&last -> next, MDL);
//ScenSim-Port//			omapi_io_reference (&last -> next, p -> next, MDL);
//ScenSim-Port//			break;
//ScenSim-Port//		}
//ScenSim-Port//		last = p;
//ScenSim-Port//	}
//ScenSim-Port//	if (obj -> next)
//ScenSim-Port//		omapi_io_dereference (&obj -> next, MDL);
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//	if (obj -> outer) {
//ScenSim-Port//		if (obj -> outer -> inner == (omapi_object_t *)obj)
//ScenSim-Port//			omapi_object_dereference (&obj -> outer -> inner,
//ScenSim-Port//						  MDL);
//ScenSim-Port//		omapi_object_dereference (&obj -> outer, MDL);
//ScenSim-Port//	}
//ScenSim-Port//	omapi_object_dereference (&obj -> inner, MDL);
//ScenSim-Port//	omapi_object_dereference (&h -> outer, MDL);
//ScenSim-Port//
//ScenSim-Port//#if SOCKDELETE
//ScenSim-Port//	/* remove isc socket associations */
//ScenSim-Port//	if (obj->fd != NULL) {
//ScenSim-Port//		isc_socket_cancel(obj->fd, dhcp_gbl_ctx.task,
//ScenSim-Port//				  ISC_SOCKCANCEL_ALL);
//ScenSim-Port//		isc_socket_detach(&obj->fd);
//ScenSim-Port//	}
//ScenSim-Port//#else
//ScenSim-Port//	obj->closed = ISC_TRUE;
//ScenSim-Port//#endif
//ScenSim-Port//
//ScenSim-Port//	omapi_io_dereference (&ph, MDL);
	return ISC_R_SUCCESS;
}

//ScenSim-Port//isc_result_t omapi_dispatch (struct timeval *t)
//ScenSim-Port//{
//ScenSim-Port//	return omapi_wait_for_completion ((omapi_object_t *)&omapi_io_states,
//ScenSim-Port//					  t);
//ScenSim-Port//}

//ScenSim-Port//isc_result_t omapi_wait_for_completion (omapi_object_t *object,
//ScenSim-Port//					struct timeval *t)
//ScenSim-Port//{
//ScenSim-Port//	isc_result_t status;
//ScenSim-Port//	omapi_waiter_object_t *waiter;
//ScenSim-Port//	omapi_object_t *inner;
//ScenSim-Port//
//ScenSim-Port//	if (object) {
//ScenSim-Port//		waiter = (omapi_waiter_object_t *)0;
//ScenSim-Port//		status = omapi_waiter_allocate (&waiter, MDL);
//ScenSim-Port//		if (status != ISC_R_SUCCESS)
//ScenSim-Port//			return status;
//ScenSim-Port//
//ScenSim-Port//		/* Paste the waiter object onto the inner object we're
//ScenSim-Port//		   waiting on. */
//ScenSim-Port//		for (inner = object; inner -> inner; inner = inner -> inner)
//ScenSim-Port//			;
//ScenSim-Port//
//ScenSim-Port//		status = omapi_object_reference (&waiter -> outer, inner, MDL);
//ScenSim-Port//		if (status != ISC_R_SUCCESS) {
//ScenSim-Port//			omapi_waiter_dereference (&waiter, MDL);
//ScenSim-Port//			return status;
//ScenSim-Port//		}
//ScenSim-Port//		
//ScenSim-Port//		status = omapi_object_reference (&inner -> inner,
//ScenSim-Port//						 (omapi_object_t *)waiter,
//ScenSim-Port//						 MDL);
//ScenSim-Port//		if (status != ISC_R_SUCCESS) {
//ScenSim-Port//			omapi_waiter_dereference (&waiter, MDL);
//ScenSim-Port//			return status;
//ScenSim-Port//		}
//ScenSim-Port//	} else
//ScenSim-Port//		waiter = (omapi_waiter_object_t *)0;
//ScenSim-Port//
//ScenSim-Port//	do {
//ScenSim-Port//		status = omapi_one_dispatch ((omapi_object_t *)waiter, t);
//ScenSim-Port//		if (status != ISC_R_SUCCESS)
//ScenSim-Port//			return status;
//ScenSim-Port//	} while (!waiter || !waiter -> ready);
//ScenSim-Port//
//ScenSim-Port//	if (waiter -> outer) {
//ScenSim-Port//		if (waiter -> outer -> inner) {
//ScenSim-Port//			omapi_object_dereference (&waiter -> outer -> inner,
//ScenSim-Port//						  MDL);
//ScenSim-Port//			if (waiter -> inner)
//ScenSim-Port//				omapi_object_reference
//ScenSim-Port//					(&waiter -> outer -> inner,
//ScenSim-Port//					 waiter -> inner, MDL);
//ScenSim-Port//		}
//ScenSim-Port//		omapi_object_dereference (&waiter -> outer, MDL);
//ScenSim-Port//	}
//ScenSim-Port//	if (waiter -> inner)
//ScenSim-Port//		omapi_object_dereference (&waiter -> inner, MDL);
//ScenSim-Port//	
//ScenSim-Port//	status = waiter -> waitstatus;
//ScenSim-Port//	omapi_waiter_dereference (&waiter, MDL);
//ScenSim-Port//	return status;
//ScenSim-Port//}

//ScenSim-Port//isc_result_t omapi_one_dispatch (omapi_object_t *wo,
//ScenSim-Port//				 struct timeval *t)
//ScenSim-Port//{
//ScenSim-Port//	fd_set r, w, x, rr, ww, xx;
//ScenSim-Port//	int max = 0;
//ScenSim-Port//	int count;
//ScenSim-Port//	int desc;
//ScenSim-Port//	struct timeval now, to;
//ScenSim-Port//	omapi_io_object_t *io, *prev, *next;
//ScenSim-Port//	omapi_waiter_object_t *waiter;
//ScenSim-Port//	omapi_object_t *tmp = (omapi_object_t *)0;
//ScenSim-Port//
//ScenSim-Port//	if (!wo || wo -> type != omapi_type_waiter)
//ScenSim-Port//		waiter = (omapi_waiter_object_t *)0;
//ScenSim-Port//	else
//ScenSim-Port//		waiter = (omapi_waiter_object_t *)wo;
//ScenSim-Port//
//ScenSim-Port//	FD_ZERO (&x);
//ScenSim-Port//
//ScenSim-Port//	/* First, see if the timeout has expired, and if so return. */
//ScenSim-Port//	if (t) {
//ScenSim-Port//		gettimeofday (&now, (struct timezone *)0);
//ScenSim-Port//		cur_tv.tv_sec = now.tv_sec;
//ScenSim-Port//		cur_tv.tv_usec = now.tv_usec;
//ScenSim-Port//		if (now.tv_sec > t -> tv_sec ||
//ScenSim-Port//		    (now.tv_sec == t -> tv_sec && now.tv_usec >= t -> tv_usec))
//ScenSim-Port//			return ISC_R_TIMEDOUT;
//ScenSim-Port//			
//ScenSim-Port//		/* We didn't time out, so figure out how long until
//ScenSim-Port//		   we do. */
//ScenSim-Port//		to.tv_sec = t -> tv_sec - now.tv_sec;
//ScenSim-Port//		to.tv_usec = t -> tv_usec - now.tv_usec;
//ScenSim-Port//		if (to.tv_usec < 0) {
//ScenSim-Port//			to.tv_usec += 1000000;
//ScenSim-Port//			to.tv_sec--;
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//		/* It is possible for the timeout to get set larger than
//ScenSim-Port//		   the largest time select() is willing to accept.
//ScenSim-Port//		   Restricting the timeout to a maximum of one day should
//ScenSim-Port//		   work around this.  -DPN.  (Ref: Bug #416) */
//ScenSim-Port//		if (to.tv_sec > (60 * 60 * 24))
//ScenSim-Port//			to.tv_sec = 60 * 60 * 24;
//ScenSim-Port//	}
//ScenSim-Port//	
//ScenSim-Port//	/* If the object we're waiting on has reached completion,
//ScenSim-Port//	   return now. */
//ScenSim-Port//	if (waiter && waiter -> ready)
//ScenSim-Port//		return ISC_R_SUCCESS;
//ScenSim-Port//	
//ScenSim-Port//      again:
//ScenSim-Port//	/* If we have no I/O state, we can't proceed. */
//ScenSim-Port//	if (!(io = omapi_io_states.next))
//ScenSim-Port//		return ISC_R_NOMORE;
//ScenSim-Port//
//ScenSim-Port//	/* Set up the read and write masks. */
//ScenSim-Port//	FD_ZERO (&r);
//ScenSim-Port//	FD_ZERO (&w);
//ScenSim-Port//
//ScenSim-Port//	for (; io; io = io -> next) {
//ScenSim-Port//		/* Check for a read socket.   If we shouldn't be
//ScenSim-Port//		   trying to read for this I/O object, either there
//ScenSim-Port//		   won't be a readfd function, or it'll return -1. */
//ScenSim-Port//		if (io -> readfd && io -> inner &&
//ScenSim-Port//		    (desc = (*(io -> readfd)) (io -> inner)) >= 0) {
//ScenSim-Port//			FD_SET (desc, &r);
//ScenSim-Port//			if (desc > max)
//ScenSim-Port//				max = desc;
//ScenSim-Port//		}
//ScenSim-Port//		
//ScenSim-Port//		/* Same deal for write fdets. */
//ScenSim-Port//		if (io -> writefd && io -> inner &&
//ScenSim-Port//		    (desc = (*(io -> writefd)) (io -> inner)) >= 0) {
//ScenSim-Port//			FD_SET (desc, &w);
//ScenSim-Port//			if (desc > max)
//ScenSim-Port//				max = desc;
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/* poll if all reader are dry */ 
//ScenSim-Port//	now.tv_sec = 0;
//ScenSim-Port//	now.tv_usec = 0;
//ScenSim-Port//	rr=r; 
//ScenSim-Port//	ww=w; 
//ScenSim-Port//	xx=x;
//ScenSim-Port//
//ScenSim-Port//	/* poll once */
//ScenSim-Port//	count = select(max + 1, &r, &w, &x, &now);
//ScenSim-Port//	if (!count) {  
//ScenSim-Port//		/* We are dry now */ 
//ScenSim-Port//		trigger_event(&rw_queue_empty);
//ScenSim-Port//		/* Wait for a packet or a timeout... XXX */
//ScenSim-Port//		r = rr;
//ScenSim-Port//		w = ww;
//ScenSim-Port//		x = xx;
//ScenSim-Port//		count = select(max + 1, &r, &w, &x, t ? &to : NULL);
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/* Get the current time... */
//ScenSim-Port//	gettimeofday (&cur_tv, (struct timezone *)0);
//ScenSim-Port//
//ScenSim-Port//	/* We probably have a bad file descriptor.   Figure out which one.
//ScenSim-Port//	   When we find it, call the reaper function on it, which will
//ScenSim-Port//	   maybe make it go away, and then try again. */
//ScenSim-Port//	if (count < 0) {
//ScenSim-Port//		struct timeval t0;
//ScenSim-Port//		omapi_io_object_t *prev = (omapi_io_object_t *)0;
//ScenSim-Port//		io = (omapi_io_object_t *)0;
//ScenSim-Port//		if (omapi_io_states.next)
//ScenSim-Port//			omapi_io_reference (&io, omapi_io_states.next, MDL);
//ScenSim-Port//
//ScenSim-Port//		while (io) {
//ScenSim-Port//			omapi_object_t *obj;
//ScenSim-Port//			FD_ZERO (&r);
//ScenSim-Port//			FD_ZERO (&w);
//ScenSim-Port//			t0.tv_sec = t0.tv_usec = 0;
//ScenSim-Port//
//ScenSim-Port//			if (io -> readfd && io -> inner &&
//ScenSim-Port//			    (desc = (*(io -> readfd)) (io -> inner)) >= 0) {
//ScenSim-Port//			    FD_SET (desc, &r);
//ScenSim-Port//			    count = select (desc + 1, &r, &w, &x, &t0);
//ScenSim-Port//			   bogon:
//ScenSim-Port//			    if (count < 0) {
//ScenSim-Port//				log_error ("Bad descriptor %d.", desc);
//ScenSim-Port//				for (obj = (omapi_object_t *)io;
//ScenSim-Port//				     obj -> outer;
//ScenSim-Port//				     obj = obj -> outer)
//ScenSim-Port//					;
//ScenSim-Port//				for (; obj; obj = obj -> inner) {
//ScenSim-Port//				    omapi_value_t *ov;
//ScenSim-Port//				    int len;
//ScenSim-Port//				    const char *s;
//ScenSim-Port//				    ov = (omapi_value_t *)0;
//ScenSim-Port//				    omapi_get_value_str (obj,
//ScenSim-Port//							 (omapi_object_t *)0,
//ScenSim-Port//							 "name", &ov);
//ScenSim-Port//				    if (ov && ov -> value &&
//ScenSim-Port//					(ov -> value -> type ==
//ScenSim-Port//					 omapi_datatype_string)) {
//ScenSim-Port//					s = (char *)
//ScenSim-Port//						ov -> value -> u.buffer.value;
//ScenSim-Port//					len = ov -> value -> u.buffer.len;
//ScenSim-Port//				    } else {
//ScenSim-Port//					s = "";
//ScenSim-Port//					len = 0;
//ScenSim-Port//				    }
//ScenSim-Port//				    log_error ("Object %lx %s%s%.*s",
//ScenSim-Port//					       (unsigned long)obj,
//ScenSim-Port//					       obj -> type -> name,
//ScenSim-Port//					       len ? " " : "",
//ScenSim-Port//					       len, s);
//ScenSim-Port//				    if (len)
//ScenSim-Port//					omapi_value_dereference (&ov, MDL);
//ScenSim-Port//				}
//ScenSim-Port//				(*(io -> reaper)) (io -> inner);
//ScenSim-Port//				if (prev) {
//ScenSim-Port//				    omapi_io_dereference (&prev -> next, MDL);
//ScenSim-Port//				    if (io -> next)
//ScenSim-Port//					omapi_io_reference (&prev -> next,
//ScenSim-Port//							    io -> next, MDL);
//ScenSim-Port//				} else {
//ScenSim-Port//				    omapi_io_dereference
//ScenSim-Port//					    (&omapi_io_states.next, MDL);
//ScenSim-Port//				    if (io -> next)
//ScenSim-Port//					omapi_io_reference
//ScenSim-Port//						(&omapi_io_states.next,
//ScenSim-Port//						 io -> next, MDL);
//ScenSim-Port//				}
//ScenSim-Port//				omapi_io_dereference (&io, MDL);
//ScenSim-Port//				goto again;
//ScenSim-Port//			    }
//ScenSim-Port//			}
//ScenSim-Port//			
//ScenSim-Port//			FD_ZERO (&r);
//ScenSim-Port//			FD_ZERO (&w);
//ScenSim-Port//			t0.tv_sec = t0.tv_usec = 0;
//ScenSim-Port//
//ScenSim-Port//			/* Same deal for write fdets. */
//ScenSim-Port//			if (io -> writefd && io -> inner &&
//ScenSim-Port//			    (desc = (*(io -> writefd)) (io -> inner)) >= 0) {
//ScenSim-Port//				FD_SET (desc, &w);
//ScenSim-Port//				count = select (desc + 1, &r, &w, &x, &t0);
//ScenSim-Port//				if (count < 0)
//ScenSim-Port//					goto bogon;
//ScenSim-Port//			}
//ScenSim-Port//			if (prev)
//ScenSim-Port//				omapi_io_dereference (&prev, MDL);
//ScenSim-Port//			omapi_io_reference (&prev, io, MDL);
//ScenSim-Port//			omapi_io_dereference (&io, MDL);
//ScenSim-Port//			if (prev -> next)
//ScenSim-Port//			    omapi_io_reference (&io, prev -> next, MDL);
//ScenSim-Port//		}
//ScenSim-Port//		if (prev)
//ScenSim-Port//			omapi_io_dereference (&prev, MDL);
//ScenSim-Port//		
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	for (io = omapi_io_states.next; io; io = io -> next) {
//ScenSim-Port//		if (!io -> inner)
//ScenSim-Port//			continue;
//ScenSim-Port//		omapi_object_reference (&tmp, io -> inner, MDL);
//ScenSim-Port//		/* Check for a read descriptor, and if there is one,
//ScenSim-Port//		   see if we got input on that socket. */
//ScenSim-Port//		if (io -> readfd &&
//ScenSim-Port//		    (desc = (*(io -> readfd)) (tmp)) >= 0) {
//ScenSim-Port//			if (FD_ISSET (desc, &r))
//ScenSim-Port//				((*(io -> reader)) (tmp));
//ScenSim-Port//		}
//ScenSim-Port//		
//ScenSim-Port//		/* Same deal for write descriptors. */
//ScenSim-Port//		if (io -> writefd &&
//ScenSim-Port//		    (desc = (*(io -> writefd)) (tmp)) >= 0)
//ScenSim-Port//		{
//ScenSim-Port//			if (FD_ISSET (desc, &w))
//ScenSim-Port//				((*(io -> writer)) (tmp));
//ScenSim-Port//		}
//ScenSim-Port//		omapi_object_dereference (&tmp, MDL);
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	/* Now check for I/O handles that are no longer valid,
//ScenSim-Port//	   and remove them from the list. */
//ScenSim-Port//	prev = NULL;
//ScenSim-Port//	io = NULL;
//ScenSim-Port//	if (omapi_io_states.next != NULL) {
//ScenSim-Port//		omapi_io_reference(&io, omapi_io_states.next, MDL);
//ScenSim-Port//	}
//ScenSim-Port//	while (io != NULL) {
//ScenSim-Port//		if ((io->inner == NULL) || 
//ScenSim-Port//		    ((io->reaper != NULL) && 
//ScenSim-Port//		     ((io->reaper)(io->inner) != ISC_R_SUCCESS))) 
//ScenSim-Port//		{
//ScenSim-Port//
//ScenSim-Port//			omapi_io_object_t *tmp = NULL;
//ScenSim-Port//			/* Save a reference to the next
//ScenSim-Port//			   pointer, if there is one. */
//ScenSim-Port//			if (io->next != NULL) {
//ScenSim-Port//				omapi_io_reference(&tmp, io->next, MDL);
//ScenSim-Port//				omapi_io_dereference(&io->next, MDL);
//ScenSim-Port//			}
//ScenSim-Port//			if (prev != NULL) {
//ScenSim-Port//				omapi_io_dereference(&prev->next, MDL);
//ScenSim-Port//				if (tmp != NULL)
//ScenSim-Port//					omapi_io_reference(&prev->next,
//ScenSim-Port//							   tmp, MDL);
//ScenSim-Port//			} else {
//ScenSim-Port//				omapi_io_dereference(&omapi_io_states.next, 
//ScenSim-Port//						     MDL);
//ScenSim-Port//				if (tmp != NULL)
//ScenSim-Port//					omapi_io_reference
//ScenSim-Port//					    (&omapi_io_states.next,
//ScenSim-Port//					     tmp, MDL);
//ScenSim-Port//				else
//ScenSim-Port//					omapi_signal_in(
//ScenSim-Port//							(omapi_object_t *)
//ScenSim-Port//						 	&omapi_io_states,
//ScenSim-Port//							"ready");
//ScenSim-Port//			}
//ScenSim-Port//			if (tmp != NULL)
//ScenSim-Port//				omapi_io_dereference(&tmp, MDL);
//ScenSim-Port//
//ScenSim-Port//		} else {
//ScenSim-Port//
//ScenSim-Port//			if (prev != NULL) {
//ScenSim-Port//				omapi_io_dereference(&prev, MDL);
//ScenSim-Port//			}
//ScenSim-Port//			omapi_io_reference(&prev, io, MDL);
//ScenSim-Port//		}
//ScenSim-Port//
//ScenSim-Port//		/*
//ScenSim-Port//		 * Equivalent to:
//ScenSim-Port//		 *   io = io->next
//ScenSim-Port//		 * But using our reference counting voodoo.
//ScenSim-Port//		 */
//ScenSim-Port//		next = NULL;
//ScenSim-Port//		if (io->next != NULL) {
//ScenSim-Port//			omapi_io_reference(&next, io->next, MDL);
//ScenSim-Port//		}
//ScenSim-Port//		omapi_io_dereference(&io, MDL);
//ScenSim-Port//		if (next != NULL) {
//ScenSim-Port//			omapi_io_reference(&io, next, MDL);
//ScenSim-Port//			omapi_io_dereference(&next, MDL);
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//	if (prev != NULL) {
//ScenSim-Port//		omapi_io_dereference(&prev, MDL);
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	return ISC_R_SUCCESS;
//ScenSim-Port//}

//ScenSim-Port//isc_result_t omapi_io_set_value (omapi_object_t *h,
//ScenSim-Port//				 omapi_object_t *id,
//ScenSim-Port//				 omapi_data_string_t *name,
//ScenSim-Port//				 omapi_typed_data_t *value)
//ScenSim-Port//{
//ScenSim-Port//	if (h -> type != omapi_type_io_object)
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//	
//ScenSim-Port//	if (h -> inner && h -> inner -> type -> set_value)
//ScenSim-Port//		return (*(h -> inner -> type -> set_value))
//ScenSim-Port//			(h -> inner, id, name, value);
//ScenSim-Port//	return ISC_R_NOTFOUND;
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//isc_result_t omapi_io_get_value (omapi_object_t *h,
//ScenSim-Port//				 omapi_object_t *id,
//ScenSim-Port//				 omapi_data_string_t *name,
//ScenSim-Port//				 omapi_value_t **value)
//ScenSim-Port//{
//ScenSim-Port//	if (h -> type != omapi_type_io_object)
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//	
//ScenSim-Port//	if (h -> inner && h -> inner -> type -> get_value)
//ScenSim-Port//		return (*(h -> inner -> type -> get_value))
//ScenSim-Port//			(h -> inner, id, name, value);
//ScenSim-Port//	return ISC_R_NOTFOUND;
//ScenSim-Port//}

/* omapi_io_destroy (object, MDL);
 *
 *	Find the requested IO [object] and remove it from the list of io
 * states, causing the cleanup functions to destroy it.  Note that we must
 * hold a reference on the object while moving its ->next reference and
 * removing the reference in the chain to the target object...otherwise it
 * may be cleaned up from under us.
 */
//ScenSim-Port//isc_result_t omapi_io_destroy (omapi_object_t *h, const char *file, int line)
//ScenSim-Port//{
//ScenSim-Port//	omapi_io_object_t *obj = NULL, *p, *last = NULL, **holder;
//ScenSim-Port//
//ScenSim-Port//	if (h -> type != omapi_type_io_object)
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//	
//ScenSim-Port//	/* remove from the list of I/O states */
//ScenSim-Port//	for (p = omapi_io_states.next; p; p = p -> next) {
//ScenSim-Port//		if (p == (omapi_io_object_t *)h) {
//ScenSim-Port//			omapi_io_reference (&obj, p, MDL);
//ScenSim-Port//
//ScenSim-Port//			if (last)
//ScenSim-Port//				holder = &last -> next;
//ScenSim-Port//			else
//ScenSim-Port//				holder = &omapi_io_states.next;
//ScenSim-Port//
//ScenSim-Port//			omapi_io_dereference (holder, MDL);
//ScenSim-Port//
//ScenSim-Port//			if (obj -> next) {
//ScenSim-Port//				omapi_io_reference (holder, obj -> next, MDL);
//ScenSim-Port//				omapi_io_dereference (&obj -> next, MDL);
//ScenSim-Port//			}
//ScenSim-Port//
//ScenSim-Port//			return omapi_io_dereference (&obj, MDL);
//ScenSim-Port//		}
//ScenSim-Port//		last = p;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	return ISC_R_NOTFOUND;
//ScenSim-Port//}

//ScenSim-Port//isc_result_t omapi_io_signal_handler (omapi_object_t *h,
//ScenSim-Port//				      const char *name, va_list ap)
//ScenSim-Port//{
//ScenSim-Port//	if (h -> type != omapi_type_io_object)
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//	
//ScenSim-Port//	if (h -> inner && h -> inner -> type -> signal_handler)
//ScenSim-Port//		return (*(h -> inner -> type -> signal_handler)) (h -> inner,
//ScenSim-Port//								  name, ap);
//ScenSim-Port//	return ISC_R_NOTFOUND;
//ScenSim-Port//}

//ScenSim-Port//isc_result_t omapi_io_stuff_values (omapi_object_t *c,
//ScenSim-Port//				    omapi_object_t *id,
//ScenSim-Port//				    omapi_object_t *i)
//ScenSim-Port//{
//ScenSim-Port//	if (i -> type != omapi_type_io_object)
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//
//ScenSim-Port//	if (i -> inner && i -> inner -> type -> stuff_values)
//ScenSim-Port//		return (*(i -> inner -> type -> stuff_values)) (c, id,
//ScenSim-Port//								i -> inner);
//ScenSim-Port//	return ISC_R_SUCCESS;
//ScenSim-Port//}

//ScenSim-Port//isc_result_t omapi_waiter_signal_handler (omapi_object_t *h,
//ScenSim-Port//					  const char *name, va_list ap)
//ScenSim-Port//{
//ScenSim-Port//	omapi_waiter_object_t *waiter;
//ScenSim-Port//
//ScenSim-Port//	if (h -> type != omapi_type_waiter)
//ScenSim-Port//		return DHCP_R_INVALIDARG;
//ScenSim-Port//	
//ScenSim-Port//	if (!strcmp (name, "ready")) {
//ScenSim-Port//		waiter = (omapi_waiter_object_t *)h;
//ScenSim-Port//		waiter -> ready = 1;
//ScenSim-Port//		waiter -> waitstatus = ISC_R_SUCCESS;
//ScenSim-Port//		return ISC_R_SUCCESS;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (!strcmp(name, "status")) {
//ScenSim-Port//		waiter = (omapi_waiter_object_t *)h;
//ScenSim-Port//		waiter->ready = 1;
//ScenSim-Port//		waiter->waitstatus = va_arg(ap, isc_result_t);
//ScenSim-Port//		return ISC_R_SUCCESS;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (!strcmp (name, "disconnect")) {
//ScenSim-Port//		waiter = (omapi_waiter_object_t *)h;
//ScenSim-Port//		waiter -> ready = 1;
//ScenSim-Port//		waiter -> waitstatus = DHCP_R_CONNRESET;
//ScenSim-Port//		return ISC_R_SUCCESS;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	if (h -> inner && h -> inner -> type -> signal_handler)
//ScenSim-Port//		return (*(h -> inner -> type -> signal_handler)) (h -> inner,
//ScenSim-Port//								  name, ap);
//ScenSim-Port//	return ISC_R_NOTFOUND;
//ScenSim-Port//}

//ScenSim-Port//isc_result_t omapi_io_state_foreach (isc_result_t (*func) (omapi_object_t *,
//ScenSim-Port//							   void *),
//ScenSim-Port//				     void *p)
//ScenSim-Port//{
//ScenSim-Port//	omapi_io_object_t *io;
//ScenSim-Port//	isc_result_t status;
//ScenSim-Port//
//ScenSim-Port//	for (io = omapi_io_states.next; io; io = io -> next) {
//ScenSim-Port//		if (io -> inner) {
//ScenSim-Port//			status = (*func) (io -> inner, p);
//ScenSim-Port//			if (status != ISC_R_SUCCESS)
//ScenSim-Port//				return status;
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//	return ISC_R_SUCCESS;
//ScenSim-Port//}
}//namespace IscDhcpPort////ScenSim-Port//
