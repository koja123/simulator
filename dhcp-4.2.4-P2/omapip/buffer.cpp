/* buffer.c

   Buffer access functions for the object management protocol... */

/*
 * Copyright (c) 2004,2005,2007,2009 by Internet Systems Consortium, Inc. ("ISC")
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
//ScenSim-Port//#include <errno.h>

//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//static void trace_connection_input_input (trace_type_t *, unsigned, char *);
//ScenSim-Port//static void trace_connection_input_stop (trace_type_t *);
//ScenSim-Port//static void trace_connection_output_input (trace_type_t *, unsigned, char *);
//ScenSim-Port//static void trace_connection_output_stop (trace_type_t *);
//ScenSim-Port//static trace_type_t *trace_connection_input;
//ScenSim-Port//static trace_type_t *trace_connection_output;
//ScenSim-Port//static isc_result_t omapi_connection_reader_trace (omapi_object_t *,
//ScenSim-Port//						   unsigned, char *,
//ScenSim-Port//						   unsigned *);
//ScenSim-Port//extern omapi_array_t *omapi_connections;
//ScenSim-Port//
//ScenSim-Port//void omapi_buffer_trace_setup ()
//ScenSim-Port//{
//ScenSim-Port//	trace_connection_input =
//ScenSim-Port//		trace_type_register ("connection-input",
//ScenSim-Port//				     (void *)0,
//ScenSim-Port//				     trace_connection_input_input,
//ScenSim-Port//				     trace_connection_input_stop, MDL);
//ScenSim-Port//	trace_connection_output =
//ScenSim-Port//		trace_type_register ("connection-output",
//ScenSim-Port//				     (void *)0,
//ScenSim-Port//				     trace_connection_output_input,
//ScenSim-Port//				     trace_connection_output_stop, MDL);
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//static void trace_connection_input_input (trace_type_t *ttype,
//ScenSim-Port//					  unsigned length, char *buf)
//ScenSim-Port//{
//ScenSim-Port//	unsigned left, taken, cc = 0;
//ScenSim-Port//	char *s;
//ScenSim-Port//	int32_t connect_index;
//ScenSim-Port//	isc_result_t status;
//ScenSim-Port//	omapi_connection_object_t *c = (omapi_connection_object_t *)0;
//ScenSim-Port//
//ScenSim-Port//	memcpy (&connect_index, buf, sizeof connect_index);
//ScenSim-Port//	connect_index = ntohl (connect_index);
//ScenSim-Port//
//ScenSim-Port//	omapi_array_foreach_begin (omapi_connections,
//ScenSim-Port//				   omapi_connection_object_t, lp) {
//ScenSim-Port//		if (lp -> index == ntohl (connect_index)) {
//ScenSim-Port//			omapi_connection_reference (&c, lp, MDL);
//ScenSim-Port//			omapi_connection_dereference (&lp, MDL);
//ScenSim-Port//			break;
//ScenSim-Port//		}
//ScenSim-Port//	} omapi_array_foreach_end (omapi_connections,
//ScenSim-Port//				   omapi_connection_object_t, lp);
//ScenSim-Port//
//ScenSim-Port//	if (!c) {
//ScenSim-Port//		log_error ("trace connection input: no connection index %ld",
//ScenSim-Port//			   (long int)connect_index);
//ScenSim-Port//		return;
//ScenSim-Port//	}
//ScenSim-Port//
//ScenSim-Port//	s = buf + sizeof connect_index;
//ScenSim-Port//	left = length - sizeof connect_index;
//ScenSim-Port//
//ScenSim-Port//	while (left) {
//ScenSim-Port//		taken = 0;
//ScenSim-Port//		status = omapi_connection_reader_trace ((omapi_object_t *)c,
//ScenSim-Port//							left, s, &taken);
//ScenSim-Port//		if (status != ISC_R_SUCCESS) {
//ScenSim-Port//			log_error ("trace connection input: %s",
//ScenSim-Port//				   isc_result_totext (status));
//ScenSim-Port//			break;
//ScenSim-Port//		}
//ScenSim-Port//		if (!taken) {
//ScenSim-Port//			if (cc > 0) {
//ScenSim-Port//				log_error ("trace connection_input: %s",
//ScenSim-Port//					   "input is not being consumed.");
//ScenSim-Port//				break;
//ScenSim-Port//			}
//ScenSim-Port//			cc++;
//ScenSim-Port//		} else {
//ScenSim-Port//			cc = 0;
//ScenSim-Port//			left -= taken;
//ScenSim-Port//		}
//ScenSim-Port//	}
//ScenSim-Port//	omapi_connection_dereference (&c, MDL);
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//static void trace_connection_input_stop (trace_type_t *ttype) { }
//ScenSim-Port//
//ScenSim-Port//static void trace_connection_output_input (trace_type_t *ttype,
//ScenSim-Port//					  unsigned length, char *buf)
//ScenSim-Port//{
//ScenSim-Port//	/* We *could* check to see if the output is correct, but for now
//ScenSim-Port//	   we aren't going to do that. */
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//static void trace_connection_output_stop (trace_type_t *ttype) { }
//ScenSim-Port//
//ScenSim-Port//#endif

/* Make sure that at least len bytes are in the input buffer, and if not,
   read enough bytes to make up the difference. */

isc_result_t omapi_connection_reader (omapi_object_t *h)
{
//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//	return omapi_connection_reader_trace (h, 0, (char *)0, (unsigned *)0);
//ScenSim-Port//}
//ScenSim-Port//
//ScenSim-Port//static isc_result_t omapi_connection_reader_trace (omapi_object_t *h,
//ScenSim-Port//						   unsigned stuff_len,
//ScenSim-Port//						   char *stuff_buf,
//ScenSim-Port//						   unsigned *stuff_taken)
//ScenSim-Port//{
//ScenSim-Port//#endif
	omapi_buffer_t *buffer;
	isc_result_t status;
	unsigned read_len;
	int read_status;
	omapi_connection_object_t *c;
	unsigned bytes_to_read;
	
	if (!h || h -> type != omapi_type_connection)
		return DHCP_R_INVALIDARG;
	c = (omapi_connection_object_t *)h;

	/* See if there are enough bytes. */
	if (c -> in_bytes >= OMAPI_BUF_SIZE - 1 &&
	    c -> in_bytes > c -> bytes_needed)
		return ISC_R_SUCCESS;


	if (c -> inbufs) {
		for (buffer = c -> inbufs; buffer -> next;
		     buffer = buffer -> next)
			;
		if (!BUFFER_BYTES_FREE (buffer)) {
			status = omapi_buffer_new (&buffer -> next, MDL);
			if (status != ISC_R_SUCCESS)
				return status;
			buffer = buffer -> next;
		}
	} else {
		status = omapi_buffer_new (&c -> inbufs, MDL);
		if (status != ISC_R_SUCCESS)
			return status;
		buffer = c -> inbufs;
	}

	bytes_to_read = BUFFER_BYTES_FREE (buffer);

	while (bytes_to_read) {
		if (buffer -> tail > buffer -> head)
			read_len = sizeof (buffer -> buf) - buffer -> tail;
		else
			read_len = buffer -> head - buffer -> tail;

//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//		if (trace_playback()) {
//ScenSim-Port//			if (stuff_len) {
//ScenSim-Port//				if (read_len > stuff_len)
//ScenSim-Port//					read_len = stuff_len;
//ScenSim-Port//				if (stuff_taken)
//ScenSim-Port//					*stuff_taken += read_len;
//ScenSim-Port//				memcpy (&buffer -> buf [buffer -> tail],
//ScenSim-Port//					stuff_buf, read_len);
//ScenSim-Port//				stuff_len -= read_len;
//ScenSim-Port//				stuff_buf += read_len;
//ScenSim-Port//				read_status = read_len;
//ScenSim-Port//			} else {
//ScenSim-Port//				break;
//ScenSim-Port//			}
//ScenSim-Port//		} else
//ScenSim-Port//#endif
		{
			read_status = read (c -> socket,
					    &buffer -> buf [buffer -> tail],
					    read_len);
		}
		if (read_status < 0) {
			if (errno == EWOULDBLOCK)
				break;
			else if (errno == EIO)
				return ISC_R_IOERROR;
			else if (errno == EINVAL)
				return DHCP_R_INVALIDARG;
			else if (errno == ECONNRESET) {
//ScenSim-Port//				omapi_disconnect (h, 1);
				return ISC_R_SHUTTINGDOWN;
			} else
				return ISC_R_UNEXPECTED;
		}

		/* If we got a zero-length read, as opposed to EWOULDBLOCK,
		   the remote end closed the connection. */
		if (read_status == 0) {
//ScenSim-Port//			omapi_disconnect (h, 0);
			return ISC_R_SHUTTINGDOWN;
		}
//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//		if (trace_record ()) {
//ScenSim-Port//			trace_iov_t iov [2];
//ScenSim-Port//			int32_t connect_index;
//ScenSim-Port//
//ScenSim-Port//			connect_index = htonl (c -> index);
//ScenSim-Port//
//ScenSim-Port//			iov [0].buf = (char *)&connect_index;
//ScenSim-Port//			iov [0].len = sizeof connect_index;
//ScenSim-Port//			iov [1].buf = &buffer -> buf [buffer -> tail];
//ScenSim-Port//			iov [1].len = read_status;
//ScenSim-Port//
//ScenSim-Port//			status = (trace_write_packet_iov
//ScenSim-Port//				  (trace_connection_input, 2, iov, MDL));
//ScenSim-Port//			if (status != ISC_R_SUCCESS) {
//ScenSim-Port//				trace_stop ();
//ScenSim-Port//				log_error ("trace connection input: %s",
//ScenSim-Port//					   isc_result_totext (status));
//ScenSim-Port//			}
//ScenSim-Port//		}
//ScenSim-Port//#endif
		buffer -> tail += read_status;
		c -> in_bytes += read_status;
		if (buffer -> tail == sizeof buffer -> buf)
			buffer -> tail = 0;
		if (read_status < read_len)
			break;
		bytes_to_read -= read_status;
	}

	if (c -> bytes_needed <= c -> in_bytes) {
		omapi_signal (h, "ready", c);
	}
	return ISC_R_SUCCESS;
}

/* Put some bytes into the output buffer for a connection. */

isc_result_t omapi_connection_copyin (omapi_object_t *h,
				      const unsigned char *bufp,
				      unsigned len)
{
	omapi_buffer_t *buffer;
	isc_result_t status;
	int bytes_copied = 0;
	unsigned copy_len;
	int sig_flags = SIG_MODE_UPDATE;
	omapi_connection_object_t *c;

	/* Make sure len is valid. */
	if (len < 0)
		return DHCP_R_INVALIDARG;
	if (!h || h -> type != omapi_type_connection)
		return DHCP_R_INVALIDARG;
	c = (omapi_connection_object_t *)h;

	/* If the connection is closed, return an error if the caller
	   tries to copy in. */
	if (c -> state == omapi_connection_disconnecting ||
	    c -> state == omapi_connection_closed)
		return ISC_R_NOTCONNECTED;

	if (c -> outbufs) {
		for (buffer = c -> outbufs;
		     buffer -> next; buffer = buffer -> next)
			;
	} else {
		status = omapi_buffer_new (&c -> outbufs, MDL);
		if (status != ISC_R_SUCCESS)
			goto leave;
		buffer = c -> outbufs;
	}

	while (bytes_copied < len) {
		/* If there is no space available in this buffer,
                   allocate a new one. */
		if (!BUFFER_BYTES_FREE (buffer)) {
			status = (omapi_buffer_new (&buffer -> next, MDL));
			if (status != ISC_R_SUCCESS)
				goto leave;
			buffer = buffer -> next;
		}

		if (buffer -> tail > buffer -> head)
			copy_len = sizeof (buffer -> buf) - buffer -> tail;
		else
			copy_len = buffer -> head - buffer -> tail;

		if (copy_len > (len - bytes_copied))
			copy_len = len - bytes_copied;

//ScenSim-Port//		if (c -> out_key) {
//ScenSim-Port//			if (!c -> out_context)
//ScenSim-Port//				sig_flags |= SIG_MODE_INIT;
//ScenSim-Port//			status = omapi_connection_sign_data
//ScenSim-Port//				(sig_flags, c -> out_key, &c -> out_context,
//ScenSim-Port//				 &bufp [bytes_copied], copy_len,
//ScenSim-Port//				 (omapi_typed_data_t **)0);
//ScenSim-Port//			if (status != ISC_R_SUCCESS)
//ScenSim-Port//				goto leave;
//ScenSim-Port//		}

		memcpy (&buffer -> buf [buffer -> tail],
			&bufp [bytes_copied], copy_len);
		buffer -> tail += copy_len;
		c -> out_bytes += copy_len;
		bytes_copied += copy_len;
		if (buffer -> tail == sizeof buffer -> buf)
			buffer -> tail = 0;
	}

	status = ISC_R_SUCCESS;

 leave:
	/*
	 * If we have any bytes to send and we have a proper io object
	 * inform the socket code that we would like to know when we
	 * can send more bytes.
	 */
//ScenSim-Port//	if (c->out_bytes != 0) {
//ScenSim-Port//		if ((c->outer != NULL) &&
//ScenSim-Port//		    (c->outer->type == omapi_type_io_object)) {
//ScenSim-Port//			omapi_io_object_t *io = (omapi_io_object_t *)c->outer;
//ScenSim-Port//			isc_socket_fdwatchpoke(io->fd,
//ScenSim-Port//					       ISC_SOCKFDWATCH_WRITE);
//ScenSim-Port//		}
//ScenSim-Port//	}

	return (status);
}

/* Copy some bytes from the input buffer, and advance the input buffer
   pointer beyond the bytes copied out. */

isc_result_t omapi_connection_copyout (unsigned char *buf,
				       omapi_object_t *h,
				       unsigned size)
{
	unsigned bytes_remaining;
	unsigned bytes_this_copy;
	unsigned first_byte;
	omapi_buffer_t *buffer;
	unsigned char *bufp;
	int sig_flags = SIG_MODE_UPDATE;
	omapi_connection_object_t *c;
	isc_result_t status;

	if (!h || h -> type != omapi_type_connection)
		return DHCP_R_INVALIDARG;
	c = (omapi_connection_object_t *)h;

	if (size > c -> in_bytes)
		return ISC_R_NOMORE;
	bufp = buf;
	bytes_remaining = size;
	buffer = c -> inbufs;

	while (bytes_remaining) {
		if (!buffer)
			return ISC_R_UNEXPECTED;
		if (BYTES_IN_BUFFER (buffer)) {
			if (buffer -> head == (sizeof buffer -> buf) - 1)
				first_byte = 0;
			else
				first_byte = buffer -> head + 1;

			if (first_byte > buffer -> tail) {
				bytes_this_copy = (sizeof buffer -> buf -
						   first_byte);
			} else {
				bytes_this_copy =
					buffer -> tail - first_byte;
			}
			if (bytes_this_copy > bytes_remaining)
				bytes_this_copy = bytes_remaining;
			if (bufp) {
//ScenSim-Port//				if (c -> in_key) {
//ScenSim-Port//					if (!c -> in_context)
//ScenSim-Port//						sig_flags |= SIG_MODE_INIT;
//ScenSim-Port//					status = omapi_connection_sign_data
//ScenSim-Port//						(sig_flags,
//ScenSim-Port//						 c -> in_key,
//ScenSim-Port//						 &c -> in_context,
//ScenSim-Port//						 (unsigned char *)
//ScenSim-Port//						 &buffer -> buf [first_byte],
//ScenSim-Port//						 bytes_this_copy,
//ScenSim-Port//						 (omapi_typed_data_t **)0);
//ScenSim-Port//					if (status != ISC_R_SUCCESS)
//ScenSim-Port//						return status;
//ScenSim-Port//				}

				memcpy (bufp, &buffer -> buf [first_byte],
					bytes_this_copy);
				bufp += bytes_this_copy;
			}
			bytes_remaining -= bytes_this_copy;
			buffer -> head = first_byte + bytes_this_copy - 1;
			c -> in_bytes -= bytes_this_copy;
		}
			
		if (!BYTES_IN_BUFFER (buffer))
			buffer = buffer -> next;
	}

	/* Get rid of any input buffers that we emptied. */
	buffer = (omapi_buffer_t *)0;
	while (c -> inbufs &&
	       !BYTES_IN_BUFFER (c -> inbufs)) {
		if (c -> inbufs -> next) {
			omapi_buffer_reference (&buffer,
						c -> inbufs -> next, MDL);
			omapi_buffer_dereference (&c -> inbufs -> next, MDL);
		}
		omapi_buffer_dereference (&c -> inbufs, MDL);
		if (buffer) {
			omapi_buffer_reference
				(&c -> inbufs, buffer, MDL);
			omapi_buffer_dereference (&buffer, MDL);
		}
	}
	return ISC_R_SUCCESS;
}

isc_result_t omapi_connection_writer (omapi_object_t *h)
{
	unsigned bytes_this_write;
	int bytes_written;
	unsigned first_byte;
	omapi_buffer_t *buffer;
	omapi_connection_object_t *c;

	if (!h || h -> type != omapi_type_connection)
		return DHCP_R_INVALIDARG;
	c = (omapi_connection_object_t *)h;

	/* Already flushed... */
	if (!c -> out_bytes)
		return ISC_R_SUCCESS;

	buffer = c -> outbufs;

	while (c -> out_bytes) {
		if (!buffer)
			return ISC_R_UNEXPECTED;
		if (BYTES_IN_BUFFER (buffer)) {
			if (buffer -> head == (sizeof buffer -> buf) - 1)
				first_byte = 0;
			else
				first_byte = buffer -> head + 1;

			if (first_byte > buffer -> tail) {
				bytes_this_write = (sizeof buffer -> buf -
						   first_byte);
			} else {
				bytes_this_write =
					buffer -> tail - first_byte;
			}
			bytes_written = write (c -> socket,
					       &buffer -> buf [first_byte],
					       bytes_this_write);
			/* If the write failed with EWOULDBLOCK or we wrote
			   zero bytes, a further write would block, so we have
			   flushed as much as we can for now.   Other errors
			   are really errors. */
			if (bytes_written < 0) {
				if (errno == EWOULDBLOCK || errno == EAGAIN)
					return ISC_R_INPROGRESS;
				else if (errno == EPIPE)
					return ISC_R_NOCONN;
//ScenSim-Port//#ifdef EDQUOT
//ScenSim-Port//				else if (errno == EFBIG || errno == EDQUOT)
//ScenSim-Port//#else
				else if (errno == EFBIG)
//ScenSim-Port//#endif
					return ISC_R_NORESOURCES;
				else if (errno == ENOSPC)
					return ISC_R_NOSPACE;
				else if (errno == EIO)
					return ISC_R_IOERROR;
				else if (errno == EINVAL)
					return DHCP_R_INVALIDARG;
				else if (errno == ECONNRESET)
					return ISC_R_SHUTTINGDOWN;
				else
					return ISC_R_UNEXPECTED;
			}
			if (bytes_written == 0)
				return ISC_R_INPROGRESS;

//ScenSim-Port//#if defined (TRACING)
//ScenSim-Port//			if (trace_record ()) {
//ScenSim-Port//				isc_result_t status;
//ScenSim-Port//				trace_iov_t iov [2];
//ScenSim-Port//				int32_t connect_index;
//ScenSim-Port//				
//ScenSim-Port//				connect_index = htonl (c -> index);
//ScenSim-Port//				
//ScenSim-Port//				iov [0].buf = (char *)&connect_index;
//ScenSim-Port//				iov [0].len = sizeof connect_index;
//ScenSim-Port//				iov [1].buf = &buffer -> buf [buffer -> tail];
//ScenSim-Port//				iov [1].len = bytes_written;
//ScenSim-Port//				
//ScenSim-Port//				status = (trace_write_packet_iov
//ScenSim-Port//					  (trace_connection_input, 2, iov,
//ScenSim-Port//					   MDL));
//ScenSim-Port//				if (status != ISC_R_SUCCESS) {
//ScenSim-Port//					trace_stop ();
//ScenSim-Port//					log_error ("trace %s output: %s",
//ScenSim-Port//						   "connection",
//ScenSim-Port//						   isc_result_totext (status));
//ScenSim-Port//				}
//ScenSim-Port//			}
//ScenSim-Port//#endif

			buffer -> head = first_byte + bytes_written - 1;
			c -> out_bytes -= bytes_written;

			/* If we didn't finish out the write, we filled the
			   O.S. output buffer and a further write would block,
			   so stop trying to flush now. */
			if (bytes_written != bytes_this_write)
				return ISC_R_INPROGRESS;
		}
			
		if (!BYTES_IN_BUFFER (buffer))
			buffer = buffer -> next;
	}
		
	/* Get rid of any output buffers we emptied. */
	buffer = (omapi_buffer_t *)0;
	while (c -> outbufs &&
	       !BYTES_IN_BUFFER (c -> outbufs)) {
		if (c -> outbufs -> next) {
			omapi_buffer_reference (&buffer,
						c -> outbufs -> next, MDL);
			omapi_buffer_dereference (&c -> outbufs -> next, MDL);
		}
		omapi_buffer_dereference (&c -> outbufs, MDL);
		if (buffer) {
			omapi_buffer_reference (&c -> outbufs, buffer, MDL);
			omapi_buffer_dereference (&buffer, MDL);
		}
	}
	return ISC_R_SUCCESS;
}

isc_result_t omapi_connection_get_uint32 (omapi_object_t *c,
					  u_int32_t *result)
{
	u_int32_t inbuf;
	isc_result_t status;

	status = omapi_connection_copyout ((unsigned char *)&inbuf,
					   c, sizeof inbuf);
	if (status != ISC_R_SUCCESS)
		return status;

	*result = ntohl (inbuf);
	return ISC_R_SUCCESS;
}

isc_result_t omapi_connection_put_uint32 (omapi_object_t *c,
					  u_int32_t value)
{
	u_int32_t inbuf;

	inbuf = htonl (value);
	
	return omapi_connection_copyin (c, (unsigned char *)&inbuf,
					sizeof inbuf);
}

isc_result_t omapi_connection_get_uint16 (omapi_object_t *c,
					  u_int16_t *result)
{
	u_int16_t inbuf;
	isc_result_t status;

	status = omapi_connection_copyout ((unsigned char *)&inbuf,
					   c, sizeof inbuf);
	if (status != ISC_R_SUCCESS)
		return status;

	*result = ntohs (inbuf);
	return ISC_R_SUCCESS;
}

isc_result_t omapi_connection_put_uint16 (omapi_object_t *c,
					  u_int32_t value)
{
	u_int16_t inbuf;

	inbuf = htons (value);
	
	return omapi_connection_copyin (c, (unsigned char *)&inbuf,
					sizeof inbuf);
}

isc_result_t omapi_connection_write_typed_data (omapi_object_t *c,
						omapi_typed_data_t *data)
{
	isc_result_t status;
	omapi_handle_t handle;

	/* Null data is valid. */
	if (!data)
		return omapi_connection_put_uint32 (c, 0);

	switch (data -> type) {
	      case omapi_datatype_int:
		status = omapi_connection_put_uint32 (c, sizeof (u_int32_t));
		if (status != ISC_R_SUCCESS)
			return status;
		return omapi_connection_put_uint32 (c, ((u_int32_t)
							(data -> u.integer)));

	      case omapi_datatype_string:
	      case omapi_datatype_data:
		status = omapi_connection_put_uint32 (c, data -> u.buffer.len);
		if (status != ISC_R_SUCCESS)
			return status;
		if (data -> u.buffer.len)
			return omapi_connection_copyin
				(c, data -> u.buffer.value,
				 data -> u.buffer.len);
		return ISC_R_SUCCESS;

	      case omapi_datatype_object:
		if (data -> u.object) {
			status = omapi_object_handle (&handle,
						      data -> u.object);
			if (status != ISC_R_SUCCESS)
				return status;
		} else
			handle = 0;
		status = omapi_connection_put_uint32 (c, sizeof handle);
		if (status != ISC_R_SUCCESS)
			return status;
		return omapi_connection_put_uint32 (c, handle);

	}
	return DHCP_R_INVALIDARG;
}

isc_result_t omapi_connection_put_name (omapi_object_t *c, const char *name)
{
	isc_result_t status;
	unsigned len = strlen (name);

	status = omapi_connection_put_uint16 (c, len);
	if (status != ISC_R_SUCCESS)
		return status;
	return omapi_connection_copyin (c, (const unsigned char *)name, len);
}

isc_result_t omapi_connection_put_string (omapi_object_t *c,
					  const char *string)
{
	isc_result_t status;
	unsigned len;

	if (string)
		len = strlen (string);
	else
		len = 0;

	status = omapi_connection_put_uint32 (c, len);
	if (status != ISC_R_SUCCESS)
		return status;
	if (len)
		return omapi_connection_copyin
			(c, (const unsigned char *)string, len);
	return ISC_R_SUCCESS;
}

isc_result_t omapi_connection_put_handle (omapi_object_t *c, omapi_object_t *h)
{
	isc_result_t status;
	omapi_handle_t handle;

	if (h) {
		status = omapi_object_handle (&handle, h);
		if (status != ISC_R_SUCCESS)
			return status;
	} else
		handle = 0;	/* The null handle. */
	status = omapi_connection_put_uint32 (c, sizeof handle);
	if (status != ISC_R_SUCCESS)
		return status;
	return omapi_connection_put_uint32 (c, handle);
}
}//namespace IscDhcpPort////ScenSim-Port//
