/* alloc.c

   Memory allocation... */

/*
 * Copyright (c) 2004-2007,2009 by Internet Systems Consortium, Inc. ("ISC")
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
#include <iscdhcp_porting.h>//ScenSim-Port//
namespace IscDhcpPort {//ScenSim-Port//

//ScenSim-Port//#include "dhcpd.h"
//ScenSim-Port//#include <omapip/omapip_p.h>

//ScenSim-Port//struct dhcp_packet *dhcp_free_list;
//ScenSim-Port//struct packet *packet_free_list;

//ScenSim-Port//int option_chain_head_allocate (ptr, file, line)
//ScenSim-Port//	struct option_chain_head **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int option_chain_head_allocate (struct option_chain_head **ptr, const char *file, int line)//ScenSim-Port//
{
	struct option_chain_head *h;

	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct option_chain_head *)0;
//ScenSim-Port//#endif
	}

//ScenSim-Port//	h = dmalloc (sizeof *h, file, line);
	h = (option_chain_head*)dmalloc (sizeof *h, file, line);//ScenSim-Port//
	if (h) {
		memset (h, 0, sizeof *h);
		return option_chain_head_reference (ptr, h, file, line);
	}
	return 0;
}

//ScenSim-Port//int option_chain_head_reference (ptr, bp, file, line)
//ScenSim-Port//	struct option_chain_head **ptr;
//ScenSim-Port//	struct option_chain_head *bp;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int option_chain_head_reference (struct option_chain_head **ptr, struct option_chain_head *bp, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct option_chain_head *)0;
//ScenSim-Port//#endif
	}
	*ptr = bp;
	bp -> refcnt++;
	rc_register (file, line, ptr, bp, bp -> refcnt, 0, RC_MISC);
	return 1;
}

//ScenSim-Port//int option_chain_head_dereference (ptr, file, line)
//ScenSim-Port//	struct option_chain_head **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int option_chain_head_dereference (struct option_chain_head **ptr, const char *file, int line)//ScenSim-Port//
{
	struct option_chain_head *option_chain_head;
	pair car, cdr;

	if (!ptr || !*ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

	option_chain_head = *ptr;
	*ptr = (struct option_chain_head *)0;
	--option_chain_head -> refcnt;
	rc_register (file, line, ptr, option_chain_head,
		     option_chain_head -> refcnt, 1, RC_MISC);
	if (option_chain_head -> refcnt > 0)
		return 1;

	if (option_chain_head -> refcnt < 0) {
		log_error ("%s(%d): negative refcnt!", file, line);
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (option_chain_head);
//ScenSim-Port//#endif
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

	/* If there are any options on this head, free them. */
	for (car = option_chain_head -> first; car; car = cdr) {
		cdr = car -> cdr;
		if (car -> car)
			option_cache_dereference ((struct option_cache **)
						  (&car -> car), MDL);
		dfree (car, MDL);
		car = cdr;
	}

	dfree (option_chain_head, file, line);
	return 1;
}

//ScenSim-Port//int group_allocate (ptr, file, line)
//ScenSim-Port//	struct group **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int group_allocate (struct group **ptr, const char *file, int line)//ScenSim-Port//
{
	struct group *g;

	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct group *)0;
//ScenSim-Port//#endif
	}

//ScenSim-Port//	g = dmalloc (sizeof *g, file, line);
	g = (group*)dmalloc (sizeof *g, file, line);//ScenSim-Port//
	if (g) {
		memset (g, 0, sizeof *g);
		return group_reference (ptr, g, file, line);
	}
	return 0;
}

//ScenSim-Port//int group_reference (ptr, bp, file, line)
//ScenSim-Port//	struct group **ptr;
//ScenSim-Port//	struct group *bp;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int group_reference (struct group **ptr, struct group *bp, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct group *)0;
//ScenSim-Port//#endif
	}
	*ptr = bp;
	bp -> refcnt++;
	rc_register (file, line, ptr, bp, bp -> refcnt, 0, RC_MISC);
	return 1;
}

//ScenSim-Port//int group_dereference (ptr, file, line)
//ScenSim-Port//	struct group **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int group_dereference (struct group **ptr, const char *file, int line)//ScenSim-Port//
{
	struct group *group;

	if (!ptr || !*ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

	group = *ptr;
	*ptr = (struct group *)0;
	--group -> refcnt;
	rc_register (file, line, ptr, group, group -> refcnt, 1, RC_MISC);
	if (group -> refcnt > 0)
		return 1;

	if (group -> refcnt < 0) {
		log_error ("%s(%d): negative refcnt!", file, line);
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (group);
//ScenSim-Port//#endif
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

//ScenSim-Port//	if (group -> object)
//ScenSim-Port//		group_object_dereference (&group -> object, file, line);
//ScenSim-Port//	if (group -> subnet)	
//ScenSim-Port//		subnet_dereference (&group -> subnet, file, line);
//ScenSim-Port//	if (group -> shared_network)
//ScenSim-Port//		shared_network_dereference (&group -> shared_network,
//ScenSim-Port//					    file, line);
	if (group -> statements)
		executable_statement_dereference (&group -> statements,
						  file, line);
	if (group -> next)
		group_dereference (&group -> next, file, line);
	dfree (group, file, line);
	return 1;
}

//ScenSim-Port//struct dhcp_packet *new_dhcp_packet (file, line)
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
struct dhcp_packet *new_dhcp_packet (const char *file, int line)//ScenSim-Port//
{
	struct dhcp_packet *rval;
	rval = (struct dhcp_packet *)dmalloc (sizeof (struct dhcp_packet),
					      file, line);
	return rval;
}

//ScenSim-Port//struct protocol *new_protocol (file, line)
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
struct protocol *new_protocol (const char *file, int line)//ScenSim-Port//
{
//ScenSim-Port//	struct protocol *rval = dmalloc (sizeof (struct protocol), file, line);
	struct protocol *rval = (protocol*)dmalloc (sizeof (struct protocol), file, line);//ScenSim-Port//
	return rval;
}

//ScenSim-Port//struct domain_search_list *new_domain_search_list (file, line)
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
struct domain_search_list *new_domain_search_list (const char *file, int line)//ScenSim-Port//
{
	struct domain_search_list *rval =
//ScenSim-Port//		dmalloc (sizeof (struct domain_search_list), file, line);
		(domain_search_list*)dmalloc (sizeof (struct domain_search_list), file, line);//ScenSim-Port//
	return rval;
}

//ScenSim-Port//struct name_server *new_name_server (file, line)
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
struct name_server *new_name_server (const char *file, int line)//ScenSim-Port//
{
	struct name_server *rval =
//ScenSim-Port//		dmalloc (sizeof (struct name_server), file, line);
		(name_server*)dmalloc (sizeof (struct name_server), file, line);//ScenSim-Port//
	return rval;
}

//ScenSim-Port//void free_name_server (ptr, file, line)
//ScenSim-Port//	struct name_server *ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
void free_name_server (struct name_server *ptr, const char *file, int line)//ScenSim-Port//
{
	dfree ((void *)ptr, file, line);
}

//ScenSim-Port//struct option *new_option (name, file, line)
//ScenSim-Port//	const char *name;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
struct option *new_option (const char *name, const char *file, int line)//ScenSim-Port//
{
	struct option *rval;
	int len;

	len = strlen(name);

//ScenSim-Port//	rval = dmalloc(sizeof(struct option) + len + 1, file, line);
	rval = (option*)dmalloc(sizeof(struct option) + len + 1, file, line);//ScenSim-Port//

	if(rval) {
		memcpy(rval + 1, name, len);
		rval->name = (char *)(rval + 1);
	}

	return rval;
}

//ScenSim-Port//struct universe *new_universe (file, line)
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
struct universe *new_universe (const char *file, int line)//ScenSim-Port//
{
	struct universe *rval =
//ScenSim-Port//		dmalloc (sizeof (struct universe), file, line);
		(universe*)dmalloc (sizeof (struct universe), file, line);//ScenSim-Port//
	return rval;
}

//ScenSim-Port//void free_universe (ptr, file, line)
//ScenSim-Port//	struct universe *ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
void free_universe (struct universe *ptr, const char *file, int line)//ScenSim-Port//
{
	dfree ((void *)ptr, file, line);
}

//ScenSim-Port//void free_domain_search_list (ptr, file, line)
//ScenSim-Port//	struct domain_search_list *ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
void free_domain_search_list (struct domain_search_list *ptr, const char *file, int line)//ScenSim-Port//
{
	dfree ((void *)ptr, file, line);
}

//ScenSim-Port//void free_protocol (ptr, file, line)
//ScenSim-Port//	struct protocol *ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
void free_protocol (struct protocol *ptr, const char *file, int line)//ScenSim-Port//
{
	dfree ((void *)ptr, file, line);
}

//ScenSim-Port//void free_dhcp_packet (ptr, file, line)
//ScenSim-Port//	struct dhcp_packet *ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
void free_dhcp_packet (struct dhcp_packet *ptr, const char *file, int line)//ScenSim-Port//
{
	dfree ((void *)ptr, file, line);
}

//ScenSim-Port//struct client_lease *new_client_lease (file, line)
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
struct client_lease *new_client_lease (const char *file, int line)//ScenSim-Port//
{
	return (struct client_lease *)dmalloc (sizeof (struct client_lease),
					       file, line);
}

//ScenSim-Port//void free_client_lease (lease, file, line)
//ScenSim-Port//	struct client_lease *lease;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
void free_client_lease (struct client_lease *lease, const char *file, int line)//ScenSim-Port//
{
	dfree (lease, file, line);
}

pair free_pairs;

//ScenSim-Port//pair new_pair (file, line)
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
pair new_pair (const char *file, int line)//ScenSim-Port//
{
	pair foo;

	if (free_pairs) {
		foo = free_pairs;
		free_pairs = foo -> cdr;
		memset (foo, 0, sizeof *foo);
		dmalloc_reuse (foo, file, line, 0);
		return foo;
	}

//ScenSim-Port//	foo = dmalloc (sizeof *foo, file, line);
	foo = (pair)dmalloc (sizeof *foo, file, line);//ScenSim-Port//
	if (!foo)
		return foo;
	memset (foo, 0, sizeof *foo);
	return foo;
}

//ScenSim-Port//void free_pair (foo, file, line)
//ScenSim-Port//	pair foo;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
void free_pair (pair foo, const char *file, int line)//ScenSim-Port//
{
	foo -> cdr = free_pairs;
	free_pairs = foo;
	dmalloc_reuse (free_pairs, __FILE__, __LINE__, 0);
}

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
void relinquish_free_pairs ()
{
	pair pf, pc;

	for (pf = free_pairs; pf; pf = pc) {
		pc = pf -> cdr;
		dfree (pf, MDL);
	}
	free_pairs = (pair)0;
}
//ScenSim-Port//#endif

struct expression *free_expressions;

//ScenSim-Port//int expression_allocate (cptr, file, line)
//ScenSim-Port//	struct expression **cptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int expression_allocate (struct expression **cptr, const char *file, int line)//ScenSim-Port//
{
	struct expression *rval;

	if (free_expressions) {
		rval = free_expressions;
//ScenSim-Port//		free_expressions = rval -> data.not;
		free_expressions = rval -> data.not_;//ScenSim-Port//
		dmalloc_reuse (rval, file, line, 1);
	} else {
//ScenSim-Port//		rval = dmalloc (sizeof (struct expression), file, line);
		rval = (expression*)dmalloc (sizeof (struct expression), file, line);//ScenSim-Port//
		if (!rval)
			return 0;
	}
	memset (rval, 0, sizeof *rval);
	return expression_reference (cptr, rval, file, line);
}

//ScenSim-Port//int expression_reference (ptr, src, file, line)
//ScenSim-Port//	struct expression **ptr;
//ScenSim-Port//	struct expression *src;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int expression_reference (struct expression **ptr, struct expression *src, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct expression *)0;
//ScenSim-Port//#endif
	}
	*ptr = src;
	src -> refcnt++;
	rc_register (file, line, ptr, src, src -> refcnt, 0, RC_MISC);
	return 1;
}

//ScenSim-Port//void free_expression (expr, file, line)
//ScenSim-Port//	struct expression *expr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
void free_expression (struct expression *expr, const char *file, int line)//ScenSim-Port//
{
//ScenSim-Port//	expr -> data.not = free_expressions;
	expr -> data.not_ = free_expressions;//ScenSim-Port//
	free_expressions = expr;
	dmalloc_reuse (free_expressions, __FILE__, __LINE__, 0);
}

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
void relinquish_free_expressions ()
{
	struct expression *e, *n;

	for (e = free_expressions; e; e = n) {
//ScenSim-Port//		n = e -> data.not;
		n = e -> data.not_;//ScenSim-Port//
		dfree (e, MDL);
	}
	free_expressions = (struct expression *)0;
}
//ScenSim-Port//#endif

struct binding_value *free_binding_values;
				
//ScenSim-Port//int binding_value_allocate (cptr, file, line)
//ScenSim-Port//	struct binding_value **cptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int binding_value_allocate (struct binding_value **cptr, const char *file, int line)//ScenSim-Port//
{
	struct binding_value *rval;

	if (free_binding_values) {
		rval = free_binding_values;
		free_binding_values = rval -> value.bv;
		dmalloc_reuse (rval, file, line, 1);
	} else {
//ScenSim-Port//		rval = dmalloc (sizeof (struct binding_value), file, line);
		rval = (binding_value*)dmalloc (sizeof (struct binding_value), file, line);//ScenSim-Port//
		if (!rval)
			return 0;
	}
	memset (rval, 0, sizeof *rval);
	return binding_value_reference (cptr, rval, file, line);
}

//ScenSim-Port//int binding_value_reference (ptr, src, file, line)
//ScenSim-Port//	struct binding_value **ptr;
//ScenSim-Port//	struct binding_value *src;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int binding_value_reference (struct binding_value **ptr, struct binding_value *src, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct binding_value *)0;
//ScenSim-Port//#endif
	}
	*ptr = src;
	src -> refcnt++;
	rc_register (file, line, ptr, src, src -> refcnt, 0, RC_MISC);
	return 1;
}

//ScenSim-Port//void free_binding_value (bv, file, line)
//ScenSim-Port//	struct binding_value *bv;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
void free_binding_value (struct binding_value *bv, const char *file, int line)//ScenSim-Port//
{
	bv -> value.bv = free_binding_values;
	free_binding_values = bv;
	dmalloc_reuse (free_binding_values, (char *)0, 0, 0);
}

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
void relinquish_free_binding_values ()
{
	struct binding_value *b, *n;

	for (b = free_binding_values; b; b = n) {
		n = b -> value.bv;
		dfree (b, MDL);
	}
	free_binding_values = (struct binding_value *)0;
}
//ScenSim-Port//#endif

//ScenSim-Port//int fundef_allocate (cptr, file, line)
//ScenSim-Port//	struct fundef **cptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int fundef_allocate (struct fundef **cptr, const char *file, int line)//ScenSim-Port//
{
	struct fundef *rval;

//ScenSim-Port//	rval = dmalloc (sizeof (struct fundef), file, line);
	rval = (fundef*)dmalloc (sizeof (struct fundef), file, line);//ScenSim-Port//
	if (!rval)
		return 0;
	memset (rval, 0, sizeof *rval);
	return fundef_reference (cptr, rval, file, line);
}

//ScenSim-Port//int fundef_reference (ptr, src, file, line)
//ScenSim-Port//	struct fundef **ptr;
//ScenSim-Port//	struct fundef *src;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int fundef_reference (struct fundef **ptr, struct fundef *src, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct fundef *)0;
//ScenSim-Port//#endif
	}
	*ptr = src;
	src -> refcnt++;
	rc_register (file, line, ptr, src, src -> refcnt, 0, RC_MISC);
	return 1;
}

struct option_cache *free_option_caches;

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
void relinquish_free_option_caches ()
{
	struct option_cache *o, *n;

	for (o = free_option_caches; o; o = n) {
		n = (struct option_cache *)(o -> expression);
		dfree (o, MDL);
	}
	free_option_caches = (struct option_cache *)0;
}
//ScenSim-Port//#endif

//ScenSim-Port//int option_cache_allocate (cptr, file, line)
//ScenSim-Port//	struct option_cache **cptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int option_cache_allocate (struct option_cache **cptr, const char *file, int line)//ScenSim-Port//
{
	struct option_cache *rval;

	if (free_option_caches) {
		rval = free_option_caches;
		free_option_caches =
			(struct option_cache *)(rval -> expression);
		dmalloc_reuse (rval, file, line, 0);
	} else {
//ScenSim-Port//		rval = dmalloc (sizeof (struct option_cache), file, line);
		rval = (struct option_cache*)dmalloc (sizeof (struct option_cache), file, line);//ScenSim-Port//
		if (!rval)
			return 0;
	}
	memset (rval, 0, sizeof *rval);
	return option_cache_reference (cptr, rval, file, line);
}

//ScenSim-Port//int option_cache_reference (ptr, src, file, line)
//ScenSim-Port//	struct option_cache **ptr;
//ScenSim-Port//	struct option_cache *src;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int option_cache_reference (struct option_cache **ptr, struct option_cache *src, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct option_cache *)0;
//ScenSim-Port//#endif
	}
	*ptr = src;
	src -> refcnt++;
	rc_register (file, line, ptr, src, src -> refcnt, 0, RC_MISC);
	return 1;
}

//ScenSim-Port//int buffer_allocate (ptr, len, file, line)
//ScenSim-Port//	struct buffer **ptr;
//ScenSim-Port//	unsigned len;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int buffer_allocate (struct buffer **ptr, unsigned len, const char *file, int line)//ScenSim-Port//
{
	struct buffer *bp;

	/* XXXSK: should check for bad ptr values, otherwise we 
		  leak memory if they are wrong */
//ScenSim-Port//	bp = dmalloc (len + sizeof *bp, file, line);
	bp = (buffer*)dmalloc (len + sizeof *bp, file, line);//ScenSim-Port//
	if (!bp)
		return 0;
	/* XXXSK: both of these initializations are unnecessary */
	memset (bp, 0, sizeof *bp);
	bp -> refcnt = 0;
	return buffer_reference (ptr, bp, file, line);
}

//ScenSim-Port//int buffer_reference (ptr, bp, file, line)
//ScenSim-Port//	struct buffer **ptr;
//ScenSim-Port//	struct buffer *bp;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int buffer_reference (struct buffer **ptr, struct buffer *bp, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct buffer *)0;
//ScenSim-Port//#endif
	}
	*ptr = bp;
	bp -> refcnt++;
	rc_register (file, line, ptr, bp, bp -> refcnt, 0, RC_MISC);
	return 1;
}

//ScenSim-Port//int buffer_dereference (ptr, file, line)
//ScenSim-Port//	struct buffer **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int buffer_dereference (struct buffer **ptr, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

	if (!*ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

	(*ptr) -> refcnt--;
	rc_register (file, line, ptr, *ptr, (*ptr) -> refcnt, 1, RC_MISC);
	if (!(*ptr) -> refcnt) {
		dfree ((*ptr), file, line);
	} else if ((*ptr) -> refcnt < 0) {
		log_error ("%s(%d): negative refcnt!", file, line);
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (*ptr);
//ScenSim-Port//#endif
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	*ptr = (struct buffer *)0;
	return 1;
}

//ScenSim-Port//int dns_host_entry_allocate (ptr, hostname, file, line)
//ScenSim-Port//	struct dns_host_entry **ptr;
//ScenSim-Port//	const char *hostname;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int dns_host_entry_allocate (struct dns_host_entry **ptr, const char *hostname, const char *file, int line)//ScenSim-Port//
{
	struct dns_host_entry *bp;

//ScenSim-Port//	bp = dmalloc (strlen (hostname) + sizeof *bp, file, line);
	bp = (dns_host_entry*)dmalloc (strlen (hostname) + sizeof *bp, file, line);//ScenSim-Port//
	if (!bp)
		return 0;
	memset (bp, 0, sizeof *bp);
	bp -> refcnt = 0;
	strcpy (bp -> hostname, hostname);
	return dns_host_entry_reference (ptr, bp, file, line);
}

//ScenSim-Port//int dns_host_entry_reference (ptr, bp, file, line)
//ScenSim-Port//	struct dns_host_entry **ptr;
//ScenSim-Port//	struct dns_host_entry *bp;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int dns_host_entry_reference (struct dns_host_entry **ptr, struct dns_host_entry *bp, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct dns_host_entry *)0;
//ScenSim-Port//#endif
	}
	*ptr = bp;
	bp -> refcnt++;
	rc_register (file, line, ptr, bp, bp -> refcnt, 0, RC_MISC);
	return 1;
}

//ScenSim-Port//int dns_host_entry_dereference (ptr, file, line)
//ScenSim-Port//	struct dns_host_entry **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int dns_host_entry_dereference (struct dns_host_entry **ptr, const char *file, int line)//ScenSim-Port//
{
	if (!ptr || !*ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

	(*ptr) -> refcnt--;
	rc_register (file, line, ptr, *ptr, (*ptr) -> refcnt, 1, RC_MISC);
	if (!(*ptr) -> refcnt)
		dfree ((*ptr), file, line);
	if ((*ptr) -> refcnt < 0) {
		log_error ("%s(%d): negative refcnt!", file, line);
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (*ptr);
//ScenSim-Port//#endif
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	*ptr = (struct dns_host_entry *)0;
	return 1;
}

//ScenSim-Port//int option_state_allocate (ptr, file, line)
//ScenSim-Port//	struct option_state **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int option_state_allocate (struct option_state **ptr, const char *file, int line)//ScenSim-Port//
{
	unsigned size;

	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct option_state *)0;
//ScenSim-Port//#endif
	}

//ScenSim-Port//	size = sizeof **ptr + (universe_count - 1) * sizeof (void *);
	size = sizeof **ptr + (universe_count_ - 1) * sizeof (void *);//ScenSim-Port//
//ScenSim-Port//	*ptr = dmalloc (size, file, line);
	*ptr = (option_state*)dmalloc (size, file, line);//ScenSim-Port//
	if (*ptr) {
		memset (*ptr, 0, size);
//ScenSim-Port//		(*ptr) -> universe_count = universe_count;
		(*ptr) -> universe_count = universe_count_;//ScenSim-Port//
		(*ptr) -> refcnt = 1;
		rc_register (file, line,
			     ptr, *ptr, (*ptr) -> refcnt, 0, RC_MISC);
		return 1;
	}
	return 0;
}

//ScenSim-Port//int option_state_reference (ptr, bp, file, line)
//ScenSim-Port//	struct option_state **ptr;
//ScenSim-Port//	struct option_state *bp;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int option_state_reference (struct option_state **ptr, struct option_state *bp, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct option_state *)0;
//ScenSim-Port//#endif
	}
	*ptr = bp;
	bp -> refcnt++;
	rc_register (file, line, ptr, bp, bp -> refcnt, 0, RC_MISC);
	return 1;
}

//ScenSim-Port//int option_state_dereference (ptr, file, line)
//ScenSim-Port//	struct option_state **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int option_state_dereference (struct option_state **ptr, const char *file, int line)//ScenSim-Port//
{
	int i;
	struct option_state *options;

	if (!ptr || !*ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

	options = *ptr;
	*ptr = (struct option_state *)0;
	--options -> refcnt;
	rc_register (file, line, ptr, options, options -> refcnt, 1, RC_MISC);
	if (options -> refcnt > 0)
		return 1;

	if (options -> refcnt < 0) {
		log_error ("%s(%d): negative refcnt!", file, line);
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (options);
//ScenSim-Port//#endif
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

	/* Loop through the per-universe state. */
	for (i = 0; i < options -> universe_count; i++)
		if (options -> universes [i] &&
//ScenSim-Port//		    universes [i] -> option_state_dereference)
		    universes_ [i] -> option_state_dereference)
//ScenSim-Port//			((*(universes [i] -> option_state_dereference))
			((*(universes_ [i] -> option_state_dereference))
//ScenSim-Port//			 (universes [i], options, file, line));
			 (universes_ [i], options, file, line));

	dfree (options, file, line);
	return 1;
}

//ScenSim-Port//int executable_statement_allocate (ptr, file, line)
//ScenSim-Port//	struct executable_statement **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int executable_statement_allocate (struct executable_statement **ptr, const char *file, int line)//ScenSim-Port//
{
	struct executable_statement *bp;

//ScenSim-Port//	bp = dmalloc (sizeof *bp, file, line);
	bp = (executable_statement*)dmalloc (sizeof *bp, file, line);//ScenSim-Port//
	if (!bp)
		return 0;
	memset (bp, 0, sizeof *bp);
	return executable_statement_reference (ptr, bp, file, line);
}

//ScenSim-Port//int executable_statement_reference (ptr, bp, file, line)
//ScenSim-Port//	struct executable_statement **ptr;
//ScenSim-Port//	struct executable_statement *bp;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int executable_statement_reference (struct executable_statement **ptr, struct executable_statement *bp, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct executable_statement *)0;
//ScenSim-Port//#endif
	}
	*ptr = bp;
	bp -> refcnt++;
	rc_register (file, line, ptr, bp, bp -> refcnt, 0, RC_MISC);
	return 1;
}

static struct packet *free_packets;

//ScenSim-Port//#if defined (DEBUG_MEMORY_LEAKAGE) || \
//ScenSim-Port//		defined (DEBUG_MEMORY_LEAKAGE_ON_EXIT)
void relinquish_free_packets ()
{
	struct packet *p, *n;
	for (p = free_packets; p; p = n) {
		n = (struct packet *)(p -> raw);
		dfree (p, MDL);
	}
	free_packets = (struct packet *)0;
}
//ScenSim-Port//#endif

//ScenSim-Port//int packet_allocate (ptr, file, line)
//ScenSim-Port//	struct packet **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int packet_allocate (struct packet **ptr, const char *file, int line)//ScenSim-Port//
{
	struct packet *p;

	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct packet *)0;
//ScenSim-Port//#endif
	}

	if (free_packets) {
		p = free_packets;
		free_packets = (struct packet *)(p -> raw);
		dmalloc_reuse (p, file, line, 1);
	} else {
//ScenSim-Port//		p = dmalloc (sizeof *p, file, line);
		p = (packet*)dmalloc (sizeof *p, file, line);//ScenSim-Port//
	}
	if (p) {
		memset (p, 0, sizeof *p);
		return packet_reference (ptr, p, file, line);
	}
	return 0;
}

//ScenSim-Port//int packet_reference (ptr, bp, file, line)
//ScenSim-Port//	struct packet **ptr;
//ScenSim-Port//	struct packet *bp;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int packet_reference (struct packet **ptr, struct packet *bp, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct packet *)0;
//ScenSim-Port//#endif
	}
	*ptr = bp;
	bp -> refcnt++;
	rc_register (file, line, ptr, bp, bp -> refcnt, 0, RC_MISC);
	return 1;
}

//ScenSim-Port//int packet_dereference (ptr, file, line)
//ScenSim-Port//	struct packet **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int packet_dereference (struct packet **ptr, const char *file, int line)//ScenSim-Port//
{
	int i;
	struct packet *packet;

	if (!ptr || !*ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

	packet = *ptr;
	*ptr = (struct packet *)0;
	--packet -> refcnt;
	rc_register (file, line, ptr, packet, packet -> refcnt, 1, RC_MISC);
	if (packet -> refcnt > 0)
		return 1;

	if (packet -> refcnt < 0) {
		log_error ("%s(%d): negative refcnt!", file, line);
//ScenSim-Port//#if defined (DEBUG_RC_HISTORY)
//ScenSim-Port//		dump_rc_history (packet);
//ScenSim-Port//#endif
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

	if (packet -> options)
		option_state_dereference (&packet -> options, file, line);
	if (packet -> interface)
		interface_dereference (&packet -> interface, MDL);
//ScenSim-Port//	if (packet -> shared_network)
//ScenSim-Port//		shared_network_dereference (&packet -> shared_network, MDL);
	for (i = 0; i < packet -> class_count && i < PACKET_MAX_CLASSES; i++) {
		if (packet -> classes [i])
			omapi_object_dereference ((omapi_object_t **)
						  &packet -> classes [i], MDL);
	}
	packet -> raw = (struct dhcp_packet *)free_packets;
	free_packets = packet;
	dmalloc_reuse (free_packets, __FILE__, __LINE__, 0);
	return 1;
}

//ScenSim-Port//int dns_zone_allocate (ptr, file, line)
//ScenSim-Port//	struct dns_zone **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int dns_zone_allocate (struct dns_zone **ptr, const char *file, int line)//ScenSim-Port//
{
	struct dns_zone *d;

	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct dns_zone *)0;
//ScenSim-Port//#endif
	}

//ScenSim-Port//	d = dmalloc (sizeof *d, file, line);
	d = (dns_zone*)dmalloc (sizeof *d, file, line);//ScenSim-Port//
	if (d) {
		memset (d, 0, sizeof *d);
		return dns_zone_reference (ptr, d, file, line);
	}
	return 0;
}

//ScenSim-Port//int dns_zone_reference (ptr, bp, file, line)
//ScenSim-Port//	struct dns_zone **ptr;
//ScenSim-Port//	struct dns_zone *bp;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int dns_zone_reference (struct dns_zone **ptr, struct dns_zone *bp, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct dns_zone *)0;
//ScenSim-Port//#endif
	}
	*ptr = bp;
	bp -> refcnt++;
	rc_register (file, line, ptr, bp, bp -> refcnt, 0, RC_MISC);
	return 1;
}

//ScenSim-Port//int binding_scope_allocate (ptr, file, line)
//ScenSim-Port//	struct binding_scope **ptr;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int binding_scope_allocate (struct binding_scope **ptr, const char *file, int line)//ScenSim-Port//
{
	struct binding_scope *bp;

	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}

//ScenSim-Port//	bp = dmalloc (sizeof *bp, file, line);
	bp = (binding_scope*)dmalloc (sizeof *bp, file, line);//ScenSim-Port//
	if (!bp)
		return 0;
	memset (bp, 0, sizeof *bp);
	binding_scope_reference (ptr, bp, file, line);
	return 1;
}

//ScenSim-Port//int binding_scope_reference (ptr, bp, file, line)
//ScenSim-Port//	struct binding_scope **ptr;
//ScenSim-Port//	struct binding_scope *bp;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
int binding_scope_reference (struct binding_scope **ptr, struct binding_scope *bp, const char *file, int line)//ScenSim-Port//
{
	if (!ptr) {
		log_error ("%s(%d): null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		return 0;
//ScenSim-Port//#endif
	}
	if (*ptr) {
		log_error ("%s(%d): non-null pointer", file, line);
//ScenSim-Port//#if defined (POINTER_DEBUG)
//ScenSim-Port//		abort ();
//ScenSim-Port//#else
		*ptr = (struct binding_scope *)0;
//ScenSim-Port//#endif
	}
	*ptr = bp;
	bp -> refcnt++;
	rc_register (file, line, ptr, bp, bp -> refcnt, 0, RC_MISC);
	return 1;
}

/* Make a copy of the data in data_string, upping the buffer reference
   count if there's a buffer. */

void
data_string_copy(struct data_string *dest, const struct data_string *src,
		 const char *file, int line)
{
	if (src -> buffer) {
		buffer_reference (&dest -> buffer, src -> buffer, file, line);
	} else {
		dest->buffer = NULL;
	}
	dest -> data = src -> data;
	dest -> terminated = src -> terminated;
	dest -> len = src -> len;
}

/* Release the reference count to a data string's buffer (if any) and
   zero out the other information, yielding the null data string. */

//ScenSim-Port//void data_string_forget (data, file, line)
//ScenSim-Port//	struct data_string *data;
//ScenSim-Port//	const char *file;
//ScenSim-Port//	int line;
void data_string_forget (struct data_string *data, const char *file, int line)//ScenSim-Port//
{
	if (data -> buffer)
		buffer_dereference (&data -> buffer, file, line);
	memset (data, 0, sizeof *data);
}

/* If the data_string is larger than the specified length, reduce 
   the data_string to the specified size. */

//ScenSim-Port//void data_string_truncate (dp, len)
//ScenSim-Port//	struct data_string *dp;
//ScenSim-Port//	int len;
void data_string_truncate (struct data_string *dp, int len)//ScenSim-Port//
{
	/* XXX: do we need to consider the "terminated" flag in the check? */
	if (len < dp -> len) {
		dp -> terminated = 0;
		dp -> len = len;
	}
}
}//namespace IscDhcpPort////ScenSim-Port//
