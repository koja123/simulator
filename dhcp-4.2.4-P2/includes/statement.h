/* statement.h

   Definitions for executable statements... */

/*
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
 * This software has been written for Internet Systems Consortium
 * by Ted Lemon in cooperation with Vixie Enterprises and Nominum, Inc.
 * To learn more about Internet Systems Consortium, see
 * ``https://www.isc.org/''.  To learn more about Vixie Enterprises,
 * see ``http://www.vix.com''.   To learn more about Nominum, Inc., see
 * ``http://www.nominum.com''.
 */

struct executable_statement {
	int refcnt;
	struct executable_statement *next;
	enum statement_op {
		null_statement,
		if_statement,
		add_statement,
		eval_statement,
		break_statement,
		default_option_statement,
		supersede_option_statement,
		append_option_statement,
		prepend_option_statement,
		send_option_statement,
		statements_statement,
		on_statement,
		switch_statement,
		case_statement,
		default_statement,
		set_statement,
		unset_statement,
		let_statement,
		define_statement,
		log_statement,
		return_statement,
		execute_statement
#define null_statement executable_statement::null_statement//ScenSim-Port//
#define if_statement executable_statement::if_statement//ScenSim-Port//
#define add_statement executable_statement::add_statement//ScenSim-Port//
#define eval_statement executable_statement::eval_statement//ScenSim-Port//
#define break_statement executable_statement::break_statement//ScenSim-Port//
#define default_option_statement executable_statement::default_option_statement//ScenSim-Port//
#define supersede_option_statement executable_statement::supersede_option_statement//ScenSim-Port//
#define append_option_statement executable_statement::append_option_statement//ScenSim-Port//
#define prepend_option_statement executable_statement::prepend_option_statement//ScenSim-Port//
#define send_option_statement executable_statement::send_option_statement//ScenSim-Port//
#define statements_statement executable_statement::statements_statement//ScenSim-Port//
#define on_statement executable_statement::on_statement//ScenSim-Port//
#define switch_statement executable_statement::switch_statement//ScenSim-Port//
#define case_statement executable_statement::case_statement//ScenSim-Port//
#define default_statement executable_statement::default_statement//ScenSim-Port//
#define set_statement executable_statement::set_statement//ScenSim-Port//
#define unset_statement executable_statement::unset_statement//ScenSim-Port//
#define let_statement executable_statement::let_statement//ScenSim-Port//
#define define_statement executable_statement::define_statement//ScenSim-Port//
#define log_statement executable_statement::log_statement//ScenSim-Port//
#define return_statement executable_statement::return_statement//ScenSim-Port//
#define execute_statement executable_statement::execute_statement//ScenSim-Port//
	} op;
#define statement_op executable_statement::statement_op//ScenSim-Port//
	enum priority_enum {//ScenSim-Port//
		log_priority_fatal,//ScenSim-Port//
		log_priority_error,//ScenSim-Port//
		log_priority_debug,//ScenSim-Port//
		log_priority_info//ScenSim-Port//
#define log_priority_fatal executable_statement::log_priority_fatal//ScenSim-Port//
#define log_priority_error executable_statement::log_priority_error//ScenSim-Port//
#define log_priority_debug executable_statement::log_priority_debug//ScenSim-Port//
#define log_priority_info executable_statement::log_priority_info//ScenSim-Port//
	};//ScenSim-Port//
	union {
		struct {
			struct executable_statement *tc, *fc;
			struct expression *expr;
		} ie;
		struct expression *eval;
		struct expression *retval;
//ScenSim-Port//		struct class *add;
		struct class_type *add;//ScenSim-Port//
		struct option_cache *option;
		struct option_cache *supersede;
		struct option_cache *prepend;
		struct option_cache *append;
		struct executable_statement *statements;
		struct {
			int evtypes;
#			define ON_COMMIT  1
#			define ON_EXPIRY  2
#			define ON_RELEASE 4
#			define ON_TRANSMISSION 8
			struct executable_statement *statements;
		} on;
		struct {
			struct expression *expr;
			struct executable_statement *statements;
		} s_switch;
		struct expression *c_case;
		struct {
			char *name;
			struct expression *expr;
			struct executable_statement *statements;
		} set, let;
		char *unset;
		struct {
//ScenSim-Port//			enum {
//ScenSim-Port//				log_priority_fatal,
//ScenSim-Port//				log_priority_error,
//ScenSim-Port//				log_priority_debug,
//ScenSim-Port//				log_priority_info
//ScenSim-Port//			} priority;
			enum priority_enum priority;//ScenSim-Port//
			struct expression *expr;
		} log;
		struct {
			char *command;
			struct expression *arglist;
			int argc;
		} execute;
	} data;
};

