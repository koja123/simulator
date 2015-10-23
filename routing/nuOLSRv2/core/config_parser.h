#ifndef NU_CORE_CONFIG_PARSER_H_
#define NU_CORE_CONFIG_PARSER_H_

#include "core/ip.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_parser Core :: Configuration file parser
 * @{
 */

struct nu_parser;

/**
 * Callback function type for nu_parser_t.
 */
typedef void (*nu_parse_func_t)(struct nu_parser*, void* target);

/**
 * Configuration values' types.
 */
typedef enum nu_parser_value_type {
    VT_FUNC,
    VT_FUNC2,
    VT_INT,
    VT_FLOAT,
    VT_DOUBLE,
    VT_U8,
    VT_U16,
    VT_U32,
    VT_BOOL,
    VT_FLAG,
    VT_STR,
    VT_IP4,
    VT_IP6,
    VT_IP,
} nu_parser_value_type_t;

/**
 * Configuration tag.
 */
typedef struct nu_parser_tag {
    const char*            name;          ///< tag name
    nu_parser_value_type_t type_code;     ///< value type
    size_t                 offset;        ///< offset of the parameter
    nu_parse_func_t        func;          ///< callback function
} nu_parser_tag_t;

/**
 * Callback function for setting log flag
 */
typedef nu_bool_t (*nu_parser_set_log_flag_func_t)(const char*);

/**
 * Log flags
 */
typedef struct nu_parser_log_flag {
    nu_parser_set_log_flag_func_t func;   ///< callback function
} nu_parser_log_flag_t;

/**
 * Parser object
 */
typedef struct nu_parser {
    FILE*            fin;                 ///< file input stream
    char             token[128];          ///< token buffer
    char*            config_filename;     ///< param. file name
    int              line_num;            ///< current line number
    int              level;               ///< level of tag table
    nu_parser_tag_t* cur_tags;            ///< current tag table
} nu_parser_t;

PUBLIC nu_parser_t* nu_parser_create(char* filename);
PUBLIC void nu_parser_free(nu_parser_t*);

PUBLIC void nu_parser_parse(nu_parser_t*, void*, nu_parser_tag_t*);

PUBLIC void nu_parser_parse_log_flags(nu_parser_t*,
        nu_parser_log_flag_t*);
PUBLIC void nu_parse_log_output_level(nu_parser_t*, void*);

PUBLIC nu_bool_t nu_parser_get_bool(nu_parser_t*, nu_bool_t*);
PUBLIC nu_bool_t nu_parser_get_int(nu_parser_t*, int*);
PUBLIC nu_bool_t nu_parser_get_float(nu_parser_t*, float*);
PUBLIC nu_bool_t nu_parser_get_double(nu_parser_t*, double*);
PUBLIC nu_bool_t nu_parser_get_strp(nu_parser_t*, const char**);
PUBLIC nu_bool_t nu_parser_get_ip(nu_parser_t*, nu_ip_t*);
PUBLIC nu_bool_t nu_parser_get_ip4(nu_parser_t*, nu_ip_t*);
PUBLIC nu_bool_t nu_parser_get_ip6(nu_parser_t*, nu_ip_t*);
PUBLIC void nu_parser_syntax_error(nu_parser_t*);
PUBLIC void nu_parser_fatal_error(nu_parser_t*, const char*, ...);

/** @} */

}//namespace// //ScenSim-Port://

#endif
