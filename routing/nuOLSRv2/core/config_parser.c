//
// Configuration file parser
//
#include "config.h"

#include <math.h>
#include <ctype.h>

#include "core/core.h"
#include "core/mem.h"
#include "core/config_parser.h"

#if defined(_WIN32) || defined(_WIN64)      //ScenSim-Port://
#define index(s, c)        strchr(s, c)     //ScenSim-Port://
#define strcasecmp(s1, s2) _stricmp(s1, s2) //ScenSim-Port://
#define strdup(s)          _strdup(s)       //ScenSim-Port://
#endif                                      //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_parser
 * @{
 */

#define T_EOF      (-1) ///< token type
#define T_OK       (-2) ///< token type
#define T_LP       (-4) ///< token type
#define T_RP       (-5) ///< token type
#define T_VALUE    (-7) ///< token type
#define T_ERR      (-8) ///< token type

static const char* SEP = "[]{}#";

/** Outputs the error message and abort the program.
 *
 * @param parser
 * @param msg
 */
PUBLIC void
nu_parser_fatal_error(nu_parser_t* parser, const char* msg, ...)
{
    va_list ap;
    va_start(ap, msg);
    fprintf(stderr, "%s:%d:", parser->config_filename, parser->line_num);
    vfprintf(stderr, msg, ap);
    va_end(ap);
    exit(1);
}

/** Outputs the error message and abort the program.
 *
 * @param parser
 */
PUBLIC void
nu_parser_syntax_error(nu_parser_t* parser)
{
    fprintf(stderr, "%s:%d:Syntax error around '%s'\n",
            parser->config_filename, parser->line_num, parser->token);
    exit(1);
}

/*
 */
static void
unexpected_eof_error(nu_parser_t* parser)
{
    nu_err("%s:%d:Unexpected EOF",
            parser->config_filename, parser->line_num);
    exit(1);
}

/*
 */
static int
skipspace(nu_parser_t* parser)
{
    int c;
    while ((c = fgetc(parser->fin)) != EOF) {
        if (c == '\n')
            ++parser->line_num;
        else if (!isspace(c))
            break;
    }
    return c;
}

/*
 */
static int
skipline(nu_parser_t* parser)
{
    int c;
    while ((c = fgetc(parser->fin)) != EOF && c != '\n')
        ;
    ++parser->line_num;
    return c;
}

/*
 */
static int
get_token(nu_parser_t* parser)
{
    int c = skipspace(parser);
    while (c == '#') {
        skipline(parser);
        c = skipspace(parser);
    }

    if (c == EOF)
        return T_EOF;
    else if (c == '{')
        return T_LP;
    else if (c == '}')
        return T_RP;

    char* p = parser->token;
    *p++ = static_cast<unsigned char>(c);
    while ((c = fgetc(parser->fin)) != EOF &&
           !isspace(c) && index(SEP, c) == NULL)
        *p++ = static_cast<unsigned char>(c);
    *p = '\0';
    if (c == '\n')
        ungetc(c, parser->fin);

    if (parser->cur_tags) {
        for (int i = 0; parser->cur_tags[i].name != 0; ++i) {
            if (strcasecmp(parser->cur_tags[i].name, parser->token) == 0)
                return i;
        }
    }
    return T_VALUE;
}

/** Reads one word.
 *
 * @param parser
 * @param[out] value
 * @return true if success
 */
PUBLIC nu_bool_t
nu_parser_get_strp(nu_parser_t* parser, const char** value)
{
    switch (get_token(parser)) {
    case T_EOF:
    case T_LP:
    case T_RP:
    case T_ERR:
        *value = 0;
        return false;

    default:
        *value = parser->token;
        return true;
    }
}

/** Reads boolean.
 *
 * @param parser
 * @param[out] value
 * @return true if success
 */
PUBLIC nu_bool_t
nu_parser_get_bool(nu_parser_t* parser, nu_bool_t* value)
{
    if (get_token(parser) != T_VALUE)
        return false;
    if (strcasecmp(parser->token, "true") == 0 ||
        strcasecmp(parser->token, "yes") == 0) {
        *value = true;
        return true;
    }
    if (strcasecmp(parser->token, "false") == 0 ||
        strcasecmp(parser->token, "no") == 0) {
        *value = false;
        return true;
    }
    return false;
}

/** Reads integer.
 *
 * @param parser
 * @param[out] value
 * @return true if success
 */
PUBLIC nu_bool_t
nu_parser_get_int(nu_parser_t* parser, int* value)
{
    if (get_token(parser) != T_VALUE)
        return false;
    char* p;
    *value = (int)strtod(parser->token, &p);
    if (p != NULL && *p != '\0')
        return false;
    return true;
}

/** Reads float value.
 *
 * @param parser
 * @param[out] value
 * @return true if success
 */
PUBLIC nu_bool_t
nu_parser_get_float(nu_parser_t* parser, float* value)
{
    if (get_token(parser) != T_VALUE)
        return false;
#ifdef __USE_ISOC99
    char* p;
    *value = strtof(parser->token, &p);
    if (p != NULL && *p != '\0')
        return T_ERR;
#else
#if defined(_WIN32) || defined(_WIN64)   //ScenSim-Port://
    *value = (float)atof(parser->token); //ScenSim-Port://
#else                                    //ScenSim-Port://
    *value = atof(parser->token);
#endif                                   //ScenSim-Port://
#endif
    return true;
}

/** Read double value.
 *
 * @param parser
 * @param[out] value
 * @return true if success
 */
PUBLIC nu_bool_t
nu_parser_get_double(nu_parser_t* parser, double* value)
{
    if (get_token(parser) != T_VALUE)
        return false;
#ifdef __USE_ISOC99
    char* p = parser->token;
    *value = (double)strtof(parser->token, &p);
    if (p != NULL && *p != '\0')
        return false;
#else
    *value = atof(parser->token);
#endif
    return true;
}

/** Reads IPv4 address.
 *
 * @param parser
 * @param[out] ip
 * @return true if success
 */
PUBLIC nu_bool_t
nu_parser_get_ip4(nu_parser_t* parser, nu_ip_t* ip)
{
    return get_token(parser) == T_VALUE &&
           ((*ip = nu_ip4_create_with_str(parser->token)) != NU_IP_UNDEF);
}

/** Reads IPv6 address.
 *
 * @param parser
 * @param[out] ip
 * @return true if success
 */
PUBLIC nu_bool_t
nu_parser_get_ip6(nu_parser_t* parser, nu_ip_t* ip)
{
    return get_token(parser) == T_VALUE &&
           ((*ip = nu_ip6_create_with_str(parser->token)) != NU_IP_UNDEF);
}

/** Reads IP address.
 *
 * @param parser
 * @param[out] ip
 * @return true if success
 */
PUBLIC nu_bool_t
nu_parser_get_ip(nu_parser_t* parser, nu_ip_t* ip)
{
    if (NU_IS_V4)
        return nu_parser_get_ip4(parser, ip);
    else
        return nu_parser_get_ip6(parser, ip);
}

/*
 */
static void
set_value(nu_parser_t* parser, const nu_parser_tag_t* k, void* obj)
{
    int value_i = 0;
    uintptr_t base = (uintptr_t)obj;
    switch (k->type_code) {
    case VT_U8:
    case VT_U16:
    case VT_U32:
    case VT_INT:
        if (!nu_parser_get_int(parser, &value_i)) {
            nu_parser_syntax_error(parser);
            /* NOTREACHED */
        }
        switch (k->type_code) {
        case VT_U8:
            *(uint8_t*)(base + k->offset) = static_cast<uint8_t>(value_i);
            break;
        case VT_U16:
            *(uint16_t*)(base + k->offset) = static_cast<uint16_t>(value_i);
            break;
        case VT_U32:
            *(uint32_t*)(base + k->offset) = value_i;
            break;
        case VT_INT:
            *(int*)(base + k->offset) = value_i;
            break;
        default:
            break;
        }
        break;

    case VT_DOUBLE:
        if (!nu_parser_get_double(parser, (double*)(base + k->offset))) {
            nu_parser_syntax_error(parser);
            /* NOTREACHED */
        }
        break;

    case VT_BOOL:
        if (!nu_parser_get_bool(parser, (nu_bool_t*)(base + k->offset))) {
            nu_parser_syntax_error(parser);
            /* NOTREACHED */
        }
        break;

    case VT_FLAG:
        *(nu_bool_t*)(base + k->offset) = !*(nu_bool_t*)(base + k->offset);
        break;

    case VT_IP4:
        if (!nu_parser_get_ip4(parser, (nu_ip_t*)(base + k->offset))) {
            nu_parser_syntax_error(parser);
            /* NOTREACHED */
        }
        break;

    case VT_IP6:
        if (!nu_parser_get_ip6(parser, (nu_ip_t*)(base + k->offset))) {
            nu_parser_syntax_error(parser);
            /* NOTREACHED */
        }
        break;

    case VT_IP:
        if (!nu_parser_get_ip(parser, (nu_ip_t*)(base + k->offset))) {
            nu_parser_syntax_error(parser);
            /* NOTREACHED */
        }
        break;

    case VT_STR:
        if (get_token(parser) != T_VALUE) {
            nu_parser_syntax_error(parser);
            /* NOTREACHED */
        }
        *(char**)(base + k->offset) = strdup(parser->token);
        break;

    default:
        nu_fatal("Internal Error:Unsupport Value Type");
    }
}

/** Reads configuration file.
 *
 * @param parser
 * @param target    object to set the parameters
 * @param tags      parser tag table
 */
PUBLIC void
nu_parser_parse(nu_parser_t* parser,
        void* target, nu_parser_tag_t* tags)
{
    nu_parser_tag_t* save_tags = parser->cur_tags;
    ++parser->level;
    if (parser->level != 1) {
        if (get_token(parser) != T_LP) {
            nu_parser_syntax_error(parser);
            /* NOTREACHED */
        }
    }

    parser->cur_tags = tags;
    int tok;
    while ((tok = get_token(parser)) != T_EOF && tok != T_RP) {
        nu_parser_tag_t* k;
        if (tok < 0) {
            nu_parser_syntax_error(parser);
            /* NOTREACHED */
        }

        k = &parser->cur_tags[tok];
        if (k->type_code == VT_FUNC)
            (*k->func)(parser, target);
        else if (k->type_code == VT_FUNC2)
            (*k->func)(parser, k);
        else
            set_value(parser, k, target);
    }

    if (parser->level != 1) {
        if (tok == T_EOF) {
            unexpected_eof_error(parser);
            /* NOTREACHED */
        }
    }
    --parser->level;
    parser->cur_tags = save_tags;
}

/** Reads log flag list.
 *
 * @param parser
 * @param flags     log flag table
 */
PUBLIC void
nu_parser_parse_log_flags(nu_parser_t* parser,
        nu_parser_log_flag_t* flags)
{
    nu_parser_tag_t* save_tags = parser->cur_tags;
    ++parser->level;
    if (parser->level != 1) {
        if (get_token(parser) != T_LP) {
            nu_parser_syntax_error(parser);
            /* NOTREACHED */
        }
    }

    parser->cur_tags = NULL;
    int tok;
    while ((tok = get_token(parser)) != T_EOF && tok != T_RP) {
        if (tok != T_VALUE) {
            nu_parser_syntax_error(parser);
            /* NOTREACHED */
        }
        nu_parser_log_flag_t* p;
        for (p = flags; p->func != NULL; ++p) {
            if (p->func(parser->token))
                break;
        }
        if (p->func == NULL) {
            nu_parser_syntax_error(parser);
            /* NOTREACHED */
        }
    }

    if (parser->level != 1) {
        if (tok == T_EOF) {
            unexpected_eof_error(parser);
            /* NOTREACHED */
        }
    }
    --parser->level;
    parser->cur_tags = save_tags;
}

/** Reads log output level.
 *
 * @param parser
 * @param target
 */
PUBLIC void
nu_parse_log_output_level(nu_parser_t* parser, void* target)
{
    const char* strp;
    if (!nu_parser_get_strp(parser, &strp))
        nu_parser_syntax_error(parser);
    if (!nu_logger_set_output_prio_str(NU_LOGGER, strp))
        nu_parser_syntax_error(parser);
}

/** Creates parser object with file name.
 *
 * @param fn    parameter file name
 * @return parser object
 */
PUBLIC nu_parser_t*
nu_parser_create(char* fn)
{
    nu_parser_t* self = nu_mem_alloc(nu_parser_t);
    self->config_filename = fn;
    if ((self->fin = fopen(self->config_filename, "r")) == NULL) {
        perror(self->config_filename);
        exit(1);
    }
    self->line_num = 1;
    self->level = 0;
    return self;
}

/** Destroys the parser.
 *
 * @param self
 */
PUBLIC void
nu_parser_free(nu_parser_t* self)
{
    fclose(self->fin);
    nu_mem_free(self);
}

/** @} */

}//namespace// //ScenSim-Port://
