#ifndef NU_CORE_LOGGER_H_
#define NU_CORE_LOGGER_H_

#include "config.h" //ScenSim-Port://

#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
#define LOG_EMERG   0                  //ScenSim-Port://
#define LOG_ALERT   1                  //ScenSim-Port://
#define LOG_CRIT    2                  //ScenSim-Port://
#define LOG_ERR     3                  //ScenSim-Port://
#define LOG_WARNING 4                  //ScenSim-Port://
#define LOG_NOTICE  5                  //ScenSim-Port://
#define LOG_INFO    6                  //ScenSim-Port://
#define LOG_DEBUG   7                  //ScenSim-Port://
#else                                  //ScenSim-Port://
#include <sys/syslog.h>
#endif                                 //ScenSim-Port://

#include "core/strbuf.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_logger Core :: Logger
 * @{
 */

/*
 * Priority
 */
#define NU_LOGGER_DEBUG             LOG_DEBUG         ///< log priority
#define NU_LOGGER_INFO              LOG_INFO          ///< log priority
#define NU_LOGGER_WARN              LOG_WARNING       ///< log priority
#define NU_LOGGER_ERR               LOG_ERR           ///< log priority
#define NU_LOGGER_FATAL             LOG_EMERG         ///< log priority

/*
 * Outputs format
 */
#define NU_LOGGER_OUTPUT_NOW        (1 << 0)   ///< current time
#define NU_LOGGER_OUTPUT_TIME       (1 << 1)   ///< time
#define NU_LOGGER_OUTPUT_LEVEL      (1 << 2)   ///< priority

#define NU_LOGGER_PREFIX_MAX_NUM    (8)        ///< prefix stack size

/**
 * Logger
 */
typedef struct nu_logger {
    FILE*       fp;                               ///< log file
    int         priority;                         ///< current log priority
    int         output_priority;                  ///< output log priority
    int         format;                           ///< tag format
    const char* prefix[NU_LOGGER_PREFIX_MAX_NUM]; ///< prefix string stack
    int         sp;                               ///< prefix stack pointer
    char*       name;                             ///< hostname
    nu_strbuf_t buf;                              ///< message string buffer
} nu_logger_t;

PUBLIC void nu_logger_init(nu_logger_t*);
PUBLIC void nu_logger_destroy(nu_logger_t*);

PUBLIC void nu_logger_log(nu_logger_t*, const char*, ...);
PUBLIC void nu_logger_log_cstr(nu_logger_t*, const char*);
PUBLIC void nu_logger_log_strbuf(nu_logger_t*, const nu_strbuf_t*);

PUBLIC nu_bool_t nu_logger_set_prio(nu_logger_t*, const int prio);
PUBLIC void nu_logger_set_output_prio(nu_logger_t*, const int prio);
PUBLIC nu_bool_t nu_logger_set_output_prio_str(nu_logger_t*, const char*);
PUBLIC void nu_logger_set_more_verbose(nu_logger_t*);
PUBLIC void nu_logger_set_more_quiet(nu_logger_t*);

PUBLIC void nu_logger_set_fp(nu_logger_t*, FILE*);
PUBLIC void nu_logger_set_format(nu_logger_t*, const uint8_t);
PUBLIC void nu_logger_set_name(nu_logger_t*, const char*);

PUBLIC void nu_logger_push_prefix(nu_logger_t*, const char*);
PUBLIC void nu_logger_pop_prefix(nu_logger_t*);
PUBLIC void nu_logger_clear_prefix(nu_logger_t*);

//PUBLIC nu_bool_t nu_snprintf(char*, size_t, const char*, ...);

////////////////////////////////////////////////////////////////
//
// Utility macros for default logger
//

/**
 * Outputs fatal log message.
 */
#define nu_fatal(...)                                   \
    do {                                                \
        nu_logger_set_prio(NU_LOGGER, NU_LOGGER_FATAL); \
        nu_logger_log(NU_LOGGER, __VA_ARGS__);          \
        nu_logger_destroy(NU_LOGGER);                   \
        abort();                                        \
    }                                                   \
    while (0)

/**
 * Outputs error log message.
 */
#define nu_err(...)                                         \
    do {                                                    \
        if (nu_logger_set_prio(NU_LOGGER, NU_LOGGER_ERR)) { \
            nu_logger_log(NU_LOGGER, __VA_ARGS__);          \
        }                                                   \
    }                                                       \
    while (0)

/**
 * Outputs warning log message.
 */
#define nu_warn(...)                                         \
    do {                                                     \
        if (nu_logger_set_prio(NU_LOGGER, NU_LOGGER_WARN)) { \
            nu_logger_log(NU_LOGGER, __VA_ARGS__);           \
        }                                                    \
    }                                                        \
    while (0)

/**
 * Outputs log message for information.
 */
#define nu_info(...)                                         \
    do {                                                     \
        if (nu_logger_set_prio(NU_LOGGER, NU_LOGGER_INFO)) { \
            nu_logger_log(NU_LOGGER, __VA_ARGS__);           \
        }                                                    \
    }                                                        \
    while (0)

/**
 * Outputs log message for debug.
 */
#define nu_debug(...)                                         \
    do {                                                      \
        if (nu_logger_set_prio(NU_LOGGER, NU_LOGGER_DEBUG)) { \
            nu_logger_log(NU_LOGGER, __VA_ARGS__);            \
        }                                                     \
    }                                                         \
    while (0)

/** @} */

}//namespace// //ScenSim-Port://

#endif
