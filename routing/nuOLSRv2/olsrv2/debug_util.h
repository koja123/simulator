#ifndef NU_OLSRV2_DEBUG_UTIL_H_
#define NU_OLSRV2_DEBUG_UTIL_H_

#include "config.h" //ScenSim-Port://

#ifndef NU_NDEBUG

#define OLSRV2_LOG_INDENT    "    "

#if defined(_WIN32) || defined(_WIN64)                              //ScenSim-Port://
#define NU_DEBUG_PROCESS(msg, ...)                                \
    if (current_olsrv2->log_process &&                            \
        nu_logger_set_prio(NU_LOGGER, NU_LOGGER_DEBUG)) {         \
        nu_logger_log(NU_LOGGER, "PROCESSING:" msg, __VA_ARGS__); \
    }                                                               //ScenSim-Port://
#else                                                               //ScenSim-Port://
#define DEBUG_PROCESS(msg, ...)                                   \
    if (current_olsrv2->log_process &&                            \
        nu_logger_set_prio(NU_LOGGER, NU_LOGGER_DEBUG)) {         \
        nu_logger_log(NU_LOGGER, "PROCESSING:" msg, __VA_ARGS__); \
    }
#endif                                                              //ScenSim-Port://

#if defined(_WIN32) || defined(_WIN64)                      //ScenSim-Port://
#define NU_DEBUG_PROCESS0(msg)                            \
    if (current_olsrv2->log_process &&                    \
        nu_logger_set_prio(NU_LOGGER, NU_LOGGER_DEBUG)) { \
        nu_logger_log_cstr(NU_LOGGER, "PROCESSING:" msg); \
    }                                                       //ScenSim-Port://
#else                                                       //ScenSim-Port://
#define DEBUG_PROCESS0(msg)                               \
    if (current_olsrv2->log_process &&                    \
        nu_logger_set_prio(NU_LOGGER, NU_LOGGER_DEBUG)) { \
        nu_logger_log_cstr(NU_LOGGER, "PROCESSING:" msg); \
    }
#endif                                                      //ScenSim-Port://

#else

#define OLSRV2_LOG_INDENT ""           //ScenSim-Port://
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
#define NU_DEBUG_PROCESS(msg, ...)     //ScenSim-Port://
#define NU_DEBUG_PROCESS0(msg)         //ScenSim-Port://
#else                                  //ScenSim-Port://
#define DEBUG_PROCESS(msg, ...)
#define DEBUG_PROCESS0(msg)            //ScenSim-Port://
#endif                                 //ScenSim-Port://

#endif


#endif
