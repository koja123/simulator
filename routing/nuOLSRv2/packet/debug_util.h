#ifndef NU_PACKET_DEBUG_UTIL_H_
#define NU_PACKET_DEBUG_UTIL_H_

#include "config.h" //ScenSim-Port://

#ifndef NU_NDEBUG

#define PACKET_LOG_INDENT    "    "

#define DEBUG_MESSAGE_PACKING(cmd)                        \
    if (current_packet->log_message_packing &&            \
        nu_logger_set_prio(NU_LOGGER, NU_LOGGER_DEBUG)) { \
        cmd;                                              \
    }

#define DEBUG_MESSAGE_PACKING_PUSH_PREFIX(str) \
    DEBUG_MESSAGE_PACKING(nu_logger_push_prefix(NU_LOGGER, str))

#define DEBUG_MESSAGE_PACKING_POP_PREFIX()                  \
    do {                                                    \
        if (current_packet->log_message_packing &&          \
            nu_logger_set_prio(NU_LOGGER, NU_LOGGER_DEBUG)) \
            nu_logger_pop_prefix(NU_LOGGER);                \
    }                                                       \
    while (0)

#if defined(_WIN32) || defined(_WIN64)                    //ScenSim-Port://
#define DEBUG_MESSAGE_PACKING_LOG(format, ...)          \
    if (current_packet->log_message_packing &&          \
        nu_logger_set_prio(NU_LOGGER, NU_LOGGER_DEBUG)) \
        nu_logger_log(NU_LOGGER, format, ## __VA_ARGS__); //ScenSim-Port://
#else                                                     //ScenSim-Port://
#define DEBUG_MESSAGE_PACKING_LOG(format, args ...)     \
    if (current_packet->log_message_packing &&          \
        nu_logger_set_prio(NU_LOGGER, NU_LOGGER_DEBUG)) \
        nu_logger_log(NU_LOGGER, format, ## args);
#endif                                                    //ScenSim-Port://

#define DEBUG_MESSAGE_PACKING_SEM(type, sem)              \
    if (current_packet->log_message_packing &&            \
        nu_logger_set_prio(NU_LOGGER, NU_LOGGER_DEBUG)) { \
        char buf[80];                                     \
        type ## _to_s(sem, buf, sizeof(buf));             \
        nu_logger_log(NU_LOGGER, "flags:%s", buf);        \
    }

#else

#define PACKET_LOG_INDENT ""                        //ScenSim-Port://
#define DEBUG_MESSAGE_PACKING_PUSH_PREFIX(str)
#define DEBUG_MESSAGE_PACKING_POP_PREFIX()
#define DEBUG_MESSAGE_PACKING(cmd)
#if defined(_WIN32) || defined(_WIN64)              //ScenSim-Port://
#define DEBUG_MESSAGE_PACKING_LOG(format, ...)      //ScenSim-Port://
#else                                               //ScenSim-Port://
#define DEBUG_MESSAGE_PACKING_LOG(format, args ...)
#endif                                              //ScenSim-Port://
#define DEBUG_MESSAGE_PACKING_VALUE(format, value, len)
#define DEBUG_MESSAGE_PACKING_SEM(type, sem)

#endif


#endif
