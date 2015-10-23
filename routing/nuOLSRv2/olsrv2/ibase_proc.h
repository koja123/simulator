//
// Received Message Information Base
// (RX)	 Received Set
// (P)	 Processed Set
// (F)	 Forwarded Set
//
#ifndef OLSRV2_IBASE_PROC_H_
#define OLSRV2_IBASE_PROC_H_

#include "packet/msg.h"

#define IBASE_PROC_TABLE_SIZE    (32)

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_proc OLSRv2 :: (RX) (P) (F) Received Message Information Base
 * @{
 */

struct tuple_proc_list;

/**
 * Tuple for received message
 */
typedef struct tuple_proc {
    struct tuple_proc*      next;         ///< next tuple
    struct tuple_proc*      prev;         ///< prev tuple

    tuple_time_t*           timeout_next; ///< next in timeout list
    tuple_time_t*           timeout_prev; ///< prev in timeout list
    tuple_time_proc_t       timeout_proc; ///< timeout processor
    nu_time_t               time;         ///< timeout time
    struct tuple_proc_list* ibase;        ///< ibase

    uint8_t                 type;         ///< type
    nu_ip_t                 ip;           ///< ip
    uint16_t                seqnum;       ///< sequence number
} tuple_proc_t;

/**
 * The list of tuple_proc_t
 */
typedef struct tuple_proc_list {
    tuple_proc_t* next;      ///< first tuple
    tuple_proc_t* prev;      ///< last tuple
    size_t        n;         ///< size
    ibase_time_t* time_list; ///< timeout list
} tuple_proc_list_t;

/**
 * Information base for received message
 */
typedef struct ibase_proc {
    tuple_proc_list_t table[IBASE_PROC_TABLE_SIZE]; ///< hash table
} ibase_proc_t;

////////////////////////////////////////////////////////////////
//
// ibase_proc_t
//

PUBLIC void ibase_proc_init(ibase_proc_t*);
PUBLIC void ibase_proc_destroy(ibase_proc_t*);

PUBLIC size_t ibase_proc_size(const ibase_proc_t*);

PUBLIC void ibase_proc_add(ibase_proc_t*, const nu_msg_t*, const double);
PUBLIC nu_bool_t ibase_proc_contain(const ibase_proc_t*, const nu_msg_t*);

PUBLIC void ibase_proc_put_log(const ibase_proc_t*,
        nu_logger_t*, const char*);

////////////////////////////////////////////////////////////////
//
// (RX) Received Message Set
//

/**
 * (RX) Received Message Set
 */
typedef ibase_proc_t   ibase_rx_t;

/** Initializes the ibase_rx.
 *
 * @param ibase_rx
 */
#define ibase_rx_init(ibase_rx) \
    ibase_proc_init(ibase_rx)

/** Destroys the ibase_rx.
 *
 * @param ibase_rx
 */
#define ibase_rx_destroy(ibase_rx) \
    ibase_proc_destroy(ibase_rx)

/** Gets the size of the ibase_rx.
 *
 * @param ibase_rx
 */
#define ibase_rx_size(ibase_rx) \
    ibase_proc_size(ibase_rx)

/** Adds the message information to the ibase_rx.
 *
 * @param ibase_rx
 * @param msg
 */
#define ibase_rx_add(ibase_rx, msg) \
    ibase_proc_add(ibase_rx, msg, RX_HOLD_TIME)

/** Checks whether the ibase_rx contains the message information.
 *
 * @param ibase_rx
 * @param msg
 * @return true if the ibase_rx contains the message information.
 */
#define ibase_rx_contain(ibase_rx, msg) \
    ibase_proc_contain(ibase_rx, msg)

/** Outputs log
 *
 * @param ibase_rx
 * @param logger
 */
#define ibase_rx_put_log(ibase_rx, logger) \
    ibase_proc_put_log(ibase_rx, logger, "I_RX")

////////////////////////////////////////////////////////////////
//
// (P) Processed Set
//

/**
 * (P) Processed Set
 */
typedef ibase_proc_t   ibase_p_t;

/** Current ibase_p. */
#define IBASE_P    (&OLSR->ibase_p)

/** Initializes the ibase_p.
 */
#define ibase_p_init()             ibase_proc_init(IBASE_P)

/** Destroys the ibase_p.
 */
#define ibase_p_destroy()          ibase_proc_destroy(IBASE_P)

/** Gets the size of the ibase_p.
 */
#define ibase_p_size()             ibase_proc_size(IBASE_P)

/** Adds the message information to the ibase_p.
 *
 * @param msg
 */
#define ibase_p_add(msg)           ibase_proc_add(IBASE_P, msg, P_HOLD_TIME)

/** Checks whether the ibase_p contains the message information.
 *
 * @param msg
 * @return true if the ibase_p contains the message information.
 */
#define ibase_p_contain(msg)       ibase_proc_contain(IBASE_P, msg)

/** Outputs log.
 *
 * @param logger
 */
#define ibase_p_put_log(logger)    ibase_proc_put_log(IBASE_P, logger, "I_P")

////////////////////////////////////////////////////////////////
//
// (F) Forwarded Set
//

/**
 * (F) Forwarded Set
 */
typedef ibase_proc_t   ibase_f_t;

/** Current ibase_f. */
#define IBASE_F    (&OLSR->ibase_f)

/** Initializes the ibase_f.
 */
#define ibase_f_init()             ibase_proc_init(IBASE_F)

/** Destroys the ibase_f.
 */
#define ibase_f_destroy()          ibase_proc_destroy(IBASE_F)

/** Gets the size of the ibase_f.
 *
 * @return the size of the ibase_f.
 */
#define ibase_f_size()             ibase_proc_size(IBASE_F)

/** Adds the message information to the ibase_f.
 *
 * @param msg
 */
#define ibase_f_add(msg)           ibase_proc_add(IBASE_F, msg, F_HOLD_TIME)

/** Checks whether the ibase_f contains the message information.
 *
 * @param msg
 * @return true if the ibase_f contains the message information.
 */
#define ibase_f_contain(msg)       ibase_proc_contain(IBASE_F, msg)

/** Outputs log.
 *
 * @param logger
 */
#define ibase_f_put_log(logger)    ibase_proc_put_log(IBASE_F, logger, "I_F")

/** @} */

}//namespace// //ScenSim-Port://

#endif
