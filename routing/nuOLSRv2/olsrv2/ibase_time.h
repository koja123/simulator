//
// Tuple timeout operations
//
#ifndef OLSRV2_IBASE_TIMEOUT_H_
#define OLSRV2_IBASE_TIMEOUT_H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup ibase_time OLSRv2 :: Tuple timeout list
 * @{
 */
struct tuple_time;

/**
 * Timeout processor
 */
typedef void (*tuple_time_proc_t)(struct tuple_time*);

/**
 * Tuple of ibase_time
 */
typedef struct tuple_time {
    void*              tuple_next;    ///< next tuple in ibase
    void*              tuple_prev;    ///< prev tuple in ibase

    struct tuple_time* next;          ///< next tuple in timeout list
    struct tuple_time* prev;          ///< prev tuple in timeout list
    tuple_time_proc_t  timeout_proc;  ///< timeout processor
    nu_time_t          time;          ///< timeout time
} tuple_time_t;

/**
 * IBase_time
 */
typedef struct ibase_time {
    void*         ibase_next;   ///< first tuple in ibase
    void*         ibase_prev;   ///< last tuple in ibase
    tuple_time_t* next;         ///< first tuple in timeout_list
    tuple_time_t* prev;         ///< last tuple in timeout_list
    void*         proc;         ///< (dummy)
    size_t        n;            ///< size
} ibase_time_t;

////////////////////////////////////////////////////////////////
//
// tuple_time_t
//

/** Destroys and frees the tuple */
#define tuple_time_free(tuple)    /* DO NOTHING */

PUBLIC void tuple_time_init(tuple_time_t*, ibase_time_t*, double);
PUBLIC void tuple_time_set_timeout(tuple_time_t*, ibase_time_t*, double);
PUBLIC void tuple_time_set_time(tuple_time_t*, ibase_time_t*, const nu_time_t);

////////////////////////////////////////////////////////////////
//
// ibase_time_t
//

PUBLIC void ibase_time_init(ibase_time_t*);
PUBLIC void ibase_time_cancel(ibase_time_t*, tuple_time_t*);
PUBLIC void ibase_time_remove(ibase_time_t*);

/** @} */

}//namespace// //ScenSim-Port://

#endif
