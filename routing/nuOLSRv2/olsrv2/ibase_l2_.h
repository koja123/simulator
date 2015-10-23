//
// This file is automatically generated from tempalte/ibase.h
//

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_l2
 * @{
 */

////////////////////////////////////////////////////////////////
//
// Iterator
//

/** Gets iterator of ibase.
 *
 * @param ibase
 * @return iterator which points the first tuple
 */
#define ibase_l2_iter(ibase) \
    ((tuple_l2_t*)(tuple_l2_t*)(ibase)->next)

/** Checks whether iterator points end of the ibase.
 *
 * @param ibase
 * @param iter
 * @return true if iter points to the end of ibase
 */
#define ibase_l2_iter_is_end(iter, ibase) \
    ((nu_bool_t)((void*)iter == (void*)(ibase)))

/** Gets next iterator.
 *
 * @param ibase
 * @param iter
 * @return iterator which points the next iter
 */
#define ibase_l2_iter_next(iter, ibase) \
    ((tuple_l2_t*)(tuple_l2_t*)(iter)->next)

/** Gets the pointer which points end of the ibase.
 *
 * @param ibase
 * @return iterator which points the end of the ibase
 */
#define ibase_l2_iter_end(ibase)    ((tuple_l2_t*)(ibase))

/** Traverses ibase.
 *
 * @param p
 */

/** Traverses the ibase.
 *
 * @param p
 * @param ibase
 */
#define FOREACH_L2(p, ibase)                   \
    for (tuple_l2_t* p = ibase_l2_iter(ibase); \
         !ibase_l2_iter_is_end(p, ibase);      \
         p = ibase_l2_iter_next(p, ibase))

////////////////////////////////////////////////////////////////
//
// Information Base
//

/** Checks whether the ibase is empty.
 *
 * @param ibase
 * @return true if the ibase is empty
 */
#define ibase_l2_is_empty(ibase)    ((void*)(ibase) == (void*)(ibase)->next)

/** Gets the size of the ibase.
 *
 * @param ibase
 * @return the size of the ibase
 */
#define ibase_l2_size(ibase)        ((ibase)->n)

/** Gets the top tuple of the ibase.
 *
 * @param ibase
 * @return the top tuple of the ibase
 */
#define ibase_l2_head(ibase)        ((ibase_l2_is_empty(ibase)) ? NULL : (ibase)->next)

/** @} */

}//namespace// //ScenSim-Port://