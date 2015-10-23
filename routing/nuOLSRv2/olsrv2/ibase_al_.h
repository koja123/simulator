//
// This file is automatically generated from tempalte/ibase.h
//

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_al
 * @{
 */

/** Current ibase_al. */
#define IBASE_AL    (&OLSR->ibase_al)

////////////////////////////////////////////////////////////////
//
// Iterator
//

/** Gets iterator.
 *
 * @return iterator which points the first tuple
 */
#define  ibase_al_iter() \
    ((tuple_al_t*)(tuple_al_t*)IBASE_AL->next)

/** Checks whether iterator points end of the ibase.
 *
 * @param iter
 * @return true if iter points to the end of ibase
 */
#define ibase_al_iter_is_end(iter) \
    ((nu_bool_t)((void*)(iter) == (void*)IBASE_AL))

/** Gets next iterator.
 *
 * @param iter
 * @return iterator which points the next tuple
 */
#define ibase_al_iter_next(iter) \
    ((tuple_al_t*)(tuple_al_t*)(iter)->next)

/** Gets the pointer which points end of the ibase.
 *
 * @return iterator which points the end of the ibase
 */
#define ibase_al_iter_end()    ((tuple_al_t*)IBASE_AL)

/** Traverses ibase.
 *
 * @param p
 */

/** Traverses the ibase.
 *
 * @param p
 */
#define FOREACH_AL(p)                     \
    for (tuple_al_t* p = ibase_al_iter(); \
         !ibase_al_iter_is_end(p);        \
         p = ibase_al_iter_next(p))

////////////////////////////////////////////////////////////////
//
// Information Base
//

/** Clears the change flag.
 */
#define ibase_al_clear_change_flag() \
    do { IBASE_AL->change = false; } \
    while (0)

/** Checks whether the ibase has been changed.
 */
#define ibase_al_is_changed() \
    ((nu_bool_t)(IBASE_AL->change))

/** Sets the change flag of the ibase.
 */
#define ibase_al_change()           \
    do { IBASE_AL->change = true; } \
    while (0)

/** Checks whether the ibase is empty.
 *
 * @return true if the ibase is empty
 */
#define ibase_al_is_empty()    ((void*)(IBASE_AL) == (void*)IBASE_AL->next)

/** Gets the size of the ibase.
 *
 * @return the size of the ibase
 */
#define ibase_al_size()        ((IBASE_AL)->n)

/** Gets the top tuple of the ibase.
 *
 * @return the top tuple of the ibase
 */
#define ibase_al_head()        ((ibase_al_is_empty()) ? NULL : (IBASE_AL)->next)

/** @} */

}//namespace// //ScenSim-Port://