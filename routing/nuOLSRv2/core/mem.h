#ifndef NU_CORE_MEM_H_
#define NU_CORE_MEM_H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @defgroup nu_mem Core :: Memory allocator
 * @{
 */

/** Allocates memory.
 *
 * @param type
 * @return the pointer
 * @return a pointer to the allocated memory
 */
#define nu_mem_alloc(type) \
    ((type*)nu_do_malloc(sizeof(type), __FILE__, __LINE__))

/** Allocates memory.
 *
 * @param sz
 * @return a pointer to the allocated memory
 */
#define nu_mem_malloc(sz) \
    nu_do_malloc((sz), __FILE__, __LINE__)

/** Allocates memory.
 *
 * @param n
 * @param sz
 * @return a pointer to the allocated memory
 */
#define nu_mem_calloc(n, sz) \
    nu_do_calloc((n), (sz), __FILE__, __LINE__)

/** Reallocates memory.
 *
 * @param p
 * @param sz
 * @return a pointer to the allocated memory
 */
#define nu_mem_realloc(p, sz) \
    nu_do_realloc((p), (sz), __FILE__, __LINE__)

/** Frees the memory.
 *
 * @param p
 */
#define nu_mem_free(p) \
    nu_do_free((p), __FILE__, __LINE__)

/** Duplicates string.
 *
 * @param p
 * @return a pointer to the allocated memory
 */
#define nu_mem_strdup(p) \
    nu_do_strdup((p), __FILE__, __LINE__)

PUBLIC void* nu_do_malloc(size_t sz, const char* file, const int line);
PUBLIC void* nu_do_calloc(size_t count, size_t sz, const char* file,
        const int line);
PUBLIC void* nu_do_realloc(void* p, size_t sz, const char* file,
        const int line);
PUBLIC void nu_do_free(void* p, const char* file, const int line);

PUBLIC char* nu_do_strdup(const char* p, const char* file, const int line);

#ifdef USE_MEM_LEAK_CHECKER
PUBLIC void nu_mem_debug_open(void);
PUBLIC void nu_mem_debug_close(void);
PUBLIC void nu_mem_debug_on(void);
PUBLIC void nu_mem_debug_off(void);
#define STATIC_MEM
#define STATIC_MEM_FREE(x)      x
#else /* USE_MEM_LEAK_CHECKER */
#define nu_mem_debug_open()     ///< disable memory debug code
#define nu_mem_debug_close()    ///< disable memory debug code
#define nu_mem_debug_on()       ///< disable memory debug code
#define nu_mem_debug_off()      ///< disable memory debug code
#define STATIC_MEM    static    ///< disable memory debug code
#define STATIC_MEM_FREE(x)      ///< disable memory debug code
#endif /* USE_MEM_LEAK_CHECKER */

/** @} */

}//namespace// //ScenSim-Port://

#endif
