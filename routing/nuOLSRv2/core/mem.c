#include "config.h"

#include "core/core.h"
#include "mem.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/****************************************************************
 * @addtogroup nu_mem
 * @{
 */

#ifdef USE_MEM_LEAK_CHECKER

typedef struct mem_chunk {
    void* ptr;
    char* name;
} mem_chunk_t;

static inline mem_chunk_t*
mem_chunk_create(void* ptr, void* name)
{
    mem_chunk_t* mem = (mem_chunk_t*)malloc(sizeof(mem_chunk_t));
    mem->ptr  = ptr;
    mem->name = (char*)malloc(strlen((const char*)name) + 1);
    memcpy((void*)mem->name, name, strlen((const char*)name) + 1);
    return mem;
}

static inline void
mem_chunk_free(mem_chunk_t** memp)
{
    free((*memp)->name);
    free(*memp);
    *memp = NULL;
}

static inline size_t
nu_mem_hash(const mem_chunk_t* mem)
{
    return (size_t)((uintptr_t)mem->ptr % 0xffff);
}

static inline int
nu_mem_eq(const mem_chunk_t* m1, const mem_chunk_t* m2)
{
    return m1->ptr == m2->ptr;
}

}//namespace// //ScenSim-Port://

#include "mem_list.h"
#include "mem_set.h"

namespace NuOLSRv2Port { //ScenSim-Port://

static nu_bool_t     mem_debug_on = false;
static nu_mem_set_t* mem_set = NULL;

/** Enables memory debug mode.
 */
PUBLIC void
nu_mem_debug_open(void)
{
    mem_set = nu_mem_set_create();
}

/** Disables memory debug mode.
 */
PUBLIC void
nu_mem_debug_close(void)
{
    nu_mem_set_free(mem_set);
    mem_set = NULL;
}

/** Start memory debug.
 */
PUBLIC void
nu_mem_debug_on(void)
{
    mem_debug_on = true;
}

/** Stop memory debug.
 */
PUBLIC void
nu_mem_debug_off(void)
{
    if (mem_set) {
        FOREACH_MEM_SET(p, mem_set) {
            mem_chunk_t* mem = nu_mem_set_iter_elt(p);
            nu_err("MEM_LEAK %lx %s", mem->ptr, mem->name);
        }
    }
    mem_debug_on = false;
}

#endif /* USE_MEM_LEAK_CHECKER */

/** Allocates memory (USE nu_mem_malloc() or nu_mem_alloc()
 * INSTEAD OF THIS FUNCTION).
 *
 * @param size
 * @param file
 * @param line
 * @return pointer to allocated memory
 */
PUBLIC void*
nu_do_malloc(size_t size, const char* file, const int line)
{
    void* ptr = malloc(size);
    if (!ptr)
        nu_fatal("Cannot allocate memory:%s:%d", file, line);
#ifdef USE_MEM_LEAK_CHECKER
    if (mem_debug_on) {
        if (mem_set) {
            char buf[64];
            snprintf(buf, sizeof(buf), "%s:%d", file, line);
            nu_mem_set_add(mem_set, mem_chunk_create(ptr, buf));
        }
    }
#endif /* USE_MEM_LEAK_CHECKER */
    return ptr;
}

/** Allocates memory (USE nu_mem_calloc() INSTEAD OF THIS FUNCTION).
 *
 * @param count
 * @param size
 * @param file
 * @param line
 * @return pointer to allocated memory
 */
PUBLIC void*
nu_do_calloc(size_t count, size_t size, const char* file, const int line)
{
    void* ptr = calloc(count, size);
    if (!ptr)
        nu_fatal("Cannot allocate memory:%s:%d", file, line);
#ifdef USE_MEM_LEAK_CHECKER
    if (mem_debug_on) {
        if (mem_set) {
            char buf[64];
            snprintf(buf, sizeof(buf), "%s:%d", file, line);
            nu_mem_set_add(mem_set, mem_chunk_create(ptr, buf));
        }
    }
#endif /* USE_MEM_LEAK_CHECKER */
    return ptr;
}

/** Allocates memory (USE nu_mem_realloc() INSTEAD OF THIS FUNCTION).
 *
 * @param ptr
 * @param size
 * @param file
 * @param line
 * @return pointer to allocated memory
 */
PUBLIC void*
nu_do_realloc(void* ptr, size_t size, const char* file, const int line)
{
#ifdef USE_MEM_LEAK_CHECKER
    void* optr = ptr;
#endif
    ptr = realloc(ptr, size);
    if (!ptr)
        nu_err("Cannot allocate memory:%s:%d", file, line);
#ifdef USE_MEM_LEAK_CHECKER
    if (mem_debug_on) {
        if (mem_set) {
            mem_chunk_t target;
            char buf[64];
            snprintf(buf, sizeof(buf), "%s:%d", file, line);

            if (optr) {
                target.ptr = optr;
                if (!nu_mem_set_delete(mem_set, &target))
                    nu_fatal("R:fail to delete %lx\n", optr);
            }
            nu_mem_set_add(mem_set, mem_chunk_create(ptr, buf));
        }
    }
#endif /* USE_MEM_LEAK_CHECKER */
    return ptr;
}

/** Frees the memory. (USE nu_mem_free() INSTEAD OF THIS FUNCTION).
 *
 * @param p
 * @param file
 * @param line
 * @return pointer to allocated memory
 */
PUBLIC void
nu_do_free(void* p, const char* file, const int line)
{
#ifdef USE_MEM_LEAK_CHECKER
    if (mem_debug_on) {
        if (mem_set) {
            mem_chunk_t target;
            target.ptr = p;
            if (!nu_mem_set_delete(mem_set, &target)) {
                FOREACH_MEM_SET(i, mem_set) {
                    mem_chunk_t* mem = nu_mem_set_iter_elt(i);
                    nu_err("mem:%lx %s", mem->ptr, mem->name);
                }
                nu_fatal("R:fail to delete 0x%lx\n", p);
            }
        }
    }
#endif /* USE_MEM_LEAK_CHECKER */
    free(p);
}

/** Allocates memory and copy the specified string (USE nu_mem_strdup()
 * INSTEAD OF THIS FUNCTION).
 *
 * @param p
 * @param file
 * @param line
 * @return pointer to allocated memory
 */
PUBLIC char*
nu_do_strdup(const char* p, const char* file, const int line)
{
    const size_t len = strlen(p) + 1;
    char* r = (char*)nu_do_malloc(len, file, line);
    strncpy(r, p, len);
    return r;
}

/** @} */

}//namespace// //ScenSim-Port://
