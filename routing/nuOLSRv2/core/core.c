//
// Mode core module
//
#include "config.h"

#include <stddef.h>
#include <time.h>

#include "core/core.h"
#include "core/scheduler.h"
#include "core/mem.h"

#if defined(_WIN32) || defined(_WIN64)   //ScenSim-Port://
#define strcasecmp(s1, s2) _stricmp(s1, s2) //ScenSim-Port://
#endif                                      //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_core
 * @{
 */

#ifndef nuOLSRv2_ALL_STATIC
nu_core_t* current_core = NULL;
#endif

/**
 * Log flags
 */
/* *INDENT-OFF* */
static nu_log_flag_t nu_core_log_flags[] =
{
    { "ip_heap",   "Ci", offsetof(nu_core_t, log_ip_heap),
      "IP heap statistics" },
    { "scheduler", "Cs", offsetof(nu_core_t, log_scheduler),
      "scheduler events" },
    { NULL },
};
/* *INDENT-ON* */

/** Allocates and initializes core module.
 *
 * @return object
 */
PUBLIC nu_core_t*
nu_core_create(void)
{
    current_core = nu_mem_alloc(nu_core_t);
    nu_logger_init(&current_core->logger);
    nu_ip_heap_init(&current_core->ip_heap);
    nu_set_af(AF_INET);
    nu_time_set_zero(&current_core->now);
    nu_scheduler_init();

    current_core->ip_attr_constructor = NULL;
    current_core->ip_attr_destructor  = NULL;

    nu_init_log_flags(nu_core_log_flags, current_core);

    return current_core;
}

/** Duplicates core module.
 *
 * @param src
 * @return copied object.
 */
PUBLIC nu_core_t*
nu_core_dup(const nu_core_t* src)
{
    current_core = nu_mem_alloc(nu_core_t);
    memcpy(current_core, src, sizeof(nu_core_t));
    nu_logger_init(&current_core->logger);
    nu_ip_heap_init(&current_core->ip_heap);
    return current_core;
}

/** Destroys and frees the core module object.
 *
 * @param self
 */
PUBLIC void
nu_core_free(nu_core_t* self)
{
    nu_scheduler_shutdown();
    nu_ip_heap_destroy(&self->ip_heap);
    nu_logger_destroy(&self->logger);
    memset(self, sizeof(nu_core_t), 0xfe);
    nu_mem_free(self);
}

/** Outputs usage.
 *
 * @param prefix
 * @param fp
 */
PUBLIC void
nu_core_put_params(const char* prefix, FILE* fp)
{
    fprintf(fp, "%s-%-8s ... more verbose log\n", prefix, "v");
    fprintf(fp, "%s-%-8s ... more quiet log\n", prefix, "q");
    nu_put_log_flags(prefix, nu_core_log_flags, fp);
}

/** Sets the option of the core module.
 *
 * @param opt	one option string
 * @return true if given option string is used in the core module,
 *	false otherwise.
 */
PUBLIC nu_bool_t
nu_core_set_param(const char* opt)
{
    if (strcmp(opt, "-v") == 0) {
        nu_logger_set_more_verbose(&current_core->logger);
        return true;
    }
    if (strcmp(opt, "-q") == 0) {
        nu_logger_set_more_quiet(&current_core->logger);
        return true;
    }
    return nu_set_log_flag(opt, nu_core_log_flags, current_core);
}

////////////////////////////////////////////////////////////////
//
// Utility functions
//

/** Sets log flags.
 *
 * @param name
 *		log flag name
 * @param flags
 *		log flag table
 * @param module
 *		target module
 * @return true if given log flag is used in the given module.
 */
PUBLIC nu_bool_t
nu_set_log_flag(const char* name, nu_log_flag_t* flags, void* module)
{
    const char* n = (name[0] == '-') ? (name + 1) : name;
    uintptr_t   base = (uintptr_t)module;
    for (nu_log_flag_t* p = flags; p->name != NULL; ++p) {
        if (strcasecmp(n, p->name) == 0 || strcmp(n, p->short_name) == 0) {
            (*(nu_bool_t*)(base + p->offset)) = true;
            return true;
        }
    }
    return false;
}

/** Outputs log flags.
 *
 * @param prefix
 * @param flags
 * @param fp
 */
PUBLIC void
nu_put_log_flags(const char* prefix, nu_log_flag_t* flags, FILE* fp)
{
    for (nu_log_flag_t* p = flags; p->name != NULL; ++p) {
        fprintf(fp, "%s-%-8s ... [%s] %s\n",
                prefix, p->short_name, p->name, p->memo);
    }
}

/** Initialize logs flags.
 *
 * @param flags		log flag table
 * @param module	target module object
 */
PUBLIC void
nu_init_log_flags(nu_log_flag_t* flags, void* module)
{
    uintptr_t base = (uintptr_t)module;
    for (nu_log_flag_t* p = flags; p->name != NULL; ++p) {
        (*(nu_bool_t*)(base + p->offset)) = false;
    }
}

/** @} */

}//namespace// //ScenSim-Port://
