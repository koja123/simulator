#include "config.h"

#include <assert.h>

#include "core/mem.h"
#include "core/ip.h"
#include "core/ip_vector.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_ip_vector
 * @{
 */

/** Default capacity */
#define DEFAULT_CAPA    2
/** Default growing size */
#define DEFAULT_GROW    8

/*
 */
static void
grow(nu_ip_vector_t* self)
{
    size_t new_capa = self->capa + self->grow;
    self->data = (nu_ip_t*)nu_mem_realloc(self->data,
            new_capa * sizeof(nu_ip_t));
    self->capa = new_capa;
}

/** Initializes the ip_vector.
 *
 * @param self
 */
PUBLIC void
nu_ip_vector_init(nu_ip_vector_t* self)
{
    self->capa = DEFAULT_CAPA;
    self->grow = DEFAULT_GROW;
    self->size = 0;
    self->data = (nu_ip_t*)nu_mem_malloc(self->capa * sizeof(nu_ip_t));
}

/** Destroys the ip_vector.
 *
 * @param self
 */
PUBLIC void
nu_ip_vector_destroy(nu_ip_vector_t* self)
{
    nu_mem_free(self->data);
}

/** Add new pointer to the ip_vector.
 *
 * @param self
 * @param p
 */
PUBLIC void
nu_ip_vector_add(nu_ip_vector_t* self, const nu_ip_t p)
{
    while (self->size + 1 >= self->capa)
        grow(self);
    nu_ip_copy(&self->data[self->size++], &p);
}

/** @} */

}//namespace// //ScenSim-Port://
