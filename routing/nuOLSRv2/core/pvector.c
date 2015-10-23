#include "config.h"

#include <assert.h>

#include "core/mem.h"
#include "core/pvector.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_pvector
 * @{
 */

/*
 */
static void
grow(nu_pvector_t* self)
{
    size_t new_capa = self->capa + self->grow;
    self->data = (void**)nu_mem_realloc(self->data,
            new_capa * sizeof(void*));
    self->capa = new_capa;
}

/** Initializes the pvector.
 *
 * @param self
 * @param capa
 * @param grow
 */
PUBLIC void
nu_pvector_init(nu_pvector_t* self, size_t capa, size_t grow)
{
    self->capa = capa;
    self->grow = grow;
    self->size = 0;
    self->data = (void**)nu_mem_malloc(self->capa * sizeof(void*));
}

/** Destroys the pvector.
 *
 * @param self
 */
PUBLIC void
nu_pvector_destroy(nu_pvector_t* self)
{
    nu_mem_free(self->data);
}

/** Add new pointer to the pvector.
 *
 * @param self
 * @param p
 */
PUBLIC void
nu_pvector_add(nu_pvector_t* self, void* p)
{
    while (self->size + 1 >= self->capa)
        grow(self);
    self->data[self->size++] = p;
}

/** @} */

}//namespace// //ScenSim-Port://
