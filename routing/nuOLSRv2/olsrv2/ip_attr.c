#include "config.h"

#include "olsrv2/olsrv2.h"

namespace NuOLSRv2Port { //ScenSim-Port://

static void*
olsrv2_ip_attr_create(const nu_ip_t ip)
{
    olsrv2_ip_attr_t* r = nu_mem_alloc(olsrv2_ip_attr_t);
    r->tuple_r_dest = NULL;
#ifndef NUOLSRV2_NOSTAT           //ScenSim-Port://
    r->nhdpHelloMessageRecvd = 0;
    r->nhdpHelloMessageLost  = 0;
#endif                            //ScenSim-Port://
    return r;
}

static void
olsrv2_ip_attr_free(void* attr)
{
    nu_mem_free(attr);
}

PUBLIC olsrv2_ip_attr_t*
olsrv2_ip_attr_get(const nu_ip_t ip)
{
    return (olsrv2_ip_attr_t*)nu_ip_attr_get(ip);
}

PUBLIC void
olsrv2_ip_attr_init(void)
{
    nu_ip_attr_set_constructor(olsrv2_ip_attr_create);
    nu_ip_attr_set_destructor(olsrv2_ip_attr_free);
}

PUBLIC void
olsrv2_ip_attr_destroy(void)
{
}

}//namespace// //ScenSim-Port://
