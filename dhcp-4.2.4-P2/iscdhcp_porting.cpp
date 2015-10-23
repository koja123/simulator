#include "iscdhcp_porting.h"

namespace IscDhcpPort {

struct context* curctx;

long int random(void)
{
    return GenerateRandomInt(curctx->glue);
}

void receive_packet(unsigned int index, unsigned char *packet, unsigned int len, unsigned int from_port, struct iaddr *from, bool was_unicast)
{
    assert(packet);
    assert(from);
    assert(from->len == 4 || from->len == 16);

    bool found = false;
    unsigned int i = 0;
    struct interface_info *info;

    for (info = interfaces; info; info = info->next) {
        if (i == index) {
            found = true;
            break;
        }
        ++i;
    }
    if (!found) return;

    if (from->len == 4) {
        do_packet(info, (struct dhcp_packet *)packet, len, from_port, *from, &info->hw_address);
    }
    else if (from->len == 16) {
        isc_boolean_t was_unicast_isc = was_unicast ? ISC_TRUE : ISC_FALSE;
        do_packet6(info, (char *)packet, len, from_port, from, was_unicast_isc);
    }
}

}//namespace IscDhcpPort//
