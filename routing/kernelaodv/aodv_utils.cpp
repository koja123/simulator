/***************************************************************************
                          utils.c  -  description
                             -------------------
    begin                : Wed Jul 30 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#include "aodv_utils.h"


namespace KernelAodvPort {

//ScenSim-Port://u_int64_t getcurrtime()
//ScenSim-Port://{
//ScenSim-Port://    struct timeval tv;
//ScenSim-Port://    u_int64_t result;
//ScenSim-Port://
//ScenSim-Port://    do_gettimeofday(&tv);
//ScenSim-Port://
//ScenSim-Port://    //This is a fix for an error that occurs on ARM Linux Kernels because they do 64bits differently
//ScenSim-Port://    //Thanks to S. Peter Li for coming up with this fix!
//ScenSim-Port://
//ScenSim-Port://    result = (u_int64_t) tv.tv_usec;
//ScenSim-Port://    do_div(result, 1000);
//ScenSim-Port://    return ((u_int64_t) tv.tv_sec) * 1000 + result;
//ScenSim-Port://}


//ScenSim-Port://char *inet_ntoa(u_int32_t ina)
//ScenSim-Port://{
//ScenSim-Port://    static char buf[4 * sizeof "123"];
//ScenSim-Port://    unsigned char *ucp = (unsigned char *) &ina;
//ScenSim-Port://    sprintf(buf, "%d.%d.%d.%d", ucp[0] & 0xff, ucp[1] & 0xff, ucp[2] & 0xff, ucp[3] & 0xff);
//ScenSim-Port://    return buf;
//ScenSim-Port://}

//ScenSim-Port://int inet_aton(const char *cp, u_int32_t * addr)
//ScenSim-Port://{
//ScenSim-Port://    unsigned int val;
//ScenSim-Port://    int base, n;
//ScenSim-Port://    char c;
//ScenSim-Port://    u_int parts[4];
//ScenSim-Port://    u_int *pp = parts;
//ScenSim-Port://
//ScenSim-Port://    for (;;)
//ScenSim-Port://    {
//ScenSim-Port://
//ScenSim-Port://        //Collect number up to ``.''. Values are specified as for C:
//ScenSim-Port://        // 0x=hex, 0=octal, other=decimal.
//ScenSim-Port://
//ScenSim-Port://        val = 0;
//ScenSim-Port://        base = 10;
//ScenSim-Port://        if (*cp == '0')
//ScenSim-Port://        {
//ScenSim-Port://            if (*++cp == 'x' || *cp == 'X')
//ScenSim-Port://                base = 16, cp++;
//ScenSim-Port://            else
//ScenSim-Port://                base = 8;
//ScenSim-Port://        }
//ScenSim-Port://        while ((c = *cp) != '\0')
//ScenSim-Port://        {
//ScenSim-Port://            if (isascii(c) && isdigit(c))
//ScenSim-Port://            {
//ScenSim-Port://                val = (val * base) + (c - '0');
//ScenSim-Port://                cp++;
//ScenSim-Port://                continue;
//ScenSim-Port://
//ScenSim-Port://            }
//ScenSim-Port://            if (base == 16 && isascii(c) && isxdigit(c))
//ScenSim-Port://            {
//ScenSim-Port://                val = (val << 4) + (c + 10 - (islower(c) ? 'a' : 'A'));
//ScenSim-Port://                cp++;
//ScenSim-Port://                continue;
//ScenSim-Port://            }
//ScenSim-Port://            break;
//ScenSim-Port://        }
//ScenSim-Port://        if (*cp == '.')
//ScenSim-Port://        {
//ScenSim-Port://            // Internet format: a.b.c.d a.b.c       (with c treated as
//ScenSim-Port://            // 16-bits) a.b         (with b treated as 24 bits)
//ScenSim-Port://
//ScenSim-Port://            if (pp >= parts + 3 || val > 0xff)
//ScenSim-Port://                return (0);
//ScenSim-Port://            *pp++ = val, cp++;
//ScenSim-Port://        } else
//ScenSim-Port://            break;
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    // Check for trailing characters.
//ScenSim-Port://
//ScenSim-Port://    if (*cp && (!isascii(*cp) || !isspace(*cp)))
//ScenSim-Port://        return (0);
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://    // Concoct the address according to the number of parts specified.
//ScenSim-Port://
//ScenSim-Port://    n = pp - parts + 1;
//ScenSim-Port://    switch (n)
//ScenSim-Port://    {
//ScenSim-Port://
//ScenSim-Port://    case 1:                    // a -- 32 bits
//ScenSim-Port://        break;
//ScenSim-Port://
//ScenSim-Port://    case 2:                    //a.b -- 8.24 bits
//ScenSim-Port://        if (val > 0xffffff)
//ScenSim-Port://            return (0);
//ScenSim-Port://        val |= parts[0] << 24;
//ScenSim-Port://        break;
//ScenSim-Port://
//ScenSim-Port://    case 3:                    //a.b.c -- 8.8.16 bits
//ScenSim-Port://        if (val > 0xffff)
//ScenSim-Port://            return (0);
//ScenSim-Port://        val |= (parts[0] << 24) | (parts[1] << 16);
//ScenSim-Port://        break;
//ScenSim-Port://
//ScenSim-Port://    case 4:                    // a.b.c.d -- 8.8.8.8 bits
//ScenSim-Port://        if (val > 0xff)
//ScenSim-Port://            return (0);
//ScenSim-Port://        val |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
//ScenSim-Port://        break;
//ScenSim-Port://    }
//ScenSim-Port://    if (addr)
//ScenSim-Port://        *addr = htonl(val);
//ScenSim-Port://    return (1);
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://int local_subnet_test(u_int32_t tmp_ip)
//ScenSim-Port://{
//ScenSim-Port:////printk("Comparing route: %s ",inet_ntoa(tmp_route->ip & tmp_route->netmask));
//ScenSim-Port:////printk(" to ip: %s\n", inet_ntoa(target_ip & tmp_route->netmask));
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://
//ScenSim-Port://   /* if ((tmp_dev->ip & tmp_dev->netmask)  == (tmp_dev->netmask & tmp_ip ))
//ScenSim-Port://        return 1;
//ScenSim-Port://*/
//ScenSim-Port:////                 printk("Comparing route: %s ",inet_ntoa(tmp_dev->ip & tmp_dev->netmask));
//ScenSim-Port:////     printk(" to ip: %s\n", inet_ntoa(tmp_ip & tmp_dev->netmask));
//ScenSim-Port://
//ScenSim-Port://  return 0;
//ScenSim-Port://}
//ScenSim-Port://
//ScenSim-Port://

//This function is not needed for Scenargie. Restricted subnetmask is needed.
//ScenSim-Port://int aodv_subnet_test(u_int32_t tmp_ip)
//ScenSim-Port://{
//ScenSim-Port://    aodv_dev *tmp_dev;
//ScenSim-Port:////printk("Comparing route: %s ",inet_ntoa(tmp_route->ip & tmp_route->netmask));
//ScenSim-Port:////printk(" to ip: %s\n", inet_ntoa(target_ip & tmp_route->netmask));
//ScenSim-Port://
//ScenSim-Port://    tmp_dev = first_aodv_dev();
//ScenSim-Port:// 
//ScenSim-Port://    while (tmp_dev != NULL) {
//ScenSim-Port://
//ScenSim-Port://        if ((tmp_dev->ip & tmp_dev->netmask) == (tmp_dev->netmask & tmp_ip )) {
//ScenSim-Port://            return 1;
//ScenSim-Port://        }
//ScenSim-Port://
//ScenSim-Port://        tmp_dev = tmp_dev->next;
//ScenSim-Port://    }
//ScenSim-Port:////                 printk("Comparing route: %s ",inet_ntoa(tmp_dev->ip & tmp_dev->netmask));
//ScenSim-Port:////     printk(" to ip: %s\n", inet_ntoa(tmp_ip & tmp_dev->netmask));
//ScenSim-Port://
//ScenSim-Port://    return 0;
//ScenSim-Port://}

//ScenSim-Port://u_int32_t calculate_netmask(int t)
//ScenSim-Port://{
//ScenSim-Port://    int i;
//ScenSim-Port://    uint32_t final = 0;
//ScenSim-Port://
//ScenSim-Port://    for (i = 0; i < 32 - t; i++)
//ScenSim-Port://    {
//ScenSim-Port://        final = final + 1;
//ScenSim-Port://        final = final << 1;
//ScenSim-Port://    }
//ScenSim-Port://
//ScenSim-Port://    final = final + 1;
//ScenSim-Port://    final = ~final;
//ScenSim-Port://    return final;
//ScenSim-Port://
//ScenSim-Port://}

//ScenSim-Port://int calculate_prefix(u_int32_t t)
//ScenSim-Port://{
//ScenSim-Port://    int i = 1;
//ScenSim-Port://
//ScenSim-Port://    while (t != 0)
//ScenSim-Port://    {
//ScenSim-Port://        t = t << 1;
//ScenSim-Port://        i++;
//ScenSim-Port://    }
//ScenSim-Port://    return i;
//ScenSim-Port://}

int seq_less_or_equal(unsigned int seq_one,unsigned int seq_two)
{
    //ScenSim-Port://int *comp_seq_one = &seq_one;
    //ScenSim-Port://int *comp_seq_two = &seq_two;
    int *comp_seq_one = (int*)&seq_one;
    int *comp_seq_two = (int*)&seq_two;

    if (  ( *comp_seq_one - *comp_seq_two ) > 0 )
    {
        return 0;
    }
    else
        return 1;
}

int seq_greater(unsigned int seq_one,unsigned int seq_two)
{
    //ScenSim-Port://int *comp_seq_one = &seq_one;
    //ScenSim-Port://int *comp_seq_two = &seq_two;
    int *comp_seq_one = (int*)&seq_one;
    int *comp_seq_two = (int*)&seq_two;

    if (  ( *comp_seq_one - *comp_seq_two ) < 0 )
        return 0;
    else
        return 1;
}

}//namespace//
