/***************************************************************************
                          utils.h  -  description
                             -------------------
    begin                : Wed Jul 30 2003
    copyright            : (C) 2003 by Luke Klein-Berndt
    email                : kleinb@nist.gov
 ***************************************************************************/

#ifndef AODV_UTILS_H
#define AODV_UTILS_H

//ScenSim-Port://#include <asm/byteorder.h>
//ScenSim-Port://#include <linux/module.h>
//ScenSim-Port://#include <linux/kernel.h>
//ScenSim-Port://#include <linux/time.h>
//ScenSim-Port://#include <linux/ctype.h>
//ScenSim-Port://#include <asm/div64.h>

//#include "aodv.h"


namespace KernelAodvPort {

//ScenSim-Port://u_int64_t getcurrtime();
//ScenSim-Port://char *inet_ntoa(unsigned int ina)
//ScenSim-Port://{
//ScenSim-Port://    static char buf[4 * sizeof "123"];
//ScenSim-Port://    unsigned char *ucp = (unsigned char *) &ina;
//ScenSim-Port://    sprintf(buf, "%d.%d.%d.%d", ucp[0] & 0xff, ucp[1] & 0xff, ucp[2] & 0xff, ucp[3] & 0xff);
//ScenSim-Port://    return buf;
//ScenSim-Port://}
//ScenSim-Port://int inet_aton(const char *cp, u_int32_t * addr);
//ScenSim-Port://u_int32_t calculate_netmask(int t);
//ScenSim-Port://int calculate_prefix(u_int32_t t);
//ScenSim-Port://int seq_greater(u_int32_t seq_one,u_int32_t seq_two);

int seq_less_or_equal(unsigned int seq_one, unsigned int seq_two);
int seq_greater(unsigned int seq_one, unsigned int seq_two);


}//namespace//

#endif
