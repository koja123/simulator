#ifndef NU_CONFIG_H_
#define NU_CONFIG_H_

#define nuOLSRv2_SIMULATOR //ScenSim-Port://
#define NU_NDEBUG          //ScenSim-Port://
#define NUOLSRV2_NOSTAT    //ScenSim-Port://

#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
#pragma warning(disable:4200)          //ScenSim-Port://
#endif                                 //ScenSim-Port://

#include <assert.h>
#include <stdio.h>
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
#include <stdint.h>            //ScenSim-Port://
#include "sysstuff.h"                  //ScenSim-Port://
#else                                  //ScenSim-Port://
#include <stdint.h>
#endif                                 //ScenSim-Port://
#include <stdlib.h>
#include <string.h>
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
#include <winsock2.h>                  //ScenSim-Port://
#include <ws2tcpip.h>                  //ScenSim-Port://
#else                                  //ScenSim-Port://
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/time.h>
#endif                                 //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

typedef char   nu_bool_t;
#define true     1
#define false    0

#define PUBLIC
#define PRIVATE
#define PUBLIC_INLINE     static inline
#define PRIVATE_INLINE    static inline
#define EXTERN            extern

#define nu_ntohl(x)    ntohl(x)
#define nu_htonl(x)    htonl(x)
#define nu_htons(x)    htons(x)
#define nu_ntohs(x)    ntohs(x)

#if 0                                                    //ScenSim-Port://
#define nu_rand()      drand48()
#else                                                    //ScenSim-Port://
extern double NuOLSRv2ProtocolGenerateRandomDouble();    //ScenSim-Port://
#define nu_rand() NuOLSRv2ProtocolGenerateRandomDouble() //ScenSim-Port://
#endif                                                   //ScenSim-Port://

}//namespace// //ScenSim-Port://

#endif
