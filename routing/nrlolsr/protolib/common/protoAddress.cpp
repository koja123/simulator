/*********************************************************************
 *
 * AUTHORIZATION TO USE AND DISTRIBUTE
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that: 
 *
 * (1) source code distributions retain this paragraph in its entirety, 
 *  
 * (2) distributions including binary code include this paragraph in
 *     its entirety in the documentation or other materials provided 
 *     with the distribution, and 
 *
 * (3) all advertising materials mentioning features or use of this 
 *     software display the following acknowledgment:
 * 
 *      "This product includes software written and developed 
 *       by Code 5520 of the Naval Research Laboratory (NRL)." 
 *         
 *  The name of NRL, the name(s) of NRL  employee(s), or any entity
 *  of the United States Government may not be used to endorse or
 *  promote  products derived from this software, nor does the 
 *  inclusion of the NRL written and developed software  directly or
 *  indirectly suggest NRL or United States  Government endorsement
 *  of this product.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 ********************************************************************/

#include "protoAddress.h"
#include "protoSocket.h"  // for ProtoSocket::GetInterfaceAddress() routines
#include "protoDebug.h"   // for print out of warnings, etc

#ifdef UNIX
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netdb.h>
#include <unistd.h>     // for gethostname()
#ifdef SOLARIS
#include <sys/sockio.h> // for SIOCGIFADDR ioctl
#endif  // SOLARIS
#endif // UNIX

#include <stdlib.h>  // for atoi()
#include <stdio.h>   // for sprintf()
#include <string.h>  // for memset()

namespace NrlProtolibPort //ScenSim-Port://
{ //ScenSim-Port://


const ProtoAddress PROTO_ADDR_NONE;

ProtoAddress::ProtoAddress()
 : type(INVALID), length(0)
{
    memset(&addr, 0, sizeof(addr));
}

ProtoAddress::~ProtoAddress()
{
}

bool ProtoAddress::IsMulticast() const
{
    switch(type)
    {
        case IPv4:
        {
            struct in_addr inAddr = ((struct sockaddr_in*)&addr)->sin_addr;
            return (((UINT32)(htonl(0xf0000000) & inAddr.s_addr))
                        == htonl(0xe0000000));
        }

#ifdef HAVE_IPV6
        case IPv6:
        {
            if (IN6_IS_ADDR_V4MAPPED(&(((struct sockaddr_in6*)&addr)->sin6_addr)))
            {
                return (htonl(0xe0000000) ==
                        ((UINT32)(htonl(0xf0000000) &
                         IN6_V4MAPPED_ADDR(&(((struct sockaddr_in6*)&addr)->sin6_addr)))));
            }
            else
            {
                return (0 != IN6_IS_ADDR_MULTICAST(&(((struct sockaddr_in6*)&addr)->sin6_addr)) ? true : false);
            }
        }
#endif // HAVE_IPV6

        case ETH:
        {
            return (0 != (0x01 & ((char*)&addr)[3]));
        }

#ifdef SIMULATE
            case SIM:
#ifdef SCENSIM_NRLOLSR //ScenSim-Port://
                return (0 != (((struct sockaddr_sim *)&addr)->addr & 0x80000000));//ScenSim-Port://
            // && ((struct sockaddr_sim *)&addr)->addr != 0xffffffff);//ScenSim-Port://
#endif // SCENSIM_NRLOLSR//ScenSim-Port://
#ifdef NS2
                return (0 != (((struct sockaddr_sim *)&addr)->addr & 0x80000000));
            // && ((struct sockaddr_sim *)&addr)->addr != 0xffffffff);
#endif // NS2
#ifdef OPNET  // For now our OPNET "model" uses broadcast address always
// JPH 11/2/2005 - use IPv4 multicast addresses
        {
            return ((0xf0000000 & ((struct sockaddr_sim *)&addr)->addr)
                        == 0xe0000000);
        }
// end JPH 11/2/2005
#endif // OPNET
#endif // SIMULATE
            default:
                return false;
    }
}  // end IsMulticast()

bool ProtoAddress::IsBroadcast() const
{
    switch(type)
    {
        case IPv4:
        {
            struct in_addr inAddr = ((struct sockaddr_in*)&addr)->sin_addr;
            return (((UINT32)inAddr.s_addr) == htonl(0xffffffff));
        }

#ifdef HAVE_IPV6
        case IPv6:
        {
            return false;  // no IPv6 broadcast address
        }
#endif // HAVE_IPV6

        case ETH:
        {
            const unsigned char temp[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
            return (0 == memcmp((const char*)&addr, temp, 6));
        }

#ifdef SIMULATE
            case SIM:
#ifdef SCENSIM_NRLOLSR //ScenSim-Port://
                return (((struct sockaddr_sim *)&addr)->addr == 0xffffffff);//ScenSim-Port://
#endif // SCENSIM_NRLOLSR //ScenSim-Port://
#ifdef NS2
                return (((struct sockaddr_sim *)&addr)->addr == 0xffffffff);
#endif // NS2
#ifdef OPNET
// JPH 11/2/2005 - add Opnet code for broadcast
        return (((struct sockaddr_sim *)&addr)->addr == 0xffffffff);
// end JPH 11/2/2005
#endif // OPNET
#endif // SIMULATE
            default:
                return false;
    }
}  // end IsBroadcast()

bool ProtoAddress::IsLoopback() const
{
    switch(type)
    {
        case IPv4:
        {
            struct in_addr inAddr = ((struct sockaddr_in*)&addr)->sin_addr;
            return (htonl(0x7f000001) == ((UINT32)inAddr.s_addr));
        }
#ifdef HAVE_IPV6
        case IPv6:
        {
            if (IN6_IS_ADDR_V4MAPPED(&(((struct sockaddr_in6*)&addr)->sin6_addr)))
                return (htonl(0x7f000001) ==
                        IN6_V4MAPPED_ADDR(&(((struct sockaddr_in6*)&addr)->sin6_addr)));
            else
                return (0 != IN6_IS_ADDR_LOOPBACK(&(((struct sockaddr_in6*)&addr)->sin6_addr)) ? true : false);
        }
#endif // HAVE_IPV6
        case ETH:
            return false;
#ifdef SIMULATE
        case SIM:
            return false;
#endif // SIMULATE
        default:
            return false;
    }
}  // end ProtoAddress::IsLoopback()

bool ProtoAddress::IsUnspecified() const
{
    switch(type)
    {
            case IPv4:
            {
                struct in_addr inAddr = ((struct sockaddr_in*)&addr)->sin_addr;
                return (0x00000000 == inAddr.s_addr);
            }
#ifdef HAVE_IPV6
        case IPv6:
        {
            if (IN6_IS_ADDR_V4MAPPED(&(((struct sockaddr_in6*)&addr)->sin6_addr)))
            {
                return (0x0000000 ==
                        IN6_V4MAPPED_ADDR(&(((struct sockaddr_in6*)&addr)->sin6_addr)));
            }
            else
            {
                return (0 != IN6_IS_ADDR_UNSPECIFIED(&(((struct sockaddr_in6*)&addr)->sin6_addr)) ? true : false);
            }
        }
#endif // HAVE_IPV6
        case ETH:
            return false;
#ifdef SIMULATE
        case SIM:
            return false;
#endif // SIMULATE
        default:
            return false;
    }
}  // end IsUnspecified()

bool ProtoAddress::IsLinkLocal() const
{
    switch(type)
    {
#ifdef HAVE_IPV6
        case IPv6:
        {
            struct in6_addr* a = &(((struct sockaddr_in6*)&addr)->sin6_addr);
            return (IN6_IS_ADDR_MULTICAST(a) ?
                    (0 != IN6_IS_ADDR_MC_LINKLOCAL(a) ? true : false) :
                    (0 != IN6_IS_ADDR_LINKLOCAL(a) ? true : false));
        }
#endif // HAVE_IPV6
        case IPv4:
        {
            UINT32 addrVal = (UINT32)(((struct sockaddr_in*)&addr)->sin_addr.s_addr);
            if (((UINT32)(htonl(0xffffff00) & addrVal))
                == (UINT32)htonl(0xe0000000))
            {
                // Address is 224.0.0/24 multicast.
                return true;
            }
            else if (((UINT32)(htonl(0xffff0000) & addrVal))
                     == (UINT32)htonl(0xa9fe0000))
            {
                // Address is 169.254/16 unicast.
                return true;
            }
            return false;
        }
        case ETH:
            return false;  // or return true???
        default:
           return false;
    }
}  // end ProtoAddress::IsLinkLocal()

bool ProtoAddress::IsSiteLocal() const
{
    switch(type)
    {
#ifdef HAVE_IPV6
        case IPv6:
        {
            struct in6_addr* a = &(((struct sockaddr_in6*)&addr)->sin6_addr);
            return (IN6_IS_ADDR_MULTICAST(a) ?
                    (0 != IN6_IS_ADDR_MC_SITELOCAL(a) ? true : false) :
                    (0 != IN6_IS_ADDR_SITELOCAL(a) ? true : false));
        }
#endif // HAVE_IPV6
        case IPv4:
        case ETH:
            return false;
        default:
           return false;
    }
}  // end ProtoAddress::IsSiteLocal()

const char* ProtoAddress::GetHostString(char* buffer, unsigned int buflen) const
{
    static char altBuffer[256];
    altBuffer[255] = '\0';
    buflen = (NULL != buffer) ? buflen : 255;
#ifdef _UNICODE
    buflen /= 2;
#endif // _UNICODE
    buffer = (NULL != buffer) ? buffer : altBuffer;
    switch (type)
    {
#ifdef WIN32
        case IPv4:
#ifdef HAVE_IPV6
        case IPv6:
#endif // HAVE_IPV6
        {
            // Startup WinSock for name lookup
            if (!Win32Startup())
            {
                DMSG(0, "ProtoAddress: GetHostString(): Error initializing WinSock!\n");
                return NULL;
            }
            unsigned long len = buflen;
            if (0 != WSAAddressToString((SOCKADDR*)&addr, sizeof(addr), NULL, (LPTSTR)buffer, &len))
                DMSG(0, "ProtoAddress::GetHostString() WSAAddressToString() error\n");
            Win32Cleanup();
#ifdef _UNICODE
            // Convert from unicode
            wcstombs(buffer, (wchar_t*)buffer, len);
#endif // _UNICODE
            // Get rid of trailing port number
            if (IPv4 == type)
            {
                char* ptr = strrchr(buffer, ':');
                if (ptr) *ptr = '\0';
            }
#ifdef HAVE_IPV6
            else if (IPv6 == type)
            {
                char* ptr = strchr(buffer, '[');  // nuke start bracket
                if (ptr)
                {
                    size_t len = strlen(buffer);
                    if (len > buflen) len = buflen;
                    memmove(buffer, ptr, len - (ptr-buffer));
                }
                ptr = strrchr(buffer, '%');  // nuke if index, if applicable
                if (!ptr) ptr = strrchr(buffer, ']'); // nuke end bracket
                if (ptr) ptr = '\0';
            }
#endif // HAVE_IPV6
            return buffer ? buffer : "(null)";
        }
#else
#ifdef HAVE_IPV6
        case IPv4:
        {
            const char* result = inet_ntop(AF_INET, &((struct sockaddr_in*)&addr)->sin_addr, buffer, buflen);
            return result ? result : "(bad address)";
        }
        case IPv6:
        {
            const char* result = inet_ntop(AF_INET6, &((struct sockaddr_in6*)&addr)->sin6_addr, buffer, buflen);
            return result ? result : "(bad address)";
        }
#else
        case IPv4:
            strncpy(buffer, inet_ntoa(((struct sockaddr_in *)&addr)->sin_addr), buflen);
            return buffer;
#endif // HAVE_IPV6
#endif // if/else WIN32/UNIX
        case ETH:
        {
            // Print as a hexadecimal number
            unsigned int len = 0;
            for (int i = 0; i < 6; i++)
            {
                if (len < buflen)
                {
                    if (i < 1)
                        len += sprintf(buffer+len, "%02x", ((unsigned char*)&addr)[i]);
                    else
                        len += sprintf(buffer+len, ":%02x", ((unsigned char*)&addr)[i]);
                }
                else
                {
                    break;
                }
            }
            return buffer;
        }
#ifdef SIMULATE
        case SIM:
        {
            char text[32];
#ifndef OPNET // JPH 5/22/06
            sprintf(text, "%u", ((struct sockaddr_sim*)&addr)->addr);
#else
            ip_address_print(text,(IpT_Address)((struct sockaddr_sim*)&addr)->addr);
#endif // OPNET
            strncpy(buffer, text, buflen);
            return buffer ? buffer : "(null)";
        }
#endif // SIMULATE
        default:
            DMSG(0, "ProtoAddress: GetHostString(): Invalid address type!\n");
            return "(invalid address)";
    }  // end switch(type)
}  // end ProtoAddress::GetHostString()

void ProtoAddress::PortSet(UINT16 thePort) {SetPort(thePort);}

void ProtoAddress::SetPort(UINT16 thePort)
{
    switch(type)
    {
            case IPv4:
                ((struct sockaddr_in*)&addr)->sin_port = htons(thePort);
                break;
#ifdef HAVE_IPV6
            case IPv6:
                ((struct sockaddr_in6*)&addr)->sin6_port = htons(thePort);
                break;
#endif // HAVE_IPV6
#ifdef SIMULATE
                case SIM:
                ((struct sockaddr_sim*)&addr)->port = thePort;
                        break;
#endif // SIMULATE
            case ETH:
                break;
            default:
                Reset(IPv4);
                SetPort(thePort);
                break;
    }
}  // end ProtoAddress::SetPort()

UINT16 ProtoAddress::GetPort() const
{
    switch(type)
    {
        case IPv4:
            return ntohs(((struct sockaddr_in *)&addr)->sin_port);
#ifdef HAVE_IPV6
        case IPv6:
            return ntohs(((struct sockaddr_in6 *)&addr)->sin6_port);
#endif // HAVE_IPV6
#ifdef SIMULATE
        case SIM:
            return (((struct sockaddr_sim*)&addr)->port);
#endif // SIMULATE
        case ETH:
        default:
            return 0;  // port 0 is an invalid port
    }
}  // end ProtoAddress::Port()

void ProtoAddress::Reset(ProtoAddress::Type theType, bool zero)
{
    char value = zero ? 0x00 : 0xff;
    switch (theType)
    {
        case IPv4:
            type = IPv4;
            length = 4;
            memset(&((struct sockaddr_in*)&addr)->sin_addr, value, 4);
            ((struct sockaddr_in*)&addr)->sin_family = AF_INET;
            break;
#ifdef HAVE_IPV6
        case IPv6:
            type = IPv6;
            length  = 16;
            memset(&((struct sockaddr_in6*)&addr)->sin6_addr, value, 16);
            ((struct sockaddr_in6*)&addr)->sin6_family = AF_INET6;
            break;
#endif // HAVE_IPV6
        case ETH:
            type = ETH;
            length = 6;
            memset((char*)&addr, value, 6);
            break;
#ifdef SIMULATE
        case SIM:
            type = SIM;
            length = sizeof(SIMADDR);
            memset(&((struct sockaddr_sim*)&addr)->addr, value, sizeof(SIMADDR));
            break;
#endif // SIMULATE
        default:
            DMSG(0, "ProtoAddress::Init() Invalid address type!\n");
            break;
    }
    SetPort(0);
}  // end ProtoAddress::Init();

bool ProtoAddress::SetSockAddr(const struct sockaddr& theAddr)
{
    switch (theAddr.sa_family)
    {
        case AF_INET:
            ((struct sockaddr_in&)addr) = ((struct sockaddr_in&)theAddr);
            type = IPv4;
            length = 4;
            return true;
#ifdef HAVE_IPV6
        case AF_INET6:
            ((struct sockaddr_in6&)addr) = ((struct sockaddr_in6&)theAddr);
            //memcpy(&addr, &theAddr, sizeof(struct sockaddr_in6));
            type = IPv6;
            length = 16;
            return true;
#endif // HAVE_IPV6
        default:
            DMSG(4, "ProtoAddress::SetSockAddr() Invalid address type!\n");
            type = INVALID;
            length = 0;
            return false;
    }
}  // end ProtoAddress:SetAddress()

bool ProtoAddress::SetRawHostAddress(ProtoAddress::Type theType,
                                     const char*        buffer,
                                     UINT8              buflen)
{
    UINT16 thePort = GetPort();
    switch (theType)
    {
        case IPv4:
            if (buflen > 4) return false;
            type = IPv4;
            length = 4;
            memset(&((struct sockaddr_in*)&addr)->sin_addr, 0, 4);
            memcpy(&((struct sockaddr_in*)&addr)->sin_addr, buffer, buflen);
            ((struct sockaddr_in*)&addr)->sin_family = AF_INET;
            break;
#ifdef HAVE_IPV6
        case IPv6:
            if (buflen > 16) return false;
            type = IPv6;
            length  = 16;
            memset(&((struct sockaddr_in6*)&addr)->sin6_addr, 0, 16);
            memcpy(&((struct sockaddr_in6*)&addr)->sin6_addr, buffer, buflen);
            ((struct sockaddr_in6*)&addr)->sin6_family = AF_INET6;
            break;
#endif // HAVE_IPV6
        case ETH:
            if (buflen > 6) return false;
            type = ETH;
            length = 6;
            memset((char*)&addr, 0, 6);
            memcpy((char*)&addr, buffer, buflen);
            break;
#ifdef SIMULATE
        case SIM:
            if (buflen > sizeof(SIMADDR)) return false;
            type = SIM;
            length = sizeof(SIMADDR);
            memset(&((struct sockaddr_sim*)&addr)->addr, 0, sizeof(SIMADDR));
            memcpy(&((struct sockaddr_sim*)&addr)->addr, buffer, buflen);
            break;
#endif // SIMULATE
        default:
            DMSG(0, "ProtoAddress::SetRawHostAddress() Invalid address type!\n");
            return false;
    }
    SetPort(thePort);
    return true;
}  // end ProtoAddress::SetRawHostAddress()

const char* ProtoAddress::GetRawHostAddress() const
{
    switch (type)
    {
        case IPv4:
            return ((char*)&((struct sockaddr_in*)&addr)->sin_addr);
#ifdef HAVE_IPV6
        case IPv6:
            return ((char*)&((struct sockaddr_in6*)&addr)->sin6_addr);
#endif // HAVE_IPV6
        case ETH:
            return ((char*)&addr);
#ifdef SIMULATE
        case SIM:
            return ((char*)&((struct sockaddr_sim*)&addr)->addr);
#endif // SIMULATE
        default:
            DMSG(0, "ProtoAddress::RawHostAddress() Invalid address type!\n");
            return NULL;
    }
}  // end ProtoAddress::RawHostAddress()

UINT8 ProtoAddress::GetPrefixLength() const
{
    UINT8* ptr = NULL;
    UINT8 maxBytes = 0;
    switch (type)
    {
        case IPv4:
            ptr = ((UINT8*)&((struct sockaddr_in*)&addr)->sin_addr);
            maxBytes = 4;
            break;
#ifdef HAVE_IPV6
        case IPv6:
            ptr = ((UINT8*)&((struct sockaddr_in6*)&addr)->sin6_addr);
            maxBytes = 16;
            break;
#endif // HAVE_IPV6
#ifdef SIMULATE
        case SIM:
            ptr = ((UINT8*)&((struct sockaddr_sim*)&addr)->addr);
            maxBytes = sizeof(SIMADDR);
            break;
#endif // SIMULATE
        case ETH:
        default:
            DMSG(0, "ProtoAddress::PrefixLength() Invalid address type!\n");
            return 0;
    }
    UINT8 prefixLen = 0;
    for (UINT8 i = 0; i < maxBytes; i++)
    {
        if (0xff == *ptr)
        {
            prefixLen += 8;
            ptr++;
        }
        else
        {
            UINT8 bit = 0x80;
            while (0 != (bit & *ptr))
            {
                bit >>= 1;
                prefixLen += 1;
            }
            break;
        }
    }
    return prefixLen;
}  // end ProtoAddress::GetPrefixLength()

void ProtoAddress::ApplyPrefixMask(UINT8 prefixLen)
{
    UINT8* ptr = NULL;
    UINT8 maxLen = 0;
    switch (type)
    {
        case IPv4:
            ptr = ((UINT8*)&((struct sockaddr_in*)&addr)->sin_addr);
            maxLen = 32;
            break;
#ifdef HAVE_IPV6
        case IPv6:
            ptr = ((UINT8*)&((struct sockaddr_in6*)&addr)->sin6_addr);
            maxLen = 128;
            break;
#endif // HAVE_IPV6
#ifdef SIMULATE
        case SIM:
            ptr = ((UINT8*)&((struct sockaddr_sim*)&addr)->addr);
            maxLen = sizeof(SIMADDR) << 3;
            break;
#endif // SIMULATE
        case ETH:
        default:
            DMSG(0, "ProtoAddress::ApplyPrefixMask() Invalid address type!\n");
            return;
    }
    if (prefixLen >= maxLen) return;
    UINT8 nbytes = prefixLen >> 3;
    UINT8 remainder = prefixLen & 0x07;
    if (remainder)
    {
        ptr[nbytes] &= (UINT8)(0x00ff << (8 - remainder));
        nbytes++;
    }
    memset(ptr + nbytes, 0, length - nbytes);
}  // end ProtoAddress::ApplyPrefixMask()

void ProtoAddress::GetSubnetAddress(UINT8         prefixLen,
                                    ProtoAddress& subnetAddr) const
{
    subnetAddr = *this;
    UINT8* ptr = NULL;
    UINT8 maxLen = 0;
    switch (type)
    {
        case IPv4:
            ptr = ((UINT8*)&((struct sockaddr_in*)&subnetAddr.addr)->sin_addr);
            maxLen = 32;
            break;
#ifdef HAVE_IPV6
        case IPv6:
            ptr = ((UINT8*)&((struct sockaddr_in6*)&subnetAddr.addr)->sin6_addr);
            maxLen = 128;
            break;
#endif // HAVE_IPV6
        case ETH:
            return;
#ifdef SIMULATE
        case SIM:
            ptr = ((UINT8*)&((struct sockaddr_sim*)&subnetAddr.addr)->addr);
            maxLen = sizeof(SIMADDR) << 3;
            break;
#endif // SIMULATE
        default:
            DMSG(0, "ProtoAddress::GetSubnetAddress() Invalid address type!\n");
            return;
    }
    if (prefixLen >= maxLen) return;
    UINT8 nbytes = prefixLen >> 3;
    UINT8 remainder = prefixLen & 0x07;
    if (remainder)
    {
        ptr[nbytes] &= (UINT8)(0xff << (8 - remainder));
        nbytes++;
    }
    memset(ptr + nbytes, 0, length - nbytes);
}  // end ProtoAddress::GetSubnetAddress()

void ProtoAddress::GetBroadcastAddress(UINT8         prefixLen,
                                       ProtoAddress& broadcastAddr) const
{
    broadcastAddr = *this;
    UINT8* ptr = NULL;
    UINT8 maxLen = 0;
    switch (type)
    {
        case IPv4:
            ptr = ((UINT8*)&((struct sockaddr_in*)&broadcastAddr.addr)->sin_addr);
            maxLen = 32;
            break;
#ifdef HAVE_IPV6
        case IPv6:
            ptr = ((UINT8*)&((struct sockaddr_in6*)&broadcastAddr.addr)->sin6_addr);
            maxLen = 128;
            break;
#endif // HAVE_IPV6
        case ETH:
            ptr = (UINT8*)&broadcastAddr.addr;
            maxLen = 48;
            prefixLen = 0;
            break;
#ifdef SIMULATE
        case SIM:
            ptr = ((UINT8*)&((struct sockaddr_sim*)&broadcastAddr.addr)->addr);
            maxLen = sizeof(SIMADDR) << 3;
            break;
#endif // SIMULATE
        default:
            DMSG(0, "ProtoAddress::GetBroadcastAddress() Invalid address type!\n");
            return;
    }
    if (prefixLen >= maxLen) return;
    UINT8 nbytes = prefixLen >> 3;
    UINT8 remainder = prefixLen & 0x07;
    if (remainder)
    {
        ptr[nbytes] |= (0x00ff >> remainder);
        nbytes++;
    }
    memset(ptr + nbytes, 0xff, length - nbytes);
}  // end ProtoAddress::GetBroadcastAddress()

UINT32 ProtoAddress::EndIdentifier() const
{
    switch(type)
    {
        case IPv4:
        {
            struct in_addr inAddr = ((struct sockaddr_in*)&addr)->sin_addr;
            return ntohl(inAddr.s_addr);
        }
#ifdef HAVE_IPV6
        case IPv6:
            return ntohl(IN6_V4MAPPED_ADDR(&(((struct sockaddr_in6*)&addr)->sin6_addr)));
#endif // HAVE_IPV6
        case ETH:
        {
            // a dumb little hash: MSB is randomized vendor id, 3 LSB's is device id
            UINT8* addrPtr = (UINT8*)&addr;
            UINT8 vendorHash = addrPtr[0] ^ addrPtr[1] ^ addrPtr[2];
            UINT32 temp32;
            memcpy((char*)&temp32, &vendorHash, 1);
            memcpy((((char*)&temp32)+1), addrPtr+3, 3);
            return ntohl(temp32);
        }
#ifdef SIMULATE
        case SIM:
            return ((UINT32)((struct sockaddr_sim*)&addr)->addr);
#endif // SIMULATE
        default:
            DMSG(0, "ProtoAddress::EndIdentifier(): Invalid address type!\n");
            return INADDR_NONE;
    }
}  // end ProtoAddress:EndIdentifier()

bool ProtoAddress::HostIsEqual(const ProtoAddress& theAddr) const
{
    if (!IsValid() && !theAddr.IsValid()) return true;
    switch(type)
    {
        case IPv4:
        {
            struct in_addr myAddrIn = ((struct sockaddr_in *)&addr)->sin_addr;
            struct in_addr theAddrIn = ((struct sockaddr_in *)&(theAddr.addr))->sin_addr;
            if ((IPv4 == theAddr.type) &&
                (myAddrIn.s_addr == theAddrIn.s_addr))
                return true;
            else
                return false;
        }
#ifdef HAVE_IPV6
        case IPv6:
            if ((IPv6 == theAddr.type) &&
                (0 == memcmp(((struct sockaddr_in6*)&addr)->sin6_addr.s6_addr,
                             ((struct sockaddr_in6*)&(theAddr.addr))->sin6_addr.s6_addr,
                             4*sizeof(UINT32))))
                return true;
            else
                return false;
#endif // HAVE_IPV6
        case ETH:
            if ((ETH == theAddr.type) &&
                0 == memcmp((char*)&addr, (char*)&theAddr.addr, 6))
                return true;
            else
                return false;
#ifdef SIMULATE
        case SIM:
            if ((SIM == theAddr.type) &&
                (((struct sockaddr_sim*)&addr)->addr ==
                 ((struct sockaddr_sim*)&(theAddr.addr))->addr))
                return true;
            else
                return false;
#endif // SIMULATE

            default:
                DMSG(0, "ProtoAddress::HostIsEqual(): Invalid address type!\n");
                return false;
    }
}  // end ProtoAddress::HostIsEqual()

int ProtoAddress::CompareHostAddr(const ProtoAddress& theAddr) const
{
    switch(type)
    {
        case IPv4:
            return memcmp(&((struct sockaddr_in *)&addr)->sin_addr,
                          &((struct sockaddr_in *)&theAddr.addr)->sin_addr,
                          4);
#ifdef HAVE_IPV6
        case IPv6:
            return memcmp(&((struct sockaddr_in6*)&addr)->sin6_addr,
                          &((struct sockaddr_in6*)&theAddr.addr)->sin6_addr,
                          16);
#endif // HAVE_IPV6
        case ETH:
            return memcmp((char*)&addr, (char*)&theAddr.addr, 6);
#ifdef SIMULATE
        case SIM:
        {
            SIMADDR addr1 = ((struct sockaddr_sim*)&addr)->addr;
            SIMADDR addr2 = ((struct sockaddr_sim*)&(theAddr.addr))->addr;
            if (addr1 < addr2)
                return -1;
            else if (addr1 > addr2)
                return 1;
            else
                return 0;
        }
#endif // SIMULATE
        default:
            DMSG(0, "ProtoAddress: CompareHostAddr(): Invalid address type!\n");
            return -1;
     }
}  // end ProtoAddress::CompareHostAddress()

// (TBD) Provide a mechanism for async lookup with optional user interaction
bool ProtoAddress::ResolveFromString(const char* text)
{
    UINT16 thePort = GetPort();
#ifdef SIMULATE
    // Assume no DNS for simulations
    SIMADDR theAddr;
#ifndef OPNET // JPH 2/8/06
    if (1 == sscanf(text, "%i", &theAddr))
#else
    if(theAddr = ip_address_create(text))
#endif // OPNET
    {
        ((struct sockaddr_sim*)&addr)->addr = theAddr;
        type = SIM;
        length = sizeof(SIMADDR);
        SetPort(thePort);  // restore port number
        return true;
    }
    else
    {
        fprintf(stderr, "ProtoAddress::ResolveFromString() Invalid simulator address!\n");
        return false;
    }
#else
   // Use DNS to look it up
   // Get host address, looking up by hostname if necessary
#ifdef WIN32
    // Startup WinSock for name lookup
    if (!Win32Startup())
    {
            DMSG(0, "ProtoAddress::ResolveFromString() Error initializing WinSock!\n");
            return false;
    }
#endif //WIN32
#ifdef HAVE_IPV6  // Try to get address using getaddrinfo()
    addrinfo* addrInfoPtr = NULL;
    if(0 == getaddrinfo(text, NULL, NULL, &addrInfoPtr))
    {
#ifdef WIN32
        Win32Cleanup();
#endif // WIN32
        memcpy((char*)&addr, addrInfoPtr->ai_addr, addrInfoPtr->ai_addrlen);
        if (addrInfoPtr->ai_family == PF_INET)
        {
            type = IPv4;
            length =  4; // IPv4 host addresses are always 4 bytes in length
            freeaddrinfo(addrInfoPtr);
        }
        else if (addrInfoPtr->ai_family == PF_INET6)
        {
            type = IPv6;
            length = 16; // IPv6 host addresses are always 16 bytes in length
            freeaddrinfo(addrInfoPtr);
        }
        else
        {
            freeaddrinfo(addrInfoPtr);
            DMSG(0, "ProtoAddress::ResolveFromString() getaddrinfo() returned unsupported address family!\n");
            return false;
        }
        SetPort(thePort);  // restore port number
        return true;
    }
    else
    {
        DMSG(4, "ProtoAddress::ResolveFromString() getaddrinfo() error: %s\n", GetErrorString());
#ifdef WIN32
        // on WinNT 4.0, getaddrinfo() doesn't work, so we check the OS version
        // to decide what to do.  Try "gethostbyaddr()" if it's an old OS (e.g. NT 4.0 or earlier)
        OSVERSIONINFO vinfo;
        vinfo.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);
        GetVersionEx(&vinfo);
        if ((VER_PLATFORM_WIN32_NT == vinfo.dwPlatformId) &&
            ((vinfo.dwMajorVersion > 4) ||
             ((vinfo.dwMajorVersion == 4) && (vinfo.dwMinorVersion > 0))))
                    return false;  // it's a modern OS where getaddrinfo() should work!
#else
        return false;
#endif // if/else WIN32/UNIX
    }
#endif  // HAVE_IPV6
    // Use this approach as a backup for NT-4 or if !HAVE_IPV6
    // 1) is it a "dotted decimal" address?
    struct sockaddr_in* addrPtr = (struct sockaddr_in*)&addr;
    if (INADDR_NONE != (addrPtr->sin_addr.s_addr = inet_addr(text)))
    {
        addrPtr->sin_family = AF_INET;
    }
    else
    {
        // 2) Use "gethostbyname()" to lookup IPv4 address
        struct hostent *hp = gethostbyname(text);
#ifdef WIN32
        Win32Cleanup();
#endif // WIN32
        if(hp)
        {
            addrPtr->sin_family = hp->h_addrtype;
            memcpy((char*)&addrPtr->sin_addr,  hp->h_addr,  hp->h_length);
        }
        else
        {
            DMSG(0, "ProtoAddress::ResolveFromString() gethostbyname() error: %s\n",
                    GetErrorString());
            return false;
        }
    }
    if (addrPtr->sin_family == AF_INET)
    {
        type = IPv4;
        length =  4;  // IPv4 addresses are always 4 bytes in length
        SetPort(thePort);  // restore port number
        return true;
    }
    else
    {
        DMSG(0, "ProtoAddress::ResolveFromString gethostbyname() returned unsupported address family!\n");
        return false;
    }

#endif // !SIMULATE
}  // end ProtoAddress::ResolveFromString()

bool ProtoAddress::ResolveToName(char* buffer, unsigned int buflen) const
{
#ifdef WIN32
        if (!Win32Startup())
        {
            DMSG(0, "ProtoAddress::ResolveToName() Error initializing WinSock!\n");
            GetHostString(buffer, buflen);
            return false;
        }
#endif // WIN32
    struct hostent* hp = NULL;
    switch (type)
    {
        case IPv4:
            hp = gethostbyaddr((char*)&(((struct sockaddr_in*)&addr)->sin_addr),
                               4, AF_INET);
            break;
#ifdef HAVE_IPV6
        case IPv6:
            hp = gethostbyaddr((char*)&(((struct sockaddr_in*)&addr)->sin_addr),
                               16, AF_INET6);
         break;
#endif // HAVE_IPV6
#ifdef SIMULATE
        case SIM:
            GetHostString(buffer, buflen);
            return true;
#endif // SIMULATE
        case ETH:
            GetHostString(buffer, buflen);
            return true;
        default:
            DMSG(0, "ProtoAddress::ResolveToName(): Invalid address type!\n");
            return false;
    }  // end switch(type)
#ifdef WIN32
        Win32Cleanup();
#endif // WIN32

    if (hp)
    {
        size_t nameLen = 0;
        unsigned int dotCount = 0;
        strncpy(buffer, hp->h_name, buflen);
        nameLen = strlen(hp->h_name);
        nameLen = nameLen < buflen ? nameLen : buflen;

        const char* ptr = hp->h_name;
        while (true) {
            ptr = strchr(ptr, '.');
            if (ptr == nullptr) {
                break;
            }//if//
            ptr++;
            dotCount++;
        }

        char** alias = hp->h_aliases;
        // Use first alias by default
        if (alias && *alias && buffer)
        {
            strncpy(buffer, *alias, buflen);
            nameLen = strlen(*alias);
            nameLen = nameLen < buflen ? nameLen : buflen;
            alias++;
            // Try to find the fully-qualified name
            // (longest alias with most '.')
            while (*alias)
            {
                unsigned int dc = 0;

                ptr = *alias;
                while (true) {
                    ptr = strchr(ptr, '.');
                    if (ptr == nullptr) {
                        break;
                    }
                    ptr++;
                    dc++;
                }

                size_t alen = strlen(*alias);
                bool override = (dc > dotCount) ||
                                ((dc == dotCount) && (alen >nameLen));
                if (override)
                {
                    strncpy(buffer, *alias, buflen);
                    nameLen = alen;
                    dotCount = dc;
                    nameLen = nameLen < buflen ? nameLen : buflen;
                }
                alias++;
            }
        }
        return true;
    }
    else
    {
        DMSG(0, "ProtoAddress::ResolveToName() gethostbyaddr() error: %s\n",
                GetErrorString());
        GetHostString(buffer, buflen);
        return false;
    }
}  // end ProtoAddress::ResolveToName()

bool ProtoAddress::ResolveLocalAddress(char* buffer, unsigned int buflen)
{
    UINT16 thePort = GetPort();  // save port number
    // Try to get fully qualified host name if possible
    char hostName[256];
    hostName[0] = '\0';
    hostName[255] = '\0';
#ifdef WIN32
    if (!Win32Startup())
    {
        DMSG(0, "ProtoAddress::ResolveLocalAddress() error startup WinSock!\n");
        return false;
    }
#endif // WIN32
    int result = gethostname(hostName, 255);
#ifdef WIN32
    Win32Cleanup();
#endif  // WIN32
    if (result)
    {
        DMSG(0, "ProtoAddress::ResolveLocalAddress(): gethostname() error: %s\n", GetErrorString());
        return false;
    }
    else
    {
        // Terminate at first '.' in name, if any (e.g. MacOS .local)
        char* dotPtr = strchr(hostName, '.');
        if (dotPtr) *dotPtr = '\0';
        bool lookupOK = ResolveFromString(hostName);
        if (lookupOK)
        {
            // Invoke ResolveToName() to get fully qualified name and use that address
            ResolveToName(hostName, 255);
            lookupOK = ResolveFromString(hostName);
        }
        if (!lookupOK || IsLoopback())
        {
            // darn it! lookup failed or we got the loopback address ...
            // So, try to troll interfaces for a decent address
            // using our handy-dandy ProtoSocket::GetInterfaceInfo routines
            gethostname(hostName, 255);
            if (!lookupOK)
            {
                // Set IPv4 loopback address as fallback id if no good address is found
                UINT32 loopbackAddr = htonl(0x7f000001);
                SetRawHostAddress(IPv4, (char*)&loopbackAddr, 4);
            }

            // Use "ProtoSocket::FindLocalAddress()" that trolls through network interfaces
            // looking for an interface with a non-loopback address assigned.
            if (!ProtoSocket::FindLocalAddress(IPv4, *this))
            {
#ifdef HAVE_IPV6
                // Try IPv6 if IPv4 wasn't assigned
                if (!ProtoSocket::FindLocalAddress(IPv6, *this))
                {
                    DMSG(0, "ProtoAddress::ResolveLocalAddress() warning: no assigned addresses found\n");
                }
#endif // HAVE_IPV6
            }
            if (IsLoopback() || IsUnspecified())
                DMSG(0, "ProtoAddress::ResolveLocalAddress() warning: only loopback address found!\n");
        }
        SetPort(thePort);  // restore port number
        buflen = buflen < 255 ? buflen : 255;
        if (buffer) strncpy(buffer, hostName, buflen);
        return true;
    }
}  // end ProtoAddress::ResolveLocalAddress()


ProtoAddressList::ProtoAddressList()
 : addr_count(0)
{
}

ProtoAddressList::~ProtoAddressList()
{
    Destroy();
}

void ProtoAddressList::Destroy()
{
    Entry* entry;
    while (NULL != (entry = static_cast<Entry*>(addr_tree.GetRoot())))
    {
        addr_tree.Remove(*entry);
        delete entry;
    }
    addr_count = 0;
}  // end ProtoAddressList::Destroy()

bool ProtoAddressList::Insert(const ProtoAddress& theAddress, const void* userData)
{
    if (!theAddress.IsValid())
    {
        DMSG(0, "ProtoAddressList::Insert() error: invalid address\n");
        return false;
    }
    if (!Contains(theAddress))
    {
        Entry* entry = new Entry(theAddress, userData);
        if (NULL == entry)
        {
            DMSG(0, "ProtoAddressList::Insert() new ProtoTree::Item error: %s\n", GetErrorString());
            return false;
        }
        addr_tree.Insert(*entry);
        addr_count++;
    }
    return true;
}  // end ProtoAddressList::Insert()

void ProtoAddressList::Remove(const ProtoAddress& addr)
{
    Entry* entry = static_cast<Entry*>(addr_tree.Find(addr.GetRawHostAddress(), addr.GetLength() << 3));
    if (NULL != entry)
    {
        addr_tree.Remove(*entry);
        delete entry;
        addr_count--;
    }
}  // end ProtoAddressList::Remove()

// Returns first address added to tree (subroot of ProtoTree) for given sizeof(addrType)
bool ProtoAddressList::GetFirstAddress(ProtoAddress::Type addrType, ProtoAddress& firstAddr) const
{
    Entry* rootEntry = static_cast<Entry*>(addr_tree.GetRoot());
    if (NULL != rootEntry)
    {
        firstAddr = rootEntry->GetAddress();
        return true;
    }
    else
    {
        firstAddr.Invalidate();
        return false;
    }
}  // end ProtoAddressList::GetFirstAddress()

ProtoAddressList::Entry::Entry(const ProtoAddress& theAddr, const void* userData)
 : addr(theAddr)
{
    addr.SetUserData(userData);
}

ProtoAddressList::Entry::~Entry()
{
}


ProtoAddressList::Iterator::Iterator(const ProtoAddressList& addrList)
 : ptree_iterator(addrList.addr_tree)
{
}

ProtoAddressList::Iterator::~Iterator()
{
}

bool ProtoAddressList::Iterator::GetNextAddress(ProtoAddress& nextAddr)
{
    Entry* nextEntry = static_cast<Entry*>(ptree_iterator.GetNextItem());
    if (NULL != nextEntry)
    {
        nextAddr = nextEntry->GetAddress();
        return true;
    }
    else
    {
        nextAddr.Invalidate();
        return false;
    }
}  // end ProtoAddressList::Iterator::GetNextAddress()



} //namespace// //ScenSim-Port://

