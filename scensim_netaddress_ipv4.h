// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_NETADDRESS_IPV4_H
#define SCENSIM_NETADDRESS_IPV4_H

#ifndef SCENSIM_NETADDRESS_H
#error "Include scensim_netaddress.h not scensim_netaddress_ipv4.h (indirect include only)"
#endif

//Network address abstraction class for 32-bit IPv4 address

#include <assert.h>
#include <string>
#include <sstream>
#include <stdint.h>
#include "scensim_nodeid.h"
#include "scensim_support.h"

namespace ScenSim {

using std::string;
using std::ostringstream;
using std::vector;

class Ipv4NetworkAddress {
public:
    static const unsigned int numberBits = 32;

    static const unsigned int maxMulticastGroupNumber = 0xFFFFFF;

    static const Ipv4NetworkAddress anyAddress;
    static const Ipv4NetworkAddress broadcastAddress;
    static const Ipv4NetworkAddress invalidAddress;

    Ipv4NetworkAddress() { ipAddress = Ipv4NetworkAddress::ipv4AnyAddress; }

    explicit
    Ipv4NetworkAddress(const uint32_t initIpAddress): ipAddress(initIpAddress) {}

    Ipv4NetworkAddress(const uint64_t& ipAddressHighBits, const uint64_t& ipAddressLowBits)
        : ipAddress(static_cast<uint32_t>(ipAddressLowBits)) {}

    Ipv4NetworkAddress(const Ipv4NetworkAddress& subnetAddress, const Ipv4NetworkAddress& hostIdentifierOnlyAddress)
        : ipAddress(subnetAddress.ipAddress + hostIdentifierOnlyAddress.ipAddress) {}

    void SetAddressFromString(const string& stringToConvert, const NodeIdType& nodeId, bool& success);

    void SetAddressFromString(const string& stringToConvert, bool& success)
        { (*this).SetAddressFromString(stringToConvert, INVALID_NODEID, success); }

    string ConvertToString() const;

    void SetToTheBroadcastAddress() { ipAddress = Ipv4NetworkAddress::ipv4BroadcastAddress; }
    static Ipv4NetworkAddress ReturnTheBroadcastAddress() { return Ipv4NetworkAddress(Ipv4NetworkAddress::ipv4BroadcastAddress); }

    static Ipv4NetworkAddress MakeABroadcastAddress(
        const Ipv4NetworkAddress& subnetAddress,
        const Ipv4NetworkAddress& subnetMask)
    {
        return Ipv4NetworkAddress(subnetAddress.ipAddress | ~subnetMask.ipAddress);
    }

    void SetToAnyAddress() { ipAddress = Ipv4NetworkAddress::ipv4AnyAddress; }
    static Ipv4NetworkAddress ReturnAnyAddress() { return Ipv4NetworkAddress(Ipv4NetworkAddress::ipv4AnyAddress); }

    bool IsTheBroadcastAddress() const { return (ipAddress == Ipv4NetworkAddress::ipv4BroadcastAddress); }

    bool IsABroadcastAddress(const Ipv4NetworkAddress& subnetMask) const {
        return ((ipAddress | subnetMask.ipAddress) == Ipv4NetworkAddress::ipv4BroadcastAddress);
    }

    bool IsAMulticastAddress() const { return ((ipAddress & 0xF0000000) == 0xE0000000); }
    bool IsLinkLocalAddress() const { return false; }
    bool IsAnyAddress() const { return (ipAddress == Ipv4NetworkAddress::ipv4AnyAddress); }

    bool IsTheBroadcastOrAMulticastAddress() const {
        return (IsTheBroadcastAddress() || IsAMulticastAddress());
    }

    bool IsABroadcastOrAMulticastAddress(const Ipv4NetworkAddress& subnetMask) const {
        return (IsABroadcastAddress(subnetMask) || IsAMulticastAddress());
    }

    bool operator==(const Ipv4NetworkAddress& right) const {
        return ((*this).ipAddress == right.ipAddress);
    }

    bool operator!=(const Ipv4NetworkAddress& right) const {
        return ((*this).ipAddress != right.ipAddress);
    }

    bool operator<(const Ipv4NetworkAddress& right) const {
        return ((*this).ipAddress < right.ipAddress);
    }

    bool operator>(const Ipv4NetworkAddress& right) const {
        return ((*this).ipAddress > right.ipAddress);
    }

    uint64_t GetRawAddressLowBits() const { return ipAddress; }
    uint64_t GetRawAddressHighBits() const { return 0; }

    uint32_t GetRawAddressLow32Bits() const { return ipAddress; }

    unsigned int GetMulticastGroupNumber() const
        { assert(IsAMulticastAddress()); return (ipAddress & 0xFFFFFF); }

    void SetWith32BitRawAddress(const uint32_t newipAddress) { (*this).ipAddress = newipAddress; }

    bool IsInSameSubnetAs(const Ipv4NetworkAddress& address, const Ipv4NetworkAddress& subnetMask) const
    {
        return (((ipAddress & subnetMask.ipAddress) == (address.ipAddress & subnetMask.ipAddress)) ||
                (IsTheBroadcastAddress() | IsAMulticastAddress()));
    }

    Ipv4NetworkAddress MakeSubnetAddress(const Ipv4NetworkAddress& subnetMask) const {
        return Ipv4NetworkAddress(ipAddress & subnetMask.ipAddress);
    }

    static Ipv4NetworkAddress MakeSubnetMask(const unsigned int numberPrefixBits);

    Ipv4NetworkAddress MakeAddressWithZeroedSubnetBits(const Ipv4NetworkAddress& subnetMask) const {
        return Ipv4NetworkAddress(ipAddress & ~subnetMask.ipAddress);
    }

    static Ipv4NetworkAddress MakeAMulticastAddress(const unsigned int multicastGroupNumber);

    static bool IsIpv4StyleAddressString(const string& addressString) {
        return ((addressString.find('.')) != string::npos);
    }

private:
    static const uint32_t ipv4BroadcastAddress = 0xFFFFFFFF;
    static const uint32_t ipv4AnyAddress = 0x00000000;

    uint32_t ipAddress;

};//Ipv4NetworkAddress//


inline
string Ipv4NetworkAddress::ConvertToString() const
{
    ostringstream outstream;

    outstream << (ipAddress / 0x01000000) << '.'
              << ((ipAddress / 0x00010000) % 256) << '.'
              << ((ipAddress / 0x00000100) % 256) << '.'
              << (ipAddress % 256);

    return (outstream.str());
}


inline
Ipv4NetworkAddress Ipv4NetworkAddress::MakeSubnetMask(const unsigned int numberPrefixBits) {
    assert(numberPrefixBits >= 0);
    assert(numberPrefixBits <= numberBits);

    unsigned int mask = 0;

    for(unsigned int i = 0; (i < numberPrefixBits); i++) {
        mask = (mask << 1);
        mask = (mask | 0x1);
    }//for//

    mask = (mask << (numberBits - numberPrefixBits));

    return (Ipv4NetworkAddress(mask));

}///MakeSubnetMask//


inline
void Ipv4NetworkAddress::SetAddressFromString(
    const string& stringToConvert,
    const NodeIdType& nodeId,
    bool& success)
{
    // Example IP Format 123.23.134.2
    // Example IP Format 123.23.0.0+$n

    success = false;

    bool nodeIdShouldBeAddedToAddress = false;

    const int NumberParts = 4;
    vector<unsigned char> parts(NumberParts);

    size_t currentPos = 0;

    for(int I = 0; I < NumberParts; I++) {

        size_t dotPos = stringToConvert.find('.',currentPos);

        bool dotWasFound = (dotPos != string::npos);

        if (I == (NumberParts - 1)) {
            // Last part

            if (dotWasFound) {
                // Too many dots: failure
                return;
            }//if//

            // Pretend there is a terminating dot after number

            dotPos = stringToConvert.find_first_not_of("0123456789", currentPos);
            if (dotPos == string::npos) {
                dotPos = stringToConvert.length();
            }//if//

            size_t addNodeIdPos = stringToConvert.find("+$",currentPos);
            if (addNodeIdPos == string::npos) {
                addNodeIdPos = stringToConvert.find("+ $",currentPos);
            }//if//

            if (addNodeIdPos != string::npos) {
                size_t dollarPos = stringToConvert.find('$',currentPos);
                if (((dollarPos + 2) == stringToConvert.length()) &&
                    (tolower(stringToConvert.at(dollarPos+1)) == 'n')) {

                    nodeIdShouldBeAddedToAddress = true;
                } else {
                    // Failure bad +$ format.
                    return;

                }//if//
            }//if//
        }
        else {
            if (!dotWasFound) {
                return;

            }//if//
        }//if//

        bool wasConverted = false;
        int anInt = 0;
        const size_t fieldLength = (dotPos - currentPos);
        ConvertStringToInt(stringToConvert.substr(currentPos, fieldLength), anInt, wasConverted);

        if ((!wasConverted) || (anInt < 0) || (anInt > 255)) {
            return;

        }//if//

        currentPos = dotPos + 1;

        parts.at(I) = static_cast<unsigned char>(anInt);

    }//for//

    ipAddress =
        (parts[0] * (256 * 256 * 256)) +
        (parts[1] * (256 * 256)) +
        (parts[2] * 256) + parts[3];

    if (nodeIdShouldBeAddedToAddress) {
        if (nodeId == INVALID_NODEID) {
            // Illegal Node ID given no "$n" allowed.
            success = false;
            return;

        }//if//

        ipAddress += nodeId;
    }//if//

    success = true;

}//SetAddressFromString//


inline
Ipv4NetworkAddress Ipv4NetworkAddress::MakeAMulticastAddress(const unsigned int multicastGroupNumber)
{
    return (
        Ipv4NetworkAddress(
            Ipv4NetworkAddress(0xE1000000), // 225.0.0.0
            Ipv4NetworkAddress(static_cast<uint32_t>(multicastGroupNumber))));
}



}//namespace//

#endif


