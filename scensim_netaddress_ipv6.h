// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_NETADDRESS_IPV6_H
#define SCENSIM_NETADDRESS_IPV6_H

#ifndef SCENSIM_NETADDRESS_H
#error "Include scensim_netaddress.h not scensim_netaddress_ipv6.h (indirect include only)"
#endif

//Network address abstraction class for 128-bit IPv6 address

#include <assert.h>
#include <string>
#include <sstream>
#include <stdint.h>
#include "scensim_nodeid.h"
#include "scensim_support.h"
#include "scensim_netaddress_ipv4.h"

namespace ScenSim {

using std::string;
using std::ostringstream;
using std::vector;

class Ipv6NetworkAddress {
public:
    static const unsigned int numberBits = 128;

    static const unsigned int maxMulticastGroupNumber = 0xFFFFFF;

    static const Ipv6NetworkAddress anyAddress;
    static const Ipv6NetworkAddress broadcastAddress;
    static const Ipv6NetworkAddress invalidAddress;

    Ipv6NetworkAddress()
    {
        ipAddressHighBits = Ipv6NetworkAddress::ipv6AnyAddressHighBits;
        ipAddressLowBits = Ipv6NetworkAddress::ipv6AnyAddressLowBits;
    }

    explicit
    Ipv6NetworkAddress(const Ipv4NetworkAddress& ipv4Address)
        :
        ipAddressHighBits(Ipv6NetworkAddress::ipv6AnyAddressHighBits),
        ipAddressLowBits(ipv4Address.GetRawAddressLowBits())
    {
        if (ipv4Address == Ipv4NetworkAddress::broadcastAddress) {
            ipAddressLowBits = Ipv6NetworkAddress::ipv6BroadcastAddressLowBits;
            ipAddressHighBits = Ipv6NetworkAddress::ipv6BroadcastAddressHighBits;
        }//if//
    }


    explicit
    Ipv6NetworkAddress(const uint32_t initIpAddress)
        :
        ipAddressHighBits(Ipv6NetworkAddress::ipv6AnyAddressHighBits),
        ipAddressLowBits(initIpAddress)
    {
        if (initIpAddress == 0xFFFFFFFF) {
            ipAddressLowBits = Ipv6NetworkAddress::ipv6BroadcastAddressLowBits;
            ipAddressHighBits = Ipv6NetworkAddress::ipv6BroadcastAddressHighBits;
        }//if//
    }

    Ipv6NetworkAddress(const uint64_t& initIpAddressHighBits, const uint64_t& initIpAddressLowBits)
        :
        ipAddressHighBits(initIpAddressHighBits),
        ipAddressLowBits(initIpAddressLowBits)
    {}

    Ipv6NetworkAddress(const Ipv6NetworkAddress& subnetAddress, const Ipv6NetworkAddress& hostIdentifierOnlyAddress)
        :
        ipAddressHighBits(subnetAddress.ipAddressHighBits + hostIdentifierOnlyAddress.ipAddressHighBits),
        ipAddressLowBits(subnetAddress.ipAddressLowBits + hostIdentifierOnlyAddress.ipAddressLowBits)
    {}

    void SetAddressFromString(const string& stringToConvert, const NodeIdType& nodeId, bool& success);

    void SetAddressFromString(const string& stringToConvert, bool& success) {
        (*this).SetAddressFromString(stringToConvert, INVALID_NODEID, success);
    }

    string ConvertToString(const bool outputAsIpv4 = false) const;

    void SetToTheBroadcastAddress()
    {
        ipAddressHighBits = Ipv6NetworkAddress::ipv6BroadcastAddressHighBits;
        ipAddressLowBits = Ipv6NetworkAddress::ipv6BroadcastAddressLowBits;
    }

    static Ipv6NetworkAddress ReturnTheBroadcastAddress() {
        return Ipv6NetworkAddress(Ipv6NetworkAddress::ipv6BroadcastAddressHighBits, Ipv6NetworkAddress::ipv6BroadcastAddressLowBits);
    }

    static Ipv6NetworkAddress MakeABroadcastAddress(
        const Ipv6NetworkAddress& subnetAddress,
        const Ipv6NetworkAddress& subnetMask)
    {
        return Ipv6NetworkAddress((subnetAddress.ipAddressHighBits | ~subnetMask.ipAddressHighBits),
                (subnetAddress.ipAddressLowBits | ~subnetMask.ipAddressLowBits));
    }

    void SetToAnyAddress()
    {
        ipAddressHighBits = Ipv6NetworkAddress::ipv6AnyAddressHighBits;
        ipAddressLowBits = Ipv6NetworkAddress::ipv6AnyAddressLowBits;
    }

    static Ipv6NetworkAddress ReturnAnyAddress() {
        return Ipv6NetworkAddress(Ipv6NetworkAddress::ipv6AnyAddressHighBits, Ipv6NetworkAddress::ipv6AnyAddressLowBits);
    }

    bool IsTheBroadcastAddress() const
    {
        return ((ipAddressHighBits == Ipv6NetworkAddress::ipv6BroadcastAddressHighBits) &&
                (ipAddressLowBits == Ipv6NetworkAddress::ipv6BroadcastAddressLowBits));
    }

    bool IsABroadcastAddress(const Ipv6NetworkAddress& subnetMask) const
    {
        return (((ipAddressHighBits | subnetMask.ipAddressHighBits) == Ipv6NetworkAddress::ipv6BroadcastAddressHighBits) &&
                ((ipAddressLowBits | subnetMask.ipAddressLowBits) == Ipv6NetworkAddress::ipv6BroadcastAddressLowBits));
    }

    bool IsAMulticastAddress() const { return ((ipAddressHighBits & 0xFF00000000000000ULL) == 0xFF00000000000000ULL); }
    bool IsLinkLocalAddress() const { return ((ipAddressHighBits >> 54) == 0x3FA); }
    bool IsAnyAddress() const
    {
        return ((ipAddressHighBits == Ipv6NetworkAddress::ipv6AnyAddressHighBits) &&
                (ipAddressLowBits == Ipv6NetworkAddress::ipv6AnyAddressLowBits));
    }

    bool IsTheBroadcastOrAMulticastAddress() const {
        return (IsTheBroadcastAddress() || IsAMulticastAddress());
    }

    bool IsABroadcastOrAMulticastAddress(const Ipv6NetworkAddress& subnetMask) const {
        return (IsABroadcastAddress(subnetMask) || IsAMulticastAddress());
    }

    bool operator==(const Ipv6NetworkAddress& right) const
    {
        return (((*this).ipAddressHighBits == right.ipAddressHighBits) &&
            ((*this).ipAddressLowBits == right.ipAddressLowBits));
    }

    bool operator!=(const Ipv6NetworkAddress& right) const
    {
        return (((*this).ipAddressHighBits != right.ipAddressHighBits) ||
                ((*this).ipAddressLowBits != right.ipAddressLowBits));
    }

    bool operator<(const Ipv6NetworkAddress& right) const
    {
        return (((*this).ipAddressHighBits < right.ipAddressHighBits) ||
                (((*this).ipAddressHighBits == right.ipAddressHighBits) &&
                 ((*this).ipAddressLowBits < right.ipAddressLowBits)));
    }

    bool operator>(const Ipv6NetworkAddress& right) const
    {
        return (((*this).ipAddressHighBits > right.ipAddressHighBits) ||
                (((*this).ipAddressHighBits == right.ipAddressHighBits) &&
                 ((*this).ipAddressLowBits > right.ipAddressLowBits)));
    }

    uint64_t GetRawAddressLowBits() const { return ipAddressLowBits; }
    uint64_t GetRawAddressHighBits() const { return ipAddressHighBits; }

    uint32_t GetRawAddressLow32Bits() const { return static_cast<uint32_t>(ipAddressLowBits); }

    unsigned int GetMulticastGroupNumber() const
    {
        assert(IsAMulticastAddress());
        return (static_cast<unsigned int>(ipAddressLowBits & 0xFFFFFF));
    }


    void SetWith32BitRawAddress(const uint32_t newipAddress)
    {
        ipAddressHighBits = Ipv6NetworkAddress::ipv6AnyAddressHighBits;
        ipAddressLowBits = newipAddress;
    }

    bool IsInSameSubnetAs(const Ipv6NetworkAddress& address, const Ipv6NetworkAddress& subnetMask) const
    {
        return ((((ipAddressHighBits & subnetMask.ipAddressHighBits) == (address.ipAddressHighBits & subnetMask.ipAddressHighBits)) &&
                ((ipAddressLowBits & subnetMask.ipAddressLowBits) == (address.ipAddressLowBits & subnetMask.ipAddressLowBits))) ||
                (IsTheBroadcastAddress()));
    }

    Ipv6NetworkAddress MakeSubnetAddress(const Ipv6NetworkAddress& subnetMask) const {
        return Ipv6NetworkAddress((ipAddressHighBits & subnetMask.ipAddressHighBits),
               (ipAddressLowBits & subnetMask.ipAddressLowBits));
    }

    static Ipv6NetworkAddress MakeSubnetMask(const unsigned int numberPrefixBits);

    Ipv6NetworkAddress MakeAddressWithZeroedSubnetBits(const Ipv6NetworkAddress& subnetMask) const {
        return Ipv6NetworkAddress((ipAddressHighBits & ~subnetMask.ipAddressHighBits), (ipAddressLowBits & ~subnetMask.ipAddressLowBits));
    }

    static Ipv6NetworkAddress MakeAMulticastAddress(const unsigned int multicastGroupNumber);

    static bool IsIpv4StyleAddressString(const string& addressString) {
        return ((addressString.find('.')) != string::npos);
    }

private:
    //abstract(IPv4 style)
    static const uint64_t ipv6BroadcastAddressHighBits;
    static const uint64_t ipv6BroadcastAddressLowBits;

    static const uint64_t ipv6AnyAddressHighBits;
    static const uint64_t ipv6AnyAddressLowBits;

    uint64_t ipAddressHighBits;
    uint64_t ipAddressLowBits;

    void SetAddressFromIpv4StyleString(
        const string& stringToConvert, const NodeIdType& nodeId, bool& success);

};//Ipv6NetworkAddress//


inline
string Ipv6NetworkAddress::ConvertToString(const bool outputAsIpv4) const
{
    if (outputAsIpv4) {
        return (Ipv4NetworkAddress(GetRawAddressLow32Bits()).ConvertToString());
    }//if//

    const uint64_t mask = 0xFFFF;

    ostringstream outstream;
    outstream << std::hex;

    outstream << (ipAddressHighBits / 0x0001000000000000LL) << ':'
              << ((ipAddressHighBits / 0x0000000100000000LL) & mask) << ':'
              << ((ipAddressHighBits / 0x0000000000001000LL) & mask) << ':'
              << (ipAddressHighBits & mask) << ':'
              << (ipAddressLowBits / 0x0001000000000000LL) << ':'
              << ((ipAddressLowBits / 0x0000000100000000LL) & mask) << ':'
              << ((ipAddressLowBits / 0x0000000000001000LL) & mask) << ':'
              << (ipAddressLowBits & mask);

    return (outstream.str());

}//ConvertToString//


inline
Ipv6NetworkAddress Ipv6NetworkAddress::MakeSubnetMask(const unsigned int numberPrefixBits)
{
    assert(numberPrefixBits >= 0);
    assert(numberPrefixBits <= numberBits);

    uint64_t maskHighBits = 0;
    uint64_t maskLowBits = 0;

    if (numberPrefixBits <= 64) {
        for(unsigned int i = 0; i < numberPrefixBits; i++) {
            maskHighBits = (maskHighBits << 1);
            maskHighBits = (maskHighBits | 0x1);
        }//for//

        maskHighBits = (maskHighBits << (64 - numberPrefixBits));

    }
    else {

        maskHighBits = 0xFFFFFFFFFFFFFFFFULL;

        for(unsigned int i = 0; i < (numberPrefixBits - 64); i++) {
            maskLowBits = (maskLowBits << 1);
            maskLowBits = (maskLowBits | 0x1);
        }//for//

        maskLowBits = (maskLowBits << (128 - numberPrefixBits));

    }//if//

    return (Ipv6NetworkAddress(maskHighBits, maskLowBits));

}///MakeSubnetMask//


inline
void Ipv6NetworkAddress::SetAddressFromIpv4StyleString(
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

    ipAddressHighBits = Ipv6NetworkAddress::ipv6AnyAddressHighBits;

    ipAddressLowBits =
        (parts[0] * (256 * 256 * 256)) +
        (parts[1] * (256 * 256)) +
        (parts[2] * 256) + parts[3];

    if (nodeIdShouldBeAddedToAddress) {
        if (nodeId == INVALID_NODEID) {
            // Illegal Node ID given no "$n" allowed.
            success = false;
            return;

        }//if//

        ipAddressLowBits += nodeId;
    }//if//

    success = true;

}//SetAddressFromIpv4StyleString//


inline
void Ipv6NetworkAddress::SetAddressFromString(
    const string& stringToConvert,
    const NodeIdType& nodeId,
    bool& success)
{

    //IPv4 style
    if (Ipv6NetworkAddress::IsIpv4StyleAddressString(stringToConvert)) {
        (*this).SetAddressFromIpv4StyleString(stringToConvert, nodeId, success);
        return;
    }//if//


    //Example IP Format: 2001:db8:0:1:1:1:1:1
    //Example IP Format: 2001:db8:0:0:0:0:0:0+$n
    //currently, not support 2001:db8::

    success = false;

    bool nodeIdShouldBeAddedToAddress = false;

    const int numberParts = 8;
    vector<unsigned char> parts;

    size_t currentPos = 0;

    for(int i = 0; i < numberParts; i++) {

        size_t colonPos = stringToConvert.find(':',currentPos);

        bool colonWasFound = (colonPos != string::npos);

        if (i == (numberParts - 1)) {
            // Last part

            if (colonWasFound) {
                // Too many dots: failure
                return;
            }//if//

            // Pretend there is a terminating dot after number

            colonPos = stringToConvert.find_first_not_of("0123456789abcdef", currentPos);
            if (colonPos == string::npos) {
                colonPos = stringToConvert.length();
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
            if (!colonWasFound) {
                return;

            }//if//
        }//if//

        const size_t fieldLength = (colonPos - currentPos);

        string hexString = stringToConvert.substr(currentPos, fieldLength);


        ConvertStringToUpperCase(hexString);
        AddLeadingZeros(hexString, 4);

        assert(hexString.size() == 4);
        parts.push_back(ConvertTwoHexCharactersToByte(hexString[0], hexString[1]));
        parts.push_back(ConvertTwoHexCharactersToByte(hexString[2], hexString[3]));

        currentPos = colonPos + 1;

    }//for//

    if (parts.size() != (numberParts * 2)) {
        success = false;
        return;
    }//if//

    ipAddressHighBits =
        (parts[0] * 0x0100000000000000ULL) +
        (parts[1] * 0x0001000000000000ULL) +
        (parts[2] * 0x0000010000000000ULL) +
        (parts[3] * 0x0000000100000000ULL) +
        (parts[4] * 0x0000000001000000ULL) +
        (parts[5] * 0x0000000000010000ULL) +
        (parts[6] * 0x0000000000000100ULL) +
        parts[7];

    ipAddressLowBits =
        (parts[8] * 0x0100000000000000ULL) +
        (parts[9] * 0x0001000000000000ULL) +
        (parts[10] * 0x0000010000000000ULL) +
        (parts[11] * 0x0000000100000000ULL) +
        (parts[12] * 0x0000000001000000ULL) +
        (parts[13] * 0x0000000000010000ULL) +
        (parts[14] * 0x0000000000000100ULL) +
        parts[15];

    if (nodeIdShouldBeAddedToAddress) {
        if (nodeId == INVALID_NODEID) {
            // Illegal Node ID given no "$n" allowed.
            success = false;
            return;

        }//if//

        ipAddressLowBits += nodeId;
    }//if//

    success = true;

}//SetAddressFromString//


inline
Ipv6NetworkAddress Ipv6NetworkAddress::MakeAMulticastAddress(const unsigned int multicastGroupNumber)
{
    const uint64_t aMulticastPrefixSubnet = 0xFF30000000000000ULL;
    return (Ipv6NetworkAddress(aMulticastPrefixSubnet, static_cast<uint64_t>(multicastGroupNumber)));
}


}//namespace//

#endif


