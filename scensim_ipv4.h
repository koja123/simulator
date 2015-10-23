// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_IPV4_H
#define SCENSIM_IPV4_H

#ifndef SCENSIM_IP_MODEL_H
#error "Include scensim_ip_model.h not scensim_ipv4.h (indirect include only)"
#endif

#include <string>
#include <array>

#include <stdint.h>
#include "scensim_nodeid.h"
#include "scensim_netaddress.h"
#include "scensim_mac.h"

namespace ScenSim {

const unsigned short int NULL_FLOW_LABEL = 0;

struct IpIcmpHeaderType {
    uint8_t icmpType;
    uint8_t icmpCode;
    uint16_t checkSum_notused;

    IpIcmpHeaderType() : icmpType(0), icmpCode(0), checkSum_notused(0) {}
};

// Does not exist in Ipv4

struct IpRouterSolicitationMessage {
    static const int icmpTypeNum = 0;

    IpIcmpHeaderType icmpHeader;
    uint32_t reserved;

    IpRouterSolicitationMessage(): reserved(0) { }
};//IpRouterSolicitationMessage//


struct IpRouterAdvertisementMessage {
    static const int icmpTypeNum = 0;

    IpIcmpHeaderType icmpHeader;
    uint32_t notUsed1[3];
    uint16_t notUsed2;
    uint16_t prefixLength;
    uint32_t notUsed3[3];
    uint64_t ipPrefixHighBits;
    uint64_t ipPrefixLowBits;
    uint64_t linkLayerAddressOption_notused;

};//IpRouterAdvertisementMessage//


struct IpNeighborSolicitationMessage {
    static const int icmpTypeNum = 0;

    IpIcmpHeaderType icmpHeader;
    uint32_t reserved;
    uint64_t targetAddressHighBits;
    uint64_t targetAddressLowBits;
    SixByteMacAddressType linkLayerAddressOption;

    IpNeighborSolicitationMessage()
        :
        icmpHeader(),
        reserved(0),
        targetAddressHighBits(0),
        targetAddressLowBits(0)
    {}

};//IpNeighborSolicitationMessage//


struct IpNeighborAdvertisementMessage {
    static const int icmpTypeNum = 0;

    IpIcmpHeaderType icmpHeader;
    uint32_t reserved;
    uint64_t targetAddressHighBits;
    uint64_t targetAddressLowBits;
    SixByteMacAddressType linkLayerAddressOption;

    IpNeighborAdvertisementMessage()
        :
        icmpHeader(),
        reserved(0),
        targetAddressHighBits(0),
        targetAddressLowBits(0)
    {}

};//IpNeighborAdvertisementMessage//


// Note: Header struct definition has general Mobility Header in front.

struct MobileIpBindingUpdateExtensionHeader {
    unsigned char nextHeaderTypeCode;
    unsigned char headerExtensionLengthIn8ByteUnitsMinus1;
    unsigned char mobilityHeaderTypeCode;
    unsigned char reserved;
    unsigned short checksum_notused;
    unsigned short sequenceNumber;
    unsigned short homeBit:14;
    unsigned short bindingId:2;          // Non-standard Extension (should be in option header)
    unsigned short lifetimein4SecUnits;
    // Alternative Care Of Address Option

    unsigned short padding;
    unsigned char mobilityMessageOptionTypeCode_notused;
    unsigned char mobilityMessageOptionLength_notused;
    uint64_t careOfAddressHighBits_notused;
    uint64_t careOfAddressLowBits_notused;

    static const unsigned char messageTypeCode = 5;

};//BindingUpdateMobileIpExtensionHeader//


class IpHeaderModel {
public:
    static const bool usingVersion6 = false;

    static const int ipInIpProtocolNumber = 41;
    static const int ipProtoNoneProtocolNumber = 59;

    IpHeaderModel(
        const unsigned char trafficClass,
        const unsigned int payloadLengthBeforeIp,
        const unsigned char hopLimit,
        const unsigned char nextHeaderTypeCode,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress);

    const unsigned char* GetPointerToRawBytes() const { return (&rawBytes[0]); }
    unsigned int GetNumberOfRawBytes() const { return rawBytesLength; }
    unsigned int GetNumberOfTrailingBytes() { return 0; }

    unsigned char GetNextHeaderProtocolCode() const { assert(false && "Not in IPv4"); abort(); }
    void SetFinalNextHeaderProtocolCode(const unsigned char nextHeader) { assert(false && "Not in IPv4"); abort(); }

    void AddBindingUpdateExtensionHeader(
        const unsigned short sequenceNumber,
        const unsigned short lifetimein4SecUnits,
        const unsigned short bindingId) { assert(false && "Not in IPv4"); }

    void AddHomeAddressDestinationOptionsHeader(const NetworkAddress& homeAddress) { assert(false && "Not in IPv4"); }

    bool HasIpsecEspOverhead() const { return false; }
    void AddIpsecEspOverhead() { assert(false && "Not in IPv4"); }

private:
    friend class IpHeaderOverlayModel;
    friend bool IsAnIpExtensionHeaderCode(const unsigned char);


    static const unsigned char mobilityExtensionHeaderTypeCode = 135;
    static const unsigned char destinationOptionsTypeCode = 60;

    static const unsigned char pad1OptionTypeCode = 0;

    struct ExtensionHeaderTypeAndSizeOverlay {
        unsigned char nextHeaderTypeCode;
        unsigned char headerExtensionLengthIn8ByteUnitsMinus1;
    };

    struct HomeAddressDestinationOptionsHeader {
        unsigned char nextHeaderTypeCode;
        unsigned char headerExtensionLengthIn8ByteUnitsMinus1;
        unsigned short padding1;
        unsigned short padding2;
        unsigned char optionType;
        unsigned char optionLengthBytes;
        uint64_t homeAddressHighBits;
        uint64_t homeAddressLowBits;

        static const int typeCode = 201;
        static const int optionLengthBytesValue = 16;

    };//HomeAddressDestinationOptionsHeader//


    struct Ipv4HeaderType {
        // Raw Fields

        unsigned char versionAndLength;
        unsigned char tos;
        unsigned short int totalLength;

        unsigned short int filler2;
        unsigned short int filler3;

        unsigned char ttl;
        unsigned char protocol;
        unsigned short int checksum;

        uint32_t sourceAddress;
        uint32_t destinationAddress;
    };//Ipv4HeaderType//

    static void SetChecksum(Ipv4HeaderType& ipHeader)
    {
        ipHeader.checksum = 0;

        const uint16_t *buffer = reinterpret_cast<uint16_t*>(&ipHeader);
        unsigned int length = sizeof(ipHeader);
        unsigned long sum = 0;

        while (length > 1) {
            sum += *buffer++;
            length -= sizeof(*buffer);
        }//while//

        if (length == 1) {
            sum += *reinterpret_cast<const uint8_t*>(buffer);
            length -= sizeof(uint8_t);
        }//if//

        assert(length == 0);
        sum  = (sum & 0xffff) + (sum >> 16);
        sum  = (sum & 0xffff) + (sum >> 16);
        ipHeader.checksum = ~static_cast<unsigned short int>(sum);

    }//SetChecksum//

    static const unsigned int maxHeaderSizeBytes = 64;

    std::array<unsigned char, maxHeaderSizeBytes> rawBytes;
    unsigned int rawBytesLength;

};//IpHeaderModel//


inline
IpHeaderModel::IpHeaderModel(
    const unsigned char trafficClass,
    const unsigned int payloadLength,
    const unsigned char hopLimit,
    const unsigned char nextHeaderTypeCode,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress)
{
    rawBytesLength = sizeof(Ipv4HeaderType);
    IpHeaderModel::Ipv4HeaderType& ipHeader = *reinterpret_cast<Ipv4HeaderType*>(&rawBytes[0]);

    ipHeader.versionAndLength = (4 << 4) | 5;
    ipHeader.tos = trafficClass;
    ipHeader.totalLength = HostToNet16(static_cast<unsigned short int>(sizeof(Ipv4HeaderType) + payloadLength));
    ipHeader.filler2 = 0;
    ipHeader.filler3 = 0;
    ipHeader.ttl = hopLimit;
    ipHeader.protocol = nextHeaderTypeCode;
    ipHeader.checksum = 0;
    ipHeader.sourceAddress = HostToNet32(sourceAddress.GetRawAddressLow32Bits());
    ipHeader.destinationAddress = HostToNet32(destinationAddress.GetRawAddressLow32Bits());

    SetChecksum(ipHeader);

}//IpHeaderModel()//



class IpHeaderOverlayModel {
public:

    IpHeaderOverlayModel(const unsigned char* initHeaderPtr, const unsigned int initPacketLength)
        :
        headerIsReadOnly(true),
        headerPtr(const_cast<unsigned char*>(initHeaderPtr)), packetLength(initPacketLength) {}

    IpHeaderOverlayModel(unsigned char* initHeaderPtr, const unsigned int initPacketLength)
        : headerIsReadOnly(false), headerPtr(initHeaderPtr), packetLength(initPacketLength) {}

    void StopOverlayingHeader() const;

    void GetHeaderTotalLengthAndNextHeaderProtocolCode(
        unsigned int& headerLength,
        unsigned char& protocolCode) const;

    unsigned int GetLength() const;

    unsigned char GetTrafficClass() const;
    unsigned short int GetFlowLabel() const { assert(false && "No Flow label in IPv4"); abort(); return 0; }
    unsigned char GetNextHeaderProtocolCode() const;

    unsigned char GetHopLimit() const;
    NetworkAddress GetSourceAddress() const;
    NetworkAddress GetDestinationAddress() const;

    void SetTrafficClass(const unsigned char trafficClass);
    void SetFlowLabel(unsigned short int flowLabel) { assert(false && "No Flow label in IPv4"); abort(); }
    void SetHopLimit(const unsigned char hopLimit);
    void SetSourceAddress(const NetworkAddress& sourceAddress);
    void SetDestinationAddress(const NetworkAddress& destinationAddress);

    bool MobilityExtensionHeaderExists() const { return false; }

    bool MobileIpBindingUpdateHeaderExists() const { return false; }
    const MobileIpBindingUpdateExtensionHeader& GetMobileIpBindingUpdateHeader() const
        { assert(false); abort(); return (*(MobileIpBindingUpdateExtensionHeader*)nullptr); }

    bool HomeAddressDestinationOptionsHeaderExists() const { return false; }
    NetworkAddress GetHomeAddressFromDestinationOptionsHeader() const
        { assert(false); abort(); return NetworkAddress::invalidAddress; }

private:

    typedef IpHeaderModel::Ipv4HeaderType Ipv4HeaderType;

    unsigned char* headerPtr;
    unsigned int packetLength;

    bool headerIsReadOnly;

};//IpHeaderOverlayModel//


// This method is a aid for code correctness.  When the source packet is changed
// (such as removing the header), this routine should be called.

inline
void IpHeaderOverlayModel::StopOverlayingHeader() const
{
    const_cast<IpHeaderOverlayModel*>(this)->headerPtr = nullptr;
}



inline
bool IsAnIpOptionHeaderCode(const unsigned char optionHeaderTypeCode)
{
    return false;
}//IsAnIpOptionHeaderCode//



inline
void IpHeaderOverlayModel::GetHeaderTotalLengthAndNextHeaderProtocolCode(
    unsigned int& headerLength,
    unsigned char& protocolCode) const
{
    Ipv4HeaderType& ipHeader = *reinterpret_cast<Ipv4HeaderType*>(headerPtr);

    headerLength = sizeof(Ipv4HeaderType);
    protocolCode = ipHeader.protocol;
}//GetHeaderTotalLengthAndNextHeaderProtocolCode//


inline
unsigned int IpHeaderOverlayModel::GetLength() const
{
    unsigned int length;
    unsigned char notUsed;

    GetHeaderTotalLengthAndNextHeaderProtocolCode(length, notUsed);

    return length;
}//GetIpHeaderLength//


inline
unsigned char IpHeaderOverlayModel::GetNextHeaderProtocolCode() const
{
    unsigned int notUsed;
    unsigned char protocolCode;

    GetHeaderTotalLengthAndNextHeaderProtocolCode(notUsed, protocolCode);

    return protocolCode;
}

inline
unsigned char IpHeaderOverlayModel::GetTrafficClass() const
{
    assert(headerPtr != nullptr);
    Ipv4HeaderType& ipHeader = *reinterpret_cast<Ipv4HeaderType*>(headerPtr);
    return (ipHeader.tos);
}


inline
NetworkAddress IpHeaderOverlayModel::GetSourceAddress() const
{
    assert(headerPtr != nullptr);
    Ipv4HeaderType& ipHeader = *reinterpret_cast<Ipv4HeaderType*>(headerPtr);
    return (NetworkAddress(NetToHost32(ipHeader.sourceAddress)));
}

inline
NetworkAddress IpHeaderOverlayModel::GetDestinationAddress() const
{
    assert(headerPtr != nullptr);
    Ipv4HeaderType& ipHeader = *reinterpret_cast<Ipv4HeaderType*>(headerPtr);
    return (NetworkAddress(NetToHost32(ipHeader.destinationAddress)));
}

inline
unsigned char IpHeaderOverlayModel::GetHopLimit() const
{
    assert(headerPtr != nullptr);
    Ipv4HeaderType& ipHeader = *reinterpret_cast<Ipv4HeaderType*>(headerPtr);
    return (ipHeader.ttl);
}

inline
void IpHeaderOverlayModel::SetHopLimit(const unsigned char hopLimit)
{
    assert(headerPtr != nullptr);
    Ipv4HeaderType& ipHeader = *reinterpret_cast<Ipv4HeaderType*>(headerPtr);

    ipHeader.ttl = hopLimit;

    IpHeaderModel::SetChecksum(ipHeader);
}

inline
void IpHeaderOverlayModel::SetTrafficClass(const unsigned char trafficClass)
{
    assert(headerPtr != nullptr);
    Ipv4HeaderType& ipHeader = *reinterpret_cast<Ipv4HeaderType*>(headerPtr);

    ipHeader.tos = trafficClass;

    IpHeaderModel::SetChecksum(ipHeader);
}

inline
void IpHeaderOverlayModel::SetSourceAddress(const NetworkAddress& sourceAddress)
{
    assert(headerPtr != nullptr);
    Ipv4HeaderType& ipHeader = *reinterpret_cast<Ipv4HeaderType*>(headerPtr);

    ipHeader.sourceAddress = HostToNet32(static_cast<uint32_t>(sourceAddress.GetRawAddressLowBits()));

    IpHeaderModel::SetChecksum(ipHeader);
}

inline
void IpHeaderOverlayModel::SetDestinationAddress(const NetworkAddress& destinationAddress)
{
    assert(headerPtr != nullptr);
    Ipv4HeaderType& ipHeader = *reinterpret_cast<Ipv4HeaderType*>(headerPtr);

    ipHeader.destinationAddress = HostToNet32(static_cast<uint32_t>(destinationAddress.GetRawAddressLowBits()));

    IpHeaderModel::SetChecksum(ipHeader);
}

}//namespace//


#endif

