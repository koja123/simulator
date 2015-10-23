// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_IPV6_H
#define SCENSIM_IPV6_H

#ifndef SCENSIM_IP_MODEL_H
#error "Include scensim_ip_model.h not scensim_ipv6.h (indirect include only)"
#endif

#include <string>
#include <array>

#include <stdint.h>
#include "scensim_nodeid.h"
#include "scensim_netaddress.h"
#include "scensim_support.h"
#include "scensim_mac.h"

namespace ScenSim {

const unsigned short int NULL_FLOW_LABEL = 0;

struct IpIcmpHeaderType {
    uint8_t icmpType;
    uint8_t icmpCode;
    uint16_t checkSum_notused;

    IpIcmpHeaderType() : icmpType(0), icmpCode(0), checkSum_notused(0) {}
};


struct IpRouterSolicitationMessage {
    static const int icmpTypeNum = 133;

    IpIcmpHeaderType icmpHeader;
    uint32_t reserved;

    IpRouterSolicitationMessage(): reserved(0) { }
};//IpRouterSolicitationMessage//


struct IpRouterAdvertisementMessage {
    static const int icmpTypeNum = 134;

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
    static const int icmpTypeNum = 135;

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
    static const int icmpTypeNum = 136;

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


// Mobile IP was included in IPv6 standard.

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
    static const bool usingVersion6 = true;

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
    unsigned int GetNumberOfTrailingBytes();

    unsigned char GetNextHeaderProtocolCode() const;
    void SetFinalNextHeaderProtocolCode(const unsigned char nextHeader);

    void AddBindingUpdateExtensionHeader(
        const unsigned short sequenceNumber,
        const unsigned short lifetimein4SecUnits,
        const unsigned short bindingId);

    void AddHomeAddressDestinationOptionsHeader(const NetworkAddress& homeAddress);

    bool HasIpsecEspOverhead() const { return hasIpsecEspHeaders; }
    void AddIpsecEspOverhead();

private:
    friend class IpHeaderOverlayModel;
    friend bool IsAnIpExtensionHeaderCode(const unsigned char);

    static const unsigned int ipsecTrailingHeaderBytes = 2;

    static const unsigned char mobilityExtensionHeaderTypeCode = 135;
    static const unsigned char destinationOptionsTypeCode = 60;

    static const unsigned char pad1OptionTypeCode = 0;

    struct ExtensionHeaderTypeAndSizeOverlay {
        unsigned char nextHeaderTypeCode;
        unsigned char headerExtensionLengthIn8ByteUnitsMinus1;
    };

    // Only used for the size, i.e. for explaination only:

    struct IpsecEspAbstractOverheadHeader {
        uint32_t securityAssociationId_notused;
        uint32_t sequenceNumber_notused;
        uint64_t authenticationData_notused[2];
        uint64_t initializationVector_notused;
        static const int typeCode = 50;
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


    struct Ipv6HeaderType {

        // Raw Fields
        unsigned char versionAndTrafficClassAndFlowLabel[4];

        unsigned short int payloadLength;
        unsigned char nextHeaderTypeCode;
        unsigned char hopLimit;

        uint64_t sourceAddressHighBits;
        uint64_t sourceAddressLowBits;
        uint64_t destinationAddressHighBits;
        uint64_t destinationAddressLowBits;

        void SetVersion(unsigned char version) {
            assert(version <= 0x0F);
            versionAndTrafficClassAndFlowLabel[0] &= 0x0F;
            versionAndTrafficClassAndFlowLabel[0] |= ((version << 4) & 0xF0);
        }//SetVersion//

        unsigned char GetVersion() const {
            return ((versionAndTrafficClassAndFlowLabel[0] >> 4) & 0x0F);
        }//GetVersion//

        void SetTrafficClass(unsigned char trafficClass) {
            versionAndTrafficClassAndFlowLabel[0] &= 0xF0;
            versionAndTrafficClassAndFlowLabel[0] |= (trafficClass >> 4) & 0x0F;
            versionAndTrafficClassAndFlowLabel[1] &= 0x0F;
            versionAndTrafficClassAndFlowLabel[1] |= (trafficClass << 4) & 0xF0;
        }//SetTrafficClass//

        unsigned char GetTrafficClass() const {
            return
                (((versionAndTrafficClassAndFlowLabel[0] << 4) & 0xF0) |
                ((versionAndTrafficClassAndFlowLabel[1] >> 4) & 0x0F));
        }//GetTrafficClass//

        void SetFlowLabel(const uint32_t flowLabel) {
            assert(flowLabel <= 0xFFFFF);
            versionAndTrafficClassAndFlowLabel[1] &= 0xF0;
            versionAndTrafficClassAndFlowLabel[1] |= ((flowLabel >> 16) & 0x0F);
            versionAndTrafficClassAndFlowLabel[2] = ((flowLabel >> 8) & 0xFF);
            versionAndTrafficClassAndFlowLabel[3] = (flowLabel & 0xFF);
        }//SetFlowLabel//

        uint32_t GetFlowLabel() const {
            return
                (((versionAndTrafficClassAndFlowLabel[1] << 16) & 0xF0000) |
                ((versionAndTrafficClassAndFlowLabel[2] << 8) & 0xFF00) |
                (versionAndTrafficClassAndFlowLabel[3] & 0xFF));
        }//GetFlowLabel//

    };//Ipv6HeaderType//

    static const unsigned int maxHeaderSizeBytes = 512;

    std::array<unsigned char, maxHeaderSizeBytes> rawBytes;
    unsigned int rawBytesLength;

    const static unsigned int ipHeaderNextHeaderTypeCodePosition = 6;
    unsigned int lastNextHeaderTypeCodePosition;

    unsigned int payloadLengthBeforeIpHeaders;

    bool hasIpsecEspHeaders;

    void SetPayloadLengthField();

};//IpHeaderModel//


inline
IpHeaderModel::IpHeaderModel(
    const unsigned char trafficClass,
    const unsigned int initPayloadLengthBeforeIpHeaders,
    const unsigned char hopLimit,
    const unsigned char nextHeaderTypeCode,
    const NetworkAddress& sourceAddress,
    const NetworkAddress& destinationAddress)
    :
    payloadLengthBeforeIpHeaders(initPayloadLengthBeforeIpHeaders),
    hasIpsecEspHeaders(false)
{
    rawBytesLength = sizeof(Ipv6HeaderType);
    lastNextHeaderTypeCodePosition = ipHeaderNextHeaderTypeCodePosition;

    Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(&rawBytes[0]);

    ipHeader.SetVersion(6);
    ipHeader.SetTrafficClass(trafficClass);
    ipHeader.payloadLength = HostToNet16(0); // Unknown.
    ipHeader.nextHeaderTypeCode = nextHeaderTypeCode;

    ipHeader.SetFlowLabel(NULL_FLOW_LABEL);
    ipHeader.hopLimit = hopLimit;
    ConvertTwoHost64ToTwoNet64(
        sourceAddress.GetRawAddressHighBits(),
        sourceAddress.GetRawAddressLowBits(),
        ipHeader.sourceAddressHighBits,
        ipHeader.sourceAddressLowBits);
    ConvertTwoHost64ToTwoNet64(
        destinationAddress.GetRawAddressHighBits(),
        destinationAddress.GetRawAddressLowBits(),
        ipHeader.destinationAddressHighBits,
        ipHeader.destinationAddressLowBits);

    (*this).SetPayloadLengthField();

}//IpHeaderModel()



inline
unsigned int IpHeaderModel::GetNumberOfTrailingBytes()
{
    unsigned int trailingBytes = 0;
    if (hasIpsecEspHeaders) {
        trailingBytes = ipsecTrailingHeaderBytes;
    }//if//

    return (
        RoundUpToNearestIntDivisibleBy8(
            (payloadLengthBeforeIpHeaders + trailingBytes) - payloadLengthBeforeIpHeaders));
}

inline
void IpHeaderModel::SetPayloadLengthField()
{
    assert(rawBytesLength % 8 == 0);

    IpHeaderModel::Ipv6HeaderType& ipHeader = *reinterpret_cast<IpHeaderModel::Ipv6HeaderType*>(&rawBytes[0]);

    ipHeader.payloadLength = HostToNet16(
        static_cast<unsigned short>(((rawBytesLength - sizeof(Ipv6HeaderType)) +
            payloadLengthBeforeIpHeaders +
            GetNumberOfTrailingBytes())));
}

inline
void IpHeaderModel::SetFinalNextHeaderProtocolCode(const unsigned char nextHeaderTypeCode)
{
    rawBytes[lastNextHeaderTypeCodePosition] = nextHeaderTypeCode;
}

inline
unsigned char IpHeaderModel::GetNextHeaderProtocolCode() const
{
    return (rawBytes[lastNextHeaderTypeCodePosition]);
}


inline
void IpHeaderModel::AddIpsecEspOverhead()
{
    (*this).hasIpsecEspHeaders = true;

    // Add in fake generic "standard" header instread of ESP monstrosity.

    ExtensionHeaderTypeAndSizeOverlay& fakedGenericHeader =
        *reinterpret_cast<ExtensionHeaderTypeAndSizeOverlay*>(&rawBytes[rawBytesLength]);

    fakedGenericHeader.nextHeaderTypeCode = rawBytes[lastNextHeaderTypeCodePosition];
    fakedGenericHeader.headerExtensionLengthIn8ByteUnitsMinus1 =
        (sizeof(IpsecEspAbstractOverheadHeader)/8 - 1);

    rawBytes[lastNextHeaderTypeCodePosition] = IpsecEspAbstractOverheadHeader::typeCode;
    lastNextHeaderTypeCodePosition = rawBytesLength;
    rawBytesLength += sizeof(IpsecEspAbstractOverheadHeader);


    (*this).SetPayloadLengthField();

}//AddIpsecEspHeaderOverhead//



inline
void IpHeaderModel::AddHomeAddressDestinationOptionsHeader(const NetworkAddress& homeAddress)
{
    assert(rawBytesLength % 8 == 0);

    HomeAddressDestinationOptionsHeader& homeAddressHeader =
        *reinterpret_cast<HomeAddressDestinationOptionsHeader*>(&rawBytes[rawBytesLength]);

    homeAddressHeader.padding1 = 0;
    homeAddressHeader.padding2 = 0;
    homeAddressHeader.nextHeaderTypeCode = rawBytes[lastNextHeaderTypeCodePosition];

    homeAddressHeader.headerExtensionLengthIn8ByteUnitsMinus1 =
        (sizeof(HomeAddressDestinationOptionsHeader)/8 - 1);
    homeAddressHeader.optionType = HomeAddressDestinationOptionsHeader::typeCode;
    homeAddressHeader.optionLengthBytes = HomeAddressDestinationOptionsHeader::optionLengthBytesValue;
    homeAddressHeader.homeAddressHighBits = homeAddress.GetRawAddressHighBits();
    homeAddressHeader.homeAddressLowBits = homeAddress.GetRawAddressLowBits();

    rawBytes[lastNextHeaderTypeCodePosition] = destinationOptionsTypeCode;
    lastNextHeaderTypeCodePosition = rawBytesLength;

    rawBytesLength += sizeof(homeAddressHeader);

    (*this).SetPayloadLengthField();

}//AddHomeAddressDestinationOptionsHeader//



inline
void IpHeaderModel::AddBindingUpdateExtensionHeader(
    const unsigned short sequenceNumber,
    const unsigned short lifetimein4SecUnits,
    const unsigned short bindingId)
{
    assert(sizeof(MobileIpBindingUpdateExtensionHeader) == 32);

    MobileIpBindingUpdateExtensionHeader& bindingUpdateHeaderOverlay =
        *reinterpret_cast<MobileIpBindingUpdateExtensionHeader*>(&rawBytes[rawBytesLength]);

    bindingUpdateHeaderOverlay.nextHeaderTypeCode = GetNextHeaderProtocolCode();

    rawBytes[lastNextHeaderTypeCodePosition] = mobilityExtensionHeaderTypeCode;
    lastNextHeaderTypeCodePosition = rawBytesLength;

    rawBytesLength += sizeof(MobileIpBindingUpdateExtensionHeader);

    bindingUpdateHeaderOverlay.homeBit = 1;
    bindingUpdateHeaderOverlay.bindingId = bindingId;
    bindingUpdateHeaderOverlay.headerExtensionLengthIn8ByteUnitsMinus1 =
        (sizeof(MobileIpBindingUpdateExtensionHeader) / 8) - 1;
    bindingUpdateHeaderOverlay.lifetimein4SecUnits = lifetimein4SecUnits;
    bindingUpdateHeaderOverlay.mobilityHeaderTypeCode = MobileIpBindingUpdateExtensionHeader::messageTypeCode;
    bindingUpdateHeaderOverlay.sequenceNumber = sequenceNumber;

    bindingUpdateHeaderOverlay.padding = 0;
    bindingUpdateHeaderOverlay.reserved = 0;
    bindingUpdateHeaderOverlay.careOfAddressHighBits_notused = 0;
    bindingUpdateHeaderOverlay.careOfAddressLowBits_notused = 0;
    bindingUpdateHeaderOverlay.checksum_notused = 0;
    bindingUpdateHeaderOverlay.mobilityMessageOptionTypeCode_notused = 0;

    (*this).SetPayloadLengthField();

}//AddBindingUpdateExtensionHeader//


//--------------------------------------------------------------------------------------------------

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
    unsigned short int GetFlowLabel() const;
    unsigned char GetNextHeaderProtocolCode() const;

    unsigned char GetHopLimit() const;
    NetworkAddress GetSourceAddress() const;
    NetworkAddress GetDestinationAddress() const;

    void SetTrafficClass(const unsigned char trafficClass);
    void SetFlowLabel(const unsigned short int flowLabel);
    void SetHopLimit(const unsigned char hopLimit);
    void SetSourceAddress(const NetworkAddress& sourceAddress);
    void SetDestinationAddress(const NetworkAddress& destinationAddress);

    bool MobilityExtensionHeaderExists() const;

    bool MobileIpBindingUpdateHeaderExists() const;
    const MobileIpBindingUpdateExtensionHeader& GetMobileIpBindingUpdateHeader() const;

    bool HomeAddressDestinationOptionsHeaderExists() const;
    NetworkAddress GetHomeAddressFromDestinationOptionsHeader() const;

private:

    typedef IpHeaderModel::Ipv6HeaderType Ipv6HeaderType;
    typedef IpHeaderModel::ExtensionHeaderTypeAndSizeOverlay ExtensionHeaderTypeAndSizeOverlay;

    unsigned char* headerPtr;
    unsigned int packetLength;

    bool headerIsReadOnly;

    void FindExtensionHeaderWithTypeCode(
        const unsigned char typeCode,
        bool& wasFound,
        unsigned int& position) const;

};//IpHeaderOverlayModel//


// This method is a aid for code correctness.  When the source packet is changed
// (such as removing the header), this routine should be called.

inline
void IpHeaderOverlayModel::StopOverlayingHeader() const
{
    const_cast<IpHeaderOverlayModel*>(this)->headerPtr = nullptr;
}


inline
bool IsAnIpExtensionHeaderCode(const unsigned char extensionHeaderTypeCode)
{
    return (
        (extensionHeaderTypeCode == IpHeaderModel::mobilityExtensionHeaderTypeCode) ||
        (extensionHeaderTypeCode == IpHeaderModel::destinationOptionsTypeCode) ||
        (extensionHeaderTypeCode == IpHeaderModel::IpsecEspAbstractOverheadHeader::typeCode));

}//IsAnIpExtensionHeaderCode//


inline
void IpHeaderOverlayModel::FindExtensionHeaderWithTypeCode(
    const unsigned char headerTypeCode,
    bool& wasFound,
    unsigned int& position) const
{
    assert(headerPtr != nullptr);

    IpHeaderModel::Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(headerPtr);

    if (ipHeader.nextHeaderTypeCode == headerTypeCode) {
        wasFound = true;
        position = sizeof(Ipv6HeaderType);
        return;
    }//if//

    if (!IsAnIpExtensionHeaderCode(ipHeader.nextHeaderTypeCode)) {
        wasFound = false;
        return;
    }//if//

    unsigned int pos = sizeof(Ipv6HeaderType);
    while (true) {
        const ExtensionHeaderTypeAndSizeOverlay& extensionHeader =
            *reinterpret_cast<ExtensionHeaderTypeAndSizeOverlay*>(&headerPtr[pos]);

        pos += 8 * (extensionHeader.headerExtensionLengthIn8ByteUnitsMinus1 + 1);

        if (extensionHeader.nextHeaderTypeCode == headerTypeCode) {
            wasFound = true;
            position = pos;
            return;
        }//if//

        if (!IsAnIpExtensionHeaderCode(extensionHeader.nextHeaderTypeCode)) {
            wasFound = false;
            return;
        }//if//

        assert(pos < packetLength);

    }//while//

    assert(false); abort();

}//GetHeaderTotalLengthAndNextHeaderProtocolCode//



inline
void IpHeaderOverlayModel::GetHeaderTotalLengthAndNextHeaderProtocolCode(
    unsigned int& headerLength,
    unsigned char& protocolCode) const
{
    assert(headerPtr != nullptr);

    Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(headerPtr);

    if (!IsAnIpExtensionHeaderCode(ipHeader.nextHeaderTypeCode)) {
        headerLength = sizeof(Ipv6HeaderType);
        protocolCode = ipHeader.nextHeaderTypeCode;
        return;
    }//if//

    unsigned int pos = sizeof(Ipv6HeaderType);
    while (true) {
        const ExtensionHeaderTypeAndSizeOverlay& extensionHeader =
            *reinterpret_cast<ExtensionHeaderTypeAndSizeOverlay*>(&headerPtr[pos]);

        pos += 8 * (extensionHeader.headerExtensionLengthIn8ByteUnitsMinus1 + 1);

        if (!IsAnIpExtensionHeaderCode(extensionHeader.nextHeaderTypeCode)) {
            headerLength = static_cast<unsigned int>(pos);
            protocolCode = extensionHeader.nextHeaderTypeCode;
            return;
        }//if//

        assert(pos < packetLength);

    }//while//

    assert(false); abort();

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
    Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(headerPtr);
    return (ipHeader.GetTrafficClass());
}


inline
unsigned short int IpHeaderOverlayModel::GetFlowLabel() const
{
    assert(headerPtr != nullptr);
    Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(headerPtr);
    const uint32_t flowLabel =  ipHeader.GetFlowLabel();
    if (flowLabel > USHRT_MAX) {
        cerr << "IPV6 flow label is too long (non-standard restriction)" << endl;
        exit(1);
    }//if//
    return (static_cast<unsigned short int>(flowLabel));
}

inline
NetworkAddress IpHeaderOverlayModel::GetSourceAddress() const
{
    assert(headerPtr != nullptr);
    Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(headerPtr);
    uint64_t sourceAddressHighBits;
    uint64_t sourceAddressLowBits;
    ConvertTwoNet64ToTwoHost64(
        ipHeader.sourceAddressHighBits,
        ipHeader.sourceAddressLowBits,
        sourceAddressHighBits,
        sourceAddressLowBits);
    return (NetworkAddress(sourceAddressHighBits, sourceAddressLowBits));
}

inline
NetworkAddress IpHeaderOverlayModel::GetDestinationAddress() const
{
    assert(headerPtr != nullptr);
    Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(headerPtr);
    uint64_t destinationAddressHighBits;
    uint64_t destinationAddressLowBits;
    ConvertTwoNet64ToTwoHost64(
        ipHeader.destinationAddressHighBits,
        ipHeader.destinationAddressLowBits,
        destinationAddressHighBits,
        destinationAddressLowBits);
    return (NetworkAddress(destinationAddressHighBits, destinationAddressLowBits));
}

inline
unsigned char IpHeaderOverlayModel::GetHopLimit() const
{
    assert(headerPtr != nullptr);
    Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(headerPtr);
    return (ipHeader.hopLimit);
}

inline
void IpHeaderOverlayModel::SetHopLimit(const unsigned char hopLimit)
{
    assert(headerPtr != nullptr);
    Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(headerPtr);

    ipHeader.hopLimit = hopLimit;
}

inline
void IpHeaderOverlayModel::SetTrafficClass(const unsigned char trafficClass)
{
    assert(headerPtr != nullptr);
    Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(headerPtr);
    ipHeader.SetTrafficClass(trafficClass);
}

inline
void IpHeaderOverlayModel::SetFlowLabel(const unsigned short int flowLabel)
{
    assert(headerPtr != nullptr);
    Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(headerPtr);
    ipHeader.SetFlowLabel(flowLabel);
}

inline
void IpHeaderOverlayModel::SetSourceAddress(const NetworkAddress& sourceAddress)
{
    assert(headerPtr != nullptr);
    Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(headerPtr);

    ipHeader.sourceAddressHighBits = sourceAddress.GetRawAddressHighBits();
    ipHeader.sourceAddressLowBits = sourceAddress.GetRawAddressLowBits();
}

inline
void IpHeaderOverlayModel::SetDestinationAddress(const NetworkAddress& destinationAddress)
{
    assert(headerPtr != nullptr);
    Ipv6HeaderType& ipHeader = *reinterpret_cast<Ipv6HeaderType*>(headerPtr);

    ipHeader.destinationAddressHighBits = destinationAddress.GetRawAddressHighBits();
    ipHeader.destinationAddressLowBits = destinationAddress.GetRawAddressLowBits();
}

inline
bool IpHeaderOverlayModel::MobilityExtensionHeaderExists() const
{
    bool wasFound;
    unsigned int notUsed;
    FindExtensionHeaderWithTypeCode(IpHeaderModel::mobilityExtensionHeaderTypeCode, wasFound, notUsed);
    return wasFound;
}

inline
bool IpHeaderOverlayModel::MobileIpBindingUpdateHeaderExists() const
{
    bool wasFound;
    unsigned int mobilityHeaderPos;
    FindExtensionHeaderWithTypeCode(
        IpHeaderModel::mobilityExtensionHeaderTypeCode,
        wasFound,
        mobilityHeaderPos);
    if (!wasFound) {
        return false;
    }//if//

    const MobileIpBindingUpdateExtensionHeader& potentialBindingUpdateHeader =
        *reinterpret_cast<MobileIpBindingUpdateExtensionHeader*>(&headerPtr[mobilityHeaderPos]);

    return (potentialBindingUpdateHeader.mobilityHeaderTypeCode ==
            MobileIpBindingUpdateExtensionHeader::messageTypeCode);

}//MobileIpBindingUpdateHeaderExists//

inline
const MobileIpBindingUpdateExtensionHeader& IpHeaderOverlayModel::GetMobileIpBindingUpdateHeader() const
{
    bool wasFound;
    unsigned int mobilityHeaderPos;
    FindExtensionHeaderWithTypeCode(
        IpHeaderModel::mobilityExtensionHeaderTypeCode,
        wasFound,
        mobilityHeaderPos);

    const MobileIpBindingUpdateExtensionHeader& bindingUpdateHeader =
        *reinterpret_cast<MobileIpBindingUpdateExtensionHeader*>(&headerPtr[mobilityHeaderPos]);

    assert (bindingUpdateHeader.mobilityHeaderTypeCode == MobileIpBindingUpdateExtensionHeader::messageTypeCode);

    return bindingUpdateHeader;

}//GetMobileIpBindingUpdateHeader//

inline
bool IpHeaderOverlayModel::HomeAddressDestinationOptionsHeaderExists() const
{
    typedef IpHeaderModel::HomeAddressDestinationOptionsHeader HomeAddressDestinationOptionsHeader;

    bool wasFound;
    unsigned int destinationHeaderPos;
    FindExtensionHeaderWithTypeCode(
        IpHeaderModel::destinationOptionsTypeCode,
        wasFound,
        destinationHeaderPos);

    if (!wasFound) {
        return false;
    }//if//

    const HomeAddressDestinationOptionsHeader& DestinationOptionsHeader =
        *reinterpret_cast<HomeAddressDestinationOptionsHeader*>(&headerPtr[destinationHeaderPos]);

    return (DestinationOptionsHeader.optionType == HomeAddressDestinationOptionsHeader::typeCode);

}//HomeAddressDestinationOptionsHeaderExists//



inline
NetworkAddress IpHeaderOverlayModel::GetHomeAddressFromDestinationOptionsHeader() const
{
    typedef IpHeaderModel::HomeAddressDestinationOptionsHeader HomeAddressDestinationOptionsHeader;

    bool wasFound;
    unsigned int destinationHeaderPos;
    FindExtensionHeaderWithTypeCode(
        IpHeaderModel::destinationOptionsTypeCode,
        wasFound,
        destinationHeaderPos);

    assert(wasFound);

    // Account for padding in front of plain-C struct header.
    const HomeAddressDestinationOptionsHeader& DestinationOptionsHeader =
        *reinterpret_cast<HomeAddressDestinationOptionsHeader*>(&headerPtr[destinationHeaderPos]);

    assert(DestinationOptionsHeader.optionType == HomeAddressDestinationOptionsHeader::typeCode);

    return (
        NetworkAddress(
            DestinationOptionsHeader.homeAddressHighBits,
            DestinationOptionsHeader.homeAddressLowBits));

}//GetHomeAddressFromDestinationOptionsHeader//

}//namespace//


#endif
