// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_PACKET_H
#define SCENSIM_PACKET_H

#include <assert.h>
#include <string>
#include <cstring>
#include <sstream>
#include <map>
#include <memory>
#include <stdint.h>

#include "scensim_nodeid.h"
#include "scensim_engine.h"

namespace ScenSim {

using std::string;
using std::map;
using std::shared_ptr;
using std::ostream;
using std::ostringstream;
using std::cout;
using std::endl;

class EmulationSubsystemImplementation;

//--------------------------------------------------------------------------------------------------
// Packet Classes
//--------------------------------------------------------------------------------------------------

typedef unsigned char PacketPriorityType;
const PacketPriorityType MaxAvailablePacketPriority = UCHAR_MAX - 1;
const PacketPriorityType MAX_AVAILABLE_PACKET_PRIORITY = MaxAvailablePacketPriority;
const PacketPriorityType InvalidPacketPriority = UCHAR_MAX;


typedef string ExtrinsicPacketInfoIdType;
const string extrinsicInfoPacketId("PacketID");
const string extrinsicInfoInterfaceId("InterfaceID");

class ExtrinsicPacketInformation {
public:
    virtual ~ExtrinsicPacketInformation() { }

    virtual shared_ptr<ExtrinsicPacketInformation> Clone() = 0;

};//ExtrinsicPacketInformation//


class ExtrinsicPacketInfoContainer {
public:
    ExtrinsicPacketInfoContainer() { }
    ExtrinsicPacketInfoContainer(const ExtrinsicPacketInfoContainer& source);
    void operator=(const ExtrinsicPacketInfoContainer& right);

    void Insert(
        const ExtrinsicPacketInfoIdType& infoId,
        const shared_ptr<ExtrinsicPacketInformation> infoPtr);

    shared_ptr<ExtrinsicPacketInformation> Retrieve(const ExtrinsicPacketInfoIdType& infoId) const;

    bool CheckIdExist(const ExtrinsicPacketInfoIdType& infoId);
private:
    void CopyInto(const ExtrinsicPacketInfoContainer& source);
    map<ExtrinsicPacketInfoIdType, shared_ptr<ExtrinsicPacketInformation> > infos;

};//ExtrinsicPacketInfoContainer//


inline
void ExtrinsicPacketInfoContainer::CopyInto(const ExtrinsicPacketInfoContainer& source) {
    typedef
        map<ExtrinsicPacketInfoIdType, shared_ptr<ExtrinsicPacketInformation> >::const_iterator IterType;

    assert(infos.empty());

    for(IterType iter = source.infos.begin(); (iter != source.infos.end()); ++iter) {
        infos[iter->first] = (iter->second)->Clone();
    }//for//

}//CopyInto//

inline
ExtrinsicPacketInfoContainer::ExtrinsicPacketInfoContainer(const ExtrinsicPacketInfoContainer& source)
{
    (*this).CopyInto(source);
}

inline
void ExtrinsicPacketInfoContainer::operator=(const ExtrinsicPacketInfoContainer& right)
{
    infos.clear();
    (*this).CopyInto(right);
}


inline
void ExtrinsicPacketInfoContainer::Insert(
    const ExtrinsicPacketInfoIdType& infoId,
    const shared_ptr<ExtrinsicPacketInformation> infoPtr)
{
    assert(infos.find(infoId) == infos.end());
    infos[infoId] = infoPtr;
}

inline
shared_ptr<ExtrinsicPacketInformation> ExtrinsicPacketInfoContainer::Retrieve(
    const ExtrinsicPacketInfoIdType& infoId) const
{
    typedef map<ExtrinsicPacketInfoIdType, shared_ptr<ExtrinsicPacketInformation> >::const_iterator PacketInformationPtrIter;
    PacketInformationPtrIter iter = infos.find(infoId);
    assert(iter != infos.end());
    return  (*iter).second;
}

inline
bool ExtrinsicPacketInfoContainer::CheckIdExist(
    const ExtrinsicPacketInfoIdType& infoId)
{
    return (infos.find(infoId) != infos.end());
}


class PacketIdType {
public:
    static const PacketIdType nullPacketId;

    PacketIdType() : packetId(NULL_PACKET_ID) { }
    PacketIdType(const NodeIdType nodeId, const unsigned long long int sequenceNumber)
    {
        assert(nodeId < (ULLONG_MAX / MAX_LOCAL_PACKET_SEQUENCE_NUM_PLUS_ONE));
        assert(sequenceNumber < MAX_LOCAL_PACKET_SEQUENCE_NUM_PLUS_ONE);

        packetId = (nodeId * MAX_LOCAL_PACKET_SEQUENCE_NUM_PLUS_ONE) + sequenceNumber;
    }

    NodeIdType GetSourceNodeId() const
        { return static_cast<NodeIdType>(packetId / MAX_LOCAL_PACKET_SEQUENCE_NUM_PLUS_ONE); }
    unsigned long long int GetSourceNodeSequenceNumber() const
        { return (packetId % MAX_LOCAL_PACKET_SEQUENCE_NUM_PLUS_ONE); }

    string ConvertToString() const;

    bool operator==(const PacketIdType& right) const { return (packetId == right.packetId); }
    bool operator!=(const PacketIdType& right) const { return (!((*this) == right)); }
    bool operator>(const PacketIdType& right) const { return (packetId > right.packetId); }
    bool operator<(const PacketIdType& right) const { return (packetId < right.packetId); }

private:
    //
    // Could be compiler constant or runtime variable but below allows for 1.8 million nodes
    // and 10 trillion packets per node.
    //
    static const unsigned long long int NULL_PACKET_ID = 0;

    static const unsigned long long int MAX_LOCAL_PACKET_SEQUENCE_NUM_PLUS_ONE = UINT64_C(10000000000000);

    unsigned long long int packetId;

};//PacketIdType//

inline
ostream& operator<<(ostream& s, const PacketIdType& packetId) {
    s << packetId.GetSourceNodeId() << '_' << packetId.GetSourceNodeSequenceNumber();
    return s;
}

inline
string PacketIdType::ConvertToString() const
{
    ostringstream aStream;
    aStream << (*this);
    return aStream.str();
}



inline
PacketIdType MakeUniquePacketId(SimulationEngineInterface& simEngineInterface)
{
    return (
        PacketIdType(
            simEngineInterface.GetNodeId(),
            simEngineInterface.GenerateAndReturnNewLocalSequenceNumber()));
}



//--------------------------------------------------------------------------------------------------

class Packet {
public:

    static const unsigned int numAllocatedBytesForHeaders = 192;


    // Create packets from data in various types. Parameter "totalPayloadLength"
    // can expand packet size, but not shrink it below the actual data size.

    // Create packets "WithExtraHeaderSpace" more space is reserved for headers in front of the
    // payload.  Note that that "AddHeader" could be used to create a complex payload multi-part
    // payload from the bottom up so the word "Header" may be a misnomer.


    // Create from a plain C struct type.

    template<typename T>
    static unique_ptr<Packet> CreatePacket(
        SimulationEngineInterface& simEngineInterface,
        const T& payload);

    template<typename T>
    static unique_ptr<Packet> CreatePacketWithExtraHeaderSpace(
        SimulationEngineInterface& simEngineInterface,
        const T& payload,
        const unsigned int extraAllocatedBytesForHeaders);

    template<typename T>
    static unique_ptr<Packet> CreatePacket(
        SimulationEngineInterface& simEngineInterface,
        const T& payload,
        const unsigned int totalPayloadLength,
        const bool initUseVirtualPayload = false);

    template<typename T>
    static unique_ptr<Packet> CreatePacketWithExtraHeaderSpace(
        SimulationEngineInterface& simEngineInterface,
        const T& payload,
        const unsigned int totalPayloadLength,
        const unsigned int extraAllocatedBytesForHeaders,
        const bool initUseVirtualPayload = false);


    // Create from byte vector.

    static unique_ptr<Packet> CreatePacket(
        SimulationEngineInterface& simEngineInterface,
        const vector<unsigned char>& payload);

    static unique_ptr<Packet> CreatePacketWithExtraHeaderSpace(
        SimulationEngineInterface& simEngineInterface,
        const vector<unsigned char>& payload,
        const unsigned int extraAllocatedBytesForHeaders);

    static unique_ptr<Packet> CreatePacket(
        SimulationEngineInterface& simEngineInterface,
        const vector<unsigned char>& payload,
        const unsigned int totalPayloadLength,
        const bool initUseVirtualPayload = false);

    static unique_ptr<Packet> CreatePacketWithExtraHeaderSpace(
        SimulationEngineInterface& simEngineInterface,
        const vector<unsigned char>& payload,
        const unsigned int totalPayloadLength,
        const unsigned int extraAllocatedBytesForHeaders,
        const bool initUseVirtualPayload = false);


    // Create from string type.

    static unique_ptr<Packet> CreatePacket(
        SimulationEngineInterface& simEngineInterface,
        const string& payload);

    static unique_ptr<Packet> CreatePacketWithExtraHeaderSpace(
        SimulationEngineInterface& simEngineInterface,
        const string& payload,
        const unsigned int extraAllocatedBytesForHeaders);

    static unique_ptr<Packet> CreatePacket(
        SimulationEngineInterface& simEngineInterface,
        const string& payload,
        const unsigned int totalPayloadLength,
        const bool initUseVirtualPayload = false);

    static unique_ptr<Packet> CreatePacketWithExtraHeaderSpace(
        SimulationEngineInterface& simEngineInterface,
        const string& payload,
        const unsigned int totalPayloadLength,
        const unsigned int extraAllocatedBytesForHeaders,
        const bool initUseVirtualPayload = false);


    // Create from char*

    static unique_ptr<Packet> CreatePacket(
        SimulationEngineInterface& simEngineInterface,
        const unsigned char* payload,
        const unsigned int payloadLength);

    static unique_ptr<Packet> CreatePacketWithExtraHeaderSpace(
        SimulationEngineInterface& simEngineInterface,
        const unsigned char* payload,
        const unsigned int payloadLength,
        const unsigned int extraAllocatedBytesForHeaders);


    // Weird Non-const version (Stop template+overloading madness).

    static unique_ptr<Packet> CreatePacket(
        SimulationEngineInterface& simEngineInterface,
        unsigned char* payload,
        const unsigned int payloadLength)
    {
        return(
            Packet::CreatePacket(
                simEngineInterface,
                static_cast<const unsigned char*>(payload),
                payloadLength));
    }


    static unique_ptr<Packet> CreatePacketWithExtraHeaderSpace(
        SimulationEngineInterface& simEngineInterface,
        unsigned char* payload,
        const unsigned int payloadLength,
        const unsigned int extraAllocatedBytesForHeaders)
    {
        return(
            Packet::CreatePacketWithExtraHeaderSpace(
                simEngineInterface,
                static_cast<const unsigned char*>(payload),
                payloadLength,
                extraAllocatedBytesForHeaders));
    }


    // Create packet with no payload (headers only).

    static unique_ptr<Packet> CreatePacket(SimulationEngineInterface& simEngineInterface);

    static unique_ptr<Packet> CreatePacketWithExtraHeaderSpace(
        SimulationEngineInterface& simEngineInterface,
        const unsigned int extraAllocatedBytesForHeaders);

    //-----------------------------------------------------

    Packet(const Packet& right);
    void operator=(const Packet& right);

    const PacketIdType GetPacketId() const { return packetId; }
    void SetPacketId(const PacketIdType& newPacketId) { (*this).packetId = newPacketId; }

    void AddRawHeader(const unsigned char rawHeader[], const unsigned int sizeBytes)
    {
        actualTotalLengthBytes += sizeBytes;
        assert(frontPosition >= sizeBytes);
        frontPosition -= sizeBytes;
        assert(frontPosition >= 0);
        for(unsigned int i = 0; i < sizeBytes; i++) {
            packetData[frontPosition + i] = rawHeader[i];
        }//for//
    }

    template<typename T> void AddPlainStructHeader(const T& header)
    {
        AddRawHeader(reinterpret_cast<const unsigned char *>(&header), sizeof(T));
    }


    template<typename T> void AddPlainStructHeaderWithTrailingAlignmentBytes(
        const T& header, const unsigned int numTrailingAlignmentBytes)
    {
        assert(sizeof(T) > numTrailingAlignmentBytes);
        assert(numTrailingAlignmentBytes < 8);

        AddRawHeader(
            reinterpret_cast<const unsigned char *>(&header),
            (sizeof(T) - numTrailingAlignmentBytes));
    }




    void AddTrailingPadding(const unsigned int paddingLengthBytes)
    {
        if (paddingLengthBytes == 0) {
            // Nothing to do.
            return;
        }//if//

        (*this).trailingPaddingBytes += paddingLengthBytes;

        if (!useVirtualPayload) {
            assert((frontPosition + actualTotalLengthBytes + trailingPaddingBytes) <= packetData.size());

            for(unsigned int i = 0; i < trailingPaddingBytes; i++) {
                packetData[frontPosition + actualTotalLengthBytes + i] = 0;
            }//for//
        }//if//

    }//AddTrailingPadding//


    void RemoveTrailingPadding(const unsigned int paddingLengthBytes)
    {
        assert(paddingLengthBytes <= trailingPaddingBytes);
        (*this).trailingPaddingBytes -= paddingLengthBytes;
    }


    void DeleteHeader(const unsigned int bytesToDelete)
    {
        assert(actualTotalLengthBytes >= bytesToDelete);
        actualTotalLengthBytes -= bytesToDelete;
        frontPosition += bytesToDelete;
    }


    unsigned int LengthBytes() const
    {
        return actualTotalLengthBytes + virtualPayloadLengthBytes + trailingPaddingBytes;
    }

    unsigned int ActualLengthBytes() const
    {
        return (actualTotalLengthBytes + trailingPaddingBytes);
    }

    const unsigned char* GetRawPayloadData() const
    {
        return (&packetData[frontPosition]);
    }

    unsigned char* GetRawPayloadData()
    {
        return (&packetData[frontPosition]);
    }


    const unsigned char* GetRawPayloadData(const unsigned int byteOffset, const unsigned int length) const
    {
        assert(length <= (actualTotalLengthBytes - byteOffset));
        return (&packetData[(frontPosition + byteOffset)]);
    }

    unsigned char* GetRawPayloadData(const unsigned int byteOffset, const unsigned int length)
    {
        assert(length <= (actualTotalLengthBytes - byteOffset));
        return (&packetData[(frontPosition + byteOffset)]);
    }

    // Ignoring memory alignment issues (will not support older RISC architectures).

    template<typename T> const T& GetAndReinterpretPayloadData(const int byteOffset = 0) const
    {
        return *reinterpret_cast<const T*>(GetRawPayloadData(byteOffset, sizeof(T)));
    }


    template<typename T> T& GetAndReinterpretPayloadData(const int byteOffset = 0)
    {
        return *reinterpret_cast<T*>(GetRawPayloadData(byteOffset, sizeof(T)));
    }

    // Extrinsic Packet Information routines.

    void AddExtrinsicPacketInformation(
        const ExtrinsicPacketInfoIdType& extrinsicPacketInfoId,
        const shared_ptr<ExtrinsicPacketInformation>& infoPtr);

    template<typename T>
    T& GetExtrinsicPacketInformation(const ExtrinsicPacketInfoIdType& extrinsicInfoId) const
    {
        return (dynamic_cast<T&>(GetExtrinsicPacketInformationInternal(extrinsicInfoId)));
    }

    bool CheckExtrinsicPacketInformationExist(const ExtrinsicPacketInfoIdType& extrinsicInfoId) const
    {
        if((*this).extrinsicPacketInfoPtr != nullptr){
            return (*this).extrinsicPacketInfoPtr->CheckIdExist(extrinsicInfoId);
        }
        return false;
    }

    // Extrinsic packet information is shared by default.

    void MakeLocalCopyOfExtrinsicPacketInfo()
    {
        (*this).extrinsicPacketInfoPtr.reset(new ExtrinsicPacketInfoContainer(*extrinsicPacketInfoPtr));
    }

    // Dirty trick of embedding of a packet reference for external software that does not
    // use the Packet class.

    // -- Routines embed global packet sequence number at end of packet (Sequence incrementation
    //    and storage must be locked in parallel mode or per-thread exclusive sequence numbers
    //    implemented).
    // -- Global expiration simulation delay for Dirty trick extrinsic packet information.
    // -- The packet information is reclaimed in sequence number order.
    // -- Reference to a packet with an old sequence number causes a simulator error (could be
    //    tested for and dropped).
    // -- "Unembed" call must be called before the end of the packet data is used.

    //TBD void EmbedReferenceToExtrinsicInformationInPacketData();
    //TBD void UnembedReferenceToExtrinsicInformationInPacketData();


    // For Emulation Support and "bogus packets".

    static unique_ptr<Packet> CreatePacketWithoutSimInfo(
        const unsigned char data[], const unsigned int size);

    template<typename T>
    static unique_ptr<Packet> CreatePacketWithoutSimInfo(const T& payload);

private:

    template<typename T> Packet(
        const T& payload,
        const unsigned int totalPayloadLength,
        const unsigned int extraAllocatedBytesForHeaders,
        const PacketIdType& packetId,
        const bool initUseVirtualPayload = false);

    Packet(
        const vector<unsigned char>& payload,
        const unsigned int totalPayloadLength,
        const unsigned int extraAllocatedBytesForHeaders,
        const PacketIdType& packetId,
        const bool initUseVirtualPayload = false);

    Packet(
        const string& payload,
        const unsigned int totalPayloadLength,
        const unsigned int extraAllocatedBytesForHeaders,
        const PacketIdType& packetId,
        const bool initUseVirtualPayload = false);

    Packet(
        const unsigned char payload[],
        const unsigned int payloadLength,
        const unsigned int totalPayloadLength,
        const unsigned int extraAllocatedBytesForHeaders,
        const PacketIdType& packetId,
        const bool initUseVirtualPayload = false);

    static const int extraSpaceForTrailingBytes = 12;

    PacketIdType packetId;
    unsigned int actualPayloadLengthBytes;
    unsigned int frontPosition;
    unsigned int actualTotalLengthBytes;
    bool useVirtualPayload;
    unsigned int virtualPayloadLengthBytes;
    unsigned int trailingPaddingBytes;

    vector<unsigned char> packetData;

    shared_ptr<ExtrinsicPacketInfoContainer> extrinsicPacketInfoPtr;

    ExtrinsicPacketInformation& GetExtrinsicPacketInformationInternal(
        const ExtrinsicPacketInfoIdType& extrinsicInfoId) const;

    Packet();

    unsigned int CalcSizeToAllocate(
        const unsigned int payloadLengthBytes,
        const unsigned int extraAllocatedBytesForHeaders);

};//Packet//



//--------------------------------------------------------------------------------------------------


inline
unsigned int Packet::CalcSizeToAllocate(
    const unsigned int payloadLengthBytes,
    const unsigned int extraAllocatedBytesForHeaders)
{
    assert((extraAllocatedBytesForHeaders >= numAllocatedBytesForHeaders) &&
           "Extra space for headers is less than the default (probably bug)");

    if (useVirtualPayload) {
        return (
            RoundUpToNearestIntDivisibleBy8(extraAllocatedBytesForHeaders) +
            RoundUpToNearestIntDivisibleBy8(payloadLengthBytes));
    }
    else {
        return (
            RoundUpToNearestIntDivisibleBy8(extraAllocatedBytesForHeaders) +
            RoundUpToNearestIntDivisibleBy8(payloadLengthBytes) +
            extraSpaceForTrailingBytes);
    }//if//
}


inline
Packet::Packet()
    :
    packetId(PacketIdType::nullPacketId),
    actualPayloadLengthBytes(0),
    frontPosition(0),
    actualTotalLengthBytes(0),
    useVirtualPayload(false),
    virtualPayloadLengthBytes(0),
    trailingPaddingBytes(0)
{}


template<typename T>
Packet::Packet(
    const T& payload,
    const unsigned int totalPayloadLength,
    const unsigned int extraAllocatedBytesForHeaders,
    const PacketIdType& initPacketId,
    const bool initUseVirtualPayload)
    :
    actualPayloadLengthBytes(totalPayloadLength),
    useVirtualPayload(initUseVirtualPayload),
    virtualPayloadLengthBytes(0),
    trailingPaddingBytes(0),
    packetId(initPacketId)
{
    assert(sizeof(T) <= totalPayloadLength);

    if (useVirtualPayload) {
        actualPayloadLengthBytes = sizeof(T);
        virtualPayloadLengthBytes = totalPayloadLength - actualPayloadLengthBytes;
    }//if//

    packetData.resize(CalcSizeToAllocate(actualPayloadLengthBytes, extraAllocatedBytesForHeaders));

    frontPosition = extraAllocatedBytesForHeaders;
    actualTotalLengthBytes = actualPayloadLengthBytes;

    memcpy(&packetData[frontPosition], reinterpret_cast<const unsigned char*>(&payload), sizeof(T));

}//Packet()//



inline
Packet::Packet(
    const vector<unsigned char>& payload,
    const unsigned int totalPayloadLength,
    const unsigned int extraAllocatedBytesForHeaders,
    const PacketIdType& initPacketId,
    const bool initUseVirtualPayload)
    :
    actualPayloadLengthBytes(totalPayloadLength),
    useVirtualPayload(initUseVirtualPayload),
    virtualPayloadLengthBytes(0),
    trailingPaddingBytes(0),
    packetId(initPacketId)
{
    assert(payload.size() <= totalPayloadLength);

    if (useVirtualPayload) {
        actualPayloadLengthBytes = static_cast<unsigned int>(payload.size());
        virtualPayloadLengthBytes = totalPayloadLength - actualPayloadLengthBytes;
    }//if//

    packetData.resize(CalcSizeToAllocate(actualPayloadLengthBytes, extraAllocatedBytesForHeaders));

    frontPosition = extraAllocatedBytesForHeaders;
    actualTotalLengthBytes = actualPayloadLengthBytes;

    for(unsigned int i = 0; (i < payload.size()); i++) {
        packetData[frontPosition + i] = payload[i];
    }

}//Packet()//


inline
Packet::Packet(
    const string& payload,
    const unsigned int totalPayloadLength,
    const unsigned int extraAllocatedBytesForHeaders,
    const PacketIdType& initPacketId,
    const bool initUseVirtualPayload)
    :
    actualPayloadLengthBytes(totalPayloadLength),
    useVirtualPayload(initUseVirtualPayload),
    virtualPayloadLengthBytes(0),
    trailingPaddingBytes(0),
    packetId(initPacketId)
{
    assert(payload.size() <= totalPayloadLength);

    if (useVirtualPayload) {
        actualPayloadLengthBytes = static_cast<unsigned int>(payload.size());
        virtualPayloadLengthBytes = totalPayloadLength - actualPayloadLengthBytes;
    }//if//

    packetData.resize(CalcSizeToAllocate(actualPayloadLengthBytes, extraAllocatedBytesForHeaders));

    frontPosition = extraAllocatedBytesForHeaders;
    actualTotalLengthBytes = actualPayloadLengthBytes;

    for(unsigned int i = 0; (i < payload.size()); i++) {
        packetData[frontPosition + i] = payload[i];
    }
}//Packet()//


inline
Packet::Packet(
    const unsigned char payloadData[],
    const unsigned int payloadDataLength,
    const unsigned int totalPayloadLength,
    const unsigned int extraAllocatedBytesForHeaders,
    const PacketIdType& initPacketId,
    const bool initUseVirtualPayload)
    :
    actualPayloadLengthBytes(totalPayloadLength),
    useVirtualPayload(initUseVirtualPayload),
    virtualPayloadLengthBytes(0),
    trailingPaddingBytes(0),
    packetId(initPacketId)
{
    assert(payloadDataLength <= totalPayloadLength);

    if (useVirtualPayload) {
        actualPayloadLengthBytes = payloadDataLength;
        virtualPayloadLengthBytes = totalPayloadLength - actualPayloadLengthBytes;
    }//if//

    packetData.resize(CalcSizeToAllocate(actualPayloadLengthBytes, extraAllocatedBytesForHeaders));

    frontPosition = extraAllocatedBytesForHeaders;
    actualTotalLengthBytes = actualPayloadLengthBytes;

    for(unsigned int i = 0; i < payloadDataLength; i++) {
        packetData[frontPosition + i] = payloadData[i];
    }
}//Packet()//


//---------------------------------------------------------



template<typename T> inline
unique_ptr<Packet> Packet::CreatePacket(
    SimulationEngineInterface& simEngineInterface,
    const T& payload)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                sizeof(T),
                numAllocatedBytesForHeaders,
                MakeUniquePacketId(simEngineInterface))));
}



template<typename T> inline
unique_ptr<Packet> Packet::CreatePacketWithExtraHeaderSpace(
    SimulationEngineInterface& simEngineInterface,
    const T& payload,
    const unsigned int extraAllocatedBytesForHeaders)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                sizeof(T),
                (numAllocatedBytesForHeaders + extraAllocatedBytesForHeaders),
                MakeUniquePacketId(simEngineInterface))));
}


template<typename T> inline
unique_ptr<Packet> Packet::CreatePacket(
    SimulationEngineInterface& simEngineInterface,
    const T& payload,
    const unsigned int totalPayloadLength,
    const bool initUseVirtualPayload)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                totalPayloadLength,
                numAllocatedBytesForHeaders,
                MakeUniquePacketId(simEngineInterface),
                initUseVirtualPayload)));
}

template<typename T> inline
unique_ptr<Packet> Packet::CreatePacketWithExtraHeaderSpace(
    SimulationEngineInterface& simEngineInterface,
    const T& payload,
    const unsigned int totalPayloadLength,
    const unsigned int extraAllocatedBytesForHeaders,
    const bool initUseVirtualPayload)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                totalPayloadLength,
                (numAllocatedBytesForHeaders + extraAllocatedBytesForHeaders),
                MakeUniquePacketId(simEngineInterface),
                initUseVirtualPayload)));
}


inline
unique_ptr<Packet> Packet::CreatePacket(
    SimulationEngineInterface& simEngineInterface,
    const vector<unsigned char>& payload)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                static_cast<unsigned int>(payload.size()),
                numAllocatedBytesForHeaders,
                MakeUniquePacketId(simEngineInterface))));
}


inline
unique_ptr<Packet> Packet::CreatePacketWithExtraHeaderSpace(
    SimulationEngineInterface& simEngineInterface,
    const vector<unsigned char>& payload,
    const unsigned int extraAllocatedBytesForHeaders)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                static_cast<unsigned int>(payload.size()),
                (numAllocatedBytesForHeaders + extraAllocatedBytesForHeaders),
                MakeUniquePacketId(simEngineInterface))));
}




inline
unique_ptr<Packet> Packet::CreatePacket(
    SimulationEngineInterface& simEngineInterface,
    const vector<unsigned char>& payload,
    const unsigned int totalPayloadLength,
    const bool initUseVirtualPayload)
{
    return (
        unique_ptr<Packet>(
        new Packet(
            payload,
            totalPayloadLength,
            numAllocatedBytesForHeaders,
            MakeUniquePacketId(simEngineInterface),
            initUseVirtualPayload)));
}


inline
unique_ptr<Packet> Packet::CreatePacketWithExtraHeaderSpace(
    SimulationEngineInterface& simEngineInterface,
    const vector<unsigned char>& payload,
    const unsigned int totalPayloadLength,
    const unsigned int extraAllocatedBytesForHeaders,
    const bool initUseVirtualPayload)
{
    return (
        unique_ptr<Packet>(
        new Packet(
            payload,
            totalPayloadLength,
            (numAllocatedBytesForHeaders + extraAllocatedBytesForHeaders),
            MakeUniquePacketId(simEngineInterface),
            initUseVirtualPayload)));
}




inline
unique_ptr<Packet> Packet::CreatePacket(
    SimulationEngineInterface& simEngineInterface,
    const string& payload)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                static_cast<unsigned int>(payload.size()),
                numAllocatedBytesForHeaders,
                MakeUniquePacketId(simEngineInterface))));
}


inline
unique_ptr<Packet> Packet::CreatePacketWithExtraHeaderSpace(
    SimulationEngineInterface& simEngineInterface,
    const string& payload,
    const unsigned int extraAllocatedBytesForHeaders)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                static_cast<unsigned int>(payload.size()),
                (numAllocatedBytesForHeaders + extraAllocatedBytesForHeaders),
                MakeUniquePacketId(simEngineInterface))));
}


inline
unique_ptr<Packet> Packet::CreatePacket(
    SimulationEngineInterface& simEngineInterface,
    const string& payload,
    const unsigned int totalPayloadLength,
    const bool initUseVirtualPayload)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                totalPayloadLength,
                numAllocatedBytesForHeaders,
                MakeUniquePacketId(simEngineInterface),
                initUseVirtualPayload)));
}


inline
unique_ptr<Packet> Packet::CreatePacketWithExtraHeaderSpace(
    SimulationEngineInterface& simEngineInterface,
    const string& payload,
    const unsigned int totalPayloadLength,
    const unsigned int extraAllocatedBytesForHeaders,
    const bool initUseVirtualPayload)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                totalPayloadLength,
                (numAllocatedBytesForHeaders + extraAllocatedBytesForHeaders),
                MakeUniquePacketId(simEngineInterface),
                initUseVirtualPayload)));
}



inline
unique_ptr<Packet> Packet::CreatePacket(
    SimulationEngineInterface& simEngineInterface,
    const unsigned char* payload,
    const unsigned int payloadLength)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                payloadLength,
                payloadLength,
                numAllocatedBytesForHeaders,
                MakeUniquePacketId(simEngineInterface))));
}


inline
unique_ptr<Packet> Packet::CreatePacketWithExtraHeaderSpace(
    SimulationEngineInterface& simEngineInterface,
    const unsigned char* payload,
    const unsigned int payloadLength,
    const unsigned int extraAllocatedBytesForHeaders)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                payloadLength,
                payloadLength,
                (numAllocatedBytesForHeaders + extraAllocatedBytesForHeaders),
                MakeUniquePacketId(simEngineInterface))));
}

//Jay// inline
//Jay// unique_ptr<Packet> Packet::CreatePacket(
//Jay//     SimulationEngineInterface& simEngineInterface,
//Jay//     const unsigned char* payloadPart1,
//Jay//     const unsigned int payloadPart1Length,
//Jay//     const unsigned char* payloadPart2,
//Jay//     const unsigned int payloadPart2Length)
//Jay// {
//Jay//     unique_ptr<Packet> packetPtr(
//Jay//         new Packet(
//Jay//             payloadPart1,
//Jay//             payloadPart1Length,
//Jay//             (payloadPart1Length + payloadPart2Length),
//Jay//             MakeUniquePacketId(simEngineInterface)));
//Jay//
//Jay//     unsigned char* const memPtr =
//Jay//           packetPtr->GetRawPayloadData(payloadPart1Length, payloadPart2Length);
//Jay//
//Jay//     for(size_t i = 0; (i < payloadPart2Length); i++) {
//Jay//         memPtr[i] = payloadPart2[i];
//Jay//     }//for//
//Jay//
//Jay//     return (packetPtr);
//Jay//
//Jay// }//CreatePacket//

inline
unique_ptr<Packet> Packet::CreatePacket(SimulationEngineInterface& simEngineInterface)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                static_cast<const unsigned char*>(nullptr), 0, 0,
                numAllocatedBytesForHeaders,
                MakeUniquePacketId(simEngineInterface))));
}

inline
unique_ptr<Packet> Packet::CreatePacketWithExtraHeaderSpace(
    SimulationEngineInterface& simEngineInterface,
    const unsigned int extraAllocatedBytesForHeaders)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                static_cast<const unsigned char*>(nullptr), 0, 0,
                (numAllocatedBytesForHeaders + extraAllocatedBytesForHeaders),
                MakeUniquePacketId(simEngineInterface))));
}



inline
Packet::Packet(const Packet& right)
{
    actualPayloadLengthBytes = right.actualPayloadLengthBytes;
    useVirtualPayload = right.useVirtualPayload;
    virtualPayloadLengthBytes = right.virtualPayloadLengthBytes;
    trailingPaddingBytes = right.trailingPaddingBytes;
    frontPosition = right.frontPosition;
    actualTotalLengthBytes = right.actualTotalLengthBytes;
    packetData = right.packetData;
    packetId = right.packetId;
    extrinsicPacketInfoPtr = right.extrinsicPacketInfoPtr;

}//Packet()//

inline
void Packet::operator=(const Packet& right) {
    assert(this != &right);
    (*this).~Packet();
    actualPayloadLengthBytes = right.actualPayloadLengthBytes;
    useVirtualPayload = right.useVirtualPayload;
    virtualPayloadLengthBytes = right.virtualPayloadLengthBytes;
    trailingPaddingBytes = right.trailingPaddingBytes;
    frontPosition = right.frontPosition;
    actualTotalLengthBytes = right.actualTotalLengthBytes;
    packetData = right.packetData;
    packetId = right.packetId;
    extrinsicPacketInfoPtr = right.extrinsicPacketInfoPtr;

}//operator=//


inline
void Packet::AddExtrinsicPacketInformation(
    const ExtrinsicPacketInfoIdType& extrinsicPacketInfoId,
    const shared_ptr<ExtrinsicPacketInformation>& infoPtr)
{
    if (extrinsicPacketInfoPtr == nullptr) {
        extrinsicPacketInfoPtr.reset(new ExtrinsicPacketInfoContainer());
    }//if//

    extrinsicPacketInfoPtr->Insert(extrinsicPacketInfoId, infoPtr);

}//AddExtrinsicPacketInformation//


inline
ExtrinsicPacketInformation& Packet::GetExtrinsicPacketInformationInternal(
    const ExtrinsicPacketInfoIdType& extrinsicPacketInfoId) const
{
    return *(extrinsicPacketInfoPtr->Retrieve(extrinsicPacketInfoId));
}


inline
unique_ptr<Packet> Packet::CreatePacketWithoutSimInfo(
    const unsigned char messageData[],
    const unsigned int messageDataSize)
{
    return (
        unique_ptr<Packet>(
            new ScenSim::Packet(
                messageData, messageDataSize, messageDataSize,
                ScenSim::PacketIdType::nullPacketId)));
}

template<typename T> inline
unique_ptr<Packet> Packet::CreatePacketWithoutSimInfo(const T& payload)
{
    return (
        unique_ptr<Packet>(
            new Packet(
                payload,
                sizeof(T),
                numAllocatedBytesForHeaders,
                ScenSim::PacketIdType::nullPacketId)));
}


}//namespace//

#endif

