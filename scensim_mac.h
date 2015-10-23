// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_MAC_H
#define SCENSIM_MAC_H

#include <array>
#include "randomnumbergen.h"
#include "scensim_stats.h"
#include "scensim_queues.h"
#include "scensim_qoscontrol.h"
#include "scensim_tracedefs.h"

namespace ScenSim {

class ObjectMobilityPosition;

template<class MacAddressType>
class MacAddressResolver {
public:
    virtual void GetMacAddress(
        const NetworkAddress& aNetworkAddress,
        const NetworkAddress& networkAddressMask,
        bool& wasFound,
        MacAddressType& resolvedMacAddress) = 0;

    // Used only to get last hop address (not used in most situations).

    virtual void GetNetworkAddressIfAvailable(
        const MacAddressType& macAddress,
        const NetworkAddress& subnetNetworkAddress,
        bool& wasFound,
        NetworkAddress& resolvedNetworkAddress) = 0;

};//MacAddressResolver//

typedef unsigned long long int GenericMacAddressType;

inline
NodeIdType CalcNodeId(const GenericMacAddressType& genericMacAddress)
{
    // Lower Bits always Node ID.
    return (static_cast<NodeIdType>(genericMacAddress));
}

const unsigned int MaxMulticastGroupNumber = (256 * 256 * 256) - 1;


// IEEE 802.X Style 6 byte MAC address.

class SixByteMacAddressType {
public:
    static const SixByteMacAddressType invalidMacAddress;

    static const unsigned int numberMacAddressBytes = 6;

    static SixByteMacAddressType GetBroadcastAddress();

    SixByteMacAddressType() { std::fill(addressBytes.begin(), addressBytes.end(), 0); }

    SixByteMacAddressType(const NodeIdType nodeId, const unsigned char interfaceSelectorByte) {
        addressBytes[addressTypeCodeBytePos] = 0;
        addressBytes[interfaceSelectorBytePos] = interfaceSelectorByte;
        (*this).SetLowerBitsWithNodeId(nodeId);
    }

    void SetFromString(
        const string& addressString,
        bool& success);

    explicit
    SixByteMacAddressType(const GenericMacAddressType& address);

    GenericMacAddressType ConvertToGenericMacAddress() const;

    void Clear() {
        for(int i = 0; (i < numberMacAddressBytes); i++) {
            addressBytes[i] = 0;
        }
    }

    bool IsABroadcastAddress() const
        { return (addressBytes[addressTypeCodeBytePos] == broadcastIndicatorByteCode); }

    bool IsAMulticastAddress() const
        { return (addressBytes[addressTypeCodeBytePos] == multicastIndicatorByteCode); }

    bool IsABroadcastOrAMulticastAddress() const
        { return (IsABroadcastAddress() || IsAMulticastAddress()); }

    bool operator==(const SixByteMacAddressType& right) const;


    bool operator!=(const SixByteMacAddressType& right) const { return (!(*this == right)); }

    bool operator<(const SixByteMacAddressType& right) const;

    // Extra-simulation hack to encode Node Id into lower bits of mac-address.

    void SetLowerBitsWithNodeId(const NodeIdType nodeId);

    NodeIdType ExtractNodeId() const;

    void SetInterfaceSelectorByte(const unsigned char selectorByte) {
        addressBytes[interfaceSelectorBytePos] = selectorByte;
    }

    unsigned char GetInterfaceSelectorByte() const { return addressBytes[interfaceSelectorBytePos]; }

    void SetToAMulticastAddress(const unsigned int multicastGroupNumber);

    unsigned int GetMulticastGroupNumber() const;

    std::array<unsigned char, numberMacAddressBytes> addressBytes;

private:
    static const unsigned int addressTypeCodeBytePos = 0;
    static const unsigned int interfaceSelectorBytePos = 1;

    static const unsigned char unicastIndicatorByteCode = 0;
    static const unsigned char multicastIndicatorByteCode = 1;
    static const unsigned char broadcastIndicatorByteCode = 0xFF;

};//SixByteMacAddressType//



inline
SixByteMacAddressType::SixByteMacAddressType(const GenericMacAddressType& address)
{
    GenericMacAddressType current = address;
    for(int i = (numberMacAddressBytes-1); (i >= 0); i--) {
        addressBytes[i] = static_cast<unsigned char>(current % 256);
        current = current / 256;
    }//for//
}

inline
GenericMacAddressType SixByteMacAddressType::ConvertToGenericMacAddress() const
{
    assert(sizeof(GenericMacAddressType) >= numberMacAddressBytes);

    GenericMacAddressType retValue = 0;
    for(unsigned int i = 0; (i < numberMacAddressBytes); i++) {
        retValue = (retValue * 256) + addressBytes[i];
    }//for//

    return retValue;
}


inline
bool SixByteMacAddressType::operator==(const SixByteMacAddressType& right) const
{
    for(unsigned int i = 0; (i < numberMacAddressBytes); i++) {
        if (addressBytes[i] != right.addressBytes[i]) {
            return false;
        }//if//
    }//for//
    return true;
}


inline
bool SixByteMacAddressType::operator<(const SixByteMacAddressType& right) const
{
    // Embedded node ID order.  Then interface index and finally broadcast byte.

    for(unsigned int i = (interfaceSelectorBytePos+1); (i < numberMacAddressBytes); i++) {
        if (addressBytes[i] < right.addressBytes[i]) {
            return true;
        }
        else if (addressBytes[i] > right.addressBytes[i]) {
            return false;
        }//if//
    }//for//

    if (addressBytes[interfaceSelectorBytePos] < right.addressBytes[interfaceSelectorBytePos]) {
        return true;
    }
    else if (addressBytes[interfaceSelectorBytePos] > right.addressBytes[interfaceSelectorBytePos]) {
        return false;
    }//if//

    return (addressBytes[addressTypeCodeBytePos] < right.addressBytes[addressTypeCodeBytePos]);

}//<//


inline
void SixByteMacAddressType::SetFromString(
    const string& addressString,
    bool& success)
{
    const unsigned int MacAddressStringSize = numberMacAddressBytes * 3 -1;

    success = false;

    if (addressString.size() != MacAddressStringSize) {
        return;
    };//if//

    for(unsigned int i = 0; (i < numberMacAddressBytes); i++) {
        const char digit1 = addressString[i*3];
        const char digit2 = addressString[i*3+1];
        char separator = ':';
        if (i < (numberMacAddressBytes -1)) {
            separator = addressString[i*3+2];
        }//if//

        if ((!isxdigit(digit1)) || (!isxdigit(digit2)) || (separator != ':')) {
            return;
        }//if//

        (*this).addressBytes[i] = ConvertTwoHexCharactersToByte(digit1, digit2);
    }//for//

    success = true;

}//SetFromString//


// Extra-simulation hack to encode Node Id into lower bits of mac-address.

inline
void SixByteMacAddressType::SetLowerBitsWithNodeId(const NodeIdType nodeId)
{
    addressBytes[2] = static_cast<unsigned char>((nodeId / (256*256*256)));
    addressBytes[3] = static_cast<unsigned char>(((nodeId / (256*256)) % 256));
    addressBytes[4] = static_cast<unsigned char>(((nodeId / 256) % 256));
    addressBytes[5] = static_cast<unsigned char>((nodeId % 256));
}


inline
NodeIdType SixByteMacAddressType::ExtractNodeId() const
{
    assert((2+sizeof(NodeIdType)) == numberMacAddressBytes);
    NodeIdType retValue = 0;
    for(unsigned int i = 0; (i < sizeof(NodeIdType)); i++) {
        retValue = (retValue * 256) + addressBytes[2+i];
    }//for//

    return retValue;
}

inline
void SixByteMacAddressType::SetToAMulticastAddress(const unsigned int multicastGroupNumber)
{
    assert(multicastGroupNumber <= MaxMulticastGroupNumber);
    std::fill(addressBytes.begin(), addressBytes.end(), 0);
    addressBytes[addressTypeCodeBytePos] = multicastIndicatorByteCode;
    addressBytes[3] = static_cast<unsigned char>(multicastGroupNumber/(256*256));
    addressBytes[4] = static_cast<unsigned char>((multicastGroupNumber/256) % 256);
    addressBytes[5] = static_cast<unsigned char>(multicastGroupNumber % 256);

}//SetToAMulticastAddress//


inline
unsigned int SixByteMacAddressType::GetMulticastGroupNumber() const
{
    assert(IsAMulticastAddress());
    return ((addressBytes[3] * (256*256)) + (addressBytes[4] * 256) + addressBytes[5]);

}//GetMulticastGroupNumber//



inline
SixByteMacAddressType SixByteMacAddressType::GetBroadcastAddress()
{
    const unsigned char fillVal = broadcastIndicatorByteCode;  //MS extension workaround.

    SixByteMacAddressType ret;
    std::fill(ret.addressBytes.begin(), ret.addressBytes.end(), fillVal);
    return ret;
}


//--------------------------------------------------------------------------------------------------

class MacLayerInterfaceForEmulation {
public:
    virtual ~MacLayerInterfaceForEmulation() { }

    virtual void SetSimpleLinkMode(const NodeIdType& otherNodeId) = 0;
    virtual SimulationEngineInterface& GetSimulationEngineInterface() = 0;

    virtual void QueueOutgoingEthernetPacket(
        unique_ptr<Packet>& ethernetPacketPtr,
        const bool isLink = false) = 0;

    virtual void QueueOutgoingNonEthernetPacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& nextHopAddress,
        const PacketPriorityType priority) = 0;

    class AbstractEthernetPacketDiverter {
    public:
        virtual void ProcessIncomingPacket(unique_ptr<Packet>& ethernetPacketPtr) = 0;
    };//AbstractPacketDiverter//

    virtual void RegisterPacketDiverter(
        const shared_ptr<AbstractEthernetPacketDiverter>& packetDiverterPtr,
        const bool isLink) = 0;

    virtual void SetPacketDiverterEthernetDestinationMacAddress(
        const SixByteMacAddressType& macAddress) = 0;

    class AbstractEmulatorInterfaceForMacLayer {
    public:
        virtual void SendScanRequests(
            const NodeIdType& nodeId,
            const unsigned int channelNum,
            const ObjectMobilityPosition& nodePosition,
            const TimeType& scanDurationTime,
            unsigned int& numberRequestsSent) = 0;

        virtual void MoveNodeToAnotherPartition(
            const NodeIdType& nodeId,
            const unsigned int emulationPartitionIndex,
            const unsigned int channelNum,
            const SixByteMacAddressType& newApAddress) = 0;

    };//AbstractEmulatorInterfaceForMacLayer//


    virtual void RegisterEmulatorInterface(
        const shared_ptr<AbstractEmulatorInterfaceForMacLayer>& interfacePtr) = 0;

    virtual void ProcessRemoteChannelScanResults(unique_ptr<Packet>& messagePtr) = 0;

    virtual void DisconnectInterface() = 0;

};//MacLayerInterfaceForEmulation//


//--------------------------------------------------------------------------------------------------

class MacAndPhyInfoInterface {
public:
    virtual ~MacAndPhyInfoInterface() { }

    virtual double GetRssiOfLastFrameDbm() const = 0;
    virtual double GetSinrOfLastFrameDb() const = 0;

    virtual TimeType GetTotalIdleChannelTime() const = 0;
    virtual TimeType GetTotalBusyChannelTime() const = 0;
    virtual TimeType GetTotalTransmissionTime() const = 0;

    virtual unsigned int GetNumberOfReceivedFrames() const = 0;
    virtual unsigned int GetNumberOfFramesWithErrors() const = 0;
    virtual unsigned int GetNumberOfSignalCaptures() const = 0;

};//MacAndPhyInfoInterface//


//--------------------------------------------------------------------------------------------------
// Mac Layer
//--------------------------------------------------------------------------------------------------


class MacLayer {
public:
    virtual ~MacLayer() { }
    // Network Layer Interface:
    virtual void NetworkLayerQueueChangeNotification() = 0;
    virtual void DisconnectFromOtherLayers() = 0;
    virtual GenericMacAddressType GetGenericMacAddress() const
        { assert(false); abort(); return GenericMacAddressType(); }

    // Mac QoS control.

    virtual shared_ptr<MacQualityOfServiceControlInterface> GetQualityOfServiceInterface() const
        { return (shared_ptr<MacQualityOfServiceControlInterface>(/*nullptr*/)); }

    // "Headless" (no upper layers) Emulation mode direct ethernet Mac layer interface.

    virtual shared_ptr<MacLayerInterfaceForEmulation> GetMacLayerInterfaceForEmulation()
    {
        return (shared_ptr<MacLayerInterfaceForEmulation>(/*nullptr*/));
    }

    // Information interface for Mac and Phy layer

    virtual shared_ptr<MacAndPhyInfoInterface> GetMacAndPhyInfoInterface()
        { assert(false); abort(); return (shared_ptr<MacAndPhyInfoInterface>(/*nullptr*/)); }

};//MacLayer//

class SimpleMacPacketHandler {
public:
    virtual ~SimpleMacPacketHandler() {}
    virtual void ReceivePacketFromMac(
        unique_ptr<Packet>& packetPtr,
        const GenericMacAddressType& transmitterAddress) = 0;
};


//==================================================================================================

class NetworkLayer;
class AbstractNetwork;

class AbstractNetworkMac: public MacLayer {
public:
    static const string modelName;

    AbstractNetworkMac(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<AbstractNetwork>& abstractNetworkPtr,
        const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
        const shared_ptr<NetworkLayer>& networkLayerPtr,
        const unsigned int macInterfaceIndex,
        const RandomNumberGeneratorSeedType& nodeSeed);

    void NetworkLayerQueueChangeNotification() {
        if (!isBusy) {
            (*this).SendAPacket();
        }//if//
    }

    void DisconnectFromOtherLayers() {
        networkLayerPtr.reset();
    }

    GenericMacAddressType GetGenericMacAddress() const {
        const SixByteMacAddressType macAddress(nodeId, static_cast<unsigned char>(interfaceIndex));
        return (macAddress.ConvertToGenericMacAddress());
    }

private:
    NodeIdType GetNodeId() { return nodeId; }

    //-----------------------------------------------------

    class PacketArrivalEvent: public SimulationEvent {
    public:
        PacketArrivalEvent(
            AbstractNetworkMac* initDestinationMac,
            unique_ptr<Packet>& initPacketPtr,
            const NetworkAddress& initLastHopAddress,
            const EtherTypeFieldType& initEtherType)
            :
            destinationMac(initDestinationMac),
            packetPtr(move(initPacketPtr)),
            lastHopAddress(initLastHopAddress),
            etherType(initEtherType)
        {
        }

        virtual void ExecuteEvent() { destinationMac->ReceivePacket(packetPtr, lastHopAddress, etherType); }
    private:
        AbstractNetworkMac* destinationMac;
        unique_ptr<Packet> packetPtr;
        NetworkAddress lastHopAddress;
        EtherTypeFieldType etherType;

    };//PacketArrivalEvent//

    //-----------------------------------------------------

    class PacketSendFinishedEvent: public SimulationEvent {
    public:
        PacketSendFinishedEvent(AbstractNetworkMac* initMacPtr) : macPtr(initMacPtr) { }

        virtual void ExecuteEvent() { macPtr->PacketHasBeenSentEvent(); }
    private:
        AbstractNetworkMac* macPtr;

    };//PacketSentEvent//


    //-----------------------------------------------------

    shared_ptr<AbstractNetwork> abstractNetworkPtr;

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    NodeIdType nodeId;

    shared_ptr<NetworkLayer> networkLayerPtr;
    unsigned int interfaceIndex;
    InterfaceIdType interfaceId;

    shared_ptr<InterfaceOutputQueue> outputQueuePtr;

    bool isBusy;

    unsigned int numberPacketsSent;

    std::set<unsigned int> packetsToLoseSet;

    //-----------------------------------------------------

    void SendAPacket();

    void ReceivePacket(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& lastHopAddress,
        const EtherTypeFieldType& etherType);

    void PacketHasBeenSentEvent() {
        isBusy = false;
        SendAPacket();
    }

    TimeType CalcBandwidthLatency(const unsigned int packetBytes) const
        { return (TimeType((packetBytes * SECOND ) / bandwidthBytesPerSecond)); }

    RandomNumberGenerator aRandomNumberGenerator;

    bool packetDropByRateEnabled;

    double packetDropRate;

    TimeType minimumLatency;
    double bandwidthBytesPerSecond;

    // Statistics:
    shared_ptr<CounterStatistic> packetsSentStatPtr;
    shared_ptr<CounterStatistic> packetsDroppedStatPtr;
    shared_ptr<CounterStatistic> packetsReceivedStatPtr;

    void OutputTraceAndStatsForFrameSend(const Packet& aPacket) const;
    void OutputTraceAndStatsForFrameDrop(const Packet& aPacket) const;
    void OutputTraceAndStatsForFrameReceive(const Packet& aPacket) const;

    void OutputTraceForFrame(const Packet& aPacket, const string& eventName) const;
};//AbstractNetworkMac//



inline
void AbstractNetworkMac::OutputTraceForFrame(const Packet& aPacket, const string& eventName) const
{

    if (simEngineInterfacePtr->TraceIsOn(TraceMac)) {
        if (simEngineInterfacePtr->BinaryOutputIsOn()) {

            AbstractMacPacketTraceRecord traceData;

            const PacketIdType& packetId = aPacket.GetPacketId();
            traceData.sourceNodeId = packetId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = packetId.GetSourceNodeSequenceNumber();
            traceData.packetLengthBytes = static_cast<uint16_t>(aPacket.LengthBytes());

            assert(sizeof(traceData) == ABSTRACT_MAC_PACKET_TRACE_RECORD_BYTES);

            simEngineInterfacePtr->OutputTraceInBinary(modelName, interfaceId, eventName, traceData);
        }
        else {

            ostringstream msgStream;

            msgStream << "PktId= " << aPacket.GetPacketId();
            msgStream << " FrameBytes= " << aPacket.LengthBytes();
            simEngineInterfacePtr->OutputTrace(modelName, interfaceId, eventName, msgStream.str());
        }//if//
    }//if//

}//OutputTraceForFrame//

inline
void AbstractNetworkMac::OutputTraceAndStatsForFrameSend(const Packet& aPacket) const
{

    OutputTraceForFrame(aPacket, "Send");

    (*this).packetsSentStatPtr->IncrementCounter();

}//OutputTraceAndStatsForFrameSend//


inline
void AbstractNetworkMac::OutputTraceAndStatsForFrameDrop(const Packet& aPacket) const
{

    OutputTraceForFrame(aPacket, "Drop");

    (*this).packetsDroppedStatPtr->IncrementCounter();

}//OutputTraceAndStatsForFrameDrop//


inline
void AbstractNetworkMac::OutputTraceAndStatsForFrameReceive(const Packet& aPacket) const
{

    OutputTraceForFrame(aPacket, "Recv");

    (*this).packetsReceivedStatPtr->IncrementCounter();

}//OutputTraceAndStatsForFrameReceive//



}//namespace//


#endif
