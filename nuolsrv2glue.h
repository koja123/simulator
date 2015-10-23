// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef NUOLSRV2GLUE_H
#define NUOLSRV2GLUE_H

#include "scensim_engine.h"
#include "scensim_netsim.h"

#include "olsrv2.h"

namespace ScenSim {

class NuOLSRv2Protocol
    :
    public ProtocolPacketHandler,
    public NetworkAddressInterface,
    public enable_shared_from_this<NuOLSRv2Protocol> {
public:
    static const string modelName;
    static const int SEED_HASH = 54446130;

    NuOLSRv2Protocol(
        const NodeIdType& initNodeId,
        const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
        const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
        const RandomNumberGeneratorSeedType& initNodeSeed);

    virtual ~NuOLSRv2Protocol();

    void CompleteInitialization(
        const ParameterDatabaseReader& theParameterDatabaseReader);

    void SetInterfaceId(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId);

    void Start();

    void SendPacket(
        const NuOLSRv2Port::nu_obuf_t* nuObufPtr,
        const unsigned int interfaceIndex);

    NuOLSRv2Port::nu_bool_t AddRoute(
        const NuOLSRv2Port::nu_ip_t destinationNuIp,
        const NuOLSRv2Port::nu_ip_t nextHopNuIp,
        const NuOLSRv2Port::nu_ip_t localNuIp,
        const uint8_t hopCount,
        const char* interfaceName);

    NuOLSRv2Port::nu_bool_t DeleteRoute(
        const NuOLSRv2Port::nu_ip_t destinationNuIp,
        const NuOLSRv2Port::nu_ip_t nextHopNuIp,
        const NuOLSRv2Port::nu_ip_t localNuIp,
        const uint8_t hopCount,
        const char* interfaceName);

    void ReceivePacketFromNetworkLayer(
        unique_ptr<Packet>& packetPtr,
        const NetworkAddress& sourceAddress,
        const NetworkAddress& destinationAddress_notused,
        const PacketPriorityType trafficClass_notused,
        const NetworkAddress& lastHopAddress_notused,
        const unsigned char hopLimit_notused,
        const unsigned int interfaceIndex);

    void GetPortNumbersFromPacket(
        const Packet& aPacket,
        const unsigned int transportHeaderOffset,
        bool& portNumbersWereRetrieved,
        unsigned short int& sourcePort,
        unsigned short int& destinationPort) const;

    void NotifyNetworkAddressIsChanged(
        const unsigned int interfaceIndex,
        const NetworkAddress& newInterfaceAddress,
        const unsigned int subnetMaskLengthBits);

    void ScheduleEvent(const NuOLSRv2Port::nu_time_t& time);

    inline double GenerateRandomDouble() { return aRandomNumberGenerator.GenerateRandomDouble(); };

    static NuOLSRv2Port::nu_ip_t NetworkAddressToNuIp(const NetworkAddress& ipAddress);
    static NetworkAddress NuIpToNetworkAddress(const NuOLSRv2Port::nu_ip_t& nuIp);

private:

    class NuOLSRv2ProcessEvent: public SimulationEvent {
    public:
        explicit
        NuOLSRv2ProcessEvent(
            const shared_ptr<NuOLSRv2Protocol>& initNuOLSRv2ProtocolPtr)
            :
            nuOLSRv2ProtocolPtr(initNuOLSRv2ProtocolPtr)
        {}

        void ExecuteEvent()
        {
            nuOLSRv2ProtocolPtr->Process();

        }//ExecuteEvent//

    private:
        shared_ptr<NuOLSRv2Protocol> nuOLSRv2ProtocolPtr;

    };//NuOLSRv2ProcessEvent//

    NodeIdType nodeId;
    shared_ptr<SimulationEngineInterface> simulationEngineInterfacePtr;
    shared_ptr<NetworkLayer> networkLayerPtr;
    PacketPriorityType broadcastPriority;
    RandomNumberGenerator aRandomNumberGenerator;
    bool isParameterSet;
    TimeType startHello;
    TimeType startTc;
    string metricListFile;
    string attachedNetworkListStr;
    string attachedNetworkMaskListStr;
    string attachedNetworkDistanceListStr;

    struct NuOLSRv2Stats {
        shared_ptr<CounterStatistic> packetsSentStatPtr;
        shared_ptr<CounterStatistic> packetsReceivedStatPtr;
        shared_ptr<CounterStatistic> bytesSentStatPtr;
        shared_ptr<CounterStatistic> bytesReceivedStatPtr;
    };//NuOLSRv2Stats//

    map<unsigned int, NuOLSRv2Stats> nuOLSRv2Stats;

    struct NuOLSRv2Port::olsrv2* nuOLSRv2Ptr;

    void SetAttachedNetwork();

    void ReadParameterConfiguration(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const InterfaceIdType& interfaceId);

    void SetCurrentNode();
    void Process();

    void SetLogOption(
        const string optionLine);

    void OutputTraceAndStatsForSendPacket(
        const Packet& packet,
        const unsigned int interfaceIndex,
        const InterfaceIdType& interfaceId);

    void OutputTraceAndStatsForReceivePacket(
        const Packet& packet,
        const unsigned int interfaceIndex,
        const InterfaceIdType& interfaceId);

    void OutputTraceAndStatsForAddRoute(
        const NetworkAddress& destinationAddress,
        const NetworkAddress& netmaskAddress,
        const NetworkAddress& nextHopAddress,
        const unsigned int interfaceIndex,
        const InterfaceIdType& interfaceId);

    void OutputTraceAndStatsForDeleteRoute(
        const NetworkAddress& destinationAddress,
        const NetworkAddress& netmaskAddress,
        const InterfaceIdType& interfaceId);

};//NuOLSRv2Protocol//


inline
NuOLSRv2Port::nu_ip_t NuOLSRv2Protocol::NetworkAddressToNuIp(
    const NetworkAddress& ipAddress)
{
    NuOLSRv2Port::nu_ip_addr_t nuIpAddress;

#ifdef USE_IPV6
    const uint64_t ipAddressHighBits = ipAddress.GetRawAddressHighBits();
    const uint64_t ipAddressLowBits = ipAddress.GetRawAddressLowBits();
    ConvertTwoHost64ToNet128(ipAddressHighBits, ipAddressLowBits, nuIpAddress.u8);
    NuOLSRv2Port::nu_ip_t nuIp = nu_ip6(&nuIpAddress);
#else
    nuIpAddress.u32[0] = nu_htonl(ipAddress.GetRawAddressLow32Bits());
    NuOLSRv2Port::nu_ip_t nuIp = nu_ip4(&nuIpAddress);
#endif
    NuOLSRv2Port::nu_ip_set_default_prefix(&nuIp);

    return nuIp;

}//NetworkAddressToNuIp//


inline
NetworkAddress NuOLSRv2Protocol::NuIpToNetworkAddress(
    const NuOLSRv2Port::nu_ip_t& nuIp)
{
    NuOLSRv2Port::nu_ip_addr_t nuIpAddress;

    nu_ip_addr_copy(&nuIpAddress, nuIp);

#ifdef USE_IPV6
    uint64_t ipAddressHighBits;
    uint64_t ipAddressLowBits;
    ConvertNet128ToTwoHost64(nuIpAddress.u8, ipAddressHighBits, ipAddressLowBits);
    return NetworkAddress(ipAddressHighBits, ipAddressLowBits);
#else
    return NetworkAddress(nu_ntohl(nuIpAddress.u32[0]));
#endif

}//NuIpToNetworkAddress//

}//namespace//

#endif
