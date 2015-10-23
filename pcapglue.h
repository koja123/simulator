// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef PCAPGLUE_H
#define PCAPGLUE_H

#include "scensim_engine.h"
#include "scensim_packet.h"
#include "scensim_time.h"

#ifdef USE_LIBPCAP
#include <pcap.h>
#endif
#include <string>

namespace ScenSim {

using std::string;
using std::unique_ptr;

using ScenSim::SimulationEngineInterface;
using ScenSim::TimeType;
using ScenSim::Packet;

#ifdef USE_LIBPCAP
class PcapGlue {
public:
    PcapGlue(
        const string& initPcapFileName,
        bool initWriteMode = false);

    virtual ~PcapGlue();

    void ReadPacketInfo(
        bool& success,
        TimeType& time,
        unsigned int& packetLengthBytes);

    void WritePacket(
        const TimeType& time,
        const unique_ptr<Packet>& packetPtr);

private:
    enum LinkType { Dot11LinkType = DLT_IEEE802_11 };

    void ReadPacketInfoInternal(
        bool& success,
        pcap_pkthdr*& pcapPacketHeaderPtr,
        const u_char*& pcapPacketDataPtr);

    static TimeType GetTimeFromPcapPacketHeader(
        const pcap_pkthdr& pcapPacketHeader);

    static void SetTimeToPcapPacketHeader(
        const TimeType& time,
        pcap_pkthdr& pcapPacketHeader);

    const string pcapFileName;
    const bool writeMode;
    char errorBuffer[PCAP_ERRBUF_SIZE];
    pcap_t* pcapPtr;
    pcap_dumper_t* pcapDumperPtr;

};//PcapGlue//
#else
class PcapGlue {
public:
    PcapGlue(
        const string& initPcapFileName,
        bool initWriteMode = false)
    {
        cerr << "Error: libpcap is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables libpcap." << endl;
        exit(1);

    }//PcapGlue//

    virtual ~PcapGlue() {}

    void ReadPacketInfo(
        bool& success,
        TimeType& time,
        unsigned int& packetLengthBytes) {}

    void WritePacket(
        const TimeType& time,
        const unique_ptr<Packet>& packetPtr) {}

};//PcapGlue//
#endif

};//namespace//

#endif
