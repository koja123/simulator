// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "pcapglue.h"

#include <iostream>
#include <cassert>
#include <cstdlib>

#if defined(_WIN32)
#pragma warning(disable:4351) // new behavior: elements of array 'array' will be default initialized (level 1)
#endif

namespace ScenSim {

using std::cerr;
using std::endl;

using ScenSim::MICRO_SECOND;
using ScenSim::SECOND;

PcapGlue::PcapGlue(
    const string& initPcapFileName,
    bool initWriteMode)
    :
    pcapFileName(initPcapFileName),
    writeMode(initWriteMode),
    errorBuffer(),
    pcapPtr(nullptr),
    pcapDumperPtr(nullptr)
{
    if (writeMode) {
        pcapPtr = pcap_open_dead(Dot11LinkType, -1);
        if (pcapPtr == nullptr) {
            cerr << "Error: pcap_open_dead() failed" << endl;
            exit(1);
        }//if//
        pcapDumperPtr = pcap_dump_open(pcapPtr, pcapFileName.c_str());
        if (pcapPtr == nullptr) {
            cerr << "Error: pcap_dump_open(" << pcapFileName << ") failed" << endl;
            cerr << pcap_geterr(pcapPtr) << endl;
            exit(1);
        }//if//
    }
    else {
        pcapPtr = pcap_open_offline(pcapFileName.c_str(), errorBuffer);
        if (pcapPtr == nullptr) {
            cerr << "Error: pcap_open_offline(" << pcapFileName << ") failed" << endl;
            cerr << errorBuffer << endl;
            exit(1);
        }//if//
    }//if//

}//PcapGlue//

PcapGlue::~PcapGlue()
{
    if (pcapDumperPtr != nullptr) {
        assert(writeMode);
        pcap_dump_close(pcapDumperPtr);
    }//if//

    if (pcapPtr != nullptr) {
        pcap_close(pcapPtr);
    }//if//

}//~PcapGlue//

void PcapGlue::ReadPacketInfo(
    bool& success,
    TimeType& time,
    unsigned int& packetLengthBytes)
{
    assert(!writeMode);

    pcap_pkthdr* pcapPacketHeaderPtr;
    const u_char* pcapPacketDataPtr;

    ReadPacketInfoInternal(success, pcapPacketHeaderPtr, pcapPacketDataPtr);

    if (success) {
        time = GetTimeFromPcapPacketHeader(*pcapPacketHeaderPtr);
        packetLengthBytes = pcapPacketHeaderPtr->caplen;
    }//if//

}//ReadPacketInfo//

void PcapGlue::WritePacket(
    const TimeType& time,
    const unique_ptr<Packet>& packetPtr)
{
    assert(writeMode);

    const unsigned int lengthBytes = packetPtr->LengthBytes();
    const unsigned int actualLengthBytes = packetPtr->ActualLengthBytes();

    if (lengthBytes != actualLengthBytes) {
        cerr << "Error: cannot write virtual packet data to file" << endl;
        exit(1);
    }//if//

    pcap_pkthdr pcapPacketHeader;
    SetTimeToPcapPacketHeader(time, pcapPacketHeader);
    pcapPacketHeader.caplen = lengthBytes;
    pcapPacketHeader.len = lengthBytes;

    u_char* pcapPacketDataPtr = packetPtr->GetRawPayloadData();

    pcap_dump((u_char*)pcapDumperPtr, &pcapPacketHeader, pcapPacketDataPtr);

}//WritePacket//

void PcapGlue::ReadPacketInfoInternal(
    bool& success,
    pcap_pkthdr*& pcapPacketHeaderPtr,
    const u_char*& pcapPacketDataPtr)
{
    int result = pcap_next_ex(pcapPtr, &pcapPacketHeaderPtr, &pcapPacketDataPtr);

    if (result == 0) {
        // result becomes to 0 if timeout, but it doesn't occur offline
        assert(false); abort();
    }
    else if (result == -1) {
        // result becomes to 1 if error
        cerr << "Error: pcap_next_ex() failed" << endl;
        cerr << pcap_geterr(pcapPtr) << endl;
        exit(1);
    }
    else if (result == -2) {
        // result becomes to -2 if no packet
        success = false;
        return;
    }
    else if (result == 1) {
        success = true;
    }
    else {
        assert(false); abort();
    }//if//

    if (pcapPacketHeaderPtr->caplen != pcapPacketHeaderPtr->len) {
        cerr << "Error: pcap file doesn't include entire packet data" << endl;
        exit(1);
    }//if//

}//ReadPacketInfoInternal//

TimeType PcapGlue::GetTimeFromPcapPacketHeader(
    const pcap_pkthdr& pcapPacketHeader)
{
    const TimeType time = pcapPacketHeader.ts.tv_sec * SECOND +
        pcapPacketHeader.ts.tv_usec * MICRO_SECOND;

    return time;

}//GetTimeFromPcapPacketHeader//

void PcapGlue::SetTimeToPcapPacketHeader(
    const TimeType& time,
    pcap_pkthdr& pcapPacketHeader)
{
    pcapPacketHeader.ts.tv_sec = static_cast<long>(time / SECOND);
    pcapPacketHeader.ts.tv_usec = static_cast<long>(time / MICRO_SECOND);

}//SetTimeToPcapPacketHeader//

};//namespace//
