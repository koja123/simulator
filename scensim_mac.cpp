// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_mac.h"
#include "scensim_network.h"

namespace ScenSim {

const SixByteMacAddressType SixByteMacAddressType::invalidMacAddress; // Default Constructor (all 0).


AbstractNetworkMac::AbstractNetworkMac(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<AbstractNetwork>& initAbstractNetworkPtr,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<NetworkLayer>& initNetworkLayerPtr,
    const unsigned int initInterfaceIndex,
    const RandomNumberGeneratorSeedType& nodeSeed)
    :
    abstractNetworkPtr(initAbstractNetworkPtr),
    simEngineInterfacePtr(simulationEngineInterfacePtr),
    nodeId(initNetworkLayerPtr->GetNodeId()),
    networkLayerPtr(initNetworkLayerPtr),
    interfaceIndex(initInterfaceIndex),
    interfaceId(initNetworkLayerPtr->GetInterfaceId(initInterfaceIndex)),
    isBusy(false),
    outputQueuePtr(
        new FifoInterfaceOutputQueue(
            theParameterDatabaseReader,
            initNetworkLayerPtr->GetInterfaceId(initInterfaceIndex),
            simulationEngineInterfacePtr)),
    numberPacketsSent(0),
    packetDropByRateEnabled(false),
    aRandomNumberGenerator(
        HashInputsToMakeSeed(nodeSeed, initInterfaceIndex)),
    minimumLatency(ZERO_TIME),
    bandwidthBytesPerSecond(0),
    packetsSentStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_FramesSent"))),
    packetsDroppedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_FramesDropped"))),
    packetsReceivedStatPtr(
        simulationEngineInterfacePtr->CreateCounterStat(
            (modelName + '_' + interfaceId + "_FramesReceived")))
{
    networkLayerPtr->SetInterfaceOutputQueue(interfaceIndex, outputQueuePtr);

    if ((theParameterDatabaseReader.ParameterExists(
        "abstract-network-mac-packets-to-lose-number-set",
        nodeId,
        interfaceId)) &&
        (theParameterDatabaseReader.ParameterExists(
        "abstract-network-mac-packet-drop-rate",
        nodeId,
        interfaceId))) {
            cerr << "Error: Cannot define abstract-network-mac-packets-to-lose-number-set" << endl;
            cerr << "                 and abstract-network-mac-packet-drop-rate simultaneously." << endl;
        exit(1);
    }//if//

    if (theParameterDatabaseReader.ParameterExists(
        "abstract-network-mac-packets-to-lose-number-set",
        nodeId,
        interfaceId)) {

        string packetSetString =
            theParameterDatabaseReader.ReadString(
                "abstract-network-mac-packets-to-lose-number-set",
                nodeId,
                interfaceId);

        DeleteTrailingSpaces(packetSetString);
        istringstream packetListStream(packetSetString);

        while(!packetListStream.eof()) {
            unsigned int packetNumberToLose;

            packetListStream >> packetNumberToLose;

            if (packetListStream.fail()) {
                cerr << "Error in: abstract-network-mac-packets-to-lose-number-set parameter." << endl;
                cerr << "          " << packetSetString <<  endl;
                exit(1);
            }//if//

            (*this).packetsToLoseSet.insert(packetNumberToLose);

        }//if//
    }
    else if (theParameterDatabaseReader.ParameterExists(
                 "abstract-network-mac-packet-drop-rate", nodeId, interfaceId)) {

        packetDropByRateEnabled = true;

        packetDropRate =
            theParameterDatabaseReader.ReadDouble(
                "abstract-network-mac-packet-drop-rate",
                nodeId,
                interfaceId);

        if ((packetDropRate < 0) || (packetDropRate > 1)) {
            cerr << "Error: abstract-network-mac-packet-drop-rate parameter: ";
            cerr << packetDropRate << " should be between 0.0 and 1.0.";
            exit(1);
        }//if//

    }//if//

    minimumLatency =
        theParameterDatabaseReader.ReadTime("abstract-network-min-latency", nodeId, interfaceId);

    const long long int bandwidthBitsPerSec =
        theParameterDatabaseReader.ReadBigInt("abstract-network-output-bandwidth-bits-per-sec", nodeId, interfaceId);
    bandwidthBytesPerSecond = static_cast<double>(bandwidthBitsPerSec / 8);

}//AbstractNetworkMac()//


void AbstractNetworkMac::SendAPacket()
{
    typedef list<shared_ptr<AbstractNetworkMac> >::iterator IterType;

    assert(!isBusy);
    if (outputQueuePtr->IsEmpty()) {
        return;
    }//if//

    isBusy = true;

    unique_ptr<Packet> packetPtr;
    NetworkAddress nextHopAddress;
    PacketPriorityType notUsed;
    EtherTypeFieldType etherType;

    outputQueuePtr->DequeuePacket(packetPtr, nextHopAddress, notUsed, etherType);

    assert(
        nextHopAddress.IsInSameSubnetAs(
            networkLayerPtr->GetNetworkAddress(interfaceIndex),
            networkLayerPtr->GetSubnetMask(interfaceIndex)));

    const TimeType packetOutTheDoorTime =
        simEngineInterfacePtr->CurrentTime() +
        (*this).CalcBandwidthLatency(packetPtr->LengthBytes());

    const TimeType arrivalTime = packetOutTheDoorTime + minimumLatency;

    (*this).numberPacketsSent++;

    const bool destinationAddressIsABroadcastAddress =
        (nextHopAddress.IsABroadcastOrAMulticastAddress(
            networkLayerPtr->GetSubnetMask(interfaceIndex)));

    if (!destinationAddressIsABroadcastAddress) {

        shared_ptr<AbstractNetworkMac> destinationMacPtr =
            abstractNetworkPtr->GetDestinationMacPtr(nextHopAddress);

        bool dropThePacket = false;

        if (packetDropByRateEnabled) {

            const double randomNumber = aRandomNumberGenerator.GenerateRandomDouble();

            dropThePacket = (randomNumber <= packetDropRate);
        }
        else if (!packetsToLoseSet.empty()) {

            dropThePacket = (packetsToLoseSet.find(numberPacketsSent) != packetsToLoseSet.end());

        }//if//


        if (!dropThePacket) {

            (*this).OutputTraceAndStatsForFrameSend(*packetPtr);

            simEngineInterfacePtr->ScheduleExternalEventAtNode(
                destinationMacPtr->GetNodeId(),
                unique_ptr<SimulationEvent>(
                    new PacketArrivalEvent(
                        destinationMacPtr.get(),
                        packetPtr,
                        networkLayerPtr->GetNetworkAddress(interfaceIndex),
                        etherType)),
                arrivalTime);
        }
        else {

            //dropped
            (*this).OutputTraceAndStatsForFrameDrop(*packetPtr);

            packetPtr = nullptr;
        }//if//
    }
    else {

        list<shared_ptr<AbstractNetworkMac> > destinationMacPtrs;

        abstractNetworkPtr->GetDestinationMacPtrsForBroadcast(
            networkLayerPtr->GetNetworkAddress(interfaceIndex),
            networkLayerPtr->GetSubnetMask(interfaceIndex),
            destinationMacPtrs);

        for(IterType iter = destinationMacPtrs.begin(); iter != destinationMacPtrs.end(); iter++) {

            shared_ptr<AbstractNetworkMac>& destinationMacPtr = (*iter);

            bool dropThePacket = false;

            if (packetDropByRateEnabled) {

                const double randomNumber = aRandomNumberGenerator.GenerateRandomDouble();
                dropThePacket = (randomNumber <= packetDropRate);
            }//if//

            if (!dropThePacket) {
                unique_ptr<Packet> copyOfPacketPtr = unique_ptr<Packet>(new Packet(*packetPtr));

                (*this).OutputTraceAndStatsForFrameSend(*copyOfPacketPtr);

                simEngineInterfacePtr->ScheduleExternalEventAtNode(
                    destinationMacPtr->GetNodeId(),
                    unique_ptr<SimulationEvent>(
                        new PacketArrivalEvent(
                            destinationMacPtr.get(),
                            copyOfPacketPtr,
                            networkLayerPtr->GetNetworkAddress(interfaceIndex),
                            etherType)),
                    arrivalTime);
            }
            else {
                //dropped

                // If packet drop is per destination with broadcast then special
                // trace and stats are needed.

                //(*this).OutputTraceAndStatsForFrameDrop(*packetPtr);
            }//if//

        }//for//

        // Delete original (copies were sent).

        packetPtr = nullptr;
    }//for//

    simEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new PacketSendFinishedEvent(this)), packetOutTheDoorTime);

}//SendAPacket//


void AbstractNetworkMac::ReceivePacket(
    unique_ptr<Packet>& packetPtr,
    const NetworkAddress& lastHopAddress,
    const EtherTypeFieldType& etherType)
{

    (*this).OutputTraceAndStatsForFrameReceive(*packetPtr);

    networkLayerPtr->ReceivePacketFromMac(interfaceIndex, packetPtr, lastHopAddress, etherType);

}//ReceivePacket//



}//namespace//
