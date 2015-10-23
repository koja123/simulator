// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_PROP_H
#define SCENSIM_PROP_H

#include <queue>
#include <sstream>

#include <array>

#include "scensim_proploss.h"
#include "scensim_fading.h"
#include "fupmglue_chooser.h"
#include "hfpmglue_chooser.h"
#include "scensim_gis.h"
#include "scensim_spectralmask.h"


namespace ScenSim {

using std::istringstream;
using std::array;
using std::unique_ptr;

const unsigned int InvalidChannelNumber = UINT_MAX;
const unsigned int MaxNumBondedChannels = 8;

template<typename FrameType> class SimplePropagationModel;


inline
bool VectorIsStrictlyIncreasing(const vector<unsigned int>& x)
{
    for(unsigned int i = 1; (i < x.size()); i++) {
        if (x[i-1] >= x[i]) {
            return false;
        }//if//
    }//for//
    return true;
}


inline
bool IntersectionIsEmpty(
    const vector<unsigned int>& x,
    const vector<unsigned int>& y)
{
    // assert(VectorIsStrictlyIncreasing(x) && VectorIsStrictlyIncreasing(y));

    unsigned int i = 0;
    unsigned int j = 0;

    while((i < x.size()) && (j < y.size())) {
        if (x[i] == y[j]) {
            return false;
        }
        else if (x[i] < y[j]) {
            i++;
        }
        else {
            j++;
        }//if//
    }//while//

    return true;

}//IntersectionIsEmpty//


//--------------------------------------------------------------------------------------------------
//
// Placeholder (Just for TGax channel model not a real shadowing architecture (scensim_shadowing.h).
//

class LinkShadowingModel {
public:
    LinkShadowingModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const RandomNumberGeneratorSeedType& runSeed);

    void CalcShadowFadingDb(
        const NodeIdType& nodeId1,
        const NodeIdType& nodeId2,
        double& fadingDb);

private:
    static const int SeedHash = 13248237;
    RandomNumberGeneratorSeedType modelSeed;

    double shadowingStandardDeviation;

    map<std::pair<NodeIdType, NodeIdType>, double> shadowingValueMap;

};//LinkShadowingModel//

inline
LinkShadowingModel::LinkShadowingModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const RandomNumberGeneratorSeedType& runSeed)
    :
    modelSeed(HashInputsToMakeSeed(runSeed, SeedHash))
{
    shadowingStandardDeviation =
        theParameterDatabaseReader.ReadDouble("lognormal-shadowing-standard-deviation");
}

inline
void LinkShadowingModel::CalcShadowFadingDb(
    const NodeIdType& nodeId1,
    const NodeIdType& nodeId2,
    double& shadowFadingDb)
{
    typedef map<std::pair<NodeIdType, NodeIdType>, double>::const_iterator IterType;

    NodeIdType lowNodeId = nodeId1;
    NodeIdType highNodeId = nodeId2;
    if (lowNodeId > highNodeId) {
        std::swap(lowNodeId, highNodeId);
    }//if//

    const std::pair<NodeIdType, NodeIdType> key(lowNodeId, highNodeId);

    const IterType iter = shadowingValueMap.find(key);

    if (iter != shadowingValueMap.end()) {
        shadowFadingDb = iter->second;
    }
    else {
       const RandomNumberGeneratorSeedType linkSeed =
           HashInputsToMakeSeed(modelSeed, lowNodeId, highNodeId);

       boost::mt19937 randGen(linkSeed);

       const double uniformRandom1 =
           static_cast<double>(randGen()) / (static_cast<double>(randGen.max()) + 1.0);
       const double uniformRandom2 =
           static_cast<double>(randGen()) / (static_cast<double>(randGen.max()) + 1.0);

       double gaussianRandom;
       double notUsed;
       ConvertToGaussianDistribution(uniformRandom1, uniformRandom2, gaussianRandom, notUsed);

       shadowFadingDb = gaussianRandom * shadowingStandardDeviation;
       shadowingValueMap[key] = shadowFadingDb;
    }//if//

}//CalcShadowFadingDb//


//--------------------------------------------------------------------------------------------------


//Future// class InterPropagationModelInterferenceSignalInterface {
//Future// public:
//Future//
//Future//     virtual void ReceiveInterferenceSignal(
//Future//         const NodeIdType& txNodeId,
//Future//         const unsigned int txInterfaceIndex,
//Future//         const unsigned int txChannelNumber,
//Future//         const ObjectMobilityPosition& txAntennaPosition,
//Future//         const AntennaModel& txAntennaModel,
//Future//         const double& transmitPowerDbm,
//Future//         const TimeType& currentTime,
//Future//         const TimeType& duration) = 0;
//Future//
//Future// };//InterPropagationModelInterferenceSignal//


//--------------------------------------------------------------

template<typename FrameType>
class SimplePropagationModelForNode : public enable_shared_from_this<SimplePropagationModelForNode<FrameType> > {
public:
    virtual ~SimplePropagationModelForNode();

    void TurnOnSignalsGetFramePtrSupport() { propagationModelPtr->TurnOnSignalsGetFramePtrSupport(); }

    void DisconnectThisInterface();
    bool IAmDisconnected() const { return (propagationModelPtr == nullptr); }
    TimeType DisconnectTime() const { return (theDisconnectTime); }

    bool IAmNotReceivingSignals() const { return (currentlyNotReceivingSignals); }

    void StopReceivingSignals();
    void StartReceivingSignals();

    virtual NodeIdType GetNodeId() const { return nodeId; }
    virtual unsigned int GetInterfaceIndex() const { return interfaceIndex; }

    InterfaceOrInstanceIdType GetInstanceId() { return (propagationModelPtr->GetInstanceId()); }

    virtual AntennaModel& GetAntennaModel() const { return *antennaModelPtr; }
    virtual const shared_ptr<AntennaModel> GetAntennaModelPtr() { return antennaModelPtr; }

    virtual const ObjectMobilityModel& GetMobilityModel() const { return *antennaMobilityModelPtr; }
    virtual shared_ptr<ObjectMobilityModel> GetMobilityModelPtr() const { return antennaMobilityModelPtr; }

    virtual const ObjectMobilityPosition GetCurrentMobilityPosition() const;

    virtual bool ReceivedSignalPowerIncludesMyAntennaGain() const { return true; }

    // Channel numbers start at the base channel number up to (base + ChannelCount - 1).

    unsigned int GetBaseChannelNumber() const { return (propagationModelPtr->GetBaseChannelNumber());}
    unsigned int GetChannelCount() const { return propagationModelPtr->GetChannelCount(); }

    double GetCarrierFrequencyMhz(const unsigned int channelNumber) const;
    double GetCarrierFrequencyMhz() const { return (GetCarrierFrequencyMhz(currentChannelNumber)); }

    double GetChannelBandwidthMhz(const unsigned int channelNumber) const;
    double GetChannelBandwidthMhz() const { return (GetChannelBandwidthMhz(currentChannelNumber)); }


    // FramePtr is only set (to nullptr) if model takes "ownership" of frame datastructure.

    virtual void TransmitSignal(
        const double& txPowerDbm,
        const TimeType& duration,
        const shared_ptr<FrameType>& framePtr);

    // "Bonded channel" support.

    virtual void TransmitSignal(
        const vector<unsigned int>& channelNumbers,
        const double& txPowerDbm,
        const TimeType& duration,
        const shared_ptr<FrameType>& framePtr);

    double CalculatePathlossToLocation(
        const double& positionXMeters,
        const double& positionYMeters,
        const double& positionZMeters) const;

    void CalculatePathlossToLocation(
        const PropagationInformationType& informationType,
        const double& positionXMeters,
        const double& positionYMeters,
        const double& positionZMeters,
        PropagationStatisticsType& propagationStatistics) const;

    unsigned int GetCurrentChannelNumber() const {
        assert(currentChannelSet.empty());
        return (currentChannelNumber);
    }

    const unsigned int CurrentlyReceivingFramesOnBondedChannels() const { return (!currentChannelSet.empty()); }
    const vector<unsigned int>& GetCurrentChannelNumberSet() const { return (currentChannelSet); }

    bool IsOnChannel(const unsigned int channelNum) const;

    // For Emulation Only (Distributed Partitions)
    bool ChannelIsBeingUsed(const unsigned int channelNumber) const
        { return propagationModelPtr->ChannelIsBeingUsed(channelNumber); }

    bool ChannelIsInUse(const unsigned int channelNumber) const {
        return (
            propagationModelPtr->HasActiveInterfaceForNodeOnChannel(
                nodeId,
                channelNumber));
    }

    void SwitchToChannelNumber(
        const unsigned int channelNumber,
        const bool doNotCalcInterferenceLevel = false);

    // "bonded channel" support:

    void SwitchToChannelNumbers(
        const vector<unsigned int> channelNumberSet,
        const bool doNotCalcInterferenceLevel = false);

    // Steerable antenna support:

    void SetRelativeAntennaAttitudeAzimuth(const double& azimuthDegrees);

    TimeType GetLastTransmissionEndTime() const { return lastTransmissionEndTime; }

    const shared_ptr<FrameType>& GetCurrentlyTransmittingFramePtr() const {
        assert(simEngineInterfacePtr->CurrentTime() <= lastTransmissionEndTime);
        return (currentlyTransmittingFramePtr);
    }

    double GetLastTransmissionPowerDbm() const { return lastTransmissionPowerDbm; }
    TimeType GetLastChannelSwitchTime() const { return lastChannelSwitchTime; }

    //---------------------------------

    class IncomingSignal {
    public:

        IncomingSignal(
            const unsigned int channelNumber,
            const NodeIdType& sourceNodeId,
            const TimeType& startTime,
            const TimeType& duration,
            const double transmittedPowerDbm,
            const double receivedPowerDbm,
            const shared_ptr<const FrameType>& framePtr,
            const bool initIsANoiseFrame);

        IncomingSignal(
            const vector<unsigned int>& channelNumbers,
            const NodeIdType& sourceNodeId,
            const TimeType& startTime,
            const TimeType& duration,
            const double transmittedPowerDbm,
            const double receivedPowerDbm,
            const shared_ptr<const FrameType>& framePtr,
            const bool initIsANoiseFrame);

        IncomingSignal(
            const unsigned int channelNumber,
            const NodeIdType& sourceNodeId,
            const TimeType& startTime,
            const TimeType& duration,
            const double transmittedPowerDbm,
            const double receivedPowerDbm,
            const FrameType* framePtr,
            const bool initIsANoiseFrame);

        IncomingSignal(
            const vector<unsigned int>& channelNumbers,
            const NodeIdType& sourceNodeId,
            const TimeType& startTime,
            const TimeType& duration,
            const double transmittedPowerDbm,
            const double receivedPowerDbm,
            const FrameType* framePtr,
            const bool initIsANoiseFrame);

        NodeIdType GetSourceNodeId() const { return sourceNodeId; }
        unsigned int GetChannelNumber() const { return channelNumbers.front(); }
        unsigned int GetNumberBondedChannels() const { return (numberChannels); }
        unsigned int GetBondedChannelNumber(const unsigned int channelIndex)
            { return (channelNumbers.at(channelIndex)); }

        bool ChannelIntersectionIsEmpty(const vector<unsigned int>& receivedChannels) const;
        bool IsOnChannel(const unsigned int channelNum) const;

        TimeType GetStartTime() const { return startTime; }

        TimeType GetDuration() const { return duration; }

        double GetTransmittedPowerDbm() const { return transmittedPowerDbm; }

        double GetReceivedPowerDbm() const { return receivedPowerDbm; }

        double GetPathlossDb() const { return (transmittedPowerDbm - receivedPowerDbm); }

        bool HasACompleteFrame() const { return ((!isANoiseFrame) && (rawFramePtr != nullptr)); }

        bool HasAFrame() const { return (rawFramePtr != nullptr); }

        const FrameType& GetFrame() const { return *rawFramePtr; }

        shared_ptr<const FrameType> GetFramePtr() const {
            assert((framePtr != nullptr) && "Need to call TurnOnSignalsGetFramePtrSupport()");
            return (framePtr);
        }

        void* operator new(size_t size, SingleSizeMemoryCache& memoryCache)
        {
            void* ptr;
            memoryCache.Allocate(size, ptr);
            return ptr;
        }

        void operator delete(void* ptr) { ::operator delete(ptr); }
        void operator delete(void* ptr, SingleSizeMemoryCache& memoryCache)
            { ::operator delete(ptr); }

    private:

        double transmittedPowerDbm;
        double receivedPowerDbm;
        TimeType startTime;
        TimeType duration;

        // Only if GetFramePtr() needs to be supported:
        shared_ptr<const FrameType> framePtr;

        const FrameType* rawFramePtr;
        NodeIdType sourceNodeId;
        unsigned char numberChannels;
        array<unsigned char, MaxNumBondedChannels> channelNumbers;
        bool isANoiseFrame;

        // Disable:
        IncomingSignal(const IncomingSignal&);
        void operator=(const IncomingSignal&);

    };//IncomingSignal//

    //---------------------------------

    class SignalHandler {
    public:
        virtual ~SignalHandler() {}

        virtual void ProcessSignal(const IncomingSignal& aSignal) = 0;
    };

    void RegisterSignalHandler(unique_ptr<SignalHandler>&& initSignalHandlerPtr)
    {
        signalHandlerPtr = move(initSignalHandlerPtr);
    }

    void RegisterSignalEndHandler(unique_ptr<SignalHandler>&& aSignalHandlerPtr)
    {
        assert(!hasAnEndEventHandler);
        signalEndHandlerPtr = move(aSignalHandlerPtr);
        (*this).hasAnEndEventHandler = (signalEndHandlerPtr != nullptr);
    }

    void UnregisterSignalHandler() { signalHandlerPtr.reset(); }

    void UnregisterSignalEndHandler() { signalEndHandlerPtr.reset(); }

    void ReceiveIncomingSignal(const IncomingSignal& aSignal)
        { signalHandlerPtr->ProcessSignal(aSignal); }

    void ReceiveIncomingSignalEnd(const IncomingSignal& aSignal)
        { signalEndHandlerPtr->ProcessSignal(aSignal); }

    //---------------------------------

    //virtual const ObjectMobilityModel& GetMobilityModel() const { return *antennaMobilityModelPtr; }

    virtual void SetMobilityModel(const shared_ptr<ObjectMobilityModel>& newMobilityModelPtr)
       { (*this).antennaMobilityModelPtr = newMobilityModelPtr; }

    unsigned int CurrentThreadPartitionIndex() const
        { return simEngineInterfacePtr->CurrentThreadPartitionIndex(); }

    shared_ptr<SimplePropagationModel<FrameType> > GetPropagationModel() const { return propagationModelPtr; }

    shared_ptr<SimplePropagationModelForNode<FrameType> > ClonePropModelInterface();

    double GetDistanceMetersTo(const NodeIdType& otherNodeId) const
        { return (propagationModelPtr->GetDistanceMetersBetween(nodeId, otherNodeId)); }

private:

    SimplePropagationModelForNode(
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<SimplePropagationModel<FrameType> >& propagationModelPtr,
        const shared_ptr<AntennaModel>& antennaModelPtr,
        const shared_ptr<ObjectMobilityModel>& antennaPositionModelPtr,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex);

    shared_ptr<SimplePropagationModel<FrameType> > propagationModelPtr;

    NodeIdType nodeId;
    InterfaceIdType interfaceId;
    unsigned int interfaceIndex;

    // Use one of (Either using bonded channels feature or not (can't switch)):

    unsigned int currentChannelNumber;
    vector<unsigned int> currentChannelSet;

    unique_ptr<SignalHandler> signalHandlerPtr;
    unique_ptr<SignalHandler> signalEndHandlerPtr;

    // Used during deletion (when handler pointer is nullptr).
    bool hasAnEndEventHandler;

    bool currentlyNotReceivingSignals;

    shared_ptr<AntennaModel> antennaModelPtr;
    shared_ptr<ObjectMobilityModel> antennaMobilityModelPtr;

    shared_ptr<SimulationEngineInterface> simEngineInterfacePtr;

    shared_ptr<FrameType> currentlyTransmittingFramePtr;
    TimeType lastTransmissionEndTime;
    double lastTransmissionPowerDbm;

    TimeType lastChannelSwitchTime;

    // Convenience feature to automatically reclaim frames.

    static const TimeType extraReclaimationDelay = 1*SECOND;

    struct FrameReclaimationQueueRecord {
        TimeType frameExpirationTime;
        const FrameType* framePtr;

        FrameReclaimationQueueRecord(const TimeType& initFrameExpirationTime, const FrameType* initFramePtr)
            : frameExpirationTime(initFrameExpirationTime), framePtr(initFramePtr) {}
    };

    std::queue<FrameReclaimationQueueRecord> frameReclaimationQueue;

    TimeType theDisconnectTime;

    //dynamic velocity
    //---------------------------------------
    TimeType velocityUpdateInterval;

    class UpdateVelocityEvent: public SimulationEvent {
    public:

        UpdateVelocityEvent(
            SimplePropagationModelForNode<FrameType>* initPropagationModelForNodePtr)
            :
            propagationModelForNodePtr(initPropagationModelForNodePtr)
        {}

        void ExecuteEvent() {
            propagationModelForNodePtr->UpdateVelocityForFadingModel();
        }

    private:
        SimplePropagationModelForNode* propagationModelForNodePtr;

    };//UpdateVelocityEvent//

    void UpdateVelocityForFadingModel();

    //---------------------------------------

    friend class SimplePropagationModel<FrameType>;

    // Disable:

    SimplePropagationModelForNode(const SimplePropagationModelForNode&);
    void operator=(const SimplePropagationModelForNode&);

};//SimplePropagationModelForNode//



//====================================================================================
//====================================================================================
//====================================================================================

//Future// class PropagationLossCalculationModelFactory {
//Future// public:
//Future//     virtual shared_ptr<SimplePropagationLossCalculationModel>
//Future//         CreatePropagationLossCalculationModel(
//Future//             const string& propModelName,
//Future//             const ParameterDatabaseReader& theParameterDatabaseReader,
//Future//             const shared_ptr<GisSubsystem>& gisSubsystemPtr,
//Future//             const double& carrierFrequencyMhz,
//Future//             const double& maximumPropagationDistanceMeters,
//Future//             const bool propagationDelayIsEnabled,
//Future//             const unsigned int numberThreadsForDataParallelPropCalculation,
//Future//             const InterfaceOrInstanceIdType& instanceId) const = 0;
//Future//
//Future// };//PropagationLossCalculationModelFactory//

//====================================================================================
//====================================================================================
//====================================================================================

const double DefaultIgnoreSignalCutoffPowerDbm = -300.0;

template<typename FrameType>
class SimplePropagationModel : public enable_shared_from_this<SimplePropagationModel<FrameType> > {
public:
    typedef typename SimplePropagationModelForNode<FrameType>::IncomingSignal IncomingSignal;

    SimplePropagationModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const RandomNumberGeneratorSeedType& runSeed,
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const shared_ptr<GisSubsystem>& gisSubsystemPtr,
        const InterfaceOrInstanceIdType& instanceId = nullInstanceId);

    virtual ~SimplePropagationModel() { }

    void TurnOnSignalsGetFramePtrSupport() { (*this).signalsGetFramePtrSupportIsOn = true; }

    //Future// void AddPropagationLossCalculationModelFactory(
    //Future//    const shared_ptr<PropagationLossCalculationModelFactory>& propLossFactoryPtr);

    InterfaceOrInstanceIdType GetInstanceId() const { return instanceId; }

    void DisconnectNodeInterface(
        const unsigned int channelNumber,
        const shared_ptr<SimplePropagationModelForNode<FrameType> >& interfacePtr);

    void StopReceivingSignalsAtNode(
        const unsigned int channelNumber,
        const shared_ptr<SimplePropagationModelForNode<FrameType> >& interfacePtr);

    void StartReceivingSignalsAtNode(
        const unsigned int channelNumber,
        const shared_ptr<SimplePropagationModelForNode<FrameType> >& interfacePtr);

    void AddNodeToChannel(
        const unsigned int channelNumber,
        const shared_ptr<SimplePropagationModelForNode<FrameType> >& nodeInterfacePtr);

    void InvalidateCachedInformationFor(const SimplePropagationModelForNode<FrameType>& nodeInfo);

    void DeleteNodeFromChannel(
        const unsigned int channelNumber,
        const shared_ptr<SimplePropagationModelForNode<FrameType> >& nodeInterfacePtr);

    // Note: Interface variables only necessary if using multiple interfaces with propagation model.
    // Note: The redundant "interface index" provided for speed.

    virtual shared_ptr<SimplePropagationModelForNode<FrameType> >
        GetNewPropagationModelInterface(
            const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
            const shared_ptr<AntennaModel>& antennaModelPtr,
            const shared_ptr<ObjectMobilityModel>& antennaMobilityModelPtr,
            const NodeIdType& nodeId,
            const InterfaceIdType& interfaceId,
            const unsigned int interfaceIndex);


    virtual shared_ptr<SimplePropagationModelForNode<FrameType> >
        GetNewPropagationModelInterface(
            const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
            const shared_ptr<AntennaModel>& antennaModelPtr,
            const shared_ptr<ObjectMobilityModel>& antennaMobilityModelPtr,
            const NodeIdType& nodeId)
    {
        return (
            (*this).GetNewPropagationModelInterface(
                simEngineInterfacePtr, antennaModelPtr, antennaMobilityModelPtr, nodeId,
                nullInterfaceId, 0));
    }


    double GetCarrierFrequencyMhz(const unsigned int channelNumber) const {
        return channels.at(channelNumber-baseChannelNumber).carrierFrequencyMhz;
    }

    double GetChannelBandwidthMhz(const unsigned int channelNumber) const {
        return channels.at(channelNumber-baseChannelNumber).channelBandwidthMhz;
    }

    // Channel numbers start at the base channel number up to base+ChannelCount.

    unsigned int GetBaseChannelNumber() const { return baseChannelNumber; }
    unsigned int GetChannelCount() const { return channelCount; }

    // For Emulation Partitions
    bool ChannelIsBeingUsed(const unsigned int channelNumber) const;

    shared_ptr<SimplePropagationLossCalculationModel> GetPropagationCalculationModel() const {
        return channels.front().propLossCalculationModelPtr;
    }

    void CalculatePathlossToNode(
        const PropagationInformationType& informationType,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const ObjectMobilityPosition& rxAntennaPosition,
        const AntennaModel& rxAntennaModel,
        const unsigned int channelNumber,
        PropagationStatisticsType& propagationStatistics) const;

    void CalculatePathlossToLocation(
        const PropagationInformationType& informationType,
        const TimeType& currentTime,
        const SimplePropagationModelForNode<FrameType>& transmittingNodeInfo,
        const double& positionXMeters,
        const double& positionYMeters,
        const double& positionZMeters,
        const unsigned int channelNumber,
        PropagationStatisticsType& propagationStatistics);

    void SwitchToChannelNumber(
        const shared_ptr<SimplePropagationModelForNode<FrameType> >& switchingNodeInfoPtr,
        const unsigned int channelNumber,
        const bool doNotCalcInterferrenceLevel);

    void SwitchToChannelNumbers(
        const shared_ptr<SimplePropagationModelForNode<FrameType> >& receivingNodeInfoPtr,
        const vector<unsigned int>& channelNumbers,
        const bool doNotCalcInterferenceLevel);

protected:

    typedef typename SimplePropagationModelForNode<FrameType>::SignalHandler SignalHandler;

    //--------------------------------------------

    class SignalStartEvent: public SimulationEvent {
    public:
        typedef typename SimplePropagationModelForNode<FrameType>::IncomingSignal IncomingSignal;

        SignalStartEvent(
            SimplePropagationModel<FrameType>* initPropagationModelPtr,
            SimplePropagationModelForNode<FrameType>* initReceivingNodeInfoPtr,
            IncomingSignal* initSignalPtr)
            :
            propagationModelPtr(initPropagationModelPtr),
            receivingNodeInfoPtr(initReceivingNodeInfoPtr),
            signalPtr(initSignalPtr)
        {}

        void ExecuteEvent() {
            propagationModelPtr->SendSignalStartEventToNode(*receivingNodeInfoPtr, signalPtr);
        }


        void* operator new(size_t size, TransparentSingleSizeMemoryCache& memoryCache)
        {
            void* ptr;
            memoryCache.Allocate(size, ptr);
            return ptr;
        }

        void operator delete(void* ptr, size_t size)
            { TransparentSingleSizeMemoryCache::Delete(ptr, size); }

        void operator delete(void* ptr, TransparentSingleSizeMemoryCache& memoryCache)
            { assert(false && "Only called by Exceptions when out of Memory"); }

    private:
        SimplePropagationModel* propagationModelPtr;
        SimplePropagationModelForNode<FrameType>* receivingNodeInfoPtr;
        IncomingSignal* signalPtr;

    };//SignalStartEvent//

    //-------------------------------------------

    class SignalEndEvent: public SimulationEvent {
    public:
        typedef typename SimplePropagationModelForNode<FrameType>::IncomingSignal IncomingSignal;

        SignalEndEvent(
            SimplePropagationModel<FrameType>* initPropagationModelPtr,
            SimplePropagationModelForNode<FrameType>* initReceivingNodeInfoPtr,
            IncomingSignal* initSignalPtr)
            :
            propagationModelPtr(initPropagationModelPtr),
            receivingNodeInfoPtr(initReceivingNodeInfoPtr),
            signalPtr(initSignalPtr)
        {}

        void ExecuteEvent() {
            propagationModelPtr->SendSignalEndEventToNode(*receivingNodeInfoPtr, signalPtr);
        }


        void* operator new(size_t size, TransparentSingleSizeMemoryCache& memoryCache)
        {
            void* ptr;
            memoryCache.Allocate(size, ptr);
            return ptr;
        }

        void operator delete(void* ptr, size_t size)
            { TransparentSingleSizeMemoryCache::Delete(ptr, size); }

        void operator delete(void* ptr, TransparentSingleSizeMemoryCache& memoryCache)
            { assert(false && "Only called by Exceptions when out of Memory"); }

    private:
        SimplePropagationModel* propagationModelPtr;
        SimplePropagationModelForNode<FrameType>* receivingNodeInfoPtr;
        IncomingSignal* signalPtr;

    };//SignalEndEvent//


    //-------------------------------------------

    //Parallel class InterPartitionSignalEvent: public SimulationEvent {
    //Parallel public:
    //Parallel     InterPartitionSignalEvent(
    //Parallel         SimplePropagationModel<FrameType>* initPropagationModelPtr,
    //Parallel         const unsigned int initDestinationPartitionIndex,
    //Parallel         const NodeIdType& initTxNodeId,
    //Parallel         const unsigned int initTxChannelNumber,
    //Parallel         const ObjectMobilityPosition& initTransmitAntennaPosition,
    //Parallel         const AntennaModel* initTransmitAntennaModelPtr,
    //Parallel         const double& initTransmitPowerDbm,
    //Parallel         const TimeType& initDuration,
    //Parallel         const FrameType* initFramePtr)
    //Parallel     :
    //Parallel         propagationModelPtr(initPropagationModelPtr),
    //Parallel         destinationPartitionIndex(initDestinationPartitionIndex),
    //Parallel         txNodeId(initTxNodeId),
    //Parallel         txChannelNumber(initTxChannelNumber),
    //Parallel         transmitAntennaPosition(initTransmitAntennaPosition),
    //Parallel         transmitAntennaModelPtr(initTransmitAntennaModelPtr),
    //Parallel         transmitPowerDbm(initTransmitPowerDbm),
    //Parallel         duration(initDuration),
    //Parallel         framePtr(initFramePtr)
    //Parallel     {}
    //Parallel
    //Parallel     void ExecuteEvent() {
    //Parallel         propagationModelPtr->ReceiveSignal(
    //Parallel             destinationPartitionIndex,
    //Parallel             txNodeId,
    //Parallel             txChannelNumber,
    //Parallel             transmitAntennaPosition,
    //Parallel             *transmitAntennaModelPtr,
    //Parallel             transmitPowerDbm,
    //Parallel             duration,
    //Parallel             framePtr);
    //Parallel     }
    //Parallel
    //Parallel     ~InterPartitionSignalEvent() { }
    //Parallel
    //Parallel private:
    //Parallel     SimplePropagationModel<FrameType>* propagationModelPtr;
    //Parallel     unsigned int destinationPartitionIndex;
    //Parallel     NodeIdType txNodeId;
    //Parallel     unsigned int txChannelNumber;
    //Parallel     ObjectMobilityPosition transmitAntennaPosition;
    //Parallel     const AntennaModel* transmitAntennaModelPtr;
    //Parallel     double transmitPowerDbm;
    //Parallel     TimeType duration;
    //Parallel     const FrameType* framePtr;
    //Parallel
    //Parallel };//InterPartitionSignalEvent//

    //-------------------------------------------

    bool signalsGetFramePtrSupportIsOn;

    shared_ptr<SimulationEngine> simulationEnginePtr;
    shared_ptr<GisSubsystem> gisSubsystemPtr;

    InterfaceOrInstanceIdType instanceId;

    struct ChannelInfo {
        double carrierFrequencyMhz;
        double channelBandwidthMhz;

        shared_ptr<SimplePropagationLossCalculationModel> propLossCalculationModelPtr;
        shared_ptr<FadingModel> fadingModelPtr;
    };

    vector<ChannelInfo> channels;

    unique_ptr<LinkShadowingModel> linkShadowingModelPtr;

    unsigned int baseChannelNumber;
    unsigned int channelCount;
    double ignoreSignalCutoffPowerDbm;
    double maximumPropagationDistanceMeters;

    // Currently used only for memory reclamation.
    TimeType maximumPropagationDelay;
    bool propagationDelayIsEnabled;
    bool allowMultipleInterfacesOnSameChannel;
    int numberThreadsForDataParallelPropCalculation;

    bool channelInterferenceModellingIsOn;
    vector<vector<double> > channelInterferenceMatrix;

    //--------------------------------------------

    void ScheduleSignalEventAtNode(
        SimulationEngineInterface& simEngineInterface,
        const NodeIdType& txNodeId,
        const vector<unsigned int>& channelNumbers,
        const shared_ptr<SimplePropagationModelForNode<FrameType> >& receivingNodeInfoPtr,
        const double& transmitPowerDbm,
        const double& receivedPowerDbm,
        const TimeType& currentTime,
        const TimeType& propagationDelay,
        const TimeType& duration,
        const shared_ptr<FrameType>& framePtr,
        const bool isANoiseFrame = false);

    void ScheduleSignalEventAtNode(
        SimulationEngineInterface& simEngineInterface,
        const NodeIdType& txNodeId,
        const unsigned int channelNumber,
        const shared_ptr<SimplePropagationModelForNode<FrameType> >& receivingNodeInfoPtr,
        const double& transmitPowerDbm,
        const double& receivedPowerDbm,
        const TimeType& currentTime,
        const TimeType& propagationDelay,
        const TimeType& duration,
        const shared_ptr<FrameType>& framePtr,
        const bool isANoiseFrame = false)
    {
        vector<unsigned int> channelNumbers(1);
        channelNumbers[0] = channelNumber;
        (*this).ScheduleSignalEventAtNode(
            simEngineInterface, txNodeId, channelNumbers, receivingNodeInfoPtr,
            transmitPowerDbm, receivedPowerDbm, currentTime, propagationDelay, duration,
            framePtr, isANoiseFrame);
    }

    bool NodeCanHearSignal(
        const SimplePropagationModelForNode<FrameType>& receivingNodeInfo,
        const IncomingSignal& aSignal) const;

    void SendSignalStartEventToNode(
        SimplePropagationModelForNode<FrameType>& receivingNodeInfo,
        IncomingSignal* aSignalPtr);

    void SendSignalEndEventToNode(
        SimplePropagationModelForNode<FrameType>&  receivingNodeInfo,
        IncomingSignal* aSignalPtr);

    void DeleteIncomingSignal(
        SimplePropagationModelForNode<FrameType>& receivingNodeInfo,
        IncomingSignal* aSignalPtr);


    void TransmitSignalToSingleNode(
        SimulationEngineInterface& simEngineInterface,
        const NodeIdType& txNodeId,
        const unsigned int txInterfaceIndex,
        const vector<unsigned int>& channelNumbers,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const double& transmitPowerDbm,
        const TimeType& currentTime,
        const TimeType& duration,
        const shared_ptr<FrameType>& framePtr,
        const shared_ptr<SimplePropagationModelForNode<FrameType> >& receivingNodeInfoPtr,
        const bool isANoiseFrame = false);

    void TransmitSignalToSingleNode(
        SimulationEngineInterface& simEngineInterface,
        const NodeIdType& txNodeId,
        const unsigned int txInterfaceIndex,
        const unsigned int txChannelNumber,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const double& transmitPowerDbm,
        const TimeType& currentTime,
        const TimeType& duration,
        const shared_ptr<FrameType>& framePtr,
        const shared_ptr<SimplePropagationModelForNode<FrameType> >& receivingNodeInfoPtr,
        const bool isANoiseFrame = false)
    {
        vector<unsigned int> channelNumbers(1);
        channelNumbers[0] = txChannelNumber;

        (*this).TransmitSignalToSingleNode(
            simEngineInterface, txNodeId, txInterfaceIndex, channelNumbers,
            txAntennaPosition, txAntennaModel, transmitPowerDbm, currentTime,
            duration, framePtr, receivingNodeInfoPtr, isANoiseFrame);
    }


    void TransmitSignalInLocalPartition(
        SimulationEngineInterface& simEngineInterface,
        const NodeIdType& txNodeId,
        const unsigned int txInterfaceIndex,
        const unsigned int txChannelNumber,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const double& transmitPowerDbm,
        const TimeType& currentTime,
        const TimeType& duration,
        const shared_ptr<FrameType>& framePtr);

    void TransmitSignalInLocalPartition(
        SimulationEngineInterface& simEngineInterface,
        const NodeIdType& txNodeId,
        const unsigned int txInterfaceIndex,
        const vector<unsigned int>& channelNumbers,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const double& transmitPowerDbm,
        const TimeType& currentTime,
        const TimeType& duration,
        const shared_ptr<FrameType>& framePtr);

    void TransmitSignalInLocalPartitionUtilizingMultipleThreads(
        SimulationEngineInterface& simEngineInterface,
        const NodeIdType& txNodeId,
        const unsigned int txInterfaceIndex,
        const unsigned int txChannelNumber,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const double& transmitPowerDbm,
        const TimeType& currentTime,
        const TimeType& duration,
        const shared_ptr<FrameType>& framePtr);

    void TransmitSignalInLocalPartitionUtilizingMultipleThreads(
        SimulationEngineInterface& simEngineInterface,
        const NodeIdType& txNodeId,
        const unsigned int txInterfaceIndex,
        const vector<unsigned int>& channelNumbers,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const double& transmitPowerDbm,
        const TimeType& currentTime,
        const TimeType& duration,
        const shared_ptr<FrameType>& framePtr);

    //Parallel// void SendSignalToNonLocalPartitions(
    //Parallel//     SimulationEngineInterface& simEngineInterfaceForTxNode,
    //Parallel//     SimplePropagationModelForNode<FrameType>& transmittingNodeInfo,
    //Parallel//     const double& transmitPowerDbm,
    //Parallel//     const TimeType& duration,
    //Parallel//     const shared_ptr<FrameType>& framePtr,
    //Parallel//     const unsigned int channelNumber);

    //Parallel// void ReceiveSignal(
    //Parallel//     const unsigned int destinationPartitionIndex,
    //Parallel//     const NodeIdType& txNodeId,
    //Parallel//     const unsigned int txChannelNumber,
    //Parallel//     const ObjectMobilityPosition& txAntennaPosition,
    //Parallel//     const AntennaModel& txAntennaModel,
    //Parallel//     const double& transmitPowerDbm,
    //Parallel//     const TimeType& duration,
    //Parallel//     const shared_ptr<FrameType>& framePtr)
    //Parallel// {
    //Parallel//     SimulationEngineInterface& simEngineInterface =
    //Parallel//         *(*this).propagationPartitions.at(destinationPartitionIndex)->simulationInterfacePtr;
    //Parallel//
    //Parallel//     TransmitSignalInLocalPartition(
    //Parallel//         simEngineInterface,
    //Parallel//         txNodeId,
    //Parallel//         0, // Not used.
    //Parallel//         txChannelNumber,
    //Parallel//         txAntennaPosition,
    //Parallel//         txAntennaModel,
    //Parallel//         transmitPowerDbm,
    //Parallel//         simEngineInterface.CurrentTime(),
    //Parallel//         duration,
    //Parallel//         framePtr);
    //Parallel// }

    void TransmitSignal(
        SimulationEngineInterface& simEngineInterfaceForTxNode,
        SimplePropagationModelForNode<FrameType>& transmittingNodeInfo,
        const unsigned int channelNumber,
        const double& transmitPowerDbm,
        const TimeType& duration,
        const shared_ptr<FrameType>& framePtr);

    void TransmitSignal(
        SimulationEngineInterface& simEngineInterfaceForTxNode,
        SimplePropagationModelForNode<FrameType>& transmittingNodeInfo,
        const vector<unsigned int>& channelNumbers,
        const double& transmitPowerDbm,
        const TimeType& duration,
        const shared_ptr<FrameType>& framePtr);

    // For node deletion.

    void TakeOwnershipOfFrame(
        const unsigned int partitionIndex,
        const TimeType& currentTime,
        const TimeType& endTransmissionTimeOfFrame,
        shared_ptr<FrameType>& framePtr);

    void TransmitChannelInterferenceSignals(
        SimulationEngineInterface& simEngineInterface,
        const NodeIdType& txNodeId,
        const unsigned int txInterfaceIndex,
        const unsigned int txChannelNumber,
        const ObjectMobilityPosition& txAntennaPosition,
        const AntennaModel& txAntennaModel,
        const double& transmitPowerDbm,
        const TimeType& currentTime,
        const TimeType& duration);

    double GetDistanceMetersBetween(const NodeIdType& nodeId1, const NodeIdType& nodeid2) const;


private:

    class PropagationPartitionInfo {
    public:
        ~PropagationPartitionInfo();

        shared_ptr<SimulationEngineInterface> simulationInterfacePtr;
        vector<vector<shared_ptr<SimplePropagationModelForNode<FrameType> > > > perChannelNodeList;
        vector<shared_ptr<SimplePropagationModelForNode<FrameType> > > currentlyNotReceivingNodeList;

        SignalLossCache aSignalLossCache;

        std::queue<shared_ptr<SimplePropagationModelForNode<FrameType> > > interfaceCorpseList;

        // Memory Managagement Optimizations

        SingleSizeMemoryCache incomingSignalMemoryCache;
        TransparentSingleSizeMemoryCache eventMemoryCache;

        // Convenience feature to automatically reclaim frames.

        struct FrameReclaimationQueueRecord {
            TimeType frameExpirationTime;
            shared_ptr<FrameType> framePtr;

            FrameReclaimationQueueRecord(
                const TimeType& initFrameExpirationTime,
                const shared_ptr<FrameType>& initFramePtr)
                : frameExpirationTime(initFrameExpirationTime), framePtr(initFramePtr) {}
        };

        std::queue<FrameReclaimationQueueRecord> frameReclaimationQueue;

    };//PropagationPartitionInfo//


    vector<shared_ptr<PropagationPartitionInfo> > propagationPartitions;

    void InitializeSimplePropagationModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const RandomNumberGeneratorSeedType& runSeed);

    //-------------------------------------------

    bool FadingIsEnabled() const;

    void GetFadingResultValueDb(
        const TimeType& targetTime,
        const NodeIdType& txNodeId,
        const NodeIdType& rxNodeId,
        const unsigned int channelNumber,
        double& returnFadingValueDb) const;

    TimeType velocityUpdateInterval;
    const TimeType& GetVelocityUpdateInterval() const { return velocityUpdateInterval; }
    void UpdateVelocityForFadingModel(
        const TimeType& currentTime,
        const NodeIdType& nodeId,
        const double& currentVelocityMetersPerSecond,
        const double& currentVelocityAzimuthDegrees,
        const double& currentVelocityElevationDegrees) const {

            for(unsigned int i = 0; i < channels.size(); i++) {
                channels[i].fadingModelPtr->UpdateVelocity(
                    currentTime,
                    nodeId,
                    currentVelocityMetersPerSecond,
                    currentVelocityAzimuthDegrees,
                    currentVelocityElevationDegrees);
            }//for//
    }


    void CalculateChannelInterferenceFromSpectralMasks(
        const ParameterDatabaseReader& theParameterDatabaseReader);

    bool AllChannelsHaveSameChannelBandwidth() const;

    void CheckThatNodeOrInterfaceIsNotAlreadyOnChannel(
         const vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >& nodeList,
         const shared_ptr<SimplePropagationModelForNode<FrameType> >& newNodeModelPtr);

    //-----------------------------------------------------


    friend class SimplePropagationModelForNode<FrameType>;

    // Disable
    SimplePropagationModel(const SimplePropagationModel&);
    void operator=(const SimplePropagationModel&);

};//SimplePropagationModel//



//====================================================================================


template<typename FrameType> inline
shared_ptr<SimplePropagationModelForNode<FrameType> >
    SimplePropagationModel<FrameType>::GetNewPropagationModelInterface(
        const shared_ptr<SimulationEngineInterface>& simEngineInterfacePtr,
        const shared_ptr<AntennaModel>& antennaModelPtr,
        const shared_ptr<ObjectMobilityModel>& antennaMobilityModelPtr,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int interfaceIndex)
{
    shared_ptr<SimplePropagationModelForNode<FrameType> > newNodeModelPtr(
        new SimplePropagationModelForNode<FrameType>(
            simEngineInterfacePtr,
            (*this).shared_from_this(),
            antennaModelPtr,
            antennaMobilityModelPtr,
            nodeId,
            interfaceId,
            interfaceIndex));

    if (channelCount == 1) {
        // If only one channel, automatically add it to the only channel
        // (SwitchToChannel not needed).

        newNodeModelPtr->currentChannelNumber = baseChannelNumber;

        PropagationPartitionInfo& partitionInfo =
           *(*this).propagationPartitions.at(simEngineInterfacePtr->CurrentThreadPartitionIndex());

        CheckThatNodeOrInterfaceIsNotAlreadyOnChannel(
            partitionInfo.perChannelNodeList.front(), newNodeModelPtr);

        partitionInfo.perChannelNodeList.front().push_back(newNodeModelPtr);
    }//if//

    return newNodeModelPtr;

}//GetNewPropagationModelInterface//



template<typename FrameType> inline
const ObjectMobilityPosition SimplePropagationModelForNode<FrameType>::GetCurrentMobilityPosition() const
{
    ObjectMobilityPosition position;
    antennaMobilityModelPtr->GetPositionForTime(simEngineInterfacePtr->CurrentTime(), position);
    return position;
}



template<typename FrameType> inline
void SimplePropagationModelForNode<FrameType>::SetRelativeAntennaAttitudeAzimuth(
    const double& azimuthDegrees)
{
    propagationModelPtr->InvalidateCachedInformationFor(*this);
    antennaMobilityModelPtr->SetRelativeAttitudeAzimuth(
        simEngineInterfacePtr->CurrentTime(), azimuthDegrees);
}



template<typename FrameType> inline
void SimplePropagationModelForNode<FrameType>::DisconnectThisInterface()
{
    if (IAmDisconnected()) {
        return;
    }

    if (currentChannelSet.empty()) {
        propagationModelPtr->DisconnectNodeInterface(currentChannelNumber, (*this).shared_from_this());
    }
    else {
        for(unsigned int i = 0; (i < currentChannelSet.size()); i++) {
            propagationModelPtr->DisconnectNodeInterface(
                currentChannelSet[i], (*this).shared_from_this());
        }//for//
    }//if//

    (*this).currentlyNotReceivingSignals = true;
    (*this).theDisconnectTime = simEngineInterfacePtr->CurrentTime();
    (*this).antennaMobilityModelPtr.reset();
    (*this).antennaModelPtr.reset();
    (*this).currentChannelNumber = InvalidChannelNumber;
    (*this).currentChannelSet.clear();
    (*this).signalHandlerPtr.reset();
    (*this).signalEndHandlerPtr.reset();
    (*this).propagationModelPtr.reset();
    (*this).simEngineInterfacePtr.reset();

}//DisconnectThisInterface//



template<typename FrameType> inline
void SimplePropagationModelForNode<FrameType>::StopReceivingSignals()
{
    if (currentChannelSet.empty()) {
        propagationModelPtr->StopReceivingSignalsAtNode(
            currentChannelNumber, (*this).shared_from_this());
    }
    else {
        for(unsigned int i = 0; (i < currentChannelSet.size()); i++) {
            propagationModelPtr->StopReceivingSignalsAtNode(
                currentChannelSet[i], (*this).shared_from_this());
        }//for//
    }//if//

    currentlyNotReceivingSignals = true;

}//StopReceivingSignals//



template<typename FrameType> inline
void SimplePropagationModelForNode<FrameType>::StartReceivingSignals()
{
    currentlyNotReceivingSignals = false;

    if (currentChannelSet.empty()) {
        propagationModelPtr->StartReceivingSignalsAtNode(
            currentChannelNumber, (*this).shared_from_this());
    }
    else {
        for(unsigned int i = 0; (i < currentChannelSet.size()); i++) {
            propagationModelPtr->StartReceivingSignalsAtNode(
                currentChannelSet[i], (*this).shared_from_this());
        }//for//
    }//if//

}//StartReceivingSignals//



template<typename FrameType> inline
void SimplePropagationModelForNode<FrameType>::UpdateVelocityForFadingModel()
{
    if (antennaMobilityModelPtr == nullptr) {
        return;
    }

    const TimeType currentTime = simEngineInterfacePtr->CurrentTime();
    ObjectMobilityPosition currentPosition;
    antennaMobilityModelPtr->GetPositionForTime(currentTime, currentPosition);

    propagationModelPtr->UpdateVelocityForFadingModel(
        currentTime,
        nodeId,
        currentPosition.VelocityMetersPerSecond(),
        currentPosition.VelocityAzimuthFromNorthClockwiseDegrees(),
        currentPosition.VelocityElevationFromHorizonDegrees());

    const TimeType nextUpdateTime = currentTime + velocityUpdateInterval;
    simEngineInterfacePtr->ScheduleEvent(
        unique_ptr<SimulationEvent>(new UpdateVelocityEvent(this)),
        nextUpdateTime);

}//UpdateVelocityForFadingModel//



shared_ptr<SimplePropagationLossCalculationModel>
CreatePropagationLossCalculationModel(
    const shared_ptr<SimulationEngine>& simulationEnginePtr,
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<GisSubsystem>& gisSubsystemPtr,
    const string& propModelName,
    const double& carrierFrequencyMhz,
    const double& maximumPropagationDistanceMeters,
    const bool propagationDelayIsEnabled,
    const unsigned int numberThreadsForDataParallelPropCalculation,
    const InterfaceOrInstanceIdType& instanceId);



template<typename FrameType> inline
SimplePropagationModelForNode<FrameType>::SimplePropagationModelForNode(
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<SimplePropagationModel<FrameType> >& initPropagationModelPtr,
    const shared_ptr<AntennaModel>& initAntennaModelPtr,
    const shared_ptr<ObjectMobilityModel>& initAntennaMobilityModelPtr,
    const NodeIdType& initNodeId,
    const InterfaceIdType& initInterfaceId,
    const unsigned int initInterfaceIndex)
    :
    simEngineInterfacePtr(initSimEngineInterfacePtr),
    propagationModelPtr(initPropagationModelPtr),
    nodeId(initNodeId),
    interfaceId(initInterfaceId),
    interfaceIndex(initInterfaceIndex),
    hasAnEndEventHandler(false),
    currentlyNotReceivingSignals(false),
    antennaModelPtr(initAntennaModelPtr),
    antennaMobilityModelPtr(initAntennaMobilityModelPtr),
    currentChannelNumber(InvalidChannelNumber),
    lastTransmissionEndTime(ZERO_TIME),
    lastChannelSwitchTime(ZERO_TIME),
    lastTransmissionPowerDbm(0.0),
    theDisconnectTime(INFINITE_TIME),
    velocityUpdateInterval(INFINITE_TIME)
{
    //schedule velocity update event for fading model with dynamic velocity
    if (initPropagationModelPtr->FadingIsEnabled()) {
        velocityUpdateInterval =
            initPropagationModelPtr->GetVelocityUpdateInterval();
        if (velocityUpdateInterval != INFINITE_TIME) {
            initSimEngineInterfacePtr->ScheduleEvent(
                unique_ptr<SimulationEvent>(new UpdateVelocityEvent(this)),
                initSimEngineInterfacePtr->CurrentTime());
        }
    }//if//
}



template<typename FrameType> inline
SimplePropagationModelForNode<FrameType>::~SimplePropagationModelForNode()
{
    if ((propagationModelPtr != nullptr) && (currentlyTransmittingFramePtr != nullptr)) {
        propagationModelPtr->TakeOwnershipOfFrame(
            CurrentThreadPartitionIndex(),
            simEngineInterfacePtr->CurrentTime(),
            lastTransmissionEndTime,
            currentlyTransmittingFramePtr);
    }//if//

    (*this).UnregisterSignalHandler();
    (*this).UnregisterSignalEndHandler();

}//~SimplePropagationModelForNode()//



template<typename FrameType> inline
void SimplePropagationModelForNode<FrameType>::TransmitSignal(
    const double& txPowerDbm,
    const TimeType& duration,
    const shared_ptr<FrameType>& framePtr)
{
    assert(currentChannelSet.empty() && "Not supported with bonded channels");

    if ((*this).IAmDisconnected()) {
        return;
    }//if//

    assert(currentChannelNumber != InvalidChannelNumber);

    propagationModelPtr->TransmitSignal(
        *simEngineInterfacePtr, *this, currentChannelNumber, txPowerDbm, duration, framePtr);

    (*this).lastTransmissionEndTime = simEngineInterfacePtr->CurrentTime() + duration;
    (*this).lastTransmissionPowerDbm = txPowerDbm;
    (*this).currentlyTransmittingFramePtr = framePtr;

}//TransmitSignal//



template<typename FrameType> inline
void SimplePropagationModelForNode<FrameType>::TransmitSignal(
    const vector<unsigned int>& channelNumbers,
    const double& txPowerDbm,
    const TimeType& duration,
    const shared_ptr<FrameType>& framePtr)
{
    if ((*this).IAmDisconnected()) {
        return;
    }//if//

    propagationModelPtr->TransmitSignal(
        *simEngineInterfacePtr, *this, channelNumbers, txPowerDbm, duration, framePtr);

    (*this).lastTransmissionEndTime = simEngineInterfacePtr->CurrentTime() + duration;
    (*this).lastTransmissionPowerDbm = txPowerDbm;
    (*this).currentlyTransmittingFramePtr = framePtr;


}//TransmitSignal//



template<typename FrameType> inline
double SimplePropagationModelForNode<FrameType>::GetCarrierFrequencyMhz(
    const unsigned int channelNumber) const
{
    assert(channelNumber != InvalidChannelNumber);
    return (propagationModelPtr->GetCarrierFrequencyMhz(channelNumber));
}



template<typename FrameType> inline
double SimplePropagationModelForNode<FrameType>::GetChannelBandwidthMhz(
    const unsigned int channelNumber) const
{
    assert(channelNumber != InvalidChannelNumber);
    return (propagationModelPtr->GetChannelBandwidthMhz(channelNumber));
}



template<typename FrameType> inline
void SimplePropagationModelForNode<FrameType>::CalculatePathlossToLocation(
    const PropagationInformationType& informationType,
    const double& positionXMeters,
    const double& positionYMeters,
    const double& positionZMeters,
    PropagationStatisticsType& propagationStatistics) const
{
    unsigned int channelNumber = currentChannelNumber;
    if (!currentChannelSet.empty()) {
        channelNumber = currentChannelSet.front();
    }//if//

    propagationModelPtr->CalculatePathlossToLocation(
        informationType,
        simEngineInterfacePtr->CurrentTime(),
        *this,
        positionXMeters,
        positionYMeters,
        positionZMeters,
        channelNumber,
        propagationStatistics);
}



template<typename FrameType> inline
double SimplePropagationModelForNode<FrameType>::CalculatePathlossToLocation(
    const double& positionXMeters,
    const double& positionYMeters,
    const double& positionZMeters) const
{
    unsigned int channelNumber = currentChannelNumber;
    if (!currentChannelSet.empty()) {
        channelNumber = currentChannelSet.front();
    }//if//


    PropagationStatisticsType propagationStatistics;

    propagationModelPtr->CalculatePathlossToLocation(
        PROPAGATION_INFORMATION_PATHLOSS,
        simEngineInterfacePtr->CurrentTime(),
        *this,
        positionXMeters,
        positionYMeters,
        positionZMeters,
        channelNumber,
        propagationStatistics);

    return (propagationStatistics.totalLossValueDb);
}


template<typename FrameType> inline
bool SimplePropagationModelForNode<FrameType>::IsOnChannel(const unsigned int channelNum) const
{
    if (currentChannelSet.empty()) {
        return (channelNum == currentChannelNumber);
    }
    else {
        for(unsigned int i = 0; (i < currentChannelSet.size()); i++) {
            if (currentChannelSet[i] == channelNum) {
                return true;
            }//if//
        }//for//
        return false;
    }//if//

}//IsOnChannel//


template<typename FrameType> inline
void SimplePropagationModelForNode<FrameType>::SwitchToChannelNumbers(
    const vector<unsigned int> channelNumberSet,
    const bool doNotCalcInterferenceLevel)
{
    if (propagationModelPtr->GetChannelCount() == 1) {
        // Degenerate case: only one channel.
        assert(channelNumberSet.size() == 1);
        (*this).SwitchToChannelNumber(channelNumberSet.front(), doNotCalcInterferenceLevel);
        return;
    }//if//

    assert((currentChannelNumber == InvalidChannelNumber) && "Error mixed non-bonded and bonded channel modes");

    propagationModelPtr->SwitchToChannelNumbers(
        (*this).shared_from_this(),
        channelNumberSet,
        doNotCalcInterferenceLevel);

}//SwitchToChannelNumbers//







template<typename FrameType> inline
void SimplePropagationModelForNode<FrameType>::SwitchToChannelNumber(
    const unsigned int channelNumber,
    const bool doNotCalcInterferenceLevel)
{
    assert(currentChannelSet.empty() && "Not supported with bonded channels");

    if ((doNotCalcInterferenceLevel) &&
        (channelNumber == currentChannelNumber)) {
        return;
    }//if//

    propagationModelPtr->SwitchToChannelNumber(
        (*this).shared_from_this(),
        channelNumber,
        doNotCalcInterferenceLevel);

    (*this).currentChannelNumber = channelNumber;

}//SwitchToChannelNumber//



template<typename FrameType> inline
SimplePropagationModelForNode<FrameType>::IncomingSignal::IncomingSignal(
    const unsigned int channelNumber,
    const NodeIdType& initSourceNodeId,
    const TimeType& initStartTime,
    const TimeType& initDuration,
    const double initTransmittedPowerDbm,
    const double initReceivedPowerDbm,
    const FrameType* initRawFramePtr,
    const bool initIsANoiseFrame)
    :
    sourceNodeId(initSourceNodeId),
    numberChannels(1),
    startTime(initStartTime),
    duration(initDuration),
    transmittedPowerDbm(initTransmittedPowerDbm),
    receivedPowerDbm(initReceivedPowerDbm),
    rawFramePtr(initRawFramePtr),
    isANoiseFrame(initIsANoiseFrame)
{
    channelNumbers[0] = channelNumber;

    assert(framePtr == nullptr);
}




template<typename FrameType> inline
SimplePropagationModelForNode<FrameType>::IncomingSignal::IncomingSignal(
    const vector<unsigned int>& initChannelNumbers,
    const NodeIdType& initSourceNodeId,
    const TimeType& initStartTime,
    const TimeType& initDuration,
    const double initTransmittedPowerDbm,
    const double initReceivedPowerDbm,
    const FrameType* initRawFramePtr,
    const bool initIsANoiseFrame)
    :
    sourceNodeId(initSourceNodeId),
    numberChannels(static_cast<unsigned char>(initChannelNumbers.size())),
    startTime(initStartTime),
    duration(initDuration),
    transmittedPowerDbm(initTransmittedPowerDbm),
    receivedPowerDbm(initReceivedPowerDbm),
    rawFramePtr(initRawFramePtr),
    isANoiseFrame(initIsANoiseFrame)
{
    assert((numberChannels > 0) && (numberChannels <= channelNumbers.size()));

    for(unsigned int i = 0; (i < numberChannels); i++) {
        assert(initChannelNumbers[i] < UCHAR_MAX);
        channelNumbers[i] = static_cast<unsigned char>(initChannelNumbers[i]);
    }//for//

    assert(framePtr == nullptr);
}



template<typename FrameType> inline
SimplePropagationModelForNode<FrameType>::IncomingSignal::IncomingSignal(
    const unsigned int channelNumber,
    const NodeIdType& initSourceNodeId,
    const TimeType& initStartTime,
    const TimeType& initDuration,
    const double initTransmittedPowerDbm,
    const double initReceivedPowerDbm,
    const shared_ptr<const FrameType>& initFramePtr,
    const bool initIsANoiseFrame)
    :
    sourceNodeId(initSourceNodeId),
    numberChannels(1),
    startTime(initStartTime),
    duration(initDuration),
    transmittedPowerDbm(initTransmittedPowerDbm),
    receivedPowerDbm(initReceivedPowerDbm),
    framePtr(initFramePtr),
    rawFramePtr(initFramePtr.get()),
    isANoiseFrame(initIsANoiseFrame)
{
    channelNumbers[0] = channelNumber;

    assert(framePtr.get() == rawFramePtr);
}



template<typename FrameType> inline
SimplePropagationModelForNode<FrameType>::IncomingSignal::IncomingSignal(
    const vector<unsigned int>& initChannelNumbers,
    const NodeIdType& initSourceNodeId,
    const TimeType& initStartTime,
    const TimeType& initDuration,
    const double initTransmittedPowerDbm,
    const double initReceivedPowerDbm,
    const shared_ptr<const FrameType>& initFramePtr,
    const bool initIsANoiseFrame)
    :
    sourceNodeId(initSourceNodeId),
    numberChannels(static_cast<unsigned char>(initChannelNumbers.size())),
    startTime(initStartTime),
    duration(initDuration),
    transmittedPowerDbm(initTransmittedPowerDbm),
    receivedPowerDbm(initReceivedPowerDbm),
    framePtr(initFramePtr),
    rawFramePtr(initFramePtr.get()),
    isANoiseFrame(initIsANoiseFrame)
{
    assert((numberChannels > 0) && (numberChannels <= channelNumbers.size()));

    for(unsigned int i = 0; (i < numberChannels); i++) {
        assert(initChannelNumbers[i] < UCHAR_MAX);
        channelNumbers[i] = static_cast<unsigned char>(initChannelNumbers[i]);
    }//for//

    assert(framePtr.get() == rawFramePtr);
}



//--------------------------------------------------------------------------------------------------

template<typename FrameType> inline
SimplePropagationModel<FrameType>::SimplePropagationModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const RandomNumberGeneratorSeedType& runSeed,
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const shared_ptr<GisSubsystem>& initGisSubsystemPtr,
    const InterfaceOrInstanceIdType& initInstanceId)
    :
    simulationEnginePtr(initSimulationEnginePtr),
    gisSubsystemPtr(initGisSubsystemPtr),
    instanceId(initInstanceId),
    numberThreadsForDataParallelPropCalculation(0),
    ignoreSignalCutoffPowerDbm(DefaultIgnoreSignalCutoffPowerDbm),
    maximumPropagationDistanceMeters(DBL_MAX),
    propagationDelayIsEnabled(false),
    allowMultipleInterfacesOnSameChannel(false),
    propagationPartitions(initSimulationEnginePtr->GetNumberPartitionThreads()),
    velocityUpdateInterval(INFINITE_TIME),
    baseChannelNumber(0),
    channelInterferenceModellingIsOn(false),
    signalsGetFramePtrSupportIsOn(false)
{
    ConvertStringToLowerCase(instanceId);

    InitializeSimplePropagationModel(
        theParameterDatabaseReader,
        runSeed);
}



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::InitializeSimplePropagationModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const RandomNumberGeneratorSeedType& runSeed)
{
    baseChannelNumber = 0;
    if (theParameterDatabaseReader.ParameterExists("first-channel-number", instanceId)) {
        baseChannelNumber =
            theParameterDatabaseReader.ReadNonNegativeInt("first-channel-number", instanceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("channel-count", instanceId)) {
        channelCount = theParameterDatabaseReader.ReadNonNegativeInt("channel-count", instanceId);

        assert((channelCount != 0) && "Error: Channel count is 0");

        channels.resize(channelCount);

        for(unsigned int channelIndex = 0; channelIndex < channels.size(); channelIndex++) {
            ChannelInfo& channel = channels.at(channelIndex);
            const unsigned int channelNumber = baseChannelNumber + channelIndex;

            ostringstream frequencyParameterStream;
            frequencyParameterStream << "channel-" << channelNumber << "-frequency-mhz";
            string frequencyParameterName = frequencyParameterStream.str();
            channel.carrierFrequencyMhz = theParameterDatabaseReader.ReadDouble(frequencyParameterName,instanceId);

            ostringstream bandwidthParameterStream;
            bandwidthParameterStream << "channel-" << channelNumber << "-bandwidth-mhz";
            string bandwidthParameterName = bandwidthParameterStream.str();
            channel.channelBandwidthMhz = theParameterDatabaseReader.ReadDouble(bandwidthParameterName,instanceId);

        }//for//
    }
    else {
        channelCount = 1;
        channels.resize(channelCount);

        ChannelInfo& channel = channels.front();

        channel.carrierFrequencyMhz = theParameterDatabaseReader.ReadDouble("channel-frequency-mhz", instanceId);
        channel.channelBandwidthMhz = theParameterDatabaseReader.ReadDouble("channel-bandwidth-mhz", instanceId);

    }//if//

    if (theParameterDatabaseReader.ParameterExists("number-data-parallel-threads-for-propagation")) {
        numberThreadsForDataParallelPropCalculation =
            theParameterDatabaseReader.ReadInt("number-data-parallel-threads-for-propagation");
    }//if//

    if (theParameterDatabaseReader.ParameterExists("min-propagated-signal-power-dbm", instanceId)) {
        ignoreSignalCutoffPowerDbm =
            theParameterDatabaseReader.ReadDouble("min-propagated-signal-power-dbm", instanceId);
    }//if//

    if (theParameterDatabaseReader.ParameterExists("max-signal-propagation-meters", instanceId)) {
        maximumPropagationDistanceMeters =
            theParameterDatabaseReader.ReadDouble("max-signal-propagation-meters", instanceId);
    }//if//

    maximumPropagationDelay = 1 * SECOND;
    if (theParameterDatabaseReader.ParameterExists("max-signal-propagation-delay", instanceId)) {
        maximumPropagationDelay =
            theParameterDatabaseReader.ReadTime("max-signal-propagation-delay", instanceId);
    }//if//


    if (theParameterDatabaseReader.ParameterExists("enable-propagation-delay", instanceId)) {
        propagationDelayIsEnabled = theParameterDatabaseReader.ReadBool("enable-propagation-delay", instanceId);
    }//if//


    string propModelName = "";

    if (theParameterDatabaseReader.ParameterExists("propagation-model", instanceId)) {

        propModelName = theParameterDatabaseReader.ReadString("propagation-model", instanceId);

    }//if//

    if ((theParameterDatabaseReader.ParameterExists(
        "propagation-allow-multiple-interfaces-on-same-channel", instanceId)) &&
        (theParameterDatabaseReader.ReadBool(
            "propagation-allow-multiple-interfaces-on-same-channel", instanceId))) {

        // For customer requirement.

        allowMultipleInterfacesOnSameChannel = true;
    }//if//

    const unsigned int THREAD_BASE_NODEID = 100000;

    ConvertStringToLowerCase(propModelName);

    bool isSymmetricSignalLoss = true;

    for(unsigned int channelIndex = 0; channelIndex < channels.size(); channelIndex++) {

        ChannelInfo& channel = channels.at(channelIndex);

        channel.propLossCalculationModelPtr =
            CreatePropagationLossCalculationModel(
                simulationEnginePtr,
                theParameterDatabaseReader,
                gisSubsystemPtr,
                propModelName,
                channel.carrierFrequencyMhz,
                maximumPropagationDistanceMeters,
                propagationDelayIsEnabled,
                numberThreadsForDataParallelPropCalculation,
                instanceId);

        if (!channel.propLossCalculationModelPtr->PropagationLossIsSymmetricValue()) {
            isSymmetricSignalLoss = false;
        }
    }//for//

    for(unsigned int partIndex = 0; (partIndex < propagationPartitions.size()); partIndex++) {
        propagationPartitions[partIndex] =
            shared_ptr<PropagationPartitionInfo>(new PropagationPartitionInfo());

        propagationPartitions[partIndex]->simulationInterfacePtr =
            simulationEnginePtr->GetSimulationEngineInterface(
            theParameterDatabaseReader,
            NodeIdType(THREAD_BASE_NODEID + partIndex), partIndex);

        propagationPartitions[partIndex]->perChannelNodeList.resize(channelCount);

        if (!isSymmetricSignalLoss) {
            propagationPartitions[partIndex]->aSignalLossCache.DisableSymmetricSignalLossCache();
        }
    }//for//

    // Set fading model

    if (theParameterDatabaseReader.ParameterExists("fading-model", instanceId)) {

        const string fadingModelStringInLowerCase =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("fading-model", instanceId));

        if ((fadingModelStringInLowerCase == "rayleigh") || (fadingModelStringInLowerCase == "nakagami")) {

            for(unsigned int channelIndex = 0; channelIndex < channels.size(); channelIndex++) {

                ChannelInfo& channel = channels.at(channelIndex);

                channel.fadingModelPtr =
                    shared_ptr<FadingModel>(
                        new FadingModel(
                            fadingModelStringInLowerCase,
                            theParameterDatabaseReader,
                            runSeed,
                            instanceId,
                            channels.at(channelIndex).carrierFrequencyMhz,
                            channels.at(channelIndex).channelBandwidthMhz));
            }//for//

            bool fixedRelativeVelocity = false;
            if (theParameterDatabaseReader.ParameterExists("fading-enable-fixed-velocity",instanceId)) {
                fixedRelativeVelocity =
                    theParameterDatabaseReader.ReadBool("fading-enable-fixed-velocity",instanceId);
            }//if//

            if(!fixedRelativeVelocity) {
                //dynamic velocity update
                velocityUpdateInterval = 1 * SECOND;
                if(theParameterDatabaseReader.ParameterExists("fading-velocity-update-interval",instanceId)) {
                    velocityUpdateInterval =
                        theParameterDatabaseReader.ReadTime("fading-velocity-update-interval", instanceId);
                }//if//
            }//if//

        }
        else if (fadingModelStringInLowerCase == "off") {
            //do nothing
        }
        else {
            cerr << "Error: Fading Model: " << fadingModelStringInLowerCase << " is invalid." << endl;
            exit(1);
        }//if//

    }//if//

    if (theParameterDatabaseReader.ParameterExists("shadowing-model", instanceId)) {
        const string shadowingModelString =
            MakeLowerCaseString(theParameterDatabaseReader.ReadString("shadowing-model", instanceId));

        if (shadowingModelString != "simplelognormal") {
            cerr << "Error: Shadowing Model: " << shadowingModelString << " is invalid." << endl;
            exit(1);
        }//if//

        linkShadowingModelPtr.reset(new LinkShadowingModel(theParameterDatabaseReader, runSeed));

    }//if//


    if (theParameterDatabaseReader.ParameterExists("propagation-channel-interference-matrix", instanceId)) {
        channelInterferenceModellingIsOn = true;

        string matrixString =
            theParameterDatabaseReader.ReadString("propagation-channel-interference-matrix", instanceId);

        DeleteTrailingSpaces(matrixString);

        istringstream matrixStream(matrixString);

        channelInterferenceMatrix.resize(channels.size(), vector<double>(channels.size(), 0.0));

        for(unsigned int i = 0; (i < channels.size()); i++) {
            for(unsigned int j = 0; (j < channels.size()); j++) {
                matrixStream >> channelInterferenceMatrix[i][j];

                if (i == j) {
                    if (channelInterferenceMatrix[i][j] != 1.0) {
                        cerr << "Error: propagation-channel-interference-matrix parameter: "
                             << "diagonal entries should be 1.0." << endl;
                        exit(1);
                    }//if//

                    // Should never be used
                    channelInterferenceMatrix[i][j] = 0.0;
                }//if//
            }//for//
        }//for//

        if ((matrixStream.fail()) || !matrixStream.eof()) {
            cerr << "Error: propagation-channel-interference-matrix parameter is invalid or too short." << endl;
            exit(1);
        }//if/

        if (!matrixStream.eof()) {
            cerr << "Error: propagation-channel-interference-matrix parameter is invalid or too long." << endl;
            exit(1);
        }//if//
    }//if//


    if ((theParameterDatabaseReader.ParameterExists(
            "propagation-enable-mask-calculated-channel-interference", instanceId)) &&
        (theParameterDatabaseReader.ReadBool("propagation-enable-mask-calculated-channel-interference", instanceId))) {

        if (channelInterferenceModellingIsOn) {
            cerr << "Error in Configuration File: Both propagation-channel-interference-matrix and" << endl;
            cerr << "     propagation-enable-mask-calculated-channel-interference enabled." << endl;
            exit(1);
        }//if//

        (*this).CalculateChannelInterferenceFromSpectralMasks(theParameterDatabaseReader);

    }//if//


}//InitializeSimplePropagationModel//



template<typename FrameType> inline
SimplePropagationModel<FrameType>::PropagationPartitionInfo::~PropagationPartitionInfo()
{
}//~SimplePropagationModel()//



template<typename FrameType> inline
bool SimplePropagationModel<FrameType>::ChannelIsBeingUsed(const unsigned int channelNumber) const
{
    // Only used for Emulation with Distributed Partitions not Thread Partitions.
    assert(propagationPartitions.size() == 1);

    const PropagationPartitionInfo& partitionInfo = *(propagationPartitions[0]);
    const unsigned int channelIndex = channelNumber - baseChannelNumber;
    return (!partitionInfo.perChannelNodeList.at(channelIndex).empty());

}//ChannelIsBeingUsed//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::AddNodeToChannel(
    const unsigned int channelNumber,
    const shared_ptr<SimplePropagationModelForNode<FrameType> >& interfacePtr)
{
    if (interfacePtr->IAmNotReceivingSignals()) {
        // Sending only.
        return;
    }//if//

    const unsigned int channelIndex = channelNumber - baseChannelNumber;

    const unsigned int partitionIndex = interfacePtr->CurrentThreadPartitionIndex();

    PropagationPartitionInfo& partitionInfo = *(propagationPartitions[partitionIndex]);

    assert(channelIndex < partitionInfo.perChannelNodeList.size());

    vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >& nodeList =
        partitionInfo.perChannelNodeList.at(channelIndex);

    CheckThatNodeOrInterfaceIsNotAlreadyOnChannel(nodeList, interfacePtr);

    nodeList.push_back(interfacePtr);

}//AddNodeToChannel//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::InvalidateCachedInformationFor(
    const SimplePropagationModelForNode<FrameType>& nodeInfo)
{
    const unsigned int partitionIndex = nodeInfo.CurrentThreadPartitionIndex();
    propagationPartitions[partitionIndex]->aSignalLossCache.InvalidateCachedInformationFor(
        nodeInfo.GetNodeId());
}



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::DeleteNodeFromChannel(
    const unsigned int channelNumber,
    const shared_ptr<SimplePropagationModelForNode<FrameType> >& nodeInterfacePtr)
{
    const unsigned int partitionIndex = nodeInterfacePtr->CurrentThreadPartitionIndex();
    const unsigned int channelIndex = channelNumber - baseChannelNumber;

    PropagationPartitionInfo& partition = *propagationPartitions[partitionIndex];

    vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >& nodeList =
        partition.perChannelNodeList.at(channelIndex);

    for(unsigned int i = 0; (i < nodeList.size()); i++) {
        if (nodeList[i] == nodeInterfacePtr) {
            nodeList.erase(nodeList.begin() + i);
            break;
        }//if//
    }//for//

    partition.aSignalLossCache.ClearCacheInformationFor(nodeInterfacePtr->GetNodeId());

}//DeleteNodeFromChannel//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::DisconnectNodeInterface(
    const unsigned int channelNumber,
    const shared_ptr<SimplePropagationModelForNode<FrameType> >& nodeInterfacePtr)
{
    const unsigned int partitionIndex = nodeInterfacePtr->CurrentThreadPartitionIndex();

    (*this).DeleteNodeFromChannel(channelNumber, nodeInterfacePtr);

    propagationPartitions[partitionIndex]->interfaceCorpseList.push(nodeInterfacePtr);

}//DisconnectNodeInterface//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::StopReceivingSignalsAtNode(
    const unsigned int channelNumber,
    const shared_ptr<SimplePropagationModelForNode<FrameType> >& nodeInterfacePtr)
{
    const unsigned int partitionIndex = nodeInterfacePtr->CurrentThreadPartitionIndex();

    (*this).DeleteNodeFromChannel(channelNumber, nodeInterfacePtr);

    propagationPartitions[partitionIndex]->currentlyNotReceivingNodeList.push_back(nodeInterfacePtr);

}//StopReceivingSignalsAtNode//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::StartReceivingSignalsAtNode(
    const unsigned int channelNumber,
    const shared_ptr<SimplePropagationModelForNode<FrameType> >& nodeInterfacePtr)
{
    const unsigned int partitionIndex = nodeInterfacePtr->CurrentThreadPartitionIndex();

    PropagationPartitionInfo& partition = *propagationPartitions[partitionIndex];

    vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >& nodeList =
        partition.currentNotReceivingNodeList;

    for(unsigned int i = 0; (i < nodeList.size()); i++) {
        if (nodeList[i] == nodeInterfacePtr) {
            nodeList.erase(nodeList.begin() + i);
            break;
        }//if//
    }//for//

    (*this).AddNodeToChannel(nodeInterfacePtr, channelNumber);

}//StartReceivingSignalsAtNode//


template<typename FrameType> inline
bool SimplePropagationModel<FrameType>::FadingIsEnabled() const
{
    for(size_t i = 0; (i < channels.size()); i++) {
        if (channels[i].fadingModelPtr != nullptr) {
            return true;
        }//if//
    }//for//

    return false;

}//FadingIsEnabled//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::GetFadingResultValueDb(
    const TimeType& targetTime,
    const NodeIdType& txNodeId,
    const NodeIdType& rxNodeId,
    const unsigned int channelNumber,
    double& returnFadingValueDb) const
{
    assert((*this).FadingIsEnabled());
    const unsigned int channelIndex = channelNumber - baseChannelNumber;

    if(txNodeId < rxNodeId) {
        channels.at(channelIndex).fadingModelPtr->FadingResultValueDb(
            targetTime,
            txNodeId,
            rxNodeId,
            returnFadingValueDb);
    }
    else {
        channels.at(channelIndex).fadingModelPtr->FadingResultValueDb(
            targetTime,
            rxNodeId,
            txNodeId,
            returnFadingValueDb);
    }//if//

}//GetFadingResultValueDb//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::ScheduleSignalEventAtNode(
    SimulationEngineInterface& simEngineInterface,
    const NodeIdType& txNodeId,
    const vector<unsigned int>& channelNumbers,
    const shared_ptr<SimplePropagationModelForNode<FrameType> >& receivingNodeInfoPtr,
    const double& transmitPowerDbm,
    const double& receivedPowerDbm,
    const TimeType& currentTime,
    const TimeType& propagationDelay,
    const TimeType& duration,
    const shared_ptr<FrameType>& framePtr,
    const bool isANoiseFrame)
{
    //Fading effect
    double fadingPowerDb = 0.0;
    if (FadingIsEnabled()) {
        GetFadingResultValueDb(
            (currentTime + propagationDelay),
            txNodeId,
            receivingNodeInfoPtr->GetNodeId(),
            channelNumbers.front(),
            fadingPowerDb);
    }//if//

    // Shadowing Effect (TGax model only for now).

    double shadowingDb = 0.0;
    if (linkShadowingModelPtr != nullptr) {
        linkShadowingModelPtr->CalcShadowFadingDb(
            txNodeId,
            receivingNodeInfoPtr->GetNodeId(),
            shadowingDb);
    }//if//

    const double finalReceivedPowerDbm = receivedPowerDbm + fadingPowerDb - shadowingDb;

    const unsigned int partitionIndex = simEngineInterface.CurrentThreadPartitionIndex();

    PropagationPartitionInfo& partitionInfo = *propagationPartitions.at(partitionIndex);

    SingleSizeMemoryCache& signalMemoryCache = partitionInfo.incomingSignalMemoryCache;

    // Note: Using raw pointer here for performance reasons.

    IncomingSignal* anIncomingSignalPtr = nullptr;

    if (!signalsGetFramePtrSupportIsOn) {
        anIncomingSignalPtr =
            new(signalMemoryCache) IncomingSignal(
                channelNumbers,
                txNodeId,
                currentTime,
                duration,
                transmitPowerDbm,
                finalReceivedPowerDbm,
                framePtr.get(),
                isANoiseFrame);
    }
    else {
        anIncomingSignalPtr =
            new(signalMemoryCache) IncomingSignal(
                channelNumbers,
                txNodeId,
                currentTime,
                duration,
                transmitPowerDbm,
                finalReceivedPowerDbm,
                framePtr,
                isANoiseFrame);
    }//if//

    if (receivingNodeInfoPtr->signalHandlerPtr == nullptr) {
        cerr << "Error: A Phy model has not registered a signal start handler" << endl;
        exit(1);
    }//if//

    if (propagationDelay == ZERO_TIME) {
        // Give the signal to the node immediately.

        receivingNodeInfoPtr->ReceiveIncomingSignal(*anIncomingSignalPtr);
        if (receivingNodeInfoPtr->signalEndHandlerPtr == nullptr) {
            signalMemoryCache.Delete(anIncomingSignalPtr);
        }//if//
    }
    else {
        // Schedule signal to show up after propagation delay.

        TransparentSingleSizeMemoryCache& eventMemoryCache = partitionInfo.eventMemoryCache;
        simEngineInterface.ScheduleEvent(
            unique_ptr<SimulationEvent>(
                new(eventMemoryCache) SignalStartEvent(
                    this, receivingNodeInfoPtr.get(), anIncomingSignalPtr)),
            (currentTime + propagationDelay));
    }//if//

    if (receivingNodeInfoPtr->signalEndHandlerPtr != nullptr) {
        TransparentSingleSizeMemoryCache& eventMemoryCache = partitionInfo.eventMemoryCache;

        simEngineInterface.ScheduleEvent(
            unique_ptr<SimulationEvent>(
                new(eventMemoryCache) SignalEndEvent(
                    this, receivingNodeInfoPtr.get(), anIncomingSignalPtr)),
            (currentTime + propagationDelay + duration));

    }//if//

}//ScheduleSignalEventAtNode//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::DeleteIncomingSignal(
    SimplePropagationModelForNode<FrameType>& receivingNodeInfo,
    IncomingSignal* aSignalPtr)
{
    if (!receivingNodeInfo.IAmDisconnected()) {
        unsigned int partitionIndex = receivingNodeInfo.CurrentThreadPartitionIndex();
        propagationPartitions.at(partitionIndex)->incomingSignalMemoryCache.Delete(aSignalPtr);
    }
    else {
        delete aSignalPtr;
        aSignalPtr = nullptr;
    }//if//

}//DeleteIncomingSignal//



template<typename FrameType> inline
bool SimplePropagationModel<FrameType>::NodeCanHearSignal(
    const SimplePropagationModelForNode<FrameType>& receivingNodeInfo,
    const IncomingSignal& aSignal) const
{
    if (receivingNodeInfo.CurrentlyReceivingFramesOnBondedChannels()) {

        return (!aSignal.ChannelIntersectionIsEmpty(receivingNodeInfo.GetCurrentChannelNumberSet()));
    }
    else {
        if (aSignal.GetNumberBondedChannels() == 1) {
            return (aSignal.GetChannelNumber() == receivingNodeInfo.GetCurrentChannelNumber());
        }
        else {
            return (aSignal.IsOnChannel(receivingNodeInfo.GetCurrentChannelNumber()));
        }//if//

    }//if//

}//CanHearSignal//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::SendSignalStartEventToNode(
    SimplePropagationModelForNode<FrameType>& receivingNodeInfo,
    IncomingSignal* aSignalPtr)
{
    if ((!receivingNodeInfo.IAmDisconnected()) &&
        ((receivingNodeInfo.GetLastChannelSwitchTime() < aSignalPtr->GetStartTime()) ||
         ((receivingNodeInfo.GetLastChannelSwitchTime() == aSignalPtr->GetStartTime()) &&
          (NodeCanHearSignal(receivingNodeInfo, *aSignalPtr))))) {

        assert(NodeCanHearSignal(receivingNodeInfo, *aSignalPtr));

        receivingNodeInfo.ReceiveIncomingSignal(*aSignalPtr);

    }//if//

    if (!receivingNodeInfo.hasAnEndEventHandler) {

        // Delete signal datastructure if not need for signal end event.

        (*this).DeleteIncomingSignal(receivingNodeInfo, aSignalPtr);

    }//if//

}//SendSignalStartEventToNode//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::SendSignalEndEventToNode(
    SimplePropagationModelForNode<FrameType>& receivingNodeInfo,
    IncomingSignal* aSignalPtr)
{
    if ((!receivingNodeInfo.IAmDisconnected()) &&
        ((receivingNodeInfo.GetLastChannelSwitchTime() < aSignalPtr->GetStartTime()) ||
         ((receivingNodeInfo.GetLastChannelSwitchTime() == aSignalPtr->GetStartTime()) &&
          (NodeCanHearSignal(receivingNodeInfo, *aSignalPtr))))) {

        assert(NodeCanHearSignal(receivingNodeInfo, *aSignalPtr));

        receivingNodeInfo.ReceiveIncomingSignalEnd(*aSignalPtr);
    }//if//

    (*this).DeleteIncomingSignal(receivingNodeInfo, aSignalPtr);

}//SendSignalEndEventToNode//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::TransmitSignalToSingleNode(
    SimulationEngineInterface& simEngineInterface,
    const NodeIdType& txNodeId,
    const unsigned int txInterfaceIndex,
    const vector<unsigned int>& channelNumbers,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntennaModel,
    const double& transmitPowerDbm,
    const TimeType& currentTime,
    const TimeType& duration,
    const shared_ptr<FrameType>& framePtr,
    const shared_ptr<SimplePropagationModelForNode<FrameType> >&  receivingNodeInfoPtr,
    const bool isANoiseFrame)
{
    assert(!channelNumbers.empty());

    double totalLossDb = 0.0;
    TimeType propagationDelay = INFINITE_TIME;

    const unsigned int txChannelNumber = channelNumbers[0];

    const unsigned int propagationPartitionIndex = simEngineInterface.CurrentThreadPartitionIndex();
    SignalLossCache& aSignalLossCache = propagationPartitions.at(propagationPartitionIndex)->aSignalLossCache;
    const ChannelInfo& channel = channels.at(txChannelNumber - baseChannelNumber);

    const SimplePropagationModelForNode<FrameType>& rxNode = *receivingNodeInfoPtr;

    channel.propLossCalculationModelPtr->CalculateOrRetrieveTotalLossDbAndPropDelay(
        aSignalLossCache, currentTime, txChannelNumber, txNodeId, txAntennaPosition, txAntennaModel,
        rxNode.GetNodeId(), *rxNode.antennaMobilityModelPtr, rxNode.GetAntennaModel(),
        totalLossDb, propagationDelay);

    if (!propagationDelayIsEnabled) {
        propagationDelay = ZERO_TIME;
    }//if//

    double receivedPowerDbm = -DBL_MAX;
    if (totalLossDb != InfinitePathlossDb) {
        receivedPowerDbm = transmitPowerDbm - totalLossDb;
    }//if//

    if (receivedPowerDbm > ignoreSignalCutoffPowerDbm) {
        (*this).ScheduleSignalEventAtNode(
            simEngineInterface,
            txNodeId,
            channelNumbers,
            receivingNodeInfoPtr,
            transmitPowerDbm,
            receivedPowerDbm,
            currentTime,
            propagationDelay,
            duration,
            framePtr,
            isANoiseFrame);
    }//if//

}//TransmitSignalToSingleNode//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::TransmitChannelInterferenceSignals(
    SimulationEngineInterface& simEngineInterface,
    const NodeIdType& txNodeId,
    const unsigned int txInterfaceIndex,
    const unsigned int txChannelNumber,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntennaModel,
    const double& transmitPowerDbm,
    const TimeType& currentTime,
    const TimeType& duration)
{
    typedef typename vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >::iterator IterType;

    const unsigned int propagationPartitionIndex = simEngineInterface.CurrentThreadPartitionIndex();

    SignalLossCache& aSignalLossCache = propagationPartitions.at(propagationPartitionIndex)->aSignalLossCache;

    assert(txChannelNumber >= baseChannelNumber);
    const unsigned int txChannelIndex = txChannelNumber - baseChannelNumber;

    for(unsigned int channelIndex = 0; (channelIndex < channelCount); channelIndex++) {

        const double channelInterference = channelInterferenceMatrix[txChannelIndex][channelIndex];

        if ((channelIndex == txChannelIndex) || (channelInterference == 0.0)) {
            //skip channel
            continue;
        }//if//

        const unsigned int rxChannelNumber = baseChannelNumber + channelIndex;

        const double adjustedTransmitPowerDbm = transmitPowerDbm + ConvertToDb(channelInterference);

        vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >& nodeList =
            propagationPartitions.at(propagationPartitionIndex)->perChannelNodeList.at(channelIndex);

        const ChannelInfo& channel = channels.at(channelIndex);

        for(IterType nodeIter = nodeList.begin(); (nodeIter != nodeList.end()); nodeIter++) {

            if (((*nodeIter)->GetNodeId() != txNodeId) ||
                ((allowMultipleInterfacesOnSameChannel) &&
                 ((*nodeIter)->GetInterfaceIndex() != txInterfaceIndex))) {

               (*this).TransmitSignalToSingleNode(
                    simEngineInterface, txNodeId, txInterfaceIndex,
                    rxChannelNumber, txAntennaPosition, txAntennaModel,
                    adjustedTransmitPowerDbm, currentTime, duration,
                    shared_ptr<FrameType>(/*nullptr*/), *nodeIter, true);
            }//if//
        }//for//
    }//for//

}//TransmitChannelInterferenceSignals//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::TransmitSignalInLocalPartition(
    SimulationEngineInterface& simEngineInterface,
    const NodeIdType& txNodeId,
    const unsigned int txInterfaceIndex,
    const unsigned int txChannelNumber,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntennaModel,
    const double& transmitPowerDbm,
    const TimeType& currentTime,
    const TimeType& duration,
    const shared_ptr<FrameType>& framePtr)
{
    if (numberThreadsForDataParallelPropCalculation > 1) {

        (*this).TransmitSignalInLocalPartitionUtilizingMultipleThreads(
            simEngineInterface,
            txNodeId,
            txInterfaceIndex,
            txChannelNumber,
            txAntennaPosition,
            txAntennaModel,
            transmitPowerDbm,
            currentTime,
            duration,
            framePtr);
    }
    else {
        typedef typename vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >::iterator IterType;

        const unsigned int propagationPartitionIndex = simEngineInterface.CurrentThreadPartitionIndex();

        SignalLossCache& aSignalLossCache = propagationPartitions.at(propagationPartitionIndex)->aSignalLossCache;

        assert(txChannelNumber >= baseChannelNumber);
        const unsigned int channelIndex = txChannelNumber - baseChannelNumber;

        vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >& nodeList =
            propagationPartitions.at(propagationPartitionIndex)->perChannelNodeList.at(channelIndex);

        const ChannelInfo& channel = channels.at(txChannelNumber - baseChannelNumber);

        if (channel.propLossCalculationModelPtr->SupportMultipointCalculation()) {
            channel.propLossCalculationModelPtr->CacheMultipointPropagationLossDb();
        }//if//


        for(IterType nodeIter = nodeList.begin(); (nodeIter != nodeList.end()); nodeIter++) {

            if (((*nodeIter)->GetNodeId() != txNodeId) ||
                ((allowMultipleInterfacesOnSameChannel) &&
                 ((*nodeIter)->GetInterfaceIndex() != txInterfaceIndex))) {

                (*this).TransmitSignalToSingleNode(
                    simEngineInterface, txNodeId, txInterfaceIndex,
                    txChannelNumber, txAntennaPosition, txAntennaModel,
                    transmitPowerDbm, currentTime, duration, framePtr, *nodeIter);
            }//if//
        }//for//

        if (channelInterferenceModellingIsOn) {
            (*this).TransmitChannelInterferenceSignals(
                    simEngineInterface, txNodeId, txInterfaceIndex,
                    txChannelNumber, txAntennaPosition, txAntennaModel,
                    transmitPowerDbm, currentTime, duration);
        }//if//
    }//if//

}//TransmitSignalInLocalPartition//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::TransmitSignalInLocalPartition(
    SimulationEngineInterface& simEngineInterface,
    const NodeIdType& txNodeId,
    const unsigned int txInterfaceIndex,
    const vector<unsigned int>& channelNumbers,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntennaModel,
    const double& transmitPowerDbm,
    const TimeType& currentTime,
    const TimeType& duration,
    const shared_ptr<FrameType>& framePtr)
{
    assert(!channelNumbers.empty());
    assert(VectorIsStrictlyIncreasing(channelNumbers));

    if (numberThreadsForDataParallelPropCalculation > 1) {

        (*this).TransmitSignalInLocalPartitionUtilizingMultipleThreads(
            simEngineInterface,
            txNodeId,
            txInterfaceIndex,
            channelNumbers,
            txAntennaPosition,
            txAntennaModel,
            transmitPowerDbm,
            currentTime,
            duration,
            framePtr);
    }
    else {
        typedef typename vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >::iterator IterType;

        const unsigned int propagationPartitionIndex = simEngineInterface.CurrentThreadPartitionIndex();

        SignalLossCache& aSignalLossCache = propagationPartitions.at(propagationPartitionIndex)->aSignalLossCache;

        const ChannelInfo& firstChannel = channels.at(channelNumbers.front() - baseChannelNumber);

        if (firstChannel.propLossCalculationModelPtr->SupportMultipointCalculation()) {
            firstChannel.propLossCalculationModelPtr->CacheMultipointPropagationLossDb();
        }//if//

        vector<unsigned int> processedChannels;

        for(unsigned int i = 0; (i < channelNumbers.size()); i++) {

            const unsigned int txChannelNumber = channelNumbers[i];
            assert(txChannelNumber >= baseChannelNumber);
            const unsigned int channelIndex = txChannelNumber - baseChannelNumber;

            vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >& nodeList =
                propagationPartitions.at(propagationPartitionIndex)->perChannelNodeList.at(channelIndex);

            const ChannelInfo& channel = channels.at(txChannelNumber - baseChannelNumber);

            for(IterType nodeIter = nodeList.begin(); (nodeIter != nodeList.end()); nodeIter++) {
                SimplePropagationModelForNode<FrameType>& aNode = (**nodeIter);

                if ((IntersectionIsEmpty(processedChannels, aNode.GetCurrentChannelNumberSet())) &&
                    ((aNode.GetNodeId() != txNodeId) ||
                     ((allowMultipleInterfacesOnSameChannel) &&
                     (aNode.GetInterfaceIndex() != txInterfaceIndex)))) {

                    (*this).TransmitSignalToSingleNode(
                        simEngineInterface, txNodeId, txInterfaceIndex,
                        channelNumbers, txAntennaPosition, txAntennaModel,
                        transmitPowerDbm, currentTime, duration, framePtr, *nodeIter);
                }//if//
            }//for//

            if (channelInterferenceModellingIsOn) {
                (*this).TransmitChannelInterferenceSignals(
                        simEngineInterface, txNodeId, txInterfaceIndex,
                        txChannelNumber, txAntennaPosition, txAntennaModel,
                        transmitPowerDbm, currentTime, duration);
            }//if//

            processedChannels.push_back(txChannelNumber);

        }//for//

    }//if//

}//TransmitSignalInLocalPartition//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::TransmitSignalInLocalPartitionUtilizingMultipleThreads(
    SimulationEngineInterface& simEngineInterface,
    const NodeIdType& txNodeId,
    const unsigned int txInterfaceIndex,
    const unsigned int txChannelNumber,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntennaModel,
    const double& transmitPowerDbm,
    const TimeType& currentTime,
    const TimeType& duration,
    const shared_ptr<FrameType>& framePtr)
{
    // Transmit to nodes with cached propagation values and build a list of propagation
    // calculations to perform.

    const unsigned int propagationPartitionIndex = simEngineInterface.CurrentThreadPartitionIndex();

    SignalLossCache& aSignalLossCache = propagationPartitions.at(propagationPartitionIndex)->aSignalLossCache;

    assert(txChannelNumber >= baseChannelNumber);
    const unsigned int channelIndex = txChannelNumber - baseChannelNumber;

    vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >& nodeList =
        propagationPartitions.at(propagationPartitionIndex)->perChannelNodeList.at(channelIndex);

    const ChannelInfo& channel = channels.at(txChannelNumber - baseChannelNumber);

    const shared_ptr<SimplePropagationLossCalculationModel>& propLossCalculationModelPtr =
        channel.propLossCalculationModelPtr;

    propLossCalculationModelPtr->ClearParallelCalculationSet();

    for(unsigned int nodeIndex = 0; (nodeIndex < nodeList.size()); nodeIndex++) {
        SimplePropagationModelForNode<FrameType>& rxNodeData = *nodeList[nodeIndex];

        if ((rxNodeData.GetNodeId() != txNodeId) ||
            ((allowMultipleInterfacesOnSameChannel) &&
             (rxNodeData.GetInterfaceIndex() != txInterfaceIndex))) {

            bool cachedValueFound = false;
            TimeType propagationDelay = INFINITE_TIME;
            double totalLossDb = 0.0;

            aSignalLossCache.GetCachedValue(
                txNodeId, rxNodeData.GetNodeId(), txChannelNumber, currentTime,
                cachedValueFound, totalLossDb, propagationDelay);

            if (cachedValueFound) {
                double receivedPowerDbm = transmitPowerDbm - totalLossDb;

                if (receivedPowerDbm > ignoreSignalCutoffPowerDbm) {

                    (*this).ScheduleSignalEventAtNode(
                        simEngineInterface,
                        txNodeId,
                        txChannelNumber,
                        nodeList[nodeIndex],
                        transmitPowerDbm,
                        receivedPowerDbm,
                        currentTime,
                        propagationDelay,
                        duration,
                        framePtr);
                }//if//
            }
            else {
                const ObjectMobilityPosition rxAntennaPosition = rxNodeData.GetCurrentMobilityPosition();

                if (propLossCalculationModelPtr->IsCloserThanMaxPropagationDistance(txAntennaPosition, rxAntennaPosition)) {

                    // Note: A raw pointer is taken to the TX antenna model, but this pointer will only
                    //       be kept during the data parallel run (nulled by ClearParallelCalculationSet()
                    //       call below).

                    propLossCalculationModelPtr->AddToParallelCalculationSet(
                        txAntennaPosition,
                        &txAntennaModel,
                        nodeIndex,
                        rxAntennaPosition,
                        rxNodeData.GetAntennaModelPtr());
                }//if//
            }//if//
        }//if//
    }//for//


    propLossCalculationModelPtr->CalculateLossesDbAndPropDelaysInParallel();

    for(unsigned int i = 0; (i < propLossCalculationModelPtr->GetNumberOfParallelCalculations()); i++) {
        const unsigned int rxNodeIndex = propLossCalculationModelPtr->GetRxNodeIndex(i);
        const double totalLossDb = propLossCalculationModelPtr->GetTotalLossDb(i);
        const TimeType propagationDelay = propLossCalculationModelPtr->GetPropagationDelay(i);
        const ObjectMobilityPosition& rxAntennaPosition = propLossCalculationModelPtr->GetRxAntennaPosition(i);

        aSignalLossCache.CacheTheValue(
            txNodeId,
            nodeList.at(rxNodeIndex)->GetNodeId(),
            txChannelNumber,
            totalLossDb,
            propagationDelay,
            std::min(
                txAntennaPosition.EarliestNextMoveTime(),
                rxAntennaPosition.EarliestNextMoveTime()));

        const double receivedPowerDbm = transmitPowerDbm - totalLossDb;

        if (receivedPowerDbm > ignoreSignalCutoffPowerDbm) {
            (*this).ScheduleSignalEventAtNode(
                simEngineInterface,
                txNodeId,
                txChannelNumber,
                nodeList.at(rxNodeIndex),
                transmitPowerDbm,
                receivedPowerDbm,
                currentTime,
                propagationDelay,
                duration,
                framePtr);
        }//if//
    }//for//

    propLossCalculationModelPtr->ClearParallelCalculationSet();

}//TransmitSignalInLocalPartitionUtilizingMultipleThreads//




template<typename FrameType> inline
void SimplePropagationModel<FrameType>::TransmitSignalInLocalPartitionUtilizingMultipleThreads(
    SimulationEngineInterface& simEngineInterface,
    const NodeIdType& txNodeId,
    const unsigned int txInterfaceIndex,
    const vector<unsigned int>& channelNumbers,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntennaModel,
    const double& transmitPowerDbm,
    const TimeType& currentTime,
    const TimeType& duration,
    const shared_ptr<FrameType>& framePtr)
{
    // Transmit to nodes with cached propagation values and build a list of propagation
    // calculations to perform.


    //typedef typename vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >::iterator IterType;

    const unsigned int propagationPartitionIndex = simEngineInterface.CurrentThreadPartitionIndex();

    SignalLossCache& aSignalLossCache = propagationPartitions.at(propagationPartitionIndex)->aSignalLossCache;

    const ChannelInfo& firstChannel = channels.at(channelNumbers.front() - baseChannelNumber);

    if (firstChannel.propLossCalculationModelPtr->SupportMultipointCalculation()) {
        firstChannel.propLossCalculationModelPtr->CacheMultipointPropagationLossDb();
    }//if//

    vector<unsigned int> processedChannels;

    for(unsigned int i = 0; (i < channelNumbers.size()); i++) {

        const unsigned int txChannelNumber = channelNumbers[i];
        assert(txChannelNumber >= baseChannelNumber);
        const unsigned int channelIndex = txChannelNumber - baseChannelNumber;

        vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >& nodeList =
            propagationPartitions.at(propagationPartitionIndex)->perChannelNodeList.at(channelIndex);

        const ChannelInfo& channel = channels.at(txChannelNumber - baseChannelNumber);

        const shared_ptr<SimplePropagationLossCalculationModel>& propLossCalculationModelPtr =
            channel.propLossCalculationModelPtr;

        propLossCalculationModelPtr->ClearParallelCalculationSet();

        for(unsigned int nodeIndex = 0; (nodeIndex < nodeList.size()); nodeIndex++) {
            SimplePropagationModelForNode<FrameType>& rxNodeData = *nodeList[nodeIndex];

            if ((IntersectionIsEmpty(processedChannels, rxNodeData.GetCurrentChannelNumberSet())) &&
                ((rxNodeData.GetNodeId() != txNodeId) ||
                 ((allowMultipleInterfacesOnSameChannel) &&
                 (rxNodeData.GetInterfaceIndex() != txInterfaceIndex)))) {

                bool cachedValueFound = false;
                TimeType propagationDelay = INFINITE_TIME;
                double totalLossDb = 0.0;

                aSignalLossCache.GetCachedValue(
                    txNodeId, rxNodeData.GetNodeId(), txChannelNumber, currentTime,
                    cachedValueFound, totalLossDb, propagationDelay);

                if (cachedValueFound) {
                    double receivedPowerDbm = transmitPowerDbm - totalLossDb;

                    if (receivedPowerDbm > ignoreSignalCutoffPowerDbm) {

                        (*this).ScheduleSignalEventAtNode(
                            simEngineInterface,
                            txNodeId,
                            channelNumbers,
                            nodeList[nodeIndex],
                            transmitPowerDbm,
                            receivedPowerDbm,
                            currentTime,
                            propagationDelay,
                            duration,
                            framePtr);
                    }//if//
                }
                else {
                    const ObjectMobilityPosition rxAntennaPosition = rxNodeData.GetCurrentMobilityPosition();

                    if (propLossCalculationModelPtr->IsCloserThanMaxPropagationDistance(txAntennaPosition, rxAntennaPosition)) {

                        // Note: A raw pointer is taken to the TX antenna model, but this pointer will only
                        //       be kept during the data parallel run (nulled by ClearParallelCalculationSet()
                        //       call below).

                        propLossCalculationModelPtr->AddToParallelCalculationSet(
                            txAntennaPosition,
                            &txAntennaModel,
                            nodeIndex,
                            rxAntennaPosition,
                            rxNodeData.GetAntennaModelPtr());
                    }//if//
                }//if//
            }//if//
        }//for//


        propLossCalculationModelPtr->CalculateLossesDbAndPropDelaysInParallel();


        for(unsigned int i = 0; (i < propLossCalculationModelPtr->GetNumberOfParallelCalculations()); i++) {
            const unsigned int rxNodeIndex = propLossCalculationModelPtr->GetRxNodeIndex(i);
            const double totalLossDb = propLossCalculationModelPtr->GetTotalLossDb(i);
            const TimeType propagationDelay = propLossCalculationModelPtr->GetPropagationDelay(i);
            const ObjectMobilityPosition& rxAntennaPosition = propLossCalculationModelPtr->GetRxAntennaPosition(i);

            aSignalLossCache.CacheTheValue(
                txNodeId,
                nodeList.at(rxNodeIndex)->GetNodeId(),
                txChannelNumber,
                totalLossDb,
                propagationDelay,
                std::min(
                    txAntennaPosition.EarliestNextMoveTime(),
                    rxAntennaPosition.EarliestNextMoveTime()));

            const double receivedPowerDbm = transmitPowerDbm - totalLossDb;

            if (receivedPowerDbm > ignoreSignalCutoffPowerDbm) {
                (*this).ScheduleSignalEventAtNode(
                    simEngineInterface,
                    txNodeId,
                    channelNumbers,
                    nodeList.at(rxNodeIndex),
                    transmitPowerDbm,
                    receivedPowerDbm,
                    currentTime,
                    propagationDelay,
                    duration,
                    framePtr);
            }//if//
        }//for//

        propLossCalculationModelPtr->ClearParallelCalculationSet();
    }//for//

}//TransmitSignalInLocalPartitionUtilizingMultipleThreads//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::CalculatePathlossToNode(
    const PropagationInformationType& informationType,
    const ObjectMobilityPosition& txAntennaPosition,
    const AntennaModel& txAntennaModel,
    const ObjectMobilityPosition& rxAntennaPosition,
    const AntennaModel& rxAntennaModel,
    const unsigned int channelNumber,
    PropagationStatisticsType& propagationStatistics) const
{
    const ChannelInfo& channel = channels.at(channelNumber-baseChannelNumber);

    channel.propLossCalculationModelPtr->CalculatePropagationPathInformation(
        informationType,
        txAntennaPosition,
        txAntennaModel,
        rxAntennaPosition,
        rxAntennaModel,
        propagationStatistics);

}//CalculatePathlossToLocation//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::CalculatePathlossToLocation(
    const PropagationInformationType& informationType,
    const TimeType& currentTime,
    const SimplePropagationModelForNode<FrameType>& transmittingNodeInfo,
    const double& positionXMeters,
    const double& positionYMeters,
    const double& positionZMeters,
    const unsigned int channelNumber,
    PropagationStatisticsType& propagationStatistics)
{
    const bool theHeightContainsGroundHeightMeters = true;
    double adjustedPositionZMeters = positionZMeters;

    if ((informationType & PROPAGATION_INFORMATION_IS_VERTICAL_PATHLOSS) != PROPAGATION_INFORMATION_IS_VERTICAL_PATHLOSS) {
        // add ground height for horizontal propagation calculation.

        adjustedPositionZMeters +=
            gisSubsystemPtr->GetGroundElevationMetersAt(positionXMeters, positionYMeters);
    }

    // Put fake RX antenna at the position.
    ObjectMobilityPosition rxAntennaPosition(
        0, 0, positionXMeters, positionYMeters, adjustedPositionZMeters,
        theHeightContainsGroundHeightMeters, 0);// objectId sets 0

    OmniAntennaModel rxAntennaModel(0.0);

    (*this).CalculatePathlossToNode(
        informationType,
        transmittingNodeInfo.GetCurrentMobilityPosition(),
        transmittingNodeInfo.GetAntennaModel(),
        rxAntennaPosition,
        rxAntennaModel,
        channelNumber,
        propagationStatistics);

}//CalculatePathlossToLocation//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::SwitchToChannelNumber(
    const shared_ptr<SimplePropagationModelForNode<FrameType> >& receivingNodeInfoPtr,
    const unsigned int channelNumber,
    const bool doNotCalcInterferenceLevel)
{
    typedef typename vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >::iterator IterType;

    assert((receivingNodeInfoPtr->currentChannelSet.empty()) && "Error mixed bonded and non-bonded channel modes");

    assert(channelNumber >= baseChannelNumber);
    assert(channelNumber < (baseChannelNumber + channelCount));

    const unsigned int propagationPartitionIndex = receivingNodeInfoPtr->CurrentThreadPartitionIndex();

    SimulationEngineInterface& simEngineInterface =
            *(propagationPartitions.at(propagationPartitionIndex)->simulationInterfacePtr);

    const TimeType currentTime = simEngineInterface.CurrentTime();

    if (receivingNodeInfoPtr->GetCurrentChannelNumber() != channelNumber) {

        if (receivingNodeInfoPtr->GetCurrentChannelNumber() != InvalidChannelNumber) {
            (*this).DeleteNodeFromChannel(receivingNodeInfoPtr->GetCurrentChannelNumber(), receivingNodeInfoPtr);
        }//if//
        (*this).AddNodeToChannel(channelNumber, receivingNodeInfoPtr);

        receivingNodeInfoPtr->lastChannelSwitchTime = currentTime;
    }//if//

    if (doNotCalcInterferenceLevel) {
        // Not needed for TDMA systems
        return;
    }//if//

    // Note: Parallel code rot! No parallel execution updates.

    const unsigned int channelIndex = channelNumber - baseChannelNumber;

    vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >& nodeList =
        propagationPartitions.at(propagationPartitionIndex)->perChannelNodeList.at(channelIndex);

    for(IterType nodeIter = nodeList.begin(); nodeIter != nodeList.end(); ++nodeIter) {

        SimplePropagationModelForNode<FrameType>& transmittingNodeInfo = (**nodeIter);

        assert(transmittingNodeInfo.currentChannelSet.empty());

        if ((transmittingNodeInfo.currentChannelNumber == channelNumber) &&
            (transmittingNodeInfo.GetLastTransmissionEndTime() != ZERO_TIME) &&
            (transmittingNodeInfo.GetLastTransmissionEndTime() >= currentTime)) {

            // Node transmits shortened noise signal.
            // Note Cheat: Noise signal is not accounted for if falls within propagation time
            //             after transmission has ended.

            vector<unsigned int> completeChannelNumberList;
            completeChannelNumberList.push_back(channelNumber);

            (*this).TransmitSignalToSingleNode(
                simEngineInterface,
                transmittingNodeInfo.GetNodeId(),
                transmittingNodeInfo.GetInterfaceIndex(),
                channelNumber,
                transmittingNodeInfo.GetCurrentMobilityPosition(),
                transmittingNodeInfo.GetAntennaModel(),
                transmittingNodeInfo.GetLastTransmissionPowerDbm(),
                currentTime,
                (transmittingNodeInfo.GetLastTransmissionEndTime() - currentTime),
                transmittingNodeInfo.GetCurrentlyTransmittingFramePtr(),
                receivingNodeInfoPtr,
                true);
        }//if//
    }//for//

}//SwitchToChannelNumber//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::SwitchToChannelNumbers(
    const shared_ptr<SimplePropagationModelForNode<FrameType> >& receivingNodeInfoPtr,
    const vector<unsigned int>& channelNumberSet,
    const bool doNotCalcInterferenceLevel)
{
    typedef typename vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >::iterator IterType;

    if (!AllChannelsHaveSameChannelBandwidth()) {
        cerr << "Error in propagation model: All channels must have the same channel bandwidth." << endl;
        exit(1);
    }//if//

    assert(VectorIsStrictlyIncreasing(channelNumberSet));
    assert((receivingNodeInfoPtr->currentChannelNumber == InvalidChannelNumber) && "Error: Mixed bonded and non-bonded channel mode");
    assert(channelNumberSet != receivingNodeInfoPtr->currentChannelSet);

    const vector<unsigned int>& currentChannelNumberSet =
        receivingNodeInfoPtr->GetCurrentChannelNumberSet();

    const unsigned int propagationPartitionIndex = receivingNodeInfoPtr->CurrentThreadPartitionIndex();

    SimulationEngineInterface& simEngineInterface =
            *(propagationPartitions.at(propagationPartitionIndex)->simulationInterfacePtr);


    // Remove old channels.

    for(unsigned int i = 0; (i < currentChannelNumberSet.size()); i++) {
        const unsigned int channelNumber = currentChannelNumberSet[i];

        assert(channelNumber >= baseChannelNumber);
        assert(channelNumber < (baseChannelNumber + channelCount));

        if (std::find(channelNumberSet.begin(), channelNumberSet.end(), channelNumber) ==
            channelNumberSet.end()) {

            (*this).DeleteNodeFromChannel(channelNumber, receivingNodeInfoPtr);
        }//if//
    }//for//

    // Add new channels.

    const TimeType currentTime = simEngineInterface.CurrentTime();

    for(unsigned int i = 0; (i < channelNumberSet.size()); i++) {
        const unsigned int channelNumber = channelNumberSet[i];

        assert(channelNumber >= baseChannelNumber);
        assert(channelNumber < (baseChannelNumber + channelCount));

        if (std::find(currentChannelNumberSet.begin(), currentChannelNumberSet.end(), channelNumber) ==
            currentChannelNumberSet.end()) {

            (*this).AddNodeToChannel(channelNumber, receivingNodeInfoPtr);

            if (!doNotCalcInterferenceLevel) {

                // Note: Parallel Code rot! No parallel execution updates.

                const unsigned int channelIndex = channelNumber - baseChannelNumber;

                vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >& nodeList =
                    propagationPartitions.at(propagationPartitionIndex)->perChannelNodeList.at(channelIndex);

                for(IterType nodeIter = nodeList.begin(); nodeIter != nodeList.end(); ++nodeIter) {

                    SimplePropagationModelForNode<FrameType>& transmittingNodeInfo = (**nodeIter);

                    if ((transmittingNodeInfo.IsOnChannel(channelNumber)) &&
                        (transmittingNodeInfo.GetLastTransmissionEndTime() != ZERO_TIME) &&
                        (transmittingNodeInfo.GetLastTransmissionEndTime() >= currentTime)) {

                        // Node transmits shortened noise signal.
                        // Note Cheat: Noise signal is not accounted for if falls within propagation time
                        //             after transmission has ended.

                        (*this).TransmitSignalToSingleNode(
                            simEngineInterface,
                            transmittingNodeInfo.GetNodeId(),
                            transmittingNodeInfo.GetInterfaceIndex(),
                            channelNumberSet,
                            transmittingNodeInfo.GetCurrentMobilityPosition(),
                            transmittingNodeInfo.GetAntennaModel(),
                            transmittingNodeInfo.GetLastTransmissionPowerDbm(),
                            currentTime,
                            (transmittingNodeInfo.GetLastTransmissionEndTime() - currentTime),
                            transmittingNodeInfo.GetCurrentlyTransmittingFramePtr(),
                            receivingNodeInfoPtr,
                            true);
                    }//if//
                }//for//
            }//if//
        }//if//
    }//for//

    receivingNodeInfoPtr->currentChannelSet = channelNumberSet;
    receivingNodeInfoPtr->lastChannelSwitchTime = currentTime;

}//SwitchToChannelNumbers//



template<typename FrameType>
class InterPartitionSignalEvent: public SimulationEvent {
public:
    InterPartitionSignalEvent(
        SimplePropagationModel<FrameType>* initPropagationModelPtr,
        const unsigned int initDestinationPartitionIndex,
        const NodeIdType& initTxNodeId,
        const unsigned int initTxChannelNumber,
        const ObjectMobilityPosition& initTransmitAntennaPosition,
        const AntennaModel* initTransmitAntennaModelPtr,
        const double& initTransmitPowerDbm,
        const TimeType& initDuration,
        const shared_ptr<FrameType>& initFramePtr)
    :
        propagationModelPtr(initPropagationModelPtr),
        destinationPartitionIndex(initDestinationPartitionIndex),
        txNodeId(initTxNodeId),
        txChannelNumber(initTxChannelNumber),
        transmitAntennaPosition(initTransmitAntennaPosition),
        transmitAntennaModelPtr(initTransmitAntennaModelPtr),
        transmitPowerDbm(initTransmitPowerDbm),
        duration(initDuration),
        framePtr(initFramePtr)
    {}

    void ExecuteEvent() {
        propagationModelPtr->ReceiveSignal(
            destinationPartitionIndex,
            txNodeId,
            txChannelNumber,
            transmitAntennaPosition,
            *transmitAntennaModelPtr,
            transmitPowerDbm,
            duration,
            framePtr);
    }

    ~InterPartitionSignalEvent() { }

private:
    SimplePropagationModel<FrameType>* propagationModelPtr;
    unsigned int destinationPartitionIndex;
    NodeIdType txNodeId;
    unsigned int txChannelNumber;
    ObjectMobilityPosition transmitAntennaPosition;
    const AntennaModel* transmitAntennaModelPtr;
    double transmitPowerDbm;
    TimeType duration;
    const shared_ptr<FrameType>& framePtr;

};//InterPartitionSignalEvent//



//Parallel// template<typename FrameType> inline
//Parallel// void SimplePropagationModel<FrameType>::SendSignalToNonLocalPartitions(
//Parallel//     SimulationEngineInterface& simEngineInterfaceForTxNode,
//Parallel//     SimplePropagationModelForNode<FrameType>& transmittingNodeInfo,
//Parallel//     const double& transmitPowerDbm,
//Parallel//     const TimeType& duration,
//Parallel//     const shared_ptr<FrameType>& framePtr,
//Parallel//     const unsigned int channelNumber)
//Parallel// {
//Parallel//     const TimeType currentTime = simEngineInterfaceForTxNode.CurrentTime();
//Parallel//     const unsigned int localThreadIndex = simEngineInterfaceForTxNode.CurrentThreadPartitionIndex();
//Parallel//
//Parallel//     for(unsigned int partIndex = 0; (partIndex < propagationPartitions.size()); partIndex++) {
//Parallel//         if (partIndex != localThreadIndex) {
//Parallel//
//Parallel//             unique_ptr<SimulationEvent> signalEventPtr(
//Parallel//                 new InterPartitionSignalEvent(
//Parallel//                     this,
//Parallel//                     partIndex,
//Parallel//                     transmittingNodeInfo.GetNodeId(),
//Parallel//                     channelNumber,
//Parallel//                     transmittingNodeInfo.GetMobilityModel().GetPositionForTime(currentTime),
//Parallel//                     transmittingNodeInfo.GetAntennaModelPtr().get(),
//Parallel//                     transmitPowerDbm,
//Parallel//                     duration,
//Parallel//                     framePtr));
//Parallel//
//Parallel//             simEngineInterfaceForTxNode.ScheduleExternalEventAtPartition(
//Parallel//                 partIndex, signalEventPtr, currentTime);
//Parallel//
//Parallel//         }//if//
//Parallel//     }//for//
//Parallel//
//Parallel// }//SendSignalToNonLocalPartitions//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::TransmitSignal(
    SimulationEngineInterface& simEngineInterfaceForTxNode,
    SimplePropagationModelForNode<FrameType>& transmittingNodeInfo,
    const unsigned int channelNumber,
    const double& transmitPowerDbm,
    const TimeType& duration,
    const shared_ptr<FrameType>& framePtr)
{
    const TimeType currentTime = simEngineInterfaceForTxNode.CurrentTime();

    //Parallel SendSignalToNonLocalPartitions(
    //Parallel     simEngineInterfaceForTxNode,
    //Parallel     transmittingNodeInfo,
    //Parallel     transmitPowerDbm,
    //Parallel     duration,
    //Parallel     framePtr,
    //Parallel     channelNumber);

    TransmitSignalInLocalPartition(
        simEngineInterfaceForTxNode,
        transmittingNodeInfo.GetNodeId(),
        transmittingNodeInfo.GetInterfaceIndex(),
        channelNumber,
        transmittingNodeInfo.GetCurrentMobilityPosition(),
        transmittingNodeInfo.GetAntennaModel(),
        transmitPowerDbm,
        currentTime,
        duration,
        framePtr);

}//TransmitSignal//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::TransmitSignal(
    SimulationEngineInterface& simEngineInterfaceForTxNode,
    SimplePropagationModelForNode<FrameType>& transmittingNodeInfo,
    const vector<unsigned int>& channelNumbers,
    const double& transmitPowerDbm,
    const TimeType& duration,
    const shared_ptr<FrameType>& framePtr)
{
    const TimeType currentTime = simEngineInterfaceForTxNode.CurrentTime();

    //Parallel SendSignalToNonLocalPartitions(
    //Parallel     simEngineInterfaceForTxNode,
    //Parallel     transmittingNodeInfo,
    //Parallel     transmitPowerDbm,
    //Parallel     duration,
    //Parallel     framePtr,
    //Parallel     channelNumber);

    TransmitSignalInLocalPartition(
        simEngineInterfaceForTxNode,
        transmittingNodeInfo.GetNodeId(),
        transmittingNodeInfo.GetInterfaceIndex(),
        channelNumbers,
        transmittingNodeInfo.GetCurrentMobilityPosition(),
        transmittingNodeInfo.GetAntennaModel(),
        transmitPowerDbm,
        currentTime,
        duration,
        framePtr);

}//TransmitSignal//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::TakeOwnershipOfFrame(
    const unsigned int partitionIndex,
    const TimeType& currentTime,
    const TimeType& endTransmissionTimeOfFrame,
    shared_ptr<FrameType>& framePtr)
{
    typedef typename PropagationPartitionInfo::FrameReclaimationQueueRecord FrameQueueRecordType;

    const TimeType frameExpirationTime = (endTransmissionTimeOfFrame + maximumPropagationDelay);

    if (frameExpirationTime >= currentTime) {

        std::queue<FrameQueueRecordType>& reclaimationQueue =
            (*this).propagationPartitions.at(partitionIndex)->frameReclaimationQueue;

        reclaimationQueue.push(FrameQueueRecordType(frameExpirationTime, framePtr));

        // Reclaim expired frames:

        while ((!reclaimationQueue.empty() &&
                (reclaimationQueue.front().frameExpirationTime < currentTime))) {

            reclaimationQueue.pop();

        }//while//
    }//if//

    framePtr.reset();

}//TakeOwnershipOfFrame//



inline
void ReadSpectralMaskFromString(
    const string& spectralMaskString,
    const unsigned int channelNumber,
    SpectralMaskType& spectralMask)
{
    spectralMask.clear();

    assert(HasNoTrailingSpaces(spectralMaskString));
    istringstream spectralMaskStream(spectralMaskString);

    while(!spectralMaskStream.eof()) {
        SpectralMaskPointType aPoint;
        spectralMaskStream >> aPoint.distanceFromCenterFrequencyMhz;
        spectralMaskStream >> aPoint.relativePowerDb;

        if ((spectralMaskStream.fail()) ||
            (aPoint.distanceFromCenterFrequencyMhz <= 0.0) ||
            (aPoint.relativePowerDb > 0.0)) {

            cerr << "Error in Spectral Mask Parameter for channel = " <<  channelNumber << ":" << endl;
            cerr << "   " << spectralMaskString << endl;
            exit(1);
        }//if//

        spectralMask.push_back(aPoint);

    }//while//

}//ReadSpectralMaskFromString//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::CalculateChannelInterferenceFromSpectralMasks(
    const ParameterDatabaseReader& theParameterDatabaseReader)
{
    (*this).channelInterferenceModellingIsOn = true;
    (*this).channelInterferenceMatrix.resize(channels.size(), vector<double>(channels.size(), 0.0));

    vector<double> nominalTransmitBandwidthMhz(channels.size());
    vector<double> receiveWidthMhz(channels.size());
    vector<SpectralMaskType> spectralMasks(channels.size());

    for(unsigned int channelIndex = 0; channelIndex < channels.size(); channelIndex++) {

        const unsigned int channelNumber = baseChannelNumber + channelIndex;

        ostringstream parameterPrefixStream;
        parameterPrefixStream<< "channel-" << channelNumber << "-";
        const string parameterNamePrefix = parameterPrefixStream.str();

        const string spectralMaskString =
            theParameterDatabaseReader.ReadString(
                (parameterNamePrefix + "transmit-spectral-mask-mhz-dbr"),
                instanceId);
        ReadSpectralMaskFromString(spectralMaskString, channelNumber, spectralMasks[channelIndex]);

        nominalTransmitBandwidthMhz[channelIndex] =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "channel-interference-nominal-transmit-width-mhz"),
                instanceId);

        receiveWidthMhz[channelIndex] =
            theParameterDatabaseReader.ReadDouble(
                (parameterNamePrefix + "channel-interference-receive-width-mhz"),
                instanceId);

    }//for//


    for(unsigned int txChannelIndex = 0; txChannelIndex < channels.size(); txChannelIndex++) {
        const double& txChannelCenterFrequencyMhz = channels[txChannelIndex].carrierFrequencyMhz;
        for(unsigned int rxChannelIndex = 0; rxChannelIndex < channels.size(); rxChannelIndex++) {

            if (rxChannelIndex != txChannelIndex) {
                const double& rxChannelCenterFrequencyMhz = channels[rxChannelIndex].carrierFrequencyMhz;

                (*this).channelInterferenceMatrix[txChannelIndex][rxChannelIndex] =
                    CalcChannelInterferenceFactor(
                        txChannelCenterFrequencyMhz,
                        spectralMasks[txChannelIndex],
                        nominalTransmitBandwidthMhz[txChannelIndex],
                        (rxChannelCenterFrequencyMhz - (receiveWidthMhz[rxChannelIndex] / 2.0)),
                        (rxChannelCenterFrequencyMhz + (receiveWidthMhz[rxChannelIndex] / 2.0)));
            }//if//
        }//for//
    }//for//

}//CalculateChannelInterferenceFromSpectralMasks//



template<typename FrameType> inline
bool SimplePropagationModel<FrameType>::AllChannelsHaveSameChannelBandwidth() const
{
    const double channelBandwidthMhz = GetChannelBandwidthMhz(baseChannelNumber);
    for(unsigned int n = (baseChannelNumber + 1); (n < baseChannelNumber + channelCount); n++) {
        if (GetChannelBandwidthMhz(n) != channelBandwidthMhz) {
            return false;
        }//if//
    }//for//

    return true;

}//AllChannelsHaveSameChannelBandwidth//



template<typename FrameType> inline
void SimplePropagationModel<FrameType>::CheckThatNodeOrInterfaceIsNotAlreadyOnChannel(
     const vector<shared_ptr<SimplePropagationModelForNode<FrameType> > >& nodeList,
     const shared_ptr<SimplePropagationModelForNode<FrameType> >& newNodeModelPtr)
{
    for(unsigned int i = 0; (i < nodeList.size()); i++) {
        if ((!allowMultipleInterfacesOnSameChannel) &&
            (nodeList[i]->GetNodeId() == newNodeModelPtr->GetNodeId()))
        {
            cerr << "Error in propagation model: Multiple interfaces of node "
                 << newNodeModelPtr->GetNodeId() << " are on the same channel." << endl;
            cerr << "  Try setting \"propagation-allow-multiple-interfaces-on-same-channel\" to true." << endl;
            exit(1);
        }//if//

        if ((nodeList[i] == newNodeModelPtr) |
            ((nodeList[i]->GetNodeId() == newNodeModelPtr->GetNodeId()) &&
             (nodeList[i]->GetInterfaceIndex() == newNodeModelPtr->GetInterfaceIndex()))) {

            cerr << "Error in propagation model: Multiple copies of interface number "
                 << newNodeModelPtr->GetInterfaceIndex() << " on node " << newNodeModelPtr->GetNodeId()
                 << " have been inserted on the same channel." << endl;
            exit(1);
        }//if//
    }//for//

}//CheckThatNodeOrInterfaceIsNotAlreadyOnChannel//


//--------------------------------------------------------------------------------------------------



template<typename FrameType> inline
bool SimplePropagationModelForNode<FrameType>::IncomingSignal::ChannelIntersectionIsEmpty(
    const vector<unsigned int>& receivedChannels) const
{
    unsigned int i = 0;
    unsigned int j = 0;

    while((i < numberChannels) && (j < receivedChannels.size())) {
        if (channelNumbers[i] == receivedChannels[j]) {
            return false;
        }
        else if (channelNumbers[i] < receivedChannels[j]) {
            i++;
        }
        else {
            j++;
        }//if//
    }//while//

    return true;

}//ChannelIntersectionIsEmpty//



template<typename FrameType> inline
bool SimplePropagationModelForNode<FrameType>::IncomingSignal::IsOnChannel(
    const unsigned int channelNum) const
{
    for(unsigned int i = 0; (i < numberChannels); i++) {
        if (channelNumbers[i] == channelNum) {
            return true;
        }//if//
    }//for//

    return false;

}//ChannelIntersectionIsEmpty//


template<typename FrameType> inline
vector<double> GetListOfCarrierFreqenciesMhz(const SimplePropagationModel<FrameType>& propModel)
{
    vector<double> theList(propModel.GetChannelCount());
    for(unsigned int i = 0; (i < propModel.GetChannelCount()); i++) {
        const unsigned int channelNumber = (propModel.GetBaseChannelNumber() + i);
        theList[i] = propModel.GetCarrierFrequencyMhz(channelNumber);
    }//for//
    return (theList);
}


template<typename FrameType> inline
vector<double> GetListOfChannelBandwidthsMhz(const SimplePropagationModel<FrameType>& propModel)
{
    vector<double> theList(propModel.GetChannelCount());
    for(unsigned int i = 0; (i < propModel.GetChannelCount()); i++) {
        const unsigned int channelNumber = (propModel.GetBaseChannelNumber() + i);
        theList[i] = propModel.GetChannelBandwidthMhz(channelNumber);
    }//for//
    return (theList);
}


//====================================================================================


}//namespace//

#endif

