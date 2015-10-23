// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_ENGINE_H
#define SCENSIM_ENGINE_H

#include <assert.h>
#include <limits.h>
#include <fstream>
#include <memory>
#ifdef VC_120
#include <functional>
#endif

#include <vector>
#include <map>
#include <algorithm>

// Disable insanely slow debugging code in stl queue on windows.

#if (_HAS_ITERATOR_DEBUGGING == 1)
#undef _HAS_ITERATOR_DEBUGGING
#include <queue>
#define _HAS_ITERATOR_DEBUGGING 1
#else
#include <queue>
#endif

#include <memory>

#include "scensim_support.h"
#include "scensim_compiler_flag_consts.h"
#include "scensim_nodeid.h"
#include "scensim_time.h"
#include "scensim_trace.h"

namespace ScenSim {

using std::string;
using std::vector;
using std::map;
using std::shared_ptr;
using std::make_shared;
using std::enable_shared_from_this;
using std::auto_ptr;
using std::unique_ptr;
using std::move;



class SimulationEngine;
class ParameterDatabaseReader;
class SimulationEngineInterface;
class SimEngineThreadPartition;
class RuntimeStatisticsSystem;
class CounterStatistic;
class RealStatistic;


class TraceSubsystem {
public:
    TraceSubsystem() { }
    virtual ~TraceSubsystem() { }
    virtual void OutputTrace(
        const TimeType& currentTime,
        const NodeIdType& nodeId,
        const string& modelName,
        const string& modelInstanceId,
        const string& eventName,
        const string& stringToOutput,
        const unsigned int threadPartitionIndex) = 0;

    virtual void OutputTraceInBinary(
        const TimeType& currentTime,
        const NodeIdType& nodeId,
        const string& modelName,
        const string& modelInstanceId,
        const string& eventName,
        const unsigned char* dataPtr,
        const size_t dataSize,
        const unsigned int threadPartitionIndex) {}

    virtual void FlushTrace() {}

    virtual void OutputRecentlyAddedTraceTexts(
        std::ostream& guiStream) {}

    virtual bool BinaryOutputIsOn() const { return false; }
private:
    // Disabled
    TraceSubsystem(const TraceSubsystem&);
    void operator=(const TraceSubsystem&);
};//TraceSubsystem//


// Helper function for Trace Subsystem

void SetEnabledTraceTagsFromParameterDatabaseReader(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    vector<bool>& traceTagVector,
    TimeType& traceStartTime);

//--------------------------------------------------------------------------------------------------
// Stable version of priority_queue (to force same time events FIFO order) for regression testing
// simulations with many same time events.
// Now we use this as default. (2012.2.11)
//--------------------------------------------------------------------------------------------------

template<typename T, bool ascendingOrder>
struct PriorityQueueStableCompare {
    typedef std::less<T> CompareType;
};

template<typename T>
struct PriorityQueueStableCompare<T, false> {
    typedef std::greater<T> CompareType;
};

template<typename T, bool ascendingOrder = true>
class priority_queue_stable {
public:
    priority_queue_stable() : currentSequenceNumber(0) { }

    bool empty() const { return (theQueue.empty()); }

    const T& top() const { return (theQueue.top().element); }

    void push(const T& item) {
        currentSequenceNumber++;
        theQueue.push(ElementType(item, currentSequenceNumber));
    }

    void push(T&& item) {
        currentSequenceNumber++;
        theQueue.push(move(ElementType(item, currentSequenceNumber)));
    }

    void pop() { theQueue.pop(); }

    void pop_move(T& item) {
        // std::priority_queue is broken with respect to movable types.
        item = move(const_cast<ElementType&>(theQueue.top()).element);
        theQueue.pop();
    }

    size_t size() const { return (theQueue.size()); }
    void reserve(const size_t reserveSize) { theQueue.reserve(reserveSize); }

private:
    struct ElementType {
        T element;
        unsigned long long int sequenceNumber;

        ElementType() : sequenceNumber(0) { }

        ElementType(
            const T& initElement,
            const unsigned long long int& initSequenceNumber)
            : element(initElement),  sequenceNumber(initSequenceNumber) {}

        ElementType(
            T& initElement,
            const unsigned long long int& initSequenceNumber)
            : element(move(initElement)),  sequenceNumber(initSequenceNumber) {}

        void operator=(ElementType&& right) {
            element = move(right.element);
            sequenceNumber = right.sequenceNumber;
        }

        ElementType(ElementType&& right) { (*this) = move(right); }

        bool operator<(const ElementType& right) const
        {
            return (
                ((*this).element < right.element) ||
                ((!(right.element < (*this).element)) &&
                 ((*this).sequenceNumber > right.sequenceNumber)));
        }

        bool operator>(const ElementType& right) const
        {
            return (
                ((*this).element > right.element) ||
                ((!(right.element > (*this).element)) &&
                 ((*this).sequenceNumber < right.sequenceNumber)));
        }

    };//ElementType//

    std::priority_queue<ElementType, vector<ElementType>, typename PriorityQueueStableCompare<ElementType, ascendingOrder>::CompareType> theQueue;
    unsigned long long int currentSequenceNumber;

};//priority_queue_stable//

//--------------------------------------------------------------------------------------------------

class SimulationEvent {
public:
    virtual ~SimulationEvent() { }
    virtual void ExecuteEvent() = 0;
};

//--------------------------------------------------------------------------------------------------
//
// Global Simulation Engine datastructure.
//

class SimulationEngine {
public:
    static const unsigned int INVALID_INDEX = UINT_MAX;

    SimulationEngine(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const bool initIsRunningSimulationSequentially = true,
        const unsigned int numberPartitionThreads = 1);

    ~SimulationEngine();

    shared_ptr<SimulationEngineInterface> GetSimulationEngineInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const NodeIdType& nodeId,
        const unsigned int startingPartitionIndex = 0);

    void RunSimulationSequentially(const TimeType& runUntilTime, TimeType& nextEventTime);

    void RunSimulationSequentially(const TimeType& runUntilTime)
    {
        TimeType notUsed;
        (*this).RunSimulationSequentially(runUntilTime, notUsed);
    }

    void RunSimulationInParallel(const TimeType& stopTime);

    // In non-parallel mode, the simulation will stop after completing the currently executing event.

    void PauseSimulation();
    void ClearAnyImpendingPauseSimulation();

    unsigned int GetNumberPartitionThreads() const;

    // Note: Accessing CurrentTime while the simulation is running (in another thread) or
    //       running in parallel is problematic. Maybe okay for a approximate "dashboard"
    //       display of the current running simulation time.

    TimeType CurrentTime() const;
    bool SimulationIsPaused() const;
    bool SimulationIsDone() const;
    void ShutdownSimulator();

    void EnableTraceAtANode(const NodeIdType& nodeId, const TraceTagType traceTag);
    void DisableTraceAtANode(const NodeIdType& nodeId, const TraceTagType traceTag);

    RuntimeStatisticsSystem& GetRuntimeStatisticsSystem();

    TraceSubsystem& GetTraceSubsystem();

    void SetTraceSubsystem(const shared_ptr<TraceSubsystem>& newTraceSubsystemPtr);

    // For external simulation connector.  Simulation must be paused and cannot advance
    // the time past the next event.

    void TriviallyAdvanceSimulationTime(const TimeType& newSimulationTime);

private:
    class Implementation;
    auto_ptr<Implementation> implPtr;

    friend class SimEngineThreadPartition;
    friend class SimulationEngineInterface;

    //Disabling:
    SimulationEngine(const SimulationEngine&);
    void operator=(const SimulationEngine&);

};//SimulationEngine//


//--------------------------------------------------------------------------------------------------


class EventRescheduleTicket {
public:
    EventRescheduleTicket() : eventNumber(0), eventInfoIndex(SimulationEngine::INVALID_INDEX) {}

    void Clear() { eventInfoIndex = SimulationEngine::INVALID_INDEX; eventNumber = 0; }

    bool IsNull() const { return (eventInfoIndex == SimulationEngine::INVALID_INDEX); }

private:
    unsigned int eventNumber;
    unsigned int eventInfoIndex;

    friend class SimEngineThreadPartition;

};//EventRescheduleTicket//


//=============================================================================


class SimulationEngineInterface: public enable_shared_from_this<SimulationEngineInterface> {
private:
    SimulationEngineInterface(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        SimulationEngine* initSimulationEnginePtr,
        const shared_ptr<SimEngineThreadPartition>& initSimThreadPartitionPtr,
        const NodeIdType& initNodeId);

public:

    void ShutdownThisInterface();

    TimeType CurrentTime() const;

    NodeIdType GetNodeId() const { return nodeId; }

    // Schedule cancelable event, share event object.

    void ScheduleEvent(
        const shared_ptr<SimulationEvent>& eventPtr,
        const TimeType& eventTime,
        EventRescheduleTicket& eventTicket);

    // Schedule non-cancelable event, share event object.

    void ScheduleEvent(
        const shared_ptr<SimulationEvent>& eventPtr,
        const TimeType& eventTime);

    // Schedule non-cancelable event, give ownership of object to simulator.
    // This version will null the event pointer.
    // (unless the type is the base event class (SimulationEvent) in which the
    // the following version will be used).

    void ScheduleEvent(
        unique_ptr<SimulationEvent>& eventPtr,
        const TimeType& eventTime);

    template<typename SimEventType>
    void ScheduleEvent(
        unique_ptr<SimEventType>& eventPtr,
        const TimeType& eventTime);

    template<typename SimEventType>
    void ScheduleEvent(
        unique_ptr<SimEventType>&& eventPtr,
        const TimeType& eventTime);

    void CancelEvent(EventRescheduleTicket& eventTicket);

    void RescheduleEvent(EventRescheduleTicket& eventTicket, const TimeType& eventTime);

    // Inter-node communication methods.

    void ScheduleExternalEventAtNode(
        const NodeIdType& destinationNodeId,
        unique_ptr<SimulationEvent>& eventPtr,
        const TimeType& eventTime);

    template<typename SimEventType>
    void ScheduleExternalEventAtNode(
        const NodeIdType& destinationNodeId,
        unique_ptr<SimEventType>& eventPtr,
        const TimeType& eventTime);

    template<typename SimEventType>
    void ScheduleExternalEventAtNode(
        const NodeIdType& destinationNodeId,
        unique_ptr<SimEventType>&& eventPtr,
        const TimeType& eventTime);

    void ScheduleExternalEventAtPartition(
        const unsigned int destinationPartitionIndex,
        unique_ptr<SimulationEvent>& eventPtr,
        const TimeType& eventTime);

    template<typename SimEventType>
    void ScheduleExternalEventAtPartition(
        const unsigned int destinationPartitionIndex,
        unique_ptr<SimEventType>& eventPtr,
        const TimeType& eventTime);

    // Use this method (along with the Node ID) to generate unique IDs (used, for example,
    // for Packet Ids).

    unsigned long long int GenerateAndReturnNewLocalSequenceNumber()
    {
        nodeLevelSequenceNumber++;
        return nodeLevelSequenceNumber;
    }

    // Trace routines:

    // When DISABLE_TRACE_FLAG is defined all tracing related object code should be removed by
    // the compiler.

    bool TraceIsOn(const TraceTagType traceTag) const;


    void OutputTrace(
        const string& modelName,
        const string& modelInstanceId,
        const string& eventName,
        const string& stringToOutput) const;

    bool BinaryOutputIsOn() const;

    template<typename T>
    void OutputTraceInBinary(
        const string& modelName,
        const string& modelInstanceId,
        const string& eventName,
        const T& data) const;

    void OutputTraceInBinary(
        const string& modelName,
        const string& modelInstanceId,
        const string& eventName) const;

    bool ParallelismIsOn() const { return PARALLELISM_IS_ENABLED; }

    shared_ptr<CounterStatistic> CreateCounterStat(
        const string& statName,
        const bool useBigCounter = false);

    shared_ptr<RealStatistic> CreateRealStat(
        const string& statName,
        const bool useBigReal = false);

    shared_ptr<RealStatistic> CreateRealStatWithDbConversion(
        const string& statName,
        const bool useBigReal = false);


    //-------------------------------------------------------------------------
    //
    // Parallel Execution methods:
    //

    unsigned int CurrentThreadPartitionIndex() const;

    //Future: void MoveNodeIntoParallelPartition(unsigned int partitionIndex)
    //Future:   { simulationEnginePartitionPtr->MoveNodeIntoParallelPartition(partitionIndex); }

    void MoveNodeToSameParallelPartitionAs(const NodeIdType& nodeId);

    // "Earliest Output Time" (EOT) is a guarantee by the model programmer that
    // the local "node" will not communicate (send an event) to another
    // node before the EOT.
    //
    // "Lookahead Time" is a guarantee by the model programmer that the local
    // "node" will not communicate (send an event) to another node without a delay
    // of at least the "Lookahead Time" (a delay from the time of a future triggering event).
    // EOT is a fixed absolute time in the future, Lookahead is relative to the current time.

    // EOT and Lookahead have been extended to allow for a node's
    // subsystems to have their own EOT variable, with the node level EOT being
    // the MINIMUM of the subsystem EOTs and Lookaheads.

    unsigned int AllocateEarliestOutputTimeIndex();
    unsigned int AllocateLookaheadTimeIndex();

    void SetAnEarliestOutputTimeForThisNode(
        const TimeType& earliestOutputTime,
        const unsigned int eotIndex);

    void SetALookaheadTimeForThisNode(
        const TimeType& aLookaheadTime,
        const unsigned int lookaheadIndex);

    TimeType GetEarliestOutputTime() const;

    TimeType GetMinLookaheadTime() const;

private:
    friend class SimulationEngine;

    void EnableTraceTag(const TraceTagType tagIndex) { traceIsEnabledForTagIndex.at(tagIndex) = true; }
    void DisableTraceTag(const TraceTagType tagIndex) { traceIsEnabledForTagIndex.at(tagIndex) = false; }

    friend class SimEngineThreadPartition;

    SimEngineThreadPartition* simulationEnginePartitionPtr;

    // Evil pointer into the simulation engine's variable for current time.
    // Makes "CurrentTime()" function just a (inlined) direct variable access.
    // and provides access to the current time in the debugger.

    const TimeType * currentTimeVariablePtr;

    void SetCurrentTimeVariablePtr(const TimeType * aCurrentTimeVariablePtr)
        { currentTimeVariablePtr = aCurrentTimeVariablePtr; }

    vector<bool> traceIsEnabledForTagIndex;
    TimeType traceStartTime;

    TimeType GetCurrentTimeInTheNormalManner() const;

    NodeIdType nodeId;

    unsigned long long int nodeLevelSequenceNumber;

    vector<TimeType> earliestOutputTimes;
    vector<TimeType> lookaheadTimes;

    void OutputTraceInBinaryInternal(
        const string& modelName,
        const string& modelInstanceId,
        const string& eventName,
        const unsigned char* dataPtr,
        const size_t dataSize) const;

    // Disabled:

    SimulationEngineInterface(const SimulationEngineInterface&);
    void operator=(const SimulationEngineInterface&);

};//SimulationEngineInterface//


//--------------------------------------------------------------------------------------------------



inline
void SimulationEngineInterface::ScheduleEvent(
    const shared_ptr<SimulationEvent>& eventPtr,
    const TimeType& eventTime)
{
    EventRescheduleTicket unusedEventTicket;
    (*this).ScheduleEvent(eventPtr, eventTime, unusedEventTicket);
}


inline
TimeType SimulationEngineInterface::CurrentTime() const
{
    assert(GetCurrentTimeInTheNormalManner() == *currentTimeVariablePtr);

    return *currentTimeVariablePtr;
}

template<typename SimEventType> inline
void SimulationEngineInterface::ScheduleEvent(
    unique_ptr<SimEventType>& eventPtr,
    const TimeType& eventTime)
{
    unique_ptr<SimulationEvent> baseClassEventPtr = move(eventPtr);
    (*this).ScheduleEvent(baseClassEventPtr,  eventTime);
}

template<typename SimEventType> inline
void SimulationEngineInterface::ScheduleEvent(
    unique_ptr<SimEventType>&& eventPtr,
    const TimeType& eventTime)
{
    unique_ptr<SimulationEvent> baseClassEventPtr = move(eventPtr);
    (*this).ScheduleEvent(baseClassEventPtr,  eventTime);
}


template<typename SimEventType> inline
void SimulationEngineInterface::ScheduleExternalEventAtNode(
    const NodeIdType& destinationNodeId,
    unique_ptr<SimEventType>& eventPtr,
    const TimeType& eventTime)
{
    unique_ptr<SimulationEvent> baseClassEventPtr = move(eventPtr);

    (*this).ScheduleExternalEventAtNode(destinationNodeId, baseClassEventPtr, eventTime);
}

template<typename SimEventType> inline
void SimulationEngineInterface::ScheduleExternalEventAtNode(
    const NodeIdType& destinationNodeId,
    unique_ptr<SimEventType>&& eventPtr,
    const TimeType& eventTime)
{
    unique_ptr<SimulationEvent> baseClassEventPtr = move(eventPtr);

    (*this).ScheduleExternalEventAtNode(destinationNodeId, baseClassEventPtr, eventTime);
}

template<typename SimEventType> inline
void SimulationEngineInterface::ScheduleExternalEventAtPartition(
    const unsigned int destinationPartitionIndex,
    unique_ptr<SimEventType>& eventPtr,
    const TimeType& eventTime)
{
    unique_ptr<SimulationEvent> eventPtrCopy = eventPtr;

    (*this).ScheduleExternalEventAtPartition(
        destinationPartitionIndex, eventPtrCopy, eventTime);
}


inline
bool SimulationEngineInterface::TraceIsOn(const TraceTagType traceTag) const
{
    if (TRACE_IS_ENABLED) {
        assert(traceTag < traceIsEnabledForTagIndex.size());
        if ((traceIsEnabledForTagIndex[traceTag]) &&
            (CurrentTime() >= traceStartTime)) {

            return true;
        }//if//
    }//if//
    return false;
}



inline
void SimulationEngineInterface::SetAnEarliestOutputTimeForThisNode(
    const TimeType& earliestOutputTime,
    const unsigned int eotIndex)
{
    if (PARALLELISM_IS_ENABLED) {
        (*this).earliestOutputTimes.at(eotIndex) = earliestOutputTime;
    }//if//
}


inline
void SimulationEngineInterface::SetALookaheadTimeForThisNode(
    const TimeType& aLookaheadTime,
    const unsigned int lookaheadIndex)
{
    if (PARALLELISM_IS_ENABLED) {
        (*this).lookaheadTimes.at(lookaheadIndex) = aLookaheadTime;
    }//if//
}



inline
TimeType SimulationEngineInterface::GetEarliestOutputTime() const
{
    using std::min;

    TimeType eot = INFINITE_TIME;
    for(unsigned int i = 0; (i < earliestOutputTimes.size()); i++) {
        eot = min(eot, earliestOutputTimes[i]);
    }//for//
    return eot;
}


inline
TimeType SimulationEngineInterface::GetMinLookaheadTime() const
{
    using std::min;
    TimeType minLookahead = INFINITE_TIME;

    for(unsigned int i = 0; (i < lookaheadTimes.size()); i++) {
        minLookahead = min(minLookahead, lookaheadTimes[i]);
    }//for//

    return minLookahead;
}

template<typename T> inline
void SimulationEngineInterface::OutputTraceInBinary(
        const string& modelName,
        const string& modelInstanceId,
        const string& eventName,
        const T& data) const
{

    (*this).OutputTraceInBinaryInternal(modelName,
                                modelInstanceId,
                                eventName,
                                reinterpret_cast<const unsigned char*>(&data),
                                sizeof(data));

}

inline
void SimulationEngineInterface::OutputTraceInBinary(
        const string& modelName,
        const string& modelInstanceId,
        const string& eventName) const
{

    (*this).OutputTraceInBinaryInternal(modelName,
                                modelInstanceId,
                                eventName,
                                (unsigned char*)(nullptr),
                                0);

}


}//namespace//


#endif


