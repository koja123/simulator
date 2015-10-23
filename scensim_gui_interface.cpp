// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

// Target Windows 2000 and beyond.
#define _WIN32_WINNT 0x500

#include <sstream>
#include <queue>

#include "scensim_time.h"
#include "scensim_gui_interface.h"
#include "scensim_gui_interface_constants.h"
#include "scensim_support.h"
#include "scensim_engine.h"
#include "scensim_stats.h"

// Boost bugs cause "windows.h" to be included.

#define _WIN32_WINNT 0x500 // Stop warning: Target w2k+
#include <boost/asio.hpp>
#include <boost/thread.hpp>

namespace ScenSim {

using std::cerr;
using std::endl;
using std::string;
using std::istringstream;

const static string confirmCommandString("Okay.");

class GuiInterfacingSubsystem::Implementation {
public:
    ~Implementation();
private:
    friend class GuiInterfacingSubsystem;

    Implementation(
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const unsigned short int guiPortNumber,
        const unsigned short int guiPauseCommandPortNumber);

    void GetAndExecuteGuiCommands(
        ParameterDatabaseReader& theParameterDatabaseReader,
        NetworkSimulator& theNetworkSimulator,
        TimeType& simulateUpToTime);

    void SendStreamedOutput(const NetworkSimulator& theNetworkSimulator);

    void PauseSimulationCommandFielderThreadRoutine();

    shared_ptr<SimulationEngine> simulationEnginePtr;

    //-------------------------------------------
    class PauseCommandFielderThreadFunctor {
    public:
        PauseCommandFielderThreadFunctor(Implementation* initGuiInterfacePtr)
            : guiInterfacePtr(initGuiInterfacePtr) {}
        void operator()() { guiInterfacePtr->PauseSimulationCommandFielderThreadRoutine(); }

    private:
        Implementation* guiInterfacePtr;
    };//PauseCommandFielderThreadFunctor//

    //-------------------------------------------

    bool isInEmulationMode;
    bool isOnlineTraceEnabled;

    friend class PauseCommandFielderThreadFunctor;

    auto_ptr<boost::thread> pauseSimulationCommandFielderThreadPtr;
    volatile bool fielderThreadIsToTerminate;

    volatile bool simIsRunning;

    // Make connections (initialize) in a predetermined order with the GUI.
    boost::asio::ip::tcp::iostream guiStream;
    boost::asio::ip::tcp::iostream guiPauseCommandStream;

    StatViewCollection guiStatViewCollection;

    TimeType nodePositionUpdateInterval;
    TimeType lastNodePositionOutputTime;

    TimeType emulationTimeOutputInterval;
    TimeType lastEmulationTimeOutputTime;

    //-------------------------------------------

    class StatStreamer {
    public:
        StatStreamer(
            const TimeType& initSendInterval,
            bool initLastValueMode)
            : sendInterval(initSendInterval), nextSendTime(initSendInterval), lastValueMode(initLastValueMode) {}

        TimeType GetNextSendTime() const { return nextSendTime; }
        void SetNextSendTime(const TimeType& newNextSendTime) { nextSendTime = newNextSendTime; }

        virtual void OutputStatsToStream(const TimeType& currentTime, ostream& outputStream) = 0;

    protected:
        TimeType sendInterval;
        TimeType nextSendTime;
        bool lastValueMode;
    };///StreamedStat//


    class CounterStatStreamer: public StatStreamer {
    public:
        CounterStatStreamer(
            const shared_ptr<CounterStatView>& initStatViewPtr,
            const TimeType& initSendInterval,
            const bool initLastValueMode)
        :
            statViewPtr(initStatViewPtr), StatStreamer(initSendInterval, initLastValueMode)
        {}
        void OutputStatsToStream(const TimeType& currentTime, ostream& outputStream);
    private:
        shared_ptr<CounterStatView> statViewPtr;
    };//StreamedCounterStat//


    class RealStatStreamer: public StatStreamer {
    public:
        RealStatStreamer(
            const shared_ptr<RealStatView>& initStatViewPtr,
            const TimeType& initSendInterval,
            const bool initLastValueMode)
        :
            statViewPtr(initStatViewPtr), StatStreamer(initSendInterval, initLastValueMode)
        {}
        void OutputStatsToStream(const TimeType& currentTime, ostream& outputStream);
    private:
        shared_ptr<RealStatView> statViewPtr;
    };//StreamedRealStat//

    class StreamerQueueComparator {
    public:
        bool operator()(const shared_ptr<StatStreamer>& left, const shared_ptr<StatStreamer>& right) const
            { return (left->GetNextSendTime() > right->GetNextSendTime()); }
    };


    typedef
        std::priority_queue<
            shared_ptr<StatStreamer>,
            vector<shared_ptr<StatStreamer> >,
            StreamerQueueComparator>
        StatStreamerQueueType;

    StatStreamerQueueType statStreamerQueue;


    //-------------------------------------------

    void ShutdownPauseCommandFielderThread();

    void SetupStreamingOfAStatsLastValues(
        const NodeIdType nodeId,
        const string& statNameString,
        const TimeType& statSendInterval,
        const bool lastValueMode);

    void SetupStreamingOfPositionUpdates(const TimeType& newPositionUpdateInterval)
        {  (*this).nodePositionUpdateInterval = newPositionUpdateInterval; }

    void SetupStreamingOfEmulationTime(const TimeType& newEmulationTimeOutputInterval)
        {  (*this).emulationTimeOutputInterval = newEmulationTimeOutputInterval; }

    // Disabled:
    Implementation(const Implementation&);
    void operator=(const Implementation&);

};//GuiInterfacingSubsystem::Implementation//


void GuiInterfacingSubsystem::Implementation::CounterStatStreamer::OutputStatsToStream(
    const TimeType& currentTime,
    ostream& outputStream)
{
    (*this).nextSendTime = currentTime + sendInterval;

    outputStream << "streamlast " << ConvertTimeToStringSecs(currentTime) << ' ';
    outputStream << statViewPtr->GetNodeId() << ' ' << statViewPtr->GetStatName() << ' ';

    if (lastValueMode) {
        outputStream << statViewPtr->CurrentCount() << endl;
    } else {
        outputStream << statViewPtr->GetCountOfLastBin(currentTime) << endl;
    }

}//OutputStatsToStream//


void GuiInterfacingSubsystem::Implementation::RealStatStreamer::OutputStatsToStream(
    const TimeType& currentTime,
    ostream& outputStream)
{
    (*this).nextSendTime = currentTime + sendInterval;

    outputStream << "streamlast " << ConvertTimeToStringSecs(currentTime) << ' ';
    outputStream << statViewPtr->GetNodeId() << ' ' << statViewPtr->GetStatName() << ' ';

    if (lastValueMode) {
        outputStream << statViewPtr->GetLastRawValue() << endl;

    } else {
        double value;
        const bool isValid =
            statViewPtr->GetAverageOfLastBin(currentTime, value);
        if (!isValid) {
            outputStream << ' ' << STAT_NO_DATA_SPECIAL_FIELD << endl;
        } else {
            outputStream << ' ' << value << endl;
        }
    }

}//OutputStatsToStream//


GuiInterfacingSubsystem::Implementation::Implementation(
    const shared_ptr<SimulationEngine>& initSimulationEnginePtr,
    const unsigned short int guiPortNumber,
    const unsigned short int guiPauseCommandPortNumber)
    :
    simulationEnginePtr(initSimulationEnginePtr),
    guiStream("127.0.0.1", ConvertToString(guiPortNumber)),
    guiPauseCommandStream("127.0.0.1", ConvertToString(guiPauseCommandPortNumber)),
    simIsRunning(false),
    fielderThreadIsToTerminate(false),
    nodePositionUpdateInterval(INFINITE_TIME),
    lastNodePositionOutputTime(ZERO_TIME),
    emulationTimeOutputInterval(INFINITE_TIME),
    lastEmulationTimeOutputTime(ZERO_TIME),
    isInEmulationMode(false),
    isOnlineTraceEnabled(false)
{
    if (!guiStream.good()) {
        cerr << "Could not connect to GUI port: " << guiPortNumber << endl;
        exit(1);
    }//if//

    if (!guiPauseCommandStream.good()) {
        cerr << "Could not connect to GUI port: " << guiPauseCommandPortNumber << endl;
        exit(1);
    }//if//

    guiStream.precision(STATISTICS_OUTPUT_PRECISION);

    pauseSimulationCommandFielderThreadPtr =
        auto_ptr<boost::thread>(new boost::thread(PauseCommandFielderThreadFunctor(this)));

}//GuiInterfacingSubsystem::Implementation()//


GuiInterfacingSubsystem::Implementation::~Implementation() {
    (*this).ShutdownPauseCommandFielderThread();
}

void GuiInterfacingSubsystem::Implementation::ShutdownPauseCommandFielderThread() {
    (*this).fielderThreadIsToTerminate = true;
    (*this).guiPauseCommandStream.close();
    pauseSimulationCommandFielderThreadPtr->join();
}


void GuiInterfacingSubsystem::Implementation::PauseSimulationCommandFielderThreadRoutine()
{
    while (!fielderThreadIsToTerminate) {

        string command;
        getline(guiPauseCommandStream, command);

        if (!guiPauseCommandStream.good()) { break; }

        ConvertStringToLowerCase(command);

        if (command == "pause_sim") {
            assert(!isInEmulationMode);
            if (simIsRunning) {
                simulationEnginePtr->PauseSimulation();
            }//if//
            guiPauseCommandStream << confirmCommandString << endl << std::flush;
        }
        else if (command == "terminate_sim") {
            assert(isInEmulationMode);

            simulationEnginePtr->ShutdownSimulator();

            guiPauseCommandStream << confirmCommandString << endl << std::flush;
        }
        else if (command == "howareyou") {
            guiPauseCommandStream << confirmCommandString << endl << std::flush;
        }
        else {
            cerr << "Error: GUI sent a command other than pause_sim" << endl;
            exit(1);
        }//if//
    }//while//

}//PauseSimulationCommandFielderThreadRoutine//

static inline
void OutputPropagationStatToStream(
    const PropagationInformationType& informationType,
    const PropagationStatisticsType& propagationStat,
    ostream& outStream)
{
    if (informationType & PROPAGATION_INFORMATION_PATHLOSS) {
        outStream << ' ' << propagationStat.totalLossValueDb;
    }

    if (informationType & PROPAGATION_INFORMATION_PROPAGATIONPATH) {
        outStream << ' ' << propagationStat.totalLossValueDb;

        const vector<PropagationPathType>& paths = propagationStat.paths;

        outStream << ' ' << paths.size();

        for(size_t i = 0; i < paths.size(); i++) {
            const PropagationPathType& propPath = paths[i];
            const vector<PropagationVertexType>& pathVertices = propPath.pathVertices;

            outStream << ' ' << propPath.lossValueDb << ' ' << pathVertices.size();

            for(size_t j= 0; j < pathVertices.size(); j++) {
                outStream << ' ' <<  pathVertices[j].vertexType;
            }

            for(size_t j= 0; j < pathVertices.size(); j++) {
                const PropagationVertexType& pathVertex = pathVertices[j];

                outStream
                    << ' ' << pathVertex.x
                    << ' ' << pathVertex.y
                    << ' ' << pathVertex.z;
            }
        }
    }
}

static inline
void ReadNodeIdFromStream(
    const string& aLine,
    istringstream& guiCommandLineStream,
    NodeIdType& nodeId)
{
    string nodeIdString;
    guiCommandLineStream >> nodeIdString;

    if (guiCommandLineStream.fail()) {
       cerr << "Error in GUI command, bad nodeId: " << aLine << endl;
       exit(1);
    }//if//

    if (nodeIdString == "*") {
        nodeId = ANY_NODEID;
    }
    else {
        int intValue;
        bool succeeded;
        ConvertStringToInt(nodeIdString, intValue, succeeded);
        if (!succeeded) {
            cerr << "Error in GUI command, bad nodeId: " << aLine << endl;
            exit(1);
        }//if//

        nodeId = intValue;
    }//if//

}//ReadNodeIdFromStream//


void GuiInterfacingSubsystem::Implementation::GetAndExecuteGuiCommands(
    ParameterDatabaseReader& theParameterDatabaseReader,
    NetworkSimulator& theNetworkSimulator,
    TimeType& simulateUpToTime)
{
    if (simIsRunning) {
        if (isOnlineTraceEnabled) {
            TraceSubsystem& traceSubsystem =
                simulationEnginePtr->GetTraceSubsystem();
            traceSubsystem.FlushTrace();
        }
        simIsRunning = false;
        guiStream << "resume_sim_completed" << endl << std::flush;
    }//if//

    while(true) {
        string aLine;
        getline(guiStream, aLine);

        istringstream guiCommandLineStream(aLine);

        string command;

        guiCommandLineStream >> command;

        ConvertStringToLowerCase(command);

        if (command == "send_simtime") {
            guiStream << ConvertTimeToStringSecs(simulationEnginePtr->CurrentTime()) << endl;
        }
        else if (command == "resume_sim") {
            string timeString;
            guiCommandLineStream >> timeString;
            bool succeeded;
            ConvertStringToTime(timeString, simulateUpToTime, succeeded);
            if (!succeeded) {
                cerr << "Error in GUI command: Bad time format in Sim control \"resume\" command: "
                     << timeString << endl;
                exit(1);
            }//if//

            simulationEnginePtr->ClearAnyImpendingPauseSimulation();
            simIsRunning = true;

            guiStream << confirmCommandString << endl << std::flush;

            break;
        }
        else if (command == "terminate_sim") {
            simulationEnginePtr->ShutdownSimulator();
            guiStream << confirmCommandString << endl;
            return;
        }
        else if (command == "set_parameter") {
            string restOfLine;
            getline(guiCommandLineStream, restOfLine);

            bool foundAnError;
            theParameterDatabaseReader.AddNewDefinitionToDatabase(restOfLine, foundAnError);

            if (foundAnError) {
                cerr << "Error in GUI set_paramater command: " << aLine << endl;
                exit(1);
            }//if//

            guiStream << confirmCommandString << endl;
        }
        else if ((command == "enable_trace") || (command == "disable_trace")) {

            NodeIdType nodeId;
            ReadNodeIdFromStream(aLine, guiCommandLineStream, nodeId);

            string traceTagString;
            guiCommandLineStream >> traceTagString;

            bool succeeded;
            TraceTagType traceTag;
            LookupTraceTagIndex(traceTagString, traceTag, succeeded);
            if (!succeeded) {
                cerr << "Error in GUI command, bad trace \"tag\" (ID) string: " << aLine << endl;
                exit(1);
            }//if//

            if (command == "enable_trace") {
                simulationEnginePtr->EnableTraceAtANode(nodeId, traceTag);
            }
            else if (command == "disable_trace") {
                simulationEnginePtr->DisableTraceAtANode(nodeId, traceTag);
            }
            else {
                assert(false); abort();
            }//if//
            guiStream << confirmCommandString << endl;
        }
        else if (command == "enable_stat") {
            NodeIdType nodeId;
            ReadNodeIdFromStream(aLine, guiCommandLineStream, nodeId);

            string statNameString;
            guiCommandLineStream >> statNameString;

            string aggregationIntervalString;

            guiCommandLineStream >> aggregationIntervalString;

            if (guiCommandLineStream.fail()) {
                cerr << "Error in Gui stat command: " << aLine << endl;
                exit(1);
            }//if//

            TimeType aggregationInterval;
            bool succeeded;
            ConvertStringToTime(aggregationIntervalString, aggregationInterval, succeeded);

            if (!succeeded) {
                cerr << "Error in Gui command at stat aggregation interval: " << aLine << endl;
                exit(1);
            }//if//

            TimeType startTime = ZERO_TIME;
            TimeType endTime = INFINITE_TIME;

            string startTimeString;

            guiCommandLineStream >> startTimeString;

            bool sendLastValueMode = false;

            if (!guiCommandLineStream.fail()) {
                bool succeeded;
                ConvertStringToTime(startTimeString, startTime, succeeded);
                if (!succeeded) {
                    cerr << "Error in Gui command at stat start time : " << aLine << endl;
                    exit(1);
                }//if//

                string endTimeString;

                guiCommandLineStream >> endTimeString;

                if (!guiCommandLineStream.fail()) {
                    ConvertStringToTime(endTimeString, endTime, succeeded);
                    if (!succeeded) {
                        cerr << "Error in Gui command at stat end time : " << aLine << endl;
                        exit(1);
                    }//if//
                }//if//

                int statViewMode;
                
                guiCommandLineStream >> statViewMode;

                if (!guiCommandLineStream.fail()) {
                    if (statViewMode > 0) {
                        sendLastValueMode = true;
                    }
                }
                

            }//if//

            CreateStatViews(
                simulationEnginePtr->GetRuntimeStatisticsSystem(),
                nodeId,
                statNameString,
                startTime,
                endTime,
                aggregationInterval,
                sendLastValueMode,
                this->guiStatViewCollection);
            guiStream << confirmCommandString << endl;
        }
        else if (command == "send_last_stat_value") {
            NodeIdType nodeId;
            ReadNodeIdFromStream(aLine, guiCommandLineStream, nodeId);
            string statNameString;
            guiCommandLineStream >> statNameString;

            OutputLastStatValueToStream(
                guiStatViewCollection,
                nodeId,
                statNameString,
                simulationEnginePtr->CurrentTime(),
                guiStream);
            guiStream << endl;
        }
        else if (command == "stream_last_stat_values") {

            NodeIdType nodeId;
            ReadNodeIdFromStream(aLine, guiCommandLineStream, nodeId);
            string statNameString;
            guiCommandLineStream >> statNameString;

            string statSendIntervalString;
            guiCommandLineStream >> statSendIntervalString;

            if (!guiCommandLineStream.fail()) {
                TimeType statSendInterval;
                bool succeeded;
                ConvertStringToTime(statSendIntervalString, statSendInterval, succeeded);
                if (!succeeded) {
                    cerr << "Error in Gui command at stream interval time : " << aLine << endl;
                    exit(1);
                }//if//

                const bool lastValueMode = true;
                (*this).SetupStreamingOfAStatsLastValues(nodeId, statNameString, statSendInterval, lastValueMode);

                guiStream << confirmCommandString << endl;
            }//if//
        }
        else if (command == "send_stat_names") {
            NodeIdType nodeId;
            ReadNodeIdFromStream(aLine, guiCommandLineStream, nodeId);

            string statNameString;
            guiCommandLineStream >> statNameString;

            if ((guiCommandLineStream.fail()) || (!guiCommandLineStream.eof())) {
                cerr << "Error in Gui stat command: " << aLine << endl;
                exit(1);
            }//if//

            OutputStatNamesToStream(
                simulationEnginePtr->GetRuntimeStatisticsSystem(),
                nodeId,
                statNameString,
                guiStream);

            guiStream << endl;
        }
        else if (command == "send_all_stat_names") {
            OutputStatNamesToStream(
                simulationEnginePtr->GetRuntimeStatisticsSystem(),
                ANY_NODEID,
                "*",
                guiStream);
            guiStream << endl;
        }
        else if (command == "send_all_trace_tag_names") {
            for(size_t i = 0; (i < GetNumberTraceTags()); i++) {
                guiStream << ' ' << GetTraceTagName(TraceTagType(i));
            }//for//
            guiStream << endl;
        }
        else if (command == "send_last_stat_value_batch") {
            string statNameString;
            guiCommandLineStream >> statNameString;

            vector<NodeIdType> nodeIds;

            while (!guiCommandLineStream.eof()) {
                NodeIdType aNodeId;
                guiCommandLineStream >> aNodeId;
                nodeIds.push_back(aNodeId);
            }//while//

            OutputLastStatValuesForAListOfNodesToStream(
                guiStatViewCollection,
                nodeIds,
                statNameString,
                simulationEnginePtr->CurrentTime(),
                guiStream);

            guiStream << endl;
        }
        else if (command == "send_stat_value_batch") {
            string statNameString;
            TimeType statTime;
            guiCommandLineStream >> statNameString >> statTime;

            vector<NodeIdType> nodeIds;

            while (!guiCommandLineStream.eof()) {
                NodeIdType aNodeId;
                guiCommandLineStream >> aNodeId;
                nodeIds.push_back(aNodeId);
            }//while//

            guiStream << statTime;
            OutputLastStatValuesForAListOfNodesToStream(
                guiStatViewCollection,
                nodeIds,
                statNameString,
                statTime,
                guiStream);

            guiStream << endl;
        }
        else if (command == "stream_stat_values") {

            NodeIdType nodeId;
            ReadNodeIdFromStream(aLine, guiCommandLineStream, nodeId);
            string statNameString;
            guiCommandLineStream >> statNameString;

            string statSendIntervalString;
            guiCommandLineStream >> statSendIntervalString;

            if (!guiCommandLineStream.fail()) {
                TimeType statSendInterval;
                bool succeeded;
                ConvertStringToTime(statSendIntervalString, statSendInterval, succeeded);
                if (!succeeded) {
                    cerr << "Error in Gui command at stream interval time : " << aLine << endl;
                    exit(1);
                }//if//

                const bool lastValueMode = false;
                (*this).SetupStreamingOfAStatsLastValues(nodeId, statNameString, statSendInterval, lastValueMode);

                guiStream << confirmCommandString << endl;
            }//if//
        }
        else if (command == "send_period_stat_events_batch") {
            string statNameString;
            guiCommandLineStream >> statNameString;
            TimeType periodStartTime;
            guiCommandLineStream >> periodStartTime;

            vector<NodeIdType> nodeIds;
            while (!guiCommandLineStream.eof()) {
                NodeIdType aNodeId;
                guiCommandLineStream >> aNodeId;
                nodeIds.push_back(aNodeId);
            }//while//

            OutputPeriodStatEventsForAListOfNodesToStream(guiStatViewCollection, nodeIds, statNameString, periodStartTime, guiStream);

            guiStream << endl;
        }
        else if (command == "send_all_node_positions") {
            theNetworkSimulator.OutputNodePositionsInXY(ZERO_TIME, guiStream);
            guiStream << endl;
            lastNodePositionOutputTime = this->simulationEnginePtr->CurrentTime();
        }
        else if (command == "send_node_position_updates") {
            theNetworkSimulator.OutputNodePositionsInXY(lastNodePositionOutputTime, guiStream);
            guiStream << endl;
            lastNodePositionOutputTime = this->simulationEnginePtr->CurrentTime();
        }
        else if (command == "stream_node_position_updates") {
            string positionUpdateIntervalString;
            guiCommandLineStream >> positionUpdateIntervalString;

            if (!guiCommandLineStream.fail()) {
                TimeType positionUpdateInterval;
                bool succeeded;
                ConvertStringToTime(positionUpdateIntervalString, positionUpdateInterval, succeeded);
                if (!succeeded) {
                    cerr << "Error in Gui command at stream interval time : " << aLine << endl;
                    exit(1);
                }//if//

                (*this).SetupStreamingOfPositionUpdates(positionUpdateInterval);

                guiStream << confirmCommandString << endl;
            }//if//
        }
        else if (command == "stream_emulation_time") {
            string timeOutputIntervalString;
            guiCommandLineStream >> timeOutputIntervalString;

            if (!guiCommandLineStream.fail()) {
                TimeType timeOutputInterval;
                bool succeeded;
                ConvertStringToTime(timeOutputIntervalString, timeOutputInterval, succeeded);
                if (!succeeded) {
                    cerr << "Error in Gui command at stream interval time : " << aLine << endl;
                    exit(1);
                }//if//

                (*this).SetupStreamingOfEmulationTime(timeOutputInterval);

                guiStream << confirmCommandString << endl;
            }//if//
        }
        else if (command == "send_all_node_ids") {
            theNetworkSimulator.OutputAllNodeIds(guiStream);
            guiStream << endl;
        }
        else if (command == "send_stat_data") {
            NodeIdType nodeId;
            ReadNodeIdFromStream(aLine, guiCommandLineStream, nodeId);
            string statNameString;
            guiCommandLineStream >> statNameString;

            if (guiCommandLineStream.fail()) {
                cerr << "Error in Gui command at stat name: " << endl << aLine << endl;
                exit(1);
            }//if//

            TimeType startTime = ZERO_TIME;
            TimeType endTime = INFINITE_TIME;

            if (!guiCommandLineStream.eof()) {

                string startTimeString;

                guiCommandLineStream >> startTimeString;

                bool succeeded = false;

                if (!guiCommandLineStream.fail()) {
                    ConvertStringToTime(startTimeString, startTime, succeeded);
                }//if//

                if (!succeeded) {
                    cerr << "Error in Gui command at start time: " << endl << aLine << endl;
                    exit(1);
                }//if//

                if (!guiCommandLineStream.eof()) {
                    string endTimeString;
                    guiCommandLineStream >> endTimeString;

                    succeeded = false;

                    if ((!guiCommandLineStream.fail()) && (guiCommandLineStream.eof())) {
                        ConvertStringToTime(endTimeString, endTime, succeeded);
                    }//if//

                    if (!succeeded) {
                        cerr << "Error in Gui command at stat end time : " << aLine << endl;
                        exit(1);
                    }//if//
                }//if//
            }//if//

            OutputAStatToStream(
                this->guiStatViewCollection,
                nodeId,
                statNameString,
                false,
                startTime,
                endTime,
                guiStream);
        }
        else if (command == "send_pathloss_batch") {

            PropagationInformationType informationType;
            guiCommandLineStream >> informationType;

            NodeIdType txNodeId;

            ReadNodeIdFromStream(aLine, guiCommandLineStream, txNodeId);

            InterfaceIdType interfaceName;
            guiCommandLineStream >> interfaceName;

            assert(guiCommandLineStream.good());

            const unsigned int interfaceIndex =
                theNetworkSimulator.LookupInterfaceIndex(txNodeId, interfaceName);

            while(!guiCommandLineStream.eof()) {
                double rxPositionXMeters;
                double rxPositionYMeters;
                double rxPositionZMeters;
                guiCommandLineStream >> rxPositionXMeters;
                guiCommandLineStream >> rxPositionYMeters;
                guiCommandLineStream >> rxPositionZMeters;

                assert(!guiCommandLineStream.fail());

                PropagationStatisticsType propagationStat;

                theNetworkSimulator.CalculatePathlossFromNodeToLocation(
                    txNodeId,
                    informationType,
                    interfaceIndex,
                    rxPositionXMeters,
                    rxPositionYMeters,
                    rxPositionZMeters,
                    propagationStat);

                OutputPropagationStatToStream(informationType, propagationStat, guiStream);

            }//while//

            guiStream << endl;
        }
        else if (command == "send_node_pathloss_batch") {

            PropagationInformationType informationType;
            guiCommandLineStream >> informationType;

            NodeIdType txNodeId;

            ReadNodeIdFromStream(aLine, guiCommandLineStream, txNodeId);

            InterfaceIdType txInterfaceName;
            guiCommandLineStream >> txInterfaceName;

            assert(guiCommandLineStream.good());

            const unsigned int txInterfaceIndex =
                theNetworkSimulator.LookupInterfaceIndex(txNodeId, txInterfaceName);

            while(!guiCommandLineStream.eof()) {
                NodeIdType rxNodeId;

                ReadNodeIdFromStream(aLine, guiCommandLineStream, rxNodeId);

                InterfaceIdType rxInterfaceName;
                guiCommandLineStream >> rxInterfaceName;

                const unsigned int rxInterfaceIndex =
                    theNetworkSimulator.LookupInterfaceIndex(rxNodeId, rxInterfaceName);

                assert(!guiCommandLineStream.fail());

                PropagationStatisticsType propagationStat;

                theNetworkSimulator.CalculatePathlossFromNodeToNode(
                    txNodeId,
                    rxNodeId,
                    informationType,
                    txInterfaceIndex,
                    rxInterfaceIndex,
                    propagationStat);

                OutputPropagationStatToStream(informationType, propagationStat, guiStream);

            }//while//

            guiStream << endl;
        }
        else if (command == "send_antenna_position") {

            NodeIdType txNodeId;
            ReadNodeIdFromStream(aLine, guiCommandLineStream, txNodeId);

            InterfaceIdType interfaceName;
            guiCommandLineStream >> interfaceName;

            const unsigned int interfaceIndex =
                theNetworkSimulator.LookupInterfaceIndex(txNodeId, interfaceName);

            const ObjectMobilityPosition antennaPosition =
                theNetworkSimulator.GetAntennaLocation(txNodeId, interfaceIndex);
            
            guiStream
                << antennaPosition.X_PositionMeters() << ' '
                << antennaPosition.Y_PositionMeters() << ' '
                << antennaPosition.HeightFromGroundMeters() << endl;
        }
        else if (command == "trigger_application") {

            NodeIdType nodeId;
            ReadNodeIdFromStream(aLine, guiCommandLineStream, nodeId);

            theNetworkSimulator.TriggerApplication(nodeId);
            
            guiStream << confirmCommandString << endl;
        }
        else if (command == "create_new_network_node") {
            NodeIdType newNodeId;
            ReadNodeIdFromStream(aLine, guiCommandLineStream, newNodeId);

            string newNodeTypeName;
            guiCommandLineStream >> newNodeTypeName;
            assert(!guiCommandLineStream.fail() && guiCommandLineStream.eof());

            theNetworkSimulator.CreateNewNode(theParameterDatabaseReader, newNodeId, newNodeTypeName);
        }
        else if (command == "delete_network_node") {
            assert(false && "TBD: delete_network_node");
            abort();
        }
        else if (command == "send_added_nodes") {
            theNetworkSimulator.OutputRecentlyAddedNodeIdsWithTypes(guiStream);
            guiStream << endl;
        }
        else if (command == "send_deleted_nodes") {
            theNetworkSimulator.OutputRecentlyDeletedNodeIds(guiStream);
            guiStream << endl;
        }
        else if (command == "send_added_trace_texts") {
            TraceSubsystem& traceSubsystem =
                simulationEnginePtr->GetTraceSubsystem();
            traceSubsystem.OutputRecentlyAddedTraceTexts(guiStream);
            guiStream << endl;
        }
        else if (command == "enable_online_trace") {
            isOnlineTraceEnabled = true;
            guiStream << confirmCommandString << endl;
        }
        else {
            cerr << "Error: Unknown GUI command on socket." << endl;
            exit(1);

        }//if//

    }//while//

}//GetAndExecuteGuiCommands//


void GuiInterfacingSubsystem::Implementation::SetupStreamingOfAStatsLastValues(
    const NodeIdType nodeId,
    const string& statName,
    const TimeType& statSendInterval,
    const bool lastValueMode)
{
    typedef map<StatViewCollection::StatViewIdType, shared_ptr<CounterStatView> >::const_iterator CounterStatIterType;
    typedef map<StatViewCollection::StatViewIdType, shared_ptr<RealStatView> >::const_iterator RealStatIterType;

    StatViewCollection::StatViewIdType statViewId(statName, nodeId);

    CounterStatIterType iter = guiStatViewCollection.counterStatViewPtrs.find(statViewId);

    if (iter != guiStatViewCollection.counterStatViewPtrs.end()) {
        shared_ptr<CounterStatView> statViewPtr = iter->second;

        shared_ptr<CounterStatStreamer> statStreamerPtr(
            new CounterStatStreamer(statViewPtr, statSendInterval, lastValueMode));

        statStreamerQueue.push(statStreamerPtr);
    }
    else {
        RealStatIterType iter = guiStatViewCollection.realStatViewPtrs.find(statViewId);

        if (iter != guiStatViewCollection.realStatViewPtrs.end()) {

            shared_ptr<RealStatView> statViewPtr = iter->second;

            shared_ptr<RealStatStreamer> statStreamerPtr(
                new RealStatStreamer(statViewPtr, statSendInterval, lastValueMode));

            statStreamerQueue.push(statStreamerPtr);
        }
    }//if//

}//SetupStreamingOfAStatsLastValues//



void GuiInterfacingSubsystem::Implementation::SendStreamedOutput(
    const NetworkSimulator& theNetworkSimulator)
{
    // Note this implementation is likely insufficient for large emulations as output
    //  should be staggered to reduce load and to avoid real-time time violations.

    TimeType currentTime = simulationEnginePtr->CurrentTime();

    if (!statStreamerQueue.empty()) {
        while (true) {
            shared_ptr<StatStreamer> streamerPtr = statStreamerQueue.top();

            if (streamerPtr->GetNextSendTime() > currentTime) {
                break;
            }//if//

            statStreamerQueue.pop();
            streamerPtr->OutputStatsToStream(currentTime, guiStream);
            statStreamerQueue.push(streamerPtr);
        }//while//
    }//if//

    if ((nodePositionUpdateInterval != INFINITE_TIME) &&
        (currentTime >= (lastNodePositionOutputTime + nodePositionUpdateInterval))) {

        guiStream << "posupdates ";
        theNetworkSimulator.OutputNodePositionsInXY(lastNodePositionOutputTime, guiStream);
        guiStream << endl;
        lastNodePositionOutputTime = currentTime;
    }//if//

    if ((emulationTimeOutputInterval != INFINITE_TIME) &&
        (currentTime >= (lastEmulationTimeOutputTime + emulationTimeOutputInterval))) {

        assert(isInEmulationMode);

        guiStream << "addedtracetexts ";
        TraceSubsystem& traceSubsystem =
            simulationEnginePtr->GetTraceSubsystem();
        traceSubsystem.OutputRecentlyAddedTraceTexts(guiStream);
        guiStream << endl;

        guiStream << "emutime " << ConvertTimeToStringSecs(currentTime) << endl;

        lastEmulationTimeOutputTime = currentTime;
    }

}//SendEmulationOutput//



//--------------------------------------------------------------------------------------------------

GuiInterfacingSubsystem::GuiInterfacingSubsystem(
    const shared_ptr<SimulationEngine>& simulationEnginePtr,
    const unsigned short int guiPortNumber,
    const unsigned short int guiPauseCommandPortNumber)
    :
    implPtr(new Implementation(simulationEnginePtr, guiPortNumber, guiPauseCommandPortNumber))
{}


GuiInterfacingSubsystem::~GuiInterfacingSubsystem() { }

void GuiInterfacingSubsystem::RunInEmulationMode()
{
    implPtr->isInEmulationMode = true;
}


void GuiInterfacingSubsystem::GetAndExecuteGuiCommands(
    ParameterDatabaseReader& theParameterDatabaseReader,
    NetworkSimulator& theNetworkSimulator,
    TimeType& simulateUpToTime)
{
    implPtr->GetAndExecuteGuiCommands(theParameterDatabaseReader, theNetworkSimulator, simulateUpToTime);
}

void GuiInterfacingSubsystem::SendStreamedOutput(const NetworkSimulator& theNetworkSimulator)
{
    implPtr->SendStreamedOutput(theNetworkSimulator);
}


}//namespace//


