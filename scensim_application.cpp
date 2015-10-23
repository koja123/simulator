// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_app_cbr.h"
#include "scensim_app_vbr.h"
#include "scensim_app_ftp.h"
#include "scensim_app_video.h"
#include "scensim_app_voip.h"
#include "scensim_app_http.h"
#include "scensim_app_flooding.h"
#include "scensim_app_iperf_udp.h"
#include "scensim_app_iperf_tcp.h"
#include "scensim_bundle.h"
#include "scensim_app_trace_based.h"

namespace ScenSim {

using std::ofstream;

const string CbrApplication::modelName = "CbrApp";
const string VbrApplication::modelName = "VbrApp";
const string FtpLikeFlowApplication::modelName = "FtpFlowApp";
const string MultipleFtpApplication::modelName = "FtpApp";
const string VideoStreamingApplication::modelName = "VideoApp";
const string VoipApplication::modelName = "VoipApp";
const string HttpFlowApplication::modelName = "HttpApp";
const string FloodingApplication::modelName = "FloodingApp";
const string IperfUdpApplication::modelName = "IperfUdpApp";
const string IperfUdpApplication::modelNameClientOnly = "IperfUdpClientApp";
const string IperfUdpApplication::modelNameServerOnly = "IperfUdpServerApp";
const string IperfTcpApplication::modelName = "IperfTcpApp";
const string IperfTcpApplication::modelNameClientOnly = "IperfTcpClientApp";
const string IperfTcpApplication::modelNameServerOnly = "IperfTcpServerApp";
const string BundleProtocol::modelName = "Bundle";
const string TraceBasedApplication::modelName = "TraceBasedApp";
const string TcpProtocol::modelName = "Tcp";
const string UdpProtocol::modelName = "Udp";
const string BasicNetworkLayer::modelName = "NetworkLayer";
const string AbstractNetworkMac::modelName = "AbstractMac";

const TimeType CbrSinkApplication::duplicateDetectorWindowTime;
const unsigned int CbrSinkApplication::duplicateDetectorMinWindowSize;
const TimeType VbrSinkApplication::duplicateDetectorWindowTime;
const unsigned int VbrSinkApplication::duplicateDetectorMinWindowSize;

const double MultipleFtpSourceApplication::maximumTcpFlowBlockSizeRate = 0.76;
const double HttpFlowSourceApplication::maximumTcpFlowBlockSizeRate = 0.76;



ApplicationMaker::ApplicationMaker(
    const shared_ptr<SimulationEngineInterface>& initSimulationEngineInterfacePtr,
    const shared_ptr<ApplicationLayer>& initAppLayerPtr,
    const NodeIdType& initNodeId,
    const RandomNumberGeneratorSeedType& initNodeSeed)
    :
    simulationEngineInterfacePtr(initSimulationEngineInterfacePtr),
    appLayerPtr(initAppLayerPtr),
    nodeId(initNodeId),
    nodeSeed(initNodeSeed),
    defaultBundleProtocolPortId(INVALID_PORT)
{
    appSpecificParameterNames[APPLICATION_CBR] = "cbr-destination";
    appSpecificParameterNames[APPLICATION_CBR_WITH_QOS] = "cbr-with-qos-destination";
    appSpecificParameterNames[APPLICATION_VBR] = "vbr-destination";
    appSpecificParameterNames[APPLICATION_VBR_WITH_QOS] = "vbr-with-qos-destination";
    appSpecificParameterNames[APPLICATION_FTP] = "ftp-destination";
    appSpecificParameterNames[APPLICATION_FTP_WITH_QOS] = "ftp-with-qos-destination";
    appSpecificParameterNames[APPLICATION_MULTIFTP] = "multiftp-destination";
    appSpecificParameterNames[APPLICATION_MULTIFTP_WITH_QOS] = "multiftp-with-qos-destination";
    appSpecificParameterNames[APPLICATION_VIDEO] = "video-destination";
    appSpecificParameterNames[APPLICATION_VIDEO_WITH_QOS] = "video-with-qos-destination";
    appSpecificParameterNames[APPLICATION_VOIP] = "voip-destination";
    appSpecificParameterNames[APPLICATION_VOIP_WITH_QOS] = "voip-with-qos-destination";
    appSpecificParameterNames[APPLICATION_HTTP] = "http-destination";
    appSpecificParameterNames[APPLICATION_HTTP_WITH_QOS] = "http-with-qos-destination";
    appSpecificParameterNames[APPLICATION_FLOODING] = "flooding-destination";
    appSpecificParameterNames[APPLICATION_IPERF_UDP] = "iperf-udp-destination";
    appSpecificParameterNames[APPLICATION_IPERF_UDP_CLIENT] = "iperf-udp-client-destination-port";
    appSpecificParameterNames[APPLICATION_IPERF_UDP_SERVER] = "iperf-udp-server-destination-port";
    appSpecificParameterNames[APPLICATION_IPERF_UDP_WITH_QOS] = "iperf-udp-with-qos-destination";
    appSpecificParameterNames[APPLICATION_IPERF_TCP] = "iperf-tcp-destination";
    appSpecificParameterNames[APPLICATION_IPERF_TCP_CLIENT] = "iperf-tcp-client-destination-port";
    appSpecificParameterNames[APPLICATION_IPERF_TCP_SERVER] = "iperf-tcp-server-destination-port";
    appSpecificParameterNames[APPLICATION_IPERF_TCP_WITH_QOS] = "iperf-tcp-with-qos-destination";
    appSpecificParameterNames[APPLICATION_BUNDLE_PROTOCOL] = "bundle-routing-algorithm";
    appSpecificParameterNames[APPLICATION_BUNDLE_MESSAGE] = "bundle-message-destination";
    appSpecificParameterNames[APPLICATION_TRACE_BASED] = "trace-based-app-destination";
    appSpecificParameterNames[APPLICATION_SENSING] = "sensing-coverage-shape-type";

    //add new app
    //Add application specification paramter for user application
    // e.g. appSpecificParameterNames[APPLICATION_USERAPP] = "userapp-destination";
}


static inline
TimeType App_ConvertStringToTime(const string& aString, const string& aLine)
{
    TimeType retTime;
    bool success = false;
    ConvertStringToTime(aString, retTime, success);
    if (!success) {
        cerr << "Error: Application File, Bad Time parameter" << aString << " in Line:" << endl;
        cerr << aLine << endl;
        exit(1);
    }//if//
    return retTime;
}

static inline
NodeIdType App_ConvertStringToSingleNodeId(const string& nodeIdString)
{
    int value = atoi(nodeIdString.c_str());
    if (value <= 0) {
        cerr << "Error: Application File, Bad Destination Node Id parameter " << nodeIdString << endl;
        exit(1);
    }//if//

    return NodeIdType(value);
}

NodeIdType App_ConvertStringToNodeIdOrAnyNodeId(const string& nodeIdString)
{
    if (nodeIdString == "*") {
        return ANY_NODEID;
    }

    return App_ConvertStringToSingleNodeId(nodeIdString);
}

static inline MacQualityOfServiceControlInterface::SchedulingSchemeChoice
App_ConvertStringToSchedulingSchemeChoice(
    const string& modelName,
    const string& schedulingSchemeChoiceString)
{
    MacQualityOfServiceControlInterface::SchedulingSchemeChoice schedulingScheme;

    bool succeeded;

    ConvertStringToSchedulingSchemeChoice(
        MakeLowerCaseString(schedulingSchemeChoiceString),
        succeeded,
        schedulingScheme);

    if (!succeeded) {
        cerr << "Error in " << modelName << " Application: Scheduling Scheme not recognized in:" << endl;
        cerr << "      >" << schedulingSchemeChoiceString << endl;
        exit(1);
    }//if//

    return schedulingScheme;
}

void ApplicationMaker::GetApplicationInstanceIds(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    map<ApplicationType, vector<ApplicationInstanceInfo> >& applicationInstanceIdsPerApplicationModel)
{
    applicationInstanceIdsPerApplicationModel.clear();

    typedef map<ApplicationType, string>::const_iterator AppTypeIter;
    typedef set<NodeIdType>::const_iterator NodeIdIter;
    typedef set<InterfaceOrInstanceIdType>::const_iterator AppInstanceIter;

    for(AppTypeIter appTypeIter = appSpecificParameterNames.begin();
        (appTypeIter != appSpecificParameterNames.end()); appTypeIter++) {

        const ApplicationType& applicationType = (*appTypeIter).first;
        const string& parameterName = (*appTypeIter).second;

        set<NodeIdType> nodeIds;
        theParameterDatabaseReader.MakeSetOfAllNodeIdsWithParameter(parameterName, nodeIds);

        //bundle protocol could be set as global paramter (all comm nodes) in cofiguration file.
        if ((applicationType == APPLICATION_BUNDLE_PROTOCOL) && (nodeIds.empty())) {
            if (theParameterDatabaseReader.ParameterExists(parameterName)) {
                //set all comm nodes
                theParameterDatabaseReader.MakeSetOfAllCommNodeIds(nodeIds);
            }//if//
        }//if//

        for(NodeIdIter nodeIdIter = nodeIds.begin(); (nodeIdIter != nodeIds.end()); nodeIdIter++) {
            const NodeIdType& sourceNodeId = *nodeIdIter;

            if (commNodeIds.find(sourceNodeId) == commNodeIds.end()) {
                continue;
            }//if//

            set<InterfaceOrInstanceIdType> instanceIds;
            theParameterDatabaseReader.MakeSetOfAllInterfaceIdsForANode(sourceNodeId, parameterName, instanceIds);
            if (instanceIds.empty()) {
                instanceIds.insert(nullInstanceId);
            }//if//

            for(AppInstanceIter instanceIter = instanceIds.begin();
                (instanceIter != instanceIds.end()); instanceIter++) {
                const InterfaceOrInstanceIdType& instanceId = *instanceIter;

                applicationInstanceIdsPerApplicationModel[applicationType].push_back(
                    ApplicationInstanceInfo(
                        sourceNodeId,
                        instanceId,
                        appLayerPtr->GetNewApplicationInstanceNumber()));

            }//for//
        }//for//
    }//for//
}//GetApplicationInstanceIds//

void ApplicationMaker::ReadApplicationLineFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag)
{
    theParameterDatabaseReader.MakeSetOfAllCommNodeIds(commNodeIds);

    if (commNodeIds.find(nodeId) == commNodeIds.end()) {
        return;
    }

    map<ApplicationType, vector<ApplicationInstanceInfo> > applicationInstanceIdsPerApplicationModel;

    (*this).GetApplicationInstanceIds(
        theParameterDatabaseReader,
        applicationInstanceIdsPerApplicationModel);

    (*this).ReadApplicationIntancesFromConfig(
        theParameterDatabaseReader,
        theGlobalNetworkingObjectBag,
        applicationInstanceIdsPerApplicationModel);
}//ReadApplicationLineFromConfig//

void ApplicationMaker::ReadSpecificApplicationLineFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const NodeIdType& sourceNodeId,
    const InterfaceOrInstanceIdType& instanceId)
{
    map<ApplicationType, vector<ApplicationInstanceInfo> > applicationInstanceIdsPerApplicationModel;

    typedef map<ApplicationType, string>::const_iterator AppTypeIter;

    for(AppTypeIter appTypeIter = appSpecificParameterNames.begin();
        (appTypeIter != appSpecificParameterNames.end()); appTypeIter++) {

        const ApplicationType& applicationType = (*appTypeIter).first;
        const string& parameterName = (*appTypeIter).second;

        if (theParameterDatabaseReader.ParameterExists(parameterName, sourceNodeId, instanceId)) {

            applicationInstanceIdsPerApplicationModel[applicationType].push_back(
                ApplicationInstanceInfo(
                    sourceNodeId,
                    instanceId,
                    appLayerPtr->GetNewApplicationInstanceNumber()));

        }//if//

    }//for//

    (*this).ReadApplicationIntancesFromConfig(
        theParameterDatabaseReader,
        theGlobalNetworkingObjectBag,
        applicationInstanceIdsPerApplicationModel);
}//ReadSpecificApplicationLineFromConfig//

void ApplicationMaker::ReadApplicationIntancesFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const map<ApplicationType, vector<ApplicationInstanceInfo> >& applicationInstanceIdsPerApplicationModel)
{
    typedef map<ApplicationType, vector<ApplicationInstanceInfo> >::const_iterator IterType;

    for(IterType iter = applicationInstanceIdsPerApplicationModel.begin();
        (iter != applicationInstanceIdsPerApplicationModel.end()); iter++) {

        const ApplicationType& applicationType = (*iter).first;
        const vector<ApplicationInstanceInfo>& applicationInstanceIds = (*iter).second;

        for(unsigned int i = 0; i < applicationInstanceIds.size(); i++) {
            const ApplicationInstanceInfo& applicationInstanceId = applicationInstanceIds[i];

            switch(applicationType) {
            case APPLICATION_CBR:
                (*this).ReadCbrFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_CBR_WITH_QOS:
                (*this).ReadCbrWithQosFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_VBR:
                (*this).ReadVbrFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_VBR_WITH_QOS:
                (*this).ReadVbrWithQosFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_FTP:
                (*this).ReadFtpFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_FTP_WITH_QOS:
                (*this).ReadFtpWithQosFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_MULTIFTP:
                (*this).ReadMultiFtpFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_MULTIFTP_WITH_QOS:
                (*this).ReadMultiFtpWithQosFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_VIDEO:
                (*this).ReadVideoFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_VIDEO_WITH_QOS:
                (*this).ReadVideoWithQosFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_VOIP:
                (*this).ReadVoipFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_VOIP_WITH_QOS:
                (*this).ReadVoipWithQosFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_HTTP:
                (*this).ReadHttpFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_HTTP_WITH_QOS:
                (*this).ReadHttpWithQosFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_FLOODING:
                (*this).ReadFloodingFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_IPERF_UDP:
                (*this).ReadIperfUdpFromConfig(
                    theParameterDatabaseReader, applicationInstanceId, true/*createClient*/, true/*createServer*/);
                break;

            case APPLICATION_IPERF_UDP_CLIENT:
                (*this).ReadIperfUdpFromConfig(
                    theParameterDatabaseReader, applicationInstanceId, true/*createClient*/, false/*createServer*/);
                break;

            case APPLICATION_IPERF_UDP_SERVER:
                (*this).ReadIperfUdpFromConfig(
                    theParameterDatabaseReader, applicationInstanceId, false/*createClient*/, true/*createServer*/);
                break;

            case APPLICATION_IPERF_UDP_WITH_QOS:
                (*this).ReadIperfUdpWithQosFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_IPERF_TCP:
                (*this).ReadIperfTcpFromConfig(
                    theParameterDatabaseReader, applicationInstanceId, true/*createClient*/, true/*createServer*/);
                break;

            case APPLICATION_IPERF_TCP_CLIENT:
                (*this).ReadIperfTcpFromConfig(
                    theParameterDatabaseReader, applicationInstanceId, true/*createClient*/, false/*createServer*/);
                break;

            case APPLICATION_IPERF_TCP_SERVER:
                (*this).ReadIperfTcpFromConfig(
                    theParameterDatabaseReader, applicationInstanceId, false/*createClient*/, true/*createServer*/);
                break;

            case APPLICATION_IPERF_TCP_WITH_QOS:
                (*this).ReadIperfTcpWithQosFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_BUNDLE_PROTOCOL:
                (*this).ReadBundleProtocolFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_BUNDLE_MESSAGE:
                (*this).ReadBundleMessageFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_TRACE_BASED:
                (*this).ReadTraceBasedFromConfig(
                    theParameterDatabaseReader, applicationInstanceId);
                break;

            case APPLICATION_SENSING:
                (*this).ReadSensingFromConfig(
                    theParameterDatabaseReader, theGlobalNetworkingObjectBag, applicationInstanceId);
                break;

            default:
                assert(false && "Implement application parser!");
            }//switch//
        }//for//
    }//for//

}//ReadApplicationLineFromConfig//

//-----------------------------------------------------------------------------

void ApplicationMaker::ReadCbrFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;
    const NodeIdType destinationNodeIdOrAnyNodeId =
        App_ConvertStringToNodeIdOrAnyNodeId(
            theParameterDatabaseReader.ReadString(
                "cbr-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<CbrSourceApplication> appPtr(
            new CbrSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeIdOrAnyNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if ((destinationNodeIdOrAnyNodeId == nodeId) ||
             (destinationNodeIdOrAnyNodeId == ANY_NODEID)) {

        // Every node makes a sink to field the broadcast packet for this application.

        shared_ptr<CbrSinkApplication> appPtr(
            new CbrSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeIdOrAnyNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadCbrFromConfig//

void ApplicationMaker::ReadCbrWithQosFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;
    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "cbr-with-qos-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<CbrSourceApplication> appPtr(
            new CbrSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if (destinationNodeId == nodeId) {
        shared_ptr<CbrSinkApplication> appPtr(
            new CbrSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadCbrWithQosFromConfig//

//-----------------------------------------------------------------------------

void ApplicationMaker::ReadVbrFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;
    const NodeIdType destinationNodeIdOrAnyNodeId =
        App_ConvertStringToNodeIdOrAnyNodeId(
            theParameterDatabaseReader.ReadString(
                "vbr-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<VbrSourceApplication> appPtr(
            new VbrSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeIdOrAnyNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/,
                nodeSeed));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if ((destinationNodeIdOrAnyNodeId == nodeId) ||
             (destinationNodeIdOrAnyNodeId == ANY_NODEID)) {

        // Every node makes a sink to field the broadcast packet for this application.

        shared_ptr<VbrSinkApplication> appPtr(
            new VbrSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeIdOrAnyNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadVbrFromConfig//

void ApplicationMaker::ReadVbrWithQosFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;
    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "vbr-with-qos-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<VbrSourceApplication> appPtr(
            new VbrSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/,
                nodeSeed));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if (destinationNodeId == nodeId) {
        shared_ptr<VbrSinkApplication> appPtr(
            new VbrSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadVbrWithQosFromConfig//

//-----------------------------------------------------------------------------

void ApplicationMaker::ReadFtpFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "ftp-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<FtpLikeFlowSourceApplication> appPtr(
            new FtpLikeFlowSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if (destinationNodeId == nodeId) {
        shared_ptr<FtpLikeFlowSinkApplication> appPtr(
            new FtpLikeFlowSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadFtpFromConfig//

void ApplicationMaker::ReadFtpWithQosFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "ftp-with-qos-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<FtpLikeFlowSourceApplication> appPtr(
            new FtpLikeFlowSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if (destinationNodeId == nodeId) {
        shared_ptr<FtpLikeFlowSinkApplication> appPtr(
            new FtpLikeFlowSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadFtpWithQosFromConfig//


//-----------------------------------------------------------------------------

void ApplicationMaker::ReadMultiFtpFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;
    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "multiftp-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<MultipleFtpSourceApplication> appPtr(
            new MultipleFtpSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/,
                nodeSeed));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if (destinationNodeId == nodeId) {
        shared_ptr<MultipleFtpSinkApplication> appPtr(
            new MultipleFtpSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadMultiftpFromConfig//

void ApplicationMaker::ReadMultiFtpWithQosFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "multiftp-with-qos-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<MultipleFtpSourceApplication> appPtr(
            new MultipleFtpSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/,
                nodeSeed));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if (destinationNodeId == nodeId) {
        shared_ptr<MultipleFtpSinkApplication> appPtr(
            new MultipleFtpSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadMultiFtpWithQosFromConfig//

//-----------------------------------------------------------------------------

void ApplicationMaker::ReadVideoFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "video-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<VideoStreamingSourceApplication> appPtr(
            new VideoStreamingSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/,
                nodeSeed));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if (destinationNodeId == nodeId) {
        shared_ptr<VideoStreamingSinkApplication> appPtr(
            new VideoStreamingSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadVideoFromConfig//

void ApplicationMaker::ReadVideoWithQosFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "video-with-qos-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<VideoStreamingSourceApplication> appPtr(
            new VideoStreamingSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/,
                nodeSeed));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if (destinationNodeId == nodeId) {
        shared_ptr<VideoStreamingSinkApplication> appPtr(
            new VideoStreamingSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadVideoWithQosFromConfig//

//-----------------------------------------------------------------------------

void ApplicationMaker::ReadVoipFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "voip-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<VoipSourceApplication> appPtr(
            new VoipSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/,
                nodeSeed));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if (destinationNodeId == nodeId) {
        shared_ptr<VoipSinkApplication> appPtr(
            new VoipSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadVoipFromConfig//

void ApplicationMaker::ReadVoipWithQosFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "voip-with-qos-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<VoipSourceApplication> appPtr(
            new VoipSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/,
                nodeSeed));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if (destinationNodeId == nodeId) {
        shared_ptr<VoipSinkApplication> appPtr(
            new VoipSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadVoipWithQosFromConfig//

//-----------------------------------------------------------------------------

void ApplicationMaker::ReadHttpFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "http-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<HttpFlowSourceApplication> appPtr(
            new HttpFlowSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/,
                nodeSeed));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if (destinationNodeId == nodeId) {
        shared_ptr<HttpFlowSinkApplication> appPtr(
            new HttpFlowSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadHttpFromConfig//


void ApplicationMaker::ReadHttpWithQosFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "http-with-qos-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if (sourceNodeId == nodeId) {
        shared_ptr<HttpFlowSourceApplication> appPtr(
            new HttpFlowSourceApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/,
                nodeSeed));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }
    else if (destinationNodeId == nodeId) {
        shared_ptr<HttpFlowSinkApplication> appPtr(
            new HttpFlowSinkApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadHttpWithQosFromConfig//

//-----------------------------------------------------------------------------

void ApplicationMaker::ReadFloodingFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeIdOrAnyNodeId =
        App_ConvertStringToNodeIdOrAnyNodeId(
            theParameterDatabaseReader.ReadString(
                "flooding-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    assert(destinationNodeIdOrAnyNodeId == ANY_NODEID);

    if (floodingApplicationPtr == nullptr) {
        shared_ptr<FloodingApplication> appPtr(
            new FloodingApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                nodeId,
                defaultDestinationPortId));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
        floodingApplicationPtr = appPtr;
    }//if//

    if (sourceNodeId == nodeId) {
        floodingApplicationPtr->AddSenderSetting(
            theParameterDatabaseReader,
            instanceId);
    }//if//

    floodingApplicationPtr->AddReceiverSetting(instanceId);

}//ReadFloodingFromConfig//


void ApplicationMaker::ReadSensingFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const GlobalNetworkingObjectBag& theGlobalNetworkingObjectBag,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    if (nodeId != sourceNodeId) {
        return;
    }

    shared_ptr<SensingApplication> appPtr(
        new SensingApplication(
            theParameterDatabaseReader,
            simulationEngineInterfacePtr,
            theGlobalNetworkingObjectBag.sensingSubsystemInterfacePtr->CreateSensingModel(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                nodeId,
                instanceId,
                nodeSeed),
            nodeId,
            instanceId));

    appLayerPtr->AddApp(appPtr);
    appPtr->CompleteInitialization();

}//ReadSensingFromConfig//


//-----------------------------------------------------------------------------

void ApplicationMaker::ReadIperfUdpFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId,
    const bool createClient,
    const bool createServer)
{
    const NodeIdType& nodeIdWithParameter = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    NodeIdType sourceNodeId = INVALID_NODEID;
    NodeIdType destinationNodeId = INVALID_NODEID;

    if (createClient && createServer) {
        sourceNodeId = nodeIdWithParameter;
        destinationNodeId = App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString("iperf-udp-destination", sourceNodeId, instanceId));
    }
    else if (createClient) {
        sourceNodeId = nodeIdWithParameter;
    }
    else if (createServer) {
        destinationNodeId = nodeIdWithParameter;
    }//if//

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if ((sourceNodeId == nodeId) ||
        (destinationNodeId == nodeId)) {
        shared_ptr<IperfUdpApplication> appPtr(
            new IperfUdpApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                nodeIdWithParameter,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadIperfUdpFromConfig//

//-----------------------------------------------------------------------------

void ApplicationMaker::ReadIperfUdpWithQosFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& nodeIdWithParameter = applicationInstanceId.nodeIdWithParameter;
    const NodeIdType& sourceNodeId = nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "iperf-udp-with-qos-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if ((sourceNodeId == nodeId) ||
        (destinationNodeId == nodeId)) {
        shared_ptr<IperfUdpApplication> appPtr(
            new IperfUdpApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                nodeIdWithParameter,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadIperfUdpWithQosFromConfig//

//-----------------------------------------------------------------------------

void ApplicationMaker::ReadIperfTcpFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId,
    const bool createClient,
    const bool createServer)
{
    const NodeIdType& nodeIdWithParameter = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    NodeIdType sourceNodeId = INVALID_NODEID;
    NodeIdType destinationNodeId = INVALID_NODEID;

    if (createClient && createServer) {
        sourceNodeId = nodeIdWithParameter;
        destinationNodeId = App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString("iperf-tcp-destination", sourceNodeId, instanceId));
    }
    else if (createClient) {
        sourceNodeId = nodeIdWithParameter;
    }
    else if (createServer) {
        destinationNodeId = nodeIdWithParameter;
    }//if//

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if ((sourceNodeId == nodeId) ||
        (destinationNodeId == nodeId)) {
        shared_ptr<IperfTcpApplication> appPtr(
            new IperfTcpApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                nodeIdWithParameter,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                false/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadIperfTcpFromConfig//

//-----------------------------------------------------------------------------

void ApplicationMaker::ReadIperfTcpWithQosFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& nodeIdWithParameter = applicationInstanceId.nodeIdWithParameter;
    const NodeIdType& sourceNodeId = nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeId =
        App_ConvertStringToSingleNodeId(
            theParameterDatabaseReader.ReadString(
                "iperf-tcp-with-qos-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if ((sourceNodeId == nodeId) ||
        (destinationNodeId == nodeId)) {
        shared_ptr<IperfTcpApplication> appPtr(
            new IperfTcpApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                nodeIdWithParameter,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId,
                true/*enableQosControl*/));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
    }//if//

}//ReadIperfTcpWithQosFromConfig//

//-----------------------------------------------------------------------------

void ApplicationMaker::ReadBundleProtocolFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    //set port number from first bundle protocol
    if (defaultBundleProtocolPortId == INVALID_PORT) {
        defaultBundleProtocolPortId =
            applicationInstanceId.GetDefaultDestinationPortNumber();
    }//if//

    if (sourceNodeId == nodeId) {

        if (bundleProtocolPtr != nullptr) {
            cerr << "Multiple Bundle Protocols can not be set to a node." << endl;
            exit(1);
        }//if//

        shared_ptr<BundleProtocol> appPtr(
            new BundleProtocol(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                nodeId,
                defaultBundleProtocolPortId));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization();
        bundleProtocolPtr = appPtr;

    }//if//

}//ReadBundleProtocolFromConfig//


void ApplicationMaker::ReadBundleMessageFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeIdOrAnyNodeId =
        App_ConvertStringToNodeIdOrAnyNodeId(
            theParameterDatabaseReader.ReadString(
                "bundle-message-destination", sourceNodeId, instanceId));

    if (sourceNodeId == nodeId) {

        if (bundleProtocolPtr == nullptr) {
            cerr << "Need to set Bundle Protocol for node: " << nodeId << endl;
            exit(1);
        }//if//

        bundleProtocolPtr->AddSenderSetting(
            theParameterDatabaseReader,
            instanceId,
            destinationNodeIdOrAnyNodeId);
    }//if//


}//ReadBundleMessageFromConfig//

//-----------------------------------------------------------------------------

void ApplicationMaker::ReadTraceBasedFromConfig(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const ApplicationInstanceInfo& applicationInstanceId)
{
    const NodeIdType& sourceNodeId = applicationInstanceId.nodeIdWithParameter;
    const InterfaceOrInstanceIdType& instanceId = applicationInstanceId.instanceId;

    const NodeIdType destinationNodeId =
        App_ConvertStringToNodeIdOrAnyNodeId(
            theParameterDatabaseReader.ReadString(
                "trace-based-app-destination", sourceNodeId, instanceId));

    const unsigned short defaultDestinationPortId =
        applicationInstanceId.GetDefaultDestinationPortNumber();

    if ((sourceNodeId == nodeId) || (destinationNodeId == nodeId) || (destinationNodeId == ANY_NODEID)) {
        shared_ptr<TraceBasedApplication> appPtr(
            new TraceBasedApplication(
                theParameterDatabaseReader,
                simulationEngineInterfacePtr,
                instanceId,
                sourceNodeId,
                destinationNodeId,
                defaultDestinationPortId));

        appLayerPtr->AddApp(appPtr);
        appPtr->CompleteInitialization(theParameterDatabaseReader);
    }//if//

}//ReadTraceBasedFromConfig//

}//namespace//

