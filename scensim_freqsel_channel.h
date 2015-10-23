// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_FREQSEL_CHANNEL_H
#define SCENSIM_FREQSEL_CHANNEL_H

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include "scensim_time.h"
#include "scensim_nodeid.h"
#include "scensim_support.h"
#include "scensim_parmio.h"

namespace ScenSim {

using std::cerr;
using std::endl;
using std::string;
using std::auto_ptr;
using std::shared_ptr;
using std::enable_shared_from_this;
using std::map;
using std::vector;
using std::istream;
using std::istringstream;
using ScenSim::TimeType;
using ScenSim::INFINITE_TIME;
using ScenSim::NodeIdType;
using ScenSim::ConvertToString;
using ScenSim::IsAConfigFileCommentLine;
using ScenSim::DeleteTrailingSpaces;
using ScenSim::ConvertStringToTime;
using ScenSim::ConvertStringToLowerCase;
using ScenSim::ParameterDatabaseReader;

//--------------------------------------------------------------------------------------------------

// Localized (to node) interface for for future caching and thread issues (sharing) to global
// models such as a pathloss matrix.


class FrequencySelectiveFadingModelInterface {
public:
    virtual ~FrequencySelectiveFadingModelInterface() { }

    virtual void SetTime(
        const unsigned int channelNumber,
        const TimeType& timeForData) = 0;

    virtual const vector<double>& GetSubcarrierOrSubchannelDbFadingVector(
        const unsigned int channelNumber,
        const NodeIdType& sourceNodeId) const = 0;

    // For multi-sector basestations and APs.

    virtual const vector<double>& GetSubcarrierOrSubchannelDbFadingVector(
        const unsigned int channelNumber,
        const unsigned int cellSectorIndex,
        const NodeIdType& sourceNodeId,
        const unsigned int sourceCellSectorIndex = 0) const = 0;


};//FrequencySelectiveFadingModelInterface//


class FrequencySelectiveFadingModel {
public:
    virtual ~FrequencySelectiveFadingModel() {}

    virtual void GetNewFadingModelInterface(
        const NodeIdType& nodeId,
        shared_ptr<FrequencySelectiveFadingModelInterface>& fadingModelInterfacePtr) = 0;

};//FrequencySelectiveFadingModel//



// CannedFileFrequencySelectiveFadingModel is just a matrix of fade values by time/node pair/channel
// and subcarrier/subchannel.  Single channels are reciprocal (i.e. same fade in both directions).
// Multiple channels can be used when uplinks and downlinks are on different channels.
// The matrix is streamed in from disk in time order.


class CannedFileFrequencySelectiveFadingModel: public FrequencySelectiveFadingModel,
    public enable_shared_from_this<CannedFileFrequencySelectiveFadingModel> {
public:

    CannedFileFrequencySelectiveFadingModel(
        const ParameterDatabaseReader& theParameterDatabase,
        const InterfaceOrInstanceIdType& initPropagationModeInstanceId,
        const unsigned int baseChannelNumber,
        const unsigned int numberChannels);

    virtual void GetNewFadingModelInterface(
        const NodeIdType& nodeId,
        shared_ptr<FrequencySelectiveFadingModelInterface>& fadingModelInterfacePtr) override;

private:
    void SetTime(
        const unsigned int channelNumber,
        const TimeType& timeForData);

    const vector<double>& GetSubcarrierOrSubchannelDbFadingVector(
        const unsigned int channelIndex,
        const NodeIdType& nodeId1,
        const NodeIdType& nodeId2) const;

    const vector<double>& GetSubcarrierOrSubchannelDbFadingVector(
        const unsigned int channelIndex,
        const NodeIdType& nodeId1,
        const unsigned int node1CellSectorIndex,
        const NodeIdType& nodeId2,
        const unsigned int node2CellSectorIndex) const;

    //----------------------------------

    InterfaceOrInstanceIdType propagationModelInstanceId;

    struct LinkKeyType {

        NodeIdType nodeId1;
        unsigned int node1CellSector;
        NodeIdType nodeId2;
        unsigned int node2CellSector;

        LinkKeyType(
            const NodeIdType& initNodeId1,
            const unsigned int initNode1CellSector,
            const NodeIdType& initNodeId2,
            const unsigned int initNode2CellSector)
        :
            nodeId1(initNodeId1),
            node1CellSector(initNode1CellSector),
            nodeId2(initNodeId2),
            node2CellSector(initNode2CellSector)
        {
            if (nodeId1 > nodeId2) {
                std::swap(nodeId1, nodeId2);
                std::swap(node1CellSector, node2CellSector);
            }//if//
        }

        bool operator<(const LinkKeyType& right) const
        {
            assert(nodeId1 <= nodeId2);

            if (nodeId1 < right.nodeId1) {
                return true;
            }

            if (nodeId1 > right.nodeId1) {
                return false;
            }

            if (node1CellSector < right.node1CellSector) {
                return true;
            }

            if (node1CellSector > right.node1CellSector) {
                return false;
            }

            if (nodeId2 < right.nodeId2) {
                return true;
            }

            if (nodeId2 > right.nodeId2) {
                return false;
            }

            return (node2CellSector < right.node2CellSector);

        }//"<"

    };//LinkKeyType//

    //----------------------------------

    struct LinkInfoType {

        vector<double> fadingValuesDb;

    };//LinkInfoType//


    // FileDataDescriptorType holds info about the next set of data.

    struct FileDataDescriptorType {
        bool hasBeenSet;
        TimeType timeForData;
        NodeIdType nodeId1;
        unsigned int node1CellSectorIndex;
        NodeIdType nodeId2;
        unsigned int node2CellSectorIndex;
        unsigned int numberSubcarriersOrSubchannels;
        FileDataDescriptorType() : hasBeenSet(false) { }
    };

    struct ChannelDataType {

        string dataFileName;
        std::ifstream dataFile;

        // For error messages:

        unsigned int currentFileLineNumber;

        bool currentLineHasData;
        FileDataDescriptorType currentDataDescriptor;

        map<LinkKeyType, LinkInfoType> linkInfos;

        // For assert checks only.
        TimeType lastSetTimeCallTime;

        ChannelDataType() : currentFileLineNumber(0), currentLineHasData(false) {}
    };

    vector<shared_ptr<ChannelDataType> > channels;
    unsigned int baseChannelNumber;

    static
    void ReadALine(
        std::istream& aStream,
        unsigned int& currentFileLineNum,
        bool& success,
        string& aLine);

    static
    void ReadDataDescriptorLine(
        const string& dataFileName,
        istream& dataFile,
        unsigned int& currentFileLineNumber,
        FileDataDescriptorType& dataDescriptor,
        bool& success);

    static
    void ReadInFadingValuesForAllSubcarriersOrSubchannels(
        const string& dataFileName,
        istream& dataFile,
        unsigned int& currentFileLineNumber,
        const FileDataDescriptorType& dataDescriptor,
        LinkInfoType& linkInfo);


    friend class CannedFileFrequencySelectiveFadingModelInterface;

};//CannedFileFrequencySelectiveFadingModel//


inline
CannedFileFrequencySelectiveFadingModel::CannedFileFrequencySelectiveFadingModel(
    const ParameterDatabaseReader& theParameterDatabase,
    const InterfaceOrInstanceIdType& initPropagationModelInstanceId,
    const unsigned int initBaseChannelNumber,
    const unsigned int numberChannels)
    :
    propagationModelInstanceId(initPropagationModelInstanceId),
    channels(numberChannels),
    baseChannelNumber(initBaseChannelNumber)
{
    for(unsigned int i = 0; (i < numberChannels); i++) {
        channels[i].reset(new ChannelDataType());
        ChannelDataType& channel = *channels[i];

        if ((numberChannels == 1) &&
            theParameterDatabase.ParameterExists(
                "freqselective-channel-file-name",
                propagationModelInstanceId)) {

            channel.dataFileName = theParameterDatabase.ReadString(
                "freqselective-channel-file-name",
                propagationModelInstanceId);
        }
        else {
            const string parameterName =
                ("freqselective-channel-" + ConvertToString(baseChannelNumber + i) + "-file-name");

            channel.dataFileName =
                theParameterDatabase.ReadString(parameterName, propagationModelInstanceId);
        }//if//

        channel.dataFile.open(channel.dataFileName.c_str());
        if (channel.dataFile.fail()) {
            cerr << "Error in Frequency Selective Fading Model: Could not open file: " << endl;
            cerr << "     " << channel.dataFileName << endl;
            exit(1);
        }//if//

        (*this).SetTime((baseChannelNumber + i), ZERO_TIME);

    }//for//

}//CannedFileFrequencySelectiveFadingModel()//


inline
void CannedFileFrequencySelectiveFadingModel::ReadALine(
    std::istream& aStream,
    unsigned int& currentFileLineNum,
    bool& success,
    string& aLine)
{
    assert(!aStream.eof());

    success = false;
    aLine.clear();

    do {
        if (aStream.eof()) {
            return;
        }//if//

        getline(aStream, aLine);
        currentFileLineNum++;

        if (aStream.fail()) {
            return;
        }//if//
    } while (IsAConfigFileCommentLine(aLine));

    DeleteTrailingSpaces(aLine);
    success = true;

}//ReadALine//



inline
void CannedFileFrequencySelectiveFadingModel::ReadDataDescriptorLine(
    const string& dataFileName,
    istream& dataFile,
    unsigned int& currentFileLineNumber,
    FileDataDescriptorType& dataDescriptor,
    bool& success)
{
    dataDescriptor.hasBeenSet = false;
    success = false;

    string aLine;
    ReadALine(dataFile, currentFileLineNumber, success, aLine);
    if (!success) {
        return;
    }//if//

    ConvertStringToLowerCase(aLine);

    istringstream lineStream(aLine);

    string timeForDataString;
    lineStream >> timeForDataString;

    ConvertStringToTime(timeForDataString, dataDescriptor.timeForData, success);

    lineStream >> dataDescriptor.nodeId1;
    lineStream >> dataDescriptor.node1CellSectorIndex;
    lineStream >> dataDescriptor.nodeId2;
    lineStream >> dataDescriptor.node2CellSectorIndex;
    lineStream >> dataDescriptor.numberSubcarriersOrSubchannels;

    if ((!success) || (lineStream.fail())) {
        cerr << "Error in Frequency Selective Fading Data file: "
             << "\"" << dataFileName << "\"" << " at line #:"
             << currentFileLineNumber << ":" << endl;
        cerr << "     " << aLine << endl;
        exit(1);
    }//if//

    dataDescriptor.hasBeenSet = true;

}//ReadDataDescriptorLine//


inline
void CannedFileFrequencySelectiveFadingModel::ReadInFadingValuesForAllSubcarriersOrSubchannels(
    const string& dataFileName,
    istream& dataFile,
    unsigned int& currentFileLineNumber,
    const FileDataDescriptorType& dataDescriptor,
    LinkInfoType& linkInfo)
{
    linkInfo.fadingValuesDb.resize(dataDescriptor.numberSubcarriersOrSubchannels);

    unsigned int subcarrierOrSubchannelIndex = 0;

    while (subcarrierOrSubchannelIndex < dataDescriptor.numberSubcarriersOrSubchannels) {

        bool success = (!dataFile.eof());

        string aLine;

        if (success) {
            ReadALine(dataFile, currentFileLineNumber, success, aLine);
        }//if//

        if (!success) {
            cerr << "Error: Frequency Selective Fading Matrix Model: Reading at line:" << endl;
            cerr << currentFileLineNumber << ": " << aLine << endl;
            exit(1);
        }//if//

        istringstream lineStream(aLine);

        while ((!lineStream.eof()) &&
               (subcarrierOrSubchannelIndex < dataDescriptor.numberSubcarriersOrSubchannels)) {

            // No trailing spaces => (!eof => something to read).

            lineStream >> linkInfo.fadingValuesDb[subcarrierOrSubchannelIndex];

            subcarrierOrSubchannelIndex++;

            if (((lineStream.fail()) && (!lineStream.eof())) ||
                ((subcarrierOrSubchannelIndex == dataDescriptor.numberSubcarriersOrSubchannels) &&
                 (!lineStream.eof()))) {

                cerr << "Error: Frequency Selective Fading Matrix Model: Bad Data (or wrong amount) in line:" << endl;
                cerr << currentFileLineNumber << ": " << aLine << endl;
                exit(1);
            }//if//
        }//while//
    }//while//

}//ReadInFadingValuesForAllSubcarriersOrSubchannels//



inline
void CannedFileFrequencySelectiveFadingModel::SetTime(
    const unsigned int channelNumber,
    const TimeType& timeForData)
{
    ChannelDataType& channel = *channels.at(channelNumber - baseChannelNumber);

    if (channel.dataFile.eof()) {
        // No more data to read.
        return;
    }//if//

    if (!channel.currentDataDescriptor.hasBeenSet) {
        bool success;
        (*this).ReadDataDescriptorLine(
            channel.dataFileName,
            channel.dataFile,
            channel.currentFileLineNumber,
            channel.currentDataDescriptor,
            success);

        if (!success) {
            cerr << "Error reading Freqency Selective Fading Data File: " << channel.dataFileName << endl;
            exit(1);
        }//if//
    }//if//

    assert(channel.lastSetTimeCallTime <= timeForData);
    channel.lastSetTimeCallTime = timeForData;

    // Read in data until next data set will be in the future.

    while (channel.currentDataDescriptor.timeForData <= timeForData) {

        const LinkKeyType linkKey(
            channel.currentDataDescriptor.nodeId1,
            channel.currentDataDescriptor.node1CellSectorIndex,
            channel.currentDataDescriptor.nodeId2,
            channel.currentDataDescriptor.node2CellSectorIndex);

        LinkInfoType& linkInfo = channel.linkInfos[linkKey];

        (*this).ReadInFadingValuesForAllSubcarriersOrSubchannels(
            channel.dataFileName,
            channel.dataFile,
            channel.currentFileLineNumber,
            channel.currentDataDescriptor,
            linkInfo);

        bool success;

        (*this).ReadDataDescriptorLine(
            channel.dataFileName,
            channel.dataFile,
            channel.currentFileLineNumber,
            channel.currentDataDescriptor,
            success);

        if (!success) {
            if (!channel.dataFile.eof()) {
                cerr << "Error reading Freqency Selective Fading Data File: " << channel.dataFileName << endl;
                exit(1);
            }//if//

            break;
        }//if//
    }//while//

}//SetTime//


inline
const vector<double>& CannedFileFrequencySelectiveFadingModel::GetSubcarrierOrSubchannelDbFadingVector(
    const unsigned int channelIndex,
    const NodeIdType& nodeId1,
    const NodeIdType& nodeId2) const
{
    typedef map<LinkKeyType, LinkInfoType>::const_iterator IterType;

    const ChannelDataType& channelData = *channels.at(channelIndex - baseChannelNumber);

    const IterType iter = channelData.linkInfos.find(LinkKeyType(nodeId1, 0, nodeId2, 0));
    assert(iter != channelData.linkInfos.end());

    const LinkInfoType& linkInfo = iter->second;

    return (linkInfo.fadingValuesDb);

}//GetSubcarrierOrSubchannelDbFadingVector//


inline
const vector<double>& CannedFileFrequencySelectiveFadingModel::GetSubcarrierOrSubchannelDbFadingVector(
    const unsigned int channelIndex,
    const NodeIdType& nodeId1,
    const unsigned int node1CellSectorIndex,
    const NodeIdType& nodeId2,
    const unsigned int node2CellSectorIndex) const
{
    typedef map<LinkKeyType, LinkInfoType>::const_iterator IterType;

    const ChannelDataType& channelData = *channels.at(channelIndex - baseChannelNumber);

    const IterType iter =
        channelData.linkInfos.find(
            LinkKeyType(nodeId1, node1CellSectorIndex, nodeId2, node2CellSectorIndex));

    assert(iter != channelData.linkInfos.end());

    const LinkInfoType& linkInfo = iter->second;

    return (linkInfo.fadingValuesDb);

}//GetSubcarrierOrSubchannelDbFadingVector//





class CannedFileFrequencySelectiveFadingModelInterface : public FrequencySelectiveFadingModelInterface {
public:
    ~CannedFileFrequencySelectiveFadingModelInterface() { }

    virtual void SetTime(
        const unsigned int channelNumber,
        const TimeType& timeForData) override
    {
        modelPtr->SetTime(channelNumber, timeForData);
    }


    virtual const vector<double>& GetSubcarrierOrSubchannelDbFadingVector(
        const unsigned int channelNumber,
        const NodeIdType& sourceNodeId) const override
    {
        return (
            modelPtr->GetSubcarrierOrSubchannelDbFadingVector(
                channelNumber,
                sourceNodeId,
                nodeId));
    }


    virtual const vector<double>& GetSubcarrierOrSubchannelDbFadingVector(
        const unsigned int channelNumber,
        const unsigned int cellSectorIndex,
        const NodeIdType& sourceNodeId,
        const unsigned int sourceCellSectorIndex) const override
    {
        return (
            modelPtr->GetSubcarrierOrSubchannelDbFadingVector(
                channelNumber,
                sourceNodeId,
                sourceCellSectorIndex,
                nodeId,
                cellSectorIndex));
    }

private:
    friend class CannedFileFrequencySelectiveFadingModel;

    CannedFileFrequencySelectiveFadingModelInterface(
        const shared_ptr<CannedFileFrequencySelectiveFadingModel>& initMatrixModelPtr,
        const NodeIdType& initNodeId);

    shared_ptr<CannedFileFrequencySelectiveFadingModel> modelPtr;

    NodeIdType nodeId;

};//CannedFileFrequencySelectiveFadingModelInterface//



inline
CannedFileFrequencySelectiveFadingModelInterface::CannedFileFrequencySelectiveFadingModelInterface(
    const shared_ptr<CannedFileFrequencySelectiveFadingModel>& initModelPtr,
    const NodeIdType& initNodeId)
    :
    modelPtr(initModelPtr),
    nodeId(initNodeId)
{
}



//--------------------------------------------------------------------------------------------------

inline
void CannedFileFrequencySelectiveFadingModel::GetNewFadingModelInterface(
    const NodeIdType& nodeId,
    shared_ptr<FrequencySelectiveFadingModelInterface>& fadingModelInterfacePtr)
{
    fadingModelInterfacePtr.reset(
        new CannedFileFrequencySelectiveFadingModelInterface(shared_from_this(), nodeId));
}


}//namespace//


#endif
