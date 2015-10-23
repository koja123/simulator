// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_MIMO_CHANNEL_H
#define SCENSIM_MIMO_CHANNEL_H

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <complex>
#include <array>
#include "boost/numeric/ublas/matrix.hpp"

#include "scensim_time.h"
#include "scensim_nodeid.h"
#include "scensim_support.h"
#include "scensim_parmio.h"
#include "tgn_mimo_ch_model.h"
#include "scensim_mobility.h"

namespace ScenSim {

using std::cerr;
using std::endl;
using std::string;
using std::auto_ptr;
using std::shared_ptr;
using std::enable_shared_from_this;
using std::map;
using std::vector;
using std::complex;
using std::array;
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
using ScenSim::FastIntToDataMap;
using namespace boost::numeric;
using TgnMimoChModel::sqrt2;


//--------------------------------------------------------------------------------------------------
//
// Localized (to node) interface for for future caching and thread issues (sharing) to global
// models such as a pathloss matrix.
//

typedef ublas::matrix<complex<double> > MimoChannelMatrixType;

const unsigned int MaxMimoMatrixRowsOrColumns = 8;


class MimoChannelModelInterface {
public:
    virtual ~MimoChannelModelInterface() { }

    virtual void SetTime(const TimeType& timeForData) = 0;

    virtual unsigned int GetNumberOfAntennas() const = 0;
    virtual unsigned int GetNumberOfAntennasFor(const NodeIdType& otherNodeId) const = 0;

    // No sectors version:

    virtual void GetChannelMatrix(
        const unsigned int channelNumber,
        const NodeIdType& sourceNodeId,
        const unsigned int subcarrierIndex,
        MimoChannelMatrixType& mimoChannelMatrix) = 0;

    // Has sectors version:

    virtual void GetChannelMatrix(
        const unsigned int channelNumber,
        const NodeIdType& sourceNodeId,
        const unsigned int cellSectorIndex,
        const unsigned int subcarrierIndex,
        MimoChannelMatrixType& mimoChannelMatrix)  = 0;

    // Support for no copy method version (reference to internal structure).
    // (Likely use: does not have to support reverse links (transposed matrices)).
    // Logically "const" but not physically const as the channel matrix may
    // be generated "ondemand" (as opposed to during SetTime()).

    virtual const MimoChannelMatrixType& GetChannelMatrix(
        const unsigned int channelNumber,
        const NodeIdType& sourceNodeId,
        const unsigned int subcarrierIndex) = 0;

    virtual const MimoChannelMatrixType& GetChannelMatrix(
        const unsigned int channelNumber,
        const NodeIdType& sourceNodeId,
        const unsigned int cellSectorIndex,
        const unsigned int subcarrierIndex) = 0;

};//MimoChannelModelInterface//



class MimoChannelModel {
public:
    virtual ~MimoChannelModel() {}

    virtual void CreateNewMimoChannelModelInterface(
        const ParameterDatabaseReader& theParameterDatabase,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        shared_ptr<MimoChannelModelInterface>& interfacePtr) = 0;

    // For multi-sector basestations and APs.

    virtual void CreateNewMimoChannelModelInterfaceForBasestation(
        const ParameterDatabaseReader& theParameterDatabase,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int cellSectorId,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        shared_ptr<MimoChannelModelInterface>& interfacePtr) = 0;

};//MimoChannelModel//




class CannedFileMimoChannelModel : public MimoChannelModel,
    public enable_shared_from_this<CannedFileMimoChannelModel> {
public:

    CannedFileMimoChannelModel(
        const ParameterDatabaseReader& theParameterDatabase,
        const InterfaceOrInstanceIdType& propagationModelInstanceId,
        const unsigned int initBaseChannelNumber,
        const unsigned int numberChannels);

    virtual void CreateNewMimoChannelModelInterface(
            const ParameterDatabaseReader& theParameterDatabase,
            const NodeIdType& nodeId,
            const InterfaceIdType& interfaceId,
            const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
            shared_ptr<MimoChannelModelInterface>& interfacePtr) override;

    virtual void CreateNewMimoChannelModelInterfaceForBasestation(
            const ParameterDatabaseReader& theParameterDatabase,
            const NodeIdType& nodeId,
            const InterfaceIdType& interfaceId,
            const unsigned int cellSectorId,
            const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
            shared_ptr<MimoChannelModelInterface>& interfacePtr) override;

private:

    void SetTime(
        const unsigned int channelNumber,
        const TimeType& timeForData);

    void SetTime(const TimeType& timeForData);

    unsigned int GetNumberOfAntennasFor(const NodeIdType& nodeId) const;

    void GetChannelMatrix(
        const unsigned int channelIndex,
        const NodeIdType& nodeId1,
        const unsigned int nodeId1CellSectorId,
        const NodeIdType& nodeId2,
        const unsigned int nodeId2CellSectorId,
        const unsigned int subcarrierIndex,
        MimoChannelMatrixType& mimoChannelMatrix) const;

    // Note: Const reference to internal structure for efficiency.

    const MimoChannelMatrixType& GetChannelMatrix(
        const unsigned int channelIndex,
        const NodeIdType& nodeId1,
        const unsigned int nodeId1CellSectorId,
        const NodeIdType& nodeId2,
        const unsigned int nodeId2CellSectorId,
        const unsigned int subcarrierIndex) const;

    InterfaceOrInstanceIdType propagationModelInstanceId;

    struct FileDataDescriptorType {
        bool hasBeenSet;
        TimeType timeForData;
        NodeIdType nodeId1;
        unsigned int node1CellSectorIndex;
        unsigned int node1NumberAntennas;
        NodeIdType nodeId2;
        unsigned int node2CellSectorIndex;
        unsigned int node2NumberAntennas;
        unsigned int numberSubcarriers;

        FileDataDescriptorType() : hasBeenSet(false) { }

    };//FileDataDescriptorType//

    //----------------------------------

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


    struct MimoLinkInfoType {
        // Link direction is defined as nodeId1 --> nodeId2 where nodeId1 < nodeId2
        // Data is "reversed" if data is from a higher node ID to a lower node ID.
        bool linkDataDirectionIsReversed;
        vector<MimoChannelMatrixType> mimoChannelMatrices;

    };//MimoLinkInfoType//


    struct ChannelDataType {

        string mimoDataFileName;
        std::ifstream mimoDataFile;

        // For error messages:

        unsigned int currentFileLineNumber;

        FileDataDescriptorType currentFileDataDescriptor;

        map<LinkKeyType, MimoLinkInfoType> linkInfos;

        TimeType loopStartTime;

        // For assert checks only.
        TimeType lastSetTimeCallTime;

        ChannelDataType() :
            currentFileLineNumber(0),
            lastSetTimeCallTime(ZERO_TIME),
            loopStartTime(ZERO_TIME) {}

    };//ChannelDataType//

    FastIntToDataMap<NodeIdType, unsigned int> numberOfAntennasMap;

    vector<shared_ptr<ChannelDataType> > channels;
    unsigned int baseChannelNumber;

    bool dataLoopingIsEnabled;

    //----------------------------------------------------------------------------------------------

    class ModelInterface: public MimoChannelModelInterface {
    public:

        virtual void SetTime(const TimeType& timeForData) override
        {
            channelModelPtr->SetTime(timeForData);
        }

        virtual unsigned int GetNumberOfAntennas() const override
        {
            return(channelModelPtr->GetNumberOfAntennasFor(nodeId));
        }

        virtual unsigned int GetNumberOfAntennasFor(const NodeIdType& otherNodeId) const override
        {
            return(channelModelPtr->GetNumberOfAntennasFor(otherNodeId));
        }

        virtual void GetChannelMatrix(
            const unsigned int channelNumber,
            const NodeIdType& sourceNodeId,
            const unsigned int subcarrierIndex,
            MimoChannelMatrixType& mimoChannelMatrix) override
        {
            channelModelPtr->GetChannelMatrix(
                channelNumber,
                sourceNodeId,
                0,
                nodeId,
                0,
                subcarrierIndex,
                mimoChannelMatrix);
        }


        virtual void GetChannelMatrix(
            const unsigned int channelNumber,
            const NodeIdType& otherNodeId,
            const unsigned int otherNodeCellSectorId,
            const unsigned int subcarrierIndex,
            MimoChannelMatrixType& mimoChannelMatrix) override
        {
            channelModelPtr->GetChannelMatrix(
                channelNumber,
                otherNodeId,
                otherNodeCellSectorId,
                nodeId,
                cellSectorId,
                subcarrierIndex,
                mimoChannelMatrix);
        }

        virtual const MimoChannelMatrixType& GetChannelMatrix(
            const unsigned int channelNumber,
            const NodeIdType& sourceNodeId,
            const unsigned int subcarrierIndex) override
        {
            return (
                channelModelPtr->GetChannelMatrix(
                    channelNumber, sourceNodeId, 0, nodeId, 0, subcarrierIndex));
        }

        virtual const MimoChannelMatrixType& GetChannelMatrix(
            const unsigned int channelNumber,
            const NodeIdType& sourceNodeId,
            const unsigned int sourceCellSectorId,
            const unsigned int subcarrierIndex) override
        {
            return (
                channelModelPtr->GetChannelMatrix(
                    channelNumber, sourceNodeId, sourceCellSectorId,
                    nodeId, cellSectorId, subcarrierIndex));
        }


    private:

        ModelInterface(
            const shared_ptr<CannedFileMimoChannelModel>& channelModelPtr,
            const NodeIdType& nodeId,
            const unsigned int cellSectorId);

        shared_ptr<CannedFileMimoChannelModel> channelModelPtr;

        NodeIdType nodeId;
        unsigned int cellSectorId;

        friend class CannedFileMimoChannelModel;

    };//ModelInterface//


    //----------------------------------------------------------------------------------------------

    void SaveNumberOfAntennaValues(const FileDataDescriptorType& dataDescriptor);

    static
    void ReadALine(
        std::istream& aStream,
        unsigned int& currentFileLineNum,
        bool& success,
        string& aLine);

    static
    void ReadDataDescriptorLine(
        const string& mimoDataFileName,
        istream& mimoDataFile,
        unsigned int& currentFileLineNumber,
        FileDataDescriptorType& dataDescriptor,
        bool& success);

    static
    void ReadMimoChannelMatricesForAllSubcarriers(
        const string& mimoDataFileName,
        istream& mimoDataFile,
        unsigned int& currentFileLineNumber,
        const FileDataDescriptorType& dataDescriptor,
        MimoLinkInfoType& linkInfo);

};//CannedFileMimoChannelModel//


inline
CannedFileMimoChannelModel::CannedFileMimoChannelModel(
    const ParameterDatabaseReader& theParameterDatabase,
    const InterfaceOrInstanceIdType& initPropagationModelInstanceId,
    const unsigned int initBaseChannelNumber,
    const unsigned int numberChannels)
    :
    propagationModelInstanceId(initPropagationModelInstanceId),
    channels(numberChannels),
    baseChannelNumber(initBaseChannelNumber),
    dataLoopingIsEnabled(false)
{
    for(unsigned int i = 0; (i < numberChannels); i++) {
        channels[i].reset(new ChannelDataType());
        ChannelDataType& channel = *channels[i];

        if ((numberChannels == 1) &&
            theParameterDatabase.ParameterExists(
                "mimo-channel-file-name",
                propagationModelInstanceId)) {

            channel.mimoDataFileName = theParameterDatabase.ReadString(
                "mimo-channel-file-name",
                propagationModelInstanceId);
        }
        else {
            const string parameterName =
                ("mimo-channel-" + ConvertToString(baseChannelNumber + i) + "-file-name");

            channel.mimoDataFileName =
                theParameterDatabase.ReadString(parameterName, propagationModelInstanceId);
        }//if//

        channel.mimoDataFile.open(channel.mimoDataFileName.c_str());

        if (channel.mimoDataFile.fail()) {
            cerr << "Error in Mimo Channel Model: Could not open "
                 << ("mimo-channel-model-file-prefix") << " file:" << endl;
            cerr << "     " << channel.mimoDataFileName << endl;
            exit(1);
        }//if//

        (*this).SetTime((baseChannelNumber + i), ZERO_TIME);
    }//for//

    if (theParameterDatabase.ParameterExists(
        "mimo-channel-model-enable-file-looping", propagationModelInstanceId)) {

        dataLoopingIsEnabled =
            theParameterDatabase.ReadBool(
                "mimo-channel-model-enable-file-looping", propagationModelInstanceId);

    }//if//



}//CannedFileMimoChannelModel()//


inline
unsigned int CannedFileMimoChannelModel::GetNumberOfAntennasFor(const NodeIdType& nodeId) const
{
    if (!numberOfAntennasMap.IsMapped(nodeId)) {
        cerr << "Error in CannedFileMimoChannelModel: No data for Node ID = " << nodeId << "." << endl;
        exit(1);
    }//if//

    return (numberOfAntennasMap.GetValue(nodeId));
}



inline
void CannedFileMimoChannelModel::ReadALine(
    std::istream& aStream,
    unsigned int& currentFileLineNum,
    bool& success,
    string& aLine)
{
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
void CannedFileMimoChannelModel::ReadDataDescriptorLine(
    const string& mimoDataFileName,
    istream& mimoDataFile,
    unsigned int& currentFileLineNumber,
    FileDataDescriptorType& dataDescriptor,
    bool& success)
{
    dataDescriptor.hasBeenSet = false;
    success = false;

    string aLine;
    ReadALine(mimoDataFile, currentFileLineNumber, success, aLine);
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
    lineStream >> dataDescriptor.node1NumberAntennas;
    lineStream >> dataDescriptor.nodeId2;
    lineStream >> dataDescriptor.node2CellSectorIndex;
    lineStream >> dataDescriptor.node2NumberAntennas;
    lineStream >> dataDescriptor.numberSubcarriers;

    if ((!success) || (lineStream.fail())) {
        cerr << "Error in Mimo Data file: "
             << "\"" << mimoDataFileName << "\"" << " at line # "
             << currentFileLineNumber << " " << endl;
        cerr << "     " << aLine << endl;
        exit(1);
    }//if//

    dataDescriptor.hasBeenSet = true;

    assert(success);

}//ReadDataDescriptorLine//



inline
void ReadInNextNonSpaceChar(
    istream& inputStream,
    bool& success,
    char& aChar)
{
    success = false;

    aChar = ' ';
    while ((!inputStream.eof()) && (aChar == ' ')) {
        inputStream >> aChar;
        assert((!inputStream.fail()) || (inputStream.eof()));
    }//while//

    success = (aChar != ' ');

}//ReadInNextNonSpaceChar//



inline
void ReadComplexMatrix(
    istream& inputStream,
    bool& success,
    MimoChannelMatrixType& matrix)
{
    assert((matrix.size1() != 0) && (matrix.size2() != 0) && "Matrix must be preallocated");

    success = false;

    bool readWasSuccessful;
    char aChar;
    ReadInNextNonSpaceChar(inputStream, readWasSuccessful, aChar);

    if ((!readWasSuccessful) || (aChar != '{')) {
        return;
    }//if//

    unsigned int i = 0;
    while(true) {

        ReadInNextNonSpaceChar(inputStream, readWasSuccessful, aChar);
        if ((!readWasSuccessful) || (aChar != '{')) {
            return;
        }//if//

        array<complex<double>, MaxMimoMatrixRowsOrColumns> rowVector;

        unsigned int j = 0;
        while (true) {

            if ((i >= matrix.size1()) || (j >= matrix.size2())) {
                return;
            }

            inputStream >> matrix(i, j);

            if (inputStream.fail()) {
                return;
            }//if//

            ReadInNextNonSpaceChar(inputStream, readWasSuccessful, aChar);
            if (!readWasSuccessful) {
                return;
            }//if//

            if (aChar == '}') {
                break;
            }//if//

            if (aChar != ',') {
                return;
            }//if//

            j++;

        }//while//

        if ((j+1) != matrix.size2()) {
           // Too few values in the row.
           return;
        }//if//

        ReadInNextNonSpaceChar(inputStream, readWasSuccessful, aChar);
        if (!readWasSuccessful) {
            return;
        }//if//

        if (aChar == '}') {
            break;
        }//if//

        if (aChar != ',') {
            return;
        }//if//

        i++;

    }//while//

    if ((i+1) != matrix.size1()) {
       // Too few rows.
       return;
    }//if//

    success = true;

}//ReadComplexMatrix//


inline
void CannedFileMimoChannelModel::ReadMimoChannelMatricesForAllSubcarriers(
    const string& mimoDataFileName,
    istream& mimoDataFile,
    unsigned int& currentFileLineNumber,
    const FileDataDescriptorType& dataDescriptor,
    MimoLinkInfoType& linkInfo)
{
    linkInfo.linkDataDirectionIsReversed = (dataDescriptor.nodeId1 > dataDescriptor.nodeId2);
    linkInfo.mimoChannelMatrices.resize(dataDescriptor.numberSubcarriers);

    for(unsigned int i = 0; (i < dataDescriptor.numberSubcarriers); i++) {

        MimoChannelMatrixType& mimoMatrix = linkInfo.mimoChannelMatrices[i];

        mimoMatrix.resize(
            dataDescriptor.node2NumberAntennas,
            dataDescriptor.node1NumberAntennas,
            false);

        string aLine;
        bool success;

        ReadALine(
            mimoDataFile,
            currentFileLineNumber,
            success,
            aLine);

        if (!success) {
            cerr << "Error in MIMO File: " << mimoDataFileName << " Not enough data for all subcarriers." << endl;
            cerr << "    Line# " << currentFileLineNumber << endl;
            exit(1);
        }//if//

        istringstream lineStream(aLine);

        ReadComplexMatrix(
            lineStream,
            success,
            mimoMatrix);

        if (!success) {
            cerr << "Error in MIMO File: " << mimoDataFileName << " Line # " << currentFileLineNumber << endl;
            cerr << "     " << aLine << endl;
            cerr << "   Bad format or Matrix dimensions do not agree with data header." << endl;
            exit(1);
        }//if//

    }//for//

}//ReadMimoChannelMatricesForAllSubcarriers//



inline
void CannedFileMimoChannelModel::SaveNumberOfAntennaValues(
    const FileDataDescriptorType& dataDescriptor)
{
    if (!numberOfAntennasMap.IsMapped(dataDescriptor.nodeId1)) {
        numberOfAntennasMap.InsertMapping(
            dataDescriptor.nodeId1, dataDescriptor.node1NumberAntennas);

    }
    else {
        assert(numberOfAntennasMap.GetValue(dataDescriptor.nodeId1) ==
               dataDescriptor.node1NumberAntennas);
    }//if//

    if (!numberOfAntennasMap.IsMapped(dataDescriptor.nodeId2)) {
        numberOfAntennasMap.InsertMapping(
            dataDescriptor.nodeId2, dataDescriptor.node2NumberAntennas);

    }
    else {
        assert(numberOfAntennasMap.GetValue(dataDescriptor.nodeId2) ==
               dataDescriptor.node2NumberAntennas);
    }//if//

}//SaveNumberOfAntennaValues//




inline
void CannedFileMimoChannelModel::SetTime(
    const unsigned int channelNumber,
    const TimeType& timeForData)
{
    ChannelDataType& channel = *channels.at(channelNumber - baseChannelNumber);

    if (channel.mimoDataFile.eof()) {
        // No more data to read.
        return;
    }//if//

    if (!channel.currentFileDataDescriptor.hasBeenSet) {
        bool success;
        (*this).ReadDataDescriptorLine(
            channel.mimoDataFileName,
            channel.mimoDataFile,
            channel.currentFileLineNumber,
            channel.currentFileDataDescriptor,
            success);

        if (!success) {
            cerr << "Error reading Mimo Data File: " << channel.mimoDataFileName << endl;
            exit(1);
        }//if//

        (*this).SaveNumberOfAntennaValues(channel.currentFileDataDescriptor);

    }//if//

    assert(channel.lastSetTimeCallTime <= timeForData);
    channel.lastSetTimeCallTime = timeForData;

    // Read in data until next data set will be in the future.

    while ((channel.loopStartTime + channel.currentFileDataDescriptor.timeForData) <= timeForData) {

        const LinkKeyType linkKey(
            channel.currentFileDataDescriptor.nodeId1,
            channel.currentFileDataDescriptor.node1CellSectorIndex,
            channel.currentFileDataDescriptor.nodeId2,
            channel.currentFileDataDescriptor.node2CellSectorIndex);

        MimoLinkInfoType& linkInfo = channel.linkInfos[linkKey];

        (*this).ReadMimoChannelMatricesForAllSubcarriers(
            channel.mimoDataFileName,
            channel.mimoDataFile,
            channel.currentFileLineNumber,
            channel.currentFileDataDescriptor,
            linkInfo);

        bool success;

        (*this).ReadDataDescriptorLine(
            channel.mimoDataFileName,
            channel.mimoDataFile,
            channel.currentFileLineNumber,
            channel.currentFileDataDescriptor,
            success);

        if (!success) {
            if (!channel.mimoDataFile.eof()) {
                cerr << "Error reading Mimo Data File: " << channel.mimoDataFileName << endl;
                exit(1);
            }//if//

            if (dataLoopingIsEnabled) {
                channel.mimoDataFile.close();
                channel.mimoDataFile.open(channel.mimoDataFileName.c_str());

                channel.loopStartTime += channel.currentFileDataDescriptor.timeForData;

                (*this).ReadDataDescriptorLine(
                    channel.mimoDataFileName,
                    channel.mimoDataFile,
                    channel.currentFileLineNumber,
                    channel.currentFileDataDescriptor,
                    success);

                if (!success) {
                    cerr << "Error reading Mimo Data File: " << channel.mimoDataFileName << endl;
                    exit(1);
                }//if//
            }
            else {
                break;

            }//if//
        }//if//

        (*this).SaveNumberOfAntennaValues(channel.currentFileDataDescriptor);

    }//while//

}//SetTime//


inline
void CannedFileMimoChannelModel::SetTime(const TimeType& timeForData)
{
    for(unsigned int i = 0; (i < channels.size()); i++) {
        (*this).SetTime((baseChannelNumber + i), timeForData);
    }//for//

}//SetTime//


inline
const MimoChannelMatrixType& CannedFileMimoChannelModel::GetChannelMatrix(
    const unsigned int channelNumber,
    const NodeIdType& nodeId1,
    const unsigned int node1CellSectorId,
    const NodeIdType& nodeId2,
    const unsigned int node2CellSectorId,
    const unsigned int subcarrierIndex) const
{
    assert(nodeId1 != nodeId2);

    const ChannelDataType& channelData = *channels.at(channelNumber - baseChannelNumber);

    const bool linkIsReversed = (nodeId1 > nodeId2);

    LinkKeyType linkKey(
        nodeId1,
        node1CellSectorId,
        nodeId2,
        node2CellSectorId);

    typedef map<LinkKeyType, MimoLinkInfoType>::const_iterator IterType;

    IterType iter = channelData.linkInfos.find(linkKey);

    if (iter == channelData.linkInfos.end()) {
        cerr << "Error in file MIMO Channel Model: No data for link Node1 = "
             << nodeId1 << " Sector = " << node1CellSectorId << " to Node2 = "
             << nodeId2 << " Sector = " << node2CellSectorId << endl;
        exit(1);
    }//if//

    const MimoLinkInfoType& linkInfo = iter->second;

    if (linkIsReversed != linkInfo.linkDataDirectionIsReversed) {
        cerr << "Error in file MIMO Channel Model: Link data is the wrong direction for Node1 = "
             << nodeId1 << " Sector = " << node1CellSectorId << " to Node2 = "
             << nodeId2 << " Sector = " << node2CellSectorId << endl;
        exit(1);
    }//if//

    if (subcarrierIndex >= linkInfo.mimoChannelMatrices.size()) {
        cerr << "Error in file MIMO Channel Model: Incorrect subcarrier #: "
             << subcarrierIndex << " for link Node1 = "
             << nodeId1 << " Sector = " << node1CellSectorId << " to Node2 = "
             << nodeId2 << " Sector = " << node2CellSectorId << endl;
        exit(1);
    }//if//

    return (linkInfo.mimoChannelMatrices.at(subcarrierIndex));

}//GetChannelMatrix//



inline
void CannedFileMimoChannelModel::GetChannelMatrix(
    const unsigned int channelNumber,
    const NodeIdType& nodeId1,
    const unsigned int node1CellSectorId,
    const NodeIdType& nodeId2,
    const unsigned int node2CellSectorId,
    const unsigned int subcarrierIndex,
    MimoChannelMatrixType& mimoChannelMatrix) const
{
    assert(nodeId1 != nodeId2);

    const ChannelDataType& channelData = *channels.at(channelNumber - baseChannelNumber);

    const bool linkIsReversed = (nodeId1 > nodeId2);

    LinkKeyType linkKey(
        nodeId1,
        node1CellSectorId,
        nodeId2,
        node2CellSectorId);

    typedef map<LinkKeyType, MimoLinkInfoType>::const_iterator IterType;

    IterType iter = channelData.linkInfos.find(linkKey);

    if (iter == channelData.linkInfos.end()) {
        cerr << "Error in file MIMO Channel Model: No data for link Node1 = "
             << nodeId1 << "Sector = " << node1CellSectorId << " to Node2 = "
             << nodeId2 << "Sector = " << node2CellSectorId << endl;
        exit(1);
    }//if//

    const MimoLinkInfoType& linkInfo = iter->second;
    const MimoChannelMatrixType& theMatrix = linkInfo.mimoChannelMatrices.at(subcarrierIndex);


    if (linkIsReversed == linkInfo.linkDataDirectionIsReversed) {
        mimoChannelMatrix.resize(theMatrix.size1(), theMatrix.size2(), false);
        mimoChannelMatrix = theMatrix;
    }
    else {
        // Transpose stored matrix

        mimoChannelMatrix.resize(theMatrix.size2(), theMatrix.size1(), false);

        for(unsigned int i = 0; (i < mimoChannelMatrix.size1()); i++) {
            for(unsigned int j = 0; (j < mimoChannelMatrix.size2()); j++) {
                mimoChannelMatrix(i, j) = theMatrix(j, i);
            }//for//
        }//for//
    }//if//

}//GetChannelMatrix//


//--------------------------------------------------------------------------------------------------

inline
CannedFileMimoChannelModel::ModelInterface::ModelInterface(
    const shared_ptr<CannedFileMimoChannelModel>& initChannelModelPtr,
    const NodeIdType& initNodeId,
    const unsigned int initCellSectorId)
    :
    channelModelPtr(initChannelModelPtr),
    nodeId(initNodeId),
    cellSectorId(initCellSectorId)
{
}


inline
void CannedFileMimoChannelModel::CreateNewMimoChannelModelInterface(
        const ParameterDatabaseReader& theParameterDatabase,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        shared_ptr<MimoChannelModelInterface>& interfacePtr)
{
    interfacePtr.reset(new ModelInterface(shared_from_this(), nodeId, 0));
}


inline
void CannedFileMimoChannelModel::CreateNewMimoChannelModelInterfaceForBasestation(
    const ParameterDatabaseReader& theParameterDatabase,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId,
    const unsigned int cellSectorId,
    const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
    shared_ptr<MimoChannelModelInterface>& interfacePtr)
{
    interfacePtr.reset(new ModelInterface(shared_from_this(), nodeId, cellSectorId));
}


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


class OnDemandTgnMimoChannelModel : public MimoChannelModel,
    public enable_shared_from_this<OnDemandTgnMimoChannelModel> {
public:

    OnDemandTgnMimoChannelModel(
        const ParameterDatabaseReader& theParameterDatabase,
        const InterfaceOrInstanceIdType& propagationModelInstanceId,
        const unsigned int initBaseChannelNumber,
        const unsigned int numberChannels,
        const vector<double>& channelCarrierFrequencyListMhz,
        const vector<double>& channelBandwidthListMhz,
        const unsigned int numberSubcarriersPerChannel,
        const RandomNumberGeneratorSeedType& runSeed);

    virtual void CreateNewMimoChannelModelInterface(
        const ParameterDatabaseReader& theParameterDatabase,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        shared_ptr<MimoChannelModelInterface>& interfacePtr) override;

    virtual void CreateNewMimoChannelModelInterfaceForBasestation(
        const ParameterDatabaseReader& theParameterDatabase,
        const NodeIdType& nodeId,
        const InterfaceIdType& interfaceId,
        const unsigned int cellSectorId,
        const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
        shared_ptr<MimoChannelModelInterface>& interfacePtr) override;

private:

    void SetTime(
        const unsigned int channelNumber,
        const TimeType& timeForData);

    void SetTime(const TimeType& timeForData);

    unsigned int GetNumberOfAntennasFor(const NodeIdType& nodeId) const;

    void GetChannelMatrix(
        const unsigned int channelIndex,
        const NodeIdType& nodeId1,
        const unsigned int nodeId1CellSectorId,
        const NodeIdType& nodeId2,
        const unsigned int nodeId2CellSectorId,
        const unsigned int subcarrierIndex,
        MimoChannelMatrixType& mimoChannelMatrix);

    // Note: Const reference to internal structure for efficiency.

    const MimoChannelMatrixType& GetChannelMatrix(
        const unsigned int channelIndex,
        const NodeIdType& nodeId1,
        const unsigned int nodeId1CellSectorId,
        const NodeIdType& nodeId2,
        const unsigned int nodeId2CellSectorId,
        const unsigned int subcarrierIndex);

    InterfaceOrInstanceIdType propagationModelInstanceId;

    // Time delay between taps, e.g. 0ns 10ns 20ns 30ns ... (is 10ns).
    double tapDelayIncrementSecs;

    double cutOffFrequencyHz;

    // Possibly make variables link specific (link specific doppler effect).
    TimeType samplingIntervalTime;
    TimeType fadingSamplingIntervalTime;

    //----------------------------------

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


    struct LinkInfoType {

        TgnMimoChModel::LinkParametersType linkParameters;

        RandomNumberGenerator aRandomNumberGenerator;

        ublas::vector<complex<double> > riceVectorLos;

        // Time domain gaussian noise filtering state variables:

        TimeType startTime;
        TimeType fadingSamplingIntervalTime;
        unsigned long long int currentFadingSampleNumber;

        bool mustAddLineOfSightComponent;

        // Sample time just provides a time "granularity" to channel matrix changes to lower
        // computatation costs.

        TimeType samplingIntervalTime;
        TimeType nextSampleTime;

        vector<TgnMimoChModel::OutputInfoForTapType> outputInfoForTaps;

        vector<MimoChannelMatrixType> mimoChannelMatrices;

        LinkInfoType(const RandomNumberGeneratorSeedType& linkSeed)
            : aRandomNumberGenerator(linkSeed), nextSampleTime(ZERO_TIME),
              currentFadingSampleNumber(0), mustAddLineOfSightComponent(true) {}

        LinkInfoType() : aRandomNumberGenerator(1) { assert(false); abort(); }

    };//LinkInfoType//



    struct ChannelDataType {
        map<LinkKeyType, LinkInfoType> linkInfos;
        double carrierFrequencyHz;
        double carrierBandwidthHz;
        unsigned int numberSubcarriers;
        RandomNumberGeneratorSeedType channelLevelSeed;

    };//ChannelDataType//

    vector<shared_ptr<ChannelDataType> > channels;
    unsigned int baseChannelNumber;

    struct NodeInfoType {
        shared_ptr<ObjectMobilityModel> mobilityModelPtr;
        unsigned int numberAntennas;
        double normalizedAntennaSpacing;

    };//NodeInfoType//

    FastIntToDataMap<NodeIdType, NodeInfoType> nodeInfoMap;

    TgnMimoChModel::ChannelModelChoicesType modelChoice;
    vector<TgnMimoChModel::InfoForTapType> infoForTaps;
    unsigned int tgacChannelSamplingRateExpansionFactor;

    double kFactor;
    double kFactorBreakpointMeters;

    TimeType currentTime;

    RandomNumberGeneratorSeedType instanceLevelSeed;
    static const int channelSeedHashingInput = 3756201631;

    //----------------------------------------------------------------------------------------------

    class ModelInterface: public MimoChannelModelInterface {
    public:

        virtual void SetTime(const TimeType& timeForData) override
        {
            channelModelPtr->SetTime(timeForData);
        }

        virtual unsigned int GetNumberOfAntennas() const override
        {
            return(channelModelPtr->GetNumberOfAntennasFor(nodeId));
        }

        virtual unsigned int GetNumberOfAntennasFor(const NodeIdType& otherNodeId) const override
        {
            return(channelModelPtr->GetNumberOfAntennasFor(otherNodeId));
        }

        virtual void GetChannelMatrix(
            const unsigned int channelNumber,
            const NodeIdType& sourceNodeId,
            const unsigned int subcarrierIndex,
            MimoChannelMatrixType& mimoChannelMatrix) override
        {
            channelModelPtr->GetChannelMatrix(
                channelNumber,
                sourceNodeId,
                0,
                nodeId,
                0,
                subcarrierIndex,
                mimoChannelMatrix);
        }


        virtual void GetChannelMatrix(
            const unsigned int channelNumber,
            const NodeIdType& otherNodeId,
            const unsigned int otherNodeCellSectorId,
            const unsigned int subcarrierIndex,
            MimoChannelMatrixType& mimoChannelMatrix) override
        {
            channelModelPtr->GetChannelMatrix(
                channelNumber,
                otherNodeId,
                otherNodeCellSectorId,
                nodeId,
                cellSectorId,
                subcarrierIndex,
                mimoChannelMatrix);
        }

        virtual const MimoChannelMatrixType& GetChannelMatrix(
            const unsigned int channelNumber,
            const NodeIdType& sourceNodeId,
            const unsigned int subcarrierIndex) override
        {
            assert(false && "Not Implemented"); abort(); return *(new MimoChannelMatrixType());
        }

        virtual const MimoChannelMatrixType& GetChannelMatrix(
            const unsigned int channelNumber,
            const NodeIdType& sourceNodeId,
            const unsigned int sourceCellSectorId,
            const unsigned int subcarrierIndex) override
        {
            assert(false && "Not Implemented"); abort(); return *(new MimoChannelMatrixType());
        }


    private:

        ModelInterface(
            const shared_ptr<OnDemandTgnMimoChannelModel>& channelModelPtr,
            const NodeIdType& nodeId,
            const unsigned int cellSectorId);

        shared_ptr<OnDemandTgnMimoChannelModel> channelModelPtr;

        NodeIdType nodeId;
        unsigned int cellSectorId;

        friend class OnDemandTgnMimoChannelModel;

    };//ModelInterface//

    //----------------------------------------------------------------------------------------------

    void InitializeLink(ChannelDataType& channelData, const LinkKeyType& linkKey);

    void UpdateChannelMatricesForLink(
        const double& channelCenterFrequencyHz,
        const double& channelBandwidthHz,
        const unsigned int numberSubcarriers,
        LinkInfoType& linkInfo);

    double CalcDistanceMeters(const NodeIdType& nodeId1, const NodeIdType& nodeId2) const;

};//OnDemandTgnMimoChannelModel//




inline
OnDemandTgnMimoChannelModel::OnDemandTgnMimoChannelModel(
    const ParameterDatabaseReader& theParameterDatabase,
    const InterfaceOrInstanceIdType& initPropagationModelInstanceId,
    const unsigned int initBaseChannelNumber,
    const unsigned int numberChannels,
    const vector<double>& channelCarrierFrequencyListMhz,
    const vector<double>& channelBandwidthListMhz,
    const unsigned int numberSubcarriersPerChannel,
    const RandomNumberGeneratorSeedType& runSeed)
    :
    propagationModelInstanceId(initPropagationModelInstanceId),
    channels(numberChannels),
    baseChannelNumber(initBaseChannelNumber),
    tapDelayIncrementSecs(pow(10.0, -8.0)),
    currentTime(ZERO_TIME),
    instanceLevelSeed(HashInputsToMakeSeed(runSeed, HashStringToInt(initPropagationModelInstanceId)))
{
    // Could make movement for doppler link specific.

    double scattererMovementSpeedMSec = TgnMimoChModel::DefaultScatterMovementSpeedMSec;

    if (theParameterDatabase.ParameterExists(
            "tgn-mimo-channel-scatterer-movement-meters-sec",
            propagationModelInstanceId)) {

        scattererMovementSpeedMSec =
            theParameterDatabase.ReadDouble(
                "tgn-mimo-channel-scatterer-movement-meters-sec",
                propagationModelInstanceId);
    }//if//

    // Assumption: first channel frequency is representative of all channels for doppler.

    const double carrierFrequencyHz = 1000000.0 * channelCarrierFrequencyListMhz.front();
    const double wavelengthMeters = TgnMimoChModel::SpeedOfLightMSecs / carrierFrequencyHz;
    (*this).cutOffFrequencyHz = (scattererMovementSpeedMSec / wavelengthMeters);

    // Tables for time domain filtering are based on this value for "normalizedDopplerSpread".
    // Do not change!

    const double normalizedDopplerSpread = (1.0/300);

    (*this).fadingSamplingIntervalTime =
        ConvertDoubleSecsToTime(normalizedDopplerSpread / cutOffFrequencyHz);

    samplingIntervalTime = fadingSamplingIntervalTime;
    if (theParameterDatabase.ParameterExists(
        "tgn-mimo-channel-sampling-interval-time", propagationModelInstanceId)) {

        samplingIntervalTime =
            theParameterDatabase.ReadTime(
                "tgn-mimo-channel-sampling-interval-time",
                propagationModelInstanceId);

        if (samplingIntervalTime == ZERO_TIME) {
            cerr << "Error in TGN MIMO: parameter \"tgn-mimo-channel-sampling-interval-time\" "
                 << "cannot be 0." << endl;
            exit(1);
        }//if//
    }//if//

    tgacChannelSamplingRateExpansionFactor = 1;

    if (theParameterDatabase.ParameterExists(
        "tgn-mimo-tgac-channel-sampling-rate-expansion-factor",
        propagationModelInstanceId))  {

        tgacChannelSamplingRateExpansionFactor =
            theParameterDatabase.ReadInt(
                "tgn-mimo-tgac-channel-sampling-rate-expansion-factor",
                propagationModelInstanceId);

        tapDelayIncrementSecs /= tgacChannelSamplingRateExpansionFactor;

    }//if//

    for(unsigned int i = 0; (i < numberChannels); i++) {
        channels[i].reset(new ChannelDataType());
        ChannelDataType& channel = *channels[i];
        channel.carrierFrequencyHz = 1000000.0 * channelCarrierFrequencyListMhz.at(i);
        if (channelBandwidthListMhz.at(i) != channelBandwidthListMhz.at(0)) {
            cerr << "Error in Tgn Mimo Model: Channel Bandwidths must be the same" << endl;
            exit(1);
        }//if//
        channel.carrierBandwidthHz = 1000000.0 * channelBandwidthListMhz.at(i);
        channel.numberSubcarriers = numberSubcarriersPerChannel;
        channel.channelLevelSeed = HashInputsToMakeSeed(instanceLevelSeed, i, channelSeedHashingInput);
    }//for//

    string modelChoiceString =
        theParameterDatabase.ReadString("tgn-mimo-channel-model-letter", propagationModelInstanceId);

    ConvertStringToUpperCase(modelChoiceString);

    if (modelChoiceString.size() != 1) {
        cerr << "Invalid TGN Mimo Model Letter (.e.g. \"B\": " << modelChoiceString << endl;
        exit(1);
    }//if//

    if (modelChoiceString[0] == 'B') {
        modelChoice = TgnMimoChModel::ModelB;
    }
    else if (modelChoiceString[0] == 'C') {
        modelChoice = TgnMimoChModel::ModelC;
    }
    else if (modelChoiceString[0] == 'D') {
        modelChoice = TgnMimoChModel::ModelD;
    }
    else if (modelChoiceString[0] == 'E') {
        modelChoice = TgnMimoChModel::ModelE;
    }
    else {
        cerr << "Invalid TGN Mimo Model Letter (.e.g. \"B\": " << modelChoiceString << endl;
        exit(1);
    }//if//

    TgnMimoChModel::GetModelParameters(
        modelChoice, tgacChannelSamplingRateExpansionFactor,
        kFactor, kFactorBreakpointMeters, infoForTaps);

    TgnMimoChModel::CalculatePowerAngleFactorQs(infoForTaps);

}//OnDemandTgnMimoChannelModel()//



inline
unsigned int OnDemandTgnMimoChannelModel::GetNumberOfAntennasFor(const NodeIdType& nodeId) const
{
    return (nodeInfoMap.GetValue(nodeId).numberAntennas);
}



inline
void OnDemandTgnMimoChannelModel::SetTime(const TimeType& timeForData)
{
    (*this).currentTime = timeForData;

}//SetTime//



inline
void OnDemandTgnMimoChannelModel::InitializeLink(
    ChannelDataType& channelData,
    const LinkKeyType& linkKey)
{
    const RandomNumberGeneratorSeedType linkSeed =
        HashInputsToMakeSeed(
            HashInputsToMakeSeed(channelData.channelLevelSeed, linkKey.nodeId1, linkKey.node1CellSector),
            linkKey.nodeId2,
            linkKey.node2CellSector);

    channelData.linkInfos.insert(
        std::pair<LinkKeyType, LinkInfoType>(linkKey, LinkInfoType(linkSeed)));

    LinkInfoType& linkInfo = channelData.linkInfos[linkKey];

    const NodeInfoType& node1Info = nodeInfoMap[linkKey.nodeId1];
    const NodeInfoType& node2Info = nodeInfoMap[linkKey.nodeId2];
    const unsigned int numTxAntennas = node1Info.numberAntennas;
    const unsigned int numRxAntennas = node2Info.numberAntennas;

    linkInfo.linkParameters.departureNodeNumberAntennas = node1Info.numberAntennas;
    linkInfo.linkParameters.departureNodeAntennaSpacing = node1Info.normalizedAntennaSpacing;
    linkInfo.linkParameters.arrivalNodeNumberAntennas = node2Info.numberAntennas;
    linkInfo.linkParameters.arrivalNodeAntennaSpacing = node2Info.normalizedAntennaSpacing;

    linkInfo.startTime = currentTime;
    linkInfo.fadingSamplingIntervalTime = fadingSamplingIntervalTime;
    linkInfo.samplingIntervalTime = samplingIntervalTime;

    linkInfo.outputInfoForTaps.resize(infoForTaps.size());
    for(unsigned int i = 0; (i < linkInfo.outputInfoForTaps.size()); i++) {
        TgnMimoChModel::OutputInfoForTapType& tapInfo = linkInfo.outputInfoForTaps[i];

        TgnMimoChModel::CalculateSpatialCorrelationMatrix(
            linkInfo.linkParameters,
            infoForTaps[i],
            tapInfo.correlationMatrix);

        // Initialize filter states.

        tapInfo.currentFilterStates.resize(node2Info.numberAntennas * node1Info.numberAntennas);
        tapInfo.currentFadingVector.resize(node2Info.numberAntennas * node1Info.numberAntennas);
        tapInfo.nextFadingVector.resize(node2Info.numberAntennas * node1Info.numberAntennas);

        for(unsigned int j = 0; (j <tapInfo.currentFilterStates.size()); j++) {
            for(unsigned int k = 0; (k < TgnMimoChModel::numberFadingSamplesForFilterInitialization); k++) {
                complex<double> notUsed;

                TgnMimoChModel::MakeBellFilterFadingValue(
                    linkInfo.aRandomNumberGenerator, tapInfo.currentFilterStates[j], notUsed);
            }//for//

            TgnMimoChModel::MakeBellFilterFadingValue(
                linkInfo.aRandomNumberGenerator,
                tapInfo.currentFilterStates[j],
                tapInfo.currentFadingVector[j]);

            TgnMimoChModel::MakeBellFilterFadingValue(
                linkInfo.aRandomNumberGenerator,
                tapInfo.currentFilterStates[j],
                tapInfo.nextFadingVector[j]);

        }//for//
    }//for//

    linkInfo.riceVectorLos =
        TgnMimoChModel::CreateLosRiceVector(
            infoForTaps.at(0),
            linkInfo.linkParameters);

}//InitializeLink//



inline
void OnDemandTgnMimoChannelModel::UpdateChannelMatricesForLink(
    const double& channelCenterFrequencyHz,
    const double& channelBandwidthHz,
    const unsigned int numberSubcarriers,
    LinkInfoType& linkInfo)
{
    using std::max;

    if(linkInfo.nextSampleTime > currentTime) {
        // Up to date.
        return;
    }//if//

    const unsigned long long int targetFadingSampleNumber =
        (currentTime - linkInfo.startTime) / linkInfo.fadingSamplingIntervalTime;

    const unsigned long long int numberFadingSamples =
        (targetFadingSampleNumber - linkInfo.currentFadingSampleNumber);

    const TimeType currentFadingSampleTime =
        linkInfo.startTime + (targetFadingSampleNumber * linkInfo.fadingSamplingIntervalTime);

    const double currentFadingTimeSecs = ConvertTimeToDoubleSecs(currentFadingSampleTime);
    const double nextFadingTimeSecs =
        (currentFadingTimeSecs + ConvertTimeToDoubleSecs(linkInfo.fadingSamplingIntervalTime));

    for(unsigned int i = 0; (i < linkInfo.outputInfoForTaps.size()); i++) {
        TgnMimoChModel::InfoForTapType& tapInfo = infoForTaps[i];
        TgnMimoChModel::OutputInfoForTapType& outputTapInfo = linkInfo.outputInfoForTaps[i];

        if (numberFadingSamples > 0) {
            for(unsigned int j = 0; (j < outputTapInfo.currentFadingVector.size()); j++) {

                if (numberFadingSamples == 1) {
                    outputTapInfo.currentFadingVector = outputTapInfo.nextFadingVector;
                }
                else {
                    for(unsigned long long int k = 0; (k < (numberFadingSamples-1)); k++) {
                        TgnMimoChModel::MakeBellFilterFadingValue(
                            linkInfo.aRandomNumberGenerator,
                            outputTapInfo.currentFilterStates[j],
                            outputTapInfo.currentFadingVector[j]);
                    }//for//
                }//if//

                TgnMimoChModel::MakeBellFilterFadingValue(
                    linkInfo.aRandomNumberGenerator,
                    outputTapInfo.currentFilterStates[j],
                    outputTapInfo.nextFadingVector[j]);

            }//for//
        }//if//

        // Make local copies:

        ublas::vector<complex<double> > currentFadingVector = outputTapInfo.currentFadingVector;
        ublas::vector<complex<double> > nextFadingVector = outputTapInfo.nextFadingVector;

        // Transform fading vectors with the spatial correlation and power normalize.

        const double sqrtOfPower =
            sqrt(tapInfo.GetNormalizedPowerDelayProfilePower(linkInfo.mustAddLineOfSightComponent));

        currentFadingVector = sqrtOfPower * prod(outputTapInfo.correlationMatrix, currentFadingVector);

        nextFadingVector = sqrtOfPower * prod(outputTapInfo.correlationMatrix, nextFadingVector);

        // Addition of the Rice component, only for first tap.

        if ((i == 0) && (linkInfo.mustAddLineOfSightComponent)) {
            // Calculation of the Rice phasor
            // AoA/AoD hard-coded to 45 degrees

            // Sum the NLOS and the LOS components adjusting percentage so combined signal sums
            // has the total tap power.

            const double losFactor = sqrt(kFactor / (kFactor + 1.0));

            const complex<double> currentRicePhasor = exp(
                complex<double>(
                    0.0,
                    ((2.0 * PI * cutOffFrequencyHz) * cos(PI/4) * currentFadingTimeSecs)));

            currentFadingVector  =
                (((1.0 - losFactor) * currentFadingVector) +
                 (losFactor * sqrtOfPower * currentRicePhasor * linkInfo.riceVectorLos));

            const complex<double> nextRicePhasor = exp(
                complex<double>(
                    0.0,
                    ((2.0 * PI * cutOffFrequencyHz) * cos(PI/4) * nextFadingTimeSecs)));

            nextFadingVector  =
                (((1.0 - losFactor) * nextFadingVector) +
                 (losFactor * sqrtOfPower * nextRicePhasor * linkInfo.riceVectorLos));

        }//if//

        // Sample time just provides a time "granularity" to channel matrix changes for
        // the purpose of computation minimization.

        const unsigned long long int sampleNumber =
            (currentTime - linkInfo.startTime) / linkInfo.samplingIntervalTime;

        linkInfo.nextSampleTime =
            linkInfo.startTime + ((sampleNumber + 1) * linkInfo.samplingIntervalTime);

        const unsigned int numTxAntennas = linkInfo.linkParameters.departureNodeNumberAntennas;
        const unsigned int numRxAntennas = linkInfo.linkParameters.arrivalNodeNumberAntennas;

        outputTapInfo.hMatrix.resize(numRxAntennas, numTxAntennas, false);

        const double currentTimeSecs = ConvertTimeToDoubleSecs(currentTime);

        for(unsigned int k = 0; (k < (numRxAntennas * numTxAntennas)); k++) {
            outputTapInfo.hMatrix((k % numRxAntennas), (k / numRxAntennas)) =
                ScenSim::CalcInterpolatedValue(
                    currentFadingTimeSecs,
                    currentFadingVector[k],
                    nextFadingTimeSecs,
                    nextFadingVector[k],
                    currentTimeSecs);
        }//for//
    }//for//

    linkInfo.currentFadingSampleNumber = targetFadingSampleNumber;

    // Combine Tap H matrices into subcarrier channel matrices.

    TgnMimoChModel::CombineTapHMatricesIntoChannelMatrices(
        linkInfo.linkParameters,
        channelCenterFrequencyHz,
        channelBandwidthHz,
        numberSubcarriers,
        tapDelayIncrementSecs,
        linkInfo.outputInfoForTaps,
        linkInfo.mimoChannelMatrices);

}//UpdateChannelMatricesForLink//


inline
double OnDemandTgnMimoChannelModel::CalcDistanceMeters(
    const NodeIdType& nodeId1,
    const NodeIdType& nodeId2) const
{
    ObjectMobilityPosition position1;
    ObjectMobilityPosition position2;
    nodeInfoMap.GetValue(nodeId1).mobilityModelPtr->GetPositionForTime(currentTime, position1);
    nodeInfoMap.GetValue(nodeId2).mobilityModelPtr->GetPositionForTime(currentTime, position2);

    return (ScenSim::CalcDistanceMeters(position1, position2));

}//CalcDistanceMeters//



inline
void OnDemandTgnMimoChannelModel::GetChannelMatrix(
    const unsigned int channelNumber,
    const NodeIdType& nodeId1,
    const unsigned int node1CellSectorId,
    const NodeIdType& nodeId2,
    const unsigned int node2CellSectorId,
    const unsigned int subcarrierIndex,
    MimoChannelMatrixType& mimoChannelMatrix)
{
    assert(nodeId1 != nodeId2);

    ChannelDataType& channelData = *channels.at(channelNumber - baseChannelNumber);

    const bool linkIsReversed = (nodeId1 > nodeId2);

    LinkKeyType linkKey(
        nodeId1,
        node1CellSectorId,
        nodeId2,
        node2CellSectorId);

    typedef map<LinkKeyType, LinkInfoType>::iterator IterType;

    IterType iter = channelData.linkInfos.find(linkKey);

    if (iter == channelData.linkInfos.end()) {
        (*this).InitializeLink(channelData, linkKey);
        iter = channelData.linkInfos.find(linkKey);
    }//if//

    LinkInfoType& linkInfo = iter->second;

    if(linkInfo.nextSampleTime <= currentTime) {

        linkInfo.mustAddLineOfSightComponent =
            (CalcDistanceMeters(nodeId1, nodeId2) < kFactorBreakpointMeters);

        (*this).UpdateChannelMatricesForLink(
            channelData.carrierFrequencyHz,
            channelData.carrierBandwidthHz,
            channelData.numberSubcarriers,
            linkInfo);

    }//if//

    const MimoChannelMatrixType& theMatrix = linkInfo.mimoChannelMatrices.at(subcarrierIndex);

    if (!linkIsReversed) {
        mimoChannelMatrix.resize(theMatrix.size1(), theMatrix.size2(), false);
        mimoChannelMatrix = theMatrix;
    }
    else {
        // Transpose stored matrix

        mimoChannelMatrix.resize(theMatrix.size2(), theMatrix.size1(), false);

        for(unsigned int i = 0; (i < mimoChannelMatrix.size1()); i++) {
            for(unsigned int j = 0; (j < mimoChannelMatrix.size2()); j++) {
                mimoChannelMatrix(i, j) = theMatrix(j, i);
            }//for//
        }//for//
    }//if//

}//GetChannelMatrix//


//--------------------------------------------------------------------------------------------------

inline
OnDemandTgnMimoChannelModel::ModelInterface::ModelInterface(
    const shared_ptr<OnDemandTgnMimoChannelModel>& initChannelModelPtr,
    const NodeIdType& initNodeId,
    const unsigned int initCellSectorId)
    :
    channelModelPtr(initChannelModelPtr),
    nodeId(initNodeId),
    cellSectorId(initCellSectorId)
{
}


//--------------------------------------------------------------------------------------------------


inline
void OnDemandTgnMimoChannelModel::CreateNewMimoChannelModelInterfaceForBasestation(
    const ParameterDatabaseReader& theParameterDatabase,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId,
    const unsigned int cellSectorId,
    const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
    shared_ptr<MimoChannelModelInterface>& interfacePtr)
{
    interfacePtr.reset(new ModelInterface(shared_from_this(), nodeId, cellSectorId));

    NodeInfoType nodeInfo;

    nodeInfo.mobilityModelPtr = mobilityModelPtr;
    nodeInfo.numberAntennas =
        theParameterDatabase.ReadNonNegativeInt("tgn-mimo-channel-number-antennas", nodeId, interfaceId);
    nodeInfo.normalizedAntennaSpacing =
        theParameterDatabase.ReadDouble("tgn-mimo-channel-normalized-antenna-spacing", nodeId, interfaceId);

    (*this).nodeInfoMap.InsertMapping(nodeId, nodeInfo);

}//CreateNewMimoChannelModelInterfaceForBasestation//


inline
void OnDemandTgnMimoChannelModel::CreateNewMimoChannelModelInterface(
    const ParameterDatabaseReader& theParameterDatabase,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId,
    const shared_ptr<ObjectMobilityModel>& mobilityModelPtr,
    shared_ptr<MimoChannelModelInterface>& interfacePtr)
{
    (*this).CreateNewMimoChannelModelInterfaceForBasestation(
        theParameterDatabase, nodeId, interfaceId, 0, mobilityModelPtr, interfacePtr);
}

}//namespace//


#endif
