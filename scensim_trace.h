// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_TRACE_H
#define SCENSIM_TRACE_H

// Scenargie Users should modify "scensim_user_trace_defs.h" not this file.

#include <assert.h>
#include <string>
#include <map>
#include <fstream>

#include "scensim_support.h"
#include "scensim_nodeid.h"
#include "scensim_time.h"

namespace ScenSim {

using std::string;
using std::map;
using std::ifstream;
using std::streamoff;

typedef unsigned int TraceTagType;

const TraceTagType TraceMobility = 0;
const TraceTagType TraceApplication = 1;
const TraceTagType TraceTransport = 2;
const TraceTagType TraceNetwork = 3;
const TraceTagType TraceRouting = 4;
const TraceTagType TraceMac = 5;
const TraceTagType TracePhy = 6;
const TraceTagType TracePhyInterference = 7;
const TraceTagType TraceGis = 8;
const TraceTagType TraceMas = 9;
const TraceTagType FirstUserTraceTag = 10;

const char* const standardTraceTagNames[] = {"Mobility",
                                             "Application",
                                             "Transport",
                                             "Network",
                                             "Routing",
                                             "Mac",
                                             "Phy",
                                             "PhyInterference",
                                             "Gis",
                                             "Mas"};

const TraceTagType numberStandardTraceTags = (sizeof(standardTraceTagNames)/sizeof(char*));

TraceTagType LookupTraceTagIndex(const string& tagString);
void LookupTraceTagIndex(const string& tagString, TraceTagType& traceTag, bool& succeeded);
const char * GetTraceTagName(const TraceTagType& traceTag);
unsigned int GetNumberTraceTags();

//------------------------------------------------------------------------------------------

//for indexing trace files
static const int INDEX_INTERVAL_BYTES = 1048576; // 1MB
typedef unsigned int TraceTextIdType;

static const string TRACE_BLANK_STRING_ID = "---";

//------------------------------------------------------------------------------------------
//binary trace format parameters
static const unsigned char TRACE_FORMAT_VERSION = 3;
//version 1/2/3 header
static const unsigned char TRACE_OPTION_NUMBER = 0; //0:general, 1:redandunt

static const unsigned char TRACE_RECORD_SIZE_BYTES = 1; //255bytes
static const unsigned char TRACE_TIME_SIZE_BYTES = 6; //3.25 days
static const unsigned char TRACE_NODE_ID_SIZE_BYTES = 4; //supports gis object id (large number)
static const unsigned char TRACE_ID_SIZE_BYTES = 2; //65k

static const unsigned char TRACE_FILE_POSITION_SIZE = 8;//supports 64bit OS (4 bytes for old 32 bit version: sizeof(streamoff))
static const unsigned char TRACE_MAX_NODE_ID_SIZE = sizeof(NodeIdType);
static const unsigned char TRACE_MAX_SIMTIME_SIZE = sizeof(TimeType);

static const unsigned char TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER1 = 9; //for insert position lately

//53bytes//
static const unsigned char TRACE_HEADER_SIZE_BYTES_FOR_VER1 =
    TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER1 +
    (TRACE_FILE_POSITION_SIZE * 4) + //event, model, instance, trace id mapping
    TRACE_MAX_NODE_ID_SIZE +
    TRACE_MAX_SIMTIME_SIZE;

static const unsigned char TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER2 =
    TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER1;
static const unsigned char TRACE_HEADER_SIZE_BYTES_FOR_VER2 =
    TRACE_HEADER_SIZE_BYTES_FOR_VER1;

static const unsigned char TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER3 =
    TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER1;
static const unsigned char TRACE_HEADER_SIZE_BYTES_FOR_VER3 =
    TRACE_HEADER_SIZE_BYTES_FOR_VER1;

//latest version header
static const unsigned char LATEST_TRACE_HEADER_FRONT_SIZE_BYTE =
    TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER3;
static const unsigned char LATEST_TRACE_HEADER_SIZE_BYTE =
    TRACE_HEADER_SIZE_BYTES_FOR_VER3;


static inline
bool TraceTextIsOkay(const string& traceText)
{

   for(string::const_iterator i = traceText.begin(); (i != traceText.end()); i++) {
        if (!isalnum(*i) && (*i != '_') && (*i != '-')) {
            return false;
        }//if//
   }//for//

   return true;
}



//----------------------------------------------------------------------------------------------
class BinaryTraceHeader {
public:

    //for write
    BinaryTraceHeader(const size_t version, const size_t optionNum);

    const unsigned char* GetRawData() const { assert(!readMode); return (data); }

    //for read
    BinaryTraceHeader(ifstream& traceFile);

    ~BinaryTraceHeader();

    bool Completed() const { return completed; }

    size_t GetFormatVersion() const { assert(readMode); return (size_t)data[0]; }
    size_t GetOptionNumber() const { assert(readMode); return (size_t)data[1]; }
    size_t GetRecordSizeBytes() const { assert(readMode); return (size_t)data[2]; }
    size_t GetTimeSizeBytes() const { assert(readMode); return (size_t)data[3]; }
    size_t GetNodeIdSizeBytes() const { assert(readMode); return (size_t)data[4]; }
    size_t GetTraceIdSizeBytes() const { assert(readMode); return (size_t)data[5]; }

    streamoff GetEventTextPosition() const
        { assert(readMode); return eventTextPosition; }
    streamoff GetModelTextPosition() const
         { assert(readMode); return modelTextPosition; }
    streamoff GetInstanceTextPosition() const
         { assert(readMode); return instanceTextPosition; }
    streamoff GetTraceIdMappingPosition() const
         { assert(readMode); return traceIdMappingPosition; }
    NodeIdType GetMaxNodeId() const
        { assert(readMode); return maxNodeId; }
    TimeType GetMaxSimTime() const
        { assert(readMode); return maxSimTime; }

private:
    bool readMode;
    bool completed;
    unsigned char* data;

    //for read
    unsigned char version;
    streamoff eventTextPosition;
    streamoff modelTextPosition;
    streamoff instanceTextPosition;
    streamoff traceIdMappingPosition;
    NodeIdType maxNodeId;
    TimeType maxSimTime;

    size_t GetFilePositionSizeBytes() const { assert(readMode); return (size_t)data[6]; }
    size_t GetMaxNodeIdSizeBytes() const { assert(readMode); return (size_t)data[7]; }
    size_t GetMaxSimTimeSizeBytes() const { assert(readMode); return (size_t)data[8]; }

    void CheckPaddingZero(const char* padding, const size_t paddingSize) const {

        for(size_t i = 0; i < paddingSize; i++) {

            if (padding[i] != 0) {
                cerr << "Invalid binary trace header" << endl;
                exit(1);
            }
        }
    }

};//BinaryTraceHeader//

inline
BinaryTraceHeader::BinaryTraceHeader(
    const size_t version,
    const size_t optionNum)
    :
    readMode(false),
    completed(false)
{
    if (version != 3) {
        cerr << "Invalid version." << endl;
        exit(1);
    }

    data = new unsigned char[TRACE_HEADER_SIZE_BYTES_FOR_VER3];

    data[0] = static_cast<unsigned char>(version);
    data[1] = static_cast<unsigned char>(optionNum);
    data[2] = TRACE_RECORD_SIZE_BYTES;
    data[3] = TRACE_TIME_SIZE_BYTES;
    data[4] = TRACE_NODE_ID_SIZE_BYTES;
    data[5] = TRACE_ID_SIZE_BYTES;
    data[6] = TRACE_FILE_POSITION_SIZE;
    data[7] = TRACE_MAX_NODE_ID_SIZE;
    data[8] = TRACE_MAX_SIMTIME_SIZE;
    for(size_t i = TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER3; i < TRACE_HEADER_SIZE_BYTES_FOR_VER3; i++) {
        data[i] = 0;
    }

    completed = true;

}//BinaryTraceHeader//

inline
BinaryTraceHeader::BinaryTraceHeader(ifstream& traceFile)
    :
    readMode(true),
    completed(false),
    data(nullptr)
{
    if (!(traceFile.good())) {
        return;
    }

    //get version
    traceFile.seekg(0, std::ios::beg);
    traceFile.read(reinterpret_cast<char *>(&version), 1);
    if (!(traceFile.good())) {
        return;
    }

    if (version >= 1 && version <= 3) {
        assert(TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER1 == TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER2);
        assert(TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER1 == TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER3);

        data = new unsigned char[TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER1];

        data[0] = version;
        traceFile.read(reinterpret_cast<char *>(&data[1]), (TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER1 - 1));

        const size_t filePositionSize = GetFilePositionSizeBytes();
        traceFile.seekg(TRACE_HEADER_FRONT_SIZE_BYTES_FOR_VER1, std::ios::beg);


        if (filePositionSize == sizeof(streamoff)) {
            //evet text position
            traceFile.read(reinterpret_cast<char *>(&eventTextPosition), filePositionSize);

            //model text position
            traceFile.read(reinterpret_cast<char *>(&modelTextPosition), filePositionSize);

            //instance text position
            traceFile.read(reinterpret_cast<char *>(&instanceTextPosition), filePositionSize);

            //trace id mapping position
            traceFile.read(reinterpret_cast<char *>(&traceIdMappingPosition), filePositionSize);

        }
        else {

            const size_t actualFilePositionSize = sizeof(streamoff);
            assert(filePositionSize > actualFilePositionSize);

            const size_t paddingSize = filePositionSize - actualFilePositionSize;
            char* padding = new char[paddingSize];

            //evet text position
            traceFile.read(reinterpret_cast<char *>(&eventTextPosition), actualFilePositionSize);
            traceFile.read(reinterpret_cast<char *>(&padding[0]), paddingSize);
            CheckPaddingZero(padding, paddingSize);

            //model text position
            traceFile.read(reinterpret_cast<char *>(&modelTextPosition), actualFilePositionSize);
            traceFile.read(reinterpret_cast<char *>(&padding[0]), paddingSize);
            CheckPaddingZero(padding, paddingSize);

            //instance text position
            traceFile.read(reinterpret_cast<char *>(&instanceTextPosition), actualFilePositionSize);
            traceFile.read(reinterpret_cast<char *>(&padding[0]), paddingSize);
            CheckPaddingZero(padding, paddingSize);

            //trace id mapping position
            traceFile.read(reinterpret_cast<char *>(&traceIdMappingPosition), actualFilePositionSize);
            traceFile.read(reinterpret_cast<char *>(&padding[0]), paddingSize);
            CheckPaddingZero(padding, paddingSize);

        }

        //max node id
        const size_t maxNodeIdSize = GetMaxNodeIdSizeBytes();
        traceFile.read(reinterpret_cast<char *>(&maxNodeId), maxNodeIdSize);

        //max sim time
        const size_t maxSimTimeSize = GetMaxSimTimeSizeBytes();
        traceFile.read(reinterpret_cast<char *>(&maxSimTime), maxSimTimeSize);

    }
    else {
        return;
    }

    if (!(traceFile.good())) {
        return;
    }

    completed = true;

}//BinaryTraceHeader//

inline
BinaryTraceHeader::~BinaryTraceHeader(){
    delete [] data;

}//~BinaryTraceHeader//

//------------------------------------------------------------------------------------------
static const size_t TRACE_ID_ELEMENTS_BYTES = 16;
struct TraceIdElements {
    TraceIdElements(
        unsigned int initEventId,
        unsigned int initModelId,
        unsigned int initInstanceId,
        unsigned int initTraceId = 0)
    :
    eventId(initEventId),
    modelId(initModelId),
    instanceId(initInstanceId),
    traceId(initTraceId)
    {}

    TraceIdElements() {}

    unsigned int eventId;
    unsigned int modelId;
    unsigned int instanceId;
    unsigned int traceId;

   //operator does not care traceId

    bool operator==(const TraceIdElements& right) const {
        return (((*this).eventId == right.eventId) &&
                ((*this).modelId == right.modelId) &&
                ((*this).instanceId == right.instanceId));
    }

    bool operator!=(const TraceIdElements& right) const {
        return (((*this).eventId != right.eventId) ||
                ((*this).modelId != right.modelId) ||
                ((*this).instanceId != right.instanceId));
    }

    bool operator<(const TraceIdElements& right) const {
        return (((*this).eventId < right.eventId) ||
               ((*this).eventId == right.eventId && (*this).modelId < right.modelId) ||
               ((*this).eventId == right.eventId &&
                (*this).modelId == right.modelId &&
                (*this).instanceId < right.instanceId));
    }

};

class TextMapForBinaryOutput {
public:

    TextMapForBinaryOutput() {}

    TraceTextIdType GetTextId(const string& textString, bool& isNewText);

    size_t Size() { return textMap.size(); }

    size_t OutputTextMapData(std::ostream& traceFile);

private:
    map<string, TraceTextIdType> textMap;

};//TextMapForBinaryOutput//

inline
TraceTextIdType TextMapForBinaryOutput::GetTextId(const string& textString, bool& isNewText)
{
    map<string, TraceTextIdType>::iterator iter = textMap.find(textString);

    if (iter == textMap.end()) {
        //check text string for using parameter in TraceAnalyzer

        if (!TraceTextIsOkay(textString)) {
            cerr << "Model/Instance/Event name for trace should be [A..Z] or [0..9]: \""
                 << textString << "\""<< endl;
            exit(1);
        }

        TraceTextIdType textMapIndex = static_cast<TraceTextIdType>(textMap.size()+1);
        textMap.insert(make_pair(textString, textMapIndex));
        isNewText = true;
        return textMapIndex;
    }
    else {
        isNewText = false;
        return iter->second;
    }//if//

}

inline
size_t TextMapForBinaryOutput::OutputTextMapData(std::ostream& traceFile)
{

    if (textMap.size() == 0) { return 0; }

    assert(traceFile.good());

    size_t outputBytes = 0;
    map<string, TraceTextIdType>::iterator iter;

    for (iter = textMap.begin(); iter != textMap.end(); ++iter) {

        string id = ConvertToString(iter->second);
        traceFile << id << " ";
        outputBytes += (id.size() + 1);

        string text(iter->first);
        if (text.empty()) {
            text = TRACE_BLANK_STRING_ID;
        }
        traceFile << text << " ";
        outputBytes += (text.size() + 1);

    }//for//

    traceFile.flush();

    return outputBytes;

}


}//namespace//


#endif

