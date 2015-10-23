// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_user_trace_defs.h"

namespace ScenSim {

const TraceTagType numberTraceTags = (numberStandardTraceTags + numberUserTraceTags);

void LookupTraceTagIndex(const string& tagString, TraceTagType& traceTag, bool& succeeded)
{
    succeeded = false;
    for(TraceTagType i = 0; (i < numberStandardTraceTags); i++) {
        if (IsEqualCaseInsensitive(tagString, standardTraceTagNames[i])) {
            succeeded = true;
            traceTag = i;
        }//if//
    }//for//

    for(TraceTagType i = 0; (i < numberUserTraceTags); i++) {
        if (IsEqualCaseInsensitive(tagString, userTraceTagNames[i])) {
            succeeded = true;
            traceTag = FirstUserTraceTag + i;
        }//if//
    }//for//
}//LookupTraceTagIndex//



TraceTagType LookupTraceTagIndex(const string& tagString)
{
    TraceTagType traceTagIndex;
    bool succeeded;

    LookupTraceTagIndex(tagString, traceTagIndex, succeeded);

    if (!succeeded) {
        cerr << "Error: Unknown trace tag string (trace-enabled-tags): \"" << tagString << "\" not valid." << endl;
        exit(1);
    }//if//

    return traceTagIndex;

}//LookupTraceTagIndex//


const char* GetTraceTagName(const TraceTagType& traceTag)
{
    assert(traceTag < numberTraceTags);
    if (traceTag < numberStandardTraceTags) {
        return (standardTraceTagNames[traceTag]);
    }
    else {
        return (userTraceTagNames[traceTag - FirstUserTraceTag]);
    }//if//

}//GetTraceTagName//


unsigned int GetNumberTraceTags() { return numberTraceTags; }

}//namespace


