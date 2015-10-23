// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

//
// This file is for Scenargie user's trace definitions and will not be changed
// by Space-Time Engineering so that the user can replace this file during
// Scenargie version upgrades without worrying about missing any updates.
//

#ifndef SCENSIM_USER_TRACE_DEFS_H
#define SCENSIM_USER_TRACE_DEFS_H

#include "scensim_trace.h"

namespace ScenSim {

const TraceTagType TraceExample1 = FirstUserTraceTag + 0;
const TraceTagType TraceExample2 = FirstUserTraceTag + 1;

const char * const userTraceTagNames[] = {"example1", "example2"};
const int numberUserTraceTags = (sizeof(userTraceTagNames) / sizeof(char*));

}//namespace

#endif

