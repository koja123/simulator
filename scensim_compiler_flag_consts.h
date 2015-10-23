// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_COMPILER_FLAG_CONSTS_H
#define SCENSIM_COMPILER_FLAG_CONSTS_H

namespace ScenSim {
#ifdef DISABLE_TRACE_FLAG
const bool TRACE_IS_ENABLED = false;
#else
const bool TRACE_IS_ENABLED = true;
#endif

#ifdef ENABLE_PARALLELISM_FLAG
const bool PARALLELISM_IS_ENABLED = true;
#else
const bool PARALLELISM_IS_ENABLED = false;
#endif


}//namespace//


#endif


