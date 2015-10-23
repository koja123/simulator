// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_IP_MODEL_H
#define SCENSIM_IP_MODEL_H

// IP model versions are mutually exclusive.
#if defined(USE_IPV4) && defined(USE_IPV6)
    #error "Cannot use both IPv4 and IPv6 at the same time."
#endif

#if !defined(USE_IPV4) && !defined(USE_IPV6)
    #define USE_IPV4
    //#define USE_IPV6
#endif

#ifdef USE_IPV4
    #include "scensim_ipv4.h"
#endif

#ifdef USE_IPV6
    #include "scensim_ipv6.h"
#endif


#endif
