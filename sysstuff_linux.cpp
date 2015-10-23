// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include <assert.h>
#include <iostream>
#include "sysstuff.h"
#include <stdlib.h>
#include <sys/time.h>
#include <netinet/in.h>

using std::cerr;
using std::endl;


#if defined(__MAC_OS_X_VERSION_MAX_ALLOWED)
// Mac OS X
void SetCpuFloatingPointBehavior()
{
}

#else
// Linux
#include <fpu_control.h>
void SetCpuFloatingPointBehavior()
{
    unsigned short cw = 0;
    _FPU_GETCW(cw);        // Get the FPU control word
    cw &= ~_FPU_EXTENDED;  // mask out '80-bit' register precision
    cw |= _FPU_DOUBLE;     // Mask in '64-bit' register precision
    _FPU_SETCW(cw);        // Set the FPU control word
}
#endif


void CommitCpuMemoryWrites() {
    __sync_synchronize();
    //asm volatile("mfence":::"memory"); // Trying the Above
}


long int AtomicDecrement(volatile long int* var) {
    // Warning this needs to be compiled with a later machine model -march=i586, et.c,
    // if this function generates a link error.

    return __sync_sub_and_fetch(var, 1);
}

uint16_t NetToHost16(uint16_t net16)
{
    return ntohs(net16);
}

uint32_t NetToHost32(uint32_t net32)
{
    return ntohl(net32);
}

uint16_t HostToNet16(uint16_t host16)
{
    return htons(host16);
}

uint32_t HostToNet32(uint32_t host32)
{
    return htonl(host32);
}

void GetTimeOfDay(uint32_t& tvSec, uint32_t& tvUsec, bool& success)
{
    struct timeval tv;
    int error = gettimeofday(&tv, NULL);
    if (!error) {
        tvSec = tv.tv_sec;
        tvUsec = tv.tv_usec;
        success = true;
    }
    else {
        success = false;
    }//if//
}

//--------------------------------------------------------------------------------------------------

class HighResolutionRelativeTimeClock::Implementation {
public:
    Implementation();
    ~Implementation() {}

    void RestartClock();

private:
    struct timeval startTimeval;

    friend class HighResolutionRelativeTimeClock;

};//Implementation//


void HighResolutionRelativeTimeClock::Implementation::RestartClock()
{
    int returnVal = gettimeofday(&startTimeval, 0);
    assert(returnVal == 0);
}


HighResolutionRelativeTimeClock::Implementation::Implementation()
{
    (*this).RestartClock();
}


HighResolutionRelativeTimeClock::HighResolutionRelativeTimeClock()
    :
    implPtr(new Implementation())
{
}


HighResolutionRelativeTimeClock::~HighResolutionRelativeTimeClock() {}


void HighResolutionRelativeTimeClock::RestartClock()
{
    implPtr->RestartClock();
}


double HighResolutionRelativeTimeClock::TimeFromStartSecs()
{
    struct timeval currentTimeval;
    int returnVal = gettimeofday(&currentTimeval, 0);
    assert(returnVal == 0);

    return (static_cast<double>(currentTimeval.tv_sec - implPtr->startTimeval.tv_sec) +
            ((currentTimeval.tv_usec - implPtr->startTimeval.tv_usec) / 1000000.0));
}



