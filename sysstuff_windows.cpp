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
#include <windows.h>
#include <winsock.h>
#include <iostream>
#include "sysstuff.h"

using std::cerr;
using std::endl;


void SetCpuFloatingPointBehavior()
{
    // Do Nothing.
}


void CommitCpuMemoryWrites() {
    MemoryBarrier();
}


long int AtomicDecrement(volatile long int * var) {
    return InterlockedDecrement(var);
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
    static const unsigned long long int deltaEpochInMicrosecs = 11644473600000000ULL;

    FILETIME fileTime;

    GetSystemTimeAsFileTime(&fileTime);

    LARGE_INTEGER largeInt;
    int64_t value;

    largeInt.LowPart = fileTime.dwLowDateTime;
    largeInt.HighPart = fileTime.dwHighDateTime;
    value = (largeInt.QuadPart - deltaEpochInMicrosecs) / 10;

    tvSec = static_cast<uint32_t>(value / 1000000);
    tvUsec = static_cast<uint32_t>(value % 1000000);
    success = true;
}

//--------------------------------------------------------------------------------------------------


class HighResolutionRelativeTimeClock::Implementation {
public:
    ~Implementation() {}
private:
    friend class HighResolutionRelativeTimeClock;

    LARGE_INTEGER startCount;
    LARGE_INTEGER lastCount;
    double conversionToSecsFactor;

};//Implementation//


HighResolutionRelativeTimeClock::HighResolutionRelativeTimeClock()
    :
    implPtr(new Implementation())
{
    LARGE_INTEGER performanceCounterFrequency;
    BOOL status = QueryPerformanceFrequency(&performanceCounterFrequency);
    assert(status);
    implPtr->conversionToSecsFactor = (1.0 / performanceCounterFrequency.QuadPart);

    (*this).RestartClock();
}


HighResolutionRelativeTimeClock::~HighResolutionRelativeTimeClock() {}


void HighResolutionRelativeTimeClock::RestartClock()
{
    BOOL status = QueryPerformanceCounter(&implPtr->startCount);
    assert(status);
    implPtr->lastCount = implPtr->startCount;
}



double HighResolutionRelativeTimeClock::TimeFromStartSecs()
{
    LARGE_INTEGER count;
    BOOL status = QueryPerformanceCounter(&count);
    assert(status);

    unsigned long long int countDifference;

    if (count.QuadPart > implPtr->lastCount.QuadPart) {

        countDifference = count.QuadPart - implPtr->startCount.QuadPart;
        implPtr->lastCount = count;
    }
    else {
        countDifference = implPtr->lastCount.QuadPart - implPtr->startCount.QuadPart;
    }//if//


    return (static_cast<double>(countDifference) * implPtr->conversionToSecsFactor);
}


//--------------------------------------------------------------------------------------------------


