// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SYSSTUFF_H
#define SYSSTUFF_H

#include <assert.h>
#include <stdlib.h>
#include <memory>
#include <vector>
#include <stdint.h>

const int CacheLineSizeBytes = 64;
const int MinimumMemoryAlignmentBytes = 8;

// The following function will become obsolete for later gcc versions via the -mpc64 compile flag.
void SetCpuFloatingPointBehavior();

void CommitCpuMemoryWrites();

long int AtomicDecrement(volatile long int* var);

uint16_t NetToHost16(uint16_t net16);
uint32_t NetToHost32(uint32_t net32);
uint16_t HostToNet16(uint16_t host16);
uint32_t HostToNet32(uint32_t host32);

inline
void ConvertHost32ToTwoNet16(
    const uint32_t host32,
    uint16_t& highNet16,
    uint16_t& lowNet16)
{
    const uint32_t net32 = HostToNet32(host32);
    const unsigned char* const net32Ptr = reinterpret_cast<const unsigned char*>(&net32);
    unsigned char* const highNet16Ptr = reinterpret_cast<unsigned char*>(&highNet16);
    unsigned char* const lowNet16Ptr = reinterpret_cast<unsigned char*>(&lowNet16);
    highNet16Ptr[0] = net32Ptr[0];
    highNet16Ptr[1] = net32Ptr[1];
    lowNet16Ptr[0] = net32Ptr[2];
    lowNet16Ptr[1] = net32Ptr[3];
}

inline
uint32_t ConvertTwoNet16ToHost32(
    const uint16_t highNet16,
    const uint16_t lowNet16)
{
    const unsigned char* const highNet16Ptr = reinterpret_cast<const unsigned char*>(&highNet16);
    const unsigned char* const lowNet16Ptr = reinterpret_cast<const unsigned char*>(&lowNet16);
    uint32_t net32;
    unsigned char* const net32Ptr = reinterpret_cast<unsigned char*>(&net32);
    net32Ptr[0] = highNet16Ptr[0];
    net32Ptr[1] = highNet16Ptr[1];
    net32Ptr[2] = lowNet16Ptr[0];
    net32Ptr[3] = lowNet16Ptr[1];
    return (NetToHost32(net32));
}

inline
void ConvertTwoHost64ToTwoNet64(
    const uint64_t& highHost64,
    const uint64_t& lowHost64,
    uint64_t& highNet64,
    uint64_t& lowNet64)
{
    //assuming host uses little endian

    const unsigned char* const highHost64Ptr = reinterpret_cast<const unsigned char*>(&highHost64);
    const unsigned char* const lowHost64Ptr = reinterpret_cast<const unsigned char*>(&lowHost64);
    unsigned char* const highNet64Ptr = reinterpret_cast<unsigned char*>(&highNet64);
    unsigned char* const lowNet64Ptr = reinterpret_cast<unsigned char*>(&lowNet64);

    for(int i = 0; i < 8; ++i) {
        highNet64Ptr[i] = highHost64Ptr[7 - i];
        lowNet64Ptr[i] = lowHost64Ptr[7 - i];
    }//for//

}//ConvertTwoHost64ToTwoNet64//

inline
void ConvertTwoNet64ToTwoHost64(
    const uint64_t& highNet64,
    const uint64_t& lowNet64,
    uint64_t& highHost64,
    uint64_t& lowHost64)
{
    //assuming host uses little endian

    const unsigned char* const highNet64Ptr = reinterpret_cast<const unsigned char*>(&highNet64);
    const unsigned char* const lowNet64Ptr = reinterpret_cast<const unsigned char*>(&lowNet64);
    unsigned char* const highHost64Ptr = reinterpret_cast<unsigned char*>(&highHost64);
    unsigned char* const lowHost64Ptr = reinterpret_cast<unsigned char*>(&lowHost64);

    for(int i = 0; i < 8; ++i) {
        highHost64Ptr[i] = highNet64Ptr[7 - i];
        lowHost64Ptr[i] = lowNet64Ptr[7 - i];
    }//for//

}//ConvertTwoNet64ToTwoHost64//

inline
void ConvertTwoHost64ToNet128(
    const uint64_t& highHost64,
    const uint64_t& lowHost64,
    unsigned char* net128)
{
    //assuming host uses little endian

    const unsigned char* const highHost64Ptr = reinterpret_cast<const unsigned char*>(&highHost64);
    const unsigned char* const lowHost64Ptr = reinterpret_cast<const unsigned char*>(&lowHost64);

    for(int i = 0; i < 8; ++i) {
        net128[i] = highHost64Ptr[7 - i];
        net128[i + 8] = lowHost64Ptr[7 - i];
    }//for//

}//ConvertTwoHost64ToNet128//

inline
void ConvertNet128ToTwoHost64(
    const unsigned char* net128,
    uint64_t& highHost64,
    uint64_t& lowHost64)
{
    //assuming host uses little endian

    unsigned char* const highHost64Ptr = reinterpret_cast<unsigned char*>(&highHost64);
    unsigned char* const lowHost64Ptr = reinterpret_cast<unsigned char*>(&lowHost64);

    for(int i = 0; i < 8; ++i) {
        highHost64Ptr[7 - i] = net128[i];
        lowHost64Ptr[7 - i] = net128[i + 8];
    }//for//

}//ConvertNet128ToTwoHost64//

void GetTimeOfDay(uint32_t& tvSec, uint32_t& tvUsec, bool& success);

//------------------------------------------------------------------------


class AbstractHighResolutionRelativeTimeClock {
public:
    virtual ~AbstractHighResolutionRelativeTimeClock() { }

    virtual double TimeFromStartSecs() = 0;

    virtual void RestartClock() = 0;

};//AbstractHighResolutionRelativeTimeClock//


class AbstractDistributedRelativeTimeClock: public AbstractHighResolutionRelativeTimeClock {
public:
    // For Distributed Syncing.

    virtual void GetStartTimeNativeMemoryBytes(
        std::vector<unsigned char>& nativeStartTimeMemoryBytes) const = 0;

    virtual void SyncUpStartTime(
        const unsigned char nativeStartTimeMemoryBytes[],
        const unsigned int numMemoryBytes) = 0;

};//AbstractDistributedRelativeTimeClock//


class HighResolutionRelativeTimeClock: public AbstractDistributedRelativeTimeClock {
public:
    HighResolutionRelativeTimeClock();

    // virtual methods
    ~HighResolutionRelativeTimeClock();
    double TimeFromStartSecs();
    void RestartClock();

    void GetStartTimeNativeMemoryBytes(
        std::vector<unsigned char>& /*nativeStartTimeMemoryBytes*/) const
        { assert(false && "Not Supported with this Clock Type"); abort(); }

    void SyncUpStartTime(
        const unsigned char /*nativeStartTimeMemoryBytes*/[],
        const unsigned int /*numMemoryBytes*/)
        { assert(false && "Not Supported with this Clock Type"); abort(); }

private:
    class Implementation;
    std::auto_ptr<Implementation> implPtr;

};//HighResolutionRelativeTimeClock//



//------------------------------------------------------------------------

//Future Stuff: const bool MachineIsBigEndian = false;
//Future Stuff:
//Future Stuff: inline
//Future Stuff: void SplitLongLongToTwoUnsignedInts(
//Future Stuff:     const long long int bigInt,
//Future Stuff:     unsigned int& low,
//Future Stuff:     unsigned int& high)
//Future Stuff: {
//Future Stuff:     assert((sizeof(long long int) == sizeof(unsigned int) * 2));
//Future Stuff:
//Future Stuff:     const unsigned int * Overlay = reinterpret_cast<unsigned int *>(bigInt);
//Future Stuff:
//Future Stuff:     unsigned int High;
//Future Stuff:     unsigned int Low;
//Future Stuff:     if (MachineIsBigEndian) {
//Future Stuff:        High = Overlay[0];
//Future Stuff:        Low = Overlay[1];
//Future Stuff:     } else {
//Future Stuff:        High = Overlay[1];
//Future Stuff:        Low = Overlay[0];
//Future Stuff:     }//if//
//Future Stuff:
//Future Stuff: }//SplitLongLongToTwoUnsignedInts//
//Future Stuff:
//Future Stuff: inline
//Future Stuff: long long int CombineTwoUnsignedIntsIntoLongLong(const unsigned int low, const unsigned int high)
//Future Stuff: {
//Future Stuff:     long long int returnValue;
//Future Stuff:
//Future Stuff:     unsigned int* overlay = reinterpret_cast<unsigned int *>(&returnValue);
//Future Stuff:     if (MachineIsBigEndian) {
//Future Stuff:        overlay[0] = high;
//Future Stuff:        overlay[1] = low;
//Future Stuff:     } else {
//Future Stuff:        overlay[0] = low;
//Future Stuff:        overlay[1] = high;
//Future Stuff:     }//if//
//Future Stuff:
//Future Stuff:     assert(returnValue == (((long long int)(UINT_MAX)+1) * high + low));
//Future Stuff:
//Future Stuff:     return returnValue;
//Future Stuff:
//Future Stuff: }//CombineTwoUnsignedIntsIntoLongLong//


#endif






