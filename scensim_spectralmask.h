// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_SPECTRALMASK_H
#define SCENSIM_SPECTRALMASK_H

#include "scensim_proploss.h"

namespace ScenSim {

struct SpectralMaskPointType {
    double distanceFromCenterFrequencyMhz;
    double relativePowerDb;

    SpectralMaskPointType() : distanceFromCenterFrequencyMhz(0.0),relativePowerDb(0.0) { }
};

typedef vector<SpectralMaskPointType> SpectralMaskType;

const double LnOf10DividedBy10 = (log(10.0) / 10.0);

inline
double IntegrateDbRangeLineOverNormalizedDomain(
    const double& startDb,
    const double& endDb)
{
    const double startNonDb = ConvertToNonDb(startDb);

    if (startDb == endDb) {
        return (startNonDb);
    }//if//

    const double term = (endDb - startDb) * LnOf10DividedBy10;
    return (startNonDb * (1.0/term) * (exp(term) - 1.0));

}//IntegrateDbRangeLine//

inline
double CalcChannelInterferenceForMaskSegment(
    const double& nominalTransmitBandwidthMhz,
    const double& lowFrequencyMhz,
    const double& lowFrequencyRelativePowerDb,
    const double& highFrequencyMhz,
    const double& highFrequencyRelativePowerDb,
    const double& receiveFrequencyMinMhz,
    const double& receiveFrequencyMaxMhz)
{
    using std::min;
    using std::max;

    if ((lowFrequencyMhz > receiveFrequencyMaxMhz) ||
        (highFrequencyMhz < receiveFrequencyMinMhz)) {

        return 0.0;
    }//if//

    const double overlapLowMhz = max(lowFrequencyMhz, receiveFrequencyMinMhz);
    const double overlapHighMhz = min(highFrequencyMhz, receiveFrequencyMaxMhz);

    const double startDb =
        CalcInterpolatedValue(
            lowFrequencyMhz,
            lowFrequencyRelativePowerDb,
            highFrequencyMhz,
            highFrequencyRelativePowerDb,
            overlapLowMhz);

    const double endDb =
        CalcInterpolatedValue(
            lowFrequencyMhz,
            lowFrequencyRelativePowerDb,
            highFrequencyMhz,
            highFrequencyRelativePowerDb,
            overlapHighMhz);

     const double segmentWidthMhz = (overlapHighMhz - overlapLowMhz);

     return
        ((segmentWidthMhz / nominalTransmitBandwidthMhz) *
        IntegrateDbRangeLineOverNormalizedDomain(startDb, endDb));

}//CalcChannelInterferenceForMaskSegment//



inline
double CalcChannelInterferenceFactor(
    const double& transmitCenterFrequencyMhz,
    const SpectralMaskType& transmitSpectralMask,
    const double& nominalTransmitBandwidthMhz,
    const double& receiveFrequencyMinMhz,
    const double& receiveFrequencyMaxMhz)
{
    assert(nominalTransmitBandwidthMhz > 0.0);

    double channelInterferenceFactor = 0.0;

    // Higher than center frequency calcuation

    // Starts with implicit 0 Mhz (center) by 0.0 relative db mask point.

    SpectralMaskPointType lastPoint;

    for(unsigned int i = 0; (i < transmitSpectralMask.size()); i++) {
        const double lowFrequencyMhz =
            (transmitCenterFrequencyMhz + lastPoint.distanceFromCenterFrequencyMhz);
        const double highFrequencyMhz =
            (transmitCenterFrequencyMhz + transmitSpectralMask[i].distanceFromCenterFrequencyMhz);

        if (lowFrequencyMhz > receiveFrequencyMaxMhz) {
            break;
        }//if//

        channelInterferenceFactor +=
            CalcChannelInterferenceForMaskSegment(
                nominalTransmitBandwidthMhz,
                lowFrequencyMhz,
                lastPoint.relativePowerDb,
                highFrequencyMhz,
                transmitSpectralMask[i].relativePowerDb,
                receiveFrequencyMinMhz,
                receiveFrequencyMaxMhz);

        lastPoint = transmitSpectralMask[i];
    }//for//

    // Lower than center frequency calcuation

    // Starts with implicit 0 Mhz by 0.0 db mask point.
    lastPoint = SpectralMaskPointType();

    for(unsigned int i = 0; (i < transmitSpectralMask.size()); i++) {
        const double highFrequencyMhz =
            (transmitCenterFrequencyMhz - lastPoint.distanceFromCenterFrequencyMhz);
        const double lowFrequencyMhz =
            (transmitCenterFrequencyMhz - transmitSpectralMask[i].distanceFromCenterFrequencyMhz);

        if (highFrequencyMhz < receiveFrequencyMinMhz) {
            break;
        }//if//

        channelInterferenceFactor +=
            CalcChannelInterferenceForMaskSegment(
                nominalTransmitBandwidthMhz,
                lowFrequencyMhz,
                transmitSpectralMask[i].relativePowerDb,
                highFrequencyMhz,
                lastPoint.relativePowerDb,
                receiveFrequencyMinMhz,
                receiveFrequencyMaxMhz);

        lastPoint = transmitSpectralMask[i];
    }//for//

    return (channelInterferenceFactor);

}//CalcChannelInterferenceFactor//


};//namespace//

#endif

