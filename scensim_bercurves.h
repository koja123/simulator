// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_BERCURVES_H
#define SCENSIM_BERCURVES_H

#include <float.h>
#include <iostream>
#include <string>
#include <map>
#include <set>
#include <memory>
#include "scensim_support.h"

namespace ScenSim {

using std::string;
using std::map;
using std::set;
using std::shared_ptr;
using std::cerr;
using std::endl;

class BitErrorRateCurve {
public:
    BitErrorRateCurve(
        const string& initCurveFamilyName,
        const string& initModeName)
        :
        curveFamilyName(initCurveFamilyName),
        modeName(initModeName),
        lastTableKey(DBL_MIN),
        lastTableValue(0)
    {
        assert(StringIsAllUpperCase(curveFamilyName));
        assert(StringIsAllUpperCase(modeName));
    }

    string GetCurveFamilyName() const { return curveFamilyName; }
    string GetModeName() const { return modeName; }

    void AddDataPoint(const double& snrValue, const double& bitErrorRateForThatSnr);

    double CalculateBitErrorRate(const double& signalToNoiseAndInterferenceRatio) const;

private:
    string curveFamilyName;
    string modeName;

    map<double, double> snrToBerTable;

    double lastTableKey;
    double lastTableValue;

};//BitErrorRateCurve//

inline
void BitErrorRateCurve::AddDataPoint(const double& snrValue, const double& bitErrorRateForThatSnr)
{
    map<double, double>::iterator iter = snrToBerTable.find(snrValue);

    if (iter != snrToBerTable.end()) {
        cerr << "Error: Conflicting data in Bit Error Rate table for SNR value: " << ConvertToDb(snrValue) << endl;
        exit(1);
    }//if//

    snrToBerTable[snrValue] = bitErrorRateForThatSnr;

    if (snrValue > lastTableKey) {
        lastTableKey = snrValue;
        lastTableValue = bitErrorRateForThatSnr;
    }//if//

}//AddDataPoint//


inline
double BitErrorRateCurve::CalculateBitErrorRate(const double& signalToNoiseAndInterferenceRatio) const
{
    typedef map<double, double>::const_iterator IteratorType;

    IteratorType iter = snrToBerTable.lower_bound(signalToNoiseAndInterferenceRatio);
    if (iter == snrToBerTable.end()) {
        return lastTableValue;
    }//if//

    if (iter->first == signalToNoiseAndInterferenceRatio) {
        return (iter->second);
    }//if//


    if (iter == snrToBerTable.begin()) {
        return (iter->second);
    }//if//

    double X2 = iter->first;
    double Y2 = iter->second;
    iter--;
    double X1 = iter->first;
    double Y1 = iter->second;

    // linear interpolation

    return (Y1 + ((Y2-Y1) * ((signalToNoiseAndInterferenceRatio-X1)/(X2-X1))));

}//CalculateBitErrorRate//

//--------------------------------------------------------------------------------------------------

class BlockErrorRateCurve {
public:
    BlockErrorRateCurve(
        const string& initCurveFamilyName,
        const string& initModeName)
        :
        curveFamilyName(initCurveFamilyName),
        modeName(initModeName),
        lastTableKey(DBL_MIN),
        lastTableValue(0)
    {
        assert(StringIsAllUpperCase(curveFamilyName));
        assert(StringIsAllUpperCase(modeName));
    }

    string GetCurveFamilyName() const { return curveFamilyName; }
    string GetModeName() const { return modeName; }
    void AddDataPoint(const double& snrValue, const double& bitErrorRateForThatSnr);

    double CalcBlockErrorRate(const double& signalToNoiseAndInterferenceRatio) const;

private:
    string curveFamilyName;
    string modeName;

    map<double, double> snrToBlerTable;

    double lastTableKey;
    double lastTableValue;

};//BlockErrorRateCurve//



inline
void BlockErrorRateCurve::AddDataPoint(const double& snrValue, const double& blockErrorRateForThatSnr)
{
    assert((blockErrorRateForThatSnr >= 0.0) && (blockErrorRateForThatSnr <= 1.0));

    map<double, double>::iterator iter = snrToBlerTable.find(snrValue);

    if (iter != snrToBlerTable.end()) {
        cerr << "Error: Conflicting data in Block Error Rate table for SNR value: " << snrValue << endl;
        exit(1);
    }//if//

    snrToBlerTable[snrValue] = blockErrorRateForThatSnr;

    if (snrValue > lastTableKey) {
        lastTableKey = snrValue;
        lastTableValue = blockErrorRateForThatSnr;
    }//if//

}//AddDataPoint//


inline
double BlockErrorRateCurve::CalcBlockErrorRate(const double& signalToInterferenceAndNoiseRatio) const
{
    typedef map<double, double>::const_iterator IteratorType;

    IteratorType iter = snrToBlerTable.lower_bound(signalToInterferenceAndNoiseRatio);
    if (iter == snrToBlerTable.end()) {
        return lastTableValue;
    }//if//

    if ((iter == snrToBlerTable.begin()) ||
        (iter->first == signalToInterferenceAndNoiseRatio)) {

        return (iter->second);
    }//if//

    const double x2 = iter->first;
    const double y2 = iter->second;
    iter--;
    const double x1 = iter->first;
    const double y1 = iter->second;

    // linear interpolation

    return (y1 + ((y2-y1) * ((signalToInterferenceAndNoiseRatio-x1)/(x2-x1))));

}//CalculateBlockErrorRate//


//-----------------------------------------------------------------------------

class BitOrBlockErrorRateCurveDatabase {
public:
    BitOrBlockErrorRateCurveDatabase() { }
    BitOrBlockErrorRateCurveDatabase(const string& berFileName);

    void LoadBerCurveFile(const string& berFileName);
    void LoadBlockErrorRateCurveFile(const string& blerFileName);

    shared_ptr<BitErrorRateCurve> GetBerCurve(
        const string& curveFamilyName,
        const string& modeName) const;

    shared_ptr<BlockErrorRateCurve> GetBlockErrorRateCurve(
        const string& curveFamilyName,
        const string& modeName) const;

private:
    typedef std::pair<string, string> FamilyAndModeNameType;

    map<FamilyAndModeNameType, shared_ptr<BitErrorRateCurve> > curveMap;
    map<FamilyAndModeNameType, shared_ptr<BlockErrorRateCurve> > blerCurveMap;

    set<string> loadedFileNames;

    void ParseLine(
        const string& aLine,
        shared_ptr<BitErrorRateCurve>& currentlyBeingReadCurvePtr);

    void ParseLine(
        const string& aLine,
        shared_ptr<BlockErrorRateCurve>& currentlyBeingReadCurvePtr);

    void InsertCurve(const shared_ptr<BitErrorRateCurve>& theCurve);
    void InsertCurve(const shared_ptr<BlockErrorRateCurve>& theCurve);

};//BitOrBlockErrorRateCurveDatabase//



inline
shared_ptr<BitErrorRateCurve> BitOrBlockErrorRateCurveDatabase::GetBerCurve(
    const string& curveFamilyName, const string& modeName) const
{
    typedef map<FamilyAndModeNameType, shared_ptr<BitErrorRateCurve> >::const_iterator MapIteratorType;

    assert(StringIsAllUpperCase(curveFamilyName));
    assert(StringIsAllUpperCase(modeName));

    const FamilyAndModeNameType key(curveFamilyName, modeName);

    MapIteratorType iter = curveMap.find(key);
    if (iter == curveMap.end()) {
        cerr << "Error: Bit Error Rate curve was not found for family: \"" << curveFamilyName
             << "\" and mode: \"" << modeName << endl;
        exit(1);
    }//if//

    return (iter->second);

}//GetBerCurve//



inline
shared_ptr<BlockErrorRateCurve> BitOrBlockErrorRateCurveDatabase::GetBlockErrorRateCurve(
    const string& curveFamilyName, const string& modeName) const
{
    typedef map<FamilyAndModeNameType, shared_ptr<BlockErrorRateCurve> >::const_iterator MapIteratorType;

    assert(StringIsAllUpperCase(curveFamilyName));
    assert(StringIsAllUpperCase(modeName));

    const FamilyAndModeNameType key(curveFamilyName, modeName);

    const MapIteratorType iter = blerCurveMap.find(key);

    if (iter == blerCurveMap.end()) {
        cerr << "Error: Block Error Rate curve was not found for family: \"" << curveFamilyName
             << "\" and mode: \"" << modeName << endl;
        exit(1);
    }//if//

    return (iter->second);

}//GetBlockErrorRateCurve//




inline
BitOrBlockErrorRateCurveDatabase::BitOrBlockErrorRateCurveDatabase(const string& berFileName)
{
    if (berFileName != "") {
        (*this).LoadBerCurveFile(berFileName);
    }//if//
}


}//namespace//

#endif


