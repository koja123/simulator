// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_SUPPORT_H
#define SCENSIM_SUPPORT_H

#include <assert.h>
#include <cmath>
#include "sysstuff.h"
#include <stdlib.h>
#include <limits.h>
#include <float.h>

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <map>
#include <set>
#include <numeric>
#include <complex>

namespace ScenSim {

using std::string;
using std::vector;
using std::cerr;
using std::endl;
using std::abs;


// BAD_SIZE_T only for uninitialized variables only (aid debugging).
// Never use in logic (compare, etc)).

const size_t BAD_SIZE_T = size_t(-1);


inline
unsigned short int ConvertToUShortInt(const unsigned int value)
{
    assert((value <= USHRT_MAX) && "ConvertToUShortInt(): value too big for short int.");
    return (static_cast<unsigned short int>(value));
}


inline
unsigned short int ConvertToUShortInt(const unsigned int value, const string& failureMessage)
{
    if (value > USHRT_MAX) {
        cerr << failureMessage << " : Value = " << value << " > " << USHRT_MAX << endl;
        exit(1);
    }//if//
    return (static_cast<unsigned short int>(value));
}


inline
unsigned char ConvertToUChar(const unsigned int value)
{
    assert((value <= UCHAR_MAX) && "ConvertToUChar(): value too big for char.");
    return (static_cast<unsigned char>(value));
}


inline
unsigned char ConvertToUChar(const unsigned int value, const string& failureMessage)
{
    if (value > UCHAR_MAX) {
        cerr << failureMessage << " : Value = " << value << " > " << UCHAR_MAX << endl;
        exit(1);
    }//if//
    return (static_cast<unsigned char>(value));
}


inline
unsigned int RoundToUint(const double& x)
{
    assert(x >= 0.0);
    return (static_cast<unsigned int>(x + 0.5));
}


inline
unsigned int RoundUpToUint(const double& x)
{
    assert(x >= 0.0);
    return (static_cast<unsigned int>(ceil(x)));
}


inline
int RoundToInt(const double& x)
{
    if (x >= 0.0) {
        return (static_cast<int>(x + 0.5));
    }
    else {
        return (-static_cast<int>(-x + 0.5));
    }//if//
}

inline
unsigned int DivideAndRoundUp(const unsigned int x, const unsigned int y)
{
    return ((x + y - 1) / y);
}


template<typename T> inline
T MinOf3(const T& x1, const T& x2, const T& x3)
{
    using std::min;
    return (min(x1, min(x2,x3)));
}



const double IntegralPowerType_PowerBaseUnitMilliwatts = 1e-13; // (-130dBm)

class IntegralPowerType {
public:
    static double PowerBaseUnitMilliwatts() { return IntegralPowerType_PowerBaseUnitMilliwatts; }

    IntegralPowerType() { powerCustomUnit = 0; }

    IntegralPowerType(const IntegralPowerType& init) { powerCustomUnit = init.powerCustomUnit; }

    void operator=(const IntegralPowerType& right) { powerCustomUnit = right.powerCustomUnit; }

    explicit
    IntegralPowerType(const double& milliwatts)
    {
        assert(milliwatts >= 0.0);
        powerCustomUnit = ConvertToCustomUnits(milliwatts);
    }

    void SetWithMilliwatts(const double& milliwatts)
    {
        assert(milliwatts >= 0.0);
        powerCustomUnit = ConvertToCustomUnits(milliwatts);
    }

    bool operator<(const IntegralPowerType& right) const
        { return (powerCustomUnit < right.powerCustomUnit); }

    bool operator<=(const IntegralPowerType& right) const
        { return (powerCustomUnit <= right.powerCustomUnit); }

    bool operator>(const IntegralPowerType& right) const
        { return (powerCustomUnit > right.powerCustomUnit); }

    bool operator>=(const IntegralPowerType& right) const
        { return (powerCustomUnit >= right.powerCustomUnit); }

    void operator+=(const IntegralPowerType& right) { powerCustomUnit += right.powerCustomUnit; }

    void operator+=(const double& powerToAddMilliwatts)
        { powerCustomUnit += ConvertToCustomUnits(powerToAddMilliwatts); }

    void operator-=(const IntegralPowerType& right)
    {
        assert(powerCustomUnit >= right.powerCustomUnit);
        powerCustomUnit -= right.powerCustomUnit;
    }

    void operator-=(const double& powerToSubMilliwatts)
    {
        IntegralPowerType subtractVal = ConvertToCustomUnits(powerToSubMilliwatts);
        assert(powerCustomUnit >= subtractVal.powerCustomUnit);
        powerCustomUnit -= subtractVal.powerCustomUnit;
    }

    IntegralPowerType operator+(const IntegralPowerType& right) const
        { return IntegralPowerType(powerCustomUnit + right.powerCustomUnit); }

    IntegralPowerType operator-(const IntegralPowerType& right) const
    {
        assert(powerCustomUnit >= right.powerCustomUnit);
        return IntegralPowerType(powerCustomUnit - right.powerCustomUnit);
    }

    double ConvertToMilliwatts() const { return (PowerBaseUnitMilliwatts() * powerCustomUnit); }

private:
    unsigned long long int ConvertToCustomUnits(const double& milliwatts)
    {
        assert(milliwatts >= 0.0);
        return (unsigned long long int)((milliwatts / PowerBaseUnitMilliwatts()) + 0.5);
    }

    unsigned long long int powerCustomUnit;

    IntegralPowerType(const unsigned long long int initValue) { powerCustomUnit = initValue; }

};//IntegralPowerType//

inline
double ConvertToNonDb(const double& dB)
{
    return pow(10.0, (dB/10.0));
}

inline
double ConvertToDb(const double& nonDb)
{
    return (10.0 * log10(nonDb));
}

//  3dB is very close to a factor of 2X and 6db is very close to 4X.

const double ThreeDbFactor = 1.9952623149688796013524553967395;
const double SixDbFactor = 3.9810717055349725077025230508775;

const double PI = 3.141592653589793238462643;

const double SPEED_OF_LIGHT_METERS_PER_SECOND =  299792458;

const double LATITUDE_DEGREES_PER_METER = 1 / (60.0 * 1852.0);
const double METERS_PER_LATITUDE_DEGREE = 60.0 * 1852.0;

const double RADIANS_PER_DEGREE = PI/180.0;


//35.0 + (39/60) + (29.1572/3600)
//139.0 + (44/60) + (28.8759/3600)
const double LATITUDE_ORIGIN_DEGREES = 35.658099222;
const double LONGITUDE_ORIGIN_DEGREES = 139.741354417;

inline
double ConvertYMetersToLatitudeDegrees(
    const double& latitudeOriginDegrees,
    const double& yMeters)
{
    return (latitudeOriginDegrees + (yMeters * LATITUDE_DEGREES_PER_METER));
}


inline
double ConvertXMetersToLongitudeDegrees(
    const double& latitudeOriginDegrees,
    const double& longitudeOriginDegrees,
    const double& xMeters)
{
    return (longitudeOriginDegrees +
            (xMeters * LATITUDE_DEGREES_PER_METER / cos(latitudeOriginDegrees * RADIANS_PER_DEGREE)));
}


static inline
double ConvertLatitudeDegreesToYMeters(
    const double latitudeOriginDegrees,
    const double latitudeDegrees)
{
    return ((latitudeDegrees - latitudeOriginDegrees) * METERS_PER_LATITUDE_DEGREE);
}


static inline
double ConvertLongitudeDegreesToXMeters(
    const double latitudeOriginDegrees,
    const double longitudeOriginDegrees,
    const double longitudeDegrees)
{
    return ((longitudeDegrees - longitudeOriginDegrees) *
            (METERS_PER_LATITUDE_DEGREE * cos(latitudeOriginDegrees * RADIANS_PER_DEGREE)));
}

template<typename T> inline
T CalcInterpolatedValue(
    const double& x1,
    const T& y1,
    const double& x2,
    const T& y2,
    const double x)
{
    if (std::abs(x - x1) < DBL_EPSILON) {
        return y1;
    }//if//

    if (std::abs(x2 - x) < DBL_EPSILON) {
        return y2;
    }//if//

    const double widthX = (x2 - x1);
    assert(widthX > 0.0);

    const double percentWidth = (x - x1) / widthX;
    assert((percentWidth >= 0.0) && (percentWidth <= 1.0));

    const T heightY = (y2 - y1);

    return (y1 + (percentWidth * heightY));

}//CalcInterpolatedValue//


//--------------------------------------------------------------------------------------------------



inline
void DeleteTrailingSpaces(string& aString)
{
    size_t pos = aString.find_last_not_of(" \t");
    if (pos == string::npos) {
        aString.clear();
    }
    else if (pos < (aString.size() - 1)) {
        aString.erase(pos+1);
    }
    else {
        // No trailing spaces => don't modify string.
    }//if//
}//DeleteTrailingSpaces//


inline
bool HasNoTrailingSpaces(const string& aString)
{
    if (aString.length() == 0) {
        return true;
    }//if//

    return (aString[(aString.length() - 1)] != ' ');
}


inline
void ConvertStringToLowerCase(string& aString)
{
    for(size_t i = 0; i < aString.length(); i++) {
        aString[i] = static_cast<unsigned char>(tolower(aString[i]));
    }
}

inline
string MakeLowerCaseString(const string& aString)
{
    string returnString = aString;
    ConvertStringToLowerCase(returnString);
    return (returnString);
}


inline
bool StringIsAllLowerCase(const string& aString)
{
    for(size_t i = 0; i < aString.length(); i++) {
        if (aString[i] != tolower(aString[i])) {
            return false;
        }//if//
    }//for//
    return true;
}//StringIsAllLowerCase//


inline
void ConvertStringToUpperCase(string& aString)
{
    for(size_t i = 0; i < aString.length(); i++) {
        aString[i] = static_cast<unsigned char>(toupper(aString[i]));
    }
}


inline
string MakeUpperCaseString(const string& aString)
{
    string returnString = aString;
    ConvertStringToUpperCase(returnString);
    return (returnString);
}

inline
bool StringIsAllUpperCase(const string& aString)
{
    for(size_t i = 0; i < aString.length(); i++) {
        if (aString[i] != toupper(aString[i])) {
            return false;
        }//if//
    }//for//
    return true;

}//StringIsAllUpperCase//



inline
bool IsEqualCaseInsensitive(const string& left, const string& right)
{
    if (left.length() != right.length()) {
        return false;
    }

    for(size_t i = 0; i < left.length(); i++) {
        if (tolower(left[i]) != tolower(right[i])) {
            return false;
        }//if///
    }//for//

    return true;
}


inline
bool IsLessThanCaseInsensitive(const string& left, const string& right)
{
    size_t i = 0;
    while (true) {
        if (i == left.length()) {
            if (i == right.length()) {
                // Are Equal
                return false;
            }
            else {
                // left is shorter than right.
                return true;
            }//if//
        }//if//

        if (i == right.length()) {
            // right is shorter than left.
            return false;
        }//if//

        if (tolower(left[i]) < tolower(right[i])) {
            return true;
        }
        else if (tolower(left[i]) > tolower(right[i])) {
            return false;
        }
        else {
            // Keep comparing.
        }//if//

        i++;
    }//while//

    assert(false); abort(); return false;

}//IsLessThanCaseInsensitive//


template<typename T> inline
string ConvertToString(const T& aT)
{
    std::ostringstream retStream;
    retStream << aT;
    return retStream.str();
}


inline
void ConvertStringToInt(const string& aString, int& intValue, bool& success)
{
    std::istringstream aStringStream(aString);

    aStringStream >> intValue;

    success = ((!aStringStream.fail()) && (aStringStream.eof()));
}


inline
void ConvertStringToNonNegativeInt(const string& aString, unsigned int& uintValue, bool& success)
{
    int anInt;
    ConvertStringToInt(aString, anInt, success);

    if ((success) && (anInt < 0)) {
        success = false;
        return;
    }//if//
    uintValue = static_cast<unsigned int>(anInt);
}


inline
void ConvertStringToBigInt(const string& aString, long long int& intValue, bool& success)
{
    std::istringstream aStringStream(aString+' ');

    aStringStream >> intValue;

    if (!aStringStream.good()) {
        success = false;
        return;
    }//if//

    char aChar;
    aStringStream.get(aChar);

    if (aChar != ' ') {
        success = false;
        return;
    }//if//

    aStringStream.get(aChar);
    success = aStringStream.eof();

}//ConvertStringToBigInt//



inline
void ConvertStringToDouble(const string& aString, double& doubleValue, bool& success)
{
    char* afterNumber = 0;

    doubleValue = strtod(aString.c_str(), &afterNumber);
    success = (*afterNumber == 0);
}


inline
unsigned char ConvertHexCharacterToHalfByte(const char aChar) {
    assert(isxdigit(aChar));

    if (aChar <= '9') {
        return (aChar - '0');
    }
    else {
        assert(isupper(aChar));
        return ((aChar - 'A') + 0x0A);
    }//if//
}

inline
unsigned char ConvertTwoHexCharactersToByte(const char aChar1, const char aChar2)
{
    return ((ConvertHexCharacterToHalfByte(aChar1) * 16) +  ConvertHexCharacterToHalfByte(aChar2));
}

inline
void ConvertHexStringToByteString(const string& hexString, string& byteString)
{
    assert((hexString.size() % 2) == 0);
    byteString.clear();
    for(size_t i = 0; (i < hexString.size()); i+= 2) {
        byteString.push_back(ConvertTwoHexCharactersToByte(hexString[i], hexString[i+1]));
    }//for//
}

inline
void ConvertHexStringToByteArray(const string& hexString, vector<unsigned char>& byteArray)
{
    assert((hexString.size() % 2) == 0);
    byteArray.clear();
    for(size_t i = 0; (i < hexString.size()); i+= 2) {
        byteArray.push_back(ConvertTwoHexCharactersToByte(hexString[i], hexString[i+1]));
    }//for//
}



inline
char ConvertHalfByteToHexCharacter(const unsigned char aChar) {
    char halfByteToCharMap[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

    assert(aChar <= 15);

    return (halfByteToCharMap[aChar]);
}

inline
void ConvertByteToHexAndAppendToString(const unsigned char aChar, string& aString)
{
    aString.push_back(ConvertHalfByteToHexCharacter(aChar/16));
    aString.push_back(ConvertHalfByteToHexCharacter(aChar%16));
}


inline
void ConvertByteStringToHexString(const string& byteString, string& hexString)
{
    hexString.clear();
    for(size_t i = 0; (i < byteString.size()); i++) {
        ConvertByteToHexAndAppendToString(byteString[i], hexString);
    }//for//
}


inline
void ConvertByteArrayToHexString(
    const unsigned char bytes[], const size_t numberBytes, string& hexString)
{
    hexString.clear();
    for(size_t i = 0; (i < numberBytes); i++) {
        ConvertByteToHexAndAppendToString(bytes[i], hexString);
    }//for//
}


inline
void SuppressLeadingZeros(string& aString)
{
    const size_t originalStringLength = aString.size();
    for(size_t i = 0; i < (originalStringLength - 1); i++) {
        if ((*aString.begin()) == '0') {
            aString.erase(aString.begin());
        }
        else {
            break;
        }//if//
    }//for//
}


inline
void AddLeadingZeros(string& aString, const size_t& expectedLengh)
{
    assert(aString.size() <= expectedLengh);
    const size_t numZeros = expectedLengh - aString.size();
    for(size_t i = 0; i < numZeros; i++) {
        aString.insert(aString.begin(), '0');
    }//for//
}


const char wildcardCharacter = '*';

inline
bool IsEqualWithWildcards(const string& wildString, const string& aString)
{
    size_t i = 0;
    size_t w = 0;
    size_t wildcardIndex = string::npos;
    size_t startMatchIndex = string::npos;

    while (i < aString.length()) {
        assert((aString[i] != wildcardCharacter) && "Only the wildcard string can have wildcards");

        if (w == wildString.length()) {
            // Wildcard string only matches substring of aString, treat as mismatch.

            if (wildcardIndex == string::npos) {
                // No wildcards in wildString and doesn't match.
                return false;
            }//if//

            w = wildcardIndex + 1;
            i = startMatchIndex + 1;
            startMatchIndex = string::npos;
        }//if//

        if (wildString[w] == wildcardCharacter) {
            wildcardIndex = w;
            w++;
            if (w == wildString.length()) {
                // Trailing wildcard matches rest of string.
                return true;
            }//if//
        }
        else {
            if (wildString[w] != aString[i]) {
                if (wildcardIndex == string::npos) {
                    // No wild card found yet and have a mismatch, return not equal.
                    return false;
                }//if//

                if (startMatchIndex == string::npos) {
                    // Failure to match character right after wildcard, keep searching aString.
                    i++;
                }
                else {
                    // Failure to match partial string, back up to right after last wildcard
                    // and try searching further down the non-wildcard string.

                    w = wildcardIndex + 1;
                    i = startMatchIndex + 1;
                    startMatchIndex = string::npos;
                }//if//
            }
            else {
                if (startMatchIndex == string::npos) {
                    startMatchIndex = i;
                }//if//
                w++;
                i++;
            }//if//
        }//if//

    }//while//

    // Exhausted aString, check for more non-wildcard characters in the wildString
    // and if some are found, then the strings do not match.

    while (w < wildString.length()) {
        if (wildString[w] != wildcardCharacter) {
            return false;
        }//if//
        w++;
    }//while//

    return true;

}//IsEqualWithWildcards//


inline
bool IsEqualWithWildcardsCaseInsensitive(const string& wildStringMixedCase, const string& aStringMixedCase)
{
    string wildString = wildStringMixedCase;
    ConvertStringToLowerCase(wildString);

    string aString = aStringMixedCase;
    ConvertStringToLowerCase(aString);

    return (IsEqualWithWildcards(wildString, aString));

}//IsEqualWithWildcards//


template<typename T> inline
void ConvertAStringSequenceOfANumericTypeIntoAVector(
    const string& aString,
    bool& success,
    vector<T>& aVector)
{
    assert(HasNoTrailingSpaces(aString));
    assert((sizeof(T) != sizeof(char)) && "IO function won't work correctly with char types");

    aVector.clear();

    std::istringstream aStringStream(aString);

    while (!aStringStream.eof()) {
        T aValue;
        aStringStream >> aValue;

        if (aStringStream.fail()) {
            success = false;
            return;
        }//if//

        aVector.push_back(aValue);
    }//while//

    success = true;

}//ConvertAStringSequenceOfANumericTypeIntoAVector//



template<> inline
void ConvertAStringSequenceOfANumericTypeIntoAVector<unsigned char>(
    const string& aString,
    bool& success,
    vector<unsigned char>& aVector)
{
    assert(HasNoTrailingSpaces(aString));

    aVector.clear();

    std::istringstream aStringStream(aString);

    while (!aStringStream.eof()) {
        int aValue;
        aStringStream >> aValue;

        if ((aStringStream.fail()) ||
            ((aValue < 0) || (aValue > UCHAR_MAX))) {
            success = false;
            return;
        }//if//

        aVector.push_back(static_cast<unsigned char>(aValue));
    }//while//

    success = true;

}//ConvertAStringSequenceOfANumericTypeIntoAVector//


inline
void ConvertAStringIntoAVectorOfStrings(
    const string& aString,
    bool& success,
    vector<string>& aVector)
{
    assert(HasNoTrailingSpaces(aString));

    aVector.clear();

    std::istringstream aStringStream(aString);

    while (!aStringStream.eof()) {
        string aValue;
        aStringStream >> aValue;

        if (aStringStream.fail()) {
            success = false;
            return;
        }//if//

        aVector.push_back(aValue);
    }//while//

    success = true;

}//ConvertAStringIntoAVectorOfStrings//


inline
void ConvertAStringSequenceOfStringPairsIntoAMap(
    const string& aString,
    bool& success,
    std::map<string, string>& aMap)
{
    assert(HasNoTrailingSpaces(aString));

    success = false;
    aMap.clear();

    std::istringstream aStringStream(aString);

    while (!aStringStream.eof()) {
        string aPair;
        aStringStream >> aPair;
        if (aStringStream.fail()) {
            return;
        }//if//

        const size_t colonPos = aPair.find(':');
        if (colonPos == string::npos) {
            return;
        }//if//

        // Check for two ":"
        const size_t secondColonPos = aPair.find(':', colonPos+1);
        if (secondColonPos != string::npos) {
            return;
        }//if//

        const string aKey = aPair.substr(0, colonPos);
        const string aValue = aPair.substr(colonPos+1);

        if ((aKey.empty()) || (aValue.empty())) {
            return;
        }//if//

        if (aMap.find(aKey) != aMap.end()) {
            return;
        }//if//

        aMap[aKey] = aValue;

    }//while//

    success = true;

}//ConvertAStringSequenceOfNumericPairsIntoAMap//




template<typename K, typename T> inline
void ConvertAStringSequenceOfNumericPairsIntoAMap(
    const string& aString,
    bool& success,
    std::map<K, T>& aMap)
{
    assert(HasNoTrailingSpaces(aString));

    aMap.clear();

    std::istringstream aStringStream(aString);

    while (!aStringStream.eof()) {
        K aKey;
        aStringStream >> aKey;
        char aChar;
        aStringStream >> aChar;
        if ((aStringStream.fail()) || (aChar != ':')) {
            success = false;
            return;
        }//if//
        T aValue;
        aStringStream >> aValue;
        if (aStringStream.fail()) {
            success = false;
            return;
        }//if//

        if (aMap.find(aKey) != aMap.end()) {
            success = false;
            return;
        }//if//

        aMap[aKey] = aValue;
    }//while//

    success = true;

}//ConvertAStringSequenceOfNumericPairsIntoAMap//


template<typename T> inline
void ConvertAStringSequenceOfRealValuedPairsIntoAMap(
    const string& aString,
    bool& success,
    std::map<T, T>& aMap)
{
    ConvertAStringSequenceOfNumericPairsIntoAMap<T,T>(aString, success, aMap);
}



inline
bool IsAConfigFileCommentLine(const string& aLine)
{
    if ((aLine.length() == 0) || (aLine.at(0) == '#')) {
        return true;
    }//if//

    const size_t position = aLine.find_first_not_of(' ');

    if (position == string::npos) {
        // No non-space characters found. Blank line (is comment).

        return true;
    }//if//

    if (aLine[position] == '#') {
        return true;
    }
    else if (aLine[position] == '\t') {
        cerr << "Error: No Tabs allowed in config files." << endl;
        cerr << "Line:" << aLine << endl;
        exit(1);
    }//if//

    return false;

}//IsAConfigFileCommentLine//



inline
bool IsAConfigFileContinuationLine(const string& aLine)
{
    const size_t position = aLine.find_first_not_of(' ');
    if (position == string::npos) {
        return false;
    }//if//

    return ((aLine[position] == '"') && (aLine.back() == '"'));
}


//--------------------------------------------------------------------------------------------------


inline
unsigned int RoundUpToNearestIntDivisibleBy8(const unsigned int aSize)
{
    return (((aSize+7) / 8) * 8);
}

inline
unsigned int RoundUpToNearestIntDivisibleBy4(const unsigned int aSize)
{
    return (((aSize+3) / 4) * 4);
}


class SingleSizeMemoryCache {
public:
    SingleSizeMemoryCache() {itemSize = 0; freeListPtr = 0; }
    void  Allocate(size_t sizeBytes, void*& ptr);
    void  FlushCache();
    ~SingleSizeMemoryCache() { this->FlushCache(); }

    template<typename T> void Delete(T*& ptr);

private:
    size_t itemSize;
    struct LinkType { LinkType* nextPtr;};
    LinkType* freeListPtr;

    void Free(void*& ptr, size_t sizeBytes);

    // Disabled:

    SingleSizeMemoryCache(const SingleSizeMemoryCache&);
    void operator=(const SingleSizeMemoryCache&);

};//SingleSizeMemoryCache//


inline
void SingleSizeMemoryCache::Allocate(size_t sizeBytes, void*& ptr)
{
    if (itemSize == 0) {
       assert(sizeBytes >= sizeof(LinkType*));
       itemSize = sizeBytes;
    }//if//

    assert(sizeBytes == itemSize);

    if (freeListPtr == 0) {
       ptr = ::operator new(itemSize);
    }
    else {
       ptr = freeListPtr;
       freeListPtr = freeListPtr->nextPtr;
    }//if//

}//Allocate//

inline
void SingleSizeMemoryCache::Free(void*& ptr, size_t sizeBytes)
{
    assert(sizeBytes == itemSize);
    reinterpret_cast<LinkType*>(ptr)->nextPtr = freeListPtr;
    freeListPtr = reinterpret_cast<LinkType*>(ptr);
    ptr = 0;
}//Free//


inline
void SingleSizeMemoryCache::FlushCache()
{
    while (freeListPtr != 0) {
       void* deletePtr = freeListPtr;
       freeListPtr = freeListPtr->nextPtr;
       operator delete(deletePtr);
    }//while//

}//FlushCache//



template<class T> inline
void SingleSizeMemoryCache::Delete(T*& ptr)
{
    if (ptr != 0) {
        ptr->~T();
        void* voidPtr = ptr;
        (*this).Free(voidPtr, sizeof(T));
        ptr = 0;
    }//if//

}//Delete//



class TransparentSingleSizeMemoryCache {
public:
    TransparentSingleSizeMemoryCache() {itemSize = 0; freeListPtr = 0; }
    void  Allocate(size_t sizeBytes, void*& ptr);
    void  FlushCache();
    ~TransparentSingleSizeMemoryCache() { this->FlushCache(); }

    static void Delete(void*& ptr, size_t sizeBytes);

private:
    size_t itemSize;
    struct LinkType { LinkType* nextPtr;};
    LinkType* freeListPtr;

    void Free(void*& ptr, size_t sizeBytes);

    // Disabled:

    TransparentSingleSizeMemoryCache(const TransparentSingleSizeMemoryCache&);
    void operator=(const TransparentSingleSizeMemoryCache&);

};//TransparentSingleSizeMemoryCache//



inline
void TransparentSingleSizeMemoryCache::Allocate(size_t sizeBytes, void*& ptr)
{
    if (itemSize == 0) {
       itemSize = sizeBytes;
    }//if//

    assert((sizeBytes == itemSize));

    if (freeListPtr == 0) {

        unsigned char * bytePtr =
            reinterpret_cast<unsigned char*>(::operator new(MinimumMemoryAlignmentBytes + itemSize));

        // Place a pointer to this Cache in header (overhead) of allocated memory block.

        *reinterpret_cast<TransparentSingleSizeMemoryCache**>(bytePtr) = this;

        ptr = &bytePtr[MinimumMemoryAlignmentBytes];
    }
    else {
        ptr = freeListPtr;
        freeListPtr = freeListPtr->nextPtr;
    }//if//

}//Allocate//


inline
void TransparentSingleSizeMemoryCache::Free(void*& ptr, size_t sizeBytes)
{
   assert((sizeBytes == itemSize));
   reinterpret_cast<LinkType*>(ptr)->nextPtr = freeListPtr;
   freeListPtr = reinterpret_cast<LinkType*>(ptr);
   ptr = 0;
}//Free//



inline
void TransparentSingleSizeMemoryCache::Delete(void*& ptr, size_t sizeBytes)
{
    if (ptr != 0) {

        // Pull out the memory cache pointer out of header (overhead).

        unsigned char * headerMemoryPtr = reinterpret_cast<unsigned char*>(ptr) - MinimumMemoryAlignmentBytes;

        TransparentSingleSizeMemoryCache& theCache =
            **reinterpret_cast<TransparentSingleSizeMemoryCache**>(headerMemoryPtr);

        theCache.Free(ptr, sizeBytes);

    }//if//

}//Delete//


inline
void TransparentSingleSizeMemoryCache::FlushCache()
{
    while (freeListPtr != 0) {
        void* blockPtr = reinterpret_cast<unsigned char*>(freeListPtr) - MinimumMemoryAlignmentBytes;
        freeListPtr = freeListPtr->nextPtr;
        ::operator delete(blockPtr);
    }//while//

}//FlushCache//


//--------------------------------------------------------------------------------------------------

class InterpolatedTable {
public:
    InterpolatedTable() { }

    template<size_t arraySize>
    InterpolatedTable(const std::array<std::array<double, 2>, arraySize>& initArray)
    {
        for(unsigned int i = 0; (i < initArray.size()); i++) {
            assert(!DatapointIsInTable(initArray[i][0]));
            AddDatapoint(initArray[i][0], initArray[i][1]);
        }//for//
    }

    bool HasData() const { return (!theTable.empty()); }

    double MinDomainValue() const { assert(HasData()); return (theTable.begin()->first); }
    double MaxDomainValue() const { assert(HasData()); return (theTable.rbegin()->first); }

    bool DatapointIsInTable(const double& domainValue) const
    {
        return (theTable.find(domainValue) != theTable.end());
    }

    void AddDatapoint(const double& domainValue, const double rangeValue)
    {
        theTable[domainValue] = rangeValue;
    }

    double CalcValue(const double& domainValue) const;

private:
    std::map<double, double> theTable;

};//InterpolatedTable//


inline
double InterpolatedTable::CalcValue(const double& domainValue) const
{
    typedef std::map<double, double>::const_iterator IteratorType;

    IteratorType iter = theTable.lower_bound(domainValue);

    if (iter == theTable.begin()) {
        return (iter->second);
    }//if//

    if (iter == theTable.end()) {
        assert(theTable.size() > 0);
        --iter;
        return (iter->second);
    }//if//

    if (iter->first == domainValue) {
        // Perfect match.
        return (iter->second);
    }//if//

    const double x2 = iter->first;
    const double y2 = iter->second;
    iter--;
    const double x1 = iter->first;
    const double y1 = iter->second;

    assert((x1 < domainValue) && (domainValue < x2));
    // linear interpolation

    return (y1 + ((y2-y1) * ((domainValue-x1) / (x2-x1))) );

}//CalcValue//




//--------------------------------------------------------------------------------------------------



inline
double CalculateThermalNoisePowerWattsPerHz(const double& theRadioNoiseFactor)
{
    const double boltzmannsConstant = 1.379e-23;
    const double temperatureK = 290.0;
    return (boltzmannsConstant * temperatureK * theRadioNoiseFactor);
}


// Note: The above function uses "Noise Factor" and the next function uses "Noise FIGURE" (dB)
//       for efficiency reasons.


inline
double CalculateThermalNoisePowerWatts(
    const double& theRadioNoiseFigureDb,
    const double& theChannelBandwidthMhz)
{
    const double theRadioNoiseFactor = ConvertToNonDb(theRadioNoiseFigureDb);
    const double theChannelBandwidthHz = 1000000.0 * theChannelBandwidthMhz;

    return (theChannelBandwidthHz * CalculateThermalNoisePowerWattsPerHz(theRadioNoiseFactor));
}



//--------------------------------------------------------------------------------------------------

inline
bool SequenceNumberIsLessThan(const unsigned int sequenceNum1, const unsigned int sequenceNum2)
{
    return
        (((sequenceNum1 < sequenceNum2) && ((sequenceNum2 - sequenceNum1) <= (UINT_MAX/2))) ||
        ((sequenceNum1 > sequenceNum2) && ((sequenceNum1 - sequenceNum2) > (UINT_MAX/2))));
}

inline
bool SequenceNumberIsLessThan(
    const unsigned short int sequenceNum1, const unsigned short int sequenceNum2)
{
    return
        (((sequenceNum1 < sequenceNum2) && ((sequenceNum2 - sequenceNum1) <= (USHRT_MAX/2))) ||
        ((sequenceNum1 > sequenceNum2) && ((sequenceNum1 - sequenceNum2) > (USHRT_MAX/2))));
}


const unsigned short int TwelveBitSequenceNumberMax = 4095;
const unsigned short int TwelveBitSequenceNumberMaxPlus1 = (TwelveBitSequenceNumberMax + 1);
const unsigned short int InvalidTwelveBitSequenceNumber = TwelveBitSequenceNumberMaxPlus1;


inline
bool TwelveBitSequenceNumberIsLessThan(
    const unsigned short int sequenceNum1,
    const unsigned short int sequenceNum2)
{
    assert(sequenceNum1 <= TwelveBitSequenceNumberMax);
    assert(sequenceNum2 <= TwelveBitSequenceNumberMax);

    return
        (((sequenceNum1 < sequenceNum2) && ((sequenceNum2 - sequenceNum1) <= (TwelveBitSequenceNumberMax/2))) ||
        ((sequenceNum1 > sequenceNum2) && ((sequenceNum1 - sequenceNum2) > (TwelveBitSequenceNumberMax/2))));
}


inline
int CalcTwelveBitSequenceNumberDifference(
    const unsigned short int sequenceNum1,
    const unsigned short int sequenceNum2)
{
    assert(sequenceNum1 <= TwelveBitSequenceNumberMax);
    assert(sequenceNum2 <= TwelveBitSequenceNumberMax);

    if (TwelveBitSequenceNumberIsLessThan(sequenceNum1, sequenceNum2)) {
        if (sequenceNum1 <= sequenceNum2) {
            return (static_cast<int>(sequenceNum1) - static_cast<int>(sequenceNum2));
        }
        else {
            return ((static_cast<int>(sequenceNum1) - TwelveBitSequenceNumberMaxPlus1) -
                    static_cast<int>(sequenceNum2));
        }//if//
    }
    else {
        if (sequenceNum1 >= sequenceNum2) {
            return (static_cast<int>(sequenceNum1) - static_cast<int>(sequenceNum2));
        }
        else {
            return (static_cast<int>(sequenceNum1) +
                    (TwelveBitSequenceNumberMaxPlus1 - static_cast<int>(sequenceNum2)));
        }//if//
    }//if//

}//CalcTwelveBitSequenceNumberDifference//



inline
unsigned short int SubtractTwelveBitSequenceNumbers(
    const unsigned short int sequenceNum1,
    const unsigned short int sequenceNum2)
{
    assert(sequenceNum1 <= TwelveBitSequenceNumberMax);
    assert(sequenceNum2 <= TwelveBitSequenceNumberMax);

    assert((sequenceNum1 == sequenceNum2) ||
           (TwelveBitSequenceNumberIsLessThan(sequenceNum2, sequenceNum1)));

    if (sequenceNum1 < sequenceNum2) {
        return (sequenceNum1 + TwelveBitSequenceNumberMaxPlus1 - sequenceNum2);
    }
    else {
        return (sequenceNum1 - sequenceNum2);
    }//if//

}//CalcTwelveBitSequenceNumberDistance//


inline
void IncrementTwelveBitSequenceNumber(unsigned short int& sequenceNum)
{
    assert(sequenceNum <= TwelveBitSequenceNumberMax);

    sequenceNum++;
    if (sequenceNum > TwelveBitSequenceNumberMax) {
        sequenceNum = 0;
    }//if//
}


inline
void DecrementTwelveBitSequenceNumber(unsigned short int& sequenceNum)
{
    assert(sequenceNum <= TwelveBitSequenceNumberMax);

    if (sequenceNum == 0) {
        sequenceNum = TwelveBitSequenceNumberMax;
    }
    else {
        sequenceNum--;
    }//if//
}


inline
unsigned short int TwelveBitSequenceNumberPlus1(const unsigned short int sequenceNum)
{
    unsigned short int retVal = sequenceNum;
    IncrementTwelveBitSequenceNumber(retVal);
    return (retVal);
}





//--------------------------------------------------------------------------------------------------

template<typename T>
class MovingAverageCalculator {
public:
    MovingAverageCalculator(const unsigned int windowSize);

    void Reset() { numDatapoints = 0; nextDatapointIndex = 0; }

    void AddDatapoint(const T& value);

    bool AverageExists() const { return (numDatapoints > 0); }

    double GetAverage() const;

private:

    vector<T> datapoints;
    T currentTotal;
    unsigned int numDatapoints;
    unsigned int nextDatapointIndex;

};//MovingAverageCalculator//


template<typename T> inline
MovingAverageCalculator<T>::MovingAverageCalculator(const unsigned int windowSize)
    :
    datapoints(windowSize),
    numDatapoints(0),
    nextDatapointIndex(0),
    currentTotal(0)
{
}

template<typename T> inline
void MovingAverageCalculator<T>::AddDatapoint(const T& value)
{
    currentTotal += value;

    if (numDatapoints < datapoints.size()) {
        numDatapoints++;
    }
    else {
        currentTotal -= datapoints[nextDatapointIndex];
    }//if//

    datapoints[nextDatapointIndex] = value;

    nextDatapointIndex++;
    if (nextDatapointIndex == datapoints.size()) {
        nextDatapointIndex = 0;
    }//if//
}//AddDatapoint//



template<typename T> inline
double MovingAverageCalculator<T>::GetAverage() const
{
    assert(numDatapoints > 0);

    return (static_cast<double>(currentTotal) / numDatapoints);

}//GetAverage//

//--------------------------------------------------------------------------------------------------

template<typename T>
class ExponentialMovingAverageCalculator {
public:
    ExponentialMovingAverageCalculator(
        const double& smoothingFactor,
        const bool useSimpleAverageAtStartup = true);

    void Reset() { numDatapoints = 0; currentAverage = 0.0; }

    void AddDatapoint(const T& value);

    bool AverageExists() const { return (numDatapoints > 0); }

    double GetAverage() const { assert(AverageExists()); return (currentAverage); }

private:

    double smoothingFactor;
    unsigned int startDatapointNum;

    T currentAverage;
    unsigned int numDatapoints;

};//ExponentialMovingAverageCalculator//


template<typename T> inline
ExponentialMovingAverageCalculator<T>::ExponentialMovingAverageCalculator(
    const double& initSmoothingFactor,
    const bool useSimpleAverageAtStartup)
    :
    smoothingFactor(initSmoothingFactor),
    startDatapointNum(1),
    numDatapoints(0),
    currentAverage(0.0)
{
    if (useSimpleAverageAtStartup) {
        startDatapointNum = RoundUpToUint(1.0 / smoothingFactor);
    }//if//
}


template<typename T> inline
void ExponentialMovingAverageCalculator<T>::AddDatapoint(const T& value)
{
    if (numDatapoints < startDatapointNum) {
        if (numDatapoints > 0) {
            (*this).currentAverage *= numDatapoints;
            (*this).currentAverage += value;
            (*this).numDatapoints++;
            (*this).currentAverage *= (1.0 / numDatapoints);
        }
        else {
            (*this).currentAverage = value;
            (*this).numDatapoints = 1;
        }//if//
    }
    else {
        (*this).currentAverage =
            (((1.0 - smoothingFactor) * currentAverage) + (value * smoothingFactor));
    }//if//

}//AddDatapoint//


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------


inline
const vector<double> MakeExponentialMovingAvgBucketWeights(
    const unsigned int numberBuckets,
    const double& smoothingFactor)
{
    assert(numberBuckets > 0);
    assert((smoothingFactor > 0.0) && (smoothingFactor < 1.0));

    vector<double> weights(numberBuckets);

    for(unsigned int i = 0; (i < weights.size()); i++) {
        weights[i] = pow((1.0-smoothingFactor), static_cast<double>(i));
    }//for//

    return (weights);

}//MakeExponentialMovingAvgBucketWeights//


//--------------------------------------------------------------------------------------------------

template<typename T>
class TimeWindowedAndWeightedMovingAvgCalculator {
public:
    TimeWindowedAndWeightedMovingAvgCalculator(
        const T& timeBucketDuration,
        const unsigned int numberTimeBuckets,
        const vector<double>& bucketWeights);

    void ClearData();

    void AddDatapoint(const T& time, const double& value);

    void AdvanceTimeTo(const T& time);

    bool AverageExists() const { return ((numBucketsWithData > 0) || (currentBucketNumDatapoints > 0)); }

    double GetAverage() const;

private:
    T timeBucketDuration;
    vector<double> bucketWeights;

    vector<double> bucketValues;
    vector<bool> bucketHasData;
    unsigned int numBucketsWithData;

    unsigned int currentBucketIndex;
    T nextBucketStartTime;

    unsigned int currentBucketNumDatapoints;
    double currentBucketTotal;

    double currentAverage;

    T checkTime;
    bool checkAdvanceTimeHasBeenCalled;

    void CompleteCurrentBucketAndGotoNewOne(const T& time);

};//TimeWindowedAndWeightedMovingAvgCalculator//



template<typename T> inline
TimeWindowedAndWeightedMovingAvgCalculator<T>::TimeWindowedAndWeightedMovingAvgCalculator(
    const T& initTimeBucketDuration,
    const unsigned int numberTimeBuckets,
    const vector<double>& initBucketWeights)
    :
    timeBucketDuration(initTimeBucketDuration),
    bucketWeights(initBucketWeights),
    numBucketsWithData(0),
    bucketValues(numberTimeBuckets, 0.0),
    bucketHasData(numberTimeBuckets, false),
    currentBucketIndex(0),
    nextBucketStartTime(0),
    currentBucketNumDatapoints(0),
    currentBucketTotal(0),
    checkAdvanceTimeHasBeenCalled(false)
{
    assert(numberTimeBuckets == bucketWeights.size());

    // Normalize Bucket Weights.

    const double total = std::accumulate(bucketWeights.begin(), bucketWeights.end(), 0.0);
    assert(total > 0.0);

    for(unsigned int i = 0; (i < bucketWeights.size()); i++) {
        assert(bucketWeights[i] > 0.0);
        bucketWeights[i] *= (1.0/total);
    }//for//

}//TimeWindowedAndWeightedMovingAvgCalculator//


template<typename T> inline
void TimeWindowedAndWeightedMovingAvgCalculator<T>::ClearData()
{
    (*this).numBucketsWithData = 0;
    std::fill(bucketHasData.begin(), bucketHasData.end(), false);
    (*this).nextBucketStartTime = 0;
    (*this).currentBucketIndex = 0;
    (*this).currentBucketNumberDatapoints = 0;
    (*this).currentBucketTotal = 0;
    (*this).checkAdvanceTimeHasBeenCalled = false;
    (*this).checkTime = 0;

}//ClearData//


template<typename T>inline
void TimeWindowedAndWeightedMovingAvgCalculator<T>::CompleteCurrentBucketAndGotoNewOne(const T& time)
{
    // Complete current Bucket.

    if (currentBucketNumDatapoints > 0) {

        (*this).bucketValues[currentBucketIndex] = (currentBucketTotal / currentBucketNumDatapoints);

        if (!bucketHasData[currentBucketIndex]) {
            (*this).numBucketsWithData++;
        }//if//
        (*this).bucketHasData[currentBucketIndex] = true;

        (*this).currentBucketNumDatapoints = 0;
        (*this).currentBucketTotal = 0.0;
    }
    else {
        if (bucketHasData[currentBucketIndex]) {
            (*this).numBucketsWithData--;
        }//if//

        (*this).bucketHasData[currentBucketIndex] = false;
        (*this).bucketValues[currentBucketIndex] = 0.0;
    }//if//

    if (numBucketsWithData == 0) {
        (*this).nextBucketStartTime = time + timeBucketDuration;
        (*this).currentBucketIndex = 0;
    }
    else {
        do {
            (*this).nextBucketStartTime = nextBucketStartTime + timeBucketDuration;
            (*this).currentBucketIndex++;
            if (currentBucketIndex >= bucketValues.size()) {
                (*this).currentBucketIndex = 0;
            }//if//

            if (bucketHasData[currentBucketIndex]) {
                (*this).numBucketsWithData--;
            }//if//

            (*this).bucketHasData[currentBucketIndex] = false;
            (*this).bucketValues[currentBucketIndex] = 0.0;

        }/*do*/ while (time >= nextBucketStartTime);
    }//if//

}//CompleteCurrentBucket//



template<typename T> inline
void TimeWindowedAndWeightedMovingAvgCalculator<T>::AddDatapoint(const T& time, const double& value)
{
    assert(time >= checkTime);
    (*this).checkTime = time;
    (*this).checkAdvanceTimeHasBeenCalled = false;

    if (time >= nextBucketStartTime) {

        (*this).CompleteCurrentBucketAndGotoNewOne(time);

    }//if//

    (*this).currentBucketTotal += value;
    (*this).currentBucketNumDatapoints++;

}//AddDatapoint//


template<typename T> inline
void TimeWindowedAndWeightedMovingAvgCalculator<T>::AdvanceTimeTo(const T& time)
{
    assert(time >= checkTime);
    (*this).checkTime = time;
    (*this).checkAdvanceTimeHasBeenCalled = true;

    double total = 0.0;
    double totalWeight = 0.0;

    if (time < nextBucketStartTime) {
        if (currentBucketNumDatapoints > 0) {
            total += bucketWeights[0] * (currentBucketTotal / currentBucketNumDatapoints);
            totalWeight += bucketWeights[0];
        }//if//
    }
    else {
        (*this).CompleteCurrentBucketAndGotoNewOne(time);
    }//if//

    unsigned int dataIndex = currentBucketIndex;
    for(unsigned int i = 1; (i < bucketWeights.size()); i++) {
        if (dataIndex != 0) {
            dataIndex--;
        }
        else {
            dataIndex = bucketValues.size() - 1;
        }//if//

        if (bucketHasData[dataIndex]) {
            total += bucketWeights[i] * bucketValues[dataIndex];
            totalWeight += bucketWeights[i];
        }//if//
    }//for//

    if (totalWeight > 0.0) {
        (*this).currentAverage = (total / totalWeight);
    }//if//

}//AdvanceTimeTo//


template<typename T> inline
double TimeWindowedAndWeightedMovingAvgCalculator<T>::GetAverage() const
{
    assert(checkAdvanceTimeHasBeenCalled);
    assert(AverageExists());
    return (currentAverage);

}//GetAverage//



//--------------------------------------------------------------------------------------

template<typename SequenceNumberType>
class DuplicateSequenceNumberDetector {
public:
    DuplicateSequenceNumberDetector() : isInitialized(false) { }

    void Clear();

    void SetPacketAsReceived(const SequenceNumberType sequenceNumber);

    bool PacketIsADuplicate(const SequenceNumberType sequenceNumber) const;

    void GarbageCollectSequenceNumbersBefore(const SequenceNumberType sequenceNumber);

public:
    bool isInitialized;
    SequenceNumberType highestSeenSequenceNumber;
    std::set<SequenceNumberType> setOfUnseenSequenceNumbers;

};//DuplicateSequenceNumberDetector//


template<typename SequenceNumberType> inline
void DuplicateSequenceNumberDetector<SequenceNumberType>::Clear()
{
    isInitialized = false;
    setOfUnseenSequenceNumbers.clear();
}


template<typename SequenceNumberType > inline
void DuplicateSequenceNumberDetector<SequenceNumberType>::SetPacketAsReceived(
    const SequenceNumberType sequenceNumber)
{
    if (!isInitialized) {
        assert((sequenceNumber != 0) && "Sequence Numbers must start at 1");
        (*this).highestSeenSequenceNumber = 0;
        (*this).isInitialized = true;
    }//if//

    if (SequenceNumberIsLessThan(sequenceNumber, highestSeenSequenceNumber)) {
        typename std::set<SequenceNumberType>::iterator iter =
            setOfUnseenSequenceNumbers.find(sequenceNumber);

        if (iter != setOfUnseenSequenceNumbers.end()) {
            setOfUnseenSequenceNumbers.erase(iter);
        }//if//
    }
    else {
        if (sequenceNumber > highestSeenSequenceNumber) {

            for(SequenceNumberType i = (highestSeenSequenceNumber+1); (i != sequenceNumber); i++) {
                setOfUnseenSequenceNumbers.insert(i);
            }//for//
        }
        else {
            for(SequenceNumberType i = (highestSeenSequenceNumber+1); (i != 0); i++) {
                setOfUnseenSequenceNumbers.insert(i);
            }//for//

            for(SequenceNumberType i = 0; (i != sequenceNumber); i++) {
                setOfUnseenSequenceNumbers.insert(i);
            }//for//
        }//if//

        highestSeenSequenceNumber = sequenceNumber;

    }//if//

}//SetPacketAsReceived//



template<typename SequenceNumberType> inline
bool DuplicateSequenceNumberDetector<SequenceNumberType>::PacketIsADuplicate(
    const SequenceNumberType sequenceNumber) const
{
    if (!isInitialized) {
        return false;
    }//if//

    if (sequenceNumber == highestSeenSequenceNumber) {
        return true;
    }//if//


    if (SequenceNumberIsLessThan(highestSeenSequenceNumber, sequenceNumber)) {
        return false;
    }//if//

    return (setOfUnseenSequenceNumbers.find(sequenceNumber) ==
            setOfUnseenSequenceNumbers.end());

}//PacketIsADuplicate//


template<typename SequenceNumberType> inline
void DuplicateSequenceNumberDetector<SequenceNumberType>::GarbageCollectSequenceNumbersBefore(
    const SequenceNumberType sequenceNumber)
{
    // Brute force algorithm.

    typedef typename std::set<SequenceNumberType>::iterator IterType;

    IterType iter = setOfUnseenSequenceNumbers.begin();
    while (iter != setOfUnseenSequenceNumbers.end()) {
        if (SequenceNumberIsLessThan(*iter, sequenceNumber)) {
            IterType deleteIter = iter;
            ++iter;
            setOfUnseenSequenceNumbers.erase(deleteIter);
        }
        else {
            ++iter;
        }//for//
    }//while//

}//GarbageCollectSequenceNumbersBefore//

//--------------------------------------------------------------------------------------

template<typename SeqNumType>
class WindowedDuplicateSequenceNumberDetector {
public:
    WindowedDuplicateSequenceNumberDetector(const unsigned int sequenceNumberWindowSize);
    void SetAsSeen(const SeqNumType sequenceNumber);
    bool IsInSequenceNumberWindow(const SeqNumType sequenceNumber) const;
    bool IsDuplicate(const SeqNumType sequenceNumber) const;
private:
    SeqNumType largestSeenSequenceNumber;

    vector<bool> hasBeenSeenRing;

};//WindowedDuplicateSequenceNumberDetector//


template<typename SeqNumType> inline
WindowedDuplicateSequenceNumberDetector<SeqNumType>::WindowedDuplicateSequenceNumberDetector(
    const unsigned int sequenceNumberWindowSize)
    :
    largestSeenSequenceNumber(0),
    hasBeenSeenRing(sequenceNumberWindowSize, false)
{
}


template<typename SeqNumType> inline
bool WindowedDuplicateSequenceNumberDetector<SeqNumType>::IsInSequenceNumberWindow(
    const SeqNumType sequenceNumber) const
{
    return ((sequenceNumber + hasBeenSeenRing.size()) > largestSeenSequenceNumber);
}



template<typename SeqNumType> inline
void WindowedDuplicateSequenceNumberDetector<SeqNumType>::SetAsSeen(const SeqNumType sequenceNumber)
{
    assert(IsInSequenceNumberWindow(sequenceNumber));

    if (sequenceNumber > largestSeenSequenceNumber) {
        const size_t numToClear = (sequenceNumber - largestSeenSequenceNumber);
        (*this).largestSeenSequenceNumber = sequenceNumber;
        const size_t largestIndex = (sequenceNumber % hasBeenSeenRing.size());
        size_t index = largestIndex + 1;
        for (size_t i = 0; (i < numToClear); i++) {
            if (index >= hasBeenSeenRing.size()) {
                index = 0;
            }//if//

            (*this).hasBeenSeenRing[index] = false;
            index++;
        }//for//

        (*this).hasBeenSeenRing[largestIndex] = true;
    }
    else {
        (*this).hasBeenSeenRing[(sequenceNumber % hasBeenSeenRing.size())] = true;
    }//if//

}//SetAsSeen//


template<typename SeqNumType> inline
bool WindowedDuplicateSequenceNumberDetector<SeqNumType>::IsDuplicate(const SeqNumType sequenceNumber) const
{
    assert(IsInSequenceNumberWindow(sequenceNumber));

    if (sequenceNumber > largestSeenSequenceNumber) {
        return false;
    }//if//

    return (hasBeenSeenRing[(sequenceNumber % hasBeenSeenRing.size())]);

}//IsDuplicate//



//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------

// Convenience types for inserting empty space in packet header formats.
// This is for header bytes that are not used by the model because the particular
// features are not implemented and helps insure packets do not contain random
// garbage.

struct OneZeroedByteType {
    unsigned char notUsedByte;
    OneZeroedByteType() : notUsedByte(0) {}
};

struct TwoZeroedBytesType {
    unsigned char notUsedByte1;
    unsigned char notUsedByte2;
    TwoZeroedBytesType()
        : notUsedByte1(0), notUsedByte2(0) {}
};

struct ThreeZeroedBytesType {
    unsigned char notUsedByte1;
    unsigned char notUsedByte2;
    unsigned char notUsedByte3;
    ThreeZeroedBytesType()
        : notUsedByte1(0), notUsedByte2(0), notUsedByte3(0) {}
};

struct FourZeroedBytesType {
    unsigned char notUsedByte1;
    unsigned char notUsedByte2;
    unsigned char notUsedByte3;
    unsigned char notUsedByte4;
    FourZeroedBytesType()
        : notUsedByte1(0), notUsedByte2(0), notUsedByte3(0), notUsedByte4(0) {}
};

struct FiveZeroedBytesType {
    TwoZeroedBytesType notUsed1;
    ThreeZeroedBytesType notUsed2;
};


struct SixZeroedBytesType {
    unsigned char notUsedByte1;
    unsigned char notUsedByte2;
    unsigned char notUsedByte3;
    unsigned char notUsedByte4;
    unsigned char notUsedByte5;
    unsigned char notUsedByte6;

    SixZeroedBytesType()
        : notUsedByte1(0), notUsedByte2(0), notUsedByte3(0), notUsedByte4(0),
          notUsedByte5(0), notUsedByte6(0) {}
};


struct EightZeroedBytesType {
    FourZeroedBytesType notUsedBytes1;
    FourZeroedBytesType notUsedBytes2;
};


//--------------------------------------------------------------------------------------

// ByteAlignedUIntType is for imbedding ints into plain structs for packet headers
// without worrying about alignment issues.

template<typename T>
class ByteAlignedUIntType {
public:
    ByteAlignedUIntType() { std::fill(theBytes.begin(), theBytes.end(), 0); }

    void Set(const T& value);
    T Get() const;

private:
    std::array<unsigned char, sizeof(T)> theBytes;

};//ByteAlignedUIntType//


template<typename T> inline
void ByteAlignedUIntType<T>::Set(const T& value)
{
    assert(value >= 0);
    T x = value;
    for(unsigned int i = 0; (i < theBytes.size()); i++) {
        theBytes[i] = static_cast<unsigned char>(x);
        x /= 256;
    }//for//
}

template<typename T> inline
T ByteAlignedUIntType<T>::Get() const
{
    T ret = 0;
    T mult = 1;
    for(unsigned int i = 0; (i < theBytes.size()); i++) {
        ret += (mult * theBytes[i]);
        mult *= 256;
    }//for//

    return ret;
}

typedef ByteAlignedUIntType<uint16_t> ByteAligned2ByteUIntType;
typedef ByteAlignedUIntType<uint32_t> ByteAligned4ByteUIntType;
typedef ByteAlignedUIntType<uint64_t> ByteAligned8ByteUIntType;


//--------------------------------------------------------------------------------------


inline
std::complex<double> CalcComplexExp(const double& x)
{
    return std::complex<double>(cos(x), sin(x));

}//CalcComplexExp//


// Purpose of FastIntToDataMap really just to use std::vector for node IDs unless
// node IDs are too sparse.


//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------

template<typename UIntType, typename T>
class FastIntToDataMap {
public:
    FastIntToDataMap(const double& initMinIntDensity = 0.5)
    :
    minIntDensity(initMinIntDensity),
    numberMappings(0),
    maximumInt(0),
    isFastMapped(initialTableSize, false),
    fastMap(initialTableSize)
    {
        //Not Yet (obsolete compiler) static_assert(std::is_unsigned<UIntType>::value, "Unsigned type only");
    }

    void InsertMapping(const UIntType& anInt, const T& value);
    bool IsMapped(const UIntType& anInt) const;
    const T& GetValue(const UIntType& anInt) const;
    T& operator[](const UIntType& anInt);
    const T& operator[](const UIntType& anInt) const { return (GetValue(anInt)); }

private:
    static const unsigned int initialTableSize = 5;

    double minIntDensity;
    unsigned int numberMappings;
    UIntType maximumInt;
    vector<bool> isFastMapped;
    vector<T> fastMap;
    std::map<UIntType, T> slowMap;

};//IntToIndexMapper//


template<typename UIntType, typename T> inline
bool FastIntToDataMap<UIntType, T>::IsMapped(const UIntType& anInt) const
{
    if (anInt < isFastMapped.size()) {
        return (isFastMapped[anInt]);
    }//if//

    return (slowMap.find(anInt) != slowMap.end());

}//IsMapped//



template<typename UIntType, typename T> inline
void FastIntToDataMap<UIntType, T>::InsertMapping(const UIntType& anInt, const T& value)
{
    maximumInt = std::max(maximumInt, anInt);

    if (slowMap.empty()) {
        if (anInt < isFastMapped.size()) {
            if (!isFastMapped[anInt]) {
                isFastMapped[anInt] = true;
                fastMap[anInt] = value;
                numberMappings++;
            }//if//
        }
        else {
            numberMappings++;

            assert(maximumInt != 0);

            const double newDensity =
                (static_cast<double>(numberMappings) / maximumInt);

            if (newDensity < minIntDensity) {
                // Density too low go to slow map.

                for (unsigned int i = 0; (i < fastMap.size()); i++) {
                    if (isFastMapped[i]) {
                        slowMap.insert(std::pair<UIntType, T>(i, fastMap[i]));
                    }//if//
                }//for//

                slowMap[anInt] = value;

                fastMap.clear();
                isFastMapped.clear();
            }
            else {
                // Keep expanding the table

                fastMap.resize(maximumInt + 1);
                isFastMapped.resize((maximumInt + 1), false);
                fastMap[anInt] = value;
                isFastMapped[anInt] = true;
            }//if//
        }//if//
    }
    else {
        const bool alreadyMapped = (slowMap.find(anInt) != slowMap.end());

        slowMap[anInt] = value;

        if (!alreadyMapped) {
            numberMappings++;

            assert(maximumInt > 0);

            const double newDensity =
                (static_cast<double>(numberMappings) / maximumInt);

            if (newDensity >= minIntDensity) {

                // Density is high enough to go to fast map.

                fastMap.resize(maximumInt + 1);
                isFastMapped.resize((maximumInt + 1), false);

                typedef typename std::map<UIntType, T>::iterator IterType;
                for(IterType iter = slowMap.begin(); (iter != slowMap.end()); ++iter) {
                    isFastMapped[iter->first] = true;
                    fastMap[iter->first] = iter->second;
                }//for//
                slowMap.clear();

                isFastMapped[anInt] = true;
                fastMap[anInt] = value;

            }//if//
        }//if//
    }//if//

}//AddMapping//


template<typename UIntType, typename T> inline
const T& FastIntToDataMap<UIntType, T>::GetValue(const UIntType& anInt) const
{
    if (anInt < fastMap.size()) {
        assert(isFastMapped[anInt]);
        return (fastMap[anInt]);
    }//if//

    typename std::map<UIntType, T>::const_iterator iter = slowMap.find(anInt);

    assert(iter != slowMap.end());
    return (iter->second);

}//GetValue//



template<typename UIntType, typename T> inline
T& FastIntToDataMap<UIntType, T>::operator[](const UIntType& anInt)
{
    if (anInt < fastMap.size()) {
        assert(isFastMapped[anInt]);
        return (fastMap[anInt]);
    }//if//

    typename std::map<UIntType, T>::iterator iter = slowMap.find(anInt);

    assert(iter != slowMap.end());
    return (iter->second);

}//[]//




//--------------------------------------------------------------------------------------------------
//
// Basic FFT algorithm from "Introduction to Algorithms", Cormen, et al., 1990.
//
//Future
//Future template<typename IntType> inline
//Future IntType CalcReverseBits(
//Future     const IntType& xIn,
//Future     const unsigned int numberBits)
//Future {
//Future     assert(numberBits < (sizeof(xIn) * 8));
//Future
//Future     IntType x = xIn;
//Future     IntType n = 0;
//Future     for (unsigned int i = 0; i < numberBits; i++) {
//Future         n <<= 1;
//Future         n |= (x & 0x1);
//Future         x >>= 1;
//Future     }//for//
//Future     return n;
//Future }
//Future
//Future inline
//Future unsigned int powOf2(unsigned int n)
//Future {
//Future     return (1 << n);
//Future }
//Future
//Future
//Future inline
//Future void CalcFft(
//Future     const vector<complex<double> >& input,
//Future     vector<complex<double> >& output)
//Future {
//Future     const unsigned int sizeLog2 =  static_cast<unsigned int>((log(1.0 * input.size()) / log(2.0)) + 0.5);
//Future
//Future     assert(static_cast<size_t>(pow(2.0, 1.0 * sizeLog2)) == input.size() && "Input size must be power of 2");
//Future
//Future     output.resize(input.size());
//Future
//Future     for (unsigned int i=0; (i < input.size()); i++) {
//Future         output[CalcReverseBits(i, sizeLog2)] = input[i];
//Future         cout << CalcReverseBits(i, sizeLog2) << endl;
//Future     }//for//
//Future
//Future     for (unsigned int s = 1; s <= sizeLog2; s++) {
//Future         const unsigned int m = powOf2(s);
//Future         const complex<double> wm(cos(2.0 * PI / m), sin(2.0 * PI / m));
//Future         complex<double> w = 1.0;
//Future         for (unsigned int j = 0; (j < (m/2)); j++) {
//Future             for (unsigned int k = j; (k < input.size()); k += m) {
//Future                 const complex<double> t = w * output[k + (m/2)];
//Future                 const complex<double> u = output[k];
//Future                 output[k] = u + t;
//Future                 output[k + (m/2)] = u - t;
//Future             }//for//
//Future
//Future             w *= wm;
//Future         }//for//
//Future     }//for//
//Future
//Future     // Reverse Order of output to match MATLAB fft() function results (default).
//Future
//Future     //for (unsigned int i=1; (i < (output.size()/2)); i++) {
//Future     //    std::swap(output[i], output[output.size() - i]);
//Future     //}//for//
//Future
//Future }//CalcFft//
//Future
//Future inline
//Future void CalcIfft(
//Future     const vector<complex<double> >& input,
//Future     vector<complex<double> >& output)
//Future {
//Future     vector<complex<double> > inputCopy(input.size());
//Future
//Future     for(unsigned int i = 0; (i < inputCopy.size()); i++) {
//Future         inputCopy[i] = conj(input[i]);
//Future     }//for//
//Future
//Future     CalcFft(inputCopy, output);
//Future
//Future     for(unsigned int i = 0; (i < output.size()); i++) {
//Future         output[i] = conj(output[i]);
//Future         output[i] /= static_cast<double>(output.size());
//Future     }//for//
//Future
//Future }//CalcIfft//


}//namespace//

#endif

