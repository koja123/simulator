// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef RANDOMNUMBERGEN_H
#define RANDOMNUMBERGEN_H

#include <cmath>
#include <string>
#include <limits>
#include "boost/math/constants/constants.hpp"
#include "boost/random.hpp"
#include <stdint.h>

namespace ScenSim {

typedef uint32_t RandomNumberGeneratorSeedType;

inline
unsigned long long int HashStringToInt(const std::string& aStringToHash)
{
    unsigned long long int hash = 5381;

    for(size_t i = 0; (i < aStringToHash.size()); i++) {
        hash = 33 * hash + static_cast<unsigned long long int>(aStringToHash[i]);
    }//if//

    return (hash);
}


inline
RandomNumberGeneratorSeedType HashInputsToMakeSeed(
    const RandomNumberGeneratorSeedType seed,
    const unsigned long long int hashingInput)
{
    // Formula is just "picked out of the air" (constants are prime).

    const unsigned long long int hash1 = seed;
    const unsigned long long int hash2 = hashingInput;

    return (static_cast<RandomNumberGeneratorSeedType>(
        (98385721723 * hash1 + 7138347631 * hash2 + 924713)));
}


template<typename T> inline
RandomNumberGeneratorSeedType HashInputsToMakeSeed(
    const RandomNumberGeneratorSeedType seed,
    const T hashingInput)
{
    return (HashInputsToMakeSeed(seed, static_cast<unsigned long long int>(hashingInput)));
}


template<typename T1, typename T2> inline
RandomNumberGeneratorSeedType HashInputsToMakeSeed(
    const RandomNumberGeneratorSeedType seed,
    const T1 hashingInput1,
    const T2 hashingInput2)
{
    return HashInputsToMakeSeed(HashInputsToMakeSeed(seed, hashingInput1), hashingInput2);
}

template<typename T1, typename T2, typename T3> inline
RandomNumberGeneratorSeedType HashInputsToMakeSeed(
    RandomNumberGeneratorSeedType Seed,
    const T1 hashingInput1,
    const T2 hashingInput2,
    const T3 hashingInput3)
{
    return HashInputsToMakeSeed(
        HashInputsToMakeSeed(Seed, hashingInput1, hashingInput2), hashingInput3);
}

template<typename T> inline
RandomNumberGeneratorSeedType HashInputsToMakeSeed(
    RandomNumberGeneratorSeedType seed,
    const std::string& hashingInput1,
    const T hashingInput2)
{
    return HashInputsToMakeSeed(HashInputsToMakeSeed(seed, HashStringToInt(hashingInput1)), hashingInput2);
}


// Lower quality (Linear congruential generator)  Small state and Fast.


class RandomNumberGenerator {
public:
    explicit RandomNumberGenerator(const RandomNumberGeneratorSeedType& seed)
        : generator(static_cast<boost::rand48::result_type>(seed)) {}

    void SetSeed(const RandomNumberGeneratorSeedType& seed)
        { generator.seed(static_cast<boost::rand48::result_type>(seed)); }

    int32_t GenerateRandomInt(const int32_t lowestValue, const int32_t highestValue);

    double GenerateRandomDouble() {
       // Generate range [0,1)
       return (static_cast<double>(generator()) / (static_cast<double>(generator.max()) + 1.0));
    }

    double erand48() { return GenerateRandomDouble(); }

    // For compatibility with STL templates.
    int32_t operator()(const int32_t maxPlus1) { return ((*this).GenerateRandomInt(0, (maxPlus1-1))); }

private:
    boost::rand48 generator;

};//RandomNumberGenerator;


inline
int32_t RandomNumberGenerator::GenerateRandomInt(const int32_t lowestValue, const int32_t highestValue)
{
    assert(lowestValue <= highestValue);
    if (lowestValue == highestValue) {
        return (lowestValue);
    }//if//

    return (static_cast<int32_t>(
        ((highestValue - lowestValue + 1) * (*this).GenerateRandomDouble()) + lowestValue));

    // Only makes performance sense running in 64bit mode:
    //long long int range = (highestValue - lowestValue + 1);
    //long long int randomVal = generator();
    //return (
    //    static_cast<int32_t>(
    //        ((range * randomVal) / (static_cast<long long int>(MAX_RANDOM_INT) + 1)) + lowestValue));
}

//--------------------------------------------------------------------------------------------------
//
// Higher quality random number generators have much larger state.
//

class HighQualityRandomNumberGenerator {
public:

    explicit HighQualityRandomNumberGenerator(const RandomNumberGeneratorSeedType& seed)
        : generator(static_cast<boost::mt19937::result_type>(seed)) {}

    void SetSeed(const RandomNumberGeneratorSeedType& seed)
        { generator.seed(static_cast<boost::mt19937::result_type>(seed)); }

    int32_t GenerateRandomInt(const int32_t lowestValue, const int32_t highestValue);

    double GenerateRandomDouble() {
       // Generate range [0,1)
       return (static_cast<double>(generator()) / (static_cast<double>(generator.max()) + 1.0));
    }

    // For compatibility with STL templates.
    int32_t operator()(const int32_t maxPlus1) { return ((*this).GenerateRandomInt(0, (maxPlus1-1))); }

private:
    boost::mt19937 generator;

};//HighQualityRandomNumberGenerator//


inline
int32_t HighQualityRandomNumberGenerator::GenerateRandomInt(
    const int32_t lowestValue, const int32_t highestValue)
{
    assert(lowestValue <= highestValue);
    if (lowestValue == highestValue) {
        return (lowestValue);
    }//if//

    return (static_cast<int32_t>(
        ((highestValue - lowestValue + 1) * (*this).GenerateRandomDouble()) + lowestValue));

}

//--------------------------------------------------------------------------------------------------

inline
double ConvertToExponentialDistribution(const double& randomDouble)
{
    // Using assumption that random Double is in the range [0.0,1.0).

    return (-log(1.0 - randomDouble));
}


inline
double ConvertToGuassianDistribution(const double& uniformRandom1, const double& uniformRandom2)
{
    // Using assumption that random input variables are in the range [0.0,1.0).
    using std::max;
    const double PI = boost::math::constants::pi<double>();


    const double minRandomValue = 1e-100;

    return (sqrt(-2.0 * log(max(uniformRandom1, minRandomValue))) * cos(2.0 * PI * uniformRandom2));
}

// More efficent version produces two gaussian distribution values.

inline
void ConvertToGaussianDistribution(
    const double& uniformRandom1,
    const double& uniformRandom2,
    double& gaussianRandom1,
    double& gaussianRandom2)
{
    // Using assumption that random input variables are in the range [0.0,1.0).
    using std::max;
    const double PI = boost::math::constants::pi<double>();

    const double minRandomValue = 1e-100;

    const double factor = sqrt(-2.0 * log(max(uniformRandom1, minRandomValue)));

    gaussianRandom1 = factor * cos(2.0 * PI * uniformRandom2);
    gaussianRandom2 = factor * sin(2.0 * PI * uniformRandom2);
}

}//namespace//


#endif


