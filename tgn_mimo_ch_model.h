// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef TGN_MIMO_CH_MODEL_H
#define TGN_MIMO_CH_MODEL_H

//
// References
//
// [1] IEEE P802.11 Wireless LANs, TGn Channel Models, IEEE 802.11-03/940r4.
//
// [2] "From Antenna Spacings To Theoretical Capacities - Guidelines For Simulating MIMO Systems",
// Laurent Schumacher, Klaus I Pedersen, Preben E. Mogensen, PIMRC 2002.
//


#include <limits.h>
#include <assert.h>
#include <array>
#include <complex>
#include <iostream>
#include <iomanip>

#include "boost/math/constants/constants.hpp"
#include "boost/math/special_functions/bessel.hpp"

//#undef BOOST_UBLAS_TYPE_CHECK
//#define BOOST_UBLAS_TYPE_CHECK 0
#include "boost/numeric/ublas/matrix.hpp"
#include "boost/numeric/ublas/lu.hpp"
#include "boost/numeric/ublas/io.hpp"
#include "scensim_support.h"
#include "randomnumbergen.h"


namespace TgnMimoChModel {

using std::complex;
using std::vector;
using std::array;
using std::min;
using namespace boost::numeric;
using boost::math::cyl_bessel_j;
using ScenSim::InterpolatedTable;
using ScenSim::ConvertToDb;
using ScenSim::ConvertToNonDb;
using ScenSim::ConvertToGaussianDistribution;

using std::cin;
using std::cerr;
using std::endl;
using std::setprecision;


const double PI = boost::math::constants::pi<double>();
const double sqrt2 = boost::math::constants::root_two<double>();

const double DegToRadFactor = (PI / 180.0);
const double SpeedOfLightMSecs = 299792458.0;


const double DefaultScatterMovementSpeedMSec = 0.333333333333333333333333333333333333;

enum ChannelModelChoicesType {
    ModelB, ModelC, ModelD, ModelE
};

const unsigned int numberFadingSamplesForFilterInitialization = 1000;


// Angle Spread (AS) is the standard deviation of the angle of Departure/Arrival distribution which is
// defined to be a truncated laplacian distribution (at +-180 degrees)  Truncation
// decreases the standard deviation (SD) from the non-truncated laplacian with a
// fixed "b" (diversity/scale parameter) where b = SD/sqrt(2).
// To counteract this, the "b" parameter is increased so that the truncated laplacian
// distribution's SD is equal to the Angle Spread.  A bruteforce generation of this conversion
// table can be done by generating large number of samples using truncated laplacian
// distribution for certain "b" and then taking the sample's SD. For example (generator in Matlab):
//
// function out=TruncLaplaceRnd(sigma,xmax,m)
// % sigma is the standard deviation, xmax is the truncation value and m is the number samples
// % and sigma is the Scale parameter
// b = sigma / sqrt(2);
// pmax = (1-exp(-xmax/b));
// randnums = (rand(m,1) - 0.5) * pmax;
// out = b * sign(randnums) .* log(1-(2 * abs(randnums)));
//
// std(TruncLaplaceRnd(pi/3, pi, 10000000))
//
// Ridiculous number of (probably not accurate) digits for ease of verification with original Matlab model.
//

const array<array<double, 2>, 116> AsToLaplacianSdTableRadData =
    {{{0.0, 0.0}, {0.104719755119660, 0.104719755119660},
      {0.122173047639596, 0.122173047639603}, {0.139626340158977, 0.139626340159546},
      {0.157079632661986, 0.157079632679490}, {0.174532924930956, 0.174532925199433},
      {0.191986215232256, 0.191986217719376}, {0.209439494442749, 0.209439510239320},
      {0.226892727669123, 0.226892802759263}, {0.244345810885999, 0.244346095279206},
      {0.261798489410822, 0.261799387799149}, {0.279250230595050, 0.279252680319093},
      {0.296700053716026, 0.296705972839036}, {0.314146331632852, 0.314159265358979},
      {0.331586587071365, 0.331612557878923}, {0.349017309085200, 0.349065850398866},
      {0.366433812341661, 0.366519142918809}, {0.383830155088697, 0.383972435438752},
      {0.401199123221207, 0.401425727958696}, {0.418532279754812, 0.418879020478639},
      {0.435820072528438, 0.436332312998582}, {0.453051988664963, 0.453785605518526},
      {0.470216742224809, 0.471238898038469}, {0.487302481242275, 0.488692190558412},
      {0.504297001436864, 0.506145483078356}, {0.521187955827321, 0.523598775598299},
      {0.537963051801369, 0.541052068118242}, {0.554610229571735, 0.558505360638185},
      {0.571117818151835, 0.575958653158129}, {0.587474666881112, 0.593411945678072},
      {0.603670252063781, 0.610865238198015}, {0.619694759451538, 0.628318530717959},
      {0.635539144129193, 0.645771823237902}, {0.651195169899051, 0.663225115757845},
      {0.666655430558279, 0.680678408277789}, {0.681913355576567, 0.698131700797732},
      {0.696963202657100, 0.715584993317675}, {0.711800039543675, 0.733038285837618},
      {0.726419717254459, 0.750491578357562}, {0.740818836704953, 0.767944870877505},
      {0.754994710449240, 0.785398163397448}, {0.768945321034146, 0.802851455917391},
      {0.782669277235311, 0.820304748437335}, {0.796165769234139, 0.837758040957278},
      {0.809434523603436, 0.855211333477221}, {0.822475758799431, 0.872664625997165},
      {0.835290141708725, 0.890117918517108}, {0.847878745670160, 0.907571211037051},
      {0.860243010282100, 0.925024503556995}, {0.872384703213731, 0.942477796076938},
      {0.884305884162805, 0.959931088596881}, {0.896008871039867, 0.977384381116825},
      {0.907496208408836, 0.994837673636768}, {0.918770638173840, 1.012290966156710},
      {0.929835072471174, 1.029744258676650}, {0.940692568701664, 1.047197551196600},
      {0.951346306621214, 1.064650843716540}, {0.961799567395006, 1.082104136236480},
      {0.972055714512689, 1.099557428756430}, {0.982118176457062, 1.117010721276370},
      {0.991990431016692, 1.134464013796310}, {1.001675991132880, 1.151917306316260},
      {1.011178392173110, 1.169370598836200}, {1.020501180525920, 1.186823891356140},
      {1.029647903416020, 1.204277183876090}, {1.038622099842810, 1.221730476396030},
      {1.047427292550560, 1.239183768915970}, {1.056066980943300, 1.256637061435920},
      {1.064544634863070, 1.274090353955860}, {1.072863689155150, 1.291543646475800},
      {1.081027538949400, 1.308996938995750}, {1.089039535591530, 1.326450231515690},
      {1.096902983163310, 1.343903524035630}, {1.104621135535220, 1.361356816555580},
      {1.112197193899670, 1.378810109075520}, {1.119634304736860, 1.396263401595460},
      {1.126935558169480, 1.413716694115410}, {1.134103986666180, 1.431169986635350},
      {1.141142564057030, 1.448623279155290}, {1.148054204827410, 1.466076571675240},
      {1.154841763660070, 1.483529864195180}, {1.161508035197230, 1.500983156715120},
      {1.168055753997810, 1.518436449235070}, {1.174487594666700, 1.535889741755010},
      {1.180806172135320, 1.553343034274950}, {1.187014042074880, 1.570796326794900},
      {1.193113701425080, 1.588249619314840}, {1.199107589023140, 1.605702911834780},
      {1.204998086319330, 1.623156204354730}, {1.210787518166350, 1.640609496874670},
      {1.216478153671760, 1.658062789394610}, {1.222072207102970, 1.675516081914560},
      {1.227571838836170, 1.692969374434500}, {1.232979156340940, 1.710422666954440},
      {1.238296215193410, 1.727875959474390}, {1.243525020111490, 1.745329251994330},
      {1.248667526006570, 1.762782544514270}, {1.253725639046540, 1.780235837034220},
      {1.258701217725560, 1.797689129554160}, {1.263596073936870, 1.815142422074100},
      {1.268411974044770, 1.832595714594050}, {1.273150639953000, 1.850049007113990},
      {1.277813750166740, 1.867502299633930}, {1.282402940845770, 1.884955592153880},
      {1.286919806846910, 1.902408884673820}, {1.291365902753950, 1.919862177193760},
      {1.295742743893520, 1.937315469713710}, {1.300051807335630, 1.954768762233650},
      {1.304294532877900, 1.972222054753590}, {1.308472324012430, 1.989675347273540},
      {1.312586548874740, 2.007128639793480}, {1.316638541174090, 2.024581932313420},
      {1.320629601104760, 2.042035224833370}, {1.324560996237890, 2.059488517353310},
      {1.328433962393790, 2.076941809873250}, {1.332249704494320, 2.094395102393200}}};


const InterpolatedTable AngleSpreadToLaplacianStandardDeviationTableRad(AsToLaplacianSdTableRadData);


struct InfoForClusterType {
    double relativePower;
    double rayAngleRad;
    double rayAngleSpreadStandardDeviationRad;

    double laplacianRayAngleSpreadSdRad;
    double normalizationFactorQ;

    InfoForClusterType() : normalizationFactorQ(-DBL_MAX) { }

};//InfoForClusterType//


struct InfoForTapType {
    double powerDelayProfileRelativePower;

    // Normalized (to total 1.0) power delay profile of the (LOS+NLOS) power.

    double normalizedPowerDelayProfilePower;
    double normalizedPowerDelayProfilePowerNoLos;

    double GetNormalizedPowerDelayProfilePower(const bool withLineOfSight)
    {
        if (withLineOfSight) {
            return (normalizedPowerDelayProfilePower);
        }
        else {
            return (normalizedPowerDelayProfilePowerNoLos);
        }//if//
    }

    // Using parallel arrays to facilitate function reuse.
    vector<InfoForClusterType> departureClusterInfo;
    vector<InfoForClusterType> arrivalClusterInfo;

};//InfoForTapType//


struct OutputInfoForTapType {
    ublas::matrix<complex<double> > correlationMatrix;

    vector<vector<complex<double> > > currentFilterStates;

    ublas::vector<complex<double> > currentFadingVector;
    ublas::vector<complex<double> > nextFadingVector;

    ublas::matrix<complex<double> > hMatrix;

};//OutputInfoForTapType//



struct LinkParametersType {
    unsigned int departureNodeNumberAntennas;
    double departureNodeAntennaSpacing;

    unsigned int arrivalNodeNumberAntennas;
    double arrivalNodeAntennaSpacing;

};//MimoChannelModelInputParametersType//



inline
void GetModelBParameters(
    double& kFactorDb,
    double& kFactorBreakpointMeters,
    vector<InfoForTapType>& infoForTaps)
{
    kFactorDb = 0.0;
    kFactorBreakpointMeters = 5.0;

    const unsigned int numberTaps = 9;
    const unsigned int numberClusters = 2;
    const double NoAngle = -DBL_MAX;

    const array<array<double, numberTaps>, numberClusters> PowerDelayProfileDb =
        {{{0.0, -5.4, -10.8, -16.2, -21.7, -DBL_MAX,  -DBL_MAX, -DBL_MAX, -DBL_MAX},
         {-DBL_MAX, -DBL_MAX, -3.2, -6.3, -9.4, -12.5, -15.6, -18.7, -21.8}}};

    const array<array<double, numberTaps>, numberClusters> AnglesOfDepartureDeg =
        {{{225.1, 225.1, 225.1, 225.1, 225.1, NoAngle, NoAngle, NoAngle, NoAngle},
         {NoAngle, NoAngle, 106.5, 106.5, 106.5, 106.5, 106.5, 106.5, 106.5}}};

    const array<array<double, numberTaps>, numberClusters> AngleOfDepartureSpreadStandardDeviationsDeg =
        {{{14.4, 14.4, 14.4, 14.4, 14.4, NoAngle, NoAngle, NoAngle, NoAngle},
         {NoAngle, NoAngle, 25.4, 25.4, 25.4, 25.4, 25.4, 25.4, 25.4}}};

    const array<array<double, numberTaps>, numberClusters> AnglesOfArrivalDeg =
        {{{4.3, 4.3, 4.3, 4.3, 4.3, NoAngle, NoAngle, NoAngle, NoAngle},
         {NoAngle, NoAngle, 118.4, 118.4, 118.4, 118.4, 118.4, 118.4, 118.4}}};

    const array<array<double, numberTaps>, numberClusters> AngleOfArrivalSpreadStandardDeviationsDeg =
        {{{14.4, 14.4, 14.4, 14.4, 14.4, NoAngle, NoAngle, NoAngle, NoAngle},
         {NoAngle, NoAngle, 25.2, 25.2, 25.2, 25.2, 25.2, 25.2, 25.2}}};


    // Original MATLAB model constants with more digits than official Tgn document.
    //
    // const array<array<double, numberTaps>, numberClusters> PowerDelayProfileDb =
    //     {{{0.0, -5.4287, -10.8574, -16.2860, -21.7147, -DBL_MAX,  -DBL_MAX, -DBL_MAX, -DBL_MAX},
    //      {-DBL_MAX, -DBL_MAX, -3.2042, -6.3063, -9.4084, -12.5105, -15.6126, -18.7147, -21.8168}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AnglesOfDepartureDeg =
    //     {{{225.1084, 225.1084, 225.1084, 225.1084, 225.1084, NoAngle, NoAngle, NoAngle, NoAngle},
    //      {NoAngle, NoAngle, 106.5545, 106.5545, 106.5545, 106.5545, 106.5545, 106.5545, 106.5545}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AngleOfDepartureSpreadStandardDeviationsDeg =
    //     {{{14.4490, 14.4490, 14.4490, 14.4490, 14.4490, NoAngle, NoAngle, NoAngle, NoAngle},
    //      {NoAngle, NoAngle, 25.4311, 25.4311, 25.4311, 25.4311, 25.4311, 25.4311, 25.4311}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AnglesOfArrivalDeg =
    //     {{{4.3943, 4.3943, 4.3943, 4.3943, 4.3943, NoAngle, NoAngle, NoAngle, NoAngle},
    //      {NoAngle, NoAngle, 118.4327, 118.4327, 118.4327, 118.4327, 118.4327, 118.4327, 118.4327}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AngleOfArrivalSpreadStandardDeviationsDeg =
    //     {{{14.4699, 14.4699, 14.4699, 14.4699, 14.4699, NoAngle, NoAngle, NoAngle, NoAngle},
    //      {NoAngle, NoAngle, 25.2566, 25.2566, 25.2566, 25.2566, 25.2566, 25.2566, 25.2566}}};

    infoForTaps.resize(numberTaps);

    for(unsigned int i = 0; (i < numberTaps); i++) {
        InfoForTapType& tapInfo = infoForTaps[i];
        tapInfo.powerDelayProfileRelativePower = 0.0;

        for(unsigned int j = 0; (j < numberClusters); j++) {
            if (PowerDelayProfileDb[j][i] != -DBL_MAX) {
                assert(AnglesOfDepartureDeg[j][i] != NoAngle);
                assert(AnglesOfArrivalDeg[j][i] != NoAngle);

                tapInfo.powerDelayProfileRelativePower += ConvertToNonDb(PowerDelayProfileDb[j][i]);

                tapInfo.departureClusterInfo.resize(tapInfo.departureClusterInfo.size() + 1);
                InfoForClusterType& clusterInfo = tapInfo.departureClusterInfo.back();

                clusterInfo.relativePower = ConvertToNonDb(PowerDelayProfileDb[j][i]);
                clusterInfo.rayAngleRad = AnglesOfDepartureDeg[j][i] * DegToRadFactor;
                clusterInfo.rayAngleSpreadStandardDeviationRad =
                    AngleOfDepartureSpreadStandardDeviationsDeg[j][i] * DegToRadFactor;


                tapInfo.arrivalClusterInfo.resize(tapInfo.arrivalClusterInfo.size() + 1);
                InfoForClusterType& arrivalClusterInfo = tapInfo.arrivalClusterInfo.back();

                arrivalClusterInfo.relativePower = ConvertToNonDb(PowerDelayProfileDb[j][i]);
                arrivalClusterInfo.rayAngleRad = AnglesOfArrivalDeg[j][i] * DegToRadFactor;
                arrivalClusterInfo.rayAngleSpreadStandardDeviationRad =
                    AngleOfArrivalSpreadStandardDeviationsDeg[j][i] * DegToRadFactor;
            }
            else {
                assert(AnglesOfDepartureDeg[j][i] == NoAngle);
                assert(AnglesOfArrivalDeg[j][i] == NoAngle);
            }//if//
        }//for//
    }//for//

}//GetModelBParameters//



inline
void GetModelCParameters(
    double& kFactorDb,
    double& kFactorBreakpointMeters,
    vector<InfoForTapType>& infoForTaps)
{
    kFactorDb = 0.0;
    kFactorBreakpointMeters = 5.0;

    const unsigned int numberTaps = 14;
    const unsigned int numberClusters = 2;
    const double NoAngle = -DBL_MAX;

    const array<array<double, numberTaps>, numberClusters> PowerDelayProfileDb =
      {{{0.0, -2.1, -4.3, -6.5, -8.6, -10.8, -13.0, -15.2, -17.3, -19.5,
         -DBL_MAX, -DBL_MAX,  -DBL_MAX,  -DBL_MAX},
        {-DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -5.0, -7.2, -9.3, -11.5,
         -13.7, -15.8, -18.0, -20.2}}};

    const array<array<double, numberTaps>, numberClusters> AnglesOfDepartureDeg =
        {{{13.5, 13.5, 13.5, 13.5, 13.5, 13.5, 13.5, 13.5, 13.5, 13.5,
           NoAngle, NoAngle, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 56.4, 56.4, 56.4, 56.4,
           56.4, 56.4, 56.4, 56.4}}};

    const array<array<double, numberTaps>, numberClusters> AngleOfDepartureSpreadStandardDeviationsDeg =
        {{{24.7, 24.7, 24.7, 24.7, 24.7, 24.7, 24.7, 24.7, 24.7, 24.7,
           NoAngle, NoAngle, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 22.5, 22.5, 22.5, 22.5,
           22.5, 22.5, 22.5, 22.5}}};

    const array<array<double, numberTaps>, numberClusters> AnglesOfArrivalDeg =
        {{{290.3, 290.3, 290.3, 290.3, 290.3, 290.3, 290.3, 290.3, 290.3, 290.3,
           NoAngle, NoAngle, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 332.3, 332.3, 332.3, 332.3,
           332.3, 332.3, 332.3, 332.3}}};

    const array<array<double, numberTaps>, numberClusters> AngleOfArrivalSpreadStandardDeviationsDeg =
        {{{24.6, 24.6, 24.6, 24.6, 24.6, 24.6, 24.6, 24.6, 24.6, 24.6,
           NoAngle, NoAngle, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 22.4, 22.4, 22.4, 22.4,
           22.4, 22.4, 22.4, 22.4}}};

    // Original MATLAB model constants with more digits than official Tgn document.
    //
    // const array<array<double, numberTaps>, numberClusters> PowerDelayProfileDb =
    //   {{{0.0, -2.1715, -4.3429, -6.5144, -8.6859, -10.8574, -13.0288, -15.2003, -17.3718, -19.5433,
    //      -DBL_MAX, -DBL_MAX,  -DBL_MAX,  -DBL_MAX},
    //     {-DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -5.0288, -7.2003, -9.3718, -11.5433,
    //      -13.7147, -15.8862, -18.0577, -20.2291}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AnglesOfDepartureDeg =
    //     {{{13.5312, 13.5312, 13.5312, 13.5312, 13.5312, 13.5312, 13.5312, 13.5312, 13.5312, 13.5312,
    //        NoAngle, NoAngle, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 56.4329, 56.4329, 56.4329, 56.4329,
    //        56.4329, 56.4329, 56.4329, 56.4329}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AngleOfDepartureSpreadStandardDeviationsDeg =
    //     {{{24.7897, 24.7897, 24.7897, 24.7897, 24.7897, 24.7897, 24.7897, 24.7897, 24.7897, 24.7897,
    //        NoAngle, NoAngle, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 22.5729, 22.5729, 22.5729, 22.5729,
    //        22.5729, 22.5729, 22.5729, 22.5729}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AnglesOfArrivalDeg =
    //     {{{290.3715, 290.3715, 290.3715, 290.3715, 290.3715, 290.3715, 290.3715, 290.3715, 290.3715, 290.3715,
    //        NoAngle, NoAngle, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 332.3754, 332.3754, 332.3754, 332.3754,
    //        332.3754, 332.3754, 332.3754, 332.3754}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AngleOfArrivalSpreadStandardDeviationsDeg =
    //     {{{24.6949, 24.6949, 24.6949, 24.6949, 24.6949, 24.6949, 24.6949, 24.6949, 24.6949, 24.6949,
    //        NoAngle, NoAngle, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 22.4530, 22.4530, 22.4530, 22.4530,
    //        22.4530, 22.4530, 22.4530, 22.4530}}};

    infoForTaps.resize(numberTaps);

    for(unsigned int i = 0; (i < numberTaps); i++) {
        InfoForTapType& tapInfo = infoForTaps[i];
        tapInfo.powerDelayProfileRelativePower = 0.0;

        for(unsigned int j = 0; (j < numberClusters); j++) {
            if (PowerDelayProfileDb[j][i] != -DBL_MAX) {
                assert(AnglesOfDepartureDeg[j][i] != NoAngle);
                assert(AnglesOfArrivalDeg[j][i] != NoAngle);

                tapInfo.powerDelayProfileRelativePower += ConvertToNonDb(PowerDelayProfileDb[j][i]);

                tapInfo.departureClusterInfo.resize(tapInfo.departureClusterInfo.size() + 1);
                InfoForClusterType& clusterInfo = tapInfo.departureClusterInfo.back();

                clusterInfo.relativePower = ConvertToNonDb(PowerDelayProfileDb[j][i]);
                clusterInfo.rayAngleRad = AnglesOfDepartureDeg[j][i] * DegToRadFactor;
                clusterInfo.rayAngleSpreadStandardDeviationRad =
                    AngleOfDepartureSpreadStandardDeviationsDeg[j][i] * DegToRadFactor;


                tapInfo.arrivalClusterInfo.resize(tapInfo.arrivalClusterInfo.size() + 1);
                InfoForClusterType& arrivalClusterInfo = tapInfo.arrivalClusterInfo.back();

                arrivalClusterInfo.relativePower = ConvertToNonDb(PowerDelayProfileDb[j][i]);
                arrivalClusterInfo.rayAngleRad = AnglesOfArrivalDeg[j][i] * DegToRadFactor;
                arrivalClusterInfo.rayAngleSpreadStandardDeviationRad =
                    AngleOfArrivalSpreadStandardDeviationsDeg[j][i] * DegToRadFactor;
            }
            else {
                assert(AnglesOfDepartureDeg[j][i] == NoAngle);
                assert(AnglesOfArrivalDeg[j][i] == NoAngle);
            }//if//
        }//for//
    }//for//

}//GetModelCParameters//



inline
void GetModelDParameters(
    double& kFactorDb,
    double& kFactorBreakpointMeters,
    vector<InfoForTapType>& infoForTaps)
{
    kFactorDb = 3.0;
    kFactorBreakpointMeters = 10.0;

    const unsigned int numberTaps = 18;
    const unsigned int numberClusters = 3;
    const double NoAngle = -DBL_MAX;


    const array<array<double, numberTaps>, numberClusters> PowerDelayProfileDb =
        {{{0.0, -0.9, -1.7, -2.6, -3.5, -4.3, -5.2, -6.1, -6.9, -7.8,
           -9.0, -11.1,  -13.7, -16.3, -19.3, -23.2, -DBL_MAX, -DBL_MAX},
          {-DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX,
           -6.6,  -9.5, -12.1, -14.7, -17.4, -21.9, -25.5, -DBL_MAX},
          {-DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX,
           -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -18.8, -23.2, -25.2, -26.7}}};

    const array<array<double, numberTaps>, numberClusters> AnglesOfDepartureDeg =
        {{{332.1, 332.1, 332.1, 332.1, 332.1, 332.1, 332.1, 332.1, 332.1, 332.1,
           332.1, 332.1, 332.1, 332.1, 332.1, 332.1, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
           49.3, 49.3, 49.3, 49.3, 49.3, 49.3, 49.3, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
           NoAngle, NoAngle, NoAngle, NoAngle, 275.9, 275.9, 275.9, 275.9}}};

    const array<array<double, numberTaps>, numberClusters> AngleOfDepartureSpreadStandardDeviationsDeg =
        {{{27.4, 27.4, 27.4, 27.4, 27.4, 27.4, 27.4, 27.4, 27.4, 27.4,
           27.4, 27.4, 27.4, 27.4, 27.4, 27.4, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
           32.1, 32.1, 32.1, 32.1, 32.1, 32.1, 32.1, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
           NoAngle, NoAngle, NoAngle, NoAngle, 36.8, 36.8, 36.8, 36.8}}};

    const array<array<double, numberTaps>, numberClusters> AnglesOfArrivalDeg =
        {{{158.9, 158.9, 158.9, 158.9, 158.9, 158.9, 158.9, 158.9, 158.9, 158.9,
           158.9, 158.9, 158.9, 158.9, 158.9, 158.9, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
           320.2, 320.2, 320.2, 320.2, 320.2, 320.2, 320.2, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
           NoAngle, NoAngle, NoAngle, NoAngle, 276.1, 276.1, 276.1, 276.1}}};

    const array<array<double, numberTaps>, numberClusters> AngleOfArrivalSpreadStandardDeviationsDeg =
        {{{27.7, 27.7, 27.7, 27.7, 27.7, 27.7, 27.7, 27.7, 27.7, 27.7,
           27.7, 27.7, 27.7, 27.7, 27.7, 27.7, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
           31.4, 31.4, 31.4, 31.4, 31.4, 31.4, 31.4, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
           NoAngle, NoAngle, NoAngle, NoAngle, 37.4, 37.4, 37.4, 37.4}}};


    // Original MATLAB model constants with more digits than official Tgn document.
    //
    // const array<array<double, numberTaps>, numberClusters> PowerDelayProfileDb =
    //     {{{0.0, -0.9, -1.7, -2.6, -3.5, -4.3, -5.2, -6.1, -6.9, -7.8,
    //        -9.0712046, -11.199064,  -13.795428, -16.391791, -19.370991, -23.201722, -DBL_MAX, -DBL_MAX},
    //       {-DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX,
    //        -6.6756386,  -9.5728825, -12.175385, -14.777891, -17.435786, -21.992788, -25.580689, -DBL_MAX},
    //       {-DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX,
    //        -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -18.843300, -23.238125, -25.246344, -26.7}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AnglesOfDepartureDeg =
    //     {{{332.1027, 332.1027, 332.1027, 332.1027, 332.1027, 332.1027, 332.1027, 332.1027, 332.1027, 332.1027,
    //        332.1027, 332.1027, 332.1027, 332.1027, 332.1027, 332.1027, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
    //        49.3840, 49.3840, 49.3840, 49.3840, 49.3840, 49.3840, 49.3840, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
    //        NoAngle, NoAngle, NoAngle, NoAngle, 275.9769, 275.9769, 275.9769, 275.9769}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AngleOfDepartureSpreadStandardDeviationsDeg =
    //     {{{27.4412, 27.4412, 27.4412, 27.4412, 27.4412, 27.4412, 27.4412, 27.4412, 27.4412, 27.4412,
    //        27.4412, 27.4412, 27.4412, 27.4412, 27.4412, 27.4412, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
    //        32.1430, 32.1430, 32.1430, 32.1430, 32.1430, 32.1430, 32.1430, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
    //        NoAngle, NoAngle, NoAngle, NoAngle, 36.8825, 36.8825, 36.8825, 36.8825}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AnglesOfArrivalDeg =
    //     {{{158.9318, 158.9318, 158.9318, 158.9318, 158.9318, 158.9318, 158.9318, 158.9318, 158.9318, 158.9318,
    //        158.9318, 158.9318, 158.9318, 158.9318, 158.9318, 158.9318, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
    //        320.2865, 320.2865, 320.2865, 320.2865, 320.2865, 320.2865, 320.2865, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
    //        NoAngle, NoAngle, NoAngle, NoAngle, 276.1246, 276.1246, 276.1246, 276.1246}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AngleOfArrivalSpreadStandardDeviationsDeg =
    //     {{{27.7580, 27.7580, 27.7580, 27.7580, 27.7580, 27.7580, 27.7580, 27.7580, 27.7580, 27.7580,
    //        27.7580, 27.7580, 27.7580, 27.7580, 27.7580, 27.7580, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
    //        31.4672, 31.4672, 31.4672, 31.4672, 31.4672, 31.4672, 31.4672, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
    //        NoAngle, NoAngle, NoAngle, NoAngle, 37.4179, 37.4179, 37.4179, 37.4179}}};

    infoForTaps.resize(numberTaps);

    for(unsigned int i = 0; (i < numberTaps); i++) {
        InfoForTapType& tapInfo = infoForTaps[i];
        tapInfo.powerDelayProfileRelativePower = 0.0;

        for(unsigned int j = 0; (j < numberClusters); j++) {
            if (PowerDelayProfileDb[j][i] != -DBL_MAX) {
                assert(AnglesOfDepartureDeg[j][i] != NoAngle);
                assert(AnglesOfArrivalDeg[j][i] != NoAngle);

                tapInfo.powerDelayProfileRelativePower += ConvertToNonDb(PowerDelayProfileDb[j][i]);

                tapInfo.departureClusterInfo.resize(tapInfo.departureClusterInfo.size() + 1);
                InfoForClusterType& clusterInfo = tapInfo.departureClusterInfo.back();

                clusterInfo.relativePower = ConvertToNonDb(PowerDelayProfileDb[j][i]);
                clusterInfo.rayAngleRad = AnglesOfDepartureDeg[j][i] * DegToRadFactor;
                clusterInfo.rayAngleSpreadStandardDeviationRad =
                    AngleOfDepartureSpreadStandardDeviationsDeg[j][i] * DegToRadFactor;


                tapInfo.arrivalClusterInfo.resize(tapInfo.arrivalClusterInfo.size() + 1);
                InfoForClusterType& arrivalClusterInfo = tapInfo.arrivalClusterInfo.back();

                arrivalClusterInfo.relativePower = ConvertToNonDb(PowerDelayProfileDb[j][i]);
                arrivalClusterInfo.rayAngleRad = AnglesOfArrivalDeg[j][i] * DegToRadFactor;
                arrivalClusterInfo.rayAngleSpreadStandardDeviationRad =
                    AngleOfArrivalSpreadStandardDeviationsDeg[j][i] * DegToRadFactor;
            }
            else {
                assert(AnglesOfDepartureDeg[j][i] == NoAngle);
                assert(AnglesOfArrivalDeg[j][i] == NoAngle);
            }//if//
        }//for//
    }//for//

}//GetModelDParameters//



inline
void GetModelEParameters(
    double& kFactorDb,
    double& kFactorBreakpointMeters,
    vector<InfoForTapType>& infoForTaps)
{
    kFactorDb = 6.0;
    kFactorBreakpointMeters = 20.0;

    const unsigned int numberTaps = 18;
    const unsigned int numberClusters = 4;
    const double NoAngle = -DBL_MAX;

    const array<array<double, numberTaps>, numberClusters> PowerDelayProfileDb =
        {{{-2.6, -3.0, -3.5, -3.9, -4.5, -5.6, -6.9, -8.2, -9.8, -11.7,
           -13.9, -16.1, -18.3, -20.5, -22.9, -DBL_MAX, -DBL_MAX, -DBL_MAX},
          {-DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -1.8, -3.2, -4.5, -5.8, -7.1, -9.9,
           -10.3, -14.3, -14.7, -18.7, -19.9, -22.4, -DBL_MAX,  -DBL_MAX},
          {-DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -7.9,  -9.6,
           -14.2, -13.8, -18.6, -18.1, -22.8, -DBL_MAX, -DBL_MAX, -DBL_MAX},
          {-DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX,
           -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -20.6, -20.5, -20.7, -24.6}}};

    const array<array<double, numberTaps>, numberClusters> AnglesOfDepartureDeg =
        {{{105.6, 105.6, 105.6, 105.6, 105.6, 105.6, 105.6, 105.6, 105.6, 105.6,
           105.6, 105.6, 105.6, 105.6, 105.6, NoAngle, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, 293.1, 293.1, 293.1, 293.1, 293.1, 293.1,
           293.1, 293.1, 293.1, 293.1, 293.1, 293.1, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 61.9, 61.9,
           61.9, 61.9, 61.9, 61.9, 61.9, NoAngle, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
           NoAngle, NoAngle, NoAngle, NoAngle, 275.7, 275.7, 275.7, 275.7}}};

    const array<array<double, numberTaps>, numberClusters> AngleOfDepartureSpreadStandardDeviationsDeg =
        {{{36.1, 36.1, 36.1, 36.1, 36.1, 36.1, 36.1, 36.1, 36.1, 36.1,
           36.1, 36.1, 36.1, 36.1, 36.1, NoAngle, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, 42.5, 42.5, 42.5, 42.5, 42.5, 42.5,
           42.5, 42.5, 42.5, 42.5, 42.5, 42.5, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 38.0, 38.0,
           38.0, 38.0, 38.0, 38.0, 38.0, NoAngle, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
           NoAngle, NoAngle, NoAngle, NoAngle, 38.7, 38.7, 38.7, 38.7}}};

    const array<array<double, numberTaps>, numberClusters> AnglesOfArrivalDeg =
        {{{163.7, 163.7, 163.7, 163.7, 163.7, 163.7, 163.7, 163.7, 163.7, 163.7,
           163.7, 163.7, 163.7, 163.7, 163.7, NoAngle, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, 251.8, 251.8, 251.8, 251.8, 251.8, 251.8,
           251.8, 251.8, 251.8, 251.8, 251.8, 251.8, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 80.0240, 80.0240,
           80.0, 80.0, 80.0, 80.0, 80.0, NoAngle, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
           NoAngle, NoAngle, NoAngle, NoAngle, 182.0, 182.0, 182.0, 182.0}}};

    const array<array<double, numberTaps>, numberClusters> AngleOfArrivalSpreadStandardDeviationsDeg =
        {{{35.8, 35.8, 35.8, 35.8, 35.8, 35.8, 35.8, 35.8, 35.8, 35.8,
           35.8, 35.8, 35.8, 35.8, 35.8, NoAngle, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, 41.6, 41.6, 41.6, 41.6, 41.6, 41.6,
           41.6, 41.6, 41.6, 41.6, 41.6, 41.6, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 37.4, 37.4,
           37.4, 37.4, 37.4, 37.4, 37.4, NoAngle, NoAngle, NoAngle},
          {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
           NoAngle, NoAngle, NoAngle, NoAngle, 40.3, 40.3, 40.3, 40.3}}};

    // Original MATLAB model constants with more digits than official Tgn document.
    //
    // const array<array<double, numberTaps>, numberClusters> PowerDelayProfileDb =
    //     {{{-2.6, -3, -3.5, -3.9, -4.5644301, -5.6551533, -6.9751533, -8.2951533, -9.8221791, -11.785521,
    //        -13.985521, -16.185521, -18.385521, -20.585521, -22.985195, -DBL_MAX, -DBL_MAX, -DBL_MAX},
    //       {-DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -1.8681171, -3.2849115, -4.5733656, -5.8619031, -7.1920408, -9.9304493,
    //        -10.343797, -14.353720, -14.767068, -18.776991, -19.982151, -22.446411, -DBL_MAX,  -DBL_MAX},
    //       {-DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -7.9044978,  -9.6851670,
    //        -14.260649, -13.812819, -18.603831, -18.192376, -22.834619, -DBL_MAX, -DBL_MAX, -DBL_MAX},
    //       {-DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX,
    //        -DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX, -20.673366, -20.574381, -20.7, -24.6}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AnglesOfDepartureDeg =
    //     {{{105.6434, 105.6434, 105.6434, 105.6434, 105.6434, 105.6434, 105.6434, 105.6434, 105.6434, 105.6434,
    //        105.6434, 105.6434, 105.6434, 105.6434, 105.6434, NoAngle, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, 293.1199, 293.1199, 293.1199, 293.1199, 293.1199, 293.1199,
    //        293.1199, 293.1199, 293.1199, 293.1199, 293.1199, 293.1199, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 61.9720, 61.9720,
    //        61.9720, 61.9720, 61.9720, 61.9720, 61.9720, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
    //        NoAngle, NoAngle, NoAngle, NoAngle, 275.7640, 275.7640, 275.7640, 275.7640}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AngleOfDepartureSpreadStandardDeviationsDeg =
    //     {{{36.1176, 36.1176, 36.1176, 36.1176, 36.1176, 36.1176, 36.1176, 36.1176, 36.1176, 36.1176,
    //        36.1176, 36.1176, 36.1176, 36.1176, 36.1176, NoAngle, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, 42.5299, 42.5299, 42.5299, 42.5299, 42.5299, 42.5299,
    //        42.5299, 42.5299, 42.5299, 42.5299, 42.5299, 42.5299, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 38.0096, 38.0096,
    //        38.0096, 38.0096, 38.0096, 38.0096, 38.0096, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
    //        NoAngle, NoAngle, NoAngle, NoAngle, 38.7026, 38.7026, 38.7026, 38.7026}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AnglesOfArrivalDeg =
    //     {{{163.7475, 163.7475, 163.7475, 163.7475, 163.7475, 163.7475, 163.7475, 163.7475, 163.7475, 163.7475,
    //        163.7475, 163.7475, 163.7475, 163.7475, 163.7475, NoAngle, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, 251.8792, 251.8792, 251.8792, 251.8792, 251.8792, 251.8792,
    //        251.8792, 251.8792, 251.8792, 251.8792, 251.8792, 251.8792, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 80.0240, 80.0240,
    //        80.0240, 80.0240, 80.0240, 80.0240, 80.0240, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
    //        NoAngle, NoAngle, NoAngle, NoAngle, 182.0, 182.0, 182.0, 182.0}}};
    //
    // const array<array<double, numberTaps>, numberClusters> AngleOfArrivalSpreadStandardDeviationsDeg =
    //     {{{35.8768, 35.8768, 35.8768, 35.8768, 35.8768, 35.8768, 35.8768, 35.8768, 35.8768, 35.8768,
    //        35.8768, 35.8768, 35.8768, 35.8768, 35.8768, NoAngle, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, 41.6812, 41.6812, 41.6812, 41.6812, 41.6812, 41.6812,
    //        41.6812, 41.6812, 41.6812, 41.6812, 41.6812, 41.6812, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, 37.4221, 37.4221,
    //        37.4221, 37.4221, 37.4221, 37.4221, 37.4221, NoAngle, NoAngle},
    //       {NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle, NoAngle,
    //        NoAngle, NoAngle, NoAngle, NoAngle, 40.3685, 40.3685, 40.3685, 40.3685}}};

    infoForTaps.resize(numberTaps);

    for(unsigned int i = 0; (i < numberTaps); i++) {
        InfoForTapType& tapInfo = infoForTaps[i];
        tapInfo.powerDelayProfileRelativePower = 0.0;

        for(unsigned int j = 0; (j < numberClusters); j++) {
            if (PowerDelayProfileDb[j][i] != -DBL_MAX) {
                assert(AnglesOfDepartureDeg[j][i] != NoAngle);
                assert(AnglesOfArrivalDeg[j][i] != NoAngle);

                tapInfo.powerDelayProfileRelativePower += ConvertToNonDb(PowerDelayProfileDb[j][i]);

                tapInfo.departureClusterInfo.resize(tapInfo.departureClusterInfo.size() + 1);
                InfoForClusterType& clusterInfo = tapInfo.departureClusterInfo.back();

                clusterInfo.relativePower = ConvertToNonDb(PowerDelayProfileDb[j][i]);
                clusterInfo.rayAngleRad = AnglesOfDepartureDeg[j][i] * DegToRadFactor;
                clusterInfo.rayAngleSpreadStandardDeviationRad =
                    AngleOfDepartureSpreadStandardDeviationsDeg[j][i] * DegToRadFactor;


                tapInfo.arrivalClusterInfo.resize(tapInfo.arrivalClusterInfo.size() + 1);
                InfoForClusterType& arrivalClusterInfo = tapInfo.arrivalClusterInfo.back();

                arrivalClusterInfo.relativePower = ConvertToNonDb(PowerDelayProfileDb[j][i]);
                arrivalClusterInfo.rayAngleRad = AnglesOfArrivalDeg[j][i] * DegToRadFactor;
                arrivalClusterInfo.rayAngleSpreadStandardDeviationRad =
                    AngleOfArrivalSpreadStandardDeviationsDeg[j][i] * DegToRadFactor;
            }
            else {
                assert(AnglesOfDepartureDeg[j][i] == NoAngle);
                assert(AnglesOfArrivalDeg[j][i] == NoAngle);
            }//if//
        }//for//
    }//for//

}//GetModelEParameters//



inline
void PerformTgacChannelSamplingRateExpansionOfTaps(
    const unsigned int expansionFactor,
    vector<InfoForTapType>& infoForTaps)
{
    assert(expansionFactor > 1);

    const double stepSize = (1.0 / expansionFactor);

    const vector<InfoForTapType> originalInfoForTaps = infoForTaps;

    infoForTaps.clear();

    for(unsigned int i = 0; (i < (originalInfoForTaps.size() - 1)); i++) {
        infoForTaps.push_back(originalInfoForTaps[i]);

        const double tapPowerDb =
            ConvertToDb(originalInfoForTaps[i].powerDelayProfileRelativePower);

        const double nextTapPowerDb =
            ConvertToDb(originalInfoForTaps[i+1].powerDelayProfileRelativePower);

        const double deltaPowerDb = (nextTapPowerDb - tapPowerDb);

        for(unsigned int j = 1; (j < expansionFactor); j++) {

            infoForTaps.push_back(originalInfoForTaps[i]);

            // Interpolate powers (in dB) for the intermediate taps.

            infoForTaps.back().powerDelayProfileRelativePower =
                ConvertToNonDb(tapPowerDb + ((stepSize * j) * deltaPowerDb));

        }//for//
    }//for//

    infoForTaps.push_back(originalInfoForTaps.back());

}//PerformTgacChannelSamplingRateExpansionOfTaps//




inline
void GetModelParameters(
    const ChannelModelChoicesType& modelChoice,
    const unsigned int tgacChannelSamplingRateExpansionFactor,
    double& kFactor,
    double& kFactorBreakpointMeters,
    vector<InfoForTapType>& infoForTaps)
{
    double kFactorDb;

    switch (modelChoice) {
    case ModelB:

        GetModelBParameters(kFactorDb, kFactorBreakpointMeters, infoForTaps);

        break;

    case ModelC:

        GetModelCParameters(kFactorDb, kFactorBreakpointMeters, infoForTaps);

        break;

    case ModelD:

        GetModelDParameters(kFactorDb, kFactorBreakpointMeters, infoForTaps);

        break;

    case ModelE:

        GetModelEParameters(kFactorDb, kFactorBreakpointMeters, infoForTaps);

        break;

    default:
        assert(false); abort();
    }//switch//

    kFactor = ConvertToNonDb(kFactorDb);

    if (tgacChannelSamplingRateExpansionFactor > 1) {

        PerformTgacChannelSamplingRateExpansionOfTaps(
            tgacChannelSamplingRateExpansionFactor, infoForTaps);

    }//if//

    // Computation of the power delay profile of the (LOS+NLOS) power
    // The PDP is defined as the time dispersion of the NLOS power. The addition
    // of the LOS component modifies the time dispersion of the total power.
    // LOS power only on first tap.

    double total = ((1.0 + kFactor) * infoForTaps[0].powerDelayProfileRelativePower);
    for(unsigned int i = 1; (i < infoForTaps.size()); i++) {
        total += infoForTaps[i].powerDelayProfileRelativePower;
    }//for//

    infoForTaps[0].normalizedPowerDelayProfilePower =
        ((1.0 + kFactor) * infoForTaps[0].powerDelayProfileRelativePower) / total;

    for(unsigned int i = 1; (i < infoForTaps.size()); i++) {
        infoForTaps[i].normalizedPowerDelayProfilePower =
            infoForTaps[i].powerDelayProfileRelativePower / total;
    }//for//

    // No LOS component table (beyond breakpoint) (don't use K-Factor).

    total = 0.0;
    for(unsigned int i = 0; (i < infoForTaps.size()); i++) {
        total += infoForTaps[i].powerDelayProfileRelativePower;
    }//for//

    for(unsigned int i = 0; (i < infoForTaps.size()); i++) {
        infoForTaps[i].normalizedPowerDelayProfilePowerNoLos =
            infoForTaps[i].powerDelayProfileRelativePower / total;
    }//for//

}//GetModelParameters//



// CalcCholeskyDecomposition is not general, i.e. it may be limited to a subset of the matrices
// that a Cholesky Decomposition can be applied.

inline
void CalcCholeskyDecomposition(
    const ublas::matrix<complex<double> >& inputMatrix,
    ublas::matrix<complex<double> >& outputMatrix)
{
    assert(inputMatrix.size1() == inputMatrix.size2());
    outputMatrix.resize(inputMatrix.size1(), inputMatrix.size1(), false);
    outputMatrix = ublas::zero_matrix<complex<double> >(inputMatrix.size1(), inputMatrix.size1());

    for (unsigned int i = 0; (i < inputMatrix.size1()); i++) {
        for (unsigned int j = 0; (j <= i); j++) {
            if (i == j) {
                assert(imag(inputMatrix(i,i)) == 0.0);

                double total = 0.0;
                for (unsigned int k = 0; k < i; k++) {
                    total += real(outputMatrix(i, k) * conj(outputMatrix(i, k)));
                }//for//

                assert(real(inputMatrix(i, i)) >= total);
                outputMatrix(i, i) = sqrt(real(inputMatrix(i, i)) - total);
            }
            else {
                assert(inputMatrix(i,j) == conj(inputMatrix(j,i)));

                complex<double> total = 0.0;

                for (unsigned int k = 0; k < j; k++) {
                    total += outputMatrix(i, k) * conj(outputMatrix(j, k));
                }//for//

                outputMatrix(i, j) =
                    ((1.0 / outputMatrix(j, j)) * (inputMatrix(i, j) - total));
            }//if//
        }//for//
    }//for//

}//CalcCholeskyDecomposition//

//--------------------------------------------------------------------------------------------------
//
// This is a clone of MATLAB filter function. Note that tjhe last digits will diverge from MATLAB
// results probably due to MATLAB likely uses larger numeric temporaries (greater than 64 bits) for the
// sub-computations.
//

template<size_t N> inline
void MatlabFilterFunction(
    const array<double, N>& b,
    const array<double, N>& a,
    const complex<double>& x,
    vector<complex<double> >& inOutFilterState,
    complex<double>& y)
{
    assert(b.size() == a.size());

    if (inOutFilterState.empty()) {
        inOutFilterState.resize((b.size() - 1), complex<double>(0.0, 0.0));
    }//if//

    assert(inOutFilterState.size() == (b.size() - 1));

    y = b[0] * x + inOutFilterState[0];

    for (unsigned int j = 0; (j < (inOutFilterState.size() - 1)); j++) {
        inOutFilterState[j] = inOutFilterState[j + 1] + ((b[j+1] * x) - (a[j+1] * y));
    }//for//

    inOutFilterState[inOutFilterState.size() - 1] =
        (b[inOutFilterState.size()] * x) - (a[inOutFilterState.size()] * y);

}//MatlabFilterFunction//


inline
void MakeBellFilterFadingValue(
    ScenSim::RandomNumberGenerator& aRandomNumberGenerator,
    vector<complex<double> >& inOutFilterState,
    complex<double>& /*out*/fadingValue)
{
    // Bell filter polynomial coefficients
    // fd = 1/300 of the normalized frequency

    const unsigned int filterOrder = 7;

    static const array<double, (filterOrder + 1)> bValues =
        {{2.785150513156437e-4, -1.289546865642764e-3, 2.616769929393532e-3, -3.041340177530218e-3,
          2.204942394725852e-3, -9.996063557790929e-4, 2.558709319878001e-4, -2.518824257145505e-5}};

    static const array<double, (filterOrder + 1)> aValues =
        {{1.0, -5.945307133332568, 1.481117656568614e1, -1.985278212976179e1,
          1.520727030904915e1, -6.437156952794267, 1.279595585941577, -6.279622049460144e-2}};

    const double random1 = aRandomNumberGenerator.GenerateRandomDouble();
    const double random2 = aRandomNumberGenerator.GenerateRandomDouble();
    double gaussian1;
    double gaussian2;
    ConvertToGaussianDistribution(random1, random2, gaussian1, gaussian2);

    const complex<double> noiseValue = complex<double>(((1.0/sqrt2) * gaussian1), gaussian2);

    MatlabFilterFunction(bValues, aValues, noiseValue, inOutFilterState, fadingValue);

}//MakeBellFilterFadingValue//


//NotUsed inline
//NotUsed void MakeBellFilterFadingVector(
//NotUsed     ScenSim::RandomNumberGenerator& aRandomNumberGenerator,
//NotUsed     vector<complex<double> >& inOutFilterState,
//NotUsed     vector<complex<double> >& /*out*/fadingVector)
//NotUsed {
//NotUsed     for(unsigned int i = 0; (i < fadingVector.size()); i++) {
//NotUsed         MakeBellFilterFadingValue(aRandomNumberGenerator, inOutFilterState, fadingVector[i]);
//NotUsed     }//for//
//NotUsed
//NotUsed }//MakeBellFilterFadingVector//


//--------------------------------------------------------------------------------------------------


inline
void CalculatePowerAngleFactorQsForTap(vector<InfoForClusterType>& clusterInfos)
{
    const unsigned int numberClusters = static_cast<unsigned int>(clusterInfos.size());

    //
    // Note: Equation from  Ref [2] (Eq 13) and
    // Q(k) = Q(0) * (Angle_SD(k) * power(k)) / (Angle_SD(0) * power(0));
    //

    for(unsigned int i = 0; (i < numberClusters); i++) {
        clusterInfos[i].laplacianRayAngleSpreadSdRad =
            AngleSpreadToLaplacianStandardDeviationTableRad.CalcValue(
                clusterInfos[i].rayAngleSpreadStandardDeviationRad);
    }//for//

    const double cluster0Factor = clusterInfos[0].laplacianRayAngleSpreadSdRad * clusterInfos[0].relativePower;

    vector<double> ratiosForQs(numberClusters);

    ratiosForQs[0] = 1.0;
    for (unsigned int i = 1; (i < numberClusters); i++) {
        ratiosForQs[i] =
            ((clusterInfos[i].laplacianRayAngleSpreadSdRad * clusterInfos[i].relativePower) /
             cluster0Factor);
    }//for//

    double total = 0.0;
    for(unsigned int i = 0; (i < numberClusters); i++) {
        total +=
            ((ratiosForQs[i]) * (1.0 - exp(-(sqrt2 * PI) / clusterInfos[i].laplacianRayAngleSpreadSdRad)));
    }//for//

    for(unsigned int i = 0; (i < numberClusters); i++) {
        //clusterInfos[i].normalizationFactorQ = min(1.0, (ratiosForQs[i] / total));
        clusterInfos[i].normalizationFactorQ = (ratiosForQs[i] / total);

    }//for//

}//CalculatePowerAngleFactorQsForTap//



inline
void CalculatePowerAngleFactorQs(vector<InfoForTapType>& infoForTaps)
{
    for(unsigned int i = 0; (i < infoForTaps.size()); i++) {
        CalculatePowerAngleFactorQsForTap(infoForTaps[i].departureClusterInfo);
        CalculatePowerAngleFactorQsForTap(infoForTaps[i].arrivalClusterInfo);
    }//for//
}


inline double Squared(const double& x) { return (x*x); }


// Using notation from reference [1].


inline
void CalcRxx(
    const vector<InfoForClusterType>& clusterInfos,
    const vector<double>& normalizedAntennaDistancesFromFirst,
    ublas::vector<double>& rxx)
{
    // Note: Equation from  Ref [2] (Eq 14).

    const double stoppingTermEpsilon = (100.0 * DBL_EPSILON);
    const unsigned int numberAntennas =
        static_cast<unsigned int>(normalizedAntennaDistancesFromFirst.size());
    const unsigned int numberClusters = static_cast<unsigned int>(clusterInfos.size());

    for(unsigned int i = 0; (i < normalizedAntennaDistancesFromFirst.size()); i++) {
        const double d = 2.0 * PI * normalizedAntennaDistancesFromFirst[i];

        rxx[i] = cyl_bessel_j(0, d);

        for(unsigned int k = 0; (k < numberClusters); k++) {
            const InfoForClusterType& clusterInfo = clusterInfos[k];
            const double& laplacianAsSdRad = clusterInfo.laplacianRayAngleSpreadSdRad;

            double total = 0.0;
            unsigned int j = 1;
            while(true) {

                // PAS definition range is -PI..PI (-180..180) and is hardcoded into equation.
                // (some 0.0 terms caused by this are commented out).

                const double term =
                    (((cyl_bessel_j(2 * j, d) / ((2.0 / Squared(laplacianAsSdRad)) + Squared(2 * j))) *
                      cos(2 * j * clusterInfo.rayAngleRad))
                     *
                     ((sqrt2 / laplacianAsSdRad) +
                      (exp((-PI * sqrt2) / laplacianAsSdRad) *
                       (/* (2 * j * sin(2 * j * PI)) */ -((sqrt2 / laplacianAsSdRad) * cos(2 * j * PI))))));

                total += term;

                if (std::abs(term) < stoppingTermEpsilon) {
                    break;
                }//if//

                j++;
                assert((j < 1000) && "Error: Did not converge");

            }//while//

            rxx[i] += 4.0 * (clusterInfo.normalizationFactorQ / (laplacianAsSdRad * sqrt2)) * total;

        }//for//
    }//for//

}//CalcRxx//



inline
void CalcRxy(
    const vector<InfoForClusterType>& clusterInfos,
    const vector<double>& normalizedAntennaDistancesFromFirst,
    ublas::vector<double>& rxy)
{
    // Note: Equation from Ref [2] (Eq 15).

    const double stoppingTermEpsilon = (100.0 * DBL_EPSILON);
    const unsigned int numberAntennas =
        static_cast<unsigned int>(normalizedAntennaDistancesFromFirst.size());
    const unsigned int numberClusters = static_cast<unsigned int>(clusterInfos.size());

    for(unsigned int i = 0; (i < normalizedAntennaDistancesFromFirst.size()); i++) {
        const double d = 2.0 * PI * normalizedAntennaDistancesFromFirst[i];

        rxy[i] = 0.0;

        for(unsigned int k = 0; (k < numberClusters); k++) {
            const InfoForClusterType& clusterInfo = clusterInfos[k];
            const double& laplacianAsSdRad = clusterInfo.laplacianRayAngleSpreadSdRad;

            double total = 0.0;
            unsigned int j = 0;
            while(true) {

                // PAS definition range is -PI..PI (-180..180) and is hardcoded into equation.
                // (some 0.0 terms caused by this are commented out).

                const double term =
                    (((cyl_bessel_j((2*j + 1), d) / ((2.0 / Squared(laplacianAsSdRad)) + Squared((2*j + 1)))) *
                      sin((2*j + 1) * clusterInfo.rayAngleRad))
                     *
                     ((sqrt2 / laplacianAsSdRad) -
                      (exp((-PI * sqrt2) / laplacianAsSdRad) *
                       (/* ((2*j + 1) * sin((2*j + 1) * PI)) + */
                        ((sqrt2 / laplacianAsSdRad) * cos((2*j + 1) * PI))))));

                total += term;

                if (std::abs(term) < stoppingTermEpsilon) {
                    break;
                }//if//

                j++;
                assert((j < 1000) && "Error: Did not converge");

            }//while//

            rxy[i] += 4.0 * (clusterInfo.normalizationFactorQ / (laplacianAsSdRad * sqrt2)) * total;

        }//for//
    }//for//

}//CalcRxy//



inline
void CalculateToeplitzMatrix(
    const ublas::vector<complex<double> >& aVector,
    ublas::matrix<complex<double> >& toeplitzMatrix)
{
    toeplitzMatrix.resize(aVector.size(), aVector.size(), false);

    // Set "0" row and column.

    toeplitzMatrix(0, 0) = aVector[0];

    for(unsigned int i = 1; (i < aVector.size()); i++) {
        toeplitzMatrix(0, i) = aVector[i];
    }//for//

    for(unsigned int i = 1; (i < aVector.size()); i++) {
        toeplitzMatrix(i, 0) = conj(aVector[i]);
    }//for//

    // Copy Row 0 elements along diagnonals.

    for (unsigned int i = 0; (i < aVector.size()); i++) {
        const complex<double> aValue = toeplitzMatrix(0, i);

        unsigned int j = 1;
        unsigned int k = i+1;
        while (k < aVector.size()) {
            toeplitzMatrix(j, k) = aValue;
            j++;
            k++;
        }//while//
    }//for//

    // Copy Column 0 elements along diagnonals.

    for (unsigned int i = 1; (i < aVector.size()); i++) {
        const complex<double> aValue = toeplitzMatrix(i, 0);

        unsigned int j = i+1;
        unsigned int k = 1;
        while (j < aVector.size()) {
            toeplitzMatrix(j, k) = aValue;
            j++;
            k++;
        }//while//
    }//for//

}//CalculateToeplitzMatrix//



inline
void CalculateKroneckerProduct(
    const ublas::matrix<complex<double> >& a,
    const ublas::matrix<complex<double> >& b,
    ublas::matrix<complex<double> >& c)
{
    c.resize((a.size1() * b.size1()), (a.size2() * b.size2()));

    for(unsigned int i = 0; (i < a.size1()); i++) {
        const unsigned int indexOffsetRow = i * static_cast<unsigned int>(b.size1());
        for(unsigned k = 0; (k < b.size1()); k++) {
            for(unsigned int j = 0; (j < a.size2()); j++) {
                const unsigned int indexOffsetCol = j * static_cast<unsigned int>(b.size2());
                for(unsigned int l = 0; (l < b.size2()); l++) {
                    c((indexOffsetRow + k), (indexOffsetCol + l)) = a(i,j) * b(k, l);
                }//for//
            }//for//
        }//for//
    }//for//

}//CalculateKroneckerProduct//



inline
void CalculateAnRMatrix(
    const vector<InfoForClusterType>& clusterInfos,
    const vector<double>& normalizedAntennaDistances,
    ublas::matrix<complex<double> >& rMatrix)
{
    const unsigned int numberAntennas = static_cast<unsigned int>(normalizedAntennaDistances.size());

    ublas::vector<double> rxx(numberAntennas);

    CalcRxx(
        clusterInfos,
        normalizedAntennaDistances,
        rxx);

    ublas::vector<double> rxy(numberAntennas);

    CalcRxy(
        clusterInfos,
        normalizedAntennaDistances,
        rxy);

    ublas::vector<complex<double> > rxxPlusRxyi(numberAntennas);

    for(unsigned int i = 0; (i < numberAntennas); i++) {
        rxxPlusRxyi[i] = complex<double>(rxx[i], rxy[i]);
    }//for//

    CalculateToeplitzMatrix(rxxPlusRxyi, rMatrix);

}//CalculateAnRMatrix//



inline
void CalculateSpatialCorrelationMatrix(
    const LinkParametersType& linkParameters,
    const InfoForTapType& tapInfo,
    ublas::matrix<complex<double> >& correlationMatrix)
{
    vector<double> normalizedDepartureNodeAntennaDistances(linkParameters.departureNodeNumberAntennas);

    double distance = 0.0;
    for(unsigned int i = 0; (i < linkParameters.departureNodeNumberAntennas); i++) {
        normalizedDepartureNodeAntennaDistances[i] = distance;
        distance += linkParameters.departureNodeAntennaSpacing;
    }//for//

    vector<double> normalizedArrivalNodeAntennaDistances(linkParameters.arrivalNodeNumberAntennas);

    distance = 0.0;
    for(unsigned int i = 0; (i < linkParameters.arrivalNodeNumberAntennas); i++) {
        normalizedArrivalNodeAntennaDistances[i] = distance;
        distance += linkParameters.arrivalNodeAntennaSpacing;
    }//for//

    ublas::matrix<complex<double> > rtxCorrelationMatrix;
    ublas::matrix<complex<double> > rrxCorrelationMatrix;

    CalculateAnRMatrix(
        tapInfo.departureClusterInfo,
        normalizedDepartureNodeAntennaDistances,
        rtxCorrelationMatrix);

    CalculateAnRMatrix(
        tapInfo.arrivalClusterInfo,
        normalizedArrivalNodeAntennaDistances,
        rrxCorrelationMatrix);

    ublas::matrix<complex<double> > kronProduct;

    CalculateKroneckerProduct(
        rtxCorrelationMatrix,
        rrxCorrelationMatrix,
        kronProduct);

    CalcCholeskyDecomposition(
        kronProduct,
        correlationMatrix);

}//CalculateRCorrelationMatrices//



inline
const ublas::matrix<complex<double> > CreateRiceSteeringMatrix(
    const unsigned int numberAntennasDepartureNode,
    const double& normalizedAntennaSpacingDepartureNode,
    const double& angleOfDepartureRad,
    const unsigned int numberAntennasArrivalNode,
    const double& normalizedAntennaSpacingArrivalNode,
    const double& angleOfArrivalRad)
{
    ublas::vector<complex<double> > steeringVectorDepartureNode(numberAntennasDepartureNode);

    steeringVectorDepartureNode[0] = 1.0;
    for (unsigned int i = 1; (i < numberAntennasDepartureNode); i++) {
        steeringVectorDepartureNode[i] =
            exp(static_cast<double>(i) *
                complex<double>(
                    0.0,
                    (2 * PI * normalizedAntennaSpacingDepartureNode * sin(angleOfDepartureRad))));
    }//for//

    ublas::vector<complex<double> > steeringVectorArrivalNode(numberAntennasArrivalNode);

    steeringVectorArrivalNode[0] = 1.0;
    for (unsigned int i = 1; (i < numberAntennasArrivalNode); i++) {
        steeringVectorArrivalNode[i] =
            exp(static_cast<double>(i) *
                complex<double>(
                    0.0,
                    (2 * PI * normalizedAntennaSpacingArrivalNode * sin(angleOfArrivalRad))));
    }//for//

    return (outer_prod(steeringVectorArrivalNode, steeringVectorDepartureNode));

}//CreateRiceSteeringMatrix//



inline
const ublas::vector<complex<double> > CreateLosRiceVector(
    const InfoForTapType& infoForFirstTap,
    const LinkParametersType& linkParameters)
{
    const ublas::matrix<complex<double> > riceMatrix =
        CreateRiceSteeringMatrix(
            linkParameters.departureNodeNumberAntennas,
            linkParameters.departureNodeAntennaSpacing,
            (PI / 4.0),
            linkParameters.arrivalNodeNumberAntennas,
            linkParameters.arrivalNodeAntennaSpacing,
            (PI / 4.0));

    ublas::vector<complex<double> > riceVectorLos(riceMatrix.size1() * riceMatrix.size2());

    for(unsigned int i = 0; (i < riceVectorLos.size()); i++) {
        riceVectorLos[i] = riceMatrix((i % riceMatrix.size1()),(i / riceMatrix.size1()));
    }//for//

    return (riceVectorLos);

}//CreateLosRiceVector//



inline
void CombineTapHMatricesIntoChannelMatrices(
    const LinkParametersType& linkParameters,
    const double& channelCenterFrequencyHz,
    const double& channelBandwidthHz,
    const unsigned int numberSubcarriers,
    const double& tapDelayIncrementSecs,
    const vector<OutputInfoForTapType>& infoForTaps,
    vector<ublas::matrix<complex<double> > >& mimoChannelMatrices)
{
    const double subcarrierBandwidthHz = channelBandwidthHz / numberSubcarriers;

    const unsigned int numTxAntennas = linkParameters.departureNodeNumberAntennas;
    const unsigned int numRxAntennas = linkParameters.arrivalNodeNumberAntennas;

    mimoChannelMatrices.resize(numberSubcarriers);
    for(unsigned int i = 0; (i < numberSubcarriers); i++) {
        ublas::matrix<complex<double> >& channelMatrix = mimoChannelMatrices[i];
        channelMatrix.resize(numRxAntennas, numTxAntennas, false);

        const int centeredSubcarrierIndex = i - (numberSubcarriers / 2);
        const double subcarrierFrequencyHz =
             channelCenterFrequencyHz + (subcarrierBandwidthHz * centeredSubcarrierIndex);

        for(unsigned int n = 0; (n < numRxAntennas); n++) {
            for(unsigned int m = 0; (m < numTxAntennas); m++) {
                channelMatrix(n, m) = 0.0;
                for(unsigned int t = 0; (t < infoForTaps.size()); t++) {
                    const TgnMimoChModel::OutputInfoForTapType& tapInfo = infoForTaps[t];
                    const double tapDelaySecs = t * tapDelayIncrementSecs;

                    const double phaseShift =
                        std::fmod((2.0 * PI * tapDelaySecs * subcarrierFrequencyHz), (2.0 * PI));

                    const complex<double> phaseShiftComplex =
                        complex<double>(cos(phaseShift), sin(phaseShift));

                    channelMatrix(n, m) += (tapInfo.hMatrix(n, m) * phaseShiftComplex);
                }//for//
            }//for//
        }//for//
    }//for//

}//CombineTapHMatricesIntoChannelMatrices//


}//namespace//


#endif


