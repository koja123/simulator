// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_MAIN_FUN_SUPPORT_H
#define SCENSIM_MAIN_FUN_SUPPORT_H

// Purpose of this header file is provide simple functions to reused in custom main() functions.

#include <iostream>
#include <fstream>
#include "randomnumbergen.h"
#include "scensim_stats.h"
#include "sysstuff.h"


namespace ScenSim {

using std::cerr;
using std::endl;
using std::cout;

inline
void MainFunctionArgvProcessingBasicSequentialOnly(
    int argc,
    const char * const argv[],
    string& configFileName,
    bool& isControlledByGui,
    bool& seedIsSet,
    RandomNumberGeneratorSeedType& runSeed)
{
    SetCpuFloatingPointBehavior();

    isControlledByGui = false;
    seedIsSet = false;

    if ((argc < 2) || (argc > 5)) {
        cout << "Simulator Command Format: sim <configfile> -g -seed <seed number>" << endl;
        exit(1);
    }//if//

    int i = 2;
    while (i < argc) {
        string option(argv[i]);

        if (option == "-g") {
            isControlledByGui = true;
        }
        else if (option == "-b") {

            // batch simulation mode controlled by GUI
        }
        else if (option == "-seed") {

            ++i;
            if (i == argc) {
                cerr << "No seed value parameter: Example: -seed 123" << endl;
                exit(1);
            }

            string seedString(argv[i]);
            std::istringstream seedStream(seedString);
            seedStream >> runSeed;
            if (seedStream.fail()) {
                cerr << "Bad seed value: " << seedString << endl;
                exit(1);
            }

            assert(seedStream.eof());

            seedIsSet = true;
        }
        else {
            cerr << "Bad command line parameter: " << option << endl;
            exit(1);

        }//if//

        ++i;

    }//while//

    configFileName.assign(argv[1]);

}//MainFunctionArgvProcessingBasicSequentialOnly//



inline
void MainFunctionArgvProcessingEmulatorVersion(
    int argc,
    const char * const argv[],
    string& configFileName,
    bool& isControlledByGui,
    bool& partitionIndexIsSet,
    unsigned int& partitionIndex,
    bool& seedIsSet,
    RandomNumberGeneratorSeedType& runSeed)
{
    SetCpuFloatingPointBehavior();

    isControlledByGui = false;
    partitionIndexIsSet = false;
    seedIsSet = false;

    if ((argc < 2) || (argc > 8)) {
        cout << "Simulator Command Format: sim <configfile> -g -seed <seed number> -p <partition index (0-based)>" << endl;
        exit(1);
    }//if//

    int i = 2;
    while (i < argc) {
        string option(argv[i]);

        if (option == "-g") {
            isControlledByGui = true;
        }
        else if (option == "-b") {

            // batch simulation mode controlled by GUI
        }
        else if (option == "-p") {
            ++i;
            if (i == argc) {
                cerr << "No partition parameter value: Example: -p 0" << endl;
                exit(1);
            }

            string partitionIndexString(argv[i]);
            std::istringstream partitionIndexStream(partitionIndexString);
            partitionIndexStream >> partitionIndex;
            if (partitionIndexStream.fail()) {
                cerr << "Bad partition index parameter: " << partitionIndexString << endl;
                exit(1);
            }//if//

            assert(partitionIndexStream.eof());
            partitionIndexIsSet = true;
        }
        else if (option == "-seed") {

            ++i;
            if (i == argc) {
                cerr << "No seed value parameter: Example: -seed 123" << endl;
                exit(1);
            }

            string seedString(argv[i]);
            std::istringstream seedStream(seedString);
            seedStream >> runSeed;
            if (seedStream.fail()) {
                cerr << "Bad seed value: " << seedString << endl;
                exit(1);
            }

            assert(seedStream.eof());

            seedIsSet = true;
        }
        else {
            cerr << "Bad command line parameter: " << option << endl;
            exit(1);

        }//if//

        ++i;

    }//while//

    configFileName.assign(argv[1]);

}//MainFunctionArgvProcessingEmulatorVersion//



inline
void MainFunctionArgvProcessingBasicParallelVersion1(
    int argc,
    const char * const argv[],
    string& configFileName,
    bool& isControlledByGui,
    unsigned int& numberParallelThreads,
    bool& runSequentially,
    bool& seedIsSet,
    RandomNumberGeneratorSeedType& runSeed)
{
    SetCpuFloatingPointBehavior();

    isControlledByGui = false;
    numberParallelThreads = 1;
    runSequentially = true;
    seedIsSet = false;

    if ((argc < 2) || (argc > 7)) {
        cout << "Simulator Command Format: sim <configfile> -g -n <number threads> -seed <seed number>" << endl;
        exit(1);
    }//if//

    int i = 2;
    while (i < argc) {
        string option(argv[i]);

        if (option == "-g") {
            isControlledByGui = true;
        }
        else if (option == "-b") {

            // batch simulation mode controlled by GUI
        }
        else if (option == "-n") {
            ++i;
            if (i == argc) {
                cerr << "No number thread parameter: Example: -n 4" << endl;
                exit(1);
            }

            string numberThreadsString(argv[i]);
            std::istringstream numberThreadsStream(numberThreadsString);
            numberThreadsStream >> numberParallelThreads;
            if (numberThreadsStream.fail()) {
                cerr << "Bad number thread parameter: " << numberThreadsString << endl;
                exit(1);
            }//if//

            assert(numberThreadsStream.eof());

            if (numberParallelThreads == 0) {
                cerr << "Number threads  can't be 0 (-n 0)" << endl;
                exit(1);

            }//if//

            runSequentially = false;
        }
        else if (option == "-seed") {

            ++i;
            if (i == argc) {
                cerr << "No seed value parameter: Example: -seed 123" << endl;
                exit(1);
            }

            string seedString(argv[i]);
            std::istringstream seedStream(seedString);
            seedStream >> runSeed;
            if (seedStream.fail()) {
                cerr << "Bad seed value: " << seedString << endl;
                exit(1);
            }

            assert(seedStream.eof());

            seedIsSet = true;
        }
        else {
            cerr << "Bad command line parameter: " << option << endl;
            exit(1);

        }//if//

        ++i;

    }//while//

    configFileName.assign(argv[1]);

}//MainArgvProcessingBasicParallelVersion1//




inline
void SetupFileControlledStats(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    RuntimeStatisticsSystem& theRuntimeStatisticsSystem,
    StatViewCollection& theStatViewCollection,
    string& statsOutputFilename,
    bool& noDataOutputIsEnabled)
{
    if (theParameterDatabaseReader.ParameterExists("statistics-configuration-file")) {
        string statConfigFilename =
            theParameterDatabaseReader.ReadString("statistics-configuration-file");

        ReadStatConfigFile(
            theRuntimeStatisticsSystem,
            statConfigFilename,
            theStatViewCollection);

        if (!theParameterDatabaseReader.ParameterExists("statistics-output-file")) {
            cerr << "Warning: No statistics-output-file defined." << endl;
            statsOutputFilename = "";
        }
        else {
            statsOutputFilename = theParameterDatabaseReader.ReadString("statistics-output-file");

            // Open/Create file to check for any problems and delete any old data  and then close it.

            std::ofstream tempFile(statsOutputFilename.c_str());

            if (!tempFile.good()) {
                cerr << "Error: could not open statistics output file: " << statsOutputFilename << endl;
                exit(1);
            }//if//

            tempFile.close();

        }//if//

        noDataOutputIsEnabled = true;//default//
        if (theParameterDatabaseReader.ParameterExists("statistics-output-for-no-data")) {
            noDataOutputIsEnabled = theParameterDatabaseReader.ReadBool("statistics-output-for-no-data");
        }//if//

    }//if//
}//SetupFileControlledStats//




}//namespace//

#endif
