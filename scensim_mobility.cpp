// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_mobility.h"
#include "its_extension_chooser.h"

namespace ScenSim {

//--------------------------------------------------------------------
// Dummy Mobiliity Model
//--------------------------------------------------------------------

class DummyMobilityModel : public ObjectMobilityModel {
public:
    DummyMobilityModel() {}

    ~DummyMobilityModel() {}

protected:

    virtual void GetUnadjustedPositionForTime(
        const TimeType& snapshotTime,
        ObjectMobilityPosition& position)
    {
        position = ObjectMobilityPosition(ZERO_TIME, ZERO_TIME, DBL_MAX/*invalid position x*/, DBL_MAX/*invalid position y*/, 0.0, false, INVALID_NODEID);
    }

};//DummyMobilityModel//


//--------------------------------------------------------
// CreateAntennaMobilityModel
//--------------------------------------------------------

shared_ptr<ObjectMobilityModel> CreateAntennaMobilityModel(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const NodeIdType& nodeId,
    const InterfaceOrInstanceIdType& interfaceId,
    const RandomNumberGeneratorSeedType& mobilitySeed,
    InorderFileCache& mobilityFileCache,
    const shared_ptr<GisSubsystem>& theGisSubsystemPtr)
{
    string mobilityModelString;
    if (theParameterDatabaseReader.ParameterExists(
            "mobility-model", nodeId, interfaceId)) {
        mobilityModelString = theParameterDatabaseReader.ReadString(
            "mobility-model", nodeId, interfaceId);
    }
    else {
        return shared_ptr<ObjectMobilityModel>(new DummyMobilityModel());
    }//if//


    ConvertStringToLowerCase(mobilityModelString);


    const double defaultMobilityGranularityMeters = 1.0;
    double mobilityGranularityMeters = defaultMobilityGranularityMeters;
    if (theParameterDatabaseReader.ParameterExists(
            "mobility-granularity-meters", nodeId, interfaceId)) {
        mobilityGranularityMeters =
            theParameterDatabaseReader.ReadDouble(
                "mobility-granularity-meters", nodeId, interfaceId);
    }

    ObjectMobilityModel::MobilityObjectIdType mobilityObjectId(nodeId);
    if (theParameterDatabaseReader.ParameterExists(
            "mobility-trace-file-object-id", nodeId, interfaceId)) {
        mobilityObjectId =
            theParameterDatabaseReader.ReadInt(
                "mobility-trace-file-object-id", nodeId, interfaceId);
    }

    if (mobilityModelString == "trace-file") {

        if (!theParameterDatabaseReader.ParameterExists(
                "mobility-trace-file", nodeId, interfaceId)) {
            cerr
                << "Error: The parameter ""mobility-trace-file"" "
                << "was not found for node: " << nodeId << endl;
            exit(1);
        }
        const string traceFileString =
            theParameterDatabaseReader.ReadString(
                "mobility-trace-file", nodeId, interfaceId);

        bool supportsCreationAndDeletion = false;

        if (theParameterDatabaseReader.ParameterExists(
                "mobility-trace-file-supports-creation-and-deletion", nodeId, interfaceId)) {

            supportsCreationAndDeletion =
                theParameterDatabaseReader.ReadBool(
                    "mobility-trace-file-supports-creation-and-deletion", nodeId, interfaceId);
        }

        return shared_ptr<ObjectMobilityModel>(
            new TraceFileMobilityModel(
                theParameterDatabaseReader,
                nodeId,
                interfaceId,
                mobilityFileCache,
                traceFileString,
                mobilityObjectId,
                mobilityGranularityMeters,
                theGisSubsystemPtr,
                supportsCreationAndDeletion));
    }
    else if (mobilityModelString == "random-waypoint") {

        return shared_ptr<ObjectMobilityModel>(
            new RandomWaypointMobilityModel(
                theParameterDatabaseReader,
                nodeId,
                interfaceId,
                mobilitySeed,
                mobilityFileCache,
                mobilityGranularityMeters,
                theGisSubsystemPtr));

    }
    else if (mobilityModelString == "gis-based-random-waypoint") {

        assert(theGisSubsystemPtr != nullptr);

        return shared_ptr<ObjectMobilityModel>(
            new GisBasedRandomWaypointMobilityModel(
                theParameterDatabaseReader,
                mobilityObjectId,
                nodeId,
                interfaceId,
                mobilitySeed,
                mobilityFileCache,
                mobilityGranularityMeters,
                theGisSubsystemPtr));

    }
    else if (mobilityModelString == "mates") {

        return shared_ptr<ObjectMobilityModel>(
            new Mates::MatesMobilityModel());

    }
    else if (mobilityModelString == "custom") {

        return shared_ptr<ObjectMobilityModel>();

    }
    else if (mobilityModelString == "stationary") {

        return shared_ptr<ObjectMobilityModel>(
            new StationaryMobilityModel(
                theParameterDatabaseReader,
                nodeId,
                interfaceId,
                mobilityFileCache,
                theGisSubsystemPtr));

    }//if//

    cerr << "Error: Mobility Model: "
         << mobilityModelString << " is invalid." << endl;
    exit(1);

    return shared_ptr<ObjectMobilityModel>();
}//CreateAntennaMobilityModel//

}; // namespace ScenSim
