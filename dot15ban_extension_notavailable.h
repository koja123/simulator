// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef DOT15BAN_EXTENSION_NOTAVAILABLE_H
#define DOT15BAN_EXTENSION_NOTAVAILABLE_H

#ifndef DOT15BAN_EXTENSION_CHOOSER_H
#error "Include dot15ban_extension_chooser.h (indirect include only)"
#endif

#include "scensim_proploss.h"
#include "scensim_prop.h"
#include "scensim_gis.h"
#include "scensim_mac.h"
#include "scensim_bercurves.h"

namespace Dot15Ban {

using std::cerr;
using std::endl;
using std::string;
using std::shared_ptr;
using ScenSim::SimplePropagationModelForNode;
using ScenSim::SimulationEngineInterface;
using ScenSim::ParameterDatabaseReader;
using ScenSim::NodeIdType;
using ScenSim::InterfaceIdType;
using ScenSim::RandomNumberGeneratorSeedType;
using ScenSim::NetworkLayer;
using ScenSim::BitOrBlockErrorRateCurveDatabase;
using ScenSim::MacLayer;

class Dot15BanPhy {
public:
    struct PropFrameType {};
};//Dot15BanPhy//

static inline
shared_ptr<MacLayer> Dot15BanFactory(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& simulationEngineInterfacePtr,
    const shared_ptr<SimplePropagationModelForNode<Dot15BanPhy::PropFrameType> >& propModelInterfacePtr,
    const shared_ptr<BitOrBlockErrorRateCurveDatabase>& berCurveDatabasePtr,
    const NodeIdType& nodeId,
    const InterfaceIdType& interfaceId,
    const shared_ptr<NetworkLayer>& networkLayerPtr,
    const RandomNumberGeneratorSeedType& nodeSeed)
{
    cerr << "Error: DOT15BAN module is not available." << endl;
    cerr << "Please confirm that the executable (simulator) enables DOT15BAN Module." << endl;
    exit(1);

    return shared_ptr<MacLayer>();
}//Dot15BanFactory//

}//namespace//


#endif
