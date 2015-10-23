// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef HFPMGLUE_NOTAVAILABLE_H
#define HFPMGLUE_NOTAVAILABLE_H

#ifndef HFPMGLUE_CHOOSER_H
#error "Include hfpmglue_chooser.h not hfpmglue_notavailable.h (indirect include only)"
#endif

#include "insiteglue_chooser.h"

namespace ScenSim {

class InsiteGeometry;

class HfpmPropagationLossCalculationModel: public SimplePropagationLossCalculationModel {
public:

    HfpmPropagationLossCalculationModel(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const double& initCarrierFrequencyMhz,
        const shared_ptr<InsiteGeometry>& initInsiteGeometryPtr,
        const string& moduleName,
        const double& initMaximumPropagationDistanceMeters = DBL_MAX,
        const bool initPropagationDelayIsEnabled = false,
        const int numberDataParallelCalculationThreads = 0,
        const InterfaceOrInstanceIdType& instanceId = "")
        :
        SimplePropagationLossCalculationModel(0.0)
    {
        cerr << "Error: HFPM propagation option is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables HFPM." << endl;
        exit(1);
    }

    double CalculatePropagationLossDb(
        const ObjectMobilityPosition& txAntennaPosition,
        const ObjectMobilityPosition& rxAntennaPosition,
        const double& notUsed) const
    {
        cerr << "Error: HFPM propagation option is not available." << endl;
        cerr << "Please confirm that the executable (simulator) enables HFPM." << endl;
        exit(1);
    }

};//HfpmPropagationLossCalculationModel//


}//namespace//

#endif
