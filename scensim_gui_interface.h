// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_GUI_INTERFACE_H
#define SCENSIM_GUI_INTERFACE_H

#include "scensim_engine.h"
#include "scensim_netsim.h"

namespace ScenSim {

using std::auto_ptr;
using std::shared_ptr;

class GuiInterfacingSubsystem {
public:
    static const int defaultPortNumber = 5000;
    static const int defaultPauseCommandPortNumber = 5001;

    GuiInterfacingSubsystem(
        const shared_ptr<SimulationEngine>& simulationEnginePtr,
        const unsigned short int guiPortNumber = defaultPortNumber,
        const unsigned short int guiPauseCommandPortNumber = defaultPauseCommandPortNumber);

    void RunInEmulationMode();

    void GetAndExecuteGuiCommands(
        ParameterDatabaseReader& theParameterDatabaseReader,
        NetworkSimulator& theNetworkSimulator,
        TimeType& executeSimUntilTime);

    void SendStreamedOutput(const NetworkSimulator& theNetworkSimulator);

    ~GuiInterfacingSubsystem();

private:
    void SendToGui(const string& output);


    // Abbreviation: impl = implementation.

    class Implementation;
    auto_ptr<Implementation> implPtr;

};//GuiInterfacingSubsystem//


}//namespace//

#endif




