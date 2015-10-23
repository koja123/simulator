// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#include "scensim_netaddress.h"

namespace ScenSim {

const uint64_t Ipv6NetworkAddress::ipv6BroadcastAddressHighBits = 0xFFFFFFFFFFFFFFFFULL;
const uint64_t Ipv6NetworkAddress::ipv6BroadcastAddressLowBits = 0xFFFFFFFFFFFFFFFFULL;
const uint64_t Ipv6NetworkAddress::ipv6AnyAddressHighBits = 0x0000000000000000ULL;
const uint64_t Ipv6NetworkAddress::ipv6AnyAddressLowBits = 0x0000000000000000ULL;
const Ipv6NetworkAddress Ipv6NetworkAddress::anyAddress = Ipv6NetworkAddress::ReturnAnyAddress();
const Ipv6NetworkAddress Ipv6NetworkAddress::broadcastAddress = Ipv6NetworkAddress::ReturnTheBroadcastAddress();
const Ipv6NetworkAddress Ipv6NetworkAddress::invalidAddress = Ipv6NetworkAddress::anyAddress;

const Ipv4NetworkAddress Ipv4NetworkAddress::anyAddress = Ipv4NetworkAddress::ReturnAnyAddress();
const Ipv4NetworkAddress Ipv4NetworkAddress::broadcastAddress = Ipv4NetworkAddress::ReturnTheBroadcastAddress();
const Ipv4NetworkAddress Ipv4NetworkAddress::invalidAddress = Ipv4NetworkAddress::anyAddress;

}//namespace//


