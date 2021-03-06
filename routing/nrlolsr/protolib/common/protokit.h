/*********************************************************************
 *
 * AUTHORIZATION TO USE AND DISTRIBUTE
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that: 
 *
 * (1) source code distributions retain this paragraph in its entirety, 
 *  
 * (2) distributions including binary code include this paragraph in
 *     its entirety in the documentation or other materials provided 
 *     with the distribution, and 
 *
 * (3) all advertising materials mentioning features or use of this 
 *     software display the following acknowledgment:
 * 
 *      "This product includes software written and developed 
 *       by Code 5520 of the Naval Research Laboratory (NRL)." 
 *         
 *  The name of NRL, the name(s) of NRL  employee(s), or any entity
 *  of the United States Government may not be used to endorse or
 *  promote  products derived from this software, nor does the 
 *  inclusion of the NRL written and developed software  directly or
 *  indirectly suggest NRL or United States  Government endorsement
 *  of this product.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 ********************************************************************/

#ifndef _PROTOKIT
#define _PROTOKIT

#include "protoVersion.h"
#include "protoDefs.h"
#include "protoSocket.h"
#include "protoDebug.h"
#include "protoTimer.h"
#include "protoTree.h"
#include "protoRouteTable.h"
#include "protoRouteMgr.h"
#include "protoPipe.h"

#ifdef SIMULATE

#ifdef SCENSIM_NRLOLSR //ScenSim-Port:// 
#include "nrlprotolibglue.h" //ScenSim-Port:// 
#endif //SCENSIM_NRLOLSR //ScenSim-Port://

#ifdef NS2
#include "nsProtoSimAgent.h"
#endif // NS2

#else
#include "protoDispatcher.h"
#include "protoApp.h"
#endif // if/else SIMULATE

#endif // _PROTOKIT
