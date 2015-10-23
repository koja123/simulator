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

#include "protoSimAgent.h"



namespace NrlProtolibPort //ScenSim-Port://
{ //ScenSim-Port:// 



bool ProtoMessageSink::HandleMessage(const char* txBuffer, unsigned int len,const ProtoAddress& srcAddr) {return true;}

ProtoMessageSink::~ProtoMessageSink()
{
}

ProtoSimAgent::ProtoSimAgent()
// : timer_mgr(*this)
{
}

ProtoSimAgent::~ProtoSimAgent()
{
}

/*ProtoSimAgent::TimerMgr::TimerMgr(ProtoSimAgent& theAgent)
 : agent(theAgent)
{
}*/ 

ProtoSimAgent::SocketProxy::SocketProxy()
 : proto_socket(NULL)
{
}

ProtoSimAgent::SocketProxy::~SocketProxy()
{
}

ProtoSimAgent::SocketProxy::List::List()
 : head(NULL)
{
}

void ProtoSimAgent::SocketProxy::List::Prepend(SocketProxy& proxy)
{
    proxy.SetPrev(NULL);
    proxy.SetNext(head);
    if (head) head->SetPrev(&proxy);
    head = &proxy;
}  // end ProtoSimAgent::SocketProxy::List::Prepend()

void ProtoSimAgent::SocketProxy::List::Remove(SocketProxy& proxy)
{
    SocketProxy* prev = proxy.GetPrev();
    SocketProxy* next = proxy.GetNext();
    if (prev)
        prev->SetNext(next);
    else
        head = next;
    if (next)
        next->SetPrev(prev);
}  // end ProtoSimAgent::SocketProxy::List::Remove()

ProtoSimAgent::SocketProxy* ProtoSimAgent::SocketProxy::List::FindProxyByPort(UINT16 thePort)
{
    SocketProxy* next = head;
    while (next)
    {
        if (next->GetPort() == thePort)
            return next;
        else
            next = next->GetNext();   
    }
    return NULL;
}  // end ProtoSimAgent::SocketProxy::List::FindProxyByPort()

// I.T. Added 27th Feb 2007
ProtoSimAgent::SocketProxy* ProtoSimAgent::SocketProxy::List::FindProxyByIdentifier(int socketProxyID)
{
    SocketProxy* next = head;
    while (next)
    {
        if (next->GetSocketProxyID() == socketProxyID)
            return next;
        else
            next = next->GetNext();   
    }
    return NULL;
}  // end OpnetProtoSimProcess::TcpSocketProxy::TcpSockList::FindProxyByConn()



} //namespace// //ScenSim-Port://



