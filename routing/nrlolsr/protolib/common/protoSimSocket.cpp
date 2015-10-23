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

#include "protoSocket.h"
#include "protoSimAgent.h"
#include "protoDebug.h"

#include <errno.h>  // for errno


namespace NrlProtolibPort //ScenSim-Port://
{ //ScenSim-Port://




#ifdef SIMULATE
const ProtoSocket::Handle ProtoSocket::INVALID_HANDLE = NULL;
#endif  // SIMULATE

ProtoSocket::ProtoSocket(ProtoSocket::Protocol theProtocol)
    : domain(SIM), protocol(theProtocol), state(CLOSED), handle(INVALID_HANDLE),
      port(-1),
#ifdef HAVE_IPV6
      flow_label(0),
#endif // HAVE_IPV6
      notifier(NULL), notify_output(false),
      listener(NULL), user_data(NULL)
{

}

ProtoSocket::~ProtoSocket()
{
    Close();
    if (listener)
    {
        delete listener;
        listener = NULL;
    }
}

bool ProtoSocket::SetNotifier(ProtoSocket::Notifier* theNotifier)
{
    ASSERT(!IsOpen());
    notifier = theNotifier;
    return true;
}  // end ProtoSocket::SetNotifier()


// WIN32 needs the address type determine IPv6 _or_ IPv4 socket domain
// Note: WIN32 can't do IPv4 on an IPV6 socket!
bool ProtoSocket::Open(UINT16               thePort,
                       ProtoAddress::Type   /*addrType*/,
                       bool                 bindOnOpen)
{
    if (IsOpen()) Close();
    ProtoSimAgent* simAgent = static_cast<ProtoSimAgent*>(notifier);
    ASSERT(simAgent);
    handle = simAgent->OpenSocket(*this);
    if (handle != nullptr)
    {
        state = IDLE;
        if (bindOnOpen)
        {
            if (!Bind(thePort))
            {
                Close();
                return false;
            }
        }

        return true;
    }
    else
    {
        fprintf(stderr, "ProtoSocket::Open() error creating socketAgent\n");
        return false;
    }
}  // end ProtoSocket::Open()

void ProtoSocket::Close()
{
    if (IsOpen())
    {
        ProtoSimAgent* simAgent = static_cast<ProtoSimAgent*>(notifier);
        ASSERT(simAgent);
        simAgent->CloseSocket(*this);
        handle = INVALID_HANDLE;
        state = CLOSED;
        port = -1;
    }
}  // end Close()

bool ProtoSocket::Shutdown()
{
    // (TBD)
    return true;
}  // end ProtoSocket::Shutdown()

bool ProtoSocket::Bind(UINT16 thePort, const ProtoAddress* /*localAddress*/)
{

  //ScenSim-Port:// if (!IsOpen()) Open(thePort, ProtoAddress::SIM, FALSE);  // I.T. Added 24/3/07
    if (IsOpen() && (port < 0)) //ScenSim-Port://
    { //ScenSim-Port://
        if (static_cast<ProtoSimAgent::SocketProxy*>(handle)->Bind(thePort))
        {
            port = thePort;
            return true;
        }
        else
        {
            return false;
        }
   } //ScenSim-Port://
        else // I.T. Taken out, logic replaced by above //ScenSim-Port://
      { //ScenSim-Port://
        return Open(thePort);  //ScenSim-Port://
      } //ScenSim-Port://
}  // end ProtoSocket::Bind()

bool ProtoSocket::Connect(const ProtoAddress& theAddress)
{
//ScenSim-Port://   if (!IsOpen()) Open(0, ProtoAddress::SIM, TRUE);  // I.T. Added 24/3/07 - use 0, as default port if not set

    state = CONNECTING; // the CONNECT is generated from the CONNECT Event

    if (static_cast<ProtoSimAgent::SocketProxy*>(handle)->Connect(theAddress))
    {
        destination = theAddress;
        return true;
    }
    else
    {
        return false;
    }
}  // end bool ProtoSocket::Connect()

void ProtoSocket::Disconnect()
{
        state = CLOSED;  // I.T. Added 24/3/07

}  // end ProtoSocket::Disconnect()

bool ProtoSocket::Listen(UINT16 thePort)
{
//ScenSim-Port://   if (!IsOpen()) Open(thePort, ProtoAddress::SIM, TRUE);  // I.T. Added 24/3/07

    state = LISTENING;  // I.T. Added 27/3/07

    if (static_cast<ProtoSimAgent::SocketProxy*>(handle)->Listen(thePort))
    {
        return true;
    }
    else
    {
        return false;
    }
}  // end ProtoSocket::Listen()

bool ProtoSocket::Accept(ProtoSocket* theSocket)
{
    if (static_cast<ProtoSimAgent::SocketProxy*>(handle)->Accept(theSocket))
    {
        // Note: ProtoSimAgent::SocketProxy _should_ set "destination" on Accept
        theSocket->state = CONNECTED; // I.T. Added - need to let the socket know that it is connected
        return true;
    }
    else
    {
        return false;
    }
}  // end ProtoSocket::Accept()

bool ProtoSocket::Send(const char*         buffer,
                       unsigned int&       numBytes)
{


    if (IsConnected())
    {
        return static_cast<ProtoSimAgent::SocketProxy*>(handle)->SendTo(buffer, numBytes, destination);
    }
    else
    {
        fprintf(stderr, "ProtoSocket::Send() error: socket not connected\n");
        return false;
    }
}  // end ProtoSocket::Send()

bool ProtoSocket::Recv(char*            buffer,
                       unsigned int&    numBytes)
{
    if (IsOpen())
    {
        ProtoAddress srcAddr;
        bool ret = static_cast<ProtoSimAgent::SocketProxy*>(handle)->RecvFrom(buffer, numBytes, srcAddr);
        destination=srcAddr; // I.T. 27/3/07 - set the sender address as the destination so it interfaces with ProtoSocket ...
        return ret;
    }
    else
    {
        fprintf(stderr, "ProtoSocket::Recv() error: socket not open\n");
        return false;
    }
}  // end ProtoSocket::Recv()

bool ProtoSocket::SendTo(const char*         buffer,
                         unsigned int        buflen,
                         const ProtoAddress& dstAddr)
{
    if (!IsOpen())
    {
        if (!Open())
        {
           fprintf(stderr, "ProtoSocket::SendTo() error opening socket\n");
           return false;
        }
    }
    else if (TCP == protocol)
    {
        if (!IsConnected() || dstAddr.IsEqual(destination)) Connect(dstAddr);
        if (IsConnected())
        {
            unsigned int put = 0;
            while (put < buflen)
            {
                unsigned int numBytes = buflen - put;
                if (Send(buffer+put, numBytes))
                {
                    put += numBytes;
                }
                else
                {
                    fprintf(stderr, "ProtoSocket::SendTo() error sending to connected socket\n");
                    return false;
                }
            }
            return true;
        }
    }
    return (static_cast<ProtoSimAgent::SocketProxy*>(handle))->SendTo(buffer, buflen, dstAddr);
}  // end ProtoSocket::SendTo()

bool ProtoSocket::RecvFrom(char*            buffer,
                           unsigned int&    numBytes,
                           ProtoAddress&    srcAddr)
{
    if (IsOpen())
    {
        return static_cast<ProtoSimAgent::SocketProxy*>(handle)->RecvFrom(buffer, numBytes, srcAddr);
    }
    else
    {
        fprintf(stderr, "ProtoSocket::RecvFrom() error: socket not open\n");
        return false;
    }
}  // end ProtoSocket::RecvFrom()

bool ProtoSocket::JoinGroup(const ProtoAddress& groupAddr,
                            const char*         /*interfaceName*/)
{
    if (!IsOpen())
    {
        if (!Open())
        {
           fprintf(stderr, "ProtoSocket::JoinGroup() error opening socket\n");
           return false;
        }
    }
    return static_cast<ProtoSimAgent::SocketProxy*>(handle)->JoinGroup(groupAddr);
}  // end ProtoSocket::JoinGroup()

bool ProtoSocket::LeaveGroup(const ProtoAddress& groupAddr,
                             const char*         /*interfaceName*/)
{
    if (IsOpen())
    {
        return static_cast<ProtoSimAgent::SocketProxy*>(handle)->LeaveGroup(groupAddr);
    }
    else
    {
        return true; // if we weren't open, we weren't joined
    }
}  // end ProtoSocket::LeaveGroup()

// (NOTE: These functions are being moved to ProtoRouteMgr
// Helper functions for group joins & leaves
/*bool ProtoSocket::GetInterfaceAddress(const char*         interfaceName,
                                      ProtoAddress::Type  addressType,
                                      ProtoAddress&       theAddress)
{
    ProtoSimAgent* simAgent = static_cast<ProtoSimAgent*>(notifier);
    if (simAgent)
    {
        return simAgent->GetLocalAddress(theAddress);
    }
    else
    {
        DMSG(0, "ProtoSocket::GetInterfaceAddress() Error: notifier not set\n");
        return false;
    }
}  // end ProtoSocket::GetInterfaceAddress()*/

//dummy get interface stuff functions use protoRouteMgr call instead when using simulation
bool ProtoSocket::GetInterfaceAddressList(const char*         interfaceName,
                                          ProtoAddress::Type  addressType,
                                          ProtoAddressList& addrList,
                                          unsigned int*       ifIndex)
{
  return false;
}

unsigned int ProtoSocket::GetInterfaceIndex(const char* interfaceName)
{
  return 0;
}
bool ProtoSocket::FindLocalAddress(ProtoAddress::Type addrType, ProtoAddress& theAddress)
{
  return false;
}

bool ProtoSocket::GetInterfaceName(unsigned int index, char* buffer, unsigned int buflen)
{
  return false;
}

bool ProtoSocket::GetInterfaceName(const ProtoAddress& ifAddr, char* buffer, unsigned int buflen)
{
  return false;
}


bool ProtoSocket::SetTTL(unsigned char ttl)
{
    static_cast<ProtoSimAgent::SocketProxy*>(handle)->SetTTL(ttl);
    return true;
}  // end ProtoSocket::SetTTL()

bool ProtoSocket::SetTOS(unsigned char tos)
{
   return false;
}  // end ProtoSocket::SetTOS()

bool ProtoSocket::SetBroadcast(bool broadcast)
{
    return true;
}  // end ProtoSocket::SetBroadcast()


bool ProtoSocket::SetLoopback(bool loopback)
{
    static_cast<ProtoSimAgent::SocketProxy*>(handle)->SetLoopback(loopback);
    return true;
}  // end ProtoSocket::SetLoopback()


bool ProtoSocket::SetFlowLabel(UINT32 label) //ScenSim-Port://
{ //ScenSim-Port://
    static_cast<ProtoSimAgent::SocketProxy*>(handle)->SetFlowLabel(label); //ScenSim-Port://
    return true; //ScenSim-Port://
} //ScenSim-Port://


bool ProtoSocket::SetMulticastInterface(const char* /*interfaceName*/)
{
    return true;
}  // end ProtoSocket::SetMulticastInterface()

bool ProtoSocket::SetReuse(bool state)
{
    return true;
}  // end ProtoSocketError::SetReuse()

bool ProtoSocket::SetTxBufferSize(unsigned int bufferSize)
{
    static_cast<ProtoSimAgent::SocketProxy*>(handle)->SetTxBufferSize(bufferSize); // I.T. Added 26/3/07
    return true;
}  // end ProtoSocket::SetTxBufferSize()

unsigned int ProtoSocket::GetTxBufferSize()
{
    return static_cast<ProtoSimAgent::SocketProxy*>(handle)->GetTxBufferSize(); // I.T. Added 26/3/07
}  // end ProtoSocket::GetTxBufferSize()

bool ProtoSocket::SetRxBufferSize(unsigned int bufferSize)
{
    static_cast<ProtoSimAgent::SocketProxy*>(handle)->SetRxBufferSize(bufferSize); // I.T. Added 26/3/07
    return false;
}  // end ProtoSocket::SetRxBufferSize()

unsigned int ProtoSocket::GetRxBufferSize()
{
    return static_cast<ProtoSimAgent::SocketProxy*>(handle)->GetRxBufferSize(); // I.T. Added 26/3/07
    return 0;
}  // end ProtoSocket::GetRxBufferSize()

bool ProtoSocket::SetBlocking(bool /*blocking*/)
{
    return true;
}

ProtoAddress::Type ProtoSocket::GetAddressType()
{
    return ProtoAddress::SIM;
}  // end ProtoSocket::GetAddressType()


bool ProtoSocket::UpdateNotification()
{
    return true;
}  // end ProtoSocket::UpdateNotification()

void ProtoSocket::OnNotify(ProtoSocket::Flag theFlag)
{
#ifndef OPNET // JPH 5/18/2007
//ScenSim-Port://printf("ProtoSimSocket::OnNotify() called\n");
#endif // OPNET
    Event event = INVALID_EVENT;
    if (NOTIFY_INPUT == theFlag)
    {
        switch (state)
        {
            case CLOSED:
                break;
            case IDLE:
                event = RECV;
                break;
            case CONNECTING:
                break;
            case LISTENING:
                // (TBD) check for error
                event = ACCEPT;
                break;
            case CONNECTED:
                event = RECV;
                break;
        }
    }
    else if (NOTIFY_OUTPUT == theFlag)
    {
        ASSERT(NOTIFY_OUTPUT == theFlag);
        switch (state)
        {
            case CLOSED:
                break;
            case IDLE:
                event = SEND; // I.T. Added 26/3/07 - only notify SEND if requested
                break;
            case CONNECTING:
                event=CONNECT;
                state = CONNECTED; // I.T. Added - need to let client know we are connected
                break;
            case LISTENING:
                break;
            case CONNECTED:
                event = SEND;  // I.T. Added 26/3/07 - only notify SEND if requested
                break;
        }
    }
    else  // NOTIFY_NONE  (connection was ended)
    {
        switch(state)
        {
            case CONNECTING:
            case CONNECTED:
                event = DISCONNECT;
                state = IDLE;
                break;
            default:
                break;
        }
    }
    if (listener) {
        if ((event==SEND) && (!notify_output)) {}
        else
            listener->on_event(*this, event);
        }
}  // end ProtoSocket::OnNotify()


ProtoSocket::List::List()
 : head(NULL)
{
}

ProtoSocket::List::~List()
{
    Destroy();
}

void ProtoSocket::List::Destroy()
{
    Item* next = head;
    while (next)
    {
        Item* current = next;
        next = next->GetNext();
        delete current->GetSocket();
        delete current;
    }
}  // end ProtoSocket::List::Destroy()

bool ProtoSocket::List::AddSocket(ProtoSocket& theSocket)
{
    Item* item = new Item(&theSocket);
    if (item)
    {
        item->SetNext(head);
        head = item;
        return true;
    }
    else
    {
        DMSG(0, "ProtoSocket::List::AddSocket() new Item error: %s\n", strerror(errno));
        return false;
    }
}  // end ProtoSocket::List::AddSocket()

void ProtoSocket::List::RemoveSocket(ProtoSocket& theSocket)
{
    Item* item = head;
    while (item)
    {
        if (&theSocket == item->GetSocket())
        {
            Item* prev = item->GetPrev();
            Item* next = item->GetNext();
            if (prev)
                prev->SetNext(next);
            else
                head = next;
            if (next) next->SetPrev(prev);
            delete item;
            break;
        }
        item = item->GetNext();
    }
}  // end ProtoSocket::List::AddSocket()

ProtoSocket::List::Item::Item(ProtoSocket* theSocket)
 : socket(theSocket), prev(NULL), next(NULL)
{
}

ProtoSocket::List::Iterator::Iterator(const ProtoSocket::List& theList)
 : list(theList), next(theList.head)
{
}



} //namespace// //ScenSim-Port://

