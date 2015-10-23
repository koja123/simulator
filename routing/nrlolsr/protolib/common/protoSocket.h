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


#ifndef _PROTO_SOCKET
#define _PROTO_SOCKET

#include "protoAddress.h"
#include "protoDebug.h"  // temp


namespace NrlProtolibPort //ScenSim-Port://
{ //ScenSim-Port://


#ifdef WIN32
/*#ifdef _WIN32_WCE
#ifndef LPGUID
#include <guiddef.h>
#endif // !LPGUID
#endif // _WIN32_WCE*/
#ifndef __LPGUID_DEFINED__
#define __LPGUID_DEFINED__
typedef GUID *LPGUID;
#endif
#include <winsock2.h>  // for SOCKET type, etc
#else
#include <errno.h> // for errno
#endif // if/else WIN32/UNIX
/*!
Network socket container class that provides
consistent interface for use of operating
system (or simulation environment) transport
sockets. Provides support for asynchronous
notification to ProtoSocket::Listeners.  The
ProtoSocket class may be used stand-alone, or
with other classes described below.  A
ProtoSocket may be instantiated as either a
UDP or TCP socket.
*/

class ProtoSocket
{
    public:
        // Type definitions
        enum Domain
        {
            LOCAL,
            IPv4,
            IPv6
#ifdef SIMULATE
            ,SIM
#endif // SIMULATE
        };
        enum Protocol {INVALID_PROTOCOL, UDP, TCP, RAW};
        enum State {CLOSED, IDLE, CONNECTING, LISTENING, CONNECTED};
#ifdef SIMULATE
        class Proxy {};
        typedef Proxy* Handle;
#else
#ifdef WIN32
        typedef SOCKET Handle;
#else
        typedef int Handle;
#endif // if/else WIN32/UNIX
#endif // if/else SIMULATE
        static const Handle INVALID_HANDLE;

        ProtoSocket(Protocol theProtocol);
        virtual ~ProtoSocket();

        // Control methods
        bool Open(UINT16                thePort = 0,
                  ProtoAddress::Type    addrType = ProtoAddress::IPv4,
                  bool                  bindOnOpen = true);
        bool Bind(UINT16 thePort = 0, const ProtoAddress* localAddr = NULL);
        bool Connect(const ProtoAddress& theAddress);
        void Disconnect();
        bool Listen(UINT16 thePort = 0);
        void Ignore();
        bool Accept(ProtoSocket* theSocket = NULL);
        bool Shutdown();
        void Close();
        bool JoinGroup(const ProtoAddress&  groupAddr,
                       const char*          interfaceName = NULL);
        bool LeaveGroup(const ProtoAddress& groupAddr,
                        const char*         interfaceName = NULL);

        void SetState(State st){state = st;}  // JPH 7/14/06 - for tcp development testing

        // Status methods
        Domain GetDomain() const {return domain;}
        ProtoAddress::Type  GetAddressType();
        Protocol GetProtocol() const {return protocol;}
        State GetState() const {return state;}
        Handle GetHandle() const {return handle;}
        UINT16 GetPort() const {return port < 0 ? 0 : static_cast<UINT16>(port);}
        bool IsOpen() const {return (CLOSED != state);}
        bool IsBound() const {return (IsOpen() ? (port >= 0) : false);}
        bool IsConnected() const {return (CONNECTED == state);}
        bool IsConnecting() const {return (CONNECTING == state);}
        bool IsIdle() const {return (IDLE == state);}
        bool IsClosed() const {return (CLOSED == state);}
        bool IsListening() const {return (LISTENING == state);}

#ifdef OPNET
        ProtoAddress& GetDestination() {return destination;}
#else
        const ProtoAddress& GetDestination() const {return destination;}
#endif  // OPNET
#ifdef WIN32
        HANDLE GetInputEventHandle() {return input_event_handle;}
        HANDLE GetOutputEventHandle() {return output_event_handle;}
        bool IsOutputReady() {return output_ready;}
        bool IsInputReady() {return input_ready;}
        bool IsReady() {return (input_ready || output_ready);}
#endif  // WIN32
        static const char* GetErrorString()
        {
#ifdef WIN32
            static char errorString[256];
            errorString[255] = '\0';
            FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM |
                          FORMAT_MESSAGE_IGNORE_INSERTS,
                          NULL,
                          WSAGetLastError(),
                          MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
                          (LPTSTR) errorString, 255, NULL);
            return errorString;
#else
            return strerror(errno);
#endif // if/else WIN32/UNIX
        }

        // Read/Write methods
        bool SendTo(const char* buffer, unsigned int buflen, const ProtoAddress& dstAddr);
        bool RecvFrom(char* buffer, unsigned int& numBytes, ProtoAddress& srcAddr);
        bool Send(const char* buffer, unsigned int& numBytes);
        bool Recv(char* buffer, unsigned int& numBytes);

        // Attributes
        bool SetTTL(unsigned char ttl);
        bool SetLoopback(bool loopback);
        bool SetBroadcast(bool broadcast);
        bool SetReuse(bool reuse);
        bool SetMulticastInterface(const char* interfaceName);
        bool SetTOS(unsigned char tos);
        bool SetTxBufferSize(unsigned int bufferSize);
        unsigned int GetTxBufferSize();
        bool SetRxBufferSize(unsigned int bufferSize);
        unsigned int GetRxBufferSize();
        bool SetFlowLabel(UINT32 label); //ScenSim-Port://

        // Helper methods
#ifdef HAVE_IPV6
        static bool HostIsIPv6Capable();
        static bool SetHostIPv6Capable();
        bool SetFlowLabel(UINT32 label);
#endif //HAVE_IPV6

        static bool GetInterfaceAddressList(const char*         ifName,
                                            ProtoAddress::Type  addrType,
                                            ProtoAddressList&   addrList,
                                            unsigned int*       ifIndex = NULL);
        static bool GetInterfaceAddress(const char*         ifName,
                                        ProtoAddress::Type  addrType,
                                        ProtoAddress&       theAddress,
                                        unsigned int*       ifIndex = NULL)
        {
            ProtoAddressList addrList;
            GetInterfaceAddressList(ifName, addrType, addrList, ifIndex);
            return addrList.GetFirstAddress(addrType, theAddress);
        }
        static int GetInterfaceIndices(int* indexArray, unsigned int indexArraySize);
        static unsigned int GetInterfaceIndex(const char* interfaceName);
        static bool FindLocalAddress(ProtoAddress::Type addrType, ProtoAddress& theAddress);
        static bool GetInterfaceName(unsigned int index, char* buffer, unsigned int buflen);
        static bool GetInterfaceName(const ProtoAddress& ifAddr, char* buffer, unsigned int buflen);

        // Asynchronous I/O notification stuff
        enum Flag
        {
            NOTIFY_NONE      =   0x00,
            NOTIFY_INPUT     =   0x01,
            NOTIFY_OUTPUT    =   0x02,
            NOTIFY_EXCEPTION =  0x04,
            NOTIFY_ERROR     =   0x08
        };
        class Notifier
        {
            public:
                virtual ~Notifier() {}
                virtual bool UpdateSocketNotification(ProtoSocket& theSocket,
                                                      int          notifyFlags) {return true;}
        };
        Notifier* GetNotifier() const {return notifier;}
        bool SetNotifier(ProtoSocket::Notifier* theNotifier);
        bool StartOutputNotification()
        {
            notify_output = true;
            notify_output = UpdateNotification();
#ifdef WIN32
            output_ready = true;
#endif // WIN32
            return notify_output;
        }
        void StopOutputNotification()
        {
            notify_output = false;
            UpdateNotification();
        }
        bool NotifyOutput() {return notify_output;}

        bool StartInputNotification()
        {
            notify_input = true;
            notify_input = UpdateNotification();
#ifdef WIN32
            input_ready = true;
#endif // WIN32
            return notify_input;
        }
        void StopInputNotification()
        {
            notify_input = false;
            UpdateNotification();
        }
        bool NotifyInput() {return notify_input;}

        bool StartExceptionNotification()
        {
            notify_exception = true;
            notify_exception = UpdateNotification();
            return notify_exception;
        }
        void StopExceptionNotification()
        {
            notify_exception = false;
            UpdateNotification();
        }

        void OnNotify(ProtoSocket::Flag theFlag);

        enum Event {INVALID_EVENT, CONNECT, ACCEPT, SEND, RECV, DISCONNECT, ERROR_, EXCEPTION};

        // NOTE: For VC++ 6.0 Debug builds, you _cannot_ use pre-compiled
        // headers with this template code.  Also, "/ZI" or "/Z7" compile options
        // must NOT be specified.  (or else VC++ 6.0 experiences an "internal compiler error")
        // (Later Visual Studio versions have fixed this error)
        template <class listenerType>
        bool SetListener(listenerType* theListener, void(listenerType::*eventHandler)(ProtoSocket&, Event))
        {
            bool doUpdate = ((NULL == listener) && (NULL != theListener)) ||
                            ((NULL == theListener) && (NULL != listener));
            if (listener) delete listener;
            listener = theListener ? new LISTENER_TYPE<listenerType>(theListener, eventHandler) : NULL;
            bool result = theListener ? (NULL != theListener) : true;
            return result ? (doUpdate ? UpdateNotification() : true) : false;
        }

        bool HasListener()
        {
            return (NULL != listener);
        }
        void SetUserData(const void* userData) {user_data = userData;}
        const void* GetUserData() {return user_data;}

        // Here's a helper linked list class
        class List
        {
            public:
                List();
                ~List();
                void Destroy();  // deletes list Items _and_ their socket

                bool AddSocket(ProtoSocket& theSocket);
                void RemoveSocket(ProtoSocket& theSocket);

                class Item;
                Item* FindItem(const ProtoSocket& theSocket) const;

            public:
                class Iterator
                {
                    public:
                        Iterator(const List& theList);
                        const Item* GetNextItem()
                        {
                            const Item* current = next;
                            next = current ? current->GetNext() : current;
                            return current;
                        }
                        ProtoSocket* GetNextSocket()
                        {
                            const Item* nextItem = GetNextItem();
                            return nextItem ? nextItem->GetSocket() : NULL;
                        }


                    private:
                        const List&         list;
                        const class Item*   next;

                };  // end class ProtoSocketList::Iterator
                friend class List::Iterator;

                class Item
                {
                    friend class List;
                    friend class Iterator;
                    public:
                        Item(ProtoSocket* theSocket);
                        ProtoSocket* GetSocket() const {return socket;}
                        const void* GetUserData() {return user_data;}
                        void SetUserData(const void* userData)
                            {user_data = userData;}
                    private:
                        Item* GetPrev() const {return prev;}
                        void SetPrev(Item* item) {prev = item;}
                        Item* GetNext() const {return next;}
                        void SetNext(Item* item) {next = item;}

                        ProtoSocket*    socket;
                        const void*     user_data;
                        Item*           prev;
                        Item*           next;
                };   // end class ProtoSocketList::Item

                Item*   head;
        };  // end class ProtoSocketList


    protected:
        class Listener
        {
            public:
                virtual ~Listener() {}
                virtual void on_event(ProtoSocket& theSocket, Event theEvent) = 0;
                virtual Listener* duplicate() = 0;
        };

        template <class listenerType>
        class LISTENER_TYPE : public Listener
        {
            public:
                LISTENER_TYPE(listenerType* theListener,
                              void(listenerType::*eventHandler)(ProtoSocket&, Event))
                    : listener(theListener), event_handler(eventHandler) {}
                void on_event(ProtoSocket& theSocket, Event theEvent)
                    {(listener->*event_handler)(theSocket, theEvent);}
                Listener* duplicate()
                    {return (static_cast<Listener*>(new LISTENER_TYPE<listenerType>(listener, event_handler)));}
            private:
                listenerType* listener;
                void(listenerType::*event_handler)(ProtoSocket&, Event);
        };
        virtual bool SetBlocking(bool blocking);
        bool UpdateNotification();

        Domain                  domain;
        Protocol                protocol;
        State                   state;
        Handle                  handle;
        int                     port;
#ifdef HAVE_IPV6
        UINT32                  flow_label;    //IPv6 flow label
#endif // HAVE_IPV6
        ProtoAddress            destination;

        Notifier*               notifier;
        bool                    notify_output;
        bool                    notify_input;
        bool                    notify_exception;
#ifdef WIN32
        HANDLE                  input_event_handle;
        HANDLE                  output_event_handle;
        bool                    output_ready;
        bool                    input_ready;
#endif // WIN32
        Listener*               listener;

        const void*             user_data;
};  // end class ProtoSocket



} //namespace// //ScenSim-Port://


#endif // _PROTO_SOCKET

