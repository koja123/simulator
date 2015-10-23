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

#ifndef _PROTO_CAP
#define _PROTO_CAP

/** ProtoCap : This class is a generic base class for simple
 *             link layer packet capture (ala libpcap) and
 *             raw link layer packet transmission.
 */

#include "protoChannel.h"

namespace NrlProtolibPort //ScenSim-Port://
{ //ScenSim-Port://
class ProtoCap : public ProtoChannel
{
    public:
        static ProtoCap* Create();
        virtual ~ProtoCap();
        
        // Packet capture "direction"
        enum Direction
        {
            UNSPECIFIED,
            INBOUND,
            OUTBOUND   
        };
        
        // These must be overridden for different implementations
        // ProtoCap::Open() should also be called at the _end_ of derived
        // implementations' Open() method
        virtual bool Open(const char* interfaceName = NULL)
            {return ProtoChannel::Open();}
        virtual bool IsOpen() {return ProtoChannel::IsOpen();}
        // ProtoCap::Close() should also be called at the _beginning_ of
        // derived implementations' Close() method
        virtual void Close() 
        {
            ProtoChannel::Close();
            if_index = -1;
        }
        
        int GetInterfaceIndex() const 
            {return if_index;}
        
        virtual bool Send(const char* buffer, unsigned int buflen) = 0;
        virtual bool Forward(char* buffer, unsigned int buflen) = 0;
        virtual bool Recv(char* buffer, unsigned int& numBytes, Direction* direction = NULL) = 0;
        
        void SetUserData(const void* userData) 
            {user_data = userData;}
        const void* GetUserData() const
            {return user_data;}
            
    protected:
        ProtoCap();
        int         if_index;
        
    private:
        const void* user_data;
            
};  // end class ProtoCap

} //namespace// //ScenSim-Port://
#endif // _PROTO_CAP
