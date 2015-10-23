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

 #ifndef _PROTO_ROUTE_TABLE
#define _PROTO_ROUTE_TABLE

#include "protoAddress.h"
#include "protoTree.h"


namespace NrlProtolibPort //ScenSim-Port://
{ //ScenSim-Port://

/*!
Class based on the ProtoTree Patricia tree to
store routing table information. Uses the
ProtoAddress class to store network routing
addresses.  It's a pretty dumbed-down routing
table at the moment, but may be enhanced in
the future.  Example use of the ProtoTree.
 
Notes:
 
1) Only one entry per destination/prefixLen is currently allowed.
   (Only one route per unique destination is maintained)
   (We may support multiple routes per dest in the future)
 
2) (ifIndex == 0) and (metric < 0) are "wildcards"
*/

class ProtoRouteTable
{
    public:
        ProtoRouteTable();
        ~ProtoRouteTable();
        
        void Init() {Destroy();}
        void Destroy();   
        
        bool IsEmpty() {return (tree.IsEmpty() && !default_entry.IsValid());}
        
        // For "GetRoute()" to work there _must_ be
        // an exact match (dst and prefixLen) route entry in the table.
        bool GetRoute(const ProtoAddress&   dst,        // input
                      unsigned int          prefixLen,  // input
                      ProtoAddress&         gw,         // output
                      unsigned int&         ifIndex,    // output
                      int&                  metric);    // output
        
        bool SetRoute(const ProtoAddress&   dst,
                      unsigned int          maskLen,
                      const ProtoAddress&   gw,
                      unsigned int          ifIndex = 0,
                      int                   metric = -1);
        
        // Note gateway is optional here
        bool DeleteRoute(const ProtoAddress&    dst,
                         unsigned int           maskLen,
                         const ProtoAddress*    gw = NULL,
                         unsigned int           ifIndex = 0);
        
        // Find the "best" route to the given destination
        // (Finds route entry with longest matching prefix (or default))
        bool FindRoute(const ProtoAddress&  dstAddr,
                       unsigned int         prefixLen,
                       ProtoAddress&        gwAddr,
                       unsigned int&        ifIndex,
                       int&                 metric);
                                     
        class Entry : public ProtoTree::Item
        {
            friend class ProtoRouteTable;
            
            public:
                bool IsValid() const {return destination.IsValid();}
                bool IsGatewayRoute() const {return gateway.IsValid();}
                bool IsDirectRoute() const
                {
                    return (!gateway.IsValid() && iface_index > 0);
                }
                void SetGateway(const ProtoAddress& gwAddr) {gateway = gwAddr;}
                void ClearGateway() {gateway.Invalidate();}
                void SetInterface(unsigned int ifaceIndex) {iface_index = ifaceIndex;}
                void ClearInterface() {iface_index = 0;}
                void SetMetric(int value) {metric = value;}
                
                const ProtoAddress& GetDestination() const {return destination;}
                unsigned int GetPrefixSize() const {return prefix_size;}  // in bits
                const ProtoAddress& GetGateway() const {return gateway;}
                unsigned int GetInterfaceIndex() const {return iface_index;}
                int GetMetric() const {return metric;}

                void Clear() 
                {
                    //ProtoTree::Item::Init(NULL, 0);
                    destination.Invalidate();
                    prefix_size = 0;
                    gateway.Invalidate();
                    iface_index = 0;
                    metric = -1;
                }
                
                // Required ProtoTree::Item overrides
                const char* GetKey() const {return destination.GetRawHostAddress();}
                unsigned int GetKeysize() const {return prefix_size;}
                
            private:
                Entry();
                Entry(const ProtoAddress& dstAddr, unsigned int prefixSize);
                ~Entry();
                
                void Init(const ProtoAddress& dstAddr, unsigned int prefixSize);
                
                ProtoAddress        destination;
                unsigned            prefix_size;  // in bits
                ProtoAddress        gateway;
                unsigned int        iface_index;
                int                 metric;
        };  // end class ProtoRouteTable::Entry
        
        class Iterator
        {
            public:
                Iterator(const ProtoRouteTable& table);
                void Reset() 
                {
                    iterator.Reset();
                    default_pending = true;
                }
                Entry* GetNextEntry();
                   
            private:
                const ProtoRouteTable&  table;
                ProtoTree::Iterator     iterator;
                bool                    default_pending;
        };  // end class ProtoRouteTable::Iterator
        friend class ProtoRouteTable::Iterator;
                
        ProtoRouteTable::Entry* CreateEntry(const ProtoAddress& dstAddr, 
                                            unsigned int          prefixLen);
        
        // Get entry exactly matching dstAddr/prefixLen
        ProtoRouteTable::Entry* GetEntry(const ProtoAddress& dstAddr, 
                                         unsigned int          prefixLen) const;
        
        // Find best matching route entry
        ProtoRouteTable::Entry* FindRouteEntry(const ProtoAddress& dstAddr, 
                                               unsigned int          prefixLen) const;
        
        ProtoRouteTable::Entry* GetDefaultEntry() const
            {return (default_entry.IsValid() ? (Entry*)&default_entry : NULL);}
        
        void DeleteEntry(ProtoRouteTable::Entry* entry);
                      
    private:
        ProtoTree  tree;
        Entry      default_entry;
};  // end class ProtoRouteTable

} //namespace// //ScenSim-Port://


#endif // _PROTO_ROUTE_TABLE
