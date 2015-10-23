// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_PARMIO_H
#define SCENSIM_PARMIO_H

#include <string>
#include <cctype>
#include <map>
#include <set>
#include <memory>

#include "scensim_support.h"
#include "scensim_time.h"
#include "scensim_nodeid.h"
#include <memory>

namespace ScenSim {

using std::string;
using std::multimap;
using std::set;
using std::auto_ptr;
using std::shared_ptr;


class GuiInterfacingSubsystem;

//=============================================================================

class ParameterDatabaseReader {
public:

    explicit
    ParameterDatabaseReader(const string& parameterFileName);

    ~ParameterDatabaseReader();

    void DisableUnusedParameterWarning();

    bool ParameterExists(const string& parameterName) const;
    bool ParameterExists(const string& parameterName, const InterfaceOrInstanceIdType& instanceId) const;
    bool ParameterExists(const string& parameterName, const NodeIdType& nodeId) const;
    bool ParameterExists(
        const string& parameterName,
        const NodeIdType& nodeId,
        const InterfaceOrInstanceIdType& interfaceId) const;

    bool ReadBool(const string& parameterName) const;
    bool ReadBool(const string& parameterName, const InterfaceOrInstanceIdType& instanceId) const;
    bool ReadBool(const string& parameterName, const NodeIdType& nodeId) const;
    bool ReadBool(
        const string& parameterName,
        const NodeIdType& nodeId,
        const InterfaceOrInstanceIdType& interfaceId) const;

    int ReadInt(const string& parameterName) const;
    int ReadInt(const string& parameterName, const InterfaceOrInstanceIdType& instanceId) const;
    int ReadInt(const string& parameterName, const NodeIdType& nodeId) const;
    int ReadInt(
        const string& parameterName,
        const NodeIdType& nodeId,
        const InterfaceOrInstanceIdType& interfaceId) const;

    long long int ReadBigInt(const string& parameterName) const;
    long long int ReadBigInt(const string& parameterName, const InterfaceOrInstanceIdType& instanceId) const;
    long long int ReadBigInt(const string& parameterName, const NodeIdType& nodeId) const;
    long long int ReadBigInt(
        const string& parameterName,
        const NodeIdType& nodeId,
        const InterfaceOrInstanceIdType& interfaceId) const;

    unsigned int ReadNonNegativeInt(const string& parameterName) const;
    unsigned int ReadNonNegativeInt(
            const string& parameterName,
            const InterfaceOrInstanceIdType& instanceId) const;
    unsigned int ReadNonNegativeInt(const string& parameterName, const NodeIdType& nodeId) const;
    unsigned int ReadNonNegativeInt(
        const string& parameterName,
        const NodeIdType& nodeId,
        const InterfaceOrInstanceIdType& interfaceId) const;

    unsigned long long int ReadNonNegativeBigInt(const string& parameterName) const;
    unsigned long long int ReadNonNegativeBigInt(
        const string& parameterName,
        const InterfaceOrInstanceIdType& instanceId) const;
    unsigned long long int ReadNonNegativeBigInt(const string& parameterName, const NodeIdType& nodeId) const;
    unsigned long long int ReadNonNegativeBigInt(
        const string& parameterName,
        const NodeIdType& nodeId,
        const InterfaceOrInstanceIdType& interfaceId) const;

    double ReadDouble(const string& parameterName) const;
    double ReadDouble(const string& parameterName, const InterfaceOrInstanceIdType& instanceId) const;
    double ReadDouble(const string& parameterName, const NodeIdType& nodeId) const;
    double ReadDouble(
        const string& parameterName,
        const NodeIdType& nodeId,
        const InterfaceOrInstanceIdType& interfaceId) const;

    TimeType ReadTime(const string& parameterName) const;
    TimeType ReadTime(const string& parameterName, const InterfaceOrInstanceIdType& instanceId) const;
    TimeType ReadTime(const string& parameterName, const NodeIdType& nodeId) const;
    TimeType ReadTime(
        const string& parameterName,
        const NodeIdType& nodeId,
        const InterfaceOrInstanceIdType& interfaceId) const;

    string ReadString(const string& parameterName) const;
    string ReadString(const string& parameterName, const InterfaceOrInstanceIdType& instanceId) const;
    string ReadString(const string& parameterName, const NodeIdType& nodeId) const;
    string ReadString(
        const string& parameterName,
        const NodeIdType& nodeId,
        const InterfaceOrInstanceIdType& interfaceId) const;

    string GetContainingNodeIdSetNameFor(const NodeIdType& nodeId) const;

    //Communication nodes and GIS nodes

    void MakeSetOfAllNodeIds(set<NodeIdType>& setOfNodeIds) const;

    //only communication nodes

    void MakeSetOfAllCommNodeIds(set<NodeIdType>& setOfNodeIds) const;
    bool CommNodeIdExists(const NodeIdType& nodeId) const;

    void MakeSetOfAllNodeIdsWithParameter(const string& parameterName, set<NodeIdType>& setOfNodeIds) const;
    void MakeSetOfAllNodeIdsWithParameter(
        const string& parameterName,
        const string& parameterValue,
        set<NodeIdType>& setOfNodeIds) const;


    void MakeSetOfAllInterfaceIdsForANode(
        const NodeIdType& nodeId,
        set<InterfaceOrInstanceIdType>& setOfInterfaces) const
    {
        MakeSetOfAllInterfaceIdsForANode(nodeId, "network-address", setOfInterfaces);
    }

    void MakeSetOfAllInterfaceIdsForANode(
        const NodeIdType& nodeId,
        const string& parameterName,
        set<InterfaceOrInstanceIdType>& setOfInterfaces) const;

    void MakeSetOfAllInterfaceIds(
        const string& parameterName,
        set<InterfaceOrInstanceIdType>& setOfInterfaces) const;

    void MakeSetOfAllInterfaceIds(
        const string& parameterName,
        const string& parameterMustBeEqual,
        set<InterfaceOrInstanceIdType>& setOfInterfaces) const;


    // Just an aliases for "MakeSetOfAllInterfaceIds*" methods.

    void MakeSetOfAllInstanceIdsForANode(
        const NodeIdType& nodeId,
        const string& parameterName,
        set<InterfaceOrInstanceIdType>& setOfInstances) const
    {
        MakeSetOfAllInterfaceIdsForANode(nodeId, parameterName, setOfInstances);
    }


    void MakeSetOfAllInstanceIds(
        const string& parameterName,
        set<InterfaceOrInstanceIdType>& setOfInstances) const
    {
        MakeSetOfAllInterfaceIds(parameterName, setOfInstances);
    }


    // Supports "Split Node" functionality.

    class NodeIdRemapper {
    public:
        virtual NodeIdType MapNodeIdTo(const NodeIdType& nodeId) const = 0;
    };

    void SetNodeIdRemapper(const shared_ptr<NodeIdRemapper>& nodeIdRemapperPtr);

    NodeIdType GetPossibleNodeIdRemap(const NodeIdType& nodeId) const;


    void AddNewDefinitionToDatabase(
        const string& definitionLineInFileFormat,
        bool& foundAnError);

private:

    // For now restricting in-simulation writing to just certain subsystems.
    friend class GuiInterfacingSubsystem;

    class Implementation;

    auto_ptr<Implementation> implementationPtr;

    static void CheckIntegerValueIsNonNegative(
        const long long int anInt,
        const string& parameterName);

    // Disable:
    ParameterDatabaseReader(ParameterDatabaseReader&);
    void operator=(ParameterDatabaseReader&);

};//ParameterDatabaseReader//


inline
void ParameterDatabaseReader::CheckIntegerValueIsNonNegative(
    const long long int anInt,
    const string& parameterName)
{
    if (anInt < 0) {
        cerr << "Error in Configuration File: Parameter \"" << parameterName
             << "\" is a negative value." << endl;
        exit(1);
    }//if//
}


inline
unsigned int ParameterDatabaseReader::ReadNonNegativeInt(const string& parameterName) const
{
    const int aValue = ReadInt(parameterName);
    CheckIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned int>(aValue));
}

inline
unsigned int ParameterDatabaseReader::ReadNonNegativeInt(
    const string& parameterName,
    const InterfaceOrInstanceIdType& instanceId) const
{
    const int aValue = ReadInt(parameterName, instanceId);
    CheckIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned int>(aValue));
}


inline
unsigned int ParameterDatabaseReader::ReadNonNegativeInt(
    const string& parameterName,
    const NodeIdType& nodeId) const
{
    const int aValue = ReadInt(parameterName, nodeId);
    CheckIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned int>(aValue));
}



inline
unsigned int ParameterDatabaseReader::ReadNonNegativeInt(
    const string& parameterName,
    const NodeIdType& nodeId,
    const InterfaceOrInstanceIdType& interfaceId) const
{
    const int aValue = ReadInt(parameterName, nodeId, interfaceId);
    CheckIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned int>(aValue));
}


inline
unsigned long long int ParameterDatabaseReader::ReadNonNegativeBigInt(const string& parameterName) const
{
    const long long int aValue = ReadBigInt(parameterName);
    CheckIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned long long int>(aValue));
}


inline
unsigned long long int ParameterDatabaseReader::ReadNonNegativeBigInt(
    const string& parameterName,
    const InterfaceOrInstanceIdType& instanceId) const
{
    const long long int aValue = ReadBigInt(parameterName, instanceId);
    CheckIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned long long int>(aValue));
}


inline
unsigned long long int ParameterDatabaseReader::ReadNonNegativeBigInt(
    const string& parameterName, const NodeIdType& nodeId) const
{
    const long long int aValue = ReadBigInt(parameterName, nodeId);
    CheckIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned long long int>(aValue));
}


inline
unsigned long long int ParameterDatabaseReader::ReadNonNegativeBigInt(
    const string& parameterName,
    const NodeIdType& nodeId,
    const InterfaceOrInstanceIdType& interfaceId) const
{
    const long long int aValue = ReadBigInt(parameterName, nodeId, interfaceId);
    CheckIntegerValueIsNonNegative(aValue, parameterName);
    return (static_cast<unsigned long long int>(aValue));
}


}//namespace//

#endif
