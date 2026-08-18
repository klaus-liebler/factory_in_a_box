#include "opcua/address_space.hpp"

namespace opcua {

Node *AddressSpace::AddNodeCommon(const NodeId &id, NodeClass nodeClass,
                                  const QualifiedName &browseName, const LocalizedText &displayName) {
    if(FindNode(id) != nullptr) return nullptr; // duplicate NodeId -- programming error, reject
    if(nodeCount_ >= MAX_NODES) return nullptr;

    Node &n = nodes_[nodeCount_++];
    n = Node{};
    n.nodeId = id;
    n.nodeClass = nodeClass;
    n.browseName = browseName;
    n.displayName = displayName;
    return &n;
}

bool AddressSpace::AddObjectNode(const NodeId &id, const QualifiedName &browseName,
                                 const LocalizedText &displayName) {
    return AddNodeCommon(id, NodeClass::Object, browseName, displayName) != nullptr;
}

bool AddressSpace::AddVariableNode(const NodeId &id, const QualifiedName &browseName,
                                   const LocalizedText &displayName, const NodeId &dataType,
                                   Byte accessLevel, Variant initialValue) {
    Node *n = AddNodeCommon(id, NodeClass::Variable, browseName, displayName);
    if(!n) return false;
    n->dataType = dataType;
    n->accessLevel = accessLevel;
    n->staticValue = std::move(initialValue);
    return true;
}

bool AddressSpace::AddVariableNodeWithCallback(const NodeId &id, const QualifiedName &browseName,
                                               const LocalizedText &displayName, const NodeId &dataType,
                                               Byte accessLevel, ReadCallback read, WriteCallback write,
                                               void *context) {
    Node *n = AddNodeCommon(id, NodeClass::Variable, browseName, displayName);
    if(!n) return false;
    n->dataType = dataType;
    n->accessLevel = accessLevel;
    n->readCallback = read;
    n->writeCallback = write;
    n->userContext = context;
    return true;
}

bool AddressSpace::AddReference(const NodeId &sourceId, const NodeId &referenceTypeId,
                                const NodeId &targetId, bool addInverse) {
    if(referenceCount_ >= MAX_REFERENCES) return false;
    references_[referenceCount_++] = ReferenceEntry{sourceId, referenceTypeId, true, targetId};

    if(addInverse) {
        if(referenceCount_ >= MAX_REFERENCES) return false;
        references_[referenceCount_++] = ReferenceEntry{targetId, referenceTypeId, false, sourceId};
    }
    return true;
}

const Node *AddressSpace::FindNode(const NodeId &id) const {
    for(size_t i = 0; i < nodeCount_; i++) {
        if(nodes_[i].nodeId == id) return &nodes_[i];
    }
    return nullptr;
}

Node *AddressSpace::FindNodeMutable(const NodeId &id) {
    for(size_t i = 0; i < nodeCount_; i++) {
        if(nodes_[i].nodeId == id) return &nodes_[i];
    }
    return nullptr;
}

} // namespace opcua
