#pragma once

#include "LoftCorrespondenceAdapter.hxx"
#include "LoftCorrespondenceProof.hxx"
#include "RectangularLoftPersistence.hxx"
#include "RetainedRecipeIdentity.hxx"
#include "RetainedSolidAttribute.hxx"

#include <BinMDF_ADriver.hxx>
#include <BinMDF_ADriverTable.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <CommonCrypto/CommonDigest.h>
#include <TDF_AttributeIterator.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelMap.hxx>
#include <TNaming_NamedShape.hxx>
#include <TopoDS.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <set>
#include <vector>

namespace core3d::loft_correspondence::persistence {

using UUID = retained_recipe::UUID;
inline constexpr std::size_t MaximumPayloadBytes = 16 * 1024;
inline constexpr std::size_t MaximumAggregateBytes = 8 * 1024 * 1024;
inline constexpr std::size_t MaximumRecords = 128;
inline constexpr int MinimumRecordTag = 17;
inline constexpr std::uint16_t Schema = 1;

inline const Standard_GUID& AttributeID() {
    static const Standard_GUID id("F49E75A9-CC8F-4AEC-9789-D66A150F4922"); return id;
}

struct Value {
    retained_recipe::OwnerKey owner;
    UUID feature{};
    std::uint64_t revision = 0;
    Definition definition;
};

namespace detail {
inline void Put16(std::vector<std::uint8_t>& bytes, std::uint16_t value) {
    bytes.push_back(std::uint8_t(value)); bytes.push_back(std::uint8_t(value >> 8));
}
inline void Put32(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
    for (unsigned shift = 0; shift < 32; shift += 8) bytes.push_back(std::uint8_t(value >> shift));
}
inline void Put64(std::vector<std::uint8_t>& bytes, std::uint64_t value) {
    for (unsigned shift = 0; shift < 64; shift += 8) bytes.push_back(std::uint8_t(value >> shift));
}
inline bool Take16(const std::vector<std::uint8_t>& bytes, std::size_t& offset,
                   std::uint16_t& value) {
    if (offset > bytes.size() || bytes.size() - offset < 2) return false;
    value = std::uint16_t(bytes[offset]) | (std::uint16_t(bytes[offset + 1]) << 8);
    offset += 2; return true;
}
inline bool Take32(const std::vector<std::uint8_t>& bytes, std::size_t& offset,
                   std::uint32_t& value) {
    if (offset > bytes.size() || bytes.size() - offset < 4) return false;
    value = 0; for (unsigned shift = 0; shift < 32; shift += 8)
        value |= std::uint32_t(bytes[offset++]) << shift;
    return true;
}
inline bool Take64(const std::vector<std::uint8_t>& bytes, std::size_t& offset,
                   std::uint64_t& value) {
    if (offset > bytes.size() || bytes.size() - offset < 8) return false;
    value = 0; for (unsigned shift = 0; shift < 64; shift += 8)
        value |= std::uint64_t(bytes[offset++]) << shift;
    return true;
}
inline void PutUUID(std::vector<std::uint8_t>& bytes, const UUID& value) {
    bytes.insert(bytes.end(), value.begin(), value.end());
}
inline bool TakeUUID(const std::vector<std::uint8_t>& bytes, std::size_t& offset,
                     UUID& value) {
    if (offset > bytes.size() || bytes.size() - offset < value.size()) return false;
    std::copy_n(bytes.begin() + offset, value.size(), value.begin());
    offset += value.size(); return true;
}
inline bool Digest(const std::uint8_t* bytes, std::size_t size,
                   std::array<std::uint8_t, CC_SHA256_DIGEST_LENGTH>& output) {
    return size <= std::size_t(std::numeric_limits<CC_LONG>::max())
        && CC_SHA256(bytes, CC_LONG(size), output.data()) != nullptr;
}
inline bool SameDefinition(const Definition& left, const Definition& right) {
    std::vector<double> a, b;
    if (!loft_persistence::Encode(left.base, a) || !loft_persistence::Encode(right.base, b)
        || !loft_persistence::SameBits(a, b) || left.implicitEqualIndex != right.implicitEqualIndex
        || left.mappings.size() != right.mappings.size()) return false;
    for (std::size_t i = 0; i < left.mappings.size(); ++i)
        if (left.mappings[i].station != right.mappings[i].station
            || left.mappings[i].targetsByLane != right.mappings[i].targetsByLane) return false;
    return true;
}
inline bool Valid(const Value& value) {
    MappingInspection mapping;
    return retained_recipe::Valid(value.owner) && retained_recipe::Nonzero(value.feature)
        && value.revision != 0 && !value.definition.implicitEqualIndex
        && loft_correspondence::detail::ValidateMapping(value.definition, mapping)
            == Admission::Accepted;
}
} // namespace detail

inline bool Encode(const Value& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!detail::Valid(value)) return false;
        std::vector<double> scalars;
        if (!loft_persistence::Encode(value.definition.base, scalars)
            || scalars.size() > std::size_t(std::numeric_limits<std::uint32_t>::max())
            || value.definition.mappings.size() > 8) return false;
        std::vector<std::uint8_t> body;
        body.reserve(80 + scalars.size() * 8 + value.definition.mappings.size() * 20);
        detail::PutUUID(body, value.owner.document); detail::PutUUID(body, value.owner.entity);
        detail::PutUUID(body, value.owner.definition); detail::PutUUID(body, value.feature);
        detail::Put64(body, value.revision); detail::Put32(body, std::uint32_t(scalars.size()));
        for (double scalar : scalars) { std::uint64_t bits = 0; std::memcpy(&bits, &scalar, 8); detail::Put64(body, bits); }
        detail::Put32(body, std::uint32_t(value.definition.mappings.size()));
        for (const auto& row : value.definition.mappings) {
            detail::Put32(body, row.station);
            for (ElementID target : row.targetsByLane) detail::Put32(body, target);
        }
        if (body.size() > std::size_t(std::numeric_limits<std::uint32_t>::max())) return false;
        output = {'S', 'Y', 'L', 'C'}; detail::Put16(output, Schema); detail::Put16(output, 0);
        detail::Put32(output, std::uint32_t(body.size())); output.insert(output.end(), body.begin(), body.end());
        std::array<std::uint8_t, CC_SHA256_DIGEST_LENGTH> digest{};
        if (!detail::Digest(body.data(), body.size(), digest)) { output.clear(); return false; }
        output.insert(output.end(), digest.begin(), digest.end());
        if (output.size() > MaximumPayloadBytes) { output.clear(); return false; }
        return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Value& output) noexcept {
    output = {};
    try {
        constexpr std::size_t prefix = 12, digestSize = CC_SHA256_DIGEST_LENGTH;
        if (bytes.size() < prefix + digestSize || bytes.size() > MaximumPayloadBytes
            || bytes[0] != 'S' || bytes[1] != 'Y' || bytes[2] != 'L' || bytes[3] != 'C') return false;
        std::size_t offset = 4; std::uint16_t schema = 0, flags = 0; std::uint32_t bodySize = 0;
        if (!detail::Take16(bytes, offset, schema) || !detail::Take16(bytes, offset, flags)
            || !detail::Take32(bytes, offset, bodySize) || schema != Schema || flags != 0
            || bodySize != bytes.size() - prefix - digestSize) return false;
        std::array<std::uint8_t, digestSize> actual{};
        if (!detail::Digest(bytes.data() + prefix, bodySize, actual)
            || !std::equal(actual.begin(), actual.end(), bytes.end() - digestSize)) return false;
        const std::size_t bodyEnd = prefix + bodySize; Value value;
        if (!detail::TakeUUID(bytes, offset, value.owner.document)
            || !detail::TakeUUID(bytes, offset, value.owner.entity)
            || !detail::TakeUUID(bytes, offset, value.owner.definition)
            || !detail::TakeUUID(bytes, offset, value.feature)
            || !detail::Take64(bytes, offset, value.revision)) return false;
        std::uint32_t scalarCount = 0;
        if (!detail::Take32(bytes, offset, scalarCount)
            || scalarCount < std::uint32_t(loft_persistence::MinimumScalars)
            || scalarCount > std::uint32_t(loft_persistence::MaximumScalars)
            || std::size_t(scalarCount) > (bodyEnd - offset) / 8) return false;
        std::vector<double> scalars(scalarCount);
        for (double& scalar : scalars) { std::uint64_t bits = 0; if (!detail::Take64(bytes, offset, bits)) return false; std::memcpy(&scalar, &bits, 8); }
        if (!loft_persistence::Decode(scalars, value.definition.base)) return false;
        std::uint32_t count = 0;
        if (!detail::Take32(bytes, offset, count) || count != value.definition.base.stations.size()
            || count > 8 || std::size_t(count) > (bodyEnd - offset) / 20) return false;
        value.definition.mappings.resize(count);
        for (auto& row : value.definition.mappings) {
            std::uint32_t station = 0; if (!detail::Take32(bytes, offset, station)) return false;
            row.station = station;
            for (auto& target : row.targetsByLane) { std::uint32_t id = 0; if (!detail::Take32(bytes, offset, id)) return false; target = id; }
        }
        if (offset != bodyEnd || !detail::Valid(value)) return false;
        std::vector<std::uint8_t> canonical;
        if (!Encode(value, canonical) || canonical != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

struct Payload { Value value; std::vector<std::uint8_t> bytes; };
class BinaryDriver;
class Attribute final : public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTI_INLINE(Attribute, TDF_Attribute)
    const Standard_GUID& ID() const override { return AttributeID(); }
    Handle(TDF_Attribute) NewEmpty() const override { return new Attribute(); }
    void Restore(const Handle(TDF_Attribute)& source) override {
        const auto typed = Handle(Attribute)::DownCast(source);
        if (typed.IsNull()) Standard_Failure::Raise("SYLC restore type"); payload_ = typed->payload_;
    }
    void Paste(const Handle(TDF_Attribute)& target, const Handle(TDF_RelocationTable)&) const override {
        const auto typed = Handle(Attribute)::DownCast(target);
        if (typed.IsNull() || !payload_) Standard_Failure::Raise("SYLC paste type");
        typed->Backup(); typed->payload_ = payload_;
    }
    const std::shared_ptr<const Payload>& payload() const noexcept { return payload_; }
private:
    friend class BinaryDriver;
#if DEBUG
    friend bool DebugInstall(const TDF_Label&, const Value&, const TopoDS_Shape&);
#endif
    std::shared_ptr<const Payload> payload_;
};

struct Record { TDF_Label label, owner; std::shared_ptr<const Payload> payload; TopoDS_Shape shape; };
inline bool HasRecord(const TDF_Label& owner) noexcept {
    try {
        if (owner.IsNull()) return false;
        if (owner.IsAttribute(AttributeID())) return true;
        int visited = 0;
        for (TDF_ChildIterator it(owner, Standard_False); it.More(); it.Next())
            if (++visited > loft_persistence::MaximumLabels || it.Value().IsAttribute(AttributeID())) return true;
        return false;
    } catch (...) { return true; }
}

inline bool ReadAll(const Handle(TDocStd_Document)& document, std::vector<Record>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(document->Main())) return false;
        const TDF_Label root = document->GetData()->Root();
        if (root.IsAttribute(AttributeID())) return false;
        std::vector<Record> staged; std::set<UUID> features; TDF_LabelMap owners, labels;
        std::size_t aggregate = 0; int visited = 0;
        for (TDF_ChildIterator it(root, Standard_True); it.More(); it.Next()) {
            if (++visited > loft_persistence::MaximumLabels) return false;
            const TDF_Label label = it.Value(); if (!label.IsAttribute(AttributeID())) continue;
            if (staged.size() >= MaximumRecords || label.Tag() < MinimumRecordTag || !labels.Add(label)) return false;
            const TDF_Label owner = label.Father(); Handle(Attribute) attribute; Handle(TNaming_NamedShape) binding;
            if (owner.IsNull() || !owners.Add(owner) || !label.FindAttribute(AttributeID(), attribute)
                || attribute.IsNull() || !attribute->payload() || !label.FindAttribute(TNaming_NamedShape::GetID(), binding)
                || binding.IsNull() || loft_persistence::HasAttribute(owner)) return false;
            const auto payload = attribute->payload(); Value decoded;
            if (payload->bytes.size() > MaximumAggregateBytes - aggregate
                || !Decode(payload->bytes, decoded) || !detail::SameDefinition(decoded.definition, payload->value.definition)
                || decoded.owner != payload->value.owner || decoded.feature != payload->value.feature
                || decoded.revision != payload->value.revision || !features.insert(decoded.feature).second) return false;
            aggregate += payload->bytes.size();
            UUID documentID{}, entityID{}, definitionID{};
            if (!retained_solid::ReadUUID(document->Main(), Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
                || !retained_solid::ReadUUID(owner, Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entityID)
                || !retained_solid::ReadUUID(owner, Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definitionID)
                || decoded.owner.document != documentID || decoded.owner.entity != entityID
                || decoded.owner.definition != definitionID || !XCAFDoc_ShapeTool::IsFree(owner)
                || !XCAFDoc_ShapeTool::IsSimpleShape(owner)) return false;
            const TopoDS_Shape shape = binding->Get(); if (shape.IsNull() || shape.ShapeType() != TopAbs_SOLID) return false;
            for (TDF_AttributeIterator attrs(label); attrs.More(); attrs.Next())
                if (attrs.Value()->ID() != AttributeID() && attrs.Value()->ID() != TNaming_NamedShape::GetID()) return false;
            for (TDF_ChildIterator child(label, Standard_True); child.More(); child.Next()) if (child.Value().HasAttribute()) return false;
            std::atomic_bool cancelled{false}; retained_topology_budget::Counter counter;
            loft_correspondence::proof::Inspection inspected;
            if (loft_correspondence::proof::Inspect(decoded.definition, cancelled, counter, inspected)
                    != Admission::Accepted) return false;
            const auto proven = loft_correspondence::proof::Verify(
                decoded.definition, inspected, TopoDS::Solid(shape), cancelled, counter);
            if (proven.status != loft_correspondence::proof::ProofStatus::Proven) return false;
            staged.push_back({label, owner, payload, shape});
        }
        output = std::move(staged); return true;
    } catch (...) { output.clear(); return false; }
}

struct ReadBudget {
    std::size_t bytes = 0, records = 0; bool rejected = false;
    void reset() noexcept { bytes = records = 0; rejected = false; }
};

class BinaryDriver final : public BinMDF_ADriver {
public:
    BinaryDriver(const Handle(Message_Messenger)& messenger, std::shared_ptr<ReadBudget> budget,
                 void (*reject)() noexcept = nullptr)
        : BinMDF_ADriver(messenger, STANDARD_TYPE(Attribute)->Name()), budget_(std::move(budget)), reject_(reject) {}
    Handle(TDF_Attribute) NewEmpty() const override { return new Attribute(); }
    const Handle(Standard_Type)& SourceType() const override { return STANDARD_TYPE(Attribute); }
    Standard_Boolean Paste(const BinObjMgt_Persistent& source, const Handle(TDF_Attribute)& target,
                           BinObjMgt_RRelocationTable&) const override {
        try {
            const auto attribute = Handle(Attribute)::DownCast(target); Standard_Integer count = 0;
            if (attribute.IsNull() || attribute->payload_ || !budget_ || budget_->rejected
                || !(source >> count) || count <= 0 || count > Standard_Integer(MaximumPayloadBytes)
                || budget_->records >= MaximumRecords || std::size_t(count) > MaximumAggregateBytes - budget_->bytes) return Refuse();
            std::vector<std::uint8_t> bytes(std::size_t(count), 0);
            if (!source.GetByteArray(bytes.data(), count)) return Refuse();
            Value value; if (!Decode(bytes, value)) return Refuse();
            auto payload = std::make_shared<Payload>(); payload->value = std::move(value); payload->bytes = std::move(bytes);
            attribute->payload_ = std::move(payload); budget_->bytes += std::size_t(count); ++budget_->records;
            return Standard_True;
        } catch (...) { return Refuse(); }
    }
    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&) const override {
        const auto attribute = Handle(Attribute)::DownCast(source); Value decoded;
        std::vector<std::uint8_t> canonical;
        if (attribute.IsNull() || !attribute->payload_ || !Decode(attribute->payload_->bytes, decoded)
            || !Encode(decoded, canonical) || canonical != attribute->payload_->bytes)
            Standard_Failure::Raise("SYLC writer source");
        target << Standard_Integer(canonical.size());
        target.PutByteArray(canonical.data(), Standard_Integer(canonical.size()));
    }
private:
    Standard_Boolean Refuse() const noexcept { if (budget_) budget_->rejected = true; if (reject_) reject_(); return Standard_False; }
    std::shared_ptr<ReadBudget> budget_; void (*reject_)() noexcept;
};

inline void Register(const Handle(BinMDF_ADriverTable)& table, const Handle(Message_Messenger)& messenger,
                     const std::shared_ptr<ReadBudget>& budget, void (*reject)() noexcept = nullptr) {
    table->AddDriver(new BinaryDriver(messenger, budget, reject));
}

template<class Base> class StorageDriver : public Base {
public:
    Handle(BinMDF_ADriverTable) AttributeDrivers(const Handle(Message_Messenger)& messenger) override {
        auto table = Base::AttributeDrivers(messenger); Register(table, messenger, std::make_shared<ReadBudget>()); return table;
    }
    void Write(const Handle(CDM_Document)& document, const TCollection_ExtendedString& file,
               const Message_ProgressRange& progress = Message_ProgressRange()) override {
        Prepare(document); Base::Write(document, file, progress);
    }
    void Write(const Handle(CDM_Document)& document, Standard_OStream& stream,
               const Message_ProgressRange& progress = Message_ProgressRange()) override {
        Prepare(document); Base::Write(document, stream, progress);
    }
private:
    static void Prepare(const Handle(CDM_Document)& value) {
        std::vector<Record> records;
        if (!ReadAll(Handle(TDocStd_Document)::DownCast(value), records)) Standard_Failure::Raise("SYLC writer validation");
    }
};

#if DEBUG
inline bool DebugInstall(const TDF_Label& label, const Value& value, const TopoDS_Shape& shape) {
    try {
        if (label.IsNull() || shape.IsNull()) return false; std::vector<std::uint8_t> bytes;
        if (!Encode(value, bytes)) return false; Handle(Attribute) attribute = new Attribute();
        auto payload = std::make_shared<Payload>(); payload->value = value; payload->bytes = std::move(bytes);
        attribute->payload_ = std::move(payload); label.AddAttribute(attribute); TNaming_Builder(label).Select(shape, shape); return true;
    } catch (...) { return false; }
}
#endif

} // namespace core3d::loft_correspondence::persistence
