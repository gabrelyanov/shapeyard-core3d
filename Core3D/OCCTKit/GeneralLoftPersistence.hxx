#pragma once

// Canonical C3-N carrier. SYGL/1 owns the complete ruled-loft definition;
// TNaming stores only the rebuilt result and never becomes recipe authority.
#include "GeneralLoftProof.hxx"
#include "RetainedSolidAttribute.hxx"
#include <BinMDF_ADriverTable.hxx>
#include <BinMNaming_NamedShapeDriver.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <TDF_Attribute.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDF_ChildIterator.hxx>
#include <TNaming_NamedShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <memory>

class OcctDocument;

namespace core3d::general_loft::persistence {
inline constexpr std::size_t MaximumPayloadBytes = 512 * 1024;
inline constexpr std::size_t MaximumDocumentBytes = 8 * 1024 * 1024;
inline constexpr std::size_t MaximumRecords = 128;
inline constexpr int RecordTag = 37;

struct ReadBudget {
    std::size_t bytes = 0, records = 0;
    bool rejected = false;
    void reset() noexcept { bytes = records = 0; rejected = false; }
};

inline const Standard_GUID& AttributeID() {
    static const Standard_GUID id("CF8B79A1-9935-40D9-8DD3-8D0D5FB7E4B2");
    return id;
}

inline bool EncodeBody(const Definition& value, bool clearDigest,
                       std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        bounded_curve::Writer writer(MaximumPayloadBytes);
        writer.raw(reinterpret_cast<const std::uint8_t*>("SYGL"), 4);
        writer.integer(Schema, 1); writer.integer(0, 3);
        writer.raw(value.owner.document); writer.raw(value.owner.entity);
        writer.raw(value.owner.definition); writer.raw(value.feature);
        writer.integer(value.definitionRevision, 8);
        const Digest empty{}; writer.raw(clearDigest ? empty : value.recipeDigest);
        writer.scalar(value.dimensionMetersPerUnit);
        writer.integer(std::uint8_t(value.interpolation), 1);
        writer.integer(std::uint8_t(value.holes), 1);
        writer.integer(std::uint8_t(value.caps), 1); writer.integer(0, 1);
        for (double scalar : value.orderAxis) writer.scalar(scalar);
        writer.integer(value.stations.size(), 1); writer.integer(0, 3);
        for (const Station& station : value.stations) {
            writer.raw(station.identifier); writer.scalar(station.orderParameter);
            writer.scalar(station.twistFromPreviousRadians);
            writer.raw(station.frame.identifier); writer.integer(station.frame.revision, 8);
            for (double scalar : station.frame.origin) writer.scalar(scalar);
            for (double scalar : station.frame.xAxis) writer.scalar(scalar);
            for (double scalar : station.frame.yAxis) writer.scalar(scalar);
            for (double scalar : station.frame.zAxis) writer.scalar(scalar);
            writer.integer(std::uint8_t(station.frame.handedness), 1);
            writer.integer(station.junctions.size(), 1); writer.integer(0, 2);
            for (const Junction& junction : station.junctions) {
                writer.raw(junction.identifier); writer.raw(junction.correspondence);
                writer.scalar(junction.local[0]); writer.scalar(junction.local[1]);
            }
            writer.integer(station.segments.size(), 1); writer.integer(0, 3);
            for (const Segment& segment : station.segments) {
                writer.raw(segment.identifier); writer.raw(segment.correspondence);
                writer.raw(segment.startJunction); writer.raw(segment.endJunction);
                bounded_curve::PersistedValue persisted;
                if (!bounded_curve::ToPersistedValue(segment.curveState, persisted)) return false;
                std::vector<std::uint8_t> curve, owner;
                if (!bounded_curve::Encode(persisted.value, curve)
                    || !bounded_curve::EncodeOwnerState(persisted.ownerState, owner)) return false;
                writer.integer(curve.size(), 4); writer.integer(owner.size(), 4);
                writer.raw(curve.data(), curve.size()); writer.raw(owner.data(), owner.size());
            }
        }
        if (!writer.valid || writer.bytes.size() > MaximumPayloadBytes - 32) return false;
        Digest envelope{};
        if (!bounded_curve::Hash(writer.bytes, MaximumPayloadBytes, envelope)) return false;
        writer.raw(envelope); if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool AssignCanonicalDigest(Definition& value) noexcept {
    try {
        std::vector<std::uint8_t> body;
        if (!EncodeBody(value, true, body)) return false;
        return bounded_curve::Hash(body, MaximumPayloadBytes, value.recipeDigest);
    } catch (...) { value.recipeDigest = {}; return false; }
}

inline bool Canonical(const Definition& value) noexcept {
    Definition copy = value;
    return AssignCanonicalDigest(copy) && copy.recipeDigest == value.recipeDigest
        && Validate(value) == Admission::Accepted;
}

inline bool Encode(const Definition& value,
                   std::vector<std::uint8_t>& output) noexcept {
    return Canonical(value) && EncodeBody(value, false, output);
}

inline bool Decode(const std::vector<std::uint8_t>& bytes,
                   Definition& output) noexcept {
    output = {};
    try {
        if (bytes.size() < 240 || bytes.size() > MaximumPayloadBytes) return false;
        const std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!bounded_curve::Hash(body, MaximumPayloadBytes, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin());
        if (expected != actual) return false;
        bounded_curve::Reader reader(body, body.size());
        std::array<std::uint8_t, 4> magic{}; std::uint64_t raw = 0, reserved = 0;
        Definition value;
        if (!reader.raw(magic) || magic != std::array<std::uint8_t, 4>{{'S','Y','G','L'}}
            || !reader.integer(1, raw) || raw != Schema
            || !reader.integer(3, reserved) || reserved != 0
            || !reader.raw(value.owner.document) || !reader.raw(value.owner.entity)
            || !reader.raw(value.owner.definition) || !reader.raw(value.feature)
            || !reader.integer(8, value.definitionRevision)
            || !reader.raw(value.recipeDigest) || !reader.scalar(value.dimensionMetersPerUnit)
            || !reader.integer(1, raw)) return false;
        value.interpolation = Interpolation(raw);
        if (!reader.integer(1, raw)) return false; value.holes = HolePolicy(raw);
        if (!reader.integer(1, raw)) return false; value.caps = CapPolicy(raw);
        if (!reader.integer(1, reserved) || reserved != 0) return false;
        for (double& scalar : value.orderAxis) if (!reader.scalar(scalar)) return false;
        std::uint64_t stationCount = 0;
        if (!reader.integer(1, stationCount) || stationCount < MinimumStations
            || stationCount > MaximumStations || !reader.integer(3, reserved)
            || reserved != 0) return false;
        value.stations.resize(std::size_t(stationCount));
        for (Station& station : value.stations) {
            if (!reader.raw(station.identifier) || !reader.scalar(station.orderParameter)
                || !reader.scalar(station.twistFromPreviousRadians)
                || !reader.raw(station.frame.identifier)
                || !reader.integer(8, station.frame.revision)) return false;
            for (double& scalar : station.frame.origin) if (!reader.scalar(scalar)) return false;
            for (double& scalar : station.frame.xAxis) if (!reader.scalar(scalar)) return false;
            for (double& scalar : station.frame.yAxis) if (!reader.scalar(scalar)) return false;
            for (double& scalar : station.frame.zAxis) if (!reader.scalar(scalar)) return false;
            if (!reader.integer(1, raw)) return false;
            station.frame.handedness = bounded_curve::Handedness(raw);
            std::uint64_t junctionCount = 0;
            if (!reader.integer(1, junctionCount) || junctionCount < MinimumSegments
                || junctionCount > MaximumSegments || !reader.integer(2, reserved)
                || reserved != 0) return false;
            station.junctions.resize(std::size_t(junctionCount));
            for (Junction& junction : station.junctions)
                if (!reader.raw(junction.identifier) || !reader.raw(junction.correspondence)
                    || !reader.scalar(junction.local[0]) || !reader.scalar(junction.local[1])) return false;
            std::uint64_t segmentCount = 0;
            if (!reader.integer(1, segmentCount) || segmentCount != junctionCount
                || !reader.integer(3, reserved) || reserved != 0) return false;
            station.segments.resize(std::size_t(segmentCount));
            for (Segment& segment : station.segments) {
                std::uint64_t curveCount = 0, ownerCount = 0;
                if (!reader.raw(segment.identifier) || !reader.raw(segment.correspondence)
                    || !reader.raw(segment.startJunction) || !reader.raw(segment.endJunction)
                    || !reader.integer(4, curveCount) || !reader.integer(4, ownerCount)
                    || curveCount == 0 || ownerCount == 0
                    || curveCount > bounded_curve::MaximumDefinitionBytes
                    || ownerCount > bounded_curve::MaximumOwnerBytes
                    || curveCount + ownerCount > reader.remaining()) return false;
                std::vector<std::uint8_t> curve(static_cast<std::size_t>(curveCount), 0);
                std::vector<std::uint8_t> owner(static_cast<std::size_t>(ownerCount), 0);
                bounded_curve::PersistedValue persisted;
                if (!reader.raw(curve.data(), curve.size())
                    || !reader.raw(owner.data(), owner.size())
                    || !bounded_curve::Decode(curve, persisted.value)
                    || !bounded_curve::DecodeOwnerState(owner, persisted.ownerState)
                    || !bounded_curve::FromPersistedValue(persisted, segment.curveState)) return false;
            }
        }
        std::vector<std::uint8_t> exact;
        if (!reader.complete() || !Encode(value, exact) || exact != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

struct Payload final { Definition definition; std::vector<std::uint8_t> bytes; };

class Attribute final : public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTI_INLINE(Attribute, TDF_Attribute)
    const Standard_GUID& ID() const override { return AttributeID(); }
    Handle(TDF_Attribute) NewEmpty() const override { return new Attribute(); }
    const std::shared_ptr<const Payload>& value() const noexcept { return value_; }
    void Restore(const Handle(TDF_Attribute)& source) override {
        const auto other = Handle(Attribute)::DownCast(source);
        if (other.IsNull()) Standard_Failure::Raise("General loft restore type");
        value_ = other->value_;
    }
    void Paste(const Handle(TDF_Attribute)& target,
               const Handle(TDF_RelocationTable)&) const override {
        const auto other = Handle(Attribute)::DownCast(target);
        if (other.IsNull() || !value_) Standard_Failure::Raise("General loft paste type");
        other->Backup(); other->value_ = value_;
    }
private:
    friend class BinaryDriver;
    friend class ::OcctDocument;
    std::shared_ptr<const Payload> value_;
};

struct Record { TDF_Label label, owner; std::shared_ptr<const Payload> value; TopoDS_Shape current; };

inline bool ReadAll(const Handle(TDocStd_Document)& document,
                    std::vector<Record>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        std::vector<Record> staged; std::set<UUID> features; std::size_t aggregate = 0;
        int visited = 0;
        for (TDF_ChildIterator it(document->GetData()->Root(), Standard_True); it.More(); it.Next()) {
            if (++visited > profile::MaximumLabels) return false;
            const TDF_Label label = it.Value();
            if (!label.IsAttribute(AttributeID())) continue;
            Handle(Attribute) attribute; Handle(TNaming_NamedShape) binding;
            const TDF_Label owner = label.Father();
            if (staged.size() >= MaximumRecords || label.Tag() != RecordTag
                || !label.FindAttribute(AttributeID(), attribute) || attribute.IsNull()
                || !attribute->value() || !label.FindAttribute(TNaming_NamedShape::GetID(), binding)
                || binding.IsNull() || !features.insert(attribute->value()->definition.feature).second
                || attribute->value()->bytes.size() > MaximumDocumentBytes - aggregate) return false;
            aggregate += attribute->value()->bytes.size();
            std::vector<std::uint8_t> exact;
            if (!Encode(attribute->value()->definition, exact)
                || exact != attribute->value()->bytes) return false;
            UUID documentID{}, entityID{}, definitionID{};
            if (!retained_solid::ReadUUID(document->Main(),
                    Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
                || !retained_solid::ReadUUID(owner,
                    Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entityID)
                || !retained_solid::ReadUUID(owner,
                    Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definitionID)
                || attribute->value()->definition.owner.document != documentID
                || attribute->value()->definition.owner.entity != entityID
                || attribute->value()->definition.owner.definition != definitionID) return false;
            const TopoDS_Shape current = binding->Get();
            if (current.IsNull() || current.ShapeType() != TopAbs_SOLID) return false;
            for (TDF_AttributeIterator a(label); a.More(); a.Next())
                if (a.Value()->ID() != AttributeID()
                    && a.Value()->ID() != TNaming_NamedShape::GetID()) return false;
            staged.push_back({label, owner, attribute->value(), current});
        }
        output = std::move(staged); return true;
    } catch (...) { output.clear(); return false; }
}

class BinaryDriver final : public BinMDF_ADriver {
public:
    BinaryDriver(const Handle(Message_Messenger)& messenger,
                 Handle(BinMNaming_NamedShapeDriver) shapes,
                 std::shared_ptr<ReadBudget> budget, void (*reject)() noexcept = nullptr)
        : BinMDF_ADriver(messenger, STANDARD_TYPE(Attribute)->Name()),
          shapes_(std::move(shapes)), budget_(std::move(budget)), reject_(reject) {}
    Handle(TDF_Attribute) NewEmpty() const override { return new Attribute(); }
    const Handle(Standard_Type)& SourceType() const override { return STANDARD_TYPE(Attribute); }
    Standard_Boolean Paste(const BinObjMgt_Persistent& source,
                           const Handle(TDF_Attribute)& target,
                           BinObjMgt_RRelocationTable&) const override {
        try {
            const auto attribute = Handle(Attribute)::DownCast(target);
            Standard_Integer schema = 0, count = 0;
            if (attribute.IsNull() || attribute->value_ || shapes_.IsNull() || !budget_
                || budget_->rejected || !(source >> schema >> count) || schema != 1
                || count <= 0 || count > Standard_Integer(MaximumPayloadBytes)
                || budget_->records >= MaximumRecords
                || std::size_t(count) > MaximumDocumentBytes - budget_->bytes) return refuse();
            std::vector<std::uint8_t> bytes(static_cast<std::size_t>(count), 0);
            Definition definition;
            if (!source.GetByteArray(bytes.data(), count) || !Decode(bytes, definition)) return refuse();
            auto payload = std::make_shared<Payload>(); payload->definition = std::move(definition);
            payload->bytes = std::move(bytes); attribute->value_ = std::move(payload);
            budget_->bytes += std::size_t(count); ++budget_->records; return Standard_True;
        } catch (...) { return refuse(); }
    }
    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&) const override {
        const auto attribute = Handle(Attribute)::DownCast(source);
        std::vector<std::uint8_t> bytes;
        if (attribute.IsNull() || !attribute->value_ || !Encode(attribute->value_->definition, bytes)
            || bytes != attribute->value_->bytes) Standard_Failure::Raise("General loft writer value");
        target << Standard_Integer(1) << Standard_Integer(bytes.size());
        target.PutByteArray(bytes.data(), Standard_Integer(bytes.size()));
    }
private:
    Standard_Boolean refuse() const noexcept {
        if (budget_) budget_->rejected = true; if (reject_) reject_(); return Standard_False;
    }
    Handle(BinMNaming_NamedShapeDriver) shapes_; std::shared_ptr<ReadBudget> budget_;
    void (*reject_)() noexcept;
};

inline void Register(const Handle(BinMDF_ADriverTable)& table,
                     const Handle(Message_Messenger)& messenger,
                     const std::shared_ptr<ReadBudget>& budget,
                     void (*reject)() noexcept = nullptr) {
    Handle(BinMDF_ADriver) found; table->GetDriver(STANDARD_TYPE(TNaming_NamedShape), found);
    const auto shapes = Handle(BinMNaming_NamedShapeDriver)::DownCast(found);
    if (shapes.IsNull()) Standard_Failure::Raise("General loft shared driver missing");
    table->AddDriver(new BinaryDriver(messenger, shapes, budget, reject));
}

template<class Base> class StorageDriver : public Base {
public:
    Handle(BinMDF_ADriverTable) AttributeDrivers(const Handle(Message_Messenger)& messenger) override {
        auto table = Base::AttributeDrivers(messenger);
        Register(table, messenger, std::make_shared<ReadBudget>()); return table;
    }
    void Write(const Handle(CDM_Document)& document, const TCollection_ExtendedString& file,
               const Message_ProgressRange& progress = Message_ProgressRange()) override {
        prepare(document); Base::Write(document, file, progress);
    }
    void Write(const Handle(CDM_Document)& document, Standard_OStream& stream,
               const Message_ProgressRange& progress = Message_ProgressRange()) override {
        prepare(document); Base::Write(document, stream, progress);
    }
private:
    static void prepare(const Handle(CDM_Document)& source) {
        std::vector<Record> records;
        if (!ReadAll(Handle(TDocStd_Document)::DownCast(source), records))
            Standard_Failure::Raise("General loft writer document");
    }
};
} // namespace core3d::general_loft::persistence
