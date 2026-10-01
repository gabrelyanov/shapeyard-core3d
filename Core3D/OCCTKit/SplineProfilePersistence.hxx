#pragma once

// C4-N owns a separate canonical record.  The legacy ProfilePersistence
// schema remains 5; spline source is never down-converted into line/arc data.
#include "BoundedCurveDefinition.hxx"
#include "SplineProfileFace.hxx"
#include "RetainedRecipeIdentity.hxx"
#include <BinMDF_ADriver.hxx>
#include <BinMDF_ADriverTable.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <CommonCrypto/CommonDigest.h>
#include <TDF_Attribute.hxx>
#include <TNaming_NamedShape.hxx>
#include <TDocStd_Document.hxx>
#include <TDocStd_FormatVersion.hxx>
#include <climits>
#include <cstring>
#include <limits>
#include <memory>

class OcctDocument;

namespace core3d::spline_profile {
using UUID = retained_recipe::UUID;
using Digest = std::array<std::uint8_t, 32>;
inline constexpr std::uint32_t Schema = 1;
inline constexpr std::size_t MaximumPayloadBytes = 256 * 1024;
inline constexpr std::size_t MaximumRecordsPerDocument = 128;
inline constexpr std::size_t MaximumDocumentAggregateBytes =
    MaximumPayloadBytes * MaximumRecordsPerDocument;
inline constexpr Standard_Integer RecordTag = 0x304;

enum class Operation : std::uint8_t { extrude = 1, revolve = 2 };
enum class Refusal : std::uint8_t {
    none, badSchema, badRegistryKey, badOwner, badUnits, badFrame,
    badOperation, badExtent, badIdentity, duplicateIdentity, badSection,
    c1NotSketch, c1RationalUnsupported, c1PeriodicUnsupported,
    c1PoleBudget, segmentBudget, nonCanonical
};

struct StableIdentity final {
    UUID uuid{};
    ProfileCurveID local = 0;
    bool operator==(const StableIdentity& value) const noexcept {
        return uuid == value.uuid && local == value.local;
    }
};

struct Definition final {
    std::uint32_t schema = Schema;
    retained_feature::Key key = SplineProfileRevolveRegistryKey;
    retained_recipe::OwnerKey owner;
    UUID feature{};
    std::uint64_t revision = 0;
    std::uint64_t nextLocalIdentity = 0;
    double metersPerUnit = 0;
    bounded_curve::Frame frame;
    int plane = 0;
    Operation operation = Operation::extrude;
    double depth = 0;
    double angleDegrees = 0;
    ProfileCurveSection section;
    SplineProfileSpec spline;
    std::vector<StableIdentity> identities;
};

struct DetachedSolid final {
    TopoDS_Solid solid;
    std::vector<std::uint8_t> canonical;
    Digest digest{};
    double expectedVolume = 0;
    int seamFaces = 0;
};

//! Narrow bridge into the existing detached profile builder's spline branch.
//! The declaration lives with the C4 carrier so the owner does not duplicate
//! viewport-owned construction policy; Core3DViewer.mm supplies the body.
bool BuildWithDetachedProfileBranch(const ProfileCurveSection&,
    const SplineProfileSpec&, int plane, double extent, bool revolve,
    TopoDS_Solid& solid, int& seamFaces) noexcept;

inline bool Finite(double value) noexcept {
    return std::isfinite(value) && std::abs(value) <= bounded_curve::CoordinateLimit;
}

inline Refusal Validate(const Definition& value) noexcept {
    try {
        if (value.schema != Schema) return Refusal::badSchema;
        if (value.key.kind != retained_feature::SplineProfileRevolveKind
            || value.key.codecVersion != SplineProfileRevolveRegistryKey.codecVersion)
            return Refusal::badRegistryKey;
        if (!retained_recipe::Valid(value.owner)
            || !retained_recipe::Nonzero(value.feature)
            || value.revision == 0 || value.nextLocalIdentity == 0)
            return Refusal::badOwner;
        if (!std::isfinite(value.metersPerUnit) || value.metersPerUnit <= 0
            || value.metersPerUnit > 1) return Refusal::badUnits;
        if (bounded_curve::ValidateFrame(value.frame)
                != bounded_curve::Refusal::None
            || value.frame.revision != value.revision) return Refusal::badFrame;
        if (value.plane < 0 || value.plane > 2) return Refusal::badSection;
        if (value.operation == Operation::extrude) {
            if (!Finite(value.depth) || value.depth <= 0 || value.angleDegrees != 0
                || value.spline.revolveAxis) return Refusal::badExtent;
        } else if (value.operation == Operation::revolve) {
            if (value.depth != 0 || !Finite(value.angleDegrees)
                || value.angleDegrees <= 0 || value.angleDegrees > 360
                || !value.spline.revolveAxis) return Refusal::badExtent;
        } else return Refusal::badOperation;
        if (value.spline.segments.empty()
            || value.spline.segments.size() > SplineMaximumSegmentsPerSection)
            return Refusal::segmentBudget;
        SplineProfileSectionMoments moments;
        if (!InspectSplineProfileSection(value.section, value.spline, moments))
            return Refusal::badSection;
        std::set<UUID> uuids;
        std::set<ProfileCurveID> locals;
        for (const auto& identity : value.identities)
            if (!retained_recipe::Nonzero(identity.uuid) || identity.local == 0
                || identity.local >= value.nextLocalIdentity
                || !uuids.insert(identity.uuid).second
                || !locals.insert(identity.local).second)
                return Refusal::duplicateIdentity;
        std::set<ProfileCurveID> required;
        for (const auto& segment : value.spline.segments) {
            if (!required.insert(segment.identifier).second) return Refusal::badIdentity;
            for (const auto& pole : segment.poles)
                if (!required.insert(pole.identifier).second) return Refusal::badIdentity;
        }
        if (locals != required) return Refusal::badIdentity;
        return Refusal::none;
    } catch (...) { return Refusal::badSection; }
}

// Explicit C1 -> C4 value/identity translation.  C1 remains the UUID source;
// C4 assigns and persists one local builder ID for that UUID.  C4 does not
// retain a live dependency on the standalone C1 owner in this bounded slice.
inline Refusal ImportC1SplineValue(const bounded_curve::Definition& source,
    ProfileCurveID segmentID, ProfileCurveID firstPoleID,
    const UUID& segmentUUID, SplineCurveSegment& segment,
    std::vector<StableIdentity>& identities) noexcept {
    segment = {};
    try {
        if (bounded_curve::Validate(source) != bounded_curve::Refusal::None
            || source.domain != bounded_curve::Domain::Sketch2D)
            return Refusal::c1NotSketch;
        if (source.controlPoints.size() > SplineMaximumPoles)
            return Refusal::c1PoleBudget;
        if (segmentID == 0 || firstPoleID == 0
            || !retained_recipe::Nonzero(segmentUUID)
            || std::uint64_t(firstPoleID) + source.controlPoints.size() - 1
                > std::numeric_limits<ProfileCurveID>::max()) return Refusal::badIdentity;
        // The guarded C4 owner intentionally narrows C1: rational and
        // periodic/unclamped structural values remain unsupported.
        if (!source.weights.empty()) return Refusal::c1RationalUnsupported;
        if (source.knots.front().multiplicity != source.degree + 1
            || source.knots.back().multiplicity != source.degree + 1)
            return Refusal::c1PeriodicUnsupported;
        std::set<UUID> uuids;
        std::set<ProfileCurveID> locals;
        for (const auto& identity : identities) {
            uuids.insert(identity.uuid); locals.insert(identity.local);
        }
        if (!uuids.insert(segmentUUID).second || !locals.insert(segmentID).second)
            return Refusal::duplicateIdentity;
        SplineCurveSegment candidate;
        std::vector<StableIdentity> additions{{segmentUUID, segmentID}};
        candidate.identifier = segmentID;
        candidate.degree = source.degree;
        for (std::size_t index = 0; index < source.controlPoints.size(); ++index) {
            const auto& point = source.controlPoints[index];
            const ProfileCurveID local = firstPoleID + ProfileCurveID(index);
            if (!uuids.insert(point.identifier).second || !locals.insert(local).second)
                return Refusal::duplicateIdentity;
            candidate.poles.push_back({local, gp_Pnt2d(point.local[0], point.local[1])});
            additions.push_back({point.identifier, local});
        }
        for (const auto& knot : source.knots) {
            candidate.knots.push_back(knot.value);
            candidate.multiplicities.push_back(knot.multiplicity);
        }
        segment = std::move(candidate);
        identities.insert(identities.end(), additions.begin(), additions.end());
        return Refusal::none;
    } catch (...) { segment = {}; return Refusal::badSection; }
}

class Writer final {
public:
    std::vector<std::uint8_t> bytes;
    bool ok = true;
    void integer(std::uint64_t value, unsigned width) {
        if (!ok || width == 0 || width > 8
            || bytes.size() > MaximumPayloadBytes - width) { ok = false; return; }
        for (unsigned index = 0; index < width; ++index)
            bytes.push_back(std::uint8_t(value >> (8 * index)));
    }
    void scalar(double value) {
        if (!std::isfinite(value)) { ok = false; return; }
        std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits)); integer(bits, 8);
    }
    void uuid(const UUID& value) { for (auto byte : value) integer(byte, 1); }
};

inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (Validate(value) != Refusal::none) return false;
        Writer w;
        for (const char c : std::string("SPRO")) w.integer(std::uint8_t(c), 1);
        w.integer(Schema, 1); w.integer(0, 3);
        w.integer(value.key.kind, 4); w.integer(value.key.codecVersion, 4);
        w.uuid(value.owner.document); w.uuid(value.owner.entity); w.uuid(value.owner.definition);
        w.uuid(value.feature); w.integer(value.revision, 8); w.integer(value.nextLocalIdentity, 8);
        w.scalar(value.metersPerUnit); w.uuid(value.frame.identifier);
        w.integer(value.frame.revision, 8);
        for (double scalar : value.frame.origin) w.scalar(scalar);
        for (double scalar : value.frame.xAxis) w.scalar(scalar);
        for (double scalar : value.frame.yAxis) w.scalar(scalar);
        for (double scalar : value.frame.zAxis) w.scalar(scalar);
        w.integer(std::uint8_t(value.frame.handedness), 1);
        w.integer(value.plane, 1); w.integer(std::uint8_t(value.operation), 1);
        w.integer(std::uint8_t(value.spline.innerLoopPolicy), 1);
        w.scalar(value.depth); w.scalar(value.angleDegrees);
        w.integer(value.spline.revolveAxis ? 1 : 0, 1);
        if (value.spline.revolveAxis) {
            w.scalar(value.spline.revolveAxis->origin.X());
            w.scalar(value.spline.revolveAxis->origin.Y());
            w.scalar(value.spline.revolveAxis->direction.X());
            w.scalar(value.spline.revolveAxis->direction.Y());
        }
        const auto loop = [&](const ProfileCurveLoop& item, Writer& out) {
            out.integer(item.identifier, 4); out.integer(item.vertices.size(), 2);
            for (const auto& vertex : item.vertices) {
                out.integer(vertex.identifier, 4); out.scalar(vertex.point.X()); out.scalar(vertex.point.Y());
            }
            for (const auto& edge : item.segments) {
                out.integer(edge.identifier, 4); out.integer(edge.startVertex, 4);
                out.integer(edge.endVertex, 4); out.integer(std::uint8_t(edge.kind), 1);
                out.scalar(edge.center.X()); out.scalar(edge.center.Y()); out.scalar(edge.radius);
                out.scalar(edge.startDegrees); out.scalar(edge.sweepDegrees);
            }
        };
        loop(value.section.outer, w); w.integer(value.section.inner.size(), 1);
        for (const auto& inner : value.section.inner) loop(inner, w);
        w.integer(value.spline.segments.size(), 1);
        for (const auto& segment : value.spline.segments) {
            w.integer(segment.identifier, 4); w.integer(segment.degree, 1);
            w.integer(segment.poles.size(), 1); w.integer(segment.knots.size(), 1);
            w.integer(segment.weights.empty() ? 0 : 1, 1);
            for (const auto& pole : segment.poles) {
                w.integer(pole.identifier, 4); w.scalar(pole.point.X()); w.scalar(pole.point.Y());
            }
            for (std::size_t index = 0; index < segment.knots.size(); ++index) {
                w.scalar(segment.knots[index]); w.integer(segment.multiplicities[index], 1);
            }
            for (double weight : segment.weights) w.scalar(weight);
        }
        w.integer(value.identities.size(), 2);
        for (const auto& identity : value.identities) { w.uuid(identity.uuid); w.integer(identity.local, 4); }
        if (!w.ok || w.bytes.empty() || w.bytes.size() > MaximumPayloadBytes - 32) return false;
        Digest digest{};
        if (!CC_SHA256(w.bytes.data(), CC_LONG(w.bytes.size()), digest.data())) return false;
        w.bytes.insert(w.bytes.end(), digest.begin(), digest.end()); output = std::move(w.bytes);
        return true;
    } catch (...) { output.clear(); return false; }
}

// Decode is intentionally declared out-of-line in SplineProfileOwner.mm so
// the reader and owner share one canonical parser and one budget boundary.
bool Decode(const std::vector<std::uint8_t>&, Definition&) noexcept;
bool Build(const Definition&, DetachedSolid&) noexcept;

inline const Standard_GUID& AttributeID() {
    static const Standard_GUID id("E5BBF47C-90EC-4A42-8FA3-FD4661D9A520"); return id;
}

class Core3D_SplineProfile final : public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTI_INLINE(Core3D_SplineProfile, TDF_Attribute)
    const Standard_GUID& ID() const override { return AttributeID(); }
    Handle(TDF_Attribute) NewEmpty() const override { return new Core3D_SplineProfile(); }
    void Restore(const Handle(TDF_Attribute)& source) override {
        const auto value = Handle(Core3D_SplineProfile)::DownCast(source);
        if (value.IsNull()) Standard_Failure::Raise("C4 restore type");
        bytes_ = value->bytes_; definition_ = value->definition_;
    }
    void Paste(const Handle(TDF_Attribute)& target,
               const Handle(TDF_RelocationTable)&) const override {
        const auto value = Handle(Core3D_SplineProfile)::DownCast(target);
        if (value.IsNull()) Standard_Failure::Raise("C4 paste type");
        value->Backup(); value->bytes_ = bytes_; value->definition_ = definition_;
    }
    const Definition& definition() const noexcept { return definition_; }
    const std::vector<std::uint8_t>& bytes() const noexcept { return bytes_; }
private:
    friend class BinaryDriver;
    friend class ::OcctDocument;
    Definition definition_;
    std::vector<std::uint8_t> bytes_;
};
using Attribute = Core3D_SplineProfile;

struct Record final { TDF_Label label; TDF_Label owner; Handle(Attribute) attribute; TopoDS_Shape shape; };
bool ReadAll(const Handle(TDocStd_Document)&, std::vector<Record>&) noexcept;

struct ReadBudget final {
    std::size_t bytes = 0, records = 0;
    bool rejected = false;
    void reset() noexcept { bytes = records = 0; rejected = false; }
};

class BinaryDriver final : public BinMDF_ADriver {
public:
    BinaryDriver(const Handle(Message_Messenger)& messenger,
                 std::shared_ptr<ReadBudget> budget,
                 void (*reject)() noexcept = nullptr)
        : BinMDF_ADriver(messenger, STANDARD_TYPE(Attribute)->Name()),
          budget_(std::move(budget)), reject_(reject) {}
    Handle(TDF_Attribute) NewEmpty() const override { return new Attribute(); }
    const Handle(Standard_Type)& SourceType() const override { return STANDARD_TYPE(Attribute); }
    Standard_Boolean Paste(const BinObjMgt_Persistent& source,
        const Handle(TDF_Attribute)& target, BinObjMgt_RRelocationTable&) const override;
    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
        BinObjMgt_SRelocationTable&) const override;
private:
    Standard_Boolean Refuse() const noexcept {
        if (budget_) budget_->rejected = true;
        if (reject_) reject_();
        return Standard_False;
    }
    std::shared_ptr<ReadBudget> budget_;
    void (*reject_)() noexcept;
};

inline void Register(const Handle(BinMDF_ADriverTable)& table,
                     const Handle(Message_Messenger)& messenger,
                     const std::shared_ptr<ReadBudget>& budget,
                     void (*reject)() noexcept = nullptr) {
    table->AddDriver(new BinaryDriver(messenger, budget, reject));
}

// Storage wrapper for the production writers: installs the one existing
// BinaryDriver on the base table so save serializes the C4 record with the
// same canonical codec the reader already uses.
template<class Base> class StorageDriver : public Base {
public:
    Handle(BinMDF_ADriverTable) AttributeDrivers(const Handle(Message_Messenger)& messenger) override {
        auto table = Base::AttributeDrivers(messenger);
        Register(table, messenger, std::make_shared<ReadBudget>());
        return table;
    }
};
} // namespace core3d::spline_profile
