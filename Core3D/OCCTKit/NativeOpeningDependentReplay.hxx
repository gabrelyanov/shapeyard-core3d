#pragma once

// Shared fail-closed boundary for topology-changing edits whose source or host
// is retained by D2, D3, or D4.  Discovery is owned by OcctDocument so callers
// cannot omit a record.  Family-specific collaborators supply detached work,
// but cannot add, remove, or reorder the discovered closure.

#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>
#include "RetainedBooleanProgram.hxx"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class OcctDocument;
namespace core3d::native_opening { class CommandLease; class Context; }

// E3 face-image dependent replay (278b portion 3) shares this header; its
// includes sit at file scope like the rest of this boundary.
#include "FaceImageDefinition.hxx"
#include "FaceImagePersistence.hxx"
#include "RetainedFaceSelector.hxx"
#include "OcctDocument.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>

#include <atomic>
#include <cstring>

namespace core3d::dependent_replay {

using UUID = std::array<std::uint8_t, 16>;

enum class Family : std::uint8_t {
    PatternD2 = 2,
    PathArrayD3 = 3,
    FeaturePatternD4 = 4
};

enum class Mutation : std::uint8_t { Replace = 1, Remove = 2 };

enum class Refusal : std::uint8_t {
    None = 0,
    ClosedDocument,
    OpenCommand,
    StaleTarget,
    CorruptTable,
    MissingRecipe,
    UnsupportedDescendant,
    Cycle,
    RecordBudget,
    DocumentBudget,
    MemoryBudget,
    TopologyBudget,
    StaleClosure,
    ForeignLease,
    SourceStageFailed,
    DependentStageFailed,
    ReadbackFailed
};

struct Limits final {
    std::size_t records = 128;
    std::size_t documentBytes = 8 * 1024 * 1024;
    std::size_t memoryBytes = 256 * 1024 * 1024;
    std::size_t topologyNodes = 2'000'000;
};

//! Exact retained-record edge in the transitive closure. canonicalRecordBytes
//! are the actual SYPT/SYPA/SYFP bytes read from OCAF, never a reconstructed
//! descriptor. memberIdentities preserve the retained member/child identities.
struct Dependency final {
    Family family = Family::PatternD2;
    UUID feature{};
    UUID inputEntity{};
    UUID resultEntity{};
    TDF_Label recordLabel;
    std::vector<std::uint8_t> canonicalRecordBytes;
    std::vector<UUID> memberIdentities;

    bool exactlyEquals(const Dependency& other) const noexcept {
        return family == other.family && feature == other.feature
            && inputEntity == other.inputEntity && resultEntity == other.resultEntity
            && !recordLabel.IsNull() && recordLabel.IsEqual(other.recordLabel)
            && canonicalRecordBytes == other.canonicalRecordBytes
            && memberIdentities == other.memberIdentities;
    }
};

//! Family-specific detached work. prepare() is called before any command.
//! stage() may only use the supplied caller lease; it must neither commit nor
//! begin nested work. read() proves the staged/committed identity and recipe.
class PreparedReplay {
public:
    virtual ~PreparedReplay() = default;
    virtual Family family() const noexcept = 0;
    virtual UUID feature() const noexcept = 0;
    virtual UUID resultEntity() const noexcept = 0;
    virtual std::size_t documentBytes() const noexcept = 0;
    virtual std::size_t memoryBytes() const noexcept = 0;
    virtual std::size_t topologyNodes() const noexcept = 0;
    virtual bool openingCurrent(OcctDocument&) const noexcept = 0;
    virtual bool stage(OcctDocument&, native_opening::CommandLease&) const noexcept = 0;
    virtual bool read(OcctDocument&) const noexcept = 0;
};

class Preparer {
public:
    virtual ~Preparer() = default;
    virtual Refusal prepare(OcctDocument&, const Dependency&, Mutation,
        std::shared_ptr<const PreparedReplay>&) noexcept = 0;
};

//! Prospective source/host state produced by the accepted source editor. It is
//! detached and immutable; family collaborators still capture all old OCAF
//! authority themselves before preparing a replay.
struct Candidate final {
    std::string targetEntityIdentifier;
    TopoDS_Shape resultShape;
    TopoDS_Shape retainedBase;
    retained_boolean::Recipe retainedRecipe;
    std::vector<std::uint8_t> retainedRecipeBytes;
    bool hasRetainedRecipe = false;
};

Refusal PreparePatternD2Replay(OcctDocument&, const Dependency&, Mutation,
    const Candidate&, std::shared_ptr<const PreparedReplay>&) noexcept;
Refusal PreparePathArrayD3Replay(OcctDocument&, const Dependency&, Mutation,
    const Candidate&, const std::shared_ptr<native_opening::Context>&,
    std::shared_ptr<const PreparedReplay>&) noexcept;
Refusal PrepareFeaturePatternD4Replay(OcctDocument&, const Dependency&, Mutation,
    const Candidate&, const std::shared_ptr<native_opening::Context>&,
    std::shared_ptr<const PreparedReplay>&) noexcept;

class ProductionPreparer final : public Preparer {
public:
    ProductionPreparer(Candidate candidate,
        std::shared_ptr<native_opening::Context> context) noexcept
        : candidate_(std::move(candidate)), context_(std::move(context)) {}
    Refusal prepare(OcctDocument& owner, const Dependency& dependency,
        Mutation mutation,
        std::shared_ptr<const PreparedReplay>& output) noexcept override {
        output.reset();
        if (!context_ || candidate_.targetEntityIdentifier.empty()
            || candidate_.resultShape.IsNull()) return Refusal::MissingRecipe;
        switch (dependency.family) {
            case Family::PatternD2:
                return PreparePatternD2Replay(owner, dependency, mutation,
                                              candidate_, output);
            case Family::PathArrayD3:
                return PreparePathArrayD3Replay(owner, dependency, mutation,
                    candidate_, context_, output);
            case Family::FeaturePatternD4:
                return PrepareFeaturePatternD4Replay(owner, dependency, mutation,
                    candidate_, context_, output);
        }
        return Refusal::UnsupportedDescendant;
    }
private:
    Candidate candidate_;
    std::shared_ptr<native_opening::Context> context_;
};

//! The source/host mutation is injected so OcctDocument can keep its narrow
//! authorization active for exactly source stage + complete dependent stage.
class SourceMutation {
public:
    virtual ~SourceMutation() = default;
    virtual bool stage(OcctDocument&, native_opening::CommandLease&) noexcept = 0;
};

class Plan final {
public:
    Plan() = delete;
    Plan(const Plan&) = delete;
    Plan& operator=(const Plan&) = delete;

    const std::vector<Dependency>& dependencies() const noexcept {
        return dependencies_;
    }
    Mutation mutation() const noexcept { return mutation_; }
    const std::string& targetEntityIdentifier() const noexcept {
        return targetEntityIdentifier_;
    }

private:
    friend class ::OcctDocument;
    Plan(Mutation mutation, std::string entity, std::string definition)
        : mutation_(mutation), targetEntityIdentifier_(std::move(entity)),
          targetDefinitionIdentifier_(std::move(definition)) {}

    Mutation mutation_ = Mutation::Replace;
    std::string documentIdentifier_;
    std::string targetEntityIdentifier_;
    std::string targetDefinitionIdentifier_;
    const void* documentData_ = nullptr;
    Standard_Integer documentTime_ = 0;
    Limits limits_;
    std::vector<Dependency> dependencies_;
    std::vector<std::shared_ptr<const PreparedReplay>> prepared_;
    std::vector<std::string> authorizedEntityIdentifiers_;
};

// ---------------------------------------------------------------------------
// E3 face-image dependent replay (278b portion 3).
//
// The D2/D3/D4 closure above is discovered from retained records owned by
// OcctDocument and is unchanged. E3 face-image bindings are attachments on the
// edited retained-solid owner itself, so they are captured, prepared, staged,
// read and deleted here through the portion-2 document face-image transaction
// surface (OcctDocument::ReadFaceImageBindings / PrepareFaceImageBindings /
// CommitFaceImageBindings / RemoveFaceImageBindings), never through a new
// mutation route. One source command stages geometry and all attachments, or
// aborts: prepare runs the positive same-shape bijection entirely before any
// mutation and refuses the whole command on a lost, split, merged, ambiguous
// or unsupported face; it never deletes a binding to let a geometry edit
// pass. Reattachment is only ever to the same owner entity; propagation to a
// Boolean/duplication result entity is UnsupportedDownstream.
//
// Durable face authority is the committed (face UUID, selectorProof) pair of
// portion 1's SYFI/1 record. selectorProof is SHA-256 over the canonical
// geometric B2 receipt (intent, plane witness, coverage, wire/use/unique-edge
// counts); the random per-issuance anchor keys are deliberately excluded so
// the proof is exactly reproducible from current B2 resolution after a cold
// reopen, and changes precisely when the resolved face geometry changes. A
// tessellation ordinal is never durable identity here either.

enum class FaceImageReplayStatus : std::uint8_t {
    Captured = 0,           // CaptureFaceImageAttachment succeeded
    Absent,                 // no committed record; replay is a no-op
    Prepared,               // bijective reattachment prepared / committed
    NoChange,               // prepared; refreshed record identical to committed
    Deleted,                // removal mutation prepared / committed
    StaleSource,
    StaleFace,
    AmbiguousFaceRemap,
    UnsupportedSurface,
    UnsupportedDownstream,
    MissingResource,
    OwnerMismatch,
    Malformed,
    Budget,
    Busy,
    PersistenceFailure
};

//! The geometric content of one current B2 receipt, excluding the random
//! anchor keys. Bit-exact equality drives both currentness and the proof.
struct FaceImageGeometricReceipt final {
    retained_face_selector::SelectorIntent intent{
        retained_face_selector::PlanarFaceBoundary{}};
    retained_face_selector::PlaneWitness plane;
    retained_face_selector::Coverage coverage =
        retained_face_selector::Coverage::EntireBoundary;
    std::uint32_t wireCount = 0;
    std::uint32_t boundaryUseCount = 0;
    std::uint32_t boundaryUniqueEdgeCount = 0;
    bool operator==(const FaceImageGeometricReceipt&) const noexcept = default;
};

namespace detail {
struct FaceImageProofWriter final {
    std::vector<std::uint8_t> bytes;
    bool ok = true;
    void raw(const std::uint8_t* value, std::size_t count) {
        if (!ok || bytes.size() > 4096 || count > 4096 - bytes.size()) { ok = false; return; }
        bytes.insert(bytes.end(), value, value + count);
    }
    template<std::size_t N> void raw(const std::array<std::uint8_t, N>& value) {
        raw(value.data(), N);
    }
    void u(std::uint64_t value, unsigned width) {
        std::uint8_t encoded[8]{};
        if (width == 0 || width > 8) { ok = false; return; }
        for (unsigned index = 0; index < width; ++index) encoded[index] = std::uint8_t(value >> (8 * index));
        raw(encoded, width);
    }
    void d(double value) {
        if (!std::isfinite(value)) { ok = false; return; }
        std::uint64_t bits = 0;
        static_assert(sizeof bits == sizeof value, "double width");
        std::memcpy(&bits, &value, sizeof bits);
        u(bits, 8);
    }
};

inline bool FaceImageLengthUnit(OcctDocument& owner, double& output) noexcept {
    output = 0;
    try {
        const auto document = owner.Document();
        if (document.IsNull()) return false;
        return XCAFDoc_DocumentTool::GetLengthUnit(document, output)
            && std::isfinite(output) && output > 0;
    } catch (...) { output = 0; return false; }
}

inline TopoDS_Shape FaceImageOwnerShape(OcctDocument& owner,
    const face_image::OwnerKey& key, TDF_Label& label) noexcept {
    label = TDF_Label();
    try {
        const auto document = owner.Document();
        if (document.IsNull()
            || !face_image::owner::ResolveOwnerLabel(document, key, label)
            || label.IsNull()) return TopoDS_Shape();
        return XCAFDoc_ShapeTool::GetShape(label);
    } catch (...) { label = TDF_Label(); return TopoDS_Shape(); }
}

//! Deterministic B2 intent derivation for one planar, axis-parallel face at a
//! bounding-box extreme of the stage — the exact scope vocabulary the B2
//! resolver admits. Faces outside that vocabulary (curved, oblique, interior,
//! or with a non-line/duplicated/over-64 boundary) are not addressable; the
//! caller refuses rather than approximating by nearest geometry. Every
//! inspected face/wire/edge is charged to the caller's shared budget.
enum class FaceImageDeriveStatus : std::uint8_t { Derived, NotAddressable, Budget };
inline FaceImageDeriveStatus DeriveFaceImageReceipt(const Bnd_Box& stageBox,
    const TopoDS_Face& face, double metersPerLocalUnit,
    retained_edge_treatment::ReplayBudget& budget,
    FaceImageGeometricReceipt& output) noexcept {
    namespace fs = retained_face_selector;
    namespace tb = retained_topology_budget;
    output = FaceImageGeometricReceipt{};
    try {
        if (!budget.visit(1, tb::Site::C02FacePasses)) return FaceImageDeriveStatus::Budget;
        BRepAdaptor_Surface surface(face, true);
        if (surface.GetType() != GeomAbs_Plane || stageBox.IsVoid()
            || !std::isfinite(metersPerLocalUnit) || metersPerLocalUnit <= 0)
            return FaceImageDeriveStatus::NotAddressable;
        gp_Pln plane = surface.Plane();
        if (face.Orientation() == TopAbs_REVERSED) plane.SetAxis(plane.Axis().Reversed());
        const gp_Dir& normal = plane.Axis().Direction();
        const double components[3] = {normal.X(), normal.Y(), normal.Z()};
        int axisIndex = 0;
        for (int index = 1; index < 3; ++index)
            if (std::abs(components[index]) > std::abs(components[axisIndex])) axisIndex = index;
        const double sign = components[axisIndex] >= 0 ? 1.0 : -1.0;
        gp_Dir axisDirection(axisIndex == 0 ? sign : 0.0,
                             axisIndex == 1 ? sign : 0.0,
                             axisIndex == 2 ? sign : 0.0);
        if (!normal.IsParallel(axisDirection, 1e-8)) return FaceImageDeriveStatus::NotAddressable;
        Standard_Real xMin = 0, yMin = 0, zMin = 0, xMax = 0, yMax = 0, zMax = 0;
        stageBox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        const double bounds[3][2] = {{xMin, xMax}, {yMin, yMax}, {zMin, zMax}};
        const double coordinate = axisIndex == 0 ? plane.Location().X()
            : axisIndex == 1 ? plane.Location().Y() : plane.Location().Z();
        const double localTolerance = 1e-4 / (1000.0 * metersPerLocalUnit);
        const bool atMin = std::abs(coordinate - bounds[axisIndex][0]) <= localTolerance;
        const bool atMax = std::abs(coordinate - bounds[axisIndex][1]) <= localTolerance;
        if (atMin == atMax) return FaceImageDeriveStatus::NotAddressable;
        TopTools_IndexedMapOfShape wireMap;
        TopExp::MapShapes(face, TopAbs_WIRE, wireMap);
        TopTools_IndexedMapOfShape edgeMap;
        TopExp::MapShapes(face, TopAbs_EDGE, edgeMap);
        std::size_t uses = 0;
        for (TopExp_Explorer wires(face, TopAbs_WIRE); wires.More(); wires.Next()) {
            for (TopExp_Explorer edges(wires.Current(), TopAbs_EDGE); edges.More(); edges.Next()) {
                if (!budget.visit(1, tb::Site::C06DirectCensus)) return FaceImageDeriveStatus::Budget;
                ++uses;
            }
        }
        if (wireMap.Extent() < 1 || wireMap.Extent() > 64 || uses == 0 || uses > 64
            || edgeMap.Extent() != static_cast<int>(uses))
            return FaceImageDeriveStatus::NotAddressable;
        fs::PlanarFaceBoundary scope;
        scope.face.axis = axisIndex == 0 ? fs::Axis::X : axisIndex == 1 ? fs::Axis::Y : fs::Axis::Z;
        scope.face.side = atMax ? fs::Side::Max : fs::Side::Min;
        scope.edgeKind = fs::BoundaryCurve::Line;
        scope.expectedCount = std::uint32_t(uses);
        output.intent = scope;
        output.plane.outwardNormal = {normal.X(), normal.Y(), normal.Z()};
        output.plane.offsetMM = normal.XYZ().Dot(plane.Location().XYZ())
            * 1000.0 * metersPerLocalUnit;
        output.coverage = fs::Coverage::EntireBoundary;
        output.wireCount = std::uint32_t(wireMap.Extent());
        output.boundaryUseCount = std::uint32_t(uses);
        output.boundaryUniqueEdgeCount = std::uint32_t(edgeMap.Extent());
        return FaceImageDeriveStatus::Derived;
    } catch (...) { output = FaceImageGeometricReceipt{}; return FaceImageDeriveStatus::NotAddressable; }
}

//! Current B2 resolution of one captured intent on a stage: the full
//! membership proof, never nearest-face geometry matching.
inline retained_face_selector::Refusal ResolveFaceImageReceipt(
    const TopoDS_Shape& stage, const retained_face_selector::SelectorIntent& intent,
    double metersPerLocalUnit, retained_edge_treatment::ReplayBudget& budget,
    FaceImageGeometricReceipt& output, TopoDS_Face& matched) noexcept {
    namespace fs = retained_face_selector;
    output = FaceImageGeometricReceipt{}; matched = TopoDS_Face();
    try {
        static const std::atomic_bool cancelled{false};
        fs::Resolution resolution;
        const fs::Refusal refusal = fs::Resolve(stage, intent, metersPerLocalUnit,
            budget, cancelled, resolution);
        if (refusal != fs::Refusal::None || !resolution.proof) return refusal;
        output.intent = resolution.proof->intent();
        output.plane = resolution.proof->plane();
        output.coverage = resolution.proof->coverage();
        output.wireCount = resolution.proof->wireCount();
        output.boundaryUseCount = std::uint32_t(resolution.proof->boundaryUses().size());
        output.boundaryUniqueEdgeCount = resolution.proof->uniqueEdgeCount();
        matched = resolution.proof->face();
        return fs::Refusal::None;
    } catch (...) { return fs::Refusal::NativeFailure; }
}

inline FaceImageReplayStatus MapFaceImageResolveRefusal(
    retained_face_selector::Refusal refusal) noexcept {
    namespace fs = retained_face_selector;
    switch (refusal) {
        case fs::Refusal::None: return FaceImageReplayStatus::Captured;
        case fs::Refusal::FaceAmbiguous: case fs::Refusal::SplitEdge:
            return FaceImageReplayStatus::AmbiguousFaceRemap;
        case fs::Refusal::Budget: case fs::Refusal::TruncatedDiscovery:
        case fs::Refusal::Cancelled:
            return FaceImageReplayStatus::Budget;
        case fs::Refusal::InvalidIntent: return FaceImageReplayStatus::Malformed;
        default: return FaceImageReplayStatus::StaleFace;
    }
}

inline FaceImageReplayStatus MapFaceImageOwnerOutcome(
    face_image::owner::Outcome outcome) noexcept {
    using Outcome = face_image::owner::Outcome;
    switch (outcome) {
        case Outcome::Prepared: case Outcome::Committed: return FaceImageReplayStatus::Prepared;
        case Outcome::StaleSource: return FaceImageReplayStatus::StaleSource;
        case Outcome::StaleFace: return FaceImageReplayStatus::StaleFace;
        case Outcome::AmbiguousFaceRemap: return FaceImageReplayStatus::AmbiguousFaceRemap;
        case Outcome::UnsupportedSurface: return FaceImageReplayStatus::UnsupportedSurface;
        case Outcome::MissingResource: return FaceImageReplayStatus::MissingResource;
        case Outcome::UnsupportedDownstream: return FaceImageReplayStatus::UnsupportedDownstream;
        case Outcome::OwnerMismatch: return FaceImageReplayStatus::OwnerMismatch;
        case Outcome::Busy: return FaceImageReplayStatus::Busy;
        case Outcome::Malformed: return FaceImageReplayStatus::Malformed;
        default: return FaceImageReplayStatus::PersistenceFailure;
    }
}
} // namespace detail

//! Canonical E3 face proof: SHA-256 over "E3FP\1", the canonical B2 intent,
//! the plane witness, coverage and the wire/use/unique-edge census. Anchor
//! keys are excluded by construction (see the section comment above).
inline bool FaceImageReceiptProof(const FaceImageGeometricReceipt& receipt,
    face_image::Digest& output) noexcept {
    output = {};
    try {
        detail::FaceImageProofWriter writer;
        writer.raw(reinterpret_cast<const std::uint8_t*>("E3FP\1"), 5);
        if (!retained_face_selector::WriteIntent(writer, receipt.intent)) return false;
        for (double component : receipt.plane.outwardNormal) writer.d(component);
        writer.d(receipt.plane.offsetMM);
        writer.u(std::uint8_t(receipt.coverage), 1);
        writer.u(receipt.wireCount, 4);
        writer.u(receipt.boundaryUseCount, 4);
        writer.u(receipt.boundaryUniqueEdgeCount, 4);
        if (!writer.ok) return false;
        return face_image::HashFaceImageBytes(writer.bytes, output)
            && face_image::Nonzero(output);
    } catch (...) { output = {}; return false; }
}

//! Recovered current state of one bound face: its durable identity, the
//! current geometric receipt and its reproducible proof.
struct FaceImageFaceState final {
    face_image::UUID face{};
    face_image::Digest proof{};
    FaceImageGeometricReceipt receipt;
};

//! Captured authority for every committed face-image binding of one owner:
//! the exact committed record and bytes, one recovered current face state per
//! binding (aligned with Definition::bindings order) and the Prepare-time
//! currentness fences. Capture never mutates the document.
struct FaceImageAttachment final {
    face_image::OwnerKey owner{};
    face_image::Definition committed;
    std::vector<std::uint8_t> canonicalBytes;
    std::vector<FaceImageFaceState> faces;
    face_image::Observed observed;
};

//! Fail-closed capture of one owner's committed face-image attachments.
//! Face intents are recovered by exact proof correspondence: every planar,
//! scope-addressable face of the current shape is re-resolved through the
//! real B2 resolver under one shared bounded budget, and a stored proof must
//! match exactly one current face, with a one-to-one correspondence across
//! all bindings. Zero matches is StaleFace; more than one is
//! AmbiguousFaceRemap; a malformed record is Malformed; no record is Absent.
inline FaceImageReplayStatus CaptureFaceImageAttachment(OcctDocument& owner,
    const face_image::OwnerKey& key, FaceImageAttachment& output) noexcept {
    namespace fs = retained_face_selector;
    output = FaceImageAttachment{};
    try {
        face_image::Definition definition;
        std::vector<std::uint8_t> bytes;
        const auto state = owner.ReadFaceImageBindings(key, definition, &bytes);
        if (state == face_image::persistence::bindings::ReadState::Absent)
            return FaceImageReplayStatus::Absent;
        if (state != face_image::persistence::bindings::ReadState::Present)
            return FaceImageReplayStatus::Malformed;
        double metersPerLocalUnit = 0;
        if (!detail::FaceImageLengthUnit(owner, metersPerLocalUnit))
            return FaceImageReplayStatus::Malformed;
        TDF_Label ownerLabel;
        const TopoDS_Shape shape = detail::FaceImageOwnerShape(owner, key, ownerLabel);
        if (shape.IsNull()) return FaceImageReplayStatus::OwnerMismatch;
        TopTools_IndexedMapOfShape faceMap;
        TopExp::MapShapes(shape, TopAbs_FACE, faceMap);
        if (faceMap.Extent() > 4096) return FaceImageReplayStatus::Budget;
        Bnd_Box stageBox;
        BRepBndLib::Add(shape, stageBox);
        if (stageBox.IsVoid()) return FaceImageReplayStatus::Malformed;

        retained_edge_treatment::ReplayBudget budget;
        struct Candidate { TopoDS_Face face; FaceImageGeometricReceipt receipt; face_image::Digest proof; };
        std::vector<Candidate> candidates;
        for (int index = 1; index <= faceMap.Extent(); ++index) {
            const TopoDS_Face face = TopoDS::Face(faceMap.FindKey(index));
            FaceImageGeometricReceipt derived;
            const auto derived_status = detail::DeriveFaceImageReceipt(
                stageBox, face, metersPerLocalUnit, budget, derived);
            if (derived_status == detail::FaceImageDeriveStatus::Budget)
                return FaceImageReplayStatus::Budget;
            if (derived_status != detail::FaceImageDeriveStatus::Derived) continue;
            face_image::Digest proof{};
            if (!FaceImageReceiptProof(derived, proof)) return FaceImageReplayStatus::Malformed;
            candidates.push_back({face, derived, proof});
        }
        output.owner = key;
        output.committed = definition;
        output.canonicalBytes = std::move(bytes);
        output.faces.resize(definition.bindings.size());
        std::vector<bool> claimed(candidates.size(), false);
        for (std::size_t binding = 0; binding < definition.bindings.size(); ++binding) {
            const auto& stored = definition.bindings[binding];
            int matched = -1; unsigned matches = 0;
            for (std::size_t index = 0; index < candidates.size(); ++index) {
                if (candidates[index].proof == stored.selectorProof) {
                    ++matches;
                    if (matched < 0) matched = int(index);
                }
            }
            if (matches == 0) { output = FaceImageAttachment{}; return FaceImageReplayStatus::StaleFace; }
            if (matches > 1) { output = FaceImageAttachment{}; return FaceImageReplayStatus::AmbiguousFaceRemap; }
            if (claimed[std::size_t(matched)]) {
                output = FaceImageAttachment{}; return FaceImageReplayStatus::AmbiguousFaceRemap;
            }
            claimed[std::size_t(matched)] = true;
            // Re-prove the matched face through the full B2 membership census;
            // the derived receipt and the resolver receipt must agree exactly.
            FaceImageGeometricReceipt resolved; TopoDS_Face resolvedFace;
            const fs::Refusal refusal = detail::ResolveFaceImageReceipt(shape,
                candidates[std::size_t(matched)].receipt.intent, metersPerLocalUnit,
                budget, resolved, resolvedFace);
            if (refusal != fs::Refusal::None) {
                output = FaceImageAttachment{};
                return detail::MapFaceImageResolveRefusal(refusal);
            }
            if (!(resolved == candidates[std::size_t(matched)].receipt)
                || !resolvedFace.IsSame(candidates[std::size_t(matched)].face)) {
                output = FaceImageAttachment{}; return FaceImageReplayStatus::Malformed;
            }
            output.faces[binding] = {stored.face, stored.selectorProof, resolved};
            output.observed.faces.push_back({stored.face, stored.selectorProof});
        }
        if (!owner.FaceImageResourceManifest(output.observed.resources)) {
            output = FaceImageAttachment{}; return FaceImageReplayStatus::Malformed;
        }
        return FaceImageReplayStatus::Captured;
    } catch (...) { output = FaceImageAttachment{}; return FaceImageReplayStatus::Malformed; }
}

//! Detached E3 replay work for one source command. prepare() runs before any
//! mutation against the prospective post-edit stage and proves a positive
//! one-to-one remap of every bound face; stage() runs after the source
//! mutation inside the caller's one open command, re-proves the identical
//! bijection against the actual document shape and commits only the refreshed
//! record; read() verifies the committed bytes. Cancel discards everything.
//! A zero-delta prepare stages nothing and produces no history.
class FaceImageReplay final {
public:
    FaceImageReplay() = default;
    FaceImageReplay(const FaceImageReplay&) = delete;
    FaceImageReplay& operator=(const FaceImageReplay&) = delete;

    FaceImageReplayStatus status() const noexcept { return status_; }
    Mutation mutation() const noexcept { return mutation_; }
    bool prepared() const noexcept { return prepared_; }
    const face_image::Definition& refreshed() const noexcept { return refreshed_; }

    //! Prepare BEFORE any mutation. resultEntity must be the attachment owner
    //! entity for both Replace and Remove; anything else is
    //! UnsupportedDownstream (no Boolean/duplication propagation). For
    //! Replace, prospectiveStage is the detached post-edit owner shape.
    FaceImageReplayStatus prepare(OcctDocument& owner,
        const FaceImageAttachment& attachment, Mutation mutation,
        const UUID& resultEntity, const TopoDS_Shape& prospectiveStage) noexcept {
        namespace fs = retained_face_selector;
        cancel();
        try {
            if (attachment.canonicalBytes.empty() || attachment.committed.bindings.empty()
                || attachment.faces.size() != attachment.committed.bindings.size()) {
                // No committed attachments: the replay is a proven no-op.
                prepared_ = true;
                status_ = FaceImageReplayStatus::NoChange;
                return status_;
            }
            if (!(resultEntity == attachment.owner.entity)) {
                status_ = FaceImageReplayStatus::UnsupportedDownstream;
                return status_;
            }
            double metersPerLocalUnit = 0;
            if (!detail::FaceImageLengthUnit(owner, metersPerLocalUnit)) {
                status_ = FaceImageReplayStatus::Malformed;
                return status_;
            }
            mutation_ = mutation;
            if (mutation == Mutation::Remove) {
                attachment_ = attachment;
                prepared_ = true;
                status_ = FaceImageReplayStatus::Deleted;
                return status_;
            }
            if (prospectiveStage.IsNull()) {
                status_ = FaceImageReplayStatus::Malformed;
                return status_;
            }
            retained_edge_treatment::ReplayBudget budget;
            face_image::Definition refreshed = attachment.committed;
            std::vector<TopoDS_Face> remapped(attachment.committed.bindings.size());
            for (std::size_t binding = 0; binding < attachment.committed.bindings.size(); ++binding) {
                FaceImageGeometricReceipt resolved;
                const fs::Refusal refusal = detail::ResolveFaceImageReceipt(prospectiveStage,
                    attachment.faces[binding].receipt.intent, metersPerLocalUnit,
                    budget, resolved, remapped[binding]);
                if (refusal != fs::Refusal::None) {
                    status_ = detail::MapFaceImageResolveRefusal(refusal);
                    return status_;
                }
                for (std::size_t other = 0; other < binding; ++other)
                    if (remapped[other].IsSame(remapped[binding])) {
                        status_ = FaceImageReplayStatus::AmbiguousFaceRemap;
                        return status_;
                    }
                face_image::Digest proof{};
                if (!FaceImageReceiptProof(resolved, proof)) {
                    status_ = FaceImageReplayStatus::Malformed;
                    return status_;
                }
                refreshed.bindings[binding].selectorProof = proof;
            }
            if (!face_image::BindBindingProof(refreshed)) {
                status_ = FaceImageReplayStatus::Malformed;
                return status_;
            }
            attachment_ = attachment;
            refreshed_ = std::move(refreshed);
            prepared_ = true;
            status_ = refreshed_ == attachment_.committed
                ? FaceImageReplayStatus::NoChange : FaceImageReplayStatus::Prepared;
            return status_;
        } catch (...) { cancel(); status_ = FaceImageReplayStatus::Malformed; return status_; }
    }

    //! Stage AFTER the source mutation, inside the caller's one open command.
    //! The current document record must still be the captured one byte-for-byte
    //! and the actual owner shape must reproduce the prepared bijection;
    //! otherwise the whole command aborts without a delta.
    FaceImageReplayStatus stage(OcctDocument& owner) noexcept {
        namespace fs = retained_face_selector;
        if (!prepared_) { status_ = FaceImageReplayStatus::Malformed; return status_; }
        try {
            if (attachment_.canonicalBytes.empty()) {
                status_ = FaceImageReplayStatus::NoChange;
                return status_;
            }
            face_image::Definition current;
            std::vector<std::uint8_t> bytes;
            const auto state = owner.ReadFaceImageBindings(attachment_.owner, current, &bytes);
            if (state != face_image::persistence::bindings::ReadState::Present
                || bytes != attachment_.canonicalBytes || !(current == attachment_.committed)) {
                status_ = FaceImageReplayStatus::StaleSource;
                return status_;
            }
            if (mutation_ == Mutation::Remove) {
                status_ = detail::MapFaceImageOwnerOutcome(
                    owner.RemoveFaceImageBindings(attachment_.owner));
                if (status_ == FaceImageReplayStatus::Prepared)
                    status_ = FaceImageReplayStatus::Deleted;
                return status_;
            }
            if (status_ == FaceImageReplayStatus::NoChange) return status_;
            double metersPerLocalUnit = 0;
            if (!detail::FaceImageLengthUnit(owner, metersPerLocalUnit)) {
                status_ = FaceImageReplayStatus::Malformed;
                return status_;
            }
            TDF_Label ownerLabel;
            const TopoDS_Shape stagedShape = detail::FaceImageOwnerShape(
                owner, attachment_.owner, ownerLabel);
            if (stagedShape.IsNull()) {
                status_ = FaceImageReplayStatus::StaleSource;
                return status_;
            }
            retained_edge_treatment::ReplayBudget budget;
            face_image::Observed observed;
            for (std::size_t binding = 0; binding < refreshed_.bindings.size(); ++binding) {
                FaceImageGeometricReceipt resolved; TopoDS_Face matched;
                const fs::Refusal refusal = detail::ResolveFaceImageReceipt(stagedShape,
                    attachment_.faces[binding].receipt.intent, metersPerLocalUnit,
                    budget, resolved, matched);
                if (refusal != fs::Refusal::None) {
                    status_ = detail::MapFaceImageResolveRefusal(refusal);
                    return status_;
                }
                face_image::Digest proof{};
                if (!FaceImageReceiptProof(resolved, proof)
                    || !(proof == refreshed_.bindings[binding].selectorProof)) {
                    // The staged geometry is not the accepted candidate.
                    status_ = FaceImageReplayStatus::StaleSource;
                    return status_;
                }
                observed.faces.push_back({refreshed_.bindings[binding].face, proof});
            }
            std::vector<face_image::ResourceFence> manifest;
            if (!owner.FaceImageResourceManifest(manifest)) {
                status_ = FaceImageReplayStatus::Malformed;
                return status_;
            }
            for (const auto& binding : refreshed_.bindings) {
                bool found = false;
                for (const auto& fence : manifest)
                    if (fence.resource == binding.resource) {
                        observed.resources.push_back(fence);
                        found = true;
                        break;
                    }
                if (!found) {
                    status_ = FaceImageReplayStatus::MissingResource;
                    return status_;
                }
            }
            staging_ = face_image::owner::Staging{};
            FaceImageReplayStatus outcome = detail::MapFaceImageOwnerOutcome(
                owner.PrepareFaceImageBindings(staging_, refreshed_, observed));
            if (outcome != FaceImageReplayStatus::Prepared) {
                owner.CancelFaceImageBindings(staging_);
                status_ = outcome;
                return status_;
            }
            outcome = detail::MapFaceImageOwnerOutcome(
                owner.CommitFaceImageBindings(staging_, observed));
            if (outcome != FaceImageReplayStatus::Prepared)
                owner.CancelFaceImageBindings(staging_);
            status_ = outcome;
            return status_;
        } catch (...) {
            owner.CancelFaceImageBindings(staging_);
            status_ = FaceImageReplayStatus::Malformed;
            return status_;
        }
    }

    //! Post-commit readback: the actual record must be exactly the refreshed
    //! candidate (Replace), the unchanged committed record (zero-delta) or
    //! absent (Remove).
    bool read(OcctDocument& owner) const noexcept {
        if (!prepared_) return false;
        try {
            if (attachment_.canonicalBytes.empty()) return true;
            face_image::Definition current;
            std::vector<std::uint8_t> bytes;
            const auto state = owner.ReadFaceImageBindings(attachment_.owner, current, &bytes);
            if (mutation_ == Mutation::Remove)
                return state == face_image::persistence::bindings::ReadState::Absent;
            if (state != face_image::persistence::bindings::ReadState::Present) return false;
            return status_ == FaceImageReplayStatus::NoChange
                ? current == attachment_.committed && bytes == attachment_.canonicalBytes
                : current == refreshed_;
        } catch (...) { return false; }
    }

    void cancel() noexcept {
        mutation_ = Mutation::Replace;
        prepared_ = false;
        status_ = FaceImageReplayStatus::Absent;
        attachment_ = FaceImageAttachment{};
        refreshed_ = face_image::Definition{};
        staging_ = face_image::owner::Staging{};
    }

private:
    FaceImageReplayStatus status_ = FaceImageReplayStatus::Absent;
    Mutation mutation_ = Mutation::Replace;
    bool prepared_ = false;
    FaceImageAttachment attachment_;
    face_image::Definition refreshed_;
    face_image::owner::Staging staging_;
};

} // namespace core3d::dependent_replay
