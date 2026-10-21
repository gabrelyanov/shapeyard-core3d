#pragma once

// B4-S2 retained native owner for Draft Faces (SYDF/1, feature kind
// 0x00002004). One issued command per create/edit; real retained-source
// capture with a byte-exact recipe receipt; dependency replay through the
// closed, call-bound owner registry in DraftFacesRegistry.hxx; native
// Undo/Redo via the single OCAF command; known failures abort and prove the
// exact prior; unknown close/publication retains the issuer, candidate and
// recovery fence and settles exactly once through reconcile. The public
// DraftRegistryEntry stays disabled; the Objects route is S4. AI stays
// unavailable.
#include "DraftFacesRegistry.hxx"
#include "RetainedFeatureOwner.hxx"
#include "CompositeRecipeAttribute.hxx"
#include "RetainedSolidAttribute.hxx"
#include "ReceiptRecord.hxx"
#include "NativeOpeningContext.hxx"
#include "PartBooleanBuild.hxx"
#include "ProfilePersistence.hxx"
#include "RetainedProfileProducer.hxx"
#include "OcctDocument.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <GProp_GProps.hxx>
#include <Standard_GUID.hxx>
#include <TDF_Label.hxx>
#include <TDocStd_Document.hxx>
#include <TNaming_Builder.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <pthread.h>
#include <string>
#include <utility>
#include <vector>

namespace core3d::draft_faces::owner {

namespace cr = composite_recipe;
namespace rf = retained_feature;

enum class Outcome : std::uint8_t {
    Rejected, Cancelled, Unchanged, Committed, RecoveryRequired,
};

struct Receipt final {
    Outcome outcome = Outcome::Rejected;
    std::string reason = "b4.unissued";
    std::uint64_t session = 0;
    int measuredHistoryDelta = 0;
    bool settled = true;
};

//! DEBUG fault injection arms. None is the only production value; every other
//! arm is raised by a test probe and exercises the real abort/recovery code.
enum class FaultPoint : std::uint8_t {
    None, FailPrepare, FailStage, FailReadback, UnprovenClose, StopBeforeLease,
};

namespace detail {
inline retained_recipe::UUID MintUUID() noexcept {
    retained_recipe::UUID value{};
    arc4random_buf(value.data(), value.size());
    value[6] = std::uint8_t((value[6] & 0x0f) | 0x40);
    value[8] = std::uint8_t((value[8] & 0x3f) | 0x80);
    return value;
}

//! B4 commitment domains. Geometry and recipe commitments are the exact
//! SHA-256 payload digests the validators require; placement/material/groups
//! commit to the actual persisted field bytes under a B4 domain prefix.
inline bool DomainHash(const char* prefix, const std::vector<std::uint8_t>& body,
                       retained_recipe::Digest& digest) noexcept {
    std::vector<std::uint8_t> joined(prefix, prefix + std::strlen(prefix));
    joined.insert(joined.end(), body.begin(), body.end());
    return cr::Hash(joined, digest);
}

inline bool PlacementCommitment(const cr::InputPlacement& placement,
                                retained_recipe::Digest& digest) noexcept {
    std::vector<std::uint8_t> body;
    const auto append = [&body](double value) {
        std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits));
        for (unsigned index = 0; index < 8; ++index)
            body.push_back(std::uint8_t(bits >> (8 * index)));
    };
    append(placement.sourceMetersPerUnit);
    append(placement.carrierMetersPerUnit);
    for (double scalar : placement.matrix) append(scalar);
    return DomainHash("B4D1-placement/1", body, digest);
}

inline bool SourceCommitments(const cr::InputPlacement& placement,
                              const std::vector<std::uint8_t>& sourceBytes,
                              const std::vector<std::uint8_t>& recipeBytes,
                              cr::Commitments& commitments) noexcept {
    return cr::Hash(sourceBytes, commitments.geometry)
        && cr::Hash(recipeBytes, commitments.recipe)
        && PlacementCommitment(placement, commitments.placement)
        && DomainHash("B4D1-material/1", {}, commitments.material)
        && DomainHash("B4D1-groups/1", {}, commitments.groups);
}

//! Reads the persisted owner key from the committed identity attributes, never
//! from caller text.
inline bool ReadOwnerKey(const OcctDocument& document,
                         const TDF_Label& carrier,
                         retained_recipe::OwnerKey& key) noexcept {
    return !document.Document().IsNull() && !carrier.IsNull()
        && receipt::ParseUUID(document.DocumentIdentifier(), key.document)
        && receipt::ParseUUID(document.EntityIdentifierForLabel(carrier), key.entity)
        && receipt::ParseUUID(document.DefinitionIdentifierForLabel(carrier), key.definition);
}

//! Canonical source bytes are an exact write/read/write round trip on both
//! sides. This removes OCCT's volatile per-TShape cache flags and omitted mesh
//! state while keeping geometry, topology, tolerances and ranges byte-exact.
inline bool CanonicalSourceBytes(const std::vector<std::uint8_t>& exactBytes,
                                 std::vector<std::uint8_t>& canonical) noexcept {
    canonical.clear();
    try {
        TopoDS_Shape roundTripped;
        if (exactBytes.empty()
            || !draft_faces::detail::readShape(exactBytes, roundTripped))
            return false;
        // OCCT 7.8 preserves Checked while reading V3 BRep bytes. Normalize
        // that derived cache state on the private round-tripped graph by
        // marking every TShape modified; OCCT's own Modified(true) operation
        // clears Checked. No serialized field is skipped and no geometry,
        // topology, tolerance or range is rewritten.
        std::vector<std::pair<TopoDS_Shape, unsigned>> pending{{roundTripped, 0}};
        std::size_t visited = 0;
        while (!pending.empty()) {
            auto current = std::move(pending.back()); pending.pop_back();
            if (++visited > 8192 || current.second > 64) return false;
            current.first.Modified(Standard_True);
            for (TopoDS_Iterator child(current.first); child.More(); child.Next()) {
                if (pending.size() >= 8192) return false;
                pending.emplace_back(child.Value(), current.second + 1);
            }
        }
        return draft_faces::detail::exactShapeBytes(roundTripped, canonical);
    } catch (...) {
        canonical.clear();
        return false;
    }
}

//! Owner-local mirror of the production createProfile polygon-prism producer
//! in Core3DViewer.mm:5619-5729 and :5761-5793. Keep this recipe path byte-
//! exact with production; callers must never substitute geometric tolerance.
inline TopoDS_Shape BuildProductionProfileSource(
    const profile::Parameters& parameters) noexcept {
    try {
        if (!part_boolean::build::IsRectangularP1BaseRecipe(parameters, false))
            return {};
        const ProfileDefinition& definition = parameters.definition;
        double signedArea = 0, expected = 0;
        if (!ProfileDefinitionExpectedVolume(definition, signedArea, expected)) return {};

        auto points = definition.points;
        if (signedArea < 0) std::reverse(points.begin(), points.end());
        BRepBuilderAPI_MakePolygon polygon;
        for (const gp_Pnt2d& point : points)
            polygon.Add(ProfilePointInPlane(point, definition.plane));
        polygon.Close();
        if (!polygon.IsDone()) return {};
        BRepBuilderAPI_MakeFace face(polygon.Wire(), Standard_True);
        if (!face.IsDone()
            || !BRepCheck_Analyzer(face.Face(), Standard_True).IsValid()) return {};

        const gp_Vec direction = definition.plane == 0
            ? gp_Vec(0, 0, definition.depth)
            : definition.plane == 1
                ? gp_Vec(0, definition.depth, 0)
                : gp_Vec(definition.depth, 0, 0);
        BRepPrimAPI_MakePrism prism(
            face.Face(), direction, Standard_True, Standard_True);
        if (!prism.IsDone() || prism.Shape().IsNull()
            || prism.Shape().ShapeType() != TopAbs_SOLID) return {};
        TopoDS_Shape result = prism.Shape();

        const std::atomic_bool cancelled{false};
        if (parameters.constructionFrame) {
            expected *= parameters.constructionFrame->AbsoluteVolumeScale();
            if (!std::isfinite(expected) || expected <= 0
                || !retained_edge_treatment::BuildFramedPolygonPrism(
                    parameters, cancelled, result)) return {};
        }
        if (result.IsNull() || result.ShapeType() != TopAbs_SOLID) return {};
        TopoDS_Solid solid = TopoDS::Solid(result);
        if (!BRepLib::OrientClosedSolid(solid)
            || !BRepCheck_Analyzer(solid, Standard_True).IsValid()) return {};
        GProp_GProps properties;
        BRepGProp::VolumeProperties(solid, properties);
        if (!std::isfinite(properties.Mass()) || properties.Mass() <= 0
            || std::abs(properties.Mass() - expected)
                > std::max(1e-8, expected * 1e-8)) return {};

        if (retained_profile_producer::EligiblePrivateProfileSource(
                definition.revolve, false, bool(definition.circle),
                bool(definition.curves), bool(parameters.constructionFrame))) {
            retained_edge_treatment::ReplayBudget producerBudget;
            TopoDS_Shape fixedPoint;
            const auto status = retained_profile_producer::
                MaterializePrivateProfileProducerFixedPoint(
                    solid, cancelled, producerBudget, fixedPoint);
            if (status == retained_profile_producer::FixedPointStatus::Cancelled
                || status == retained_profile_producer::
                    FixedPointStatus::BudgetExceeded) return {};
            if (status == retained_profile_producer::FixedPointStatus::Adopted)
                solid = TopoDS::Solid(fixedPoint);
        }
        return solid;
    } catch (...) {
        return {};
    }
}

//! Rebuilds the declared rectangular-profile source through the production
//! profile source builder and requires byte-exact equality of the canonical
//! forms with the actual retained source. This is the create-time receipt: a
//! recipe that cannot regenerate the committed source shape exactly is never
//! adopted.
inline bool ProveSourceReceipt(const profile::Parameters& parameters,
                               const std::vector<std::uint8_t>& sourceBytes,
                               composite_recipe::SourceRecipe& recipe) noexcept {
    recipe = {};
    std::vector<double> values;
    if (sourceBytes.empty()
        || !part_boolean::build::IsRectangularP1BaseRecipe(parameters, false)
        || !profile::Encode(parameters, values)
        || !cr::EncodeScalarRecipe(cr::RecipeKind::Profile,
                                   std::uint32_t(profile::SchemaFor(parameters)),
                                   values, recipe.bytes))
        return false;
    recipe.kind = cr::RecipeKind::Profile;
    recipe.schema = std::uint32_t(profile::SchemaFor(parameters));
    const TopoDS_Shape rebuilt = BuildProductionProfileSource(parameters);
    std::vector<std::uint8_t> rebuiltBytes, canonicalRebuilt, canonicalSource;
    return !rebuilt.IsNull()
        && draft_faces::detail::exactShapeBytes(rebuilt, rebuiltBytes)
        && CanonicalSourceBytes(rebuiltBytes, canonicalRebuilt)
        && CanonicalSourceBytes(sourceBytes, canonicalSource)
        && canonicalRebuilt == canonicalSource;
}

//! One shared abort/recovery tail for every owner verb. On a known failure
//! the open command is aborted and the exact prior must be proven; when even
//! that proof is impossible the issuer keeps the candidate behind the armed
//! recovery fence instead of reporting a silent partial state.
struct RecoverySnapshot final {
    bool create = false;
    bool baselineProfilePresent = false;
    std::string entityIdentifier;
    std::string baselineProfileIdentifier;
    TDF_Label carrier;
    std::shared_ptr<native_opening::Context> context;
    std::vector<double> baselineProfileValues;
    std::vector<std::uint8_t> baselineGraphBytes, baselineShapeBytes;
    std::vector<std::uint8_t> expectedGraphBytes, expectedShapeBytes;
};
} // namespace detail

//! Immutable authority minted by MintCreateCapture/MintEditCapture. Callers
//! cannot construct it: the fence, labels, identities and receipts are read
//! from the live document at mint time.
class Capture final {
    friend class NativeOwner;
    friend struct MintAccess;
    Handle(TDocStd_Document) document;
    std::shared_ptr<native_opening::Context> context;
    std::uint32_t viewportWidth = 0, viewportHeight = 0;
    TDF_Label carrier;
    std::string entityIdentifier;
    retained_recipe::OwnerKey ownerKey{};
    double metersPerUnit = 0;
    Standard_Integer undoDepth = 0, redoDepth = 0;
    bool forEdit = false;
    // Create-only: declared retained source proven against the actual shape.
    profile::Parameters sourceProfile;
    cr::SourceRecipe sourceRecipe{};
    std::vector<std::uint8_t> sourceBytes;
    // Edit-only: the exact committed record read back at mint time.
    cr::Record record;
    std::size_t sourceIndex = 0, featureIndex = 0;
    Capture() = default;
};

class NativeOwner final {
public:
    explicit NativeOwner(OcctDocument& owner) noexcept : owner_(owner) {
        if (!owner.Document().IsNull()) document_ = owner.Document();
    }
    NativeOwner(const NativeOwner&) = delete;
    NativeOwner& operator=(const NativeOwner&) = delete;

    bool blocksOtherWork() const noexcept { return recoverySession_ != 0; }

    //! Create-capture authority for one ordinary free solid about to be
    //! promoted into the retained Draft Faces carrier. The declared profile
    //! recipe must regenerate the actual source shape byte-exactly.
    static std::shared_ptr<const Capture> MintCreateCapture(
        OcctDocument& owner, const std::shared_ptr<native_opening::Context>& context,
        const TDF_Label& carrier, const std::string& entityIdentifier,
        const profile::Parameters& sourceProfile,
        std::uint32_t viewportWidth, std::uint32_t viewportHeight) noexcept {
        try {
            if (pthread_main_np() == 0 || !context || carrier.IsNull()
                || entityIdentifier.empty()) return nullptr;
            const auto native = owner.Document();
            if (native.IsNull() || native->GetData().IsNull() || native->HasOpenCommand()
                || context->openingFence().document() != native
                || context->openingFence().data() != native->GetData()
                || !context->isCurrent(viewportWidth, viewportHeight)
                || !XCAFDoc_ShapeTool::IsFree(carrier)
                || !XCAFDoc_ShapeTool::IsSimpleShape(carrier)
                || carrier.Data() != native->GetData()
                || owner.EntityIdentifierForLabel(carrier) != entityIdentifier
                || cr::HasRecord(carrier)) return nullptr;
            const double unit = context->openingFence().metersPerUnit();
            if (!std::isfinite(unit) || unit <= 0 || unit > 1
                || !std::isfinite(sourceProfile.metersPerUnit)
                || sourceProfile.metersPerUnit != unit) return nullptr;
            const TopoDS_Shape source = XCAFDoc_ShapeTool::GetShape(carrier);
            std::vector<std::uint8_t> sourceBytes;
            if (source.IsNull() || source.ShapeType() != TopAbs_SOLID
                || source.Orientation() != TopAbs_FORWARD
                || !draft_faces::detail::exactShapeBytes(source, sourceBytes))
                return nullptr;
            auto capture = std::shared_ptr<Capture>(new Capture());
            if (!detail::ProveSourceReceipt(sourceProfile, sourceBytes,
                                            capture->sourceRecipe)
                || !detail::ReadOwnerKey(owner, carrier, capture->ownerKey))
                return nullptr;
            capture->document = native; capture->context = context;
            capture->viewportWidth = viewportWidth; capture->viewportHeight = viewportHeight;
            capture->carrier = carrier; capture->entityIdentifier = entityIdentifier;
            capture->metersPerUnit = unit;
            capture->undoDepth = native->GetAvailableUndos();
            capture->redoDepth = native->GetAvailableRedos();
            capture->forEdit = false;
            capture->sourceProfile = sourceProfile;
            capture->sourceBytes = std::move(sourceBytes);
            return capture;
        } catch (...) { return nullptr; }
    }

    //! Edit-capture authority re-derived from the committed composite record
    //! (including after a cold ordinary reopen). The minimal S2 grammar is
    //! exactly one source node and one Draft Faces feature node whose single
    //! input is that source and whose node is the graph output.
    static std::shared_ptr<const Capture> MintEditCapture(
        OcctDocument& owner, const std::shared_ptr<native_opening::Context>& context,
        const TDF_Label& carrier, const std::string& entityIdentifier,
        std::uint32_t viewportWidth, std::uint32_t viewportHeight) noexcept {
        try {
            if (pthread_main_np() == 0 || !context || carrier.IsNull()
                || entityIdentifier.empty()) return nullptr;
            const auto native = owner.Document();
            if (native.IsNull() || native->GetData().IsNull() || native->HasOpenCommand()
                || context->openingFence().document() != native
                || context->openingFence().data() != native->GetData()
                || !context->isCurrent(viewportWidth, viewportHeight)
                || carrier.Data() != native->GetData()
                || owner.EntityIdentifierForLabel(carrier) != entityIdentifier)
                return nullptr;
            cr::Record record;
            if (!cr::Read(native, carrier, record) || !record.value
                || record.value->definition.schemaVersion != 3
                || record.value->definition.nodes.size() != 2) return nullptr;
            const auto* source = std::get_if<cr::SourceNode>(
                &record.value->definition.nodes[0].value);
            const auto* feature = std::get_if<cr::FeatureNode>(
                &record.value->definition.nodes[1].value);
            if (!source || !feature || feature->kind != FeatureKind
                || feature->codecVersion != CodecVersion || feature->inputs.size() != 1
                || feature->inputs[0] != source->node
                || record.value->definition.outputNode != feature->node
                || source->shapeSlot >= record.value->sourceShapes.size()) return nullptr;
            Definition parameters;
            if (!Decode(feature->parameters, parameters)) return nullptr;
            const TopoDS_Shape& sourceShape = record.value->sourceShapes[source->shapeSlot];
            std::vector<std::uint8_t> sourceBytes;
            if (sourceShape.IsNull()
                || !draft_faces::detail::exactShapeBytes(sourceShape, sourceBytes))
                return nullptr;
            const double unit = context->openingFence().metersPerUnit();
            if (!std::isfinite(unit) || unit <= 0 || unit > 1
                || parameters.metersPerLocalUnit != unit) return nullptr;
            auto capture = std::shared_ptr<Capture>(new Capture());
            if (!detail::ReadOwnerKey(owner, carrier, capture->ownerKey)
                || !(capture->ownerKey == record.value->definition.owner)) return nullptr;
            capture->document = native; capture->context = context;
            capture->viewportWidth = viewportWidth; capture->viewportHeight = viewportHeight;
            capture->carrier = carrier; capture->entityIdentifier = entityIdentifier;
            capture->metersPerUnit = unit;
            capture->undoDepth = native->GetAvailableUndos();
            capture->redoDepth = native->GetAvailableRedos();
            capture->forEdit = true;
            capture->record = record;
            capture->sourceIndex = 0; capture->featureIndex = 1;
            capture->sourceBytes = std::move(sourceBytes);
            return capture;
        } catch (...) { return nullptr; }
    }

    //! Promotes the captured ordinary solid into the retained Draft Faces
    //! carrier under one issued command. The candidate bytes must be the
    //! detached candidate issued for this exact opening; the owner rebuilds
    //! the graph through the call-bound registry and requires byte equality.
    Receipt create(const std::shared_ptr<const Capture>& capture,
                   const std::vector<std::uint8_t>& parameters,
                   const std::vector<std::uint8_t>& candidateBytes,
                   const std::atomic_bool& stop,
                   FaultPoint fault = FaultPoint::None) noexcept {
        Receipt receipt;
        try {
            if (!capture || capture->forEdit) return refuse("b4.foreign-capture");
            if (!current(*capture, receipt)) return receipt;
            Definition definition;
            if (parameters.empty() || !Decode(parameters, definition)
                || candidateBytes.empty()
                || definition.metersPerLocalUnit != capture->metersPerUnit)
                return refuse("b4.malformed-candidate");
            cr::Definition graph = cr::MakeV3Definition();
            graph.owner = capture->ownerKey;
            cr::SourceNode source; source.node = detail::MintUUID(); source.localID = 1;
            source.original.document = capture->ownerKey.document;
            source.original.entity = capture->ownerKey.entity;
            source.original.definition = capture->ownerKey.definition;
            // Production promotion retires the standalone profile attribute
            // because the composite becomes the carrier's sole recipe owner.
            // Preserve its feature identity as the composite source lineage;
            // the lower-level owner fixture without a stored profile retains
            // its historical minted identity path.
            source.original.sourceFeature = detail::MintUUID();
            profile::Record sourceProfile;
            if (!profile::Read(capture->document, capture->carrier, sourceProfile))
                return refuse("b4.stale-capture");
            if (!sourceProfile.label.IsNull()
                && (!sourceProfile.IsCurrent(capture->document, capture->carrier)
                    || !receipt::ParseUUID(sourceProfile.identifier,
                                           source.original.sourceFeature)))
                return refuse("b4.stale-capture");
            source.recipe = capture->sourceRecipe;
            source.inputToCarrier.sourceMetersPerUnit = capture->metersPerUnit;
            source.inputToCarrier.carrierMetersPerUnit = capture->metersPerUnit;
            source.shapeSlot = 0;
            if (!detail::SourceCommitments(source.inputToCarrier, capture->sourceBytes,
                                           source.recipe.bytes, source.commitments))
                return refuse("b4.commitment-failure");
            cr::FeatureNode feature; feature.node = detail::MintUUID();
            feature.feature = detail::MintUUID(); feature.localID = 2;
            feature.kind = FeatureKind; feature.codecVersion = CodecVersion;
            feature.inputs.push_back(source.node); feature.parameters = parameters;
            graph.outputNode = feature.node;
            graph.issuance.nextLocalID = 3;
            graph.nodes.push_back({std::move(source)});
            graph.nodes.push_back({std::move(feature)});
            std::vector<std::uint8_t> graphBytes, replayOutput;
            if (!replayCandidate(*capture, graph, capture->sourceBytes,
                                 graphBytes, replayOutput, receipt)) return receipt;
            if (replayOutput != candidateBytes) return refuse("b4.candidate-replay-mismatch");
            if (fault == FaultPoint::FailPrepare) return refuse("b4.prepare-fault");
            if (stop.load()) return cancel("b4.cancelled");
            return transact(capture, graph, graphBytes, replayOutput,
                            {}, nullptr, stop, fault);
        } catch (...) { return refuse("b4.create-exception"); }
    }

    //! Replaces the SYDF parameters of the committed feature under one issued
    //! command. Node/feature/local identities, ordered inputs, source recipe
    //! and surviving face intents are preserved byte-exactly; identical
    //! parameters are an Unchanged no-op with no command.
    Receipt editParameters(const std::shared_ptr<const Capture>& capture,
                           const std::vector<std::uint8_t>& newParameters,
                           const std::atomic_bool& stop,
                           FaultPoint fault = FaultPoint::None) noexcept {
        Receipt receipt;
        try {
            if (!capture || !capture->forEdit) return refuse("b4.foreign-capture");
            if (!current(*capture, receipt)) return receipt;
            Definition decoded;
            if (newParameters.empty() || !Decode(newParameters, decoded)
                || decoded.metersPerLocalUnit != capture->metersPerUnit)
                return refuse("b4.malformed-parameters");
            cr::Definition graph = capture->record.value->definition;
            auto& feature = std::get<cr::FeatureNode>(graph.nodes[1].value);
            if (feature.parameters == newParameters)
                return Receipt{Outcome::Unchanged, "b4.no-op", 0, 0, true};
            feature.parameters = newParameters;
            std::vector<std::uint8_t> graphBytes, replayOutput;
            if (!replayCandidate(*capture, graph, capture->sourceBytes,
                                 graphBytes, replayOutput, receipt)) return receipt;
            if (fault == FaultPoint::FailPrepare) return refuse("b4.prepare-fault");
            if (stop.load()) return cancel("b4.cancelled");
            return transact(capture, graph, graphBytes, replayOutput,
                            capture->record.value->bytes,
                            &capture->record.value->sourceShapes, stop, fault);
        } catch (...) { return refuse("b4.edit-exception"); }
    }

    //! Real upstream dimension edit: rebuilds the retained source from edited
    //! profile recipe values through the production profile source builder,
    //! re-resolves the issued faces and replays the nonempty downstream draft
    //! program, all staged under the same one command. A lost or ambiguous
    //! face anchor refuses the whole edit before any command opens, leaving
    //! the exact prior untouched.
    Receipt editSource(const std::shared_ptr<const Capture>& capture,
                       const std::vector<double>& newProfileValues,
                       const std::atomic_bool& stop,
                       FaultPoint fault = FaultPoint::None) noexcept {
        Receipt receipt;
        try {
            if (!capture || !capture->forEdit) return refuse("b4.foreign-capture");
            if (!current(*capture, receipt)) return receipt;
            profile::Parameters updated;
            if (newProfileValues.empty() || !profile::Decode(newProfileValues, updated)
                || !part_boolean::build::IsRectangularP1BaseRecipe(updated, false)
                || updated.metersPerUnit != capture->metersPerUnit)
                return refuse("b4.invalid-source-edit");
            std::vector<double> encoded;
            cr::SourceRecipe recipe{}; recipe.kind = cr::RecipeKind::Profile;
            recipe.schema = std::uint32_t(profile::SchemaFor(updated));
            if (!profile::Encode(updated, encoded) || encoded != newProfileValues
                || !cr::EncodeScalarRecipe(recipe.kind, recipe.schema, encoded, recipe.bytes))
                return refuse("b4.invalid-source-edit");
            const TopoDS_Shape newSource =
                detail::BuildProductionProfileSource(updated);
            std::vector<std::uint8_t> newSourceBytes;
            if (newSource.IsNull()
                || !draft_faces::detail::exactShapeBytes(newSource, newSourceBytes))
                return refuse("b4.invalid-source-edit");
            cr::Definition graph = capture->record.value->definition;
            auto& source = std::get<cr::SourceNode>(graph.nodes[0].value);
            const auto& feature = std::get<cr::FeatureNode>(graph.nodes[1].value);
            if (recipe.bytes == source.recipe.bytes) {
                std::vector<std::uint8_t> canonicalNew, canonicalCurrent;
                if (!detail::CanonicalSourceBytes(newSourceBytes, canonicalNew)
                    || !detail::CanonicalSourceBytes(capture->sourceBytes, canonicalCurrent))
                    return refuse("b4.invalid-source-edit");
                if (canonicalNew == canonicalCurrent)
                    return Receipt{Outcome::Unchanged, "b4.no-op", 0, 0, true};
            }
            // Typed downstream proof first: the issued intents must re-resolve
            // on the rebuilt source; a lost/ambiguous anchor refuses here with
            // its exact native reason and no command is ever opened.
            rf::ReplayValue observedInput; observedInput.shape = rf::ShapeKind::Solid;
            observedInput.detachedShape = newSourceBytes;
            retained_topology_budget::Counter observedBudget;
            const std::atomic_bool neverCancelled{false};
            rf::ReplayValue observed; Refusal refusal = Refusal::None;
            if (!draft_faces::detail::buildDetachedObserved(
                    feature, {observedInput}, observedBudget, neverCancelled,
                    observed, &refusal, nullptr))
                return refuse(dfReason(refusal));
            source.recipe = recipe;
            if (!detail::SourceCommitments(source.inputToCarrier, newSourceBytes,
                                           recipe.bytes, source.commitments))
                return refuse("b4.commitment-failure");
            std::vector<std::uint8_t> graphBytes, replayOutput;
            if (!replayCandidate(*capture, graph, newSourceBytes,
                                 graphBytes, replayOutput, receipt)) return receipt;
            if (replayOutput != observed.detachedShape)
                return refuse("b4.candidate-replay-mismatch");
            if (fault == FaultPoint::FailPrepare) return refuse("b4.prepare-fault");
            if (stop.load()) return cancel("b4.cancelled");
            auto stagedSources = capture->record.value->sourceShapes;
            stagedSources[source.shapeSlot] = newSource;
            return transact(capture, graph, graphBytes, replayOutput,
                            capture->record.value->bytes, &stagedSources, stop, fault);
        } catch (...) { return refuse("b4.source-edit-exception"); }
    }

    //! Terminal settlement of an unknown close/publication. Re-reads the
    //! actual document and classifies committed-vs-prior exactly once; it
    //! never opens a command, Undo or Redo. A repeated call returns the cached
    //! terminal receipt without republishing.
    Receipt reconcile(std::uint64_t session) noexcept {
        try {
            if (terminalSet_ && session == terminal_.session) return terminal_;
            if (session == 0 || session != recoverySession_ || recovery_.carrier.IsNull())
                return refuse("b4.foreign-recovery-session");
            const auto native = document_;
            if (native.IsNull() || native->GetData().IsNull())
                return Receipt{Outcome::RecoveryRequired, "b4.recovery-unsettled",
                               session, 0, false};
            bool recordPresent = false;
            std::vector<std::uint8_t> graphBytes;
            cr::Record record;
            if (!cr::Read(native, recovery_.carrier, record)) {
                terminal_ = Receipt{Outcome::RecoveryRequired, "b4.recovery-unsettled",
                                    session, 0, false};
                return terminal_;
            }
            recordPresent = record.value != nullptr;
            if (recordPresent) graphBytes = record.value->bytes;
            std::vector<std::uint8_t> shapeBytes;
            const TopoDS_Shape current = XCAFDoc_ShapeTool::GetShape(recovery_.carrier);
            if (current.IsNull()
                || !draft_faces::detail::exactShapeBytes(current, shapeBytes))
                return Receipt{Outcome::RecoveryRequired, "b4.recovery-unsettled",
                               session, 0, false};
            const bool expectedProven = recordPresent
                && graphBytes == recovery_.expectedGraphBytes
                && shapeBytes == recovery_.expectedShapeBytes;
            const bool priorProven = recovery_.create
                ? (!recordPresent && shapeBytes == recovery_.baselineShapeBytes
                   && profileBaselineEqual(native, recovery_.carrier, recovery_))
                : (recordPresent && graphBytes == recovery_.baselineGraphBytes
                   && shapeBytes == recovery_.baselineShapeBytes);
            if (!expectedProven && !priorProven)
                return Receipt{Outcome::RecoveryRequired, "b4.recovery-unsettled",
                               session, 0, false};
            if (recovery_.context) recovery_.context->reconcileRecovery(true);
            terminal_ = expectedProven
                ? Receipt{Outcome::Committed, "b4.reconciled-candidate", session, 1, true}
                : Receipt{Outcome::Rejected, "b4.reconciled-prior", session, 0, true};
            terminalSet_ = true;
            recoverySession_ = 0; recovery_ = {};
            return terminal_;
        } catch (...) {
            return Receipt{Outcome::RecoveryRequired, "b4.recovery-unsettled",
                           session, 0, false};
        }
    }

private:
    static Receipt refuse(const char* reason) noexcept {
        return Receipt{Outcome::Rejected, reason, 0, 0, true};
    }
    static Receipt cancel(const char* reason) noexcept {
        return Receipt{Outcome::Cancelled, reason, 0, 0, true};
    }
    static const char* dfReason(Refusal refusal) noexcept {
        return draft_faces::Reason(refusal);
    }

    //! Exact pre-transaction reread of the captured authority. Any fence,
    //! history, entity or record drift since mint refuses.
    bool current(const Capture& capture, Receipt& receipt) const noexcept {
        if (recoverySession_ != 0) {
            receipt = refuse("b4.owner-recovery-pending"); return false;
        }
        const auto native = owner_.Document();
        if (pthread_main_np() == 0 || native.IsNull() || native != capture.document
            || native->HasOpenCommand() || !capture.context
            || !capture.context->isCurrent(capture.viewportWidth, capture.viewportHeight)
            || native->GetAvailableUndos() != capture.undoDepth
            || native->GetAvailableRedos() != capture.redoDepth
            || capture.carrier.IsNull()
            || owner_.EntityIdentifierForLabel(capture.carrier) != capture.entityIdentifier) {
            receipt = refuse("b4.stale-capture"); return false;
        }
        if (!capture.forEdit) {
            std::vector<std::uint8_t> shapeBytes;
            const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(capture.carrier);
            if (shape.IsNull()
                || !draft_faces::detail::exactShapeBytes(shape, shapeBytes)
                || shapeBytes != capture.sourceBytes) {
                receipt = refuse("b4.stale-capture"); return false;
            }
            return true;
        }
        cr::Record record;
        if (!cr::Read(native, capture.carrier, record) || !record.value
            || record.value->bytes != capture.record.value->bytes) {
            receipt = refuse("b4.stale-capture"); return false;
        }
        return true;
    }

    //! Candidate graph validation plus the call-bound owner replay. The
    //! registry is constructed fresh per operation and never escapes.
    bool replayCandidate(const Capture& capture, const cr::Definition& graph,
                         const std::vector<std::uint8_t>& sourceBytes,
                         std::vector<std::uint8_t>& graphBytes,
                         std::vector<std::uint8_t>& outputBytes,
                         Receipt& receipt) const noexcept {
        if (!cr::EncodeV3(graph, rf::ProductionRegistry(),
                          retained_source::ProductionRegistry(), graphBytes)) {
            receipt = refuse("b4.noncanonical-graph"); return false;
        }
        OwnerRegistry registry;
        rf::ReplayValue sourceValue; sourceValue.shape = rf::ShapeKind::Solid;
        sourceValue.detachedShape = sourceBytes;
        if (!cr::Hash(sourceBytes, sourceValue.geometry)) {
            receipt = refuse("b4.commitment-failure"); return false;
        }
        const auto& source = std::get<cr::SourceNode>(graph.nodes[0].value);
        if (!cr::Hash(source.recipe.bytes, sourceValue.familyProof)) {
            receipt = refuse("b4.commitment-failure"); return false;
        }
        rf::SourceValue bound; bound.node = source.node; bound.value = sourceValue;
        rf::OwnerExecutionContext context; context.graph = &graph;
        rf::ReplayBudget budget;
        const auto replay = rf::ReplayOwnerDetached(
            graph, registry.validationView(), registry.executionView(),
            retained_source::ProductionRegistry(), {bound}, context, budget);
        if (!replay.replayed()) {
            receipt = refuse(replayReason(replay.reason)); return false;
        }
        outputBytes = replay.output.detachedShape;
        return !outputBytes.empty();
    }

    static const char* replayReason(const std::string& reason) noexcept {
        if (reason == "foreign-owner-execution-context") return "b4.foreign-owner-context";
        if (reason == "owner-execution-not-installed") return "b4.owner-execution-not-installed";
        if (reason == "unknown-feature-codec") return "b4.unknown-feature-codec";
        if (reason == "detached-build") return "b4.replay-build-refused";
        if (reason == "family-proof") return "b4.replay-family-proof";
        if (reason == "fixed-point") return "b4.replay-fixed-point";
        if (reason == "replay-budget") return "b4.replay-budget";
        return "b4.replay-refused";
    }

    //! The one issued command: lease, stage all affected records under it,
    //! exact readback, close, publish. Known failure aborts and proves the
    //! exact prior; an unknown close/publication retains issuer, candidate
    //! and recovery fence for exactly-one reconciliation.
    Receipt transact(const std::shared_ptr<const Capture>& capture,
                     const cr::Definition& graph,
                     const std::vector<std::uint8_t>& graphBytes,
                     const std::vector<std::uint8_t>& outputBytes,
                     const std::vector<std::uint8_t>& baselineGraphBytes,
                     const std::vector<TopoDS_Shape>* stagedSources,
                     const std::atomic_bool& stop,
                     FaultPoint fault) noexcept {
        const auto native = owner_.Document();
        TopoDS_Shape drafted, previous = XCAFDoc_ShapeTool::GetShape(capture->carrier);
        if (previous.IsNull()
            || !draft_faces::detail::readShape(outputBytes, drafted) || drafted.IsNull())
            return refuse("b4.candidate-unreadable");
        // Retain the recovery record before any mutation.
        recovery_ = {};
        recovery_.create = !capture->forEdit;
        recovery_.entityIdentifier = capture->entityIdentifier;
        recovery_.carrier = capture->carrier;
        recovery_.context = capture->context;
        recovery_.baselineGraphBytes = baselineGraphBytes;
        recovery_.expectedGraphBytes = graphBytes;
        recovery_.expectedShapeBytes = outputBytes;
        if (!draft_faces::detail::exactShapeBytes(previous, recovery_.baselineShapeBytes)) {
            recovery_ = {}; return refuse("b4.commitment-failure");
        }
        native_opening::CommittedEditPublication publication;
        publication.replaced.push_back({capture->entityIdentifier, previous});
        auto lease = capture->context->beginCommandLease(
            capture->context->openingFence(), capture->viewportWidth,
            capture->viewportHeight);
        if (!lease || !lease->ownsOpenCommand()) {
            recovery_ = {}; return refuse("b4.lease-refused");
        }
        const char* stageFailure = nullptr;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(native->Main());
        profile::Record sourceProfile;
        if (!capture->forEdit) {
            std::vector<double> capturedValues, storedValues;
            if (!profile::Read(native, capture->carrier, sourceProfile)) {
                stageFailure = "b4.stage-profile";
            } else if (!sourceProfile.label.IsNull()) {
                if (!profile::Encode(capture->sourceProfile, capturedValues)
                    || !profile::Encode(sourceProfile.parameters, storedValues)
                    || storedValues != capturedValues) {
                    stageFailure = "b4.stage-profile-foreign";
                } else if (!sourceProfile.IsCurrent(native, capture->carrier)) {
                    stageFailure = "b4.stage-profile-stale";
                } else {
                    recovery_.baselineProfilePresent = true;
                    recovery_.baselineProfileIdentifier = sourceProfile.identifier;
                    recovery_.baselineProfileValues = std::move(storedValues);
                }
            }
        }
        if (!stageFailure && !capture->forEdit && !sourceProfile.label.IsNull()) {
            sourceProfile.label.ForgetAllAttributes(Standard_True);
            if (profile::HasAttribute(sourceProfile.label))
                stageFailure = "b4.stage-profile-retire";
        }
        if (!stageFailure && !capture->forEdit) {
            for (TDF_ChildIterator child(capture->carrier, Standard_False);
                 child.More(); child.Next()) {
                const TDF_Label label = child.Value();
                if (profile::HasAttribute(label) || enclosure::HasAttribute(label)
                    || sweep_persistence::HasAttribute(label)
                    || loft_persistence::HasAttribute(label)
                    || retained_solid::HasRecord(label)) {
                    stageFailure = "b4.stage-recipe-owner";
                    break;
                }
            }
        }
        TDF_Label record;
        if (!stageFailure && !capture->forEdit) {
            record = capture->carrier.FindChild(cr::MinimumRecordTag, Standard_True);
            if (record.IsNull() || record.HasAttribute())
                stageFailure = "b4.stage-record";
        }
        if (!stageFailure) {
            shapes->SetShape(capture->carrier, drafted);
            if (capture->forEdit) {
                const TDF_Label ensured = capture->carrier.FindChild(
                    cr::MinimumRecordTag, Standard_True);
                if (ensured.IsNull()) stageFailure = "b4.stage-record";
            }
        }
        if (capture->forEdit)
            record = capture->carrier.FindChild(cr::MinimumRecordTag, Standard_False);
        if (!stageFailure) {
            auto payload = std::make_shared<cr::Payload>();
            payload->definition = graph; payload->bytes = graphBytes;
            if (stagedSources) payload->sourceShapes = *stagedSources;
            else {
                TopoDS_Shape source;
                if (!draft_faces::detail::readShape(capture->sourceBytes, source))
                    stageFailure = "b4.stage-source";
                else payload->sourceShapes.push_back(source);
            }
            if (!stageFailure) {
                if (!capture->forEdit) {
                    Handle(cr::Attribute) attribute = new cr::Attribute();
                    record.AddAttribute(attribute);
                }
                if (!owner_.StagePartBooleanPayload(record, payload))
                    stageFailure = "b4.stage-payload";
            }
            if (!stageFailure) TNaming_Builder(record).Select(drafted, drafted);
        }
        // DEBUG FailStage injects after real staging so the abort below rolls
        // back an actually partially staged command and must prove the prior.
        if (fault == FaultPoint::FailStage) stageFailure = "b4.stage-fault";
        if (!stageFailure) {
            cr::Record readback;
            if (!cr::Read(native, capture->carrier, readback) || !readback.value
                || readback.value->bytes != graphBytes
                || !readback.current.IsEqual(drafted))
                stageFailure = "b4.readback-failed";
        }
        if (fault == FaultPoint::FailReadback) stageFailure = "b4.readback-fault";
        if (stageFailure) {
            const bool aborted = lease->abort();
            bool priorProven = false;
            if (aborted && !native->HasOpenCommand()) {
                std::vector<std::uint8_t> actual;
                const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(capture->carrier);
                priorProven = !shape.IsNull()
                    && draft_faces::detail::exactShapeBytes(shape, actual)
                    && actual == recovery_.baselineShapeBytes
                    && (capture->forEdit
                        ? recordBytesEqual(native, capture->carrier,
                                           recovery_.baselineGraphBytes)
                        : (!cr::HasRecord(capture->carrier)
                           && profileBaselineEqual(native, capture->carrier, recovery_)));
            }
            if (priorProven) { recovery_ = {}; return refuse(stageFailure); }
            capture->context->armRecoveryAfterAbortedClose();
            capture->context->retainUnprovenEdit(publication);
            recoverySession_ = ++nextSession_;
            return Receipt{Outcome::RecoveryRequired, "b4.stage-outcome-unknown",
                           recoverySession_, 0, false};
        }
#if DEBUG
        if (fault == FaultPoint::UnprovenClose)
            capture->context->debugReportNextCloseUnproven();
#endif
        if (stop.load()) {
            // Late Stop drains through a real abort and must then prove the
            // exact prior; it never claims Cancelled while state is unproven.
            const bool aborted = lease->abort();
            std::vector<std::uint8_t> actual;
            const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(capture->carrier);
            const bool priorProven = aborted && !native->HasOpenCommand()
                && !shape.IsNull()
                && draft_faces::detail::exactShapeBytes(shape, actual)
                && actual == recovery_.baselineShapeBytes
                && (capture->forEdit
                    ? recordBytesEqual(native, capture->carrier,
                                       recovery_.baselineGraphBytes)
                    : (!cr::HasRecord(capture->carrier)
                       && profileBaselineEqual(native, capture->carrier, recovery_)));
            if (priorProven) { recovery_ = {}; return cancel("b4.cancelled"); }
            capture->context->armRecoveryAfterAbortedClose();
            capture->context->retainUnprovenEdit(publication);
            recoverySession_ = ++nextSession_;
            return Receipt{Outcome::RecoveryRequired, "b4.close-outcome-unknown",
                           recoverySession_, 0, false};
        }
        if (!lease->commit()) {
            capture->context->retainUnprovenEdit(publication);
            recoverySession_ = ++nextSession_;
            return Receipt{Outcome::RecoveryRequired, "b4.close-outcome-unknown",
                           recoverySession_, 0, false};
        }
        const int delta = int(native->GetAvailableUndos() - capture->undoDepth);
        if (delta != 1) {
            capture->context->retainUnprovenEdit(publication);
            recoverySession_ = ++nextSession_;
            return Receipt{Outcome::RecoveryRequired, "b4.close-outcome-unknown",
                           recoverySession_, 0, false};
        }
        if (!capture->context->publishCommittedEdit(publication)) {
            recoverySession_ = ++nextSession_;
            return Receipt{Outcome::RecoveryRequired, "b4.publication-outcome-unknown",
                           recoverySession_, 0, false};
        }
        recovery_ = {};
        return Receipt{Outcome::Committed, "b4.committed", 0, delta, true};
    }

    static bool recordBytesEqual(const Handle(TDocStd_Document)& native,
                                 const TDF_Label& carrier,
                                 const std::vector<std::uint8_t>& expected) noexcept {
        cr::Record record;
        return cr::Read(native, carrier, record) && record.value
            && record.value->bytes == expected;
    }

    static bool profileBaselineEqual(const Handle(TDocStd_Document)& native,
                                     const TDF_Label& carrier,
                                     const detail::RecoverySnapshot& expected) noexcept {
        profile::Record actual;
        if (!profile::Read(native, carrier, actual)) return false;
        if (!expected.baselineProfilePresent) return actual.label.IsNull();
        std::vector<double> values;
        return !actual.label.IsNull()
            && actual.identifier == expected.baselineProfileIdentifier
            && profile::Encode(actual.parameters, values)
            && values == expected.baselineProfileValues
            && actual.IsCurrent(native, carrier);
    }

    // Recovery state is owner-held per document; S3 binds the long-lived
    // instance. Nothing here is persisted or serializable.
    OcctDocument& owner_;
    Handle(TDocStd_Document) document_;
    std::uint64_t nextSession_ = 0, recoverySession_ = 0;
    detail::RecoverySnapshot recovery_;
    Receipt terminal_{};
    bool terminalSet_ = false;
};

} // namespace core3d::draft_faces::owner
