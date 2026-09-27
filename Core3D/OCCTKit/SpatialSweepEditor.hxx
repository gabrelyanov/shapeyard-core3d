#pragma once

// Native-only C2 Objects editor adapter. A Candidate contains descriptive
// authored values, never document authority. Authority remains the exact G0
// CaptureResult and is re-read by Transaction::Apply immediately before the
// single owned command.
#include "SpatialSweepG0Transaction.hxx"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <TopoDS_Wire.hxx>
#include <atomic>

namespace core3d::spatial_sweep::editor {
using composite_recipe::Digest;
using composite_recipe::FeatureNode;
using composite_recipe::Node;
using composite_recipe::Payload;
using composite_recipe::SourceNode;
using composite_recipe::spatial_g0::CaptureResult;
using composite_recipe::spatial_g0::Prepared;

enum class Status : std::uint8_t {
    Prepared = 0,
    Unchanged,
    Cancelled,
    StaleAuthority,
    UnsupportedDescendant,
    InvalidCurve,
    InvalidFeature,
    InvalidBinding,
    BuildRefused,
    FixedPointRefused,
};

struct Candidate {
    bounded_curve::Definition curve;
    spatial_sweep::Definition sweep;
};

struct Result {
    Status status = Status::InvalidBinding;
    Prepared prepared;
    bool admitted() const noexcept { return status == Status::Prepared && prepared.admitted(); }
};

inline bool SameFrameScalars(const bounded_curve::Frame& a,
                             const bounded_curve::Frame& b) noexcept {
    return a.identifier == b.identifier && a.origin == b.origin
        && a.xAxis == b.xAxis && a.yAxis == b.yAxis && a.zAxis == b.zAxis
        && a.handedness == b.handedness;
}

inline bool SamePoleIdentities(const bounded_curve::Definition& a,
                               const bounded_curve::Definition& b) noexcept {
    if (a.controlPoints.size() != b.controlPoints.size()) return false;
    for (std::size_t i = 0; i < a.controlPoints.size(); ++i)
        if (a.controlPoints[i].identifier != b.controlPoints[i].identifier) return false;
    return true;
}

inline bool MakeNativeWire(const bounded_curve::Definition& definition,
                           TopoDS_Shape& wire,
                           Digest& digest) noexcept {
    wire.Nullify(); digest = {};
    try {
        // The retained source is stored in source-native units with its frame
        // applied exactly once. Physical-mm conversion belongs to the builder.
        Handle(Geom_BSplineCurve) curve = geomfill_detail::Curve(definition, 1.0);
        BRepBuilderAPI_MakeEdge edge(curve);
        if (!edge.IsDone()) return false;
        BRepBuilderAPI_MakeWire builder(edge.Edge());
        if (!builder.IsDone()) return false;
        TopoDS_Wire candidate = builder.Wire();
        candidate.Orientation(TopAbs_FORWARD);
        if (!geomfill_detail::HashShape(candidate, digest)) return false;
        wire = candidate;
        return true;
    } catch (...) { wire.Nullify(); digest = {}; return false; }
}

inline Result Prepare(const CaptureResult& opening,
                      Candidate candidate,
                      const std::atomic_bool& cancelled) noexcept {
    Result result;
    try {
        if (cancelled.load()) { result.status = Status::Cancelled; return result; }
        if (!opening.admitted() || !opening.payload) {
            result.status = opening.refusal == composite_recipe::spatial_g0::CaptureRefusal::UnsupportedDescendant
                ? Status::UnsupportedDescendant : Status::StaleAuthority;
            return result;
        }
        if (candidate.curve.domain != bounded_curve::Domain::Path3D
            || candidate.curve.schema != opening.curve.definition.schema
            || candidate.curve.frame.identifier != opening.curve.definition.frame.identifier
            || !SamePoleIdentities(candidate.curve, opening.curve.definition)) {
            result.status = Status::InvalidCurve; return result;
        }
        const bool frameChanged = !SameFrameScalars(candidate.curve.frame,
                                                     opening.curve.definition.frame);
        candidate.curve.frame.revision = frameChanged
            ? opening.curve.definition.frame.revision + 1
            : opening.curve.definition.frame.revision;
        if (bounded_curve::Validate(candidate.curve) != bounded_curve::Refusal::None
            || (frameChanged && bounded_curve::ValidateFrameReplacement(
                    opening.curve.definition.frame, candidate.curve.frame)
                    != bounded_curve::Refusal::None)) {
            result.status = Status::InvalidCurve; return result;
        }

        // Every internal identity, binding, unit and compatibility policy is
        // copied from the opening. The editor changes authored geometry only.
        candidate.sweep.schema = opening.sweep.schema;
        candidate.sweep.path = opening.sweep.path;
        candidate.sweep.dimensionMetersPerUnit = opening.sweep.dimensionMetersPerUnit;
        candidate.sweep.section = opening.sweep.section;
        candidate.sweep.sectionIdentifier = opening.sweep.sectionIdentifier;
        candidate.sweep.transport = opening.sweep.transport;
        candidate.sweep.parameterization = opening.sweep.parameterization;
        candidate.sweep.profile = opening.sweep.profile;
        if (candidate.sweep.closure == ClosureKind::OpenFlatCaps) {
            candidate.sweep.witness = {};
            candidate.sweep.twist.kind = TwistLawKind::LinearArcLength;
            candidate.sweep.twist.windingTurns = 0;
        } else {
            candidate.sweep.twist.kind = TwistLawKind::CloseFrame;
            candidate.sweep.twist.totalRadians = 0;
            // Retain the checked continuation branch. Opening a previously
            // open recipe as closed is deliberately refused until a native
            // seam rebase has supplied a witness.
            if (opening.sweep.closure != ClosureKind::ClosedNoCaps
                || !opening.sweep.witness.present) {
                result.status = Status::InvalidFeature; return result;
            }
            candidate.sweep.witness = opening.sweep.witness;
        }

        bounded_curve::Value curveValue = opening.curve;
        curveValue.definition = candidate.curve;
        std::vector<std::uint8_t> curveBytes;
        Digest curveDigest{};
        if (!bounded_curve::Encode(curveValue, curveBytes)
            || !bounded_curve::Hash(curveBytes, bounded_curve::MaximumDefinitionBytes,
                                    curveDigest)) {
            result.status = Status::InvalidCurve; return result;
        }
        const bool authoredUnchanged = curveBytes == opening.fence.curveBytes
            && SameDefinition(candidate.sweep, opening.sweep);
        if (authoredUnchanged) {
            result.status = Status::Unchanged; return result;
        }
        candidate.sweep.path.sourceRecipeDigest = curveDigest;
        candidate.sweep.path.ownerState.definitionRevision =
            opening.sweep.path.ownerState.definitionRevision + 1;
        candidate.sweep.path.ownerState.canonicalDefinitionDigest = curveDigest;
        if (Validate(candidate.sweep) != Refusal::None) {
            result.status = Status::InvalidFeature; return result;
        }
        std::vector<std::uint8_t> featureBytes;
        if (!spatial_sweep::Encode(candidate.sweep, featureBytes)) {
            result.status = Status::InvalidFeature; return result;
        }

        auto payload = std::make_shared<Payload>(*opening.payload);
        SourceNode* source = nullptr; FeatureNode* feature = nullptr;
        for (Node& node : payload->definition.nodes) {
            if (auto* value = std::get_if<SourceNode>(&node.value)) source = value;
            else if (auto* value = std::get_if<FeatureNode>(&node.value)) feature = value;
        }
        if (!source || !feature || source->node != opening.fence.sourceNode
            || feature->feature != opening.fence.sweepFeature
            || source->shapeSlot >= payload->sourceShapes.size()) {
            result.status = Status::InvalidBinding; return result;
        }
        TopoDS_Shape sourceWire; Digest wireDigest{};
        if (!MakeNativeWire(candidate.curve, sourceWire, wireDigest)) {
            result.status = Status::InvalidCurve; return result;
        }
        source->recipe.bytes = curveBytes;
        source->commitments.recipe = curveDigest;
        source->commitments.geometry = wireDigest;
        feature->parameters = featureBytes;
        payload->sourceShapes[source->shapeSlot] = sourceWire;
        if (!composite_recipe::EncodeV2(payload->definition, payload->bytes)) {
            result.status = Status::InvalidBinding; return result;
        }

        const GeomFillBuildResult first = BuildGeomFillSweep(candidate.curve,
                                                              candidate.sweep,
                                                              cancelled);
        if (cancelled.load()) { result.status = Status::Cancelled; return result; }
        const GeomFillBuildResult second = BuildGeomFillSweep(candidate.curve,
                                                               candidate.sweep,
                                                               cancelled);
        if (!first.built() || !second.built()) {
            result.status = Status::BuildRefused; return result;
        }
        result.prepared = composite_recipe::spatial_g0::Prepare(
            opening, first, second, payload,
            composite_recipe::spatial_g0::PinnedProfile);
        result.status = result.prepared.admitted()
            ? Status::Prepared : Status::FixedPointRefused;
        return result;
    } catch (...) { return result; }
}
} // namespace core3d::spatial_sweep::editor
