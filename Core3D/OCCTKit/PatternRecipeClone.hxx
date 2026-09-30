#pragma once

// D1a policy for independent copies of current sweep and rectangular-loft
// owners. Callers own geometry construction and the surrounding OCAF command;
// this component owns recipe admission, frame composition, identity separation,
// exact staging, and source/candidate read-back.
#include "CompositeRecipeAttribute.hxx"
#include "PartBooleanBuild.hxx"
#include "PartBooleanRebuild.hxx"
#include "RectangularLoftPersistence.hxx"
#include "RetainedSolidAttribute.hxx"

#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>

namespace core3d::pattern_recipe_clone {

enum class Family : std::uint8_t {
    None = 0, Sweep = 1, Loft = 2, AnalyticBoolean = 3
};

using UUID = retained_recipe::UUID;
using IssueUUID = std::function<bool(UUID&)>;

struct IdentityMap {
    std::map<UUID, UUID> global;
};

struct Source {
    Family family = Family::None;
    TDF_Label owner;
    TopoDS_Shape ownerShape;
    bool carrierLengthUnitPresent = false;
    double carrierMetersPerUnit = 0;
    sweep_persistence::Record sweep;
    loft_persistence::Record loft;
    composite_recipe::Record analyticBoolean;
    part_boolean::AnalyticDefinition analyticDefinition;
};

struct Prepared {
    Family family = Family::None;
    TopoDS_Shape binding;
    std::string featureIdentifier;
    planar_sweep::Definition sweep;
    rectangular_loft::Definition loft;
    std::shared_ptr<const composite_recipe::Payload> analyticBoolean;
    IdentityMap identities;
};

struct Candidate {
    Family family = Family::None;
    std::string ownerEntityIdentifier;
    std::string ownerDefinitionIdentifier;
    sweep_persistence::Record sweep;
    loft_persistence::Record loft;
    std::vector<std::uint8_t> analyticBooleanBytes;
    IdentityMap identities;
};

inline bool ShapeBytesEqual(const TopoDS_Shape& left,
                            const TopoDS_Shape& right) noexcept {
    try {
        std::string a, b;
        return !left.IsNull() && !right.IsNull()
            && retained_part_boolean::ExactShapeBytes(left, a)
            && retained_part_boolean::ExactShapeBytes(right, b) && a == b;
    } catch (...) { return false; }
}

inline bool IsProperRigidPlacement(const gp_Trsf& value) noexcept {
    try {
        const gp_TrsfForm form = value.Form();
        return !value.IsNegative() && value.ScaleFactor() == 1.0
            && form != gp_Other && form != gp_Scale && form != gp_PntMirror
            && form != gp_Ax1Mirror && form != gp_Ax2Mirror;
    } catch (...) { return false; }
}

inline bool AnalyticReads(const composite_recipe::Definition& graph,
                          retained_part_boolean::OperandReadSet& reads) noexcept {
    reads = {};
    try {
        if (graph.nodes.size() != 3) return false;
        std::array<const composite_recipe::SourceNode*, 2> sources{{nullptr, nullptr}};
        std::size_t count = 0;
        for (const auto& node : graph.nodes) {
            const auto* source = std::get_if<composite_recipe::SourceNode>(&node.value);
            if (source) {
                if (count == sources.size()) return false;
                sources[count++] = source;
            }
        }
        const auto* feature = std::get_if<composite_recipe::FeatureNode>(
            &graph.nodes.back().value);
        if (count != 2 || !feature
            || feature->kind != composite_recipe::PartBooleanFeatureKind
            || feature->codecVersion != composite_recipe::PartBooleanFeatureCodec
            || feature->inputs.size() != 2
            || feature->inputs[0] != sources[0]->node
            || feature->inputs[1] != sources[1]->node) return false;
        const auto assign = [&](retained_recipe::DependencyRead& read,
                                const composite_recipe::SourceNode& source) {
            read.locator = {graph.owner, source.node, source.original.sourceFeature};
            read.geometry = source.commitments.geometry;
            read.recipe = source.commitments.recipe;
            read.placement = source.commitments.placement;
            read.material = source.commitments.material;
            read.groups = source.commitments.groups;
        };
        assign(reads.leftSource, *sources[0]);
        assign(reads.rightSource, *sources[1]);
        return true;
    } catch (...) { reads = {}; return false; }
}

inline bool IsCurrentAnalyticBoolean(const Handle(TDocStd_Document)& document,
                                     const TDF_Label& owner,
                                     composite_recipe::Record& record,
                                     part_boolean::AnalyticDefinition& analytic) noexcept {
    record = {}; analytic = {};
    try {
        if (!composite_recipe::Read(document, owner, record) || !record.value
            || record.value->definition.nodes.size() != 3
            || record.value->sourceShapes.size() != 2
            || !record.current.IsEqual(XCAFDoc_ShapeTool::GetShape(owner))) return false;
        const auto* feature = std::get_if<composite_recipe::FeatureNode>(
            &record.value->definition.nodes.back().value);
        retained_part_boolean::OperandReadSet reads;
        if (!feature || !AnalyticReads(record.value->definition, reads)
            || !part_boolean::DecodeAnalytic(feature->parameters, analytic)) return false;
        double unit = 0;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document, unit)) return false;
        const auto built = part_boolean::build::BuildAnalytic(analytic, unit, reads, reads);
        const auto fixed = part_boolean::rebuild::CheckAnalytic(analytic, unit, reads);
        return built.complete && fixed.fixedPoint()
            && ShapeBytesEqual(built.candidate.solid, record.current)
            && ShapeBytesEqual(built.sources[0], record.value->sourceShapes[0])
            && ShapeBytesEqual(built.sources[1], record.value->sourceShapes[1]);
    } catch (...) { record = {}; analytic = {}; return false; }
}

inline bool IsEqual(const Source& left, const Source& right) noexcept {
    try {
        const bool sameOwner = left.owner.IsNull() ? right.owner.IsNull()
            : !right.owner.IsNull() && left.owner.IsEqual(right.owner)
                && left.owner.Data() == right.owner.Data();
        const bool sameShape = left.ownerShape.IsNull() ? right.ownerShape.IsNull()
            : !right.ownerShape.IsNull() && left.ownerShape.IsEqual(right.ownerShape);
        return left.family == right.family && sameOwner && sameShape
            && left.carrierLengthUnitPresent == right.carrierLengthUnitPresent
            && (!left.carrierLengthUnitPresent
                || left.carrierMetersPerUnit == right.carrierMetersPerUnit)
            && left.sweep.IsEqual(right.sweep) && left.loft.IsEqual(right.loft)
            && ((!left.analyticBoolean.value && !right.analyticBoolean.value)
                || (left.analyticBoolean.value && right.analyticBoolean.value
                    && left.analyticBoolean.value->bytes == right.analyticBoolean.value->bytes
                    && ShapeBytesEqual(left.analyticBoolean.current,
                                       right.analyticBoolean.current)));
    } catch (...) { return false; }
}

inline bool SameLocalIDs(const planar_sweep::Definition& left,
                         const planar_sweep::Definition& right) noexcept {
    if (left.pathIdentifier != right.pathIdentifier
        || left.vertices.size() != right.vertices.size()
        || left.segments.size() != right.segments.size()) return false;
    for (std::size_t index = 0; index < left.vertices.size(); ++index)
        if (left.vertices[index].identifier != right.vertices[index].identifier) return false;
    for (std::size_t index = 0; index < left.segments.size(); ++index) {
        const auto& a = left.segments[index]; const auto& b = right.segments[index];
        if (a.identifier != b.identifier || a.startVertex != b.startVertex
            || a.endVertex != b.endVertex) return false;
    }
    return true;
}

inline bool SameLocalIDs(const rectangular_loft::Definition& left,
                         const rectangular_loft::Definition& right) noexcept {
    if (left.loftIdentifier != right.loftIdentifier
        || left.correspondence != right.correspondence
        || left.stations.size() != right.stations.size()) return false;
    for (std::size_t index = 0; index < left.stations.size(); ++index) {
        const auto& a = left.stations[index]; const auto& b = right.stations[index];
        if (a.identifier != b.identifier || a.cornerIdentifiers != b.cornerIdentifiers
            || a.correspondence != b.correspondence) return false;
    }
    return true;
}

// A present-but-stale, malformed, composite, or retained-cut owner refuses.
// The legacy readers already require their binding and document units to be
// current, so success is a usable transaction fence rather than a readable
// historical recipe.
inline bool Capture(const Handle(TDocStd_Document)& document,
                    const TDF_Label& owner, Source& output) noexcept {
    output = {};
    try {
        if (document.IsNull() || owner.IsNull() || owner.Data() != document->GetData()
            || retained_solid::HasRecord(owner)) return false;
        Source value; value.owner = owner; value.ownerShape = XCAFDoc_ShapeTool::GetShape(owner);
        if (value.ownerShape.IsNull()) return false;
        double carrierMetersPerUnit = 0;
        value.carrierLengthUnitPresent = XCAFDoc_DocumentTool::GetLengthUnit(
            document, carrierMetersPerUnit);
        if (value.carrierLengthUnitPresent) {
            if (!std::isfinite(carrierMetersPerUnit) || carrierMetersPerUnit <= 0) return false;
            value.carrierMetersPerUnit = carrierMetersPerUnit;
        }
        if (!sweep_persistence::Read(document, owner, value.sweep)
            || !loft_persistence::Read(document, owner, value.loft)) return false;
        const bool hasAnalyticBoolean = composite_recipe::HasRecord(owner);
        const unsigned families = unsigned(hasAnalyticBoolean)
            + unsigned(!value.sweep.label.IsNull()) + unsigned(!value.loft.label.IsNull());
        if (families > 1) return false;
        value.family = hasAnalyticBoolean ? Family::AnalyticBoolean
            : !value.sweep.label.IsNull() ? Family::Sweep
            : !value.loft.label.IsNull() ? Family::Loft : Family::None;
        if (value.family == Family::None) {
            output = std::move(value); return true;
        }
        if (!value.carrierLengthUnitPresent
            || value.ownerShape.ShapeType() != TopAbs_SOLID) return false;
        if (value.family == Family::AnalyticBoolean) {
            if (!IsCurrentAnalyticBoolean(document, owner, value.analyticBoolean,
                                          value.analyticDefinition)) return false;
            output = std::move(value); return true;
        }
        if ((value.family == Family::Sweep
                && (!value.sweep.IsCurrent(document, owner)
                    || !value.sweep.boundShape.IsEqual(value.ownerShape)))
            || (value.family == Family::Loft
                && (!value.loft.IsCurrent(document, owner)
                    || !value.loft.boundShape.IsEqual(value.ownerShape)))) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

inline bool IssueMapped(const UUID& oldValue, const IssueUUID& issue,
                        std::set<UUID>& occupied, IdentityMap& map,
                        UUID& output) noexcept {
    try {
        if (!retained_recipe::Nonzero(oldValue) || !issue) return false;
        const auto found = map.global.find(oldValue);
        if (found != map.global.end()) { output = found->second; return true; }
        UUID fresh{};
        if (!issue(fresh) || !retained_recipe::Nonzero(fresh)
            || fresh == oldValue || !occupied.insert(fresh).second) return false;
        map.global.emplace(oldValue, fresh); output = fresh; return true;
    } catch (...) { return false; }
}

// D1-C is deliberately a separate typed path. It reissues every durable global
// graph/source/feature/material identity while retaining owner-local localIDs
// and the complete issuance/nonreuse state byte-for-byte.
inline bool PrepareAnalyticBoolean(const Source& source,
                                   const TopoDS_Shape& destinationShape,
                                   const retained_recipe::OwnerKey& destinationOwner,
                                   const IssueUUID& issue, Prepared& output) noexcept {
    output = {};
    try {
        if (source.family != Family::AnalyticBoolean
            || !source.analyticBoolean.value || destinationShape.IsNull()
            || destinationShape.IsPartner(source.ownerShape)
            || !retained_recipe::Valid(destinationOwner)
            || destinationOwner.document
                != source.analyticBoolean.value->definition.owner.document) return false;
        auto payload = std::make_shared<composite_recipe::Payload>();
        payload->definition = source.analyticBoolean.value->definition;
        const auto localIssuance = payload->definition.issuance;
        std::set<UUID> occupied;
        for (const auto& node : payload->definition.nodes) {
            occupied.insert(composite_recipe::NodeID(node));
            if (const auto* value = std::get_if<composite_recipe::SourceNode>(&node.value)) {
                occupied.insert(value->original.entity);
                occupied.insert(value->original.definition);
                occupied.insert(value->original.sourceFeature);
            } else occupied.insert(std::get<composite_recipe::FeatureNode>(node.value).feature);
        }
        IdentityMap identities;
        identities.global.emplace(payload->definition.owner.entity, destinationOwner.entity);
        identities.global.emplace(payload->definition.owner.definition, destinationOwner.definition);
        payload->definition.owner = destinationOwner;
        for (auto& node : payload->definition.nodes) {
            if (auto* sourceNode = std::get_if<composite_recipe::SourceNode>(&node.value)) {
                if (!IssueMapped(sourceNode->node, issue, occupied, identities, sourceNode->node)
                    || !IssueMapped(sourceNode->original.entity, issue, occupied, identities, sourceNode->original.entity)
                    || !IssueMapped(sourceNode->original.definition, issue, occupied, identities, sourceNode->original.definition)
                    || !IssueMapped(sourceNode->original.sourceFeature, issue, occupied, identities,
                                    sourceNode->original.sourceFeature)) return false;
                sourceNode->original.document = destinationOwner.document;
            } else {
                auto& featureNode = std::get<composite_recipe::FeatureNode>(node.value);
                if (!IssueMapped(featureNode.node, issue, occupied, identities, featureNode.node)
                    || !IssueMapped(featureNode.feature, issue, occupied, identities, featureNode.feature)) return false;
                for (UUID& input : featureNode.inputs) {
                    const auto found = identities.global.find(input);
                    if (found == identities.global.end()) return false;
                    input = found->second;
                }
            }
        }
        const auto outputFound = identities.global.find(payload->definition.outputNode);
        if (outputFound == identities.global.end()) return false;
        payload->definition.outputNode = outputFound->second;
        auto* feature = std::get_if<composite_recipe::FeatureNode>(
            &payload->definition.nodes.back().value);
        part_boolean::AnalyticDefinition analytic = source.analyticDefinition;
        for (auto& input : analytic.inputs) {
            auto root = identities.global.find(input.rootNode);
            auto sourceFeature = identities.global.find(input.originalSourceFeature);
            if (root == identities.global.end() || sourceFeature == identities.global.end()) return false;
            input.rootNode = root->second;
            input.originalSourceFeature = sourceFeature->second;
            if (!IssueMapped(input.originalMaterial.identifier, issue, occupied,
                             identities, input.originalMaterial.identifier)) return false;
        }
        if (!feature || !part_boolean::EncodeAnalytic(analytic, feature->parameters)
            || payload->definition.issuance.nextLocalID != localIssuance.nextLocalID
            || payload->definition.issuance.retiredLocalIDs != localIssuance.retiredLocalIDs
            || !composite_recipe::Encode(payload->definition, payload->bytes)) return false;
        retained_part_boolean::OperandReadSet reads;
        if (!AnalyticReads(payload->definition, reads)
            || !std::isfinite(source.carrierMetersPerUnit)
            || source.carrierMetersPerUnit <= 0) return false;
        // Build/replay is repeated by Stage against the live document. This
        // detached pass proves the copy is recipe-derived, never a cached root.
        const auto built = part_boolean::build::BuildAnalytic(
            analytic, source.carrierMetersPerUnit, reads, reads);
        const auto fixed = part_boolean::rebuild::CheckAnalytic(
            analytic, source.carrierMetersPerUnit, reads);
        if (!built.complete || !fixed.fixedPoint()
            || !ShapeBytesEqual(built.candidate.solid, destinationShape)) return false;
        payload->sourceShapes.assign(built.sources.begin(), built.sources.end());
        output.family = Family::AnalyticBoolean; output.binding = destinationShape;
        output.analyticBoolean = std::move(payload); output.identities = std::move(identities);
        return true;
    } catch (...) { output = {}; return false; }
}

inline bool ComposeFrame(const std::optional<profile::ConstructionFrame>& original,
                         const gp_Trsf& bakedTransform,
                         std::optional<profile::ConstructionFrame>& output) noexcept {
    output.reset();
    try {
        gp_Trsf originalTransform;
        if (original && !original->Transform(originalTransform)) return false;
        profile::ConstructionFrame frame;
        if (!profile::ConstructionFrame::Capture(bakedTransform * originalTransform, frame)) return false;
        output = frame; return true;
    } catch (...) { output.reset(); return false; }
}

inline bool Prepare(const Source& source, const TopoDS_Shape& destinationShape,
                    const std::string& newFeatureIdentifier,
                    const std::optional<gp_Trsf>& bakedTransform,
                    Prepared& output) noexcept {
    output = {};
    try {
        if (source.owner.IsNull() || source.ownerShape.IsNull()
            || destinationShape.IsNull()
            || destinationShape.IsPartner(source.ownerShape)) return false;
        Prepared value; value.family = source.family; value.binding = destinationShape;
        if (source.family == Family::None) {
            // Legacy recipe-less/profile/enclosure mirrors own their frame in
            // the existing path; this policy is deliberately a no-op for them.
            if (!newFeatureIdentifier.empty()) return false;
            output = std::move(value); return true;
        }
        if (destinationShape.ShapeType() != TopAbs_SOLID) return false;
        const std::string& oldIdentifier = source.family == Family::Sweep
            ? source.sweep.identifier : source.loft.identifier;
        if (!profile::IsIdentifier(newFeatureIdentifier)
            || newFeatureIdentifier == oldIdentifier) return false;
        value.featureIdentifier = newFeatureIdentifier;
        if (source.family == Family::Sweep) {
            value.sweep = source.sweep.definition;
            if (bakedTransform
                && !ComposeFrame(source.sweep.definition.constructionFrame,
                                 *bakedTransform, value.sweep.constructionFrame)) return false;
            std::vector<double> encoded;
            if (!SameLocalIDs(source.sweep.definition, value.sweep)
                || !sweep_persistence::Encode(value.sweep, encoded)) return false;
        } else if (source.family == Family::Loft) {
            value.loft = source.loft.definition;
            if (bakedTransform
                && !ComposeFrame(source.loft.definition.constructionFrame,
                                 *bakedTransform, value.loft.constructionFrame)) return false;
            std::vector<double> encoded;
            if (!SameLocalIDs(source.loft.definition, value.loft)
                || !loft_persistence::Encode(value.loft, encoded)) return false;
        } else return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

// Rebuild one already-existing survivor from a freshly captured source. This
// is intentionally not the independent-copy path above: the survivor keeps
// its original feature identifier, recipe record label and every owner-local
// ID. Structural source edits which cannot preserve those IDs refuse rather
// than silently turning the survivor into a new clone.
inline bool PrepareReplacement(const Source& updatedSource,
                               const Source& existingClone,
                               const TopoDS_Shape& destinationShape,
                               const std::optional<gp_Trsf>& bakedTransform,
                               Prepared& output) noexcept {
    output = {};
    try {
        if (updatedSource.owner.IsNull() || existingClone.owner.IsNull()
            || updatedSource.owner.IsEqual(existingClone.owner)
            || destinationShape.IsNull() || destinationShape.ShapeType() != TopAbs_SOLID
            || destinationShape.IsPartner(existingClone.ownerShape)
            || updatedSource.family != existingClone.family
            || (updatedSource.family != Family::Sweep
                && updatedSource.family != Family::Loft)) return false;
        Prepared value; value.family = existingClone.family;
        value.binding = destinationShape;
        if (value.family == Family::Sweep) {
            if (existingClone.sweep.label.IsNull()
                || !SameLocalIDs(updatedSource.sweep.definition,
                                 existingClone.sweep.definition)
                || !profile::IsIdentifier(existingClone.sweep.identifier)) return false;
            value.featureIdentifier = existingClone.sweep.identifier;
            value.sweep = updatedSource.sweep.definition;
            if (bakedTransform
                && !ComposeFrame(updatedSource.sweep.definition.constructionFrame,
                                 *bakedTransform, value.sweep.constructionFrame)) return false;
            std::vector<double> encoded;
            if (!SameLocalIDs(existingClone.sweep.definition, value.sweep)
                || !sweep_persistence::Encode(value.sweep, encoded)) return false;
        } else {
            if (existingClone.loft.label.IsNull()
                || !SameLocalIDs(updatedSource.loft.definition,
                                 existingClone.loft.definition)
                || !profile::IsIdentifier(existingClone.loft.identifier)) return false;
            value.featureIdentifier = existingClone.loft.identifier;
            value.loft = updatedSource.loft.definition;
            if (bakedTransform
                && !ComposeFrame(updatedSource.loft.definition.constructionFrame,
                                 *bakedTransform, value.loft.constructionFrame)) return false;
            std::vector<double> encoded;
            if (!SameLocalIDs(existingClone.loft.definition, value.loft)
                || !loft_persistence::Encode(value.loft, encoded)) return false;
        }
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

inline bool Stage(const Handle(TDocStd_Document)& document,
                  const Source& capturedSource, const TDF_Label& destinationOwner,
                  const std::string& sourceEntityIdentifier,
                  const std::string& sourceDefinitionIdentifier,
                  const std::string& destinationEntityIdentifier,
                  const std::string& destinationDefinitionIdentifier,
                  const Prepared& prepared, Candidate& output) noexcept {
    output = {};
    try {
        Source live, destinationBefore;
        if (document.IsNull() || !document->HasOpenCommand()
            || destinationOwner.IsNull() || destinationOwner.Data() != document->GetData()
            || sourceEntityIdentifier.empty() || sourceDefinitionIdentifier.empty()
            || destinationEntityIdentifier.empty() || destinationDefinitionIdentifier.empty()
            || sourceEntityIdentifier == destinationEntityIdentifier
            || sourceDefinitionIdentifier == destinationDefinitionIdentifier
            || !Capture(document, capturedSource.owner, live)
            || !IsEqual(live, capturedSource) || prepared.family != live.family
            || !Capture(document, destinationOwner, destinationBefore)
            || destinationBefore.family != Family::None
            || !prepared.binding.IsEqual(XCAFDoc_ShapeTool::GetShape(destinationOwner))) return false;
        Candidate candidate; candidate.family = live.family;
        candidate.ownerEntityIdentifier = destinationEntityIdentifier;
        candidate.ownerDefinitionIdentifier = destinationDefinitionIdentifier;
        if (live.family == Family::None) {
            if (!prepared.featureIdentifier.empty()) return false;
        } else if (live.family == Family::Sweep) {
            if (!SameLocalIDs(live.sweep.definition, prepared.sweep)
                || !sweep_persistence::Stage(document, destinationOwner,
                    prepared.sweep, prepared.featureIdentifier)
                || !sweep_persistence::Read(document, destinationOwner, candidate.sweep)
                || candidate.sweep.label.IsNull()
                || candidate.sweep.identifier != prepared.featureIdentifier
                || candidate.sweep.identifier == live.sweep.identifier
                || !candidate.sweep.IsCurrent(document, destinationOwner)) return false;
        } else if (live.family == Family::Loft) {
            if (!SameLocalIDs(live.loft.definition, prepared.loft)
                || !loft_persistence::Stage(document, destinationOwner,
                    prepared.loft, prepared.featureIdentifier)
                || !loft_persistence::Read(document, destinationOwner, candidate.loft)
                || candidate.loft.label.IsNull()
                || candidate.loft.identifier != prepared.featureIdentifier
                || candidate.loft.identifier == live.loft.identifier
                || !candidate.loft.IsCurrent(document, destinationOwner)) return false;
        } else return false;
        Source after;
        if (!Capture(document, capturedSource.owner, after) || !IsEqual(after, capturedSource)) return false;
        output = std::move(candidate); return true;
    } catch (...) { output = {}; return false; }
}

inline bool StageAnalyticBoolean(
    const Handle(TDocStd_Document)& document, const Source& capturedSource,
    const TDF_Label& destinationOwner, const Prepared& prepared,
    Candidate& output) noexcept {
    output = {};
    try {
        Source live;
        if (document.IsNull() || !document->HasOpenCommand()
            || capturedSource.family != Family::AnalyticBoolean
            || prepared.family != Family::AnalyticBoolean
            || !prepared.analyticBoolean || destinationOwner.IsNull()
            || destinationOwner.Data() != document->GetData()
            || !Capture(document, capturedSource.owner, live)
            || !IsEqual(live, capturedSource)
            || composite_recipe::HasRecord(destinationOwner)
            || !prepared.binding.IsEqual(XCAFDoc_ShapeTool::GetShape(destinationOwner))
            || !composite_recipe::Attribute::StageIndependentClone(
                document, destinationOwner, prepared.binding,
                prepared.analyticBoolean)) return false;
        composite_recipe::Record readback;
        part_boolean::AnalyticDefinition analytic;
        Source after;
        if (!IsCurrentAnalyticBoolean(document, destinationOwner, readback, analytic)
            || readback.value->bytes != prepared.analyticBoolean->bytes
            || !Capture(document, capturedSource.owner, after)
            || !IsEqual(after, capturedSource)) return false;
        output.family = Family::AnalyticBoolean;
        output.analyticBooleanBytes = readback.value->bytes;
        output.identities = prepared.identities;
        return true;
    } catch (...) { output = {}; return false; }
}

// Replace the survivor root and retained recipe together in the caller's one
// owned command, reusing its original metadata label, and prove the updated
// source remained exact. Analytic/composite replacement is outside the
// accepted D1 copy policy and therefore has no fallback here.
inline bool StageReplacement(const Handle(TDocStd_Document)& document,
                             const Source& capturedUpdatedSource,
                             const Source& capturedExistingClone,
                             const TDF_Label& destinationOwner,
                             const Prepared& prepared,
                             Candidate& output) noexcept {
    output = {};
    try {
        Source liveSource, liveClone;
        if (document.IsNull() || !document->HasOpenCommand()
            || destinationOwner.IsNull() || destinationOwner.Data() != document->GetData()
            || !destinationOwner.IsEqual(capturedExistingClone.owner)
            || prepared.family != capturedUpdatedSource.family
            || prepared.family != capturedExistingClone.family
            || (prepared.family != Family::Sweep && prepared.family != Family::Loft)
            || !Capture(document, capturedUpdatedSource.owner, liveSource)
            || !IsEqual(liveSource, capturedUpdatedSource)
            || !Capture(document, destinationOwner, liveClone)
            || !IsEqual(liveClone, capturedExistingClone)
            || prepared.binding.IsPartner(liveClone.ownerShape))
            return false;
        Candidate candidate; candidate.family = prepared.family;
        candidate.ownerEntityIdentifier = {};
        candidate.ownerDefinitionIdentifier = {};
        if (prepared.family == Family::Sweep) {
            const TDF_Label recordLabel = liveClone.sweep.label;
            if (prepared.featureIdentifier != liveClone.sweep.identifier
                || !SameLocalIDs(liveClone.sweep.definition, prepared.sweep)
                || !sweep_persistence::Stage(document, destinationOwner,
                    prepared.sweep, liveClone.sweep.identifier)) return false;
            const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
            if (shapes.IsNull()) return false;
            shapes->SetShape(destinationOwner, prepared.binding);
            TNaming_Builder(recordLabel).Select(prepared.binding, prepared.binding);
            if (!sweep_persistence::Read(document, destinationOwner, candidate.sweep)
                || candidate.sweep.label.IsNull()
                || !candidate.sweep.label.IsEqual(recordLabel)
                || candidate.sweep.identifier != liveClone.sweep.identifier
                || !SameLocalIDs(liveClone.sweep.definition,
                                 candidate.sweep.definition)
                || !candidate.sweep.IsCurrent(document, destinationOwner)) return false;
        } else {
            const TDF_Label recordLabel = liveClone.loft.label;
            if (prepared.featureIdentifier != liveClone.loft.identifier
                || !SameLocalIDs(liveClone.loft.definition, prepared.loft)
                || !loft_persistence::Stage(document, destinationOwner,
                    prepared.loft, liveClone.loft.identifier)) return false;
            const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
            if (shapes.IsNull()) return false;
            shapes->SetShape(destinationOwner, prepared.binding);
            TNaming_Builder(recordLabel).Select(prepared.binding, prepared.binding);
            if (!loft_persistence::Read(document, destinationOwner, candidate.loft)
                || candidate.loft.label.IsNull()
                || !candidate.loft.label.IsEqual(recordLabel)
                || candidate.loft.identifier != liveClone.loft.identifier
                || !SameLocalIDs(liveClone.loft.definition,
                                 candidate.loft.definition)
                || !candidate.loft.IsCurrent(document, destinationOwner)) return false;
        }
        Source afterSource;
        if (!Capture(document, capturedUpdatedSource.owner, afterSource)
            || !IsEqual(afterSource, capturedUpdatedSource)) return false;
        output = std::move(candidate); return true;
    } catch (...) { output = {}; return false; }
}

inline bool ReadCandidate(const Handle(TDocStd_Document)& document,
                          const TDF_Label& owner, const Candidate& expected) noexcept {
    try {
        Source actual;
        if (!Capture(document, owner, actual) || actual.family != expected.family) return false;
        if (actual.family == Family::None) return true;
        if (actual.family == Family::Sweep) return actual.sweep.IsEqual(expected.sweep);
        if (actual.family == Family::Loft) return actual.loft.IsEqual(expected.loft);
        if (actual.family == Family::AnalyticBoolean)
            return actual.analyticBoolean.value
                && actual.analyticBoolean.value->bytes == expected.analyticBooleanBytes;
        return false;
    } catch (...) { return false; }
}

} // namespace core3d::pattern_recipe_clone
