#pragma once

// The sole G0 mutation boundary for a spatial sweep. Geometry and recipes are
// prepared detached; every refusal before Apply leaves history untouched.
// Apply re-reads the complete composite/C1/unit/geometry fence, then owns one
// OCAF command from marker acquisition through paired shape+recipe readback.
#include "CompositeRecipeAttribute.hxx"
#include "SpatialSweepGeomFillBuilder.hxx"
#include "SpatialSweepRebuild.hxx"
#include <BinTools.hxx>
#include <BinTools_FormatVersion.hxx>
#include <BRep_Tool.hxx>
#include <TDataStd_Integer.hxx>
#include <TNaming_Builder.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <cfenv>
#include <cstring>
#include <limits>
#include <sstream>
#include <type_traits>

namespace core3d::composite_recipe::spatial_g0 {
using spatial_sweep::GeometryBytes;

inline const Standard_GUID& CommandOwnerAttributeID() {
    static const Standard_GUID id("5C6EA81A-0739-4A77-918B-3D79EAB77DD6");
    return id;
}

struct PinnedGeometryProfile {
    std::uint32_t occtMajor = 7, occtMinor = 8, occtMaintenance = 0;
    std::uint32_t binToolsFormat = 4;
    std::uint32_t algorithm = 1, tolerance = 1, proof = 1, serializer = 1;
    std::uint32_t platform = 1; // Apple arm64
    std::uint32_t binary64 = 1, roundToNearest = 1, contractionDisabled = 1;
};
inline constexpr PinnedGeometryProfile PinnedProfile{};

inline bool SameProfile(const PinnedGeometryProfile& a,
                        const PinnedGeometryProfile& b) noexcept {
    return std::memcmp(&a, &b, sizeof(PinnedGeometryProfile)) == 0;
}

inline bool RuntimeProfileIsPinned(const PinnedGeometryProfile& actual) noexcept {
    return SameProfile(actual, PinnedProfile)
        && std::numeric_limits<double>::is_iec559 && sizeof(double) == 8
        && std::fegetround() == FE_TONEAREST
        && BinTools_FormatVersion_VERSION_4 == BinTools_FormatVersion(4);
}

inline bool Geometry(const TopoDS_Shape& shape, GeometryBytes& output) noexcept {
    output.clear();
    try {
        if (shape.IsNull()) return false;
        std::ostringstream stream(std::ios::binary);
        BinTools::Write(shape, stream, Standard_False, Standard_False,
                        BinTools_FormatVersion_VERSION_4);
        if (!stream.good()) return false;
        const std::string bytes = stream.str();
        if (bytes.empty() || bytes.size() > 64 * 1024 * 1024) return false;
        output.assign(bytes.begin(), bytes.end());
        return true;
    } catch (...) { output.clear(); return false; }
}

inline bool ReadGeometry(const GeometryBytes& bytes, TopoDS_Shape& output) noexcept {
    output.Nullify();
    try {
        if (bytes.empty() || bytes.size() > 64 * 1024 * 1024) return false;
        const std::string storage(bytes.begin(), bytes.end());
        std::istringstream stream(storage, std::ios::binary);
        BinTools::Read(output, stream);
        return stream.good() && !output.IsNull();
    } catch (...) { output.Nullify(); return false; }
}

enum class FixedPointRefusal : std::uint8_t {
    None = 0, PinnedProfileMismatch, EmptyGeometry, FirstReadWrite,
    IndependentBuildMismatch, SecondReadWrite, NonFixedPoint,
};
struct FixedPointResult {
    FixedPointRefusal refusal = FixedPointRefusal::EmptyGeometry;
    TopoDS_Shape canonical;
    GeometryBytes bytes;
    Digest digest{};
    bool admitted() const noexcept { return refusal == FixedPointRefusal::None; }
};

// Exactly one transition S -> C(S) is allowed. Q(C(S)) must already be a
// fixed point; this function never loops until a convenient representation
// appears and never rewrites recipe scalars.
inline FixedPointResult PrepareFixedPoint(const TopoDS_Shape& firstBuild,
                                          const TopoDS_Shape& secondBuild,
                                          const PinnedGeometryProfile& actual) noexcept {
    FixedPointResult result;
    try {
        if (!RuntimeProfileIsPinned(actual)) {
            result.refusal = FixedPointRefusal::PinnedProfileMismatch; return result;
        }
        GeometryBytes firstRaw, secondRaw, firstCanonical, secondCanonical, repeated;
        TopoDS_Shape firstRead, secondRead, repeatedRead;
        if (!Geometry(firstBuild, firstRaw) || !Geometry(secondBuild, secondRaw)) return result;
        if (!ReadGeometry(firstRaw, firstRead) || !Geometry(firstRead, firstCanonical)
            || !ReadGeometry(secondRaw, secondRead) || !Geometry(secondRead, secondCanonical)) {
            result.refusal = FixedPointRefusal::FirstReadWrite; return result;
        }
        if (firstCanonical != secondCanonical) {
            result.refusal = FixedPointRefusal::IndependentBuildMismatch; return result;
        }
        if (!ReadGeometry(firstCanonical, repeatedRead)
            || !Geometry(repeatedRead, repeated)) {
            result.refusal = FixedPointRefusal::SecondReadWrite; return result;
        }
        if (repeated != firstCanonical) {
            result.refusal = FixedPointRefusal::NonFixedPoint; return result;
        }
        if (!bounded_curve::Hash(firstCanonical, 64 * 1024 * 1024, result.digest)) return result;
        result.canonical = firstRead;
        result.bytes = std::move(firstCanonical);
        result.refusal = FixedPointRefusal::None;
        return result;
    } catch (...) { return result; }
}

struct Fence {
    Handle(TDF_Data) data;
    TDF_Label ownerLabel, recordLabel;
    OwnerKey owner;
    UUID outputNode{}, sourceNode{}, curveFeature{}, sweepFeature{}, section{};
    std::uint64_t curveRevision = 0, curveNextLocalID = 0;
    double metersPerUnit = 0;
    std::vector<std::uint8_t> compositeBytes, curveBytes, featureBytes;
    GeometryBytes ownerGeometry, sourceGeometry;
    std::vector<Commitments> sourceCommitments;
    bool descendantsSupported = false;
};

inline bool SameFence(const Fence& a, const Fence& b) noexcept {
    std::uint64_t au = 0, bu = 0;
    std::memcpy(&au, &a.metersPerUnit, sizeof(au));
    std::memcpy(&bu, &b.metersPerUnit, sizeof(bu));
    if (a.data.IsNull() || b.data.IsNull() || a.data != b.data
        || a.ownerLabel.IsNull() || b.ownerLabel.IsNull()
        || !a.ownerLabel.IsEqual(b.ownerLabel) || !a.recordLabel.IsEqual(b.recordLabel)
        || !(a.owner == b.owner) || a.outputNode != b.outputNode
        || a.sourceNode != b.sourceNode || a.curveFeature != b.curveFeature
        || a.sweepFeature != b.sweepFeature || a.section != b.section
        || a.curveRevision != b.curveRevision
        || a.curveNextLocalID != b.curveNextLocalID || au != bu
        || a.compositeBytes != b.compositeBytes || a.curveBytes != b.curveBytes
        || a.featureBytes != b.featureBytes || a.ownerGeometry != b.ownerGeometry
        || a.sourceGeometry != b.sourceGeometry
        || a.sourceCommitments.size() != b.sourceCommitments.size()
        || a.descendantsSupported != b.descendantsSupported) return false;
    for (std::size_t i = 0; i < a.sourceCommitments.size(); ++i) {
        const auto& x = a.sourceCommitments[i]; const auto& y = b.sourceCommitments[i];
        if (x.geometry != y.geometry || x.recipe != y.recipe
            || x.placement != y.placement || x.material != y.material
            || x.groups != y.groups) return false;
    }
    return true;
}

enum class CaptureRefusal : std::uint8_t {
    None = 0, Busy, MissingOwner, MalformedComposite, UnsupportedDescendant,
    BadCurve, BadFeature, ForeignBinding, UnitChanged, GeometryUnavailable,
};
struct CaptureResult {
    CaptureRefusal refusal = CaptureRefusal::MissingOwner;
    Fence fence;
    std::shared_ptr<const Payload> payload;
    spatial_sweep::Definition sweep;
    bounded_curve::Value curve;
    bool admitted() const noexcept { return refusal == CaptureRefusal::None; }
};

inline CaptureResult Capture(const Handle(TDocStd_Document)& document,
                             const TDF_Label& owner) noexcept {
    CaptureResult result;
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || document->HasOpenCommand()) { result.refusal = CaptureRefusal::Busy; return result; }
        Record record;
        if (!Read(document, owner, record) || !record.value) return result;
        const Definition& definition = record.value->definition;
        std::vector<std::uint8_t> exact;
        if (definition.schemaVersion != 2 || !EncodeV2(definition, exact)
            || exact != record.value->bytes) {
            result.refusal = CaptureRefusal::MalformedComposite; return result;
        }
        const SourceNode* source = nullptr; const FeatureNode* sweep = nullptr;
        for (const Node& node : definition.nodes) {
            if (const auto* candidate = std::get_if<SourceNode>(&node.value)) {
                if (source != nullptr || candidate->recipe.kind != RecipeKind::BoundedCurvePath) {
                    result.refusal = CaptureRefusal::UnsupportedDescendant; return result;
                }
                source = candidate;
            } else {
                const auto& featureCandidate = std::get<FeatureNode>(node.value);
                if (featureCandidate.kind == SpatialCircleSweepFeatureKind
                    && featureCandidate.codecVersion == SpatialCircleSweepFeatureCodec
                    && sweep == nullptr) sweep = &featureCandidate;
                else { result.refusal = CaptureRefusal::UnsupportedDescendant; return result; }
            }
        }
        if (source == nullptr || sweep == nullptr || definition.nodes.size() != 2
            || definition.outputNode != sweep->node || sweep->inputs.size() != 1
            || sweep->inputs.front() != source->node) {
            result.refusal = CaptureRefusal::UnsupportedDescendant; return result;
        }
        bounded_curve::Value curve;
        spatial_sweep::Definition feature;
        Digest sourceDigest{};
        if (!bounded_curve::Decode(source->recipe.bytes, curve)
            || !bounded_curve::Hash(source->recipe.bytes,
                bounded_curve::MaximumDefinitionBytes, sourceDigest)) {
            result.refusal = CaptureRefusal::BadCurve; return result;
        }
        if (!spatial_sweep::Decode(sweep->parameters, feature)
            || feature.path.inputNode != source->node
            || feature.path.curveFeature != curve.feature
            || feature.path.curveFeature != source->original.sourceFeature
            || feature.path.sourceRecipeDigest != sourceDigest
            || feature.path.ownerState.canonicalDefinitionDigest != sourceDigest) {
            result.refusal = CaptureRefusal::BadFeature; return result;
        }
        double unit = 0;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document, unit)
            || !std::isfinite(unit) || unit <= 0
            || retained_solid::Bits(unit) != retained_solid::Bits(feature.dimensionMetersPerUnit)
            || definition.owner.document != source->original.document
            || definition.owner.entity != source->original.entity
            || definition.owner.definition != source->original.definition) {
            result.refusal = CaptureRefusal::UnitChanged; return result;
        }
        if (source->shapeSlot >= record.value->sourceShapes.size()) {
            result.refusal = CaptureRefusal::ForeignBinding; return result;
        }
        const TopoDS_Shape& wire = record.value->sourceShapes[source->shapeSlot];
        Fence fence;
        fence.data = document->GetData(); fence.ownerLabel = owner; fence.recordLabel = record.label;
        fence.owner = definition.owner; fence.outputNode = definition.outputNode;
        fence.sourceNode = source->node; fence.curveFeature = curve.feature;
        fence.sweepFeature = sweep->feature; fence.section = feature.sectionIdentifier;
        fence.curveRevision = feature.path.ownerState.definitionRevision;
        fence.curveNextLocalID = feature.path.ownerState.nextLocalID;
        fence.metersPerUnit = unit; fence.compositeBytes = exact;
        fence.curveBytes = source->recipe.bytes; fence.featureBytes = sweep->parameters;
        fence.sourceCommitments.push_back(source->commitments);
        fence.descendantsSupported = true;
        if (!Geometry(record.current, fence.ownerGeometry)
            || !Geometry(wire, fence.sourceGeometry)) {
            result.refusal = CaptureRefusal::GeometryUnavailable; return result;
        }
        result.refusal = CaptureRefusal::None; result.fence = std::move(fence);
        result.payload = record.value; result.sweep = std::move(feature);
        result.curve = std::move(curve); return result;
    } catch (...) { return result; }
}

struct Prepared {
    Fence captured;
    std::shared_ptr<const Payload> payload;
    TopoDS_Shape canonicalSolid;
    GeometryBytes canonicalGeometry;
    Digest geometryDigest{};
    bool admitted() const noexcept {
        return payload && !canonicalSolid.IsNull() && !canonicalGeometry.empty();
    }
};

inline Prepared Prepare(const CaptureResult& captured,
                        const spatial_sweep::GeomFillBuildResult& first,
                        const spatial_sweep::GeomFillBuildResult& second,
                        const std::shared_ptr<const Payload>& payload,
                        const PinnedGeometryProfile& actual) noexcept {
    Prepared result;
    try {
        if (!captured.admitted() || !captured.fence.descendantsSupported
            || !first.built() || !second.built() || !payload) return result;
        std::vector<std::uint8_t> exact;
        if (!EncodeV2(payload->definition, exact) || exact != payload->bytes
            || !(payload->definition.owner == captured.fence.owner)
            || payload->definition.outputNode != captured.fence.outputNode) return result;
        const auto fixed = PrepareFixedPoint(first.solid, second.solid, actual);
        if (!fixed.admitted()) return result;
        result.captured = captured.fence; result.payload = payload;
        result.canonicalSolid = fixed.canonical;
        result.canonicalGeometry = fixed.bytes; result.geometryDigest = fixed.digest;
        return result;
    } catch (...) { return {}; }
}

enum class ApplyOutcome : std::uint8_t {
    Refused = 0, Committed, AbortedExact, OutcomeUnknown,
};
struct ApplyResult {
    ApplyOutcome outcome = ApplyOutcome::Refused;
    bool openedExactlyOneCommand = false;
};

struct Transaction {
    static ApplyResult Apply(const Handle(TDocStd_Document)& document,
                             const Prepared& prepared) noexcept {
        ApplyResult result;
        CaptureResult reread;
        try {
            if (document.IsNull() || !prepared.admitted()
                || document->HasOpenCommand()) return result;
            reread = Capture(document, prepared.captured.ownerLabel);
            if (!reread.admitted() || !SameFence(prepared.captured, reread.fence)) return result;
            bool priorPresent = false; Standard_Integer prior = 0;
            Handle(TDataStd_Integer) priorAttribute;
            priorPresent = document->Main().FindAttribute(CommandOwnerAttributeID(), priorAttribute);
            if (priorPresent) {
                if (priorAttribute.IsNull()) return result;
                prior = priorAttribute->Get();
            }
            const Standard_Integer marker = prior == std::numeric_limits<Standard_Integer>::max()
                ? std::numeric_limits<Standard_Integer>::min() : prior + 1;
            document->NewCommand(); result.openedExactlyOneCommand = true;
            if (!document->HasOpenCommand()) return result;
            const Standard_Integer transaction = document->GetData()->Transaction();
            const Standard_Integer time = document->GetData()->Time();
            TDataStd_Integer::Set(document->Main(), CommandOwnerAttributeID(), marker);
            auto owns = [&]() {
                Handle(TDataStd_Integer) value;
                return document->HasOpenCommand()
                    && document->GetData()->Transaction() == transaction
                    && document->GetData()->Time() == time
                    && document->Main().FindAttribute(CommandOwnerAttributeID(), value)
                    && !value.IsNull() && value->Get() == marker;
            };
            if (!owns()) { result.outcome = ApplyOutcome::OutcomeUnknown; return result; }
            const Handle(XCAFDoc_ShapeTool) shapes =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            Handle(Attribute) attribute;
            if (shapes.IsNull() || !reread.fence.recordLabel.FindAttribute(AttributeID(), attribute)
                || attribute.IsNull()) throw Standard_Failure("spatial G0 stage owner");
            shapes->SetShape(reread.fence.ownerLabel, prepared.canonicalSolid);
            TNaming_Builder(reread.fence.recordLabel).Select(
                prepared.canonicalSolid, prepared.canonicalSolid);
            attribute->Backup(); attribute->value_ = prepared.payload;
            Record stored;
            GeometryBytes storedGeometry;
            if (!Read(document, reread.fence.ownerLabel, stored) || !stored.value
                || stored.value->bytes != prepared.payload->bytes
                || !Geometry(stored.current, storedGeometry)
                || storedGeometry != prepared.canonicalGeometry || !owns())
                throw Standard_Failure("spatial G0 paired readback");
            (void)document->CommitCommand();
            Handle(TDataStd_Integer) closedMarker;
            if (!document->HasOpenCommand()
                && document->Main().FindAttribute(CommandOwnerAttributeID(), closedMarker)
                && !closedMarker.IsNull() && closedMarker->Get() == marker) {
                result.outcome = ApplyOutcome::Committed; return result;
            }
            result.outcome = ApplyOutcome::OutcomeUnknown; return result;
        } catch (...) {
            try {
                Handle(TDataStd_Integer) marker;
                if (document->HasOpenCommand()
                    && document->Main().FindAttribute(CommandOwnerAttributeID(), marker)
                    && !marker.IsNull()) document->AbortCommand();
            } catch (...) { result.outcome = ApplyOutcome::OutcomeUnknown; return result; }
            const CaptureResult restored = Capture(document, prepared.captured.ownerLabel);
            result.outcome = restored.admitted() && SameFence(prepared.captured, restored.fence)
                ? ApplyOutcome::AbortedExact : ApplyOutcome::OutcomeUnknown;
            return result;
        }
    }
};
} // namespace core3d::composite_recipe::spatial_g0
