#import <Foundation/Foundation.h>

#include "SpatialSweepOwner.hxx"

#include "CompositeRecipeAttribute.hxx"
#include "OcctDocument.h"
#include "SpatialSweepEditor.hxx"
#include "SpatialSweepG0Transaction.hxx"

#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Name.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <set>
#include <vector>

namespace core3d::spatial_sweep::owner {
namespace {
using bounded_curve::Digest;
using bounded_curve::UUID;
using composite_recipe::FeatureNode;
using composite_recipe::Node;
using composite_recipe::OwnerKey;
using composite_recipe::Payload;
using composite_recipe::RecipeKind;
using composite_recipe::SourceNode;

constexpr std::uint32_t kViewportWidth = 64;
constexpr std::uint32_t kViewportHeight = 64;

Receipt Refused(Outcome outcome, const char *reason) noexcept {
    Receipt value;
    value.outcome = outcome;
    value.reason = reason ? reason : "refused";
    return value;
}

bool SameScene(const SceneFence& expected,
               const native_opening::Fence& actual) noexcept {
    return expected.documentGeneration == actual.documentGeneration()
        && expected.modelRevision == actual.modelRevision()
        && expected.metersPerUnit == actual.metersPerUnit();
}

bool UUIDFromText(const std::string& text, UUID& output) noexcept {
    output = {};
    if (!profile::IsIdentifier(text)) return false;
    std::size_t index = 0;
    unsigned nibble = 0;
    for (const char value : text) {
        if (value == '-') continue;
        unsigned digit = 0;
        if (value >= '0' && value <= '9') digit = unsigned(value - '0');
        else if (value >= 'a' && value <= 'f') digit = unsigned(value - 'a' + 10);
        else if (value >= 'A' && value <= 'F') digit = unsigned(value - 'A' + 10);
        else return false;
        if ((nibble++ & 1U) == 0) output[index] = std::uint8_t(digit << 4);
        else output[index++] |= std::uint8_t(digit);
    }
    return index == output.size() && retained_recipe::Nonzero(output);
}

bool MintUUID(UUID& output) noexcept {
    output = {};
    for (unsigned attempt = 0; attempt < 16; ++attempt) {
        if (UUIDFromText(OcctDocument::NewProfileIdentifier(), output)) return true;
    }
    return false;
}

bool HashDomain(const char *domain, const std::vector<std::uint8_t>& bytes,
                Digest& output) noexcept {
    try {
        std::vector<std::uint8_t> input;
        while (*domain) input.push_back(std::uint8_t(*domain++));
        input.insert(input.end(), bytes.begin(), bytes.end());
        return bounded_curve::Hash(input, composite_recipe::MaximumEnvelopeBytes, output);
    } catch (...) { output = {}; return false; }
}

bool SetUUID(const TDF_Label& label, const char *attribute,
             const std::string& value) noexcept {
    try {
        return profile::IsIdentifier(value)
            && !TDataStd_AsciiString::Set(label, Standard_GUID(attribute),
                TCollection_AsciiString(value.c_str())).IsNull();
    } catch (...) { return false; }
}

bool ComputeClosedWitness(const bounded_curve::Definition& curve,
                          Definition& sweep) noexcept {
    if (sweep.closure != ClosureKind::ClosedNoCaps) {
        sweep.witness = {};
        return true;
    }
    try {
        // Compute the creation branch from the same Bishop transport used by
        // the builder. The caller cannot supply or choose this witness.
        sweep.witness.present = true;
        sweep.witness.solverVersion = 1;
        sweep.witness.proofVersion = 1;
        const double mmPerUnit = sweep.dimensionMetersPerUnit * 1000.0;
        const double radiusEnvelope = std::max(sweep.radius.startRadius,
            sweep.radius.endRadius) * mmPerUnit + 4 * PositionalEpsilonMM;
        Handle(Geom_BSplineCurve) geometry = geomfill_detail::Curve(curve, mmPerUnit);
        Handle(GeomAdaptor_Curve) adaptor = new GeomAdaptor_Curve(geometry);
        std::atomic_bool cancelled{false};
        auto table = PrepareBishopTransport(adaptor,
            geomfill_detail::WorldSeed(curve, sweep.orientation.authoredSeed),
            0, 0, FrameTolerance(radiusEnvelope), &cancelled);
        double holonomy = 0;
        if (!table || !Holonomy(table->nodes.front().bishop,
                                table->nodes.back().bishop, holonomy)
            || !std::isfinite(holonomy)) return false;
        sweep.witness.unwrappedHolonomyReference = holonomy;
        sweep.witness.lift = 0;
        return Validate(sweep) == Refusal::None;
    } catch (...) { return false; }
}

} // namespace

struct OcafOwner::State final {
    OcctDocument *document = nullptr;
    std::shared_ptr<native_opening::Context> context;
    bool retired = false;
    bool applying = false;
    std::uint64_t session = 1;

    bool bound() const noexcept {
        try {
            return !retired && document && context && !document->Document().IsNull()
                && context->openingFence().document() == document->Document()
                && context->openingFence().data() == document->Document()->GetData();
        } catch (...) { return false; }
    }
};

OcafOwner::OcafOwner(OcctDocument& document,
                     std::shared_ptr<native_opening::Context> context) noexcept
    : state_(new State()) {
    state_->document = &document;
    state_->context = std::move(context);
    if (!state_->bound()) state_->retired = true;
}

OcafOwner::~OcafOwner() = default;

bool OcafOwner::boundToCurrentDocument() const noexcept {
    return [NSThread isMainThread] && state_ && state_->bound();
}

bool OcafOwner::blocksOtherWork() const noexcept {
    return [NSThread isMainThread] && state_ && state_->bound() && state_->applying;
}

void OcafOwner::retireForDocumentReplacement() noexcept {
    if (!state_) return;
    state_->retired = true;
    state_->context.reset();
    state_->document = nullptr;
}

Receipt OcafOwner::cancel(std::uint64_t session) noexcept {
    if (![NSThread isMainThread] || !state_ || !state_->bound()
        || state_->applying || session != state_->session)
        return Refused(Outcome::refused, "cancel-refused");
    state_->retired = true;
    return Refused(Outcome::cancelled, "cancelled");
}

Receipt OcafOwner::create(const CreateRequest& request) noexcept {
    if (![NSThread isMainThread] || !state_ || !state_->bound())
        return Refused(Outcome::refused, "owner-not-current");
    if (state_->applying) return Refused(Outcome::busy, "owner-busy");
    state_->applying = true;
    const auto finish = [&](Receipt value) {
        state_->applying = false;
        return value;
    };
    try {
        OcctDocument& owner = *state_->document;
        const auto& opening = state_->context->openingFence();
        const Handle(TDocStd_Document) document = owner.Document();
        if (!SameScene(request.expectedScene, opening)
            || !state_->context->isCurrent(kViewportWidth, kViewportHeight)
            || document.IsNull() || document->HasOpenCommand()
            || request.requestedName.empty() || request.requestedName.size() > 128)
            return finish(Refused(Outcome::refused, "creation-fence-stale"));

        bounded_curve::Definition curve = request.curve;
        curve.schema = bounded_curve::Schema;
        curve.domain = bounded_curve::Domain::Path3D;
        curve.frame.revision = 1;
        curve.frame.handedness = bounded_curve::Handedness::Right;
        if (!MintUUID(curve.frame.identifier))
            return finish(Refused(Outcome::refused, "frame-identity-refused"));
        std::set<UUID> internal;
        internal.insert(curve.frame.identifier);
        for (auto& pole : curve.controlPoints) {
            if (!MintUUID(pole.identifier) || !internal.insert(pole.identifier).second)
                return finish(Refused(Outcome::refused, "pole-identity-refused"));
        }
        if (bounded_curve::Validate(curve) != bounded_curve::Refusal::None)
            return finish(Refused(Outcome::refused, "curve-value-refused"));

        std::vector<OcctIssuedLabelIdentity> issued;
        if (!owner.ReserveExactLabelIdentities(4, {}, issued) || issued.size() != 4)
            return finish(Refused(Outcome::busy, "identity-issuance-refused"));
        UUID documentID{}, ownerEntity{}, ownerDefinition{}, sourceNode{}, curveFeature{},
            featureNode{}, sweepFeature{}, section{};
        if (!UUIDFromText(owner.DocumentIdentifier(), documentID)
            || !UUIDFromText(issued[0].EntityIdentifier(), ownerEntity)
            || !UUIDFromText(issued[0].DefinitionIdentifier(), ownerDefinition)
            || !UUIDFromText(issued[1].EntityIdentifier(), sourceNode)
            || !UUIDFromText(issued[1].DefinitionIdentifier(), curveFeature)
            || !UUIDFromText(issued[2].EntityIdentifier(), featureNode)
            || !UUIDFromText(issued[2].DefinitionIdentifier(), sweepFeature)
            || !UUIDFromText(issued[3].EntityIdentifier(), section))
            return finish(Refused(Outcome::refused, "issued-identity-invalid"));

        bounded_curve::Value curveValue;
        curveValue.feature = curveFeature;
        curveValue.definition = curve;
        std::vector<std::uint8_t> curveBytes;
        Digest curveDigest{};
        if (!bounded_curve::Encode(curveValue, curveBytes)
            || !bounded_curve::Hash(curveBytes,
                bounded_curve::MaximumDefinitionBytes, curveDigest))
            return finish(Refused(Outcome::refused, "curve-encoding-refused"));

        Definition sweep = request.sweep;
        sweep.schema = Schema;
        sweep.path.inputNode = sourceNode;
        sweep.path.curveFeature = curveFeature;
        sweep.path.sourceRecipeDigest = curveDigest;
        sweep.path.ownerState.definitionRevision = 1;
        sweep.path.ownerState.nextLocalID = curve.controlPoints.size() + 1;
        sweep.path.ownerState.canonicalDefinitionDigest = curveDigest;
        sweep.path.ownerState.tombstones.clear();
        sweep.dimensionMetersPerUnit = opening.metersPerUnit();
        sweep.section = SectionKind::SolidCircle;
        sweep.sectionIdentifier = section;
        sweep.transport = TransportKind::BishopV1;
        sweep.parameterization = ParameterizationKind::NormalizedArcLengthV1;
        sweep.profile = BuildProfile{};
        if (!ComputeClosedWitness(curve, sweep)
            || Validate(sweep) != Refusal::None)
            return finish(Refused(Outcome::refused, "sweep-value-refused"));
        std::vector<std::uint8_t> featureBytes;
        if (!spatial_sweep::Encode(sweep, featureBytes))
            return finish(Refused(Outcome::refused, "sweep-encoding-refused"));

        TopoDS_Shape sourceWire;
        Digest wireDigest{};
        if (!editor::MakeNativeWire(curve, sourceWire, wireDigest))
            return finish(Refused(Outcome::refused, "source-wire-refused"));
        std::atomic_bool cancelled{false};
        const auto first = BuildGeomFillSweep(curve, sweep, cancelled);
        const auto second = BuildGeomFillSweep(curve, sweep, cancelled);
        if (!first.built() || !second.built())
            return finish(Refused(Outcome::refused, "admission-or-build-refused"));
        const auto fixed = composite_recipe::spatial_g0::PrepareFixedPoint(
            first.solid, second.solid, composite_recipe::spatial_g0::PinnedProfile);
        if (!fixed.admitted())
            return finish(Refused(Outcome::refused, "fixed-point-refused"));

        Digest placement{}, material{}, groups{};
        if (!HashDomain("placement", curveBytes, placement)
            || !HashDomain("material", curveBytes, material)
            || !HashDomain("groups", curveBytes, groups))
            return finish(Refused(Outcome::refused, "commitment-refused"));
        SourceNode source;
        source.node = sourceNode;
        source.localID = 1;
        source.original = {documentID, ownerEntity, ownerDefinition, curveFeature};
        source.recipe = {RecipeKind::BoundedCurvePath,
                         bounded_curve::Schema, curveBytes};
        source.inputToCarrier.sourceMetersPerUnit = opening.metersPerUnit();
        source.inputToCarrier.carrierMetersPerUnit = opening.metersPerUnit();
        source.commitments = {wireDigest, curveDigest, placement, material, groups};
        source.shapeSlot = 0;
        FeatureNode feature;
        feature.node = featureNode;
        feature.feature = sweepFeature;
        feature.localID = 2;
        feature.kind = SpatialCircleSweepFeatureKind;
        feature.codecVersion = SpatialCircleSweepFeatureCodec;
        feature.inputs = {sourceNode};
        feature.parameters = featureBytes;
        auto payload = std::make_shared<Payload>();
        payload->definition.schemaVersion = 2;
        payload->definition.owner = OwnerKey{documentID, ownerEntity, ownerDefinition};
        payload->definition.outputNode = featureNode;
        payload->definition.issuance.nextLocalID = 3;
        payload->definition.nodes = {Node{source}, Node{feature}};
        payload->sourceShapes = {sourceWire};
        if (!composite_recipe::EncodeV2(payload->definition, payload->bytes))
            return finish(Refused(Outcome::refused, "composite-encoding-refused"));

        const int undoBefore = document->GetAvailableUndos();
        auto lease = state_->context->beginCommandLease(opening,
            kViewportWidth, kViewportHeight);
        if (!lease) return finish(Refused(Outcome::busy, "command-refused"));
        const auto abort = [&](const char *reason) {
            const bool closed = lease->abort();
            return finish(Refused(closed ? Outcome::refused
                                        : Outcome::recoveryRequired, reason));
        };
        const Handle(XCAFDoc_LayerTool) layers =
            XCAFDoc_DocumentTool::LayerTool(document->Main());
        if (layers.IsNull()) return abort("layer-tool-refused");
        const Handle(XCAFDoc_ShapeTool) shapes =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return abort("shape-tool-refused");
        const TDF_Label carrier = shapes->AddShape(
            fixed.canonical, Standard_False, Standard_True);
        if (carrier.IsNull()
            || !SetUUID(carrier, "0074F7C2-9EAA-4F89-B2DE-8716E155FF62",
                        issued[0].EntityIdentifier())
            || !SetUUID(carrier, "3611F2B2-C694-4E12-AED8-A2A97A3D283B",
                        issued[0].DefinitionIdentifier()))
            return abort("carrier-identity-staging-refused");
        TDataStd_Integer::Set(carrier,
            Standard_GUID("67E669F4-00C0-4C45-BC55-9CC5DA22A2B5"), 1);
        TDataStd_Name::Set(carrier,
            TCollection_ExtendedString(request.requestedName.c_str()));
        Standard_Integer commandMarker = 0;
        if (!composite_recipe::spatial_g0::Transaction::StageCreation(
                document, carrier, XCAFDoc_ShapeTool::GetShape(carrier), payload,
                commandMarker))
            return abort("composite-staging-refused");
        composite_recipe::Record staged;
        if (!composite_recipe::Read(document, carrier, staged) || !staged.value
            || staged.value->bytes != payload->bytes
            || !staged.current.IsEqual(XCAFDoc_ShapeTool::GetShape(carrier)))
            return abort("staged-readback-refused");

        native_opening::CommittedEditPublication publication;
        publication.created.push_back({issued[0].EntityIdentifier(), {}});
        if (!composite_recipe::spatial_g0::Transaction::OwnsCreation(
                document, commandMarker))
            return abort("command-owner-marker-lost");
        if (!lease->commit()) {
            state_->context->retainUnprovenEdit(publication);
            return finish(Refused(Outcome::outcomeUnknown, "create-close-unknown"));
        }
        composite_recipe::Record readback;
        const int historyDelta = document->GetAvailableUndos() - undoBefore;
        if (document->HasOpenCommand() || historyDelta != 1
            || !composite_recipe::Read(document, carrier, readback)
            || !readback.value || readback.value->bytes != payload->bytes
            || !readback.current.IsEqual(staged.current)
            || !composite_recipe::spatial_g0::Capture(document, carrier).admitted()) {
            state_->context->retainUnprovenEdit(publication);
            return finish(Refused(Outcome::outcomeUnknown,
                                  "create-post-close-unreadable"));
        }
        if (!state_->context->publishCommittedEdit(publication)) {
            state_->context->retainUnprovenEdit(publication);
            return finish(Refused(Outcome::recoveryRequired,
                                  "create-publication-unproven"));
        }
        Receipt result;
        result.outcome = Outcome::committed;
        result.reason = "committed";
        result.entityIdentifier = issued[0].EntityIdentifier();
        result.historyDelta = historyDelta;
        return finish(std::move(result));
    } catch (...) {
        return finish(Refused(Outcome::refused, "create-exception"));
    }
}

} // namespace core3d::spatial_sweep::owner
