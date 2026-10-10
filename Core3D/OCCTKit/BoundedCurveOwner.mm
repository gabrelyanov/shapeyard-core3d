#import <Foundation/Foundation.h>
#import <TargetConditionals.h>

#include "BoundedCurveOwner.hxx"
#include "OcctDocument.h"
#include "PathArrayOwnerBridge.hxx"

#include <XCAFDoc_DocumentTool.hxx>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <map>
#include <set>

#if DEBUG
#include <cstdio>
#include "NativeOpeningFactories.hxx"
#include "NativeOpeningSurfaceProbe.hxx"
#include "../UI/Core3DViewer.h"
#include "../Viewport/Core3DSceneSnapshotFactory.hpp"
#if TARGET_OS_IOS
#import "GLView.h"
#endif
#include <BRepPrimAPI_MakeBox.hxx>
#include <Aspect_GraphicDeviceDefinitionError.hxx>
#include <Standard_Failure.hxx>
#include <XCAFDoc_DocumentTool.hxx>

namespace core3d::native_opening {
Core3DBoundedCurveEditingOpening *OpenBoundedCurve(
    const Handle(OcctDocument)&, const std::string&,
    const std::shared_ptr<Context>&) noexcept;
}
#endif

namespace core3d::bounded_curve::owner {
namespace {
constexpr std::uint32_t kFenceWidth = 64;
constexpr std::uint32_t kFenceHeight = 64;

SceneFence SceneOf(const native_opening::Fence& fence) noexcept {
    return {fence.documentGeneration(), fence.modelRevision(), fence.metersPerUnit()};
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
        else if (value >= 'A' && value <= 'F') digit = unsigned(value - 'A' + 10);
        else if (value >= 'a' && value <= 'f') digit = unsigned(value - 'a' + 10);
        else return false;
        if ((nibble++ & 1U) == 0) output[index] = std::uint8_t(digit << 4);
        else output[index++] |= std::uint8_t(digit);
    }
    return index == output.size() && Nonzero(output);
}

Receipt Refusal(const SceneFence& scene, Outcome outcome,
                const char* reason) noexcept {
#if DEBUG
    std::fprintf(stderr, "R179_CURVE_REFUSED outcome=%d reason=%.64s\n",
                 int(outcome), reason ? reason : "");
#endif
    Receipt receipt;
    receipt.outcome = outcome;
    receipt.scene = scene;
    receipt.reason = reason;
    return receipt;
}

bool RehashDefinition(PersistedValue& persisted) noexcept {
    std::vector<std::uint8_t> bytes;
    return Encode(persisted.value, bytes)
        && Hash(bytes, MaximumDefinitionBytes,
                persisted.ownerState.canonicalDefinitionDigest)
        && ValidatePersisted(persisted);
}
} // namespace

struct OcafOwner::State final {
    struct Captured final {
        std::shared_ptr<const Opening> publicValue;
        OcctBoundedCurveCapture exact;
    };
    struct PreparedValue final {
        std::shared_ptr<const Prepared> publicValue;
        OcctBoundedCurveCapture exact;
        path_array_owner::DependentReplayPlan dependents;
    };

    OcctDocument* owner = nullptr;
    std::shared_ptr<native_opening::Context> context;
    bool retired = false;
    std::uint64_t nextSession = 0;
    std::uint64_t nextPreparation = 0;
    std::uint64_t nextIssuance = 0;
    std::map<std::uint64_t, Captured> captures;
    std::map<std::uint64_t, PreparedValue> preparations;
    std::map<std::uint64_t, retained_recipe::OwnerKey> pathIssuances;

    bool bound() const noexcept {
        try {
            if (retired || owner == nullptr || !context) return false;
            const auto& fence = context->openingFence();
            return !owner->Document().IsNull()
                && fence.document() == owner->Document()
                && fence.data() == owner->Document()->GetData();
        } catch (...) { return false; }
    }
};

OcafOwner::OcafOwner(OcctDocument& owner,
                     std::shared_ptr<native_opening::Context> context) noexcept
    : state_(new State()) {
    state_->owner = &owner;
    state_->context = std::move(context);
    if (!state_->bound()) state_->retired = true;
}

OcafOwner::~OcafOwner() = default;

bool OcafOwner::boundToCurrentDocument() const noexcept {
    return [NSThread isMainThread] && state_ && state_->bound();
}

bool OcafOwner::blocksOtherWork() const noexcept {
    return [NSThread isMainThread] && state_ && state_->bound()
        && (!state_->captures.empty() || !state_->preparations.empty());
}

void OcafOwner::retireForDocumentReplacement() noexcept {
    if (!state_) return;
    state_->retired = true;
    state_->captures.clear();
    state_->preparations.clear();
    state_->pathIssuances.clear();
    state_->context.reset();
    state_->owner = nullptr;
}

Receipt OcafOwner::create(const CreateRequest& request) noexcept {
    const SceneFence scene = state_ && state_->context
        ? SceneOf(state_->context->openingFence()) : SceneFence{};
    if (![NSThread isMainThread] || !state_ || !state_->bound())
        return Refusal(scene, Outcome::staleOwner, "owner-retired");
    try {
        const auto& fence = state_->context->openingFence();
        if (!SameScene(request.expectedScene, fence)
            || !ValidatePersisted(request.persisted)
            || !MatchesPersistedValue(request.detachedWire, request.persisted)
            || retained_solid::UUIDText(request.persisted.ownerState.owner.document)
                != state_->owner->DocumentIdentifier())
            return Refusal(scene, Outcome::staleDigest, "create-request-not-current");

        std::vector<OcctIssuedLabelIdentity> issued;
        if (!state_->owner->ReserveExactLabelIdentities(1, {}, issued)
            || issued.size() != 1)
            return Refusal(scene, Outcome::busy, "identity-issuance-refused");
        PersistedValue assigned = request.persisted;
        if (!UUIDFromText(issued.front().EntityIdentifier(),
                          assigned.ownerState.owner.entity)
            || !UUIDFromText(issued.front().DefinitionIdentifier(),
                             assigned.ownerState.owner.definition)
            || !RehashDefinition(assigned))
            return Refusal(scene, Outcome::refused, "issued-identity-invalid");
        DetachedWire rebuilt;
        if (BuildWire(assigned, rebuilt) != BuildRefusal::None
            || !MatchesPersistedValue(rebuilt, assigned)
            || !WireMatchesDefinition(request.persisted,
                                      request.detachedWire.wire))
            return Refusal(scene, Outcome::staleDigest, "independent-rebuild-failed");

        // Publish the natively issued object through the owning viewer only
        // after exact committed readback. A saved label alone is not displayed
        // or selectable by the ordinary Objects selection route.
        native_opening::CommittedEditPublication publication;
        publication.created.push_back({issued.front().EntityIdentifier(), {}});
        const int undoBefore = state_->owner->Document()->GetAvailableUndos();
        auto lease = state_->context->beginCommandLease(
            fence, kFenceWidth, kFenceHeight);
        if (!lease) return Refusal(scene, Outcome::busy, "command-refused");
        OcctBoundedCurveCapture staged;
        if (!state_->owner->StageBoundedCurveCreate(*lease, issued.front(),
                assigned, rebuilt, request.requestedName, staged)) {
            const bool closed = lease->abort();
            return Refusal(scene, closed ? Outcome::refused : Outcome::recoveryRequired,
                           closed ? "create-staging-refused" : "create-abort-unknown");
        }
        if (!lease->commit()) {
            state_->context->retainUnprovenEdit(publication);
            return Refusal(scene, Outcome::outcomeUnknown, "create-close-unknown");
        }
        OcctBoundedCurveCapture read;
        const int historyDelta = state_->owner->Document()->GetAvailableUndos() - undoBefore;
        if (historyDelta != 1
            || !state_->owner->ReadBoundedCurveExact(
                assigned.ownerState.owner, read)
            || !staged.IsEqual(read))
            return Refusal(scene, Outcome::outcomeUnknown, "create-post-close-unreadable");
        RetainedState retained;
        Receipt receipt;
        if (!FromPersistedValue(assigned, retained))
            return Refusal(scene, Outcome::outcomeUnknown, "create-retained-read-failed");
        if (!state_->context->publishCommittedEdit(publication))
            return Refusal(scene, Outcome::recoveryRequired, "create-publication-unproven");
        receipt.outcome = Outcome::committed;
        receipt.reason = "committed";
        receipt.scene = scene;
        receipt.authority = retained.authority;
        receipt.historyDelta = historyDelta;
        return receipt;
    } catch (...) { return Refusal(scene, Outcome::refused, "create-exception"); }
}

std::shared_ptr<const Opening> OcafOwner::capture(
    const std::string& entityIdentifier,
    const SceneFence& expectedScene) noexcept {
    if (![NSThread isMainThread] || !state_ || !state_->bound()
        || !SameScene(expectedScene, state_->context->openingFence())
        || !state_->preparations.empty() || state_->nextSession == UINT64_MAX) {
#if DEBUG
        std::fprintf(stderr, "R179_CURVE_REFUSED stage=capture reason=precondition\n");
#endif
        return {};
    }
    try {
        OcctBoundedCurveCapture exact;
        if (!state_->owner->CaptureBoundedCurveExact(
                entityIdentifier, *state_->context, exact)) {
#if DEBUG
            std::fprintf(stderr, "R179_CURVE_REFUSED stage=capture reason=capture-exact-refused\n");
#endif
            return {};
        }
        RetainedState retained;
        if (!FromPersistedValue(exact.persisted, retained)) {
#if DEBUG
            std::fprintf(stderr, "R179_CURVE_REFUSED stage=capture reason=retained-read-refused\n");
#endif
            return {};
        }
        auto opening = std::make_shared<Opening>();
        opening->scene = expectedScene;
        opening->ownerLabel = exact.record.owner;
        opening->retained = std::move(retained);
        opening->wire = exact.wire;
        opening->session = ++state_->nextSession;
        state_->captures.emplace(opening->session,
            State::Captured{opening, std::move(exact)});
        return opening;
    } catch (...) {
#if DEBUG
        std::fprintf(stderr, "R179_CURVE_REFUSED stage=capture reason=capture-exception\n");
#endif
        return {};
    }
}

std::shared_ptr<const Prepared> OcafOwner::prepare(
    const std::shared_ptr<const Opening>& opening,
    const Candidate& candidate, Receipt& receipt) noexcept {
    receipt = Refusal(opening ? opening->scene : SceneFence{},
                      Outcome::refused, "prepare-refused");
    if (![NSThread isMainThread] || !state_ || !state_->bound() || !opening
        || state_->nextPreparation == UINT64_MAX) {
#if DEBUG
        std::fprintf(stderr, "R179_CURVE_REFUSED stage=prepare reason=precondition\n");
#endif
        return {};
    }
    try {
        const auto found = state_->captures.find(opening->session);
        if (found == state_->captures.end()
            || found->second.publicValue != opening) {
#if DEBUG
            std::fprintf(stderr, "R179_CURVE_REFUSED stage=prepare reason=capture-not-current\n");
#endif
            return {};
        }
        OcctBoundedCurveCapture current;
        if (!state_->owner->ReadBoundedCurveExact(
                found->second.exact.persisted.ownerState.owner, current)
            || !found->second.exact.IsEqual(current)) {
            receipt.outcome = Outcome::staleOwner;
            receipt.reason = "capture-no-longer-current";
#if DEBUG
            std::fprintf(stderr, "R179_CURVE_REFUSED stage=prepare reason=capture-no-longer-current\n");
#endif
            return {};
        }

        EditProposal normalized = candidate.proposal;
        normalized.replacementRecipeDigest = {};
        normalized.replacementRecipeDigest[0] = 1;
        if (normalized.replacementRecipeDigest
                == opening->retained.authority.recipeDigest)
            normalized.replacementRecipeDigest[1] = 1;
        PreparedEdit provisional = PrepareEdit(opening->retained, normalized);
        if (provisional.refusal != EditRefusal::None
            || !SameDefinition(provisional.candidate.definition,
                               candidate.completeDefinition)) {
#if DEBUG
            std::fprintf(stderr, "R179_CURVE_REFUSED stage=prepare reason=proposal-refused\n");
#endif
            return {};
        }
        Value replacement;
        replacement.feature = provisional.candidate.authority.feature;
        replacement.definition = provisional.candidate.definition;
        std::vector<std::uint8_t> canonical;
        if (!Encode(replacement, canonical)
            || !Hash(canonical, MaximumDefinitionBytes,
                     normalized.replacementRecipeDigest)) {
#if DEBUG
            std::fprintf(stderr, "R179_CURVE_REFUSED stage=prepare reason=digest-refused\n");
#endif
            return {};
        }
        const PreparedEdit values = PrepareEdit(opening->retained, normalized);
        PersistedValue persisted;
        if (values.refusal != EditRefusal::None
            || !SameDefinition(values.candidate.definition,
                               candidate.completeDefinition)
            || !ToPersistedValue(values.candidate, persisted)) {
#if DEBUG
            std::fprintf(stderr, "R179_CURVE_REFUSED stage=prepare reason=persist-refused\n");
#endif
            return {};
        }
        DetachedWire detached;
        if (BuildWire(persisted, detached) != BuildRefusal::None
            || !MatchesPersistedValue(detached, persisted)) {
#if DEBUG
            std::fprintf(stderr, "R179_CURVE_REFUSED stage=prepare reason=wire-refused\n");
#endif
            return {};
        }
        auto prepared = std::make_shared<Prepared>();
        prepared->opening = *opening;
        prepared->values = values;
        prepared->persisted = std::move(persisted);
        prepared->detachedWire = std::move(detached);
        prepared->preparation = ++state_->nextPreparation;
        path_array_owner::DependentReplayPlan dependents;
        if (!path_array_owner::PrepareDependentReplay(*state_->owner,
                state_->context, prepared, dependents)) {
            receipt.outcome = Outcome::unsupportedDependent;
            receipt.reason = "dependent-path-array-prepare-refused";
#if DEBUG
            std::fprintf(stderr, "R179_CURVE_REFUSED stage=prepare reason=dependent-replay-refused\n");
#endif
            return {};
        }
        state_->preparations.emplace(prepared->preparation,
            State::PreparedValue{prepared, found->second.exact,
                                 std::move(dependents)});
        receipt.outcome = Outcome::prepared;
        receipt.reason = "prepared";
        receipt.scene = opening->scene;
        receipt.authority = values.candidate.authority;
        receipt.session = opening->session;
        receipt.preparation = prepared->preparation;
        return prepared;
    } catch (...) {
        receipt.reason = "prepare-exception";
#if DEBUG
        std::fprintf(stderr, "R179_CURVE_REFUSED stage=prepare reason=prepare-exception\n");
#endif
        return {};
    }
}

Receipt OcafOwner::apply(
    const std::shared_ptr<const Prepared>& prepared) noexcept {
    const SceneFence scene = prepared ? prepared->opening.scene : SceneFence{};
    if (![NSThread isMainThread] || !state_ || !state_->bound() || !prepared)
        return Refusal(scene, Outcome::refused, "apply-refused");
    try {
        const auto found = state_->preparations.find(prepared->preparation);
        if (found == state_->preparations.end()
            || found->second.publicValue != prepared
            || !SameScene(scene, state_->context->openingFence()))
            return Refusal(scene, Outcome::staleOwner, "preparation-not-current");
        OcctBoundedCurveCapture before;
        if (!state_->owner->ReadBoundedCurveExact(
                found->second.exact.persisted.ownerState.owner, before)
            || !found->second.exact.IsEqual(before))
            return Refusal(scene, Outcome::staleDefinition, "opening-changed");
        // Freeze all affected native identities before opening the one command.
        // A replacement must reconcile the selected AIS wire as well as OCAF.
        native_opening::CommittedEditPublication publication;
        // previousShape is the validated exact capture's record.current, proven
        // IsEqual to the owner label's admitted shape by the gate above.
        publication.replaced.push_back({retained_solid::UUIDText(
            found->second.exact.persisted.ownerState.owner.entity),
            found->second.exact.record.current});
        if (!path_array_owner::AppendDependentReplayPublication(
                found->second.dependents, publication))
            return Refusal(scene, Outcome::refused, "publication-plan-refused");
        const int undoBefore = state_->owner->Document()->GetAvailableUndos();
        auto lease = state_->context->beginCommandLease(
            state_->context->openingFence(), kFenceWidth, kFenceHeight);
        if (!lease) return Refusal(scene, Outcome::busy, "command-refused");
        OcctBoundedCurveCapture staged;
        if (!state_->owner->StageBoundedCurveReplacement(*lease,
                found->second.exact, prepared->persisted,
                prepared->detachedWire, staged)
            || !path_array_owner::StageDependentReplayInsideOwnedCommand(
                *state_->owner, *lease, found->second.dependents)) {
            const bool aborted = lease->abort();
            OcctBoundedCurveCapture restored;
            const bool exact = aborted && state_->owner->ReadBoundedCurveExact(
                found->second.exact.persisted.ownerState.owner, restored)
                && found->second.exact.IsEqual(restored)
                && state_->owner->Document()->GetAvailableUndos() == undoBefore;
            return Refusal(scene, exact ? Outcome::refused : Outcome::recoveryRequired,
                           exact ? "stage-failed-state-restored" : "abort-proof-failed");
        }
        if (!lease->commit()) {
            state_->context->retainUnprovenEdit(publication);
            return Refusal(scene, Outcome::outcomeUnknown, "close-unknown");
        }
        OcctBoundedCurveCapture after;
        const int historyDelta = state_->owner->Document()->GetAvailableUndos() - undoBefore;
        if (historyDelta != 1
            || !state_->owner->ReadBoundedCurveExact(
                prepared->persisted.ownerState.owner, after)
            || !staged.IsEqual(after)
            || !path_array_owner::VerifyDependentReplayAfterCommit(
                *state_->owner, found->second.dependents))
            return Refusal(scene, Outcome::outcomeUnknown, "post-close-proof-failed");
        if (!state_->context->publishCommittedEdit(publication))
            return Refusal(scene, Outcome::recoveryRequired, "publication-unproven");
        Receipt receipt;
        receipt.outcome = Outcome::committed;
        receipt.reason = "committed";
        receipt.scene = scene;
        receipt.authority = prepared->values.candidate.authority;
        receipt.session = prepared->opening.session;
        receipt.preparation = prepared->preparation;
        receipt.historyDelta = historyDelta;
        state_->preparations.erase(found);
        state_->captures.erase(prepared->opening.session);
        return receipt;
    } catch (...) { return Refusal(scene, Outcome::refused, "apply-exception"); }
}

Receipt OcafOwner::cancel(std::uint64_t session) noexcept {
    SceneFence scene{};
    if (![NSThread isMainThread] || !state_ || !state_->bound())
        return Refusal(scene, Outcome::staleOwner, "owner-retired");
    for (auto iterator = state_->preparations.begin();
         iterator != state_->preparations.end();) {
        if (iterator->second.publicValue->opening.session == session)
            iterator = state_->preparations.erase(iterator);
        else ++iterator;
    }
    const auto opening = state_->captures.find(session);
    if (opening == state_->captures.end())
        return Refusal(scene, Outcome::refused, "unknown-session");
    scene = opening->second.publicValue->scene;
    state_->captures.erase(opening);
    Receipt receipt = Refusal(scene, Outcome::cancelled, "cancelled");
    receipt.session = session;
    return receipt;
}

std::shared_ptr<const PathReceipt> OcafOwner::pickCurrentPath3D(
    const std::string& entityIdentifier,
    const SceneFence& expectedScene) const noexcept {
    if (![NSThread isMainThread] || !state_ || !state_->bound()
        || !SameScene(expectedScene, state_->context->openingFence())
        || state_->nextIssuance == UINT64_MAX) return {};
    try {
        OcctBoundedCurveCapture exact;
        if (!state_->owner->CaptureBoundedCurveExact(
                entityIdentifier, *state_->context, exact)
            || exact.persisted.value.definition.domain != Domain::Path3D)
            return {};
        RetainedState retained;
        if (!FromPersistedValue(exact.persisted, retained)) return {};
        auto receipt = std::make_shared<PathReceipt>();
        receipt->scene = expectedScene;
        receipt->owner = retained.authority.owner;
        receipt->feature = retained.authority.feature;
        receipt->definitionRevision = retained.authority.definitionRevision;
        receipt->frame = retained.authority.frame;
        receipt->frameRevision = retained.authority.frameRevision;
        receipt->nextLocalID = retained.nextLocalID;
        receipt->canonicalDefinitionDigest = retained.authority.recipeDigest;
        receipt->issuance = ++state_->nextIssuance;
        if (!ValidPathReceipt(*receipt)) return {};
        state_->pathIssuances.emplace(receipt->issuance, receipt->owner);
        return receipt;
    } catch (...) { return {}; }
}

} // namespace core3d::bounded_curve::owner

#if DEBUG
namespace core3d::native_opening::debug {
namespace {
using namespace bounded_curve;
using namespace bounded_curve::owner;

#if TARGET_OS_IOS
struct OwnerProbeContextRestorer final {
    __strong EAGLContext *context = nil;
    ~OwnerProbeContextRestorer() noexcept {
        (void)[EAGLContext setCurrentContext:context];
    }
};
#endif

struct OwnerProbeFixture final {
    Core3DViewer viewer;
    Handle(OcctDocument) document;
    Handle(AIS_Shape) sourcePresentation;
    std::string sourceEntity;
    bool injectSetupFailure = false;
    bool injectionReached = false;
    bool setupFailureRecognized = false;
    bool setupCommandSettled = false;
    bool cleanupSucceeded = true;
#if TARGET_OS_IOS
    __strong GLView *host = nil;
#endif

    ~OwnerProbeFixture() noexcept { shutdown(); }

    bool perform(void (^work)(void)) noexcept {
#if TARGET_OS_IOS
        OwnerProbeContextRestorer restore{[EAGLContext currentContext]};
        try {
            return host != nil && work != nil
                && [host debugPerformWithProbeFramebuffer:work];
        } catch (const Standard_Failure& failure) {
            NSLog(@"Owner probe wrapper failed: %.256s",
                  failure.GetMessageString() ?: "Standard_Failure");
            return false;
        } catch (...) { return false; }
#else
        (void)work;
        return false;
#endif
    }

    void shutdown() noexcept {
#if TARGET_OS_IOS
        if (host == nil) return;
        OwnerProbeContextRestorer restore{[EAGLContext currentContext]};
        OwnerProbeFixture *fixture = this;
        __block bool cleaned = false;
        void (^cleanup)(void) = ^{
            try {
                if (!fixture->document.IsNull()
                    && !fixture->document->Document().IsNull()
                    && fixture->document->Document()->HasOpenCommand())
                    fixture->document->Document()->AbortCommand();
                fixture->viewer.release();
                cleaned = true;
            } catch (const Standard_Failure& failure) {
                NSLog(@"Owner probe cleanup failed: %.256s",
                      failure.GetMessageString() ?: "Standard_Failure");
            } catch (...) {}
        };
        bool performed = perform(cleanup);
        if (!performed && !cleaned) {
            try { performed = [host performWithRenderingContext:cleanup]; }
            catch (...) { performed = false; }
        }
        cleanupSucceeded = cleanupSucceeded && performed && cleaned;
        sourcePresentation.Nullify();
        document.Nullify();
        host = nil;
#endif
    }

    bool initialize(double metersPerUnit,
                    bool shouldInjectSetupFailure = false) noexcept {
#if !TARGET_OS_IOS
        (void)metersPerUnit;
        return false;
#else
        try {
            OwnerProbeContextRestorer restore{[EAGLContext currentContext]};
            injectSetupFailure = shouldInjectSetupFailure;
            injectionReached = false;
            setupFailureRecognized = false;
            setupCommandSettled = false;
            host = [[GLView alloc] initWithFrame:
                CGRectMake(0.0, 0.0, 64.0, 64.0)];
            if (host == nil || host->myGLContext == nil
                || ![host debugPrepareProbeFramebuffer]) {
                host = nil;
                return false;
            }
            if (![EAGLContext setCurrentContext:restore.context]) {
                shutdown();
                return false;
            }

            __block bool initialized = false;
            OwnerProbeFixture *fixture = this;
            const bool performed = perform(^{
                if (!fixture->viewer.InitViewer(fixture->host)
                    || fixture->viewer.getObjectInteractor() == nullptr
                    || fixture->viewer.getShapeInteractor() == nullptr
                    || fixture->viewer.AisContext().IsNull()
                    || fixture->viewer.ActiveView().IsNull()) return;
                fixture->document = fixture->viewer.getDocument();
                if (fixture->document.IsNull()
                    || fixture->document->Document().IsNull()
                    || fixture->document->Document()->GetData().IsNull()
                    || fixture->document->Document()->HasOpenCommand()) return;
                XCAFDoc_DocumentTool::SetLengthUnit(
                    fixture->document->Document(), metersPerUnit);
                fixture->document->Document()->NewCommand();
                if (fixture->injectSetupFailure) {
                    fixture->injectionReached = true;
                    throw Aspect_GraphicDeviceDefinitionError(
                        "Injected owner probe setup failure");
                }
                fixture->sourcePresentation = new AIS_Shape(
                    BRepPrimAPI_MakeBox(8.0, 9.0, 10.0).Shape());
                const TDF_Label label = fixture->document->AddShape(
                    fixture->sourcePresentation,
                    OcctGeometryRepresentation::BRep);
                if (label.IsNull()) {
                    fixture->document->Document()->AbortCommand();
                    return;
                }
                fixture->sourceEntity =
                    fixture->document->EntityIdentifierForLabel(label);
                if (fixture->sourceEntity.empty()) {
                    fixture->document->Document()->AbortCommand();
                    return;
                }
                if (!fixture->document->Document()->CommitCommand()) return;
                fixture->document->Document()->ClearUndos();
                fixture->document->NotifyChanges();

                const auto ais = fixture->viewer.AisContext();
                ais->Display(fixture->sourcePresentation,
                    AIS_Shaded, 0, Standard_False);
                const auto shapeInteractor =
                    fixture->viewer.getShapeInteractor();
                if (!ais->IsDisplayed(fixture->sourcePresentation)
                    || shapeInteractor->setSelectionMode(
                        core3d::ShapeSelectionMode::WholeShape)
                        != core3d::ShapeSelectionModeChangeResult::Succeeded
                    || !shapeInteractor->selectionModeAuthorityIsExact()) return;
                ais->UpdateCurrentViewer();
                fixture->viewer.ActiveView()->FitAll();

                const auto snapshot =
                    fixture->viewer.captureSceneSnapshot(64, 64);
                if (!snapshot
                    || snapshot->publicationSourceIdentifier.empty()
                    || snapshot->metersPerUnit != metersPerUnit
                    || std::none_of(snapshot->instances.begin(),
                        snapshot->instances.end(),
                        [&](const core3d::scene::InstanceSnapshot& instance) {
                            return instance.entityIdentifier
                                == fixture->sourceEntity;
                        })
                    || fixture->document->Document()->HasOpenCommand()
                    || fixture->document->Document()->GetAvailableUndos() != 0
                    || fixture->document->Document()->GetAvailableRedos() != 0)
                    return;
                initialized = true;
            });
            if (!performed || !initialized) {
                setupFailureRecognized = injectSetupFailure && injectionReached
                    && !performed;
                OwnerProbeFixture *failedFixture = this;
                if (!perform(^{
                    if (!failedFixture->document.IsNull()
                        && !failedFixture->document->Document().IsNull()
                        && failedFixture->document->Document()->HasOpenCommand()) {
                        failedFixture->document->Document()->AbortCommand();
                    }
                    failedFixture->setupCommandSettled =
                        !failedFixture->document.IsNull()
                        && !failedFixture->document->Document().IsNull()
                        && !failedFixture->document->Document()->HasOpenCommand();
                })) cleanupSucceeded = false;
                return false;
            }
            return true;
        } catch (const Standard_Failure& failure) {
            NSLog(@"Owner probe initialization failed: %.256s",
                  failure.GetMessageString() ?: "Standard_Failure");
            try {
                OwnerProbeFixture *failedFixture = this;
                (void)perform(^{
                    if (!failedFixture->document.IsNull()
                        && !failedFixture->document->Document().IsNull()
                        && failedFixture->document->Document()->HasOpenCommand())
                        failedFixture->document->Document()->AbortCommand();
                });
            } catch (...) {}
            return false;
        } catch (...) { return false; }
#endif
    }

    std::shared_ptr<native_opening::Context> context(
        const std::string& entity = {}) noexcept {
        __block std::shared_ptr<native_opening::Context> result;
        OwnerProbeFixture *fixture = this;
        const std::string requested = entity.empty() ? sourceEntity : entity;
        if (!perform(^{
            result = fixture->viewer.captureNativeOpeningContext(
                64, 64, {requested});
        })) return {};
        return result;
    }

    bool request(const std::shared_ptr<native_opening::Context>& context,
                 CreateRequest& output) {
        if (!context) return false;
        output = {};
        output.expectedScene = {context->openingFence().documentGeneration(),
            context->openingFence().modelRevision(),
            context->openingFence().metersPerUnit()};
        output.persisted = wire_probe::Fixture();
        if (!UUIDFromText(document->DocumentIdentifier(),
                          output.persisted.ownerState.owner.document)
            || !RehashDefinition(output.persisted)
            || BuildWire(output.persisted, output.detachedWire)
                != BuildRefusal::None) return false;
        output.requestedName = "R179 C1 native owner";
        return true;
    }
};

bool CreateOne(OwnerProbeFixture& fixture, Receipt& receipt,
               retained_recipe::OwnerKey& key) noexcept {
    __block bool created = false;
    OwnerProbeFixture *fixturePointer = &fixture;
    Receipt *receiptPointer = &receipt;
    retained_recipe::OwnerKey *keyPointer = &key;
    if (!fixture.perform(^{
        const auto context = fixturePointer->context();
        if (!context) {
            NSLog(@"Bounded-curve owner probe create failed: phase=context");
            return;
        }
        CreateRequest request;
        if (!fixturePointer->request(context, request)) {
            NSLog(@"Bounded-curve owner probe create failed: phase=request");
            return;
        }
        OcafOwner owner(*fixturePointer->document, context);
        *receiptPointer = owner.create(request);
        *keyPointer = receiptPointer->authority.owner;
        created = receiptPointer->outcome == Outcome::committed
            && receiptPointer->historyDelta == 1
            && retained_recipe::Valid(*keyPointer);
        if (!created) {
            NSLog(@"Bounded-curve owner probe create failed: phase=create outcome=%u reason=%.128s historyDelta=%d",
                  static_cast<unsigned>(receiptPointer->outcome),
                  receiptPointer->reason.empty()
                      ? "unspecified" : receiptPointer->reason.c_str(),
                  receiptPointer->historyDelta);
        }
    })) {
        NSLog(@"Bounded-curve owner probe create failed: phase=perform");
        return false;
    }
    return created;
}

struct OwnerProbeLeaseGuard final {
    OwnerProbeFixture *fixture = nullptr;
    std::shared_ptr<native_opening::CommandLease> lease;
    ~OwnerProbeLeaseGuard() noexcept {
        if (fixture == nullptr || !lease || !lease->ownsOpenCommand()) return;
        OwnerProbeLeaseGuard *guard = this;
        (void)fixture->perform(^{
            if (guard->lease && guard->lease->ownsOpenCommand())
                (void)guard->lease->abort();
        });
    }
};

std::uint64_t RunOwnerProbeScenario0(OwnerProbeFixture& fixture) {
    std::uint64_t bits = 1ULL;
    Receipt receipt; retained_recipe::OwnerKey key;
    if (!CreateOne(fixture, receipt, key)) return bits;
    bits |= 2ULL;
    if (fixture.document->Document()->GetAvailableUndos() == 1) bits |= 4ULL;
    OcctBoundedCurveCapture read;
    if (fixture.document->ReadBoundedCurveExact(key, read)) bits |= 8ULL;
    if (!read.record.current.IsNull()
        && read.record.current.IsEqual(
            XCAFDoc_ShapeTool::GetShape(read.record.owner))) bits |= 16ULL;
    return bits;
}

std::uint64_t RunOwnerProbeScenario1(
    OwnerProbeFixture& fixture, OwnerProbeFixture& foreign) {
    std::uint64_t bits = 1ULL;
    auto context = fixture.context();
    CreateRequest request;
    if (!fixture.request(context, request)) return bits;
    request.persisted.ownerState.canonicalDefinitionDigest[0] ^= 0xff;
    const int history = fixture.document->Document()->GetAvailableUndos();
    OcafOwner owner(*fixture.document, context);
    if (owner.create(request).outcome != Outcome::committed) bits |= 2ULL;
    if (fixture.document->Document()->GetAvailableUndos() == history) bits |= 4ULL;

    __block bool foreignRefused = false;
    __block bool foreignContextPresent = false;
    OwnerProbeFixture *foreignPointer = &foreign;
    const std::string requested = fixture.sourceEntity;
    if (!foreign.perform(^{
        auto foreignContext = foreignPointer->context();
        if (!foreignContext) return;
        foreignContextPresent = true;
        OcafOwner foreignOwner(*foreignPointer->document, foreignContext);
        foreignRefused = !foreignOwner.pickCurrentPath3D(requested,
            {foreignContext->openingFence().documentGeneration(),
             foreignContext->openingFence().modelRevision(),
             foreignContext->openingFence().metersPerUnit()});
    })) return bits;
    if (!foreignContextPresent) return bits;
    if (foreignRefused) bits |= 8ULL;
    if (request.detachedWire.definitionDigest
        != request.persisted.ownerState.canonicalDefinitionDigest) bits |= 16ULL;
    return bits;
}

std::uint64_t RunOwnerProbeScenario2(OwnerProbeFixture& fixture) {
    std::uint64_t bits = 1ULL;
    Receipt receipt; retained_recipe::OwnerKey key;
    if (!CreateOne(fixture, receipt, key)) return bits;
    bits |= 2ULL;
    OcctBoundedCurveCapture before;
    if (!fixture.document->ReadBoundedCurveExact(key, before)) return bits;
    const int history = fixture.document->Document()->GetAvailableUndos();
    fixture.document->NotifyChanges();
    auto context = fixture.context(retained_solid::UUIDText(key.entity));
    if (!context) return bits;
    auto lease = context->beginCommandLease(
        context->openingFence(), 64, 64);
    if (!lease) return bits;
    OwnerProbeLeaseGuard settle{&fixture, lease};
    DetachedWire forged;
    if (BuildWire(before.persisted, forged) != BuildRefusal::None) return bits;
    forged.geometryCommitment[0] ^= 0xff;
    OcctBoundedCurveCapture rejected;
    if (fixture.document->StageBoundedCurveReplacement(*lease, before,
            before.persisted, forged, rejected)) return bits;
    if (!lease->abort()) return bits;
    bits |= 4ULL;
    OcctBoundedCurveCapture after;
    if (fixture.document->ReadBoundedCurveExact(key, after)
        && before.IsEqual(after)) bits |= 8ULL;
    if (fixture.document->Document()->GetAvailableUndos() == history
        && !fixture.document->Document()->HasOpenCommand()) bits |= 16ULL;
    return bits;
}

struct OwnerProbeColdSavedState final {
    std::string path;
    std::string documentIdentifier;
    retained_recipe::OwnerKey key;
    std::vector<std::uint8_t> definitionBytes;
    std::vector<std::uint8_t> ownerBytes;
    std::uint64_t definitionRevision = 0;
    std::array<std::uint8_t, 16> writerInstanceNonce{};
};

bool RunOwnerProbeColdSave(OwnerProbeFixture& fixture,
                           OwnerProbeColdSavedState& state,
                           bool& created,
                           double unit) {
    state = {};
    created = false;
    Receipt receipt; retained_recipe::OwnerKey key;
    if (!CreateOne(fixture, receipt, key)) return false;
    created = true;
    NSString* filename = [NSString stringWithFormat:
        @"r179-c1-owner-%@.cbf", NSUUID.UUID.UUIDString];
    NSString* temporary = [NSTemporaryDirectory()
        stringByAppendingPathComponent:filename];
    __block bool saved = false;
    OwnerProbeFixture *fixturePointer = &fixture;
    OwnerProbeColdSavedState *statePointer = &state;
    if (!fixture.perform(^{
        OcctBoundedCurveCapture exact;
        const auto& nativeDocument = fixturePointer->document->Document();
        const auto& data = nativeDocument->GetData();
        const auto application = nativeDocument->Application();
        if (nativeDocument.IsNull() || data.IsNull() || application.IsNull()
            || !fixturePointer->document->ReadBoundedCurveExact(key, exact)) {
            NSLog(@"Bounded-curve cold probe failed: phase=writer-exact-read");
            return;
        }
        const auto stamp = fixturePointer->document->DebugNativeMutationStamp();
        if (!stamp || std::none_of(stamp->instanceNonce.begin(),
                                  stamp->instanceNonce.end(),
                                  [](std::uint8_t byte) { return byte != 0; })) {
            NSLog(@"Bounded-curve cold probe failed: phase=writer-stamp unit=%.17g reason=missing-stamp",
                  unit);
            return;
        }
        statePointer->path = fixturePointer->document->save(
            temporary.UTF8String);
        if (statePointer->path.empty()) {
            NSLog(@"Bounded-curve cold probe failed: phase=save");
            return;
        }
        statePointer->documentIdentifier =
            fixturePointer->document->DocumentIdentifier();
        statePointer->key = key;
        statePointer->definitionBytes = exact.record.value->definitionBytes;
        statePointer->ownerBytes = exact.record.value->ownerBytes;
        statePointer->definitionRevision =
            exact.persisted.ownerState.definitionRevision;
        statePointer->writerInstanceNonce = stamp->instanceNonce;
        saved = true;
    })) {
        NSLog(@"Bounded-curve cold probe failed: phase=save-framebuffer");
        return false;
    }
    return saved;
}

bool RunOwnerProbeColdOpenAndEdit(OwnerProbeFixture& fixture, double unit,
                                  const OwnerProbeColdSavedState& saved) {
    const auto initialDocument = fixture.document->Document();
    const auto initialData = initialDocument.IsNull()
        ? Handle(TDF_Data)() : initialDocument->GetData();
    const auto initialApplication = initialDocument.IsNull()
        ? Handle(CDM_Application)() : initialDocument->Application();
    if (initialDocument.IsNull() || initialData.IsNull()
        || initialApplication.IsNull()
        || fixture.document->DocumentIdentifier() == saved.documentIdentifier) {
        NSLog(@"Bounded-curve cold probe failed: phase=fresh-session unit=%.17g reason=invalid-reader-session",
              unit);
        return false;
    }
    const auto initialStamp = fixture.document->DebugNativeMutationStamp();
    if (!initialStamp) {
        NSLog(@"Bounded-curve cold probe failed: phase=fresh-session unit=%.17g reason=missing-stamp",
              unit);
        return false;
    }
    if (initialStamp->instanceNonce == saved.writerInstanceNonce) {
        NSLog(@"Bounded-curve cold probe failed: phase=fresh-session unit=%.17g reason=same-owner-nonce",
              unit);
        return false;
    }
    const AssetImportResult importResult = fixture.viewer.ImportCbf(saved.path);
    if (importResult != AssetImportResult::Success) {
        NSLog(@"Bounded-curve cold probe failed: phase=import status=%u",
              static_cast<unsigned>(importResult));
        return false;
    }
    fixture.document = fixture.viewer.getDocument();
    if (fixture.document.IsNull() || fixture.document->Document().IsNull())
        return false;
    const auto reopenedDocument = fixture.document->Document();
    const auto reopenedData = reopenedDocument->GetData();
    const auto reopenedApplication = reopenedDocument->Application();
    if (reopenedData.IsNull() || reopenedApplication.IsNull()
        || reopenedDocument.get() == initialDocument.get()
        || reopenedData.get() == initialData.get()
        || reopenedApplication.get() != initialApplication.get()) {
        NSLog(@"Bounded-curve cold probe failed: phase=isolated-open unit=%.17g reason=adoption-check-failure",
              unit);
        return false;
    }
    const auto reopenedStamp = fixture.document->DebugNativeMutationStamp();
    if (!reopenedStamp) {
        NSLog(@"Bounded-curve cold probe failed: phase=isolated-open unit=%.17g reason=missing-stamp",
              unit);
        return false;
    }
    if (reopenedStamp->instanceNonce != initialStamp->instanceNonce
        || reopenedStamp->instanceNonce == saved.writerInstanceNonce
        || reopenedStamp->opening <= initialStamp->opening) {
        NSLog(@"Bounded-curve cold probe failed: phase=isolated-open unit=%.17g reason=adoption-check-failure",
              unit);
        return false;
    }
    double reopenedUnit = 0.0;
    if (!XCAFDoc_DocumentTool::GetLengthUnit(
            fixture.document->Document(), reopenedUnit)
        || reopenedUnit != unit) return false;
    const std::string entity = retained_solid::UUIDText(saved.key.entity);
    const auto snapshot = fixture.viewer.captureSceneSnapshot(64, 64);
    if (!snapshot || snapshot->publicationSourceIdentifier.empty()
        || snapshot->metersPerUnit != unit
        || std::none_of(snapshot->instances.begin(), snapshot->instances.end(),
            [&](const core3d::scene::InstanceSnapshot& instance) {
                return instance.entityIdentifier == entity;
            })) {
        NSLog(@"Bounded-curve cold probe failed: phase=snapshot");
        return false;
    }

    OcctBoundedCurveCapture reopened;
    if (!fixture.document->ReadBoundedCurveExact(saved.key, reopened)
        || !(reopened.persisted.ownerState.owner == saved.key)
        || reopened.record.value->definitionBytes != saved.definitionBytes
        || reopened.record.value->ownerBytes != saved.ownerBytes
        || reopened.persisted.ownerState.definitionRevision
            != saved.definitionRevision) {
        NSLog(@"Bounded-curve cold probe failed: phase=exact-read");
        return false;
    }
    fixture.document->NotifyChanges();
    auto context = fixture.context(entity);
    if (!context) {
        NSLog(@"Bounded-curve cold probe failed: phase=capture-context");
        return false;
    }
    OcafOwner reopenedOwner(*fixture.document, context);
    const SceneFence scene{context->openingFence().documentGeneration(),
        context->openingFence().modelRevision(),
        context->openingFence().metersPerUnit()};
    auto opening = reopenedOwner.capture(entity, scene);
    if (!opening || opening->retained.definition.controlPoints.empty()) {
        NSLog(@"Bounded-curve cold probe failed: phase=capture");
        return false;
    }
    Candidate candidate;
    candidate.proposal.kind = EditKind::MovePole;
    candidate.proposal.expected = opening->retained.authority;
    candidate.proposal.controlPoint =
        opening->retained.definition.controlPoints.front().identifier;
    candidate.proposal.replacementLocal =
        opening->retained.definition.controlPoints.front().local;
    candidate.proposal.replacementLocal[0] += unit == 0.001 ? 0.5 : 0.0005;
    candidate.completeDefinition = opening->retained.definition;
    candidate.completeDefinition.controlPoints.front().local =
        candidate.proposal.replacementLocal;
    Receipt preparedReceipt;
    auto prepared = reopenedOwner.prepare(opening, candidate, preparedReceipt);
    if (!prepared || preparedReceipt.outcome != Outcome::prepared) {
        NSLog(@"Bounded-curve cold probe failed: phase=prepare outcome=%u",
              static_cast<unsigned>(preparedReceipt.outcome));
        return false;
    }
    const Receipt edited = reopenedOwner.apply(prepared);
    OcctBoundedCurveCapture after;
    const bool applied = edited.outcome == Outcome::committed
        && edited.historyDelta == 1
        && fixture.document->ReadBoundedCurveExact(saved.key, after)
        && after.persisted.ownerState.owner == saved.key
        && after.persisted.ownerState.definitionRevision
            == reopened.persisted.ownerState.definitionRevision + 1;
    if (!applied) {
        NSLog(@"Bounded-curve cold probe failed: phase=apply outcome=%u historyDelta=%d",
              static_cast<unsigned>(edited.outcome), edited.historyDelta);
    }
    return applied;
}

struct BoundedCurveObservationScope final {
    bool consumed = false;
    BoundedCurveObservationScope() noexcept {
        scene::DebugBeginBoundedCurvePublicationObservation();
    }
    ~BoundedCurveObservationScope() noexcept {
        if (!consumed)
            scene::DebugCancelBoundedCurvePublicationObservation();
    }
};

bool SetOwnerVisibility(OwnerProbeFixture& fixture,
                        const retained_recipe::OwnerKey& key,
                        bool visible) noexcept {
    try {
        OcctBoundedCurveCapture exact;
        const auto& document = fixture.document->Document();
        const auto colors = XCAFDoc_DocumentTool::ColorTool(document->Main());
        if (document.IsNull() || colors.IsNull()
            || !fixture.document->ReadBoundedCurveExact(key, exact)) return false;
        document->NewCommand();
        colors->SetVisibility(exact.record.owner, visible);
        if (!document->CommitCommand()) {
            if (document->HasOpenCommand()) document->AbortCommand();
            return false;
        }
        fixture.document->NotifyChanges();
        return true;
    } catch (...) { return false; }
}

bool ValidateBoundedCurvePublication(
    OwnerProbeFixture& fixture,
    const retained_recipe::OwnerKey& key,
    scene::OcctSceneSnapshotBuilder::SnapshotPointer& published,
    scene::DebugBoundedCurvePublicationObservation& observation,
    OcctBoundedCurveCapture& exact) noexcept {
    try {
        BoundedCurveObservationScope scope;
        published = fixture.viewer.captureSceneSnapshot(64, 64);
        if (!published
            || !scene::DebugTakeBoundedCurvePublicationObservation(
                observation)) return false;
        scope.consumed = true;
        const std::string entity = retained_solid::UUIDText(key.entity);
        const auto item = std::find_if(published->instances.begin(),
            published->instances.end(), [&](const auto& value) {
                return value.entityIdentifier == entity;
            });
        if (item == published->instances.end()
            || item->meshIndex >= published->meshes.size()) return false;
        const auto& mesh = published->meshes[item->meshIndex];
        if (mesh.geometryKind != scene::GeometryKind::SurfaceTriangles
            || mesh.nativeC1Wire.has_value() || mesh.vertices.empty()
            || mesh.indices.empty() || mesh.primitives.size() != 1
            || item->primitiveBindings.size() != 1
            || item->primitiveBindings.front().materialIndex
                >= published->materials.size()
            || mesh.topology.faceCount != 0
            || mesh.topology.edgeCount == 0
            || mesh.topology.vertexCount == 0
            || mesh.topology.edgeCount != observation.edgeCount
            || mesh.topology.vertexCount != observation.vertexCount
            || !mesh.localBounds.valid
            || observation.entityIdentifier != item->entityIdentifier
            || observation.definitionIdentifier
                != mesh.definitionIdentifier
            || observation.publicationSourceIdentifier
                != published->publicationSourceIdentifier
            || observation.modelRevision != published->revisions.model)
            return false;
        for (const auto index : mesh.indices)
            if (index >= mesh.vertices.size()) return false;
        const auto& bounds = mesh.localBounds;
        const auto& matrix = item->worldFromObject.values;
        double minimum[3] = {DBL_MAX, DBL_MAX, DBL_MAX};
        double maximum[3] = {-DBL_MAX, -DBL_MAX, -DBL_MAX};
        for (double x : {bounds.minimum.x, bounds.maximum.x})
            for (double y : {bounds.minimum.y, bounds.maximum.y})
                for (double z : {bounds.minimum.z, bounds.maximum.z}) {
                    const double world[3] = {
                        matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12],
                        matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13],
                        matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14],
                    };
                    for (std::size_t axis = 0; axis < 3; ++axis) {
                        if (!std::isfinite(world[axis])) return false;
                        minimum[axis] = std::min(minimum[axis], world[axis]);
                        maximum[axis] = std::max(maximum[axis], world[axis]);
                    }
                }
        if (!(minimum[0] < maximum[0]
            || minimum[1] < maximum[1]
            || minimum[2] < maximum[2])) return false;
        return fixture.document->ReadBoundedCurveExact(key, exact)
            && observation.canonicalDefinitionBytes
                == exact.record.value->definitionBytes
            && observation.canonicalOwnerBytes
                == exact.record.value->ownerBytes
            && observation.canonicalDefinitionDigest
                == exact.persisted.ownerState.canonicalDefinitionDigest
            && exact.record.current.IsEqual(
                XCAFDoc_ShapeTool::GetShape(exact.record.owner))
            && WireMatchesDefinition(exact.persisted, exact.wire);
    } catch (...) { return false; }
}
} // namespace

extern "C" bool Core3DDebugBoundedCurveOwnerProbeSetupFailureIsContained()
noexcept {
#if TARGET_OS_IOS
    if (![NSThread isMainThread]) return false;
    EAGLContext *callerContext = EAGLContext.currentContext;
    try {
        OwnerProbeFixture fixture;
        if (fixture.initialize(0.001, true)) return false;
        const bool recognized = fixture.injectionReached
            && fixture.setupFailureRecognized;
        fixture.shutdown();
        fixture.shutdown();
        return recognized && fixture.setupCommandSettled
            && fixture.cleanupSucceeded && fixture.host == nil
            && fixture.document.IsNull()
            && fixture.sourcePresentation.IsNull()
            && EAGLContext.currentContext == callerContext;
    } catch (...) { return false; }
#else
    return false;
#endif
}

extern "C" std::uint64_t Core3DDebugBoundedCurveOwnerProbe(
    std::int32_t scenario) noexcept {
    if (![NSThread isMainThread] || scenario < 0 || scenario > 3) return 0;
    try {
        if (scenario == 0) {
            OwnerProbeFixture fixture;
            if (!fixture.initialize(0.001)) return 0;
            __block std::uint64_t bits = 0;
            OwnerProbeFixture *fixturePointer = &fixture;
            if (!fixture.perform(^{
                bits = RunOwnerProbeScenario0(*fixturePointer);
            })) return 0;
            return bits;
        }
        if (scenario == 1) {
            OwnerProbeFixture fixture, foreign;
            if (!fixture.initialize(0.001) || !foreign.initialize(0.001)) return 0;
            __block std::uint64_t bits = 0;
            OwnerProbeFixture *fixturePointer = &fixture;
            OwnerProbeFixture *foreignPointer = &foreign;
            if (!fixture.perform(^{
                bits = RunOwnerProbeScenario1(
                    *fixturePointer, *foreignPointer);
            })) return 0;
            return bits;
        }
        if (scenario == 2) {
            OwnerProbeFixture fixture;
            if (!fixture.initialize(0.001)) return 0;
            __block std::uint64_t bits = 0;
            OwnerProbeFixture *fixturePointer = &fixture;
            if (!fixture.perform(^{
                bits = RunOwnerProbeScenario2(*fixturePointer);
            })) return 0;
            return bits;
        }
        std::uint64_t bits = 0;
        for (double unit : {0.001, 1.0}) {
            OwnerProbeColdSavedState saved;
            bool created = false;
            {
                OwnerProbeFixture writer;
                if (!writer.initialize(unit)) return bits;
                bits |= unit == 0.001 ? 1ULL : 2ULL;
                if (!RunOwnerProbeColdSave(writer, saved, created, unit)) {
                    if (created) bits |= unit == 0.001 ? 4ULL : 8ULL;
                    return bits;
                }
                if (created) bits |= unit == 0.001 ? 4ULL : 8ULL;
                // End the writer session outside its active framebuffer block.
                writer.shutdown();
                if (!writer.cleanupSucceeded
                    || !writer.document.IsNull()
                    || !writer.viewer.getDocument().IsNull()
#if TARGET_OS_IOS
                    || writer.host != nil
#endif
                    ) {
                    NSLog(@"Bounded-curve cold probe failed: phase=writer-teardown unit=%.17g reason=cleanup-not-completed",
                          unit);
                    return bits;
                }
            }

            OwnerProbeFixture reopened;
            if (!reopened.initialize(unit)) return bits;
            __block bool completed = false;
            OwnerProbeFixture *reopenedPointer = &reopened;
            if (!reopened.perform(^{
                completed = RunOwnerProbeColdOpenAndEdit(
                    *reopenedPointer, unit, saved);
            })) return bits;
            if (!completed) return bits;
        }
        bits |= 16ULL;
        return bits;
    } catch (...) { return 0; }
}

extern "C" std::uint64_t Core3DDebugBoundedCurvePublicationProbe(
    std::int32_t scenario) noexcept {
    if (![NSThread isMainThread] || scenario < 0 || scenario > 3) return 0;
    try {
        OwnerProbeFixture fixture;
        if (!fixture.initialize(scenario == 3 ? 1.0 : 0.001)) return 0;
        const auto before = fixture.viewer.captureSceneSnapshot(64, 64);
        std::uint64_t bits = before ? 1ULL : 0ULL;
        scene::DebugBoundedCurvePublicationObservation absent;
        if (!before || scene::DebugTakeBoundedCurvePublicationObservation(absent)
            || std::any_of(before->meshes.begin(), before->meshes.end(),
                [](const auto& mesh) { return mesh.nativeC1Wire.has_value(); }))
            return bits;
        bits |= 2ULL;
        Receipt receipt; retained_recipe::OwnerKey key;
        if (!CreateOne(fixture, receipt, key)) return bits;
        bits |= 4ULL;
        const std::string entity = retained_solid::UUIDText(key.entity);
        std::shared_ptr<native_opening::Context> oldContext;
        Core3DBoundedCurveEditingOpening *currentOpening = nil;
        bool currentAdmissionObserved = false;
        if (scenario == 0 && !SetOwnerVisibility(fixture, key, false))
            return bits;
        if (scenario == 2) {
            oldContext = fixture.context(entity);
            if (!oldContext) {
                NSLog(@"Bounded-curve publication probe failed: phase=current-context");
                return bits;
            }
            currentOpening = native_opening::OpenBoundedCurve(
                fixture.document, entity, oldContext);
            if (!currentOpening) {
                NSLog(@"Bounded-curve publication probe failed: phase=current-open");
                return bits;
            }
            currentAdmissionObserved = true;
            if (!SetOwnerVisibility(fixture, key, false)) {
                NSLog(@"Bounded-curve publication probe failed: phase=visibility-mutation");
                return bits;
            }
        }

        scene::OcctSceneSnapshotBuilder::SnapshotPointer published;
        scene::DebugBoundedCurvePublicationObservation observation;
        OcctBoundedCurveCapture exact;
        if (!ValidateBoundedCurvePublication(
                fixture, key, published, observation, exact))
            return bits;

        bool scenarioObserved = false;
        const auto curveItem = std::find_if(published->instances.begin(),
            published->instances.end(), [&](const auto& value) {
                return value.entityIdentifier == entity;
            });
        if (scenario == 0) {
            const auto sourceItem = std::find_if(published->instances.begin(),
                published->instances.end(), [&](const auto& value) {
                    return value.entityIdentifier == fixture.sourceEntity;
                });
            scenarioObserved = curveItem != published->instances.end()
                && sourceItem != published->instances.end()
                && !curveItem->visible && sourceItem->visible
                && !curveItem->selected && !sourceItem->selected
                && published->selection.selected.empty();
        } else if (scenario == 1) {
            const std::uint64_t modelRevision =
                fixture.viewer.DebugPublishedModelRevision();
            const auto& document = fixture.document->Document();
            document->NewCommand();
            const TDF_Label invalidRecord = TDF_TagSource::NewChild(
                document->Main());
            invalidRecord.AddAttribute(new bounded_curve::Attribute());
            const bool injected = document->CommitCommand();
            fixture.document->NotifyChanges();
            const auto refused = fixture.viewer.captureSceneSnapshot(64, 64);
            scenarioObserved = injected && !refused
                && fixture.viewer.DebugPublishedModelRevision()
                    == modelRevision
                && document->Undo();
            fixture.document->NotifyChanges();
        } else if (scenario == 2) {
            Core3DBoundedCurveEditingOpening *staleOpening =
                native_opening::OpenBoundedCurve(
                    fixture.document, entity, oldContext);
            const bool staleRefusalObserved = staleOpening == nil;
            if (staleOpening) {
                (void)[staleOpening cancel];
                NSLog(@"Bounded-curve publication probe failed: phase=stale-refusal");
                return bits;
            }
            const std::weak_ptr<native_opening::Context> oldContextLifetime =
                oldContext;
            if (![currentOpening cancel]) {
                NSLog(@"Bounded-curve publication probe failed: phase=current-cancel");
                return bits;
            }
            currentOpening = nil;
            oldContext.reset();
            if (!oldContextLifetime.expired()) {
                NSLog(@"Bounded-curve publication probe failed: phase=context-release");
                return bits;
            }
            auto freshContext = fixture.context(entity);
            if (!freshContext) {
                NSLog(@"Bounded-curve publication probe failed: phase=fresh-context");
                return bits;
            }
            Core3DBoundedCurveEditingOpening *freshOpening =
                native_opening::OpenBoundedCurve(
                    fixture.document, entity, freshContext);
            const bool freshAdmissionObserved = freshOpening != nil;
            if (!freshOpening) {
                NSLog(@"Bounded-curve publication probe failed: phase=fresh-open");
                return bits;
            }
            const std::weak_ptr<native_opening::Context> freshContextLifetime =
                freshContext;
            const bool freshCancelled = [freshOpening cancel];
            freshOpening = nil;
            freshContext.reset();
            if (!freshCancelled || !freshContextLifetime.expired()) {
                NSLog(@"Bounded-curve publication probe failed: phase=fresh-release");
                return bits;
            }
            scenarioObserved = currentAdmissionObserved
                && staleRefusalObserved && freshAdmissionObserved
                && published->metersPerUnit == 0.001;
        } else {
            const std::size_t curveCount = std::count_if(
                published->instances.begin(), published->instances.end(),
                [&](const auto& value) {
                    return value.meshIndex < published->meshes.size()
                        && published->meshes[value.meshIndex]
                            .definitionIdentifier
                            == observation.definitionIdentifier;
                });
            const auto exportResult =
                scene::OcctSceneSnapshotBuilder::BuildPrivateExportDerivative(
                    fixture.document, *published, false,
                    [](const TopoDS_Shape&) {}, [] { return false; });
            scenarioObserved = published->metersPerUnit == 1.0
                && curveCount == 1 && !exportResult;
        }
        if (!scenarioObserved) return bits;
        bits |= 8ULL;
        const auto repeated = fixture.viewer.captureSceneSnapshot(64, 64);
        if (repeated && repeated->revisions.model == published->revisions.model)
            bits |= 16ULL;
        else if (scenario == 2)
            NSLog(@"Bounded-curve publication probe failed: phase=repeated-publication");
        return bits;
    } catch (...) { return 0; }
}

extern "C" void* Core3DDebugBoundedCurveMetalFixture() noexcept {
    if (![NSThread isMainThread]) return nullptr;
    try {
        OwnerProbeFixture fixture;
        if (!fixture.initialize(0.001)) return nullptr;
        Receipt receipt;
        retained_recipe::OwnerKey key;
        if (!CreateOne(fixture, receipt, key)) return nullptr;
        scene::OcctSceneSnapshotBuilder::SnapshotPointer published;
        scene::DebugBoundedCurvePublicationObservation observation;
        OcctBoundedCurveCapture exact;
        if (!ValidateBoundedCurvePublication(
                fixture, key, published, observation, exact)) return nullptr;
        const auto overlay = fixture.viewer.captureScenePresentationOverlay();
        if (!overlay) return nullptr;
        Core3DSceneSnapshot *sceneDTO =
            Core3DCreateSceneSnapshotDTO(*published);
        Core3DScenePresentationOverlaySnapshot *overlayDTO =
            Core3DCreateScenePresentationOverlaySnapshotDTO(*overlay);
        if (sceneDTO == nil || overlayDTO == nil) return nullptr;
        NSArray *result = @[sceneDTO, overlayDTO];
        return (__bridge_retained void*)result;
    } catch (...) { return nullptr; }
}
} // namespace core3d::native_opening::debug
#endif
