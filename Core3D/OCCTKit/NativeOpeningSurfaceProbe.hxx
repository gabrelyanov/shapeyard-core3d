#pragma once

#if DEBUG

#include "BoundedCurveBuild.hxx"
#include "FeaturePatternBaselineBinaryDriver.hxx"
#include "FeaturePatternChildBinaryDriver.hxx"
#include "FeaturePatternNativeBuild.hxx"
#include <BinDrivers_DocumentRetrievalDriver.hxx>
#include <BinXCAFDrivers_DocumentStorageDriver.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Message.hxx>
#include <PCDM_ReaderStatus.hxx>
#include <PCDM_StoreStatus.hxx>
#include <TDF_Data.hxx>
#include <TDF_Tool.hxx>
#include <TDataStd_ByteArray.hxx>
#include <TNaming_Builder.hxx>
#include <TDocStd_Application.hxx>
#include <TopExp_Explorer.hxx>

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <string_view>
#include <cstdint>
#include <cstring>
#include <sstream>

// Defined in OcctDocument.mm (declared in OcctDocument.h); redeclared here so
// C++ includers that do not import OcctDocument.h first still compile.
Standard_EXPORT void Core3DBeginSafeBinaryRead();
Standard_EXPORT Standard_Boolean Core3DSafeBinaryReadWasRejected();
Standard_EXPORT void Core3DDefineSafeBinXCAFFormat(
    const Handle(TDocStd_Application)& application);
Standard_EXPORT Standard_Boolean Core3DValidateOwnedFrameUsage(
    const Handle(TDocStd_Document)& document, Standard_Size& nativeBytes,
    Standard_Size maximumBytes) noexcept;
#if DEBUG
void Core3DDebugDefineLegacyReceiptFormats(
    const Handle(TDocStd_Application)& application);
#endif

namespace core3d::native_opening::debug {

//! Observations from real native fixtures. This is deliberately descriptive:
//! it carries no document handles, labels, leases, stamps, or edit receipts.
struct SurfaceProbeEvidence final {
    bool accepted = false;
    std::string error;
    std::string documentIdentifier;
    std::string viewerDocumentIdentifier;
    int initialUndoCount = -1;
    int initialRedoCount = -1;
    int committedUndoCount = -1;
    int committedRedoCount = -1;
    bool undoSucceeded = false;
    int undoUndoCount = -1;
    int undoRedoCount = -1;
    bool redoSucceeded = false;
    int redoUndoCount = -1;
    int redoRedoCount = -1;
};

//! Exact-name dispatch; unrecognised scenarios fail without constructing a
//! fixture. Implemented by the DEBUG-only Objective-C++ modeling surface.
SurfaceProbeEvidence RunSurfaceProbe(std::string_view scenario) noexcept;

//! Link-private XCTest seam for the exact-label primitives. The probe owns a
//! fully initialized viewer on a detached real 64x64 offscreen framebuffer and
//! returns the fixed five-bit receipt assembled from real OcctDocument
//! operations. Graphics/setup failure leaves the readiness bit absent.
extern "C" std::uint64_t Core3DDebugExactLabelProbe(std::int32_t scenario) noexcept;
//! Exercises a real initialized viewer and owned seed command, injects an OCCT
//! setup failure, and observes that cleanup and caller context restoration are
//! contained without exposing document or lease authority.
extern "C" bool Core3DDebugExactLabelProbeSetupFailureIsContained() noexcept;

//! Concrete C1 owner/document lifecycle scenarios. The DEBUG probe owns a real
//! initialized viewer and retains its detached real 64x64 offscreen graphics
//! host/context for every native opening through teardown. Missing graphics
//! readiness leaves the readiness bit absent. Its receipt remains the five-bit
//! observation mask. Implemented in the owner translation unit so Release
//! contains neither declaration nor dispatch seam.
extern "C" std::uint64_t Core3DDebugBoundedCurveOwnerProbe(
    std::int32_t scenario) noexcept;
//! Descriptive publication evidence only. It never returns a context, fence,
//! command lease, label, document handle, or authority token.
extern "C" std::uint64_t Core3DDebugBoundedCurvePublicationProbe(
    std::int32_t scenario) noexcept;
//! Retained NSArray containing one validated public scene DTO followed by its
//! exact empty overlay DTO. The Swift Metal admission test consumes ownership.
extern "C" void* Core3DDebugBoundedCurveMetalFixture() noexcept;
extern "C" bool
Core3DDebugBoundedCurveOwnerProbeSetupFailureIsContained() noexcept;

//! D2 all-label production collaborator scenarios. The implementation lives
//! in PatternNativeCollaborators.mm and is absent from Release declarations.
extern "C" std::uint64_t Core3DDebugPatternNativeCollaboratorsProbe(
    std::int32_t scenario) noexcept;

//! D3 live-path, replacement-path and C1 dependent replay scenarios.
extern "C" std::uint64_t Core3DDebugPathArrayNativeCollaboratorsProbe(
    std::int32_t scenario) noexcept;

//! D4 paired observer, detached rebuilder and one-command stager scenarios.
extern "C" std::uint64_t Core3DDebugFeaturePatternNativeCollaboratorsProbe(
    std::int32_t scenario) noexcept;
extern "C" std::uint64_t Core3DDebugFeaturePatternReferenceResolutionProbe(
    std::int32_t scenario) noexcept;

//! Shared D2/D3/D4 closure, unrouted-mutation and unsupported-descendant
//! safeguards. Implemented beside the real OcctDocument census in DEBUG only.
extern "C" std::uint64_t Core3DDebugDependentReplaySafeguardsProbe(
    std::int32_t scenario) noexcept;

//! Package-18 opaque opening body, strict DTO, one-shot and ownership probe.
extern "C" std::uint64_t Core3DDebugNativeOpeningBodiesProbe(
    std::int32_t scenario) noexcept;

//! Row-265 lifecycle probe: real controller/category factories, real native
//! D2/D3/D4 sessions on a DEBUG fixture document, real commit/refusal/close
//! and shared recovery-fence observations. Nine independent bits (0x1ff) per
//! scenario; implemented beside Core3DDebugNativeOpeningBodiesProbe.
extern "C" std::uint64_t Core3DDebugNativeOpeningLifecycleProbe(
    std::int32_t scenario) noexcept;

//! Row-179 D4 typed-baseline probe: the actual R179 D4 fixture in both unit
//! systems, a nonnull exact published snapshot, production normal save, safe
//! cold reopen with exact recipe/identity/native-reference/child-receipt
//! comparison, and one real subsequent D4 edit. Five independent bits (0x1f);
//! implemented beside Core3DDebugNativeOpeningLifecycleProbe.
extern "C" std::uint64_t Core3DDebugNativeOpeningBaselineProbe(
    std::int32_t scenario) noexcept;

//! Installs and validates the real retained-solid record used by the D4 seed.
//! Implemented beside the document-internal retained-solid probe so UI code
//! does not depend on that private header.
extern "C" bool Core3DDebugInstallRetainedSolidSeedRecord(
    const Handle(TDocStd_Document)& document, const TDF_Label& source,
    const retained_boolean::Program& program, const TopoDS_Shape& current,
    const TopoDS_Shape& retainedBase) noexcept;

//! Package-21 complete C1 projection, UUID edit parsing and unsupported
//! structural-mutation probe. Production owner execution is cross-checked by
//! the existing concrete C1 owner fixture from this same test surface.
extern "C" std::uint64_t Core3DDebugBoundedCurveOpeningAdapterProbe(
    std::int32_t scenario) noexcept;

//! Effective retained-pattern presentation scenarios. Implemented beside the
//! real viewer traversal and excluded, declaration included, from Release.
extern "C" std::uint64_t Core3DDebugRetainedSuppressionPresentationProbe(
    std::int32_t scenario) noexcept;

namespace child_receipt_probe {
inline feature_pattern_child::UUID ID(std::uint8_t value) noexcept {
    feature_pattern_child::UUID result{}; result[0] = value; return result;
}
inline feature_pattern_child::Digest Proof(std::uint8_t value) noexcept {
    feature_pattern_child::Digest result{}; result[0] = value; return result;
}
inline feature_pattern_child::Receipt Fixture() {
    using namespace feature_pattern_child;
    Receipt value; value.document = ID(1); value.hostEntity = ID(2);
    value.hostDefinition = ID(3); value.patternFeature = ID(4);
    value.childFeature = ID(5); value.instanceIdentity = ID(6);
    value.baselineRecipeIdentity = ID(7); value.localID = 11;
    value.row = -2; value.column = 3;
    value.selectors = {
        {SelectorKind::GeneratedSection, ID(9), 4, Proof(19)},
        {SelectorKind::SourceOperand, ID(8), 2, Proof(18)}
    };
    return value;
}
inline bool Redigest(std::vector<std::uint8_t>& bytes) noexcept {
    using namespace feature_pattern_child;
    if (bytes.size() < Digest{}.size()) return false;
    bytes.resize(bytes.size() - Digest{}.size()); Digest digest{};
    if (!feature_pattern_child::detail::Hash(bytes, digest)) return false;
    bytes.insert(bytes.end(), digest.begin(), digest.end()); return true;
}
inline std::uint64_t Run(std::int32_t scenario) noexcept {
    using namespace feature_pattern_child;
    try {
        if (scenario == 0) {
            const Receipt fixture = Fixture(); Receipt decoded;
            std::vector<std::uint8_t> canonical;
            std::uint64_t bits = Encode(fixture, canonical)
                && Decode(canonical, decoded) ? 1ULL : 0ULL;
            auto unknown = canonical; unknown[4] = 2;
            if (!Decode(unknown, decoded)) bits |= 2ULL;
            Receipt duplicate = fixture; duplicate.selectors.push_back(duplicate.selectors.front());
            if (!Encode(duplicate, unknown)) bits |= 4ULL;
            auto overflow = canonical;
            constexpr std::size_t selectorCountOffset = 8 + 7 * 16 + 8 + 4 + 4;
            overflow[selectorCountOffset] = std::uint8_t(MaximumSelectors + 1);
            if (Redigest(overflow) && !Decode(overflow, decoded)) bits |= 8ULL;
            auto noncanonical = canonical;
            constexpr std::size_t selectorOffset = selectorCountOffset + 4;
            std::swap_ranges(noncanonical.begin() + selectorOffset,
                noncanonical.begin() + selectorOffset + 56,
                noncanonical.begin() + selectorOffset + 56);
            if (Redigest(noncanonical) && !Decode(noncanonical, decoded)) bits |= 16ULL;
            return bits;
        }
        if (scenario == 1) {
            Handle(TDF_Data) sourceData = new TDF_Data();
            Handle(TDF_Data) targetData = new TDF_Data();
            const TDF_Label sourceRoot = sourceData->Root();
            const TDF_Label targetRoot = targetData->Root();
            const TDF_Label sourceHost = sourceRoot.FindChild(1, Standard_True);
            const TDF_Label sourceBase = sourceRoot.FindChild(2, Standard_True);
            const TDF_Label sourceRecord = sourceHost.FindChild(10, Standard_True);
            const TDF_Label targetHost = targetRoot.FindChild(7, Standard_True);
            const TDF_Label targetBase = targetRoot.FindChild(8, Standard_True);
            const TDF_Label targetRecord = targetHost.FindChild(12, Standard_True);
            std::uint64_t bits = Attach(sourceRecord, sourceHost, sourceBase, Fixture())
                ? 1ULL : 0ULL;
            Handle(Attribute) sourceAttribute;
            sourceRecord.FindAttribute(AttributeID(), sourceAttribute);
            BinaryDriver driver(Message::DefaultMessenger(), std::make_shared<Budget>());
            Handle(Attribute) driverEmpty = Handle(Attribute)::DownCast(driver.NewEmpty());
            targetRecord.AddAttribute(driverEmpty); TDF_Reference::Set(targetRecord, targetBase);
            if (!driverEmpty.IsNull() && !driverEmpty->value()
                && driver.SourceType() == STANDARD_TYPE(Attribute)) bits |= 2ULL;
            Handle(TDF_RelocationTable) relocation = new TDF_RelocationTable(Standard_False);
            relocation->SetRelocation(sourceHost, targetHost);
            relocation->SetRelocation(sourceBase, targetBase);
            sourceAttribute->Paste(driverEmpty, relocation);
            const auto copied = driverEmpty->value();
            if (copied && copied->host.IsEqual(targetHost)
                && copied->baselineRecipe.IsEqual(targetBase)) bits |= 4ULL;
            std::vector<std::uint8_t> exact;
            if (copied && Encode(copied->receipt, exact)
                && exact == copied->canonicalBytes) bits |= 8ULL;
            const TDF_Label refusedRecord = targetHost.FindChild(13, Standard_True);
            Handle(Attribute) refused = new Attribute(); refusedRecord.AddAttribute(refused);
            try {
                Handle(TDF_RelocationTable) missing = new TDF_RelocationTable(Standard_False);
                sourceAttribute->Paste(refused, missing);
            } catch (const Standard_Failure&) {}
            if (!refused->value()) bits |= 16ULL;
            return bits;
        }
        if (scenario == 2) {
            const Receipt fixture = Fixture(); std::vector<std::uint8_t> bytes;
            if (!Encode(fixture, bytes)) return 0;
            Budget budget; budget.limit = bytes.size() + PersistedReferenceBudgetBytes;
            std::uint64_t bits = Charge(budget, bytes.size(), fixture.selectors.size())
                ? 1ULL : 0ULL;
            if (budget.bytes == bytes.size() + PersistedReferenceBudgetBytes) bits |= 2ULL;
            if (budget.selectors == fixture.selectors.size()) bits |= 4ULL;
            if (budget.baselineReferences == 1) bits |= 8ULL;
            if (!Charge(budget, bytes.size(), fixture.selectors.size())
                && budget.rejected) bits |= 16ULL;
            return bits;
        }
    } catch (...) {}
    return 0;
}
} // namespace child_receipt_probe

extern "C" __attribute__((used, visibility("default"))) inline std::uint64_t
Core3DDebugFeaturePatternChildReceiptProbe(std::int32_t scenario) noexcept {
    return child_receipt_probe::Run(scenario);
}

namespace detached_tool_probe {
using UUID = feature_pattern::UUID;
inline UUID ID(std::uint8_t seed) noexcept {
    UUID result{};
    for (std::size_t index = 0; index < result.size(); ++index)
        result[index] = std::uint8_t(seed + index);
    return result;
}

inline retained_boolean::Program Program() {
    retained_boolean::Program program;
    program.source.document = ID(1);
    program.source.entity = ID(20);
    program.source.definition = ID(21);
    program.source.sourceFeature = ID(22);
    program.source.derivedFeature = ID(23);
    program.source.family = 1;
    program.source.metersPerUnit = .001;
    profile::Parameters profile;
    profile.metersPerUnit = .001;
    profile.definition.plane = 0;
    profile.definition.depth = 20;
    profile.definition.points = {{0,0}, {90,0}, {90,60}, {0,60}};
    program.source.schema = std::uint32_t(profile::SchemaFor(profile));
    if (!profile::Encode(profile, program.source.values))
        throw std::runtime_error("D4 source profile");
    retained_boolean::Step other;
    other.operand.identifier = 7;
    other.operand.axis = analytic_boolean::Axis::Z;
    other.operand.point = {{72, 30, 0}};
    other.operand.radius = 3;
    retained_boolean::Step selected = other;
    selected.operand.identifier = 9;
    selected.operand.point = {{15, 30, 0}};
    selected.operand.radius = 4;
    program.steps = {other, selected};
    program.nextOperandID = 10;
    if (!retained_boolean::Valid(program))
        throw std::runtime_error("D4 source program");
    return program;
}

inline feature_pattern::Definition Definition(std::uint32_t columns) {
    feature_pattern::Definition value;
    value.host = {ID(1), ID(30), ID(31)};
    value.feature = ID(32);
    value.sourceCut = {ID(1), ID(20), ID(21), ID(23)};
    value.sourceCutStepID = 9;
    value.metersPerUnit = .001;
    value.minimumHostLigamentMM = .002;
    value.expectedBoundarySectionsPerFeature = 2;
    auto& distribution = value.distribution;
    distribution.owner = value.host;
    distribution.feature = value.feature;
    distribution.source = value.sourceCut;
    distribution.kind = pattern::Kind::Linear;
    distribution.columnAxis = pattern::Axis::X;
    distribution.rowAxis = pattern::Axis::Y;
    distribution.rowCount = 1;
    distribution.columnCount = columns;
    distribution.columnSpacing = 25;
    distribution.sourceFrame = pattern::IdentityMatrix();
    distribution.issuance.nextLocalID = std::uint64_t(columns) + 1;
    for (std::uint32_t column = 0; column < columns; ++column) {
        pattern::Member member;
        member.identity = column == 0 ? value.sourceCut.entity
                                      : ID(std::uint8_t(50 + column));
        member.localID = column + 1;
        member.coordinate = {0, column};
        member.state = pattern::MemberState::Active;
        distribution.members.push_back(member);
    }
    if (!feature_pattern::Valid(value))
        throw std::runtime_error("D4 pattern definition");
    return value;
}

struct Fixture final {
    feature_pattern_native::HostBaseline host;
    feature_pattern_native::SourceProgram source;
    feature_pattern::ExpansionBudget budget;
    std::atomic_bool stop{false};
    Fixture() {
        host.host = {ID(1), ID(30), ID(31)};
        host.retainedRecipeFeature = ID(33);
        host.baselineRecipeIdentity = ID(34);
        host.exactRecipe = {'H','O','S','T',1};
        host.solid = BRepPrimAPI_MakeBox(90, 60, 20).Shape();
        host.retainedRecipeCurrent = true;
        source.program = Program();
        retained_boolean::Encode(source.program, source.exactProgram);
        source.retainedBase = BRepPrimAPI_MakeBox(90, 60, 20).Shape();
        budget.existingDocumentBytes = 1024;
        budget.existingMemoryBytes = 1024;
    }
    feature_pattern_native::AttributedBuild Build(
        const feature_pattern::Definition& definition) {
        return feature_pattern_native::BuildAttributedPattern(
            host, source, definition, budget, stop);
    }
};

inline std::uint64_t Run(std::int32_t scenario) noexcept {
    using namespace feature_pattern_native;
    try {
        Fixture fixture;
        if (scenario == 0) {
            std::uint64_t bits = retained_boolean::Valid(fixture.source.program)
                ? 1ULL : 0ULL;
            SourceTool selected;
            if (BuildSourceTool(fixture.source, 9, fixture.stop, selected)
                == Status::Built) bits |= 2ULL;
            if (!selected.detached.tool.IsNull()
                && selected.selectedStableOperand == 9
                && selected.detached.stableOperandID == 9) bits |= 4ULL;
            std::vector<std::uint8_t> after;
            if (retained_boolean::Encode(fixture.source.program, after)
                && after == fixture.source.exactProgram
                && selected.detached.exactProgram == after) bits |= 8ULL;
            SourceTool missing;
            if (BuildSourceTool(fixture.source, 7, fixture.stop, missing)
                    == Status::Built
                && missing.selectedStableOperand == 7
                && missing.detached.toolGeometry != selected.detached.toolGeometry)
                bits |= 16ULL;
            return bits;
        }
        if (scenario == 1) {
            const auto definition = Definition(2);
            const auto built = fixture.Build(definition);
            std::uint64_t bits = built.admitted() ? 1ULL : 0ULL;
            if (!built.admitted()) return bits;
            auto missing = built.selectors; missing.pop_back();
            if (ResolveSelectors(built.result, definition, missing)
                == Status::ChildSetMismatch) bits |= 2ULL;
            auto swapped = built.selectors;
            std::swap(swapped[0].childFeature, swapped[1].childFeature);
            feature_pattern_native::detail::SelectorProof(swapped[0], swapped[0].proof);
            feature_pattern_native::detail::SelectorProof(swapped[1], swapped[1].proof);
            if (ResolveSelectors(built.result, definition, swapped)
                == Status::ChildSetMismatch) bits |= 4ULL;
            auto extra = built.selectors; extra.push_back(extra.front());
            if (ResolveSelectors(built.result, definition, extra)
                == Status::ChildSetMismatch) bits |= 8ULL;
            const double before = feature_pattern_native::detail::Volume(built.result);
            if (std::isfinite(before) && before > 0
                && feature_pattern_native::detail::Volume(built.result) == before
                && ResolveSelectors(built.result, definition, built.selectors)
                    == Status::Built) bits |= 16ULL;
            return bits;
        }
        if (scenario == 2) {
            const auto definition = Definition(2);
            const auto built = fixture.Build(definition);
            std::uint64_t bits = built.admitted() ? 1ULL : 0ULL;
            if (!built.admitted()) return bits;
            BRepBuilderAPI_Copy copy(built.result, Standard_True, Standard_False);
            if (copy.IsDone() && !copy.Shape().IsSame(built.result)) bits |= 2ULL;
            if (ResolveSelectors(copy.Shape(), definition, built.selectors)
                == Status::Built) bits |= 4ULL;
            bool canonical = built.childReceipts.size() == built.selectors.size();
            for (const auto& receipt : built.childReceipts) {
                std::vector<std::uint8_t> bytes;
                feature_pattern_child::Receipt decoded;
                canonical = canonical && feature_pattern_child::Encode(receipt, bytes)
                    && feature_pattern_child::Decode(bytes, decoded)
                    && decoded.childFeature == receipt.childFeature;
            }
            if (canonical) bits |= 8ULL;
            bool semantic = true;
            for (const auto& selector : built.selectors)
                semantic = semantic && selector.sourceStableOperand == 9
                    && selector.orientedBoundarySections == 2
                    && feature_pattern_child::Nonzero(selector.proof);
            if (semantic) bits |= 16ULL;
            return bits;
        }
        if (scenario == 3) {
            auto three = Definition(3);
            const auto full = fixture.Build(three);
            std::uint64_t bits = full.admitted() ? 1ULL : 0ULL;
            if (!full.admitted()) return bits;
            auto suppressedDefinition = three;
            if (pattern::SetSuppressed(suppressedDefinition.distribution,
                                       {0, 2}, true)) bits |= 2ULL;
            const auto suppressed = fixture.Build(suppressedDefinition);
            if (suppressed.admitted()
                && feature_pattern_native::detail::Volume(suppressed.result) > feature_pattern_native::detail::Volume(full.result))
                bits |= 4ULL;
            auto shrunkDefinition = three;
            const auto issue = [](UUID&) { return false; };
            if (feature_pattern::ReconcileCounts(shrunkDefinition, 1, 2, issue))
                bits |= 8ULL;
            const auto shrunk = fixture.Build(shrunkDefinition);
            if (shrunk.admitted() && suppressed.admitted()
                && shrunk.retainedHost == full.retainedHost
                && suppressed.retainedHost == full.retainedHost
                && std::abs(feature_pattern_native::detail::Volume(shrunk.result)
                    - feature_pattern_native::detail::Volume(suppressed.result)) <= 1e-8
                && shrunk.completeProgramBytes == fixture.source.exactProgram
                && suppressed.completeProgramBytes == fixture.source.exactProgram)
                bits |= 16ULL;
            return bits;
        }
    } catch (...) {}
    return 0;
}
} // namespace detached_tool_probe

extern "C" __attribute__((used, visibility("default"))) inline std::uint64_t
Core3DDebugFeaturePatternNativeBuildProbe(std::int32_t scenario) noexcept {
    return detached_tool_probe::Run(scenario);
}

namespace paired_receipt_probe {
struct Document final {
    Handle(TDocStd_Application) app = new TDocStd_Application();
    Handle(TDocStd_Document) value;
    Document() {
        Core3DDefineSafeBinXCAFFormat(app);
        app->NewDocument(TCollection_ExtendedString("BinXCAF"), value);
        if (value.IsNull()) throw std::runtime_error("D4 paired document");
        value->SetUndoLimit(8);
    }
    ~Document() noexcept {
        try { if (!value.IsNull()) app->Close(value); } catch (...) {}
    }
};
inline feature_pattern_child::Receipt ReceiptFor(
    const feature_pattern::Definition& definition,
    const pattern::Member& member) {
    feature_pattern_child::Receipt receipt;
    receipt.document = definition.host.document;
    receipt.hostEntity = definition.host.entity;
    receipt.hostDefinition = definition.host.definition;
    receipt.patternFeature = definition.feature;
    receipt.childFeature = feature_pattern::ChildFeatureID(definition, member);
    receipt.instanceIdentity = member.identity;
    receipt.baselineRecipeIdentity = detached_tool_probe::ID(34);
    receipt.localID = member.localID;
    receipt.row = std::int32_t(member.coordinate.row);
    receipt.column = std::int32_t(member.coordinate.column);
    receipt.selectors = {{feature_pattern_child::SelectorKind::SourceOperand,
        detached_tool_probe::ID(std::uint8_t(80 + member.localID)), 0,
        child_receipt_probe::Proof(std::uint8_t(90 + member.localID))}};
    return receipt;
}
struct Labels final { TDF_Label host, baseline, source; };
// Real XCAF host/source identities plus a distinct direct metadata child of
// the actual host for the typed baseline. Bare Main children without these
// identity attributes are not a valid native-baseline fixture.
inline Labels MakeLabels(const Handle(TDocStd_Document)& document,
                         const feature_pattern::Definition& definition) {
    Labels labels;
    labels.host = document->Main().FindChild(10, Standard_True);
    labels.baseline = TDF_TagSource::NewChild(labels.host);
    labels.source = document->Main().FindChild(12, Standard_True);
    const auto setUUID = [](const TDF_Label& label, const char* id,
                            const feature_pattern::UUID& value) {
        TDataStd_AsciiString::Set(label, Standard_GUID(id),
            TCollection_AsciiString(
                retained_solid::UUIDText(value).c_str()));
    };
    setUUID(document->Main(), "74386E4E-F620-498F-8092-E6D883AF33A4",
            definition.host.document);
    setUUID(labels.host, "0074F7C2-9EAA-4F89-B2DE-8716E155FF62",
            definition.host.entity);
    setUUID(labels.host, "3611F2B2-C694-4E12-AED8-A2A97A3D283B",
            definition.host.definition);
    setUUID(labels.source, "0074F7C2-9EAA-4F89-B2DE-8716E155FF62",
            definition.sourceCut.entity);
    setUUID(labels.source, "3611F2B2-C694-4E12-AED8-A2A97A3D283B",
            definition.sourceCut.definition);
    return labels;
}
// Stages the same native host-baseline value the production fixture uses,
// through the typed helper, with the generated identities persisted as
// evidence. ID(34) matches ReceiptFor's baselineRecipeIdentity.
inline bool StageTypedBaseline(const Labels& labels,
                               const feature_pattern::Definition& definition) {
    const std::string recipe = "Shapeyard R179 D4 codec host baseline v1";
    const std::vector<std::uint8_t> bytes(recipe.begin(), recipe.end());
    const TopoDS_Shape solid = BRepPrimAPI_MakeBox(8, 8, 2).Shape();
    return feature_pattern_baseline::Stage(labels.baseline, labels.host,
        definition.host, detached_tool_probe::ID(33),
        detached_tool_probe::ID(34), bytes, solid);
}
inline bool Stage(const Handle(TDocStd_Document)& document, const Labels& labels,
                  const feature_pattern::Definition& definition,
                  std::size_t childCount) {
    feature_pattern::Record patternRecord;
    if (!feature_pattern::Stage(document, definition, patternRecord)
        || !StageTypedBaseline(labels, definition)) return false;
    std::size_t staged = 0;
    for (const pattern::Member& member : definition.distribution.members) {
        if (member.state != pattern::MemberState::Active || staged == childCount) continue;
        const TDF_Label record = TDF_TagSource::NewChild(labels.host);
        if (!feature_pattern_child::Attach(record, labels.host, labels.baseline,
                labels.source, patternRecord.label, ReceiptFor(definition, member))) return false;
        ++staged;
    }
    return staged == childCount;
}
// Row-179 selectors 4 and 5: fail-closed guards for the bounded native
// TDF_Reference reader adapter (Core3DBoundedReferenceDriver in
// OcctDocument.mm). A persisted baseline reference record is located in the
// real saved bytes by its exact BinObjMgt framing derived from the actual
// baseline label path: the Length word (path depth plus one Standard_Integer
// payload cells), then PutLabel's tag count and the root-to-leaf tags of the
// staged baseline label. The staged fixture saves exactly two such records,
// one per child; any other occurrence count makes the signature ambiguous and
// awards nothing.
inline std::string SavedBaselineReferenceSignature(const TDF_Label& baseline) {
    std::string signature;
    TCollection_AsciiString entry;
    if (baseline.IsNull()) return signature;
    TDF_Tool::Entry(baseline, entry);
    const std::string text = entry.ToCString();
    std::vector<Standard_Integer> tags;
    std::size_t at = 0;
    while (at <= text.size()) {
        const std::size_t colon = text.find(':', at);
        const std::string tag =
            text.substr(at, colon == std::string::npos ? colon : colon - at);
        if (tag.empty()) return "";
        char* end = nullptr;
        const long value = std::strtol(tag.c_str(), &end, 10);
        if (end == nullptr || *end != 0 || value < 0 || value > 0x7fffffffL)
            return "";
        tags.push_back(Standard_Integer(value));
        if (colon == std::string::npos) break;
        at = colon + 1;
    }
    if (tags.empty()) return "";
    const auto appendInt = [&signature](Standard_Integer value) {
        char raw[sizeof(Standard_Integer)];
        std::memcpy(raw, &value, sizeof(raw));
        signature.append(raw, sizeof(raw));
    };
    appendInt(Standard_Integer(tags.size() + 1)
        * Standard_Integer(sizeof(Standard_Integer)));
    appendInt(Standard_Integer(tags.size()));
    for (const Standard_Integer tag : tags) appendInt(tag);
    return signature;
}
inline std::vector<std::size_t> FindAll(const std::string& bytes,
                                        const std::string& needle) {
    std::vector<std::size_t> offsets;
    for (std::size_t at = bytes.find(needle); at != std::string::npos;
         at = bytes.find(needle, at + 1)) offsets.push_back(at);
    return offsets;
}
inline void PatchInt(std::string& bytes, std::size_t offset,
                     Standard_Integer value) {
    char raw[sizeof(Standard_Integer)];
    std::memcpy(raw, &value, sizeof(raw));
    bytes.replace(offset, sizeof(raw), raw, sizeof(raw));
}
struct SavedPairs final {
    std::string bytes;
    std::string referenceSignature;
    std::vector<std::uint8_t> pattern;
    std::vector<std::vector<std::uint8_t>> children;
};
// Stages the exact two-child paired fixture through one committed command and
// saves it through the production application, capturing the canonical bytes a
// safe reopen must reproduce and the bounded reference signature derived from
// the actual staged baseline label path. No document or application handle
// escapes.
inline bool SaveValidPaired(const feature_pattern::Definition& definition,
                            SavedPairs& saved) {
    try {
        Document holder; const auto& document = holder.value;
        const Labels labels = MakeLabels(document, definition);
        document->NewCommand();
        if (!Stage(document, labels, definition, 2)
            || !document->CommitCommand()) return false;
        saved.referenceSignature =
            SavedBaselineReferenceSignature(labels.baseline);
        if (saved.referenceSignature.empty()) return false;
        std::vector<feature_pattern_child::PairedRecord> pairs;
        if (feature_pattern_child::ReadPairs(document, pairs)
                != feature_pattern_child::PairStatus::Valid
            || pairs.size() != 1 || pairs.front().children.size() != 2)
            return false;
        saved.pattern = pairs.front().pattern.bytes;
        for (const auto& child : pairs.front().children) {
            if (!child) return false;
            saved.children.push_back(child->canonicalBytes);
        }
        std::ostringstream output(std::ios::out | std::ios::binary);
        if (holder.app->SaveAs(document, output) != PCDM_SS_OK) return false;
        saved.bytes = output.str();
        return !saved.bytes.empty();
    } catch (...) { return false; }
}
// Positive control: a separate safe application must reopen the untouched
// bytes with the sticky rejection absent and reproduce the pattern and ordered
// child canonical bytes exactly.
inline bool SafeReopenExact(const SavedPairs& saved) {
    try {
        Handle(TDocStd_Application) reader = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(reader);
        Handle(TDocStd_Document) opened;
        std::istringstream input(saved.bytes, std::ios::in | std::ios::binary);
        Core3DBeginSafeBinaryRead();
        if (reader->Open(input, opened) != PCDM_RS_OK
            || Core3DSafeBinaryReadWasRejected() || opened.IsNull()) return false;
        std::vector<feature_pattern_child::PairedRecord> pairs;
        const bool exact = feature_pattern_child::ReadPairs(opened, pairs)
                == feature_pattern_child::PairStatus::Valid
            && pairs.size() == 1
            && pairs.front().pattern.bytes == saved.pattern
            && pairs.front().children.size() == saved.children.size()
            && std::equal(saved.children.begin(), saved.children.end(),
                pairs.front().children.begin(),
                [](const auto& bytes, const auto& child) {
                    return child && bytes == child->canonicalBytes;
                });
        reader->Close(opened);
        return exact;
    } catch (...) { return false; }
}
struct OpenObservation final {
    bool refused = false;
    bool sticky = false;
    bool noAuthority = false;
};
// Attempts one safe open of mutated bytes. refused = open failure or sticky
// safe-read rejection; noAuthority = no published document, or a published
// remainder with no undo/redo history, no open command and a pair census that
// cannot return Valid.
inline OpenObservation ObserveSafeOpen(const std::string& bytes) {
    OpenObservation observation;
    Handle(TDocStd_Application) reader = new TDocStd_Application();
    Core3DDefineSafeBinXCAFFormat(reader);
    Handle(TDocStd_Document) opened;
    std::istringstream input(bytes, std::ios::in | std::ios::binary);
    Core3DBeginSafeBinaryRead();
    const PCDM_ReaderStatus status = reader->Open(input, opened);
    observation.sticky = Core3DSafeBinaryReadWasRejected() == Standard_True;
    observation.refused = status != PCDM_RS_OK || observation.sticky;
    if (opened.IsNull()) {
        observation.noAuthority = true;
        return observation;
    }
    std::vector<feature_pattern_child::PairedRecord> pairs;
    observation.noAuthority = opened->GetAvailableUndos() == 0
        && opened->GetAvailableRedos() == 0 && !opened->HasOpenCommand()
        && feature_pattern_child::ReadPairs(opened, pairs)
            != feature_pattern_child::PairStatus::Valid;
    reader->Close(opened);
    return observation;
}
inline std::uint64_t Run(std::int32_t scenario) noexcept {
    try {
        using namespace feature_pattern_child;
        const auto definition = detached_tool_probe::Definition(2);
        if (scenario == 0) {
            Document holder; const auto& document = holder.value;
            const Labels labels = MakeLabels(document, definition);
            document->NewCommand(); std::uint64_t bits = Stage(document, labels, definition, 2) ? 1 : 0;
            std::vector<PairedRecord> pairs;
            if (ReadPairs(document, pairs) == PairStatus::Valid) bits |= 2;
            if (pairs.size() == 1 && pairs[0].host.IsEqual(labels.host)
                && pairs[0].source.IsEqual(labels.source)) bits |= 4;
            if (pairs.size() == 1 && pairs[0].baselineRecipe.IsEqual(labels.baseline)
                && pairs[0].pattern.definition.sourceCut == definition.sourceCut) bits |= 8;
            if (pairs.size() == 1 && pairs[0].children.size() == 2) bits |= 16;
            document->AbortCommand(); return bits;
        }
        if (scenario == 1) {
            Document holder; const auto& document = holder.value;
            document->NewCommand(); feature_pattern::Record legacy;
            std::uint64_t bits = feature_pattern::Stage(document, definition, legacy) ? 1 : 0;
            if (!legacy.label.IsNull() && document->CommitCommand()) bits |= 2;
            const auto time = document->GetData()->Time(); const auto bytes = legacy.bytes;
            std::vector<PairedRecord> pairs;
            if (ReadPairs(document, pairs) == PairStatus::Legacy && pairs.empty()) bits |= 4;
            feature_pattern::Record after;
            if (feature_pattern::ReadFeature(document, definition.feature, after)
                && after.bytes == bytes && document->GetData()->Time() == time) bits |= 8;
            if (ReadPairs(document, pairs) != PairStatus::Valid) bits |= 16;
            return bits;
        }
        if (scenario == 2) {
            Document holder; const auto& document = holder.value;
            const Labels labels = MakeLabels(document, definition);
            document->NewCommand(); std::uint64_t bits = Stage(document, labels, definition, 1) ? 1 : 0;
            std::vector<PairedRecord> pairs;
            if (ReadPairs(document, pairs) == PairStatus::Invalid) bits |= 2;
            // StorageDriver uses this same preflight and therefore refuses save.
            if (ReadPairs(document, pairs) == PairStatus::Invalid) bits |= 4;
            Handle(TDF_Data) foreign = new TDF_Data();
            const TDF_Label foreignSource = foreign->Root().FindChild(1, Standard_True);
            const TDF_Label record = TDF_TagSource::NewChild(labels.host);
            const auto receipt = ReceiptFor(definition, definition.distribution.members.front());
            if (!Attach(record, labels.host, labels.baseline, foreignSource,
                    document->Main().FindChild(feature_pattern::DocumentRootTag, Standard_False),
                    receipt)) bits |= 8;
            if (ReadPairs(document, pairs) == PairStatus::Invalid) bits |= 16;
            document->AbortCommand(); return bits;
        }
        if (scenario == 3) {
            Document holder; const auto& document = holder.value;
            const Labels labels = MakeLabels(document, definition);
            document->NewCommand(); if (!Stage(document, labels, definition, 2)) return 0;
            document->AbortCommand(); std::vector<PairedRecord> pairs; std::uint64_t bits = 0;
            if (ReadPairs(document, pairs) == PairStatus::Absent) bits |= 1;
            document->NewCommand(); if (!Stage(document, labels, definition, 2)
                || !document->CommitCommand()) return bits;
            if (ReadPairs(document, pairs) == PairStatus::Valid) bits |= 2;
            if (document->Undo() && ReadPairs(document, pairs) == PairStatus::Absent) bits |= 4;
            if (document->Redo() && ReadPairs(document, pairs) == PairStatus::Valid) bits |= 8;
            std::vector<std::vector<std::uint8_t>> exact;
            for (const auto& child : pairs.front().children) exact.push_back(child->canonicalBytes);
            std::ostringstream output(std::ios::out | std::ios::binary);
            if (holder.app->SaveAs(document, output) != PCDM_SS_OK) return bits;
            Document reloaded; reloaded.app->Close(reloaded.value); reloaded.value.Nullify();
            std::istringstream input(output.str(), std::ios::in | std::ios::binary);
            Core3DBeginSafeBinaryRead();
            if (reloaded.app->Open(input, reloaded.value) != PCDM_RS_OK
                || Core3DSafeBinaryReadWasRejected() || reloaded.value.IsNull()) return bits;
            std::vector<PairedRecord> reread;
            if (ReadPairs(reloaded.value, reread) == PairStatus::Valid
                && reread.front().pattern.bytes == pairs.front().pattern.bytes
                && reread.front().children.size() == exact.size()
                && std::equal(exact.begin(), exact.end(), reread.front().children.begin(),
                    [](const auto& bytes, const auto& child) {
                        return child && bytes == child->canonicalBytes;
                    })) bits |= 16;
            return bits;
        }
        if (scenario == 4) {
            // Malformed and truncated saved baseline reference records must be
            // refused by the bounded reader without edit authority or partial
            // readable state.
            SavedPairs saved;
            if (!SaveValidPaired(definition, saved)) return 0;
            const auto records = FindAll(saved.bytes,
                saved.referenceSignature);
            if (records.size() != 2) return 0;
            const std::size_t record = records.front();
            std::uint64_t bits = SafeReopenExact(saved) ? 1ULL : 0ULL;
            // Malformed: the encoded tag count claims one more path tag than
            // the record length admits.
            std::string malformed = saved.bytes;
            PatchInt(malformed, record + sizeof(Standard_Integer), 4);
            const OpenObservation malformedOpen = ObserveSafeOpen(malformed);
            if (malformedOpen.refused && malformedOpen.sticky) bits |= 2ULL;
            // Truncated: the record length word drops the trailing path tags.
            std::string truncated = saved.bytes;
            PatchInt(truncated, record,
                2 * Standard_Integer(sizeof(Standard_Integer)));
            const OpenObservation truncatedOpen = ObserveSafeOpen(truncated);
            if (truncatedOpen.refused && truncatedOpen.sticky) bits |= 4ULL;
            if (malformedOpen.noAuthority && truncatedOpen.noAuthority) bits |= 8ULL;
            if ((bits & 0x0fULL) == 0x0fULL && SafeReopenExact(saved)) bits |= 16ULL;
            return bits;
        }
        if (scenario == 5) {
            // An over-deep encoded label path must be refused by the depth
            // budget alone, without edit authority or partial readable state.
            SavedPairs saved;
            if (!SaveValidPaired(definition, saved)) return 0;
            const auto records = FindAll(saved.bytes,
                saved.referenceSignature);
            if (records.size() != 2) return 0;
            const std::size_t record = records.front();
            std::uint64_t bits = SafeReopenExact(saved) ? 1ULL : 0ULL;
            // Over-deep: the encoded tag count exceeds the reader's
            // kMaximumPersistentReferencePathDepth (64) budget by one.
            constexpr Standard_Integer overDeep = 64 + 1;
            std::string deep = saved.bytes;
            PatchInt(deep, record + sizeof(Standard_Integer), overDeep);
            const OpenObservation deepOpen = ObserveSafeOpen(deep);
            if (deepOpen.refused && deepOpen.sticky) bits |= 2ULL;
            // Depth isolation: the declared record length is extended to hold
            // the full encoded path, so the size-consistency bound admits the
            // record and the depth budget alone refuses it. Attempted only
            // when enough trailing stream bytes remain for the wider record.
            const Standard_Integer isolatedLength =
                (overDeep + 1) * Standard_Integer(sizeof(Standard_Integer));
            OpenObservation isolatedOpen;
            if (saved.bytes.size() - record
                >= 3 * sizeof(Standard_Integer) + std::size_t(isolatedLength)) {
                std::string isolated = saved.bytes;
                PatchInt(isolated, record, isolatedLength);
                PatchInt(isolated, record + sizeof(Standard_Integer), overDeep);
                isolatedOpen = ObserveSafeOpen(isolated);
            }
            if (isolatedOpen.refused && isolatedOpen.sticky) bits |= 4ULL;
            if (deepOpen.noAuthority && isolatedOpen.noAuthority) bits |= 8ULL;
            if ((bits & 0x0fULL) == 0x0fULL && SafeReopenExact(saved)) bits |= 16ULL;
            return bits;
        }
    } catch (...) {}
    return 0;
}
} // namespace paired_receipt_probe

extern "C" __attribute__((used, visibility("default"))) inline std::uint64_t
Core3DDebugFeaturePatternPairedPersistenceProbe(std::int32_t scenario) noexcept {
    return paired_receipt_probe::Run(scenario);
}

namespace child_reference_probe {
using paired_receipt_probe::Document;
using paired_receipt_probe::Labels;

inline bool AttachOrdered(const TDF_Label& record, const TDF_Label& host,
                          const TDF_Label& baseline, const TDF_Label& source,
                          const TDF_Label& patternRecord,
                          const feature_pattern_child::Receipt& receipt,
                          bool childFirst) {
    using namespace feature_pattern_child;
    std::vector<std::uint8_t> bytes;
    if (!Encode(receipt, bytes)) return false;
    auto payload = std::make_shared<Payload>();
    payload->receipt = receipt; payload->canonicalBytes = std::move(bytes);
    payload->host = host; payload->baselineRecipe = baseline;
    payload->source = source; payload->patternRecord = patternRecord;
    Handle(Attribute) attribute = new Attribute();
    if (!attribute->initialize(std::move(payload))) return false;
    if (childFirst) record.AddAttribute(attribute);
    TDF_Reference::Set(record, baseline);
    if (!childFirst) record.AddAttribute(attribute);
    return SetLink(record, SourceLinkMarker, source)
        && SetLink(record, PatternLinkMarker, patternRecord);
}

inline bool StageBothOrders(const Handle(TDocStd_Document)& document,
                            std::vector<TDF_Label>& records) {
    using namespace feature_pattern_child;
    records.clear();
    const auto definition = detached_tool_probe::Definition(2);
    const Labels labels = paired_receipt_probe::MakeLabels(document, definition);
    feature_pattern::Record patternRecord;
    if (!feature_pattern::Stage(document, definition, patternRecord)
        || !paired_receipt_probe::StageTypedBaseline(labels, definition))
        return false;
    std::size_t index = 0;
    for (const auto& member : definition.distribution.members) {
        if (member.state != pattern::MemberState::Active) continue;
        const TDF_Label record = TDF_TagSource::NewChild(labels.host);
        if (!AttachOrdered(record, labels.host, labels.baseline, labels.source,
                patternRecord.label,
                paired_receipt_probe::ReceiptFor(definition, member),
                (index++ & 1U) == 0)) return false;
        records.push_back(record);
    }
    return records.size() == 2;
}

inline bool ExactPairs(const std::vector<feature_pattern_child::PairedRecord>& left,
                       const std::vector<feature_pattern_child::PairedRecord>& right) {
    const auto sameEntry = [](const TDF_Label& first, const TDF_Label& second) {
        TCollection_AsciiString firstEntry, secondEntry;
        TDF_Tool::Entry(first, firstEntry);
        TDF_Tool::Entry(second, secondEntry);
        return !firstEntry.IsEmpty() && firstEntry.IsEqual(secondEntry);
    };
    if (left.size() != 1 || right.size() != 1
        || left.front().pattern.bytes != right.front().pattern.bytes
        || left.front().children.size() != right.front().children.size()
        || !sameEntry(left.front().baselineRecipe,
                      right.front().baselineRecipe)
        || !sameEntry(left.front().source, right.front().source)) return false;
    for (std::size_t index = 0; index < left.front().children.size(); ++index)
        if (!left.front().children[index] || !right.front().children[index]
            || left.front().children[index]->canonicalBytes
                != right.front().children[index]->canonicalBytes) return false;
    return true;
}

inline bool SaveOpen(const Document& writer,
                     Handle(TDocStd_Application)& reader,
                     Handle(TDocStd_Document)& opened) {
    std::ostringstream output(std::ios::out | std::ios::binary);
    if (writer.app->SaveAs(writer.value, output) != PCDM_SS_OK) return false;
    reader = new TDocStd_Application();
    Core3DDefineSafeBinXCAFFormat(reader);
    std::istringstream input(output.str(), std::ios::in | std::ios::binary);
    Core3DBeginSafeBinaryRead();
    return reader->Open(input, opened) == PCDM_RS_OK
        && !Core3DSafeBinaryReadWasRejected() && !opened.IsNull();
}

inline bool ValidBothOrdersAndRoundTrip(bool secondRoundTrip) {
    using namespace feature_pattern_child;
    Document writer;
    writer.value->NewCommand();
    std::vector<TDF_Label> records;
    if (!StageBothOrders(writer.value, records)
        || !writer.value->CommitCommand()) return false;
    std::vector<PairedRecord> original;
    if (ReadPairs(writer.value, original) != PairStatus::Valid) return false;
    Handle(TDocStd_Application) firstReader;
    Handle(TDocStd_Document) firstOpened;
    if (!SaveOpen(writer, firstReader, firstOpened)) return false;
    std::vector<PairedRecord> first;
    if (ReadPairs(firstOpened, first) != PairStatus::Valid
        || !ExactPairs(original, first)) return false;
    if (!secondRoundTrip) {
        firstReader->Close(firstOpened);
        return true;
    }
    std::ostringstream secondBytes(std::ios::out | std::ios::binary);
    if (firstReader->SaveAs(firstOpened, secondBytes) != PCDM_SS_OK) return false;
    Handle(TDocStd_Application) secondReader = new TDocStd_Application();
    Core3DDefineSafeBinXCAFFormat(secondReader);
    Handle(TDocStd_Document) secondOpened;
    std::istringstream secondInput(secondBytes.str(), std::ios::in | std::ios::binary);
    Core3DBeginSafeBinaryRead();
    const bool opened = secondReader->Open(secondInput, secondOpened) == PCDM_RS_OK
        && !Core3DSafeBinaryReadWasRejected() && !secondOpened.IsNull();
    std::vector<PairedRecord> second;
    const bool exact = opened
        && ReadPairs(secondOpened, second) == PairStatus::Valid
        && ExactPairs(first, second);
    if (!secondOpened.IsNull()) secondReader->Close(secondOpened);
    if (!firstOpened.IsNull()) firstReader->Close(firstOpened);
    return exact;
}

inline bool InvalidBaselineControl(std::int32_t mode) {
    using namespace feature_pattern_child;
    Document holder;
    holder.value->NewCommand();
    std::vector<TDF_Label> records;
    if (!StageBothOrders(holder.value, records) || records.empty()) return false;
    const TDF_Label record = records.front();
    const auto definition = detached_tool_probe::Definition(2);
    const Labels labels = paired_receipt_probe::MakeLabels(holder.value, definition);
    if (mode == 0) {
        record.ForgetAttribute(TDF_Reference::GetID());
    } else if (mode == 1) {
        record.ForgetAttribute(TDF_Reference::GetID());
        record.AddAttribute(new TDF_Reference());
    } else if (mode == 2) {
        Handle(TDF_Data) foreign = new TDF_Data();
        TDF_Reference::Set(record,
            foreign->Root().FindChild(1, Standard_True));
    } else {
        TDF_Reference::Set(record, labels.source);
    }
    std::vector<PairedRecord> pairs;
    const bool refused = ReadPairs(holder.value, pairs) == PairStatus::Invalid;
    holder.value->AbortCommand();
    return refused;
}

inline std::uint64_t Run(std::int32_t scenario) noexcept {
    try {
        if (scenario == 0) {
            std::uint64_t bits = ValidBothOrdersAndRoundTrip(false) ? 1ULL : 0ULL;
            if (ValidBothOrdersAndRoundTrip(true)) bits |= 2ULL;
            Document holder; holder.value->NewCommand();
            std::vector<TDF_Label> records;
            if (StageBothOrders(holder.value, records)) bits |= 4ULL;
            std::vector<feature_pattern_child::PairedRecord> pairs;
            if (feature_pattern_child::ReadPairs(holder.value, pairs)
                == feature_pattern_child::PairStatus::Valid) bits |= 8ULL;
            if (pairs.size() == 1 && pairs.front().children.size() == 2
                && records.size() == 2) bits |= 16ULL;
            holder.value->AbortCommand();
            return bits;
        }
        if (scenario == 1) {
            std::uint64_t bits = InvalidBaselineControl(0) ? 1ULL : 0ULL;
            if (InvalidBaselineControl(1)) bits |= 2ULL;
            if (InvalidBaselineControl(2)) bits |= 4ULL;
            if (InvalidBaselineControl(3)) bits |= 8ULL;
            if (bits == 0x0f) bits |= 16ULL;
            return bits;
        }
        if (scenario == 2) {
            // The production safe reader supplies the bounded native-reference
            // driver. Truncation, an appended record tail, and a damaged native
            // header must all refuse without publishing a document.
            Document writer; writer.value->NewCommand();
            std::vector<TDF_Label> records;
            if (!StageBothOrders(writer.value, records)
                || !writer.value->CommitCommand()) return 0;
            std::ostringstream output(std::ios::out | std::ios::binary);
            if (writer.app->SaveAs(writer.value, output) != PCDM_SS_OK) return 0;
            const std::string valid = output.str();
            const auto refuses = [](const std::string& bytes) {
                Handle(TDocStd_Application) reader = new TDocStd_Application();
                Core3DDefineSafeBinXCAFFormat(reader);
                Handle(TDocStd_Document) opened;
                std::istringstream input(bytes, std::ios::in | std::ios::binary);
                Core3DBeginSafeBinaryRead();
                const auto status = reader->Open(input, opened);
                if (!opened.IsNull()) reader->Close(opened);
                return status != PCDM_RS_OK || Core3DSafeBinaryReadWasRejected();
            };
            std::uint64_t bits = !valid.empty() ? 1ULL : 0ULL;
            if (valid.size() > 16 && refuses(valid.substr(0, valid.size() - 1))) bits |= 2ULL;
            std::string trailing = valid; trailing.append(4096, '\x7f');
            if (refuses(trailing)) bits |= 4ULL;
            std::string malformed = valid;
            if (malformed.size() > 24) malformed[malformed.size() / 2] ^= char(0xff);
            if (refuses(malformed)) bits |= 8ULL;
            if ((bits & 0x0f) == 0x0f && ValidBothOrdersAndRoundTrip(false)) bits |= 16ULL;
            return bits;
        }
        if (scenario == 3) return ValidBothOrdersAndRoundTrip(true) ? 0x1f : 0;
    } catch (...) {}
    return 0;
}
} // namespace child_reference_probe

extern "C" __attribute__((used, visibility("default"))) inline std::uint64_t
Core3DDebugFeaturePatternReferenceResolutionProbe(
    std::int32_t scenario) noexcept {
    return child_reference_probe::Run(scenario);
}

namespace baseline_probe {
using paired_receipt_probe::Labels;

inline feature_pattern::Definition Def() {
    return detached_tool_probe::Definition(2);
}

inline feature_pattern_baseline::Envelope EnvelopeFor(
    const feature_pattern::Definition& definition) {
    feature_pattern_baseline::Envelope envelope;
    envelope.document = definition.host.document;
    envelope.hostEntity = definition.host.entity;
    envelope.hostDefinition = definition.host.definition;
    envelope.retainedRecipeFeature = detached_tool_probe::ID(33);
    envelope.baselineRecipeIdentity = detached_tool_probe::ID(34);
    const std::string recipe = "Shapeyard R179 D4 codec host baseline v1";
    envelope.exactRecipe.assign(recipe.begin(), recipe.end());
    return envelope;
}

inline TopoDS_Shape Solid() {
    return BRepPrimAPI_MakeBox(8, 8, 2).Shape();
}

// Manual typed-baseline assembly for order and negative variants. The payload
// host is the label's actual father, so the production writer persists even
// deliberately misplaced fixtures and the safe reader observes the real
// corruption instead of a writer-side refusal.
inline bool AttachRaw(const TDF_Label& label,
                      const feature_pattern_baseline::Envelope& envelope,
                      const TopoDS_Shape& solid, bool attributeFirst) {
    std::vector<std::uint8_t> bytes;
    if (label.IsNull() || label.Father().IsNull()
        || !feature_pattern_baseline::Encode(envelope, bytes)) return false;
    auto payload = std::make_shared<feature_pattern_baseline::Payload>();
    payload->envelope = envelope;
    payload->canonicalBytes = std::move(bytes);
    payload->host = label.Father();
    Handle(feature_pattern_baseline::Attribute) attribute =
        new feature_pattern_baseline::Attribute();
    if (!attribute->initialize(std::move(payload))) return false;
    if (attributeFirst) label.AddAttribute(attribute);
    if (!solid.IsNull()) TNaming_Builder(label).Select(solid, solid);
    if (!attributeFirst) label.AddAttribute(attribute);
    return true;
}

// DEBUG fixture writer only. Preserve canonical SYFC payload bytes even
// when the fixture deliberately removed a native reference. The real SYFC
// writer must keep refusing that state; production and DEBUG safe readers
// must inspect the missing reference after native traversal.
class StockChildStorageDriver final : public BinMDF_ADriver {
public:
    explicit StockChildStorageDriver(const Handle(Message_Messenger)& messenger)
        : BinMDF_ADriver(messenger,
            STANDARD_TYPE(feature_pattern_child::Attribute)->Name()) {}
    Handle(TDF_Attribute) NewEmpty() const override {
        return new feature_pattern_child::Attribute();
    }
    const Handle(Standard_Type)& SourceType() const override {
        return STANDARD_TYPE(feature_pattern_child::Attribute);
    }
    Standard_Boolean Paste(const BinObjMgt_Persistent&,
        const Handle(TDF_Attribute)&, BinObjMgt_RRelocationTable&) const override {
        return Standard_False; // Never a retrieval driver.
    }
    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&) const override {
        const auto attribute =
            Handle(feature_pattern_child::Attribute)::DownCast(source);
        std::vector<std::uint8_t> canonical;
        if (attribute.IsNull() || attribute->Label().IsNull()
            || !attribute->value()
            || !feature_pattern_child::Encode(
                attribute->value()->receipt, canonical)
            || canonical != attribute->value()->canonicalBytes)
            Standard_Failure::Raise("Invalid stock SYFC fixture payload");
        target << Standard_Integer(1) << Standard_Integer(canonical.size());
        target.PutByteArray(canonical.data(), Standard_Integer(canonical.size()));
    }
};

// Persists invalid probe states faithfully without a document preflight.
// Only the fixture SYFC writer bypasses native-link validation; the SYFB
// attribute writer and both safe reader registrations remain production code.
// This class is inside the file's DEBUG guard and the probe namespace.
class StockStorageDriver final : public BinXCAFDrivers_DocumentStorageDriver {
public:
    Handle(BinMDF_ADriverTable) AttributeDrivers(
        const Handle(Message_Messenger)& messenger) override {
        auto table =
            BinXCAFDrivers_DocumentStorageDriver::AttributeDrivers(messenger);
        table->AddDriver(new StockChildStorageDriver(messenger));
        feature_pattern_baseline::Register(table, messenger,
            std::make_shared<feature_pattern_baseline::Budget>());
        return table;
    }
};

inline Handle(TDocStd_Application) ProductionApp() {
    Handle(TDocStd_Application) app = new TDocStd_Application();
    Core3DDefineSafeBinXCAFFormat(app);
    return app;
}

inline Handle(TDocStd_Application) StockWriter() {
    Handle(TDocStd_Application) app = new TDocStd_Application();
    app->DefineFormat(TCollection_AsciiString("BinXCAF"),
        TCollection_AsciiString("Binary XCAF Document"),
        TCollection_AsciiString("xbf"),
        new BinDrivers_DocumentRetrievalDriver(),
        new StockStorageDriver());
    return app;
}

struct Holder final {
    Handle(TDocStd_Application) app;
    Handle(TDocStd_Document) value;
    explicit Holder(bool stock) {
        app = stock ? StockWriter() : ProductionApp();
        app->NewDocument(TCollection_ExtendedString("BinXCAF"), value);
        if (value.IsNull()) throw std::runtime_error("baseline probe document");
        value->SetUndoLimit(8);
    }
    ~Holder() noexcept {
        try { if (!value.IsNull()) app->Close(value); } catch (...) {}
    }
};

// Builds one paired document with the typed baseline; mode 0 is the valid
// control and modes 1..13 stage exactly one corrupted or reordered variant.
// Returns false only on fixture construction failure: the mutated state is
// the intended product of a true return.
inline bool Build(const Handle(TDocStd_Document)& document, int mode,
                  Labels& labels,
                  feature_pattern::Definition& definition) {
    definition = Def();
    labels = paired_receipt_probe::MakeLabels(document, definition);
    feature_pattern::Record patternRecord;
    if (!feature_pattern::Stage(document, definition, patternRecord))
        return false;
    const auto envelope = EnvelopeFor(definition);
    const TopoDS_Shape solid = Solid();
    const auto attachChildren = [&](const TDF_Label& baseline,
                                    bool tamperIdentity) {
        std::size_t index = 0;
        for (const pattern::Member& member :
                definition.distribution.members) {
            if (member.state != pattern::MemberState::Active) continue;
            auto receipt =
                paired_receipt_probe::ReceiptFor(definition, member);
            if (tamperIdentity && index == 0)
                receipt.baselineRecipeIdentity = detached_tool_probe::ID(77);
            const TDF_Label record = TDF_TagSource::NewChild(labels.host);
            if (!feature_pattern_child::Attach(record, labels.host, baseline,
                    labels.source, patternRecord.label, receipt)) return false;
            ++index;
        }
        return true;
    };
    switch (mode) {
        case 0: // valid control
            return paired_receipt_probe::StageTypedBaseline(labels, definition)
                && attachChildren(labels.baseline, false);
        case 1: { // wrong placement: the baseline is not under the host
            const TDF_Label wrong =
                document->Main().FindChild(99, Standard_True);
            return AttachRaw(wrong, envelope, solid, true)
                && attachChildren(wrong, false);
        }
        case 2: // orphan typed attribute on an unrelated label
            if (!paired_receipt_probe::StageTypedBaseline(labels, definition)
                || !attachChildren(labels.baseline, false)) return false;
            return AttachRaw(document->Main().FindChild(77, Standard_True),
                envelope, solid, true);
        case 3: { // a second incompatible baseline on the same host
            if (!paired_receipt_probe::StageTypedBaseline(labels, definition)
                || !attachChildren(labels.baseline, false)) return false;
            auto second = envelope;
            second.baselineRecipeIdentity = detached_tool_probe::ID(77);
            return AttachRaw(TDF_TagSource::NewChild(labels.host),
                second, solid, true);
        }
        case 4: // missing typed payload on the baseline label
            if (!paired_receipt_probe::StageTypedBaseline(labels, definition)
                || !attachChildren(labels.baseline, false)) return false;
            labels.baseline.ForgetAttribute(
                feature_pattern_baseline::AttributeID());
            return !labels.baseline.IsAttribute(
                feature_pattern_baseline::AttributeID());
        case 5: // same-document wrong host identity
            if (!paired_receipt_probe::StageTypedBaseline(labels, definition)
                || !attachChildren(labels.baseline, false)) return false;
            TDataStd_AsciiString::Set(labels.host,
                Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"),
                TCollection_AsciiString(retained_solid::UUIDText(
                    detached_tool_probe::ID(99)).c_str()));
            return true;
        case 6: // foreign document identity
            if (!paired_receipt_probe::StageTypedBaseline(labels, definition)
                || !attachChildren(labels.baseline, false)) return false;
            TDataStd_AsciiString::Set(document->Main(),
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"),
                TCollection_AsciiString(retained_solid::UUIDText(
                    detached_tool_probe::ID(98)).c_str()));
            return true;
        case 7: // mismatched child baseline identity
            return paired_receipt_probe::StageTypedBaseline(labels, definition)
                && attachChildren(labels.baseline, true);
        case 8: // null TNaming on the baseline label
            if (!paired_receipt_probe::StageTypedBaseline(labels, definition)
                || !attachChildren(labels.baseline, false)) return false;
            labels.baseline.ForgetAttribute(TNaming_NamedShape::GetID());
            return !labels.baseline.IsAttribute(TNaming_NamedShape::GetID());
        case 9: { // nonsolid TNaming on the baseline label
            if (!paired_receipt_probe::StageTypedBaseline(labels, definition)
                || !attachChildren(labels.baseline, false)) return false;
            TopExp_Explorer explorer(solid, TopAbs_FACE);
            if (!explorer.More()) return false;
            const TopoDS_Shape face = explorer.Current();
            if (face.IsNull() || face.ShapeType() != TopAbs_FACE) return false;
            TNaming_Builder(labels.baseline).Select(face, face);
            return true;
        }
        case 10: { // broken child TDF_Reference
            if (!paired_receipt_probe::StageTypedBaseline(labels, definition)
                || !attachChildren(labels.baseline, false)) return false;
            for (TDF_ChildIterator record(labels.host, Standard_False);
                 record.More(); record.Next()) {
                Handle(feature_pattern_child::Attribute) attribute;
                if (!record.Value().FindAttribute(
                        feature_pattern_child::AttributeID(), attribute)
                    || attribute.IsNull()) continue;
                record.Value().ForgetAttribute(TDF_Reference::GetID());
                return !record.Value().IsAttribute(TDF_Reference::GetID());
            }
            return false;
        }
        case 11: { // raw default byte-array baseline control
            const TDF_Label raw = labels.baseline;
            const auto bytes = TDataStd_ByteArray::Set(raw, 0,
                Standard_Integer(envelope.exactRecipe.size()) - 1,
                Standard_False);
            if (bytes.IsNull()) return false;
            for (std::size_t index = 0; index < envelope.exactRecipe.size();
                 ++index)
                bytes->SetValue(Standard_Integer(index),
                    Standard_Byte(envelope.exactRecipe[index]));
            TNaming_Builder(raw).Select(solid, solid);
            return attachChildren(raw, false);
        }
        case 12: // valid, baseline attribute before the bound solid
            return AttachRaw(labels.baseline, envelope, solid, true)
                && attachChildren(labels.baseline, false);
        case 13: // valid, bound solid before the baseline attribute
            return AttachRaw(labels.baseline, envelope, solid, false)
                && attachChildren(labels.baseline, false);
        default: return false;
    }
}

// Every refusal leg for one corrupted mode: the shared strict census and the
// owned-frame snapshot/save gate refuse in memory, production normal save is
// refused before writing without advancing document time or history, and both
// the production and the DEBUG legacy safe registrations refuse the
// stock-persisted bytes.
inline bool RefusalLegs(int mode) {
    {
        Holder production(false);
        const auto& document = production.value;
        Labels labels; feature_pattern::Definition definition;
        document->NewCommand();
        if (!Build(document, mode, labels, definition)
            || !document->CommitCommand()) return false;
        const auto time = document->GetData()->Time();
        const Standard_Integer undos = document->GetAvailableUndos();
        std::vector<feature_pattern_child::BaselineRecord> baselines;
        Standard_Size frameBytes = 0;
        bool saveRefused = false;
        try {
            std::ostringstream output(std::ios::out | std::ios::binary);
            saveRefused =
                production.app->SaveAs(document, output) != PCDM_SS_OK;
        } catch (...) { saveRefused = true; }
        if (feature_pattern_child::ReadBaselines(document, baselines)
                != feature_pattern_child::BaselineStatus::Invalid
            || Core3DValidateOwnedFrameUsage(document, frameBytes,
                Standard_Size(64U * 1024U * 1024U))
            || !saveRefused
            || document->GetData()->Time() != time
            || document->GetAvailableUndos() != undos
            || document->HasOpenCommand()) return false;
    }
    std::string bytes;
    {
        Holder stock(true);
        const auto& document = stock.value;
        Labels labels; feature_pattern::Definition definition;
        document->NewCommand();
        if (!Build(document, mode, labels, definition)
            || !document->CommitCommand()) return false;
        std::ostringstream output(std::ios::out | std::ios::binary);
        if (stock.app->SaveAs(document, output) != PCDM_SS_OK) return false;
        bytes = output.str();
    }
    if (bytes.empty()) return false;
    const auto refused = [](const Handle(TDocStd_Application)& reader,
                            const std::string& data) {
        Handle(TDocStd_Document) opened;
        std::istringstream input(data, std::ios::in | std::ios::binary);
        Core3DBeginSafeBinaryRead();
        const PCDM_ReaderStatus status = reader->Open(input, opened);
        const bool sticky =
            Core3DSafeBinaryReadWasRejected() == Standard_True;
        if (!opened.IsNull()) reader->Close(opened);
        return status != PCDM_RS_OK || sticky;
    };
    {
        const Handle(TDocStd_Application) production = ProductionApp();
        if (!refused(production, bytes)) return false;
    }
    {
        Handle(TDocStd_Application) legacy = new TDocStd_Application();
        Core3DDebugDefineLegacyReceiptFormats(legacy);
        if (!refused(legacy, bytes)) return false;
    }
    return true;
}

inline bool SameEnvelope(const feature_pattern_baseline::Envelope& left,
                         const feature_pattern_baseline::Envelope& right) {
    return left.document == right.document
        && left.hostEntity == right.hostEntity
        && left.hostDefinition == right.hostDefinition
        && left.retainedRecipeFeature == right.retainedRecipeFeature
        && left.baselineRecipeIdentity == right.baselineRecipeIdentity
        && left.exactRecipe == right.exactRecipe;
}

inline std::vector<std::vector<std::uint8_t>> ExpectedChildren(
    const feature_pattern::Definition& definition) {
    std::vector<std::vector<std::uint8_t>> children;
    for (const pattern::Member& member : definition.distribution.members) {
        if (member.state != pattern::MemberState::Active) continue;
        std::vector<std::uint8_t> bytes;
        if (!feature_pattern_child::Encode(
                paired_receipt_probe::ReceiptFor(definition, member), bytes))
            return {};
        children.push_back(std::move(bytes));
    }
    return children;
}

inline bool ReopenExact(const Handle(TDocStd_Application)& reader,
                        const std::string& bytes,
                        const feature_pattern_baseline::Envelope& expected,
                        const std::vector<std::vector<std::uint8_t>>& children) {
    Handle(TDocStd_Document) opened;
    std::istringstream input(bytes, std::ios::in | std::ios::binary);
    Core3DBeginSafeBinaryRead();
    const PCDM_ReaderStatus status = reader->Open(input, opened);
    const bool sticky = Core3DSafeBinaryReadWasRejected() == Standard_True;
    bool exact = status == PCDM_RS_OK && !sticky && !opened.IsNull();
    if (exact) {
        std::vector<feature_pattern_child::PairedRecord> pairs;
        std::vector<feature_pattern_child::BaselineRecord> baselines;
        exact = feature_pattern_child::ReadPairs(opened, pairs)
                == feature_pattern_child::PairStatus::Valid
            && pairs.size() == 1
            && feature_pattern_child::ReadBaselines(opened, baselines)
                == feature_pattern_child::BaselineStatus::Valid
            && baselines.size() == 1 && baselines.front().value
            && SameEnvelope(baselines.front().value->envelope, expected)
            && pairs.front().children.size() == children.size();
        if (exact)
            for (std::size_t index = 0; index < children.size(); ++index)
                if (!pairs.front().children[index]
                    || pairs.front().children[index]->canonicalBytes
                        != children[index]) {
                    exact = false; break;
                }
    }
    if (!opened.IsNull()) reader->Close(opened);
    return exact;
}

// The valid control must cross every boundary the corrupted modes fail: the
// census and owned-frame gate pass, production normal save succeeds and a
// separate safe application reopens the exact baseline and child receipts.
inline bool ValidControl() {
    Holder production(false);
    const auto& document = production.value;
    Labels labels; feature_pattern::Definition definition;
    document->NewCommand();
    if (!Build(document, 0, labels, definition)
        || !document->CommitCommand()) return false;
    std::vector<feature_pattern_child::PairedRecord> pairs;
    std::vector<feature_pattern_child::BaselineRecord> baselines;
    Standard_Size frameBytes = 0;
    if (feature_pattern_child::ReadPairs(document, pairs)
            != feature_pattern_child::PairStatus::Valid
        || feature_pattern_child::ReadBaselines(document, baselines)
            != feature_pattern_child::BaselineStatus::Valid
        || baselines.size() != 1 || !baselines.front().value
        || !Core3DValidateOwnedFrameUsage(document, frameBytes,
            Standard_Size(64U * 1024U * 1024U))) return false;
    const auto envelope = baselines.front().value->envelope;
    const auto children = ExpectedChildren(definition);
    std::ostringstream output(std::ios::out | std::ios::binary);
    if (children.empty()
        || production.app->SaveAs(document, output) != PCDM_SS_OK)
        return false;
    const Handle(TDocStd_Application) reader = ProductionApp();
    return ReopenExact(reader, output.str(), envelope, children);
}

inline bool SaveMode(int mode, std::string& bytes) {
    bytes.clear();
    Holder production(false);
    const auto& document = production.value;
    Labels labels; feature_pattern::Definition definition;
    document->NewCommand();
    if (!Build(document, mode, labels, definition)
        || !document->CommitCommand()) return false;
    std::ostringstream output(std::ios::out | std::ios::binary);
    if (production.app->SaveAs(document, output) != PCDM_SS_OK) return false;
    bytes = output.str();
    return !bytes.empty();
}

inline bool RefusedRead(const Handle(TDocStd_Application)& reader,
                        const std::string& data, bool& sticky) {
    Handle(TDocStd_Document) opened;
    std::istringstream input(data, std::ios::in | std::ios::binary);
    Core3DBeginSafeBinaryRead();
    const PCDM_ReaderStatus status = reader->Open(input, opened);
    sticky = Core3DSafeBinaryReadWasRejected() == Standard_True;
    if (!opened.IsNull()) reader->Close(opened);
    return status != PCDM_RS_OK || sticky;
}

// Alter only the native SYFB record's declared payload count. Keep the
// archive, record length, label terminators and shape-section offsets intact.
// +1 promises a byte beyond the record; -1 leaves a byte after the payload.
// Refuse to construct a fixture unless the native header and complete original
// canonical payload have first been identified exactly.
inline bool BaselineCountMismatch(const std::string& valid,
    const std::vector<std::uint8_t>& canonical, Standard_Integer delta,
    std::string& corrupted) {
    corrupted.clear();
    static_assert(sizeof(Standard_Integer) == 4, "Native framing width changed");
    if (canonical.empty() || canonical.size() > std::size_t(INT_MAX - 8)
        || (delta != 1 && delta != -1)) return false;
    const std::string needle(reinterpret_cast<const char*>(canonical.data()),
        canonical.size());
    const std::size_t at = valid.find(needle);
    if (at == std::string::npos || at < 20
        || valid.find(needle, at + 1) != std::string::npos) return false;
    Standard_Integer length = 0, schema = 0, count = 0;
    std::memcpy(&length, valid.data() + at - 12, sizeof(length));
    std::memcpy(&schema, valid.data() + at - 8, sizeof(schema));
    std::memcpy(&count, valid.data() + at - 4, sizeof(count));
    if (schema != 1 || count != Standard_Integer(canonical.size())
        || length != count + 8 || count <= 1) return false;
    const Standard_Integer replacement = count + delta;
    corrupted = valid;
    std::memcpy(corrupted.data() + at - 4, &replacement, sizeof(replacement));
    return true;
}

inline std::uint64_t Run(std::int32_t scenario) noexcept {
    try {
        if (scenario == 0) {
            std::uint64_t bits = 0;
            if (RefusalLegs(1)) bits |= 0x001ULL;
            if (RefusalLegs(2)) bits |= 0x002ULL;
            if (RefusalLegs(3)) bits |= 0x004ULL;
            if (RefusalLegs(4)) bits |= 0x008ULL;
            if (RefusalLegs(5)) bits |= 0x010ULL;
            if (RefusalLegs(6)) bits |= 0x020ULL;
            if (RefusalLegs(7)) bits |= 0x040ULL;
            if (RefusalLegs(8)) bits |= 0x080ULL;
            if (RefusalLegs(9)) bits |= 0x100ULL;
            if (RefusalLegs(10)) bits |= 0x200ULL;
            if (RefusalLegs(11)) bits |= 0x400ULL;
            if (ValidControl()) bits |= 0x800ULL;
            return bits;
        }
        if (scenario == 1) {
            using namespace feature_pattern_baseline;
            const auto definition = Def();
            const auto envelope = EnvelopeFor(definition);
            std::vector<std::uint8_t> canonical;
            if (!Encode(envelope, canonical)) return 0;
            Envelope decoded;
            bool codec = Decode(canonical, decoded)
                && SameEnvelope(decoded, envelope);
            {
                auto unknown = canonical; unknown[4] = 2;
                codec = codec && !Decode(unknown, decoded);
                auto truncated = canonical; truncated.pop_back();
                codec = codec && !Decode(truncated, decoded);
                auto trailing = canonical; trailing.push_back(0);
                codec = codec && !Decode(trailing, decoded);
                auto corrupt = canonical;
                corrupt[corrupt.size() / 2] ^= 0x5a;
                codec = codec && !Decode(corrupt, decoded);
                std::vector<std::uint8_t> scratch;
                Envelope empty = envelope; empty.exactRecipe.clear();
                codec = codec && !Encode(empty, scratch);
                Envelope oversized = envelope;
                oversized.exactRecipe.assign(MaximumBaselineBytes, 7);
                codec = codec && !Encode(oversized, scratch);
                Envelope zeroed = envelope; zeroed.hostEntity = {};
                codec = codec && !Encode(zeroed, scratch);
                codec = codec && !Decode({}, decoded);
            }
            std::uint64_t bits = codec ? 0x01ULL : 0ULL;
            const auto children = ExpectedChildren(definition);
            std::string attributeFirst, solidFirst;
            if (children.empty()
                || !SaveMode(12, attributeFirst)
                || !SaveMode(13, solidFirst)) return bits;
            {
                const Handle(TDocStd_Application) production = ProductionApp();
                if (ReopenExact(production, attributeFirst, envelope, children)
                    && ReopenExact(production, solidFirst, envelope, children))
                    bits |= 0x02ULL;
            }
            {
                Handle(TDocStd_Application) legacy =
                    new TDocStd_Application();
                Core3DDebugDefineLegacyReceiptFormats(legacy);
                if (ReopenExact(legacy, attributeFirst, envelope, children)
                    && ReopenExact(legacy, solidFirst, envelope, children))
                    bits |= 0x04ULL;
            }
            std::string flipped = attributeFirst;
            const std::string magic("SYFB\1\0\0\0", 8);
            const std::size_t at = flipped.find(magic);
            if (at == std::string::npos
                || flipped.find(magic, at + 1) != std::string::npos)
                return bits;
            flipped[at + 8 + 16] ^= 0x5a;
            {
                const Handle(TDocStd_Application) production = ProductionApp();
                Handle(TDocStd_Application) legacy =
                    new TDocStd_Application();
                Core3DDebugDefineLegacyReceiptFormats(legacy);
                bool stickyProduction = false, stickyLegacy = false;
                if (RefusedRead(production, flipped, stickyProduction)
                    && stickyProduction
                    && RefusedRead(legacy, flipped, stickyLegacy)
                    && stickyLegacy) bits |= 0x08ULL;
            }
            {
                const Handle(TDocStd_Application) production = ProductionApp();
                Handle(TDocStd_Application) legacy = new TDocStd_Application();
                Core3DDebugDefineLegacyReceiptFormats(legacy);
                bool exact = true;
                for (const std::string* valid : {&attributeFirst, &solidFirst}) {
                    for (Standard_Integer delta : {1, -1}) {
                        std::string malformed;
                        if (!BaselineCountMismatch(*valid, canonical, delta, malformed)) {
                            exact = false; continue;
                        }
                        for (const auto& reader : {production, legacy}) {
                            bool sticky = false;
                            const bool refused = RefusedRead(reader, malformed, sticky);
                            const bool recovered = ReopenExact(
                                reader, *valid, envelope, children);
                            exact = exact && refused && sticky && recovered;
                        }
                    }
                }
                // Whole-file truncation belongs to the container reader.
                // It must fail, but need not visit a SYFB driver or set its
                // family-specific sticky flag. Keep that separate proof.
                bool containerSticky = false;
                const bool containerRefused = RefusedRead(production,
                    attributeFirst.substr(0, attributeFirst.size() - 1),
                    containerSticky);
                const bool containerRecovered = ReopenExact(
                    production, attributeFirst, envelope, children);
                if (exact && containerRefused && containerRecovered)
                    bits |= 0x10ULL;
            }
            {
                Budget budget; budget.limit = canonical.size();
                bool exact = Charge(budget, canonical.size())
                    && budget.bytes == canonical.size()
                    && budget.records == 1
                    && !Charge(budget, canonical.size()) && budget.rejected;
                budget.reset();
                exact = exact && !budget.rejected && budget.bytes == 0
                    && budget.records == 0
                    && Charge(budget, canonical.size());
                Budget oversized;
                exact = exact && !Charge(oversized, MaximumBaselineBytes + 1)
                    && !oversized.rejected;
                Budget emptyBudget;
                exact = exact && !Charge(emptyBudget, 0)
                    && !emptyBudget.rejected;
                if (exact) bits |= 0x20ULL;
            }
            {
                const Handle(TDocStd_Application) reader = ProductionApp();
                bool sticky = false;
                if (RefusedRead(reader, flipped, sticky) && sticky
                    && ReopenExact(reader, attributeFirst, envelope, children))
                    bits |= 0x40ULL;
            }
            return bits;
        }
    } catch (...) {}
    return 0;
}
} // namespace baseline_probe

extern "C" __attribute__((used, visibility("default"))) inline std::uint64_t
Core3DDebugFeaturePatternBaselineProbe(std::int32_t scenario) noexcept {
    return baseline_probe::Run(scenario);
}

namespace wire_probe {
inline bounded_curve::UUID ID(std::uint8_t value) noexcept {
    bounded_curve::UUID result{};
    result[0] = value;
    return result;
}

inline bounded_curve::PersistedValue Fixture() noexcept {
    using namespace bounded_curve;
    PersistedValue result;
    result.value.feature = ID(4);
    Definition& definition = result.value.definition;
    definition.domain = Domain::Path3D;
    definition.frame.identifier = ID(5);
    definition.frame.revision = 1;
    definition.frame.origin = {{10, -4, 2}};
    definition.degree = 2;
    definition.controlPoints = {
        {ID(10), {{0, 0, 0}}}, {ID(11), {{2, 3, 1}}},
        {ID(12), {{6, -1, 2}}}, {ID(13), {{9, 2, 4}}}
    };
    definition.knots = {{0, 3}, {0.5, 1}, {1, 3}};
    definition.weights = {1, 0.75, 1.25, 1};
    result.ownerState.owner.document = ID(1);
    result.ownerState.owner.entity = ID(2);
    result.ownerState.owner.definition = ID(3);
    result.ownerState.feature = result.value.feature;
    result.ownerState.definitionRevision = 1;
    result.ownerState.nextLocalID = 5;
    std::vector<std::uint8_t> bytes;
    Encode(result.value, bytes);
    Hash(bytes, MaximumDefinitionBytes,
         result.ownerState.canonicalDefinitionDigest);
    return result;
}

inline bool Rehash(bounded_curve::PersistedValue& value) noexcept {
    std::vector<std::uint8_t> bytes;
    return bounded_curve::Encode(value.value, bytes)
        && bounded_curve::Hash(bytes, bounded_curve::MaximumDefinitionBytes,
            value.ownerState.canonicalDefinitionDigest);
}

inline std::uint64_t Run(std::int32_t scenario) noexcept {
    using namespace bounded_curve;
    try {
        if (scenario == 0) {
            const PersistedValue value = Fixture();
            DetachedWire millimetres, metres;
            std::uint64_t bits = ValidatePersisted(value) ? 1ULL : 0ULL;
            if (BuildWire(value, millimetres) != BuildRefusal::None) return bits;
            bits |= 2ULL;
            if (!MatchesPersistedValue(millimetres, value)) return bits;
            bits |= 4ULL;
            // Unit conversion belongs at the UI boundary. Building the same
            // retained definition for mm and m documents must not rescale it.
            constexpr std::array<double, 2> metersPerUnit{{0.001, 1.0}};
            if (metersPerUnit[0] == metersPerUnit[1]
                || BuildWire(value, metres) != BuildRefusal::None
                || millimetres.geometryCommitment != metres.geometryCommitment)
                return bits;
            bits |= 8ULL;
            if (millimetres.canonicalSamples == metres.canonicalSamples
                && WireMatchesDefinition(value, millimetres.wire)
                && WireMatchesDefinition(value, metres.wire)) bits |= 16ULL;
            return bits;
        }
        if (scenario == 1) {
            std::uint64_t bits = 0;
            PersistedValue knots = Fixture();
            knots.value.definition.knots[1].value = 0;
            Rehash(knots);
            DetachedWire output;
            if (BuildWire(knots, output) != BuildRefusal::None) bits |= 1ULL;
            PersistedValue frame = Fixture();
            frame.value.definition.frame.yAxis = frame.value.definition.frame.xAxis;
            Rehash(frame);
            if (BuildWire(frame, output) != BuildRefusal::None) bits |= 2ULL;
            PersistedValue weights = Fixture();
            weights.value.definition.weights[1] = 0;
            Rehash(weights);
            if (BuildWire(weights, output) != BuildRefusal::None) bits |= 4ULL;
            PersistedValue count = Fixture();
            count.value.definition.weights.pop_back();
            Rehash(count);
            if (BuildWire(count, output) != BuildRefusal::None) bits |= 8ULL;
            PersistedValue flipped = Fixture();
            flipped.value.definition.frame.zAxis = {{0, 0, -1}};
            Rehash(flipped);
            if (BuildWire(flipped, output) != BuildRefusal::None) bits |= 16ULL;
            return bits;
        }
        if (scenario == 2) {
            PersistedValue before = Fixture();
            DetachedWire oldWire, rebuilt;
            std::uint64_t bits = BuildWire(before, oldWire) == BuildRefusal::None
                ? 1ULL : 0ULL;
            PersistedValue after = before;
            after.value.definition.controlPoints[1].local[1] += 2;
            ++after.ownerState.definitionRevision;
            if (!Rehash(after) || !ValidatePersisted(after)) return bits;
            bits |= 2ULL;
            if (!MatchesPersistedValue(oldWire, after)
                && !WireMatchesDefinition(after, oldWire.wire)) bits |= 4ULL;
            if (BuildWire(after, rebuilt) != BuildRefusal::None) return bits;
            bits |= 8ULL;
            if (MatchesPersistedValue(rebuilt, after)
                && rebuilt.definitionDigest != oldWire.definitionDigest
                && rebuilt.geometryCommitment != oldWire.geometryCommitment) bits |= 16ULL;
            return bits;
        }
    } catch (...) {}
    return 0;
}
} // namespace wire_probe

//! Header-defined because package 04 intentionally adds no implementation
//! unit. `used` leaves one weak ODR export for the XCTest bridge in DEBUG only.
extern "C" __attribute__((used, visibility("default"))) inline std::uint64_t
Core3DDebugBoundedCurveBuildProbe(std::int32_t scenario) noexcept {
    return wire_probe::Run(scenario);
}

} // namespace core3d::native_opening::debug

#endif
