#include "PatternRecipeClone.hxx"
#include "PatternBuild.hxx"
#include "PatternPersistence.hxx"
#include "FeaturePatternBuild.hxx"
#include "FeaturePatternPersistence.hxx"
#include "PathArrayBuild.hxx"
#include "PathArrayPersistence.hxx"
#include "DetachedPlanarSweepProbe.hxx"
#include "DetachedRectangularLoftProbe.hxx"
#include "PlanarSweepSolid.hxx"
#include "RectangularLoftSolid.hxx"

#include <BinXCAFDrivers.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <TDocStd_Application.hxx>
#include <TopExp_Explorer.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace clone = core3d::pattern_recipe_clone;
namespace sweep = core3d::planar_sweep;
namespace loft = core3d::rectangular_loft;
namespace pattern = core3d::pattern;
namespace feature_pattern = core3d::feature_pattern;
namespace profile = core3d::profile;
namespace retained_boolean = core3d::retained_boolean;
namespace analytic_boolean = core3d::analytic_boolean;
namespace analytic_boolean_ring = core3d::analytic_boolean_ring;
namespace path_array = core3d::path_array;
namespace curve = core3d::bounded_curve;

namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

clone::UUID uuid(std::uint8_t seed) {
    clone::UUID value{};
    for (std::size_t index = 0; index < value.size(); ++index)
        value[index] = std::uint8_t(seed + index);
    return value;
}

TopoDS_Shape build(const sweep::Definition& definition) {
    sweep::Admission admission; const auto prepared = sweep::Prepare(definition, admission);
    std::atomic_bool cancelled{false}; sweep::SolidResult result;
    require(prepared && sweep::Build(prepared, cancelled, result) == sweep::BuildStatus::Built,
            "sweep build");
    return result.solid;
}

TopoDS_Shape build(const loft::Definition& definition) {
    loft::Admission admission; const auto prepared = loft::Prepare(definition, admission);
    std::atomic_bool cancelled{false}; loft::SolidResult result;
    require(prepared && loft::Build(prepared, cancelled, result) == loft::BuildStatus::Built,
            "loft build");
    return result.solid;
}

Handle(TDocStd_Document) document(double unit) {
    static Handle(TDocStd_Application) application;
    if (application.IsNull()) {
        application = new TDocStd_Application;
        BinXCAFDrivers::DefineFormat(application);
    }
    Handle(TDocStd_Document) result;
    application->NewDocument(TCollection_ExtendedString("BinXCAF"), result);
    XCAFDoc_DocumentTool::SetLengthUnit(result, unit); result->SetUndoLimit(10);
    return result;
}

const char* sourceID = "75D8D07A-3F43-4DE3-9028-96DBBFC29D20";
const char* copyID = "AF547413-00E8-4B64-BBCA-6FC4B72C9875";

template<class Definition>
struct Fixture;

template<>
struct Fixture<sweep::Definition> {
    static sweep::Definition make(double unit) { return sweep::probe::Fixture(4, unit, 2); }
    static bool stage(const Handle(TDocStd_Document)& doc, const TDF_Label& owner,
                      const sweep::Definition& value, const char* id) {
        return core3d::sweep_persistence::Stage(doc, owner, value, id);
    }
};

template<>
struct Fixture<loft::Definition> {
    static loft::Definition make(double unit) { return loft::probe::Fixture(3, unit, 1); }
    static bool stage(const Handle(TDocStd_Document)& doc, const TDF_Label& owner,
                      const loft::Definition& value, const char* id) {
        return core3d::loft_persistence::Stage(doc, owner, value, id);
    }
};

template<class Definition>
void testIndependentRecipeClonePreservesLocalIDsAndIssuesFeatureIdentity() {
    for (double unit : {0.001, 1.0}) {
        const auto doc = document(unit); const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        const Definition definition = Fixture<Definition>::make(unit);
        const TopoDS_Shape sourceShape = build(definition);
        const TDF_Label source = shapeTool->AddShape(sourceShape, Standard_False);
        doc->NewCommand(); require(Fixture<Definition>::stage(doc, source, definition, sourceID), "source stage");
        require(doc->CommitCommand(), "source commit");
        clone::Source captured; require(clone::Capture(doc, source, captured), "source capture");
        BRepBuilderAPI_Copy copier(sourceShape, Standard_True, Standard_False);
        require(copier.IsDone() && !copier.Shape().IsPartner(sourceShape), "deep geometry copy");
        doc->NewCommand(); const TDF_Label destination = shapeTool->AddShape(copier.Shape(), Standard_False);
        clone::Prepared prepared;
        require(clone::Prepare(captured, copier.Shape(), copyID, std::nullopt, prepared), "prepare clone");
        clone::Candidate candidate;
        require(clone::Stage(doc, captured, destination, "source-entity", "source-definition",
                "copy-entity", "copy-definition", prepared, candidate), "stage clone");
        require(candidate.family == captured.family, "family retained");
        if constexpr (std::is_same_v<Definition, sweep::Definition>) {
            require(candidate.sweep.identifier != captured.sweep.identifier, "new sweep feature id");
            require(clone::SameLocalIDs(captured.sweep.definition, candidate.sweep.definition),
                    "stable sweep local ids");
        } else {
            require(candidate.loft.identifier != captured.loft.identifier, "new loft feature id");
            require(clone::SameLocalIDs(captured.loft.definition, candidate.loft.definition),
                    "stable loft local ids");
        }
        clone::Source sourceAfter; require(clone::Capture(doc, source, sourceAfter)
            && clone::IsEqual(captured, sourceAfter), "source unchanged by staging");
        require(doc->CommitCommand(), "clone commit");
        require(doc->Undo(), "clone undo"); require(XCAFDoc_ShapeTool::GetShape(destination).IsNull(), "undo atomic");
        require(doc->Redo(), "clone redo"); require(clone::ReadCandidate(doc, destination, candidate), "redo exact");
    }
}

template<class Definition>
void testMirrorCloneComposesFrameAndKeepsValidForwardSolid() {
    const double unit = 0.001; const auto doc = document(unit);
    const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    const Definition definition = Fixture<Definition>::make(unit);
    const TopoDS_Shape sourceShape = build(definition);
    const TDF_Label source = shapeTool->AddShape(sourceShape, Standard_False);
    doc->NewCommand(); require(Fixture<Definition>::stage(doc, source, definition, sourceID), "mirror source stage");
    require(doc->CommitCommand(), "mirror source commit");
    clone::Source captured; require(clone::Capture(doc, source, captured), "mirror capture");
    gp_Trsf mirror; mirror.SetMirror(gp_Ax2(gp_Pnt(17, 0, 0), gp_Dir(1, 0, 0), gp_Dir(0, 1, 0)));
    BRepBuilderAPI_Transform transformed(sourceShape, mirror, Standard_True, Standard_False);
    require(transformed.IsDone() && !transformed.Shape().IsPartner(sourceShape), "mirror deep copy");
    clone::Prepared prepared;
    require(clone::Prepare(captured, transformed.Shape(), copyID, mirror, prepared), "mirror prepare");
    const Definition& mirrored = [&]() -> const Definition& {
        if constexpr (std::is_same_v<Definition, sweep::Definition>) return prepared.sweep;
        else return prepared.loft;
    }();
    require(mirrored.constructionFrame && mirrored.constructionFrame->values[7] < 0,
            "reflection stored as signed frame");
    const TopoDS_Shape rebuilt = build(mirrored);
    require(BRepCheck_Analyzer(rebuilt, Standard_True).IsValid(), "mirrored recipe rebuild valid");
    for (TopExp_Explorer shell(rebuilt, TopAbs_SHELL); shell.More(); shell.Next())
        require(BRep_Tool::IsClosed(shell.Current()), "mirrored winding closed");
    BRepClass3d_SolidClassifier classifier(rebuilt); classifier.PerformInfinitePoint(Precision::Confusion());
    GProp_GProps properties; BRepGProp::VolumeProperties(rebuilt, properties);
    require(classifier.State() == TopAbs_OUT && properties.Mass() > 0, "mirrored forward volume");
}

void testStaleSweepAndLoftOwnersRefuseInsteadOfBecomingCurrentCopies() {
    for (bool useSweep : {false, true}) {
        const auto doc = document(0.001); const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        const TopoDS_Shape shape = useSweep ? build(Fixture<sweep::Definition>::make(0.001))
                                            : build(Fixture<loft::Definition>::make(0.001));
        const TDF_Label owner = shapeTool->AddShape(shape, Standard_False);
        doc->NewCommand();
        require(useSweep ? Fixture<sweep::Definition>::stage(doc, owner,
                    Fixture<sweep::Definition>::make(0.001), sourceID)
                : Fixture<loft::Definition>::stage(doc, owner,
                    Fixture<loft::Definition>::make(0.001), sourceID), "stale fixture stage");
        require(doc->CommitCommand(), "stale fixture commit");
        BRepBuilderAPI_Copy replacement(shape, Standard_True, Standard_False);
        doc->NewCommand(); shapeTool->SetShape(owner, replacement.Shape()); require(doc->CommitCommand(), "stale shape commit");
        clone::Source refused; require(!clone::Capture(doc, owner, refused), "stale recipe refusal");
    }
}

void testColdReopenPreservesIndependentRecipeBytesInBothUnitSystems() {
    for (double unit : {0.001, 1.0}) {
        const auto doc = document(unit); const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        const auto definition = Fixture<sweep::Definition>::make(unit);
        const TopoDS_Shape shape = build(definition); const TDF_Label source = shapeTool->AddShape(shape, Standard_False);
        doc->NewCommand(); require(Fixture<sweep::Definition>::stage(doc, source, definition, sourceID), "reopen source");
        require(doc->CommitCommand(), "reopen source commit"); clone::Source captured;
        require(clone::Capture(doc, source, captured), "reopen capture"); BRepBuilderAPI_Copy copy(shape, Standard_True, Standard_False);
        doc->NewCommand(); const TDF_Label destination = shapeTool->AddShape(copy.Shape(), Standard_False);
        clone::Prepared prepared; require(clone::Prepare(captured, copy.Shape(), copyID, std::nullopt, prepared), "reopen prepare");
        clone::Candidate candidate; require(clone::Stage(doc, captured, destination, "se", "sd", "de", "dd",
            prepared, candidate), "reopen stage"); require(doc->CommitCommand(), "reopen clone commit");
        std::ostringstream bytes(std::ios::binary);
        const auto app = Handle(TDocStd_Application)::DownCast(doc->Application());
        require(!app.IsNull() && app->SaveAs(doc, bytes) == PCDM_SS_OK, "save clone");
        std::istringstream input(bytes.str(), std::ios::binary); Handle(TDocStd_Document) reopened;
        require(app->Open(input, reopened) == PCDM_RS_OK, "cold reopen"); TDF_LabelSequence roots;
        XCAFDoc_DocumentTool::ShapeTool(reopened->Main())->GetFreeShapes(roots); require(roots.Length() == 2, "two reopened owners");
        bool found = false; for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            core3d::sweep_persistence::Record record;
            require(core3d::sweep_persistence::Read(reopened, roots.Value(index), record), "reopened read");
            if (!record.label.IsNull() && record.identifier == copyID) {
                found = core3d::sweep_persistence::SameBits(record.values, candidate.sweep.values);
            }
        }
        require(found, "cold reopen exact independent copy"); app->Close(reopened);
    }
}

pattern::UUID patternUUID(std::uint32_t value) {
    pattern::UUID result{};
    result[0] = std::uint8_t(value); result[1] = std::uint8_t(value >> 8);
    result[2] = std::uint8_t(value >> 16); result[3] = std::uint8_t(value >> 24);
    result[15] = 0xA5; return result;
}

curve::PersistedValue pathValue(const std::vector<curve::Vector3>& points,
                                std::uint8_t degree = 1,
                                std::uint64_t revision = 1) {
    curve::PersistedValue result;
    result.value.feature = patternUUID(2000);
    auto& definition = result.value.definition;
    definition.domain = curve::Domain::Path3D; definition.degree = degree;
    definition.frame.identifier = patternUUID(2001);
    definition.frame.revision = 1;
    for (std::size_t index = 0; index < points.size(); ++index)
        definition.controlPoints.push_back({patternUUID(2100 + std::uint32_t(index)), points[index]});
    if (degree == 1) {
        for (std::size_t index = 0; index < points.size(); ++index)
            definition.knots.push_back({double(index), std::uint8_t(
                index == 0 || index + 1 == points.size() ? 2 : 1)});
    } else {
        require(points.size() == 3 && degree == 2, "quadratic fixture shape");
        definition.knots = {{0, 3}, {1, 3}};
    }
    result.ownerState.owner.document = patternUUID(2200);
    result.ownerState.owner.entity = patternUUID(2201);
    result.ownerState.owner.definition = patternUUID(2202);
    result.ownerState.feature = result.value.feature;
    result.ownerState.definitionRevision = revision;
    result.ownerState.nextLocalID = points.size() + 1;
    std::vector<std::uint8_t> bytes;
    require(curve::Encode(result.value, bytes), "curve fixture encode");
    require(curve::Hash(bytes, curve::MaximumDefinitionBytes,
                        result.ownerState.canonicalDefinitionDigest), "curve fixture digest");
    require(curve::ValidatePersisted(result), "curve fixture persisted");
    return result;
}

feature_pattern::Definition featurePatternFixture(pattern::Kind kind,
                                                  std::uint32_t rows,
                                                  std::uint32_t columns,
                                                  std::uint32_t& seed,
                                                  double unit = .001) {
    feature_pattern::Definition value;
    value.host = {patternUUID(1101), patternUUID(1102), patternUUID(1103)};
    value.feature = patternUUID(1104);
    value.sourceCut = {value.host.document, patternUUID(1105),
                       patternUUID(1106), patternUUID(1107)};
    value.sourceCutStepID = 41; value.metersPerUnit = unit;
    value.distribution.kind = kind; value.distribution.owner = value.host;
    value.distribution.feature = value.feature;
    value.distribution.source = value.sourceCut;
    value.distribution.rowCount = rows; value.distribution.columnCount = columns;
    value.distribution.rowSpacing = 8; value.distribution.columnSpacing = 12;
    value.distribution.sweepRadians = pattern::TwoPi;
    value.distribution.issuance.nextLocalID = 1;
    if (kind == pattern::Kind::Radial)
        value.distribution.columnAxis = pattern::Axis::Z;
    const pattern::IssueUUID issue = [&](pattern::UUID& identifier) {
        identifier = patternUUID(++seed); return true;
    };
    require(pattern::Reconcile(value.distribution, rows, columns, issue),
            "feature pattern distribution reconcile");
    require(feature_pattern::Valid(value), "feature pattern fixture valid");
    return value;
}

feature_pattern::AdmissionInput admittedFeatureInput(
    const feature_pattern::Definition& value, double volumePerFeature = 7.5) {
    feature_pattern::AdmissionInput input;
    input.hostSurface = feature_pattern::HostSurface::Planar;
    input.hostRecipeAvailable = true; input.sourceCutFeatureAvailable = true;
    input.budget.hostTopologyNodes = 100; input.budget.sourceToolTopologyNodes = 20;
    input.budget.sourceRecipeBytes = 512;
    std::vector<pattern::Placement> placements;
    require(pattern::BuildPlacements(value.distribution, placements),
            "feature placements");
    for (const pattern::Placement& placement : placements) {
        const auto member = std::find_if(value.distribution.members.begin(),
            value.distribution.members.end(), [&](const pattern::Member& item) {
                return item.localID == placement.localID;
            });
        require(member != value.distribution.members.end(), "feature member");
        feature_pattern::GeneratedMeasurement measured;
        measured.childFeature = feature_pattern::ChildFeatureID(value, *member);
        measured.instanceIdentity = placement.identity; measured.localID = placement.localID;
        measured.coordinate = placement.coordinate; measured.worldFrame = placement.worldFrame;
        measured.intersectsHost = true; measured.minimumHostLigamentMM = 2;
        measured.removedVolume = volumePerFeature;
        measured.boundarySections = value.expectedBoundarySectionsPerFeature;
        input.generated.push_back(measured);
        input.result.presentChildFeatures.push_back(measured.childFeature);
    }
    input.result.hostVolume = 1000;
    input.result.resultVolume = 1000 - volumePerFeature * placements.size();
    input.result.generatedBoundarySections = std::uint32_t(placements.size())
        * value.expectedBoundarySectionsPerFeature;
    return input;
}

std::vector<std::uint8_t> legacyRingProgramBytes() {
    profile::Parameters source; source.metersPerUnit = .001;
    source.definition.plane = 0; source.definition.depth = 20;
    source.definition.points = {{0,0},{20,0},{20,20},{0,20}};
    std::vector<double> values;
    require(profile::Encode(source, values), "legacy ring source encode");
    retained_boolean::Program program; program.codecMinor = 2;
    program.source = {patternUUID(1201), patternUUID(1202), patternUUID(1203),
        patternUUID(1204), patternUUID(1205), 1,
        std::uint32_t(profile::SchemaFor(source)), .001, values};
    retained_boolean::Step bore; bore.operand.identifier = 1;
    bore.operand.axis = analytic_boolean::Axis::Z; bore.operand.point = {{10,10,0}};
    bore.operand.radius = 1;
    retained_boolean::Step ring; ring.operand.identifier = 2;
    ring.operand.kind = analytic_boolean::OperandKind::CylinderRing;
    ring.operand.axis = analytic_boolean::Axis::Z; ring.operand.point = {{10,10,0}};
    ring.operand.radius = .5; ring.operand.boltCircleRadius = 5;
    ring.operand.hostRadiusRatio = .25; ring.operand.count = 4;
    program.steps = {bore, ring}; program.nextOperandID = 3;
    std::vector<std::uint8_t> bytes;
    require(retained_boolean::Encode(program, bytes), "legacy ring program encode");
    return bytes;
}

void testCutFeaturePatternRetainsHostSourceDistributionAndLegacyRingBytes() {
    const auto before = legacyRingProgramBytes();
    require(retained_boolean::MaximumOperands == 4
            && analytic_boolean_ring::kMaximumExpandedDisks == 32,
            "legacy program limits unchanged");
    std::uint32_t seed = 1300;
    auto value = featurePatternFixture(pattern::Kind::Grid, 2, 3, seed);
    std::vector<std::uint8_t> bytes; require(feature_pattern::Encode(value, bytes),
        "feature pattern encode");
    feature_pattern::Definition decoded;
    require(feature_pattern::Decode(bytes, decoded)
            && decoded.host == value.host && decoded.sourceCut == value.sourceCut
            && decoded.sourceCutStepID == value.sourceCutStepID,
            "host source cut and distribution retained");
    require(legacyRingProgramBytes() == before,
            "feature pattern cannot perturb legacy ring bytes");
}

void testCutFeaturePatternPlacementVolumeBoundaryAndAttributionAreIndependent() {
    std::uint32_t seed = 1400;
    auto value = featurePatternFixture(pattern::Kind::Radial, 1, 6, seed);
    value.distribution.radialPivotLocal = {{-20, 0, 0}};
    const double radius = 1.5, depth = 4;
    const double independentVolume = std::acos(-1.0) * radius * radius * depth;
    auto input = admittedFeatureInput(value, independentVolume);
    const auto admitted = feature_pattern::Admit(value, input);
    require(admitted.admitted() && admitted.attribution.size() == 6,
            "complete cut feature pattern admitted");
    require(std::abs(admitted.measuredRemovedVolume - 6 * independentVolume)
            <= independentVolume * 1e-10, "independent aggregate removed volume");
    require(admitted.measuredBoundarySections == 12,
            "independent generated boundary sections");
    for (const auto& item : admitted.attribution)
        require(item.sourceCutFeature == value.sourceCut.sourceFeature
                && item.sourceCutStepID == value.sourceCutStepID,
                "every child attributed to source and instance");
}

void testCutFeaturePatternChildIDsSurviveCountSourceAndHostEdits() {
    std::uint32_t seed = 1500;
    auto value = featurePatternFixture(pattern::Kind::Grid, 3, 3, seed);
    const auto original = value.distribution.members;
    value.host.definition = patternUUID(1510);
    value.distribution.owner = value.host;
    value.sourceCut.definition = patternUUID(1511);
    value.distribution.source = value.sourceCut;
    require(feature_pattern::Valid(value), "host and source recipe edit valid");
    for (const auto& member : value.distribution.members) {
        const auto prior = std::find_if(original.begin(), original.end(),
            [&](const pattern::Member& item) { return item.coordinate == member.coordinate; });
        require(prior != original.end() && prior->identity == member.identity
                && prior->localID == member.localID, "host source edit preserves child ids");
    }
    const pattern::IssueUUID issue = [&](pattern::UUID& identifier) {
        identifier = patternUUID(++seed); return true;
    };
    require(feature_pattern::ReconcileCounts(value, 2, 2, issue), "count down");
    const auto survivors = value.distribution.members;
    const auto removals = value.distribution.removals;
    require(feature_pattern::ReconcileCounts(value, 3, 3, issue), "count up");
    for (const auto& survivor : survivors) {
        const auto current = std::find_if(value.distribution.members.begin(),
            value.distribution.members.end(), [&](const pattern::Member& item) {
                return item.coordinate == survivor.coordinate;
            });
        require(current != value.distribution.members.end()
                && current->identity == survivor.identity
                && current->localID == survivor.localID, "surviving coordinate stable");
    }
    for (const auto& removed : removals) for (const auto& current : value.distribution.members)
        require(removed.identity != current.identity && removed.localID != current.localID,
                "retired child feature id never reused");
}

void testCutFeaturePatternTypedRefusalsIncludeMissingOneFeature() {
    std::uint32_t seed = 1600;
    auto value = featurePatternFixture(pattern::Kind::Linear, 1, 4, seed);
    auto input = admittedFeatureInput(value);
    input.generated.pop_back(); input.result.presentChildFeatures.pop_back();
    require(feature_pattern::Admit(value, input).refusal
            == feature_pattern::Refusal::MissingGeneratedFeature,
            "missing one feature refuses");
    input = admittedFeatureInput(value); input.hostSurface = feature_pattern::HostSurface::Curved;
    require(feature_pattern::Admit(value, input).refusal == feature_pattern::Refusal::CurvedHost,
            "curved host typed refusal");
    input = admittedFeatureInput(value); input.generated[1].overlapsGeneratedTool = true;
    require(feature_pattern::Admit(value, input).refusal
            == feature_pattern::Refusal::GeneratedToolsOverlap, "overlap refusal");
    input = admittedFeatureInput(value); input.generated[1].minimumHostLigamentMM = .001;
    require(feature_pattern::Admit(value, input).refusal
            == feature_pattern::Refusal::InsufficientHostLigament, "ligament refusal");
    input = admittedFeatureInput(value); input.budget.maximumTopologyNodes = 100;
    require(feature_pattern::Admit(value, input).refusal
            == feature_pattern::Refusal::ExpansionBudget, "bounded expansion refusal");
}

void testCutFeaturePatternReplayParityAndTamperRefusal() {
    std::uint32_t seed = 1700;
    auto value = featurePatternFixture(pattern::Kind::Grid, 2, 3, seed);
    const auto first = feature_pattern::Admit(value, admittedFeatureInput(value));
    std::vector<std::uint8_t> bytes, exact;
    require(first.admitted() && feature_pattern::Encode(value, bytes), "first replay");
    feature_pattern::Definition decoded;
    require(feature_pattern::Decode(bytes, decoded)
            && feature_pattern::Encode(decoded, exact) && exact == bytes, "exact recipe replay");
    const auto replay = feature_pattern::Admit(decoded, admittedFeatureInput(decoded));
    require(replay.admitted() && replay.measuredRemovedVolume == first.measuredRemovedVolume
            && replay.attribution.size() == first.attribution.size(), "measurement replay parity");
    bytes[bytes.size() / 2] ^= 1;
    require(!feature_pattern::Decode(bytes, decoded), "tampered feature pattern refused");
}

void testCutFeaturePatternEditUndoRedoSaveColdReopenLaterEditBothUnits() {
    for (double unit : {.001, 1.0}) {
        std::uint32_t seed = unit < 1 ? 1800 : 1900;
        auto value = featurePatternFixture(pattern::Kind::Grid, 2, 2, seed, unit);
        const auto stable = value.distribution.members[1];
        const auto doc = document(unit); feature_pattern::Record record;
        doc->NewCommand(); require(feature_pattern::Stage(doc, value, record), "feature stage");
        require(doc->CommitCommand(), "feature commit");
        const pattern::IssueUUID issue = [&](pattern::UUID& identifier) {
            identifier = patternUUID(++seed); return true;
        };
        require(feature_pattern::ReconcileCounts(value, 2, 3, issue), "feature count edit");
        doc->NewCommand(); require(feature_pattern::Stage(doc, value, record), "feature edit stage");
        require(doc->CommitCommand(), "feature edit commit");
        require(doc->Undo() && feature_pattern::ReadFeature(doc, value.feature, record)
                && record.definition.distribution.columnCount == 2, "feature undo");
        require(doc->Redo() && feature_pattern::ReadFeature(doc, value.feature, record)
                && record.definition.distribution.columnCount == 3, "feature redo");
        std::ostringstream bytes(std::ios::binary);
        const auto app = Handle(TDocStd_Application)::DownCast(doc->Application());
        require(!app.IsNull() && app->SaveAs(doc, bytes) == PCDM_SS_OK, "feature save");
        std::istringstream input(bytes.str(), std::ios::binary);
        Handle(TDocStd_Document) reopened;
        require(app->Open(input, reopened) == PCDM_RS_OK, "feature cold reopen");
        require(feature_pattern::ReadFeature(reopened, value.feature, record), "feature reopen read");
        const auto survivor = std::find_if(record.definition.distribution.members.begin(),
            record.definition.distribution.members.end(), [&](const pattern::Member& item) {
                return item.coordinate == stable.coordinate;
            });
        require(survivor != record.definition.distribution.members.end()
                && survivor->identity == stable.identity && survivor->localID == stable.localID,
                "feature reopen identity");
        auto later = record.definition; later.minimumHostLigamentMM = .25;
        reopened->SetUndoLimit(10); reopened->NewCommand();
        require(feature_pattern::Stage(reopened, later, record), "feature later edit");
        require(reopened->CommitCommand() && feature_pattern::ReadFeature(reopened, later.feature, record)
                && record.definition.minimumHostLigamentMM == .25, "feature later readback");
        app->Close(reopened);
    }
}
path_array::Definition pathArrayFixture(const curve::PersistedValue& path,
        std::uint32_t count, std::uint32_t& seed, bool closed = false) {
    path_array::Definition value;
    value.owner.document = path.ownerState.owner.document;
    value.owner.entity = patternUUID(2300); value.owner.definition = patternUUID(2301);
    value.feature = patternUUID(2302);
    value.source.document = value.owner.document;
    value.source.entity = patternUUID(2303);
    value.source.definition = patternUUID(2304);
    value.source.sourceFeature = patternUUID(2305);
    value.path.owner = path.ownerState.owner; value.path.feature = path.value.feature;
    value.path.definitionRevision = path.ownerState.definitionRevision;
    value.path.canonicalDefinitionDigest = path.ownerState.canonicalDefinitionDigest;
    value.distribution.count = count; value.closedPath = closed;
    value.arcLengthTolerance = 1e-6; value.minimumTangent = 1e-12;
    value.orientation.maximumFrameStepRadians = 2.5;
    value.issuance.nextLocalID = 1;
    const pattern::IssueUUID issue = [&](pattern::UUID& output) {
        output = patternUUID(++seed); return true;
    };
    require(path_array::ReconcileMembers(value, count, issue), "path members reconcile");
    return value;
}

double independentlyMeasuredArcLength(const curve::Definition& definition,
                                      double parameter) {
    double first = 0, last = 0;
    require(curve::ParameterDomain(definition, first, last), "independent domain");
    if (parameter == first) return 0;
    constexpr unsigned Steps = 20000;
    curve::Evaluation prior;
    require(curve::Evaluate(definition, first, prior) == curve::EvaluationRefusal::None,
            "independent first");
    double length = 0;
    for (unsigned step = 1; step <= Steps; ++step) {
        const double value = first + (parameter - first) * double(step) / Steps;
        curve::Evaluation next;
        require(curve::Evaluate(definition, value, next) == curve::EvaluationRefusal::None,
                "independent sample");
        length += curve::Norm(curve::Subtract(next.point, prior.point)); prior = next;
    }
    return length;
}

pattern::Definition patternFixture(pattern::Kind kind, std::uint32_t rows,
                                   std::uint32_t columns, std::uint32_t& seed) {
    pattern::Definition value; value.kind = kind; value.rowCount = rows; value.columnCount = columns;
    if (kind == pattern::Kind::Radial) value.columnAxis = pattern::Axis::Z;
    value.owner.document = patternUUID(1); value.owner.entity = patternUUID(2);
    value.owner.definition = patternUUID(3); value.feature = patternUUID(4);
    value.source.document = value.owner.document; value.source.entity = patternUUID(5);
    value.source.definition = patternUUID(6); value.source.sourceFeature = patternUUID(7);
    value.rowSpacing = 2; value.columnSpacing = 3; value.sweepRadians = pattern::TwoPi;
    value.issuance.nextLocalID = 1;
    const pattern::IssueUUID issue = [&](pattern::UUID& output) { output = patternUUID(++seed); return true; };
    require(pattern::Reconcile(value, rows, columns, issue), "pattern fixture reconcile");
    return value;
}

const pattern::Placement& at(const std::vector<pattern::Placement>& values,
                             std::uint32_t row, std::uint32_t column) {
    const auto found = std::find_if(values.begin(), values.end(), [&](const pattern::Placement& item) {
        return item.coordinate == pattern::Coordinate{row, column};
    });
    require(found != values.end(), "placement coordinate"); return *found;
}

void testExactLinearRadialAndGridTransformLattices() {
    std::uint32_t seed = 100;
    auto linear = patternFixture(pattern::Kind::Linear, 1, 4, seed);
    std::vector<pattern::Placement> placements; require(pattern::BuildPlacements(linear, placements), "linear lattice");
    for (std::uint32_t column = 0; column < 4; ++column)
        require(at(placements, 0, column).worldFrame[3] == 3.0 * column, "linear exact x");

    auto grid = patternFixture(pattern::Kind::Grid, 3, 2, seed);
    require(pattern::BuildPlacements(grid, placements), "grid lattice");
    for (std::uint32_t row = 0; row < 3; ++row) for (std::uint32_t column = 0; column < 2; ++column) {
        const auto& item = at(placements, row, column);
        require(item.worldFrame[3] == 3.0 * column && item.worldFrame[7] == 2.0 * row,
                "grid exact xy");
    }

    auto radial = patternFixture(pattern::Kind::Radial, 1, 4, seed);
    radial.radialPivotLocal = {{-2, 0, 0}};
    require(pattern::BuildPlacements(radial, placements), "radial lattice");
    require(std::abs(at(placements, 0, 1).worldFrame[3] + 2) < 1e-12
            && std::abs(at(placements, 0, 1).worldFrame[7] - 2) < 1e-12, "radial quarter turn");
    require(std::abs(at(placements, 0, 3).worldFrame[3] + 2) < 1e-12
            && std::abs(at(placements, 0, 3).worldFrame[7] + 2) < 1e-12, "radial no seam duplicate");
}

void testCountAndProjectedDocumentMemoryBudgetsRefuseBeforeMutation() {
    std::uint32_t seed = 200; auto value = patternFixture(pattern::Kind::Grid, 4, 4, seed);
    pattern::AdmissionBudget budget; budget.sourceDocumentBytes = 1024; budget.sourceMemoryBytes = 4096;
    budget.existingDocumentBytes = 4096; budget.documentLimitBytes = 64 * 1024;
    budget.existingMemoryBytes = 8192; budget.memoryLimitBytes = 128 * 1024;
    const auto admitted = pattern::Project(value, budget); require(admitted.admitted, "bounded grid admitted");
    budget.memoryLimitBytes = admitted.projectedMemoryBytes - 1;
    const auto refused = pattern::Project(value, budget);
    require(!refused.admitted && refused.projectedMemoryBytes == admitted.projectedMemoryBytes,
            "projected memory refusal reports projection");
    const auto before = value; const pattern::IssueUUID issue = [&](pattern::UUID& id) { id = patternUUID(++seed); return true; };
    require(!pattern::Reconcile(value, pattern::MaximumInstances, 2, issue), "product budget refusal");
    require(value.members.size() == before.members.size(), "refusal no mutation");
}

void testZeroStepAndZeroAngleNeverCreateDuplicateMembers() {
    std::uint32_t seed = 300;
    auto linear = patternFixture(pattern::Kind::Linear, 1, 2, seed); linear.columnSpacing = 0;
    require(!pattern::ParametersValid(linear), "zero linear step refused");
    auto grid = patternFixture(pattern::Kind::Grid, 2, 2, seed); grid.rowSpacing = 0;
    require(!pattern::ParametersValid(grid), "zero repeated grid row refused");
    grid.rowCount = 1; require(pattern::ParametersValid(grid), "zero unused grid row accepted");
    auto radial = patternFixture(pattern::Kind::Radial, 1, 2, seed); radial.sweepRadians = 0;
    require(!pattern::ParametersValid(radial), "zero radial angle refused");
}

void testNegativeSpacingAndRotatedSourceFrameRemainExact() {
    std::uint32_t seed = 400; auto value = patternFixture(pattern::Kind::Grid, 2, 3, seed);
    value.columnSpacing = -4; value.rowSpacing = -2;
    value.sourceFrame = {{0,-1,0,10, 1,0,0,20, 0,0,1,30, 0,0,0,1}};
    std::vector<pattern::Placement> placements; require(pattern::BuildPlacements(value, placements), "rotated negative grid");
    const auto& item = at(placements, 1, 2);
    require(item.worldFrame[3] == 12 && item.worldFrame[7] == 12 && item.worldFrame[11] == 30,
            "negative local lattice transformed by rotated source frame");
}

void testCountDownThenUpPreservesSurvivorsAndNeverReusesRemovedIDs() {
    std::uint32_t seed = 500; auto value = patternFixture(pattern::Kind::Grid, 3, 3, seed);
    const auto original = value.members; const pattern::IssueUUID issue = [&](pattern::UUID& id) { id = patternUUID(++seed); return true; };
    require(pattern::Reconcile(value, 2, 2, issue), "count down");
    require(value.removals.size() == 5 && value.issuance.retiredLocalIDs.size() == 5, "five removals recorded");
    for (const auto& member : value.members) {
        const auto old = std::find_if(original.begin(), original.end(), [&](const pattern::Member& item) {
            return item.coordinate == member.coordinate;
        });
        require(old != original.end() && old->identity == member.identity && old->localID == member.localID,
                "survivor identity stable");
    }
    const auto removed = value.removals; require(pattern::Reconcile(value, 3, 3, issue), "count up");
    for (const auto& member : value.members) for (const auto& retired : removed)
        require(member.identity != retired.identity && member.localID != retired.localID, "removed id never reused");
}

void testSuppressionPreservesIdentityWithoutRemoval() {
    std::uint32_t seed = 600; auto value = patternFixture(pattern::Kind::Grid, 2, 2, seed);
    const auto original = at([&] { std::vector<pattern::Placement> p; require(pattern::BuildPlacements(value,p),"initial placements"); return p; }(), 1, 1);
    require(pattern::SetSuppressed(value, {1,1}, true), "suppress");
    std::vector<pattern::Placement> placements; require(pattern::BuildPlacements(value, placements), "suppressed build");
    require(placements.size() == 3 && value.removals.empty(), "suppression omits without removal");
    require(pattern::SetSuppressed(value, {1,1}, false) && pattern::BuildPlacements(value, placements), "unsuppress");
    const auto& restored = at(placements, 1, 1);
    require(restored.identity == original.identity && restored.localID == original.localID, "suppressed identity stable");
}

void testPatternPersistenceRoundTripRejectsTamperAndKeepsSourceAuthority() {
    std::uint32_t seed = 700; auto value = patternFixture(pattern::Kind::Grid, 2, 3, seed);
    const auto source = value.source; std::vector<std::uint8_t> bytes;
    require(pattern::Encode(value, bytes), "pattern encode"); pattern::Definition decoded;
    require(pattern::Decode(bytes, decoded) && decoded.source == source, "associative source identity roundtrip");
    require(decoded.members.size() == value.members.size() && decoded.feature == value.feature,
            "feature and member identities roundtrip");
    bytes[bytes.size() / 2] ^= 1; require(!pattern::Decode(bytes, decoded), "tampered pattern refused");
}

void testSourceRecipeChangeReusesPatternMemberIdentities() {
    std::uint32_t seed = 800; auto value = patternFixture(pattern::Kind::Linear, 1, 3, seed);
    std::vector<pattern::Placement> before, after;
    require(pattern::BuildPlacements(value, before), "source edit before");
    // Geometry/recipe bytes are intentionally absent from the pattern record:
    // rebuild resolves value.source and applies these same member identities.
    require(pattern::BuildPlacements(value, after), "source edit after");
    require(before.size() == after.size(), "source edit member count");
    for (std::size_t index = 0; index < before.size(); ++index)
        require(before[index].identity == after[index].identity
                && before[index].localID == after[index].localID, "source edit stable instance identity");
}

void testPatternEditUndoRedoSaveColdReopenAndLaterEditInBothUnitSystems() {
    for (double unit : {0.001, 1.0}) {
        std::uint32_t seed = unit < 1 ? 900 : 1000;
        auto value = patternFixture(pattern::Kind::Grid, 2, 2, seed);
        const auto doc = document(unit); pattern::Record staged;
        doc->NewCommand(); require(pattern::Stage(doc, value, staged), "initial pattern stage");
        require(doc->CommitCommand(), "initial pattern commit");
        const auto survivor = value.members[1];
        const pattern::IssueUUID issue = [&](pattern::UUID& id) { id = patternUUID(++seed); return true; };
        require(pattern::Reconcile(value, 2, 3, issue), "pattern count edit");
        doc->NewCommand(); require(pattern::Stage(doc, value, staged), "edited pattern stage");
        require(doc->CommitCommand(), "edited pattern commit");
        require(doc->Undo(), "pattern undo"); pattern::Record readback;
        require(pattern::ReadFeature(doc, value.feature, readback)
                && readback.definition.columnCount == 2, "undo exact recipe");
        require(doc->Redo() && pattern::ReadFeature(doc, value.feature, readback)
                && readback.definition.columnCount == 3, "redo exact recipe");
        std::ostringstream bytes(std::ios::binary); const auto app = Handle(TDocStd_Application)::DownCast(doc->Application());
        require(!app.IsNull() && app->SaveAs(doc, bytes) == PCDM_SS_OK, "save pattern");
        std::istringstream input(bytes.str(), std::ios::binary); Handle(TDocStd_Document) reopened;
        require(app->Open(input, reopened) == PCDM_RS_OK, "cold reopen pattern");
        require(pattern::ReadFeature(reopened, value.feature, readback)
                && readback.definition.members[1].identity == survivor.identity, "reopen stable survivor");
        auto later = readback.definition; later.columnSpacing = -7;
        require(pattern::Valid(later), "later definition valid");
        reopened->SetUndoLimit(10); reopened->NewCommand();
        require(pattern::Stage(reopened, later, readback), "later pattern stage");
        require(reopened->CommitCommand() && pattern::ReadFeature(reopened, later.feature, readback)
                && readback.definition.columnSpacing == -7, "later edit committed");
        app->Close(reopened);
    }
}

void testPathArrayCountAndDistancePlacementHaveBoundedIndependentArcError() {
    const auto path = pathValue({{{0,0,0}}, {{5,8,0}}, {{10,0,0}}}, 2);
    std::uint32_t seed = 2400; auto value = pathArrayFixture(path, 6, seed);
    value.orientation.policy = path_array::OrientationPolicy::Tangent;
    value.arcLengthTolerance = 1e-5;
    std::vector<path_array::Placement> placements; path_array::BuildReceipt receipt;
    require(path_array::BuildPlacements(value, path, placements, receipt)
            == path_array::BuildRefusal::None, "count path build");
    require(placements.size() == 6 && receipt.maximumMeasuredArcError <= 1e-5,
            "count bounded inversion receipt");
    double maximumIndependentError = 0;
    for (const auto& placement : placements) {
        const double error = std::abs(independentlyMeasuredArcLength(path.value.definition,
            placement.parameter) - placement.requestedArcLength);
        maximumIndependentError = std::max(maximumIndependentError, error);
        require(error <= 2e-5,
                "independent count arc measurement");
    }

    value.distribution.mode = path_array::DistributionMode::Distance;
    value.distribution.distance = receipt.totalArcLength / 4.25;
    std::uint32_t required = 0; double total = 0;
    require(path_array::RequiredInstanceCount(value, path, required, total)
            == path_array::BuildRefusal::None, "distance required count");
    const pattern::IssueUUID issue = [&](pattern::UUID& output) {
        output = patternUUID(++seed); return true;
    };
    require(path_array::ReconcileMembers(value, required, issue), "distance reconcile");
    require(path_array::BuildPlacements(value, path, placements, receipt)
            == path_array::BuildRefusal::None, "distance path build");
    for (const auto& placement : placements) {
        const double error = std::abs(independentlyMeasuredArcLength(path.value.definition,
            placement.parameter) - placement.requestedArcLength);
        maximumIndependentError = std::max(maximumIndependentError, error);
        require(error <= 2e-5,
                "independent distance arc measurement");
    }
    std::cout << "D3 independent arc error " << maximumIndependentError
              << " <= 2e-05; inversion receipt <= 1e-05\n";
}

void testPathArrayTangentAlignmentAtPinnedParameters() {
    const auto path = pathValue({{{0,0,0}}, {{5,10,3}}, {{10,0,6}}}, 2);
    std::uint32_t seed = 2500; auto value = pathArrayFixture(path, 7, seed);
    value.orientation.policy = path_array::OrientationPolicy::Tangent;
    value.orientation.hasUpVector = true; value.orientation.upVector = {{0,0,1}};
    std::vector<path_array::Placement> placements; path_array::BuildReceipt receipt;
    require(path_array::BuildPlacements(value, path, placements, receipt)
            == path_array::BuildRefusal::None, "tangent build");
    for (const auto& placement : placements) {
        curve::Evaluation evaluated;
        require(curve::Evaluate(path.value.definition, placement.parameter, evaluated)
                == curve::EvaluationRefusal::None, "pinned evaluate");
        curve::Vector3 tangent{};
        require(curve::Normalize(evaluated.derivative, value.minimumTangent, tangent),
                "pinned tangent");
        const auto x = path_array::MatrixAxis(placement.occurrenceFrame, 0);
        require(curve::Dot(x, tangent) > 1 - 1e-12, "pinned tangent aligned");
        require(curve::Norm(curve::Subtract(evaluated.point,
            {{placement.occurrenceFrame[3], placement.occurrenceFrame[7],
              placement.occurrenceFrame[11]}})) < 1e-12, "pinned position aligned");
    }
}

void testClosedPathSeamNeverDuplicatesEndpoint() {
    const auto path = pathValue({{{0,0,0}}, {{10,0,0}}, {{10,10,0}},
                                 {{0,10,0}}, {{0,0,0}}});
    std::uint32_t seed = 2600; auto value = pathArrayFixture(path, 8, seed, true);
    value.orientation.policy = path_array::OrientationPolicy::Bishop;
    value.orientation.hasUpVector = true; value.orientation.upVector = {{0,0,1}};
    value.orientation.maximumFrameStepRadians = 2.0;
    std::vector<path_array::Placement> placements; path_array::BuildReceipt receipt;
    require(path_array::BuildPlacements(value, path, placements, receipt)
            == path_array::BuildRefusal::None, "closed path build");
    require(placements.size() == 8 && receipt.closedSeamCanonicalized,
            "closed seam canonicalized");
    const curve::Vector3 first{{placements.front().occurrenceFrame[3],
        placements.front().occurrenceFrame[7], placements.front().occurrenceFrame[11]}};
    for (std::size_t index = 1; index < placements.size(); ++index) {
        const curve::Vector3 point{{placements[index].occurrenceFrame[3],
            placements[index].occurrenceFrame[7], placements[index].occurrenceFrame[11]}};
        require(curve::Norm(curve::Subtract(first, point)) > value.arcLengthTolerance,
                "no duplicate closed seam instance");
    }
    require(placements.back().requestedArcLength < receipt.totalArcLength,
            "closed distribution half open");
}

void testZeroTangentCuspAndFrameFlipRefuse() {
    const auto zero = pathValue({{{0,0,0}}, {{0,0,0}}, {{5,0,0}}});
    require(curve::AuditTangents(zero.value.definition, 1e-12, 2.5)
            == curve::EvaluationRefusal::ZeroTangent, "zero tangent refusal");
    const auto cusp = pathValue({{{0,0,0}}, {{5,0,0}}, {{0,0,0}}});
    require(curve::AuditTangents(cusp.value.definition, 1e-12, 2.5)
            == curve::EvaluationRefusal::Cusp, "cusp refusal");
    const auto bend = pathValue({{{0,0,0}}, {{5,10,0}}, {{10,0,0}}}, 2);
    std::uint32_t seed = 2700; auto value = pathArrayFixture(bend, 2, seed);
    value.orientation.policy = path_array::OrientationPolicy::Tangent;
    value.orientation.maximumFrameStepRadians = 0.5;
    std::vector<path_array::Placement> placements; path_array::BuildReceipt receipt;
    require(path_array::BuildPlacements(value, bend, placements, receipt)
            == path_array::BuildRefusal::FrameFlip, "placement frame flip refusal");
}

void testFixedTangentAndBishopRollPolicies() {
    const auto path = pathValue({{{0,0,0}}, {{10,0,0}}});
    std::uint32_t seed = 2800; auto value = pathArrayFixture(path, 3, seed);
    std::vector<path_array::Placement> placements; path_array::BuildReceipt receipt;
    value.orientation.policy = path_array::OrientationPolicy::Fixed;
    require(path_array::BuildPlacements(value, path, placements, receipt)
            == path_array::BuildRefusal::None, "fixed policy");
    require(path_array::MatrixAxis(placements[1].occurrenceFrame, 1)
            == curve::Vector3({{0,1,0}}), "fixed source orientation");
    value.orientation.policy = path_array::OrientationPolicy::Tangent;
    value.orientation.hasUpVector = true; value.orientation.upVector = {{0,0,1}};
    require(path_array::BuildPlacements(value, path, placements, receipt)
            == path_array::BuildRefusal::None, "tangent policy");
    require(curve::Dot(path_array::MatrixAxis(placements[1].occurrenceFrame, 0),
                       {{1,0,0}}) > 1 - 1e-12, "tangent policy axis");
    value.orientation.policy = path_array::OrientationPolicy::Bishop;
    value.orientation.rollRadians = path_array::Pi * 0.5;
    require(path_array::BuildPlacements(value, path, placements, receipt)
            == path_array::BuildRefusal::None, "bishop roll policy");
    require(curve::Dot(path_array::MatrixAxis(placements[1].occurrenceFrame, 1),
                       {{0,-1,0}}) > 1 - 1e-12, "bishop roll applied once");
}

void testPathCountAndSourceEditsPreserveOrdinalIDsAndNeverReuseTail() {
    auto path = pathValue({{{0,0,0}}, {{5,8,0}}, {{10,0,0}}}, 2);
    std::uint32_t seed = 2900; auto value = pathArrayFixture(path, 5, seed);
    const auto original = value.members; const auto source = value.source;
    // A source recipe edit leaves the retained SourceIdentity unchanged.
    require(value.source == source, "source edit retains authority");
    auto editedPath = pathValue({{{0,0,0}}, {{5,12,0}}, {{10,0,0}}}, 2, 2);
    value.path.definitionRevision = editedPath.ownerState.definitionRevision;
    value.path.canonicalDefinitionDigest = editedPath.ownerState.canonicalDefinitionDigest;
    std::vector<path_array::Placement> placements; path_array::BuildReceipt receipt;
    value.orientation.policy = path_array::OrientationPolicy::Tangent;
    require(path_array::BuildPlacements(value, editedPath, placements, receipt)
            == path_array::BuildRefusal::None, "edited path rebuild");
    for (std::size_t index = 0; index < original.size(); ++index)
        require(value.members[index].identity == original[index].identity
                && value.members[index].localID == original[index].localID,
                "path edit ordinal identity stable");
    const pattern::IssueUUID issue = [&](pattern::UUID& output) {
        output = patternUUID(++seed); return true;
    };
    require(path_array::ReconcileMembers(value, 3, issue), "path count down");
    const auto removed = value.removals;
    require(path_array::ReconcileMembers(value, 5, issue), "path count up");
    for (std::size_t index = 0; index < 3; ++index)
        require(value.members[index].identity == original[index].identity,
                "surviving prefix stable");
    for (const auto& member : value.members) for (const auto& retired : removed)
        require(member.identity != retired.identity && member.localID != retired.localID,
                "retired tail never reused");
}

void testPathArrayTopologyCountDocumentMemoryBudgetsRefuseBeforeMutation() {
    path_array::AdmissionBudget budget; budget.maximumInstances = 32;
    budget.sourceTopologyNodes = 1000; budget.existingTopologyNodes = 500;
    budget.maximumAggregateTopologyNodes = 10'500;
    budget.sourceDocumentBytes = 100; budget.existingDocumentBytes = 200;
    budget.maximumDocumentBytes = 4096;
    budget.sourceMemoryBytes = 1000; budget.existingMemoryBytes = 2000;
    budget.maximumMemoryBytes = 20'000;
    const auto admitted = path_array::Project(10, budget);
    require(admitted.admitted && admitted.aggregateTopologyNodes == 10'500,
            "aggregate topology admitted at bound");
    require(!path_array::Project(11, budget).admitted, "aggregate topology refusal");
    budget.maximumAggregateTopologyNodes = 100'000;
    budget.maximumMemoryBytes = admitted.projectedMemoryBytes - 1;
    const auto memoryRefusal = path_array::Project(10, budget);
    require(!memoryRefusal.admitted
            && memoryRefusal.projectedMemoryBytes == admitted.projectedMemoryBytes,
            "memory projection refusal reported");
    require(!path_array::Project(path_array::MaximumInstances + 1, budget).admitted,
            "instance bound refusal");
}

void testPathArrayCanonicalPersistenceRejectsTamperAndRetainsReferences() {
    const auto path = pathValue({{{0,0,0}}, {{10,0,0}}});
    std::uint32_t seed = 3000; auto value = pathArrayFixture(path, 4, seed);
    value.distribution.includeStart = false; value.distribution.includeEnd = true;
    value.orientation.policy = path_array::OrientationPolicy::Bishop;
    value.orientation.rollRadians = 0.25; value.orientation.hasUpVector = true;
    value.orientation.upVector = {{0,1,1}};
    std::vector<std::uint8_t> bytes;
    require(path_array::Encode(value, bytes), "path array encode");
    path_array::Definition decoded;
    require(path_array::Decode(bytes, decoded), "path array decode");
    require(decoded.source == value.source && decoded.path.owner == value.path.owner
            && decoded.path.feature == value.path.feature
            && decoded.path.canonicalDefinitionDigest == value.path.canonicalDefinitionDigest
            && decoded.orientation.rollRadians == value.orientation.rollRadians,
            "source path and editor parameters retained");
    bytes[bytes.size() / 2] ^= 1;
    require(!path_array::Decode(bytes, decoded), "path array tamper refused");
}

void testPathArrayEditUndoRedoSaveColdReopenLaterEditBothUnits() {
    const auto path = pathValue({{{0,0,0}}, {{10,0,0}}});
    for (double unit : {0.001, 1.0}) {
        std::uint32_t seed = unit < 1 ? 3100 : 3200;
        auto value = pathArrayFixture(path, 4, seed);
        const auto stable = value.members[1]; const auto doc = document(unit);
        path_array::Record staged; doc->NewCommand();
        require(path_array::Stage(doc, value, staged), "initial path stage");
        require(doc->CommitCommand(), "initial path commit");
        value.orientation.rollRadians = 0.5;
        doc->NewCommand(); require(path_array::Stage(doc, value, staged), "edited path stage");
        require(doc->CommitCommand(), "edited path commit");
        path_array::Record readback;
        require(doc->Undo() && path_array::ReadFeature(doc, value.feature, readback)
                && readback.definition.orientation.rollRadians == 0, "path undo");
        require(doc->Redo() && path_array::ReadFeature(doc, value.feature, readback)
                && readback.definition.orientation.rollRadians == 0.5, "path redo");
        std::ostringstream bytes(std::ios::binary);
        const auto app = Handle(TDocStd_Application)::DownCast(doc->Application());
        require(!app.IsNull() && app->SaveAs(doc, bytes) == PCDM_SS_OK, "save path array");
        std::istringstream input(bytes.str(), std::ios::binary); Handle(TDocStd_Document) reopened;
        require(app->Open(input, reopened) == PCDM_RS_OK, "cold reopen path array");
        require(path_array::ReadFeature(reopened, value.feature, readback)
                && readback.definition.members[1].identity == stable.identity,
                "reopen path member identity");
        auto later = readback.definition; later.distribution.includeEnd = false;
        reopened->SetUndoLimit(10); reopened->NewCommand();
        require(path_array::Stage(reopened, later, readback), "later path edit stage");
        require(reopened->CommitCommand()
                && path_array::ReadFeature(reopened, later.feature, readback)
                && !readback.definition.distribution.includeEnd, "later path edit committed");
        app->Close(reopened);
    }
}

void testAnalyticBooleanCloneGlobalIdentityMapNeverAliasesLocalIDs() {
    clone::IdentityMap map; std::set<clone::UUID> occupied;
    occupied.insert(uuid(1)); occupied.insert(uuid(21));
    std::uint8_t next = 101;
    const clone::IssueUUID issue = [&](clone::UUID& value) {
        value = uuid(next); next = std::uint8_t(next + 20); return true;
    };
    clone::UUID first{}, repeated{}, second{};
    require(clone::IssueMapped(uuid(1), issue, occupied, map, first), "issue first global");
    require(clone::IssueMapped(uuid(1), issue, occupied, map, repeated), "reuse mapped global");
    require(clone::IssueMapped(uuid(21), issue, occupied, map, second), "issue second global");
    require(first == repeated && first != second && first != uuid(1) && second != uuid(21),
            "global identity bijection");
    const core3d::retained_recipe::IssuanceState local{7, {4, 6}};
    require(local.nextLocalID == 7 && local.retiredLocalIDs == std::vector<std::uint64_t>({4, 6}),
            "owner-local issuance is not a uuid namespace");
}

void testAnalyticBooleanCloneIdentityIssuerRefusesCollision() {
    clone::IdentityMap map; std::set<clone::UUID> occupied{uuid(90)};
    const clone::IssueUUID collision = [](clone::UUID& value) {
        value = uuid(90); return true;
    };
    clone::UUID output{};
    require(!clone::IssueMapped(uuid(10), collision, occupied, map, output)
            && map.global.empty() && !core3d::retained_recipe::Nonzero(output),
            "colliding clone identity refuses without partial map");
}
} // namespace

int main() {
    try {
        testIndependentRecipeClonePreservesLocalIDsAndIssuesFeatureIdentity<sweep::Definition>();
        testIndependentRecipeClonePreservesLocalIDsAndIssuesFeatureIdentity<loft::Definition>();
        testMirrorCloneComposesFrameAndKeepsValidForwardSolid<sweep::Definition>();
        testMirrorCloneComposesFrameAndKeepsValidForwardSolid<loft::Definition>();
        testStaleSweepAndLoftOwnersRefuseInsteadOfBecomingCurrentCopies();
        testColdReopenPreservesIndependentRecipeBytesInBothUnitSystems();
        testExactLinearRadialAndGridTransformLattices();
        testCountAndProjectedDocumentMemoryBudgetsRefuseBeforeMutation();
        testZeroStepAndZeroAngleNeverCreateDuplicateMembers();
        testNegativeSpacingAndRotatedSourceFrameRemainExact();
        testCountDownThenUpPreservesSurvivorsAndNeverReusesRemovedIDs();
        testSuppressionPreservesIdentityWithoutRemoval();
        testPatternPersistenceRoundTripRejectsTamperAndKeepsSourceAuthority();
        testSourceRecipeChangeReusesPatternMemberIdentities();
        testPatternEditUndoRedoSaveColdReopenAndLaterEditInBothUnitSystems();
        testCutFeaturePatternRetainsHostSourceDistributionAndLegacyRingBytes();
        testCutFeaturePatternPlacementVolumeBoundaryAndAttributionAreIndependent();
        testCutFeaturePatternChildIDsSurviveCountSourceAndHostEdits();
        testCutFeaturePatternTypedRefusalsIncludeMissingOneFeature();
        testCutFeaturePatternReplayParityAndTamperRefusal();
        testCutFeaturePatternEditUndoRedoSaveColdReopenLaterEditBothUnits();
        testPathArrayCountAndDistancePlacementHaveBoundedIndependentArcError();
        testPathArrayTangentAlignmentAtPinnedParameters();
        testClosedPathSeamNeverDuplicatesEndpoint();
        testZeroTangentCuspAndFrameFlipRefuse();
        testFixedTangentAndBishopRollPolicies();
        testPathCountAndSourceEditsPreserveOrdinalIDsAndNeverReuseTail();
        testPathArrayTopologyCountDocumentMemoryBudgetsRefuseBeforeMutation();
        testPathArrayCanonicalPersistenceRejectsTamperAndRetainsReferences();
        testPathArrayEditUndoRedoSaveColdReopenLaterEditBothUnits();
        testAnalyticBooleanCloneGlobalIdentityMapNeverAliasesLocalIDs();
        testAnalyticBooleanCloneIdentityIssuerRefusesCollision();
        std::cout << "PASS D1a/D1-C, D2 and D4 retained pattern identity, admission, persistence and replay\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n'; return 1;
    }
}
