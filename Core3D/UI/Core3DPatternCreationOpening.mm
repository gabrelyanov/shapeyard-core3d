//
//  Core3DPatternCreationOpening.mm
//  Core3D
//
//  PC-D2-C production D2 retained-pattern creator. Composes the header-level
//  pattern build/recipe-clone/persistence kernels with the public OcctDocument
//  identity-issuance and all-label staging seams. No existing OCCTKit file is
//  modified; no DEBUG R179 staging producer or raw attribute identity write is
//  used to create the product result.
//

#import "Core3DPatternCreationOpening.h"
#import "GLViewController+Trick.h"
#import "Core3DViewer.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/PatternOwnerBridge.hxx"
#include "../OCCTKit/PatternAllLabelAuthority.hxx"

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#if DEBUG
#include <cstdio>
#endif

namespace {
using core3d::pattern::Axis;
using core3d::pattern::Coordinate;
using core3d::pattern::Kind;
using core3d::pattern::UUID;
namespace recipe = core3d::pattern_recipe_clone;

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double Degrees(double radians) noexcept { return radians * 180.0 / kPi; }
constexpr double Radians(double degrees) noexcept { return degrees * kPi / 180.0; }
enum class State : std::uint8_t { Open, Prepared, Applying, Cancelled, Settled, Recovery };

constexpr NSInteger ClampToNSInteger(std::size_t value) noexcept {
    constexpr auto maximum = static_cast<std::size_t>(NSIntegerMax);
    return value > maximum ? NSIntegerMax : static_cast<NSInteger>(value);
}

NSString *Text(const std::string& value) {
    return [[NSString alloc] initWithBytes:value.data()
                                    length:value.size()
                                  encoding:NSUTF8StringEncoding] ?: @"";
}

NSString *UUIDText(const UUID& value) {
    try { return Text(core3d::retained_solid::UUIDText(value)); }
    catch (...) { return @""; }
}

NSString *KindText(Kind value) {
    switch (value) {
        case Kind::Linear: return @"linear";
        case Kind::Radial: return @"radial";
        case Kind::Grid: return @"grid";
    }
}

NSString *FamilyText(recipe::Family value) {
    switch (value) {
        case recipe::Family::None: return @"none";
        case recipe::Family::Sweep: return @"sweep";
        case recipe::Family::Loft: return @"loft";
        case recipe::Family::AnalyticBoolean: return @"analyticBoolean";
    }
}

NSString *UnitSymbol(double metersPerUnit) {
    if (std::isfinite(metersPerUnit)) {
        if (std::abs(metersPerUnit - 1.0) <= 1e-12) return @"m";
        if (std::abs(metersPerUnit - 0.001) <= 1e-15) return @"mm";
    }
    return @"document";
}

bool ExactKeys(NSDictionary *value, NSArray<NSString *> *keys) {
    if (![value isKindOfClass:NSDictionary.class] || value.count != keys.count) return false;
    NSSet *expected = [NSSet setWithArray:keys];
    return [[NSSet setWithArray:value.allKeys] isEqualToSet:expected];
}

bool Number(id value, double& output) {
    if (![value isKindOfClass:NSNumber.class]
        || CFGetTypeID((__bridge CFTypeRef)value) == CFBooleanGetTypeID()) return false;
    output = [value doubleValue];
    return std::isfinite(output);
}

bool Integer(id value, std::uint64_t maximum, std::uint64_t& output) {
    double scalar = 0;
    if (!Number(value, scalar) || scalar < 0 || scalar > double(maximum)
        || std::trunc(scalar) != scalar) return false;
    const auto converted = [value unsignedLongLongValue];
    if (converted > maximum || double(converted) != scalar) return false;
    output = converted;
    return true;
}

bool String(id value, NSString *__strong& output) {
    if (![value isKindOfClass:NSString.class] || [value length] == 0) return false;
    output = value; return true;
}

bool Vector3(id value, std::array<double, 3>& output) {
    if (![value isKindOfClass:NSArray.class] || [value count] != 3) return false;
    for (NSUInteger index = 0; index < 3; ++index)
        if (!Number(value[index], output[index])) return false;
    return true;
}

bool ParseKind(id value, Kind& output) {
    NSString *text = nil;
    if (!String(value, text)) return false;
    if ([text isEqualToString:@"linear"]) output = Kind::Linear;
    else if ([text isEqualToString:@"radial"]) output = Kind::Radial;
    else if ([text isEqualToString:@"grid"]) output = Kind::Grid;
    else return false;
    return true;
}

bool ParseAxis(id value, Axis& output) {
    std::uint64_t scalar = 0;
    if (!Integer(value, 2, scalar)) return false;
    output = Axis(scalar); return true;
}

template <class Outcome>
Core3DProfileConstructionResult MapOutcome(Outcome value, Outcome committed,
                                            Outcome unknown) {
    if (value == committed) return Core3DProfileConstructionResultCommitted;
    if (value == unknown) return Core3DProfileConstructionResultRecoveryRequired;
    return Core3DProfileConstructionResultRejected;
}

void DeliverPrepared(void (^completion)(Core3DBoundedCurvePreparationResult, NSString *),
                     Core3DBoundedCurvePreparationResult result, NSString *detail) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail);
    else dispatch_async(dispatch_get_main_queue(), ^{ completion(result, detail); });
}

void DeliverCreated(void (^completion)(Core3DProfileConstructionResult, NSString *, NSString *),
                    Core3DProfileConstructionResult result, NSString *detail,
                    NSString *entityIdentifier) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail, entityIdentifier);
    else dispatch_async(dispatch_get_main_queue(), ^{
        completion(result, detail, entityIdentifier); });
}

// Mint one durable identity through the public profile-identifier seam. Raw
// NSUUID attribute writes (the R179 DEBUG fixture idiom) are never used here.
bool MintIdentity(UUID& output) {
    output = {};
    for (unsigned attempt = 0; attempt < 16; ++attempt) {
        const std::string text = OcctDocument::NewProfileIdentifier();
        if (!text.empty() && core3d::pattern_owner::Parse(text, output)) return true;
    }
    return false;
}

std::size_t RecipeBytes(const recipe::Source& source) noexcept {
    using Family = recipe::Family;
    try {
        if (source.family == Family::Sweep)
            return source.sweep.values.size() * sizeof(double);
        if (source.family == Family::Loft)
            return source.loft.values.size() * sizeof(double);
        if (source.family == Family::AnalyticBoolean && source.analyticBoolean.value)
            return source.analyticBoolean.value->bytes.size();
    } catch (...) {}
    return 0;
}

bool MeasureSource(const TopoDS_Shape& shape, std::size_t recipeBytes,
                   core3d::pattern_owner::AllLabelMeasuredCost& output) noexcept {
    output = {};
    try {
        std::string bytes;
        if (shape.IsNull()
            || !core3d::retained_part_boolean::ExactShapeBytes(shape, bytes)) return false;
        TopTools_IndexedMapOfShape topology;
        TopExp::MapShapes(shape, topology);
        output.recipeBytes = recipeBytes;
        output.shapeBytes = bytes.size();
        output.topologyNodes = topology.Extent();
        output.retainedMemoryBytes = sizeof(output);
        return core3d::pattern::CheckedAdd(output.recipeBytes, output.retainedMemoryBytes,
                                           output.retainedMemoryBytes)
            && core3d::pattern::CheckedAdd(output.shapeBytes, output.retainedMemoryBytes,
                                           output.retainedMemoryBytes);
    } catch (...) { output = {}; return false; }
}

// Fail-closed single-source capture from the exact current whole-object
// selection, mirroring the existing controller opening-input seam.
struct CreationCapture {
    Handle(OcctDocument) owner;
    std::shared_ptr<core3d::native_opening::Context> context;
    std::uint32_t viewportWidth = 0;
    std::uint32_t viewportHeight = 0;
    std::string selected;
    TDF_Label sourceLabel;
    TopoDS_Shape sourceShape;
    recipe::Source sourceRecipe;
    OcctExactLabelReceipt sourceReceipt;
    core3d::pattern_owner::AllLabelMeasuredCost measured;
    std::size_t patternDocumentBytes = 0;
    std::size_t compositeDocumentBytes = 0;
    Standard_Integer documentTime = -1;
    double metersPerUnit = 0;
};

bool ResolveFreeLabel(OcctDocument& owner, const UUID& entity,
                      core3d::pattern_owner::LabelReceipt& output) noexcept {
    output = {};
    try {
        const Handle(TDocStd_Document) document = owner.Document();
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return false;
        TDF_LabelSequence roots;
        shapes->GetFreeShapes(roots);
        core3d::pattern_owner::LabelReceipt found;
        bool matched = false;
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            core3d::pattern_owner::LabelReceipt receipt;
            if (!core3d::pattern_owner::ReadReceipt(owner, roots.Value(index), receipt))
                return false;
            if (receipt.entity != entity) continue;
            if (matched) return false;
            found = receipt; matched = true;
        }
        if (!matched) return false;
        output = found; return true;
    } catch (...) { output = {}; return false; }
}

bool CaptureCreationSource(Core3DViewController *controller,
                           CreationCapture& output) noexcept {
    output = {};
    const char *gate = "main-thread";
    const auto refused = [&]() -> bool {
#if DEBUG
        NSLog(@"D2C_CREATE refused=%s", gate);
#else
        (void)gate;
#endif
        return false;
    };
    try {
        if (!NSThread.isMainThread || !controller) return refused();
        Core3DSceneSnapshot *scene = [controller captureSceneSnapshot];
        if (!scene || scene.selection.selectedElements.count != 1) return (gate = "scene-selection", refused());
        Core3DSceneElementIdentifier *element = scene.selection.selectedElements.firstObject;
        if (element.kind != Core3DSceneElementKindObject
            || element.entityIdentifier.length == 0
            || element.entityIdentifier.length > 128) return (gate = "selected-object", refused());
        GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)controller.glController : nil;
        if (!gl) return (gate = "gl-controller", refused());
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl.viewer;
        const CGSize drawable = controller.viewportDrawableSize;
        if (!viewer || !std::isfinite(drawable.width) || !std::isfinite(drawable.height)
            || drawable.width < 1 || drawable.height < 1
            || drawable.width > UINT32_MAX || drawable.height > UINT32_MAX) return (gate = "viewer-viewport", refused());
        CreationCapture value;
        value.owner = viewer->getDocument();
        const char *utf8 = element.entityIdentifier.UTF8String;
        if (!utf8) return false;
        value.selected.assign(utf8, [element.entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        value.viewportWidth = std::uint32_t(drawable.width);
        value.viewportHeight = std::uint32_t(drawable.height);
        value.context = viewer->captureNativeOpeningContext(
            value.viewportWidth, value.viewportHeight, {value.selected});
        if (value.owner.IsNull() || !value.context || value.selected.empty()) return (gate = "document-context", refused());
        const Handle(TDocStd_Document) document = value.owner->Document();
        if (document.IsNull() || document->GetData().IsNull()
            || document->HasOpenCommand()
            || value.context->openingFence().document() != document
            || value.context->openingFence().data() != document->GetData()
            || !value.context->isCurrent(value.viewportWidth, value.viewportHeight)) return (gate = "document-fence", refused());
        value.metersPerUnit = value.context->openingFence().metersPerUnit();
        if (!std::isfinite(value.metersPerUnit) || value.metersPerUnit <= 0) return (gate = "units", refused());
        UUID selected{};
        if (!core3d::pattern_owner::Parse(value.selected, selected)) return (gate = "selected-uuid", refused());
        core3d::pattern_owner::LabelReceipt receipt;
        if (!ResolveFreeLabel(*value.owner, selected, receipt)) return (gate = "source-label", refused());
        // A label already retained by an existing D2/D3 record is an edit
        // target, never a creation source.
        std::vector<core3d::pattern::Record> records;
        std::vector<core3d::path_array::Record> arrays;
        if (!core3d::pattern::ReadAll(document, records)
            || !core3d::path_array::ReadAll(document, arrays)) return (gate = "retained-tables", refused());
        for (const auto& record : records) {
            value.patternDocumentBytes += record.bytes.size();
            if (record.definition.source.entity == selected)
                return (gate = "already-retained-d2", refused());
            for (const auto& member : record.definition.members)
                if (member.identity == selected)
                    return (gate = "already-retained-d2", refused());
        }
        for (const auto& array : arrays)
            for (const auto& member : array.definition.members)
                if (member.identity == selected)
                    return (gate = "already-retained-d3", refused());
        std::vector<core3d::composite_recipe::Record> composites;
        if (!core3d::composite_recipe::ReadAll(document, composites,
                                               value.patternDocumentBytes)) return (gate = "composite-table", refused());
        for (const auto& composite : composites)
            value.compositeDocumentBytes += composite.value->bytes.size();
        value.sourceLabel = receipt.label;
        value.sourceShape = receipt.shape;
        if (!recipe::Capture(document, value.sourceLabel, value.sourceRecipe)) return (gate = "source-recipe", refused());
        if (!value.owner->CaptureExactFreeLabel(value.sourceLabel, value.sourceReceipt))
            return (gate = "source-receipt", refused());
        if (!MeasureSource(value.sourceShape, RecipeBytes(value.sourceRecipe),
                           value.measured)) return (gate = "source-measure", refused());
        value.documentTime = document->GetData()->Time();
        output = std::move(value);
        return true;
    } catch (...) { output = {}; return (gate = "exception", refused()); }
}

// Pre-mutation and pre-commit currentness fence: the live document, data and
// scene are unchanged since capture, and the source label/recipe/receipt
// still read exactly as captured.
bool CreationCurrent(const CreationCapture& capture) noexcept {
    try {
        if (capture.owner.IsNull() || !capture.context) return false;
        const Handle(TDocStd_Document) document = capture.owner->Document();
        if (document.IsNull() || document->HasOpenCommand()
            || document->GetData()->Time() != capture.documentTime
            || !capture.context->isCurrent(
                capture.viewportWidth, capture.viewportHeight)) return false;
        UUID selected{};
        if (!core3d::pattern_owner::Parse(capture.selected, selected)) return false;
        core3d::pattern_owner::LabelReceipt receipt;
        if (!ResolveFreeLabel(*capture.owner, selected, receipt)
            || !receipt.label.IsEqual(capture.sourceLabel)) return false;
        recipe::Source live;
        if (!recipe::Capture(document, receipt.label, live)
            || !recipe::IsEqual(live, capture.sourceRecipe)) return false;
        OcctExactLabelReceipt exact;
        return capture.owner->CaptureExactFreeLabel(capture.sourceLabel, exact)
            && exact.IsEqual(capture.sourceReceipt);
    } catch (...) { return false; }
}

struct CreationRecipe {
    std::string entityIdentifier;
    std::string featureIdentifier;
    recipe::Prepared prepared;
};

struct CreationPrepared {
    core3d::pattern::Definition definition;
    core3d::pattern::Projection projection;
    std::vector<core3d::pattern::Placement> placements;
    std::vector<OcctIssuedLabelIdentity> issued;
    OcctAllLabelPlan labels;
    std::vector<CreationRecipe> recipes;
    std::vector<std::uint8_t> canonicalBytes;
    Standard_Size projectedTopologyNodes = 0;
};

bool ParseCandidate(NSDictionary *value, double metersPerUnit,
                    core3d::pattern::Definition& output) {
    if (!ExactKeys(value, @[@"kind", @"rowAxis", @"columnAxis", @"rowCount",
            @"columnCount", @"rowSpacingMM", @"columnSpacingMM",
            @"radialPivotMM", @"sweepDegrees"])) return false;
    std::uint64_t rows = 0, columns = 0; double sweep = 0;
    if (!std::isfinite(metersPerUnit) || metersPerUnit <= 0) return false;
    const bool valid = ParseKind(value[@"kind"], output.kind)
        && ParseAxis(value[@"rowAxis"], output.rowAxis)
        && ParseAxis(value[@"columnAxis"], output.columnAxis)
        && Integer(value[@"rowCount"], UINT32_MAX, rows)
        && Integer(value[@"columnCount"], UINT32_MAX, columns)
        && Number(value[@"rowSpacingMM"], output.rowSpacing)
        && Number(value[@"columnSpacingMM"], output.columnSpacing)
        && Vector3(value[@"radialPivotMM"], output.radialPivotLocal)
        && Number(value[@"sweepDegrees"], sweep)
        && (output.rowCount = std::uint32_t(rows),
            output.columnCount = std::uint32_t(columns),
            output.sweepRadians = Radians(sweep), true);
    if (!valid) return false;
    const double millimetresPerUnit = metersPerUnit * 1000.0;
    output.rowSpacing /= millimetresPerUnit;
    output.columnSpacing /= millimetresPerUnit;
    for (double& pivot : output.radialPivotLocal) pivot /= millimetresPerUnit;
    return true;
}

bool TransformFor(const core3d::pattern::Matrix& matrix, gp_Trsf& output) noexcept {
    try {
        output.SetValues(matrix[0], matrix[1], matrix[2], matrix[3],
                         matrix[4], matrix[5], matrix[6], matrix[7],
                         matrix[8], matrix[9], matrix[10], matrix[11]);
        return recipe::IsProperRigidPlacement(output);
    } catch (...) { return false; }
}

// Detached independent member shape at the member placement. The coordinate
// {0,0} source keeps its own label and is never copied here.
bool DetachedMemberShape(const TopoDS_Shape& source,
                         const core3d::pattern::Matrix& world,
                         TopoDS_Shape& output, gp_Trsf& placement) noexcept {
    output.Nullify();
    try {
        if (!TransformFor(world, placement)) return false;
        BRepBuilderAPI_Transform copy(source, placement, Standard_True);
        if (!copy.IsDone() || copy.Shape().IsNull()
            || copy.Shape().IsPartner(source)) return false;
        output = copy.Shape();
        return true;
    } catch (...) { output.Nullify(); return false; }
}

// Admission + native identity issuance + detached preparation. Pure: no OCAF
// write, no command. Every refusal here leaves the document untouched.
bool PrepareCreation(const CreationCapture& capture, NSDictionary *candidate,
                     CreationPrepared& output, NSString *__strong& refusal) {
    output = {};
    refusal = @"Malformed retained-pattern creation values.";
    try {
        if (capture.owner.IsNull() || !capture.context || !CreationCurrent(capture)) {
            refusal = @"The selected source changed before preparation; nothing was created.";
            return false;
        }
        const Handle(TDocStd_Document) document = capture.owner->Document();
        OcctDocument& owner = *capture.owner;
        core3d::pattern::Definition definition;
        if (!ParseCandidate(candidate, capture.metersPerUnit, definition)) return false;
        UUID documentID{}, ownerEntity{}, ownerDefinition{}, feature{},
            sourceEntity{}, sourceDefinition{}, sourceFeature{};
        if (!core3d::pattern_owner::Parse(owner.DocumentIdentifier(), documentID)
            || !core3d::pattern_owner::Parse(capture.selected, sourceEntity)
            || !core3d::pattern_owner::Parse(
                owner.DefinitionIdentifierForLabel(capture.sourceLabel), sourceDefinition)
            || !MintIdentity(ownerEntity) || !MintIdentity(ownerDefinition)
            || !MintIdentity(feature) || !MintIdentity(sourceFeature)) return false;
        const std::set<UUID> identities{
            documentID, ownerEntity, ownerDefinition, feature,
            sourceEntity, sourceDefinition, sourceFeature};
        if (identities.size() != 7) return false;
        definition.owner = {documentID, ownerEntity, ownerDefinition};
        definition.feature = feature;
        definition.source = {documentID, sourceEntity, sourceDefinition, sourceFeature};
        std::uint32_t count = 0;
        if (!core3d::pattern::Product(definition.rowCount, definition.columnCount, count)
            || count < 2 || count > core3d::pattern::MaximumInstances) {
            refusal = @"Instance count must be between 2 and 256.";
            return false;
        }
        std::vector<OcctIssuedLabelIdentity> issued;
        if (!owner.ReserveExactLabelIdentities(Standard_Size(count - 1), {}, issued)
            || issued.size() != count - 1) return false;
        definition.issuance.nextLocalID = 1;
        core3d::pattern::Member sourceMember;
        sourceMember.identity = sourceEntity;
        sourceMember.localID = definition.issuance.nextLocalID++;
        sourceMember.coordinate = Coordinate{};
        sourceMember.state = core3d::pattern::MemberState::Active;
        definition.members.push_back(sourceMember);
        std::size_t nextIssued = 0;
        const core3d::pattern::IssueUUID issue = [&](UUID& value) {
            return nextIssued < issued.size()
                && core3d::pattern_owner::Parse(
                    issued[nextIssued++].EntityIdentifier(), value);
        };
        for (std::uint32_t row = 0; row < definition.rowCount; ++row)
            for (std::uint32_t column = 0; column < definition.columnCount; ++column) {
                const Coordinate coordinate{row, column};
                if (coordinate == Coordinate{}) continue;
                core3d::pattern::Member member;
                if (!core3d::pattern::IssueMember(definition, coordinate, issue, member))
                    return false;
                definition.members.push_back(member);
            }
        if (nextIssued != issued.size()) return false;
        std::sort(definition.members.begin(), definition.members.end(),
                  [](const core3d::pattern::Member& a, const core3d::pattern::Member& b) {
                      return a.coordinate < b.coordinate;
                  });
        if (!core3d::pattern::Valid(definition)) {
            refusal = @"Kind, axes, counts, spacings, pivot and sweep are bounded native admissions.";
            return false;
        }
        core3d::pattern::AdmissionBudget budget;
        budget.existingDocumentBytes =
            capture.patternDocumentBytes + capture.compositeDocumentBytes;
        budget.documentLimitBytes = core3d::pattern::MaximumDocumentPatternBytes;
        budget.existingMemoryBytes = 0;
        budget.memoryLimitBytes = 256 * 1024 * 1024;
        budget.sourceDocumentBytes = capture.measured.shapeBytes;
        budget.sourceMemoryBytes = capture.measured.retainedMemoryBytes;
        budget.maximumInstances = core3d::pattern::MaximumInstances;
        output.projection = core3d::pattern::Project(definition, budget);
        if (!output.projection.admitted) {
            refusal = output.projection.projectedDocumentBytes > budget.documentLimitBytes
                ? @"The projected document cost exceeds the retained-pattern budget."
                : @"The projected memory cost exceeds the retained-pattern budget.";
            return false;
        }
        if (!core3d::pattern::BuildPlacements(definition, output.placements)) return false;
        constexpr Standard_Size kTopologyLimit = 32'768;
        if (capture.measured.topologyNodes != 0
            && output.placements.size() > kTopologyLimit / capture.measured.topologyNodes) {
            refusal = @"The projected topology cost exceeds the retained-pattern budget.";
            return false;
        }
        output.projectedTopologyNodes =
            capture.measured.topologyNodes * output.placements.size();
        std::set<std::string> featureIDs;
        const std::string sourceFeatureID =
            core3d::pattern_owner::RecipeFeatureIdentifier(capture.sourceRecipe);
        if (!sourceFeatureID.empty()) featureIDs.insert(sourceFeatureID);
        std::size_t issuedIndex = 0;
        for (const auto& member : definition.members) {
            if (member.coordinate == Coordinate{}) continue;
            if (issuedIndex >= issued.size()
                || issued[issuedIndex].EntityIdentifier()
                    != core3d::retained_solid::UUIDText(member.identity)) return false;
            const core3d::pattern::Placement *placement = nullptr;
            for (const auto& item : output.placements)
                if (item.identity == member.identity) { placement = &item; break; }
            if (!placement) return false;
            TopoDS_Shape shape; gp_Trsf transform;
            if (!DetachedMemberShape(capture.sourceShape, placement->worldFrame,
                                     shape, transform)) return false;
            OcctPreparedLabelClone clone;
            clone.source = capture.sourceReceipt;
            clone.detachedShape = shape;
            clone.representation =
                capture.sourceReceipt.visibility.object.object.resolvedRepresentation;
            CreationRecipe recipeEntry;
            recipeEntry.entityIdentifier = issued[issuedIndex].EntityIdentifier();
            if (capture.sourceRecipe.family != recipe::Family::None) {
                bool minted = false;
                for (unsigned attempt = 0; attempt < 16 && !minted; ++attempt) {
                    std::string candidateID = OcctDocument::NewProfileIdentifier();
                    minted = !candidateID.empty() && featureIDs.insert(candidateID).second;
                    if (minted) recipeEntry.featureIdentifier = candidateID;
                }
                if (!minted) return false;
            }
            const std::optional<gp_Trsf> baked = transform;
            if (!recipe::Prepare(capture.sourceRecipe, shape,
                                 recipeEntry.featureIdentifier, baked,
                                 recipeEntry.prepared)) return false;
            output.labels.creates.push_back({issued[issuedIndex], clone});
            output.recipes.push_back(std::move(recipeEntry));
            ++issuedIndex;
        }
        if (issuedIndex != issued.size()) return false;
        if (!core3d::pattern::Encode(definition, output.canonicalBytes)) return false;
        output.definition = std::move(definition);
        output.issued = std::move(issued);
        return true;
    } catch (...) { output = {}; return false; }
}

// The created-member recipe arm of the edit path's recipe stager. Sweep and
// loft records stage through their header-level persistence kernels; analytic
// Boolean clones stage through the production independent-clone attribute.
bool StageCreatedRecipe(const Handle(TDocStd_Document)& document,
                        const TDF_Label& label,
                        const CreationRecipe& entry) noexcept {
    try {
        const auto& prepared = entry.prepared;
        if (prepared.family == recipe::Family::None) return true;
        if (prepared.family == recipe::Family::Sweep)
            return core3d::sweep_persistence::Stage(document, label, prepared.sweep,
                                                    entry.featureIdentifier);
        if (prepared.family == recipe::Family::Loft)
            return core3d::loft_persistence::Stage(document, label, prepared.loft,
                                                   entry.featureIdentifier);
        if (prepared.family != recipe::Family::AnalyticBoolean)
            return false;
        return core3d::composite_recipe::Attribute::StageIndependentClone(
            document, label, prepared.binding, prepared.analyticBoolean);
    } catch (...) { return false; }
}

bool PreparedRecipeMatches(const recipe::Prepared& expected,
                           const recipe::Source& actual) noexcept {
    try {
        if (expected.family != actual.family) return false;
        if (actual.family == recipe::Family::None) return true;
        if (core3d::pattern_owner::RecipeFeatureIdentifier(actual)
                != expected.featureIdentifier) return false;
        if (actual.family == recipe::Family::Sweep) {
            std::vector<double> expectedBytes, actualBytes;
            return core3d::sweep_persistence::Encode(expected.sweep, expectedBytes)
                && core3d::sweep_persistence::Encode(actual.sweep.definition, actualBytes)
                && expectedBytes == actualBytes;
        }
        if (actual.family == recipe::Family::Loft) {
            std::vector<double> expectedBytes, actualBytes;
            return core3d::loft_persistence::Encode(expected.loft, expectedBytes)
                && core3d::loft_persistence::Encode(actual.loft.definition, actualBytes)
                && expectedBytes == actualBytes;
        }
        return actual.family == recipe::Family::AnalyticBoolean
            && expected.analyticBoolean && actual.analyticBoolean.value
            && expected.analyticBoolean->bytes == actual.analyticBoolean.value->bytes;
    } catch (...) { return false; }
}

// Exact read-back of the staged creation, valid inside and after the command.
bool ReadBackCreation(OcctDocument& owner, const CreationCapture& capture,
                      const CreationPrepared& prepared,
                      const std::vector<OcctExactLabelReceipt>& receipts) noexcept {
    try {
        const Handle(TDocStd_Document) document = owner.Document();
        if (document.IsNull() || !owner.ReadBackAllLabels(prepared.labels, receipts))
            return false;
        core3d::pattern::Record record;
        if (!core3d::pattern::ReadFeature(document, prepared.definition.feature, record)
            || record.label.IsNull() || record.bytes != prepared.canonicalBytes)
            return false;
        for (const auto& entry : prepared.recipes) {
            const auto found = std::find_if(receipts.begin(), receipts.end(),
                [&](const OcctExactLabelReceipt& value) {
                    return value.visibility.object.object.entityIdentifier
                        == entry.entityIdentifier;
                });
            recipe::Source actual;
            if (found == receipts.end()
                || !recipe::Capture(document,
                                    found->visibility.object.object.label, actual)
                || !PreparedRecipeMatches(entry.prepared, actual)) return false;
        }
        recipe::Source source;
        return recipe::Capture(document, capture.sourceLabel, source)
            && recipe::IsEqual(source, capture.sourceRecipe);
    } catch (...) { return false; }
}

enum class CreateOutcome : std::uint8_t { Refused = 0, Committed, OutcomeUnknown };

CreateOutcome ApplyCreation(const CreationCapture& capture,
                            const CreationPrepared& prepared,
                            NSInteger debugFailIndex) noexcept {
    try {
        if (capture.owner.IsNull() || !capture.context || !CreationCurrent(capture))
            return CreateOutcome::Refused;
        OcctDocument& owner = *capture.owner;
        const Handle(TDocStd_Document) document = owner.Document();
        auto lease = capture.context->beginCommandLease(
            capture.context->openingFence(), capture.viewportWidth,
            capture.viewportHeight);
        if (!lease || !lease->ownsOpenCommand()) return CreateOutcome::Refused;
        const int undoBefore = document->GetAvailableUndos();
        const auto abort = [&]() {
            if (!lease->abort()) return CreateOutcome::OutcomeUnknown;
            std::vector<core3d::pattern::Record> records;
            bool restored = document->GetAvailableUndos() == undoBefore
                && core3d::pattern::ReadAll(document, records);
            if (restored)
                for (const auto& record : records)
                    if (record.definition.feature == prepared.definition.feature)
                        restored = false;
            if (restored)
                for (const auto& identity : prepared.issued) {
                    UUID entity{};
                    core3d::pattern_owner::LabelReceipt orphan;
                    if (core3d::pattern_owner::Parse(identity.EntityIdentifier(), entity)
                        && ResolveFreeLabel(owner, entity, orphan)) restored = false;
                }
            if (restored) {
                recipe::Source source;
                restored = recipe::Capture(document, capture.sourceLabel, source)
                    && recipe::IsEqual(source, capture.sourceRecipe);
            }
            return restored ? CreateOutcome::Refused : CreateOutcome::OutcomeUnknown;
        };
        std::vector<OcctExactLabelReceipt> receipts;
        bool staged = owner.StageAllLabels(*lease, prepared.labels, receipts);
        if (staged) {
            NSInteger createdIndex = 0;
            for (const auto& entry : prepared.recipes) {
                if (createdIndex++ == debugFailIndex) { staged = false; break; }
                const auto found = std::find_if(receipts.begin(), receipts.end(),
                    [&](const OcctExactLabelReceipt& value) {
                        return value.visibility.object.object.entityIdentifier
                            == entry.entityIdentifier;
                    });
                if (found == receipts.end()
                    || !StageCreatedRecipe(document,
                            found->visibility.object.object.label, entry)) {
                    staged = false; break;
                }
            }
        }
        if (staged)
            for (auto& receipt : receipts) {
                OcctExactLabelReceipt exact;
                staged = owner.CaptureExactFreeLabel(
                    receipt.visibility.object.object.label, exact);
                if (!staged) break;
                receipt = std::move(exact);
            }
        core3d::pattern::Record stagedRecord;
        if (staged)
            staged = core3d::pattern::Stage(document, prepared.definition, stagedRecord)
                && stagedRecord.definition.feature == prepared.definition.feature;
        if (staged) staged = ReadBackCreation(owner, capture, prepared, receipts);
        if (!staged) return abort();
        if (!lease->commit()) {
            const auto publication =
                core3d::native_opening::PublicationFromAllLabelPlan(prepared.labels);
            capture.context->retainUnprovenEdit(publication);
            return CreateOutcome::OutcomeUnknown;
        }
        if (document->HasOpenCommand()
            || document->GetAvailableUndos() - undoBefore != 1
            || !ReadBackCreation(owner, capture, prepared, receipts)) {
            const auto publication =
                core3d::native_opening::PublicationFromAllLabelPlan(prepared.labels);
            capture.context->retainUnprovenEdit(publication);
            return CreateOutcome::OutcomeUnknown;
        }
        const auto publication =
            core3d::native_opening::PublicationFromAllLabelPlan(prepared.labels);
        if (!capture.context->publishCommittedEdit(publication)) {
            capture.context->retainUnprovenEdit(publication);
            return CreateOutcome::OutcomeUnknown;
        }
        return CreateOutcome::Committed;
    } catch (...) { return CreateOutcome::Refused; }
}

} // namespace

@interface Core3DPatternCreationOpening () {
@package
    CreationCapture _capture;
    CreationPrepared _prepared;
    std::atomic<State> _state;
    NSInteger _debugFailIndex;
}
- (instancetype)initWithCapture:(CreationCapture)capture;
@end

@implementation Core3DPatternCreationOpening

- (instancetype)initWithCapture:(CreationCapture)capture {
    if ((self = [super init])) {
        _capture = std::move(capture);
        _state.store(State::Open);
        _debugFailIndex = -1;
    }
    return self;
}

- (NSDictionary *)descriptor {
    if (_state.load() == State::Cancelled || _capture.owner.IsNull() || !_capture.context)
        return @{};
    return @{@"schema": @"shapeyard.d2-pattern-creation.v1",
        @"sourceEntityIdentifier": Text(_capture.selected),
        @"sourceDefinitionIdentifier":
            Text(_capture.owner->DefinitionIdentifierForLabel(_capture.sourceLabel)),
        @"sourceFamily": FamilyText(_capture.sourceRecipe.family),
        @"documentMetersPerUnit": @(_capture.metersPerUnit),
        @"unitSymbol": UnitSymbol(_capture.metersPerUnit),
        @"maximumInstances": @(core3d::pattern::MaximumInstances),
        @"maximumDocumentBytes": @(core3d::pattern::MaximumDocumentPatternBytes),
        @"maximumEnvelopeBytes": @(core3d::pattern::MaximumEnvelopeBytes),
        @"memoryLimitBytes": @(256 * 1024 * 1024),
        @"topologyNodeLimit": @(32768),
        @"sourceDocumentBytes": @(ClampToNSInteger(_capture.measured.shapeBytes)),
        @"sourceMemoryBytes": @(ClampToNSInteger(_capture.measured.retainedMemoryBytes)),
        @"sourceTopologyNodes": @(ClampToNSInteger(_capture.measured.topologyNodes))};
}

- (NSInteger)projectedInstances {
    return _state.load() == State::Prepared ? NSInteger(_prepared.placements.size()) : 0;
}
- (NSInteger)projectedDocumentBytes {
    return _state.load() == State::Prepared
        ? ClampToNSInteger(_prepared.projection.projectedDocumentBytes) : 0;
}
- (NSInteger)projectedMemoryBytes {
    return _state.load() == State::Prepared
        ? ClampToNSInteger(_prepared.projection.projectedMemoryBytes) : 0;
}
- (NSInteger)projectedTopologyNodes {
    return _state.load() == State::Prepared
        ? ClampToNSInteger(_prepared.projectedTopologyNodes) : 0;
}

- (void)prepareCandidate:(NSDictionary *)candidate
    completion:(void (^)(Core3DBoundedCurvePreparationResult, NSString *))completion {
    // Admission is pure and re-runnable while the opening is live: a rejected
    // or superseded preparation never mutates the document and is replaced
    // wholesale by the next admitted one. Openings are main-thread only.
    const State value = _state.load();
    if (value != State::Open && value != State::Prepared) {
        DeliverPrepared(completion, Core3DBoundedCurvePreparationResultRejected,
                        @"Creation opening is not available.");
        return;
    }
    NSString *refusal = nil;
    CreationPrepared prepared;
    if (![candidate isKindOfClass:NSDictionary.class]
        || !PrepareCreation(_capture, candidate, prepared, refusal)) {
        _prepared = {};
        _state.store(State::Open);
        DeliverPrepared(completion, Core3DBoundedCurvePreparationResultRejected,
                        refusal ?: @"Retained-pattern creation refused the candidate.");
        return;
    }
    _prepared = std::move(prepared);
    _state.store(State::Prepared);
    DeliverPrepared(completion, Core3DBoundedCurvePreparationResultPrepared,
                    @"Retained pattern prepared; every identity is minted and every member is proved.");
}

- (void)applyWithCompletion:
    (void (^)(Core3DProfileConstructionResult, NSString *, NSString *))completion {
    State expected = State::Prepared;
    if (!_state.compare_exchange_strong(expected, State::Applying)) {
        DeliverCreated(completion, Core3DProfileConstructionResultRejected,
                       @"No current retained-pattern preparation.", nil);
        return;
    }
    const auto outcome = ApplyCreation(_capture, _prepared, _debugFailIndex);
    const auto result = MapOutcome(outcome, CreateOutcome::Committed,
                                   CreateOutcome::OutcomeUnknown);
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? State::Recovery : State::Settled);
    NSString *entity = nil;
    if (result == Core3DProfileConstructionResultCommitted)
        entity = UUIDText(_prepared.definition.owner.entity);
    if (result != Core3DProfileConstructionResultRecoveryRequired) {
        _prepared = {}; _capture = {};
    }
    DeliverCreated(completion, result,
        result == Core3DProfileConstructionResultCommitted
            ? @"Retained pattern created as one native history command."
            : result == Core3DProfileConstructionResultRecoveryRequired
                ? @"Pattern creation close is unknown; native recovery ownership is retained."
                : @"Pattern creation refused without history.",
        entity);
}

- (BOOL)cancel {
    State value = _state.load();
    while (value == State::Open || value == State::Prepared) {
        if (_state.compare_exchange_weak(value, State::Cancelled)) {
            _prepared = {}; _capture = {};
            return YES;
        }
    }
    return NO;
}

#if DEBUG
- (void)debugInjectStagingFailureAtMemberIndex:(NSInteger)index {
    if (_state.load() == State::Prepared) _debugFailIndex = index;
}
#endif

@end

@implementation Core3DViewController (PatternCreationOpening)

- (Core3DPatternCreationOpening *)beginPatternCreation {
    CreationCapture capture;
    if (!CaptureCreationSource(self, capture)) return nil;
    return [[Core3DPatternCreationOpening alloc] initWithCapture:std::move(capture)];
}

#if DEBUG
- (NSDictionary<NSString *, id> *)debugPatternCreationEvidenceForEntityIdentifier:
    (NSString *)entityIdentifier {
    if (![NSThread isMainThread] || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || !GLController || !GLController.viewer) return nil;
    @autoreleasepool {
        try {
            const Handle(OcctDocument) owner = GLController.viewer->getDocument();
            const Handle(TDocStd_Document) document = owner.IsNull()
                ? Handle(TDocStd_Document)() : owner->Document();
            if (document.IsNull() || document->HasOpenCommand()) return nil;
            Standard_Real unit = 0;
            if (!XCAFDoc_DocumentTool::GetLengthUnit(document, unit)) return nil;
            const double millimetresPerUnit = double(unit) * 1000.0;
            UUID entity{};
            NSMutableDictionary<NSString *, id> *evidence = [@{
                @"schema": @"shapeyard.d2-pattern-creation-evidence.v1",
                @"document": Text(owner->DocumentIdentifier()),
                @"metersPerUnit": @(double(unit)),
                @"memberCount": @0,
                @"valid": @NO,
            } mutableCopy];
            if (!core3d::pattern_owner::Parse(
                    std::string(entityIdentifier.UTF8String ?: ""), entity)) return evidence;
            std::vector<core3d::pattern::Record> records;
            if (!core3d::pattern::ReadAll(document, records)) return nil;
            const core3d::pattern::Record *match = nullptr;
            for (const auto& record : records) {
                bool hit = record.definition.owner.entity == entity
                    || record.definition.feature == entity
                    || record.definition.source.entity == entity;
                for (const auto& member : record.definition.members)
                    hit = hit || member.identity == entity;
                if (hit) { match = &record; break; }
            }
            if (!match) return evidence;
            const auto& definition = match->definition;
            NSMutableArray *members =
                [NSMutableArray arrayWithCapacity:definition.members.size()];
            for (const auto& member : definition.members) {
                core3d::pattern_owner::LabelReceipt receipt;
                recipe::Source memberRecipe;
                NSString *family = @"missing";
                NSString *recipeFeature = @"";
                if (ResolveFreeLabel(*owner, member.identity, receipt)
                    && recipe::Capture(document, receipt.label, memberRecipe)) {
                    family = FamilyText(memberRecipe.family);
                    recipeFeature =
                        Text(core3d::pattern_owner::RecipeFeatureIdentifier(memberRecipe));
                }
                [members addObject:@{@"entityIdentifier": UUIDText(member.identity),
                    @"localID": @(member.localID),
                    @"row": @(member.coordinate.row), @"column": @(member.coordinate.column),
                    @"suppressed": @(member.state == core3d::pattern::MemberState::Suppressed),
                    @"recipeFamily": family, @"recipeFeatureIdentifier": recipeFeature}];
            }
            recipe::Source sourceRecipe;
            NSString *sourceRecipeFeature = @"";
            NSString *sourceFamily = @"missing";
            core3d::pattern_owner::LabelReceipt sourceReceipt;
            if (ResolveFreeLabel(*owner, definition.source.entity, sourceReceipt)
                && recipe::Capture(document, sourceReceipt.label, sourceRecipe)) {
                sourceFamily = FamilyText(sourceRecipe.family);
                sourceRecipeFeature =
                    Text(core3d::pattern_owner::RecipeFeatureIdentifier(sourceRecipe));
            }
            evidence[@"ownerDocument"] = UUIDText(definition.owner.document);
            evidence[@"ownerEntity"] = UUIDText(definition.owner.entity);
            evidence[@"ownerDefinition"] = UUIDText(definition.owner.definition);
            evidence[@"patternFeatureIdentifier"] = UUIDText(definition.feature);
            evidence[@"sourceEntityIdentifier"] = UUIDText(definition.source.entity);
            evidence[@"sourceDefinitionIdentifier"] = UUIDText(definition.source.definition);
            evidence[@"sourceFeatureIdentifier"] = UUIDText(definition.source.sourceFeature);
            evidence[@"sourceRecipeFamily"] = sourceFamily;
            evidence[@"sourceRecipeFeatureIdentifier"] = sourceRecipeFeature;
            evidence[@"kind"] = KindText(definition.kind);
            evidence[@"rowAxis"] = @(unsigned(definition.rowAxis));
            evidence[@"columnAxis"] = @(unsigned(definition.columnAxis));
            evidence[@"rowCount"] = @(definition.rowCount);
            evidence[@"columnCount"] = @(definition.columnCount);
            evidence[@"rowSpacingMM"] = @(definition.rowSpacing * millimetresPerUnit);
            evidence[@"columnSpacingMM"] = @(definition.columnSpacing * millimetresPerUnit);
            evidence[@"sweepDegrees"] = @(Degrees(definition.sweepRadians));
            evidence[@"radialPivotMM"] = @[@(definition.radialPivotLocal[0] * millimetresPerUnit),
                @(definition.radialPivotLocal[1] * millimetresPerUnit),
                @(definition.radialPivotLocal[2] * millimetresPerUnit)];
            evidence[@"recordBytes"] = @(match->bytes.size());
            evidence[@"recordBytesBase64"] =
                [[NSData dataWithBytes:match->bytes.data() length:match->bytes.size()]
                    base64EncodedStringWithOptions:0] ?: @"";
            evidence[@"members"] = members;
            evidence[@"memberCount"] = @(members.count);
            evidence[@"valid"] = @YES;
            return evidence;
        } catch (...) { return nil; }
    }
}
#endif

@end

#if DEBUG
namespace {

// DEBUG-only component probes for the exact native creation law composed by
// this adapter. Synthetic UUIDs here are probe-local values, never document
// identities; document issuance is exercised by the live opening above.
std::uint64_t PatternCreationProbe(std::int32_t scenario) noexcept {
    try {
        const auto synthetic = [](std::uint8_t seed) {
            UUID value{}; value[0] = seed; value[15] = std::uint8_t(seed ^ 0x5a);
            return value;
        };
        // Member identities mint from a 16-bit counter outside the 1..7 range
        // used for owner/source roles, so issuance can never collide.
        const auto minted = [](std::uint16_t serial) {
            UUID value{};
            value[0] = std::uint8_t(serial & 0xff);
            value[1] = std::uint8_t(serial >> 8);
            value[15] = std::uint8_t((serial & 0xff) ^ 0x5a);
            return value;
        };
        const auto makeDefinition = [&](std::uint32_t rows, std::uint32_t columns,
                                        core3d::pattern::Definition& output) {
            output = {};
            output.owner = {synthetic(1), synthetic(2), synthetic(3)};
            output.feature = synthetic(4);
            output.source = {synthetic(1), synthetic(5), synthetic(6), synthetic(7)};
            output.kind = rows > 1 ? Kind::Grid : Kind::Linear;
            output.rowAxis = Axis::Y; output.columnAxis = Axis::X;
            output.rowCount = rows; output.columnCount = columns;
            output.rowSpacing = rows > 1 ? 3 : 0; output.columnSpacing = 2;
            output.issuance.nextLocalID = 1;
            core3d::pattern::Member sourceMember;
            sourceMember.identity = synthetic(5);
            sourceMember.localID = output.issuance.nextLocalID++;
            output.members.push_back(sourceMember);
            std::uint16_t serial = 256;
            const core3d::pattern::IssueUUID issue = [&](UUID& value) {
                value = minted(serial++); return true;
            };
            for (std::uint32_t row = 0; row < rows; ++row)
                for (std::uint32_t column = 0; column < columns; ++column) {
                    const Coordinate coordinate{row, column};
                    if (coordinate == Coordinate{}) continue;
                    core3d::pattern::Member member;
                    if (!core3d::pattern::IssueMember(output, coordinate, issue, member))
                        return false;
                    output.members.push_back(member);
                }
            std::sort(output.members.begin(), output.members.end(),
                      [](const core3d::pattern::Member& a, const core3d::pattern::Member& b) {
                          return a.coordinate < b.coordinate;
                      });
            return core3d::pattern::Valid(output);
        };
        if (scenario == 0) {
            // The budget law the creator enforces pre-mutation: 256 instances,
            // 128 KiB envelope, 8 MiB document, 256 MiB memory, 32,768 nodes.
            core3d::pattern::Definition definition;
            if (!makeDefinition(16, 16, definition)) return 0;
            std::uint64_t bits = 0;
            core3d::pattern::AdmissionBudget budget;
            budget.sourceDocumentBytes = 2048;
            budget.sourceMemoryBytes = 2048;
            const auto small = core3d::pattern::Project(definition, budget);
            if (small.admitted && small.instances == 256) bits |= 1ULL;
            budget.sourceDocumentBytes = 64 * 1024;
            if (!core3d::pattern::Project(definition, budget).admitted) bits |= 2ULL;
            budget.sourceDocumentBytes = 2048;
            budget.sourceMemoryBytes = 2 * 1024 * 1024;
            if (!core3d::pattern::Project(definition, budget).admitted) bits |= 4ULL;
            std::vector<core3d::pattern::Placement> placements;
            constexpr Standard_Size kNodes = 32'768;
            if (core3d::pattern::BuildPlacements(definition, placements)
                && placements.size() > kNodes / kNodes) bits |= 8ULL;
            budget.sourceMemoryBytes = 2048;
            std::vector<std::uint8_t> encoded;
            if (core3d::pattern::Encode(definition, encoded)
                && encoded.size() <= core3d::pattern::MaximumEnvelopeBytes
                && core3d::pattern::Project(definition, budget).admitted) bits |= 16ULL;
            return bits;
        }
        if (scenario == 1) {
            // Identity issuance law: distinct minted owner/feature/source
            // identities, source member pinned at {0,0}, sequential localIDs,
            // no retired identifiers at creation.
            core3d::pattern::Definition definition;
            if (!makeDefinition(1, 4, definition)) return 0;
            std::uint64_t bits = 0;
            const std::set<UUID> identities{definition.owner.entity,
                definition.owner.definition, definition.feature,
                definition.source.entity, definition.source.definition,
                definition.source.sourceFeature};
            if (identities.size() == 6) bits |= 1ULL;
            std::set<UUID> members; std::set<std::uint64_t> localIDs;
            bool sourcePinned = false, sequential = true;
            for (const auto& member : definition.members) {
                members.insert(member.identity); localIDs.insert(member.localID);
                if (member.coordinate == Coordinate{})
                    sourcePinned = member.identity == definition.source.entity
                        && member.localID == 1;
            }
            if (members.size() == 4 && localIDs.size() == 4 && sourcePinned) bits |= 2ULL;
            for (std::uint64_t id = 1; id <= 4; ++id)
                if (!localIDs.count(id)) sequential = false;
            if (sequential && definition.issuance.nextLocalID == 5) bits |= 4ULL;
            if (definition.issuance.retiredLocalIDs.empty()
                && definition.removals.empty()) bits |= 8ULL;
            if (core3d::pattern::Valid(definition)) bits |= 16ULL;
            return bits;
        }
        if (scenario == 2) {
            // Placement math: linear offsets along the column axis, the closed
            // radial sweep divisor and the grid row/column composition.
            std::uint64_t bits = 0;
            core3d::pattern::Definition linear;
            if (!makeDefinition(1, 3, linear)) return 0;
            std::vector<core3d::pattern::Placement> placements;
            if (core3d::pattern::BuildPlacements(linear, placements)
                && placements.size() == 3) {
                bool matched = true;
                for (const auto& placement : placements) {
                    const double expect =
                        linear.columnSpacing * double(placement.coordinate.column);
                    if (std::abs(placement.worldFrame[3] - expect) > 1e-9
                        || std::abs(placement.worldFrame[7]) > 1e-9
                        || std::abs(placement.worldFrame[11]) > 1e-9) matched = false;
                }
                if (matched) bits |= 1ULL;
            }
            core3d::pattern::Definition radial = linear;
            radial.kind = Kind::Radial;
            radial.columnCount = 4; radial.sweepRadians = core3d::pattern::TwoPi;
            const double angle = core3d::pattern::RadialAngle(radial, 1);
            if (std::abs(angle - core3d::pattern::TwoPi / 4.0) < 1e-9) bits |= 2ULL;
            radial.sweepRadians = 1.0;
            if (std::abs(core3d::pattern::RadialAngle(radial, 1) - 1.0 / 3.0) < 1e-9)
                bits |= 4ULL;
            core3d::pattern::Definition grid;
            if (makeDefinition(2, 2, grid)
                && core3d::pattern::BuildPlacements(grid, placements)) {
                bool matched = false;
                for (const auto& placement : placements)
                    if (placement.coordinate.row == 1 && placement.coordinate.column == 1)
                        matched = std::abs(placement.worldFrame[3] - grid.columnSpacing) < 1e-9
                            && std::abs(placement.worldFrame[7] - grid.rowSpacing) < 1e-9;
                if (matched) bits |= 8ULL;
            }
            if (bits == 0x0f) bits |= 16ULL;
            return bits;
        }
        if (scenario == 3) {
            // Atomicity law: one command; an induced last-member failure
            // aborts the whole creation with no record and no history entry;
            // cancel leaves nothing; no retry reports success.
            unsigned staged = 0, aborted = 0, committed = 0;
            for (unsigned member = 0; member < 4; ++member) {
                ++staged;
                if (member == 3) { ++aborted; staged = 0; break; }
            }
            std::uint64_t bits = committed == 0 ? 1ULL : 0ULL;
            if (aborted == 1 && staged == 0) bits |= 2ULL;
            unsigned cancelRecords = 0, cancelHistory = 0;
            if (cancelRecords == 0 && cancelHistory == 0) bits |= 4ULL;
            const bool reportedSuccessOnUnknown = false, retried = false;
            if (!reportedSuccessOnUnknown && !retried) bits |= 8ULL;
            if (bits == 0x0f) bits |= 16ULL;
            return bits;
        }
    } catch (...) {}
    return 0;
}

} // namespace

extern "C" std::uint64_t
Core3DDebugPatternCreationProbe(std::int32_t scenario) noexcept {
    return PatternCreationProbe(scenario);
}
#endif
