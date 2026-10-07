//
//  Core3DPathArrayCreationOpening.mm
//  Core3D
//
//  Production D3 record-free candidate host. This file composes public C1,
//  exact-label, recipe-clone, path-array and native command APIs. It does not
//  synthesize an existing-owner Snapshot or call private D3 collaborators.
//

#define CORE3D_PATH_ARRAY_NATIVE_SUPPORT 1
#import "Core3DPathArrayCreationOpening.h"
#undef CORE3D_PATH_ARRAY_NATIVE_SUPPORT
#import "GLViewController+Trick.h"
#import "Core3DViewer.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/BoundedCurveOwner.hxx"
#include "../OCCTKit/PathArrayBuild.hxx"
#include "../OCCTKit/PathArrayPersistence.hxx"
#include "../OCCTKit/PatternOwnerBridge.hxx"

#include <BRepBuilderAPI_Transform.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#if DEBUG
namespace core3d::path_array_owner {
NSDictionary<NSString *, id> *DebugNativeMutationEvidence(
    const std::string& sourceEntity) noexcept;
}
#endif

namespace {
namespace array = core3d::path_array;
namespace curve = core3d::bounded_curve;
namespace curve_owner = core3d::bounded_curve::owner;
namespace recipe = core3d::pattern_recipe_clone;
using UUID = core3d::retained_recipe::UUID;

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double Radians(double degrees) noexcept { return degrees * kPi / 180.0; }
constexpr double Degrees(double radians) noexcept { return radians * 180.0 / kPi; }
enum class State : std::uint8_t { Open, Prepared, Applying, Cancelled, Settled, Recovery };

NSString *Text(const std::string& value) {
    return [[NSString alloc] initWithBytes:value.data() length:value.size()
                                  encoding:NSUTF8StringEncoding] ?: @"";
}

NSString *UUIDText(const UUID& value) {
    try { return Text(core3d::retained_solid::UUIDText(value)); }
    catch (...) { return @""; }
}

bool ExactKeys(NSDictionary *value, NSArray<NSString *> *keys) {
    if (![value isKindOfClass:NSDictionary.class] || value.count != keys.count) return false;
    return [[NSSet setWithArray:value.allKeys] isEqualToSet:[NSSet setWithArray:keys]];
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
    output = converted; return true;
}

bool Boolean(id value, bool& output) {
    if (![value isKindOfClass:NSNumber.class]
        || CFGetTypeID((__bridge CFTypeRef)value) != CFBooleanGetTypeID()) return false;
    output = [value boolValue]; return true;
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

bool MintIdentity(UUID& output) {
    output = {};
    for (unsigned attempt = 0; attempt < 16; ++attempt) {
        const std::string text = OcctDocument::NewProfileIdentifier();
        if (!text.empty() && core3d::pattern_owner::Parse(text, output)) return true;
    }
    return false;
}

NSString *BuildRefusalCode(array::BuildRefusal value) {
    switch (value) {
        case array::BuildRefusal::None: return @"none";
        case array::BuildRefusal::InvalidDefinition: return @"invalidDefinition";
        case array::BuildRefusal::StalePath: return @"stalePath";
        case array::BuildRefusal::ClosedStateMismatch: return @"closedStateMismatch";
        case array::BuildRefusal::ArcLengthFailure: return @"arcLengthFailure";
        case array::BuildRefusal::TooFewInstances: return @"tooFewInstances";
        case array::BuildRefusal::TooManyInstances: return @"tooManyInstances";
        case array::BuildRefusal::ZeroTangent: return @"zeroTangent";
        case array::BuildRefusal::Cusp: return @"cusp";
        case array::BuildRefusal::FrameFlip: return @"frameFlip";
        case array::BuildRefusal::InvalidUpVector: return @"invalidUpVector";
        case array::BuildRefusal::InversionFailure: return @"inversionFailure";
    }
}

std::size_t RecipeBytes(const recipe::Source& source) noexcept {
    try {
        if (source.family == recipe::Family::Sweep)
            return source.sweep.values.size() * sizeof(double);
        if (source.family == recipe::Family::Loft)
            return source.loft.values.size() * sizeof(double);
        if (source.family == recipe::Family::AnalyticBoolean && source.analyticBoolean.value)
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
        return core3d::pattern::CheckedAdd(output.recipeBytes,
                    output.retainedMemoryBytes, output.retainedMemoryBytes)
            && core3d::pattern::CheckedAdd(output.shapeBytes,
                    output.retainedMemoryBytes, output.retainedMemoryBytes);
    } catch (...) { output = {}; return false; }
}

bool TransformMatrix(const gp_Trsf& transform, array::Matrix& output) noexcept {
    try {
        if (!recipe::IsProperRigidPlacement(transform)) return false;
        output = {{transform.Value(1,1), transform.Value(1,2), transform.Value(1,3), transform.Value(1,4),
                   transform.Value(2,1), transform.Value(2,2), transform.Value(2,3), transform.Value(2,4),
                   transform.Value(3,1), transform.Value(3,2), transform.Value(3,3), transform.Value(3,4),
                   0, 0, 0, 1}};
        for (double scalar : output) if (!std::isfinite(scalar)) return false;
        return true;
    } catch (...) { return false; }
}

bool MatrixTransform(const array::Matrix& value, gp_Trsf& output) noexcept {
    try {
        output.SetValues(value[0], value[1], value[2], value[3],
                         value[4], value[5], value[6], value[7],
                         value[8], value[9], value[10], value[11]);
        return recipe::IsProperRigidPlacement(output);
    } catch (...) { return false; }
}

struct CreationCapture {
    Handle(OcctDocument) owner;
    std::shared_ptr<core3d::native_opening::Context> context;
    std::uint32_t viewportWidth = 0, viewportHeight = 0;
    std::string selected, pathEntity;
    TDF_Label sourceLabel;
    TopoDS_Shape sourceShape;
    recipe::Source sourceRecipe;
    OcctExactLabelReceipt sourceReceipt;
    core3d::pattern_owner::AllLabelMeasuredCost measured;
    std::size_t patternDocumentBytes = 0;
    std::size_t pathArrayDocumentBytes = 0;
    std::size_t compositeDocumentBytes = 0;
    OcctBoundedCurveCapture pathExact;
    std::shared_ptr<const curve_owner::PathReceipt> pathReceipt;
    Standard_Integer documentTime = -1;
    double metersPerUnit = 0;
    bool pathClosed = false;
};

bool ResolveFreeLabel(OcctDocument& owner, const UUID& entity,
                      core3d::pattern_owner::LabelReceipt& output) noexcept {
    output = {};
    try {
        const Handle(TDocStd_Document) document = owner.Document();
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return false;
        TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
        core3d::pattern_owner::LabelReceipt found; bool matched = false;
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

bool CaptureTables(CreationCapture& value) noexcept {
    value.patternDocumentBytes = value.pathArrayDocumentBytes = value.compositeDocumentBytes = 0;
    try {
        const Handle(TDocStd_Document) document = value.owner->Document();
        UUID selected{};
        if (!core3d::pattern_owner::Parse(value.selected, selected)) return false;
        std::vector<core3d::pattern::Record> patterns;
        std::vector<array::Record> arrays;
        if (!core3d::pattern::ReadAll(document, patterns)
            || !array::ReadAll(document, arrays)) return false;
        for (const auto& record : patterns) {
            value.patternDocumentBytes += record.bytes.size();
            if (record.definition.source.entity == selected) return false;
            for (const auto& member : record.definition.members)
                if (member.identity == selected) return false;
        }
        for (const auto& record : arrays) {
            value.pathArrayDocumentBytes += record.bytes.size();
            if (record.definition.source.entity == selected) return false;
            for (const auto& member : record.definition.members)
                if (member.identity == selected) return false;
        }
        std::vector<core3d::composite_recipe::Record> composites;
        if (!core3d::composite_recipe::ReadAll(document, composites,
                value.patternDocumentBytes + value.pathArrayDocumentBytes)) return false;
        for (const auto& record : composites)
            value.compositeDocumentBytes += record.value->bytes.size();
        return true;
    } catch (...) { return false; }
}

curve_owner::SceneFence SceneFor(const CreationCapture& value) noexcept {
    const auto& fence = value.context->openingFence();
    return {fence.documentGeneration(), fence.modelRevision(), fence.metersPerUnit()};
}

bool PathReceiptMatches(const curve_owner::PathReceipt& receipt,
                        const OcctBoundedCurveCapture& exact) noexcept {
    try {
        const auto& persisted = exact.persisted;
        return curve_owner::ValidPathReceipt(receipt)
            && curve::ValidatePersisted(persisted)
            && persisted.value.definition.domain == curve::Domain::Path3D
            && receipt.owner == persisted.ownerState.owner
            && receipt.feature == persisted.value.feature
            && receipt.feature == persisted.ownerState.feature
            && receipt.definitionRevision == persisted.ownerState.definitionRevision
            && receipt.frame == persisted.value.definition.frame.identifier
            && receipt.frameRevision == persisted.value.definition.frame.revision
            && receipt.nextLocalID == persisted.ownerState.nextLocalID
            && receipt.canonicalDefinitionDigest == persisted.ownerState.canonicalDefinitionDigest
            && exact.ownerReceipt.visibility.object.object.label.IsEqual(exact.record.owner)
            && !exact.record.label.IsNull();
    } catch (...) { return false; }
}

bool CapturePath(CreationCapture& value, const std::string& entity) noexcept {
    try {
        if (value.owner.IsNull() || !value.context || entity.empty()
            || entity.size() > 128) return false;
        curve_owner::OcafOwner picker(*value.owner, value.context);
        auto receipt = picker.pickCurrentPath3D(entity, SceneFor(value));
        OcctBoundedCurveCapture exact;
        if (!receipt || !value.owner->CaptureBoundedCurveExact(entity, *value.context, exact)
            || exact.documentData != value.owner->Document()->GetData()
            || exact.documentIdentifier != value.owner->DocumentIdentifier()
            || !PathReceiptMatches(*receipt, exact)) return false;
        const array::Definition defaults;
        bool closed = false;
        if (!array::DetectClosed(exact.persisted.value.definition,
                defaults.arcLengthTolerance, defaults.minimumTangent, closed)) return false;
        value.pathEntity = entity;
        value.pathReceipt = std::move(receipt);
        value.pathExact = std::move(exact);
        value.pathClosed = closed;
        return true;
    } catch (...) { return false; }
}

NSDictionary<NSString *, id> *CreationDescriptor(const CreationCapture& capture) {
    const array::Definition defaults;
    const auto& distributionLaw = defaults.distribution;
    const auto& orientationLaw = defaults.orientation;
    NSString *distribution = distributionLaw.mode == array::DistributionMode::Count
        ? @"count" : @"distance";
    NSString *orientation = orientationLaw.policy == array::OrientationPolicy::Fixed
        ? @"fixed" : orientationLaw.policy == array::OrientationPolicy::Tangent
            ? @"tangent" : @"bishop";
    return @{@"sourceEntityIdentifier": Text(capture.selected),
        @"pathEntityIdentifier": Text(capture.pathEntity),
        @"documentMetersPerUnit": @(capture.metersPerUnit),
        @"closedPath": @(capture.pathClosed), @"distribution": distribution,
        @"count": @(distributionLaw.count),
        @"distanceInDocumentUnits": @(distributionLaw.distance),
        @"includeStart": @(distributionLaw.includeStart),
        @"includeEnd": @(distributionLaw.includeEnd), @"orientation": orientation,
        @"rollDegrees": @(Degrees(orientationLaw.rollRadians)),
        @"hasUpVector": @(orientationLaw.hasUpVector),
        @"upVector": @[@(orientationLaw.upVector[0]), @(orientationLaw.upVector[1]),
                       @(orientationLaw.upVector[2])],
        @"maximumFrameStepDegrees": @(Degrees(orientationLaw.maximumFrameStepRadians)),
        @"arcToleranceInDocumentUnits": @(defaults.arcLengthTolerance),
        @"minimumTangentInDocumentUnits": @(defaults.minimumTangent)};
}

bool CaptureCreationSource(Core3DViewController *controller, NSString *pathIdentifier,
                           CreationCapture& output) noexcept {
    output = {};
    try {
        if (!NSThread.isMainThread || !controller
            || ![pathIdentifier isKindOfClass:NSString.class]
            || pathIdentifier.length == 0 || pathIdentifier.length > 128) return false;
        Core3DSceneSnapshot *scene = [controller captureSceneSnapshot];
        if (!scene || scene.selection.selectedElements.count != 1) return false;
        Core3DSceneElementIdentifier *element = scene.selection.selectedElements.firstObject;
        if (element.kind != Core3DSceneElementKindObject
            || element.entityIdentifier.length == 0
            || element.entityIdentifier.length > 128) return false;
        GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)controller.glController : nil;
        const CGSize drawable = controller.viewportDrawableSize;
        if (!gl || !gl.viewer || !std::isfinite(drawable.width)
            || !std::isfinite(drawable.height) || drawable.width < 1 || drawable.height < 1
            || drawable.width > UINT32_MAX || drawable.height > UINT32_MAX) return false;
        CreationCapture value;
        value.owner = gl.viewer->getDocument();
        value.selected = element.entityIdentifier.UTF8String ?: "";
        value.viewportWidth = std::uint32_t(drawable.width);
        value.viewportHeight = std::uint32_t(drawable.height);
        value.context = gl.viewer->captureNativeOpeningContext(
            value.viewportWidth, value.viewportHeight, {value.selected});
        if (value.owner.IsNull() || !value.context || value.selected.empty()) return false;
        const Handle(TDocStd_Document) document = value.owner->Document();
        if (document.IsNull() || document->GetData().IsNull() || document->HasOpenCommand()
            || value.context->openingFence().document() != document
            || value.context->openingFence().data() != document->GetData()
            || !value.context->isCurrent(value.viewportWidth, value.viewportHeight)) return false;
        value.metersPerUnit = value.context->openingFence().metersPerUnit();
        if (!std::isfinite(value.metersPerUnit) || value.metersPerUnit <= 0) return false;
        UUID selected{};
        if (!core3d::pattern_owner::Parse(value.selected, selected)) return false;
        core3d::pattern_owner::LabelReceipt source;
        if (!ResolveFreeLabel(*value.owner, selected, source)) return false;
        value.sourceLabel = source.label; value.sourceShape = source.shape;
        if (!CaptureTables(value)
            || !recipe::Capture(document, value.sourceLabel, value.sourceRecipe)
            || !value.owner->CaptureExactFreeLabel(value.sourceLabel, value.sourceReceipt)
            || !MeasureSource(value.sourceShape, RecipeBytes(value.sourceRecipe), value.measured)
            || !CapturePath(value, pathIdentifier.UTF8String ?: "")) return false;
        value.documentTime = document->GetData()->Time();
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

bool RefreshCapture(const CreationCapture& frozen, CreationCapture& output) noexcept {
    output = frozen;
    try {
        if (frozen.owner.IsNull() || !frozen.context) return false;
        const Handle(TDocStd_Document) document = frozen.owner->Document();
        if (document.IsNull() || document->HasOpenCommand()
            || document->GetData()->Time() != frozen.documentTime
            || !frozen.context->isCurrent(frozen.viewportWidth, frozen.viewportHeight)) return false;
        UUID selected{};
        if (!core3d::pattern_owner::Parse(frozen.selected, selected)) return false;
        core3d::pattern_owner::LabelReceipt source;
        if (!ResolveFreeLabel(*frozen.owner, selected, source)
            || !source.label.IsEqual(frozen.sourceLabel)) return false;
        CreationCapture fresh = frozen;
        fresh.sourceLabel = source.label; fresh.sourceShape = source.shape;
        fresh.sourceRecipe = {}; fresh.sourceReceipt = {}; fresh.measured = {};
        fresh.pathExact = {}; fresh.pathReceipt.reset();
        if (!CaptureTables(fresh)
            || !recipe::Capture(document, fresh.sourceLabel, fresh.sourceRecipe)
            || !fresh.owner->CaptureExactFreeLabel(fresh.sourceLabel, fresh.sourceReceipt)
            || !MeasureSource(fresh.sourceShape, RecipeBytes(fresh.sourceRecipe), fresh.measured)
            || !CapturePath(fresh, frozen.pathEntity)
            || !fresh.sourceReceipt.IsEqual(frozen.sourceReceipt)
            || !recipe::IsEqual(fresh.sourceRecipe, frozen.sourceRecipe)
            || !fresh.pathExact.IsEqual(frozen.pathExact)
            || fresh.patternDocumentBytes != frozen.patternDocumentBytes
            || fresh.pathArrayDocumentBytes != frozen.pathArrayDocumentBytes
            || fresh.compositeDocumentBytes != frozen.compositeDocumentBytes
            || !core3d::pattern_owner::SameMeasuredCost(fresh.measured, frozen.measured)) return false;
        output = std::move(fresh); return true;
    } catch (...) { output = {}; return false; }
}

struct CreationRecipe {
    std::string entityIdentifier;
    std::string featureIdentifier;
    recipe::Prepared prepared;
};

struct CreationPrepared {
    array::Definition definition;
    array::Projection projection;
    std::uint32_t required = 0;
    double totalLength = 0;
    std::vector<array::Placement> placements;
    array::BuildReceipt build;
    std::vector<OcctIssuedLabelIdentity> issued;
    OcctAllLabelPlan labels;
    std::vector<CreationRecipe> recipes;
    std::vector<std::uint8_t> canonicalBytes;
};

bool ParseCandidate(NSDictionary *value, array::Definition& output) {
    if (!ExactKeys(value, @[@"distribution", @"count", @"distanceInDocumentUnits",
            @"includeStart", @"includeEnd", @"closedPath", @"orientation",
            @"rollDegrees", @"hasUpVector", @"upVector"])) return false;
    NSString *distribution = nil, *orientation = nil;
    std::uint64_t count = 0; double roll = 0;
    if (!String(value[@"distribution"], distribution)
        || !String(value[@"orientation"], orientation)
        || !Integer(value[@"count"], UINT32_MAX, count)
        || !Number(value[@"distanceInDocumentUnits"], output.distribution.distance)
        || !Boolean(value[@"includeStart"], output.distribution.includeStart)
        || !Boolean(value[@"includeEnd"], output.distribution.includeEnd)
        || !Boolean(value[@"closedPath"], output.closedPath)
        || !Number(value[@"rollDegrees"], roll)
        || !Boolean(value[@"hasUpVector"], output.orientation.hasUpVector)
        || !Vector3(value[@"upVector"], output.orientation.upVector)) return false;
    if ([distribution isEqualToString:@"count"])
        output.distribution.mode = array::DistributionMode::Count;
    else if ([distribution isEqualToString:@"distance"])
        output.distribution.mode = array::DistributionMode::Distance;
    else return false;
    if ([orientation isEqualToString:@"fixed"])
        output.orientation.policy = array::OrientationPolicy::Fixed;
    else if ([orientation isEqualToString:@"tangent"])
        output.orientation.policy = array::OrientationPolicy::Tangent;
    else if ([orientation isEqualToString:@"bishop"])
        output.orientation.policy = array::OrientationPolicy::Bishop;
    else return false;
    output.distribution.count = std::uint32_t(count);
    output.orientation.rollRadians = Radians(roll);
    return true;
}

bool DetachedMemberShape(const CreationCapture& capture,
                         const array::Matrix& sourceFrameValue,
                         const array::Placement& placement,
                         TopoDS_Shape& output, gp_Trsf& transform) noexcept {
    output.Nullify();
    try {
        gp_Trsf sourceFrame, occurrence;
        if (!MatrixTransform(placement.occurrenceFrame, occurrence)
            || !MatrixTransform(sourceFrameValue, sourceFrame)) return false;
        transform = occurrence * sourceFrame.Inverted();
        if (!recipe::IsProperRigidPlacement(transform)) return false;
        BRepBuilderAPI_Transform copied(capture.sourceShape, transform, Standard_True);
        if (!copied.IsDone() || copied.Shape().IsNull()
            || copied.Shape().IsPartner(capture.sourceShape)) return false;
        output = copied.Shape(); return true;
    } catch (...) { output.Nullify(); return false; }
}

bool BuildPreparedPlan(const CreationCapture& capture,
                       const array::Definition& frozenDefinition,
                       const std::vector<OcctIssuedLabelIdentity>& issued,
                       const std::vector<std::string>& recipeFeatures,
                       CreationPrepared& output,
                       NSString *__strong& phase,
                       NSString *__strong& code) {
    output = {}; phase = @"count"; code = @"invalidDefinition";
    try {
        array::Definition definition = frozenDefinition;
        std::uint32_t required = 0; double total = 0;
        const auto countResult = array::RequiredInstanceCount(
            definition, capture.pathExact.persisted, required, total);
        if (countResult != array::BuildRefusal::None) {
            code = BuildRefusalCode(countResult); return false;
        }
        output.required = required; output.totalLength = total;
        if (definition.members.size() != required || issued.size() != required - 1
            || recipeFeatures.size() != required - 1) return false;

        phase = @"project"; code = @"budget";
        array::AdmissionBudget budget;
        budget.maximumInstances = array::MaximumInstances;
        budget.sourceTopologyNodes = capture.measured.topologyNodes;
        budget.existingTopologyNodes = 0;
        budget.maximumAggregateTopologyNodes = 2'000'000;
        budget.sourceDocumentBytes = capture.measured.shapeBytes;
        budget.existingDocumentBytes = capture.patternDocumentBytes
            + capture.pathArrayDocumentBytes + capture.compositeDocumentBytes;
        budget.maximumDocumentBytes = array::MaximumDocumentBytes;
        budget.sourceMemoryBytes = capture.measured.retainedMemoryBytes;
        budget.existingMemoryBytes = 0;
        budget.maximumMemoryBytes = 256 * 1024 * 1024;
        output.projection = array::Project(required, budget);
        if (!output.projection.admitted) {
            code = output.projection.aggregateTopologyNodes
                    > budget.maximumAggregateTopologyNodes ? @"topologyBudget"
                : output.projection.projectedDocumentBytes
                    > budget.maximumDocumentBytes ? @"documentBudget" : @"memoryBudget";
            return false;
        }

        phase = @"build"; code = @"arcLengthFailure";
        const auto buildResult = array::BuildPlacements(definition,
            capture.pathExact.persisted, output.placements, output.build);
        if (buildResult != array::BuildRefusal::None) {
            code = BuildRefusalCode(buildResult); return false;
        }
        if (output.placements.size() != required
            || output.build.requestedInstances != required
            || output.build.emittedInstances != required) return false;

        phase = @"recipePlan"; code = @"recipeCloneRefused";
        output.issued = issued;
        std::set<std::string> occupiedFeatures;
        const std::string sourceFeature =
            core3d::pattern_owner::RecipeFeatureIdentifier(capture.sourceRecipe);
        if (!sourceFeature.empty()) occupiedFeatures.insert(sourceFeature);
        for (std::size_t ordinal = 1; ordinal < definition.members.size(); ++ordinal) {
            const auto& member = definition.members[ordinal];
            if (issued[ordinal - 1].EntityIdentifier()
                    != core3d::retained_solid::UUIDText(member.identity)) return false;
            const std::string& feature = recipeFeatures[ordinal - 1];
            if (capture.sourceRecipe.family == recipe::Family::None) {
                if (!feature.empty()) return false;
            } else if (feature.empty() || !occupiedFeatures.insert(feature).second) {
                return false;
            }
            TopoDS_Shape shape; gp_Trsf transform;
            if (!DetachedMemberShape(capture, definition.sourceFrame,
                    output.placements[ordinal], shape, transform)) return false;
            OcctPreparedLabelClone clone;
            clone.source = capture.sourceReceipt;
            clone.detachedShape = shape;
            clone.representation = capture.sourceReceipt.visibility.object.object
                .resolvedRepresentation;
            CreationRecipe entry;
            entry.entityIdentifier = issued[ordinal - 1].EntityIdentifier();
            entry.featureIdentifier = feature;
            const std::optional<gp_Trsf> baked = transform;
            if (!recipe::Prepare(capture.sourceRecipe, shape, feature,
                                 baked, entry.prepared)) return false;
            output.labels.creates.push_back({issued[ordinal - 1], clone});
            output.recipes.push_back(std::move(entry));
        }
        phase = @"encode"; code = @"invalidDefinition";
        if (!array::Encode(definition, output.canonicalBytes)) return false;
        output.definition = std::move(definition);
        phase = @"admitted"; code = @"none";
        return true;
    } catch (...) { output = {}; phase = @"exception"; code = @"nativeFailure"; return false; }
}

bool PrepareInitial(const CreationCapture& capture, NSDictionary *candidate,
                    CreationPrepared& output, NSString *__strong& phase,
                    NSString *__strong& code) {
    output = {}; phase = @"parse"; code = @"malformedCandidate";
    try {
        CreationCapture current;
        if (!RefreshCapture(capture, current)) {
            phase = @"capture"; code = @"staleCapture"; return false;
        }
        array::Definition definition;
        if (!ParseCandidate(candidate, definition)) return false;
        UUID documentID{}, ownerEntity{}, ownerDefinition{}, feature{}, sourceEntity{},
            sourceDefinition{}, sourceFeature{};
        if (!core3d::pattern_owner::Parse(current.owner->DocumentIdentifier(), documentID)
            || !core3d::pattern_owner::Parse(current.selected, sourceEntity)
            || !core3d::pattern_owner::Parse(
                current.owner->DefinitionIdentifierForLabel(current.sourceLabel), sourceDefinition)
            || !MintIdentity(ownerEntity) || !MintIdentity(ownerDefinition)
            || !MintIdentity(feature) || !MintIdentity(sourceFeature)) {
            phase = @"issuance"; code = @"metadataIdentityRefused"; return false;
        }
        const std::set<UUID> metadata{documentID, ownerEntity, ownerDefinition,
            feature, sourceEntity, sourceDefinition, sourceFeature};
        if (metadata.size() != 7) {
            phase = @"issuance"; code = @"identityCollision"; return false;
        }
        definition.owner = {documentID, ownerEntity, ownerDefinition};
        definition.feature = feature;
        definition.source = {documentID, sourceEntity, sourceDefinition, sourceFeature};
        definition.path.owner = current.pathExact.persisted.ownerState.owner;
        definition.path.feature = current.pathExact.persisted.value.feature;
        definition.path.definitionRevision =
            current.pathExact.persisted.ownerState.definitionRevision;
        definition.path.canonicalDefinitionDigest =
            current.pathExact.persisted.ownerState.canonicalDefinitionDigest;
        if (!TransformMatrix(current.sourceReceipt.visibility.object.object.transform,
                             definition.sourceFrame)) {
            phase = @"capture"; code = @"sourceFrameRefused"; return false;
        }
        std::uint32_t required = 0; double length = 0;
        const auto countResult = array::RequiredInstanceCount(
            definition, current.pathExact.persisted, required, length);
        if (countResult != array::BuildRefusal::None) {
            phase = @"count"; code = BuildRefusalCode(countResult); return false;
        }
        phase = @"issuance"; code = @"memberIdentityRefused";
        std::vector<OcctIssuedLabelIdentity> issued;
        if (!current.owner->ReserveExactLabelIdentities(
                Standard_Size(required - 1), {}, issued)
            || issued.size() != required - 1) return false;
        definition.issuance.nextLocalID = 1;
        std::size_t nextIssued = 0;
        const core3d::pattern::IssueUUID issue = [&](UUID& identity) {
            return nextIssued < issued.size()
                && core3d::pattern_owner::Parse(
                    issued[nextIssued++].EntityIdentifier(), identity);
        };
        if (!array::ReconcileMembers(definition, required, issue)
            || nextIssued != issued.size()) return false;
        std::vector<std::string> recipeFeatures(required - 1);
        if (current.sourceRecipe.family != recipe::Family::None) {
            std::set<std::string> occupied;
            const std::string sourceID =
                core3d::pattern_owner::RecipeFeatureIdentifier(current.sourceRecipe);
            if (!sourceID.empty()) occupied.insert(sourceID);
            for (auto& item : recipeFeatures) {
                for (unsigned attempt = 0; attempt < 16 && item.empty(); ++attempt) {
                    std::string minted = OcctDocument::NewProfileIdentifier();
                    if (!minted.empty() && occupied.insert(minted).second)
                        item = std::move(minted);
                }
                if (item.empty()) {
                    code = @"recipeIdentityRefused"; return false;
                }
            }
        }
        return BuildPreparedPlan(current, definition, issued, recipeFeatures,
                                 output, phase, code);
    } catch (...) { output = {}; phase = @"exception"; code = @"nativeFailure"; return false; }
}

bool SameProjection(const array::Projection& a, const array::Projection& b) noexcept {
    return a.instances == b.instances
        && a.aggregateTopologyNodes == b.aggregateTopologyNodes
        && a.projectedDocumentBytes == b.projectedDocumentBytes
        && a.projectedMemoryBytes == b.projectedMemoryBytes
        && a.admitted == b.admitted;
}

bool SamePreparedRecipe(const recipe::Prepared& a,
                        const recipe::Prepared& b) noexcept {
    try {
        if (a.family != b.family || a.featureIdentifier != b.featureIdentifier
            || !recipe::ShapeBytesEqual(a.binding, b.binding)) return false;
        if (a.family == recipe::Family::None) return true;
        if (a.family == recipe::Family::Sweep) {
            std::vector<double> x, y;
            return core3d::sweep_persistence::Encode(a.sweep, x)
                && core3d::sweep_persistence::Encode(b.sweep, y) && x == y;
        }
        if (a.family == recipe::Family::Loft) {
            std::vector<double> x, y;
            return core3d::loft_persistence::Encode(a.loft, x)
                && core3d::loft_persistence::Encode(b.loft, y) && x == y;
        }
        return a.family == recipe::Family::AnalyticBoolean
            && a.analyticBoolean && b.analyticBoolean
            && a.analyticBoolean->bytes == b.analyticBoolean->bytes;
    } catch (...) { return false; }
}

bool SamePrepared(const CreationPrepared& a,
                  const CreationPrepared& b) noexcept {
    try {
        if (a.required != b.required || a.totalLength != b.totalLength
            || !SameProjection(a.projection, b.projection)
            || a.build.totalArcLength != b.build.totalArcLength
            || a.build.maximumMeasuredArcError != b.build.maximumMeasuredArcError
            || a.build.requestedInstances != b.build.requestedInstances
            || a.build.emittedInstances != b.build.emittedInstances
            || a.build.closedSeamCanonicalized != b.build.closedSeamCanonicalized
            || a.canonicalBytes != b.canonicalBytes
            || a.placements.size() != b.placements.size()
            || a.issued.size() != b.issued.size()
            || a.recipes.size() != b.recipes.size()
            || a.labels.creates.size() != b.labels.creates.size()) return false;
        for (std::size_t i = 0; i < a.placements.size(); ++i) {
            const auto& x = a.placements[i]; const auto& y = b.placements[i];
            if (x.identity != y.identity || x.localID != y.localID
                || x.ordinal != y.ordinal || x.requestedArcLength != y.requestedArcLength
                || x.measuredArcLength != y.measuredArcLength
                || x.parameter != y.parameter
                || x.occurrenceFrame != y.occurrenceFrame) return false;
        }
        for (std::size_t i = 0; i < a.issued.size(); ++i)
            if (a.issued[i].EntityIdentifier() != b.issued[i].EntityIdentifier()
                || a.issued[i].DefinitionIdentifier() != b.issued[i].DefinitionIdentifier())
                return false;
        for (std::size_t i = 0; i < a.recipes.size(); ++i)
            if (a.recipes[i].entityIdentifier != b.recipes[i].entityIdentifier
                || a.recipes[i].featureIdentifier != b.recipes[i].featureIdentifier
                || !SamePreparedRecipe(a.recipes[i].prepared, b.recipes[i].prepared))
                return false;
        return true;
    } catch (...) { return false; }
}

bool EqualityReprepare(const CreationCapture& frozenCapture,
                       const CreationPrepared& frozen,
                       CreationCapture& freshCapture,
                       CreationPrepared& fresh,
                       NSString *__strong& phase,
                       NSString *__strong& code) noexcept {
    phase = @"finalCurrentness"; code = @"staleCapture";
    try {
        if (!RefreshCapture(frozenCapture, freshCapture)) return false;
        std::vector<std::string> features;
        features.reserve(frozen.recipes.size());
        for (const auto& entry : frozen.recipes)
            features.push_back(entry.featureIdentifier);
        if (!BuildPreparedPlan(freshCapture, frozen.definition, frozen.issued,
                               features, fresh, phase, code)) return false;
        phase = @"finalEquality"; code = @"repreparationMismatch";
        if (!SamePrepared(frozen, fresh)) return false;
        phase = @"finalEquality"; code = @"none"; return true;
    } catch (...) { fresh = {}; phase = @"exception"; code = @"nativeFailure"; return false; }
}

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
        if (prepared.family != recipe::Family::AnalyticBoolean
            || !prepared.analyticBoolean) return false;
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

bool ReadBackCreation(OcctDocument& owner, const CreationCapture& capture,
                      const CreationPrepared& prepared,
                      const std::vector<OcctExactLabelReceipt>& receipts) noexcept {
    try {
        const Handle(TDocStd_Document) document = owner.Document();
        if (document.IsNull() || !owner.ReadBackAllLabels(prepared.labels, receipts))
            return false;
        array::Record record;
        if (!array::ReadFeature(document, prepared.definition.feature, record)
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
                || !recipe::Capture(document, found->visibility.object.object.label, actual)
                || !PreparedRecipeMatches(entry.prepared, actual)) return false;
        }
        recipe::Source source;
        OcctBoundedCurveCapture path;
        return recipe::Capture(document, capture.sourceLabel, source)
            && recipe::IsEqual(source, capture.sourceRecipe)
            && owner.ReadBoundedCurveExact(prepared.definition.path.owner, path)
            && path.IsEqual(capture.pathExact);
    } catch (...) { return false; }
}

enum class CreateOutcome : std::uint8_t { Refused = 0, Committed, OutcomeUnknown };

CreateOutcome ApplyCreation(const CreationCapture& frozenCapture,
                            const CreationPrepared& frozenPrepared) noexcept {
    try {
        CreationCapture capture; CreationPrepared prepared;
        NSString *phase = nil, *code = nil;
        if (!EqualityReprepare(frozenCapture, frozenPrepared, capture, prepared,
                               phase, code)) return CreateOutcome::Refused;
        OcctDocument& owner = *capture.owner;
        const Handle(TDocStd_Document) document = owner.Document();
        auto lease = capture.context->beginCommandLease(
            capture.context->openingFence(), capture.viewportWidth,
            capture.viewportHeight);
        if (!lease || !lease->ownsOpenCommand()) return CreateOutcome::Refused;
        const int undoBefore = document->GetAvailableUndos();
        const auto abort = [&]() {
            if (!lease->abort()) return CreateOutcome::OutcomeUnknown;
            std::vector<array::Record> records;
            bool restored = document->GetAvailableUndos() == undoBefore
                && array::ReadAll(document, records);
            if (restored)
                for (const auto& record : records)
                    if (record.definition.feature == prepared.definition.feature)
                        restored = false;
            if (restored)
                for (const auto& identity : prepared.issued) {
                    UUID entity{}; core3d::pattern_owner::LabelReceipt orphan;
                    if (core3d::pattern_owner::Parse(identity.EntityIdentifier(), entity)
                        && ResolveFreeLabel(owner, entity, orphan)) restored = false;
                }
            return restored ? CreateOutcome::Refused : CreateOutcome::OutcomeUnknown;
        };
        std::vector<OcctExactLabelReceipt> receipts;
        bool staged = owner.StageAllLabels(*lease, prepared.labels, receipts);
        if (staged) {
            for (const auto& entry : prepared.recipes) {
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
        array::Record stagedRecord;
        if (staged)
            staged = array::Stage(document, prepared.definition, stagedRecord)
                && stagedRecord.definition.feature == prepared.definition.feature
                && stagedRecord.bytes == prepared.canonicalBytes;
        if (staged) staged = ReadBackCreation(owner, capture, prepared, receipts);
        if (!staged) return abort();
        const auto publication =
            core3d::native_opening::PublicationFromAllLabelPlan(prepared.labels);
        if (!lease->commit()) {
            capture.context->retainUnprovenEdit(publication);
            return CreateOutcome::OutcomeUnknown;
        }
        if (document->HasOpenCommand()
            || document->GetAvailableUndos() - undoBefore != 1
            || !ReadBackCreation(owner, capture, prepared, receipts)) {
            capture.context->retainUnprovenEdit(publication);
            return CreateOutcome::OutcomeUnknown;
        }
        if (!capture.context->publishCommittedEdit(publication)) {
            capture.context->retainUnprovenEdit(publication);
            return CreateOutcome::OutcomeUnknown;
        }
        return CreateOutcome::Committed;
    } catch (...) { return CreateOutcome::Refused; }
}

void DeliverCreated(void (^completion)(Core3DProfileConstructionResult, NSString *, NSString *),
                    Core3DProfileConstructionResult result, NSString *detail,
                    NSString *entityIdentifier) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail, entityIdentifier);
    else dispatch_async(dispatch_get_main_queue(), ^{
        completion(result, detail, entityIdentifier); });
}

} // namespace

@interface Core3DPathArrayPlacementPreview () {
@package
    NSString *_entityIdentifier;
    NSUInteger _localIdentifier, _ordinal;
    double _requestedArcLength, _measuredArcLength;
    NSArray<NSNumber *> *_occurrenceFrameValues;
}
@end

@implementation Core3DPathArrayPlacementPreview
@synthesize entityIdentifier = _entityIdentifier;
@synthesize localIdentifier = _localIdentifier;
@synthesize ordinal = _ordinal;
@synthesize requestedArcLength = _requestedArcLength;
@synthesize measuredArcLength = _measuredArcLength;
@synthesize occurrenceFrameValues = _occurrenceFrameValues;
+ (instancetype)core3dPlacementWithEntityIdentifier:(NSString *)entityIdentifier
    localIdentifier:(NSUInteger)localIdentifier
    ordinal:(NSUInteger)ordinal
    requestedArcLength:(double)requestedArcLength
    measuredArcLength:(double)measuredArcLength
    occurrenceFrameValues:(NSArray<NSNumber *> *)occurrenceFrameValues {
    Core3DPathArrayPlacementPreview *result = [self alloc];
    result->_entityIdentifier = [entityIdentifier copy] ?: @"";
    result->_localIdentifier = localIdentifier; result->_ordinal = ordinal;
    result->_requestedArcLength = requestedArcLength;
    result->_measuredArcLength = measuredArcLength;
    result->_occurrenceFrameValues = [occurrenceFrameValues copy] ?: @[];
    return result;
}
@end

@interface Core3DPathArrayPreview () {
@package
    BOOL _admitted;
    NSString *_phase, *_refusalDomain, *_refusalCode;
    NSNumber *_requiredInstanceCount, *_totalLength;
    NSNumber *_requestedPlacementCount, *_emittedPlacementCount;
    NSNumber *_maximumMeasuredArcError, *_closedSeamCanonicalized;
    NSNumber *_projectedInstances, *_projectedTopologyNodes;
    NSNumber *_projectedDocumentBytes, *_projectedMemoryBytes;
    NSNumber *_sourceTopologyNodes, *_sourceDocumentBytes, *_sourceMemoryBytes;
    NSNumber *_issuedMemberIdentityCount;
    double _documentMetersPerUnit;
    NSString *_sourceEntityIdentifier, *_pathEntityIdentifier;
    NSString *_ownerEntityIdentifier, *_featureIdentifier;
    NSArray<Core3DPathArrayPlacementPreview *> *_placements;
}
@end

@implementation Core3DPathArrayPreview
@synthesize admitted = _admitted;
@synthesize phase = _phase;
@synthesize refusalDomain = _refusalDomain;
@synthesize refusalCode = _refusalCode;
@synthesize requiredInstanceCount = _requiredInstanceCount;
@synthesize totalLength = _totalLength;
@synthesize requestedPlacementCount = _requestedPlacementCount;
@synthesize emittedPlacementCount = _emittedPlacementCount;
@synthesize maximumMeasuredArcError = _maximumMeasuredArcError;
@synthesize closedSeamCanonicalized = _closedSeamCanonicalized;
@synthesize projectedInstances = _projectedInstances;
@synthesize projectedTopologyNodes = _projectedTopologyNodes;
@synthesize projectedDocumentBytes = _projectedDocumentBytes;
@synthesize projectedMemoryBytes = _projectedMemoryBytes;
@synthesize sourceTopologyNodes = _sourceTopologyNodes;
@synthesize sourceDocumentBytes = _sourceDocumentBytes;
@synthesize sourceMemoryBytes = _sourceMemoryBytes;
@synthesize issuedMemberIdentityCount = _issuedMemberIdentityCount;
@synthesize documentMetersPerUnit = _documentMetersPerUnit;
@synthesize sourceEntityIdentifier = _sourceEntityIdentifier;
@synthesize pathEntityIdentifier = _pathEntityIdentifier;
@synthesize ownerEntityIdentifier = _ownerEntityIdentifier;
@synthesize featureIdentifier = _featureIdentifier;
@synthesize placements = _placements;
@end

@interface Core3DPathArrayPreparedCandidate () {
@package
    __weak id _issuer;
    std::uint64_t _generation;
    std::uint8_t _kind;
}
- (instancetype)initWithIssuer:(id)issuer
    generation:(std::uint64_t)generation
    kind:(std::uint8_t)kind;
@end

@implementation Core3DPathArrayPreparedCandidate
- (instancetype)initWithIssuer:(id)issuer
    generation:(std::uint64_t)generation
    kind:(std::uint8_t)kind {
    if ((self = [super init])) {
        _issuer = issuer; _generation = generation; _kind = kind;
    }
    return self;
}
+ (instancetype)core3dTokenWithIssuer:(id)issuer
    generation:(uint64_t)generation
    kind:(uint8_t)kind {
    return [[self alloc] initWithIssuer:issuer generation:generation kind:kind];
}
- (BOOL)core3dMatchesIssuer:(id)issuer
    generation:(uint64_t)generation
    kind:(uint8_t)kind {
    return _issuer == issuer && _generation == generation && _kind == kind;
}
@end

@interface Core3DPathArrayPreparation () {
@package
    Core3DPathArrayPreview *_preview;
    Core3DPathArrayPreparedCandidate *_prepared;
}
- (instancetype)initWithPreview:(Core3DPathArrayPreview *)preview
    prepared:(Core3DPathArrayPreparedCandidate *)prepared;
@end

@implementation Core3DPathArrayPreparation
@synthesize preview = _preview;
@synthesize prepared = _prepared;
- (instancetype)initWithPreview:(Core3DPathArrayPreview *)preview
    prepared:(Core3DPathArrayPreparedCandidate *)prepared {
    if ((self = [super init])) { _preview = preview; _prepared = prepared; }
    return self;
}
+ (instancetype)core3dPreparationWithPreview:(Core3DPathArrayPreview *)preview
    prepared:(Core3DPathArrayPreparedCandidate *)prepared {
    return [[self alloc] initWithPreview:preview prepared:prepared];
}
@end

@implementation Core3DPathArrayPreview (Core3DNativeSupport)
+ (instancetype)core3dPreviewWithValues:(NSDictionary<NSString *, id> *)values {
    Core3DPathArrayPreview *result = [self alloc];
    id (^nullable)(NSString *) = ^id(NSString *key) {
        id value = values[key]; return value == NSNull.null ? nil : value;
    };
    result->_admitted = [values[@"admitted"] boolValue];
    result->_phase = [values[@"phase"] copy] ?: @"unavailable";
    result->_refusalDomain = [nullable(@"refusalDomain") copy];
    result->_refusalCode = [nullable(@"refusalCode") copy];
    result->_requiredInstanceCount = nullable(@"requiredInstanceCount");
    result->_totalLength = nullable(@"totalLength");
    result->_requestedPlacementCount = nullable(@"requestedPlacementCount");
    result->_emittedPlacementCount = nullable(@"emittedPlacementCount");
    result->_maximumMeasuredArcError = nullable(@"maximumMeasuredArcError");
    result->_closedSeamCanonicalized = nullable(@"closedSeamCanonicalized");
    result->_projectedInstances = nullable(@"projectedInstances");
    result->_projectedTopologyNodes = nullable(@"projectedTopologyNodes");
    result->_projectedDocumentBytes = nullable(@"projectedDocumentBytes");
    result->_projectedMemoryBytes = nullable(@"projectedMemoryBytes");
    result->_sourceTopologyNodes = nullable(@"sourceTopologyNodes");
    result->_sourceDocumentBytes = nullable(@"sourceDocumentBytes");
    result->_sourceMemoryBytes = nullable(@"sourceMemoryBytes");
    result->_issuedMemberIdentityCount = nullable(@"issuedMemberIdentityCount");
    result->_documentMetersPerUnit = [values[@"documentMetersPerUnit"] doubleValue];
    result->_sourceEntityIdentifier = [values[@"sourceEntityIdentifier"] copy] ?: @"";
    result->_pathEntityIdentifier = [values[@"pathEntityIdentifier"] copy] ?: @"";
    result->_ownerEntityIdentifier = [nullable(@"ownerEntityIdentifier") copy];
    result->_featureIdentifier = [nullable(@"featureIdentifier") copy];
    result->_placements = nullable(@"placements") ?: @[];
    return result;
}
@end

namespace {
Core3DPathArrayPreview *Preview(const CreationCapture& capture,
                                const CreationPrepared& prepared,
                                BOOL admitted, NSString *phase,
                                NSString *code) {
    Core3DPathArrayPreview *result = [Core3DPathArrayPreview alloc];
    result->_admitted = admitted;
    result->_phase = [phase copy] ?: @"unavailable";
    result->_refusalDomain = admitted ? nil : @"core3d.path-array.creation";
    result->_refusalCode = admitted ? nil : ([code copy] ?: @"nativeFailure");
    result->_documentMetersPerUnit = capture.metersPerUnit;
    result->_sourceEntityIdentifier = Text(capture.selected);
    result->_pathEntityIdentifier = Text(capture.pathEntity);
    result->_sourceTopologyNodes = @(capture.measured.topologyNodes);
    result->_sourceDocumentBytes = @(capture.measured.shapeBytes);
    result->_sourceMemoryBytes = @(capture.measured.retainedMemoryBytes);
    if (prepared.required != 0) {
        result->_requiredInstanceCount = @(prepared.required);
        result->_totalLength = @(prepared.totalLength);
        result->_issuedMemberIdentityCount = @(prepared.required - 1);
    }
    if (prepared.projection.instances != 0) {
        result->_projectedInstances = @(prepared.projection.instances);
        result->_projectedTopologyNodes = @(prepared.projection.aggregateTopologyNodes);
        result->_projectedDocumentBytes = @(prepared.projection.projectedDocumentBytes);
        result->_projectedMemoryBytes = @(prepared.projection.projectedMemoryBytes);
    }
    if (prepared.build.requestedInstances != 0) {
        result->_requestedPlacementCount = @(prepared.build.requestedInstances);
        result->_emittedPlacementCount = @(prepared.build.emittedInstances);
        result->_maximumMeasuredArcError = @(prepared.build.maximumMeasuredArcError);
        result->_closedSeamCanonicalized = @(prepared.build.closedSeamCanonicalized);
    }
    if (admitted) {
        result->_ownerEntityIdentifier = UUIDText(prepared.definition.owner.entity);
        result->_featureIdentifier = UUIDText(prepared.definition.feature);
    }
    NSMutableArray *placements = [NSMutableArray arrayWithCapacity:prepared.placements.size()];
    for (const auto& item : prepared.placements) {
        Core3DPathArrayPlacementPreview *placement =
            [Core3DPathArrayPlacementPreview alloc];
        placement->_entityIdentifier = UUIDText(item.identity);
        placement->_localIdentifier = NSUInteger(item.localID);
        placement->_ordinal = NSUInteger(item.ordinal);
        placement->_requestedArcLength = item.requestedArcLength;
        placement->_measuredArcLength = item.measuredArcLength;
        NSMutableArray *frame = [NSMutableArray arrayWithCapacity:16];
        for (double scalar : item.occurrenceFrame) [frame addObject:@(scalar)];
        placement->_occurrenceFrameValues = frame;
        [placements addObject:placement];
    }
    result->_placements = placements;
    return result;
}
} // namespace

@interface Core3DPathArrayCreationOpening () {
@package
    CreationCapture _capture;
    CreationPrepared _preparedPlan;
    std::atomic<State> _state;
    std::uint64_t _generation;
    Core3DPathArrayPreparedCandidate *_token;
    NSDictionary<NSString *, id> *_descriptor;
#if DEBUG
    void (^_debugApplyingObserver)(Core3DPathArrayCreationOpening *);
#endif
}
- (instancetype)initWithCapture:(CreationCapture)capture;
@end

@implementation Core3DPathArrayCreationOpening

@synthesize descriptor = _descriptor;

- (instancetype)initWithCapture:(CreationCapture)capture {
    if ((self = [super init])) {
        _capture = std::move(capture); _state.store(State::Open); _generation = 1;
        _descriptor = [CreationDescriptor(_capture) copy];
    }
    return self;
}

- (Core3DPathArrayPreparation *)prepareCandidate:(NSDictionary *)candidate {
    const State state = _state.load();
    if (state != State::Open && state != State::Prepared) {
        return [[Core3DPathArrayPreparation alloc]
            initWithPreview:Preview(_capture, {}, NO, @"lifecycle", @"issuerUnavailable")
            prepared:nil];
    }
    ++_generation; _token = nil; _preparedPlan = {}; _state.store(State::Open);
    NSString *phase = nil, *code = nil;
    CreationPrepared prepared;
    const bool admitted = [candidate isKindOfClass:NSDictionary.class]
        && PrepareInitial(_capture, candidate, prepared, phase, code);
    Core3DPathArrayPreview *preview = Preview(_capture, prepared, admitted,
        phase ?: @"parse", code ?: @"malformedCandidate");
    if (!admitted)
        return [[Core3DPathArrayPreparation alloc]
            initWithPreview:preview prepared:nil];
    _preparedPlan = std::move(prepared);
    _state.store(State::Prepared);
    _token = [Core3DPathArrayPreparedCandidate core3dTokenWithIssuer:self
        generation:_generation kind:1];
    return [[Core3DPathArrayPreparation alloc]
        initWithPreview:preview prepared:_token];
}

- (void)applyPrepared:(Core3DPathArrayPreparedCandidate *)prepared
    completion:(void (^)(Core3DProfileConstructionResult, NSString *, NSString *))completion {
    // Validate kind/issuer/generation before spending this issuer's rightful token.
    if (!NSThread.isMainThread
        || ![prepared isMemberOfClass:Core3DPathArrayPreparedCandidate.class]
        || ![prepared core3dMatchesIssuer:self generation:_generation kind:1]
        || prepared != _token) {
        DeliverCreated(completion, Core3DProfileConstructionResultRejected,
                       @"Prepared candidate belongs to another or retired issuer.", nil);
        return;
    }
    State expected = State::Prepared;
    if (!_state.compare_exchange_strong(expected, State::Applying)) {
        DeliverCreated(completion, Core3DProfileConstructionResultRejected,
                       @"No current path-array preparation.", nil);
        return;
    }
#if DEBUG
    if (_debugApplyingObserver) {
        auto observer = _debugApplyingObserver;
        _debugApplyingObserver = nil;
        observer(self);
    }
#endif
    _token = nil; ++_generation;
    const auto outcome = ApplyCreation(_capture, _preparedPlan);
    const Core3DProfileConstructionResult result = outcome == CreateOutcome::Committed
        ? Core3DProfileConstructionResultCommitted
        : outcome == CreateOutcome::OutcomeUnknown
            ? Core3DProfileConstructionResultRecoveryRequired
            : Core3DProfileConstructionResultRejected;
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? State::Recovery : State::Settled);
    NSString *source = result == Core3DProfileConstructionResultCommitted
        ? Text(_capture.selected) : nil;
    if (result != Core3DProfileConstructionResultRecoveryRequired) {
        _preparedPlan = {}; _capture = {};
    }
    DeliverCreated(completion, result,
        result == Core3DProfileConstructionResultCommitted
            ? @"Path array created as one native history command."
            : result == Core3DProfileConstructionResultRecoveryRequired
                ? @"Path-array close is unknown; native recovery ownership is retained."
                : @"Final currentness or equality re-preparation refused without history.",
        source);
}

- (Core3DPathArrayCreationOpening *)replacingPathWithEntityIdentifier:
    (NSString *)entityIdentifier {
    const State state = _state.load();
    if ((state != State::Open && state != State::Prepared)
        || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0 || entityIdentifier.length > 128) return nil;
    CreationCapture replacement = _capture;
    if (!CapturePath(replacement, entityIdentifier.UTF8String ?: "")) return nil;
    Core3DPathArrayCreationOpening *next =
        [[Core3DPathArrayCreationOpening alloc] initWithCapture:std::move(replacement)];
    if (![self cancel]) return nil;
    return next;
}

- (BOOL)cancel {
    State value = _state.load();
    while (value == State::Open || value == State::Prepared) {
        if (_state.compare_exchange_weak(value, State::Cancelled)) {
            ++_generation; _token = nil; _preparedPlan = {}; _capture = {};
            return YES;
        }
    }
    return NO;
}

#if DEBUG
- (void)debugReportNextCloseUnproven {
    if (_capture.context) _capture.context->debugReportNextCloseUnproven();
}

- (void)debugObserveApplying:
    (void (^)(Core3DPathArrayCreationOpening *))observer {
    _debugApplyingObserver = [observer copy];
}

- (BOOL)debugReconcileExactRecovery {
    if (_state.load() != State::Recovery || _capture.owner.IsNull()
        || !_capture.context || _preparedPlan.recipes.empty()) return NO;
    try {
        std::vector<OcctExactLabelReceipt> receipts;
        receipts.reserve(_preparedPlan.recipes.size());
        for (const auto& entry : _preparedPlan.recipes) {
            UUID entity{}; core3d::pattern_owner::LabelReceipt label;
            OcctExactLabelReceipt exact;
            if (!core3d::pattern_owner::Parse(entry.entityIdentifier, entity)
                || !ResolveFreeLabel(*_capture.owner, entity, label)
                || !_capture.owner->CaptureExactFreeLabel(label.label, exact)) return NO;
            receipts.push_back(std::move(exact));
        }
        if (!ReadBackCreation(*_capture.owner, _capture, _preparedPlan, receipts)
            || !_capture.context->reconcileRecovery(true)) return NO;
        _state.store(State::Settled);
        _preparedPlan = {}; _capture = {};
        return YES;
    } catch (...) { return NO; }
}
#endif

#if DEBUG
- (NSDictionary<NSString *, id> *)debugNativePreparationObservation {
    if (!NSThread.isMainThread || _state.load() != State::Prepared) return nil;
    @autoreleasepool {
        try {
            const array::Definition definition = _preparedPlan.definition;
            std::uint32_t required = 0; double totalLength = 0;
            const auto count = array::RequiredInstanceCount(definition,
                _capture.pathExact.persisted, required, totalLength);
            if (count != array::BuildRefusal::None) return nil;
            array::AdmissionBudget budget;
            budget.maximumInstances = array::MaximumInstances;
            budget.sourceTopologyNodes = _capture.measured.topologyNodes;
            budget.maximumAggregateTopologyNodes = 2'000'000;
            budget.sourceDocumentBytes = _capture.measured.shapeBytes;
            budget.existingDocumentBytes = _capture.patternDocumentBytes
                + _capture.pathArrayDocumentBytes + _capture.compositeDocumentBytes;
            budget.maximumDocumentBytes = array::MaximumDocumentBytes;
            budget.sourceMemoryBytes = _capture.measured.retainedMemoryBytes;
            budget.maximumMemoryBytes = 256 * 1024 * 1024;
            const array::Projection projection = array::Project(required, budget);
            std::vector<array::Placement> placements;
            array::BuildReceipt receipt;
            if (!projection.admitted
                || array::BuildPlacements(definition, _capture.pathExact.persisted,
                    placements, receipt) != array::BuildRefusal::None) return nil;
            NSMutableArray *rows = [NSMutableArray arrayWithCapacity:placements.size()];
            for (const auto& placement : placements)
                [rows addObject:@{ @"ordinal": @(placement.ordinal),
                    @"requestedArcLength": @(placement.requestedArcLength),
                    @"measuredArcLength": @(placement.measuredArcLength) }];
            return @{ @"schema": @"shapeyard.d3-native-observation.v1",
                @"requiredInstanceCount": @(required), @"totalLength": @(totalLength),
                @"requestedPlacementCount": @(receipt.requestedInstances),
                @"emittedPlacementCount": @(receipt.emittedInstances),
                @"maximumMeasuredArcError": @(receipt.maximumMeasuredArcError),
                @"closedSeamCanonicalized": @(receipt.closedSeamCanonicalized),
                @"projectedInstances": @(projection.instances),
                @"projectedTopologyNodes": @(projection.aggregateTopologyNodes),
                @"projectedDocumentBytes": @(projection.projectedDocumentBytes),
                @"projectedMemoryBytes": @(projection.projectedMemoryBytes),
                @"sourceTopologyNodes": @(_capture.measured.topologyNodes),
                @"sourceDocumentBytes": @(_capture.measured.shapeBytes),
                @"sourceMemoryBytes": @(_capture.measured.retainedMemoryBytes),
                @"placements": rows };
        } catch (...) { return nil; }
    }
}
#endif

@end

@implementation Core3DViewController (PathArrayCreationOpening)

- (Core3DPathArrayCreationOpening *)beginPathArrayCreationWithPathEntityIdentifier:
    (NSString *)entityIdentifier {
    CreationCapture capture;
    if (!CaptureCreationSource(self, entityIdentifier, capture)) return nil;
    return [[Core3DPathArrayCreationOpening alloc] initWithCapture:std::move(capture)];
}

#if DEBUG
- (NSDictionary<NSString *, id> *)debugPathArrayCreationEvidenceForEntityIdentifier:
    (NSString *)entityIdentifier {
    if (![NSThread isMainThread] || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0 || entityIdentifier.length > 128) return nil;
    @autoreleasepool {
        try {
            GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
                ? (GLViewController *)self.glController : nil;
            const Handle(OcctDocument) owner = gl && gl.viewer
                ? gl.viewer->getDocument() : Handle(OcctDocument)();
            const Handle(TDocStd_Document) document = owner.IsNull()
                ? Handle(TDocStd_Document)() : owner->Document();
            if (document.IsNull()) return nil;
            NSMutableDictionary *base = [@{
                @"schema": @"shapeyard.d3-path-array-creation-evidence.v1",
                @"recordCount": @0, @"valid": @NO,
                @"undoCount": @(document->GetAvailableUndos()),
                @"redoCount": @(document->GetAvailableRedos()),
                @"documentMetersPerUnit": @0,
                @"openCommand": @(document->HasOpenCommand()),
                @"tableReadable": @NO
            } mutableCopy];
            double unit = 0;
            if (XCAFDoc_DocumentTool::GetLengthUnit(document, unit))
                base[@"documentMetersPerUnit"] = @(unit);
            if (document->HasOpenCommand()) return base;
            UUID requested{};
            if (!core3d::pattern_owner::Parse(entityIdentifier.UTF8String ?: "", requested))
                return base;
            std::vector<array::Record> records;
            if (!array::ReadAll(document, records)) return base;
            base[@"tableReadable"] = @YES;
            base[@"recordCount"] = @(records.size());
            const array::Record *match = nullptr;
            for (const auto& record : records) {
                bool hit = record.definition.owner.entity == requested
                    || record.definition.owner.definition == requested
                    || record.definition.feature == requested
                    || record.definition.source.entity == requested;
                for (const auto& member : record.definition.members)
                    hit = hit || member.identity == requested;
                if (!hit) continue;
                if (match) return nil;
                match = &record;
            }
            NSMutableDictionary *evidence = base;
            evidence[@"valid"] = @(match != nullptr);
            if (!match) return evidence;
            const auto& definition = match->definition;
            NSMutableArray *members = [NSMutableArray array];
            for (const auto& member : definition.members) {
                core3d::pattern_owner::LabelReceipt label;
                recipe::Source memberRecipe;
                NSString *family = @"missing", *recipeFeature = @"";
                NSString *definitionIdentifier = @"";
                NSData *shapeBytes = [NSData data];
                if (ResolveFreeLabel(*owner, member.identity, label)
                    && recipe::Capture(document, label.label, memberRecipe)) {
                    family = memberRecipe.family == recipe::Family::None ? @"none"
                        : memberRecipe.family == recipe::Family::Sweep ? @"sweep"
                        : memberRecipe.family == recipe::Family::Loft ? @"loft"
                        : @"analyticBoolean";
                    recipeFeature = Text(
                        core3d::pattern_owner::RecipeFeatureIdentifier(memberRecipe));
                    OcctExactLabelReceipt exact;
                    std::string bytes;
                    if (owner->CaptureExactFreeLabel(label.label, exact)
                        && core3d::retained_part_boolean::ExactShapeBytes(
                            exact.visibility.object.object.shape, bytes)) {
                        definitionIdentifier = Text(
                            exact.visibility.object.object.definitionIdentifier);
                        shapeBytes = [NSData dataWithBytes:bytes.data()
                                                   length:bytes.size()];
                    }
                }
                [members addObject:@{@"entityIdentifier": UUIDText(member.identity),
                    @"definitionIdentifier": definitionIdentifier,
                    @"localID": @(member.localID), @"ordinal": @(member.coordinate.column),
                    @"suppressed": @(member.state
                        == core3d::pattern::MemberState::Suppressed),
                    @"recipeFamily": family,
                    @"recipeFeatureIdentifier": recipeFeature,
                    @"shapeBytes": shapeBytes}];
            }
            NSMutableArray *removals = [NSMutableArray array];
            for (const auto& removed : definition.removals)
                [removals addObject:@{
                    @"entityIdentifier": UUIDText(removed.identity),
                    @"localID": @(removed.localID),
                    @"ordinal": @(removed.coordinate.column)
                }];
            NSMutableArray *retiredLocalIDs = [NSMutableArray array];
            for (const auto localID : definition.issuance.retiredLocalIDs)
                [retiredLocalIDs addObject:@(localID)];
            evidence[@"ownerEntityIdentifier"] = UUIDText(definition.owner.entity);
            evidence[@"ownerDefinitionIdentifier"] = UUIDText(definition.owner.definition);
            evidence[@"featureIdentifier"] = UUIDText(definition.feature);
            evidence[@"sourceEntityIdentifier"] = UUIDText(definition.source.entity);
            evidence[@"pathEntityIdentifier"] = UUIDText(definition.path.owner.entity);
            evidence[@"memberCount"] = @(definition.members.size());
            evidence[@"removalCount"] = @(definition.removals.size());
            evidence[@"recordBytes"] = @(match->bytes.size());
            evidence[@"canonicalRecordBytes"] = [NSData dataWithBytes:match->bytes.data()
                length:match->bytes.size()];
            OcctBoundedCurveCapture path;
            if (owner->ReadBoundedCurveExact(definition.path.owner, path)) {
                std::vector<std::uint8_t> pathValue, pathOwner;
                if (curve::Encode(path.persisted.value, pathValue)
                    && curve::EncodeOwnerState(path.persisted.ownerState, pathOwner)) {
                    evidence[@"pathDefinitionBytes"] = [NSData dataWithBytes:pathValue.data()
                        length:pathValue.size()];
                    evidence[@"pathOwnerBytes"] = [NSData dataWithBytes:pathOwner.data()
                        length:pathOwner.size()];
                    evidence[@"pathDefinitionRevision"] =
                        @(path.persisted.ownerState.definitionRevision);
                }
            }
            evidence[@"members"] = members;
            evidence[@"removals"] = removals;
            evidence[@"retiredLocalIDs"] = retiredLocalIDs;
            return evidence;
        } catch (...) { return nil; }
    }
}

- (NSDictionary<NSString *, id> *)
    debugPathArrayNativeMutationEvidenceForEntityIdentifier:
    (NSString *)entityIdentifier {
    if (![NSThread isMainThread] || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || !entityIdentifier.UTF8String) return nil;
    const std::string entity(entityIdentifier.UTF8String,
        [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
    return core3d::path_array_owner::DebugNativeMutationEvidence(entity);
}
#endif

@end
