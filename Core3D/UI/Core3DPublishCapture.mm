#import "Core3DPublishCapture.h"

#import "Core3DViewController.h"
#import "Core3DViewController+ExportManager.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/NativeOpeningContext.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/CylindricalCutDefinition.hxx"
#include "../OCCTKit/ReceiptRecord.hxx"
#include "../OCCTKit/RetainedBooleanProgram.hxx"
#include "../OCCTKit/SavedCutSourceEdit.hxx"
#include "Core3DViewer.h"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TDF_Tool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

NSErrorDomain const Core3DPublishCaptureErrorDomain = @"Core3DPublishCaptureErrorDomain";

namespace {
using OpeningContext = core3d::native_opening::Context;
using Recipe = core3d::retained_boolean::Recipe;

constexpr NSUInteger kMaximumOwners = 96;
constexpr NSUInteger kMaximumIdentifierBytes = 128;
constexpr NSUInteger kMaximumEvidenceBytes = 64U * 1024U * 1024U;

struct Bytes final {
    std::vector<std::uint8_t> value;
    bool good = true;

    void raw(const void *source, std::size_t count) {
        if (!good || count > kMaximumEvidenceBytes - std::min<std::size_t>(value.size(), kMaximumEvidenceBytes)) {
            good = false; return;
        }
        const auto *first = static_cast<const std::uint8_t *>(source);
        value.insert(value.end(), first, first + count);
    }
    void u64(std::uint64_t number) {
        std::uint8_t bytes[8];
        for (unsigned index = 0; index < 8; ++index) bytes[index] = std::uint8_t(number >> (8 * index));
        raw(bytes, sizeof(bytes));
    }
    void number(double number) {
        std::uint64_t bits = 0; std::memcpy(&bits, &number, sizeof(bits)); u64(bits);
    }
    void field(const void *source, std::size_t count) { u64(count); raw(source, count); }
    void text(NSString *text) {
        NSData *data = [text dataUsingEncoding:NSUTF8StringEncoding allowLossyConversion:NO];
        if (!data) { good = false; return; }
        field(data.bytes, data.length);
    }
    void text(const std::string& text) { field(text.data(), text.size()); }
    void data(NSData *data) { field(data.bytes, data.length); }
    NSData *finish() const {
        return good && !value.empty() ? [NSData dataWithBytes:value.data() length:value.size()] : nil;
    }
};

struct OwnerObservation final {
    NSString *entity = nil;
    NSString *definition = nil;
    NSData *envelope = nil;
    NSData *operand = nil;
    std::array<double, 6> boundsMM{};
};

struct Observation final {
    Core3DSceneSnapshot *scene = nil;
    NSString *document = nil;
    NSArray<NSString *> *selected = nil;
    NSArray<NSNumber *> *bounds = nil;
    NSData *source = nil;
    NSArray<NSData *> *operands = nil;
};

NSError *Refusal(Core3DPublishCaptureError code, NSString *reason) {
    return [NSError errorWithDomain:Core3DPublishCaptureErrorDomain code:code
                           userInfo:@{NSLocalizedDescriptionKey: reason ?: @"publish capture refused"}];
}

bool Identifier(NSString *value) {
    NSData *bytes = [value dataUsingEncoding:NSUTF8StringEncoding allowLossyConversion:NO];
    return bytes.length > 0 && bytes.length <= kMaximumIdentifierBytes
        && memchr(bytes.bytes, 0, bytes.length) == nullptr;
}

NSString *UUIDText(const core3d::retained_solid::UUID& value) {
    return [NSString stringWithUTF8String:core3d::retained_solid::UUIDText(value).c_str()] ?: @"";
}

bool RootForEntity(const Handle(OcctDocument)& owner, NSString *entity,
                   TDF_Label& output, Core3DPublishCaptureError& refusal) {
    output.Nullify(); refusal = Core3DPublishCaptureErrorUnsupportedOwner;
    if (owner.IsNull() || !Identifier(entity)) return false;
    const auto document = owner->Document();
    if (document.IsNull() || document->GetData().IsNull()) return false;
    const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
    if (shapes.IsNull()) return false;
    TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
    unsigned matches = 0;
    const char *raw = entity.UTF8String;
    for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
        if (owner->EntityIdentifierForLabel(roots.Value(index)) == (raw ? raw : "")) {
            output = roots.Value(index); ++matches;
        }
    }
    if (matches > 1) refusal = Core3DPublishCaptureErrorAmbiguousRecipe;
    return matches == 1 && !output.IsNull();
}

bool NativeBounds(const OcctObjectTransformState& state, double metersPerUnit,
                  std::array<double, 6>& result) {
    try {
        if (state.shape.IsNull() || !std::isfinite(metersPerUnit) || metersPerUnit <= 0) return false;
        Bnd_Box box; BRepBndLib::AddOptimal(state.shape, box, Standard_False, Standard_False);
        if (box.IsVoid() || box.IsOpen()) return false;
        Standard_Real xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        gp_Pnt corners[8]; unsigned at = 0;
        for (double x : {double(xmin), double(xmax)})
            for (double y : {double(ymin), double(ymax)})
                for (double z : {double(zmin), double(zmax)}) {
                    corners[at] = gp_Pnt(x, y, z).Transformed(state.transform); ++at;
                }
        double lo[3] = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        double hi[3] = {-lo[0], -lo[1], -lo[2]};
        const double scale = metersPerUnit * 1000.0;
        for (const auto& point : corners) {
            const double values[3] = {point.X() * scale, point.Y() * scale, point.Z() * scale};
            for (unsigned axis = 0; axis < 3; ++axis) {
                if (!std::isfinite(values[axis])) return false;
                lo[axis] = std::min(lo[axis], values[axis]); hi[axis] = std::max(hi[axis], values[axis]);
            }
        }
        result = {lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]};
        return lo[0] <= hi[0] && lo[1] <= hi[1] && lo[2] <= hi[2];
    } catch (...) { return false; }
}

bool MaterialWitness(Core3DSceneSnapshot *scene, NSString *entity,
                     const OcctScalarAppearanceState& appearance,
                     bool retainedScalarAvailable, Bytes& bytes) {
    bytes.text("shapeyard.publish-appearance.v1");
    bytes.u64(retainedScalarAvailable);
    bytes.u64(appearance.legacyPresent[0]); bytes.u64(appearance.legacyPresent[1]);
    bytes.u64(std::uint64_t(appearance.legacyValues[0])); bytes.u64(std::uint64_t(appearance.legacyValues[1]));
    bytes.u64(appearance.localPBR); bytes.u64(appearance.visualValues.size());
    for (double value : appearance.visualValues) bytes.number(value);
    TCollection_AsciiString entry;
    if (!appearance.materialLabel.IsNull()) TDF_Tool::Entry(appearance.materialLabel, entry);
    bytes.text(std::string(entry.ToCString() ? entry.ToCString() : ""));

    NSMutableIndexSet *materialIndexes = [NSMutableIndexSet indexSet];
    NSMutableIndexSet *textureIndexes = [NSMutableIndexSet indexSet];
    NSUInteger matches = 0;
    for (Core3DSceneRenderItemSnapshot *item in scene.renderItems) {
        if (![item.entityIdentifier isEqualToString:entity] || item.renderRole != Core3DSceneRenderRoleModel) continue;
        ++matches;
        for (Core3DScenePrimitiveBindingSnapshot *binding in item.primitiveBindings)
            [materialIndexes addIndex:binding.materialIndex];
        for (Core3DSceneFaceImageBindingSnapshot *binding in item.faceImageBindings) {
            if (binding.hasTexture && binding.textureIndex >= 0) [textureIndexes addIndex:NSUInteger(binding.textureIndex)];
        }
    }
    if (matches != 1) return false;
    bytes.u64(materialIndexes.count);
    [materialIndexes enumerateIndexesUsingBlock:^(NSUInteger index, BOOL *stop) {
        if (index >= scene.materials.count) { bytes.good = false; *stop = YES; return; }
        Core3DSceneMaterialSnapshot *m = scene.materials[index];
        const simd_float4 baseColor = m.linearBaseColorRGBA;
        const simd_float3 emission = m.linearEmissionRGB;
        bytes.text(m.identifier); bytes.raw(&baseColor, sizeof(baseColor));
        bytes.raw(&emission, sizeof(emission));
        bytes.number(m.metallic); bytes.number(m.roughness); bytes.number(m.indexOfRefraction);
        bytes.u64(m.alphaMode); bytes.number(m.alphaCutoff); bytes.u64(m.cullMode);
        for (NSInteger texture : {m.baseColorTextureIndex, m.emissiveTextureIndex,
                                  m.metallicRoughnessTextureIndex, m.occlusionTextureIndex,
                                  m.normalTextureIndex}) {
            bytes.u64(texture < 0 ? UINT64_MAX : std::uint64_t(texture));
            if (texture >= 0) [textureIndexes addIndex:NSUInteger(texture)];
        }
    }];
    bytes.u64(textureIndexes.count);
    [textureIndexes enumerateIndexesUsingBlock:^(NSUInteger index, BOOL *stop) {
        if (index >= scene.textures.count) { bytes.good = false; *stop = YES; return; }
        Core3DSceneTextureSnapshot *texture = scene.textures[index];
        bytes.text(texture.identifier); bytes.u64(texture.encoding); bytes.u64(texture.pixelWidth);
        bytes.u64(texture.pixelHeight); bytes.u64(texture.isPaintedAtlasDerivative); bytes.data(texture.encodedData);
    }];
    return bytes.good;
}

bool GroupWitness(const Handle(OcctDocument)& owner, const TDF_Label& label, Bytes& bytes) {
    OcctSavedGroupState groups;
    if (!owner->CaptureSavedGroups(groups)) return false;
    std::vector<const OcctSavedGroup *> memberships;
    for (const auto& group : groups.groups)
        for (const auto& member : group.members) if (member.IsEqual(label)) { memberships.push_back(&group); break; }
    std::sort(memberships.begin(), memberships.end(), [](const auto *a, const auto *b) { return a->identifier < b->identifier; });
    bytes.u64(memberships.size());
    for (const auto *group : memberships) {
        bytes.text(group->identifier);
        const TCollection_AsciiString name(group->name, '?'); bytes.text(std::string(name.ToCString()));
        bytes.u64(group->originPresent);
        if (group->originPresent) { bytes.number(group->origin.X()); bytes.number(group->origin.Y()); bytes.number(group->origin.Z()); }
    }
    return bytes.good;
}

bool SceneOwnerHasTexture(Core3DSceneSnapshot *scene, NSString *entity) {
    NSUInteger matches = 0;
    bool found = false;
    for (Core3DSceneRenderItemSnapshot *item in scene.renderItems) {
        if (![item.entityIdentifier isEqualToString:entity]
            || item.renderRole != Core3DSceneRenderRoleModel) continue;
        ++matches;
        for (Core3DScenePrimitiveBindingSnapshot *binding in item.primitiveBindings) {
            if (binding.materialIndex >= scene.materials.count) return false;
            Core3DSceneMaterialSnapshot *material = scene.materials[binding.materialIndex];
            if (material.hasBaseColorTexture || material.hasEmissiveTexture
                || material.hasMetallicRoughnessTexture || material.hasOcclusionTexture
                || material.hasNormalTexture) found = true;
        }
        for (Core3DSceneFaceImageBindingSnapshot *binding in item.faceImageBindings)
            if (binding.hasTexture) found = true;
    }
    return matches == 1 && found;
}

bool CaptureTexturedProgramSource(const Handle(OcctDocument)& owner, const TDF_Label& label,
                                  Core3DSceneSnapshot *scene, NSString *entity,
                                  OcctCylindricalCutProgramSource& output) {
    output = {};
    if (!SceneOwnerHasTexture(scene, entity)) return false;
    OcctObjectTransformState state;
    if (!owner->CaptureObjectTransformStateForLabel(label, state)
        || state.resolvedRepresentation != OcctGeometryRepresentation::BRep
        || state.authoredFramesPresent || state.meshUVAtlasVersion
        || state.shape.IsNull() || state.shape.ShapeType() != TopAbs_SOLID
        || state.shape.Orientation() != TopAbs_FORWARD) return false;
    const unsigned families = unsigned(!state.profile.label.IsNull())
        + unsigned(!state.enclosure.label.IsNull()) + unsigned(!state.sweep.label.IsNull())
        + unsigned(!state.loft.label.IsNull()) + unsigned(bool(state.retained.value));
    if (families != 1 || !state.retained.value) return false;
    output.original = state;
    output.recipe = state.retained.value->envelope;
    output.recipeBytes = state.retained.value->bytes;
    output.base = state.retained.value->base;
    std::vector<std::uint8_t> exact;
    double scale = 0;
    if (!core3d::retained_boolean::Encode(output.recipe, exact)
        || exact != output.recipeBytes || output.base.IsNull()
        || output.base.ShapeType() != TopAbs_SOLID
        || !core3d::cylindrical_cut::EffectiveMM(
            state.transform, core3d::retained_boolean::Identities(output.recipe).metersPerUnit,
            scale, output.effectiveMM)) {
        output = {}; return false;
    }
    return true;
}

bool CaptureOwner(const Handle(OcctDocument)& owner, Core3DSceneSnapshot *scene,
                  NSString *entity, std::size_t& geometryBudget,
                  OwnerObservation& output, Core3DPublishCaptureError& refusal) {
    refusal = Core3DPublishCaptureErrorUnsupportedOwner;
    TDF_Label label;
    if (!RootForEntity(owner, entity, label, refusal)) return false;
    OcctCylindricalCutProgramSource program;
    OcctCylindricalCutSource source;
    Recipe recipe;
    std::vector<std::uint8_t> recipeBytes;
    OcctObjectTransformState state;
    TopoDS_Shape base;
    double effectiveMM = 0;
    if (owner->CaptureCylindricalCutProgramSource(label, program)
        || CaptureTexturedProgramSource(owner, label, scene, entity, program)) {
        recipe = program.recipe; recipeBytes = program.recipeBytes; state = program.original;
        base = program.base; effectiveMM = program.effectiveMM;
    } else if (owner->CaptureCylindricalCutSource(label, source)
               && source.envelope.sourceFamily == 2) {
        recipe = source.envelope;
        if (!core3d::retained_boolean::Encode(recipe, recipeBytes)) { refusal = Core3DPublishCaptureErrorMissingRecipe; return false; }
        state = source.original; base = source.base; effectiveMM = source.effectiveMM;
    } else { refusal = Core3DPublishCaptureErrorMissingRecipe; return false; }

    const auto identities = core3d::retained_boolean::Identities(recipe);
    if (![entity isEqualToString:UUIDText(identities.entity)]
        || owner->DefinitionIdentifierForLabel(label) != UUIDText(identities.definition).UTF8String
        || owner->DocumentIdentifier() != UUIDText(identities.document).UTF8String) {
        refusal = Core3DPublishCaptureErrorAmbiguousRecipe; return false;
    }
    OcctScalarAppearanceState appearance;
    const bool retainedScalarAvailable = owner->CaptureScalarAppearanceForSavedCut(label, appearance);
    std::atomic_bool stop(false);
    core3d::saved_cut_source_edit::ShapeCommitment shapeCommitment, baseCommitment;
    if (!core3d::saved_cut_source_edit::Commit(state.shape, stop, geometryBudget, shapeCommitment)
        || !core3d::saved_cut_source_edit::Commit(base, stop, geometryBudget, baseCommitment)) {
        refusal = Core3DPublishCaptureErrorBudget; return false;
    }
    std::array<double, 6> bounds;
    if (!NativeBounds(state, scene.metersPerUnit, bounds)) { refusal = Core3DPublishCaptureErrorBounds; return false; }

    Bytes material;
    if (!MaterialWitness(scene, entity, appearance, retainedScalarAvailable, material)) {
        refusal = Core3DPublishCaptureErrorUnsupportedAppearance; return false;
    }
    NSData *materialBytes = material.finish();
    if (!materialBytes) { refusal = Core3DPublishCaptureErrorBudget; return false; }

    Bytes witness; witness.text("shapeyard.publish-owner-witness.v1");
    witness.text(owner->DocumentIdentifier()); witness.text(entity);
    witness.text(owner->DefinitionIdentifierForLabel(label)); witness.text(UUIDText(identities.sourceFeature));
    witness.text(UUIDText(identities.derivedFeature)); witness.number(identities.metersPerUnit);
    witness.field(recipeBytes.data(), recipeBytes.size()); witness.number(effectiveMM);
    witness.u64(shapeCommitment.bytes); witness.raw(shapeCommitment.sha256.data(), shapeCommitment.sha256.size());
    witness.u64(baseCommitment.bytes); witness.raw(baseCommitment.sha256.data(), baseCommitment.sha256.size());
    for (double scalar : state.scalars) witness.number(scalar);
    for (bool present : state.present) witness.u64(present);
    const gp_Mat matrix = state.transform.VectorialPart(); const gp_XYZ translation = state.transform.TranslationPart();
    for (unsigned row = 1; row <= 3; ++row) for (unsigned column = 1; column <= 3; ++column) witness.number(matrix.Value(row, column));
    witness.number(translation.X()); witness.number(translation.Y()); witness.number(translation.Z());
    if (!GroupWitness(owner, label, witness)) { refusal = Core3DPublishCaptureErrorUnsupportedOwner; return false; }
    witness.data(materialBytes);
    for (double value : bounds) witness.number(value);
    NSData *envelope = witness.finish();
    if (!envelope) { refusal = Core3DPublishCaptureErrorBudget; return false; }

    Bytes operands; operands.text("shapeyard.publish-operands.v1"); operands.field(recipeBytes.data(), recipeBytes.size());
    if (const auto *legacy = std::get_if<core3d::retained_boolean::Legacy>(&recipe)) {
        operands.u64(1); operands.u64(legacy->operandID);
    } else {
        const auto& retained = std::get<core3d::retained_boolean::Program>(recipe);
        operands.u64(retained.steps.size());
        for (const auto& step : retained.steps) { operands.u64(step.operand.identifier); operands.u64(std::uint64_t(step.operand.kind)); }
        operands.u64(retained.filletSteps.size());
        for (const auto& step : retained.filletSteps) operands.u64(step.stepIdentifier);
    }
    NSData *operandBytes = operands.finish();
    if (!operandBytes) { refusal = Core3DPublishCaptureErrorBudget; return false; }

    output.entity = [entity copy]; output.definition = [UUIDText(identities.definition) copy];
    output.envelope = envelope; output.operand = operandBytes; output.boundsMM = bounds;
    return true;
}

bool CaptureObservation(Core3DViewController *controller, const Handle(OcctDocument)& owner,
                        NSArray<NSString *> *expectedSelection, Observation& output,
                        Core3DPublishCaptureError& refusal) {
    output = {}; refusal = Core3DPublishCaptureErrorBusy;
    Core3DSceneSnapshot *scene = [controller captureExportSceneSnapshot];
    if (!scene) return false;
    if (scene.selectionMode != Core3DSceneElementKindObject
        || scene.selection.selectedElements.count == 0
        || scene.selection.selectedElements.count > kMaximumOwners) {
        refusal = Core3DPublishCaptureErrorSelection; return false;
    }
    for (Core3DSceneMeshSnapshot *mesh in scene.meshes)
        if (mesh.geometryKind == Core3DSceneGeometryKindNativeC1Wire) {
            refusal = Core3DPublishCaptureErrorNativeC1Wire; return false;
        }
    NSMutableArray<NSString *> *selected = [NSMutableArray array];
    NSMutableSet<NSString *> *unique = [NSMutableSet set];
    for (Core3DSceneElementIdentifier *element in scene.selection.selectedElements) {
        if (element.kind != Core3DSceneElementKindObject || element.topologyIndex != 0
            || !Identifier(element.entityIdentifier) || [unique containsObject:element.entityIdentifier]) {
            refusal = Core3DPublishCaptureErrorSelection; return false;
        }
        [unique addObject:element.entityIdentifier]; [selected addObject:element.entityIdentifier];
    }
    [selected sortUsingSelector:@selector(compare:)];
    if (expectedSelection && ![selected isEqualToArray:expectedSelection]) {
        refusal = Core3DPublishCaptureErrorStale; return false;
    }
    std::size_t geometryBudget = 0;
    std::vector<OwnerObservation> observations; observations.reserve(selected.count);
    for (NSString *entity in selected) {
        OwnerObservation observation;
        if (!CaptureOwner(owner, scene, entity, geometryBudget, observation, refusal)) return false;
        observations.push_back(std::move(observation));
    }
    std::array<double, 6> unionBounds = observations.front().boundsMM;
    for (const auto& observation : observations) for (unsigned axis = 0; axis < 3; ++axis) {
        unionBounds[axis] = std::min(unionBounds[axis], observation.boundsMM[axis]);
        unionBounds[axis + 3] = std::max(unionBounds[axis + 3], observation.boundsMM[axis + 3]);
    }
    NSMutableArray<NSNumber *> *bounds = [NSMutableArray arrayWithCapacity:6];
    for (double value : unionBounds) [bounds addObject:@(value)];
    NSMutableArray<NSData *> *operands = [NSMutableArray arrayWithCapacity:observations.size()];
    Bytes source; source.text("shapeyard.publish-source-witness.v1");
    source.text(owner->DocumentIdentifier()); source.text(scene.publicationSourceIdentifier);
    source.u64(scene.revisions.documentGeneration); source.u64(scene.revisions.modelRevision);
    source.u64(scene.revisions.presentationRevision); source.number(scene.metersPerUnit);
    source.text("native-rh-z-up_to_gltf-rh-y-up.v1"); source.u64(observations.size());
    for (const auto& observation : observations) { source.data(observation.envelope); [operands addObject:observation.operand]; }
    for (double value : unionBounds) source.number(value);
    NSData *sourceBytes = source.finish();
    if (!sourceBytes) { refusal = Core3DPublishCaptureErrorBudget; return false; }
    output.scene = scene; output.document = [NSString stringWithUTF8String:owner->DocumentIdentifier().c_str()] ?: @"";
    output.selected = selected; output.bounds = bounds; output.source = sourceBytes; output.operands = operands;
    return true;
}
} // namespace

@interface Core3DPublishCapture ()
@property(nonatomic, strong, readwrite) Core3DSceneSnapshot *sceneSnapshot;
@property(nonatomic, copy, readwrite) NSData *sourceWitnessBytes;
@property(nonatomic, copy, readwrite) NSArray<NSData *> *operandWitnessBytes;
@property(nonatomic, copy, readwrite) NSArray<NSString *> *selectedEntityIdentifiers;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *nativeBoundsMM;
@property(nonatomic, copy, readwrite) NSString *documentIdentifier;
@property(nonatomic, copy, readwrite) NSString *publicationSourceIdentifier;
@property(nonatomic, copy, readwrite) NSString *frameConvention;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@property(nonatomic, assign, readwrite) uint64_t documentGeneration;
@property(nonatomic, assign, readwrite) uint64_t modelRevision;
@property(nonatomic, assign, readwrite) uint64_t presentationRevision;
@end

@implementation Core3DPublishCapture {
    __weak Core3DViewController *_controller;
    Handle(OcctDocument) _owner;
    std::shared_ptr<OpeningContext> _context;
}

+ (instancetype)captureController:(Core3DViewController *)controller error:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !controller) {
        if (error) *error = Refusal(Core3DPublishCaptureErrorInvalidThread, @"main-thread controller required");
        return nil;
    }
    @try {
        GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)controller.glController : nil;
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl ? gl.viewer : nullptr;
        const Handle(OcctDocument) owner = viewer ? viewer->getDocument() : Handle(OcctDocument)();
        if (!viewer || owner.IsNull()) {
            if (error) *error = Refusal(Core3DPublishCaptureErrorBusy, @"native document unavailable"); return nil;
        }
        Observation observation; Core3DPublishCaptureError refusal;
        if (!CaptureObservation(controller, owner, nil, observation, refusal)) {
            if (error) *error = Refusal(refusal, @"native source observation refused"); return nil;
        }
        const CGSize drawable = controller.viewportDrawableSize;
        if (!std::isfinite(drawable.width) || !std::isfinite(drawable.height)
            || drawable.width < 1 || drawable.height < 1 || drawable.width > UINT32_MAX || drawable.height > UINT32_MAX) {
            if (error) *error = Refusal(Core3DPublishCaptureErrorBusy, @"viewport unavailable"); return nil;
        }
        std::vector<std::string> receipts;
        for (NSString *entity in observation.selected) receipts.emplace_back(entity.UTF8String ?: "");
        auto context = viewer->captureNativeOpeningContext(std::uint32_t(drawable.width), std::uint32_t(drawable.height), receipts);
        if (!context || !context->isCurrent(64, 64)) {
            if (error) *error = Refusal(Core3DPublishCaptureErrorStale, @"opening fence changed"); return nil;
        }
        Core3DPublishCapture *capture = [[self alloc] initPrivate];
        capture->_controller = controller; capture->_owner = owner; capture->_context = std::move(context);
        capture.sceneSnapshot = observation.scene; capture.sourceWitnessBytes = observation.source;
        capture.operandWitnessBytes = observation.operands; capture.selectedEntityIdentifiers = observation.selected;
        capture.nativeBoundsMM = observation.bounds; capture.documentIdentifier = observation.document;
        capture.publicationSourceIdentifier = observation.scene.publicationSourceIdentifier;
        capture.frameConvention = @"native-rh-z-up_to_gltf-rh-y-up.v1";
        capture.metersPerUnit = observation.scene.metersPerUnit;
        capture.documentGeneration = observation.scene.revisions.documentGeneration;
        capture.modelRevision = observation.scene.revisions.modelRevision;
        capture.presentationRevision = observation.scene.revisions.presentationRevision;
        return capture;
    } @catch (__unused NSException *exception) {
        if (error) *error = Refusal(Core3DPublishCaptureErrorUnsupportedOwner, @"native capture exception");
        return nil;
    }
}

- (instancetype)initPrivate { return [super init]; }

- (BOOL)isCurrent:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !_controller) {
        if (error) *error = Refusal(Core3DPublishCaptureErrorInvalidThread, @"main-thread live controller required"); return NO;
    }
    if (!_context || !_context->isCurrent(64, 64)) {
        if (error) *error = Refusal(Core3DPublishCaptureErrorStale, @"opening fence changed"); return NO;
    }
    Observation fresh; Core3DPublishCaptureError refusal = Core3DPublishCaptureErrorStale;
    if (!CaptureObservation(_controller, _owner, self.selectedEntityIdentifiers, fresh, refusal)
        || ![fresh.document isEqualToString:self.documentIdentifier]
        || ![fresh.source isEqualToData:self.sourceWitnessBytes]
        || ![fresh.operands isEqualToArray:self.operandWitnessBytes]
        || ![fresh.bounds isEqualToArray:self.nativeBoundsMM]) {
        if (error) *error = Refusal(refusal == Core3DPublishCaptureErrorBusy ? Core3DPublishCaptureErrorStale : refusal,
                                    @"captured dependency changed");
        return NO;
    }
    return YES;
}

@end
