#import "Core3DModelingTypes.h"
#import "Core3DViewController.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

// E3 face-image native opening (278b portion 3). Mirrors the row-268/row-278a
// opening boundary: captured authority only, viewer-owned lease, one OCAF
// command per committed mutation, authoritative terminal outcome and exact
// recovery. Bind Edit uses only the seven transform fields; Set uses those
// plus resourceIdentifier/role/colorSpace; Remove has its own typed method.
// Face/owner/generation/proof remain captured authority, never candidate
// overrides. The dependent-replay collaborator (bijective same-command
// reattachment) lives in NativeOpeningDependentReplay.hxx.
#include "../OCCTKit/NativeOpeningContext.hxx"
#include "../OCCTKit/NativeOpeningDependentReplay.hxx"
#include "../OCCTKit/NativeDocumentSession.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/FaceImageResourceValidation.hxx"
#include "../OCCTKit/DecalLayerPersistence.hxx"
#include "../OCCTKit/ReceiptRecord.hxx"
#include "../OCCTKit/RetainedSolidAttribute.hxx"
#include "Core3DViewer.h"

#include <ImageIO/ImageIO.h>
#include <Image_Texture.hxx>

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#if DEBUG
// DEBUG fixture construction only: one literal retained box with a real
// retained-solid record, one adopted E3 image resource and one committed
// face-image binding, mirroring the row-278a atlas fixture seam.
#include "../OCCTKit/NativeOpeningSurfaceProbe.hxx"
#include "../OCCTKit/PatternPersistence.hxx"
#include "../OCCTKit/ProfilePersistence.hxx"
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDocStd_FormatVersion.hxx>
#include <TDocStd_Application.hxx>
#include <PCDM_StoreStatus.hxx>
#include <Standard_Failure.hxx>
#include <functional>
#include <limits>
#include <stdexcept>
#endif

namespace {
using OwnerKey = core3d::retained_recipe::OwnerKey;
using UUID = core3d::retained_recipe::UUID;
using OpeningContext = core3d::native_opening::Context;
namespace fi = core3d::face_image;
namespace dr = core3d::dependent_replay;

enum class FaceImageOpeningState : std::uint8_t {
    Open, Prepared, Applying, Regenerating, Removing, Cancelled, Settled, Recovery
};

struct ControllerFaceImageInput final {
    Handle(OcctDocument) owner;
    std::shared_ptr<OpeningContext> context;
    OwnerKey key;
    std::string selected;
    bool hasFace = false;        // a live face was captured this session
    bool faceSupported = false;  // the captured face is planar and B2-addressable
    fi::UUID face{};             // durable face identity (existing or minted)
    dr::FaceImageFaceState faceState;  // current receipt/proof when supported
    bool hasFocus = false;       // an existing committed binding is focused
    fi::Binding focused;
};

NSString *IdentifierText(const UUID& value) {
    return [NSString stringWithUTF8String:core3d::retained_solid::UUIDText(value).c_str()] ?: @"";
}

NSString *RoleText(fi::Role value) {
    switch (value) {
        case fi::Role::BaseColor: return @"baseColor";
        case fi::Role::Emissive: return @"emissive";
        case fi::Role::MetallicRoughness: return @"metallicRoughness";
        case fi::Role::Occlusion: return @"occlusion";
        case fi::Role::Normal: return @"normal";
    }
    return @"baseColor";
}

bool RoleForString(id value, fi::Role& output) {
    if (![value isKindOfClass:NSString.class]) return false;
    NSString *text = (NSString *)value;
    if ([text isEqualToString:@"baseColor"]) { output = fi::Role::BaseColor; return true; }
    if ([text isEqualToString:@"emissive"]) { output = fi::Role::Emissive; return true; }
    if ([text isEqualToString:@"metallicRoughness"]) { output = fi::Role::MetallicRoughness; return true; }
    if ([text isEqualToString:@"occlusion"]) { output = fi::Role::Occlusion; return true; }
    if ([text isEqualToString:@"normal"]) { output = fi::Role::Normal; return true; }
    return false;
}

NSString *ColorSpaceText(fi::ColorSpace value) {
    return value == fi::ColorSpace::SRGB ? @"srgb" : @"linear";
}

bool ColorSpaceForString(id value, fi::ColorSpace& output) {
    if (![value isKindOfClass:NSString.class]) return false;
    NSString *text = (NSString *)value;
    if ([text isEqualToString:@"srgb"]) { output = fi::ColorSpace::SRGB; return true; }
    if ([text isEqualToString:@"linear"]) { output = fi::ColorSpace::Linear; return true; }
    return false;
}

NSString *WrapText(fi::Wrap value) {
    switch (value) {
        case fi::Wrap::ClampToEdge: return @"clampToEdge";
        case fi::Wrap::Repeat: return @"repeat";
        case fi::Wrap::MirroredRepeat: return @"mirroredRepeat";
    }
    return @"repeat";
}

bool WrapForString(id value, fi::Wrap& output) {
    if (![value isKindOfClass:NSString.class]) return false;
    NSString *text = (NSString *)value;
    if ([text isEqualToString:@"clampToEdge"]) { output = fi::Wrap::ClampToEdge; return true; }
    if ([text isEqualToString:@"repeat"]) { output = fi::Wrap::Repeat; return true; }
    if ([text isEqualToString:@"mirroredRepeat"]) { output = fi::Wrap::MirroredRepeat; return true; }
    return false;
}

bool Scalar(id value, double minimum, double maximum, bool minimumExclusive,
            double& output) {
    if (![value isKindOfClass:NSNumber.class]
        || CFGetTypeID((__bridge CFTypeRef)value) == CFBooleanGetTypeID()) return false;
    const double scalar = [value doubleValue];
    if (!std::isfinite(scalar) || scalar > maximum) return false;
    if (minimumExclusive ? scalar <= minimum : scalar < minimum) return false;
    output = scalar;
    return true;
}

bool Rotation(id value, double& output) {
    if (![value isKindOfClass:NSNumber.class]
        || CFGetTypeID((__bridge CFTypeRef)value) == CFBooleanGetTypeID()) return false;
    const double scalar = [value doubleValue];
    if (!std::isfinite(scalar) || scalar < 0.0 || scalar >= 360.0) return false;
    output = scalar;
    return true;
}

bool ParseIdentifier(id value, UUID& output) {
    output = {};
    if (![value isKindOfClass:NSString.class]) return false;
    const char *raw = [(NSString *)value UTF8String];
    const std::string text = raw ? raw : "";
    return !text.empty() && text.size() <= 128 && core3d::receipt::ParseUUID(text, output)
        && core3d::retained_recipe::Nonzero(output);
}

//! Exact host/selection-resolved owner key and label, mirroring the row-278a
//! KeyForSelected. The entity text is authority captured by the caller (the
//! scene selection or the host-resolved Objects row), never response data.
bool LabelForSelected(const Handle(OcctDocument)& owner, const std::string& selected,
                      OwnerKey& key, TDF_Label& label) noexcept {
    key = {}; label = TDF_Label();
    try {
        if (owner.IsNull() || selected.empty()) return false;
        const auto document = owner->Document();
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return false;
        TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
        TDF_Label match; unsigned count = 0;
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            if (owner->EntityIdentifierForLabel(roots.Value(index)) == selected) {
                match = roots.Value(index); ++count;
            }
        }
        if (count != 1 || match.IsNull()) return false;
        if (!core3d::receipt::ParseUUID(owner->DocumentIdentifier(), key.document)
            || !core3d::receipt::ParseUUID(owner->EntityIdentifierForLabel(match), key.entity)
            || !core3d::receipt::ParseUUID(owner->DefinitionIdentifierForLabel(match), key.definition)
            || !core3d::retained_recipe::Valid(key)) return false;
        label = match;
        return true;
    } catch (...) { key = {}; label = TDF_Label(); return false; }
}

bool MintUUID(UUID& output) {
    output = {};
    const char *minted = NSUUID.UUID.UUIDString.UTF8String;
    return minted && core3d::receipt::ParseUUID(minted, output)
        && core3d::retained_recipe::Nonzero(output);
}

//! Capture the picked face of one entity as B2 authority: the transient mesh
//! ordinal only locates the live TopoDS face; the durable identity is the
//! current B2 receipt proof. An already-committed binding for the identical
//! proof keeps its durable face UUID (bijective retention); a new face mints
//! one at capture and publishes it only at commit.
bool CapturePickedFace(const Handle(OcctDocument)& owner, const OwnerKey& key,
                       const TDF_Label& label, uint32_t topologyIndex,
                       ControllerFaceImageInput& input) noexcept {
    try {
        const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(label);
        if (shape.IsNull()) return false;
        TopTools_IndexedMapOfShape faceMap;
        TopExp::MapShapes(shape, TopAbs_FACE, faceMap);
        if (topologyIndex >= static_cast<uint32_t>(faceMap.Extent())) return false;
        const TopoDS_Face face = TopoDS::Face(faceMap.FindKey(Standard_Integer(topologyIndex) + 1));
        input.hasFace = true;
        double metersPerLocalUnit = 0;
        if (!dr::detail::FaceImageLengthUnit(*owner, metersPerLocalUnit)) return false;
        Bnd_Box stageBox;
        if (!dr::detail::FaceImageStageBounds(shape, stageBox)) return false;
        core3d::retained_edge_treatment::ReplayBudget budget;
        dr::FaceImageGeometricReceipt derived;
        const auto derived_status = dr::detail::DeriveFaceImageReceipt(
            stageBox, face, metersPerLocalUnit, budget, derived);
        if (derived_status == dr::detail::FaceImageDeriveStatus::Budget) return false;
        if (derived_status != dr::detail::FaceImageDeriveStatus::Derived) {
            // Curved, oblique or interior faces stay captured but unsupported;
            // preparation refuses them with UnsupportedSurface.
            input.faceSupported = false;
            return true;
        }
        dr::FaceImageGeometricReceipt resolved; TopoDS_Face matched;
        const auto refusal = dr::detail::ResolveFaceImageReceipt(
            shape, derived.intent, metersPerLocalUnit, budget, resolved, matched);
        if (refusal != core3d::retained_face_selector::Refusal::None
            || !matched.IsSame(face)) return false;
        fi::Digest proof{};
        if (!dr::FaceImageReceiptProof(resolved, proof)) return false;
        input.faceState.face = {};
        input.faceState.proof = proof;
        input.faceState.receipt = resolved;
        input.faceSupported = true;
        // Retain the durable face identity of an already-committed binding with
        // the identical current proof; focus that binding for Edit/Remove.
        fi::Definition committed;
        const auto state = owner->ReadFaceImageBindings(key, committed, nullptr);
        if (state == fi::persistence::bindings::ReadState::Malformed) return false;
        if (state == fi::persistence::bindings::ReadState::Present) {
            for (const auto& binding : committed.bindings) {
                if (binding.selectorProof == proof) {
                    input.face = binding.face;
                    input.faceState.face = binding.face;
                    input.hasFocus = true;
                    input.focused = binding;
                    return true;
                }
            }
        }
        if (!MintUUID(input.face)) return false;
        input.faceState.face = input.face;
        return true;
    } catch (...) { return false; }
}

bool CaptureControllerFaceImageInput(Core3DViewController *controller,
                                     NSString *entityIdentifier,
                                     ControllerFaceImageInput& output) noexcept {
    output = {};
    try {
        if (!NSThread.isMainThread || !controller) return false;
        GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)controller.glController : nil;
        if (!gl) return false;
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl.viewer;
        const CGSize drawable = controller.viewportDrawableSize;
        if (!viewer || !std::isfinite(drawable.width) || !std::isfinite(drawable.height)
            || drawable.width < 1 || drawable.height < 1
            || drawable.width > UINT32_MAX || drawable.height > UINT32_MAX) return false;
        output.owner = viewer->getDocument();
        if (output.owner.IsNull() || output.owner->Document().IsNull()) return false;
        Core3DSceneSnapshot *scene = [controller captureSceneSnapshot];
        if (!scene) return false;
        Core3DSceneElementIdentifier *faceElement = nil;
        std::string selected;
        if (entityIdentifier != nil) {
            // Host-resolved Objects row: the entity is the row's entity; a
            // currently picked face of the same entity is captured with it.
            if (entityIdentifier.length == 0 || entityIdentifier.length > 128) return false;
            const char *raw = entityIdentifier.UTF8String;
            selected = raw ? raw : "";
            if (scene.selection.selectedElements.count == 1) {
                Core3DSceneElementIdentifier *element = scene.selection.selectedElements.firstObject;
                if (element.kind == Core3DSceneElementKindFace
                    && [element.entityIdentifier isEqualToString:entityIdentifier])
                    faceElement = element;
                else if (element.kind == Core3DSceneElementKindObject
                         && ![element.entityIdentifier isEqualToString:entityIdentifier])
                    return false;
            }
        } else {
            if (scene.selection.selectedElements.count != 1) return false;
            Core3DSceneElementIdentifier *element = scene.selection.selectedElements.firstObject;
            if (element.entityIdentifier.length == 0 || element.entityIdentifier.length > 128)
                return false;
            const char *raw = element.entityIdentifier.UTF8String;
            selected = raw ? raw : "";
            if (element.kind == Core3DSceneElementKindFace) faceElement = element;
            else if (element.kind != Core3DSceneElementKindObject) return false;
        }
        TDF_Label label;
        if (!LabelForSelected(output.owner, selected, output.key, label)) return false;
        output.selected = selected;
        if (faceElement != nil) {
            if (!CapturePickedFace(output.owner, output.key, label,
                                   faceElement.topologyIndex, output)) return false;
        } else {
            // Whole-object focus: exactly one committed binding may be focused
            // for Edit/Remove; a bind always requires a picked face.
            fi::Definition committed;
            const auto state = output.owner->ReadFaceImageBindings(output.key, committed, nullptr);
            if (state == fi::persistence::bindings::ReadState::Malformed) return false;
            if (state == fi::persistence::bindings::ReadState::Present
                && committed.bindings.size() == 1) {
                output.hasFocus = true;
                output.focused = committed.bindings.front();
                output.face = output.focused.face;
                output.hasFace = true;
                output.faceState.face = output.focused.face;
                output.faceState.proof = output.focused.selectorProof;
            }
        }
        output.context = viewer->captureNativeOpeningContext(
            std::uint32_t(drawable.width), std::uint32_t(drawable.height), {selected});
        return output.context && output.context->isCurrent(64, 64);
    } catch (...) { output = {}; return false; }
}

//! Shared observed-fence construction for prepare/apply/remove. Every
//! candidate binding's face must end up fenced with its exact proof and every
//! bound resource must be present in the live manifest.
bool BuildObserved(const Handle(OcctDocument)& owner, const OwnerKey& key,
                   bool hadRecord, const fi::Definition& candidate,
                   bool addCapturedFace, const dr::FaceImageFaceState& captured,
                   fi::Observed& observed, dr::FaceImageReplayStatus& status) noexcept {
    observed = {};
    status = dr::FaceImageReplayStatus::Malformed;
    try {
        if (hadRecord) {
            dr::FaceImageAttachment attachment;
            status = dr::CaptureFaceImageAttachment(*owner, key, attachment);
            if (status != dr::FaceImageReplayStatus::Captured) return false;
            observed = attachment.observed;
        } else if (!owner->FaceImageResourceManifest(observed.resources)) {
            status = dr::FaceImageReplayStatus::Malformed;
            return false;
        }
        if (addCapturedFace)
            observed.faces.push_back({captured.face, captured.proof});
        for (const auto& binding : candidate.bindings) {
            bool fencedFace = false;
            for (const auto& fence : observed.faces)
                if (fence.face == binding.face && fence.selectorProof == binding.selectorProof) {
                    fencedFace = true; break;
                }
            if (!fencedFace) { status = dr::FaceImageReplayStatus::StaleFace; return false; }
            bool fencedResource = false;
            for (const auto& fence : observed.resources)
                if (fence.resource == binding.resource) { fencedResource = true; break; }
            if (!fencedResource) { status = dr::FaceImageReplayStatus::MissingResource; return false; }
        }
        status = dr::FaceImageReplayStatus::Captured;
        return true;
    } catch (...) { observed = {}; status = dr::FaceImageReplayStatus::Malformed; return false; }
}

Core3DFaceImagePreparationResult PreparationForOutcome(fi::owner::Outcome value) noexcept {
    using Outcome = fi::owner::Outcome;
    switch (value) {
        case Outcome::Prepared: return Core3DFaceImagePreparationResultPrepared;
        case Outcome::StaleSource: return Core3DFaceImagePreparationResultStaleSource;
        case Outcome::StaleFace: return Core3DFaceImagePreparationResultStaleFace;
        case Outcome::AmbiguousFaceRemap: return Core3DFaceImagePreparationResultAmbiguousFaceRemap;
        case Outcome::UnsupportedSurface: return Core3DFaceImagePreparationResultUnsupportedSurface;
        case Outcome::MissingResource: return Core3DFaceImagePreparationResultMissingResource;
        case Outcome::StaleResource: return Core3DFaceImagePreparationResultStaleResource;
        case Outcome::ForeignResource: return Core3DFaceImagePreparationResultForeignResource;
        case Outcome::UnsupportedDownstream: return Core3DFaceImagePreparationResultUnsupportedDownstream;
        default: return Core3DFaceImagePreparationResultRejected;
    }
}

Core3DFaceImagePreparationResult PreparationForReplayStatus(
    dr::FaceImageReplayStatus value) noexcept {
    switch (value) {
        case dr::FaceImageReplayStatus::StaleSource:
            return Core3DFaceImagePreparationResultStaleSource;
        case dr::FaceImageReplayStatus::StaleFace:
            return Core3DFaceImagePreparationResultStaleFace;
        case dr::FaceImageReplayStatus::AmbiguousFaceRemap:
            return Core3DFaceImagePreparationResultAmbiguousFaceRemap;
        case dr::FaceImageReplayStatus::UnsupportedSurface:
            return Core3DFaceImagePreparationResultUnsupportedSurface;
        case dr::FaceImageReplayStatus::MissingResource:
            return Core3DFaceImagePreparationResultMissingResource;
        case dr::FaceImageReplayStatus::UnsupportedDownstream:
            return Core3DFaceImagePreparationResultUnsupportedDownstream;
        default: return Core3DFaceImagePreparationResultRejected;
    }
}

void DeliverPreparation(void (^completion)(Core3DFaceImagePreparationResult,
                                            NSString *),
                        Core3DFaceImagePreparationResult result,
                        NSString *detail) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail);
    else dispatch_async(dispatch_get_main_queue(), ^{ completion(result, detail); });
}

void DeliverMutation(void (^completion)(Core3DProfileConstructionResult, NSString *),
                     Core3DProfileConstructionResult result, NSString *detail) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail);
    else dispatch_async(dispatch_get_main_queue(), ^{ completion(result, detail); });
}
} // namespace

@interface Core3DFaceImageOpening () {
@package
    Handle(OcctDocument) _owner;
    std::shared_ptr<OpeningContext> _context;
    OwnerKey _key;
    std::string _selected;
    bool _hasFace;
    bool _faceSupported;
    fi::UUID _face;
    dr::FaceImageFaceState _faceState;
    bool _hasFocus;
    fi::Binding _focused;
    bool _hadRecord;
    fi::Definition _candidate;
    fi::Observed _observed;
    fi::owner::Staging _staging;
    std::atomic<FaceImageOpeningState> _state;
}
- (instancetype)initWithInput:(ControllerFaceImageInput&&)input;
@end

@implementation Core3DFaceImageOpening
- (instancetype)initWithInput:(ControllerFaceImageInput&&)input {
    if ((self = [super init])) {
        _owner = input.owner; _context = std::move(input.context);
        _key = input.key; _selected = std::move(input.selected);
        _hasFace = input.hasFace; _faceSupported = input.faceSupported;
        _face = input.face; _faceState = input.faceState;
        _hasFocus = input.hasFocus; _focused = input.focused;
        _hadRecord = false;
        _state.store(FaceImageOpeningState::Open);
    }
    return self;
}

- (Core3DFaceImageCurrentness)currentness {
    if (!NSThread.isMainThread || _owner.IsNull() || !_hasFocus)
        return Core3DFaceImageCurrentnessAbsent;
    fi::Definition committed;
    const auto state = _owner->ReadFaceImageBindings(_key, committed, nullptr);
    if (state == fi::persistence::bindings::ReadState::Absent)
        return Core3DFaceImageCurrentnessAbsent;
    if (state != fi::persistence::bindings::ReadState::Present)
        return Core3DFaceImageCurrentnessStale;
    dr::FaceImageAttachment attachment;
    return dr::CaptureFaceImageAttachment(*_owner, _key, attachment)
            == dr::FaceImageReplayStatus::Captured
        ? Core3DFaceImageCurrentnessCurrent : Core3DFaceImageCurrentnessStale;
}

- (NSDictionary<NSString *, id> *)descriptor {
    if (!NSThread.isMainThread || _owner.IsNull()) return @{};
    @try {
        fi::Definition committed;
        const auto state = _owner->ReadFaceImageBindings(_key, committed, nullptr);
        bool focused = _hasFocus;
        fi::Binding binding = _focused;
        if (!focused && _hasFace
            && state == fi::persistence::bindings::ReadState::Present) {
            for (const auto& entry : committed.bindings)
                if (entry.face == _face) { focused = true; binding = entry; break; }
        }
        NSString *diagnosis = @"";
        if (_hasFace && !_faceSupported)
            diagnosis = @"The captured face is not a supported planar face.";
        else if (state == fi::persistence::bindings::ReadState::Malformed)
            diagnosis = @"The face-image record is malformed.";
        NSString *currentness = @"absent";
        switch (self.currentness) {
            case Core3DFaceImageCurrentnessCurrent: currentness = @"current"; break;
            case Core3DFaceImageCurrentnessStale: currentness = @"stale"; break;
            default: break;
        }
        const unsigned long long generation = _context
            ? (unsigned long long)_context->openingFence().documentGeneration() : 0;
        const fi::UVTransform transform = focused ? binding.transform : fi::UVTransform{};
        return @{@"schema": @"shapeyard.face-image.opening.v1",
            @"ownerEntityIdentifier": [NSString stringWithUTF8String:_selected.c_str()] ?: @"",
            @"faceIdentifier": focused || (_hasFace && _faceSupported) ? IdentifierText(_face) : @"",
            @"resourceIdentifier": focused ? IdentifierText(binding.resource) : @"",
            @"role": RoleText(focused ? binding.role : fi::Role::BaseColor),
            @"colorSpace": ColorSpaceText(focused ? binding.colorSpace : fi::ColorSpace::SRGB),
            @"scaleU": @(transform.scale[0]),
            @"scaleV": @(transform.scale[1]),
            @"offsetU": @(transform.offset[0]),
            @"offsetV": @(transform.offset[1]),
            @"rotationDegrees": @(transform.rotationDegrees),
            @"wrapU": WrapText(transform.wrapU),
            @"wrapV": WrapText(transform.wrapV),
            @"currentness": currentness,
            @"diagnosis": diagnosis,
            @"sourceDocumentGeneration": @(generation)};
    } @catch (...) { return @{}; }
}

- (void)prepareCandidate:(NSDictionary<NSString *, id> *)candidate
              completion:(void (^)(Core3DFaceImagePreparationResult,
                                   NSString *))completion {
    if (!NSThread.isMainThread || _state.load() != FaceImageOpeningState::Open
        || _owner.IsNull() || !_context) {
        DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                           @"Opening is not available for preparation.");
        return;
    }
    if (![candidate isKindOfClass:NSDictionary.class]) {
        DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                           @"The candidate must be a dictionary.");
        return;
    }
    NSSet *keys = [NSSet setWithArray:candidate.allKeys];
    NSSet *transformKeys = [NSSet setWithArray:@[
        @"scaleU", @"scaleV", @"offsetU", @"offsetV",
        @"rotationDegrees", @"wrapU", @"wrapV"]];
    NSSet *setKeys = [NSSet setWithArray:@[
        @"resourceIdentifier", @"role", @"colorSpace",
        @"scaleU", @"scaleV", @"offsetU", @"offsetV",
        @"rotationDegrees", @"wrapU", @"wrapV"]];
    const bool isSet = [keys isEqualToSet:setKeys];
    const bool isEdit = !isSet && [keys isEqualToSet:transformKeys];
    if (!isSet && !isEdit) {
        DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                           @"Set uses resourceIdentifier, role, colorSpace and the seven transform fields; Edit uses only the seven transform fields.");
        return;
    }
    fi::UVTransform transform;
    if (!Scalar(candidate[@"scaleU"], 0.0, 64.0, true, transform.scale[0])
        || !Scalar(candidate[@"scaleV"], 0.0, 64.0, true, transform.scale[1])
        || !Scalar(candidate[@"offsetU"], -4096.0, 4096.0, false, transform.offset[0])
        || !Scalar(candidate[@"offsetV"], -4096.0, 4096.0, false, transform.offset[1])
        || !Rotation(candidate[@"rotationDegrees"], transform.rotationDegrees)
        || !WrapForString(candidate[@"wrapU"], transform.wrapU)
        || !WrapForString(candidate[@"wrapV"], transform.wrapV)) {
        DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                           @"Use finite scale (0, 64], offset [-4096, 4096], rotation [0, 360) and a supported wrap mode.");
        return;
    }
    fi::Definition committed;
    const auto readState = _owner->ReadFaceImageBindings(_key, committed, nullptr);
    if (readState == fi::persistence::bindings::ReadState::Malformed) {
        DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                           @"The face-image record is malformed.");
        return;
    }
    const bool hadRecord = readState == fi::persistence::bindings::ReadState::Present;
    fi::Definition next = hadRecord ? committed : fi::Definition{};
    next.owner = _key;
    bool addCapturedFace = false;
    if (isEdit) {
        if (!_hasFocus) {
            DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                               @"There is no committed binding for this face to edit.");
            return;
        }
        bool found = false;
        for (auto& binding : next.bindings) {
            if (binding.binding == _focused.binding) {
                binding.transform = transform; found = true; break;
            }
        }
        if (!found) {
            DeliverPreparation(completion, Core3DFaceImagePreparationResultStaleFace,
                               @"The focused binding is no longer committed.");
            return;
        }
    } else {
        if (!_hasFace) {
            DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                               @"Select a planar face of this object to bind an image.");
            return;
        }
        if (!_faceSupported) {
            DeliverPreparation(completion, Core3DFaceImagePreparationResultUnsupportedSurface,
                               @"Only a planar, selector-addressable face can carry an image binding.");
            return;
        }
        fi::Role role = fi::Role::BaseColor;
        fi::ColorSpace colorSpace = fi::ColorSpace::SRGB;
        UUID resource{};
        if (!RoleForString(candidate[@"role"], role)
            || !ColorSpaceForString(candidate[@"colorSpace"], colorSpace)) {
            DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                               @"Use a supported role and color space.");
            return;
        }
        if (!fi::ValidColorSpaceForRole(role, colorSpace)) {
            DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                               @"Color roles require sRGB; numeric roles require linear.");
            return;
        }
        if (!ParseIdentifier(candidate[@"resourceIdentifier"], resource)) {
            DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                               @"The resource identifier is not a valid identity.");
            return;
        }
        fi::ResourceEnvelope envelope;
        if (!_owner->ReadFaceImageResource(resource, envelope)) {
            DeliverPreparation(completion, Core3DFaceImagePreparationResultMissingResource,
                               @"The image resource is not adopted in this document.");
            return;
        }
        bool replaced = false;
        for (auto& binding : next.bindings) {
            if (binding.face == _face && binding.role == role) {
                // One binding per face/role pair: Set updates it in place and
                // keeps every durable identity.
                binding.resource = resource;
                binding.colorSpace = colorSpace;
                binding.transform = transform;
                replaced = true;
                break;
            }
        }
        if (!replaced) {
            if (next.bindings.size() >= fi::kMaximumBindings) {
                DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                                   @"A face-image owner admits at most 256 bindings.");
                return;
            }
            fi::Binding created;
            if (!MintUUID(created.binding)) {
                DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                                   @"A durable binding identity could not be minted.");
                return;
            }
            created.face = _face;
            created.selectorProof = _faceState.proof;
            created.resource = resource;
            created.role = role;
            created.colorSpace = colorSpace;
            created.transform = transform;
            next.bindings.push_back(created);
            // A face new to the record joins the observed fence; a face
            // already bound under another role is fenced by the capture.
            addCapturedFace = !hadRecord;
            if (hadRecord) {
                bool known = false;
                for (const auto& binding : committed.bindings)
                    if (binding.face == _face) { known = true; break; }
                addCapturedFace = !known;
            }
        }
    }
    if (hadRecord && next == committed) {
        DeliverPreparation(completion, Core3DFaceImagePreparationResultUnchanged,
                           @"The face-image bindings already match these values.");
        return;
    }
    if (!fi::BindBindingProof(next)) {
        DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                           @"The candidate bindings could not be proven.");
        return;
    }
    fi::Observed observed;
    dr::FaceImageReplayStatus observedStatus = dr::FaceImageReplayStatus::Malformed;
    if (!BuildObserved(_owner, _key, hadRecord, next, addCapturedFace,
                       _faceState, observed, observedStatus)) {
        DeliverPreparation(completion, PreparationForReplayStatus(observedStatus),
                           @"The captured faces or resources are no longer current.");
        return;
    }
    _staging = fi::owner::Staging{};
    const fi::owner::Outcome outcome = _owner->PrepareFaceImageBindings(
        _staging, next, observed);
    if (outcome != fi::owner::Outcome::Prepared) {
        _owner->CancelFaceImageBindings(_staging);
        DeliverPreparation(completion, PreparationForOutcome(outcome),
                           @"The face-image edit was refused without history.");
        return;
    }
    FaceImageOpeningState expected = FaceImageOpeningState::Open;
    if (!_state.compare_exchange_strong(expected, FaceImageOpeningState::Prepared)) {
        _owner->CancelFaceImageBindings(_staging);
        DeliverPreparation(completion, Core3DFaceImagePreparationResultRejected,
                           @"Opening changed during preparation.");
        return;
    }
    _hadRecord = hadRecord;
    _candidate = std::move(next);
    _observed = std::move(observed);
    DeliverPreparation(completion, Core3DFaceImagePreparationResultPrepared,
                       @"Face image prepared. Apply is one undoable face-image change.");
}

- (void)applyWithCompletion:(void (^)(Core3DProfileConstructionResult,
                                      NSString *))completion {
    FaceImageOpeningState expected = FaceImageOpeningState::Prepared;
    if (!NSThread.isMainThread
        || !_state.compare_exchange_strong(expected, FaceImageOpeningState::Applying)
        || _owner.IsNull() || !_context) {
        DeliverMutation(completion, Core3DProfileConstructionResultRejected,
                        @"No current face-image preparation.");
        return;
    }
    const auto lease = _context->beginCommandLease(_context->openingFence(), 64, 64);
    if (!lease) {
        _state.store(FaceImageOpeningState::Settled);
        DeliverMutation(completion, Core3DProfileConstructionResultBusy,
                        @"The face image edit could not begin a native command.");
        return;
    }
    Core3DProfileConstructionResult result = Core3DProfileConstructionResultRejected;
    NSString *detail = @"The face image edit was refused without history.";
    for (;;) {
        // Re-fence against the actual current state inside the command: the
        // committed record, every bound face and the live manifest must be
        // exactly what preparation fenced.
        bool addCapturedFace = false;
        if (_hasFace && _faceSupported && !_hasFocus) {
            fi::Definition fresh;
            const auto state = _owner->ReadFaceImageBindings(_key, fresh, nullptr);
            bool known = false;
            if (state == fi::persistence::bindings::ReadState::Present)
                for (const auto& binding : fresh.bindings)
                    if (binding.face == _face) { known = true; break; }
            addCapturedFace = !known;
        }
        fi::Observed observed;
        dr::FaceImageReplayStatus observedStatus = dr::FaceImageReplayStatus::Malformed;
        if (!BuildObserved(_owner, _key, _hadRecord, _candidate, addCapturedFace,
                           _faceState, observed, observedStatus)
            || !(observed == _observed)) {
            detail = @"Source changed after preparation; the edit was refused without history.";
            break;
        }
        const fi::owner::Outcome outcome = _owner->CommitFaceImageBindings(_staging, observed);
        if (outcome == fi::owner::Outcome::Committed) {
            result = lease->commit() ? Core3DProfileConstructionResultCommitted
                                     : Core3DProfileConstructionResultRecoveryRequired;
            detail = result == Core3DProfileConstructionResultCommitted
                ? @"Face image committed as one undoable change."
                : @"Command close is unknown; native recovery ownership is retained.";
        } else {
            _owner->CancelFaceImageBindings(_staging);
            if (!lease->abort()) {
                result = Core3DProfileConstructionResultRecoveryRequired;
                detail = @"Command close is unknown; native recovery ownership is retained.";
            } else if (outcome == fi::owner::Outcome::Busy) {
                result = Core3DProfileConstructionResultBusy;
                detail = @"The face image edit could not begin a native command.";
            }
        }
        break;
    }
    if (result == Core3DProfileConstructionResultRejected && lease->ownsOpenCommand()) {
        if (!lease->abort()) {
            result = Core3DProfileConstructionResultRecoveryRequired;
            detail = @"Command close is unknown; native recovery ownership is retained.";
        }
    }
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? FaceImageOpeningState::Recovery : FaceImageOpeningState::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) _context.reset();
    DeliverMutation(completion, result, detail);
}

- (void)regenerateWithCompletion:(void (^)(Core3DProfileConstructionResult,
                                           NSString *))completion {
    FaceImageOpeningState value = _state.load();
    while (value == FaceImageOpeningState::Open || value == FaceImageOpeningState::Prepared) {
        if (_state.compare_exchange_weak(value, FaceImageOpeningState::Regenerating)) break;
    }
    if (!NSThread.isMainThread || _state.load() != FaceImageOpeningState::Regenerating
        || _owner.IsNull() || !_context) {
        DeliverMutation(completion, Core3DProfileConstructionResultRejected,
                        @"Opening is not available for regeneration.");
        return;
    }
    // Re-prove the committed record against the current shape through the
    // dependent-replay collaborator: exact proof recovery plus a full
    // bijective B2 re-resolution. Any lost, split, merged or ambiguous face
    // refuses without history; a current record reports Unchanged.
    dr::FaceImageAttachment attachment;
    const dr::FaceImageReplayStatus captured = dr::CaptureFaceImageAttachment(
        *_owner, _key, attachment);
    if (captured == dr::FaceImageReplayStatus::Absent) {
        _state.store(FaceImageOpeningState::Settled); _context.reset();
        DeliverMutation(completion, Core3DProfileConstructionResultRejected,
                        @"There is no committed face-image record to re-prove.");
        return;
    }
    if (captured != dr::FaceImageReplayStatus::Captured) {
        _state.store(FaceImageOpeningState::Settled); _context.reset();
        DeliverMutation(completion, Core3DProfileConstructionResultRejected,
            captured == dr::FaceImageReplayStatus::AmbiguousFaceRemap
                ? @"The committed faces no longer remap one-to-one; the record was preserved."
                : captured == dr::FaceImageReplayStatus::StaleFace
                    ? @"A committed face is stale after a source change; the record was preserved."
                    : @"The committed face-image record could not be re-proven.");
        return;
    }
    TDF_Label ownerLabel;
    const TopoDS_Shape currentShape = dr::detail::FaceImageOwnerShape(*_owner, _key, ownerLabel);
    dr::FaceImageReplay replay;
    const dr::FaceImageReplayStatus prepared = replay.prepare(
        *_owner, attachment, dr::Mutation::Replace, _key.entity, currentShape);
    if (prepared == dr::FaceImageReplayStatus::NoChange) {
        _state.store(FaceImageOpeningState::Settled); _context.reset();
        DeliverMutation(completion, Core3DProfileConstructionResultUnchanged,
                        @"Every committed face re-resolves one-to-one; the record is current.");
        return;
    }
    if (prepared != dr::FaceImageReplayStatus::Prepared) {
        _state.store(FaceImageOpeningState::Settled); _context.reset();
        DeliverMutation(completion, Core3DProfileConstructionResultRejected,
            prepared == dr::FaceImageReplayStatus::AmbiguousFaceRemap
                ? @"The committed faces no longer remap one-to-one; the record was preserved."
                : prepared == dr::FaceImageReplayStatus::UnsupportedDownstream
                    ? @"Face-image bindings never propagate to a Boolean or duplication result."
                    : @"The committed face-image record could not be re-proven.");
        return;
    }
    const auto lease = _context->beginCommandLease(_context->openingFence(), 64, 64);
    Core3DProfileConstructionResult result = Core3DProfileConstructionResultBusy;
    NSString *detail = @"The face image regeneration could not begin a native command.";
    if (lease) {
        const dr::FaceImageReplayStatus staged = replay.stage(*_owner);
        if (staged == dr::FaceImageReplayStatus::Prepared && replay.read(*_owner)) {
            result = lease->commit() ? Core3DProfileConstructionResultCommitted
                                     : Core3DProfileConstructionResultRecoveryRequired;
            detail = result == Core3DProfileConstructionResultCommitted
                ? @"Face image re-proven as one undoable change."
                : @"Command close is unknown; native recovery ownership is retained.";
        } else {
            replay.cancel();
            result = lease->abort() ? Core3DProfileConstructionResultRejected
                                    : Core3DProfileConstructionResultRecoveryRequired;
            detail = result == Core3DProfileConstructionResultRecoveryRequired
                ? @"Command close is unknown; native recovery ownership is retained."
                : @"Regeneration was refused without reusing a stale record.";
        }
    }
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? FaceImageOpeningState::Recovery : FaceImageOpeningState::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) _context.reset();
    DeliverMutation(completion, result, detail);
}

- (void)removeWithCompletion:(void (^)(Core3DProfileConstructionResult,
                                       NSString *))completion {
    FaceImageOpeningState value = _state.load();
    while (value == FaceImageOpeningState::Open || value == FaceImageOpeningState::Prepared) {
        if (_state.compare_exchange_weak(value, FaceImageOpeningState::Removing)) break;
    }
    if (!NSThread.isMainThread || _state.load() != FaceImageOpeningState::Removing
        || _owner.IsNull() || !_context) {
        DeliverMutation(completion, Core3DProfileConstructionResultRejected,
                        @"Opening is not available for removal.");
        return;
    }
    if (!_hasFocus) {
        _state.store(FaceImageOpeningState::Settled); _context.reset();
        DeliverMutation(completion, Core3DProfileConstructionResultRejected,
                        @"There is no committed binding for this face to remove.");
        return;
    }
    const auto lease = _context->beginCommandLease(_context->openingFence(), 64, 64);
    Core3DProfileConstructionResult result = Core3DProfileConstructionResultBusy;
    NSString *detail = @"The face image removal could not begin a native command.";
    if (lease) {
        for (;;) {
            fi::Definition committed;
            const auto readState = _owner->ReadFaceImageBindings(_key, committed, nullptr);
            if (readState != fi::persistence::bindings::ReadState::Present) {
                result = Core3DProfileConstructionResultRejected;
                detail = @"The face-image record is no longer committed.";
                break;
            }
            fi::Definition next = committed;
            bool found = false;
            for (auto iterator = next.bindings.begin(); iterator != next.bindings.end(); ++iterator) {
                if (iterator->binding == _focused.binding) {
                    next.bindings.erase(iterator); found = true; break;
                }
            }
            if (!found) {
                result = Core3DProfileConstructionResultRejected;
                detail = @"The focused binding is no longer committed.";
                break;
            }
            if (next.bindings.empty()) {
                // Last-binding removal retires the whole record.
                const fi::owner::Outcome removed = _owner->RemoveFaceImageBindings(_key);
                if (removed == fi::owner::Outcome::Committed) {
                    result = lease->commit() ? Core3DProfileConstructionResultCommitted
                                             : Core3DProfileConstructionResultRecoveryRequired;
                    detail = result == Core3DProfileConstructionResultCommitted
                        ? @"Face image binding removed as one undoable change."
                        : @"Command close is unknown; native recovery ownership is retained.";
                } else {
                    result = lease->abort() ? Core3DProfileConstructionResultRejected
                                            : Core3DProfileConstructionResultRecoveryRequired;
                    detail = result == Core3DProfileConstructionResultRecoveryRequired
                        ? @"Command close is unknown; native recovery ownership is retained."
                        : @"The face image removal was refused without history.";
                }
                break;
            }
            if (!fi::BindBindingProof(next)) {
                result = Core3DProfileConstructionResultRejected;
                break;
            }
            fi::Observed observed;
            dr::FaceImageReplayStatus observedStatus = dr::FaceImageReplayStatus::Malformed;
            if (!BuildObserved(_owner, _key, true, next, false,
                               _faceState, observed, observedStatus)) {
                result = Core3DProfileConstructionResultRejected;
                detail = @"The remaining faces or resources are no longer current.";
                break;
            }
            fi::owner::Staging staging;
            if (_owner->PrepareFaceImageBindings(staging, next, observed)
                    != fi::owner::Outcome::Prepared
                || _owner->CommitFaceImageBindings(staging, observed)
                    != fi::owner::Outcome::Committed) {
                _owner->CancelFaceImageBindings(staging);
                result = lease->abort() ? Core3DProfileConstructionResultRejected
                                        : Core3DProfileConstructionResultRecoveryRequired;
                detail = result == Core3DProfileConstructionResultRecoveryRequired
                    ? @"Command close is unknown; native recovery ownership is retained."
                    : @"The face image removal was refused without history.";
                break;
            }
            result = lease->commit() ? Core3DProfileConstructionResultCommitted
                                     : Core3DProfileConstructionResultRecoveryRequired;
            detail = result == Core3DProfileConstructionResultCommitted
                ? @"Face image binding removed as one undoable change."
                : @"Command close is unknown; native recovery ownership is retained.";
            break;
        }
        if (result == Core3DProfileConstructionResultRejected && lease->ownsOpenCommand()) {
            if (!lease->abort()) {
                result = Core3DProfileConstructionResultRecoveryRequired;
                detail = @"Command close is unknown; native recovery ownership is retained.";
            }
        }
    }
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? FaceImageOpeningState::Recovery : FaceImageOpeningState::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) _context.reset();
    DeliverMutation(completion, result, detail);
}

- (BOOL)cancel {
    FaceImageOpeningState value = _state.load();
    while (value == FaceImageOpeningState::Open || value == FaceImageOpeningState::Prepared) {
        if (_state.compare_exchange_weak(value, FaceImageOpeningState::Cancelled)) {
            if (!_owner.IsNull()) _owner->CancelFaceImageBindings(_staging);
            _staging = fi::owner::Staging{};
            _context.reset();
            return YES;
        }
    }
    return NO;
}
@end

// ---------------------------------------------------------------------------
// E3 trusted resource adoption (278b portion 4b). The app's importer hands
// over the original imported bytes and the normalized working bytes; this
// boundary rehashes both itself (a caller digest or a decoded manifest is
// never trusted), validates them with the existing bounded image validator
// (Core3DCreateAuthoredTexture: PNG/JPEG signature, single complete frame,
// 8192 per side, 16,777,216 pixels, 128 MiB decoded) and the importer caps,
// charges the single aggregate resource budget before allocation inside
// OcctDocument::AdoptFaceImageResource and adopts them as ONE ordinary
// undoable command under the existing document mutation owner (a native
// opening context command lease). The returned identifier is the runtime
// adoption capability a Set candidate's resourceIdentifier accepts. No URL,
// provider path, inline blob, request handle or TrustedInputAssetManifest can
// mint it.
namespace {

NSString *DigestText(const fi::Digest& value) {
    char text[65];
    static const char hex[] = "0123456789abcdef";
    for (std::size_t index = 0; index < value.size(); ++index) {
        text[2 * index] = hex[value[index] >> 4];
        text[2 * index + 1] = hex[value[index] & 15];
    }
    text[64] = 0;
    return [NSString stringWithUTF8String:text] ?: @"";
}

NSString *ImageFormatText(fi::ImageEncoding value) {
    return value == fi::ImageEncoding::JPEG ? @"jpeg" : @"png";
}

NSString *AlphaText(fi::AlphaInterpretation value) {
    return value == fi::AlphaInterpretation::Straight ? @"straight" : @"opaque";
}

// Capture the live document and one current native opening context for one
// ordinary undoable command under the existing document mutation owner.
bool CaptureFaceImageDocumentOwner(Core3DViewController *controller,
                                   Handle(OcctDocument)& owner,
                                   std::shared_ptr<OpeningContext>& context) noexcept {
    owner.Nullify();
    context.reset();
    try {
        if (!NSThread.isMainThread || !controller) return false;
        GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)controller.glController : nil;
        if (!gl) return false;
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl.viewer;
        const CGSize drawable = controller.viewportDrawableSize;
        if (!viewer || !std::isfinite(drawable.width) || !std::isfinite(drawable.height)
            || drawable.width < 1 || drawable.height < 1
            || drawable.width > UINT32_MAX || drawable.height > UINT32_MAX) return false;
        owner = viewer->getDocument();
        if (owner.IsNull() || owner->Document().IsNull()) return false;
        context = viewer->captureNativeOpeningContext(
            std::uint32_t(drawable.width), std::uint32_t(drawable.height), {});
        return context && context->isCurrent(64, 64);
    } catch (...) { owner.Nullify(); context.reset(); return false; }
}

void DeliverAdoption(void (^completion)(Core3DFaceImageResourceAdoptionResult,
                                        NSString * _Nullable, NSString *),
                     Core3DFaceImageResourceAdoptionResult result,
                     NSString *identifier, NSString *detail) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, identifier, detail);
    else dispatch_async(dispatch_get_main_queue(), ^{ completion(result, identifier, detail); });
}

void DeliverAdoption(void (^completion)(Core3DFaceImageResourceAdoptionResult, NSString *),
                     Core3DFaceImageResourceAdoptionResult result, NSString *detail) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail);
    else dispatch_async(dispatch_get_main_queue(), ^{ completion(result, detail); });
}

} // namespace

@interface Core3DFaceImageResourceSnapshot ()
- (instancetype)initWithResourceIdentifier:(NSString *)resourceIdentifier
                       originalContentSHA256:(NSString *)originalContentSHA256
                        workingContentSHA256:(NSString *)workingContentSHA256
                              originalFormat:(NSString *)originalFormat
                               workingFormat:(NSString *)workingFormat
                         alphaInterpretation:(NSString *)alphaInterpretation
                         originalWidthTexels:(NSUInteger)originalWidthTexels
                        originalHeightTexels:(NSUInteger)originalHeightTexels
                          workingWidthTexels:(NSUInteger)workingWidthTexels
                         workingHeightTexels:(NSUInteger)workingHeightTexels
                               originalBytes:(NSData *)originalBytes
                                workingBytes:(NSData *)workingBytes NS_DESIGNATED_INITIALIZER;
@end

@implementation Core3DFaceImageResourceSnapshot
- (instancetype)initWithResourceIdentifier:(NSString *)resourceIdentifier
                       originalContentSHA256:(NSString *)originalContentSHA256
                        workingContentSHA256:(NSString *)workingContentSHA256
                              originalFormat:(NSString *)originalFormat
                               workingFormat:(NSString *)workingFormat
                         alphaInterpretation:(NSString *)alphaInterpretation
                         originalWidthTexels:(NSUInteger)originalWidthTexels
                        originalHeightTexels:(NSUInteger)originalHeightTexels
                          workingWidthTexels:(NSUInteger)workingWidthTexels
                         workingHeightTexels:(NSUInteger)workingHeightTexels
                               originalBytes:(NSData *)originalBytes
                                workingBytes:(NSData *)workingBytes {
    if ((self = [super init])) {
        _resourceIdentifier = [resourceIdentifier copy];
        _originalContentSHA256 = [originalContentSHA256 copy];
        _workingContentSHA256 = [workingContentSHA256 copy];
        _originalFormat = [originalFormat copy];
        _workingFormat = [workingFormat copy];
        _alphaInterpretation = [alphaInterpretation copy];
        _originalWidthTexels = originalWidthTexels;
        _originalHeightTexels = originalHeightTexels;
        _workingWidthTexels = workingWidthTexels;
        _workingHeightTexels = workingHeightTexels;
        _originalBytes = [originalBytes copy];
        _workingBytes = [workingBytes copy];
    }
    return self;
}
@end

@implementation Core3DViewController (FaceImageResourceAdoption)

- (void)adoptFaceImageResourceWithOriginalBytes:(NSData *)originalBytes
                                  workingBytes:(NSData *)workingBytes
                           alphaInterpretation:(NSString *)alphaInterpretation
                                    provenance:(NSData *)provenance
                                    completion:(void (^)(Core3DFaceImageResourceAdoptionResult,
                                                         NSString * _Nullable,
                                                         NSString *))completion {
    if (!NSThread.isMainThread) {
        DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRejected, nil,
                        @"Face image resource adoption is main-thread only.");
        return;
    }
    @try {
        fi::UUID resource;
        fi::ResourceEnvelope envelope;
        Handle(OcctDocument) owner;
        std::shared_ptr<OpeningContext> context;
        if (!MintUUID(resource)) {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRejected, nil,
                            @"A durable resource identity could not be minted.");
            return;
        }
        if (!fi::validation::BuildFaceImageEnvelope(
                originalBytes, workingBytes, alphaInterpretation,
                provenance, resource, envelope)) {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRejected, nil,
                            @"The image bytes are not a supported bounded PNG/JPEG pair.");
            return;
        }
        if (!CaptureFaceImageDocumentOwner(self, owner, context)) {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRejected, nil,
                            @"There is no current document for adoption.");
            return;
        }
        const auto lease = context->beginCommandLease(context->openingFence(), 64, 64);
        if (!lease) {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultBusy, nil,
                            @"The adoption could not begin a native command.");
            return;
        }
        const fi::owner::Outcome outcome = owner->AdoptFaceImageResource(envelope);
        if (outcome == fi::owner::Outcome::Committed) {
            if (lease->commit()) {
                DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultCommitted,
                                IdentifierText(resource),
                                @"Image resource adopted as one undoable change.");
            } else {
                DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRecoveryRequired,
                                nil, @"Command close is unknown; native recovery ownership is retained.");
            }
            return;
        }
        if (!lease->abort()) {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRecoveryRequired,
                            nil, @"Command close is unknown; native recovery ownership is retained.");
            return;
        }
        if (outcome == fi::owner::Outcome::Busy) {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultBusy, nil,
                            @"The adoption could not begin a native command.");
        } else {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRejected, nil,
                            @"The image resource adoption was refused without history.");
        }
    } @catch (...) {
        DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRejected, nil,
                        @"The image resource adoption failed.");
    }
}

- (void)removeFaceImageResourceWithIdentifier:(NSString *)resourceIdentifier
                                   completion:(void (^)(Core3DFaceImageResourceAdoptionResult,
                                                        NSString *))completion {
    if (!NSThread.isMainThread) {
        DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRejected, @"Face image resource removal is main-thread only.");
        return;
    }
    @try {
        UUID resource;
        Handle(OcctDocument) owner;
        std::shared_ptr<OpeningContext> context;
        if (!ParseIdentifier(resourceIdentifier, resource)) {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRejected, @"The resource identifier is not a valid identity.");
            return;
        }
        if (!CaptureFaceImageDocumentOwner(self, owner, context)) {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRejected, @"There is no current document for removal.");
            return;
        }
        const auto lease = context->beginCommandLease(context->openingFence(), 64, 64);
        if (!lease) {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultBusy, @"The removal could not begin a native command.");
            return;
        }
        const fi::owner::Outcome outcome = owner->RemoveFaceImageResource(resource);
        if (outcome == fi::owner::Outcome::Committed) {
            if (lease->commit()) {
                DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultCommitted, @"Image resource removed as one undoable change.");
            } else {
                DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRecoveryRequired, @"Command close is unknown; native recovery ownership is retained.");
            }
            return;
        }
        if (!lease->abort()) {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRecoveryRequired, @"Command close is unknown; native recovery ownership is retained.");
            return;
        }
        if (outcome == fi::owner::Outcome::Busy) {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultBusy, @"The removal could not begin a native command.");
        } else {
            DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRejected, @"The image resource removal was refused without history "
                            @"(absent, or still referenced by a committed binding).");
        }
    } @catch (...) {
        DeliverAdoption(completion, Core3DFaceImageResourceAdoptionResultRejected, @"The image resource removal failed.");
    }
}

- (Core3DFaceImageResourceSnapshot *)faceImageResourceSnapshotForIdentifier:(NSString *)resourceIdentifier {
    if (!NSThread.isMainThread) return nil;
    @try {
        UUID resource;
        if (!ParseIdentifier(resourceIdentifier, resource)) return nil;
        GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)self.glController : nil;
        if (!gl) return nil;
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl.viewer;
        if (!viewer) return nil;
        const Handle(OcctDocument) owner = viewer->getDocument();
        if (owner.IsNull()) return nil;
        fi::ResourceEnvelope envelope;
        if (!owner->ReadFaceImageResource(resource, envelope)) return nil;
        return [[Core3DFaceImageResourceSnapshot alloc]
            initWithResourceIdentifier:IdentifierText(envelope.resource)
            originalContentSHA256:DigestText(envelope.originalContent)
            workingContentSHA256:DigestText(envelope.workingContent)
            originalFormat:ImageFormatText(envelope.originalFormat)
            workingFormat:ImageFormatText(envelope.workingFormat)
            alphaInterpretation:AlphaText(envelope.alpha)
            originalWidthTexels:envelope.originalWidthTexels
            originalHeightTexels:envelope.originalHeightTexels
            workingWidthTexels:envelope.workingWidthTexels
            workingHeightTexels:envelope.workingHeightTexels
            originalBytes:[NSData dataWithBytes:envelope.originalBytes.data()
                                         length:envelope.originalBytes.size()]
            workingBytes:[NSData dataWithBytes:envelope.workingBytes.data()
                                        length:envelope.workingBytes.size()]];
    } @catch (...) { return nil; }
}

@end

#if DEBUG
namespace {
UUID FixtureIndexedUUID(std::uint8_t seed, std::size_t index) {
    UUID value{}; value.fill(seed);
    value[0] = std::uint8_t(index); value[1] = std::uint8_t(index >> 8);
    return value;
}

// One part's feature identities, derived from its owner key so a second
// fixture part in the same document never duplicates the first part's
// feature UUIDs (retained admission refuses duplicate feature identities).
UUID FixturePartFeature(const OwnerKey& key, std::size_t index) {
    return FixtureIndexedUUID(key.entity[0], index);
}

bool FixtureSetUUID(const TDF_Label& label, const char* attributeID, const UUID& value) {
    return !TDataStd_AsciiString::Set(label, Standard_GUID(attributeID),
        TCollection_AsciiString(core3d::retained_solid::UUIDText(value).c_str())).IsNull();
}

// One literal WIDTHx30x20 mm box with a production saved-cut recipe, a stored
// triangulation and the real retained-solid seed record, mirroring the
// row-278a fixture part seam (kind 0) so the owner satisfies the
// retained-carrier admission. The pilot cut position/radius are parameters so
// the U22 split/merge fixture can place the cut across the bound side.
TDF_Label FixtureAddFaceImagePartSized(const Handle(TDocStd_Document)& doc,
                                       const OwnerKey& key, double unit,
                                       double widthMM, double cutXMM,
                                       double cutRMM,
                                       bool verifySourceEditable = true) {
    if (doc.IsNull() || doc->HasOpenCommand())
        throw std::invalid_argument("face image fixture command");
    const double n = .001 / unit;
    TopoDS_Shape shape = BRepPrimAPI_MakeBox(widthMM * n, 30 * n, 20 * n).Shape();
    core3d::retained_boolean::Program program;
    program.source.document = key.document;
    program.source.entity = key.entity;
    program.source.definition = key.definition;
    program.source.sourceFeature = FixturePartFeature(key, 3);
    program.source.derivedFeature = FixturePartFeature(key, 4);
    program.source.family = 1;
    program.source.metersPerUnit = unit;
    core3d::profile::Parameters source;
    source.metersPerUnit = unit;
    source.definition.depth = 20 * n;
    source.definition.points = {{0, 0}, {widthMM * n, 0}, {widthMM * n, 30 * n}, {0, 30 * n}};
    if (!core3d::profile::Encode(source, program.source.values))
        throw std::invalid_argument("face image fixture source recipe");
    program.source.schema = std::uint32_t(core3d::profile::SchemaFor(source));
    core3d::retained_boolean::Step pilot;
    pilot.operand.identifier = 1;
    pilot.operand.axis = core3d::analytic_boolean::Axis::Z;
    pilot.operand.point = std::array<double, 3>{{cutXMM * n, 15 * n, 0}};
    pilot.operand.radius = cutRMM * n;
    auto companion = pilot;
    companion.operand.identifier = 2;
    const bool splitFixture = widthMM == 44 && cutXMM == 41 && cutRMM == 2;
    companion.operand.point = std::array<double, 3>{{
        (splitFixture ? 12 : 28) * n, 15 * n, 0}};
    companion.operand.radius = 3 * n;
    program.steps = {pilot, companion};
    program.nextOperandID = 3;
    if (!core3d::retained_boolean::Valid(program))
        throw std::invalid_argument("face image fixture program");
    BRepBuilderAPI_Copy retainedCopy(shape, Standard_True, Standard_False);
    if (!retainedCopy.IsDone() || retainedCopy.Shape().IsNull())
        throw std::invalid_argument("face image fixture retained base");
    const TopoDS_Shape retainedBase = retainedCopy.Shape();
    const std::atomic_bool stop(false);
    shape = retainedBase;
    for (const auto& step : program.steps) {
        core3d::analytic_boolean::Recipe recipe;
        recipe.metersPerUnit = unit;
        recipe.operation = step.operation;
        recipe.tool = step.operand;
        core3d::analytic_boolean::Result cut;
        if (core3d::analytic_boolean::Build(shape, recipe, stop, cut)
            != core3d::analytic_boolean::Status::Built)
            throw std::invalid_argument("face image fixture production cut");
        shape = cut.solid;
    }
    core3d::saved_cut_source_edit::Patch editableSource =
        core3d::saved_cut_source_values::PolygonPatch{};
    core3d::cut_display::Settings display;
    core3d::saved_boolean_build::Budget verificationBudget;
    if ((verifySourceEditable
            && !core3d::saved_boolean_build::VerifyCurrent(
                retainedBase, shape, program, display, stop, verificationBudget))
        || !core3d::saved_boolean_build::SourcePatch(program, editableSource))
        throw std::invalid_argument("face image fixture source editing");
    // Mesh BEFORE the retained record exists so the geometry fence (whose
    // digest covers stored triangulation) stays consistent.
    BRepMesh_IncrementalMesh mesher(shape, 2.5e-4 * n, Standard_False, 0.5, Standard_True);
    const auto tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    doc->NewCommand();
    const TDF_Label owner = tool->AddShape(shape, Standard_False);
    if (owner.IsNull()) throw std::invalid_argument("face image fixture owner");
    if (!FixtureSetUUID(owner, "0074F7C2-9EAA-4F89-B2DE-8716E155FF62", key.entity)
        || !FixtureSetUUID(owner, "3611F2B2-C694-4E12-AED8-A2A97A3D283B", key.definition))
        throw std::invalid_argument("face image fixture identity");
    TDataStd_Integer::Set(owner,
        Standard_GUID("67E669F4-00C0-4C45-BC55-9CC5DA22A2B5"), 1);
    TDataStd_Name::Set(owner, TCollection_ExtendedString("E3 face image box"));
    if (!core3d::native_opening::debug::Core3DDebugInstallRetainedSolidSeedRecord(
            doc, owner, program, shape, retainedBase))
        throw std::invalid_argument("face image fixture retained admission");
    if (!doc->CommitCommand())
        throw std::invalid_argument("face image fixture retained admission");
    return owner;
}

TDF_Label FixtureAddFaceImagePart(const Handle(TDocStd_Document)& doc,
                                  const OwnerKey& key, double unit) {
    return FixtureAddFaceImagePartSized(doc, key, unit, 40, 12, 3);
}

NSData *FixturePNGData() {
    return [[NSData alloc] initWithBase64EncodedString:
        @"iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAYAAACqaXHeAAAApklEQVR42u3aQQ3AQAzEwFyZF3kKY06qTWAtK8+cndmBnJ1X7j9y/AYKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNAXQApoCaAFNAbSApgBaQFMALaApgBbQnJml/wF2vQsoQAG0gKYAWkBTAC2gKYAW0BRAC2gKoAU0BdACmgJoAU0BtICmAFtAUQAtoCqAFNL8P8AESbQf6Ta5RUwAAAABJRU5ErkJggg=="
        options:0];
}

// The frozen portion-3 fixture: one adopted 64x64 PNG resource (original and
// working bytes deliberately identical here; the importer distinction is
// portion 5's; portion 4b's byte-supplied fixture seam covers it for the
// tests). Extracted verbatim from the portion-3 staging so the fixture bytes
// are unchanged.
fi::ResourceEnvelope FixturePNGEnvelope() {
    NSData *png = FixturePNGData();
    if (png.length == 0 || png.length > fi::kMaximumEncodedImageBytes)
        throw std::invalid_argument("face image fixture png");
    const std::vector<std::uint8_t> pngBytes(
        static_cast<const std::uint8_t*>(png.bytes),
        static_cast<const std::uint8_t*>(png.bytes) + png.length);
    fi::ResourceEnvelope envelope;
    envelope.resource = FixtureIndexedUUID(0x51, 1);
    if (!fi::HashFaceImageBytes(pngBytes, envelope.originalContent)
        || !fi::HashFaceImageBytes(pngBytes, envelope.workingContent))
        throw std::invalid_argument("face image fixture content hashes");
    envelope.originalFormat = fi::ImageEncoding::PNG;
    envelope.workingFormat = fi::ImageEncoding::PNG;
    envelope.alpha = fi::AlphaInterpretation::Opaque;
    envelope.originalWidthTexels = 64; envelope.originalHeightTexels = 64;
    envelope.workingWidthTexels = 64; envelope.workingHeightTexels = 64;
    const std::string provenanceText = "shapeyard.e3.opening.fixture.provenance.v1";
    const std::vector<std::uint8_t> provenanceBytes(provenanceText.begin(), provenanceText.end());
    if (!fi::HashFaceImageBytes(provenanceBytes, envelope.provenance))
        throw std::invalid_argument("face image fixture provenance");
    envelope.originalBytes = pngBytes;
    envelope.workingBytes = pngBytes;
    return envelope;
}

// Capture one planar, axis-parallel bounding-box-extreme side of a shape
// through the real derivation + B2 resolver (portion 4b: shared by the
// fixture staging and the scenario probes). The stage box is measured from
// the exact geometry (never the stored triangulation, whose deflection
// inflates the box past the derivation's extrema tolerance on any meshed
// document). When allowAmbiguous is set (the scenario-1 split-side fixture),
// an ambiguously resolving receipt still captures its derived form: both
// coplanar halves derive the identical receipt, and the committed proof then
// measures the real AmbiguousFaceRemap refusal at capture time.
bool FixtureCapturePlanarFace(const TopoDS_Shape& shape, double unit,
                              core3d::retained_face_selector::Axis axis,
                              core3d::retained_face_selector::Side side,
                              dr::FaceImageGeometricReceipt& captured,
                              bool allowAmbiguous = false) {
    TopTools_IndexedMapOfShape faceMap;
    TopExp::MapShapes(shape, TopAbs_FACE, faceMap);
    Bnd_Box stageBox;
    if (!dr::detail::FaceImageStageBounds(shape, stageBox)) return false;
    core3d::retained_edge_treatment::ReplayBudget budget;
    for (int index = 1; index <= faceMap.Extent(); ++index) {
        const TopoDS_Face face = TopoDS::Face(faceMap.FindKey(index));
        dr::FaceImageGeometricReceipt derived;
        const auto deriveStatus =
            dr::detail::DeriveFaceImageReceipt(stageBox, face, unit, budget, derived);
        if (deriveStatus != dr::detail::FaceImageDeriveStatus::Derived) {
            continue;
        }
        const auto *scope = std::get_if<core3d::retained_face_selector::PlanarFaceBoundary>(
            &derived.intent);
        if (!scope || scope->face.axis != axis || scope->face.side != side) continue;
        dr::FaceImageGeometricReceipt resolved; TopoDS_Face matched;
        const auto resolveRefusal = dr::detail::ResolveFaceImageReceipt(
            shape, derived.intent, unit, budget, resolved, matched);
        if (resolveRefusal != core3d::retained_face_selector::Refusal::None
            || !matched.IsSame(face)) {
            if (allowAmbiguous
                && resolveRefusal == core3d::retained_face_selector::Refusal::FaceAmbiguous) {
                captured = derived;
                return true;
            }
            continue;
        }
        captured = resolved;
        return true;
    }
    return false;
}

// Shared staging for the portion-3 fixture and the portion-4b sized/caller
// byte variants: the retained part, one adopted resource (an already
// validated envelope) and one committed BaseColor binding on the box's max-X
// planar side with a non-default transform and both non-Repeat wrap modes.
void StageFaceImageFixtureSized(const Handle(TDocStd_Document)& doc, double unit,
                                double widthMM, double cutXMM, double cutRMM,
                                const fi::ResourceEnvelope& envelope) {
    doc->ChangeStorageFormatVersion(TDocStd_FormatVersion(12));
    (void)XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    XCAFDoc_DocumentTool::SetLengthUnit(doc, unit);
    UUID documentID{}; documentID.fill(1);
    if (!FixtureSetUUID(doc->Main(), "74386E4E-F620-498F-8092-E6D883AF33A4", documentID))
        throw std::invalid_argument("face image fixture document identity");
    doc->SetUndoLimit(40); doc->ClearUndos();
    const OwnerKey key{documentID, FixtureIndexedUUID(0x42, 1), FixtureIndexedUUID(0x42, 2)};
    const TDF_Label ownerLabel = FixtureAddFaceImagePartSized(doc, key, unit,
                                                              widthMM, cutXMM, cutRMM);

    doc->NewCommand();
    if (fi::owner::AdoptResource(doc, envelope) != fi::owner::Outcome::Committed
        || !doc->CommitCommand())
        throw std::invalid_argument("face image fixture resource adoption");

    // Capture the max-X planar side through the real derivation + B2 resolver.
    const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(ownerLabel);
    if (shape.IsNull()) throw std::invalid_argument("face image fixture shape");
    dr::FaceImageGeometricReceipt captured;
    if (!FixtureCapturePlanarFace(shape, unit,
            core3d::retained_face_selector::Axis::X,
            core3d::retained_face_selector::Side::Max, captured))
        throw std::invalid_argument("face image fixture face capture");
    fi::Binding binding;
    binding.binding = FixtureIndexedUUID(0x51, 2);
    binding.face = FixtureIndexedUUID(0x51, 3);
    if (!dr::FaceImageReceiptProof(captured, binding.selectorProof))
        throw std::invalid_argument("face image fixture face proof");
    binding.resource = envelope.resource;
    binding.role = fi::Role::BaseColor;
    binding.colorSpace = fi::ColorSpace::SRGB;
    binding.transform.scale = {1.25, 1.25};
    binding.transform.offset = {0.5, -0.25};
    binding.transform.rotationDegrees = 30;
    binding.transform.wrapU = fi::Wrap::MirroredRepeat;
    binding.transform.wrapV = fi::Wrap::ClampToEdge;
    fi::Definition candidate;
    candidate.owner = key;
    candidate.bindings = {binding};
    if (!fi::BindBindingProof(candidate))
        throw std::invalid_argument("face image fixture binding proof");
    fi::Observed observed;
    observed.faces.push_back({binding.face, binding.selectorProof});
    if (!fi::owner::ResourceManifest(doc, observed.resources))
        throw std::invalid_argument("face image fixture manifest");
    doc->NewCommand();
    fi::owner::Staging staging;
    const auto fixturePrepare = fi::owner::Prepare(staging, doc, candidate, observed);
    const auto fixtureCommit = fixturePrepare == fi::owner::Outcome::Prepared
        ? fi::owner::Commit(staging, doc, observed) : fi::owner::Outcome::Malformed;
    if (fixturePrepare != fi::owner::Outcome::Prepared
        || fixtureCommit != fi::owner::Outcome::Committed
        || !doc->CommitCommand()) {
        fi::owner::Cancel(staging);
        doc->AbortCommand();
        throw std::invalid_argument("face image fixture binding commit");
    }
    doc->ClearUndos();
    if (!Core3DValidateFaceImageDocument(doc))
        throw std::invalid_argument("face image fixture validation");
}

void StageFaceImageFixture(const Handle(TDocStd_Document)& doc, double unit) {
    StageFaceImageFixtureSized(doc, unit, 40, 12, 3, FixturePNGEnvelope());
}

TDF_Label FixtureAddE4RetainedCapPart(
    const Handle(TDocStd_Document)& doc, const OwnerKey& key, double unit) {
    if (doc.IsNull() || doc->HasOpenCommand())
        throw std::invalid_argument("E4 cap fixture command");
    const double n = .001 / unit;
    const TopoDS_Shape shape =
        BRepPrimAPI_MakeBox(100 * n, 80 * n, 10 * n).Shape();
    BRepBuilderAPI_Copy retainedCopy(shape, Standard_True, Standard_False);
    if (!retainedCopy.IsDone() || retainedCopy.Shape().IsNull())
        throw std::invalid_argument("E4 cap fixture retained base");

    // Retained-v1 requires one bounded Difference step. Its tool is wholly
    // outside the source, so the admitted current solid remains exactly the
    // plain rectangular source: no bore, slot, or mixed boundary.
    core3d::retained_boolean::Program program;
    program.source.document = key.document;
    program.source.entity = key.entity;
    program.source.definition = key.definition;
    program.source.sourceFeature = FixturePartFeature(key, 3);
    program.source.derivedFeature = FixturePartFeature(key, 4);
    program.source.family = 1;
    program.source.metersPerUnit = unit;
    core3d::profile::Parameters source;
    source.metersPerUnit = unit;
    source.definition.depth = 10 * n;
    source.definition.points = {
        {0, 0}, {100 * n, 0}, {100 * n, 80 * n}, {0, 80 * n}};
    if (!core3d::profile::Encode(source, program.source.values))
        throw std::invalid_argument("E4 cap fixture source recipe");
    program.source.schema = std::uint32_t(core3d::profile::SchemaFor(source));
    core3d::retained_boolean::Step outside;
    outside.operand.identifier = 1;
    outside.operand.axis = core3d::analytic_boolean::Axis::Z;
    outside.operand.point = {{200 * n, 200 * n, 0}};
    outside.operand.radius = 3 * n;
    program.steps = {outside};
    program.nextOperandID = 2;
    if (!core3d::retained_boolean::Valid(program))
        throw std::invalid_argument("E4 cap fixture program");

    BRepMesh_IncrementalMesh mesher(
        shape, 2.5e-4 * n, Standard_False, 0.5, Standard_True);
    // The ordinary E4 path consumes the carrier's actual current
    // triangulation. Normalize each analytic face's own parameter rectangle
    // into [0,1]^2 without changing topology, positions, normals or the BRep.
    for (TopExp_Explorer face(shape, TopAbs_FACE); face.More(); face.Next()) {
        TopLoc_Location location;
        const Handle(Poly_Triangulation) mesh = BRep_Tool::Triangulation(
            TopoDS::Face(face.Current()), location);
        if (mesh.IsNull() || !mesh->HasUVNodes() || mesh->NbNodes() <= 0)
            throw std::invalid_argument("E4 cap fixture UV source");
        double minU = std::numeric_limits<double>::max();
        double minV = minU;
        double maxU = -minU;
        double maxV = -minU;
        for (Standard_Integer node = 1; node <= mesh->NbNodes(); ++node) {
            const gp_Pnt2d uv = mesh->UVNode(node);
            minU = std::min(minU, uv.X()); maxU = std::max(maxU, uv.X());
            minV = std::min(minV, uv.Y()); maxV = std::max(maxV, uv.Y());
        }
        const double spanU = maxU - minU, spanV = maxV - minV;
        if (!std::isfinite(spanU) || !std::isfinite(spanV)
            || spanU <= 0.0 || spanV <= 0.0)
            throw std::invalid_argument("E4 cap fixture UV bounds");
        for (Standard_Integer node = 1; node <= mesh->NbNodes(); ++node) {
            const gp_Pnt2d uv = mesh->UVNode(node);
            mesh->SetUVNode(node, gp_Pnt2d(
                (uv.X() - minU) / spanU, (uv.Y() - minV) / spanV));
        }
    }
    const auto tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    doc->NewCommand();
    const TDF_Label owner = tool->AddShape(shape, Standard_False);
    if (owner.IsNull()
        || !FixtureSetUUID(owner,
            "0074F7C2-9EAA-4F89-B2DE-8716E155FF62", key.entity)
        || !FixtureSetUUID(owner,
            "3611F2B2-C694-4E12-AED8-A2A97A3D283B", key.definition))
        throw std::invalid_argument("E4 cap fixture identity");
    TDataStd_Integer::Set(owner,
        Standard_GUID("67E669F4-00C0-4C45-BC55-9CC5DA22A2B5"), 1);
    TDataStd_Name::Set(owner,
        TCollection_ExtendedString("E4 retained cap carrier"));
    if (!core3d::native_opening::debug::
            Core3DDebugInstallRetainedSolidSeedRecord(
                doc, owner, program, shape, retainedCopy.Shape())
        || !doc->CommitCommand())
        throw std::invalid_argument("E4 cap fixture retained admission");
    return owner;
}

void StageE4RetainedCapFixture(const Handle(TDocStd_Document)& doc,
                               double unit,
                               const std::vector<fi::ResourceEnvelope>& envelopes) {
    doc->ChangeStorageFormatVersion(TDocStd_FormatVersion(12));
    (void)XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    XCAFDoc_DocumentTool::SetLengthUnit(doc, unit);
    UUID documentID{}; documentID.fill(1);
    if (!FixtureSetUUID(doc->Main(),
            "74386E4E-F620-498F-8092-E6D883AF33A4", documentID))
        throw std::invalid_argument("E4 cap fixture document identity");
    doc->SetUndoLimit(40); doc->ClearUndos();
    const OwnerKey key{documentID, FixtureIndexedUUID(0x72, 1),
        FixtureIndexedUUID(0x72, 2)};
    (void)FixtureAddE4RetainedCapPart(doc, key, unit);
    doc->NewCommand();
    for (const auto& envelope : envelopes)
        if (fi::owner::AdoptResource(doc, envelope)
                != fi::owner::Outcome::Committed) {
            doc->AbortCommand();
            throw std::invalid_argument("E4 cap fixture resource adoption");
        }
    if (!doc->CommitCommand())
        throw std::invalid_argument("E4 cap fixture resource adoption");
    doc->ClearUndos();
    if (!Core3DValidateFaceImageDocument(doc))
        throw std::invalid_argument("E4 cap fixture validation");
}

// Local mirror of the file-local Core3DCreateDebugBinXCAFFixture helper in
// Core3DViewController.mm (anonymous namespace there; not linkable here):
// one safe BinXCAF document, the caller's staging block, one bounded save.
NSData *CreateFaceImageDebugFixture(
    NSString *suffix,
    const std::function<void(const Handle(TDocStd_Document)&)>& populate) {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL *baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.%@", NSUUID.UUID.UUIDString, suffix]];
    NSString *xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData *result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        if (document.IsNull()) {
            throw Standard_Failure("Unable to create debug XCAF fixture");
        }
        XCAFDoc_DocumentTool::SetLengthUnit(document, 0.001);
        populate(document);
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure("Unable to save debug XCAF fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (const Standard_Failure&) {
        result = nil;
    } catch (const std::exception&) {
        result = nil;
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}
// ---------------------------------------------------------------------------
// E3 DEBUG scenario seams (278b portion 4b). Every seam runs the real
// production operations (the portion-2 document surface, the portion-1 owner
// store, the portion-3 replay collaborator) on attempt-local documents and
// returns measured observations, never a precomputed passing mask.

// Deterministic 16x16 probe raster: an asymmetric opaque gradient. The JPEG
// and PNG encodings of the same pixels differ byte-for-byte, so the original
// and working payloads exercise distinct content identities.
NSData *ProbeRasterImage(BOOL jpeg) {
    @try {
        const NSUInteger side = 16;
        std::vector<std::uint8_t> pixels(side * side * 4);
        for (NSUInteger y = 0; y < side; ++y) {
            for (NSUInteger x = 0; x < side; ++x) {
                const std::size_t at = static_cast<std::size_t>(y * side + x) * 4;
                pixels[at] = std::uint8_t(x * 16);
                pixels[at + 1] = std::uint8_t(y * 16);
                pixels[at + 2] = std::uint8_t((x + y) * 8);
                pixels[at + 3] = 255;
            }
        }
        CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
        if (!colorSpace) return nil;
        CGContextRef context = CGBitmapContextCreate(pixels.data(), side, side, 8,
            side * 4, colorSpace, kCGImageAlphaPremultipliedLast);
        CGColorSpaceRelease(colorSpace);
        if (!context) return nil;
        CGImageRef image = CGBitmapContextCreateImage(context);
        CGContextRelease(context);
        if (!image) return nil;
        NSMutableData *data = [NSMutableData data];
        CGImageDestinationRef destination = CGImageDestinationCreateWithData(
            (__bridge CFMutableDataRef)data,
            jpeg ? CFSTR("public.jpeg") : CFSTR("public.png"), 1, nullptr);
        if (destination) CGImageDestinationAddImage(destination, image, nullptr);
        const bool finished = destination && CGImageDestinationFinalize(destination);
        CGImageRelease(image);
        if (destination) CFRelease(destination);
        return finished && data.length > 0 ? data : nil;
    } @catch (...) { return nil; }
}

// One flat-color opaque PNG of exact dimensions (DEBUG only; used by the
// exact-limit resource fixture).
NSData *ProbeFlatPNG(NSUInteger width, NSUInteger height) {
    @try {
        if (width == 0 || height == 0
            || width > NSUInteger(fi::kMaximumImageDimension)
            || height > NSUInteger(fi::kMaximumImageDimension)
            || width > NSUInteger(fi::kMaximumImagePixels) / height) return nil;
        std::vector<std::uint8_t> pixels(width * height * 4);
        for (NSUInteger index = 0; index < width * height; ++index) {
            pixels[index * 4] = std::uint8_t(index % 251);
            pixels[index * 4 + 1] = std::uint8_t((index / 251) % 251);
            pixels[index * 4 + 2] = std::uint8_t((index / 63001) % 251);
            pixels[index * 4 + 3] = 255;
        }
        CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
        if (!colorSpace) return nil;
        CGContextRef context = CGBitmapContextCreate(pixels.data(), width, height, 8,
            width * 4, colorSpace, kCGImageAlphaPremultipliedLast);
        CGColorSpaceRelease(colorSpace);
        if (!context) return nil;
        CGImageRef image = CGBitmapContextCreateImage(context);
        CGContextRelease(context);
        if (!image) return nil;
        NSMutableData *data = [NSMutableData data];
        CGImageDestinationRef destination = CGImageDestinationCreateWithData(
            (__bridge CFMutableDataRef)data, CFSTR("public.png"), 1, nullptr);
        if (destination) CGImageDestinationAddImage(destination, image, nullptr);
        const bool finished = destination && CGImageDestinationFinalize(destination);
        CGImageRelease(image);
        if (destination) CFRelease(destination);
        return finished && data.length > 0 ? data : nil;
    } @catch (...) { return nil; }
}

// Real production recipe build of one box-with-pilot-cut shape without a
// document: the exact operation sequence the fixture part performs.
TopoDS_Shape ProbeBuildBoxWithCut(double unit, double widthMM, double cutXMM, double cutRMM) {
    try {
        const double n = .001 / unit;
        TopoDS_Shape shape = BRepPrimAPI_MakeBox(widthMM * n, 30 * n, 20 * n).Shape();
        core3d::retained_boolean::Step pilot;
        pilot.operand.identifier = 1;
        pilot.operand.axis = core3d::analytic_boolean::Axis::Z;
        pilot.operand.point = std::array<double, 3>{{cutXMM * n, 15 * n, 0}};
        pilot.operand.radius = cutRMM * n;
        core3d::analytic_boolean::Recipe recipe;
        recipe.metersPerUnit = unit;
        recipe.operation = pilot.operation;
        recipe.tool = pilot.operand;
        const std::atomic_bool stop(false);
        core3d::analytic_boolean::Result cut;
        if (core3d::analytic_boolean::Build(shape, recipe, stop, cut)
            != core3d::analytic_boolean::Status::Built) return TopoDS_Shape();
        return cut.solid;
    } catch (...) { return TopoDS_Shape(); }
}

// Stage the retained box owner into the probe session's document through the
// real fixture part builder (real retained-solid seed record included).
bool ProbeStageBase(OcctDocument& owner, double unit, OwnerKey& key,
                    TDF_Label& ownerLabel, double widthMM, double cutXMM,
                    double cutRMM, bool verifySourceEditable = true) {
    key = {};
    ownerLabel = TDF_Label();
    try {
        const Handle(TDocStd_Document) doc = owner.Document();
        if (doc.IsNull() || doc->HasOpenCommand()) return false;
        doc->ChangeStorageFormatVersion(TDocStd_FormatVersion(12));
        XCAFDoc_DocumentTool::SetLengthUnit(doc, unit);
        if (!core3d::receipt::ParseUUID(owner.DocumentIdentifier(), key.document)) return false;
        key.entity = FixtureIndexedUUID(0x42, 1);
        key.definition = FixtureIndexedUUID(0x42, 2);
        ownerLabel = FixtureAddFaceImagePartSized(
            doc, key, unit, widthMM, cutXMM, cutRMM, verifySourceEditable);
        return !ownerLabel.IsNull();
    } catch (const Standard_Failure&) {
        key = {}; ownerLabel = TDF_Label(); return false;
    } catch (const std::exception&) {
        key = {}; ownerLabel = TDF_Label(); return false;
    } catch (...) { key = {}; ownerLabel = TDF_Label(); return false; }
}

// The probe envelope: caller-style distinct original JPEG and working PNG
// bytes through the shared validated envelope builder (native rehash, derived
// formats/dimensions).
bool ProbeEnvelopeSeeded(NSData *original, NSData *working, std::uint8_t seed,
                         fi::ResourceEnvelope& envelope) {
    envelope = {};
    return fi::validation::BuildFaceImageEnvelope(original, working, @"opaque",
        [@"shapeyard.e3.probe.v1" dataUsingEncoding:NSUTF8StringEncoding],
        FixtureIndexedUUID(seed, 1), envelope);
}

bool ProbeEnvelope(NSData *original, NSData *working, fi::ResourceEnvelope& envelope) {
    return ProbeEnvelopeSeeded(original, working, 0x51, envelope);
}

// One ordinary undoable adoption command through the production document
// surface; the delta is the real undo-command count change.
bool ProbeAdopt(OcctDocument& owner, const fi::ResourceEnvelope& envelope, int& delta) {
    delta = 0;
    try {
        const Handle(TDocStd_Document) doc = owner.Document();
        if (doc.IsNull() || doc->HasOpenCommand()) return false;
        const auto before = doc->GetAvailableUndos();
        doc->NewCommand();
        const fi::owner::Outcome outcome = owner.AdoptFaceImageResource(envelope);
        if (outcome != fi::owner::Outcome::Committed) {
            if (doc->HasOpenCommand()) doc->AbortCommand();
            return false;
        }
        if (!doc->CommitCommand()) return false;
        delta = int(doc->GetAvailableUndos() - before);
        return true;
    } catch (...) { return false; }
}

fi::Observed ProbeObserved(OcctDocument& owner, const fi::Definition& candidate) {
    fi::Observed observed;
    for (const auto& binding : candidate.bindings)
        observed.faces.push_back({binding.face, binding.selectorProof});
    if (!owner.FaceImageResourceManifest(observed.resources))
        throw std::invalid_argument("probe manifest");
    return observed;
}

// One ordinary undoable binding commit through the production document
// transaction surface (Prepare before the command, Commit inside it).
fi::owner::Outcome ProbeCommitBindings(OcctDocument& owner, const fi::Definition& candidate,
                                       int& delta) {
    delta = 0;
    try {
        const Handle(TDocStd_Document) doc = owner.Document();
        if (doc.IsNull() || doc->HasOpenCommand()) return fi::owner::Outcome::Busy;
        const fi::Observed observed = ProbeObserved(owner, candidate);
        const auto before = doc->GetAvailableUndos();
        fi::owner::Staging staging;
        fi::owner::Outcome outcome = owner.PrepareFaceImageBindings(staging, candidate, observed);
        if (outcome != fi::owner::Outcome::Prepared) {
            owner.CancelFaceImageBindings(staging);
            return outcome;
        }
        doc->NewCommand();
        outcome = owner.CommitFaceImageBindings(staging, observed);
        if (outcome != fi::owner::Outcome::Committed) {
            owner.CancelFaceImageBindings(staging);
            if (doc->HasOpenCommand()) doc->AbortCommand();
            delta = int(doc->GetAvailableUndos() - before);
            return outcome;
        }
        if (!doc->CommitCommand()) {
            return fi::owner::Outcome::PersistenceFailure;
        }
        delta = int(doc->GetAvailableUndos() - before);
        return outcome;
    } catch (...) { return fi::owner::Outcome::Malformed; }
}

NSData *ProbeRecordBytes(OcctDocument& owner, const OwnerKey& key) {
    fi::Definition current;
    std::vector<std::uint8_t> bytes;
    if (owner.ReadFaceImageBindings(key, current, &bytes)
            != fi::persistence::bindings::ReadState::Present) return nil;
    return [NSData dataWithBytes:bytes.data() length:bytes.size()];
}

// Bind one committed BaseColor binding on the owner's max-X planar side,
// captured through the real derivation + B2 resolver. Shared probe setup.
bool ProbeBindBaseColor(OcctDocument& owner, const OwnerKey& key,
                        const TDF_Label& ownerLabel, double unit,
                        fi::ResourceEnvelope& adoptedEnvelope) {
    int delta = 0;
    if (!ProbeAdopt(owner, adoptedEnvelope, delta)) return false;
    const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(ownerLabel);
    if (shape.IsNull()) return false;
    dr::FaceImageGeometricReceipt captured;
    if (!FixtureCapturePlanarFace(shape, unit,
            core3d::retained_face_selector::Axis::X,
            core3d::retained_face_selector::Side::Max, captured))
        return false;
    fi::Binding binding;
    binding.binding = FixtureIndexedUUID(0x52, 1);
    binding.face = FixtureIndexedUUID(0x52, 2);
    if (!dr::FaceImageReceiptProof(captured, binding.selectorProof)) return false;
    binding.resource = adoptedEnvelope.resource;
    binding.role = fi::Role::BaseColor;
    binding.colorSpace = fi::ColorSpace::SRGB;
    binding.transform.scale = {1.25, 2.0};
    binding.transform.offset = {0.5, -0.25};
    binding.transform.rotationDegrees = 30;
    binding.transform.wrapU = fi::Wrap::MirroredRepeat;
    binding.transform.wrapV = fi::Wrap::ClampToEdge;
    fi::Definition candidate;
    candidate.owner = key;
    candidate.bindings = {binding};
    if (!fi::BindBindingProof(candidate)) return false;
    return ProbeCommitBindings(owner, candidate, delta) == fi::owner::Outcome::Committed
        && delta == 1;
}

// A real D2 pattern dependency record sourced from the bound owner, staged
// through the production pattern commit seam (278b portion 4b, U22
// unsupported-downstream fixture).
bool FixtureStagePatternDependency(const Handle(TDocStd_Document)& doc,
                                   const OwnerKey& sourceKey, const UUID& sourceFeature,
                                   const OwnerKey& resultKey, double unit) {
    try {
        namespace pat = core3d::pattern;
        pat::Definition definition;
        definition.owner = resultKey;
        definition.feature = FixtureIndexedUUID(0x61, 1);
        definition.source.document = sourceKey.document;
        definition.source.entity = sourceKey.entity;
        definition.source.definition = sourceKey.definition;
        definition.source.sourceFeature = sourceFeature;
        definition.kind = pat::Kind::Linear;
        definition.columnAxis = pat::Axis::X;
        definition.rowAxis = pat::Axis::Y;
        definition.rowCount = 1;
        definition.columnCount = 2;
        definition.rowSpacing = 0;
        definition.columnSpacing = 60 * (.001 / unit);
        definition.issuance.nextLocalID = 3;
        pat::Member sourceMember;
        sourceMember.identity = sourceKey.entity;
        sourceMember.localID = 1;
        sourceMember.coordinate = {0, 0};
        sourceMember.state = pat::MemberState::Active;
        pat::Member copyMember;
        copyMember.identity = FixtureIndexedUUID(0x61, 2);
        copyMember.localID = 2;
        copyMember.coordinate = {0, 1};
        copyMember.state = pat::MemberState::Active;
        definition.members = {sourceMember, copyMember};
        pat::Record staged;
        if (doc->HasOpenCommand()) return false;
        doc->NewCommand();
        const bool ok = pat::Stage(doc, definition, staged) && !staged.label.IsNull();
        if (ok) {
            if (!doc->CommitCommand()) return false;
        } else if (doc->HasOpenCommand()) {
            doc->AbortCommand();
        }
        return ok;
    } catch (...) { return false; }
}

void ProbeRecordBit(NSMutableDictionary *bits, unsigned& mask, unsigned bit,
                    bool pass, NSString *detail) {
    bits[@(bit)] = @{@"passed": @(pass ? YES : NO), @"detail": detail ?: @""};
    if (pass) mask |= (1u << bit);
}

// Scenario 0 (U19 positive lifecycle, document level): adoption, distinct
// original/working readback, bind, edit, real Undo/Redo, every role with its
// color-space coupling, all wrap modes, save + destroy + cold reopen through
// the production open path, and removal/refusal semantics. Every bit is
// measured independently; the mask is the AND of real measurements.
NSDictionary *FaceImageScenarioZeroProbe(double unit) {
    NSMutableDictionary *bits = [NSMutableDictionary dictionary];
    unsigned mask = 0;
    NSURL *baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.e3-probe-zero", NSUUID.UUID.UUIDString]];
    NSString *xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    @try {
        try {
        core3d::NativeDocumentSession session;
        OcctDocument& owner = *session.Document();
        OwnerKey key;
        TDF_Label ownerLabel;
        NSData *working = ProbeRasterImage(NO);
        NSData *original = ProbeRasterImage(YES);
        fi::ResourceEnvelope envelope;
        if (!working || !original
            || !ProbeStageBase(owner, unit, key, ownerLabel, 40, 12, 3)) {
            ProbeRecordBit(bits, mask, 0, false, @"fixture staging failed");
        } else {
            if (!ProbeEnvelope(original, working, envelope))
                ProbeRecordBit(bits, mask, 0, false, @"envelope validation failed");
        }
        fi::OwnerKey fiKey;
        fiKey.document = key.document; fiKey.entity = key.entity; fiKey.definition = key.definition;

        // bit 0: adoption is one ordinary undoable command with a live identity.
        int delta = 0;
        const bool adopted = fi::Nonzero(envelope.provenance)
            && ProbeAdopt(owner, envelope, delta);
        {
            const auto probe = fi::FaceImageProbe::Observe(owner.Document());
            ProbeRecordBit(bits, mask, 0, adopted && delta == 1 && probe.complete
                           && probe.resources == 1,
                [NSString stringWithFormat:@"adopted=%d commandDelta=%d probeComplete=%d resources=%ld",
                    adopted ? 1 : 0, delta, probe.complete ? 1 : 0, (long)probe.resources]);
        }
        // bit 1: readback; the stored digests are the native rehashes of the
        // exact bytes and the original/working payloads stay distinct.
        {
            fi::ResourceEnvelope back;
            fi::Digest originalHash{}, workingHash{};
            const bool ok = owner.ReadFaceImageResource(envelope.resource, back)
                && back == envelope
                && fi::HashFaceImageBytes(back.originalBytes, originalHash)
                && fi::HashFaceImageBytes(back.workingBytes, workingHash)
                && originalHash == back.originalContent
                && workingHash == back.workingContent
                && !(back.originalContent == back.workingContent)
                && back.originalFormat == fi::ImageEncoding::JPEG
                && back.workingFormat == fi::ImageEncoding::PNG
                && back.alpha == fi::AlphaInterpretation::Opaque
                && back.originalWidthTexels == 16 && back.originalHeightTexels == 16
                && back.workingWidthTexels == 16 && back.workingHeightTexels == 16;
            ProbeRecordBit(bits, mask, 1, ok, @"readback of both payloads and derived metadata");
        }
        // bit 2: bind one committed BaseColor binding as one command with
        // every durable identity and exact transform/role/colorSpace/wrap.
        fi::Definition committed;
        {
            const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(ownerLabel);
            dr::FaceImageGeometricReceipt captured;
            fi::Binding binding;
            binding.binding = FixtureIndexedUUID(0x52, 1);
            binding.face = FixtureIndexedUUID(0x52, 2);
            bool ok = !shape.IsNull()
                && FixtureCapturePlanarFace(shape, unit,
                    core3d::retained_face_selector::Axis::X,
                    core3d::retained_face_selector::Side::Max, captured)
                && dr::FaceImageReceiptProof(captured, binding.selectorProof);
            binding.resource = envelope.resource;
            binding.role = fi::Role::BaseColor;
            binding.colorSpace = fi::ColorSpace::SRGB;
            binding.transform.scale = {1.25, 2.0};
            binding.transform.offset = {0.5, -0.25};
            binding.transform.rotationDegrees = 30;
            binding.transform.wrapU = fi::Wrap::MirroredRepeat;
            binding.transform.wrapV = fi::Wrap::ClampToEdge;
            fi::Definition candidate;
            candidate.owner = fiKey;
            candidate.bindings = {binding};
            ok = ok && fi::BindBindingProof(candidate)
                && ProbeCommitBindings(owner, candidate, delta)
                        == fi::owner::Outcome::Committed && delta == 1;
            fi::Definition back;
            std::vector<std::uint8_t> bytes;
            const auto readState = owner.ReadFaceImageBindings(fiKey, back, &bytes);
            ok = ok && readState == fi::persistence::bindings::ReadState::Present
                && back == candidate && !bytes.empty();
            if (ok) committed = candidate;
            ProbeRecordBit(bits, mask, 2, ok, @"bind committed and read back exactly");
        }
        // bit 3: edit the transform as one command, then Undo/Redo restore
        // the exact record bytes with real history counts.
        {
            bool ok = false;
            NSData *beforeBytes = ProbeRecordBytes(owner, fiKey);
            fi::Definition edited = committed;
            if (edited.bindings.empty()) {
                ProbeRecordBit(bits, mask, 3, false, @"prerequisite binding missing");
                goto scenarioZeroBit4;
            }
            edited.bindings[0].transform.scale = {2.5, 0.75};
            edited.bindings[0].transform.rotationDegrees = 315;
            if (beforeBytes && fi::BindBindingProof(edited)
                && ProbeCommitBindings(owner, edited, delta) == fi::owner::Outcome::Committed
                && delta == 1) {
                NSData *editedBytes = ProbeRecordBytes(owner, fiKey);
                if (editedBytes && ![editedBytes isEqualToData:beforeBytes] && owner.undo()) {
                    NSData *undone = ProbeRecordBytes(owner, fiKey);
                    if ([undone isEqualToData:beforeBytes] && owner.redo()) {
                        NSData *redone = ProbeRecordBytes(owner, fiKey);
                        ok = [redone isEqualToData:editedBytes];
                    }
                }
            }
            ProbeRecordBit(bits, mask, 3, ok, @"edit plus real Undo/Redo byte round trip");
            committed = edited;
        }
        // bit 4: every role binds with its frozen color-space coupling, and a
        // wrong coupling refuses without history.
        scenarioZeroBit4:
        {
            bool ok = true;
            const fi::Role roles[4] = {fi::Role::Emissive, fi::Role::MetallicRoughness,
                fi::Role::Occlusion, fi::Role::Normal};
            const fi::ColorSpace spaces[4] = {fi::ColorSpace::SRGB, fi::ColorSpace::Linear,
                fi::ColorSpace::Linear, fi::ColorSpace::Linear};
            if (committed.bindings.empty()) ok = false;
            for (int index = 0; ok && index < 4; ++index) {
                fi::Definition next = committed;
                fi::Binding added = committed.bindings[0];
                added.binding = FixtureIndexedUUID(0x52, std::size_t(3 + index));
                added.role = roles[index];
                added.colorSpace = spaces[index];
                added.transform = fi::UVTransform{};
                next.bindings.push_back(added);
                ok = fi::BindBindingProof(next)
                    && ProbeCommitBindings(owner, next, delta) == fi::owner::Outcome::Committed
                    && delta == 1;
                if (ok) {
                    fi::Definition back;
                    ok = owner.ReadFaceImageBindings(fiKey, back, nullptr)
                            == fi::persistence::bindings::ReadState::Present
                        && back == next;
                    committed = next;
                }
            }
            // Wrong coupling: BaseColor with Linear must refuse with no delta.
            if (committed.bindings.empty()) ok = false;
            fi::Definition wrong = committed;
            if (!wrong.bindings.empty())
                wrong.bindings[0].colorSpace = fi::ColorSpace::Linear;
            const auto undosBefore = owner.Document()->GetAvailableUndos();
            const fi::owner::Outcome refused = ProbeCommitBindings(owner, wrong, delta);
            fi::Definition back;
            ok = ok && refused != fi::owner::Outcome::Committed
                && owner.Document()->GetAvailableUndos() == undosBefore
                && owner.ReadFaceImageBindings(fiKey, back, nullptr)
                        == fi::persistence::bindings::ReadState::Present
                && back == committed;
            ProbeRecordBit(bits, mask, 4, ok, @"five roles with coupling plus wrong-coupling refusal");
        }
        // bit 5: all three wrap modes round-trip exactly.
        {
            bool ok = true;
            const fi::Wrap wraps[3] = {fi::Wrap::ClampToEdge, fi::Wrap::Repeat,
                fi::Wrap::MirroredRepeat};
            if (committed.bindings.empty()) ok = false;
            for (int index = 0; ok && index < 3; ++index) {
                fi::Definition next = committed;
                next.bindings[0].transform.wrapU = wraps[index];
                next.bindings[0].transform.wrapV = wraps[(index + 1) % 3];
                ok = fi::BindBindingProof(next)
                    && ProbeCommitBindings(owner, next, delta) == fi::owner::Outcome::Committed;
                if (ok) {
                    fi::Definition back;
                    ok = owner.ReadFaceImageBindings(fiKey, back, nullptr)
                            == fi::persistence::bindings::ReadState::Present && back == next;
                    committed = next;
                }
            }
            ProbeRecordBit(bits, mask, 5, ok, @"all wrap modes committed and read back exactly");
        }
        // bit 6: save, destroy, cold reopen through the production open path;
        // every UUID and both byte arrays survive.
        {
            bool ok = false;
            NSData *beforeBytes = ProbeRecordBytes(owner, fiKey);
            const Handle(TDocStd_Application) application =
                Handle(TDocStd_Application)::DownCast(owner.Document()->Application());
            if (beforeBytes && !application.IsNull()
                && application->SaveAs(owner.Document(),
                       baseURL.path.UTF8String) == PCDM_SS_OK) {
                Handle(OcctDocument) reopened = new OcctDocument();
                if (reopened->OpenPrivateExportSnapshot(xbfPath.UTF8String,
                        Message_ProgressRange())) {
                    fi::ResourceEnvelope backResource;
                    fi::Definition backDefinition;
                    std::vector<std::uint8_t> reopenedBytes;
                    const bool resourceOK = reopened->ReadFaceImageResource(
                        envelope.resource, backResource) && backResource == envelope;
                    const auto reopenedState = reopened->ReadFaceImageBindings(
                        fiKey, backDefinition, &reopenedBytes);
                    const bool bytesOK = [[NSData dataWithBytes:reopenedBytes.data()
                                                         length:reopenedBytes.size()]
                        isEqualToData:beforeBytes];
                    const bool docOK = reopened->DocumentIdentifier()
                        == owner.DocumentIdentifier();
                    const bool validOK = Core3DValidateFaceImageDocument(reopened->Document());
                    ok = resourceOK
                        && reopenedState == fi::persistence::bindings::ReadState::Present
                        && backDefinition == committed
                        && bytesOK && docOK && validOK;
                    reopened->ClosePrivateExportSnapshot();
                }
            }
            ProbeRecordBit(bits, mask, 6, ok, @"save, destroy, cold reopen, exact identities");
        }
        // bit 7: typed resource removal refuses while bound (no history), the
        // unbind+removal commits as one command and Undo restores both.
        {
            bool ok = false;
            const Handle(TDocStd_Document) doc = owner.Document();
            NSData *beforeBytes = ProbeRecordBytes(owner, fiKey);
            const auto undosBefore = doc->GetAvailableUndos();
            doc->NewCommand();
            const fi::owner::Outcome boundRefusal =
                owner.RemoveFaceImageResource(envelope.resource);
            if (doc->HasOpenCommand()) doc->AbortCommand();
            fi::ResourceEnvelope stillThere;
            if (beforeBytes && boundRefusal == fi::owner::Outcome::Refused
                && doc->GetAvailableUndos() == undosBefore
                && owner.ReadFaceImageResource(envelope.resource, stillThere)
                && stillThere == envelope) {
                doc->NewCommand();
                const bool removed =
                    owner.RemoveFaceImageBindings(fiKey) == fi::owner::Outcome::Committed
                    && owner.RemoveFaceImageResource(envelope.resource)
                        == fi::owner::Outcome::Committed;
                if (removed && doc->CommitCommand()
                    && doc->GetAvailableUndos() == undosBefore + 1) {
                    fi::ResourceEnvelope gone;
                    fi::Definition absent;
                    if (!owner.ReadFaceImageResource(envelope.resource, gone)
                        && owner.ReadFaceImageBindings(fiKey, absent, nullptr)
                            == fi::persistence::bindings::ReadState::Absent
                        && owner.undo()) {
                        fi::ResourceEnvelope restored;
                        ok = owner.ReadFaceImageResource(envelope.resource, restored)
                            && restored == envelope
                            && [(ProbeRecordBytes(owner, fiKey)
                                    ?: [NSData data]) isEqualToData:beforeBytes];
                    }
                }
            }
            ProbeRecordBit(bits, mask, 7, ok,
                @"removal refused while bound; one-command unbind+remove; Undo restores");
        }
        } catch (const Standard_Failure& error) {
            ProbeRecordBit(bits, mask, 0, false, [NSString stringWithFormat:
                @"probe aborted by occt exception: %s",
                error.GetMessageString() ? error.GetMessageString() : "?"]);
        } catch (const std::exception& error) {
            ProbeRecordBit(bits, mask, 0, false, [NSString stringWithFormat:
                @"probe aborted by c++ exception: %s", error.what()]);
        } catch (...) {
            ProbeRecordBit(bits, mask, 0, false, @"probe aborted by c++ exception");
        }
    } @catch (...) {
        ProbeRecordBit(bits, mask, 0, false, @"probe aborted by exception");
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return @{@"schema": @"shapeyard.face-image.probe.v1",
             @"scenario": @"scenario0",
             @"metersPerUnit": @(unit),
             @"mask": @(mask),
             @"bits": bits};
}

// Scenario 1 (U20 negatives, document level): stale face, ambiguous remap,
// stale/missing/foreign resource, unsupported surface, unsupported
// downstream with a real D2 dependency record, no-op and cancelled prepare,
// and aborted-command recovery. Every bit measures the real refusal and the
// exact byte/history preservation; the mask is the AND of real measurements.
NSDictionary *FaceImageScenarioOneProbe(double unit) {
    NSMutableDictionary *bits = [NSMutableDictionary dictionary];
    unsigned mask = 0;
    // bit 0: a committed proof that no current face matches is StaleFace; the
    // capture is read-only and the record/history are untouched.
    @try {
        try {
        core3d::NativeDocumentSession session;
        OcctDocument& owner = *session.Document();
        OwnerKey key;
        TDF_Label ownerLabel;
        bool ok = ProbeStageBase(owner, unit, key, ownerLabel, 40, 12, 3);
        fi::OwnerKey fiKey{key.document, key.entity, key.definition};
        fi::ResourceEnvelope envelope;
        if (ok) ok = ProbeEnvelope(ProbeRasterImage(YES), ProbeRasterImage(NO), envelope);
        if (ok) {
            int delta = 0;
            // Commit a binding carrying the real proof of the 44 mm sibling
            // shape's max-X side: the stored proof matches no current face.
            const TopoDS_Shape sibling = ProbeBuildBoxWithCut(unit, 44, 12, 3);
            dr::FaceImageGeometricReceipt captured;
            fi::Binding binding;
            binding.binding = FixtureIndexedUUID(0x52, 1);
            binding.face = FixtureIndexedUUID(0x52, 2);
            ok = !sibling.IsNull()
                && FixtureCapturePlanarFace(sibling, unit,
                    core3d::retained_face_selector::Axis::X,
                    core3d::retained_face_selector::Side::Max, captured)
                && dr::FaceImageReceiptProof(captured, binding.selectorProof)
                && ProbeAdopt(owner, envelope, delta);
            binding.resource = envelope.resource;
            binding.role = fi::Role::BaseColor;
            binding.colorSpace = fi::ColorSpace::SRGB;
            fi::Definition candidate;
            candidate.owner = fiKey;
            candidate.bindings = {binding};
            ok = ok && fi::BindBindingProof(candidate)
                && ProbeCommitBindings(owner, candidate, delta)
                    == fi::owner::Outcome::Committed;
            NSData *beforeBytes = ok ? ProbeRecordBytes(owner, fiKey) : nil;
            const auto undosBefore = owner.Document()->GetAvailableUndos();
            dr::FaceImageAttachment attachment;
            const auto status = dr::CaptureFaceImageAttachment(owner, fiKey, attachment);
            ok = ok && status == dr::FaceImageReplayStatus::StaleFace
                && [(ProbeRecordBytes(owner, fiKey) ?: [NSData data]) isEqualToData:beforeBytes]
                && owner.Document()->GetAvailableUndos() == undosBefore;
        }
        ProbeRecordBit(bits, mask, 0, ok, @"stale face refusal preserves bytes and history");
    } catch (...) { ProbeRecordBit(bits, mask, 0, false, @"c++ exception"); }
    } @catch (...) { ProbeRecordBit(bits, mask, 0, false, @"exception"); }
    // bit 1: a stored proof matching two current coplanar faces is
    // AmbiguousFaceRemap; the record/history are untouched.
    @try {
        try {
        core3d::NativeDocumentSession session;
        OcctDocument& owner = *session.Document();
        OwnerKey key;
        TDF_Label ownerLabel;
        // The pilot cut crosses the bound max-X side: the side is split into
        // two coplanar faces with identical derived receipts.
        bool ok = ProbeStageBase(owner, unit, key, ownerLabel, 40, 40, 5,
            /*verifySourceEditable=*/false);
        fi::OwnerKey fiKey{key.document, key.entity, key.definition};
        fi::ResourceEnvelope envelope;
        if (ok) ok = ProbeEnvelope(ProbeRasterImage(YES), ProbeRasterImage(NO), envelope);
        if (ok) {
            int delta = 0;
            const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(ownerLabel);
            dr::FaceImageGeometricReceipt captured;
            fi::Binding binding;
            binding.binding = FixtureIndexedUUID(0x52, 1);
            binding.face = FixtureIndexedUUID(0x52, 2);
            ok = !shape.IsNull()
                && FixtureCapturePlanarFace(shape, unit,
                    core3d::retained_face_selector::Axis::X,
                    core3d::retained_face_selector::Side::Max, captured,
                    /*allowAmbiguous=*/true)
                && dr::FaceImageReceiptProof(captured, binding.selectorProof)
                && ProbeAdopt(owner, envelope, delta);
            binding.resource = envelope.resource;
            binding.role = fi::Role::BaseColor;
            binding.colorSpace = fi::ColorSpace::SRGB;
            fi::Definition candidate;
            candidate.owner = fiKey;
            candidate.bindings = {binding};
            ok = ok && fi::BindBindingProof(candidate)
                && ProbeCommitBindings(owner, candidate, delta)
                    == fi::owner::Outcome::Committed;
            NSData *beforeBytes = ok ? ProbeRecordBytes(owner, fiKey) : nil;
            const auto undosBefore = owner.Document()->GetAvailableUndos();
            dr::FaceImageAttachment attachment;
            const auto status = dr::CaptureFaceImageAttachment(owner, fiKey, attachment);
            ok = ok && status == dr::FaceImageReplayStatus::AmbiguousFaceRemap
                && [(ProbeRecordBytes(owner, fiKey) ?: [NSData data]) isEqualToData:beforeBytes]
                && owner.Document()->GetAvailableUndos() == undosBefore;
        }
        ProbeRecordBit(bits, mask, 1, ok, @"ambiguous remap refusal preserves bytes and history");
    } catch (...) { ProbeRecordBit(bits, mask, 1, false, @"c++ exception"); }
    } @catch (...) { ProbeRecordBit(bits, mask, 1, false, @"exception"); }
    // bit 2: a drifted resource fence refuses StaleResource without history.
    @try {
        try {
        core3d::NativeDocumentSession session;
        OcctDocument& owner = *session.Document();
        OwnerKey key;
        TDF_Label ownerLabel;
        bool ok = ProbeStageBase(owner, unit, key, ownerLabel, 40, 12, 3);
        fi::OwnerKey fiKey{key.document, key.entity, key.definition};
        fi::ResourceEnvelope envelope;
        if (ok) ok = ProbeEnvelope(ProbeRasterImage(YES), ProbeRasterImage(NO), envelope);
        if (ok) ok = ProbeBindBaseColor(owner, fiKey, ownerLabel, unit, envelope);
        // Note: ProbeBindBaseColor adopts its own envelope; reuse the live one.
        if (ok) {
            fi::Definition committed;
            ok = owner.ReadFaceImageBindings(fiKey, committed, nullptr)
                == fi::persistence::bindings::ReadState::Present;
            NSData *beforeBytes = ok ? ProbeRecordBytes(owner, fiKey) : nil;
            const auto undosBefore = owner.Document()->GetAvailableUndos();
            fi::Observed observed = ProbeObserved(owner, committed);
            if (observed.resources.empty()) ok = false;
            else observed.resources[0].content[0] ^= 0xFF; // drifted fence
            fi::owner::Staging staging;
            const fi::owner::Outcome outcome =
                ok ? owner.PrepareFaceImageBindings(staging, committed, observed)
                   : fi::owner::Outcome::Malformed;
            owner.CancelFaceImageBindings(staging);
            fi::Definition back;
            ok = ok && outcome == fi::owner::Outcome::StaleResource
                && owner.Document()->GetAvailableUndos() == undosBefore
                && owner.ReadFaceImageBindings(fiKey, back, nullptr)
                        == fi::persistence::bindings::ReadState::Present
                && back == committed
                && [(ProbeRecordBytes(owner, fiKey) ?: [NSData data]) isEqualToData:beforeBytes];
        }
        ProbeRecordBit(bits, mask, 2, ok, @"stale resource fence refuses without history");
    } catch (...) { ProbeRecordBit(bits, mask, 2, false, @"c++ exception"); }
    } @catch (...) { ProbeRecordBit(bits, mask, 2, false, @"exception"); }
    // bit 3: a missing resource and a foreign (other-document) resource both
    // refuse MissingResource without history.
    @try {
        try {
        core3d::NativeDocumentSession session;
        OcctDocument& owner = *session.Document();
        OwnerKey key;
        TDF_Label ownerLabel;
        bool ok = ProbeStageBase(owner, unit, key, ownerLabel, 40, 12, 3);
        fi::OwnerKey fiKey{key.document, key.entity, key.definition};
        fi::ResourceEnvelope envelope;
        if (ok) ok = ProbeEnvelope(ProbeRasterImage(YES), ProbeRasterImage(NO), envelope);
        if (ok) ok = ProbeBindBaseColor(owner, fiKey, ownerLabel, unit, envelope);
        if (ok) {
            fi::Definition committed;
            ok = owner.ReadFaceImageBindings(fiKey, committed, nullptr)
                == fi::persistence::bindings::ReadState::Present;
            const auto undosBefore = owner.Document()->GetAvailableUndos();
            // Missing: a never-adopted identity.
            fi::Definition missing = committed;
            UUID missingResource{};
            ok = ok && MintUUID(missingResource);
            for (auto& binding : missing.bindings) binding.resource = missingResource;
            ok = ok && fi::BindBindingProof(missing);
            fi::owner::Staging staging;
            fi::Observed observed;
            if (ok) observed = ProbeObserved(owner, missing);
            const fi::owner::Outcome missingOutcome =
                ok ? owner.PrepareFaceImageBindings(staging, missing, observed)
                   : fi::owner::Outcome::Malformed;
            owner.CancelFaceImageBindings(staging);
            // Foreign: an identity adopted by another document only.
            fi::owner::Outcome foreignOutcome = fi::owner::Outcome::Malformed;
            if (ok) {
                core3d::NativeDocumentSession otherSession;
                OcctDocument& other = *otherSession.Document();
                fi::ResourceEnvelope foreign;
                ok = ProbeEnvelopeSeeded(ProbeRasterImage(NO), ProbeRasterImage(NO), 0x54, foreign);
                int delta = 0;
                ok = ok && ProbeAdopt(other, foreign, delta);
                fi::Definition foreignCandidate = committed;
                for (auto& binding : foreignCandidate.bindings)
                    binding.resource = foreign.resource;
                ok = ok && fi::BindBindingProof(foreignCandidate);
                if (ok) {
                    observed = ProbeObserved(owner, foreignCandidate);
                    foreignOutcome = owner.PrepareFaceImageBindings(
                        staging, foreignCandidate, observed);
                    owner.CancelFaceImageBindings(staging);
                }
            }
            fi::Definition back;
            ok = ok && missingOutcome == fi::owner::Outcome::MissingResource
                && foreignOutcome == fi::owner::Outcome::MissingResource
                && owner.Document()->GetAvailableUndos() == undosBefore
                && owner.ReadFaceImageBindings(fiKey, back, nullptr)
                        == fi::persistence::bindings::ReadState::Present
                && back == committed;
        }
        ProbeRecordBit(bits, mask, 3, ok, @"missing and foreign resources refuse without history");
    } catch (...) { ProbeRecordBit(bits, mask, 3, false, @"c++ exception"); }
    } @catch (...) { ProbeRecordBit(bits, mask, 3, false, @"exception"); }
    // bit 4: a curved face and a planar face inset from the stage extremum by
    // more than the fixed tolerance are both refused without history.
    @try {
        try {
        core3d::NativeDocumentSession session;
        OcctDocument& owner = *session.Document();
        OwnerKey key;
        TDF_Label ownerLabel;
        bool ok = ProbeStageBase(owner, unit, key, ownerLabel, 40, 12, 3);
        if (ok) {
            const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(ownerLabel);
            TopTools_IndexedMapOfShape faceMap;
            TopExp::MapShapes(shape, TopAbs_FACE, faceMap);
            Bnd_Box stageBox;
            if (!dr::detail::FaceImageStageBounds(shape, stageBox)) ok = false;
            core3d::retained_edge_treatment::ReplayBudget budget;
            bool foundCurved = false, curvedRefused = false;
            bool foundMaximumX = false, insetRefused = false;
            const auto undosBefore = owner.Document()->GetAvailableUndos();
            Standard_Real xMin = 0, yMin = 0, zMin = 0, xMax = 0, yMax = 0, zMax = 0;
            stageBox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
            const double tolerance = 1e-4 / (1000.0 * unit);
            for (int index = 1; index <= faceMap.Extent(); ++index) {
                const TopoDS_Face face = TopoDS::Face(faceMap.FindKey(index));
                BRepAdaptor_Surface surface(face, true);
                if (surface.GetType() == GeomAbs_Cylinder) {
                    foundCurved = true;
                    dr::FaceImageGeometricReceipt derived;
                    curvedRefused = dr::detail::DeriveFaceImageReceipt(
                        stageBox, face, unit, budget, derived)
                        == dr::detail::FaceImageDeriveStatus::NotAddressable;
                } else if (surface.GetType() == GeomAbs_Plane
                    && std::abs(surface.Plane().Location().X() - xMax) <= tolerance) {
                    foundMaximumX = true;
                    Bnd_Box insetBox = stageBox;
                    insetBox.Update(xMin, yMin, zMin,
                        xMax + 2 * tolerance, yMax, zMax);
                    dr::FaceImageGeometricReceipt derived;
                    core3d::retained_edge_treatment::ReplayBudget insetBudget;
                    insetRefused = dr::detail::DeriveFaceImageReceipt(
                        insetBox, face, unit, insetBudget, derived)
                        == dr::detail::FaceImageDeriveStatus::NotAddressable;
                }
            }
            ok = foundCurved && curvedRefused && foundMaximumX && insetRefused
                && owner.Document()->GetAvailableUndos() == undosBefore;
        }
        ProbeRecordBit(bits, mask, 4, ok,
            @"curved and tolerance-inset planar faces refuse without history");
    } catch (...) { ProbeRecordBit(bits, mask, 4, false, @"c++ exception"); }
    } @catch (...) { ProbeRecordBit(bits, mask, 4, false, @"exception"); }
    // bit 5: propagation to a Boolean/duplication result entity is
    // UnsupportedDownstream, measured against a real D2 dependency record.
    @try {
        try {
        core3d::NativeDocumentSession session;
        OcctDocument& owner = *session.Document();
        OwnerKey key;
        TDF_Label ownerLabel;
        bool ok = ProbeStageBase(owner, unit, key, ownerLabel, 40, 12, 3);
        fi::OwnerKey fiKey{key.document, key.entity, key.definition};
        fi::ResourceEnvelope envelope;
        if (ok) ok = ProbeEnvelope(ProbeRasterImage(YES), ProbeRasterImage(NO), envelope);
        if (ok) ok = ProbeBindBaseColor(owner, fiKey, ownerLabel, unit, envelope);
        if (ok) {
            const Handle(TDocStd_Document) doc = owner.Document();
            const OwnerKey resultKey{key.document, FixtureIndexedUUID(0x42, 11),
                FixtureIndexedUUID(0x42, 12)};
            const TDF_Label resultLabel =
                FixtureAddFaceImagePartSized(doc, resultKey, unit, 40, 12, 3);
            ok = !resultLabel.IsNull()
                && FixtureStagePatternDependency(doc, key,
                    FixturePartFeature(key, 3), resultKey, unit);
            // The dependency is a real persisted record, not a test flag.
            std::vector<core3d::pattern::Record> records;
            ok = ok && core3d::pattern::ReadAll(doc, records) && records.size() == 1
                && records[0].definition.source.entity == key.entity
                && records[0].definition.owner.entity == resultKey.entity;
            NSData *beforeBytes = ok ? ProbeRecordBytes(owner, fiKey) : nil;
            const auto undosBefore = doc->GetAvailableUndos();
            dr::FaceImageAttachment attachment;
            ok = ok && dr::CaptureFaceImageAttachment(owner, fiKey, attachment)
                    == dr::FaceImageReplayStatus::Captured;
            if (ok) {
                const TopoDS_Shape currentShape = XCAFDoc_ShapeTool::GetShape(ownerLabel);
                dr::FaceImageReplay replay;
                const auto refused = replay.prepare(owner, attachment,
                    dr::Mutation::Replace, resultKey.entity, currentShape);
                dr::FaceImageReplay control;
                const auto supported = control.prepare(owner, attachment,
                    dr::Mutation::Replace, fiKey.entity, currentShape);
                ok = refused == dr::FaceImageReplayStatus::UnsupportedDownstream
                    && (supported == dr::FaceImageReplayStatus::NoChange
                        || supported == dr::FaceImageReplayStatus::Prepared)
                    && doc->GetAvailableUndos() == undosBefore
                    && [(ProbeRecordBytes(owner, fiKey) ?: [NSData data]) isEqualToData:beforeBytes];
            }
        }
        ProbeRecordBit(bits, mask, 5, ok,
            @"unsupported downstream refuses against a real dependency record");
    } catch (const Standard_Failure& error) {
        ProbeRecordBit(bits, mask, 5, false, [NSString stringWithFormat:
            @"occt exception: %s", error.GetMessageString() ? error.GetMessageString() : "?"]);
    } catch (const std::exception& error) {
        ProbeRecordBit(bits, mask, 5, false, [NSString stringWithFormat:
            @"c++ exception: %s", error.what()]);
    } catch (...) { ProbeRecordBit(bits, mask, 5, false, @"c++ exception"); }
    } @catch (...) { ProbeRecordBit(bits, mask, 5, false, @"exception"); }
    // bit 6: a no-op commit is a zero-delta refusal and a cancelled prepare
    // leaves no trace; bytes and history are exact.
    @try {
        try {
        core3d::NativeDocumentSession session;
        OcctDocument& owner = *session.Document();
        OwnerKey key;
        TDF_Label ownerLabel;
        bool ok = ProbeStageBase(owner, unit, key, ownerLabel, 40, 12, 3);
        fi::OwnerKey fiKey{key.document, key.entity, key.definition};
        fi::ResourceEnvelope envelope;
        if (ok) ok = ProbeEnvelope(ProbeRasterImage(YES), ProbeRasterImage(NO), envelope);
        if (ok) ok = ProbeBindBaseColor(owner, fiKey, ownerLabel, unit, envelope);
        if (ok) {
            fi::Definition committed;
            ok = owner.ReadFaceImageBindings(fiKey, committed, nullptr)
                == fi::persistence::bindings::ReadState::Present;
            NSData *beforeBytes = ok ? ProbeRecordBytes(owner, fiKey) : nil;
            const Handle(TDocStd_Document) doc = owner.Document();
            const auto undosBefore = doc->GetAvailableUndos();
            const fi::Observed observed = ProbeObserved(owner, committed);
            fi::owner::Staging staging;
            fi::owner::Outcome prepared =
                owner.PrepareFaceImageBindings(staging, committed, observed);
            fi::owner::Outcome committedOutcome = fi::owner::Outcome::Malformed;
            if (prepared == fi::owner::Outcome::Prepared) {
                doc->NewCommand();
                committedOutcome = owner.CommitFaceImageBindings(staging, observed);
                if (doc->HasOpenCommand()) doc->AbortCommand();
            }
            // Cancelled prepare: prepare again and discard.
            fi::owner::Outcome cancelled = fi::owner::Outcome::Malformed;
            if (ok) {
                fi::owner::Staging discarded;
                cancelled = owner.PrepareFaceImageBindings(discarded, committed, observed);
                owner.CancelFaceImageBindings(discarded);
            }
            ok = ok && prepared == fi::owner::Outcome::Prepared
                && committedOutcome == fi::owner::Outcome::Refused
                && cancelled == fi::owner::Outcome::Prepared
                && doc->GetAvailableUndos() == undosBefore
                && [(ProbeRecordBytes(owner, fiKey) ?: [NSData data]) isEqualToData:beforeBytes];
        }
        ProbeRecordBit(bits, mask, 6, ok, @"no-op refusal and cancelled prepare leave no trace");
    } catch (...) { ProbeRecordBit(bits, mask, 6, false, @"c++ exception"); }
    } @catch (...) { ProbeRecordBit(bits, mask, 6, false, @"exception"); }
    // bit 7: an aborted command restores the exact prior bytes and history
    // (the document-level recovery measurement).
    @try {
        try {
        core3d::NativeDocumentSession session;
        OcctDocument& owner = *session.Document();
        OwnerKey key;
        TDF_Label ownerLabel;
        bool ok = ProbeStageBase(owner, unit, key, ownerLabel, 40, 12, 3);
        fi::OwnerKey fiKey{key.document, key.entity, key.definition};
        fi::ResourceEnvelope envelope;
        if (ok) ok = ProbeEnvelope(ProbeRasterImage(YES), ProbeRasterImage(NO), envelope);
        if (ok) ok = ProbeBindBaseColor(owner, fiKey, ownerLabel, unit, envelope);
        if (ok) {
            fi::Definition committed;
            ok = owner.ReadFaceImageBindings(fiKey, committed, nullptr)
                == fi::persistence::bindings::ReadState::Present;
            NSData *beforeBytes = ok ? ProbeRecordBytes(owner, fiKey) : nil;
            const Handle(TDocStd_Document) doc = owner.Document();
            const auto undosBefore = doc->GetAvailableUndos();
            fi::Definition edited = committed;
            if (edited.bindings.empty()) ok = false;
            else edited.bindings[0].transform.rotationDegrees = 90;
            ok = ok && fi::BindBindingProof(edited);
            fi::Observed observed;
            if (ok) observed = ProbeObserved(owner, edited);
            fi::owner::Staging staging;
            fi::owner::Outcome outcome = fi::owner::Outcome::Malformed;
            if (ok && owner.PrepareFaceImageBindings(staging, edited, observed)
                    == fi::owner::Outcome::Prepared) {
                doc->NewCommand();
                outcome = owner.CommitFaceImageBindings(staging, observed);
                // The staged record is visible inside the open command...
                NSData *stagedBytes = ProbeRecordBytes(owner, fiKey);
                ok = ok && outcome == fi::owner::Outcome::Committed
                    && stagedBytes && ![stagedBytes isEqualToData:beforeBytes];
                // ...and the abort restores the exact prior bytes and history.
                doc->AbortCommand();
                ok = ok && [(ProbeRecordBytes(owner, fiKey) ?: [NSData data])
                        isEqualToData:beforeBytes]
                    && doc->GetAvailableUndos() == undosBefore;
            } else {
                ok = false;
            }
        }
        ProbeRecordBit(bits, mask, 7, ok, @"aborted command restores exact bytes and history");
    } catch (...) { ProbeRecordBit(bits, mask, 7, false, @"c++ exception"); }
    } @catch (...) { ProbeRecordBit(bits, mask, 7, false, @"exception"); }
    return @{@"schema": @"shapeyard.face-image.probe.v1",
             @"scenario": @"scenario1",
             @"metersPerUnit": @(unit),
             @"mask": @(mask),
             @"bits": bits};
}

// Locate the single SYFR/1 resource blob inside a saved fixture document;
// -2 when ambiguous, -1 when absent.
NSInteger FaceImageResourceBlobOffset(NSData *data) {
    static const std::uint8_t magic[8] = {'S', 'Y', 'F', 'R', 1, 0, 0, 0};
    const auto *bytes = static_cast<const std::uint8_t*>(data.bytes);
    NSInteger found = -1;
    for (NSUInteger at = 0; at + 8 <= data.length; ++at) {
        if (std::memcmp(bytes + at, magic, 8) == 0) {
            if (found >= 0) return -2;
            found = NSInteger(at);
        }
    }
    return found;
}

// Replace 32 lower-hex characters (one UUID field) of the hex-encoded SYFI/1
// record inside the saved document bytes; the record digests no longer match,
// so the strict reader refuses the open. The record persists as fixed
// 256-character chunk strings (the SYFI/1 codec's chunking), so the canonical
// hex is not one contiguous span: locate every chunk's hex text exactly once,
// in order, and patch the field's characters at their per-chunk offsets (a
// field may span a chunk boundary). Returns nil unless every chunk is found
// exactly once.
NSData *FaceImageSpliceRecordHex(NSData *data, const std::vector<std::uint8_t>& canonical,
                                 std::size_t byteOffset, const char *replacementHex32) {
    std::string hex;
    if (!fi::persistence::EncodeHex(canonical, hex)) return nil;
    if (std::strlen(replacementHex32) != 32) return nil;
    const std::size_t chunkCharacters =
        std::size_t(fi::persistence::bindings::ChunkCharacters);
    const std::size_t chunkCount =
        (hex.size() + chunkCharacters - 1) / chunkCharacters;
    if (chunkCount == 0) return nil;
    const auto *bytes = static_cast<const std::uint8_t*>(data.bytes);
    std::vector<NSInteger> chunkPositions(chunkCount, -1);
    for (std::size_t chunk = 0; chunk < chunkCount; ++chunk) {
        const std::string piece = hex.substr(chunk * chunkCharacters,
            std::min(chunkCharacters, hex.size() - chunk * chunkCharacters));
        NSInteger found = -1;
        for (NSUInteger at = 0; at + piece.size() <= data.length; ++at) {
            if (std::memcmp(bytes + at, piece.data(), piece.size()) == 0) {
                if (found >= 0) return nil;
                found = NSInteger(at);
            }
        }
        if (found < 0) return nil;
        if (chunk > 0 && found <= chunkPositions[chunk - 1]) return nil;
        chunkPositions[chunk] = found;
    }
    NSMutableData *mutableData = [data mutableCopy];
    for (std::size_t index = 0; index < 32; ++index) {
        const std::size_t hexPosition = 2 * byteOffset + index;
        if (hexPosition >= hex.size()) return nil;
        const NSInteger at = chunkPositions[hexPosition / chunkCharacters]
            + NSInteger(hexPosition % chunkCharacters);
        [mutableData replaceBytesInRange:NSMakeRange(NSUInteger(at), 1)
                               withBytes:replacementHex32 + index];
    }
    return mutableData;
}

// E3 malformed/budget fixture documents (U23). OCAF-level scenarios tamper
// the committed state before the bounded save; byte-surgery scenarios mutate
// the saved bytes afterwards. The safe storage writer cannot persist a
// non-canonical SYFR/1 table at all (its Prepare raises), so resource-table
// corruptions are produced by byte surgery on a valid saved document.
NSData *CreateFaceImageMalformedFixture(NSString *scenario, double unit) {
    if (unit != 0.001 && unit != 1.0) return nil;
    @try {
        if ([scenario isEqualToString:@"unknownSchema"]
            || [scenario isEqualToString:@"extraAttribute"]
            || [scenario isEqualToString:@"crossDocumentBinding"]) {
            return CreateFaceImageDebugFixture(
                [NSString stringWithFormat:@"e3-malformed-%@-%@",
                    scenario, unit == 0.001 ? @"mm" : @"m"],
                [scenario, unit](const Handle(TDocStd_Document)& document) {
                    StageFaceImageFixture(document, unit);
                    UUID documentID{}; documentID.fill(1);
                    const OwnerKey key{documentID, FixtureIndexedUUID(0x42, 1),
                        FixtureIndexedUUID(0x42, 2)};
                    TDF_Label ownerLabel;
                    if (!fi::owner::ResolveOwnerLabel(document, key, ownerLabel))
                        throw std::invalid_argument("malformed fixture owner");
                    fi::Definition committed;
                    std::vector<std::uint8_t> canonical;
                    TDF_Label recordLabel;
                    if (fi::persistence::bindings::Read(document, ownerLabel, committed,
                            &canonical, &recordLabel)
                            != fi::persistence::bindings::ReadState::Present
                        || recordLabel.IsNull())
                        throw std::invalid_argument("malformed fixture record");
                    if ([scenario isEqualToString:@"unknownSchema"]) {
                        TDataStd_Integer::Set(recordLabel,
                            fi::persistence::bindings::VersionID(), 2);
                    } else if ([scenario isEqualToString:@"extraAttribute"]) {
                        TDataStd_Integer::Set(recordLabel,
                            Standard_GUID("E278B1A5-1E3F-4C2A-8B3D-6F0E9A4C7D99"), 1);
                    } else {
                        // A well-formed record bound to a foreign document
                        // identity: canonical and digest-valid, but the
                        // whole-document admission refuses the open.
                        fi::Definition foreign = committed;
                        foreign.owner.document = FixtureIndexedUUID(0x77, 1);
                        std::vector<std::uint8_t> foreignBytes;
                        std::string hex, digest;
                        if (!fi::Encode(foreign, foreignBytes)
                            || !fi::persistence::EncodeHex(foreignBytes, hex)
                            || !fi::persistence::HashHex(foreignBytes, digest))
                            throw std::invalid_argument("malformed fixture reseal");
                        recordLabel.ForgetAllAttributes(Standard_True);
                        if (!fi::persistence::bindings::WriteChunks(recordLabel, hex,
                                Standard_Integer(foreign.bindings.size()), digest))
                            throw std::invalid_argument("malformed fixture rewrite");
                    }
                });
        }
        if ([scenario isEqualToString:@"duplicateBindingIdentity"]
            || [scenario isEqualToString:@"staleFaceUUID"]
            || [scenario isEqualToString:@"foreignResourceUUID"]) {
            // A second committed binding gives the duplicate case its two
            // rows; every case then patches one hex-encoded UUID field, which
            // breaks the canonical digests and fails the strict reader.
            auto canonicalPtr = std::make_shared<std::vector<std::uint8_t>>();
            NSData *valid = CreateFaceImageDebugFixture(
                [NSString stringWithFormat:@"e3-malformed-%@-%@",
                    scenario, unit == 0.001 ? @"mm" : @"m"],
                [unit, canonicalPtr](const Handle(TDocStd_Document)& document) {
                    StageFaceImageFixture(document, unit);
                    UUID documentID{}; documentID.fill(1);
                    const OwnerKey key{documentID, FixtureIndexedUUID(0x42, 1),
                        FixtureIndexedUUID(0x42, 2)};
                    fi::Definition committed;
                    TDF_Label ownerLabel;
                    if (!fi::owner::ResolveOwnerLabel(document, key, ownerLabel)
                        || fi::persistence::bindings::Read(document, ownerLabel, committed,
                               nullptr, nullptr)
                            != fi::persistence::bindings::ReadState::Present)
                        throw std::invalid_argument("malformed fixture record");
                    fi::Binding added = committed.bindings[0];
                    added.binding = FixtureIndexedUUID(0x53, 1);
                    added.role = fi::Role::Emissive;
                    added.colorSpace = fi::ColorSpace::SRGB;
                    fi::Definition next = committed;
                    next.bindings.push_back(added);
                    if (!fi::BindBindingProof(next))
                        throw std::invalid_argument("malformed fixture binding proof");
                    fi::Observed observed;
                    for (const auto& binding : next.bindings)
                        observed.faces.push_back({binding.face, binding.selectorProof});
                    if (!fi::owner::ResourceManifest(document, observed.resources))
                        throw std::invalid_argument("malformed fixture manifest");
                    document->NewCommand();
                    fi::owner::Staging staging;
                    if (fi::owner::Prepare(staging, document, next, observed)
                            != fi::owner::Outcome::Prepared
                        || fi::owner::Commit(staging, document, observed)
                            != fi::owner::Outcome::Committed
                        || !document->CommitCommand()) {
                        fi::owner::Cancel(staging);
                        if (document->HasOpenCommand()) document->AbortCommand();
                        throw std::invalid_argument("malformed fixture second binding");
                    }
                    // Capture the committed record's canonical bytes for the
                    // post-save hex surgery below.
                    fi::Definition committedNow;
                    if (fi::persistence::bindings::Read(document, ownerLabel, committedNow,
                            canonicalPtr.get(), nullptr)
                            != fi::persistence::bindings::ReadState::Present
                        || committedNow != next || !Core3DValidateFaceImageDocument(document))
                        throw std::invalid_argument("malformed fixture canonical capture");
                });
            if (!valid || canonicalPtr->empty()) return nil;
            const std::vector<std::uint8_t>& canonical = *canonicalPtr;
            // SYFI/1 record layout (FaceImageDefinition.hxx): binding rows
            // begin at byte 60; row i field offsets: binding +0, face +16,
            // resource +64. Hex text is two characters per byte.
            if ([scenario isEqualToString:@"duplicateBindingIdentity"]) {
                // Row 1's binding identity becomes row 0's: bytes [60+124, +16)
                // copied from [60, +16).
                std::string hex;
                if (!fi::persistence::EncodeHex(canonical, hex)) return nil;
                const std::string row0Binding = hex.substr(2 * 60, 32);
                return FaceImageSpliceRecordHex(valid, canonical, 60 + 124,
                                                row0Binding.c_str());
            }
            if ([scenario isEqualToString:@"staleFaceUUID"]) {
                // Flip the first hex character of row 0's face identity.
                std::string hex;
                if (!fi::persistence::EncodeHex(canonical, hex)) return nil;
                std::string replacement = hex.substr(2 * (60 + 16), 32);
                replacement[0] = replacement[0] == '0' ? '1' : '0';
                return FaceImageSpliceRecordHex(valid, canonical, 60 + 16,
                                                replacement.c_str());
            }
            // foreignResourceUUID: flip the first hex character of row 0's
            // resource identity.
            std::string hex;
            if (!fi::persistence::EncodeHex(canonical, hex)) return nil;
            std::string replacement = hex.substr(2 * (60 + 64), 32);
            replacement[0] = replacement[0] == '0' ? '1' : '0';
            return FaceImageSpliceRecordHex(valid, canonical, 60 + 64,
                                            replacement.c_str());
        }
        if ([scenario isEqualToString:@"exactLimitResource"]) {
            // Exactly 16,777,216 pixels (4096 x 4096), the pixel cap: admitted.
            NSData *flat = ProbeFlatPNG(4096, 4096);
            if (!flat) return nil;
            fi::ResourceEnvelope envelope;
            if (!fi::validation::BuildFaceImageEnvelope(flat, flat, @"opaque",
                    [@"shapeyard.e3.fixture.exact-limit.v1"
                        dataUsingEncoding:NSUTF8StringEncoding],
                    FixtureIndexedUUID(0x51, 1), envelope)) return nil;
            return CreateFaceImageDebugFixture(
                unit == 0.001 ? @"e3-exact-limit-mm" : @"e3-exact-limit-m",
                [envelope, unit](const Handle(TDocStd_Document)& document) {
                    StageFaceImageFixtureSized(document, unit, 40, 12, 3, envelope);
                });
        }
        // Byte-surgery scenarios start from a valid saved document.
        NSData *valid = CreateFaceImageDebugFixture(
            unit == 0.001 ? @"e3-malformed-base-mm" : @"e3-malformed-base-m",
            [unit](const Handle(TDocStd_Document)& document) {
                StageFaceImageFixture(document, unit);
            });
        if (!valid) return nil;
        if ([scenario isEqualToString:@"trailingBytes"]) {
            NSMutableData *mutated = [valid mutableCopy];
            static const std::uint8_t garbage[32] = {0xA5};
            [mutated appendBytes:garbage length:sizeof(garbage)];
            return mutated;
        }
        const NSInteger blobAt = FaceImageResourceBlobOffset(valid);
        if (blobAt < 12) return nil;
        if ([scenario isEqualToString:@"digestMismatch"]) {
            // Flip one byte inside the originalBytes region of the SYFR/1
            // blob (past the 140-byte header and the 4-byte count): the
            // content digest no longer matches the carried bytes.
            NSMutableData *mutated = [valid mutableCopy];
            auto *bytes = static_cast<std::uint8_t*>(mutated.mutableBytes);
            const NSUInteger at = NSUInteger(blobAt) + 144;
            if (at >= mutated.length) return nil;
            bytes[at] ^= 0xFF;
            return mutated;
        }
        if ([scenario isEqualToString:@"overLimitResource"]) {
            // Inflate the driver-level record length prefix past the
            // kMaximumEnvelopeBytes cap; the bounded reader refuses before
            // allocation. The integer endianness is discovered from the known
            // canonical record size.
            std::vector<std::uint8_t> canonical;
            if (!fi::Encode(FixturePNGEnvelope(), canonical)) return nil;
            NSMutableData *mutated = [valid mutableCopy];
            auto *bytes = static_cast<std::uint8_t*>(mutated.mutableBytes);
            const NSUInteger at = NSUInteger(blobAt) - 4;
            std::uint32_t little = 0, big = 0;
            std::memcpy(&little, bytes + at, 4);
            for (int index = 0; index < 4; ++index)
                big = (big << 8) | bytes[at + index];
            const std::uint32_t over = std::uint32_t(fi::kMaximumEnvelopeBytes) + 1;
            if (little == canonical.size()) {
                std::memcpy(bytes + at, &over, 4);
            } else if (big == canonical.size()) {
                for (int index = 0; index < 4; ++index)
                    bytes[at + index] = std::uint8_t(over >> (8 * (3 - index)));
            } else {
                return nil;
            }
            return mutated;
        }
        return nil;
    } @catch (...) { return nil; }
}

} // namespace
#endif

@implementation Core3DViewController (FaceImageOpenings)
- (Core3DFaceImageOpening *)openFaceImageEditor {
    ControllerFaceImageInput input;
    return CaptureControllerFaceImageInput(self, nil, input)
        ? [[Core3DFaceImageOpening alloc] initWithInput:std::move(input)] : nil;
}

- (Core3DFaceImageOpening *)openFaceImageOpeningForEntityIdentifier:(NSString *)entityIdentifier {
    ControllerFaceImageInput input;
    return CaptureControllerFaceImageInput(self, entityIdentifier, input)
        ? [[Core3DFaceImageOpening alloc] initWithInput:std::move(input)] : nil;
}
#if DEBUG
+ (NSData *)debugE4RetainedCapFixtureAssetDataWithResourceBytes:(NSData *)resourceBytes
                                                 metersPerUnit:(NSNumber *)metersPerUnit {
    const double unit = metersPerUnit.doubleValue;
    if (!NSThread.isMainThread || resourceBytes.length == 0
        || (unit != 0.001 && unit != 1.0)) return nil;
    fi::ResourceEnvelope envelope;
    NSData *provenance = [@"shapeyard.e4.retained-cap.fixture.v1"
        dataUsingEncoding:NSUTF8StringEncoding];
    if (!fi::validation::BuildFaceImageEnvelope(
            resourceBytes, resourceBytes, @"straight", provenance,
            FixtureIndexedUUID(0x73, 1), envelope)) return nil;
    return CreateFaceImageDebugFixture(
        unit == 0.001 ? @"e4-retained-cap-mm" : @"e4-retained-cap-m",
        [envelope, unit](const Handle(TDocStd_Document)& document) {
            StageE4RetainedCapFixture(document, unit, {envelope});
        });
}

+ (NSData *)debugE4RetainedCapRoleFixtureAssetDataWithResourceBytes:
    (NSArray<NSData *> *)resourceBytes metersPerUnit:(NSNumber *)metersPerUnit {
    const double unit = metersPerUnit.doubleValue;
    if (!NSThread.isMainThread || resourceBytes.count != 3
        || (unit != 0.001 && unit != 1.0)) return nil;
    std::vector<fi::ResourceEnvelope> envelopes;
    envelopes.reserve(3);
    for (NSUInteger index = 0; index < resourceBytes.count; ++index) {
        NSData *bytes = resourceBytes[index];
        if (bytes.length == 0) return nil;
        fi::ResourceEnvelope envelope;
        NSData *provenance = [[NSString stringWithFormat:
            @"shapeyard.e4.retained-cap.roles.fixture.v1.%lu",
            (unsigned long)index] dataUsingEncoding:NSUTF8StringEncoding];
        if (!fi::validation::BuildFaceImageEnvelope(
                bytes, bytes, @"opaque", provenance,
                FixtureIndexedUUID(0x73, std::uint8_t(index + 1)),
                envelope)) return nil;
        envelopes.push_back(std::move(envelope));
    }
    return CreateFaceImageDebugFixture(
        unit == 0.001 ? @"e4-retained-cap-roles-mm"
                      : @"e4-retained-cap-roles-m",
        [envelopes, unit](const Handle(TDocStd_Document)& document) {
            StageE4RetainedCapFixture(document, unit, envelopes);
        });
}

- (NSNumber *)debugInstallE4RetainedCapLayerForEntityIdentifier:
    (NSString *)entityIdentifier {
    if (!NSThread.isMainThread || entityIdentifier.length == 0
        || entityIdentifier.length > 128) return @NO;
    @try {
        GLViewController *gl = [self.glController
            isKindOfClass:GLViewController.class]
            ? (GLViewController *)self.glController : nil;
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl ? gl.viewer : nullptr;
        const Handle(OcctDocument) wrapper = viewer
            ? viewer->getDocument() : Handle(OcctDocument)();
        OwnerKey key; TDF_Label owner;
        const char *raw = entityIdentifier.UTF8String;
        if (wrapper.IsNull()
            || !LabelForSelected(wrapper, raw ? raw : "", key, owner)) return @NO;
        const Handle(TDocStd_Document)& document = wrapper->Document();
        if (document.IsNull() || document->HasOpenCommand()) return @NO;

        core3d::retained_edge_treatment::ReplayBudget budget;
        core3d::decal_layer::source::Witness witness;
        if (!core3d::decal_layer::source::CaptureSource(
                wrapper, key, budget, [] { return false; }, witness))
            return @NO;
        const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(owner);
        Standard_Real unit = 0.0;
        dr::FaceImageGeometricReceipt receipt;
        if (shape.IsNull()
            || !XCAFDoc_DocumentTool::GetLengthUnit(document, unit)
            || !FixtureCapturePlanarFace(shape, unit,
                core3d::retained_face_selector::Axis::Z,
                core3d::retained_face_selector::Side::Max, receipt))
            return @NO;
        core3d::decal_layer::Digest selectorProof{};
        if (!dr::FaceImageReceiptProof(receipt, selectorProof))
            return @NO;
        std::vector<fi::ResourceEnvelope> envelopes;
        for (std::uint8_t slot = 1; slot <= 3; ++slot) {
            fi::ResourceEnvelope envelope;
            if (!fi::owner::ReadResource(
                    document, FixtureIndexedUUID(0x73, slot), envelope)) {
                if (slot == 1) return @NO;
                break;
            }
            envelopes.push_back(std::move(envelope));
        }

        core3d::decal_layer::Definition definition;
        definition.owner = key;
        definition.sourceRecipe = witness.sourceRecipe;
        definition.sourceRevision = witness.sourceRevision;
        definition.geometryRevision = witness.geometryRevision;
        definition.placementRevision = witness.placementRevision;
        definition.sourceProof = witness.sourceProof;
        const std::array<fi::Role, 5> roles{{
            fi::Role::BaseColor, fi::Role::Emissive,
            fi::Role::MetallicRoughness, fi::Role::Occlusion,
            fi::Role::Normal}};
        const std::array<std::size_t, 5> resources{{0, 0, 1, 1, 2}};
        const std::size_t roleCount = envelopes.size() == 3 ? roles.size() : 1;
        for (std::size_t index = 0; index < roleCount; ++index) {
            const auto& envelope = envelopes[resources[index]];
            core3d::decal_layer::Layer layer;
            layer.identifier = FixtureIndexedUUID(
                0x74, std::uint8_t(index + 1));
            layer.image.resource = envelope.resource;
            layer.image.normalizedContent = envelope.workingContent;
            layer.image.originalContent = envelope.originalContent;
            layer.image.provenance = envelope.provenance;
            layer.image.producerVersion =
                core3d::decal_layer::image_contract::kProducerVersion;
            layer.image.mediaType = envelope.workingFormat;
            layer.image.widthTexels = envelope.workingWidthTexels;
            layer.image.heightTexels = envelope.workingHeightTexels;
            layer.image.role = roles[index];
            layer.image.colorSpace = index < 2
                ? fi::ColorSpace::SRGB : fi::ColorSpace::Linear;
            layer.image.alpha = envelope.alpha;
            layer.placement.kind = core3d::decal_layer::PlacementKind::Face;
            layer.placement.edgePolicy =
                core3d::decal_layer::EdgePolicy::RejectCrossing;
            layer.placement.expectedCardinality = 1;
            layer.placement.face.receiver.face = FixtureIndexedUUID(0x73, 3);
            layer.placement.face.receiver.selectorProof = selectorProof;
            layer.placement.face.anchorMeters = {{0.05, 0.04}};
            layer.widthMeters = 0.05;
            layer.heightMeters = 0.04;
            layer.opacity = 1.0;
            definition.layers.push_back(std::move(layer));
        }
        std::vector<std::uint8_t> bytes;
        std::string hex, digest;
        if (!core3d::decal_layer::BindLayerProof(definition)
            || !core3d::decal_layer::Encode(definition, bytes)
            || !core3d::decal_layer::persistence::EncodeHex(bytes, hex)
            || !core3d::decal_layer::persistence::DigestHex(bytes, digest))
            return @NO;
        document->NewCommand();
        const TDF_Label record = owner.FindChild(
            core3d::decal_layer::persistence::RecordTag, Standard_True);
        if (!core3d::decal_layer::persistence::WriteChunks(
                record, hex, Standard_Integer(definition.layers.size()), digest)
            || !document->CommitCommand()) {
            document->AbortCommand(); return @NO;
        }
        core3d::decal_layer::Definition strict;
        std::vector<std::uint8_t> strictBytes;
        const auto strictState = core3d::decal_layer::persistence::Read(
            document, owner, strict, &strictBytes, nullptr);
        const bool bytesMatch = strictBytes == bytes;
        const bool sourceMatch =
            core3d::decal_layer::source::ExactMatch(strict, witness);
        return @(strictState
                == core3d::decal_layer::persistence::ReadState::Present
            && bytesMatch && sourceMatch);
    } @catch (...) { return @NO; }
}

- (NSDictionary<NSString *,id> *)debugE4RetainedCapObservationForEntityIdentifier:
    (NSString *)entityIdentifier {
    if (!NSThread.isMainThread || entityIdentifier.length == 0
        || entityIdentifier.length > 128) return nil;
    @try {
        GLViewController *gl = [self.glController
            isKindOfClass:GLViewController.class]
            ? (GLViewController *)self.glController : nil;
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl ? gl.viewer : nullptr;
        const Handle(OcctDocument) wrapper = viewer
            ? viewer->getDocument() : Handle(OcctDocument)();
        OwnerKey key; TDF_Label owner;
        const char *raw = entityIdentifier.UTF8String;
        if (wrapper.IsNull()
            || !LabelForSelected(wrapper, raw ? raw : "", key, owner)) return nil;
        const Handle(TDocStd_Document)& document = wrapper->Document();
        core3d::decal_layer::Definition definition;
        std::vector<std::uint8_t> canonical;
        if (core3d::decal_layer::persistence::Read(
                document, owner, definition, &canonical, nullptr)
            != core3d::decal_layer::persistence::ReadState::Present)
            return nil;
        core3d::retained_edge_treatment::ReplayBudget budget;
        core3d::decal_layer::source::Witness witness;
        if (!core3d::decal_layer::source::CaptureSource(
                wrapper, key, budget, [] { return false; }, witness)
            || definition.layers.size() != 1) return nil;
        const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(owner);
        Standard_Real unit = 0.0;
        dr::FaceImageGeometricReceipt receipt;
        if (shape.IsNull()
            || !XCAFDoc_DocumentTool::GetLengthUnit(document, unit)
            || !FixtureCapturePlanarFace(shape, unit,
                core3d::retained_face_selector::Axis::Z,
                core3d::retained_face_selector::Side::Max, receipt)) return nil;
        core3d::decal_layer::Digest proof{};
        const auto *intent = std::get_if<
            core3d::retained_face_selector::PlanarFaceBoundary>(
                &receipt.intent);
        if (!intent || !dr::FaceImageReceiptProof(receipt, proof)
            || proof != definition.layers.front().placement.face.receiver.selectorProof)
            return nil;
        Bnd_Box box; BRepBndLib::AddOptimal(shape, box, Standard_False, Standard_False);
        Standard_Real x0=0,y0=0,z0=0,x1=0,y1=0,z1=0;
        if (box.IsVoid() || box.IsWhole() || box.IsOpen()) return nil;
        box.Get(x0,y0,z0,x1,y1,z1);
        std::string canonicalDigest;
        if (!core3d::decal_layer::persistence::DigestHex(
                canonical, canonicalDigest)) return nil;
        return @{
            @"schema": @"shapeyard.e4-retained-cap.observation.v1",
            @"recordState": @"present",
            @"exactSourceMatch": @(core3d::decal_layer::source::ExactMatch(
                definition, witness)),
            @"axis": @"z", @"side": @"max",
            @"intent": @"planarFaceBoundary", @"edgeKind": @"line",
            @"coverage": @"entireBoundary",
            @"expectedCount": @(intent->expectedCount),
            @"wireCount": @(receipt.wireCount),
            @"boundaryUseCount": @(receipt.boundaryUseCount),
            @"boundaryUniqueEdgeCount": @(receipt.boundaryUniqueEdgeCount),
            @"widthMM": @((x1-x0)*unit*1000.0),
            @"heightMM": @((y1-y0)*unit*1000.0),
            @"depthMM": @((z1-z0)*unit*1000.0),
            @"canonicalByteCount": @(canonical.size()),
            @"canonicalDigest": [NSString stringWithUTF8String:
                canonicalDigest.c_str()] ?: @"",
            @"sourceProof": DigestText(witness.sourceProof),
        };
    } @catch (...) { return nil; }
}

+ (NSData *)debugFaceImageFixtureAssetData:(double)metersPerUnit {
    if (!NSThread.isMainThread || (metersPerUnit != 0.001 && metersPerUnit != 1.0))
        return nil;
    return CreateFaceImageDebugFixture(
        metersPerUnit == 0.001 ? @"e3-face-image-mm" : @"e3-face-image-m",
        [metersPerUnit](const Handle(TDocStd_Document)& document) {
            StageFaceImageFixture(document, metersPerUnit);
        });
}

+ (NSData *)debugFaceImageFixtureAssetDataWithOriginalBytes:(NSData *)originalBytes
                                               workingBytes:(NSData *)workingBytes
                                        alphaInterpretation:(NSString *)alphaInterpretation
                                                 provenance:(NSData *)provenance
                                              metersPerUnit:(double)metersPerUnit {
    if (!NSThread.isMainThread || (metersPerUnit != 0.001 && metersPerUnit != 1.0)) {
        return nil;
    }
    fi::ResourceEnvelope envelope;
    if (!fi::validation::BuildFaceImageEnvelope(
            originalBytes, workingBytes, alphaInterpretation,
            provenance, FixtureIndexedUUID(0x51, 1), envelope)) {
        return nil;
    }
    return CreateFaceImageDebugFixture(
        metersPerUnit == 0.001 ? @"e3-face-image-bytes-mm" : @"e3-face-image-bytes-m",
        [envelope, metersPerUnit](const Handle(TDocStd_Document)& document) {
            StageFaceImageFixtureSized(document, metersPerUnit, 40, 12, 3, envelope);
        });
}

- (NSDictionary<NSString *,id> *)debugFaceImageObservationForEntityIdentifier:(NSString *)entityIdentifier {
    if (!NSThread.isMainThread || entityIdentifier.length == 0
        || entityIdentifier.length > 128) return nil;
    @try {
        GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)self.glController : nil;
        if (!gl) return nil;
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl.viewer;
        if (!viewer) return nil;
        const Handle(OcctDocument) owner = viewer->getDocument();
        if (owner.IsNull() || owner->Document().IsNull()) return nil;
        const Handle(TDocStd_Document) document = owner->Document();
        OwnerKey key;
        TDF_Label label;
        const char *raw = entityIdentifier.UTF8String;
        if (!LabelForSelected(owner, raw ? raw : "", key, label)) return nil;
        fi::Definition definition;
        std::vector<std::uint8_t> bytes;
        const auto state = owner->ReadFaceImageBindings(key, definition, &bytes);
        const fi::FaceImageProbe::Observation probe = fi::FaceImageProbe::Observe(document);
        NSString *sourceFeature = @"";
        core3d::retained_solid::Record retained;
        if (core3d::retained_solid::Read(document, label, retained) && retained.value) {
            sourceFeature = IdentifierText(
                core3d::retained_boolean::Identities(retained.value->envelope).sourceFeature);
        }
        NSString *currentness = @"absent";
        if (state == fi::persistence::bindings::ReadState::Present) {
            dr::FaceImageAttachment attachment;
            currentness = dr::CaptureFaceImageAttachment(*owner, key, attachment)
                    == dr::FaceImageReplayStatus::Captured ? @"current" : @"stale";
        } else if (state == fi::persistence::bindings::ReadState::Malformed) {
            currentness = @"malformed";
        }
        NSMutableArray<NSDictionary<NSString *,id> *> *bindings =
            [NSMutableArray arrayWithCapacity:definition.bindings.size()];
        for (const auto& binding : definition.bindings) {
            [bindings addObject:@{
                @"bindingIdentifier": IdentifierText(binding.binding),
                @"faceIdentifier": IdentifierText(binding.face),
                @"resourceIdentifier": IdentifierText(binding.resource),
                @"selectorProof": DigestText(binding.selectorProof),
                @"role": RoleText(binding.role),
                @"colorSpace": ColorSpaceText(binding.colorSpace),
                @"scaleU": @(binding.transform.scale[0]),
                @"scaleV": @(binding.transform.scale[1]),
                @"offsetU": @(binding.transform.offset[0]),
                @"offsetV": @(binding.transform.offset[1]),
                @"rotationDegrees": @(binding.transform.rotationDegrees),
                @"wrapU": WrapText(binding.transform.wrapU),
                @"wrapV": WrapText(binding.transform.wrapV),
            }];
        }
        NSString *recordState = @"malformed";
        if (state == fi::persistence::bindings::ReadState::Present) recordState = @"present";
        else if (state == fi::persistence::bindings::ReadState::Absent) recordState = @"absent";
        return @{
            @"schema": @"shapeyard.face-image.observation.v1",
            @"documentIdentifier": [NSString stringWithUTF8String:
                owner->DocumentIdentifier().c_str()] ?: @"",
            @"entityIdentifier": IdentifierText(key.entity),
            @"definitionIdentifier": IdentifierText(key.definition),
            @"sourceFeatureIdentifier": sourceFeature,
            @"recordState": recordState,
            @"recordBytes": bytes.empty()
                ? [NSData data] : [NSData dataWithBytes:bytes.data() length:bytes.size()],
            @"currentness": currentness,
            @"bindings": bindings,
            @"resources": @(probe.resources),
            @"boundOwners": @(probe.boundOwners),
            @"aggregateBytes": @(probe.aggregateBytes),
            @"probeComplete": @(probe.complete ? YES : NO),
            @"undoCount": @(document->GetAvailableUndos()),
            @"redoCount": @(document->GetAvailableRedos()),
            @"undoLimit": @(document->GetUndoLimit()),
            @"hasOpenCommand": @(document->HasOpenCommand() ? YES : NO),
            @"recoveryState": document->HasOpenCommand() ? @"openCommand" : @"clean",
        };
    } @catch (...) { return nil; }
}

+ (NSDictionary<NSString *,id> *)debugFaceImageScenarioZeroProbe:(double)metersPerUnit {
    if (!NSThread.isMainThread || (metersPerUnit != 0.001 && metersPerUnit != 1.0))
        return nil;
    return FaceImageScenarioZeroProbe(metersPerUnit);
}

+ (NSDictionary<NSString *,id> *)debugFaceImageScenarioOneProbe:(double)metersPerUnit {
    if (!NSThread.isMainThread || (metersPerUnit != 0.001 && metersPerUnit != 1.0))
        return nil;
    return FaceImageScenarioOneProbe(metersPerUnit);
}

+ (NSData *)debugFaceImageMalformedFixtureAssetData:(NSString *)scenario
                                      metersPerUnit:(double)metersPerUnit {
    if (!NSThread.isMainThread || ![scenario isKindOfClass:NSString.class]
        || scenario.length == 0 || scenario.length > 64) return nil;
    return CreateFaceImageMalformedFixture(scenario, metersPerUnit);
}

+ (NSData *)debugFaceImageSplitMergeFixtureAssetData:(double)metersPerUnit {
    if (!NSThread.isMainThread || (metersPerUnit != 0.001 && metersPerUnit != 1.0))
        return nil;
    // Width 44 with the pilot cut interior (x 39..43); editing the in-plane
    // width down to 40 mm drives the cut across the bound max-X side and
    // splits it into two coplanar faces.
    return CreateFaceImageDebugFixture(
        metersPerUnit == 0.001 ? @"e3-face-image-split-mm" : @"e3-face-image-split-m",
        [metersPerUnit](const Handle(TDocStd_Document)& document) {
            StageFaceImageFixtureSized(document, metersPerUnit, 44, 41, 2,
                                       FixturePNGEnvelope());
        });
}

+ (NSData *)debugFaceImageUnsupportedDownstreamFixtureAssetData:(double)metersPerUnit {
    if (!NSThread.isMainThread || (metersPerUnit != 0.001 && metersPerUnit != 1.0))
        return nil;
    return CreateFaceImageDebugFixture(
        metersPerUnit == 0.001 ? @"e3-face-image-downstream-mm" : @"e3-face-image-downstream-m",
        [metersPerUnit](const Handle(TDocStd_Document)& document) {
            StageFaceImageFixture(document, metersPerUnit);
            UUID documentID{}; documentID.fill(1);
            const OwnerKey sourceKey{documentID, FixtureIndexedUUID(0x42, 1),
                FixtureIndexedUUID(0x42, 2)};
            const OwnerKey resultKey{documentID, FixtureIndexedUUID(0x42, 11),
                FixtureIndexedUUID(0x42, 12)};
            const TDF_Label resultLabel = FixtureAddFaceImagePartSized(
                document, resultKey, metersPerUnit, 40, 12, 3);
            if (resultLabel.IsNull()
                || !FixtureStagePatternDependency(document, sourceKey,
                    FixturePartFeature(sourceKey, 3), resultKey, metersPerUnit)
                || !Core3DValidateFaceImageDocument(document))
                throw std::invalid_argument("face image downstream fixture");
        });
}
#endif
@end
