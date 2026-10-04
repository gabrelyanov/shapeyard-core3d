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
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/ReceiptRecord.hxx"
#include "../OCCTKit/RetainedSolidAttribute.hxx"
#include "Core3DViewer.h"

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#if DEBUG
// DEBUG fixture construction only: one literal retained box with a real
// retained-solid record, one adopted E3 image resource and one committed
// face-image binding, mirroring the row-278a atlas fixture seam.
#include "../OCCTKit/NativeOpeningSurfaceProbe.hxx"
#include "../OCCTKit/ProfilePersistence.hxx"
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepBuilderAPI_Copy.hxx>
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
        BRepBndLib::Add(shape, stageBox);
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

#if DEBUG
namespace {
UUID FixtureIndexedUUID(std::uint8_t seed, std::size_t index) {
    UUID value{}; value.fill(seed);
    value[0] = std::uint8_t(index); value[1] = std::uint8_t(index >> 8);
    return value;
}

bool FixtureSetUUID(const TDF_Label& label, const char* attributeID, const UUID& value) {
    return !TDataStd_AsciiString::Set(label, Standard_GUID(attributeID),
        TCollection_AsciiString(core3d::retained_solid::UUIDText(value).c_str())).IsNull();
}

// One literal 40x30x20 mm box with a production saved-cut recipe, a stored
// triangulation and the real retained-solid seed record, mirroring the
// row-278a fixture part seam (kind 0) so the owner satisfies the
// retained-carrier admission.
TDF_Label FixtureAddFaceImagePart(const Handle(TDocStd_Document)& doc,
                                  const OwnerKey& key, double unit) {
    if (doc.IsNull() || doc->HasOpenCommand())
        throw std::invalid_argument("face image fixture command");
    const double n = .001 / unit;
    TopoDS_Shape shape = BRepPrimAPI_MakeBox(40 * n, 30 * n, 20 * n).Shape();
    core3d::retained_boolean::Program program;
    program.source.document = key.document;
    program.source.entity = key.entity;
    program.source.definition = key.definition;
    program.source.sourceFeature = FixtureIndexedUUID(0x42, 3);
    program.source.derivedFeature = FixtureIndexedUUID(0x42, 4);
    program.source.family = 1;
    program.source.metersPerUnit = unit;
    core3d::profile::Parameters source;
    source.metersPerUnit = unit;
    source.definition.depth = 20 * n;
    source.definition.points = {{0, 0}, {40 * n, 0}, {40 * n, 30 * n}, {0, 30 * n}};
    if (!core3d::profile::Encode(source, program.source.values))
        throw std::invalid_argument("face image fixture source recipe");
    program.source.schema = std::uint32_t(core3d::profile::SchemaFor(source));
    core3d::retained_boolean::Step pilot;
    pilot.operand.identifier = 1;
    pilot.operand.axis = core3d::analytic_boolean::Axis::Z;
    pilot.operand.point = std::array<double, 3>{{12 * n, 15 * n, 0}};
    pilot.operand.radius = 3 * n;
    program.steps = {pilot};
    program.nextOperandID = 2;
    if (!core3d::retained_boolean::Valid(program))
        throw std::invalid_argument("face image fixture program");
    core3d::saved_cut_source_edit::Patch editableSource =
        core3d::saved_cut_source_values::PolygonPatch{};
    if (!core3d::saved_boolean_build::SourcePatch(program, editableSource))
        throw std::invalid_argument("face image fixture source editing");
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
            doc, owner, program, shape, retainedBase) || !doc->CommitCommand())
        throw std::invalid_argument("face image fixture retained admission");
    return owner;
}

NSData *FixturePNGData() {
    return [[NSData alloc] initWithBase64EncodedString:
        @"iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAYAAACqaXHeAAAApklEQVR42u3aQQ3AQAzEwFyZF3kKY06qTWAtK8+cndmBnJ1X7j9y/AYKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNAXQApoCaAFNAbSApgBaQFMALaApgBbQnJml/wF2vQsoQAG0gKYAWkBTAC2gKYAW0BRAC2gKoAU0BdACmgJoAU0BtICmAFtAUQAtoCqAFNL8P8AESbQf6Ta5RUwAAAABJRU5ErkJggg=="
        options:0];
}

// The frozen portion-3 fixture: one adopted 64x64 PNG resource (original and
// working bytes deliberately identical here; the importer distinction is
// portion 5's) and one committed BaseColor binding on the box's max-X planar
// side with a non-default transform and both non-Repeat wrap modes.
void StageFaceImageFixture(const Handle(TDocStd_Document)& doc, double unit) {
    doc->ChangeStorageFormatVersion(TDocStd_FormatVersion(12));
    (void)XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    XCAFDoc_DocumentTool::SetLengthUnit(doc, unit);
    UUID documentID{}; documentID.fill(1);
    if (!FixtureSetUUID(doc->Main(), "74386E4E-F620-498F-8092-E6D883AF33A4", documentID))
        throw std::invalid_argument("face image fixture document identity");
    doc->SetUndoLimit(40); doc->ClearUndos();
    const OwnerKey key{documentID, FixtureIndexedUUID(0x42, 1), FixtureIndexedUUID(0x42, 2)};
    const TDF_Label ownerLabel = FixtureAddFaceImagePart(doc, key, unit);

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
    doc->NewCommand();
    if (fi::owner::AdoptResource(doc, envelope) != fi::owner::Outcome::Committed
        || !doc->CommitCommand())
        throw std::invalid_argument("face image fixture resource adoption");

    // Capture the max-X planar side through the real derivation + B2 resolver.
    const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(ownerLabel);
    if (shape.IsNull()) throw std::invalid_argument("face image fixture shape");
    TopTools_IndexedMapOfShape faceMap;
    TopExp::MapShapes(shape, TopAbs_FACE, faceMap);
    Bnd_Box stageBox;
    BRepBndLib::Add(shape, stageBox);
    core3d::retained_edge_treatment::ReplayBudget budget;
    dr::FaceImageGeometricReceipt captured;
    bool found = false;
    for (int index = 1; index <= faceMap.Extent() && !found; ++index) {
        const TopoDS_Face face = TopoDS::Face(faceMap.FindKey(index));
        dr::FaceImageGeometricReceipt derived;
        if (dr::detail::DeriveFaceImageReceipt(stageBox, face, unit, budget, derived)
            != dr::detail::FaceImageDeriveStatus::Derived) continue;
        const auto *scope = std::get_if<core3d::retained_face_selector::PlanarFaceBoundary>(
            &derived.intent);
        if (!scope || scope->face.axis != core3d::retained_face_selector::Axis::X
            || scope->face.side != core3d::retained_face_selector::Side::Max) continue;
        dr::FaceImageGeometricReceipt resolved; TopoDS_Face matched;
        if (dr::detail::ResolveFaceImageReceipt(shape, derived.intent, unit, budget,
                resolved, matched) != core3d::retained_face_selector::Refusal::None
            || !matched.IsSame(face)) continue;
        captured = resolved; found = true;
    }
    if (!found) throw std::invalid_argument("face image fixture face capture");
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
    if (fi::owner::Prepare(staging, doc, candidate, observed) != fi::owner::Outcome::Prepared
        || fi::owner::Commit(staging, doc, observed) != fi::owner::Outcome::Committed
        || !doc->CommitCommand()) {
        fi::owner::Cancel(staging);
        doc->AbortCommand();
        throw std::invalid_argument("face image fixture binding commit");
    }
    doc->ClearUndos();
    if (!Core3DValidateFaceImageDocument(doc))
        throw std::invalid_argument("face image fixture validation");
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
+ (NSData *)debugFaceImageFixtureAssetData:(double)metersPerUnit {
    if (!NSThread.isMainThread || (metersPerUnit != 0.001 && metersPerUnit != 1.0))
        return nil;
    return CreateFaceImageDebugFixture(
        metersPerUnit == 0.001 ? @"e3-face-image-mm" : @"e3-face-image-m",
        [metersPerUnit](const Handle(TDocStd_Document)& document) {
            StageFaceImageFixture(document, metersPerUnit);
        });
}
#endif
@end
