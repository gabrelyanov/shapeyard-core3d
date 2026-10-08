#import "Core3DAuthoredParameterOpening.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/AuthoredParameterNativeOwner.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "Core3DViewer.h"

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <cmath>
#include <memory>

namespace {
using NativeOwner = core3d::authored_parameter::Owner;

NSString *AuthoredIdentifier(const core3d::retained_recipe::UUID& value) {
    return [[NSUUID alloc] initWithUUIDBytes:value.data()].UUIDString;
}

bool SameExpectedScene(Core3DSceneSnapshot *expected, Core3DSceneSnapshot *current,
                       NSString *entity) noexcept {
    if (!expected || !current || !entity
        || ![expected.publicationSourceIdentifier isEqualToString:
             current.publicationSourceIdentifier]
        || expected.revisions.documentGeneration != current.revisions.documentGeneration
        || expected.revisions.modelRevision != current.revisions.modelRevision
        || expected.revisions.presentationRevision != current.revisions.presentationRevision
        || current.selectionMode != Core3DSceneElementKindObject
        || current.selection.selectedElements.count != 1) return false;
    Core3DSceneElementIdentifier *selected = current.selection.selectedElements.firstObject;
    Core3DSceneElementIdentifier *observed = expected.selection.selectedElements.firstObject;
    return expected.selection.selectedElements.count == 1
        && selected.kind == Core3DSceneElementKindObject
        && observed.kind == selected.kind
        && [selected.entityIdentifier isEqualToString:entity]
        && [observed.entityIdentifier isEqualToString:entity]
        && observed.topologyIndex == selected.topologyIndex
        && observed.geometryRevision == selected.geometryRevision;
}

TDF_Label ExactRoot(const Handle(OcctDocument)& document,
                    const std::string& entity) noexcept {
    try {
        if (document.IsNull() || document->Document().IsNull() || entity.empty()) return {};
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Document()->Main());
        if (shapes.IsNull()) return {};
        TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
        TDF_Label result; unsigned matches = 0;
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            if (document->EntityIdentifierForLabel(roots.Value(index)) == entity) {
                result = roots.Value(index); ++matches;
            }
        }
        return matches == 1 ? result : TDF_Label();
    } catch (...) { return {}; }
}

Core3DAuthoredParameterPreparationResult PublicResult(
    core3d::authored_parameter::Refusal refusal) noexcept {
    using Refusal = core3d::authored_parameter::Refusal;
    switch (refusal) {
        case Refusal::MalformedMutation:
        case Refusal::WrongThread:
            return Core3DAuthoredParameterPreparationResultMalformedMutation;
        case Refusal::ForeignOpening:
            return Core3DAuthoredParameterPreparationResultForeignOpening;
        case Refusal::StaleOpening:
            return Core3DAuthoredParameterPreparationResultStaleOpening;
        case Refusal::Cancelled:
            return Core3DAuthoredParameterPreparationResultCancelled;
        case Refusal::RecoveryRequired:
            return Core3DAuthoredParameterPreparationResultRecoveryRequired;
        case Refusal::None:
        case Refusal::UnsupportedCapability:
            return Core3DAuthoredParameterPreparationResultUnsupportedCapability;
    }
}
} // namespace

@interface Core3DAuthoredParameterOpening () {
@public
    std::shared_ptr<NativeOwner> _nativeOwner;
    NSString *_documentIdentifier;
    NSString *_entityIdentifier;
    NSString *_definitionIdentifier;
    NSData *_sourceCanonicalBytes;
    NSData *_suffixCanonicalBytes;
    double _metersPerUnit;
}
- (instancetype)initWithNativeOwner:(std::shared_ptr<NativeOwner>)owner;
@end

@implementation Core3DAuthoredParameterOpening
- (instancetype)initWithNativeOwner:(std::shared_ptr<NativeOwner>)owner {
    self = [super init];
    if (!self || !owner) return nil;
    _nativeOwner = std::move(owner);
    const auto& capture = _nativeOwner->capture();
    _documentIdentifier = [AuthoredIdentifier(capture.owner().document) copy];
    _entityIdentifier = [AuthoredIdentifier(capture.owner().entity) copy];
    _definitionIdentifier = [AuthoredIdentifier(capture.owner().definition) copy];
    _sourceCanonicalBytes = [[NSData alloc] initWithBytes:capture.sourceBytes().data()
                                                   length:capture.sourceBytes().size()];
    _suffixCanonicalBytes = [[NSData alloc] initWithBytes:capture.suffixBytes().data()
                                                   length:capture.suffixBytes().size()];
    _metersPerUnit = capture.metersPerUnit();
    return self;
}
- (NSString *)documentIdentifier { return _documentIdentifier; }
- (NSString *)entityIdentifier { return _entityIdentifier; }
- (NSString *)definitionIdentifier { return _definitionIdentifier; }
- (NSData *)sourceCanonicalBytes { return _sourceCanonicalBytes; }
- (NSData *)suffixCanonicalBytes { return _suffixCanonicalBytes; }
- (double)metersPerUnit { return _metersPerUnit; }
- (Core3DAuthoredParameterOpeningCurrentness)currentness {
    using Currentness = core3d::authored_parameter::Currentness;
    if (!_nativeOwner) return Core3DAuthoredParameterOpeningCurrentnessCancelled;
    switch (_nativeOwner->currentness()) {
        case Currentness::Current: return Core3DAuthoredParameterOpeningCurrentnessCurrent;
        case Currentness::Stale: return Core3DAuthoredParameterOpeningCurrentnessStale;
        case Currentness::Cancelled: return Core3DAuthoredParameterOpeningCurrentnessCancelled;
        case Currentness::Recovery: return Core3DAuthoredParameterOpeningCurrentnessRecovery;
    }
}
@end

@implementation Core3DViewController (AuthoredParameterOpening)
- (Core3DAuthoredParameterOpening *)openAuthoredParameterForEntityIdentifier:
    (NSString *)entityIdentifier expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0 || entityIdentifier.length > 128) return nil;
    try {
        GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)self.glController : nil;
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl ? gl.viewer : nullptr;
        const CGSize size = self.viewportDrawableSize;
        Core3DSceneSnapshot *current = [self captureSceneSnapshot];
        if (!viewer || !SameExpectedScene(expected, current, entityIdentifier)
            || !std::isfinite(size.width) || !std::isfinite(size.height)
            || size.width < 1 || size.height < 1
            || size.width > UINT32_MAX || size.height > UINT32_MAX) return nil;
        const Handle(OcctDocument) document = viewer->getDocument();
        const std::string entity = entityIdentifier.UTF8String ?: "";
        const TDF_Label ownerLabel = ExactRoot(document, entity);
        if (ownerLabel.IsNull()) return nil;
        const std::uint32_t width = std::uint32_t(size.width);
        const std::uint32_t height = std::uint32_t(size.height);
        auto context = viewer->captureNativeOpeningContext(width, height, {entity});
        if (!context) return nil;
        auto capture = document->CaptureAuthoredParameterAuthority(ownerLabel, *context);
        auto owner = NativeOwner::Open(document, std::move(context), std::move(capture),
                                       width, height);
        return owner ? [[Core3DAuthoredParameterOpening alloc]
            initWithNativeOwner:std::move(owner)] : nil;
    } catch (...) { return nil; }
}

- (Core3DAuthoredParameterPreparationResult)prepareAuthoredParameterOpening:
    (Core3DAuthoredParameterOpening *)opening
    mutationKind:(Core3DAuthoredParameterMutationKind)mutationKind {
    if (![NSThread isMainThread]
        || ![opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        || !opening->_nativeOwner)
        return Core3DAuthoredParameterPreparationResultMalformedMutation;
    GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)self.glController : nil;
    const auto viewer = gl ? gl.viewer : nullptr;
    const Handle(OcctDocument) document = viewer ? viewer->getDocument() : Handle(OcctDocument)();
    if (!opening->_nativeOwner->belongsTo(document))
        return Core3DAuthoredParameterPreparationResultForeignOpening;
    core3d::authored_parameter::Mutation mutation;
    mutation.capability = static_cast<core3d::authored_parameter::Capability>(mutationKind);
    return PublicResult(opening->_nativeOwner->prepare(mutation));
}

- (BOOL)cancelAuthoredParameterOpening:(Core3DAuthoredParameterOpening *)opening {
    if (![NSThread isMainThread]
        || ![opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        || !opening->_nativeOwner) return NO;
    GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)self.glController : nil;
    const auto viewer = gl ? gl.viewer : nullptr;
    const Handle(OcctDocument) document = viewer ? viewer->getDocument() : Handle(OcctDocument)();
    return opening->_nativeOwner->belongsTo(document) && opening->_nativeOwner->cancel();
}
@end
