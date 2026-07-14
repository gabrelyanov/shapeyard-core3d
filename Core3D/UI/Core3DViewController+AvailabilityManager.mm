//
//  Core3DViewController+AvailabilityManager.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 15.04.2024.
//

#import <Foundation/Foundation.h>
#import "GLViewController.h"
#import <Core3D/Core3DViewController+AvailabilityManager.h>

#include "GLViewController+Trick.h"

#include <TDF_LabelMap.hxx>

namespace {

constexpr Standard_Size kMaximumSelectedCapabilityDefinitions = 50'000;

constexpr Core3DModelCapability kBRepCapabilities =
    static_cast<Core3DModelCapability>(
        Core3DModelCapabilityObjectSelection
        | Core3DModelCapabilitySubshapeSelection
        | Core3DModelCapabilityTranslate
        | Core3DModelCapabilityRotate
        | Core3DModelCapabilityUniformScale
        | Core3DModelCapabilityNonuniformScale
        | Core3DModelCapabilityDelete
        | Core3DModelCapabilityDuplicate
        | Core3DModelCapabilityMirror
        | Core3DModelCapabilityBoolean
        | Core3DModelCapabilityChamfer
        | Core3DModelCapabilityExtrusion
        | Core3DModelCapabilityLinearArray
        | Core3DModelCapabilityMaterial
        | Core3DModelCapabilityExportOBJ
        | Core3DModelCapabilityExportSTL
        | Core3DModelCapabilityExportGLB
        | Core3DModelCapabilityExportSTEP);

constexpr Core3DModelCapability kTriangleMeshCapabilities =
    static_cast<Core3DModelCapability>(
        Core3DModelCapabilityObjectSelection
        | Core3DModelCapabilityTranslate
        | Core3DModelCapabilityRotate
        | Core3DModelCapabilityDelete
        | Core3DModelCapabilityDuplicate
        | Core3DModelCapabilityLinearArray
        | Core3DModelCapabilityMaterial
        | Core3DModelCapabilityExportOBJ
        | Core3DModelCapabilityExportSTL
        | Core3DModelCapabilityExportGLB);

constexpr Core3DModelCapability kAllExportCapabilities =
    static_cast<Core3DModelCapability>(
        Core3DModelCapabilityExportOBJ
        | Core3DModelCapabilityExportSTL
        | Core3DModelCapabilityExportGLB
        | Core3DModelCapabilityExportSTEP);

Core3DModelCapability CapabilitiesForRepresentation(
    const OcctGeometryRepresentation representation) noexcept {
    switch (representation) {
        case OcctGeometryRepresentation::LegacyUnknown:
        case OcctGeometryRepresentation::BRep:
            return kBRepCapabilities;
        case OcctGeometryRepresentation::TriangleMesh:
            return kTriangleMeshCapabilities;
        case OcctGeometryRepresentation::Invalid:
            return Core3DModelCapabilityNone;
    }
    return Core3DModelCapabilityNone;
}

Core3DModelCapability ExportCapabilityForType(
    const ExportType exportType) noexcept {
    switch (exportType) {
        case ExportTypeObj:
            return Core3DModelCapabilityExportOBJ;
        case ExportTypeStl:
            return Core3DModelCapabilityExportSTL;
        case ExportTypeGltf:
            return Core3DModelCapabilityExportGLB;
        case ExportTypeStep:
            return Core3DModelCapabilityExportSTEP;
    }
    return Core3DModelCapabilityNone;
}

bool CapabilitiesAllowGizmo(
    const Core3DModelCapability capabilities,
    const PrimitiveGizmoType gizmoType) noexcept {
    const auto has = [capabilities](
        const Core3DModelCapability required) {
        return (capabilities & required) == required;
    };
    switch (gizmoType) {
        case PrimitiveGizmoTypeMoveRotate:
            return has(static_cast<Core3DModelCapability>(
                Core3DModelCapabilityTranslate
                | Core3DModelCapabilityRotate));
        case PrimitiveGizmoTypeScale:
            return has(static_cast<Core3DModelCapability>(
                Core3DModelCapabilityUniformScale
                | Core3DModelCapabilityNonuniformScale));
        case PrimitiveGizmoTypeChamfer:
            return has(Core3DModelCapabilityChamfer);
        case PrimitiveGizmoTypeSubtract:
        case PrimitiveGizmoTypeUnion:
        case PrimitiveGizmoTypeIntersect:
            return has(Core3DModelCapabilityBoolean);
        case PrimitiveGizmoTypeMirror:
            return has(Core3DModelCapabilityMirror);
        case PrimitiveGizmoTypeMaterial:
            return has(Core3DModelCapabilityMaterial);
        case PrimitiveGizmoTypeExtrude:
            return has(Core3DModelCapabilityExtrusion);
        case PrimitiveGizmoTypeLinearArray:
            return has(Core3DModelCapabilityLinearArray);
        case PrimitiveGizmoTypeNone:
            return false;
    }
    return false;
}

bool Core3DIsBooleanGizmo(const PrimitiveGizmoType type) noexcept {
    return type == PrimitiveGizmoTypeSubtract
        || type == PrimitiveGizmoTypeUnion
        || type == PrimitiveGizmoTypeIntersect;
}

Core3DModelCapability DocumentExportCapabilities(
    const std::shared_ptr<core3d::Core3DViewer>& viewer) {
    if (viewer == nullptr) {
        return Core3DModelCapabilityNone;
    }
    const Handle(OcctDocument) document = viewer->getDocument();
    if (document.IsNull()) {
        return Core3DModelCapabilityNone;
    }
    const Standard_Integer formats =
        document->SupportedGeometryExportFormats();
    Core3DModelCapability capabilities = Core3DModelCapabilityNone;
    const auto addIfSupported = [&](
        const OcctGeometryExportFormat format,
        const Core3DModelCapability capability) {
        if ((formats & static_cast<Standard_Integer>(format)) != 0) {
            capabilities = static_cast<Core3DModelCapability>(
                capabilities | capability);
        }
    };
    addIfSupported(
        OcctGeometryExportFormat::Obj,
        Core3DModelCapabilityExportOBJ);
    addIfSupported(
        OcctGeometryExportFormat::Stl,
        Core3DModelCapabilityExportSTL);
    addIfSupported(
        OcctGeometryExportFormat::Gltf,
        Core3DModelCapabilityExportGLB);
    addIfSupported(
        OcctGeometryExportFormat::Step,
        Core3DModelCapabilityExportSTEP);
    return static_cast<Core3DModelCapability>(
        capabilities & kAllExportCapabilities);
}

} // namespace

@implementation Core3DViewController (AvailabilityManager)

- (BOOL)canAdd {
    return self.can_add;
}

- (BOOL)canDelete {
    return self.can_delete
        && (self.selectedModelCapabilities
            & Core3DModelCapabilityDelete) != 0;
}

- (BOOL)canDuplicate {
    return self.can_duplicate
        && (self.selectedModelCapabilities
            & Core3DModelCapabilityDuplicate) != 0;
}

- (BOOL)canUndo {
    const std::shared_ptr<core3d::Core3DViewer> viewer = GLController.viewer;
    return viewer != nullptr
        && !viewer->getDocument().IsNull()
        && viewer->getDocument()->canUndo();
}
- (BOOL)canRedo {
    const std::shared_ptr<core3d::Core3DViewer> viewer = GLController.viewer;
    return viewer != nullptr
        && !viewer->getDocument().IsNull()
        && viewer->getDocument()->canRedo();
}

- (BOOL)canApply {
    return self.can_apply;
}

- (BOOL) canApplyMaterial {
    return self.can_apply_material
        && (self.selectedModelCapabilities
            & Core3DModelCapabilityMaterial) != 0;
}

- (Core3DModelCapability)selectedModelCapabilities {
    if (![NSThread isMainThread] || GLController == nil) {
        return Core3DModelCapabilityNone;
    }
    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        if (viewer == nullptr) {
            return Core3DModelCapabilityNone;
        }
        const Handle(OcctDocument) document = viewer->getDocument();
        const Handle(AIS_InteractiveContext)& context =
            viewer->AisContext();
        if (document.IsNull() || context.IsNull()) {
            return Core3DModelCapabilityNone;
        }

        Core3DModelCapability capabilities = kBRepCapabilities;
        TDF_LabelMap selectedDefinitionLabels;
        Standard_Size selectedDefinitionCount = 0;
        bool hasSelection = false;
        for (context->InitSelected(); context->MoreSelected();
             context->NextSelected()) {
            const Handle(AIS_InteractiveObject) selected =
                context->SelectedInteractive();
            if (selected.IsNull()
                || !document->IsPresentationEditable(selected)) {
                return Core3DModelCapabilityNone;
            }
            const TDF_Label label = document->ShapeLabel(selected);
            if (label.IsNull()
                || !document->IsEditableFreeSimpleDefinitionLabel(
                    label)) {
                return Core3DModelCapabilityNone;
            }
            if (selectedDefinitionLabels.Contains(label)) {
                continue;
            }
            if (selectedDefinitionCount
                    >= kMaximumSelectedCapabilityDefinitions
                || !selectedDefinitionLabels.Add(label)) {
                return Core3DModelCapabilityNone;
            }
            ++selectedDefinitionCount;
            hasSelection = true;
            const Core3DModelCapability definitionCapabilities =
                CapabilitiesForRepresentation(
                    document->GeometryRepresentationForLabel(label));
            if (definitionCapabilities == Core3DModelCapabilityNone) {
                return Core3DModelCapabilityNone;
            }
            capabilities = static_cast<Core3DModelCapability>(
                capabilities & definitionCapabilities);
        }
        if (!hasSelection) {
            return Core3DModelCapabilityNone;
        }
        // Linear Array deliberately has single-source semantics in its first
        // touch contract. Other common capabilities remain intersection-based
        // for multi-selection.
        if (selectedDefinitionCount != 1) {
            capabilities = static_cast<Core3DModelCapability>(
                static_cast<NSUInteger>(capabilities)
                & ~static_cast<NSUInteger>(
                    Core3DModelCapabilityLinearArray));
        }
        return capabilities;
    } catch (...) {
        return Core3DModelCapabilityNone;
    }
}

- (NSArray<NSNumber *> *)availableGizmoTypes {
    // Boolean, Bevel, and Extrusion previews can suppress or consume their live
    // AIS selection while retaining an admitted BRep-only operation. Keep the
    // tool rail (and, critically, Apply/Cancel) available until it resolves.
    const BOOL hasRetainedBoolean =
        Core3DIsBooleanGizmo(_currentGizmoType)
        && GLController != nil
        && [GLController hasActiveBoolean];
    const BOOL hasRetainedChamfer =
        _currentGizmoType == PrimitiveGizmoTypeChamfer
        && GLController != nil
        && [GLController hasActiveBevel];
    const std::shared_ptr<core3d::Core3DViewer> viewer =
        GLController == nil ? nullptr : GLController.viewer;
    const BOOL hasRetainedExtrusion =
        _currentGizmoType == PrimitiveGizmoTypeExtrude
        && viewer != nullptr
        && viewer->getShapeInteractor() != nullptr
        && viewer->getShapeInteractor()->hasActiveExtrusion();
    const BOOL hasRetainedLinearArray =
        _currentGizmoType == PrimitiveGizmoTypeLinearArray
        && viewer != nullptr
        && viewer->getObjectInteractor() != nullptr
        && viewer->getObjectInteractor()->hasActiveLinearArray();
    const BOOL hasRetainedBRepOperation =
        hasRetainedBoolean || hasRetainedChamfer || hasRetainedExtrusion;
    Core3DModelCapability capabilities = self.selectedModelCapabilities;
    if (hasRetainedBRepOperation) {
        capabilities = kBRepCapabilities;
    } else if (hasRetainedLinearArray
               && capabilities == Core3DModelCapabilityNone) {
        // Array normally keeps its one admitted source selected. If selection
        // publication is momentarily unavailable, expose only Array itself so
        // Apply/Cancel stays reachable without leaking BRep-only tools to a
        // retained TriangleMesh source.
        capabilities = Core3DModelCapabilityLinearArray;
    }
    if (capabilities == Core3DModelCapabilityNone
        || _availableGizmoTypes.count == 0) {
        return @[];
    }
    NSMutableArray<NSNumber *> *available =
        [NSMutableArray arrayWithCapacity:_availableGizmoTypes.count];
    for (NSNumber *value in _availableGizmoTypes) {
        const PrimitiveGizmoType gizmoType =
            static_cast<PrimitiveGizmoType>(value.unsignedIntegerValue);
        if (CapabilitiesAllowGizmo(capabilities, gizmoType)) {
            [available addObject:value];
        }
    }
    return [available copy];
}

- (BOOL)canExportType:(ExportType)exportType {
    if (![NSThread isMainThread] || GLController == nil) {
        return NO;
    }
    try {
        const Core3DModelCapability required =
            ExportCapabilityForType(exportType);
        if (required == Core3DModelCapabilityNone) {
            return NO;
        }
        const Core3DModelCapability capabilities =
            DocumentExportCapabilities(GLController.viewer);
        return (capabilities & required) == required;
    } catch (...) {
        return NO;
    }
}

- (NSArray<NSNumber *> *)availableExportTypes {
    if (![NSThread isMainThread] || GLController == nil) {
        return @[];
    }
    try {
        const Core3DModelCapability capabilities =
            DocumentExportCapabilities(GLController.viewer);
        if (capabilities == Core3DModelCapabilityNone) {
            return @[];
        }
        NSMutableArray<NSNumber *> *available =
            [NSMutableArray arrayWithCapacity:4];
        const ExportType orderedTypes[] = {
            ExportTypeObj,
            ExportTypeStl,
            ExportTypeGltf,
            ExportTypeStep,
        };
        for (const ExportType exportType : orderedTypes) {
            const Core3DModelCapability required =
                ExportCapabilityForType(exportType);
            if ((capabilities & required) == required) {
                [available addObject:@(exportType)];
            }
        }
        return [available copy];
    } catch (...) {
        return @[];
    }
}

@end
