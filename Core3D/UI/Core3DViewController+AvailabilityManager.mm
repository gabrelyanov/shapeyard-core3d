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
#include <AIS_Shape.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <Standard_ErrorHandler.hxx>

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
        | Core3DModelCapabilityShell
        | Core3DModelCapabilityRadialArray
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
        | Core3DModelCapabilityUniformScale
        | Core3DModelCapabilityDelete
        | Core3DModelCapabilityDuplicate
        | Core3DModelCapabilityLinearArray
        | Core3DModelCapabilityRadialArray
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
        case ExportTypeUsdz:
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
        case PrimitiveGizmoTypeShell:
            return has(Core3DModelCapabilityShell);
        case PrimitiveGizmoTypeRadialArray:
            return has(Core3DModelCapabilityRadialArray);
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

@interface Core3DViewController (VisibleRawR2BevelAvailability)
- (BOOL)core3d_hasVisibleRawR2Bevel;
- (BOOL)core3d_canBeginVisibleRawR2Bevel;
@end

@implementation Core3DViewController (AvailabilityManager)

- (BOOL)canAdd {
    if (![NSThread isMainThread] || GLController == nil) {
        return NO;
    }
    const std::shared_ptr<core3d::Core3DViewer> viewer =
        GLController.viewer;
    return self.can_add && viewer != nullptr
        && !viewer->hasUnresolvedEdit();
}

- (BOOL)canDelete {
    return self.can_delete
        && (self.selectedModelCapabilities
            & Core3DModelCapabilityDelete) != 0;
}

- (BOOL)canDuplicate {
    if (![NSThread isMainThread] || GLController == nil) {
        return NO;
    }
    const std::shared_ptr<core3d::Core3DViewer> viewer =
        GLController.viewer;
    if (viewer != nullptr && viewer->hasUnresolvedOrdinaryEdit()) { return NO; }
    if (viewer != nullptr && viewer->hasUnresolvedDuplicate()) {
        // Presentation repair can legitimately have zero selected owners. The
        // typed ledger, not selectedModelCapabilities, is then the authority:
        // Duplicate is the sole action that can reconcile the committed result.
        return YES;
    }
    return self.can_duplicate
        && (self.selectedModelCapabilities
            & Core3DModelCapabilityDuplicate) != 0;
}

- (BOOL)canUndo {
    if (![NSThread isMainThread] || GLController == nil) {
        return NO;
    }
    const std::shared_ptr<core3d::Core3DViewer> viewer = GLController.viewer;
    return viewer != nullptr
        && !viewer->hasUnresolvedEdit()
        && !viewer->getDocument().IsNull()
        && viewer->getDocument()->canUndo();
}
- (BOOL)canRedo {
    if (![NSThread isMainThread] || GLController == nil) {
        return NO;
    }
    const std::shared_ptr<core3d::Core3DViewer> viewer = GLController.viewer;
    return viewer != nullptr
        && !viewer->hasUnresolvedEdit()
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
        if (viewer == nullptr || viewer->hasUnresolvedEdit()) {
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
        bool hasRetainedOccurrence = false;
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
            Core3DModelCapability definitionCapabilities =
                CapabilitiesForRepresentation(
                    document->GeometryRepresentationForLabel(label));
            core3d::sweep_persistence::Record sweep;
            if (!core3d::sweep_persistence::Read(document->Document(),label,sweep)) return Core3DModelCapabilityNone;
            core3d::loft_persistence::Record loft;
            if (!core3d::loft_persistence::Read(document->Document(),label,loft))return Core3DModelCapabilityNone;
            core3d::retained_solid::Record retained;
            if(!core3d::retained_solid::Read(document->Document(),label,retained))return Core3DModelCapabilityNone;
            if(retained.value) {
                hasRetainedOccurrence=true;
                // A part with a saved (resizable) hole. The engine already runs Push-pull,
                // Split and plain Cut / Join / Overlap on it by baking the hole in the same
                // command (ShapeInteractor, ApplyBakedBooleanBatch); only this list refused
                // them, so a holed part lost Subtract, Union and Push-pull (phone stand rows
                // 23 and 30, 15 Nov). Offer them whenever that bake is possible. The constant
                // itself stays as it is: By numbers pins it.
                std::uint64_t unsupported=core3d::cylindrical_cut::UnsupportedOccurrenceCapabilities;
                // Push-pull, Bevel/Round and Shell bake a standalone saved Hole
                // inside their own Apply command (PrepareBooleanResultForFaceEdit;
                // Undo restores the resizable hole) and their admission accepts
                // this owner (push-pull lane 15 Nov, 5785354, e7433bf). Keep those
                // tools; other derived routes stay withheld.
                if(document->CanBakeSavedCutForFaceEdit(label)) {
                    unsupported&=~(static_cast<std::uint64_t>(Core3DModelCapabilityBoolean)
                        |static_cast<std::uint64_t>(Core3DModelCapabilityExtrusion)
                        |static_cast<std::uint64_t>(Core3DModelCapabilityChamfer)
                        |static_cast<std::uint64_t>(Core3DModelCapabilityShell)
                        // Duplicate and Mirror copy it as a plain solid; the source keeps the record.
                        |static_cast<std::uint64_t>(Core3DModelCapabilityDuplicate)
                        |static_cast<std::uint64_t>(Core3DModelCapabilityMirror)
                        // Repeat in a row / in a circle: every copy is plain too.
                        |static_cast<std::uint64_t>(Core3DModelCapabilityLinearArray)
                        |static_cast<std::uint64_t>(Core3DModelCapabilityRadialArray));
                }
                definitionCapabilities=static_cast<Core3DModelCapability>(
                    static_cast<std::uint64_t>(definitionCapabilities)&~unsupported);
            }
            if (!sweep.label.IsNull() || !loft.label.IsNull()) {
                // Translation/local-transform copies stage an exact recipe on
                // every destination; Mirror still lacks a reflection-frame policy.
                // Union/Subtract stay: the plain route bakes the recipe (BakedBooleanAdmits).
                definitionCapabilities=static_cast<Core3DModelCapability>(definitionCapabilities
                    & ~(Core3DModelCapabilitySubshapeSelection|Core3DModelCapabilityNonuniformScale
                        |Core3DModelCapabilityMirror
                        |Core3DModelCapabilityChamfer|Core3DModelCapabilityExtrusion|Core3DModelCapabilityShell
                        ));
            }
            if (definitionCapabilities == Core3DModelCapabilityNone) {
                return Core3DModelCapabilityNone;
            }
            capabilities = static_cast<Core3DModelCapability>(
                capabilities & definitionCapabilities);
        }
        if (!hasSelection) {
            return Core3DModelCapabilityNone;
        }
        // Linear/Radial Array and Shell deliberately have single-source
        // semantics in their first touch contracts. Other common capabilities
        // remain intersection-based for multi-selection.
        if(hasRetainedOccurrence&&selectedDefinitionCount!=1)capabilities=static_cast<Core3DModelCapability>(
            capabilities&~(Core3DModelCapabilityTranslate|Core3DModelCapabilityRotate|Core3DModelCapabilityUniformScale));
        if (selectedDefinitionCount != 1) {
            capabilities = static_cast<Core3DModelCapability>(
                static_cast<NSUInteger>(capabilities)
                & ~static_cast<NSUInteger>(
                    Core3DModelCapabilityLinearArray
                    | Core3DModelCapabilityShell
                    | Core3DModelCapabilityRadialArray));
        }
        return capabilities;
    } catch (...) {
        return Core3DModelCapabilityNone;
    }
}

- (NSArray<NSNumber *> *)availableGizmoTypes {
    if (![NSThread isMainThread] || GLController == nil) {
        return @[];
    }
    try {
        // Topology-changing previews can suppress or consume their live AIS
        // selection while retaining an admitted BRep-only operation. Keep the
        // tool rail (and Apply/Cancel) available until it resolves.
        const BOOL hasRetainedBoolean =
            Core3DIsBooleanGizmo(_currentGizmoType)
            && [GLController hasActiveBoolean];
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        const std::shared_ptr<core3d::ShapeInteractor> shapeInteractor =
            viewer == nullptr ? nullptr : viewer->getShapeInteractor();
		// Round owns its Selecting ledger immediately: unlike generic Bevel,
		// it is not eligible for a second admission merely because no radius
		// preview has been requested yet. Keep its one Chamfer control surface.
		const BOOL hasRetainedChamfer =
			_currentGizmoType == PrimitiveGizmoTypeChamfer
			&& ([self core3d_hasVisibleRawR2Bevel]
				|| ([GLController hasActiveBevel]
					&& (([GLController roundMetersPerUnit] > 0.0
						&& [GLController roundMaximumRadiusMM] > 0.0)
						|| (shapeInteractor != nullptr
							&& (shapeInteractor->isBevelSelectionFrozen()
								|| shapeInteractor->bevelPreviewState()
									!= core3d::BevelPreviewState::Selecting)))));
		const BOOL hasSelectingChamfer =
			_currentGizmoType == PrimitiveGizmoTypeChamfer
			&& [GLController hasActiveBevel]
			&& !hasRetainedChamfer;
        const BOOL hasRetainedExtrusion =
            _currentGizmoType == PrimitiveGizmoTypeExtrude
            && shapeInteractor != nullptr
            && shapeInteractor->hasActiveExtrusion();
        const BOOL hasRetainedShell =
            _currentGizmoType == PrimitiveGizmoTypeShell
            && shapeInteractor != nullptr
            && shapeInteractor->hasActiveShell();
        const BOOL hasRetainedLinearArray =
            _currentGizmoType == PrimitiveGizmoTypeLinearArray
            && viewer != nullptr
            && viewer->getObjectInteractor() != nullptr
            && viewer->getObjectInteractor()->hasActiveLinearArray();
        const BOOL hasRetainedRadialArray =
            _currentGizmoType == PrimitiveGizmoTypeRadialArray
            && viewer != nullptr
            && viewer->getObjectInteractor() != nullptr
            && viewer->getObjectInteractor()->hasActiveRadialArray();
        const BOOL hasRetainedBRepOperation =
            hasRetainedBoolean || hasRetainedChamfer
            || hasRetainedExtrusion || hasRetainedShell;
        if (hasRetainedExtrusion) {
            // The preview ledger owns only Extrude; do not fabricate Shell.
            return @[@(PrimitiveGizmoTypeExtrude)];
        }
        if (hasRetainedShell) {
            // Keep Apply/Cancel reachable without advertising Extrude.
            return @[@(PrimitiveGizmoTypeShell)];
        }
		if (hasRetainedChamfer) {
			// The retained Bevel ledger owns exactly Chamfer. Its suppressed
			// live selection must not fabricate unrelated BRep tools.
			return @[@(PrimitiveGizmoTypeChamfer)];
		}
        Core3DModelCapability capabilities =
            self.selectedModelCapabilities;
        if (hasRetainedBRepOperation) {
            capabilities = kBRepCapabilities;
        } else if ((hasRetainedLinearArray || hasRetainedRadialArray)
                   && capabilities == Core3DModelCapabilityNone) {
            // A retained array owns only its exact array tool.
            capabilities = hasRetainedRadialArray
                ? Core3DModelCapabilityRadialArray
                : Core3DModelCapabilityLinearArray;
        }
        if (capabilities == Core3DModelCapabilityNone
            || _availableGizmoTypes.count == 0) {
            return @[];
        }
        const BOOL exposesExtrusion =
            [_availableGizmoTypes containsObject:
                @(PrimitiveGizmoTypeExtrude)];
        const BOOL exposesShell =
            [_availableGizmoTypes containsObject:
                @(PrimitiveGizmoTypeShell)];
		const BOOL exposesChamfer =
			[_availableGizmoTypes containsObject:
				@(PrimitiveGizmoTypeChamfer)];
        Standard_Boolean extrusionAdmission = Standard_False;
        Standard_Boolean shellAdmission = Standard_False;
        if (shapeInteractor != nullptr
            && (exposesExtrusion || exposesShell)) {
            (void)shapeInteractor->queryFaceOperationAdmission(
                extrusionAdmission, shellAdmission);
        }
        const BOOL canBeginExtrusion =
            exposesExtrusion && extrusionAdmission;
        const BOOL canBeginShell = exposesShell && shellAdmission;
		const BOOL canBeginRawR2Chamfer = exposesChamfer
			&& [self core3d_canBeginVisibleRawR2Bevel];
		// R9-C3 (CTO 22 Nov 18:47): Bevel is always offered on a solid part;
		// past the edge limit it is offered dimmed with its reason.
		const BOOL canBeginChamfer = exposesChamfer
			&& (canBeginRawR2Chamfer
				|| (shapeInteractor != nullptr
					&& (shapeInteractor->canBeginBevelSelection()
						|| shapeInteractor->bevelSelectionOverEdgeLimit())));
		if (hasSelectingChamfer) {
			return canBeginChamfer
				? @[@(PrimitiveGizmoTypeChamfer)]
				: @[];
		}
        NSMutableArray<NSNumber *> *available =
            [NSMutableArray arrayWithCapacity:_availableGizmoTypes.count];
        for (NSNumber *value in _availableGizmoTypes) {
            const PrimitiveGizmoType gizmoType =
                static_cast<PrimitiveGizmoType>(
                    value.unsignedIntegerValue);
			const BOOL faceOperationIsAdmitted =
				(gizmoType != PrimitiveGizmoTypeExtrude
                    || canBeginExtrusion)
                && (gizmoType != PrimitiveGizmoTypeShell
                    || canBeginShell)
				&& (gizmoType != PrimitiveGizmoTypeChamfer
					|| canBeginChamfer);
			const BOOL admittedRawR2Chamfer =
				gizmoType == PrimitiveGizmoTypeChamfer
				&& canBeginRawR2Chamfer;
			if (faceOperationIsAdmitted
				&& (admittedRawR2Chamfer
					|| CapabilitiesAllowGizmo(capabilities, gizmoType))) {
				[available addObject:value];
            }
        }
        return [available copy];
    } catch (...) {
        return @[];
    }
}

- (BOOL)bevelOverEdgeLimit {
    if (![NSThread isMainThread] || GLController == nil) {
        return NO;
    }
    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        const std::shared_ptr<core3d::ShapeInteractor> shapeInteractor =
            viewer == nullptr ? nullptr : viewer->getShapeInteractor();
        return shapeInteractor != nullptr
            && CapabilitiesAllowGizmo(self.selectedModelCapabilities,
                                      PrimitiveGizmoTypeChamfer)
            && shapeInteractor->bevelSelectionOverEdgeLimit();
    } catch (...) {
        return NO;
    }
}

- (BOOL)bevelTooDetailed {
    if (![NSThread isMainThread] || GLController == nil) {
        return NO;
    }
    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        const std::shared_ptr<core3d::ShapeInteractor> shapeInteractor =
            viewer == nullptr ? nullptr : viewer->getShapeInteractor();
        return shapeInteractor != nullptr
            && CapabilitiesAllowGizmo(self.selectedModelCapabilities,
                                      PrimitiveGizmoTypeChamfer)
            && shapeInteractor->bevelSelectionTooDetailed();
    } catch (...) {
        return NO;
    }
}

- (BOOL)bevelAllRound {
    if (![NSThread isMainThread] || GLController == nil) {
        return NO;
    }
    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        const std::shared_ptr<core3d::ShapeInteractor> shapeInteractor =
            viewer == nullptr ? nullptr : viewer->getShapeInteractor();
        return shapeInteractor != nullptr
            && CapabilitiesAllowGizmo(self.selectedModelCapabilities,
                                      PrimitiveGizmoTypeChamfer)
            && shapeInteractor->bevelSelectionAllRound();
    } catch (...) {
        return NO;
    }
}

- (BOOL)singleSelectedPartIsMesh {
    // Store-build twin of what debugTopologySelectionState reports as
    // selectedCount 1, selectedKind Object, presentationRepresentation
    // TriangleMesh (R10-F4: the column keeps a mesh part's other cells dimmed
    // with a reason). Read-only; fails closed (NO) off-main or on any doubt.
    if (![NSThread isMainThread] || GLController == nil) {
        return NO;
    }
    try {
        OCC_CATCH_SIGNALS
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        if (viewer == nullptr) {
            return NO;
        }
        const std::shared_ptr<core3d::ShapeInteractor> shapeInteractor =
            viewer->getShapeInteractor();
        const Handle(OcctDocument) document = viewer->getDocument();
        const Handle(AIS_InteractiveContext)& context =
            viewer->AisContext();
        if (shapeInteractor == nullptr || document.IsNull()
            || document->Document().IsNull() || context.IsNull()
            || shapeInteractor->getSelectionMode()
                != core3d::ShapeSelectionMode::WholeShape
            || !shapeInteractor->selectionModeAuthorityIsExact()) {
            return NO;
        }
        Standard_Size selectedCount = 0;
        OcctGeometryRepresentation representation =
            OcctGeometryRepresentation::Invalid;
        for (context->InitSelected(); context->MoreSelected();
             context->NextSelected()) {
            const Handle(SelectMgr_EntityOwner) owner =
                context->SelectedOwner();
            const Handle(AIS_Shape) presentation =
                Handle(AIS_Shape)::DownCast(context->SelectedInteractive());
            if (owner.IsNull() || !owner->HasSelectable()
                || presentation.IsNull() || presentation->Shape().IsNull()
                || owner->Selectable() != presentation) {
                continue;
            }
            const TDF_Label label = document->ShapeLabel(presentation);
            if (label.IsNull()) {
                continue;
            }
            const OcctGeometryRepresentation labelRepresentation =
                document->GeometryRepresentationForLabel(label);
            if (labelRepresentation == OcctGeometryRepresentation::Invalid
                || document->EntityIdentifierForLabel(label).empty()) {
                continue;
            }
            if (++selectedCount == 1) {
                representation = labelRepresentation;
            }
        }
        return selectedCount == 1
            && representation == OcctGeometryRepresentation::TriangleMesh;
    } catch (...) {
        return NO;
    }
}

- (BOOL)canExportType:(ExportType)exportType {
    if (![NSThread isMainThread] || GLController == nil) {
        return NO;
    }
    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        if (viewer == nullptr || viewer->hasUnresolvedEdit()) {
            return NO;
        }
        const Core3DModelCapability required =
            ExportCapabilityForType(exportType);
        if (required == Core3DModelCapabilityNone) {
            return NO;
        }
        const Core3DModelCapability capabilities =
            DocumentExportCapabilities(viewer);
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
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        if (viewer == nullptr || viewer->hasUnresolvedEdit()) {
            return @[];
        }
        const Core3DModelCapability capabilities =
            DocumentExportCapabilities(viewer);
        if (capabilities == Core3DModelCapabilityNone) {
            return @[];
        }
        NSMutableArray<NSNumber *> *available =
            [NSMutableArray arrayWithCapacity:5];
        const ExportType orderedTypes[] = {
            ExportTypeObj,
            ExportTypeStl,
            ExportTypeGltf,
            ExportTypeStep,
            ExportTypeUsdz,
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
