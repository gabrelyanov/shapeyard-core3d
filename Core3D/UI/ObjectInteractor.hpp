//
//  ObjectInteractor.hpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#ifndef ObjectInteractor_hpp
#define ObjectInteractor_hpp

#include "Interactor.hpp"
#include "Core3DManipulator.hpp"
#include <AIS_Shape.hxx>
#include "BooleanOperationController.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace core3d {

    enum struct PrimitiveManipulatorType {
        PrimitiveGizmoTypeNone = 0,
        PrimitiveGizmoTypeMoveRotate,
        PrimitiveGizmoTypeScale,
        PrimitiveGizmoTypeChamfer,
        PrimitiveGizmoTypeSubtract,
        PrimitiveGizmoTypeUnion,
        PrimitiveGizmoTypeMirror,
        PrimitiveGizmoTypeMaterial,
        PrimitiveGizmoTypeExtrude,
    };

    enum class PresentationOverlayCaptureStatus : std::uint8_t {
        Available = 0,
        Unsafe,
    };

    class ObjectInteractor : public Interactor {
        
		PrimitiveManipulatorType _manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
        BooleanOperationController _booleanOpController;
        Handle(Core3DManipulator) _manipulator;
//        std::vector<TopoDS_Shape> _beforeTransformObjects;
    public:
        static constexpr Standard_ShortReal kManipulatorGap = 100;
        static constexpr std::size_t kMaxMirrorPreviewBodies = 8;
        
        ObjectInteractor() = delete;
        ObjectInteractor(Handle(Core3DContext), Handle(Core3DView), Handle(OcctDocument) doc, Standard_ShortReal manipulatorSide = 300);
        
        void selectLastObject();
		void selectAll();
        void deleteSelected();
        void duplicateSelected();
		void attachManipulatorToSelection(bool detach = false);
        
        bool transformManipulator(int theX, int theY);
        bool startTransformManipulator(int theX, int theY);
        void finishInteraction();
        void cancelInteraction();
        void setManipulatorType(PrimitiveManipulatorType type);
		Handle(TopLoc_Datum3D) manipulatorTransform();
		gp_XYZ manipulatorPosition();
        
        void SelectAndAttachManipulator(Handle(AIS_InteractiveObject) toObject);
        const bool isManipulatorAttached() const;
        const bool isManipulatorInteractionActive() const;
        const PrimitiveManipulatorType getManipulatorType() const;

        //! Capture only supported idle presentation. A valid mirror trial set
        //! is returned as explicit owned AIS_Shape handles so the snapshot
        //! builder never enumerates the interactive context. Available with
        //! empty content is an explicit clear; Unsafe means a renderer must
        //! retain OCCT.
        PresentationOverlayCaptureStatus captureIdlePresentationOverlay(
            scene::PresentationOverlayContent& theContent,
            std::vector<Handle(AIS_Shape)>& theMirrorPreviewObjects,
            BooleanPreviewCapture& theBooleanPreview) const noexcept;

        const bool isSelected() const;
		
		void fillSelectedState(Standard_Boolean forceActor, BooleanAction action);
		void updateDetectedState(Standard_Boolean forceActor, BooleanAction action);
		Standard_Boolean beginBoolean(BooleanAction action) noexcept;
		BooleanApplyResult applyBoolean(BooleanAction action) noexcept;
		void cancelBoolean(BooleanAction action) noexcept;
		void cancelActiveBoolean() noexcept;
		const bool canApplyBoolean() const;
		const bool hasActiveBoolean() const;
		const bool hasUnresolvedBoolean() const;
		const bool isBooleanSelectionFrozen() const;
#ifdef DEBUG
		Standard_Boolean debugBeginBooleanSelection(
			const std::vector<Handle(AIS_InteractiveObject)>& actors,
			const std::vector<Handle(AIS_InteractiveObject)>& subjects,
			BooleanAction action) noexcept;
		Standard_Boolean debugRecomputeBooleanPreview(
			BooleanAction action) noexcept;
#endif
		void applyMirror();
		void tryMirror(Standard_Integer axisIndex, bool backward) noexcept;
		void clearTrialMirrorObjects() noexcept;
		const bool hasTrialMirrorObjects() const;
		const bool hasUnresolvedMirrorObjects() const;
		
		void setManipulator(Handle(Core3DManipulator) manipulator) { _manipulator = manipulator; }
		void setObjectTransparent(Handle(AIS_InteractiveObject) selected, const bool on);
		void detachManipulator(bool updateViewer);

    private:
        void createManipulatorIfNeeded();
        void attachManipulator(Handle(AIS_InteractiveObject) toObject);
		void detachManipulator(Handle(AIS_InteractiveObject) fromObject);
		void tryMirrorImpl(Standard_Integer axisIndex, bool backward);
		
		void setSelectionTransparent(Handle(AIS_InteractiveObject) selected, const bool on);
		
    private:
		Standard_ShortReal _manipulatorSide;
		std::vector<Handle(AIS_Shape)> _trialMirrorObjects;
		std::unordered_map<const AIS_Shape*, TDF_Label>
			_trialMirrorSourceLabels;
		bool _trialMirrorObjectsValid = false;
    };
}
#endif /* Core3dObjectInteractor_hpp */
