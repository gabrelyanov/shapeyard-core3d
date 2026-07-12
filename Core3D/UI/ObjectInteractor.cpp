//
//  ObjectInteractor.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#include "ObjectInteractor.hpp"
#include "../Scene/SceneSnapshot.hpp"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <Precision.hxx>
#include <V3d_View.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDataStd_Real.hxx>
#include <GP_Quaternion.hxx>
#include <AIS_Shape.hxx>
#include <Standard_Failure.hxx>
#include <array>
#include <cmath>

namespace core3d {
	namespace {
		Standard_Boolean IsTopologicallyValid(const TopoDS_Shape& shape) {
			if (shape.IsNull()) {
				return Standard_False;
			}
			try {
				BRepCheck_Analyzer analyzer(shape, Standard_True);
				return analyzer.IsValid();
			} catch (...) {
				return Standard_False;
			}
		}

		bool TransformDiffers(const gp_Trsf& theLeft,
		                      const gp_Trsf& theRight) {
			for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
				for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
					if (std::abs(theLeft.Value(aRow, aColumn)
					             - theRight.Value(aRow, aColumn))
					    > Precision::Confusion()) {
						return true;
					}
				}
			}
			return false;
		}
	}

    ObjectInteractor::ObjectInteractor(Handle(Core3DContext) context
                                       , Handle(Core3DView) view
                                       , Handle(OcctDocument) doc
                                       , Standard_ShortReal manipulatorSide)
        : Interactor(context, view, doc)
        , _manipulatorSide(manipulatorSide)
        , _booleanOpController(context, doc) {
    }

    void ObjectInteractor::selectLastObject() {
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        auto object = objects.Last();
        myContext->SetSelected(object, Standard_True);
        attachManipulator(object);
    }

    void ObjectInteractor::createManipulatorIfNeeded() {
        if(_manipulator.IsNull()) {
            _manipulator = new Core3DManipulator();
            _manipulator->SetSize(_manipulatorSide);
            _manipulator->SetZoomPersistence(Standard_True);
            _manipulator->SetModeActivationOnDetection(Standard_True);
        }
    }

    void ObjectInteractor::attachManipulator(Handle(AIS_InteractiveObject) toObject) {
        createManipulatorIfNeeded();
        _manipulator->Attach(toObject);
        myContext->UpdateCurrentViewer();
    }

	void ObjectInteractor::detachManipulator(Handle(AIS_InteractiveObject) fromObject) {
		if (!_manipulator.IsNull()) {
			if (_manipulator->HasActiveTransformation()) {
				cancelInteraction();
			}
			_manipulator->Detach(fromObject);
			myContext->UpdateCurrentViewer();
		}
	}

	void ObjectInteractor::detachManipulator(bool updateViewer) {
		if (!_manipulator.IsNull()) {
			if (_manipulator->HasActiveTransformation()) {
				cancelInteraction();
			}
			_manipulator->Detach();
			if (updateViewer)
				myContext->UpdateCurrentViewer();
		}
	}

	void ObjectInteractor::	selectAll() {
		if (!_manipulator.IsNull() && _manipulator->HasActiveTransformation()) {
			cancelInteraction();
		}
		AIS_ListOfInteractive objects;
		myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
		AIS_ListIteratorOfListOfInteractive iobject(objects);
		if (objects.Size()) {
			myContext->ClearSelected(Standard_False);
			createManipulatorIfNeeded();
			while (iobject.More()) {
				auto object = iobject.Value();
				myContext->AddSelect(object);
				myContext->HilightSelected(Standard_False);
				_manipulator->Attach(object);
				
				iobject.Next();
			}
			myContext->UpdateCurrentViewer();
		}
	}

	Handle(TopLoc_Datum3D) ObjectInteractor::manipulatorTransform() {
		if (!_manipulator.IsNull())
			return _manipulator->Transform();
		else
			return nullptr;
	}

	gp_XYZ ObjectInteractor::manipulatorPosition() {
		if (!_manipulator.IsNull())
			return _manipulator->Position().Location().Coord();
		else
			return gp_XYZ();
	}

    PresentationOverlayCaptureStatus
    ObjectInteractor::captureIdlePresentationOverlay(
        scene::PresentationOverlayContent& theContent,
        std::vector<Handle(AIS_Shape)>& theMirrorPreviewObjects,
        BooleanPreviewCapture& theBooleanPreview) const noexcept {
        try {
            theContent = {};
            theMirrorPreviewObjects.clear();
            theBooleanPreview = {};
            const bool hasMirrorPreview = !_trialMirrorObjects.empty();
            // Cleanup residue is deliberately not publishable. Retaining OCCT
            // prevents an unresolved presentation from being omitted.
            if (hasMirrorPreview
                && (!_trialMirrorObjectsValid
                    || _trialMirrorObjects.size() > kMaxMirrorPreviewBodies)) {
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            if (_booleanOpController.hasUnresolvedState()) {
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            const bool isSubtract = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract;
            const bool isUnion = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeUnion;
            if (isSubtract || isUnion) {
                if (hasMirrorPreview) {
                    return PresentationOverlayCaptureStatus::Unsafe;
                }
                if (!_booleanOpController.hasSelectionState()) {
                    return PresentationOverlayCaptureStatus::Available;
                }
                if (!_booleanOpController.capturePreview(theBooleanPreview)
                    || (isSubtract
                        && theBooleanPreview.action
                            != BooleanAction::BooleanSubtract)
                    || (isUnion
                        && theBooleanPreview.action
                            != BooleanAction::BooleanUnion)) {
                    theBooleanPreview = {};
                    return PresentationOverlayCaptureStatus::Unsafe;
                }
                return PresentationOverlayCaptureStatus::Available;
            }
            if (_booleanOpController.hasActiveOperation()
                || _booleanOpController.hasSelectionState()) {
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            if (_manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeNone) {
                return hasMirrorPreview
                    ? PresentationOverlayCaptureStatus::Unsafe
                    : PresentationOverlayCaptureStatus::Available;
            }
            const bool isMoveRotate = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate;
            const bool isScale = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeScale;
            const bool isMirror = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeMirror;
            if (!isMoveRotate && !isScale && !isMirror) {
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            if (_manipulator.IsNull() || !_manipulator->IsAttached()) {
                return hasMirrorPreview
                    ? PresentationOverlayCaptureStatus::Unsafe
                    : PresentationOverlayCaptureStatus::Available;
            }
            if (myContext.IsNull()
                || !_manipulator->IsInstance(
                    STANDARD_TYPE(Core3DManipulator))
                || !myContext->IsDisplayed(_manipulator)
                || !_manipulator->ZoomPersistence()
                || _manipulator->HasActiveMode()
                || _manipulator->HasActiveTransformation()) {
                theContent = {};
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            const Standard_Boolean didCapture = isMoveRotate
                ? _manipulator->CaptureIdleMoveRotateOverlay(theContent)
                : isScale
                    ? _manipulator->CaptureIdleScaleOverlay(theContent)
                    : _manipulator->CaptureIdleMirrorOverlay(theContent);
            if (!didCapture) {
                theContent = {};
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            if (hasMirrorPreview) {
                if (!isMirror) {
                    theContent = {};
                    return PresentationOverlayCaptureStatus::Unsafe;
                }
                for (const Handle(AIS_Shape)& aShape : _trialMirrorObjects) {
                    if (aShape.IsNull() || aShape->Shape().IsNull()
                        || !myContext->IsDisplayed(aShape)) {
                        theContent = {};
                        return PresentationOverlayCaptureStatus::Unsafe;
                    }
                }
                theContent.kind =
                    scene::PresentationOverlayKind::MirrorPreview;
                theMirrorPreviewObjects = _trialMirrorObjects;
            }
            return PresentationOverlayCaptureStatus::Available;
        } catch (...) {
            theContent = {};
            theMirrorPreviewObjects.clear();
            theBooleanPreview = {};
            return PresentationOverlayCaptureStatus::Unsafe;
        }
    }

    void ObjectInteractor::deleteSelected() {
        if (_manipulator.IsNull() || !_manipulator->IsAttached()) { return; }
		auto doc = myDoc->ChangeDocument();
		if (doc.IsNull() || doc->HasOpenCommand()) { return; }

        Handle(Core3DManipulatorObjectSequence) objects = _manipulator->Objects();
		std::vector<std::pair<Handle(AIS_InteractiveObject), TDF_Label>> removals;
        for (Core3DManipulatorObjectSequence::Iterator it(*objects); it.More(); it.Next()) {
			const TDF_Label label = myDoc->ShapeLabel(it.Value());
			if (label.IsNull()) { return; }
			for (const auto& removal : removals) {
				if (removal.second.IsEqual(label)) { return; }
			}
			removals.push_back({it.Value(), label});
        }
		if (removals.empty()) { return; }

		try {
			doc->NewCommand();
			for (const auto& removal : removals) {
				if (!myDoc->RemoveShape(removal.second)) {
					doc->AbortCommand();
					return;
				}
			}
			if (!doc->CommitCommand()) {
				if (doc->HasOpenCommand()) { doc->AbortCommand(); }
				return;
			}
		} catch (...) {
			if (doc->HasOpenCommand()) { doc->AbortCommand(); }
			return;
		}

		_manipulator->Detach();
		for (const auto& removal : removals) {
			myContext->Remove(removal.first, Standard_False);
		}
		myDoc->NotifyChanges();
        myContext->UpdateCurrentViewer();
    }

    void ObjectInteractor::duplicateSelected() {
        auto doc = myDoc->ChangeDocument();
		if (doc.IsNull() || doc->HasOpenCommand()) { return; }
        
        Handle(AIS_InteractiveObject) selected;
        Bnd_Box overallBox;
        for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
            selected = myContext->SelectedInteractive();
            
            Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(selected);
            if (shape.IsNull() || shape->Shape().IsNull())
                continue;

            Bnd_Box b;
            shape->BoundingBox(b);
            
            if(b.IsVoid()) {
                continue;
            }
            if(overallBox.IsVoid()) {
                overallBox = b;
            } else {
                overallBox.Add(b);
            }
        }

		if (overallBox.IsVoid()) { return; }
		
        Standard_Real xmin, xmax, ymin, ymax, zmin, zmax;
        overallBox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        const Standard_Real w = xmax - xmin;
        const Standard_Real d = ymax - ymin;
        
        gp_Trsf minAxisDisplacement;
        
        minAxisDisplacement.SetTranslation((w>d)?(gp_Vec){0., d, 0.}:(gp_Vec){w, 0., 0.});
        
        std::vector<Handle(AIS_InteractiveObject)> copyInteractives;
        
        for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
            selected = myContext->SelectedInteractive();
            Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(selected);
			if (shape.IsNull() || shape->Shape().IsNull())
				continue;
			
            copyInteractives.push_back(selected);
        }
        
		struct DuplicateRecord {
			Handle(AIS_Shape) presentation;
			TDF_Label sourceLabel;
		};
		std::vector<DuplicateRecord> duplicates;
		try {
			for (const auto& item : copyInteractives) {
				Handle(AIS_Shape) source = Handle(AIS_Shape)::DownCast(item);
				BRepBuilderAPI_Copy shapeCopy;
				shapeCopy.Perform(source->Shape(), Standard_True, Standard_False);
				if (!shapeCopy.IsDone() || !IsTopologicallyValid(shapeCopy.Shape())) { return; }
				Handle(AIS_Shape) copy = new AIS_Shape(shapeCopy.Shape());
				copy->SetLocalTransformation(
					source->LocalTransformation().Multiplied(minAxisDisplacement));
				const TDF_Label sourceLabel = myDoc->ShapeLabel(item);
				if (sourceLabel.IsNull()) { return; }
				myDoc->LoadObjectMeterial(sourceLabel, copy);
				duplicates.push_back({copy, sourceLabel});
			}
		} catch (...) {
			return;
		}
		if (duplicates.empty()) { return; }

		try {
			doc->NewCommand();
			for (const auto& duplicate : duplicates) {
				const TDF_Label label = myDoc->AddShape(duplicate.presentation);
				if (label.IsNull()) {
					doc->AbortCommand();
					return;
				}
				if (!myDoc->CopyObjectAppearance(
						duplicate.sourceLabel, label)) {
					doc->AbortCommand();
					return;
				}
				myDoc->LoadObjectMeterial(label, duplicate.presentation);
			}
			if (!doc->CommitCommand()) {
				if (doc->HasOpenCommand()) { doc->AbortCommand(); }
				return;
			}
		} catch (...) {
			if (doc->HasOpenCommand()) { doc->AbortCommand(); }
			return;
		}

		_manipulator->Detach();
		myContext->ClearSelected(Standard_False);
		for (const auto& duplicate : duplicates) {
			myContext->Display(duplicate.presentation, AIS_Shaded, 0, Standard_False);
			myContext->AddSelect(duplicate.presentation);
			_manipulator->Attach(duplicate.presentation);
		}
		myContext->HilightSelected(Standard_True);
		myDoc->NotifyChanges();
		_manipulator->Redisplay();
		myContext->UpdateCurrentViewer();
    }

    const bool ObjectInteractor::isSelected() const {
//        printf(">>> ObjectInteractor::isSelected - %d\n", !myContext->FirstSelectedObject().IsNull());
        return !myContext->FirstSelectedObject().IsNull();
    }

    void ObjectInteractor::attachManipulatorToSelection(bool detach) {
        if (_manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeNone) { return; }

		if (detach) {
			myContext->InitDetected();
			if (myContext->MoreDetected()) {
				auto detected = myContext->DetectedInteractive();
                if (!_manipulator.IsNull() && !_manipulator->Objects().IsNull()) {
                    if (_manipulator->Objects()->Size() > 1) {
                        detachManipulator(detected);
                        return;
                    }
                }
			}
		}
		
        Handle(AIS_InteractiveObject) selected;
        for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected())
        {
            selected = myContext->SelectedInteractive();
        }
        
        if(selected.IsNull()) {
            if(!_manipulator.IsNull()) {
                _manipulator->Detach();
                myContext->UpdateCurrentViewer();
            }
            return;
        }
        
        attachManipulator(selected);
		return;
    }

	void ObjectInteractor::setObjectTransparent(Handle(AIS_InteractiveObject) selected, const bool on) {
		
		Quantity_Color color = Quantity_Color(on ? Quantity_NameOfColor::Quantity_NOC_BLUE : Quantity_NameOfColor::Quantity_NOC_GRAY80);
		
		Graphic3d_MaterialAspect mat = selected->Material();
		mat.SetAlpha(on ? 0.4 : 1.0);
		selected->SetMaterial(mat);
		selected->SetColor(color);
		myContext->SetMaterial(selected, mat, Standard_True);
	}

    void ObjectInteractor::setManipulatorType(PrimitiveManipulatorType type) {
		if (!_manipulator.IsNull() && _manipulator->HasActiveTransformation()) {
			cancelInteraction();
		}
        _manipulatorType = type;
        createManipulatorIfNeeded();
        bool scale = type == PrimitiveManipulatorType::PrimitiveGizmoTypeScale;
        bool movRot = type == PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate;
        bool mirror = type == PrimitiveManipulatorType::PrimitiveGizmoTypeMirror;
        for(auto axis = 0; axis < 3; ++axis) {
            _manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_Scaling, scale);
			_manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_ScalingUniform, scale);
            _manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_Translation, movRot && !mirror);
            _manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_Rotation, movRot && !mirror);
            _manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_TranslationPlane, Standard_False);
			_manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_MirroringPlanePos, mirror);
			_manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_MirroringPlaneNeg, mirror);
        }
        
        auto objects = _manipulator->Objects();
		if(!objects.IsNull() && objects->Size() > 0) {
            if (_manipulator->IsAttached()) {
                _manipulator->Detach();
            }
            if (_manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer
                && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeNone) {
				_manipulator->Attach(objects);
            }
        }

		_manipulator->Redisplay();

        if (type == PrimitiveManipulatorType::PrimitiveGizmoTypeNone) { // this trick: remove manipulator after operation finished
            _manipulator->DeactivateCurrentMode();
            myContext->Remove(_manipulator, Standard_True);
        }

        myContext->UpdateCurrentViewer();
    }

    bool ObjectInteractor::transformManipulator(const int theX, const int theY) {
        if(!_manipulator.IsNull() && _manipulator->IsAttached()) {
            if(_manipulator->HasActiveMode()) {
                _manipulator->Transform(theX, theY, myView, myContext);
                myContext->UpdateCurrentViewer();
                return true;
            }
        }
        return false;
    }

    bool ObjectInteractor::startTransformManipulator(const int theX, const int theY) {
        if(!_manipulator.IsNull() && _manipulator->IsAttached()) {
            myContext->MoveTo(theX, theY, myView, Standard_False);
            myContext->UpdateCurrentViewer();
            if(_manipulator->HasActiveMode()) {
                _manipulator->StartTransform(theX, theY, myView, myContext);
                return true;
            }
        }
        return false;
    }

    void ObjectInteractor::finishInteraction() {
        if(!_manipulator.IsNull() && _manipulator->IsAttached() && _manipulator->HasActiveMode()) {
			const AIS_ManipulatorMode activeMode = _manipulator->ActiveMode();
			const bool isMirrorPlane = _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeMirror
				&& (activeMode
						== AIS_ManipulatorMode::AIS_MM_MirroringPlaneNeg
					|| activeMode
						== AIS_ManipulatorMode::AIS_MM_MirroringPlanePos);
			if (isMirrorPlane) {
				// Mirror handles select an operation plane; they intentionally do
				// not mutate the source presentation. Resolve the transient preview
				// before the transform-only no-op guard below.
				_manipulator->StopTransform(Standard_False);
				tryMirror(
					_manipulator->ActiveAxisIndex(),
					activeMode
						== AIS_ManipulatorMode::AIS_MM_MirroringPlaneNeg);
				_manipulator->DeactivateCurrentMode();
				myContext->ClearDetected(Standard_False);
				_manipulator->Redisplay();
				myContext->UpdateCurrentViewer();
				return;
			}

			if (!_manipulator->HasActiveTransformation()) {
				cancelInteraction();
				return;
			}
			const auto &cachedShapes = _manipulator->cachedShapes();
			Handle(Core3DManipulatorObjectSequence) objects =
				_manipulator->Objects();
			if (objects.IsNull()) {
				cancelInteraction();
				return;
			}
			bool didChange = false;
			Standard_Integer objectIndex = 1;
			for (Core3DManipulatorObjectSequence::Iterator object(*objects);
			     object.More(); object.Next(), ++objectIndex) {
				const Handle(AIS_InteractiveObject)& current = object.Value();
				const auto cached = cachedShapes.find(current);
				const Handle(AIS_Shape) presentation =
					Handle(AIS_Shape)::DownCast(current);
				if (cached == cachedShapes.end()
				    || cached->second.IsNull()
				    || presentation.IsNull()
				    || presentation->Shape().IsNull()) {
					cancelInteraction();
					return;
				}
				didChange = didChange
					|| TransformDiffers(
						presentation->LocalTransformation(),
						_manipulator->StartTransformation(objectIndex))
					|| !presentation->Shape().IsEqual(cached->second);
			}
			if (!didChange) {
				cancelInteraction();
				return;
			}

            Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myDoc->Document()->Main());
            auto doc = myDoc->ChangeDocument();
			if (doc.IsNull() || doc->HasOpenCommand()) {
				cancelInteraction();
				return;
			}

			std::vector<std::pair<Handle(AIS_Shape), TDF_Label>> changes;
			for (const auto& cachedShape : cachedShapes) {
				Handle(AIS_Shape) presentation = Handle(AIS_Shape)::DownCast(cachedShape.first);
				TDF_Label label;
				if (presentation.IsNull()
					|| presentation->Shape().IsNull()
					|| !shapeTool->FindShape(cachedShape.second, label)
					|| label.IsNull()
					|| (_manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeScale
						&& !IsTopologicallyValid(presentation->Shape()))) {
					cancelInteraction();
					return;
				}
				changes.push_back({presentation, label});
			}
			if (changes.empty()) {
				cancelInteraction();
				return;
			}

			try {
				doc->NewCommand();
				for (const auto& change : changes) {
					if (_manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeScale) {
						myDoc->ReplaceShape(change.second, change.first);
					}
					myDoc->SaveObjectTransform(change.second, change.first);
				}
				if (!doc->CommitCommand()) {
					if (doc->HasOpenCommand()) { doc->AbortCommand(); }
					cancelInteraction();
					return;
				}
			} catch (...) {
				if (doc->HasOpenCommand()) { doc->AbortCommand(); }
				cancelInteraction();
				return;
			}

			_manipulator->StopTransform(Standard_True);
			myDoc->NotifyChanges();

			_manipulator->UpdateCachedShapes();
            
            _manipulator->DeactivateCurrentMode();
            // A touch release has no continuing pointer hover. OCCT otherwise
            // retains the last detected handle and can immediately reactivate
            // its mode during redisplay, leaving the idle presentation in an
            // unsafe pseudo-active state after a successful transform.
            myContext->ClearDetected(Standard_False);
            _manipulator->Redisplay();
            myContext->UpdateCurrentViewer();
        }
    }

    void ObjectInteractor::cancelInteraction() {
        if(!_manipulator.IsNull() && _manipulator->IsAttached()) {
			_manipulator->StopTransform(Standard_False);
			for (const auto& cachedShape : _manipulator->cachedShapes()) {
				Handle(AIS_Shape) presentation = Handle(AIS_Shape)::DownCast(cachedShape.first);
				if (!presentation.IsNull() && !cachedShape.second.IsNull()) {
					presentation->SetShape(cachedShape.second);
					myContext->Redisplay(presentation, Standard_False);
				}
            }
            _manipulator->DeactivateCurrentMode();
			myContext->ClearDetected(Standard_False);
			_manipulator->UpdateCachedShapes();
            _manipulator->Redisplay();
            myContext->UpdateCurrentViewer();
        }
    }

    const bool ObjectInteractor::isManipulatorAttached() const {
        return !_manipulator.IsNull() && _manipulator->IsAttached();
    }

    const bool ObjectInteractor::isManipulatorInteractionActive() const {
        return !_manipulator.IsNull()
            && _manipulator->IsAttached()
            && _manipulator->HasActiveMode();
    }

    void ObjectInteractor::SelectAndAttachManipulator(Handle(AIS_InteractiveObject) toObject) {
        myContext->SetSelected(toObject, Standard_True);
        attachManipulatorToSelection();
    }

    const PrimitiveManipulatorType ObjectInteractor::getManipulatorType() const {
        return _manipulatorType;
    }

	void ObjectInteractor::fillSelectedState(Standard_Boolean forceActor, BooleanAction action) {
		const auto failClosed = [this]() noexcept {
			_booleanOpController.cancelActive();
			try {
				if (!myContext.IsNull()) {
					myContext->ClearSelected(Standard_True);
				}
			} catch (...) {
			}
		};

		try {
			std::array<Handle(AIS_InteractiveObject),
				BooleanOperationController::kMaxSourceOperands> selectedObjects;
			std::size_t selectedCount = 0;
			for (myContext->InitSelected();
				 myContext->MoreSelected();
				 myContext->NextSelected()) {
				if (selectedCount >= selectedObjects.size()) {
					failClosed();
					return;
				}
				selectedObjects[selectedCount++] =
					myContext->SelectedInteractive();
			}
			for (std::size_t index = 0; index < selectedCount; ++index) {
				_booleanOpController.updateDetectedState(
					selectedObjects[index],
					Handle(SelectMgr_EntityOwner)(),
					forceActor,
					action);
				if (forceActor) {
					forceActor = false;
				}
			}
			_booleanOpController.visualApply(action);
		} catch (...) {
			failClosed();
		}
	}

	void ObjectInteractor::updateDetectedState(Standard_Boolean forceActor, BooleanAction action) {
		if (_booleanOpController.isSelectionFrozen()) {
			return;
		}
		std::size_t selectedCount = 0;
		for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
			++selectedCount;
		}
		if (selectedCount > BooleanOperationController::kMaxSourceOperands) {
			_booleanOpController.cancelActive();
			myContext->ClearSelected(Standard_True);
			return;
		}
		if (myContext->HasDetected()) {
			_booleanOpController.updateDetectedState(myContext->DetectedInteractive(), myContext->DetectedOwner(), forceActor, action);
			_booleanOpController.visualApply(action);
		}
	}

	Standard_Boolean ObjectInteractor::beginBoolean(BooleanAction action) noexcept {
		return _booleanOpController.begin(action);
	}

	BooleanApplyResult ObjectInteractor::applyBoolean(BooleanAction action) noexcept {
		return _booleanOpController.apply(action);
	}

	void ObjectInteractor::cancelBoolean(BooleanAction action) noexcept {
		_booleanOpController.cancel(action);
		try {
			attachManipulatorToSelection();
		} catch (...) {
		}
	}

	void ObjectInteractor::cancelActiveBoolean() noexcept {
		_booleanOpController.cancelActive();
		try {
			attachManipulatorToSelection();
		} catch (...) {
		}
	}

    const bool ObjectInteractor::canApplyBoolean() const {
        return _booleanOpController.canApply();
    }

	const bool ObjectInteractor::hasActiveBoolean() const {
		return _booleanOpController.hasActiveOperation();
	}

	const bool ObjectInteractor::hasUnresolvedBoolean() const {
		return _booleanOpController.hasUnresolvedState();
	}

	const bool ObjectInteractor::isBooleanSelectionFrozen() const {
		return _booleanOpController.isSelectionFrozen();
	}

#ifdef DEBUG
	Standard_Boolean ObjectInteractor::debugBeginBooleanSelection(
		const std::vector<Handle(AIS_InteractiveObject)>& actors,
		const std::vector<Handle(AIS_InteractiveObject)>& subjects,
		BooleanAction action) noexcept {
		try {
			if ((action == BooleanAction::BooleanUnion && !actors.empty())
				|| actors.size() + subjects.size()
					> BooleanOperationController::kMaxSourceOperands
				|| (action == BooleanAction::BooleanSubtract
					&& (actors.empty() || subjects.empty()))
				|| (action == BooleanAction::BooleanUnion
					&& subjects.size() < 2)) {
				return Standard_False;
			}
			_booleanOpController.cancelActive();
			if (!_booleanOpController.begin(action)) {
				return Standard_False;
			}
			for (const Handle(AIS_InteractiveObject)& actor : actors) {
				if (!_booleanOpController.setSelectionState(
						actor,
						BooleanSelectionType::Actor,
						action)) {
					_booleanOpController.cancelActive();
					return Standard_False;
				}
			}
			for (const Handle(AIS_InteractiveObject)& subject : subjects) {
				if (!_booleanOpController.setSelectionState(
						subject,
						BooleanSelectionType::Subject,
						action)) {
					_booleanOpController.cancelActive();
					return Standard_False;
				}
			}
			_booleanOpController.visualApply(action);
			return _booleanOpController.canApply();
		} catch (...) {
			_booleanOpController.cancelActive();
			return Standard_False;
		}
	}

	Standard_Boolean ObjectInteractor::debugRecomputeBooleanPreview(
		BooleanAction action) noexcept {
		try {
			if (!_booleanOpController.hasActiveOperation()) {
				return Standard_False;
			}
			_booleanOpController.visualApply(action);
			return _booleanOpController.canApply();
		} catch (...) {
			_booleanOpController.cancelActive();
			return Standard_False;
		}
	}
#endif

	void ObjectInteractor::tryMirror(
		Standard_Integer axisIndex,
		bool backward) noexcept {
		try {
			tryMirrorImpl(axisIndex, backward);
		} catch (...) {
			clearTrialMirrorObjects();
		}
	}

	void ObjectInteractor::tryMirrorImpl(
		Standard_Integer axisIndex,
		bool backward) {
		if (!_trialMirrorObjects.empty() && !_trialMirrorObjectsValid) {
			clearTrialMirrorObjects();
			if (!_trialMirrorObjects.empty()) {
				return;
			}
		}
		if (_manipulator.IsNull()
			|| !_manipulator->IsAttached()
			|| axisIndex < 0
			|| axisIndex > 2) {
			return;
		}

		const gp_Ax2 &manipulatorTransform = _manipulator->Position();
		
		Standard_ShortReal sign = backward ? -1.0f : 1.0f;
		
		Handle(Core3DManipulatorObjectSequence) anObjects = _manipulator->Objects();
		if (anObjects.IsNull() || anObjects->Size() == 0
			|| static_cast<std::size_t>(anObjects->Size())
				> kMaxMirrorPreviewBodies) {
			return;
		}
		Core3DManipulatorObjectSequence::Iterator anObjIter (*anObjects);
		std::vector<Handle(AIS_Shape)> replacementObjects;
		replacementObjects.reserve(
			static_cast<std::size_t>(anObjects->Size()));
		std::vector<TDF_Label> replacementSourceLabels;
		replacementSourceLabels.reserve(
			static_cast<std::size_t>(anObjects->Size()));
		
		Bnd_Box aBox, aBoxSum;
	
		for (auto &obj : *anObjects) { //calculate summary bounding box
			obj->BoundingBox (aBox);
			aBoxSum.Add(aBox);
		}
		
		for (; anObjIter.More(); anObjIter.Next()) {
			gp_Pnt offset = manipulatorTransform.Location();
		
			Handle(AIS_InteractiveObject) selected = anObjIter.Value();
			
			Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(selected);
			if (shape.IsNull() || shape->Shape().IsNull()) {
				return;
			}
			
			gp_Trsf aTrsfSelected = selected->Transformation();
			gp_Pnt max = aBoxSum.CornerMax();
			gp_Pnt min = aBoxSum.CornerMin();
			gp_Dir mirrorAxis;
			gp_Dir mirrorPln;
			
			switch (axisIndex) {
				case 0:
					mirrorAxis = gp_Dir(1.0, 0.0, 0.0);
					mirrorPln = gp_Dir(0.0, 1.0, 0.0);
					offset.Translate(gp_Vec(abs(max.X() - min.X()) * 0.5f * sign, 0.0, 0.0));
					break;
				case 1:
					mirrorAxis = gp_Dir(0.0, 1.0, 0.0);
					mirrorPln = gp_Dir(0.0, 0.0, 1.0);
					offset.Translate(gp_Vec(0.0, abs(max.Y() - min.Y()) * 0.5f * sign, 0.0));
					break;
				case 2:
					mirrorAxis = gp_Dir(0.0, 0.0, 1.0);
					mirrorPln = gp_Dir(1.0, 0.0, 0.0);
					offset.Translate(gp_Vec(0.0, 0.0, abs(max.Z() - min.Z()) * 0.5f * sign));
					break;
				default:
					break;
			}
			
			mirrorAxis.Transform(aTrsfSelected.Inverted());
			mirrorPln.Transform(aTrsfSelected.Inverted());
			offset.Transform(aTrsfSelected.Inverted());

			gp_Trsf aTrsfMirror;
			aTrsfMirror.SetMirror(gp_Ax2(offset, mirrorAxis, mirrorPln));
			
			// OCCT's negative-transform mesh-copy path corrupts allocator state
			// when mirror previews are replaced repeatedly. Keep mesh copying
			// disabled; AIS Display below triangulates the owned trial before
			// renderer-neutral capture reads that cache.
			BRepBuilderAPI_Transform aBRepTrsf(
				shape->Shape(),
				aTrsfSelected * aTrsfMirror,
				Standard_False,
				Standard_False);
			Handle(AIS_Shape) aShapePrs = new AIS_Shape (aBRepTrsf.Shape());
			const TDF_Label sourceLabel = myDoc->ShapeLabel(selected);
			if (sourceLabel.IsNull()) {
				return;
			}
			myDoc->LoadObjectMeterial(sourceLabel, aShapePrs);
			replacementObjects.push_back(aShapePrs);
			replacementSourceLabels.push_back(sourceLabel);
		}

		// A plane is a choice for the current mirror operation, not an
		// additional operation. Take ownership of every handle that might be
		// displayed before mutating OCCT. Until the full swap succeeds the set
		// is deliberately non-applicable, and cleanup retains any handle whose
		// erase operation fails so no presentation can become orphaned.
		const std::vector<Handle(AIS_Shape)> previousObjects =
			_trialMirrorObjects;
		std::vector<Handle(AIS_Shape)> transitionObjects =
			previousObjects;
		transitionObjects.insert(
			transitionObjects.end(),
			replacementObjects.begin(),
			replacementObjects.end());
		_trialMirrorObjects = std::move(transitionObjects);
		_trialMirrorObjectsValid = false;

		for (const Handle(AIS_Shape)& aShapePrs : replacementObjects) {
			myContext->Display(
				aShapePrs,
				AIS_Shaded,
				(Standard_Integer)0,
				Standard_False);
			// Mirror trials are explicit presentation only. Keep the OCCT
			// fallback behavior aligned with the renderer-neutral nonselectable
			// contract while the operation is pending.
			myContext->Deactivate(aShapePrs);
		}
		for (const Handle(AIS_Shape)& aShapePrs
			 : previousObjects) {
			myContext->Erase(aShapePrs, Standard_False);
		}
		_trialMirrorObjects = std::move(replacementObjects);
		_trialMirrorSourceLabels.clear();
		for (std::size_t index = 0;
			 index < _trialMirrorObjects.size(); ++index) {
			_trialMirrorSourceLabels.emplace(
				_trialMirrorObjects[index].get(),
				replacementSourceLabels[index]);
		}
		_trialMirrorObjectsValid = true;
		myContext->UpdateCurrentViewer();
	}

	void ObjectInteractor::clearTrialMirrorObjects() noexcept {
		_trialMirrorObjectsValid = false;
		std::vector<Handle(AIS_Shape)> unresolvedObjects;
		try {
			unresolvedObjects.reserve(_trialMirrorObjects.size());
		} catch (...) {
			return;
		}
		for (const Handle(AIS_Shape)& aShapePrs
			 : _trialMirrorObjects) {
			try {
				myContext->Erase(aShapePrs, Standard_False);
			} catch (...) {
				unresolvedObjects.push_back(aShapePrs);
			}
		}
		_trialMirrorObjects = std::move(unresolvedObjects);
		_trialMirrorSourceLabels.clear();
		try {
			myContext->UpdateCurrentViewer();
		} catch (...) {
			// Cleanup state remains authoritative and can be retried later.
		}
	}

	const bool ObjectInteractor::hasTrialMirrorObjects() const {
		return _trialMirrorObjectsValid && !_trialMirrorObjects.empty();
	}

	const bool ObjectInteractor::hasUnresolvedMirrorObjects() const {
		return !_trialMirrorObjects.empty();
	}

	void ObjectInteractor::applyMirror() {
		if (!hasTrialMirrorObjects()) {
			if (hasUnresolvedMirrorObjects()) {
				clearTrialMirrorObjects();
			}
			return;
		}

		auto doc = myDoc->ChangeDocument();
		if (doc.IsNull() || doc->HasOpenCommand()) {
			clearTrialMirrorObjects();
			return;
		}
		if (_trialMirrorSourceLabels.size()
			!= _trialMirrorObjects.size()) {
			clearTrialMirrorObjects();
			return;
		}
		for (const Handle(AIS_Shape)& shape : _trialMirrorObjects) {
			if (shape.IsNull() || !IsTopologicallyValid(shape->Shape())) {
				clearTrialMirrorObjects();
				return;
			}
		}

		try {
			doc->NewCommand();
			for (const Handle(AIS_Shape)& shape : _trialMirrorObjects) {
				const auto source =
					_trialMirrorSourceLabels.find(shape.get());
				if (source == _trialMirrorSourceLabels.end()) {
					doc->AbortCommand();
					clearTrialMirrorObjects();
					return;
				}
				const TDF_Label label = myDoc->AddShape(shape);
				if (label.IsNull()) {
					doc->AbortCommand();
					clearTrialMirrorObjects();
					return;
				}
				if (!myDoc->CopyObjectAppearance(source->second, label)) {
					doc->AbortCommand();
					clearTrialMirrorObjects();
					return;
				}
				myDoc->LoadObjectMeterial(label, shape);
			}
			if (!doc->CommitCommand()) {
				if (doc->HasOpenCommand()) { doc->AbortCommand(); }
				clearTrialMirrorObjects();
				return;
			}
		} catch (...) {
			if (doc->HasOpenCommand()) { doc->AbortCommand(); }
			clearTrialMirrorObjects();
			return;
		}
		myDoc->NotifyChanges();
		_trialMirrorObjects.clear();
		_trialMirrorSourceLabels.clear();
		_trialMirrorObjectsValid = false;
	}
}
