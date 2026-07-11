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
        scene::PresentationOverlayContent& theContent) const noexcept {
        try {
            theContent = {};
            if (_manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeNone) {
                return PresentationOverlayCaptureStatus::Available;
            }
            const bool isMoveRotate = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate;
            const bool isScale = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeScale;
            if (!isMoveRotate && !isScale) {
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            if (_manipulator.IsNull() || !_manipulator->IsAttached()) {
                return PresentationOverlayCaptureStatus::Available;
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
                : _manipulator->CaptureIdleScaleOverlay(theContent);
            if (!didCapture) {
                theContent = {};
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            return PresentationOverlayCaptureStatus::Available;
        } catch (...) {
            theContent = {};
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
			Graphic3d_NameOfMaterial material;
			Quantity_NameOfColor color;
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
				const Graphic3d_NameOfMaterial material = myDoc->MaterialNameForShape(source);
				Quantity_Color color;
				source->Color(color);
				copy->SetMaterial(material);
				copy->SetColor(color);
				duplicates.push_back({copy, material, color.Name()});
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
				myDoc->SaveObjectMaterial(label, duplicate.material);
				myDoc->SaveObjectColor(label, duplicate.color);
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
		for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
			_booleanOpController.updateDetectedState(myContext->SelectedInteractive(), Handle(SelectMgr_EntityOwner)(), forceActor, action);
			if (forceActor)
				forceActor = false;
		}
		_booleanOpController.visualApply(action);
	}

	void ObjectInteractor::updateDetectedState(Standard_Boolean forceActor, BooleanAction action) {
		if (myContext->HasDetected()) {
			_booleanOpController.updateDetectedState(myContext->DetectedInteractive(), myContext->DetectedOwner(), forceActor, action);
			_booleanOpController.visualApply(action);
		}
	}

	void ObjectInteractor::applyBoolean(BooleanAction action) {
		_booleanOpController.apply(action);
	}

	void ObjectInteractor::cancelBoolean(BooleanAction action) {
		_booleanOpController.cancel(action);
		attachManipulatorToSelection();
	}

    const bool ObjectInteractor::canApplyBoolean() const {
        return _booleanOpController.canApply();
    }

	void ObjectInteractor::tryMirror(Standard_Integer axisIndex, bool backward) {
		const gp_Ax2 &manipulatorTransform = _manipulator->Position();
		
		Standard_ShortReal sign = backward ? -1.0f : 1.0f;
		
		Handle(Core3DManipulatorObjectSequence) anObjects = _manipulator->Objects();
		Core3DManipulatorObjectSequence::Iterator anObjIter (*anObjects);
		
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
			
			BRepBuilderAPI_Transform aBRepTrsf(shape->Shape(), aTrsfSelected * aTrsfMirror);
			Handle(AIS_InteractiveObject) aShapePrs = new AIS_Shape (aBRepTrsf.Shape());
            aShapePrs->SetMaterial(myDoc->MaterialNameForShape(Handle(AIS_Shape)::DownCast(selected)));
            Quantity_Color color;
            selected->Color(color);
            aShapePrs->SetColor(color.Name());
			myContext->Display(aShapePrs, AIS_Shaded, (Standard_Integer)0, Standard_False);
			
			_trialMirrorObjects.push_back(aShapePrs);
		}
	}

	void ObjectInteractor::clearTrialMirrorObjects() {
		for (Handle(AIS_InteractiveObject) aShapePrs : _trialMirrorObjects) {
			myContext->Erase(aShapePrs, Standard_False);
		}
		myContext->UpdateCurrentViewer();
		_trialMirrorObjects.clear();
	}

	void ObjectInteractor::applyMirror() {
		if (_trialMirrorObjects.empty()) {
			return;
		}

		auto doc = myDoc->ChangeDocument();
		if (doc.IsNull() || doc->HasOpenCommand()) {
			clearTrialMirrorObjects();
			return;
		}
		for (const auto& trial : _trialMirrorObjects) {
			Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(trial);
			if (shape.IsNull() || !IsTopologicallyValid(shape->Shape())) {
				clearTrialMirrorObjects();
				return;
			}
		}

		try {
			doc->NewCommand();
			for (const auto& trial : _trialMirrorObjects) {
				Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(trial);
				const TDF_Label label = myDoc->AddShape(shape);
				if (label.IsNull()) {
					doc->AbortCommand();
					clearTrialMirrorObjects();
					return;
				}
				myDoc->SaveObjectMaterial(label, shape->Material());
				Quantity_Color color;
				shape->Color(color);
				myDoc->SaveObjectColor(label, color.Name());
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
	}
}
