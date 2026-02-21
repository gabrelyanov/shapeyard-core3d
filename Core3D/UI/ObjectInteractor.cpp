//
//  ObjectInteractor.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#include "ObjectInteractor.hpp"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <V3d_View.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDataStd_Real.hxx>
#include <GP_Quaternion.hxx>
#include <AIS_Shape.hxx>

namespace core3d {
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
			_manipulator->Detach(fromObject);
			myContext->UpdateCurrentViewer();
		}
	}

	void ObjectInteractor::detachManipulator(bool updateViewer) {
		if (!_manipulator.IsNull()) {
			_manipulator->Detach();
			if (updateViewer)
				myContext->UpdateCurrentViewer();
		}
	}

	void ObjectInteractor::	selectAll() {
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

    void ObjectInteractor::deleteSelected() {
        
        if (_manipulator.IsNull() || !_manipulator->IsAttached()) { return; }
        
        Handle(Core3DManipulatorObjectSequence) objects = _manipulator->Objects();
        _manipulator->Detach();
        
        myDoc->Document()->NewCommand();
        for (Core3DManipulatorObjectSequence::Iterator it(*objects); it.More(); it.Next()) {
            
            myDoc->RemoveShape(it.Value());
            myContext->Remove(it.Value(), Standard_True);
        }
        // commit undo command
        myDoc->Document()->CommitCommand();
        myDoc->NotifyChanges();
        myContext->UpdateCurrentViewer();
    }

    void ObjectInteractor::duplicateSelected() {
        
        auto doc = myDoc->ChangeDocument();
        if(doc->HasOpenCommand()) {
            doc->CommitCommand();
        }
        doc->NewCommand();
        
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

		if (overallBox.IsVoid()) //no selected
			return;
		
        Standard_Real xmin, xmax, ymin, ymax, zmin, zmax;
        overallBox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        const Standard_Real w = xmax - xmin;
        const Standard_Real d = ymax - ymin;
        
        gp_Trsf minAxisDisplacement;
        
        minAxisDisplacement.SetTranslation((w>d)?(gp_Vec){0., d, 0.}:(gp_Vec){w, 0., 0.});
        
        std::vector<Handle(AIS_InteractiveObject)> newInteractives;
        std::vector<Handle(AIS_InteractiveObject)> copyInteractives;
        
        for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
            selected = myContext->SelectedInteractive();
            Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(selected);
			if (shape.IsNull() || shape->Shape().IsNull())
				continue;
			
            copyInteractives.push_back(selected);
        }
        
        for(auto item : copyInteractives) {
            Handle(AIS_Shape) anAis = Handle(AIS_Shape)::DownCast(item);
            
            BRepBuilderAPI_Copy shapeCopy(anAis->Shape(), Standard_True, Standard_False);
            Handle(AIS_Shape) objectCopy = new AIS_Shape(shapeCopy);
            
            newInteractives.push_back(objectCopy);
            
            objectCopy->SetLocalTransformation(anAis->LocalTransformation().Multiplied(minAxisDisplacement));
            myDoc->AddShape(objectCopy);
            
            auto ais_mat_name = myDoc->MaterialNameForShape(anAis);
            Quantity_Color qc;
            anAis->Color(qc);
            myDoc->SaveObjectMaterial(objectCopy, ais_mat_name);
            myDoc->SaveObjectColor(objectCopy, qc.Name());
            
            objectCopy->SetMaterial(ais_mat_name);
            objectCopy->SetColor(qc);
            
            myContext->Display(objectCopy, AIS_Shaded, (Standard_Integer)0, Standard_False);
        }
        
        _manipulator->Detach();
        myContext->ClearSelected(Standard_False);
        for(auto anAis : newInteractives) {
            myContext->AddSelect(anAis);
			_manipulator->Attach(anAis);
        }

        myContext->HilightSelected(Standard_True);
        
        doc->CommitCommand();
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
            
            Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myDoc->Document()->Main());

            TDF_Label label;

            auto doc = myDoc->ChangeDocument();
            if(doc->HasOpenCommand()) {
                doc->AbortCommand();
            }
			
			doc->NewCommand();
			
			const auto &cachedShapes = _manipulator->cachedShapes();
			
			for (const auto &shape : cachedShapes) {
				if(shapeTool->FindShape(shape.second, label)) {
					if(_manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeScale) {
						myDoc->ReplaceShape(label, Handle(AIS_Shape)::DownCast(shape.first));
					}
					myDoc->SaveObjectTransform(label, Handle(AIS_Shape)::DownCast(shape.first));
				}
			}
			
			doc->CommitCommand();
			myDoc->NotifyChanges();
			
			if (_manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeMirror) {
				if (_manipulator->ActiveMode() == AIS_ManipulatorMode::AIS_MM_MirroringPlaneNeg
					|| _manipulator->ActiveMode() == AIS_ManipulatorMode::AIS_MM_MirroringPlanePos) {
					tryMirror(_manipulator->ActiveAxisIndex(), _manipulator->ActiveMode() == AIS_ManipulatorMode::AIS_MM_MirroringPlaneNeg);
				}
			}
			
			_manipulator->UpdateCachedShapes();
            
            _manipulator->DeactivateCurrentMode();
            _manipulator->Redisplay();
            myContext->UpdateCurrentViewer();
        }
    }

    void ObjectInteractor::cancelInteraction() {
        if(!_manipulator.IsNull() && _manipulator->IsAttached()) {
            _manipulator->DeactivateCurrentMode();
            _manipulator->Redisplay();
            myContext->UpdateCurrentViewer();
        }
    }

    const bool ObjectInteractor::isManipulatorAttached() const {
        return !_manipulator.IsNull() && _manipulator->IsAttached();
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
		
		auto doc = myDoc->ChangeDocument();
		if(doc->HasOpenCommand()) {
			doc->CommitCommand();
		}
		doc->NewCommand();

		for (Handle(AIS_InteractiveObject) aShapePrs : _trialMirrorObjects) {
			Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(aShapePrs);
			myDoc->addSolidObject(shape->Shape());
		}
		
		_trialMirrorObjects.clear();
    }
}
