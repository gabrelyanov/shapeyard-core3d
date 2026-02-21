//
//  BooleanOperationController.cpp
//  Core3D
//
//  Created by Vlad on 31.05.2024.
//

#include <stdio.h>
#include "BooleanOperationController.hpp"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <AIS_Shape.hxx>
#include "Core3DContext.hpp"

namespace core3d {

	constexpr const float kFuzzyValue = 1e-3f;

	BooleanOperationController::BooleanOperationController(Handle(AIS_InteractiveContext) context
                                                           , Handle(OcctDocument) doc)
		: myContext(context), myDoc(doc) {
			_canApply = Standard_False;
	}

	void BooleanOperationController::updateDetectedState(Handle(AIS_InteractiveObject) detected, Handle(SelectMgr_EntityOwner) detectedOwner, Standard_Boolean forceActor, BooleanAction action) {
		if (!detected.IsNull()) {

			TemporalBooleanObject subject;
			Standard_Integer dispMode = detected->DisplayMode();
			if (BooleanAction::BooleanSubtract == action) {
				if (-1 == dispMode && forceActor) { //set as actor
					subject.selectionType = BooleanSelectionType::Actor;
				} else if (AIS_WireFrame == dispMode || !forceActor) { //set as acted
					subject.selectionType = BooleanSelectionType::Subject;
				} else { //unselect
					subject.selectionType = BooleanSelectionType::Undefined;
				}
			} else {
				if (-1 == dispMode) { //set as acted
					subject.selectionType = BooleanSelectionType::Subject;
				} else { //unselect
					subject.selectionType = BooleanSelectionType::Undefined;
				}
			}
			
			switch(subject.selectionType) {
				case BooleanSelectionType::Undefined:
				{
					if (_selectionMap.find(detected) != _selectionMap.end()) { //and revert from copied shape
						Handle(AIS_InteractiveObject) reversionShape = _selectionMap[detected].copy;
						TopoDS_Shape shape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(reversionShape)->Shape(), reversionShape->LocalTransformation());
						_selectionMap.erase(detected);
						myContext->Remove(detected, Standard_False); //remove old shape
						detected = ioCopyWithMaterial(reversionShape);
						showInteractiveByType(detected, subject.selectionType);
					}
					break;
				}
				case BooleanSelectionType::Actor:
				{
					TopoDS_Shape shape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(detected)->Shape(), detected->LocalTransformation());
					subject.copy = ioCopyWithMaterial(detected);
                    // copy materials & colors
//                    applyMaterialAndColor(subject, detected);
                    //
					showInteractiveByType(detected, subject.selectionType);
					_selectionMap[detected] = subject;
					if (!detectedOwner.IsNull() && !detectedOwner->IsSelected()) {
						myContext->SetSelectedState(detectedOwner, Standard_False);
					}
					break;
				}
				case BooleanSelectionType::Subject:
				{
					TopoDS_Shape shape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(detected)->Shape(), detected->LocalTransformation());
					subject.copy = ioCopyWithMaterial(detected);;
                    // copy materials & colors
//                    applyMaterialAndColor(subject, detected);
                    //
					_selectionMap[detected] = subject;
					showInteractiveByType(detected, subject.selectionType);
					if (!detectedOwner.IsNull() && !detectedOwner->IsSelected()) {
						myContext->SetSelectedState(detectedOwner, Standard_False);
					}
					break;
				}
			}
		}
	}

	void BooleanOperationController::resetCachedSelection() {
		std::map<Handle(AIS_InteractiveObject), TemporalBooleanObject> selectionMapCopy = _selectionMap;
		_selectionMap.clear();
		for (const auto &obj : selectionMapCopy) {
			TemporalBooleanObject subject;
			subject.selectionType = obj.second.selectionType;
			subject.copy = obj.second.copy;
			Handle(AIS_InteractiveObject) orig = ioCopyWithMaterial(subject.copy);
			myContext->Remove(obj.first, Standard_False);
			_selectionMap[orig] = subject;
			showInteractiveByType(orig, subject.selectionType);
		}
		selectionMapCopy.clear();
	}

	void BooleanOperationController::visualApply(BooleanAction action) {
		resetCachedSelection();
        
        _actedIOArray.clear();
        _actorIOArray.clear();
        
		for (const auto &obj : _selectionMap) {
			if (BooleanSelectionType::Actor == obj.second.selectionType)
				_actorIOArray.push_back(obj.first);
			else if (BooleanSelectionType::Subject == obj.second.selectionType)
				_actedIOArray.push_back(obj.first);
		}
		if (BooleanAction::BooleanSubtract == action && _actorIOArray.size() > 0 && _actedIOArray.size() > 0) {
			_canApply = Standard_True;
			boolSubtract(_actorIOArray, _actedIOArray);
		} else if (BooleanAction::BooleanUnion == action && _actedIOArray.size() > 1) {
			_canApply = Standard_True;
		} else {
			_canApply = Standard_False;
		}
	}

	void BooleanOperationController::boolSubtract(const std::vector<Handle(AIS_InteractiveObject)> &actorIOArray,
											  const std::vector<Handle(AIS_InteractiveObject)> &actedIOArray) {
		for (Handle(AIS_InteractiveObject) actedIO : actedIOArray) {

			TopTools_ListOfShape theLSA;
			Handle(AIS_InteractiveObject) accumulatedIO = actedIO;
			TopoDS_Shape accumulatedShape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(actedIO)->Shape(), actedIO->LocalTransformation());
			theLSA.Append(accumulatedShape);

			myContext->Remove(actedIO, Standard_False);

			TopTools_ListOfShape theLST;
			for (int i = 0; i < actorIOArray.size(); ++i) {
				TopoDS_Shape actorShape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(actorIOArray[i])->Shape(), actorIOArray[i]->LocalTransformation());
				theLST.Append(actorShape);
			}

			BRepAlgoAPI_Cut cutMaker;
			cutMaker.SetArguments(theLSA);
			cutMaker.SetTools(theLST);
			cutMaker.SetFuzzyValue(kFuzzyValue);
			cutMaker.SetUseOBB(Standard_True);
//			cutMaker.SetGlue(BOPAlgo_GlueFull);
			cutMaker.SetCheckInverted(Standard_True);
			cutMaker.Build();

			if (cutMaker.IsDone()) {
				cutMaker.SimplifyResult();
				accumulatedIO = new AIS_Shape(cutMaker.Shape());
				accumulatedShape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(accumulatedIO)->Shape(), accumulatedIO->LocalTransformation());
			}
			
			copyMaterial(accumulatedIO, actedIO);
			myContext->SetSelected(accumulatedIO, Standard_False);
			showInteractiveByType(accumulatedIO, BooleanSelectionType::Subject);
			
			//replace old cached subject to modified, and save original shape copy
			TemporalBooleanObject subject;
			subject.selectionType = BooleanSelectionType::Subject;
			subject.copy = actedIO;
			_selectionMap.erase(actedIO);
			_selectionMap[accumulatedIO] = subject;
		}
	
	}

	void BooleanOperationController::copyMaterial(Handle(AIS_InteractiveObject) &to, const Handle(AIS_InteractiveObject) &from) {
		to->UnsetColor();
		to->SetMaterial(myDoc->MaterialNameForShape(Handle(AIS_Shape)::DownCast(from)));
		Quantity_Color c;
		from->Color(c);
		to->SetColor(c.Name());
	}


	Handle(AIS_InteractiveObject) BooleanOperationController::ioCopyWithMaterial(const Handle(AIS_InteractiveObject) &orig) {

		TopoDS_Shape shape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(orig)->Shape(), orig->LocalTransformation());
		Handle(AIS_InteractiveObject) copyIO = new AIS_Shape(shape);
		copyMaterial(copyIO, orig);
		return copyIO;
	}

	void BooleanOperationController::boolUnion(const std::vector<Handle(AIS_InteractiveObject)> &actedIOArray) {
        
        auto doc = myDoc->ChangeDocument();
        if(doc->HasOpenCommand()) {
            doc->AbortCommand();
        }
        doc->NewCommand();

		TopTools_ListOfShape theLSA;
		Handle(AIS_InteractiveObject) accumulatedIO = actedIOArray.front();
		TopoDS_Shape accumulatedShape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(accumulatedIO)->Shape(), accumulatedIO->LocalTransformation());
		theLSA.Append(accumulatedShape);


		TopTools_ListOfShape theLST;
		for (int i = 1; i < actedIOArray.size(); ++i) {
			TopoDS_Shape actedShape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(actedIOArray[i])->Shape(), actedIOArray[i]->LocalTransformation());
			theLST.Append(actedShape);
		}

		BRepAlgoAPI_Fuse fuseMaker;
		fuseMaker.SetArguments(theLSA);
		fuseMaker.SetTools(theLST);
		fuseMaker.SetFuzzyValue(kFuzzyValue);
		fuseMaker.SetUseOBB(Standard_True);
//		fuseMaker.SetGlue(BOPAlgo_GlueFull);
		fuseMaker.SetCheckInverted(Standard_True);
		fuseMaker.Build();

		if (fuseMaker.IsDone()) {
			fuseMaker.SimplifyResult();

			accumulatedIO = new AIS_Shape(fuseMaker.Shape());
			copyMaterial(accumulatedIO, actedIOArray.front());

			for (int i = 0; i < actedIOArray.size(); ++i) {
				myContext->Remove(actedIOArray[i], Standard_False);
				myDoc->RemoveShape(actedIOArray[i]);
			}

			showInteractiveByType(accumulatedIO, BooleanSelectionType::Undefined);

			myDoc->AddShape(accumulatedIO);

			doc->CommitCommand();
			myDoc->NotifyChanges();
		}

	}

	void BooleanOperationController::showInteractiveByType(const Handle(AIS_InteractiveObject) shape, BooleanSelectionType type) {
		if (shape.IsNull())
			return;
		
		Quantity_Color color;
		Standard_Boolean showAsWireframe = Standard_False;
		switch (type) {
			case BooleanSelectionType::Undefined:
				color = Quantity_NameOfColor::Quantity_NOC_GRAY;
				break;
			case BooleanSelectionType::Actor:
				showAsWireframe = Standard_True;
				color = Quantity_NameOfColor::Quantity_NOC_ORANGE;
				break;
			case BooleanSelectionType::Subject:
				color = Quantity_NameOfColor::Quantity_NOC_LIGHTSKYBLUE;
				break;
			default:
				color = Quantity_NameOfColor::Quantity_NOC_GRAY;
				break;
		}
		
        Handle(Prs3d_Drawer) highlightStyle = new Prs3d_Drawer();
        highlightStyle->SetColor(color);
        shape->SetHilightAttributes(highlightStyle);
        myContext->HilightWithColor(shape, highlightStyle, Standard_False);
		
		bool selected = false;
		shape->SetDisplayMode(showAsWireframe ? AIS_WireFrame : AIS_Shaded);
		
		if (BooleanSelectionType::Undefined == type)
			shape->UnsetDisplayMode();
		else
			selected = true;
		
//		shape->SetColor(Core3DContext::kDefaultObjectColor);

		myContext->Display (shape, showAsWireframe ? AIS_WireFrame : AIS_Shaded, TopAbs_ShapeEnum::TopAbs_SHAPE, Standard_False);
		if (selected)
			myContext->HilightWithColor(shape, highlightStyle, Standard_True);
	}

	void BooleanOperationController::apply(BooleanAction action) {
        
		if (BooleanAction::BooleanSubtract == action) {

            auto doc = myDoc->ChangeDocument();
            if(doc->HasOpenCommand()) {
                doc->AbortCommand();
            }
            doc->NewCommand();
            
            // remove acted and actors
            for(auto a : _actedIOArray) {
                myDoc->RemoveShape(a);
            }

            for(auto a : _actorIOArray) {
                myDoc->RemoveShape(a);
            }

            for (const auto &obj : _selectionMap) {
				myContext->Remove(obj.second.copy, Standard_False);
                myDoc->RemoveShape(obj.second.copy);

                if (BooleanSelectionType::Actor == obj.second.selectionType) {
                    myContext->Remove(obj.first, Standard_False);
                }
                else {
                    // add result of sub
                    showInteractiveByType(obj.first, BooleanSelectionType::Undefined);
                    myDoc->AddShape(obj.first);
                }
			}
            doc->CommitCommand();
            myDoc->NotifyChanges();

		} else if (BooleanAction::BooleanUnion == action) {
			std::vector<Handle(AIS_InteractiveObject)> actedIOArray;
			for (const auto &obj : _selectionMap) {
				if (BooleanSelectionType::Subject == obj.second.selectionType)
					actedIOArray.push_back(obj.first);
				showInteractiveByType(obj.first, BooleanSelectionType::Undefined);
			}
			if (actedIOArray.size() > 1)
				boolUnion(actedIOArray);
		}
		
		myContext->ClearSelected(Standard_True);
		_selectionMap.clear();
	}

	void BooleanOperationController::cancel(BooleanAction action) {
		if (BooleanAction::BooleanSubtract == action) {
			for (const auto &obj : _selectionMap) {
				myContext->Remove(obj.first, Standard_False);
				showInteractiveByType(obj.second.copy, BooleanSelectionType::Undefined);
			}
        } else if (BooleanAction::BooleanUnion == action) {
            for (const auto &obj : _selectionMap) {
                showInteractiveByType(obj.first, BooleanSelectionType::Undefined);
            }
        }

		myContext->ClearSelected(Standard_True);
		_selectionMap.clear();
	}
}
