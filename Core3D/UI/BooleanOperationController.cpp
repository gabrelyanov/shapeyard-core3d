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
#include <BRepCheck_Analyzer.hxx>
#include <TopExp_Explorer.hxx>
#include <AIS_Shape.hxx>
#include <Standard_Failure.hxx>
#include "Core3DContext.hpp"

namespace core3d {
	namespace {
		Standard_Boolean IsValidSolidBooleanResult(const TopoDS_Shape& shape) {
			if (shape.IsNull()) {
				return Standard_False;
			}
			try {
				BRepCheck_Analyzer analyzer(shape, Standard_True);
				return analyzer.IsValid()
					&& TopExp_Explorer(shape, TopAbs_SOLID).More();
			} catch (...) {
				return Standard_False;
			}
		}
	}

	constexpr const float kFuzzyValue = 1e-3f;

	BooleanOperationController::BooleanOperationController(Handle(AIS_InteractiveContext) context
                                                           , Handle(OcctDocument) doc)
		: myContext(context), myDoc(doc) {
			_canApply = Standard_False;
	}

	void BooleanOperationController::updateDetectedState(Handle(AIS_InteractiveObject) detected, Handle(SelectMgr_EntityOwner) detectedOwner, Standard_Boolean forceActor, BooleanAction action) {
		if (!detected.IsNull()) {

			TemporalBooleanObject subject;
			auto existingSelection = _selectionMap.find(detected);
			if (existingSelection == _selectionMap.end()) {
				subject.documentLabel = myDoc->ShapeLabel(detected);
				subject.original = detected;
				subject.materialName = myDoc->MaterialNameForLabel(subject.documentLabel);
				subject.colorName = myDoc->ColorNameForLabel(subject.documentLabel);
			} else {
				subject = existingSelection->second;
			}
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
					auto selection = _selectionMap.find(detected);
					if (selection != _selectionMap.end()) {
						Handle(AIS_InteractiveObject) original = selection->second.original;
						forgetSubjectSelection(selection->second.documentLabel);
						_selectionMap.erase(detected);
						myContext->Remove(detected, Standard_False);
						showInteractiveByType(original, subject.selectionType);
					}
					break;
				}
				case BooleanSelectionType::Actor:
				{
					subject.copy = ioCopyWithStyle(detected, subject);
                    // copy materials & colors
//                    applyMaterialAndColor(subject, detected);
                    //
					showInteractiveByType(detected, subject.selectionType);
					_selectionMap[detected] = subject;
					forgetSubjectSelection(subject.documentLabel);
					if (!detectedOwner.IsNull() && !detectedOwner->IsSelected()) {
						myContext->SetSelectedState(detectedOwner, Standard_False);
					}
					break;
				}
				case BooleanSelectionType::Subject:
				{
					subject.copy = ioCopyWithStyle(detected, subject);
                    // copy materials & colors
//                    applyMaterialAndColor(subject, detected);
                    //
					_selectionMap[detected] = subject;
					rememberSubjectSelection(subject.documentLabel);
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
			TemporalBooleanObject subject = obj.second;
			Handle(AIS_InteractiveObject) orig = ioCopyWithStyle(subject.original, subject);
			myContext->Remove(obj.first, Standard_False);
			_selectionMap[orig] = subject;
			showInteractiveByType(orig, subject.selectionType);
		}
		selectionMapCopy.clear();
	}

	void BooleanOperationController::clearOperationState() {
		_selectionMap.clear();
		_actedIOArray.clear();
		_actorIOArray.clear();
		_subjectSelectionOrder.clear();
		_canApply = Standard_False;
	}

	void BooleanOperationController::rememberSubjectSelection(const TDF_Label& label) {
		if (label.IsNull()) {
			return;
		}
		for (const TDF_Label& selectedLabel : _subjectSelectionOrder) {
			if (selectedLabel.IsEqual(label)) {
				return;
			}
		}
		_subjectSelectionOrder.push_back(label);
	}

	void BooleanOperationController::forgetSubjectSelection(const TDF_Label& label) {
		if (label.IsNull()) {
			return;
		}
		for (auto selectedLabel = _subjectSelectionOrder.begin();
			 selectedLabel != _subjectSelectionOrder.end(); ++selectedLabel) {
			if (selectedLabel->IsEqual(label)) {
				_subjectSelectionOrder.erase(selectedLabel);
				return;
			}
		}
	}

	std::vector<Handle(AIS_InteractiveObject)>
	BooleanOperationController::orderedSubjectPresentations(
		Standard_Boolean& isComplete) const {
		std::vector<Handle(AIS_InteractiveObject)> orderedSubjects;
		Standard_Size subjectCount = 0;
		for (const auto& selection : _selectionMap) {
			if (selection.second.selectionType == BooleanSelectionType::Subject) {
				++subjectCount;
			}
		}
		orderedSubjects.reserve(subjectCount);

		isComplete = Standard_True;
		for (const TDF_Label& selectedLabel : _subjectSelectionOrder) {
			Handle(AIS_InteractiveObject) matchingSubject;
			for (const auto& selection : _selectionMap) {
				if (selection.second.selectionType != BooleanSelectionType::Subject
					|| selection.second.documentLabel.IsNull()
					|| !selection.second.documentLabel.IsEqual(selectedLabel)) {
					continue;
				}
				if (!matchingSubject.IsNull()) {
					isComplete = Standard_False;
					return {};
				}
				matchingSubject = selection.first;
			}
			if (matchingSubject.IsNull()) {
				isComplete = Standard_False;
				return {};
			}
			orderedSubjects.push_back(matchingSubject);
		}

		if (orderedSubjects.size() != subjectCount) {
			isComplete = Standard_False;
			return {};
		}
		return orderedSubjects;
	}

	void BooleanOperationController::visualApply(BooleanAction action) {
		resetCachedSelection();
        
        _actedIOArray.clear();
		_actorIOArray.clear();

		Standard_Boolean hasAllDocumentLabels = Standard_True;
		for (const auto &obj : _selectionMap) {
			hasAllDocumentLabels = hasAllDocumentLabels && !obj.second.documentLabel.IsNull();
			if (BooleanSelectionType::Actor == obj.second.selectionType)
				_actorIOArray.push_back(obj.first);
		}
		Standard_Boolean hasCompleteSubjectOrder = Standard_False;
		_actedIOArray = orderedSubjectPresentations(hasCompleteSubjectOrder);
		if (BooleanAction::BooleanSubtract == action
			&& hasAllDocumentLabels
			&& hasCompleteSubjectOrder
			&& !_actorIOArray.empty()
			&& !_actedIOArray.empty()) {
			_canApply = boolSubtract(_actorIOArray, _actedIOArray);
		} else if (BooleanAction::BooleanUnion == action
				   && hasAllDocumentLabels
				   && hasCompleteSubjectOrder
				   && _actedIOArray.size() > 1) {
			_canApply = Standard_True;
		} else {
			_canApply = Standard_False;
		}
	}

	Standard_Boolean BooleanOperationController::boolSubtract(
		const std::vector<Handle(AIS_InteractiveObject)> &actorIOArray,
		const std::vector<Handle(AIS_InteractiveObject)> &actedIOArray) {
		struct PreviewResult {
			Handle(AIS_InteractiveObject) original;
			Handle(AIS_InteractiveObject) result;
			TemporalBooleanObject state;
		};

		TopTools_ListOfShape tools;
		try {
			for (const Handle(AIS_InteractiveObject)& actorIO : actorIOArray) {
				Handle(AIS_Shape) actor = Handle(AIS_Shape)::DownCast(actorIO);
				if (actor.IsNull() || actor->Shape().IsNull()) {
					return Standard_False;
				}
				tools.Append(BRepBuilderAPI_Transform(
					actor->Shape(), actorIO->LocalTransformation()).Shape());
			}

			std::vector<PreviewResult> previews;
			previews.reserve(actedIOArray.size());
			for (const Handle(AIS_InteractiveObject)& actedIO : actedIOArray) {
				auto selection = _selectionMap.find(actedIO);
				Handle(AIS_Shape) acted = Handle(AIS_Shape)::DownCast(actedIO);
				if (selection == _selectionMap.end()
					|| selection->second.documentLabel.IsNull()
					|| acted.IsNull()
					|| acted->Shape().IsNull()) {
					return Standard_False;
				}

				TopTools_ListOfShape arguments;
				arguments.Append(BRepBuilderAPI_Transform(
					acted->Shape(), actedIO->LocalTransformation()).Shape());

				BRepAlgoAPI_Cut cutMaker;
				cutMaker.SetArguments(arguments);
				cutMaker.SetTools(tools);
				cutMaker.SetFuzzyValue(kFuzzyValue);
				cutMaker.SetUseOBB(Standard_True);
//				cutMaker.SetGlue(BOPAlgo_GlueFull);
				cutMaker.SetCheckInverted(Standard_True);
				cutMaker.Build();
				if (!cutMaker.IsDone()) {
					return Standard_False;
				}
				cutMaker.SimplifyResult();
				// A fully consumed subject has no solid result. Keep the operands
				// untouched and make Apply unavailable instead of silently deleting
				// the subject together with its cutter.
				if (!IsValidSolidBooleanResult(cutMaker.Shape())) {
					return Standard_False;
				}

				TemporalBooleanObject state = selection->second;
				state.selectionType = BooleanSelectionType::Subject;
				state.copy = actedIO;
				Handle(AIS_InteractiveObject) result = new AIS_Shape(cutMaker.Shape());
				applyStyle(result, state);
				previews.push_back({actedIO, result, state});
			}

			// Publish no partial preview: every cut must be valid before AIS or
			// selection state is changed.
			for (const PreviewResult& preview : previews) {
				myContext->Remove(preview.original, Standard_False);
				myContext->SetSelected(preview.result, Standard_False);
				showInteractiveByType(preview.result, BooleanSelectionType::Subject);
				_selectionMap.erase(preview.original);
				_selectionMap[preview.result] = preview.state;
			}
			return Standard_True;
		} catch (const Standard_Failure& failure) {
			std::cout << "Boolean subtract preview failure: "
				<< failure.GetMessageString() << std::endl;
			return Standard_False;
		} catch (...) {
			return Standard_False;
		}
	}

	void BooleanOperationController::applyStyle(
		Handle(AIS_InteractiveObject)& object,
		const TemporalBooleanObject& style) {
		Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(object);
		if (shape.IsNull()) {
			return;
		}
		shape->UnsetColor();
		shape->SetMaterial(style.materialName);
		shape->SetColor(style.colorName);
	}

	void BooleanOperationController::persistStyle(
		const TDF_Label& label,
		const TemporalBooleanObject& style) {
		if (label.IsNull()) {
			return;
		}
		myDoc->SaveObjectMaterial(label, style.materialName);
		myDoc->SaveObjectColor(label, style.colorName);
	}


	Handle(AIS_InteractiveObject) BooleanOperationController::ioCopyWithStyle(
		const Handle(AIS_InteractiveObject)& orig,
		const TemporalBooleanObject& style) {

		TopoDS_Shape shape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(orig)->Shape(), orig->LocalTransformation());
		Handle(AIS_InteractiveObject) copyIO = new AIS_Shape(shape);
		applyStyle(copyIO, style);
		return copyIO;
	}

	Standard_Boolean BooleanOperationController::boolUnion(const std::vector<Handle(AIS_InteractiveObject)> &actedIOArray) {
		if (actedIOArray.size() < 2) {
			return Standard_False;
		}
		std::vector<TDF_Label> documentLabels;
		documentLabels.reserve(actedIOArray.size());
		for (const Handle(AIS_InteractiveObject)& actedIO : actedIOArray) {
			auto selection = _selectionMap.find(actedIO);
			Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(actedIO);
			if (selection == _selectionMap.end()
				|| selection->second.documentLabel.IsNull()
				|| shape.IsNull()
				|| shape->Shape().IsNull()) {
				return Standard_False;
			}
			for (const TDF_Label& existingLabel : documentLabels) {
				if (existingLabel.IsEqual(selection->second.documentLabel)) {
					return Standard_False;
				}
			}
			documentLabels.push_back(selection->second.documentLabel);
		}

		TopTools_ListOfShape theLSA;
		Handle(AIS_InteractiveObject) accumulatedIO = actedIOArray.front();
		TopoDS_Shape accumulatedShape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(accumulatedIO)->Shape(), accumulatedIO->LocalTransformation());
		theLSA.Append(accumulatedShape);


		TopTools_ListOfShape theLST;
		for (int i = 1; i < actedIOArray.size(); ++i) {
			TopoDS_Shape actedShape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(actedIOArray[i])->Shape(), actedIOArray[i]->LocalTransformation());
			theLST.Append(actedShape);
		}

		TopoDS_Shape fusedShape;
		try {
			BRepAlgoAPI_Fuse fuseMaker;
			fuseMaker.SetArguments(theLSA);
			fuseMaker.SetTools(theLST);
			fuseMaker.SetFuzzyValue(kFuzzyValue);
			fuseMaker.SetUseOBB(Standard_True);
//			fuseMaker.SetGlue(BOPAlgo_GlueFull);
			fuseMaker.SetCheckInverted(Standard_True);
			fuseMaker.Build();

			if (!fuseMaker.IsDone()) {
				return Standard_False;
			}
			fuseMaker.SimplifyResult();
			fusedShape = fuseMaker.Shape();
			if (!IsValidSolidBooleanResult(fusedShape)) {
				return Standard_False;
			}
		} catch (const Standard_Failure& failure) {
			std::cout << "Boolean union failure: " << failure.GetMessageString() << std::endl;
			return Standard_False;
		} catch (...) {
			return Standard_False;
		}

		const TemporalBooleanObject resultStyle = _selectionMap.find(actedIOArray.front())->second;
		accumulatedIO = new AIS_Shape(fusedShape);
		applyStyle(accumulatedIO, resultStyle);

		auto doc = myDoc->ChangeDocument();
		if (doc.IsNull() || doc->HasOpenCommand()) {
			return Standard_False;
		}

		try {
			doc->NewCommand();
			for (const TDF_Label& label : documentLabels) {
				if (!myDoc->RemoveShape(label)) {
					doc->AbortCommand();
					return Standard_False;
				}
			}

			const TDF_Label resultLabel = myDoc->AddShape(accumulatedIO);
			if (resultLabel.IsNull()) {
				doc->AbortCommand();
				return Standard_False;
			}
			persistStyle(resultLabel, resultStyle);

			if (!doc->CommitCommand()) {
				if (doc->HasOpenCommand()) {
					doc->AbortCommand();
				}
				return Standard_False;
			}
		} catch (const Standard_Failure& failure) {
			std::cout << "Boolean union transaction failure: " << failure.GetMessageString() << std::endl;
			if (doc->HasOpenCommand()) {
				doc->AbortCommand();
			}
			return Standard_False;
		} catch (...) {
			if (doc->HasOpenCommand()) {
				doc->AbortCommand();
			}
			return Standard_False;
		}

		// The document is now authoritative. Only mutate the presentation after
		// its transaction has committed successfully.
		for (const Handle(AIS_InteractiveObject)& actedIO : actedIOArray) {
			myContext->Remove(actedIO, Standard_False);
		}
		showInteractiveByType(accumulatedIO, BooleanSelectionType::Undefined);
		myDoc->NotifyChanges();
		return Standard_True;
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
		if (!_canApply || _selectionMap.empty()) {
			cancel(action);
			return;
		}

		Standard_Boolean didApply = Standard_False;

		if (BooleanAction::BooleanSubtract == action) {
			bool hasActor = false;
			bool hasSubject = false;
			for (const auto& obj : _selectionMap) {
				hasActor = hasActor
					|| obj.second.selectionType == BooleanSelectionType::Actor;
				hasSubject = hasSubject
					|| obj.second.selectionType == BooleanSelectionType::Subject;
			}
			if (!hasActor || !hasSubject
				|| _actorIOArray.empty() || _actedIOArray.empty()) {
				cancel(action);
				return;
			}
			std::vector<TDF_Label> documentLabels;
			documentLabels.reserve(_selectionMap.size());
			for (const auto& obj : _selectionMap) {
				if (obj.second.documentLabel.IsNull()) {
					cancel(action);
					return;
				}
				for (const TDF_Label& existingLabel : documentLabels) {
					if (existingLabel.IsEqual(obj.second.documentLabel)) {
						cancel(action);
						return;
					}
				}
				documentLabels.push_back(obj.second.documentLabel);
			}

			auto doc = myDoc->ChangeDocument();
			if (doc.IsNull() || doc->HasOpenCommand()) {
				cancel(action);
				return;
			}

			try {
				doc->NewCommand();

				// Change only the persistent document while the command is open.
				// Stable labels identify originals even when preview geometry has
				// baked a local transform into a copy.
				for (const TDF_Label& label : documentLabels) {
					if (!myDoc->RemoveShape(label)) {
						doc->AbortCommand();
						cancel(action);
						return;
					}
				}

				for (const auto& obj : _selectionMap) {
					if (BooleanSelectionType::Subject != obj.second.selectionType) {
						continue;
					}
					const TDF_Label resultLabel = myDoc->AddShape(obj.first);
					if (resultLabel.IsNull()) {
						doc->AbortCommand();
						cancel(action);
						return;
					}
					persistStyle(resultLabel, obj.second);
				}

				if (!doc->CommitCommand()) {
					if (doc->HasOpenCommand()) {
						doc->AbortCommand();
					}
					cancel(action);
					return;
				}
			} catch (const Standard_Failure& failure) {
				std::cout << "Boolean subtract transaction failure: " << failure.GetMessageString() << std::endl;
				if (doc->HasOpenCommand()) {
					doc->AbortCommand();
				}
				cancel(action);
				return;
			} catch (...) {
				if (doc->HasOpenCommand()) {
					doc->AbortCommand();
				}
				cancel(action);
				return;
			}

			// Commit succeeded: promote the existing cut preview to the visible
			// result and remove cutters from the presentation.
			for (const auto& obj : _selectionMap) {
				if (BooleanSelectionType::Actor == obj.second.selectionType) {
					myContext->Remove(obj.first, Standard_False);
				} else if (BooleanSelectionType::Subject == obj.second.selectionType) {
					showInteractiveByType(obj.first, BooleanSelectionType::Undefined);
				}
			}
			myDoc->NotifyChanges();
			didApply = Standard_True;

		} else if (BooleanAction::BooleanUnion == action) {
			Standard_Boolean hasCompleteSubjectOrder = Standard_False;
			const std::vector<Handle(AIS_InteractiveObject)> actedIOArray =
				orderedSubjectPresentations(hasCompleteSubjectOrder);
			if (!hasCompleteSubjectOrder || actedIOArray.size() <= 1) {
				cancel(action);
				return;
			}
			didApply = boolUnion(actedIOArray);
		}

		if (!didApply) {
			cancel(action);
			return;
		}

		myContext->ClearSelected(Standard_True);
		clearOperationState();
		myContext->UpdateCurrentViewer();
	}

	void BooleanOperationController::cancel(BooleanAction action) {
		(void)action;
		for (const auto &obj : _selectionMap) {
			myContext->Remove(obj.first, Standard_False);
			showInteractiveByType(obj.second.original, BooleanSelectionType::Undefined);
		}

		myContext->ClearSelected(Standard_True);
		clearOperationState();
		myContext->UpdateCurrentViewer();
	}
}
