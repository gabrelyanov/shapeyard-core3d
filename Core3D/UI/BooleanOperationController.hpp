//
//  BoooleanOperationController.h
//  Core3D
//
//  Created by Vlad on 31.05.2024.
//

#ifndef BooleanOperationController_h
#define BooleanOperationController_h

#include "AIS_InteractiveContext.hxx"
#include "OcctDocument.h"
#include <map>
#include <vector>

namespace core3d {

	enum BooleanAction {
		BooleanSubtract = 0,
		BooleanUnion
	};

	enum BooleanSelectionType {
		Undefined = -1,
		Actor = 0,
		Subject
	};

	struct TemporalBooleanObject {
		Handle(AIS_InteractiveObject) copy;
		Handle(AIS_InteractiveObject) original;
		BooleanSelectionType selectionType;
		TDF_Label documentLabel;
		Graphic3d_NameOfMaterial materialName = Graphic3d_NameOfMaterial_ShinyPlastified;
		Quantity_NameOfColor colorName = Quantity_NOC_GRAY80;
	};

	class BooleanOperationController {
	public:
		BooleanOperationController() = delete;
        BooleanOperationController(Handle(AIS_InteractiveContext), Handle(OcctDocument) doc);
		
		void updateDetectedState(Handle(AIS_InteractiveObject) detected, Handle(SelectMgr_EntityOwner) detectedOwner, Standard_Boolean forceActor, BooleanAction action);
		void apply(BooleanAction action);
		void cancel(BooleanAction action);
		const Standard_Boolean canApply() const { return _canApply; }
		void visualApply(BooleanAction action);
		
	private:
		void showInteractiveByType(const Handle(AIS_InteractiveObject) shape, BooleanSelectionType type);
		Standard_Boolean boolSubtract(const std::vector<Handle(AIS_InteractiveObject)> &actorIOArray,
					  const std::vector<Handle(AIS_InteractiveObject)> &actedIOArray);
		Standard_Boolean boolUnion(const std::vector<Handle(AIS_InteractiveObject)> &actedIOArray);
		void resetCachedSelection();
		void clearOperationState();
		void rememberSubjectSelection(const TDF_Label& label);
		void forgetSubjectSelection(const TDF_Label& label);
		std::vector<Handle(AIS_InteractiveObject)> orderedSubjectPresentations(
			Standard_Boolean& isComplete) const;
		void applyStyle(Handle(AIS_InteractiveObject)& object, const TemporalBooleanObject& style);
		void persistStyle(const TDF_Label& label, const TemporalBooleanObject& style);
		Handle(AIS_InteractiveObject) ioCopyWithStyle(const Handle(AIS_InteractiveObject)& orig,
			const TemporalBooleanObject& style);
		
    private:
        std::vector<Handle(AIS_InteractiveObject)> _actedIOArray;
        std::vector<Handle(AIS_InteractiveObject)> _actorIOArray;
		// AIS presentations are recreated while previewing. Persistent labels keep
		// the semantic first-selected order stable across those pointer changes.
		std::vector<TDF_Label> _subjectSelectionOrder;

	protected:
        Handle(AIS_InteractiveContext)  myContext;
        Handle(OcctDocument) myDoc;
		std::map<Handle(AIS_InteractiveObject), TemporalBooleanObject> _selectionMap; //first - orig, second - copy and type
		Standard_Boolean _canApply;
	};
}

#endif /* BooleanOperationController_h */
