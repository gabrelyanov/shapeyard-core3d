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
		BooleanSelectionType selectionType;
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
		void boolSubtract(const std::vector<Handle(AIS_InteractiveObject)> &actorIOArray,
					  const std::vector<Handle(AIS_InteractiveObject)> &actedIOArray);
		void boolUnion(const std::vector<Handle(AIS_InteractiveObject)> &actedIOArray);
		void resetCachedSelection();
		void copyMaterial(Handle(AIS_InteractiveObject) &to, const Handle(AIS_InteractiveObject) &from);
		Handle(AIS_InteractiveObject) ioCopyWithMaterial(const Handle(AIS_InteractiveObject) &orig);
		
    private:
        std::vector<Handle(AIS_InteractiveObject)> _actedIOArray;
        std::vector<Handle(AIS_InteractiveObject)> _actorIOArray;

	protected:
        Handle(AIS_InteractiveContext)  myContext;
        Handle(OcctDocument) myDoc;
		std::map<Handle(AIS_InteractiveObject), TemporalBooleanObject> _selectionMap; //first - orig, second - copy and type
		Standard_Boolean _canApply;
	};
}

#endif /* BooleanOperationController_h */
