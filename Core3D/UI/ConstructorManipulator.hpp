//
//  ConstructorManipulator.hpp
//  Core3D
//
//  Created by Vlad Mar on 10.09.2024.
//

#ifndef ConstructorManipulator_hpp
#define ConstructorManipulator_hpp

#include <stdio.h>
#include "Core3DManipulator.hpp"

NCOLLECTION_HSEQUENCE(ConstructorManipulatorObjectSequence, Handle(AIS_InteractiveObject))

DEFINE_STANDARD_HANDLE (ConstructorManipulator, AIS_InteractiveObject)

class ConstructorManipulator : public Core3DManipulator {
	
public:
	Standard_EXPORT ConstructorManipulator();
	~ConstructorManipulator() {}
	
	void SetAxisIndex(Standard_Integer index);
	Standard_EXPORT void SetPart (const Standard_Integer theAxisIndex, const AIS_ManipulatorMode theMode, const Standard_Boolean theIsEnabled) override;
	
protected:
	void Compute (const Handle(PrsMgr_PresentationManager)& thePrsMgr,
				  const Handle(Prs3d_Presentation)& thePrs,
				  const Standard_Integer theMode) override;
	
	private:
		Standard_Integer myAxisIndex;
};

#endif /* ConstructorManipulator_hpp */
