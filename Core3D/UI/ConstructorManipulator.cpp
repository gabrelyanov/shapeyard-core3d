//
//  ConstructorManipulator.cpp
//  Core3D
//
//  Created by Vlad Mar on 10.09.2024.
//

#include "ConstructorManipulator.hpp"
#include <AIS_DisplayMode.hxx>


ConstructorManipulator::ConstructorManipulator() : myAxisIndex(1) {
	Core3DManipulator::SetPart (AIS_ManipulatorMode::AIS_MM_Scaling, Standard_False);
	Core3DManipulator::SetPart (AIS_ManipulatorMode::AIS_MM_Translation, Standard_True);
	Core3DManipulator::SetPart (AIS_ManipulatorMode::AIS_MM_Rotation, Standard_False);
	Core3DManipulator::SetPart (AIS_ManipulatorMode::AIS_MM_TranslationPlane, Standard_False);
	Core3DManipulator::SetPart (AIS_ManipulatorMode::AIS_MM_MirroringPlanePos, Standard_False);
	Core3DManipulator::SetPart (AIS_ManipulatorMode::AIS_MM_MirroringPlaneNeg, Standard_False);
}

void ConstructorManipulator::Compute (const Handle(PrsMgr_PresentationManager)& thePrsMgr,
									  const Handle(Prs3d_Presentation)& thePrs,
									  const Standard_Integer theMode)
{
	if (theMode != AIS_Shaded)
	{
		return;
	}
	
	thePrs->SetInfiniteState (Standard_True);
	thePrs->SetMutable (Standard_True);
	Handle(Graphic3d_Group) aGroup;
	Handle(Prs3d_ShadingAspect) anAspect = new Prs3d_ShadingAspect();
	anAspect->Aspect()->SetInteriorStyle (Aspect_IS_SOLID);
	anAspect->SetMaterial (myDrawer->ShadingAspect()->Material());
	anAspect->SetTransparency (myDrawer->ShadingAspect()->Transparency());
	
	// Display center
	if (myHasCenter) {
		myCenter.Init (myAxes[0].AxisRadius() * 1.5f, gp::Origin());
		aGroup = thePrs->NewGroup ();
		aGroup->SetPrimitivesAspect (myDrawer->ShadingAspect()->Aspect());
		aGroup->AddPrimitiveArray (myCenter.Array());
	}
	
	for (Standard_Integer anIt = 0; anIt < 3; ++anIt)
	{
		// Display axes
		aGroup = thePrs->NewGroup ();
		
		Handle(Prs3d_ShadingAspect) anAspectAx = new Prs3d_ShadingAspect (new Graphic3d_AspectFillArea3d(*anAspect->Aspect()));
		anAspectAx->SetColor (myAxes[anIt].Color());
		aGroup->SetGroupPrimitivesAspect (anAspectAx->Aspect());
		myAxes[anIt].Compute (thePrsMgr, thePrs, anAspectAx, true);
		myAxes[anIt].SetTransformPersistence (TransformPersistence());
	}
	
	updateTransformation();
}

void ConstructorManipulator::SetPart (const Standard_Integer theAxisIndex, const AIS_ManipulatorMode theMode, const Standard_Boolean theIsEnabled)
{
	Standard_ProgramError_Raise_if (theAxisIndex < 0 || theAxisIndex > 2, "ConstructorManipulator::SetMode(): axis index should be between 0 and 2");
	myHasCenter = false;
	myObjectOrientation = false;
	
	switch (theMode)
	{
		case AIS_MM_Translation:
			if (theAxisIndex == myAxisIndex)
				myAxes[theAxisIndex].SetTranslation (theIsEnabled);
			else
				myAxes[theAxisIndex].SetTranslation (Standard_False);
			break;
			
		case AIS_MM_Rotation:
			myAxes[theAxisIndex].SetRotation (Standard_False);
			break;
			
		case AIS_MM_Scaling:
			myAxes[theAxisIndex].SetScaling (Standard_False);
			break;
			
		case AIS_MM_ScalingUniform:
			myAxes[theAxisIndex].SetScalingUniform (Standard_False);
			break;
			
		case AIS_MM_TranslationPlane:
			myAxes[theAxisIndex].SetDragging(Standard_False);
			break;
			
		case AIS_MM_MirroringPlanePos:
			myAxes[theAxisIndex].SetMirroringPos(Standard_False);
			break;
			
		case AIS_MM_MirroringPlaneNeg:
			myAxes[theAxisIndex].SetMirroringNeg(Standard_False);
			break;
			
		default:
			break;
	}
}

void ConstructorManipulator::SetAxisIndex(Standard_Integer index) {
	myAxisIndex = index;
	Core3DManipulator::SetPart (AIS_ManipulatorMode::AIS_MM_Translation, Standard_True);
}
