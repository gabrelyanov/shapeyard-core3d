// Copyright (c) 2017 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of Open CASCADE
// commercial license or contractual agreement.

#include "CafShapePrs.h"

#import <Foundation/Foundation.h>

IMPLEMENT_STANDARD_RTTIEXT(CafShapePrs, XCAFPrs_AISObject)

// =======================================================================
// function : CafShapePrs
// purpose  :
// =======================================================================
CafShapePrs::CafShapePrs(const TDF_Label&                theLabel,
                         const XCAFPrs_Style&            theStyle,
                         const Graphic3d_MaterialAspect& theMaterial)
: CafShapePrs(theLabel, theLabel, theStyle, theMaterial)
{
}

// =======================================================================
// function : CafShapePrs
// purpose  : Keep definition styling and occurrence edit identity separate.
// =======================================================================
CafShapePrs::CafShapePrs(const TDF_Label&                theDefinitionLabel,
                         const TDF_Label&                theOccurrenceLabel,
                         const XCAFPrs_Style&            theStyle,
                         const Graphic3d_MaterialAspect& theMaterial)
: XCAFPrs_AISObject (theDefinitionLabel),
  myDefStyle(theStyle),
  myOccurrenceLabel(theOccurrenceLabel)
{
  SetMaterial(theMaterial);
}

// =======================================================================
// function : DispatchStyles
// purpose  : Preserve resolved occurrence precedence over root definition.
// =======================================================================
void CafShapePrs::DispatchStyles(const Standard_Boolean theToSyncStyles)
{
  XCAFPrs_AISObject::DispatchStyles(theToSyncStyles);
  if (!IsEditablePresentation() && !Shape().IsNull())
  {
    // CollectStyleSettings(definition) repeats the definition's whole-object
    // material/color as a custom drawer. For an occurrence that lower-priority
    // drawer would mask myDefStyle (which already merges the occurrence style).
    // Remove only the root drawer; explicit face/sub-shape styles remain.
    removeDefinitionRootCustomAspects();
  }
}

// =======================================================================
// function : ApplyAuthoredVisualMaterial
// purpose  : Make an app-owned PBR material authoritative in XCAFPrs.
// =======================================================================
void CafShapePrs::ApplyAuthoredVisualMaterial(
    const Handle(XCAFDoc_VisMaterial)& theMaterial)
{
  if (theMaterial.IsNull())
  {
    return;
  }

  const Quantity_ColorRGBA aBaseColor = theMaterial->BaseColor();
  myDefStyle.SetMaterial(theMaterial);
  myDefStyle.SetColorSurf(aBaseColor);
  myDefStyle.SetColorCurv(aBaseColor.GetRGB());
  clearImportedSubshapeAppearanceOverrides(
      Standard_True, Standard_True);
  removeDefinitionRootCustomAspects();

  Graphic3d_MaterialAspect anAspect;
  theMaterial->FillMaterialAspect(anAspect);
  SetMaterial(anAspect);
  SetColor(aBaseColor.GetRGB());
}

// =======================================================================
// function : ApplyAuthoredLegacyAppearance
// purpose  : Supersede captured imported style with preset/color authoring.
// =======================================================================
void CafShapePrs::ApplyAuthoredLegacyAppearance(
    const Standard_Boolean          theHasMaterial,
    const Graphic3d_MaterialAspect& theMaterial,
    const Standard_Boolean          theHasColor,
    const Quantity_Color&           theColor)
{
  if (!theHasMaterial && !theHasColor)
  {
    return;
  }

  if (theHasMaterial)
  {
    // The preset lives in the AIS drawer rather than the XCAF material table.
    // Clear any lower-priority imported material retained by the explorer.
    myDefStyle.SetMaterial(Handle(XCAFDoc_VisMaterial)());
    SetMaterial(theMaterial);
  }
  if (theHasColor)
  {
    myDefStyle.SetColorSurf(theColor);
    myDefStyle.SetColorCurv(theColor);
    SetColor(theColor);
  }
  clearImportedSubshapeAppearanceOverrides(
      theHasMaterial, theHasColor);
  removeDefinitionRootCustomAspects();
}

// =======================================================================
// function : clearImportedSubshapeAppearanceOverrides
// purpose  : Whole-object authoring follows snapshot precedence over faces.
// =======================================================================
void CafShapePrs::clearImportedSubshapeAppearanceOverrides(
    const Standard_Boolean theClearMaterial,
    const Standard_Boolean theClearColor)
{
  for (AIS_DataMapOfShapeDrawer::Iterator anOverride(myShapeColors);
       anOverride.More(); anOverride.Next())
  {
    const Handle(AIS_ColoredDrawer)& aDrawer = anOverride.Value();
    if (aDrawer.IsNull())
    {
      continue;
    }
    if (theClearMaterial)
    {
      aDrawer->UnsetOwnMaterial();
    }
    if (theClearColor)
    {
      aDrawer->UnsetOwnColor();
    }
    // Visibility, transparency and line width are not appearance values
    // authored by the material editor and deliberately remain inherited from
    // the XDE face style.
  }
}

// =======================================================================
// function : removeDefinitionRootCustomAspects
// purpose  : Preserve face/subshape drawers while replacing whole-object XDE.
// =======================================================================
void CafShapePrs::removeDefinitionRootCustomAspects()
{
  if (!Shape().IsNull())
  {
    UnsetCustomAspects(Shape(), Standard_True);
  }
}
