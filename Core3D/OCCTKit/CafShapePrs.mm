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

#include <Graphic3d_TextureSet.hxx>
#include <Prs3d_ShadingAspect.hxx>

namespace {

Handle(Graphic3d_AspectFillArea3d) ClearDrawerTextureMapping(
    const Handle(Prs3d_Drawer)& theDrawer)
{
  if (theDrawer.IsNull())
  {
    return {};
  }
  theDrawer->SetupOwnShadingAspect();
  const Handle(Prs3d_ShadingAspect)& aShading =
      theDrawer->ShadingAspect();
  if (aShading.IsNull() || aShading->Aspect().IsNull())
  {
    return {};
  }
  aShading->Aspect()->SetTextureMapOff();
  aShading->Aspect()->SetTextureSet(
      Handle(Graphic3d_TextureSet)());
  return aShading->Aspect();
}

void ResetDrawerForLegacyMaterial(
    const Handle(Prs3d_Drawer)& theDrawer)
{
  const Handle(Graphic3d_AspectFillArea3d) anAspect =
      ClearDrawerTextureMapping(theDrawer);
  if (anAspect.IsNull())
  {
    return;
  }
  // These are the documented Graphic3d/XCAF legacy defaults. BlendAuto
  // follows preset transparency and Auto follows the closed-group flag,
  // matching the Metal snapshot's preset resolution.
  anAspect->SetAlphaMode(Graphic3d_AlphaMode_BlendAuto, 0.5f);
  anAspect->SetFaceCulling(
      Graphic3d_TypeOfBackfacingModel_Auto);
}

void CopyDrawerTextureMapping(
    const Handle(Prs3d_Drawer)& theSource,
    const Handle(Prs3d_Drawer)& theDestination)
{
  if (theSource.IsNull() || theDestination.IsNull()
      || !theDestination->HasOwnShadingAspect()
      || theSource->ShadingAspect().IsNull()
      || theSource->ShadingAspect()->Aspect().IsNull())
  {
    return;
  }
  const Handle(Graphic3d_AspectFillArea3d)& aSourceAspect =
      theSource->ShadingAspect()->Aspect();
  const Handle(Graphic3d_AspectFillArea3d) aDestinationAspect =
      ClearDrawerTextureMapping(theDestination);
  if (aDestinationAspect.IsNull())
  {
    return;
  }
  if (aSourceAspect->ToMapTexture()
      && !aSourceAspect->TextureSet().IsNull()
      && !aSourceAspect->TextureSet()->IsEmpty())
  {
    // Custom XCAF drawers remain separate so visibility, transparency, and
    // line width survive whole-object authoring. Share only the immutable
    // renderer texture set produced for the authoritative root material.
    aDestinationAspect->SetTextureSet(aSourceAspect->TextureSet());
    aDestinationAspect->SetTextureMapOn();
  }
}

} // namespace

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

  // XCAFDoc_VisMaterial::FillAspect() leaves an old texture untouched when
  // the replacement has no maps. Reset first, then apply the complete native
  // aspect so assign, replace, and clear share one renderer path.
  const Handle(Graphic3d_AspectFillArea3d) aRootAspect =
      ClearDrawerTextureMapping(Attributes());
  if (aRootAspect.IsNull())
  {
    SynchronizeAspects();
    return;
  }
  theMaterial->FillAspect(aRootAspect);
  for (AIS_DataMapOfShapeDrawer::Iterator anOverride(myShapeColors);
       anOverride.More(); anOverride.Next())
  {
    CopyDrawerTextureMapping(
        Attributes(), anOverride.Value());
  }
  SynchronizeAspects();
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

  // Clear presentation-only imported overrides before SetMaterial/SetColor so
  // XCAFPrs propagates the authoritative values into every retained drawer.
  clearImportedSubshapeAppearanceOverrides(
      theHasMaterial, theHasColor);
  removeDefinitionRootCustomAspects();
  if (theHasMaterial)
  {
    ResetDrawerForLegacyMaterial(Attributes());
    for (AIS_DataMapOfShapeDrawer::Iterator anOverride(myShapeColors);
         anOverride.More(); anOverride.Next())
    {
      const Handle(AIS_ColoredDrawer)& aDrawer = anOverride.Value();
      if (!aDrawer.IsNull() && aDrawer->HasOwnShadingAspect())
      {
        // Keep visibility, transparency, line width, and the drawer itself;
        // only renderer material state is superseded by the legacy preset.
        ResetDrawerForLegacyMaterial(aDrawer);
      }
    }
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
  SynchronizeAspects();
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
      if (aDrawer->HasOwnShadingAspect())
      {
        ClearDrawerTextureMapping(aDrawer);
      }
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
