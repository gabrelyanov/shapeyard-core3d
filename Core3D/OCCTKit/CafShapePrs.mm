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
#include "OcctDocument.h"

#import <Foundation/Foundation.h>

#include <Graphic3d_TextureSet.hxx>
#include <Graphic3d_ShaderProgram.hxx>
#include "../UI/Core3DContext.hpp"
#include <Prs3d_ShadingAspect.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <BRep_Tool.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <Prs3d_Drawer.hxx>
#include <Precision.hxx>
#include <StdPrs_ToolTriangulatedShape.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>

namespace {

bool HasAuthoredPBRShader(const Handle(Graphic3d_AspectFillArea3d)& aspect)
{
  return !aspect.IsNull() && !aspect->ShaderProgram().IsNull()
      && aspect->ShaderProgram()->GetId().StartsWith("shapeyard-data-maps-v1-");
}

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
  // Scalar PBR intentionally survives an empty texture set. A complete
  // material replacement must therefore clear our shader explicitly.
  if (HasAuthoredPBRShader(aShading->Aspect()))
  {
    aShading->Aspect()->SetShaderProgram({});
  }
  aShading->Aspect()->SetTextureMapOff();
  aShading->Aspect()->SetTextureSet(
      Handle(Graphic3d_TextureSet)());
  Core3DPrepareRendererTextures(aShading->Aspect());
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
  Core3DPrepareRendererTextures(
      aDestinationAspect, HasAuthoredPBRShader(aSourceAspect));
}

Handle(Graphic3d_TextureSet) TextureSetReplacingBaseColor(
    const Handle(Prs3d_Drawer)& theSource,
    const Handle(Graphic3d_TextureMap)& theBaseColor)
{
  if (theBaseColor.IsNull() || theBaseColor->GetParams().IsNull())
  {
    return {};
  }
  theBaseColor->GetParams()->SetTextureUnit(Graphic3d_TextureUnit_BaseColor);
  Handle(Graphic3d_TextureSet) aSourceSet;
  if (!theSource.IsNull() && !theSource->ShadingAspect().IsNull()
      && !theSource->ShadingAspect()->Aspect().IsNull())
  {
    aSourceSet = theSource->ShadingAspect()->Aspect()->TextureSet();
  }
  Standard_Integer aBaseIndex = -1;
  const Standard_Integer aSourceSize = aSourceSet.IsNull() ? 0 : aSourceSet->Size();
  for (Standard_Integer anIndex = 0; anIndex < aSourceSize; ++anIndex)
  {
    const Handle(Graphic3d_TextureMap)& aTexture = aSourceSet->Value(anIndex);
    if (!aTexture.IsNull() && !aTexture->GetParams().IsNull()
        && aTexture->GetParams()->TextureUnit() == Graphic3d_TextureUnit_BaseColor)
    {
      if (aBaseIndex >= 0) return {};
      aBaseIndex = anIndex;
    }
  }
  Handle(Graphic3d_TextureSet) aResult = new Graphic3d_TextureSet(
      aSourceSize + (aBaseIndex < 0 ? 1 : 0));
  Standard_Integer aDestination = 0;
  bool aBaseWritten = false;
  for (Standard_Integer anIndex = 0; anIndex < aSourceSize; ++anIndex)
  {
    const Handle(Graphic3d_TextureMap)& aTexture = aSourceSet->Value(anIndex);
    const bool isBase = anIndex == aBaseIndex;
    aResult->SetValue(aDestination++, isBase ? theBaseColor : aTexture);
    aBaseWritten = aBaseWritten || isBase;
  }
  if (!aBaseWritten) aResult->SetValue(aDestination, theBaseColor);
  return aResult;
}

// A face whose mesher could not reach the display accuracy keeps a mesh
// coarser than asked (BRepMesh records the reached deflection). In a session
// OCCT still reuses it, because the mesh carries the settings it was made
// with (Poly_TriangulationParameters); a mesh read from a saved file carries
// no settings, so the first display after opening re-meshes that face: its
// Checked flag is cleared and its edge meshes are renumbered, which changes a
// saved file just by opening it (lamp shade face 6, 19 Nov).
// A face is kept only when its stored mesh was made by a display pass for
// this face geometry with the asked deflection (CTO 19 Nov 03:25, option A):
// every edge of the face carries a mesh polygon on that very triangulation
// (new or changed geometry has none), and a face coarser than asked carries
// edge polygons made with exactly the asked deflection (so the mesher already
// tried this accuracy and stopped where it stopped). The angle is checked by
// the caller. All other shapes are left to OCCT as before.
bool ShapeMeshedForDisplayDeflection(const TopoDS_Shape& theShape,
                                     const Standard_Real theDeflection)
{
  if (theShape.IsNull() || !(theDeflection > 0.0)) return false;
  const Standard_Real aSlack = theDeflection * 1.0e-9 + 1.0e-12;
  const Standard_Real aLimit = theDeflection + aSlack;
  // Free edges and vertices are drawn from their own polygons: leave those to OCCT.
  if (TopExp_Explorer(theShape, TopAbs_EDGE, TopAbs_FACE).More()) return false;
  TopTools_IndexedMapOfShape aFaces;
  TopExp::MapShapes(theShape, TopAbs_FACE, aFaces);
  if (aFaces.IsEmpty()) return false;
  for (Standard_Integer anIndex = 1; anIndex <= aFaces.Extent(); ++anIndex)
  {
    const TopoDS_Face& aFace = TopoDS::Face(aFaces(anIndex));
    TopLoc_Location aLocation;
    const Handle(Poly_Triangulation)& aMesh = BRep_Tool::Triangulation(aFace, aLocation);
    if (aMesh.IsNull() || aMesh->NbTriangles() < 1) return false;
    const bool isFineEnough = aMesh->Deflection() <= aLimit;
    Standard_Integer anEdgeCount = 0;
    for (TopExp_Explorer anEdge(aFace, TopAbs_EDGE); anEdge.More(); anEdge.Next())
    {
      ++anEdgeCount;
      const Handle(Poly_PolygonOnTriangulation)& aPolygon =
          BRep_Tool::PolygonOnTriangulation(TopoDS::Edge(anEdge.Current()), aMesh, aLocation);
      if (aPolygon.IsNull()) return false;
      if (!isFineEnough && !(Abs(aPolygon->Deflection() - theDeflection) <= aSlack)) return false;
    }
    if (!isFineEnough && anEdgeCount == 0) return false;
  }
  return true;
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
// function : Compute
// purpose  : Do not re-mesh a shape that is already meshed for display.
// =======================================================================
void CafShapePrs::Compute(const Handle(PrsMgr_PresentationManager)& thePrsMgr,
                          const Handle(Prs3d_Presentation)&         thePrs,
                          const Standard_Integer                    theMode)
{
  const Handle(Prs3d_Drawer)& aDrawer = Attributes();
  TopoDS_Shape aShape;
  if (!GetLabel().IsNull()) aShape = XCAFDoc_ShapeTool::GetShape(GetLabel());
  if (aShape.IsNull()) aShape = Shape();
  // Every display pass in the app meshes with OCCT's default angle; a drawer
  // asking another angle, or whose own deflection settings just changed (OCCT
  // then clears and re-meshes), always goes the normal way.
  static const Standard_Real THE_DISPLAY_ANGLE = Handle(Prs3d_Drawer)(new Prs3d_Drawer())->DeviationAngle();
  bool keepMesh = false;
  if (!aDrawer.IsNull() && aDrawer->IsAutoTriangulation() && !aShape.IsNull()
      && Abs(aDrawer->DeviationAngle() - THE_DISPLAY_ANGLE) <= Precision::Angular()
      && (!aDrawer->HasOwnDeviationAngle()
          || Abs(aDrawer->DeviationAngle() - aDrawer->PreviousDeviationAngle()) <= Precision::Angular())
      && (!aDrawer->HasOwnDeviationCoefficient()
          || Abs(aDrawer->DeviationCoefficient() - aDrawer->PreviousDeviationCoefficient()) <= Precision::Confusion())
      && !StdPrs_ToolTriangulatedShape::IsTessellated(aShape, aDrawer))
  {
    keepMesh = ShapeMeshedForDisplayDeflection(
        aShape, StdPrs_ToolTriangulatedShape::GetDeflection(aShape, aDrawer));
  }
  if (!keepMesh)
  {
    XCAFPrs_AISObject::Compute(thePrsMgr, thePrs, theMode);
    return;
  }
  const Standard_Boolean hadOwn = aDrawer->HasOwnIsAutoTriangulation();
  const Standard_Boolean ownValue = aDrawer->IsAutoTriangulation();
  struct RestoreAutoTriangulation {
    Handle(Prs3d_Drawer) drawer; Standard_Boolean hadOwn; Standard_Boolean value;
    ~RestoreAutoTriangulation()
    {
      if (hadOwn) drawer->SetAutoTriangulation(value);
      else drawer->UnsetOwnIsAutoTriangulation();
    }
  } restore{aDrawer, hadOwn, ownValue};
  aDrawer->SetAutoTriangulation(Standard_False);
  XCAFPrs_AISObject::Compute(thePrsMgr, thePrs, theMode);
}

// =======================================================================
// function : DispatchStyles
// purpose  : Preserve resolved occurrence precedence over root definition.
// =======================================================================
void CafShapePrs::DispatchStyles(const Standard_Boolean theToSyncStyles)
{
  // Style grouping can place the persisted root into a temporary compound,
  // clearing Free even when no geometry edit occurs. Dispatch can also be
  // requested outside Compute/redrawDocument, so preserve at this boundary.
  struct RootFreeFlag {
    TopoDS_Shape shape;
    Standard_Boolean wasFree;
    ~RootFreeFlag() { if (!shape.IsNull()) shape.Free(wasFree); }
  };
  const TopoDS_Shape root = GetLabel().IsNull()
      ? TopoDS_Shape() : XCAFDoc_ShapeTool::GetShape(GetLabel());
  const RootFreeFlag restore{root, !root.IsNull() && root.Free()};
  XCAFPrs_AISObject::DispatchStyles(theToSyncStyles);
  // The parent recreates wrappers for default and imported sub-shape styles.
  // Normalize every own aspect after dispatch, before Compute consumes it.
  Core3DContext::DisableSurfaceIsolines(Attributes());
  if (!Attributes()->ShadingAspect().IsNull())
  {
    Core3DPrepareRendererTextures(Attributes()->ShadingAspect()->Aspect(), myUseScalarPBR);
  }
  for (AIS_DataMapOfShapeDrawer::Iterator anOverride(myShapeColors);
       anOverride.More(); anOverride.Next())
  {
    const Handle(AIS_ColoredDrawer)& aDrawer = anOverride.Value();
    Core3DContext::DisableSurfaceIsolines(aDrawer);
    if (!aDrawer.IsNull() && aDrawer->HasOwnShadingAspect()
        && !aDrawer->ShadingAspect().IsNull())
    {
      Core3DPrepareRendererTextures(aDrawer->ShadingAspect()->Aspect(), myUseScalarPBR);
    }
  }
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
  myUseScalarPBR = theMaterial->HasPbrMaterial();
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
  Core3DPrepareRendererTextures(aRootAspect, myUseScalarPBR);
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
    myUseScalarPBR = Standard_False;
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
// function : ApplyEphemeralDecalBaseColor
// purpose  : Per-face E4 fallback without changing XCAF authority.
// =======================================================================
Standard_Boolean CafShapePrs::ApplyEphemeralDecalBaseColor(
    const TopoDS_Shape& theFace,
    const Handle(Graphic3d_TextureMap)& theTexture)
{
  if (theFace.IsNull() || theTexture.IsNull()) return Standard_False;
  const Handle(AIS_ColoredDrawer) aDrawer = CustomAspects(theFace);
  if (aDrawer.IsNull()) return Standard_False;
  aDrawer->SetupOwnShadingAspect();
  if (aDrawer->ShadingAspect().IsNull()
      || aDrawer->ShadingAspect()->Aspect().IsNull()) return Standard_False;
  const Handle(Prs3d_Drawer) aSource = aDrawer;
  const Handle(Graphic3d_TextureSet) aTextures =
      TextureSetReplacingBaseColor(aSource, theTexture);
  if (aTextures.IsNull() || aTextures->IsEmpty()) return Standard_False;
  aDrawer->ShadingAspect()->Aspect()->SetTextureSet(aTextures);
  aDrawer->ShadingAspect()->Aspect()->SetTextureMapOn();
  // The shared E4 artifact already contains the resolved base factor. Keep
  // data-map material properties but make the per-face color multiplier unity
  // so fallback, Metal and exported GLB apply that factor exactly once.
  aDrawer->ShadingAspect()->SetColor(Quantity_Color(Quantity_NOC_WHITE));
  aDrawer->ShadingAspect()->SetTransparency(0.0);
  Core3DPrepareRendererTextures(aDrawer->ShadingAspect()->Aspect());
  SynchronizeAspects();
  return Standard_True;
}

// =======================================================================
// function : ClearEphemeralDecalAppearance
// purpose  : Restore authoritative presentation styles.
// =======================================================================
void CafShapePrs::ClearEphemeralDecalAppearance()
{
  DispatchStyles(Standard_True);
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
