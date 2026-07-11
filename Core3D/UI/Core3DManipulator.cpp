#include "Core3DManipulator.hpp"
#include "../Scene/SceneSnapshot.hpp"

#include <AIS_DisplayMode.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_ManipulatorOwner.hxx>
#include <Extrema_ExtElC.hxx>
#include <gce_MakeDir.hxx>
#include <IntAna_IntConicQuad.hxx>
#include <Prs3d_Arrow.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <Prs3d_ToolDisk.hxx>
#include <Prs3d_ToolSector.hxx>
#include <Prs3d_ToolSphere.hxx>
#include <Select3D_SensitiveCircle.hxx>
#include <Select3D_SensitivePoint.hxx>
#include <Select3D_SensitiveSegment.hxx>
#include <Select3D_SensitiveTriangulation.hxx>
#include <Select3D_SensitivePrimitiveArray.hxx>
#include <SelectMgr_SequenceOfOwner.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <V3d_View.hxx>
#include <AIS_Shape.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include "Core3DContext.hpp"
#include <TopExp_Explorer.hxx>
#include <ShapeUpgrade_RemoveLocations.hxx>
#include <gp_Quaternion.hxx>
#include <Standard_Failure.hxx>
#include "Snapping.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

IMPLEMENT_STANDARD_RTTIEXT(Core3DManipulator, AIS_InteractiveObject)
IMPLEMENT_STANDARD_HANDLE (Core3DManipulator, AIS_InteractiveObject)

IMPLEMENT_HSEQUENCE(Core3DManipulatorObjectSequence)

namespace
{
//! Return Ax1 for specified direction of Ax2.
static gp_Ax1 getAx1FromAx2Dir (const gp_Ax2& theAx2,
                                int theIndex)
{
    switch (theIndex)
    {
        case 0: return gp_Ax1 (theAx2.Location(), theAx2.XDirection());
        case 1: return gp_Ax1 (theAx2.Location(), theAx2.YDirection());
        case 2: return theAx2.Axis();
    }
    throw Standard_ProgramError ("AIS_Manipulator - Invalid axis index");
}

//! Auxiliary tool for filtering picking ray.
class ManipSensRotation
{
public:
    //! Main constructor.
    ManipSensRotation (const gp_Dir& thePlaneNormal) : myPlaneNormal (thePlaneNormal), myAngleTol (10.0 * M_PI / 180.0) {}
    
    //! Checks if picking ray can be used for detection.
    Standard_Boolean isValidRay (const SelectBasics_SelectingVolumeManager& theMgr) const
    {
        if (theMgr.GetActiveSelectionType() != SelectMgr_SelectionType_Point)
        {
            return Standard_False;
        }
        
        const gp_Dir aRay = theMgr.GetViewRayDirection();
        return !aRay.IsNormal (myPlaneNormal, myAngleTol);
    }
private:
    gp_Dir        myPlaneNormal;
    Standard_Real myAngleTol;
};

//! Sensitive circle with filtering picking ray.
class ManipSensCircle : public Select3D_SensitiveCircle, public ManipSensRotation
{
public:
    //! Main constructor.
    ManipSensCircle (const Handle(SelectMgr_EntityOwner)& theOwnerId,
                     const gp_Circ& theCircle)
    : Select3D_SensitiveCircle (theOwnerId, theCircle, Standard_False),
    ManipSensRotation (theCircle.Position().Direction()) {}
    
    //! Checks whether the circle overlaps current selecting volume
    virtual Standard_Boolean Matches (SelectBasics_SelectingVolumeManager& theMgr,
                                      SelectBasics_PickResult& thePickResult) Standard_OVERRIDE
    {
        return isValidRay (theMgr)
        && Select3D_SensitiveCircle::Matches (theMgr, thePickResult);
    }
};

//! Sensitive triangulation with filtering picking ray.
class ManipSensTriangulation : public Select3D_SensitiveTriangulation, public ManipSensRotation
{
public:
    ManipSensTriangulation (const Handle(SelectMgr_EntityOwner)& theOwnerId,
                            const Handle(Poly_Triangulation)& theTrg,
                            const gp_Dir& thePlaneNormal)
    : Select3D_SensitiveTriangulation (theOwnerId, theTrg, TopLoc_Location(), Standard_True),
    ManipSensRotation (thePlaneNormal) {}
    
    //! Checks whether the circle overlaps current selecting volume
    virtual Standard_Boolean Matches (SelectBasics_SelectingVolumeManager& theMgr,
                                      SelectBasics_PickResult& thePickResult) Standard_OVERRIDE
    {
        return isValidRay (theMgr)
        && Select3D_SensitiveTriangulation::Matches (theMgr, thePickResult);
    }
};

constexpr Standard_Integer kMaximumOverlayVerticesPerComponent = 100'000;
constexpr Standard_Integer kMaximumOverlayIndicesPerComponent = 300'000;

bool CopyTriangleArray(
    const Handle(Graphic3d_ArrayOfTriangles)& theArray,
    const std::string& theDefinitionIdentifier,
    core3d::scene::MeshSnapshot& theMesh)
{
    if (theArray.IsNull() || !theArray->HasVertexNormals()) {
        return false;
    }
    const Standard_Integer aVertexCount = theArray->VertexNumber();
    const Standard_Integer anEdgeCount = theArray->EdgeNumber();
    if (aVertexCount <= 0
        || aVertexCount > kMaximumOverlayVerticesPerComponent
        || anEdgeCount == 0 || anEdgeCount < -1
        || anEdgeCount > kMaximumOverlayIndicesPerComponent) {
        return false;
    }
    const Standard_Integer anIndexCount = anEdgeCount > 0
        ? anEdgeCount
        : aVertexCount;
    if (anIndexCount <= 0 || anIndexCount % 3 != 0
        || static_cast<std::uint64_t>(aVertexCount)
            > std::numeric_limits<std::uint32_t>::max()
        || static_cast<std::uint64_t>(anIndexCount)
            > std::numeric_limits<std::uint32_t>::max()) {
        return false;
    }

    core3d::scene::MeshSnapshot aMesh;
    aMesh.definitionIdentifier = theDefinitionIdentifier;
    aMesh.vertices.reserve(static_cast<std::size_t>(aVertexCount));
    aMesh.indices.reserve(static_cast<std::size_t>(anIndexCount));
    for (Standard_Integer aRank = 1; aRank <= aVertexCount; ++aRank) {
        const gp_Pnt aPoint = theArray->Vertice(aRank);
        const gp_Dir aNormal = theArray->VertexNormal(aRank);
        core3d::scene::Vertex aVertex;
        aVertex.positionX = static_cast<float>(aPoint.X());
        aVertex.positionY = static_cast<float>(aPoint.Y());
        aVertex.positionZ = static_cast<float>(aPoint.Z());
        aVertex.normalX = static_cast<float>(aNormal.X());
        aVertex.normalY = static_cast<float>(aNormal.Y());
        aVertex.normalZ = static_cast<float>(aNormal.Z());
        if (!std::isfinite(aVertex.positionX)
            || !std::isfinite(aVertex.positionY)
            || !std::isfinite(aVertex.positionZ)
            || !std::isfinite(aVertex.normalX)
            || !std::isfinite(aVertex.normalY)
            || !std::isfinite(aVertex.normalZ)) {
            return false;
        }
        aMesh.vertices.push_back(aVertex);
        if (!aMesh.localBounds.valid) {
            aMesh.localBounds.minimum = {aPoint.X(), aPoint.Y(), aPoint.Z()};
            aMesh.localBounds.maximum = aMesh.localBounds.minimum;
            aMesh.localBounds.valid = true;
        } else {
            aMesh.localBounds.minimum.x = Min(
                aMesh.localBounds.minimum.x, aPoint.X());
            aMesh.localBounds.minimum.y = Min(
                aMesh.localBounds.minimum.y, aPoint.Y());
            aMesh.localBounds.minimum.z = Min(
                aMesh.localBounds.minimum.z, aPoint.Z());
            aMesh.localBounds.maximum.x = Max(
                aMesh.localBounds.maximum.x, aPoint.X());
            aMesh.localBounds.maximum.y = Max(
                aMesh.localBounds.maximum.y, aPoint.Y());
            aMesh.localBounds.maximum.z = Max(
                aMesh.localBounds.maximum.z, aPoint.Z());
        }
    }
    if (anEdgeCount > 0) {
        for (Standard_Integer aRank = 1; aRank <= anEdgeCount; ++aRank) {
            const Standard_Integer anIndex = theArray->Edge(aRank) - 1;
            if (anIndex < 0 || anIndex >= aVertexCount) {
                return false;
            }
            aMesh.indices.push_back(static_cast<std::uint32_t>(anIndex));
        }
    } else {
        for (Standard_Integer anIndex = 0;
             anIndex < aVertexCount; ++anIndex) {
            aMesh.indices.push_back(static_cast<std::uint32_t>(anIndex));
        }
    }
    aMesh.primitives.push_back({
        0,
        static_cast<std::uint32_t>(aMesh.indices.size()),
        0,
    });
    theMesh = std::move(aMesh);
    return true;
}

core3d::scene::MaterialSnapshot GizmoMaterial(
    const std::string& theIdentifier,
    const Quantity_Color& theColor)
{
    core3d::scene::MaterialSnapshot aMaterial;
    aMaterial.identifier = theIdentifier;
    aMaterial.baseColor = {
        static_cast<float>(theColor.Red()),
        static_cast<float>(theColor.Green()),
        static_cast<float>(theColor.Blue()),
        1.0f,
    };
    aMaterial.metallic = 0.0f;
    aMaterial.roughness = 1.0f;
    aMaterial.indexOfRefraction = 1.5f;
    aMaterial.alphaMode = core3d::scene::AlphaMode::Opaque;
    aMaterial.alphaCutoff = 0.5f;
    aMaterial.doubleSided = true;
    return aMaterial;
}

core3d::scene::Matrix4d GizmoWorldAnchor(const gp_Ax2& thePosition)
{
    const gp_Dir anX = thePosition.XDirection();
    const gp_Dir aY = thePosition.YDirection();
    const gp_Dir aZ = thePosition.Direction();
    const gp_Pnt anAnchor = thePosition.Location();
    core3d::scene::Matrix4d aWorldFromPixels;
    aWorldFromPixels.values = {
        anX.X(), anX.Y(), anX.Z(), 0.0,
        aY.X(), aY.Y(), aY.Z(), 0.0,
        aZ.X(), aZ.Y(), aZ.Z(), 0.0,
        anAnchor.X(), anAnchor.Y(), anAnchor.Z(), 1.0,
    };
    return aWorldFromPixels;
}

bool AddGizmoComponent(
    core3d::scene::PresentationOverlayContent& theContent,
    const core3d::scene::Matrix4d& theWorldFromPixels,
    const Handle(Graphic3d_ArrayOfTriangles)& theArray,
    const std::string& theIdentifier,
    const std::string& theName,
    const std::uint32_t theMaterialIndex)
{
    core3d::scene::MeshSnapshot aMesh;
    if (!CopyTriangleArray(theArray,
                           theIdentifier + "/mesh",
                           aMesh)) {
        return false;
    }
    const std::uint32_t aMeshIndex =
        static_cast<std::uint32_t>(theContent.meshes.size());
    core3d::scene::InstanceSnapshot anInstance;
    anInstance.entityIdentifier = theIdentifier;
    anInstance.meshIndex = aMeshIndex;
    anInstance.worldFromObject = theWorldFromPixels;
    anInstance.reversesWinding = false;
    anInstance.visible = true;
    anInstance.selectable = false;
    anInstance.selected = false;
    anInstance.name = theName;
    anInstance.role = core3d::scene::RenderRole::Gizmo;
    anInstance.coordinateSpace =
        core3d::scene::CoordinateSpace::WorldAnchorPixels;
    anInstance.depthPolicy = core3d::scene::DepthPolicy::Topmost;
    anInstance.renderStyle = core3d::scene::RenderStyle::Shaded;
    anInstance.primitiveBindings.push_back({
        theMaterialIndex,
        0,
        true,
    });
    theContent.meshes.push_back(std::move(aMesh));
    theContent.instances.push_back(std::move(anInstance));
    return true;
}
}

//=======================================================================
//function : init
//purpose  :
//=======================================================================
void Core3DManipulator::init()
{
    // Create axis in the default coordinate system. The custom position is applied in local transformation.
    myAxes[0] = Axis (gp::OX(), Quantity_NOC_DEEPSKYBLUE2);
    myAxes[1] = Axis (gp::OY(), Quantity_NOC_PALEGREEN1);
    myAxes[2] = Axis (gp::OZ(), Quantity_NOC_INDIANRED1);
    
    Graphic3d_MaterialAspect aShadingMaterial;
    aShadingMaterial.SetSpecularColor(Quantity_NOC_BLACK);
    aShadingMaterial.SetMaterialType (Graphic3d_MATERIAL_ASPECT);
    
    myDrawer->SetShadingAspect (new Prs3d_ShadingAspect());
    myDrawer->ShadingAspect()->Aspect()->SetInteriorStyle (Aspect_IS_SOLID);
    myDrawer->ShadingAspect()->SetColor (Quantity_NOC_WHITE);
    myDrawer->ShadingAspect()->SetMaterial (aShadingMaterial);
    
    Graphic3d_MaterialAspect aHilightMaterial;
    aHilightMaterial.SetColor (Quantity_NOC_AZURE);
    aHilightMaterial.SetAmbientColor (Quantity_NOC_BLACK);
    aHilightMaterial.SetDiffuseColor (Quantity_NOC_BLACK);
    aHilightMaterial.SetSpecularColor(Quantity_NOC_BLACK);
    aHilightMaterial.SetEmissiveColor(Quantity_NOC_BLACK);
    aHilightMaterial.SetMaterialType (Graphic3d_MATERIAL_ASPECT);
    
    myHighlightAspect = new Prs3d_ShadingAspect();
    myHighlightAspect->Aspect()->SetInteriorStyle (Aspect_IS_SOLID);
    myHighlightAspect->SetMaterial (aHilightMaterial);
    
    Graphic3d_MaterialAspect aDraggerMaterial;
    aDraggerMaterial.SetAmbientColor (Quantity_NOC_BLACK);
    aDraggerMaterial.SetDiffuseColor (Quantity_NOC_BLACK);
    aDraggerMaterial.SetSpecularColor(Quantity_NOC_BLACK);
    aDraggerMaterial.SetMaterialType(Graphic3d_MATERIAL_ASPECT);
    
    myDraggerHighlight = new Prs3d_ShadingAspect();
    myDraggerHighlight->Aspect()->SetInteriorStyle(Aspect_IS_SOLID);
    myDraggerHighlight->SetMaterial(aDraggerMaterial);
    
    myDraggerHighlight->SetTransparency(0.5);
    
    SetSize (100);
    SetZLayer (Graphic3d_ZLayerId_Topmost);
	
	myOldScaleFactor = 1.0;
	myOldIndexScale = 0;
}

Standard_Boolean Core3DManipulator::CaptureIdleMoveRotateOverlay(
    core3d::scene::PresentationOverlayContent& theContent) const noexcept
{
    try {
        if (!myHasCenter) {
            return Standard_False;
        }
        for (Standard_Integer anAxis = 0; anAxis < 3; ++anAxis) {
            if (!myAxes[anAxis].HasTranslation()
                || !myAxes[anAxis].HasRotation()) {
                return Standard_False;
            }
        }

        core3d::scene::PresentationOverlayContent aContent;
        aContent.kind =
            core3d::scene::PresentationOverlayKind::MoveRotateGizmo;
        aContent.meshes.reserve(7);
        aContent.instances.reserve(7);
        aContent.materials.reserve(4);
        aContent.materials.push_back(GizmoMaterial(
            "gizmo/material/center", Quantity_Color(Quantity_NOC_WHITE)));
        aContent.materials.push_back(GizmoMaterial(
            "gizmo/material/x", myAxes[0].Color()));
        aContent.materials.push_back(GizmoMaterial(
            "gizmo/material/y", myAxes[1].Color()));
        aContent.materials.push_back(GizmoMaterial(
            "gizmo/material/z", myAxes[2].Color()));

        const core3d::scene::Matrix4d aWorldFromPixels =
            GizmoWorldAnchor(myPosition);

        if (!AddGizmoComponent(aContent,
                               aWorldFromPixels,
                               myCenter.Array(),
                               "gizmo/center",
                               "Move/rotate center",
                               0)) {
            return Standard_False;
        }
        static constexpr const char* kAxisNames[3] = {"x", "y", "z"};
        static constexpr const char* kAxisDisplayNames[3] = {"X", "Y", "Z"};
        for (Standard_Integer anAxis = 0; anAxis < 3; ++anAxis) {
            const std::string aTranslationIdentifier =
                std::string("gizmo/translation/") + kAxisNames[anAxis];
            const std::string aTranslationName =
                std::string(kAxisDisplayNames[anAxis]) + " translation";
            if (!AddGizmoComponent(
                    aContent,
                    aWorldFromPixels,
                    myAxes[anAxis].TriangleArrayF(),
                    aTranslationIdentifier,
                    aTranslationName,
                    static_cast<std::uint32_t>(anAxis + 1))) {
                return Standard_False;
            }
            const std::string aRotationIdentifier =
                std::string("gizmo/rotation/") + kAxisNames[anAxis];
            const std::string aRotationName =
                std::string(kAxisDisplayNames[anAxis]) + " rotation";
            if (!AddGizmoComponent(
                    aContent,
                    aWorldFromPixels,
                    myAxes[anAxis].RotatorDisk().Array(),
                    aRotationIdentifier,
                    aRotationName,
                    static_cast<std::uint32_t>(anAxis + 1))) {
                return Standard_False;
            }
        }
        if (aContent.meshes.size() != 7
            || aContent.instances.size() != 7
            || aContent.materials.size() != 4) {
            return Standard_False;
        }
        theContent = std::move(aContent);
        return Standard_True;
    } catch (const Standard_Failure&) {
        return Standard_False;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean Core3DManipulator::CaptureIdleScaleOverlay(
    core3d::scene::PresentationOverlayContent& theContent) const noexcept
{
    try {
        if (!myHasCenter) {
            return Standard_False;
        }
        for (Standard_Integer anAxis = 0; anAxis < 3; ++anAxis) {
            if (!myAxes[anAxis].HasScaling()
                || !myAxes[anAxis].HasScalingUniform()
                || myAxes[anAxis].HasTranslation()
                || myAxes[anAxis].HasRotation()
                || myAxes[anAxis].HasDragging()
                || myAxes[anAxis].HasMirroringPos()
                || myAxes[anAxis].HasMirroringNeg()) {
                return Standard_False;
            }
        }

        core3d::scene::PresentationOverlayContent aContent;
        aContent.kind = core3d::scene::PresentationOverlayKind::ScaleGizmo;
        aContent.meshes.reserve(5);
        aContent.instances.reserve(5);
        aContent.materials.reserve(4);
        aContent.materials.push_back(GizmoMaterial(
            "gizmo/material/center", Quantity_Color(Quantity_NOC_WHITE)));
        aContent.materials.push_back(GizmoMaterial(
            "gizmo/material/x", myAxes[0].Color()));
        aContent.materials.push_back(GizmoMaterial(
            "gizmo/material/y", myAxes[1].Color()));
        aContent.materials.push_back(GizmoMaterial(
            "gizmo/material/z", myAxes[2].Color()));

        const core3d::scene::Matrix4d aWorldFromPixels =
            GizmoWorldAnchor(myPosition);
        if (!AddGizmoComponent(aContent,
                               aWorldFromPixels,
                               myCenter.Array(),
                               "gizmo/center",
                               "Scale center",
                               0)) {
            return Standard_False;
        }

        static constexpr const char* kAxisNames[3] = {"x", "y", "z"};
        static constexpr const char* kAxisDisplayNames[3] = {"X", "Y", "Z"};
        for (Standard_Integer anAxis = 0; anAxis < 3; ++anAxis) {
            const std::string anIdentifier =
                std::string("gizmo/scaling/") + kAxisNames[anAxis];
            const std::string aName =
                std::string(kAxisDisplayNames[anAxis]) + " scale";
            if (!AddGizmoComponent(
                    aContent,
                    aWorldFromPixels,
                    myAxes[anAxis].ScalerCube().Array(),
                    anIdentifier,
                    aName,
                    static_cast<std::uint32_t>(anAxis + 1))) {
                return Standard_False;
            }
        }
        if (!AddGizmoComponent(
                aContent,
                aWorldFromPixels,
                myAxes[1].ScalerSphereUniform().Array(),
                "gizmo/scaling/uniform",
                "Uniform scale",
                2)) {
            return Standard_False;
        }

        if (aContent.meshes.size() != 5
            || aContent.instances.size() != 5
            || aContent.materials.size() != 4) {
            return Standard_False;
        }
        theContent = std::move(aContent);
        return Standard_True;
    } catch (const Standard_Failure&) {
        return Standard_False;
    } catch (...) {
        return Standard_False;
    }
}

//=======================================================================
//function : getHighlightPresentation
//purpose  :
//=======================================================================
Handle(Prs3d_Presentation) Core3DManipulator::getHighlightPresentation (const Handle(SelectMgr_EntityOwner)& theOwner) const
{
    Handle(Prs3d_Presentation) aDummyPrs;
    Handle(AIS_ManipulatorOwner) anOwner = Handle(AIS_ManipulatorOwner)::DownCast (theOwner);
    if (anOwner.IsNull())
    {
        return aDummyPrs;
    }
    
    switch (anOwner->Mode())
    {
        case AIS_MM_Translation     : return myAxes[anOwner->Index()].TranslatorHighlightPrs();
        case AIS_MM_Rotation        : return myAxes[anOwner->Index()].RotatorHighlightPrs();
        case AIS_MM_Scaling         : return myAxes[anOwner->Index()].ScalerHighlightPrs();
		case AIS_MM_ScalingUniform  : return myAxes[anOwner->Index()].ScalerHighlightUniformPrs();
        case AIS_MM_TranslationPlane: return myAxes[anOwner->Index()].DraggerHighlightPrs();
		case AIS_MM_MirroringPlanePos: return myAxes[anOwner->Index()].MirroringHighlightPosPrs();
		case AIS_MM_MirroringPlaneNeg: return myAxes[anOwner->Index()].MirroringHighlightNegPrs();
        case AIS_MM_None            : break;
    }
    
    return aDummyPrs;
}

//=======================================================================
//function : getGroup
//purpose  :
//=======================================================================
Handle(Graphic3d_Group) Core3DManipulator::getGroup (const Standard_Integer theIndex, const AIS_ManipulatorMode theMode) const
{
    Handle(Graphic3d_Group) aDummyGroup;
    
    if (theIndex < 0 || theIndex > 2)
    {
        return aDummyGroup;
    }
    
    switch (theMode)
    {
        case AIS_MM_Translation     : return myAxes[theIndex].TranslatorGroup();
        case AIS_MM_Rotation        : return myAxes[theIndex].RotatorGroup();
        case AIS_MM_Scaling         : return myAxes[theIndex].ScalerGroup();
		case AIS_MM_ScalingUniform  : return myAxes[theIndex].ScalerUniformGroup();
        case AIS_MM_TranslationPlane: return myAxes[theIndex].DraggerGroup();
		case AIS_MM_MirroringPlanePos: return myAxes[theIndex].MirroringGroupPos();
		case AIS_MM_MirroringPlaneNeg: return myAxes[theIndex].MirroringGroupNeg();
        case AIS_MM_None            : break;
    }
    
    return aDummyGroup;
}

//=======================================================================
//function : Constructor
//purpose  :
//=======================================================================
Core3DManipulator::Core3DManipulator()
: myPosition (gp::XOY()),
myCurrentIndex (-1),
myCurrentMode (AIS_MM_None),
myIsActivationOnDetection (Standard_False),
myIsZoomPersistentMode (Standard_True),
myHasStartedTransformation (Standard_False),
myStartPosition (gp::XOY()),
myStartPick (0.0, 0.0, 0.0),
myPrevState (0.0),
myHasCenter (true),
myObjectOrientation(false)
{
    SetInfiniteState();
    SetMutable (Standard_True);
    SetDisplayMode (AIS_Shaded);
    init();
}

//=======================================================================
//function : Constructor
//purpose  :
//=======================================================================
Core3DManipulator::Core3DManipulator (const gp_Ax2& thePosition)
: myPosition (thePosition),
myCurrentIndex (-1),
myCurrentMode (AIS_MM_None),
myIsActivationOnDetection (Standard_False),
myIsZoomPersistentMode (Standard_True),
myHasStartedTransformation (Standard_False),
myStartPosition (gp::XOY()),
myStartPick (0.0, 0.0, 0.0),
myPrevState (0.0),
myHasCenter (true),
myObjectOrientation(false)
{
    SetInfiniteState();
    SetMutable (Standard_True);
    SetDisplayMode (AIS_Shaded);
    init();
}

//=======================================================================
//function : SetPart
//purpose  :
//=======================================================================
void Core3DManipulator::SetPart (const Standard_Integer theAxisIndex, const AIS_ManipulatorMode theMode, const Standard_Boolean theIsEnabled)
{
    Standard_ProgramError_Raise_if (theAxisIndex < 0 || theAxisIndex > 2, "Core3DManipulator::SetMode(): axis index should be between 0 and 2");
	myHasCenter = true;
	if (theIsEnabled)
		myObjectOrientation = false;

    switch (theMode)
    {
        case AIS_MM_Translation:
            myAxes[theAxisIndex].SetTranslation (theIsEnabled);
            break;
            
        case AIS_MM_Rotation:
            myAxes[theAxisIndex].SetRotation (theIsEnabled);
            break;
            
        case AIS_MM_Scaling:
            myAxes[theAxisIndex].SetScaling (theIsEnabled);
			myObjectOrientation = theIsEnabled;
            break;
			
		case AIS_MM_ScalingUniform:
			myAxes[theAxisIndex].SetScalingUniform (theIsEnabled);
			myObjectOrientation = theIsEnabled;
			break;
            
        case AIS_MM_TranslationPlane:
            myAxes[theAxisIndex].SetDragging (theIsEnabled);
            break;
			
		case AIS_MM_MirroringPlanePos:
			myAxes[theAxisIndex].SetMirroringPos (theIsEnabled);
			if (theIsEnabled)
				myHasCenter = false;
			break;
			
		case AIS_MM_MirroringPlaneNeg:
			myAxes[theAxisIndex].SetMirroringNeg (theIsEnabled);
			if (theIsEnabled)
				myHasCenter = false;
			break;
            
        case AIS_MM_None:
            break;
    }
}

//=======================================================================
//function : SetPart
//purpose  :
//=======================================================================
void Core3DManipulator::SetPart (const AIS_ManipulatorMode theMode, const Standard_Boolean theIsEnabled)
{
    for (Standard_Integer anIt = 0; anIt < 3; ++anIt)
    {
        SetPart (anIt, theMode, theIsEnabled);
    }
}

//=======================================================================
//function : EnableMode
//purpose  :
//=======================================================================
void Core3DManipulator::EnableMode (const AIS_ManipulatorMode theMode)
{
    if (!IsAttached())
    {
        return;
    }
    
    const Handle(AIS_InteractiveContext)& aContext = GetContext();
    if (aContext.IsNull())
    {
        return;
    }
    
    aContext->Activate (this, theMode);
}

//=======================================================================
//function : attachToBox
//purpose  :
//=======================================================================
void Core3DManipulator::attachToBox (const Bnd_Box& theBox, const gp_Ax2& anAx2)
{
    if (theBox.IsVoid())
    {
        return;
    }
    
    Standard_Real anXmin = 0.0, anYmin = 0.0, aZmin = 0.0, anXmax = 0.0, anYmax = 0.0, aZmax = 0.0;
    theBox.Get (anXmin, anYmin, aZmin, anXmax, anYmax, aZmax);
    
    gp_Ax2 aPosition = gp::XOY();
    aPosition.SetLocation (gp_Pnt ((anXmin + anXmax) * 0.5, (anYmin + anYmax) * 0.5, (aZmin + aZmax) * 0.5));

	if (myObjectOrientation) {
		aPosition.SetDirection(anAx2.Direction());
		aPosition.SetXDirection(anAx2.XDirection());
	}

    SetPosition (aPosition);
}

//=======================================================================
//function : adjustSize
//purpose  :
//=======================================================================
void Core3DManipulator::adjustSize (const Bnd_Box& theBox)
{
    Standard_Real aXmin = 0., aYmin = 0., aZmin = 0., aXmax = 0., aYmax = 0., aZmax = 0.0;
    theBox.Get (aXmin, aYmin, aZmin, aXmax, aYmax, aZmax);
    Standard_Real aXSize = aXmax - aXmin;
    Standard_Real aYSize = aYmax - aYmin;
    Standard_Real aZSize  = aZmax  - aZmin;
    
    SetSize ((Standard_ShortReal) (Max (aXSize, Max (aYSize, aZSize)) * 0.5));
}

//=======================================================================
//function : Attach
//purpose  :
//=======================================================================
void Core3DManipulator::Attach (const Handle(AIS_InteractiveObject)& theObject, const OptionsForAttach& theOptions)
{
	if (theObject->IsKind (STANDARD_TYPE(AIS_Manipulator)))
	{
		return;
	}
	Handle(Core3DManipulatorObjectSequence) aSeq = new Core3DManipulatorObjectSequence();

	Handle(Core3DManipulatorObjectSequence) anObjects = Objects();
	if (!anObjects.IsNull()) {
		for (Handle(AIS_InteractiveObject) obj : anObjects->Sequence() ) {
			aSeq->Append(obj);
		}
	}
	aSeq->Append(theObject);
//	Handle(Core3DManipulatorObjectSequence)::DownCast (myOwner)->Append (theObject);
//	aSeq-> (theObject);
    Handle(AIS_Shape) ais = Handle(AIS_Shape)::DownCast(theObject);
    if(!ais.IsNull()) {
        mySourceShapes[theObject] = ais->Shape();
        Attach (aSeq, theOptions);
    }
}

//=======================================================================
//function : Attach
//purpose  :
//=======================================================================
void Core3DManipulator::Attach (const Handle(Core3DManipulatorObjectSequence)& theObjects, const OptionsForAttach& theOptions)
{
    if (theObjects->Size() < 1)
    {
        return;
    }
    
    SetOwner (theObjects);
	UpdateCachedShapes();
    Bnd_Box aBoxSum;
	Bnd_Box aBox;
	for (auto &obj : *theObjects) {
		obj->BoundingBox (aBox);
		aBoxSum.Add(aBox);
	}
    
    if (theOptions.AdjustPosition)
    {
		gp_Trsf ltrsf;
		if (Objects()->Size() == 1) //single-select
			ltrsf = Object()->LocalTransformation();
		gp_Ax2 anAx2 (gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
		anAx2.Transform (ltrsf);
        attachToBox (aBoxSum, anAx2);
    }
    
    if (theOptions.AdjustSize)
    {
        adjustSize (aBoxSum);
    }
    
    const Handle(AIS_InteractiveContext)& aContext = Object()->GetContext();
    if (!aContext.IsNull())
    {
        if (!aContext->IsDisplayed (this))
        {
            aContext->Display (this, Standard_False);
        }
        else
        {
            aContext->Update (this, Standard_False);
            aContext->RecomputeSelectionOnly (this);
        }
        
        aContext->Load (this);
    }
    
    if (theOptions.EnableModes)
    {
        EnableMode (AIS_MM_Rotation);
        EnableMode (AIS_MM_Translation);
        EnableMode (AIS_MM_Scaling);
		EnableMode (AIS_MM_ScalingUniform);
        EnableMode (AIS_MM_TranslationPlane);
		EnableMode (AIS_MM_MirroringPlanePos);
		EnableMode (AIS_MM_MirroringPlaneNeg);
    }
}

//=======================================================================
//function : Detach
//purpose  :
//=======================================================================
void Core3DManipulator::Detach()
{
    DeactivateCurrentMode();
    
    if (!IsAttached())
    {
        return;
    }
    
    Handle(AIS_InteractiveObject) anObject = Object();
	mySourceShapes.clear();

    const Handle(AIS_InteractiveContext)& aContext = anObject->GetContext();
    if (!aContext.IsNull())
    {
        aContext->Remove (this, Standard_False);
    }
    
    SetOwner (NULL);
}

void Core3DManipulator::Detach (const Handle(AIS_InteractiveObject)& theObject) {

	Handle(Core3DManipulatorObjectSequence) anObjects = Objects();
	Core3DManipulatorObjectSequence::Iterator anObjIter (*anObjects);
	NCollection_Sequence<gp_Trsf>::Iterator aTrsfIter (myStartTrsfs);
	for (; anObjIter.More(); anObjIter.Next(), aTrsfIter.Next())
	{
		if (theObject == anObjIter.Value()) {
			anObjects->Remove(anObjIter);
			break;
		}
	}
	Attach(anObjects);

}


//=======================================================================
//function : Objects
//purpose  :
//=======================================================================
Handle(Core3DManipulatorObjectSequence) Core3DManipulator::Objects() const
{
    return Handle(Core3DManipulatorObjectSequence)::DownCast (GetOwner());
}

//=======================================================================
//function : Object
//purpose  :
//=======================================================================
Handle(AIS_InteractiveObject) Core3DManipulator::Object (const Standard_Integer theIndex) const
{
    Handle(Core3DManipulatorObjectSequence) anOwner = Handle(Core3DManipulatorObjectSequence)::DownCast (GetOwner());

    if (anOwner.IsNull() || anOwner->IsEmpty())
    {
        return NULL;
    }
    

    Standard_ProgramError_Raise_if (theIndex < anOwner->Lower() || theIndex > anOwner->Upper(), "Core3DManipulator::Object(): wrong index value");
    
    return anOwner->Value (theIndex);
}

//=======================================================================
//function : Object
//purpose  :
//=======================================================================
Handle(AIS_InteractiveObject) Core3DManipulator::Object() const
{
    return Object (1);
}

//=======================================================================
//function : ObjectTransformation
//purpose  :
//=======================================================================
Standard_Boolean Core3DManipulator::ObjectTransformation (const Standard_Integer theMaxX, const Standard_Integer theMaxY,
                                                          const Handle(Core3DView)& theView, const Handle(AIS_InteractiveContext)& theCtx, gp_Trsf& theTrsf)
{
    
    const auto snapping_linear = Snapping::Instance().getLinear();
    const auto snapping_angular = Snapping::Instance().getAngular();
    const auto snapping_scaling = Snapping::Instance().getScaling();
    
    // Initialize start reference data
    if (!myHasStartedTransformation)
    {
        myStartTrsfs.Clear();
        Handle(Core3DManipulatorObjectSequence) anObjects = Objects();
        for (Core3DManipulatorObjectSequence::Iterator anObjIter (*anObjects); anObjIter.More(); anObjIter.Next())
        {
            myStartTrsfs.Append (anObjIter.Value()->LocalTransformation());
        }
        myStartPosition = myPosition;
    }
    
    // Get 3d point with projection vector
    Graphic3d_Vec3d anInputPoint, aProj;
    theView->ConvertWithProj (theMaxX, theMaxY, anInputPoint.x(), anInputPoint.y(), anInputPoint.z(), aProj.x(), aProj.y(), aProj.z());
    const gp_Lin anInputLine (gp_Pnt (anInputPoint.x(), anInputPoint.y(), anInputPoint.z()), gp_Dir (aProj.x(), aProj.y(), aProj.z()));
    switch (myCurrentMode)
    {
        case AIS_MM_Translation:
        case AIS_MM_Scaling:
		case AIS_MM_ScalingUniform:
        {
            const gp_Lin aLine (myStartPosition.Location(), myAxes[myCurrentIndex].Position().Direction());
            Extrema_ExtElC anExtrema (anInputLine, aLine, Precision::Angular());
            if (!anExtrema.IsDone()
                || anExtrema.IsParallel()
                || anExtrema.NbExt() != 1)
            {
                // translation cannot be done co-directed with camera
                return Standard_False;
            }
            
            Extrema_POnCurv anExPnts[2];
            anExtrema.Points (1, anExPnts[0], anExPnts[1]);
            gp_Pnt aNewPosition = anExPnts[1].Value();
            
            // apply linear snapping
            if(snapping_linear.has_value() && myHasStartedTransformation) {
                double count = aNewPosition.Distance(myStartPick) / *snapping_linear;
                auto v = aNewPosition.XYZ() - myStartPick.XYZ();
                if(v.Modulus() > gp::Resolution()) {
                    auto vec = v.Normalized().Multiplied(*snapping_linear * floor(count));
                    
                    aNewPosition = gp_Pnt(myStartPick.Translated(gp_Vec(vec)));
                    gp_Pnt current = Snapping::Instance().getCurrentPosition();
                    
                    if(current.Distance(aNewPosition) < *snapping_linear) {
                        aNewPosition = current;
                    } else if(aNewPosition.Distance(myStartPick) < *snapping_linear) {
                        aNewPosition = myStartPick;
                        Snapping::Instance().setCurrentPosition(myStartPick);
                        return Standard_True;
                    } else {
                        Snapping::Instance().setCurrentPosition(aNewPosition);
                    }
                } else {
                    return Standard_False;
                }
                
                
            }
            
            if (!myHasStartedTransformation)
            {
                Snapping::Instance().setCurrentPosition(aNewPosition);
                myStartPick = aNewPosition;
                myHasStartedTransformation = Standard_True;
                return Standard_True;
            }
            else if (aNewPosition.Distance (myStartPick) < Precision::Confusion())
            {
                return Standard_False;
            }
            
            gp_Trsf aNewTrsf;
            if (myCurrentMode == AIS_MM_Translation)
            {
                aNewTrsf.SetTranslation (gp_Vec(myStartPick, aNewPosition));
                theTrsf *= aNewTrsf;
                
            }
            else if (myCurrentMode == AIS_MM_ScalingUniform || myCurrentMode == AIS_MM_Scaling)
            {
                if (aNewPosition.Distance (myStartPosition.Location()) < Precision::Confusion())
                {
                    return Standard_False;
                }
                
                Standard_Real aCoeff = myStartPosition.Location().Distance (aNewPosition)
                / myStartPosition.Location().Distance (myStartPick);
                
                if(snapping_scaling.has_value()) {
                    const auto steps = aCoeff / *snapping_scaling;
                    aCoeff = floor(steps) * (*snapping_scaling);
                    aCoeff = aCoeff < 0.1f ? 0.1 : aCoeff;
                }
				
				if (myCurrentMode == AIS_MM_Scaling) {
					NonUniformScale(aCoeff, theCtx);
				} else {
					aNewTrsf.SetScale (myPosition.Location(), aCoeff);
					theTrsf = aNewTrsf;
				}
            }
            return Standard_True;
        }
        case AIS_MM_Rotation:
        {
            const gp_Pnt aPosLoc   = myStartPosition.Location();
            const gp_Ax1 aCurrAxis = getAx1FromAx2Dir (myStartPosition, myCurrentIndex);
            IntAna_IntConicQuad aIntersector (anInputLine, gp_Pln (aPosLoc, aCurrAxis.Direction()), Precision::Angular(), Precision::Intersection());
            if (!aIntersector.IsDone()
                || aIntersector.IsParallel()
                || aIntersector.NbPoints() < 1)
            {
                return Standard_False;
            }
            
            const gp_Pnt aNewPosition = aIntersector.Point (1);
            if (!myHasStartedTransformation)
            {
                myStartPick = aNewPosition;
                myHasStartedTransformation = Standard_True;
                gp_Dir aStartAxis = gce_MakeDir (aPosLoc, myStartPick);
                myPrevState = aStartAxis.AngleWithRef (gce_MakeDir(aPosLoc, aNewPosition), aCurrAxis.Direction());
                return Standard_True;
            }
            
            if (aNewPosition.Distance (myStartPick) < Precision::Confusion())
            {
                return Standard_False;
            }
            
            gp_Dir aStartAxis = aPosLoc.IsEqual (myStartPick, Precision::Confusion())
            ? getAx1FromAx2Dir (myStartPosition, (myCurrentIndex + 1) % 3).Direction()
            : gce_MakeDir (aPosLoc, myStartPick);
            
            gp_Dir aCurrentAxis = gce_MakeDir (aPosLoc, aNewPosition);
            Standard_Real anAngle = aStartAxis.AngleWithRef (aCurrentAxis, aCurrAxis.Direction());
            
            // Change value of an angle if it should have different sign.
            if (anAngle * myPrevState < 0 && Abs (anAngle) < M_PI_2)
            {
                Standard_Real aSign = myPrevState > 0 ? -1.0 : 1.0;
                anAngle = aSign * (M_PI * 2 - anAngle);
            }
            
            if (Abs (anAngle) < Precision::Confusion())
            {
                return Standard_False;
            }
            
            if(snapping_angular.has_value()) {
                const auto radians_step = *snapping_angular * M_PI / 180.;
                auto steps = anAngle / radians_step;
                anAngle = floor(steps) * radians_step;
            }
            
            gp_Trsf aNewTrsf;
            aNewTrsf.SetRotation (aCurrAxis, anAngle);
            theTrsf *= aNewTrsf;
            myPrevState = anAngle;
            return Standard_True;
        }
        case AIS_MM_TranslationPlane:
        {
            const gp_Pnt aPosLoc = myStartPosition.Location();
            const gp_Ax1 aCurrAxis = getAx1FromAx2Dir(myStartPosition, myCurrentIndex);
            IntAna_IntConicQuad aIntersector(anInputLine, gp_Pln(aPosLoc, aCurrAxis.Direction()), Precision::Angular(), Precision::Intersection());
            if (!aIntersector.IsDone() || aIntersector.NbPoints() < 1)
            {
                return Standard_False;
            }
            
            const gp_Pnt aNewPosition = aIntersector.Point(1);
            if (!myHasStartedTransformation)
            {
                myStartPick = aNewPosition;
                myHasStartedTransformation = Standard_True;
                return Standard_True;
            }
            
            if (aNewPosition.Distance(myStartPick) < Precision::Confusion())
            {
                return Standard_False;
            }
            
            gp_Trsf aNewTrsf;
            aNewTrsf.SetTranslation(gp_Vec(myStartPick, aNewPosition));
            theTrsf *= aNewTrsf;
            return Standard_True;
        }
        case AIS_MM_None:
        {
            return Standard_False;
        }
		default:
		{
			return Standard_False;
		}
    }
    return Standard_False;
}

//=======================================================================
//function : ProcessDragging
//purpose  :
//=======================================================================
Standard_Boolean Core3DManipulator::ProcessDragging (const Handle(AIS_InteractiveContext)& theCtx,
                                                     const Handle(Core3DView)& theView,
                                                     const Handle(SelectMgr_EntityOwner)&,
                                                     const Graphic3d_Vec2i& theDragFrom,
                                                     const Graphic3d_Vec2i& theDragTo,
                                                     const AIS_DragAction theAction)
{
    switch (theAction)
    {
        case AIS_DragAction_Start:
        {
            if (HasActiveMode())
            {
                StartTransform (theDragFrom.x(), theDragFrom.y(), theView, theCtx);
                return Standard_True;
            }
            break;
        }
        case AIS_DragAction_Confirmed:
        {
            return Standard_True;
        }
        case AIS_DragAction_Update:
        {
            Transform (theDragTo.x(), theDragTo.y(), theView, theCtx);
            return Standard_True;
        }
        case AIS_DragAction_Abort:
        {
            StopTransform (false);
            return Standard_True;
        }
        case AIS_DragAction_Stop:
            break;
    }
    return Standard_False;
}

void Core3DManipulator::UpdateCachedShapes() {
	mySourceShapes.clear();
	
	Handle(Core3DManipulatorObjectSequence) anObjects = Objects();
	Core3DManipulatorObjectSequence::Iterator anObjIter (*anObjects);
	for (; anObjIter.More(); anObjIter.Next()) {
		mySourceShapes[anObjIter.Value()] = Handle(AIS_Shape)::DownCast(anObjIter.Value())->Shape();
	}
//	Handle(AIS_InteractiveObject) anObject = Object();
//	if (!anObject.IsNull()) {
//		mySourceShapes.push_back(Handle(AIS_Shape)::DownCast(anObject)->Shape());
//	}
}

//=======================================================================
//function : StartTransform
//purpose  :
//=======================================================================
void Core3DManipulator::StartTransform (const Standard_Integer theX, const Standard_Integer theY, const Handle(Core3DView)& theView, const Handle(AIS_InteractiveContext)& theCtx)
{
    if (myHasStartedTransformation)
    {
        return;
    }
    
    gp_Trsf aTrsf;
    ObjectTransformation (theX, theY, theView, theCtx, aTrsf);
}

//=======================================================================
//function : StopTransform
//purpose  :
//=======================================================================
void Core3DManipulator::StopTransform (const Standard_Boolean theToApply)
{
    if (!IsAttached() || !myHasStartedTransformation)
    {
        return;
    }
    
    myHasStartedTransformation = Standard_False;
    if (theToApply)
    {
        return;
    }
    
    Handle(Core3DManipulatorObjectSequence) anObjects = Objects();
    Core3DManipulatorObjectSequence::Iterator anObjIter (*anObjects);
    NCollection_Sequence<gp_Trsf>::Iterator aTrsfIter (myStartTrsfs);
    for (; anObjIter.More(); anObjIter.Next(), aTrsfIter.Next())
    {
        anObjIter.ChangeValue()->SetLocalTransformation (aTrsfIter.Value());
    }
    SetPosition (myStartPosition);
}

//=======================================================================
//function : Transform
//purpose  :
//=======================================================================
void Core3DManipulator::Transform (const gp_Trsf& theTrsf)
{
    if (!IsAttached() || !myHasStartedTransformation)
    {
        return;
    }
    
	//if (!myAxes[myCurrentIndex].HasMirroringPos() && !myAxes[myCurrentIndex].HasMirroringNeg())
    {
        Handle(Core3DManipulatorObjectSequence) anObjects = Objects();
        Core3DManipulatorObjectSequence::Iterator anObjIter (*anObjects);
        NCollection_Sequence<gp_Trsf>::Iterator aTrsfIter (myStartTrsfs);
        for (; anObjIter.More(); anObjIter.Next(), aTrsfIter.Next())
        {
            const Handle(AIS_InteractiveObject)& anObj = anObjIter.ChangeValue();
            const gp_Trsf& anOldTrsf = aTrsfIter.Value();
            const Handle(TopLoc_Datum3D)& aParentTrsf = anObj->CombinedParentTransformation();
            if (!aParentTrsf.IsNull()
                && aParentTrsf->Form() != gp_Identity)
            {
                // recompute local transformation relative to parent transformation
                const gp_Trsf aNewLocalTrsf = aParentTrsf->Trsf().Inverted() * theTrsf * aParentTrsf->Trsf() * anOldTrsf;
                anObj->SetLocalTransformation (aNewLocalTrsf);
            }
            else
            {
                anObj->SetLocalTransformation (theTrsf * anOldTrsf);
            }
        }
    }
    
    if ((myCurrentMode == AIS_MM_Translation      && myBehaviorOnTransform.FollowTranslation)
        || (myCurrentMode == AIS_MM_Rotation         && myBehaviorOnTransform.FollowRotation)
        || (myCurrentMode == AIS_MM_TranslationPlane && myBehaviorOnTransform.FollowDragging))
    {
        gp_Pnt aPos  = myStartPosition.Location().Transformed (theTrsf);
        gp_Dir aVDir = myStartPosition.Direction().Transformed (theTrsf);
        gp_Dir aXDir = myStartPosition.XDirection().Transformed (theTrsf);
        SetPosition (gp_Ax2 (aPos, aVDir, aXDir));
    }
}

void Core3DManipulator::NonUniformScale (const Standard_Real scaleFactor, const Handle(AIS_InteractiveContext)& theCtx)
{
	if (!IsAttached() || !myHasStartedTransformation)
	{
		return;
	}
	
	{
		if (abs(myOldScaleFactor - scaleFactor) < 1e-2f) //throttle
			return;
		
		if (scaleFactor < 1e-2f)
			return;
		
        bool useCachedShape = mySourceShapes.size() > 0;
		
		Standard_Real sf = useCachedShape ? scaleFactor : scaleFactor / myOldScaleFactor;

		if (myOldIndexScale != myCurrentIndex) {//reset on change dir
			myOldScaleFactor = 1.0;
			myOldIndexScale = myCurrentIndex;
		} else {
			myOldScaleFactor = scaleFactor;
		}

		Handle(Core3DManipulatorObjectSequence) anObjects = Objects();
		Core3DManipulatorObjectSequence::Iterator anObjIter (*anObjects);
		
		bool singleSelect = (anObjects->Size() == 1);
		
		for (; anObjIter.More(); anObjIter.Next())
		{
			gp_Trsf objTrsf = anObjIter.Value()->Transformation();
			gp_Trsf manTrsf;
			
			if (ZoomPersistence()) {
				manTrsf.SetTranslation(myPosition.Location().XYZ());
			} else
				manTrsf = Transformation();
			
			if (singleSelect)
				manTrsf.SetRotationPart(objTrsf.GetRotation()); //prevent skew transforms
			
			gp_GTrsf worldT = objTrsf.Inverted() * manTrsf;
		
			NCollection_Vec3<Standard_Real> diagonal;
			switch (myCurrentIndex) {
				case 0:
					diagonal = NCollection_Vec3<Standard_Real>(sf,1,1);
					break;
				case 1:
					diagonal = NCollection_Vec3<Standard_Real>(1,sf,1);
					break;
				case 2:
					diagonal = NCollection_Vec3<Standard_Real>(1,1,sf);
					break;
				default:
					diagonal = NCollection_Vec3<Standard_Real>(1,1,1);
					break;
			}
			
			NCollection_Mat4<Standard_Real> matScale, mat;
			worldT.GetMat4(mat);
			matScale.SetDiagonal(diagonal);
			mat = mat * matScale * mat.Inverted();
			gp_GTrsf t;
			t.SetMat4(mat);
				
			ShapeUpgrade_RemoveLocations rl;
            
			if (useCachedShape && mySourceShapes.find(anObjIter.Value()) != mySourceShapes.end()) {
				rl.Remove(mySourceShapes[anObjIter.Value()]);
			} else {
				auto anAis = Handle(AIS_Shape)::DownCast(anObjIter.Value());
				rl.Remove(anAis->Shape());
			}
			
			const TopoDS_Shape &shapeClean = rl.GetResult();
			
			BRepBuilderAPI_GTransform tran(shapeClean, t);
			tran.Build();
//			tran.Perform(shapeClean);
			
			if (tran.IsDone()) {
				Handle(AIS_Shape) newShape = Handle(AIS_Shape)::DownCast(anObjIter.ChangeValue());
				newShape->SetShape(tran.Shape());
				theCtx->Redisplay(anObjIter.Value(), Standard_False);
//				std::cout << "replaced scaled shape with " << scaleFactor << std::endl;
			}
		}
	}
}
//=======================================================================
//function : Transform
//purpose  :
//=======================================================================
gp_Trsf Core3DManipulator::Transform (const Standard_Integer thePX, const Standard_Integer thePY,
                                      const Handle(Core3DView)& theView, const Handle(AIS_InteractiveContext)& theCtx)
{
    gp_Trsf aTrsf;
    if (ObjectTransformation (thePX, thePY, theView, theCtx, aTrsf))
    {
        Transform (aTrsf);
    }
    
    return aTrsf;
}

//=======================================================================
//function : SetPosition
//purpose  :
//=======================================================================
void Core3DManipulator::SetPosition (const gp_Ax2& thePosition)
{
    if (!myPosition.Location().IsEqual (thePosition.Location(), Precision::Confusion())
        || !myPosition.Direction().IsEqual (thePosition.Direction(), Precision::Angular())
        || !myPosition.XDirection().IsEqual (thePosition.XDirection(), Precision::Angular()))
    {
        myPosition = thePosition;
        myAxes[0].SetPosition (getAx1FromAx2Dir (thePosition, 0));
        myAxes[1].SetPosition (getAx1FromAx2Dir (thePosition, 1));
        myAxes[2].SetPosition (getAx1FromAx2Dir (thePosition, 2));
        updateTransformation();
    }
}

//=======================================================================
//function : updateTransformation
//purpose  : set local transformation to avoid graphics recomputation
//=======================================================================
void Core3DManipulator::updateTransformation()
{
    gp_Trsf aTrsf;
    
    if (!myIsZoomPersistentMode)
    {
        aTrsf.SetTransformation (myPosition, gp::XOY());
    }
    else
    {
        const gp_Dir& aVDir = myPosition.Direction();
        const gp_Dir& aXDir = myPosition.XDirection();
        aTrsf.SetTransformation (gp_Ax2 (gp::Origin(), aVDir, aXDir), gp::XOY());
    }
    
    Handle(TopLoc_Datum3D) aGeomTrsf = new TopLoc_Datum3D (aTrsf);
    // we explicitly call here setLocalTransformation() of the base class
    // since Core3DManipulator::setLocalTransformation() implementation throws exception
    // as protection from external calls
    AIS_InteractiveObject::setLocalTransformation (aGeomTrsf);
    for (Standard_Integer anIt = 0; anIt < 3; ++anIt)
    {
        myAxes[anIt].Transform (aGeomTrsf);
    }
    
    if (myIsZoomPersistentMode)
    {
        if (TransformPersistence().IsNull()
            ||  TransformPersistence()->Mode() != Graphic3d_TMF_ZoomPers
            || !TransformPersistence()->AnchorPoint().IsEqual (myPosition.Location(), 0.0))
        {
            setTransformPersistence (new Graphic3d_TransformPers (Graphic3d_TMF_ZoomPers, myPosition.Location()));
        }
    }
}

//=======================================================================
//function : SetSize
//purpose  :
//=======================================================================
void Core3DManipulator::SetSize (const Standard_ShortReal theSideLength)
{
    for (Standard_Integer anIt = 0; anIt < 3; ++anIt)
    {
        myAxes[anIt].SetSize (theSideLength);
    }
    
    SetToUpdate();
}

//=======================================================================
//function : SetGap
//purpose  :
//=======================================================================
void Core3DManipulator::SetGap (const Standard_ShortReal theValue)
{
    for (Standard_Integer anIt = 0; anIt < 3; ++anIt)
    {
        myAxes[anIt].SetIndent (theValue);
    }
    
    SetToUpdate();
}

//=======================================================================
//function : DeactivateCurrentMode
//purpose  :
//=======================================================================
void Core3DManipulator::DeactivateCurrentMode()
{
    if (!myIsActivationOnDetection)
    {
        Handle(Graphic3d_Group) aGroup = getGroup (myCurrentIndex, myCurrentMode);
        if (aGroup.IsNull())
        {
            return;
        }
        
        Handle(Prs3d_ShadingAspect) anAspect = new Prs3d_ShadingAspect();
        anAspect->Aspect()->SetInteriorStyle (Aspect_IS_SOLID);
        anAspect->SetMaterial (myDrawer->ShadingAspect()->Material());
        if (myCurrentMode == AIS_MM_TranslationPlane)
            anAspect->SetTransparency(1.0);
        else
        {
            anAspect->SetTransparency(myDrawer->ShadingAspect()->Transparency());
            anAspect->SetColor(myAxes[myCurrentIndex].Color());
        }
        
        aGroup->SetGroupPrimitivesAspect (anAspect->Aspect());
    }
    
    myCurrentIndex = -1;
    myCurrentMode = AIS_MM_None;
    
    if (myHasStartedTransformation)
    {
        myHasStartedTransformation = Standard_False;
    }
}

//=======================================================================
//function : SetZoomPersistence
//purpose  :
//=======================================================================
void Core3DManipulator::SetZoomPersistence (const Standard_Boolean theToEnable)
{
    if (myIsZoomPersistentMode != theToEnable)
    {
        SetToUpdate();
    }
    
    myIsZoomPersistentMode = theToEnable;
    
    if (!theToEnable)
    {
        setTransformPersistence (Handle(Graphic3d_TransformPers)());
    }
    
    updateTransformation();
}

//=======================================================================
//function : SetTransformPersistence
//purpose  :
//=======================================================================
void Core3DManipulator::SetTransformPersistence (const Handle(Graphic3d_TransformPers)& theTrsfPers)
{
    Standard_ASSERT_RETURN (!myIsZoomPersistentMode,
                            "Core3DManipulator::SetTransformPersistence: "
                            "Custom settings are not allowed by this class in ZoomPersistence mode",);
    
    setTransformPersistence (theTrsfPers);
}

//=======================================================================
//function : setTransformPersistence
//purpose  :
//=======================================================================
void Core3DManipulator::setTransformPersistence (const Handle(Graphic3d_TransformPers)& theTrsfPers)
{
    AIS_InteractiveObject::SetTransformPersistence (theTrsfPers);
    
    for (Standard_Integer anIt = 0; anIt < 3; ++anIt)
    {
        myAxes[anIt].SetTransformPersistence (theTrsfPers);
    }
}

//=======================================================================
//function : setLocalTransformation
//purpose  :
//=======================================================================
void Core3DManipulator::setLocalTransformation (const Handle(TopLoc_Datum3D)& /*theTrsf*/)
{
    Standard_ASSERT_INVOKE ("Core3DManipulator::setLocalTransformation: "
                            "Custom transformation is not supported by this class");
}

//=======================================================================
//function : Compute
//purpose  :
//=======================================================================
void Core3DManipulator::Compute (const Handle(PrsMgr_PresentationManager)& thePrsMgr,
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
		Standard_Real radius = myAxes[0].AxisRadius() * 1.5f;
		myCenter.Init (radius, gp::Origin());
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
        myAxes[anIt].Compute (thePrsMgr, thePrs, anAspectAx);
        myAxes[anIt].SetTransformPersistence (TransformPersistence());
    }
    
    updateTransformation();
}

//=======================================================================
//function : HilightSelected
//purpose  :
//=======================================================================
void Core3DManipulator::HilightSelected (const Handle(PrsMgr_PresentationManager)& thePM,
                                         const SelectMgr_SequenceOfOwner& theSeq)
{
    if (theSeq.IsEmpty())
    {
        return;
    }
    
    if (myIsActivationOnDetection)
    {
        return;
    }
    
    if (!theSeq (1)->IsKind (STANDARD_TYPE (AIS_ManipulatorOwner)))
    {
        thePM->Color (this, GetContext()->HighlightStyle(), 0);
        return;
    }
    
    Handle(AIS_ManipulatorOwner) anOwner = Handle(AIS_ManipulatorOwner)::DownCast (theSeq (1));
    myHighlightAspect->Aspect()->SetInteriorColor (GetContext()->HighlightStyle()->Color());
    Handle(Graphic3d_Group) aGroup = getGroup (anOwner->Index(), anOwner->Mode());
    if (aGroup.IsNull())
    {
        return;
    }
    
    if (anOwner->Mode() == AIS_MM_TranslationPlane)
    {
        myDraggerHighlight->SetColor(myAxes[anOwner->Index()].Color());
        aGroup->SetGroupPrimitivesAspect(myDraggerHighlight->Aspect());
    }
    else
        aGroup->SetGroupPrimitivesAspect(myHighlightAspect->Aspect());
    
    myCurrentIndex = anOwner->Index();
    myCurrentMode = anOwner->Mode();
}

//=======================================================================
//function : ClearSelected
//purpose  :
//=======================================================================
void Core3DManipulator::ClearSelected()
{
    DeactivateCurrentMode();
}

//=======================================================================
//function : HilightOwnerWithColor
//purpose  :
//=======================================================================
void Core3DManipulator::HilightOwnerWithColor (const Handle(PrsMgr_PresentationManager)& thePM,
                                               const Handle(Prs3d_Drawer)& theStyle,
                                               const Handle(SelectMgr_EntityOwner)& theOwner)
{
    Handle(AIS_ManipulatorOwner) anOwner = Handle(AIS_ManipulatorOwner)::DownCast (theOwner);
    Handle(Prs3d_Presentation) aPresentation = getHighlightPresentation (anOwner);
    if (aPresentation.IsNull())
    {
        return;
    }
    
    aPresentation->CStructure()->ViewAffinity = myViewAffinity;
    
    if (anOwner->Mode() == AIS_MM_TranslationPlane)
    {
        Handle(Prs3d_Drawer) aStyle = new Prs3d_Drawer();
        aStyle->SetColor (myAxes[anOwner->Index()].Color());
        aStyle->SetTransparency (0.5);
        aPresentation->Highlight (aStyle);
    }
    else
    {
        aPresentation->Highlight (theStyle);
    }
    
    for (Graphic3d_SequenceOfGroup::Iterator aGroupIter (aPresentation->Groups());
         aGroupIter.More(); aGroupIter.Next())
    {
        Handle(Graphic3d_Group)& aGrp = aGroupIter.ChangeValue();
        if (!aGrp.IsNull())
        {
            aGrp->SetGroupPrimitivesAspect (myHighlightAspect->Aspect());
        }
    }
    aPresentation->SetZLayer (Graphic3d_ZLayerId_Topmost);
    thePM->AddToImmediateList (aPresentation);
    
    if (myIsActivationOnDetection)
    {
        if (HasActiveMode())
        {
            DeactivateCurrentMode();
        }
        
        myCurrentIndex = anOwner->Index();
        myCurrentMode = anOwner->Mode();
    }
}

//=======================================================================
//function : ComputeSelection
//purpose  :
//=======================================================================
void Core3DManipulator::ComputeSelection (const Handle(SelectMgr_Selection)& theSelection,
                                          const Standard_Integer theMode)
{
    //Check mode
    const AIS_ManipulatorMode aMode = (AIS_ManipulatorMode) theMode;
    if (aMode == AIS_MM_None)
    {
        return;
    }
    Handle(SelectMgr_EntityOwner) anOwner;
    
    // Sensitivity calculation for manipulator parts allows to avoid
    // overlapping of sensitive areas when size of manipulator is small.
    // Sensitivity is calculated relative to the default size of the manipulator (100.0f).
    const Standard_ShortReal aSensitivityCoef = myAxes[0].Size() / 100.0f;
    const Standard_Integer aHighSensitivity = Max (Min (RealToInt (aSensitivityCoef * 15), 15), 3); // clamp sensitivity within range [3, 15]
    const Standard_Integer aLowSensitivity  = Max (Min (RealToInt (aSensitivityCoef * 10), 10), 2); // clamp sensitivity within range [2, 10]
    
    switch (aMode)
    {
        case AIS_MM_Translation:
        {
            for (Standard_Integer anIt = 0; anIt < 3; ++anIt)
            {
                if (!myAxes[anIt].HasTranslation())
                {
                    continue;
                }
                const Axis& anAxis = myAxes[anIt];
                anOwner = new AIS_ManipulatorOwner(this, anIt, AIS_MM_Translation, 9);
                
                // define sensitivity by line
                Handle(Select3D_SensitiveSegment) aLine = new Select3D_SensitiveSegment(anOwner, gp::Origin(), anAxis.TranslatorTipPosition());
                aLine->SetSensitivityFactor (aHighSensitivity);
                theSelection->Add (aLine);
                
                // enlarge sensitivity by triangulation
                Handle(Select3D_SensitivePrimitiveArray) aTri = new Select3D_SensitivePrimitiveArray(anOwner);
                aTri->InitTriangulation (anAxis.TriangleArrayF()->Attributes(), anAxis.TriangleArrayF()->Indices(), TopLoc_Location());
                theSelection->Add (aTri);
				
				if (!anAxis.TriangleArrayB().IsNull()) {
					// enlarge sensitivity by triangulation
					Handle(Select3D_SensitivePrimitiveArray) aTri = new Select3D_SensitivePrimitiveArray(anOwner);
					aTri->InitTriangulation (anAxis.TriangleArrayB()->Attributes(), anAxis.TriangleArrayB()->Indices(), TopLoc_Location());
					theSelection->Add (aTri);
				}
            }
            break;
        }
        case AIS_MM_Rotation:
        {
            for (Standard_Integer anIt = 0; anIt < 3; ++anIt)
            {
                if (!myAxes[anIt].HasRotation())
                {
                    continue;
                }
                const Axis& anAxis = myAxes[anIt];
                anOwner = new AIS_ManipulatorOwner(this, anIt, AIS_MM_Rotation, 9);
                
                // define sensitivity by circle
                const gp_Circ aGeomCircle (gp_Ax2(gp::Origin(), anAxis.ReferenceAxis().Direction()), anAxis.RotatorDiskRadius());
                Handle(Select3D_SensitiveCircle) aCircle = new ManipSensCircle(anOwner, aGeomCircle);
                aCircle->SetSensitivityFactor (aLowSensitivity);
                theSelection->Add(aCircle);
                // enlarge sensitivity by triangulation
                Handle(Select3D_SensitiveTriangulation) aTri = new ManipSensTriangulation(anOwner, myAxes[anIt].RotatorDisk().Triangulation(), anAxis.ReferenceAxis().Direction());
                theSelection->Add (aTri);
            }
            break;
        }
        case AIS_MM_Scaling:
        {
            for (Standard_Integer anIt = 0; anIt < 3; ++anIt)
            {
                if (!myAxes[anIt].HasScaling())
                {
                    continue;
                }
                anOwner = new AIS_ManipulatorOwner(this, anIt, AIS_MM_Scaling, 9);
                
                // define sensitivity by point
                Handle(Select3D_SensitivePoint) aPnt = new Select3D_SensitivePoint(anOwner, myAxes[anIt].ScalerCubePosition());
                aPnt->SetSensitivityFactor (aHighSensitivity);
                theSelection->Add (aPnt);
                // enlarge sensitivity by triangulation
                Handle(Select3D_SensitiveTriangulation) aTri = new Select3D_SensitiveTriangulation(anOwner, myAxes[anIt].ScalerCube().Triangulation(), TopLoc_Location(), Standard_True);
                theSelection->Add (aTri);
            }
            break;
        }
		case AIS_MM_ScalingUniform:
		{
			for (Standard_Integer anIt = 1; anIt < 2; ++anIt)
			{
				if (!myAxes[anIt].HasScalingUniform())
				{
					continue;
				}
				anOwner = new AIS_ManipulatorOwner(this, anIt, AIS_MM_ScalingUniform, 9);
				
				// define sensitivity by point
				Handle(Select3D_SensitivePoint) aPnt = new Select3D_SensitivePoint(anOwner, myAxes[anIt].ScalerSphereUniformPosition());
				aPnt->SetSensitivityFactor (aHighSensitivity);
				theSelection->Add (aPnt);
				// enlarge sensitivity by triangulation
				Handle(Select3D_SensitiveTriangulation) aTri = new Select3D_SensitiveTriangulation(anOwner, myAxes[anIt].ScalerSphereUniform().Triangulation(), TopLoc_Location(), Standard_True);
				theSelection->Add (aTri);
			}
			break;
		}
		case AIS_MM_MirroringPlanePos:
		{
			for (Standard_Integer anIt = 0; anIt < 3; ++anIt)
			{
				if (!myAxes[anIt].HasMirroringPos())
				{
					continue;
				}
				anOwner = new AIS_ManipulatorOwner(this, anIt, AIS_MM_MirroringPlanePos, 9);
				
				// define sensitivity by point
				Handle(Select3D_SensitivePoint) aPntPos = new Select3D_SensitivePoint(anOwner, myAxes[anIt].MirroringPlanePosPosition());
				aPntPos->SetSensitivityFactor (aHighSensitivity);
				theSelection->Add (aPntPos);
				// enlarge sensitivity by triangulation
				Handle(Select3D_SensitiveTriangulation) aTriPos = new Select3D_SensitiveTriangulation(anOwner, myAxes[anIt].MirroringPlanePos().Triangulation(), TopLoc_Location(), Standard_True);
				theSelection->Add (aTriPos);
			}
			break;
		}
		case AIS_MM_MirroringPlaneNeg:
		{
			for (Standard_Integer anIt = 0; anIt < 3; ++anIt)
			{
				if (!myAxes[anIt].HasMirroringNeg())
				{
					continue;
				}
				anOwner = new AIS_ManipulatorOwner(this, anIt, AIS_MM_MirroringPlaneNeg, 9);
				
				// define sensitivity by point
				Handle(Select3D_SensitivePoint) aPntNeg = new Select3D_SensitivePoint(anOwner, myAxes[anIt].MirroringPlaneNegPosition());
				aPntNeg->SetSensitivityFactor (aHighSensitivity);
				theSelection->Add (aPntNeg);
				// enlarge sensitivity by triangulation
				Handle(Select3D_SensitiveTriangulation) aTriNeg = new Select3D_SensitiveTriangulation(anOwner, myAxes[anIt].MirroringPlaneNeg().Triangulation(), TopLoc_Location(), Standard_True);
				theSelection->Add (aTriNeg);
			}
			break;
		}
        case AIS_MM_TranslationPlane:
        {
            for (Standard_Integer anIt = 0; anIt < 3; ++anIt)
            {
                if (!myAxes[anIt].HasDragging())
                {
                    continue;
                }
                anOwner = new AIS_ManipulatorOwner(this, anIt, AIS_MM_TranslationPlane, 9);
                
                // define sensitivity by two crossed lines
                Standard_Real aSensitivityOffset = ZoomPersistence() ? aHighSensitivity * (0.5 + M_SQRT2) : 0.0;
                gp_Pnt aP1 = myAxes[((anIt + 1) % 3)].TranslatorTipPosition().Translated (myAxes[((anIt + 2) % 3)].ReferenceAxis().Direction().XYZ() * aSensitivityOffset);
                gp_Pnt aP2 = myAxes[((anIt + 2) % 3)].TranslatorTipPosition().Translated (myAxes[((anIt + 1) % 3)].ReferenceAxis().Direction().XYZ() * aSensitivityOffset);
                gp_XYZ aMidP = (aP1.XYZ() + aP2.XYZ()) / 2.0;
                gp_XYZ anOrig = aMidP.Normalized().Multiplied (aSensitivityOffset);
                
                Handle(Select3D_SensitiveSegment) aLine1 = new Select3D_SensitiveSegment(anOwner, aP1, aP2);
                aLine1->SetSensitivityFactor(aLowSensitivity);
                theSelection->Add (aLine1);
                Handle(Select3D_SensitiveSegment) aLine2 = new Select3D_SensitiveSegment(anOwner, anOrig, aMidP);
                aLine2->SetSensitivityFactor (aLowSensitivity);
                theSelection->Add (aLine2);
                
                // enlarge sensitivity by triangulation
                Handle(Select3D_SensitiveTriangulation) aTri = new Select3D_SensitiveTriangulation(anOwner, myAxes[anIt].DraggerSector().Triangulation(), TopLoc_Location(), Standard_True);
                theSelection->Add (aTri);
            }
            break;
        }
        default:
        {
            anOwner = new SelectMgr_EntityOwner(this, 5);
            break;
        }
    }
}

//=======================================================================
//class    : Disk
//function : Init
//purpose  :
//=======================================================================
void Core3DManipulator::Disk::Init (const Standard_ShortReal theInnerRadius,
                                    const Standard_ShortReal theOuterRadius,
                                    const gp_Ax1& thePosition,
                                    const Standard_Integer theSlicesNb,
                                    const Standard_Integer theStacksNb,
									bool invertedAngleRange)
{
    myPosition = thePosition;
    myInnerRad = theInnerRadius;
	myOuterRad = theOuterRadius;
    
    Prs3d_ToolDisk aTool (theInnerRadius, theOuterRadius, theSlicesNb, theStacksNb);
	if (invertedAngleRange)
		aTool.SetAngleRange(myAnglePad + M_PI + M_PI_2, M_PI * 2.0f - myAnglePad);
	else
		aTool.SetAngleRange(myAnglePad, M_PI_2 - myAnglePad);
	
    gp_Ax3 aSystem (myPosition.Location(), myPosition.Direction());
    gp_Trsf aTrsf;
    aTrsf.SetTransformation (aSystem, gp_Ax3());
    myArray = aTool.CreateTriangulation (aTrsf);
    myTriangulation = aTool.CreatePolyTriangulation (aTrsf);
}

//=======================================================================
//class    : Sphere
//function : Init
//purpose  :
//=======================================================================
void Core3DManipulator::Sphere::Init (const Standard_ShortReal theRadius,
                                      const gp_Pnt& thePosition,
                                      const Standard_Integer theSlicesNb,
                                      const Standard_Integer theStacksNb)
{
    myPosition = thePosition;
    myRadius = theRadius;
    
    Prs3d_ToolSphere aTool (theRadius, theSlicesNb, theStacksNb);
    gp_Trsf aTrsf;
    aTrsf.SetTranslation (gp_Vec(gp::Origin(), thePosition));
    myArray = aTool.CreateTriangulation (aTrsf);
    myTriangulation = aTool.CreatePolyTriangulation (aTrsf);
}

//=======================================================================
//class    : Plane
//function : Init
//purpose  :
//=======================================================================
void Core3DManipulator::Plane::Init (const gp_Ax1& thePosition, const Standard_ShortReal theSize, const Standard_ShortReal planeSize)
{
	myArray = new Graphic3d_ArrayOfTriangles (2 * 3, 0, Standard_True);
	
	Poly_Array1OfTriangle aPolyTriangles (1, 2);
	TColgp_Array1OfPnt aPoints (1, 6);
	NCollection_Array1<gp_Dir> aNormals (1, 2);
	myTriangulation = new Poly_Triangulation (aPoints, aPolyTriangles);
	
	gp_Ax2 aPln (thePosition.Location(), thePosition.Direction());
	gp_Pnt aBottomLeft = thePosition.Location().XYZ() - aPln.XDirection().XYZ() * planeSize * 0.5 - aPln.YDirection().XYZ() * planeSize * 0.5;
	gp_Pnt aV2 = aBottomLeft.XYZ() + aPln.YDirection().XYZ() * planeSize;
	gp_Pnt aV3 = aBottomLeft.XYZ() + aPln.YDirection().XYZ() * planeSize + aPln.XDirection().XYZ() * planeSize;
	gp_Pnt aV4 = aBottomLeft.XYZ() + aPln.XDirection().XYZ() * planeSize;
	
	addTriangle (0, aBottomLeft, aV2, aV3, -thePosition.Direction());
	addTriangle (1, aBottomLeft, aV3, aV4, -thePosition.Direction());
	
}

//=======================================================================
//class    : Cube
//function : Init
//purpose  :
//=======================================================================
void Core3DManipulator::Cube::Init (const gp_Ax1& thePosition, const Standard_ShortReal theSize)
{
    myArray = new Graphic3d_ArrayOfTriangles (12 * 3, 0, Standard_True);
    
    Poly_Array1OfTriangle aPolyTriangles (1, 12);
    TColgp_Array1OfPnt aPoints (1, 36);
    NCollection_Array1<gp_Dir> aNormals (1, 12);
    myTriangulation = new Poly_Triangulation (aPoints, aPolyTriangles);
    
    gp_Ax2 aPln (thePosition.Location(), thePosition.Direction());
    gp_Pnt aBottomLeft = thePosition.Location().XYZ() - aPln.XDirection().XYZ() * theSize * 0.5 - aPln.YDirection().XYZ() * theSize * 0.5;
    gp_Pnt aV2 = aBottomLeft.XYZ() + aPln.YDirection().XYZ() * theSize;
    gp_Pnt aV3 = aBottomLeft.XYZ() + aPln.YDirection().XYZ() * theSize + aPln.XDirection().XYZ() * theSize;
    gp_Pnt aV4 = aBottomLeft.XYZ() + aPln.XDirection().XYZ() * theSize;
    gp_Pnt aTopRight = thePosition.Location().XYZ() + thePosition.Direction().XYZ() * theSize
    + aPln.XDirection().XYZ() * theSize * 0.5 + aPln.YDirection().XYZ() * theSize * 0.5;
    gp_Pnt aV5 = aTopRight.XYZ() - aPln.YDirection().XYZ() * theSize;
    gp_Pnt aV6 = aTopRight.XYZ() - aPln.YDirection().XYZ() * theSize - aPln.XDirection().XYZ() * theSize;
    gp_Pnt aV7 = aTopRight.XYZ() - aPln.XDirection().XYZ() * theSize;
    
    gp_Dir aRight ((gp_Vec(aTopRight, aV7) ^ gp_Vec(aTopRight, aV2)).XYZ());
    gp_Dir aFront ((gp_Vec(aV3, aV4) ^ gp_Vec(aV3, aV5)).XYZ());
    
    // Bottom
    addTriangle (0, aBottomLeft, aV2, aV3, -thePosition.Direction());
    addTriangle (1, aBottomLeft, aV3, aV4, -thePosition.Direction());
    
    // Front
    addTriangle (2, aV3, aV5, aV4, -aFront);
    addTriangle (3, aV3, aTopRight, aV5, -aFront);
    
    // Back
    addTriangle (4, aBottomLeft, aV7, aV2, aFront);
    addTriangle (5, aBottomLeft, aV6, aV7, aFront);
    
    // aTop
    addTriangle (6, aV7, aV6, aV5, thePosition.Direction());
    addTriangle (7, aTopRight, aV7, aV5, thePosition.Direction());
    
    // Left
    addTriangle (8, aV6, aV4, aV5, aRight);
    addTriangle (9, aBottomLeft, aV4, aV6, aRight);
    
    // Right
    addTriangle (10, aV3, aV7, aTopRight, -aRight);
    addTriangle (11, aV3, aV2, aV7, -aRight);
}

//=======================================================================
//class    : Triangles
//function : addTriangle
//purpose  :
//=======================================================================
void Core3DManipulator::Triangles::addTriangle (const Standard_Integer theIndex,
                                           const gp_Pnt& theP1, const gp_Pnt& theP2, const gp_Pnt& theP3,
                                           const gp_Dir& theNormal)
{
    myTriangulation->SetNode (theIndex * 3 + 1, theP1);
    myTriangulation->SetNode (theIndex * 3 + 2, theP2);
    myTriangulation->SetNode (theIndex * 3 + 3, theP3);
    
    myTriangulation->SetTriangle (theIndex + 1, Poly_Triangle (theIndex * 3 + 1, theIndex * 3 + 2, theIndex * 3 + 3));
    myArray->AddVertex (theP1, theNormal);
    myArray->AddVertex (theP2, theNormal);
    myArray->AddVertex (theP3, theNormal);
}

//=======================================================================
//class    : Sector
//function : Init
//purpose  :
//=======================================================================
void Core3DManipulator::Sector::Init (const Standard_ShortReal theRadius,
                                      const gp_Ax1&            thePosition,
                                      const gp_Dir&            theXDirection,
                                      const Standard_Integer   theSlicesNb,
                                      const Standard_Integer   theStacksNb)
{
    Prs3d_ToolSector aTool(theRadius, theSlicesNb, theStacksNb);
    gp_Ax3 aSystem(thePosition.Location(), thePosition.Direction(), theXDirection);
    gp_Trsf aTrsf;
    aTrsf.SetTransformation(aSystem, gp_Ax3());
    myArray = aTool.CreateTriangulation (aTrsf);
    myTriangulation = aTool.CreatePolyTriangulation (aTrsf);
}

//=======================================================================
//class    : Axis
//function : Constructor
//purpose  :
//=======================================================================
Core3DManipulator::Axis::Axis (const gp_Ax1& theAxis,
                               const Quantity_Color& theColor,
                               const Standard_ShortReal theLength)
: myReferenceAxis (theAxis),
myPosition (theAxis),
myColor (theColor),
myHasTranslation (Standard_True),
myLength (theLength),
myAxisRadius (0.5f),
myHasScaling (Standard_True),
myHasScalingUniform (Standard_True),
myBoxSize (2.0f),
myHasMirroringPos(Standard_True),
myHasMirroringNeg(Standard_True),
myMirroringPlaneSize(20.0f),
myHasRotation (Standard_True),
myInnerRadius (myLength + myBoxSize),
myDiskThickness (myBoxSize * 0.5f),
myIndent (0.2f),
myHasDragging(Standard_True),
myFacettesNumber (20),
myCircleRadius (myLength + myBoxSize + myBoxSize * 0.5f * 0.5f)
{
    //
}

//=======================================================================
//class    : Axis
//function : Compute
//purpose  :
//=======================================================================

void Core3DManipulator::Axis::Compute (const Handle(PrsMgr_PresentationManager)& thePrsMgr,
                                       const Handle(Prs3d_Presentation)& thePrs,
                                       const Handle(Prs3d_ShadingAspect)& theAspect,
									   const Standard_Boolean biDirected)
{
    if (myHasTranslation)
    {
        const Standard_Real anArrowLength   = 0.25 * myLength;
        const Standard_Real aCylinderLength = myLength - anArrowLength;
        myArrowTipPos = gp_Pnt (0.0, 0.0, 0.0).Translated (myReferenceAxis.Direction().XYZ() * aCylinderLength);
        
        myTriangleArrayF = Prs3d_Arrow::DrawShaded (gp_Ax1(gp::Origin(), myReferenceAxis.Direction()),
                                                   myAxisRadius,
                                                   myLength,
                                                   myAxisRadius * 1.5,
                                                   anArrowLength,
                                                   myFacettesNumber);
		
		if (biDirected) {
			myTriangleArrayB = Prs3d_Arrow::DrawShaded (gp_Ax1(gp::Origin(), -myReferenceAxis.Direction()),
														myAxisRadius,
														myLength,
														myAxisRadius * 1.5,
														anArrowLength,
														myFacettesNumber);
		}
		

        myTranslatorGroup = thePrs->NewGroup();
        myTranslatorGroup->SetClosed (true);
        myTranslatorGroup->SetGroupPrimitivesAspect (theAspect->Aspect());
        myTranslatorGroup->AddPrimitiveArray (myTriangleArrayF);
		if (biDirected) myTranslatorGroup->AddPrimitiveArray (myTriangleArrayB);
        
        if (myHighlightTranslator.IsNull())
        {
            myHighlightTranslator = new Prs3d_Presentation (thePrsMgr->StructureManager());
        }
        else
        {
            myHighlightTranslator->Clear();
        }
        {
            Handle(Graphic3d_Group) aGroup = myHighlightTranslator->CurrentGroup();
            aGroup->SetGroupPrimitivesAspect (theAspect->Aspect());
            aGroup->AddPrimitiveArray (myTriangleArrayF);
			if (biDirected) aGroup->AddPrimitiveArray (myTriangleArrayB);
        }
    }
    
    if (myHasScaling)
    {
        myCubePos = myReferenceAxis.Direction().XYZ() * (myLength + myIndent);
        myCube.Init (gp_Ax1 (myCubePos, myReferenceAxis.Direction()), myBoxSize);
        
        myScalerGroup = thePrs->NewGroup();
        myScalerGroup->SetClosed (true);
        myScalerGroup->SetGroupPrimitivesAspect (theAspect->Aspect());
        myScalerGroup->AddPrimitiveArray (myCube.Array());
        
        if (myHighlightScaler.IsNull())
        {
            myHighlightScaler = new Prs3d_Presentation (thePrsMgr->StructureManager());
        }
        else
        {
            myHighlightScaler->Clear();
        }
        {
            Handle(Graphic3d_Group) aGroup = myHighlightScaler->CurrentGroup();
            aGroup->SetGroupPrimitivesAspect (theAspect->Aspect());
            aGroup->AddPrimitiveArray (myCube.Array());
        }
    }
	
	if (myHasScalingUniform)
	{
		if (myReferenceAxis.Direction().Y() > 0) {
			mySphereUniformPos = myReferenceAxis.Direction().XYZ() * (myLength * 1.8f + myIndent);//myReferenceAxis.Direction().XYZ() * (myLength + myIndent);
			
			Standard_Real radius = AxisRadius() * 3.0f;
			mySphereUniform.Init (radius, mySphereUniformPos);
			
			myScalerUniformGroup = thePrs->NewGroup();
			myScalerUniformGroup->SetClosed (true);
			myScalerUniformGroup->SetGroupPrimitivesAspect (theAspect->Aspect());
			myScalerUniformGroup->AddPrimitiveArray (mySphereUniform.Array());
			
			if (myHighlightScalerUniform.IsNull()) {
				myHighlightScalerUniform = new Prs3d_Presentation (thePrsMgr->StructureManager());
			}
			else {
				myHighlightScalerUniform->Clear();
			}
			{
				Handle(Graphic3d_Group) aGroup = myHighlightScalerUniform->CurrentGroup();
				aGroup->SetGroupPrimitivesAspect (theAspect->Aspect());
				aGroup->AddPrimitiveArray (mySphereUniform.Array());
			}
		}
	}
    
    if (myHasRotation)
    {
        myCircleRadius = myInnerRadius + myIndent * 2 + myDiskThickness * 0.5f;
		myCircle.Init (myInnerRadius + myIndent * 2, myInnerRadius + myDiskThickness + myIndent * 2, gp_Ax1(gp::Origin(), myReferenceAxis.Direction()), myFacettesNumber * 2, myFacettesNumber * 2, myReferenceAxis.Direction().X() > 0);
        myRotatorGroup = thePrs->NewGroup ();
        myRotatorGroup->SetGroupPrimitivesAspect (theAspect->Aspect());
        myRotatorGroup->AddPrimitiveArray (myCircle.Array());
        
        if (myHighlightRotator.IsNull())
        {
            myHighlightRotator = new Prs3d_Presentation (thePrsMgr->StructureManager());
        }
        else
        {
            myHighlightRotator->Clear();
        }
        {
            Handle(Graphic3d_Group) aGroup = myHighlightRotator->CurrentGroup();
            aGroup->SetGroupPrimitivesAspect (theAspect->Aspect());
            aGroup->AddPrimitiveArray (myCircle.Array());
        }
    }
    
    if (myHasDragging)
    {
        gp_Dir aXDirection;
        if (myReferenceAxis.Direction().X() > 0)
            aXDirection = gp::DY();
        else if (myReferenceAxis.Direction().Y() > 0)
            aXDirection = gp::DZ();
        else
            aXDirection = gp::DX();
        
        mySector.Init(myInnerRadius + myIndent * 2, gp_Ax1(gp::Origin(), myReferenceAxis.Direction()), aXDirection, myFacettesNumber * 2);
        myDraggerGroup = thePrs->NewGroup();
        
        Handle(Graphic3d_AspectFillArea3d) aFillArea = new Graphic3d_AspectFillArea3d();
        myDraggerGroup->SetGroupPrimitivesAspect(aFillArea);
        myDraggerGroup->AddPrimitiveArray(mySector.Array());
        
        if (myHighlightDragger.IsNull())
        {
            myHighlightDragger = new Prs3d_Presentation(thePrsMgr->StructureManager());
        }
        else
        {
            myHighlightDragger->Clear();
        }
        {
            Handle(Graphic3d_Group) aGroup = myHighlightDragger->CurrentGroup();
            aGroup->SetGroupPrimitivesAspect(aFillArea);
            aGroup->AddPrimitiveArray(mySector.Array());
        }
    }
	
	if (myHasMirroringPos)
	{
		myPlanePosPos = myReferenceAxis.Direction().XYZ() * (myLength + myIndent);
		myPlanePos.Init (gp_Ax1 (myPlanePosPos, myReferenceAxis.Direction()), (myLength + myIndent), myMirroringPlaneSize);
		
		Handle(Prs3d_ShadingAspect) mirrAspect = new Prs3d_ShadingAspect(theAspect->Aspect());
		mirrAspect->SetTransparency(0.25f);
		myMirroringGroupPos = thePrs->NewGroup();
		myMirroringGroupPos->SetClosed (false);
		myMirroringGroupPos->SetGroupPrimitivesAspect (mirrAspect->Aspect());
		myMirroringGroupPos->AddPrimitiveArray (myPlanePos.Array());
		
		if (myHighlightMirroringPos.IsNull())
		{
			myHighlightMirroringPos = new Prs3d_Presentation (thePrsMgr->StructureManager());
		}
		else
		{
			myHighlightMirroringPos->Clear();
		}
		
		{
			Handle(Graphic3d_Group) aGroupPos = myHighlightMirroringPos->CurrentGroup();
			aGroupPos->SetGroupPrimitivesAspect (mirrAspect->Aspect());
			aGroupPos->AddPrimitiveArray (myPlanePos.Array());
		}

	}
	
	if (myHasMirroringNeg)
	{
		myPlanePosNeg = myReferenceAxis.Direction().Reversed().XYZ() * (myLength + myIndent);
		myPlaneNeg.Init (gp_Ax1 (myPlanePosNeg, myReferenceAxis.Direction().Reversed()), (myLength + myIndent), myMirroringPlaneSize);
		
		myMirroringGroupNeg = thePrs->NewGroup();
		myMirroringGroupNeg->SetClosed (false);
		myMirroringGroupNeg->SetGroupPrimitivesAspect (theAspect->Aspect());
		myMirroringGroupNeg->AddPrimitiveArray (myPlaneNeg.Array());
		
		if (myHighlightMirroringNeg.IsNull())
		{
			myHighlightMirroringNeg = new Prs3d_Presentation (thePrsMgr->StructureManager());
		}
		else
		{
			myHighlightMirroringNeg->Clear();
		}
		
		{
			Handle(Graphic3d_Group) aGroupNeg = myHighlightMirroringNeg->CurrentGroup();
			aGroupNeg->SetGroupPrimitivesAspect (theAspect->Aspect());
			aGroupNeg->AddPrimitiveArray (myPlaneNeg.Array());
		}
		
	}
}
