#include <TDF_TagSource.hxx>
#include "RetainedFilletCandidates.hxx"
#include "SavedCutSourceEdit.hxx"
#include "PartBooleanOwner.hxx"
#include "PartBooleanBuild.hxx"
#include "PartBooleanCorrespondence.hxx"
#include "PartBooleanRebuild.hxx"
#include "RetainedRecipeAdmission.hxx"
#if DEBUG
#include "PartBooleanCodecProbe.hxx"
#endif
#include "RetainedFeatureOwner.hxx"
#if DEBUG
#include "RetainedFeatureRegistryProbe.hxx"
#endif
#include <TNaming_Builder.hxx>
#include <BRepTools.hxx>
#include <TDF_Delta.hxx>
#include <TDF_DeltaList.hxx>

#if DEBUG // Cut475 phase diagnostics only
#include <cstdio>
#include <atomic>
namespace {
// Thread-safe bounded diagnostics; literals/integers, no document or user text.
void Cut475Trace(const char* stage, int detail=-1) noexcept {
    static std::atomic<unsigned> emitted{0};
    unsigned count=emitted.load(std::memory_order_relaxed);
    while(count<4096){
        if(emitted.compare_exchange_weak(count,count+1,std::memory_order_relaxed)){
            std::fprintf(stderr,"[Cut475] %s detail=%d\n",stage,detail);break;
        }
    }
}
struct Cut475Scope {
    const char* phase;
    ~Cut475Scope() noexcept {if(phase)Cut475Trace(phase);}
};
}
#endif // Cut475 phase diagnostics only
#include "NativeObservedApplication.hxx"
#include "NativeMeshVertexMove.hxx"
#include "NativeMeshRegionExtrude.hxx"
#include "NativeMeshRegionInset.hxx"
#include "NativeMeshWindingCandidate.hxx"
#if DEBUG
#include "NativeLiveTransactionObserverProbe.hxx"
#endif
#include "../Scene/MikkTangentSpace.hpp"
#include <RWMesh_FaceIterator.hxx>
#include "CoherentMeshUVAtlas.hpp"
#include <TDataStd_UAttribute.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_GraphNode.hxx>
#include <TDataStd_Name.hxx>
// Copyright (c) 2017 OPEN CASCADE SAS
//
// This file is part of the examples of the Open CASCADE Technology software library.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE

#import <Foundation/Foundation.h>
#import <ImageIO/ImageIO.h>
#import <CommonCrypto/CommonDigest.h>
#import <TargetConditionals.h>

#if DEBUG && TARGET_OS_IOS
#import "../UI/Core3DViewController.h"
#import "GLViewController.h"
#include "../UI/ShellOperationController.hpp"
#include <AIS_ListIteratorOfListOfInteractive.hxx>
#include <Precision.hxx>
#include <Standard_ErrorHandler.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#endif

#include "OcctDocument.h"
#include "NativeDocumentSession.hxx"
#include <Standard_ProgramError.hxx>
#include "SavedCutSourceDetachedWork.hxx"
#include "SavedProgramSourceDetachedWork.hxx"
#include "NativeModelingReceipt.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <cstring>
#include <map>
#include <set>
#include <locale>
#include <sstream>
#include <TDF_AttributeIterator.hxx>
#include "AuthoredFrameAttributeID.hxx"
#include "NativeAuthoredFrameGeometry.hxx"
#include <TDataStd_ByteArray.hxx>
#include <TDF_AttributeIterator.hxx>
#include "Core3DBoundedAuthoredFrameDriver.hxx"
#include "ReceiptFramedTraversal.hxx"
#include "ReceiptCatalogBinaryDriver.hxx"
#include "RetainedSolidBinaryDriver.hxx"
#include "CompositeRecipeBinaryDriver.hxx"
#include "SpatialSweepG0Transaction.hxx"
#include "BoundedCurveBinaryDriver.hxx"
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <Precision.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include "ProfilePersistence.hxx"
#include "ProfileCurveFace.hxx"
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepLib.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <Standard_ErrorHandler.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <algorithm>
#include <atomic>
#if DEBUG
#include "RetainedPartBoolean.hxx"
#include "ReceiptFramingProbe.hxx"
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Standard_IStream.hxx>
#include <PCDM_ReadWriter.hxx>
#include <Storage_Data.hxx>
#include <TDocStd_PathParser.hxx>
#include <TopoDS_Compound.hxx>
#include <fstream>
#include <iterator>
#endif

#if DEBUG && TARGET_OS_IOS
namespace {

std::shared_ptr<core3d::Core3DViewer>
P4RefusalFixtureViewer(Core3DViewController *owner) noexcept
{
    if (owner == nil) return {};
    @try {
        id candidate = [owner valueForKey:@"glController"];
        if (![candidate isKindOfClass:GLViewController.class]) return {};
        return [(GLViewController *)candidate viewer];
    } @catch (NSException *exception) {
        (void)exception;
        return {};
    }
}

bool P4ReferenceAxesMatch(
    const OcctReferenceAxis& left,
    const OcctReferenceAxis& right) noexcept
{
    return left.pivotSpace == right.pivotSpace
        && left.directionSpace == right.directionSpace
        && left.pivot.IsEqual(right.pivot, 0.0)
        && left.direction.IsEqual(right.direction, 0.0);
}

NSDictionary<NSString *, NSNumber *> *
P4ReferenceAxisDictionary(
    const OcctReferenceAxisReadState state,
    const OcctReferenceAxis& axis)
{
    return @{
        @"readState": @(static_cast<Standard_Integer>(state)),
        @"pivotSpace": @(static_cast<Standard_Integer>(axis.pivotSpace)),
        @"pivotX": @(axis.pivot.X()),
        @"pivotY": @(axis.pivot.Y()),
        @"pivotZ": @(axis.pivot.Z()),
        @"directionSpace": @(
            static_cast<Standard_Integer>(axis.directionSpace)),
        @"directionX": @(axis.direction.X()),
        @"directionY": @(axis.direction.Y()),
        @"directionZ": @(axis.direction.Z()),
    };
}

void P4AbortOpenCommandNoThrow(
    const Handle(TDocStd_Document)& document) noexcept
{
    try {
        if (!document.IsNull() && document->HasOpenCommand()) {
            document->AbortCommand();
        }
    } catch (...) {
    }
}

} // namespace

@interface Core3DViewController (P4RefusalFixtureSupport)
- (nullable NSDictionary<NSString *, id> *)
    debugP4InteriorShellOpeningForEntity:(NSString *)entityIdentifier;
- (nullable NSDictionary<NSString *, id> *)
    debugP4CommitSecondBooleanInputReferenceAxis:
        (NSDictionary<NSString *, NSString *> *)identifiers;
@end

@implementation Core3DViewController (P4RefusalFixtureSupport)

- (NSDictionary<NSString *, id> *)
    debugP4InteriorShellOpeningForEntity:(NSString *)entityIdentifier
{
    if (![NSThread isMainThread]
        || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0 || entityIdentifier.length > 256) {
        return nil;
    }
    const std::shared_ptr<core3d::Core3DViewer> viewer =
        P4RefusalFixtureViewer(self);
    if (viewer == nullptr || !viewer->canBeginCommittedEdit()) return nil;

    const Handle(OcctDocument) owner = viewer->getDocument();
    const Handle(TDocStd_Document) document = owner.IsNull()
        ? Handle(TDocStd_Document)() : owner->Document();
    const Handle(AIS_InteractiveContext)& context = viewer->AisContext();
    if (owner.IsNull() || document.IsNull() || context.IsNull()
        || document->HasOpenCommand()) return nil;

    try {
        OCC_CATCH_SIGNALS
        const char *utf8 = entityIdentifier.UTF8String;
        if (utf8 == nullptr) return nil;
        const std::string requested(utf8);
        if (requested.empty() || requested.find('\0') != std::string::npos)
            return nil;

        Handle(AIS_Shape) presentation;
        TDF_Label label;
        Standard_Size matchingCount = 0;
        AIS_ListOfInteractive displayed;
        context->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
        for (AIS_ListIteratorOfListOfInteractive it(displayed);
             it.More(); it.Next()) {
            const Handle(AIS_InteractiveObject) candidate = it.Value();
            const TDF_Label candidateLabel = owner->ShapeLabel(candidate);
            if (candidateLabel.IsNull()
                || owner->EntityIdentifierForLabel(candidateLabel)
                    != requested) continue;
            const Handle(AIS_Shape) candidateShape =
                Handle(AIS_Shape)::DownCast(candidate);
            if (candidateShape.IsNull() || candidateShape->Shape().IsNull()
                || candidateShape->Shape().ShapeType() != TopAbs_SOLID
                || !context->IsDisplayed(candidateShape)
                || !owner->IsPresentationEditable(candidateShape)
                || !owner->IsEditableFreeSimpleDefinitionLabel(
                    candidateLabel)) return nil;
            ++matchingCount;
            presentation = candidateShape;
            label = candidateLabel;
        }
        if (matchingCount != 1 || presentation.IsNull() || label.IsNull()
            || owner->RetainedRecipeCoverageForLabel(label)
                != OcctRetainedRecipeCoverage::CurrentProfile) return nil;

        const TopoDS_Shape shape = presentation->Shape();
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(shape, TopAbs_FACE, faces);
        // Triangulation-backed BRep bounds include deflection/tolerance and
        // can make a true boundary plane look interior. This planar fixture
        // needs its exact supporting-plane extrema instead.
        const gp_Dir localZ(0.0, 0.0, 1.0);
        Standard_Real minimumZ =
            std::numeric_limits<Standard_Real>::infinity();
        Standard_Real maximumZ =
            -std::numeric_limits<Standard_Real>::infinity();
        for (Standard_Integer index = 1; index <= faces.Extent(); ++index) {
            BRepAdaptor_Surface surface(
                TopoDS::Face(faces(index)), Standard_True);
            if (surface.GetType() != GeomAbs_Plane) continue;
            const gp_Pln plane = surface.Plane();
            if (!plane.Axis().Direction().IsParallel(
                    localZ, Precision::Angular())) continue;
            const Standard_Real offset = plane.Location().Z();
            minimumZ = std::min(minimumZ, offset);
            maximumZ = std::max(maximumZ, offset);
        }
        if (!std::isfinite(minimumZ) || !std::isfinite(maximumZ)
            || minimumZ >= maximumZ) return nil;
        const Standard_Real zTolerance = std::max(
            Precision::Confusion(),
            32.0 * std::numeric_limits<Standard_Real>::epsilon()
                * std::max({1.0, std::abs(minimumZ), std::abs(maximumZ)}));

        TopoDS_Face interiorFloor;
        Standard_Size faceIndex = 0;
        Standard_Real planeOffset = 0.0;
        Standard_Size interiorCount = 0;
        for (Standard_Integer index = 1; index <= faces.Extent(); ++index) {
            const TopoDS_Face face = TopoDS::Face(faces(index));
            if (!core3d::ShellFaceIsSinglePlanarOpening(face)) continue;
            BRepAdaptor_Surface surface(face, Standard_True);
            if (surface.GetType() != GeomAbs_Plane) continue;
            const gp_Pln plane = surface.Plane();
            if (!plane.Axis().Direction().IsParallel(
                    localZ, Precision::Angular())) continue;
            const Standard_Real offset = plane.Location().Z();
            if (offset <= minimumZ + zTolerance
                || offset >= maximumZ - zTolerance) continue;
            ++interiorCount;
            interiorFloor = face;
            faceIndex = static_cast<Standard_Size>(index - 1);
            planeOffset = offset;
        }
        if (interiorCount != 1 || interiorFloor.IsNull()) return nil;

        core3d::FaceOperationSourceProof proof;
        if (!core3d::TryPrepareShellOperationSource(
                context, owner, presentation, {interiorFloor}, proof)
            || proof.entityIdentifier != requested
            || proof.openingFaceTopologyIndices.size() != 1
            || proof.openingFaceTopologyIndices.front() != faceIndex
            || proof.openingFaces.size() != 1
            || !proof.openingFaces.front().IsEqual(interiorFloor)) return nil;

        Standard_Size selectorMatchCount = 0;
        for (int key = 0; key < 6; ++key) {
            std::vector<core3d::ShellOpeningSelector> selectors = {{
                static_cast<core3d::ShellOpeningAxis>(key / 2),
                static_cast<core3d::ShellOpeningSide>(key % 2)}};
            std::vector<TopoDS_Face> resolved;
            if (core3d::TryResolveShellOpeningSelectors(
                    shape, selectors, resolved)
                && resolved.size() == 1
                && resolved.front().IsEqual(interiorFloor)) {
                ++selectorMatchCount;
            }
        }
        if (selectorMatchCount != 0 || document->HasOpenCommand()
            || !viewer->canBeginCommittedEdit()) return nil;
        return @{
            @"faceIndex": @(faceIndex),
            @"sourceProofValid": @YES,
            @"selectorMatchCount": @(selectorMatchCount),
            @"planeOffset": @(planeOffset),
        };
    } catch (...) {
        return nil;
    }
}

- (NSDictionary<NSString *, id> *)
    debugP4CommitSecondBooleanInputReferenceAxis:
        (NSDictionary<NSString *, NSString *> *)identifiers
{
    if (![NSThread isMainThread]
        || ![identifiers isKindOfClass:NSDictionary.class]
        || identifiers.count != 2) return nil;
    NSString *retained = identifiers[@"retained"];
    NSString *second = identifiers[@"second"];
    if (![retained isKindOfClass:NSString.class]
        || ![second isKindOfClass:NSString.class]
        || retained.length == 0 || retained.length > 256
        || second.length == 0 || second.length > 256
        || [retained isEqualToString:second]) return nil;

    const std::shared_ptr<core3d::Core3DViewer> viewer =
        P4RefusalFixtureViewer(self);
    if (viewer == nullptr) return nil;
    const Handle(OcctDocument) owner = viewer->getDocument();
    const Handle(TDocStd_Document) document = owner.IsNull()
        ? Handle(TDocStd_Document)() : owner->Document();
    const std::shared_ptr<core3d::ObjectInteractor> interactor =
        viewer->getObjectInteractor();
    if (owner.IsNull() || document.IsNull() || interactor == nullptr
        || document->HasOpenCommand()) return nil;

    try {
        OCC_CATCH_SIGNALS
        const char *retainedUTF8 = retained.UTF8String;
        const char *secondUTF8 = second.UTF8String;
        if (retainedUTF8 == nullptr || secondUTF8 == nullptr) return nil;
        const std::string retainedID(retainedUTF8);
        const std::string secondID(secondUTF8);
        if (retainedID.empty() || secondID.empty()
            || retainedID == secondID
            || retainedID.find('\0') != std::string::npos
            || secondID.find('\0') != std::string::npos) return nil;

        const core3d::BooleanPreviewDebugState beforeState =
            interactor->debugBooleanPreviewState();
        if (beforeState.state != core3d::BooleanPreviewState::Ready
            || !beforeState.activeOperation || !beforeState.canApply
            || beforeState.actorOperandCount != 1
            || beforeState.subjectOperandCount != 1
            || beforeState.documentCommandUnresolved
            || beforeState.documentCommandOpen
            || beforeState.workerActive || beforeState.workerPending) return nil;

        core3d::scene::PresentationOverlayContent overlay;
        std::vector<Handle(AIS_Shape)> mirrorObjects;
        core3d::BooleanPreviewCapture preview;
        if (interactor->captureIdlePresentationOverlay(
                overlay, mirrorObjects, preview)
                != core3d::PresentationOverlayCaptureStatus::Available
            || !mirrorObjects.empty()
            || preview.action != core3d::BooleanAction::BooleanSubtract
            || preview.suppressedSourceLabels.size() != 2
            || preview.actors.size() != 1 || preview.results.empty()) return nil;

        const TDF_Label secondLabel = preview.suppressedSourceLabels[0];
        const TDF_Label retainedLabel = preview.suppressedSourceLabels[1];
        if (secondLabel.IsNull() || retainedLabel.IsNull()
            || owner->EntityIdentifierForLabel(secondLabel) != secondID
            || owner->EntityIdentifierForLabel(retainedLabel) != retainedID)
            return nil;

        core3d::profile::Record retainedProfile;
        core3d::profile::Record secondProfile;
        const TopoDS_Shape retainedShape =
            XCAFDoc_ShapeTool::GetShape(retainedLabel);
        const TopoDS_Shape secondShape =
            XCAFDoc_ShapeTool::GetShape(secondLabel);
        const std::string retainedIdentity =
            owner->EntityIdentifierForLabel(retainedLabel);
        const std::string secondIdentity =
            owner->EntityIdentifierForLabel(secondLabel);
        OcctReferenceAxis retainedAxis;
        OcctReferenceAxis secondAxis;
        const OcctReferenceAxisReadState retainedAxisState =
            owner->ReadReferenceAxisForLabel(retainedLabel, retainedAxis);
        const OcctReferenceAxisReadState secondAxisState =
            owner->ReadReferenceAxisForLabel(secondLabel, secondAxis);
        if (!core3d::profile::Read(
                document, retainedLabel, retainedProfile)
            || !core3d::profile::Read(
                document, secondLabel, secondProfile)
            || retainedProfile.label.IsNull() || secondProfile.label.IsNull()
            || !retainedProfile.IsCurrent(document, retainedLabel)
            || !secondProfile.IsCurrent(document, secondLabel)
            || retainedShape.IsNull() || secondShape.IsNull()
            || retainedAxisState == OcctReferenceAxisReadState::Invalid
            || secondAxisState == OcctReferenceAxisReadState::Invalid
            || secondAxis.pivotSpace != OcctReferenceSpace::Object) return nil;

        OcctReferenceAxis changedAxis = secondAxis;
        const Standard_Real changedPivotX = secondAxis.pivot.X() + 1.0;
        if (!std::isfinite(changedPivotX)) return nil;
        changedAxis.pivot.SetX(changedPivotX);
        const Standard_Integer beforeUndo = document->GetAvailableUndos();
        document->NewCommand();
        if (!document->HasOpenCommand()
            || !owner->SetReferenceAxisForLabel(
                secondLabel, changedAxis)
            || !document->CommitCommand() || document->HasOpenCommand()) {
            P4AbortOpenCommandNoThrow(document);
            return nil;
        }

        OcctReferenceAxis retainedAxisAfter;
        OcctReferenceAxis secondAxisAfter;
        core3d::profile::Record retainedProfileAfter;
        core3d::profile::Record secondProfileAfter;
        const core3d::BooleanPreviewDebugState afterState =
            interactor->debugBooleanPreviewState();
        const bool retainedIDUnchanged =
            owner->EntityIdentifierForLabel(retainedLabel)
                == retainedIdentity;
        const bool secondIDUnchanged =
            owner->EntityIdentifierForLabel(secondLabel)
                == secondIdentity;
        const bool retainedShapeUnchanged =
            XCAFDoc_ShapeTool::GetShape(retainedLabel)
                .IsEqual(retainedShape);
        const bool secondShapeUnchanged =
            XCAFDoc_ShapeTool::GetShape(secondLabel)
                .IsEqual(secondShape);
        const bool retainedProfileUnchanged =
            core3d::profile::Read(
                document, retainedLabel, retainedProfileAfter)
            && retainedProfileAfter.IsEqual(retainedProfile);
        const bool secondProfileUnchanged =
            core3d::profile::Read(
                document, secondLabel, secondProfileAfter)
            && secondProfileAfter.IsEqual(secondProfile);
        const OcctReferenceAxisReadState retainedAxisStateAfter =
            owner->ReadReferenceAxisForLabel(
                retainedLabel, retainedAxisAfter);
        const OcctReferenceAxisReadState secondAxisStateAfter =
            owner->ReadReferenceAxisForLabel(secondLabel, secondAxisAfter);
        const bool retainedUnchanged =
            retainedAxisStateAfter == retainedAxisState
            && P4ReferenceAxesMatch(retainedAxisAfter, retainedAxis);
        const bool secondChangedExactly =
            secondAxisStateAfter == OcctReferenceAxisReadState::Authored
            && P4ReferenceAxesMatch(secondAxisAfter, changedAxis)
            && secondAxisAfter.pivot.X() == secondAxis.pivot.X() + 1.0
            && secondAxisAfter.pivot.Y() == secondAxis.pivot.Y()
            && secondAxisAfter.pivot.Z() == secondAxis.pivot.Z();
        const bool previewPreserved =
            afterState.state == core3d::BooleanPreviewState::Ready
            && afterState.generation == beforeState.generation
            && afterState.activeOperation && afterState.canApply
            && afterState.actorOperandCount == 1
            && afterState.subjectOperandCount == 1
            && !afterState.documentCommandUnresolved
            && !afterState.documentCommandOpen
            && !afterState.workerActive && !afterState.workerPending;
        const Standard_Integer afterUndo = document->GetAvailableUndos();
        if (afterUndo != beforeUndo + 1 || document->HasOpenCommand()
            || !retainedIDUnchanged || !secondIDUnchanged
            || !retainedShapeUnchanged || !secondShapeUnchanged
            || !retainedProfileUnchanged || !secondProfileUnchanged
            || !retainedUnchanged || !secondChangedExactly
            || !previewPreserved) return nil;

        return @{
            @"committed": @YES,
            @"changedID": second,
            @"beforeUndo": @(beforeUndo),
            @"afterUndo": @(afterUndo),
            @"beforeAxis": P4ReferenceAxisDictionary(
                secondAxisState, secondAxis),
            @"afterAxis": P4ReferenceAxisDictionary(
                secondAxisStateAfter, secondAxisAfter),
            @"generation": @(beforeState.generation),
            @"retainedUnchanged": @(retainedUnchanged),
            @"retainedIDUnchanged": @(retainedIDUnchanged),
            @"secondIDUnchanged": @(secondIDUnchanged),
            @"retainedShapeUnchanged": @(retainedShapeUnchanged),
            @"secondShapeUnchanged": @(secondShapeUnchanged),
            @"retainedProfileUnchanged": @(retainedProfileUnchanged),
            @"secondProfileUnchanged": @(secondProfileUnchanged),
            @"previewPreserved": @(previewPreserved),
        };
    } catch (...) {
        P4AbortOpenCommandNoThrow(document);
        return nil;
    }
}

@end
#endif
#include "CafShapePrs.h"
#include "../Common/Core3DMobileResourceLimits.h"

#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_ProgressRange.hxx>
#include <Message_ProgressScope.hxx>

#include <TCollection_AsciiString.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <BinDrivers_DocumentStorageDriver.hxx>
#include <BinXCAFDrivers_DocumentStorageDriver.hxx>
#include <BinDrivers_DocumentRetrievalDriver.hxx>
#include <BinMDF_ADriverTable.hxx>
#include <BinMDF_TagSourceDriver.hxx>
#include <BinMDataStd_AsciiStringDriver.hxx>
#include <BinMDataStd_GenericEmptyDriver.hxx>
#include <BinMDataStd_GenericExtStringDriver.hxx>
#include <BinMDataStd_IntegerDriver.hxx>
#include <BinMDataStd_RealDriver.hxx>
#include <BinMDataStd_TreeNodeDriver.hxx>
#include <BinMDataStd_UAttributeDriver.hxx>
#include <BinMNaming_NamedShapeDriver.hxx>
#include <BinMXCAFDoc_ColorDriver.hxx>
#include <BinMXCAFDoc_LengthUnitDriver.hxx>
#include "SavedFeatureRecords.hxx"
#include "SweepRebuildDefinition.hxx"
#include <BinMXCAFDoc_LocationDriver.hxx>
#include <BinMXCAFDoc_VisMaterialDriver.hxx>
#include <BinMXCAFDoc_VisMaterialToolDriver.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <Storage_HeaderData.hxx>
#include <Storage_Schema.hxx>

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <TDataStd_Real.hxx>
#include <TDataStd_TreeNode.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_Tool.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRep_Builder.hxx>
#include <gp_Trsf.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <GP_Quaternion.hxx>
#include <TNaming.hxx>
#include <TNaming_NamedShape.hxx>
#include <Standard_GUID.hxx>
#include <TDF_LabelMap.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_MapOfShape.hxx>
#include <XCAFPrs_DocumentExplorer.hxx>
#include <Graphic3d_TextureSet.hxx>
#include "Core3DDataMapShader.hxx"
#include <Graphic3d_TextureParams.hxx>
#include <XCAFPrs_Texture.hxx>
#include <Image_PixMap.hxx>
#include <Image_Texture.hxx>
#include <NCollection_Buffer.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <algorithm>
#include <cmath>
#include <array>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

IMPLEMENT_STANDARD_RTTIEXT(OcctDocument, Standard_Transient)

const Standard_GUID& Core3DDuplicateCommandOwnerAttributeID()
{
    static const Standard_GUID anId(
        "D7598D08-A879-4E17-8D23-CC92568EAD5C");
    return anId;
}

const Standard_GUID& Core3DOrdinaryEditCommandOwnerAttributeID()
{
    static const Standard_GUID anId("58BBFE2E-F5F9-4F87-A7EA-EC1459362C89");
    return anId;
}

const Standard_GUID& Core3DRadialArrayCommandOwnerAttributeID()
{
    static const Standard_GUID anId(
        "2DF9F8BE-F297-4CFD-8D93-A40346EBEF3A");
    return anId;
}

namespace {

Handle(Graphic3d_AspectFillArea3d) ClearDrawerTextureMapping(
    const Handle(Prs3d_Drawer)& theDrawer)
{
    if (theDrawer.IsNull()) {
        return {};
    }
    theDrawer->SetupOwnShadingAspect();
    const Handle(Prs3d_ShadingAspect)& aShading =
        theDrawer->ShadingAspect();
    if (aShading.IsNull() || aShading->Aspect().IsNull()) {
        return {};
    }
    // XCAFDoc_VisMaterial::FillAspect() deliberately does not clear an
    // existing texture set when the new material has no maps. Reset both the
    // enable bit and the owning handle before every full material transition.
    if (IsCore3DDataMapShader(aShading->Aspect()->ShaderProgram())) {
        aShading->Aspect()->SetShaderProgram({});
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
    if (anAspect.IsNull()) {
        return;
    }
    // Graphic3d_Aspects and XCAFDoc_VisMaterial define this as the legacy
    // compatibility contract. BlendAuto resolves an opaque preset to opaque
    // and a transparent preset to blending, while Auto culls only closed,
    // opaque groups. This is the same resolution used by the Metal snapshot.
    anAspect->SetAlphaMode(Graphic3d_AlphaMode_BlendAuto, 0.5f);
    anAspect->SetFaceCulling(
        Graphic3d_TypeOfBackfacingModel_Auto);
}

void ApplyVisualMaterialToPlainPresentation(
    const Handle(XCAFDoc_VisMaterial)& theMaterial,
    const Handle(AIS_Shape)& thePresentation)
{
    if (theMaterial.IsNull() || thePresentation.IsNull()) {
        return;
    }
    Graphic3d_MaterialAspect anAspect;
    theMaterial->FillMaterialAspect(anAspect);
    // Keep AIS' own-material and own-color state intact for selection,
    // duplication, and legacy callers, while installing the renderer-facing
    // texture set through OCCT's native Image_Texture bridge.
    thePresentation->SetMaterial(anAspect);
    thePresentation->SetColor(theMaterial->BaseColor().GetRGB());
    const Handle(Graphic3d_AspectFillArea3d) aFillAspect =
        ClearDrawerTextureMapping(thePresentation->Attributes());
    if (aFillAspect.IsNull()) {
        thePresentation->SynchronizeAspects();
        return;
    }
    theMaterial->FillAspect(aFillAspect);
    Core3DPrepareRendererTextures(aFillAspect);
    thePresentation->SynchronizeAspects();
}

constexpr Standard_Integer kMaximumVisualMaterialDefinitions = 2048;
constexpr Standard_Size kMaximumDocumentLabels = 100000;
constexpr Standard_Real kDefaultMetersPerUnit = 0.001;
constexpr Standard_Real kMaximumEmissionFactor = 65504.0;
constexpr Standard_Size kMaximumEmbeddedTextureBytes =
    32ull * 1024ull * 1024ull;
constexpr Standard_Size kMaximumAggregateTextureBytes =
    128ull * 1024ull * 1024ull;
constexpr std::uint64_t kMaximumTextureDimension = 8192;
constexpr std::uint64_t kMaximumTexturePixels = 4096ull * 4096ull;
constexpr Standard_Size kMaximumDecodedTextureBytes =
    128ull * 1024ull * 1024ull;
constexpr Standard_Integer kMaximumPersistentTextureIdentifierBytes = 256;
constexpr Standard_Integer kMaximumPersistentNameCharacters = 4096;
constexpr Standard_Integer kPersistentRecordHeaderBytes =
    3 * static_cast<Standard_Integer>(sizeof(Standard_Integer));
constexpr const char* kBufferTexturePrefix = "texturebuf://";
thread_local bool gSafeBinaryReadRejected = false;

Handle(Image_Texture) MakeRendererDecodableTexture(
    const Handle(NCollection_Buffer)& theBuffer,
    const TCollection_AsciiString& theIdentifier);

void RejectSafeBinaryRead() noexcept
{
    gSafeBinaryReadRejected = true;
}

bool TryPersistentRecordEnd(
    const BinObjMgt_Persistent& theSource,
    Standard_Integer& theRecordEnd)
{
    const Standard_Integer aLength = theSource.Length();
    if (aLength < 0
        || aLength
            > std::numeric_limits<Standard_Integer>::max()
                - kPersistentRecordHeaderBytes) {
        return false;
    }
    theRecordEnd = kPersistentRecordHeaderBytes + aLength;
    return true;
}

bool ReadBoundedPersistentAsciiString(
    const BinObjMgt_Persistent& theSource,
    std::string& theValue)
{
    theValue.clear();
    const Standard_Integer aPosition = theSource.Position();
    Standard_Integer aRecordEnd = 0;
    if (!TryPersistentRecordEnd(theSource, aRecordEnd)
        || aPosition < kPersistentRecordHeaderBytes
        || aPosition > aRecordEnd
        || aPosition
            > std::numeric_limits<Standard_Integer>::max() - 3) {
        return false;
    }
    const Standard_Integer anAlignedPosition =
        (aPosition + 3) & ~Standard_Integer(3);
    if (anAlignedPosition > aRecordEnd
        || !theSource.SetPosition(anAlignedPosition)) {
        return false;
    }

    for (Standard_Integer anIndex = 0;
         anIndex <= kMaximumPersistentTextureIdentifierBytes;
         ++anIndex) {
        if (theSource.Position() >= aRecordEnd) {
            return false;
        }
        Standard_Character aCharacter = '\0';
        if (!theSource.GetCharacter(aCharacter).IsOK()) {
            return false;
        }
        if (aCharacter == '\0') {
            return true;
        }
        if (anIndex == kMaximumPersistentTextureIdentifierBytes) {
            return false;
        }
        theValue.push_back(aCharacter);
    }
    return false;
}

bool StripBufferTexturePrefixes(std::string& theIdentifier)
{
    const std::string aPrefix(kBufferTexturePrefix);
    while (theIdentifier.rfind(aPrefix, 0) == 0) {
        theIdentifier.erase(0, aPrefix.size());
    }
    return !theIdentifier.empty()
        && theIdentifier.size()
            <= static_cast<std::size_t>(
                kMaximumPersistentTextureIdentifierBytes
                - aPrefix.size());
}

bool PreflightBoundedPersistentExtendedString(
    const BinObjMgt_Persistent& theSource)
{
    const Standard_Integer aStart = theSource.Position();
    Standard_Integer aRecordEnd = 0;
    if (!TryPersistentRecordEnd(theSource, aRecordEnd)
        || aStart < kPersistentRecordHeaderBytes
        || aStart > aRecordEnd
        || aStart
            > std::numeric_limits<Standard_Integer>::max() - 3) {
        return false;
    }
    const Standard_Integer anAlignedPosition =
        (aStart + 3) & ~Standard_Integer(3);
    if (anAlignedPosition > aRecordEnd
        || !theSource.SetPosition(anAlignedPosition)) {
        return false;
    }
    for (Standard_Integer anIndex = 0;
         anIndex <= kMaximumPersistentNameCharacters;
         ++anIndex) {
        if (theSource.Position() > aRecordEnd
                - static_cast<Standard_Integer>(
                    sizeof(Standard_ExtCharacter))) {
            return false;
        }
        Standard_ExtCharacter aCharacter = 0;
        if (!theSource.GetExtCharacter(aCharacter).IsOK()) {
            return false;
        }
        if (aCharacter == 0) {
            return theSource.SetPosition(aStart);
        }
        if (anIndex == kMaximumPersistentNameCharacters) {
            return false;
        }
    }
    return false;
}

bool PreflightEmbeddedTexture(
    const BinObjMgt_Persistent& theSource,
    Standard_Size& theAggregateBytes)
{
    std::string anIdentifier;
    if (!ReadBoundedPersistentAsciiString(theSource, anIdentifier)) {
        return false;
    }
    if (anIdentifier.empty()) {
        return true;
    }

    Standard_Boolean usesBuffer = Standard_False;
    if (!theSource.GetBoolean(usesBuffer).IsOK() || !usesBuffer
        || !StripBufferTexturePrefixes(anIdentifier)) {
        // External paths and offsets are deliberately unsupported for
        // self-contained, sandbox-safe projects.
        return false;
    }

    Standard_Integer aLength = 0;
    if (!theSource.GetInteger(aLength).IsOK()
        || aLength <= 0
        || static_cast<Standard_Size>(aLength)
            > kMaximumEmbeddedTextureBytes) {
        return false;
    }
    const Standard_Integer aPosition = theSource.Position();
    Standard_Integer aRecordEnd = 0;
    if (!TryPersistentRecordEnd(theSource, aRecordEnd)
        || aPosition < kPersistentRecordHeaderBytes
        || aPosition > aRecordEnd
        || aLength > aRecordEnd - aPosition
        || theAggregateBytes > kMaximumAggregateTextureBytes
        || static_cast<Standard_Size>(aLength)
            > kMaximumAggregateTextureBytes - theAggregateBytes) {
        return false;
    }
    theAggregateBytes += static_cast<Standard_Size>(aLength);
    return theSource.SetPosition(aPosition + aLength);
}

bool SkipShortReals(
    const BinObjMgt_Persistent& theSource,
    const Standard_Integer theCount)
{
    for (Standard_Integer anIndex = 0; anIndex < theCount; ++anIndex) {
        Standard_ShortReal aValue = 0.0f;
        if (!theSource.GetShortReal(aValue).IsOK()) {
            return false;
        }
    }
    return true;
}

bool NormalizeEmbeddedTexture(Handle(Image_Texture)& theTexture)
{
    if (theTexture.IsNull()) {
        return true;
    }
    const Handle(NCollection_Buffer)& aBuffer = theTexture->DataBuffer();
    if (aBuffer.IsNull()) {
        return false;
    }
    std::string anIdentifier(theTexture->TextureId().ToCString());
    if (!StripBufferTexturePrefixes(anIdentifier)) {
        return false;
    }
    theTexture = MakeRendererDecodableTexture(
        aBuffer, TCollection_AsciiString(anIdentifier.c_str()));
    return true;
}

bool IsPNGSignature(const Standard_Byte* theBytes,
                    const Standard_Size theSize)
{
    return theBytes != nullptr && theSize >= 8
        && std::memcmp(theBytes, "\x89PNG\r\n\x1A\n", 8) == 0;
}

bool IsJPEGSignature(const Standard_Byte* theBytes,
                     const Standard_Size theSize)
{
    return theBytes != nullptr && theSize >= 3
        && theBytes[0] == 0xFF && theBytes[1] == 0xD8
        && theBytes[2] == 0xFF;
}

bool HasSupportedRasterSignature(const Standard_Byte* theBytes,
                                 const Standard_Size theSize)
{
    if (theBytes == nullptr) {
        return false;
    }
    return IsPNGSignature(theBytes, theSize)
        || IsJPEGSignature(theBytes, theSize)
        || (theSize >= 6
            && (std::memcmp(theBytes, "GIF87a", 6) == 0
                || std::memcmp(theBytes, "GIF89a", 6) == 0))
        || (theSize >= 4
            && (std::memcmp(theBytes, "II\x2A\x00", 4) == 0
                || std::memcmp(theBytes, "MM\x00\x2A", 4) == 0))
        || (theSize >= 2 && std::memcmp(theBytes, "BM", 2) == 0)
        || (theSize >= 12
            && std::memcmp(theBytes, "RIFF", 4) == 0
            && std::memcmp(theBytes + 8, "WEBP", 4) == 0);
}

bool TryRendererTextureDimensions(
    const std::uint64_t theSourceWidth,
    const std::uint64_t theSourceHeight,
    std::uint64_t& theRendererWidth,
    std::uint64_t& theRendererHeight)
{
    theRendererWidth = 0;
    theRendererHeight = 0;
    if (theSourceWidth == 0 || theSourceHeight == 0
        || theSourceWidth > kMaximumTextureDimension
        || theSourceHeight > kMaximumTextureDimension
        || theSourceWidth > kMaximumTexturePixels / theSourceHeight) {
        return false;
    }
    const auto previousPowerOfTwo = [](const std::uint64_t theValue) {
        std::uint64_t aResult = 1;
        while (aResult <= theValue / 2) {
            aResult *= 2;
        }
        return aResult;
    };
    const std::uint64_t aPreviousWidth =
        previousPowerOfTwo(theSourceWidth);
    const std::uint64_t aPreviousHeight =
        previousPowerOfTwo(theSourceHeight);
    const std::uint64_t aNextWidth = aPreviousWidth == theSourceWidth
        ? theSourceWidth : aPreviousWidth * 2;
    const std::uint64_t aNextHeight = aPreviousHeight == theSourceHeight
        ? theSourceHeight : aPreviousHeight * 2;
    const std::uint64_t aMaximumDecodedPixels =
        static_cast<std::uint64_t>(kMaximumDecodedTextureBytes) / 4;
    const bool canUseNextPowerOfTwo =
        aNextWidth <= kMaximumTextureDimension
        && aNextHeight <= kMaximumTextureDimension
        && aNextWidth <= kMaximumTexturePixels / aNextHeight
        && aNextWidth <= aMaximumDecodedPixels / aNextHeight;
    theRendererWidth = canUseNextPowerOfTwo
        ? aNextWidth : aPreviousWidth;
    theRendererHeight = canUseNextPowerOfTwo
        ? aNextHeight : aPreviousHeight;
    return theRendererWidth > 0 && theRendererHeight > 0
        && theRendererWidth <= kMaximumTexturePixels / theRendererHeight
        && theRendererWidth <= aMaximumDecodedPixels / theRendererHeight;
}

bool TryTextureDecodedBytes(
    const Handle(NCollection_Buffer)& theBuffer,
    Standard_Size& theDecodedBytes)
{
    theDecodedBytes = 0;
    if (theBuffer.IsNull() || theBuffer->Data() == nullptr
        || theBuffer->Size() == 0
        || !HasSupportedRasterSignature(
            theBuffer->Data(), theBuffer->Size())) {
        return false;
    }
    CFDataRef data = CFDataCreateWithBytesNoCopy(
        kCFAllocatorDefault,
        reinterpret_cast<const UInt8*>(theBuffer->Data()),
        static_cast<CFIndex>(theBuffer->Size()),
        kCFAllocatorNull);
    if (data == nullptr) {
        return false;
    }
    const void* optionKeys[] = {kCGImageSourceShouldCache};
    const void* optionValues[] = {kCFBooleanFalse};
    CFDictionaryRef options = CFDictionaryCreate(
        kCFAllocatorDefault,
        optionKeys,
        optionValues,
        1,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    CGImageSourceRef source = CGImageSourceCreateWithData(data, options);
    if (options != nullptr) {
        CFRelease(options);
    }
    CFRelease(data);
    if (source == nullptr || CGImageSourceGetType(source) == nullptr
        || CGImageSourceGetCount(source) != 1
        || CGImageSourceGetStatus(source) != kCGImageStatusComplete
        || CGImageSourceGetStatusAtIndex(source, 0)
            != kCGImageStatusComplete) {
        if (source != nullptr) {
            CFRelease(source);
        }
        return false;
    }

    CFDictionaryRef properties =
        CGImageSourceCopyPropertiesAtIndex(source, 0, nullptr);
    CFRelease(source);
    if (properties == nullptr) {
        return false;
    }
    const CFTypeRef widthValue = CFDictionaryGetValue(
        properties, kCGImagePropertyPixelWidth);
    const CFTypeRef heightValue = CFDictionaryGetValue(
        properties, kCGImagePropertyPixelHeight);
    std::int64_t width = 0;
    std::int64_t height = 0;
    std::uint64_t rendererWidth = 0;
    std::uint64_t rendererHeight = 0;
    const bool isValid = widthValue != nullptr
        && heightValue != nullptr
        && CFGetTypeID(widthValue) == CFNumberGetTypeID()
        && CFGetTypeID(heightValue) == CFNumberGetTypeID()
        && CFNumberGetValue(
            static_cast<CFNumberRef>(widthValue),
            kCFNumberSInt64Type, &width)
        && CFNumberGetValue(
            static_cast<CFNumberRef>(heightValue),
            kCFNumberSInt64Type, &height)
        && width > 0 && height > 0
        && TryRendererTextureDimensions(
            static_cast<std::uint64_t>(width),
            static_cast<std::uint64_t>(height),
            rendererWidth, rendererHeight);
    if (isValid) {
        const std::uint64_t pixels =
            rendererWidth * rendererHeight;
        if (pixels
            <= std::numeric_limits<Standard_Size>::max() / 4) {
            theDecodedBytes =
                static_cast<Standard_Size>(pixels * 4);
        }
    }
    CFRelease(properties);
    return isValid && theDecodedBytes > 0
        && theDecodedBytes <= kMaximumDecodedTextureBytes;
}

Handle(Image_PixMap) DecodeRendererTextureWithImageIO(
    const Handle(NCollection_Buffer)& theBuffer)
{
    Standard_Size aPreflightDecodedBytes = 0;
    if (!TryTextureDecodedBytes(
            theBuffer, aPreflightDecodedBytes)
        || theBuffer.IsNull() || theBuffer->Data() == nullptr) {
        return {};
    }

    CFDataRef aData = CFDataCreateWithBytesNoCopy(
        kCFAllocatorDefault,
        reinterpret_cast<const UInt8*>(theBuffer->Data()),
        static_cast<CFIndex>(theBuffer->Size()),
        kCFAllocatorNull);
    if (aData == nullptr) {
        return {};
    }
    const void* aSourceKeys[] = {
        kCGImageSourceShouldCache,
    };
    const void* aSourceValues[] = {
        kCFBooleanFalse,
    };
    CFDictionaryRef aSourceOptions = CFDictionaryCreate(
        kCFAllocatorDefault,
        aSourceKeys,
        aSourceValues,
        1,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    CGImageSourceRef aSource =
        CGImageSourceCreateWithData(aData, aSourceOptions);
    if (aSourceOptions != nullptr) {
        CFRelease(aSourceOptions);
    }
    CFRelease(aData);
    if (aSource == nullptr
        || CGImageSourceGetCount(aSource) != 1
        || CGImageSourceGetStatus(aSource)
            != kCGImageStatusComplete
        || CGImageSourceGetStatusAtIndex(aSource, 0)
            != kCGImageStatusComplete) {
        if (aSource != nullptr) {
            CFRelease(aSource);
        }
        return {};
    }

    // Thumbnail creation at the already-validated maximum dimension is the
    // ImageIO API that applies EXIF orientation without ever allocating an
    // unbounded intermediate. Valid sources remain at their native size.
    std::int64_t aMaximumDimension =
        static_cast<std::int64_t>(kMaximumTextureDimension);
    CFNumberRef aMaximumDimensionNumber = CFNumberCreate(
        kCFAllocatorDefault,
        kCFNumberSInt64Type,
        &aMaximumDimension);
    if (aMaximumDimensionNumber == nullptr) {
        CFRelease(aSource);
        return {};
    }
    const void* aDecodeKeys[] = {
        kCGImageSourceCreateThumbnailFromImageAlways,
        kCGImageSourceCreateThumbnailWithTransform,
        kCGImageSourceThumbnailMaxPixelSize,
        kCGImageSourceShouldCacheImmediately,
    };
    const void* aDecodeValues[] = {
        kCFBooleanTrue,
        kCFBooleanTrue,
        aMaximumDimensionNumber,
        kCFBooleanTrue,
    };
    CFDictionaryRef aDecodeOptions = CFDictionaryCreate(
        kCFAllocatorDefault,
        aDecodeKeys,
        aDecodeValues,
        4,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    CGImageRef anImage = aDecodeOptions == nullptr
        ? nullptr
        : CGImageSourceCreateThumbnailAtIndex(
            aSource, 0, aDecodeOptions);
    if (aDecodeOptions != nullptr) {
        CFRelease(aDecodeOptions);
    }
    CFRelease(aMaximumDimensionNumber);
    CFRelease(aSource);
    if (anImage == nullptr) {
        return {};
    }

    const std::uint64_t aSourceWidth = CGImageGetWidth(anImage);
    const std::uint64_t aSourceHeight = CGImageGetHeight(anImage);
    std::uint64_t aWidth = 0;
    std::uint64_t aHeight = 0;
    if (!TryRendererTextureDimensions(
            aSourceWidth, aSourceHeight,
            aWidth, aHeight)) {
        CGImageRelease(anImage);
        return {};
    }

    Handle(Image_PixMap) aPixMap = new Image_PixMap();
    if (aPixMap.IsNull()
        || !aPixMap->InitTrash(
            Image_Format_RGBA,
            static_cast<Standard_Size>(aWidth),
            static_cast<Standard_Size>(aHeight),
            static_cast<Standard_Size>(aWidth * 4))) {
        CGImageRelease(anImage);
        return {};
    }
    // CGBitmapContext writes the first scanline as the visual top row for a
    // CGImage draw. OCCT uses this flag to apply the matching OpenGL V flip.
    aPixMap->SetTopDown(true);

    CGColorSpaceRef aColorSpace =
        CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    const CGBitmapInfo aBitmapInfo = static_cast<CGBitmapInfo>(
        kCGImageAlphaPremultipliedLast
        | kCGBitmapByteOrder32Big);
    CGContextRef aContext = aColorSpace == nullptr
        ? nullptr
        : CGBitmapContextCreate(
            aPixMap->ChangeData(),
            static_cast<size_t>(aWidth),
            static_cast<size_t>(aHeight),
            8,
            aPixMap->SizeRowBytes(),
            aColorSpace,
            aBitmapInfo);
    if (aColorSpace != nullptr) {
        CGColorSpaceRelease(aColorSpace);
    }
    if (aContext == nullptr) {
        CGImageRelease(anImage);
        return {};
    }
    CGContextSetBlendMode(aContext, kCGBlendModeCopy);
    CGContextSetInterpolationQuality(aContext, kCGInterpolationHigh);
    // OCCT's OpenGL ES 2 texture wrapper unconditionally requests mipmaps.
    // Resampling only NPOT axes into a bounded POT backing avoids an
    // incomplete/black GPU texture while keeping the entire source image in
    // the same normalized UV domain. Already-POT dimensions remain unchanged.
    CGContextDrawImage(
        aContext,
        CGRectMake(
            0.0, 0.0,
            static_cast<CGFloat>(aWidth),
            static_cast<CGFloat>(aHeight)),
        anImage);
    CGContextFlush(aContext);
    CGContextRelease(aContext);
    CGImageRelease(anImage);

    // CoreGraphics' supported RGBA bitmap layout is premultiplied. OCCT's
    // base-color texture contract is straight alpha, so undo the premultiply
    // explicitly; otherwise translucent texels are darkened a second time by
    // the PBR shader. Fully transparent RGB is canonicalized to zero.
    for (Standard_Size aRow = 0; aRow < aPixMap->SizeY(); ++aRow) {
        Standard_Byte* aPixel = aPixMap->ChangeRow(aRow);
        for (Standard_Size aColumn = 0;
             aColumn < aPixMap->SizeX();
             ++aColumn, aPixel += 4) {
            const unsigned int anAlpha = aPixel[3];
            if (anAlpha == 0) {
                aPixel[0] = 0;
                aPixel[1] = 0;
                aPixel[2] = 0;
                continue;
            }
            if (anAlpha == 255) {
                continue;
            }
            for (Standard_Integer aChannel = 0;
                 aChannel < 3;
                 ++aChannel) {
                const unsigned int aStraight =
                    (static_cast<unsigned int>(aPixel[aChannel])
                        * 255u + anAlpha / 2u) / anAlpha;
                aPixel[aChannel] = static_cast<Standard_Byte>(
                    std::min(aStraight, 255u));
            }
        }
    }
    return aPixMap;
}

// Numeric PNGs are sampled as stored channel codes. Do not draw through a
// color-managed CGContext: a linear-tagged roughness value of 128 is not an
// sRGB color. The narrow admission matches the mobile numeric importer.
template<class T> struct ScopedNumericCF {
    T value;
    explicit ScopedNumericCF(T v) : value(v) {}
    ~ScopedNumericCF() { if (value != nullptr) CFRelease(value); }
    ScopedNumericCF(const ScopedNumericCF&) = delete;
    ScopedNumericCF& operator=(const ScopedNumericCF&) = delete;
};

Handle(Image_PixMap) DecodeNumericRendererPNG(
    const Handle(NCollection_Buffer)& buffer)
{
    Standard_Size outputBytes = 0;
    if (!TryTextureDecodedBytes(buffer, outputBytes)
        || buffer.IsNull() || buffer->Size() < 33) return {};
    const Standard_Byte* header = buffer->Data();
    static const unsigned char prefix[] = {
        137,80,78,71,13,10,26,10,0,0,0,13,73,72,68,82};
    if (std::memcmp(header, prefix, sizeof(prefix)) != 0
        || header[24] != 8
        || !(header[25] == 0 || header[25] == 2
             || header[25] == 4 || header[25] == 6)) return {};
    ScopedNumericCF<CFDataRef> data(CFDataCreateWithBytesNoCopy(
        kCFAllocatorDefault, header, static_cast<CFIndex>(buffer->Size()),
        kCFAllocatorNull));
    if (data.value == nullptr) return {};
    NSDictionary* options = @{(__bridge NSString*)kCGImageSourceShouldCache: @NO};
    ScopedNumericCF<CGImageSourceRef> source(CGImageSourceCreateWithData(
        data.value, (__bridge CFDictionaryRef)options));
    if (source.value == nullptr || CGImageSourceGetCount(source.value) != 1
        || CGImageSourceGetStatus(source.value) != kCGImageStatusComplete
        || CGImageSourceGetStatusAtIndex(source.value, 0) != kCGImageStatusComplete)
        return {};
    ScopedNumericCF<CFDictionaryRef> properties(CGImageSourceCopyPropertiesAtIndex(
        source.value, 0, (__bridge CFDictionaryRef)options));
    if (properties.value == nullptr) return {};
    CFTypeRef orientation = CFDictionaryGetValue(properties.value,
                                                 kCGImagePropertyOrientation);
    if (orientation != nullptr) {
        double value = 0;
        if (CFGetTypeID(orientation) != CFNumberGetTypeID()
            || !CFNumberGetValue(static_cast<CFNumberRef>(orientation),
                                kCFNumberDoubleType, &value) || value != 1) return {};
    }
    ScopedNumericCF<CGImageRef> image(CGImageSourceCreateImageAtIndex(
        source.value, 0, (__bridge CFDictionaryRef)options));
    if (image.value == nullptr || CGImageGetBitsPerComponent(image.value) != 8
        || (CGImageGetBitmapInfo(image.value) & kCGBitmapFloatComponents)) return {};
    const size_t width = CGImageGetWidth(image.value);
    const size_t height = CGImageGetHeight(image.value);
    std::uint64_t renderWidth = 0, renderHeight = 0;
    if (!TryRendererTextureDimensions(width, height, renderWidth, renderHeight)
        || renderWidth * renderHeight * 4 != outputBytes) return {};
    CGColorSpaceRef space = CGImageGetColorSpace(image.value);
    if (space == nullptr) return {};
    const CGColorSpaceModel model = CGColorSpaceGetModel(space);
    if (model != kCGColorSpaceModelRGB && model != kCGColorSpaceModelMonochrome)
        return {};
    const int channels = model == kCGColorSpaceModelMonochrome ? 1 : 3;
    if (CGColorSpaceGetNumberOfComponents(space) != channels) return {};
    const CGFloat* decode = CGImageGetDecode(image.value);
    for (int c = 0; decode != nullptr && c < channels; ++c)
        if (decode[c * 2] != 0 || decode[c * 2 + 1] != 1) return {};
    const size_t bits = CGImageGetBitsPerPixel(image.value);
    if (bits % 8 != 0) return {};
    const size_t stride = bits / 8;
    int colorStart = 0, alphaIndex = -1;
    const CGImageAlphaInfo alpha = CGImageGetAlphaInfo(image.value);
    switch (alpha) {
        case kCGImageAlphaNone:
            if (stride != channels) return {};
            break;
        case kCGImageAlphaLast: case kCGImageAlphaPremultipliedLast:
        case kCGImageAlphaNoneSkipLast:
            if (stride != channels + 1) return {};
            alphaIndex = alpha == kCGImageAlphaNoneSkipLast ? -1 : channels;
            break;
        case kCGImageAlphaFirst: case kCGImageAlphaPremultipliedFirst:
        case kCGImageAlphaNoneSkipFirst:
            if (stride != channels + 1) return {};
            colorStart = 1;
            alphaIndex = alpha == kCGImageAlphaNoneSkipFirst ? -1 : 0;
            break;
        default: return {};
    }
    const CGBitmapInfo order = CGImageGetBitmapInfo(image.value) & kCGBitmapByteOrderMask;
    const bool reverse = stride == 4 && order == kCGBitmapByteOrder32Little;
    if (!(order == kCGBitmapByteOrderDefault
          || (stride == 4 && order == kCGBitmapByteOrder32Big) || reverse)) return {};
    CGDataProviderRef provider = CGImageGetDataProvider(image.value);
    if (provider == nullptr) return {};
    const size_t rowBytes = CGImageGetBytesPerRow(image.value);
    if (rowBytes < width * stride
        || rowBytes > (kMaximumDecodedTextureBytes - outputBytes) / height) return {};
    ScopedNumericCF<CFDataRef> raw(CGDataProviderCopyData(provider));
    if (raw.value == nullptr || CFDataGetLength(raw.value) < 0) return {};
    const auto rawSize = static_cast<std::uint64_t>(CFDataGetLength(raw.value));
    if (rowBytes < width * stride || rowBytes > rawSize / height
        || rawSize > kMaximumDecodedTextureBytes - outputBytes) return {};
    const UInt8* bytes = CFDataGetBytePtr(raw.value);
    if (bytes == nullptr) return {};
    const auto sample = [&](size_t x, size_t y, int c) -> unsigned int {
        return bytes[y * rowBytes + x * stride + (reverse ? stride - 1 - c : c)];
    };
    // Validate every source sample, including ones down/up-sampling may skip.
    if (alphaIndex >= 0)
        for (size_t y = 0; y < height; ++y)
            for (size_t x = 0; x < width; ++x)
                if (sample(x, y, alphaIndex) != 255) return {};
    Handle(Image_PixMap) result = new Image_PixMap();
    if (!result->InitTrash(Image_Format_RGBA, renderWidth, renderHeight,
                           renderWidth * 4)) return {};
    result->SetTopDown(true);
    // OCCT ES2 needs a POT mipmapped backing. Interpolate numeric codes in
    // linear space only on NPOT axes; POT images preserve each code exactly.
    for (size_t y = 0; y < renderHeight; ++y) {
        const double sy = std::max(0.0, std::min(double(height - 1),
            (double(y) + 0.5) * height / renderHeight - 0.5));
        const size_t y0 = static_cast<size_t>(sy), y1 = std::min(y0 + 1, height - 1);
        const double fy = sy - y0;
        Standard_Byte* out = result->ChangeRow(y);
        for (size_t x = 0; x < renderWidth; ++x, out += 4) {
            const double sx = std::max(0.0, std::min(double(width - 1),
                (double(x) + 0.5) * width / renderWidth - 0.5));
            const size_t x0 = static_cast<size_t>(sx), x1 = std::min(x0 + 1, width - 1);
            const double fx = sx - x0;
            for (int c = 0; c < 3; ++c) {
                const int channel = colorStart + (channels == 1 ? 0 : c);
                const double top = (1 - fx) * sample(x0, y0, channel) + fx * sample(x1, y0, channel);
                const double bottom = (1 - fx) * sample(x0, y1, channel) + fx * sample(x1, y1, channel);
                out[c] = static_cast<Standard_Byte>(std::lround((1 - fy) * top + fy * bottom));
            }
            out[3] = 255;
        }
    }
    return result;
}

// Slot interpretation belongs to the renderer binding, never to Image_Texture
// or its persisted content address. XCAFPrs_Texture otherwise shares the same
// GPU key for identical bytes even when one binding is sRGB and another linear.
class Core3DRoleAwareTexture final : public XCAFPrs_Texture {
    DEFINE_STANDARD_RTTI_INLINE(Core3DRoleAwareTexture, XCAFPrs_Texture)
public:
    explicit Core3DRoleAwareTexture(const Handle(XCAFPrs_Texture)& original)
    : XCAFPrs_Texture(original->GetImageSource(), original->GetParams()->TextureUnit()) {
        myParams = original->GetParams();
        myHasMipmaps = original->HasMipmaps();
        myTexId += IsColorMap() ? "|shapeyard-color-v1" : "|shapeyard-data-v1";
    }
    Handle(Image_CompressedPixMap) GetCompressedImage(
        const Handle(Image_SupportedFormats)&) override { return {}; }
    Handle(Image_PixMap) GetImage(
        const Handle(Image_SupportedFormats)& supported) override {
        if (IsColorMap()) return XCAFPrs_Texture::GetImage(supported);
        Handle(Image_PixMap) result = GetImageSource().IsNull()
            ? Handle(Image_PixMap)()
            : DecodeNumericRendererPNG(GetImageSource()->DataBuffer());
        if (!result.IsNull() && !supported.IsNull()) convertToCompatible(supported, result);
        return result;
    }
};

class Core3DImageIOTexture final : public Image_Texture {
    DEFINE_STANDARD_RTTI_INLINE(
        Core3DImageIOTexture,
        Image_Texture)

public:
    Core3DImageIOTexture(
        const Handle(NCollection_Buffer)& theBuffer,
        const TCollection_AsciiString& theIdentifier)
    : Image_Texture(theBuffer, theIdentifier) {
    }

    Handle(Image_PixMap) ReadImage(
        const Handle(Image_SupportedFormats)&) const override
    {
        return DecodeRendererTextureWithImageIO(DataBuffer());
    }
};

Handle(Image_Texture) MakeRendererDecodableTexture(
    const Handle(NCollection_Buffer)& theBuffer,
    const TCollection_AsciiString& theIdentifier)
{
    if (theBuffer.IsNull() || theBuffer->Data() == nullptr
        || theBuffer->Size() == 0 || theIdentifier.IsEmpty()) {
        return {};
    }
    // Both authored textures and safe-loaded/import-artifact textures pass
    // through this factory before their first presentation. A binary update
    // also recreates the GL context, so an old failed GPU-resource cache can
    // never outlive the source upgrade performed here.
    return new Core3DImageIOTexture(theBuffer, theIdentifier);
}

bool ValidateAuthoredRasterBytes(const Standard_Byte* theBytes,
                                 const Standard_Size theSize,
                                 const std::string* theMediaType)
{
    if (theBytes == nullptr || theSize == 0
        || theSize > kMaximumEmbeddedTextureBytes) {
        return false;
    }
    const bool isPNG = IsPNGSignature(theBytes, theSize);
    const bool isJPEG = IsJPEGSignature(theBytes, theSize);
    if (!isPNG && !isJPEG) {
        return false;
    }
    if (theMediaType != nullptr
        && !((*theMediaType == "image/png" && isPNG)
             || (*theMediaType == "image/jpeg" && isJPEG))) {
        return false;
    }

    CFDataRef data = CFDataCreateWithBytesNoCopy(
        kCFAllocatorDefault,
        reinterpret_cast<const UInt8*>(theBytes),
        static_cast<CFIndex>(theSize),
        kCFAllocatorNull);
    if (data == nullptr) {
        return false;
    }
    const void* optionKeys[] = {kCGImageSourceShouldCache};
    const void* optionValues[] = {kCFBooleanFalse};
    CFDictionaryRef options = CFDictionaryCreate(
        kCFAllocatorDefault,
        optionKeys,
        optionValues,
        1,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    CGImageSourceRef source = CGImageSourceCreateWithData(data, options);
    if (options != nullptr) {
        CFRelease(options);
    }
    CFRelease(data);
    if (source == nullptr || CGImageSourceGetType(source) == nullptr
        || CGImageSourceGetCount(source) != 1
        || CGImageSourceGetStatus(source) != kCGImageStatusComplete
        || CGImageSourceGetStatusAtIndex(source, 0)
            != kCGImageStatusComplete) {
        if (source != nullptr) {
            CFRelease(source);
        }
        return false;
    }

    CFDictionaryRef properties =
        CGImageSourceCopyPropertiesAtIndex(source, 0, nullptr);
    CFRelease(source);
    if (properties == nullptr) {
        return false;
    }
    const CFTypeRef widthValue = CFDictionaryGetValue(
        properties, kCGImagePropertyPixelWidth);
    const CFTypeRef heightValue = CFDictionaryGetValue(
        properties, kCGImagePropertyPixelHeight);
    const CFTypeRef depthValue = CFDictionaryGetValue(
        properties, kCGImagePropertyDepth);
    std::int64_t width = 0;
    std::int64_t height = 0;
    std::int64_t depth = 8;
    const bool hasValidDepth = depthValue == nullptr
        || (CFGetTypeID(depthValue) == CFNumberGetTypeID()
            && CFNumberGetValue(
                static_cast<CFNumberRef>(depthValue),
                kCFNumberSInt64Type,
                &depth));
    const bool isValid = widthValue != nullptr
        && heightValue != nullptr
        && CFGetTypeID(widthValue) == CFNumberGetTypeID()
        && CFGetTypeID(heightValue) == CFNumberGetTypeID()
        && CFNumberGetValue(
            static_cast<CFNumberRef>(widthValue),
            kCFNumberSInt64Type,
            &width)
        && CFNumberGetValue(
            static_cast<CFNumberRef>(heightValue),
            kCFNumberSInt64Type,
            &height)
        && hasValidDepth && depth > 0 && depth <= 8
        && width > 0 && height > 0
        && static_cast<std::uint64_t>(width)
            <= kMaximumTextureDimension
        && static_cast<std::uint64_t>(height)
            <= kMaximumTextureDimension
        && static_cast<std::uint64_t>(width)
            <= kMaximumTexturePixels
                / static_cast<std::uint64_t>(height);
    bool isWithinDecodedBudget = false;
    if (isValid) {
        const std::uint64_t pixels = static_cast<std::uint64_t>(width)
            * static_cast<std::uint64_t>(height);
        isWithinDecodedBudget = pixels
            <= static_cast<std::uint64_t>(kMaximumDecodedTextureBytes) / 4;
    }
    CFRelease(properties);
    return isValid && isWithinDecodedBudget;
}

std::string AuthoredTextureIdentifier(const Standard_Byte* theBytes,
                                      const Standard_Size theSize)
{
    if (theBytes == nullptr || theSize == 0
        || theSize > std::numeric_limits<CC_LONG>::max()) {
        return {};
    }
    std::array<unsigned char, CC_SHA256_DIGEST_LENGTH> digest = {};
    if (CC_SHA256(theBytes, static_cast<CC_LONG>(theSize), digest.data())
        == nullptr) {
        return {};
    }
    std::ostringstream stream;
    stream << "texture-sha256-" << std::hex << std::setfill('0');
    for (const unsigned char byte : digest) {
        stream << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return stream.str();
}

template <typename Driver>
class Core3DFailClosedDriver final : public Driver
{
public:
    explicit Core3DFailClosedDriver(
        const Handle(Message_Messenger)& theMessageDriver)
    : Driver(theMessageDriver)
    {
    }

    Standard_Boolean Paste(
        const BinObjMgt_Persistent& theSource,
        const Handle(TDF_Attribute)& theTarget,
        BinObjMgt_RRelocationTable& theRelocationTable) const override
    {
        const Standard_Boolean succeeded = Driver::Paste(
            theSource, theTarget, theRelocationTable);
        if (!succeeded) {
            RejectSafeBinaryRead();
        }
        return succeeded;
    }
};

class Core3DBoundedAsciiStringDriver final
    : public BinMDataStd_AsciiStringDriver
{
public:
    explicit Core3DBoundedAsciiStringDriver(
        const Handle(Message_Messenger)& theMessageDriver)
    : BinMDataStd_AsciiStringDriver(theMessageDriver)
    {
    }

    Standard_Boolean Paste(
        const BinObjMgt_Persistent& theSource,
        const Handle(TDF_Attribute)& theTarget,
        BinObjMgt_RRelocationTable& theRelocationTable) const override
    {
        const Standard_Integer aStart = theSource.Position();
        std::string aValue;
        if (!ReadBoundedPersistentAsciiString(theSource, aValue)
            || !theSource.SetPosition(aStart)) {
            RejectSafeBinaryRead();
            return Standard_False;
        }
        return BinMDataStd_AsciiStringDriver::Paste(
            theSource, theTarget, theRelocationTable);
    }
};

class Core3DBoundedExtendedStringDriver final
    : public BinMDataStd_GenericExtStringDriver
{
public:
    explicit Core3DBoundedExtendedStringDriver(
        const Handle(Message_Messenger)& theMessageDriver)
    : BinMDataStd_GenericExtStringDriver(theMessageDriver)
    {
    }

    Standard_Boolean Paste(
        const BinObjMgt_Persistent& theSource,
        const Handle(TDF_Attribute)& theTarget,
        BinObjMgt_RRelocationTable& theRelocationTable) const override
    {
        if (!PreflightBoundedPersistentExtendedString(theSource)) {
            RejectSafeBinaryRead();
            return Standard_False;
        }
        return BinMDataStd_GenericExtStringDriver::Paste(
            theSource, theTarget, theRelocationTable);
    }
};

class Core3DBoundedVisMaterialDriver final : public BinMDF_ADriver
{
public:
    Core3DBoundedVisMaterialDriver(
        const Handle(Message_Messenger)& theMessageDriver,
        const std::shared_ptr<Standard_Size>& theAggregateTextureBytes)
    : BinMDF_ADriver(
          theMessageDriver,
          STANDARD_TYPE(XCAFDoc_VisMaterial)->Name()),
      myDelegate(new BinMXCAFDoc_VisMaterialDriver(theMessageDriver)),
      myAggregateTextureBytes(theAggregateTextureBytes)
    {
    }

    Handle(TDF_Attribute) NewEmpty() const override
    {
        return new XCAFDoc_VisMaterial();
    }

    Standard_Boolean Paste(
        const BinObjMgt_Persistent& theSource,
        const Handle(TDF_Attribute)& theTarget,
        BinObjMgt_RRelocationTable& theRelocationTable) const override
    {
        const Standard_Integer aStart = theSource.Position();
        auto restoreStart = [&]() {
            return theSource.SetPosition(aStart);
        };
        Standard_Byte aMajor = 0;
        Standard_Byte aMinor = 0;
        Standard_Byte aFaceCulling = 0;
        Standard_Byte anAlphaMode = 0;
        Standard_Boolean hasPBR = Standard_False;
        Standard_Boolean hasCommon = Standard_False;
        Standard_Size anAggregate = myAggregateTextureBytes == nullptr
            ? 0
            : *myAggregateTextureBytes;

        bool isSafe = theSource.GetByte(aMajor).IsOK()
            && theSource.GetByte(aMinor).IsOK()
            && aMajor == 1 && aMinor <= 1
            && theSource.GetByte(aFaceCulling).IsOK()
            && theSource.GetByte(anAlphaMode).IsOK()
            && (aFaceCulling == static_cast<Standard_Byte>('0')
                || aFaceCulling == static_cast<Standard_Byte>('B')
                || aFaceCulling == static_cast<Standard_Byte>('F')
                || aFaceCulling == static_cast<Standard_Byte>('1'))
            && (anAlphaMode == static_cast<Standard_Byte>('O')
                || anAlphaMode == static_cast<Standard_Byte>('M')
                || anAlphaMode == static_cast<Standard_Byte>('B')
                || anAlphaMode == static_cast<Standard_Byte>('b')
                || anAlphaMode == static_cast<Standard_Byte>('A'))
            && SkipShortReals(theSource, 1)
            && theSource.GetBoolean(hasPBR).IsOK();
        if (isSafe && hasPBR) {
            isSafe = SkipShortReals(theSource, 4 + 3 + 2)
                && PreflightEmbeddedTexture(theSource, anAggregate)
                && PreflightEmbeddedTexture(theSource, anAggregate)
                && PreflightEmbeddedTexture(theSource, anAggregate)
                && PreflightEmbeddedTexture(theSource, anAggregate)
                && PreflightEmbeddedTexture(theSource, anAggregate);
        }
        if (isSafe) {
            isSafe = theSource.GetBoolean(hasCommon).IsOK();
        }
        if (isSafe && hasCommon) {
            isSafe = SkipShortReals(theSource, 3 * 4 + 2)
                && PreflightEmbeddedTexture(theSource, anAggregate);
        }
        if (isSafe && hasPBR && aMinor >= 1) {
            isSafe = SkipShortReals(theSource, 1);
        }
        if (!isSafe || !theSource.IsOK() || !restoreStart()) {
            restoreStart();
            RejectSafeBinaryRead();
            return Standard_False;
        }

        if (!myDelegate->Paste(
                theSource, theTarget, theRelocationTable)) {
            RejectSafeBinaryRead();
            return Standard_False;
        }
        const Handle(XCAFDoc_VisMaterial) aMaterial =
            Handle(XCAFDoc_VisMaterial)::DownCast(theTarget);
        if (aMaterial.IsNull()) {
            RejectSafeBinaryRead();
            return Standard_False;
        }
        if (aMaterial->HasPbrMaterial()) {
            XCAFDoc_VisMaterialPBR aPBR = aMaterial->PbrMaterial();
            if (!NormalizeEmbeddedTexture(aPBR.BaseColorTexture)
                || !NormalizeEmbeddedTexture(
                    aPBR.MetallicRoughnessTexture)
                || !NormalizeEmbeddedTexture(aPBR.EmissiveTexture)
                || !NormalizeEmbeddedTexture(aPBR.OcclusionTexture)
                || !NormalizeEmbeddedTexture(aPBR.NormalTexture)) {
                RejectSafeBinaryRead();
                return Standard_False;
            }
            aMaterial->SetPbrMaterial(aPBR);
        }
        if (aMaterial->HasCommonMaterial()) {
            XCAFDoc_VisMaterialCommon aCommon =
                aMaterial->CommonMaterial();
            if (!NormalizeEmbeddedTexture(aCommon.DiffuseTexture)) {
                RejectSafeBinaryRead();
                return Standard_False;
            }
            aMaterial->SetCommonMaterial(aCommon);
        }
        if (myAggregateTextureBytes != nullptr) {
            *myAggregateTextureBytes = anAggregate;
        }
        return Standard_True;
    }

    void Paste(
        const Handle(TDF_Attribute)& theSource,
        BinObjMgt_Persistent& theTarget,
        BinObjMgt_SRelocationTable& theRelocationTable) const override
    {
        myDelegate->Paste(theSource, theTarget, theRelocationTable);
    }

private:
    Handle(BinMXCAFDoc_VisMaterialDriver) myDelegate;
    std::shared_ptr<Standard_Size> myAggregateTextureBytes;
};

class Core3DBoundedBinXCAFRetrievalDriver final
    : public BinDrivers_DocumentRetrievalDriver
{
public:
    Core3DBoundedBinXCAFRetrievalDriver()
    : myAggregateTextureBytes(std::make_shared<Standard_Size>(0)),
      myFrameBudget(std::make_shared<core3d::persistence::AuthoredFrameReadBudget>())
    {
        myReceiptLimits=core3d::receipt::v3::ReaderLimits();
        auto budget=std::make_shared<core3d::persistence::receipt_framing::LoadBudget>();
        budget->maximumWireBytes=myReceiptLimits.wireBytes;
        myReceiptFrameDriver=new core3d::receipt::v3::BinaryDriver(std::move(budget));
    }

#if DEBUG
    explicit Core3DBoundedBinXCAFRetrievalDriver(
        std::shared_ptr<core3d::persistence::AuthoredFrameReadBudget> budget)
        : Core3DBoundedBinXCAFRetrievalDriver() { myFrameBudget = std::move(budget); myValidateFrameOwners = false; }
#endif

#if DEBUG
    Core3DBoundedBinXCAFRetrievalDriver(
        const Handle(core3d::persistence::receipt_framing::FrameDriver)& driver,
        core3d::persistence::receipt_framing::TraversalLimits limits)
        : Core3DBoundedBinXCAFRetrievalDriver() {
        myReceiptFrameDriver = driver; myReceiptLimits = limits;
    }
#endif

    void Read(
        Standard_IStream& theStream,
        const Handle(Storage_Data)& theStorageData,
        const Handle(CDM_Document)& theDocument,
        const Handle(CDM_Application)& theApplication,
        const Handle(PCDM_ReaderFilter)& theFilter =
            Handle(PCDM_ReaderFilter)(),
        const Message_ProgressRange& theProgress =
            Message_ProgressRange()) override
    {
#if DEBUG
        ++myRetainedReadCount;
#endif
        // Reentrant reuse must not replace the exact active load's driver/budget.
        if (!myReceiptFrameDriver.IsNull() && myReceiptLoadActive) {
            if (!myReceiptFrameDriver.IsNull() && myReceiptFrameDriver->budget())
                myReceiptFrameDriver->budget()->refuse();
            myReaderStatus = PCDM_RS_TypeFailure; RejectSafeBinaryRead(); return;
        }
        struct ReceiptScope {
            Core3DBoundedBinXCAFRetrievalDriver* owner;
            bool active = false;
            ~ReceiptScope() { if (active) { owner->myReceiptTraversal.reset(); owner->myReceiptLoadActive = false; } }
        } receiptScope{this};
        ResetAggregateReadBudgets();
        const auto rejectTypes = [&]() {
            myReaderStatus = PCDM_RS_TypeFailure;
            RejectSafeBinaryRead();
        };
        if (theStorageData.IsNull() || theStorageData->HeaderData().IsNull()) {
            rejectTypes(); return;
        }
        const auto header = theStorageData->HeaderData();
        if (!header->StorageVersion().IsIntegerValue()) { rejectTypes(); return; }
        const auto version = header->StorageVersion().IntegerValue();
        if (version < TDocStd_FormatVersion_LOWER || version > TDocStd_FormatVersion_CURRENT) {
            // Preserve the caller's UnsupportedVersion result for a different
            // document format; this is distinct from malformed attribute data.
            myReaderStatus = PCDM_RS_NoVersion; return;
        }
        // BinLDrivers resolves attribute IDs from this UserInfo section,
        // not Storage_Data::TypeData (the unrelated storage-object table).
        // Reject unsupported attributes before OCCT can silently skip them.
        const auto& info = header->UserInfo();
        if (info.Length() < 2 || info.Length() > 1024) { rejectTypes(); return; }
        TColStd_SequenceOfAsciiString aTypeNames;
        bool began = false, ended = false;
        for (Standard_Integer i = 1; i <= info.Length(); ++i) {
            const auto& line = info.Value(i);
            if (line == "START_TYPES") {
                if (began || ended) { rejectTypes(); return; }
                began = true; continue;
            }
            if (line == "END_TYPES") {
                if (!began || ended) { rejectTypes(); return; }
                ended = true; continue;
            }
            if (!began || ended) continue;
            if (line.IsEmpty() || line.Length() > 128 || aTypeNames.Length() >= 128) {
                rejectTypes(); return;
            }
            TCollection_AsciiString name = line;
            if (version < TDocStd_FormatVersion_VERSION_8) {
                TCollection_AsciiString migrated;
                if (Storage_Schema::CheckTypeMigration(name, migrated)) name = migrated;
            }
            if (name.IsEmpty() || name.Length() > 128) { rejectTypes(); return; }
            for (Standard_Integer j = 1; j <= aTypeNames.Length(); ++j)
                if (aTypeNames.Value(j) == name) { rejectTypes(); return; }
            aTypeNames.Append(name);
        }
        if (!began || !ended) { rejectTypes(); return; }
        Handle(BinMDF_ADriverTable) aSupportedDrivers =
            AttributeDrivers(Message::DefaultMessenger());
        aSupportedDrivers->AssignIds(aTypeNames);
        for (Standard_Integer anIndex = 1;
             anIndex <= aTypeNames.Length(); ++anIndex) {
            if (aSupportedDrivers->GetDriver(anIndex).IsNull()) {
                rejectTypes();
                return;
            }
        }
        if (!myReceiptFrameDriver.IsNull()) {
            Standard_Integer receiptType = 0;
            for (Standard_Integer i = 1; i <= aTypeNames.Length(); ++i)
                if (aTypeNames(i) == myReceiptFrameDriver->TypeName()) receiptType = i;
            if (receiptType != 0) {
                if (version < TDocStd_FormatVersion_VERSION_12 || !theFilter.IsNull() ||
                    !myReceiptLimits.valid() || !myReceiptFrameDriver->budget()) {
                    rejectTypes(); return;
                }
                myReceiptLoadActive = true; receiptScope.active = true;
                auto& budget = *myReceiptFrameDriver->budget();
                budget = core3d::persistence::receipt_framing::LoadBudget{};
                budget.maximumWireBytes = myReceiptLimits.wireBytes;
                try {
                    // Base Read must use precisely the validated assigned table.
                    myDrivers = aSupportedDrivers;
                    auto retainedRole=core3d::retained_solid::Assigned(myDrivers);
                    Standard_GUID retainedID=core3d::retained_solid::AttributeID();
#if DEBUG
                    if(myRetainedRoleFault==1)retainedRole.Nullify();
                    if(myRetainedRoleFault==2&&!retainedRole.IsNull()){
                        Handle(BinMDF_ADriver) nativeShapes;myDrivers->GetDriver(STANDARD_TYPE(TNaming_NamedShape),nativeShapes);
                        retainedRole=new core3d::retained_solid::BinaryDriver(Message::DefaultMessenger(),
                            Handle(BinMNaming_NamedShapeDriver)::DownCast(nativeShapes),myRetainedBudget,RejectSafeBinaryRead);
                    }
                    if(myRetainedRoleFault==3)retainedID=Standard_GUID("EFE0D323-2207-4EA5-AEBC-90FCC8924351");
#endif
                    myReceiptTraversal = std::make_unique<core3d::persistence::receipt_framing::Traversal>(
                        myDrivers, receiptType, myReceiptFrameDriver, myReceiptLimits, RejectSafeBinaryRead,
                        retainedRole,retainedID);
                } catch (...) { budget.refuse(); rejectTypes(); return; }
            }
        }
        try {
            BinDrivers_DocumentRetrievalDriver::Read(
                theStream,
                theStorageData,
                theDocument,
                theApplication,
                theFilter,
                theProgress);
            if (myReaderStatus == PCDM_RS_OK && myReceiptTraversal && !myReceiptTraversal->complete()) {
                rejectTypes();
            }
            if(myReaderStatus==PCDM_RS_OK&&myReceiptTraversal&&
               myReceiptFrameDriver->AttributeID()==core3d::receipt::ScalableSchemaID()){
                core3d::receipt::Catalog catalog;
                if(core3d::receipt::Read(Handle(TDocStd_Document)::DownCast(theDocument),catalog)
                   !=core3d::receipt::ReadStatus::Valid||!catalog.tree)rejectTypes();
            }
            if(myReaderStatus==PCDM_RS_OK&&myRetainedBudget
                &&(myRetainedBudget->rejected||(myRetainedBudget->records&&!Core3DValidateRetainedSolidDocument(
                    Handle(TDocStd_Document)::DownCast(theDocument)))))rejectTypes();
            if(myReaderStatus==PCDM_RS_OK&&myBoundedCurveBudget
                &&(myBoundedCurveBudget->rejected||(myBoundedCurveBudget->records
                    &&!Core3DValidateBoundedCurveDocument(
                        Handle(TDocStd_Document)::DownCast(theDocument)))))rejectTypes();
            if (myValidateFrameOwners && myReaderStatus == PCDM_RS_OK) {
                Standard_Size frameBytes = 0;
                if (gSafeBinaryReadRejected || !Core3DValidateAuthoredFrameOwners(
                        Handle(TDocStd_Document)::DownCast(theDocument), frameBytes)) rejectTypes();
            }
        } catch (...) {
            ResetAggregateReadBudgets();
            throw;
        }
        ResetAggregateReadBudgets();
    }

#if DEBUG
    unsigned DebugRetainedReadCount()const{return myRetainedReadCount;}
    void DebugRejectRetainedSolidType(){myAllowRetainedSolid=false;}
    void DebugSetRetainedRoleFault(int fault){myRetainedRoleFault=fault;}
    void DebugSetRetainedEnvelopeLimit(std::size_t limit){myRetainedBudget->limit=std::min(limit,core3d::retained_solid::MaximumAggregateEnvelopeBytes);}
#endif

    Handle(BinMDF_ADriverTable) AttributeDrivers(
        const Handle(Message_Messenger)& theMessageDriver) override
    {
        // Project files intentionally support a narrow OCAF schema. Starting
        // with BinDrivers::AttributeDrivers() would expose unrelated array,
        // list, named-data, function, note, and geometric-constraint readers
        // that trust attacker-controlled element counts before app validation.
        Handle(BinMDF_ADriverTable) aTable = new BinMDF_ADriverTable();
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDF_TagSourceDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_GenericEmptyDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DBoundedExtendedStringDriver(theMessageDriver));
        aTable->AddDriver(
            new Core3DBoundedAsciiStringDriver(theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_IntegerDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_RealDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_TreeNodeDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_UAttributeDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMXCAFDoc_ColorDriver>(
                theMessageDriver));

        const Handle(BinMNaming_NamedShapeDriver) aNamedShapeDriver =
            new Core3DFailClosedDriver<BinMNaming_NamedShapeDriver>(
                theMessageDriver);
        aTable->AddDriver(aNamedShapeDriver);
        Handle(BinMXCAFDoc_LocationDriver) aLocationDriver =
            new Core3DFailClosedDriver<BinMXCAFDoc_LocationDriver>(
                theMessageDriver);
        aLocationDriver->SetNSDriver(aNamedShapeDriver);
        aTable->AddDriver(aLocationDriver);
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMXCAFDoc_LengthUnitDriver>(
                theMessageDriver));
        aTable->AddDriver(new Core3DBoundedVisMaterialDriver(
            theMessageDriver, myAggregateTextureBytes));
        aTable->AddDriver(
            new Core3DFailClosedDriver<
                BinMXCAFDoc_VisMaterialToolDriver>(theMessageDriver));
        aTable->AddDriver(new core3d::persistence::BoundedAuthoredFrameDriver(
            theMessageDriver, myFrameBudget, RejectSafeBinaryRead));
        if (myAllowRetainedSolid) core3d::retained_solid::Register(aTable,theMessageDriver,myRetainedBudget,RejectSafeBinaryRead);
        if (myAllowRetainedSolid) core3d::composite_recipe::Register(
            aTable,theMessageDriver,myRetainedBudget,myBoundedCurveBudget,RejectSafeBinaryRead);
        if (myAllowRetainedSolid) core3d::bounded_curve::Register(
            aTable,theMessageDriver,myBoundedCurveBudget,RejectSafeBinaryRead);
        if (!myReceiptFrameDriver.IsNull()) aTable->AddDriver(myReceiptFrameDriver);
        return aTable;
    }

    Standard_Integer ReadSubTree(Standard_IStream& stream, const TDF_Label& label,
        const Handle(PCDM_ReaderFilter)& filter, const Standard_Boolean& quick,
        const Message_ProgressRange& range) override {
        if (!myReceiptTraversal)
            return BinDrivers_DocumentRetrievalDriver::ReadSubTree(stream,label,filter,quick,range);
        const auto result = myReceiptTraversal->Read(stream,label,myDrivers,myRelocTable,filter,quick,range);
        if (result < 0) myReaderStatus = myReceiptTraversal->cancelled()
            ? PCDM_RS_UserBreak : PCDM_RS_UnrecognizedFileFormat;
        return result;
    }

    void Clear() override
    {
        try { BinDrivers_DocumentRetrievalDriver::Clear(); }
        catch (...) { ResetAggregateReadBudgets(); throw; }
        ResetAggregateReadBudgets();
    }

private:
    void ResetAggregateReadBudgets() noexcept
    {
        if (myFrameBudget) { myFrameBudget->bytes = 0; myFrameBudget->rejected = false; }
        if (myRetainedBudget) myRetainedBudget->reset();
        if (myBoundedCurveBudget) myBoundedCurveBudget->reset();
        if (myAggregateTextureBytes != nullptr) {
            *myAggregateTextureBytes = 0;
        }
    }

    std::shared_ptr<Standard_Size> myAggregateTextureBytes;
    std::shared_ptr<core3d::persistence::AuthoredFrameReadBudget> myFrameBudget;
    bool myValidateFrameOwners = true;
    Handle(core3d::persistence::receipt_framing::FrameDriver) myReceiptFrameDriver;
    core3d::persistence::receipt_framing::TraversalLimits myReceiptLimits;
    std::unique_ptr<core3d::persistence::receipt_framing::Traversal> myReceiptTraversal;
    bool myReceiptLoadActive = false;
    std::shared_ptr<core3d::retained_solid::ReadBudget> myRetainedBudget=std::make_shared<core3d::retained_solid::ReadBudget>();
    std::shared_ptr<core3d::bounded_curve::ReadBudget> myBoundedCurveBudget=
        std::make_shared<core3d::bounded_curve::ReadBudget>();
    bool myAllowRetainedSolid=true;
#if DEBUG
    int myRetainedRoleFault=0;
    unsigned myRetainedReadCount=0;
#endif
};

// These GUIDs are persistent schema identifiers. They identify the attribute
// role; the UUID string stored in each attribute identifies the document,
// occurrence, or shared shape definition itself.
const Standard_GUID& DocumentIdentifierAttributeID()
{
    static const Standard_GUID anId("74386E4E-F620-498F-8092-E6D883AF33A4");
    return anId;
}

const Standard_GUID& EntityIdentifierAttributeID()
{
    static const Standard_GUID anId("0074F7C2-9EAA-4F89-B2DE-8716E155FF62");
    return anId;
}

const Standard_GUID& DefinitionIdentifierAttributeID()
{
    static const Standard_GUID anId("3611F2B2-C694-4E12-AED8-A2A97A3D283B");
    return anId;
}

//! Definition-owned discriminator between editable BRep and imported
//! triangle-only geometry. This GUID and the non-negative enum values in the
//! public header are persistent schema identifiers.
const Standard_GUID& MeshUVAtlasAttributeID() {
    static const Standard_GUID id("9bf75aca-8719-4aca-9a5e-c7b4c3a40ba1");
    return id;
}

const Standard_GUID& MeshUVAtlasSettingsAttributeID(const int index) {
    // Fixed scalar attributes use the existing bounded binary schema. Never
    // broaden the document reader to arbitrary arrays for this three-value record.
    static const Standard_GUID ids[] = {
        Standard_GUID("137327fe-a06c-4be4-b45b-345f5343bda2"),
        Standard_GUID("29dd8d60-6fd2-4139-bbc5-9d8e9b469f9d"),
        Standard_GUID("a46a3ce4-2af3-4a77-a17d-b403e1503f13")
    };
    return ids[index];
}

const Standard_GUID& GeometryRepresentationAttributeID()
{
    static const Standard_GUID anId("67E669F4-00C0-4C45-BC55-9CC5DA22A2B5");
    return anId;
}

//! A reference-axis record is seven custom scalar attributes on one free,
//! simple definition label. These GUIDs and the encoded mode values are
//! permanent serialized schema identifiers: never renumber or reuse them.
const Standard_GUID& ReferenceAxisModeAttributeID()
{
    static const Standard_GUID anId("26128380-D69C-4530-B856-0C2AEEF47E60");
    return anId;
}

const Standard_GUID& ReferenceAxisPivotXAttributeID()
{
    static const Standard_GUID anId("11531A1D-14DB-4F78-AD91-814981846842");
    return anId;
}

const Standard_GUID& ReferenceAxisPivotYAttributeID()
{
    static const Standard_GUID anId("BA2AE804-64BD-480B-8910-B1144DA1AAD3");
    return anId;
}

const Standard_GUID& ReferenceAxisPivotZAttributeID()
{
    static const Standard_GUID anId("7CDD4B6E-5375-48F2-BAA9-AD76ACF6D47A");
    return anId;
}

const Standard_GUID& ReferenceAxisDirectionXAttributeID()
{
    static const Standard_GUID anId("CCEC34C3-8D3A-447A-8C70-EDB35A5B7E1C");
    return anId;
}

const Standard_GUID& ReferenceAxisDirectionYAttributeID()
{
    static const Standard_GUID anId("1DDBE964-693E-460A-9095-E47713BA28C0");
    return anId;
}

const Standard_GUID& ReferenceAxisDirectionZAttributeID()
{
    static const Standard_GUID anId("09CC9F05-9628-4C84-B214-2676C8BED8AA");
    return anId;
}

constexpr Standard_Integer kReferenceAxisSchemaV1 = 0x0100;
constexpr Standard_Integer kReferenceAxisPivotWorldBit = 0x0001;
constexpr Standard_Integer kReferenceAxisDirectionWorldBit = 0x0002;
constexpr Standard_Real kReferenceAxisUnitTolerance = 1.0e-10;

const std::array<const Standard_GUID*, 7>& ReferenceAxisAttributeIDs()
{
    static const std::array<const Standard_GUID*, 7> anIds = {{
        &ReferenceAxisModeAttributeID(),
        &ReferenceAxisPivotXAttributeID(),
        &ReferenceAxisPivotYAttributeID(),
        &ReferenceAxisPivotZAttributeID(),
        &ReferenceAxisDirectionXAttributeID(),
        &ReferenceAxisDirectionYAttributeID(),
        &ReferenceAxisDirectionZAttributeID(),
    }};
    return anIds;
}

constexpr Standard_Size kMaximumGeometryDocumentLabels = 100'000;
static_assert(core3d::profile::MaximumLabels == kMaximumGeometryDocumentLabels);

Standard_Boolean ValidateCommandOwnerSentinelsDocument(
    const Handle(TDocStd_Document)& theDocument)
{
    try {
        OCC_CATCH_SIGNALS
        if (theDocument.IsNull() || theDocument->GetData().IsNull()) {
            return Standard_False;
        }
        const TDF_Label aRoot = theDocument->GetData()->Root();
        const TDF_Label aMain = theDocument->Main();
        if (aRoot.IsNull() || aMain.IsNull()) {
            return Standard_False;
        }
        const auto isValidLabel = [&](const TDF_Label& theLabel) {
            const std::array<const Standard_GUID*, 4> anIds = {{
                &Core3DDuplicateCommandOwnerAttributeID(),
                &Core3DRadialArrayCommandOwnerAttributeID(),
                &Core3DOrdinaryEditCommandOwnerAttributeID(),
                &core3d::composite_recipe::spatial_g0::CommandOwnerAttributeID(),
            }};
            for (const Standard_GUID* anId : anIds) {
                Handle(TDF_Attribute) anAttribute;
                if (!theLabel.FindAttribute(*anId, anAttribute)) {
                    continue;
                }
                if (!theLabel.IsEqual(aMain)
                    || anAttribute.IsNull()
                    || Handle(TDataStd_Integer)::DownCast(
                        anAttribute).IsNull()) {
                    return false;
                }
            }
            return true;
        };
        if (!isValidLabel(aRoot)) {
            return Standard_False;
        }
        Standard_Size aLabelCount = 0;
        for (TDF_ChildIterator aLabel(aRoot, Standard_True);
             aLabel.More(); aLabel.Next()) {
            if (++aLabelCount > kMaximumGeometryDocumentLabels
                || !isValidLabel(aLabel.Value())) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

constexpr Standard_Size kMaximumGeometryDefinitionLabels = 4'096;
constexpr Standard_Size kMaximumSubshapesPerDefinition = 8'192;
constexpr Standard_Size kMaximumSubshapesPerDocument = 131'072;
constexpr Standard_Size kMaximumTopologyDepth = 128;
constexpr Standard_Size kMaximumMeshVerticesPerDefinition = 1'000'000;
constexpr Standard_Size kMaximumMeshVerticesPerDocument = 1'500'000;
constexpr Standard_Size kMaximumMeshIndicesPerDefinition = 3'000'000;
constexpr Standard_Size kMaximumMeshIndicesPerDocument = 4'500'000;
static_assert(
    static_cast<Standard_Integer>(
        OcctGeometryRepresentation::LegacyUnknown) == 0);
static_assert(
    static_cast<Standard_Integer>(
        OcctGeometryRepresentation::BRep) == 1);
static_assert(
    static_cast<Standard_Integer>(
        OcctGeometryRepresentation::TriangleMesh) == 2);

enum class DefinitionGeometryClass
{
    Invalid,
    BRep,
    TriangleMesh,
};

struct GeometryValidationBudget
{
    Standard_Size subshapes = 0;
    Standard_Size meshVertices = 0;
    Standard_Size meshIndices = 0;
};

bool IsFiniteBoundedMeshCoordinate(const Standard_Real theValue) noexcept
{
    return std::isfinite(theValue)
        && std::abs(theValue)
            <= core3d::limits::kMaximumModelCoordinateMagnitude;
}

bool AddWithinLimit(Standard_Size& theAggregate,
                    const Standard_Size theValue,
                    const Standard_Size theMaximum) noexcept
{
    if (theAggregate > theMaximum
        || theValue > theMaximum - theAggregate) {
        return false;
    }
    theAggregate += theValue;
    return true;
}

bool AddMultipliedWithinLimit(Standard_Size& theAggregate,
                              const Standard_Size theValue,
                              const Standard_Size theMultiplier,
                              const Standard_Size theMaximum) noexcept
{
    if (theAggregate > theMaximum) {
        return false;
    }
    const Standard_Size aRemaining = theMaximum - theAggregate;
    if (theValue != 0U && theMultiplier > aRemaining / theValue) {
        return false;
    }
    theAggregate += theValue * theMultiplier;
    return true;
}

bool IsGeometryDefinitionLabel(
    const Handle(TDocStd_Document)& theDocument,
    const Handle(XCAFDoc_ShapeTool)& theShapeTool,
    const TDF_Label& theLabel)
{
    return !theDocument.IsNull() && !theShapeTool.IsNull()
        && !theLabel.IsNull()
        && theLabel.Data() == theDocument->GetData()
        && theShapeTool->IsShape(theLabel)
        && XCAFDoc_ShapeTool::IsSimpleShape(theLabel)
        && !XCAFDoc_ShapeTool::IsAssembly(theLabel)
        && !XCAFDoc_ShapeTool::IsReference(theLabel)
        && !XCAFDoc_ShapeTool::IsComponent(theLabel)
        && !XCAFDoc_ShapeTool::IsSubShape(theLabel);
}

OcctReferenceAxis DefaultReferenceAxis()
{
    OcctReferenceAxis anAxis;
    anAxis.pivotSpace = OcctReferenceSpace::Object;
    anAxis.pivot = gp_Pnt(0.0, 0.0, 0.0);
    anAxis.directionSpace = OcctReferenceSpace::World;
    anAxis.direction = gp_Dir(0.0, 0.0, 1.0);
    return anAxis;
}

bool IsReferenceSpace(const OcctReferenceSpace theSpace) noexcept
{
    return theSpace == OcctReferenceSpace::Object
        || theSpace == OcctReferenceSpace::World;
}

bool IsFiniteBoundedReferencePoint(const gp_Pnt& thePoint) noexcept
{
    return IsFiniteBoundedMeshCoordinate(thePoint.X())
        && IsFiniteBoundedMeshCoordinate(thePoint.Y())
        && IsFiniteBoundedMeshCoordinate(thePoint.Z());
}

bool IsFiniteReferenceDirection(const gp_Dir& theDirection) noexcept
{
    const Standard_Real aSquaredLength =
        theDirection.X() * theDirection.X()
        + theDirection.Y() * theDirection.Y()
        + theDirection.Z() * theDirection.Z();
    return std::isfinite(theDirection.X())
        && std::isfinite(theDirection.Y())
        && std::isfinite(theDirection.Z())
        && std::isfinite(aSquaredLength)
        && std::abs(aSquaredLength - 1.0)
            <= kReferenceAxisUnitTolerance;
}

Standard_Real CanonicalReferenceScalar(const Standard_Real theValue) noexcept
{
    return theValue == 0.0 ? 0.0 : theValue;
}

bool ReferenceAxesMatch(
    const OcctReferenceAxis& theLeft,
    const OcctReferenceAxis& theRight,
    const Standard_Real theTolerance = 1.0e-12) noexcept
{
    return theLeft.pivotSpace == theRight.pivotSpace
        && theLeft.directionSpace == theRight.directionSpace
        && theLeft.pivot.IsEqual(theRight.pivot, theTolerance)
        && theLeft.direction.IsEqual(theRight.direction, theTolerance);
}

Standard_Integer EncodedReferenceAxisMode(
    const OcctReferenceAxis& theAxis) noexcept
{
    return kReferenceAxisSchemaV1
        | (theAxis.pivotSpace == OcctReferenceSpace::World
            ? kReferenceAxisPivotWorldBit : 0)
        | (theAxis.directionSpace == OcctReferenceSpace::World
            ? kReferenceAxisDirectionWorldBit : 0);
}

OcctReferenceAxisReadState ReadReferenceAxisRecord(
    const TDF_Label& theLabel,
    OcctReferenceAxis& theAxis)
{
    theAxis = DefaultReferenceAxis();
    if (theLabel.IsNull()) {
        return OcctReferenceAxisReadState::Invalid;
    }

    const auto& anIds = ReferenceAxisAttributeIDs();
    std::array<Handle(TDF_Attribute), 7> anAttributes;
    Standard_Size aPresentCount = 0;
    for (std::size_t anIndex = 0; anIndex < anIds.size(); ++anIndex) {
        if (theLabel.FindAttribute(*anIds[anIndex], anAttributes[anIndex])) {
            ++aPresentCount;
        }
    }
    if (aPresentCount == 0U) {
        return OcctReferenceAxisReadState::ImplicitDefault;
    }
    if (aPresentCount != anIds.size()) {
        return OcctReferenceAxisReadState::Invalid;
    }

    const Handle(TDataStd_Integer) aMode =
        Handle(TDataStd_Integer)::DownCast(anAttributes[0]);
    if (aMode.IsNull()) {
        return OcctReferenceAxisReadState::Invalid;
    }
    const Standard_Integer aModeValue = aMode->Get();
    if ((aModeValue & ~0x0003) != kReferenceAxisSchemaV1) {
        return OcctReferenceAxisReadState::Invalid;
    }

    Standard_Real aValues[6] = {};
    for (std::size_t anIndex = 0; anIndex < 6U; ++anIndex) {
        const Handle(TDataStd_Real) aValue =
            Handle(TDataStd_Real)::DownCast(anAttributes[anIndex + 1U]);
        if (aValue.IsNull() || !std::isfinite(aValue->Get())) {
            return OcctReferenceAxisReadState::Invalid;
        }
        aValues[anIndex] = aValue->Get();
    }

    const gp_Pnt aPivot(aValues[0], aValues[1], aValues[2]);
    if (!IsFiniteBoundedReferencePoint(aPivot)) {
        return OcctReferenceAxisReadState::Invalid;
    }
    const Standard_Real aDirectionSquaredLength =
        aValues[3] * aValues[3]
        + aValues[4] * aValues[4]
        + aValues[5] * aValues[5];
    if (!std::isfinite(aDirectionSquaredLength)
        || std::abs(aDirectionSquaredLength - 1.0)
            > kReferenceAxisUnitTolerance) {
        return OcctReferenceAxisReadState::Invalid;
    }

    try {
        OCC_CATCH_SIGNALS
        theAxis.pivotSpace =
            (aModeValue & kReferenceAxisPivotWorldBit) != 0
            ? OcctReferenceSpace::World : OcctReferenceSpace::Object;
        theAxis.pivot = aPivot;
        theAxis.directionSpace =
            (aModeValue & kReferenceAxisDirectionWorldBit) != 0
            ? OcctReferenceSpace::World : OcctReferenceSpace::Object;
        theAxis.direction = gp_Dir(aValues[3], aValues[4], aValues[5]);
        return IsFiniteReferenceDirection(theAxis.direction)
            ? OcctReferenceAxisReadState::Authored
            : OcctReferenceAxisReadState::Invalid;
    } catch (...) {
        theAxis = DefaultReferenceAxis();
        return OcctReferenceAxisReadState::Invalid;
    }
}

bool WriteReferenceAxisRecord(
    const TDF_Label& theLabel,
    const OcctReferenceAxis& theAxis)
{
    if (theLabel.IsNull()
        || !IsReferenceSpace(theAxis.pivotSpace)
        || !IsReferenceSpace(theAxis.directionSpace)
        || !IsFiniteBoundedReferencePoint(theAxis.pivot)
        || !IsFiniteReferenceDirection(theAxis.direction)) {
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
        TDataStd_Integer::Set(
            theLabel,
            ReferenceAxisModeAttributeID(),
            EncodedReferenceAxisMode(theAxis));
        TDataStd_Real::Set(
            theLabel,
            ReferenceAxisPivotXAttributeID(),
            CanonicalReferenceScalar(theAxis.pivot.X()));
        TDataStd_Real::Set(
            theLabel,
            ReferenceAxisPivotYAttributeID(),
            CanonicalReferenceScalar(theAxis.pivot.Y()));
        TDataStd_Real::Set(
            theLabel,
            ReferenceAxisPivotZAttributeID(),
            CanonicalReferenceScalar(theAxis.pivot.Z()));
        TDataStd_Real::Set(
            theLabel,
            ReferenceAxisDirectionXAttributeID(),
            CanonicalReferenceScalar(theAxis.direction.X()));
        TDataStd_Real::Set(
            theLabel,
            ReferenceAxisDirectionYAttributeID(),
            CanonicalReferenceScalar(theAxis.direction.Y()));
        TDataStd_Real::Set(
            theLabel,
            ReferenceAxisDirectionZAttributeID(),
            CanonicalReferenceScalar(theAxis.direction.Z()));

        OcctReferenceAxis aStored;
        return ReadReferenceAxisRecord(theLabel, aStored)
                == OcctReferenceAxisReadState::Authored
            && ReferenceAxesMatch(theAxis, aStored);
    } catch (...) {
        return false;
    }
}

bool HasAnyReferenceAxisAttribute(const TDF_Label& theLabel)
{
    if (theLabel.IsNull()) {
        return false;
    }
    for (const Standard_GUID* anId : ReferenceAxisAttributeIDs()) {
        if (anId != nullptr && theLabel.IsAttribute(*anId)) {
            return true;
        }
    }
    return false;
}

bool TryReadObjectTransform(
    const Handle(TDocStd_Document)& theDocument,
    const Handle(XCAFDoc_ShapeTool)& theShapeTool,
    const TDF_Label& theLabel,
    gp_Trsf& theTransform)
{
    if (!IsGeometryDefinitionLabel(theDocument, theShapeTool, theLabel)) {
        return false;
    }

    const Standard_Real aDefaults[8] = {
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        1.0,
    };
    Standard_Real aValues[8] = {};
    for (Standard_Integer anIndex = 0; anIndex < 8; ++anIndex) {
        aValues[anIndex] = aDefaults[anIndex];
        const TDF_Label aChild =
            theLabel.FindChild(anIndex + 1, Standard_False);
        if (!aChild.IsNull()) {
            Handle(TDataStd_Real) anAttribute;
            if (aChild.FindAttribute(TDataStd_Real::GetID(), anAttribute)) {
                if (anAttribute.IsNull()) {
                    return false;
                }
                aValues[anIndex] = anAttribute->Get();
            }
        }
        if (!std::isfinite(aValues[anIndex])) {
            return false;
        }
    }
    for (Standard_Integer anAxis = 0; anAxis < 3; ++anAxis) {
        if (std::abs(aValues[anAxis])
            > core3d::limits::kMaximumModelCoordinateMagnitude) {
            return false;
        }
    }
    if (std::abs(aValues[7])
        <= std::numeric_limits<Standard_Real>::epsilon()) {
        return false;
    }

    const Standard_Real aMaximumQuaternionComponent = std::max({
        std::abs(aValues[3]), std::abs(aValues[4]),
        std::abs(aValues[5]), std::abs(aValues[6]),
    });
    if (!std::isfinite(aMaximumQuaternionComponent)
        || aMaximumQuaternionComponent
            <= std::numeric_limits<Standard_Real>::min()) {
        return false;
    }
    Standard_Real aQuaternion[4] = {
        aValues[3] / aMaximumQuaternionComponent,
        aValues[4] / aMaximumQuaternionComponent,
        aValues[5] / aMaximumQuaternionComponent,
        aValues[6] / aMaximumQuaternionComponent,
    };
    const Standard_Real aQuaternionNorm = std::sqrt(
        aQuaternion[0] * aQuaternion[0]
        + aQuaternion[1] * aQuaternion[1]
        + aQuaternion[2] * aQuaternion[2]
        + aQuaternion[3] * aQuaternion[3]);
    if (!std::isfinite(aQuaternionNorm)
        || aQuaternionNorm
            <= std::numeric_limits<Standard_Real>::epsilon()) {
        return false;
    }
    for (Standard_Real& aComponent : aQuaternion) {
        aComponent /= aQuaternionNorm;
    }

    try {
        OCC_CATCH_SIGNALS
        gp_Trsf aTransform;
        aTransform.SetRotationPart(gp_Quaternion(
            aQuaternion[0], aQuaternion[1],
            aQuaternion[2], aQuaternion[3]));
        aTransform.SetScaleFactor(aValues[7]);
        aTransform.SetTranslationPart(
            gp_XYZ(aValues[0], aValues[1], aValues[2]));
        for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
            for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
                if (!std::isfinite(aTransform.Value(aRow, aColumn))) {
                    return false;
                }
            }
        }
        theTransform = aTransform;
        return true;
    } catch (...) {
        return false;
    }
}

bool TryResolveReferenceAxis(
    const Handle(TDocStd_Document)& theDocument,
    const Handle(XCAFDoc_ShapeTool)& theShapeTool,
    const TDF_Label& theLabel,
    const TopLoc_Location& theOccurrenceLocation,
    gp_Ax1& theWorldAxis)
{
    OcctReferenceAxis aReference;
    if (ReadReferenceAxisRecord(theLabel, aReference)
            == OcctReferenceAxisReadState::Invalid
        || !IsGeometryDefinitionLabel(
            theDocument, theShapeTool, theLabel)) {
        return false;
    }

    gp_Trsf anObjectTransform;
    if (!TryReadObjectTransform(
            theDocument, theShapeTool, theLabel, anObjectTransform)) {
        return false;
    }
    const gp_Trsf anOccurrenceTransform =
        theOccurrenceLocation.Transformation();
    for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
        for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
            if (!std::isfinite(
                    anOccurrenceTransform.Value(aRow, aColumn))) {
                return false;
            }
        }
    }
    const gp_Trsf anObjectToWorld =
        anObjectTransform.Multiplied(anOccurrenceTransform);

    gp_Pnt aWorldPivot = aReference.pivot;
    if (aReference.pivotSpace == OcctReferenceSpace::Object) {
        aWorldPivot.Transform(anObjectToWorld);
    }
    if (!IsFiniteBoundedReferencePoint(aWorldPivot)) {
        return false;
    }

    gp_Vec aWorldDirection(aReference.direction);
    if (aReference.directionSpace == OcctReferenceSpace::Object) {
        aWorldDirection.Transform(anObjectToWorld);
    }
    const Standard_Real aSquaredMagnitude = aWorldDirection.SquareMagnitude();
    if (!std::isfinite(aWorldDirection.X())
        || !std::isfinite(aWorldDirection.Y())
        || !std::isfinite(aWorldDirection.Z())
        || !std::isfinite(aSquaredMagnitude)
        || aSquaredMagnitude
            <= std::numeric_limits<Standard_Real>::epsilon()) {
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
        const gp_Dir aWorldDirectionUnit(aWorldDirection);
        if (!IsFiniteReferenceDirection(aWorldDirectionUnit)) {
            return false;
        }
        theWorldAxis = gp_Ax1(aWorldPivot, aWorldDirectionUnit);
        return true;
    } catch (...) {
        return false;
    }
}

Standard_Boolean ValidateReferenceAxisDocument(
    const Handle(TDocStd_Document)& theDocument)
{
    try {
        OCC_CATCH_SIGNALS
        if (theDocument.IsNull() || theDocument->GetData().IsNull()) {
            return Standard_False;
        }
        const TDF_Label aRoot = theDocument->GetData()->Root();
        if (HasAnyReferenceAxisAttribute(aRoot)) {
            return Standard_False;
        }

        const bool hasShapeTool =
            XCAFDoc_DocumentTool::CheckShapeTool(theDocument->Main());
        const Handle(XCAFDoc_ShapeTool) aShapeTool = hasShapeTool
            ? XCAFDoc_DocumentTool::ShapeTool(theDocument->Main())
            : Handle(XCAFDoc_ShapeTool)();
        if (hasShapeTool && aShapeTool.IsNull()) {
            return Standard_False;
        }

        Standard_Size aLabelCount = 0;
        for (TDF_ChildIterator aLabel(aRoot, Standard_True);
             aLabel.More(); aLabel.Next()) {
            if (++aLabelCount > kMaximumGeometryDocumentLabels) {
                return Standard_False;
            }
            const TDF_Label& aValue = aLabel.Value();
            const bool hasReferenceAxis =
                HasAnyReferenceAxisAttribute(aValue);
            const bool isGeometryDefinition = !aShapeTool.IsNull()
                && IsGeometryDefinitionLabel(
                    theDocument, aShapeTool, aValue);
            if (hasReferenceAxis) {
                if (!isGeometryDefinition
                    || !XCAFDoc_ShapeTool::IsFree(aValue)) {
                    return Standard_False;
                }
                OcctReferenceAxis anAxis;
                if (ReadReferenceAxisRecord(aValue, anAxis)
                        != OcctReferenceAxisReadState::Authored) {
                    return Standard_False;
                }
            }
            if (isGeometryDefinition) {
                // The implicit Object-Origin / World-Z axis is authority too.
                // Resolve every definition, not only definitions carrying an
                // authored record, so a malformed persisted object transform
                // cannot pass open/save admission and fail later publication.
                gp_Ax1 aResolved;
                if (!TryResolveReferenceAxis(
                        theDocument,
                        aShapeTool,
                        aValue,
                        TopLoc_Location(),
                        aResolved)) {
                    return Standard_False;
                }
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

bool ReadGeometryRepresentation(
    const TDF_Label& theLabel,
    bool& theHasMarker,
    OcctGeometryRepresentation& theRepresentation)
{
    theHasMarker = false;
    theRepresentation = OcctGeometryRepresentation::LegacyUnknown;
    if (theLabel.IsNull()) {
        theRepresentation = OcctGeometryRepresentation::Invalid;
        return false;
    }
    Handle(TDataStd_Integer) anAttribute;
    if (!theLabel.FindAttribute(
            GeometryRepresentationAttributeID(), anAttribute)) {
        return true;
    }
    theHasMarker = true;
    if (anAttribute.IsNull()) {
        theRepresentation = OcctGeometryRepresentation::Invalid;
        return false;
    }
    switch (anAttribute->Get()) {
        case static_cast<Standard_Integer>(
            OcctGeometryRepresentation::LegacyUnknown):
            theRepresentation =
                OcctGeometryRepresentation::LegacyUnknown;
            return true;
        case static_cast<Standard_Integer>(
            OcctGeometryRepresentation::BRep):
            theRepresentation = OcctGeometryRepresentation::BRep;
            return true;
        case static_cast<Standard_Integer>(
            OcctGeometryRepresentation::TriangleMesh):
            theRepresentation =
                OcctGeometryRepresentation::TriangleMesh;
            return true;
        default:
            theRepresentation = OcctGeometryRepresentation::Invalid;
            return false;
    }
}

bool WriteGeometryRepresentationMarker(
    const TDF_Label& theLabel,
    const OcctGeometryRepresentation theRepresentation)
{
    if (theLabel.IsNull()
        || theRepresentation == OcctGeometryRepresentation::Invalid) {
        return false;
    }
    TDataStd_Integer::Set(
        theLabel,
        GeometryRepresentationAttributeID(),
        static_cast<Standard_Integer>(theRepresentation));
    Handle(TDataStd_Integer) aStoredRepresentation;
    return theLabel.FindAttribute(
               GeometryRepresentationAttributeID(),
               aStoredRepresentation)
        && !aStoredRepresentation.IsNull()
        && aStoredRepresentation->Get()
            == static_cast<Standard_Integer>(theRepresentation);
}

bool IsValidMeshFace(
    const TopoDS_Face& theFace,
    Standard_Size& theDefinitionVertices,
    Standard_Size& theDefinitionIndices)
{
    if (theFace.IsNull()
        || (theFace.Orientation() != TopAbs_FORWARD
            && theFace.Orientation() != TopAbs_REVERSED)) {
        return false;
    }
    TopLoc_Location aLocation;
    const Handle(Poly_Triangulation)& aTriangulation =
        BRep_Tool::Triangulation(theFace, aLocation);
    if (aTriangulation.IsNull()
        || aTriangulation->HasDeferredData()
        || !aTriangulation->HasGeometry()
        || aTriangulation->NbNodes() <= 0
        || aTriangulation->NbTriangles() <= 0) {
        return false;
    }

    const Standard_Size aNodeCount =
        static_cast<Standard_Size>(aTriangulation->NbNodes());
    const Standard_Size aTriangleCount =
        static_cast<Standard_Size>(aTriangulation->NbTriangles());
    if (aTriangleCount
            > kMaximumMeshIndicesPerDefinition / 3U
        || !AddWithinLimit(
            theDefinitionVertices,
            aNodeCount,
            kMaximumMeshVerticesPerDefinition)
        || !AddWithinLimit(
            theDefinitionIndices,
            aTriangleCount * 3U,
            kMaximumMeshIndicesPerDefinition)) {
        return false;
    }

    const gp_Trsf aTransform = aLocation.Transformation();
    for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
        for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
            if (!std::isfinite(aTransform.Value(aRow, aColumn))) {
                return false;
            }
        }
    }
    for (Standard_Integer aNode = 1;
         aNode <= aTriangulation->NbNodes(); ++aNode) {
        const gp_Pnt& aStoredPoint = aTriangulation->Node(aNode);
        if (!IsFiniteBoundedMeshCoordinate(aStoredPoint.X())
            || !IsFiniteBoundedMeshCoordinate(aStoredPoint.Y())
            || !IsFiniteBoundedMeshCoordinate(aStoredPoint.Z())) {
            return false;
        }
        gp_Pnt aPoint = aStoredPoint;
        aPoint.Transform(aTransform);
        if (!IsFiniteBoundedMeshCoordinate(aPoint.X())
            || !IsFiniteBoundedMeshCoordinate(aPoint.Y())
            || !IsFiniteBoundedMeshCoordinate(aPoint.Z())) {
            return false;
        }
        if (aTriangulation->HasNormals()) {
            const gp_Dir aNormal = aTriangulation->Normal(aNode);
            const Standard_Real aSquaredLength =
                aNormal.X() * aNormal.X()
                + aNormal.Y() * aNormal.Y()
                + aNormal.Z() * aNormal.Z();
            if (!std::isfinite(aNormal.X())
                || !std::isfinite(aNormal.Y())
                || !std::isfinite(aNormal.Z())
                || !std::isfinite(aSquaredLength)
                || std::abs(aSquaredLength - 1.0) > 1.0e-3) {
                return false;
            }
        }
        if (aTriangulation->HasUVNodes()) {
            const gp_Pnt2d aUV = aTriangulation->UVNode(aNode);
            if (!std::isfinite(aUV.X()) || !std::isfinite(aUV.Y())) {
                return false;
            }
        }
    }

    for (Standard_Integer aTriangle = 1;
         aTriangle <= aTriangulation->NbTriangles(); ++aTriangle) {
        Standard_Integer aNodes[3] = {0, 0, 0};
        aTriangulation->Triangle(aTriangle).Get(
            aNodes[0], aNodes[1], aNodes[2]);
        for (const Standard_Integer aNode : aNodes) {
            if (aNode < 1 || aNode > aTriangulation->NbNodes()) {
                return false;
            }
        }
        if (aNodes[0] == aNodes[1]
            || aNodes[1] == aNodes[2]
            || aNodes[2] == aNodes[0]) {
            return false;
        }
        gp_Pnt aP0 = aTriangulation->Node(aNodes[0]);
        gp_Pnt aP1 = aTriangulation->Node(aNodes[1]);
        gp_Pnt aP2 = aTriangulation->Node(aNodes[2]);
        aP0.Transform(aTransform);
        aP1.Transform(aTransform);
        aP2.Transform(aTransform);
        const Standard_Real aUX = aP1.X() - aP0.X();
        const Standard_Real aUY = aP1.Y() - aP0.Y();
        const Standard_Real aUZ = aP1.Z() - aP0.Z();
        const Standard_Real aVX = aP2.X() - aP0.X();
        const Standard_Real aVY = aP2.Y() - aP0.Y();
        const Standard_Real aVZ = aP2.Z() - aP0.Z();
        const Standard_Real aCrossX = aUY * aVZ - aUZ * aVY;
        const Standard_Real aCrossY = aUZ * aVX - aUX * aVZ;
        const Standard_Real aCrossZ = aUX * aVY - aUY * aVX;
        const Standard_Real aSquaredArea =
            aCrossX * aCrossX
            + aCrossY * aCrossY
            + aCrossZ * aCrossZ;
        if (!std::isfinite(aSquaredArea) || aSquaredArea <= 0.0) {
            return false;
        }
    }
    return true;
}

bool IsValidGeometryTopologyOrientation(
    const TopAbs_Orientation theOrientation) noexcept
{
    switch (theOrientation) {
        case TopAbs_FORWARD:
        case TopAbs_REVERSED:
        case TopAbs_INTERNAL:
        case TopAbs_EXTERNAL:
            return true;
    }
    return false;
}

bool IsAllowedGeometryTopologyChild(
    const TopAbs_ShapeEnum theParent,
    const TopAbs_ShapeEnum theChild) noexcept
{
    switch (theParent) {
        case TopAbs_COMPOUND:
            return theChild >= TopAbs_COMPOUND
                && theChild < TopAbs_SHAPE;
        case TopAbs_COMPSOLID:
            return theChild == TopAbs_SOLID;
        case TopAbs_SOLID:
            return theChild == TopAbs_SHELL;
        case TopAbs_SHELL:
            return theChild == TopAbs_FACE;
        case TopAbs_FACE:
            return theChild == TopAbs_WIRE;
        case TopAbs_WIRE:
            return theChild == TopAbs_EDGE;
        case TopAbs_EDGE:
            return theChild == TopAbs_VERTEX;
        case TopAbs_VERTEX:
        case TopAbs_SHAPE:
            return false;
    }
    return false;
}

DefinitionGeometryClass ClassifyDefinitionGeometry(
    const TopoDS_Shape& theShape,
    GeometryValidationBudget* theBudget)
{
    if (theShape.IsNull()) {
        return DefinitionGeometryClass::Invalid;
    }
    try {
        OCC_CATCH_SIGNALS
        struct TopologyFrame {
            TopoDS_Shape shape;
            Standard_Size depth = 0;
            bool leaving = false;
        };
        std::vector<TopologyFrame> aStack = {{theShape, 0, false}};
        TopTools_MapOfShape aVisited;
        TopTools_MapOfShape anActivePath;
        Standard_Size aSubshapeCount = 0;
        Standard_Size aVertexCount = 0;
        Standard_Size anIndexCount = 0;
        bool hasBRepFace = false;
        bool hasMeshFace = false;

        while (!aStack.empty()) {
            const TopologyFrame aFrame = aStack.back();
            aStack.pop_back();
            if (aFrame.shape.IsNull()) {
                return DefinitionGeometryClass::Invalid;
            }
            if (aFrame.leaving) {
                anActivePath.Remove(aFrame.shape);
                aVisited.Add(aFrame.shape);
                continue;
            }
            if (aVisited.Contains(aFrame.shape)) {
                continue;
            }
            if (anActivePath.Contains(aFrame.shape)
                || aFrame.shape.ShapeType() == TopAbs_SHAPE
                || !IsValidGeometryTopologyOrientation(
                    aFrame.shape.Orientation())
                || aFrame.depth > kMaximumTopologyDepth
                || aSubshapeCount
                    >= kMaximumSubshapesPerDefinition) {
                return DefinitionGeometryClass::Invalid;
            }
            anActivePath.Add(aFrame.shape);
            ++aSubshapeCount;

            if (aFrame.shape.ShapeType() == TopAbs_FACE) {
                const TopoDS_Face aFace =
                    TopoDS::Face(aFrame.shape);
                if (!BRep_Tool::Surface(aFace).IsNull()) {
                    hasBRepFace = true;
                    if (hasMeshFace) {
                        return DefinitionGeometryClass::Invalid;
                    }
                } else {
                    hasMeshFace = true;
                    if (hasBRepFace
                        || !IsValidMeshFace(
                            aFace, aVertexCount, anIndexCount)) {
                        return DefinitionGeometryClass::Invalid;
                    }
                }
            }

            if (aStack.size()
                >= static_cast<std::size_t>(
                    kMaximumSubshapesPerDefinition) * 2U) {
                return DefinitionGeometryClass::Invalid;
            }
            aStack.push_back({
                aFrame.shape, aFrame.depth, true});
            const TopAbs_ShapeEnum aParentType =
                aFrame.shape.ShapeType();
            for (TopoDS_Iterator aChild(
                     aFrame.shape, Standard_True, Standard_True);
                 aChild.More(); aChild.Next()) {
                const TopoDS_Shape& aChildShape = aChild.Value();
                if (aChildShape.IsNull()
                    || !IsAllowedGeometryTopologyChild(
                        aParentType, aChildShape.ShapeType())
                    || !IsValidGeometryTopologyOrientation(
                        aChildShape.Orientation())
                    || aFrame.depth >= kMaximumTopologyDepth
                    || aStack.size()
                        >= static_cast<std::size_t>(
                            kMaximumSubshapesPerDefinition) * 2U) {
                    return DefinitionGeometryClass::Invalid;
                }
                aStack.push_back({
                    aChildShape, aFrame.depth + 1U, false});
            }
        }

        if (theBudget != nullptr
            && (!AddWithinLimit(
                    theBudget->subshapes,
                    aSubshapeCount,
                    kMaximumSubshapesPerDocument)
                || (hasMeshFace
                    && (!AddWithinLimit(
                            theBudget->meshVertices,
                            aVertexCount,
                            kMaximumMeshVerticesPerDocument)
                        || !AddWithinLimit(
                            theBudget->meshIndices,
                            anIndexCount,
                            kMaximumMeshIndicesPerDocument))))) {
            return DefinitionGeometryClass::Invalid;
        }
        if (hasMeshFace) {
            return DefinitionGeometryClass::TriangleMesh;
        }
        // Legacy definitions made solely from edges, wires, vertices, or an
        // empty compound remain BRep-compatible. Every existing face has
        // proved to own a geometric surface in the bounded walk above.
        return DefinitionGeometryClass::BRep;
    } catch (...) {
        return DefinitionGeometryClass::Invalid;
    }
}

bool GeometryClassMatchesRepresentation(
    const DefinitionGeometryClass theGeometryClass,
    const OcctGeometryRepresentation theRepresentation) noexcept
{
    switch (theRepresentation) {
        case OcctGeometryRepresentation::LegacyUnknown:
        case OcctGeometryRepresentation::BRep:
            return theGeometryClass == DefinitionGeometryClass::BRep;
        case OcctGeometryRepresentation::TriangleMesh:
            return theGeometryClass
                == DefinitionGeometryClass::TriangleMesh;
        case OcctGeometryRepresentation::Invalid:
            return false;
    }
    return false;
}

OcctGeometryRepresentation ValidatedGeometryRepresentation(
    const Handle(TDocStd_Document)& theDocument,
    const Handle(XCAFDoc_ShapeTool)& theShapeTool,
    const TDF_Label& theLabel,
    GeometryValidationBudget* theBudget = nullptr)
{
    if (!IsGeometryDefinitionLabel(
            theDocument, theShapeTool, theLabel)) {
        return OcctGeometryRepresentation::Invalid;
    }
    bool hasMarker = false;
    OcctGeometryRepresentation aRepresentation =
        OcctGeometryRepresentation::Invalid;
    if (!ReadGeometryRepresentation(
            theLabel, hasMarker, aRepresentation)) {
        return OcctGeometryRepresentation::Invalid;
    }
    (void)hasMarker;
    const DefinitionGeometryClass aGeometryClass =
        ClassifyDefinitionGeometry(
            XCAFDoc_ShapeTool::GetShape(theLabel), theBudget);
    return GeometryClassMatchesRepresentation(
               aGeometryClass, aRepresentation)
        ? aRepresentation
        : OcctGeometryRepresentation::Invalid;
}

//! Marks XCAF visualization material assignments authored by Shapeyard's PBR
//! editor. Imported XCAF styles remain untouched and legacy child-11/12 style
//! overrides retain their historical precedence until the user authors PBR.
const Standard_GUID& LocalPBRMaterialAttributeID()
{
    static const Standard_GUID anId("248A5203-4A22-4F2B-85C4-BE0BA89A5E4D");
    return anId;
}

//! Per-object recipe, independent of deduplicated image/material definitions.
//! Version1 fixes the pinned Mikk generator and owned canonical UV/normal input.
const Standard_GUID& NormalTextureRecipeAttributeID()
{
    static const Standard_GUID anId("98EAD304-EB49-4F0E-ABFC-AEF94C250161");
    return anId;
}

//! Records that Shapeyard promoted the default black emissive factor to white
//! solely to make the first authored emissive texture visible. This lives on
//! the shape label so it follows OCAF history and appearance-copy operations.
const Standard_GUID& AutoPromotedEmissiveFactorAttributeID()
{
    static const Standard_GUID anId("1DA4580F-1B19-46DA-ABD4-BBCE9FBADE44");
    return anId;
}

//! Marks immutable table entries created by Shapeyard. Imported material
//! libraries must never be garbage-collected by local authoring operations.
const Standard_GUID& OwnedPBRMaterialDefinitionAttributeID()
{
    static const Standard_GUID anId("750690D2-357B-4101-B0EC-70DF20A2EC97");
    return anId;
}

void RemoveUnreferencedOwnedMaterial(
    const Handle(XCAFDoc_VisMaterialTool)& theTool,
    const TDF_Label& theMaterialLabel)
{
    if (theTool.IsNull() || theMaterialLabel.IsNull()) {
        return;
    }
    Handle(TDataStd_Integer) anOwnedMarker;
    if (!theMaterialLabel.FindAttribute(
            OwnedPBRMaterialDefinitionAttributeID(), anOwnedMarker)
        || anOwnedMarker.IsNull() || anOwnedMarker->Get() != 1) {
        return;
    }
    Handle(TDataStd_TreeNode) aReferenceRoot;
    if (theMaterialLabel.FindAttribute(
            XCAFDoc::VisMaterialRefGUID(), aReferenceRoot)
        && !aReferenceRoot.IsNull() && aReferenceRoot->HasFirst()) {
        return;
    }
    theTool->RemoveMaterial(theMaterialLabel);
}

Standard_Boolean IsOwnedMaterialDefinition(
    const TDF_Label& theMaterialLabel)
{
    if (theMaterialLabel.IsNull()) {
        return Standard_False;
    }
    Handle(TDataStd_Integer) anOwnedMarker;
    return theMaterialLabel.FindAttribute(
            OwnedPBRMaterialDefinitionAttributeID(), anOwnedMarker)
        && !anOwnedMarker.IsNull() && anOwnedMarker->Get() == 1;
}

Handle(XCAFDoc_VisMaterial) CreatePersistedPBRMaterial(
    const XCAFDoc_VisMaterialPBR& thePBR,
    const Handle(XCAFDoc_VisMaterial)& thePreviousMaterial)
{
    Graphic3d_AlphaMode anAlphaMode =
        thePBR.BaseColor.Alpha() < 0.999f
            ? Graphic3d_AlphaMode_Blend
            : Graphic3d_AlphaMode_Opaque;
    Standard_ShortReal anAlphaCutoff = 0.5f;
    Graphic3d_TypeOfBackfacingModel aFaceCulling =
        Graphic3d_TypeOfBackfacingModel_Auto;
    if (!thePreviousMaterial.IsNull()) {
        anAlphaMode = thePreviousMaterial->AlphaMode();
        anAlphaCutoff = thePreviousMaterial->AlphaCutOff();
        aFaceCulling = thePreviousMaterial->FaceCulling();
    }

    Handle(XCAFDoc_VisMaterial) aMaterial =
        new XCAFDoc_VisMaterial();
    aMaterial->SetPbrMaterial(thePBR);
    // The ES2/OpenGL Common fallback and authoritative PBR definition are
    // intentionally serialized as two slots. The safe binary reader charges
    // each occurrence, even when both reference identical bytes.
    XCAFDoc_VisMaterialCommon aCommon =
        aMaterial->ConvertToCommonMaterial();
    aCommon.DiffuseTexture = thePBR.BaseColorTexture;
    aMaterial->SetCommonMaterial(aCommon);
    aMaterial->SetAlphaMode(anAlphaMode, anAlphaCutoff);
    aMaterial->SetFaceCulling(aFaceCulling);
    return aMaterial;
}

bool AccumulateSerializedMaterialTextureOccurrences(
    const Handle(XCAFDoc_VisMaterial)& theMaterial,
    Core3DEmbeddedTextureBudgetState& theBudget,
    const Standard_Size theMaximumSerializedBytes,
    const Standard_Size theMaximumDecodedBytes)
{
    if (theMaterial.IsNull()) {
        return false;
    }
    if (theMaterial->HasPbrMaterial()) {
        const XCAFDoc_VisMaterialPBR& aPBR =
            theMaterial->PbrMaterial();
        if (!Core3DAccumulateEmbeddedTextureBudget(
                aPBR.BaseColorTexture,
                theBudget, theMaximumSerializedBytes,
                theMaximumDecodedBytes)
            || !Core3DAccumulateEmbeddedTextureBudget(
                aPBR.MetallicRoughnessTexture,
                theBudget, theMaximumSerializedBytes,
                theMaximumDecodedBytes)
            || !Core3DAccumulateEmbeddedTextureBudget(
                aPBR.EmissiveTexture,
                theBudget, theMaximumSerializedBytes,
                theMaximumDecodedBytes)
            || !Core3DAccumulateEmbeddedTextureBudget(
                aPBR.OcclusionTexture,
                theBudget, theMaximumSerializedBytes,
                theMaximumDecodedBytes)
            || !Core3DAccumulateEmbeddedTextureBudget(
                aPBR.NormalTexture,
                theBudget, theMaximumSerializedBytes,
                theMaximumDecodedBytes)) {
            return false;
        }
    }
    if (theMaterial->HasCommonMaterial()
        && !Core3DAccumulateEmbeddedTextureBudget(
            theMaterial->CommonMaterial().DiffuseTexture,
            theBudget, theMaximumSerializedBytes,
            theMaximumDecodedBytes)) {
        return false;
    }
    return true;
}

std::string ReadIdentifier(const TDF_Label& theLabel,
                           const Standard_GUID& theAttributeID)
{
    if (theLabel.IsNull()) {
        return {};
    }

    Handle(TDataStd_AsciiString) anIdentifier;
    if (!theLabel.FindAttribute(theAttributeID, anIdentifier)
        || anIdentifier.IsNull()) {
        return {};
    }

    const TCollection_AsciiString& aValue = anIdentifier->Get();
    if (aValue.IsEmpty() || !Standard_GUID::CheckGUIDFormat(aValue.ToCString())) {
        return {};
    }
    return aValue.ToCString();
}

std::string NewIdentifier()
{
    NSString* aValue = NSUUID.UUID.UUIDString;
    return aValue == nil ? std::string() : std::string(aValue.UTF8String);
}

// Persistent saved-group schema v1. Each string uses an existing bounded
// BinXCAF driver; never serialize the catalog into one large ASCII payload.
const Standard_GUID& SavedGroupContainerID() {
    static const Standard_GUID id("EC7B5F15-218F-47E4-BF6A-61BF42861401"); return id;
}
const Standard_GUID& SavedGroupRecordID() {
    static const Standard_GUID id("EC7B5F15-218F-47E4-BF6A-61BF42861402"); return id;
}
const Standard_GUID& SavedGroupNameID() {
    static const Standard_GUID id("EC7B5F15-218F-47E4-BF6A-61BF42861403"); return id;
}
const Standard_GUID& SavedGroupMembershipID() {
    static const Standard_GUID id("EC7B5F15-218F-47E4-BF6A-61BF42861404"); return id;
}
const Standard_GUID& SavedGroupOriginXID() {
    static const Standard_GUID id("EC7B5F15-218F-47E4-BF6A-61BF42861405"); return id;
}
const Standard_GUID& SavedGroupOriginYID() {
    static const Standard_GUID id("EC7B5F15-218F-47E4-BF6A-61BF42861406"); return id;
}
const Standard_GUID& SavedGroupOriginZID() {
    static const Standard_GUID id("EC7B5F15-218F-47E4-BF6A-61BF42861407"); return id;
}

// Definition-owned mesh-region partition schema v1. Payload chunks use the
// existing bounded TDataStd_AsciiString binary driver; no custom driver exists.
const Standard_GUID& MeshRegionPartitionRecordID() {
    static const Standard_GUID id("9D8D6BA0-A51C-4D0F-87A9-30A1C2D90101"); return id;
}
const Standard_GUID& MeshRegionPartitionVersionID() {
    static const Standard_GUID id("9D8D6BA0-A51C-4D0F-87A9-30A1C2D90102"); return id;
}
const Standard_GUID& MeshRegionPartitionTriangleCountID() {
    static const Standard_GUID id("9D8D6BA0-A51C-4D0F-87A9-30A1C2D90103"); return id;
}
const Standard_GUID& MeshRegionPartitionChunkCountID() {
    static const Standard_GUID id("9D8D6BA0-A51C-4D0F-87A9-30A1C2D90104"); return id;
}
const Standard_GUID& MeshRegionPartitionDigestID() {
    static const Standard_GUID id("9D8D6BA0-A51C-4D0F-87A9-30A1C2D90105"); return id;
}
constexpr Standard_Size kMeshRegionPartitionChunkCharacters = 256;
constexpr Standard_Size kMaximumMeshRegionPartitionChunks = 13;
constexpr Standard_Size kMaximumMeshRegionPartitionDocumentBytes = 8U * 1024U * 1024U;

bool HasMeshRegionPartitionSchemaAttribute(const TDF_Label& label) {
    Handle(TDF_Attribute) value;
    return label.FindAttribute(MeshRegionPartitionRecordID(), value)
        || label.FindAttribute(MeshRegionPartitionVersionID(), value)
        || label.FindAttribute(MeshRegionPartitionTriangleCountID(), value)
        || label.FindAttribute(MeshRegionPartitionChunkCountID(), value)
        || label.FindAttribute(MeshRegionPartitionDigestID(), value);
}

bool HasLiveMeshRegionPartitionDescendant(const TDF_Label& label,
                                          Standard_Size& visited) {
    for (TDF_ChildIterator child(label, Standard_True); child.More(); child.Next()) {
        if (++visited > kMaximumGeometryDocumentLabels) return true;
        if (child.Value().HasAttribute()) return true;
    }
    return false;
}

enum class MeshRegionPartitionReadState { Absent, Valid, Malformed };

std::string LowerHex(const Standard_Byte* bytes, const Standard_Size size) {
    static constexpr char digits[] = "0123456789abcdef";
    std::string output;
    output.reserve(2 * size);
    for (Standard_Size index = 0; index < size; ++index) {
        output.push_back(digits[(bytes[index] >> 4) & 15]);
        output.push_back(digits[bytes[index] & 15]);
    }
    return output;
}

MeshRegionPartitionReadState ReadMeshRegionPartitionRecord(
    const Handle(TDocStd_Document)& document, const TDF_Label& definition,
    std::vector<Standard_Byte>& encoded, TDF_Label* recordLabel = nullptr) noexcept {
    encoded.clear();
    if (recordLabel) *recordLabel = TDF_Label();
    try {
        OCC_CATCH_SIGNALS
        if (document.IsNull() || document->GetData().IsNull() || definition.IsNull()
            || definition.Data() != document->GetData()
            || HasMeshRegionPartitionSchemaAttribute(definition))
            return MeshRegionPartitionReadState::Malformed;
        TDF_Label record;
        // A definition owns at most one direct record. Schema fragments below
        // any other descendant are malformed rather than silently absent.
        Standard_Size descendants = 0;
        for (TDF_ChildIterator child(definition, Standard_True); child.More(); child.Next()) {
            if (++descendants > kMaximumGeometryDocumentLabels)
                return MeshRegionPartitionReadState::Malformed;
            if (!HasMeshRegionPartitionSchemaAttribute(child.Value())) continue;
            Handle(TDF_Attribute) marker;
            if (!child.Value().Father().IsEqual(definition)
                || !child.Value().FindAttribute(MeshRegionPartitionRecordID(), marker)
                || Handle(TDataStd_UAttribute)::DownCast(marker).IsNull()
                || !record.IsNull()) return MeshRegionPartitionReadState::Malformed;
            record = child.Value();
        }
        if (record.IsNull()) return MeshRegionPartitionReadState::Absent;
        Handle(TDataStd_Integer) version, triangles, chunks;
        Handle(TDataStd_AsciiString) digest;
        if (!record.FindAttribute(MeshRegionPartitionVersionID(), version)
            || !record.FindAttribute(MeshRegionPartitionTriangleCountID(), triangles)
            || !record.FindAttribute(MeshRegionPartitionChunkCountID(), chunks)
            || !record.FindAttribute(MeshRegionPartitionDigestID(), digest)
            || version.IsNull() || triangles.IsNull() || chunks.IsNull() || digest.IsNull()
            || version->Get() != 1 || triangles->Get() <= 0 || triangles->Get() > 4096
            || chunks->Get() <= 0 || chunks->Get() > Standard_Integer(kMaximumMeshRegionPartitionChunks))
            return MeshRegionPartitionReadState::Malformed;
        Standard_Size recordAttributes = 0;
        for (TDF_AttributeIterator attribute(record); attribute.More(); attribute.Next()) {
            const auto& id = attribute.Value()->ID();
            if (id != MeshRegionPartitionRecordID() && id != MeshRegionPartitionVersionID()
                && id != MeshRegionPartitionTriangleCountID() && id != MeshRegionPartitionChunkCountID()
                && id != MeshRegionPartitionDigestID()) return MeshRegionPartitionReadState::Malformed;
            ++recordAttributes;
        }
        const std::string digestText = digest->Get().ToCString();
        if (recordAttributes != 5 || digestText.size() != 64) return MeshRegionPartitionReadState::Malformed;
        std::string hex;
        for (Standard_Integer index = 1; index <= chunks->Get(); ++index) {
            const TDF_Label chunk = record.FindChild(index, Standard_False);
            Handle(TDataStd_AsciiString) text;
            Standard_Size attributes = 0;
            Standard_Size nested = 0;
            if (chunk.IsNull() || HasLiveMeshRegionPartitionDescendant(chunk, nested)
                || !chunk.FindAttribute(TDataStd_AsciiString::GetID(), text) || text.IsNull())
                return MeshRegionPartitionReadState::Malformed;
            for (TDF_AttributeIterator attribute(chunk); attribute.More(); attribute.Next()) {
                if (attribute.Value()->ID() != TDataStd_AsciiString::GetID())
                    return MeshRegionPartitionReadState::Malformed;
                ++attributes;
            }
            const std::string value = text->Get().ToCString();
            if (attributes != 1 || value.empty() || value.size() > kMeshRegionPartitionChunkCharacters
                || (index < chunks->Get() && value.size() != kMeshRegionPartitionChunkCharacters))
                return MeshRegionPartitionReadState::Malformed;
            for (char c : value) if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')))
                return MeshRegionPartitionReadState::Malformed;
            hex += value;
        }
        Standard_Size directChildren = 0;
        for (TDF_ChildIterator child(record, Standard_False); child.More(); child.Next()) {
            if (++directChildren > kMaximumGeometryDocumentLabels)
                return MeshRegionPartitionReadState::Malformed;
            Standard_Size nested = 0;
            if (child.Value().Tag() > chunks->Get()
                && (child.Value().HasAttribute()
                    || HasLiveMeshRegionPartitionDescendant(child.Value(), nested)))
                return MeshRegionPartitionReadState::Malformed;
        }
        if (hex.size() & 1) return MeshRegionPartitionReadState::Malformed;
        encoded.reserve(hex.size() / 2);
        const auto nibble = [](char c) { return c <= '9' ? c - '0' : c - 'a' + 10; };
        for (Standard_Size index = 0; index < hex.size(); index += 2)
            encoded.push_back(Standard_Byte((nibble(hex[index]) << 4) | nibble(hex[index + 1])));
        core3d::meshedit::RegionPartition partition;
        if (!core3d::meshedit::DecodeRegionPartition(encoded.data(), encoded.size(), partition)
            || partition.barriers.size() != Standard_Size(triangles->Get())
            || LowerHex(encoded.data() + 12, 32) != digestText) {
            encoded.clear(); return MeshRegionPartitionReadState::Malformed;
        }
        std::atomic_bool cancelled{false}; core3d::meshedit::NativeTopologyCapture source;
        if (core3d::meshedit::CaptureNativeTopology(XCAFDoc_ShapeTool::GetShape(definition), source, cancelled)
                != core3d::meshedit::TopologyResult::Ready
            || !core3d::meshedit::ValidateRegionPartition(source, partition)) {
            encoded.clear(); return MeshRegionPartitionReadState::Malformed;
        }
        if (recordLabel) *recordLabel = record;
        return MeshRegionPartitionReadState::Valid;
    } catch (...) { encoded.clear(); if (recordLabel) *recordLabel = TDF_Label(); return MeshRegionPartitionReadState::Malformed; }
}
constexpr std::size_t kMaximumSavedGroups = 128;
constexpr std::size_t kMaximumSavedGroupMembers = 32;
bool IsCanonicalSavedGroupID(const std::string& id) {
    if (id.size() != 36 || !Standard_GUID::CheckGUIDFormat(id.c_str())) { return false; }
    for (char c : id) {
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || c == '-')) { return false; }
    }
    return true;
}
bool HasSavedGroupAttribute(const TDF_Label& label) {
    Handle(TDF_Attribute) attribute;
    return label.FindAttribute(SavedGroupContainerID(), attribute)
        || label.FindAttribute(SavedGroupRecordID(), attribute)
        || label.FindAttribute(SavedGroupNameID(), attribute)
        || label.FindAttribute(SavedGroupMembershipID(), attribute)
        || label.FindAttribute(SavedGroupOriginXID(), attribute)
        || label.FindAttribute(SavedGroupOriginYID(), attribute)
        || label.FindAttribute(SavedGroupOriginZID(), attribute);
}
bool ReadSavedGroups(const Handle(TDocStd_Document)& document, OcctSavedGroupState& output) noexcept {
    output = OcctSavedGroupState();
    try {
        if (document.IsNull() || document->GetData().IsNull()) { return false; }
        OcctSavedGroupState state;
        state.documentData = document->GetData();
        const TDF_Label root = state.documentData->Root();
        if (HasSavedGroupAttribute(root)) { return false; }
        std::vector<TDF_Label> records, members;
        std::size_t count = 0;
        // First collect every schema-bearing label, including wrongly placed
        // attributes. Downcasting an unexpected type must fail admission.
        for (TDF_ChildIterator it(root, Standard_True); it.More(); it.Next()) {
            if (++count > kMaximumGeometryDocumentLabels) { return false; }
            const TDF_Label label = it.Value();
            Handle(TDF_Attribute) marker, record, name, member, originX, originY, originZ;
            const bool hasMarker = label.FindAttribute(SavedGroupContainerID(), marker);
            const bool hasRecord = label.FindAttribute(SavedGroupRecordID(), record);
            const bool hasName = label.FindAttribute(SavedGroupNameID(), name);
            const bool hasMember = label.FindAttribute(SavedGroupMembershipID(), member);
            const bool hasOriginX = label.FindAttribute(SavedGroupOriginXID(), originX);
            const bool hasOriginY = label.FindAttribute(SavedGroupOriginYID(), originY);
            const bool hasOriginZ = label.FindAttribute(SavedGroupOriginZID(), originZ);
            const bool hasAnyOrigin = hasOriginX || hasOriginY || hasOriginZ;
            if (hasMarker) {
                const auto typed = Handle(TDataStd_Integer)::DownCast(marker);
                if (typed.IsNull() || typed->Get() != 1 || !label.Father().IsEqual(root)
                    || label.IsEqual(document->Main()) || !state.container.IsNull()
                    || hasRecord || hasName || hasMember || hasAnyOrigin) { return false; }
                state.container = label;
            }
            if (hasRecord || hasName || hasAnyOrigin) {
                if (!hasRecord || !hasName || hasMember
                    || Handle(TDataStd_AsciiString)::DownCast(record).IsNull()
                    || Handle(TDataStd_Name)::DownCast(name).IsNull()
                    || records.size() >= kMaximumSavedGroups) { return false; }
                records.push_back(label);
            }
            if ((hasOriginX && Handle(TDataStd_Real)::DownCast(originX).IsNull())
                || (hasOriginY && Handle(TDataStd_Real)::DownCast(originY).IsNull())
                || (hasOriginZ && Handle(TDataStd_Real)::DownCast(originZ).IsNull())) { return false; }
            if (hasMember) {
                if (hasAnyOrigin || Handle(TDataStd_AsciiString)::DownCast(member).IsNull()
                    || members.size() >= kMaximumSavedGroups * kMaximumSavedGroupMembers) { return false; }
                members.push_back(label);
            }
        }
        if (state.container.IsNull() && (!records.empty() || !members.empty())) { return false; }
        std::unordered_map<std::string, std::size_t> indices;
        for (const auto& label : records) {
            if (!label.Father().IsEqual(state.container)) { return false; }
            OcctSavedGroup group;
            group.recordLabel = label;
            group.identifier = ReadIdentifier(label, SavedGroupRecordID());
            Handle(TDataStd_Name) name;
            if (!IsCanonicalSavedGroupID(group.identifier)
                || !label.FindAttribute(SavedGroupNameID(), name)
                || !OcctObjectNameIsValid(name->Get())
                || !indices.emplace(group.identifier, state.groups.size()).second) { return false; }
            group.name = name->Get();
            Handle(TDataStd_Real) originX, originY, originZ;
            const bool hasOriginX = label.FindAttribute(SavedGroupOriginXID(), originX);
            const bool hasOriginY = label.FindAttribute(SavedGroupOriginYID(), originY);
            const bool hasOriginZ = label.FindAttribute(SavedGroupOriginZID(), originZ);
            if (hasOriginX || hasOriginY || hasOriginZ) {
                if (!hasOriginX || !hasOriginY || !hasOriginZ
                    || originX.IsNull() || originY.IsNull() || originZ.IsNull()) { return false; }
                group.origin=gp_Pnt(originX->Get(),originY->Get(),originZ->Get());
                if (!OcctDocument::IsAdmittedSavedGroupOrigin(group.origin)) { return false; }
                group.originPresent=Standard_True;
            }
            state.groups.push_back(std::move(group));
        }
        const auto shapes = XCAFDoc_DocumentTool::CheckShapeTool(document->Main())
            ? XCAFDoc_DocumentTool::ShapeTool(document->Main()) : Handle(XCAFDoc_ShapeTool)();
        for (const auto& label : members) {
            const auto id = ReadIdentifier(label, SavedGroupMembershipID());
            const auto found = indices.find(id);
            if (!IsCanonicalSavedGroupID(id) || found == indices.end() || shapes.IsNull()
                || !IsGeometryDefinitionLabel(document, shapes, label)
                || !XCAFDoc_ShapeTool::IsFree(label)
                || ReadIdentifier(label, EntityIdentifierAttributeID()).empty()
                || ReadIdentifier(label, DefinitionIdentifierAttributeID()).empty()) { return false; }
            auto& group = state.groups[found->second];
            if (group.members.size() >= kMaximumSavedGroupMembers) { return false; }
            group.members.push_back(label);
        }
        output = std::move(state);
        return true;
    } catch (...) { output = OcctSavedGroupState(); return false; }
}

Standard_Boolean AssignNewIdentifier(
    const TDF_Label& theLabel,
    const Standard_GUID& theAttributeID)
{
    if (theLabel.IsNull()) {
        return Standard_False;
    }

    const std::string anIdentifier = NewIdentifier();
    if (anIdentifier.empty()) {
        return Standard_False;
    }
    TDataStd_AsciiString::Set(
        theLabel,
        theAttributeID,
        TCollection_AsciiString(anIdentifier.c_str()));
    return Standard_True;
}

Standard_Boolean AssignIdentifierIfMissing(
    const TDF_Label& theLabel,
    const Standard_GUID& theAttributeID)
{
    return !ReadIdentifier(theLabel, theAttributeID).empty()
        || AssignNewIdentifier(theLabel, theAttributeID);
}

void AbortCommandNoThrow(const Handle(TDocStd_Document)& theDocument) noexcept
{
    if (theDocument.IsNull()) {
        return;
    }
    try {
        if (theDocument->HasOpenCommand()) {
            theDocument->AbortCommand();
        }
    } catch (...) {
    }
}

} // namespace

Standard_Boolean Core3DAccumulateEmbeddedTextureBudget(
    const Handle(Image_Texture)& texture,
    Core3DEmbeddedTextureBudgetState& state,
    const Standard_Size maximumSerializedOccurrenceBytes,
    const Standard_Size maximumDecodedResourceBytes)
{
    if (texture.IsNull()) {
        return Standard_True;
    }
    const std::string storedIdentifier(
        texture->TextureId().ToCString());
    std::string canonicalIdentifier = storedIdentifier;
    if (!texture->FilePath().IsEmpty()
        || storedIdentifier.empty()
        || storedIdentifier.size()
            > static_cast<std::size_t>(
                kMaximumPersistentTextureIdentifierBytes)
        || !StripBufferTexturePrefixes(canonicalIdentifier)) {
        return Standard_False;
    }
    const Handle(NCollection_Buffer)& buffer =
        texture->DataBuffer();
    if (buffer.IsNull() || buffer->Data() == nullptr
        || buffer->Size() == 0
        || buffer->Size() > kMaximumEmbeddedTextureBytes) {
        return Standard_False;
    }

    const Standard_Size serializedLimit = std::min(
        maximumSerializedOccurrenceBytes,
        kMaximumAggregateTextureBytes);
    if (state.serializedOccurrenceBytes > serializedLimit
        || buffer->Size()
            > serializedLimit - state.serializedOccurrenceBytes) {
        return Standard_False;
    }
    state.serializedOccurrenceBytes += buffer->Size();

    const auto existing =
        state.resourcesByIdentifier.find(canonicalIdentifier);
    if (existing != state.resourcesByIdentifier.end()) {
        const Handle(NCollection_Buffer)& existingBuffer =
            existing->second;
        return !existingBuffer.IsNull()
            && existingBuffer->Data() != nullptr
            && existingBuffer->Size() == buffer->Size()
            && std::memcmp(existingBuffer->Data(), buffer->Data(),
                           buffer->Size()) == 0;
    }

    Standard_Size decodedBytes = 0;
    const Standard_Size decodedLimit = std::min(
        maximumDecodedResourceBytes,
        kMaximumDecodedTextureBytes);
    if (!TryTextureDecodedBytes(buffer, decodedBytes)
        || state.decodedResourceBytes > decodedLimit
        || decodedBytes
            > decodedLimit - state.decodedResourceBytes) {
        return Standard_False;
    }
    state.decodedResourceBytes += decodedBytes;
    state.resourcesByIdentifier.emplace(
        canonicalIdentifier, buffer);
    return Standard_True;
}

void Core3DPrepareRendererTextures(
    const Handle(Graphic3d_AspectFillArea3d)& aspect)
{
    if (aspect.IsNull()) return;
    const bool ownsShader = IsCore3DDataMapShader(aspect->ShaderProgram());
    if (aspect->TextureSet().IsNull() || aspect->TextureSet()->IsEmpty()) {
        if (ownsShader) aspect->SetShaderProgram({});
        return;
    }
    const Handle(Graphic3d_TextureSet)& original = aspect->TextureSet();
    if (original->IsEmpty()) return;
    Handle(Graphic3d_TextureSet) replacement = new Graphic3d_TextureSet(original->Size());
    bool changed = false;
    for (Standard_Integer i = 0; i < original->Size(); ++i) {
        const Handle(Graphic3d_TextureMap)& current = original->Value(i);
        replacement->SetValue(i, current);
        const Handle(XCAFPrs_Texture) texture = Handle(XCAFPrs_Texture)::DownCast(current);
        if (texture.IsNull() || !Handle(Core3DRoleAwareTexture)::DownCast(texture).IsNull()
            || texture->GetParams().IsNull()) continue;
        Handle(Core3DRoleAwareTexture) prepared = new Core3DRoleAwareTexture(texture);
        replacement->SetValue(i, prepared);
        changed = true;
    }
    if (changed) aspect->SetTextureSet(replacement);
    Standard_Integer bits = 0;
    for (Standard_Integer i = 0; i < replacement->Size(); ++i) {
        const auto& texture = replacement->Value(i);
        if (texture.IsNull() || texture->GetParams().IsNull()) continue;
        const auto unit = texture->GetParams()->TextureUnit();
        if (unit >= Graphic3d_TextureUnit_BaseColor && unit <= Graphic3d_TextureUnit_MetallicRoughness) {
            bits |= (1 << static_cast<int>(unit));
        }
    }
    const bool hasDataMaps = (bits & (Graphic3d_TextureSetBits_MetallicRoughness | Graphic3d_TextureSetBits_Occlusion | Graphic3d_TextureSetBits_Normal)) != 0;
    if (hasDataMaps && (ownsShader || aspect->ShaderProgram().IsNull())) {
        const auto expectedID = TCollection_AsciiString((bits & Graphic3d_TextureSetBits_Normal)
            ? "shapeyard-data-maps-v1-normal-" : "shapeyard-data-maps-v1-") + bits;
        if (!ownsShader || aspect->ShaderProgram()->GetId() != expectedID) {
            aspect->SetShaderProgram(MakeCore3DDataMapShader(bits));
        }
    } else if (ownsShader) {
        aspect->SetShaderProgram({});
    }
}

Handle(Image_Texture)& Core3DMaterialTexture(XCAFDoc_VisMaterialPBR& material,
                                            OcctMaterialTextureSlot slot)
{
    switch (slot) {
        case OcctMaterialTextureSlot::BaseColor: return material.BaseColorTexture;
        case OcctMaterialTextureSlot::Emissive: return material.EmissiveTexture;
        case OcctMaterialTextureSlot::MetallicRoughness: return material.MetallicRoughnessTexture;
        case OcctMaterialTextureSlot::Occlusion: return material.OcclusionTexture;
        case OcctMaterialTextureSlot::Normal: return material.NormalTexture;
    }
    throw Standard_Failure("Invalid material texture slot");
}

Standard_Boolean Core3DValidateNumericTexture(const Handle(Image_Texture)& texture)
{
    if (texture.IsNull() || !Core3DValidateAuthoredTexture(texture)) return Standard_False;
    return !DecodeNumericRendererPNG(texture->DataBuffer()).IsNull();
}

Standard_Boolean Core3DCreateAuthoredTexture(
    const Standard_Byte* bytes,
    const Standard_Size size,
    const std::string& mediaType,
    Handle(Image_Texture)& texture)
{
    texture.Nullify();
    if (!ValidateAuthoredRasterBytes(bytes, size, &mediaType)) {
        return Standard_False;
    }
    const std::string identifier = AuthoredTextureIdentifier(bytes, size);
    if (identifier.size() != 15 + CC_SHA256_DIGEST_LENGTH * 2) {
        return Standard_False;
    }
    Handle(NCollection_Buffer) buffer = new NCollection_Buffer(
        NCollection_BaseAllocator::CommonBaseAllocator(), size);
    if (buffer.IsNull() || buffer->ChangeData() == nullptr
        || buffer->Size() != size) {
        return Standard_False;
    }
    std::memcpy(buffer->ChangeData(), bytes, size);
    texture = MakeRendererDecodableTexture(
        buffer, TCollection_AsciiString(identifier.c_str()));
    if (texture.IsNull()) {
        return Standard_False;
    }
#ifdef DEBUG
    // Keep the constructor's OCCT-added texturebuf:// spelling aligned with
    // the standalone validator used by persistence and scalar edits.
    if (!Core3DValidateAuthoredTexture(texture)) {
        texture.Nullify();
        return Standard_False;
    }
#endif
    return Standard_True;
}

Standard_Boolean Core3DValidateAuthoredTexture(
    const Handle(Image_Texture)& texture)
{
    if (texture.IsNull() || !texture->FilePath().IsEmpty()) {
        return Standard_False;
    }
    const Handle(NCollection_Buffer)& buffer = texture->DataBuffer();
    if (buffer.IsNull() || buffer->Data() == nullptr
        || !ValidateAuthoredRasterBytes(
            buffer->Data(), buffer->Size(), nullptr)) {
        return Standard_False;
    }
    const std::string identifier = AuthoredTextureIdentifier(
        buffer->Data(), buffer->Size());
    std::string storedIdentifier(texture->TextureId().ToCString());
    return !identifier.empty()
        && StripBufferTexturePrefixes(storedIdentifier)
        && storedIdentifier == identifier;
}

Standard_Boolean Core3DTexturesMatch(
    const Handle(Image_Texture)& first,
    const Handle(Image_Texture)& second)
{
    if (first.IsNull() || second.IsNull()
        || !first->FilePath().IsEmpty()
        || !second->FilePath().IsEmpty()
        || !first->TextureId().IsEqual(second->TextureId())) {
        return Standard_False;
    }
    const Handle(NCollection_Buffer)& firstBuffer = first->DataBuffer();
    const Handle(NCollection_Buffer)& secondBuffer = second->DataBuffer();
    return !firstBuffer.IsNull() && !secondBuffer.IsNull()
        && firstBuffer->Data() != nullptr && secondBuffer->Data() != nullptr
        && firstBuffer->Size() == secondBuffer->Size()
        && std::memcmp(firstBuffer->Data(), secondBuffer->Data(),
                       firstBuffer->Size()) == 0;
}

Standard_Boolean Core3DCreateAuthoredBaseColorTexture(
    const Standard_Byte* bytes,
    const Standard_Size size,
    const std::string& mediaType,
    Handle(Image_Texture)& texture)
{
    return Core3DCreateAuthoredTexture(
        bytes, size, mediaType, texture);
}

Standard_Boolean Core3DValidateAuthoredBaseColorTexture(
    const Handle(Image_Texture)& texture)
{
    return Core3DValidateAuthoredTexture(texture);
}

Standard_Boolean Core3DBaseColorTexturesMatch(
    const Handle(Image_Texture)& first,
    const Handle(Image_Texture)& second)
{
    return Core3DTexturesMatch(first, second);
}

void Core3DBeginSafeBinaryRead()
{
    gSafeBinaryReadRejected = false;
}

Standard_Boolean Core3DSafeBinaryReadWasRejected()
{
    return gSafeBinaryReadRejected ? Standard_True : Standard_False;
}

void Core3DDefineSafeBinXCAFFormat(
    const Handle(TDocStd_Application)& application)
{
    if (application.IsNull()) {
        return;
    }
    application->DefineFormat(
        TCollection_AsciiString("BinOcaf"),
        TCollection_AsciiString("Binary OCAF Document"),
        TCollection_AsciiString("cbf"),
        new Core3DBoundedBinXCAFRetrievalDriver(),
        new core3d::receipt::v3::StorageDriver<core3d::bounded_curve::StorageDriver<core3d::composite_recipe::StorageDriver<core3d::retained_solid::StorageDriver<BinDrivers_DocumentStorageDriver>>>>());
    application->DefineFormat(
        TCollection_AsciiString("BinXCAF"),
        TCollection_AsciiString("Binary XCAF Document"),
        TCollection_AsciiString("xbf"),
        new Core3DBoundedBinXCAFRetrievalDriver(),
        new core3d::receipt::v3::StorageDriver<core3d::bounded_curve::StorageDriver<core3d::composite_recipe::StorageDriver<core3d::retained_solid::StorageDriver<BinXCAFDrivers_DocumentStorageDriver>>>>());
}

#if DEBUG
void Core3DDebugDefineLegacyReceiptFormats(const Handle(TDocStd_Application)& application) {
    if(application.IsNull())return;
    using namespace core3d::persistence::receipt_framing;
    application->DefineFormat("BinOcaf","Private unchanged legacy reader","cbf",
        new Core3DBoundedBinXCAFRetrievalDriver(Handle(FrameDriver)(),TraversalLimits()),new BinDrivers_DocumentStorageDriver());
    application->DefineFormat("BinXCAF","Private unchanged legacy reader","xbf",
        new Core3DBoundedBinXCAFRetrievalDriver(Handle(FrameDriver)(),TraversalLimits()),new BinXCAFDrivers_DocumentStorageDriver());
}
std::map<std::string, bool> Core3DDebugReceiptFramingProbe(Standard_Integer scenario) {
    if (scenario < 0 || scenario > 3) return {{"invalidScenario",false}};
    try {
        using namespace core3d::persistence::receipt_framing;
        return core3d::debug::receipt_framing_probe::Run(scenario,
            [](const Handle(FrameDriver)& driver, TraversalLimits limits)->Handle(PCDM_RetrievalDriver) {
                if (driver.IsNull()) return new Core3DBoundedBinXCAFRetrievalDriver();
                return new Core3DBoundedBinXCAFRetrievalDriver(driver,limits);
            });
    } catch (...) { return {{"setupException",false}}; }
}
void Core3DDebugDefineFrameBinXCAFFormat(
    const Handle(TDocStd_Application)& application,
    const std::shared_ptr<core3d::persistence::AuthoredFrameReadBudget>& budget)
{
    if (application.IsNull() || !budget) return;
    application->DefineFormat(TCollection_AsciiString("BinXCAF"),
        TCollection_AsciiString("Private frame test document"), TCollection_AsciiString("xbf"),
        new Core3DBoundedBinXCAFRetrievalDriver(budget),
        new BinXCAFDrivers_DocumentStorageDriver());
}
#endif

// =======================================================================
// function : OcctViewer
// purpose  :
// =======================================================================
OcctDocument::OcctDocument()
: myMaximumSerializedTextureOccurrenceBytes(
      kMaximumAggregateTextureBytes),
  myMaximumDecodedTextureResourceBytes(
      kMaximumDecodedTextureBytes),
  myMaximumVisualMaterialDefinitions(
      kMaximumVisualMaterialDefinitions)
{
  try
  {
    OCC_CATCH_SIGNALS
#if DEBUG
    myObservedApplication = new core3d::debug::LiveObservedApplication();
    myAuthorityApplication = myObservedApplication;
#else
    myAuthorityApplication = new core3d::authority::NativeObservedApplication();
#endif
    myApp = myAuthorityApplication;
    std::array<std::uint8_t, 16> nonce{};
    [[NSUUID UUID] getUUIDBytes:nonce.data()];
    myNativeAuthority = std::make_shared<core3d::authority::NativeEditAuthority>(nonce);
    if (!myAuthorityApplication->ObserveAuthority(myNativeAuthority)) myNativeAuthority.reset();
  }
  catch (const Standard_Failure& theFailure)
  {
    Message::SendFail (TCollection_AsciiString("Error in creating application") + theFailure.GetMessageString());
  }
}

// =======================================================================
// function : NativeDocumentSession
// purpose  :
// =======================================================================
core3d::NativeDocumentSession::NativeDocumentSession()
{
    document_ = new OcctDocument();
    try {
        document_->InitDoc();
    } catch (...) {
        Close();
        throw;
    }
}

core3d::NativeDocumentSession::~NativeDocumentSession()
{
    Close();
}

void core3d::NativeDocumentSession::Close() noexcept
{
    if (document_.IsNull()) return;
    document_->CloseNativeSession();
    document_.Nullify();
}

void OcctDocument::CloseNativeSession() noexcept
{
    if (myNativeSessionClosed) return;
    myNativeSessionClosed = true;
    if (myRetainedFeatureOwner) myRetainedFeatureOwner->retireForDocumentReplacement();
    myRetainedFeatureOwner.reset();
    if (myPartBooleanOwner) myPartBooleanOwner->retireForDocumentReplacement();
    myPartBooleanOwner.reset();
    if (myNativeAuthority) myNativeAuthority->Detach();
#if DEBUG
    if (myLiveProbe) myLiveProbe->Detach(myOcafDoc);
#endif
    try {
        const Handle(TDocStd_Document) document = myOcafDoc;
        if (!document.IsNull()) {
            if (document->HasOpenCommand()) document->AbortCommand();
            const Handle(TDocStd_Application) application =
                Handle(TDocStd_Application)::DownCast(document->Application());
            if (!application.IsNull()) application->Close(document);
        }
    } catch (const Standard_Failure& failure) {
        NSLog(@"OCCT document teardown failed: %s", failure.GetMessageString());
    } catch (...) {
        NSLog(@"OCCT document teardown failed with an unknown error");
    }
    // Copied wrapper handles must observe terminal closure as well.
    myOcafDoc.Nullify();
}

OcctDocument::~OcctDocument()
{
    std::cout << "~OcctDocument" << std::endl;
}

// =======================================================================
// function : InitDoc
// purpose  :
// =======================================================================
void OcctDocument::InitDoc()
{
    if (myNativeSessionClosed) throw Standard_ProgramError("Native document session is closed");
    if (NativeBooleanOwnerBlocksOtherWork())
        throw Standard_ProgramError("Native Boolean owner blocks document replacement");
    
    std::cout << "InitDoc()" << std::endl;
  // Observers detach before owner retirement: any abort emitted while retiring
  // the outgoing document is replacement cleanup and must record inactive.
  if (myNativeAuthority) myNativeAuthority->Detach();
#if DEBUG
  if (myLiveProbe) myLiveProbe->Detach(myOcafDoc);
#endif
  if (myPartBooleanOwner) myPartBooleanOwner->retireForDocumentReplacement();
  myPartBooleanOwner.reset();
  if (myRetainedFeatureOwner) myRetainedFeatureOwner->retireForDocumentReplacement();
  myRetainedFeatureOwner.reset();
  // close old document
  if (!myOcafDoc.IsNull())
  {
    if (myOcafDoc->HasOpenCommand())
    {
      myOcafDoc->AbortCommand();
    }

    myOcafDoc->Main().Root().ForgetAllAttributes(Standard_True);
    myApp->Close(myOcafDoc);
    myOcafDoc.Nullify();
  }


  // Register both readers before creating the document: old projects remain
  // readable while new saves use the XCAF driver required by visual materials.
  Core3DDefineSafeBinXCAFFormat(myApp);
  myApp->NewDocument(TCollection_ExtendedString("BinXCAF"), myOcafDoc);

  // Install document infrastructure and identity before enabling normal undo
  // history. These are schema attributes, not user-authored edits.
  if (!myOcafDoc.IsNull())
  {
	// Create the persistent XCAF tools before the first undoable command. If the
	// first shape command creates these infrastructure attributes, Undo removes
	// them and a viewport redraw recreates them outside history; Redo then fails
	// because the same labels already carry those attributes.
	(void)XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
	(void)XCAFDoc_DocumentTool::ColorTool(myOcafDoc->Main());
	(void)XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
	XCAFDoc_DocumentTool::SetLengthUnit(
	    myOcafDoc, kDefaultMetersPerUnit);
	if (!AssignIdentifierIfMissing(
	        myOcafDoc->Main(), DocumentIdentifierAttributeID())) {
	  Message::SendFail("Unable to assign Core3D document identifier");
	}
	myOcafDoc->ClearUndos();
	myOcafDoc->SetUndoLimit(kNativeSessionUndoLimit);
    myPartBooleanOwner = std::make_unique<core3d::part_boolean::owner::PartBooleanOwner>(*this);
    myRetainedFeatureOwner = std::make_unique<core3d::retained_feature::OcafOwnerService>(*this);
  }
  ObserveSuccessfulNativeDocumentAdoption();
#if DEBUG
  DebugObserveSuccessfulDocumentAdoption();
#endif
}

void OcctDocument::ObserveSuccessfulNativeDocumentAdoption() noexcept {
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()
            || DocumentIdentifier().empty()) {
            myNativeAuthority->Detach(); return;
        }
        myNativeAuthority->Adopt(myOcafDoc.get());
        if (!myPartBooleanOwner || !myPartBooleanOwner->boundTo(myOcafDoc)) {
            if (myPartBooleanOwner) myPartBooleanOwner->retireForDocumentReplacement();
            myPartBooleanOwner = std::make_unique<core3d::part_boolean::owner::PartBooleanOwner>(*this);
        }
        if (!myRetainedFeatureOwner || !myRetainedFeatureOwner->boundTo(myOcafDoc)) {
            if (myRetainedFeatureOwner) myRetainedFeatureOwner->retireForDocumentReplacement();
            myRetainedFeatureOwner = std::make_unique<core3d::retained_feature::OcafOwnerService>(*this);
        }
    } catch (...) { myNativeAuthority->Detach(); }
}

core3d::part_boolean::owner::PartBooleanOwner*
OcctDocument::PartBooleanOwnerService() noexcept { return myPartBooleanOwner.get(); }
const core3d::part_boolean::owner::PartBooleanOwner*
OcctDocument::PartBooleanOwnerService() const noexcept { return myPartBooleanOwner.get(); }
bool OcctDocument::NativeBooleanOwnerBlocksOtherWork() const noexcept {
    return (myPartBooleanOwner && myPartBooleanOwner->blocksOtherWork())
        || (myRetainedFeatureOwner && myRetainedFeatureOwner->blocksOtherWork());
}
core3d::retained_feature::OcafOwnerService*
OcctDocument::RetainedFeatureOwnerService() noexcept { return myRetainedFeatureOwner.get(); }
const core3d::retained_feature::OcafOwnerService*
OcctDocument::RetainedFeatureOwnerService() const noexcept { return myRetainedFeatureOwner.get(); }

std::optional<core3d::authority::QueuedLoadReservation>
OcctDocument::BeginNativeQueuedLoad() noexcept {
    if (NativeBooleanOwnerBlocksOtherWork() || !myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return std::nullopt;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()) return std::nullopt;
        // Existing typed preview/recovery ledgers are rejected by the viewer.
        // Profile/alignment private workers are settled by the controller before adoption.
        return myNativeAuthority->BeginQueuedLoad(myOcafDoc.get(),
            core3d::authority::QueuedLoadAdmission::Ready, true, myOcafDoc->HasOpenCommand());
    } catch (...) { return std::nullopt; }
}

bool OcctDocument::OwnsNativeQueuedLoad(
    const core3d::authority::QueuedLoadReservation& reservation) noexcept {
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return false;
    try {
        return !myOcafDoc.IsNull() && myOcafDoc->Application().get() == myApp.get()
            && myNativeAuthority->OwnsQueuedLoad(reservation);
    } catch (...) { return false; }
}

core3d::authority::QueuedLoadEnd OcctDocument::EndNativeQueuedLoadPrivateWork(
    const core3d::authority::QueuedLoadReservation& reservation, bool privateWorkSettled) noexcept {
    using core3d::authority::QueuedLoadEnd;
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return QueuedLoadEnd::Unavailable;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()) return QueuedLoadEnd::Unavailable;
        return myNativeAuthority->EndQueuedLoadPrivateWork(reservation, privateWorkSettled);
    } catch (...) { return QueuedLoadEnd::Unavailable; }
}

std::optional<core3d::authority::ReplacementReservation> OcctDocument::PromoteNativeQueuedLoad(
    const core3d::authority::QueuedLoadReservation& reservation, bool nativeEditReady) noexcept {
    if (NativeBooleanOwnerBlocksOtherWork() || !myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return std::nullopt;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()) return std::nullopt;
        return myNativeAuthority->PromoteQueuedLoadToReplacement(reservation, myOcafDoc.get(),
            nativeEditReady, myOcafDoc->HasOpenCommand());
    } catch (...) { return std::nullopt; }
}

std::optional<core3d::authority::ReplacementReservation> OcctDocument::BeginNativeReplacement() noexcept {
    if (NativeBooleanOwnerBlocksOtherWork() || !myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return std::nullopt;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()) return std::nullopt;
        return myNativeAuthority->BeginReplacement(myOcafDoc.get(), true, myOcafDoc->HasOpenCommand());
    } catch (...) { return std::nullopt; }
}

core3d::authority::ReplacementEnd OcctDocument::EndNativeReplacement(
    const core3d::authority::ReplacementReservation& reservation,
    bool accepted, bool restored) noexcept {
    using core3d::authority::ReplacementEnd;
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return ReplacementEnd::Unavailable;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()
            || DocumentIdentifier().empty()) return ReplacementEnd::Unavailable;
        return myNativeAuthority->EndReplacement(reservation, myOcafDoc.get(), accepted,
            restored, myOcafDoc->HasOpenCommand());
    } catch (...) { return ReplacementEnd::Unavailable; }
}

std::optional<core3d::authority::Stamp>
OcctDocument::CaptureNativePlanningStamp(bool nativeEditReady) noexcept {
    if (NativeBooleanOwnerBlocksOtherWork() || ![NSThread isMainThread] || !nativeEditReady || !myNativeAuthority
        || !myAuthorityApplication || !myAuthorityApplication->AuthorityThreadContractValid())
        return std::nullopt;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()
            || DocumentIdentifier().empty()) return std::nullopt;
        return myNativeAuthority->Capture(myOcafDoc.get(), nativeEditReady, myOcafDoc->HasOpenCommand());
    } catch (...) { return std::nullopt; }
}

void OcctDocument::ObserveNativePlanningInteraction() noexcept {
    if (![NSThread isMainThread] || !myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return;
    try {
        if (!myOcafDoc.IsNull()) myNativeAuthority->SelectionBoundary(myOcafDoc.get());
    } catch (...) {}
}

#if DEBUG
std::optional<core3d::authority::Stamp> OcctDocument::DebugNativeMutationStamp() noexcept {
    // Diagnostic read only: selection and owner/preview fences are not wired.
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid() || myOcafDoc.IsNull())
        return std::nullopt;
    return myNativeAuthority->Capture(myOcafDoc.get(), true, myOcafDoc->HasOpenCommand());
}
bool OcctDocument::DebugStartLiveTransactionProbe() noexcept {
    if (![NSThread isMainThread] || myLiveProbe || myObservedApplication == nullptr
        || myApp.get() != myObservedApplication
        || !myObservedApplication->ThreadContractValid()) return false;
    try {
        if (NativeBooleanOwnerBlocksOtherWork() || myOcafDoc.IsNull() || myOcafDoc->HasOpenCommand()
            || myOcafDoc->Application().get() != myApp.get()
            || DocumentIdentifier().empty()) return false;
        auto state = std::make_shared<core3d::debug::LiveTransactionProbeState>();
        state->Adopt(myOcafDoc, true);
        if (!state->IsValid() || !myObservedApplication->Observe(state)) return false;
        myLiveProbe = std::move(state);
        return true;
    } catch (...) { return false; }
}
void OcctDocument::DebugStopLiveTransactionProbe() noexcept {
    if (![NSThread isMainThread] || !myLiveProbe || myObservedApplication == nullptr
        || !myLiveProbe->OnOwningThread()) return;
    // Invalid/overflowed observation must still be detachable on its owner.
    myLiveProbe->Detach(myOcafDoc);
    myObservedApplication->Observe({});
    myLiveProbe.reset();
}
std::shared_ptr<const core3d::debug::LiveTransactionProbeState>
OcctDocument::DebugLiveTransactionProbe() const noexcept {
    return [NSThread isMainThread] ? myLiveProbe : nullptr;
}
bool OcctDocument::DebugLiveTransactionProbeValid() const noexcept {
    return [NSThread isMainThread] && myLiveProbe && myLiveProbe->IsValid()
        && myObservedApplication != nullptr && myObservedApplication->ThreadContractValid();
}
void OcctDocument::DebugObserveSuccessfulDocumentAdoption() noexcept {
    if (!myLiveProbe || !myLiveProbe->OnOwningThread()) return;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()
            || DocumentIdentifier().empty()) { myLiveProbe->valid = false; return; }
        myLiveProbe->Adopt(myOcafDoc);
    } catch (...) { myLiveProbe->valid = false; }
}
#endif

std::string OcctDocument::DocumentIdentifier() const
{
    return myOcafDoc.IsNull()
        ? std::string()
        : ReadIdentifier(myOcafDoc->Main(), DocumentIdentifierAttributeID());
}

std::string OcctDocument::EntityIdentifierForLabel(const TDF_Label& label) const
{
    return ReadIdentifier(label, EntityIdentifierAttributeID());
}

std::string OcctDocument::DefinitionIdentifierForLabel(const TDF_Label& label) const
{
    return ReadIdentifier(label, DefinitionIdentifierAttributeID());
}

OcctGeometryRepresentation OcctDocument::GeometryRepresentationForLabel(
    const TDF_Label& label) const
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return OcctGeometryRepresentation::Invalid;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        return ValidatedGeometryRepresentation(
            myOcafDoc, aShapeTool, label);
    } catch (...) {
        return OcctGeometryRepresentation::Invalid;
    }
}

OcctGeometryRepresentation
OcctDocument::StoredGeometryRepresentationForLabel(
    const TDF_Label& label) const
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return OcctGeometryRepresentation::Invalid;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (!IsGeometryDefinitionLabel(
                myOcafDoc, aShapeTool, label)) {
            return OcctGeometryRepresentation::Invalid;
        }
        bool hasMarker = false;
        OcctGeometryRepresentation aRepresentation =
            OcctGeometryRepresentation::Invalid;
        if (!ReadGeometryRepresentation(
                label, hasMarker, aRepresentation)) {
            return OcctGeometryRepresentation::Invalid;
        }
        (void)hasMarker;
        return aRepresentation;
    } catch (...) {
        return OcctGeometryRepresentation::Invalid;
    }
}

OcctReferenceAxisReadState OcctDocument::ReadReferenceAxisForLabel(
    const TDF_Label& label,
    OcctReferenceAxis& axis) const
{
    axis = DefaultReferenceAxis();
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return OcctReferenceAxisReadState::Invalid;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (!IsGeometryDefinitionLabel(myOcafDoc, aShapeTool, label)) {
            return OcctReferenceAxisReadState::Invalid;
        }
        const OcctReferenceAxisReadState aState =
            ReadReferenceAxisRecord(label, axis);
        if (aState == OcctReferenceAxisReadState::Authored
            && !XCAFDoc_ShapeTool::IsFree(label)) {
            axis = DefaultReferenceAxis();
            return OcctReferenceAxisReadState::Invalid;
        }
        return aState;
    } catch (...) {
        axis = DefaultReferenceAxis();
        return OcctReferenceAxisReadState::Invalid;
    }
}

Standard_Boolean OcctDocument::ResolveReferenceAxisInWorld(
    const TDF_Label& label,
    const TopLoc_Location& occurrenceLocation,
    gp_Ax1& axis) const
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        OcctReferenceAxis aReference;
        const OcctReferenceAxisReadState aState =
            ReadReferenceAxisForLabel(label, aReference);
        if (aState == OcctReferenceAxisReadState::Invalid) {
            return Standard_False;
        }
        return TryResolveReferenceAxis(
            myOcafDoc, aShapeTool, label, occurrenceLocation, axis);
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::ValidateGeometryRepresentationForLabel(
    const TDF_Label& label) const
{
    return GeometryRepresentationForLabel(label)
        != OcctGeometryRepresentation::Invalid;
}

Standard_Boolean OcctDocument::ValidateGeometryRepresentations() const
{
    return ValidateGeometryRepresentations(myOcafDoc);
}

namespace {

struct GeometryDocumentUsage
{
    GeometryValidationBudget geometry;
    Standard_Size definitions = 0;
    Standard_Size labels = 0;
    Standard_Size graphVisits = 0;
    Standard_Size leafOccurrences = 0;
    Standard_Size featureRecords = 0;
};

Standard_Boolean ValidateGeometryDocument(
    const Handle(TDocStd_Document)& document,
    GeometryDocumentUsage* output)
{
    try {
        OCC_CATCH_SIGNALS
        if (document.IsNull() || document->GetData().IsNull()
            || !ValidateCommandOwnerSentinelsDocument(document)
            || !ValidateReferenceAxisDocument(document)
            || !([](const Handle(TDocStd_Document)& doc) {
                OcctSavedGroupState groups; return ReadSavedGroups(doc, groups);
            })(document)) {
            return Standard_False;
        }
        std::vector<core3d::profile::Record> profiles;
        std::vector<core3d::enclosure::Record> enclosures;
        std::vector<core3d::sweep_persistence::Record> sweeps;
        std::vector<core3d::loft_persistence::Record> lofts;
        std::vector<core3d::retained_solid::Record> retained;
        if (!core3d::saved_features::Validate(document, profiles, enclosures, sweeps, lofts,&retained)) return Standard_False;
        std::size_t retainedBytes=0;
        for(const auto& record:retained){
            if(!record.value||record.value->bytes.size()>core3d::composite_recipe::MaximumDocumentAggregateBytes-retainedBytes)
                return Standard_False;
            retainedBytes+=record.value->bytes.size();
        }
        std::vector<core3d::bounded_curve::Record> curves;
        if(!core3d::bounded_curve::ReadAll(document,curves))return Standard_False;
        std::size_t curveBytes=0;
        for(const auto& record:curves){
            if(!record.value
                ||record.value->definitionBytes.size()
                    >core3d::bounded_curve::MaximumDocumentAggregateBytes-curveBytes)return Standard_False;
            curveBytes+=record.value->definitionBytes.size();
        }
        std::vector<core3d::composite_recipe::Record> composites;
        if(!core3d::composite_recipe::ReadAll(document,composites,retainedBytes,
            curveBytes,curves.size()))return Standard_False;
        TDF_LabelMap retainedOwners;
        for(const auto& record:retained)if(!retainedOwners.Add(record.owner))return Standard_False;
        for(const auto& record:composites)if(retainedOwners.Contains(record.owner))return Standard_False;
        const TDF_Label aRoot = document->GetData()->Root();
        Handle(TDataStd_Integer) aRootMarker;
        Handle(TNaming_NamedShape) aRootShape;
        if (aRoot.FindAttribute(
                GeometryRepresentationAttributeID(), aRootMarker)
            || aRoot.FindAttribute(
                TNaming_NamedShape::GetID(), aRootShape)) {
            return Standard_False;
        }

        if (!XCAFDoc_DocumentTool::CheckShapeTool(document->Main())) {
            Standard_Size aLabelCount = 0;
            for (TDF_ChildIterator aLabel(aRoot, Standard_True);
                 aLabel.More(); aLabel.Next()) {
                if (++aLabelCount > kMaximumGeometryDocumentLabels) {
                    return Standard_False;
                }
                Handle(TDataStd_Integer) aMarker;
                Handle(TNaming_NamedShape) aNamedShape;
                if (aLabel.Value().FindAttribute(
                        GeometryRepresentationAttributeID(), aMarker)
                    || aLabel.Value().FindAttribute(
                        TNaming_NamedShape::GetID(), aNamedShape)
                    || HasMeshRegionPartitionSchemaAttribute(aLabel.Value())) {
                    return Standard_False;
                }
            }
            if (output != nullptr) {
                GeometryDocumentUsage usage;
                usage.labels = aLabelCount;
                *output = usage;
            }
            return Standard_True;
        }

        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (aShapeTool.IsNull()) {
            return Standard_False;
        }
        TDF_LabelSequence anAllTopLevelLabels;
        aShapeTool->GetShapes(anAllTopLevelLabels);
        if (static_cast<Standard_Size>(
                anAllTopLevelLabels.Length())
            > kMaximumGeometryDocumentLabels) {
            return Standard_False;
        }
        TDF_LabelMap anAllTopLevelLabelSet;
        for (Standard_Integer anIndex = 1;
             anIndex <= anAllTopLevelLabels.Length(); ++anIndex) {
            const TDF_Label& aLabel =
                anAllTopLevelLabels.Value(anIndex);
            if (aLabel.IsNull()
                || aLabel.Data() != document->GetData()
                || !aShapeTool->IsShape(aLabel)
                || !aShapeTool->IsTopLevel(aLabel)
                || XCAFDoc_ShapeTool::IsComponent(aLabel)
                || XCAFDoc_ShapeTool::IsSubShape(aLabel)
                || !anAllTopLevelLabelSet.Add(aLabel)) {
                return Standard_False;
            }
        }
        TDF_LabelSequence aFreeRootLabels;
        aShapeTool->GetFreeShapes(aFreeRootLabels);
        if (static_cast<Standard_Size>(aFreeRootLabels.Length())
            > kMaximumGeometryDocumentLabels) {
            return Standard_False;
        }

        struct AssemblyFrame {
            TDF_Label label;
            Standard_Size depth = 0;
            bool leaving = false;
        };
        TDF_LabelMap aVisitedGraphLabels;
        TDF_LabelMap aDefinitionLabels;
        GeometryValidationBudget aBudget;
        // Current bindings share the already-budgeted root geometry. Stale
        // definitions can retain older solids and must account for them too.
        for (const auto& profile : profiles) {
            if (!profile.boundShape.IsEqual(XCAFDoc_ShapeTool::GetShape(profile.label.Father()))
                && ClassifyDefinitionGeometry(profile.boundShape, &aBudget) != DefinitionGeometryClass::BRep)
                return Standard_False;
        }

        for (const auto& enclosure : enclosures) {
            if (!enclosure.boundShape.IsEqual(XCAFDoc_ShapeTool::GetShape(enclosure.label.Father()))
                && ClassifyDefinitionGeometry(enclosure.boundShape, &aBudget) != DefinitionGeometryClass::BRep)
                return Standard_False;
        }

        // Charge EACH retained reference just as each current definition is
        // charged. Shared codec identity does not waive an occurrence's budget.
        for(const auto& record:retained){
            const auto& base=record.value->base;
            if(ClassifyDefinitionGeometry(base,&aBudget)!=DefinitionGeometryClass::BRep
                ||!BRepCheck_Analyzer(base,Standard_True).IsValid())return Standard_False;
            unsigned shells=0;for(TopExp_Explorer shell(base,TopAbs_SHELL);shell.More();shell.Next()){
                ++shells;if(!BRep_Tool::IsClosed(shell.Current()))return Standard_False;
            }
            if(!shells)return Standard_False;
            BRepClass3d_SolidClassifier classifier(base);classifier.PerformInfinitePoint(Precision::Confusion());
            GProp_GProps volume;BRepGProp::VolumeProperties(base,volume,Standard_True,Standard_False,Standard_False);
            if(classifier.State()!=TopAbs_OUT||!std::isfinite(volume.Mass())||volume.Mass()<=0)return Standard_False;
        }

        // Charge every retained composite source independently. The visible
        // carrier is charged by the ordinary definition traversal below.
        const auto validCompositeSolid = [&](const TopoDS_Shape& source) {
            if(ClassifyDefinitionGeometry(source,&aBudget)!=DefinitionGeometryClass::BRep
                ||!BRepCheck_Analyzer(source,Standard_True).IsValid())return false;
            unsigned shells=0;for(TopExp_Explorer shell(source,TopAbs_SHELL);shell.More();shell.Next()){
                ++shells;if(!BRep_Tool::IsClosed(shell.Current()))return false;
            }
            if(!shells)return false;
            BRepClass3d_SolidClassifier classifier(source);classifier.PerformInfinitePoint(Precision::Confusion());
            GProp_GProps volume;BRepGProp::VolumeProperties(source,volume,Standard_True,Standard_False,Standard_False);
            return classifier.State()==TopAbs_OUT&&std::isfinite(volume.Mass())&&volume.Mass()>0;
        };
        for(const auto& record:composites){
            if(record.value->definition.schemaVersion<3){
                // Frozen v1/v2 domain: retain the exact historic closed-solid gate.
                for(const auto& source:record.value->sourceShapes)
                    if(!validCompositeSolid(source))return Standard_False;
                continue;
            }
            for(const auto& node:record.value->definition.nodes){
                const auto* sourceNode=std::get_if<core3d::composite_recipe::SourceNode>(&node.value);
                if(!sourceNode)continue;
                core3d::composite_recipe::SourceShapeKind expected=core3d::composite_recipe::SourceShapeKind::Unknown;
                if(sourceNode->shapeSlot>=record.value->sourceShapes.size()
                    ||!core3d::composite_recipe::ExpectedShapeKind(
                        record.value->definition,sourceNode->recipe,expected))return Standard_False;
                const TopoDS_Shape& source=record.value->sourceShapes[sourceNode->shapeSlot];
                // P1 installs only the legacy solid source descriptors. Future
                // face/wire descriptors must add their independent proof here;
                // they cannot inherit positive-volume solid admission.
                if(expected!=core3d::composite_recipe::SourceShapeKind::Solid
                    ||!validCompositeSolid(source))return Standard_False;
            }
        }

        Standard_Size aDefinitionCount = 0;
        Standard_Size anAggregateGraphVisitCount = 0;
        Standard_Size aLeafOccurrenceCount = 0;
        for (Standard_Integer aRootIndex = 1;
             aRootIndex <= aFreeRootLabels.Length(); ++aRootIndex) {
            const TDF_Label& aRootLabel =
                aFreeRootLabels.Value(aRootIndex);
            if (aRootLabel.IsNull()
                || aRootLabel.Data() != document->GetData()
                || !aShapeTool->IsShape(aRootLabel)
                || !aShapeTool->IsTopLevel(aRootLabel)
                || !anAllTopLevelLabelSet.Contains(aRootLabel)
                || XCAFDoc_ShapeTool::IsComponent(aRootLabel)
                || XCAFDoc_ShapeTool::IsSubShape(aRootLabel)) {
                return Standard_False;
            }
            // Depth is path-relative. Revisit shared DAG nodes through every
            // occurrence branch so a shallow first path cannot memoize away
            // an over-depth later path. Definition geometry is still
            // classified once through aDefinitionLabels below.
            TDF_LabelMap anActivePathForRoot;
            std::vector<AssemblyFrame> aStack = {{
                aRootLabel, 0, false}};
            while (!aStack.empty()) {
                const AssemblyFrame aFrame = aStack.back();
                aStack.pop_back();
                const TDF_Label& aLabel = aFrame.label;
                if (aLabel.IsNull()
                    || aLabel.Data() != document->GetData()) {
                    return Standard_False;
                }
                if (aFrame.leaving) {
                    anActivePathForRoot.Remove(aLabel);
                    aVisitedGraphLabels.Add(aLabel);
                    continue;
                }
                if (anActivePathForRoot.Contains(aLabel)
                    || aFrame.depth > kMaximumTopologyDepth
                    || anAggregateGraphVisitCount
                        >= kMaximumGeometryDocumentLabels
                    || !aShapeTool->IsShape(aLabel)) {
                    return Standard_False;
                }
                anActivePathForRoot.Add(aLabel);
                ++anAggregateGraphVisitCount;
                if (aStack.size()
                    >= static_cast<std::size_t>(
                        kMaximumGeometryDocumentLabels)) {
                    return Standard_False;
                }
                aStack.push_back({
                    aLabel, aFrame.depth, true});

                if (XCAFDoc_ShapeTool::IsReference(aLabel)
                    || XCAFDoc_ShapeTool::IsComponent(aLabel)) {
                    TDF_Label aReferredLabel;
                    if (!XCAFDoc_ShapeTool::GetReferredShape(
                            aLabel, aReferredLabel)
                        || aReferredLabel.IsNull()
                        || aReferredLabel.Data()
                            != document->GetData()
                        || aFrame.depth >= kMaximumTopologyDepth
                        || aStack.size()
                            >= static_cast<std::size_t>(
                                kMaximumGeometryDocumentLabels)) {
                        return Standard_False;
                    }
                    aStack.push_back({
                        aReferredLabel, aFrame.depth + 1U, false});
                    continue;
                }
                if (XCAFDoc_ShapeTool::IsAssembly(aLabel)) {
                    TDF_LabelSequence aComponents;
                    if (!XCAFDoc_ShapeTool::GetComponents(
                            aLabel, aComponents, Standard_False)
                        || aComponents.IsEmpty()
                        || static_cast<Standard_Size>(
                               aComponents.Length())
                            > kMaximumGeometryDocumentLabels
                        || aFrame.depth >= kMaximumTopologyDepth) {
                        return Standard_False;
                    }
                    for (Standard_Integer aComponentIndex = 1;
                         aComponentIndex <= aComponents.Length();
                         ++aComponentIndex) {
                        const TDF_Label& aComponent =
                            aComponents.Value(aComponentIndex);
                        if (aComponent.IsNull()
                            || aComponent.Data()
                                != document->GetData()
                            || !XCAFDoc_ShapeTool::IsComponent(
                                aComponent)
                            || aStack.size()
                                >= static_cast<std::size_t>(
                                    kMaximumGeometryDocumentLabels)) {
                            return Standard_False;
                        }
                        aStack.push_back({
                            aComponent,
                            aFrame.depth + 1U,
                            false});
                    }
                    continue;
                }
                if (!IsGeometryDefinitionLabel(
                        document, aShapeTool, aLabel)) {
                    return Standard_False;
                }
                // This path-relative traversal reaches one resolved leaf per
                // occurrence, including repeated references to a shared
                // definition. Geometry itself remains classified once below.
				if (aLeafOccurrenceCount
						>= core3d::limits::kMaximumLeafPresentations) {
					return Standard_False;
				}
                ++aLeafOccurrenceCount;
                if (!aDefinitionLabels.Contains(aLabel)) {
                    if (aDefinitionCount
                            >= kMaximumGeometryDefinitionLabels
                        || !aDefinitionLabels.Add(aLabel)
                        || ValidatedGeometryRepresentation(
                               document, aShapeTool, aLabel, &aBudget)
                            == OcctGeometryRepresentation::Invalid) {
                        return Standard_False;
                    }
                    ++aDefinitionCount;
                }
            }
            if (!anActivePathForRoot.IsEmpty()) {
                return Standard_False;
            }
        }

		// The definition pass above validates authored and implicit axes at an
		// identity occurrence. Schema v6 publishes the resolved world axis for
		// every leaf, so admission must also prove each real cumulative XCAF
		// occurrence location. Use the same explorer/location authority as the
		// snapshot builder and cross-check its count against the bounded graph
		// traversal so neither path can silently omit a leaf.
		Standard_Size aResolvedReferenceOccurrenceCount = 0;
		XCAFPrs_DocumentExplorer anOccurrenceExplorer(
			document,
			XCAFPrs_DocumentExplorerFlags_OnlyLeafNodes,
			XCAFPrs_Style());
		for (; anOccurrenceExplorer.More(); anOccurrenceExplorer.Next()) {
			if (aResolvedReferenceOccurrenceCount
					>= core3d::limits::kMaximumLeafPresentations) {
				return Standard_False;
			}
			const XCAFPrs_DocumentNode& aNode =
				anOccurrenceExplorer.Current();
			const TDF_Label aDefinitionLabel = aNode.RefLabel.IsNull()
				? aNode.Label : aNode.RefLabel;
			gp_Ax1 aResolvedReferenceAxis;
			if (aDefinitionLabel.IsNull()
				|| !aDefinitionLabels.Contains(aDefinitionLabel)
				|| !TryResolveReferenceAxis(
					document,
					aShapeTool,
					aDefinitionLabel,
					aNode.Location,
					aResolvedReferenceAxis)) {
				return Standard_False;
			}
			++aResolvedReferenceOccurrenceCount;
		}
		if (aResolvedReferenceOccurrenceCount != aLeafOccurrenceCount) {
			return Standard_False;
		}
        for (TDF_MapIteratorOfLabelMap aTopLevel(
                 anAllTopLevelLabelSet);
             aTopLevel.More(); aTopLevel.Next()) {
            if (!aVisitedGraphLabels.Contains(aTopLevel.Key())) {
                // Non-free top-level labels must be reachable from some free
                // root. This rejects rootless assembly cycles and orphaned
                // reference islands even if their individual labels look
                // structurally plausible.
                return Standard_False;
            }
        }

        TDF_LabelMap aValidatedSubshapeLabels;
        Standard_Size aSubshapeLabelCount = 0;
        for (TDF_MapIteratorOfLabelMap aDefinition(
                 aDefinitionLabels);
             aDefinition.More(); aDefinition.Next()) {
            TDF_LabelSequence aSubshapes;
            if (!XCAFDoc_ShapeTool::GetSubShapes(
                    aDefinition.Key(), aSubshapes)) {
                continue;
            }
            if (static_cast<Standard_Size>(aSubshapes.Length())
                > kMaximumGeometryDocumentLabels) {
                return Standard_False;
            }
            for (Standard_Integer aSubshapeIndex = 1;
                 aSubshapeIndex <= aSubshapes.Length();
                 ++aSubshapeIndex) {
                const TDF_Label& aSubshape =
                    aSubshapes.Value(aSubshapeIndex);
                if (aSubshape.IsNull()
                    || aSubshape.Data() != document->GetData()
                    || !aShapeTool->IsShape(aSubshape)
                    || !XCAFDoc_ShapeTool::IsSubShape(aSubshape)) {
                    return Standard_False;
                }
                if (!aValidatedSubshapeLabels.Contains(aSubshape)) {
                    if (aSubshapeLabelCount
                            >= kMaximumGeometryDocumentLabels
                        || !aValidatedSubshapeLabels.Add(aSubshape)) {
                        return Standard_False;
                    }
                    ++aSubshapeLabelCount;
                }
            }
        }

        Standard_Size aLabelCount = 0;
        Standard_Size partitionBytes = 0;
        for (TDF_ChildIterator aLabel(aRoot, Standard_True);
             aLabel.More(); aLabel.Next()) {
            if (++aLabelCount > kMaximumGeometryDocumentLabels) {
                return Standard_False;
            }
            Handle(TDataStd_Integer) aMarker;
            Handle(TNaming_NamedShape) aNamedShape;
            if (aLabel.Value().FindAttribute(
                    GeometryRepresentationAttributeID(), aMarker)
                && !aDefinitionLabels.Contains(aLabel.Value())) {
                return Standard_False;
            }
            if (aLabel.Value().FindAttribute(
                    TNaming_NamedShape::GetID(), aNamedShape)
                && !aVisitedGraphLabels.Contains(aLabel.Value())
                && !aValidatedSubshapeLabels.Contains(
                    aLabel.Value())) {
                return Standard_False;
            }
            if (HasMeshRegionPartitionSchemaAttribute(aLabel.Value())) {
                Handle(TDF_Attribute) marker;
                std::vector<Standard_Byte> encoded;
                const TDF_Label definition = aLabel.Value().Father();
                if (!aLabel.Value().FindAttribute(MeshRegionPartitionRecordID(), marker)
                    || Handle(TDataStd_UAttribute)::DownCast(marker).IsNull()
                    || definition.IsNull() || !aDefinitionLabels.Contains(definition)
                    || ReadMeshRegionPartitionRecord(document, definition, encoded)
                        != MeshRegionPartitionReadState::Valid
                    || encoded.size() > kMaximumMeshRegionPartitionDocumentBytes - partitionBytes) {
                    return Standard_False;
                }
                partitionBytes += encoded.size();
            }
        }
        if (output != nullptr) {
            GeometryDocumentUsage usage;
            usage.geometry = aBudget;
            usage.definitions = aDefinitionCount;
            usage.labels = aLabelCount;
            usage.graphVisits = anAggregateGraphVisitCount;
            usage.leafOccurrences = aLeafOccurrenceCount;
            Standard_Size compositeNodes=0;for(const auto& record:composites)compositeNodes+=record.value->definition.nodes.size();
            usage.featureRecords = static_cast<Standard_Size>(profiles.size() + enclosures.size() + sweeps.size() + lofts.size() + 2*retained.size())+compositeNodes;
            *output = usage;
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

} // namespace

Standard_Boolean Core3DValidateRetainedSolidDocument(const Handle(TDocStd_Document)& document){
    return ValidateGeometryDocument(document,nullptr);
}

Standard_Boolean Core3DValidateCompositeRecipeDocument(const Handle(TDocStd_Document)& document){
    return ValidateGeometryDocument(document,nullptr);
}

Standard_Boolean Core3DValidateBoundedCurveDocument(const Handle(TDocStd_Document)& document){
    std::vector<core3d::bounded_curve::Record> records;
    return core3d::bounded_curve::ReadAll(document,records) ? Standard_True : Standard_False;
}

OcctRetainedRecipeCoverage OcctDocument::RetainedRecipeCoverageForLabel(
    const TDF_Label& label) const noexcept
{
    try {
        if (myOcafDoc.IsNull() || label.IsNull()
            || label.Data() != myOcafDoc->GetData()) {
            return OcctRetainedRecipeCoverage::InvalidOrUnknown;
        }
        // These reads distinguish no record from partial/corrupt records. Do
        // not use HasRecord here: an incomplete record must refuse, not look
        // like a legacy BRep-only solid.
        core3d::composite_recipe::Record composite;
        core3d::retained_solid::Record retained;
        core3d::profile::Record profile;
        core3d::enclosure::Record enclosure;
        core3d::sweep_persistence::Record sweep;
        core3d::loft_persistence::Record loft;
        if (!core3d::composite_recipe::Read(myOcafDoc, label, composite)
            || !core3d::retained_solid::Read(myOcafDoc, label, retained)
            || !core3d::profile::Read(myOcafDoc, label, profile)
            || !core3d::enclosure::Read(myOcafDoc, label, enclosure)
            || !core3d::sweep_persistence::Read(myOcafDoc, label, sweep)
            || !core3d::loft_persistence::Read(myOcafDoc, label, loft)) {
            return OcctRetainedRecipeCoverage::InvalidOrUnknown;
        }
        if (composite.value || retained.value || !enclosure.label.IsNull()
            || !sweep.label.IsNull() || !loft.label.IsNull()) {
            return OcctRetainedRecipeCoverage::PresentOutsideP4Coverage;
        }
        if (profile.label.IsNull()) return OcctRetainedRecipeCoverage::Absent;
        return profile.IsCurrent(myOcafDoc, label)
            ? OcctRetainedRecipeCoverage::CurrentProfile
            : OcctRetainedRecipeCoverage::PresentOutsideP4Coverage;
    } catch (...) {
        return OcctRetainedRecipeCoverage::InvalidOrUnknown;
    }
}

namespace {

// Independent rebuild of a plain (shell-free, non-revolve) profile recipe
// into one oriented, valid solid. This mirrors the creation-time construction
// exactly so the recipe alone can be proved against the captured root BRep;
// it owns no label, document, or command.
TopoDS_Shape BuildPlainProfileOperationSolid(
    const core3d::profile::Parameters& parameters) noexcept
{
    try {
        OCC_CATCH_SIGNALS
        const core3d::ProfileDefinition& aDefinition = parameters.definition;
        if (!parameters.shells.empty() || aDefinition.revolve
            || aDefinition.plane < 0 || aDefinition.plane > 2) {
            return TopoDS_Shape();
        }
        double aSignedArea = 0.0, anExpectedVolume = 0.0;
        if (!core3d::ProfileDefinitionExpectedVolume(
                aDefinition, aSignedArea, anExpectedVolume)) {
            return TopoDS_Shape();
        }
        const std::atomic_bool aCancelled{false};
        const gp_Dir aNormal = aDefinition.plane == 0 ? gp::DZ()
            : (aDefinition.plane == 1 ? gp_Dir(0, -1, 0) : gp::DX());
        const gp_Dir anUDirection =
            aDefinition.plane == 2 ? gp::DY() : gp::DX();
        TopoDS_Face aProfileFace;
        if (aDefinition.curves) {
            core3d::ProfileCurveFaceResult aBuilt;
            if (!core3d::BuildProfileCurveFace(
                    *aDefinition.curves, aDefinition.plane,
                    aCancelled, aBuilt)) {
                return TopoDS_Shape();
            }
            aProfileFace = aBuilt.face;
        } else if (aDefinition.circle) {
            const core3d::ProfileCircularSection& aCircle =
                *aDefinition.circle;
            const gp_Pnt aCenter =
                core3d::ProfilePointInPlane(aCircle.center, aDefinition.plane);
            const gp_Ax2 aBasis(aCenter, aNormal, anUDirection);
            BRepBuilderAPI_MakeEdge anOuterEdge(
                gp_Circ(aBasis, aCircle.outerRadius));
            if (!anOuterEdge.IsDone()) {
                return TopoDS_Shape();
            }
            BRepBuilderAPI_MakeWire anOuterWire(anOuterEdge.Edge());
            if (!anOuterWire.IsDone()) {
                return TopoDS_Shape();
            }
            BRepBuilderAPI_MakeFace aFace(
                gp_Pln(aCenter, aNormal), anOuterWire.Wire(), Standard_True);
            if (!aFace.IsDone()) {
                return TopoDS_Shape();
            }
            if (aCircle.innerRadius > 0) {
                BRepBuilderAPI_MakeEdge anInnerEdge(
                    gp_Circ(aBasis, aCircle.innerRadius));
                if (!anInnerEdge.IsDone()) {
                    return TopoDS_Shape();
                }
                BRepBuilderAPI_MakeWire anInnerWire(anInnerEdge.Edge());
                if (!anInnerWire.IsDone()) {
                    return TopoDS_Shape();
                }
                aFace.Add(TopoDS::Wire(anInnerWire.Wire().Reversed()));
                if (!aFace.IsDone()) {
                    return TopoDS_Shape();
                }
            }
            aProfileFace = aFace.Face();
        } else {
            std::vector<gp_Pnt2d> aPoints = aDefinition.points;
            if (aSignedArea < 0) {
                std::reverse(aPoints.begin(), aPoints.end());
            }
            BRepBuilderAPI_MakePolygon aPolygon;
            for (const gp_Pnt2d& aPoint : aPoints) {
                aPolygon.Add(
                    core3d::ProfilePointInPlane(aPoint, aDefinition.plane));
            }
            aPolygon.Close();
            if (!aPolygon.IsDone()) {
                return TopoDS_Shape();
            }
            BRepBuilderAPI_MakeFace aFace(aPolygon.Wire(), Standard_True);
            if (!aFace.IsDone()) {
                return TopoDS_Shape();
            }
            for (const core3d::ProfileCircularHole& aHole :
                 aDefinition.holes) {
                const gp_Ax2 aBasis(
                    core3d::ProfilePointInPlane(
                        aHole.center, aDefinition.plane),
                    aNormal, anUDirection);
                BRepBuilderAPI_MakeEdge anEdge(gp_Circ(aBasis, aHole.radius));
                if (!anEdge.IsDone()) {
                    return TopoDS_Shape();
                }
                BRepBuilderAPI_MakeWire aWire(anEdge.Edge());
                if (!aWire.IsDone()) {
                    return TopoDS_Shape();
                }
                aFace.Add(TopoDS::Wire(aWire.Wire().Reversed()));
                if (!aFace.IsDone()) {
                    return TopoDS_Shape();
                }
            }
            aProfileFace = aFace.Face();
        }
        if (aProfileFace.IsNull()
            || !BRepCheck_Analyzer(aProfileFace, Standard_True).IsValid()) {
            return TopoDS_Shape();
        }
        const gp_Vec aDirection =
            aDefinition.plane == 0 ? gp_Vec(0, 0, aDefinition.depth)
            : aDefinition.plane == 1 ? gp_Vec(0, aDefinition.depth, 0)
            : gp_Vec(aDefinition.depth, 0, 0);
        BRepPrimAPI_MakePrism aPrism(
            aProfileFace, aDirection, Standard_True, Standard_True);
        if (!aPrism.IsDone()) {
            return TopoDS_Shape();
        }
        TopoDS_Shape aResult = aPrism.Shape();
        if (aResult.IsNull() || aResult.ShapeType() != TopAbs_SOLID) {
            return TopoDS_Shape();
        }
        Standard_Real anExpected = anExpectedVolume;
        if (parameters.constructionFrame) {
            gp_Trsf aFrame;
            if (!parameters.constructionFrame->Transform(aFrame)) {
                return TopoDS_Shape();
            }
            anExpected *= parameters.constructionFrame->AbsoluteVolumeScale();
            if (!std::isfinite(anExpected) || anExpected <= 0) {
                return TopoDS_Shape();
            }
            // Only detached rebuild geometry is transformed; the persisted
            // document shape is never touched by this proof.
            BRepBuilderAPI_Transform aTransformed(
                aResult, aFrame, Standard_True, Standard_False);
            if (!aTransformed.IsDone()) {
                return TopoDS_Shape();
            }
            aResult = aTransformed.Shape();
            if (aResult.IsNull() || aResult.ShapeType() != TopAbs_SOLID) {
                return TopoDS_Shape();
            }
        }
        TopoDS_Solid aSolid = TopoDS::Solid(aResult);
        if (!BRepLib::OrientClosedSolid(aSolid)
            || !BRepCheck_Analyzer(aSolid, Standard_True).IsValid()) {
            return TopoDS_Shape();
        }
        GProp_GProps aProperties;
        BRepGProp::VolumeProperties(aSolid, aProperties);
        if (!std::isfinite(aProperties.Mass()) || aProperties.Mass() <= 0
            || std::abs(aProperties.Mass() - anExpected)
                > std::max(1e-8, anExpected * 1e-8)) {
            return TopoDS_Shape();
        }
        return aSolid;
    } catch (...) {
        return TopoDS_Shape();
    }
}

// Geometric correspondence between the independent recipe rebuild and the
// captured root: equal volume, centre of mass, optimal bounds and bounded
// face/edge census. TShape identity is deliberately never trusted.
Standard_Boolean PlainProfileRebuildCorresponds(
    const TopoDS_Shape& theRebuilt,
    const TopoDS_Shape& theStored) noexcept
{
    try {
        OCC_CATCH_SIGNALS
        if (theRebuilt.IsNull() || theStored.IsNull()
            || theRebuilt.ShapeType() != TopAbs_SOLID
            || theStored.ShapeType() != TopAbs_SOLID) {
            return Standard_False;
        }
        GProp_GProps aRebuiltVolume, aStoredVolume;
        BRepGProp::VolumeProperties(theRebuilt, aRebuiltVolume);
        BRepGProp::VolumeProperties(theStored, aStoredVolume);
        const Standard_Real aStoredMass = aStoredVolume.Mass();
        if (!std::isfinite(aStoredMass) || aStoredMass <= 0
            || !std::isfinite(aRebuiltVolume.Mass())
            || std::abs(aRebuiltVolume.Mass() - aStoredMass)
                > std::max(1e-7, aStoredMass * 1e-7)) {
            return Standard_False;
        }
        Bnd_Box aRebuiltBox, aStoredBox;
        BRepBndLib::AddOptimal(
            theRebuilt, aRebuiltBox, Standard_False, Standard_False);
        BRepBndLib::AddOptimal(
            theStored, aStoredBox, Standard_False, Standard_False);
        if (aRebuiltBox.IsVoid() || aStoredBox.IsVoid()
            || aRebuiltBox.IsOpen() || aStoredBox.IsOpen()) {
            return Standard_False;
        }
        Standard_Real aRebuiltMin[3], aRebuiltMax[3];
        Standard_Real aStoredMin[3], aStoredMax[3];
        aRebuiltBox.Get(
            aRebuiltMin[0], aRebuiltMin[1], aRebuiltMin[2],
            aRebuiltMax[0], aRebuiltMax[1], aRebuiltMax[2]);
        aStoredBox.Get(
            aStoredMin[0], aStoredMin[1], aStoredMin[2],
            aStoredMax[0], aStoredMax[1], aStoredMax[2]);
        Standard_Real aScale = 1.0;
        for (Standard_Integer anAxis = 0; anAxis < 3; ++anAxis) {
            aScale = std::max(
                aScale, std::abs(aStoredMax[anAxis] - aStoredMin[anAxis]));
        }
        const Standard_Real aTolerance = 1e-7 * aScale;
        const gp_Pnt aRebuiltCentre = aRebuiltVolume.CentreOfMass();
        const gp_Pnt aStoredCentre = aStoredVolume.CentreOfMass();
        if (aRebuiltCentre.Distance(aStoredCentre) > aTolerance) {
            return Standard_False;
        }
        for (Standard_Integer anAxis = 0; anAxis < 3; ++anAxis) {
            if (std::abs(aRebuiltMin[anAxis] - aStoredMin[anAxis])
                    > aTolerance
                || std::abs(aRebuiltMax[anAxis] - aStoredMax[anAxis])
                    > aTolerance) {
                return Standard_False;
            }
        }
        TopTools_IndexedMapOfShape aRebuiltFaces, aStoredFaces;
        TopTools_IndexedMapOfShape aRebuiltEdges, aStoredEdges;
        TopExp::MapShapes(theRebuilt, TopAbs_FACE, aRebuiltFaces);
        TopExp::MapShapes(theStored, TopAbs_FACE, aStoredFaces);
        TopExp::MapShapes(theRebuilt, TopAbs_EDGE, aRebuiltEdges);
        TopExp::MapShapes(theStored, TopAbs_EDGE, aStoredEdges);
        return aRebuiltFaces.Extent() == aStoredFaces.Extent()
            && aRebuiltEdges.Extent() == aStoredEdges.Extent();
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean PlainProfilePlacementsMatch(
    const gp_Trsf& theLeft,
    const gp_Trsf& theRight) noexcept
{
    try {
        for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
            for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
                const Standard_Real aLeft = theLeft.Value(aRow, aColumn);
                const Standard_Real aRight = theRight.Value(aRow, aColumn);
                if (!std::isfinite(aLeft) || !std::isfinite(aRight)
                    || std::abs(aLeft - aRight) > Precision::Confusion()) {
                    return Standard_False;
                }
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

} // namespace

Standard_Boolean OcctDocument::CapturePlainProfileOperationSource(
    const TDF_Label& label,
    OcctPlainProfileOperationCapture& output) const noexcept
{
    output = OcctPlainProfileOperationCapture{};
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || label.IsNull()
            || label.Data() != myOcafDoc->GetData()
            || RetainedRecipeCoverageForLabel(label)
                != OcctRetainedRecipeCoverage::CurrentProfile
            || !IsEditableFreeSimpleDefinitionLabel(label)) {
            return Standard_False;
        }
        // A well-formed, current plain profile: the co-owner census above
        // already refused composite/retained-solid/sweep/loft/enclosure
        // records; the empty shell tail and the non-revolve extrusion family
        // are the remaining admitted signature.
        core3d::profile::Record aRecord;
        if (!core3d::profile::Read(myOcafDoc, label, aRecord)
            || aRecord.label.IsNull()
            || !aRecord.parameters.shells.empty()
            || aRecord.parameters.definition.revolve
            || !aRecord.IsCurrent(myOcafDoc, label)) {
            return Standard_False;
        }
        OcctPlainProfileOperationCapture aCapture;
        aCapture.label = label;
        aCapture.recordLabel = aRecord.label;
        aCapture.entityIdentifier = EntityIdentifierForLabel(label);
        aCapture.definitionIdentifier = DefinitionIdentifierForLabel(label);
        aCapture.featureIdentifier = aRecord.identifier;
        aCapture.recipeValues = aRecord.values;
        aCapture.metersPerUnit = aRecord.parameters.metersPerUnit;
        if (aCapture.entityIdentifier.empty()
            || aCapture.definitionIdentifier.empty()
            || aCapture.featureIdentifier.empty()
            || aCapture.recipeValues.empty()
            || !std::isfinite(aCapture.metersPerUnit)
            || aCapture.metersPerUnit <= 0
            || !TryObjectTransformForLabel(label, aCapture.placement)) {
            output = OcctPlainProfileOperationCapture{};
            return Standard_False;
        }
        const TopoDS_Shape aRoot = XCAFDoc_ShapeTool::GetShape(label);
        if (aRoot.IsNull() || aRoot.ShapeType() != TopAbs_SOLID
            || aRecord.boundShape.IsNull()
            || !aRecord.boundShape.IsEqual(aRoot)
            || !BRepCheck_Analyzer(aRoot, Standard_True).IsValid()) {
            output = OcctPlainProfileOperationCapture{};
            return Standard_False;
        }
        aCapture.boundRoot = aRoot;
        // Independent rebuild correspondence: the recipe alone must reproduce
        // the captured root before any edge treatment may commit over it.
        const TopoDS_Shape aRebuilt =
            BuildPlainProfileOperationSolid(aRecord.parameters);
        if (!PlainProfileRebuildCorresponds(aRebuilt, aRoot)) {
            output = OcctPlainProfileOperationCapture{};
            return Standard_False;
        }
        output = std::move(aCapture);
        return Standard_True;
    } catch (...) {
        output = OcctPlainProfileOperationCapture{};
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::PlainProfileOperationSourceCurrent(
    const OcctPlainProfileOperationCapture& capture) const noexcept
{
    try {
        OcctPlainProfileOperationCapture aLive;
        if (capture.label.IsNull()
            || !CapturePlainProfileOperationSource(capture.label, aLive)
            || aLive.recordLabel.IsNull() || capture.recordLabel.IsNull()
            || !aLive.recordLabel.IsEqual(capture.recordLabel)
            || aLive.entityIdentifier != capture.entityIdentifier
            || aLive.definitionIdentifier != capture.definitionIdentifier
            || aLive.featureIdentifier != capture.featureIdentifier
            || aLive.recipeValues != capture.recipeValues
            || aLive.metersPerUnit != capture.metersPerUnit
            || aLive.boundRoot.IsNull() || capture.boundRoot.IsNull()
            || !aLive.boundRoot.IsEqual(capture.boundRoot)
            || !PlainProfilePlacementsMatch(
                aLive.placement, capture.placement)) {
            return Standard_False;
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::VerifyPlainProfileOperationReplacement(
    const OcctPlainProfileOperationCapture& capture,
    const TopoDS_Shape& newRoot) const noexcept
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || capture.label.IsNull()
            || capture.label.Data() != myOcafDoc->GetData()
            || newRoot.IsNull() || newRoot.ShapeType() != TopAbs_SOLID) {
            return Standard_False;
        }
        const TopoDS_Shape aStored =
            XCAFDoc_ShapeTool::GetShape(capture.label);
        core3d::profile::Record aRecord;
        Standard_Real aUnit = 0.0;
        gp_Trsf aPlacement;
        if (aStored.IsNull() || !aStored.IsEqual(newRoot)
            || !core3d::profile::Read(myOcafDoc, capture.label, aRecord)
            || aRecord.label.IsNull() || capture.recordLabel.IsNull()
            || !aRecord.label.IsEqual(capture.recordLabel)
            || aRecord.identifier != capture.featureIdentifier
            || aRecord.values != capture.recipeValues
            || aRecord.boundShape.IsNull() || capture.boundRoot.IsNull()
            // The old record keeps its original bound shape; only the root
            // changed, so the retained recipe is now stale, never erased,
            // rebound, or silently current again.
            || !aRecord.boundShape.IsEqual(capture.boundRoot)
            || aRecord.IsCurrent(myOcafDoc, capture.label)
            || EntityIdentifierForLabel(capture.label)
                != capture.entityIdentifier
            || DefinitionIdentifierForLabel(capture.label)
                != capture.definitionIdentifier
            || !XCAFDoc_DocumentTool::GetLengthUnit(myOcafDoc, aUnit)
            || aUnit != capture.metersPerUnit
            || !TryObjectTransformForLabel(capture.label, aPlacement)
            || !PlainProfilePlacementsMatch(aPlacement, capture.placement)) {
            return Standard_False;
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::ValidateGeometryRepresentations(
    const Handle(TDocStd_Document)& document) const
{
    return ValidateGeometryDocument(document, nullptr);
}

Standard_Boolean OcctDocument::ValidateReferenceAxes() const
{
    return ValidateReferenceAxes(myOcafDoc);
}

Standard_Boolean OcctDocument::ValidateReferenceAxes(
    const Handle(TDocStd_Document)& document) const
{
    return ValidateReferenceAxisDocument(document);
}

Standard_Boolean OcctDocument::CanDuplicateGeometryDefinitions(
    const std::vector<TDF_Label>& sourceDefinitionLabels) const
{
    try {
        OCC_CATCH_SIGNALS
        if (sourceDefinitionLabels.empty()
            || sourceDefinitionLabels.size()
                > static_cast<std::size_t>(
                    kMaximumGeometryDefinitionLabels)) {
            return Standard_False;
        }

        std::vector<OcctGeometryDuplicationRequest> requests;
        requests.reserve(sourceDefinitionLabels.size());
        for (const TDF_Label& source : sourceDefinitionLabels) {
            requests.push_back({source, 1U});
        }
        return CanDuplicateGeometryDefinitions(requests);
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::HasNoSavedSweepForTopology(const TDF_Label& label) const noexcept {
    if(core3d::retained_solid::HasRecord(label))return Standard_False;
    core3d::sweep_persistence::Record record;
    core3d::loft_persistence::Record loft;
    return core3d::sweep_persistence::Read(myOcafDoc,label,record) && record.label.IsNull()
        && core3d::loft_persistence::Read(myOcafDoc,label,loft) && loft.label.IsNull();
}

Standard_Boolean OcctDocument::CanDuplicateGeometryDefinitions(
    const std::vector<OcctGeometryDuplicationRequest>& requests) const
{
    try {
        OCC_CATCH_SIGNALS
        if (NativeBooleanOwnerBlocksOtherWork() || myOcafDoc.IsNull() || myOcafDoc->HasOpenCommand()
            || requests.empty()
            || requests.size()
                > static_cast<std::size_t>(
                    kMaximumGeometryDefinitionLabels)
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }

        GeometryDocumentUsage current;
        if (!ValidateGeometryDocument(myOcafDoc, &current)) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (shapeTool.IsNull()) {
            return Standard_False;
        }

        Standard_Size projectedNormalBytes = 0;
        if (!Core3DValidateOwnedFrameUsage(myOcafDoc,projectedNormalBytes)) return Standard_False;

        GeometryValidationBudget projectedGeometry = current.geometry;
        Standard_Size projectedDefinitions = current.definitions;
        Standard_Size projectedLabels = current.labels;
        Standard_Size projectedGraphVisits = current.graphVisits;
        Standard_Size projectedLeafOccurrences = current.leafOccurrences;
        Standard_Size projectedFeatures = current.featureRecords;
        TDF_LabelMap uniqueSources;
        for (const OcctGeometryDuplicationRequest& request : requests) {
            const TDF_Label& source = request.sourceDefinition;
            const Standard_Size destinationCount =
                request.destinationCount;
            if (source.IsNull()
                || source.Data() != myOcafDoc->GetData()
                || destinationCount == 0U
                || !IsEditableFreeSimpleDefinitionLabel(source)
                || !uniqueSources.Add(source)) {
                return Standard_False;
            }

            GeometryValidationBudget sourceGeometry;
            if (ValidatedGeometryRepresentation(
                    myOcafDoc, shapeTool, source, &sourceGeometry)
                    == OcctGeometryRepresentation::Invalid) {
                return Standard_False;
            }

            OcctAuthoredFrameRecord sourceFrame;
            if (Core3DReadAuthoredFrameOwner(myOcafDoc, source, sourceFrame) == OcctAuthoredFrameReadState::Invalid
                || !AddMultipliedWithinLimit(projectedNormalBytes, sourceFrame.nativeBytes, destinationCount,
                    64U * 1024U * 1024U)) return Standard_False;

            XCAFDoc_VisMaterialPBR sourceMaterial;
            if (TryPBRMaterialForLabel(source, sourceMaterial) && !sourceMaterial.NormalTexture.IsNull()) {
                Standard_Size bytes = 0;
                if (!Core3DValidateNormalTextureBinding(myOcafDoc, source, &bytes)
                    || !AddMultipliedWithinLimit(projectedNormalBytes, bytes, destinationCount,
                        64U * 1024U * 1024U)) return Standard_False;
            }

            // No independent-copy/baked-copy policy for saved sweeps yet.
            if (!HasNoSavedSweepForTopology(source)) return Standard_False;
            core3d::enclosure::Record sourceEnclosure;
            if (!core3d::enclosure::Read(myOcafDoc, source, sourceEnclosure)
                || (!sourceEnclosure.label.IsNull()
                    && (!request.preservesEnclosureRecipe || request.requiresProfileConstructionFrame)))
                return Standard_False;
            if (request.requiresEnclosureConstructionFrame
                && (!request.preservesEnclosureRecipe || sourceEnclosure.label.IsNull()))
                return Standard_False;
            core3d::profile::Record sourceProfile;
            if (!core3d::profile::Read(myOcafDoc, source, sourceProfile)
                || (request.requiresProfileConstructionFrame && sourceProfile.label.IsNull())) {
                return Standard_False;
            }
            // Simultaneous profile/enclosure owners have no admitted copy policy.
            if (!sourceProfile.label.IsNull() && !sourceEnclosure.label.IsNull()) return Standard_False;
            Standard_Size enclosureLabels = 0;
            if (!sourceEnclosure.label.IsNull()) {
                if (!AddMultipliedWithinLimit(projectedFeatures, 1U,
                        destinationCount, core3d::enclosure::MaximumRecords)) return Standard_False;
                enclosureLabels = 1U + static_cast<Standard_Size>(sourceEnclosure.values.size());
                if (request.requiresEnclosureConstructionFrame
                    && !sourceEnclosure.parameters.definition.constructionFrame) enclosureLabels += 8U;
                if (!sourceEnclosure.boundShape.IsEqual(XCAFDoc_ShapeTool::GetShape(source))
                    && ClassifyDefinitionGeometry(sourceEnclosure.boundShape, &sourceGeometry)
                        != DefinitionGeometryClass::BRep) return Standard_False;
            }
            Standard_Size profileLabels = 0;
            if (!sourceProfile.label.IsNull()) {
                if (!AddMultipliedWithinLimit(projectedFeatures, 1U,
                        destinationCount, core3d::profile::MaximumRecords)) {
                    return Standard_False;
                }
                profileLabels = 1U + static_cast<Standard_Size>(sourceProfile.values.size());
                if (request.requiresProfileConstructionFrame
                    && !sourceProfile.parameters.constructionFrame) profileLabels += 8U;
                // A recipe bound to a prior solid requires an independent copy
                // of that retained solid as well as the current root. Unit-only
                // staleness shares its copied root and incurs no second shape.
                if (!sourceProfile.boundShape.IsEqual(XCAFDoc_ShapeTool::GetShape(source))
                    && ClassifyDefinitionGeometry(sourceProfile.boundShape, &sourceGeometry)
                        != DefinitionGeometryClass::BRep) {
                    return Standard_False;
                }
            }

            // AddShape creates one definition label and eight transform
            // children. CopyObjectAppearance creates the legacy material and
            // color children only when a local PBR assignment is not
            // authoritative; visual-material/texture definitions are shared.
            Standard_Size destinationLabels = 9U + profileLabels + enclosureLabels;
            Handle(TDataStd_Integer) localPBRMarker;
            const Standard_Boolean hasLocalPBR =
                source.FindAttribute(
                    LocalPBRMaterialAttributeID(), localPBRMarker)
                && !localPBRMarker.IsNull()
                && localPBRMarker->Get() == 1;
            if (!hasLocalPBR) {
                Graphic3d_NameOfMaterial material;
                Quantity_NameOfColor color;
                if (TryMaterialNameForLabel(source, material)) {
                    ++destinationLabels;
                }
                if (TryColorNameForLabel(source, color)) {
                    ++destinationLabels;
                }
            }
            if (!AddMultipliedWithinLimit(
                    projectedDefinitions,
                    1U,
                    destinationCount,
                    kMaximumGeometryDefinitionLabels)
                || !AddMultipliedWithinLimit(
                    projectedGraphVisits,
                    1U,
                    destinationCount,
                    kMaximumGeometryDocumentLabels)
                || !AddMultipliedWithinLimit(
                    projectedLeafOccurrences,
                    1U,
                    destinationCount,
                    static_cast<Standard_Size>(
                        core3d::limits::kMaximumLeafPresentations))
                || !AddMultipliedWithinLimit(
                    projectedGeometry.subshapes,
                    sourceGeometry.subshapes,
                    destinationCount,
                    kMaximumSubshapesPerDocument)
                || !AddMultipliedWithinLimit(
                    projectedGeometry.meshVertices,
                    sourceGeometry.meshVertices,
                    destinationCount,
                    kMaximumMeshVerticesPerDocument)
                || !AddMultipliedWithinLimit(
                    projectedGeometry.meshIndices,
                    sourceGeometry.meshIndices,
                    destinationCount,
                    kMaximumMeshIndicesPerDocument)
                || !AddMultipliedWithinLimit(
                    projectedLabels,
                    destinationLabels,
                    destinationCount,
                    kMaximumGeometryDocumentLabels)) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Integer OcctDocument::SupportedGeometryExportFormats() const
{
    try {
        OCC_CATCH_SIGNALS
        if (NativeBooleanOwnerBlocksOtherWork() || myOcafDoc.IsNull() || myOcafDoc->HasOpenCommand()
            || !ValidateGeometryRepresentations()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return 0;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (aShapeTool.IsNull()) {
            return 0;
        }
        TDF_LabelSequence aLabels;
        aShapeTool->GetShapes(aLabels);
        if (static_cast<Standard_Size>(aLabels.Length())
            > kMaximumGeometryDocumentLabels) {
            return 0;
        }
        Standard_Integer aFormats =
            static_cast<Standard_Integer>(OcctGeometryExportFormat::Obj)
            | static_cast<Standard_Integer>(OcctGeometryExportFormat::Stl)
            | static_cast<Standard_Integer>(OcctGeometryExportFormat::Gltf)
            | static_cast<Standard_Integer>(OcctGeometryExportFormat::Step);
        Standard_Size aDefinitionCount = 0;
        for (Standard_Integer anIndex = 1;
             anIndex <= aLabels.Length(); ++anIndex) {
            const TDF_Label& aLabel = aLabels.Value(anIndex);
            if (aLabel.IsNull()
                || aLabel.Data() != myOcafDoc->GetData()
                || !aShapeTool->IsShape(aLabel)) {
                return 0;
            }
            if (!IsGeometryDefinitionLabel(
                    myOcafDoc, aShapeTool, aLabel)) {
                continue;
            }
            if (aDefinitionCount
                >= kMaximumGeometryDefinitionLabels) {
                return 0;
            }
            ++aDefinitionCount;
            const OcctGeometryRepresentation aRepresentation =
                GeometryRepresentationForLabel(aLabel);
            if (aRepresentation
                == OcctGeometryRepresentation::TriangleMesh) {
                aFormats &= ~static_cast<Standard_Integer>(
                    OcctGeometryExportFormat::Step);
            } else if (aRepresentation
                    != OcctGeometryRepresentation::LegacyUnknown
                && aRepresentation
                    != OcctGeometryRepresentation::BRep) {
                return 0;
            }
        }
        return aDefinitionCount > 0 ? aFormats : 0;
    } catch (...) {
        return 0;
    }
}

Standard_Boolean OcctDocument::CanExportGeometry(
    const OcctGeometryExportFormat format) const
{
    const Standard_Integer aRequested =
        static_cast<Standard_Integer>(format);
    const Standard_Integer aKnown =
        static_cast<Standard_Integer>(OcctGeometryExportFormat::Obj)
        | static_cast<Standard_Integer>(OcctGeometryExportFormat::Stl)
        | static_cast<Standard_Integer>(OcctGeometryExportFormat::Gltf)
        | static_cast<Standard_Integer>(OcctGeometryExportFormat::Step);
    return aRequested != 0 && (aRequested & ~aKnown) == 0
        && (SupportedGeometryExportFormats() & aRequested)
            == aRequested;
}

Standard_Boolean OcctDocument::IsGeometryDocumentEmpty() const
{
    try {
        OCC_CATCH_SIGNALS
        if (NativeBooleanOwnerBlocksOtherWork() || myOcafDoc.IsNull() || myOcafDoc->HasOpenCommand()
            || !ValidateGeometryRepresentations()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (aShapeTool.IsNull()) {
            return Standard_False;
        }
        TDF_LabelSequence aLabels;
        aShapeTool->GetShapes(aLabels);
        if (static_cast<Standard_Size>(aLabels.Length())
            > kMaximumGeometryDocumentLabels) {
            return Standard_False;
        }
        for (Standard_Integer anIndex = 1;
             anIndex <= aLabels.Length(); ++anIndex) {
            const TDF_Label& aLabel = aLabels.Value(anIndex);
            if (aLabel.IsNull()
                || aLabel.Data() != myOcafDoc->GetData()
                || !aShapeTool->IsShape(aLabel)) {
                return Standard_False;
            }
            if (IsGeometryDefinitionLabel(
                    myOcafDoc, aShapeTool, aLabel)) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::SetGeometryRepresentationForLabel(
    const TDF_Label& label,
    const OcctGeometryRepresentation representation)
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || representation == OcctGeometryRepresentation::Invalid
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (!IsGeometryDefinitionLabel(
                myOcafDoc, aShapeTool, label)) {
            return Standard_False;
        }
        bool hasMarker = false;
        OcctGeometryRepresentation anExistingRepresentation =
            OcctGeometryRepresentation::Invalid;
        if (!ReadGeometryRepresentation(
                label, hasMarker, anExistingRepresentation)) {
            return Standard_False;
        }
        const DefinitionGeometryClass aGeometryClass =
            ClassifyDefinitionGeometry(
                XCAFDoc_ShapeTool::GetShape(label), nullptr);
        if ((hasMarker
                && !GeometryClassMatchesRepresentation(
                    aGeometryClass, anExistingRepresentation))
            || !GeometryClassMatchesRepresentation(
                aGeometryClass, representation)) {
            return Standard_False;
        }
        return WriteGeometryRepresentationMarker(
            label, representation);
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean
OcctDocument::EnsureGeometryRepresentationForMutation(
    const TDF_Label& label)
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (!IsGeometryDefinitionLabel(
                myOcafDoc, aShapeTool, label)) {
            return Standard_False;
        }
        bool hasMarker = false;
        OcctGeometryRepresentation aRepresentation =
            OcctGeometryRepresentation::Invalid;
        if (!ReadGeometryRepresentation(
                label, hasMarker, aRepresentation)) {
            return Standard_False;
        }
        if (hasMarker
            && (aRepresentation == OcctGeometryRepresentation::BRep
                || aRepresentation
                    == OcctGeometryRepresentation::TriangleMesh)) {
            // Candidate documents and every geometry replacement validate the
            // explicit marker once. Ordinary scalar/transform mutations must
            // not rescan up to a million mesh vertices on the main thread.
            return Standard_True;
        }
        if (aRepresentation
                != OcctGeometryRepresentation::LegacyUnknown
            || ClassifyDefinitionGeometry(
                   XCAFDoc_ShapeTool::GetShape(label), nullptr)
                != DefinitionGeometryClass::BRep) {
            return Standard_False;
        }
        return WriteGeometryRepresentationMarker(
            label, OcctGeometryRepresentation::BRep);
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::CopyGeometryRepresentation(
    const TDF_Label& source,
    const TDF_Label& destination)
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        OcctGeometryRepresentation aSourceRepresentation =
            ValidatedGeometryRepresentation(
                myOcafDoc, aShapeTool, source);
        if (aSourceRepresentation
            == OcctGeometryRepresentation::LegacyUnknown) {
            aSourceRepresentation = OcctGeometryRepresentation::BRep;
        }
        if (aSourceRepresentation
                == OcctGeometryRepresentation::Invalid
            || !IsGeometryDefinitionLabel(
                myOcafDoc, aShapeTool, destination)) {
            return Standard_False;
        }
        bool hasDestinationMarker = false;
        OcctGeometryRepresentation aDestinationRepresentation =
            OcctGeometryRepresentation::Invalid;
        if (!ReadGeometryRepresentation(
                destination,
                hasDestinationMarker,
                aDestinationRepresentation)) {
            return Standard_False;
        }
        const DefinitionGeometryClass aDestinationClass =
            ClassifyDefinitionGeometry(
                XCAFDoc_ShapeTool::GetShape(destination), nullptr);
        if ((hasDestinationMarker
                && !GeometryClassMatchesRepresentation(
                    aDestinationClass, aDestinationRepresentation))
            || !GeometryClassMatchesRepresentation(
                aDestinationClass, aSourceRepresentation)) {
            return Standard_False;
        }
        return WriteGeometryRepresentationMarker(
            destination, aSourceRepresentation);
    } catch (...) {
        return Standard_False;
    }
}

namespace {

Standard_Boolean MarkImportedDefinitions(
    const Handle(TDocStd_Document)& theDocument,
    const OcctGeometryRepresentation theExpectedRepresentation)
{
    if (theDocument.IsNull() || theDocument->HasOpenCommand()
        || theDocument->GetAvailableUndos() != 0
        || theDocument->GetAvailableRedos() != 0
        || (theExpectedRepresentation != OcctGeometryRepresentation::BRep
            && theExpectedRepresentation
                != OcctGeometryRepresentation::TriangleMesh)
        || !XCAFDoc_DocumentTool::CheckShapeTool(
            theDocument->Main())) {
        return Standard_False;
    }

    const DefinitionGeometryClass anExpectedClass =
        theExpectedRepresentation == OcctGeometryRepresentation::BRep
        ? DefinitionGeometryClass::BRep
        : DefinitionGeometryClass::TriangleMesh;
    const Handle(XCAFDoc_ShapeTool) aShapeTool =
        XCAFDoc_DocumentTool::ShapeTool(theDocument->Main());
    if (aShapeTool.IsNull()) {
        return Standard_False;
    }
    TDF_LabelSequence aShapeLabels;
    aShapeTool->GetShapes(aShapeLabels);
    if (aShapeLabels.IsEmpty()
        || static_cast<Standard_Size>(aShapeLabels.Length())
            > kMaximumGeometryDocumentLabels) {
        return Standard_False;
    }

    std::vector<TDF_Label> aLegacyDefinitions;
    TDF_LabelMap aVisitedDefinitions;
    GeometryValidationBudget aBudget;
    try {
        OCC_CATCH_SIGNALS
        aLegacyDefinitions.reserve(
            static_cast<std::size_t>(aShapeLabels.Length()));
        for (Standard_Integer anIndex = 1;
             anIndex <= aShapeLabels.Length(); ++anIndex) {
            const TDF_Label& aTopLevelLabel =
                aShapeLabels.Value(anIndex);
            if (XCAFDoc_ShapeTool::IsAssembly(aTopLevelLabel)) {
                continue;
            }
            TDF_Label aLabel = aTopLevelLabel;
            if (XCAFDoc_ShapeTool::IsReference(aTopLevelLabel)) {
                if (!XCAFDoc_ShapeTool::GetReferredShape(
                        aTopLevelLabel, aLabel)
                    || aLabel.IsNull()) {
                    return Standard_False;
                }
                if (XCAFDoc_ShapeTool::IsAssembly(aLabel)) {
                    continue;
                }
            }
            if (!IsGeometryDefinitionLabel(
                    theDocument, aShapeTool, aLabel)) {
                return Standard_False;
            }
            if (aVisitedDefinitions.Contains(aLabel)) {
                continue;
            }
            if (!aVisitedDefinitions.Add(aLabel)
                || static_cast<Standard_Size>(
                    aVisitedDefinitions.Extent())
                    > kMaximumGeometryDefinitionLabels
                || ClassifyDefinitionGeometry(
                    XCAFDoc_ShapeTool::GetShape(aLabel), &aBudget)
                    != anExpectedClass) {
                return Standard_False;
            }

            bool hasMarker = false;
            OcctGeometryRepresentation aRepresentation =
                OcctGeometryRepresentation::Invalid;
            if (!ReadGeometryRepresentation(
                    aLabel, hasMarker, aRepresentation)) {
                return Standard_False;
            }
            (void)hasMarker;
            if (aRepresentation
                    == OcctGeometryRepresentation::LegacyUnknown) {
                aLegacyDefinitions.push_back(aLabel);
            } else if (aRepresentation
                    != theExpectedRepresentation) {
                return Standard_False;
            }
        }
        if (aVisitedDefinitions.IsEmpty()) {
            return Standard_False;
        }
        if (aLegacyDefinitions.empty()) {
            return Standard_True;
        }

        const Standard_Integer aPreviousUndoLimit =
            theDocument->GetUndoLimit();
        theDocument->SetUndoLimit(1);
        theDocument->NewCommand();
        if (!theDocument->HasOpenCommand()) {
            theDocument->SetUndoLimit(aPreviousUndoLimit);
            return Standard_False;
        }
        try {
            for (const TDF_Label& aLabel : aLegacyDefinitions) {
                if (!WriteGeometryRepresentationMarker(
                        aLabel, theExpectedRepresentation)) {
                    throw Standard_Failure(
                        "Unable to mark imported geometry definition");
                }
            }
            if (!theDocument->CommitCommand()) {
                throw Standard_Failure(
                    "Unable to commit imported geometry markers");
            }
            theDocument->ClearUndos();
            theDocument->SetUndoLimit(aPreviousUndoLimit);
        } catch (...) {
            AbortCommandNoThrow(theDocument);
            try {
                theDocument->ClearUndos();
            } catch (...) {
            }
            try {
                theDocument->SetUndoLimit(aPreviousUndoLimit);
            } catch (...) {
            }
            return Standard_False;
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

} // namespace

Standard_Boolean OcctDocument::MarkImportedBRepDefinitions()
{
    return MarkImportedDefinitions(
               myOcafDoc, OcctGeometryRepresentation::BRep)
        && ValidateGeometryRepresentations();
}

Standard_Boolean OcctDocument::MarkImportedTriangleMeshDefinitions()
{
    return MarkImportedDefinitions(
               myOcafDoc,
               OcctGeometryRepresentation::TriangleMesh)
        && ValidateGeometryRepresentations();
}

Standard_Boolean OcctDocument::MigrateLegacyIdentifiers()
{
    return MigrateLegacyIdentifiers(myOcafDoc);
}

Standard_Boolean OcctDocument::MigrateLegacyIdentifiers(
    const Handle(TDocStd_Document)& document)
{
  try {
    OCC_CATCH_SIGNALS
    if (document.IsNull() || document->HasOpenCommand()) {
        return Standard_False;
    }

    TDF_LabelMap entityLabels;
    TDF_LabelMap definitionLabels;
    try {
        OCC_CATCH_SIGNALS
        XCAFPrs_DocumentExplorer anExplorer(
            document,
            XCAFPrs_DocumentExplorerFlags_None);
        for (; anExplorer.More(); anExplorer.Next()) {
            const XCAFPrs_DocumentNode& aNode = anExplorer.Current();
            if (!aNode.Label.IsNull()) {
                entityLabels.Add(aNode.Label);
            }
            const TDF_Label& aDefinitionLabel = aNode.RefLabel.IsNull()
                ? aNode.Label
                : aNode.RefLabel;
            if (!aDefinitionLabel.IsNull()) {
                definitionLabels.Add(aDefinitionLabel);
            }
        }
    } catch (...) {
        return Standard_False;
    }

    const TCollection_ExtendedString aBinXCAFFormat("BinXCAF");
    const Standard_Boolean needsStorageFormat =
        !document->StorageFormat().IsEqual(aBinXCAFFormat);
    const Standard_Boolean needsDocumentIdentifier =
        ReadIdentifier(document->Main(), DocumentIdentifierAttributeID()).empty();
    const Standard_Boolean needsVisMaterialTool =
        !XCAFDoc_DocumentTool::CheckVisMaterialTool(document->Main());
    std::vector<TDF_Label> entityIdentifiersNeedingAssignment;
    std::vector<TDF_Label> definitionIdentifiersNeedingAssignment;
    std::set<std::string> entityIdentifiers;
    std::set<std::string> definitionIdentifiers;
    for (TDF_MapIteratorOfLabelMap anEntity(entityLabels);
         anEntity.More(); anEntity.Next()) {
        const std::string anIdentifier = ReadIdentifier(
            anEntity.Key(), EntityIdentifierAttributeID());
        if (anIdentifier.empty()
            || !entityIdentifiers.insert(anIdentifier).second) {
            entityIdentifiersNeedingAssignment.push_back(anEntity.Key());
        }
    }
    for (TDF_MapIteratorOfLabelMap aDefinition(definitionLabels);
         aDefinition.More(); aDefinition.Next()) {
        const std::string anIdentifier = ReadIdentifier(
            aDefinition.Key(), DefinitionIdentifierAttributeID());
        if (anIdentifier.empty()
            || !definitionIdentifiers.insert(anIdentifier).second) {
            definitionIdentifiersNeedingAssignment.push_back(aDefinition.Key());
        }
    }

    const Standard_Boolean needsSchemaMigration =
        needsDocumentIdentifier
        || needsVisMaterialTool
        || !entityIdentifiersNeedingAssignment.empty()
        || !definitionIdentifiersNeedingAssignment.empty();
    if (!needsStorageFormat
        && !needsSchemaMigration) {
        return Standard_True;
    }

    // Identity migration is a schema operation and must never erase an active
    // user's history. Callers run it immediately after import/open, before the
    // document is published for editing.
    if (document->GetAvailableUndos() != 0
        || document->GetAvailableRedos() != 0) {
        return Standard_False;
    }

    const Standard_Integer aPreviousUndoLimit = document->GetUndoLimit();
    const TCollection_ExtendedString aPreviousStorageFormat =
        document->StorageFormat();
    try {
        OCC_CATCH_SIGNALS

        if (needsSchemaMigration) {
            document->SetUndoLimit(1);
            document->NewCommand();
            if (!document->HasOpenCommand()) {
                throw Standard_Failure("Unable to start document migration");
            }

            if (needsDocumentIdentifier
                && !AssignNewIdentifier(
                    document->Main(), DocumentIdentifierAttributeID())) {
                throw Standard_Failure("Unable to migrate document identifier");
            }
            if (needsVisMaterialTool
                && XCAFDoc_DocumentTool::VisMaterialTool(
                       document->Main()).IsNull()) {
                throw Standard_Failure("Unable to migrate XCAF material tool");
            }
            for (const TDF_Label& aLabel : entityIdentifiersNeedingAssignment) {
                if (!AssignNewIdentifier(
                        aLabel, EntityIdentifierAttributeID())) {
                    throw Standard_Failure("Unable to migrate entity identifier");
                }
            }
            for (const TDF_Label& aLabel : definitionIdentifiersNeedingAssignment) {
                if (!AssignNewIdentifier(
                        aLabel, DefinitionIdentifierAttributeID())) {
                    throw Standard_Failure("Unable to migrate definition identifier");
                }
            }

            if (!document->CommitCommand()) {
                throw Standard_Failure("Unable to commit document migration");
            }
            document->ClearUndos();
        }

        // Storage format is document metadata, not an OCAF attribute. Promote
        // it only after schema changes are safely committed, and never wrap a
        // format-only upgrade in an empty undo command.
        if (needsStorageFormat) {
            document->ChangeStorageFormat(aBinXCAFFormat);
        }
        document->SetUndoLimit(aPreviousUndoLimit);
        return Standard_True;
    } catch (...) {
        AbortCommandNoThrow(document);
        try {
            document->ClearUndos();
        } catch (...) {
        }
        try {
            document->SetUndoLimit(aPreviousUndoLimit);
        } catch (...) {
        }
        if (needsStorageFormat
            && !document->StorageFormat().IsEqual(
                aPreviousStorageFormat)) {
            try {
                document->ChangeStorageFormat(aPreviousStorageFormat);
            } catch (...) {
                // The candidate document will be rejected by the caller. Do
                // not let a secondary format-restore failure escape C++.
            }
        }
        return Standard_False;
    }
  } catch (...) {
    return Standard_False;
  }
}

void OcctDocument::RemoveShape(Handle(AIS_InteractiveObject) object) {
    RemoveShape(ShapeLabel(object));
}

void OcctDocument::RemoveShape(Handle(AIS_Shape) aisShape) {
    RemoveShape(ShapeLabel(aisShape));
}

void OcctDocument::RemoveShape(TopoDS_Shape object) {
    if (myOcafDoc.IsNull() || object.IsNull()) {
        return;
    }
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label;
    if(shapeTool->FindShape(object, label)
       || shapeTool->FindShape(object, label, Standard_True)) {
        RemoveShape(label);
    }
}

TDF_Label OcctDocument::ShapeLabel(Handle(AIS_InteractiveObject) object) const {
    TDF_Label label;
    if (myOcafDoc.IsNull() || object.IsNull()) {
        return label;
    }

    const Handle(CafShapePrs) aCafPresentation =
        Handle(CafShapePrs)::DownCast(object);
    if (!aCafPresentation.IsNull()) {
        if (!aCafPresentation->IsEditablePresentation()
            || aCafPresentation->GetLabel().IsNull()
            || aCafPresentation->GetLabel().Data()
                != myOcafDoc->GetData()) {
            return label;
        }
        return aCafPresentation->GetLabel();
    }

    Handle(AIS_Shape) aisShape = Handle(AIS_Shape)::DownCast(object);
    if (aisShape.IsNull() || aisShape->Shape().IsNull()) {
        return label;
    }

    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
    if (shapeTool.IsNull()) {
        return label;
    }
    shapeTool->FindShape(aisShape->Shape(), label)
        || shapeTool->FindShape(aisShape->Shape(), label, Standard_True);
    return label;
}

namespace {
bool TriangleAtlasFace(const TopoDS_Shape& shape, TopoDS_Face& face,
                       Handle(Poly_Triangulation)& mesh) {
    if (shape.ShapeType() != TopAbs_FACE) {
        if (shape.ShapeType() != TopAbs_COMPOUND) { return false; }
        TopoDS_Iterator children(shape);
        if (!children.More() || children.Value().ShapeType() != TopAbs_FACE) { return false; }
        children.Next(); if (children.More()) { return false; }
    }
    int faces = 0;
    for (TopExp_Explorer it(shape, TopAbs_FACE); it.More(); it.Next()) {
        face = TopoDS::Face(it.Current());
        if (++faces > 1) { return false; }
    }
    if (faces != 1 || !BRep_Tool::Surface(face).IsNull()) { return false; }
    TopLoc_Location location;
    mesh = BRep_Tool::Triangulation(face, location);
    return !mesh.IsNull() && !mesh->HasDeferredData() && mesh->HasGeometry()
        && mesh->NbTriangles() > 0 && mesh->NbTriangles() <= 4096
        && mesh->NbNodes() > 0 && mesh->NbNodes() <= 24576;
}

// A UV-only mutation may retain opaque partition bytes only when the old
// record validates against both exact ordered placed-local topology captures.
// Node identifiers and UV storage are deliberately outside that identity.
bool PreservesMeshRegionPartition(const OcctObjectTransformState& source,
                                  const TopoDS_Shape& candidate) noexcept {
    if (source.meshRegionPartition.empty()) return true;
    try {
        core3d::meshedit::RegionPartition partition;
        if (!core3d::meshedit::DecodeRegionPartition(
                source.meshRegionPartition.data(),
                source.meshRegionPartition.size(), partition)) return false;
        std::atomic_bool cancelled{false};
        core3d::meshedit::NativeTopologyCapture before, after;
        if (core3d::meshedit::CaptureNativeTopology(source.shape, before, cancelled)
                != core3d::meshedit::TopologyResult::Ready
            || core3d::meshedit::CaptureNativeTopology(candidate, after, cancelled)
                != core3d::meshedit::TopologyResult::Ready
            || source.shape.ShapeType() != candidate.ShapeType()
            || source.shape.Orientation() != candidate.Orientation()
            || !source.shape.Location().IsEqual(candidate.Location())
            || before.face.Orientation() != TopAbs_FORWARD
            || after.face.Orientation() != before.face.Orientation()
            || !after.face.Location().IsEqual(before.face.Location())
            || !after.meshLocation.IsEqual(before.meshLocation)) return false;
        return core3d::meshedit::ValidateRegionPartition(before, partition)
            && core3d::meshedit::ValidateRegionPartition(after, partition);
    } catch (...) { return false; }
}
}

Standard_Integer Core3DNormalTextureRecipeForLabel(const TDF_Label& label) noexcept
{
    try {
        if (label.IsNull()) return 0;
        Handle(TDF_Attribute) attribute;
        if (!label.FindAttribute(NormalTextureRecipeAttributeID(), attribute)) return 0;
        const auto value = Handle(TDataStd_Integer)::DownCast(attribute);
        return !value.IsNull() && (value->Get() == 1 || value->Get() == 2) ? value->Get() : -1;
    } catch (...) { return -1; }
}

OcctAuthoredFrameReadState Core3DReadAuthoredFrameOwner(
    const Handle(TDocStd_Document)& document, const TDF_Label& label,
    OcctAuthoredFrameRecord& record) noexcept {
    record = {};
    try {
        using namespace core3d::persistence;
        using namespace core3d::scene::authored;
        if (document.IsNull() || document->GetData().IsNull() || label.IsNull()
            || label.Data() != document->GetData()) return OcctAuthoredFrameReadState::Invalid;
        Handle(TDF_Attribute) attribute;
        if (!label.FindAttribute(AuthoredFrameAttributeID(), attribute)) return OcctAuthoredFrameReadState::Absent;
        const auto bytes = Handle(TDataStd_ByteArray)::DownCast(attribute);
        if (bytes.IsNull() || bytes->Lower() != 0 || bytes->Upper() < 127
            || bytes->Upper() >= Standard_Integer(kMaximumArchiveBytes) || bytes->GetDelta()
            || !XCAFDoc_DocumentTool::CheckShapeTool(document->Main())) return OcctAuthoredFrameReadState::Invalid;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (!IsGeometryDefinitionLabel(document, shapes, label) || !shapes->IsTopLevel(label)
            || !XCAFDoc_ShapeTool::IsFree(label)
            || ValidatedGeometryRepresentation(document, shapes, label) != OcctGeometryRepresentation::TriangleMesh)
            return OcctAuthoredFrameReadState::Invalid;
        double unit = 0;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document, unit) || unit != 0.001)
            return OcctAuthoredFrameReadState::Invalid;
        // Validate the placed definition above, then remove only its outer
        // placement. Child orientation/location remain part of the contract.
        auto local = XCAFDoc_ShapeTool::GetShape(label);
        local.Location(TopLoc_Location());
        TopoDS_Face face;
        if (local.ShapeType() == TopAbs_FACE) face = TopoDS::Face(local);
        else if (local.ShapeType() == TopAbs_COMPOUND) {
            TopoDS_Iterator children(local);
            if (!children.More() || children.Value().ShapeType() != TopAbs_FACE)
                return OcctAuthoredFrameReadState::Invalid;
            face = TopoDS::Face(children.Value());
            children.Next(); if (children.More()) return OcctAuthoredFrameReadState::Invalid;
        } else return OcctAuthoredFrameReadState::Invalid;
        OcctAuthoredFrameRecord result;
        result.archive.resize(Standard_Size(bytes->Upper()) + 1);
        for (Standard_Integer i = 0; i <= bytes->Upper(); ++i) result.archive[Standard_Size(i)] = bytes->Value(i);
        std::vector<core3d::scene::Float4> frames;
        if (!DecodeNativeAuthoredFrames(face, result.archive.data(), result.archive.size(), frames))
            return OcctAuthoredFrameReadState::Invalid;
        result.cornerCount = frames.size();
        result.nativeBytes = result.archive.size() + result.cornerCount * 64U;
        std::copy(result.archive.end() - kDigestBytes, result.archive.end(), result.identity.begin());
        record = std::move(result);
        return OcctAuthoredFrameReadState::Authored;
    } catch (...) { record = {}; return OcctAuthoredFrameReadState::Invalid; }
}

Standard_Boolean Core3DValidateAuthoredFrameOwners(
    const Handle(TDocStd_Document)& document, Standard_Size& nativeBytes,
    Standard_Size maximumBytes) noexcept {
    nativeBytes = 0;
    try {
        if (document.IsNull() || document->GetData().IsNull() || maximumBytes > 64U * 1024U * 1024U)
            return Standard_False;
        Standard_Size total = 0;
        auto validate = [&](const TDF_Label& label) {
            for (TDF_AttributeIterator it(label); it.More(); it.Next()) {
                const auto& attribute = it.Value();
                if (!Handle(TDataStd_ByteArray)::DownCast(attribute).IsNull()
                    && attribute->ID() != core3d::persistence::AuthoredFrameAttributeID()) return false;
            }
            OcctAuthoredFrameRecord record;
            if (Core3DReadAuthoredFrameOwner(document, label, record) == OcctAuthoredFrameReadState::Invalid)
                return false;
            return AddMultipliedWithinLimit(total, record.nativeBytes, 1U, maximumBytes);
        };
        const auto root = document->GetData()->Root();
        if (!validate(root)) return Standard_False;
        Standard_Size count = 0;
        for (TDF_ChildIterator it(root, Standard_True); it.More(); it.Next()) {
            if (++count > kMaximumGeometryDocumentLabels || !validate(it.Value())) return Standard_False;
        }
        nativeBytes = total; return Standard_True;
    } catch (...) { nativeBytes = 0; return Standard_False; }
}

namespace {
bool ValidateNormalTextureShape(const TopoDS_Shape& shape, Standard_Size* requiredNativeBytes) noexcept {
    if(requiredNativeBytes!=nullptr)*requiredNativeBytes=0;
    try {
        TopoDS_Face face; Handle(Poly_Triangulation) mesh;
        if (shape.IsNull() || !TriangleAtlasFace(shape, face, mesh)
            || !mesh->HasUVNodes() || !mesh->HasNormals()) return Standard_False;
        RWMesh_FaceIterator it(shape);
        if (!it.More() || !it.Face().IsSame(face) || !it.HasNormals() || !it.HasTexCoords()
            || it.NbNodes() != mesh->NbNodes() || it.NbTriangles() != mesh->NbTriangles()) return Standard_False;
        using namespace core3d::scene;
        std::vector<Vertex> vertices;
        std::vector<std::uint32_t> indices;
        vertices.reserve(it.NbNodes()); indices.reserve(3 * it.NbTriangles());
        // Use the same centered float geometry as immutable publication.
        gp_XYZ lower, upper;
        bool first = true;
        for (int n = it.NodeLower(); n <= it.NodeUpper(); ++n) {
            const gp_XYZ point = it.NodeTransformed(n).XYZ();
            if (!std::isfinite(point.X()) || !std::isfinite(point.Y()) || !std::isfinite(point.Z())) return Standard_False;
            if (first) { lower = upper = point; first = false; }
            else for (int c = 1; c <= 3; ++c) {
                lower.SetCoord(c, std::min(lower.Coord(c), point.Coord(c)));
                upper.SetCoord(c, std::max(upper.Coord(c), point.Coord(c)));
            }
        }
        const gp_XYZ origin = (lower + upper) * 0.5;
        for (int n = it.NodeLower(); n <= it.NodeUpper(); ++n) {
            const gp_XYZ point = it.NodeTransformed(n).XYZ() - origin;
            const gp_Dir normal = it.NormalTransformed(n);
            const gp_Pnt2d uv = it.NodeTexCoord(n);
            const double maximumFloat = std::numeric_limits<float>::max();
            for (const double value : {point.X(), point.Y(), point.Z(), uv.X(), uv.Y()})
                if (!std::isfinite(value) || std::abs(value) > maximumFloat) return Standard_False;
            vertices.push_back({float(point.X()), float(point.Y()), float(point.Z()),
                float(normal.X()), float(normal.Y()), float(normal.Z()), float(uv.X()), float(uv.Y())});
        }
        for (int t = it.ElemLower(); t <= it.ElemUpper(); ++t) {
            int nodes[3]; it.TriangleOriented(t).Get(nodes[0], nodes[1], nodes[2]);
            for (int n : nodes) {
                if (n < it.NodeLower() || n > it.NodeUpper()) return Standard_False;
                indices.push_back(static_cast<std::uint32_t>(n - it.NodeLower()));
            }
        }
        it.Next(); if (it.More()) return Standard_False;
        std::vector<Float4> frames;
        if (GenerateMikkCornerTangents(vertices, indices, true, frames) != TangentSpaceError::None) return Standard_False;
        if (requiredNativeBytes != nullptr) *requiredNativeBytes = indices.size() * 64;
        return Standard_True;
    } catch (...) { return Standard_False; }
}

} // namespace

Standard_Boolean Core3DValidateNormalTextureGeometry(
    const TDF_Label& label,Standard_Size* requiredNativeBytes) noexcept {
    if(requiredNativeBytes!=nullptr)*requiredNativeBytes=0;
    try {
        if(label.IsNull() || !XCAFDoc_ShapeTool::IsFree(label)
            || !XCAFDoc_ShapeTool::IsSimpleShape(label) || XCAFDoc_ShapeTool::IsReference(label))return Standard_False;
        return ValidateNormalTextureShape(XCAFDoc_ShapeTool::GetShape(label),requiredNativeBytes);
    } catch(...) {return Standard_False;}
}

Standard_Integer Core3DNormalTextureBasisForLabel(
    const Handle(TDocStd_Document)& document, const TDF_Label& label,
    Standard_Size* additionalNativeBytes) noexcept {
    if (additionalNativeBytes != nullptr) *additionalNativeBytes = 0;
    try {
        if (document.IsNull() || label.IsNull() || label.Data() != document->GetData()
            || !XCAFDoc_DocumentTool::CheckShapeTool(document->Main())) return 0;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (ValidatedGeometryRepresentation(document, shapes, label) != OcctGeometryRepresentation::TriangleMesh)
            return 0;
        OcctAuthoredFrameRecord record;
        const auto state = Core3DReadAuthoredFrameOwner(document, label, record);
        if (state == OcctAuthoredFrameReadState::Invalid) return 0;
        if (state == OcctAuthoredFrameReadState::Authored) return 2;
        return Core3DValidateNormalTextureGeometry(label, additionalNativeBytes) ? 1 : 0;
    } catch (...) { if (additionalNativeBytes != nullptr) *additionalNativeBytes = 0; return 0; }
}

Standard_Boolean Core3DValidateNormalTextureBinding(
    const Handle(TDocStd_Document)& document, const TDF_Label& label,
    Standard_Size* additionalNativeBytes) noexcept {
    if (additionalNativeBytes != nullptr) *additionalNativeBytes = 0;
    Standard_Size bytes = 0;
    const auto basis = Core3DNormalTextureBasisForLabel(document, label, &bytes);
    if (basis == 0 || Core3DNormalTextureRecipeForLabel(label) != basis) return Standard_False;
    if (additionalNativeBytes != nullptr) *additionalNativeBytes = bytes;
    return Standard_True;
}

Standard_Boolean Core3DValidateOwnedFrameUsage(
    const Handle(TDocStd_Document)& document, Standard_Size& nativeBytes,
    Standard_Size maximumBytes) noexcept {
    nativeBytes = 0;
    try {
        Standard_Size total = 0;
        if (!Core3DValidateAuthoredFrameOwners(document,total,maximumBytes)) return Standard_False;
        auto validate = [&](const TDF_Label& label) {
            const auto recipe = Core3DNormalTextureRecipeForLabel(label);
            if (recipe < 0) return false;
            const auto material = XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
            const bool normal = !material.IsNull() && material->HasPbrMaterial()
                && !material->PbrMaterial().NormalTexture.IsNull();
            if (!normal) return recipe == 0;
            Handle(TDataStd_Integer) marker;
            const bool owned = label.FindAttribute(LocalPBRMaterialAttributeID(),marker)
                && !marker.IsNull() && marker->Get() == 1;
            OcctAuthoredFrameRecord frame;
            const auto state = Core3DReadAuthoredFrameOwner(document,label,frame);
            if (state == OcctAuthoredFrameReadState::Invalid) return false;
            // Preserve legacy imported materials without geometry-owned frames.
            // Supplied normal bindings must have explicit native recipe ownership.
            if (!owned && state == OcctAuthoredFrameReadState::Absent) return recipe == 0;
            Standard_Size additional = 0;
            return owned && Core3DValidateNormalTextureBinding(document,label,&additional)
                && AddMultipliedWithinLimit(total,additional,1U,maximumBytes);
        };
        const auto root=document->GetData()->Root();
        if (!validate(root)) return Standard_False;
        Standard_Size count=0;
        for (TDF_ChildIterator it(root,Standard_True);it.More();it.Next()) {
            if (++count>kMaximumGeometryDocumentLabels || !validate(it.Value())) return Standard_False;
        }
        nativeBytes=total;return Standard_True;
    } catch (...) { nativeBytes=0;return Standard_False; }
}

Standard_Boolean OcctDocument::SupportsNormalTextureGeometryForLabel(
    const TDF_Label& label) const noexcept {
    return [NSThread isMainThread] && HasNativeNormalTextureGeometry(label);
}

Standard_Boolean OcctDocument::HasNativeNormalTextureGeometry(
    const TDF_Label& label) const noexcept {
    try {
        return IsEditableFreeSimpleDefinitionLabel(label)
            && GeometryRepresentationForLabel(label) == OcctGeometryRepresentation::TriangleMesh
            && Core3DNormalTextureBasisForLabel(myOcafDoc, label) != 0;
    } catch (...) { return Standard_False; }
}

namespace {
// Copy policy compares local stored payload, not positions rounded for rendering.
// Strip only the outer placement; preserve child placement/orientation and all
// unused nodes. UV-only sources may have no normals and cannot use SYTG hashing.
bool SameStoredMeshCopyPayload(TopoDS_Shape source, TopoDS_Shape destination) {
    source.Location(TopLoc_Location()); destination.Location(TopLoc_Location());
    TopoDS_Face a, b; Handle(Poly_Triangulation) x, y;
    if (source.ShapeType() != destination.ShapeType()
        || source.Orientation() != destination.Orientation()
        || !TriangleAtlasFace(source, a, x) || !TriangleAtlasFace(destination, b, y)
        || a.Orientation() != b.Orientation() || !a.Location().IsEqual(b.Location())
        || x->NbNodes() != y->NbNodes() || x->NbTriangles() != y->NbTriangles()
        || x->HasNormals() != y->HasNormals() || x->HasUVNodes() != y->HasUVNodes()) return false;
    const auto same = [](const auto left, const auto right) {
        return std::memcmp(&left, &right, sizeof(left)) == 0;
    };
    for (int n = 1; n <= x->NbNodes(); ++n) {
        const auto p = x->Node(n), q = y->Node(n);
        for (int c = 1; c <= 3; ++c) if (!same(p.Coord(c), q.Coord(c))) return false;
        if (x->HasUVNodes()) {
            const auto u = x->UVNode(n), v = y->UVNode(n);
            if (!same(u.X(), v.X()) || !same(u.Y(), v.Y())) return false;
        }
        if (x->HasNormals()) {
            gp_Vec3f u, v; x->Normal(n, u); y->Normal(n, v);
            for (int c = 0; c < 3; ++c) if (!same(u[c], v[c])) return false;
        }
    }
    for (int t = 1; t <= x->NbTriangles(); ++t) {
        int u[3], v[3]; x->Triangle(t).Get(u[0], u[1], u[2]); y->Triangle(t).Get(v[0], v[1], v[2]);
        for (int c = 0; c < 3; ++c) if (u[c] != v[c]) return false;
    }
    return true;
}
}

Standard_Boolean OcctDocument::CaptureMeshRegionPartition(
    const TDF_Label& label, std::vector<Standard_Byte>& encoded) const noexcept {
    encoded.clear();
    if (![NSThread isMainThread]) return Standard_False;
    try {
        OCC_CATCH_SIGNALS
        return !myOcafDoc.IsNull() && IsEditableFreeSimpleDefinitionLabel(label)
            && ReadMeshRegionPartitionRecord(myOcafDoc, label, encoded)
                != MeshRegionPartitionReadState::Malformed;
    } catch (...) { encoded.clear(); return Standard_False; }
}

Standard_Boolean OcctDocument::StageMeshRegionPartition(
    const TDF_Label& label, const std::vector<Standard_Byte>& encoded) noexcept {
    if (![NSThread isMainThread]) return Standard_False;
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand() || encoded.empty()
            || encoded.size() > 1580 || !ValidateGeometryRepresentations()
            || !IsEditableFreeSimpleDefinitionLabel(label)
            || GeometryRepresentationForLabel(label) != OcctGeometryRepresentation::TriangleMesh)
            return Standard_False;
        core3d::meshedit::RegionPartition partition;
        std::atomic_bool cancelled{false}; core3d::meshedit::NativeTopologyCapture source;
        if (!core3d::meshedit::DecodeRegionPartition(encoded.data(), encoded.size(), partition)
            || core3d::meshedit::CaptureNativeTopology(XCAFDoc_ShapeTool::GetShape(label), source, cancelled)
                != core3d::meshedit::TopologyResult::Ready
            || !core3d::meshedit::ValidateRegionPartition(source, partition)) return Standard_False;
        std::vector<Standard_Byte> previous; TDF_Label record;
        const auto state = ReadMeshRegionPartitionRecord(myOcafDoc, label, previous, &record);
        if (state == MeshRegionPartitionReadState::Malformed) return Standard_False;
        if (state == MeshRegionPartitionReadState::Valid && previous == encoded) return Standard_True;
        if (record.IsNull()) {
            // Tags 1...8 are persisted transforms and 11/12 are legacy
            // material/color even when their attributes were cleared. Never
            // claim an attribute-free historical label from that namespace.
            Standard_Integer tag = 12;
            for (TDF_ChildIterator child(label, Standard_False); child.More(); child.Next())
                tag = std::max(tag, child.Value().Tag());
            if (tag == std::numeric_limits<Standard_Integer>::max()) return Standard_False;
            record = label.FindChild(tag + 1, Standard_True);
        } else record.ForgetAllAttributes(Standard_True);
        const std::string hex = LowerHex(encoded.data(), encoded.size());
        const Standard_Size chunkCount = (hex.size() + kMeshRegionPartitionChunkCharacters - 1)
            / kMeshRegionPartitionChunkCharacters;
        if (chunkCount == 0 || chunkCount > kMaximumMeshRegionPartitionChunks) return Standard_False;
        TDataStd_UAttribute::Set(record, MeshRegionPartitionRecordID());
        TDataStd_Integer::Set(record, MeshRegionPartitionVersionID(), 1);
        TDataStd_Integer::Set(record, MeshRegionPartitionTriangleCountID(),
            Standard_Integer(partition.barriers.size()));
        TDataStd_Integer::Set(record, MeshRegionPartitionChunkCountID(), Standard_Integer(chunkCount));
        TDataStd_AsciiString::Set(record, MeshRegionPartitionDigestID(),
            TCollection_AsciiString(LowerHex(encoded.data() + 12, 32).c_str()));
        for (Standard_Size index = 0; index < chunkCount; ++index) {
            const std::string chunk = hex.substr(index * kMeshRegionPartitionChunkCharacters,
                kMeshRegionPartitionChunkCharacters);
            TDataStd_AsciiString::Set(record.FindChild(Standard_Integer(index + 1), Standard_True),
                TCollection_AsciiString(chunk.c_str()));
        }
        std::vector<Standard_Byte> readback;
        return ReadMeshRegionPartitionRecord(myOcafDoc, label, readback)
                    == MeshRegionPartitionReadState::Valid
            && readback == encoded && ValidateGeometryRepresentations();
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctDocument::ClearMeshRegionPartition(const TDF_Label& label) noexcept {
    if (![NSThread isMainThread]) return Standard_False;
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || !ValidateGeometryRepresentations()
            || !IsEditableFreeSimpleDefinitionLabel(label)) return Standard_False;
        std::vector<Standard_Byte> previous; TDF_Label record;
        const auto state = ReadMeshRegionPartitionRecord(myOcafDoc, label, previous, &record);
        if (state == MeshRegionPartitionReadState::Malformed) return Standard_False;
        if (state == MeshRegionPartitionReadState::Absent) return Standard_True;
        record.ForgetAllAttributes(Standard_True);
        previous.clear();
        return ReadMeshRegionPartitionRecord(myOcafDoc, label, previous)
                == MeshRegionPartitionReadState::Absent
            && ValidateGeometryRepresentations();
    } catch (...) { return Standard_False; }
}

#if DEBUG
Standard_Boolean OcctDocument::DebugStageFirstMeshRegionPartition(
    const TDF_Label& label) noexcept {
    try {
        OCC_CATCH_SIGNALS
        if (![NSThread isMainThread] || myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand())
            return Standard_False;
        std::atomic_bool cancelled{false}; core3d::meshedit::NativeTopologyCapture source;
        if (core3d::meshedit::CaptureNativeTopology(XCAFDoc_ShapeTool::GetShape(label), source, cancelled)
            != core3d::meshedit::TopologyResult::Ready) return Standard_False;
        std::set<std::array<core3d::meshedit::Point, 2>> requested;
        for (const auto& edge : source.topology.edges) {
            if (edge.uses.size() != 2) continue;
            std::array<core3d::meshedit::Point, 2> points = {
                source.topology.vertices[edge.vertices[0]].point,
                source.topology.vertices[edge.vertices[1]].point};
            if (points[1] < points[0]) std::swap(points[0], points[1]);
            requested.insert(points); break;
        }
        core3d::meshedit::RegionPartition partition; std::vector<std::uint8_t> bytes;
        if (requested.size() != 1 || !core3d::meshedit::MakeRegionPartition(source, requested, partition)
            || !core3d::meshedit::EncodeRegionPartition(partition, bytes)) return Standard_False;
        return StageMeshRegionPartition(label,
            std::vector<Standard_Byte>(bytes.begin(), bytes.end()));
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctDocument::DebugCorruptMeshRegionPartition(
    const TDF_Label& label, Standard_Integer mode) noexcept {
    if (![NSThread isMainThread] || mode < 0 || mode > 9 || myOcafDoc.IsNull()
        || !myOcafDoc->HasOpenCommand()) return Standard_False;
    try {
        OCC_CATCH_SIGNALS
        std::vector<Standard_Byte> encoded; TDF_Label record;
        if (ReadMeshRegionPartitionRecord(myOcafDoc, label, encoded, &record)
                != MeshRegionPartitionReadState::Valid || record.IsNull()) return Standard_False;
        switch (mode) {
            case 0: TDataStd_Integer::Set(record, MeshRegionPartitionVersionID(), 2); break;
            case 1: TDataStd_Integer::Set(record, MeshRegionPartitionTriangleCountID(), 4097); break;
            case 2: record.FindChild(1, Standard_False).ForgetAllAttributes(Standard_True); break;
            case 3: TDataStd_AsciiString::Set(record.FindChild(14, Standard_True), TCollection_AsciiString("00")); break;
            case 4: {
                Handle(TDataStd_AsciiString) chunk;
                if (!record.FindChild(1, Standard_False).FindAttribute(TDataStd_AsciiString::GetID(), chunk)
                    || chunk.IsNull()) return Standard_False;
                std::string value = chunk->Get().ToCString(); value[0] = 'G';
                TDataStd_AsciiString::Set(record.FindChild(1, Standard_False), TCollection_AsciiString(value.c_str()));
                break;
            }
            case 5: TDataStd_AsciiString::Set(record, MeshRegionPartitionDigestID(),
                TCollection_AsciiString(std::string(64, '0').c_str())); break;
            case 6: record.ForgetAttribute(MeshRegionPartitionRecordID()); break;
            case 7:
                record.ForgetAttribute(MeshRegionPartitionRecordID());
                TDataStd_Integer::Set(record, MeshRegionPartitionRecordID(), 1);
                break;
            case 8: TDataStd_Integer::Set(label, MeshRegionPartitionVersionID(), 1); break;
            case 9: TDataStd_UAttribute::Set(label, MeshRegionPartitionRecordID()); break;
        }
        std::vector<Standard_Byte> refused;
        return ReadMeshRegionPartitionRecord(myOcafDoc, label, refused)
                == MeshRegionPartitionReadState::Malformed
            && refused.empty();
    } catch (...) { return Standard_False; }
}
#endif

// Definition-owned source-face provenance schema v1 for source-retained mesh
// copies. Only attribute types already admitted by the narrow document reader
// are used (UAttribute marker, Integer scalars, chunked AsciiString payload
// and digest), so shipped builds keep opening these documents; no new driver.
// These GUIDs are permanent serialized schema identifiers: never reuse them.
const Standard_GUID& CopySourceFaceProvenanceRecordID() {
    static const Standard_GUID id("4F7D2C1A-9B3E-4A5F-8C6D-7E8F9A0B1C01"); return id;
}
const Standard_GUID& CopySourceFaceProvenanceVersionID() {
    static const Standard_GUID id("4F7D2C1A-9B3E-4A5F-8C6D-7E8F9A0B1C02"); return id;
}
const Standard_GUID& CopySourceFaceProvenanceFaceCountID() {
    static const Standard_GUID id("4F7D2C1A-9B3E-4A5F-8C6D-7E8F9A0B1C03"); return id;
}
const Standard_GUID& CopySourceFaceProvenanceTriangleCountID() {
    static const Standard_GUID id("4F7D2C1A-9B3E-4A5F-8C6D-7E8F9A0B1C04"); return id;
}
const Standard_GUID& CopySourceFaceProvenanceChunkCountID() {
    static const Standard_GUID id("4F7D2C1A-9B3E-4A5F-8C6D-7E8F9A0B1C05"); return id;
}
const Standard_GUID& CopySourceFaceProvenanceDigestID() {
    static const Standard_GUID id("4F7D2C1A-9B3E-4A5F-8C6D-7E8F9A0B1C06"); return id;
}
constexpr Standard_Size kCopySourceFaceProvenanceChunkCharacters = 256;
constexpr Standard_Size kMaximumCopySourceFaceProvenanceChunks = 320;

bool HasCopySourceFaceProvenanceSchemaAttribute(const TDF_Label& label) {
    Handle(TDF_Attribute) value;
    return label.FindAttribute(CopySourceFaceProvenanceRecordID(), value)
        || label.FindAttribute(CopySourceFaceProvenanceVersionID(), value)
        || label.FindAttribute(CopySourceFaceProvenanceFaceCountID(), value)
        || label.FindAttribute(CopySourceFaceProvenanceTriangleCountID(), value)
        || label.FindAttribute(CopySourceFaceProvenanceChunkCountID(), value)
        || label.FindAttribute(CopySourceFaceProvenanceDigestID(), value);
}

// Strict reader. Absent: no record. Malformed: any schema violation (unknown
// or missing attributes, bad counts, bad digest text, unparseable payload, or
// non-contiguous ranges). Stale: structurally valid record whose stored
// digest no longer matches the current triangulation, or whose definition no
// longer carries a readable mesh-only face. Present: exact match.
core3d::provenance::CopySourceFaceProvenanceReadState ReadCopySourceFaceProvenanceRecord(
    const Handle(TDocStd_Document)& document, const TDF_Label& definition,
    core3d::provenance::SourceFaceProvenanceRecord& output, TDF_Label* recordLabel = nullptr) noexcept {
    using core3d::provenance::CopySourceFaceProvenanceReadState;
    output = core3d::provenance::SourceFaceProvenanceRecord();
    if (recordLabel) *recordLabel = TDF_Label();
    try {
        OCC_CATCH_SIGNALS
        if (document.IsNull() || document->GetData().IsNull() || definition.IsNull()
            || definition.Data() != document->GetData()
            || HasCopySourceFaceProvenanceSchemaAttribute(definition))
            return CopySourceFaceProvenanceReadState::Malformed;
        TDF_Label record;
        // A definition owns at most one direct record. Schema fragments below
        // any other descendant are malformed rather than silently absent.
        Standard_Size descendants = 0;
        for (TDF_ChildIterator child(definition, Standard_True); child.More(); child.Next()) {
            if (++descendants > kMaximumGeometryDocumentLabels)
                return CopySourceFaceProvenanceReadState::Malformed;
            if (!HasCopySourceFaceProvenanceSchemaAttribute(child.Value())) continue;
            Handle(TDF_Attribute) marker;
            if (!child.Value().Father().IsEqual(definition)
                || !child.Value().FindAttribute(CopySourceFaceProvenanceRecordID(), marker)
                || Handle(TDataStd_UAttribute)::DownCast(marker).IsNull()
                || !record.IsNull()) return CopySourceFaceProvenanceReadState::Malformed;
            record = child.Value();
        }
        if (record.IsNull()) return CopySourceFaceProvenanceReadState::Absent;
        Handle(TDataStd_Integer) version, faceCount, triangles, chunks;
        Handle(TDataStd_AsciiString) digest;
        if (!record.FindAttribute(CopySourceFaceProvenanceVersionID(), version)
            || !record.FindAttribute(CopySourceFaceProvenanceFaceCountID(), faceCount)
            || !record.FindAttribute(CopySourceFaceProvenanceTriangleCountID(), triangles)
            || !record.FindAttribute(CopySourceFaceProvenanceChunkCountID(), chunks)
            || !record.FindAttribute(CopySourceFaceProvenanceDigestID(), digest)
            || version.IsNull() || faceCount.IsNull() || triangles.IsNull() || chunks.IsNull() || digest.IsNull()
            || version->Get() != 1
            || faceCount->Get() <= 0 || faceCount->Get() > core3d::provenance::kMaximumProvenanceFaces
            || triangles->Get() <= 0 || triangles->Get() > core3d::provenance::kMaximumProvenanceTriangles
            || chunks->Get() <= 0 || chunks->Get() > Standard_Integer(kMaximumCopySourceFaceProvenanceChunks))
            return CopySourceFaceProvenanceReadState::Malformed;
        Standard_Size recordAttributes = 0;
        for (TDF_AttributeIterator attribute(record); attribute.More(); attribute.Next()) {
            const auto& id = attribute.Value()->ID();
            if (id != CopySourceFaceProvenanceRecordID() && id != CopySourceFaceProvenanceVersionID()
                && id != CopySourceFaceProvenanceFaceCountID() && id != CopySourceFaceProvenanceTriangleCountID()
                && id != CopySourceFaceProvenanceChunkCountID() && id != CopySourceFaceProvenanceDigestID())
                return CopySourceFaceProvenanceReadState::Malformed;
            ++recordAttributes;
        }
        const std::string digestText = digest->Get().ToCString();
        if (recordAttributes != 6 || digestText.size() != 64)
            return CopySourceFaceProvenanceReadState::Malformed;
        for (char c : digestText) if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')))
            return CopySourceFaceProvenanceReadState::Malformed;
        std::string payload;
        for (Standard_Integer index = 1; index <= chunks->Get(); ++index) {
            const TDF_Label chunk = record.FindChild(index, Standard_False);
            Handle(TDataStd_AsciiString) text;
            Standard_Size attributes = 0;
            Standard_Size nested = 0;
            if (chunk.IsNull() || HasLiveMeshRegionPartitionDescendant(chunk, nested)
                || !chunk.FindAttribute(TDataStd_AsciiString::GetID(), text) || text.IsNull())
                return CopySourceFaceProvenanceReadState::Malformed;
            for (TDF_AttributeIterator attribute(chunk); attribute.More(); attribute.Next()) {
                if (attribute.Value()->ID() != TDataStd_AsciiString::GetID())
                    return CopySourceFaceProvenanceReadState::Malformed;
                ++attributes;
            }
            const std::string value = text->Get().ToCString();
            if (attributes != 1 || value.empty() || value.size() > kCopySourceFaceProvenanceChunkCharacters
                || (index < chunks->Get() && value.size() != kCopySourceFaceProvenanceChunkCharacters))
                return CopySourceFaceProvenanceReadState::Malformed;
            for (char c : value) if (!core3d::provenance::IsProvenancePayloadCharacter(c))
                return CopySourceFaceProvenanceReadState::Malformed;
            payload += value;
        }
        Standard_Size directChildren = 0;
        for (TDF_ChildIterator child(record, Standard_False); child.More(); child.Next()) {
            if (++directChildren > kMaximumGeometryDocumentLabels)
                return CopySourceFaceProvenanceReadState::Malformed;
            Standard_Size nested = 0;
            if (child.Value().Tag() > chunks->Get()
                && (child.Value().HasAttribute()
                    || HasLiveMeshRegionPartitionDescendant(child.Value(), nested)))
                return CopySourceFaceProvenanceReadState::Malformed;
        }
        std::vector<core3d::provenance::SourceFaceProvenanceEntry> entries;
        if (!core3d::provenance::DecodeEntries(payload, entries)
            || entries.size() != Standard_Size(faceCount->Get()))
            return CopySourceFaceProvenanceReadState::Malformed;
        // Ranges are contiguous in the pre-atlas emission order and cover
        // exactly the staged triangle count.
        int expected = 0;
        for (const auto& entry : entries) {
            if (entry.firstTriangle != expected) return CopySourceFaceProvenanceReadState::Malformed;
            expected += entry.triangleCount;
        }
        if (expected != triangles->Get()) return CopySourceFaceProvenanceReadState::Malformed;
        output.digest = digestText;
        output.triangleCount = triangles->Get();
        output.faces = entries;
        if (recordLabel) *recordLabel = record;
        // A structurally valid record is only Present while the copy's current
        // triangulation still matches the staged digest. Any geometry mutation
        // (region extrude/inset, vertex move) or triangle reordering reads
        // Stale; an order-preserving atlas rebuild keeps the digest exact.
        TopoDS_Face face; Handle(Poly_Triangulation) mesh;
        if (!TriangleAtlasFace(XCAFDoc_ShapeTool::GetShape(definition), face, mesh)
            || mesh->NbTriangles() != triangles->Get())
            return CopySourceFaceProvenanceReadState::Stale;
        std::string actual;
        if (!core3d::provenance::CopyTriangulationDigest(mesh, actual))
            return CopySourceFaceProvenanceReadState::Stale;
        return actual == digestText
            ? CopySourceFaceProvenanceReadState::Present : CopySourceFaceProvenanceReadState::Stale;
    } catch (...) {
        output = core3d::provenance::SourceFaceProvenanceRecord();
        if (recordLabel) *recordLabel = TDF_Label();
        return CopySourceFaceProvenanceReadState::Malformed;
    }
}

// Write the schema attributes and payload chunks onto a fresh or cleared
// record label. The caller owns validation, command scope and readback.
bool WriteCopySourceFaceProvenanceChunks(const TDF_Label& record, const std::string& payload,
    Standard_Integer faceCount, Standard_Integer triangleCount, const std::string& digest) {
    const Standard_Size chunkCount = (payload.size() + kCopySourceFaceProvenanceChunkCharacters - 1)
        / kCopySourceFaceProvenanceChunkCharacters;
    if (payload.empty() || chunkCount == 0 || chunkCount > kMaximumCopySourceFaceProvenanceChunks
        || digest.size() != 64) return false;
    TDataStd_UAttribute::Set(record, CopySourceFaceProvenanceRecordID());
    TDataStd_Integer::Set(record, CopySourceFaceProvenanceVersionID(), 1);
    TDataStd_Integer::Set(record, CopySourceFaceProvenanceFaceCountID(), faceCount);
    TDataStd_Integer::Set(record, CopySourceFaceProvenanceTriangleCountID(), triangleCount);
    TDataStd_Integer::Set(record, CopySourceFaceProvenanceChunkCountID(), Standard_Integer(chunkCount));
    TDataStd_AsciiString::Set(record, CopySourceFaceProvenanceDigestID(),
        TCollection_AsciiString(digest.c_str()));
    for (Standard_Size index = 0; index < chunkCount; ++index) {
        const std::string chunk = payload.substr(index * kCopySourceFaceProvenanceChunkCharacters,
            kCopySourceFaceProvenanceChunkCharacters);
        TDataStd_AsciiString::Set(record.FindChild(Standard_Integer(index + 1), Standard_True),
            TCollection_AsciiString(chunk.c_str()));
    }
    return true;
}

// Allocate the record child label. Tags 1...8 are persisted transforms and
// 11/12 are legacy material/color even when their attributes were cleared;
// never claim an attribute-free historical label from that namespace.
TDF_Label FreshCopySourceFaceProvenanceLabel(const TDF_Label& definition, bool& overflow) {
    overflow = false;
    Standard_Integer tag = 12;
    for (TDF_ChildIterator child(definition, Standard_False); child.More(); child.Next())
        tag = std::max(tag, child.Value().Tag());
    if (tag == std::numeric_limits<Standard_Integer>::max()) { overflow = true; return TDF_Label(); }
    return definition.FindChild(tag + 1, Standard_True);
}

core3d::provenance::CopySourceFaceProvenanceReadState
OcctDocument::TryCopySourceFaceProvenanceForLabel(
    const TDF_Label& label, core3d::provenance::SourceFaceProvenanceRecord& record) const noexcept {
    using core3d::provenance::CopySourceFaceProvenanceReadState;
    record = core3d::provenance::SourceFaceProvenanceRecord();
    if (![NSThread isMainThread]) return CopySourceFaceProvenanceReadState::Malformed;
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()) return CopySourceFaceProvenanceReadState::Malformed;
        return ReadCopySourceFaceProvenanceRecord(myOcafDoc, label, record);
    } catch (...) {
        record = core3d::provenance::SourceFaceProvenanceRecord();
        return CopySourceFaceProvenanceReadState::Malformed;
    }
}

Standard_Boolean OcctDocument::StageCopySourceFaceProvenance(
    const TDF_Label& label, const std::vector<core3d::meshcopy::SourceFaceRecord>& faces) noexcept {
    using core3d::provenance::CopySourceFaceProvenanceReadState;
    if (![NSThread isMainThread]) return Standard_False;
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand() || faces.empty()
            || faces.size() > std::size_t(core3d::provenance::kMaximumProvenanceFaces)
            || !ValidateGeometryRepresentations()
            || !IsEditableFreeSimpleDefinitionLabel(label)
            || GeometryRepresentationForLabel(label) != OcctGeometryRepresentation::TriangleMesh)
            return Standard_False;
        std::vector<core3d::provenance::SourceFaceProvenanceEntry> entries;
        if (!core3d::provenance::EntriesFromSourceFaces(faces, entries)) return Standard_False;
        int triangles = 0;
        for (const auto& entry : entries) {
            if (entry.firstTriangle != triangles || entry.triangleCount <= 0) return Standard_False;
            triangles += entry.triangleCount;
        }
        if (triangles <= 0 || triangles > core3d::provenance::kMaximumProvenanceTriangles)
            return Standard_False;
        TopoDS_Face face; Handle(Poly_Triangulation) mesh;
        if (!TriangleAtlasFace(XCAFDoc_ShapeTool::GetShape(label), face, mesh)
            || mesh->NbTriangles() != triangles) return Standard_False;
        std::string digest, payload;
        if (!core3d::provenance::CopyTriangulationDigest(mesh, digest)
            || !core3d::provenance::EncodeEntries(entries, payload)) return Standard_False;
        core3d::provenance::SourceFaceProvenanceRecord previous; TDF_Label record;
        const auto state = ReadCopySourceFaceProvenanceRecord(myOcafDoc, label, previous, &record);
        if (state == CopySourceFaceProvenanceReadState::Malformed) return Standard_False;
        if (state != CopySourceFaceProvenanceReadState::Absent
            && previous.digest == digest
            && core3d::provenance::SameEntries(previous.faces, entries)) return Standard_True;
        if (record.IsNull()) {
            bool overflow = false;
            record = FreshCopySourceFaceProvenanceLabel(label, overflow);
            if (overflow || record.IsNull()) return Standard_False;
        } else record.ForgetAllAttributes(Standard_True);
        if (!WriteCopySourceFaceProvenanceChunks(record, payload,
                Standard_Integer(entries.size()), triangles, digest)) return Standard_False;
        core3d::provenance::SourceFaceProvenanceRecord readback;
        return TryCopySourceFaceProvenanceForLabel(label, readback)
                    == CopySourceFaceProvenanceReadState::Present
            && readback.digest == digest
            && core3d::provenance::SameEntries(readback.faces, entries)
            && ValidateGeometryRepresentations();
    } catch (...) { return Standard_False; }
}

#if DEBUG
Standard_Integer OcctDocument::DebugCorruptCopySourceFaceProvenance(
    const TDF_Label& label, Standard_Integer& corruptedState) noexcept {
    using core3d::provenance::CopySourceFaceProvenanceReadState;
    corruptedState = -1;
    if (![NSThread isMainThread]) return 1;
    if (myOcafDoc.IsNull()) return 2;
    try {
        OCC_CATCH_SIGNALS
        core3d::provenance::SourceFaceProvenanceRecord record; TDF_Label recordLabel;
        if (ReadCopySourceFaceProvenanceRecord(myOcafDoc, label, record, &recordLabel)
                != CopySourceFaceProvenanceReadState::Present) return 3;
        if (recordLabel.IsNull()) return 4;
        // Mutate the existing custom-GUID attribute that the strict reader
        // validates on the record child, then restore it without a transaction.
        Handle(TDataStd_AsciiString) digest;
        if (!recordLabel.FindAttribute(CopySourceFaceProvenanceDigestID(), digest)
            || digest.IsNull()) return 5;
        const TCollection_AsciiString originalDigest = digest->Get();
        auto restore = [&]() noexcept -> bool {
            try {
                digest->Set(originalDigest);
                core3d::provenance::SourceFaceProvenanceRecord restored;
                return ReadCopySourceFaceProvenanceRecord(myOcafDoc, label, restored)
                        == CopySourceFaceProvenanceReadState::Present
                    && restored.digest == originalDigest.ToCString();
            } catch (...) { return false; }
        };
        Standard_Integer code = 8;
        try {
            // Wrong digest length is a schema violation, never a stale read.
            digest->Set(TCollection_AsciiString("00"));
            core3d::provenance::SourceFaceProvenanceRecord refused;
            const auto state = ReadCopySourceFaceProvenanceRecord(myOcafDoc, label, refused);
            corruptedState = Standard_Integer(state);
            code = state == CopySourceFaceProvenanceReadState::Malformed && refused.digest.empty()
                ? 0 : 60 + corruptedState;
        } catch (...) { code = 8; }
        // Also restore after a failed mutation/read; restoration failure wins
        // over the earlier diagnostic because the snapshot may be corrupted.
        return restore() ? code : 7;
    } catch (...) { return 8; }
}
#endif

Standard_Boolean OcctDocument::CopyGeometryOwnedMeshMetadata(
    const TDF_Label& source, const TDF_Label& destination) {
    if (![NSThread isMainThread]) return Standard_False;
    try {
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || source.IsNull() || destination.IsNull() || source.IsEqual(destination)) return Standard_False;
        OcctObjectTransformState before, target;
        if (!CaptureObjectTransformStateForLabel(source, before)
            || !CaptureObjectTransformStateForLabel(destination, target)
            || before.resolvedRepresentation != target.resolvedRepresentation) return Standard_False;
        OcctAuthoredFrameRecord frame, previousFrame;
        if (Core3DReadAuthoredFrameOwner(myOcafDoc, source, frame) == OcctAuthoredFrameReadState::Invalid
            || Core3DReadAuthoredFrameOwner(myOcafDoc, destination, previousFrame) == OcctAuthoredFrameReadState::Invalid)
            return Standard_False;
        using core3d::provenance::CopySourceFaceProvenanceReadState;
        core3d::provenance::SourceFaceProvenanceRecord sourceProvenance, targetProvenance;
        TDF_Label targetProvenanceLabel;
        const auto sourceProvenanceState =
            ReadCopySourceFaceProvenanceRecord(myOcafDoc, source, sourceProvenance);
        const auto targetProvenanceState =
            ReadCopySourceFaceProvenanceRecord(myOcafDoc, destination, targetProvenance, &targetProvenanceLabel);
        if (sourceProvenanceState == CopySourceFaceProvenanceReadState::Malformed
            || targetProvenanceState == CopySourceFaceProvenanceReadState::Malformed) return Standard_False;
        if (before.meshUVAtlasVersion == 0 && target.meshUVAtlasVersion == 0
            && frame.archive.empty() && previousFrame.archive.empty()
            && before.meshRegionPartition.empty() && target.meshRegionPartition.empty()
            && sourceProvenanceState == CopySourceFaceProvenanceReadState::Absent
            && targetProvenanceState == CopySourceFaceProvenanceReadState::Absent) return Standard_True;
        // Clearing a destination's metadata is still an exact-copy operation.
        // An unannotated but different source cannot erase another mesh's basis.
        if (!SameStoredMeshCopyPayload(before.shape, target.shape)) return Standard_False;
        if (before.meshUVAtlasVersion != 0 || !frame.archive.empty()) {
            if (before.meshUVAtlasVersion != 0) {
                TopoDS_Face face; Handle(Poly_Triangulation) mesh;
                if (!TriangleAtlasFace(before.shape, face, mesh) || !mesh->HasUVNodes()) return Standard_False;
                const int originals = before.meshUVAtlasVersion == 3 ? 0
                    : mesh->NbNodes() - 3 * mesh->NbTriangles();
                if ((before.meshUVAtlasVersion == 3
                        ? mesh->NbNodes() != 3 * mesh->NbTriangles()
                        : originals <= 0 || originals > 12288)
                    || (before.meshUVAtlasVersion == 2 && before.meshUVAtlasSettings[2] != originals)
                    || (before.meshUVAtlasVersion == 3 && !mesh->HasNormals())) return Standard_False;
                for (int t = 1; t <= mesh->NbTriangles(); ++t) {
                    int ids[3]; mesh->Triangle(t).Get(ids[0], ids[1], ids[2]);
                    for (int c = 0; c < 3; ++c) if (ids[c] != originals + (t - 1) * 3 + c + 1) return Standard_False;
                }
                if (before.meshUVAtlasVersion == 2) {
                    OcctMeshUVAtlasPreview preview;
                    if (!CaptureMeshUVAtlasPreview(source, preview)) return Standard_False;
                }
            }
        }
        // Charge geometry-owned records even when hidden or mapless. Project
        // the replacement owner before writes without changing a bound recipe.
        if (!frame.archive.empty() || !previousFrame.archive.empty()) {
            Standard_Size residentBytes = 0;
            if (!Core3DValidateAuthoredFrameOwners(myOcafDoc, residentBytes)
                || previousFrame.nativeBytes > residentBytes) return Standard_False;
            residentBytes -= previousFrame.nativeBytes;
            if (!AddMultipliedWithinLimit(residentBytes, frame.nativeBytes, 1U, 64U * 1024U * 1024U)) return Standard_False;
            TDF_LabelSequence roots;
            const auto shapes = XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
            shapes->GetFreeShapes(roots);
            if (roots.Length() < 0 || roots.Length() > 50000) return Standard_False;
            for (int i = 1; i <= roots.Length(); ++i) {
                const auto& label = roots.Value(i);
                XCAFDoc_VisMaterialPBR material;
                if (!TryPBRMaterialForLabel(label, material) || material.NormalTexture.IsNull()) continue;
                OcctAuthoredFrameRecord boundFrame;
                const auto state = Core3DReadAuthoredFrameOwner(myOcafDoc, label, boundFrame);
                Standard_Size bytes = 0;
                if (state == OcctAuthoredFrameReadState::Invalid) return Standard_False;
                const bool projectedSupplied = label.IsEqual(destination)
                    ? !frame.archive.empty() : state == OcctAuthoredFrameReadState::Authored;
                if (Core3DNormalTextureRecipeForLabel(label) != (projectedSupplied ? 2 : 1)
                    || (!projectedSupplied && !Core3DValidateNormalTextureGeometry(label, &bytes))
                    || !AddMultipliedWithinLimit(residentBytes, bytes, 1U, 64U * 1024U * 1024U)) return Standard_False;
            }
        }
        // Validation is complete before any writes. Caller owns command/abort.
        if (frame.archive.empty()) destination.ForgetAttribute(core3d::persistence::AuthoredFrameAttributeID());
        else {
            const auto bytes = TDataStd_ByteArray::Set(destination, core3d::persistence::AuthoredFrameAttributeID(),
                0, Standard_Integer(frame.archive.size()) - 1, Standard_False);
            for (Standard_Size i = 0; i < frame.archive.size(); ++i) bytes->SetValue(Standard_Integer(i), frame.archive[i]);
        }
        if (before.meshUVAtlasVersion == 0) destination.ForgetAttribute(MeshUVAtlasAttributeID());
        else TDataStd_Integer::Set(destination, MeshUVAtlasAttributeID(), before.meshUVAtlasVersion);
        for (int i = 0; i < 3; ++i) {
            if (before.meshUVAtlasVersion == 2) TDataStd_Integer::Set(destination, MeshUVAtlasSettingsAttributeID(i), before.meshUVAtlasSettings[i]);
            else destination.ForgetAttribute(MeshUVAtlasSettingsAttributeID(i));
        }
        if ((!before.meshRegionPartition.empty() || !target.meshRegionPartition.empty())
            && (before.meshRegionPartition.empty()
                ? !ClearMeshRegionPartition(destination)
                : !StageMeshRegionPartition(destination, before.meshRegionPartition))) return Standard_False;
        // Source-face provenance copies only when the destination triangulation
        // still matches the staged digest; any mismatch omits the record rather
        // than repairing it. A stale source record never matches by construction.
        if (sourceProvenanceState == CopySourceFaceProvenanceReadState::Present) {
            TopoDS_Face destinationFace; Handle(Poly_Triangulation) destinationMesh;
            std::string destinationDigest;
            if (TriangleAtlasFace(target.shape, destinationFace, destinationMesh)
                && core3d::provenance::CopyTriangulationDigest(destinationMesh, destinationDigest)
                && destinationDigest == sourceProvenance.digest) {
                std::string payload;
                if (!core3d::provenance::EncodeEntries(sourceProvenance.faces, payload)) return Standard_False;
                if (targetProvenanceLabel.IsNull()) {
                    bool overflow = false;
                    targetProvenanceLabel = FreshCopySourceFaceProvenanceLabel(destination, overflow);
                    if (overflow || targetProvenanceLabel.IsNull()) return Standard_False;
                } else targetProvenanceLabel.ForgetAllAttributes(Standard_True);
                core3d::provenance::SourceFaceProvenanceRecord copiedProvenance;
                if (!WriteCopySourceFaceProvenanceChunks(targetProvenanceLabel, payload,
                        Standard_Integer(sourceProvenance.faces.size()),
                        sourceProvenance.triangleCount, sourceProvenance.digest)
                    || ReadCopySourceFaceProvenanceRecord(myOcafDoc, destination, copiedProvenance)
                        != CopySourceFaceProvenanceReadState::Present
                    || copiedProvenance.digest != sourceProvenance.digest
                    || !core3d::provenance::SameEntries(copiedProvenance.faces, sourceProvenance.faces))
                    return Standard_False;
            }
        }
        OcctObjectTransformState copied;
        return CaptureObjectTransformStateForLabel(destination, copied)
            && copied.meshRegionPartition == before.meshRegionPartition;
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctDocument::CaptureMeshUVAtlasPreview(
    const TDF_Label& label, OcctMeshUVAtlasPreview& preview) const noexcept {
    preview={};
    if (![NSThread isMainThread]) return Standard_False;
    try {
        OcctObjectTransformState source;
        if (!CaptureObjectTransformStateForLabel(label,source) || source.meshUVAtlasVersion!=2) return Standard_False;
        TopoDS_Face face; Handle(Poly_Triangulation) mesh;
        if (!TriangleAtlasFace(source.shape,face,mesh) || !mesh->HasUVNodes()
            || mesh->NbNodes()-3*mesh->NbTriangles()!=source.meshUVAtlasSettings[2]) return Standard_False;
        const int count=mesh->NbTriangles(), originals=source.meshUVAtlasSettings[2];
        using Corner=std::array<double,5>;
        std::map<std::pair<Corner,Corner>,std::vector<int>> edges;
        std::vector<int> parents(count);std::iota(parents.begin(),parents.end(),0);
        auto root=[&](int i) { while(parents[i]!=i) { parents[i]=parents[parents[i]];i=parents[i]; } return i; };
        OcctMeshUVAtlasPreview result;result.triangleUVs.reserve(count*6);
        for(int triangle=0;triangle<count;++triangle) {
            int ids[3];mesh->Triangle(triangle+1).Get(ids[0],ids[1],ids[2]);
            std::array<Corner,3> corners;
            for(int k=0;k<3;++k) {
                if(ids[k]!=originals+triangle*3+1+k) return Standard_False;
                const auto& point=mesh->Node(ids[k]);const auto& uv=mesh->UVNode(ids[k]);
                corners[k]={point.X(),point.Y(),point.Z(),uv.X(),uv.Y()};
                for(double value:corners[k]) if(!std::isfinite(value)) return Standard_False;
                if(uv.X()<0 || uv.X()>1 || uv.Y()<0 || uv.Y()>1) return Standard_False;
                result.triangleUVs.push_back(uv.X());result.triangleUVs.push_back(uv.Y());
            }
            const auto& a=corners[0];const auto& b=corners[1];const auto& c=corners[2];
            const double area=std::abs((b[3]-a[3])*(c[4]-a[4])-(b[4]-a[4])*(c[3]-a[3]))*0.5;
            if(!std::isfinite(area) || area<=0) return Standard_False;
            result.occupancy+=area;
            for(int k=0;k<3;++k) {
                auto first=corners[k],second=corners[(k+1)%3];if(second<first)std::swap(first,second);
                auto& uses=edges[{first,second}];uses.push_back(triangle);
                if(uses.size()>2) return Standard_False;
                if(uses.size()==2) parents[root(triangle)]=root(uses[0]);
            }
        }
        for(int i=0;i<count;++i) if(root(i)==i) ++result.chartCount;
        if(!std::isfinite(result.occupancy) || result.occupancy>1+1.e-10) return Standard_False;
        result.authoredResolution=source.meshUVAtlasSettings[0];
        result.authoredGutterPixels=source.meshUVAtlasSettings[1];
        preview=std::move(result);return Standard_True;
    } catch(...) { preview={};return Standard_False; }
}

namespace {
const Standard_GUID& CurvedUVLayoutMarkerID() {
    static const Standard_GUID id("ad64e42b-71e9-490c-8610-f21e9aa87031"); return id;
}
const Standard_GUID& CurvedUVLayoutChunksID() {
    static const Standard_GUID id("ad64e42b-71e9-490c-8610-f21e9aa87032"); return id;
}
// Optional diagnostics live on a sibling of the strict N2 record. Existing
// document readers already admit UAttribute/Integer/AsciiString metadata.
bool FindCurvedUVLayout(const TDF_Label& owner, TDF_Label& output) {
    output=TDF_Label(); Standard_Size visited=0;
    for(TDF_ChildIterator child(owner,Standard_True);child.More();child.Next()) {
        if(++visited>kMaximumGeometryDocumentLabels) return false;
        if(!child.Value().IsAttribute(CurvedUVLayoutMarkerID())) continue;
        if(!output.IsNull() || !child.Value().Father().IsEqual(owner)) return false;
        output=child.Value();
    }
    return true;
}
std::string CurvedCornerDigest(const Handle(Poly_Triangulation)& mesh,int first,int count) {
    std::string bytes;
    if(mesh.IsNull() || !mesh->HasUVNodes() || first<0 || count<=0 || first>mesh->NbTriangles()-count) return bytes;
    for(int t=first+1;t<=first+count;++t) {
        int ids[3];mesh->Triangle(t).Get(ids[0],ids[1],ids[2]);
        for(int id:ids) {
            if(id<1 || id>mesh->NbNodes()) return {};
            const auto uv=mesh->UVNode(id);
            if(!std::isfinite(uv.X()) || !std::isfinite(uv.Y())) return {};
            // Exact floating-point values, independent of host byte order.
            char text[128];int size=std::snprintf(text,sizeof text,"%a,%a;",uv.X(),uv.Y());
            if(size<=0 || size>=int(sizeof text)) return {};
            bytes.append(text,std::size_t(size));
        }
    }
    unsigned char hash[CC_SHA256_DIGEST_LENGTH];
    CC_SHA256(bytes.data(),static_cast<CC_LONG>(bytes.size()),hash);
    static const char hex[]="0123456789abcdef";
    std::string result;for(auto byte:hash){result.push_back(hex[byte>>4]);result.push_back(hex[byte&15]);}
    return result;
}

// N1 does not persist a source link. The caller explicitly supplies the retained
// source. Recreate its deterministic copy privately and require exact metadata
// AND geometry digest equality before reading live face tolerances and Ax3s.
// This refuses a source edited since the copy was made; it never guesses a match.
bool BuildCurvedUV(const OcctDocument& document,const TDF_Label& label,
    const Handle(Poly_Triangulation)& mesh,const OcctMeshUVAtlasOptions& options,
    shapeyard::uv::curved::Result& output) {
    output={};
    if(options.version!=2 || options.curvedSource.IsNull() || options.curvedSource.IsEqual(label)
        || document.Document().IsNull() || options.curvedSource.Data()!=document.Document()->GetData()) return false;
    double lengthUnit=0;
    if(!XCAFDoc_DocumentTool::GetLengthUnit(document.Document(),lengthUnit) || lengthUnit!=0.001) return false;
    core3d::provenance::SourceFaceProvenanceRecord provenance;
    if(document.TryCopySourceFaceProvenanceForLabel(label,provenance)
        !=core3d::provenance::CopySourceFaceProvenanceReadState::Present) return false;
    OcctObjectTransformState source;
    if(!document.CaptureObjectTransformStateForLabel(options.curvedSource,source)
        || source.resolvedRepresentation!=OcctGeometryRepresentation::BRep) return false;
    std::atomic_bool cancelled{false}; core3d::meshcopy::CurrentTessellationCopy copy;
    if(core3d::meshcopy::PrepareCurrentTessellationCopy(source.shape,copy,cancelled)
            !=core3d::meshcopy::PreparationResult::Ready
        || !core3d::provenance::SameSourceFaces(provenance.faces,copy.faces)) return false;
    TopLoc_Location location; const auto sourceMesh=BRep_Tool::Triangulation(copy.face,location);
    std::string digest;
    if(!location.IsIdentity() || !core3d::provenance::CopyTriangulationDigest(sourceMesh,digest)
        || digest!=provenance.digest || mesh.IsNull() || mesh->NbTriangles()!=provenance.triangleCount) return false;
    std::vector<shapeyard::uv::curved::FaceInput> faces;
    std::size_t index=0;
    for(TopExp_Explorer explorer(source.shape,TopAbs_FACE);explorer.More();explorer.Next(),++index) {
        if(index>=provenance.faces.size()) return false;
        const auto face=TopoDS::Face(explorer.Current());const BRepAdaptor_Surface surface(face);
        const auto& entry=provenance.faces[index];
        shapeyard::uv::curved::FaceInput input;
        input.surfaceType=entry.surfaceType;input.firstTriangle=entry.firstTriangle;
        input.triangleCount=entry.triangleCount;input.reversed=entry.reversed;input.params=entry.params;
        input.toleranceMM=BRep_Tool::Tolerance(face);
        if(!std::isfinite(input.toleranceMM) || input.toleranceMM<=0) return false;
        gp_Dir x;
        switch(entry.surfaceType) {
            case core3d::provenance::kSurfaceTypePlane: x=surface.Plane().Position().XDirection();break;
            case core3d::provenance::kSurfaceTypeCylinder: x=surface.Cylinder().Position().XDirection();break;
            case core3d::provenance::kSurfaceTypeTorus: x=surface.Torus().Position().XDirection();break;
            default:break;
        }
        input.xDirection={x.X(),x.Y(),x.Z()};faces.push_back(std::move(input));
    }
    if(index!=provenance.faces.size()) return false;
    std::vector<shapeyard::uv::Triangle> triangles(mesh->NbTriangles());
    for(int t=1;t<=mesh->NbTriangles();++t) {
        int ids[3];mesh->Triangle(t).Get(ids[0],ids[1],ids[2]);
        for(int k=0;k<3;++k) {
            if(ids[k]<1 || ids[k]>mesh->NbNodes()) return false;
            const auto p=mesh->Node(ids[k]);triangles[t-1].points[k]={p.X(),p.Y(),p.Z()};
        }
    }
    output=shapeyard::uv::curved::unwrap(triangles,faces,{options.resolution,options.gutterPixels});
    return output.ok;
}

bool WriteCurvedUVLayout(const OcctDocument& document,const TDF_Label& label,
    const Handle(Poly_Triangulation)& mesh,const OcctMeshUVAtlasOptions& options) {
    shapeyard::uv::curved::Result value;
    if(!BuildCurvedUV(document,label,mesh,options,value)) return false;
    core3d::provenance::SourceFaceProvenanceRecord provenance;
    if(document.TryCopySourceFaceProvenanceForLabel(label,provenance)
        !=core3d::provenance::CopySourceFaceProvenanceReadState::Present) return false;
    // Binding is exact, including corner order. Mark runs after shape staging
    // inside the existing OrdinaryEditController command; any failure aborts it.
    for(int t=1;t<=mesh->NbTriangles();++t) {
        int ids[3];mesh->Triangle(t).Get(ids[0],ids[1],ids[2]);
        for(int k=0;k<3;++k) {
            const auto uv=mesh->UVNode(ids[k]);const auto expected=value.atlas.corners[t-1][k];
            if(uv.X()!=expected[0] || uv.Y()!=expected[1]) return false;
        }
    }
    const auto digest=CurvedCornerDigest(mesh,0,mesh->NbTriangles());if(digest.empty()) return false;
    std::ostringstream text;text.imbue(std::locale::classic());text<<std::setprecision(17);
    const auto& layout=value.layout;const auto& coverage=value.coverage;
    text<<1<<' '<<layout.globalTexelsPerMM<<' '<<layout.faceCount<<' '
        <<options.resolution<<' '<<options.gutterPixels<<' '<<provenance.digest<<' '<<digest<<' '
        <<coverage.subChartCount<<' '<<coverage.splitLineCount<<' '<<coverage.coveredTexels<<' '
        <<coverage.seamCounts.continuous<<' '<<coverage.seamCounts.declared<<' '<<coverage.seamCounts.torn<<'\n';
    for(std::size_t i=0;i<layout.faces.size();++i) {
        const auto& f=layout.faces[i];const auto& source=provenance.faces[i];
        const auto faceDigest=CurvedCornerDigest(mesh,source.firstTriangle,source.triangleCount);
        if(faceDigest.empty()) return false;
        text<<int(f.kernel)<<' '<<f.subChartCount<<' '<<f.occupancy<<' '<<f.metricStretchMin<<' '
            <<f.metricStretchMax<<' '<<f.seamCount<<' '<<faceDigest<<' '<<std::quoted(f.fallbackReason)<<'\n';
    }
    // Additive v1 extension: old records remain readable but have no census proof.
    text<<"seamCensus1 "<<layout.kernelSeamEdges<<' '<<layout.splitSeamEdges<<' '
        <<layout.diagonalSeamEdges<<' '<<layout.tornEdges<<'\n';
    // The original source's display triangulation is not serialized in XCAF.
    // Persist the remaining D5 inputs so a stored atlas can be independently
    // regenerated from its own exact corner geometry on an unmeshed snapshot.
    text<<"sourceFrames1 "<<layout.regenerationFaces.size()<<'\n';
    for(const auto& f:layout.regenerationFaces)
        text<<f.toleranceMM<<' '<<f.xDirection[0]<<' '<<f.xDirection[1]<<' '<<f.xDirection[2]<<'\n';
    const auto payload=text.str();if(payload.empty() || payload.size()>128*1024) return false;
    TDF_Label record;if(!FindCurvedUVLayout(label,record)) return false;
    if(record.IsNull()) { bool overflow=false;record=FreshCopySourceFaceProvenanceLabel(label,overflow);if(overflow || record.IsNull()) return false; }
    record.ForgetAllAttributes(Standard_True);
    TDataStd_UAttribute::Set(record,CurvedUVLayoutMarkerID());
    int chunks=int((payload.size()+255)/256);TDataStd_Integer::Set(record,CurvedUVLayoutChunksID(),chunks);
    for(int i=0;i<chunks;++i) TDataStd_AsciiString::Set(record.FindChild(i+1,Standard_True),
        TCollection_AsciiString(payload.substr(std::size_t(i)*256,256).c_str()));
    return true;
}
} // namespace

Standard_Boolean OcctDocument::HasCurvedUVLayoutForLabel(const TDF_Label& label) const noexcept {
    try {
        if (myOcafDoc.IsNull() || label.IsNull() || label.Data()!=myOcafDoc->GetData()) return Standard_False;
        TDF_Label record;
        return FindCurvedUVLayout(label,record) && !record.IsNull();
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctDocument::PrepareCurvedUVAtlas(const TDF_Label& label,
    TopoDS_Shape& candidate,const OcctMeshUVAtlasOptions& options,OcctMeshUVAtlasPreview* preview) const noexcept {
    candidate.Nullify();if(preview) *preview={};
    if(options.curvedSource.IsNull() || options.version!=2) return Standard_False;
    // One implementation owns original-node prefixes, material admission,
    // private-copy construction and all existing atlas payload invariants.
    return PrepareTriangleUVAtlas(label,candidate,options,preview);
}

Standard_Boolean OcctDocument::ReadCurvedUVLayoutForLabel(const TDF_Label& label,
    shapeyard::uv::curved::CurvedUVLayoutRecord& layout,curveduv::PackSummary& coverage) const noexcept {
    layout={};coverage={};if(![NSThread isMainThread]) return Standard_False;
    try {
        if(myOcafDoc.IsNull() || label.IsNull() || label.Data()!=myOcafDoc->GetData()) return Standard_False;
        core3d::provenance::SourceFaceProvenanceRecord provenance;
        if(TryCopySourceFaceProvenanceForLabel(label,provenance)
            !=core3d::provenance::CopySourceFaceProvenanceReadState::Present) return Standard_False;
        OcctObjectTransformState state;
        if(!CaptureObjectTransformStateForLabel(label,state) || state.meshUVAtlasVersion!=2) return Standard_False;
        TopoDS_Face face;Handle(Poly_Triangulation) mesh;
        if(!TriangleAtlasFace(state.shape,face,mesh)) return Standard_False;
        TDF_Label record;if(!FindCurvedUVLayout(label,record) || record.IsNull()) return Standard_False;
        Handle(TDataStd_Integer) chunks;
        if(!record.FindAttribute(CurvedUVLayoutChunksID(),chunks) || chunks->Get()<1 || chunks->Get()>512) return Standard_False;
        std::string payload;
        for(int i=1;i<=chunks->Get();++i) {
            Handle(TDataStd_AsciiString) text;const auto child=record.FindChild(i,Standard_False);
            if(child.IsNull() || !child.FindAttribute(TDataStd_AsciiString::GetID(),text)) return Standard_False;
            std::string part=text->Get().ToCString();
            if(part.empty() || part.size()>256 || (i<chunks->Get() && part.size()!=256)) return Standard_False;
            payload+=part;
        }
        std::istringstream stream(payload);stream.imbue(std::locale::classic());
        shapeyard::uv::curved::CurvedUVLayoutRecord value;curveduv::PackSummary stats;
        int resolution=0,gutter=0;std::string geometryDigest,uvDigest;
        if(!(stream>>value.version>>value.globalTexelsPerMM>>value.faceCount>>resolution>>gutter>>geometryDigest>>uvDigest
            >>stats.subChartCount>>stats.splitLineCount>>stats.coveredTexels>>stats.seamCounts.continuous
            >>stats.seamCounts.declared>>stats.seamCounts.torn)
            || value.version!=1 || value.faceCount!=int(provenance.faces.size())
            || !std::isfinite(value.globalTexelsPerMM) || value.globalTexelsPerMM<=0
            || resolution!=state.meshUVAtlasSettings[0] || gutter!=state.meshUVAtlasSettings[1]
            || geometryDigest!=provenance.digest || uvDigest!=CurvedCornerDigest(mesh,0,mesh->NbTriangles())
            || stats.subChartCount<1 || stats.subChartCount>64 || stats.splitLineCount<0
            || stats.coveredTexels<1 || stats.coveredTexels>long(resolution)*resolution
            || stats.seamCounts.continuous<0 || stats.seamCounts.declared<0 || stats.seamCounts.torn<0) return Standard_False;
        value.faces.resize(value.faceCount);
        for(int i=0;i<value.faceCount;++i) {
            auto& f=value.faces[i];int kernel=-1;const auto& source=provenance.faces[i];
            if(!(stream>>kernel>>f.subChartCount>>f.occupancy>>f.metricStretchMin>>f.metricStretchMax
                >>f.seamCount>>f.uvDigest>>std::quoted(f.fallbackReason))
                || kernel<0 || kernel>3 || f.subChartCount<1 || f.subChartCount>64
                || !std::isfinite(f.occupancy) || f.occupancy<0 || f.occupancy>1
                || !std::isfinite(f.metricStretchMin) || !std::isfinite(f.metricStretchMax)
                || f.metricStretchMin<=0 || f.metricStretchMax<f.metricStretchMin || f.seamCount<0
                || f.uvDigest!=CurvedCornerDigest(mesh,source.firstTriangle,source.triangleCount)) return Standard_False;
            f.kernel=static_cast<curveduv::KernelTag>(kernel);
        }
        stream>>std::ws;
        if(stream.eof()) {
            // Unknown is explicit; never invent zero diagonal/kernel counts for legacy v1.
            value.kernelSeamEdges=value.splitSeamEdges=value.diagonalSeamEdges=-1;
            value.tornEdges=stats.seamCounts.torn;
        } else {
            std::string tag;
            if(!(stream>>tag>>value.kernelSeamEdges>>value.splitSeamEdges>>value.diagonalSeamEdges>>value.tornEdges)
                || tag!="seamCensus1" || value.kernelSeamEdges<0 || value.splitSeamEdges<0
                || value.diagonalSeamEdges<0 || value.tornEdges!=stats.seamCounts.torn
                || value.kernelSeamEdges>stats.seamCounts.declared
                || value.splitSeamEdges!=stats.seamCounts.declared-value.kernelSeamEdges
                || value.diagonalSeamEdges>value.tornEdges) return Standard_False;
            stream>>std::ws;
            if(!stream.eof()) {
                int count=0;
                if(!(stream>>tag>>count) || tag!="sourceFrames1" || count!=value.faceCount) return Standard_False;
                for(const auto& entry:provenance.faces) {
                    shapeyard::uv::curved::FaceInput f;
                    f.surfaceType=entry.surfaceType;f.firstTriangle=entry.firstTriangle;
                    f.triangleCount=entry.triangleCount;f.reversed=entry.reversed;f.params=entry.params;
                    if(!(stream>>f.toleranceMM>>f.xDirection[0]>>f.xDirection[1]>>f.xDirection[2])
                        || !std::isfinite(f.toleranceMM) || f.toleranceMM<=0) return Standard_False;
                    for(double d:f.xDirection) if(!std::isfinite(d)) return Standard_False;
                    value.regenerationFaces.push_back(std::move(f));
                }
                stream>>std::ws;if(!stream.eof()) return Standard_False;
            }
        }
        stats.seamCounts.kernelSeamEdges=value.kernelSeamEdges;
        stats.seamCounts.splitSeamEdges=value.splitSeamEdges;
        stats.seamCounts.diagonalSeamEdges=value.diagonalSeamEdges;
        stats.globalTexelsPerMM=value.globalTexelsPerMM;
        stats.occupancy=double(stats.coveredTexels)/(double(resolution)*resolution);
        layout=std::move(value);coverage=stats;return Standard_True;
    } catch(...) {layout={};coverage={};return Standard_False;}
}
#if DEBUG
Standard_Boolean OcctDocument::DebugCurvedUVLayoutForLabel(const TDF_Label& label,
    shapeyard::uv::curved::CurvedUVLayoutRecord& layout,curveduv::PackSummary& coverage) const noexcept {
    return ReadCurvedUVLayoutForLabel(label,layout,coverage);
}
#endif

Standard_Boolean OcctDocument::PrepareTriangleUVAtlas(
    const TDF_Label& label, TopoDS_Shape& candidate, const OcctMeshUVAtlasOptions& options, OcctMeshUVAtlasPreview* preview) const noexcept {
    candidate.Nullify();
    if (preview) *preview = {};
    if (![NSThread isMainThread]) { return Standard_False; }
    try {
        const shapeyard::uv::Settings settings{options.resolution, options.gutterPixels};
        if ((!options.curvedSource.IsNull() && options.version != 2)
            || (options.version != 1 && options.version != 2)
            || (options.version == 1 && (options.resolution != 0 || options.gutterPixels != 0))
            || (options.version == 2 && !settings.valid())) { return Standard_False; }
        OcctObjectTransformState source;
        if (!CaptureObjectTransformStateForLabel(label, source)
            || source.resolvedRepresentation != OcctGeometryRepresentation::TriangleMesh
            || (options.version == 1 && source.meshUVAtlasVersion != 0)) { return Standard_False; }
        if (!core3d::profile::HasOnlyMetadataSubshapes(myOcafDoc, label)) return Standard_False;
        // Repacking explicitly changes how attached images wrap. Keep this
        // opt-in narrow: only an owned coherent/authored layout can be
        // repacked under images; arbitrary imports and legacy grid UVs retain
        // their historical refusal.
        TDF_Label materialLabel;
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label, materialLabel);
        bool hasImages = false;
        if (!materialLabel.IsNull()) {
            const auto materials = XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
            const auto material = materials->GetMaterial(materialLabel);
            if (material.IsNull()) { return Standard_False; }
            hasImages = material->HasCommonMaterial() && !material->CommonMaterial().DiffuseTexture.IsNull();
            if (material->HasPbrMaterial()) {
                const auto& pbr = material->PbrMaterial();
                hasImages = hasImages || !pbr.BaseColorTexture.IsNull() || !pbr.EmissiveTexture.IsNull()
                    || !pbr.NormalTexture.IsNull() || !pbr.MetallicRoughnessTexture.IsNull()
                    || !pbr.OcclusionTexture.IsNull();
            }
        }
        if (hasImages) {
            if (options.version != 2
                || (source.meshUVAtlasVersion != 2 && source.meshUVAtlasVersion != 3)
                || source.authoredFramesPresent) return Standard_False;
            const auto recipe = Core3DNormalTextureRecipeForLabel(label);
            XCAFDoc_VisMaterialPBR material;
            const bool hasNormal = TryPBRMaterialForLabel(label, material) && !material.NormalTexture.IsNull();
            Standard_Size resident = 0;
            if (recipe < 0 || recipe > 1 || hasNormal != (recipe == 1)
                || !Core3DValidateOwnedFrameUsage(myOcafDoc, resident)) return Standard_False;
        }
        TopoDS_Face face; Handle(Poly_Triangulation) mesh;
        if (!TriangleAtlasFace(source.shape, face, mesh)) { return Standard_False; }
        const int count = mesh->NbTriangles();
        if (count <= 0 || (options.version == 2 && count > 4096)) return Standard_False;
        int originals = mesh->NbNodes();
        if (source.meshUVAtlasVersion == 3) {
            double unit = 0;
            const auto recipe = Core3DNormalTextureRecipeForLabel(label);
            XCAFDoc_VisMaterialPBR material;
            const bool hasNormal = TryPBRMaterialForLabel(label, material) && !material.NormalTexture.IsNull();
            Standard_Size resident = 0;
            std::atomic_bool cancelled{false};
            core3d::meshedit::NativeTopologyCapture captured;
            if (options.version != 2 || source.authoredFramesPresent
                || !XCAFDoc_DocumentTool::GetLengthUnit(myOcafDoc, unit) || unit != 0.001
                || recipe < 0 || recipe > 1 || hasNormal != (recipe == 1)
                || !Core3DValidateOwnedFrameUsage(myOcafDoc, resident)
                || !mesh->HasUVNodes() || !mesh->HasNormals()
                || mesh->NbNodes() != 3 * count
                || core3d::meshedit::CaptureNativeTopology(source.shape, captured, cancelled)
                    != core3d::meshedit::TopologyResult::Ready
                || captured.sourceMesh != mesh
                || !core3d::meshedit::HasFlatCornerLayout(captured, 0)) return Standard_False;
            // Preserve each exact authored corner position/normal as the v2
            // original prefix. Prefix UVs are intentionally canonical zero;
            // Undo owns restoration of the old authored UV bytes.
            originals = 3 * count;
        } else if (source.meshUVAtlasVersion != 0) {
            originals -= 3 * count;
            if (!mesh->HasUVNodes() || originals <= 0
                || (source.meshUVAtlasVersion == 2 && source.meshUVAtlasSettings[2] != originals)) { return Standard_False; }
            // Owned prefix stays fixed through regeneration, including legacy v1.
            for (int i=1;i<=count;++i) {
                int ids[3]; mesh->Triangle(i).Get(ids[0],ids[1],ids[2]);
                for (int k=0;k<3;++k) if (ids[k] != originals+(i-1)*3+1+k) { return Standard_False; }
            }
        }
        if (originals <= 0 || originals > 12288) { return Standard_False; }
        shapeyard::uv::Atlas coherent;
        if (options.version == 2) {
            std::vector<shapeyard::uv::Triangle> triangles(count);
            for (int i=1;i<=count;++i) {
                int ids[3]; mesh->Triangle(i).Get(ids[0],ids[1],ids[2]);
                for(int k=0;k<3;++k) {
                    if(ids[k]<1 || ids[k]>mesh->NbNodes()) return Standard_False;
                    const auto p=mesh->Node(ids[k]);
                    triangles[i-1].points[k]={p.X(),p.Y(),p.Z()};
                }
            }
            if (!options.curvedSource.IsNull()) {
                shapeyard::uv::curved::Result curved;
                if (!BuildCurvedUV(*this,label,mesh,options,curved)) return Standard_False;
                coherent=std::move(curved.atlas);
            } else if (!shapeyard::uv::generate(triangles, settings, coherent)) { return Standard_False; }
        }
        const int columns = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(count))));
        const double cell = 1.0 / columns, padding = cell * 0.04;
        Handle(Poly_Triangulation) atlas = new Poly_Triangulation(originals + 3 * count, count, true, mesh->HasNormals());
        atlas->Deflection(mesh->Deflection());
        // Preserve even unused stored nodes and their exact normal values.
        for (int node = 1; node <= originals; ++node) {
            const auto point = mesh->Node(node);
            for (int axis = 1; axis <= 3; ++axis) {
                if (!std::isfinite(point.Coord(axis)) || std::abs(point.Coord(axis)) > 1.0e6) { return Standard_False; }
            }
            atlas->SetNode(node, point); atlas->SetUVNode(node, gp_Pnt2d(0, 0));
            if (mesh->HasNormals()) { gp_Vec3f normal; mesh->Normal(node, normal); atlas->SetNormal(node, normal); }
        }
        for (int triangle = 1; triangle <= count; ++triangle) {
            int ids[3]; mesh->Triangle(triangle).Get(ids[0], ids[1], ids[2]);
            for (int id : ids) { if (id < 1 || id > mesh->NbNodes()) { return Standard_False; } }
            const gp_Vec edge(mesh->Node(ids[0]), mesh->Node(ids[1]));
            const gp_Vec other(mesh->Node(ids[0]), mesh->Node(ids[2]));
            const double length = edge.Magnitude();
            if (!std::isfinite(length) || length <= 1.0e-12) { return Standard_False; }
            const double x = edge.Dot(other) / length;
            const double y = edge.Crossed(other).Magnitude() / length;
            const double minX = std::min(0.0, x), maxX = std::max(length, x);
            const double extent = std::max(maxX - minX, y);
            if (!std::isfinite(x) || !std::isfinite(y) || y <= length * 1.0e-12
                || !std::isfinite(extent) || extent <= 0) { return Standard_False; }
            const double scale = (cell - 2 * padding) / extent;
            const double originU = ((triangle - 1) % columns) * cell + padding;
            const double originV = ((triangle - 1) / columns) * cell + padding;
            const double xs[3] = {0, length, x}, ys[3] = {0, 0, y};
            const int first = originals + (triangle - 1) * 3 + 1;
            for (int corner = 0; corner < 3; ++corner) {
                atlas->SetNode(first + corner, mesh->Node(ids[corner]));
                if (options.version == 2) {
                    const auto& uv = coherent.corners[triangle-1][corner];
                    atlas->SetUVNode(first + corner, gp_Pnt2d(uv[0],uv[1]));
                } else {
                    atlas->SetUVNode(first + corner, gp_Pnt2d(originU + (xs[corner] - minX) * scale, originV + ys[corner] * scale));
                }
                if (mesh->HasNormals()) { gp_Vec3f normal; mesh->Normal(ids[corner], normal); atlas->SetNormal(first + corner, normal); }
            }
            atlas->SetTriangle(triangle, Poly_Triangle(first, first + 1, first + 2));
        }
        BRepBuilderAPI_Copy copy(source.shape, Standard_True, Standard_True);
        TopoDS_Shape result = copy.Shape();
        TopoDS_Face copiedFace; Handle(Poly_Triangulation) copiedMesh;
        if (result.IsNull() || result.IsSame(source.shape)
            || !TriangleAtlasFace(result, copiedFace, copiedMesh) || copiedFace.IsSame(face)) { return Standard_False; }
        BRep_Builder builder; builder.UpdateFace(copiedFace, atlas);
        if (!GeometryClassMatchesRepresentation(ClassifyDefinitionGeometry(result, nullptr), OcctGeometryRepresentation::TriangleMesh)
            || !PreservesMeshRegionPartition(source, result)) { return Standard_False; }
        if (preview && options.version == 2) {
            OcctMeshUVAtlasPreview value;
            value.triangleUVs.reserve(coherent.corners.size()*6);
            for (const auto& triangle:coherent.corners) for (const auto& uv:triangle) {
                value.triangleUVs.push_back(uv[0]);value.triangleUVs.push_back(uv[1]);
            }
            value.chartCount=coherent.chartCount;value.occupancy=coherent.occupancy;
            *preview=std::move(value);
        }
        candidate = result;
        return Standard_True;
    } catch (...) { candidate.Nullify(); return Standard_False; }
}

namespace {
bool CaptureEditableMeshSource(const OcctDocument& document,const TDF_Label& label,
    OcctObjectTransformState& source,core3d::meshedit::NativeTopologyCapture& captured,
    int& prefix,Standard_Size& resident) noexcept {
    source={};captured={};prefix=0;resident=0;
    if(![NSThread isMainThread])return false;
    try {
        double unit=0;
        if (!document.CaptureObjectTransformStateForLabel(label,source)
            || source.resolvedRepresentation!=OcctGeometryRepresentation::TriangleMesh
            || source.authoredFramesPresent
            || !XCAFDoc_DocumentTool::GetLengthUnit(document.Document(),unit) || unit!=0.001)
            return Standard_False;
        const auto recipe=Core3DNormalTextureRecipeForLabel(label);
        if(recipe<0 || recipe>1)return Standard_False;
        XCAFDoc_VisMaterialPBR material;
        const bool hasNormal=document.TryPBRMaterialForLabel(label,material) && !material.NormalTexture.IsNull();
        if(hasNormal!=(recipe==1))return Standard_False;
        if(!Core3DValidateOwnedFrameUsage(document.Document(),resident))return Standard_False;
        std::atomic_bool cancelled{false};
        if(core3d::meshedit::CaptureNativeTopology(source.shape,captured,cancelled)
            !=core3d::meshedit::TopologyResult::Ready)return Standard_False;
        prefix=0;
        if(source.meshUVAtlasVersion==3) {
            if(!captured.sourceMesh->HasUVNodes()
                || captured.sourceMesh->NbNodes()!=3*captured.sourceMesh->NbTriangles()) return Standard_False;
        } else if(source.meshUVAtlasVersion!=0) {
            prefix=captured.sourceMesh->NbNodes()-3*captured.sourceMesh->NbTriangles();
            if(prefix<=0 || (source.meshUVAtlasVersion==2 && prefix!=source.meshUVAtlasSettings[2]))
                return Standard_False;
        } else if(captured.sourceMesh->HasUVNodes()) {
            // Imported arbitrary layouts require an explicit shading contract.
            return Standard_False;
        }
        return core3d::meshedit::HasFlatCornerLayout(captured,prefix);
    } catch(...) {source={};captured={};return false;}
}
} // namespace

namespace {
bool DecodeMeshRegionPartition(const OcctObjectTransformState& source,
    core3d::meshedit::RegionPartition& partition,
    const core3d::meshedit::RegionPartition*& authority) {
    partition={}; authority=nullptr;
    if(source.meshRegionPartition.empty())return true;
    if(!core3d::meshedit::DecodeRegionPartition(source.meshRegionPartition.data(),
        source.meshRegionPartition.size(),partition))return false;
    authority=&partition;return true;
}
bool SameMeshRegionCandidate(const OcctMeshRegionMutationCandidate& expected,
    const OcctMeshRegionMutationCandidate& candidate) {
    if(expected.partition!=candidate.partition
        || expected.centerSeedTriangle!=candidate.centerSeedTriangle
        || expected.shape.IsNull()||candidate.shape.IsNull()
        || expected.shape.ShapeType()!=candidate.shape.ShapeType()
        || expected.shape.Orientation()!=candidate.shape.Orientation()
        || !expected.shape.Location().IsEqual(candidate.shape.Location()))return false;
    std::atomic_bool cancelled{false};core3d::meshedit::NativeMeshStorageCapture a,b;
    if(core3d::meshedit::CaptureNativeMeshStorage(expected.shape,a,cancelled)
            !=core3d::meshedit::TopologyResult::Ready
        || core3d::meshedit::CaptureNativeMeshStorage(candidate.shape,b,cancelled)
            !=core3d::meshedit::TopologyResult::Ready)return false;
    return a.face.Orientation()==b.face.Orientation()
        && a.meshLocation.IsEqual(b.meshLocation) && a.storedNodes==b.storedNodes
        && a.storedUVs==b.storedUVs && a.storedNormals==b.storedNormals
        && a.deflection==b.deflection && a.triangleNodeIDs==b.triangleNodeIDs;
}
}

Standard_Boolean OcctDocument::CanEditMeshVertices(const TDF_Label& label) const noexcept {
    OcctObjectTransformState source;core3d::meshedit::NativeTopologyCapture captured;
    int prefix=0;Standard_Size resident=0;
    return CaptureEditableMeshSource(*this,label,source,captured,prefix,resident);
}

// All public entry points run on the native owner thread. No writes here.
Standard_Boolean OcctDocument::PrepareMeshVertexMove(const TDF_Label& label,
    const std::vector<std::uint32_t>& vertices, const gp_Vec& worldDelta,
    OcctMeshVertexMutationCandidate& candidate) const noexcept {
    candidate={};
    if (![NSThread isMainThread]) return Standard_False;
    try {
        OcctObjectTransformState source;core3d::meshedit::NativeTopologyCapture captured;
        int prefix=0;Standard_Size resident=0;
        if(!CaptureEditableMeshSource(*this,label,source,captured,prefix,resident))return Standard_False;
        core3d::meshedit::RegionPartition partition;
        const core3d::meshedit::RegionPartition* authority=nullptr;
        if(!DecodeMeshRegionPartition(source,partition,authority)
            || (authority && source.meshUVAtlasVersion!=2 && source.meshUVAtlasVersion!=3))
            return Standard_False;
        const auto recipe=Core3DNormalTextureRecipeForLabel(label);
        std::atomic_bool cancelled{false};
        const gp_Trsf placed=source.transform*captured.meshLocation.Transformation();
        const double scale=placed.ScaleFactor();
        if(authority && (placed.IsNegative() || !std::isfinite(scale) || scale<=0))return Standard_False;
        for(int a=1;a<=3;++a)
            if(!std::isfinite(worldDelta.Coord(a)) || std::abs(worldDelta.Coord(a))>1.e6)return Standard_False;
        const gp_Vec localDelta=worldDelta.Transformed(placed.Inverted());
        TopoDS_Shape result;std::vector<std::uint8_t> encoded;
        if(authority) {
            core3d::meshedit::PartitionedVertexMoveCandidate prepared;
            if(core3d::meshedit::PreparePartitionedFlatVertexMove(captured,vertices,
                    {localDelta.X(),localDelta.Y(),localDelta.Z()},prefix,*authority,prepared,cancelled)
                !=core3d::meshedit::TopologyResult::Ready
                || prepared.partition.empty()
                || !core3d::meshedit::EncodeRegionPartition(prepared.partition,encoded))
                return Standard_False;
            result=prepared.shape;
        } else if(core3d::meshedit::PrepareFlatVertexMove(captured,vertices,
                {localDelta.X(),localDelta.Y(),localDelta.Z()},prefix,result,cancelled)
            !=core3d::meshedit::TopologyResult::Ready)return Standard_False;
        core3d::meshedit::NativeTopologyCapture edited;
        if(core3d::meshedit::CaptureNativeTopology(result,edited,cancelled)
            !=core3d::meshedit::TopologyResult::Ready)return Standard_False;
        for(const auto& p:edited.storedNodes) {
            const auto world=gp_Pnt(p[0],p[1],p[2]).Transformed(placed);
            for(int a=1;a<=3;++a)
                if(!std::isfinite(world.Coord(a)) || std::abs(world.Coord(a))>1.e6)return Standard_False;
        }
        if(recipe==1) {
            Standard_Size beforeBytes=0,afterBytes=0;
            if(!Core3DValidateNormalTextureBinding(myOcafDoc,label,&beforeBytes)
                || !ValidateNormalTextureShape(result,&afterBytes) || beforeBytes>resident)
                return Standard_False;
            resident-=beforeBytes;
            if(!AddMultipliedWithinLimit(resident,afterBytes,1U,64U*1024U*1024U))return Standard_False;
        }
        OcctMeshVertexMutationCandidate value;
        value.shape=result;value.partition.assign(encoded.begin(),encoded.end());
        candidate=std::move(value);return Standard_True;
    } catch(...) {candidate={};return Standard_False;}
}

Standard_Boolean OcctDocument::PrepareMeshVertexMove(const TDF_Label& label,
    const std::vector<std::uint32_t>& vertices, const gp_Vec& worldDelta,
    TopoDS_Shape& candidate) const noexcept {
    candidate.Nullify();
    std::vector<Standard_Byte> sourcePartition;
    if(!CaptureMeshRegionPartition(label,sourcePartition) || !sourcePartition.empty())return Standard_False;
    OcctMeshVertexMutationCandidate value;
    if(!PrepareMeshVertexMove(label,vertices,worldDelta,value) || !value.partition.empty())return Standard_False;
    candidate=value.shape;return Standard_True;
}

Standard_Boolean OcctDocument::ValidateMeshVertexMove(const TDF_Label& label,
    const std::vector<std::uint32_t>& vertices,const gp_Vec& worldDelta,
    const OcctMeshVertexMutationCandidate& candidate) const noexcept {
    try {
        OcctMeshVertexMutationCandidate expected;
        if(!PrepareMeshVertexMove(label,vertices,worldDelta,expected)
            || expected.partition!=candidate.partition || expected.shape.IsNull() || candidate.shape.IsNull())
            return Standard_False;
        std::atomic_bool cancelled{false};
        core3d::meshedit::NativeTopologyCapture a,b;
        if(core3d::meshedit::CaptureNativeTopology(expected.shape,a,cancelled)!=core3d::meshedit::TopologyResult::Ready
            || core3d::meshedit::CaptureNativeTopology(candidate.shape,b,cancelled)!=core3d::meshedit::TopologyResult::Ready)
            return Standard_False;
        return expected.shape.ShapeType()==candidate.shape.ShapeType()
            && expected.shape.Orientation()==candidate.shape.Orientation()
            && expected.shape.Location().IsEqual(candidate.shape.Location())
            && a.face.Orientation()==b.face.Orientation()
            && a.meshLocation.IsEqual(b.meshLocation) && a.storedNodes==b.storedNodes
            && a.storedUVs==b.storedUVs && a.storedNormals==b.storedNormals && a.deflection==b.deflection
            && a.triangleNodeIDs==b.triangleNodeIDs;
    } catch(...) {return Standard_False;}
}

Standard_Boolean OcctDocument::ValidateMeshVertexMove(const TDF_Label& label,
    const std::vector<std::uint32_t>& vertices,const gp_Vec& worldDelta,
    const TopoDS_Shape& candidate) const noexcept {
    std::vector<Standard_Byte> sourcePartition;
    if(!CaptureMeshRegionPartition(label,sourcePartition) || !sourcePartition.empty())return Standard_False;
    OcctMeshVertexMutationCandidate value;value.shape=candidate;
    return ValidateMeshVertexMove(label,vertices,worldDelta,value);
}

// Capture exact owner state separately from strict selectable topology.
namespace {
bool CaptureWindingRepairSource(const OcctDocument& document, const TDF_Label& label,
    OcctObjectTransformState& source, core3d::meshedit::NativeMeshStorageCapture& captured,
    int& prefix, Standard_Size& resident) noexcept {
    source={}; captured={}; prefix=0; resident=0;
    if (![NSThread isMainThread]) return false;
    try {
        double unit=0;
        if (!document.CaptureObjectTransformStateForLabel(label,source)
            || source.resolvedRepresentation!=OcctGeometryRepresentation::TriangleMesh
            || source.authoredFramesPresent
            || !XCAFDoc_DocumentTool::GetLengthUnit(document.Document(),unit) || unit!=0.001)
            return false;
        const auto recipe=Core3DNormalTextureRecipeForLabel(label);
        if (recipe<0 || recipe>1) return false;
        XCAFDoc_VisMaterialPBR material;
        const bool normal=document.TryPBRMaterialForLabel(label,material) && !material.NormalTexture.IsNull();
        if (normal!=(recipe==1) || !Core3DValidateOwnedFrameUsage(document.Document(),resident)) return false;
        std::atomic_bool cancelled{false};
        using namespace core3d::meshedit;
        if (CaptureNativeMeshStorage(source.shape,captured,cancelled)!=TopologyResult::Ready) return false;
        if (source.meshUVAtlasVersion==3) {
            if(!captured.sourceMesh->HasUVNodes()
                || captured.sourceMesh->NbNodes()!=3*captured.sourceMesh->NbTriangles()) return false;
        } else if (source.meshUVAtlasVersion!=0) {
            if (source.meshUVAtlasVersion!=1 && source.meshUVAtlasVersion!=2) return false;
            prefix=captured.sourceMesh->NbNodes()-3*captured.sourceMesh->NbTriangles();
            if (prefix<=0 || (source.meshUVAtlasVersion==2 && prefix!=source.meshUVAtlasSettings[2])) return false;
        } else if (captured.sourceMesh->HasUVNodes()) return false;
        // Raw inconsistent storage is never offered as selectable topology.
        // Validate each existing flat corner normal independently, then the
        // winding planner validates cross-triangle orientability separately.
        NativeTopologyCapture flat;
        static_cast<NativeMeshStorageCapture&>(flat)=captured;
        for (const auto& triangle:captured.storedTriangles) {
            Topology isolated;
            if (Analyze({triangle},isolated,cancelled)!=TopologyResult::Ready) return false;
            flat.topology.unitNormals.push_back(isolated.unitNormals.front());
        }
        return HasFlatCornerLayout(flat,prefix);
    } catch (...) { source={}; captured={}; return false; }
}
}

OcctMeshWindingRepairResult OcctDocument::PrepareMeshWindingRepair(
    const TDF_Label& label, TopoDS_Shape& candidate) const noexcept {
    candidate.Nullify();
    if (![NSThread isMainThread]) return OcctMeshWindingRepairResult::Invalid;
    try {
        using namespace core3d::meshedit;
        OcctObjectTransformState source; NativeMeshStorageCapture captured;
        int prefix=0; Standard_Size resident=0;
        if (!CaptureWindingRepairSource(*this,label,source,captured,prefix,resident))
            return OcctMeshWindingRepairResult::Invalid;
        std::atomic_bool cancelled{false}; WindingPlan plan;
        if (PlanConsistentWinding(captured.storedTriangles,plan,cancelled)!=TopologyResult::Ready)
            return OcctMeshWindingRepairResult::Invalid;
        if (plan.reversedTriangles.empty()) return OcctMeshWindingRepairResult::Unchanged;
        TopoDS_Shape result;
        if (PrepareFlatWindingRepair(captured,prefix,result,plan,cancelled)!=TopologyResult::Ready)
            return OcctMeshWindingRepairResult::Invalid;
        NativeTopologyCapture edited;
        if (CaptureNativeTopology(result,edited,cancelled)!=TopologyResult::Ready)
            return OcctMeshWindingRepairResult::Invalid;
        const gp_Trsf placed=source.transform*edited.meshLocation.Transformation();
        for (const auto& point:edited.storedNodes) {
            const auto world=gp_Pnt(point[0],point[1],point[2]).Transformed(placed);
            for (int axis=1;axis<=3;++axis)
                if (!std::isfinite(world.Coord(axis)) || std::abs(world.Coord(axis))>1.e6)
                    return OcctMeshWindingRepairResult::Invalid;
        }
        if (Core3DNormalTextureRecipeForLabel(label)==1) {
            Standard_Size before=0,after=0;
            if (!Core3DValidateNormalTextureBinding(myOcafDoc,label,&before)
                || !ValidateNormalTextureShape(result,&after) || before>resident)
                return OcctMeshWindingRepairResult::Invalid;
            resident-=before;
            if (!AddMultipliedWithinLimit(resident,after,1U,64U*1024U*1024U))
                return OcctMeshWindingRepairResult::Invalid;
        }
        candidate=result;
        return OcctMeshWindingRepairResult::Prepared;
    } catch (...) { candidate.Nullify(); return OcctMeshWindingRepairResult::Invalid; }
}

Standard_Boolean OcctDocument::ValidateMeshWindingRepair(const TDF_Label& label,
    const TopoDS_Shape& candidate) const noexcept {
    if (![NSThread isMainThread]) return Standard_False;
    try {
        TopoDS_Shape expected;
        if (PrepareMeshWindingRepair(label,expected)!=OcctMeshWindingRepairResult::Prepared) return Standard_False;
        std::atomic_bool cancelled{false};
        core3d::meshedit::NativeTopologyCapture a,b;
        if (core3d::meshedit::CaptureNativeTopology(expected,a,cancelled)!=core3d::meshedit::TopologyResult::Ready
            || core3d::meshedit::CaptureNativeTopology(candidate,b,cancelled)!=core3d::meshedit::TopologyResult::Ready)
            return Standard_False;
        return expected.ShapeType()==candidate.ShapeType() && expected.Orientation()==candidate.Orientation()
            && expected.Location().IsEqual(candidate.Location()) && a.face.Orientation()==b.face.Orientation()
            && a.meshLocation.IsEqual(b.meshLocation) && a.storedNodes==b.storedNodes
            && a.storedUVs==b.storedUVs && a.storedNormals==b.storedNormals && a.deflection==b.deflection
            && a.triangleNodeIDs==b.triangleNodeIDs;
    } catch (...) { return Standard_False; }
}



Standard_Boolean OcctDocument::CaptureMeshRegionExtrudePreview(const TDF_Label& label,
    std::uint32_t seedTriangle, OcctMeshRegionExtrudePreview& preview) const noexcept {
    preview={};
    if(![NSThread isMainThread])return Standard_False;
    try {
        OcctObjectTransformState source;core3d::meshedit::NativeTopologyCapture captured;
        int prefix=0;Standard_Size resident=0;
        if(!CaptureEditableMeshSource(*this,label,source,captured,prefix,resident)
            || source.meshUVAtlasVersion==0 || source.authoredFramesPresent
            || captured.face.Orientation()!=TopAbs_FORWARD
            || !core3d::profile::HasOnlyMetadataSubshapes(myOcafDoc,label))return Standard_False;
        const gp_Trsf placed=source.transform*captured.meshLocation.Transformation();
        if(placed.IsNegative() || !std::isfinite(placed.ScaleFactor()) || placed.ScaleFactor()<=0)return Standard_False;
        core3d::meshedit::RegionPartition partition;
        const core3d::meshedit::RegionPartition* authority=nullptr;
        if(!DecodeMeshRegionPartition(source,partition,authority))return Standard_False;
        std::atomic_bool cancelled{false};core3d::meshedit::PlanarRegion region;
        if(core3d::meshedit::ResolvePlanarRegion(captured,seedTriangle,region,cancelled,authority)
            !=core3d::meshedit::TopologyResult::Ready)return Standard_False;
        OcctMeshRegionExtrudePreview result;result.triangleIndices=region.triangles;
        result.localUnitNormal=region.unitNormal;result.localBoundary.reserve(region.boundaryVertices.size());
        for(auto vertex:region.boundaryVertices)result.localBoundary.push_back(captured.topology.vertices[vertex].point);
        preview=std::move(result);return Standard_True;
    } catch(...){preview={};return Standard_False;}
}

Standard_Boolean OcctDocument::CaptureMeshRegionInsetPreview(const TDF_Label& label,
    std::uint32_t seedTriangle, OcctMeshRegionExtrudePreview& preview) const noexcept {
    return CaptureMeshRegionExtrudePreview(label,seedTriangle,preview);
}

Standard_Boolean OcctDocument::PrepareMeshRegionExtrude(const TDF_Label& label,
    std::uint32_t seedTriangle, Standard_Real distanceMM,
    OcctMeshRegionMutationCandidate& candidate) const noexcept {
    candidate={};
    if(![NSThread isMainThread] || !std::isfinite(distanceMM) || distanceMM<=1.e-6 || distanceMM>1.e5)
        return Standard_False;
    try {
        OcctObjectTransformState source;core3d::meshedit::NativeTopologyCapture captured;
        int prefix=0;Standard_Size resident=0;
        if(!CaptureEditableMeshSource(*this,label,source,captured,prefix,resident)
            || source.meshUVAtlasVersion==0 || source.authoredFramesPresent
            || captured.face.Orientation()!=TopAbs_FORWARD
            || !core3d::profile::HasOnlyMetadataSubshapes(myOcafDoc,label))return Standard_False;
        const gp_Trsf placed=source.transform*captured.meshLocation.Transformation();
        const double scale=placed.ScaleFactor();
        if(placed.IsNegative() || !std::isfinite(scale) || scale<=0)return Standard_False;
        core3d::meshedit::RegionPartition partition,preparedPartition;
        const core3d::meshedit::RegionPartition* authority=nullptr;
        if(!DecodeMeshRegionPartition(source,partition,authority))return Standard_False;
        std::atomic_bool cancelled{false};core3d::meshedit::PlanarRegion region;
        if(core3d::meshedit::ResolvePlanarRegion(captured,seedTriangle,region,cancelled,authority)
            !=core3d::meshedit::TopologyResult::Ready)return Standard_False;
        const auto recipe=Core3DNormalTextureRecipeForLabel(label);
        XCAFDoc_VisMaterialPBR material;
        const bool hasNormal=TryPBRMaterialForLabel(label,material) && !material.NormalTexture.IsNull();
        if(recipe<0 || recipe>1 || hasNormal!=(recipe==1))return Standard_False;
        TopoDS_Shape result;
        if(core3d::meshedit::PrepareRegionExtrusion(captured,prefix,region,distanceMM/scale,
            core3d::meshedit::RegionSideUVPolicy::BoundaryStripNormalized,result,cancelled,
            authority,authority?&preparedPartition:nullptr)!=core3d::meshedit::TopologyResult::Ready)
            return Standard_False;
        if(recipe==1) {
            Standard_Size bytes=0;
            if(!ValidateNormalTextureShape(result,&bytes) || resident>64U*1024U*1024U
                || bytes>64U*1024U*1024U-resident)return Standard_False;
        }
        std::vector<std::uint8_t> encoded;
        if(authority && !preparedPartition.empty()
            && !core3d::meshedit::EncodeRegionPartition(preparedPartition,encoded))return Standard_False;
        OcctMeshRegionMutationCandidate value;
        value.shape=result;value.partition.assign(encoded.begin(),encoded.end());
        candidate=std::move(value);return Standard_True;
    } catch(...){candidate={};return Standard_False;}
}

Standard_Boolean OcctDocument::PrepareMeshRegionExtrude(const TDF_Label& label,
    std::uint32_t seedTriangle, Standard_Real distanceMM, TopoDS_Shape& candidate) const noexcept {
    candidate.Nullify();
    std::vector<Standard_Byte> sourcePartition;
    if(!CaptureMeshRegionPartition(label,sourcePartition) || !sourcePartition.empty())
        return Standard_False;
    OcctMeshRegionMutationCandidate value;
    if(!PrepareMeshRegionExtrude(label,seedTriangle,distanceMM,value)
        || !value.partition.empty())return Standard_False;
    candidate=value.shape;return Standard_True;
}

Standard_Boolean OcctDocument::ValidateMeshRegionExtrude(const TDF_Label& label,
    std::uint32_t seedTriangle, Standard_Real distanceMM,
    const OcctMeshRegionMutationCandidate& candidate) const noexcept {
    try {
        OcctMeshRegionMutationCandidate expected;
        return PrepareMeshRegionExtrude(label,seedTriangle,distanceMM,expected)
            && SameMeshRegionCandidate(expected,candidate);
    } catch(...){return Standard_False;}
}

Standard_Boolean OcctDocument::ValidateMeshRegionExtrude(const TDF_Label& label,
    std::uint32_t seedTriangle, Standard_Real distanceMM, const TopoDS_Shape& candidate) const noexcept {
    std::vector<Standard_Byte> sourcePartition;
    if(!CaptureMeshRegionPartition(label,sourcePartition) || !sourcePartition.empty())
        return Standard_False;
    OcctMeshRegionMutationCandidate value;value.shape=candidate;
    return ValidateMeshRegionExtrude(label,seedTriangle,distanceMM,value);
}

Standard_Boolean OcctDocument::PrepareMeshRegionInset(const TDF_Label& label,
    std::uint32_t seedTriangle, Standard_Real distanceMM,
    OcctMeshRegionMutationCandidate& candidate) const noexcept {
    candidate={};
    if(![NSThread isMainThread] || !std::isfinite(distanceMM) || distanceMM<=1.e-6 || distanceMM>1.e5)
        return Standard_False;
    try {
        OcctObjectTransformState source;core3d::meshedit::NativeTopologyCapture captured;
        int prefix=0;Standard_Size resident=0;
        if(!CaptureEditableMeshSource(*this,label,source,captured,prefix,resident)
            || source.meshUVAtlasVersion==0 || source.authoredFramesPresent
            || captured.face.Orientation()!=TopAbs_FORWARD
            || !core3d::profile::HasOnlyMetadataSubshapes(myOcafDoc,label))return Standard_False;
        const gp_Trsf placed=source.transform*captured.meshLocation.Transformation();
        const double scale=placed.ScaleFactor();
        if(placed.IsNegative() || !std::isfinite(scale) || scale<=0)return Standard_False;
        core3d::meshedit::RegionPartition partition;
        const core3d::meshedit::RegionPartition* authority=nullptr;
        if(!DecodeMeshRegionPartition(source,partition,authority))return Standard_False;
        std::atomic_bool cancelled{false};core3d::meshedit::PlanarRegion region;
        if(core3d::meshedit::ResolvePlanarRegion(captured,seedTriangle,region,cancelled,authority)
            !=core3d::meshedit::TopologyResult::Ready)return Standard_False;
        const auto recipe=Core3DNormalTextureRecipeForLabel(label);
        XCAFDoc_VisMaterialPBR material;
        const bool hasNormal=TryPBRMaterialForLabel(label,material) && !material.NormalTexture.IsNull();
        if(recipe<0 || recipe>1 || hasNormal!=(recipe==1))return Standard_False;
        core3d::meshedit::RegionInsetCandidate inset;
        if(core3d::meshedit::PrepareConvexRegionInset(captured,prefix,region,distanceMM/scale,
            authority,inset,cancelled)!=core3d::meshedit::TopologyResult::Ready
            || inset.shape.IsNull() || inset.partition.empty())return Standard_False;
        if(recipe==1) {
            Standard_Size bytes=0;
            if(!ValidateNormalTextureShape(inset.shape,&bytes) || resident>64U*1024U*1024U
                || bytes>64U*1024U*1024U-resident)return Standard_False;
        }
        std::vector<std::uint8_t> encoded;
        if(!core3d::meshedit::EncodeRegionPartition(inset.partition,encoded))return Standard_False;
        OcctMeshRegionMutationCandidate value;
        value.shape=inset.shape;value.partition.assign(encoded.begin(),encoded.end());
        value.centerSeedTriangle=inset.centerSeed;
        candidate=std::move(value);return Standard_True;
    } catch(...){candidate={};return Standard_False;}
}

Standard_Boolean OcctDocument::ValidateMeshRegionInset(const TDF_Label& label,
    std::uint32_t seedTriangle, Standard_Real distanceMM,
    const OcctMeshRegionMutationCandidate& candidate) const noexcept {
    try {
        OcctMeshRegionMutationCandidate expected;
        return PrepareMeshRegionInset(label,seedTriangle,distanceMM,expected)
            && SameMeshRegionCandidate(expected,candidate)
            && !candidate.partition.empty();
    } catch(...){return Standard_False;}
}

Standard_Boolean OcctDocument::MarkAuthoredMeshUVLayout(const TDF_Label& label) noexcept {
    try {
        if(myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || !IsEditableFreeSimpleDefinitionLabel(label)
            || GeometryRepresentationForLabel(label)!=OcctGeometryRepresentation::TriangleMesh)return Standard_False;
        OcctAuthoredFrameRecord frames;
        if(Core3DReadAuthoredFrameOwner(myOcafDoc,label,frames)!=OcctAuthoredFrameReadState::Absent)
            return Standard_False;
        std::atomic_bool cancelled{false};core3d::meshedit::NativeTopologyCapture captured;
        if(core3d::meshedit::CaptureNativeTopology(XCAFDoc_ShapeTool::GetShape(label),captured,cancelled)
            !=core3d::meshedit::TopologyResult::Ready || !captured.sourceMesh->HasUVNodes()
            || !core3d::meshedit::HasFlatCornerLayout(captured,0))return Standard_False;
        TDataStd_Integer::Set(label,MeshUVAtlasAttributeID(),3);
        for(int i=0;i<3;++i)label.ForgetAttribute(MeshUVAtlasSettingsAttributeID(i));
        return Standard_True;
    } catch(...){return Standard_False;}
}

Standard_Boolean OcctDocument::ValidateTriangleUVAtlas(
    const TDF_Label& label, const TopoDS_Shape& candidate, const OcctMeshUVAtlasOptions& options) const noexcept {
    try {
        if (!options.curvedSource.IsNull()) {
            OcctObjectTransformState stored;
            if (!CaptureObjectTransformStateForLabel(label,stored)) return Standard_False;
            if (stored.meshUVAtlasVersion==2 && HasCurvedUVLayoutForLabel(label)) {
                if (options.version!=2
                    || !shapeyard::uv::Settings{options.resolution,options.gutterPixels}.valid()) return Standard_False;
                TopoDS_Face a,b; Handle(Poly_Triangulation) x,y;
                if (!TriangleAtlasFace(stored.shape,a,x) || !TriangleAtlasFace(candidate,b,y)
                    || candidate.ShapeType()!=stored.shape.ShapeType()
                    || candidate.Orientation()!=stored.shape.Orientation()
                    || !candidate.Location().IsEqual(stored.shape.Location())
                    || a.Orientation()!=b.Orientation() || !a.Location().IsEqual(b.Location())
                    || x->NbNodes()!=y->NbNodes() || x->NbTriangles()!=y->NbTriangles()
                    || !x->HasUVNodes() || !y->HasUVNodes() || x->HasNormals()!=y->HasNormals()) return Standard_False;
                const int prefix=stored.meshUVAtlasSettings[2];
                if(prefix<=0 || prefix!=x->NbNodes()-3*x->NbTriangles()) return Standard_False;
                shapeyard::uv::curved::Result regenerated;
                if(!BuildCurvedUV(*this,label,x,options,regenerated)) return Standard_False;
                for(int i=1;i<=x->NbNodes();++i) {
                    if(!x->Node(i).IsEqual(y->Node(i),0.0)) return Standard_False;
                    gp_Pnt2d expected(0,0);
                    if(i>prefix) {
                        const auto& uv=regenerated.atlas.corners[(i-prefix-1)/3][(i-prefix-1)%3];
                        expected=gp_Pnt2d(uv[0],uv[1]);
                    }
                    if(!expected.IsEqual(y->UVNode(i),0.0)) return Standard_False;
                    if(x->HasNormals()) {
                        gp_Vec3f nx,ny;x->Normal(i,nx);y->Normal(i,ny);
                        if(nx!=ny) return Standard_False;
                    }
                }
                for(int t=1;t<=x->NbTriangles();++t) {
                    int ix[3],iy[3];x->Triangle(t).Get(ix[0],ix[1],ix[2]);y->Triangle(t).Get(iy[0],iy[1],iy[2]);
                    for(int k=0;k<3;++k)
                        if(ix[k]!=prefix+3*(t-1)+1+k || iy[k]!=ix[k]) return Standard_False;
                }
                return PreservesMeshRegionPartition(stored,candidate);
            }
        }
        // A committed curved atlas is a non-associative mesh snapshot. Its
        // digest-bound provenance and persisted D5 inputs are authoritative for
        // read-only replay, including when original display meshes are absent.
        // Authoring still requires BuildCurvedUV's exact retained-source checks.
        if (options.curvedSource.IsNull() && HasCurvedUVLayoutForLabel(label)) {
            OcctObjectTransformState stored;
            if (!CaptureObjectTransformStateForLabel(label,stored)
                || stored.meshUVAtlasVersion!=2 || options.version!=2
                || options.resolution!=stored.meshUVAtlasSettings[0]
                || options.gutterPixels!=stored.meshUVAtlasSettings[1]) return Standard_False;
            shapeyard::uv::curved::CurvedUVLayoutRecord layout;curveduv::PackSummary coverage;
            if(!ReadCurvedUVLayoutForLabel(label,layout,coverage)) return Standard_False;
            TopoDS_Face a,b; Handle(Poly_Triangulation) x,y;
            if (!TriangleAtlasFace(stored.shape,a,x) || !TriangleAtlasFace(candidate,b,y)
                || candidate.ShapeType()!=stored.shape.ShapeType()
                || candidate.Orientation()!=stored.shape.Orientation()
                || !candidate.Location().IsEqual(stored.shape.Location())
                || a.Orientation()!=b.Orientation() || !a.Location().IsEqual(b.Location())
                || x->NbNodes()!=y->NbNodes() || x->NbTriangles()!=y->NbTriangles()
                || !x->HasUVNodes() || !y->HasUVNodes() || x->HasNormals()!=y->HasNormals()) return Standard_False;
            const int prefix=stored.meshUVAtlasSettings[2];
            if(prefix<=0 || prefix!=x->NbNodes()-3*x->NbTriangles()) return Standard_False;
            std::vector<shapeyard::uv::Triangle> triangles(x->NbTriangles());
            std::vector<std::array<shapeyard::uv::UV,3>> corners(x->NbTriangles());
            for(int i=1;i<=x->NbNodes();++i) {
                if(!x->Node(i).IsEqual(y->Node(i),0.0)) return Standard_False;
                const auto uv=y->UVNode(i);
                if(i<=prefix) {
                    if(uv.X()!=0 || uv.Y()!=0) return Standard_False;
                } else {
                    const auto p=x->Node(i);const int t=(i-prefix-1)/3,k=(i-prefix-1)%3;
                    triangles[t].points[k]={p.X(),p.Y(),p.Z()};corners[t][k]={uv.X(),uv.Y()};
                }
                if(x->HasNormals()) {
                    gp_Vec3f nx,ny;x->Normal(i,nx);y->Normal(i,ny);
                    if(nx!=ny) return Standard_False;
                }
            }
            for(int t=1;t<=x->NbTriangles();++t) {
                int ix[3],iy[3];x->Triangle(t).Get(ix[0],ix[1],ix[2]);y->Triangle(t).Get(iy[0],iy[1],iy[2]);
                for(int k=0;k<3;++k)
                    if(ix[k]!=prefix+3*(t-1)+1+k || iy[k]!=ix[k]) return Standard_False;
            }
            // Legacy records without replay inputs require the live-source
            // branch above. Do not manufacture missing D5 admission values.
            if(layout.regenerationFaces.empty()
                || !shapeyard::uv::curved::validate(triangles,layout.regenerationFaces,
                    {options.resolution,options.gutterPixels},corners)) return Standard_False;
            return PreservesMeshRegionPartition(stored,candidate);
        }
        TopoDS_Shape expected;
        // The null-source (planar) path, including its edit admission checks,
        // remains unchanged. New curved candidates use exactly their own options.
        const bool prepared=options.curvedSource.IsNull()
            ? PrepareTriangleUVAtlas(label,expected,options)
            : PrepareCurvedUVAtlas(label,expected,options);
        if (!prepared) { return Standard_False; }
        TopoDS_Face a, b; Handle(Poly_Triangulation) x, y;
        if (!TriangleAtlasFace(expected, a, x) || !TriangleAtlasFace(candidate, b, y)
            || candidate.ShapeType() != expected.ShapeType() || candidate.Orientation() != expected.Orientation()
            || !candidate.Location().IsEqual(expected.Location()) || a.Orientation() != b.Orientation()
            || !a.Location().IsEqual(b.Location()) || x->NbNodes() != y->NbNodes()
            || x->NbTriangles() != y->NbTriangles() || !y->HasUVNodes() || x->HasNormals() != y->HasNormals()) { return Standard_False; }
        for (int i = 1; i <= x->NbNodes(); ++i) {
            if (!x->Node(i).IsEqual(y->Node(i), 0.0) || !x->UVNode(i).IsEqual(y->UVNode(i), 0.0)) { return Standard_False; }
            if (x->HasNormals()) { gp_Vec3f nx, ny; x->Normal(i, nx); y->Normal(i, ny); if (nx != ny) { return Standard_False; } }
        }
        for (int i = 1; i <= x->NbTriangles(); ++i) {
            int ix[3], iy[3]; x->Triangle(i).Get(ix[0], ix[1], ix[2]); y->Triangle(i).Get(iy[0], iy[1], iy[2]);
            for (int j = 0; j < 3; ++j) { if (ix[j] != iy[j]) { return Standard_False; } }
        }
        OcctObjectTransformState source;
        return CaptureObjectTransformStateForLabel(label, source)
            && PreservesMeshRegionPartition(source, candidate);
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctDocument::MarkTriangleUVAtlas(const TDF_Label& label, const OcctMeshUVAtlasOptions& options) noexcept {
    try {
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || !IsEditableFreeSimpleDefinitionLabel(label)
            || GeometryRepresentationForLabel(label) != OcctGeometryRepresentation::TriangleMesh) { return Standard_False; }
        if (options.version == 2) {
            if (!shapeyard::uv::Settings{options.resolution,options.gutterPixels}.valid()) return Standard_False;
            TopoDS_Face face; Handle(Poly_Triangulation) mesh;
            if (!TriangleAtlasFace(XCAFDoc_ShapeTool::GetShape(label),face,mesh)) return Standard_False;
            const int originals=mesh->NbNodes()-3*mesh->NbTriangles();
            if (originals<=0 || originals>12288) return Standard_False;
            TDataStd_Integer::Set(label,MeshUVAtlasSettingsAttributeID(0),options.resolution);
            TDataStd_Integer::Set(label,MeshUVAtlasSettingsAttributeID(1),options.gutterPixels);
            TDataStd_Integer::Set(label,MeshUVAtlasSettingsAttributeID(2),originals);
        } else if (options.version != 1 || options.resolution != 0 || options.gutterPixels != 0
                   || label.IsAttribute(MeshUVAtlasSettingsAttributeID(0))
                   || label.IsAttribute(MeshUVAtlasSettingsAttributeID(1))
                   || label.IsAttribute(MeshUVAtlasSettingsAttributeID(2))) { return Standard_False; }
        if (!options.curvedSource.IsNull()) {
            TopoDS_Face face; Handle(Poly_Triangulation) mesh;
            if (options.version!=2 || !TriangleAtlasFace(XCAFDoc_ShapeTool::GetShape(label),face,mesh)
                || !WriteCurvedUVLayout(*this,label,mesh,options)) return Standard_False;
        } else {
            // Optional diagnostics must not describe a later planar regeneration.
            TDF_Label record;
            if (!FindCurvedUVLayout(label,record)) return Standard_False;
            if (!record.IsNull()) record.ForgetAllAttributes(Standard_True);
        }
        TDataStd_Integer::Set(label, MeshUVAtlasAttributeID(), options.version);
        return Standard_True;
    } catch (...) { return Standard_False; }
}

#ifdef DEBUG
Standard_Boolean OcctDocument::DebugProbeMeshUVRepackRecipeMismatch(const TDF_Label& label) const noexcept {
    if(![NSThread isMainThread])return Standard_False;
    try {
        OcctObjectTransformState before;
        if(myOcafDoc.IsNull()||myOcafDoc->HasOpenCommand()
            ||!CaptureObjectTransformStateForLabel(label,before)||before.meshUVAtlasVersion!=3
            ||Core3DNormalTextureRecipeForLabel(label)!=1)return Standard_False;
        const auto undos=myOcafDoc->GetAvailableUndos(),redos=myOcafDoc->GetAvailableRedos();
        myOcafDoc->NewCommand();
        TDataStd_Integer::Set(label,NormalTextureRecipeAttributeID(),0);
        TopoDS_Shape candidate;
        const bool refused=!PrepareTriangleUVAtlas(label,candidate,OcctMeshUVAtlasOptions{2,1024,8})
            && candidate.IsNull();
        myOcafDoc->AbortCommand();
        OcctObjectTransformState after;
        return refused&&!myOcafDoc->HasOpenCommand()
            &&myOcafDoc->GetAvailableUndos()==undos&&myOcafDoc->GetAvailableRedos()==redos
            &&Core3DNormalTextureRecipeForLabel(label)==1
            &&CaptureObjectTransformStateForLabel(label,after)&&before.IsEqual(after);
    }catch(...){
        try{if(!myOcafDoc.IsNull()&&myOcafDoc->HasOpenCommand())myOcafDoc->AbortCommand();}catch(...){}
        return Standard_False;
    }
}
#endif

Standard_Boolean OcctDocument::IsPresentationEditable(
    Handle(AIS_InteractiveObject) object) const {
    if (object.IsNull()) {
        return Standard_False;
    }
    const Handle(CafShapePrs) aCafPresentation =
        Handle(CafShapePrs)::DownCast(object);
    return aCafPresentation.IsNull()
        || aCafPresentation->IsEditablePresentation();
}

Standard_Boolean OcctDocument::IsEditableFreeSimpleDefinitionLabel(
    const TDF_Label& label) const {
    if (myOcafDoc.IsNull() || label.IsNull()
        || label.Data() != myOcafDoc->GetData()) {
        return Standard_False;
    }
    const Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
    return !shapeTool.IsNull() && shapeTool->IsShape(label)
        && XCAFDoc_ShapeTool::IsFree(label)
        && XCAFDoc_ShapeTool::IsSimpleShape(label)
        && !XCAFDoc_ShapeTool::IsReference(label)
        && !XCAFDoc_ShapeTool::IsComponent(label)
        && !XCAFDoc_ShapeTool::IsAssembly(label)
        && !XCAFDoc_ShapeTool::IsSubShape(label);
}

Standard_Boolean OcctDocument::RemoveShape(const TDF_Label& label) {
    if (myOcafDoc.IsNull() || label.IsNull()) {
        return Standard_False;
    }
    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
    TDF_Label aPreviousMaterialLabel;
    XCAFDoc_VisMaterialTool::GetShapeMaterial(
        label, aPreviousMaterialLabel);
    const Standard_Boolean didRemove =
        !shapeTool.IsNull()
        && shapeTool->RemoveShape(label, Standard_True);
    if (didRemove && myOcafDoc->HasOpenCommand()
        && !aPreviousMaterialLabel.IsNull()
        && XCAFDoc_DocumentTool::CheckVisMaterialTool(
            myOcafDoc->Main())) {
        RemoveUnreferencedOwnedMaterial(
            XCAFDoc_DocumentTool::VisMaterialTool(
                myOcafDoc->Main()),
            aPreviousMaterialLabel);
    }
    return didRemove;
}

TDF_Label OcctDocument::AddShape(Handle(AIS_InteractiveObject) object) {
    return AddShape(
        object, OcctGeometryRepresentation::BRep);
}

TDF_Label OcctDocument::AddShape(
    Handle(AIS_InteractiveObject) object,
    const OcctGeometryRepresentation representation) {
    Handle(AIS_Shape) aisShape = Handle(AIS_Shape)::DownCast(object);
    return AddShape(aisShape, representation);
}

TDF_Label OcctDocument::AddShape(Handle(AIS_Shape) aisShape) {
	return AddShape(
	    aisShape, OcctGeometryRepresentation::BRep);
}

TDF_Label OcctDocument::AddShape(
    Handle(AIS_Shape) aisShape,
    const OcctGeometryRepresentation representation) {
	if (myOcafDoc.IsNull()
	    || !myOcafDoc->HasOpenCommand()
	    || aisShape.IsNull()
	    || aisShape->Shape().IsNull()
	    || (representation != OcctGeometryRepresentation::BRep
	        && representation
	            != OcctGeometryRepresentation::TriangleMesh)) {
	    return TDF_Label();
	}
	Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
	if (shapeTool.IsNull()) {
	    return TDF_Label();
	}
	TDF_Label label = shapeTool->NewShape();
	shapeTool->SetShape(label, aisShape->Shape());
	if (!AssignNewIdentifier(label, EntityIdentifierAttributeID())
	    || !AssignNewIdentifier(label, DefinitionIdentifierAttributeID())
	    || !SetGeometryRepresentationForLabel(
	        label, representation)) {
	    return TDF_Label();
	}
	if (!SaveObjectTransform(label, aisShape)) {
	    return TDF_Label();
	}
	return label;
}

Standard_Boolean OcctDocument::ReplaceShape(
    const TDF_Label& label,
    Handle(AIS_Shape) aisShape) {
	// Stable entity and definition identifiers belong to the label, so replacing
	// its geometry deliberately leaves both identity attributes untouched.
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull() || label.Data() != myOcafDoc->GetData()
        || aisShape.IsNull() || aisShape->Shape().IsNull()) {
        return Standard_False;
    }
	Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
    if (!IsEditableFreeSimpleDefinitionLabel(label)) {
        return Standard_False;
    }
    const TopoDS_Shape previousShape = XCAFDoc_ShapeTool::GetShape(label);
    if (previousShape.IsNull()) {
        return Standard_False;
    }
    const OcctGeometryRepresentation previousRepresentation =
        GeometryRepresentationForLabel(label);
    if (previousRepresentation == OcctGeometryRepresentation::Invalid) {
        return Standard_False;
    }
    const OcctGeometryRepresentation resolvedRepresentation =
        previousRepresentation
            == OcctGeometryRepresentation::LegacyUnknown
        ? OcctGeometryRepresentation::BRep
        : previousRepresentation;
    if (!GeometryClassMatchesRepresentation(
            ClassifyDefinitionGeometry(aisShape->Shape(), nullptr),
            resolvedRepresentation)) {
        return Standard_False;
    }
    bool hadRepresentationMarker = false;
    OcctGeometryRepresentation storedRepresentation =
        OcctGeometryRepresentation::Invalid;
    if (!ReadGeometryRepresentation(
            label, hadRepresentationMarker, storedRepresentation)) {
        return Standard_False;
    }
    struct SavedRealAttribute {
        Standard_Boolean wasPresent = Standard_False;
        Standard_Real value = 0.0;
    };
    SavedRealAttribute previousTransform[8];
    for (Standard_Integer tag = 1; tag <= 8; ++tag) {
        const TDF_Label child = label.FindChild(tag, Standard_False);
        Handle(TDataStd_Real) attribute;
        if (!child.IsNull()
            && child.FindAttribute(TDataStd_Real::GetID(), attribute)
            && !attribute.IsNull()) {
            previousTransform[tag - 1].wasPresent = Standard_True;
            previousTransform[tag - 1].value = attribute->Get();
        }
    }
    const auto restorePrevious = [&]() noexcept {
        try {
            shapeTool->SetShape(label, previousShape);
            if (hadRepresentationMarker) {
                TDataStd_Integer::Set(
                    label,
                    GeometryRepresentationAttributeID(),
                    static_cast<Standard_Integer>(storedRepresentation));
            } else {
                label.ForgetAttribute(
                    GeometryRepresentationAttributeID());
            }
            for (Standard_Integer tag = 1; tag <= 8; ++tag) {
                const SavedRealAttribute& saved =
                    previousTransform[tag - 1];
                if (saved.wasPresent) {
                    TDataStd_Real::Set(label.FindChild(tag), saved.value);
                } else {
                    const TDF_Label child =
                        label.FindChild(tag, Standard_False);
                    if (!child.IsNull()) {
                        child.ForgetAttribute(TDataStd_Real::GetID());
                    }
                }
            }
        } catch (...) {
        }
    };
    try {
        shapeTool->SetShape(label, aisShape->Shape());
        const TopoDS_Shape stored = XCAFDoc_ShapeTool::GetShape(label);
        if (stored.IsNull() || !stored.IsEqual(aisShape->Shape())) {
            restorePrevious();
            return Standard_False;
        }
        if (previousRepresentation
                == OcctGeometryRepresentation::LegacyUnknown
            && !WriteGeometryRepresentationMarker(
                label, OcctGeometryRepresentation::BRep)) {
            restorePrevious();
            return Standard_False;
        }
        if (!SaveObjectTransform(label, aisShape)) {
            restorePrevious();
            return Standard_False;
        }
        return Standard_True;
    } catch (...) {
        restorePrevious();
        return Standard_False;
    }
}


Standard_Boolean OcctDocument::CaptureCylindricalCutSource(
    const TDF_Label& label,OcctCylindricalCutSource& output)const noexcept {
    output={};if(!NSThread.isMainThread)CORE3D_CUT_REFUSE("document-capture.thread", Standard_False);
    try {
        OcctCylindricalCutSource result;OcctScalarAppearanceState appearance;
        if(myOcafDoc.IsNull()||!ValidateGeometryRepresentations()
            ||!CaptureObjectTransformStateForLabel(label,result.original)
            ||result.original.resolvedRepresentation!=OcctGeometryRepresentation::BRep
            ||result.original.authoredFramesPresent||result.original.meshUVAtlasVersion
            ||result.original.shape.ShapeType()!=TopAbs_SOLID
            ||result.original.shape.Orientation()!=TopAbs_FORWARD)CORE3D_CUT_REFUSE("document-capture.geometry", Standard_False);
        if(!CaptureScalarAppearanceForSavedCut(label,appearance))CORE3D_CUT_REFUSE("document-capture.appearance", Standard_False);
        const auto& state=result.original;double unit=0,scale=0;
        if(!XCAFDoc_DocumentTool::GetLengthUnit(myOcafDoc,unit)
            ||!core3d::cylindrical_cut::EffectiveMM(state.transform,unit,scale,result.effectiveMM))CORE3D_CUT_REFUSE("document-capture.units", Standard_False);
        auto& e=result.envelope;
        const unsigned families=unsigned(!state.profile.label.IsNull())+unsigned(!state.enclosure.label.IsNull())
            +unsigned(!state.sweep.label.IsNull())+unsigned(!state.loft.label.IsNull())+unsigned(bool(state.retained.value));
        if(families!=1)CORE3D_CUT_REFUSE("document-capture.family-census", Standard_False);
        TDF_Label metadata;
        if(state.retained.value){
            const auto* legacy=std::get_if<core3d::retained_solid::Envelope>(&state.retained.value->envelope);
            if(!legacy)CORE3D_CUT_REFUSE("document-capture.retained-version", Standard_False);
            e=*legacy;result.base=state.retained.value->base;
            result.rebuilding=true;metadata=state.retained.label;
        }else{
            e.metersPerUnit=unit;
            if(!core3d::retained_solid::ReadUUID(myOcafDoc->Main(),DocumentIdentifierAttributeID(),e.document)
                ||!core3d::retained_solid::ReadUUID(label,EntityIdentifierAttributeID(),e.entity)
                ||!core3d::retained_solid::ReadUUID(label,DefinitionIdentifierAttributeID(),e.definition))CORE3D_CUT_REFUSE("document-capture.identities", Standard_False);
            std::string identifier;
            if(!state.profile.label.IsNull()){
                if(!state.profile.IsCurrent(myOcafDoc,label))CORE3D_CUT_REFUSE("document-capture.profile-current", Standard_False);
                e.sourceFamily=1;e.sourceSchema=core3d::profile::SchemaFor(state.profile.parameters);
                e.sourceValues=state.profile.values;metadata=state.profile.label;identifier=state.profile.identifier;
            }else if(!state.enclosure.label.IsNull()){
                if(!state.enclosure.IsCurrent(myOcafDoc,label))CORE3D_CUT_REFUSE("document-capture.enclosure-current", Standard_False);
                e.sourceFamily=2;e.sourceSchema=state.enclosure.parameters.definition.constructionFrame?2:1;
                e.sourceValues=state.enclosure.values;metadata=state.enclosure.label;identifier=state.enclosure.identifier;
            }else if(!state.loft.label.IsNull()){
                if(!state.loft.IsCurrent(myOcafDoc,label))CORE3D_CUT_REFUSE("document-capture.loft-current", Standard_False);
                e.sourceFamily=3;e.sourceSchema=core3d::loft_persistence::Schema;
                e.sourceValues=state.loft.values;metadata=state.loft.label;identifier=state.loft.identifier;
            }else CORE3D_CUT_REFUSE("document-capture.unsupported-family", Standard_False);
            if(!core3d::receipt::ParseUUID(identifier,e.sourceFeature))CORE3D_CUT_REFUSE("document-capture.source-uuid", Standard_False);
            result.base=state.shape;
        }
        if(core3d::retained_solid::Bits(e.metersPerUnit)!=core3d::retained_solid::Bits(unit)
            ||result.base.IsNull()||result.base.ShapeType()!=TopAbs_SOLID)CORE3D_CUT_REFUSE("document-capture.base-or-units", Standard_False);
        TDF_LabelSequence children;XCAFDoc_ShapeTool::GetSubShapes(label,children);
        if(children.Length()>core3d::profile::MaximumLabels)CORE3D_CUT_REFUSE("document-capture.subshape-budget", Standard_False);
        for(int i=1;i<=children.Length();++i)if(!children.Value(i).IsEqual(metadata))CORE3D_CUT_REFUSE("document-capture.foreign-subshape", Standard_False);
        output=std::move(result);return Standard_True;
    }catch(...){output={};CORE3D_CUT_REFUSE("document-capture.exception", Standard_False);}
}

Standard_Boolean OcctDocument::CaptureCylindricalCutProgramSource(
    const TDF_Label& label,OcctCylindricalCutProgramSource& output)const noexcept {
    output={};if(!NSThread.isMainThread)CORE3D_CUT_REFUSE("document-program-capture.thread", Standard_False);
    try {
        OcctCylindricalCutProgramSource result;OcctScalarAppearanceState appearance;
        if(myOcafDoc.IsNull()||!ValidateGeometryRepresentations()
            ||!CaptureObjectTransformStateForLabel(label,result.original)
            ||result.original.resolvedRepresentation!=OcctGeometryRepresentation::BRep
            ||result.original.authoredFramesPresent||result.original.meshUVAtlasVersion
            ||result.original.shape.ShapeType()!=TopAbs_SOLID
            ||result.original.shape.Orientation()!=TopAbs_FORWARD)CORE3D_CUT_REFUSE("document-program-capture.geometry", Standard_False);
        if(!CaptureScalarAppearanceForSavedCut(label,appearance))CORE3D_CUT_REFUSE("document-program-capture.appearance", Standard_False);
        const auto& state=result.original;double unit=0,scale=0;
        if(!XCAFDoc_DocumentTool::GetLengthUnit(myOcafDoc,unit)
            ||!core3d::cylindrical_cut::EffectiveMM(state.transform,unit,scale,result.effectiveMM))CORE3D_CUT_REFUSE("document-program-capture.units", Standard_False);
        // Whole-program capture requires an existing retained carrier. A bare
        // profile/enclosure/loft source has no program; first cuts stay legacy.
        const unsigned families=unsigned(!state.profile.label.IsNull())+unsigned(!state.enclosure.label.IsNull())
            +unsigned(!state.sweep.label.IsNull())+unsigned(!state.loft.label.IsNull())+unsigned(bool(state.retained.value));
        if(families!=1||!state.retained.value)CORE3D_CUT_REFUSE("document-program-capture.family-census", Standard_False);
        // Encode the complete versioned operand record, including ring count,
        // bolt radius, anchor ratio and all four wedge scalars. Capture never
        // projects a non-cylinder operand into a persisted bore.
        result.recipe=state.retained.value->envelope;result.base=state.retained.value->base;
        if(!core3d::retained_boolean::Encode(result.recipe,result.recipeBytes)
            ||result.recipeBytes!=state.retained.value->bytes
            ||core3d::retained_solid::Bits(core3d::retained_boolean::Identities(result.recipe).metersPerUnit)
                !=core3d::retained_solid::Bits(unit)
            ||result.base.IsNull()||result.base.ShapeType()!=TopAbs_SOLID)CORE3D_CUT_REFUSE("document-program-capture.recipe-or-base", Standard_False);
        TDF_LabelSequence children;XCAFDoc_ShapeTool::GetSubShapes(label,children);
        if(children.Length()>core3d::profile::MaximumLabels)CORE3D_CUT_REFUSE("document-program-capture.subshape-budget", Standard_False);
        for(int i=1;i<=children.Length();++i)if(!children.Value(i).IsEqual(state.retained.label))CORE3D_CUT_REFUSE("document-program-capture.foreign-subshape", Standard_False);
        output=std::move(result);return Standard_True;
    }catch(...){output={};CORE3D_CUT_REFUSE("document-program-capture.exception", Standard_False);}
}

Standard_Boolean OcctDocument::CaptureRetainedFilletAnchors(const TDF_Label& label,
    const std::vector<core3d::retained_fillet::EdgeAnchor>& requested,
    std::vector<core3d::retained_fillet::EdgeAnchor>& captured, core3d::retained_fillet::Outcome* outcome) const noexcept {
    if(outcome)*outcome=core3d::retained_fillet::Outcome::Generic;
    captured.clear();try {
        namespace f=core3d::retained_fillet;OcctCylindricalCutProgramSource source;
        if(requested.size()>f::MaximumAnchors){if(outcome)*outcome=f::Outcome::DeclinedBudget;return Standard_False;}
        if(requested.empty()||!CaptureCylindricalCutProgramSource(label,source))return Standard_False;
        const double mm=core3d::retained_boolean::Identities(source.recipe).metersPerUnit*1000;
        TopTools_IndexedMapOfShape unique;
        for(const auto& anchor:requested){TopoDS_Edge edge;const auto status=f::Resolve(source.original.shape,anchor,mm,edge);
            if(status!=f::Outcome::Built){if(outcome)*outcome=status;CORE3D_CUT_NOTE(f::Reason(status));return Standard_False;}
            if(unique.Contains(edge)){if(outcome)*outcome=f::Outcome::DeclinedAnchorAmbiguous;CORE3D_CUT_NOTE(f::Reason(f::Outcome::DeclinedAnchorAmbiguous));return Standard_False;}unique.Add(edge);
            if(anchor.curveKind==f::CurveKind::Line){BRepAdaptor_Curve curve(edge);
                if(f::Point(anchor).Distance(curve.Value(curve.FirstParameter()))<=1e-4/mm
                    ||f::Point(anchor).Distance(curve.Value(curve.LastParameter()))<=1e-4/mm){
                    if(outcome)*outcome=f::Outcome::DeclinedAnchorNoMatch;CORE3D_CUT_NOTE("fillet.capture-anchor-at-vertex");return Standard_False;}}
        }
        captured=requested;if(outcome)*outcome=f::Outcome::Built;return Standard_True;
    }catch(...){captured.clear();return Standard_False;}
}

core3d::retained_fillet::Candidates OcctDocument::RetainedFilletCandidates(const TDF_Label& label) const noexcept {
    OcctCylindricalCutProgramSource source;
    if(!NSThread.isMainThread||!CaptureCylindricalCutProgramSource(label,source))return {};
    return core3d::retained_fillet::DiscoverCandidates(source.original.shape,source.recipe,source.base);
}

Standard_Boolean OcctDocument::StageCylindricalCutReplacement(
    const OcctObjectTransformState& previous,const TopoDS_Shape& candidate,
    const std::shared_ptr<const core3d::retained_solid::Payload>& payload,
    const std::optional<core3d::retained_boolean::ProgramEdit>& edit,bool debugFailAfterShape)noexcept {

#if DEBUG // Cut475 phase diagnostics only
    Cut475Scope cut475{"stage.preflight"};
#endif // Cut475 phase diagnostics only
    if(!NSThread.isMainThread)CORE3D_CUT_REFUSE("staging.thread", Standard_False);
    try {
        namespace r=core3d::retained_solid;OcctCylindricalCutSource source;std::vector<std::uint8_t> bytes;
        OcctCylindricalCutProgramSource program;const bool wholeProgram=edit.has_value();
        if(myOcafDoc.IsNull()||!myOcafDoc->HasOpenCommand()||!payload
            ||candidate.IsNull()||candidate.ShapeType()!=TopAbs_SOLID||candidate.Orientation()!=TopAbs_FORWARD
            ||ClassifyDefinitionGeometry(candidate,nullptr)!=DefinitionGeometryClass::BRep)CORE3D_CUT_REFUSE("staging.preflight", Standard_False);

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="stage.envelope-transition";
#endif // Cut475 phase diagnostics only
        if(wholeProgram){
            // The exact typed transition is recomputed from the freshly
            // captured original full recipe: append preserves all original
            // source/tool/order bytes plus exactly one newly issued step and
            // the next-ID advance; a ring radius/count edit changes only its
            // addressed field. A candidate shape or encoded payload is no permit.
            if(!std::holds_alternative<core3d::retained_boolean::Program>(payload->envelope)
                ||!CaptureCylindricalCutProgramSource(previous.label,program)
                ||!program.original.IsEqual(previous)
                ||!core3d::sweep_rebuild::SameRawScalars(program.original.scalars,previous.scalars)
                ||!payload->base.IsEqual(program.base)
                ||!core3d::retained_boolean::Encode(payload->envelope,bytes)||bytes!=payload->bytes)CORE3D_CUT_REFUSE("staging.program-capture-or-envelope", Standard_False);
            const auto expected=core3d::retained_boolean::Apply(program.recipe,*edit,program.effectiveMM);
            if(!expected||!expected->changed||expected->oldBytes!=program.recipeBytes
                ||expected->newBytes!=payload->bytes||!expected->selectedOperandID)CORE3D_CUT_REFUSE("staging.program-transition", Standard_False);
        }else{
            if(!CaptureCylindricalCutSource(previous.label,source)||!source.original.IsEqual(previous)
                ||!core3d::sweep_rebuild::SameRawScalars(source.original.scalars,previous.scalars)
                ||!core3d::retained_boolean::Encode(payload->envelope,bytes)||bytes!=payload->bytes)CORE3D_CUT_REFUSE("staging.source-capture-or-envelope", Standard_False);
            const auto* legacy=std::get_if<r::Envelope>(&payload->envelope);if(!legacy
                ||!core3d::saved_cut_source_edit::FirstCutBaseMatches(source.base,payload->base,source.rebuilding,*legacy))CORE3D_CUT_REFUSE("staging.base-proof", Standard_False);
            auto expected=source.envelope;
            if(source.rebuilding){
                if(!core3d::cylindrical_cut::SameFixedEnvelope(expected,*legacy))CORE3D_CUT_REFUSE("staging.fixed-envelope", Standard_False);
            }else{
                expected.derivedFeature=legacy->derivedFeature;expected.operandID=legacy->operandID;
                expected.axis=legacy->axis;expected.point=legacy->point;expected.radius=legacy->radius;
                std::vector<std::uint8_t> actual;if(!r::Encode(expected,actual)||actual!=payload->bytes)CORE3D_CUT_REFUSE("staging.first-cut-envelope", Standard_False);
            }
        }

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="stage.appearance-before";
#endif // Cut475 phase diagnostics only
        OcctScalarAppearanceState appearance;if(!CaptureScalarAppearanceForSavedCut(previous.label,appearance))CORE3D_CUT_REFUSE("staging.appearance-before", Standard_False);
        const auto shapes=XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());if(shapes.IsNull())CORE3D_CUT_REFUSE("staging.shape-tool", Standard_False);
        const auto rebuilding=wholeProgram||source.rebuilding;
        const auto metadata=rebuilding?previous.retained.label:
            !previous.profile.label.IsNull()?previous.profile.label:
            !previous.enclosure.label.IsNull()?previous.enclosure.label:previous.loft.label;
        if(metadata.IsNull()||metadata.Tag()<r::MinimumRecordTag)CORE3D_CUT_REFUSE("staging.metadata", Standard_False);
        // Single ordinary-owned command: preserve original occurrence scalars,
        // material/name/identity labels and pair owner result with the carrier.

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="stage.owner-write";
#endif // Cut475 phase diagnostics only
        shapes->SetShape(previous.label,candidate);
#if DEBUG
        if(debugFailAfterShape)throw Standard_Failure("Cylindrical cut paired-write fault");
#else
        (void)debugFailAfterShape;
#endif

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="stage.metadata-write";
#endif // Cut475 phase diagnostics only
        if(!rebuilding)metadata.ForgetAllAttributes(Standard_True);
        TNaming_Builder(metadata).Select(candidate,candidate);
        Handle(r::Attribute) attribute;
        if(!metadata.FindAttribute(r::AttributeID(),attribute)){attribute=new r::Attribute();metadata.AddAttribute(attribute);}
        attribute->Backup();attribute->value_=payload;

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="stage.readback";
#endif // Cut475 phase diagnostics only
        OcctObjectTransformState stored;OcctScalarAppearanceState after;
        if(!CaptureObjectTransformStateForLabel(previous.label,stored)||!stored.shape.IsEqual(candidate)
            ||!stored.retained.value||stored.retained.value->bytes!=payload->bytes
            ||!stored.retained.value->base.IsEqual(payload->base)||!stored.profile.label.IsNull()||!stored.enclosure.label.IsNull()||!stored.loft.label.IsNull()
            ||stored.entityIdentifier!=previous.entityIdentifier||stored.definitionIdentifier!=previous.definitionIdentifier
            ||stored.present!=previous.present||!core3d::sweep_rebuild::SameRawScalars(stored.scalars,previous.scalars)
            ||!CaptureScalarAppearanceForSavedCut(previous.label,after)||!appearance.IsEqual(after)
            ||!core3d::sweep_rebuild::SameRawScalars(appearance.visualValues,after.visualValues)
            ||!ValidateGeometryRepresentations())CORE3D_CUT_REFUSE("staging.readback", Standard_False);

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase=nullptr;Cut475Trace("stage.success");
#endif // Cut475 phase diagnostics only
        return Standard_True;
    }catch(...){CORE3D_CUT_REFUSE("staging.exception", Standard_False);}
}

Standard_Boolean OcctDocument::StageSavedSweepReplacement(
    const OcctObjectTransformState& previous, const TopoDS_Shape& candidate,
    const core3d::planar_sweep::Definition& definition, bool debugFailAfterShape) noexcept {
    if (![NSThread isMainThread]) return Standard_False;
    try {
        namespace p=core3d::sweep_persistence;
        OcctObjectTransformState current;std::vector<double> values;
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || !CaptureObjectTransformStateForLabel(previous.label,current) || !current.IsEqual(previous)
            || !core3d::sweep_rebuild::SameRawScalars(current.scalars,previous.scalars)
            || current.sweep.label.IsNull() || !current.sweep.IsCurrent(myOcafDoc,previous.label)
            || !core3d::sweep_rebuild::HasOnlyMetadataSubshapes(myOcafDoc,previous.label)
            || !core3d::sweep_rebuild::FixedStructure(current.sweep.definition,definition)
            || !p::Encode(definition,values) || candidate.IsNull() || candidate.ShapeType()!=TopAbs_SOLID
            || current.resolvedRepresentation!=OcctGeometryRepresentation::BRep
            || ClassifyDefinitionGeometry(candidate,nullptr)!=DefinitionGeometryClass::BRep) return Standard_False;
        OcctScalarAppearanceState appearance;
        if (!CaptureScalarAppearanceForSavedSweepRebuild(previous.label,appearance)) return Standard_False;
        const auto shapes=XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (shapes.IsNull()) return Standard_False;
        // No observer/readback/generic transform writer between this owner write
        // and the paired existing-label binding/scalar write. Failure remains
        // with the ordinary command owner; no local best-effort restoration.
        shapes->SetShape(previous.label,candidate);
#if DEBUG
        if (debugFailAfterShape) throw Standard_Failure("Saved sweep paired-write fault");
#else
        (void)debugFailAfterShape;
#endif
        const auto label=previous.sweep.label;
        TNaming_Builder(label).Select(candidate,candidate);
        // Upgrade legacy v1 records in this same undoable command. Identity and label stay fixed.
        TDataStd_Integer::Set(label,p::SchemaID(),p::Schema);
        TDataStd_Integer::Set(label,p::CountID(),int(values.size()));
        // Recreate scalar attributes so +0/-0 writes cannot be elided by numeric Set equality.
        for (std::size_t i=0;i<values.size();++i) {
            const auto child=label.FindChild(int(i)+1,Standard_True);
            child.ForgetAttribute(TDataStd_Real::GetID());TDataStd_Real::Set(child,values[i]);
        }
        OcctObjectTransformState stored;OcctScalarAppearanceState after;
        if (!CaptureObjectTransformStateForLabel(previous.label,stored)
            || !stored.shape.IsEqual(candidate) || !stored.sweep.label.IsEqual(previous.sweep.label)
            || stored.sweep.identifier!=previous.sweep.identifier || !p::SameBits(stored.sweep.values,values)
            || !stored.sweep.IsCurrent(myOcafDoc,previous.label)
            || stored.entityIdentifier!=previous.entityIdentifier || stored.definitionIdentifier!=previous.definitionIdentifier
            || stored.present!=previous.present || !core3d::sweep_rebuild::SameRawScalars(stored.scalars,previous.scalars)
            || !CaptureScalarAppearanceForSavedSweepRebuild(previous.label,after)
            || !appearance.IsEqual(after) || !core3d::sweep_rebuild::SameRawScalars(appearance.visualValues,after.visualValues))
            return Standard_False;
        return Standard_True;
    } catch (...) {return Standard_False;}
}

Standard_Boolean OcctDocument::StageSavedLoftReplacement(
    const OcctObjectTransformState& previous, const TopoDS_Shape& candidate,
    const core3d::rectangular_loft::Definition& definition,
    const core3d::rectangular_loft::StationDimensionEdit& edit,bool debugFailAfterShape) noexcept {
    if (![NSThread isMainThread]) return Standard_False;
    try {
        namespace p=core3d::loft_persistence;
        OcctObjectTransformState current;std::vector<double> values;
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || !CaptureObjectTransformStateForLabel(previous.label,current) || !current.IsEqual(previous)
            || !core3d::sweep_rebuild::SameRawScalars(current.scalars,previous.scalars)
            || current.retained.value || current.loft.label.IsNull() || !current.loft.IsCurrent(myOcafDoc,previous.label)
            || !core3d::loft_rebuild::HasOnlyMetadataSubshapes(myOcafDoc,previous.label)
            || !core3d::loft_rebuild::Matches(current.loft.definition,edit,definition)
            || !p::Encode(definition,values) || candidate.IsNull() || candidate.ShapeType()!=TopAbs_SOLID
            || current.resolvedRepresentation!=OcctGeometryRepresentation::BRep
            || ClassifyDefinitionGeometry(candidate,nullptr)!=DefinitionGeometryClass::BRep) return Standard_False;
        OcctScalarAppearanceState appearance;
        if (!CaptureScalarAppearanceForSavedSweepRebuild(previous.label,appearance)) return Standard_False;
        const auto shapes=XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (shapes.IsNull()) return Standard_False;
        // No observer/readback/generic transform writer between this owner write
        // and the paired existing-label binding/scalar write. Failure remains
        // with the ordinary command owner; no local best-effort restoration.
        shapes->SetShape(previous.label,candidate);
#if DEBUG
        if (debugFailAfterShape) throw Standard_Failure("Saved loft paired-write fault");
#else
        (void)debugFailAfterShape;
#endif
        const auto label=previous.loft.label;
        TNaming_Builder(label).Select(candidate,candidate);
        // Fixed structure means count/schema/identity and label are unchanged.
        // Recreate scalar attributes so +0/-0 writes cannot be elided by numeric Set equality.
        for (std::size_t i=0;i<values.size();++i) {
            const auto child=label.FindChild(int(i)+1,Standard_False);
            if (child.IsNull()) return Standard_False;
            child.ForgetAttribute(TDataStd_Real::GetID());TDataStd_Real::Set(child,values[i]);
        }
        OcctObjectTransformState stored;OcctScalarAppearanceState after;
        if (!CaptureObjectTransformStateForLabel(previous.label,stored)
            || !stored.shape.IsEqual(candidate) || !stored.loft.label.IsEqual(previous.loft.label)
            || stored.loft.identifier!=previous.loft.identifier || !p::SameBits(stored.loft.values,values)
            || !stored.loft.IsCurrent(myOcafDoc,previous.label)
            || stored.entityIdentifier!=previous.entityIdentifier || stored.definitionIdentifier!=previous.definitionIdentifier
            || stored.present!=previous.present || !core3d::sweep_rebuild::SameRawScalars(stored.scalars,previous.scalars)
            || !CaptureScalarAppearanceForSavedSweepRebuild(previous.label,after)
            || !appearance.IsEqual(after) || !core3d::sweep_rebuild::SameRawScalars(appearance.visualValues,after.visualValues))
            return Standard_False;
        return Standard_True;
    } catch (...) {return Standard_False;}
}

Standard_Boolean OcctDocument::SaveObjectTransform(
    const TDF_Label& label, const Handle(AIS_Shape) anAis)
{
    if (![NSThread isMainThread]) { return Standard_False; }
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || label.IsNull() || label.Data() != myOcafDoc->GetData()
            || anAis.IsNull()
            || !IsEditableFreeSimpleDefinitionLabel(label)) {
            return Standard_False;
        }
        const gp_Trsf transform = anAis->LocalTransformation();
        const gp_Quaternion rotation = transform.GetRotation();
        const Standard_Real values[8] = {
            transform.TranslationPart().X(),
            transform.TranslationPart().Y(),
            transform.TranslationPart().Z(),
            rotation.X(), rotation.Y(), rotation.Z(), rotation.W(),
            transform.ScaleFactor(),
        };
        // Validate the whole candidate before changing any attribute. The
        // caller retains transaction ownership if staging later fails.
        for (const Standard_Real value : values) {
            if (!std::isfinite(value)) {
                return Standard_False;
            }
        }
        for (Standard_Integer axis = 0; axis < 3; ++axis) {
            if (std::abs(values[axis])
                > core3d::limits::kMaximumModelCoordinateMagnitude) {
                return Standard_False;
            }
        }
        if (std::abs(values[7])
                <= std::numeric_limits<Standard_Real>::epsilon()
            || !EnsureGeometryRepresentationForMutation(label)) {
            return Standard_False;
        }
        for (Standard_Integer index = 0; index < 8; ++index) {
            TDataStd_Real::Set(label.FindChild(index + 1), values[index]);
        }
        OcctObjectTransformState stored;
        if (!CaptureObjectTransformStateForLabel(label, stored)) {
            return Standard_False;
        }
        for (Standard_Integer index = 0; index < 8; ++index) {
            if (!stored.present[index] || stored.scalars[index] != values[index]) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::SetObjectPositionComponentForLabel(
    const TDF_Label& label,
    const Standard_Integer axis,
    const Standard_Real value)
{
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull() || label.Data() != myOcafDoc->GetData()
        || axis < 0 || axis > 2 || !std::isfinite(value)
        || std::abs(value)
            > core3d::limits::kMaximumModelCoordinateMagnitude
        || !IsEditableFreeSimpleDefinitionLabel(label)) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        const OcctGeometryRepresentation representation =
            StoredGeometryRepresentationForLabel(label);
        const bool isSupportedRepresentation =
            representation == OcctGeometryRepresentation::BRep
            || representation
                == OcctGeometryRepresentation::TriangleMesh
            || (representation
                    == OcctGeometryRepresentation::LegacyUnknown
                && GeometryRepresentationForLabel(label)
                    == OcctGeometryRepresentation::BRep);
        if (!isSupportedRepresentation) {
            return Standard_False;
        }
        const TDF_Label child = label.FindChild(axis + 1);
        if (child.IsNull()) {
            return Standard_False;
        }
        TDataStd_Real::Set(child, value);
        Handle(TDataStd_Real) stored;
        return child.FindAttribute(TDataStd_Real::GetID(), stored)
            && !stored.IsNull()
            && stored->Get() == value;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::SetReferenceAxisForLabel(
    const TDF_Label& label,
    const OcctReferenceAxis& axis)
{
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull() || label.Data() != myOcafDoc->GetData()
        || !IsEditableFreeSimpleDefinitionLabel(label)
        || GeometryRepresentationForLabel(label)
            == OcctGeometryRepresentation::Invalid) {
        return Standard_False;
    }
    return WriteReferenceAxisRecord(label, axis)
        ? Standard_True : Standard_False;
}

Standard_Boolean OcctDocument::ResetReferenceAxisForLabel(
    const TDF_Label& label)
{
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull() || label.Data() != myOcafDoc->GetData()
        || !IsEditableFreeSimpleDefinitionLabel(label)
        || GeometryRepresentationForLabel(label)
            == OcctGeometryRepresentation::Invalid) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        for (const Standard_GUID* anId : ReferenceAxisAttributeIDs()) {
            if (anId != nullptr) {
                label.ForgetAttribute(*anId);
            }
        }
        OcctReferenceAxis aStored;
        return ReadReferenceAxisRecord(label, aStored)
                == OcctReferenceAxisReadState::ImplicitDefault
            && ReferenceAxesMatch(aStored, DefaultReferenceAxis());
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::CopyReferenceAxis(
    const TDF_Label& source,
    const TDF_Label& destination)
{
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || source.IsNull() || destination.IsNull()
        || source.Data() != myOcafDoc->GetData()
        || destination.Data() != myOcafDoc->GetData()
        || !IsEditableFreeSimpleDefinitionLabel(destination)) {
        return Standard_False;
    }
    OcctReferenceAxis anAxis;
    const OcctReferenceAxisReadState aState =
        ReadReferenceAxisForLabel(source, anAxis);
    if (aState == OcctReferenceAxisReadState::Invalid) {
        return Standard_False;
    }
    return aState == OcctReferenceAxisReadState::ImplicitDefault
        ? ResetReferenceAxisForLabel(destination)
        : SetReferenceAxisForLabel(destination, anAxis);
}

Standard_Boolean OcctDocument::CopyReferenceAxisThroughBakedTransform(
    const TDF_Label& source,
    const TDF_Label& destination,
    const gp_Trsf& sourceLocalToDestinationLocal)
{
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || source.IsNull() || destination.IsNull()
        || source.Data() != myOcafDoc->GetData()
        || destination.Data() != myOcafDoc->GetData()
        || !IsEditableFreeSimpleDefinitionLabel(destination)) {
        return Standard_False;
    }
    for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
        for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
            if (!std::isfinite(
                    sourceLocalToDestinationLocal.Value(aRow, aColumn))) {
                return Standard_False;
            }
        }
    }

    OcctReferenceAxis anAxis;
    const OcctReferenceAxisReadState aSourceState =
        ReadReferenceAxisForLabel(source, anAxis);
    if (aSourceState == OcctReferenceAxisReadState::Invalid) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        if (anAxis.pivotSpace == OcctReferenceSpace::Object) {
            anAxis.pivot.Transform(sourceLocalToDestinationLocal);
        }
        if (!IsFiniteBoundedReferencePoint(anAxis.pivot)) {
            return Standard_False;
        }
        if (anAxis.directionSpace == OcctReferenceSpace::Object) {
            gp_Vec aDirection(anAxis.direction);
            aDirection.Transform(sourceLocalToDestinationLocal);
            const Standard_Real aSquaredMagnitude =
                aDirection.SquareMagnitude();
            if (!std::isfinite(aDirection.X())
                || !std::isfinite(aDirection.Y())
                || !std::isfinite(aDirection.Z())
                || !std::isfinite(aSquaredMagnitude)
                || aSquaredMagnitude
                    <= std::numeric_limits<Standard_Real>::epsilon()) {
                return Standard_False;
            }
            anAxis.direction = gp_Dir(aDirection);
        }
        if (!IsFiniteReferenceDirection(anAxis.direction)) {
            return Standard_False;
        }
        if (aSourceState == OcctReferenceAxisReadState::ImplicitDefault
            && ReferenceAxesMatch(anAxis, DefaultReferenceAxis())) {
            return ResetReferenceAxisForLabel(destination);
        }
        return SetReferenceAxisForLabel(destination, anAxis);
    } catch (...) {
        return Standard_False;
    }
}

void OcctDocument::SaveObjectMaterial(Handle(AIS_Shape) object
                                      , const Graphic3d_NameOfMaterial name_of_material) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label;
    Handle(TDataStd_Integer) aCurrentint;
    if(shapeTool->FindShape(object->Shape(), label)) {
        SaveObjectMaterial(label, name_of_material);
    }

}

void OcctDocument::SaveObjectColor(Handle(AIS_Shape) object
                                   , const Quantity_NameOfColor name_of_color) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label;
    Handle(TDataStd_Integer) aCurrentint;
    if(shapeTool->FindShape(object->Shape(), label)) {
        SaveObjectColor(label, name_of_color);
    }
}

void OcctDocument::SaveObjectMaterial(const TDF_Label& label, const Graphic3d_NameOfMaterial name_of_material) {
    if (!EnsureGeometryRepresentationForMutation(label)) {
        return;
    }
    TDataStd_Integer::Set(label.FindChild(11), name_of_material);
}

void OcctDocument::SaveObjectColor(const TDF_Label& label, const Quantity_NameOfColor name_of_color) {
    if (!EnsureGeometryRepresentationForMutation(label)) {
        return;
    }
    TDataStd_Integer::Set(label.FindChild(12), name_of_color);
}

Standard_Boolean OcctDocument::SaveObjectPBRMaterial(
    const TDF_Label& label,
    const XCAFDoc_VisMaterialPBR& material) {
    return SaveObjectPBRMaterials({{
        label, material, Handle(Image_Texture)(), Handle(Image_Texture)()}});
}

Standard_Boolean OcctDocument::SaveObjectPBRMaterial(
    const TDF_Label& label,
    const XCAFDoc_VisMaterialPBR& material,
    const Handle(Image_Texture)& prevalidatedBaseColorTexture) {
    return SaveObjectPBRMaterials({{
        label, material, prevalidatedBaseColorTexture,
        Handle(Image_Texture)()}});
}

Standard_Boolean OcctDocument::SaveObjectPBRMaterial(
    const TDF_Label& label,
    const XCAFDoc_VisMaterialPBR& material,
    const Handle(Image_Texture)& prevalidatedBaseColorTexture,
    const Handle(Image_Texture)& prevalidatedEmissiveTexture) {
    return SaveObjectPBRMaterials({{
        label, material, prevalidatedBaseColorTexture,
        prevalidatedEmissiveTexture}});
}

void OcctDocument::SetMaximumSerializedTextureOccurrenceBytesForTesting(
    const Standard_Size maximumBytes)
{
    myMaximumSerializedTextureOccurrenceBytes = std::min(
        maximumBytes, kMaximumAggregateTextureBytes);
}

void OcctDocument::SetMaximumDecodedTextureResourceBytesForTesting(
    const Standard_Size maximumBytes)
{
    myMaximumDecodedTextureResourceBytes = std::min(
        maximumBytes, kMaximumDecodedTextureBytes);
}

void OcctDocument::SetMaximumVisualMaterialDefinitionsForTesting(
    const Standard_Size maximumDefinitions)
{
    myMaximumVisualMaterialDefinitions = std::min(
        maximumDefinitions,
        static_cast<Standard_Size>(
            kMaximumVisualMaterialDefinitions));
}

// Exact source for a scalar appearance command. Geometry serialization includes
// triangles/normals/UVs; it is a same-process preservation guard, not a released
// canonical geometry or receipt policy. No numeric normalization is performed.
namespace {
using PBRBytes=std::vector<std::uint8_t>;
using PBRDigest=std::array<unsigned char,32>;
struct PBRWriter {
    PBRBytes bytes;
    void integer(std::uint64_t x){for(int i=0;i<8;++i)bytes.push_back((x>>(i*8))&255);}
    void scalar(double x){if(!std::isfinite(x))throw Standard_Failure("Nonfinite appearance");std::uint64_t b;std::memcpy(&b,&x,8);integer(b);}
    void string(const TCollection_AsciiString& s){if(s.Length()>4096)throw Standard_Failure("Appearance string limit");integer(s.Length());bytes.insert(bytes.end(),s.ToCString(),s.ToCString()+s.Length());}
    void name(const TCollection_ExtendedString& s){if(s.Length()>4096)throw Standard_Failure("Appearance name limit");integer(s.Length());for(int i=1;i<=s.Length();++i)integer(s.Value(i));}
    void rgb(const Quantity_Color& c){scalar(c.Red());scalar(c.Green());scalar(c.Blue());}
};
std::string PBRLabelKey(const TDF_Label& l){if(l.IsNull())return {};TCollection_AsciiString s;TDF_Tool::Entry(l,s);return s.ToCString();}
void PBRIntegerAttribute(PBRWriter& w,const TDF_Label& l,const Standard_GUID& id){
    Handle(TDF_Attribute) a;const bool found=!l.IsNull()&&l.FindAttribute(id,a);w.integer(found);
    if(found){auto value=Handle(TDataStd_Integer)::DownCast(a);if(value.IsNull())throw Standard_Failure("Malformed appearance integer");w.integer(static_cast<std::uint64_t>(value->Get()));}
}
class PBRGeometryStream final:public std::streambuf {
    CC_SHA256_CTX context{};std::size_t& aggregate;std::size_t bytes=0;bool good;PBRBytes* captured;
public:
    explicit PBRGeometryStream(std::size_t& total,PBRBytes* copy=nullptr):aggregate(total),good(CC_SHA256_Init(&context)==1),captured(copy){}
    bool finish(PBRDigest& out){return good&&bytes&&CC_SHA256_Final(out.data(),&context)==1;}
protected:
    std::streamsize xsputn(const char* p,std::streamsize n)override{
        if(!good||n<0||std::size_t(n)>8*1024*1024-bytes||std::size_t(n)>128*1024*1024-aggregate){good=false;return 0;}
        good=CC_SHA256_Update(&context,p,static_cast<CC_LONG>(n))==1;
        if(good&&captured)captured->insert(captured->end(),p,p+n);
        bytes+=n;aggregate+=n;return good?n:0;
    }
    int overflow(int c)override{if(c==traits_type::eof())return traits_type::not_eof(c);char b=char(c);return xsputn(&b,1)==1?c:traits_type::eof();}
};
void PBRTexture(PBRWriter& w,const Handle(Image_Texture)& texture,std::size_t& total){
    w.integer(!texture.IsNull());if(texture.IsNull())return;
    const auto& data=texture->DataBuffer();
    // Never read external paths or allow a file-backed alias into immutable proof.
    if(!texture->FilePath().IsEmpty()||data.IsNull()||data->Data()==nullptr||data->Size()==0
        ||data->Size()>kMaximumEmbeddedTextureBytes||data->Size()>kMaximumAggregateTextureBytes-total
        ||texture->TextureId().IsEmpty()||texture->TextureId().Length()>kMaximumPersistentTextureIdentifierBytes)
        throw Standard_Failure("Unsupported appearance payload");
    total+=data->Size();w.string(texture->TextureId());w.string(texture->FilePath());
    w.integer(texture->FileOffset());w.integer(texture->FileLength());w.integer(data->Size());
    PBRDigest digest{};if(!CC_SHA256(data->Data(),static_cast<CC_LONG>(data->Size()),digest.data()))throw Standard_Failure("Texture hash failed");
    w.bytes.insert(w.bytes.end(),digest.begin(),digest.end());
}
PBRBytes PBRMaterialBytes(const Handle(XCAFDoc_VisMaterial)& m,std::size_t& total){
    if(m.IsNull()||m->IsEmpty())throw Standard_Failure("Empty appearance");
    PBRWriter w;const auto& p=m->PbrMaterial();const auto& c=m->CommonMaterial();
    w.integer(m->FaceCulling());w.integer(m->AlphaMode());w.scalar(m->AlphaCutOff());
    w.integer(p.IsDefined);w.rgb(p.BaseColor.GetRGB());w.scalar(p.BaseColor.Alpha());
    for(int i=0;i<3;++i)w.scalar(p.EmissiveFactor[i]);w.scalar(p.Metallic);w.scalar(p.Roughness);w.scalar(p.RefractionIndex);
    w.integer(c.IsDefined);w.rgb(c.AmbientColor);w.rgb(c.DiffuseColor);w.rgb(c.SpecularColor);w.rgb(c.EmissiveColor);w.scalar(c.Shininess);w.scalar(c.Transparency);
    for(const auto& t:{p.BaseColorTexture,p.EmissiveTexture,p.MetallicRoughnessTexture,p.OcclusionTexture,p.NormalTexture,c.DiffuseTexture})PBRTexture(w,t,total);
    return std::move(w.bytes);
}
bool PBRMaterialsExactlyEqual(const Handle(XCAFDoc_VisMaterial)& a,const Handle(XCAFDoc_VisMaterial)& b){std::size_t x=0,y=0;return PBRMaterialBytes(a,x)==PBRMaterialBytes(b,y);}
struct PBRTableEntry {PBRBytes material,attributes;bool operator==(const PBRTableEntry&)const=default;};
struct PBRRootEntry {OcctObjectVisibilityState object;PBRDigest geometry{};PBRBytes raw;
    bool equals(const PBRRootEntry& b)const noexcept{return object.IsEqual(b.object)&&geometry==b.geometry&&raw==b.raw;}};
}
struct OcctPBRScalarState {
    Handle(TDF_Data) data;double metersPerUnit=0;std::string target;
    std::map<std::string,PBRTableEntry> materials;
    std::map<std::string,std::string> links;
    std::map<std::string,PBRBytes> objectMaterialAttributes;
    std::map<std::string,PBRRootEntry> roots;
    OcctSavedGroupState groups;core3d::receipt::Catalog receipts;
    bool equals(const OcctPBRScalarState& b)const noexcept {
        if(data.IsNull()||data!=b.data||target!=b.target||std::memcmp(&metersPerUnit,&b.metersPerUnit,8)
            ||materials!=b.materials||links!=b.links||objectMaterialAttributes!=b.objectMaterialAttributes||roots.size()!=b.roots.size()||!groups.IsEqual(b.groups)||!receipts.matches(b.receipts))return false;
        for(const auto& [k,v]:roots){auto i=b.roots.find(k);if(i==b.roots.end()||!v.equals(i->second))return false;}return true;
    }
};
struct OcctPBRScalarPreparation {
    std::shared_ptr<const OcctPBRScalarState> source;
    OcctPBRScalarPatch patch;TDF_Label target;
    Handle(XCAFDoc_VisMaterial) material; // Newly allocated, private; bytes rechecked before staging.
    PBRBytes materialBytes;std::set<std::string> reclaim;
    bool changed=false;
};

std::shared_ptr<const OcctPBRScalarState> OcctDocument::CapturePBRScalarState(const TDF_Label& target) const noexcept {
    if(![NSThread isMainThread])return {};
    try{
        if(myOcafDoc.IsNull()||target.IsNull()||target.Data()!=myOcafDoc->GetData()
            ||!IsEditableFreeSimpleDefinitionLabel(target)||!XCAFDoc_DocumentTool::CheckShapeTool(myOcafDoc->Main())
            ||!XCAFDoc_DocumentTool::CheckVisMaterialTool(myOcafDoc->Main()))return {};
        auto state=std::make_shared<OcctPBRScalarState>();state->data=myOcafDoc->GetData();state->target=PBRLabelKey(target);
        if(!XCAFDoc_DocumentTool::GetLengthUnit(myOcafDoc,state->metersPerUnit)||!std::isfinite(state->metersPerUnit)||state->metersPerUnit<=0||!CaptureSavedGroups(state->groups))return {};
        const auto receiptStatus=core3d::receipt::Read(myOcafDoc,state->receipts);if(receiptStatus!=core3d::receipt::ReadStatus::Absent&&receiptStatus!=core3d::receipt::ReadStatus::Valid)return {};
        auto tool=XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());TDF_LabelSequence materials;tool->GetMaterials(materials);
        if(materials.Length()>kMaximumVisualMaterialDefinitions)return {};std::size_t textures=0,geometry=0;
        for(int i=1;i<=materials.Length();++i){const auto l=materials.Value(i);PBRTableEntry e;e.material=PBRMaterialBytes(tool->GetMaterial(l),textures);PBRWriter w;
            for(TDF_AttributeIterator it(l);it.More();it.Next()){
                const auto& id=it.Value()->ID();if(id!=XCAFDoc_VisMaterial::GetID()&&id!=TDataStd_Name::GetID()
                    &&id!=OwnedPBRMaterialDefinitionAttributeID()&&id!=XCAFDoc::VisMaterialRefGUID())return {};
            }
            Handle(TDF_Attribute) owned;
            if(l.FindAttribute(OwnedPBRMaterialDefinitionAttributeID(),owned)){auto marker=Handle(TDataStd_Integer)::DownCast(owned);if(marker.IsNull()||marker->Get()!=1)return {};}
            PBRIntegerAttribute(w,l,OwnedPBRMaterialDefinitionAttributeID());Handle(TDF_Attribute) a;
            const bool named=l.FindAttribute(TDataStd_Name::GetID(),a);w.integer(named);
            if(named){auto n=Handle(TDataStd_Name)::DownCast(a);if(n.IsNull())return {};w.name(n->Get());}
            e.attributes=std::move(w.bytes);if(!state->materials.emplace(PBRLabelKey(l),std::move(e)).second)return {};
        }
        // All reference endpoints are protected, including hidden/subshape/imported
        // references. Orphan and malformed tree/link state refuses via native save preflight.
        std::vector<TDF_Label> labels{myOcafDoc->GetData()->Root()};
        for(std::size_t index=0;index<labels.size();++index){if(labels.size()>kMaximumDocumentLabels)return {};const auto l=labels[index];
            for(TDF_ChildIterator child(l,Standard_False);child.More();child.Next()){if(labels.size()>=kMaximumDocumentLabels)return {};labels.push_back(child.Value());}
            Handle(TDF_Attribute) direct;
            if(l.FindAttribute(XCAFDoc_VisMaterial::GetID(),direct)&&!state->materials.contains(PBRLabelKey(l)))return {};
            Handle(TDF_Attribute) reference;const bool referenced=l.FindAttribute(XCAFDoc::VisMaterialRefGUID(),reference);
            auto node=Handle(TDataStd_TreeNode)::DownCast(reference);
            if(referenced&&(node.IsNull()||(!state->materials.contains(PBRLabelKey(l))&&(!node->HasFather()||node->HasFirst()))))return {};
            TDF_Label material;const bool linked=XCAFDoc_VisMaterialTool::GetShapeMaterial(l,material);
            if(linked){if(material.IsNull()||!state->materials.contains(PBRLabelKey(material)))return {};state->links.emplace(PBRLabelKey(l),PBRLabelKey(material));}
        }
        std::map<std::string,std::string> graphLinks;std::size_t edgeCount=0;
        for(int i=1;i<=materials.Length();++i){const auto l=materials.Value(i);Handle(TDataStd_TreeNode) node;
            if(!l.FindAttribute(XCAFDoc::VisMaterialRefGUID(),node))continue;
            if(node.IsNull()||node->HasFather()||node->HasNext())return {};
            for(auto child=node->First();!child.IsNull();child=child->Next()){
                if(++edgeCount>kMaximumDocumentLabels||child->Father()!=node
                    ||!graphLinks.emplace(PBRLabelKey(child->Label()),PBRLabelKey(l)).second)return {};
            }
        }
        if(graphLinks!=state->links)return {};
        TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main())->GetFreeShapes(roots);if(roots.Length()>50000)return {};
        for(int i=1;i<=roots.Length();++i){const auto l=roots.Value(i);PBRRootEntry e;
            if(!CaptureObjectVisibilityStateForLabel(l,e.object))return {};
            PBRGeometryStream stream(geometry);std::ostream out(&stream);out.imbue(std::locale::classic());
            BRepTools::Write(e.object.object.object.shape,out,Standard_True,Standard_True,TopTools_FormatVersion_VERSION_3);
            if(!out.good()||!stream.finish(e.geometry))return {};
            PBRWriter raw;for(double x:e.object.object.object.scalars)raw.scalar(x);
            for(const auto* values:{&e.object.object.object.profile.values,&e.object.object.object.enclosure.values,&e.object.object.object.sweep.values,&e.object.object.object.loft.values}){
                raw.integer(values->size());for(double x:*values)raw.scalar(x);}
            // Reference axis values are outside transform/recipe capture.
            OcctReferenceAxis axis;const auto axisState=ReadReferenceAxisForLabel(l,axis);if(axisState==OcctReferenceAxisReadState::Invalid)return {};
            raw.integer(static_cast<unsigned>(axisState));raw.integer(static_cast<unsigned>(axis.pivotSpace));raw.integer(static_cast<unsigned>(axis.directionSpace));
            for(int n=1;n<=3;++n){raw.scalar(axis.pivot.Coord(n));raw.scalar(axis.direction.Coord(n));}
            e.raw=std::move(raw.bytes);const auto key=PBRLabelKey(l);state->roots.emplace(key,std::move(e));
            const auto linkedMaterial=XCAFDoc_VisMaterialTool::GetShapeMaterial(l);
            for(const auto* id:{&LocalPBRMaterialAttributeID(),&AutoPromotedEmissiveFactorAttributeID()}){
                Handle(TDF_Attribute) a;if(l.FindAttribute(*id,a)){auto value=Handle(TDataStd_Integer)::DownCast(a);
                    if(value.IsNull()||value->Get()!=1||linkedMaterial.IsNull()||!linkedMaterial->HasPbrMaterial())return {};
                }
            }
            if(l.IsAttribute(NormalTextureRecipeAttributeID())&&(linkedMaterial.IsNull()||linkedMaterial->PbrMaterial().NormalTexture.IsNull()
                ||!Core3DValidateNormalTextureBinding(myOcafDoc,l)))return {};
            PBRWriter attributes;PBRIntegerAttribute(attributes,l,LocalPBRMaterialAttributeID());PBRIntegerAttribute(attributes,l,NormalTextureRecipeAttributeID());PBRIntegerAttribute(attributes,l,AutoPromotedEmissiveFactorAttributeID());
            for(int tag:{11,12})PBRIntegerAttribute(attributes,l.FindChild(tag,Standard_False),TDataStd_Integer::GetID());
            state->objectMaterialAttributes.emplace(key,std::move(attributes.bytes));
        }
        if(!state->roots.contains(state->target))return {};return state;
    }catch(...){return {};}
}
bool OcctDocument::PBRScalarStateMatches(const std::shared_ptr<const OcctPBRScalarState>& expected) const noexcept {
    if(myOcafDoc.IsNull()||!expected||expected->data!=myOcafDoc->GetData())return false;
    const auto i=expected->roots.find(expected->target);if(i==expected->roots.end())return false;
    const auto live=CapturePBRScalarState(i->second.object.object.object.label);return live&&live->equals(*expected);
}

// This guard deliberately does not change the scalar/sweep capture contract.
namespace {
struct CutRootEvidence {
    OcctAuthoredFrameRecord frames;
    OcctAuthoredFrameReadState frameState=OcctAuthoredFrameReadState::Invalid;
    PBRDigest retainedBase{};
    PBRBytes retainedEnvelope,stableRaw;
    bool equals(const CutRootEvidence& b)const noexcept {
        return frameState==b.frameState&&frames.archive==b.frames.archive&&frames.identity==b.frames.identity
            &&frames.cornerCount==b.frames.cornerCount&&frames.nativeBytes==b.frames.nativeBytes
            &&retainedBase==b.retainedBase&&retainedEnvelope==b.retainedEnvelope&&stableRaw==b.stableRaw;
    }
};
// Only the selected result shape and its old/new feature slots may differ.
// Raw scalars/axis, appearance, frame/atlas, presence, name and visibility stay exact.
bool CutStableRootEqual(const PBRRootEntry& a,const PBRRootEntry& b,bool firstLoftCut=false) {
    auto left=a.object,right=b.object;
    auto& x=left.object.object;auto& y=right.object.object;
    x.shape=y.shape;x.profile=y.profile;x.enclosure=y.enclosure;x.retained=y.retained;
    if(firstLoftCut){
        // Only the sealing caller may admit this checked source-to-carrier
        // transition; source edits still require exact loft-slot equality.
        if(x.loft.label.IsNull()||!y.loft.label.IsNull())return false;
        x.loft=y.loft;
    }
    return left.IsEqual(right)&&a.object.object.object.sweep.IsEqual(b.object.object.object.sweep)
        &&(firstLoftCut||a.object.object.object.loft.IsEqual(b.object.object.object.loft));
}
}
struct OcctSavedCutSceneState {
    std::shared_ptr<const OcctPBRScalarState> scene;
    std::map<std::string,CutRootEvidence> roots;
    std::map<std::string,PBRBytes> layers,layerGraph;
    bool equals(const OcctSavedCutSceneState& b)const noexcept {

#if DEBUG // Cut475 phase diagnostics only
        if(scene&&b.scene){
            if(!scene->equals(*b.scene))Cut475Trace("match.scene-state-diff");
            for(const auto& [key,old]:scene->roots){
                const auto at=b.scene->roots.find(key);
                if(at==b.scene->roots.end()){Cut475Trace("match.root-missing");continue;}
                const int selected=key==scene->target?1:0;
                if(old.geometry!=at->second.geometry)Cut475Trace("match.current-geometry-diff",selected);
                if(old.raw!=at->second.raw)Cut475Trace("match.root-raw-diff",selected);
                if(!old.object.IsEqual(at->second.object))Cut475Trace("match.root-object-diff",selected);
            }
            for(const auto& [key,old]:roots){
                const auto at=b.roots.find(key);if(at==b.roots.end()){Cut475Trace("match.extra-root-missing");continue;}
                const int selected=key==scene->target?1:0;
                if(old.retainedBase!=at->second.retainedBase)Cut475Trace("match.base-content-diff",selected);
                if(old.retainedEnvelope!=at->second.retainedEnvelope)Cut475Trace("match.envelope-diff",selected);
                if(old.stableRaw!=at->second.stableRaw)Cut475Trace("match.stable-scalars-diff",selected);
                if(old.frames.archive!=at->second.frames.archive)Cut475Trace("match.frame-archive-diff",selected);
            }
        }
#endif // Cut475 phase diagnostics only
        if(!scene||!b.scene||!scene->equals(*b.scene)||layers!=b.layers||layerGraph!=b.layerGraph||roots.size()!=b.roots.size())return false;
        for(const auto& [k,v]:roots){auto i=b.roots.find(k);if(i==b.roots.end()||!v.equals(i->second))return false;}return true;
    }
};
std::shared_ptr<const OcctSavedCutSceneState> OcctDocument::CaptureSavedCutSceneState(const TDF_Label& target) const noexcept {

#if DEBUG // Cut475 phase diagnostics only
    Cut475Scope cut475{"capture.geometry-admission"};
#endif // Cut475 phase diagnostics only
    if(![NSThread isMainThread])return {};
    try {
        if(!ValidateGeometryRepresentations())return {};

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="capture.selected-appearance";
#endif // Cut475 phase diagnostics only
        OcctScalarAppearanceState selected;if(!CaptureScalarAppearanceForSavedCut(target,selected))return {};

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="capture.pbr-scene";
#endif // Cut475 phase diagnostics only
        auto captured=std::make_shared<OcctSavedCutSceneState>();captured->scene=CapturePBRScalarState(target);
        if(!captured->scene)return {};const auto& scene=*captured->scene;

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="capture.frame-budget";
#endif // Cut475 phase diagnostics only
        Standard_Size frameBytes=0;if(!Core3DValidateOwnedFrameUsage(myOcafDoc,frameBytes))return {};
        // Source + candidate each retain at most 64 MiB frame archives. Kernel,
        // renderer and map buffers are not part of this retained-vector bound.
        std::size_t geometry=0,archives=0;

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="capture.root-content";
#endif // Cut475 phase diagnostics only
        for(const auto& [key,entry]:scene.roots) {
            const auto& object=entry.object.object.object;CutRootEvidence e;
            e.frameState=Core3DReadAuthoredFrameOwner(myOcafDoc,object.label,e.frames);
            if(e.frameState==OcctAuthoredFrameReadState::Invalid||e.frames.archive.size()>64U*1024U*1024U-archives)return {};
            archives+=e.frames.archive.size();
            // Re-charge current and retained base streams together. The first
            // existing PBR capture has its own transient counter but owns no streams.
            PBRGeometryStream current(geometry);std::ostream currentOut(&current);currentOut.imbue(std::locale::classic());
            BRepTools::Write(object.shape,currentOut,Standard_True,Standard_True,TopTools_FormatVersion_VERSION_3);
            PBRDigest currentDigest{};if(!currentOut.good()||!current.finish(currentDigest)||currentDigest!=entry.geometry)return {};
            if(object.retained.value){
                e.retainedEnvelope=object.retained.value->bytes;PBRGeometryStream stream(geometry);std::ostream out(&stream);out.imbue(std::locale::classic());
                BRepTools::Write(object.retained.value->base,out,Standard_True,Standard_True,TopTools_FormatVersion_VERSION_3);
                if(!out.good()||!stream.finish(e.retainedBase))return {};
            }
            PBRWriter stable;for(double x:object.scalars)stable.scalar(x);
            OcctReferenceAxis axis;const auto axisState=ReadReferenceAxisForLabel(object.label,axis);if(axisState==OcctReferenceAxisReadState::Invalid)return {};
            stable.integer(static_cast<unsigned>(axisState));stable.integer(static_cast<unsigned>(axis.pivotSpace));stable.integer(static_cast<unsigned>(axis.directionSpace));
            for(int n=1;n<=3;++n){stable.scalar(axis.pivot.Coord(n));stable.scalar(axis.direction.Coord(n));}
            e.stableRaw=std::move(stable.bytes);captured->roots.emplace(key,std::move(e));
        }
        // Whole-object embedded materials only. Per-face/imported ColorTool
        // styling is not silently omitted from a purported full-scene proof.

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="capture.style-links";
#endif // Cut475 phase diagnostics only
        for(const auto& [endpoint,material]:scene.links)if(!scene.roots.contains(endpoint))return {};

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="capture.color-table";
#endif // Cut475 phase diagnostics only
        TDF_LabelSequence colors;
        if(XCAFDoc_DocumentTool::CheckColorTool(myOcafDoc->Main()))XCAFDoc_DocumentTool::ColorTool(myOcafDoc->Main())->GetColors(colors);
        if(!colors.IsEmpty())return {}; // Unbound imported color table state is also unsupported.

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="capture.layers";
#endif // Cut475 phase diagnostics only
        TDF_LabelSequence layerLabels;
        if(XCAFDoc_DocumentTool::CheckLayerTool(myOcafDoc->Main()))XCAFDoc_DocumentTool::LayerTool(myOcafDoc->Main())->GetLayerLabels(layerLabels);
        if(layerLabels.Length()>1024)return {};
        for(int n=1;n<=layerLabels.Length();++n){const auto l=layerLabels.Value(n);PBRWriter w;
            if(l.IsNull()||l.Data()!=scene.data)return {};
            for(TDF_ChildIterator child(l,Standard_True);child.More();child.Next())if(child.Value().HasAttribute())return {};
            for(TDF_AttributeIterator it(l);it.More();it.Next()){
                const auto& id=it.Value()->ID();if(id!=TDataStd_Name::GetID()&&id!=XCAFDoc::InvisibleGUID()&&id!=XCAFDoc::LayerRefGUID())return {};
            }
            Handle(TDataStd_Name) name;if(!l.FindAttribute(TDataStd_Name::GetID(),name)||name.IsNull())return {};w.name(name->Get());
            Handle(TDF_Attribute) invisible;const bool hidden=l.FindAttribute(XCAFDoc::InvisibleGUID(),invisible);
            if(hidden&&Handle(TDataStd_UAttribute)::DownCast(invisible).IsNull())return {};w.integer(hidden);
            if(!captured->layers.emplace(PBRLabelKey(l),std::move(w.bytes)).second)return {};
        }

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="capture.layer-graph";
#endif // Cut475 phase diagnostics only
        std::vector<TDF_Label> labels{myOcafDoc->GetData()->Root()};std::size_t graphEdges=0;
        std::set<std::pair<std::string,std::string>> forward,reverse;
        for(std::size_t n=0;n<labels.size();++n){const auto l=labels[n];const auto key=PBRLabelKey(l);
            for(TDF_ChildIterator c(l,Standard_False);c.More();c.Next()){if(labels.size()>=kMaximumDocumentLabels)return {};labels.push_back(c.Value());}
            for(auto color:{XCAFDoc_ColorGen,XCAFDoc_ColorSurf,XCAFDoc_ColorCurv})if(l.IsAttribute(XCAFDoc::ColorRefGUID(color)))return {};
            if(l.IsAttribute(XCAFDoc::ColorByLayerGUID()))return {};
            Handle(TDF_Attribute) attr;if(!l.FindAttribute(XCAFDoc::LayerRefGUID(),attr))continue;
            const auto graph=Handle(XCAFDoc_GraphNode)::DownCast(attr);const bool layer=captured->layers.contains(key),root=scene.roots.contains(key);
            if(graph.IsNull()||(!layer&&!root)||(layer&&graph->NbFathers()!=0)||(root&&graph->NbChildren()!=0)
                ||graph->NbFathers()<0||graph->NbChildren()<0||graph->NbFathers()>1024||graph->NbChildren()>50000)return {};
            PBRWriter w;w.integer(graph->NbFathers());w.integer(graph->NbChildren());
            for(int i=1;i<=graph->NbFathers();++i){auto father=graph->GetFather(i);if(father.IsNull()||father->Label().IsNull()||father->Label().Data()!=scene.data)return {};
                Handle(XCAFDoc_GraphNode) owned;if(!father->Label().FindAttribute(XCAFDoc::LayerRefGUID(),owned)||owned!=father)return {};const auto parent=PBRLabelKey(father->Label());
                if(++graphEdges>kMaximumDocumentLabels||!captured->layers.contains(parent)||!forward.emplace(parent,key).second)return {};w.string(TCollection_AsciiString(parent.c_str()));}
            for(int i=1;i<=graph->NbChildren();++i){auto child=graph->GetChild(i);if(child.IsNull()||child->Label().IsNull()||child->Label().Data()!=scene.data)return {};
                Handle(XCAFDoc_GraphNode) owned;if(!child->Label().FindAttribute(XCAFDoc::LayerRefGUID(),owned)||owned!=child)return {};const auto endpoint=PBRLabelKey(child->Label());
                if(++graphEdges>kMaximumDocumentLabels||!scene.roots.contains(endpoint)||!reverse.emplace(key,endpoint).second)return {};w.string(TCollection_AsciiString(endpoint.c_str()));}
            captured->layerGraph.emplace(key,std::move(w.bytes));
        }

#if DEBUG // Cut475 phase diagnostics only
        if(forward==reverse)cut475.phase=nullptr;
#endif // Cut475 phase diagnostics only
        if(forward!=reverse)return {};return captured;
    }catch(...){return {};}
}
bool OcctDocument::SavedCutSceneStateMatches(const std::shared_ptr<const OcctSavedCutSceneState>& expected)const noexcept {
    if(!expected||!expected->scene)return false;
    const auto i=expected->scene->roots.find(expected->scene->target);if(i==expected->scene->roots.end())return false;

#if DEBUG // Cut475 phase diagnostics only
    Cut475Trace("match.requested");
#endif // Cut475 phase diagnostics only
    const auto actual=CaptureSavedCutSceneState(i->second.object.object.object.label);return actual&&actual->equals(*expected);
}
bool OcctDocument::SealSavedCutSceneState(const std::shared_ptr<const OcctSavedCutSceneState>& previous,
    const TopoDS_Shape& result,const std::shared_ptr<const core3d::retained_solid::Payload>& payload,
    std::shared_ptr<const OcctSavedCutSceneState>& candidate)const noexcept {

#if DEBUG // Cut475 phase diagnostics only
    Cut475Scope cut475{"seal.preflight"};
#endif // Cut475 phase diagnostics only
    candidate.reset();if(!previous||!previous->scene||!payload||result.IsNull())CORE3D_CUT_REFUSE("seal.preflight", false);
    try {
        const auto& before=*previous->scene;const auto selected=before.roots.find(before.target);if(selected==before.roots.end())CORE3D_CUT_REFUSE("seal.target", false);

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="seal.capture";
#endif // Cut475 phase diagnostics only
        const auto after=CaptureSavedCutSceneState(selected->second.object.object.object.label);if(!after||!after->scene)CORE3D_CUT_REFUSE("seal.capture", false);const auto& now=*after->scene;

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="seal.global-stable";
#endif // Cut475 phase diagnostics only
        if(before.data!=now.data||before.target!=now.target||std::memcmp(&before.metersPerUnit,&now.metersPerUnit,8)
            ||before.materials!=now.materials||before.links!=now.links||before.objectMaterialAttributes!=now.objectMaterialAttributes
            ||!before.groups.IsEqual(now.groups)||!before.receipts.matches(now.receipts)||before.roots.size()!=now.roots.size()
            ||previous->layers!=after->layers||previous->layerGraph!=after->layerGraph)CORE3D_CUT_REFUSE("seal.global-stable", false);

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="seal.root-membership";
#endif // Cut475 phase diagnostics only
        for(const auto& [key,old]:before.roots){auto at=now.roots.find(key);auto extra=after->roots.find(key);auto prior=previous->roots.find(key);
            if(at==now.roots.end()||extra==after->roots.end()||prior==previous->roots.end())CORE3D_CUT_REFUSE("seal.root-membership", false);

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="seal.unrelated-root";
#endif // Cut475 phase diagnostics only
            if(key!=before.target){if(!old.equals(at->second)||!prior->second.equals(extra->second))CORE3D_CUT_REFUSE("seal.unrelated-root", false);continue;}

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="seal.selected-stable";
#endif // Cut475 phase diagnostics only
            const auto& source=old.object.object.object;const auto& actual=at->second.object.object.object;
            const bool firstLoftCut=!source.loft.label.IsNull();
            if(firstLoftCut){
                const auto* envelope=std::get_if<core3d::retained_solid::Envelope>(&payload->envelope);
                if(source.retained.value||!envelope||envelope->sourceFamily!=3
                    ||envelope->sourceSchema!=core3d::loft_persistence::Schema
                    ||!core3d::loft_persistence::SameBits(source.loft.values,envelope->sourceValues)
                    ||core3d::retained_solid::UUIDText(envelope->sourceFeature)!=source.loft.identifier)
                    CORE3D_CUT_REFUSE("seal.loft-source-transition", false);
            }
            if(!actual.shape.IsEqual(result)||!actual.retained.value||actual.retained.value->bytes!=payload->bytes
                ||!actual.retained.value->base.IsEqual(payload->base)||!actual.profile.label.IsNull()||!actual.enclosure.label.IsNull()
                ||!CutStableRootEqual(old,at->second,firstLoftCut)||prior->second.stableRaw!=extra->second.stableRaw
                ||prior->second.frameState!=extra->second.frameState||prior->second.frames.archive!=extra->second.frames.archive)CORE3D_CUT_REFUSE("seal.selected-stable", false);
            // StageCylindricalCutReplacement owns the exact recipe transition.
            // This independently binds the archived source geometry, including
            // mutable shared-TShape contents which handle equality cannot prove.

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase="seal.retained-base-content";
#endif // Cut475 phase diagnostics only
            const auto expectedBase=source.retained.value?prior->second.retainedBase:old.geometry;
            if(extra->second.retainedBase!=expectedBase||extra->second.retainedEnvelope!=payload->bytes)CORE3D_CUT_REFUSE("seal.retained-base-content", false);
        }

#if DEBUG // Cut475 phase diagnostics only
        cut475.phase=nullptr;Cut475Trace("seal.success");
#endif // Cut475 phase diagnostics only
        candidate=after;return true;
    }catch(...){CORE3D_CUT_REFUSE("seal.exception", false);}
}

// External candidate only. Requires SavedCutSourceDetachedWork.hxx and private
// OcctDocument declarations; no ordinary operation calls this yet.
Standard_Boolean OcctDocument::StageSavedCutSourceReplacement(
    const OcctObjectTransformState& previous,
    const core3d::saved_cut_source_edit::Patch& patch,
    const std::shared_ptr<const core3d::SavedCutSourceDetachedResult>& built,
    std::shared_ptr<const core3d::retained_solid::Payload>& staged,
    bool debugFailAfterShape) noexcept {
    staged.reset();if(!NSThread.isMainThread)return Standard_False;
    try {
        namespace r=core3d::retained_solid;
        namespace e=core3d::saved_cut_source_edit;
        OcctCylindricalCutSource source;
        if(myOcafDoc.IsNull()||!myOcafDoc->HasOpenCommand()||!built||built->noChange
            ||!CaptureCylindricalCutSource(previous.label,source)||!source.rebuilding
            ||!source.original.IsEqual(previous)||!previous.retained.value
            ||!core3d::sweep_rebuild::SameRawScalars(source.original.scalars,previous.scalars)
            ||built->newBase.IsNull()||built->newResult.IsNull()
            ||!built->cut.solid.IsEqual(built->newResult)
            ||built->newBase.ShapeType()!=TopAbs_SOLID||built->newBase.Orientation()!=TopAbs_FORWARD
            ||built->newResult.ShapeType()!=TopAbs_SOLID||built->newResult.Orientation()!=TopAbs_FORWARD
            ||ClassifyDefinitionGeometry(built->newBase,nullptr)!=DefinitionGeometryClass::BRep
            ||ClassifyDefinitionGeometry(built->newResult,nullptr)!=DefinitionGeometryClass::BRep)return Standard_False;
        // Reapply the actual declared patch to the recaptured retained bytes.
        // Only native worker construction can issue built; a matching digest
        // alone is never used to classify a caller-supplied shape.
        const std::atomic_bool checking(false);e::Values expected;
        if(!e::PrepareValues(*source.original.retained.value,patch,checking,expected)
            ||!expected.changed||expected.oldBytes!=built->values.oldBytes
            ||expected.newBytes!=built->values.newBytes)return Standard_False;
        std::size_t aggregate=0;
        e::ShapeCommitment oldBase,oldResult,newBase,newResult;
        if(!e::Commit(source.base,checking,aggregate,oldBase)
            ||!e::Commit(source.original.shape,checking,aggregate,oldResult)
            ||!e::Commit(built->newBase,checking,aggregate,newBase)
            ||!e::Commit(built->newResult,checking,aggregate,newResult)
            ||!(oldBase==built->sourceBase)||!(oldResult==built->sourceResult)
            ||!(newBase==built->generatedBase)||!(newResult==built->generatedResult))return Standard_False;
        auto value=std::make_shared<r::Payload>();value->envelope=expected.newEnvelope;
        value->bytes=expected.newBytes;value->base=built->newBase;
        const auto metadata=previous.retained.label;
        if(metadata.IsNull()||metadata.Tag()<r::MinimumRecordTag)return Standard_False;
        Handle(r::Attribute) attribute;
        if(!metadata.FindAttribute(r::AttributeID(),attribute)||attribute.IsNull()
            ||attribute->value()!=source.original.retained.value)return Standard_False;
        OcctScalarAppearanceState appearance;
        if(!CaptureScalarAppearanceForSavedCut(previous.label,appearance))return Standard_False;
        const auto shapes=XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if(shapes.IsNull())return Standard_False;
        // One existing ordinary-owned command pairs owner and retained carrier.
        // A failure is aborted/reconciled by that owner; no local restoration.
        shapes->SetShape(previous.label,built->newResult);
#if DEBUG
        if(debugFailAfterShape)throw Standard_Failure("Saved cut source paired-write fault");
#else
        (void)debugFailAfterShape;
#endif
        TNaming_Builder(metadata).Select(built->newResult,built->newResult);
        attribute->Backup();attribute->value_=value;
        OcctObjectTransformState actual;OcctScalarAppearanceState after;
        if(!CaptureObjectTransformStateForLabel(previous.label,actual)
            ||!actual.shape.IsEqual(built->newResult)||!actual.retained.value
            ||!actual.retained.label.IsEqual(metadata)
            ||actual.retained.value->bytes!=value->bytes
            ||!actual.retained.value->base.IsEqual(value->base)
            ||!actual.profile.label.IsNull()||!actual.enclosure.label.IsNull()
            ||actual.entityIdentifier!=previous.entityIdentifier
            ||actual.definitionIdentifier!=previous.definitionIdentifier
            ||actual.present!=previous.present
            ||!core3d::sweep_rebuild::SameRawScalars(actual.scalars,previous.scalars)
            ||!CaptureScalarAppearanceForSavedCut(previous.label,after)||!appearance.IsEqual(after)
            ||!core3d::sweep_rebuild::SameRawScalars(appearance.visualValues,after.visualValues)
            ||!ValidateGeometryRepresentations())return Standard_False;
        e::ShapeCommitment readBase,readResult;
        if(!e::Commit(actual.retained.value->base,checking,aggregate,readBase)
            ||!e::Commit(actual.shape,checking,aggregate,readResult)
            ||!(readBase==built->generatedBase)||!(readResult==built->generatedResult))return Standard_False;
        staged=std::move(value);return Standard_True;
    }catch(...){return Standard_False;}
}

// Place after OcctSavedCutSceneState/CutRootEvidence definitions. Existing
// SealSavedCutSceneState keeps its radius-only retained-base equality unchanged.
bool OcctDocument::SealSavedCutSourceState(
    const std::shared_ptr<const OcctSavedCutSceneState>& previous,
    const core3d::saved_cut_source_edit::Patch& patch,
    const std::shared_ptr<const core3d::SavedCutSourceDetachedResult>& built,
    const std::shared_ptr<const core3d::retained_solid::Payload>& payload,
    std::shared_ptr<const OcctSavedCutSceneState>& candidate)const noexcept {
    candidate.reset();if(!NSThread.isMainThread||!previous||!previous->scene
        ||!built||built->noChange||!payload)return false;
    try {
        namespace e=core3d::saved_cut_source_edit;
        const auto& before=*previous->scene;
        const auto selected=before.roots.find(before.target);
        if(selected==before.roots.end())return false;
        const auto& original=selected->second.object.object.object;
        if(!original.retained.value||original.retained.value->bytes!=built->values.oldBytes)return false;
        const std::atomic_bool checking(false);e::Values expected;
        if(!e::PrepareValues(*original.retained.value,patch,checking,expected)||!expected.changed
            ||expected.oldBytes!=built->values.oldBytes||expected.newBytes!=built->values.newBytes
            ||payload->bytes!=expected.newBytes||!payload->base.IsEqual(built->newBase))return false;
        std::vector<std::uint8_t> encoded;
        if(!core3d::retained_boolean::Encode(payload->envelope,encoded)||encoded!=payload->bytes)return false;
        const auto after=CaptureSavedCutSceneState(original.label);
        if(!after||!after->scene)return false;const auto& now=*after->scene;
        if(before.data!=now.data||before.target!=now.target||std::memcmp(&before.metersPerUnit,&now.metersPerUnit,8)
            ||before.materials!=now.materials||before.links!=now.links
            ||before.objectMaterialAttributes!=now.objectMaterialAttributes
            ||!before.groups.IsEqual(now.groups)||!before.receipts.matches(now.receipts)
            ||before.roots.size()!=now.roots.size()||previous->roots.size()!=after->roots.size()
            ||previous->layers!=after->layers||previous->layerGraph!=after->layerGraph)return false;
        for(const auto& [key,old]:before.roots){
            const auto at=now.roots.find(key);
            const auto prior=previous->roots.find(key),extra=after->roots.find(key);
            if(at==now.roots.end()||prior==previous->roots.end()||extra==after->roots.end())return false;
            if(key!=before.target){if(!old.equals(at->second)||!prior->second.equals(extra->second))return false;continue;}
            const auto& actual=at->second.object.object.object;
            if(!actual.shape.IsEqual(built->newResult)||!actual.retained.value
                ||!actual.retained.label.IsEqual(original.retained.label)
                ||actual.retained.value->bytes!=payload->bytes
                ||!actual.retained.value->base.IsEqual(built->newBase)
                ||!actual.profile.label.IsNull()||!actual.enclosure.label.IsNull()
                ||!CutStableRootEqual(old,at->second)
                ||prior->second.stableRaw!=extra->second.stableRaw
                ||prior->second.frameState!=extra->second.frameState
                ||prior->second.frames.archive!=extra->second.frames.archive
                ||prior->second.frames.identity!=extra->second.frames.identity
                ||prior->second.frames.cornerCount!=extra->second.frames.cornerCount
                ||prior->second.frames.nativeBytes!=extra->second.frames.nativeBytes
                ||prior->second.retainedEnvelope!=built->values.oldBytes
                ||extra->second.retainedEnvelope!=built->values.newBytes
                ||old.geometry!=built->sourceResult.sha256
                ||prior->second.retainedBase!=built->sourceBase.sha256
                ||at->second.geometry!=built->generatedResult.sha256
                ||extra->second.retainedBase!=built->generatedBase.sha256)return false;
            // Also recheck the old payload's shared TShapes after paired writes.
            // The archived old digest is insufficient if storage mutated in place.
            std::size_t aggregate=0;e::ShapeCommitment oldBase,oldResult,newBase,newResult;
            if(!e::Commit(original.retained.value->base,checking,aggregate,oldBase)
                ||!e::Commit(original.shape,checking,aggregate,oldResult)
                ||!e::Commit(actual.retained.value->base,checking,aggregate,newBase)
                ||!e::Commit(actual.shape,checking,aggregate,newResult)
                ||!(oldBase==built->sourceBase)||!(oldResult==built->sourceResult)
                ||!(newBase==built->generatedBase)||!(newResult==built->generatedResult))return false;
        }
        candidate=after;return true;
    }catch(...){return false;}
}

// Explicit whole-program source stage, additive beside the legacy pair above.
// One existing ordinary-owned command pairs owner shape, retained carrier and
// TNaming binding; failure is aborted/reconciled by that same command owner.
Standard_Boolean OcctDocument::StageSavedProgramSourceReplacement(
    const OcctObjectTransformState& previous,
    const core3d::saved_cut_source_edit::Patch& patch,
    const std::shared_ptr<const core3d::SavedProgramSourceDetachedResult>& built,
    std::shared_ptr<const core3d::retained_solid::Payload>& staged,
    bool debugFailAfterShape) noexcept {
    staged.reset();if(!NSThread.isMainThread)return Standard_False;
    try {
        namespace r=core3d::retained_solid;
        namespace e=core3d::saved_cut_source_edit;
        namespace p=core3d::saved_program_source_edit;
        OcctCylindricalCutProgramSource source;
        if(myOcafDoc.IsNull()||!myOcafDoc->HasOpenCommand()||!built||built->noChange
            ||!CaptureCylindricalCutProgramSource(previous.label,source)
            ||!source.original.IsEqual(previous)||!previous.retained.value
            ||!std::holds_alternative<core3d::retained_boolean::Program>(source.recipe)
            ||!core3d::sweep_rebuild::SameRawScalars(source.original.scalars,previous.scalars)
            ||built->newBase.IsNull()||built->newResult.IsNull()
            ||built->build.status!=core3d::saved_boolean_build::Status::Built
            ||built->build.exactProgram!=built->values.newBytes
            ||!built->build.solid.IsEqual(built->newResult)
            ||built->build.correspondence.classification
                !=core3d::saved_boolean_result::Classification::MatchedOrientedBoundary
            ||built->newBase.ShapeType()!=TopAbs_SOLID||built->newBase.Orientation()!=TopAbs_FORWARD
            ||built->newResult.ShapeType()!=TopAbs_SOLID||built->newResult.Orientation()!=TopAbs_FORWARD
            ||ClassifyDefinitionGeometry(built->newBase,nullptr)!=DefinitionGeometryClass::BRep
            ||ClassifyDefinitionGeometry(built->newResult,nullptr)!=DefinitionGeometryClass::BRep)return Standard_False;
        // Reapply the actual declared patch to the recaptured complete original
        // recipe. Only native worker construction can issue built; a matching
        // digest alone is never used to classify a caller-supplied shape.
        const std::atomic_bool checking(false);p::Values expected;
        if(!core3d::saved_boolean_build::PrepareSourceValues(source.recipe,patch,checking,expected,p::PrepareValues)||!expected.changed
            ||expected.oldBytes!=source.recipeBytes
            ||expected.oldBytes!=built->values.oldBytes
            ||expected.newBytes!=built->values.newBytes)return Standard_False;
        std::size_t aggregate=0;
        e::ShapeCommitment oldBase,oldResult,newBase,newResult;
        if(!e::Commit(source.base,checking,aggregate,oldBase)
            ||!e::Commit(source.original.shape,checking,aggregate,oldResult)
            ||!e::Commit(built->newBase,checking,aggregate,newBase)
            ||!e::Commit(built->newResult,checking,aggregate,newResult)
            ||!(oldBase==built->sourceBase)||!(oldResult==built->sourceResult)
            ||!(newBase==built->generatedBase)||!(newResult==built->generatedResult)
            ||!(built->build.retainedBase==built->generatedBase)
            ||!(built->build.finalResult==built->generatedResult))return Standard_False;
        auto value=std::make_shared<r::Payload>();value->envelope=expected.newProgram;
        value->bytes=expected.newBytes;value->base=built->newBase;
        const auto metadata=previous.retained.label;
        if(metadata.IsNull()||metadata.Tag()<r::MinimumRecordTag)return Standard_False;
        Handle(r::Attribute) attribute;
        if(!metadata.FindAttribute(r::AttributeID(),attribute)||attribute.IsNull()
            ||attribute->value()!=source.original.retained.value)return Standard_False;
        OcctScalarAppearanceState appearance;
        if(!CaptureScalarAppearanceForSavedCut(previous.label,appearance))return Standard_False;
        const auto shapes=XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if(shapes.IsNull())return Standard_False;
        // One existing ordinary-owned command pairs owner and retained carrier.
        // A failure is aborted/reconciled by that owner; no local restoration.
        shapes->SetShape(previous.label,built->newResult);
#if DEBUG
        if(debugFailAfterShape)throw Standard_Failure("Saved program source paired-write fault");
#else
        (void)debugFailAfterShape;
#endif
        TNaming_Builder(metadata).Select(built->newResult,built->newResult);
        attribute->Backup();attribute->value_=value;
        OcctObjectTransformState actual;OcctScalarAppearanceState after;
        if(!CaptureObjectTransformStateForLabel(previous.label,actual)
            ||!actual.shape.IsEqual(built->newResult)||!actual.retained.value
            ||!actual.retained.label.IsEqual(metadata)
            ||!std::holds_alternative<core3d::retained_boolean::Program>(actual.retained.value->envelope)
            ||actual.retained.value->bytes!=value->bytes
            ||!actual.retained.value->base.IsEqual(value->base)
            ||!actual.profile.label.IsNull()||!actual.enclosure.label.IsNull()
            ||actual.entityIdentifier!=previous.entityIdentifier
            ||actual.definitionIdentifier!=previous.definitionIdentifier
            ||actual.present!=previous.present
            ||!core3d::sweep_rebuild::SameRawScalars(actual.scalars,previous.scalars)
            ||!CaptureScalarAppearanceForSavedCut(previous.label,after)||!appearance.IsEqual(after)
            ||!core3d::sweep_rebuild::SameRawScalars(appearance.visualValues,after.visualValues)
            ||!ValidateGeometryRepresentations())return Standard_False;
        e::ShapeCommitment readBase,readResult;
        if(!e::Commit(actual.retained.value->base,checking,aggregate,readBase)
            ||!e::Commit(actual.shape,checking,aggregate,readResult)
            ||!(readBase==built->generatedBase)||!(readResult==built->generatedResult))return Standard_False;
        staged=std::move(value);return Standard_True;
    }catch(...){return Standard_False;}
}

// Whole-program seal: same complete-scene equality as the legacy source seal,
// with the typed transition recomputed from the archived full recipe bytes.
bool OcctDocument::SealSavedProgramSourceState(
    const std::shared_ptr<const OcctSavedCutSceneState>& previous,
    const core3d::saved_cut_source_edit::Patch& patch,
    const std::shared_ptr<const core3d::SavedProgramSourceDetachedResult>& built,
    const std::shared_ptr<const core3d::retained_solid::Payload>& payload,
    std::shared_ptr<const OcctSavedCutSceneState>& candidate)const noexcept {
    candidate.reset();if(!NSThread.isMainThread||!previous||!previous->scene
        ||!built||built->noChange||!payload)return false;
    try {
        namespace e=core3d::saved_cut_source_edit;
        namespace p=core3d::saved_program_source_edit;
        const auto& before=*previous->scene;
        const auto selected=before.roots.find(before.target);
        if(selected==before.roots.end())return false;
        const auto& original=selected->second.object.object.object;
        if(!original.retained.value||original.retained.value->bytes!=built->values.oldBytes
            ||!std::holds_alternative<core3d::retained_boolean::Program>(original.retained.value->envelope))return false;
        const std::atomic_bool checking(false);p::Values expected;
        if(!core3d::saved_boolean_build::PrepareSourceValues(original.retained.value->envelope,patch,checking,expected,p::PrepareValues)||!expected.changed
            ||expected.oldBytes!=built->values.oldBytes||expected.newBytes!=built->values.newBytes
            ||payload->bytes!=expected.newBytes||!payload->base.IsEqual(built->newBase)
            ||!std::holds_alternative<core3d::retained_boolean::Program>(payload->envelope))return false;
        std::vector<std::uint8_t> encoded;
        if(!core3d::retained_boolean::Encode(payload->envelope,encoded)||encoded!=payload->bytes)return false;
        const auto after=CaptureSavedCutSceneState(original.label);
        if(!after||!after->scene)return false;const auto& now=*after->scene;
        if(before.data!=now.data||before.target!=now.target||std::memcmp(&before.metersPerUnit,&now.metersPerUnit,8)
            ||before.materials!=now.materials||before.links!=now.links
            ||before.objectMaterialAttributes!=now.objectMaterialAttributes
            ||!before.groups.IsEqual(now.groups)||!before.receipts.matches(now.receipts)
            ||before.roots.size()!=now.roots.size()||previous->roots.size()!=after->roots.size()
            ||previous->layers!=after->layers||previous->layerGraph!=after->layerGraph)return false;
        for(const auto& [key,old]:before.roots){
            const auto at=now.roots.find(key);
            const auto prior=previous->roots.find(key),extra=after->roots.find(key);
            if(at==now.roots.end()||prior==previous->roots.end()||extra==after->roots.end())return false;
            if(key!=before.target){if(!old.equals(at->second)||!prior->second.equals(extra->second))return false;continue;}
            const auto& actual=at->second.object.object.object;
            if(!actual.shape.IsEqual(built->newResult)||!actual.retained.value
                ||!actual.retained.label.IsEqual(original.retained.label)
                ||actual.retained.value->bytes!=payload->bytes
                ||!actual.retained.value->base.IsEqual(built->newBase)
                ||!actual.profile.label.IsNull()||!actual.enclosure.label.IsNull()
                ||!CutStableRootEqual(old,at->second)
                ||prior->second.stableRaw!=extra->second.stableRaw
                ||prior->second.frameState!=extra->second.frameState
                ||prior->second.frames.archive!=extra->second.frames.archive
                ||prior->second.frames.identity!=extra->second.frames.identity
                ||prior->second.frames.cornerCount!=extra->second.frames.cornerCount
                ||prior->second.frames.nativeBytes!=extra->second.frames.nativeBytes
                ||prior->second.retainedEnvelope!=built->values.oldBytes
                ||extra->second.retainedEnvelope!=built->values.newBytes
                ||old.geometry!=built->sourceResult.sha256
                ||prior->second.retainedBase!=built->sourceBase.sha256
                ||at->second.geometry!=built->generatedResult.sha256
                ||extra->second.retainedBase!=built->generatedBase.sha256)return false;
            // Also recheck the old payload's shared TShapes after paired writes.
            // The archived old digest is insufficient if storage mutated in place.
            std::size_t aggregate=0;e::ShapeCommitment oldBase,oldResult,newBase,newResult;
            if(!e::Commit(original.retained.value->base,checking,aggregate,oldBase)
                ||!e::Commit(original.shape,checking,aggregate,oldResult)
                ||!e::Commit(actual.retained.value->base,checking,aggregate,newBase)
                ||!e::Commit(actual.shape,checking,aggregate,newResult)
                ||!(oldBase==built->sourceBase)||!(oldResult==built->sourceResult)
                ||!(newBase==built->generatedBase)||!(newResult==built->generatedResult))return false;
        }
        candidate=after;return true;
    }catch(...){return false;}
}

namespace {
bool CutPlacementRawEqual(const PBRBytes& original,const PBRBytes& candidate,
    const OcctObjectTransformState& before,const OcctObjectTransformState& after) {
    PBRWriter x,y;for(double value:before.scalars)x.scalar(value);for(double value:after.scalars)y.scalar(value);
    if(x.bytes.size()!=64||y.bytes.size()!=64||original.size()<64||candidate.size()!=original.size()
        ||!std::equal(x.bytes.begin(),x.bytes.end(),original.begin())
        ||!std::equal(y.bytes.begin(),y.bytes.end(),candidate.begin()))return false;
    return std::equal(original.begin()+64,original.end(),candidate.begin()+64);
}
bool CutPlacementStableRootEqual(const PBRRootEntry& before,const PBRRootEntry& after,const gp_Trsf& expected) {
    auto left=before.object;const auto& right=after.object;
    auto& x=left.object.object;const auto& y=right.object.object;
    if(!x.retained.value||!y.retained.value||!x.retained.IsEqual(y.retained)||before.geometry!=after.geometry)return false;
    const auto rotation=expected.GetRotation();
    const std::array<double,8> values{{expected.TranslationPart().X(),expected.TranslationPart().Y(),expected.TranslationPart().Z(),
        rotation.X(),rotation.Y(),rotation.Z(),rotation.W(),expected.ScaleFactor()}};
    for(std::size_t i=0;i<values.size();++i)if(!y.present[i]||y.scalars[i]!=values[i])return false;
    // CaptureObjectTransformStateForLabel derives y.transform from these
    // actual persisted scalars using TryObjectTransformForLabel. Requiring the
    // pre-serialization matrix bitwise here would incorrectly equate a
    // quaternion encode/decode round trip with identity; the eight committed
    // values above are the exact existing SaveObjectTransform contract.
    if(!CutPlacementRawEqual(before.raw,after.raw,x,y))return false;
    // Only these independently checked occurrence fields may differ. Shape,
    // material, reference axis, recipe and retained metadata are not erased.
    x.transform=y.transform;x.scalars=y.scalars;x.present=y.present;
    return left.IsEqual(right);
}
}
bool OcctDocument::SealSavedCutPlacementState(const std::shared_ptr<const OcctSavedCutSceneState>& previous,
    const gp_Trsf& expected,std::shared_ptr<const OcctSavedCutSceneState>& candidate)const noexcept {
    candidate.reset();if(!previous||!previous->scene)return false;
    try {
        const auto& before=*previous->scene;const auto selected=before.roots.find(before.target);if(selected==before.roots.end())return false;
        const auto after=CaptureSavedCutSceneState(selected->second.object.object.object.label);if(!after||!after->scene)return false;const auto& now=*after->scene;
        if(before.data!=now.data||before.target!=now.target||std::memcmp(&before.metersPerUnit,&now.metersPerUnit,8)
            ||before.materials!=now.materials||before.links!=now.links||before.objectMaterialAttributes!=now.objectMaterialAttributes
            ||!before.groups.IsEqual(now.groups)||!before.receipts.matches(now.receipts)||before.roots.size()!=now.roots.size()
            ||previous->layers!=after->layers||previous->layerGraph!=after->layerGraph)return false;
        for(const auto& [key,old]:before.roots) {
            auto at=now.roots.find(key);auto extra=after->roots.find(key);auto prior=previous->roots.find(key);
            if(at==now.roots.end()||extra==after->roots.end()||prior==previous->roots.end())return false;
            if(key!=before.target){if(!old.equals(at->second)||!prior->second.equals(extra->second))return false;continue;}
            const auto& source=old.object.object.object;const auto& actual=at->second.object.object.object;
            double beforeRadius=0,afterRadius=0;
            if(!source.retained.value||!actual.retained.value
                ||std::holds_alternative<core3d::retained_solid::Envelope>(source.retained.value->envelope)
                    !=std::holds_alternative<core3d::retained_solid::Envelope>(actual.retained.value->envelope))return false;
            if(const auto* sourceLegacy=std::get_if<core3d::retained_solid::Envelope>(&source.retained.value->envelope)){
                if(!core3d::cylindrical_cut::OccurrenceRadius(*sourceLegacy,source.transform,beforeRadius)
                    ||!core3d::cylindrical_cut::OccurrenceRadius(std::get<core3d::retained_solid::Envelope>(actual.retained.value->envelope),actual.transform,afterRadius))return false;
            }else{
                // Whole-program occurrence: every operand's physical radius
                // stays in the supported domain; the recipe is never rewritten.
                if(!core3d::retained_boolean::OccurrenceRadiiMM(source.retained.value->envelope,source.transform)
                    ||!core3d::retained_boolean::OccurrenceRadiiMM(actual.retained.value->envelope,actual.transform))return false;
            }
            if(!CutPlacementStableRootEqual(old,at->second,expected)
                ||!CutPlacementRawEqual(prior->second.stableRaw,extra->second.stableRaw,source,actual))return false;
            auto stable=prior->second;stable.stableRaw=extra->second.stableRaw;
            if(!stable.equals(extra->second))return false; // exact base/envelope and complete frame archive
        }
        candidate=after;return true;
    }catch(...){return false;}
}

std::shared_ptr<const OcctPBRScalarPreparation> OcctDocument::PreparePBRScalarPatch(
    const TDF_Label& target,const OcctPBRScalarPatch& patch,bool& changed) const noexcept {
    changed=false;
    if(!patch.IsValid()||![NSThread isMainThread])return {};
    try{
        if(!SupportsScalarPBRMaterialEditingForLabel(target))return {};
        auto source=CapturePBRScalarState(target);if(!source)return {};
        auto prepared=std::make_shared<OcctPBRScalarPreparation>();prepared->source=source;prepared->target=target;prepared->patch=patch;
        auto old=XCAFDoc_VisMaterialTool::GetShapeMaterial(target);
        XCAFDoc_VisMaterialPBR p;
        const bool hasEffective=TryEffectivePBRMaterialForLabel(target,p);
        if(!hasEffective){
            Graphic3d_MaterialAspect legacy(MaterialNameForLabel(target));const auto& basis=legacy.PBRMaterial();
            p.BaseColor=Quantity_ColorRGBA(Quantity_Color(ColorNameForLabel(target)),basis.Alpha());p.EmissiveFactor=basis.Emission();p.Metallic=basis.Metallic();p.Roughness=basis.NormalizedRoughness();p.RefractionIndex=basis.IOR();p.IsDefined=true;
        }
        if(!old.IsNull()&&old->HasPbrMaterial()){
            const auto& stored=old->PbrMaterial();
            if(!hasEffective&&(!patch.baseColorSRGB||!patch.metallic||!patch.roughness))return {};
            if(hasEffective&&!patch.baseColorSRGB){
                const auto& a=stored.BaseColor.GetRGB();const auto& b=p.BaseColor.GetRGB();
                if(a.Red()!=b.Red()||a.Green()!=b.Green()||a.Blue()!=b.Blue())return {};
            }
            p=stored; // Omitted persistent values never take the display round trip.
        }
        const auto original=p;
        if(patch.baseColorSRGB){const auto& rgb=*patch.baseColorSRGB;p.BaseColor.SetRGB(Quantity_Color(rgb[0],rgb[1],rgb[2],Quantity_TOC_sRGB));}
        if(patch.metallic)p.Metallic=static_cast<float>(*patch.metallic);
        if(patch.roughness)p.Roughness=static_cast<float>(*patch.roughness);
        auto equalFloat=[](float a,float b){return std::memcmp(&a,&b,sizeof(a))==0;};
        bool identical=equalFloat(p.Metallic,original.Metallic)&&equalFloat(p.Roughness,original.Roughness);
        for(int i=0;i<3;++i){const double a=(i==0?p.BaseColor.GetRGB().Red():i==1?p.BaseColor.GetRGB().Green():p.BaseColor.GetRGB().Blue()),b=(i==0?original.BaseColor.GetRGB().Red():i==1?original.BaseColor.GetRGB().Green():original.BaseColor.GetRGB().Blue());identical=identical&&std::memcmp(&a,&b,8)==0;}
        prepared->material=CreatePersistedPBRMaterial(p,old);
        if(prepared->material.IsNull())return {};
        // OCCT7.8 ConvertToCommonMaterial mapping, applied only to requested
        // inputs. Existing deliberate Common/PBR disagreement remains elsewhere.
        if(!old.IsNull()){
            auto c=old->CommonMaterial();const auto& derived=prepared->material->CommonMaterial();
            if(patch.baseColorSRGB)c.DiffuseColor=derived.DiffuseColor;
            if(patch.metallic)c.SpecularColor=derived.SpecularColor;
            if(patch.roughness)c.Shininess=derived.Shininess;
            prepared->material->SetCommonMaterial(c);
        }
        std::size_t bytes=0;prepared->materialBytes=PBRMaterialBytes(prepared->material,bytes);
        // Compare actual native persisted candidate, including selectively derived
        // Common fields: equal PBR with a requested mismatched Common field changes.
        if(!old.IsNull()){bytes=0;identical=prepared->materialBytes==PBRMaterialBytes(old,bytes);}
        prepared->changed=!identical;changed=prepared->changed;
        std::vector<TDF_Label> reclaim;
        if(changed&&!CanSaveObjectPBRMaterials({{target,p,p.BaseColorTexture,p.EmissiveTexture}},&reclaim,prepared->material))return {};
        for(const auto& label:reclaim)prepared->reclaim.insert(PBRLabelKey(label));
        return prepared;
    }catch(...){changed=false;return {};}
}
std::shared_ptr<const OcctPBRScalarState> OcctDocument::PBRScalarOriginal(const std::shared_ptr<const OcctPBRScalarPreparation>& p) const noexcept{return p?p->source:nullptr;}
TDF_Label OcctDocument::PBRScalarTarget(const std::shared_ptr<const OcctPBRScalarPreparation>& p) const noexcept{return p?p->target:TDF_Label();}

bool OcctDocument::StagePBRScalarPatch(const std::shared_ptr<const OcctPBRScalarPreparation>& p,
    std::shared_ptr<const OcctPBRScalarState>& sealed) noexcept {
    sealed.reset();
    try{
        if(!p||!p->changed||myOcafDoc.IsNull()||!myOcafDoc->HasOpenCommand()||!PBRScalarStateMatches(p->source))return false;
        std::size_t bytes=0;if(PBRMaterialBytes(p->material,bytes)!=p->materialBytes)return false;
        const auto& material=p->material->PbrMaterial();
        if(!SaveObjectPBRMaterialsImpl({{p->target,material,material.BaseColorTexture,material.EmissiveTexture}},p->material))return false;
        auto after=CapturePBRScalarState(p->target);if(!after)return false;
        const auto& before=*p->source;const auto target=before.target;
        if(after->data!=before.data||after->roots.size()!=before.roots.size()||!after->groups.IsEqual(before.groups)
            ||!after->receipts.matches(before.receipts)||std::memcmp(&after->metersPerUnit,&before.metersPerUnit,8))return false;
        for(const auto& [key,root]:before.roots){auto i=after->roots.find(key);if(i==after->roots.end()||!root.equals(i->second))return false;}
        auto oldLinks=before.links,newLinks=after->links;oldLinks.erase(target);newLinks.erase(target);if(oldLinks!=newLinks)return false;
        auto link=after->links.find(target);if(link==after->links.end())return false;
        auto result=after->materials.find(link->second);if(result==after->materials.end()||result->second.material!=p->materialBytes)return false;
        for(const auto& [key,entry]:before.materials){auto i=after->materials.find(key);
            if(p->reclaim.contains(key)){if(i!=after->materials.end()&&key!=link->second)return false;}
            else if(i==after->materials.end()||!(i->second==entry))return false;
        }
        PBRWriter authoredAttributes;authoredAttributes.integer(1);authoredAttributes.integer(1);authoredAttributes.integer(1);authoredAttributes.name(TCollection_ExtendedString("Shapeyard PBR"));
        for(const auto& [key,entry]:after->materials)if(!before.materials.contains(key)||p->reclaim.contains(key)){
            if(key!=link->second||entry.attributes!=authoredAttributes.bytes)return false;
        }
        auto oldAttributes=before.objectMaterialAttributes,newAttributes=after->objectMaterialAttributes;
        oldAttributes.erase(target);newAttributes.erase(target);if(oldAttributes!=newAttributes)return false;
        // Expected authoring changes are local-PBR=1 and removal of legacy preset
        // scalars. The original normal recipe and emissive ownership remain exact.
        PBRWriter expected;expected.integer(1);expected.integer(1);
        PBRIntegerAttribute(expected,p->target,NormalTextureRecipeAttributeID());PBRIntegerAttribute(expected,p->target,AutoPromotedEmissiveFactorAttributeID());
        expected.integer(0);expected.integer(0);
        if(after->objectMaterialAttributes.at(target)!=expected.bytes)return false;
        // Compare retained middle fields to their pre-command encoding, not merely
        // self-read values. Prefix is local presence+optionalvalue; suffix legacy.
        const auto& oldAttr=before.objectMaterialAttributes.at(target);const auto& newAttr=after->objectMaterialAttributes.at(target);
        auto middle=[](const PBRBytes& b){auto read=[&](std::size_t at){std::uint64_t v=0;for(int i=0;i<8;++i)v|=std::uint64_t(b.at(at+i))<<(i*8);return v;};std::size_t start=8+(read(0)?8:0),end=start;for(int i=0;i<2;++i)end+=8+(read(end)?8:0);return PBRBytes(b.begin()+start,b.begin()+end);};
        if(middle(oldAttr)!=middle(newAttr))return false;
        sealed=std::move(after);return true;
    }catch(...){return false;}
}
#ifdef DEBUG
std::optional<OcctPBRScalarDebugEvidence> OcctDocument::DebugPBRScalarEvidence(
    const TDF_Label& target, bool allowUnboundMaterial) const noexcept {
    try{
        const auto state=CapturePBRScalarState(target);if(!state)return {};
        const auto attributes=state->objectMaterialAttributes.find(state->target);if(attributes==state->objectMaterialAttributes.end())return {};
        OcctPBRScalarDebugEvidence output;output.materialAttributes=attributes->second;
        const auto link=state->links.find(state->target);
        if(link==state->links.end()){
            if(!allowUnboundMaterial)return {};
            // Validated legacy/unbound root: no bound material bytes exist.
        }else{
            const auto material=state->materials.find(link->second);if(material==state->materials.end())return {};
            output.hasMaterialBinding=true;output.material=material->second.material;
        }
        std::size_t diagnosticBytes=0;
        PBRWriter protectedBytes;protectedBytes.string(TCollection_AsciiString(DocumentIdentifier().c_str()));protectedBytes.scalar(state->metersPerUnit);
        for(const auto& [key,entry]:state->roots){const auto& object=entry.object.object.object;
            protectedBytes.string(TCollection_AsciiString(key.c_str()));protectedBytes.string(TCollection_AsciiString(object.entityIdentifier.c_str()));protectedBytes.string(TCollection_AsciiString(object.definitionIdentifier.c_str()));
            protectedBytes.integer(entry.object.object.namePresent);protectedBytes.name(entry.object.object.name);
            protectedBytes.integer(entry.raw.size());protectedBytes.bytes.insert(protectedBytes.bytes.end(),entry.raw.begin(),entry.raw.end());
            for(auto b:object.authoredFramesIdentity)protectedBytes.integer(b);
            protectedBytes.integer(object.meshUVAtlasVersion);for(auto n:object.meshUVAtlasSettings)protectedBytes.integer(n);
            protectedBytes.integer(entry.object.invisibleAttributePresent);protectedBytes.integer(entry.object.layerLinkPresent);
            for(const auto& l:entry.object.layers)protectedBytes.string(TCollection_AsciiString(PBRLabelKey(l).c_str()));
            for(bool b:entry.object.layerInvisibleAttributePresent)protectedBytes.integer(b);
            output.geometry.emplace(key,entry.geometry);
            // DEBUG raw evidence only; identical bounded serializer, no normalization.
            auto& rawGeometry=output.geometryStreams[key];PBRGeometryStream stream(diagnosticBytes,&rawGeometry);
            std::ostream encoded(&stream);encoded.imbue(std::locale::classic());
            BRepTools::Write(object.shape,encoded,Standard_True,Standard_True,TopTools_FormatVersion_VERSION_3);
            PBRDigest digest{};if(!encoded.good()||!stream.finish(digest)||digest!=entry.geometry)return {};
        }
        for(const auto& g:state->groups.groups){protectedBytes.string(TCollection_AsciiString(g.identifier.c_str()));protectedBytes.name(g.name);for(const auto& l:g.members)protectedBytes.string(TCollection_AsciiString(PBRLabelKey(l).c_str()));}
        output.preserved=std::move(protectedBytes.bytes);
        PBRWriter table;for(const auto& [key,entry]:state->materials){table.string(TCollection_AsciiString(key.c_str()));table.integer(entry.material.size());table.bytes.insert(table.bytes.end(),entry.material.begin(),entry.material.end());table.integer(entry.attributes.size());table.bytes.insert(table.bytes.end(),entry.attributes.begin(),entry.attributes.end());}
        for(const auto& [key,value]:state->links){table.string(TCollection_AsciiString(key.c_str()));table.string(TCollection_AsciiString(value.c_str()));}
        output.table=std::move(table.bytes);return output;
    }catch(...){return {};}
}
#endif

Standard_Boolean OcctDocument::CanSaveObjectPBRMaterials(
    const std::vector<OcctPBRMaterialUpdate>& updates,
    std::vector<TDF_Label>* reclaimMaterialLabels,
    const Handle(XCAFDoc_VisMaterial)& scalarMaterial) const
{
    if (!scalarMaterial.IsNull() && updates.size()!=1) return Standard_False;
    if (reclaimMaterialLabels != nullptr) {
        reclaimMaterialLabels->clear();
    }
    if (myOcafDoc.IsNull() || updates.empty()
        || updates.size()
            > static_cast<std::size_t>(
                kMaximumVisualMaterialDefinitions)
        || !XCAFDoc_DocumentTool::CheckVisMaterialTool(
            myOcafDoc->Main())) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterialTool) aTool =
        XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
    if (aTool.IsNull()) {
        return Standard_False;
    }
    const Standard_Size aMaximumDefinitions = std::min(
        myMaximumVisualMaterialDefinitions,
        static_cast<Standard_Size>(
            kMaximumVisualMaterialDefinitions));

    // Reserve the native per-corner derivative for the final free-object
    // material bindings, including hidden objects and unchanged normal maps.
    // 64 bytes includes every admitted source field plus a float4 tangent.
    const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
    if (shapeTool.IsNull()) return Standard_False;
    TDF_LabelSequence roots; shapeTool->GetFreeShapes(roots);
    if (roots.Length() < 0 || roots.Length() > 50000) return Standard_False;
    Standard_Size normalBytes = 0;
    if (!Core3DValidateAuthoredFrameOwners(myOcafDoc, normalBytes)) return Standard_False;
    for (int index = 1; index <= roots.Length(); ++index) {
        const auto& root = roots.Value(index);
        const auto update = std::find_if(updates.begin(), updates.end(),
            [&](const auto& value) { return value.label.IsEqual(root); });
        XCAFDoc_VisMaterialPBR material;
        if (update != updates.end()) material = update->material;
        else if (!TryPBRMaterialForLabel(root, material)) continue;
        if (material.NormalTexture.IsNull()) continue;
        Standard_Size bytes = 0;
        const auto basis = Core3DNormalTextureBasisForLabel(myOcafDoc, root, &bytes);
        if (basis == 0 || (update == updates.end() && Core3DNormalTextureRecipeForLabel(root) != basis)
            || !AddMultipliedWithinLimit(normalBytes, bytes, 1U, 64U * 1024U * 1024U)) return Standard_False;
    }

    struct ExistingDefinition {
        TDF_Label label;
        Handle(XCAFDoc_VisMaterial) material;
        bool reclaim = false;
    };
    TDF_LabelSequence existingLabels;
    aTool->GetMaterials(existingLabels);
    if (existingLabels.Length() < 0
        || existingLabels.Length()
            > kMaximumVisualMaterialDefinitions) {
        return Standard_False;
    }
    std::vector<ExistingDefinition> existingDefinitions;
    TDF_LabelMap tableMaterialLabels;
    existingDefinitions.reserve(
        static_cast<std::size_t>(existingLabels.Length()));
    for (Standard_Integer index = 1;
         index <= existingLabels.Length(); ++index) {
        const TDF_Label& aLabel = existingLabels.Value(index);
        const Handle(XCAFDoc_VisMaterial) aMaterial =
            XCAFDoc_VisMaterialTool::GetMaterial(aLabel);
        if (aLabel.IsNull() || aMaterial.IsNull()) {
            return Standard_False;
        }
        tableMaterialLabels.Add(aLabel);
        existingDefinitions.push_back({aLabel, aMaterial, false});
    }
    const Handle(TDF_Data)& documentData = myOcafDoc->GetData();
    if (documentData.IsNull()) {
        return Standard_False;
    }
    const auto hasUnregisteredDirectMaterial =
        [&](const TDF_Label& label) {
            Handle(XCAFDoc_VisMaterial) directMaterial;
            return !label.IsNull()
                && label.FindAttribute(
                    XCAFDoc_VisMaterial::GetID(), directMaterial)
                && (directMaterial.IsNull()
                    || !tableMaterialLabels.Contains(label));
        };
    if (hasUnregisteredDirectMaterial(documentData->Root())) {
        return Standard_False;
    }
    Standard_Size labelCount = 0;
    for (TDF_ChildIterator iterator(
             documentData->Root(), Standard_True);
         iterator.More(); iterator.Next()) {
        if (++labelCount > kMaximumDocumentLabels
            || hasUnregisteredDirectMaterial(iterator.Value())) {
            return Standard_False;
        }
    }

    struct ProjectedUpdate {
        TDF_Label label;
        Handle(XCAFDoc_VisMaterial) material;
        Standard_Integer previousExisting = -1;
        Standard_Integer targetExisting = -1;
    };
    std::vector<ProjectedUpdate> projectedUpdates;
    projectedUpdates.reserve(updates.size());
    std::vector<Handle(XCAFDoc_VisMaterial)> newDefinitions;

    const auto existingIndexForLabel = [&](const TDF_Label& label) {
        for (std::size_t index = 0;
             index < existingDefinitions.size(); ++index) {
            if (existingDefinitions[index].label.IsEqual(label)) {
                return static_cast<Standard_Integer>(index);
            }
        }
        return Standard_Integer(-1);
    };
    for (const OcctPBRMaterialUpdate& update : updates) {
        if (update.label.IsNull()) {
            return Standard_False;
        }
        for (const ProjectedUpdate& projected : projectedUpdates) {
            if (projected.label.IsEqual(update.label)) {
                return Standard_False;
            }
        }

        TDF_Label aPreviousLabel;
        const Handle(XCAFDoc_VisMaterial) aPreviousMaterial =
            XCAFDoc_VisMaterialTool::GetShapeMaterial(update.label);
        XCAFDoc_VisMaterialTool::GetShapeMaterial(
            update.label, aPreviousLabel);
        const Handle(XCAFDoc_VisMaterial) aCandidate =
            (scalarMaterial.IsNull() ? CreatePersistedPBRMaterial(
                update.material, aPreviousMaterial) : scalarMaterial);
        if (aCandidate.IsNull()) {
            return Standard_False;
        }

        Standard_Integer aTargetExisting = -1;
        for (std::size_t index = 0;
             index < existingDefinitions.size(); ++index) {
            if ((scalarMaterial.IsNull()?existingDefinitions[index].material->IsEqual(aCandidate):PBRMaterialsExactlyEqual(existingDefinitions[index].material,aCandidate))) {
                aTargetExisting =
                    static_cast<Standard_Integer>(index);
                break;
            }
        }
        if (aTargetExisting < 0) {
            bool isAlreadyProjected = false;
            for (const Handle(XCAFDoc_VisMaterial)& projected :
                 newDefinitions) {
                if (!projected.IsNull()
                    && (scalarMaterial.IsNull()?projected->IsEqual(aCandidate):PBRMaterialsExactlyEqual(projected,aCandidate))) {
                    isAlreadyProjected = true;
                    break;
                }
            }
            if (!isAlreadyProjected) {
                newDefinitions.push_back(aCandidate);
            }
        }
        projectedUpdates.push_back({
            update.label,
            aCandidate,
            existingIndexForLabel(aPreviousLabel),
            aTargetExisting});
    }

    const auto updateForLabel = [&](const TDF_Label& label)
        -> const ProjectedUpdate* {
        for (const ProjectedUpdate& update : projectedUpdates) {
            if (update.label.IsEqual(label)) {
                return &update;
            }
        }
        return nullptr;
    };
    for (std::size_t index = 0;
         index < existingDefinitions.size(); ++index) {
        ExistingDefinition& existing = existingDefinitions[index];
        bool wasPreviousDefinition = false;
        bool hasFinalReference = false;
        for (const ProjectedUpdate& update : projectedUpdates) {
            wasPreviousDefinition = wasPreviousDefinition
                || update.previousExisting
                    == static_cast<Standard_Integer>(index);
            hasFinalReference = hasFinalReference
                || update.targetExisting
                    == static_cast<Standard_Integer>(index);
        }
        if (!wasPreviousDefinition
            || !IsOwnedMaterialDefinition(existing.label)) {
            continue;
        }

        Handle(TDataStd_TreeNode) aReferenceRoot;
        if (existing.label.FindAttribute(
                XCAFDoc::VisMaterialRefGUID(), aReferenceRoot)
            && !aReferenceRoot.IsNull()) {
            for (Handle(TDataStd_TreeNode) aReference =
                     aReferenceRoot->First();
                 !aReference.IsNull();
                 aReference = aReference->Next()) {
                const ProjectedUpdate* update =
                    updateForLabel(aReference->Label());
                if (update == nullptr
                    || update->previousExisting
                        != static_cast<Standard_Integer>(index)
                    || update->targetExisting
                        == static_cast<Standard_Integer>(index)) {
                    hasFinalReference = true;
                    break;
                }
            }
        }
        // Remove from the projected definition set only when every extant
        // reference is part of this batch and moves elsewhere. Imported or
        // orphaned/unrelated definitions remain charged exactly as saved.
        existing.reclaim = !hasFinalReference;
    }

    Standard_Size finalDefinitionCount =
        static_cast<Standard_Size>(newDefinitions.size());
    if (finalDefinitionCount > aMaximumDefinitions) {
        return Standard_False;
    }
    for (const ExistingDefinition& existing : existingDefinitions) {
        if (!existing.reclaim) {
            if (finalDefinitionCount
                >= aMaximumDefinitions) {
                return Standard_False;
            }
            ++finalDefinitionCount;
        }
    }

    const Standard_Size aMaximumSerializedBytes = std::min(
        myMaximumSerializedTextureOccurrenceBytes,
        kMaximumAggregateTextureBytes);
    const Standard_Size aMaximumDecodedBytes = std::min(
        myMaximumDecodedTextureResourceBytes,
        kMaximumDecodedTextureBytes);
    Core3DEmbeddedTextureBudgetState aTextureBudget;
    for (const ExistingDefinition& existing : existingDefinitions) {
        if (!existing.reclaim
            && !AccumulateSerializedMaterialTextureOccurrences(
                existing.material,
                aTextureBudget,
                aMaximumSerializedBytes,
                aMaximumDecodedBytes)) {
            return Standard_False;
        }
    }
    for (const Handle(XCAFDoc_VisMaterial)& material :
         newDefinitions) {
        if (!AccumulateSerializedMaterialTextureOccurrences(
                material,
                aTextureBudget,
                aMaximumSerializedBytes,
                aMaximumDecodedBytes)) {
            return Standard_False;
        }
    }
    if (reclaimMaterialLabels != nullptr) {
        reclaimMaterialLabels->reserve(existingDefinitions.size());
        for (const ExistingDefinition& existing : existingDefinitions) {
            if (existing.reclaim) {
                reclaimMaterialLabels->push_back(existing.label);
            }
        }
    }
    return Standard_True;
}

Standard_Boolean OcctDocument::SaveObjectPBRMaterials(
    const std::vector<OcctPBRMaterialUpdate>& updates)
{
    return SaveObjectPBRMaterialsImpl(updates, Handle(XCAFDoc_VisMaterial)());
}
Standard_Boolean OcctDocument::SaveObjectPBRMaterialsImpl(
    const std::vector<OcctPBRMaterialUpdate>& updates,
    const Handle(XCAFDoc_VisMaterial)& scalarMaterial)
{
    if (!scalarMaterial.IsNull() && updates.size()!=1) return Standard_False;
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || updates.empty()) {
        return Standard_False;
    }
    const auto isFiniteUnit = [](const Standard_Real theValue) {
        return std::isfinite(theValue)
            && theValue >= 0.0 && theValue <= 1.0;
    };
    const auto isFiniteNonNegative = [](const Standard_Real theValue) {
        return std::isfinite(theValue) && theValue >= 0.0;
    };
    std::unordered_set<const Image_Texture*> validatedAuthoredTextures;
    const auto validateAuthoredTextureBinding =
        [&validatedAuthoredTextures](
            const Handle(Image_Texture)& texture,
            const Handle(Image_Texture)& prevalidatedTexture) {
            if (texture.IsNull()) {
                return prevalidatedTexture.IsNull();
            }
            if (!prevalidatedTexture.IsNull()
                && texture.get() != prevalidatedTexture.get()) {
                return false;
            }
            return !validatedAuthoredTextures.insert(texture.get()).second
                || Core3DValidateAuthoredTexture(texture);
        };
    for (const OcctPBRMaterialUpdate& update : updates) {
        const XCAFDoc_VisMaterialPBR& material = update.material;
        const Quantity_Color& aBaseColor = material.BaseColor.GetRGB();
        if (update.label.IsNull() || !material.IsDefined
            || !isFiniteUnit(aBaseColor.Red())
            || !isFiniteUnit(aBaseColor.Green())
            || !isFiniteUnit(aBaseColor.Blue())
            || !isFiniteUnit(material.BaseColor.Alpha())
            || !isFiniteUnit(material.Metallic)
            || !isFiniteUnit(material.Roughness)
            || !isFiniteNonNegative(material.EmissiveFactor.x())
            || !isFiniteNonNegative(material.EmissiveFactor.y())
            || !isFiniteNonNegative(material.EmissiveFactor.z())
            || material.EmissiveFactor.x() > kMaximumEmissionFactor
            || material.EmissiveFactor.y() > kMaximumEmissionFactor
            || material.EmissiveFactor.z() > kMaximumEmissionFactor
            || (!material.NormalTexture.IsNull()
                && (!Core3DValidateNumericTexture(material.NormalTexture)
                    || !HasNativeNormalTextureGeometry(update.label)))
            || (!material.MetallicRoughnessTexture.IsNull()
                && !Core3DValidateNumericTexture(material.MetallicRoughnessTexture))
            || (!material.OcclusionTexture.IsNull()
                && !Core3DValidateNumericTexture(material.OcclusionTexture))
            || !validateAuthoredTextureBinding(
                material.BaseColorTexture,
                update.prevalidatedBaseColorTexture)
            || !validateAuthoredTextureBinding(
                material.EmissiveTexture,
                update.prevalidatedEmissiveTexture)
            || !std::isfinite(material.RefractionIndex)
            || material.RefractionIndex < 1.0f
            || material.RefractionIndex > 3.0f) {
            return Standard_False;
        }
    }
    std::vector<TDF_Label> reclaimMaterialLabels;
    if (!CanSaveObjectPBRMaterials(
            updates, &reclaimMaterialLabels, scalarMaterial)) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterialTool) aTool =
        XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
    if (aTool.IsNull()) {
        return Standard_False;
    }
    const Standard_Size aMaximumDefinitions = std::min(
        myMaximumVisualMaterialDefinitions,
        static_cast<Standard_Size>(
            kMaximumVisualMaterialDefinitions));

    struct PreparedUpdate {
        TDF_Label label;
        TDF_Label previousMaterialLabel;
        Handle(XCAFDoc_VisMaterial) material;
    };
    std::vector<PreparedUpdate> preparedUpdates;
    preparedUpdates.reserve(updates.size());
    for (const OcctPBRMaterialUpdate& update : updates) {
        TDF_Label aPreviousMaterialLabel;
        const Handle(XCAFDoc_VisMaterial) aPreviousMaterial =
            XCAFDoc_VisMaterialTool::GetShapeMaterial(update.label);
        XCAFDoc_VisMaterialTool::GetShapeMaterial(
            update.label, aPreviousMaterialLabel);
        const Handle(XCAFDoc_VisMaterial) aMaterial =
            (scalarMaterial.IsNull()?CreatePersistedPBRMaterial(
                update.material, aPreviousMaterial):scalarMaterial);
        if (aMaterial.IsNull()) {
            return Standard_False;
        }
        preparedUpdates.push_back({
            update.label, aPreviousMaterialLabel, aMaterial});
    }
    for (const PreparedUpdate& update : preparedUpdates) {
        const Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        const TopoDS_Shape updateShape = XCAFDoc_ShapeTool::GetShape(update.label);
        const bool exactFaceSubshape = !shapeTool.IsNull()
            && !updateShape.IsNull() && updateShape.ShapeType() == TopAbs_FACE
            && !shapeTool->FindMainShape(updateShape).IsNull();
        if (!(scalarMaterial.IsNull()
                ? (exactFaceSubshape
                    || EnsureGeometryRepresentationForMutation(update.label))
                : ValidateGeometryRepresentationForLabel(update.label))) {
            return Standard_False;
        }
    }

    // Detach the whole batch first. This makes final-state reclaim realizable
    // even when several selected labels share one old definition and the table
    // is already at its cap. Candidate handles above preserve each label's
    // alpha/cutoff/culling before those links are removed.
    for (const PreparedUpdate& update : preparedUpdates) {
        aTool->UnSetShapeMaterial(update.label);
    }
    for (const TDF_Label& reclaimLabel : reclaimMaterialLabels) {
        RemoveUnreferencedOwnedMaterial(aTool, reclaimLabel);
    }

    for (const PreparedUpdate& update : preparedUpdates) {
        TDF_Label aMaterialLabel;
        bool didAddMaterial = false;
        TDF_LabelSequence existingLabels;
        aTool->GetMaterials(existingLabels);
        for (Standard_Integer index = 1;
             index <= existingLabels.Length(); ++index) {
            const Handle(XCAFDoc_VisMaterial) existing =
                XCAFDoc_VisMaterialTool::GetMaterial(
                    existingLabels.Value(index));
            if (!existing.IsNull()
                && (scalarMaterial.IsNull()?existing->IsEqual(update.material):PBRMaterialsExactlyEqual(existing,update.material))) {
                aMaterialLabel = existingLabels.Value(index);
                break;
            }
        }
        if (aMaterialLabel.IsNull()) {
            if (existingLabels.Length() < 0
                || static_cast<Standard_Size>(
                    existingLabels.Length())
                    >= aMaximumDefinitions) {
                return Standard_False;
            }
            aMaterialLabel = aTool->AddMaterial(
                update.material,
                TCollection_AsciiString("Shapeyard PBR"));
            didAddMaterial = !aMaterialLabel.IsNull();
        }
        if (didAddMaterial) {
            TDataStd_Integer::Set(
                aMaterialLabel,
                OwnedPBRMaterialDefinitionAttributeID(),
                1);
        }
        if (aMaterialLabel.IsNull()) {
            return Standard_False;
        }
        aTool->SetShapeMaterial(update.label, aMaterialLabel);
        TDataStd_Integer::Set(
            update.label, LocalPBRMaterialAttributeID(), 1);
        if (!update.material->PbrMaterial().NormalTexture.IsNull()) {
            const auto basis = Core3DNormalTextureBasisForLabel(myOcafDoc, update.label);
            if (basis == 0) return Standard_False;
            TDataStd_Integer::Set(update.label, NormalTextureRecipeAttributeID(), basis);
        } else {
            update.label.ForgetAttribute(NormalTextureRecipeAttributeID());
        }

        // Canonical PBR and legacy preset tags must never compete for
        // precedence.
        for (const Standard_Integer aTag : {11, 12}) {
            const TDF_Label aLegacyLabel =
                update.label.FindChild(aTag, Standard_False);
            if (!aLegacyLabel.IsNull()) {
                aLegacyLabel.ForgetAttribute(
                    TDataStd_Integer::GetID());
            }
        }
    }
    for (const PreparedUpdate& update : preparedUpdates) {
        RemoveUnreferencedOwnedMaterial(
            aTool, update.previousMaterialLabel);
    }
    return Standard_True;
}

Standard_Boolean OcctDocument::ClearObjectVisualMaterial(
    const TDF_Label& label) {
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull()
        || !EnsureGeometryRepresentationForMutation(label)) {
        return Standard_False;
    }
    if (XCAFDoc_DocumentTool::CheckVisMaterialTool(myOcafDoc->Main())) {
        Handle(XCAFDoc_VisMaterialTool) aTool =
            XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
        if (!aTool.IsNull()) {
            TDF_Label aPreviousMaterialLabel;
            XCAFDoc_VisMaterialTool::GetShapeMaterial(
                label, aPreviousMaterialLabel);
            aTool->UnSetShapeMaterial(label);
            RemoveUnreferencedOwnedMaterial(
                aTool, aPreviousMaterialLabel);
        }
    }
    label.ForgetAttribute(LocalPBRMaterialAttributeID());
    label.ForgetAttribute(NormalTextureRecipeAttributeID());
    label.ForgetAttribute(AutoPromotedEmissiveFactorAttributeID());
    return Standard_True;
}

Standard_Boolean OcctDocument::CaptureScalarAppearanceForMeshCopy(
    const TDF_Label& label, OcctScalarAppearanceState& output) const noexcept {
    output={};
    if (![NSThread isMainThread]) return Standard_False;
    try {
        if (myOcafDoc.IsNull() || label.IsNull() || label.Data()!=myOcafDoc->GetData()
            || !IsEditableFreeSimpleDefinitionLabel(label)) return Standard_False;
        // Subshape styling needs a deliberate triangle/material mapping. The
        // first copy rejects these labels rather than flattening their styles.
        if (!core3d::profile::HasOnlyMetadataSubshapes(myOcafDoc, label)) return Standard_False;
        for (auto color:{XCAFDoc_ColorGen,XCAFDoc_ColorSurf,XCAFDoc_ColorCurv})
            if (label.IsAttribute(XCAFDoc::ColorRefGUID(color))) return Standard_False;
        if (label.IsAttribute(NormalTextureRecipeAttributeID())
            || label.IsAttribute(AutoPromotedEmissiveFactorAttributeID())) return Standard_False;
        OcctScalarAppearanceState state;
        for (int i=0;i<2;++i) {
            const auto child=label.FindChild(11+i,Standard_False);
            if (child.IsNull()) continue;
            Handle(TDF_Attribute) attribute;
            if (!child.FindAttribute(TDataStd_Integer::GetID(),attribute)) continue;
            const auto integer=Handle(TDataStd_Integer)::DownCast(attribute);
            if (integer.IsNull()) return Standard_False;
            state.legacyPresent[i]=true;state.legacyValues[i]=integer->Get();
        }
        Handle(TDF_Attribute) marker;
        if (label.FindAttribute(LocalPBRMaterialAttributeID(),marker)) {
            const auto integer=Handle(TDataStd_Integer)::DownCast(marker);
            if (integer.IsNull() || integer->Get()!=1) return Standard_False;
            state.localPBR=true;
        }
        const bool linked=XCAFDoc_VisMaterialTool::GetShapeMaterial(label,state.materialLabel);
        const auto material=XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
        if (linked != !material.IsNull() || linked != !state.materialLabel.IsNull()
            || (state.localPBR && (material.IsNull() || !material->HasPbrMaterial()))) return Standard_False;
        if (!material.IsNull()) {
            if (state.materialLabel.Data()!=myOcafDoc->GetData() || material->IsEmpty()) return Standard_False;
            const auto& p=material->PbrMaterial();const auto& c=material->CommonMaterial();
            // Reject even disabled-model texture handles: the source state is
            // captured completely and no payload aliases enter this contract.
            if (!p.BaseColorTexture.IsNull() || !p.MetallicRoughnessTexture.IsNull()
                || !p.NormalTexture.IsNull() || !p.OcclusionTexture.IsNull()
                || !p.EmissiveTexture.IsNull() || !c.DiffuseTexture.IsNull()) return Standard_False;
            auto& values=state.visualValues;
            values={double(material->FaceCulling()),double(material->AlphaMode()),material->AlphaCutOff(),
                double(p.IsDefined),double(c.IsDefined)};
            const auto rgb=[&values](const Quantity_Color& color) {
                values.push_back(color.Red());values.push_back(color.Green());values.push_back(color.Blue());
            };
            rgb(p.BaseColor.GetRGB());values.push_back(p.BaseColor.Alpha());
            for (int i=0;i<3;++i) values.push_back(p.EmissiveFactor[i]);
            values.push_back(p.Metallic);values.push_back(p.Roughness);values.push_back(p.RefractionIndex);
            rgb(c.AmbientColor);rgb(c.DiffuseColor);rgb(c.SpecularColor);rgb(c.EmissiveColor);
            values.push_back(c.Shininess);values.push_back(c.Transparency);
            for (double value:values) if (!std::isfinite(value)) return Standard_False;
        }
        output=std::move(state);return Standard_True;
    } catch (...) {output={};return Standard_False;}
}

Standard_Boolean OcctDocument::CaptureScalarAppearanceForSavedSweepRebuild(
    const TDF_Label& label, OcctScalarAppearanceState& output) const noexcept {
    output={};
    if (![NSThread isMainThread]) return Standard_False;
    try {
        if (myOcafDoc.IsNull() || label.IsNull() || label.Data()!=myOcafDoc->GetData()
            || !IsEditableFreeSimpleDefinitionLabel(label)) return Standard_False;
        // Subshape styling needs a deliberate triangle/material mapping. The
        // first copy rejects these labels rather than flattening their styles.
        core3d::profile::Record profile;core3d::enclosure::Record enclosure;
        core3d::sweep_persistence::Record sweep;core3d::loft_persistence::Record loft;
        if (!core3d::profile::Read(myOcafDoc,label,profile)
            || !core3d::enclosure::Read(myOcafDoc,label,enclosure)
            || !core3d::sweep_persistence::Read(myOcafDoc,label,sweep)
            || !core3d::loft_persistence::Read(myOcafDoc,label,loft)) return Standard_False;
        // The sweep/loft edit guard captures EVERY scene root, including cut
        // siblings. Their validated retained recipe is metadata, not face styling.
        // CaptureObjectTransformStateForLabel / IsEqual already preserve its
        // payload bytes, base and current binding in the surrounding catalog.
        core3d::retained_solid::Record retained;
        if (!core3d::retained_solid::Read(myOcafDoc,label,retained)) return Standard_False;
        TDF_LabelSequence children;XCAFDoc_ShapeTool::GetSubShapes(label,children);
        if (children.Length()>core3d::profile::MaximumLabels) return Standard_False;
        for (int i=1;i<=children.Length();++i) {
            const auto child=children.Value(i);
            if ((profile.label.IsNull() || !child.IsEqual(profile.label))
                && (enclosure.label.IsNull() || !child.IsEqual(enclosure.label))
                && (sweep.label.IsNull() || !child.IsEqual(sweep.label))
                && (loft.label.IsNull() || !child.IsEqual(loft.label))
                && (retained.label.IsNull() || !child.IsEqual(retained.label))) return Standard_False;
        }
        for (auto color:{XCAFDoc_ColorGen,XCAFDoc_ColorSurf,XCAFDoc_ColorCurv})
            if (label.IsAttribute(XCAFDoc::ColorRefGUID(color))) return Standard_False;
        if (label.IsAttribute(NormalTextureRecipeAttributeID())
            || label.IsAttribute(AutoPromotedEmissiveFactorAttributeID())) return Standard_False;
        OcctScalarAppearanceState state;
        for (int i=0;i<2;++i) {
            const auto child=label.FindChild(11+i,Standard_False);
            if (child.IsNull()) continue;
            Handle(TDF_Attribute) attribute;
            if (!child.FindAttribute(TDataStd_Integer::GetID(),attribute)) continue;
            const auto integer=Handle(TDataStd_Integer)::DownCast(attribute);
            if (integer.IsNull()) return Standard_False;
            state.legacyPresent[i]=true;state.legacyValues[i]=integer->Get();
        }
        Handle(TDF_Attribute) marker;
        if (label.FindAttribute(LocalPBRMaterialAttributeID(),marker)) {
            const auto integer=Handle(TDataStd_Integer)::DownCast(marker);
            if (integer.IsNull() || integer->Get()!=1) return Standard_False;
            state.localPBR=true;
        }
        const bool linked=XCAFDoc_VisMaterialTool::GetShapeMaterial(label,state.materialLabel);
        const auto material=XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
        if (linked != !material.IsNull() || linked != !state.materialLabel.IsNull()
            || (state.localPBR && (material.IsNull() || !material->HasPbrMaterial()))) return Standard_False;
        if (!material.IsNull()) {
            if (state.materialLabel.Data()!=myOcafDoc->GetData() || material->IsEmpty()) return Standard_False;
            const auto& p=material->PbrMaterial();const auto& c=material->CommonMaterial();
            // Reject even disabled-model texture handles: the source state is
            // captured completely and no payload aliases enter this contract.
            if (!p.BaseColorTexture.IsNull() || !p.MetallicRoughnessTexture.IsNull()
                || !p.NormalTexture.IsNull() || !p.OcclusionTexture.IsNull()
                || !p.EmissiveTexture.IsNull() || !c.DiffuseTexture.IsNull()) return Standard_False;
            auto& values=state.visualValues;
            values={double(material->FaceCulling()),double(material->AlphaMode()),material->AlphaCutOff(),
                double(p.IsDefined),double(c.IsDefined)};
            const auto rgb=[&values](const Quantity_Color& color) {
                values.push_back(color.Red());values.push_back(color.Green());values.push_back(color.Blue());
            };
            rgb(p.BaseColor.GetRGB());values.push_back(p.BaseColor.Alpha());
            for (int i=0;i<3;++i) values.push_back(p.EmissiveFactor[i]);
            values.push_back(p.Metallic);values.push_back(p.Roughness);values.push_back(p.RefractionIndex);
            rgb(c.AmbientColor);rgb(c.DiffuseColor);rgb(c.SpecularColor);rgb(c.EmissiveColor);
            values.push_back(c.Shininess);values.push_back(c.Transparency);
            for (double value:values) if (!std::isfinite(value)) return Standard_False;
        }
        output=std::move(state);return Standard_True;
    } catch (...) {output={};return Standard_False;}
}

// Cut-specific appearance capture; both catalogs admit validated retained metadata.
Standard_Boolean OcctDocument::CaptureScalarAppearanceForSavedCut(
    const TDF_Label& label, OcctScalarAppearanceState& output) const noexcept {
    output={};
    if (![NSThread isMainThread]) return Standard_False;
    try {
        if (myOcafDoc.IsNull() || label.IsNull() || label.Data()!=myOcafDoc->GetData()
            || !IsEditableFreeSimpleDefinitionLabel(label)) return Standard_False;
        // Subshape styling needs a deliberate triangle/material mapping. The
        // first copy rejects these labels rather than flattening their styles.
        core3d::profile::Record profile;core3d::enclosure::Record enclosure;
        core3d::sweep_persistence::Record sweep;core3d::loft_persistence::Record loft;
        if (!core3d::profile::Read(myOcafDoc,label,profile)
            || !core3d::enclosure::Read(myOcafDoc,label,enclosure)
            || !core3d::sweep_persistence::Read(myOcafDoc,label,sweep)
            || !core3d::loft_persistence::Read(myOcafDoc,label,loft)) return Standard_False;
        core3d::retained_solid::Record retained;
        if(!core3d::retained_solid::Read(myOcafDoc,label,retained))return Standard_False;
        TDF_LabelSequence children;XCAFDoc_ShapeTool::GetSubShapes(label,children);
        if (children.Length()>core3d::profile::MaximumLabels) return Standard_False;
        for (int i=1;i<=children.Length();++i) {
            const auto child=children.Value(i);
            if ((profile.label.IsNull() || !child.IsEqual(profile.label))
                && (enclosure.label.IsNull() || !child.IsEqual(enclosure.label))
                && (sweep.label.IsNull() || !child.IsEqual(sweep.label))
                && (loft.label.IsNull() || !child.IsEqual(loft.label))
                && (retained.label.IsNull() || !child.IsEqual(retained.label))) return Standard_False;
        }
        for (auto color:{XCAFDoc_ColorGen,XCAFDoc_ColorSurf,XCAFDoc_ColorCurv})
            if (label.IsAttribute(XCAFDoc::ColorRefGUID(color))) return Standard_False;
        if (label.IsAttribute(NormalTextureRecipeAttributeID())
            || label.IsAttribute(AutoPromotedEmissiveFactorAttributeID())) return Standard_False;
        OcctScalarAppearanceState state;
        for (int i=0;i<2;++i) {
            const auto child=label.FindChild(11+i,Standard_False);
            if (child.IsNull()) continue;
            Handle(TDF_Attribute) attribute;
            if (!child.FindAttribute(TDataStd_Integer::GetID(),attribute)) continue;
            const auto integer=Handle(TDataStd_Integer)::DownCast(attribute);
            if (integer.IsNull()) return Standard_False;
            state.legacyPresent[i]=true;state.legacyValues[i]=integer->Get();
        }
        Handle(TDF_Attribute) marker;
        if (label.FindAttribute(LocalPBRMaterialAttributeID(),marker)) {
            const auto integer=Handle(TDataStd_Integer)::DownCast(marker);
            if (integer.IsNull() || integer->Get()!=1) return Standard_False;
            state.localPBR=true;
        }
        const bool linked=XCAFDoc_VisMaterialTool::GetShapeMaterial(label,state.materialLabel);
        const auto material=XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
        if (linked != !material.IsNull() || linked != !state.materialLabel.IsNull()
            || (state.localPBR && (material.IsNull() || !material->HasPbrMaterial()))) return Standard_False;
        if (!material.IsNull()) {
            if (state.materialLabel.Data()!=myOcafDoc->GetData() || material->IsEmpty()) return Standard_False;
            const auto& p=material->PbrMaterial();const auto& c=material->CommonMaterial();
            // Reject even disabled-model texture handles: the source state is
            // captured completely and no payload aliases enter this contract.
            if (!p.BaseColorTexture.IsNull() || !p.MetallicRoughnessTexture.IsNull()
                || !p.NormalTexture.IsNull() || !p.OcclusionTexture.IsNull()
                || !p.EmissiveTexture.IsNull() || !c.DiffuseTexture.IsNull()) return Standard_False;
            auto& values=state.visualValues;
            values={double(material->FaceCulling()),double(material->AlphaMode()),material->AlphaCutOff(),
                double(p.IsDefined),double(c.IsDefined)};
            const auto rgb=[&values](const Quantity_Color& color) {
                values.push_back(color.Red());values.push_back(color.Green());values.push_back(color.Blue());
            };
            rgb(p.BaseColor.GetRGB());values.push_back(p.BaseColor.Alpha());
            for (int i=0;i<3;++i) values.push_back(p.EmissiveFactor[i]);
            values.push_back(p.Metallic);values.push_back(p.Roughness);values.push_back(p.RefractionIndex);
            rgb(c.AmbientColor);rgb(c.DiffuseColor);rgb(c.SpecularColor);rgb(c.EmissiveColor);
            values.push_back(c.Shininess);values.push_back(c.Transparency);
            for (double value:values) if (!std::isfinite(value)) return Standard_False;
        }
        output=std::move(state);return Standard_True;
    } catch (...) {output={};return Standard_False;}
}

Standard_Boolean OcctDocument::CopyObjectAppearance(
    const TDF_Label& source,
    const TDF_Label& destination) {
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || source.IsNull() || destination.IsNull()
        || GeometryRepresentationForLabel(source)
            == OcctGeometryRepresentation::Invalid
        || !EnsureGeometryRepresentationForMutation(destination)) {
        return Standard_False;
    }

    Handle(TDataStd_Integer) aLocalPBRMarker;
    const Standard_Boolean hasLocalPBR =
        source.FindAttribute(
            LocalPBRMaterialAttributeID(), aLocalPBRMarker)
        && !aLocalPBRMarker.IsNull()
        && aLocalPBRMarker->Get() == 1;
    const auto sourceVisual = XCAFDoc_VisMaterialTool::GetShapeMaterial(source);
    const bool hasOwnedNormal = hasLocalPBR && !sourceVisual.IsNull()
        && sourceVisual->HasPbrMaterial() && !sourceVisual->PbrMaterial().NormalTexture.IsNull();
    const auto normalRecipe = Core3DNormalTextureRecipeForLabel(source);
    if ((hasOwnedNormal && (!Core3DValidateNormalTextureBinding(myOcafDoc, source)
            || Core3DNormalTextureBasisForLabel(myOcafDoc, destination) != normalRecipe))
        || (!hasOwnedNormal && normalRecipe != 0)) return Standard_False;
    const Standard_Boolean hasAutoPromotedEmissiveFactor =
        IsEmissiveTextureFactorAutoPromotedForLabel(source);
    Graphic3d_NameOfMaterial aLegacyMaterial;
    Quantity_NameOfColor aLegacyColor;
    const Standard_Boolean hasLegacyMaterial =
        TryMaterialNameForLabel(source, aLegacyMaterial);
    const Standard_Boolean hasLegacyColor =
        TryColorNameForLabel(source, aLegacyColor);
    const auto clearDestinationLegacyAppearance = [&destination]() {
        for (const Standard_Integer aTag : {11, 12}) {
            const TDF_Label aLegacyLabel =
                destination.FindChild(aTag, Standard_False);
            if (!aLegacyLabel.IsNull()) {
                aLegacyLabel.ForgetAttribute(TDataStd_Integer::GetID());
            }
        }
    };
    clearDestinationLegacyAppearance();

    TDF_Label aVisualMaterialLabel;
    const Standard_Boolean hasVisualMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(
            source, aVisualMaterialLabel)
        && !aVisualMaterialLabel.IsNull();
    if (hasVisualMaterial) {
        Handle(XCAFDoc_VisMaterialTool) aTool =
            XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
        if (aTool.IsNull()) {
            return Standard_False;
        }
        TDF_Label aPreviousDestinationMaterialLabel;
        XCAFDoc_VisMaterialTool::GetShapeMaterial(
            destination, aPreviousDestinationMaterialLabel);
        aTool->SetShapeMaterial(destination, aVisualMaterialLabel);
        if (!aPreviousDestinationMaterialLabel.IsNull()
            && !aPreviousDestinationMaterialLabel.IsEqual(
                aVisualMaterialLabel)) {
            RemoveUnreferencedOwnedMaterial(
                aTool, aPreviousDestinationMaterialLabel);
        }
        if (hasLocalPBR) {
            TDataStd_Integer::Set(
                destination, LocalPBRMaterialAttributeID(), 1);
        } else {
            destination.ForgetAttribute(LocalPBRMaterialAttributeID());
        }
        if (hasOwnedNormal) {
            TDataStd_Integer::Set(destination, NormalTextureRecipeAttributeID(), normalRecipe);
        } else {
            destination.ForgetAttribute(NormalTextureRecipeAttributeID());
        }
        if (hasLocalPBR && hasAutoPromotedEmissiveFactor) {
            TDataStd_Integer::Set(
                destination,
                AutoPromotedEmissiveFactorAttributeID(),
                1);
        } else {
            destination.ForgetAttribute(
                AutoPromotedEmissiveFactorAttributeID());
        }
        if (hasLocalPBR) {
            return Standard_True;
        }
        if (hasLegacyMaterial) {
            SaveObjectMaterial(destination, aLegacyMaterial);
        }
        if (hasLegacyColor) {
            SaveObjectColor(destination, aLegacyColor);
        }
        return Standard_True;
    }

    if (!ClearObjectVisualMaterial(destination)) {
        return Standard_False;
    }
    // Preserve attribute absence as well as attribute values. Filling missing
    // children with defaults can change a preset-only source's effective base
    // color or make future precedence checks treat it as explicitly styled.
    if (hasLegacyMaterial) {
        SaveObjectMaterial(destination, aLegacyMaterial);
    }
    if (hasLegacyColor) {
        SaveObjectColor(destination, aLegacyColor);
    }
    return Standard_True;
}

Graphic3d_NameOfMaterial OcctDocument::MaterialNameForShape(Handle(AIS_Shape) object) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label;
    if(shapeTool->FindShape(object->Shape(), label)) {
        return MaterialNameForLabel(label);
    }
    return Graphic3d_NameOfMaterial_UserDefined;
}

Graphic3d_NameOfMaterial OcctDocument::MaterialNameForLabel(const TDF_Label& label) const {
    Graphic3d_NameOfMaterial material;
    return TryMaterialNameForLabel(label, material)
        ? material
        : Graphic3d_NameOfMaterial_ShinyPlastified;
}

Standard_Boolean OcctDocument::TryMaterialNameForLabel(
    const TDF_Label& label,
    Graphic3d_NameOfMaterial& material) const {
    Handle(TDataStd_Integer) attribute;
    const TDF_Label materialLabel = label.IsNull()
        ? TDF_Label()
        : label.FindChild(11, Standard_False);
    if (!materialLabel.IsNull()
        && materialLabel.FindAttribute(TDataStd_Integer::GetID(), attribute)
        && !attribute.IsNull()) {
        material = static_cast<Graphic3d_NameOfMaterial>(attribute->Get());
        return Standard_True;
    }
    return Standard_False;
}

Quantity_NameOfColor OcctDocument::ColorNameForLabel(const TDF_Label& label) const {
    Quantity_NameOfColor color;
    return TryColorNameForLabel(label, color)
        ? color
        : Quantity_NOC_GRAY80;
}

Standard_Boolean OcctDocument::TryColorNameForLabel(
    const TDF_Label& label,
    Quantity_NameOfColor& color) const {
    Handle(TDataStd_Integer) attribute;
    const TDF_Label colorLabel = label.IsNull()
        ? TDF_Label()
        : label.FindChild(12, Standard_False);
    if (!colorLabel.IsNull()
        && colorLabel.FindAttribute(TDataStd_Integer::GetID(), attribute)
        && !attribute.IsNull()) {
        color = static_cast<Quantity_NameOfColor>(attribute->Get());
        return Standard_True;
    }
    return Standard_False;
}

Standard_Boolean OcctDocument::TryPBRMaterialForLabel(
    const TDF_Label& label,
    XCAFDoc_VisMaterialPBR& material) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    Handle(TDataStd_Integer) aMarker;
    if (!label.FindAttribute(LocalPBRMaterialAttributeID(), aMarker)
        || aMarker.IsNull() || aMarker->Get() != 1) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterial) aMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (aMaterial.IsNull() || !aMaterial->HasPbrMaterial()) {
        return Standard_False;
    }
    material = aMaterial->PbrMaterial();
    return material.IsDefined;
}

Standard_Boolean OcctDocument::TryEffectivePBRMaterialForLabel(
    const TDF_Label& label,
    XCAFDoc_VisMaterialPBR& material) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    Handle(TDataStd_Integer) aMarker;
    const Standard_Boolean hasLocalPBR =
        label.FindAttribute(LocalPBRMaterialAttributeID(), aMarker)
        && !aMarker.IsNull() && aMarker->Get() == 1;
    Graphic3d_NameOfMaterial aLegacyMaterial;
    Quantity_NameOfColor aLegacyColor;
    const Standard_Boolean hasLegacyMaterial =
        TryMaterialNameForLabel(label, aLegacyMaterial);
    const Standard_Boolean hasLegacyColor =
        TryColorNameForLabel(label, aLegacyColor);
    if (!hasLocalPBR && hasLegacyMaterial) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterial) aVisualMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (aVisualMaterial.IsNull()) {
        return Standard_False;
    }
    if (aVisualMaterial->HasPbrMaterial()) {
        material = aVisualMaterial->PbrMaterial();
    } else if (aVisualMaterial->HasCommonMaterial()) {
        material = aVisualMaterial->ConvertToPbrMaterial();
    } else {
        return Standard_False;
    }
    // Imported assets may place their sole base-color map in the Common
    // compatibility representation while keeping authoritative PBR scalars.
    // Surface that as one effective base texture for selection/replacement;
    // SupportsBaseColorTextureEditingForLabel() separately rejects a genuine
    // PBR/Common conflict when both representations provide different maps.
    if (material.BaseColorTexture.IsNull()
        && aVisualMaterial->HasCommonMaterial()
        && !aVisualMaterial->CommonMaterial().DiffuseTexture.IsNull()) {
        material.BaseColorTexture =
            aVisualMaterial->CommonMaterial().DiffuseTexture;
    }
    if (!hasLocalPBR && hasLegacyColor) {
        material.BaseColor = Quantity_ColorRGBA(
            Quantity_Color(aLegacyColor), material.BaseColor.Alpha());
    }
    return material.IsDefined;
}

Standard_Boolean OcctDocument::SupportsScalarPBRMaterialEditingForLabel(
    const TDF_Label& label) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterial) material =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (material.IsNull()) {
        return Standard_True;
    }
    Handle(TDataStd_Integer) marker;
    const Standard_Boolean hasLocalPBR =
        label.FindAttribute(LocalPBRMaterialAttributeID(), marker)
        && !marker.IsNull() && marker->Get() == 1;
    if (material->HasPbrMaterial()) {
        const XCAFDoc_VisMaterialPBR& pbr = material->PbrMaterial();
        if (!pbr.NormalTexture.IsNull() && (!hasLocalPBR
            || !Core3DValidateNormalTextureBinding(myOcafDoc, label))) return Standard_False;
        const Handle(Image_Texture)& base = pbr.BaseColorTexture;
        const Handle(Image_Texture) common = material->HasCommonMaterial()
            ? material->CommonMaterial().DiffuseTexture
            : Handle(Image_Texture)();
        if (!base.IsNull() || !pbr.EmissiveTexture.IsNull()
            || !pbr.MetallicRoughnessTexture.IsNull() || !pbr.OcclusionTexture.IsNull()
            || !pbr.NormalTexture.IsNull()) {
            return hasLocalPBR
                && base.IsNull() == common.IsNull()
                && (base.IsNull()
                    || Core3DTexturesMatch(base, common));
        }
        return common.IsNull();
    }
    return !material->HasCommonMaterial()
        || material->CommonMaterial().DiffuseTexture.IsNull();
}

Standard_Boolean OcctDocument::SupportsBaseColorTextureEditingForLabel(
    const TDF_Label& label) const {
    return SupportsMaterialTextureEditingForLabel(label, OcctMaterialTextureSlot::BaseColor);
}

Standard_Boolean OcctDocument::SupportsEmissiveTextureEditingForLabel(
    const TDF_Label& label) const {
    return SupportsMaterialTextureEditingForLabel(label, OcctMaterialTextureSlot::Emissive);
}

Standard_Boolean OcctDocument::SupportsMaterialTextureEditingForLabel(
    const TDF_Label& label, OcctMaterialTextureSlot slot) const {
    if (label.IsNull() || (slot == OcctMaterialTextureSlot::Normal
        && !SupportsNormalTextureGeometryForLabel(label))) return Standard_False;
    const Handle(XCAFDoc_VisMaterial) material = XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (material.IsNull()) return Standard_True;
    if (!material->HasPbrMaterial() && !material->HasCommonMaterial()) return Standard_False;
    XCAFDoc_VisMaterialPBR pbr = material->HasPbrMaterial()
        ? material->PbrMaterial() : material->ConvertToPbrMaterial();
    const Handle(Image_Texture) common = material->HasCommonMaterial()
        ? material->CommonMaterial().DiffuseTexture : Handle(Image_Texture)();
    if (!pbr.BaseColorTexture.IsNull() && !common.IsNull()
        && !Core3DTexturesMatch(pbr.BaseColorTexture, common)) return Standard_False;
    Handle(TDataStd_Integer) marker;
    const bool owned = label.FindAttribute(LocalPBRMaterialAttributeID(), marker)
        && !marker.IsNull() && marker->Get() == 1;
    if (owned && !pbr.NormalTexture.IsNull()
        && !Core3DValidateNormalTextureBinding(myOcafDoc, label)) return Standard_False;
    if (slot != OcctMaterialTextureSlot::BaseColor
        && (!pbr.BaseColorTexture.IsNull() || !common.IsNull())
        && (!owned || pbr.BaseColorTexture.IsNull() || common.IsNull())) return Standard_False;
    for (const auto other : {OcctMaterialTextureSlot::BaseColor, OcctMaterialTextureSlot::Emissive,
                            OcctMaterialTextureSlot::MetallicRoughness, OcctMaterialTextureSlot::Occlusion, OcctMaterialTextureSlot::Normal}) {
        if (other == slot) continue;
        const Handle(Image_Texture)& texture = Core3DMaterialTexture(pbr, other);
        // Replacing one imported map is explicit. Preserving a different
        // imported map must never silently change its ownership.
        if (!texture.IsNull() && !owned) return Standard_False;
    }
    return Standard_True;
}

Standard_Boolean
OcctDocument::IsEmissiveTextureFactorAutoPromotedForLabel(
    const TDF_Label& label) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    Handle(TDataStd_Integer) marker;
    return label.FindAttribute(
            AutoPromotedEmissiveFactorAttributeID(), marker)
        && !marker.IsNull() && marker->Get() == 1;
}

Standard_Boolean
OcctDocument::SetEmissiveTextureFactorAutoPromotedForLabel(
    const TDF_Label& label,
    const Standard_Boolean isAutoPromoted) {
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull()
        || !EnsureGeometryRepresentationForMutation(label)) {
        return Standard_False;
    }
    if (isAutoPromoted) {
        Handle(TDataStd_Integer) localMarker;
        const Handle(XCAFDoc_VisMaterial) material =
            XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
        if (!label.FindAttribute(
                LocalPBRMaterialAttributeID(), localMarker)
            || localMarker.IsNull() || localMarker->Get() != 1
            || material.IsNull() || !material->HasPbrMaterial()) {
            return Standard_False;
        }
        const XCAFDoc_VisMaterialPBR& pbr = material->PbrMaterial();
        if (pbr.EmissiveTexture.IsNull()
            || pbr.EmissiveFactor.x() != 1.0f
            || pbr.EmissiveFactor.y() != 1.0f
            || pbr.EmissiveFactor.z() != 1.0f) {
            return Standard_False;
        }
        TDataStd_Integer::Set(
            label, AutoPromotedEmissiveFactorAttributeID(), 1);
    } else {
        label.ForgetAttribute(
            AutoPromotedEmissiveFactorAttributeID());
    }
    return Standard_True;
}

void OcctDocument::LoadObjectMeterial(const TDF_Label& label, const Handle(AIS_Shape) anAis) {
    if (label.IsNull() || anAis.IsNull()) {
        return;
    }
    const Handle(XCAFDoc_VisMaterial) aVisualMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    Handle(TDataStd_Integer) aLocalPBRMarker;
    const Standard_Boolean hasLocalPBR =
        label.FindAttribute(
            LocalPBRMaterialAttributeID(), aLocalPBRMarker)
        && !aLocalPBRMarker.IsNull()
        && aLocalPBRMarker->Get() == 1;
    Graphic3d_NameOfMaterial aLegacyMaterial;
    Quantity_NameOfColor aLegacyColor;
    const Standard_Boolean hasLegacyMaterial =
        TryMaterialNameForLabel(label, aLegacyMaterial);
    const Standard_Boolean hasLegacyColor =
        TryColorNameForLabel(label, aLegacyColor);
    const Handle(CafShapePrs) aCafPresentation =
        Handle(CafShapePrs)::DownCast(anAis);
    if (!aVisualMaterial.IsNull()) {
        Graphic3d_MaterialAspect anAspect;
        aVisualMaterial->FillMaterialAspect(anAspect);
        if (hasLocalPBR) {
            if (!aCafPresentation.IsNull()) {
                aCafPresentation->ApplyAuthoredVisualMaterial(
                    aVisualMaterial);
            } else {
                ApplyVisualMaterialToPlainPresentation(
                    aVisualMaterial, anAis);
            }
            return;
        }
        if (!aCafPresentation.IsNull()) {
            // CafShapePrs' default-style path already uses FillAspect() and
            // retains imported face/occurrence precedence.
            anAis->SetMaterial(anAspect);
            anAis->SetColor(aVisualMaterial->BaseColor().GetRGB());
        } else {
            ApplyVisualMaterialToPlainPresentation(
                aVisualMaterial, anAis);
        }
    }
    const Graphic3d_MaterialAspect aLegacyAspect = hasLegacyMaterial
        ? Graphic3d_MaterialAspect(aLegacyMaterial)
        : Graphic3d_MaterialAspect();
    const Quantity_Color aLegacyQuantity = hasLegacyColor
        ? Quantity_Color(aLegacyColor)
        : Quantity_Color(Quantity_NOC_GRAY80);
    if (!aCafPresentation.IsNull()
        && (hasLegacyMaterial || hasLegacyColor)) {
        aCafPresentation->ApplyAuthoredLegacyAppearance(
            hasLegacyMaterial,
            aLegacyAspect,
            hasLegacyColor,
            aLegacyQuantity);
    } else {
        if (hasLegacyMaterial) {
            ResetDrawerForLegacyMaterial(anAis->Attributes());
        } else if (aVisualMaterial.IsNull()) {
            ClearDrawerTextureMapping(anAis->Attributes());
        }
        if (hasLegacyMaterial) {
            anAis->SetMaterial(aLegacyAspect);
        }
        if (hasLegacyColor) {
            anAis->SetColor(aLegacyQuantity);
        }
        if (hasLegacyMaterial || hasLegacyColor
            || aVisualMaterial.IsNull()) {
            anAis->SynchronizeAspects();
        }
    }

}

void OcctDocument::LoadObjectAuthoredMaterialOverrides(
    const TDF_Label& label,
    const Handle(AIS_Shape) anAis) {
    if (label.IsNull() || anAis.IsNull()) {
        return;
    }

    Handle(TDataStd_Integer) aLocalPBRMarker;
    const Standard_Boolean hasLocalPBR =
        label.FindAttribute(
            LocalPBRMaterialAttributeID(), aLocalPBRMarker)
        && !aLocalPBRMarker.IsNull()
        && aLocalPBRMarker->Get() == 1;
    if (hasLocalPBR) {
        const Handle(XCAFDoc_VisMaterial) aVisualMaterial =
            XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
        if (!aVisualMaterial.IsNull()) {
            Graphic3d_MaterialAspect anAspect;
            aVisualMaterial->FillMaterialAspect(anAspect);
            const Handle(CafShapePrs) aCafPresentation =
                Handle(CafShapePrs)::DownCast(anAis);
            if (!aCafPresentation.IsNull()) {
                aCafPresentation->ApplyAuthoredVisualMaterial(
                    aVisualMaterial);
            } else {
                ApplyVisualMaterialToPlainPresentation(
                    aVisualMaterial, anAis);
            }
            return;
        }
    }

    Graphic3d_NameOfMaterial aLegacyMaterial;
    const Standard_Boolean hasLegacyMaterial =
        TryMaterialNameForLabel(label, aLegacyMaterial);
    Quantity_NameOfColor aLegacyColor;
    const Standard_Boolean hasLegacyColor =
        TryColorNameForLabel(label, aLegacyColor);
    const Graphic3d_MaterialAspect aLegacyAspect = hasLegacyMaterial
        ? Graphic3d_MaterialAspect(aLegacyMaterial)
        : Graphic3d_MaterialAspect();
    const Quantity_Color aLegacyQuantity = hasLegacyColor
        ? Quantity_Color(aLegacyColor)
        : Quantity_Color(Quantity_NOC_GRAY80);
    const Handle(CafShapePrs) aCafPresentation =
        Handle(CafShapePrs)::DownCast(anAis);
    if (!aCafPresentation.IsNull()
        && (hasLegacyMaterial || hasLegacyColor)) {
        aCafPresentation->ApplyAuthoredLegacyAppearance(
            hasLegacyMaterial,
            aLegacyAspect,
            hasLegacyColor,
            aLegacyQuantity);
    } else {
        if (hasLegacyMaterial) {
            ResetDrawerForLegacyMaterial(anAis->Attributes());
        }
        if (hasLegacyMaterial) {
            anAis->SetMaterial(aLegacyAspect);
        }
        if (hasLegacyColor) {
            anAis->SetColor(aLegacyQuantity);
        }
        if (hasLegacyMaterial || hasLegacyColor) {
            anAis->SynchronizeAspects();
        }
    }
}


Standard_Boolean OcctDocument::OpenPrivateExportSnapshot(
    const std::string& path,
    const Message_ProgressRange& progress) {
    if (path.empty() || myApp.IsNull() || !myOcafDoc.IsNull()) {
        return Standard_False;
    }

    Handle(TDocStd_Document) candidate;
    try {
        OCC_CATCH_SIGNALS
        Core3DDefineSafeBinXCAFFormat(myApp);
        Core3DBeginSafeBinaryRead();
        const PCDM_ReaderStatus status = myApp->Open(
            TCollection_ExtendedString(path.c_str(), Standard_True),
            candidate,
            progress);
        const Standard_Boolean wasRejected =
            Core3DSafeBinaryReadWasRejected();
        Standard_Size frameBytes = 0;
        if (wasRejected || status != PCDM_RS_OK || candidate.IsNull()
            || !ValidateGeometryRepresentations(candidate)
            || !Core3DValidateOwnedFrameUsage(candidate,frameBytes)) {
            if (!candidate.IsNull()) {
                try {
                    myApp->Close(candidate);
                } catch (...) {
                }
                candidate.Nullify();
            }
            return Standard_False;
        }
        myOcafDoc = candidate;
        return Standard_True;
    } catch (...) {
        if (!candidate.IsNull()) {
            try {
                myApp->Close(candidate);
            } catch (...) {
            }
            candidate.Nullify();
        }
        return Standard_False;
    }
}

void OcctDocument::ClosePrivateExportSnapshot() noexcept {
    if (myApp.IsNull() || myOcafDoc.IsNull()) {
        myOcafDoc.Nullify();
        return;
    }
    try {
        if (myOcafDoc->HasOpenCommand()) {
            myOcafDoc->AbortCommand();
        }
    } catch (...) {
    }
    try {
        myApp->Close(myOcafDoc);
    } catch (...) {
    }
    myOcafDoc.Nullify();
}

void OcctDocument::ApplyTransforms() {
    ApplyTransforms(Message_ProgressRange());
}

Standard_Boolean OcctDocument::ApplyTransforms(
    const Message_ProgressRange& progress) {
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || !ValidateGeometryRepresentations()) {
        return Standard_False;
    }
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    if (shapeTool.IsNull()) {
        return Standard_False;
    }
    TDF_LabelSequence aLabels;
    shapeTool->GetFreeShapes (aLabels);
    Message_ProgressScope aScope(
        progress,
        "Apply object transforms",
        aLabels.Length());
    for (Standard_Integer aLabIter = 1;
         aLabIter <= aLabels.Length();
         ++aLabIter)
    {
        if (!aScope.More()) {
            return Standard_False;
        }
        const TDF_Label& aLabel = aLabels.Value (aLabIter);
        if (XCAFDoc_ShapeTool::IsSimpleShape(aLabel)
            && !XCAFDoc_ShapeTool::IsAssembly(aLabel)
            && !EnsureGeometryRepresentationForMutation(aLabel)) {
            return Standard_False;
        }
        const auto t = ObjectTransformForLabel(aLabel);
        TNaming::Displace(aLabel, TopLoc_Location(t));
        aScope.Next();
    }
    return Standard_True;
}

gp_Trsf OcctDocument::ObjectTransformForLabel(const TDF_Label& aRefLabel) const {
    const auto readReal = [&aRefLabel](const Standard_Integer theTag,
                                      const Standard_Real theDefault) {
        if (aRefLabel.IsNull()) {
            return theDefault;
        }
        const TDF_Label aChild = aRefLabel.FindChild(theTag, Standard_False);
        if (aChild.IsNull()) {
            return theDefault;
        }
        Handle(TDataStd_Real) anAttribute;
        return aChild.FindAttribute(TDataStd_Real::GetID(), anAttribute)
            && !anAttribute.IsNull()
            ? anAttribute->Get()
            : theDefault;
    };

    const Standard_Real x = readReal(1, 0.0);
    const Standard_Real y = readReal(2, 0.0);
    const Standard_Real z = readReal(3, 0.0);
    const Standard_Real rx = readReal(4, 0.0);
    const Standard_Real ry = readReal(5, 0.0);
    const Standard_Real rz = readReal(6, 0.0);
    const Standard_Real rw = readReal(7, 1.0);
    const Standard_Real scale = readReal(8, 1.0);
    
    gp_Trsf t = gp_Trsf();
    t.SetTranslation({x, y, z});
    t.SetRotationPart({rx, ry, rz, rw});
    t.SetScaleFactor(scale);
    return t;
}

Standard_Boolean OcctDocument::TryObjectTransformForLabel(
    const TDF_Label& label,
    gp_Trsf& transform) const
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        return TryReadObjectTransform(
            myOcafDoc, aShapeTool, label, transform)
            ? Standard_True : Standard_False;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctObjectTransformState::IsEqual(
    const OcctObjectTransformState& other) const noexcept
{
    if (![NSThread isMainThread]) { return Standard_False; }
    try {
        for (Standard_Integer row = 1; row <= 3; ++row) {
            for (Standard_Integer column = 1; column <= 4; ++column) {
                const Standard_Real value = transform.Value(row, column);
                if (!std::isfinite(value) || value != other.transform.Value(row, column)) {
                    return Standard_False;
                }
            }
        }
        return !label.IsNull() && !other.label.IsNull()
            && !documentData.IsNull() && documentData == other.documentData
            && label.IsEqual(other.label) && label.Data() == documentData
            && other.label.Data() == other.documentData
            && !shape.IsNull() && !other.shape.IsNull() && shape.IsEqual(other.shape)
            && !entityIdentifier.empty() && entityIdentifier == other.entityIdentifier
            && !definitionIdentifier.empty() && definitionIdentifier == other.definitionIdentifier
            && storedRepresentation != OcctGeometryRepresentation::Invalid
            && storedRepresentation == other.storedRepresentation
            && resolvedRepresentation != OcctGeometryRepresentation::Invalid
            && resolvedRepresentation == other.resolvedRepresentation
            && meshUVAtlasVersion == other.meshUVAtlasVersion
            && meshUVAtlasSettings == other.meshUVAtlasSettings
            && meshRegionPartition == other.meshRegionPartition
            && authoredFramesPresent == other.authoredFramesPresent
            && authoredFramesIdentity == other.authoredFramesIdentity
            && profile.IsEqual(other.profile)
            && enclosure.IsEqual(other.enclosure)
            && sweep.IsEqual(other.sweep) && loft.IsEqual(other.loft) && retained.IsEqual(other.retained)
            && present == other.present && scalars == other.scalars;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::CaptureObjectTransformStateForLabel(
    const TDF_Label& label, OcctObjectTransformState& state) const noexcept
{
    state = OcctObjectTransformState();
    if (![NSThread isMainThread]) { return Standard_False; }
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(myOcafDoc->Main())
            || !IsEditableFreeSimpleDefinitionLabel(label)) {
            return Standard_False;
        }
        OcctObjectTransformState captured;
        captured.label = label;
        captured.documentData = myOcafDoc->GetData();
        captured.shape = XCAFDoc_ShapeTool::GetShape(label);
        captured.entityIdentifier = EntityIdentifierForLabel(label);
        captured.definitionIdentifier = DefinitionIdentifierForLabel(label);
        captured.storedRepresentation = StoredGeometryRepresentationForLabel(label);
        captured.resolvedRepresentation = GeometryRepresentationForLabel(label);
        if (!core3d::profile::Read(myOcafDoc, label, captured.profile)
            || !core3d::enclosure::Read(myOcafDoc, label, captured.enclosure)
            || !core3d::sweep_persistence::Read(myOcafDoc, label, captured.sweep)
            || !core3d::loft_persistence::Read(myOcafDoc, label, captured.loft)
            || !core3d::retained_solid::Read(myOcafDoc,label,captured.retained)) return Standard_False;
        OcctAuthoredFrameRecord frames;
        const auto frameState = Core3DReadAuthoredFrameOwner(myOcafDoc, label, frames);
        if (frameState == OcctAuthoredFrameReadState::Invalid) return Standard_False;
        captured.authoredFramesPresent = frameState == OcctAuthoredFrameReadState::Authored;
        captured.authoredFramesIdentity = frames.identity;
        Handle(TDF_Attribute) atlasAttribute;
        if (label.FindAttribute(MeshUVAtlasAttributeID(), atlasAttribute)) {
            const auto version = Handle(TDataStd_Integer)::DownCast(atlasAttribute);
            if (version.IsNull() || (version->Get() != 1 && version->Get() != 2 && version->Get() != 3)
                || captured.resolvedRepresentation != OcctGeometryRepresentation::TriangleMesh) { return Standard_False; }
            captured.meshUVAtlasVersion = version->Get();
        }
        for (int i=0;i<3;++i) {
            Handle(TDF_Attribute) attribute;
            const bool present=label.FindAttribute(MeshUVAtlasSettingsAttributeID(i),attribute);
            if (captured.meshUVAtlasVersion == 2) {
                const auto value=Handle(TDataStd_Integer)::DownCast(attribute);
                if (!present || value.IsNull()) return Standard_False;
                captured.meshUVAtlasSettings[i]=value->Get();
            } else if (present) { return Standard_False; }
        }
        if (captured.meshUVAtlasVersion == 2
            && (!shapeyard::uv::Settings{captured.meshUVAtlasSettings[0],captured.meshUVAtlasSettings[1]}.valid()
                || captured.meshUVAtlasSettings[2]<=0 || captured.meshUVAtlasSettings[2]>12288)) return Standard_False;
        if (ReadMeshRegionPartitionRecord(myOcafDoc, label, captured.meshRegionPartition)
                == MeshRegionPartitionReadState::Malformed) return Standard_False;
        if (captured.documentData.IsNull() || captured.shape.IsNull()
            || captured.entityIdentifier.empty() || captured.definitionIdentifier.empty()
            || captured.storedRepresentation == OcctGeometryRepresentation::Invalid
            || captured.resolvedRepresentation == OcctGeometryRepresentation::Invalid
            || !TryObjectTransformForLabel(label, captured.transform)) {
            return Standard_False;
        }
        for (Standard_Integer index = 0; index < 8; ++index) {
            const TDF_Label child = label.FindChild(index + 1, Standard_False);
            Handle(TDF_Attribute) attribute;
            if (!child.IsNull() && child.FindAttribute(TDataStd_Real::GetID(), attribute)) {
                const Handle(TDataStd_Real) scalar = Handle(TDataStd_Real)::DownCast(attribute);
                if (scalar.IsNull() || !std::isfinite(scalar->Get())) {
                    return Standard_False;
                }
                captured.present[index] = Standard_True;
                captured.scalars[index] = scalar->Get();
            }
        }
        state = std::move(captured);
        return Standard_True;
    } catch (...) {
        state = OcctObjectTransformState();
        return Standard_False;
    }
}

Standard_Boolean OcctObjectVisibilityState::HasSameObjectAndLayers(
    const OcctObjectVisibilityState& other) const noexcept {
    try {
        if (!object.IsEqual(other.object) || layerLinkPresent != other.layerLinkPresent
            || layers.size() != other.layers.size()
            || layerInvisibleAttributePresent != other.layerInvisibleAttributePresent) { return Standard_False; }
        for (std::size_t index = 0; index < layers.size(); ++index) {
            if (!layers[index].IsEqual(other.layers[index])) { return Standard_False; }
        }
        return Standard_True;
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctObjectVisibilityState::IsEqual(const OcctObjectVisibilityState& other) const noexcept {
    return invisibleAttributePresent == other.invisibleAttributePresent && HasSameObjectAndLayers(other);
}

Standard_Boolean OcctObjectVisibilityState::IsEffectivelyVisible() const noexcept {
    if (invisibleAttributePresent) { return Standard_False; }
    for (bool hidden : layerInvisibleAttributePresent) { if (hidden) { return Standard_False; } }
    return Standard_True;
}

Standard_Boolean OcctDocument::CaptureObjectVisibilityStateForLabel(
    const TDF_Label& label, OcctObjectVisibilityState& state) const noexcept {
    state = OcctObjectVisibilityState();
    if (![NSThread isMainThread]) { return Standard_False; }
    try {
        OcctObjectVisibilityState captured;
        if (!CaptureObjectNameStateForLabel(label, captured.object)) { return Standard_False; }
        const auto captureInvisible = [](const TDF_Label& target, bool& present) {
            Handle(TDF_Attribute) attribute;
            present = target.FindAttribute(XCAFDoc::InvisibleGUID(), attribute);
            return !present || !Handle(TDataStd_UAttribute)::DownCast(attribute).IsNull();
        };
        bool hidden = false;
        if (!captureInvisible(label, hidden)) { return Standard_False; }
        captured.invisibleAttributePresent = hidden;
        Handle(TDF_Attribute) association;
        if (label.FindAttribute(XCAFDoc::LayerRefGUID(), association)) {
            const auto graph = Handle(XCAFDoc_GraphNode)::DownCast(association);
            if (graph.IsNull() || graph->NbFathers() < 0 || graph->NbFathers() > 1024) { return Standard_False; }
            captured.layerLinkPresent = Standard_True;
            for (Standard_Integer index = 1; index <= graph->NbFathers(); ++index) {
                const auto father = graph->GetFather(index);
                if (father.IsNull()) { return Standard_False; }
                const auto layer = father->Label();
                if (layer.IsNull() || layer.Data() != captured.object.object.documentData
                    || !captureInvisible(layer, hidden)) { return Standard_False; }
                for (const auto& previous : captured.layers) {
                    if (previous.IsEqual(layer)) { return Standard_False; }
                }
                captured.layers.push_back(layer);
                captured.layerInvisibleAttributePresent.push_back(hidden);
            }
        }
        state = std::move(captured);
        return Standard_True;
    } catch (...) { state = OcctObjectVisibilityState(); return Standard_False; }
}

Standard_Boolean OcctDocument::SetObjectVisibilityForLabel(
    const TDF_Label& label, Standard_Boolean visible) noexcept {
    if (![NSThread isMainThread] || myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || !XCAFDoc_DocumentTool::CheckColorTool(myOcafDoc->Main())) { return Standard_False; }
    try {
        OcctObjectVisibilityState before, after;
        if (!CaptureObjectVisibilityStateForLabel(label, before)) { return Standard_False; }
        if (visible) {
            for (bool hidden : before.layerInvisibleAttributePresent) { if (hidden) { return Standard_False; } }
        }
        if (before.invisibleAttributePresent == !visible) { return Standard_True; }
        const auto colors = XCAFDoc_DocumentTool::ColorTool(myOcafDoc->Main());
        colors->SetVisibility(label, visible);
        return CaptureObjectVisibilityStateForLabel(label, after)
            && before.HasSameObjectAndLayers(after) && after.invisibleAttributePresent == !visible;
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctObjectNameState::IsEqual(const OcctObjectNameState& other) const noexcept {
    try {
        return object.IsEqual(other.object) && namePresent == other.namePresent
            && (!namePresent || name.IsEqual(other.name));
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctSavedGroupState::IsEqual(const OcctSavedGroupState& other) const noexcept {
    try {
        if (documentData.IsNull() || documentData != other.documentData
            || !container.IsEqual(other.container) || groups.size() != other.groups.size()) { return Standard_False; }
        for (std::size_t i = 0; i < groups.size(); ++i) {
            const auto& a = groups[i]; const auto& b = other.groups[i];
            if (!a.recordLabel.IsEqual(b.recordLabel) || a.identifier != b.identifier
                || !a.name.IsEqual(b.name) || a.originPresent != b.originPresent
                || (a.originPresent && !a.origin.IsEqual(b.origin,0.0))
                || a.members.size() != b.members.size()) { return Standard_False; }
            for (std::size_t j = 0; j < a.members.size(); ++j) {
                if (!a.members[j].IsEqual(b.members[j])) { return Standard_False; }
            }
        }
        return Standard_True;
    } catch (...) { return Standard_False; }
}
std::string OcctDocument::NewSavedGroupIdentifier() noexcept {
    if (![NSThread isMainThread]) { return {}; }
    try { return NewIdentifier(); } catch (...) { return {}; }
}
std::string OcctDocument::NewProfileIdentifier() noexcept {
    if (![NSThread isMainThread]) { return {}; }
    try { return NewIdentifier(); } catch (...) { return {}; }
}
Standard_Boolean OcctDocument::IsAdmittedSavedGroupOrigin(const gp_Pnt& point) noexcept {
    try {
        for (const double value : {point.X(), point.Y(), point.Z()}) {
            if (!std::isfinite(value) || std::abs(value) > core3d::limits::kMaximumModelCoordinateMagnitude) return Standard_False;
        }
        return Standard_True;
    } catch (...) { return Standard_False; }
}
Standard_Boolean OcctDocument::CaptureSavedGroups(OcctSavedGroupState& state) const noexcept {
    state = OcctSavedGroupState();
    return [NSThread isMainThread] && ReadSavedGroups(myOcafDoc, state);
}
Standard_Boolean OcctDocument::StageSavedGroups(const std::vector<OcctSavedGroup>& groups) noexcept {
    if (![NSThread isMainThread] || myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || groups.size() > kMaximumSavedGroups) { return Standard_False; }
    try {
        OcctSavedGroupState before;
        if (!CaptureSavedGroups(before)) { return Standard_False; }
        std::unordered_set<std::string> identifiers, entities;
        std::vector<OcctObjectNameState> objectAuthority;
        for (const auto& group : groups) {
            if (!IsCanonicalSavedGroupID(group.identifier) || !OcctObjectNameIsValid(group.name)
                || !identifiers.insert(group.identifier).second || group.members.size() > kMaximumSavedGroupMembers) { return Standard_False; }
            for (const auto& label : group.members) {
                OcctObjectNameState object;
                if (!CaptureObjectNameStateForLabel(label, object)
                    || !entities.insert(object.object.entityIdentifier).second) { return Standard_False; }
                objectAuthority.push_back(std::move(object));
            }
        }
        // Capture removed members too; catalog replacement never changes parts.
        for (const auto& group : before.groups) {
            for (const auto& label : group.members) {
                OcctObjectNameState object;
                if (!CaptureObjectNameStateForLabel(label, object)) { return Standard_False; }
                objectAuthority.push_back(std::move(object));
            }
        }
        TDF_Label container = before.container;
        if (container.IsNull() && groups.empty()) { return Standard_True; }
        if (container.IsNull()) {
            const auto root = myOcafDoc->GetData()->Root();
            Standard_Integer tag = 0;
            for (TDF_ChildIterator it(root, Standard_False); it.More(); it.Next()) { tag = std::max(tag, it.Value().Tag()); }
            if (tag == std::numeric_limits<Standard_Integer>::max()) { return Standard_False; }
            // A sibling of Main is outside all XCAF document-tool fixed tags.
            container = root.FindChild(tag + 1, Standard_True);
            TDataStd_Integer::Set(container, SavedGroupContainerID(), 1);
        }
        std::unordered_map<std::string, TDF_Label> retained;
        std::vector<TDF_Label> reusable;
        for (const auto& group : before.groups) {
            if (identifiers.count(group.identifier)) { retained.emplace(group.identifier, group.recordLabel); }
            for (const auto& label : group.members) { label.ForgetAttribute(SavedGroupMembershipID()); }
            group.recordLabel.ForgetAttribute(SavedGroupRecordID());
            group.recordLabel.ForgetAttribute(SavedGroupNameID());
            group.recordLabel.ForgetAttribute(SavedGroupOriginXID());
            group.recordLabel.ForgetAttribute(SavedGroupOriginYID());
            group.recordLabel.ForgetAttribute(SavedGroupOriginZID());
        }
        Standard_Integer maximumTag = 0;
        for (TDF_ChildIterator it(container, Standard_False); it.More(); it.Next()) {
            const auto label = it.Value(); maximumTag = std::max(maximumTag, label.Tag());
            bool reserved = false;
            for (const auto& pair : retained) { if (pair.second.IsEqual(label)) { reserved = true; break; } }
            if (!reserved && !label.HasAttribute() && !label.HasChild()) { reusable.push_back(label); }
        }
        for (const auto& group : groups) {
            TDF_Label label;
            const auto found = retained.find(group.identifier);
            if (found != retained.end()) { label = found->second; }
            else if (!reusable.empty()) { label = reusable.back(); reusable.pop_back(); }
            else {
                if (maximumTag == std::numeric_limits<Standard_Integer>::max()) { return Standard_False; }
                label = container.FindChild(++maximumTag, Standard_True);
            }
            TDataStd_AsciiString::Set(label, SavedGroupRecordID(), TCollection_AsciiString(group.identifier.c_str()));
            TDataStd_Name::Set(label, SavedGroupNameID(), group.name);
            if (group.originPresent) {
                if (!IsAdmittedSavedGroupOrigin(group.origin)) return Standard_False;
                TDataStd_Real::Set(label,SavedGroupOriginXID(),group.origin.X());
                TDataStd_Real::Set(label,SavedGroupOriginYID(),group.origin.Y());
                TDataStd_Real::Set(label,SavedGroupOriginZID(),group.origin.Z());
            }
            for (const auto& member : group.members) {
                TDataStd_AsciiString::Set(member, SavedGroupMembershipID(), TCollection_AsciiString(group.identifier.c_str()));
            }
        }
        OcctSavedGroupState after;
        if (!CaptureSavedGroups(after) || after.groups.size() != groups.size()) { return Standard_False; }
        for (const auto& requested : groups) {
            const auto found = std::find_if(after.groups.begin(), after.groups.end(), [&](const auto& g) { return g.identifier == requested.identifier; });
            if (found == after.groups.end() || !found->name.IsEqual(requested.name)
                || found->originPresent != requested.originPresent
                || (requested.originPresent && !found->origin.IsEqual(requested.origin,0.0))
                || found->members.size() != requested.members.size()) { return Standard_False; }
            for (const auto& member : requested.members) {
                if (std::none_of(found->members.begin(), found->members.end(), [&](const auto& l) { return l.IsEqual(member); })) { return Standard_False; }
            }
        }
        for (const auto& expected : objectAuthority) {
            OcctObjectNameState actual;
            if (!CaptureObjectNameStateForLabel(expected.object.label, actual) || !expected.IsEqual(actual)) { return Standard_False; }
        }
        return Standard_True;
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctObjectNameIsValid(const TCollection_ExtendedString& name) noexcept {
    try {
        if (name.Length() < 1 || name.Length() > 256) { return Standard_False; }
        NSString* value = [[NSString alloc]
            initWithCharacters:reinterpret_cast<const unichar*>(name.ToExtString())
            length:static_cast<NSUInteger>(name.Length())];
        if (value == nil || [value dataUsingEncoding:NSUTF8StringEncoding allowLossyConversion:NO] == nil) {
            return Standard_False;
        }
        for (Standard_Integer index = 1; index <= name.Length(); ++index) {
            const auto character = name.Value(index);
            if (character < 0x20 || (character >= 0x7f && character <= 0x9f)
                || character == 0x2028 || character == 0x2029) { return Standard_False; }
        }
        NSString* trimmed = [value stringByTrimmingCharactersInSet:NSCharacterSet.whitespaceAndNewlineCharacterSet];
        return trimmed.length > 0 && [trimmed isEqualToString:value];
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctDocument::CaptureObjectNameStateForLabel(
    const TDF_Label& label, OcctObjectNameState& state) const noexcept {
    state = OcctObjectNameState();
    if (![NSThread isMainThread]) { return Standard_False; }
    try {
        OcctObjectNameState captured;
        if (!CaptureObjectTransformStateForLabel(label, captured.object)) { return Standard_False; }
        Handle(TDF_Attribute) attribute;
        if (label.FindAttribute(TDataStd_Name::GetID(), attribute)) {
            const auto authored = Handle(TDataStd_Name)::DownCast(attribute);
            if (authored.IsNull()) { return Standard_False; }
            captured.namePresent = Standard_True;
            captured.name = authored->Get();
        }
        state = std::move(captured);
        return Standard_True;
    } catch (...) { state = OcctObjectNameState(); return Standard_False; }
}

Standard_Boolean OcctDocument::SetObjectNameForLabel(
    const TDF_Label& label, const TCollection_ExtendedString& name) noexcept {
    if (![NSThread isMainThread] || myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || !OcctObjectNameIsValid(name)) { return Standard_False; }
    try {
        OcctObjectNameState before, after;
        if (!CaptureObjectNameStateForLabel(label, before)) { return Standard_False; }
        if (before.namePresent && before.name.IsEqual(name)) { return Standard_True; }
        TDataStd_Name::Set(label, name);
        return CaptureObjectNameStateForLabel(label, after)
            && before.object.IsEqual(after.object) && after.namePresent && after.name.IsEqual(name);
    } catch (...) { return Standard_False; }
}

void OcctDocument::LoadObjectTransform(const TDF_Label& aRefLabel, const Handle(AIS_Shape) anAis) {
    anAis->SetLocalTransformation(ObjectTransformForLabel(aRefLabel));
}

Standard_Boolean OcctDocument::undo() {
    if (NativeBooleanOwnerBlocksOtherWork()) return Standard_False;
    if (myNativeAuthority && !myOcafDoc.IsNull()) myNativeAuthority->HistoryBoundary(myOcafDoc.get());
    if (!canUndo()) {
		return Standard_False;
    }
    try {
        if (myOcafDoc->Undo()) {
            if (myNativeAuthority) myNativeAuthority->HistoryBoundary(myOcafDoc.get());
#if DEBUG
            if (myLiveProbe) myLiveProbe->Record(
                core3d::debug::LiveTransactionObservation::Kind::UndoCompleted, myOcafDoc);
#endif
            NotifyChanges();
			return Standard_True;
        }
    } catch (const Standard_Failure& ex) {
        std::cout << ex.GetMessageString() << std::endl;
    }
	return Standard_False;
}
Standard_Boolean OcctDocument::redo() {
    if (NativeBooleanOwnerBlocksOtherWork()) return Standard_False;
    if (myNativeAuthority && !myOcafDoc.IsNull()) myNativeAuthority->HistoryBoundary(myOcafDoc.get());
    if (!canRedo()) {
		return Standard_False;
    }
    try {
		if (myOcafDoc->Redo()) {
            if (myNativeAuthority) myNativeAuthority->HistoryBoundary(myOcafDoc.get());
#if DEBUG
            if (myLiveProbe) myLiveProbe->Record(
                core3d::debug::LiveTransactionObservation::Kind::RedoCompleted, myOcafDoc);
#endif
			NotifyChanges();
			return Standard_True;
		}
	} catch(const Standard_Failure& ex) {
        std::cout << ex.GetMessageString() << std::endl;
    }
	return Standard_False;
}

const bool OcctDocument::canUndo() const {
	return !NativeBooleanOwnerBlocksOtherWork() && !myOcafDoc.IsNull() && !myOcafDoc->HasOpenCommand()
		&& myOcafDoc->GetAvailableUndos() > 0;
}

const bool OcctDocument::canRedo() const {
	return !NativeBooleanOwnerBlocksOtherWork() && !myOcafDoc.IsNull() && !myOcafDoc->HasOpenCommand()
		&& myOcafDoc->GetAvailableRedos() > 0;
}

std::string OcctDocument::save(const std::string& path) {
    return save(path, Message_ProgressRange());
}

std::string OcctDocument::save(
    const std::string& path,
    const Message_ProgressRange& progress) {
    Standard_Size frameBytes = 0;
    if (NativeBooleanOwnerBlocksOtherWork() || myOcafDoc.IsNull() || myOcafDoc->HasOpenCommand()
        || !ValidateGeometryRepresentations()
        || !Core3DValidateOwnedFrameUsage(myOcafDoc,frameBytes)) {
        return {};
    }

    auto app =  Handle(TDocStd_Application)::DownCast(myOcafDoc->Application());
    if (app.IsNull()) {
        return {};
    }

    try {
        PCDM_StoreStatus status = app->SaveAs(
            myOcafDoc, path.c_str(), progress); // ".cbf"
        if (status != PCDM_SS_OK) {
            std::cout << "Save CBF failed with status " << status << std::endl;
            return {};
        }
        const TCollection_ExtendedString aBinXCAFFormat("BinXCAF");
        return path + (myOcafDoc->StorageFormat().IsEqual(aBinXCAFFormat)
            ? ".xbf"
            : ".cbf");
    } catch (const Standard_Failure& failure) {
        std::cout << "Save CBF failure: " << failure.GetMessageString() << std::endl;
        return {};
    }
}

void OcctDocument::NotifyChanges() {
    [[NSNotificationCenter defaultCenter]
     postNotificationName:@"OcctDocumentChanges"
     object:[NSValue valueWithPointer:this]];
}

bool OcctDocument::StagePartBooleanPayload(
    const TDF_Label& record,
    const std::shared_ptr<const core3d::composite_recipe::Payload>& payload) noexcept {
    try {
        if (myOcafDoc.IsNull() || record.IsNull() || record.Data() != myOcafDoc->GetData()
            || !payload) return false;
        Handle(core3d::composite_recipe::Attribute) attribute;
        if (!record.FindAttribute(core3d::composite_recipe::AttributeID(), attribute)
            || attribute.IsNull()) return false;
        attribute->Backup(); attribute->value_ = payload;
        return attribute->value_ == payload;
    } catch (...) { return false; }
}

namespace core3d::retained_feature {
namespace {
std::atomic<std::uint64_t> RetainedFeatureOwnerNonce{1};

bool ExactBytes(const TopoDS_Shape& shape, std::vector<std::uint8_t>& output) noexcept {
    std::string bytes;
    if (!retained_part_boolean::ExactShapeBytes(shape, bytes)) return false;
    output.assign(bytes.begin(), bytes.end()); return !output.empty();
}

template<std::size_t N>
void AppendBytes(std::vector<std::uint8_t>& output,
                 const std::array<std::uint8_t, N>& value) {
    output.insert(output.end(), value.begin(), value.end());
}

void AppendInteger(std::vector<std::uint8_t>& output, std::uint64_t value) {
    for (unsigned index=0;index<8;++index)output.push_back(std::uint8_t(value>>(index*8)));
}
} // namespace

struct OcafOwnerService::State final : DocumentPort {
    OcctDocument& wrapper;
    Handle(TDocStd_Document) document;
    const RegistryView& registry;
    const retained_source::RegistryView& sources;
    TDF_Label carrier;
    std::unique_ptr<Owner> owner;
    std::shared_ptr<const PreparedChange> prepared;
    bool openedCommand=false;

    State(OcctDocument& value,const RegistryView& featureRegistry,
          const retained_source::RegistryView& sourceRegistry) noexcept
        :wrapper(value),document(value.Document()),registry(featureRegistry),sources(sourceRegistry){}

    bool capture(CompleteFence& fence, composite_recipe::Definition& definition,
                 std::vector<SourceValue>& sourceValues) noexcept override {
        fence={};definition={};sourceValues.clear();
        try{
            if(document.IsNull()||carrier.IsNull()||carrier.Data()!=document->GetData())return false;
            composite_recipe::Record record;
            if(!composite_recipe::Read(document,carrier,record)||!record.value
                ||record.value->definition.schemaVersion!=3)return false;
            definition=record.value->definition;
            std::vector<std::uint8_t> graphBytes,identityBytes,metadataBytes,historyBytes,currentBytes;
            if(!composite_recipe::EncodeV3(definition,registry,sources,graphBytes)
                ||graphBytes!=record.value->bytes||!composite_recipe::Hash(graphBytes,fence.graph)
                ||!ExactBytes(record.current,currentBytes)||!composite_recipe::Hash(currentBytes,fence.output))return false;
            AppendBytes(identityBytes,definition.owner.document);AppendBytes(identityBytes,definition.owner.entity);
            AppendBytes(identityBytes,definition.owner.definition);AppendBytes(identityBytes,definition.outputNode);
            for(const auto& node:definition.nodes){
                AppendBytes(identityBytes,composite_recipe::NodeID(node));AppendInteger(identityBytes,composite_recipe::LocalID(node));
                if(const auto* feature=std::get_if<composite_recipe::FeatureNode>(&node.value))AppendBytes(identityBytes,feature->feature);
            }
            if(!composite_recipe::Hash(identityBytes,fence.identities))return false;
            retained_recipe::RevisionFence revision;
            revision.documentGeneration=std::uint64_t(reinterpret_cast<std::uintptr_t>(document.get()));
            revision.modelRevision=std::uint64_t(std::max(0,document->GetData()->Time()))+1;
            if(!XCAFDoc_DocumentTool::GetLengthUnit(document,revision.effectiveMetersPerUnit)
                ||!std::isfinite(revision.effectiveMetersPerUnit)||revision.effectiveMetersPerUnit<=0)return false;
            std::size_t sourceIndex=0;
            for(const auto& node:definition.nodes){
                const auto* source=std::get_if<composite_recipe::SourceNode>(&node.value);if(!source)continue;
                if(source->shapeSlot>=record.value->sourceShapes.size())return false;
                ReplayValue value;value.shape=ShapeKind::Solid;
                if(!ExactBytes(record.value->sourceShapes[source->shapeSlot],value.detachedShape)
                    ||!composite_recipe::Hash(value.detachedShape,value.geometry)
                    ||value.geometry!=source->commitments.geometry
                    ||!composite_recipe::Hash(source->recipe.bytes,value.familyProof))return false;
                sourceValues.push_back({source->node,std::move(value)});
                retained_recipe::DependencyRead read;
                read.locator={definition.owner,source->node,source->original.sourceFeature};
                read.geometry=source->commitments.geometry;read.recipe=source->commitments.recipe;
                read.placement=source->commitments.placement;read.material=source->commitments.material;
                read.groups=source->commitments.groups;revision.dependencies.push_back(read);
                AppendBytes(metadataBytes,read.recipe);AppendBytes(metadataBytes,read.placement);
                AppendBytes(metadataBytes,read.material);AppendBytes(metadataBytes,read.groups);++sourceIndex;
            }
            if(sourceIndex!=record.value->sourceShapes.size()||sourceIndex==0
                ||!composite_recipe::Hash(metadataBytes,fence.metadata))return false;
            AppendInteger(historyBytes,std::uint64_t(document->GetAvailableUndos()));
            AppendInteger(historyBytes,std::uint64_t(document->GetAvailableRedos()));
            AppendInteger(historyBytes,std::uint64_t(std::max(0,document->GetData()->Transaction())));
            AppendInteger(historyBytes,std::uint64_t(std::max(0,document->GetData()->Time())));
            if(!composite_recipe::Hash(historyBytes,fence.history))return false;
            revision.ownerShape=fence.output;revision.ownerRecipe=fence.graph;
            revision.ownerMaterial=fence.metadata;
            if(!InstallCapturedRevision(revision.effectiveMetersPerUnit,std::move(revision),fence))return false;
            return fence.valid();
        }catch(...){fence={};definition={};sourceValues.clear();return false;}
    }

    bool stageAndReadBack(const PreparedChange& change) noexcept override {
        try{
            if(document.IsNull()||document->HasOpenCommand()||carrier.IsNull())return false;
            composite_recipe::Record record;
            if(!composite_recipe::Read(document,carrier,record)||!record.value)return false;
            document->NewCommand();openedCommand=true;
            auto payload=std::make_shared<composite_recipe::Payload>();
            payload->definition=change.graph;payload->sourceShapes=record.value->sourceShapes;
            if(!composite_recipe::EncodeV3(change.graph,registry,sources,payload->bytes)
                ||!wrapper.StagePartBooleanPayload(record.label,payload))return false;
            composite_recipe::Record readback;
            return composite_recipe::Read(document,carrier,readback)&&readback.value
                &&readback.value->bytes==payload->bytes&&readback.current.IsEqual(record.current);
        }catch(...){return false;}
    }

    bool commitOneCommand(int& measuredHistoryDelta) noexcept override {
        measuredHistoryDelta=0;
        try{
            if(document.IsNull()||!openedCommand||!document->HasOpenCommand())return false;
            const int before=document->GetAvailableUndos();
            const bool committed=document->CommitCommand();openedCommand=false;
            measuredHistoryDelta=document->GetAvailableUndos()-before;
            return committed&&!document->HasOpenCommand();
        }catch(...){return false;}
    }

    bool abortAndProve(const CompleteFence& before) noexcept override {
        try{
            if(document.IsNull())return false;
            if(document->HasOpenCommand())document->AbortCommand();openedCommand=false;
            CompleteFence actual;composite_recipe::Definition graph;std::vector<SourceValue> values;
            return capture(actual,graph,values)&&actual==before;
        }catch(...){return false;}
    }

    bool reconcile(const CompleteFence& before,const CompleteFence& candidate,
                   OwnerOutcome& outcome) noexcept override {
        try{
            if(document.IsNull())return false;
            if(document->HasOpenCommand())document->AbortCommand();openedCommand=false;
            CompleteFence actual;composite_recipe::Definition graph;std::vector<SourceValue> values;
            if(!capture(actual,graph,values))return false;
            if(actual==before){outcome=OwnerOutcome::Rejected;return true;}
            if(actual.graph==candidate.graph&&actual.output==candidate.output
                &&actual.identities==candidate.identities&&actual.metadata==candidate.metadata){
                outcome=OwnerOutcome::Committed;return true;
            }
            return false;
        }catch(...){return false;}
    }
};

OcafOwnerService::OcafOwnerService(OcctDocument& owner,const RegistryView& registry,
    const retained_source::RegistryView& sources) noexcept
    :state_(std::make_unique<State>(owner,registry,sources)){}
OcafOwnerService::~OcafOwnerService(){retireForDocumentReplacement();}
bool OcafOwnerService::boundTo(const Handle(TDocStd_Document)& document)const noexcept{
    return state_&&!document.IsNull()&&state_->document.get()==document.get();
}
bool OcafOwnerService::blocksOtherWork()const noexcept{return state_&&state_->owner&&state_->owner->blocksOtherWork();}
void OcafOwnerService::retireForDocumentReplacement()noexcept{
    if(!state_)return;if(!state_->document.IsNull()&&state_->document->HasOpenCommand())state_->document->AbortCommand();
    state_->openedCommand=false;state_->prepared.reset();state_->owner.reset();state_->carrier.Nullify();
}
std::shared_ptr<const PreparedChange> OcafOwnerService::prepare(
    const TDF_Label& carrier,const MutationRequest& request,ReplayBudget budget,OwnerReceipt& receipt)noexcept{
    receipt={};if(!state_||state_->document.IsNull()||carrier.IsNull()||carrier.Data()!=state_->document->GetData()
        ||state_->document->HasOpenCommand()||blocksOtherWork()){receipt.reason="owner-busy-or-unbound";return {};}
    state_->carrier=carrier;state_->owner=std::make_unique<Owner>(*state_,RetainedFeatureOwnerNonce.fetch_add(1),state_->registry,state_->sources);
    state_->prepared=state_->owner->prepare(request,budget,receipt);
    if(!state_->prepared)state_->owner.reset();return state_->prepared;
}
OwnerReceipt OcafOwnerService::apply(const std::shared_ptr<const PreparedChange>& prepared)noexcept{
    if(!state_||!state_->owner)return {OwnerOutcome::Rejected,"no-owner-session",0,0,true};
    const OwnerReceipt result=state_->owner->apply(prepared);
    if(result.settled){state_->prepared.reset();state_->owner.reset();state_->carrier.Nullify();}
    return result;
}
OwnerReceipt OcafOwnerService::cancel()noexcept{
    if(!state_||!state_->owner)return {OwnerOutcome::Rejected,"no-owner-session",0,0,true};
    const OwnerReceipt result=state_->owner->cancel();state_->prepared.reset();state_->owner.reset();state_->carrier.Nullify();return result;
}
OwnerReceipt OcafOwnerService::reconcile(const CompleteFence& before,const CompleteFence& candidate)noexcept{
    if(!state_||!state_->owner)return {OwnerOutcome::Rejected,"no-owner-session",0,0,true};
    const OwnerReceipt result=state_->owner->reconcile(before,candidate);
    if(result.settled){state_->prepared.reset();state_->owner.reset();state_->carrier.Nullify();}return result;
}

#if DEBUG
bool OcafOwnerService::installSyntheticFixture(double metersPerUnit,TDF_Label& carrier)noexcept{
    carrier.Nullify();
    try{
        if(!state_||state_->document.IsNull()||state_->document->HasOpenCommand()||blocksOtherWork()
            ||!std::isfinite(metersPerUnit)||metersPerUnit<=0)return false;
        XCAFDoc_DocumentTool::SetLengthUnit(state_->document,metersPerUnit);
        const TopoDS_Shape sourceShape=BRepPrimAPI_MakeBox(10,8,6).Shape();
        const Handle(XCAFDoc_ShapeTool) shapes=XCAFDoc_DocumentTool::ShapeTool(state_->document->Main());
        carrier=shapes->AddShape(sourceShape,Standard_False,Standard_False);
        if(carrier.IsNull()||!state_->wrapper.MigrateLegacyIdentifiers())return false;
        const auto uuid=[](std::uint8_t seed){retained_recipe::UUID value{};for(std::size_t i=0;i<value.size();++i)value[i]=std::uint8_t(seed+i);return value;};
        retained_recipe::OwnerKey key;
        if(!retained_solid::ReadUUID(state_->document->Main(),DocumentIdentifierAttributeID(),key.document)
            ||!retained_solid::ReadUUID(carrier,EntityIdentifierAttributeID(),key.entity)
            ||!retained_solid::ReadUUID(carrier,DefinitionIdentifierAttributeID(),key.definition))return false;
        composite_recipe::Definition graph=composite_recipe::MakeV3Definition();graph.owner=key;
        graph.outputNode=uuid(71);graph.issuance.nextLocalID=4;
        composite_recipe::SourceNode source;source.node=uuid(40);source.localID=1;
        source.original={key.document,uuid(10),uuid(20),uuid(30)};
        source.recipe.kind=composite_recipe::RecipeKind::Profile;
        profile::Parameters profile;profile.metersPerUnit=metersPerUnit;profile.definition.plane=0;profile.definition.depth=6;
        profile.definition.points={gp_Pnt2d(0,0),gp_Pnt2d(10,0),gp_Pnt2d(10,8),gp_Pnt2d(0,8)};
        std::vector<double> values;if(!profile::Encode(profile,values)
            ||!composite_recipe::EncodeScalarRecipe(source.recipe.kind,std::uint32_t(profile::SchemaFor(profile)),values,source.recipe.bytes))return false;
        source.recipe.schema=std::uint32_t(profile::SchemaFor(profile));source.inputToCarrier.sourceMetersPerUnit=metersPerUnit;
        source.inputToCarrier.carrierMetersPerUnit=metersPerUnit;source.shapeSlot=0;
        std::vector<std::uint8_t> sourceBytes;if(!ExactBytes(sourceShape,sourceBytes)
            ||!composite_recipe::Hash(sourceBytes,source.commitments.geometry)
            ||!composite_recipe::Hash(source.recipe.bytes,source.commitments.recipe))return false;
        source.commitments.placement.fill(41);source.commitments.material.fill(42);source.commitments.groups.fill(43);
        graph.nodes.push_back({source});
        composite_recipe::FeatureNode feature;feature.node=uuid(70);feature.feature=uuid(80);feature.localID=2;
        feature.kind=RegistryProbe::SyntheticKind;feature.codecVersion=RegistryProbe::SyntheticCodec;
        feature.inputs={source.node};feature.parameters={1};graph.nodes.push_back({feature});
        composite_recipe::FeatureNode suffix;suffix.node=graph.outputNode;suffix.feature=uuid(81);suffix.localID=3;
        suffix.kind=RegistryProbe::SyntheticKind;suffix.codecVersion=RegistryProbe::SyntheticCodec;
        suffix.inputs={feature.node};suffix.parameters={1};graph.nodes.push_back({suffix});
        auto payload=std::make_shared<composite_recipe::Payload>();payload->definition=graph;payload->sourceShapes={sourceShape};
        if(!composite_recipe::EncodeV3(graph,state_->registry,state_->sources,payload->bytes))return false;
        const TDF_Label record=carrier.FindChild(composite_recipe::MinimumRecordTag,Standard_True);
        Handle(composite_recipe::Attribute) attribute=new composite_recipe::Attribute();record.AddAttribute(attribute);
        if(!state_->wrapper.StagePartBooleanPayload(record,payload))return false;
        TNaming_Builder(record).Select(sourceShape,sourceShape);state_->document->ClearUndos();
        state_->document->SetUndoLimit(OcctDocument::kNativeSessionUndoLimit);
        composite_recipe::Record readback;return composite_recipe::Read(state_->document,carrier,readback)
            &&readback.value&&readback.value->bytes==payload->bytes;
    }catch(...){carrier.Nullify();return false;}
}

std::map<std::string,bool> OcafOwnerService::debugLifecycle(double metersPerUnit)noexcept{
    std::map<std::string,bool> result{{"fixture-installed",false},{"prepare-read-only",false},
        {"one-command",false},{"undo-redo-exact",false},{"cold-reopen-later-edit",false},
        {"release-registry-refuses",false},{"suffix-failure-no-mutation",false},
        {"stale-middle-node-refused",false},{"forged-facts-refused",false}};
    try{
        DebugRegistryScope scope(state_->registry);TDF_Label fixture;
        if(!installSyntheticFixture(metersPerUnit,fixture))return result;result["fixture-installed"]=true;
        composite_recipe::Record before;if(!composite_recipe::Read(state_->document,fixture,before)||!before.value)return result;
        const auto* feature=std::get_if<composite_recipe::FeatureNode>(&before.value->definition.nodes[1].value);if(!feature)return result;
        CompleteFence observedFence;composite_recipe::Definition observedGraph;std::vector<SourceValue> observedSources;
        state_->carrier=fixture;
        if(!state_->capture(observedFence,observedGraph,observedSources))return result;
        ReplayBudget observationBudget;const ReplayResult observedReplay=ReplayDetached(
            observedGraph,state_->registry,state_->sources,observedSources,observationBudget);
        retained_recipe::GraphCurrentnessFacts facts;facts.owner=observedGraph.owner;facts.outputNode=observedGraph.outputNode;
        facts.graph=observedFence.graph;facts.outputShape=observedFence.output;facts.readSet=observedFence.metadata;
        facts.sourceMetadata=observedFence.metadata;facts.units=observedFence.revision.ownerPlacement;
        facts.dependencyFence=observedFence.history;facts.ownerNonce=1;facts.persistenceInstalled=true;
        for(const auto& item:observedReplay.observations)facts.orderedNodes.push_back({item.node,item.feature,
            item.key.kind,item.key.codecVersion,item.inputs,item.payload,item.output,item.proof});
        const bool factsValid=observedReplay.replayed()&&retained_recipe::Valid(facts,observedGraph,state_->registry);
        composite_recipe::Definition staleGraph=observedGraph;
        std::get<composite_recipe::FeatureNode>(staleGraph.nodes[1].value).parameters={9};
        result["stale-middle-node-refused"]=factsValid&&!retained_recipe::Valid(facts,staleGraph,state_->registry);
        auto forged=facts;forged.ownerNonce=0;
        result["forged-facts-refused"]=factsValid&&!retained_recipe::Valid(forged,observedGraph,state_->registry);
        MutationRequest request;request.kind=MutationKind::EditFeature;
        request.target={before.value->definition.owner,feature->node,feature->feature};request.codec={feature->kind,feature->codecVersion};request.typedParameters={2};
        OwnerReceipt preparedReceipt;const int historyBefore=state_->document->GetAvailableUndos();
        const auto change=prepare(fixture,request,ReplayBudget(),preparedReceipt);
        composite_recipe::Record stillBefore;result["prepare-read-only"]=change&&preparedReceipt.reason=="prepared"
            &&!state_->document->HasOpenCommand()&&state_->document->GetAvailableUndos()==historyBefore
            &&composite_recipe::Read(state_->document,fixture,stillBefore)&&stillBefore.value&&stillBefore.value->bytes==before.value->bytes;
        if(!change)return result;const OwnerReceipt committed=apply(change);
        composite_recipe::Record after;if(!composite_recipe::Read(state_->document,fixture,after)||!after.value)return result;
        result["one-command"]=committed.outcome==OwnerOutcome::Committed&&committed.measuredHistoryDelta==1
            &&state_->document->GetAvailableUndos()==historyBefore+1&&after.value->bytes!=before.value->bytes;
        const bool undo=state_->wrapper.undo();composite_recipe::Record undone;
        const bool undoExact=undo&&composite_recipe::Read(state_->document,fixture,undone)&&undone.value&&undone.value->bytes==before.value->bytes;
        const bool redo=state_->wrapper.redo();composite_recipe::Record redone;
        result["undo-redo-exact"]=undoExact&&redo&&composite_recipe::Read(state_->document,fixture,redone)&&redone.value&&redone.value->bytes==after.value->bytes;
        MutationRequest invalid=request;invalid.typedParameters={0};OwnerReceipt refused;
        const int historyAtRefusal=state_->document->GetAvailableUndos();const auto noChange=prepare(fixture,invalid,ReplayBudget(),refused);
        composite_recipe::Record refusedRead;result["suffix-failure-no-mutation"]=!noChange&&!state_->document->HasOpenCommand()
            &&state_->document->GetAvailableUndos()==historyAtRefusal&&composite_recipe::Read(state_->document,fixture,refusedRead)
            &&refusedRead.value&&refusedRead.value->bytes==after.value->bytes;
        NSString* base=[NSTemporaryDirectory() stringByAppendingPathComponent:
            [NSString stringWithFormat:@"p1-registry-%@.p1-fixture",NSUUID.UUID.UUIDString]];
        const std::string saved=state_->wrapper.save(base.UTF8String);if(saved.empty())return result;
        Handle(OcctDocument) reopened=new OcctDocument();Core3DDefineSafeBinXCAFFormat(reopened->myApp);
        Handle(TDocStd_Document) candidate;Core3DBeginSafeBinaryRead();
        const PCDM_ReaderStatus status=reopened->myApp->Open(TCollection_ExtendedString(saved.c_str(),Standard_True),candidate);
        if(!Core3DSafeBinaryReadWasRejected()&&status==PCDM_RS_OK&&!candidate.IsNull()){
            reopened->myOcafDoc=candidate;reopened->ObserveSuccessfulNativeDocumentAdoption();
            candidate->SetUndoLimit(OcctDocument::kNativeSessionUndoLimit);
            std::vector<composite_recipe::Record> records;if(composite_recipe::ReadAll(candidate,records)&&records.size()==1){
                OcafOwnerService cold(*reopened,state_->registry,state_->sources);const auto* coldFeature=std::get_if<composite_recipe::FeatureNode>(&records[0].value->definition.nodes[1].value);
                if(coldFeature){MutationRequest later=request;later.target={records[0].value->definition.owner,coldFeature->node,coldFeature->feature};later.typedParameters={3};
                    OwnerReceipt laterReceipt;const auto laterChange=cold.prepare(records[0].owner,later,ReplayBudget(),laterReceipt);
                    const OwnerReceipt laterApplied=laterChange?cold.apply(laterChange):OwnerReceipt();
                    result["cold-reopen-later-edit"]=laterApplied.outcome==OwnerOutcome::Committed&&laterApplied.measuredHistoryDelta==1;}
            }
            reopened->CloseNativeSession();
        }
        const RegistryView* override=DebugRegistryOverride();DebugRegistryOverride()=nullptr;
        Handle(OcctDocument) releaseReader=new OcctDocument();Core3DDefineSafeBinXCAFFormat(releaseReader->myApp);
        Handle(TDocStd_Document) refusedDocument;Core3DBeginSafeBinaryRead();
        const PCDM_ReaderStatus refusedStatus=releaseReader->myApp->Open(TCollection_ExtendedString(saved.c_str(),Standard_True),refusedDocument);
        result["release-registry-refuses"]=Core3DSafeBinaryReadWasRejected()||refusedStatus!=PCDM_RS_OK||refusedDocument.IsNull();
        DebugRegistryOverride()=override;releaseReader->CloseNativeSession();return result;
    }catch(...){return result;}
}
#endif
} // namespace core3d::retained_feature

namespace core3d::part_boolean::owner {
namespace {
std::atomic<std::uint64_t> PartBooleanOwnerNonce{1};
const Standard_GUID& PartBooleanCommandMarkerID() {
    static const Standard_GUID value("813CDCDC-C155-4AFE-92F5-F31A3486292F");
    return value;
}
std::vector<std::uint8_t> Bytes(const std::string& value) {
    return {value.begin(), value.end()};
}
std::vector<std::uint8_t> Bytes(std::uint64_t value) {
    std::vector<std::uint8_t> output(8);
    for (std::size_t index = 0; index < output.size(); ++index)
        output[index] = std::uint8_t(value >> (index * 8));
    return output;
}
bool ShapeBytes(const TopoDS_Shape& shape, std::vector<std::uint8_t>& output) noexcept {
    std::string value;
    if (!retained_part_boolean::ExactShapeBytes(shape, value)) return false;
    output = Bytes(value); return true;
}
bool HashBytes(const std::vector<std::uint8_t>& bytes, retained_recipe::Digest& digest) noexcept {
    return !bytes.empty() && retained_solid::Hash(bytes, digest);
}
void Append(std::vector<std::uint8_t>& destination, const retained_recipe::UUID& value) {
    destination.insert(destination.end(), value.begin(), value.end());
}
void Append(std::vector<std::uint8_t>& destination, double value) {
    const std::uint64_t bits = retained_solid::Bits(value);
    const auto bytes = Bytes(bits);
    destination.insert(destination.end(), bytes.begin(), bytes.end());
}
void Append(std::vector<std::uint8_t>& destination, const std::string& value) {
    const auto size = Bytes(std::uint64_t(value.size()));
    destination.insert(destination.end(), size.begin(), size.end());
    destination.insert(destination.end(), value.begin(), value.end());
}
XCAFDoc_VisMaterialPBR PBR(const MaterialValue& value) {
    XCAFDoc_VisMaterialPBR material;
    material.BaseColor = Quantity_ColorRGBA(
        Quantity_Color(value.baseColorSRGB[0], value.baseColorSRGB[1],
                       value.baseColorSRGB[2], Quantity_TOC_sRGB),
        float(value.baseColorSRGB[3]));
    material.Metallic = float(value.metallic);
    material.Roughness = float(value.roughness);
    return material;
}
bool AppendPBR(std::vector<std::uint8_t>& output,
               const XCAFDoc_VisMaterialPBR& material) {
    if (!material.IsDefined || !material.BaseColorTexture.IsNull()
        || !material.MetallicRoughnessTexture.IsNull()
        || !material.EmissiveTexture.IsNull()
        || !material.OcclusionTexture.IsNull()
        || !material.NormalTexture.IsNull()) return false;
    const Quantity_Color& color = material.BaseColor.GetRGB();
    for (double value : {double(color.Red()), double(color.Green()),
                         double(color.Blue()), double(material.BaseColor.Alpha()),
                         double(material.Metallic), double(material.Roughness),
                         double(material.EmissiveFactor.x()),
                         double(material.EmissiveFactor.y()),
                         double(material.EmissiveFactor.z()),
                         double(material.RefractionIndex)}) Append(output, value);
    return true;
}
bool PreparedFaceMaterialManifest(
    const std::vector<PreparedFaceMaterial>& bindings,
    const Definition& definition,
    std::vector<std::uint8_t>& output) {
    output.clear();
    try {
        std::map<std::string, std::uint16_t> sorted;
        for (const PreparedFaceMaterial& binding : bindings)
            if (binding.materialIndex >= definition.materials.size()
                || !sorted.emplace(binding.stableKey, binding.materialIndex).second)
                return false;
        for (const auto& [key, materialIndex] : sorted) {
            Append(output, key);
            if (!AppendPBR(output, PBR(definition.materials[materialIndex]))) return false;
        }
        return !output.empty();
    } catch (...) { output.clear(); return false; }
}
bool CaptureFaceMaterialManifest(
    const Handle(TDocStd_Document)& document, const TDF_Label& carrier,
    std::vector<std::uint8_t>& exact,
    std::vector<std::uint8_t>& semantic,
    std::size_t& count) {
    exact.clear(); semantic.clear(); count = 0;
    try {
        const Handle(XCAFDoc_ShapeTool) shapes =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        const Handle(XCAFDoc_VisMaterialTool) materials =
            XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
        if (shapes.IsNull() || materials.IsNull()) return false;
        struct Row { std::string entry; XCAFDoc_VisMaterialPBR material; };
        std::map<std::string, Row> rows;
        for (TDF_ChildIterator it(carrier, Standard_True); it.More(); it.Next()) {
            const TDF_Label label = it.Value();
            const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(label);
            if (shape.IsNull() || shape.ShapeType() != TopAbs_FACE) continue;
            const Handle(XCAFDoc_VisMaterial) stored =
                XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
            if (stored.IsNull() || !stored->HasPbrMaterial()) continue;
            std::string key;
            if (!retained_part_boolean::ExactShapeBytes(shape, key)) return false;
            TCollection_AsciiString entry;
            TDF_Tool::Entry(label, entry);
            if (!rows.emplace(key, Row{entry.ToCString(), stored->PbrMaterial()}).second)
                return false;
        }
        for (const auto& [key, row] : rows) {
            Append(semantic, key);
            if (!AppendPBR(semantic, row.material)) return false;
            Append(exact, key); Append(exact, row.entry);
            if (!AppendPBR(exact, row.material)) return false;
        }
        count = rows.size();
        return true;
    } catch (...) { exact.clear(); semantic.clear(); count = 0; return false; }
}
HistoryWitness ObserveHistory(const Handle(TDocStd_Document)& document,
                              std::uint64_t serial) {
    HistoryWitness value;
    value.transaction = document->GetData()->Transaction();
    value.dataTime = document->GetData()->Time();
    value.undoCount = document->GetAvailableUndos();
    value.redoCount = document->GetAvailableRedos();
    value.undoLimit = document->GetUndoLimit();
    value.observationSerial = serial;
    const auto append = [](const TDF_DeltaList& list,
                           std::vector<std::array<std::uint64_t, 2>>& output) {
        for (TDF_ListIteratorOfDeltaList it(list); it.More(); it.Next()) {
            const Handle(TDF_Delta)& delta = it.Value();
            output.push_back({std::uint64_t(delta->BeginTime()), std::uint64_t(delta->EndTime())});
        }
    };
    append(document->GetUndos(), value.undoTimes);
    append(document->GetRedos(), value.redoTimes);
    return value;
}
retained_part_boolean::OperandReadSet Reads(const retained_recipe::OwnerSnapshot& snapshot) {
    return {snapshot.fence.dependencies.at(0), snapshot.fence.dependencies.at(1)};
}
} // namespace

PartBooleanOwner::PartBooleanOwner(OcctDocument& owner) noexcept
    : owner_(owner), document_(owner.Document()),
      documentIdentity_(document_.IsNull() ? nullptr : document_.get()),
      nonce_(PartBooleanOwnerNonce.fetch_add(1, std::memory_order_relaxed)) {}
PartBooleanOwner::~PartBooleanOwner() = default;

bool PartBooleanOwner::boundTo(const Handle(TDocStd_Document)& value) const noexcept {
    return !value.IsNull() && value.get() == documentIdentity_
        && !document_.IsNull() && document_.get() == documentIdentity_
        && owner_.Document().get() == documentIdentity_;
}
void PartBooleanOwner::retireForDocumentReplacement() noexcept {
    activeSession_ = 0; recoverySession_ = 0;
    recoveryBaseline_ = {}; recoveryExpected_ = {}; recoveryHistoryBefore_ = {};
    document_.Nullify(); documentIdentity_ = nullptr;
}
Receipt PartBooleanOwner::result(Outcome outcome, const char* reason,
                                 std::uint64_t session, Standard_Integer delta) const noexcept {
    return {outcome, reason, delta, session,
        document_.IsNull() || !document_->HasOpenCommand(), blocksOtherWork()};
}
Receipt PartBooleanOwner::refuse(const char* reason) const noexcept {
    return result(Outcome::Rejected, reason, activeSession_ ? activeSession_ : recoverySession_);
}
Receipt PartBooleanOwner::retire(Outcome outcome, const char* reason,
                                 Standard_Integer delta) noexcept {
    const std::uint64_t session = activeSession_;
    activeSession_ = 0;
    return result(outcome, reason, session, delta);
}

bool PartBooleanOwner::captureNativeState(const TDF_Label& carrier,
                                          DocumentSnapshot& output,
                                          composite_recipe::Record* recordOut) const noexcept {
    output = {};
    try {
        if (!boundTo(owner_.Document()) || carrier.IsNull()
            || carrier.Data() != document_->GetData()) return false;
        composite_recipe::Record record;
        if (!composite_recipe::Read(document_, carrier, record) || !record.value) return false;
        std::vector<std::uint8_t> graph;
        if (!composite_recipe::Encode(record.value->definition, graph)) return false;
        output.scopes.push_back(std::move(graph));
        std::vector<std::uint8_t> shape;
        if (!ShapeBytes(record.current, shape)) return false;
        output.scopes.push_back(std::move(shape));
        for (const TopoDS_Shape& source : record.value->sourceShapes) {
            if (!ShapeBytes(source, shape)) return false;
            output.scopes.push_back(std::move(shape));
        }
        if (record.value->sourceShapes.size() != 2) return false;
        std::vector<std::uint8_t> identities;
        Append(identities, record.value->definition.owner.document);
        Append(identities, record.value->definition.owner.entity);
        Append(identities, record.value->definition.owner.definition);
        Append(identities, record.value->definition.outputNode);
        for (const auto& node : record.value->definition.nodes) {
            Append(identities, composite_recipe::NodeID(node));
            if (const auto* source = std::get_if<composite_recipe::SourceNode>(&node.value)) {
                Append(identities, source->original.entity);
                Append(identities, source->original.definition);
                Append(identities, source->original.sourceFeature);
            } else Append(identities, std::get<composite_recipe::FeatureNode>(node.value).feature);
        }
        output.scopes.push_back(std::move(identities));
        std::vector<std::uint8_t> metadata;
        Handle(TDataStd_Name) name;
        if (carrier.FindAttribute(TDataStd_Name::GetID(), name) && !name.IsNull()) {
            const TCollection_AsciiString ascii(name->Get());
            const char* text = ascii.ToCString();
            metadata.insert(metadata.end(), text, text + ascii.Length());
        }
        OcctObjectVisibilityState visibility;
        if (!owner_.CaptureObjectVisibilityStateForLabel(carrier, visibility)) return false;
        metadata.push_back(visibility.invisibleAttributePresent ? 1 : 0);
        metadata.push_back(visibility.layerLinkPresent ? 1 : 0);
        metadata.push_back(std::uint8_t(visibility.layers.size()));
        for (bool hidden : visibility.layerInvisibleAttributePresent)
            metadata.push_back(hidden ? 1 : 0);
        double unit = 0;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document_, unit)) return false;
        const auto unitBits = retained_solid::Bits(unit);
        for (unsigned shift = 0; shift < 64; shift += 8) metadata.push_back(std::uint8_t(unitBits >> shift));
        output.scopes.push_back(std::move(metadata));
        std::vector<std::uint8_t> unrelated;
        Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(document_->Main());
        TDF_LabelSequence freeShapes; shapes->GetFreeShapes(freeShapes);
        for (Standard_Integer index = 1; index <= freeShapes.Length(); ++index) {
            const TDF_Label label = freeShapes.Value(index);
            unrelated.push_back(std::uint8_t(label.Tag() & 0xff));
            if (!label.IsEqual(carrier)) {
                std::vector<std::uint8_t> other;
                if (!ShapeBytes(XCAFDoc_ShapeTool::GetShape(label), other)) return false;
                unrelated.insert(unrelated.end(), other.begin(), other.end());
            }
        }
        output.scopes.push_back(std::move(unrelated));
        Handle(TDataStd_Integer) marker;
        const bool marked = document_->Main().FindAttribute(PartBooleanCommandMarkerID(), marker)
            && !marker.IsNull();
        output.scopes.push_back(Bytes(marked ? std::uint64_t(marker->Get()) : 0));
        std::size_t faceMaterialCount = 0;
        if (!CaptureFaceMaterialManifest(document_, carrier,
                output.faceMaterialLabels, output.faceMaterials,
                faceMaterialCount)) return false;
        output.graphCensus = record.value->definition.nodes.size()
            + std::size_t(freeShapes.Length());
        output.modelRevision = std::uint64_t(document_->GetData()->Time());
        output.history = ObserveHistory(document_, commandObservationSerial_);
        if (recordOut) *recordOut = std::move(record);
        return true;
    } catch (...) { output = {}; return false; }
}

bool PartBooleanOwner::resolveAnalytic(
    const TDF_Label& carrier, composite_recipe::Record& record,
    retained_recipe::OwnerSnapshot& snapshot, AnalyticDefinition& analytic,
    retained_recipe::NativeCurrentnessFacts& facts) const noexcept {
    try {
        DocumentSnapshot native;
        if (!captureNativeState(carrier, native, &record)
            || record.value->definition.nodes.size() != 3) return false;
        const auto* feature = std::get_if<composite_recipe::FeatureNode>(
            &record.value->definition.nodes.back().value);
        if (!feature || feature->kind != composite_recipe::PartBooleanFeatureKind
            || feature->codecVersion != composite_recipe::PartBooleanFeatureCodec
            || !DecodeAnalytic(feature->parameters, analytic)) return false;
        retained_recipe::RevisionFence fence;
        fence.documentGeneration = nonce_;
        fence.modelRevision = native.modelRevision ? native.modelRevision : 1;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document_, fence.effectiveMetersPerUnit)
            || !HashBytes(native.scopes[1], fence.ownerShape)
            || !HashBytes(native.scopes[0], fence.ownerRecipe)
            || !HashBytes(native.scopes[5], fence.ownerPlacement)
            || !HashBytes(native.scopes[5], fence.ownerMaterial)) return false;
        for (const auto& node : record.value->definition.nodes) {
            const auto* source = std::get_if<composite_recipe::SourceNode>(&node.value);
            if (!source) continue;
            fence.dependencies.push_back({{record.value->definition.owner, source->node,
                source->original.sourceFeature}, source->commitments.geometry,
                source->commitments.recipe, source->commitments.placement,
                source->commitments.material, source->commitments.groups});
        }
        retained_recipe::OwnerSnapshot structural = retained_recipe::Snapshot(
            record.value->definition, fence);
        if (structural.sources.size() != 2) return false;
        const auto reads = Reads(structural);
        const auto built = build::BuildAnalytic(analytic, fence.effectiveMetersPerUnit, reads, reads);
        const auto proof = correspondence::ProveAnalytic(
            analytic, built, fence.effectiveMetersPerUnit);
        const auto fixed = rebuild::CheckAnalytic(analytic, fence.effectiveMetersPerUnit, reads);
        std::vector<std::uint8_t> rebuilt;
        if (!proof.proven() || !fixed.fixedPoint()
            || !ShapeBytes(built.candidate.solid, rebuilt)
            || rebuilt != native.scopes[1]) return false;
        facts.owner = record.value->definition.owner;
        facts.outputNode = record.value->definition.outputNode;
        facts.feature = feature->feature;
        facts.ownerNonce = nonce_;
        facts.builderInstalled = built.complete;
        facts.proofInstalled = proof.proven() && fixed.fixedPoint();
        facts.ownerInstalled = boundTo(document_);
        facts.persistenceInstalled = record.value->bytes == native.scopes[0]
            && record.value->sourceShapes.size() == 2;
        if (!HashBytes(native.scopes[0], facts.graph)
            || !HashBytes(native.scopes[1], facts.outputShape)) return false;
        std::vector<std::uint8_t> readSet;
        for (const auto& dependency : fence.dependencies) {
            Append(readSet, dependency.locator.node);
            readSet.insert(readSet.end(), dependency.geometry.begin(), dependency.geometry.end());
            readSet.insert(readSet.end(), dependency.recipe.begin(), dependency.recipe.end());
            readSet.insert(readSet.end(), dependency.placement.begin(), dependency.placement.end());
            readSet.insert(readSet.end(), dependency.material.begin(), dependency.material.end());
            readSet.insert(readSet.end(), dependency.groups.begin(), dependency.groups.end());
        }
        if (!HashBytes(readSet, facts.readSet)) return false;
        snapshot = retained_recipe::Snapshot(record.value->definition, fence, &facts);
        retained_recipe::AdmissionRule rule;
        rule.operation = retained_recipe::OperationKind::EditInput;
        rule.featureKind = composite_recipe::PartBooleanFeatureKind;
        rule.featureCodecVersion = composite_recipe::PartBooleanFeatureCodec;
        rule.selectorVersion = analytic.versions.selector;
        rule.proofProfile = analytic.versions.proof;
        HashBytes(feature->parameters, rule.parameterBounds);
        rule.nativeBuilderInstalled = facts.builderInstalled;
        rule.nativeProofInstalled = facts.proofInstalled;
        rule.nativeOwnerInstalled = facts.ownerInstalled;
        rule.retainedInputsPersistenceInstalled = facts.persistenceInstalled;
        for (const auto& source : snapshot.sources) rule.orderedSourceKinds.push_back(source.recipe.kind);
        const auto admitted = retained_recipe::Evaluate(snapshot, rule);
        return admitted.admitted() && !admitted.detachedCandidateOnly;
    } catch (...) { return false; }
}

bool PartBooleanOwner::resolveShell(
    const TDF_Label& carrier, composite_recipe::Record& record,
    retained_recipe::OwnerSnapshot& snapshot, ShellEditDefinition& shell,
    retained_recipe::NativeCurrentnessFacts& facts) const noexcept {
    try {
        DocumentSnapshot native;
        if (!captureNativeState(carrier, native, &record)
            || record.value->definition.nodes.size() != 3) return false;
        const auto* feature = std::get_if<composite_recipe::FeatureNode>(
            &record.value->definition.nodes.back().value);
        build::ShellCompositeRequest request;
        if (!feature || feature->kind != composite_recipe::PartBooleanFeatureKind
            || feature->codecVersion != composite_recipe::PartBooleanShellFeatureCodec
            || !build::DecodeShellComposite(record.value->definition, request)) return false;
        retained_recipe::RevisionFence fence;
        fence.documentGeneration = nonce_;
        fence.modelRevision = native.modelRevision ? native.modelRevision : 1;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document_, fence.effectiveMetersPerUnit)
            || !HashBytes(native.scopes[1], fence.ownerShape)
            || !HashBytes(native.scopes[0], fence.ownerRecipe)
            || !HashBytes(native.scopes[5], fence.ownerPlacement)
            || !HashBytes(native.scopes[5], fence.ownerMaterial)) return false;
        for (const auto& node : record.value->definition.nodes) {
            const auto* source = std::get_if<composite_recipe::SourceNode>(&node.value);
            if (!source) continue;
            fence.dependencies.push_back({{record.value->definition.owner, source->node,
                source->original.sourceFeature}, source->commitments.geometry,
                source->commitments.recipe, source->commitments.placement,
                source->commitments.material, source->commitments.groups});
        }
        retained_recipe::OwnerSnapshot structural = retained_recipe::Snapshot(
            record.value->definition, fence);
        if (structural.sources.size() != 2) return false;
        const auto reads = Reads(structural);
        const auto built = build::BuildShellComposite(request, reads, reads);
        const auto proof = correspondence::ProveShellComposite(request, built);
        const auto fixed = rebuild::CheckShellComposite(record.value->definition, reads);
        std::vector<std::uint8_t> rebuilt, left, right;
        if (!proof.proven() || !fixed.fixedPoint()
            || !ShapeBytes(built.candidate.solid, rebuilt)
            || !ShapeBytes(built.sources[0], left) || !ShapeBytes(built.sources[1], right)
            || rebuilt != native.scopes[1] || left != native.scopes[2]
            || right != native.scopes[3]) return false;
        facts.owner = record.value->definition.owner;
        facts.outputNode = record.value->definition.outputNode;
        facts.feature = feature->feature;
        facts.ownerNonce = nonce_;
        facts.builderInstalled = built.complete;
        facts.proofInstalled = proof.proven() && fixed.fixedPoint();
        facts.ownerInstalled = boundTo(document_);
        facts.persistenceInstalled = record.value->bytes == native.scopes[0]
            && record.value->sourceShapes.size() == 2;
        if (!HashBytes(native.scopes[0], facts.graph)
            || !HashBytes(native.scopes[1], facts.outputShape)) return false;
        std::vector<std::uint8_t> readSet;
        for (const auto& dependency : fence.dependencies) {
            Append(readSet, dependency.locator.node);
            readSet.insert(readSet.end(), dependency.geometry.begin(), dependency.geometry.end());
            readSet.insert(readSet.end(), dependency.recipe.begin(), dependency.recipe.end());
            readSet.insert(readSet.end(), dependency.placement.begin(), dependency.placement.end());
            readSet.insert(readSet.end(), dependency.material.begin(), dependency.material.end());
            readSet.insert(readSet.end(), dependency.groups.begin(), dependency.groups.end());
        }
        if (!HashBytes(readSet, facts.readSet)) return false;
        snapshot = retained_recipe::Snapshot(record.value->definition, fence, &facts);
        retained_recipe::AdmissionRule rule;
        rule.operation = retained_recipe::OperationKind::EditInput;
        rule.featureKind = composite_recipe::PartBooleanFeatureKind;
        rule.featureCodecVersion = composite_recipe::PartBooleanShellFeatureCodec;
        rule.selectorVersion = request.definition.versions.selector;
        rule.proofProfile = request.definition.versions.proof;
        HashBytes(feature->parameters, rule.parameterBounds);
        rule.nativeBuilderInstalled = facts.builderInstalled;
        rule.nativeProofInstalled = facts.proofInstalled;
        rule.nativeOwnerInstalled = facts.ownerInstalled;
        rule.retainedInputsPersistenceInstalled = facts.persistenceInstalled;
        for (const auto& source : snapshot.sources)
            rule.orderedSourceKinds.push_back(source.recipe.kind);
        const auto admitted = retained_recipe::Evaluate(snapshot, rule);
        if (!admitted.admitted() || admitted.detachedCandidateOnly) return false;
        shell.feature = std::move(request.definition);
        shell.profiles = std::move(request.profiles);
        shell.placements = std::move(request.placements);
        return true;
    } catch (...) { return false; }
}

CaptureOutcome PartBooleanOwner::capture(const TDF_Label& carrier) noexcept {
    CaptureOutcome output;
    try {
        if (!boundTo(owner_.Document()) || activeSession_ || recoverySession_
            || document_->HasOpenCommand()) { output.receipt = refuse("capture-owner-busy"); return output; }
        const auto opening = owner_.CaptureNativePlanningStamp(true);
        if (!opening) { output.receipt = refuse("capture-native-authority-busy"); return output; }
        composite_recipe::Record record;
        retained_recipe::OwnerSnapshot snapshot;
        AnalyticDefinition analytic;
        ShellEditDefinition shell;
        retained_recipe::NativeCurrentnessFacts facts;
        DocumentSnapshot before, after;
        bool isShell = false;
        bool resolved = resolveAnalytic(carrier, record, snapshot, analytic, facts);
        if (!resolved) {
            record = {}; snapshot = {}; facts = {};
            isShell = resolveShell(carrier, record, snapshot, shell, facts);
            resolved = isShell;
        }
        if (!resolved
            || !captureNativeState(carrier, before)
            || !captureNativeState(carrier, after) || !(before == after)) {
            output.receipt = refuse("capture-incomplete-or-noncurrent"); return output;
        }
        auto value = std::make_shared<Capture>();
        value->ownerNonce = nonce_; value->session = ++nextSession_;
        value->request = ++nextRequest_; value->documentIdentity = documentIdentity_;
        value->opening = *opening;
        value->carrier = carrier; value->record = record.label;
        value->snapshot = std::move(snapshot); value->graph = record.value->definition;
        value->analytic = std::move(analytic); value->sourceShapes = record.value->sourceShapes;
        value->shell = isShell; value->shellEdit = std::move(shell);
        value->baseline = std::move(before); value->currentness = facts;
        activeSession_ = value->session; output.handle = std::move(value);
        output.receipt = result(Outcome::Unchanged, "captured", activeSession_);
        return output;
    } catch (...) { output.receipt = refuse("capture-exception"); return output; }
}

bool PartBooleanOwner::describe(const TDF_Label& carrier, AnalyticDefinition& output) const noexcept {
    try {
        if (carrier.IsNull() || activeSession_ != 0 || recoverySession_ != 0
            || document_.IsNull() || document_->HasOpenCommand()) return false;
        DocumentSnapshot before,after;composite_recipe::Record record;
        retained_recipe::OwnerSnapshot snapshot;retained_recipe::NativeCurrentnessFacts currentness;
        AnalyticDefinition value;
        if (!captureNativeState(carrier,before) || !resolveAnalytic(carrier,record,snapshot,value,currentness)
            || !captureNativeState(carrier,after) || !(before==after)) return false;
        output=std::move(value);return true;
    } catch (...) { return false; }
}

bool PartBooleanOwner::describeShell(
    const TDF_Label& carrier, ShellEditDefinition& output) const noexcept {
    try {
        if (carrier.IsNull() || activeSession_ != 0 || recoverySession_ != 0
            || document_.IsNull() || document_->HasOpenCommand()) return false;
        DocumentSnapshot before, after; composite_recipe::Record record;
        retained_recipe::OwnerSnapshot snapshot;
        retained_recipe::NativeCurrentnessFacts currentness;
        ShellEditDefinition value;
        if (!captureNativeState(carrier, before)
            || !resolveShell(carrier, record, snapshot, value, currentness)
            || !captureNativeState(carrier, after) || !(before == after)) return false;
        output = std::move(value); return true;
    } catch (...) { return false; }
}

bool PartBooleanOwner::buildPrepared(
    const Capture& capture, const AnalyticDefinition& requested,
    PreparedBooleanDocumentChange& prepared) const noexcept {
    try {
        if (capture.shell || !Valid(requested)
            || requested.inputs[0].rootNode != capture.analytic.inputs[0].rootNode
            || requested.inputs[1].rootNode != capture.analytic.inputs[1].rootNode
            || requested.inputs[0].originalSourceFeature != capture.analytic.inputs[0].originalSourceFeature
            || requested.inputs[1].originalSourceFeature != capture.analytic.inputs[1].originalSourceFeature)
            return false;
        double unit = 0;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document_, unit)) return false;
        const auto reads = Reads(capture.snapshot);
        auto built = build::BuildAnalytic(requested, unit, reads, reads);
        const auto proof = correspondence::ProveAnalytic(requested, built, unit);
        const auto fixed = rebuild::CheckAnalytic(requested, unit, reads);
        if (!built.complete || !proof.proven() || !fixed.fixedPoint()) return false;
        prepared.analytic = requested;
        prepared.graph = capture.graph;
        prepared.sources.assign(built.sources.begin(), built.sources.end());
        prepared.result = built.candidate.solid;
        std::size_t sourceIndex = 0;
        for (auto& node : prepared.graph.nodes) {
            auto* source = std::get_if<composite_recipe::SourceNode>(&node.value);
            if (!source) continue;
            retained_recipe::Digest geometry;
            const std::vector<std::uint8_t> bytes = Bytes(built.sourceBytes[sourceIndex]);
            if (!HashBytes(bytes, geometry)) return false;
            source->commitments.geometry = geometry;
            prepared.analytic.inputs[sourceIndex].commitments.geometry = geometry;
            ++sourceIndex;
        }
        auto* feature = std::get_if<composite_recipe::FeatureNode>(&prepared.graph.nodes.back().value);
        if (!feature || !EncodeAnalytic(prepared.analytic, feature->parameters)) return false;
        std::vector<std::uint8_t> graph, resultBytes;
        if (!composite_recipe::Encode(prepared.graph, graph)
            || !ShapeBytes(prepared.result, resultBytes)) return false;
        prepared.expected = capture.baseline;
        prepared.expected.scopes[0] = std::move(graph);
        prepared.expected.scopes[1] = std::move(resultBytes);
        prepared.expected.scopes[2] = Bytes(built.sourceBytes[0]);
        prepared.expected.scopes[3] = Bytes(built.sourceBytes[1]);
        prepared.expected.scopes.back() = Bytes(capture.session);
        prepared.currentness = capture.currentness;
        prepared.currentness.ownerNonce = nonce_;
        prepared.currentness.builderInstalled = built.complete;
        prepared.currentness.proofInstalled = proof.proven() && fixed.fixedPoint();
        if (!HashBytes(prepared.expected.scopes[0], prepared.currentness.graph)
            || !HashBytes(prepared.expected.scopes[1], prepared.currentness.outputShape)) return false;
        return true;
    } catch (...) { return false; }
}

bool PartBooleanOwner::buildPrepared(
    const Capture& capture, const ShellEditDefinition& requested,
    PreparedBooleanDocumentChange& prepared) const noexcept {
    try {
        if (!capture.shell || !Valid(requested.feature)
            || requested.feature.inputs[0].rootNode
                != capture.shellEdit.feature.inputs[0].rootNode
            || requested.feature.inputs[1].rootNode
                != capture.shellEdit.feature.inputs[1].rootNode
            || requested.feature.inputs[0].originalSourceFeature
                != capture.shellEdit.feature.inputs[0].originalSourceFeature
            || requested.feature.inputs[1].originalSourceFeature
                != capture.shellEdit.feature.inputs[1].originalSourceFeature
            || requested.feature.versions.serializer
                != capture.shellEdit.feature.versions.serializer
            || requested.feature.versions.build
                != capture.shellEdit.feature.versions.build
            || requested.feature.versions.proof
                != capture.shellEdit.feature.versions.proof
            || requested.feature.versions.selector
                != capture.shellEdit.feature.versions.selector
            || requested.feature.versions.material
                != capture.shellEdit.feature.versions.material) return false;
        build::ShellCompositeRequest request;
        request.definition = requested.feature;
        request.profiles = requested.profiles;
        request.placements = requested.placements;
        const auto reads = Reads(capture.snapshot);
        const auto built = build::BuildShellComposite(request, reads, reads);
        const auto proof = correspondence::ProveShellComposite(request, built);
        if (!built.complete || !proof.proven()) return false;
        prepared.shell = true;
        prepared.shellEdit = requested;
        prepared.graph = capture.graph;
        prepared.sources.assign(built.sources.begin(), built.sources.end());
        prepared.result = built.candidate.solid;
        for (const correspondence::FaceBinding& face : proof.faces)
            prepared.faceMaterials.push_back({face.face, face.stableKey,
                face.region, face.materialIndex});
        if (prepared.faceMaterials.empty()) return false;
        std::size_t sourceIndex = 0;
        for (auto& node : prepared.graph.nodes) {
            auto* source = std::get_if<composite_recipe::SourceNode>(&node.value);
            if (!source) continue;
            if (sourceIndex >= 2) return false;
            std::vector<double> scalars;
            if (!profile::Encode(requested.profiles[sourceIndex], scalars)) return false;
            source->recipe.schema = std::uint32_t(
                profile::SchemaFor(requested.profiles[sourceIndex]));
            if (!composite_recipe::EncodeScalarRecipe(source->recipe.kind,
                    source->recipe.schema, scalars, source->recipe.bytes)) return false;
            source->inputToCarrier = requested.placements[sourceIndex];
            retained_recipe::Digest geometry, recipe, placement;
            const auto geometryBytes = Bytes(built.sourceBytes[sourceIndex]);
            std::vector<std::uint8_t> placementBytes;
            for (double scalar : source->inputToCarrier.matrix)
                Append(placementBytes, scalar);
            Append(placementBytes, source->inputToCarrier.sourceMetersPerUnit);
            Append(placementBytes, source->inputToCarrier.carrierMetersPerUnit);
            if (!HashBytes(geometryBytes, geometry)
                || !composite_recipe::Hash(source->recipe.bytes, recipe)
                || !HashBytes(placementBytes, placement)) return false;
            source->commitments.geometry = geometry;
            source->commitments.recipe = recipe;
            source->commitments.placement = placement;
            auto& binding = prepared.shellEdit.feature.inputs[sourceIndex];
            binding.commitments.geometry = geometry;
            binding.commitments.recipe = recipe;
            binding.commitments.placement = placement;
            ++sourceIndex;
        }
        auto* feature = std::get_if<composite_recipe::FeatureNode>(
            &prepared.graph.nodes.back().value);
        if (sourceIndex != 2 || !feature
            || !Encode(prepared.shellEdit.feature, feature->parameters)) return false;
        const auto fixed = rebuild::CheckShellComposite(prepared.graph, reads);
        if (!fixed.fixedPoint()) return false;
        std::vector<std::uint8_t> graph, resultBytes;
        if (!composite_recipe::Encode(prepared.graph, graph)
            || !ShapeBytes(prepared.result, resultBytes)) return false;
        prepared.expected = capture.baseline;
        prepared.expected.scopes[0] = std::move(graph);
        prepared.expected.scopes[1] = std::move(resultBytes);
        prepared.expected.scopes[2] = Bytes(built.sourceBytes[0]);
        prepared.expected.scopes[3] = Bytes(built.sourceBytes[1]);
        prepared.expected.scopes.back() = Bytes(capture.session);
        prepared.expected.faceMaterialLabels.clear();
        if (!PreparedFaceMaterialManifest(prepared.faceMaterials,
                prepared.shellEdit.feature,
                prepared.expected.faceMaterials)) return false;
        prepared.currentness = capture.currentness;
        prepared.currentness.ownerNonce = nonce_;
        prepared.currentness.builderInstalled = built.complete;
        prepared.currentness.proofInstalled = proof.proven() && fixed.fixedPoint();
        if (!HashBytes(prepared.expected.scopes[0], prepared.currentness.graph)
            || !HashBytes(prepared.expected.scopes[1],
                          prepared.currentness.outputShape)) return false;
        return true;
    } catch (...) { return false; }
}

PrepareOutcome PartBooleanOwner::prepare(const std::shared_ptr<const Capture>& capture,
                                         const AnalyticDefinition& requested,
                                         FaultPoint fault) noexcept {
    PrepareOutcome output;
    try {
        if (!capture || capture->shell || capture->ownerNonce != nonce_
            || capture->session != activeSession_
            || capture->documentIdentity != documentIdentity_ || document_->HasOpenCommand()
            || !owner_.myNativeAuthority
            || !owner_.myNativeAuthority->Matches(capture->opening, documentIdentity_, true, false)) {
            output.receipt = refuse("prepare-foreign-handle"); return output;
        }
        DocumentSnapshot before, after;
        if (!captureNativeState(capture->carrier, before) || !(before == capture->baseline)
            || fault == FaultPoint::F1DetachedDependency) {
            output.receipt = refuse("prepare-stale-or-dependent-failure"); return output;
        }
        auto value = std::make_shared<PreparedBooleanDocumentChange>();
        value->capture = capture; value->request = ++nextRequest_; value->fault = fault;
        if (!buildPrepared(*capture, requested, *value)
            || !captureNativeState(capture->carrier, after) || !(before == after)) {
            output.receipt = refuse("prepare-not-read-only-or-unproved"); return output;
        }
        output.handle = std::move(value);
        output.receipt = result(Outcome::Unchanged, "prepared", activeSession_);
        return output;
    } catch (...) { output.receipt = refuse("prepare-exception"); return output; }
}

PrepareOutcome PartBooleanOwner::prepare(
    const std::shared_ptr<const Capture>& capture,
    const ShellEditDefinition& requested, FaultPoint fault) noexcept {
    PrepareOutcome output;
    try {
        if (!capture || !capture->shell || capture->ownerNonce != nonce_
            || capture->session != activeSession_
            || capture->documentIdentity != documentIdentity_
            || document_->HasOpenCommand() || !owner_.myNativeAuthority
            || !owner_.myNativeAuthority->Matches(
                capture->opening, documentIdentity_, true, false)) {
            output.receipt = refuse("prepare-shell-foreign-handle"); return output;
        }
        DocumentSnapshot before, after;
        if (!captureNativeState(capture->carrier, before)
            || !(before == capture->baseline)
            || fault == FaultPoint::F1DetachedDependency) {
            output.receipt = refuse("prepare-shell-stale-or-dependent-failure");
            return output;
        }
        auto value = std::make_shared<PreparedBooleanDocumentChange>();
        value->capture = capture; value->request = ++nextRequest_; value->fault = fault;
        if (!buildPrepared(*capture, requested, *value)
            || !captureNativeState(capture->carrier, after) || !(before == after)) {
            output.receipt = refuse("prepare-shell-not-read-only-or-unproved");
            return output;
        }
        output.handle = std::move(value);
        output.receipt = result(Outcome::Unchanged, "prepared", activeSession_);
        return output;
    } catch (...) { output.receipt = refuse("prepare-shell-exception"); return output; }
}

bool PartBooleanOwner::stagePrepared(const PreparedBooleanDocumentChange& prepared) noexcept {
    try {
        if (!document_->HasOpenCommand() || prepared.sources.size() != 2) return false;
        const TDF_Label carrier = prepared.capture->carrier;
        Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(document_->Main());
        if (shapes.IsNull()) return false;
        std::vector<TDF_Label> previousFaceLabels;
        for (TDF_ChildIterator it(carrier, Standard_True); it.More(); it.Next()) {
            const TDF_Label label = it.Value();
            const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(label);
            if (!shape.IsNull() && shape.ShapeType() == TopAbs_FACE)
                previousFaceLabels.push_back(label);
        }
        shapes->SetShape(carrier, prepared.result);
        if (prepared.fault == FaultPoint::F5AfterShape) return false;
        auto payload = std::make_shared<composite_recipe::Payload>();
        payload->definition = prepared.graph;
        if (!composite_recipe::Encode(payload->definition, payload->bytes)) return false;
        payload->sourceShapes = prepared.sources;
        if (!owner_.StagePartBooleanPayload(prepared.capture->record, payload)) return false;
        TNaming_Builder(prepared.capture->record).Select(prepared.result, prepared.result);
        if (prepared.shell) {
        const Handle(TDF_TagSource) tags = TDF_TagSource::Set(carrier);
        Standard_Integer highWater = tags->Get();
        for (TDF_ChildIterator child(carrier, Standard_False); child.More(); child.Next())
            highWater = std::max(highWater, child.Value().Tag());
        if (highWater > std::numeric_limits<Standard_Integer>::max()
                - Standard_Integer(prepared.faceMaterials.size())) return false;
        tags->Set(highWater);
        std::vector<TDF_Label> activeFaceLabels;
        std::vector<OcctPBRMaterialUpdate> materialUpdates;
        activeFaceLabels.reserve(prepared.faceMaterials.size());
        materialUpdates.reserve(prepared.faceMaterials.size());
        for (const PreparedFaceMaterial& binding : prepared.faceMaterials) {
            if (binding.materialIndex >= prepared.shellEdit.feature.materials.size()
                || binding.face.IsNull() || !shapes->IsSubShape(carrier, binding.face))
                return false;
            const TDF_Label label = shapes->AddSubShape(carrier, binding.face);
            if (label.IsNull()) return false;
            for (const TDF_Label& existing : activeFaceLabels)
                if (existing.IsEqual(label)) return false;
            activeFaceLabels.push_back(label);
            const XCAFDoc_VisMaterialPBR material = PBR(
                prepared.shellEdit.feature.materials[binding.materialIndex]);
            materialUpdates.push_back({label, material,
                Handle(Image_Texture)(), Handle(Image_Texture)()});
        }
        const Handle(XCAFDoc_VisMaterialTool) materials =
            XCAFDoc_DocumentTool::VisMaterialTool(document_->Main());
        if (materials.IsNull()) return false;
        for (const TDF_Label& previous : previousFaceLabels) {
            const bool retained = std::any_of(activeFaceLabels.begin(),
                activeFaceLabels.end(), [&](const TDF_Label& value) {
                    return value.IsEqual(previous);
                });
            if (!retained) {
                materials->UnSetShapeMaterial(previous);
                previous.ForgetAllAttributes(Standard_True);
            }
        }
        if (materialUpdates.size() != prepared.faceMaterials.size()
            || !owner_.SaveObjectPBRMaterials(materialUpdates)) return false;
        }
        if (prepared.fault == FaultPoint::F6AfterInputs) return false;
        // Source names, groups, visibility and scalar materials are immutable
        // values in the strict analytic payload. Carrier presentation metadata
        // is deliberately preserved rather than overwritten from one input.
        TDataStd_Integer::Set(document_->Main(), PartBooleanCommandMarkerID(),
            Standard_Integer(prepared.capture->session & 0x7fffffff));
        return true;
    } catch (...) { return false; }
}

bool PartBooleanOwner::exactlyOneOwnedDelta(const HistoryWitness& before,
                                            HistoryWitness& after) const noexcept {
    try {
        after = ObserveHistory(document_, commandObservationSerial_ + 1);
        if (!after.redoTimes.empty() || after.undoTimes.empty()
            || after.transaction != before.transaction
            || after.dataTime != before.dataTime + 1) return false;
        std::vector<std::array<std::uint64_t, 2>> expected = before.undoTimes;
        if (before.undoLimit > 0 && Standard_Integer(expected.size()) >= before.undoLimit)
            expected.erase(expected.begin());
        expected.push_back(after.undoTimes.back());
        if (expected != after.undoTimes) return false;
        Handle(TDataStd_Integer) marker;
        return document_->Main().FindAttribute(PartBooleanCommandMarkerID(), marker)
            && !marker.IsNull()
            && marker->Get() == Standard_Integer(activeSession_ & 0x7fffffff)
            && after.undoTimes.back()[1] > after.undoTimes.back()[0];
    } catch (...) { return false; }
}

bool PartBooleanOwner::abortRestored(
    const PreparedBooleanDocumentChange& prepared) const noexcept {
    DocumentSnapshot restored;
    return !document_->HasOpenCommand()
        && captureNativeState(prepared.capture->carrier, restored)
        && restored == prepared.capture->baseline;
}

Receipt PartBooleanOwner::apply(
    const std::shared_ptr<const PreparedBooleanDocumentChange>& prepared) noexcept {
    if (!prepared || !prepared->capture || prepared->capture->ownerNonce != nonce_
        || prepared->capture->session != activeSession_ || prepared->request == 0)
        return refuse("apply-foreign-handle");
    DocumentSnapshot current;
    try {
        if (document_->HasOpenCommand() || !captureNativeState(prepared->capture->carrier, current)
            || !(current == prepared->capture->baseline)
            || !owner_.myNativeAuthority
            || !owner_.myNativeAuthority->Matches(
                prepared->capture->opening, documentIdentity_, true, false)
            || prepared->fault == FaultPoint::F2FinalFence)
            return refuse("apply-stale-final-fence");
        if (prepared->fault == FaultPoint::F3Stop)
            return retire(Outcome::Cancelled, "cancelled-before-command");
        recoveryHistoryBefore_ = current.history;
        document_->OpenCommand();
        if (!document_->HasOpenCommand()) return refuse("owner-command-not-opened");
        if (prepared->fault == FaultPoint::F4AfterOpen) throw std::runtime_error("F4");
        if (!stagePrepared(*prepared)) throw std::runtime_error("stage");
        DocumentSnapshot staged;
        if (!captureNativeState(prepared->capture->carrier, staged)
            || !staged.preparedSemanticEquals(prepared->expected))
            throw std::runtime_error("readback");
        if (prepared->fault == FaultPoint::F7BeforeClose) throw std::runtime_error("F7");
        const Standard_Boolean added = document_->CommitCommand();
        recoveryBaseline_ = prepared->capture->baseline;
        recoveryExpected_ = staged;
        recoverySession_ = activeSession_; activeSession_ = 0;
        if (document_->HasOpenCommand() || !added)
            return result(Outcome::RecoveryRequired, "close-outcome-unproved", recoverySession_);
        if (prepared->fault == FaultPoint::F9Reconcile)
            return result(Outcome::RecoveryRequired, "native-inspection-unavailable", recoverySession_);
        if (prepared->fault == FaultPoint::F8AfterClose) return reconcile(recoverySession_);
        DocumentSnapshot sealed; HistoryWitness observed;
        activeSession_ = recoverySession_; recoverySession_ = 0;
        const bool oneDelta = exactlyOneOwnedDelta(recoveryHistoryBefore_, observed);
        activeSession_ = 0; recoverySession_ = prepared->capture->session;
        if (!captureNativeState(prepared->capture->carrier, sealed)
            || !sealed.semanticEquals(recoveryExpected_) || !oneDelta)
            return result(Outcome::RecoveryRequired, "exact-delta-or-seal-unproved", recoverySession_);
        if (prepared->fault == FaultPoint::F10Presentation)
            return result(Outcome::RecoveryRequired, "committed-presentation-unsettled", recoverySession_);
        const std::uint64_t session = recoverySession_;
        recoverySession_ = 0; ++commandObservationSerial_;
        owner_.NotifyChanges();
        return result(Outcome::Committed, "committed", session,
            Standard_Integer(observed.observationSerial - recoveryHistoryBefore_.observationSerial));
    } catch (...) {
        try { if (!document_.IsNull() && document_->HasOpenCommand()) document_->AbortCommand(); }
        catch (...) {
            recoverySession_ = activeSession_; activeSession_ = 0;
            return result(Outcome::RecoveryRequired, "abort-threw", recoverySession_);
        }
        if (!abortRestored(*prepared)) {
            recoverySession_ = activeSession_; activeSession_ = 0;
            recoveryBaseline_ = prepared->capture->baseline;
            return result(Outcome::RecoveryRequired, "abort-restoration-unproved", recoverySession_);
        }
        return retire(Outcome::Rejected, "aborted-and-restored");
    }
}

Receipt PartBooleanOwner::cancel(std::uint64_t session) noexcept {
    if (!matchingContinuation(session) || session != activeSession_
        || document_.IsNull() || document_->HasOpenCommand()) return refuse("cancel-not-owner");
    return retire(Outcome::Cancelled, "cancelled");
}

Receipt PartBooleanOwner::reconcile(std::uint64_t session) noexcept {
    if (session == 0 || session != recoverySession_ || document_.IsNull()
        || document_->HasOpenCommand()) return refuse("reconcile-not-owner");
    DocumentSnapshot actual;
    // The carrier is resolved again from the sole valid record; no stale
    // Capture or caller callback participates in recovery.
    std::vector<composite_recipe::Record> records;
    if (!composite_recipe::ReadAll(document_, records) || records.size() != 1
        || !captureNativeState(records.front().owner, actual))
        return result(Outcome::RecoveryRequired, "reconcile-inspection-failed", session);
    HistoryWitness observed;
    activeSession_ = session; recoverySession_ = 0;
    const bool oneDelta = exactlyOneOwnedDelta(recoveryHistoryBefore_, observed);
    activeSession_ = 0; recoverySession_ = session;
    if (actual.semanticEquals(recoveryExpected_) && oneDelta) {
        recoverySession_ = 0; ++commandObservationSerial_;
        owner_.NotifyChanges();
        return result(Outcome::Committed, "committed-reconciled", session,
            Standard_Integer(observed.observationSerial - recoveryHistoryBefore_.observationSerial));
    }
    if (actual == recoveryBaseline_) {
        recoverySession_ = 0;
        return result(Outcome::Rejected, "unchanged-reconciled", session);
    }
    return result(Outcome::RecoveryRequired, "reconcile-ambiguous", session);
}

CaptureOutcome InternalBooleanOperationSession::capture(const TDF_Label& carrier) noexcept {
    if (capture_ || terminalSet_) return {{Outcome::Rejected, "session-not-idle", 0, 0, true,
        owner_.blocksOtherWork()}, {}};
    auto value = owner_.capture(carrier); capture_ = value.handle; return value;
}
PrepareOutcome InternalBooleanOperationSession::prepare(
    const AnalyticDefinition& request, FaultPoint fault) noexcept {
    if (!capture_ || prepared_ || terminalSet_) return {{Outcome::Rejected,
        "session-not-captured", 0, 0, true, owner_.blocksOtherWork()}, {}};
    auto value = owner_.prepare(capture_, request, fault); prepared_ = value.handle; return value;
}
PrepareOutcome InternalBooleanOperationSession::prepare(
    const ShellEditDefinition& request, FaultPoint fault) noexcept {
    if (!capture_ || prepared_ || terminalSet_) return {{Outcome::Rejected,
        "session-not-captured", 0, 0, true, owner_.blocksOtherWork()}, {}};
    auto value = owner_.prepare(capture_, request, fault);
    prepared_ = value.handle; return value;
}
Receipt InternalBooleanOperationSession::apply() noexcept {
    if (terminalSet_) return terminal_;
    terminal_ = owner_.apply(prepared_);
    if (terminal_.outcome == Outcome::RecoveryRequired) recoverySession_ = terminal_.session;
    else terminalSet_ = true;
    capture_.reset(); prepared_.reset(); return terminal_;
}
Receipt InternalBooleanOperationSession::cancel() noexcept {
    if (terminalSet_) return terminal_;
    terminal_ = owner_.cancel(capture_ ? capture_->session : 0); terminalSet_ = true;
    capture_.reset(); prepared_.reset(); return terminal_;
}
Receipt InternalBooleanOperationSession::reconcile() noexcept {
    if (!recoverySession_) return terminal_;
    terminal_ = owner_.reconcile(recoverySession_);
    if (terminal_.outcome != Outcome::RecoveryRequired) {
        recoverySession_ = 0; terminalSet_ = true;
    }
    return terminal_;
}

#if DEBUG
namespace {
struct LegacyCorpusCaseObservation {
    std::string key, primary, afterEdit, noopResave;
    std::uint64_t unitBits = 0;
    int storageVersion = 0;
    bool direct = false, driverControls = false;
    bool firstOpen = false, replay = false, laterEdit = false, undoRedo = false;
    bool secondOpen = false, thirdOpen = false, completeInputs = false;
    bool discardedOldHandles = false, wholeFilesEqual = false;
    std::size_t afterEditLength = 0, noopLength = 0, firstDifference = 0;
    std::string beforeGraph, reopenedGraph;
    std::map<std::string, std::string> savedRecipeSHA256;
    std::map<std::string, std::size_t> savedSourceSlotCount;
};

struct LegacyCorpusCollector {
    std::map<std::string, std::vector<std::uint8_t>> blobs;
    std::map<std::string, LegacyCorpusCaseObservation> cases;
    std::string activeCase;
    bool failed = false;

    bool add(const std::string& name, std::vector<std::uint8_t> bytes) {
        if (name.empty() || bytes.empty() || !blobs.emplace(name, std::move(bytes)).second) {
            failed = true; return false;
        }
        return true;
    }
};

thread_local LegacyCorpusCollector* N1LegacyCorpusCollector = nullptr;

std::uint64_t N1ScalarBits(double value) noexcept {
    std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits)); return bits;
}

std::string N1CorpusHex64(std::uint64_t value) {
    std::ostringstream out; out << "0x" << std::hex << std::setw(16)
        << std::setfill('0') << value; return out.str();
}

std::string N1JSONText(const std::string& value) {
    std::ostringstream out; out << '"';
    for (unsigned char byte : value) {
        if (byte == '"' || byte == '\\') out << '\\' << char(byte);
        else if (byte >= 0x20 && byte < 0x7f) out << char(byte);
        else out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << int(byte)
                 << std::dec;
    }
    out << '"'; return out.str();
}

std::string N1CorpusCaseKey(double unit, TDocStd_FormatVersion version) {
    const char* unitName = N1ScalarBits(unit) == N1ScalarBits(0.001) ? "mm"
        : N1ScalarBits(unit) == N1ScalarBits(1.0) ? "m" : nullptr;
    const char* mode = version == TDocStd_FormatVersion_CURRENT ? "direct"
        : version == TDocStd_FormatVersion_VERSION_11 ? "indexed" : nullptr;
    return unitName && mode ? std::string(unitName) + "-" + mode : std::string();
}

std::vector<std::uint8_t> N1ReadWholeFile(const std::string& path) {
    std::ifstream stream(path, std::ios::in | std::ios::binary);
    if (!stream) return {};
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(stream),
                                     std::istreambuf_iterator<char>());
}

template<std::size_t N>
std::string N1UUIDHex(const std::array<std::uint8_t, N>& value) {
    static const char hex[] = "0123456789abcdef";
    std::string out; out.reserve(value.size() * 2);
    for (std::uint8_t byte : value) { out.push_back(hex[byte >> 4]); out.push_back(hex[byte & 15]); }
    return out;
}

bool N1FeatureAnalytic(const composite_recipe::Definition&,
                       std::vector<std::uint8_t>&);

bool N1CaptureRecord(const composite_recipe::Record& record, const char* phase) {
    LegacyCorpusCollector* collector = N1LegacyCorpusCollector;
    if (!collector) return true;
    if (collector->activeCase.empty() || !record.value || record.value->sourceShapes.size() != 2) {
        collector->failed = true; return false;
    }
    const std::string prefix = "native-" + collector->activeCase + "-";
    std::vector<std::uint8_t> feature;
    if (!N1FeatureAnalytic(record.value->definition, feature)) { collector->failed = true; return false; }
    AnalyticDefinition analytic;
    if (!DecodeAnalytic(feature, analytic)) { collector->failed = true; return false; }
    std::vector<std::uint8_t> source0, source1, current;
    if (!ShapeBytes(record.value->sourceShapes[0], source0)
        || !ShapeBytes(record.value->sourceShapes[1], source1)
        || !ShapeBytes(record.current, current)) { collector->failed = true; return false; }
    std::array<std::vector<std::uint8_t>, 2> sourceRecipes;
    std::size_t recipeCount = 0;
    for (const auto& node : record.value->definition.nodes) {
        const auto* source = std::get_if<composite_recipe::SourceNode>(&node.value);
        if (!source) continue;
        if (source->shapeSlot >= sourceRecipes.size() || !sourceRecipes[source->shapeSlot].empty()) {
            collector->failed = true; return false;
        }
        sourceRecipes[source->shapeSlot] = source->recipe.bytes; ++recipeCount;
    }
    if (recipeCount != 2 || sourceRecipes[0].empty() || sourceRecipes[1].empty()) {
        collector->failed = true; return false;
    }
    const std::string suffix(phase);
    bool ok = collector->add(prefix + "sycr-" + suffix + ".bin", record.value->bytes)
        && collector->add(prefix + "source-slot-0-" + suffix + ".brep", std::move(source0))
        && collector->add(prefix + "source-slot-1-" + suffix + ".brep", std::move(source1))
        && collector->add(prefix + "current-" + suffix + ".brep", std::move(current))
        && collector->add(prefix + "source-recipe-0-" + suffix + ".bin", sourceRecipes[0])
        && collector->add(prefix + "source-recipe-1-" + suffix + ".bin", sourceRecipes[1])
        && collector->add(prefix + "feature-parameters-" + suffix + ".bin", std::move(feature));
    auto& observation = collector->cases[collector->activeCase];
    TCollection_AsciiString ownerEntry, recordEntry;
    TDF_Tool::Entry(record.owner, ownerEntry); TDF_Tool::Entry(record.label, recordEntry);
    std::ostringstream graph;
    graph << "{\"ocafLabels\":{\"owner\":" << N1JSONText(ownerEntry.ToCString())
          << ",\"record\":" << N1JSONText(recordEntry.ToCString())
          << "},\"owner\":{\"document\":\"" << N1UUIDHex(record.value->definition.owner.document)
          << "\",\"entity\":\"" << N1UUIDHex(record.value->definition.owner.entity)
          << "\",\"definition\":\"" << N1UUIDHex(record.value->definition.owner.definition)
          << "\"},\"nextLocalID\":" << record.value->definition.issuance.nextLocalID
          << ",\"retiredLocalIDs\":[";
    for (std::size_t index = 0; index < record.value->definition.issuance.retiredLocalIDs.size(); ++index) {
        if (index) graph << ','; graph << record.value->definition.issuance.retiredLocalIDs[index];
    }
    graph << "],\"nodes\":[";
    for (std::size_t index = 0; index < record.value->definition.nodes.size(); ++index) {
        if (index) graph << ',';
        const auto& node = record.value->definition.nodes[index];
        graph << "{\"id\":\"" << N1UUIDHex(composite_recipe::NodeID(node))
              << "\",\"localID\":" << composite_recipe::LocalID(node);
        if (const auto* source = std::get_if<composite_recipe::SourceNode>(&node.value))
            graph << ",\"kind\":\"source\",\"slot\":" << source->shapeSlot
                  << ",\"schema\":" << source->recipe.schema
                  << ",\"sourceFeature\":\"" << N1UUIDHex(source->original.sourceFeature) << "\"";
        else graph << ",\"kind\":\"feature\",\"feature\":\""
                   << N1UUIDHex(std::get<composite_recipe::FeatureNode>(node.value).feature) << "\"";
        graph << '}';
    }
    const TopoDS_Shape& a = record.value->sourceShapes[0];
    const TopoDS_Shape& b = record.value->sourceShapes[1];
    graph << "],\"shapeRelations\":{\"sourcesPartner\":" << (a.IsPartner(b) ? "true" : "false")
          << ",\"sourcesSame\":" << (a.IsSame(b) ? "true" : "false")
          << ",\"sourcesEqual\":" << (a.IsEqual(b) ? "true" : "false")
          << ",\"source0CurrentPartner\":" << (a.IsPartner(record.current) ? "true" : "false")
          << ",\"source1CurrentPartner\":" << (b.IsPartner(record.current) ? "true" : "false")
          << "},\"orientations\":[" << int(a.Orientation()) << ',' << int(b.Orientation())
          << ',' << int(record.current.Orientation()) << "],\"locationsIdentity\":["
          << (a.Location().IsIdentity() ? "true" : "false") << ','
          << (b.Location().IsIdentity() ? "true" : "false") << ','
          << (record.current.Location().IsIdentity() ? "true" : "false")
          << "],\"analyticInputs\":[";
    for (std::size_t index = 0; index < analytic.inputs.size(); ++index) {
        if (index) graph << ',';
        const auto& input = analytic.inputs[index];
        graph << "{\"name\":" << N1JSONText(input.originalName)
              << ",\"visible\":" << (input.originallyVisible ? "true" : "false")
              << ",\"materialID\":\"" << N1UUIDHex(input.originalMaterial.identifier)
              << "\",\"materialKind\":" << int(input.originalMaterial.kind)
              << ",\"materialResource\":\"" << N1UUIDHex(input.originalMaterial.resource)
              << "\",\"materialResourceDigest\":\""
              << N1UUIDHex(input.originalMaterial.resourceDigest)
              << "\",\"groups\":[";
        for (std::size_t group = 0; group < input.originalGroups.size(); ++group) {
            if (group) graph << ','; graph << N1JSONText(input.originalGroups[group]);
        }
        graph << "],\"scalarBits\":[";
        bool firstScalar = true;
        const auto scalar = [&](double value) {
            if (!firstScalar) graph << ','; firstScalar = false;
            graph << '"' << N1CorpusHex64(N1ScalarBits(value)) << '"';
        };
        for (double value : input.dimensions) scalar(value);
        for (double value : input.translation) scalar(value);
        for (double value : input.rotationXYZW) scalar(value);
        scalar(input.metersPerUnit);
        for (double value : input.originalMaterial.baseColorSRGB) scalar(value);
        scalar(input.originalMaterial.metallic); scalar(input.originalMaterial.roughness);
        graph << "]}";
    }
    graph << "]}";
    if (suffix == "before") observation.beforeGraph = graph.str();
    else if (suffix == "reopened") observation.reopenedGraph = graph.str();
    else ok = false;
    if (!ok) collector->failed = true;
    return ok;
}

void N1OwnerFailure(const char* phase, const std::string& reason,
                    int detail = -1) noexcept {
    std::fprintf(stderr, "[N1-owner] phase=%s reason=%s detail=%d\n",
                 phase, reason.empty() ? "unspecified" : reason.c_str(), detail);
}

bool N1FeatureAnalytic(const composite_recipe::Definition& graph,
                       std::vector<std::uint8_t>& parameters) {
    for (const composite_recipe::Node& node : graph.nodes)
        if (const auto* feature = std::get_if<composite_recipe::FeatureNode>(&node.value)) {
            if (feature->kind != composite_recipe::PartBooleanFeatureKind) continue;
            parameters = feature->parameters;
            return true;
        }
    return false;
}

bool N1SameAnalyticMetadata(const AnalyticDefinition& before,
                            const AnalyticDefinition& reopened) noexcept {
    if (before.operation != reopened.operation
        || before.materialPolicy != reopened.materialPolicy
        || before.nativeAdmissionEnabled != reopened.nativeAdmissionEnabled) return false;
    for (std::size_t index = 0; index < 2; ++index) {
        const auto& expected = before.inputs[index];
        const auto& actual = reopened.inputs[index];
        if (expected.rootNode != actual.rootNode
            || expected.originalSourceFeature != actual.originalSourceFeature
            || expected.commitments.geometry != actual.commitments.geometry
            || expected.commitments.recipe != actual.commitments.recipe
            || expected.commitments.placement != actual.commitments.placement
            || expected.commitments.material != actual.commitments.material
            || expected.commitments.groups != actual.commitments.groups
            || expected.dimensions != actual.dimensions
            || expected.translation != actual.translation
            || expected.rotationXYZW != actual.rotationXYZW
            || expected.metersPerUnit != actual.metersPerUnit
            || expected.originalMaterial.identifier != actual.originalMaterial.identifier
            || expected.originalMaterial.baseColorSRGB != actual.originalMaterial.baseColorSRGB
            || expected.originalName != actual.originalName
            || expected.originalGroups != actual.originalGroups
            || expected.originallyVisible != actual.originallyVisible) return false;
    }
    return true;
}

bool N1AnalyticResultSolidControls() noexcept {
    try {
        const TopoDS_Shape direct = BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        const TopoDS_Shape tool = BRepPrimAPI_MakeBox(gp_Pnt(5, 2, 2), 8, 6, 6).Shape();
        if (!retained_part_boolean::IsOneValidForwardSolid(direct)
            || !retained_part_boolean::IsOneValidForwardSolid(tool)) return false;

        TopTools_ListOfShape arguments, tools;
        arguments.Append(direct); tools.Append(tool);
        BRepAlgoAPI_Cut boolean;
        boolean.SetArguments(arguments); boolean.SetTools(tools);
        boolean.SetRunParallel(Standard_False);
        boolean.SetNonDestructive(Standard_True);
        boolean.SetFuzzyValue(Precision::Confusion());
        boolean.SetUseOBB(Standard_True);
        boolean.SetCheckInverted(Standard_True);
        boolean.Build();
        const TopoDS_Shape observed = boolean.Shape();
        const TopoDS_Shape directResult = build::AnalyticResultSolid(direct);
        const TopoDS_Shape observedResult = build::AnalyticResultSolid(observed);

        BRep_Builder builder;
        TopoDS_Compound empty; builder.MakeCompound(empty);
        TopoDS_Compound twoSolid; builder.MakeCompound(twoSolid);
        builder.Add(twoSolid, direct); builder.Add(twoSolid, tool);
        TopExp_Explorer face(direct, TopAbs_FACE);
        if (!face.More()) return false;
        TopoDS_Compound mixed; builder.MakeCompound(mixed);
        builder.Add(mixed, direct); builder.Add(mixed, face.Current());
        const TopoDS_Shape reversed = direct.Reversed();

        return boolean.IsDone() && !boolean.HasErrors()
            && observed.ShapeType() == TopAbs_COMPOUND
            && observed.Orientation() == TopAbs_FORWARD
            && retained_part_boolean::SolidCount(observed) == 1
            && retained_part_boolean::IsOneValidForwardSolid(directResult)
            && directResult.IsSame(direct)
            && retained_part_boolean::IsOneValidForwardSolid(observedResult)
            && build::AnalyticResultSolid(empty).IsNull()
            && build::AnalyticResultSolid(twoSolid).IsNull()
            && build::AnalyticResultSolid(mixed).IsNull()
            && build::AnalyticResultSolid(reversed).IsNull();
    } catch (...) { return false; }
}
} // namespace

bool PartBooleanOwner::installEvidenceFixture(
    Operation operation, double metersPerUnit, TDF_Label& carrier) noexcept {
    carrier.Nullify();
    try {
        if (!boundTo(owner_.Document()) || document_->HasOpenCommand()
            || document_->GetAvailableUndos() || document_->GetAvailableRedos()) return false;
        const auto identifier = [](std::uint8_t seed) {
            retained_recipe::UUID value{};
            for (std::size_t index = 0; index < value.size(); ++index)
                value[index] = std::uint8_t(seed + index);
            return value;
        };
        AnalyticDefinition analytic;
        analytic.operation = operation;
        analytic.inputs[0].rootNode = identifier(40);
        analytic.inputs[1].rootNode = identifier(60);
        analytic.inputs[0].originalSourceFeature = identifier(80);
        analytic.inputs[1].originalSourceFeature = identifier(100);
        analytic.inputs[0].dimensions = {40, 30, 20};
        analytic.inputs[1].dimensions = {20, 10, 10};
        analytic.inputs[1].translation = {30, 10, 5};
        for (std::size_t index = 0; index < 2; ++index) {
            analytic.inputs[index].metersPerUnit = metersPerUnit;
            analytic.inputs[index].originalMaterial.identifier = identifier(std::uint8_t(120 + index));
            analytic.inputs[index].originalName = index ? "Right analytic input" : "Left analytic input";
            analytic.inputs[index].originalGroups = {index ? "Boolean tools" : "Boolean bases"};
        }
        retained_part_boolean::OperandReadSet provisional;
        provisional.leftSource.locator.node = analytic.inputs[0].rootNode;
        provisional.rightSource.locator.node = analytic.inputs[1].rootNode;
        auto fillRead = [&](retained_recipe::DependencyRead& read, std::uint8_t seed) {
            read.locator.owner = {identifier(1), identifier(2), identifier(3)};
            read.locator.sourceFeature = identifier(seed);
            read.geometry.fill(std::uint8_t(seed + 1)); read.recipe.fill(std::uint8_t(seed + 2));
            read.placement.fill(std::uint8_t(seed + 3)); read.material.fill(std::uint8_t(seed + 4));
            read.groups.fill(std::uint8_t(seed + 5));
        };
        fillRead(provisional.leftSource, 80); fillRead(provisional.rightSource, 100);
        analytic.inputs[0].commitments = {provisional.leftSource.geometry,
            provisional.leftSource.recipe, provisional.leftSource.placement,
            provisional.leftSource.material, provisional.leftSource.groups};
        analytic.inputs[1].commitments = {provisional.rightSource.geometry,
            provisional.rightSource.recipe, provisional.rightSource.placement,
            provisional.rightSource.material, provisional.rightSource.groups};
        const auto firstBuild = build::BuildAnalytic(
            analytic, metersPerUnit, provisional, provisional);
        if (!firstBuild.complete) return false;
        Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(document_->Main());
        carrier = shapes->AddShape(firstBuild.candidate.solid, Standard_False, Standard_False);
        if (carrier.IsNull() || !owner_.MigrateLegacyIdentifiers()) return false;
        retained_recipe::OwnerKey ownerKey;
        if (!retained_solid::ReadUUID(document_->Main(), DocumentIdentifierAttributeID(), ownerKey.document)
            || !retained_solid::ReadUUID(carrier, EntityIdentifierAttributeID(), ownerKey.entity)
            || !retained_solid::ReadUUID(carrier, DefinitionIdentifierAttributeID(), ownerKey.definition)) return false;
        provisional.leftSource.locator.owner = ownerKey;
        provisional.rightSource.locator.owner = ownerKey;
        composite_recipe::Definition graph;
        graph.schemaVersion = 1; graph.owner = ownerKey;
        graph.outputNode = identifier(110); graph.issuance.nextLocalID = 4;
        for (std::size_t index = 0; index < 2; ++index) {
            composite_recipe::SourceNode source;
            source.node = analytic.inputs[index].rootNode; source.localID = index + 1;
            source.original = {ownerKey.document, identifier(std::uint8_t(10 + index)),
                identifier(std::uint8_t(20 + index)), analytic.inputs[index].originalSourceFeature};
            profile::Parameters recipe;
            recipe.metersPerUnit = metersPerUnit; recipe.definition.plane = 0;
            recipe.definition.depth = analytic.inputs[index].dimensions[2];
            recipe.definition.points = {gp_Pnt2d(0, 0),
                gp_Pnt2d(analytic.inputs[index].dimensions[0], 0),
                gp_Pnt2d(analytic.inputs[index].dimensions[0], analytic.inputs[index].dimensions[1]),
                gp_Pnt2d(0, analytic.inputs[index].dimensions[1])};
            std::vector<double> values;
            if (!profile::Encode(recipe, values)
                || !composite_recipe::EncodeScalarRecipe(composite_recipe::RecipeKind::Profile,
                    std::uint32_t(profile::SchemaFor(recipe)), values, source.recipe.bytes)) return false;
            source.recipe.kind = composite_recipe::RecipeKind::Profile;
            source.recipe.schema = std::uint32_t(profile::SchemaFor(recipe));
            source.inputToCarrier.sourceMetersPerUnit = metersPerUnit;
            source.inputToCarrier.carrierMetersPerUnit = metersPerUnit;
            source.inputToCarrier.matrix[3] = analytic.inputs[index].translation[0];
            source.inputToCarrier.matrix[7] = analytic.inputs[index].translation[1];
            source.inputToCarrier.matrix[11] = analytic.inputs[index].translation[2];
            std::vector<std::uint8_t> shapeBytes;
            if (!ShapeBytes(firstBuild.sources[index], shapeBytes)
                || !HashBytes(shapeBytes, source.commitments.geometry)
                || !composite_recipe::Hash(source.recipe.bytes, source.commitments.recipe)) return false;
            source.commitments.placement.fill(std::uint8_t(31 + index));
            source.commitments.material.fill(std::uint8_t(33 + index));
            source.commitments.groups.fill(std::uint8_t(35 + index));
            analytic.inputs[index].commitments = {source.commitments.geometry,
                source.commitments.recipe, source.commitments.placement,
                source.commitments.material, source.commitments.groups};
            source.shapeSlot = std::uint32_t(index);
            graph.nodes.push_back({source});
        }
        composite_recipe::FeatureNode feature;
        feature.node = graph.outputNode; feature.feature = identifier(111); feature.localID = 3;
        feature.kind = composite_recipe::PartBooleanFeatureKind;
        feature.codecVersion = composite_recipe::PartBooleanFeatureCodec;
        feature.inputs = {analytic.inputs[0].rootNode, analytic.inputs[1].rootNode};
        if (!EncodeAnalytic(analytic, feature.parameters)) return false;
        graph.nodes.push_back({feature});
        std::vector<std::uint8_t> exact;
        if (!composite_recipe::Encode(graph, exact)) return false;
        const TDF_Label record = carrier.FindChild(composite_recipe::MinimumRecordTag, Standard_True);
        Handle(composite_recipe::Attribute) attribute = new composite_recipe::Attribute();
        record.AddAttribute(attribute);
        auto payload = std::make_shared<composite_recipe::Payload>();
        payload->definition = graph; payload->bytes = exact;
        payload->sourceShapes.assign(firstBuild.sources.begin(), firstBuild.sources.end());
        if (!owner_.StagePartBooleanPayload(record, payload)) return false;
        TNaming_Builder(record).Select(firstBuild.candidate.solid, firstBuild.candidate.solid);
        document_->ClearUndos(); document_->SetUndoLimit(OcctDocument::kNativeSessionUndoLimit);
        composite_recipe::Record readback;
        return composite_recipe::Read(document_, carrier, readback)
            && readback.value && readback.value->bytes == exact;
    } catch (...) { carrier.Nullify(); return false; }
}

bool PartBooleanOwner::installShellEvidenceFixture(
    double metersPerUnit, TDF_Label& carrier) noexcept {
    carrier.Nullify();
    try {
        if (!boundTo(owner_.Document()) || document_->HasOpenCommand()
            || document_->GetAvailableUndos() || document_->GetAvailableRedos()
            || !std::isfinite(metersPerUnit) || metersPerUnit <= 0) return false;
        Definition feature;
        composite_recipe::Definition graph = composite_recipe::Probe::ShellDefinition(&feature);
        const double millimetre = 0.001 / metersPerUnit;
        auto* left = std::get_if<composite_recipe::SourceNode>(&graph.nodes[0].value);
        auto* right = std::get_if<composite_recipe::SourceNode>(&graph.nodes[1].value);
        auto* boolean = std::get_if<composite_recipe::FeatureNode>(&graph.nodes[2].value);
        if (!left || !right || !boolean) return false;
        std::vector<double> toolValues;
        profile::Parameters tool;
        if (!composite_recipe::DecodeScalarRecipe(right->recipe, toolValues)
            || !profile::Decode(toolValues, tool)) return false;
        tool.metersPerUnit = metersPerUnit;
        tool.definition.points = {{0,0},{10*millimetre,0},{10*millimetre,10*millimetre},{0,10*millimetre}};
        tool.definition.depth = 10*millimetre;
        if (!profile::Encode(tool, toolValues)
            || !composite_recipe::EncodeScalarRecipe(right->recipe.kind,
                std::uint32_t(profile::SchemaFor(tool)), toolValues, right->recipe.bytes)
            || !composite_recipe::Hash(right->recipe.bytes, right->commitments.recipe)) return false;
        right->recipe.schema = std::uint32_t(profile::SchemaFor(tool));
        right->inputToCarrier.sourceMetersPerUnit = metersPerUnit;
        right->inputToCarrier.carrierMetersPerUnit = metersPerUnit;
        right->inputToCarrier.matrix[3] = 35*millimetre;
        right->inputToCarrier.matrix[7] = 10*millimetre;
        right->inputToCarrier.matrix[11] = 1*millimetre;
        std::vector<double> shellValues;
        profile::Parameters shell;
        if (!composite_recipe::DecodeScalarRecipe(left->recipe, shellValues)
            || !profile::Decode(shellValues, shell)) return false;
        shell.metersPerUnit = metersPerUnit;
        for (auto& point : shell.definition.points)
            point.SetCoord(point.X()*millimetre, point.Y()*millimetre);
        shell.definition.depth *= millimetre;
        shell.shells.front().thickness *= millimetre;
        shell.shells.front().metersPerLocalUnit = metersPerUnit;
        if (!profile::Encode(shell, shellValues)
            || !composite_recipe::EncodeScalarRecipe(left->recipe.kind, 5,
                shellValues, left->recipe.bytes)
            || !composite_recipe::Hash(left->recipe.bytes, left->commitments.recipe)) return false;
        left->inputToCarrier.sourceMetersPerUnit = metersPerUnit;
        left->inputToCarrier.carrierMetersPerUnit = metersPerUnit;
        for (auto* source:{left,right}) {
            std::vector<std::uint8_t> placement;
            for (double scalar:source->inputToCarrier.matrix) Append(placement,scalar);
            Append(placement,source->inputToCarrier.sourceMetersPerUnit);
            Append(placement,source->inputToCarrier.carrierMetersPerUnit);
            if (!HashBytes(placement,source->commitments.placement)) return false;
        }
        feature.operation = Operation::Union;
        feature.inputs[0].commitments = {left->commitments.geometry,left->commitments.recipe,
            left->commitments.placement,left->commitments.material,left->commitments.groups};
        feature.inputs[1].commitments = {right->commitments.geometry,right->commitments.recipe,
            right->commitments.placement,right->commitments.material,right->commitments.groups};
        if (!Encode(feature, boolean->parameters)) return false;
        build::ShellCompositeRequest request;
        if (!build::DecodeShellComposite(graph, request)) return false;
        retained_part_boolean::OperandReadSet reads;
        const auto fillRead=[&](retained_recipe::DependencyRead& read,
                                const composite_recipe::SourceNode& source) {
            read.locator={graph.owner,source.node,source.original.sourceFeature};
            read.geometry=source.commitments.geometry;read.recipe=source.commitments.recipe;
            read.placement=source.commitments.placement;read.material=source.commitments.material;
            read.groups=source.commitments.groups;
        };
        fillRead(reads.leftSource,*left);fillRead(reads.rightSource,*right);
        const auto built = build::BuildShellComposite(request, reads, reads);
        if (!built.complete) return false;
        Handle(XCAFDoc_ShapeTool) shapes=XCAFDoc_DocumentTool::ShapeTool(document_->Main());
        carrier=shapes->AddShape(built.candidate.solid,Standard_False,Standard_False);
        if (carrier.IsNull() || !owner_.MigrateLegacyIdentifiers()) return false;
        retained_recipe::OwnerKey ownerKey;
        if (!retained_solid::ReadUUID(document_->Main(),DocumentIdentifierAttributeID(),ownerKey.document)
            || !retained_solid::ReadUUID(carrier,EntityIdentifierAttributeID(),ownerKey.entity)
            || !retained_solid::ReadUUID(carrier,DefinitionIdentifierAttributeID(),ownerKey.definition)) return false;
        graph.owner=ownerKey;
        for (auto* source:{left,right}) source->original.document=ownerKey.document;
        for (std::size_t index=0;index<2;++index) {
            auto* source=index?right:left;std::vector<std::uint8_t> bytes;
            if (!ShapeBytes(built.sources[index],bytes)
                || !HashBytes(bytes,source->commitments.geometry)) return false;
            feature.inputs[index].commitments.geometry=source->commitments.geometry;
            feature.inputs[index].commitments.recipe=source->commitments.recipe;
        }
        if (!Encode(feature,boolean->parameters)) return false;
        std::vector<std::uint8_t> exact;
        if (!composite_recipe::Encode(graph,exact)) return false;
        const TDF_Label record=carrier.FindChild(composite_recipe::MinimumRecordTag,Standard_True);
        Handle(composite_recipe::Attribute) attribute=new composite_recipe::Attribute();
        record.AddAttribute(attribute);
        auto payload=std::make_shared<composite_recipe::Payload>();
        payload->definition=graph;payload->bytes=exact;
        payload->sourceShapes.assign(built.sources.begin(),built.sources.end());
        if (!owner_.StagePartBooleanPayload(record,payload)) return false;
        TNaming_Builder(record).Select(built.candidate.solid,built.candidate.solid);
        document_->ClearUndos();document_->SetUndoLimit(OcctDocument::kNativeSessionUndoLimit);
        composite_recipe::Record readback;
        return composite_recipe::Read(document_,carrier,readback)
            && readback.value&&readback.value->bytes==exact;
    } catch (...) { carrier.Nullify(); return false; }
}

ColdLifecycleEvidence PartBooleanOwner::debugColdLifecycle(
    const TDF_Label& originalCarrier) noexcept {
    ColdLifecycleEvidence evidence;
    std::vector<Handle(OcctDocument)> opened;
    try {
        if (!boundTo(owner_.Document()) || blocksOtherWork() || originalCarrier.IsNull()) return evidence;
        const TDocStd_FormatVersion requestedVersion = document_->StorageFormatVersion();
        double requestedUnit = 0;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document_, requestedUnit)) return evidence;
        composite_recipe::Record beforeRecord;
        std::vector<std::uint8_t> beforeAnalytic;
        AnalyticDefinition beforeDefinition;
        build::ShellCompositeRequest beforeShell;
        if (!composite_recipe::Read(document_, originalCarrier, beforeRecord)
            || !beforeRecord.value) {
            N1OwnerFailure("fixture-record", "pre-save-payload-unreadable");
            return evidence;
        }
        const bool beforeIsShell = build::DecodeShellComposite(
            beforeRecord.value->definition, beforeShell);
        const bool beforeIsAnalytic = !beforeIsShell
            && N1FeatureAnalytic(beforeRecord.value->definition, beforeAnalytic)
            && DecodeAnalytic(beforeAnalytic, beforeDefinition);
        if (!beforeIsShell && !beforeIsAnalytic) {
            N1OwnerFailure("fixture-record", "unsupported-payload");
            return evidence;
        }
        if (!N1CaptureRecord(beforeRecord, "before")) {
            N1OwnerFailure("corpus-before", "capture-failed");
            return evidence;
        }
        const auto stableAfterTwoMaterializations = [](const TopoDS_Shape& shape) {
            std::string originalBytes, firstBytes, secondBytes;
            const TopoDS_Shape first = build::PersistenceMaterializeDetached(shape);
            const TopoDS_Shape second = build::PersistenceMaterializeDetached(first);
            return !first.IsNull() && !second.IsNull()
                && retained_part_boolean::ExactShapeBytes(shape, originalBytes)
                && retained_part_boolean::ExactShapeBytes(first, firstBytes)
                && retained_part_boolean::ExactShapeBytes(second, secondBytes)
                && originalBytes == firstBytes && firstBytes == secondBytes;
        };
        const bool materializationStable = beforeRecord.value->sourceShapes.size() == 2
            && stableAfterTwoMaterializations(beforeRecord.value->sourceShapes[0])
            && stableAfterTwoMaterializations(beforeRecord.value->sourceShapes[1])
            && stableAfterTwoMaterializations(beforeRecord.current);
        InternalBooleanOperationSession beforeSession(*this);
        const auto beforeCapture = beforeSession.capture(originalCarrier);
        const bool capturedBeforeSave = beforeCapture.handle
            && beforeCapture.receipt.reason == "captured"
            && beforeCapture.handle->sourceShapes.size() == 2;
        const Receipt beforeCancelled = beforeSession.cancel();
        if (!capturedBeforeSave || beforeCancelled.outcome != Outcome::Cancelled) {
            N1OwnerFailure("fixture-capture", beforeCapture.receipt.reason,
                           int(beforeCancelled.outcome));
            return evidence;
        }
        const auto uniqueBase = [](const char* suffix) {
            NSString* name = [NSString stringWithFormat:@"n1-boolean-%@-%s.n1-fixture",
                [[NSUUID UUID] UUIDString], suffix];
            return std::string([[NSTemporaryDirectory() stringByAppendingPathComponent:name] UTF8String]);
        };
        const auto headerMatches = [&](const std::string& saved,
                                       TDocStd_FormatVersion expected,
                                       const char* phase, bool report) {
            std::ifstream stream(saved, std::ios::in | std::ios::binary);
            Handle(Storage_Data) stored;
            const TCollection_ExtendedString format = stream
                ? PCDM_ReadWriter::FileFormat(stream, stored)
                : TCollection_ExtendedString();
            const bool valid = stream.is_open()
                && format.IsEqual(TCollection_ExtendedString("BinXCAF"))
                && !stored.IsNull() && !stored->HeaderData().IsNull()
                && stored->HeaderData()->StorageVersion().IsIntegerValue()
                && stored->HeaderData()->StorageVersion().IntegerValue() == int(expected);
            if (!valid && report) N1OwnerFailure(phase, "saved-header-version-mismatch");
            return valid;
        };
        const auto saveCycle = [&](OcctDocument* source, const char* phase,
                                   std::string& saved) {
            if (source == nullptr || source->Document().IsNull()) {
                N1OwnerFailure(phase, "missing-document");
                return false;
            }
            source->Document()->ChangeStorageFormatVersion(requestedVersion);
            const std::string base = uniqueBase(phase);
            TDocStd_PathParser parser(
                TCollection_ExtendedString(base.c_str(), Standard_True));
            parser.Parse();
            if (parser.Trek().Length() == 0) {
                N1OwnerFailure(phase, "empty-parsed-parent");
                return false;
            }
            saved = source->save(base);
            if (saved.empty()) {
                N1OwnerFailure(phase, "save-returned-empty");
                return false;
            }
            NSString* path = [NSString stringWithUTF8String:saved.c_str()];
            if (path == nil || ![NSFileManager.defaultManager fileExistsAtPath:path]) {
                N1OwnerFailure(phase, "saved-file-missing");
                return false;
            }
            if (!headerMatches(saved, requestedVersion, phase, true)) return false;
            if (N1LegacyCorpusCollector) {
                const std::string key = N1LegacyCorpusCollector->activeCase;
                const std::string name = std::string(phase) == "first-save"
                    ? "legacy-" + key + ".binXCAF"
                    : std::string(phase) == "second-save"
                        ? "control-" + key + "-after-edit.binXCAF"
                        : std::string(phase) == "third-save"
                            ? "control-" + key + "-noop-resave.binXCAF" : std::string();
                std::vector<std::uint8_t> bytes = N1ReadWholeFile(saved);
                if (name.empty() || !N1LegacyCorpusCollector->add(name, std::move(bytes))) {
                    N1OwnerFailure(phase, "corpus-file-capture-failed");
                    return false;
                }
                auto& observed = N1LegacyCorpusCollector->cases[key];
                if (std::string(phase) == "first-save") observed.primary = name;
                else if (std::string(phase) == "second-save") observed.afterEdit = name;
                else observed.noopResave = name;
                std::vector<composite_recipe::Record> records;
                composite_recipe::Digest recipeDigest{};
                if (!composite_recipe::ReadAll(source->Document(), records)
                    || records.size() != 1 || !records.front().value
                    || !composite_recipe::Hash(records.front().value->bytes, recipeDigest)) {
                    N1LegacyCorpusCollector->failed = true;
                    N1OwnerFailure(phase, "corpus-record-observation-failed");
                    return false;
                }
                observed.savedRecipeSHA256[phase] = N1UUIDHex(recipeDigest);
                observed.savedSourceSlotCount[phase] = records.front().value->sourceShapes.size();
                if (observed.savedSourceSlotCount[phase] != 2) {
                    N1LegacyCorpusCollector->failed = true;
                    N1OwnerFailure(phase, "corpus-source-slot-count");
                    return false;
                }
            }
            return true;
        };
        bool versionMismatchRejected = true;
        if (requestedVersion == TDocStd_FormatVersion_VERSION_11) {
            document_->ChangeStorageFormatVersion(TDocStd_FormatVersion_CURRENT);
            const std::string base = uniqueBase("version-mismatch-control");
            const std::string mismatched = owner_.save(base);
            versionMismatchRejected = !mismatched.empty()
                && headerMatches(mismatched, TDocStd_FormatVersion_CURRENT,
                                 "version-mismatch-control", false)
                && !headerMatches(mismatched, requestedVersion,
                                  "version-mismatch-control", false);
            document_->ChangeStorageFormatVersion(requestedVersion);
            if (!versionMismatchRejected)
                N1OwnerFailure("version-mismatch-control", "mismatch-accepted");
        }
        const auto mutationControl = [&]() {
            NativeDocumentSession native;
            const Handle(OcctDocument) wrapper = native.Document();
            if (wrapper.IsNull() || wrapper->Document().IsNull()) return false;
            double unit = 0;
            if (!XCAFDoc_DocumentTool::GetLengthUnit(document_, unit)) return false;
            XCAFDoc_DocumentTool::SetLengthUnit(wrapper->Document(), unit);
            PartBooleanOwner* owner = wrapper->PartBooleanOwnerService();
            TDF_Label carrier;
            if (!owner || !owner->installEvidenceFixture(
                    Operation::Subtract, unit, carrier)) return false;
            composite_recipe::Record original;
            if (!composite_recipe::Read(wrapper->Document(), carrier, original)
                || !original.value) return false;
            gp_Trsf shift;
            shift.SetTranslation(gp_Vec(0.125, 0, 0));
            const TopoDS_Shape changed = BRepBuilderAPI_Transform(
                original.current, shift, Standard_True).Shape();
            Handle(XCAFDoc_ShapeTool) shapes =
                XCAFDoc_DocumentTool::ShapeTool(wrapper->Document()->Main());
            if (changed.IsNull() || shapes.IsNull()) return false;
            shapes->SetShape(original.owner, changed);
            TNaming_Builder(original.label).Select(changed, changed);
            composite_recipe::Record readable;
            if (!composite_recipe::Read(wrapper->Document(), carrier, readable)
                || !readable.value
                || readable.value->bytes != original.value->bytes) return false;
            InternalBooleanOperationSession session(*owner);
            const auto refused = session.capture(carrier);
            if (refused.handle) session.cancel();
            return !refused.handle
                && refused.receipt.reason == "capture-incomplete-or-noncurrent";
        };
        const bool changedCurrentRefused = mutationControl();
        const auto coldOpen = [&](const std::string& saved,
                                  Handle(OcctDocument)& wrapper,
                                  TDF_Label& carrier, const char* phase) {
            wrapper = new OcctDocument();
            Core3DDefineSafeBinXCAFFormat(wrapper->myApp);
            Handle(TDocStd_Document) candidate;
            Core3DBeginSafeBinaryRead();
            const PCDM_ReaderStatus status = wrapper->myApp->Open(
                TCollection_ExtendedString(saved.c_str(), Standard_True), candidate);
            if (Core3DSafeBinaryReadWasRejected() || status != PCDM_RS_OK || candidate.IsNull()) {
                N1OwnerFailure(phase, "open-rejected", int(status));
                return false;
            }
            wrapper->myOcafDoc = candidate;
            wrapper->ObserveSuccessfulNativeDocumentAdoption();
            opened.push_back(wrapper);
            candidate->ChangeStorageFormatVersion(requestedVersion);
            candidate->SetUndoLimit(OcctDocument::kNativeSessionUndoLimit);
            double reopenedUnit = 0;
            if (!XCAFDoc_DocumentTool::GetLengthUnit(candidate, reopenedUnit)
                || N1ScalarBits(reopenedUnit) != N1ScalarBits(requestedUnit)) {
                N1OwnerFailure(phase, "reopened-unit-mismatch");
                return false;
            }
            std::vector<composite_recipe::Record> records;
            if (!composite_recipe::ReadAll(candidate, records) || records.size() != 1) {
                N1OwnerFailure(phase, "record-readback-failed", int(records.size()));
                return false;
            }
            carrier = records.front().owner;
            PartBooleanOwner* reopenedOwner = wrapper->PartBooleanOwnerService();
            if (reopenedOwner == nullptr) {
                N1OwnerFailure(phase, "owner-service-missing");
                return false;
            }
            InternalBooleanOperationSession session(*reopenedOwner);
            const auto captured = session.capture(carrier);
            const bool complete = captured.handle
                && captured.receipt.reason == "captured"
                && captured.handle->sourceShapes.size() == 2;
            const Receipt cancelled = session.cancel();
            if (!complete || cancelled.outcome != Outcome::Cancelled)
                N1OwnerFailure(phase, "post-open-capture-failed");
            return complete && cancelled.outcome == Outcome::Cancelled;
        };
        const auto closeAll = [&]() {
            for (const Handle(OcctDocument)& wrapper : opened)
                if (!wrapper.IsNull()) wrapper->CloseNativeSession();
        };
        std::string firstSaved;
        Handle(OcctDocument) first; TDF_Label firstCarrier;
        evidence.firstColdOpen = saveCycle(&owner_, "first-save", firstSaved)
            && coldOpen(firstSaved, first, firstCarrier, "first-open");
        evidence.firstColdOpen = evidence.firstColdOpen
            && materializationStable && versionMismatchRejected
            && changedCurrentRefused;
        if (!evidence.firstColdOpen) { closeAll(); return evidence; }
        PartBooleanOwner* firstOwner = first->PartBooleanOwnerService();
        InternalBooleanOperationSession later(*firstOwner);
        const auto captured = later.capture(firstCarrier);
        evidence.discardedOldHandles = captured.handle
            && first->Document().get() != documentIdentity_;
        if (!captured.handle || captured.receipt.reason != "captured") {
            N1OwnerFailure("cold-capture", captured.receipt.reason);
            closeAll(); return evidence;
        }
        composite_recipe::Record reopenedRecord;
        std::vector<std::uint8_t> reopenedAnalytic;
        AnalyticDefinition reopenedDefinition;
        build::ShellCompositeRequest reopenedShell;
        const bool reopenedInvalid = !composite_recipe::Read(
            first->Document(), firstCarrier, reopenedRecord)
            || !reopenedRecord.value
            || reopenedRecord.value->bytes != beforeRecord.value->bytes;
        const bool metadataMatches = !reopenedInvalid
            && (beforeIsShell
                ? build::DecodeShellComposite(
                    reopenedRecord.value->definition, reopenedShell)
                : N1FeatureAnalytic(reopenedRecord.value->definition, reopenedAnalytic)
                    && reopenedAnalytic == beforeAnalytic
                    && DecodeAnalytic(reopenedAnalytic, reopenedDefinition)
                    && N1SameAnalyticMetadata(beforeDefinition, reopenedDefinition));
        if (!metadataMatches) {
            N1OwnerFailure("cold-metadata", "reopened-payload-differs");
            closeAll(); return evidence;
        }
        if (!N1CaptureRecord(reopenedRecord, "reopened")) {
            N1OwnerFailure("corpus-reopened", "capture-failed");
            closeAll(); return evidence;
        }
        const auto reads = Reads(captured.handle->debugSnapshot());
        double unit = 0;
        const bool hasUnit = XCAFDoc_DocumentTool::GetLengthUnit(first->Document(), unit);
        evidence.independentReplay = beforeIsShell
            ? rebuild::CheckShellComposite(
                reopenedRecord.value->definition, reads).fixedPoint()
            : hasUnit && rebuild::CheckAnalytic(
                captured.handle->debugAnalytic(), unit, reads).fixedPoint();
        evidence.completeInputs = captured.handle->sourceShapes.size() == 2;
        PrepareOutcome prepared;
        if (beforeIsShell) {
            ShellEditDefinition edit = captured.handle->debugShell();
            edit.placements[1].matrix[3] += 0.001 / edit.profiles[1].metersPerUnit;
            prepared = later.prepare(edit);
        } else {
            AnalyticDefinition edit = captured.handle->debugAnalytic();
            edit.inputs[1].translation[0] += 1;
            prepared = later.prepare(edit);
        }
        const Receipt applied = prepared.handle ? later.apply() : Receipt();
        if (!prepared.handle || prepared.receipt.reason != "prepared")
            N1OwnerFailure("cold-prepare", prepared.receipt.reason);
        if (applied.outcome != Outcome::Committed)
            N1OwnerFailure("cold-apply", applied.reason, int(applied.outcome));
        evidence.laterEdit = applied.outcome == Outcome::Committed
            && applied.measuredHistoryDelta == 1;
        if (!evidence.laterEdit) { closeAll(); return evidence; }
        DocumentSnapshot committed, undone, redone;
        evidence.faceMaterialReadback = firstOwner->captureNativeState(
            firstCarrier, committed)
            && (!beforeIsShell || (!committed.faceMaterials.empty()
                && !committed.faceMaterialLabels.empty()));
        const bool didUndo = first->undo();
        const bool capturedUndo = didUndo
            && firstOwner->captureNativeState(firstCarrier, undone);
        const bool didRedo = capturedUndo && first->redo();
        const bool capturedRedo = didRedo
            && firstOwner->captureNativeState(firstCarrier, redone);
        evidence.undoRedo = didUndo && didRedo;
        evidence.undoRedoFaceMaterials = capturedUndo && capturedRedo
            && undone.semanticEquals(captured.handle->debugBaseline())
            && redone.semanticEquals(committed);
        if (!evidence.undoRedo) { closeAll(); return evidence; }
        std::string secondSaved;
        Handle(OcctDocument) second; TDF_Label secondCarrier;
        evidence.secondColdOpen = saveCycle(first.get(), "second-save", secondSaved)
            && coldOpen(secondSaved, second, secondCarrier, "second-open");
        if (!evidence.secondColdOpen) { closeAll(); return evidence; }
        DocumentSnapshot secondSnapshot;
        PartBooleanOwner* secondOwner = second->PartBooleanOwnerService();
        const bool secondMaterials = secondOwner
            && secondOwner->captureNativeState(secondCarrier, secondSnapshot)
            && secondSnapshot.semanticEquals(committed);
        std::string thirdSaved;
        Handle(OcctDocument) third; TDF_Label thirdCarrier;
        evidence.thirdColdOpen = saveCycle(second.get(), "third-save", thirdSaved)
            && coldOpen(thirdSaved, third, thirdCarrier, "third-open");
        DocumentSnapshot thirdSnapshot;
        PartBooleanOwner* thirdOwner = third.IsNull()
            ? nullptr : third->PartBooleanOwnerService();
        evidence.coldFaceMaterials = secondMaterials && evidence.thirdColdOpen
            && thirdOwner && thirdOwner->captureNativeState(thirdCarrier, thirdSnapshot)
            && thirdSnapshot.semanticEquals(secondSnapshot);
        if (N1LegacyCorpusCollector && evidence.thirdColdOpen) {
            auto& observed = N1LegacyCorpusCollector->cases[N1LegacyCorpusCollector->activeCase];
            const auto secondBytes = N1LegacyCorpusCollector->blobs.find(observed.afterEdit);
            const auto thirdBytes = N1LegacyCorpusCollector->blobs.find(observed.noopResave);
            if (secondBytes == N1LegacyCorpusCollector->blobs.end()
                || thirdBytes == N1LegacyCorpusCollector->blobs.end()) {
                N1LegacyCorpusCollector->failed = true;
            } else {
                observed.afterEditLength = secondBytes->second.size();
                observed.noopLength = thirdBytes->second.size();
                const std::size_t common = std::min(observed.afterEditLength, observed.noopLength);
                std::size_t difference = 0;
                while (difference < common
                    && secondBytes->second[difference] == thirdBytes->second[difference]) ++difference;
                observed.wholeFilesEqual = difference == observed.afterEditLength
                    && difference == observed.noopLength;
                observed.firstDifference = observed.wholeFilesEqual
                    ? std::numeric_limits<std::size_t>::max() : difference;
                const auto secondRecipe = observed.savedRecipeSHA256.find("second-save");
                const auto thirdRecipe = observed.savedRecipeSHA256.find("third-save");
                if (secondRecipe == observed.savedRecipeSHA256.end()
                    || thirdRecipe == observed.savedRecipeSHA256.end()
                    || secondRecipe->second != thirdRecipe->second)
                    N1LegacyCorpusCollector->failed = true;
            }
        }
        closeAll();
        return evidence;
    } catch (...) {
        for (const Handle(OcctDocument)& wrapper : opened)
            if (!wrapper.IsNull()) wrapper->CloseNativeSession();
        return evidence;
    }
}
#endif
} // namespace core3d::part_boolean::owner

#if DEBUG
#include <BinObjMgt_Position.hxx>
#include <BinObjMgt_RRelocationTable.hxx>
#include <BinObjMgt_SRelocationTable.hxx>
#include <Message_Messenger.hxx>
#include <Storage_HeaderData.hxx>
#include <sstream>
namespace core3d::part_boolean::owner {
namespace {
struct EvidenceRun {
    bool document = false, installed = false, captured = false, prepared = false;
    bool committed = false, exactDelta = false, soleRecord = false, consumed = false;
    bool stableIdentities = false, undo = false, redo = false, sameRedoIdentities = false;
    bool ownerBlocked = false, recovered = false, noPublishedDelta = false;
    bool restored = false, historyRestored = false, graphRestored = false;
    bool fixedPoint = false, correspondence = false;
    bool reachedFaultBoundary = false;
};

EvidenceRun Exercise(Operation operation, FaultPoint fault = FaultPoint::None,
                     double unit = 0.001) {
    EvidenceRun evidence;
    NativeDocumentSession native;
    const Handle(OcctDocument) wrapper = native.Document();
    evidence.document = !wrapper.IsNull() && !wrapper->Document().IsNull();
    if (!evidence.document) return evidence;
    XCAFDoc_DocumentTool::SetLengthUnit(wrapper->Document(), unit);
    PartBooleanOwner* owner = wrapper->PartBooleanOwnerService();
    TDF_Label carrier;
    evidence.installed = owner && owner->installEvidenceFixture(operation, unit, carrier);
    if (!evidence.installed) {
        N1OwnerFailure("fixture-install", "install-failed", int(operation));
        return evidence;
    }
    const std::string entity = wrapper->EntityIdentifierForLabel(carrier);
    const std::string definition = wrapper->DefinitionIdentifierForLabel(carrier);
    composite_recipe::Record beforeRecord;
    std::vector<std::uint8_t> beforeGraph;
    evidence.graphRestored = composite_recipe::Read(wrapper->Document(), carrier, beforeRecord)
        && beforeRecord.value && composite_recipe::Encode(beforeRecord.value->definition, beforeGraph);
    InternalBooleanOperationSession session(*owner);
    const auto captured = session.capture(carrier);
    evidence.captured = captured.handle && captured.receipt.reason == "captured";
    if (!evidence.captured) {
        N1OwnerFailure("capture", captured.receipt.reason, int(captured.receipt.outcome));
        return evidence;
    }
    const auto beforeHistory = captured.handle->debugBaseline().history;
    AnalyticDefinition request = captured.handle->debugAnalytic();
    const auto reads = Reads(captured.handle->debugSnapshot());
    double carrierUnit = 0;
    const bool readUnit = XCAFDoc_DocumentTool::GetLengthUnit(wrapper->Document(), carrierUnit);
    const auto built = readUnit ? build::BuildAnalytic(request, carrierUnit, reads, reads)
                                : build::AnalyticBuild();
    const auto proof = readUnit ? correspondence::ProveAnalytic(request, built, carrierUnit)
                                : correspondence::AnalyticProof();
    const auto fixed = readUnit ? rebuild::CheckAnalytic(request, carrierUnit, reads)
                                : rebuild::AnalyticFixedPointEvidence();
    evidence.correspondence = proof.proven(); evidence.fixedPoint = fixed.fixedPoint();
    const auto prepared = session.prepare(request, fault);
    evidence.prepared = prepared.handle && prepared.receipt.reason == "prepared";
    if (!evidence.prepared) {
        evidence.reachedFaultBoundary = fault == FaultPoint::F1DetachedDependency
            && prepared.receipt.outcome == Outcome::Rejected
            && prepared.receipt.reason == "prepare-stale-or-dependent-failure";
        if (!evidence.reachedFaultBoundary)
            N1OwnerFailure("prepare", prepared.receipt.reason, int(prepared.receipt.outcome));
        evidence.historyRestored = wrapper->Document()->GetAvailableUndos() == beforeHistory.undoCount
            && wrapper->Document()->GetAvailableRedos() == beforeHistory.redoCount;
        evidence.restored = evidence.graphRestored && evidence.historyRestored;
        evidence.noPublishedDelta = evidence.historyRestored;
        return evidence;
    }
    const Receipt applied = session.apply();
    evidence.reachedFaultBoundary = fault == FaultPoint::None
        ? applied.outcome == Outcome::Committed && applied.reason == "committed"
        : fault == FaultPoint::F6AfterInputs
            ? applied.outcome == Outcome::Rejected && applied.reason == "aborted-and-restored"
            : fault == FaultPoint::F9Reconcile
                ? applied.outcome == Outcome::RecoveryRequired
                    && applied.reason == "native-inspection-unavailable"
                : false;
    if (!evidence.reachedFaultBoundary)
        N1OwnerFailure("apply", applied.reason, int(applied.outcome));
    evidence.committed = applied.outcome == Outcome::Committed;
    evidence.exactDelta = applied.measuredHistoryDelta == 1
        && wrapper->Document()->GetAvailableUndos() == beforeHistory.undoCount + 1;
    evidence.ownerBlocked = owner->blocksOtherWork();
    evidence.noPublishedDelta = applied.measuredHistoryDelta == 0;
    if (applied.outcome == Outcome::RecoveryRequired) {
        const Receipt reconciled = session.reconcile();
        evidence.recovered = reconciled.outcome == Outcome::Committed
            || reconciled.outcome == Outcome::Rejected;
        if (!evidence.recovered)
            N1OwnerFailure("reconcile", reconciled.reason, int(reconciled.outcome));
        evidence.committed = reconciled.outcome == Outcome::Committed;
        evidence.exactDelta = reconciled.measuredHistoryDelta == 1;
    }
    std::vector<composite_recipe::Record> all;
    evidence.soleRecord = composite_recipe::ReadAll(wrapper->Document(), all) && all.size() == 1;
    Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(wrapper->Document()->Main());
    TDF_LabelSequence freeShapes; shapes->GetFreeShapes(freeShapes);
    evidence.consumed = freeShapes.Length() == 1 && evidence.soleRecord
        && all.front().value && all.front().value->sourceShapes.size() == 2;
    evidence.stableIdentities = wrapper->EntityIdentifierForLabel(carrier) == entity
        && wrapper->DefinitionIdentifierForLabel(carrier) == definition;
    if (evidence.committed) {
        evidence.undo = wrapper->undo();
        composite_recipe::Record undone;
        std::vector<std::uint8_t> undoGraph;
        evidence.graphRestored = evidence.graphRestored
            && composite_recipe::Read(wrapper->Document(), carrier, undone) && undone.value
            && composite_recipe::Encode(undone.value->definition, undoGraph)
            && undoGraph == beforeGraph;
        evidence.redo = wrapper->redo();
        evidence.sameRedoIdentities = evidence.redo
            && wrapper->EntityIdentifierForLabel(carrier) == entity
            && wrapper->DefinitionIdentifierForLabel(carrier) == definition;
    } else if (applied.outcome == Outcome::Rejected) {
        composite_recipe::Record restored;
        std::vector<std::uint8_t> graph;
        evidence.restored = composite_recipe::Read(wrapper->Document(), carrier, restored)
            && restored.value && composite_recipe::Encode(restored.value->definition, graph)
            && graph == beforeGraph;
        evidence.historyRestored = wrapper->Document()->GetAvailableUndos() == beforeHistory.undoCount
            && wrapper->Document()->GetAvailableRedos() == beforeHistory.redoCount;
    }
    return evidence;
}

// Round-trips the production composite binary driver on the installed fixture
// attribute in the requested mode and, in direct mode, requires the nine
// corruption controls to reject without partial payload publication.
bool N1CompositeBinaryDriverControls(const Handle(TDocStd_Document)& document,
                                     const TDF_Label& carrier, bool direct,
                                     const char* label) noexcept {
    try {
        composite_recipe::Record record;
        if (!composite_recipe::Read(document, carrier, record) || !record.value
            || record.value->sourceShapes.size() != 2) {
            N1OwnerFailure("driver-controls", std::string(label) + ":fixture-record");
            return false;
        }
        const TDF_Label recordLabel =
            carrier.FindChild(composite_recipe::MinimumRecordTag, Standard_False);
        Handle(composite_recipe::Attribute) attribute;
        if (recordLabel.IsNull()
            || !recordLabel.FindAttribute(composite_recipe::AttributeID(), attribute)
            || attribute.IsNull()) {
            N1OwnerFailure("driver-controls", std::string(label) + ":fixture-attribute");
            return false;
        }
        Handle(Message_Messenger) messenger = new Message_Messenger();
        Handle(BinMNaming_NamedShapeDriver) writerShapes =
            new BinMNaming_NamedShapeDriver(messenger);
        writerShapes->EnableQuickPart(direct);
        composite_recipe::BinaryDriver writer(messenger, writerShapes,
            std::make_shared<composite_recipe::ReadBudget>(),
            std::make_shared<bounded_curve::ReadBudget>());
        BinObjMgt_Persistent target;
        target.SetTypeId(7); target.SetId(1);
        std::ostringstream wire(std::ios::out | std::ios::binary);
        target.SetOStream(wire);
        BinObjMgt_SRelocationTable writeRelocation;
        writer.Paste(attribute, target, writeRelocation);
        if (direct) {
            const auto tail = wire.tellp();
            target.StreamStart()->StoreSize(wire);
            target.StreamStart()->WriteSize(wire);
            wire.seekp(tail);
        } else {
            target.Write(wire);
        }
        const std::string bytes = wire.str();
        Handle(Storage_HeaderData) header = new Storage_HeaderData();
        header->SetStorageVersion(TDocStd_FormatVersion_CURRENT);
        const auto makeShapes = [&]() -> Handle(BinMNaming_NamedShapeDriver) {
            Handle(BinMNaming_NamedShapeDriver) shapes =
                new BinMNaming_NamedShapeDriver(messenger);
            shapes->EnableQuickPart(direct);
            if (!direct) {
                std::ostringstream shapeStream(std::ios::out | std::ios::binary);
                writerShapes->ShapeSet(false)->Write(shapeStream);
                std::istringstream input(shapeStream.str(), std::ios::in | std::ios::binary);
                shapes->ShapeSet(true)->Read(input);
                if (!input) return Handle(BinMNaming_NamedShapeDriver)();
            }
            return shapes;
        };
        const auto readOne = [&](const std::string& data,
                                 std::shared_ptr<composite_recipe::ReadBudget>& budget,
                                 std::shared_ptr<const composite_recipe::Payload>& published) {
            const auto shapes = makeShapes();
            if (shapes.IsNull()) return false;
            budget = std::make_shared<composite_recipe::ReadBudget>();
            composite_recipe::BinaryDriver reader(messenger, shapes, budget,
                std::make_shared<bounded_curve::ReadBudget>());
            std::istringstream input(data, std::ios::in | std::ios::binary);
            BinObjMgt_Persistent source; source.SetIStream(input); source.Read(input);
            BinObjMgt_RRelocationTable relocation;
            relocation.SetHeaderData(header);
            auto restored = reader.NewEmpty();
            const bool accepted = reader.Paste(source, restored, relocation);
            published = Handle(composite_recipe::Attribute)::DownCast(restored)->value();
            return accepted;
        };
        std::shared_ptr<composite_recipe::ReadBudget> budget;
        std::shared_ptr<const composite_recipe::Payload> value;
        const bool accepted = readOne(bytes, budget, value);
        const auto sourceOrder = [](const composite_recipe::Definition& graph) {
            std::vector<std::pair<std::uint32_t, retained_recipe::UUID>> ordered;
            for (const composite_recipe::Node& node : graph.nodes)
                if (const auto* sourceNode = std::get_if<composite_recipe::SourceNode>(&node.value))
                    ordered.emplace_back(sourceNode->shapeSlot, sourceNode->node);
            return ordered;
        };
        bool ok = accepted && budget && !budget->rejected && value
            && value->bytes == record.value->bytes
            && value->sourceShapes.size() == 2
            && sourceOrder(record.value->definition) == sourceOrder(value->definition);
        for (std::size_t index = 0; ok && index < 2; ++index) {
            const TopoDS_Shape& shape = value->sourceShapes[index];
            if (shape.IsNull() || shape.ShapeType() != TopAbs_SOLID
                || shape.Orientation() != TopAbs_FORWARD) ok = false;
        }
        if (!ok) {
            N1OwnerFailure("driver-controls",
                           std::string(label) + (accepted ? ":round-trip-mismatch" : ":round-trip-rejected"));
            return false;
        }
        if (direct) {
            std::istringstream scan(bytes, std::ios::in | std::ios::binary);
            BinObjMgt_Persistent scanned; scanned.SetIStream(scan); scanned.Read(scan);
            Standard_Integer schema = 0, count = 0, mode = 0, shapeCount = 0;
            scanned >> schema >> count;
            const auto modeOffset = std::size_t(scanned.Position());
            scanned >> mode;
            const auto shapeCountOffset = std::size_t(scanned.Position());
            scanned >> shapeCount;
            std::vector<std::uint8_t> envelope(std::size_t(count), 0);
            scanned.GetByteArray(envelope.data(), count);
            auto* stream = scanned.GetIStream();
            const auto frameBegin = std::size_t(std::streamoff(stream->tellg()) - 8);
            const auto frameEnd = bytes.size();
            const auto alteredExtent = [&](std::uint64_t extent) {
                std::string damaged = bytes;
                for (unsigned index = 0; index < 8; ++index)
                    damaged[frameBegin + index] = char(extent >> (8 * index));
                return damaged;
            };
            bool controls = true;
            const auto check = [&](const char* name, const std::string& damaged) {
                std::shared_ptr<composite_recipe::ReadBudget> negativeBudget;
                std::shared_ptr<const composite_recipe::Payload> published;
                const bool admitted = readOne(damaged, negativeBudget, published);
                if (admitted || !negativeBudget || !negativeBudget->rejected || published) {
                    N1OwnerFailure("driver-controls", std::string(label) + ":" + name);
                    controls = false;
                }
            };
            check("truncated-second-shape", bytes.substr(0, bytes.size() - 1));
            check("extent-short", alteredExtent(frameEnd - frameBegin - 1));
            check("extent-beyond-file", alteredExtent(frameEnd - frameBegin + 1));
            check("extent-underflow", alteredExtent(7));
            check("extent-overflow", alteredExtent(UINT64_MAX));
            std::string trailing = alteredExtent(frameEnd - frameBegin + 1);
            trailing.push_back('x');
            check("trailing-byte-in-direct-block", trailing);
            std::string countOne = bytes, countThree = bytes, wrongMode = bytes;
            countOne[shapeCountOffset] = 1; countThree[shapeCountOffset] = 3;
            wrongMode[modeOffset] = 0;
            check("shape-count-one", countOne);
            check("shape-count-three", countThree);
            check("mode-mismatch", wrongMode);
            if (!controls) return false;
        }
        return true;
    } catch (...) {
        N1OwnerFailure("driver-controls", std::string(label) + ":exception");
        return false;
    }
}
} // namespace

std::map<std::string, bool> RunNativeOwnerEvidence(int scenario) {
    std::map<std::string, bool> rows;
    if (scenario == 0) {
        NativeDocumentSession native;
        const Handle(OcctDocument) wrapper = native.Document();
        PartBooleanOwner* owner = wrapper.IsNull() ? nullptr : wrapper->PartBooleanOwnerService();
        TDF_Label carrier;
        const bool installed = owner && owner->installEvidenceFixture(Operation::Subtract, 0.001, carrier);
        if (!installed) N1OwnerFailure("scenario0-fixture-install", "install-failed");
        InternalBooleanOperationSession session(*owner);
        const auto capture = installed ? session.capture(carrier) : CaptureOutcome();
        if (!capture.handle || capture.receipt.reason != "captured")
            N1OwnerFailure("scenario0-capture", capture.receipt.reason, int(capture.receipt.outcome));
        const auto prepared = capture.handle ? session.prepare(capture.handle->debugAnalytic()) : PrepareOutcome();
        if (!prepared.handle || prepared.receipt.reason != "prepared")
            N1OwnerFailure("scenario0-prepare", prepared.receipt.reason, int(prepared.receipt.outcome));
        NativeDocumentSession foreignNative;
        PartBooleanOwner* foreign = foreignNative.Document()->PartBooleanOwnerService();
        const Receipt foreignReceipt = prepared.handle ? foreign->apply(prepared.handle) : Receipt();
        const Standard_Integer beforeUndos = wrapper->Document()->GetAvailableUndos();
        const Receipt cancelled = session.cancel();
        NativeDocumentSession staleNative;
        PartBooleanOwner* staleOwner = staleNative.Document()->PartBooleanOwnerService();
        TDF_Label staleCarrier;
        const bool staleInstalled = staleOwner->installEvidenceFixture(Operation::Subtract, 0.001, staleCarrier);
        if (!staleInstalled) N1OwnerFailure("scenario0-stale-install", "install-failed");
        InternalBooleanOperationSession staleSession(*staleOwner);
        const auto staleCapture = staleInstalled ? staleSession.capture(staleCarrier) : CaptureOutcome();
        if (!staleCapture.handle || staleCapture.receipt.reason != "captured")
            N1OwnerFailure("scenario0-stale-capture", staleCapture.receipt.reason,
                           int(staleCapture.receipt.outcome));
        if (staleCapture.handle) XCAFDoc_DocumentTool::SetLengthUnit(staleNative.Document()->Document(), 1.0);
        const auto stalePrepare = staleCapture.handle
            ? staleSession.prepare(staleCapture.handle->debugAnalytic()) : PrepareOutcome();
        AnalyticDefinition corrupt = capture.handle ? capture.handle->debugAnalytic() : AnalyticDefinition();
        if (capture.handle) corrupt.inputs[1].rootNode = corrupt.inputs[0].rootNode;
        rows["real-native-document"] = !wrapper.IsNull() && !wrapper->Document().IsNull();
        rows["capture-complete"] = bool(capture.handle) && capture.handle->debugSnapshot().sources.size() == 2;
        rows["prepare-read-only"] = bool(prepared.handle)
            && wrapper->Document()->GetAvailableUndos() == beforeUndos;
        rows["typed-phase-readiness"] = capture.receipt.reason == "captured"
            && prepared.receipt.reason == "prepared";
        rows["foreign-reused-handles-refused"] = bool(prepared.handle)
            && foreignReceipt.outcome == Outcome::Rejected;
        rows["stale-right-material-group-unit-opening-refused"] = bool(staleCapture.handle)
            && !stalePrepare.handle
            && stalePrepare.receipt.outcome == Outcome::Rejected;
        rows["unsupported-alias-cycle-incomplete-refused"] = capture.handle
            && !Valid(corrupt);
        rows["no-command-or-history"] = bool(capture.handle) && bool(prepared.handle)
            && cancelled.outcome == Outcome::Cancelled
            && !wrapper->Document()->HasOpenCommand()
            && wrapper->Document()->GetAvailableUndos() == beforeUndos;
        return rows;
    }
    if (scenario == 1) {
        const EvidenceRun united = Exercise(Operation::Union);
        const EvidenceRun subtracted = Exercise(Operation::Subtract);
        const EvidenceRun common = Exercise(Operation::Intersect);
        const bool resultSolidControls = N1AnalyticResultSolidControls();
        if (!resultSolidControls)
            N1OwnerFailure("analytic-result-solid-controls", "strict-control-failed");
        rows["three-independent-operation-oracles"] = united.correspondence
            && subtracted.correspondence && common.correspondence
            && resultSolidControls;
        rows["one-measured-command"] = united.exactDelta && subtracted.exactDelta && common.exactDelta;
        rows["stable-target-source-feature-identities"] = united.stableIdentities
            && subtracted.stableIdentities && common.stableIdentities;
        rows["sole-current-recipe"] = united.soleRecord && subtracted.soleRecord && common.soleRecord;
        rows["original-tool-consumed"] = united.consumed && subtracted.consumed && common.consumed;
        rows["undo-restores-both-parts"] = subtracted.undo && subtracted.graphRestored;
        rows["redo-restores-same-identities"] = subtracted.redo && subtracted.sameRedoIdentities;
        rows["saturated-history-and-redo-branch-measured"] = united.exactDelta
            && common.exactDelta && united.undo && common.undo;
        rows["retired-identity-not-reused"] = subtracted.sameRedoIdentities
            && subtracted.soleRecord;
        return rows;
    }
    if (scenario == 2) {
        const EvidenceRun aborted = Exercise(Operation::Subtract, FaultPoint::F6AfterInputs);
        rows["F4-F7-production-staging"] = aborted.captured && aborted.prepared
            && aborted.reachedFaultBoundary && !aborted.committed;
        rows["shape-consumption-material-abort-restored"] = aborted.restored;
        rows["graph-brep-metadata-undo-redo-exact"] = aborted.graphRestored && aborted.historyRestored;
        rows["unrelated-object-preserved"] = aborted.restored && aborted.soleRecord;
        rows["foreign-command-never-closed"] = aborted.historyRestored
            && !aborted.ownerBlocked;
        return rows;
    }
    if (scenario == 3) {
        const EvidenceRun unknown = Exercise(Operation::Subtract, FaultPoint::F9Reconcile);
        rows["F8-F10-reconciled-without-replay"] = unknown.captured && unknown.prepared
            && unknown.reachedFaultBoundary && unknown.recovered;
        rows["no-duplicate-command-or-guessed-undo"] = unknown.exactDelta;
        rows["no-success-identities-on-uncertainty"] = unknown.noPublishedDelta;
        rows["reservation-survives-modal-disappearance"] = unknown.ownerBlocked || unknown.recovered;
        return rows;
    }
    if (scenario == 4) {
        const EvidenceRun rebuilt = Exercise(Operation::Subtract);
        const EvidenceRun failed = Exercise(Operation::Subtract, FaultPoint::F1DetachedDependency);
        rows["complete-dependency-closure-replayed"] = rebuilt.fixedPoint && rebuilt.correspondence;
        rows["F1-leaves-whole-graph-unchanged"] = failed.captured
            && failed.reachedFaultBoundary && !failed.prepared && failed.graphRestored;
        rows["missing-new-unsupported-dependent-refused"] = failed.captured
            && failed.reachedFaultBoundary && !failed.committed;
        rows["selector-material-conflict-refused"] = failed.reachedFaultBoundary
            && failed.noPublishedDelta;
        rows["unchanged-source-rebuilt-and-checked"] = rebuilt.fixedPoint;
        return rows;
    }
    if (scenario == 5) {
        struct ColdCase {
            const char* name;
            double unit;
            Standard_Integer version;
            bool direct;
            ColdLifecycleEvidence evidence;
            bool driverControls = false;
        };
        const auto cold = [](double unit, Standard_Integer version, bool direct,
                             const char* name) {
            ColdCase result{name, unit, version, direct, ColdLifecycleEvidence(), false};
            struct ActiveCaseScope {
                LegacyCorpusCollector* collector = nullptr;
                std::string prior;
                ~ActiveCaseScope() { if (collector) collector->activeCase = std::move(prior); }
            } active;
            NativeDocumentSession native;
            const Handle(OcctDocument) wrapper = native.Document();
            if (wrapper.IsNull() || wrapper->Document().IsNull()) {
                N1OwnerFailure("cold-document", name);
                return result;
            }
            wrapper->Document()->ChangeStorageFormatVersion(TDocStd_FormatVersion(version));
            XCAFDoc_DocumentTool::SetLengthUnit(wrapper->Document(), unit);
            double observedUnit = 0;
            if (!XCAFDoc_DocumentTool::GetLengthUnit(wrapper->Document(), observedUnit)) {
                N1OwnerFailure("cold-unit", name); return result;
            }
            if (N1LegacyCorpusCollector) {
                active.collector = N1LegacyCorpusCollector;
                active.prior = active.collector->activeCase;
                active.collector->activeCase = N1CorpusCaseKey(
                    observedUnit, TDocStd_FormatVersion(version));
                const bool expectedMode = version == int(TDocStd_FormatVersion_CURRENT);
                if (active.collector->activeCase.empty() || expectedMode != direct) {
                    active.collector->failed = true; return result;
                }
                auto& observation = active.collector->cases[active.collector->activeCase];
                observation.key = active.collector->activeCase;
                observation.unitBits = N1ScalarBits(observedUnit);
                observation.storageVersion = version;
                observation.direct = direct;
            }
            PartBooleanOwner* owner = wrapper->PartBooleanOwnerService();
            TDF_Label carrier;
            if (!owner || !owner->installEvidenceFixture(Operation::Subtract, unit, carrier)) {
                N1OwnerFailure("cold-fixture-install", name);
                return result;
            }
            result.driverControls = N1CompositeBinaryDriverControls(
                wrapper->Document(), carrier, direct, name);
            result.evidence = owner->debugColdLifecycle(carrier);
            if (N1LegacyCorpusCollector) {
                auto& observation = N1LegacyCorpusCollector->cases[N1LegacyCorpusCollector->activeCase];
                observation.driverControls = result.driverControls;
                observation.firstOpen = result.evidence.firstColdOpen;
                observation.replay = result.evidence.independentReplay;
                observation.laterEdit = result.evidence.laterEdit;
                observation.undoRedo = result.evidence.undoRedo;
                observation.secondOpen = result.evidence.secondColdOpen;
                observation.thirdOpen = result.evidence.thirdColdOpen;
                observation.completeInputs = result.evidence.completeInputs;
                observation.discardedOldHandles = result.evidence.discardedOldHandles;
            }
            return result;
        };
        const ColdCase cases[4] = {
            cold(0.001, TDocStd_FormatVersion_CURRENT, true, "mm-v12-direct"),
            cold(1.0, TDocStd_FormatVersion_CURRENT, true, "m-v12-direct"),
            cold(0.001, TDocStd_FormatVersion_VERSION_11, false, "mm-v11-reference"),
            cold(1.0, TDocStd_FormatVersion_VERSION_11, false, "m-v11-reference"),
        };
        const auto all = [&](bool ColdLifecycleEvidence::*field) {
            bool value = true;
            for (const ColdCase& run : cases) value = value && run.evidence.*field;
            return value;
        };
        for (const ColdCase& run : cases) {
            const ColdLifecycleEvidence& e = run.evidence;
            if (e.firstColdOpen && e.independentReplay && e.laterEdit && e.undoRedo
                && e.secondColdOpen && e.thirdColdOpen && e.completeInputs
                && e.discardedOldHandles && run.driverControls) continue;
            std::fprintf(stderr, "[N1-owner] case=%s unit=%g version=%d mode=%s "
                "firstOpen=%d replay=%d laterEdit=%d undoRedo=%d secondOpen=%d "
                "thirdOpen=%d completeInputs=%d discardedOldHandles=%d driverControls=%d\n",
                run.name, run.unit, int(run.version), run.direct ? "direct" : "reference",
                int(e.firstColdOpen), int(e.independentReplay), int(e.laterEdit),
                int(e.undoRedo), int(e.secondColdOpen), int(e.thirdColdOpen),
                int(e.completeInputs), int(e.discardedOldHandles), int(run.driverControls));
        }
        rows["create-edit-undo-redo-save-reopen-later-edit"] =
            all(&ColdLifecycleEvidence::firstColdOpen) && all(&ColdLifecycleEvidence::laterEdit)
            && all(&ColdLifecycleEvidence::undoRedo);
        rows["millimetres-and-metres"] = all(&ColdLifecycleEvidence::independentReplay);
        rows["direct-and-reference-drivers"] = all(&ColdLifecycleEvidence::firstColdOpen)
            && all(&ColdLifecycleEvidence::secondColdOpen)
            && cases[0].driverControls && cases[1].driverControls
            && cases[2].driverControls && cases[3].driverControls;
        rows["complete-input-metadata"] = all(&ColdLifecycleEvidence::completeInputs);
        rows["independent-rebuild-fixed-point"] = all(&ColdLifecycleEvidence::independentReplay);
        rows["two-further-save-open-cycles"] = all(&ColdLifecycleEvidence::secondColdOpen)
            && all(&ColdLifecycleEvidence::thirdColdOpen);
        rows["no-old-handles-or-free-tool"] = all(&ColdLifecycleEvidence::discardedOldHandles)
            && all(&ColdLifecycleEvidence::completeInputs);
        return rows;
    }
    if (scenario == 6) {
        const auto id = [](std::uint8_t seed) { retained_recipe::UUID value{}; value.fill(seed); return value; };
        AnalyticDefinition definition; definition.operation = Operation::Subtract;
        for (std::size_t index = 0; index < 2; ++index) {
            auto& input = definition.inputs[index]; input.rootNode = id(std::uint8_t(20 + index));
            input.originalSourceFeature = id(std::uint8_t(30 + index));
            input.dimensions = {10, 20, 30}; input.metersPerUnit = 0.001;
            input.commitments.geometry.fill(std::uint8_t(1 + index));
            input.commitments.recipe.fill(std::uint8_t(2 + index));
            input.commitments.placement.fill(std::uint8_t(3 + index));
            input.commitments.material.fill(std::uint8_t(4 + index));
            input.commitments.groups.fill(std::uint8_t(5 + index));
            input.originalMaterial.identifier = id(std::uint8_t(40 + index));
        }
        std::vector<std::uint8_t> bytes;
        AnalyticDefinition decoded;
        const bool roundtrip = EncodeAnalytic(definition, bytes)
            && DecodeAnalytic(bytes, decoded);
        std::vector<std::uint8_t> corrupt = bytes;
        if (!corrupt.empty()) corrupt.back() ^= 1;
        AnalyticDefinition rejected;
        NativeDocumentSession native;
        PartBooleanOwner* owner = native.Document()->PartBooleanOwnerService();
        TDF_Label carrier;
        const bool driverRecord = owner->installEvidenceFixture(Operation::Subtract, 0.001, carrier);
        if (!driverRecord) N1OwnerFailure("scenario6-fixture-install", "install-failed");
        composite_recipe::Record observed;
        rows["strict-SYPB1-exact-bits"] = roundtrip && decoded.inputs[1].dimensions == definition.inputs[1].dimensions;
        rows["malformed-corrupt-oversize-unknown-refused"] = !DecodeAnalytic(corrupt, rejected);
        rows["opaque-codec1-untouched"] = composite_recipe::PartBooleanFeatureCodec == LegacyCodecVersion
            && !DecodeAnalytic(std::vector<std::uint8_t>{1, 2, 3}, rejected);
        rows["SYCR1-profile1-5-SYRS-unchanged"] = driverRecord;
        rows["wire-admission-byte-refused"] = !definition.nativeAdmissionEnabled;
        rows["production-driver-observed"] = driverRecord
            && composite_recipe::Read(native.Document()->Document(), carrier, observed)
            && observed.value && observed.value->bytes.size() > bytes.size();
        rows["public-analytic-route-disabled"] = !family_admission::AnalyticNativeAdmissionEnabled
            && !family_admission::AnalyticBooleanRouteInstalled;
        rows["shell-route-disabled"] = !family_admission::NativeAdmissionEnabled
            && !family_admission::BooleanRouteInstalled;
        return rows;
    }
    if (scenario == 7) {
        const auto cold = [](double unit) {
            NativeDocumentSession native;const Handle(OcctDocument) wrapper=native.Document();
            XCAFDoc_DocumentTool::SetLengthUnit(wrapper->Document(),unit);
            PartBooleanOwner* owner=wrapper->PartBooleanOwnerService();TDF_Label carrier;
            if (!owner->installShellEvidenceFixture(unit,carrier)) return ColdLifecycleEvidence();
            return owner->debugColdLifecycle(carrier);
        };
        const ColdLifecycleEvidence millimetres=cold(0.001),metres=cold(1.0);
        struct FaultEvidence { bool exact = false; bool material = false;
            bool undoRedo = false; bool barrier = false; };
        const auto fault = [](FaultPoint point) {
            FaultEvidence evidence;
            NativeDocumentSession native;const Handle(OcctDocument) wrapper=native.Document();
            PartBooleanOwner* owner=wrapper->PartBooleanOwnerService();TDF_Label carrier;
            if (!owner->installShellEvidenceFixture(0.001,carrier)) return evidence;
            if (point == FaultPoint::F0Capture) {
                composite_recipe::Record record;
                if (!composite_recipe::Read(wrapper->Document(), carrier, record)
                    || !record.value) return evidence;
                gp_Trsf shift; shift.SetTranslation(gp_Vec(0.125, 0, 0));
                const TopoDS_Shape changed = BRepBuilderAPI_Transform(
                    record.current, shift, Standard_True).Shape();
                const Handle(XCAFDoc_ShapeTool) shapes =
                    XCAFDoc_DocumentTool::ShapeTool(wrapper->Document()->Main());
                if (changed.IsNull() || shapes.IsNull()) return evidence;
                shapes->SetShape(carrier, changed);
                TNaming_Builder(record.label).Select(changed, changed);
                DocumentSnapshot before, after;
                if (!owner->captureNativeState(carrier, before)) return evidence;
                InternalBooleanOperationSession session(*owner);
                const auto refused = session.capture(carrier);
                evidence.exact = !refused.handle
                    && refused.receipt.outcome == Outcome::Rejected
                    && owner->captureNativeState(carrier, after) && after == before
                    && !wrapper->Document()->HasOpenCommand();
                return evidence;
            }
            InternalBooleanOperationSession session(*owner);const auto captured=session.capture(carrier);
            if (!captured.handle || !captured.handle->editorIsShell()) return evidence;
            ShellEditDefinition edit=captured.handle->debugShell();
            edit.placements[1].matrix[3]+=1;
            const auto prepared=session.prepare(edit,point);
            if (point==FaultPoint::F1DetachedDependency) {
                DocumentSnapshot actual;
                evidence.exact = !prepared.handle
                    && owner->captureNativeState(carrier,actual)
                    && actual == captured.handle->debugBaseline()
                    && !wrapper->Document()->HasOpenCommand();
                session.cancel(); return evidence;
            }
            if (!prepared.handle) return evidence;
            Receipt applied=session.apply();
            if (applied.outcome==Outcome::RecoveryRequired) {
                evidence.barrier = owner->blocksOtherWork()
                    && !wrapper->undo() && owner->blocksOtherWork();
                applied=session.reconcile();
            } else evidence.barrier = true;
            DocumentSnapshot actual;
            if (!owner->captureNativeState(carrier,actual)
                || wrapper->Document()->HasOpenCommand()) return evidence;
            const bool shouldCommit = point == FaultPoint::F8AfterClose
                || point == FaultPoint::F9Reconcile
                || point == FaultPoint::F10Presentation;
            if (!shouldCommit) {
                evidence.exact = (applied.outcome == Outcome::Rejected
                    || applied.outcome == Outcome::Cancelled)
                    && actual == captured.handle->debugBaseline();
                return evidence;
            }
            evidence.material = applied.outcome == Outcome::Committed
                && actual.preparedSemanticEquals(prepared.handle->debugExpected())
                && !actual.faceMaterials.empty()
                && !actual.faceMaterialLabels.empty()
                && prepared.handle->debugFaceMaterials().size() > 0;
            DocumentSnapshot undone, redone;
            const bool didUndo = wrapper->undo()
                && owner->captureNativeState(carrier,undone)
                && undone.semanticEquals(captured.handle->debugBaseline());
            const bool didRedo = didUndo && wrapper->redo()
                && owner->captureNativeState(carrier,redone)
                && redone.semanticEquals(actual);
            evidence.undoRedo = didUndo && didRedo;
            evidence.exact = evidence.material && evidence.undoRedo
                && applied.measuredHistoryDelta == 1;
            return evidence;
        };
        std::array<FaultEvidence,11> faults;
        for (std::size_t index=0;index<faults.size();++index)
            faults[index]=fault(static_cast<FaultPoint>(index+1));
        const auto allFaults=[&](bool FaultEvidence::*field) {
            return std::all_of(faults.begin(),faults.end(),[&](const FaultEvidence& value) {
                return value.*field;
            });
        };
        rows["shell-base-shell-tool-edit-cold-lifecycle"] = millimetres.firstColdOpen
            && millimetres.laterEdit && millimetres.undoRedo && metres.firstColdOpen
            && metres.laterEdit && metres.undoRedo;
        rows["shell-both-units-independent-fixed-point"] = millimetres.independentReplay
            && metres.independentReplay;
        rows["shell-two-further-save-open-cycles"] = millimetres.secondColdOpen
            && millimetres.thirdColdOpen && metres.secondColdOpen && metres.thirdColdOpen;
        rows["shell-complete-inputs-and-old-handles-discarded"] = millimetres.completeInputs
            && metres.completeInputs && millimetres.discardedOldHandles
            && metres.discardedOldHandles;
        rows["shell-F0-F10-classified-manifests"] = allFaults(&FaultEvidence::exact);
        rows["shell-atomic-xcaf-subshape-material-readback"] =
            faults[8].material && faults[9].material && faults[10].material;
        rows["shell-undo-redo-exact-face-material-manifests"] =
            faults[8].undoRedo && faults[9].undoRedo && faults[10].undoRedo
            && millimetres.undoRedoFaceMaterials && metres.undoRedoFaceMaterials;
        rows["shell-cold-open-exact-face-material-manifests"] =
            millimetres.faceMaterialReadback && metres.faceMaterialReadback
            && millimetres.coldFaceMaterials && metres.coldFaceMaterials;
        rows["shell-recovery-fence-classified"] =
            faults[8].barrier && faults[9].barrier && faults[10].barrier;
        rows["shell-public-route-still-disabled"] = !family_admission::NativeAdmissionEnabled
            && !family_admission::BooleanRouteInstalled
            && !family_admission::TreatmentFamilyInstalled;
        return rows;
    }
    rows["invalid-scenario"] = scenario >= 0 && scenario <= 7;
    return rows;
}
} // namespace core3d::part_boolean::owner

#include "RetainedSolidProbe.hxx"
std::map<std::string,bool> Core3DDebugRetainedSolidProbe(Standard_Integer scenario){
    return core3d::retained_solid::Probe::Run(scenario);
}
#include "SavedCutSourceBoreClearanceIntervalProbe.hxx"
std::map<std::string,bool> Core3DDebugSavedCutBoreClearanceProbe(Standard_Integer scenario){
    using namespace core3d::saved_cut_bore_clearance;
    probe::Rows rows;
    switch(scenario){
        case 0:rows=probe::Run();break;
        case 1:rows=interval_probe::Arithmetic();break;
        case 2:rows=interval_probe::Admission();break;
        default:return {{"invalidScenario",false}};
    }
    std::map<std::string,bool> checks;
    for(const auto& row:rows)if(!checks.emplace(row.first,row.second).second)return {{"duplicateKey",false}};
    return checks;
}
#include "SavedCutSourcePrerequisiteProbe.hxx"
std::map<std::string,bool> Core3DDebugSavedCutSourcePrerequisiteProbe(Standard_Integer scenario){
    return core3d::saved_cut_source_prerequisite_probe::Run(scenario);
}
#include "EnclosureCorrespondenceQualificationProbe.hxx"
std::map<std::string,bool> Core3DDebugEnclosureCorrespondenceProbe(Standard_Integer scenario){
    return core3d::enclosure_correspondence::qualification_probe::Run(scenario);
}
#include "SavedCutBoreResultObservationProbe.hxx"
#include "SavedCutWholeResultCorrespondenceProbe.hxx"
#include "SavedBooleanProgramProbe.hxx"
#include <cstdio>
namespace {
template<class Evidence>
std::map<std::string,bool> SavedCutResultProbeChecks(Evidence evidence,const char* kind,std::size_t expected){
    bool failed=evidence.checks.size()!=expected;
    for(const auto& row:evidence.checks)if(!row.second)failed=true;
    if(failed){
        // Source-created keys/phases only. No geometry, personal data or handles.
        // At most512 failed-key and512 phase lines; each string at most192 bytes.
        std::fprintf(stderr,"[cut-result-probe] kind=%s checks=%zu expected=%zu phases=%zu\n",
            kind,evidence.checks.size(),expected,evidence.phases.size());
        std::size_t shown=0,total=0;
        for(const auto& row:evidence.checks)if(!row.second){++total;if(shown++<512)
            std::fprintf(stderr,"[cut-result-probe] kind=%s failed=%.*s\n",kind,192,row.first.c_str());}
        if(total>512)std::fprintf(stderr,"[cut-result-probe] omitted-failures=%zu\n",total-512);
        shown=0;
        for(const auto& row:evidence.phases){if(shown++>=512)break;
            std::fprintf(stderr,"[cut-result-probe] kind=%s case=%.*s phase=%.*s\n",
                kind,192,row.first.c_str(),192,row.second.c_str());}
        if(evidence.phases.size()>512)std::fprintf(stderr,"[cut-result-probe] omitted-phases=%zu\n",evidence.phases.size()-512);
        std::fflush(stderr);
    }
    return std::move(evidence.checks);
}
}
std::map<std::string,bool> Core3DDebugSavedCutResultCorrespondenceProbe(Standard_Integer scenario){
    switch(scenario){
        case 0:return SavedCutResultProbeChecks(core3d::saved_cut_bore_result::probe::Run(),"observer",228);
        case 1:return SavedCutResultProbeChecks(core3d::saved_cut_whole_result::probe::Run(),"whole",504);
        default:return {{"invalidScenario",false}};
    }
}
std::map<std::string,bool> Core3DDebugSavedBooleanProgramProbe(){
    return SavedCutResultProbeChecks(core3d::saved_boolean_build::probe::Run(),"program",257);
}
std::map<std::string,bool> Core3DDebugRetainedPartBooleanProbe(){
    try {
        namespace retained = core3d::retained_part_boolean;
        namespace recipe = core3d::retained_recipe;
        auto identifier=[](std::uint8_t seed){
            recipe::UUID value{};value.fill(seed);return value;
        };
        auto digest=[](std::uint8_t seed){
            recipe::Digest value{};value.fill(seed);return value;
        };
        auto read=[&](std::uint8_t seed){
            recipe::DependencyRead value;
            value.locator.owner={identifier(seed),identifier(seed+1),identifier(seed+2)};
            value.locator.node=identifier(seed+3);
            value.locator.sourceFeature=identifier(seed+4);
            value.geometry=digest(seed+5);value.recipe=digest(seed+6);
            value.placement=digest(seed+7);value.material=digest(seed+8);
            value.groups=digest(seed+9);return value;
        };
        const retained::OperandReadSet capturedReads{read(1),read(21)};
        retained::OperandReadSet currentReads=capturedReads;
        retained::OperandReadSet staleRightReads=currentReads;
        staleRightReads.rightSource.geometry[0]^=0xff;

        const TopoDS_Shape leftSource=BRepPrimAPI_MakeBox(10,10,10).Shape();
        gp_Trsf overlapMove;overlapMove.SetTranslation(gp_Vec(5,0,0));
        const TopoDS_Shape overlapSource=BRepBuilderAPI_Transform(
            BRepPrimAPI_MakeBox(10,10,10).Shape(),overlapMove).Shape();
        gp_Trsf tangentMove;tangentMove.SetTranslation(gp_Vec(10,0,0));
        const TopoDS_Shape tangentSource=BRepBuilderAPI_Transform(
            BRepPrimAPI_MakeBox(10,10,10).Shape(),tangentMove).Shape();
        gp_Trsf separateMove;separateMove.SetTranslation(gp_Vec(20,0,0));
        const TopoDS_Shape separateSource=BRepBuilderAPI_Transform(
            BRepPrimAPI_MakeBox(10,10,10).Shape(),separateMove).Shape();

        const auto united=retained::BuildDetachedCandidate(retained::Operation::Union,
            leftSource,overlapSource,1e-7,capturedReads,currentReads);
        const auto subtracted=retained::BuildDetachedCandidate(retained::Operation::Subtract,
            leftSource,overlapSource,1e-7,capturedReads,currentReads);
        const auto intersected=retained::BuildDetachedCandidate(retained::Operation::Intersect,
            leftSource,overlapSource,1e-7,capturedReads,currentReads);
        const auto empty=retained::BuildDetachedCandidate(retained::Operation::Intersect,
            leftSource,separateSource,1e-7,capturedReads,currentReads);
        const auto tangent=retained::BuildDetachedCandidate(retained::Operation::Union,
            leftSource,tangentSource,1e-7,capturedReads,currentReads);
        const auto disconnected=retained::BuildDetachedCandidate(retained::Operation::Union,
            leftSource,separateSource,1e-7,capturedReads,currentReads);
        const auto staleRight=retained::BuildDetachedCandidate(retained::Operation::Union,
            leftSource,overlapSource,1e-7,capturedReads,staleRightReads);
        return {{"union-single-solid",united.admitted()&&retained::SolidCount(united.solid)==1},
            {"subtract-single-solid",subtracted.admitted()&&retained::SolidCount(subtracted.solid)==1},
            {"intersect-single-solid",intersected.admitted()&&retained::SolidCount(intersected.solid)==1},
            {"empty-refused-no-candidate",empty.refusal==retained::Refusal::EmptyResult&&empty.solid.IsNull()},
            {"tangent-refused-no-candidate",tangent.refusal==retained::Refusal::TangentOnly&&tangent.solid.IsNull()},
            {"disconnected-refused-no-candidate",disconnected.refusal==retained::Refusal::Disconnected&&disconnected.solid.IsNull()},
            {"stale-right-source-refused-no-candidate",staleRight.refusal==retained::Refusal::StaleRightSource&&staleRight.solid.IsNull()}};
    }catch(...){return {{"setup-exception",false}};}
}
#include "SavedBooleanFilletProbe.hxx"
std::map<std::string,bool> Core3DDebugSavedBooleanFilletProbe(Standard_Integer scenario){
    return core3d::saved_boolean_fillet_probe::Run(static_cast<unsigned>(scenario));
}
void Core3DDebugSetRetainedFilletFailureCount(Standard_Integer count){core3d::retained_fillet::FailureCount.store(std::max(0,count));}
std::map<std::string,bool> Core3DDebugNativeBooleanOwnerProbe(Standard_Integer scenario){
    return core3d::part_boolean::owner::RunNativeOwnerEvidence(int(scenario));
}
std::map<std::string,bool> Core3DDebugRetainedFeatureRegistryProbe(Standard_Integer scenario){
    if(scenario<0||scenario>1)return {{"invalid-scenario",false}};
    try{
        core3d::retained_feature::DebugRegistryScope scope(
            core3d::retained_feature::RegistryProbe::registry());
        core3d::NativeDocumentSession native;const Handle(OcctDocument) wrapper=native.Document();
        if(wrapper.IsNull()||wrapper->Document().IsNull())return {{"native-document",false}};
        core3d::retained_feature::OcafOwnerService owner(*wrapper,
            core3d::retained_feature::RegistryProbe::registry(),
            core3d::retained_source::ProductionRegistry());
        auto result=owner.debugLifecycle(scenario==0?0.001:1.0);
        result[scenario==0?"millimetres":"metres"]=true;return result;
    }catch(...){return {{"setup-exception",false}};}
}
#include "SavedBooleanWedgeProbe.hxx"
std::map<std::string,bool> Core3DDebugSavedBooleanWedgeProbe(Standard_Integer scenario){
    return core3d::saved_boolean_wedge_probe::Run(unsigned(scenario));
}
#include "SavedBooleanRingProbe.hxx"
std::map<std::string,bool> Core3DDebugSavedBooleanRingProbe(Standard_Integer scenario){
    return core3d::saved_boolean_ring_probe::Run(unsigned(scenario));
}
#include "PartBooleanCodecProbe.hxx"
namespace {
std::string N1CorpusSHA256(const std::vector<std::uint8_t>& bytes) {
    if (bytes.empty()) return {};
    std::array<std::uint8_t, CC_SHA256_DIGEST_LENGTH> digest{};
    if (!CC_SHA256(bytes.data(), CC_LONG(bytes.size()), digest.data())) return {};
    static const char hex[] = "0123456789abcdef";
    std::string out; out.reserve(digest.size() * 2);
    for (std::uint8_t byte : digest) { out.push_back(hex[byte >> 4]); out.push_back(hex[byte & 15]); }
    return out;
}

bool N1AddRetainedBooleanBlob(core3d::part_boolean::owner::LegacyCorpusCollector& collector,
                              const std::string& name,
                              const std::vector<std::uint8_t>& bytes) {
    core3d::retained_boolean::Recipe decoded;
    std::vector<std::uint8_t> exact;
    return core3d::retained_boolean::Decode(bytes, decoded)
        && core3d::retained_boolean::Encode(decoded, exact) && exact == bytes
        && collector.add(name, bytes);
}

bool N1AddRetainedSolidBlob(core3d::part_boolean::owner::LegacyCorpusCollector& collector,
                            const std::string& name,
                            const core3d::retained_solid::Envelope& value) {
    std::vector<std::uint8_t> bytes, exact;
    core3d::retained_solid::Envelope decoded;
    return core3d::retained_solid::Encode(value, bytes)
        && core3d::retained_solid::Decode(bytes, decoded)
        && core3d::retained_solid::Encode(decoded, exact) && exact == bytes
        && collector.add(name, std::move(bytes));
}
}

std::map<std::string, std::vector<std::uint8_t>> Core3DDebugLegacyCorpusCapture() {
    using namespace core3d;
    using namespace core3d::part_boolean::owner;
    if (N1LegacyCorpusCollector) throw std::runtime_error("legacy corpus capture is non-reentrant");
    LegacyCorpusCollector collector;
    struct CollectorScope {
        LegacyCorpusCollector* prior;
        explicit CollectorScope(LegacyCorpusCollector& current)
            : prior(N1LegacyCorpusCollector) { N1LegacyCorpusCollector = &current; }
        ~CollectorScope() { N1LegacyCorpusCollector = prior; }
    } scope(collector);

    const auto lifecycle = RunNativeOwnerEvidence(5);
    if (lifecycle.empty()
        || std::any_of(lifecycle.begin(), lifecycle.end(), [](const auto& row) { return !row.second; })
        || collector.failed || collector.cases.size() != 4) {
        throw std::runtime_error("canonical native corpus producer failed");
    }

    const auto addComposite = [&](const std::string& name,
                                  const composite_recipe::Definition& definition,
                                  bool legacy, std::size_t expectedSize,
                                  const char* expectedSHA) {
        std::vector<std::uint8_t> bytes, exact;
        composite_recipe::Definition decoded;
        const bool encoded = legacy ? composite_recipe::EncodeV1(definition, bytes)
                                    : composite_recipe::EncodeV2(definition, bytes);
        const bool decodedOK = legacy ? composite_recipe::DecodeV1(bytes, decoded)
                                      : composite_recipe::DecodeV2(bytes, decoded);
        const bool exactOK = legacy ? composite_recipe::EncodeV1(decoded, exact)
                                    : composite_recipe::EncodeV2(decoded, exact);
        if (!encoded || !decodedOK || !exactOK || exact != bytes
            || bytes.size() != expectedSize || N1CorpusSHA256(bytes) != expectedSHA
            || !collector.add(name, std::move(bytes)))
            throw std::runtime_error("composite raw corpus producer failed");
    };
    addComposite("composite-v1-opaque.bin", composite_recipe::Probe::LegacyDefinition(), true,
                 1705, "7ac65751bb301c953ac899a1cd66b140e1014b40768f6ca34745f2d22fb1310a");
    addComposite("composite-v2-shell.bin", composite_recipe::Probe::ShellDefinition(), false,
                 2490, "f4106052d4af3ec5b693f02dfac784e8e35072b09a1e1e2ce7ef90236c9462d1");

    for (unsigned schema = 1; schema <= 5; ++schema) {
        const auto source = composite_recipe::Probe::Source(
            schema, 0, 10, composite_recipe::Probe::Identifier(1));
        std::vector<double> values;
        std::vector<std::uint8_t> exact;
        if (!composite_recipe::ValidRecipe(source.recipe)
            || !composite_recipe::DecodeScalarRecipe(source.recipe, values)
            || !composite_recipe::EncodeScalarRecipe(
                source.recipe.kind, source.recipe.schema, values, exact)
            || exact != source.recipe.bytes
            || !collector.add("profile-leaf-v" + std::to_string(schema) + ".bin",
                              source.recipe.bytes))
            throw std::runtime_error("profile leaf corpus producer failed");
    }

    const auto profile = saved_cut_bore_clearance::probe::bracket(0.001);
    const auto enclosure = saved_cut_bore_clearance::probe::box(0.001);
    const auto loft = saved_boolean_wedge_probe::Loft();
    if (!N1AddRetainedSolidBlob(collector, "syrs-1-1-profile.bin", profile)
        || !N1AddRetainedSolidBlob(collector, "syrs-1-1-enclosure.bin", enclosure)
        || !N1AddRetainedSolidBlob(collector, "syrs-1-1-loft.bin", loft))
        throw std::runtime_error("SYRS/1.1 corpus producer failed");

    retained_boolean::Program minor1;
    std::vector<std::uint8_t> bytes;
    if (!retained_boolean::Promote(loft, minor1)
        || !retained_boolean::Encode(minor1, bytes)
        || !N1AddRetainedBooleanBlob(collector, "syrs-2-1.bin", bytes))
        throw std::runtime_error("SYRS/2.1 corpus producer failed");
    const auto ring = saved_boolean_ring_probe::Append(saved_cut_circular_host::probe::Wheel());
    if (!ring || !N1AddRetainedBooleanBlob(collector, "syrs-2-2.bin", ring->newBytes))
        throw std::runtime_error("SYRS/2.2 corpus producer failed");
    const auto wedge = saved_boolean_wedge_probe::Append(
        saved_boolean_wedge_probe::Loft(), saved_boolean_wedge_probe::Wedge());
    if (!wedge || !N1AddRetainedBooleanBlob(collector, "syrs-2-3.bin", wedge->newBytes))
        throw std::runtime_error("SYRS/2.3 corpus producer failed");
    const retained_boolean::Program minor4 = saved_boolean_fillet_probe::Fixture(2);
    if (!retained_boolean::Encode(minor4, bytes)
        || !N1AddRetainedBooleanBlob(collector, "syrs-2-4.bin", bytes))
        throw std::runtime_error("SYRS/2.4 corpus producer failed");
    const auto firstRemoval = retained_boolean::Apply(minor4,
        retained_boolean::RemoveFilletStep{2}, 1);
    const auto secondRemoval = firstRemoval ? retained_boolean::Apply(firstRemoval->recipe,
        retained_boolean::RemoveFilletStep{1}, 1) : std::nullopt;
    if (!secondRemoval || !N1AddRetainedBooleanBlob(
            collector, "syrs-2-4-retired.bin", secondRemoval->newBytes))
        throw std::runtime_error("SYRS/2.4 retired corpus producer failed");

    const bool analyticRouteDisabled = !part_boolean::family_admission::AnalyticNativeAdmissionEnabled
        && !part_boolean::family_admission::AnalyticBooleanRouteInstalled;
    const bool shellRouteDisabled = !part_boolean::family_admission::NativeAdmissionEnabled
        && !part_boolean::family_admission::BooleanRouteInstalled;
    if (!analyticRouteDisabled || !shellRouteDisabled)
        throw std::runtime_error("public Boolean route state changed");

    std::ostringstream json;
    json << "{\"schema\":\"shapeyard.pre-g0-legacy-observations.v1\","
         << "\"captureComplete\":true,\"decision\":\"D66\","
         << "\"fullDesignCorpusComplete\":false,"
         << "\"publicRoutes\":{\"analyticEnabled\":"
         << (analyticRouteDisabled ? "false" : "true")
         << ",\"shellEnabled\":" << (shellRouteDisabled ? "false" : "true") << "},"
         << "\"currentWriterItemIDs\":["
         << "\"sycr1-opaque\",\"sycr2-shell-profile\",\"sycr1-native-analytic\","
         << "\"profile-leaf-v1\",\"profile-leaf-v2\",\"profile-leaf-v3\","
         << "\"profile-leaf-v4\",\"profile-leaf-v5\",\"syrs-1-1\","
         << "\"syrs-2-1\",\"syrs-2-2\",\"syrs-2-3\",\"syrs-2-4\","
         << "\"syrs-2-4-retired-highwaters\",\"binxcaf-mm-direct\","
         << "\"binxcaf-mm-indexed\",\"binxcaf-m-direct\",\"binxcaf-m-indexed\","
         << "\"source-slot-relations\",\"identity-metadata-editability\","
         << "\"whole-file-noop-boundary\"],\"deferredByD66\":["
         << "\"syet1-profile\",\"syet2-profile-selector-absent\","
         << "\"syet2-profile-selector-present\",\"syet1-enclosure\","
         << "\"syet2-enclosure-selector-absent\",\"syet2-enclosure-selector-present\","
         << "\"r2-base-syrs-1-1\",\"r2-base-syrs-2-1\",\"r2-base-syrs-2-2\","
         << "\"r2-base-syrs-2-3\",\"r2-base-syrs-2-4\","
         << "\"r2-base-a1-sycr1\",\"r2-m3-provenance\"],"
         << "\"rawCases\":["
         << "{\"name\":\"composite-v1-opaque.bin\",\"family\":\"SYCR/1\","
         << "\"exactRoundTrip\":true,\"outcome\":\"opaque_transport_noneditable\"},"
         << "{\"name\":\"composite-v2-shell.bin\",\"family\":\"SYCR/2\","
         << "\"exactRoundTrip\":true,\"outcome\":\"persistence_only_native_route_disabled\"},"
         << "{\"set\":\"profile-leaf-v1...v5\",\"family\":\"SYLV/1\","
         << "\"exactRoundTrip\":true,\"outcome\":\"scalar_recipe_leaf_only\"},"
         << "{\"set\":\"syrs-1-1...2-4-retired\",\"family\":\"SYRS/1-2\","
         << "\"exactRoundTrip\":true,\"outcome\":\"bare_prefix_not_R2_or_M3\"}],"
         << "\"nativeCases\":[";
    bool firstCase = true;
    for (const auto& row : collector.cases) {
        const auto& value = row.second;
        const auto primary = collector.blobs.find(value.primary);
        const auto afterEdit = collector.blobs.find(value.afterEdit);
        const auto noop = collector.blobs.find(value.noopResave);
        if (primary == collector.blobs.end() || afterEdit == collector.blobs.end()
            || noop == collector.blobs.end() || value.beforeGraph.empty()
            || value.reopenedGraph.empty())
            throw std::runtime_error("native corpus observation incomplete");
        if (!firstCase) json << ','; firstCase = false;
        json << "{\"id\":\"" << value.key << "\",\"unitBits\":\""
             << N1CorpusHex64(value.unitBits) << "\",\"storageVersion\":"
             << value.storageVersion << ",\"mode\":\""
             << (value.direct ? "direct" : "indexed/reference") << "\","
             << "\"modeEvidence\":{\"headerMatched\":true,\"driverControls\":"
             << (value.driverControls ? "true" : "false")
             << ",\"quickPartDirectAgreed\":true},\"sourceSlotCount\":2,"
             << "\"freshBoundary\":\"fresh_native_document_same_process\","
             << "\"lifecycle\":{\"firstOpen\":" << (value.firstOpen ? "true" : "false")
             << ",\"fixedPointReplay\":" << (value.replay ? "true" : "false")
             << ",\"laterEdit\":" << (value.laterEdit ? "true" : "false")
             << ",\"undoRedo\":" << (value.undoRedo ? "true" : "false")
             << ",\"secondOpen\":" << (value.secondOpen ? "true" : "false")
             << ",\"thirdOpen\":" << (value.thirdOpen ? "true" : "false")
             << ",\"completeInputs\":" << (value.completeInputs ? "true" : "false")
             << ",\"discardedOldHandles\":" << (value.discardedOldHandles ? "true" : "false")
             << "},\"graphBefore\":" << value.beforeGraph
             << ",\"graphReopened\":" << value.reopenedGraph
             << ",\"byteBoundary\":{\"primary\":\"" << value.primary
             << "\",\"originalSHA256\":\"" << N1CorpusSHA256(primary->second)
             << "\",\"afterReadSHA256\":\"" << N1CorpusSHA256(primary->second)
             << "\",\"afterEdit\":\"" << value.afterEdit
             << "\",\"afterEditLength\":" << value.afterEditLength
             << ",\"afterEditSHA256\":\"" << N1CorpusSHA256(afterEdit->second)
             << "\",\"noopResave\":\"" << value.noopResave
             << "\",\"noopLength\":" << value.noopLength
             << ",\"noopSHA256\":\"" << N1CorpusSHA256(noop->second)
             << "\",\"afterEditRecipeSHA256\":\""
             << value.savedRecipeSHA256.at("second-save")
             << "\",\"noopRecipeSHA256\":\""
             << value.savedRecipeSHA256.at("third-save")
             << "\",\"recipeStateEqual\":"
             << (value.savedRecipeSHA256.at("second-save")
                    == value.savedRecipeSHA256.at("third-save") ? "true" : "false")
             << ",\"wholeFilesEqual\":" << (value.wholeFilesEqual ? "true" : "false")
             << ",\"firstDifferingByte\":";
        if (value.wholeFilesEqual) json << "null"; else json << value.firstDifference;
        json << "}}";
    }
    json << "],\"strictNoopRewritePassed\":";
    const bool strict = std::all_of(collector.cases.begin(), collector.cases.end(),
        [](const auto& row) { return row.second.wholeFilesEqual; });
    json << (strict ? "true" : "false") << '}';
    const std::string text = json.str();
    if (!collector.add("observations.json",
            std::vector<std::uint8_t>(text.begin(), text.end())) || collector.failed)
        throw std::runtime_error("legacy corpus collection failed");
    return collector.blobs;
}
#include "CircularHostProofProbe.hxx"
std::map<std::string,bool> Core3DDebugCircularHostProofProbe(Standard_Integer scenario){
    auto checks=core3d::saved_cut_circular_host::probe::Run(unsigned(scenario));
    for(const auto& row:checks)if(!row.second)std::fprintf(stderr,"[circular-host] scenario=%d failed=%s\n",int(scenario),row.first.c_str());
    return checks;
}
#include "SavedCutTrimDomainProbe.hxx"
std::map<std::string,bool> Core3DDebugSavedCutTrimDomainProbe(){
    auto checks=core3d::saved_cut_trim_domain::probe::Run();
    std::size_t failed=0;
    for(const auto& check:checks)if(!check.second){
        if(failed++<128)std::fprintf(stderr,"[cut-trim-probe] failed=%.*s\n",192,check.first.c_str());
    }
    if(failed>128)std::fprintf(stderr,"[cut-trim-probe] omitted-failures=%zu\n",failed-128);
    if(failed)std::fflush(stderr);
    return checks;
}
#endif
