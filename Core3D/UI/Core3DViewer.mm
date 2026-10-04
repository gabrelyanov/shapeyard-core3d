#include "../OCCTKit/NativeMeshVertexMove.hxx"
#include "../OCCTKit/NativeMeshRegionExtrude.hxx"
//
//  Core3DViewer.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 11.04.2024.
//

#include "Core3DViewer.h"
#include "OrdinaryEditCommand.hpp"
#include "../OCCTKit/NativeOpeningDependentReplay.hxx"
#include "../OCCTKit/SavedCutWholeResultCorrespondence.hxx"
#include "../OCCTKit/AnalyticBooleanSolid.hxx"
#include "../OCCTKit/CompositeRecipeAttribute.hxx"
#include "../OCCTKit/CutDisplayPreparation.hxx"
#include "../OCCTKit/SavedBooleanProgramBuild.hxx"
#include "../OCCTKit/SavedFeatureRecords.hxx"
#include "../OCCTKit/PlanarSweepSolid.hxx"
#include "../OCCTKit/PatternAllLabelAuthority.hxx"
#include "../OCCTKit/RectangularLoftSolid.hxx"
#include "../OCCTKit/SweepRebuildDefinition.hxx"
#include "../OCCTKit/ProfileCurveFace.hxx"
#include "../OCCTKit/SplineProfileFace.hxx"
#include "../OCCTKit/SplineProfilePersistence.hxx"
#include "../OCCTKit/EnclosureGeometry.hxx"
#include "../OCCTKit/ReceiptRecord.hxx"
#include "../OCCTKit/RetainedFinishingAttribute.hxx"
#if DEBUG
// R179/D249 diagnostic-only include. Never compiled into Release.
#include "../OCCTKit/GeneralLoftPersistence.hxx"
#endif

#include <BRepFilletAPI_MakeFillet.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS.hxx>
#include <TopoDS_TShape.hxx>
#include <TopoDS_Iterator.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_DisplayMode.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Aspect_NeutralWindow.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Image_AlienPixMap.hxx>
#include <V3d_View.hxx>
#include <V3d_DirectionalLight.hxx>
#include <V3d_AmbientLight.hxx>

#include "BRepPrimAPI_MakeCylinder.hxx"
#include "BRepPrimAPI_MakeBox.hxx"
#include "BRepPrimAPI_MakeSphere.hxx"
#include "BRepPrimAPI_MakeCone.hxx"
#include "BRepPrimAPI_MakeTorus.hxx"
#include "BRepAlgoAPI_Cut.hxx"
#include "BRepAlgoAPI_Fuse.hxx"
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <BRepBndLib.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_MapOfShape.hxx>
#include <TColStd_ListIteratorOfListOfInteger.hxx>
#include <TColStd_ListOfInteger.hxx>


#include <BRepBuilderAPI_GTransform.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <exception>
#include <map>
#include <new>
#include <set>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#if TARGET_OS_OSX
#import <AppKit/NSOpenGL.h>
#include "../OCCTKit/Core3DMacImageExport.hxx"
#else
#import <UIKit/UIKit.h>
#endif
#import <CoreGraphics/CoreGraphics.h>
#import <CoreImage/CIFilter.h>

#if DEBUG
#import <Foundation/Foundation.h>
#include "../OCCTKit/SavedCutSourceEdit.hxx"
#include "../OCCTKit/SavedCutSourceViewerProbe.hxx"
#endif

namespace core3d {

namespace {
namespace tb = retained_topology_budget;

template <class Destination, class Source>
void CopyTopologyBudget(Destination& destination, const Source& source) noexcept {
    static_cast<tb::Counter&>(destination)=static_cast<const tb::Counter&>(source);
}

retained_edge_treatment::UUID MintEdgeTreatmentUUID() noexcept {
    retained_edge_treatment::UUID id{};arc4random_buf(id.data(),id.size());
    id[6]=std::uint8_t((id[6]&0x0f)|0x40);id[8]=std::uint8_t((id[8]&0x3f)|0x80);return id;
}

bool SameSelectorMembership(const retained_face_selector::FaceMembershipProof& lhs,
    const retained_face_selector::FaceMembershipProof& rhs,
    retained_edge_treatment::ReplayBudget& budget) noexcept {
    if(!budget.visit(1,tb::Site::C09ViewerIssuance))return false;
    if (lhs.intent() != rhs.intent() || lhs.plane() != rhs.plane()
        || lhs.coverage() != rhs.coverage() || lhs.wireCount() != rhs.wireCount()
        || lhs.uniqueEdgeCount() != rhs.uniqueEdgeCount()
        || lhs.selectedEdgeCount() != rhs.selectedEdgeCount()
        || lhs.boundaryUses().size() != rhs.boundaryUses().size()) return false;
    for (std::size_t index = 0; index < lhs.boundaryUses().size(); ++index) {
        if(!budget.visit(1,tb::Site::C09ViewerIssuance))return false;
        const auto& first = lhs.boundaryUses()[index];
        const auto& second = rhs.boundaryUses()[index];
        if (first.selected != second.selected || first.direction != second.direction
            || !first.edge.IsSame(second.edge) || first.ownerFaces.size() != second.ownerFaces.size()) return false;
    }
    return true;
}

// A1 first-enrollment graph admission: exactly two source nodes and one Part
// Boolean feature node whose two inputs are those sources, on a SYCR/1 wire,
// with both placements agreeing on a positive carrier unit. Anything else is
// not an admitted two-input composite and refuses before any binding is built.
struct A1GraphAdmission {
    const core3d::composite_recipe::SourceNode* left = nullptr;
    const core3d::composite_recipe::SourceNode* right = nullptr;
    const core3d::composite_recipe::FeatureNode* boolean = nullptr;
    double metersPerLocalUnit = 0;
};
bool AdmitA1Graph(const core3d::composite_recipe::Definition& graph, A1GraphAdmission& output) noexcept {
    output = A1GraphAdmission();
    if (graph.schemaVersion != 1) return false;
    std::vector<const core3d::composite_recipe::SourceNode*> sources;
    const core3d::composite_recipe::FeatureNode* boolean = nullptr;
    for (const auto& node : graph.nodes) {
        if (const auto* source = std::get_if<core3d::composite_recipe::SourceNode>(&node.value)) {
            sources.push_back(source);
        } else if (const auto* feature = std::get_if<core3d::composite_recipe::FeatureNode>(&node.value)) {
            if (boolean) return false;
            boolean = feature;
        }
    }
    if (sources.size() != 2 || !boolean || boolean->inputs.size() != 2
        || boolean->kind != core3d::composite_recipe::PartBooleanFeatureKind
        || boolean->codecVersion != core3d::composite_recipe::PartBooleanFeatureCodec
        || boolean->inputs[0] != sources[0]->node || boolean->inputs[1] != sources[1]->node
        || boolean->node != graph.outputNode) return false;
    const double unit = sources[0]->inputToCarrier.carrierMetersPerUnit;
    if (!std::isfinite(unit) || unit <= 0
        || sources[1]->inputToCarrier.carrierMetersPerUnit != unit) return false;
    output.left = sources[0]; output.right = sources[1]; output.boolean = boolean;
    output.metersPerLocalUnit = unit;
    return true;
}

bool SelectorAnchor(const retained_face_selector::BoundaryUse& use, double metersPerLocalUnit,
    retained_edge_treatment::ReplayBudget& budget,
    retained_edge_treatment::Anchor& output) {
    namespace et = retained_edge_treatment;
    if (use.ownerFaces.size() != 2 || !std::isfinite(metersPerLocalUnit)
        || metersPerLocalUnit <= 0) return false;
    if(!budget.visit(1,tb::Site::C09ViewerIssuance))return false;
    BRepAdaptor_Curve curve(use.edge);
    const auto type = curve.GetType();
    if (type != GeomAbs_Line && type != GeomAbs_Circle) return false;
    const double parameter = (curve.FirstParameter() + curve.LastParameter()) * .5;
    gp_Pnt point; gp_Vec tangent; curve.D1(parameter, point, tangent);
    if (tangent.SquareMagnitude() <= Precision::SquareConfusion()) return false;
    tangent.Normalize();
    if (use.direction == retained_face_selector::FaceUseDirection::Reversed) tangent.Reverse();
    const double millimetersPerLocal = metersPerLocalUnit * 1000.0;
    output.key = MintEdgeTreatmentUUID();
    output.curve = type == GeomAbs_Line ? et::CurveKind::Line : et::CurveKind::Circle;
    output.pointMM = {point.X() * millimetersPerLocal, point.Y() * millimetersPerLocal,
        point.Z() * millimetersPerLocal};
    output.tangent = {tangent.X(), tangent.Y(), tangent.Z()};
    output.circleRadiusMM = type == GeomAbs_Circle
        ? curve.Circle().Radius() * millimetersPerLocal : 0;
    std::array<std::array<double, 3>, 2> normals{};
    for (std::size_t index = 0; index < 2; ++index) {
        if(!budget.visit(1,tb::Site::C09ViewerIssuance))return false;
        const TopoDS_Face& face = use.ownerFaces[index];
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
        ShapeAnalysis_Surface analysis(surface);
        const gp_Pnt2d uv = analysis.ValueOfUV(point, 1e-7);
        BRepLProp_SLProps properties(BRepAdaptor_Surface(face), uv.X(), uv.Y(), 1, 1e-9);
        if (!properties.IsNormalDefined()) return false;
        gp_Dir normal = properties.Normal();
        if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
        normals[index] = {normal.X(), normal.Y(), normal.Z()};
    }
    if (normals[1] < normals[0]) std::swap(normals[0], normals[1]);
    output.normalA = normals[0]; output.normalB = normals[1];
    return true;
}

// Shared proof-to-step issuance: the native membership proof is the only
// authority input; the minted step carries the proof's selected boundary uses
// as anchors and its census as the schema-2 selector receipt, with anchors and
// receipt entries kept in the same key order.
bool IssueSelectorStep(const retained_face_selector::FaceMembershipProof& proof,
    double metersPerLocalUnit, double amountMM, std::uint64_t localID,
    retained_edge_treatment::ReplayBudget& budget,
    retained_edge_treatment::Step& step, retained_edge_treatment::Refusal& refusal) {
    namespace et = retained_edge_treatment; namespace fs = retained_face_selector;
    step = et::Step();
    step.node = MintEdgeTreatmentUUID(); step.feature = MintEdgeTreatmentUUID();
    step.localID = localID; step.kind = et::Kind::ConstantFillet; step.amountMM = amountMM;
    fs::SelectorReceipt receipt; receipt.intent = proof.intent(); receipt.face = proof.plane();
    receipt.coverage = proof.coverage(); receipt.wireCount = proof.wireCount();
    receipt.boundaryUseCount = std::uint32_t(proof.boundaryUses().size());
    receipt.boundaryUniqueEdgeCount = proof.uniqueEdgeCount();
    for (const auto& use : proof.boundaryUses()) if (use.selected) {
        et::Anchor anchor;
        if (!SelectorAnchor(use, metersPerLocalUnit, budget, anchor)) {
            refusal = budget.exhausted?et::Refusal::Budget:et::Refusal::UnsupportedEdge; return false;
        }
        step.anchors.push_back(anchor); receipt.entries.push_back({anchor.key, use.direction});
    }
    if (step.anchors.empty() || step.anchors.size() != proof.selectedEdgeCount()) {
        refusal = et::Refusal::ReplayMismatch; return false;
    }
    std::sort(step.anchors.begin(), step.anchors.end(), [](const et::Anchor& a, const et::Anchor& b) {
        return a.key < b.key;
    });
    std::sort(receipt.entries.begin(), receipt.entries.end(), [](const fs::ReceiptEntry& a,
        const fs::ReceiptEntry& b) { return a.anchorKey < b.anchorKey; });
    step.selector = std::move(receipt);
    return true;
}

// Complete value equality for a migration request: every selector's old step
// ID and full intent value, the append's presence, full intent and amount.
bool SameMigrationRequest(const retained_edge_treatment::r2::MigrationM3& first,
    const retained_edge_treatment::r2::MigrationM3& second) noexcept {
    if (first.version != second.version || first.selectors.size() != second.selectors.size()
        || bool(first.append) != bool(second.append)) return false;
    for (std::size_t index = 0; index < first.selectors.size(); ++index)
        if (first.selectors[index].oldStepID != second.selectors[index].oldStepID
            || !(first.selectors[index].intent == second.selectors[index].intent)) return false;
    return !first.append || (first.append->intent == second.append->intent
        && first.append->amountMM == second.append->amountMM);
}

// Canonical deterministic serialization of every request field for the review
// digest: no pointers, no addresses, intent values through the existing
// canonical intent writer, amount as its exact bits.
void AppendMigrationRequestDigest(std::vector<std::uint8_t>& output,
    const retained_edge_treatment::r2::MigrationM3& request) {
    retained_edge_treatment::detail::Writer writer;
    writer.u(request.version, 4);
    writer.u(request.selectors.size(), 4);
    for (const auto& binding : request.selectors) {
        writer.u(binding.oldStepID, 8);
        retained_face_selector::WriteIntent(writer, binding.intent);
    }
    writer.u(request.append ? 1 : 0, 1);
    if (request.append) {
        retained_face_selector::WriteIntent(writer, request.append->intent);
        writer.d(request.append->amountMM);
    }
    output.insert(output.end(), writer.b.begin(), writer.b.end());
}

// Typed selector-intent edit (B1 and B2 lanes): re-prove the requested intent
// against the exact pre-step shape replayed from the stored base, then replace
// only the target step's anchors and receipt through the same proof-to-step
// issuance the append paths use. Node, feature, localID, kind and amountMM are
// preserved verbatim; the caller re-encodes the candidate and downstream steps
// replay in the build. Raw-anchor replacement of a selector step stays refused.
bool B1PrepareSelectorIntentEdit(retained_edge_treatment::Definition& candidate,
    const TopoDS_Shape& storedBase, double metersPerLocalUnit,
    const retained_edge_treatment::SetSelectorIntent& edit,
    retained_edge_treatment::ReplayBudget& budget,
    retained_edge_treatment::Refusal& refusal) {
    namespace et = retained_edge_treatment; namespace fs = retained_face_selector;
    auto step = std::find_if(candidate.steps.begin(), candidate.steps.end(),
        [&](const et::Step& value) { return value.feature == edit.feature; });
    if (step == candidate.steps.end()) { refusal = et::Refusal::IdentityMismatch; return false; }
    if (!step->selector) { refusal = et::Refusal::UnsupportedOperation; return false; }
    if (!fs::ValidIntent(edit.requested)) { refusal = et::Refusal::MalformedCarrier; return false; }
    et::Definition prefix = candidate;
    prefix.steps.resize(std::size_t(step - candidate.steps.begin()));
    prefix.outputNode = prefix.steps.empty() ? candidate.base.sourceNode : prefix.steps.back().node;
    TopoDS_Shape preStep; std::vector<et::StepProof> proofs;
    if (!et::Replay(storedBase, prefix, preStep, proofs, budget, refusal)) return false;
    const std::atomic_bool cancelled{false}; fs::Resolution resolution;
    const fs::Refusal selectorRefusal = fs::Resolve(preStep, edit.requested,
        metersPerLocalUnit, budget, cancelled, resolution);
    if (selectorRefusal != fs::Refusal::None || !resolution.proof) {
        refusal = fs::MapToB1(selectorRefusal); return false;
    }
    et::Step issued;
    if (!IssueSelectorStep(*resolution.proof, metersPerLocalUnit, step->amountMM,
        step->localID, budget, issued, refusal)) return false;
    step->anchors = std::move(issued.anchors);
    step->selector = std::move(issued.selector);
    return true;
}
}

std::shared_ptr<retained_edge_treatment::Work> Core3DViewer::prepareEdgeTreatment(
    const std::shared_ptr<const retained_edge_treatment::Snapshot>& snapshot,
    const retained_edge_treatment::Edit& edit,const ObjectFrameIdentity& identity,
    std::uint64_t presentationRevision,std::uint32_t width,std::uint32_t height,
    retained_edge_treatment::Refusal& refusal,
    const retained_edge_treatment::ReplayBudget* continuation) noexcept {
    namespace et=retained_edge_treatment;refusal=et::Refusal::StaleSnapshot;
    if(![NSThread isMainThread]||!snapshot||!canBeginCommittedEdit()||myDoc.IsNull()
        ||myContext.IsNull()||!width||!height||snapshot->ownerLabel_.IsNull())return {};
    try{
        const auto scene=captureSceneSnapshot(width,height);if(!scene
            ||scene->publicationSourceIdentifier!=identity.publicationSourceIdentifier
            ||scene->revisions.documentGeneration!=identity.documentGeneration
            ||scene->revisions.model!=identity.modelRevision
            ||scene->revisions.presentation!=presentationRevision)return {};
        myContext->InitSelected();if(!myContext->MoreSelected())return {};auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
        if(myContext->MoreSelected()||selected.IsNull()||!myDoc->ShapeLabel(selected).IsEqual(snapshot->ownerLabel_)
            ||!selected->Shape().IsEqual(snapshot->current_))return {};
        et::Definition candidate=snapshot->definition_.value_or(snapshot->seed_);
        std::optional<et::ReplayBudget> intentBudget;
        if(const auto* append=std::get_if<et::Append>(&edit)){
            if(candidate.steps.size()>=et::MaximumSteps||append->anchors.empty()){refusal=et::Refusal::Budget;return {};}
            et::Step step;step.node=MintEdgeTreatmentUUID();step.feature=MintEdgeTreatmentUUID();step.localID=candidate.issuance.nextLocalID++;step.kind=append->kind;step.amountMM=append->amountMM;step.anchors=append->anchors;std::sort(step.anchors.begin(),step.anchors.end(),[](const et::Anchor&a,const et::Anchor&b){return a.key<b.key;});candidate.steps.push_back(std::move(step));candidate.outputNode=candidate.steps.back().node;
        }else if(const auto* amount=std::get_if<et::SetAmount>(&edit)){auto i=std::find_if(candidate.steps.begin(),candidate.steps.end(),[&](const et::Step&s){return s.feature==amount->feature;});if(i==candidate.steps.end()){refusal=et::Refusal::IdentityMismatch;return {};}i->amountMM=amount->amountMM;
        }else if(const auto* targets=std::get_if<et::ReplaceTargets>(&edit)){auto i=std::find_if(candidate.steps.begin(),candidate.steps.end(),[&](const et::Step&s){return s.feature==targets->feature;});if(i==candidate.steps.end()){refusal=et::Refusal::IdentityMismatch;return {};}if(i->selector){refusal=et::Refusal::UnsupportedOperation;return {};}i->anchors=targets->anchors;std::sort(i->anchors.begin(),i->anchors.end(),[](const et::Anchor&a,const et::Anchor&b){return a.key<b.key;});
        }else if(const auto* remove=std::get_if<et::Remove>(&edit)){auto i=std::find_if(candidate.steps.begin(),candidate.steps.end(),[&](const et::Step&s){return s.feature==remove->feature;});if(i==candidate.steps.end()){refusal=et::Refusal::IdentityMismatch;return {};}candidate.issuance.retiredLocalIDs.push_back(i->localID);std::sort(candidate.issuance.retiredLocalIDs.begin(),candidate.issuance.retiredLocalIDs.end());candidate.steps.erase(i);candidate.outputNode=candidate.steps.empty()?candidate.base.sourceNode:candidate.steps.back().node;
        }else if(const auto* rebuild=std::get_if<et::RebuildSource>(&edit)){std::vector<double>values;std::vector<std::uint8_t>bytes;bool encoded=false;if(const auto*p=std::get_if<profile::Parameters>(&rebuild->requested)){encoded=profile::Encode(*p,values)&&composite_recipe::EncodeScalarRecipe(composite_recipe::RecipeKind::Profile,profile::SchemaFor(*p),values,bytes);}else if(const auto*e=std::get_if<enclosure::Parameters>(&rebuild->requested)){encoded=enclosure::Encode(*e,values)&&composite_recipe::EncodeScalarRecipe(composite_recipe::RecipeKind::Enclosure,e->definition.constructionFrame?2:1,values,bytes);}else if(const auto*l=std::get_if<rectangular_loft::Definition>(&rebuild->requested)){encoded=loft_persistence::Encode(*l,values)&&composite_recipe::EncodeScalarRecipe(composite_recipe::RecipeKind::RectangularLoft,std::uint32_t(loft_persistence::Schema),values,bytes);}if(!encoded||!composite_recipe::Hash(bytes,candidate.base.sourceRecipeDigest)){refusal=et::Refusal::MalformedCarrier;return {};}}
        else if(const auto* intent=std::get_if<et::SetSelectorIntent>(&edit)){intentBudget.emplace(continuation?*continuation:snapshot->chargedBudget_);if(!B1PrepareSelectorIntentEdit(candidate,snapshot->base_,snapshot->dimensionMetersPerUnit(),*intent,*intentBudget,refusal))return {};}
        std::vector<std::uint8_t>bytes;if(!et::Encode(candidate,bytes,refusal))return {};
        auto work=std::make_shared<et::Work>();work->snapshot_=snapshot;work->edit_=edit;work->candidate_=std::move(candidate);work->label_=snapshot->ownerLabel_;work->presentation_=selected;work->state_=et::Work::State::Prepared;
        if(intentBudget)_edgeTreatmentContinuations.emplace(work.get(),*intentBudget);
        else if(continuation)_edgeTreatmentContinuations.emplace(work.get(),*continuation);
        refusal=et::Refusal::None;return work;
    }catch(...){refusal=et::Refusal::BuildFailed;return {};}
}

std::shared_ptr<const retained_edge_treatment::SelectorTargetCapture>
Core3DViewer::captureEdgeTreatmentSelectorTargets(
    const std::shared_ptr<const retained_edge_treatment::Snapshot>& original,
    const retained_face_selector::SelectorIntent& intent,
    const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
    std::uint32_t width, std::uint32_t height,
    retained_face_selector::Refusal& refusal) noexcept {
    namespace et = retained_edge_treatment; namespace fs = retained_face_selector;
    refusal = fs::Refusal::StaleSource;
    if (![NSThread isMainThread] || !original || !original->current() || myDoc.IsNull()
        || myContext.IsNull() || !width || !height || original->ownerLabel_.IsNull()) return {};
    try {
        const auto scene = captureSceneSnapshot(width, height);
        if (!scene || scene->publicationSourceIdentifier != identity.publicationSourceIdentifier
            || scene->revisions.documentGeneration != identity.documentGeneration
            || scene->revisions.model != identity.modelRevision
            || scene->revisions.presentation != presentationRevision) return {};
        myContext->InitSelected();
        if (!myContext->MoreSelected()) return {};
        const auto selected = Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        myContext->NextSelected();
        if (myContext->MoreSelected() || selected.IsNull()
            || !myDoc->ShapeLabel(selected).IsEqual(original->ownerLabel_)
            || !selected->Shape().IsEqual(original->current_)) return {};
        et::ReplayBudget budget = original->chargedBudget_;
        const std::atomic_bool cancelled{false}; fs::Resolution resolution;
        refusal = fs::Resolve(original->current_, intent, original->dimensionMetersPerUnit(),
            budget, cancelled, resolution);
#if DEBUG
        tb::debug::RecordPhase("capture", original->chargedBudget_, budget);
#endif
        if (refusal != fs::Refusal::None || !resolution.proof) return {};
        return std::shared_ptr<const et::SelectorTargetCapture>(
            new et::SelectorTargetCapture(original, resolution.proof, budget));
    } catch (...) { refusal = fs::Refusal::NativeFailure; return {}; }
}

std::shared_ptr<const retained_edge_treatment::SelectorTargetCapture>
Core3DViewer::projectEdgeTreatmentSelectorTargets(
    const std::shared_ptr<const retained_edge_treatment::SelectorTargetCapture>& targets,
    std::vector<retained_edge_treatment::CurveKind>& kinds,
    retained_face_selector::Refusal& refusal) noexcept {
    namespace et=retained_edge_treatment;namespace fs=retained_face_selector;
    kinds.clear();refusal=fs::Refusal::NativeFailure;
    if(![NSThread isMainThread]||!targets)return {};
    try{
        et::ReplayBudget budget=targets->chargedBudget_;
        kinds.reserve(targets->proof().boundaryUses().size());
        for(const auto& use:targets->proof().boundaryUses()){
            if(!budget.visit(1,tb::Site::C10ReceiptProjection)){
#if DEBUG
                tb::debug::RecordPhase("project",targets->chargedBudget_,budget);
#endif
                kinds.clear();refusal=fs::Refusal::Budget;return {};
            }
            const auto type=BRepAdaptor_Curve(use.edge).GetType();
            if(type!=GeomAbs_Line&&type!=GeomAbs_Circle){
                kinds.clear();refusal=fs::Refusal::InvalidFaceUse;return {};
            }
            kinds.push_back(type==GeomAbs_Circle?et::CurveKind::Circle:et::CurveKind::Line);
        }
#if DEBUG
        tb::debug::RecordPhase("project",targets->chargedBudget_,budget);
#endif
        refusal=fs::Refusal::None;
        return std::shared_ptr<const et::SelectorTargetCapture>(
            new et::SelectorTargetCapture(targets->original_,targets->proof_,budget));
    }catch(...){kinds.clear();return {};}
}

std::shared_ptr<retained_edge_treatment::Work> Core3DViewer::prepareEdgeTreatmentSelectorAppend(
    const std::shared_ptr<const retained_edge_treatment::Snapshot>& original,
    const std::shared_ptr<const retained_edge_treatment::SelectorTargetCapture>& targets,
    double amountMM, const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
    std::uint32_t width, std::uint32_t height,
    retained_edge_treatment::Refusal& refusal) noexcept {
    namespace et = retained_edge_treatment; namespace fs = retained_face_selector;
    refusal = et::Refusal::StaleSnapshot;
    if (![NSThread isMainThread] || !original || !targets || targets->original_ != original
        || !std::isfinite(amountMM) || amountMM <= 0 || amountMM > 20
        || !canBeginCommittedEdit() || myDoc.IsNull() || myContext.IsNull()
        || !width || !height || original->ownerLabel_.IsNull()) {
        if (std::isfinite(amountMM) && amountMM > 0 && amountMM <= 20) return {};
        refusal = et::Refusal::InvalidAmount; return {};
    }
    try {
        const auto scene = captureSceneSnapshot(width, height);
        if (!scene || scene->publicationSourceIdentifier != identity.publicationSourceIdentifier
            || scene->revisions.documentGeneration != identity.documentGeneration
            || scene->revisions.model != identity.modelRevision
            || scene->revisions.presentation != presentationRevision) return {};
        myContext->InitSelected(); if (!myContext->MoreSelected()) return {};
        const auto selected = Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        myContext->NextSelected();
        if (myContext->MoreSelected() || selected.IsNull()
            || !myDoc->ShapeLabel(selected).IsEqual(original->ownerLabel_)
            || !selected->Shape().IsEqual(original->current_)) return {};
        et::ReplayBudget budget = targets->chargedBudget_;
        const std::atomic_bool cancelled{false}; fs::Resolution refreshed;
        const fs::Refusal queryRefusal = fs::Resolve(original->current_, targets->proof().intent(),
            original->dimensionMetersPerUnit(), budget, cancelled, refreshed);
        if (queryRefusal != fs::Refusal::None || !refreshed.proof) {
#if DEBUG
            tb::debug::RecordPhase("prepare", targets->chargedBudget_, budget);
#endif
            refusal = fs::MapToB1(queryRefusal); return {};
        }
        if (!SameSelectorMembership(targets->proof(), *refreshed.proof, budget)) {
#if DEBUG
            tb::debug::RecordPhase("prepare", targets->chargedBudget_, budget);
#endif
            refusal = budget.exhausted?et::Refusal::Budget:et::Refusal::StaleSnapshot; return {};
        }
        et::Definition candidate = original->definition_.value_or(original->seed_);
        if (candidate.steps.size() >= et::MaximumSteps) { refusal = et::Refusal::Budget; return {}; }
        et::Step step;
        if (!IssueSelectorStep(*refreshed.proof, original->dimensionMetersPerUnit(), amountMM,
            candidate.issuance.nextLocalID++, budget, step, refusal)){
#if DEBUG
            tb::debug::RecordPhase("prepare", targets->chargedBudget_, budget);
#endif
            return {};}
        candidate.schema = 2;
        candidate.steps.push_back(step); candidate.outputNode = step.node;
        std::vector<std::uint8_t> encoded;
        if (!et::Encode(candidate, encoded, refusal)) return {};
        auto admitted = std::shared_ptr<et::SelectorAppendValues>(new et::SelectorAppendValues);
        admitted->captureNonce_ = original->nonce_; admitted->fence_ = original->owner_.fence;
        admitted->source_ = original->source_; admitted->base_ = candidate.base;
        admitted->edit_ = et::Append{et::Kind::ConstantFillet, amountMM, step.anchors};
        admitted->issuedStep_ = step; admitted->chargedBudget_ = budget;
#if DEBUG
        tb::debug::RecordPhase("prepare", targets->chargedBudget_, budget);
#endif
        auto work = std::make_shared<et::Work>(); work->snapshot_ = original;
        work->selectorAppend_ = admitted; work->edit_ = admitted->edit_;
        work->candidate_ = std::move(candidate); work->label_ = original->ownerLabel_;
        work->presentation_ = selected; work->state_ = et::Work::State::Prepared;
        refusal = et::Refusal::None; return work;
    } catch (...) { refusal = et::Refusal::BuildFailed; return {}; }
}

std::shared_ptr<const retained_edge_treatment::DetachedInput> Core3DViewer::edgeTreatmentGeometry(const std::shared_ptr<retained_edge_treatment::Work>&work,retained_edge_treatment::Refusal&refusal) noexcept {
    using namespace retained_edge_treatment;
    refusal=Refusal::StaleSnapshot;
    if(![NSThread isMainThread]||!work||!work->snapshot_
        ||work->state_!=Work::State::Prepared)return {};
    auto input=std::shared_ptr<DetachedInput>(new DetachedInput);
    input->selectorAppend_=work->selectorAppend_;
    // A selector capture owns main-thread document/topology authority and must
    // never cross the detached boundary. Only its admitted value adjunct does.
    if(!work->selectorAppend_) input->snapshot_=work->snapshot_;
    input->edit_=work->edit_;input->candidate_=work->candidate_;
    input->source_=work->selectorAppend_?work->selectorAppend_->source_:work->snapshot_->source_;
    input->nonce_=work->snapshot_->nonce_;
    // D253: genuinely detached, bounded native replay geometry is produced
    // here, while still at the owner boundary, through the shared verified
    // detachment helper. The worker's mutable TShapes — including untouched
    // faces a fillet may reuse — stay disjoint from live owner/source/history
    // TShapes; native witnesses resolve on the private input during replay.
    // The snapshot retains document authority; only private geometry crosses.
    // The exact D253 detachment gate is preserved; on refusal the real
    // detachment cause propagates to the caller instead of being lost.
    {ReplayBudget detachBudget=work->selectorAppend_?work->selectorAppend_->chargedBudget_
        :work->snapshot_->chargedBudget_;
    const auto continuation=_edgeTreatmentContinuations.find(work.get());
    if(continuation!=_edgeTreatmentContinuations.end()){
        detachBudget=continuation->second;_edgeTreatmentContinuations.erase(continuation);
    }
#if DEBUG
    const ReplayBudget b2DetachEntry=detachBudget;
#endif
    Refusal detachRefusal=Refusal::None;TopoDS_Shape detachedBase;
    if(!DetachReplayGeometry(work->snapshot_->base_,detachBudget,detachedBase,detachRefusal)){
#if DEBUG
        tb::debug::RecordPhase("detach",b2DetachEntry,detachBudget);
#endif
        refusal=detachRefusal;return {};}
    input->base_=detachedBase;input->chargedBudget_=detachBudget;
#if DEBUG
    tb::debug::RecordPhase("detach",b2DetachEntry,detachBudget);
#endif
    }
    input->cancelled_=work->cancelled_;work->state_=Work::State::Building;refusal=Refusal::None;return input;
}

std::shared_ptr<const retained_edge_treatment::DetachedResult> Core3DViewer::buildEdgeTreatment(const std::shared_ptr<const retained_edge_treatment::DetachedInput>&input,retained_edge_treatment::Refusal&refusal) noexcept {
    using namespace retained_edge_treatment;refusal=Refusal::BuildFailed;if(!input||!input->cancelled_||input->cancelled_->load()) {refusal=Refusal::Cancelled;return {};}
    try{auto result=std::shared_ptr<DetachedResult>(new DetachedResult);
        result->nonce_=input->nonce_;
        result->source_=std::holds_alternative<RebuildSource>(input->edit_)
            ?std::get<RebuildSource>(input->edit_).requested:input->source_;
        result->definition_=input->candidate_;result->base_=input->base_;
        result->selectorAppend_=input->selectorAppend_;result->budget_=input->chargedBudget_;
        if(!Encode(result->definition_,result->definitionBytes_,refusal)
            ||!Replay(result->base_,result->definition_,result->result_,result->proofs_,result->budget_,refusal)){
#if DEBUG
            tb::debug::RecordPhase("build",input->chargedBudget_,result->budget_);
#endif
            return {};}
#if DEBUG
        tb::debug::RecordPhase("build",input->chargedBudget_,result->budget_);
#endif
        if(input->cancelled_->load()){refusal=Refusal::Cancelled;return { };}
        refusal=Refusal::None;return result;}catch(...){refusal=Refusal::BuildFailed;return {};}
}

bool Core3DViewer::cancelEdgeTreatment(const std::shared_ptr<retained_edge_treatment::Work>&work) noexcept {if(!work)return false;work->cancelled_->store(true);if([NSThread isMainThread])work->state_=retained_edge_treatment::Work::State::Settled;return true;}

std::vector<retained_edge_treatment::Anchor> Core3DViewer::captureEdgeTreatmentTargets(
    const std::shared_ptr<const retained_edge_treatment::Snapshot>& snapshot,
    retained_edge_treatment::ReplayBudget& budget,
    retained_edge_treatment::Refusal& refusal) noexcept {
    namespace et=retained_edge_treatment;
    std::vector<et::Anchor> result;refusal=et::Refusal::UnsupportedEdge;
    if(![NSThread isMainThread]||!snapshot||myContext.IsNull()||!canBeginCommittedEdit())return result;
    try{
        budget=snapshot->chargedBudget_;
        const std::atomic_bool cancelled{false};tb::Census census;
        if(tb::CensusTopology(snapshot->current_,budget,cancelled,census,tb::Site::C27CapturedAnchor,false)
                !=tb::WalkStatus::Completed
            ||!budget.visit(census.occurrences+census.edgeUsesUnderFaces,tb::Site::C27CapturedAnchor)){
#if DEBUG
            tb::debug::RecordPhase("capture-anchors",snapshot->chargedBudget_,budget);
#endif
            refusal=et::Refusal::Budget;return {};
        }
        TopTools_IndexedDataMapOfShapeListOfShape adjacency;
        TopExp::MapShapesAndAncestors(snapshot->current_,TopAbs_EDGE,TopAbs_FACE,adjacency);
        myContext->InitSelected();
        while(myContext->MoreSelected()){
            if(!myContext->HasSelectedShape()){myContext->NextSelected();continue;}
            TopoDS_Shape selected=myContext->SelectedShape();myContext->NextSelected();
            if(selected.ShapeType()!=TopAbs_EDGE)continue;
            if(!budget.visit(1,tb::Site::C27CapturedAnchor)){refusal=et::Refusal::Budget;return {};}
            TopoDS_Edge edge=TopoDS::Edge(selected);int index=adjacency.FindIndex(edge);
            if(index<=0||adjacency.FindFromIndex(index).Extent()!=2)return {};
            if(!budget.visit(1,tb::Site::C27CapturedAnchor)){refusal=et::Refusal::Budget;return {};}
            BRepAdaptor_Curve curve(edge);double t=(curve.FirstParameter()+curve.LastParameter())*.5;
            gp_Pnt point;gp_Vec derivative;curve.D1(t,point,derivative);
            if(derivative.SquareMagnitude()<=Precision::SquareConfusion())return {};
            derivative.Normalize();et::Anchor anchor;anchor.key=MintEdgeTreatmentUUID();
            anchor.curve=curve.GetType()==GeomAbs_Line?et::CurveKind::Line
                :curve.GetType()==GeomAbs_Circle?et::CurveKind::Circle:et::CurveKind(0);
            if(int(anchor.curve)==0)return {};
            anchor.pointMM={point.X(),point.Y(),point.Z()};
            anchor.tangent={derivative.X(),derivative.Y(),derivative.Z()};
            anchor.circleRadiusMM=anchor.curve==et::CurveKind::Circle?curve.Circle().Radius():0;
            std::array<std::array<double,3>,2>normals{};int faceIndex=0;
            for(TopTools_ListIteratorOfListOfShape it(adjacency.FindFromIndex(index));it.More();it.Next()){
                if(!budget.visit(1,tb::Site::C27CapturedAnchor)){refusal=et::Refusal::Budget;return {};}
                TopoDS_Face face=TopoDS::Face(it.Value());Handle(Geom_Surface)surface=BRep_Tool::Surface(face);
                ShapeAnalysis_Surface analysis(surface);gp_Pnt2d uv=analysis.ValueOfUV(point,1e-7);
                BRepLProp_SLProps props(BRepAdaptor_Surface(face),uv.X(),uv.Y(),1,1e-9);
                if(!props.IsNormalDefined())return {};
                gp_Dir normal=props.Normal();if(face.Orientation()==TopAbs_REVERSED)normal.Reverse();
                normals[faceIndex++]={normal.X(),normal.Y(),normal.Z()};
            }
            anchor.normalA=normals[0];anchor.normalB=normals[1];result.push_back(anchor);
        }
        if(result.empty()||result.size()>et::MaximumAnchorsPerStep)return {};
        std::sort(result.begin(),result.end(),[](const et::Anchor&a,const et::Anchor&b){return a.key<b.key;});
#if DEBUG
        tb::debug::RecordPhase("capture-anchors",snapshot->chargedBudget_,budget);
#endif
        refusal=et::Refusal::None;return result;
    }catch(...){result.clear();return result;}
}

retained_face_selector::Resolution Core3DViewer::resolveFaceSelector(
    const std::shared_ptr<const retained_edge_treatment::Snapshot>& snapshot,
    const retained_face_selector::SelectorIntent& intent) noexcept {
    retained_face_selector::Resolution result;
    if (![NSThread isMainThread] || !snapshot || !snapshot->current() || myDoc.IsNull()) {
        result.refusal = retained_face_selector::Refusal::StaleSource;
        return result;
    }
    return myDoc->ResolveRetainedFaceSelector(*snapshot, intent, snapshot->owner().fence);
}

retained_edge_treatment::CommitResult Core3DViewer::commitEdgeTreatment(const std::shared_ptr<retained_edge_treatment::Work>&work,const std::shared_ptr<const retained_edge_treatment::DetachedResult>&result) noexcept {
    namespace et=retained_edge_treatment;et::CommitResult answer;if(![NSThread isMainThread]||!work||!result||work->state_!=et::Work::State::Building||result->nonce_!=work->snapshot_->nonce_){answer.outcome=et::CommitOutcome::Refused;answer.refusal=et::Refusal::StaleSnapshot;
#if DEBUG
        tb::debug::RecordCommit(int(answer.outcome),et::RefusalCode(answer.refusal),false);
#endif
        return answer;}if(work->cancelled_->load()){work->state_=et::Work::State::Settled;answer.outcome=et::CommitOutcome::Cancelled;answer.refusal=et::Refusal::Cancelled;
#if DEBUG
        tb::debug::RecordCommit(int(answer.outcome),et::RefusalCode(answer.refusal),false);
#endif
        return answer;}
    try{OrdinaryTransformChange change;change.label=work->label_;change.presentation=work->presentation_;change.shape=result->result_;change.transform=work->presentation_->LocalTransformation();change.operation=OrdinaryTransformOperation::RetainedEdgeTreatment;change.edgeTreatmentSnapshot=work->snapshot_;change.edgeTreatmentEdit=work->edit_;change.edgeTreatmentResult=result;OrdinaryEditResult ordinary=OrdinaryEditResult::Invalid;work->state_=et::Work::State::Committing;auto lease=_ordinaryEditController->beginTransform({change},&ordinary);if(lease)ordinary=lease.stageAndCommit();work->state_=et::Work::State::Settled;if(ordinary==OrdinaryEditResult::Committed){answer.outcome=et::CommitOutcome::Committed;answer.refusal=et::Refusal::None;answer.measuredUndoDelta=1;for(const auto&s:result->definition_.steps)answer.replayedFeatures.push_back(s.feature);}else if(ordinary==OrdinaryEditResult::NoChange){answer.outcome=et::CommitOutcome::Unchanged;answer.refusal=et::Refusal::None;answer.measuredUndoDelta=0;}else if(ordinary==OrdinaryEditResult::Busy){answer.outcome=et::CommitOutcome::Busy;answer.refusal=et::Refusal::Busy;}else if(ordinary==OrdinaryEditResult::OutcomeUnknown){answer.outcome=et::CommitOutcome::OutcomeUnknown;answer.refusal=et::Refusal::OutcomeUnknown;answer.measuredUndoDelta.reset();}else{answer.outcome=et::CommitOutcome::Refused;answer.refusal=ordinary==OrdinaryEditResult::BudgetRefused?et::Refusal::Budget:et::Refusal::StageFailed;}
#if DEBUG
        tb::debug::RecordCommit(int(answer.outcome),et::RefusalCode(answer.refusal),answer.outcome==et::CommitOutcome::OutcomeUnknown);
#endif
        return answer;}catch(...){work->state_=et::Work::State::Settled;answer.outcome=et::CommitOutcome::OutcomeUnknown;answer.refusal=et::Refusal::OutcomeUnknown;answer.measuredUndoDelta.reset();
#if DEBUG
        tb::debug::RecordCommit(int(answer.outcome),et::RefusalCode(answer.refusal),true);
#endif
        return answer;}
}

std::shared_ptr<const retained_edge_treatment::r2::MigrationReview>
Core3DViewer::reviewRetainedBooleanMigrationR2(
    const std::shared_ptr<const retained_edge_treatment::r2::MigrationCapture>& original,
    const retained_edge_treatment::r2::MigrationM3& request,
    const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height,retained_edge_treatment::Refusal& refusal) noexcept {
    namespace et=retained_edge_treatment;namespace r2=retained_edge_treatment::r2;
    refusal=et::Refusal::StaleSnapshot;
    if(![NSThread isMainThread]||!original||request.version!=1||request.selectors.size()>8
        ||!width||!height||myDoc.IsNull()||myContext.IsNull())return {};
    try{
        std::uint64_t previous=0;for(const auto& binding:request.selectors){
            if(!binding.oldStepID||binding.oldStepID<=previous
                ||!retained_face_selector::ValidIntent(binding.intent)){refusal=et::Refusal::MalformedCarrier;return {};}
            previous=binding.oldStepID;
        }
        if(request.append&&(!retained_face_selector::ValidIntent(request.append->intent)
            ||!std::isfinite(request.append->amountMM)||request.append->amountMM<=0||request.append->amountMM>20)){
            refusal=et::Refusal::InvalidAmount;return {};
        }
        const auto scene=captureSceneSnapshot(width,height);
        if(!scene||scene->publicationSourceIdentifier!=identity.publicationSourceIdentifier
            ||scene->revisions.documentGeneration!=identity.documentGeneration
            ||scene->revisions.model!=identity.modelRevision
            ||scene->revisions.presentation!=presentationRevision)return {};
        // Complete native review: rebuild the exact replay stages from the
        // proven pre-Boolean source base, prove the captured current shape is
        // the final replayed stage, then resolve each requested binding on the
        // stage immediately before its legacy step and the optional append on
        // the post-existing-treatment stage. Review stores proofs, stage
        // values and the charged budget only; it never issues identities.
        namespace fs=retained_face_selector;
        et::ReplayBudget budget;CopyTopologyBudget(budget,original->chargedBudget_);
        const std::atomic_bool cancelled{false};
        r2::MigrationStages stages;
        if(!r2::PrepareMigrationStages(original->original_,original->originalBase_,cancelled,
            budget,stages,refusal)){
#if DEBUG
            tb::debug::RecordPhase("review-r2",original->chargedBudget_,budget);
            std::fprintf(stderr,"B1B2_REVIEW phase=stage-preparation refusal=%s\n",et::RefusalCode(refusal));
#endif
            return {};
        }
        // Legacy fillet tail: rebuild the per-step and post-treatment stages
        // with the legacy authority's own retained-fillet build on the proven
        // post-Boolean stage -- the exact construction the live legacy commit
        // used -- so the exact mesh-independent V3 equivalence below compares
        // the captured current shape against the stage that construction
        // actually produced. Every stage stays bounded and validated, each
        // legacy step is replayed exactly once per prefix, and the
        // post-Boolean base is unchanged.
        {
            retained_boolean::Program tailProgram;
            if(const auto* capturedProgram=std::get_if<retained_boolean::Program>(&original->original_))tailProgram=*capturedProgram;
            else if(!retained_boolean::Promote(std::get<retained_boolean::Legacy>(original->original_),tailProgram)){refusal=et::Refusal::UnsupportedBase;return {};}
            if(!tailProgram.filletSteps.empty()){
                std::vector<TopoDS_Shape> mirroredStages;TopoDS_Shape mirroredPost;
                for(std::size_t prefixCount=0;prefixCount<=tailProgram.filletSteps.size();++prefixCount){
                    TopoDS_Shape stage=stages.postBoolean;
                    if(prefixCount){
                        retained_boolean::Program prefix=tailProgram;
                        prefix.filletSteps.resize(prefixCount);
                        const auto built=retained_fillet::Build(stages.postBoolean,prefix,cancelled,
                            [&](const TopoDS_Shape& shape){return et::detail::ChargeTopology(shape,budget);},
                            nullptr,&budget);
                        if(built.outcome!=retained_fillet::Outcome::Built||built.solid.IsNull()){
                            refusal=cancelled.load()?et::Refusal::Cancelled
                                :budget.exhausted?et::Refusal::Budget:et::Refusal::BuildFailed;
#if DEBUG
                            tb::debug::RecordPhase("review-r2",original->chargedBudget_,budget);
#endif
                            return {};
                        }
                        stage=built.solid;
                    }
                    if(prefixCount<tailProgram.filletSteps.size())mirroredStages.push_back(stage);
                    else mirroredPost=stage;
                }
                stages.stepStages=std::move(mirroredStages);
                stages.postTreatment=mirroredPost;
            }
        }
        // Actual/expectation roles: the live captured current is the actual;
        // the independently replayed post-treatment stage is the authoritative
        // rebuilt expectation. Only the expectation may undergo the bounded
        // binary-readback normalization inside EquivalentReplayGeometry — the
        // observed candidate is never normalized to make it pass.
        if(!et::EquivalentReplayGeometry(original->originalCurrent_,stages.postTreatment,budget,refusal)){
#if DEBUG
            tb::debug::RecordPhase("review-r2",original->chargedBudget_,budget);
            std::fprintf(stderr,"B1B2_REVIEW phase=equivalence refusal=%s\n",et::RefusalCode(refusal));
#endif
            if(refusal!=et::Refusal::Budget)refusal=et::Refusal::StaleSnapshot;return {};
        }
        const auto identityValue=retained_boolean::Identities(original->original_);
        const auto* program=std::get_if<retained_boolean::Program>(&original->original_);
        auto review=std::shared_ptr<r2::MigrationReview>(new r2::MigrationReview);
        review->original_=original;review->request_=request;
        review->postBooleanBase_=stages.postBoolean;review->postTreatmentStage_=stages.postTreatment;
        for(const auto& binding:request.selectors){
            std::size_t stageIndex=stages.stepStages.size();
            if(program)for(std::size_t index=0;index<program->filletSteps.size();++index)
                if(program->filletSteps[index].stepIdentifier==binding.oldStepID){stageIndex=index;break;}
            if(stageIndex>=stages.stepStages.size()){refusal=et::Refusal::IdentityMismatch;return {};}
            fs::Resolution resolution;
            const auto queryRefusal=fs::Resolve(stages.stepStages[stageIndex],binding.intent,
                identityValue.metersPerUnit,budget,cancelled,resolution);
            if(queryRefusal!=fs::Refusal::None||!resolution.proof){refusal=fs::MapToB1(queryRefusal);
#if DEBUG
                tb::debug::RecordPhase("review-r2",original->chargedBudget_,budget);
                std::fprintf(stderr,"B1B2_REVIEW phase=old-step-membership refusal=%s\n",et::RefusalCode(refusal));
#endif
                return {};}
            review->proofs_.push_back(resolution.proof);
        }
        if(request.append){
            fs::Resolution resolution;
            const auto queryRefusal=fs::Resolve(stages.postTreatment,request.append->intent,
                identityValue.metersPerUnit,budget,cancelled,resolution);
            if(queryRefusal!=fs::Refusal::None||!resolution.proof){refusal=fs::MapToB1(queryRefusal);
#if DEBUG
                tb::debug::RecordPhase("review-r2",original->chargedBudget_,budget);
                std::fprintf(stderr,"B1B2_REVIEW phase=append-membership refusal=%s\n",et::RefusalCode(refusal));
#endif
                return {};}
            review->appendProof_=resolution.proof;
        }
        review->chargedBudget_=original->chargedBudget_;CopyTopologyBudget(review->chargedBudget_,budget);
#if DEBUG
        tb::debug::RecordPhase("review-r2",original->chargedBudget_,budget);
#endif
        std::vector<std::uint8_t> digestBytes=original->originalBytes_;
        AppendMigrationRequestDigest(digestBytes,request);
        CC_SHA256(digestBytes.data(),CC_LONG(digestBytes.size()),review->requestDigest_.data());
        refusal=et::Refusal::None;return review;
    }catch(...){refusal=et::Refusal::BuildFailed;return {};}
}

std::shared_ptr<const retained_edge_treatment::r2::EnrollmentReview>
Core3DViewer::reviewRetainedBooleanEnrollmentR2(
    const std::shared_ptr<const retained_edge_treatment::r2::EnrollmentCapture>& original,
    const retained_edge_treatment::r2::SelectorAppendIntent& request,
    const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height,retained_edge_treatment::Refusal& refusal) noexcept {
    namespace et=retained_edge_treatment;namespace r2=retained_edge_treatment::r2;
    namespace fs=retained_face_selector;
    refusal=et::Refusal::StaleSnapshot;
    if(![NSThread isMainThread]||!original||!width||!height||myDoc.IsNull()||myContext.IsNull())return {};
    try{
        if(!fs::ValidIntent(request.intent)||!std::isfinite(request.amountMM)
            ||request.amountMM<=0||request.amountMM>20){refusal=et::Refusal::InvalidAmount;return {};}
        const auto scene=captureSceneSnapshot(width,height);
        if(!scene||scene->publicationSourceIdentifier!=identity.publicationSourceIdentifier
            ||scene->revisions.documentGeneration!=identity.documentGeneration
            ||scene->revisions.model!=identity.modelRevision
            ||scene->revisions.presentation!=presentationRevision)return {};
        // The first append's selector resolves against the exact untreated
        // carrier shape and unit; the proof travels with the review and is the
        // only source of treatment anchors for this lane.
        A1GraphAdmission admitted;
        if(!AdmitA1Graph(original->original_,admitted)||original->originalCurrent_.IsNull()){
            refusal=et::Refusal::UnsupportedBase;return {};
        }
        et::ReplayBudget budget;CopyTopologyBudget(budget,original->chargedBudget_);
        fs::Resolution resolution;const std::atomic_bool cancelled{false};
        const auto selectorRefusal=fs::Resolve(original->originalCurrent_,request.intent,
            admitted.metersPerLocalUnit,budget,cancelled,resolution);
        if(selectorRefusal!=fs::Refusal::None||!resolution.proof){
#if DEBUG
            tb::debug::RecordPhase("review-r2",original->chargedBudget_,budget);
#endif
            refusal=fs::MapToB1(selectorRefusal);return {};
        }
        auto review=std::shared_ptr<r2::EnrollmentReview>(new r2::EnrollmentReview);
        review->original_=original;review->request_=request;review->proof_=resolution.proof;
        review->chargedBudget_=original->chargedBudget_;CopyTopologyBudget(review->chargedBudget_,budget);
#if DEBUG
        tb::debug::RecordPhase("review-r2",original->chargedBudget_,budget);
#endif
        std::vector<std::uint8_t> digestBytes=original->originalBytes_;
        digestBytes.push_back(0);digestBytes.push_back(1);
        CC_SHA256(digestBytes.data(),CC_LONG(digestBytes.size()),review->requestDigest_.data());
        refusal=et::Refusal::None;return review;
    }catch(...){refusal=et::Refusal::BuildFailed;return {};}
}

std::shared_ptr<const retained_edge_treatment::r2::Snapshot> Core3DViewer::captureEdgeTreatmentR2(
    const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height,retained_edge_treatment::Refusal& refusal) noexcept {
    refusal=retained_edge_treatment::Refusal::StaleSnapshot;
    if(![NSThread isMainThread]||myDoc.IsNull()||myContext.IsNull()||!width||!height)return {};
    const auto scene=captureSceneSnapshot(width,height);if(!scene
        ||scene->publicationSourceIdentifier!=identity.publicationSourceIdentifier
        ||scene->revisions.documentGeneration!=identity.documentGeneration
        ||scene->revisions.model!=identity.modelRevision
        ||scene->revisions.presentation!=presentationRevision)return {};
    myContext->InitSelected();if(!myContext->MoreSelected())return {};
    const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
    if(myContext->MoreSelected()||selected.IsNull())return {};
    retained_recipe::RevisionFence fence;fence.documentGeneration=identity.documentGeneration;
    fence.modelRevision=identity.modelRevision;fence.effectiveMetersPerUnit=1;
    fence.ownerShape.fill(1);fence.ownerRecipe.fill(1);fence.ownerPlacement.fill(1);fence.ownerMaterial.fill(1);
    retained_recipe::DependencyRead dependency;dependency.locator.owner.document.fill(1);dependency.locator.owner.entity.fill(1);
    dependency.locator.owner.definition.fill(1);dependency.locator.node.fill(1);dependency.locator.sourceFeature.fill(1);
    dependency.geometry.fill(1);dependency.recipe.fill(1);dependency.placement.fill(1);dependency.material.fill(1);dependency.groups.fill(1);
    fence.dependencies.push_back(dependency);
    return myDoc->CaptureRetainedEdgeTreatmentR2(myDoc->ShapeLabel(selected),fence,refusal);
}

std::shared_ptr<const retained_edge_treatment::r2::MigrationCapture>
Core3DViewer::captureRetainedBooleanMigrationR2(const ObjectFrameIdentity& identity,
    std::uint64_t presentationRevision,std::uint32_t width,std::uint32_t height,
    retained_edge_treatment::Refusal& refusal) noexcept {
    refusal=retained_edge_treatment::Refusal::StaleSnapshot;
    if(![NSThread isMainThread]||myDoc.IsNull()||myContext.IsNull()||!width||!height)return {};
    const auto scene=captureSceneSnapshot(width,height);if(!scene
        ||scene->publicationSourceIdentifier!=identity.publicationSourceIdentifier
        ||scene->revisions.documentGeneration!=identity.documentGeneration
        ||scene->revisions.model!=identity.modelRevision
        ||scene->revisions.presentation!=presentationRevision)return {};
    myContext->InitSelected();if(!myContext->MoreSelected())return {};
    const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
    if(myContext->MoreSelected()||selected.IsNull())return {};
    retained_recipe::RevisionFence fence;fence.documentGeneration=identity.documentGeneration;
    fence.modelRevision=identity.modelRevision;fence.effectiveMetersPerUnit=1;
    fence.ownerShape.fill(1);fence.ownerRecipe.fill(1);fence.ownerPlacement.fill(1);fence.ownerMaterial.fill(1);
    retained_recipe::DependencyRead dependency;dependency.locator.owner.document.fill(1);dependency.locator.owner.entity.fill(1);
    dependency.locator.owner.definition.fill(1);dependency.locator.node.fill(1);dependency.locator.sourceFeature.fill(1);
    dependency.geometry.fill(1);dependency.recipe.fill(1);dependency.placement.fill(1);dependency.material.fill(1);dependency.groups.fill(1);
    fence.dependencies.push_back(dependency);
    return myDoc->CaptureRetainedBooleanMigrationR2(myDoc->ShapeLabel(selected),fence,refusal);
}

std::shared_ptr<const retained_edge_treatment::r2::EnrollmentCapture>
Core3DViewer::captureRetainedBooleanEnrollmentR2(const ObjectFrameIdentity& identity,
    std::uint64_t presentationRevision,std::uint32_t width,std::uint32_t height,
    retained_edge_treatment::Refusal& refusal) noexcept {
    refusal=retained_edge_treatment::Refusal::StaleSnapshot;
    if(![NSThread isMainThread]||myDoc.IsNull()||myContext.IsNull()||!width||!height)return {};
    const auto scene=captureSceneSnapshot(width,height);if(!scene
        ||scene->publicationSourceIdentifier!=identity.publicationSourceIdentifier
        ||scene->revisions.documentGeneration!=identity.documentGeneration
        ||scene->revisions.model!=identity.modelRevision
        ||scene->revisions.presentation!=presentationRevision)return {};
    myContext->InitSelected();if(!myContext->MoreSelected())return {};
    const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
    if(myContext->MoreSelected()||selected.IsNull())return {};
    retained_recipe::RevisionFence fence;fence.documentGeneration=identity.documentGeneration;
    fence.modelRevision=identity.modelRevision;fence.effectiveMetersPerUnit=1;
    fence.ownerShape.fill(1);fence.ownerRecipe.fill(1);fence.ownerPlacement.fill(1);fence.ownerMaterial.fill(1);
    retained_recipe::DependencyRead dependency;dependency.locator.owner.document.fill(1);dependency.locator.owner.entity.fill(1);
    dependency.locator.owner.definition.fill(1);dependency.locator.node.fill(1);dependency.locator.sourceFeature.fill(1);
    dependency.geometry.fill(1);dependency.recipe.fill(1);dependency.placement.fill(1);dependency.material.fill(1);dependency.groups.fill(1);
    fence.dependencies.push_back(dependency);
    return myDoc->CaptureRetainedBooleanEnrollmentR2(myDoc->ShapeLabel(selected),fence,refusal);
}

std::shared_ptr<retained_edge_treatment::r2::Work>
Core3DViewer::prepareRetainedBooleanMigrationR2(
    const std::shared_ptr<const retained_edge_treatment::r2::MigrationCapture>& original,
    const retained_edge_treatment::r2::MigrationM3& request,
    const std::shared_ptr<const retained_edge_treatment::r2::MigrationReview>& review,
    const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height,retained_edge_treatment::Refusal& refusal) noexcept {
    namespace et=retained_edge_treatment;namespace r2=retained_edge_treatment::r2;
    refusal=et::Refusal::StaleSnapshot;
    if(![NSThread isMainThread]||!original||!review||review->original_!=original
        ||request.version!=1||!canBeginCommittedEdit()||myDoc.IsNull()||myContext.IsNull())return {};
    try{
        et::ReplayBudget budget;CopyTopologyBudget(budget,review->chargedBudget_);
        myContext->InitSelected();if(!myContext->MoreSelected())return {};
        auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
        if(myContext->MoreSelected()||selected.IsNull()||!myDoc->ShapeLabel(selected).IsEqual(original->ownerLabel_)
            ||!selected->Shape().IsEqual(original->originalCurrent_))return {};
        // Exact request identity: the passed request must be the complete
        // reviewed value, the review must carry every binding proof and the
        // reviewed stage values, and the recomputed request digest must match.
        if(!SameMigrationRequest(request,review->request_)
            ||review->proofs_.size()!=request.selectors.size()
            ||bool(request.append)!=bool(review->appendProof_)
            ||review->postBooleanBase_.IsNull()){refusal=et::Refusal::IdentityMismatch;return {};}
        {std::vector<std::uint8_t> digestBytes=original->originalBytes_;
        AppendMigrationRequestDigest(digestBytes,request);
        r2::Digest requestDigest{};
        CC_SHA256(digestBytes.data(),CC_LONG(digestBytes.size()),requestDigest.data());
        if(requestDigest!=review->requestDigest_){refusal=et::Refusal::IdentityMismatch;return {};}}
        auto prefix=original->original_;
        const auto identityValue=retained_boolean::Identities(prefix);
        std::uint64_t nextOperandID=std::uint64_t(1),nextStepID=1,nextEdgeID=1;
        std::vector<retained_fillet::Step> imported;
        std::uint8_t wireMajor=1,wireMinor=1;
        if(auto* program=std::get_if<retained_boolean::Program>(&prefix)){
            wireMajor=2;wireMinor=program->codecMinor;nextOperandID=program->nextOperandID;
            nextStepID=program->nextFilletStepID;nextEdgeID=program->nextFilletEdgeID;
            imported=program->filletSteps;program->filletSteps.clear();
        }else nextOperandID=std::uint64_t(std::get<retained_boolean::Legacy>(prefix).operandID)+1;
        std::vector<std::uint8_t> prefixBytes;if(!retained_boolean::Encode(prefix,prefixBytes)){
            refusal=et::Refusal::MalformedCarrier;return {};
        }
        r2::BooleanBaseBinding binding;binding.sourceWireMajor=wireMajor;binding.sourceWireMinor=wireMinor;
        binding.source={identityValue.document,identityValue.entity,identityValue.definition,identityValue.sourceFeature};
        binding.sourceSchema=std::visit([](const auto& value){if constexpr(std::is_same_v<std::decay_t<decltype(value)>,retained_boolean::Legacy>)return value.sourceSchema;else return value.source.schema;},prefix);
        binding.metersPerLocalUnit=identityValue.metersPerUnit;
        CC_SHA256(prefixBytes.data(),CC_LONG(prefixBytes.size()),binding.sourceRecipeDigest.data());
        r2::LegacyPrefixBinding links;links.rootNode=MintEdgeTreatmentUUID();
        const std::size_t operationCount=std::visit([](const auto& value)->std::size_t{if constexpr(std::is_same_v<std::decay_t<decltype(value)>,retained_boolean::Legacy>)return 1;else return value.steps.size();},prefix);
        r2::UUID left=links.rootNode;
        for(std::size_t index=0;index<operationCount;++index){r2::LegacyLink link;
            link.operandID=std::visit([&](const auto& value)->std::uint32_t{if constexpr(std::is_same_v<std::decay_t<decltype(value)>,retained_boolean::Legacy>)return value.operandID;else return value.steps[index].operand.identifier;},prefix);
            link.node=MintEdgeTreatmentUUID();link.feature=index+1==operationCount?identityValue.derivedFeature:MintEdgeTreatmentUUID();
            link.toolNode=MintEdgeTreatmentUUID();link.toolFeature=MintEdgeTreatmentUUID();link.leftNode=left;link.rightNode=link.toolNode;
            left=link.node;links.links.push_back(link);
        }
        binding.sourceNode=left;binding.prefix=links;
        r2::MigrationM3Provenance provenance;provenance.originalRecipeDigest={};
        CC_SHA256(original->originalBytes_.data(),CC_LONG(original->originalBytes_.size()),provenance.originalRecipeDigest.data());
        provenance.nextOperandID=nextOperandID;provenance.nextFilletStepID=nextStepID;provenance.nextFilletEdgeID=nextEdgeID;
        r2::Definition candidate;candidate.owner={identityValue.document,identityValue.entity,identityValue.definition};
        candidate.base=binding;candidate.issuance.nextLocalID=nextStepID;candidate.outputNode=binding.sourceNode;
        const double mmPerLocal=identityValue.metersPerUnit*1000;
        for(const auto& old:imported){et::Step step;step.localID=old.stepIdentifier;step.node=MintEdgeTreatmentUUID();step.feature=MintEdgeTreatmentUUID();
            step.kind=et::Kind::ConstantFillet;step.amountMM=old.radiusLocal*mmPerLocal;
            for(const auto& oldAnchor:old.anchors){et::Anchor anchor;anchor.key=MintEdgeTreatmentUUID();anchor.curve=et::CurveKind(oldAnchor.curveKind);
                for(unsigned axis=0;axis<3;++axis){anchor.pointMM[axis]=oldAnchor.anchorPoint[axis]*mmPerLocal;anchor.tangent[axis]=oldAnchor.axis[axis];}
                anchor.normalA={1,0,0};anchor.normalB={0,1,0};anchor.circleRadiusMM=oldAnchor.circleRadius*mmPerLocal;step.anchors.push_back(anchor);
                provenance.anchors.push_back({oldAnchor.identifier,anchor.key,false});}
            std::sort(step.anchors.begin(),step.anchors.end(),[](const et::Anchor&a,const et::Anchor&b){return a.key<b.key;});
            candidate.steps.push_back(step);provenance.steps.push_back({old.stepIdentifier,step.node,step.feature,false});candidate.outputNode=step.node;
        }
        // Re-anchor every imported step named by a reviewed binding from that
        // binding's proof only; the step keeps its provenance-mapped
        // node/feature/local ID and the replaced legacy anchors are marked
        // retired in the provenance anchor map. The optional append is issued
        // from the reviewed append proof, exactly as the A1 lane issues it.
        for(std::size_t index=0;index<request.selectors.size();++index){
            const auto oldStepID=request.selectors[index].oldStepID;
            auto row=std::find_if(candidate.steps.begin(),candidate.steps.end(),
                [&](const et::Step& value){return value.localID==oldStepID;});
            if(row==candidate.steps.end()){refusal=et::Refusal::IdentityMismatch;return {};}
            et::Step issued;
            if(!IssueSelectorStep(*review->proofs_[index],binding.metersPerLocalUnit,
                row->amountMM,row->localID,budget,issued,refusal)){
#if DEBUG
                tb::debug::RecordPhase("prepare-r2",review->chargedBudget_,budget);
#endif
                return {};}
            issued.node=row->node;issued.feature=row->feature;
            for(auto& mapping:provenance.anchors)
                if(std::any_of(row->anchors.begin(),row->anchors.end(),
                    [&](const et::Anchor& anchor){return anchor.key==mapping.key;}))mapping.retired=true;
            *row=std::move(issued);
        }
        if(request.append){
            if(candidate.steps.size()>=et::MaximumSteps){refusal=et::Refusal::Budget;return {};}
            et::Step issued;
            if(!IssueSelectorStep(*review->appendProof_,binding.metersPerLocalUnit,
                request.append->amountMM,candidate.issuance.nextLocalID++,budget,issued,refusal)){
#if DEBUG
                tb::debug::RecordPhase("prepare-r2",review->chargedBudget_,budget);
#endif
                return {};}
            candidate.steps.push_back(std::move(issued));candidate.outputNode=candidate.steps.back().node;
        }
        binding.migration=std::move(provenance);candidate.base=binding;
        std::vector<std::uint8_t> bytes;if(!r2::Encode(candidate,bytes,refusal))return {};
        auto work=std::make_shared<r2::Work>();work->migration_=original;work->review_=review;work->mutation_=request;
        work->candidate_=candidate;work->source_=r2::RetainedBooleanBase{binding,r2::LegacyBooleanBase{prefix,prefixBytes}};
        work->prefixBytes_=prefixBytes;work->base_=review->postBooleanBase_;work->originalCurrent_=original->originalCurrent_;
        work->chargedBudget_=review->chargedBudget_;CopyTopologyBudget(work->chargedBudget_,budget);
#if DEBUG
        tb::debug::RecordPhase("prepare-r2",review->chargedBudget_,budget);
#endif
        work->nonce_=original->nonce_;work->label_=original->ownerLabel_;work->presentation_=selected;work->state_=r2::Work::State::Prepared;
        refusal=et::Refusal::None;return work;
    }catch(...){refusal=et::Refusal::BuildFailed;return {};}
}

std::shared_ptr<retained_edge_treatment::r2::Work>
Core3DViewer::prepareRetainedBooleanEnrollmentR2(
    const std::shared_ptr<const retained_edge_treatment::r2::EnrollmentCapture>& original,
    const std::shared_ptr<const retained_edge_treatment::r2::EnrollmentReview>& review,
    const ObjectFrameIdentity&,std::uint64_t,std::uint32_t,std::uint32_t,
    retained_edge_treatment::Refusal& refusal) noexcept {
    namespace et=retained_edge_treatment;namespace r2=retained_edge_treatment::r2;
    namespace fs=retained_face_selector;
    refusal=et::Refusal::StaleSnapshot;
    if(![NSThread isMainThread]||!original||!review||review->original_!=original
        ||!review->proof_||!canBeginCommittedEdit()||myDoc.IsNull()||myContext.IsNull())return {};
    try{
        et::ReplayBudget budget;CopyTopologyBudget(budget,review->chargedBudget_);
        myContext->InitSelected();if(!myContext->MoreSelected())return {};
        auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
        if(myContext->MoreSelected()||selected.IsNull()||!myDoc->ShapeLabel(selected).IsEqual(original->ownerLabel_)
            ||!selected->Shape().IsEqual(original->originalCurrent_))return {};
        const auto& graph=original->original_;
        A1GraphAdmission admitted;
        if(!AdmitA1Graph(graph,admitted)){refusal=et::Refusal::UnsupportedBase;return {};}
        // The reviewed request is the only mutation source: the exact reviewed
        // amount and the proof resolved against the untreated carrier shape.
        r2::BooleanBaseBinding binding;binding.format=r2::PrefixFormat::A1Composite;
        binding.sourceWireMajor=1;binding.sourceWireMinor=0;
        binding.source={graph.owner.document,graph.owner.entity,graph.owner.definition,
            admitted.left->original.sourceFeature};
        binding.sourceNode=admitted.boolean->node;binding.sourceSchema=graph.schemaVersion;
        binding.metersPerLocalUnit=admitted.metersPerLocalUnit;
        CC_SHA256(original->originalBytes_.data(),CC_LONG(original->originalBytes_.size()),
            binding.sourceRecipeDigest.data());
        r2::CompositePrefixBinding links;
        links.links.push_back({admitted.boolean->node,admitted.boolean->feature,
            admitted.boolean->inputs[0],admitted.boolean->inputs[1]});
        binding.prefix=links;
        r2::Definition candidate;candidate.owner=graph.owner;candidate.base=binding;
        candidate.issuance.nextLocalID=1;candidate.outputNode=binding.sourceNode;
        et::Step step;
        if(!IssueSelectorStep(*review->proof_,binding.metersPerLocalUnit,review->request_.amountMM,
            candidate.issuance.nextLocalID++,budget,step,refusal)){
#if DEBUG
            tb::debug::RecordPhase("prepare-r2",review->chargedBudget_,budget);
#endif
            return {};}
        candidate.steps.push_back(step);candidate.outputNode=step.node;
        std::vector<std::uint8_t> bytes;if(!r2::Encode(candidate,bytes,refusal))return {};
        auto work=std::make_shared<r2::Work>();work->enrollment_=original;work->candidate_=candidate;
        work->source_=r2::RetainedBooleanBase{binding,r2::CompositeBooleanBase{graph,original->originalBytes_}};
        work->prefixBytes_=original->originalBytes_;work->base_=original->originalCurrent_;
        work->originalCurrent_=original->originalCurrent_;
        work->chargedBudget_=review->chargedBudget_;CopyTopologyBudget(work->chargedBudget_,budget);
#if DEBUG
        tb::debug::RecordPhase("prepare-r2",review->chargedBudget_,budget);
#endif
        work->nonce_=original->nonce_;work->label_=original->ownerLabel_;work->presentation_=selected;
        work->state_=r2::Work::State::Prepared;
        refusal=et::Refusal::None;return work;
    }catch(...){refusal=et::Refusal::BuildFailed;return {};}
}

std::shared_ptr<const retained_edge_treatment::r2::SelectorTargetCapture>
Core3DViewer::captureEdgeTreatmentSelectorTargetsR2(
    const std::shared_ptr<const retained_edge_treatment::r2::Snapshot>& original,
    const retained_face_selector::SelectorIntent& intent,const ObjectFrameIdentity&,
    std::uint64_t,std::uint32_t,std::uint32_t,retained_face_selector::Refusal& refusal) noexcept {
    namespace et=retained_edge_treatment;namespace r2=retained_edge_treatment::r2;namespace fs=retained_face_selector;
    refusal=fs::Refusal::StaleSource;if(![NSThread isMainThread]||!original||!original->current())return {};
    et::ReplayBudget budget;CopyTopologyBudget(budget,original->chargedBudget_);
    fs::Resolution resolution;const std::atomic_bool cancelled{false};
    refusal=fs::Resolve(original->current_,intent,original->dimensionMetersPerUnit(),budget,cancelled,resolution);
#if DEBUG
    tb::debug::RecordPhase("capture-r2",original->chargedBudget_,budget);
#endif
    if(refusal!=fs::Refusal::None||!resolution.proof)return {};
    auto value=std::shared_ptr<r2::SelectorTargetCapture>(new r2::SelectorTargetCapture);
    value->original_=original;value->proof_=resolution.proof;value->chargedBudget_=original->chargedBudget_;
    CopyTopologyBudget(value->chargedBudget_,budget);return value;
}

std::shared_ptr<retained_edge_treatment::r2::Work>
Core3DViewer::prepareEdgeTreatmentSelectorAppendR2(
    const std::shared_ptr<const retained_edge_treatment::r2::Snapshot>& original,
    const std::shared_ptr<const retained_edge_treatment::r2::SelectorTargetCapture>& targets,
    double amountMM,const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height,
    retained_edge_treatment::Refusal& refusal) noexcept {
    namespace et=retained_edge_treatment;namespace r2=retained_edge_treatment::r2;namespace fs=retained_face_selector;
    refusal=et::Refusal::StaleSnapshot;
    if(![NSThread isMainThread]||!original||!targets||targets->original_!=original
        ||!std::isfinite(amountMM)||amountMM<=0||amountMM>20||original->definition_.steps.size()>=et::MaximumSteps
        ||!canBeginCommittedEdit()||myDoc.IsNull()||myContext.IsNull()||!width||!height
        ||original->ownerLabel_.IsNull())return {};
    const auto scene=captureSceneSnapshot(width,height);
    if(!scene||scene->publicationSourceIdentifier!=identity.publicationSourceIdentifier
        ||scene->revisions.documentGeneration!=identity.documentGeneration
        ||scene->revisions.model!=identity.modelRevision
        ||scene->revisions.presentation!=presentationRevision)return {};
    myContext->InitSelected();if(!myContext->MoreSelected())return {};
    const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
    if(myContext->MoreSelected()||selected.IsNull()||!myDoc->ShapeLabel(selected).IsEqual(original->ownerLabel_)
        ||!selected->Shape().IsEqual(original->current_))return {};
    et::ReplayBudget budget;CopyTopologyBudget(budget,targets->chargedBudget_);
    r2::Definition candidate=original->definition_;et::Step step;
    if(!IssueSelectorStep(*targets->proof_,original->dimensionMetersPerUnit(),amountMM,
        candidate.issuance.nextLocalID++,budget,step,refusal)){
#if DEBUG
        tb::debug::RecordPhase("prepare-r2",targets->chargedBudget_,budget);
#endif
        return {};}
    candidate.steps.push_back(step);candidate.outputNode=step.node;std::vector<std::uint8_t> bytes;if(!r2::Encode(candidate,bytes,refusal))return {};
    auto work=std::make_shared<r2::Work>();work->snapshot_=original;work->candidate_=candidate;work->source_=original->source_;
    work->base_=original->base_;work->originalCurrent_=original->current_;work->nonce_=original->nonce_;work->label_=original->ownerLabel_;
    work->chargedBudget_=targets->chargedBudget_;CopyTopologyBudget(work->chargedBudget_,budget);
#if DEBUG
    tb::debug::RecordPhase("prepare-r2",targets->chargedBudget_,budget);
#endif
    work->presentation_=selected;work->state_=r2::Work::State::Prepared;refusal=et::Refusal::None;return work;
}

std::shared_ptr<retained_edge_treatment::r2::Work> Core3DViewer::prepareEdgeTreatmentEditR2(
    const std::shared_ptr<const retained_edge_treatment::r2::Snapshot>& original,
    const retained_edge_treatment::r2::Edit& edit,const ObjectFrameIdentity& identity,
    std::uint64_t presentationRevision,std::uint32_t width,std::uint32_t height,
    retained_edge_treatment::Refusal& refusal) noexcept {
    namespace et=retained_edge_treatment;namespace r2=retained_edge_treatment::r2;refusal=et::Refusal::StaleSnapshot;
    if(![NSThread isMainThread]||!original||!canBeginCommittedEdit()||myDoc.IsNull()
        ||myContext.IsNull()||!width||!height||original->ownerLabel_.IsNull())return {};
    const auto scene=captureSceneSnapshot(width,height);
    if(!scene||scene->publicationSourceIdentifier!=identity.publicationSourceIdentifier
        ||scene->revisions.documentGeneration!=identity.documentGeneration
        ||scene->revisions.model!=identity.modelRevision
        ||scene->revisions.presentation!=presentationRevision)return {};
    myContext->InitSelected();if(!myContext->MoreSelected())return {};
    const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
    if(myContext->MoreSelected()||selected.IsNull()||!myDoc->ShapeLabel(selected).IsEqual(original->ownerLabel_)
        ||!selected->Shape().IsEqual(original->current_))return {};
    r2::Definition candidate=original->definition_;
    std::optional<et::ReplayBudget> intentBudget;
    std::shared_ptr<const et::SourceRebindRoles> rebindRoles;
    if(const auto* amount=std::get_if<et::SetAmount>(&edit)){auto row=std::find_if(candidate.steps.begin(),candidate.steps.end(),[&](const et::Step&s){return s.feature==amount->feature;});if(row==candidate.steps.end()){refusal=et::Refusal::IdentityMismatch;return {};}row->amountMM=amount->amountMM;}
    else if(const auto* remove=std::get_if<et::Remove>(&edit)){auto row=std::find_if(candidate.steps.begin(),candidate.steps.end(),[&](const et::Step&s){return s.feature==remove->feature;});if(row==candidate.steps.end()){refusal=et::Refusal::IdentityMismatch;return {};}candidate.issuance.retiredLocalIDs.push_back(row->localID);candidate.steps.erase(row);candidate.outputNode=candidate.steps.empty()?std::get<r2::BooleanBaseBinding>(candidate.base).sourceNode:candidate.steps.back().node;}
    else if(const auto* rebuild=std::get_if<r2::RebuildBooleanInput>(&edit)){
        // Admitted only for the proven source family of a legacy SYRS prefix:
        // the locator must name the captured left source input exactly and the
        // requested recipe must keep family and unit. Recipe content is proven
        // by the re-validation and re-encode inside RebuildEditedPrefix.
        const auto* boolean=std::get_if<r2::BooleanBaseBinding>(&candidate.base);
        const auto* source=std::get_if<r2::RetainedBooleanBase>(&original->source_);
        const auto* legacy=source?std::get_if<r2::LegacyBooleanBase>(&source->source):nullptr;
        retained_boolean::Program promoted;
        const auto* program=legacy?std::get_if<retained_boolean::Program>(&legacy->prefix):nullptr;
        if(!program&&legacy){const auto*old=std::get_if<retained_boolean::Legacy>(&legacy->prefix);
            if(!old||!retained_boolean::Promote(*old,promoted)){refusal=et::Refusal::MalformedCarrier;return {};}
            program=&promoted;}
        const auto* links=boolean?std::get_if<r2::LegacyPrefixBinding>(&boolean->prefix):nullptr;
        if(!boolean||!program||!links||original->sourceBase_.IsNull()
            ||rebuild->locator.owner!=candidate.owner||rebuild->locator.node!=links->rootNode
            ||rebuild->locator.sourceFeature!=program->source.sourceFeature){refusal=et::Refusal::IdentityMismatch;return {};}
        if(program->source.family==1){
            const auto* p=std::get_if<profile::Parameters>(&rebuild->requested);
            if(!p||p->metersPerUnit!=boolean->metersPerLocalUnit){
                refusal=et::Refusal::UnsupportedBase;return {};
            }
        }else if(program->source.family==2){
            const auto* e=std::get_if<enclosure::Parameters>(&rebuild->requested);
            if(!e||e->metersPerUnit!=boolean->metersPerLocalUnit){
                refusal=et::Refusal::UnsupportedBase;return {};
            }
        }else{
            const auto* loft=std::get_if<rectangular_loft::Definition>(&rebuild->requested);
            if(!loft||program->source.family!=3||loft->dimensionMetersPerUnit!=boolean->metersPerLocalUnit){
                refusal=et::Refusal::UnsupportedBase;return {};
            }
        }
        // C17 capture, main-thread authority side: prove the stored witnesses
        // against the old replay stages and record the selected-use roles so
        // the detached build can rebind witnesses the source edit moves, under
        // the unique-correspondence discipline of et::ApplySourceRebind. The
        // shadow B1 view shares the exact steps/issuance; its byte proof uses
        // the freshly encoded shadow bytes.
        et::Definition shadowDef;shadowDef.schema=2;shadowDef.owner=candidate.owner;
        shadowDef.base.family=program->source.family==1?et::SourceFamily::Profile
            :program->source.family==2?et::SourceFamily::Enclosure:et::SourceFamily::RectangularLoft;
        shadowDef.base.source=boolean->source;shadowDef.base.sourceNode=boolean->sourceNode;
        shadowDef.base.sourceSchema=boolean->sourceSchema;
        shadowDef.base.sourceRecipeDigest=boolean->sourceRecipeDigest;
        shadowDef.base.metersPerLocalUnit=boolean->metersPerLocalUnit;
        shadowDef.issuance=candidate.issuance;shadowDef.outputNode=candidate.outputNode;
        shadowDef.steps=candidate.steps;
        std::vector<std::uint8_t> shadowBytes;
        if(!et::Encode(shadowDef,shadowBytes,refusal))return {};
        auto captured=std::make_shared<et::SourceRebindRoles>();
        et::ReplayBudget rebindBudget;CopyTopologyBudget(rebindBudget,original->chargedBudget_);
        if(!et::CaptureSourceRebindRoles(original->base_,shadowDef,shadowBytes,rebindBudget,refusal,*captured))return {};
        rebindRoles=std::move(captured);
        if(!intentBudget)intentBudget.emplace();
        CopyTopologyBudget(*intentBudget,rebindBudget);
    }
    else if(const auto* tool=std::get_if<r2::RebuildAnalyticTool>(&edit)){
        // The operand must exist exactly in both the captured program and the
        // retained prefix binding, and the request must keep the stable ID.
        const auto* boolean=std::get_if<r2::BooleanBaseBinding>(&candidate.base);
        const auto* source=std::get_if<r2::RetainedBooleanBase>(&original->source_);
        const auto* legacy=source?std::get_if<r2::LegacyBooleanBase>(&source->source):nullptr;
        const auto* program=legacy?std::get_if<retained_boolean::Program>(&legacy->prefix):nullptr;
        const auto* links=boolean?std::get_if<r2::LegacyPrefixBinding>(&boolean->prefix):nullptr;
        const bool known=program&&links&&tool->operandID&&!original->sourceBase_.IsNull()
            &&std::any_of(program->steps.begin(),program->steps.end(),[&](const retained_boolean::Step&s){return s.operand.identifier==tool->operandID;})
            &&std::any_of(links->links.begin(),links->links.end(),[&](const r2::LegacyLink&l){return l.operandID==tool->operandID;});
        if(!known||tool->requested.identifier!=tool->operandID){refusal=et::Refusal::IdentityMismatch;return {};}
    }
    else if(const auto* intent=std::get_if<et::SetSelectorIntent>(&edit)){
        // Typed selector-intent edit: re-prove on the replayed pre-step shape
        // and replace only anchors/receipt. The Boolean prefix is untouched,
        // so the suffix replay in buildEdgeTreatmentR2 re-resolves the new
        // receipt exactly like any other candidate step.
        et::Definition shadow;shadow.schema=2;shadow.owner=candidate.owner;
        if(const auto* old=std::get_if<et::BaseBinding>(&candidate.base))shadow.base=*old;
        else{const auto& boolean=std::get<r2::BooleanBaseBinding>(candidate.base);
            shadow.base.family=et::SourceFamily::Profile;shadow.base.source=boolean.source;
            shadow.base.sourceNode=boolean.sourceNode;shadow.base.sourceSchema=boolean.sourceSchema;
            shadow.base.sourceRecipeDigest=boolean.sourceRecipeDigest;
            shadow.base.metersPerLocalUnit=boolean.metersPerLocalUnit;}
        shadow.issuance=candidate.issuance;shadow.outputNode=candidate.outputNode;shadow.steps=candidate.steps;
        intentBudget.emplace();CopyTopologyBudget(*intentBudget,original->chargedBudget_);
        if(!B1PrepareSelectorIntentEdit(shadow,original->base_,original->dimensionMetersPerUnit(),*intent,*intentBudget,refusal))return {};
        candidate.steps=std::move(shadow.steps);
    }
    else if(std::holds_alternative<r2::SetBooleanOperation>(edit)||std::holds_alternative<r2::SetInputPlacement>(edit)){
        // The A1 composite graph rebuild (operation change or input placement)
        // has no existing detached builder inside this change's file scope;
        // refuse atomically rather than widening scope or faking a lifecycle.
        refusal=et::Refusal::UnsupportedDependency;return {};
    }
    else {refusal=et::Refusal::UnsupportedDependency;return {};}
    std::vector<std::uint8_t> bytes;if(!r2::Encode(candidate,bytes,refusal))return {};
    auto work=std::make_shared<r2::Work>();work->snapshot_=original;work->mutation_=edit;work->candidate_=candidate;
    work->source_=original->source_;work->sourceBase_=original->sourceBase_;work->base_=original->base_;work->originalCurrent_=original->current_;work->nonce_=original->nonce_;
    work->rebindRoles_=std::move(rebindRoles);
    work->chargedBudget_=original->chargedBudget_;if(intentBudget)CopyTopologyBudget(work->chargedBudget_,*intentBudget);work->label_=original->ownerLabel_;work->presentation_=selected;work->state_=r2::Work::State::Prepared;
#if DEBUG
    tb::debug::RecordPhase("prepare-r2",original->chargedBudget_,work->chargedBudget_);
#endif
    refusal=et::Refusal::None;return work;
}

std::shared_ptr<const retained_edge_treatment::r2::DetachedInput> Core3DViewer::edgeTreatmentGeometryR2(
    const std::shared_ptr<retained_edge_treatment::r2::Work>& work) noexcept {
    namespace r2=retained_edge_treatment::r2;if(![NSThread isMainThread]||!work||work->state_!=r2::Work::State::Prepared)return {};
    auto input=std::shared_ptr<r2::DetachedInput>(new r2::DetachedInput);input->nonce_=work->nonce_;input->candidate_=work->candidate_;
    input->source_=work->source_;input->base_=work->base_;input->originalCurrent_=work->originalCurrent_;input->chargedBudget_=work->chargedBudget_;
    input->sourceBase_=work->sourceBase_;input->rebindRoles_=work->rebindRoles_;
    if(const auto* edit=std::get_if<r2::Edit>(&work->mutation_))input->edit_=*edit;
#if DEBUG
    tb::debug::RecordPhase("detach-r2",work->chargedBudget_,input->chargedBudget_);
#endif
    input->cancelled_=work->cancelled_;work->state_=r2::Work::State::Building;return input;
}

// Detached source-base rebuild for the admitted R2 Boolean-input edit; the
// family switch mirrors buildSavedCutSourceDetached and is defined next to
// the other detached source builders below.
static bool R2RebuildBooleanInputBase(const retained_solid::Envelope& envelope,
    const std::atomic_bool& stop, retained_edge_treatment::ReplayBudget& budget,
    TopoDS_Shape& out) noexcept;

std::shared_ptr<const retained_edge_treatment::r2::DetachedResult> Core3DViewer::buildEdgeTreatmentR2(
    const std::shared_ptr<const retained_edge_treatment::r2::DetachedInput>& input,
    retained_edge_treatment::Refusal& refusal) noexcept {
    namespace et=retained_edge_treatment;namespace r2=retained_edge_treatment::r2;refusal=et::Refusal::BuildFailed;
    if(!input||!input->cancelled_||input->cancelled_->load()){refusal=et::Refusal::Cancelled;return {};}
    auto result=std::shared_ptr<r2::DetachedResult>(new r2::DetachedResult);result->nonce_=input->nonce_;result->definition_=input->candidate_;
    result->budget_=input->chargedBudget_;et::ReplayBudget budget;CopyTopologyBudget(budget,input->chargedBudget_);
    result->source_=input->source_;result->base_=input->base_;std::visit([&](const auto& source){if constexpr(std::is_same_v<std::decay_t<decltype(source)>,r2::RetainedBooleanBase>)result->prefixBytes_=std::visit([](const auto& arm){return arm.canonicalPrefixBytes;},source.source);},input->source_);
    if(input->edit_&&(std::holds_alternative<r2::RebuildBooleanInput>(*input->edit_)||std::holds_alternative<r2::RebuildAnalyticTool>(*input->edit_))){
        // Replay the complete changed prefix first; the preserved suffix is
        // replayed on its result below. The candidate binding is re-digested
        // against the exact new prefix bytes, never copied forward.
        r2::RetainedBooleanBase edited;std::vector<std::uint8_t> editedBytes;TopoDS_Shape prefixResult,editedBase;
        const auto* booleanBase=std::get_if<r2::RetainedBooleanBase>(&input->source_);
        if(!booleanBase||!r2::RebuildEditedPrefix(*booleanBase,*input->edit_,input->sourceBase_,*input->cancelled_,edited,editedBytes,prefixResult,editedBase,budget,refusal,R2RebuildBooleanInputBase)){
#if DEBUG
            tb::debug::RecordPhase("build-r2",input->chargedBudget_,budget);
#endif
            return {};}
        result->source_=edited;result->prefixBytes_=editedBytes;result->base_=prefixResult;result->editedSourceBase_=editedBase;result->sourceChanged_=true;
        auto& binding=std::get<r2::BooleanBaseBinding>(result->definition_.base);
        if(!CC_SHA256(editedBytes.data(),CC_LONG(editedBytes.size()),binding.sourceRecipeDigest.data()))return {};
        if(input->rebindRoles_){
            // C18: rebind the witnesses the source edit moved through the
            // captured unique role correspondence and strictly replay the
            // complete suffix on its proper stages; the rebound steps keep
            // feature/node/local IDs, kind, amount and authored intent.
            r2::Definition rebound=result->definition_;TopoDS_Shape treated;
            if(!r2::ApplySourceRebindR2(*input->rebindRoles_,result->base_,rebound,treated,budget,refusal)){
#if DEBUG
                tb::debug::RecordPhase("build-r2",input->chargedBudget_,budget);
#endif
                return {};}
            result->definition_=std::move(rebound);result->result_=treated;
        }
    }
    if(!r2::Encode(result->definition_,result->definitionBytes_,refusal))return {};
    if(result->result_.IsNull()
        &&!r2::ReplayTreatmentSuffix(result->base_,result->definition_,result->result_,budget,refusal)){
#if DEBUG
        tb::debug::RecordPhase("build-r2",input->chargedBudget_,budget);
#endif
        return {};}
    CopyTopologyBudget(result->budget_,budget);
#if DEBUG
    tb::debug::RecordPhase("build-r2",input->chargedBudget_,budget);
#endif
    refusal=et::Refusal::None;return result;
}
bool Core3DViewer::cancelEdgeTreatmentR2(const std::shared_ptr<retained_edge_treatment::r2::Work>& work) noexcept {if(!work)return false;work->cancelled_->store(true);return true;}

retained_edge_treatment::CommitResult Core3DViewer::commitEdgeTreatmentR2(
    const std::shared_ptr<retained_edge_treatment::r2::Work>& work,
    const std::shared_ptr<const retained_edge_treatment::r2::DetachedResult>& result) noexcept {
    namespace et=retained_edge_treatment;namespace r2=retained_edge_treatment::r2;et::CommitResult answer;
    if(![NSThread isMainThread]||!work||!result||result->nonce_!=work->nonce_){answer.refusal=et::Refusal::StaleSnapshot;
#if DEBUG
        tb::debug::RecordCommit(int(answer.outcome),et::RefusalCode(answer.refusal),false);
#endif
        return answer;}
    OrdinaryTransformChange change;change.label=work->label_;change.presentation=work->presentation_;change.shape=result->result_;
    change.transform=work->presentation_.IsNull()?gp_Trsf():work->presentation_->LocalTransformation();change.operation=OrdinaryTransformOperation::RetainedEdgeTreatment;
    change.edgeTreatmentSnapshotR2=work->snapshot_;change.edgeTreatmentMigrationR2=work->migration_;if(work->migration_)change.edgeTreatmentMigrationRequestR2=std::get<r2::MigrationM3>(work->mutation_);change.edgeTreatmentEnrollmentR2=work->enrollment_;change.edgeTreatmentResultR2=result;
    OrdinaryEditResult ordinary=OrdinaryEditResult::Invalid;auto lease=_ordinaryEditController->beginTransform({change},&ordinary);if(lease)ordinary=lease.stageAndCommit();
    if(ordinary==OrdinaryEditResult::Committed){answer.outcome=et::CommitOutcome::Committed;answer.refusal=et::Refusal::None;answer.measuredUndoDelta=1;}
    else if(ordinary==OrdinaryEditResult::OutcomeUnknown){answer.outcome=et::CommitOutcome::OutcomeUnknown;answer.refusal=et::Refusal::OutcomeUnknown;answer.measuredUndoDelta.reset();}
    else{answer.outcome=et::CommitOutcome::Refused;answer.refusal=ordinary==OrdinaryEditResult::BudgetRefused?et::Refusal::Budget:et::Refusal::StageFailed;answer.measuredUndoDelta=0;}
#if DEBUG
    tb::debug::RecordCommit(int(answer.outcome),et::RefusalCode(answer.refusal),answer.outcome==et::CommitOutcome::OutcomeUnknown);
#endif
    return answer;
}
namespace native_opening::detail {
struct State final {
    Core3DViewer* owner = nullptr;
    std::uint64_t nextContext = 0;
    std::uint64_t activeContext = 0;
    std::uint64_t nextMarker = 0;
    std::uint64_t activeMarker = 0;
    std::uint64_t recoveryMarker = 0;
    std::uint64_t recoveryContext = 0;
    // A committed or outcome-unknown native edit whose affected presentations
    // are not yet proven against the document. Exact recovery reconciles this
    // same plan before the shared fence can clear.
    std::shared_ptr<const CommittedEditPublication> pendingPublication;
    std::uint64_t publicationContext = 0;
#if DEBUG
    // Row-265 lifecycle probe: real closes at this viewer's lease boundary
    // that must be reported unproven after the real CommitCommand has run.
    std::uint32_t debugUnprovenCloseResults = 0;
#endif
};
} // namespace native_opening::detail

namespace {
bool SameElement(const scene::ElementIdentifier& a,
                 const scene::ElementIdentifier& b) noexcept {
    return a.entityIdentifier == b.entityIdentifier && a.kind == b.kind
        && a.topologyIndex == b.topologyIndex
        && a.geometryRevision == b.geometryRevision;
}
}

namespace native_opening {
bool SameFence(const Fence& a, const Fence& b) noexcept {
    if (!(a.stamp() == b.stamp())
        || a.publicationSourceIdentifier() != b.publicationSourceIdentifier()
        || a.documentGeneration() != b.documentGeneration()
        || a.modelRevision() != b.modelRevision()
        || a.selectionMode() != b.selectionMode()
        || a.document() != b.document() || a.data() != b.data()
        || a.metersPerUnit() != b.metersPerUnit()
        || a.sourceReceipts() != b.sourceReceipts()
        || a.selection().size() != b.selection().size()
        || a.hovered().has_value() != b.hovered().has_value()) return false;
    for (std::size_t i = 0; i < a.selection().size(); ++i)
        if (!SameElement(a.selection()[i], b.selection()[i])) return false;
    return !a.hovered() || SameElement(*a.hovered(), *b.hovered());
}

Context::Context(std::weak_ptr<detail::State> state, std::uint64_t identifier,
                 Fence fence) noexcept
    : state_(std::move(state)), identifier_(identifier),
      openingFence_(std::move(fence)) {}

Context::~Context() {
    if (const auto state = state_.lock(); state && state->activeContext == identifier_
        && state->activeMarker == 0 && state->recoveryMarker == 0) {
        state->activeContext = 0;
    }
}

std::shared_ptr<const Fence> Context::recapture(
    std::uint32_t width, std::uint32_t height) const noexcept {
    const auto state = state_.lock();
    if (!state || state->owner == nullptr || state->activeContext != identifier_)
        return {};
    const auto current = state->owner->recaptureNativeOpeningFence(
        identifier_, width, height, openingFence_.sourceReceipts());
    if (!current || !(current->stamp() == openingFence_.stamp())) return {};
    return current;
}

bool Context::isCurrent(std::uint32_t width, std::uint32_t height) const noexcept {
    const auto current = recapture(width, height);
    return current && SameFence(openingFence_, *current);
}

bool Context::ownsMarker(std::uint64_t marker) const noexcept {
    const auto state = state_.lock();
    return state && state->activeContext == identifier_
        && state->activeMarker == marker && state->recoveryMarker == 0;
}

void Context::finishMarker(std::uint64_t marker, bool recovery) noexcept {
    const auto state = state_.lock();
    if (!state || state->activeContext != identifier_ || state->activeMarker != marker)
        return;
    state->activeMarker = 0;
    if (recovery) {
        state->recoveryMarker = marker;
        state->recoveryContext = identifier_;
    }
}

std::shared_ptr<CommandLease> Context::beginCommandLease(
    const Fence& expected, std::uint32_t width, std::uint32_t height) noexcept {
    const auto state = state_.lock();
    const auto fresh = recapture(width, height);
    if (!state || !fresh || !SameFence(expected, *fresh)
        || !SameFence(openingFence_, expected) || state->activeMarker != 0
        || state->recoveryMarker != 0 || state->nextMarker == UINT64_MAX)
        return {};
    const auto document = expected.document();
    const auto data = expected.data();
    if (document.IsNull() || data.IsNull() || document->GetData() != data
        || document->HasOpenCommand()) return {};
    const std::uint64_t marker = ++state->nextMarker;
    state->activeMarker = marker;
    try {
        document->NewCommand();
        if (!document->HasOpenCommand() || data->Transaction() <= 0) {
            finishMarker(marker, true);
            return {};
        }
        return std::shared_ptr<CommandLease>(new CommandLease(
            shared_from_this(), document, data, data->Transaction(), marker));
    } catch (...) {
        finishMarker(marker, true);
        return {};
    }
}

std::shared_ptr<CommandLease> Context::borrowCommandLease(
    OrdinaryEditCommandStamp& ordinary) noexcept {
    const auto state = state_.lock();
    const auto document = openingFence_.document();
    const auto data = openingFence_.data();
    if (!state || state->activeContext != identifier_
        || state->activeMarker != 0 || state->recoveryMarker != 0
        || state->nextMarker == UINT64_MAX || document.IsNull() || data.IsNull()
        || document->GetData() != data) return {};
    int transaction = 0;
    if (!ordinary.lendNativeCommand(document, data, transaction)) return {};
    const std::uint64_t marker = ++state->nextMarker;
    state->activeMarker = marker;
    auto lease = std::shared_ptr<CommandLease>(new CommandLease(
        shared_from_this(), document, data, transaction, marker, true));
    lease->ordinary_ = &ordinary;
    return lease;
}

bool Context::reconcileRecovery(bool exactStateKnown) noexcept {
    const auto state = state_.lock();
    if (!state || !exactStateKnown || state->activeContext != identifier_
        || state->recoveryContext != identifier_ || state->recoveryMarker == 0
        || state->activeMarker != 0 || state->owner == nullptr) return false;
    const auto document = openingFence_.document();
    if (document.IsNull() || document->HasOpenCommand()
        || document->GetData() != openingFence_.data()) return false;
    // Exact recovery first proves the actual committed/aborted document state
    // for the retained plan and refreshes the corresponding real
    // presentations; only then may the shared fence clear and a new context
    // be permitted. A failed reconciliation keeps the fence armed.
    if (state->pendingPublication) {
        if (state->publicationContext != identifier_
            || !state->owner->reconcileNativeOpeningEdit(
                openingFence_, *state->pendingPublication)) return false;
        state->pendingPublication.reset();
        state->publicationContext = 0;
    }
    state->recoveryMarker = 0;
    state->recoveryContext = 0;
    // Exact recovery is terminal for this authority. Old opening references
    // may outlive recovery, but may neither block nor acquire a new command.
    // Every identity, close, and publication proof above has already passed.
    state->activeContext = 0;
    return true;
}

bool Context::publishCommittedEdit(
    const CommittedEditPublication& publication) noexcept {
    const auto state = state_.lock();
    if (!state || state->owner == nullptr || state->activeContext != identifier_
        || state->activeMarker != 0 || state->recoveryMarker != 0) return false;
    try {
        if (state->owner->reconcileNativeOpeningEdit(openingFence_, publication))
            return true;
        // A proven commit without a proven publication is never reported as a
        // pre-command rejection and never retried as another edit: arm the
        // shared recovery fence and retain the same plan for exact recovery.
        state->pendingPublication =
            std::make_shared<const CommittedEditPublication>(publication);
        state->publicationContext = identifier_;
    } catch (...) {}
    // Even if retention itself failed, the unproven committed presentation
    // state must keep the shared fence armed.
    state->recoveryMarker = state->nextMarker != 0 ? state->nextMarker : 1;
    state->recoveryContext = identifier_;
    return false;
}

void Context::retainUnprovenEdit(
    const CommittedEditPublication& publication) noexcept {
    try {
        const auto state = state_.lock();
        if (!state || state->activeContext != identifier_
            || state->recoveryMarker == 0 || state->recoveryContext != identifier_
            || state->pendingPublication) return;
        state->pendingPublication =
            std::make_shared<const CommittedEditPublication>(publication);
        state->publicationContext = identifier_;
    } catch (...) {}
}

#if DEBUG
void Context::debugReportNextCloseUnproven() noexcept {
    const auto state = state_.lock();
    if (!state || state->activeContext != identifier_
        || state->activeMarker != 0 || state->recoveryMarker != 0) return;
    ++state->debugUnprovenCloseResults;
}
#endif

CommandLease::CommandLease(std::shared_ptr<Context> context,
    const Handle(TDocStd_Document)& document, const Handle(TDF_Data)& data,
    int transaction, std::uint64_t marker, bool borrowed) noexcept
    : context_(std::move(context)), document_(document), data_(data),
      transaction_(transaction), marker_(marker), borrowed_(borrowed) {}

CommandLease::~CommandLease() {
    if (!settled_) {
        if (borrowed_) {
            if (context_) context_->finishMarker(marker_, false);
            settled_ = true;
        } else retainRecovery();
    }
}

bool CommandLease::ownsOpenCommand() const noexcept {
    try {
        int ordinaryTransaction = 0;
        return !settled_ && context_ && context_->ownsMarker(marker_)
            && !document_.IsNull() && !data_.IsNull()
            && document_->GetData() == data_ && document_->HasOpenCommand()
            && data_->Transaction() == transaction_
            && (!borrowed_ || (ordinary_
                && ordinary_->lendNativeCommand(document_, data_, ordinaryTransaction)
                && ordinaryTransaction == transaction_));
    } catch (...) { return false; }
}

void CommandLease::retainRecovery() noexcept {
    if (context_) context_->finishMarker(marker_, true);
    settled_ = true;
}

bool CommandLease::commit() noexcept {
    if (borrowed_) return false;
    if (!ownsOpenCommand()) { retainRecovery(); return false; }
    try {
        if (!document_->CommitCommand() || document_->HasOpenCommand()) {
            retainRecovery(); return false;
        }
#if DEBUG
        // Row-265 lifecycle probe seam: the real close above ran; the lease
        // result itself is reported unproven exactly at this boundary, so the
        // caller maps a genuine outcome-unknown and the shared recovery fence
        // is armed through the same retainRecovery path as a real failure.
        if (const auto state = context_ ? context_->state_.lock() : nullptr) {
            if (state->debugUnprovenCloseResults > 0) {
                --state->debugUnprovenCloseResults;
                retainRecovery();
                return false;
            }
        }
#endif
        context_->finishMarker(marker_, false);
        settled_ = true;
        return true;
    } catch (...) { retainRecovery(); return false; }
}

bool CommandLease::abort() noexcept {
    if (borrowed_) return false;
    if (!ownsOpenCommand()) { retainRecovery(); return false; }
    try {
        document_->AbortCommand();
        if (document_->HasOpenCommand()) { retainRecovery(); return false; }
        context_->finishMarker(marker_, false);
        settled_ = true;
        return true;
    } catch (...) { retainRecovery(); return false; }
}
} // namespace native_opening

bool Core3DViewer::reconcileNativeOpeningEdit(
    const native_opening::Fence& fence,
    const native_opening::CommittedEditPublication& publication) noexcept {
    if (![NSThread isMainThread]) return false;
    try {
        const auto traceRefusal = [](const char* predicate, bool failed) noexcept {
#if DEBUG
            if (failed) std::fprintf(
                stderr, "R179_PUBLISH predicate=%s failed=1\n", predicate);
#else
            (void)predicate;
#endif
            return failed;
        };
        if (traceRefusal("owner-or-context", myDoc.IsNull() || myContext.IsNull()))
            return false;
        const Handle(TDocStd_Document) document = myDoc->Document();
        if (traceRefusal("document-identity",
                document.IsNull() || fence.document() != document
                || fence.data().IsNull() || document->GetData() != fence.data())
            || traceRefusal("document.open-command", document->HasOpenCommand()))
            return false;
        const std::size_t affected = publication.created.size()
            + publication.replaced.size() + publication.removed.size();
        if (traceRefusal("affected-bounds", affected == 0 || affected > 4096))
            return false;
        std::set<std::string> identities;
        const auto collect = [&](const std::vector<native_opening::CommittedEditItem>&
                                 items) {
            for (const auto& item : items)
                if (item.entityIdentifier.empty() || item.entityIdentifier.size() > 128
                    || item.entityIdentifier.find('\0') != std::string::npos
                    || !identities.insert(item.entityIdentifier).second) return false;
            return true;
        };
        if (traceRefusal("identity-syntax",
                !collect(publication.created) || !collect(publication.replaced)
                || !collect(publication.removed))) return false;
        // Document authority: persistent identity -> live free label.
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (traceRefusal("shape-tool", shapes.IsNull())) return false;
        TDF_LabelSequence roots;
        shapes->GetFreeShapes(roots);
        if (traceRefusal("free-shape-bounds",
                roots.Length() < 0 || roots.Length() > 50000)) return false;
        std::map<std::string, TDF_Label> live;
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            const TDF_Label label = roots.Value(index);
            if (label.IsNull() || label.Data() != document->GetData()) {
                (void)traceRefusal("free-shape-label", true); return false;
            }
            const std::string entity = myDoc->EntityIdentifierForLabel(label);
            if (!entity.empty() && !live.emplace(entity, label).second) {
                (void)traceRefusal("duplicate-live-identity", true); return false;
            }
        }
        // Renderer authority: persistent identity -> displayed presentation.
        // Presentations without a resolvable document label are stale
        // candidates, matched to replacements/removals by exact previous shape.
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        if (traceRefusal("displayed-bounds", objects.Extent() > 50000))
            return false;
        std::map<std::string, Handle(AIS_Shape)> displayed;
        std::vector<Handle(AIS_Shape)> stale;
        for (AIS_ListIteratorOfListOfInteractive iterator(objects);
             iterator.More(); iterator.Next()) {
            const auto presentation =
                Handle(AIS_Shape)::DownCast(iterator.Value());
            if (presentation.IsNull()) continue;
            const TDF_Label label = myDoc->ShapeLabel(presentation);
            if (label.IsNull()) { stale.push_back(presentation); continue; }
            const std::string entity = myDoc->EntityIdentifierForLabel(label);
            if (entity.empty() || !displayed.emplace(entity, presentation).second) {
                (void)traceRefusal("displayed-identity", true); return false;
            }
        }
        const auto reconcileItem = [&](
                const native_opening::CommittedEditItem& item) {
            const auto liveLabel = live.find(item.entityIdentifier);
            const auto current = displayed.find(item.entityIdentifier);
            if (liveLabel != live.end()) {
                const TDF_Label label = liveLabel->second;
                const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(label);
                OcctObjectTransformState state;
                if (shape.IsNull()
                    || !myDoc->CaptureObjectTransformStateForLabel(label, state)
                    || state.entityIdentifier != item.entityIdentifier) {
                    (void)traceRefusal("committed-state", true); return false;
                }
                const bool created = current == displayed.end();
                Handle(AIS_Shape) presentation;
                if (!created) {
                    presentation = current->second;
                    const bool wasSelected = myContext->IsSelected(presentation);
                    // Redisplay/RecomputeSelectionOnly rebuild the selection
                    // primitives, so an owner selected now is stale afterwards
                    // but remains held by the context selection. Remove this
                    // presentation's selected owner(s) while they are still
                    // valid, recompute, then select the new global owner
                    // exactly once iff the presentation was selected.
                    std::size_t selectedBefore = 0;
                    std::vector<Handle(SelectMgr_EntityOwner)> staleOwners;
                    for (myContext->InitSelected(); myContext->MoreSelected();
                         myContext->NextSelected()) {
                        if (selectedBefore >= 50000) {
                            (void)traceRefusal("selection-owner", true);
                            return false;
                        }
                        ++selectedBefore;
                        const auto selectedOwner = myContext->SelectedOwner();
                        if (!selectedOwner.IsNull()
                            && selectedOwner->Selectable() == presentation) {
                            staleOwners.push_back(selectedOwner);
                        }
                    }
                    for (const auto& staleOwner : staleOwners) {
                        myContext->AddOrRemoveSelected(staleOwner, Standard_False);
                    }
                    presentation->SetShape(shape);
                    presentation->SetLocalTransformation(state.transform);
                    myContext->Redisplay(presentation, Standard_False);
                    myContext->RecomputeSelectionOnly(presentation);
                    if (wasSelected) {
                        const auto owner = presentation->GlobalSelOwner();
                        if (owner.IsNull() || owner->Selectable() != presentation) {
                            (void)traceRefusal("selection-owner", true);
                            return false;
                        }
                        if (!myContext->IsSelected(owner)) {
                            myContext->AddOrRemoveSelected(owner, Standard_False);
                        }
                    }
                    // Postcondition: the same number of selected owners as
                    // before, exactly one of them for this presentation iff it
                    // was selected, and IsSelected agreeing with wasSelected.
                    std::size_t selectedAfter = 0;
                    std::size_t ownersForPresentation = 0;
                    for (myContext->InitSelected(); myContext->MoreSelected();
                         myContext->NextSelected()) {
                        if (selectedAfter >= 50000) {
                            (void)traceRefusal("selection-owner", true);
                            return false;
                        }
                        ++selectedAfter;
                        const auto selectedOwner = myContext->SelectedOwner();
                        if ((!selectedOwner.IsNull()
                                && selectedOwner->Selectable() == presentation)
                            || myContext->SelectedInteractive() == presentation) {
                            ++ownersForPresentation;
                        }
                    }
                    if (selectedAfter != selectedBefore
                        || ownersForPresentation
                            != (wasSelected ? std::size_t(1) : std::size_t(0))
                        || myContext->IsSelected(presentation) != wasSelected) {
                        (void)traceRefusal("selection-owner", true);
                        return false;
                    }
                } else {
                    // SetShape can retire the label of a plain AIS_Shape.
                    // Only the stager's exact previous shape may identify it.
                    auto previous = stale.end();
                    if (!item.previousShape.IsNull()) {
                        for (auto candidate = stale.begin();
                             candidate != stale.end(); ++candidate) {
                            if (!(*candidate)->Shape().IsEqual(item.previousShape))
                                continue;
                            if (previous != stale.end()) {
                                (void)traceRefusal("replacement-stale-ambiguous", true);
                                return false;
                            }
                            previous = candidate;
                        }
                    }
                    const bool wasSelected = previous != stale.end()
                        && myContext->IsSelected(*previous);
                    presentation = new AIS_Shape(shape);
                    presentation->SetLocalTransformation(state.transform);
                    myDoc->LoadObjectMeterial(label, presentation);
                    myContext->Display(
                        presentation, AIS_Shaded, 0, Standard_False);
                    if (!myContext->IsDisplayed(presentation)) {
                        (void)traceRefusal("display-created", true); return false;
                    }
                    if (!myDoc->ShapeLabel(presentation).IsEqual(label)) {
                        (void)traceRefusal("created-label", true); return false;
                    }
                    const auto owner = presentation->GlobalSelOwner();
                    if (wasSelected
                        && (owner.IsNull() || owner->Selectable() != presentation)) {
                        (void)traceRefusal("replacement-selection-owner", true);
                        return false;
                    }
                    if (previous != stale.end()) {
                        if (wasSelected)
                            myContext->AddOrRemoveSelected(*previous, Standard_False);
                        if (myContext->IsSelected(*previous)) {
                            (void)traceRefusal("deselect-replaced-stale", true);
                            return false;
                        }
                        myContext->Remove(*previous, Standard_False);
                        if (myContext->IsDisplayed(*previous)
                            || myContext->IsSelected(*previous)) {
                            (void)traceRefusal("remove-replaced-stale", true);
                            return false;
                        }
                        stale.erase(previous);
                    }
                    if (wasSelected) {
                        if (!myContext->IsSelected(presentation))
                            myContext->AddOrRemoveSelected(owner, Standard_False);
                        if (!myContext->IsSelected(presentation)) {
                            (void)traceRefusal("select-replacement", true);
                            return false;
                        }
                    }
                    displayed.emplace(item.entityIdentifier, presentation);
                }
#if DEBUG
                unsigned faces = 0, unmeshed = 0;
                for (TopExp_Explorer face(shape, TopAbs_FACE);
                     face.More(); face.Next()) {
                    ++faces;
                    TopLoc_Location location;
                    if (BRep_Tool::Triangulation(
                            TopoDS::Face(face.Current()), location).IsNull())
                        ++unmeshed;
                }
                std::fprintf(stderr,
                    "R179_PUBLISH entity=%.128s created=%d faces=%u unmeshed=%u\n",
                    item.entityIdentifier.c_str(), int(created), faces, unmeshed);
#endif
                return true;
            }
            // The committed document holds no such label: only a stale
            // presentation may remain, recognized by its exact previous
            // shape. Absence is consistent (aborted or never displayed).
            if (current != displayed.end()) {
                (void)traceRefusal("removal-contradiction", true); return false;
            }
            if (item.previousShape.IsNull()) return true;
            for (auto candidate = stale.begin(); candidate != stale.end();
                 ++candidate) {
                if (!(*candidate)->Shape().IsEqual(item.previousShape)) continue;
                if (myContext->IsSelected(*candidate))
                    myContext->AddOrRemoveSelected(*candidate, Standard_False);
                myContext->Remove(*candidate, Standard_False);
                if (myContext->IsDisplayed(*candidate)) {
                    (void)traceRefusal("remove-stale", true); return false;
                }
                stale.erase(candidate);
                return true;
            }
            return true;
        };
        for (const auto& item : publication.replaced)
            if (!reconcileItem(item)) return false;
        for (const auto& item : publication.created)
            if (!reconcileItem(item)) return false;
        for (const auto& item : publication.removed)
            if (!reconcileItem(item)) return false;
        myContext->UpdateCurrentViewer();
        const auto snapshot = captureSceneSnapshot(64, 64);
        return !traceRefusal("snapshot-proof",
            !snapshot || snapshot->publicationSourceIdentifier.empty());
    } catch (...) { return false; }
}

// This is synchronous and bounded; no UI session or network wait is admitted.
OrdinaryEditResult Core3DViewer::repairMeshWinding(
    const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
    std::uint32_t width, std::uint32_t height) noexcept {
    if (![NSThread isMainThread]) return OrdinaryEditResult::Invalid;
    if (!canBeginCommittedEdit() || !_ordinaryEditController) return OrdinaryEditResult::Busy;
    if (myDoc.IsNull() || myContext.IsNull() || width == 0 || height == 0
        || identity.entityIdentifier.empty() || identity.entityIdentifier.size() > 128
        || identity.entityIdentifier.find('\0') != std::string::npos
        || identity.publicationSourceIdentifier.empty() || identity.publicationSourceIdentifier.size() > 128
        || identity.publicationSourceIdentifier.find('\0') != std::string::npos)
        return OrdinaryEditResult::Invalid;
    try {
        const auto owner = myDoc;
        const auto document = owner->Document();
        if (document.IsNull() || document->HasOpenCommand()) return OrdinaryEditResult::Busy;
        const auto documentTime = document->GetData()->Time();
        const auto snapshot = captureSceneSnapshot(width, height);
        if (!snapshot || snapshot->publicationSourceIdentifier != identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration != identity.documentGeneration
            || snapshot->revisions.model != identity.modelRevision
            || snapshot->revisions.presentation != presentationRevision
            || snapshot->selectionMode != scene::ElementKind::Object
            || snapshot->selection.selected.size() != 1
            || snapshot->selection.selected[0].kind != scene::ElementKind::Object
            || snapshot->selection.selected[0].entityIdentifier != identity.entityIdentifier)
            return OrdinaryEditResult::Invalid;
        myContext->InitSelected();
        if (!myContext->MoreSelected()) return OrdinaryEditResult::Invalid;
        const auto presentation = Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        myContext->NextSelected();
        if (myContext->MoreSelected() || presentation.IsNull()) return OrdinaryEditResult::Invalid;
        const auto label = owner->ShapeLabel(presentation);
        OrdinaryTransformRecord record;
        if (!owner->CaptureObjectTransformStateForLabel(label, record.previous)
            || record.previous.entityIdentifier != identity.entityIdentifier
            || record.previous.resolvedRepresentation != OcctGeometryRepresentation::TriangleMesh
            || record.previous.authoredFramesPresent) return OrdinaryEditResult::Invalid;
        record.requested.label = label; record.requested.presentation = presentation;
        record.requested.shape = record.previous.shape;
        record.requested.transform = record.previous.transform;
        OrdinaryTransformLedger authority; authority.records.push_back(record);
        if (!admitTransform(authority)) return OrdinaryEditResult::Invalid;
        TopoDS_Shape candidate;
        const auto result = owner->PrepareMeshWindingRepair(label, candidate);
        if (result == OcctMeshWindingRepairResult::Invalid) return OrdinaryEditResult::Invalid;
        OcctObjectTransformState actual;
        auto freshAuthority = authority;
        if (myDoc != owner || owner->Document() != document
            || document->GetData()->Time() != documentTime
            || !owner->CaptureObjectTransformStateForLabel(label, actual)
            || !record.previous.IsEqual(actual)
            || !admitTransform(freshAuthority)
            || freshAuthority.selectionOwners != authority.selectionOwners
            || freshAuthority.manipulatorType != authority.manipulatorType
            || freshAuthority.hadManipulator != authority.hadManipulator)
            return OrdinaryEditResult::Invalid;
        if (result == OcctMeshWindingRepairResult::Unchanged)
            return candidate.IsNull() ? OrdinaryEditResult::NoChange : OrdinaryEditResult::Invalid;
        if (result != OcctMeshWindingRepairResult::Prepared || candidate.IsNull())
            return OrdinaryEditResult::Invalid;
        auto request = record.requested;
        request.shape = candidate;
        request.operation = OrdinaryTransformOperation::MeshWindingRepair;
        OrdinaryEditResult failure = OrdinaryEditResult::Invalid;
        auto lease = _ordinaryEditController->beginTransform({request}, &failure);
        return lease ? lease.stageAndCommit() : failure;
    } catch (...) { return OrdinaryEditResult::Invalid; }
}



namespace {

bool TryManipulatorForBooleanAction(
    const BooleanAction theAction,
    PrimitiveManipulatorType& theType) noexcept
{
    switch (theAction) {
        case BooleanAction::BooleanSubtract:
            theType = PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract;
            return true;
        case BooleanAction::BooleanUnion:
            theType = PrimitiveManipulatorType::PrimitiveGizmoTypeUnion;
            return true;
        case BooleanAction::BooleanIntersect:
            theType = PrimitiveManipulatorType::PrimitiveGizmoTypeIntersect;
            return true;
    }
    return false;
}

scene::PresentationOverlayKind BooleanOverlayKind(
    const BooleanAction theAction) noexcept
{
    switch (theAction) {
        case BooleanAction::BooleanSubtract:
            return scene::PresentationOverlayKind::BooleanSubtractPreview;
        case BooleanAction::BooleanUnion:
            return scene::PresentationOverlayKind::BooleanUnionPreview;
        case BooleanAction::BooleanIntersect:
            return scene::PresentationOverlayKind::BooleanIntersectPreview;
    }
    return scene::PresentationOverlayKind::None;
}

#ifdef DEBUG
std::atomic<Standard_Size> gBoundedProjectTopologyValidationCount{0};
std::atomic<Standard_Size> gGeometricBRepValidationCount{0};
#endif

bool RunGeometricBRepValidation(const TopoDS_Shape& shape) {
#ifdef DEBUG
    gGeometricBRepValidationCount.fetch_add(1, std::memory_order_relaxed);
#endif
    BRepCheck_Analyzer analyzer(shape, Standard_True);
    return analyzer.IsValid();
}

bool IsTopologicallyValid(const TopoDS_Shape& shape) {
    if (shape.IsNull()) {
        return false;
    }
    try {
        return RunGeometricBRepValidation(shape);
    } catch (...) {
        return false;
    }
}

bool TryCountDisplayedModelShapes(
    const Handle(Core3DContext)& context,
    Standard_Size& count) noexcept {
    count = 0;
    if (context.IsNull()) {
        return false;
    }

    try {
        OCC_CATCH_SIGNALS
        AIS_ListOfInteractive displayedShapes;
        context->DisplayedObjects(AIS_KOI_Shape, -1, displayedShapes);
        for (AIS_ListIteratorOfListOfInteractive displayed(displayedShapes);
             displayed.More(); displayed.Next()) {
            const Handle(AIS_Shape) modelShape =
                Handle(AIS_Shape)::DownCast(displayed.Value());
            if (!modelShape.IsNull() && !modelShape->Shape().IsNull()) {
                ++count;
            }
        }
        return true;
    } catch (...) {
        // If the graphics context cannot provide a trustworthy count, preserve
        // the user's camera instead of risking an unexpected reframe.
        count = 0;
        return false;
    }
}

struct SelectionPresentationModesSnapshot {
    Handle(AIS_InteractiveObject) presentation;
    std::vector<Standard_Integer> activeModes;
};

struct SelectionContextModesSnapshot {
    std::vector<SelectionPresentationModesSnapshot> presentations;
    // OCCT represents its adaptive/default picker tolerance with a negative
    // raw custom value. Keep that representation exact: SetPixelTolerance(2)
    // is allowed to return early when the effective tolerance is already 2,
    // leaving CustomPixelTolerance() at -1.
    Standard_Integer pixelTolerance = -1;
};

bool CaptureSelectionContextModes(
    const Handle(Core3DContext)& theContext,
    SelectionContextModesSnapshot& theSnapshot) noexcept
{
    theSnapshot = {};
    if (theContext.IsNull()) {
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
        AIS_ListOfInteractive displayedObjects;
        theContext->DisplayedObjects(displayedObjects);
        theSnapshot.presentations.reserve(
            static_cast<std::size_t>(displayedObjects.Size()));
        for (AIS_ListIteratorOfListOfInteractive anIterator(displayedObjects);
             anIterator.More(); anIterator.Next()) {
            SelectionPresentationModesSnapshot presentationSnapshot;
            presentationSnapshot.presentation = anIterator.Value();
            TColStd_ListOfInteger activeModes;
            theContext->ActivatedModes(
                presentationSnapshot.presentation, activeModes);
            presentationSnapshot.activeModes.reserve(
                static_cast<std::size_t>(activeModes.Extent()));
            for (TColStd_ListIteratorOfListOfInteger mode(activeModes);
                 mode.More(); mode.Next()) {
                presentationSnapshot.activeModes.push_back(mode.Value());
            }
            theSnapshot.presentations.push_back(
                std::move(presentationSnapshot));
        }
        const Handle(StdSelect_ViewerSelector3d)& selector =
            theContext->MainSelector();
        if (selector.IsNull()) {
            theSnapshot = {};
            return false;
        }
        const Standard_Integer rawPixelTolerance =
            selector->CustomPixelTolerance();
        theSnapshot.pixelTolerance = rawPixelTolerance < 0
            ? -1 : rawPixelTolerance;
        return true;
    } catch (...) {
        theSnapshot = {};
        return false;
    }
}

bool RestoreSelectionContextModes(
    const Handle(Core3DContext)& theContext,
    const SelectionContextModesSnapshot& theSnapshot) noexcept
{
    if (theContext.IsNull()) {
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
        for (const SelectionPresentationModesSnapshot& snapshot :
             theSnapshot.presentations) {
            if (snapshot.presentation.IsNull()) {
                return false;
            }
            theContext->Deactivate(snapshot.presentation);
            for (const Standard_Integer activeMode : snapshot.activeModes) {
                theContext->SetSelectionModeActive(
                    snapshot.presentation,
                    activeMode,
                    Standard_True,
                    AIS_SelectionModesConcurrency_Multiple,
                    Standard_True);
            }

            TColStd_ListOfInteger restoredModes;
            theContext->ActivatedModes(
                snapshot.presentation, restoredModes);
            std::vector<Standard_Integer> actualModes;
            actualModes.reserve(
                static_cast<std::size_t>(restoredModes.Extent()));
            for (TColStd_ListIteratorOfListOfInteger mode(restoredModes);
                 mode.More(); mode.Next()) {
                actualModes.push_back(mode.Value());
            }
            std::vector<Standard_Integer> expectedModes =
                snapshot.activeModes;
            std::sort(actualModes.begin(), actualModes.end());
            std::sort(expectedModes.begin(), expectedModes.end());
            if (actualModes != expectedModes) {
                return false;
            }
        }
        theContext->SetPixelTolerance(theSnapshot.pixelTolerance);
        const Handle(StdSelect_ViewerSelector3d)& selector =
            theContext->MainSelector();
        return !selector.IsNull()
            && selector->CustomPixelTolerance()
                == theSnapshot.pixelTolerance;
    } catch (...) {
        return false;
    }
}

bool IsStatelessManipulatorType(
    const PrimitiveManipulatorType theType) noexcept
{
    switch (theType) {
        case PrimitiveManipulatorType::PrimitiveGizmoTypeNone:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial:
            return true;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeScale:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeUnion:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMirror:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeExtrude:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeIntersect:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeShell:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray:
            return false;
    }
    return false;
}

PrimitiveManipulatorType StatelessManipulatorTypeOrNone(
    const PrimitiveManipulatorType theType) noexcept
{
    return IsStatelessManipulatorType(theType)
        ? theType
        : PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
}

PrimitiveManipulatorType InspectorRestorableManipulatorTypeOrNone(
    const PrimitiveManipulatorType theType) noexcept
{
    // Scale has no operation ledger, but unlike the passive stateless tools it
    // is valid only after the Position fallback restores its exact source
    // selection. Never pass it through empty-selection interactor recreation.
    return theType
            == PrimitiveManipulatorType::PrimitiveGizmoTypeScale
        ? theType
        : StatelessManipulatorTypeOrNone(theType);
}

bool HasActiveOperationLedger(
    const std::shared_ptr<ObjectInteractor>& theObjectInteractor,
    const std::shared_ptr<ShapeInteractor>& theShapeInteractor) noexcept
{
    try {
        if (theObjectInteractor != nullptr
            && (theObjectInteractor->hasUnresolvedDuplicate()
                || theObjectInteractor->isPickingMirrorPlane()
                || theObjectInteractor->hasActiveBoolean()
                || theObjectInteractor->hasUnresolvedBoolean()
                || theObjectInteractor->hasActiveMirror()
                || theObjectInteractor->hasUnresolvedMirrorObjects()
                || theObjectInteractor->hasActiveLinearArray()
                || theObjectInteractor->hasUnresolvedLinearArray()
                || theObjectInteractor->hasActiveRadialArray()
                || theObjectInteractor->hasUnresolvedRadialArray())) {
            return true;
        }
        return theShapeInteractor != nullptr
            && (theShapeInteractor->hasActiveBevel()
                || theShapeInteractor->hasActiveExtrusion()
                || theShapeInteractor->hasActiveShell()
                || theShapeInteractor->hasUnresolvedShell());
    } catch (...) {
        // If an operation controller cannot prove that it is idle, replacing
        // its document or interactors would destroy its only recovery ledger.
        return true;
    }
}

constexpr NSUInteger kMaximumPrimitiveCount = 1024;
constexpr std::uint64_t kMaximumProjectDocumentBytes =
    256ull * 1024ull * 1024ull;
constexpr Standard_Integer kMaximumVisualMaterialDefinitions = 2048;
constexpr Standard_Size kMaximumAggregateTextureBytes =
    128ull * 1024ull * 1024ull;
constexpr Standard_Real kMaximumEmissionFactor = 65504.0;
constexpr Standard_Size kMaximumAssemblyTraversalDepth = 128;
constexpr Standard_Size kMaximumAssemblyTraversalNodes = 32768;
constexpr Standard_Size kMaximumShapeDefinitions = 4096;
constexpr Standard_Size kMaximumSubshapesPerDefinition = 250000;
constexpr Standard_Size kMaximumDocumentLabels = 100000;
constexpr Standard_Size kMaximumDecodedTextureBytes =
    128ull * 1024ull * 1024ull;
constexpr double kMinimumPrimitiveScale = 1.0e-4;
constexpr double kMaximumPrimitiveScale = 1.0e4;
constexpr double kMinimumPrimitiveDimension = 1.0e-3;
constexpr double kMaximumPrimitiveDimension = 1.0e6;
constexpr double kMaximumPositionMagnitude = 1.0e6;
constexpr double kMaximumRotationMagnitude = 360000.0;

using PreparedPrimitive = OrdinaryCreationRequest;

bool ReadFiniteJSONNumber(id value, double& result) {
    if (![value isKindOfClass:[NSNumber class]]) {
        return false;
    }

    NSNumber* number = static_cast<NSNumber*>(value);
    if (CFGetTypeID((__bridge CFTypeRef)number) == CFBooleanGetTypeID()) {
        return false;
    }

    result = number.doubleValue;
    return std::isfinite(result);
}

bool ReadFiniteVector3(id value, std::array<double, 3>& result) {
    if (![value isKindOfClass:[NSArray class]]) {
        return false;
    }

    NSArray* values = static_cast<NSArray*>(value);
    if (values.count != result.size()) {
        return false;
    }

    for (NSUInteger index = 0; index < values.count; ++index) {
        if (!ReadFiniteJSONNumber(values[index], result[index])) {
            return false;
        }
    }
    return true;
}

bool IsBoundedMagnitude(const std::array<double, 3>& values, double maximum) {
    for (double value : values) {
        if (std::abs(value) > maximum) {
            return false;
        }
    }
    return true;
}

bool IsValidScale(const std::array<double, 3>& scale) {
    for (double value : scale) {
        if (value < kMinimumPrimitiveScale || value > kMaximumPrimitiveScale) {
            return false;
        }
    }
    return true;
}

bool IsValidDimension(double value) {
    return std::isfinite(value)
        && value >= kMinimumPrimitiveDimension
        && value <= kMaximumPrimitiveDimension;
}



AssetImportResult ImportResultForReaderStatus(PCDM_ReaderStatus status) {
    switch (status) {
        case PCDM_RS_OK:
            return AssetImportResult::Success;
        case PCDM_RS_NoDocument:
        case PCDM_RS_FormatFailure:
        case PCDM_RS_UnrecognizedFileFormat:
        case PCDM_RS_NoModel:
            return AssetImportResult::InvalidData;
        case PCDM_RS_OpenError:
        case PCDM_RS_WrongStreamMode:
        case PCDM_RS_PermissionDenied:
        case PCDM_RS_UnknownDocument:
            return AssetImportResult::TemporaryFileFailure;
        case PCDM_RS_AlreadyRetrievedAndModified:
        case PCDM_RS_AlreadyRetrieved:
            return AssetImportResult::Busy;
        case PCDM_RS_UnknownFileDriver:
        case PCDM_RS_NoVersion:
        case PCDM_RS_TypeFailure:
        case PCDM_RS_TypeNotFoundInSchema:
        case PCDM_RS_NoDriver:
        case PCDM_RS_NoSchema:
            return AssetImportResult::UnsupportedVersion;
        case PCDM_RS_MakeFailure:
        case PCDM_RS_ReaderException:
        case PCDM_RS_ExtensionFailure:
        case PCDM_RS_DriverFailure:
        case PCDM_RS_WrongResource:
        case PCDM_RS_UserBreak:
            return AssetImportResult::InternalFailure;
    }
}

void CloseDocumentNoThrow(const Handle(TDocStd_Application)& app,
                          Handle(TDocStd_Document)& document) noexcept {
    if (app.IsNull() || document.IsNull()) {
        return;
    }

    try {
        if (document->HasOpenCommand()) {
            document->AbortCommand();
        }
        app->Close(document);
    } catch (...) {
        // A failed cleanup must not replace the original load result or crash rollback.
    }
    document.Nullify();
}

bool IsValidTopologyOrientation(
    const TopAbs_Orientation orientation) noexcept {
    switch (orientation) {
        case TopAbs_FORWARD:
        case TopAbs_REVERSED:
        case TopAbs_INTERNAL:
        case TopAbs_EXTERNAL:
            return true;
    }
    return false;
}

bool IsAllowedTopologyChild(
    const TopAbs_ShapeEnum parent,
    const TopAbs_ShapeEnum child) noexcept {
    switch (parent) {
        case TopAbs_COMPOUND:
            return child >= TopAbs_COMPOUND && child < TopAbs_SHAPE;
        case TopAbs_COMPSOLID:
            return child == TopAbs_SOLID;
        case TopAbs_SOLID:
            return child == TopAbs_SHELL;
        case TopAbs_SHELL:
            return child == TopAbs_FACE;
        case TopAbs_FACE:
            return child == TopAbs_WIRE;
        case TopAbs_WIRE:
            return child == TopAbs_EDGE;
        case TopAbs_EDGE:
            return child == TopAbs_VERTEX;
        case TopAbs_VERTEX:
        case TopAbs_SHAPE:
            return false;
    }
    return false;
}

bool ValidateBoundedTopologicalShape(
    const TopoDS_Shape& shape,
    Standard_Size& aggregateSubshapes,
    const Standard_Size maximumAggregateSubshapes) {
    if (shape.IsNull() || maximumAggregateSubshapes == 0) {
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
#ifdef DEBUG
        gBoundedProjectTopologyValidationCount.fetch_add(
            1, std::memory_order_relaxed);
#endif
        struct TopologyFrame {
            TopoDS_Shape shape;
            Standard_Size depth = 0;
            bool leaving = false;
        };
        const Standard_Size maximumSubshapesPerDefinition =
            std::min(kMaximumSubshapesPerDefinition,
                     maximumAggregateSubshapes);
        const std::size_t maximumStackFrames =
            static_cast<std::size_t>(maximumSubshapesPerDefinition) * 2U;
        std::vector<TopologyFrame> stack = {{shape, 0, false}};
        TopTools_MapOfShape visited;
        TopTools_MapOfShape activePath;
        Standard_Size count = 0;
        while (!stack.empty()) {
            const TopologyFrame frame = stack.back();
            stack.pop_back();
            if (frame.shape.IsNull()) {
                return false;
            }
            if (frame.leaving) {
                activePath.Remove(frame.shape);
                visited.Add(frame.shape);
                continue;
            }
            if (visited.Contains(frame.shape)) {
                continue;
            }
            if (activePath.Contains(frame.shape)
                || frame.shape.ShapeType() == TopAbs_SHAPE
                || !IsValidTopologyOrientation(frame.shape.Orientation())
                || frame.depth > kMaximumAssemblyTraversalDepth
                || count >= maximumSubshapesPerDefinition
                || aggregateSubshapes >= maximumAggregateSubshapes) {
                return false;
            }
            activePath.Add(frame.shape);
            ++count;
            if (count > maximumAggregateSubshapes - aggregateSubshapes
                || stack.size() >= maximumStackFrames) {
                return false;
            }
            stack.push_back({frame.shape, frame.depth, true});
            const TopAbs_ShapeEnum parentType = frame.shape.ShapeType();
            for (TopoDS_Iterator child(
                     frame.shape, Standard_True, Standard_True);
                 child.More(); child.Next()) {
                const TopoDS_Shape& childShape = child.Value();
                if (childShape.IsNull()
                    || !IsAllowedTopologyChild(
                        parentType, childShape.ShapeType())
                    || !IsValidTopologyOrientation(
                        childShape.Orientation())
                    || stack.size() >= maximumStackFrames) {
                    return false;
                }
                stack.push_back({childShape, frame.depth + 1, false});
            }
        }
        aggregateSubshapes += count;
        // Do not call BRepCheck_Analyzer here. It evaluates attacker-controlled
        // curves and surfaces without a progress/cancellation hook. Safe binary
        // retrieval plus this canonical, cycle-aware, allocation-bounded walk
        // is the fail-closed project-load gate; ordinary modeling operations
        // retain geometric validity checks through IsTopologicallyValid.
        return true;
    } catch (...) {
        return false;
    }
}

struct ShapeTraversalFrame {
    TDF_Label label;
    Standard_Size depth = 0;
    bool leaving = false;
};

bool HasValidOptionalLengthUnit(
    const Handle(TDocStd_Document)& document) {
    if (document.IsNull()) {
        return false;
    }

    Standard_Real metersPerUnit = 0.0;
    const bool hasLengthUnit =
        XCAFDoc_DocumentTool::GetLengthUnit(document, metersPerUnit);
    return !hasLengthUnit
        || (std::isfinite(metersPerUnit) && metersPerUnit > 0.0);
}

bool ValidateShapeTree(
    const Handle(TDocStd_Document)& document,
    TDF_LabelMap& activeDefinitionLabels,
    const Standard_Size maximumAggregateSubshapes) {
    activeDefinitionLabels.Clear();
    if (document.IsNull() || maximumAggregateSubshapes == 0) {
        return false;
    }

    const Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(document->Main());
    if (shapeTool.IsNull()) {
        return false;
    }

    std::vector<ShapeTraversalFrame> stack;
    stack.reserve(64);
    for (TDF_ChildIterator root(shapeTool->Label(), Standard_False);
         root.More(); root.Next()) {
        const TDF_Label& label = root.Value();
        if (!label.IsNull() && XCAFDoc_ShapeTool::IsShape(label)) {
            if (stack.size()
                >= static_cast<std::size_t>(kMaximumShapeDefinitions)) {
                return false;
            }
            stack.push_back({label, 0, false});
        }
    }

    TDF_LabelMap activePath;
    TDF_LabelMap visitedLabels;
    Standard_Size traversalNodes = 0;
    Standard_Size definitionCount = 0;
    Standard_Size aggregateSubshapes = 0;
    while (!stack.empty()) {
        const ShapeTraversalFrame frame = stack.back();
        stack.pop_back();
        if (frame.leaving) {
            activePath.Remove(frame.label);
            visitedLabels.Add(frame.label);
            continue;
        }
        if (visitedLabels.Contains(frame.label)) {
            continue;
        }
        if (frame.label.IsNull()
            || frame.depth > kMaximumAssemblyTraversalDepth
            || activePath.Contains(frame.label)
            || ++traversalNodes > kMaximumAssemblyTraversalNodes) {
            return false;
        }

        activePath.Add(frame.label);
        stack.push_back({frame.label, frame.depth, true});

        if (XCAFDoc_ShapeTool::IsReference(frame.label)
            || XCAFDoc_ShapeTool::IsComponent(frame.label)) {
            TDF_Label referredLabel;
            if (!XCAFDoc_ShapeTool::GetReferredShape(
                    frame.label, referredLabel)
                || referredLabel.IsNull()
                || referredLabel.Data() != frame.label.Data()
                || !XCAFDoc_ShapeTool::IsShape(referredLabel)) {
                return false;
            }
            stack.push_back({referredLabel, frame.depth + 1, false});
            continue;
        }

        if (!XCAFDoc_ShapeTool::IsShape(frame.label)
            || ++definitionCount > kMaximumShapeDefinitions) {
            return false;
        }
        activeDefinitionLabels.Add(frame.label);
        const TopoDS_Shape definitionShape =
            XCAFDoc_ShapeTool::GetShape(frame.label);
        if (!ValidateBoundedTopologicalShape(
                definitionShape,
                aggregateSubshapes,
                maximumAggregateSubshapes)) {
            return false;
        }

        if (!XCAFDoc_ShapeTool::IsAssembly(frame.label)) {
            continue;
        }

        std::vector<TDF_Label> components;
        for (TDF_ChildIterator child(frame.label, Standard_False);
             child.More(); child.Next()) {
            const TDF_Label& component = child.Value();
            if (XCAFDoc_ShapeTool::IsComponent(component)) {
                const Standard_Size componentCount =
                    static_cast<Standard_Size>(components.size()) + 1;
                if (traversalNodes > kMaximumAssemblyTraversalNodes
                    || componentCount
                        > kMaximumAssemblyTraversalNodes - traversalNodes
                    || stack.size()
                        > static_cast<std::size_t>(
                            kMaximumAssemblyTraversalNodes)
                    || componentCount
                        > kMaximumAssemblyTraversalNodes
                            - static_cast<Standard_Size>(stack.size())) {
                    return false;
                }
                components.push_back(component);
            }
        }
        for (auto component = components.rbegin();
             component != components.rend(); ++component) {
            stack.push_back({*component, frame.depth + 1, false});
        }
    }

    // An empty document is valid: a user can intentionally save a blank scene.
    return true;
}

const Standard_GUID& LocalPBRMaterialValidationAttributeID() {
    static const Standard_GUID id(
        "248A5203-4A22-4F2B-85C4-BE0BA89A5E4D");
    return id;
}

const Standard_GUID& AutoPromotedEmissiveFactorValidationAttributeID() {
    static const Standard_GUID id(
        "1DA4580F-1B19-46DA-ABD4-BBCE9FBADE44");
    return id;
}

bool IsFiniteUnit(const Standard_Real value) {
    return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

bool ValidateColor(const Quantity_Color& color) {
    return IsFiniteUnit(color.Red())
        && IsFiniteUnit(color.Green())
        && IsFiniteUnit(color.Blue());
}

using TextureValidationState = Core3DEmbeddedTextureBudgetState;

bool ValidateEmbeddedTexture(const Handle(Image_Texture)& texture,
                             TextureValidationState& state) {
    return Core3DAccumulateEmbeddedTextureBudget(
        texture, state,
        kMaximumAggregateTextureBytes,
        kMaximumDecodedTextureBytes);
}

bool ValidatePBRMaterial(const XCAFDoc_VisMaterialPBR& material,
                         TextureValidationState& textures) {
    const Quantity_Color& baseColor = material.BaseColor.GetRGB();
    return material.IsDefined
        && ValidateColor(baseColor)
        && IsFiniteUnit(material.BaseColor.Alpha())
        && IsFiniteUnit(material.Metallic)
        && IsFiniteUnit(material.Roughness)
        && std::isfinite(material.RefractionIndex)
        && material.RefractionIndex >= 1.0f
        && material.RefractionIndex <= 3.0f
        && std::isfinite(material.EmissiveFactor.x())
        && std::isfinite(material.EmissiveFactor.y())
        && std::isfinite(material.EmissiveFactor.z())
        && material.EmissiveFactor.x() >= 0.0f
        && material.EmissiveFactor.y() >= 0.0f
        && material.EmissiveFactor.z() >= 0.0f
        && material.EmissiveFactor.x() <= kMaximumEmissionFactor
        && material.EmissiveFactor.y() <= kMaximumEmissionFactor
        && material.EmissiveFactor.z() <= kMaximumEmissionFactor
        && ValidateEmbeddedTexture(material.BaseColorTexture, textures)
        && ValidateEmbeddedTexture(
            material.MetallicRoughnessTexture, textures)
        && ValidateEmbeddedTexture(material.EmissiveTexture, textures)
        && ValidateEmbeddedTexture(material.OcclusionTexture, textures)
        && ValidateEmbeddedTexture(material.NormalTexture, textures);
}

bool ValidateCommonMaterial(const XCAFDoc_VisMaterialCommon& material,
                            TextureValidationState& textures) {
    return material.IsDefined
        && ValidateColor(material.AmbientColor)
        && ValidateColor(material.DiffuseColor)
        && ValidateColor(material.SpecularColor)
        && ValidateColor(material.EmissiveColor)
        && IsFiniteUnit(material.Shininess)
        && IsFiniteUnit(material.Transparency)
        && ValidateEmbeddedTexture(material.DiffuseTexture, textures);
}

bool ValidateVisualMaterials(
    const Handle(TDocStd_Document)& document,
    const TDF_LabelMap& activeDefinitionLabels) {
    if (document.IsNull()) {
        return false;
    }

    Standard_Size frameResidentBytes = 0;
    if (!Core3DValidateOwnedFrameUsage(document,frameResidentBytes)) return false;

    Handle(XCAFDoc_VisMaterialTool) materialTool;
    if (XCAFDoc_DocumentTool::CheckVisMaterialTool(document->Main())) {
        materialTool = XCAFDoc_DocumentTool::VisMaterialTool(
            document->Main());
        if (materialTool.IsNull()) {
            return false;
        }
    }

    TextureValidationState textureState;
    TDF_LabelMap tableMaterialLabels;
    if (!materialTool.IsNull()) {
        TDF_LabelSequence materialLabels;
        materialTool->GetMaterials(materialLabels);
        if (materialLabels.Length()
            > kMaximumVisualMaterialDefinitions) {
            return false;
        }
        for (TDF_LabelSequence::Iterator iterator(materialLabels);
             iterator.More(); iterator.Next()) {
            tableMaterialLabels.Add(iterator.Value());
            const Handle(XCAFDoc_VisMaterial) material =
                XCAFDoc_VisMaterialTool::GetMaterial(iterator.Value());
            if (material.IsNull()
                || (!material->HasPbrMaterial()
                    && !material->HasCommonMaterial())) {
                return false;
            }
            if (material->HasPbrMaterial()
                && !ValidatePBRMaterial(
                    material->PbrMaterial(), textureState)) {
                return false;
            }
            if (material->HasCommonMaterial()
                && !ValidateCommonMaterial(
                    material->CommonMaterial(), textureState)) {
                return false;
            }
            switch (material->AlphaMode()) {
                case Graphic3d_AlphaMode_BlendAuto:
                case Graphic3d_AlphaMode_Opaque:
                case Graphic3d_AlphaMode_Mask:
                case Graphic3d_AlphaMode_Blend:
                    break;
                case Graphic3d_AlphaMode_MaskBlend:
                    // The renderer-neutral contract currently has distinct
                    // mask and blend modes, not a combined two-stage mode.
                    // Reject instead of silently discarding the cutoff.
                    return false;
                default:
                    return false;
            }
            if (!IsFiniteUnit(material->AlphaCutOff())) {
                return false;
            }
            switch (material->FaceCulling()) {
                case Graphic3d_TypeOfBackfacingModel_Auto:
                case Graphic3d_TypeOfBackfacingModel_DoubleSided:
                case Graphic3d_TypeOfBackfacingModel_BackCulled:
                case Graphic3d_TypeOfBackfacingModel_FrontCulled:
                    break;
                default:
                    return false;
            }
        }
    }

    TDF_LabelMap validatedLocalMaterialLabels;
    std::unordered_set<std::string> validatedCanonicalTextureIDs;
    const auto hasUnregisteredDirectMaterial =
        [&](const TDF_Label& label) {
            Handle(XCAFDoc_VisMaterial) directMaterial;
            return !label.IsNull()
                && label.FindAttribute(
                    XCAFDoc_VisMaterial::GetID(), directMaterial)
                && (directMaterial.IsNull()
                    || !tableMaterialLabels.Contains(label));
        };
    const Handle(TDF_Data)& documentData = document->GetData();
    if (documentData.IsNull()
        || Core3DNormalTextureRecipeForLabel(documentData->Root()) != 0
        || hasUnregisteredDirectMaterial(documentData->Root())
        || hasUnregisteredDirectMaterial(document->Main())) {
        return false;
    }
    Standard_Size labelCount = 0;
    for (TDF_ChildIterator iterator(
             documentData->Root(), Standard_True);
         iterator.More(); iterator.Next()) {
        if (++labelCount > kMaximumDocumentLabels) {
            return false;
        }
        const TDF_Label& label = iterator.Value();
        Handle(XCAFDoc_VisMaterial) directMaterial;
        if (label.FindAttribute(
                XCAFDoc_VisMaterial::GetID(), directMaterial)
            && (directMaterial.IsNull() || materialTool.IsNull()
                || !tableMaterialLabels.Contains(label))) {
            // Raw BinXCAF preflight charges every material attribute. Keeping
            // orphans outside the material-tool table would let a writer scan
            // omit serialized texture occurrences and exceed the reopen cap.
            return false;
        }
        TDF_Label assignedMaterialLabel;
        if (XCAFDoc_VisMaterialTool::GetShapeMaterial(
                label, assignedMaterialLabel)
            && (materialTool.IsNull()
                || assignedMaterialLabel.IsNull()
                || !materialTool->IsMaterial(assignedMaterialLabel))) {
            return false;
        }

        Handle(TDataStd_Integer) marker;
        const auto normalRecipe = Core3DNormalTextureRecipeForLabel(label);
        Handle(TDataStd_Integer) autoPromotedEmissiveFactor;
        const bool hasAutoPromotedEmissiveFactor = label.FindAttribute(
            AutoPromotedEmissiveFactorValidationAttributeID(),
            autoPromotedEmissiveFactor);
        if (!label.FindAttribute(
                LocalPBRMaterialValidationAttributeID(), marker)) {
            if (hasAutoPromotedEmissiveFactor || normalRecipe != 0) {
                return false;
            }
            continue;
        }
        if (marker.IsNull() || marker->Get() != 1
            || !activeDefinitionLabels.Contains(label)
            || materialTool.IsNull()
            || assignedMaterialLabel.IsNull()) {
            return false;
        }
        if (hasAutoPromotedEmissiveFactor) {
            const Handle(XCAFDoc_VisMaterial) material =
                XCAFDoc_VisMaterialTool::GetMaterial(
                    assignedMaterialLabel);
            if (autoPromotedEmissiveFactor.IsNull()
                || autoPromotedEmissiveFactor->Get() != 1
                || material.IsNull() || !material->HasPbrMaterial()) {
                return false;
            }
            const XCAFDoc_VisMaterialPBR& pbr =
                material->PbrMaterial();
            if (pbr.EmissiveTexture.IsNull()
                || pbr.EmissiveFactor.x() != 1.0f
                || pbr.EmissiveFactor.y() != 1.0f
                || pbr.EmissiveFactor.z() != 1.0f) {
                return false;
            }
        }
        // Geometry and frame cost belong to each object binding, even when
        // several objects share one already-validated material definition.
        const Handle(XCAFDoc_VisMaterial) assignedMaterial =
            XCAFDoc_VisMaterialTool::GetMaterial(assignedMaterialLabel);
        if (!assignedMaterial.IsNull() && assignedMaterial->HasPbrMaterial()
            && !assignedMaterial->PbrMaterial().NormalTexture.IsNull()) {
            if (!Core3DValidateNormalTextureBinding(document,label)) return false;
        } else if (normalRecipe != 0) {
            return false;
        }
        if (!validatedLocalMaterialLabels.Contains(
                assignedMaterialLabel)) {
            const Handle(XCAFDoc_VisMaterial) material =
                XCAFDoc_VisMaterialTool::GetMaterial(
                    assignedMaterialLabel);
            if (material.IsNull() || !material->HasPbrMaterial()) {
                return false;
            }
            const XCAFDoc_VisMaterialPBR& pbr = material->PbrMaterial();
            if ((!pbr.NormalTexture.IsNull()
                    && !Core3DValidateNumericTexture(pbr.NormalTexture))
                || (!pbr.MetallicRoughnessTexture.IsNull()
                    && !Core3DValidateNumericTexture(pbr.MetallicRoughnessTexture))
                || (!pbr.OcclusionTexture.IsNull()
                    && !Core3DValidateNumericTexture(pbr.OcclusionTexture))) {
                return false;
            }
            const Handle(Image_Texture)& pbrBase = pbr.BaseColorTexture;
            const Handle(Image_Texture) commonBase =
                material->HasCommonMaterial()
                    ? material->CommonMaterial().DiffuseTexture
                    : Handle(Image_Texture)();
            if (pbrBase.IsNull() != commonBase.IsNull()) {
                return false;
            }
            if (!pbrBase.IsNull()) {
                if (!Core3DTexturesMatch(pbrBase, commonBase)) {
                    // Exact same-ID byte equality has already been established
                    // by TextureValidationState while walking the bounded
                    // material table above.
                    return false;
                }
            }
            for (const Handle(Image_Texture)& texture : {
                     pbrBase, pbr.EmissiveTexture, pbr.MetallicRoughnessTexture, pbr.OcclusionTexture, pbr.NormalTexture}) {
                if (texture.IsNull()) {
                    continue;
                }
                const std::string textureID(
                    texture->TextureId().ToCString());
                if (textureID.empty()
                    || (validatedCanonicalTextureIDs.insert(textureID).second
                        && !Core3DValidateAuthoredTexture(texture))) {
                    return false;
                }
            }
            validatedLocalMaterialLabels.Add(assignedMaterialLabel);
        }
        for (const Standard_Integer tag : {11, 12}) {
            const TDF_Label legacy = label.FindChild(tag, Standard_False);
            Handle(TDataStd_Integer) legacyValue;
            if (!legacy.IsNull()
                && legacy.FindAttribute(
                    TDataStd_Integer::GetID(), legacyValue)) {
                return false;
            }
        }
    }
    return true;
}

bool IsProjectFileWithinSizeLimit(const std::string& path) {
    struct stat fileInfo = {};
    return !path.empty()
        && ::stat(path.c_str(), &fileInfo) == 0
        && fileInfo.st_size >= 0
        && static_cast<std::uint64_t>(fileInfo.st_size)
            <= kMaximumProjectDocumentBytes;
}

} // namespace

// Retain both documents and original AIS handles until actual restoration is
// confirmed. A previous OCAF pointer alone never releases the replacement fence.
struct DocumentReplacementWork {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) previous, candidate;
    AIS_ListOfInteractive presentations;
    PrimitiveManipulatorType manipulator;
    ShapeSelectionMode selection;
    std::optional<authority::ReplacementReservation> reservation;
    enum class Phase { Installing, RestorePending, Restoring };
    Phase phase = Phase::Installing;
};

void Core3DViewer::release() noexcept {
    if (_nativeOpeningState) _nativeOpeningState->owner = nullptr;
    _nativeOpeningState.reset();
    // Retire pre-commit source authority on main before graphics teardown.
    (void)discardSavedCutSourceEdit(_savedCutSourceEditWork.lock());
    (void)discardSavedProgramSourceEdit(_savedProgramSourceEditWork.lock());
    // Interactors retain the view, context, document, and manipulator graphics.
    // GLViewController calls this while the viewport EAGL context is current,
    // so release them before the base handles and before that context is
    // restored. Repeated calls are intentionally harmless.
    _meshVertexEditWork.reset();
    _meshRegionExtrudeWork.reset();
    _meshRegionInsetWork.reset();
    _interactiveCallback = {};
    _booleanPreviewStateChangedCallback = {};
    _linearArrayPreviewStateChangedCallback = {};
    _bevelPreviewStateChangedCallback = {};
    _shellPreviewStateChangedCallback = {};
	if (_objectInteractor != nullptr) {
		// Close any temporary face selection modes while their presentations
		// and the AIS context are still alive. Full Mirror cancellation then
		// retires the owned reference/preview handles before graphics teardown.
		(void)_objectInteractor->cancelMirrorPlanePicking();
		(void)_objectInteractor->cancelMirror();
	}
    if (_transformInspectorMeasurementController != nullptr) {
        _transformInspectorMeasurementController->shutdown();
        _transformInspectorMeasurementController.reset();
    }
    _shapeInteractor.reset();
    _objectInteractor.reset();
    _ordinaryEditController.reset();
    // Terminal teardown revokes adoption. It does not assert that detached
    // file preparation completed; late completions no longer own this slot.
    _queuedAssetLoadWork.reset();
    OcctViewer::release();
    // Graphics and interactors are gone. The failed candidate is never saved or
    // promoted; release its application ownership during final viewer teardown.
    if (_documentReplacementWork) {
        CloseDocumentNoThrow(_documentReplacementWork->application, _documentReplacementWork->candidate);
        _documentReplacementWork.reset();
    }
}

NSString* Core3DViewer::addTestPrimitives() {
    if (!canBeginCommittedEdit()) { return @""; }

    gp_Pnt lowerLeftCornerOfBox(-50.0,-50.0,0.0);
    BRepPrimAPI_MakeBox boxMaker(lowerLeftCornerOfBox,100,100,50);
    TopoDS_Shape box = boxMaker.Shape();

    //Create a cylinder with a radius 25.0 and height 50.0, centered at the origin
    BRepPrimAPI_MakeCylinder cylinderMaker(25.0,50.0);
    TopoDS_Shape cylinder = cylinderMaker.Shape();

    //Cut the cylinder out from the box
    BRepAlgoAPI_Cut cutMaker(box,cylinder);
    //TopoDS_Shape boxWithHole = cutMaker.Shape();

    // fillets
    BRepFilletAPI_MakeFillet  MF(cutMaker);
    TopExp_Explorer  ex(cutMaker,TopAbs_EDGE);
    while (ex.More()) {
        MF.Add(5,TopoDS::Edge(ex.Current()));
        ex.Next();
    }

    gp_Ax2 cc1;
    cc1.SetLocation(gp_Pnt(50.0, 50.0, 10.0));

    BRepPrimAPI_MakeCylinder cylinderMaker1(cc1, 25.0,50.0);
    TopoDS_Shape cylinder1 = cylinderMaker1.Shape();

    BRepAlgoAPI_Fuse fuseMaker(cylinder1, MF.Shape());

    Handle(AIS_InteractiveObject) aShapePrs = new AIS_Shape (fuseMaker.Shape());
    myContext->Display (aShapePrs, AIS_Shaded, 0, false);

    return @"";

    //    NSString *cacheDirectory = NSSearchPathForDirectoriesInDomains(NSCachesDirectory, NSUserDomainMask, YES).firstObject;
    //
    //    __auto_type stlFilename = [cacheDirectory stringByAppendingPathComponent:@"test.stl"];
    //
    //    StlAPI_Writer aStlWriter;
    //    aStlWriter.Write(fuseMaker.Shape(), stlFilename.UTF8String);
    //
    //    return stlFilename;
}

bool Core3DViewer::InitViewer (Core3DPlatformView* theWin) {
    bool result = OcctViewer::InitViewer(theWin);
    if(result) {
        if(_objectInteractor == nullptr) {
#if TARGET_OS_OSX
            const float device_independent_side = 96.0f; // OCCT logical view units.
#else
            const float scale = [[UIScreen mainScreen] scale];
            const float ppm = scale * (([[UIDevice currentDevice] userInterfaceIdiom] == UIUserInterfaceIdiomPad) ? 132 : 163) / 25.4;
            const float device_independent_side = 15 * ppm;
#endif
            _objectInteractor = std::make_shared<ObjectInteractor>(myContext, myView, myDoc, device_independent_side);
            _objectInteractor->setBooleanPreviewStateChangedCallback(
                _booleanPreviewStateChangedCallback);
            _objectInteractor->setLinearArrayPreviewStateChangedCallback(
                _linearArrayPreviewStateChangedCallback);
            _objectInteractor->setRadialArrayPreviewStateChangedCallback(
                _radialArrayPreviewStateChangedCallback);
        }
        if(_shapeInteractor == nullptr) {
            _shapeInteractor = std::make_shared<ShapeInteractor>(myContext, myView, myDoc);
            _shapeInteractor->setBevelPreviewStateChangedCallback(
                _bevelPreviewStateChangedCallback);
            _shapeInteractor->setShellPreviewStateChangedCallback(
                _shellPreviewStateChangedCallback);
        }
        if (_transformInspectorMeasurementController == nullptr) {
            try {
                _transformInspectorMeasurementController =
                    std::make_shared<
                        TransformInspectorMeasurementController>(
                        myContext,
                        myDoc);
            } catch (...) {
                return false;
            }
        }
        if (_ordinaryEditController == nullptr) {
            _ordinaryEditController = std::make_shared<OrdinaryEditController>(myDoc,
                static_cast<OrdinaryEditPresentationHost&>(*this));
        }
        _objectInteractor->_ordinaryEditController = _ordinaryEditController;
    }
    return result;
}

namespace {
std::optional<BooleanAction> HistoryBooleanAction(PrimitiveManipulatorType type)
{
    switch (type) {
        case PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract:
            return BooleanAction::BooleanSubtract;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeUnion:
            return BooleanAction::BooleanUnion;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeIntersect:
            return BooleanAction::BooleanIntersect;
        default: return std::nullopt;
    }
}
}

bool Core3DViewer::retireBooleanAction(BooleanAction action)
{
    if (![NSThread isMainThread] || !_objectInteractor
        || (action != BooleanAction::BooleanSubtract
            && action != BooleanAction::BooleanUnion
            && action != BooleanAction::BooleanIntersect)) return false;
    const auto objectInteractor = _objectInteractor;
    objectInteractor->cancelBoolean(action);
    return _objectInteractor == objectInteractor
        && !objectInteractor->hasUnresolvedBoolean()
        && !objectInteractor->hasActiveBoolean(action);
}

NativeBooleanRetention Core3DViewer::restoreBooleanAction(BooleanAction action)
{
    if (![NSThread isMainThread] || !_objectInteractor)
        return NativeBooleanRetention::Unavailable;
    const auto objectInteractor = _objectInteractor;
    const auto retained = HistoryBooleanAction(objectInteractor->getManipulatorType());
    // Redraw recreates stateful tools as None. A stale host enum must not
    // fabricate a hidden Boolean ledger after committed history changed.
    if (!retained || *retained != action)
        return NativeBooleanRetention::Unavailable;
    const bool active = objectInteractor->hasActiveBoolean(action)
        || objectInteractor->beginBoolean(action);
    if (_objectInteractor != objectInteractor)
        return NativeBooleanRetention::Unavailable;
    if (active) {
        return HistoryBooleanAction(objectInteractor->getManipulatorType()) == retained
            && objectInteractor->hasActiveBoolean(action)
            ? NativeBooleanRetention::Retained : NativeBooleanRetention::Unavailable;
    }
    if (objectInteractor->hasActiveBoolean(action)
        || objectInteractor->hasUnresolvedBoolean())
        return NativeBooleanRetention::Unavailable;
    // The host owns its cached tool/UI state and must reconcile it before
    // publishing selection/render notifications. Never silently change it here.
    return NativeBooleanRetention::HostReconciliationRequired;
}

NativeHistoryTransition Core3DViewer::performHistory(NativeHistoryDirection direction)
{
    NativeHistoryTransition result;
    if (![NSThread isMainThread] || !_objectInteractor || myDoc.IsNull()
        || hasUnresolvedEdit()
        || (direction != NativeHistoryDirection::Undo
            && direction != NativeHistoryDirection::Redo)) return result;
    // Preview notifications may synchronously replace the interactors. Keep the
    // executing owners alive and stop before touching a replacement's state.
    const auto objectInteractor = _objectInteractor;
    const auto shapeInteractor = _shapeInteractor;
    const Handle(OcctDocument) document = myDoc;
    const auto ownsHistory = [&] {
        return _objectInteractor == objectInteractor
            && _shapeInteractor == shapeInteractor && myDoc == document;
    };
    const auto type = objectInteractor->getManipulatorType();
    const auto booleanAction = HistoryBooleanAction(type);
    const auto cancelledPreview = [&](bool cancelled, bool refreshSelection) {
        result.outcome = cancelled ? NativeHistoryOutcome::PreviewCancelled
                                  : NativeHistoryOutcome::PreviewCancellationFailed;
        result.refreshSelection = refreshSelection;
        result.requestRender = true;
    };
    if (shapeInteractor && shapeInteractor->hasActiveBevel()) {
        const bool cancelled = shapeInteractor->cancelChamfer();
        if (!ownsHistory()) return result;
        cancelledPreview(cancelled, true);
        result.primaryInteractionCancelled = cancelled;
        return result;
    }
    if (shapeInteractor
        && (type == PrimitiveManipulatorType::PrimitiveGizmoTypeExtrude
            || shapeInteractor->hasActiveExtrusion())) {
        const bool cancelled = shapeInteractor->cancelExtrusion();
        if (!ownsHistory()) return result;
        if (cancelled) objectInteractor->setManipulatorType(
            PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
        if (!ownsHistory()) return result;
        cancelledPreview(cancelled, false);
        return result;
    }
    if (shapeInteractor
        && (type == PrimitiveManipulatorType::PrimitiveGizmoTypeShell
            || shapeInteractor->hasActiveShell())) {
        const bool cancelled = shapeInteractor->cancelShell();
        if (!ownsHistory()) return result;
        if (cancelled) objectInteractor->setManipulatorType(
            PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
        if (!ownsHistory()) return result;
        cancelledPreview(cancelled, true);
        return result;
    }
    if (booleanAction) {
        const bool cancelled = retireBooleanAction(*booleanAction);
        if (!ownsHistory()) return result;
        if (!cancelled) {
            cancelledPreview(false, true);
            return result;
        }
    } else if (type == PrimitiveManipulatorType::PrimitiveGizmoTypeMirror) {
        const bool cancelled = objectInteractor->cancelMirror();
        if (!ownsHistory()) return result;
        if (!cancelled) {
            cancelledPreview(false, true);
            return result;
        }
    } else if (type == PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray) {
        const bool cancelled = objectInteractor->cancelLinearArray();
        if (!ownsHistory()) return result;
        cancelledPreview(cancelled, true);
        return result;
    } else if (type == PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray) {
        const bool cancelled = objectInteractor->cancelRadialArray();
        if (!ownsHistory()) return result;
        cancelledPreview(cancelled, true);
        return result;
    }
    objectInteractor->detachManipulator(false);
    if (!ownsHistory()) return result;
    const bool canChange = direction == NativeHistoryDirection::Undo
        ? document->canUndo() : document->canRedo();
    result.outcome = NativeHistoryOutcome::NoHistory;
    if (canChange) {
        const bool changed = direction == NativeHistoryDirection::Undo
            ? document->undo() : document->redo();
        result.outcome = changed ? NativeHistoryOutcome::HistoryChanged
                                 : NativeHistoryOutcome::HistoryFailed;
        if (changed) result.documentRedrawn = redrawDocument();
    }
    if (booleanAction) {
        result.reconcileBooleanTool = restoreBooleanAction(*booleanAction)
            == NativeBooleanRetention::HostReconciliationRequired;
    }
    result.refreshSelection = true;
    result.requestRender = true;
    return result;
}

bool Core3DViewer::recreateInteractors(PrimitiveManipulatorType theManipulatorType,
                                       ShapeSelectionMode theSelectionMode) {
    if (!IsStatelessManipulatorType(theManipulatorType)) {
        // Operation controllers own state that is not encoded by this enum.
        // Reconstructing one from the enum would fabricate a fresh ledger and
        // silently discard the retained operation's recovery authority.
        return false;
    }
    SelectionContextModesSnapshot previousSelectionContext;
    if (!CaptureSelectionContextModes(
            myContext, previousSelectionContext)) {
        return false;
    }
#if TARGET_OS_OSX
    const float manipulatorSide = 96.0f; // Same view units as initial attachment.
#else
    const float scale = [[UIScreen mainScreen] scale];
    const float pointsPerMillimeter = scale
        * (([[UIDevice currentDevice] userInterfaceIdiom] == UIUserInterfaceIdiomPad) ? 132 : 163)
        / 25.4;
    const float manipulatorSide = 15 * pointsPerMillimeter;
#endif

    std::shared_ptr<ObjectInteractor> objectInteractor =
        std::make_shared<ObjectInteractor>(
            myContext, myView, myDoc, manipulatorSide);
    objectInteractor->setBooleanPreviewStateChangedCallback(
        _booleanPreviewStateChangedCallback);
    objectInteractor->setLinearArrayPreviewStateChangedCallback(
        _linearArrayPreviewStateChangedCallback);
    objectInteractor->setRadialArrayPreviewStateChangedCallback(
        _radialArrayPreviewStateChangedCallback);
    std::shared_ptr<ShapeInteractor> shapeInteractor =
        std::make_shared<ShapeInteractor>(myContext, myView, myDoc);
    shapeInteractor->setBevelPreviewStateChangedCallback(
        _bevelPreviewStateChangedCallback);
    shapeInteractor->setShellPreviewStateChangedCallback(
        _shellPreviewStateChangedCallback);

    const ShapeSelectionModeChangeResult selectionResult =
        shapeInteractor->setSelectionMode(theSelectionMode);
    if (selectionResult != ShapeSelectionModeChangeResult::Succeeded
        || !shapeInteractor->selectionModeAuthorityIsExact()) {
        // The temporary interactor shares the live AIS context. Restore the
        // exact modes that existed before it attempted configuration; if this
        // restoration itself fails, the retained interactor's dynamic
        // authority check remains poisoned and public getters fail closed.
        (void)RestoreSelectionContextModes(
            myContext, previousSelectionContext);
        return false;
    }
    if (theManipulatorType
            != PrimitiveManipulatorType::PrimitiveGizmoTypeNone) {
        objectInteractor->setManipulatorType(theManipulatorType);
        if (objectInteractor->getManipulatorType()
                != theManipulatorType) {
            // A nominally passive tool is publishable only if the fresh
            // interactor accepted it exactly. In particular, Scale is never
            // admitted here because an empty selection downgrades it to None.
            (void)RestoreSelectionContextModes(
                myContext, previousSelectionContext);
            return false;
        }
    }

    // Publish the replacement interactors only after the requested topology
    // mode is accepted. A failed reconstruction therefore cannot silently
    // replace the retained Face/Edge authority with WholeShape.
    objectInteractor->_ordinaryEditController = _ordinaryEditController;
    _objectInteractor = std::move(objectInteractor);
    _shapeInteractor = std::move(shapeInteractor);
    return publishRecreatedInteractorState();
}

bool Core3DViewer::recreateFreshInteractorsForDocumentReplacement()
{
    if (myContext.IsNull()) {
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
        myContext->ClearDetected(Standard_False);
        myContext->ClearSelected(Standard_False);
        if (!recreateInteractors(
                PrimitiveManipulatorType::PrimitiveGizmoTypeNone,
                ShapeSelectionMode::WholeShape)
            || _objectInteractor == nullptr
            || _shapeInteractor == nullptr
            || _objectInteractor->getManipulatorType()
                != PrimitiveManipulatorType::PrimitiveGizmoTypeNone
            || _shapeInteractor->getSelectionMode()
                != ShapeSelectionMode::WholeShape
            || !_shapeInteractor->selectionModeAuthorityIsExact()
            || myContext->HasDetected()) {
            return false;
        }
        myContext->InitSelected();
        return !myContext->MoreSelected();
    } catch (...) {
        return false;
    }
}

bool Core3DViewer::publishRecreatedInteractorState() noexcept
{
    if (_objectInteractor == nullptr) {
        return false;
    }
    if (!_interactorRecreatedCallback) {
        return true;
    }
    try {
        _interactorRecreatedCallback(
            _objectInteractor->getManipulatorType());
        return true;
    } catch (...) {
        return false;
    }
}

void Core3DViewer::setBooleanPreviewStateChangedCallback(
    std::function<void()> theCallback)
{
    _booleanPreviewStateChangedCallback = std::move(theCallback);
    if (_objectInteractor != nullptr) {
        _objectInteractor->setBooleanPreviewStateChangedCallback(
            _booleanPreviewStateChangedCallback);
    }
}

void Core3DViewer::setLinearArrayPreviewStateChangedCallback(
    std::function<void()> theCallback)
{
    _linearArrayPreviewStateChangedCallback = std::move(theCallback);
    if (_objectInteractor != nullptr) {
        _objectInteractor->setLinearArrayPreviewStateChangedCallback(
            _linearArrayPreviewStateChangedCallback);
    }
}

void Core3DViewer::setRadialArrayPreviewStateChangedCallback(
    std::function<void()> theCallback)
{
    _radialArrayPreviewStateChangedCallback = std::move(theCallback);
    if (_objectInteractor != nullptr) {
        _objectInteractor->setRadialArrayPreviewStateChangedCallback(
            _radialArrayPreviewStateChangedCallback);
    }
}

void Core3DViewer::setBevelPreviewStateChangedCallback(
    std::function<void()> theCallback)
{
    _bevelPreviewStateChangedCallback = std::move(theCallback);
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->setBevelPreviewStateChangedCallback(
            _bevelPreviewStateChangedCallback);
    }
}

bool Core3DViewer::captureShellOpenings(const std::string& entityIdentifier,
    const std::vector<ShellOpeningSelector>& selectors,
    FaceOperationSourceProof& proof) const noexcept
{
    proof = FaceOperationSourceProof();
    return canBeginCommittedEdit() && _shapeInteractor
        && _shapeInteractor->captureShellOpenings(entityIdentifier, selectors, proof);
}

bool Core3DViewer::beginCapturedShell(const FaceOperationSourceProof& proof) noexcept
{
    return canBeginCommittedEdit() && _shapeInteractor && _objectInteractor
        && _shapeInteractor->beginCapturedShell(proof);
}

void Core3DViewer::setShellPreviewStateChangedCallback(
    std::function<void()> theCallback)
{
    _shellPreviewStateChangedCallback = std::move(theCallback);
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->setShellPreviewStateChangedCallback(
            _shellPreviewStateChangedCallback);
    }
}

void Core3DViewer::setInteractorRecreatedCallback(
    std::function<void(PrimitiveManipulatorType)> theCallback)
{
    _interactorRecreatedCallback = std::move(theCallback);
}

TransformInspectorMeasurement
Core3DViewer::captureTransformInspectorMeasurement(
    TransformInspectorMeasurementCompletion theCompletion) noexcept
{
    if (hasUnresolvedOrdinaryEdit() || _transformInspectorMeasurementController == nullptr) {
        TransformInspectorMeasurement aMeasurement;
        aMeasurement.state =
            TransformInspectorMeasurementState::Invalid;
        return aMeasurement;
    }
    return _transformInspectorMeasurementController->capture(
        _objectInteractor,
        _shapeInteractor,
        std::move(theCompletion));
}

TransformInspectorPositionCommitResult
Core3DViewer::commitTransformInspectorPosition(
    const TransformInspectorPositionCommitRequest& theRequest,
    std::shared_ptr<NativeModelingCommitPermit> placementPermit) noexcept
{
    if (hasUnresolvedOrdinaryEdit() || !_transformInspectorMeasurementController
        || !_ordinaryEditController) {
        return TransformInspectorPositionCommitResult::Unavailable;
    }
#ifdef DEBUG
    const auto fault = std::exchange(_debugTransformInspectorPositionPublicationFallbackMode, 0);
    if (fault > 0) {
        _debugOrdinaryRepairFailures = 1;
        _debugOrdinaryRedrawFailures = fault == 2 ? 1 : 0;
        _debugOrdinaryOwnerResolutionFailures = fault == 3 ? 1 : 0;
    }
#endif
    return _transformInspectorMeasurementController->commitPosition(
        _objectInteractor, _shapeInteractor, theRequest, _ordinaryEditController,std::move(placementPermit)).result;
}

void Core3DViewer::cancelTransformInspectorMeasurement() noexcept
{
    if (_transformInspectorMeasurementController != nullptr) {
        _transformInspectorMeasurementController
            ->cancelPendingMeasurement();
    }
}

TransformInspectorMeasurementPerformanceState
Core3DViewer::transformInspectorMeasurementPerformanceState() const noexcept
{
    return _transformInspectorMeasurementController == nullptr
        ? TransformInspectorMeasurementPerformanceState()
        : _transformInspectorMeasurementController->performanceState();
}

std::shared_ptr<ObjectInteractor> Core3DViewer::getObjectInteractor() {
    return _objectInteractor;
}

std::shared_ptr<ShapeInteractor> Core3DViewer::getShapeInteractor() {
    return _shapeInteractor;
}

Handle(OcctDocument) Core3DViewer::getDocument() {
    return myDoc;
}

bool Core3DViewer::nativeOpeningReady(std::uint64_t contextIdentifier) const noexcept {
    try {
        if (_objectInteractor == nullptr || _shapeInteractor == nullptr
            || hasUnresolvedOrdinaryEdit()
            || HasActiveOperationLedger(_objectInteractor, _shapeInteractor)
            || myDoc.IsNull() || myDoc->NativeBooleanOwnerBlocksOtherWork()) return false;
        const Handle(TDocStd_Document) document = myDoc->Document();
        if (document.IsNull() || document->HasOpenCommand()) return false;
        if (!_nativeOpeningState) return contextIdentifier == 0;
        const auto& state = *_nativeOpeningState;
        if (state.owner != this || state.activeMarker != 0 || state.recoveryMarker != 0)
            return false;
        return contextIdentifier == 0 ? state.activeContext == 0
                                      : state.activeContext == contextIdentifier;
    } catch (...) { return false; }
}

std::shared_ptr<const native_opening::Fence>
Core3DViewer::recaptureNativeOpeningFence(
    std::uint64_t contextIdentifier, std::uint32_t width, std::uint32_t height,
    const std::vector<std::string>& sourceReceipts) noexcept {
    if (![NSThread isMainThread] || width == 0 || height == 0
        || !nativeOpeningReady(contextIdentifier)) {
#if DEBUG
        // Non-authoritative factory provenance (F2): name the first false
        // branch of the next factory attempt. Never feeds admission.
        std::fprintf(stderr,
            "R179_FACTORY predicate=%s failed=1 context=%llu\n",
            ![NSThread isMainThread] ? "main-thread"
            : (width == 0 || height == 0) ? "viewport" : "nativeOpeningReady",
            (unsigned long long)contextIdentifier);
#endif
        return {};
    }
    try {
        const auto snapshot = captureSceneSnapshot(width, height);
        if (!snapshot || snapshot->publicationSourceIdentifier.empty()
            || !std::isfinite(snapshot->metersPerUnit)
            || snapshot->metersPerUnit <= 0.0) {
#if DEBUG
            std::fprintf(stderr,
                "R179_FACTORY predicate=%s failed=1 context=%llu\n",
                !snapshot ? "snapshot-null" : "snapshot-invalid",
                (unsigned long long)contextIdentifier);
#endif
            return {};
        }
        const Handle(TDocStd_Document) document = myDoc->Document();
        if (document.IsNull() || document->GetData().IsNull()) return {};
        Standard_Real observedUnit = 0.0;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document, observedUnit)
            || observedUnit != snapshot->metersPerUnit) {
#if DEBUG
            std::fprintf(stderr,
                "R179_FACTORY predicate=unit-mismatch failed=1 context=%llu\n",
                (unsigned long long)contextIdentifier);
#endif
            return {};
        }
        const auto stamp = myDoc->CaptureNativePlanningStamp(true);
        if (!stamp) {
#if DEBUG
            std::fprintf(stderr,
                "R179_FACTORY predicate=planning-stamp failed=1 context=%llu\n",
                (unsigned long long)contextIdentifier);
#endif
            return {};
        }
        auto fence = std::shared_ptr<native_opening::Fence>(new native_opening::Fence());
        fence->stamp_ = *stamp;
        fence->publicationSourceIdentifier_ = snapshot->publicationSourceIdentifier;
        fence->documentGeneration_ = snapshot->revisions.documentGeneration;
        fence->modelRevision_ = snapshot->revisions.model;
        fence->selectionMode_ = snapshot->selectionMode;
        fence->selection_ = snapshot->selection.selected;
        fence->hovered_ = snapshot->selection.hovered;
        fence->document_ = document;
        fence->data_ = document->GetData();
        fence->metersPerUnit_ = observedUnit;
        fence->sourceReceipts_ = sourceReceipts;
        return fence;
    } catch (...) { return {}; }
}

std::shared_ptr<native_opening::Context>
Core3DViewer::captureNativeOpeningContext(
    std::uint32_t width, std::uint32_t height,
    const std::vector<std::string>& sourceReceipts) noexcept {
    if (![NSThread isMainThread] || sourceReceipts.size() > 1024) return {};
    std::unordered_set<std::string> unique;
    for (const auto& receipt : sourceReceipts) {
        if (receipt.empty() || receipt.size() > 4096
            || !unique.insert(receipt).second) return {};
    }
    if (!_nativeOpeningState) {
        _nativeOpeningState = std::make_shared<native_opening::detail::State>();
        _nativeOpeningState->owner = this;
    }
    if (!nativeOpeningReady(0)
        || _nativeOpeningState->nextContext == UINT64_MAX) {
#if DEBUG
        std::fprintf(stderr, "R179_FACTORY predicate=%s failed=1 context=0\n",
            !nativeOpeningReady(0) ? "context-capture-ready" : "context-exhausted");
#endif
        return {};
    }
    const std::uint64_t identifier = ++_nativeOpeningState->nextContext;
    _nativeOpeningState->activeContext = identifier;
    const auto fence = recaptureNativeOpeningFence(identifier, width, height, sourceReceipts);
    if (!fence) { _nativeOpeningState->activeContext = 0; return {}; }
    return std::shared_ptr<native_opening::Context>(new native_opening::Context(
        _nativeOpeningState, identifier, *fence));
}

void Core3DViewer::observeNativePlanningInteraction() noexcept {
    if (!myDoc.IsNull()) myDoc->ObserveNativePlanningInteraction();
}

bool Core3DViewer::canBeginCommittedEdit() const noexcept {
    try {
        if (_objectInteractor == nullptr || _shapeInteractor == nullptr
            || hasUnresolvedOrdinaryEdit()
            || HasActiveOperationLedger(
                _objectInteractor, _shapeInteractor)
            || myDoc.IsNull() || myDoc->NativeBooleanOwnerBlocksOtherWork()) {
            return false;
        }
        if (!nativeOpeningReady(0)) return false;
        const Handle(TDocStd_Document) document = myDoc->Document();
        return !document.IsNull() && !document->HasOpenCommand();
    } catch (...) {
        return false;
    }
}

bool Core3DViewer::canBeginCommittedEditHolding(
    const std::shared_ptr<native_opening::Context>& openingContext) const noexcept {
    try {
        if (!openingContext || _objectInteractor == nullptr || _shapeInteractor == nullptr
            || hasUnresolvedOrdinaryEdit()
            || HasActiveOperationLedger(
                _objectInteractor, _shapeInteractor)
            || myDoc.IsNull() || myDoc->NativeBooleanOwnerBlocksOtherWork()) {
            return false;
        }
        const Handle(TDocStd_Document) document = myDoc->Document();
        if (document.IsNull() || document->HasOpenCommand()
            || openingContext->openingFence().document() != document) return false;
        // nativeOpeningReady with the opening's own identifier still requires
        // this viewer to own the opening state, no active/recovery marker and
        // the context to be the active one; a settled or foreign context
        // therefore refuses here.
        return nativeOpeningReady(openingContext->identifier_);
    } catch (...) {
        return false;
    }
}


struct ProfileSolidGeometry : ProfileDefinition {
    std::atomic_bool cancelled{false};
    std::optional<profile::ConstructionFrame> constructionFrame;
    std::vector<profile::ShellStep> shells;
    double shellMetersPerUnit = 0;
    // T-C C4: spline payloads, explicit revolve axis and inner-loop policy for
    // sections carrying ProfileCurveKind::Spline segments. Absent for every
    // legacy polygon/circle/line-arc profile; their bytes and routes are
    // unchanged.
    std::optional<SplineProfileSpec> spline;
    TopoDS_Shape solid;
    std::array<double, 6> bounds{};
    bool built = false;
    std::optional<retained_edge_treatment::Definition> treatmentDefinition;
    TopoDS_Shape treatmentBase;
    std::vector<retained_edge_treatment::StepProof> treatmentProofs;
    retained_edge_treatment::ReplayBudget treatmentBudget;
    // D253 source-edit rebind inputs: main-thread verified value data only.
    std::optional<retained_edge_treatment::SourceRebindRoles> treatmentRebind;
    std::optional<retained_edge_treatment::BaseRecipe> treatmentRebuildSource;
    // D369/O6b: non-owning pointer to the enclosing operation's shared replay
    // budget, set only when this build runs inside a budgeted operation (the
    // admitted R2 Boolean-input base rebuild). Null on initial creation and
    // on every B1 route, which keep the existing budget selection below.
    retained_edge_treatment::ReplayBudget* operationBudget = nullptr;
};


bool BuildProfileSolidGeometry(const std::shared_ptr<ProfileSolidGeometry>& geometry) noexcept {
    if (!geometry || geometry->cancelled.load() || geometry->built) { return false; }
    try {
        OCC_CATCH_SIGNALS
        double signedArea = 0, expected = 0;
        if (geometry->spline) {
            if (!geometry->curves
                || !SplineProfileExpectedVolume(*geometry->curves, *geometry->spline,
                    geometry->plane, geometry->depth, geometry->revolve,
                    [&] { return geometry->cancelled.load(); }, signedArea, expected)) { return false; }
        } else if (!ProfileDefinitionExpectedVolume(*geometry, signedArea, expected)) { return false; }
        TopoDS_Face profileFace;
        if (geometry->spline) {
            SplineProfileFaceResult built;
            if (!BuildSplineProfileFace(*geometry->curves, *geometry->spline, geometry->plane,
                    geometry->cancelled, built)) { return false; }
            profileFace = built.face;
        } else if (geometry->curves) {
            ProfileCurveFaceResult built;
            if (!BuildProfileCurveFace(*geometry->curves,geometry->plane,geometry->cancelled,built)) return false;
            profileFace=built.face;
        } else if (geometry->circle) {
            const auto& circle = *geometry->circle;
            const gp_Pnt center = ProfilePointInPlane(circle.center, geometry->plane);
            // Orient each analytic circle in the same U/V basis as polygons.
            const gp_Dir normal = geometry->plane == 0 ? gp::DZ()
                : (geometry->plane == 1 ? gp_Dir(0, -1, 0) : gp::DX());
            const gp_Dir uDirection = geometry->plane == 2 ? gp::DY() : gp::DX();
            const gp_Ax2 basis(center, normal, uDirection);
            BRepBuilderAPI_MakeEdge outerEdge(gp_Circ(basis, circle.outerRadius));
            if (!outerEdge.IsDone()) { return false; }
            BRepBuilderAPI_MakeWire outerWire(outerEdge.Edge());
            if (!outerWire.IsDone()) { return false; }
            BRepBuilderAPI_MakeFace face(gp_Pln(center, normal), outerWire.Wire(), Standard_True);
            if (!face.IsDone()) { return false; }
            if (circle.innerRadius > 0) {
                BRepBuilderAPI_MakeEdge innerEdge(gp_Circ(basis, circle.innerRadius));
                if (!innerEdge.IsDone()) { return false; }
                BRepBuilderAPI_MakeWire innerWire(innerEdge.Edge());
                if (!innerWire.IsDone()) { return false; }
                face.Add(TopoDS::Wire(innerWire.Wire().Reversed()));
                if (!face.IsDone()) { return false; }
            }
            profileFace = face.Face();
        } else {
            auto points = geometry->points;
            if (signedArea < 0) { std::reverse(points.begin(), points.end()); }
            BRepBuilderAPI_MakePolygon polygon;
            for (const auto& point : points) {
                if (geometry->cancelled.load()) { return false; }
                polygon.Add(ProfilePointInPlane(point, geometry->plane));
            }
            polygon.Close();
            if (!polygon.IsDone()) { return false; }
            BRepBuilderAPI_MakeFace face(polygon.Wire(), Standard_True);
            if (!face.IsDone()) { return false; }
            const gp_Dir normal = geometry->plane == 0 ? gp::DZ()
                : (geometry->plane == 1 ? gp_Dir(0, -1, 0) : gp::DX());
            const gp_Dir uDirection = geometry->plane == 2 ? gp::DY() : gp::DX();
            for (const auto& hole : geometry->holes) {
                if (geometry->cancelled.load()) { return false; }
                const gp_Ax2 basis(ProfilePointInPlane(hole.center, geometry->plane), normal, uDirection);
                BRepBuilderAPI_MakeEdge edge(gp_Circ(basis, hole.radius));
                if (!edge.IsDone()) { return false; }
                BRepBuilderAPI_MakeWire wire(edge.Edge());
                if (!wire.IsDone()) { return false; }
                face.Add(TopoDS::Wire(wire.Wire().Reversed()));
                if (!face.IsDone()) { return false; }
            }
            profileFace = face.Face();
        }
        if (profileFace.IsNull() || geometry->cancelled.load()
            || !BRepCheck_Analyzer(profileFace, Standard_True).IsValid()) { return false; }
        TopoDS_Shape result;
        if (geometry->revolve) {
            gp_Ax1 axis(gp::Origin(), geometry->plane == 0 ? gp::DY() : gp::DZ());
            if (geometry->spline) {
                // The explicit authored axis replaces the legacy implicit one;
                // SplineProfileExpectedVolume has already admitted it.
                if (!geometry->spline->revolveAxis) { return false; }
                axis = SplineRevolveAxis3D(*geometry->spline->revolveAxis, geometry->plane);
                SplineRevolveResult built;
                if (!BuildSplineProfileRevolve(profileFace,
                        *geometry->spline->revolveAxis, geometry->plane,
                        geometry->depth, expected, geometry->cancelled, built)) return false;
                result = built.solid;
            } else if (geometry->depth == 360.0) {
                BRepPrimAPI_MakeRevol sweep(profileFace, axis, Standard_True);
                if (!sweep.IsDone()) { return false; }
                result = sweep.Shape();
            } else {
                BRepPrimAPI_MakeRevol sweep(profileFace, axis,
                    geometry->depth * std::acos(-1.0) / 180.0, Standard_True);
                if (!sweep.IsDone()) { return false; }
                result = sweep.Shape();
            }
        } else if (geometry->spline) {
            TopoDS_Solid built;
            if (!BuildSplineProfileExtrude(profileFace, geometry->plane,
                    geometry->depth, expected, geometry->cancelled, built)) return false;
            result = built;
        } else {
            const gp_Vec direction = geometry->plane == 0 ? gp_Vec(0, 0, geometry->depth)
                : geometry->plane == 1 ? gp_Vec(0, geometry->depth, 0) : gp_Vec(geometry->depth, 0, 0);
            BRepPrimAPI_MakePrism prism(profileFace, direction, Standard_True, Standard_True);
            if (!prism.IsDone()) { return false; }
            result = prism.Shape();
        }
        if (geometry->cancelled.load() || result.IsNull() || result.ShapeType() != TopAbs_SOLID) { return false; }
        if (geometry->constructionFrame) {
            gp_Trsf frame;
            if (!geometry->constructionFrame->Transform(frame)) return false;
            expected *= geometry->constructionFrame->AbsoluteVolumeScale();
            if (!std::isfinite(expected) || expected <= 0 || geometry->cancelled.load()) return false;
            // Only detached worker-owned geometry is transformed. Repeated
            // reflection must never enter OCCT's negative mesh-copy path.
            BRepBuilderAPI_Transform transformed(result, frame, Standard_True, Standard_False);
            if (!transformed.IsDone()) return false;
            result = transformed.Shape();
            if (geometry->cancelled.load() || result.IsNull() || result.ShapeType() != TopAbs_SOLID) return false;
        }
        auto solid = TopoDS::Solid(result);
        if (!BRepLib::OrientClosedSolid(solid) || !BRepCheck_Analyzer(solid, Standard_True).IsValid()) { return false; }
        GProp_GProps properties;
        BRepGProp::VolumeProperties(solid, properties);
        const double volumeTolerance = geometry->spline ? 1e-6 : 1e-8;
        if (!std::isfinite(properties.Mass()) || properties.Mass() <= 0
            || std::abs(properties.Mass() - expected) > std::max(1e-8, expected * volumeTolerance)) { return false; }
        if (geometry->spline && geometry->revolve) {
            int seamFaces = 0;
            if (!ProveSplineRevolveSeam(solid, SplineRevolveAxis3D(*geometry->spline->revolveAxis,
                    geometry->plane), geometry->depth, seamFaces)) { return false; }
        }
        if (!geometry->shells.empty()) {
            profile::Parameters parameters{static_cast<const ProfileDefinition&>(*geometry), geometry->shellMetersPerUnit};
            parameters.constructionFrame = geometry->constructionFrame;
            parameters.shells = geometry->shells;
            TopoDS_Shape shelled;
            if (!ReplayProfileShells(solid, parameters, geometry->cancelled, shelled)
                || shelled.IsNull() || shelled.ShapeType() != TopAbs_SOLID) return false;
            solid = TopoDS::Solid(shelled);
            if (!BRepCheck_Analyzer(solid, Standard_True).IsValid()) return false;
        }
        // D355/D359: a circular-profile prism's supporting cylinder frame is
        // not stable under the bounded binary readback (the frame's local Y
        // direction returns with Z = -0), so the exact detachment commitment
        // at the owner boundary refuses the otherwise valid source. While the
        // new source is still private — before any owner/naming binding or
        // treatment replay — materialize it through the existing bounded
        // readback primitive and adopt the readback form only with an exact
        // fixed-point proof under the existing mesh-independent V3 commitment.
        // A fixed-point failure is never adopted, and not adopting the readback
        // form never by itself refuses a valid source: cancellation and
        // readback/commit budget exhaustion still refuse, and the
        // owner-boundary exact detachment commitment stays the gate that
        // decides. A rebuild participating in B1/B2 debits the operation's
        // existing shared budget before these added traversals and carries its
        // debt forward; initial creation owns a bounded producer budget. No
        // candidate normalization, no digest change and no repeat-until-equal
        // loop.
        if (geometry->circle && !geometry->revolve && !geometry->spline) {
            retained_edge_treatment::ReplayBudget producerBudget;
            // D369/O6b: an explicit operation budget handoff (the admitted R2
            // Boolean-input rebuild) carries the existing debt; it never
            // implies treatmentRebind authority.
            retained_edge_treatment::ReplayBudget& budget =
                geometry->operationBudget ? *geometry->operationBudget :
                geometry->treatmentRebind && geometry->treatmentRebuildSource
                    ? geometry->treatmentBudget : producerBudget;
            TopoDS_Shape canonical, reopened;
            retained_edge_treatment::Digest canonicalDigest{}, reopenedDigest{};
            if (geometry->cancelled.load()) { return false; }
            bool fixedPoint =
                retained_edge_treatment::detail::ReadbackGeometry(solid, budget, canonical);
            if (budget.exhausted) { return false; }
            fixedPoint = fixedPoint && canonical.ShapeType() == TopAbs_SOLID
                && retained_edge_treatment::detail::ReadbackGeometry(canonical, budget, reopened)
                && retained_edge_treatment::detail::CommitGeometry(canonical, budget, canonicalDigest)
                && retained_edge_treatment::detail::CommitGeometry(reopened, budget, reopenedDigest)
                && canonicalDigest == reopenedDigest;
            if (budget.exhausted || geometry->cancelled.load()) { return false; }
            if (fixedPoint) { solid = TopoDS::Solid(canonical); }
        }
        if(geometry->treatmentRebind&&geometry->treatmentRebuildSource){geometry->treatmentBase=solid;retained_edge_treatment::SourceRebindResult rebound;retained_edge_treatment::Refusal refusal=retained_edge_treatment::Refusal::BuildFailed;if(!retained_edge_treatment::ApplySourceRebind(*geometry->treatmentRebind,*geometry->treatmentRebuildSource,solid,geometry->treatmentBudget,refusal,rebound))return false;geometry->treatmentDefinition=rebound.definition;geometry->treatmentProofs=rebound.proofs;solid=TopoDS::Solid(rebound.treated);}
        Bnd_Box bounds;
        BRepBndLib::AddOptimal(solid, bounds, Standard_False, Standard_False);
        if (bounds.IsVoid() || bounds.IsOpen()) { return false; }
        auto& b = geometry->bounds;
        bounds.Get(b[0], b[1], b[2], b[3], b[4], b[5]);
        for (double value : b) { if (!std::isfinite(value) || std::abs(value) > 1e6 + 1e-7) { return false; } }
        if (geometry->cancelled.load()) { return false; }
        geometry->solid = solid;
        geometry->built = true;
        return true;
    } catch (...) { return false; }
}

// Detached source-base rebuild for the admitted R2 Boolean-input edit. The
// envelope carries the proven edited recipe values and the legacy first
// operand; the family switch mirrors buildSavedCutSourceDetached, ending in
// the same source-base boundary proof. Any failure or cancellation refuses.
static bool R2RebuildBooleanInputBase(const retained_solid::Envelope& envelope,
    const std::atomic_bool& stop, retained_edge_treatment::ReplayBudget& budget,
    TopoDS_Shape& out) noexcept {
    out.Nullify();
    try {
        if (stop.load()) return false;
        TopoDS_Shape base;
        if (envelope.sourceFamily == 1) {
            profile::Parameters p;
            if (!profile::Decode(envelope.sourceValues, p)) return false;
            auto geometry = std::make_shared<ProfileSolidGeometry>();
            static_cast<ProfileDefinition&>(*geometry) = p.definition;
            geometry->constructionFrame = p.constructionFrame;
            // D369/O6b: pass this operation's existing debt explicitly into
            // the circular producer's bounded readback/commitment traversals;
            // no treatmentRebind authority is forged to select the budget.
            geometry->operationBudget = &budget;
            if (stop.load() || !BuildProfileSolidGeometry(geometry) || stop.load()) return false;
            base = geometry->solid;
        } else if (envelope.sourceFamily == 2) {
            enclosure::Parameters p; EnclosureSolidResult built;
            if (!enclosure::Decode(int(envelope.sourceSchema), envelope.sourceValues, p) || stop.load()) return false;
            const std::shared_ptr<std::atomic_bool> cancelled(std::shared_ptr<std::atomic_bool>(),
                const_cast<std::atomic_bool*>(&stop));
            if (!BuildEnclosureSolidGeometry(p.definition, cancelled, built) || stop.load()) return false;
            base = built.solid;
            // Live rounded enclosures are published only after the bounded
            // persistence readback reaches an exact commitment fixed point.
            // The R2 input-rebuild seam must produce that same canonical
            // representation before analytic-prefix replay, charged to this
            // operation's one shared budget rather than a fresh counter.
            if (p.definition.dimensions.cornerRadius > 0) {
                TopoDS_Shape canonical, reopened;
                retained_edge_treatment::Digest canonicalDigest{}, reopenedDigest{};
                if (!retained_edge_treatment::detail::ReadbackGeometry(base, budget, canonical)
                    || !retained_edge_treatment::detail::ReadbackGeometry(canonical, budget, reopened)
                    || !retained_edge_treatment::detail::CommitGeometry(canonical, budget, canonicalDigest)
                    || !retained_edge_treatment::detail::CommitGeometry(reopened, budget, reopenedDigest)
                    || canonicalDigest != reopenedDigest) return false;
                base = canonical;
            }
        } else if (envelope.sourceFamily == 3) {
            if (saved_cut_source_edit::RebuildLoftBase(envelope, stop, base)
                != saved_cut_source_edit::LoftBaseStatus::Built) return false;
        } else return false;
        if (stop.load() || base.IsNull()
            || !saved_boolean_build::InspectSourceBase(base, envelope, stop) || stop.load()) return false;
        out = base;
        return true;
    } catch (...) { out.Nullify(); return false; }
}

bool spline_profile::BuildWithDetachedProfileBranch(
    const ProfileCurveSection& section, const SplineProfileSpec& spline,
    int plane, double extent, bool revolve, TopoDS_Solid& solid,
    int& seamFaces) noexcept {
    solid.Nullify(); seamFaces = 0;
    try {
        auto geometry = std::make_shared<ProfileSolidGeometry>();
        geometry->curves = section; geometry->spline = spline;
        geometry->plane = plane; geometry->depth = extent; geometry->revolve = revolve;
        if (!BuildProfileSolidGeometry(geometry) || geometry->solid.IsNull()
            || geometry->solid.ShapeType() != TopAbs_SOLID) return false;
        solid = TopoDS::Solid(geometry->solid);
        if (revolve && (!spline.revolveAxis
            || !ProveSplineRevolveSeam(solid,
                SplineRevolveAxis3D(*spline.revolveAxis, plane), extent, seamFaces))) {
            solid.Nullify(); seamFaces = 0; return false;
        }
        return true;
    } catch (...) { solid.Nullify(); seamFaces = 0; return false; }
}

std::shared_ptr<SavedCutSourceDetachedWork> Core3DViewer::makeSavedCutSourceDetachedWork() noexcept {
    try {
        auto work=std::shared_ptr<SavedCutSourceDetachedWork>(new SavedCutSourceDetachedWork());
        work->profile=std::make_shared<ProfileSolidGeometry>();
        work->stop=std::shared_ptr<std::atomic_bool>(work->profile,&work->profile->cancelled);
        return work;
    }catch(...){return {};}
}
void Core3DViewer::cancelSavedCutSourceDetached(const std::shared_ptr<SavedCutSourceDetachedWork>& work) noexcept {
    if(work&&work->stop)work->stop->store(true);
}
bool Core3DViewer::prepareSavedCutSourceDetached(
    const std::shared_ptr<SavedCutSourceDetachedWork>& work,const CylindricalCutSnapshot& original,
    const saved_cut_source_edit::Patch& patch,const ObjectFrameIdentity& identity,
    std::uint64_t presentation,std::uint32_t width,std::uint32_t height) noexcept {
    using Phase=SavedCutSourceDetachedWork::Phase;
    if(!work||!work->stop||!NSThread.isMainThread)return false;
    auto fresh=Phase::Fresh;
    if(!work->phase.compare_exchange_strong(fresh,Phase::Preparing))return false;
    const auto refuse=[&](){work->phase.store(Phase::Refused);return false;};
    try {
        const auto& stop=*work->stop;
        if(stop.load()||!original.source.rebuilding||!original.source.original.retained.value
            ||identity.entityIdentifier!=original.identity.entityIdentifier
            ||identity.publicationSourceIdentifier!=original.identity.publicationSourceIdentifier
            ||identity.documentGeneration!=original.identity.documentGeneration
            ||identity.modelRevision!=original.identity.modelRevision)return refuse();
        const auto current=cylindricalCutSource(identity,presentation,width,height);
        if(stop.load()||!current||!current->source.rebuilding
            ||!(current->authorityStamp==original.authorityStamp)
            ||!current->source.original.IsEqual(original.source.original)
            ||!sweep_rebuild::SameRawScalars(current->source.original.scalars,original.source.original.scalars)
            ||current->source.effectiveMM!=original.source.effectiveMM
            ||!myDoc->SavedCutSceneStateMatches(original.guard))return refuse();
        myContext->InitSelected();
        const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        if(selected.IsNull()||!cut_display::Capture(selected->Attributes(),work->displaySettings))return refuse();
        work->displayCaptured=true;
        const auto& retained=*original.source.original.retained.value;
        std::vector<std::uint8_t> originalEnvelope;
        if(stop.load()||!retained_solid::Encode(original.source.envelope,originalEnvelope)
            ||originalEnvelope!=retained.bytes||!retained.base.IsEqual(original.source.base)
            ||!saved_cut_source_edit::PrepareValues(retained,patch,stop,work->values)
            ||stop.load())return refuse();
        const auto& base=original.source.base;
        const auto& result=original.source.original.shape;
        // Source guard already binds exact live content. Perform classifiers'
        // raw location/representation admission before our cumulative traversal
        // or stream/copy. Final full-scene check also covers these inspections.
        if(!saved_boolean_build::InspectSourceBase(base,work->values.oldEnvelope,stop)
            ||saved_boolean_result::InspectAxialBore(result,work->values.oldEnvelope,work->values.oldEnvelope,stop).classification
                !=saved_cut_whole_result::Classification::MatchedOrientedBoundary
            ||stop.load())return refuse();
        // Re-charge occurrence and stream budgets before any geometry copy.
        if(!analytic_boolean::detail::Bounded(base,1024,stop)
            ||!analytic_boolean::detail::Bounded(result,32768,stop)
            ||!saved_cut_source_edit::Commit(base,stop,work->streamBytes,work->sourceBase)
            ||!saved_cut_source_edit::Commit(result,stop,work->streamBytes,work->sourceResult))return refuse();
        // Copy only after complete admitted original geometry, preserving meshes
        // for content evidence. No shared live labels/materials reach the worker.
        BRepBuilderAPI_Copy baseCopy(base,Standard_True,Standard_True);
        if(stop.load()||!baseCopy.IsDone()||baseCopy.Shape().IsNull()
            ||baseCopy.Shape().IsPartner(base))return refuse();
        BRepBuilderAPI_Copy resultCopy(result,Standard_True,Standard_True);
        if(stop.load()||!resultCopy.IsDone()||resultCopy.Shape().IsNull()
            ||resultCopy.Shape().IsPartner(result))return refuse();
        work->oldBase=baseCopy.Shape();work->oldResult=resultCopy.Shape();
        if(!saved_boolean_build::InspectSourceBase(work->oldBase,work->values.oldEnvelope,stop)
            ||saved_boolean_result::InspectAxialBore(work->oldResult,work->values.oldEnvelope,work->values.oldEnvelope,stop).classification
                !=saved_cut_whole_result::Classification::MatchedOrientedBoundary
            ||!saved_cut_source_edit::Commit(work->oldBase,stop,work->streamBytes,work->privateBase)
            ||!saved_cut_source_edit::Commit(work->oldResult,stop,work->streamBytes,work->privateResult))return refuse();
        saved_cut_source_edit::ShapeCommitment afterBase,afterResult;
        if(!saved_cut_source_edit::Commit(base,stop,work->streamBytes,afterBase)
            ||!saved_cut_source_edit::Commit(result,stop,work->streamBytes,afterResult)
            ||!(afterBase==work->sourceBase)||!(afterResult==work->sourceResult))return refuse();
        cut_display::Settings afterDisplay;
        if(!cut_display::Capture(selected->Attributes(),afterDisplay)
            ||!(afterDisplay==work->displaySettings))return refuse();
        // No root, snapshot, currentness stamp or guard is retained by work.
        // Future ordinary owner must preserve them and recheck before dispatch.
        const auto after=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        if(stop.load()||!after||!(*after==original.authorityStamp)
            ||!myDoc->SavedCutSceneStateMatches(original.guard))return refuse();
        work->phase.store(Phase::Prepared,std::memory_order_release);return true;
    }catch(...){return refuse();}
}
std::shared_ptr<const SavedCutSourceDetachedResult> Core3DViewer::buildSavedCutSourceDetached(
    const std::shared_ptr<SavedCutSourceDetachedWork>& work) noexcept {
    using Phase=SavedCutSourceDetachedWork::Phase;
    if(!work||!work->stop)return {};
    auto prepared=Phase::Prepared;
    if(!work->phase.compare_exchange_strong(prepared,Phase::Building,std::memory_order_acquire))return {};
    const auto refuse=[&]()->std::shared_ptr<const SavedCutSourceDetachedResult>{
        work->phase.store(Phase::Refused,std::memory_order_release);return {};
    };
    try {
        const auto& stop=*work->stop;
        saved_cut_source_edit::ShapeCommitment beforeBase,beforeResult;
        if(stop.load()||!work->displayCaptured||!saved_cut_source_edit::Commit(work->oldBase,stop,work->streamBytes,beforeBase)
            ||!saved_cut_source_edit::Commit(work->oldResult,stop,work->streamBytes,beforeResult)
            ||!(beforeBase==work->privateBase)||!(beforeResult==work->privateResult)
            ||!saved_boolean_build::InspectSourceBase(work->oldBase,work->values.oldEnvelope,stop)
            ||saved_boolean_result::InspectAxialBore(work->oldResult,work->values.oldEnvelope,work->values.oldEnvelope,stop).classification
                !=saved_cut_whole_result::Classification::MatchedOrientedBoundary)return refuse();
        auto output=std::shared_ptr<SavedCutSourceDetachedResult>(new SavedCutSourceDetachedResult());
        output->values=work->values;output->sourceBase=work->sourceBase;output->sourceResult=work->sourceResult;
        output->privateOldBase=work->privateBase;output->privateOldResult=work->privateResult;
        output->noChange=!work->values.changed;
        output->displaySettings=work->displaySettings;output->originatingWork=work;
        if(work->values.changed){
            const auto& e=work->values.newEnvelope;
            TopoDS_Shape base;
            if(e.sourceFamily==1){
                profile::Parameters p;if(!profile::Decode(e.sourceValues,p))return refuse();
                static_cast<ProfileDefinition&>(*work->profile)=p.definition;
                work->profile->constructionFrame=p.constructionFrame;
                if(stop.load()||!BuildProfileSolidGeometry(work->profile))return refuse();
                base=work->profile->solid;
            }else if(e.sourceFamily==2){
                enclosure::Parameters p;EnclosureSolidResult built;
                if(!enclosure::Decode(int(e.sourceSchema),e.sourceValues,p)||stop.load()
                    ||!BuildEnclosureSolidGeometry(p.definition,work->stop,built))return refuse();
                base=built.solid;
            }else if(e.sourceFamily==3){
                if(saved_cut_source_edit::RebuildLoftBase(e,stop,base)!=saved_cut_source_edit::LoftBaseStatus::Built)return refuse();
            }else return refuse();
            if(stop.load()||!saved_boolean_build::InspectSourceBase(base,e,stop)
                ||!saved_cut_source_edit::Commit(base,stop,work->streamBytes,output->generatedBase))return refuse();
            analytic_boolean::Result cut;
            if(analytic_boolean::Build(base,cylindrical_cut::Recipe(e),stop,cut)!=analytic_boolean::Status::Built
                // Existing radius presentation preparation mutates only this
                // detached result. Classify and commit the final meshed stream.
                ||!cut_display::Prepare(cut.solid,work->displaySettings,stop)
                ||saved_boolean_result::InspectAxialBore(cut.solid,work->values.oldEnvelope,e,stop).classification
                    !=saved_cut_whole_result::Classification::MatchedOrientedBoundary)return refuse();
            saved_cut_source_edit::ShapeCommitment afterBase;
            if(stop.load()||!saved_cut_source_edit::Commit(base,stop,work->streamBytes,afterBase)
                ||!(afterBase==output->generatedBase)
                ||!saved_cut_source_edit::Commit(cut.solid,stop,work->streamBytes,output->generatedResult))return refuse();
            output->newBase=base;output->newResult=cut.solid;output->cut=std::move(cut);
        }else{
            // Informational exact no-op, not an OrdinaryEditResult or receipt.
            // No replacement shape is exposed or fabricated.
            output->generatedBase=output->sourceBase;output->generatedResult=output->sourceResult;
        }
        saved_cut_source_edit::ShapeCommitment finalOldBase,finalOldResult;
        if(!saved_cut_source_edit::Commit(work->oldBase,stop,work->streamBytes,finalOldBase)
            ||!saved_cut_source_edit::Commit(work->oldResult,stop,work->streamBytes,finalOldResult)
            ||!(finalOldBase==work->privateBase)||!(finalOldResult==work->privateResult)||stop.load())return refuse();
        work->phase.store(Phase::Finished,std::memory_order_release);return output;
    }catch(...){return refuse();}
}

struct EnclosureSolidGeometry {
    enclosure::Parameters parameters;
    std::shared_ptr<std::atomic_bool> cancelled = std::make_shared<std::atomic_bool>(false);
    EnclosureSolidResult result;
    bool built = false;
    std::optional<retained_edge_treatment::Definition> treatmentDefinition;
    TopoDS_Shape treatmentBase;
    std::vector<retained_edge_treatment::StepProof> treatmentProofs;
    retained_edge_treatment::ReplayBudget treatmentBudget;
    // D253 source-edit rebind inputs: main-thread verified value data only.
    std::optional<retained_edge_treatment::SourceRebindRoles> treatmentRebind;
    std::optional<retained_edge_treatment::BaseRecipe> treatmentRebuildSource;
};
struct CutSolidGeometry {
    cut_display::Settings displaySettings;
    TopoDS_Shape detachedBase;
    analytic_boolean::Recipe recipe;
    std::optional<retained_solid::Envelope> provenHost;
    bool rebuildLoftBase=false;
    saved_cut_source_edit::LoftBaseStatus loftBaseStatus=saved_cut_source_edit::LoftBaseStatus::InvalidRecipe;
    std::atomic_bool cancelled{false};
    analytic_boolean::Result result;
    bool built=false;
};
// Whole-program payload: detached deep base copy, the complete typed program
// and the exact expected recipe bytes. No live label, material or AIS handle.
struct CutProgramGeometry {
    cut_display::Settings displaySettings;
    TopoDS_Shape detachedBase;
    retained_boolean::Program program;
    std::vector<std::uint8_t> expectedBytes;
    std::uint32_t selectedOperandID=0;
    std::atomic_bool cancelled{false};
    saved_boolean_build::Result result;
    bool built=false;
};
struct LoftSolidGeometry {
    std::shared_ptr<const rectangular_loft::Prepared> prepared;
    std::atomic_bool cancelled{false};
    rectangular_loft::SolidResult result;
    bool built=false;
    std::optional<retained_edge_treatment::Definition> treatmentDefinition;
    TopoDS_Shape treatmentBase;
    std::vector<retained_edge_treatment::StepProof> treatmentProofs;
    retained_edge_treatment::ReplayBudget treatmentBudget;
    // D253 source-edit rebind inputs: main-thread verified value data only.
    std::optional<retained_edge_treatment::SourceRebindRoles> treatmentRebind;
    std::optional<retained_edge_treatment::BaseRecipe> treatmentRebuildSource;
};
struct SweepSolidGeometry {
    std::shared_ptr<const planar_sweep::Prepared> prepared;
    std::atomic_bool cancelled{false};
    planar_sweep::SolidResult result;
    bool built=false;
};

struct AssemblySolidGeometry {
    std::vector<std::shared_ptr<ProfileSolidGeometry>> parts;
    std::atomic_bool cancelled{false};
    std::array<double,6> bounds{};
    bool built = false;
};

static bool BuildAssemblySolidGeometry(const std::shared_ptr<AssemblySolidGeometry>& geometry) noexcept {
    if (!geometry || geometry->built || geometry->cancelled.load()
        || geometry->parts.empty() || geometry->parts.size() > 16) return false;
    try {
        bool first = true;
        for (const auto& part : geometry->parts) {
            if (geometry->cancelled.load() || !BuildProfileSolidGeometry(part)) return false;
            if (first) { geometry->bounds = part->bounds; first = false; }
            else for (int axis=0; axis<3; ++axis) {
                geometry->bounds[axis] = std::min(geometry->bounds[axis], part->bounds[axis]);
                geometry->bounds[axis+3] = std::max(geometry->bounds[axis+3], part->bounds[axis+3]);
            }
        }
        if (geometry->cancelled.load()) return false;
        geometry->built = true; return true;
    } catch (...) { return false; }
}

struct CompletedNativeSolid {
    TopoDS_Shape solid;
    std::array<double,6> bounds;
};
std::optional<CompletedNativeSolid> CompletedNativeSolidFor(const NativeSolidGeometryPayload& payload) noexcept {
    if (const auto p = std::get_if<std::shared_ptr<ProfileSolidGeometry>>(&payload)) {
        if (*p && (*p)->built && !(*p)->cancelled.load() && !(*p)->solid.IsNull())
            return CompletedNativeSolid{(*p)->solid,(*p)->bounds};
    } else if (const auto p = std::get_if<std::shared_ptr<EnclosureSolidGeometry>>(&payload)) {
        if (*p && (*p)->built && (*p)->cancelled && !(*p)->cancelled->load() && !(*p)->result.solid.IsNull())
            return CompletedNativeSolid{(*p)->result.solid,(*p)->result.bounds};
    }
    if(const auto p=std::get_if<std::shared_ptr<CutSolidGeometry>>(&payload)) {
        if(*p&&(*p)->built&&!(*p)->cancelled.load()&&!(*p)->result.solid.IsNull())
            return CompletedNativeSolid{(*p)->result.solid,(*p)->result.resultBounds};
    }
    if(const auto p=std::get_if<std::shared_ptr<CutProgramGeometry>>(&payload)) {
        if(*p&&(*p)->built&&!(*p)->cancelled.load()&&!(*p)->result.solid.IsNull()){
            std::array<double,6> bounds{};Bnd_Box box;
            try{BRepBndLib::Add((*p)->result.solid,box);}catch(...){return {};}
            if(box.IsVoid()||box.IsWhole())return {};
            box.Get(bounds[0],bounds[1],bounds[2],bounds[3],bounds[4],bounds[5]);
            return CompletedNativeSolid{(*p)->result.solid,bounds};
        }
    }
    if (const auto p=std::get_if<std::shared_ptr<LoftSolidGeometry>>(&payload)) {
        if (*p && (*p)->built && !(*p)->cancelled.load() && !(*p)->result.solid.IsNull())
            return CompletedNativeSolid{(*p)->result.solid,(*p)->result.bounds};
    }
    if (const auto p=std::get_if<std::shared_ptr<SweepSolidGeometry>>(&payload)) {
        if (*p && (*p)->built && !(*p)->cancelled.load() && !(*p)->result.solid.IsNull())
            return CompletedNativeSolid{(*p)->result.solid,(*p)->result.bounds};
    }
    return {};
}

struct NativePBRScalarWork {
    std::shared_ptr<const OcctPBRScalarPreparation> prepared;
    OrdinaryNameLedger authority;Handle(OcctDocument) owner;Handle(TDocStd_Document) document;
    ObjectFrameIdentity identity;std::uint64_t presentationRevision=0;std::uint32_t width=0,height=0;
    core3d::authority::Stamp stamp;Standard_Integer documentTime=0;bool consumed=false,changed=false;
};
std::shared_ptr<NativePBRScalarWork> Core3DViewer::preparePBRScalar(
    const OcctPBRScalarPatch& patch,const ObjectFrameIdentity& identity,std::uint64_t presentation,
    std::uint32_t width,std::uint32_t height) noexcept {
    if(![NSThread isMainThread]||!canBeginCommittedEdit()||myDoc.IsNull()||myContext.IsNull()||!width||!height)return {};
    try{
        const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        const auto snapshot=captureSceneSnapshot(width,height);
        if(!stamp||!snapshot||snapshot->selectionMode!=scene::ElementKind::Object||snapshot->publicationSourceIdentifier!=identity.publicationSourceIdentifier
            ||snapshot->revisions.documentGeneration!=identity.documentGeneration||snapshot->revisions.model!=identity.modelRevision
            ||snapshot->revisions.presentation!=presentation)return {};
        myContext->InitSelected();if(!myContext->MoreSelected())return {};const auto selected=myContext->SelectedInteractive();myContext->NextSelected();
        if(myContext->MoreSelected())return {};const auto label=myDoc->ShapeLabel(selected);
        if(label.IsNull())return {};
        auto work=std::make_shared<NativePBRScalarWork>();work->owner=myDoc;work->document=myDoc->Document();
        if(work->document.IsNull()||!admitNames(work->authority)||work->authority.selectedPresentations.size()!=1)return {};
        work->identity=identity;work->identity.entityIdentifier=myDoc->EntityIdentifierForLabel(label);
        work->stamp=*stamp;work->presentationRevision=presentation;work->width=width;work->height=height;work->documentTime=work->document->GetData()->Time();
        work->prepared=myDoc->PreparePBRScalarPatch(label,patch,work->changed);
        const auto after=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        if(!work->prepared||!after||!(*after==work->stamp)||!myDoc->PBRScalarStateMatches(myDoc->PBRScalarOriginal(work->prepared)))return {};
        return work;
    }catch(...){return {};}
}
void Core3DViewer::cancelPBRScalar(const std::shared_ptr<NativePBRScalarWork>& work) noexcept {
    if([NSThread isMainThread]&&work&&work->owner==myDoc)work->consumed=true;
}
OrdinaryEditResult Core3DViewer::executePBRScalar(const std::shared_ptr<NativePBRScalarWork>& work) noexcept {
    if(![NSThread isMainThread]||!work||work->owner!=myDoc||work->consumed)return OrdinaryEditResult::Invalid;
    work->consumed=true;
    if(!canBeginCommittedEdit()||!_ordinaryEditController)return OrdinaryEditResult::Busy;
    try{
        const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        const auto snapshot=captureSceneSnapshot(work->width,work->height);
        if(!stamp||!(*stamp==work->stamp)||!snapshot||myDoc->Document()!=work->document||work->document->GetData()->Time()!=work->documentTime
            ||snapshot->publicationSourceIdentifier!=work->identity.publicationSourceIdentifier
            ||snapshot->revisions.documentGeneration!=work->identity.documentGeneration||snapshot->revisions.model!=work->identity.modelRevision
            ||snapshot->revisions.presentation!=work->presentationRevision||!_objectInteractor||!_shapeInteractor
            ||!_shapeInteractor->selectionModeAuthorityIsExact()||_shapeInteractor->getSelectionMode()!=work->authority.selectionMode
            ||!_objectInteractor->verifyOrdinaryNameAuthority(work->authority)
            ||!myDoc->PBRScalarStateMatches(myDoc->PBRScalarOriginal(work->prepared)))return OrdinaryEditResult::Invalid;
        if(!work->changed)return OrdinaryEditResult::NoChange;
        OrdinaryEditResult result=OrdinaryEditResult::Invalid;auto lease=_ordinaryEditController->beginAppearance(work->prepared,&result);
        if(lease)result=lease.stageAndCommit();return result;
    }catch(...){return OrdinaryEditResult::OutcomeUnknown;}
}
bool Core3DViewer::admitAppearance(OrdinaryAppearanceLedger& ledger) noexcept {
    if(!admitNames(ledger.authority)||ledger.authority.selectedPresentations.size()!=1)return false;
    const auto target=myDoc->PBRScalarTarget(ledger.prepared);
    return !target.IsNull()&&myDoc->ShapeLabel(ledger.authority.selectedPresentations.front().presentation).IsEqual(target);
}
bool Core3DViewer::repairAppearance(const OrdinaryAppearanceLedger& ledger,bool committed) noexcept {
    if(!repairNames(ledger.authority,committed))return false;
    try{
        const auto target=myDoc->PBRScalarTarget(ledger.prepared);
        const auto shape=ledger.authority.selectedPresentations.front().presentation;
        if(shape.IsNull()||target.IsNull())return false;
        shape->UnsetColor();myDoc->LoadObjectMeterial(target,shape);
        myContext->UpdateCurrentViewer();
        return repairNames(ledger.authority,committed);
    }catch(...){return false;}
}

struct NativeSolidWork {
    std::shared_ptr<NativeModelingCommitPermit> modelingPermit; // Main only; excluded from numeric geometry payload.
    NativeSolidGeometryPayload geometry;
    OrdinaryNameLedger authority;
    std::string sweepIdentifier; // Generated once on main, never from a provider or worker.
    std::string loftIdentifier; // Native feature UUID; distinct from local numeric loftIdentifier.
    std::optional<authority::Stamp> sweepRebuildStamp; // Main only, fences selection/tool ABA.
    std::optional<authority::Stamp> loftRebuildStamp;
    std::optional<authority::Stamp> cutStamp; // Same saved-feature authority, no receipt route.
    std::vector<AssemblyPartDefinition> assemblyParts; // Main-owned metadata, never worker payload.
    std::optional<OrdinaryTransformLedger> rebuildAuthority;
    ObjectFrameIdentity identity;
    Handle(OcctDocument) owner;
    Handle(TDocStd_Document) document;
    std::uint64_t presentationRevision = 0;
    std::uint32_t width = 0, height = 0;
    Standard_Integer documentTime = 0;
    double metersPerUnit = 0;
    bool frameFirst = false;
    bool consumed = false;
    std::shared_ptr<const retained_edge_treatment::Snapshot> edgeTreatmentSnapshot;
    std::optional<retained_edge_treatment::Edit> edgeTreatmentEdit;
};

// Thread-safe cancellation owns only private detached geometry and atomics.
// Destroying the final token on a worker cannot release any AIS/document handle.
class SavedCutSourceEditCancellation final {
    friend class Core3DViewer;
    enum class Decision { Pending, Cancelled, Committing, Settled };
    std::atomic<Decision> decision{Decision::Pending};
    const std::shared_ptr<SavedCutSourceDetachedWork> geometry;
    explicit SavedCutSourceEditCancellation(std::shared_ptr<SavedCutSourceDetachedWork> work):geometry(std::move(work)){}
public:
    SavedCutSourceEditCancellation(const SavedCutSourceEditCancellation&)=delete;
    SavedCutSourceEditCancellation& operator=(const SavedCutSourceEditCancellation&)=delete;
};
// Main-thread lease. Worker callers receive only detachedGeometry, never this
// document/selection/presentation authority. The slot is weak on the Viewer.
class SavedCutSourceEditWork final {
    friend class Core3DViewer;
    enum class Phase { Fresh, Preparing, Prepared, Settled };
    Phase phase=Phase::Fresh; // Main thread only.
    const std::shared_ptr<SavedCutSourceEditCancellation> cancellation;
    const std::shared_ptr<SavedCutSourceDetachedWork> detachedGeometry;
    std::shared_ptr<NativeSolidWork> authority; // Main thread only.
    std::weak_ptr<OrdinaryEditController> ordinaryOwner;
    Handle(AIS_InteractiveContext) context;
    SavedCutSourceEditWork(std::shared_ptr<SavedCutSourceDetachedWork> geometry,
        std::shared_ptr<SavedCutSourceEditCancellation> token)
        :cancellation(std::move(token)),detachedGeometry(std::move(geometry)){}
public:
    ~SavedCutSourceEditWork(){(void)Core3DViewer::cancelSavedCutSourceEdit(cancellation);}
    SavedCutSourceEditWork(const SavedCutSourceEditWork&)=delete;
    SavedCutSourceEditWork& operator=(const SavedCutSourceEditWork&)=delete;
};

// Thread-safe cancellation owns only private detached geometry and atomics.
// Destroying the final token on a worker cannot release any AIS/document handle.
class SavedProgramSourceEditCancellation final {
    friend class Core3DViewer;
    enum class Decision { Pending, Cancelled, Committing, Settled };
    std::atomic<Decision> decision{Decision::Pending};
    const std::shared_ptr<SavedProgramSourceDetachedWork> geometry;
    explicit SavedProgramSourceEditCancellation(std::shared_ptr<SavedProgramSourceDetachedWork> work):geometry(std::move(work)){}
public:
    SavedProgramSourceEditCancellation(const SavedProgramSourceEditCancellation&)=delete;
    SavedProgramSourceEditCancellation& operator=(const SavedProgramSourceEditCancellation&)=delete;
};
// Main-thread whole-program lease. Worker callers receive only detachedGeometry,
// never this document/selection/presentation authority. Weak slot on Viewer.
class SavedProgramSourceEditWork final {
    friend class Core3DViewer;
    enum class Phase { Fresh, Preparing, Prepared, Settled };
    Phase phase=Phase::Fresh; // Main thread only.
    const std::shared_ptr<SavedProgramSourceEditCancellation> cancellation;
    const std::shared_ptr<SavedProgramSourceDetachedWork> detachedGeometry;
    std::shared_ptr<NativeSolidWork> authority; // Main thread only.
    std::weak_ptr<OrdinaryEditController> ordinaryOwner;
    Handle(AIS_InteractiveContext) context;
    SavedProgramSourceEditWork(std::shared_ptr<SavedProgramSourceDetachedWork> geometry,
        std::shared_ptr<SavedProgramSourceEditCancellation> token)
        :cancellation(std::move(token)),detachedGeometry(std::move(geometry)){}
public:
    ~SavedProgramSourceEditWork(){(void)Core3DViewer::cancelSavedProgramSourceEdit(cancellation);}
    SavedProgramSourceEditWork(const SavedProgramSourceEditWork&)=delete;
    SavedProgramSourceEditWork& operator=(const SavedProgramSourceEditWork&)=delete;
};

std::shared_ptr<SavedCutSourceEditWork> Core3DViewer::makeSavedCutSourceEditWork() noexcept {
    if(!NSThread.isMainThread)return {};
    try {
        auto geometry=makeSavedCutSourceDetachedWork();if(!geometry)return {};
        auto token=std::shared_ptr<SavedCutSourceEditCancellation>(new SavedCutSourceEditCancellation(geometry));
        return std::shared_ptr<SavedCutSourceEditWork>(new SavedCutSourceEditWork(std::move(geometry),std::move(token)));
    }catch(...){return {};}
}
std::shared_ptr<SavedCutSourceEditCancellation> Core3DViewer::savedCutSourceEditCancellation(
    const std::shared_ptr<SavedCutSourceEditWork>& work) noexcept {
    if(!NSThread.isMainThread||!work)return {};
    return work->cancellation;
}
bool Core3DViewer::cancelSavedCutSourceEdit(const std::shared_ptr<SavedCutSourceEditCancellation>& token) noexcept {
    if(!token)return false;
    using Decision=SavedCutSourceEditCancellation::Decision;
    auto pending=Decision::Pending;
    if(!token->decision.compare_exchange_strong(pending,Decision::Cancelled))return pending==Decision::Cancelled;
    cancelSavedCutSourceDetached(token->geometry);return true;
}
bool Core3DViewer::discardSavedCutSourceEdit(const std::shared_ptr<SavedCutSourceEditWork>& work) noexcept {
    if(!NSThread.isMainThread||!work||_savedCutSourceEditWork.lock()!=work
        ||work->phase==SavedCutSourceEditWork::Phase::Settled
        ||!cancelSavedCutSourceEdit(work->cancellation))return false;
    work->authority.reset();work->ordinaryOwner.reset();work->context.Nullify();
    work->phase=SavedCutSourceEditWork::Phase::Settled;
    work->cancellation->decision.store(SavedCutSourceEditCancellation::Decision::Settled);
    _savedCutSourceEditWork.reset();return true;
}
std::shared_ptr<SavedCutSourceDetachedWork> Core3DViewer::savedCutSourceEditGeometry(
    const std::shared_ptr<SavedCutSourceEditWork>& work) noexcept {
    if(!NSThread.isMainThread||!work||work->phase!=SavedCutSourceEditWork::Phase::Prepared
        ||work->cancellation->decision.load()!=SavedCutSourceEditCancellation::Decision::Pending)return {};
    return work->detachedGeometry;
}
bool Core3DViewer::prepareSavedCutSourceEdit(const std::shared_ptr<SavedCutSourceEditWork>& work,
    const CylindricalCutSnapshot& original,const saved_cut_source_edit::Patch& patch,
    const ObjectFrameIdentity& identity,std::uint64_t presentation,std::uint32_t width,std::uint32_t height) noexcept {
    using Phase=SavedCutSourceEditWork::Phase;using Decision=SavedCutSourceEditCancellation::Decision;
    if(!NSThread.isMainThread||!work||work->phase!=Phase::Fresh)return false;
    work->phase=Phase::Preparing;
    const auto refuse=[&](){
        (void)cancelSavedCutSourceEdit(work->cancellation);work->authority.reset();work->ordinaryOwner.reset();work->context.Nullify();
        work->phase=Phase::Settled;work->cancellation->decision.store(Decision::Settled);
        if(_savedCutSourceEditWork.lock()==work)_savedCutSourceEditWork.reset();return false;
    };
    try {
        if(work->cancellation->decision.load()!=Decision::Pending||!canBeginCommittedEdit()||!_ordinaryEditController)return refuse();
        // The whole-program family shares this one-lease discipline: a live
        // program lease refuses here instead of stacking a second source job.
        if(const auto foreign=_savedProgramSourceEditWork.lock()){
            if(foreign->phase==SavedProgramSourceEditWork::Phase::Settled)_savedProgramSourceEditWork.reset();
            else return refuse();
        }
        // Context/document reset need not call release. A retained cancelled or
        // demonstrably stale old lease must not occupy the next source slot.
        if(const auto old=_savedCutSourceEditWork.lock()){
            if(old->phase==Phase::Settled)_savedCutSourceEditWork.reset();
            else {
                if(old->phase!=Phase::Prepared||old->cancellation->decision.load()==Decision::Committing)return refuse();
                const auto a=old->authority;
                bool stale=old->cancellation->decision.load()==Decision::Cancelled||!a||!a->cutStamp
                    ||!a->rebuildAuthority||a->rebuildAuthority->records.size()!=1
                    ||old->ordinaryOwner.lock()!=_ordinaryEditController||old->context!=myContext
                    ||myDoc!=a->owner||myDoc.IsNull()||myDoc->Document()!=a->document
                    ||a->document.IsNull()||a->document->GetData()->Time()!=a->documentTime;
                if(!stale){
                    const auto current=cylindricalCutSource(a->identity,a->presentationRevision,a->width,a->height);
                    const auto& record=a->rebuildAuthority->records.front();cut_display::Settings settings;
                    auto admitted=*a->rebuildAuthority;
                    stale=!current||!(current->authorityStamp==*a->cutStamp)
                        ||!current->source.original.IsEqual(record.previous)
                        ||!myDoc->SavedCutSceneStateMatches(record.requested.cutSource)
                        ||!_objectInteractor||!_objectInteractor->verifyOrdinaryNameAuthority(a->authority)
                        ||!admitTransform(admitted)||admitted.selectionOwners!=a->rebuildAuthority->selectionOwners
                        ||admitted.manipulatorType!=a->rebuildAuthority->manipulatorType
                        ||admitted.hadManipulator!=a->rebuildAuthority->hadManipulator
                        ||record.requested.presentation.IsNull()
                        ||!cut_display::Capture(record.requested.presentation->Attributes(),settings)
                        ||!(settings==old->detachedGeometry->displaySettings);
                }
                if(!stale||!discardSavedCutSourceEdit(old))return refuse();
            }
        }
        _savedCutSourceEditWork=work;work->ordinaryOwner=_ordinaryEditController;work->context=myContext;
        if(!prepareSavedCutSourceDetached(work->detachedGeometry,original,patch,identity,presentation,width,height)
            ||work->cancellation->decision.load()!=Decision::Pending)return refuse();
        auto authority=prepareNativeSolidWork(identity,presentation,width,height);
        if(!authority||retained_solid::Bits(authority->metersPerUnit)!=retained_solid::Bits(original.source.envelope.metersPerUnit)
            ||authority->modelingPermit||authority->rebuildAuthority)return refuse();
        myContext->InitSelected();if(!myContext->MoreSelected())return refuse();
        const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
        if(selected.IsNull()||myContext->MoreSelected())return refuse();
        OrdinaryTransformRecord record;record.previous=original.source.original;
        record.requested.label=record.previous.label;record.requested.presentation=selected;
        record.requested.shape=record.previous.shape;record.requested.transform=record.previous.transform;
        record.requested.operation=OrdinaryTransformOperation::CylindricalCutSourceRebuild;
        record.requested.cutSourcePatch=patch;record.requested.cutSource=original.guard;
        authority->rebuildAuthority.emplace();authority->rebuildAuthority->records.push_back(std::move(record));
        if(!admitTransform(*authority->rebuildAuthority))return refuse();
        const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        cut_display::Settings settings;
        if(!stamp||!(*stamp==original.authorityStamp)||!myDoc->SavedCutSceneStateMatches(original.guard)
            ||!cut_display::Capture(selected->Attributes(),settings)
            ||!(settings==work->detachedGeometry->displaySettings)
            ||work->ordinaryOwner.lock()!=_ordinaryEditController||work->context!=myContext
            ||work->cancellation->decision.load()!=Decision::Pending)return refuse();
        authority->cutStamp=original.authorityStamp;authority->frameFirst=false;
        work->authority=std::move(authority);work->phase=Phase::Prepared;return true;
    }catch(...){return refuse();}
}
OrdinaryEditResult Core3DViewer::commitSavedCutSourceEdit(const std::shared_ptr<SavedCutSourceEditWork>& work,
    const std::shared_ptr<const SavedCutSourceDetachedResult>& built) noexcept {
    using Phase=SavedCutSourceEditWork::Phase;using Decision=SavedCutSourceEditCancellation::Decision;
    if(!NSThread.isMainThread||!work||work->phase!=Phase::Prepared
        ||_savedCutSourceEditWork.lock()!=work)return OrdinaryEditResult::Invalid;
    const auto finish=[&](OrdinaryEditResult result){
        if(work->cancellation->decision.load()!=Decision::Committing)(void)cancelSavedCutSourceEdit(work->cancellation);
        work->authority.reset();work->ordinaryOwner.reset();work->context.Nullify();
        work->phase=Phase::Settled;work->cancellation->decision.store(Decision::Settled);
        if(_savedCutSourceEditWork.lock()==work)_savedCutSourceEditWork.reset();return result;
    };
    try {
        const auto a=work->authority;
        if(work->cancellation->decision.load()!=Decision::Pending||!built||!a||a->consumed||a->modelingPermit
            ||!a->cutStamp||!a->rebuildAuthority||a->rebuildAuthority->records.size()!=1
            ||built->originatingWork.lock()!=work->detachedGeometry
            ||work->detachedGeometry->stop->load()
            ||work->ordinaryOwner.lock()!=_ordinaryEditController||!_ordinaryEditController
            ||work->context!=myContext||myDoc!=a->owner||myDoc.IsNull()
            ||myDoc->Document()!=a->document||a->document.IsNull()
            ||a->document->GetData()->Time()!=a->documentTime)return finish(OrdinaryEditResult::Invalid);
        if(!canBeginCommittedEdit())return finish(OrdinaryEditResult::Busy);
        const auto snapshot=captureSceneSnapshot(a->width,a->height);
        if(!snapshot||snapshot->publicationSourceIdentifier!=a->identity.publicationSourceIdentifier
            ||snapshot->selectionMode!=scene::ElementKind::Object
            ||snapshot->revisions.documentGeneration!=a->identity.documentGeneration
            ||snapshot->revisions.model!=a->identity.modelRevision
            ||snapshot->revisions.presentation!=a->presentationRevision
            ||!_shapeInteractor||!_objectInteractor||!_shapeInteractor->selectionModeAuthorityIsExact()
            ||_shapeInteractor->getSelectionMode()!=a->authority.selectionMode
            ||!_objectInteractor->verifyOrdinaryNameAuthority(a->authority))return finish(OrdinaryEditResult::Invalid);
        auto authority=*a->rebuildAuthority;
        if(!admitTransform(authority)||authority.selectionOwners!=a->rebuildAuthority->selectionOwners
            ||authority.manipulatorType!=a->rebuildAuthority->manipulatorType
            ||authority.hadManipulator!=a->rebuildAuthority->hadManipulator)return finish(OrdinaryEditResult::Invalid);
        auto& record=authority.records.front();OcctObjectTransformState current;
        cut_display::Settings settings;
        if(record.requested.operation!=OrdinaryTransformOperation::CylindricalCutSourceRebuild
            ||!record.requested.cutSourcePatch||!record.requested.cutSource||record.requested.cut
            ||!myDoc->CaptureObjectTransformStateForLabel(record.previous.label,current)
            ||!current.IsEqual(record.previous)
            ||!sweep_rebuild::SameRawScalars(current.scalars,record.previous.scalars)
            ||record.requested.presentation.IsNull()
            ||!cut_display::Capture(record.requested.presentation->Attributes(),settings)
            ||!(settings==work->detachedGeometry->displaySettings)||!(settings==built->displaySettings)
            ||!myDoc->SavedCutSceneStateMatches(record.requested.cutSource))return finish(OrdinaryEditResult::Invalid);
        const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        if(!stamp||!(*stamp==*a->cutStamp)||work->cancellation->decision.load()!=Decision::Pending
            ||work->detachedGeometry->stop->load())return finish(OrdinaryEditResult::Invalid);
        record.requested.cutSourceRebuild=built;
        record.requested.shape=built->noChange?record.previous.shape:built->newResult;
        OcctExactLabelReceipt replayTarget;
        auto replayContext = captureNativeOpeningContext(a->width, a->height,
            {record.previous.entityIdentifier});
        dependent_replay::Candidate replayCandidate;
        replayCandidate.targetEntityIdentifier = record.previous.entityIdentifier;
        replayCandidate.resultShape = record.requested.shape;
        replayCandidate.retainedBase = built->newBase;
        replayCandidate.retainedRecipe = built->values.newEnvelope;
        replayCandidate.retainedRecipeBytes = built->values.newBytes;
        replayCandidate.hasRetainedRecipe = true;
        std::shared_ptr<const dependent_replay::Plan> replayPlan;
        dependent_replay::ProductionPreparer replayPreparer(
            std::move(replayCandidate), replayContext);
        if (!replayContext
            || !myDoc->CaptureExactFreeLabel(record.previous.label, replayTarget)
            || myDoc->PrepareDependentReplayPlan(replayTarget,
                dependent_replay::Mutation::Replace, dependent_replay::Limits{},
                replayPreparer, record.requested.shape,
                replayPlan) != dependent_replay::Refusal::None
            || !replayPlan) return finish(OrdinaryEditResult::Invalid);
        // Stop and synchronous ordinary dispatch have one atomic decision. Stop
        // that loses this race cannot change the actual ordinary result afterward.
        auto pending=Decision::Pending;
        if(!work->cancellation->decision.compare_exchange_strong(pending,Decision::Committing))return finish(OrdinaryEditResult::Invalid);
        a->consumed=true;
        OrdinaryEditResult result=OrdinaryEditResult::Invalid;
        auto lease=_ordinaryEditController->beginDependentTransform(record.requested,
            std::move(replayPlan),std::move(replayContext),&result);
        return finish(lease?lease.stageAndCommit():result);
    }catch(...){return finish(OrdinaryEditResult::Invalid);}
}

std::shared_ptr<SavedProgramSourceDetachedWork> Core3DViewer::makeSavedProgramSourceDetachedWork() noexcept {
    try {
        auto work=std::shared_ptr<SavedProgramSourceDetachedWork>(new SavedProgramSourceDetachedWork());
        work->profile=std::make_shared<ProfileSolidGeometry>();
        work->stop=std::shared_ptr<std::atomic_bool>(work->profile,&work->profile->cancelled);
        return work;
    }catch(...){return {};}
}
void Core3DViewer::cancelSavedProgramSourceDetached(const std::shared_ptr<SavedProgramSourceDetachedWork>& work) noexcept {
    if(work&&work->stop)work->stop->store(true);
}
bool Core3DViewer::prepareSavedProgramSourceDetached(
    const std::shared_ptr<SavedProgramSourceDetachedWork>& work,const CylindricalCutProgramSnapshot& original,
    const saved_cut_source_edit::Patch& patch,const ObjectFrameIdentity& identity,
    std::uint64_t presentation,std::uint32_t width,std::uint32_t height) noexcept {
    using Phase=SavedProgramSourceDetachedWork::Phase;
    if(!work||!work->stop||!NSThread.isMainThread)return false;
    auto fresh=Phase::Fresh;
    if(!work->phase.compare_exchange_strong(fresh,Phase::Preparing))return false;
    const auto refuse=[&](){work->phase.store(Phase::Refused);return false;};
    try {
        const auto& stop=*work->stop;
        if(stop.load()||!original.source.original.retained.value
            ||identity.entityIdentifier!=original.identity.entityIdentifier
            ||identity.publicationSourceIdentifier!=original.identity.publicationSourceIdentifier
            ||identity.documentGeneration!=original.identity.documentGeneration
            ||identity.modelRevision!=original.identity.modelRevision)return refuse();
        // Revalidate against a freshly captured original; stale input is never
        // refreshed onto a different target, and no recapture happens later.
        const auto current=cylindricalCutProgramSource(identity,presentation,width,height);
        if(stop.load()||!current
            ||!(current->authorityStamp==original.authorityStamp)
            ||!current->source.original.IsEqual(original.source.original)
            ||!sweep_rebuild::SameRawScalars(current->source.original.scalars,original.source.original.scalars)
            ||current->source.effectiveMM!=original.source.effectiveMM
            ||current->source.recipeBytes!=original.source.recipeBytes
            ||!myDoc->SavedCutSceneStateMatches(original.guard))return refuse();
        myContext->InitSelected();
        const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        if(selected.IsNull()||!cut_display::Capture(selected->Attributes(),work->displaySettings))return refuse();
        work->displayCaptured=true;
        const auto& retained=*original.source.original.retained.value;
        std::vector<std::uint8_t> originalRecipe;
        if(stop.load()||!retained_boolean::Encode(original.source.recipe,originalRecipe)
            ||originalRecipe!=retained.bytes||!retained.base.IsEqual(original.source.base)
            ||!saved_boolean_build::PrepareSourceValues(original.source.recipe,patch,stop,work->values,saved_program_source_edit::PrepareValues)
            ||stop.load())return refuse();
        const auto& base=original.source.base;
        const auto& result=original.source.original.shape;
        // Source guard already binds exact live content. Perform classifiers'
        // raw admission for EVERY operand before our cumulative traversal or
        // stream/copy. Final full-scene check also covers these inspections.
        for(std::size_t i=0;i<work->values.oldProgram.steps.size();++i)
            if(!saved_boolean_build::InspectSourceBase(base,saved_boolean_result::detail::GeometryView(work->values.oldProgram,i),stop))return refuse();
        if(!saved_boolean_build::VerifyCurrent(base,result,work->values.oldProgram,work->displaySettings,stop,work->budget)||stop.load())return refuse();
        // Re-charge occurrence and stream budgets into the ONE aggregate job
        // budget before any geometry copy; the rebuild below shares it.
        if(!analytic_boolean::detail::Bounded(base,1024,stop)
            ||!analytic_boolean::detail::Bounded(result,32768,stop)
            ||!saved_boolean_build::Charge(base,stop,work->budget)
            ||!saved_boolean_build::Charge(result,stop,work->budget)
            ||!saved_cut_source_edit::Commit(base,stop,work->budget.streamBytes,work->sourceBase)
            ||!saved_cut_source_edit::Commit(result,stop,work->budget.streamBytes,work->sourceResult))return refuse();
        // Copy only after complete admitted original geometry, preserving meshes
        // for content evidence. No shared live labels/materials reach the worker.
        BRepBuilderAPI_Copy baseCopy(base,Standard_True,Standard_True);
        if(stop.load()||!baseCopy.IsDone()||baseCopy.Shape().IsNull()
            ||baseCopy.Shape().IsPartner(base))return refuse();
        BRepBuilderAPI_Copy resultCopy(result,Standard_True,Standard_True);
        if(stop.load()||!resultCopy.IsDone()||resultCopy.Shape().IsNull()
            ||resultCopy.Shape().IsPartner(result))return refuse();
        work->oldBase=baseCopy.Shape();work->oldResult=resultCopy.Shape();
        if(!saved_boolean_build::Charge(work->oldBase,stop,work->budget)
            ||!saved_boolean_build::Charge(work->oldResult,stop,work->budget))return refuse();
        for(std::size_t i=0;i<work->values.oldProgram.steps.size();++i)
            if(!saved_boolean_build::InspectSourceBase(work->oldBase,saved_boolean_result::detail::GeometryView(work->values.oldProgram,i),stop))return refuse();
        if((work->values.oldProgram.filletSteps.empty()&&saved_boolean_build::PreFilletInspection(work->oldResult,work->values.oldProgram,stop).classification
            !=saved_boolean_result::Classification::MatchedOrientedBoundary)
            ||!saved_cut_source_edit::Commit(work->oldBase,stop,work->budget.streamBytes,work->privateBase)
            ||!saved_cut_source_edit::Commit(work->oldResult,stop,work->budget.streamBytes,work->privateResult))return refuse();
        saved_cut_source_edit::ShapeCommitment afterBase,afterResult;
        if(!saved_cut_source_edit::Commit(base,stop,work->budget.streamBytes,afterBase)
            ||!saved_cut_source_edit::Commit(result,stop,work->budget.streamBytes,afterResult)
            ||!(afterBase==work->sourceBase)||!(afterResult==work->sourceResult))return refuse();
        cut_display::Settings afterDisplay;
        if(!cut_display::Capture(selected->Attributes(),afterDisplay)
            ||!(afterDisplay==work->displaySettings))return refuse();
        // No root, snapshot, currentness stamp or guard is retained by work.
        // Future ordinary owner must preserve them and recheck before dispatch.
        const auto after=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        if(stop.load()||!after||!(*after==original.authorityStamp)
            ||!myDoc->SavedCutSceneStateMatches(original.guard))return refuse();
        work->phase.store(Phase::Prepared,std::memory_order_release);return true;
    }catch(...){return refuse();}
}
std::shared_ptr<const SavedProgramSourceDetachedResult> Core3DViewer::buildSavedProgramSourceDetached(
    const std::shared_ptr<SavedProgramSourceDetachedWork>& work,retained_fillet::Outcome* filletOutcome) noexcept {
    if(filletOutcome)*filletOutcome=retained_fillet::Outcome::Built;
    using Phase=SavedProgramSourceDetachedWork::Phase;
    if(!work||!work->stop)return {};
    auto prepared=Phase::Prepared;
    if(!work->phase.compare_exchange_strong(prepared,Phase::Building,std::memory_order_acquire))return {};
    const auto refuse=[&]()->std::shared_ptr<const SavedProgramSourceDetachedResult>{
        work->phase.store(Phase::Refused,std::memory_order_release);return {};
    };
    try {
        const auto& stop=*work->stop;
        saved_cut_source_edit::ShapeCommitment beforeBase,beforeResult;
        if(stop.load()||!work->displayCaptured
            ||!saved_cut_source_edit::Commit(work->oldBase,stop,work->budget.streamBytes,beforeBase)
            ||!saved_cut_source_edit::Commit(work->oldResult,stop,work->budget.streamBytes,beforeResult)
            ||!(beforeBase==work->privateBase)||!(beforeResult==work->privateResult))return refuse();
        for(std::size_t i=0;i<work->values.oldProgram.steps.size();++i)
            if(!saved_boolean_build::InspectSourceBase(work->oldBase,saved_boolean_result::detail::GeometryView(work->values.oldProgram,i),stop))return refuse();
        if((work->values.oldProgram.filletSteps.empty()&&saved_boolean_build::PreFilletInspection(work->oldResult,work->values.oldProgram,stop).classification
            !=saved_boolean_result::Classification::MatchedOrientedBoundary))return refuse();
        auto output=std::shared_ptr<SavedProgramSourceDetachedResult>(new SavedProgramSourceDetachedResult());
        output->values=work->values;output->sourceBase=work->sourceBase;output->sourceResult=work->sourceResult;
        output->privateOldBase=work->privateBase;output->privateOldResult=work->privateResult;
        output->noChange=!work->values.changed;
        output->displaySettings=work->displaySettings;output->originatingWork=work;
        if(work->values.changed){
            // Regenerate the ORIGINAL source base from the new source values.
            // The already-cut result is never a substitute base; the complete
            // cut program replays on the regenerated base under the same
            // aggregate budget, and the exact new recipe bytes must round-trip.
            const auto& source=work->values.newProgram.source;
            TopoDS_Shape base;
            if(source.family==1){
                profile::Parameters p;if(!profile::Decode(source.values,p))return refuse();
                static_cast<ProfileDefinition&>(*work->profile)=p.definition;
                work->profile->constructionFrame=p.constructionFrame;
                if(stop.load()||!BuildProfileSolidGeometry(work->profile))return refuse();
                base=work->profile->solid;
            }else if(source.family==2){
                enclosure::Parameters p;EnclosureSolidResult built;
                if(!enclosure::Decode(int(source.schema),source.values,p)||stop.load()
                    ||!BuildEnclosureSolidGeometry(p.definition,work->stop,built))return refuse();
                base=built.solid;
            }else if(source.family==3){
                const auto view=saved_boolean_result::detail::GeometryView(work->values.newProgram,0);
                if(saved_cut_source_edit::RebuildLoftBase(view,stop,base)!=saved_cut_source_edit::LoftBaseStatus::Built)return refuse();
            }else return refuse();
            output->build=saved_boolean_build::Build(base,work->values.newProgram,work->displaySettings,stop,work->budget);
            if(filletOutcome)*filletOutcome=output->build.filletOutcome;
            if(stop.load()||output->build.status!=saved_boolean_build::Status::Built
                ||output->build.exactProgram!=work->values.newBytes)return refuse();
            output->generatedBase=output->build.retainedBase;output->generatedResult=output->build.finalResult;
            output->newBase=base;output->newResult=output->build.solid;
        }else{
            // Informational exact no-op, not an OrdinaryEditResult or receipt.
            // No replacement shape is exposed or fabricated.
            output->generatedBase=output->sourceBase;output->generatedResult=output->sourceResult;
        }
        saved_cut_source_edit::ShapeCommitment finalOldBase,finalOldResult;
        if(!saved_cut_source_edit::Commit(work->oldBase,stop,work->budget.streamBytes,finalOldBase)
            ||!saved_cut_source_edit::Commit(work->oldResult,stop,work->budget.streamBytes,finalOldResult)
            ||!(finalOldBase==work->privateBase)||!(finalOldResult==work->privateResult)||stop.load())return refuse();
        output->build.budget=work->budget;
        work->phase.store(Phase::Finished,std::memory_order_release);return output;
    }catch(...){return refuse();}
}

std::shared_ptr<SavedProgramSourceEditWork> Core3DViewer::makeSavedProgramSourceEditWork() noexcept {
    if(!NSThread.isMainThread)return {};
    try {
        auto geometry=makeSavedProgramSourceDetachedWork();if(!geometry)return {};
        auto token=std::shared_ptr<SavedProgramSourceEditCancellation>(new SavedProgramSourceEditCancellation(geometry));
        return std::shared_ptr<SavedProgramSourceEditWork>(new SavedProgramSourceEditWork(std::move(geometry),std::move(token)));
    }catch(...){return {};}
}
std::shared_ptr<SavedProgramSourceEditCancellation> Core3DViewer::savedProgramSourceEditCancellation(
    const std::shared_ptr<SavedProgramSourceEditWork>& work) noexcept {
    if(!NSThread.isMainThread||!work)return {};
    return work->cancellation;
}
bool Core3DViewer::cancelSavedProgramSourceEdit(const std::shared_ptr<SavedProgramSourceEditCancellation>& token) noexcept {
    if(!token)return false;
    using Decision=SavedProgramSourceEditCancellation::Decision;
    auto pending=Decision::Pending;
    if(!token->decision.compare_exchange_strong(pending,Decision::Cancelled))return pending==Decision::Cancelled;
    cancelSavedProgramSourceDetached(token->geometry);return true;
}
bool Core3DViewer::discardSavedProgramSourceEdit(const std::shared_ptr<SavedProgramSourceEditWork>& work) noexcept {
    if(!NSThread.isMainThread||!work||_savedProgramSourceEditWork.lock()!=work
        ||work->phase==SavedProgramSourceEditWork::Phase::Settled
        ||!cancelSavedProgramSourceEdit(work->cancellation))return false;
    work->authority.reset();work->ordinaryOwner.reset();work->context.Nullify();
    work->phase=SavedProgramSourceEditWork::Phase::Settled;
    work->cancellation->decision.store(SavedProgramSourceEditCancellation::Decision::Settled);
    _savedProgramSourceEditWork.reset();return true;
}
std::shared_ptr<SavedProgramSourceDetachedWork> Core3DViewer::savedProgramSourceEditGeometry(
    const std::shared_ptr<SavedProgramSourceEditWork>& work) noexcept {
    if(!NSThread.isMainThread||!work||work->phase!=SavedProgramSourceEditWork::Phase::Prepared
        ||work->cancellation->decision.load()!=SavedProgramSourceEditCancellation::Decision::Pending)return {};
    return work->detachedGeometry;
}
bool Core3DViewer::prepareSavedProgramSourceEdit(const std::shared_ptr<SavedProgramSourceEditWork>& work,
    const CylindricalCutProgramSnapshot& original,const saved_cut_source_edit::Patch& patch,
    const ObjectFrameIdentity& identity,std::uint64_t presentation,std::uint32_t width,std::uint32_t height) noexcept {
    using Phase=SavedProgramSourceEditWork::Phase;using Decision=SavedProgramSourceEditCancellation::Decision;
    if(!NSThread.isMainThread||!work||work->phase!=Phase::Fresh)return false;
    work->phase=Phase::Preparing;
    const auto refuse=[&](){
        (void)cancelSavedProgramSourceEdit(work->cancellation);work->authority.reset();work->ordinaryOwner.reset();work->context.Nullify();
        work->phase=Phase::Settled;work->cancellation->decision.store(Decision::Settled);
        if(_savedProgramSourceEditWork.lock()==work)_savedProgramSourceEditWork.reset();return false;
    };
    try {
        if(work->cancellation->decision.load()!=Decision::Pending||!canBeginCommittedEdit()||!_ordinaryEditController)return refuse();
        // Cross-family exclusion: a live legacy one-bore source lease refuses
        // here; a settled one releases its slot without calling release().
        if(const auto foreign=_savedCutSourceEditWork.lock()){
            if(foreign->phase==SavedCutSourceEditWork::Phase::Settled)_savedCutSourceEditWork.reset();
            else return refuse();
        }
        // Context/document reset need not call release. A retained cancelled or
        // demonstrably stale old lease must not occupy the next source slot.
        if(const auto old=_savedProgramSourceEditWork.lock()){
            if(old->phase==Phase::Settled)_savedProgramSourceEditWork.reset();
            else {
                if(old->phase!=Phase::Prepared||old->cancellation->decision.load()==Decision::Committing)return refuse();
                const auto a=old->authority;
                bool stale=old->cancellation->decision.load()==Decision::Cancelled||!a||!a->cutStamp
                    ||!a->rebuildAuthority||a->rebuildAuthority->records.size()!=1
                    ||old->ordinaryOwner.lock()!=_ordinaryEditController||old->context!=myContext
                    ||myDoc!=a->owner||myDoc.IsNull()||myDoc->Document()!=a->document
                    ||a->document.IsNull()||a->document->GetData()->Time()!=a->documentTime;
                if(!stale){
                    const auto current=cylindricalCutProgramSource(a->identity,a->presentationRevision,a->width,a->height);
                    const auto& record=a->rebuildAuthority->records.front();cut_display::Settings settings;
                    auto admitted=*a->rebuildAuthority;
                    stale=!current||!(current->authorityStamp==*a->cutStamp)
                        ||!current->source.original.IsEqual(record.previous)
                        ||!myDoc->SavedCutSceneStateMatches(record.requested.cutSource)
                        ||!_objectInteractor||!_objectInteractor->verifyOrdinaryNameAuthority(a->authority)
                        ||!admitTransform(admitted)||admitted.selectionOwners!=a->rebuildAuthority->selectionOwners
                        ||admitted.manipulatorType!=a->rebuildAuthority->manipulatorType
                        ||admitted.hadManipulator!=a->rebuildAuthority->hadManipulator
                        ||record.requested.presentation.IsNull()
                        ||!cut_display::Capture(record.requested.presentation->Attributes(),settings)
                        ||!(settings==old->detachedGeometry->displaySettings);
                }
                if(!stale||!discardSavedProgramSourceEdit(old))return refuse();
            }
        }
        _savedProgramSourceEditWork=work;work->ordinaryOwner=_ordinaryEditController;work->context=myContext;
        if(!prepareSavedProgramSourceDetached(work->detachedGeometry,original,patch,identity,presentation,width,height)
            ||work->cancellation->decision.load()!=Decision::Pending)return refuse();
        auto authority=prepareNativeSolidWork(identity,presentation,width,height);
        if(!authority||retained_solid::Bits(authority->metersPerUnit)
                !=retained_solid::Bits(retained_boolean::Identities(original.source.recipe).metersPerUnit)
            ||authority->modelingPermit||authority->rebuildAuthority)return refuse();
        myContext->InitSelected();if(!myContext->MoreSelected())return refuse();
        const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
        if(selected.IsNull()||myContext->MoreSelected())return refuse();
        OrdinaryTransformRecord record;record.previous=original.source.original;
        record.requested.label=record.previous.label;record.requested.presentation=selected;
        record.requested.shape=record.previous.shape;record.requested.transform=record.previous.transform;
        record.requested.operation=OrdinaryTransformOperation::CylindricalCutProgramSourceRebuild;
        record.requested.cutProgramSourcePatch=patch;record.requested.cutSource=original.guard;
        authority->rebuildAuthority.emplace();authority->rebuildAuthority->records.push_back(std::move(record));
        if(!admitTransform(*authority->rebuildAuthority))return refuse();
        const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        cut_display::Settings settings;
        if(!stamp||!(*stamp==original.authorityStamp)||!myDoc->SavedCutSceneStateMatches(original.guard)
            ||!cut_display::Capture(selected->Attributes(),settings)
            ||!(settings==work->detachedGeometry->displaySettings)
            ||work->ordinaryOwner.lock()!=_ordinaryEditController||work->context!=myContext
            ||work->cancellation->decision.load()!=Decision::Pending)return refuse();
        authority->cutStamp=original.authorityStamp;authority->frameFirst=false;
        work->authority=std::move(authority);work->phase=Phase::Prepared;return true;
    }catch(...){return refuse();}
}
OrdinaryEditResult Core3DViewer::commitSavedProgramSourceEdit(const std::shared_ptr<SavedProgramSourceEditWork>& work,
    const std::shared_ptr<const SavedProgramSourceDetachedResult>& built) noexcept {
    using Phase=SavedProgramSourceEditWork::Phase;using Decision=SavedProgramSourceEditCancellation::Decision;
    if(!NSThread.isMainThread||!work||work->phase!=Phase::Prepared
        ||_savedProgramSourceEditWork.lock()!=work)return OrdinaryEditResult::Invalid;
    const auto finish=[&](OrdinaryEditResult result){
        if(work->cancellation->decision.load()!=Decision::Committing)(void)cancelSavedProgramSourceEdit(work->cancellation);
        work->authority.reset();work->ordinaryOwner.reset();work->context.Nullify();
        work->phase=Phase::Settled;work->cancellation->decision.store(Decision::Settled);
        if(_savedProgramSourceEditWork.lock()==work)_savedProgramSourceEditWork.reset();return result;
    };
    try {
        const auto a=work->authority;
        if(work->cancellation->decision.load()!=Decision::Pending||!built||!a||a->consumed||a->modelingPermit
            ||!a->cutStamp||!a->rebuildAuthority||a->rebuildAuthority->records.size()!=1
            ||built->originatingWork.lock()!=work->detachedGeometry
            ||work->detachedGeometry->stop->load()
            ||work->ordinaryOwner.lock()!=_ordinaryEditController||!_ordinaryEditController
            ||work->context!=myContext||myDoc!=a->owner||myDoc.IsNull()
            ||myDoc->Document()!=a->document||a->document.IsNull()
            ||a->document->GetData()->Time()!=a->documentTime)return finish(OrdinaryEditResult::Invalid);
        if(!canBeginCommittedEdit())return finish(OrdinaryEditResult::Busy);
        const auto snapshot=captureSceneSnapshot(a->width,a->height);
        if(!snapshot||snapshot->publicationSourceIdentifier!=a->identity.publicationSourceIdentifier
            ||snapshot->selectionMode!=scene::ElementKind::Object
            ||snapshot->revisions.documentGeneration!=a->identity.documentGeneration
            ||snapshot->revisions.model!=a->identity.modelRevision
            ||snapshot->revisions.presentation!=a->presentationRevision
            ||!_shapeInteractor||!_objectInteractor||!_shapeInteractor->selectionModeAuthorityIsExact()
            ||_shapeInteractor->getSelectionMode()!=a->authority.selectionMode
            ||!_objectInteractor->verifyOrdinaryNameAuthority(a->authority))return finish(OrdinaryEditResult::Invalid);
        auto authority=*a->rebuildAuthority;
        if(!admitTransform(authority)||authority.selectionOwners!=a->rebuildAuthority->selectionOwners
            ||authority.manipulatorType!=a->rebuildAuthority->manipulatorType
            ||authority.hadManipulator!=a->rebuildAuthority->hadManipulator)return finish(OrdinaryEditResult::Invalid);
        auto& record=authority.records.front();OcctObjectTransformState current;
        cut_display::Settings settings;
        if(record.requested.operation!=OrdinaryTransformOperation::CylindricalCutProgramSourceRebuild
            ||!record.requested.cutProgramSourcePatch||!record.requested.cutSource||record.requested.cut
            ||record.requested.cutSourcePatch||record.requested.cutSourceRebuild||record.requested.cutProgramEdit
            ||!myDoc->CaptureObjectTransformStateForLabel(record.previous.label,current)
            ||!current.IsEqual(record.previous)
            ||!sweep_rebuild::SameRawScalars(current.scalars,record.previous.scalars)
            ||record.requested.presentation.IsNull()
            ||!cut_display::Capture(record.requested.presentation->Attributes(),settings)
            ||!(settings==work->detachedGeometry->displaySettings)||!(settings==built->displaySettings)
            ||!myDoc->SavedCutSceneStateMatches(record.requested.cutSource))return finish(OrdinaryEditResult::Invalid);
        const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        if(!stamp||!(*stamp==*a->cutStamp)||work->cancellation->decision.load()!=Decision::Pending
            ||work->detachedGeometry->stop->load())return finish(OrdinaryEditResult::Invalid);
        record.requested.cutProgramSourceRebuild=built;
        record.requested.shape=built->noChange?record.previous.shape:built->newResult;
        OcctExactLabelReceipt replayTarget;
        auto replayContext = captureNativeOpeningContext(a->width, a->height,
            {record.previous.entityIdentifier});
        dependent_replay::Candidate replayCandidate;
        replayCandidate.targetEntityIdentifier = record.previous.entityIdentifier;
        replayCandidate.resultShape = record.requested.shape;
        replayCandidate.retainedBase = built->newBase;
        replayCandidate.retainedRecipe = built->values.newProgram;
        replayCandidate.retainedRecipeBytes = built->values.newBytes;
        replayCandidate.hasRetainedRecipe = true;
        std::shared_ptr<const dependent_replay::Plan> replayPlan;
        dependent_replay::ProductionPreparer replayPreparer(
            std::move(replayCandidate), replayContext);
        if (!replayContext
            || !myDoc->CaptureExactFreeLabel(record.previous.label, replayTarget)
            || myDoc->PrepareDependentReplayPlan(replayTarget,
                dependent_replay::Mutation::Replace, dependent_replay::Limits{},
                replayPreparer, record.requested.shape,
                replayPlan) != dependent_replay::Refusal::None
            || !replayPlan) return finish(OrdinaryEditResult::Invalid);
        // Stop and synchronous ordinary dispatch have one atomic decision. Stop
        // that loses this race cannot change the actual ordinary result afterward.
        auto pending=Decision::Pending;
        if(!work->cancellation->decision.compare_exchange_strong(pending,Decision::Committing))return finish(OrdinaryEditResult::Invalid);
        a->consumed=true;
        OrdinaryEditResult result=OrdinaryEditResult::Invalid;
        auto lease=_ordinaryEditController->beginDependentTransform(record.requested,
            std::move(replayPlan),std::move(replayContext),&result);
        return finish(lease?lease.stageAndCommit():result);
    }catch(...){return finish(OrdinaryEditResult::Invalid);}
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareProfileSolid(
    const profile::Parameters& parameters, const ObjectFrameIdentity& identity,
    std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height) noexcept {
    if (![NSThread isMainThread] || parameters.constructionFrame || !parameters.shells.empty()
        || !std::isfinite(parameters.metersPerUnit) || parameters.metersPerUnit <= 0) return {};
    try {
        // No unit conversion or new history occurs during preparation. The
        // captured document time also fences unit changes before commit.
        auto work = prepareProfileSolid(parameters.definition, identity,
            presentationRevision, width, height);
        if (!work || work->metersPerUnit != parameters.metersPerUnit) return {};
        return work;
    } catch (...) { return {}; }
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareProfileSolid(
    const std::vector<gp_Pnt2d>& points, int plane, double depth,
    const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
    std::uint32_t width, std::uint32_t height, bool revolve,
    const std::optional<ProfileCircularSection>& circle,
    const std::vector<ProfileCircularHole>& holes) noexcept {
    try {
        ProfileDefinition definition;
        definition.points=points;definition.circle=circle;definition.holes=holes;
        definition.plane=plane;definition.depth=depth;definition.revolve=revolve;
        return prepareProfileSolid(definition,identity,presentationRevision,width,height);
    } catch (...) { return {}; }
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareNativeSolidWork(
    const ObjectFrameIdentity& identity,
    std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height) noexcept {
    if (![NSThread isMainThread] || !canBeginCommittedEdit() || myContext.IsNull()
        || myDoc.IsNull() || myDoc->Document().IsNull() || width == 0 || height == 0) { return {}; }
    try {
        const auto snapshot = captureSceneSnapshot(width, height);
        if (!snapshot || snapshot->selectionMode != scene::ElementKind::Object
            || snapshot->publicationSourceIdentifier != identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration != identity.documentGeneration
            || snapshot->revisions.model != identity.modelRevision
            || snapshot->revisions.presentation != presentationRevision) { return {}; }
        auto work = std::make_shared<NativeSolidWork>();
        if (!admitNames(work->authority)) { return {}; }
        work->identity = identity; work->presentationRevision = presentationRevision;
        work->width = width; work->height = height;
        work->owner = myDoc; work->document = myDoc->Document();
        if (!XCAFDoc_DocumentTool::GetLengthUnit(work->document, work->metersPerUnit)
            || !std::isfinite(work->metersPerUnit) || work->metersPerUnit <= 0) return {};
        work->documentTime = work->document->GetData()->Time();
        Standard_Size count = 0;
        if (!TryCountDisplayedModelShapes(myContext, count)) { return {}; }
        work->frameFirst = count == 0;
        return work;
    } catch (...) { return {}; }
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareProfileSolid(
    const ProfileDefinition& definition, const ObjectFrameIdentity& identity,
    std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height) noexcept {
    if (![NSThread isMainThread]) return {};
    try {
        double area=0,volume=0;
        if (!ProfileDefinitionExpectedVolume(definition,area,volume)) return {};
        auto work=prepareNativeSolidWork(identity,presentationRevision,width,height);
        if (!work) return {};
        auto geometry=std::make_shared<ProfileSolidGeometry>();
        static_cast<ProfileDefinition&>(*geometry)=definition;
        work->geometry=std::move(geometry);
        return work;
    } catch (...) { return {}; }
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareAssemblySolid(
    const std::vector<AssemblyPartDefinition>& parts, const ObjectFrameIdentity& identity,
    std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height) noexcept {
    if (![NSThread isMainThread] || parts.empty() || parts.size()>16) return {};
    try {
        auto work = prepareNativeSolidWork(identity,presentationRevision,width,height);
        if (!work) return {};
        std::unordered_set<std::string> identifiers;
        auto geometry = std::make_shared<AssemblySolidGeometry>();
        for (const auto& part : parts) {
            std::vector<double> encoded;
            if (part.parameters.metersPerUnit != work->metersPerUnit || !part.parameters.constructionFrame
                || !profile::Encode(part.parameters, encoded) || !profile::IsIdentifier(part.identifier)
                || !identifiers.insert(part.identifier).second || !OcctObjectNameIsValid(part.name)) return {};
            auto solid = std::make_shared<ProfileSolidGeometry>();
            static_cast<ProfileDefinition&>(*solid) = part.parameters.definition;
            solid->constructionFrame = part.parameters.constructionFrame;
            geometry->parts.push_back(std::move(solid));
        }
        work->assemblyParts = parts; work->geometry = std::move(geometry);
        return work;
    } catch (...) { return {}; }
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareLoftSolid(
    const rectangular_loft::Definition& definition,const ObjectFrameIdentity& identity,
    std::uint64_t presentationRevision,std::uint32_t width,std::uint32_t height) noexcept {
    if (![NSThread isMainThread]) return {};
    try {
        rectangular_loft::Admission admission;
        const auto prepared=rectangular_loft::Prepare(definition,admission);
        if (!prepared) return {};
        auto work=prepareNativeSolidWork(identity,presentationRevision,width,height);
        if (!work || !work->owner->ValidateGeometryRepresentations()
            || sweep_persistence::Bits(work->metersPerUnit)!=sweep_persistence::Bits(definition.dimensionMetersPerUnit)) return {};
        NSString *identifier=NSUUID.UUID.UUIDString;
        if (!identifier || !identifier.UTF8String) return {};
        work->loftIdentifier=identifier.UTF8String;
        auto geometry=std::make_shared<LoftSolidGeometry>();geometry->prepared=prepared;
        work->geometry=std::move(geometry);return work;
    } catch (...) {return {};}
}
std::shared_ptr<NativeSolidWork> Core3DViewer::prepareSweepSolid(
    const planar_sweep::Definition& definition,const ObjectFrameIdentity& identity,
    std::uint64_t presentationRevision,std::uint32_t width,std::uint32_t height) noexcept {
    if (![NSThread isMainThread]) return {};
    try {
        planar_sweep::Admission admission;
        const auto prepared=planar_sweep::Prepare(definition,admission);
        if (!prepared) return {};
        auto work=prepareNativeSolidWork(identity,presentationRevision,width,height);
        if (!work || !work->owner->ValidateGeometryRepresentations()
            || sweep_persistence::Bits(work->metersPerUnit)!=sweep_persistence::Bits(definition.dimensionMetersPerUnit)) return {};
        NSString *identifier=NSUUID.UUID.UUIDString;
        if (!identifier || !identifier.UTF8String) return {};
        work->sweepIdentifier=identifier.UTF8String;
        auto geometry=std::make_shared<SweepSolidGeometry>();geometry->prepared=prepared;
        work->geometry=std::move(geometry);return work;
    } catch (...) {return {};}
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareEnclosureSolid(
    const enclosure::Parameters& parameters, const ObjectFrameIdentity& identity,
    std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height) noexcept {
    if (![NSThread isMainThread]) return {};
    try {
        std::vector<double> values;
        if (!enclosure::Encode(parameters,values)) return {};
        auto work=prepareNativeSolidWork(identity,presentationRevision,width,height);
        if (!work || work->metersPerUnit != parameters.metersPerUnit) return {};
        auto geometry=std::make_shared<EnclosureSolidGeometry>(); geometry->parameters=parameters;
        work->geometry=std::move(geometry);
        return work;
    } catch (...) { return {}; }
}

std::optional<StoredProfileSnapshot> Core3DViewer::storedProfileDefinition(
    const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
    std::uint32_t width, std::uint32_t height) noexcept {
    if (![NSThread isMainThread] || !canBeginCommittedEdit() || myContext.IsNull()
        || myDoc.IsNull() || identity.entityIdentifier.empty() || width == 0 || height == 0) return {};
    try {
        const auto snapshot = captureSceneSnapshot(width, height);
        if (!snapshot || snapshot->selectionMode != scene::ElementKind::Object
            || snapshot->publicationSourceIdentifier != identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration != identity.documentGeneration
            || snapshot->revisions.model != identity.modelRevision
            || snapshot->revisions.presentation != presentationRevision) return {};
        myContext->InitSelected();
        if (!myContext->MoreSelected()) return {};
        const auto selected = Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        myContext->NextSelected();
        if (myContext->MoreSelected() || selected.IsNull()) return {};
        OcctObjectTransformState state;
        const auto label = myDoc->ShapeLabel(selected);
        if (!myDoc->CaptureObjectTransformStateForLabel(label, state)
            || state.entityIdentifier != identity.entityIdentifier
            || state.resolvedRepresentation != OcctGeometryRepresentation::BRep
            || state.profile.label.IsNull()) return {};
        StoredProfileSnapshot result;
        result.parameters = state.profile.parameters; result.identity = identity;
        result.definitionIdentifier = state.definitionIdentifier;
        result.featureIdentifier = state.profile.identifier;
        const double constructionScale = result.parameters.constructionFrame
            ? result.parameters.constructionFrame->values[7] : 1.0;
        result.dimensionMetersPerUnit = result.parameters.metersPerUnit
            * std::abs(constructionScale) * std::abs(state.scalars[7]);
        if (!std::isfinite(result.dimensionMetersPerUnit) || result.dimensionMetersPerUnit <= 0
            || !std::isfinite(result.dimensionMetersPerUnit * 1000.0))
            result.dimensionMetersPerUnit = 0; // Unsupported physical scale does not hide a manual recipe.
        result.current = state.profile.IsCurrent(myDoc->Document(), label)
            && profile::HasOnlyMetadataSubshapes(myDoc->Document(), label);
        if(result.current){
            namespace et=retained_edge_treatment;auto capture=std::shared_ptr<et::Snapshot>(new et::Snapshot);
            capture->ownerLabel_=label;capture->sourceLabel_=state.profile.label;capture->sourceIdentifier_=state.profile.identifier;capture->source_=state.profile.parameters;capture->current_=state.shape;capture->base_=state.edgeTreatment?state.edgeTreatment->value->base:state.shape;capture->nonce_=std::uint64_t(myDoc->Document()->GetData()->Time());capture->presentationRevision_=presentationRevision;
            auto&seed=capture->seed_;seed.schema=1;receipt::ParseUUID(myDoc->DocumentIdentifier(),seed.owner.document);receipt::ParseUUID(state.entityIdentifier,seed.owner.entity);receipt::ParseUUID(state.definitionIdentifier,seed.owner.definition);seed.base.source.document=seed.owner.document;seed.base.source.entity=seed.owner.entity;seed.base.source.definition=seed.owner.definition;receipt::ParseUUID(state.profile.identifier,seed.base.source.sourceFeature);seed.base.sourceNode=MintEdgeTreatmentUUID();seed.base.family=et::SourceFamily::Profile;seed.base.sourceSchema=std::uint32_t(profile::SchemaFor(state.profile.parameters));seed.base.metersPerLocalUnit=result.dimensionMetersPerUnit;seed.outputNode=seed.base.sourceNode;seed.issuance.nextLocalID=1;composite_recipe::EncodeScalarRecipe(composite_recipe::RecipeKind::Profile,seed.base.sourceSchema,state.profile.values,capture->sourceBytes_);composite_recipe::Hash(capture->sourceBytes_,seed.base.sourceRecipeDigest);
            if(state.edgeTreatment){capture->definition_=state.edgeTreatment->value->definition;capture->definitionBytes_=state.edgeTreatment->value->bytes;capture->seed_=*capture->definition_;TopoDS_Shape replayed;std::vector<et::StepProof>proofs;et::Refusal replayRefusal;if(!et::Replay(capture->base_,*capture->definition_,replayed,proofs,capture->chargedBudget_,replayRefusal)||!et::EquivalentReplayGeometry(capture->current_,replayed,capture->chargedBudget_,replayRefusal))return {};}
            capture->owner_.owner=seed.owner;capture->owner_.outputNode=capture->seed_.outputNode;capture->owner_.status=retained_recipe::OwnerStatus::CurrentEditable;result.edgeTreatment=std::move(capture);
        }
        return result;
    } catch (...) { return {}; }
}

std::optional<StoredEnclosureSnapshot> Core3DViewer::storedEnclosureDefinition(
    const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
    std::uint32_t width, std::uint32_t height) noexcept {
    if (![NSThread isMainThread] || !canBeginCommittedEdit() || myContext.IsNull()
        || myDoc.IsNull() || identity.entityIdentifier.empty() || width == 0 || height == 0) return {};
    try {
        const auto snapshot = captureSceneSnapshot(width, height);
        if (!snapshot || snapshot->selectionMode != scene::ElementKind::Object
            || snapshot->publicationSourceIdentifier != identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration != identity.documentGeneration
            || snapshot->revisions.model != identity.modelRevision
            || snapshot->revisions.presentation != presentationRevision) return {};
        myContext->InitSelected();
        if (!myContext->MoreSelected()) return {};
        const auto selected = Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        myContext->NextSelected();
        if (myContext->MoreSelected() || selected.IsNull()) return {};
        OcctObjectTransformState state;
        const auto label = myDoc->ShapeLabel(selected);
        if (!myDoc->CaptureObjectTransformStateForLabel(label, state)
            || state.entityIdentifier != identity.entityIdentifier
            || state.resolvedRepresentation != OcctGeometryRepresentation::BRep
            || state.enclosure.label.IsNull()) return {};
        StoredEnclosureSnapshot result;
        result.parameters = state.enclosure.parameters; result.identity = identity;
        result.definitionIdentifier = state.definitionIdentifier;
        result.featureIdentifier = state.enclosure.identifier;
        const double constructionScale = state.enclosure.parameters.definition.constructionFrame
            ? state.enclosure.parameters.definition.constructionFrame->values[7] : 1.0;
        const double authoredScale = state.scalars[7];
        result.dimensionMetersPerUnit = result.parameters.metersPerUnit
            * std::abs(constructionScale) * std::abs(authoredScale);
        if (!std::isfinite(result.dimensionMetersPerUnit) || result.dimensionMetersPerUnit <= 0
            || !std::isfinite(result.dimensionMetersPerUnit * 1000.0)) return {};
        result.sourceState = state;
        result.current = state.enclosure.IsCurrent(myDoc->Document(), label)
            && enclosure::HasOnlyMetadataSubshapes(myDoc->Document(), label);
        if(result.current){
            namespace et=retained_edge_treatment;auto capture=std::shared_ptr<et::Snapshot>(new et::Snapshot);
            capture->ownerLabel_=label;capture->sourceLabel_=state.enclosure.label;capture->sourceIdentifier_=state.enclosure.identifier;capture->source_=state.enclosure.parameters;capture->current_=state.shape;capture->base_=state.edgeTreatment?state.edgeTreatment->value->base:state.shape;capture->nonce_=std::uint64_t(myDoc->Document()->GetData()->Time());capture->presentationRevision_=presentationRevision;
            auto&seed=capture->seed_;seed.schema=1;receipt::ParseUUID(myDoc->DocumentIdentifier(),seed.owner.document);receipt::ParseUUID(state.entityIdentifier,seed.owner.entity);receipt::ParseUUID(state.definitionIdentifier,seed.owner.definition);seed.base.source.document=seed.owner.document;seed.base.source.entity=seed.owner.entity;seed.base.source.definition=seed.owner.definition;receipt::ParseUUID(state.enclosure.identifier,seed.base.source.sourceFeature);seed.base.sourceNode=MintEdgeTreatmentUUID();seed.base.family=et::SourceFamily::Enclosure;seed.base.sourceSchema=state.enclosure.parameters.definition.constructionFrame?2:1;seed.base.metersPerLocalUnit=result.dimensionMetersPerUnit;seed.outputNode=seed.base.sourceNode;seed.issuance.nextLocalID=1;composite_recipe::EncodeScalarRecipe(composite_recipe::RecipeKind::Enclosure,seed.base.sourceSchema,state.enclosure.values,capture->sourceBytes_);composite_recipe::Hash(capture->sourceBytes_,seed.base.sourceRecipeDigest);
            if(state.edgeTreatment){capture->definition_=state.edgeTreatment->value->definition;capture->definitionBytes_=state.edgeTreatment->value->bytes;capture->seed_=*capture->definition_;TopoDS_Shape replayed;std::vector<et::StepProof>proofs;et::Refusal replayRefusal;if(!et::Replay(capture->base_,*capture->definition_,replayed,proofs,capture->chargedBudget_,replayRefusal)||!et::EquivalentReplayGeometry(capture->current_,replayed,capture->chargedBudget_,replayRefusal))return {};}
            capture->owner_.owner=seed.owner;capture->owner_.outputNode=capture->seed_.outputNode;capture->owner_.status=retained_recipe::OwnerStatus::CurrentEditable;result.edgeTreatment=std::move(capture);
        }
        return result;
    } catch (...) { return {}; }
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareStoredProfileRebuild(
    double parameter, const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
    std::uint32_t width, std::uint32_t height) noexcept {
    const auto original = storedProfileDefinition(identity, presentationRevision, width, height);
    if (!original) return {};
    auto parameters = original->parameters; parameters.definition.depth = parameter;
    return prepareStoredProfileRebuild(parameters, *original, identity, presentationRevision, width, height);
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareStoredProfileRebuild(
    const profile::Parameters& parameters, const StoredProfileSnapshot& original,
    const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
    std::uint32_t width, std::uint32_t height) noexcept {
    if (!original.current || identity.entityIdentifier != original.identity.entityIdentifier
        || identity.publicationSourceIdentifier != original.identity.publicationSourceIdentifier
        || identity.documentGeneration != original.identity.documentGeneration
        || identity.modelRevision != original.identity.modelRevision
        || parameters.metersPerUnit != original.parameters.metersPerUnit
        || parameters.constructionFrame != original.parameters.constructionFrame
        || parameters.shells != original.parameters.shells) return {};
    try {
        const auto current = storedProfileDefinition(identity, presentationRevision, width, height);
        if (!current || !current->current || current->featureIdentifier != original.featureIdentifier
            || current->definitionIdentifier != original.definitionIdentifier
            || bool(current->edgeTreatment)!=bool(original.edgeTreatment)
            || (current->edgeTreatment&&original.edgeTreatment
                && current->edgeTreatment->definitionBytes_!=original.edgeTreatment->definitionBytes_)) return {};
        std::vector<double> originalValues, currentValues, requestedValues;
        if (!profile::Encode(original.parameters, originalValues)
            || !profile::Encode(current->parameters, currentValues) || originalValues != currentValues
            || !profile::Encode(parameters, requestedValues)) return {};
        myContext->InitSelected();
        const auto selected = Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        OrdinaryTransformRecord record;
        const auto label = myDoc->ShapeLabel(selected);
        if (!myDoc->CaptureObjectTransformStateForLabel(label, record.previous)) return {};
        const auto& d = parameters.definition;
        auto work = prepareProfileSolid(d, identity, presentationRevision, width, height);
        if (!work) return {};
        profileSolidGeometry(work)->constructionFrame = parameters.constructionFrame;
        profileSolidGeometry(work)->shells = parameters.shells;
        profileSolidGeometry(work)->shellMetersPerUnit = parameters.metersPerUnit;
        if(original.edgeTreatment&&original.edgeTreatment->definition_){
            namespace et=retained_edge_treatment;
            // A source edit moves native edges; the stored selector witnesses
            // must be verified against the old source and rebound against the
            // actually rebuilt stage, never replayed with stale midpoints.
            et::SourceRebindRoles roles;et::ReplayBudget rebindBudget=original.edgeTreatment->chargedBudget_;et::Refusal rebindRefusal=et::Refusal::ReplayMismatch;
            if(!et::CaptureSourceRebindRoles(original.edgeTreatment->base_,*original.edgeTreatment->definition_,
                original.edgeTreatment->definitionBytes_,rebindBudget,rebindRefusal,roles))return {};
            auto geometry=profileSolidGeometry(work);
            geometry->treatmentRebind=std::move(roles);geometry->treatmentRebuildSource=parameters;geometry->treatmentBudget=rebindBudget;
            work->edgeTreatmentSnapshot=original.edgeTreatment;work->edgeTreatmentEdit=retained_edge_treatment::RebuildSource{parameters};
        }
        record.requested.label = label; record.requested.presentation = selected;
        record.requested.shape = record.previous.shape; record.requested.transform = record.previous.transform;
        record.requested.operation = OrdinaryTransformOperation::ProfileRebuild;
        record.requested.profileRebuild = parameters;
        work->rebuildAuthority.emplace();
        work->rebuildAuthority->records.push_back(std::move(record));
        if (!admitTransform(*work->rebuildAuthority)) return {};
        work->frameFirst = false;
        return work;
    } catch (...) { return {}; }
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareStoredEnclosureRebuild(
    const enclosure::Parameters& parameters, const StoredEnclosureSnapshot& original,
    const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
    std::uint32_t width, std::uint32_t height) noexcept {
    if (!original.current || identity.entityIdentifier != original.identity.entityIdentifier
        || identity.publicationSourceIdentifier != original.identity.publicationSourceIdentifier
        || identity.documentGeneration != original.identity.documentGeneration
        || identity.modelRevision != original.identity.modelRevision
        || parameters.metersPerUnit != original.parameters.metersPerUnit
        || parameters.definition.constructionFrame != original.parameters.definition.constructionFrame) return {};
    try {
        const auto current = storedEnclosureDefinition(identity, presentationRevision, width, height);
        if (!current || !current->current || current->featureIdentifier != original.featureIdentifier
            || current->definitionIdentifier != original.definitionIdentifier
            || current->dimensionMetersPerUnit != original.dimensionMetersPerUnit
            || !current->sourceState.IsEqual(original.sourceState)
            || bool(current->edgeTreatment)!=bool(original.edgeTreatment)
            || (current->edgeTreatment&&original.edgeTreatment
                && current->edgeTreatment->definitionBytes_!=original.edgeTreatment->definitionBytes_)) return {};
        std::vector<double> originalValues, currentValues, requestedValues;
        if (!enclosure::Encode(original.parameters, originalValues)
            || !enclosure::Encode(current->parameters, currentValues) || originalValues != currentValues
            || !enclosure::Encode(parameters, requestedValues)) return {};
        myContext->InitSelected();
        const auto selected = Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        OrdinaryTransformRecord record;
        const auto label = myDoc->ShapeLabel(selected);
        if (!myDoc->CaptureObjectTransformStateForLabel(label, record.previous)) return {};
        auto work = prepareEnclosureSolid(parameters, identity, presentationRevision, width, height);
        if (!work) return {};
        if(original.edgeTreatment&&original.edgeTreatment->definition_){
            namespace et=retained_edge_treatment;
            // A source edit moves native edges; the stored selector witnesses
            // must be verified against the old source and rebound against the
            // actually rebuilt stage, never replayed with stale midpoints.
            et::SourceRebindRoles roles;et::ReplayBudget rebindBudget=original.edgeTreatment->chargedBudget_;et::Refusal rebindRefusal=et::Refusal::ReplayMismatch;
            if(!et::CaptureSourceRebindRoles(original.edgeTreatment->base_,*original.edgeTreatment->definition_,
                original.edgeTreatment->definitionBytes_,rebindBudget,rebindRefusal,roles))return {};
            auto enclosureGeometry=std::get<std::shared_ptr<EnclosureSolidGeometry>>(work->geometry);
            enclosureGeometry->treatmentRebind=std::move(roles);enclosureGeometry->treatmentRebuildSource=parameters;enclosureGeometry->treatmentBudget=rebindBudget;
            work->edgeTreatmentSnapshot=original.edgeTreatment;work->edgeTreatmentEdit=retained_edge_treatment::RebuildSource{parameters};
        }
        record.requested.label = label; record.requested.presentation = selected;
        record.requested.shape = record.previous.shape; record.requested.transform = record.previous.transform;
        record.requested.operation = OrdinaryTransformOperation::EnclosureRebuild;
        record.requested.enclosureRebuild = parameters;
        work->rebuildAuthority.emplace();
        work->rebuildAuthority->records.push_back(std::move(record));
        if (!admitTransform(*work->rebuildAuthority)) return {};
        work->frameFirst = false;
        return work;
    } catch (...) { return {}; }
}

std::optional<StoredSweepSnapshot> Core3DViewer::storedSweepDefinition(
    const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height) noexcept {
    if (![NSThread isMainThread] || !canBeginCommittedEdit() || myContext.IsNull()
        || myDoc.IsNull() || identity.entityIdentifier.empty() || width==0 || height==0) return {};
    try {
        const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        const auto snapshot=captureSceneSnapshot(width,height);
        if (!stamp || !snapshot || snapshot->selectionMode!=scene::ElementKind::Object
            || snapshot->publicationSourceIdentifier!=identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration!=identity.documentGeneration
            || snapshot->revisions.model!=identity.modelRevision
            || snapshot->revisions.presentation!=presentationRevision) return {};
        myContext->InitSelected();if(!myContext->MoreSelected())return {};
        const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        myContext->NextSelected();if(myContext->MoreSelected()||selected.IsNull())return {};
        OcctObjectTransformState state;OcctScalarAppearanceState appearance;
        const auto label=myDoc->ShapeLabel(selected);
        if (!myDoc->CaptureObjectTransformStateForLabel(label,state)
            || state.entityIdentifier!=identity.entityIdentifier || state.resolvedRepresentation!=OcctGeometryRepresentation::BRep
            || !state.sweep.IsCurrent(myDoc->Document(),label)
            || !sweep_rebuild::HasOnlyMetadataSubshapes(myDoc->Document(),label)
            || !myDoc->CaptureScalarAppearanceForSavedSweepRebuild(label,appearance)) return {};
        const double constructionScale=state.sweep.definition.constructionFrame
            ?state.sweep.definition.constructionFrame->values[7]:1.0;
        const double effective=state.sweep.definition.dimensionMetersPerUnit*std::abs(constructionScale)*std::abs(state.scalars[7]);
        const auto guard=_ordinaryEditController?_ordinaryEditController->captureSavedSweepRebuildSource():nullptr;
        const auto after=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        if (!guard || !after || !(*after==*stamp) || !std::isfinite(effective) || effective<=0
            || !std::isfinite(effective*1000)) return {};
        StoredSweepSnapshot result;result.definition=state.sweep.definition;result.identity=identity;
        result.definitionIdentifier=state.definitionIdentifier;result.featureIdentifier=state.sweep.identifier;
        result.sourceState=std::move(state);result.authorityStamp=*stamp;result.sourceGuard=guard;
        result.effectiveDimensionMetersPerUnit=effective;result.current=true;return result;
    } catch (...) {return {};}
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareStoredSweepRebuild(
    const planar_sweep::Definition& definition,const StoredSweepSnapshot& original,
    const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height) noexcept {
    if (![NSThread isMainThread] || !original.current || identity.entityIdentifier!=original.identity.entityIdentifier
        || identity.publicationSourceIdentifier!=original.identity.publicationSourceIdentifier
        || identity.documentGeneration!=original.identity.documentGeneration || identity.modelRevision!=original.identity.modelRevision
        || !sweep_rebuild::FixedStructure(original.definition,definition)) return {};
    try {
        const auto current=storedSweepDefinition(identity,presentationRevision,width,height);
        if (!current || !current->current || !(current->authorityStamp==original.authorityStamp)
            || current->featureIdentifier!=original.featureIdentifier || current->definitionIdentifier!=original.definitionIdentifier
            || current->effectiveDimensionMetersPerUnit!=original.effectiveDimensionMetersPerUnit
            || !current->sourceState.IsEqual(original.sourceState)
            || !sweep_rebuild::SameRawScalars(current->sourceState.scalars,original.sourceState.scalars)
            || !_ordinaryEditController || !_ordinaryEditController->savedSweepRebuildSourceIsCurrent(original.sourceGuard)) return {};
        planar_sweep::Admission admission;const auto prepared=planar_sweep::Prepare(definition,admission);
        if (!prepared) return {};
        auto work=prepareNativeSolidWork(identity,presentationRevision,width,height);
        if (!work || sweep_persistence::Bits(work->metersPerUnit)!=sweep_persistence::Bits(definition.dimensionMetersPerUnit))return {};
        myContext->InitSelected();const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        if(selected.IsNull())return {};
        auto geometry=std::make_shared<SweepSolidGeometry>();geometry->prepared=prepared;work->geometry=std::move(geometry);
        OrdinaryTransformRecord record;record.previous=current->sourceState;
        record.requested.label=record.previous.label;record.requested.presentation=selected;
        record.requested.shape=record.previous.shape;record.requested.transform=record.previous.transform;
        record.requested.operation=OrdinaryTransformOperation::SweepRebuild;record.requested.sweepRebuild=definition;
        record.requested.sweepSource=original.sourceGuard;
        work->rebuildAuthority.emplace();work->rebuildAuthority->records.push_back(std::move(record));
        if (!admitTransform(*work->rebuildAuthority)) return {};
        work->sweepRebuildStamp=original.authorityStamp;work->frameFirst=false;return work;
    } catch (...) {return {};}
}

std::optional<StoredRectangularLoftSnapshot> Core3DViewer::storedRectangularLoftDefinition(
    const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height) noexcept {
    if (![NSThread isMainThread] || !canBeginCommittedEdit() || myContext.IsNull()
        || myDoc.IsNull() || identity.entityIdentifier.empty() || width==0 || height==0) return {};
    try {
        const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        const auto snapshot=captureSceneSnapshot(width,height);
        if (!stamp || !snapshot || snapshot->selectionMode!=scene::ElementKind::Object
            || snapshot->publicationSourceIdentifier!=identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration!=identity.documentGeneration
            || snapshot->revisions.model!=identity.modelRevision
            || snapshot->revisions.presentation!=presentationRevision) return {};
        myContext->InitSelected();if(!myContext->MoreSelected())return {};
        const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        myContext->NextSelected();if(myContext->MoreSelected()||selected.IsNull())return {};
        OcctObjectTransformState state;OcctScalarAppearanceState appearance;
        const auto label=myDoc->ShapeLabel(selected);
        if (!myDoc->CaptureObjectTransformStateForLabel(label,state)
            || state.entityIdentifier!=identity.entityIdentifier || state.resolvedRepresentation!=OcctGeometryRepresentation::BRep
            || !state.loft.IsCurrent(myDoc->Document(),label)
            || !loft_rebuild::HasOnlyMetadataSubshapes(myDoc->Document(),label)
            || !myDoc->CaptureScalarAppearanceForSavedSweepRebuild(label,appearance)) return {};
        const double constructionScale=state.loft.definition.constructionFrame
            ?state.loft.definition.constructionFrame->values[7]:1.0;
        const double effective=state.loft.definition.dimensionMetersPerUnit*std::abs(constructionScale)*std::abs(state.scalars[7]);
        const auto guard=_ordinaryEditController?_ordinaryEditController->captureSavedSweepRebuildSource():nullptr;
        const auto after=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        if (!guard || !after || !(*after==*stamp) || !std::isfinite(effective) || effective<=0
            || !std::isfinite(effective*1000)) return {};
        std::shared_ptr<const retained_edge_treatment::Snapshot> treatment;
        {
            namespace et=retained_edge_treatment;auto capture=std::shared_ptr<et::Snapshot>(new et::Snapshot);
            capture->ownerLabel_=label;capture->sourceLabel_=state.loft.label;capture->sourceIdentifier_=state.loft.identifier;capture->source_=state.loft.definition;capture->current_=state.shape;capture->base_=state.edgeTreatment?state.edgeTreatment->value->base:state.shape;capture->nonce_=std::uint64_t(myDoc->Document()->GetData()->Time());capture->presentationRevision_=presentationRevision;
            auto&seed=capture->seed_;seed.schema=1;receipt::ParseUUID(myDoc->DocumentIdentifier(),seed.owner.document);receipt::ParseUUID(state.entityIdentifier,seed.owner.entity);receipt::ParseUUID(state.definitionIdentifier,seed.owner.definition);seed.base.source.document=seed.owner.document;seed.base.source.entity=seed.owner.entity;seed.base.source.definition=seed.owner.definition;receipt::ParseUUID(state.loft.identifier,seed.base.source.sourceFeature);seed.base.sourceNode=MintEdgeTreatmentUUID();seed.base.family=et::SourceFamily::RectangularLoft;seed.base.sourceSchema=std::uint32_t(loft_persistence::Schema);seed.base.metersPerLocalUnit=effective;seed.outputNode=seed.base.sourceNode;seed.issuance.nextLocalID=1;composite_recipe::EncodeScalarRecipe(composite_recipe::RecipeKind::RectangularLoft,seed.base.sourceSchema,state.loft.values,capture->sourceBytes_);composite_recipe::Hash(capture->sourceBytes_,seed.base.sourceRecipeDigest);
            if(state.edgeTreatment){capture->definition_=state.edgeTreatment->value->definition;capture->definitionBytes_=state.edgeTreatment->value->bytes;capture->seed_=*capture->definition_;TopoDS_Shape replayed;std::vector<et::StepProof>proofs;et::Refusal replayRefusal;if(!et::Replay(capture->base_,*capture->definition_,replayed,proofs,capture->chargedBudget_,replayRefusal)||!et::EquivalentReplayGeometry(capture->current_,replayed,capture->chargedBudget_,replayRefusal))return {};}
            capture->owner_.owner=seed.owner;capture->owner_.outputNode=capture->seed_.outputNode;capture->owner_.status=retained_recipe::OwnerStatus::CurrentEditable;treatment=std::move(capture);
        }
        StoredRectangularLoftSnapshot result;result.definition=state.loft.definition;result.identity=identity;
        result.definitionIdentifier=state.definitionIdentifier;result.featureIdentifier=state.loft.identifier;
        result.sourceState=std::move(state);result.authorityStamp=*stamp;result.sourceGuard=guard;
        result.effectiveDimensionMetersPerUnit=effective;result.current=true;result.edgeTreatment=std::move(treatment);return result;
    } catch (...) {return {};}
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareStoredLoftStationRebuild(
    const rectangular_loft::StationDimensionEdit& edit,const StoredRectangularLoftSnapshot& original,
    const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height) noexcept {
    if (![NSThread isMainThread] || !original.current || identity.entityIdentifier!=original.identity.entityIdentifier
        || identity.publicationSourceIdentifier!=original.identity.publicationSourceIdentifier
        || identity.documentGeneration!=original.identity.documentGeneration || identity.modelRevision!=original.identity.modelRevision) return {};
    try {
        rectangular_loft::Definition definition;
        if(!loft_rebuild::Apply(original.definition,edit,definition))return {};
        const auto current=storedRectangularLoftDefinition(identity,presentationRevision,width,height);
        if (!current || !current->current || !(current->authorityStamp==original.authorityStamp)
            || current->featureIdentifier!=original.featureIdentifier || current->definitionIdentifier!=original.definitionIdentifier
            || current->effectiveDimensionMetersPerUnit!=original.effectiveDimensionMetersPerUnit
            || bool(current->edgeTreatment)!=bool(original.edgeTreatment)
            || (current->edgeTreatment&&original.edgeTreatment
                && current->edgeTreatment->definitionBytes_!=original.edgeTreatment->definitionBytes_)
            || !current->sourceState.IsEqual(original.sourceState)
            || !sweep_rebuild::SameRawScalars(current->sourceState.scalars,original.sourceState.scalars)
            || !_ordinaryEditController || !_ordinaryEditController->savedSweepRebuildSourceIsCurrent(original.sourceGuard)) return {};
        rectangular_loft::Admission admission;const auto prepared=rectangular_loft::Prepare(definition,admission);
        if (!prepared) return {};
        auto work=prepareNativeSolidWork(identity,presentationRevision,width,height);
        if (!work || sweep_persistence::Bits(work->metersPerUnit)!=sweep_persistence::Bits(definition.dimensionMetersPerUnit))return {};
        myContext->InitSelected();const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        if(selected.IsNull())return {};
        auto geometry=std::make_shared<LoftSolidGeometry>();geometry->prepared=prepared;
        if(original.edgeTreatment&&original.edgeTreatment->definition_){
            namespace et=retained_edge_treatment;
            // A source edit moves native edges; the stored selector witnesses
            // must be verified against the old source and rebound against the
            // actually rebuilt stage, never replayed with stale midpoints.
            et::SourceRebindRoles roles;et::ReplayBudget rebindBudget=original.edgeTreatment->chargedBudget_;et::Refusal rebindRefusal=et::Refusal::ReplayMismatch;
            if(!et::CaptureSourceRebindRoles(original.edgeTreatment->base_,*original.edgeTreatment->definition_,
                original.edgeTreatment->definitionBytes_,rebindBudget,rebindRefusal,roles))return {};
            geometry->treatmentRebind=std::move(roles);geometry->treatmentRebuildSource=definition;geometry->treatmentBudget=rebindBudget;
            work->edgeTreatmentSnapshot=original.edgeTreatment;work->edgeTreatmentEdit=retained_edge_treatment::RebuildSource{definition};
        }
        work->geometry=std::move(geometry);
        OrdinaryTransformRecord record;record.previous=current->sourceState;
        record.requested.label=record.previous.label;record.requested.presentation=selected;
        record.requested.shape=record.previous.shape;record.requested.transform=record.previous.transform;
        record.requested.operation=OrdinaryTransformOperation::LoftStationRebuild;record.requested.loftRebuild=definition;
        record.requested.loftStationEdit=edit;
        record.requested.sweepSource=original.sourceGuard;
        work->rebuildAuthority.emplace();work->rebuildAuthority->records.push_back(std::move(record));
        if (!admitTransform(*work->rebuildAuthority)) return {};
        work->loftRebuildStamp=original.authorityStamp;work->frameFirst=false;return work;
    } catch (...) {return {};}
}

std::optional<CylindricalCutSnapshot> Core3DViewer::cylindricalCutSource(
    const ObjectFrameIdentity& identity,std::uint64_t presentation,std::uint32_t width,std::uint32_t height)noexcept {
    if(!NSThread.isMainThread||!canBeginCommittedEdit()||myDoc.IsNull()||myContext.IsNull()||identity.entityIdentifier.empty()||!width||!height)CORE3D_CUT_REFUSE("capture.context", {});
    try {
        const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());const auto snapshot=captureSceneSnapshot(width,height);
        if(!stamp||!snapshot||snapshot->selectionMode!=scene::ElementKind::Object
            ||snapshot->publicationSourceIdentifier!=identity.publicationSourceIdentifier
            ||snapshot->revisions.documentGeneration!=identity.documentGeneration||snapshot->revisions.model!=identity.modelRevision
            ||snapshot->revisions.presentation!=presentation)CORE3D_CUT_REFUSE("capture.scene-context", {});
        myContext->InitSelected();if(!myContext->MoreSelected())CORE3D_CUT_REFUSE("capture.selection-empty", {});
        const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
        if(selected.IsNull()||myContext->MoreSelected())CORE3D_CUT_REFUSE("capture.selection-count", {});
        CylindricalCutSnapshot result;const auto label=myDoc->ShapeLabel(selected);
        if(!myDoc->CaptureCylindricalCutSource(label,result.source)||result.source.original.entityIdentifier!=identity.entityIdentifier)CORE3D_CUT_REFUSE("capture.document-source-or-identity", {});
        result.guard=myDoc->CaptureSavedCutSceneState(label);const auto after=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        if(!result.guard||!after||!(*after==*stamp))CORE3D_CUT_REFUSE("capture.scene-state-or-stamp", {});
        result.identity=identity;result.authorityStamp=*stamp;return result;
    }catch(...){CORE3D_CUT_REFUSE("capture.exception", {});}
}
std::shared_ptr<NativeSolidWork> Core3DViewer::prepareCylindricalCut(const CylindricalCutSnapshot& original,
    const std::optional<cylindrical_cut::CreateEdit>& create,const std::optional<cylindrical_cut::RadiusEdit>& radius,
    const ObjectFrameIdentity& identity,std::uint64_t presentation,std::uint32_t width,std::uint32_t height)noexcept {
    if(!NSThread.isMainThread||bool(create)==bool(radius)||original.source.rebuilding!=bool(radius)
        ||identity.entityIdentifier!=original.identity.entityIdentifier||identity.publicationSourceIdentifier!=original.identity.publicationSourceIdentifier
        ||identity.documentGeneration!=original.identity.documentGeneration||identity.modelRevision!=original.identity.modelRevision)CORE3D_CUT_REFUSE("prepare.context-or-operation", {});
    try {
        const auto current=cylindricalCutSource(identity,presentation,width,height);
        if(!current||!(current->authorityStamp==original.authorityStamp)||!current->source.original.IsEqual(original.source.original)
            ||!sweep_rebuild::SameRawScalars(current->source.original.scalars,original.source.original.scalars)
            ||current->source.effectiveMM!=original.source.effectiveMM||!myDoc->SavedCutSceneStateMatches(original.guard))CORE3D_CUT_REFUSE("prepare.stale-source", {});
        auto envelope=original.source.envelope;
        if(create){
            envelope.axis=static_cast<std::uint8_t>(create->axis);envelope.point=create->localCenter;envelope.operandID=1;
            if(!receipt::ParseUUID(NSUUID.UUID.UUIDString.UTF8String,envelope.derivedFeature)
                ||!cylindrical_cut::Radius(create->worldRadiusMM,original.source.effectiveMM,envelope.radius))CORE3D_CUT_REFUSE("prepare.create-radius-or-identity", {});
        }else if(!cylindrical_cut::Rebuild(original.source.envelope,*radius,original.source.effectiveMM,envelope))CORE3D_CUT_REFUSE("prepare.radius-edit", {});
        auto carrier=std::make_shared<retained_solid::Payload>();carrier->envelope=envelope;carrier->base=original.source.base;
        if(!retained_solid::Encode(envelope,carrier->bytes)||!analytic_boolean::Inspect(cylindrical_cut::Recipe(envelope)))CORE3D_CUT_REFUSE("prepare.envelope-or-recipe", {});
        // Circular and loft hosts require a separated through-bore. Refuse unsupported
        // recipe clearance at admission, before copying or building geometry;
        // the legacy polygon/enclosure route also supports notches/side cuts.
        bool circularHost=false;
        if(envelope.sourceFamily==1){profile::Parameters source;
            if(!profile::Decode(envelope.sourceValues,source))CORE3D_CUT_REFUSE("prepare.profile-decode", {});
            circularHost=bool(source.definition.circle)||source.definition.revolve;
        }
        if(circularHost||envelope.sourceFamily==3){
            const auto clearance=saved_cut_bore_clearance::Inspect(envelope);
            if(clearance.status!=saved_cut_bore_clearance::Status::ClearRecipeDisk
                &&clearance.status!=saved_cut_bore_clearance::Status::ClearRecipeTransverse){
                CORE3D_CUT_DETAIL("clearance.source-disk",clearance.status);return {};
            }
        }
        // Creation keeps the original source UUID and adds one derived UUID;
        // charge both in the existing global namespace before the geometry copy.
        std::vector<profile::Record> profiles;std::vector<enclosure::Record> enclosures;
        std::vector<sweep_persistence::Record> sweeps;std::vector<loft_persistence::Record> lofts;
        std::vector<retained_solid::Record> retained;
        if(!saved_features::Validate(myDoc->Document(),profiles,enclosures,sweeps,lofts,&retained))CORE3D_CUT_REFUSE("prepare.feature-census", {});
        const auto count=profiles.size()+enclosures.size()+sweeps.size()+lofts.size()+retained.size()*2;
        if(create&&count>=std::size_t(profile::MaximumRecords))CORE3D_CUT_REFUSE("prepare.feature-budget", {});
        const auto newID=retained_solid::UUIDText(envelope.derivedFeature);
        if(create){
            for(const auto& r:profiles)if(r.identifier==newID)CORE3D_CUT_REFUSE("prepare.profile-uuid-collision", {});
            for(const auto& r:enclosures)if(r.identifier==newID)CORE3D_CUT_REFUSE("prepare.enclosure-uuid-collision", {});
            for(const auto& r:sweeps)if(r.identifier==newID)CORE3D_CUT_REFUSE("prepare.sweep-uuid-collision", {});
            for(const auto& r:lofts)if(r.identifier==newID)CORE3D_CUT_REFUSE("prepare.loft-uuid-collision", {});
            for(const auto& r:retained){const auto identity=retained_boolean::Identities(r.value->envelope);
                if(identity.sourceFeature==envelope.derivedFeature||identity.derivedFeature==envelope.derivedFeature)CORE3D_CUT_REFUSE("prepare.retained-uuid-collision", {});}
        }
        std::size_t envelopeBytes=carrier->bytes.size();
        for(const auto& r:retained){
            if(radius&&r.owner.IsEqual(original.source.original.label))continue;
            if(r.value->bytes.size()>retained_solid::MaximumAggregateEnvelopeBytes-envelopeBytes)CORE3D_CUT_REFUSE("prepare.envelope-budget", {});
            envelopeBytes+=r.value->bytes.size();
        }
        std::atomic_bool stopped{false};if(!analytic_boolean::detail::Bounded(original.source.base,1024,stopped))CORE3D_CUT_REFUSE("prepare.base-budget", {});
        // The main-only immutable carrier keeps the actual original. Only a
        // deep geometry copy with no live labels/materials reaches the worker.
        BRepBuilderAPI_Copy copy(original.source.base,Standard_True,Standard_False);if(!copy.IsDone())CORE3D_CUT_REFUSE("prepare.base-copy", {});
        auto geometry=std::make_shared<CutSolidGeometry>();geometry->detachedBase=copy.Shape();geometry->recipe=cylindrical_cut::Recipe(envelope);
        if(circularHost||envelope.sourceFamily==3)geometry->provenHost=envelope;
        geometry->rebuildLoftBase=envelope.sourceFamily==3&&!original.source.rebuilding;
        myContext->InitSelected();const auto displaySource=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        if(displaySource.IsNull()||!cut_display::Capture(displaySource->Attributes(),geometry->displaySettings))CORE3D_CUT_REFUSE("prepare.display-settings", {});
        if(geometry->detachedBase.IsNull()||geometry->detachedBase.IsPartner(original.source.base))CORE3D_CUT_REFUSE("prepare.detached-base", {});
        auto work=prepareNativeSolidWork(identity,presentation,width,height);
        if(!work||retained_solid::Bits(work->metersPerUnit)!=retained_solid::Bits(envelope.metersPerUnit))CORE3D_CUT_REFUSE("prepare.work-or-units", {});
        myContext->InitSelected();const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());if(selected.IsNull())CORE3D_CUT_REFUSE("prepare.selection", {});
        OrdinaryTransformRecord record;record.previous=original.source.original;
        record.requested.label=record.previous.label;record.requested.presentation=selected;record.requested.shape=record.previous.shape;
        record.requested.transform=record.previous.transform;record.requested.operation=OrdinaryTransformOperation::CylindricalCut;
        record.requested.cut=carrier;record.requested.cutSource=original.guard;
        work->rebuildAuthority.emplace();work->rebuildAuthority->records.push_back(std::move(record));
        if(!admitTransform(*work->rebuildAuthority))CORE3D_CUT_REFUSE("prepare.transform-authority", {});
        const auto after=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        if(!after||!(*after==original.authorityStamp)||!myDoc->SavedCutSceneStateMatches(original.guard))CORE3D_CUT_REFUSE("prepare.final-scene-state", {});
        work->geometry=geometry;work->cutStamp=original.authorityStamp;work->frameFirst=false;return work;
    }catch(...){CORE3D_CUT_REFUSE("prepare.exception", {});}
}

std::optional<CylindricalCutProgramSnapshot> Core3DViewer::cylindricalCutProgramSource(
    const ObjectFrameIdentity& identity,std::uint64_t presentation,std::uint32_t width,std::uint32_t height)noexcept {
    if(!NSThread.isMainThread||!canBeginCommittedEdit()||myDoc.IsNull()||myContext.IsNull()||identity.entityIdentifier.empty()||!width||!height)CORE3D_CUT_REFUSE("program-capture.context", {});
    try {
        const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());const auto snapshot=captureSceneSnapshot(width,height);
        if(!stamp||!snapshot||snapshot->selectionMode!=scene::ElementKind::Object
            ||snapshot->publicationSourceIdentifier!=identity.publicationSourceIdentifier
            ||snapshot->revisions.documentGeneration!=identity.documentGeneration||snapshot->revisions.model!=identity.modelRevision
            ||snapshot->revisions.presentation!=presentation)CORE3D_CUT_REFUSE("program-capture.scene-context", {});
        myContext->InitSelected();if(!myContext->MoreSelected())CORE3D_CUT_REFUSE("program-capture.selection-empty", {});
        const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
        if(selected.IsNull()||myContext->MoreSelected())CORE3D_CUT_REFUSE("program-capture.selection-count", {});
        CylindricalCutProgramSnapshot result;const auto label=myDoc->ShapeLabel(selected);
        if(!myDoc->CaptureCylindricalCutProgramSource(label,result.source)||result.source.original.entityIdentifier!=identity.entityIdentifier)CORE3D_CUT_REFUSE("program-capture.document-source-or-identity", {});
        result.guard=myDoc->CaptureSavedCutSceneState(label);const auto after=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        if(!result.guard||!after||!(*after==*stamp))CORE3D_CUT_REFUSE("program-capture.scene-state-or-stamp", {});
        result.identity=identity;result.authorityStamp=*stamp;return result;
    }catch(...){CORE3D_CUT_REFUSE("program-capture.exception", {});}
}
retained_fillet::Candidates Core3DViewer::retainedFilletCandidates(const CylindricalCutProgramSnapshot& original,
    const ObjectFrameIdentity& identity,std::uint64_t presentation,std::uint32_t width,std::uint32_t height) noexcept {
    using namespace retained_fillet;
    if(!NSThread.isMainThread)return {CandidateStatus::Stale,false,{}};
    try {
        const auto current=cylindricalCutProgramSource(identity,presentation,width,height);
        if(!current||!(current->authorityStamp==original.authorityStamp)||!current->source.original.IsEqual(original.source.original)
            ||!sweep_rebuild::SameRawScalars(current->source.original.scalars,original.source.original.scalars)
            ||current->source.effectiveMM!=original.source.effectiveMM||current->source.recipeBytes!=original.source.recipeBytes
            ||!myDoc->SavedCutSceneStateMatches(original.guard))return {CandidateStatus::Stale,false,{}};
        auto result=myDoc->RetainedFilletCandidates(original.source.original.label);
        const auto after=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        if(!after||!(*after==original.authorityStamp)||!myDoc->SavedCutSceneStateMatches(original.guard))
            return {CandidateStatus::Stale,false,{}};
        return result;
    }catch(...){return {CandidateStatus::Failed,false,{}};}
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareCylindricalCutProgramEdit(const CylindricalCutProgramSnapshot& original,
    const retained_boolean::ProgramEdit& edit,
    const ObjectFrameIdentity& identity,std::uint64_t presentation,std::uint32_t width,std::uint32_t height)noexcept {
    if(!NSThread.isMainThread)CORE3D_CUT_REFUSE("program-prepare.thread", {});
    lastRetainedFilletAdmissionOutcome=retained_fillet::Outcome::Generic;
    if(identity.entityIdentifier!=original.identity.entityIdentifier||identity.publicationSourceIdentifier!=original.identity.publicationSourceIdentifier
        ||identity.documentGeneration!=original.identity.documentGeneration||identity.modelRevision!=original.identity.modelRevision)CORE3D_CUT_REFUSE("program-prepare.context", {});
    try {
        // Revalidate against a freshly captured original; stale input is never
        // refreshed onto a different target.
        const auto current=cylindricalCutProgramSource(identity,presentation,width,height);
        if(!current||!(current->authorityStamp==original.authorityStamp)||!current->source.original.IsEqual(original.source.original)
            ||!sweep_rebuild::SameRawScalars(current->source.original.scalars,original.source.original.scalars)
            ||current->source.effectiveMM!=original.source.effectiveMM
            ||current->source.recipeBytes!=original.source.recipeBytes
            ||!myDoc->SavedCutSceneStateMatches(original.guard))CORE3D_CUT_REFUSE("program-prepare.stale-source", {});
        if(const auto* append=std::get_if<retained_boolean::AppendFilletStep>(&edit)){
            std::vector<retained_fillet::EdgeAnchor> captured;
            if(!myDoc->CaptureRetainedFilletAnchors(original.source.original.label,append->anchors,captured,&lastRetainedFilletAdmissionOutcome))return {};
        }
        if(retained_boolean::FilletEdit(edit)){
            // Report typed value/budget failures before Apply's generic refusal.
            retained_boolean::Program p;
            if(const auto* legacy=std::get_if<retained_boolean::Legacy>(&original.source.recipe)){
                if(!retained_boolean::Promote(*legacy,p))return {};
            }else p=std::get<retained_boolean::Program>(original.source.recipe);
            const auto* append=std::get_if<retained_boolean::AppendFilletStep>(&edit);
            const auto* radius=std::get_if<retained_boolean::SetFilletRadius>(&edit);
            if(append){std::size_t count=append->anchors.size();for(const auto& step:p.filletSteps)count+=step.anchors.size();
                if(p.filletSteps.size()>=retained_fillet::MaximumSteps||count>retained_fillet::MaximumAnchors
                    ||p.nextFilletStepID==UINT64_MAX||p.nextFilletEdgeID>UINT64_MAX-append->anchors.size()){
                    lastRetainedFilletAdmissionOutcome=retained_fillet::Outcome::DeclinedBudget;return {};}}
            if(append||radius){double local=0;
                if(!cylindrical_cut::Radius(append?append->radiusMM:radius->radiusMM,original.source.effectiveMM,local)){
                    lastRetainedFilletAdmissionOutcome=retained_fillet::Outcome::DeclinedRadiusAdmission;return {};}}
            lastRetainedFilletAdmissionOutcome=retained_fillet::Outcome::Generic;
        }
        const auto change=retained_boolean::Apply(original.source.recipe,edit,original.source.effectiveMM);
        if(!change){
            if(retained_boolean::RingEdit(edit)){
                retained_boolean::Program diagnostic;
                if(const auto* legacy=std::get_if<retained_boolean::Legacy>(&original.source.recipe))retained_boolean::Promote(*legacy,diagnostic);
                else diagnostic=std::get<retained_boolean::Program>(original.source.recipe);
                if(const auto* append=std::get_if<retained_boolean::AppendRing>(&edit)){
                    analytic_boolean::Operand candidate;candidate.kind=analytic_boolean::OperandKind::CylinderRing;
                    candidate.axis=append->edit.axis;candidate.point=append->edit.localCenter;
                    candidate.boltCircleRadius=append->edit.boltCircleRadius;candidate.radius=append->edit.worldHoleRadiusMM/original.source.effectiveMM;
                    candidate.count=append->edit.count;double extent=0;
                    if(saved_boolean_result::detail::HostRadialExtent(diagnostic,candidate,extent))candidate.hostRadiusRatio=candidate.boltCircleRadius/extent;
                    const auto status=saved_boolean_result::detail::RingAdmissionStatus(diagnostic,candidate);
                    if(status==analytic_boolean_ring::Status::OverlappingHoles)CORE3D_CUT_REFUSE("ring.OverlappingHoles", {});
                    if(status==analytic_boolean_ring::Status::OutsideOrInsufficientLigament)CORE3D_CUT_REFUSE("ring.OutsideOrInsufficientLigament", {});
                }
            }
            CORE3D_CUT_REFUSE("program-prepare.apply-edit", {});
        }
        const auto* program=std::get_if<retained_boolean::Program>(&change->recipe);
        // This packet emits only v2 programs. A legacy one-bore radius keeps
        // its exact legacy API; nothing is silently downgraded or reissued.
        if(!program||!retained_boolean::Valid(*program))CORE3D_CUT_REFUSE("program-prepare.program-validation", {});
        // Derived disk budget and all pair/source clearances are admission
        // obligations, before a work item or ordinary history lease exists.
        if(retained_boolean::HasWedge(*program)){
            if(!saved_boolean_result::detail::AdmitSections(*program))CORE3D_CUT_REFUSE("program-clearance.convex-sections-or-host", {});
        }else if(retained_boolean::HasRing(*program)?!saved_boolean_result::detail::AdmitDisks(*program):
            (program->steps.size()>=2&&!saved_boolean_result::detail::SeparateDisks(*program)))CORE3D_CUT_REFUSE("program-clearance.disk-budget-or-separation", {});
        // Pair separation alone says nothing about containment by the host.
        // Check every plain bore against the unchanged source at admission,
        // including an append to an already retained loft/circle program.
        if(!retained_boolean::HasRing(*program)&&!retained_boolean::HasWedge(*program)){
            const bool singleTransverseFillet=saved_boolean_result::detail::SingleTransverseFilletCarrier(*program);
            for(const auto& step:program->steps){
                const auto clearance=saved_cut_bore_clearance::Inspect(
                    saved_boolean_result::detail::GeometryView(*program,step.operand));
                if(clearance.status!=saved_cut_bore_clearance::Status::ClearRecipeDisk
                    &&!(singleTransverseFillet
                        &&clearance.status==saved_cut_bore_clearance::Status::ClearRecipeTransverse)){
                    CORE3D_CUT_DETAIL("program-clearance.source-disk",clearance.status);return {};
                }
            }
        }
        auto carrier=std::make_shared<retained_solid::Payload>();carrier->envelope=change->recipe;
        carrier->bytes=change->newBytes;carrier->base=original.source.base;
        // Append/radius keeps the retained source/derived UUIDs; no new feature
        // identity is issued. Only the persisted high-water ID may advance.
        std::vector<profile::Record> profiles;std::vector<enclosure::Record> enclosures;
        std::vector<sweep_persistence::Record> sweeps;std::vector<loft_persistence::Record> lofts;
        std::vector<retained_solid::Record> retained;
        if(!saved_features::Validate(myDoc->Document(),profiles,enclosures,sweeps,lofts,&retained))CORE3D_CUT_REFUSE("program-prepare.feature-census", {});
        std::size_t envelopeBytes=carrier->bytes.size();
        for(const auto& r:retained){
            if(r.owner.IsEqual(original.source.original.label))continue;
            if(r.value->bytes.size()>retained_solid::MaximumAggregateEnvelopeBytes-envelopeBytes){
                lastRetainedFilletAdmissionOutcome=retained_fillet::Outcome::DeclinedBudget;CORE3D_CUT_REFUSE("program-prepare.envelope-budget", {});}
            envelopeBytes+=r.value->bytes.size();
        }
        std::atomic_bool stopped{false};if(!analytic_boolean::detail::Bounded(original.source.base,1024,stopped)){
            lastRetainedFilletAdmissionOutcome=retained_fillet::Outcome::DeclinedBudget;CORE3D_CUT_REFUSE("program-prepare.base-budget", {});}
        // The main-only immutable carrier keeps the actual original. Only a
        // deep geometry copy with no live labels/materials reaches the worker.
        BRepBuilderAPI_Copy copy(original.source.base,Standard_True,Standard_False);if(!copy.IsDone())CORE3D_CUT_REFUSE("program-prepare.base-copy", {});
        auto geometry=std::make_shared<CutProgramGeometry>();geometry->detachedBase=copy.Shape();
        geometry->program=*program;geometry->expectedBytes=change->newBytes;geometry->selectedOperandID=change->selectedOperandID;
        myContext->InitSelected();const auto displaySource=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        if(displaySource.IsNull()||!cut_display::Capture(displaySource->Attributes(),geometry->displaySettings))CORE3D_CUT_REFUSE("program-prepare.display-settings", {});
        if(geometry->detachedBase.IsNull()||geometry->detachedBase.IsPartner(original.source.base))CORE3D_CUT_REFUSE("program-prepare.detached-base", {});
        auto work=prepareNativeSolidWork(identity,presentation,width,height);
        if(!work||retained_solid::Bits(work->metersPerUnit)!=retained_solid::Bits(program->source.metersPerUnit))CORE3D_CUT_REFUSE("program-prepare.work-or-units", {});
        myContext->InitSelected();const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());if(selected.IsNull())CORE3D_CUT_REFUSE("program-prepare.selection", {});
        OrdinaryTransformRecord record;record.previous=original.source.original;
        record.requested.label=record.previous.label;record.requested.presentation=selected;record.requested.shape=record.previous.shape;
        record.requested.transform=record.previous.transform;record.requested.operation=retained_boolean::FilletEdit(edit)?OrdinaryTransformOperation::RetainedFillet:
            retained_boolean::WedgeEdit(edit)?OrdinaryTransformOperation::WedgeCut:
            retained_boolean::RingEdit(edit)?OrdinaryTransformOperation::CylindricalCutRing:OrdinaryTransformOperation::CylindricalCut;
        record.requested.cut=carrier;record.requested.cutSource=original.guard;record.requested.cutProgramEdit=edit;
        work->rebuildAuthority.emplace();work->rebuildAuthority->records.push_back(std::move(record));
        if(!admitTransform(*work->rebuildAuthority))CORE3D_CUT_REFUSE("program-prepare.transform-authority", {});
        const auto after=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
        if(!after||!(*after==original.authorityStamp)||!myDoc->SavedCutSceneStateMatches(original.guard))CORE3D_CUT_REFUSE("program-prepare.final-scene-state", {});
        work->geometry=geometry;work->cutStamp=original.authorityStamp;work->frameFirst=false;return work;
    }catch(...){CORE3D_CUT_REFUSE("program-prepare.exception", {});}
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareWedgeCutProgramEdit(const CylindricalCutProgramSnapshot& original,
    const retained_boolean::ProgramEdit& edit,const ObjectFrameIdentity& identity,std::uint64_t presentation,
    std::uint32_t width,std::uint32_t height)noexcept {
    if(!retained_boolean::WedgeEdit(edit))CORE3D_CUT_REFUSE("program-prepare.wedge-edit-kind", {});
    return prepareCylindricalCutProgramEdit(original,edit,identity,presentation,width,height);
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareRetainedFilletEdit(const CylindricalCutProgramSnapshot& original,
    const retained_boolean::ProgramEdit& edit,const ObjectFrameIdentity& identity,std::uint64_t presentation,
    std::uint32_t width,std::uint32_t height)noexcept {
    if(!retained_boolean::FilletEdit(edit))CORE3D_CUT_REFUSE("program-prepare.fillet-edit-kind", {});
    return prepareCylindricalCutProgramEdit(original,edit,identity,presentation,width,height);
}

std::shared_ptr<NativeSolidWork> Core3DViewer::prepareCylindricalCutRingEdit(const CylindricalCutProgramSnapshot& original,
    const retained_boolean::ProgramEdit& edit,const ObjectFrameIdentity& identity,std::uint64_t presentation,
    std::uint32_t width,std::uint32_t height)noexcept {
    if(!retained_boolean::RingEdit(edit))CORE3D_CUT_REFUSE("program-prepare.ring-edit-kind", {});
    return prepareCylindricalCutProgramEdit(original,edit,identity,presentation,width,height);
}

#if DEBUG
std::map<std::string,bool> Core3DViewer::debugSavedCutSourceViewerQualification(const CylindricalCutSnapshot& original,
    const ObjectFrameIdentity& identity,std::uint64_t presentation,std::uint32_t width,std::uint32_t height) {
    if(!NSThread.isMainThread||myContext.IsNull()||myDoc.IsNull())return {{"fixture",false}};
    try {
        myContext->InitSelected();if(!myContext->MoreSelected())return {{"fixture",false}};
        const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
        cut_display::Settings settings;
        if(selected.IsNull()||myContext->MoreSelected()||!selected->Shape().IsEqual(original.source.original.shape)
            ||!cut_display::Capture(selected->Attributes(),settings))return {{"fixture",false}};
        auto checks=saved_cut_source_viewer_probe::MeshIsolation(original.source.base,original.source.envelope,settings);
        const auto lifecycle=saved_cut_source_viewer_probe::Run(*this,original,identity,presentation,width,height);
        for(const auto& row:lifecycle)checks.emplace(row.first,row.second);
        return checks;
    }catch(...){return {{"fixture",false}};}
}
bool Core3DViewer::debugSetCutDisplayCoefficient(double coefficient,bool pending) noexcept {
    if(!NSThread.isMainThread||myContext.IsNull()||myDoc.IsNull()||myDoc->Document().IsNull()
        ||myDoc->Document()->HasOpenCommand()||!std::isfinite(coefficient)||coefficient<=0)return false;
    try {
        myContext->InitSelected();if(!myContext->MoreSelected())return false;
        const auto selected=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());myContext->NextSelected();
        if(selected.IsNull()||myContext->MoreSelected()||selected->Attributes().IsNull())return false;
        const auto drawer=selected->Attributes();drawer->SetDeviationCoefficient(coefficient);
        if(!pending){
            // Settle through actual display/selection work before any source
            // snapshot is captured. Do not pretend a pending change was applied.
            myContext->Redisplay(selected,Standard_False);
            myContext->RecomputeSelectionOnly(selected);
            myContext->UpdateCurrentViewer();
            cut_display::Settings settings;if(!cut_display::Capture(drawer,settings))return false;
        }
        return true;
    }catch(...){return false;}
}
#endif

std::shared_ptr<ProfileSolidGeometry> Core3DViewer::profileSolidGeometry(
    const std::shared_ptr<NativeSolidWork>& work) noexcept {
    if (!work) return {};
    const auto payload=std::get_if<std::shared_ptr<ProfileSolidGeometry>>(&work->geometry);
    return payload ? *payload : nullptr;
}
bool Core3DViewer::buildProfileSolidGeometry(const std::shared_ptr<ProfileSolidGeometry>& geometry) noexcept {
    return BuildProfileSolidGeometry(geometry);
}
NativeSolidGeometryPayload Core3DViewer::nativeSolidGeometry(const std::shared_ptr<NativeSolidWork>& work) noexcept {
    return work ? work->geometry : NativeSolidGeometryPayload{};
}
bool Core3DViewer::buildNativeSolidGeometry(const NativeSolidGeometryPayload& payload,
    retained_fillet::Outcome* filletOutcome) noexcept {
    if(filletOutcome)*filletOutcome=retained_fillet::Outcome::Built;
    if(const auto p=std::get_if<std::shared_ptr<CutSolidGeometry>>(&payload)) {
        if(!*p||(*p)->built||(*p)->cancelled.load())CORE3D_CUT_REFUSE("build.state-or-cancelled", false);
        if((*p)->rebuildLoftBase){
            if(!(*p)->provenHost)CORE3D_CUT_REFUSE("build.loft-host-missing", false);
            // A current planar loft already has the full source boundary.
            // Keep its original carrier/base content; regenerating it would
            // break the independent exact retained-base seal after meshing.
            if(saved_boolean_build::InspectSourceBase((*p)->detachedBase,*(*p)->provenHost,(*p)->cancelled))
                (*p)->rebuildLoftBase=false;
        }
        if((*p)->rebuildLoftBase){
            TopoDS_Shape planar;
            (*p)->loftBaseStatus=saved_cut_source_edit::RebuildLoftBase(*(*p)->provenHost,(*p)->cancelled,planar);
            if((*p)->loftBaseStatus!=saved_cut_source_edit::LoftBaseStatus::Built){
                CORE3D_CUT_DETAIL("build.loft-base",(*p)->loftBaseStatus);return false;
            }
            (*p)->detachedBase=planar;
        }
        const auto status=analytic_boolean::Build((*p)->detachedBase,(*p)->recipe,(*p)->cancelled,(*p)->result);
        if(status!=analytic_boolean::Status::Built){CORE3D_CUT_DETAIL("build.boolean",status);return false;}
        if(!cut_display::Prepare((*p)->result.solid,(*p)->displaySettings,(*p)->cancelled))
            CORE3D_CUT_REFUSE("build.display", false);
        if((*p)->provenHost){const auto& source=*(*p)->provenHost;
            if(!saved_boolean_build::InspectSourceBase((*p)->detachedBase,source,(*p)->cancelled))
                CORE3D_CUT_REFUSE("proof.base", false);
            if(saved_cut_bore_clearance::Inspect(source).status==saved_cut_bore_clearance::Status::ClearRecipeTransverse){
                analytic_boolean::Result replay;
                if(analytic_boolean::Build((*p)->detachedBase,cylindrical_cut::Recipe(source),(*p)->cancelled,replay)!=analytic_boolean::Status::Built
                    ||!saved_cut_bore_clearance::VerifyTransverseResult((*p)->detachedBase,(*p)->result.solid,source,(*p)->cancelled,replay.solid))
                    CORE3D_CUT_REFUSE("proof.transverse-replay", false);
            }else{
                const auto proof=saved_boolean_result::InspectAxialBore((*p)->result.solid,source,source,(*p)->cancelled);
                if(proof.classification!=saved_cut_whole_result::Classification::MatchedOrientedBoundary){
                    CORE3D_CUT_DETAIL("proof.whole-result",proof.classification);return false;
                }
            }
        }
        (*p)->built=true;return true;
    }
    if(const auto p=std::get_if<std::shared_ptr<CutProgramGeometry>>(&payload)) {
        if(!*p||(*p)->built||(*p)->cancelled.load())CORE3D_CUT_REFUSE("program-build.state-or-cancelled", false);
        // Whole-program detached build retains the complete boundary proof.
        (*p)->result=saved_boolean_build::Build((*p)->detachedBase,(*p)->program,(*p)->displaySettings,(*p)->cancelled);
        if(filletOutcome)*filletOutcome=(*p)->result.filletOutcome;
        if((*p)->result.status!=saved_boolean_build::Status::Built){
            CORE3D_CUT_DETAIL("program-build.status",(*p)->result.status);return false;
        }
        if((*p)->result.exactProgram!=(*p)->expectedBytes)CORE3D_CUT_REFUSE("program-build.exact-bytes", false);
        (*p)->built=true;return true;
    }
    if (const auto p=std::get_if<std::shared_ptr<LoftSolidGeometry>>(&payload)) {
        if (!*p || (*p)->built || (*p)->cancelled.load()) return false;
        (*p)->built=rectangular_loft::Build((*p)->prepared,(*p)->cancelled,(*p)->result)==rectangular_loft::BuildStatus::Built;
        if((*p)->built&&(*p)->treatmentRebind&&(*p)->treatmentRebuildSource){(*p)->treatmentBase=(*p)->result.solid;retained_edge_treatment::SourceRebindResult rebound;retained_edge_treatment::Refusal refusal=retained_edge_treatment::Refusal::BuildFailed;if(!retained_edge_treatment::ApplySourceRebind(*(*p)->treatmentRebind,*(*p)->treatmentRebuildSource,(*p)->treatmentBase,(*p)->treatmentBudget,refusal,rebound))return (*p)->built=false;(*p)->treatmentDefinition=rebound.definition;(*p)->treatmentProofs=rebound.proofs;(*p)->result.solid=rebound.treated;}
        return (*p)->built;
    }
    if (const auto p=std::get_if<std::shared_ptr<SweepSolidGeometry>>(&payload)) {
        if (!*p || (*p)->built || (*p)->cancelled.load()) return false;
        (*p)->built=planar_sweep::Build((*p)->prepared,(*p)->cancelled,(*p)->result)==planar_sweep::BuildStatus::Built;
        return (*p)->built;
    }
    if (const auto assembly=std::get_if<std::shared_ptr<AssemblySolidGeometry>>(&payload))
        return BuildAssemblySolidGeometry(*assembly);
    if (const auto p=std::get_if<std::shared_ptr<ProfileSolidGeometry>>(&payload))
        return BuildProfileSolidGeometry(*p);
    if (const auto p=std::get_if<std::shared_ptr<EnclosureSolidGeometry>>(&payload)) {
        if (!*p || (*p)->built || !(*p)->cancelled || (*p)->cancelled->load()) return false;
        (*p)->built=BuildEnclosureSolidGeometry((*p)->parameters.definition,(*p)->cancelled,(*p)->result);
        // Publish the persistence-canonical representation of a rounded
        // enclosure: the bounded binary readback phase reconstructs
        // gp_Dir/gp_Ax components (signed zeros included), and only the
        // canonical form is a fixed point of the verified detachment round
        // trip that retained edge-treatment replay requires. The canonical
        // form is adopted only when a second readback preserves its exact
        // mesh-independent V3 commitment; otherwise the built result is kept
        // unchanged and the detachment gate's behavior is untouched.
        if((*p)->built&&(*p)->parameters.definition.dimensions.cornerRadius>0){
            retained_edge_treatment::ReplayBudget canonicalBudget;TopoDS_Shape canonical;
            if(retained_edge_treatment::detail::ReadbackGeometry((*p)->result.solid,canonicalBudget,canonical)){
                TopoDS_Shape reopened;retained_edge_treatment::Digest canonicalDigest,reopenedDigest;
                if(retained_edge_treatment::detail::ReadbackGeometry(canonical,canonicalBudget,reopened)
                    &&retained_edge_treatment::detail::CommitGeometry(canonical,canonicalBudget,canonicalDigest)
                    &&retained_edge_treatment::detail::CommitGeometry(reopened,canonicalBudget,reopenedDigest)
                    &&canonicalDigest==reopenedDigest)(*p)->result.solid=canonical;
            }
        }
        if((*p)->built&&(*p)->treatmentRebind&&(*p)->treatmentRebuildSource){(*p)->treatmentBase=(*p)->result.solid;retained_edge_treatment::SourceRebindResult rebound;retained_edge_treatment::Refusal refusal=retained_edge_treatment::Refusal::BuildFailed;if(!retained_edge_treatment::ApplySourceRebind(*(*p)->treatmentRebind,*(*p)->treatmentRebuildSource,(*p)->treatmentBase,(*p)->treatmentBudget,refusal,rebound))return (*p)->built=false;(*p)->treatmentDefinition=rebound.definition;(*p)->treatmentProofs=rebound.proofs;(*p)->result.solid=rebound.treated;}
        return (*p)->built;
    }
    return false;
}
void Core3DViewer::cancelNativeSolid(const std::shared_ptr<NativeSolidWork>& work) noexcept {
    if (!work) return;
    if(const auto p=std::get_if<std::shared_ptr<CutSolidGeometry>>(&work->geometry)) {
        if(*p)(*p)->cancelled.store(true);return;
    }
    if(const auto p=std::get_if<std::shared_ptr<CutProgramGeometry>>(&work->geometry)) {
        if(*p)(*p)->cancelled.store(true);return;
    }
    if (const auto p=std::get_if<std::shared_ptr<LoftSolidGeometry>>(&work->geometry)) {
        if (*p) (*p)->cancelled.store(true);
        return;
    }
    if (const auto p=std::get_if<std::shared_ptr<SweepSolidGeometry>>(&work->geometry)) {
        if (*p) (*p)->cancelled.store(true);
        return;
    }
    if (const auto assembly=std::get_if<std::shared_ptr<AssemblySolidGeometry>>(&work->geometry)) {
        if (*assembly) {
            (*assembly)->cancelled.store(true);
            for (const auto& part : (*assembly)->parts) if (part) part->cancelled.store(true);
        }
        return;
    }
    if (const auto p=std::get_if<std::shared_ptr<ProfileSolidGeometry>>(&work->geometry)) {
        if (*p) (*p)->cancelled.store(true);
    } else if (const auto p=std::get_if<std::shared_ptr<EnclosureSolidGeometry>>(&work->geometry)) {
        if (*p && (*p)->cancelled) (*p)->cancelled->store(true);
    }
}

bool Core3DViewer::attachModelingCreationPermit(const std::shared_ptr<NativeSolidWork>& work,
    std::shared_ptr<NativeModelingCommitPermit> permit) noexcept {
    if(!NSThread.isMainThread||!work||!permit||work->consumed||work->modelingPermit
        ||work->rebuildAuthority||permit->attached_||!permit->current()
        ||!permit->admission_||permit->admission_->phase()!=request::Phase::Building
        ||work->document!=permit->document_||work->document->HasOpenCommand())return false;
    try {
        request::Descriptor actual;actual.operation=static_cast<request::Operation>(permit->operation_);
        if(actual.operation==request::Operation::CreateEnclosure){
            const auto g=std::get_if<std::shared_ptr<EnclosureSolidGeometry>>(&work->geometry);
            if(!g||!*g||permit->featureIDs_.size()!=1||!work->assemblyParts.empty())return false;
            request::Part p;p.recipe=request::Recipe::Enclosure;
            p.schema=(*g)->parameters.definition.constructionFrame?enclosure::FramedSchemaVersion:enclosure::SchemaVersion;
            if(!enclosure::Encode((*g)->parameters,p.values))return false;
            actual.parts.push_back(std::move(p));
        }else if(actual.operation==request::Operation::CreateAssembly){
            if(!std::holds_alternative<std::shared_ptr<AssemblySolidGeometry>>(work->geometry)
                ||work->assemblyParts.size()!=permit->featureIDs_.size()
                ||work->assemblyParts.size()!=permit->descriptor_.parts.size())return false;
            for(std::size_t i=0;i<work->assemblyParts.size();++i){
                const auto& part=work->assemblyParts[i];request::Part p;receipt::UUID feature;
                p.recipe=request::Recipe::Profile;p.schema=profile::SchemaFor(part.parameters);
                p.name=permit->descriptor_.parts[i].name;
                if(!part.name.IsEqual(TCollection_ExtendedString(p.name.c_str(),Standard_True))
                    ||!receipt::ParseUUID(part.identifier,feature)||feature!=permit->featureIDs_[i]
                    ||!receipt::SupportedProfile(part.parameters)||!profile::Encode(part.parameters,p.values))return false;
                actual.parts.push_back(std::move(p));
            }
        }else return false;
        std::vector<std::uint8_t> a,b;request::Digest digest;
        if(!request::Encode(actual,a)||!request::Encode(permit->descriptor_,b)||a!=b
            ||!request::CommandHash(actual,digest)||digest!=permit->key_.command)return false;
        permit->attached_=true;work->modelingPermit=std::move(permit);return true;
    }catch(...){return false;}
}

bool Core3DViewer::attachModelingRebuildPermit(const std::shared_ptr<NativeSolidWork>& work,
    std::shared_ptr<NativeModelingCommitPermit> permit) noexcept {
    if(!NSThread.isMainThread||!work||!permit||work->consumed||work->modelingPermit
        ||!work->rebuildAuthority||work->rebuildAuthority->records.size()!=1||!work->assemblyParts.empty()
        ||permit->attached_||!permit->current()||!permit->expectedSource_||permit->featureIDs_.size()!=1
        ||!permit->admission_||permit->admission_->phase()!=request::Phase::Building
        ||work->document!=permit->document_||work->document->HasOpenCommand())return false;
    try {
        const auto& record=work->rebuildAuthority->records.front();
        request::Descriptor actual;actual.operation=static_cast<request::Operation>(permit->operation_);
        request::Part part;receipt::UUID feature;
        if(actual.operation==request::Operation::RebuildEnclosure){
            const auto geometry=std::get_if<std::shared_ptr<EnclosureSolidGeometry>>(&work->geometry);
            if(!geometry||!*geometry||!record.requested.enclosureRebuild||record.requested.profileRebuild
                ||!receipt::ParseUUID(record.previous.enclosure.identifier,feature))return false;
            part.recipe=request::Recipe::Enclosure;
            part.schema=(*geometry)->parameters.definition.constructionFrame?enclosure::FramedSchemaVersion:enclosure::SchemaVersion;
            if(!enclosure::Encode((*geometry)->parameters,part.values))return false;
            std::vector<double> requested;if(!enclosure::Encode(*record.requested.enclosureRebuild,requested)||requested!=part.values)return false;
        }else if(actual.operation==request::Operation::RebuildProfile){
            const auto geometry=profileSolidGeometry(work);
            if(!geometry||!record.requested.profileRebuild||record.requested.enclosureRebuild
                ||!receipt::SupportedProfile(record.previous.profile.parameters)
                ||!receipt::SupportedProfile(*record.requested.profileRebuild)
                ||!receipt::ParseUUID(record.previous.profile.identifier,feature))return false;
            profile::Parameters parameters{static_cast<const ProfileDefinition&>(*geometry),record.requested.profileRebuild->metersPerUnit};
            parameters.constructionFrame=geometry->constructionFrame;
            parameters.shells=geometry->shells;
            part.recipe=request::Recipe::Profile;part.schema=profile::SchemaFor(parameters);
            if(!profile::Encode(parameters,part.values))return false;
            std::vector<double> requested;if(!profile::Encode(*record.requested.profileRebuild,requested)||requested!=part.values)return false;
        }else if(actual.operation==request::Operation::RebuildLoftStation){
            const auto geometry=std::get_if<std::shared_ptr<LoftSolidGeometry>>(&work->geometry);
            if(!geometry||!*geometry||!(*geometry)->prepared||!work->loftRebuildStamp
                ||work->sweepRebuildStamp||!work->loftIdentifier.empty()
                ||record.requested.operation!=OrdinaryTransformOperation::LoftStationRebuild
                ||!record.requested.loftRebuild||!record.requested.loftStationEdit
                ||record.requested.profileRebuild||record.requested.enclosureRebuild||record.requested.sweepRebuild
                ||permit->expectedSource_->policy!=receipt::ExactLoftPolicy4097
                ||permit->expectedSource_->feature!=receipt::Feature::RectangularLoft
                ||!receipt::ParseUUID(record.previous.loft.identifier,feature)
                ||!receipt::LoftStationDescriptor(record.previous.loft.definition,*record.requested.loftStationEdit,actual))return false;
            std::vector<double> prepared,requested;
            if(!loft_persistence::Encode((*geometry)->prepared->definition,prepared)
                ||!loft_persistence::Encode(*record.requested.loftRebuild,requested)
                ||!loft_persistence::SameBits(prepared,requested)
                ||!loft_persistence::SameBits(prepared,actual.parts.front().values))return false;
        }else return false;
        if(actual.operation!=request::Operation::RebuildLoftStation)actual.parts.push_back(std::move(part));
        std::vector<std::uint8_t> a,b;request::Digest digest;receipt::Effect source;
        if(feature!=permit->featureIDs_.front()||feature!=permit->expectedSource_->featureID
            ||!request::Encode(actual,a)||!request::Encode(permit->descriptor_,b)||a!=b
            ||!request::CommandHash(actual,digest)||digest!=permit->key_.command
            ||!receipt::CaptureEffect(work->owner,record.previous.label,source)||!(source==*permit->expectedSource_))return false;
        permit->attached_=true;work->modelingPermit=std::move(permit);return true;
    }catch(...){return false;}
}

OrdinaryEditResult Core3DViewer::commitNativeSolid(const std::shared_ptr<NativeSolidWork>& work) noexcept {
    if (![NSThread isMainThread] || !work || work->consumed) CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
    const auto cutPayload=std::get_if<std::shared_ptr<CutSolidGeometry>>(&work->geometry);
    if(cutPayload){
        if(!*cutPayload||work->modelingPermit||!work->cutStamp||work->loftRebuildStamp||work->sweepRebuildStamp
            ||!work->loftIdentifier.empty()||!work->sweepIdentifier.empty()||!work->assemblyParts.empty()
            ||!work->rebuildAuthority||work->rebuildAuthority->records.size()!=1
            ||work->rebuildAuthority->records.front().requested.operation!=OrdinaryTransformOperation::CylindricalCut)CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
    }else if(const auto cutProgram=std::get_if<std::shared_ptr<CutProgramGeometry>>(&work->geometry)){
        if(!*cutProgram||work->modelingPermit||!work->cutStamp||work->loftRebuildStamp||work->sweepRebuildStamp
            ||!work->loftIdentifier.empty()||!work->sweepIdentifier.empty()||!work->assemblyParts.empty()
            ||!work->rebuildAuthority||work->rebuildAuthority->records.size()!=1
            ||!IsCylindricalCutOperation(work->rebuildAuthority->records.front().requested.operation)
            ||!work->rebuildAuthority->records.front().requested.cutProgramEdit)CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
    }else if(work->cutStamp)CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
    const auto loftPayload=std::get_if<std::shared_ptr<LoftSolidGeometry>>(&work->geometry);
    if(loftPayload) {
        if(work->sweepRebuildStamp)CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
        if(work->modelingPermit && (!work->rebuildAuthority
            ||work->modelingPermit->operation_!=receipt::Operation::RebuildLoftStation
            ||!work->modelingPermit->attached_))CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
        if(work->rebuildAuthority) {
            if(!work->loftIdentifier.empty() || !work->loftRebuildStamp || work->rebuildAuthority->records.size()!=1
                || work->rebuildAuthority->records.front().requested.operation!=OrdinaryTransformOperation::LoftStationRebuild)
                CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
        }else if(work->loftRebuildStamp || !profile::IsIdentifier(work->loftIdentifier))CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
    }
    if(!loftPayload && (!work->loftIdentifier.empty() || work->loftRebuildStamp))CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
    const auto sweepPayload=std::get_if<std::shared_ptr<SweepSolidGeometry>>(&work->geometry);
    if (sweepPayload) {
        if (work->modelingPermit) CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid); // No sweep receipt route.
        if (work->rebuildAuthority) {
            if (!work->sweepIdentifier.empty() || !work->sweepRebuildStamp
                || work->rebuildAuthority->records.size()!=1
                || work->rebuildAuthority->records.front().requested.operation!=OrdinaryTransformOperation::SweepRebuild)
                CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
        } else if (work->sweepRebuildStamp || !profile::IsIdentifier(work->sweepIdentifier)) CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
    }
    const auto completed=CompletedNativeSolidFor(work->geometry);
    std::shared_ptr<AssemblySolidGeometry> assembly;
    if (const auto payload=std::get_if<std::shared_ptr<AssemblySolidGeometry>>(&work->geometry)) assembly=*payload;
    if (assembly) {
        if (!assembly->built || assembly->cancelled.load() || assembly->parts.empty() || assembly->parts.size()>16
            || work->rebuildAuthority || work->assemblyParts.size()!=assembly->parts.size()) CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
        for (const auto& part : assembly->parts)
            if (!part || !part->built || part->cancelled.load() || part->solid.IsNull()) CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
    } else if (!completed) CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
    work->consumed = true;
    if (!canBeginCommittedEdit()) { CORE3D_CUT_REFUSE("commit.busy:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Busy); }
    try {
        if (myDoc != work->owner || myDoc->Document() != work->document
            || work->document->GetData()->Time() != work->documentTime) { CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid); }
        const auto snapshot = captureSceneSnapshot(work->width, work->height);
        if (!snapshot || snapshot->publicationSourceIdentifier != work->identity.publicationSourceIdentifier
            || snapshot->selectionMode != scene::ElementKind::Object
            || snapshot->revisions.documentGeneration != work->identity.documentGeneration
            || snapshot->revisions.model != work->identity.modelRevision
            || snapshot->revisions.presentation != work->presentationRevision
            || !_shapeInteractor->selectionModeAuthorityIsExact()
            || _shapeInteractor->getSelectionMode() != work->authority.selectionMode
            || !_objectInteractor->verifyOrdinaryNameAuthority(work->authority)) { CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid); }
        if (work->sweepRebuildStamp) {
            const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
            if (!stamp || !(*stamp==*work->sweepRebuildStamp)) CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
        }
        if(work->cutStamp){
            const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
            if(!stamp||!(*stamp==*work->cutStamp))CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
        }
        if(work->loftRebuildStamp) {
            const auto stamp=myDoc->CaptureNativePlanningStamp(canBeginCommittedEdit());
            if(!stamp || !(*stamp==*work->loftRebuildStamp))CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
        }
        if(work->modelingPermit){
            auto& p=*work->modelingPermit;
            const bool rebuild=p.operation_==receipt::Operation::RebuildEnclosure||p.operation_==receipt::Operation::RebuildProfile
                ||p.operation_==receipt::Operation::RebuildLoftStation;
            if(bool(work->rebuildAuthority)!=rebuild)CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
            if(!p.current()||!p.admission_
                ||!p.admission_->geometryReady(true,true)||!p.admission_->takeForOrdinary(true))CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
        }
        if (assembly) {
            std::vector<OrdinaryCreationRequest> requests;
            std::vector<Handle(AIS_InteractiveObject)> presentations;
            for (std::size_t i=0; i<assembly->parts.size(); ++i) {
                Handle(AIS_Shape) presentation = new AIS_Shape(assembly->parts[i]->solid);
                myContext->ApplyDefaultMaterial(presentation);
                Quantity_Color color; presentation->Color(color);
                OrdinaryCreationRequest request{presentation, Graphic3d_NameOfMaterial_ShinyPlastified,
                    color.Name(), OcctGeometryRepresentation::BRep};
                request.profile=work->assemblyParts[i].parameters;
                request.profileIdentifier=work->assemblyParts[i].identifier;
                request.name=work->assemblyParts[i].name;
                requests.push_back(std::move(request)); presentations.push_back(presentation);
            }
            // Exactly one existing ordinary creation transaction stages every
            // part, profile identity and display name, or reconciles/aborts all.
            const auto result=publishCreatedPrimitives(requests,work->modelingPermit);
            if (result!=OrdinaryEditResult::Committed) return result;
            try {
                if (work->frameFirst && !myView.IsNull()) {
                    const auto& b=assembly->bounds;
                    Bnd_Box bounds; bounds.Add(gp_Pnt(b[0],b[1],b[2])); bounds.Add(gp_Pnt(b[3],b[4],b[5]));
                    myView->FitAll(bounds,0.45,Standard_False);myView->ZFitAll();
                }
                bool touched=false; (void)_objectInteractor->replaceSelectedObjectsForBrowser(presentations,touched);
            } catch (...) {}
            return result;
        }
        if (work->rebuildAuthority) {
            auto authority = *work->rebuildAuthority;
            if (authority.records.size() != 1 || !admitTransform(authority)
                || authority.selectionOwners != work->rebuildAuthority->selectionOwners
                || authority.manipulatorType != work->rebuildAuthority->manipulatorType
                || authority.hadManipulator != work->rebuildAuthority->hadManipulator) CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
            auto& record = authority.records.front();
            OcctObjectTransformState current;
            if (!myDoc->CaptureObjectTransformStateForLabel(record.previous.label, current)
                || !current.IsEqual(record.previous)) CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
            if(const auto cut=std::get_if<std::shared_ptr<CutSolidGeometry>>(&work->geometry)){
                cut_display::Settings currentDisplay;
                if(!*cut||record.requested.presentation.IsNull()
                    ||!cut_display::Capture(record.requested.presentation->Attributes(),currentDisplay)
                    ||!(currentDisplay==(*cut)->displaySettings))CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
            }
            if(const auto programCut=std::get_if<std::shared_ptr<CutProgramGeometry>>(&work->geometry)){
                cut_display::Settings currentDisplay;
                if(!*programCut||record.requested.presentation.IsNull()
                    ||!cut_display::Capture(record.requested.presentation->Attributes(),currentDisplay)
                    ||!(currentDisplay==(*programCut)->displaySettings))CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
            }
            if(cutPayload&&(*cutPayload)->rebuildLoftBase){
                if((*cutPayload)->loftBaseStatus!=saved_cut_source_edit::LoftBaseStatus::Built||!record.requested.cut)CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
                auto carrier=std::make_shared<retained_solid::Payload>(*record.requested.cut);
                carrier->base=(*cutPayload)->detachedBase;record.requested.cut=std::move(carrier);
            }
            record.requested.shape = completed->solid;
            if(work->edgeTreatmentSnapshot&&work->edgeTreatmentEdit){
                auto treatment=std::shared_ptr<retained_edge_treatment::DetachedResult>(new retained_edge_treatment::DetachedResult);
                treatment->nonce_=work->edgeTreatmentSnapshot->nonce_;treatment->source_=std::get<retained_edge_treatment::RebuildSource>(*work->edgeTreatmentEdit).requested;
                if(const auto profileGeometry=std::get_if<std::shared_ptr<ProfileSolidGeometry>>(&work->geometry)){
                    if(!*profileGeometry||!(*profileGeometry)->treatmentDefinition)return OrdinaryEditResult::Invalid;treatment->definition_=*(*profileGeometry)->treatmentDefinition;treatment->base_=(*profileGeometry)->treatmentBase;treatment->proofs_=(*profileGeometry)->treatmentProofs;treatment->budget_=(*profileGeometry)->treatmentBudget;
                }else if(const auto enclosureGeometry=std::get_if<std::shared_ptr<EnclosureSolidGeometry>>(&work->geometry)){
                    if(!*enclosureGeometry||!(*enclosureGeometry)->treatmentDefinition)return OrdinaryEditResult::Invalid;treatment->definition_=*(*enclosureGeometry)->treatmentDefinition;treatment->base_=(*enclosureGeometry)->treatmentBase;treatment->proofs_=(*enclosureGeometry)->treatmentProofs;treatment->budget_=(*enclosureGeometry)->treatmentBudget;
                }else if(const auto loftGeometry=std::get_if<std::shared_ptr<LoftSolidGeometry>>(&work->geometry)){
                    if(!*loftGeometry||!(*loftGeometry)->treatmentDefinition)return OrdinaryEditResult::Invalid;treatment->definition_=*(*loftGeometry)->treatmentDefinition;treatment->base_=(*loftGeometry)->treatmentBase;treatment->proofs_=(*loftGeometry)->treatmentProofs;treatment->budget_=(*loftGeometry)->treatmentBudget;
                }else return OrdinaryEditResult::Invalid;
                treatment->result_=completed->solid;retained_edge_treatment::Refusal refusal;if(!retained_edge_treatment::Encode(treatment->definition_,treatment->definitionBytes_,refusal))return OrdinaryEditResult::Invalid;
                record.requested.edgeTreatmentSnapshot=work->edgeTreatmentSnapshot;record.requested.edgeTreatmentEdit=work->edgeTreatmentEdit;record.requested.edgeTreatmentResult=std::move(treatment);
            }
            OrdinaryEditResult result = OrdinaryEditResult::Invalid;
            auto lease = work->modelingPermit
                ? _ordinaryEditController->beginModelingRebuild(record.requested,work->modelingPermit,&result)
                : _ordinaryEditController->beginTransform({record.requested}, &result);
            return lease ? lease.stageAndCommit() : result;
        }
        Handle(AIS_Shape) presentation = new AIS_Shape(completed->solid);
        myContext->ApplyDefaultMaterial(presentation);
        Quantity_Color color;
        presentation->Color(color);
        OrdinaryCreationRequest request{presentation,
            Graphic3d_NameOfMaterial_ShinyPlastified, color.Name(), OcctGeometryRepresentation::BRep};
        NSString* identifier = work->modelingPermit
            ? [[NSUUID alloc] initWithUUIDBytes:work->modelingPermit->featureIDs_.front().data()].UUIDString
            : loftPayload ? [NSString stringWithUTF8String:work->loftIdentifier.c_str()]
            : NSUUID.UUID.UUIDString;
        if (identifier == nil) CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
        if (const auto geometry=profileSolidGeometry(work)) {
            request.profile=profile::Parameters{static_cast<const ProfileDefinition&>(*geometry),work->metersPerUnit};
            request.profileIdentifier=identifier.UTF8String;
        } else if (loftPayload && *loftPayload && (*loftPayload)->prepared) {
            request.loft=(*loftPayload)->prepared->definition;
            request.loftIdentifier=work->loftIdentifier;
            request.name=TCollection_ExtendedString("Lofted solid");
        } else if (sweepPayload && *sweepPayload && (*sweepPayload)->prepared) {
            request.sweep=(*sweepPayload)->prepared->definition;
            request.sweepIdentifier=work->sweepIdentifier;
            request.name=TCollection_ExtendedString("Swept solid");
        } else {
            const auto enclosureGeometry=std::get_if<std::shared_ptr<EnclosureSolidGeometry>>(&work->geometry);
            if (!enclosureGeometry || !*enclosureGeometry) CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid);
            request.enclosure=(*enclosureGeometry)->parameters;
            request.enclosureIdentifier=identifier.UTF8String;
        }
        const std::vector<OrdinaryCreationRequest> requests = {request};
        const auto result = publishCreatedPrimitives(requests,work->modelingPermit);
        if (result != OrdinaryEditResult::Committed) { return result; }
        // Cosmetic continuation follows durable publication. Recovery can safely
        // leave the new solid visible but unselected if this continuation fails.
        try {
            if (work->frameFirst && !myView.IsNull()) {
                const auto& b = completed->bounds;
                Bnd_Box bounds; bounds.Add(gp_Pnt(b[0], b[1], b[2])); bounds.Add(gp_Pnt(b[3], b[4], b[5]));
                myView->FitAll(bounds, 0.45, Standard_False); myView->ZFitAll();
            }
            // Replace the old selection and its gizmo ownership together.
            // Deselecting alone leaves a previous manipulator attached, and
            // SelectAndAttachManipulator appends the new object to it.
            bool selectionWasTouched = false;
            (void)_objectInteractor->replaceSelectedObjectForBrowser(presentation, selectionWasTouched);
        } catch (...) {}
        return result;
    } catch (...) { CORE3D_CUT_REFUSE("commit.invalid:" CORE3D_CUT_STRINGIFY(__LINE__), OrdinaryEditResult::Invalid); }
}

// The worker payload owns only copied geometry and values. Live OCAF/AIS
// authority is separately owned on the main thread, including its destruction
// if the editor closes before the background block finishes releasing captures.
struct ObjectAlignmentMeasurement {
    struct Geometry {
        TopoDS_Shape copy;
        gp_Trsf transform;
        bool mesh = false;
        std::array<double, 6> bounds{};
    };
    std::vector<Geometry> geometry;
    std::atomic_bool cancelled{false};
    bool measured = false;
};
struct ObjectAlignmentWork {
    std::shared_ptr<ObjectAlignmentMeasurement> measurement = std::make_shared<ObjectAlignmentMeasurement>();
    OrdinaryTransformLedger authority;
    ObjectFrameIdentity identity;
    std::uint64_t presentationRevision = 0;
    std::uint32_t width = 0, height = 0;
    Standard_Integer documentTime = 0;
    int axis = 0;
    OcctSavedGroupState savedGroups;
    ObjectAlignmentAnchor anchor = ObjectAlignmentAnchor::Minimum;
    bool consumed = false;
};

std::shared_ptr<ObjectAlignmentWork> Core3DViewer::prepareObjectAlignment(
    int axis, ObjectAlignmentAnchor anchor, const ObjectFrameIdentity& identity,
    std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height) noexcept {
    if (![NSThread isMainThread] || !canBeginCommittedEdit() || myContext.IsNull()
        || axis < 0 || axis > 2 || width == 0 || height == 0
        || (anchor != ObjectAlignmentAnchor::Minimum && anchor != ObjectAlignmentAnchor::Center
            && anchor != ObjectAlignmentAnchor::Maximum && anchor != ObjectAlignmentAnchor::Ground
            && anchor != ObjectAlignmentAnchor::EqualCenters && anchor != ObjectAlignmentAnchor::EqualGaps
            && anchor != ObjectAlignmentAnchor::CenterGround && anchor != ObjectAlignmentAnchor::GroupBaseOrigin)
        || ((anchor == ObjectAlignmentAnchor::Ground || anchor == ObjectAlignmentAnchor::CenterGround || anchor == ObjectAlignmentAnchor::GroupBaseOrigin) && axis != 2)) { return {}; }
    try {
        OCC_CATCH_SIGNALS
        const auto snapshot = captureSceneSnapshot(width, height);
        if (!snapshot || snapshot->selectionMode != scene::ElementKind::Object
            || snapshot->publicationSourceIdentifier != identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration != identity.documentGeneration
            || snapshot->revisions.model != identity.modelRevision
            || snapshot->revisions.presentation != presentationRevision
            || snapshot->selection.selected.size() > 32
            || snapshot->selection.selected.size() < ((anchor == ObjectAlignmentAnchor::Ground
                || anchor == ObjectAlignmentAnchor::CenterGround) ? 1
                : (anchor == ObjectAlignmentAnchor::EqualCenters || anchor == ObjectAlignmentAnchor::EqualGaps) ? 3 : 2)) { return {}; }
        auto work = std::make_shared<ObjectAlignmentWork>();
        work->identity = identity; work->presentationRevision = presentationRevision;
        work->width = width; work->height = height; work->axis = axis; work->anchor = anchor;
        work->documentTime = myDoc->Document()->GetData()->Time();
        std::unordered_set<std::string> selected;
        for (const auto& element : snapshot->selection.selected) {
            if (element.kind != scene::ElementKind::Object || !selected.insert(element.entityIdentifier).second) { return {}; }
        }
        if(anchor==ObjectAlignmentAnchor::GroupBaseOrigin) {
            if(identity.entityIdentifier.size()!=36||!myDoc->CaptureSavedGroups(work->savedGroups))return {};
            const auto group=std::find_if(work->savedGroups.groups.begin(),work->savedGroups.groups.end(),[&](const auto& g){return g.identifier==identity.entityIdentifier;});
            if(group==work->savedGroups.groups.end()||group->members.size()!=selected.size())return {};
            for(const auto& member:group->members)if(!selected.count(myDoc->EntityIdentifierForLabel(member)))return {};
        }
        for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
            const auto presentation = Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
            if (presentation.IsNull()) { return {}; }
            OrdinaryTransformRecord record;
            const auto label = myDoc->ShapeLabel(presentation);
            if (!myDoc->IsEditableFreeSimpleDefinitionLabel(label)
                || !myDoc->CaptureObjectTransformStateForLabel(label, record.previous)
                || selected.erase(record.previous.entityIdentifier) != 1) { return {}; }
            record.requested.label = label; record.requested.presentation = presentation;
            record.requested.shape = record.previous.shape; record.requested.transform = record.previous.transform;
            record.requested.operation = OrdinaryTransformOperation::Translate;
            work->authority.records.push_back(record);
        }
        if (!selected.empty() || !admitTransform(work->authority)) { return {}; }
        // Bound all copies together, including mesh vertex tables. Admission
        // precedes copying so one large selection cannot queue unbounded work.
        std::size_t nodes = 0, faces = 0, meshNodes = 0;
        for (const auto& record : work->authority.records) {
            const auto& state = record.previous;
            if (state.resolvedRepresentation != OcctGeometryRepresentation::BRep
                && state.resolvedRepresentation != OcctGeometryRepresentation::TriangleMesh) { return {}; }
            std::vector<TopoDS_Shape> stack{state.shape};
            TopTools_IndexedMapOfShape visited;
            while (!stack.empty()) {
                const auto shape = stack.back(); stack.pop_back();
                if (visited.Contains(shape)) { continue; }
                visited.Add(shape);
                if (++nodes > 8192) { return {}; }
                for (TopoDS_Iterator child(shape); child.More(); child.Next()) {
                    if (stack.size() >= 8192) { return {}; }
                    if (!visited.Contains(child.Value())) { stack.push_back(child.Value()); }
                }
            }
            if (state.resolvedRepresentation == OcctGeometryRepresentation::TriangleMesh) {
                bool hasFace = false;
                for (TopExp_Explorer face(state.shape, TopAbs_FACE); face.More(); face.Next()) {
                    hasFace = true;
                    if (++faces > 4096) { return {}; }
                    TopLoc_Location location;
                    const auto mesh = BRep_Tool::Triangulation(TopoDS::Face(face.Current()), location);
                    if (mesh.IsNull() || mesh->HasDeferredData() || !mesh->HasGeometry()
                        || mesh->NbNodes() <= 0 || mesh->NbTriangles() <= 0
                        || static_cast<std::size_t>(mesh->NbNodes()) > 262144 - meshNodes) { return {}; }
                    meshNodes += mesh->NbNodes();
                }
                if (!hasFace) { return {}; }
            }
        }
        for (const auto& record : work->authority.records) {
            const bool mesh = record.previous.resolvedRepresentation == OcctGeometryRepresentation::TriangleMesh;
            BRepBuilderAPI_Copy copy(record.previous.shape, Standard_True, mesh);
            if (!copy.IsDone() || copy.Shape().IsNull()) { return {}; }
            ObjectAlignmentMeasurement::Geometry geometry;
            geometry.copy = copy.Shape(); geometry.transform = record.previous.transform; geometry.mesh = mesh;
            work->measurement->geometry.push_back(std::move(geometry));
        }
        return work;
    } catch (...) { return {}; }
}

std::shared_ptr<ObjectAlignmentMeasurement> Core3DViewer::objectAlignmentMeasurement(
    const std::shared_ptr<ObjectAlignmentWork>& work) noexcept {
    return [NSThread isMainThread] && work ? work->measurement : nullptr;
}

bool Core3DViewer::measureObjectAlignment(const std::shared_ptr<ObjectAlignmentMeasurement>& measurement) noexcept {
    if (!measurement || measurement->cancelled.load()) { return false; }
    try {
        OCC_CATCH_SIGNALS
        for (auto& geometry : measurement->geometry) {
            if (measurement->cancelled.load()) { return false; }
            Bnd_Box bounds;
            if (geometry.mesh) {
                for (TopExp_Explorer face(geometry.copy, TopAbs_FACE); face.More(); face.Next()) {
                    if (measurement->cancelled.load()) { return false; }
                    TopLoc_Location location;
                    const auto mesh = BRep_Tool::Triangulation(TopoDS::Face(face.Current()), location);
                    if (mesh.IsNull()) { return false; }
                    for (int node = 1; node <= mesh->NbNodes(); ++node) {
                        auto point = mesh->Node(node).Transformed(location.Transformation());
                        point.Transform(geometry.transform);
                        if (!std::isfinite(point.X()) || !std::isfinite(point.Y()) || !std::isfinite(point.Z())) { return false; }
                        bounds.Add(point);
                    }
                }
            } else {
                // Transform the private analytic shape, rather than its local
                // AABB: rotating an AABB is not an accurate curved-solid bound.
                BRepBuilderAPI_Transform transformed(geometry.copy, geometry.transform, Standard_True);
                if (!transformed.IsDone()) { return false; }
                BRepBndLib::AddOptimal(transformed.Shape(), bounds, Standard_False, Standard_False);
            }
            if (bounds.IsVoid() || bounds.IsOpen()) { return false; }
            auto& b = geometry.bounds;
            bounds.Get(b[0], b[1], b[2], b[3], b[4], b[5]);
            for (double value : b) { if (!std::isfinite(value)) { return false; } }
            for (int axis = 0; axis < 3; ++axis) { if (b[axis] > b[axis + 3]) { return false; } }
        }
        measurement->measured = !measurement->cancelled.load();
        return measurement->measured;
    } catch (...) { return false; }
}

void Core3DViewer::cancelObjectAlignment(const std::shared_ptr<ObjectAlignmentWork>& work) noexcept {
    if (work) { work->measurement->cancelled.store(true); }
}

OrdinaryEditResult Core3DViewer::commitObjectAlignment(const std::shared_ptr<ObjectAlignmentWork>& work) noexcept {
    if (![NSThread isMainThread] || !work || work->measurement->cancelled.load() || !work->measurement->measured || work->consumed) {
        return OrdinaryEditResult::Invalid;
    }
    work->consumed = true;
    if (!canBeginCommittedEdit()) { return OrdinaryEditResult::Busy; }
    try {
        const auto snapshot = captureSceneSnapshot(work->width, work->height);
        if (!snapshot || snapshot->publicationSourceIdentifier != work->identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration != work->identity.documentGeneration
            || snapshot->revisions.model != work->identity.modelRevision
            || snapshot->revisions.presentation != work->presentationRevision
            || myDoc->Document()->GetData()->Time() != work->documentTime) { return OrdinaryEditResult::Invalid; }
        for (const auto& record : work->authority.records) {
            OcctObjectTransformState actual;
            if (!myDoc->CaptureObjectTransformStateForLabel(record.previous.label, actual)
                || !actual.IsEqual(record.previous)) { return OrdinaryEditResult::Invalid; }
        }
        auto authority = work->authority;
        if (!admitTransform(authority) || authority.selectionOwners != work->authority.selectionOwners
            || authority.manipulatorType != work->authority.manipulatorType
            || authority.hadManipulator != work->authority.hadManipulator) { return OrdinaryEditResult::Invalid; }
        const int axis = work->axis;
        if(work->anchor==ObjectAlignmentAnchor::GroupBaseOrigin) {
            OcctSavedGroupState current;if(!myDoc->CaptureSavedGroups(current)||!current.IsEqual(work->savedGroups))return OrdinaryEditResult::Invalid;
            const auto& measured=work->measurement->geometry;if(measured.empty())return OrdinaryEditResult::Invalid;
            double lo[3]={measured[0].bounds[0],measured[0].bounds[1],measured[0].bounds[2]},hi[3]={measured[0].bounds[3],measured[0].bounds[4],measured[0].bounds[5]};
            for(const auto& item:measured)for(int d=0;d<3;++d){lo[d]=std::min(lo[d],item.bounds[d]);hi[d]=std::max(hi[d],item.bounds[d+3]);}
            gp_Pnt origin(lo[0]*.5+hi[0]*.5,lo[1]*.5+hi[1]*.5,lo[2]);
            if(!OcctDocument::IsAdmittedSavedGroupOrigin(origin))return OrdinaryEditResult::Invalid;
            auto requested=current.groups;const auto group=std::find_if(requested.begin(),requested.end(),[&](const auto& g){return g.identifier==work->identity.entityIdentifier;});
            if(group==requested.end())return OrdinaryEditResult::Invalid;group->originPresent=Standard_True;group->origin=origin;
            OrdinaryEditResult failure=OrdinaryEditResult::Invalid;auto lease=_ordinaryEditController->beginGrouping(requested,&failure);
            return lease?lease.stageAndCommit():failure;
        }
        double minimum = work->measurement->geometry.front().bounds[axis];
        double maximum = work->measurement->geometry.front().bounds[axis + 3];
        for (const auto& geometry : work->measurement->geometry) {
            minimum = std::min(minimum, geometry.bounds[axis]);
            maximum = std::max(maximum, geometry.bounds[axis + 3]);
        }
        const auto coordinate = [&](double lo, double hi) {
            switch (work->anchor) {
                case ObjectAlignmentAnchor::Minimum: case ObjectAlignmentAnchor::Ground:
                case ObjectAlignmentAnchor::CenterGround: return lo;
                case ObjectAlignmentAnchor::Center:
                case ObjectAlignmentAnchor::EqualCenters: return lo * 0.5 + hi * 0.5;
                case ObjectAlignmentAnchor::EqualGaps: return lo;
                case ObjectAlignmentAnchor::Maximum: return hi;
                case ObjectAlignmentAnchor::GroupBaseOrigin: return lo;
            }
            return lo;
        };
        const double target = work->anchor == ObjectAlignmentAnchor::Ground ? 0 : coordinate(minimum, maximum);
        const auto& geometry = work->measurement->geometry;
        std::array<double, 3> sharedTranslation{};
        if (work->anchor == ObjectAlignmentAnchor::CenterGround) {
            for (int dimension = 0; dimension < 3; ++dimension) {
                double aggregateMinimum = geometry.front().bounds[dimension];
                double aggregateMaximum = geometry.front().bounds[dimension + 3];
                for (const auto& item : geometry) {
                    aggregateMinimum = std::min(aggregateMinimum, item.bounds[dimension]);
                    aggregateMaximum = std::max(aggregateMaximum, item.bounds[dimension + 3]);
                }
                sharedTranslation[dimension] = dimension == 2 ? -aggregateMinimum
                    : -(aggregateMinimum * 0.5 + aggregateMaximum * 0.5);
                if (!std::isfinite(sharedTranslation[dimension])) { return OrdinaryEditResult::Invalid; }
            }
        }
        std::vector<double> targets(geometry.size(), target);
        if (work->anchor == ObjectAlignmentAnchor::EqualCenters || work->anchor == ObjectAlignmentAnchor::EqualGaps) {
            if (geometry.size() < 3 || geometry.size() != work->authority.records.size()) { return OrdinaryEditResult::Invalid; }
            std::vector<std::size_t> order;
            for (std::size_t i = 0; i < geometry.size(); ++i) { order.push_back(i); }
            const auto center = [&](std::size_t i) {
                return geometry[i].bounds[axis] * 0.5 + geometry[i].bounds[axis + 3] * 0.5;
            };
            std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
                const double ca = center(a), cb = center(b);
                return ca != cb ? ca < cb : work->authority.records[a].previous.entityIdentifier
                    < work->authority.records[b].previous.entityIdentifier;
            });
            if (work->anchor == ObjectAlignmentAnchor::EqualCenters) {
                const double first = center(order.front()), last = center(order.back());
                for (std::size_t rank = 0; rank < order.size(); ++rank) {
                    const double fraction = static_cast<double>(rank) / static_cast<double>(order.size() - 1);
                    targets[order[rank]] = first * (1 - fraction) + last * fraction;
                }
            } else {
                double totalExtent = 0;
                for (const auto& item : geometry) { totalExtent += item.bounds[axis + 3] - item.bounds[axis]; }
                const double available = maximum - minimum - totalExtent;
                // This command promises free gaps, not overlapping centers.
                // Allow only kernel rounding below modeling precision.
                if (!std::isfinite(available) || available < -1e-7) { return OrdinaryEditResult::Invalid; }
                const double gap = std::max(0.0, available) / static_cast<double>(order.size() - 1);
                double nextMinimum = minimum;
                for (const auto index : order) {
                    targets[index] = nextMinimum;
                    nextMinimum += geometry[index].bounds[axis + 3] - geometry[index].bounds[axis] + gap;
                }
            }
        }
        std::vector<OrdinaryTransformChange> changes;
        for (std::size_t index = 0; index < geometry.size(); ++index) {
            auto change = work->authority.records[index].requested;
            const auto& b = work->measurement->geometry[index].bounds;
            auto translation = change.transform.TranslationPart();
            if (work->anchor == ObjectAlignmentAnchor::CenterGround) {
                bool changed = false;
                for (int dimension = 0; dimension < 3; ++dimension) {
                    const double delta = sharedTranslation[dimension];
                    // Ignore kernel-bound rounding below modeling precision.
                    // Repeating the command then remains a true no-op.
                    if (std::abs(delta) <= 1e-7) { continue; }
                    const double translated = translation.Coord(dimension + 1) + delta;
                    if (!std::isfinite(translated)) { return OrdinaryEditResult::Invalid; }
                    translation.SetCoord(dimension + 1, translated);
                    changed = true;
                }
                if (changed) {
                    change.transform.SetTranslationPart(gp_Vec(translation));
                    gp_Trsf collective;collective.SetTranslation(gp_Vec(sharedTranslation[0],sharedTranslation[1],sharedTranslation[2]));
                    change.collectiveWorldDelta=collective;
                }
            } else {
                const double delta = targets[index] - coordinate(b[axis], b[axis + 3]);
                if (!std::isfinite(delta)) { return OrdinaryEditResult::Invalid; }
                // Ignore kernel-bound rounding below modeling precision. This
                // makes repeated alignment a true no-op without history noise.
                if (std::abs(delta) > 1e-7) {
                    const double translated = translation.Coord(axis + 1) + delta;
                    if (!std::isfinite(translated)) { return OrdinaryEditResult::Invalid; }
                    translation.SetCoord(axis + 1, translated);
                    change.transform.SetTranslationPart(gp_Vec(translation));
                }
            }
            changes.push_back(std::move(change));
        }
        OrdinaryEditResult failure = OrdinaryEditResult::Invalid;
        auto lease = beginOrdinaryTransform(changes, &failure);
        return lease ? lease.stageAndCommit() : failure;
    } catch (...) { return OrdinaryEditResult::Invalid; }
}

bool Core3DViewer::hasUnresolvedOrdinaryEditExcludingQueuedLoad() const noexcept {
    return _documentReplacementWork != nullptr
        || (_ordinaryEditController != nullptr && _ordinaryEditController->blocksNormalWork());
}
bool Core3DViewer::hasUnresolvedOrdinaryEdit() const noexcept {
    return _queuedAssetLoadWork != nullptr || hasUnresolvedOrdinaryEditExcludingQueuedLoad();
}
bool Core3DViewer::hasUnresolvedEdit() const noexcept {
    if (!myDoc.IsNull() && myDoc->NativeBooleanOwnerBlocksOtherWork()) return true;
    if (hasUnresolvedOrdinaryEdit() || hasUnresolvedDuplicate()) return true;
    if (!_objectInteractor) return false;
    // Availability must agree with the authoritative preview outcome. A ready
    // preview remains usable; a committing or uncertain result cannot expose
    // Undo, Add or Export while its own controller retains recovery ownership.
    const auto linear = _objectInteractor->linearArrayPreviewState();
    const auto radial = _objectInteractor->radialArrayPreviewState();
    const auto mirror = _objectInteractor->mirrorPreviewState();
    return linear == LinearArrayPreviewState::Committing
        || linear == LinearArrayPreviewState::OutcomeUnknown
        || radial == RadialArrayPreviewState::Committing
        || radial == RadialArrayPreviewState::OutcomeUnknown
        || mirror == MirrorPreviewState::Committing
        || mirror == MirrorPreviewState::OutcomeUnknown;
}
OrdinaryEditLease Core3DViewer::beginOrdinaryTransform(
    const std::vector<OrdinaryTransformChange>& changes, OrdinaryEditResult* failure) noexcept {
    if (![NSThread isMainThread] || !canBeginCommittedEdit() || !_ordinaryEditController) {
        if (failure) { *failure = OrdinaryEditResult::Busy; }
        return {};
    }
    return _ordinaryEditController->beginTransform(changes, failure);
}
OrdinaryEditResult Core3DViewer::reconcileOrdinaryEdit() noexcept {
    if (_queuedAssetLoadWork) return OrdinaryEditResult::Busy;
    if (_documentReplacementWork) {
        return restoreDocumentReplacement() ? OrdinaryEditResult::NoChange : OrdinaryEditResult::OutcomeUnknown;
    }
    return _ordinaryEditController ? _ordinaryEditController->reconcile() : OrdinaryEditResult::NoChange;
}
bool Core3DViewer::admitVisibility(OrdinaryVisibilityLedger& ledger) noexcept {
    if (![NSThread isMainThread] || !_objectInteractor || !_shapeInteractor || myDoc.IsNull()
        || myDoc->Document().IsNull() || HasActiveOperationLedger(_objectInteractor, _shapeInteractor)
        || !_shapeInteractor->selectionModeAuthorityIsExact()
        || _shapeInteractor->getSelectionMode() != ShapeSelectionMode::WholeShape
        || !XCAFDoc_DocumentTool::CheckLayerTool(myDoc->Document()->Main())
        || !XCAFDoc_DocumentTool::CheckColorTool(myDoc->Document()->Main())) { return false; }
    ledger.selectionMode = ShapeSelectionMode::WholeShape;
    return _objectInteractor->captureOrdinaryVisibilityAuthority(ledger);
}

bool Core3DViewer::repairVisibility(const OrdinaryVisibilityLedger& ledger, bool committed) noexcept {
    if (![NSThread isMainThread] || !_ordinaryEditController
        || _ordinaryEditController->state() != OrdinaryEditState::RepairPending
        || !_objectInteractor || !_shapeInteractor || myDoc.IsNull() || myContext.IsNull()
        || HasActiveOperationLedger(_objectInteractor, _shapeInteractor)
        || _shapeInteractor->getSelectionMode() != ledger.selectionMode
        || ledger.selectionMode != ShapeSelectionMode::WholeShape) { return false; }
#ifdef DEBUG
    if (_debugOrdinaryRepairFailures > 0) { --_debugOrdinaryRepairFailures; return false; }
#endif
    try {
        const auto document = myDoc->Document();
        if (document.IsNull() || document->HasOpenCommand()) { return false; }
        // Recreate only a missing visible target. Surviving presentations and
        // selection owners retain their identity; retries never duplicate it.
        for (const auto& record : ledger.records) {
            const auto& expected = committed ? record.candidate : record.previous;
            OcctObjectVisibilityState actual;
            if (!myDoc->CaptureObjectVisibilityStateForLabel(expected.object.object.label, actual)
                || !actual.IsEqual(expected)) { return false; }
            AIS_ListOfInteractive displayed;
            myContext->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
            if (displayed.Extent() > 50000) { return false; }
            int matches = 0;
            for (AIS_ListIteratorOfListOfInteractive it(displayed); it.More(); it.Next()) {
                const auto shape = Handle(AIS_Shape)::DownCast(it.Value());
                if (!shape.IsNull() && myDoc->ShapeLabel(shape).IsEqual(expected.object.object.label)) { ++matches; }
            }
            if (matches > 1) { return false; }
            if (expected.IsEffectivelyVisible() && matches == 0) {
                XCAFPrs_Style style;
                style.SetColorSurf(Quantity_NOC_GRAY80);
                style.SetColorCurv(Quantity_NOC_GRAY80);
                if (!displayWithChildren(document, expected.object.object.label, style)) { return false; }
            }
        }
        if (!_objectInteractor->repairOrdinaryVisibilityPresentation(ledger, committed)
            || !_shapeInteractor->selectionModeAuthorityIsExact()) { return false; }
#ifdef DEBUG
        if (_debugOrdinaryVisibilityAfterRepairFailures > 0) {
            --_debugOrdinaryVisibilityAfterRepairFailures;
            return false;
        }
#endif
        return true;
    } catch (...) { return false; }
}

OrdinaryEditResult Core3DViewer::setObjectVisibilityFromBrowser(
    const ObjectFrameIdentity& identity, bool visible, std::uint64_t presentationRevision,
    std::uint32_t viewportWidth, std::uint32_t viewportHeight, bool* blockedByLayer) noexcept {
    if (blockedByLayer) { *blockedByLayer = false; }
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    if (!canBeginCommittedEdit() || !_ordinaryEditController) { return OrdinaryEditResult::Busy; }
    if (viewportWidth == 0 || viewportHeight == 0
        || identity.entityIdentifier.empty() || identity.entityIdentifier.size() > 128
        || identity.entityIdentifier.find('\0') != std::string::npos
        || identity.publicationSourceIdentifier.empty() || identity.publicationSourceIdentifier.size() > 128
        || identity.publicationSourceIdentifier.find('\0') != std::string::npos
        || !_shapeInteractor || _shapeInteractor->getSelectionMode() != ShapeSelectionMode::WholeShape) {
        return OrdinaryEditResult::Invalid;
    }
    try {
        OCC_CATCH_SIGNALS
        const auto snapshot = captureSceneSnapshot(viewportWidth, viewportHeight);
        if (!snapshot || identity.publicationSourceIdentifier != snapshot->publicationSourceIdentifier
            || identity.documentGeneration != snapshot->revisions.documentGeneration
            || identity.modelRevision != snapshot->revisions.model
            || presentationRevision != snapshot->revisions.presentation) { return OrdinaryEditResult::Invalid; }
        std::size_t matches = 0;
        for (const auto& instance : snapshot->instances) {
            if (instance.entityIdentifier == identity.entityIdentifier) {
                if (instance.role != scene::RenderRole::Model) { return OrdinaryEditResult::Invalid; }
                ++matches;
            }
        }
        if (matches != 1) { return OrdinaryEditResult::Invalid; }
        const auto document = myDoc->Document();
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        TDF_LabelSequence labels;
        shapes->GetFreeShapes(labels);
        if (labels.Length() > 50000) { return OrdinaryEditResult::Invalid; }
        TDF_Label target;
        for (Standard_Integer i = 1; i <= labels.Length(); ++i) {
            const auto& label = labels.Value(i);
            if (myDoc->EntityIdentifierForLabel(label) != identity.entityIdentifier) { continue; }
            if (!target.IsNull() || !myDoc->IsEditableFreeSimpleDefinitionLabel(label)) { return OrdinaryEditResult::Invalid; }
            target = label;
        }
        OcctObjectVisibilityState state;
        if (target.IsNull() || !myDoc->CaptureObjectVisibilityStateForLabel(target, state)) { return OrdinaryEditResult::Invalid; }
        if (visible) {
            for (bool hidden : state.layerInvisibleAttributePresent) {
                if (hidden) {
                    if (blockedByLayer) { *blockedByLayer = true; }
                    return OrdinaryEditResult::Invalid;
                }
            }
        }
        OrdinaryEditResult failure = OrdinaryEditResult::Invalid;
        auto lease = _ordinaryEditController->beginVisibility({{target, visible}}, &failure);
        return lease ? lease.stageAndCommit() : failure;
    } catch (...) { return OrdinaryEditResult::Invalid; }
}

OrdinaryEditResult Core3DViewer::editSavedGroup(int operation, const std::string& groupIdentifier,
    const std::vector<std::string>& entities, const TCollection_ExtendedString& name,
    const ObjectFrameIdentity& expected, std::uint64_t presentationRevision,
    std::uint32_t width, std::uint32_t height, bool* blockedByLayer) noexcept {
    if (blockedByLayer) { *blockedByLayer = false; }
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    if (!canBeginCommittedEdit() || !_ordinaryEditController) { return OrdinaryEditResult::Busy; }
    if (operation < 0 || operation > 5 || width == 0 || height == 0
        || !_shapeInteractor || _shapeInteractor->getSelectionMode() != ShapeSelectionMode::WholeShape
        || (operation == 0 ? (entities.size() < 2 || entities.size() > 32 || !groupIdentifier.empty())
                            : (groupIdentifier.size() != 36 || !entities.empty()))
        || (operation < 2 && !OcctObjectNameIsValid(name))) { return OrdinaryEditResult::Invalid; }
    try {
        const auto snapshot = captureSceneSnapshot(width, height);
        if (!snapshot || expected.publicationSourceIdentifier != snapshot->publicationSourceIdentifier
            || expected.documentGeneration != snapshot->revisions.documentGeneration
            || expected.modelRevision != snapshot->revisions.model
            || presentationRevision != snapshot->revisions.presentation) { return OrdinaryEditResult::Invalid; }
        OcctSavedGroupState state;
        if (!myDoc->CaptureSavedGroups(state)) { return OrdinaryEditResult::Invalid; }
        auto requested = state.groups;
        auto target = std::find_if(requested.begin(), requested.end(), [&](const auto& g) { return g.identifier == groupIdentifier; });
        if (operation != 0 && (target == requested.end() || target->members.empty())) { return OrdinaryEditResult::Invalid; }
        if (operation == 0) {
            std::unordered_set<std::string> wanted;
            for (const auto& id : entities) {
                if (id.empty() || id.size() > 128 || id.find('\0') != std::string::npos || !wanted.insert(id).second) { return OrdinaryEditResult::Invalid; }
                const auto count = std::count_if(snapshot->instances.begin(), snapshot->instances.end(), [&](const auto& i) {
                    return i.entityIdentifier == id && i.role == scene::RenderRole::Model;
                });
                if (count != 1) { return OrdinaryEditResult::Invalid; }
            }
            OcctSavedGroup group;
            group.identifier = OcctDocument::NewSavedGroupIdentifier(); group.name = name;
            if (group.identifier.empty()) { return OrdinaryEditResult::Invalid; }
            TDF_LabelSequence labels;
            XCAFDoc_DocumentTool::ShapeTool(myDoc->Document()->Main())->GetFreeShapes(labels);
            if (labels.Length() > 50000) { return OrdinaryEditResult::Invalid; }
            for (Standard_Integer i = 1; i <= labels.Length(); ++i) {
                const auto label = labels.Value(i);
                if (!wanted.count(myDoc->EntityIdentifierForLabel(label))) { continue; }
                if (!myDoc->IsEditableFreeSimpleDefinitionLabel(label)) { return OrdinaryEditResult::Invalid; }
                group.members.push_back(label);
            }
            if (group.members.size() != wanted.size()) { return OrdinaryEditResult::Invalid; }
            for (auto& previous : requested) {
                previous.members.erase(std::remove_if(previous.members.begin(), previous.members.end(), [&](const auto& label) {
                    return wanted.count(myDoc->EntityIdentifierForLabel(label)) != 0;
                }), previous.members.end());
            }
            requested.erase(std::remove_if(requested.begin(), requested.end(), [](const auto& g) { return g.members.empty(); }), requested.end());
            if (requested.size() >= 128) { return OrdinaryEditResult::Invalid; }
            requested.push_back(std::move(group));
        } else if (operation == 1) { target->name = name; }
        else if (operation == 2) { requested.erase(target); }
        else if (operation == 5) { target->originPresent=Standard_False;target->origin=gp_Pnt(); }
        else {
            std::vector<OrdinaryVisibilityChange> changes;
            for (const auto& label : target->members) {
                OcctObjectVisibilityState visibility;
                if (!myDoc->CaptureObjectVisibilityStateForLabel(label, visibility)) { return OrdinaryEditResult::Invalid; }
                if (operation == 4 && std::any_of(visibility.layerInvisibleAttributePresent.begin(), visibility.layerInvisibleAttributePresent.end(), [](bool hidden) { return hidden; })) {
                    if (blockedByLayer) { *blockedByLayer = true; }
                    return OrdinaryEditResult::Invalid;
                }
                changes.push_back({label, operation == 4});
            }
            OrdinaryEditResult failure = OrdinaryEditResult::Invalid;
            auto lease = _ordinaryEditController->beginVisibility(changes, &failure);
            return lease ? lease.stageAndCommit() : failure;
        }
        OrdinaryEditResult failure = OrdinaryEditResult::Invalid;
        auto lease = _ordinaryEditController->beginGrouping(requested, &failure);
        return lease ? lease.stageAndCommit() : failure;
    } catch (...) { return OrdinaryEditResult::Invalid; }
}

bool Core3DViewer::selectSavedGroup(const ObjectFrameIdentity& expected,
    std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height,
    bool& selectionWasTouched) noexcept {
    observeNativePlanningInteraction();
    selectionWasTouched = false;

    if (![NSThread isMainThread] || !canBeginCommittedEdit() || width == 0 || height == 0
        || expected.entityIdentifier.size() != 36 || myContext.IsNull() || !_objectInteractor
        || !_shapeInteractor || _shapeInteractor->getSelectionMode() != ShapeSelectionMode::WholeShape
        || !_shapeInteractor->selectionModeAuthorityIsExact()) { return false; }
    try {
        const auto snapshot = captureSceneSnapshot(width, height);
        if (!snapshot || expected.publicationSourceIdentifier != snapshot->publicationSourceIdentifier
            || expected.documentGeneration != snapshot->revisions.documentGeneration
            || expected.modelRevision != snapshot->revisions.model
            || presentationRevision != snapshot->revisions.presentation) { return false; }
        std::unordered_set<std::string> members;
        for (const auto& item : snapshot->instances) {
            if (item.groupIdentifier != expected.entityIdentifier) { continue; }
            // Selecting only visible members would silently move a partial group.
            if (!item.visible || !item.selectable || item.role != scene::RenderRole::Model
                || !members.insert(item.entityIdentifier).second || members.size() > 32) { return false; }
        }
        if (members.empty()) { return false; }
        std::vector<Handle(AIS_InteractiveObject)> targets;
        AIS_ListOfInteractive displayed;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
        std::size_t count = 0;
        for (AIS_ListIteratorOfListOfInteractive it(displayed); it.More(); it.Next()) {
            if (++count > 50000) { return false; }
            const auto object = it.Value();
            const auto label = myDoc->ShapeLabel(object);
            if (!members.count(myDoc->EntityIdentifierForLabel(label))) { continue; }
            if (!myDoc->IsEditableFreeSimpleDefinitionLabel(label) || !myDoc->IsPresentationEditable(object)) { return false; }
            targets.push_back(object);
        }
        if (targets.size() != members.size()) { return false; }
        return _objectInteractor->replaceSelectedObjectsForBrowser(targets, selectionWasTouched);
    } catch (...) { return false; }
}


OrdinaryEditResult Core3DViewer::publishCreatedPrimitives(
    const std::vector<OrdinaryCreationRequest>& requests, std::shared_ptr<NativeModelingCommitPermit> permit) noexcept {
    if (![NSThread isMainThread] || !canBeginCommittedEdit() || !_ordinaryEditController) {
        return OrdinaryEditResult::Busy;
    }
    OrdinaryEditResult failure = OrdinaryEditResult::Invalid;
    auto lease = permit ? _ordinaryEditController->beginModelingCreation(requests,std::move(permit),&failure)
        : _ordinaryEditController->beginCreation(requests, &failure);
    return lease ? lease.stageAndCommit() : failure;
}

bool Core3DViewer::admitMeshCopy(OrdinaryCreationLedger& ledger) noexcept {
    try {
        if (!ledger.meshCopy || ledger.records.size()!=1 || !_shapeInteractor || !_objectInteractor
            || _shapeInteractor->getSelectionMode()!=ShapeSelectionMode::WholeShape
            || !_shapeInteractor->selectionModeAuthorityIsExact()
            || ledger.authority.selectionOwners.size()!=1
            || ledger.authority.selectedPresentations.size()!=1
            || ledger.authority.selectedPresentations.front().presentation!=ledger.meshCopy->presentation
            || ledger.authority.selectionOwners.front()!=ledger.meshCopy->presentation->GlobalSelOwner()
            || !myDoc->ShapeLabel(ledger.meshCopy->presentation).IsEqual(ledger.meshCopy->previous.object.object.label)) return false;
        return _objectInteractor->verifyOrdinaryNameAuthority(ledger.authority);
    } catch (...) {return false;}
}

bool Core3DViewer::repairMeshCopy(const OrdinaryCreationLedger& ledger,bool committed) noexcept {
    if (!committed) return repairCreation(ledger,false);
    if (![NSThread isMainThread] || !_ordinaryEditController
        || _ordinaryEditController->state()!=OrdinaryEditState::RepairPending
        || !_objectInteractor || !_shapeInteractor || myDoc.IsNull() || myContext.IsNull()
        || HasActiveOperationLedger(_objectInteractor,_shapeInteractor)
        || _shapeInteractor->getSelectionMode()!=ShapeSelectionMode::WholeShape
        || !_shapeInteractor->selectionModeAuthorityIsExact()) return false;
#ifdef DEBUG
    if (_debugOrdinaryRepairFailures>0) {--_debugOrdinaryRepairFailures;return false;}
#endif
    if (!_objectInteractor->repairCommittedMeshCopyPresentation(ledger)) return false;
#ifdef DEBUG
    if (_debugOrdinaryCreationAfterRepairFailures>0) {--_debugOrdinaryCreationAfterRepairFailures;return false;}
#endif
    return _shapeInteractor->selectionModeAuthorityIsExact();
}

struct MeshVertexEditWork {
    Handle(OcctDocument) owner;
    Handle(TDocStd_Document) document;
    Standard_Integer documentTime=0;
    OrdinaryTransformLedger authority;
    meshedit::NativeTopologyCapture geometry;
    ObjectFrameIdentity identity;
    std::uint64_t presentationRevision=0;
    std::uint32_t width=0,height=0;
    std::string sessionIdentifier;
    meshedit::ElementKind elementKind=meshedit::ElementKind::Vertex;
    bool consumed=false;
};

std::optional<MeshVertexEditSnapshot> Core3DViewer::prepareMeshVertexEdit(
    const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height) noexcept {
    return prepareMeshElementEdit(identity,presentationRevision,width,height,meshedit::ElementKind::Vertex);
}

std::optional<MeshVertexEditSnapshot> Core3DViewer::prepareMeshElementEdit(
    const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height,meshedit::ElementKind kind) noexcept {
    if(![NSThread isMainThread] || !canBeginCommittedEdit() || _meshRegionExtrudeWork || _meshRegionInsetWork || myDoc.IsNull()
        || myContext.IsNull() || width==0 || height==0)return std::nullopt;
    switch(kind) {
        case meshedit::ElementKind::Vertex:
        case meshedit::ElementKind::Edge:
        case meshedit::ElementKind::Triangle:break;
        default:return std::nullopt;
    }
    try {
        const auto snapshot=captureSceneSnapshot(width,height);
        if(!snapshot || snapshot->publicationSourceIdentifier!=identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration!=identity.documentGeneration
            || snapshot->revisions.model!=identity.modelRevision
            || snapshot->revisions.presentation!=presentationRevision
            || snapshot->selectionMode!=scene::ElementKind::Object
            || snapshot->selection.selected.size()!=1
            || snapshot->selection.selected[0].kind!=scene::ElementKind::Object
            || snapshot->selection.selected[0].entityIdentifier!=identity.entityIdentifier)return std::nullopt;
        myContext->InitSelected();if(!myContext->MoreSelected())return std::nullopt;
        const auto presentation=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        myContext->NextSelected();if(myContext->MoreSelected() || presentation.IsNull())return std::nullopt;
        const auto label=myDoc->ShapeLabel(presentation);
        OrdinaryTransformRecord record;
        if(!myDoc->CaptureObjectTransformStateForLabel(label,record.previous)
            || record.previous.entityIdentifier!=identity.entityIdentifier
            || record.previous.resolvedRepresentation!=OcctGeometryRepresentation::TriangleMesh
            || record.previous.authoredFramesPresent)return std::nullopt;
        auto work=std::make_shared<MeshVertexEditWork>();
        std::atomic_bool cancelled{false};
        if(meshedit::CaptureNativeTopology(record.previous.shape,work->geometry,cancelled)
            !=meshedit::TopologyResult::Ready)return std::nullopt;
        if(!myDoc->CanEditMeshVertices(label))return std::nullopt;
        record.requested.label=label;record.requested.presentation=presentation;
        record.requested.shape=record.previous.shape;record.requested.transform=record.previous.transform;
        work->authority.records.push_back(record);
        if(!admitTransform(work->authority))return std::nullopt;
        work->owner=myDoc;work->document=myDoc->Document();
        if(work->document.IsNull())return std::nullopt;
        work->documentTime=work->document->GetData()->Time();
        work->identity=identity;work->presentationRevision=presentationRevision;
        work->width=width;work->height=height;
        work->sessionIdentifier=[[NSUUID UUID].UUIDString UTF8String];
        MeshVertexEditSnapshot result;result.sessionIdentifier=work->sessionIdentifier;
        result.entityIdentifier=identity.entityIdentifier;
        const auto placed=record.previous.transform*work->geometry.meshLocation.Transformation();
        for(const auto& v:work->geometry.topology.vertices) {
            const auto p=gp_Pnt(v.point[0],v.point[1],v.point[2]).Transformed(placed);
            for(int a=1;a<=3;++a)
                if(!std::isfinite(p.Coord(a)) || std::abs(p.Coord(a))>1.e6)return std::nullopt;
            result.worldVertices.push_back({p.X(),p.Y(),p.Z()});
        }
        work->elementKind=kind;result.elementKind=kind;
        if(kind==meshedit::ElementKind::Edge) {
            result.edgeVertices.reserve(work->geometry.topology.edges.size());
            for(const auto& edge:work->geometry.topology.edges)result.edgeVertices.push_back(edge.vertices);
        } else if(kind==meshedit::ElementKind::Triangle) {
            result.triangleVertices=work->geometry.topology.triangleVertices;
        }
        _meshVertexEditWork=std::move(work);return result;
    } catch(...) {return std::nullopt;}
}

void Core3DViewer::cancelMeshVertexEdit(const std::string& sessionIdentifier) noexcept {
    if([NSThread isMainThread] && _meshVertexEditWork
        && _meshVertexEditWork->sessionIdentifier==sessionIdentifier)_meshVertexEditWork.reset();
}

OrdinaryEditResult Core3DViewer::commitMeshVertexEdit(const std::string& sessionIdentifier,
    const std::vector<std::uint32_t>& vertices,const gp_Vec& worldDelta) noexcept {
    return commitMeshElementEdit(sessionIdentifier,meshedit::ElementKind::Vertex,vertices,worldDelta);
}

OrdinaryEditResult Core3DViewer::commitMeshElementEdit(const std::string& sessionIdentifier,
    meshedit::ElementKind kind,const std::vector<std::uint32_t>& elements,const gp_Vec& worldDelta) noexcept {
    if(![NSThread isMainThread])return OrdinaryEditResult::Invalid;
    const auto work=_meshVertexEditWork;
    if(!work || work->consumed || work->sessionIdentifier!=sessionIdentifier
        || work->elementKind!=kind)return OrdinaryEditResult::Invalid;
    work->consumed=true;
    // Any uncertain outcome is retained by the ordinary command ledger. The
    // consumed UI session must not keep an older document/selection alive.
    _meshVertexEditWork.reset();
    if(!canBeginCommittedEdit())return OrdinaryEditResult::Busy;
    try {
        if(myDoc!=work->owner || myDoc->Document()!=work->document
            || work->document->GetData()->Time()!=work->documentTime)return OrdinaryEditResult::Invalid;
        const auto snapshot=captureSceneSnapshot(work->width,work->height);
        if(!snapshot || snapshot->publicationSourceIdentifier!=work->identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration!=work->identity.documentGeneration
            || snapshot->revisions.model!=work->identity.modelRevision
            || snapshot->revisions.presentation!=work->presentationRevision
            || snapshot->selectionMode!=scene::ElementKind::Object)return OrdinaryEditResult::Invalid;
        const auto& previous=work->authority.records.front().previous;
        OcctObjectTransformState actual;
        if(!myDoc->CaptureObjectTransformStateForLabel(previous.label,actual)
            || !previous.IsEqual(actual))return OrdinaryEditResult::Invalid;
        auto authority=work->authority;
        if(!admitTransform(authority) || authority.selectionOwners!=work->authority.selectionOwners
            || authority.manipulatorType!=work->authority.manipulatorType
            || authority.hadManipulator!=work->authority.hadManipulator)return OrdinaryEditResult::Invalid;
        std::atomic_bool cancelled{false};meshedit::NativeTopologyCapture fresh;
        if(meshedit::CaptureNativeTopology(actual.shape,fresh,cancelled)!=meshedit::TopologyResult::Ready
            || fresh.sourceMesh!=work->geometry.sourceMesh
            || !fresh.face.IsEqual(work->geometry.face)
            || !fresh.meshLocation.IsEqual(work->geometry.meshLocation)
            || fresh.storedNodes!=work->geometry.storedNodes
            || fresh.storedUVs!=work->geometry.storedUVs
            || fresh.storedNormals!=work->geometry.storedNormals
            || fresh.deflection!=work->geometry.deflection
            || fresh.triangleNodeIDs!=work->geometry.triangleNodeIDs)return OrdinaryEditResult::Invalid;
        // Resolve domain-qualified element IDs only after proving the exact
        // retained native geometry. UI/provider endpoint arrays never authorize
        // a move, and the same ordinal cannot change domains mid-session.
        std::vector<std::uint32_t> vertices;
        if(meshedit::ResolveElementVertices(work->geometry.topology,kind,elements,vertices,cancelled)
            !=meshedit::ElementSelectionResult::Ready)return OrdinaryEditResult::Invalid;
        OcctMeshVertexMutationCandidate candidate;
        if(!myDoc->PrepareMeshVertexMove(previous.label,vertices,worldDelta,candidate))return OrdinaryEditResult::Invalid;
        OrdinaryTransformChange request=work->authority.records.front().requested;
        request.shape=candidate.shape;request.operation=OrdinaryTransformOperation::MeshVertexMove;
        request.meshVertexMove=OrdinaryMeshVertexMove{vertices,worldDelta,candidate.partition};
        OrdinaryEditResult failure=OrdinaryEditResult::Invalid;
        auto lease=_ordinaryEditController->beginTransform({request},&failure);
        return lease?lease.stageAndCommit():failure;
    } catch(...) {return OrdinaryEditResult::Invalid;}
}

struct MeshRegionExtrudeWork {
    Handle(OcctDocument) owner;
    Handle(TDocStd_Document) document;
    Standard_Integer documentTime=0;
    OrdinaryTransformLedger authority;
    meshedit::NativeTopologyCapture geometry;
    OcctMeshRegionExtrudePreview preview;
    TDF_Label materialLabel;
    Handle(XCAFDoc_VisMaterial) material;
    Standard_Integer normalRecipe=0;
    ObjectFrameIdentity identity;
    std::uint64_t presentationRevision=0;
    std::uint32_t width=0,height=0,seedTriangle=0;
    std::string sessionIdentifier;
    bool consumed=false;
};

std::optional<MeshRegionExtrudeSnapshot> Core3DViewer::prepareMeshRegionExtrude(
    const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height,std::uint32_t seedTriangle) noexcept {
    if(![NSThread isMainThread] || !canBeginCommittedEdit() || _meshVertexEditWork
        || _meshRegionExtrudeWork || _meshRegionInsetWork || myDoc.IsNull() || myContext.IsNull() || !width || !height)
        return std::nullopt;
    try {
        const auto snapshot=captureSceneSnapshot(width,height);
        if(!snapshot || snapshot->publicationSourceIdentifier!=identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration!=identity.documentGeneration
            || snapshot->revisions.model!=identity.modelRevision
            || snapshot->revisions.presentation!=presentationRevision
            || snapshot->selectionMode!=scene::ElementKind::Object
            || snapshot->selection.selected.size()!=1
            || snapshot->selection.selected[0].kind!=scene::ElementKind::Object
            || snapshot->selection.selected[0].entityIdentifier!=identity.entityIdentifier)return std::nullopt;
        myContext->InitSelected();if(!myContext->MoreSelected())return std::nullopt;
        const auto presentation=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        myContext->NextSelected();if(myContext->MoreSelected()||presentation.IsNull())return std::nullopt;
        const auto label=myDoc->ShapeLabel(presentation);OrdinaryTransformRecord record;
        if(!myDoc->CaptureObjectTransformStateForLabel(label,record.previous)
            || record.previous.entityIdentifier!=identity.entityIdentifier
            || record.previous.resolvedRepresentation!=OcctGeometryRepresentation::TriangleMesh
            || record.previous.authoredFramesPresent || record.previous.meshUVAtlasVersion==0)return std::nullopt;
        auto work=std::make_shared<MeshRegionExtrudeWork>();std::atomic_bool cancelled{false};
        if(meshedit::CaptureNativeTopology(record.previous.shape,work->geometry,cancelled)
                !=meshedit::TopologyResult::Ready
            || !myDoc->CaptureMeshRegionExtrudePreview(label,seedTriangle,work->preview))return std::nullopt;
        const bool linked=XCAFDoc_VisMaterialTool::GetShapeMaterial(label,work->materialLabel);
        work->material=XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
        if(linked!=!work->material.IsNull() || linked!=!work->materialLabel.IsNull())return std::nullopt;
        work->normalRecipe=Core3DNormalTextureRecipeForLabel(label);
        record.requested.label=label;record.requested.presentation=presentation;
        record.requested.shape=record.previous.shape;record.requested.transform=record.previous.transform;
        work->authority.records.push_back(record);if(!admitTransform(work->authority))return std::nullopt;
        work->owner=myDoc;work->document=myDoc->Document();if(work->document.IsNull())return std::nullopt;
        work->documentTime=work->document->GetData()->Time();work->identity=identity;
        work->presentationRevision=presentationRevision;work->width=width;work->height=height;
        work->seedTriangle=seedTriangle;work->sessionIdentifier=[[NSUUID UUID].UUIDString UTF8String];
        MeshRegionExtrudeSnapshot result;result.sessionIdentifier=work->sessionIdentifier;
        result.entityIdentifier=identity.entityIdentifier;result.triangleIndices=work->preview.triangleIndices;
        const auto placed=record.previous.transform*work->geometry.meshLocation.Transformation();
        const auto normal=gp_Dir(work->preview.localUnitNormal[0],work->preview.localUnitNormal[1],work->preview.localUnitNormal[2]).Transformed(placed);
        result.worldUnitNormal={normal.X(),normal.Y(),normal.Z()};
        for(const auto& local:work->preview.localBoundary) {
            const auto point=gp_Pnt(local[0],local[1],local[2]).Transformed(placed);
            result.worldBoundary.push_back({point.X(),point.Y(),point.Z()});
        }
        _meshRegionExtrudeWork=std::move(work);return result;
    } catch(...){return std::nullopt;}
}

void Core3DViewer::cancelMeshRegionExtrude(const std::string& sessionIdentifier) noexcept {
    if([NSThread isMainThread] && _meshRegionExtrudeWork
        && _meshRegionExtrudeWork->sessionIdentifier==sessionIdentifier)_meshRegionExtrudeWork.reset();
}

OrdinaryEditResult Core3DViewer::commitMeshRegionExtrude(const std::string& sessionIdentifier,
    double distanceMM) noexcept {
    if(![NSThread isMainThread])return OrdinaryEditResult::Invalid;
    const auto work=_meshRegionExtrudeWork;
    if(!work || work->consumed || work->sessionIdentifier!=sessionIdentifier)return OrdinaryEditResult::Invalid;
    work->consumed=true;_meshRegionExtrudeWork.reset();
    if(!canBeginCommittedEdit())return OrdinaryEditResult::Busy;
    try {
        if(!std::isfinite(distanceMM)||distanceMM<=1.e-6||distanceMM>1.e5
            || myDoc!=work->owner || myDoc->Document()!=work->document
            || work->document->GetData()->Time()!=work->documentTime)return OrdinaryEditResult::Invalid;
        const auto snapshot=captureSceneSnapshot(work->width,work->height);
        if(!snapshot || snapshot->publicationSourceIdentifier!=work->identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration!=work->identity.documentGeneration
            || snapshot->revisions.model!=work->identity.modelRevision
            || snapshot->revisions.presentation!=work->presentationRevision
            || snapshot->selectionMode!=scene::ElementKind::Object)return OrdinaryEditResult::Invalid;
        const auto& previous=work->authority.records.front().previous;OcctObjectTransformState actual;
        if(!myDoc->CaptureObjectTransformStateForLabel(previous.label,actual)||!previous.IsEqual(actual))
            return OrdinaryEditResult::Invalid;
        auto authority=work->authority;
        if(!admitTransform(authority)||authority.selectionOwners!=work->authority.selectionOwners
            || authority.manipulatorType!=work->authority.manipulatorType
            || authority.hadManipulator!=work->authority.hadManipulator)return OrdinaryEditResult::Invalid;
        std::atomic_bool cancelled{false};meshedit::NativeTopologyCapture storage;
        OcctMeshRegionExtrudePreview fresh;
        if(meshedit::CaptureNativeTopology(actual.shape,storage,cancelled)!=meshedit::TopologyResult::Ready
            || !meshedit::SameRegionStorage(work->geometry,storage)
            || !myDoc->CaptureMeshRegionExtrudePreview(previous.label,work->seedTriangle,fresh)
            || fresh.triangleIndices!=work->preview.triangleIndices
            || fresh.localBoundary!=work->preview.localBoundary
            || fresh.localUnitNormal!=work->preview.localUnitNormal)return OrdinaryEditResult::Invalid;
        TDF_Label materialLabel;const bool linked=XCAFDoc_VisMaterialTool::GetShapeMaterial(previous.label,materialLabel);
        const auto material=XCAFDoc_VisMaterialTool::GetShapeMaterial(previous.label);
        if(linked!=!material.IsNull() || linked!=!materialLabel.IsNull()
            || material!=work->material || (linked&&!materialLabel.IsEqual(work->materialLabel))
            || Core3DNormalTextureRecipeForLabel(previous.label)!=work->normalRecipe)return OrdinaryEditResult::Invalid;
        OcctMeshRegionMutationCandidate candidate;
        if(!myDoc->PrepareMeshRegionExtrude(previous.label,work->seedTriangle,distanceMM,candidate))
            return OrdinaryEditResult::Invalid;
        OrdinaryTransformChange request=work->authority.records.front().requested;
        request.shape=candidate.shape;request.operation=OrdinaryTransformOperation::MeshRegionExtrude;
        request.meshRegionExtrude=OrdinaryMeshRegionExtrude{
            work->seedTriangle,work->preview.triangleIndices,distanceMM,1,candidate.partition};
        OrdinaryEditResult failure=OrdinaryEditResult::Invalid;
        auto lease=_ordinaryEditController->beginTransform({request},&failure);
        return lease?lease.stageAndCommit():failure;
    } catch(...){return OrdinaryEditResult::Invalid;}
}


struct MeshRegionInsetWork {
    Handle(OcctDocument) owner;
    Handle(TDocStd_Document) document;
    Standard_Integer documentTime=0;
    OrdinaryTransformLedger authority;
    meshedit::NativeTopologyCapture geometry;
    OcctMeshRegionExtrudePreview preview;
    TDF_Label materialLabel;
    Handle(XCAFDoc_VisMaterial) material;
    Standard_Integer normalRecipe=0;
    ObjectFrameIdentity identity;
    std::uint64_t presentationRevision=0;
    std::uint32_t width=0,height=0,seedTriangle=0;
    std::string sessionIdentifier;
    bool consumed=false;
};

std::optional<MeshRegionInsetSnapshot> Core3DViewer::prepareMeshRegionInset(
    const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
    std::uint32_t width,std::uint32_t height,std::uint32_t seedTriangle) noexcept {
    if(![NSThread isMainThread] || !canBeginCommittedEdit() || _meshVertexEditWork
        || _meshRegionExtrudeWork || _meshRegionInsetWork || myDoc.IsNull() || myContext.IsNull()
        || !width || !height)return std::nullopt;
    try {
        const auto snapshot=captureSceneSnapshot(width,height);
        if(!snapshot || snapshot->publicationSourceIdentifier!=identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration!=identity.documentGeneration
            || snapshot->revisions.model!=identity.modelRevision
            || snapshot->revisions.presentation!=presentationRevision
            || snapshot->selectionMode!=scene::ElementKind::Object
            || snapshot->selection.selected.size()!=1
            || snapshot->selection.selected[0].kind!=scene::ElementKind::Object
            || snapshot->selection.selected[0].entityIdentifier!=identity.entityIdentifier)return std::nullopt;
        myContext->InitSelected();if(!myContext->MoreSelected())return std::nullopt;
        const auto presentation=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        myContext->NextSelected();if(myContext->MoreSelected()||presentation.IsNull())return std::nullopt;
        const auto label=myDoc->ShapeLabel(presentation);OrdinaryTransformRecord record;
        if(!myDoc->CaptureObjectTransformStateForLabel(label,record.previous)
            || record.previous.entityIdentifier!=identity.entityIdentifier
            || record.previous.resolvedRepresentation!=OcctGeometryRepresentation::TriangleMesh
            || record.previous.authoredFramesPresent || record.previous.meshUVAtlasVersion==0)return std::nullopt;
        auto work=std::make_shared<MeshRegionInsetWork>();std::atomic_bool cancelled{false};
        if(meshedit::CaptureNativeTopology(record.previous.shape,work->geometry,cancelled)
                !=meshedit::TopologyResult::Ready
            || !myDoc->CaptureMeshRegionInsetPreview(label,seedTriangle,work->preview))return std::nullopt;
        const bool linked=XCAFDoc_VisMaterialTool::GetShapeMaterial(label,work->materialLabel);
        work->material=XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
        if(linked!=!work->material.IsNull() || linked!=!work->materialLabel.IsNull())return std::nullopt;
        work->normalRecipe=Core3DNormalTextureRecipeForLabel(label);
        record.requested.label=label;record.requested.presentation=presentation;
        record.requested.shape=record.previous.shape;record.requested.transform=record.previous.transform;
        work->authority.records.push_back(record);if(!admitTransform(work->authority))return std::nullopt;
        work->owner=myDoc;work->document=myDoc->Document();if(work->document.IsNull())return std::nullopt;
        work->documentTime=work->document->GetData()->Time();work->identity=identity;
        work->presentationRevision=presentationRevision;work->width=width;work->height=height;
        work->seedTriangle=seedTriangle;work->sessionIdentifier=[[NSUUID UUID].UUIDString UTF8String];
        MeshRegionInsetSnapshot result;result.sessionIdentifier=work->sessionIdentifier;
        result.entityIdentifier=identity.entityIdentifier;result.triangleIndices=work->preview.triangleIndices;
        const auto placed=record.previous.transform*work->geometry.meshLocation.Transformation();
        const auto normal=gp_Dir(work->preview.localUnitNormal[0],work->preview.localUnitNormal[1],
            work->preview.localUnitNormal[2]).Transformed(placed);
        result.worldUnitNormal={normal.X(),normal.Y(),normal.Z()};
        for(const auto& local:work->preview.localBoundary) {
            const auto point=gp_Pnt(local[0],local[1],local[2]).Transformed(placed);
            result.worldBoundary.push_back({point.X(),point.Y(),point.Z()});
        }
        _meshRegionInsetWork=std::move(work);return result;
    } catch(...){return std::nullopt;}
}

void Core3DViewer::cancelMeshRegionInset(const std::string& sessionIdentifier) noexcept {
    if([NSThread isMainThread] && _meshRegionInsetWork
        && _meshRegionInsetWork->sessionIdentifier==sessionIdentifier)_meshRegionInsetWork.reset();
}

OrdinaryEditResult Core3DViewer::commitMeshRegionInset(const std::string& sessionIdentifier,
    double distanceMM) noexcept {
    if(![NSThread isMainThread])return OrdinaryEditResult::Invalid;
    const auto work=_meshRegionInsetWork;
    if(!work || work->consumed || work->sessionIdentifier!=sessionIdentifier)return OrdinaryEditResult::Invalid;
    work->consumed=true;_meshRegionInsetWork.reset();
    if(!canBeginCommittedEdit())return OrdinaryEditResult::Busy;
    try {
        if(!std::isfinite(distanceMM)||distanceMM<=1.e-6||distanceMM>1.e5
            || myDoc!=work->owner || myDoc->Document()!=work->document
            || work->document->GetData()->Time()!=work->documentTime)return OrdinaryEditResult::Invalid;
        const auto snapshot=captureSceneSnapshot(work->width,work->height);
        if(!snapshot || snapshot->publicationSourceIdentifier!=work->identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration!=work->identity.documentGeneration
            || snapshot->revisions.model!=work->identity.modelRevision
            || snapshot->revisions.presentation!=work->presentationRevision
            || snapshot->selectionMode!=scene::ElementKind::Object)return OrdinaryEditResult::Invalid;
        const auto& previous=work->authority.records.front().previous;OcctObjectTransformState actual;
        if(!myDoc->CaptureObjectTransformStateForLabel(previous.label,actual)||!previous.IsEqual(actual))
            return OrdinaryEditResult::Invalid;
        auto authority=work->authority;
        if(!admitTransform(authority)||authority.selectionOwners!=work->authority.selectionOwners
            || authority.manipulatorType!=work->authority.manipulatorType
            || authority.hadManipulator!=work->authority.hadManipulator)return OrdinaryEditResult::Invalid;
        std::atomic_bool cancelled{false};meshedit::NativeTopologyCapture storage;
        OcctMeshRegionExtrudePreview fresh;
        if(meshedit::CaptureNativeTopology(actual.shape,storage,cancelled)!=meshedit::TopologyResult::Ready
            || !meshedit::SameRegionStorage(work->geometry,storage)
            || !myDoc->CaptureMeshRegionInsetPreview(previous.label,work->seedTriangle,fresh)
            || fresh.triangleIndices!=work->preview.triangleIndices
            || fresh.localBoundary!=work->preview.localBoundary
            || fresh.localUnitNormal!=work->preview.localUnitNormal)return OrdinaryEditResult::Invalid;
        TDF_Label materialLabel;const bool linked=XCAFDoc_VisMaterialTool::GetShapeMaterial(previous.label,materialLabel);
        const auto material=XCAFDoc_VisMaterialTool::GetShapeMaterial(previous.label);
        if(linked!=!material.IsNull() || linked!=!materialLabel.IsNull()
            || material!=work->material || (linked&&!materialLabel.IsEqual(work->materialLabel))
            || Core3DNormalTextureRecipeForLabel(previous.label)!=work->normalRecipe)return OrdinaryEditResult::Invalid;
        OcctMeshRegionMutationCandidate candidate;
        if(!myDoc->PrepareMeshRegionInset(previous.label,work->seedTriangle,distanceMM,candidate))
            return OrdinaryEditResult::Invalid;
        OrdinaryTransformChange request=work->authority.records.front().requested;
        request.shape=candidate.shape;request.operation=OrdinaryTransformOperation::MeshRegionInset;
        request.meshRegionInset=OrdinaryMeshRegionInset{work->seedTriangle,work->preview.triangleIndices,
            distanceMM,candidate.partition,candidate.centerSeedTriangle};
        OrdinaryEditResult failure=OrdinaryEditResult::Invalid;
        auto lease=_ordinaryEditController->beginTransform({request},&failure);
        return lease?lease.stageAndCommit():failure;
    } catch(...){return OrdinaryEditResult::Invalid;}
}

#ifdef DEBUG
// Synchronous test-only corruption window. Restore the exact original storage
// before returning, including handle identity; never leave a fixture mutated.
bool Core3DViewer::debugProbeMeshVertexStorageChange(int mode) noexcept {
    if(![NSThread isMainThread] || mode<0 || mode>8 || !_meshVertexEditWork
        || !canBeginCommittedEdit())return false;
    const auto work=_meshVertexEditWork;
    const auto mesh=work->geometry.sourceMesh;
    const int prefix=mesh.IsNull()?0:3*mesh->NbTriangles();
    if(mesh.IsNull() || prefix<=0 || mesh->NbNodes()!=2*prefix
        || !mesh->HasUVNodes() || !mesh->HasNormals())return false;
    Handle(Poly_Triangulation) saved;
    try {saved=mesh->Copy();}catch(...){return false;}
    if(saved.IsNull() || saved==mesh)return false;
    const auto document=work->document;
    const auto beforeTime=document->GetData()->Time();
    const auto beforeUndo=document->GetAvailableUndos();
    const auto beforeRedo=document->GetAvailableRedos();
    const auto restore=[&]() noexcept -> bool {
        try {
            for(int node=1;node<=mesh->NbNodes();++node) {
                mesh->SetNode(node,saved->Node(node));
                mesh->SetUVNode(node,saved->UVNode(node));
                gp_Vec3f n;saved->Normal(node,n);mesh->SetNormal(node,n);
            }
            for(int t=1;t<=mesh->NbTriangles();++t)mesh->SetTriangle(t,saved->Triangle(t));
            mesh->Deflection(saved->Deflection());
            if(mode==8)BRep_Builder().UpdateFace(work->geometry.face,mesh);
            return true;
        }catch(...){return false;}
    };
    OrdinaryEditResult result=OrdinaryEditResult::RetryableFailure;
    bool invoked=false;
    try {
        const int active=prefix+1;
        switch(mode) {
            case 0: {
                // Move all exact coincident stored positions together, leaving
                // topology valid; stale data alone must reject the old session.
                const auto point=mesh->Node(active);
                for(int node=1;node<=mesh->NbNodes();++node)
                    if(mesh->Node(node).IsEqual(point,0.0))
                        mesh->SetNode(node,gp_Pnt(point.X()+0.01,point.Y(),point.Z()));
                break;
            }
            case 1: {auto p=mesh->Node(1);p.SetX(p.X()+0.01);mesh->SetNode(1,p);break;}
            case 2: case 3: {
                const int node=mode==2?active:1;auto uv=mesh->UVNode(node);
                uv.SetX(uv.X()+0.01);mesh->SetUVNode(node,uv);break;
            }
            case 4: case 5: {
                const int node=mode==4?active:1;gp_Vec3f n;mesh->Normal(node,n);
                mesh->SetNormal(node,gp_Vec3f(-n[0],-n[1],-n[2]));break;
            }
            case 6: mesh->Deflection(mesh->Deflection()+0.01);break;
            case 7: {
                int a,b,c;mesh->Triangle(1).Get(a,b,c);
                mesh->SetTriangle(1,Poly_Triangle(b,c,a));break;
            }
            case 8: BRep_Builder().UpdateFace(work->geometry.face,mesh->Copy());break;
        }
        invoked=true;
        result=commitMeshVertexEdit(work->sessionIdentifier,{0},gp_Vec(0.5,0,0));
    }catch(...){invoked=false;}
    const bool restored=restore();
    if(!restored || !invoked || result!=OrdinaryEditResult::Invalid
        || _meshVertexEditWork || document->GetData()->Time()!=beforeTime
        || document->GetAvailableUndos()!=beforeUndo || document->GetAvailableRedos()!=beforeRedo)
        return false;
    std::atomic_bool cancelled{false};meshedit::NativeTopologyCapture after;
    return meshedit::CaptureNativeTopology(work->geometry.shape,after,cancelled)==meshedit::TopologyResult::Ready
        && after.sourceMesh==work->geometry.sourceMesh
        && after.face.IsEqual(work->geometry.face)
        && after.meshLocation.IsEqual(work->geometry.meshLocation)
        && after.storedNodes==work->geometry.storedNodes
        && after.storedUVs==work->geometry.storedUVs
        && after.storedNormals==work->geometry.storedNormals
        && after.deflection==work->geometry.deflection
        && after.triangleNodeIDs==work->geometry.triangleNodeIDs;
}

// Test-only, synchronous corruption window over an exact selected marker-3
// owner. Restore the same triangulation handle and every byte before return.
bool Core3DViewer::debugProbeMeshUVRepackStorageRefusal(int mode) noexcept {
    if(![NSThread isMainThread] || mode<0 || mode>2 || !canBeginCommittedEdit()
        || myContext.IsNull() || myDoc.IsNull())return false;
    myContext->InitSelected();if(!myContext->MoreSelected())return false;
    const auto selected=myContext->SelectedInteractive();myContext->NextSelected();
    if(myContext->MoreSelected())return false;
    const auto label=myDoc->ShapeLabel(selected);OcctObjectTransformState state;
    if(label.IsNull() || !myDoc->CaptureObjectTransformStateForLabel(label,state)
        || state.meshUVAtlasVersion!=3)return false;
    if(mode==2)return myDoc->DebugProbeMeshUVRepackRecipeMismatch(label);
    std::atomic_bool cancelled{false};meshedit::NativeTopologyCapture captured;
    if(meshedit::CaptureNativeTopology(state.shape,captured,cancelled)!=meshedit::TopologyResult::Ready
        || captured.sourceMesh.IsNull() || !captured.sourceMesh->HasNormals())return false;
    const auto mesh=captured.sourceMesh;Handle(Poly_Triangulation) saved;
    try{saved=mesh->Copy();}catch(...){return false;}
    if(saved.IsNull()||saved==mesh)return false;
    const auto document=myDoc->Document();const auto beforeTime=document->GetData()->Time();
    const auto beforeUndo=document->GetAvailableUndos(),beforeRedo=document->GetAvailableRedos();
    const auto restore=[&]() noexcept {
        try {
            for(int node=1;node<=mesh->NbNodes();++node){
                mesh->SetNode(node,saved->Node(node));mesh->SetUVNode(node,saved->UVNode(node));
                gp_Vec3f normal;saved->Normal(node,normal);mesh->SetNormal(node,normal);
            }
            for(int triangle=1;triangle<=mesh->NbTriangles();++triangle)
                mesh->SetTriangle(triangle,saved->Triangle(triangle));
            mesh->Deflection(saved->Deflection());return true;
        }catch(...){return false;}
    };
    TopoDS_Shape candidate;bool refused=false;
    try {
        if(mode==0){int a,b,c;mesh->Triangle(1).Get(a,b,c);mesh->SetTriangle(1,Poly_Triangle(b,c,a));}
        else {gp_Vec3f n;mesh->Normal(1,n);mesh->SetNormal(1,gp_Vec3f(-n[0],-n[1],-n[2]));}
        refused=!myDoc->PrepareTriangleUVAtlas(label,candidate,OcctMeshUVAtlasOptions{2,1024,8})
            && candidate.IsNull();
    }catch(...){refused=false;}
    if(!restore()||!refused||document->GetData()->Time()!=beforeTime
        ||document->GetAvailableUndos()!=beforeUndo||document->GetAvailableRedos()!=beforeRedo)return false;
    meshedit::NativeTopologyCapture after;
    return meshedit::CaptureNativeTopology(state.shape,after,cancelled)==meshedit::TopologyResult::Ready
        && after.sourceMesh==mesh && after.storedNodes==captured.storedNodes
        && after.storedUVs==captured.storedUVs && after.storedNormals==captured.storedNormals
        && after.triangleNodeIDs==captured.triangleNodeIDs;
}
#endif

OrdinaryEditResult Core3DViewer::createSourceRetainedMeshCopy(
    const ObjectFrameIdentity& identity,std::uint32_t width,std::uint32_t height) noexcept {
    if (![NSThread isMainThread]) return OrdinaryEditResult::Invalid;
    if (!canBeginCommittedEdit() || !_ordinaryEditController) return OrdinaryEditResult::Busy;
    try {
        if (width==0 || height==0 || identity.entityIdentifier.empty() || identity.entityIdentifier.size()>128
            || identity.publicationSourceIdentifier.empty() || identity.publicationSourceIdentifier.size()>128
            || identity.entityIdentifier.find('\0')!=std::string::npos
            || identity.publicationSourceIdentifier.find('\0')!=std::string::npos
            || !_shapeInteractor || _shapeInteractor->getSelectionMode()!=ShapeSelectionMode::WholeShape)
            return OrdinaryEditResult::Invalid;
        const auto snapshot=captureSceneSnapshot(width,height);
        if (!snapshot || snapshot->publicationSourceIdentifier!=identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration!=identity.documentGeneration
            || snapshot->revisions.model!=identity.modelRevision || myContext.IsNull() || myDoc.IsNull())
            return OrdinaryEditResult::Invalid;
        myContext->InitSelected();if (!myContext->MoreSelected()) return OrdinaryEditResult::Invalid;
        const auto presentation=Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
        myContext->NextSelected();if (myContext->MoreSelected() || presentation.IsNull()) return OrdinaryEditResult::Invalid;
        const auto label=myDoc->ShapeLabel(presentation);
        OcctObjectNameState source;
        if (label.IsNull() || myDoc->EntityIdentifierForLabel(label)!=identity.entityIdentifier
            || !myDoc->CaptureObjectNameStateForLabel(label,source)) return OrdinaryEditResult::Invalid;
        TCollection_ExtendedString name("Mesh copy");
        if (source.namePresent && !source.name.IsEmpty()) {
            auto candidate=source.name;candidate.AssignCat(TCollection_ExtendedString(" mesh"));
            if (OcctObjectNameIsValid(candidate)) name=candidate;
        }
        OrdinaryEditResult failure=OrdinaryEditResult::Invalid;
        auto lease=_ordinaryEditController->beginMeshCopy(presentation,name,&failure);
        return lease?lease.stageAndCommit():failure;
    } catch (...) {return OrdinaryEditResult::Invalid;}
}

bool Core3DViewer::admitCreation(OrdinaryCreationLedger& ledger) noexcept {
    try {
        if (myContext.IsNull() || myDoc.IsNull() || !admitNames(ledger.authority)) { return false; }
        for (const auto& record : ledger.records) {
            const auto& presentation = record.requested.presentation;
            if (presentation.IsNull() || presentation->HasInteractiveContext()
                || myContext->IsDisplayed(presentation) || !myDoc->ShapeLabel(presentation).IsNull()) { return false; }
        }
        return true;
    } catch (...) { return false; }
}

bool Core3DViewer::repairCreation(const OrdinaryCreationLedger& ledger, bool committed) noexcept {
    try {
        if (![NSThread isMainThread] || !_ordinaryEditController
            || _ordinaryEditController->state() != OrdinaryEditState::RepairPending
            || myContext.IsNull() || myDoc.IsNull() || !_objectInteractor || !_shapeInteractor
            || HasActiveOperationLedger(_objectInteractor, _shapeInteractor)
            || !_shapeInteractor->selectionModeAuthorityIsExact()
            || _shapeInteractor->getSelectionMode() != ledger.authority.selectionMode
            || !_objectInteractor->verifyOrdinaryNameAuthority(ledger.authority)) { return false; }
#ifdef DEBUG
        if (_debugOrdinaryRepairFailures > 0) { --_debugOrdinaryRepairFailures; return false; }
#endif
        const auto sameTransform = [](const gp_Trsf& a, const gp_Trsf& b) {
            for (int row = 1; row <= 3; ++row) {
                for (int column = 1; column <= 4; ++column) {
                    if (a.Value(row, column) != b.Value(row, column)) { return false; }
                }
            }
            return true;
        };
        const auto mode = static_cast<Standard_Integer>(ledger.authority.selectionMode);
        for (const auto& record : ledger.records) {
            const auto& presentation = record.requested.presentation;
            if (presentation.IsNull() || !presentation->Shape().IsEqual(record.shape)
                || !sameTransform(presentation->LocalTransformation(), record.transform)
                || (presentation->HasInteractiveContext() && presentation->InteractiveContext() != myContext.get())) { return false; }
            if (committed) {
                OcctObjectNameState actual;
                if (!myDoc->CaptureObjectNameStateForLabel(record.candidate.object.label, actual)
                    || !record.candidate.IsEqual(actual)
                    || !myDoc->ShapeLabel(presentation).IsEqual(record.candidate.object.label)
                    || !myDoc->IsPresentationEditable(presentation)) { return false; }
                if (!myContext->IsDisplayed(presentation)) {
                    myContext->Display(presentation, AIS_Shaded, mode, Standard_False);
                }
#ifdef DEBUG
                if (_debugOrdinaryCreationAfterRepairFailures > 0) {
                    --_debugOrdinaryCreationAfterRepairFailures; return false;
                }
#endif
                TColStd_ListOfInteger modes;
                myContext->ActivatedModes(presentation, modes);
                if (!myContext->IsDisplayed(presentation) || modes.Extent() != 1 || modes.First() != mode) { return false; }
            } else {
                if (presentation->HasInteractiveContext()) { myContext->Remove(presentation, Standard_False); }
                if (myContext->IsDisplayed(presentation) || presentation->HasInteractiveContext()) { return false; }
            }
        }
        if (!_objectInteractor->verifyOrdinaryNameAuthority(ledger.authority)) { return false; }
        myContext->UpdateCurrentViewer();
        return true;
    } catch (...) { return false; }
}

bool Core3DViewer::admitGrouping(OrdinaryGroupingLedger& ledger) noexcept {
    return _shapeInteractor && _shapeInteractor->getSelectionMode() == ShapeSelectionMode::WholeShape
        && admitNames(ledger.authority);
}
bool Core3DViewer::repairGrouping(const OrdinaryGroupingLedger& ledger, bool committed) noexcept {
    return repairNames(ledger.authority, committed);
}

bool Core3DViewer::admitNames(OrdinaryNameLedger& ledger) noexcept {
    if (![NSThread isMainThread] || !_objectInteractor || !_shapeInteractor
        || HasActiveOperationLedger(_objectInteractor, _shapeInteractor)
        || !_shapeInteractor->selectionModeAuthorityIsExact()) { return false; }
    ledger.selectionMode = _shapeInteractor->getSelectionMode();
    return _objectInteractor->captureOrdinaryNameAuthority(ledger);
}

bool Core3DViewer::repairNames(const OrdinaryNameLedger& ledger, bool) noexcept {
    if (![NSThread isMainThread]) { return false; }
#ifdef DEBUG
    if (_debugOrdinaryRepairFailures > 0) { --_debugOrdinaryRepairFailures; return false; }
#endif
    // Metadata leaves existing presentation and topology owners intact. A
    // mismatch retains recovery authority; never erase the user's selection.
    const bool valid = _ordinaryEditController && _ordinaryEditController->state() == OrdinaryEditState::RepairPending
        && _objectInteractor && _shapeInteractor
        && !HasActiveOperationLedger(_objectInteractor, _shapeInteractor)
        && _shapeInteractor->selectionModeAuthorityIsExact()
        && _shapeInteractor->getSelectionMode() == ledger.selectionMode
        && _objectInteractor->verifyOrdinaryNameAuthority(ledger);
    if (!valid) return false;
    return _objectInteractor->positionManipulatorAtExactSavedGroupOrigin()
        != ObjectInteractor::SavedGroupPivotResult::Failed;
}

namespace {
// N2 deliberately has no associative source link. Resolve only exact retained
// B-reps, including hidden originals. Never infer a source from names/bounds.
bool ResolveCoherentAtlasOptions(const Handle(OcctDocument)& document,
    const TDF_Label& copyLabel, OcctMeshUVAtlasOptions& options) {
    options.curvedSource.Nullify();
    if (options.version != 2) return true;
    core3d::provenance::SourceFaceProvenanceRecord provenance;
    if (document->TryCopySourceFaceProvenanceForLabel(copyLabel, provenance)
        != core3d::provenance::CopySourceFaceProvenanceReadState::Present) return true;
    TDF_LabelSequence labels;
    XCAFDoc_DocumentTool::ShapeTool(document->Document()->Main())->GetFreeShapes(labels);
    if (labels.Length() > 50000) return false;
    for (int i=1; i<=labels.Length(); ++i) {
        const auto label=labels.Value(i);
        if (label.IsEqual(copyLabel)) continue;
        OcctObjectTransformState source;
        if (!document->CaptureObjectTransformStateForLabel(label,source)
            || source.resolvedRepresentation != OcctGeometryRepresentation::BRep) continue;
        std::atomic_bool cancelled{false};
        core3d::meshcopy::CurrentTessellationCopy copy;
        if (core3d::meshcopy::PrepareCurrentTessellationCopy(source.shape,copy,cancelled)
                != core3d::meshcopy::PreparationResult::Ready
            || !core3d::provenance::SameSourceFaces(provenance.faces,copy.faces)) continue;
        TopLoc_Location location;
        const auto mesh=BRep_Tool::Triangulation(copy.face,location);
        std::string digest;
        if (location.IsIdentity() && core3d::provenance::CopyTriangulationDigest(mesh,digest)
            && digest==provenance.digest) {
            options.curvedSource=label;
            return true;
        }
    }
    // Present provenance with a removed/edited source cannot authorize live D5
    // reads. Refuse without mutation rather than silently publish a planar atlas.
    return false;
}
}

std::optional<OcctMeshUVAtlasPreview> Core3DViewer::previewCoherentUVAtlas(
    const ObjectFrameIdentity& identity, std::uint32_t width, std::uint32_t height,
    const std::optional<OcctMeshUVAtlasOptions>& options) noexcept {
    if (![NSThread isMainThread]) { return std::nullopt; }
    if (!canBeginCommittedEdit() || !_ordinaryEditController) { return std::nullopt; }
    try {
        if (width == 0 || height == 0 || identity.entityIdentifier.empty()
            || identity.entityIdentifier.size() > 128 || identity.publicationSourceIdentifier.empty()
            || identity.publicationSourceIdentifier.size() > 128) { return std::nullopt; }
        const auto snapshot = captureSceneSnapshot(width, height);
        if (!snapshot || snapshot->publicationSourceIdentifier != identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration != identity.documentGeneration
            || snapshot->revisions.model != identity.modelRevision
            || myContext.IsNull() || myDoc.IsNull()) { return std::nullopt; }
        myContext->InitSelected();
        if (!myContext->MoreSelected()) { return std::nullopt; }
        const auto selected = myContext->SelectedInteractive();
        const auto presentation = Handle(AIS_Shape)::DownCast(selected);
        const auto label = myDoc->ShapeLabel(selected);
        myContext->NextSelected();
        if (myContext->MoreSelected() || presentation.IsNull() || label.IsNull()
            || myDoc->EntityIdentifierForLabel(label) != identity.entityIdentifier) { return std::nullopt; }
        OcctObjectTransformState previous;
        if (!myDoc->CaptureObjectTransformStateForLabel(label, previous)) { return std::nullopt; }
        if (options && options->version != 2) return std::nullopt;
        OcctMeshUVAtlasOptions resolved = options.value_or(OcctMeshUVAtlasOptions{});
        if (options && !ResolveCoherentAtlasOptions(myDoc,label,resolved)) return std::nullopt;
        TopoDS_Shape candidate; OcctMeshUVAtlasPreview preview;
        if (!options) {
            // Discover the current settings without attempting replacement at
            // guessed defaults. This remains available under attached images.
            if (!myDoc->CaptureMeshUVAtlasPreview(label,preview)) return std::nullopt;
        } else if (previous.meshUVAtlasVersion==2 && previous.meshUVAtlasSettings[0]==options->resolution
            && previous.meshUVAtlasSettings[1]==options->gutterPixels
            && (resolved.curvedSource.IsNull() || myDoc->HasCurvedUVLayoutForLabel(label))) {
            // Match Generate's Unchanged semantics after later geometry edits.
            // Reading a stored atlas does not authorize replacement under images.
            if (!myDoc->CaptureMeshUVAtlasPreview(label,preview)) return std::nullopt;
        } else if (!myDoc->PrepareTriangleUVAtlas(label,candidate,resolved,&preview)) return std::nullopt;
        preview.authoredResolution=previous.meshUVAtlasSettings[0];
        preview.authoredGutterPixels=previous.meshUVAtlasSettings[1];
        return preview;
    } catch (...) { return std::nullopt; }
}

OrdinaryEditResult Core3DViewer::generateTriangleUVAtlas(
    const ObjectFrameIdentity& identity, std::uint32_t width, std::uint32_t height, const OcctMeshUVAtlasOptions& options) noexcept {
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    if (!canBeginCommittedEdit() || !_ordinaryEditController) { return OrdinaryEditResult::Busy; }
    try {
        if (width == 0 || height == 0 || identity.entityIdentifier.empty()
            || identity.entityIdentifier.size() > 128 || identity.publicationSourceIdentifier.empty()
            || identity.publicationSourceIdentifier.size() > 128) { return OrdinaryEditResult::Invalid; }
        const auto snapshot = captureSceneSnapshot(width, height);
        if (!snapshot || snapshot->publicationSourceIdentifier != identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration != identity.documentGeneration
            || snapshot->revisions.model != identity.modelRevision
            || myContext.IsNull() || myDoc.IsNull()) { return OrdinaryEditResult::Invalid; }
        myContext->InitSelected();
        if (!myContext->MoreSelected()) { return OrdinaryEditResult::Invalid; }
        const auto selected = myContext->SelectedInteractive();
        const auto presentation = Handle(AIS_Shape)::DownCast(selected);
        const auto label = myDoc->ShapeLabel(selected);
        myContext->NextSelected();
        if (myContext->MoreSelected() || presentation.IsNull() || label.IsNull()
            || myDoc->EntityIdentifierForLabel(label) != identity.entityIdentifier) { return OrdinaryEditResult::Invalid; }
        OcctObjectTransformState previous;
        if (!myDoc->CaptureObjectTransformStateForLabel(label, previous)) { return OrdinaryEditResult::Invalid; }
        OcctMeshUVAtlasOptions resolved=options;
        if (!ResolveCoherentAtlasOptions(myDoc,label,resolved)) return OrdinaryEditResult::Invalid;
        if ((options.version == 1 && options.resolution == 0 && options.gutterPixels == 0 && previous.meshUVAtlasVersion == 1)
            || (options.version == 2 && previous.meshUVAtlasVersion == 2
                && previous.meshUVAtlasSettings[0] == options.resolution
                && previous.meshUVAtlasSettings[1] == options.gutterPixels
                && (resolved.curvedSource.IsNull() || myDoc->HasCurvedUVLayoutForLabel(label)))) { return OrdinaryEditResult::NoChange; }
        TopoDS_Shape candidate;
        if (!myDoc->PrepareTriangleUVAtlas(label, candidate, resolved)) { return OrdinaryEditResult::Invalid; }
        OrdinaryTransformChange request;
        request.label = label; request.presentation = presentation; request.shape = candidate;
        request.transform = previous.transform; request.operation = OrdinaryTransformOperation::MeshUVAtlas;
        request.meshUVAtlasOptions = resolved;
        OrdinaryEditResult failure = OrdinaryEditResult::Invalid;
        auto lease = _ordinaryEditController->beginTransform({request}, &failure);
        return lease ? lease.stageAndCommit() : failure;
    } catch (...) { return OrdinaryEditResult::Invalid; }
}

namespace {
bool E1OwnerKeyForLabel(const Handle(OcctDocument)& owner, const TDF_Label& label,
                        core3d::retained_recipe::OwnerKey& key) noexcept {
    try {
        return !owner.IsNull() && !label.IsNull()
            && core3d::receipt::ParseUUID(owner->DocumentIdentifier(), key.document)
            && core3d::receipt::ParseUUID(owner->EntityIdentifierForLabel(label), key.entity)
            && core3d::receipt::ParseUUID(owner->DefinitionIdentifierForLabel(label), key.definition)
            && core3d::retained_recipe::Valid(key);
    } catch (...) { return false; }
}

OrdinaryEditResult E1Finish(const std::shared_ptr<native_opening::CommandLease>& lease,
                            OcctRetainedFinishingOutcome outcome) noexcept {
    if (!lease) return OrdinaryEditResult::Busy;
    if (outcome == OcctRetainedFinishingOutcome::Committed)
        return lease->commit() ? OrdinaryEditResult::Committed
                               : OrdinaryEditResult::OutcomeUnknown;
    if (!lease->abort()) return OrdinaryEditResult::OutcomeUnknown;
    if (outcome == OcctRetainedFinishingOutcome::Absent)
        return OrdinaryEditResult::NoChange;
    if (outcome == OcctRetainedFinishingOutcome::Busy)
        return OrdinaryEditResult::Busy;
    return OrdinaryEditResult::Invalid;
}
} // namespace

OrdinaryEditResult Core3DViewer::produceRetainedFinishing(
    const ObjectFrameIdentity& identity, std::uint32_t width, std::uint32_t height,
    const OcctRetainedFinishingSettings& settings) noexcept {
    if (![NSThread isMainThread]) return OrdinaryEditResult::Invalid;
    if (!canBeginCommittedEdit() || myDoc.IsNull() || myContext.IsNull())
        return OrdinaryEditResult::Busy;
    try {
        if (width == 0 || height == 0 || identity.entityIdentifier.empty()
            || identity.entityIdentifier.size() > 128
            || identity.publicationSourceIdentifier.empty()
            || identity.publicationSourceIdentifier.size() > 128)
            return OrdinaryEditResult::Invalid;
        const auto snapshot = captureSceneSnapshot(width, height);
        if (!snapshot || snapshot->publicationSourceIdentifier != identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration != identity.documentGeneration
            || snapshot->revisions.model != identity.modelRevision
            || snapshot->selectionMode != scene::ElementKind::Object
            || snapshot->selection.selected.size() != 1
            || snapshot->selection.selected[0].kind != scene::ElementKind::Object
            || snapshot->selection.selected[0].entityIdentifier != identity.entityIdentifier)
            return OrdinaryEditResult::Invalid;
        myContext->InitSelected();
        if (!myContext->MoreSelected()) return OrdinaryEditResult::Invalid;
        const auto selected = myContext->SelectedInteractive();
        const auto label = myDoc->ShapeLabel(selected);
        myContext->NextSelected();
        core3d::retained_recipe::OwnerKey owner;
        if (myContext->MoreSelected() || !E1OwnerKeyForLabel(myDoc, label, owner))
            return OrdinaryEditResult::Invalid;
        const auto context = captureNativeOpeningContext(width, height, {identity.entityIdentifier});
        if (!context) return OrdinaryEditResult::Busy;
        const auto lease = context->beginCommandLease(context->openingFence(), width, height);
        return E1Finish(lease, myDoc->ProduceRetainedFinishing(owner, settings));
    } catch (...) { return OrdinaryEditResult::Invalid; }
}

OrdinaryEditResult Core3DViewer::regenerateRetainedFinishing(
    const ObjectFrameIdentity& identity, std::uint32_t width, std::uint32_t height) noexcept {
    if (![NSThread isMainThread]) return OrdinaryEditResult::Invalid;
    if (!canBeginCommittedEdit() || myDoc.IsNull() || myContext.IsNull())
        return OrdinaryEditResult::Busy;
    try {
        const auto snapshot = captureSceneSnapshot(width, height);
        if (!snapshot || snapshot->publicationSourceIdentifier != identity.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration != identity.documentGeneration
            || snapshot->revisions.model != identity.modelRevision
            || snapshot->selectionMode != scene::ElementKind::Object
            || snapshot->selection.selected.size() != 1
            || snapshot->selection.selected[0].entityIdentifier != identity.entityIdentifier)
            return OrdinaryEditResult::Invalid;
        myContext->InitSelected();
        if (!myContext->MoreSelected()) return OrdinaryEditResult::Invalid;
        const auto label = myDoc->ShapeLabel(myContext->SelectedInteractive());
        myContext->NextSelected();
        core3d::retained_recipe::OwnerKey owner;
        if (myContext->MoreSelected() || !E1OwnerKeyForLabel(myDoc, label, owner))
            return OrdinaryEditResult::Invalid;
        const auto context = captureNativeOpeningContext(width, height, {identity.entityIdentifier});
        if (!context) return OrdinaryEditResult::Busy;
        const auto lease = context->beginCommandLease(context->openingFence(), width, height);
        return E1Finish(lease, myDoc->RegenerateRetainedFinishing(owner));
    } catch (...) { return OrdinaryEditResult::Invalid; }
}

OcctRetainedFinishingCurrentness Core3DViewer::retainedFinishingCurrentness(
    const ObjectFrameIdentity& identity) const noexcept {
    if (![NSThread isMainThread] || myDoc.IsNull() || identity.entityIdentifier.empty())
        return OcctRetainedFinishingCurrentness::Absent;
    try {
        const auto document = myDoc->Document();
        if (document.IsNull()) return OcctRetainedFinishingCurrentness::Absent;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return OcctRetainedFinishingCurrentness::Absent;
        TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
        TDF_Label match;
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            const auto label = roots.Value(index);
            if (myDoc->EntityIdentifierForLabel(label) != identity.entityIdentifier) continue;
            if (!match.IsNull()) return OcctRetainedFinishingCurrentness::Absent;
            match = label;
        }
        core3d::retained_recipe::OwnerKey owner;
        return E1OwnerKeyForLabel(myDoc, match, owner)
            ? myDoc->RetainedFinishingCurrentness(owner)
            : OcctRetainedFinishingCurrentness::Absent;
    } catch (...) { return OcctRetainedFinishingCurrentness::Stale; }
}

OrdinaryEditResult Core3DViewer::renameObjectFromBrowser(
    const ObjectFrameIdentity& identity, const TCollection_ExtendedString& name,
    std::uint32_t viewportWidth, std::uint32_t viewportHeight) noexcept {
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    if (!canBeginCommittedEdit() || !_ordinaryEditController) { return OrdinaryEditResult::Busy; }
    if (!OcctObjectNameIsValid(name) || viewportWidth == 0 || viewportHeight == 0
        || identity.entityIdentifier.empty() || identity.entityIdentifier.size() > 128
        || identity.entityIdentifier.find('\0') != std::string::npos
        || identity.publicationSourceIdentifier.empty() || identity.publicationSourceIdentifier.size() > 128
        || identity.publicationSourceIdentifier.find('\0') != std::string::npos) { return OrdinaryEditResult::Invalid; }
    try {
        OCC_CATCH_SIGNALS
        const auto snapshot = captureSceneSnapshot(viewportWidth, viewportHeight);
        if (!snapshot || identity.publicationSourceIdentifier != snapshot->publicationSourceIdentifier
            || identity.documentGeneration != snapshot->revisions.documentGeneration
            || identity.modelRevision != snapshot->revisions.model) { return OrdinaryEditResult::Invalid; }
        std::size_t matches = 0;
        for (const auto& instance : snapshot->instances) {
            if (instance.entityIdentifier == identity.entityIdentifier) {
                if (instance.role != scene::RenderRole::Model) { return OrdinaryEditResult::Invalid; }
                ++matches;
            }
        }
        if (matches != 1) { return OrdinaryEditResult::Invalid; }
        const auto document = myDoc->Document();
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        TDF_LabelSequence labels;
        shapes->GetFreeShapes(labels);
        if (labels.Length() > 50000) { return OrdinaryEditResult::Invalid; }
        TDF_Label target;
        for (Standard_Integer i = 1; i <= labels.Length(); ++i) {
            const auto& label = labels.Value(i);
            if (myDoc->EntityIdentifierForLabel(label) != identity.entityIdentifier) { continue; }
            if (!target.IsNull() || !myDoc->IsEditableFreeSimpleDefinitionLabel(label)) {
                return OrdinaryEditResult::Invalid;
            }
            target = label;
        }
        if (target.IsNull()) { return OrdinaryEditResult::Invalid; }
        OrdinaryEditResult failure = OrdinaryEditResult::Invalid;
        auto lease = _ordinaryEditController->beginNames({{target, name}}, &failure);
        return lease ? lease.stageAndCommit() : failure;
    } catch (...) { return OrdinaryEditResult::Invalid; }
}

bool Core3DViewer::admitTransform(OrdinaryTransformLedger& ledger) noexcept {
    return [NSThread isMainThread] && _objectInteractor && _shapeInteractor
        && !HasActiveOperationLedger(_objectInteractor, _shapeInteractor)
        && _shapeInteractor->selectionModeAuthorityIsExact()
        && _shapeInteractor->getSelectionMode() == ShapeSelectionMode::WholeShape
        && _objectInteractor->captureOrdinaryTransformAuthority(ledger);
}
bool Core3DViewer::repairTransform(const OrdinaryTransformLedger& ledger, bool committed) noexcept {
    if (![NSThread isMainThread]) { return false; }
#ifdef DEBUG
    if (_debugOrdinaryRepairFailures > 0) { --_debugOrdinaryRepairFailures; return false; }
#endif
    const bool valid = [NSThread isMainThread] && _ordinaryEditController
        && _ordinaryEditController->state() == OrdinaryEditState::RepairPending
        && _objectInteractor && _shapeInteractor
        && !HasActiveOperationLedger(_objectInteractor, _shapeInteractor)
        && _shapeInteractor->selectionModeAuthorityIsExact()
        && _shapeInteractor->getSelectionMode() == ShapeSelectionMode::WholeShape
        && _objectInteractor->repairOrdinaryTransformPresentation(ledger, committed);
    if (!valid) return false;
    return _objectInteractor->positionManipulatorAtExactSavedGroupOrigin()
        != ObjectInteractor::SavedGroupPivotResult::Failed;
}

bool Core3DViewer::rebuildTransform(const OrdinaryTransformLedger& ledger, bool committed,
                                  std::vector<Handle(AIS_Shape)>& replacements) noexcept {
    replacements.clear();
    if (![NSThread isMainThread] || !_ordinaryEditController
        || _ordinaryEditController->state() != OrdinaryEditState::RepairPending
        || !_objectInteractor || !_shapeInteractor || myDoc.IsNull() || myContext.IsNull()
        || HasActiveOperationLedger(_objectInteractor, _shapeInteractor)
        || ledger.records.empty() || ledger.records.size() > 1024) { return false; }
    AIS_ListOfInteractive previousPresentations;
    bool touchedDisplay = false;
    try {
        OCC_CATCH_SIGNALS
        const auto document = myDoc->Document();
        if (document.IsNull() || document->HasOpenCommand()) { return false; }
        // The caller has proven the closed result. Recheck every durable
        // object before touching the display; no OCAF write belongs here.
        for (const auto& record : ledger.records) {
            const auto& saved = committed ? record.candidate : record.previous;
            OcctObjectTransformState actual;
            if (!myDoc->CaptureObjectTransformStateForLabel(saved.label, actual)
                || !actual.IsEqual(saved)) { return false; }
        }
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, previousPresentations);
        touchedDisplay = true;
        clearContext();
#ifdef DEBUG
        ++_debugOrdinaryRedrawAttempts;
        if (_debugOrdinaryRedrawFailures > 0) {
            --_debugOrdinaryRedrawFailures;
            throw Standard_Failure("Injected ordinary recovery redraw failure after clear");
        }
#endif
        if (traverseDocument(document)) {
            throw Standard_Failure("Unable to rebuild ordinary edit presentations");
        }
        std::unordered_map<std::string, Handle(AIS_Shape)> displayedByIdentity;
        AIS_ListOfInteractive displayed;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
        std::size_t visited = 0;
        for (AIS_ListIteratorOfListOfInteractive it(displayed); it.More(); it.Next()) {
            if (++visited > 50000) { throw Standard_Failure("Ordinary redraw display budget exceeded"); }
            const auto presentation = Handle(AIS_Shape)::DownCast(it.Value());
            const auto label = myDoc->ShapeLabel(presentation);
            if (presentation.IsNull() || label.IsNull()) { continue; }
            const auto identity = myDoc->EntityIdentifierForLabel(label);
            if (identity.empty() || !displayedByIdentity.emplace(identity, presentation).second) {
                throw Standard_Failure("Ambiguous ordinary redraw identity");
            }
        }
        auto repaired = ledger;
        for (auto& record : repaired.records) {
            const auto& saved = committed ? record.candidate : record.previous;
#ifdef DEBUG
            if (_debugOrdinaryOwnerResolutionFailures > 0) {
                --_debugOrdinaryOwnerResolutionFailures;
                displayedByIdentity.erase(saved.entityIdentifier);
            }
#endif
            const auto found = displayedByIdentity.find(saved.entityIdentifier);
            if (found == displayedByIdentity.end()
                || !myDoc->ShapeLabel(found->second).IsEqual(saved.label)
                || !myDoc->IsPresentationEditable(found->second)) {
                throw Standard_Failure("Unable to resolve ordinary redraw object");
            }
            record.requested.presentation = found->second;
        }
        // Keep the same typed interactors and controller. Their other operation
        // ledgers were proved idle; rebuilding their callbacks would publish
        // intermediate state through the recovery barrier.
        if (!_shapeInteractor->selectionModeAuthorityIsExact()
            || _shapeInteractor->getSelectionMode() != ShapeSelectionMode::WholeShape
            || !_objectInteractor->repairOrdinaryTransformPresentation(repaired, committed)) {
            throw Standard_Failure("Unable to restore ordinary redraw selection and tool");
        }
        for (const auto& record : repaired.records) { replacements.push_back(record.requested.presentation); }
        if (_objectInteractor->positionManipulatorAtExactSavedGroupOrigin()
            == ObjectInteractor::SavedGroupPivotResult::Failed) {
            throw Standard_Failure("Unable to restore saved-group pivot");
        }
        return true;
    } catch (...) {
        replacements.clear();
    }
    if (!touchedDisplay) { return false; }
    // Restore retained handles if traversal or repair failed. The controller
    // remains RepairPending, so partial restoration is never publishable.
    try {
        clearContext();
        for (AIS_ListIteratorOfListOfInteractive it(previousPresentations); it.More(); it.Next()) {
            if (!Handle(AIS_Shape)::DownCast(it.Value()).IsNull()) {
                myContext->Display(it.Value(), Standard_False);
            }
        }
        myContext->UpdateCurrentViewer();
    } catch (...) {}
    return false;
}

bool Core3DViewer::hasUnresolvedDuplicate() const noexcept {
    try {
        return _objectInteractor == nullptr
            || _objectInteractor->hasUnresolvedDuplicate();
    } catch (...) {
        return true;
    }
}

static Quantity_NameOfColor colorNameFromString(NSString* colorStr) {
    static NSDictionary<NSString*, NSNumber*>* colorMap = @{
        @"white":   @(Quantity_NOC_WHITE),
        @"black":   @(Quantity_NOC_BLACK),
        @"red":     @(Quantity_NOC_RED),
        @"green":   @(Quantity_NOC_GREEN),
        @"blue":    @(Quantity_NOC_BLUE1),
        @"yellow":  @(Quantity_NOC_YELLOW),
        @"orange":  @(Quantity_NOC_ORANGE),
        @"brown":   @(Quantity_NOC_SADDLEBROWN),
        @"gray":    @(Quantity_NOC_GRAY80),
        @"grey":    @(Quantity_NOC_GRAY80),
        @"pink":    @(Quantity_NOC_PINK),
        @"purple":  @(Quantity_NOC_PURPLE),
        @"cyan":    @(Quantity_NOC_CYAN1),
    };
    NSNumber* val = colorMap[colorStr.lowercaseString];
    return val ? (Quantity_NameOfColor)val.intValue : Quantity_NOC_GRAY50;
}

void Core3DViewer::addPrimitivesFromJSON(NSString* json) {
    if (![json isKindOfClass:[NSString class]]
        || !canBeginCommittedEdit()
        || myDoc.IsNull()
        || myContext.IsNull()
        || _shapeInteractor == nullptr) {
        return;
    }

    NSData* data = [json dataUsingEncoding:NSUTF8StringEncoding];
    if (!data) {
        return;
    }

    NSError* error = nil;
    id rootValue = [NSJSONSerialization JSONObjectWithData:data options:0 error:&error];
    if (error || ![rootValue isKindOfClass:[NSDictionary class]]) {
        return;
    }

    NSDictionary* root = static_cast<NSDictionary*>(rootValue);
    id primitivesValue = root[@"primitives"];
    if (![primitivesValue isKindOfClass:[NSArray class]]) {
        return;
    }

    NSArray* primitives = static_cast<NSArray*>(primitivesValue);
    if (primitives.count == 0 || primitives.count > kMaximumPrimitiveCount) {
        return;
    }

    std::vector<PreparedPrimitive> preparedPrimitives;
    preparedPrimitives.reserve(primitives.count);

    try {
        OCC_CATCH_SIGNALS
        for (id primitiveValue in primitives) {
            if (![primitiveValue isKindOfClass:[NSDictionary class]]) {
                return;
            }

            NSDictionary* primitive = static_cast<NSDictionary*>(primitiveValue);
            id typeValue = primitive[@"type"];
            id positionValue = primitive[@"position"];
            id scaleValue = primitive[@"scale"];
            id rotationValue = primitive[@"rotation"];
            id colorValue = primitive[@"color"];

            if (![typeValue isKindOfClass:[NSString class]]
                || (colorValue != nil && ![colorValue isKindOfClass:[NSString class]])) {
                return;
            }

            NSString* type = static_cast<NSString*>(typeValue);
            NSString* color = colorValue == nil
                ? nil
                : static_cast<NSString*>(colorValue);

            std::array<double, 3> position;
            std::array<double, 3> scale;
            std::array<double, 3> rotation = {0.0, 0.0, 0.0};
            if (!ReadFiniteVector3(positionValue, position)
                || !ReadFiniteVector3(scaleValue, scale)
                || (rotationValue != nil && !ReadFiniteVector3(rotationValue, rotation))
                || !IsBoundedMagnitude(position, kMaximumPositionMagnitude)
                || !IsValidScale(scale)
                || !IsBoundedMagnitude(rotation, kMaximumRotationMagnitude)) {
                return;
            }

            const double px = position[0];
            const double py = position[1];
            const double pz = position[2];
            const double sx = scale[0];
            const double sy = scale[1];
            const double sz = scale[2];
            const double rx = rotation[0];
            const double ry = rotation[1];
            const double rz = rotation[2];

            // No coordinate conversion — JSON uses Z-up matching OCCT natively.
            // Create shapes with correct dimensions directly (no GTransform).
            TopoDS_Shape shape;
            if ([type isEqualToString:@"cube"]) {
                const double width = 50.0 * sx;
                const double depth = 50.0 * sy;
                const double height = 50.0 * sz;
                if (!IsValidDimension(width)
                    || !IsValidDimension(depth)
                    || !IsValidDimension(height)) {
                    return;
                }
                gp_Pnt corner(-25.0 * sx, -25.0 * sy, 0.0);
                BRepPrimAPI_MakeBox maker(corner, width, depth, height);
                shape = maker.Shape();
            } else if ([type isEqualToString:@"sphere"]) {
                const double radiusX = 25.0 * sx;
                const double radiusY = 25.0 * sy;
                const double radiusZ = 25.0 * sz;
                if (!IsValidDimension(radiusX)
                    || !IsValidDimension(radiusY)
                    || !IsValidDimension(radiusZ)) {
                    return;
                }

                // Average scale for radius, then apply non-uniform via GTransform only if needed.
                const double averageScale = (sx + sy + sz) / 3.0;
                if (std::abs(sx - sy) < 0.01 && std::abs(sy - sz) < 0.01) {
                    BRepPrimAPI_MakeSphere maker(25.0 * averageScale);
                    shape = maker.Shape();
                } else {
                    BRepPrimAPI_MakeSphere maker(25.0);
                    gp_GTrsf scaleTransform;
                    scaleTransform.SetValue(1, 1, sx);
                    scaleTransform.SetValue(2, 2, sy);
                    scaleTransform.SetValue(3, 3, sz);
                    BRepBuilderAPI_GTransform scaler(maker.Shape(), scaleTransform, true);
                    shape = scaler.Shape();
                }
            } else if ([type isEqualToString:@"cylinder"]
                       || [type isEqualToString:@"cone"]
                       || [type isEqualToString:@"torus"]) {
                const double radialScale = (sx + sy) / 2.0;
                const double radius = 25.0 * radialScale;
                const double height = 50.0 * sz;
                if (!IsValidDimension(radius) || !IsValidDimension(height)) {
                    return;
                }

                if ([type isEqualToString:@"cylinder"]) {
                    BRepPrimAPI_MakeCylinder maker(radius, height);
                    shape = maker.Shape();
                } else if ([type isEqualToString:@"cone"]) {
                    gp_Ax2 axis;
                    axis.SetLocation(gp_Pnt(0.0, 0.0, 0.0));
                    BRepPrimAPI_MakeCone maker(axis, radius, 0.0, height);
                    shape = maker.Shape();
                } else {
                    const double minorRadius = 10.0 * sz;
                    if (!IsValidDimension(minorRadius)) {
                        return;
                    }
                    gp_Ax2 axis;
                    axis.SetLocation(gp_Pnt(0.0, 0.0, 0.0));
                    BRepPrimAPI_MakeTorus maker(axis, radius, minorRadius);
                    shape = maker.Shape();
                }
            } else {
                return;
            }

            if (!IsTopologicallyValid(shape)) {
                return;
            }

            // Keep rotation/translation as the editable presentation transform, the
            // same representation produced by the on-screen gizmo. Geometry remains
            // stable in OCAF and the transform is persisted on its label.
            gp_Trsf rotationTransform;
            if (rx != 0.0 || ry != 0.0 || rz != 0.0) {
                gp_Trsf xRotation;
                gp_Trsf yRotation;
                gp_Trsf zRotation;
                if (rx != 0.0) {
                    xRotation.SetRotation(
                        gp_Ax1(gp::Origin(), gp::DX()),
                        rx * M_PI / 180.0);
                }
                if (ry != 0.0) {
                    yRotation.SetRotation(
                        gp_Ax1(gp::Origin(), gp::DY()),
                        ry * M_PI / 180.0);
                }
                if (rz != 0.0) {
                    zRotation.SetRotation(
                        gp_Ax1(gp::Origin(), gp::DZ()),
                        rz * M_PI / 180.0);
                }
                rotationTransform = zRotation * yRotation * xRotation;
            }

            gp_Trsf translationTransform;
            translationTransform.SetTranslation(gp_Vec(px, py, pz));
            const gp_Trsf objectTransform = translationTransform * rotationTransform;

            Handle(AIS_Shape) presentation = new AIS_Shape(shape);
            presentation->SetLocalTransformation(objectTransform);
            myContext->ApplyDefaultMaterial(presentation);
            Quantity_NameOfColor shapeColor = Quantity_NOC_GRAY80;
            if (color != nil) {
                shapeColor = colorNameFromString(color);
                presentation->SetColor(shapeColor);
            }

            if (presentation.IsNull()
                || presentation->Shape().IsNull()
                || !IsTopologicallyValid(presentation->Shape())) {
                return;
            }

            preparedPrimitives.push_back({
                presentation,
                Graphic3d_NameOfMaterial_ShinyPlastified,
                shapeColor,
            });
        }
    } catch (const Standard_Failure&) {
        return;
    } catch (const std::exception&) {
        return;
    } catch (...) {
        return;
    }

	// Clear outgoing selection/tool visuals before opening the batch command;
	// deselection can itself finalize an explicitly pending chamfer.
	deselectAll();

	if (publishCreatedPrimitives(preparedPrimitives) != OrdinaryEditResult::Committed) {
        return;
    }
	try {
		if (!myView.IsNull()) {
			myView->FitAll();
			myView->Redraw();
		}
	} catch (...) {
		// Framing failure does not invalidate the fully displayed, durable batch.
    }
}

void Core3DViewer::addPrimitive(PrimitiveType primitiveType) {
	if (!canBeginCommittedEdit()) { return; }

    TopoDS_Shape shape;
	Standard_Size displayedModelShapeCount = 0;
	const bool shouldFrameFirstPrimitive =
		TryCountDisplayedModelShapes(myContext, displayedModelShapeCount)
		&& displayedModelShapeCount == 0;

    switch (primitiveType) {
        case PrimitiveTypeCube:
        {
            gp_Pnt lowerLeftCornerOfBox(-25.0, -25.0, 0.0);
            BRepPrimAPI_MakeBox boxMaker(lowerLeftCornerOfBox,50, 50, 50);
            shape = boxMaker.Shape();
        }
            break;
        case PrimitiveTypeSphere:
        {
            gp_Ax2 anAxis;
            anAxis.SetLocation(gp_Pnt(0.0, 0.0, 0.0));
            BRepPrimAPI_MakeSphere sphereMaker(anAxis, 25.0);
            shape = sphereMaker.Shape();
        }
            break;
        case PrimitiveTypeCylinder:
        {
            BRepPrimAPI_MakeCylinder cylinderMaker(25.0, 50.0);
            shape = cylinderMaker.Shape();
        }
            break;
        case PrimitiveTypeCone:
        {
            gp_Ax2 anAxis;
            anAxis.SetLocation(gp_Pnt(0.0, 0.0, 0.0));
            BRepPrimAPI_MakeCone coneMaker(anAxis, 25.0, 0.0, 50.0);
            shape = coneMaker.Shape();
        }
            break;
        case PrimitiveTypeTorus:
        {
            gp_Ax2 anAxis;
            anAxis.SetLocation(gp_Pnt(0.0, 0.0, 0.0));
            BRepPrimAPI_MakeTorus torusMaker(anAxis, 25.0, 10.0);
            shape = torusMaker.Shape();
        }
            break;
        default:
            break;
    }

    Handle(AIS_Shape) aShapePrs = new AIS_Shape (shape);
    myContext->ApplyDefaultMaterial(aShapePrs);
    Quantity_Color qc;
    aShapePrs->Color(qc);
	if (!IsTopologicallyValid(shape)) {
		return;
	}
	deselectAll();
	const std::vector<PreparedPrimitive> primitive = {{
		aShapePrs,
		Graphic3d_NameOfMaterial_ShinyPlastified,
		qc.Name(),
	}};
	if (publishCreatedPrimitives(primitive) != OrdinaryEditResult::Committed) {
        return;
    }
	if (shouldFrameFirstPrimitive && !myView.IsNull()) {
		// Frame only the first real model object. The V3d construction grid is not
		// an AIS shape, and the manipulator does not downcast to AIS_Shape, so
		// neither affects the pre-insert count. Later inserts must preserve the
		// camera the user established while modeling.
		Bnd_Box aFrameBox;
		BRepBndLib::AddOptimal(shape, aFrameBox, Standard_False, Standard_False);
		if (!aFrameBox.IsVoid()) {
			// Frame the analytic shape independently of cached tessellation.
			// Leave room for the surrounding touch controls in either orientation;
			// later additions preserve the camera established by the user.
			myView->FitAll(aFrameBox, 0.45, Standard_False);
			myView->ZFitAll();
		}
	}
	_objectInteractor->attachManipulatorToSelection();
    getObjectInteractor()->SelectAndAttachManipulator(aShapePrs);
}

bool Core3DViewer::traverseLabel (const Handle(TDocStd_Document)& theDoc,
                                  const TDF_Label& theLabel,
                                  const TCollection_AsciiString& theNamePrefix,
                                  const TopLoc_Location& theLoc,
                                  MapOfPrsForShapes& theMapOfShapes)
{
    if (theDoc.IsNull() || theLabel.IsNull()) {
        return true;
    }
    (void)theNamePrefix;
    (void)theLoc;
    (void)theMapOfShapes;
    // Common display admission prepares hidden geometry without publishing it.
    XCAFPrs_Style aDefStyle;
    aDefStyle.SetColorSurf(Quantity_NOC_GRAY80);
    aDefStyle.SetColorCurv(Quantity_NOC_GRAY80);
    return !displayWithChildren(theDoc, theLabel, aDefStyle);
}

bool Core3DViewer::traverseDocument (const Handle(TDocStd_Document)& theDoc)
{
    TDF_LabelSequence aLabels;
    XCAFDoc_DocumentTool::ShapeTool (theDoc->Main())->GetFreeShapes (aLabels);
    if (aLabels.IsEmpty()) {
        return false;
    }
    XCAFPrs_Style aDefStyle;
    aDefStyle.SetColorSurf(Quantity_NOC_GRAY80);
    aDefStyle.SetColorCurv(Quantity_NOC_GRAY80);
    // Keep suppressed roots in the bounded load walk. The common leaf
    // admission projects suppression and prepares their hidden BRep caches.
    return !displayWithChildren(theDoc, aLabels, aDefStyle);
}

#if DEBUG
namespace {
std::uint64_t RetainedSuppressionColdReopenProbe() {
    using namespace core3d;
    using namespace core3d::pattern_owner;
    struct Documents final {
        Handle(TDocStd_Application) writer = new TDocStd_Application();
        Handle(TDocStd_Application) reader = new TDocStd_Application();
        Handle(TDocStd_Document) authored;
        Handle(TDocStd_Document) reopened;
        ~Documents() noexcept {
            try { if (!authored.IsNull()) writer->Close(authored); } catch (...) {}
            try { if (!reopened.IsNull()) reader->Close(reopened); } catch (...) {}
        }
    };
    const char* phase = "enter";
    NSLog(@"[RetainedSuppressionColdReopenProbe] phase=enter scenario=0");
    try {
        phase = "construct-documents";
        Documents documents;
        phase = "writer-format";
        Core3DDefineSafeBinXCAFFormat(documents.writer);
        phase = "reader-format";
        Core3DDefineSafeBinXCAFFormat(documents.reader);
        phase = "new-document";
        documents.writer->NewDocument(
            TCollection_ExtendedString("BinXCAF"), documents.authored);
        if (documents.authored.IsNull()) {
            NSLog(@"[RetainedSuppressionColdReopenProbe] phase=new-document exit=return-zero authoredNull=1");
            return 0;
        }
        phase = "set-units";
        XCAFDoc_DocumentTool::SetLengthUnit(documents.authored, 0.001);
        phase = "shape-tool";
        const auto shapes =
            XCAFDoc_DocumentTool::ShapeTool(documents.authored->Main());
        phase = "color-tool";
        const auto colors =
            XCAFDoc_DocumentTool::ColorTool(documents.authored->Main());
        phase = "layer-tool";
        const auto layers =
            XCAFDoc_DocumentTool::LayerTool(documents.authored->Main());
        if (shapes.IsNull() || colors.IsNull() || layers.IsNull()) {
            NSLog(@"[RetainedSuppressionColdReopenProbe] phase=tools exit=return-zero shapesNull=%d colorsNull=%d layersNull=%d",
                  int(shapes.IsNull()), int(colors.IsNull()), int(layers.IsNull()));
            return 0;
        }

        const auto id = [](std::uint8_t value) {
            retained_recipe::UUID result{};
            result.back() = value;
            return result;
        };
        const auto setEntity = [](const TDF_Label& label,
                                  const retained_recipe::UUID& value) {
            static const Standard_GUID identifier(
                "0074F7C2-9EAA-4F89-B2DE-8716E155FF62");
            TDataStd_AsciiString::Set(label, identifier,
                TCollection_AsciiString(retained_solid::UUIDText(value).c_str()));
        };
        phase = "source-shape";
        const TDF_Label source = shapes->AddShape(
            BRepPrimAPI_MakeBox(2.0, 2.0, 2.0).Shape(), Standard_False);
        phase = "suppressed-shape";
        const TDF_Label suppressed = shapes->AddShape(
            BRepPrimAPI_MakeBox(3.0, 3.0, 3.0).Shape(), Standard_False);
        phase = "labels";
        if (source.IsNull() || suppressed.IsNull()) {
            NSLog(@"[RetainedSuppressionColdReopenProbe] phase=labels exit=return-zero sourceNull=%d suppressedNull=%d",
                  int(source.IsNull()), int(suppressed.IsNull()));
            return 0;
        }
        phase = "source-entity";
        setEntity(source, id(2));
        phase = "suppressed-entity";
        setEntity(suppressed, id(6));

        phase = "hidden-layer";
        const TDF_Label hiddenLayer = layers->AddLayer(
            TCollection_ExtendedString("Retained suppression authored layer"));
        if (hiddenLayer.IsNull()) {
            NSLog(@"[RetainedSuppressionColdReopenProbe] phase=hidden-layer exit=return-zero hiddenLayerNull=1");
            return 0;
        }
        phase = "authored-hidden";
        colors->SetVisibility(suppressed, Standard_False);
        phase = "layer-hidden";
        layers->SetVisibility(hiddenLayer, Standard_False);
        phase = "assign-layer";
        layers->SetLayer(suppressed, hiddenLayer, Standard_False);

        phase = "definition";
        pattern::Definition definition;
        definition.owner = {id(1), id(2), id(3)};
        definition.feature = id(4);
        definition.source = {id(1), id(2), id(3), id(5)};
        definition.issuance.nextLocalID = 3;
        definition.members = {
            {id(2), 1, {0, 0}, pattern::MemberState::Active},
            {id(6), 2, {0, 1}, pattern::MemberState::Suppressed},
        };
        const auto diagnoseCommandFailure = [&](const char* failedPhase) {
            NSLog(@"[RetainedSuppressionColdReopenProbe] phase=%s exit=return-zero undoLimit=%d open=%d undos=%d",
                  failedPhase,
                  int(documents.authored->GetUndoLimit()),
                  int(documents.authored->HasOpenCommand()),
                  int(documents.authored->GetAvailableUndos()));
        };
        // Stage requires a real OCAF command; a fresh document has undo disabled.
        phase = "undo-limit";
        documents.authored->SetUndoLimit(1);
        phase = "command-open";
        documents.authored->NewCommand();
        if (!documents.authored->HasOpenCommand()) {
            diagnoseCommandFailure("command-open");
            return 0;
        }
        phase = "stage";
        pattern::Record staged;
        if (!pattern::Stage(documents.authored, definition, staged)) {
            diagnoseCommandFailure("stage");
            return 0;
        }
        phase = "commit";
        if (!documents.authored->CommitCommand()) {
            diagnoseCommandFailure("commit");
            return 0;
        }
        phase = "commit-state";
        if (documents.authored->HasOpenCommand()
            || documents.authored->GetAvailableUndos() != 1) {
            diagnoseCommandFailure("commit-state");
            return 0;
        }
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=commit-state exit=continue undoLimit=%d open=%d undos=%d",
              int(documents.authored->GetUndoLimit()),
              int(documents.authored->HasOpenCommand()),
              int(documents.authored->GetAvailableUndos()));

        std::ostringstream output(std::ios::binary | std::ios::out);
        phase = "save";
        const auto saveStatus = documents.writer->SaveAs(
            documents.authored, output);
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=save status=%d",
              int(saveStatus));
        if (saveStatus != PCDM_SS_OK) {
            NSLog(@"[RetainedSuppressionColdReopenProbe] phase=save exit=return-zero status=%d",
                  int(saveStatus));
            return 0;
        }
        phase = "serialized-bytes";
        const std::string bytes = output.str();
        const bool serializedEmpty = bytes.empty();
        const bool serializedOversized =
            bytes.size() > pattern::MaximumDocumentPatternBytes;
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=serialized-bytes bytes=%llu bound=%llu empty=%d oversized=%d",
              static_cast<unsigned long long>(bytes.size()),
              static_cast<unsigned long long>(pattern::MaximumDocumentPatternBytes),
              int(serializedEmpty), int(serializedOversized));
        if (serializedEmpty || serializedOversized) {
            NSLog(@"[RetainedSuppressionColdReopenProbe] phase=serialized-bytes exit=return-zero bytes=%llu bound=%llu empty=%d oversized=%d",
                  static_cast<unsigned long long>(bytes.size()),
                  static_cast<unsigned long long>(pattern::MaximumDocumentPatternBytes),
                  int(serializedEmpty), int(serializedOversized));
            return 0;
        }
        phase = "close-writer";
        documents.writer->Close(documents.authored);
        phase = "nullify-writer";
        documents.authored.Nullify();
        phase = "begin-safe-read";
        Core3DBeginSafeBinaryRead();
        phase = "input-stream";
        std::istringstream input(bytes, std::ios::binary | std::ios::in);
        phase = "open";
        const auto openStatus = documents.reader->Open(
            input, documents.reopened);
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=open status=%d bytes=%llu reopenedNull=%d",
              int(openStatus), static_cast<unsigned long long>(bytes.size()),
              int(documents.reopened.IsNull()));
        if (openStatus != PCDM_RS_OK) {
            NSLog(@"[RetainedSuppressionColdReopenProbe] phase=open exit=return-zero reason=status status=%d bytes=%llu reopenedNull=%d",
                  int(openStatus), static_cast<unsigned long long>(bytes.size()),
                  int(documents.reopened.IsNull()));
            return 0;
        }
        phase = "safe-read-rejected";
        const bool safeReadRejected = Core3DSafeBinaryReadWasRejected();
        if (safeReadRejected) {
            NSLog(@"[RetainedSuppressionColdReopenProbe] phase=safe-read-rejected exit=return-zero rejected=1");
            return 0;
        }
        phase = "reopened-document";
        if (documents.reopened.IsNull()) {
            NSLog(@"[RetainedSuppressionColdReopenProbe] phase=reopened-document exit=return-zero reopenedNull=1");
            return 0;
        }

        phase = "retained-index";
        RetainedPresentationIndex retained;
        const bool retainedIndexCaptured =
            CaptureRetainedPresentationIndex(documents.reopened, retained);
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=retained-index success=%d members=%llu",
              int(retainedIndexCaptured),
              static_cast<unsigned long long>(retained.members.size()));
        if (!retainedIndexCaptured) {
            NSLog(@"[RetainedSuppressionColdReopenProbe] phase=retained-index exit=return-zero success=0 members=%llu",
                  static_cast<unsigned long long>(retained.members.size()));
            return 0;
        }
        phase = "free-labels";
        TDF_LabelSequence freeLabels;
        XCAFDoc_DocumentTool::ShapeTool(
            documents.reopened->Main())->GetFreeShapes(freeLabels);
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=free-labels count=%d",
              int(freeLabels.Length()));
        phase = "find-suppressed";
        TDF_Label reopenedSuppressed;
        static const Standard_GUID identifier(
            "0074F7C2-9EAA-4F89-B2DE-8716E155FF62");
        for (TDF_LabelSequence::Iterator label(freeLabels);
             label.More(); label.Next()) {
            retained_recipe::UUID entity{};
            if (retained_solid::ReadUUID(label.Value(), identifier, entity)
                && entity == id(6)) reopenedSuppressed = label.Value();
        }
        if (reopenedSuppressed.IsNull()) {
            NSLog(@"[RetainedSuppressionColdReopenProbe] phase=find-suppressed exit=return-zero freeLabels=%d found=0",
                  int(freeLabels.Length()));
            return 0;
        }
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=find-suppressed freeLabels=%d found=1",
              int(freeLabels.Length()));
        phase = "reopened-layer-tool";
        const auto reopenedLayers =
            XCAFDoc_DocumentTool::LayerTool(documents.reopened->Main());
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=reopened-layer-tool null=%d",
              int(reopenedLayers.IsNull()));
        TDF_LabelSequence assignedLayers;
        phase = "authored-hidden-read";
        const bool authoredHidden =
            !XCAFDoc_ColorTool::IsVisible(reopenedSuppressed);
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=authored-hidden-read authoredHidden=%d",
              int(authoredHidden));
        phase = "assigned-layers-read";
        const bool assignedLayersRead = !reopenedLayers.IsNull()
            && reopenedLayers->GetLayers(reopenedSuppressed, assignedLayers);
        const bool hasHiddenLayer = assignedLayersRead
            && assignedLayers.Length() == 1
            && !reopenedLayers->IsVisible(assignedLayers.First());
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=assigned-layers-read toolNull=%d read=%d count=%d hasHiddenLayer=%d",
              int(reopenedLayers.IsNull()), int(assignedLayersRead),
              int(assignedLayers.Length()), int(hasHiddenLayer));
        phase = "retained-state";
        const auto state = retained.StateFor(id(6));
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=retained-state state=%d",
              int(state));
        std::uint64_t bits =
            state == RetainedPresentationState::Suppressed ? 1ULL : 0ULL;
        if (authoredHidden) bits |= 2ULL;
        if (hasHiddenLayer) bits |= 4ULL;
        if (!EffectiveVisibility(!authoredHidden, !hasHiddenLayer, state))
            bits |= 8ULL;
        pattern::Record readback;
        bool readbackEvaluated = false;
        bool readbackSucceeded = false;
        phase = "observations";
        if (freeLabels.Length() == 2) {
            readbackEvaluated = true;
            readbackSucceeded = pattern::ReadFeature(
                documents.reopened, id(4), readback);
            if (readbackSucceeded
                && readback.definition.members.size() == 2
                && readback.definition.members[1].localID == 2
                && readback.definition.members[1].identity == id(6)) bits |= 16ULL;
        }
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=observations mask=%llu state=%d authoredHidden=%d hasHiddenLayer=%d freeLabels=%d assignedLayers=%d readbackEvaluated=%d readbackSucceeded=%d",
              static_cast<unsigned long long>(bits), int(state),
              int(authoredHidden), int(hasHiddenLayer), int(freeLabels.Length()),
              int(assignedLayers.Length()), int(readbackEvaluated),
              int(readbackSucceeded));
        return bits;
    } catch (const Standard_Failure&) {
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=%s exit=exception kind=Standard_Failure return=0",
              phase);
        throw;
    } catch (const std::exception&) {
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=%s exit=exception kind=std::exception return=0",
              phase);
        throw;
    } catch (...) {
        NSLog(@"[RetainedSuppressionColdReopenProbe] phase=%s exit=exception kind=unknown return=0",
              phase);
        throw;
    }
}
} // namespace

extern "C" std::uint64_t
Core3DDebugRetainedSuppressionPresentationProbe(std::int32_t scenario) noexcept {
    using namespace core3d::pattern_owner;
    try {
        if (scenario == 0) {
            return RetainedSuppressionColdReopenProbe();
        }
        if (scenario == 1) {
            const retained_recipe::UUID first{{1}}, second{{2}};
            const std::uint64_t firstLocal = 7, secondLocal = 11;
            RetainedPresentationIndex index;
            index.members.emplace(first, RetainedPresentationState::Active);
            index.members.emplace(second, RetainedPresentationState::Suppressed);
            std::uint64_t bits = index.members.size() == 2 ? 1ULL : 0ULL;
            if (index.StateFor(second) == RetainedPresentationState::Suppressed) bits |= 2ULL;
            if (index.StateFor(first) == RetainedPresentationState::Active) bits |= 4ULL;
            if (firstLocal == 7 && secondLocal == 11) bits |= 8ULL;
            if (index.members.count(first) && index.members.count(second)) bits |= 16ULL;
            return bits;
        }
        if (scenario == 2) {
            const retained_recipe::UUID unrelated{{9}};
            RetainedPresentationIndex index;
            std::uint64_t bits = index.StateFor(unrelated)
                == RetainedPresentationState::Unretained ? 1ULL : 0ULL;
            if (EffectiveVisibility(true, true, index.StateFor(unrelated))) bits |= 2ULL;
            if (!EffectiveVisibility(false, true, index.StateFor(unrelated))) bits |= 4ULL;
            if (!EffectiveVisibility(true, false, index.StateFor(unrelated))) bits |= 8ULL;
            if (index.members.empty()) bits |= 16ULL;
            return bits;
        }
    } catch (...) {}
    return 0;
}
#endif

#ifdef DEBUG
void Core3DViewer::DebugResetProjectTopologyValidationCounters() const
{
    gBoundedProjectTopologyValidationCount.store(
        0, std::memory_order_relaxed);
    gGeometricBRepValidationCount.store(0, std::memory_order_relaxed);
}

Standard_Size Core3DViewer::DebugBoundedProjectTopologyValidationCount() const
{
    return gBoundedProjectTopologyValidationCount.load(
        std::memory_order_relaxed);
}

Standard_Size Core3DViewer::DebugGeometricBRepValidationCount() const
{
    return gGeometricBRepValidationCount.load(std::memory_order_relaxed);
}

void Core3DViewer::DebugSetSceneSnapshotTriangulationFailure(
    const scene::OcctSceneSnapshotBuilder::DebugTriangulationFailure
        theFailure) noexcept
{
    _sceneSnapshotBuilder.DebugSetTriangulationFailure(theFailure);
}

void Core3DViewer::DebugResetSceneSnapshotMesherInvocationCount() noexcept
{
    _sceneSnapshotBuilder.DebugResetMesherInvocationCount();
}

std::uint64_t
Core3DViewer::DebugSceneSnapshotMesherInvocationCount() const noexcept
{
    return _sceneSnapshotBuilder.DebugMesherInvocationCount();
}

std::uint64_t Core3DViewer::DebugPublishedDocumentGeneration() const noexcept
{
    return _sceneSnapshotBuilder.DebugPublishedDocumentGeneration(myDoc);
}

std::uint64_t Core3DViewer::DebugPublishedModelRevision() const noexcept
{
    return _sceneSnapshotBuilder.DebugPublishedModelRevision(myDoc);
}

TransformInspectorMeasurementDebugState
Core3DViewer::DebugTransformInspectorMeasurementState() const noexcept
{
    return _transformInspectorMeasurementController == nullptr
        ? TransformInspectorMeasurementDebugState()
        : _transformInspectorMeasurementController->debugState();
}

void Core3DViewer::DebugSetTransformInspectorWorkerBlocked(
    const Standard_Boolean theBlocked) noexcept
{
    if (_transformInspectorMeasurementController != nullptr) {
        _transformInspectorMeasurementController
            ->debugSetWorkerBlocked(theBlocked);
    }
}

void Core3DViewer::
DebugSetTransformInspectorForcedMeasurementFailure(
    const Standard_Boolean theFailure) noexcept
{
    if (_transformInspectorMeasurementController != nullptr) {
        _transformInspectorMeasurementController
            ->debugSetForcedBoundsFailure(theFailure);
    }
}

void Core3DViewer::
DebugSetMaximumTransformInspectorBRepTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    if (_transformInspectorMeasurementController != nullptr) {
        _transformInspectorMeasurementController
            ->debugSetMaximumBRepTopologyNodes(theLimit);
    }
}

void Core3DViewer::
DebugSetMaximumTransformInspectorTriangleMeshSweepNodes(
    const Standard_Size theLimit) noexcept
{
    if (_transformInspectorMeasurementController != nullptr) {
        _transformInspectorMeasurementController
            ->debugSetMaximumTriangleMeshSweepNodes(theLimit);
    }
}

void Core3DViewer::DebugSetTransformInspectorMeshSweepWatchdog(
    const Standard_Real theDeadlineMilliseconds,
    const Standard_Size thePollNodes) noexcept
{
    if (_transformInspectorMeasurementController != nullptr) {
        _transformInspectorMeasurementController->debugSetMeshSweepWatchdog(
            theDeadlineMilliseconds, thePollNodes);
    }
}

void Core3DViewer::DebugSetTransformInspectorPositionCommitMode(
    const Standard_Integer theMode) noexcept
{
    if (_transformInspectorMeasurementController != nullptr) {
        _transformInspectorMeasurementController->debugSetPositionCommitMode(
            theMode);
    }
}

void Core3DViewer::
DebugSetTransformInspectorPositionPublicationFallbackMode(
    const Standard_Integer theMode) noexcept
{
    _debugTransformInspectorPositionPublicationFallbackMode =
        theMode >= 0 && theMode <= 3 ? theMode : 0;
    _debugForceNextTransformInspectorRedrawFailure = Standard_False;
}
#endif

bool Core3DViewer::restoreDocumentReplacement(bool afterImportFailure) noexcept {
    if (![NSThread isMainThread] || !_documentReplacementWork || myDoc.IsNull() || myContext.IsNull()) return false;
    const auto work = _documentReplacementWork;
    if (!work->reservation || work->previous.IsNull()) return false;
    using Phase = DocumentReplacementWork::Phase;
    if (work->phase == Phase::Restoring || (work->phase == Phase::Installing && !afterImportFailure)) return false;
    work->phase = Phase::Restoring;
    myDoc->ChangeDocument() = work->previous;
    bool restored = false;
    for (int route = 0; route < 2 && !restored; ++route) {
        try {
#if DEBUG
            if (_debugDocumentRestorationFailures > 0) {
                --_debugDocumentRestorationFailures;
                throw Standard_Failure("Injected document presentation restoration failure");
            }
#endif
            clearContext();
            if (route == 0) {
                if (traverseDocument(work->previous)) throw Standard_Failure("Prior document traversal failed");
            } else {
                for (AIS_ListIteratorOfListOfInteractive item(work->presentations); item.More(); item.Next()) {
                    if (!Handle(AIS_Shape)::DownCast(item.Value()).IsNull()) myContext->Display(item.Value(), Standard_False);
                }
            }
            if (!recreateInteractors(work->manipulator, work->selection)) throw Standard_Failure("Prior interactors unavailable");
            myContext->UpdateCurrentViewer();
            restored = true;
        } catch (...) { /* Keep exact documents, presentations and reservation for a later retry. */ }
    }
    const auto end = myDoc->EndNativeReplacement(*work->reservation, false, restored);
    if (!restored || end != authority::ReplacementEnd::Restored) {
        work->phase = Phase::RestorePending;
        return false;
    }
    CloseDocumentNoThrow(work->application, work->candidate);
    _documentReplacementWork.reset();
    return true;
}

struct QueuedAssetLoadWork {
    Handle(OcctDocument) owner;
    Handle(TDocStd_Document) document;
    std::optional<authority::QueuedLoadReservation> reservation;
    bool promoted = false;
};

std::shared_ptr<QueuedAssetLoadWork> Core3DViewer::beginQueuedAssetLoad() noexcept {
    if (![NSThread isMainThread] || !canBeginCommittedEdit() || myContext.IsNull()
        || _objectInteractor == nullptr || _objectInteractor->isManipulatorGestureActive()) return {};
    try {
        auto work = std::make_shared<QueuedAssetLoadWork>();
        work->owner = myDoc; work->document = myDoc->Document();
        work->reservation = myDoc->BeginNativeQueuedLoad();
        if (!work->reservation) return {};
        _queuedAssetLoadWork = work;
        return work;
    } catch (...) { return {}; }
}

bool Core3DViewer::ownsQueuedAssetLoad(const std::shared_ptr<QueuedAssetLoadWork>& work) const noexcept {
    return [NSThread isMainThread] && work && _queuedAssetLoadWork == work;
}

bool Core3DViewer::canAdoptQueuedAssetLoad(const std::shared_ptr<QueuedAssetLoadWork>& work) noexcept {
    if (!ownsQueuedAssetLoad(work) || work->promoted || !work->reservation
        || _objectInteractor == nullptr || _shapeInteractor == nullptr || myContext.IsNull()
        || myDoc.IsNull() || myDoc != work->owner
        || hasUnresolvedOrdinaryEditExcludingQueuedLoad()
        || HasActiveOperationLedger(_objectInteractor, _shapeInteractor)
        || _objectInteractor->isManipulatorGestureActive()) return false;
    try {
        return !myDoc->Document().IsNull() && myDoc->Document() == work->document
            && !myDoc->Document()->HasOpenCommand()
            && myDoc->OwnsNativeQueuedLoad(*work->reservation);
    } catch (...) { return false; }
}

bool Core3DViewer::finishQueuedAssetLoadPrivateWork(
    const std::shared_ptr<QueuedAssetLoadWork>& work, bool privateWorkSettled) noexcept {
    if (!ownsQueuedAssetLoad(work) || !privateWorkSettled || myDoc.IsNull()
        || myDoc != work->owner || !work->reservation) return false;
    // Promoted ownership now belongs to _documentReplacementWork, including
    // unknown restoration. Releasing staged input cannot end that reservation.
    if (!work->promoted && myDoc->EndNativeQueuedLoadPrivateWork(*work->reservation, true)
        != authority::QueuedLoadEnd::PrivateWorkSettled) return false;
    _queuedAssetLoadWork.reset();
    return true;
}

AssetImportResult Core3DViewer::ImportCbf(const std::string& theFilename) {
    return ImportCbf(theFilename, {});
}

AssetImportResult Core3DViewer::ImportCbf(const std::string& theFilename,
    const std::shared_ptr<QueuedAssetLoadWork>& queuedWork) {
    if (![NSThread isMainThread] || myContext.IsNull() || myDoc.IsNull()) return AssetImportResult::InternalFailure;
    if ((_queuedAssetLoadWork && !queuedWork)
        || (queuedWork && !canAdoptQueuedAssetLoad(queuedWork))
        || hasUnresolvedOrdinaryEditExcludingQueuedLoad()
        || HasActiveOperationLedger(_objectInteractor, _shapeInteractor)) {
        // Replacement is a hard document boundary. Never cancel or recreate
        // an operation here: its controller may own previews, an open command,
        // or an exactly-once reconciliation token that the enum cannot encode.
        return AssetImportResult::Busy;
    }

    Handle(TDocStd_Document) previous = myDoc->Document();
    if (previous.IsNull()) {
        return AssetImportResult::InternalFailure;
    }
    if (previous->HasOpenCommand()) {
        return AssetImportResult::Busy;
    }

    const AssetImportResult validationResult = ValidateCbf(theFilename);
    if (validationResult != AssetImportResult::Success) {
        return validationResult;
    }

    auto app = Handle(TDocStd_Application)::DownCast(previous->Application());
    if (app.IsNull()) {
        return AssetImportResult::InternalFailure;
    }

    const PrimitiveManipulatorType previousManipulatorType =
        StatelessManipulatorTypeOrNone(
            _objectInteractor == nullptr
                ? PrimitiveManipulatorType::PrimitiveGizmoTypeNone
                : _objectInteractor->getManipulatorType());
    const ShapeSelectionMode previousSelectionMode = _shapeInteractor == nullptr
        ? ShapeSelectionMode::WholeShape
        : _shapeInteractor->getSelectionMode();
	AIS_ListOfInteractive previousPresentations;
	myContext->DisplayedObjects(AIS_KOI_Shape, -1, previousPresentations);

    Handle(TDocStd_Document) candidate;
    try {
        OCC_CATCH_SIGNALS
        Core3DBeginSafeBinaryRead();
        PCDM_ReaderStatus status = app->Open(theFilename.c_str(), candidate);
        const Standard_Boolean wasRejected =
            Core3DSafeBinaryReadWasRejected();
        if (wasRejected || status != PCDM_RS_OK || candidate.IsNull()) {
            if (wasRejected) {
                CloseDocumentNoThrow(app, candidate);
                return AssetImportResult::InvalidData;
            }
            AssetImportResult result = status == PCDM_RS_OK
                ? AssetImportResult::InvalidData
                : ImportResultForReaderStatus(status);
            // Isolated validation already proved these bytes readable. A
            // conflicting result from the live application is an engine/state
            // failure, never evidence that the committed revision is corrupt.
            if (result == AssetImportResult::InvalidData) {
                result = AssetImportResult::InternalFailure;
            }
            CloseDocumentNoThrow(app, candidate);
            return result;
        }
    } catch (const Standard_Failure& failure) {
        std::cout << "Load CBF failure: " << failure.GetMessageString() << std::endl;
        CloseDocumentNoThrow(app, candidate);
        return AssetImportResult::InternalFailure;
    } catch (const std::exception& exception) {
        std::cout << "Load CBF exception: " << exception.what() << std::endl;
        CloseDocumentNoThrow(app, candidate);
        return AssetImportResult::InternalFailure;
    } catch (...) {
        CloseDocumentNoThrow(app, candidate);
        return AssetImportResult::InternalFailure;
    }

	try {
		OCC_CATCH_SIGNALS
		TDF_LabelMap candidateDefinitions;
		if (!HasValidOptionalLengthUnit(candidate)
			|| !ValidateShapeTree(
                candidate,
                candidateDefinitions,
                myMaximumProjectTopologyValidationNodes)
			|| !ValidateVisualMaterials(candidate, candidateDefinitions)
            || !myDoc->ValidateGeometryRepresentations(candidate)) {
			CloseDocumentNoThrow(app, candidate);
			return AssetImportResult::InvalidData;
		}
		// CBF files created before persistent scene identity need a one-time
		// migration. Do this while the candidate is isolated, before it becomes
		// the editable document and before normal undo history is enabled.
#if DEBUG
		// R179/D249 passive census of the isolated candidate, immediately
		// before legacy migration. Diagnostics only.
		try {
			const std::string census =
				core3d::general_loft::persistence::debug::Census(candidate, "");
			std::fprintf(stderr,
				"R179_LOFT_CENSUS phase=import-pre-migration %s\n",
				census.c_str());
		} catch (...) {}
#endif
		if (!myDoc->MigrateLegacyIdentifiers(candidate)) {
			CloseDocumentNoThrow(app, candidate);
			return AssetImportResult::InternalFailure;
		}
#if DEBUG
		// R179/D249 passive census of the isolated candidate, immediately
		// after legacy migration. Diagnostics only.
		try {
			const std::string census =
				core3d::general_loft::persistence::debug::Census(candidate, "");
			std::fprintf(stderr,
				"R179_LOFT_CENSUS phase=import-post-migration %s\n",
				census.c_str());
		} catch (...) {}
#endif
		candidate->SetUndoLimit(OcctDocument::kNativeSessionUndoLimit);

#if DEBUG
        if (_debugFailNextDocumentPreparation) {
            _debugFailNextDocumentPreparation = false;
            throw Standard_Failure("Injected failure during private candidate preparation");
        }
#endif
    } catch (...) {
        // No live presentation or document assignment has occurred.
        CloseDocumentNoThrow(app, candidate);
        return AssetImportResult::InternalFailure;
    }

    try {
        auto work = std::make_shared<DocumentReplacementWork>();
        work->application = app; work->previous = previous; work->candidate = candidate;
        work->presentations = previousPresentations;
        work->manipulator = previousManipulatorType; work->selection = previousSelectionMode;
        // The matching owner transfers atomically after private candidate
        // validation. No permission can be recaptured between End and Begin.
        work->reservation = queuedWork
            ? myDoc->PromoteNativeQueuedLoad(*queuedWork->reservation, canAdoptQueuedAssetLoad(queuedWork))
            : myDoc->BeginNativeReplacement();
        if (!work->reservation) {
            CloseDocumentNoThrow(app, candidate);
            return AssetImportResult::Busy;
        }
        // Install the retained owner before the first context mutation. Assignment
        // of shared_ptr is noexcept; every subsequent failure goes through recovery.
        _documentReplacementWork = std::move(work);
        if (queuedWork) queuedWork->promoted = true;
    } catch (...) {
        CloseDocumentNoThrow(app, candidate);
        return AssetImportResult::InternalFailure;
    }

    try {
        OCC_CATCH_SIGNALS

        // Keep the previous OCAF document alive until the candidate has been
        // fully traversed and displayed. Only the presentation is temporary.
        _meshVertexEditWork.reset();
        _meshRegionExtrudeWork.reset();
        _meshRegionInsetWork.reset();
        clearContext();
        if (traverseDocument(candidate)) {
			(void)restoreDocumentReplacement(true);
			// Both isolated and live document validation have succeeded. This
			// failure belongs to presentation: display budgets, AIS allocation,
			// or graphics-driver errors can all make traversal fail. Reporting
			// InvalidData here would authorize the app to promote an older saved
			// revision and delete this valid one. Preserve it for a later retry.
			return AssetImportResult::InternalFailure;
		}
        myContext->UpdateCurrentViewer();

        myDoc->ChangeDocument() = candidate;
#if DEBUG
        if (_debugFailNextDocumentAdoption) {
            _debugFailNextDocumentAdoption = false;
            throw Standard_Failure("Injected failure after provisional document assignment");
        }
#endif
        if (!recreateFreshInteractorsForDocumentReplacement()) {
            throw Standard_Failure(
                "Unable to create fresh document interactors");
        }
        myContext->UpdateCurrentViewer();
    } catch (const Standard_Failure& failure) {
        std::cout << "Display CBF failure: " << failure.GetMessageString() << std::endl;
		(void)restoreDocumentReplacement(true);
        return AssetImportResult::InternalFailure;
    } catch (...) {
		(void)restoreDocumentReplacement(true);
        return AssetImportResult::InternalFailure;
    }

    _meshVertexEditWork.reset();
    _meshRegionExtrudeWork.reset();
    _meshRegionInsetWork.reset();
    if (myDoc->EndNativeReplacement(*_documentReplacementWork->reservation, true, true)
        != authority::ReplacementEnd::Adopted) {
        (void)restoreDocumentReplacement(true);
        return AssetImportResult::InternalFailure;
    }
    _documentReplacementWork.reset();
#if DEBUG
    myDoc->DebugObserveSuccessfulDocumentAdoption();
#endif
    CloseDocumentNoThrow(app, previous);
    return AssetImportResult::Success;
}

AssetImportResult Core3DViewer::ValidateCbf(const std::string &theFilename) const {
    if (!IsProjectFileWithinSizeLimit(theFilename)) {
        return AssetImportResult::InvalidData;
    }

    Handle(TDocStd_Application) validationApplication;
    Handle(TDocStd_Document) candidate;
    try {
        OCC_CATCH_SIGNALS
        validationApplication = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(validationApplication);

        Core3DBeginSafeBinaryRead();
        const PCDM_ReaderStatus status =
            validationApplication->Open(theFilename.c_str(), candidate);
        const Standard_Boolean wasRejected =
            Core3DSafeBinaryReadWasRejected();
        if (wasRejected || status != PCDM_RS_OK || candidate.IsNull()) {
            if (wasRejected) {
                CloseDocumentNoThrow(validationApplication, candidate);
                return AssetImportResult::InvalidData;
            }
            const AssetImportResult result = status == PCDM_RS_OK
                ? AssetImportResult::InvalidData
                : ImportResultForReaderStatus(status);
            CloseDocumentNoThrow(validationApplication, candidate);
            return result;
        }

#if DEBUG
        // R179/D249 passive census of the candidate after the isolated Open,
        // before it is closed. Diagnostics only.
        try {
            const std::string census =
                core3d::general_loft::persistence::debug::Census(candidate, "");
            std::fprintf(stderr, "R179_LOFT_CENSUS phase=validate-cbf %s\n",
                         census.c_str());
        } catch (...) {}
#endif
        TDF_LabelMap activeDefinitionLabels;
        const bool isValid = HasValidOptionalLengthUnit(candidate)
            && ValidateShapeTree(
                candidate,
                activeDefinitionLabels,
                myMaximumProjectTopologyValidationNodes)
            && ValidateVisualMaterials(
                candidate, activeDefinitionLabels)
            && myDoc->ValidateGeometryRepresentations(candidate);
        CloseDocumentNoThrow(validationApplication, candidate);
        return isValid
            ? AssetImportResult::Success
            : AssetImportResult::InvalidData;
    } catch (const Standard_Failure& failure) {
        std::cout << "Validate CBF failure: " << failure.GetMessageString() << std::endl;
        CloseDocumentNoThrow(validationApplication, candidate);
        return AssetImportResult::InternalFailure;
    } catch (const std::bad_alloc&) {
        CloseDocumentNoThrow(validationApplication, candidate);
        return AssetImportResult::InternalFailure;
    } catch (const std::exception& exception) {
        std::cout << "Validate CBF exception: " << exception.what() << std::endl;
        CloseDocumentNoThrow(validationApplication, candidate);
        return AssetImportResult::InternalFailure;
    } catch (...) {
        CloseDocumentNoThrow(validationApplication, candidate);
        return AssetImportResult::InternalFailure;
    }
}

bool Core3DViewer::redrawDocument() noexcept {
    if (hasUnresolvedOrdinaryEdit() || HasActiveOperationLedger(_objectInteractor, _shapeInteractor)) {
        // A same-document rebuild is safe only after every typed operation has
        // retired its controller ledger. The manipulator enum alone cannot
        // recreate Boolean, Bevel, Mirror, Array, Extrude, or Shell state.
        return false;
    }
#ifdef DEBUG
    const Standard_Boolean shouldForceTraversalFailure = std::exchange(
        _debugForceNextTransformInspectorRedrawFailure,
        Standard_False);
#endif
    const PrimitiveManipulatorType manipulatorType =
        StatelessManipulatorTypeOrNone(
            _objectInteractor == nullptr
                ? PrimitiveManipulatorType::PrimitiveGizmoTypeNone
                : _objectInteractor->getManipulatorType());
    const ShapeSelectionMode selectionMode = _shapeInteractor == nullptr
        ? ShapeSelectionMode::WholeShape
        : _shapeInteractor->getSelectionMode();
    AIS_ListOfInteractive previousPresentations;
    // Shaded publication may clear a face TShape's Checked bit while adding
    // triangulation. That bit belongs to the exact persisted BRep stream even
    // though triangles do not. Keep only a synchronous, structurally-derived
    // snapshot and restore the original value on every exit; this is not a
    // discovery-time repair or a new currentness baseline.
    struct RedrawShapeFlags final {
        std::vector<std::pair<TopoDS_Shape,Standard_Boolean>> rootFree;
        std::vector<std::pair<Handle(TopoDS_TShape),Standard_Boolean>> carrierFaceChecked;
        ~RedrawShapeFlags() {
            for (auto& entry:carrierFaceChecked)
                if (!entry.first.IsNull()) entry.first->Checked(entry.second);
            for (auto& entry:rootFree) entry.first.Free(entry.second);
        }
    } preservedFlags;
    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document) document = myDoc.IsNull()
            ? Handle(TDocStd_Document)() : myDoc->Document();
        if (document.IsNull() || document->HasOpenCommand()) return false;
        const Handle(XCAFDoc_ShapeTool) shapes =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return false;
        TDF_LabelSequence roots;
        shapes->GetFreeShapes(roots);
        for (int i=1;i<=roots.Length();++i) {
            const TopoDS_Shape shape=XCAFDoc_ShapeTool::GetShape(roots.Value(i));
            if (!shape.IsNull()) preservedFlags.rootFree.emplace_back(shape,shape.Free());
        }

        std::vector<composite_recipe::Record> records;
        if (!composite_recipe::ReadAll(document,records)) return false;
        constexpr std::size_t maximumAnalyticCarrierFaces=1024;
        std::unordered_set<const TopoDS_TShape*> retainedFaceIdentities;
        for (const auto& record:records) {
            if (!record.value || record.value->definition.nodes.empty()) return false;
            const auto* feature=std::get_if<composite_recipe::FeatureNode>(
                &record.value->definition.nodes.back().value);
            if (!feature || feature->kind!=composite_recipe::PartBooleanFeatureKind) continue;
            if (feature->codecVersion!=composite_recipe::PartBooleanShellFeatureCodec) {
                part_boolean::AnalyticDefinition analytic;
                if (record.value->definition.nodes.size()!=3
                    || feature->codecVersion!=composite_recipe::PartBooleanFeatureCodec
                    || !part_boolean::DecodeAnalytic(feature->parameters,analytic)
                    || record.current.IsNull()) return false;
            }
            std::unordered_set<const TopoDS_TShape*> distinctCarrierFaces;
            std::size_t faceCount=0;
            for (TopExp_Explorer face(record.current,TopAbs_FACE);face.More();face.Next()) {
                const Handle(TopoDS_TShape)& tshape=face.Current().TShape();
                if (tshape.IsNull()) return false;
                if (!distinctCarrierFaces.insert(tshape.get()).second) continue;
                if (++faceCount>maximumAnalyticCarrierFaces) return false;
                if (retainedFaceIdentities.insert(tshape.get()).second)
                    preservedFlags.carrierFaceChecked.emplace_back(tshape,tshape->Checked());
            }
            if (faceCount==0) return false;
        }
    } catch (...) {
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
        // OCAF has already restored the original TShapes, including their
        // bookkeeping bits. XCAFPrs material dispatch temporarily groups a
        // root into a presentation compound; BRep_Builder::Add then clears
        // its shared Free flag outside any command. Preserve that exact bit
        // through redraw (also on failure), so Undo does not alter the saved
        // geometry stream merely by replacing a plain AIS presentation.
        myContext->DisplayedObjects(
            AIS_KOI_Shape,
            -1,
            previousPresentations
        );
        clearContext();
#ifdef DEBUG
        if (shouldForceTraversalFailure) {
            throw Standard_Failure(
                "Injected Position redraw traversal failure"
            );
        }
#endif
        if (traverseDocument(myDoc->ChangeDocument())) {
            throw Standard_Failure(
                "Unable to rebuild document presentations"
            );
        }
        if (!recreateInteractors(manipulatorType, selectionMode)) {
            throw Standard_Failure(
                "Unable to restore topology selection mode");
        }
        myContext->UpdateCurrentViewer();
        return true;
    } catch (...) {
        // Remove any partial traversal before restoring the exact retained
        // presentation handles. RemoveAll() never destroys those handles.
    }

    try {
        OCC_CATCH_SIGNALS
        clearContext();
        for (AIS_ListIteratorOfListOfInteractive aPresentation(
                 previousPresentations);
             aPresentation.More(); aPresentation.Next()) {
            const Handle(AIS_InteractiveObject)& anObject =
                aPresentation.Value();
            if (!Handle(AIS_Shape)::DownCast(anObject).IsNull()) {
                myContext->Display(anObject, Standard_False);
            }
        }
        if (!recreateInteractors(manipulatorType, selectionMode)) {
            throw Standard_Failure(
                "Unable to restore retained topology selection mode");
        }
        myContext->UpdateCurrentViewer();
    } catch (...) {
    }
    return false;
}

void Core3DViewer::setPreviewMode() {
    myView->TriedronErase();
}

void Core3DViewer::showGrid(bool show) {
    if (show) {
        myViewer->ActivateGrid(Aspect_GridType::Aspect_GT_Rectangular, Aspect_GridDrawMode::Aspect_GDM_Lines);
    } else {
        myViewer->DeactivateGrid();
    }
    myView->Redraw();
}

#if DEBUG
namespace {
// R179/D249 passive diagnostic. Reads only the already captured snapshot; it
// never recaptures, meshes or redraws. Bounded to 64 KiB with an explicit
// truncated/error status and an explicit invalid-mesh-index marker (an invalid
// index is never dereferenced).
std::string R179SceneInstanceCensus(const scene::SceneSnapshot& snapshot) noexcept {
    const auto clean = [](const std::string& raw) {
        return core3d::general_loft::persistence::debug::CensusClean(
            raw.c_str(), 128);
    };
    try {
        std::string rows;
        std::size_t emitted = 0, index = 0;
        bool truncated = false;
        for (const auto& instance : snapshot.instances) {
            const std::string row = std::to_string(index++)
                + "|entity=" + clean(instance.entityIdentifier)
                + "|definition="
                + (instance.meshIndex >= snapshot.meshes.size()
                    ? std::string("INVALID-MESH-INDEX")
                    : clean(snapshot.meshes[instance.meshIndex].definitionIdentifier))
                + "|role=" + std::to_string(static_cast<unsigned>(instance.role))
                + "|visible=" + (instance.visible ? "1" : "0")
                + "|selectable=" + (instance.selectable ? "1" : "0")
                + "|selected=" + (instance.selected ? "1" : "0")
                + "|mesh=" + std::to_string(instance.meshIndex)
                + ";";
            if (truncated || rows.size() + row.size() > 64 * 1024 - 256) {
                truncated = true;
                break;
            }
            rows += row;
            ++emitted;
        }
        return std::string("status=") + (truncated ? "truncated" : "ok")
            + " instances=" + std::to_string(snapshot.instances.size())
            + " emitted=" + std::to_string(emitted)
            + " rows=[" + rows + "]";
    } catch (...) {
        return "status=error";
    }
}
} // namespace
#endif

bool Core3DViewer::selectObjectFromBrowser(
    const ObjectFrameIdentity& identity,
    const std::uint32_t viewportWidth,
    const std::uint32_t viewportHeight,
    bool& selectionWasTouched) noexcept {
    observeNativePlanningInteraction();
    selectionWasTouched = false;
    if (![NSThread isMainThread] || !canBeginCommittedEdit()
        || myContext.IsNull() || myView.IsNull()
        || viewportWidth == 0 || viewportHeight == 0
        || identity.entityIdentifier.empty() || identity.entityIdentifier.size() > 128
        || identity.entityIdentifier.find('\0') != std::string::npos
        || identity.publicationSourceIdentifier.empty()
        || identity.publicationSourceIdentifier.size() > 128
        || identity.publicationSourceIdentifier.find('\0') != std::string::npos
        || _objectInteractor->isManipulatorGestureActive()
        || _shapeInteractor->getSelectionMode() != ShapeSelectionMode::WholeShape
        || !_shapeInteractor->selectionModeAuthorityIsExact()) {
#if DEBUG
        std::fprintf(stderr, "R179_SELECT_REFUSED stage=precondition\n");
#endif
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
        const auto snapshot = captureSceneSnapshot(viewportWidth, viewportHeight);
        if (snapshot == nullptr
            || snapshot->selectionMode != scene::ElementKind::Object) {
#if DEBUG
            std::fprintf(stderr, "R179_SELECT_REFUSED stage=snapshot\n");
#endif
            return false;
        }
        if (identity.publicationSourceIdentifier != snapshot->publicationSourceIdentifier
            || identity.documentGeneration != snapshot->revisions.documentGeneration
            || identity.modelRevision != snapshot->revisions.model) {
#if DEBUG
            std::fprintf(stderr, "R179_SELECT_REFUSED stage=snapshot-identity\n");
#endif
            return false;
        }
        std::size_t matchingInstances = 0;
        for (const auto& instance : snapshot->instances) {
            if (instance.entityIdentifier == identity.entityIdentifier) {
                if (instance.role != scene::RenderRole::Model || !instance.visible
                    || !instance.selectable) {
#if DEBUG
                    std::fprintf(stderr, "R179_SELECT_REFUSED stage=instance\n");
#endif
                    return false;
                }
                ++matchingInstances;
            }
        }
        if (matchingInstances != 1) {
#if DEBUG
            // R179/D249: keep the exact refusal. The two census strings are
            // computed only after reaching this failure block; both are
            // passive, bounded and never selection authority.
            std::string nativeRows = "status=error";
            std::string sceneRows = "status=error";
            try {
                nativeRows = core3d::general_loft::persistence::debug::Census(
                    myDoc->Document(), identity.entityIdentifier);
                sceneRows = R179SceneInstanceCensus(*snapshot);
            } catch (...) {}
            std::fprintf(stderr, "R179_SELECT_REFUSED stage=instance-count entity=%.128s matching=%zu total=%zu document=%.128s publication=%.128s generation=%llu model=%llu native=%s scene=%s\n", identity.entityIdentifier.c_str(), matchingInstances, snapshot->instances.size(), myDoc->DocumentIdentifier().c_str(), snapshot->publicationSourceIdentifier.c_str(), static_cast<unsigned long long>(snapshot->revisions.documentGeneration), static_cast<unsigned long long>(snapshot->revisions.model), nativeRows.c_str(), sceneRows.c_str());
#endif
            return false;
        }
        Handle(AIS_InteractiveObject) target;
        AIS_ListOfInteractive displayed;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
        std::size_t inspected = 0;
        for (AIS_ListIteratorOfListOfInteractive item(displayed); item.More(); item.Next()) {
            if (++inspected > 50000) {
#if DEBUG
                std::fprintf(stderr, "R179_SELECT_REFUSED stage=target-bounds\n");
#endif
                return false;
            }
            const auto candidate = item.Value();
            const TDF_Label label = myDoc->ShapeLabel(candidate);
            if (label.IsNull() || myDoc->EntityIdentifierForLabel(label) != identity.entityIdentifier) {
                continue;
            }
            // Assembly occurrences must never resolve to their shared definition.
            if (!target.IsNull() || !myDoc->IsPresentationEditable(candidate)
                || !myDoc->IsEditableFreeSimpleDefinitionLabel(label)) {
#if DEBUG
                std::fprintf(stderr, "R179_SELECT_REFUSED stage=target-editable\n");
#endif
                return false;
            }
            target = candidate;
        }
        if (target.IsNull()) {
#if DEBUG
            std::fprintf(stderr, "R179_SELECT_REFUSED stage=target-lookup\n");
#endif
            return false;
        }
        if (!_objectInteractor->replaceSelectedObjectForBrowser(target, selectionWasTouched)) {
#if DEBUG
            std::fprintf(stderr, "R179_SELECT_REFUSED stage=interactor-busy\n");
#endif
            return false;
        }
        return true;
    } catch (...) {
#if DEBUG
        std::fprintf(stderr, "R179_SELECT_REFUSED stage=exception\n");
#endif
        return false;
    }
}

// Shared camera math only. Admission and authoritative bounds remain with
// each caller; a failed fit restores the exact previous camera.
static bool FitCameraToBounds(const Handle(V3d_View)& view, const Bnd_Box& bounds,
    double targetX, double targetY, double targetWidth, double targetHeight) noexcept {
    // Target coordinates are normalized to the full viewport, origin top-left.
    if (!std::isfinite(targetX) || !std::isfinite(targetY)
        || !std::isfinite(targetWidth) || !std::isfinite(targetHeight)
        || targetX < 0.0 || targetY < 0.0
        || targetWidth <= 0.0 || targetHeight <= 0.0
        || targetX + targetWidth > 1.0 || targetY + targetHeight > 1.0) {
        return false;
    }
    if (view.IsNull() || view->Camera().IsNull()
        || bounds.IsVoid() || bounds.IsWhole() || bounds.IsOpen()) { return false; }
    Handle(Graphic3d_Camera) previousCamera;
    bool cameraWasMutated = false;
    try {
        OCC_CATCH_SIGNALS
        previousCamera = new Graphic3d_Camera(view->Camera());
        cameraWasMutated = true;
        view->FitAll(bounds, 0.15, Standard_False);
        const auto& camera = view->Camera();
        {
            // Verify full-viewport fits too: FitAll alone can clip tall
            // perspective models. Keep projection, aspect and orientation
            // intact. First shrink the
            // full-view fit, then translate the camera parallel to its image
            // plane. Verify every bounding-box corner after perspective divide:
            // deep geometry must fit as well as the center plane.
            Standard_Real minX, minY, minZ, maxX, maxY, maxZ;
            bounds.Get(minX, minY, minZ, maxX, maxY, maxZ);
            const gp_Pnt worldCenter((minX + maxX) * 0.5,
                                     (minY + maxY) * 0.5,
                                     (minZ + maxZ) * 0.5);
            const double left = 2.0 * targetX - 1.0;
            const double right = 2.0 * (targetX + targetWidth) - 1.0;
            const double bottom = 1.0 - 2.0 * (targetY + targetHeight);
            const double top = 1.0 - 2.0 * targetY;
            camera->SetScale(camera->Scale() / std::min(targetWidth, targetHeight));
            bool fits = false;
            for (int attempt = 0; attempt < 12; ++attempt) {
                const auto projectedCenter = camera->Project(worldCenter);
                const auto targetWorld = camera->UnProject(gp_Pnt(
                    (left + right) * 0.5, (bottom + top) * 0.5,
                    projectedCenter.Z()));
                gp_Trsf shift;
                shift.SetTranslation(gp_Vec(targetWorld, worldCenter));
                camera->Transform(shift);
                fits = true;
                for (const double x : {minX, maxX}) {
                    for (const double y : {minY, maxY}) {
                        for (const double z : {minZ, maxZ}) {
                            const auto point = camera->Project(gp_Pnt(x, y, z));
                            if (!std::isfinite(point.X()) || !std::isfinite(point.Y())
                                || !std::isfinite(point.Z())
                                || point.X() < left || point.X() > right
                                || point.Y() < bottom || point.Y() > top) {
                                fits = false;
                            }
                        }
                    }
                }
                if (fits) { break; }
                camera->SetScale(camera->Scale() * 1.2);
            }
            if (!fits) {
                camera->Copy(previousCamera);
                return false;
            }
        }
        view->ZFitAll();
        const gp_Pnt eye = camera->Eye();
        const gp_Pnt center = camera->Center();
        const gp_Dir up = camera->Up();
        for (const double value : {eye.X(), eye.Y(), eye.Z(),
                                   center.X(), center.Y(), center.Z(),
                                   up.X(), up.Y(), up.Z(), camera->Scale(),
                                   camera->Distance(), camera->ZNear(),
                                   camera->ZFar()}) {
            if (!std::isfinite(value)) {
                view->Camera()->Copy(previousCamera);
                return false;
            }
        }
        if (camera->Scale() <= 0.0 || camera->Distance() <= 0.0
            || camera->ZFar() <= camera->ZNear()) {
            view->Camera()->Copy(previousCamera);
            return false;
        }
        return true;
    } catch (...) {
        if (cameraWasMutated && !previousCamera.IsNull()) {
            try { view->Camera()->Copy(previousCamera); } catch (...) {}
        }
        return false;
    }
}

bool Core3DViewer::frameModel(
    const bool selectedObjectsOnly,
    const std::uint32_t viewportWidth,
    const std::uint32_t viewportHeight,
    const double targetX, const double targetY,
    const double targetWidth, const double targetHeight,
    const ObjectFrameIdentity* objectIdentity) noexcept {
    // Target coordinates are normalized to the full viewport, origin top-left.
    if (!std::isfinite(targetX) || !std::isfinite(targetY)
        || !std::isfinite(targetWidth) || !std::isfinite(targetHeight)
        || targetX < 0.0 || targetY < 0.0
        || targetWidth <= 0.0 || targetHeight <= 0.0
        || targetX + targetWidth > 1.0 || targetY + targetHeight > 1.0) {
        return false;
    }
    if (![NSThread isMainThread] || !canBeginCommittedEdit()
        || myView.IsNull() || myContext.IsNull()
        || viewportWidth == 0 || viewportHeight == 0) {
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
        // Reuse the bounded, document-authoritative extraction contract. This
        // reads existing triangulations only and excludes gizmos/preview actors.
        const auto snapshot = captureSceneSnapshot(viewportWidth, viewportHeight);
        if (snapshot == nullptr
            || (selectedObjectsOnly
                && snapshot->selectionMode != scene::ElementKind::Object)) {
            return false;
        }
        if (objectIdentity != nullptr
            && (selectedObjectsOnly || objectIdentity->entityIdentifier.empty()
                || objectIdentity->publicationSourceIdentifier
                    != snapshot->publicationSourceIdentifier
                || objectIdentity->documentGeneration
                    != snapshot->revisions.documentGeneration
                || objectIdentity->modelRevision != snapshot->revisions.model)) {
            return false;
        }
        Bnd_Box bounds;
        for (const auto& instance : snapshot->instances) {
            if (!instance.visible || instance.role != scene::RenderRole::Model
                || (selectedObjectsOnly && !instance.selected)
                || (objectIdentity != nullptr && instance.entityIdentifier
                    != objectIdentity->entityIdentifier)) {
                continue;
            }
            if (instance.meshIndex >= snapshot->meshes.size()) { return false; }
            const auto& local = snapshot->meshes[instance.meshIndex].localBounds;
            if (!local.valid) { return false; }
            const auto& m = instance.worldFromObject.values;
            for (const double x : {local.minimum.x, local.maximum.x}) {
                for (const double y : {local.minimum.y, local.maximum.y}) {
                    for (const double z : {local.minimum.z, local.maximum.z}) {
                        const double wx = m[0]*x + m[4]*y + m[8]*z + m[12];
                        const double wy = m[1]*x + m[5]*y + m[9]*z + m[13];
                        const double wz = m[2]*x + m[6]*y + m[10]*z + m[14];
                        if (!std::isfinite(wx) || !std::isfinite(wy)
                            || !std::isfinite(wz)) { return false; }
                        bounds.Add(gp_Pnt(wx, wy, wz));
                    }
                }
            }
        }
        if (bounds.IsVoid() || bounds.IsWhole() || bounds.IsOpen()) { return false; }
        return FitCameraToBounds(myView, bounds,
            targetX, targetY, targetWidth, targetHeight);
    } catch (...) { return false; }
}

bool Core3DViewer::frameMirrorPreview(
    const MirrorPreviewFrameIdentity& expected,
    const std::uint32_t viewportWidth, const std::uint32_t viewportHeight,
    const double targetX, const double targetY,
    const double targetWidth, const double targetHeight) noexcept {
    // Target coordinates are normalized to the full viewport, origin top-left.
    if (!std::isfinite(targetX) || !std::isfinite(targetY)
        || !std::isfinite(targetWidth) || !std::isfinite(targetHeight)
        || targetX < 0.0 || targetY < 0.0
        || targetWidth <= 0.0 || targetHeight <= 0.0
        || targetX + targetWidth > 1.0 || targetY + targetHeight > 1.0) {
        return false;
    }
    if (![NSThread isMainThread] || viewportWidth == 0 || viewportHeight == 0
        || myView.IsNull() || myContext.IsNull() || myDoc.IsNull()
        || !_objectInteractor || !_shapeInteractor || hasUnresolvedEdit()
        || expected.publicationSourceIdentifier.empty()
        || expected.publicationSourceIdentifier.size() > 128
        || expected.publicationSourceIdentifier.find('\0') != std::string::npos) {
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
        // An active Ready mirror is intentionally admitted here, never through
        // canBeginCommittedEdit or the committed Frame All operation.
        const auto document = myDoc->Document();
        if (document.IsNull() || document->HasOpenCommand()
            || !_objectInteractor->canFrameMirrorPreview(expected.previewGeneration)
            || _objectInteractor->hasActiveBoolean()
            || _objectInteractor->hasUnresolvedBoolean()
            || _objectInteractor->hasActiveLinearArray()
            || _objectInteractor->hasUnresolvedLinearArray()
            || _objectInteractor->hasActiveRadialArray()
            || _objectInteractor->hasUnresolvedRadialArray()
            || _shapeInteractor->hasActiveBevel()
            || _shapeInteractor->hasActiveExtrusion()
            || _shapeInteractor->hasActiveShell()
            || _shapeInteractor->hasUnresolvedShell()) { return false; }
        const auto snapshot = captureSceneSnapshot(viewportWidth, viewportHeight);
        if (!snapshot || snapshot->selectionMode != scene::ElementKind::Object
            || snapshot->publicationSourceIdentifier != expected.publicationSourceIdentifier
            || snapshot->revisions.documentGeneration != expected.revisions.documentGeneration
            || snapshot->revisions.model != expected.revisions.model
            || snapshot->revisions.presentation != expected.revisions.presentation
            || snapshot->revisions.camera != expected.revisions.camera) { return false; }
        const auto overlay = captureScenePresentationOverlay();
        if (!overlay || overlay->kind != scene::PresentationOverlayKind::MirrorPreview
            || overlay->publicationSourceIdentifier != snapshot->publicationSourceIdentifier
            || overlay->baseSnapshotRevision != snapshot->revisions.snapshot
            || overlay->baseDocumentGeneration != snapshot->revisions.documentGeneration
            || overlay->baseModelRevision != snapshot->revisions.model
            || overlay->basePresentationRevision != snapshot->revisions.presentation
            || !overlay->suppressedEntityIdentifiers.empty()) { return false; }
        Bnd_Box bounds;
        const auto addBounds = [&bounds](const scene::InstanceSnapshot& item,
            const std::vector<scene::MeshSnapshot>& meshes) {
            if (item.coordinateSpace != scene::CoordinateSpace::World
                || item.meshIndex >= meshes.size()) { return false; }
            const auto& local = meshes[item.meshIndex].localBounds;
            if (!local.valid) { return false; }
            const auto& m = item.worldFromObject.values;
            for (const double x : {local.minimum.x, local.maximum.x}) {
                for (const double y : {local.minimum.y, local.maximum.y}) {
                    for (const double z : {local.minimum.z, local.maximum.z}) {
                        const double wx = m[0]*x + m[4]*y + m[8]*z + m[12];
                        const double wy = m[1]*x + m[5]*y + m[9]*z + m[13];
                        const double wz = m[2]*x + m[6]*y + m[10]*z + m[14];
                        if (!std::isfinite(wx) || !std::isfinite(wy)
                            || !std::isfinite(wz)) { return false; }
                        bounds.Add(gp_Pnt(wx, wy, wz));
                    }
                }
            }
            return true;
        };
        std::size_t sourceCount = 0, previewCount = 0;
        for (const auto& item : snapshot->instances) {
            if (!item.visible || !item.selected || item.role != scene::RenderRole::Model) { continue; }
            if (!addBounds(item, snapshot->meshes)) { return false; }
            ++sourceCount;
        }
        for (const auto& item : overlay->instances) {
            // The validated six leading plane handles have Gizmo role and
            // pixel-space bounds; include only world-space result geometry.
            if (!item.visible || item.role != scene::RenderRole::MirrorPreview) { continue; }
            if (!addBounds(item, overlay->meshes)) { return false; }
            ++previewCount;
        }
        if (sourceCount == 0 || previewCount == 0
            || !_objectInteractor->canFrameMirrorPreview(expected.previewGeneration)) { return false; }
        return FitCameraToBounds(myView, bounds,
            targetX, targetY, targetWidth, targetHeight);
    } catch (...) { return false; }
}

bool Core3DViewer::setCameraOrthographic(const bool orthographic) noexcept {
    if (![NSThread isMainThread] || !canBeginCommittedEdit()
        || myView.IsNull() || myContext.IsNull()) { return false; }
    Handle(Graphic3d_Camera) previous;
    try {
        OCC_CATCH_SIGNALS
        const auto camera = myView->Camera();
        if (camera.IsNull()) { return false; }
        const auto projection = orthographic
            ? Graphic3d_Camera::Projection_Orthographic
            : Graphic3d_Camera::Projection_Perspective;
        if (camera->ProjectionType() == projection) { return true; }
        const double scale = camera->Scale();
        if (!std::isfinite(scale) || scale <= 0.0) { return false; }
        previous = new Graphic3d_Camera(camera);
        camera->SetProjectionType(projection);
        // OCCT interprets perspective scale as distance and orthographic scale
        // as view height. Preserve the same apparent size at the target plane.
        camera->SetScale(scale);
        camera->SetCenter(previous->Center());
        myView->ZFitAll();
        const auto eye = camera->Eye();
        const auto center = camera->Center();
        for (const double value : {camera->Scale(), camera->Distance(),
                camera->ZNear(), camera->ZFar(), eye.X(), eye.Y(), eye.Z(),
                center.X(), center.Y(), center.Z()}) {
            if (!std::isfinite(value)) {
                camera->Copy(previous);
                return false;
            }
        }
        if (camera->Scale() <= 0.0 || camera->Distance() <= 0.0
            || camera->ZFar() <= camera->ZNear()
            || (!orthographic && camera->ZNear() <= 0.0)) {
            camera->Copy(previous);
            return false;
        }
        return true;
    } catch (...) {
        if (!previous.IsNull()) {
            try { myView->Camera()->Copy(previous); } catch (...) {}
        }
        return false;
    }
}

void Core3DViewer::setOrthoProjection(const OrthoProjectionType orthoType) {

        V3d_TypeOfOrientation orientation = V3d_Yneg;

        switch (orthoType) {
            case OrthoProjectionTypeFront:
                orientation = V3d_Xpos;
                break;
            case OrthoProjectionTypeBack:
                orientation = V3d_Xneg;
                break;
            case OrthoProjectionTypeTop:
                orientation = V3d_Zpos;
                break;
            case OrthoProjectionTypeBottom:
                orientation = V3d_Zneg;
                break;
            case OrthoProjectionTypeLeft:
                orientation = V3d_Yneg;
                break;
            case OrthoProjectionTypeRight:
                orientation = V3d_Ypos;
                break;
            default:
                break;
        }

        myView->SetProj(orientation);

        if (myView->Camera()->ProjectionType() != Graphic3d_Camera::Projection_Orthographic) {
            myView->Camera()->SetProjectionType(Graphic3d_Camera::Projection_Orthographic);
			if (orthoType == OrthoProjectionTypeTop)
				myView->Camera()->SetUp(gp::DX());
			else if (orthoType == OrthoProjectionTypeBottom)
				myView->Camera()->SetUp(-gp::DX());
        }

        myView->FitAll(0.2, Standard_False);
        myView->RedrawImmediate();
    }

void Core3DViewer::StartRotation(int theX, int theY) {
    if (_documentReplacementWork || _queuedAssetLoadWork) return;
    if(_objectInteractor == nullptr) {
        return;
    }
	if (_objectInteractor->isPickingMirrorPlane()) {
		// Keep the raw touch lifecycle from activating the Mirror gizmo or
		// rotating the camera. The tap recognizer will deliver the final point
		// to Select(), which consumes it without touching AIS selection.
		return;
	}
    if(!_objectInteractor->startTransformManipulator(theX, theY)) {
        OcctViewer::StartRotation(theX, theY);
    }
}

Standard_Boolean
Core3DViewer::captureSelectionModeSuspendedPresentations(
    std::vector<Handle(AIS_Shape)>& thePresentations) const noexcept
{
    thePresentations.clear();
    if (_objectInteractor == nullptr || _shapeInteractor == nullptr) {
        return Standard_False;
    }
    switch (_objectInteractor->getManipulatorType()) {
        case PrimitiveManipulatorType::PrimitiveGizmoTypeExtrude:
            return _shapeInteractor
                ->captureExtrusionSelectionModeSuspendedPresentations(
                    thePresentations);
        case PrimitiveManipulatorType::PrimitiveGizmoTypeShell:
            return _shapeInteractor
                ->captureShellSelectionModeSuspendedPresentations(
                    thePresentations);
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMirror:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeUnion:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeIntersect:
            return _objectInteractor
                ->captureSelectionModeSuspendedPresentations(
                    thePresentations);
        case PrimitiveManipulatorType::PrimitiveGizmoTypeNone:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeScale:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial:
            return Standard_False;
    }
    return Standard_False;
}

bool Core3DViewer::selectionModeAuthorityAllowsRetainedOperation()
    const noexcept
{
    if (_shapeInteractor == nullptr) {
        return false;
    }
    std::vector<Handle(AIS_Shape)> suspendedPresentations;
    return captureSelectionModeSuspendedPresentations(
               suspendedPresentations)
        && _shapeInteractor->selectionModeAuthorityIsExactIgnoring(
            suspendedPresentations);
}

#ifdef DEBUG
Standard_Boolean Core3DViewer::DebugSetDetectedOwner(
	const Handle(SelectMgr_EntityOwner)& theOwner) noexcept
{
	if (myContext.IsNull() || theOwner.IsNull()) {
		return Standard_False;
	}
	try {
		OCC_CATCH_SIGNALS
		(void)myContext->ClearDetected(Standard_False);
		return myContext->DebugSetDetectedOwner(theOwner);
	} catch (...) {
		return Standard_False;
	}
}

Standard_Boolean
Core3DViewer::DebugSelectRetainedOperationPresentation() noexcept
{
    if (myContext.IsNull()) {
        return Standard_False;
    }
    std::vector<Handle(AIS_Shape)> suspendedPresentations;
    if (!captureSelectionModeSuspendedPresentations(
            suspendedPresentations)
        || suspendedPresentations.empty()) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(AIS_Shape)& aPresentation =
            suspendedPresentations.back();
        if (aPresentation.IsNull()
            || aPresentation->Shape().IsNull()
            || !myContext->IsDisplayed(aPresentation)) {
            return Standard_False;
        }
        // The retained presentation intentionally has no active selection
        // mode, so SetSelected(AIS_InteractiveObject) has no cached owner to
        // publish. Manufacture the same full-shape BRep owner OCCT would have
        // produced before deactivation. This remains DEBUG-only and is bound
        // to the controller-proven presentation.
        const Handle(StdSelect_BRepOwner) anOwner =
            new StdSelect_BRepOwner(
                aPresentation->Shape(), aPresentation, 0, Standard_False);
        myContext->ClearSelected(Standard_False);
        myContext->SetSelected(anOwner, Standard_True);
        return selectedCount() == 1;
    } catch (...) {
        try {
            OCC_CATCH_SIGNALS
            myContext->ClearSelected(Standard_False);
        } catch (...) {
        }
        return Standard_False;
    }
}
#endif

meshcheck::ContactSourceStatus Core3DViewer::captureNativeMeshContacts(
    const meshcheck::ContactSourceIdentity& expected,
    const std::uint32_t width, const std::uint32_t height,
    const std::atomic_bool& cancelled,
    meshcheck::ContactSourceCapture& output) noexcept
{
    using Status = meshcheck::ContactSourceStatus;
    output = {};
    if (cancelled.load(std::memory_order_relaxed)) return Status::Cancelled;
    if (![NSThread isMainThread]) return Status::InternalFailure;
    const auto snapshot = captureSceneSnapshot(width, height);
    if (!snapshot) return Status::StaleSource;
    return _sceneSnapshotBuilder.CaptureNativeMeshContacts(
        myDoc, expected, cancelled, output);
}

meshcheck::ContactSourceStatus Core3DViewer::validateNativeMeshContacts(
    const meshcheck::ContactSourceCapture& original,
    const std::uint32_t width, const std::uint32_t height,
    const std::atomic_bool& cancelled) noexcept
{
    meshcheck::ContactSourceCapture current;
    const auto status = captureNativeMeshContacts(
        original.identity, width, height, cancelled, current);
    if (status != meshcheck::ContactSourceStatus::Ready) return status;
    return meshcheck::SameContactSource(original, current)
        ? meshcheck::ContactSourceStatus::Ready
        : meshcheck::ContactSourceStatus::StaleSource;
}

scene::OcctSceneSnapshotBuilder::SnapshotPointer
Core3DViewer::captureSceneSnapshot(
    const std::uint32_t viewportWidth,
    const std::uint32_t viewportHeight) noexcept {
    if (hasUnresolvedEdit()) {
        return {};
    }
    if (_objectInteractor != nullptr
        && (_objectInteractor->mirrorPreviewState()
                == MirrorPreviewState::OutcomeUnknown
            || _objectInteractor->linearArrayPreviewState()
                == LinearArrayPreviewState::OutcomeUnknown
            || _objectInteractor->hasUnresolvedBoolean())) {
        // A closed command may already have changed OCAF, but only the typed
        // controller can reconcile that outcome exactly once. Do not advance
        // committed renderer revisions while its ledger remains authoritative.
        return {};
    }
    if (_objectInteractor != nullptr
        && _objectInteractor->radialArrayPreviewState()
            == RadialArrayPreviewState::OutcomeUnknown) {
        // The geometry command or separate reference-axis edit may already
        // have committed. Do not advance committed renderer revision state
        // until the controller's retained ledger proves the exact outcome.
        return {};
    }
    if (_shapeInteractor != nullptr
        && (_shapeInteractor->shellPreviewState()
                == ShellPreviewState::OutcomeUnknown
            || _shapeInteractor->extrusionPreviewState()
                == ExtrusionPreviewState::OutcomeUnknown)) {
        // The candidate may already be committed, but the Shell controller
        // still owns the only authoritative reconciliation token. Publishing
        // here would expose an outcome that the operation cannot yet prove and
        // would also advance the snapshot builder's committed revision state.
        return {};
    }
    if (_shapeInteractor == nullptr) {
        return {};
    }
    if (!_shapeInteractor->selectionModeAuthorityIsExact()) {
        bool hasRendererSafeReadyObjectPreview = false;
        if (_objectInteractor != nullptr
            && _objectInteractor->linearArrayPreviewState()
                == LinearArrayPreviewState::Ready) {
            std::vector<Handle(AIS_Shape)> previewObjects;
            hasRendererSafeReadyObjectPreview =
                _objectInteractor->captureLinearArrayPreview(previewObjects)
                && _shapeInteractor->selectionModeAuthorityIsExactIgnoring(
                    previewObjects);
        } else if (_objectInteractor != nullptr
                   && _objectInteractor->radialArrayPreviewState()
                       == RadialArrayPreviewState::Ready) {
            RadialArrayPreviewCapture preview;
            hasRendererSafeReadyObjectPreview =
                _objectInteractor->captureRadialArrayPreview(preview)
                && _shapeInteractor->selectionModeAuthorityIsExactIgnoring(
                    preview.previewObjects);
        } else if (_objectInteractor != nullptr
                   && _objectInteractor->mirrorPreviewState()
                       == MirrorPreviewState::Ready) {
            scene::PresentationOverlayContent content;
            std::vector<Handle(AIS_Shape)> previewObjects;
            BooleanPreviewCapture booleanPreview;
            hasRendererSafeReadyObjectPreview =
                _objectInteractor->captureIdlePresentationOverlay(
                    content, previewObjects, booleanPreview)
                    == PresentationOverlayCaptureStatus::Available
                && !previewObjects.empty()
                && _shapeInteractor->selectionModeAuthorityIsExactIgnoring(
                    previewObjects);
        }
        if (!hasRendererSafeReadyObjectPreview) {
        // Never promote a retained enum whose OCCT presentations could not be
        // proven or restored. A same-mode request can repair this fail-closed
        // state without publishing false renderer selection identity.
            return {};
        }
    }
    scene::ElementKind acceptedSelectionKind = scene::ElementKind::None;
    switch (_shapeInteractor->getSelectionMode()) {
        case ShapeSelectionMode::WholeShape:
            acceptedSelectionKind = scene::ElementKind::Object;
            break;
        case ShapeSelectionMode::Face:
            acceptedSelectionKind = scene::ElementKind::Face;
            break;
        case ShapeSelectionMode::Edge:
            acceptedSelectionKind = scene::ElementKind::Edge;
            break;
        case ShapeSelectionMode::Vertex:
            return {};
        case ShapeSelectionMode::Wire:
            return {};
    }
    return _sceneSnapshotBuilder.Build(
        myDoc,
        myContext,
        myView,
        scene::UInt2{viewportWidth, viewportHeight},
        acceptedSelectionKind);
}

std::optional<scene::FrameSnapshot>
Core3DViewer::captureSceneFrameSnapshot(
    const std::uint32_t viewportWidth,
    const std::uint32_t viewportHeight) noexcept {
    if (hasUnresolvedEdit()) {
        return std::nullopt;
    }
    return _sceneSnapshotBuilder.CaptureFrame(
        myDoc,
        myView,
        scene::UInt2{viewportWidth, viewportHeight});
}

scene::OcctSceneSnapshotBuilder::OverlayPointer
Core3DViewer::captureScenePresentationOverlay() noexcept {
    if (hasUnresolvedEdit()) {
        return {};
    }
    if (_shapeInteractor != nullptr
        && _shapeInteractor->hasActiveShell()) {
        ShellPreviewCapture aShellPreview;
        if (!_shapeInteractor->captureShellPreview(aShellPreview)) {
            // Computing, committing, failed, or otherwise unsafe Shell state
            // remains exclusively owned by the OCCT viewport.
            return {};
        }
        return _sceneSnapshotBuilder.PublishShellPreviewOverlay(
            myDoc,
            aShellPreview.result,
            aShellPreview.suppressedSourceLabel);
    }
    if (_shapeInteractor != nullptr
        && _shapeInteractor->hasActiveBevel()) {
        BevelPreviewCapture aBevelPreview;
        if (!_shapeInteractor->captureBevelPreview(aBevelPreview)) {
            // Selecting, computing, committing, failed, or otherwise unsafe
            // Bevel state remains exclusively owned by the OCCT viewport.
            return {};
        }
        return _sceneSnapshotBuilder.PublishChamferPreviewOverlay(
            myDoc,
            aBevelPreview.results,
            aBevelPreview.suppressedSourceLabels);
    }
    if (_objectInteractor == nullptr) {
        return {};
    }
    if (_objectInteractor->hasActiveRadialArray()) {
        RadialArrayPreviewCapture aRadialArrayPreview;
        if (!_objectInteractor->captureRadialArrayPreview(
                aRadialArrayPreview)) {
            if (!_objectInteractor
                    ->canPublishEmptyRadialArrayPreview()) {
                // Failed, committing, outcome-unknown, stale-reference, and
                // textured states remain exclusively authoritative in OCCT.
                return {};
            }
            return _sceneSnapshotBuilder
                .PublishEmptyRadialArrayPreviewOverlay(myDoc);
        }
        return _sceneSnapshotBuilder.PublishRadialArrayPreviewOverlay(
            myDoc,
            aRadialArrayPreview.sourcePresentation,
            aRadialArrayPreview.previewObjects);
    }
    if (_objectInteractor->hasActiveLinearArray()) {
        std::vector<Handle(AIS_Shape)> aLinearArrayPreviewObjects;
        if (!_objectInteractor->captureLinearArrayPreview(
                aLinearArrayPreviewObjects)) {
            if (!_objectInteractor
                    ->canPublishEmptyLinearArrayPreview()) {
                // Failed, committing, outcome-unknown, unresolved, and
                // textured states stay exclusively in OCCT. Only a stable
                // zero-spacing clear is renderer-neutral without geometry.
                return {};
            }
            return _sceneSnapshotBuilder
                .PublishEmptyLinearArrayPreviewOverlay(myDoc);
        }
        return _sceneSnapshotBuilder.PublishLinearArrayPreviewOverlay(
            myDoc,
            aLinearArrayPreviewObjects);
    }
    scene::PresentationOverlayContent aContent;
    std::vector<Handle(AIS_Shape)> aMirrorPreviewObjects;
    BooleanPreviewCapture aBooleanPreview;
    if (_objectInteractor->captureIdlePresentationOverlay(
            aContent,
            aMirrorPreviewObjects,
            aBooleanPreview)
        != PresentationOverlayCaptureStatus::Available) {
        return {};
    }
    if (!aMirrorPreviewObjects.empty()) {
        return _sceneSnapshotBuilder.PublishMirrorPreviewOverlay(
            myDoc,
            std::move(aContent),
            aMirrorPreviewObjects);
    }
    if (!aBooleanPreview.actors.empty()
        || !aBooleanPreview.results.empty()) {
        return _sceneSnapshotBuilder.PublishBooleanPreviewOverlay(
            myDoc,
            BooleanOverlayKind(aBooleanPreview.action),
            aBooleanPreview.actors,
            aBooleanPreview.results,
            aBooleanPreview.suppressedSourceLabels);
    }
    return _sceneSnapshotBuilder.PublishPresentationOverlay(
        myDoc,
        std::move(aContent));
}

#ifdef DEBUG
Standard_Boolean Core3DViewer::debugCycleBooleanSelection(
    BooleanAction action, const std::string& entity, bool beginEmpty) noexcept {
    PrimitiveManipulatorType expected = PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
    if (![NSThread isMainThread] || _objectInteractor == nullptr || myDoc.IsNull()
        || myContext.IsNull() || !TryManipulatorForBooleanAction(action, expected)
        || _objectInteractor->getManipulatorType() != expected) return Standard_False;
    return _objectInteractor->debugCycleBooleanSelection(action, entity, beginEmpty);
}

Standard_Boolean Core3DViewer::debugBeginBooleanSelection(
    const BooleanAction theAction,
    const std::vector<std::string>& theActorEntityIdentifiers,
    const std::vector<std::string>& theSubjectEntityIdentifiers) noexcept {
    PrimitiveManipulatorType anExpectedManipulator =
        PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
    if (_objectInteractor == nullptr || myContext.IsNull() || myDoc.IsNull()
        || !TryManipulatorForBooleanAction(
            theAction, anExpectedManipulator)
        || (theAction != BooleanAction::BooleanSubtract
            && !theActorEntityIdentifiers.empty())) {
        return Standard_False;
    }
    if (_objectInteractor->getManipulatorType()
        != anExpectedManipulator) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        _objectInteractor->cancelActiveBoolean();

        std::unordered_set<std::string> aRequested;
        for (const std::string& anIdentifier : theActorEntityIdentifiers) {
            if (anIdentifier.empty()
                || !aRequested.insert(anIdentifier).second) {
                return Standard_False;
            }
        }
        for (const std::string& anIdentifier : theSubjectEntityIdentifiers) {
            if (anIdentifier.empty()
                || !aRequested.insert(anIdentifier).second) {
                return Standard_False;
            }
        }

        std::unordered_map<std::string, Handle(AIS_InteractiveObject)>
            aCommittedPresentations;
        AIS_ListOfInteractive aDisplayed;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, aDisplayed);
        for (AIS_ListIteratorOfListOfInteractive anObject(aDisplayed);
             anObject.More(); anObject.Next()) {
            const Handle(AIS_InteractiveObject)& aPresentation =
                anObject.Value();
            const TDF_Label aLabel = myDoc->ShapeLabel(aPresentation);
            const std::string anIdentifier =
                myDoc->EntityIdentifierForLabel(aLabel);
            if (aLabel.IsNull() || anIdentifier.empty()
                || aRequested.find(anIdentifier) == aRequested.end()) {
                continue;
            }
            if (!aCommittedPresentations.emplace(
                    anIdentifier,
                    aPresentation).second) {
                return Standard_False;
            }
        }
        if (aCommittedPresentations.size() != aRequested.size()) {
            return Standard_False;
        }

        std::vector<Handle(AIS_InteractiveObject)> anActors;
        std::vector<Handle(AIS_InteractiveObject)> aSubjects;
        anActors.reserve(theActorEntityIdentifiers.size());
        aSubjects.reserve(theSubjectEntityIdentifiers.size());
        for (const std::string& anIdentifier : theActorEntityIdentifiers) {
            anActors.push_back(aCommittedPresentations.at(anIdentifier));
        }
        for (const std::string& anIdentifier : theSubjectEntityIdentifiers) {
            aSubjects.push_back(aCommittedPresentations.at(anIdentifier));
        }
        return _objectInteractor->debugBeginBooleanSelection(
            anActors,
            aSubjects,
            theAction);
    } catch (...) {
        _objectInteractor->cancelActiveBoolean();
        return Standard_False;
    }
}

BooleanPreviewDebugState
Core3DViewer::DebugBooleanPreviewState() const noexcept
{
    return _objectInteractor == nullptr
        ? BooleanPreviewDebugState()
        : _objectInteractor->debugBooleanPreviewState();
}

void Core3DViewer::DebugSetBooleanPreviewWorkerBlocked(
    const Standard_Boolean theBlocked) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetBooleanPreviewWorkerBlocked(
            theBlocked);
    }
}

void Core3DViewer::DebugSetMaximumBooleanCaptureTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetMaximumBooleanCaptureTopologyNodes(
            theLimit);
    }
}

void Core3DViewer::DebugSetMaximumBooleanResultTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetMaximumBooleanResultTopologyNodes(
            theLimit);
    }
}

void Core3DViewer::DebugSetMaximumBooleanResultSolids(
    const Standard_Size theLimit) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetMaximumBooleanResultSolids(theLimit);
    }
}

void Core3DViewer::DebugSetBooleanTransactionFailureCount(
    const Standard_Size theCount) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetBooleanTransactionFailureCount(
            theCount);
    }
}

void Core3DViewer::DebugSetBooleanMetadataFailurePhase(
    const Standard_Size thePhase) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetBooleanMetadataFailurePhase(
            thePhase);
    }
}

void Core3DViewer::DebugSetBooleanAbortFailureCount(
    const Standard_Size theCount) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetBooleanAbortFailureCount(theCount);
    }
}

void Core3DViewer::DebugSetBooleanPostCommitInspectFailureCount(
    const Standard_Size theCount) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor
            ->debugSetBooleanPostCommitInspectFailureCount(theCount);
    }
}

Standard_Boolean Core3DViewer::debugBeginExtrusionSelection(
    const std::string& theEntityIdentifier,
    const Standard_Size theFaceTopologyIndex) noexcept {
    if (_shapeInteractor == nullptr || myContext.IsNull() || myDoc.IsNull()
        || theEntityIdentifier.empty()) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        if (!_shapeInteractor->cancelExtrusion()) {
            return Standard_False;
        }
        Handle(AIS_Shape) aPresentation;
        AIS_ListOfInteractive aDisplayed;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, aDisplayed);
        for (AIS_ListIteratorOfListOfInteractive anObject(aDisplayed);
             anObject.More(); anObject.Next()) {
            const Handle(AIS_InteractiveObject)& anInteractive =
                anObject.Value();
            const TDF_Label aLabel = myDoc->ShapeLabel(anInteractive);
            if (aLabel.IsNull()
                || myDoc->EntityIdentifierForLabel(aLabel)
                    != theEntityIdentifier) {
                continue;
            }
            const Handle(AIS_Shape) aCandidate =
                Handle(AIS_Shape)::DownCast(anInteractive);
            if (!aPresentation.IsNull() || aCandidate.IsNull()
                || !myDoc->IsPresentationEditable(aCandidate)) {
                return Standard_False;
            }
            aPresentation = aCandidate;
        }
        if (aPresentation.IsNull() || aPresentation->Shape().IsNull()) {
            return Standard_False;
        }
        TopoDS_Face aFace;
        return TryResolveCanonicalFaceTopologyIndexBounded(
                aPresentation->Shape(),
                theFaceTopologyIndex,
                ShellOperationController::kMaximumSourceTopologyNodes,
                aFace)
            && _shapeInteractor->debugBeginExtrusionSelection(
                aPresentation, aFace);
    } catch (...) {
        _shapeInteractor->cancelExtrusion();
        return Standard_False;
    }
}

Standard_Boolean Core3DViewer::debugBeginShellSelection(
    const std::string& theEntityIdentifier,
    const Standard_Size theFaceTopologyIndex) noexcept
{
    try {
        return _shapeInteractor != nullptr
            && _shapeInteractor->debugBeginShellSelection(
                theEntityIdentifier, std::vector<Standard_Size>{theFaceTopologyIndex});
    } catch (...) {
        return Standard_False;
    }
}

ShellPreviewDebugState
Core3DViewer::DebugShellPreviewState() const noexcept
{
    return _shapeInteractor == nullptr
        ? ShellPreviewDebugState()
        : _shapeInteractor->debugShellState();
}

void Core3DViewer::DebugSetShellPreviewWorkerBlocked(
    const Standard_Boolean theBlocked) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetShellWorkerBlocked(theBlocked);
    }
}

void Core3DViewer::DebugSetMaximumShellCaptureTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetMaximumShellCaptureTopologyNodes(
            theLimit);
    }
}

void Core3DViewer::DebugSetMaximumShellResultTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetMaximumShellResultTopologyNodes(
            theLimit);
    }
}

void Core3DViewer::DebugSetShellTransactionFailureCount(
    const Standard_Size theCount) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetShellTransactionFailureCount(
            theCount);
    }
}

void Core3DViewer::DebugSetShellAbortFailureCount(
    const Standard_Size theCount) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetShellAbortFailureCount(theCount);
    }
}

void Core3DViewer::DebugSetShellPreviewEraseFailureCount(
    const Standard_Size theCount) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetShellPreviewEraseFailureCount(
            theCount);
    }
}

void Core3DViewer::DebugSetShellCommitMode(
    const Standard_Integer theMode) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetShellCommitMode(theMode);
    }
}

void Core3DViewer::DebugSetShellPostCommitInspectFailureCount(
    const Standard_Size theCount) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetShellPostCommitInspectFailureCount(
            theCount);
    }
}

Standard_Boolean
Core3DViewer::DebugMutateShellSourcePersistedTransform() noexcept
{
    return _shapeInteractor != nullptr
        && _shapeInteractor->debugMutateShellSourcePersistedTransform();
}

Standard_Boolean
Core3DViewer::DebugMutateShellSourcePersistedShape() noexcept
{
    return _shapeInteractor != nullptr
        && _shapeInteractor->debugMutateShellSourcePersistedShape();
}

Standard_Boolean Core3DViewer::debugBeginBevelSelection(
    const std::string& theEntityIdentifier,
    const std::vector<Standard_Size>& theEdgeTopologyIndices) noexcept {
    return debugBeginBevelSelection(
        std::vector<std::string>{theEntityIdentifier},
        std::vector<std::vector<Standard_Size>>{
            theEdgeTopologyIndices});
}

Standard_Boolean Core3DViewer::debugBeginBevelSelection(
    const std::vector<std::string>& theEntityIdentifiers,
    const std::vector<std::vector<Standard_Size>>&
        theEdgeTopologyIndices) noexcept {
    if (_shapeInteractor == nullptr || _objectInteractor == nullptr
        || myContext.IsNull() || myDoc.IsNull()
        || theEntityIdentifiers.empty()
        || theEntityIdentifiers.size() != theEdgeTopologyIndices.size()
        || theEntityIdentifiers.size()
            > BevelOperationController::kMaxSourceBodies) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        std::unordered_map<std::string, Standard_Size> anIdentifierIndices;
        Standard_Size anAggregateEdgeCount = 0;
        for (Standard_Size anIndex = 0;
             anIndex < theEntityIdentifiers.size(); ++anIndex) {
            const std::string& anIdentifier = theEntityIdentifiers[anIndex];
            const std::vector<Standard_Size>& anEdges =
                theEdgeTopologyIndices[anIndex];
            if (anIdentifier.empty() || anEdges.empty()
                || anEdges.size()
                    > BevelOperationController::kMaxSelectedEdges
                || anAggregateEdgeCount
                    > BevelOperationController::kMaxSelectedEdges
                        - anEdges.size()
                || !anIdentifierIndices.emplace(
                    anIdentifier, anIndex).second) {
                return Standard_False;
            }
            anAggregateEdgeCount += anEdges.size();
        }
        std::vector<Handle(AIS_Shape)> aPresentations(
            theEntityIdentifiers.size());
        AIS_ListOfInteractive aDisplayed;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, aDisplayed);
        for (AIS_ListIteratorOfListOfInteractive anObject(aDisplayed);
             anObject.More(); anObject.Next()) {
            const Handle(AIS_InteractiveObject)& anInteractive =
                anObject.Value();
            const TDF_Label aLabel = myDoc->ShapeLabel(anInteractive);
            if (aLabel.IsNull()) {
                continue;
            }
            const auto anIdentifier = anIdentifierIndices.find(
                myDoc->EntityIdentifierForLabel(aLabel));
            if (anIdentifier == anIdentifierIndices.end()) {
                continue;
            }
            const Handle(AIS_Shape) aCandidate =
                Handle(AIS_Shape)::DownCast(anInteractive);
            Handle(AIS_Shape)& aPresentation =
                aPresentations[anIdentifier->second];
            if (!aPresentation.IsNull() || aCandidate.IsNull()
                || !myDoc->IsPresentationEditable(aCandidate)) {
                return Standard_False;
            }
            aPresentation = aCandidate;
        }
        for (const Handle(AIS_Shape)& aPresentation : aPresentations) {
            if (aPresentation.IsNull()) {
                return Standard_False;
            }
        }
        if (!_shapeInteractor->debugBeginBevelSelection(
                aPresentations, theEdgeTopologyIndices)) {
            return Standard_False;
        }
        _objectInteractor->setManipulatorType(
            PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer);
        return _objectInteractor->getManipulatorType()
            == PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer;
    } catch (...) {
        return Standard_False;
    }
}

BevelPreviewDebugState
Core3DViewer::DebugBevelPreviewState() const noexcept
{
    return _shapeInteractor == nullptr
        ? BevelPreviewDebugState()
        : _shapeInteractor->debugBevelState();
}

void Core3DViewer::DebugSetBevelPreviewWorkerBlocked(
    const Standard_Boolean theBlocked) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetBevelWorkerBlocked(theBlocked);
    }
}

void Core3DViewer::DebugSetMaximumBevelCaptureTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetMaximumBevelCaptureTopologyNodes(
            theLimit);
    }
}

void Core3DViewer::DebugSetMaximumBevelResultTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetMaximumBevelResultTopologyNodes(
            theLimit);
    }
}

void Core3DViewer::DebugSetMaximumBevelResultSolids(
    const Standard_Size theLimit) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetMaximumBevelResultSolids(theLimit);
    }
}

void Core3DViewer::DebugSetBevelTransactionFailureCount(
    const Standard_Size theCount) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetBevelTransactionFailureCount(theCount);
    }
}

void Core3DViewer::DebugSetBevelCancelDiscardFailureCount(
    const Standard_Size theCount) noexcept
{
    if (_shapeInteractor != nullptr) {
        _shapeInteractor->debugSetBevelCancelDiscardFailureCount(theCount);
    }
}

Standard_Boolean
Core3DViewer::DebugMutateFirstBevelSourcePersistedTransform() noexcept
{
    return _shapeInteractor != nullptr
        && _shapeInteractor
            ->debugMutateFirstBevelSourcePersistedTransform();
}
#endif

void Core3DViewer::Rotation(int theX, int theY) {
    if (_documentReplacementWork || _queuedAssetLoadWork) return;
    if(_objectInteractor == nullptr) {
        return;
    }
	if (_objectInteractor->isPickingMirrorPlane()) {
		return;
	}
    if(!_objectInteractor->transformManipulator(theX, theY)){
        OcctViewer::Rotation(theX, theY);
        myContext->UpdateCurrentViewer();
    }
    
    if(_interactiveCallback != nullptr) {
        _interactiveCallback(theX, theY);
    }
}

void Core3DViewer::FinishInteraction(int theX, int theY) {
    if (_documentReplacementWork || _queuedAssetLoadWork) return;
    if(_objectInteractor != nullptr) {
		if (_objectInteractor->isPickingMirrorPlane()) {
			return;
		}
        _objectInteractor->finishInteraction();
    }
}

void Core3DViewer::CancelInteraction(int theX, int theY) {
    if (_documentReplacementWork || _queuedAssetLoadWork) return;
    if(_objectInteractor != nullptr) {
		if (_objectInteractor->isPickingMirrorPlane()) {
			return;
		}
		_objectInteractor->cancelInteraction();
    }
}

const int Core3DViewer::selectedCount() const {
    int count = 0;
    for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
        ++count;
    }
    return count;
}

void Core3DViewer::Select(int theX, int theY) {
    observeNativePlanningInteraction();
    if (_objectInteractor == nullptr || _shapeInteractor == nullptr) {
        return;
    }
	if (hasUnresolvedEdit()) {
		return;
	}
	if (_objectInteractor->isPickingMirrorPlane()) {
		(void)_objectInteractor->pickMirrorPlaneAt(theX, theY);
		redraw();
		return;
	}
    if (_objectInteractor->isBooleanSelectionFrozen()) {
        return;
    }
    if (_objectInteractor->hasActiveLinearArray()
        || _objectInteractor->hasUnresolvedLinearArray()) {
        return;
    }
    if (_objectInteractor->hasActiveRadialArray()
        || _objectInteractor->hasUnresolvedRadialArray()) {
        return;
    }
    if (_shapeInteractor->hasActiveExtrusion()) {
        return;
    }
    if (_shapeInteractor->isBevelSelectionFrozen()) {
        return;
    }

    if (_shapeInteractor->hasActiveShell()) {
        if (_objectInteractor->getManipulatorType() == PrimitiveManipulatorType::PrimitiveGizmoTypeShell
            && _shapeInteractor->shellPreviewState() == ShellPreviewState::Selecting
            && hitTest(theX, theY)) {
            (void)_shapeInteractor->toggleDetectedShellOpening();
            redraw();
        }
        return;
    }

    switch(_objectInteractor->getManipulatorType()) {
        case PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeUnion:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeIntersect:
            if (!hitTest(theX, theY)) {
                return;
            }
            break;
        default:
            break;
    }

    int oldCount = selectedCount();
    // cancel chamfer when empty tapped
    //		if (!hitTest(theX, theY) && _objectInteractor->getManipulatorType() == PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer) {
    //            _shapeInteractor->setChamferValueForSelection(0);
    //            _shapeInteractor->resetWireframeTemplateShape();
    //		}

//	bool isMirrorOp = _objectInteractor->getManipulatorType() == PrimitiveManipulatorType::PrimitiveGizmoTypeMirror;
//	OcctViewer::Select(theX, theY, isMirrorOp ? AIS_SelectionScheme::AIS_SelectionScheme_Replace : AIS_SelectionScheme::AIS_SelectionScheme_XOR);
    OcctViewer::Select(theX, theY, AIS_SelectionScheme::AIS_SelectionScheme_XOR);

    int newCount = selectedCount();

    _objectInteractor->attachManipulatorToSelection(newCount < oldCount);
    
    switch (_objectInteractor->getManipulatorType()) {
        case PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer:
            if (PREVENT_RECHAMFER) {
                // Admission failure keeps the authoritative AIS selection
                // visible so the user can remove an unsupported body and
                // retry. The Bevel controller/cache remains empty and no OCAF
                // state is changed.
                (void)_shapeInteractor->saveSelectionEdges();
            }
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract:
            _objectInteractor->updateDetectedState(true, BooleanAction::BooleanSubtract);
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeUnion:
            _objectInteractor->updateDetectedState(false, BooleanAction::BooleanUnion);
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeIntersect:
            _objectInteractor->updateDetectedState(
                false, BooleanAction::BooleanIntersect);
            break;
        default:
            break;
    }

    redraw();
}

	void Core3DViewer::deselectAll() {
        observeNativePlanningInteraction();
		if (myContext.IsNull()) { return; }
		if (_objectInteractor != nullptr
			&& hasUnresolvedEdit()) {
			return;
		}
		if (_shapeInteractor != nullptr
			&& !_shapeInteractor->cancelExtrusion()) {
			return;
		}
		if (_shapeInteractor != nullptr
			&& _shapeInteractor->hasActiveBevel()
			&& !_shapeInteractor->cancelChamfer()) {
			return;
		}
		if (_objectInteractor != nullptr) {
			if ((_objectInteractor->hasActiveRadialArray()
					|| _objectInteractor->hasUnresolvedRadialArray())
				&& !_objectInteractor->cancelRadialArray()) {
				return;
			}
			if ((_objectInteractor->hasActiveLinearArray()
					|| _objectInteractor->hasUnresolvedLinearArray())
				&& !_objectInteractor->cancelLinearArray()) {
				return;
			}
			_objectInteractor->cancelInteraction();
		}
        myContext->SelectDetected(AIS_SelectionScheme::AIS_SelectionScheme_Remove);
        myContext->ClearSelected(Standard_True);
        if (_objectInteractor == nullptr) { return; }
        if (_objectInteractor->getManipulatorType() == PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer
            && _shapeInteractor != nullptr) {
            (void)_shapeInteractor->resetWireframeTemplateShape();
        }
        redraw();
    }

    void Core3DViewer::redraw() {
        myView->Redraw();
    }

    const bool Core3DViewer::hitTest(const int x, const int y) const {
        return myContext->MoveTo(x, y, myView, Standard_False) != AIS_SOD_Nothing;
    }

    bool Core3DViewer::dumpOfDisplayedColoredObjects(const Standard_Integer width,
                                                     const Standard_Integer height,
                                                     const TCollection_AsciiString &fileName) {
        if (hasUnresolvedEdit()) { return false; }

        // prepare viewer
        Handle(Aspect_DisplayConnection) displayConnection = new Aspect_DisplayConnection();
#if TARGET_OS_OSX
        Handle(OpenGl_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection, Standard_False);
#else
        Handle(OpenGl_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection);
#endif
        Handle(V3d_Viewer) viewer = new V3d_Viewer(graphicDriver);
        viewer->SetDefaultTypeOfView(V3d_PERSPECTIVE);
        Handle(V3d_DirectionalLight) lightDir = new V3d_DirectionalLight(V3d_Zneg, Quantity_Color(Quantity_NOC_WHITE), Standard_True);
        Handle(V3d_AmbientLight)     lightAmb = new V3d_AmbientLight();
        lightDir->SetDirection(1.0, -2.0, -10.0);
        viewer->AddLight(lightDir);
        viewer->AddLight(lightAmb);
        viewer->SetLightOn(lightDir);
        viewer->SetLightOn(lightAmb);

        // prepare context
        Handle(AIS_InteractiveContext) context = new AIS_InteractiveContext(viewer);
        // prepare off-screen view
        Handle(V3d_View) view = viewer->CreateView();
        Handle(Aspect_NeutralWindow) wnd = new Aspect_NeutralWindow();
#if TARGET_OS_OSX
        NSOpenGLContext* aRendCtx = [NSOpenGLContext currentContext];
        if (aRendCtx == nil) return false;
#else
        EAGLContext* aRendCtx = [EAGLContext currentContext];
#endif
        wnd->SetSize(width, height);
        wnd->SetVirtual(true);
        view->SetWindow(wnd, aRendCtx);
        view->SetBackgroundColor(Quantity_Color(Quantity_NOC_BLACK));
        view->MustBeResized();
		//			Graphic3d_RenderingParams& aParams = view->ChangeRenderingParams();
		//			aParams.FrustumCullingState = Graphic3d_RenderingParams::FrustumCulling::FrustumCulling_On;
		//			aParams.Method = Graphic3d_RenderingMode::Graphic3d_RM_RASTERIZATION;
		//			aParams.Exposure = 2;
		//			aParams.RaytracingDepth = 3;
		//			aParams.SamplesPerPixel = 2;
		//			aParams.CoherentPathTracingMode = true;
		//			aParams.IsReflectionEnabled = false;
		//			aParams.IsTransparentShadowEnabled = true;
		//			aParams.TwoSidedBsdfModels = true;
		//			aParams.ToneMappingMethod = Graphic3d_ToneMappingMethod_Filmic;
		//			aParams.ToEnableDepthPrepass = true;
		//			aParams.TransparencyMethod = Graphic3d_RenderTransparentMethod::Graphic3d_RTM_DEPTH_PEELING_OIT;
		//			aParams.RebuildRayTracingShaders = true;
		//			aParams.IsAntialiasingEnabled = false;
		//			aParams.NbMsaaSamples = 1;
		//			aParams.ShadingModel = Graphic3d_TypeOfShadingModel::Graphic3d_TOSM_FRAGMENT;
		//			aParams.UseEnvironmentMapBackground = false;
		//			aParams.IsGlobalIlluminationEnabled = false;

        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        AIS_ListIteratorOfListOfInteractive iobject(objects);

        while (iobject.More()) {
            TopoDS_Shape shape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(iobject.Value())->Shape(),
                                                          iobject.Value()->LocalTransformation());

            // prepare presentation filled by shape
            auto color = Quantity_Color();
            Handle(AIS_Shape)::DownCast(iobject.Value())->Color(color);
            Handle(AIS_Shape) presentation = new AIS_Shape(shape);
			presentation->UnsetColor();
			auto mat = myDoc->MaterialNameForShape(Handle(AIS_Shape)::DownCast(iobject.Value()));
			presentation->SetMaterial(mat);
			presentation->SetColor(color.Name());
            context->Display(presentation, Standard_False);
            context->SetDisplayMode(presentation, AIS_Shaded, Standard_False);

            iobject.Next();
        }

        view->FitAll();
        view->Redraw();
        // prepare pixmap image
        Image_AlienPixMap img;
        if (!view->ToPixMap(img, width, height))
            return false;

#if TARGET_OS_OSX
        NSData* pngDataRep = Core3DMacPNGData(img, true);
#else
        CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
        CGBitmapInfo bitmapInfo = kCGImageByteOrderDefault;

        const auto sz = sizeof(unsigned char)*img.SizeBytes();

        CGDataProviderRef provider = CGDataProviderCreateWithData(nil, img.Data(), sz, nil);
        CGImageRef cgImageRef = CGImageCreate(width, height, 8, 24, img.SizeRowBytes(), colorSpace, bitmapInfo, provider, nil, false, (CGColorRenderingIntent)kCGRenderingIntentDefault);

        UIGraphicsBeginImageContext(CGSizeMake(width, height));
        [[UIImage imageWithCGImage:cgImageRef scale:1.0 orientation:UIImageOrientationDownMirrored] drawInRect:CGRectMake(0,0,width ,height)];
        UIImage* newImage = UIGraphicsGetImageFromCurrentImageContext();
        UIGraphicsEndImageContext();

		CIImage *ciImage = [CIImage imageWithCGImage:newImage.CGImage];
		CIFilter* filterGA = [CIFilter filterWithName:@"CIGammaAdjust"];
		[filterGA setValue:ciImage forKey:@"inputImage"];
		[filterGA setValue: [NSNumber numberWithFloat:2.0f] forKey: @"inputPower"];
		CIImage *resultImage = [filterGA valueForKey: @"outputImage"];
		UIImage* newImageGamma = [UIImage imageWithCIImage:resultImage];
		NSData* pngDataRep = UIImagePNGRepresentation(newImageGamma);

        CFRelease(provider);
        CFRelease(colorSpace);
        CFRelease(cgImageRef);

#endif

        return [pngDataRep writeToFile:[NSString stringWithCString:fileName.ToCString() encoding:NSASCIIStringEncoding] atomically:YES];
    }

    bool Core3DViewer::dumpOfDisplayedObjects(const Standard_Integer width, 
                                              const Standard_Integer height,
                                              const TCollection_AsciiString &fileName) {
        if (hasUnresolvedEdit()) { return false; }
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        AIS_ListIteratorOfListOfInteractive iobject(objects);

        TopoDS_Compound resultShape;
        BRep_Builder builder;
        builder.MakeCompound(resultShape);

        while (iobject.More()) {
            TopoDS_Shape shape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(iobject.Value())->Shape(),
                                                          iobject.Value()->LocalTransformation());
			if (!shape.IsNull())
				builder.Add(resultShape, shape);
            iobject.Next();
        }

        return dumpShape(resultShape, width, height, fileName);
    }

    bool Core3DViewer::dumpShape(const TopoDS_Shape& shape,
                                 const Standard_Integer width,
                                 const Standard_Integer height,
                                 const TCollection_AsciiString& fileName) {
        // prepare viewer
        Handle(Aspect_DisplayConnection) displayConnection = new Aspect_DisplayConnection();
#if TARGET_OS_OSX
        Handle(OpenGl_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection, Standard_False);
#else
        Handle(OpenGl_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection);
#endif
        Handle(V3d_Viewer) viewer = new V3d_Viewer(graphicDriver);
		viewer->SetDefaultTypeOfView(V3d_PERSPECTIVE);
        Handle(V3d_DirectionalLight) lightDir = new V3d_DirectionalLight(V3d_Zneg, Quantity_Color(Quantity_NOC_WHITE), Standard_True);
        Handle(V3d_AmbientLight)     lightAmb = new V3d_AmbientLight();
        lightDir->SetDirection(1.0, -2.0, -10.0);
        viewer->AddLight(lightDir);
        viewer->AddLight(lightAmb);
        viewer->SetLightOn(lightDir);
        viewer->SetLightOn(lightAmb);

        // prepare context
        Handle(AIS_InteractiveContext) context = new AIS_InteractiveContext(viewer);
        // prepare off-screen view
        Handle(V3d_View) view = viewer->CreateView();
        Handle(Aspect_NeutralWindow) wnd = new Aspect_NeutralWindow();
#if TARGET_OS_OSX
        NSOpenGLContext* aRendCtx = [NSOpenGLContext currentContext];
        if (aRendCtx == nil) return false;
#else
        EAGLContext* aRendCtx = [EAGLContext currentContext];
#endif
        wnd->SetSize(width, height);
        wnd->SetVirtual(true);
        view->SetWindow(wnd, aRendCtx);
        view->SetBackgroundColor(Quantity_Color(Quantity_NOC_BLACK));
        view->MustBeResized();
        // prepare presentation filled by shape
        Handle(AIS_Shape) presentation = new AIS_Shape(shape);
        presentation->SetMaterial(Graphic3d_NameOfMaterial_ShinyPlastified);
        presentation->SetColor(Quantity_NOC_GRAY50);
        context->Display(presentation, Standard_False);
        context->SetDisplayMode(presentation, AIS_Shaded, Standard_False);
        view->FitAll();
        view->Redraw();
        // prepare pixmap image
        Image_AlienPixMap img;
        if (!view->ToPixMap(img, width, height))
            return false;

#if TARGET_OS_OSX
        NSData* pngDataRep = Core3DMacPNGData(img, false);
#else
        CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
        CGBitmapInfo bitmapInfo = kCGImageByteOrderDefault;

        const auto sz = sizeof(unsigned char)*img.SizeBytes();

        CGDataProviderRef provider = CGDataProviderCreateWithData(nil, img.Data(), sz, nil);
        CGImageRef cgImageRef = CGImageCreate(width, height, 8, 24, img.SizeRowBytes(), colorSpace, bitmapInfo, provider, nil, false, (CGColorRenderingIntent)kCGRenderingIntentDefault);

        UIGraphicsBeginImageContext(CGSizeMake(width, height));
        [[UIImage imageWithCGImage:cgImageRef scale:1.0 orientation:UIImageOrientationDownMirrored] drawInRect:CGRectMake(0,0,width ,height)];
        UIImage* newImage = UIGraphicsGetImageFromCurrentImageContext();
        UIGraphicsEndImageContext();

        NSData* pngDataRep = UIImagePNGRepresentation(newImage);

        CFRelease(provider);
        CFRelease(colorSpace);
        CFRelease(cgImageRef);

#endif

        return [pngDataRep writeToFile:[NSString stringWithCString:fileName.ToCString() encoding:NSASCIIStringEncoding] atomically:YES];

//        // save image into a file
//        return img.Save(fileName);
    }

    bool Core3DViewer::saveSnapshot(const TCollection_AsciiString& thePath,
                                    int theWidth,
                                    int theHeight) {
        if (hasUnresolvedEdit()) { return false; }
        showGrid(false);
        myView->TriedronErase();
        myView->FitAll();
        const auto result = myView->Dump(thePath.ToCString());
        showGrid(true);
        myView->TriedronDisplay (Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.20, V3d_ZBUFFER); //FIXME: tmp solution!
        return result;

// FIXME: commented non-working code
//        if (myContext.IsNull() || thePath.IsEmpty()) {
//            Message::DefaultMessenger()->Send ("Image dump failed - view is unavailable", Message_Fail);
//            return false;
//        }
//
//        if (theWidth  < 1 || theHeight < 1) {
//            myView->Window()->Size(theWidth, theHeight);
//        }
//        if (theWidth  < 1 || theHeight < 1) {
//            Message::DefaultMessenger()->Send ("Image dump failed - view is unavailable", Message_Fail);
//            return false;
//        }
//
//        Image_AlienPixMap anAlienImage;
//        if (!anAlienImage.InitTrash(Image_Format::Image_Format_BGRA, theWidth, theHeight)) {
//            Message::DefaultMessenger()->Send (TCollection_AsciiString() + "RGBA image " + theWidth + "x" + theHeight + " allocation failed", Message_Fail);
//            return false;
//        }
//
//        // OpenGL ES does not support fetching data in BGRA format
//        // while FreeImage does not support RGBA format.
//        Image_PixMap anImage;
//        anImage.InitWrapper (Image_Format::Image_Format_BGRA,
//                             anAlienImage.ChangeData(),
//                             anAlienImage.SizeX(),
//                             anAlienImage.SizeY(),
//                             anAlienImage.SizeRowBytes());
//        if (!myView->ToPixMap (anImage, theWidth, theHeight, Graphic3d_BT_RGBA)) {
//            Message::DefaultMessenger()->Send (TCollection_AsciiString() + "View dump to the image " + theWidth + "x" + theHeight + " failed", Message_Fail);
//        }
//
//        for (Standard_Size aRow = 0; aRow < anAlienImage.SizeY(); ++aRow) {
//            for (Standard_Size aCol = 0; aCol < anAlienImage.SizeX(); ++aCol) {
//                Image_ColorRGBA& aPixel = anAlienImage.ChangeValue<Image_ColorRGBA> (aRow, aCol);
//                std::swap (aPixel.r(), aPixel.b());
//                //aPixel.a() = 1.0;
//            }
//        }
//
//        if (!anAlienImage.Save (thePath)) {
//            Message::DefaultMessenger()->Send (TCollection_AsciiString() + "Image saving to path '" + thePath + "' failed", Message_Fail);
//            return false;
//        }
//        Message::DefaultMessenger()->Send (TCollection_AsciiString() + "View " + theWidth + "x" + theHeight + " dumped to image '" + thePath + "'", Message_Info);
//        return true;
    }
}

#if DEBUG
// Runtime-only Objective-C surface keeps this diagnostic out of the public
// framework module and release binary. Swift's DEBUG wrapper owns the URL.
@interface Core3DCurvedUVDebugBridge : NSObject
- (NSDictionary<NSString *, id> *)layoutAtURL:(NSURL *)url entityIdentifier:(NSString *)identifier;
@end
@implementation Core3DCurvedUVDebugBridge
- (NSDictionary<NSString *, id> *)layoutAtURL:(NSURL *)url entityIdentifier:(NSString *)identifier {
    if (![NSThread isMainThread] || !url.isFileURL || identifier.length==0 || identifier.length>256)
        return @{@"error":@"Curved layout requires a file URL and entity on main thread"};
    try {
        struct Owner {
            Handle(OcctDocument) document = new OcctDocument();
            ~Owner() noexcept { try { document->ClosePrivateExportSnapshot(); } catch (...) {} }
        } owner;
        if (!owner.document->OpenPrivateExportSnapshot(url.fileSystemRepresentation,Message_ProgressRange()))
            return @{@"error":@"Curved layout snapshot rejected"};
        TDF_Label target; TDF_LabelSequence labels;
        XCAFDoc_DocumentTool::ShapeTool(owner.document->Document()->Main())->GetFreeShapes(labels);
        for (int i=1;i<=labels.Length();++i)
            if (owner.document->EntityIdentifierForLabel(labels.Value(i))==identifier.UTF8String) target=labels.Value(i);
        if (target.IsNull()) return @{@"error":@"Curved layout entity not found"};
        core3d::provenance::SourceFaceProvenanceRecord provenance;
        const auto state=owner.document->TryCopySourceFaceProvenanceForLabel(target,provenance);
        NSMutableDictionary* report=[@{@"provenanceState":@(int(state)),
            @"hasLayout":@(owner.document->HasCurvedUVLayoutForLabel(target))} mutableCopy];
        NSMutableArray* census=[NSMutableArray array];
        for (const auto& face:provenance.faces) {
            NSMutableArray* params=[NSMutableArray array]; for(double p:face.params) [params addObject:@(p)];
            [census addObject:@{@"surfaceType":@(face.surfaceType),@"params":params,
                @"firstTriangle":@(face.firstTriangle),@"triangleCount":@(face.triangleCount)}];
        }
        report[@"provenanceFaces"]=census;
        report[@"geometryDigest"]=[NSString stringWithUTF8String:provenance.digest.c_str()];
        if (![report[@"hasLayout"] boolValue]) return report;
        shapeyard::uv::curved::CurvedUVLayoutRecord layout; curveduv::PackSummary coverage;
        if (!owner.document->DebugCurvedUVLayoutForLabel(target,layout,coverage)) {
            report[@"error"]=@"Persisted curved layout failed digest/settings validation"; return report;
        }
        OcctObjectTransformState stored;
        if (!owner.document->CaptureObjectTransformStateForLabel(target,stored)) return @{@"error":@"Atlas state unavailable"};
        OcctMeshUVAtlasOptions options{2,stored.meshUVAtlasSettings[0],stored.meshUVAtlasSettings[1]};
        // Stored curved validation replays persisted authoring inputs. A private
        // snapshot has no original B-rep display triangulations to resolve.
        if(layout.regenerationFaces.empty()) core3d::ResolveCoherentAtlasOptions(owner.document,target,options);
        report[@"atlasValidatorAccepted"]=@(owner.document->ValidateTriangleUVAtlas(target,stored.shape,options));
        // Negative control: a private copy with one UV changed must fail the
        // same node-for-node validator. The loaded document remains untouched.
        report[@"atlasValidatorRejectsChangedUV"]=@NO;
        {
            BRepBuilderAPI_Copy copied(stored.shape,Standard_True,Standard_True);
            const auto changed=copied.Shape();
            TopExp_Explorer face(changed,TopAbs_FACE);
            if (!changed.IsNull() && !changed.IsSame(stored.shape) && face.More()) {
                TopLoc_Location location;
                const auto mesh=BRep_Tool::Triangulation(TopoDS::Face(face.Current()),location);
                if (!mesh.IsNull() && mesh->HasUVNodes() && mesh->NbTriangles()>0) {
                    const bool unchangedCopyAccepted=owner.document->ValidateTriangleUVAtlas(target,changed,options);
                    int a,b,c;mesh->Triangle(1).Get(a,b,c);
                    auto uv=mesh->UVNode(a);
                    uv.SetX(std::nextafter(uv.X(),1.0));mesh->SetUVNode(a,uv);
                    report[@"atlasValidatorRejectsChangedUV"]=@(unchangedCopyAccepted
                        && !owner.document->ValidateTriangleUVAtlas(target,changed,options));
                }
            }
        }
        report[@"version"]=@(layout.version); report[@"faceCount"]=@(layout.faceCount);
        report[@"globalTexelsPerMM"]=@(layout.globalTexelsPerMM);
        report[@"kernelSeamEdges"]=@(layout.kernelSeamEdges);
        report[@"splitSeamEdges"]=@(layout.splitSeamEdges);
        report[@"diagonalSeamEdges"]=@(layout.diagonalSeamEdges);
        report[@"tornEdges"]=@(layout.tornEdges);
        report[@"resolution"]=@(options.resolution); report[@"gutterPixels"]=@(options.gutterPixels);
        NSArray* kernels=@[@"planar",@"cylinder",@"torus",@"fallback"];
        NSMutableArray* faces=[NSMutableArray array];
        for (const auto& f:layout.faces) [faces addObject:@{
            @"kernel":kernels[int(f.kernel)],@"subChartCount":@(f.subChartCount),@"occupancy":@(f.occupancy),
            @"metricStretch":@{@"min":@(f.metricStretchMin),@"max":@(f.metricStretchMax)},
            @"seamCount":@(f.seamCount),@"uvDigest":[NSString stringWithUTF8String:f.uvDigest.c_str()],
            @"fallbackReason":[NSString stringWithUTF8String:f.fallbackReason.c_str()]}];
        report[@"faces"]=faces;
        // These are retained native measurements, not inferred seam budgets.
        // All seam categories count shared edges; splitLineCount counts cuts.
        report[@"coverage"]=@{@"subChartCount":@(coverage.subChartCount),@"splitLineCount":@(coverage.splitLineCount),
            @"occupancy":@(coverage.occupancy),@"coveredTexels":@(coverage.coveredTexels),
            @"globalTexelsPerMM":@(coverage.globalTexelsPerMM),@"continuousEdges":@(coverage.seamCounts.continuous),
            @"declaredSeams":@(coverage.seamCounts.declared),@"tornEdges":@(coverage.seamCounts.torn),
            @"kernelSeamEdges":@(coverage.seamCounts.kernelSeamEdges),
            @"splitSeamEdges":@(coverage.seamCounts.splitSeamEdges),
            @"diagonalSeamEdges":@(coverage.seamCounts.diagonalSeamEdges)};
        return report;
    } catch (...) { return @{@"error":@"Curved layout observation failed"}; }
}
@end
#endif
