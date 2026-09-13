#pragma once
// Proposed DEBUG native probe only. No bridge/registration or production caller
// is added by this package. Caller supplies a real selected saved-cut fixture.
#if DEBUG
#include "../UI/Core3DViewer.h"
#include <map>
#include <thread>

namespace core3d::saved_cut_source_viewer_probe {
// Independent native ownership evidence for the actual existing Boolean copy.
// Pass a real admitted base/envelope and settings captured from its live drawer.
inline std::map<std::string,bool> MeshIsolation(const TopoDS_Shape& base,
    const retained_solid::Envelope& envelope,const cut_display::Settings& settings) {
    std::map<std::string,bool> out;const std::atomic_bool stop(false);std::size_t bytes=0;
    saved_cut_source_edit::ShapeCommitment before,after;
    out["base-content-before"]=saved_cut_source_edit::Commit(base,stop,bytes,before);
    analytic_boolean::Result cut;
    out["actual-boolean-built"]=analytic_boolean::Build(base,cylindrical_cut::Recipe(envelope),stop,cut)==analytic_boolean::Status::Built;
    if(!out["actual-boolean-built"])return out;
    std::vector<TopoDS_Shape> baseFaces,resultFaces;
    for(TopExp_Explorer it(base,TopAbs_FACE);it.More();it.Next()){
        if(baseFaces.size()>=1024)return {{"face-budget",false}};baseFaces.push_back(it.Current());
    }
    for(TopExp_Explorer it(cut.solid,TopAbs_FACE);it.More();it.Next()){
        if(resultFaces.size()>=1024)return {{"face-budget",false}};resultFaces.push_back(it.Current());
    }
    bool separate=!baseFaces.empty()&&!resultFaces.empty();
    for(const auto& a:baseFaces)for(const auto& b:resultFaces)if(a.IsPartner(b))separate=false;
    out["no-shared-source-result-face-tshape"]=separate;
    out["actual-display-preparation"]=cut_display::Prepare(cut.solid,settings,stop);
    out["base-stream-unchanged-after-meshing"]=saved_cut_source_edit::Commit(base,stop,bytes,after)&&after==before;
    saved_cut_source_edit::ShapeCommitment result1,result2;
    out["final-result-stream-stable"]=saved_cut_source_edit::Commit(cut.solid,stop,bytes,result1)
        &&saved_cut_source_edit::Commit(cut.solid,stop,bytes,result2)&&result1==result2;
    return out;
}
inline std::map<std::string,bool> Run(Core3DViewer& viewer,const CylindricalCutSnapshot& original,
    const ObjectFrameIdentity& identity,std::uint64_t presentation,std::uint32_t width,std::uint32_t height) {
    std::map<std::string,bool> out;
    if(!NSThread.isMainThread||!original.source.rebuilding||!original.source.original.retained.value)
        return {{"fixture",false}};
    saved_cut_source_edit::Patch empty=original.source.envelope.sourceFamily==1
        ?saved_cut_source_edit::Patch(saved_cut_source_values::PolygonPatch{})
        :saved_cut_source_edit::Patch(saved_cut_source_values::EnclosurePatch{});
    const std::atomic_bool running(false);std::size_t bytes=0;
    saved_cut_source_edit::ShapeCommitment beforeBase,beforeResult;
    out["original-streams-captured"]=saved_cut_source_edit::Commit(original.source.base,running,bytes,beforeBase)
        &&saved_cut_source_edit::Commit(original.source.original.shape,running,bytes,beforeResult);
    const auto prepare=[&](const auto& lease){return viewer.prepareSavedCutSourceEdit(lease,original,empty,identity,presentation,width,height);};
    const auto build=[&](const auto& lease){
        const auto geometry=Core3DViewer::savedCutSourceEditGeometry(lease);
        std::shared_ptr<const SavedCutSourceDetachedResult> result;
        if(geometry){std::thread worker([geometry,&result]{result=Core3DViewer::buildSavedCutSourceDetached(geometry);});worker.join();}
        return result;
    };
    {
        auto lease=Core3DViewer::makeSavedCutSourceEditWork();
        const auto token=Core3DViewer::savedCutSourceEditCancellation(lease);bool stopped=false;
        std::thread worker([token,&stopped]{stopped=Core3DViewer::cancelSavedCutSourceEdit(token);});worker.join();
        out["token-stops-before-synchronous-prepare"]=stopped&&!prepare(lease);
        out["refused-lease-has-no-geometry-dispatch"]=!Core3DViewer::savedCutSourceEditGeometry(lease);
    }
    {
        auto lease=Core3DViewer::makeSavedCutSourceEditWork();
        std::weak_ptr<SavedCutSourceEditWork> weak=lease;
        auto token=Core3DViewer::savedCutSourceEditCancellation(lease);
        out["actual-lease-prepared-before-drop"]=prepare(lease);
        const auto geometry=Core3DViewer::savedCutSourceEditGeometry(lease);
        lease.reset(); // Main thread destroys all its document/presentation fields.
        out["token-does-not-retain-main-lease"]=weak.expired();
        bool stopped=false,abandoned=false;
        std::thread worker([geometry,token=std::move(token),&stopped,&abandoned]()mutable{
            abandoned=geometry&&!Core3DViewer::buildSavedCutSourceDetached(geometry);
            stopped=Core3DViewer::cancelSavedCutSourceEdit(token);token.reset();
        });worker.join();
        out["dropped-lease-cancels-real-detached-dispatch"]=abandoned;
        out["last-token-release-is-numeric-only"]=stopped&&weak.expired();
    }
    std::shared_ptr<const SavedCutSourceDetachedResult> oldResult;
    std::shared_ptr<SavedCutSourceEditCancellation> oldToken;
    {
        auto lease=Core3DViewer::makeSavedCutSourceEditWork();oldToken=Core3DViewer::savedCutSourceEditCancellation(lease);
        out["nochange-prepare"]=prepare(lease);oldResult=build(lease);
        out["nochange-native-worker-result"]=oldResult&&oldResult->isNoChange();
        out["nochange-ordinary-outcome"]=viewer.commitSavedCutSourceEdit(lease,oldResult)==OrdinaryEditResult::NoChange;
        out["nochange-exact-once"]=viewer.commitSavedCutSourceEdit(lease,oldResult)==OrdinaryEditResult::Invalid;
        out["settled-token-cannot-stop"]=!Core3DViewer::cancelSavedCutSourceEdit(oldToken);
    }
    {
        auto lease=Core3DViewer::makeSavedCutSourceEditWork();out["new-lease-prepared"]=prepare(lease);
        const auto actual=build(lease);
        out["new-native-result-issued"]=actual&&actual->isNoChange();
        out["other-job-result-refused"]=oldResult&&viewer.commitSavedCutSourceEdit(lease,oldResult)==OrdinaryEditResult::Invalid;
        out["invalid-delivery-consumes-exact-lease"]=viewer.commitSavedCutSourceEdit(lease,actual)==OrdinaryEditResult::Invalid;
    }
    {
        auto lease=Core3DViewer::makeSavedCutSourceEditWork();const auto token=Core3DViewer::savedCutSourceEditCancellation(lease);
        out["stop-lease-prepared"]=prepare(lease);const auto actual=build(lease);
        out["stop-wins-before-commit"]=actual&&Core3DViewer::cancelSavedCutSourceEdit(token)
            &&viewer.commitSavedCutSourceEdit(lease,actual)==OrdinaryEditResult::Invalid;
    }
    {
        auto lease=Core3DViewer::makeSavedCutSourceEditWork();out["later-lease-prepared"]=prepare(lease);const auto actual=build(lease);
        out["old-stop-cannot-cancel-later-lease"]=!Core3DViewer::cancelSavedCutSourceEdit(oldToken)
            &&viewer.commitSavedCutSourceEdit(lease,actual)==OrdinaryEditResult::NoChange;
    }
    saved_cut_source_edit::ShapeCommitment afterBase,afterResult;
    out["original-streams-preserved"]=saved_cut_source_edit::Commit(original.source.base,running,bytes,afterBase)
        &&saved_cut_source_edit::Commit(original.source.original.shape,running,bytes,afterResult)
        &&afterBase==beforeBase&&afterResult==beforeResult;
    return out;
}
}
#endif
