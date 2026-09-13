#pragma once
#include "SavedCutSourceEdit.hxx"
#include "AnalyticBooleanSolid.hxx"
#include "CutDisplayPreparation.hxx"
#include <memory>
#include <atomic>
class OcctDocument;
namespace core3d {
class Core3DViewer;
class OrdinaryEditController;
struct ProfileSolidGeometry;
class SavedCutSourceDetachedWork;
// Deliberately outside NativeSolidGeometryPayload: cannot enter existing commit.
class SavedCutSourceDetachedResult final {
    friend class Core3DViewer;
    friend class ::OcctDocument; // Future audited staging only; no current caller.
    friend class OrdinaryEditController; // Exact ordinary admission; no public geometry access.
    saved_cut_source_edit::Values values;
    TopoDS_Shape newBase,newResult;
    saved_cut_source_edit::ShapeCommitment sourceBase,sourceResult,
        privateOldBase,privateOldResult,generatedBase,generatedResult;
    analytic_boolean::Result cut;
    bool noChange=false;
    cut_display::Settings displaySettings;
    // The weak control block binds a result to its exact still-owned job.
    // A native result from another source-edit lease cannot be substituted.
    std::weak_ptr<SavedCutSourceDetachedWork> originatingWork;
public:
    const saved_cut_source_edit::Values& sourceValues()const noexcept{return values;}
    bool isNoChange()const noexcept{return noChange;}
    const saved_cut_source_edit::ShapeCommitment& oldBaseContent()const noexcept{return sourceBase;}
    const saved_cut_source_edit::ShapeCommitment& oldResultContent()const noexcept{return sourceResult;}
    const saved_cut_source_edit::ShapeCommitment& newBaseContent()const noexcept{return generatedBase;}
    const saved_cut_source_edit::ShapeCommitment& newResultContent()const noexcept{return generatedResult;}
    // No public constructor/factory accepts geometry, and no shape handle escapes.
private:
    SavedCutSourceDetachedResult()=default;
};
class SavedCutSourceDetachedWork final {
    friend class Core3DViewer;
    enum class Phase:unsigned {Fresh,Preparing,Prepared,Building,Finished,Refused};
    std::atomic<Phase> phase{Phase::Fresh};
    std::shared_ptr<ProfileSolidGeometry> profile;
    // Aliasing pointer owns profile while referring to its actual existing flag.
    std::shared_ptr<std::atomic_bool> stop;
    saved_cut_source_edit::Values values;
    TopoDS_Shape oldBase,oldResult;
    saved_cut_source_edit::ShapeCommitment sourceBase,sourceResult,privateBase,privateResult;
    std::size_t streamBytes=0;
    cut_display::Settings displaySettings;
    bool displayCaptured=false;
    SavedCutSourceDetachedWork()=default;
public:
    SavedCutSourceDetachedWork(const SavedCutSourceDetachedWork&)=delete;
    SavedCutSourceDetachedWork& operator=(const SavedCutSourceDetachedWork&)=delete;
};
} // namespace core3d
