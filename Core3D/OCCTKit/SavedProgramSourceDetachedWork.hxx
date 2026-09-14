#pragma once
// Detached whole-program source-rebuild values and content proof only.
// Program-specific additive path sharing the existing commit/build helpers;
// the legacy one-bore capture and command authority are not widened here.
#include "SavedBooleanProgramBuild.hxx"
#include <memory>
#include <atomic>
class OcctDocument;
namespace core3d {
class Core3DViewer;
class OrdinaryEditController;
struct ProfileSolidGeometry;
class SavedProgramSourceDetachedWork;
namespace saved_program_source_edit {
using Patch = saved_cut_source_values::Patch;
struct Values {
    retained_boolean::Program oldProgram,newProgram;
    std::vector<std::uint8_t> oldBytes,newBytes;
    bool changed=false;
};
// Value-only preparation for the COMPLETE original program. Only the shared
// source values may move; every stable ID, operation, order, raw axis/center/
// radius, high-water ID, identity UUID, unit and schema bit is proven exact.
// Hole clearance is revalidated for every operand view on old and new source
// before any geometry work; the complete boundary is proven separately by the
// detached build and the paired stage/seal readbacks. Not a permit.
inline bool PrepareValues(const retained_boolean::Recipe& original,const Patch& patch,
    const std::atomic_bool& stop,Values& out)noexcept {
    out={};try {
        const auto* program=std::get_if<retained_boolean::Program>(&original);
        if(stop.load()||!program||!retained_boolean::Valid(*program))return false;
        std::vector<std::uint8_t> oldBytes;
        if(!retained_boolean::Encode(*program,oldBytes))return false;
        const auto applied=saved_boolean_build::SourcePatch(*program,patch);
        if(stop.load()||!applied||applied->oldBytes!=oldBytes)return false;
        const auto* changedProgram=std::get_if<retained_boolean::Program>(&applied->recipe);
        if(!changedProgram||!retained_boolean::Valid(*changedProgram))return false;
        Values v;v.oldProgram=*program;v.newProgram=*changedProgram;
        v.oldBytes=std::move(oldBytes);
        std::vector<std::uint8_t> newBytes;
        if(!retained_boolean::Encode(v.newProgram,newBytes)||newBytes!=applied->newBytes)return false;
        v.newBytes=std::move(newBytes);v.changed=applied->changed;
        if(v.changed!=(v.oldBytes!=v.newBytes))return false;
        // Exact fixed-program check independent of any regenerated DTO.
        auto fixed=v.newProgram;fixed.source.values=v.oldProgram.source.values;
        std::vector<std::uint8_t> fixedBytes;
        if(!retained_boolean::Encode(fixed,fixedBytes)||fixedBytes!=v.oldBytes)return false;
        // Every operand view keeps a clear recipe disk on old AND new source.
        for(std::size_t i=0;i<v.oldProgram.steps.size();++i){
            if(stop.load())return false;
            if(saved_cut_bore_clearance::Inspect(saved_boolean_result::detail::GeometryView(v.oldProgram,i)).status
                !=saved_cut_bore_clearance::Status::ClearRecipeDisk)return false;
            if(saved_cut_bore_clearance::Inspect(saved_boolean_result::detail::GeometryView(v.newProgram,i)).status
                !=saved_cut_bore_clearance::Status::ClearRecipeDisk)return false;
        }
        if(stop.load())return false;
        out=std::move(v);return true;
    }catch(...){out={};return false;}
}
} // namespace core3d::saved_program_source_edit
// Deliberately outside NativeSolidGeometryPayload: cannot enter existing commit.
class SavedProgramSourceDetachedResult final {
    friend class Core3DViewer;
    friend class ::OcctDocument; // Audited paired staging/seal readbacks only.
    friend class OrdinaryEditController; // Exact ordinary admission; no public geometry access.
    saved_program_source_edit::Values values;
    TopoDS_Shape newBase,newResult;
    saved_cut_source_edit::ShapeCommitment sourceBase,sourceResult,
        privateOldBase,privateOldResult,generatedBase,generatedResult;
    // Complete-program build receipt: final shape, retained-base/final-result
    // commitments, full oriented-boundary correspondence and aggregate budget.
    saved_boolean_build::Result build;
    bool noChange=false;
    cut_display::Settings displaySettings;
    // The weak control block binds a result to its exact still-owned job.
    // A native result from another source-edit lease cannot be substituted.
    std::weak_ptr<SavedProgramSourceDetachedWork> originatingWork;
public:
    const saved_program_source_edit::Values& sourceValues()const noexcept{return values;}
    bool isNoChange()const noexcept{return noChange;}
    const saved_cut_source_edit::ShapeCommitment& oldBaseContent()const noexcept{return sourceBase;}
    const saved_cut_source_edit::ShapeCommitment& oldResultContent()const noexcept{return sourceResult;}
    const saved_cut_source_edit::ShapeCommitment& newBaseContent()const noexcept{return generatedBase;}
    const saved_cut_source_edit::ShapeCommitment& newResultContent()const noexcept{return generatedResult;}
    // No public constructor/factory accepts geometry, and no shape handle escapes.
private:
    SavedProgramSourceDetachedResult()=default;
};
class SavedProgramSourceDetachedWork final {
    friend class Core3DViewer;
    enum class Phase:unsigned {Fresh,Preparing,Prepared,Building,Finished,Refused};
    std::atomic<Phase> phase{Phase::Fresh};
    std::shared_ptr<ProfileSolidGeometry> profile;
    // Aliasing pointer owns profile while referring to its actual existing flag.
    std::shared_ptr<std::atomic_bool> stop;
    saved_program_source_edit::Values values;
    TopoDS_Shape oldBase,oldResult;
    saved_cut_source_edit::ShapeCommitment sourceBase,sourceResult,privateBase,privateResult;
    // One aggregate occurrence/stream/step budget across old capture, private
    // copies and the complete rebuild; no fresh per-substep allowance.
    saved_boolean_build::Budget budget;
    cut_display::Settings displaySettings;
    bool displayCaptured=false;
    SavedProgramSourceDetachedWork()=default;
public:
    SavedProgramSourceDetachedWork(const SavedProgramSourceDetachedWork&)=delete;
    SavedProgramSourceDetachedWork& operator=(const SavedProgramSourceDetachedWork&)=delete;
};
} // namespace core3d
