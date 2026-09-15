
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
#include "OrdinaryEditController.hpp"
#include "../OCCTKit/SweepRebuildDefinition.hxx"
#include "../OCCTKit/CurrentTessellationMeshCopy.hxx"
#include "../OCCTKit/NativeMeshTopologyCapture.hxx"
#include "../Common/Core3DMobileResourceLimits.h"
#include "../OCCTKit/AuthoredFrameAttributeID.hxx"
#import <Foundation/Foundation.h>
#include <gp_Quaternion.hxx>
#include <Standard_Failure.hxx>
#include <Standard_GUID.hxx>
#include <cmath>
#include <algorithm>
#include <limits>
#include <unordered_set>
#include <utility>
#include <type_traits>
#include <new>
#include <TDF_Tool.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Integer.hxx>
#include <XCAFDoc_DocumentTool.hxx>

namespace core3d {
struct SweepRebuildGuard {
    struct Entry {
        OcctObjectVisibilityState visibility;
        OcctScalarAppearanceState appearance;
        OcctReferenceAxis axis;
        OcctReferenceAxisReadState axisState;
    };
    using Catalog=std::map<std::string,Entry>;
    Catalog previous,candidate;
    OcctSavedGroupState groups;
};
namespace {

std::string CreationLabelKey(const TDF_Label& label) {
    TCollection_AsciiString entry;
    TDF_Tool::Entry(label, entry);
    return entry.ToCString();
}
bool CaptureCreationRoots(const Handle(OcctDocument)& owner, OrdinaryCreationCatalog& roots) {
    roots.clear();
    const auto document = owner.IsNull() ? Handle(TDocStd_Document)() : owner->Document();
    if (document.IsNull()) { return false; }
    const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
    if (shapes.IsNull()) { return false; }
    TDF_LabelSequence labels;
    shapes->GetFreeShapes(labels);
    if (labels.Length() > 50000) { return false; }
    for (Standard_Integer index = 1; index <= labels.Length(); ++index) {
        const auto& label = labels.Value(index);
        const auto shape = XCAFDoc_ShapeTool::GetShape(label);
        if (label.IsNull() || label.Data() != document->GetData() || shape.IsNull()) { return false; }
        profile::Record storedProfile;
        enclosure::Record storedEnclosure;
        sweep_persistence::Record storedSweep;loft_persistence::Record storedLoft;
        if (!profile::Read(document, label, storedProfile)
            || !enclosure::Read(document, label, storedEnclosure)
            || !sweep_persistence::Read(document, label, storedSweep)
            || !loft_persistence::Read(document,label,storedLoft)) return false;
        if (!roots.emplace(CreationLabelKey(label), OrdinaryCreationRoot{label, shape,
                owner->EntityIdentifierForLabel(label), owner->DefinitionIdentifierForLabel(label),
                owner->GeometryRepresentationForLabel(label), storedProfile, storedEnclosure, storedSweep, storedLoft}).second) { return false; }
    }
    return true;
}
bool CreationRootsEqual(const OrdinaryCreationRoot& a, const OrdinaryCreationRoot& b) {
    return a.label.IsEqual(b.label) && a.label.Data() == b.label.Data()
        && a.shape.IsEqual(b.shape) && a.entityIdentifier == b.entityIdentifier
        && a.definitionIdentifier == b.definitionIdentifier && a.representation == b.representation && a.profile.IsEqual(b.profile)
        && a.enclosure.IsEqual(b.enclosure) && a.sweep.IsEqual(b.sweep) && a.loft.IsEqual(b.loft);
}
bool CaptureSweepEditCatalog(const Handle(OcctDocument)& owner,
    SweepRebuildGuard::Catalog& output,OcctSavedGroupState& groups) {
    output.clear();OrdinaryCreationCatalog roots;
    if (owner.IsNull() || !owner->ValidateGeometryRepresentations()
        || !CaptureCreationRoots(owner,roots) || !owner->CaptureSavedGroups(groups)) return false;
    for (const auto& [key,root]:roots) {
        SweepRebuildGuard::Entry entry;
        if (!owner->CaptureObjectVisibilityStateForLabel(root.label,entry.visibility)
            || !owner->CaptureScalarAppearanceForSavedSweepRebuild(root.label,entry.appearance)) return false;
        entry.axisState=owner->ReadReferenceAxisForLabel(root.label,entry.axis);
        if (entry.axisState==OcctReferenceAxisReadState::Invalid || !output.emplace(key,std::move(entry)).second) return false;
    }
    return true;
}
bool SweepEditEntryEqual(const SweepRebuildGuard::Entry& a,const SweepRebuildGuard::Entry& b) {
    if (!a.visibility.IsEqual(b.visibility) || !a.appearance.IsEqual(b.appearance)
        || !sweep_rebuild::SameRawScalars(a.visibility.object.object.scalars,b.visibility.object.object.scalars)
        || !sweep_rebuild::SameRawScalars(a.appearance.visualValues,b.appearance.visualValues)
        || a.axisState!=b.axisState || a.axis.pivotSpace!=b.axis.pivotSpace
        || a.axis.directionSpace!=b.axis.directionSpace) return false;
    for (int i=1;i<=3;++i)
        if (sweep_persistence::Bits(a.axis.pivot.Coord(i))!=sweep_persistence::Bits(b.axis.pivot.Coord(i))
            || sweep_persistence::Bits(a.axis.direction.Coord(i))!=sweep_persistence::Bits(b.axis.direction.Coord(i))) return false;
    return true;
}
bool SweepEditCatalogEqual(const SweepRebuildGuard::Catalog& a,const SweepRebuildGuard::Catalog& b) {
    if (a.size()!=b.size()) return false;
    for (const auto& [key,value]:a) {
        const auto found=b.find(key);if(found==b.end() || !SweepEditEntryEqual(value,found->second)) return false;
    }
    return true;
}
bool CutGuardMatches(const Handle(OcctDocument)& owner,const OrdinaryTransformLedger& ledger,bool candidate) {
    if(!ledger.cutPrevious)return !ledger.cutCandidate;
    const auto& expected=candidate?ledger.cutCandidate:ledger.cutPrevious;
    return !owner.IsNull()&&expected&&owner->SavedCutSceneStateMatches(expected);
}
bool SweepGuardMatches(const Handle(OcctDocument)& owner,const OrdinaryTransformLedger& ledger,bool candidate) {
    if (!ledger.sweepGuard) return true;
    SweepRebuildGuard::Catalog live;OcctSavedGroupState groups;
    return CaptureSweepEditCatalog(owner,live,groups) && groups.IsEqual(ledger.sweepGuard->groups)
        && SweepEditCatalogEqual(live,candidate?ledger.sweepGuard->candidate:ledger.sweepGuard->previous);
}
bool SealSweepCandidate(const Handle(OcctDocument)& owner,OrdinaryTransformLedger& ledger) {
    if (!ledger.sweepGuard) return true;
    if (ledger.records.size()!=1 || (bool(ledger.records.front().requested.sweepRebuild)
        ==bool(ledger.records.front().requested.loftRebuild))) return false;
    auto& guard=*ledger.sweepGuard;OcctSavedGroupState groups;
    if (!CaptureSweepEditCatalog(owner,guard.candidate,groups) || !groups.IsEqual(guard.groups)
        || guard.previous.size()!=guard.candidate.size()) return false;
    const auto& record=ledger.records.front();const auto key=CreationLabelKey(record.previous.label);
    std::vector<double> values;
    if (record.requested.loftRebuild) {if(!loft_persistence::Encode(*record.requested.loftRebuild,values))return false;}
    else if (!sweep_persistence::Encode(*record.requested.sweepRebuild,values)) return false;
    bool sawTarget=false;
    for (const auto& [oldKey,oldEntry]:guard.previous) {
        const auto found=guard.candidate.find(oldKey);if(found==guard.candidate.end()) return false;
        if (oldKey!=key) {if(!SweepEditEntryEqual(oldEntry,found->second))return false;continue;}
        sawTarget=true;auto normalized=found->second;
        const auto& before=oldEntry.visibility.object.object;
        auto& after=normalized.visibility.object.object;
        if (!after.shape.IsEqual(record.requested.shape)) return false;
        if (record.requested.loftRebuild) {
            if(!after.loft.label.IsEqual(before.loft.label) || after.loft.identifier!=before.loft.identifier
                || !loft_persistence::SameBits(after.loft.values,values)
                || !after.loft.IsCurrent(owner->Document(),record.previous.label))return false;
            after.loft=before.loft;
        } else {
            if(!after.sweep.label.IsEqual(before.sweep.label) || after.sweep.identifier!=before.sweep.identifier
                || !sweep_persistence::SameBits(after.sweep.values,values)
                || !after.sweep.IsCurrent(owner->Document(),record.previous.label))return false;
            after.sweep=before.sweep;
        }
        after.shape=before.shape;
        if (!SweepEditEntryEqual(oldEntry,normalized)) return false;
    }
    return sawTarget;
}

bool CreationIntegerEquals(const TDF_Label& label, Standard_Integer tag, Standard_Integer expected) {
    const auto child = label.FindChild(tag, Standard_False);
    Handle(TDataStd_Integer) value;
    return !child.IsNull() && child.FindAttribute(TDataStd_Integer::GetID(), value)
        && !value.IsNull() && value->Get() == expected;
}

bool MatricesEqual(const gp_Trsf& a, const gp_Trsf& b) {
    for (int row = 1; row <= 3; ++row) {
        for (int column = 1; column <= 4; ++column) {
            if (!std::isfinite(a.Value(row, column))
                || a.Value(row, column) != b.Value(row, column)) { return false; }
        }
    }
    return true;
}
// Admission accounts only for bounded floating-point arithmetic in composing
// a rotation with a persisted transform. Candidate sealing/readback below
// still compares exact persisted values, never this tolerance.
bool RotationArithmeticEqual(double a, double b, double arithmeticScale) {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(arithmeticScale)
        && std::abs(a - b) <= 64 * std::numeric_limits<double>::epsilon()
            * std::max({1.0, std::abs(a), std::abs(b), arithmeticScale});
}
bool IsRotationAroundPivot(const OrdinaryRotationAroundPivot& rotation) {
    if (rotation.delta.ScaleFactor() != 1.0) { return false; }
    for (int index = 1; index <= 3; ++index) {
        if (!std::isfinite(rotation.pivot.Coord(index))
            || std::abs(rotation.pivot.Coord(index)) > limits::kMaximumModelCoordinateMagnitude) { return false; }
        for (int column = 1; column <= 4; ++column) {
            if (!std::isfinite(rotation.delta.Value(index, column))) { return false; }
        }
    }
    const gp_Pnt fixed = rotation.pivot.Transformed(rotation.delta);
    for (int index = 1; index <= 3; ++index) {
        // Near-zero coordinates can result from subtracting large terms.
        const double scale = std::max({1.0, std::abs(rotation.pivot.X()),
                                       std::abs(rotation.pivot.Y()), std::abs(rotation.pivot.Z())});
        if (!std::isfinite(fixed.Coord(index))
            || std::abs(fixed.Coord(index) - rotation.pivot.Coord(index))
                > 64 * std::numeric_limits<double>::epsilon() * scale) { return false; }
    }
    return true;
}
std::array<Standard_Real, 8> EncodedTransform(const gp_Trsf& transform) {
    const gp_Quaternion q = transform.GetRotation();
    return {{transform.TranslationPart().X(), transform.TranslationPart().Y(),
             transform.TranslationPart().Z(), q.X(), q.Y(), q.Z(), q.W(), transform.ScaleFactor()}};
}
bool CandidateIsFinite(const gp_Trsf& transform) {
    const auto values = EncodedTransform(transform);
    for (double value : values) { if (!std::isfinite(value)) { return false; } }
    for (int index = 0; index < 3; ++index) {
        if (std::abs(values[index]) > limits::kMaximumModelCoordinateMagnitude) { return false; }
    }
    return std::abs(values[7]) > std::numeric_limits<Standard_Real>::epsilon();
}
bool TransformGroupsMatch(const Handle(OcctDocument)& owner,const OrdinaryTransformLedger& ledger,bool candidate) {
    OcctSavedGroupState actual;
    return !owner.IsNull() && owner->CaptureSavedGroups(actual)
        && actual.IsEqual(candidate?ledger.groupsCandidate:ledger.groupsPrevious);
}
bool PrepareCollectiveGroupOrigin(const Handle(OcctDocument)& owner,OrdinaryTransformLedger& ledger) {
    if(owner.IsNull()||!owner->CaptureSavedGroups(ledger.groupsPrevious))return false;
    ledger.groupsRequested=ledger.groupsPrevious;ledger.groupsCandidate=ledger.groupsPrevious;
    const bool any=std::any_of(ledger.records.begin(),ledger.records.end(),[](const auto& r){return bool(r.requested.collectiveWorldDelta);});
    if(!any)return true;
    const auto& delta=ledger.records.front().requested.collectiveWorldDelta;
    if(!delta||!CandidateIsFinite(*delta))return false;
    std::unordered_set<std::string> labels;
    for(const auto& record:ledger.records){
        if(!record.requested.collectiveWorldDelta||!MatricesEqual(*delta,*record.requested.collectiveWorldDelta)
            ||(record.requested.operation!=OrdinaryTransformOperation::Translate
                &&record.requested.operation!=OrdinaryTransformOperation::Rotate
                &&record.requested.operation!=OrdinaryTransformOperation::Scale)
            ||!labels.insert(CreationLabelKey(record.previous.label)).second)return false;
        // The caller explicitly marks a collective operation. Bind it to the
        // already admitted operation inputs; never infer collectivity from
        // coincident independent numeric edits or inverse-roundtrip matrices.
        if(record.requested.operation==OrdinaryTransformOperation::Rotate) {
            if(!record.requested.rotationAroundPivot
                ||!MatricesEqual(*delta,record.requested.rotationAroundPivot->delta))return false;
        } else if(record.requested.operation==OrdinaryTransformOperation::Translate) {
            for(int row=1;row<=3;++row)for(int column=1;column<=3;++column)
                if(delta->Value(row,column)!=(row==column?1.0:0.0))return false;
            for(int row=1;row<=3;++row) {
                const double previous=record.previous.transform.Value(row,4),change=delta->Value(row,4);
                if(!RotationArithmeticEqual(previous+change,record.requested.transform.Value(row,4),
                    std::abs(previous)+std::abs(change)))return false;
            }
        } else {
            const gp_Trsf expected=(*delta)*record.previous.transform;
            for(int row=1;row<=3;++row)for(int column=1;column<=4;++column) {
                double arithmeticScale=std::abs(delta->Value(row,4));
                if(column!=4)arithmeticScale=0.0;
                for(int term=1;term<=3;++term)arithmeticScale+=std::abs(delta->Value(row,term)*record.previous.transform.Value(term,column));
                if(!RotationArithmeticEqual(expected.Value(row,column),record.requested.transform.Value(row,column),arithmeticScale))return false;
            }
        }
    }
    for(auto& group:ledger.groupsRequested.groups){
        if(!group.originPresent||group.members.size()!=labels.size())continue;
        bool exact=true;for(const auto& member:group.members)exact=exact&&labels.count(CreationLabelKey(member));
        if(!exact)continue;
        const gp_Pnt before=group.origin;gp_Pnt moved=before.Transformed(*delta);
        if(!OcctDocument::IsAdmittedSavedGroupOrigin(moved))return false;
        group.origin=moved;ledger.groupOriginChanges=!moved.IsEqual(before,0.0);
        return true;
    }
    return true; // partial or unrelated collective edits leave authored origins fixed.
}
}

OrdinaryEditLease::OrdinaryEditLease(std::weak_ptr<OrdinaryEditController> controller,
                                   std::uint64_t token,
                                   std::shared_ptr<const std::uint8_t> lifetime) noexcept
    : _controller(std::move(controller)), _lifetime(std::move(lifetime)), _token(token) {}
OrdinaryEditLease::OrdinaryEditLease(OrdinaryEditLease&& other) noexcept
    : _controller(std::move(other._controller)), _lifetime(std::move(other._lifetime)),
      _token(std::exchange(other._token, 0)) {}
OrdinaryEditLease& OrdinaryEditLease::operator=(OrdinaryEditLease&& other) noexcept {
    if (this != &other) {
        (void)cancel();
        _controller = std::move(other._controller);
        _lifetime = std::move(other._lifetime);
        _token = std::exchange(other._token, 0);
    }
    return *this;
}
OrdinaryEditLease::~OrdinaryEditLease() noexcept { (void)cancel(); }
OrdinaryEditResult OrdinaryEditLease::stageAndCommit() noexcept {
    const auto controller = _controller.lock();
    if (!controller || _token == 0) { return OrdinaryEditResult::Invalid; }
    // Wrong-thread rejection must not consume the main-thread lease.
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    const auto token = std::exchange(_token, 0);
    const auto result = controller->stageAndCommit(token);
    _lifetime.reset();
    return result;
}
OrdinaryEditResult OrdinaryEditLease::cancel() noexcept {
    const auto controller = _controller.lock();
    if (!controller || _token == 0) { return OrdinaryEditResult::NoChange; }
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    const auto result = controller->cancel(std::exchange(_token, 0));
    _lifetime.reset();
    return result;
}

OrdinaryEditController::OrdinaryEditController(Handle(OcctDocument) document,
                                             OrdinaryEditPresentationHost& host)
    : _document(std::move(document)), _host(host) {}

std::shared_ptr<const SweepRebuildGuard> OrdinaryEditController::captureSavedSweepRebuildSource() const noexcept {
    if (!NSThread.isMainThread || blocksNormalWork() || _document.IsNull()
        || _document->Document().IsNull() || _document->Document()->HasOpenCommand()) return {};
    try {
        auto source=std::make_shared<SweepRebuildGuard>();
        if(!CaptureSweepEditCatalog(_document,source->previous,source->groups))return {};
        return source;
    }catch(...){return {};}
}
bool OrdinaryEditController::savedSweepRebuildSourceIsCurrent(
    const std::shared_ptr<const SweepRebuildGuard>& source) const noexcept {
    if(!NSThread.isMainThread || !source || !source->candidate.empty())return false;
    try {
        SweepRebuildGuard::Catalog current;OcctSavedGroupState groups;
        return CaptureSweepEditCatalog(_document,current,groups) && groups.IsEqual(source->groups)
            && SweepEditCatalogEqual(current,source->previous);
    }catch(...){return false;}
}

bool OrdinaryEditController::blocksNormalWork() const noexcept {
    return ![NSThread isMainThread] || _entering || _reconciling
        || _state != OrdinaryEditState::Idle || _pending.has_value() || _command.isRetained();
}

OrdinaryEditLease OrdinaryEditController::beginTransform(
    const std::vector<OrdinaryTransformChange>& changes, OrdinaryEditResult* failure) noexcept {
    return beginTransformImpl(changes, failure, {});
}
OrdinaryEditLease OrdinaryEditController::beginModelingRebuild(const OrdinaryTransformChange& change,
    std::shared_ptr<NativeModelingCommitPermit> permit, OrdinaryEditResult* failure) noexcept {
    if (!permit) { if (failure) *failure=OrdinaryEditResult::Invalid; return {}; }
    try { return beginTransformImpl({change}, failure, std::move(permit)); }
    catch (...) { if (failure) *failure=OrdinaryEditResult::Invalid; return {}; }
}
OrdinaryEditLease OrdinaryEditController::beginModelingPlacement(const OrdinaryTransformChange& change,
    std::shared_ptr<NativeModelingCommitPermit> permit, OrdinaryEditResult* failure) noexcept {
    if(!permit||permit->operation_!=receipt::Operation::SetPlacement||!change.placementContinuation){
        if(failure)*failure=OrdinaryEditResult::Invalid;return {};
    }
    try{return beginTransformImpl({change},failure,std::move(permit));}
    catch(...){if(failure)*failure=OrdinaryEditResult::Invalid;return {};}
}
OrdinaryEditLease OrdinaryEditController::beginTransformImpl(
    const std::vector<OrdinaryTransformChange>& changes, OrdinaryEditResult* failure,
    std::shared_ptr<NativeModelingCommitPermit> permit) noexcept
{
    const auto reject = [&](OrdinaryEditResult reason) {
        if (failure) { *failure = reason; }
        return OrdinaryEditLease();
    };
    if (![NSThread isMainThread]) { return reject(OrdinaryEditResult::Invalid); }
    if (blocksNormalWork()) { return reject(OrdinaryEditResult::Busy); }
    if (_document.IsNull() || changes.empty() || changes.size() > 1024
        || _nextToken == std::numeric_limits<std::uint64_t>::max()) {
        return reject(OrdinaryEditResult::Invalid);
    }
    _entering = true;
    struct EnterReset { bool& flag; ~EnterReset() { flag = false; } } reset{_entering};
    try {
        const auto self = shared_from_this();
        const auto lifetime = std::make_shared<const std::uint8_t>(0);
        const auto document = _document->Document();
        if (document.IsNull() || document->HasOpenCommand()) { return reject(OrdinaryEditResult::Busy); }
        OrdinaryTransformLedger ledger;
        ledger.records.reserve(changes.size());
        if (permit && changes.size()!=1) return reject(OrdinaryEditResult::Invalid);
        std::unordered_set<std::string> entities;
        std::unordered_set<const AIS_Shape*> presentations;
        bool changed = false, sweepNoChange = false;
        for (const auto& request : changes) {
            OrdinaryTransformRecord record;
            if (request.operation != OrdinaryTransformOperation::Translate
                && request.operation != OrdinaryTransformOperation::Rotate
                && request.operation != OrdinaryTransformOperation::Scale
                && request.operation != OrdinaryTransformOperation::MeshUVAtlas
                && request.operation != OrdinaryTransformOperation::MeshVertexMove
                && request.operation != OrdinaryTransformOperation::MeshRegionExtrude
                && request.operation != OrdinaryTransformOperation::MeshRegionInset
                && request.operation != OrdinaryTransformOperation::MeshWindingRepair
                && request.operation != OrdinaryTransformOperation::ProfileRebuild
                && request.operation != OrdinaryTransformOperation::EnclosureRebuild
                && request.operation != OrdinaryTransformOperation::SweepRebuild
                && request.operation != OrdinaryTransformOperation::LoftStationRebuild
                && request.operation != OrdinaryTransformOperation::CylindricalCut
                && request.operation != OrdinaryTransformOperation::CylindricalCutSourceRebuild
                && request.operation != OrdinaryTransformOperation::CylindricalCutProgramSourceRebuild) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.profileRebuild.has_value() != (request.operation == OrdinaryTransformOperation::ProfileRebuild)) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.enclosureRebuild.has_value() != (request.operation == OrdinaryTransformOperation::EnclosureRebuild)) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.sweepRebuild.has_value() != (request.operation == OrdinaryTransformOperation::SweepRebuild)
                || bool(request.sweepSource)!=(request.operation==OrdinaryTransformOperation::SweepRebuild
                    || request.operation==OrdinaryTransformOperation::LoftStationRebuild)
                || request.loftRebuild.has_value()!=(request.operation==OrdinaryTransformOperation::LoftStationRebuild)
                || request.loftStationEdit.has_value()!=(request.operation==OrdinaryTransformOperation::LoftStationRebuild)) {
                return reject(OrdinaryEditResult::Invalid);
            }
            const bool sourceRebuild=request.operation==OrdinaryTransformOperation::CylindricalCutSourceRebuild;
            const bool programSourceRebuild=request.operation==OrdinaryTransformOperation::CylindricalCutProgramSourceRebuild;
            if(bool(request.cut)!=(request.operation==OrdinaryTransformOperation::CylindricalCut)
                ||bool(request.cutSource)!=(request.operation==OrdinaryTransformOperation::CylindricalCut||sourceRebuild||programSourceRebuild)
                ||request.cutSourcePatch.has_value()!=sourceRebuild||bool(request.cutSourceRebuild)!=sourceRebuild
                ||request.cutProgramSourcePatch.has_value()!=programSourceRebuild
                ||bool(request.cutProgramSourceRebuild)!=programSourceRebuild
                ||(request.cutProgramEdit.has_value()&&request.operation!=OrdinaryTransformOperation::CylindricalCut)
                ||(sourceRebuild&&(permit||changes.size()!=1))
                ||(programSourceRebuild&&(permit||changes.size()!=1)))
                return reject(OrdinaryEditResult::Invalid);
            if (request.meshVertexMove.has_value() != (request.operation == OrdinaryTransformOperation::MeshVertexMove)
                || request.meshRegionExtrude.has_value() != (request.operation == OrdinaryTransformOperation::MeshRegionExtrude)
                || request.meshRegionInset.has_value() != (request.operation == OrdinaryTransformOperation::MeshRegionInset)) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.presentation.IsNull() || request.shape.IsNull()
                || !CandidateIsFinite(request.transform)
                || !_document->CaptureObjectTransformStateForLabel(request.label, record.previous)
                || !entities.insert(record.previous.entityIdentifier).second
                || !presentations.insert(request.presentation.get()).second) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if(bool(request.placementContinuation)!=(permit&&permit->operation_==receipt::Operation::SetPlacement))
                return reject(OrdinaryEditResult::Invalid);
            if (permit && !(permit->operation_==receipt::Operation::SetPlacement
                ?bindPlacementReceipt(ledger,request,record.previous,permit)
                :bindRebuildReceipt(ledger, request, record.previous, permit)))
                return reject(OrdinaryEditResult::Invalid);
            const auto unchanged = [&]() {
                if (permit) {
                    record.requested=request;
                    ledger.records.push_back(record);
                    if (!permit->current() || document->HasOpenCommand()
                        || !captureMatches(record.previous) || !rebuildReceiptMatches(ledger,false))
                        return reject(OrdinaryEditResult::Invalid);
                    // No command was opened: no receipt-only Undo or durable
                    // commit claim. The permanent reservation still forbids replay.
                    permit->resolution_->state_=NativeModelingReceiptResolution::State::Unchanged;
                }
                return reject(OrdinaryEditResult::NoChange);
            };
            if (request.rotationAroundPivot.has_value() != changes.front().rotationAroundPivot.has_value()) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.rotationAroundPivot) {
                const auto& rotation = *request.rotationAroundPivot;
                const auto& common = *changes.front().rotationAroundPivot;
                if (request.operation != OrdinaryTransformOperation::Rotate || !IsRotationAroundPivot(rotation)
                    || !rotation.pivot.IsEqual(common.pivot, 0.0)
                    || !MatricesEqual(rotation.delta, common.delta)) { return reject(OrdinaryEditResult::Invalid); }
            }
            if (request.operation == OrdinaryTransformOperation::Translate) {
                for (int row = 1; row <= 3; ++row) {
                    for (int column = 1; column <= 3; ++column) {
                        if (record.previous.transform.Value(row, column) != request.transform.Value(row, column)) {
                            return reject(OrdinaryEditResult::Invalid);
                        }
                    }
                }
            } else if (request.operation == OrdinaryTransformOperation::Rotate) {
                if (record.previous.transform.ScaleFactor() != request.transform.ScaleFactor()) {
                    return reject(OrdinaryEditResult::Invalid);
                }
                if (request.rotationAroundPivot) {
                    const gp_Trsf expected = request.rotationAroundPivot->delta * record.previous.transform;
                    for (int row = 1; row <= 3; ++row) {
                        for (int column = 1; column <= 4; ++column) {
                            // Bound cancellation by the actual products being summed,
                            // including the pivot translation for the position column.
                            const gp_Trsf& delta = request.rotationAroundPivot->delta;
                            double arithmeticScale = column == 4 ? std::abs(delta.Value(row, 4)) : 0.0;
                            for (int term = 1; term <= 3; ++term) {
                                arithmeticScale += std::abs(delta.Value(row, term)
                                    * record.previous.transform.Value(term, column));
                            }
                            if (!RotationArithmeticEqual(expected.Value(row, column),
                                                         request.transform.Value(row, column), arithmeticScale)) {
                                return reject(OrdinaryEditResult::Invalid);
                            }
                        }
                    }
                } else {
                    for (int row = 1; row <= 3; ++row) {
                        if (record.previous.transform.Value(row, 4) != request.transform.Value(row, 4)) {
                            return reject(OrdinaryEditResult::Invalid);
                        }
                    }
                }
            }
            const bool geometryChanges = !record.previous.shape.IsEqual(request.shape);
            if (!record.previous.loft.label.IsNull() && geometryChanges
                && request.operation!=OrdinaryTransformOperation::LoftStationRebuild) return reject(OrdinaryEditResult::Invalid);
            if (!record.previous.sweep.label.IsNull() && geometryChanges
                && request.operation!=OrdinaryTransformOperation::SweepRebuild)
                return reject(OrdinaryEditResult::Invalid);
            if (geometryChanges && request.operation != OrdinaryTransformOperation::Scale
                && request.operation != OrdinaryTransformOperation::MeshUVAtlas
                && request.operation != OrdinaryTransformOperation::MeshVertexMove
                && request.operation != OrdinaryTransformOperation::MeshRegionExtrude
                && request.operation != OrdinaryTransformOperation::MeshRegionInset
                && request.operation != OrdinaryTransformOperation::MeshWindingRepair
                && request.operation != OrdinaryTransformOperation::ProfileRebuild
                && request.operation != OrdinaryTransformOperation::EnclosureRebuild
                && request.operation != OrdinaryTransformOperation::SweepRebuild
                && request.operation != OrdinaryTransformOperation::LoftStationRebuild
                && request.operation != OrdinaryTransformOperation::CylindricalCut
                && request.operation != OrdinaryTransformOperation::CylindricalCutSourceRebuild
                && request.operation != OrdinaryTransformOperation::CylindricalCutProgramSourceRebuild) {
                return reject(OrdinaryEditResult::Invalid);
            }
            const auto representation = record.previous.resolvedRepresentation;
            if(request.operation==OrdinaryTransformOperation::CylindricalCut) {
                OcctCylindricalCutSource source;std::vector<std::uint8_t> encoded;
                if(!request.cut)return reject(OrdinaryEditResult::Invalid);
                const auto* legacy=std::get_if<retained_solid::Envelope>(&request.cut->envelope);
                if(!legacy){
                    // Whole-program append/identified-radius admission: the exact
                    // typed transition is recomputed from the freshly captured
                    // original full recipe and matched against prepared bytes.
                    OcctCylindricalCutProgramSource program;
                    if(!request.cutProgramEdit||!std::holds_alternative<retained_boolean::Program>(request.cut->envelope)
                        ||permit||changes.size()!=1||representation!=OcctGeometryRepresentation::BRep||request.rotationAroundPivot
                        ||!MatricesEqual(record.previous.transform,request.transform)
                        ||!_document->CaptureCylindricalCutProgramSource(request.label,program)
                        ||!program.original.IsEqual(record.previous)
                        ||!request.cut->base.IsEqual(program.base)
                        ||!retained_boolean::Encode(request.cut->envelope,encoded)||encoded!=request.cut->bytes
                        ||!_document->SavedCutSceneStateMatches(request.cutSource))return reject(OrdinaryEditResult::Invalid);
                    const auto expected=retained_boolean::Apply(program.recipe,*request.cutProgramEdit,program.effectiveMM);
                    if(!expected||expected->oldBytes!=program.recipeBytes||expected->newBytes!=request.cut->bytes
                        ||!std::holds_alternative<retained_boolean::Program>(expected->recipe))return reject(OrdinaryEditResult::Invalid);
                    if(!geometryChanges&&program.recipeBytes!=request.cut->bytes)return reject(OrdinaryEditResult::Invalid);
                    ledger.cutPrevious=request.cutSource;
                    sweepNoChange=program.recipeBytes==request.cut->bytes;
                }else{
                if(request.cutProgramEdit
                    ||permit||changes.size()!=1||representation!=OcctGeometryRepresentation::BRep||request.rotationAroundPivot
                    ||!MatricesEqual(record.previous.transform,request.transform)
                    ||!_document->CaptureCylindricalCutSource(request.label,source)||!source.original.IsEqual(record.previous)
                    ||!request.cut->base.IsEqual(source.base)
                    ||!retained_solid::Encode((*legacy),encoded)||encoded!=request.cut->bytes
                    ||!_document->SavedCutSceneStateMatches(request.cutSource))return reject(OrdinaryEditResult::Invalid);
                auto expected=source.envelope;
                if(source.rebuilding){
                    if(!cylindrical_cut::SameFixedEnvelope(expected,(*legacy)))return reject(OrdinaryEditResult::Invalid);
                }else{
                    expected.derivedFeature=(*legacy).derivedFeature;expected.operandID=(*legacy).operandID;
                    expected.axis=(*legacy).axis;expected.point=(*legacy).point;expected.radius=(*legacy).radius;
                    if(!retained_solid::Encode(expected,encoded)||encoded!=request.cut->bytes)return reject(OrdinaryEditResult::Invalid);
                }
                if(!geometryChanges&&(!source.rebuilding||source.original.retained.value->bytes!=request.cut->bytes))return reject(OrdinaryEditResult::Invalid);
                ledger.cutPrevious=request.cutSource;
                sweepNoChange=source.rebuilding&&source.original.retained.value->bytes==request.cut->bytes;
                }
            }
            if(sourceRebuild) {
                if(!savedCutSourceChangeMatches(request,record.previous))return reject(OrdinaryEditResult::Invalid);
                ledger.cutPrevious=request.cutSource;
                sweepNoChange=request.cutSourceRebuild->isNoChange();
            }
            if(programSourceRebuild) {
                if(!savedProgramSourceChangeMatches(request,record.previous))return reject(OrdinaryEditResult::Invalid);
                ledger.cutPrevious=request.cutSource;
                sweepNoChange=request.cutProgramSourceRebuild->isNoChange();
            }
            if(record.previous.retained.value&&request.operation!=OrdinaryTransformOperation::CylindricalCut
                &&request.operation!=OrdinaryTransformOperation::CylindricalCutSourceRebuild
                &&request.operation!=OrdinaryTransformOperation::CylindricalCutProgramSourceRebuild) {
                // An occurrence edit never changes the original-local result,
                // retained base or cylinder. No reserved-placement fallback.
                OcctCylindricalCutSource cut;double originalRadius=0,candidateRadius=0;
                OcctCylindricalCutProgramSource program;
                const bool wholeProgram=std::holds_alternative<retained_boolean::Program>(record.previous.retained.value->envelope);
                if(permit||changes.size()!=1||geometryChanges
                    ||(request.operation!=OrdinaryTransformOperation::Translate
                        &&request.operation!=OrdinaryTransformOperation::Rotate
                        &&request.operation!=OrdinaryTransformOperation::Scale))
                    return reject(OrdinaryEditResult::Invalid);
                if(wholeProgram){
                    // Every program operand's physical radius must stay inside
                    // the supported domain before and after the occurrence edit.
                    if(!_document->CaptureCylindricalCutProgramSource(request.label,program)
                        ||!program.original.IsEqual(record.previous)
                        ||!retained_boolean::OccurrenceRadiiMM(program.recipe,record.previous.transform)
                        ||!retained_boolean::OccurrenceRadiiMM(program.recipe,request.transform))
                        return reject(OrdinaryEditResult::Invalid);
                }else if(!_document->CaptureCylindricalCutSource(request.label,cut)||!cut.rebuilding
                    ||!cut.original.IsEqual(record.previous)
                    ||!cylindrical_cut::OccurrenceRadius(cut.envelope,record.previous.transform,originalRadius)
                    ||!cylindrical_cut::OccurrenceRadius(cut.envelope,request.transform,candidateRadius))
                    return reject(OrdinaryEditResult::Invalid);
                if(request.operation==OrdinaryTransformOperation::Scale) {
                    auto expected=record.previous.transform;
                    expected.SetScaleFactor(request.transform.ScaleFactor());
                    if(!MatricesEqual(expected,request.transform))return reject(OrdinaryEditResult::Invalid);
                }
                ledger.cutPrevious=_document->CaptureSavedCutSceneState(request.label);
                if(!ledger.cutPrevious)return reject(OrdinaryEditResult::Invalid);
                sweepNoChange=MatricesEqual(record.previous.transform,request.transform);
            }
            if (request.operation==OrdinaryTransformOperation::SweepRebuild) {
                std::vector<double> values;
                if (permit || changes.size()!=1 || representation!=OcctGeometryRepresentation::BRep
                    || !record.previous.sweep.IsCurrent(document,request.label)
                    || !MatricesEqual(record.previous.transform,request.transform) || request.rotationAroundPivot
                    || !geometryChanges || !sweep_rebuild::HasOnlyMetadataSubshapes(document,request.label)
                    || !sweep_rebuild::FixedStructure(record.previous.sweep.definition,*request.sweepRebuild)
                    || !sweep_persistence::Encode(*request.sweepRebuild,values)) return reject(OrdinaryEditResult::Invalid);
                if (!savedSweepRebuildSourceIsCurrent(request.sweepSource)) return reject(OrdinaryEditResult::Invalid);
                ledger.sweepGuard=std::make_shared<SweepRebuildGuard>(*request.sweepSource);
                // All numeric values are finite and fixed fields bit-equal. Only
                // mutable geometric signed-zero aliases compare numerically here.
                sweepNoChange=values==record.previous.sweep.values;
            }
            if(request.operation==OrdinaryTransformOperation::LoftStationRebuild) {
                std::vector<double> values;
                if((permit&&permit->operation_!=receipt::Operation::RebuildLoftStation) || changes.size()!=1 || representation!=OcctGeometryRepresentation::BRep
                    || !record.previous.loft.IsCurrent(document,request.label)
                    || !MatricesEqual(record.previous.transform,request.transform) || request.rotationAroundPivot
                    || !geometryChanges || !loft_rebuild::HasOnlyMetadataSubshapes(document,request.label)
                    || !loft_rebuild::Matches(record.previous.loft.definition,*request.loftStationEdit,*request.loftRebuild)
                    || !loft_persistence::Encode(*request.loftRebuild,values)
                    || !savedSweepRebuildSourceIsCurrent(request.sweepSource)) return reject(OrdinaryEditResult::Invalid);
                ledger.sweepGuard=std::make_shared<SweepRebuildGuard>(*request.sweepSource);
                sweepNoChange=loft_persistence::SameBits(values,record.previous.loft.values);
                if(sweepNoChange&&permit)return unchanged();
            }
            if (request.operation == OrdinaryTransformOperation::ProfileRebuild) {
                std::vector<double> values;
                if (changes.size() != 1 || representation != OcctGeometryRepresentation::BRep
                    || !record.previous.profile.IsCurrent(document, request.label)
                    || !MatricesEqual(record.previous.transform, request.transform)
                    || request.rotationAroundPivot || !geometryChanges
                    || !profile::HasOnlyMetadataSubshapes(document, request.label)
                    || !profile::Encode(*request.profileRebuild, values)
                    || request.profileRebuild->metersPerUnit != record.previous.profile.parameters.metersPerUnit
                    || request.profileRebuild->constructionFrame != record.previous.profile.parameters.constructionFrame) {
                    return reject(OrdinaryEditResult::Invalid);
                }
                if (values == record.previous.profile.values) return unchanged();
            }
            if (request.operation == OrdinaryTransformOperation::EnclosureRebuild) {
                std::vector<double> values;
                if (changes.size() != 1 || representation != OcctGeometryRepresentation::BRep
                    || !record.previous.enclosure.IsCurrent(document,request.label)
                    || !MatricesEqual(record.previous.transform,request.transform)
                    || request.rotationAroundPivot || !geometryChanges
                    || !enclosure::HasOnlyMetadataSubshapes(document,request.label)
                    || !enclosure::Encode(*request.enclosureRebuild,values)
                    || request.enclosureRebuild->metersPerUnit != record.previous.enclosure.parameters.metersPerUnit
                    || request.enclosureRebuild->definition.constructionFrame
                        != record.previous.enclosure.parameters.definition.constructionFrame) {
                    return reject(OrdinaryEditResult::Invalid);
                }
                if (values == record.previous.enclosure.values) return unchanged();
            }
            if (request.operation == OrdinaryTransformOperation::MeshUVAtlas
                && (changes.size() != 1 || !geometryChanges
                    || representation != OcctGeometryRepresentation::TriangleMesh
                    || !MatricesEqual(record.previous.transform, request.transform)
                    || !_document->ValidateTriangleUVAtlas(request.label, request.shape, request.meshUVAtlasOptions))) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.operation == OrdinaryTransformOperation::MeshVertexMove
                && (changes.size()!=1 || !geometryChanges
                    || representation!=OcctGeometryRepresentation::TriangleMesh
                    || !MatricesEqual(record.previous.transform,request.transform)
                    || !_document->ValidateMeshVertexMove(request.label,request.meshVertexMove->vertices,
                        request.meshVertexMove->worldDelta,request.shape))) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.operation == OrdinaryTransformOperation::MeshRegionExtrude) {
                OcctMeshRegionExtrudePreview region;
                if(changes.size()!=1 || !geometryChanges || !request.meshRegionExtrude
                    || request.meshRegionExtrude->sideUVPolicy!=1
                    || representation!=OcctGeometryRepresentation::TriangleMesh
                    || !MatricesEqual(record.previous.transform,request.transform)
                    || request.rotationAroundPivot.has_value()
                    || !_document->CaptureMeshRegionExtrudePreview(request.label,
                        request.meshRegionExtrude->seedTriangle,region)
                    || region.triangleIndices!=request.meshRegionExtrude->resolvedTriangles
                    || !_document->ValidateMeshRegionExtrude(request.label,
                        request.meshRegionExtrude->seedTriangle,request.meshRegionExtrude->distanceMM,
                        OcctMeshRegionMutationCandidate{request.shape,
                            request.meshRegionExtrude->candidatePartition,0})) {
                    return reject(OrdinaryEditResult::Invalid);
                }
            }
            if (request.operation == OrdinaryTransformOperation::MeshRegionInset) {
                OcctMeshRegionExtrudePreview region;
                if(changes.size()!=1 || !geometryChanges || !request.meshRegionInset
                    || representation!=OcctGeometryRepresentation::TriangleMesh
                    || !MatricesEqual(record.previous.transform,request.transform)
                    || request.rotationAroundPivot.has_value()
                    || !_document->CaptureMeshRegionInsetPreview(request.label,
                        request.meshRegionInset->seedTriangle,region)
                    || region.triangleIndices!=request.meshRegionInset->resolvedTriangles
                    || !_document->ValidateMeshRegionInset(request.label,
                        request.meshRegionInset->seedTriangle,request.meshRegionInset->distanceMM,
                        OcctMeshRegionMutationCandidate{request.shape,
                            request.meshRegionInset->candidatePartition,
                            request.meshRegionInset->centerSeedTriangle})) {
                    return reject(OrdinaryEditResult::Invalid);
                }
            }
            // Region mutations carry a validated replacement. Coherent UV
            // repack proves the old partition against its exact candidate.
            // Other geometry mutations must refuse live partition authority.
            if (!record.previous.meshRegionPartition.empty() && geometryChanges
                && request.operation!=OrdinaryTransformOperation::MeshUVAtlas
                && request.operation!=OrdinaryTransformOperation::MeshRegionExtrude
                && request.operation!=OrdinaryTransformOperation::MeshRegionInset) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.operation == OrdinaryTransformOperation::MeshWindingRepair
                && (changes.size() != 1 || !geometryChanges
                    || representation != OcctGeometryRepresentation::TriangleMesh
                    || !MatricesEqual(record.previous.transform, request.transform)
                    || request.rotationAroundPivot.has_value()
                    || request.meshVertexMove.has_value()
                    || !_document->ValidateMeshWindingRepair(request.label, request.shape))) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if ((representation != OcctGeometryRepresentation::BRep
                    && representation != OcctGeometryRepresentation::LegacyUnknown
                    && representation != OcctGeometryRepresentation::TriangleMesh)) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.operation == OrdinaryTransformOperation::Scale
                && representation == OcctGeometryRepresentation::TriangleMesh) {
                // Mesh scaling changes only the persisted dimensionless factor.
                // A Scale request cannot replace topology, move its origin or
                // rotate it; those edits require their own admitted operation.
                if (geometryChanges) { return reject(OrdinaryEditResult::Invalid); }
                gp_Trsf expected = record.previous.transform;
                expected.SetScaleFactor(request.transform.ScaleFactor());
                if (!MatricesEqual(expected, request.transform)) {
                    return reject(OrdinaryEditResult::Invalid);
                }
            }
            if(permit&&permit->operation_==receipt::Operation::SetPlacement
                &&!geometryChanges&&MatricesEqual(record.previous.transform,request.transform))return unchanged();
            changed = changed || geometryChanges || !MatricesEqual(record.previous.transform, request.transform);
            record.requested = request;
            ledger.records.push_back(std::move(record));
        }
        if (!changed && !ledger.cutPrevious) { return reject(OrdinaryEditResult::NoChange); }
        if (!PrepareCollectiveGroupOrigin(_document,ledger)) return reject(OrdinaryEditResult::Invalid);
        if (!_host.admitTransform(ledger)) { return reject(OrdinaryEditResult::Invalid); }
        // Admission is synchronous, but re-read all authority after the host
        // boundary rather than assuming that a successful callback kept it.
        for (const auto& record : ledger.records) {
            if (!captureMatches(record.previous)) { return reject(OrdinaryEditResult::Invalid); }
        }
        // Rebind the exact opaque source result after the host boundary too.
        for(const auto& record:ledger.records)if(record.requested.operation==OrdinaryTransformOperation::CylindricalCutSourceRebuild
            &&!savedCutSourceChangeMatches(record.requested,record.previous))return reject(OrdinaryEditResult::Invalid);
        for(const auto& record:ledger.records)if(record.requested.operation==OrdinaryTransformOperation::CylindricalCutProgramSourceRebuild
            &&!savedProgramSourceChangeMatches(record.requested,record.previous))return reject(OrdinaryEditResult::Invalid);
        if (permit && (!permit->current() || !rebuildReceiptMatches(ledger,false)))
            return reject(OrdinaryEditResult::Invalid);
        if ((!SweepGuardMatches(_document,ledger,false)||!CutGuardMatches(_document,ledger,false)||!TransformGroupsMatch(_document,ledger,false))) return reject(OrdinaryEditResult::Invalid);
        if (sweepNoChange) return reject(OrdinaryEditResult::NoChange);
        _pending.emplace(std::move(ledger));
        _leaseLifetime = lifetime;
        _activeToken = ++_nextToken;
        const auto began = _command.begin(_document);
        if (began != OrdinaryCommandBeginResult::Started) {
            if (began == OrdinaryCommandBeginResult::OutcomeUnknown) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return reject(OrdinaryEditResult::OutcomeUnknown);
            }
            _pending.reset();
            _activeToken = 0;
            return reject(began == OrdinaryCommandBeginResult::Busy
                ? OrdinaryEditResult::Busy : began == OrdinaryCommandBeginResult::Invalid
                ? OrdinaryEditResult::Invalid : OrdinaryEditResult::RetryableFailure);
        }
        _state = OrdinaryEditState::OpenOwned;
        return OrdinaryEditLease(self, _activeToken, lifetime);
    } catch (...) {
        if (_command.isRetained()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return reject(OrdinaryEditResult::OutcomeUnknown);
        }
        _pending.reset();
        _activeToken = 0;
        return reject(OrdinaryEditResult::Invalid);
    }
}


OrdinaryEditLease OrdinaryEditController::beginMeshCopy(
    const Handle(AIS_Shape)& source, const TCollection_ExtendedString& name,
    OrdinaryEditResult* failure) noexcept {
    const auto reject=[&](OrdinaryEditResult result) {
        if (failure) *failure=result;
        return OrdinaryEditLease();
    };
    if (![NSThread isMainThread]) return reject(OrdinaryEditResult::Invalid);
    if (blocksNormalWork()) return reject(OrdinaryEditResult::Busy);
    try {
        if (_document.IsNull() || _document->Document().IsNull()
            || _document->Document()->HasOpenCommand() || source.IsNull()
            || !OcctObjectNameIsValid(name) || !_document->IsPresentationEditable(source)
            || !_document->ValidateGeometryRepresentations()) return reject(OrdinaryEditResult::Invalid);
        OrdinaryMeshCopySource copy;copy.presentation=source;copy.destinationName=name;
        const auto label=_document->ShapeLabel(source);
        if (!_document->CaptureObjectVisibilityStateForLabel(label,copy.previous)
            || !copy.previous.IsEffectivelyVisible()
            || copy.previous.object.object.resolvedRepresentation!=OcctGeometryRepresentation::BRep
            || !copy.previous.object.object.shape.IsEqual(source->Shape())
            || !MatricesEqual(copy.previous.object.object.transform,source->LocalTransformation())
            || !_document->CaptureScalarAppearanceForMeshCopy(label,copy.appearance)) return reject(OrdinaryEditResult::Invalid);
        copy.axisState=_document->ReadReferenceAxisForLabel(label,copy.axis);
        if (copy.axisState==OcctReferenceAxisReadState::Invalid) return reject(OrdinaryEditResult::Invalid);
        std::atomic_bool cancelled(false);
        meshcopy::CurrentTessellationCopy geometry;
        if (meshcopy::PrepareCurrentTessellationCopy(source->Shape(),geometry,cancelled)
            !=meshcopy::PreparationResult::Ready) return reject(OrdinaryEditResult::Invalid);
        Handle(AIS_Shape) presentation=new AIS_Shape(geometry.face);
        presentation->SetLocalTransformation(copy.previous.object.object.transform);
        _document->LoadObjectMeterial(label,presentation);
        OrdinaryCreationRequest request;
        request.presentation=presentation;request.representation=OcctGeometryRepresentation::TriangleMesh;
        request.material=_document->MaterialNameForLabel(label);request.color=_document->ColorNameForLabel(label);
        return beginCreationImpl({request},std::move(copy),failure);
    } catch (...) {return reject(OrdinaryEditResult::Invalid);}
}

bool OrdinaryEditController::meshCopySourceMatches(
    const OrdinaryMeshCopySource& source,bool candidate) const noexcept {
    try {
        OcctObjectVisibilityState actual;
        OcctScalarAppearanceState appearance;
        const auto label=source.previous.object.object.label;
        const auto& expected=candidate?source.candidate:source.previous;
        if (!_document->CaptureObjectVisibilityStateForLabel(label,actual) || !expected.IsEqual(actual)
            || !_document->CaptureScalarAppearanceForMeshCopy(label,appearance)
            || !source.appearance.IsEqual(appearance)) return false;
        OcctReferenceAxis axis;
        if (_document->ReadReferenceAxisForLabel(label,axis)!=source.axisState
            || axis.pivotSpace!=source.axis.pivotSpace || axis.directionSpace!=source.axis.directionSpace) return false;
        for (int i=1;i<=3;++i)
            if (axis.pivot.Coord(i)!=source.axis.pivot.Coord(i)
                || axis.direction.Coord(i)!=source.axis.direction.Coord(i)) return false;
        return true;
    } catch (...) {return false;}
}


OrdinaryEditLease OrdinaryEditController::beginCreation(
    const std::vector<OrdinaryCreationRequest>& requests, OrdinaryEditResult* failure) noexcept {
    return beginCreationImpl(requests,std::nullopt,failure);
}

OrdinaryEditLease OrdinaryEditController::beginModelingCreation(
    const std::vector<OrdinaryCreationRequest>& requests,
    std::shared_ptr<NativeModelingCommitPermit> permit,OrdinaryEditResult* failure) noexcept {
    if(!permit){if(failure)*failure=OrdinaryEditResult::Invalid;return {};}
    return beginCreationImpl(requests,std::nullopt,failure,std::move(permit));
}

OrdinaryEditLease OrdinaryEditController::beginCreationImpl(
    const std::vector<OrdinaryCreationRequest>& requests,
    std::optional<OrdinaryMeshCopySource> meshCopy, OrdinaryEditResult* failure,
    std::shared_ptr<NativeModelingCommitPermit> permit) noexcept {
    const auto reject = [&](OrdinaryEditResult result) {
        if (failure) { *failure = result; }
        return OrdinaryEditLease();
    };
    if (![NSThread isMainThread]) { return reject(OrdinaryEditResult::Invalid); }
    if (blocksNormalWork()) { return reject(OrdinaryEditResult::Busy); }
    if (_document.IsNull() || requests.empty() || requests.size() > 1024
        || _nextToken == std::numeric_limits<std::uint64_t>::max()) { return reject(OrdinaryEditResult::Invalid); }
    _entering = true;
    struct EnterReset { bool& flag; ~EnterReset() { flag = false; } } reset{_entering};
    try {
        const auto self = shared_from_this();
        const auto lifetime = std::make_shared<const std::uint8_t>(0);
        const auto document = _document->Document();
        if (document.IsNull() || document->HasOpenCommand()) { return reject(OrdinaryEditResult::Busy); }
        OrdinaryCreationLedger ledger;
        ledger.meshCopy=std::move(meshCopy);
        if(permit){
            // Consumed even on rejected admission; a failed attempt is never replayable.
            if(permit->admitted_||!permit->attached_||!permit->current()||!permit->admission_
                ||permit->admission_->terminal()!=request::Terminal::HandedToOrdinary||ledger.meshCopy
                ||permit->document_!=document||!permit->resolution_
                ||permit->resolution_->state()!=NativeModelingReceiptResolution::State::Pending
                ||requests.size()!=permit->featureIDs_.size())return reject(OrdinaryEditResult::Invalid);
            permit->admitted_=true;
            request::Descriptor actual;actual.operation=static_cast<request::Operation>(permit->operation_);
            if(actual.operation!=request::Operation::CreateEnclosure&&actual.operation!=request::Operation::CreateAssembly)
                return reject(OrdinaryEditResult::Invalid);
            for(std::size_t i=0;i<requests.size();++i){
                const auto& item=requests[i];request::Part part;receipt::UUID feature;
                if(actual.operation==request::Operation::CreateEnclosure){
                    if(requests.size()!=1||!item.enclosure||item.profile||item.name
                        ||!receipt::ParseUUID(item.enclosureIdentifier,feature))return reject(OrdinaryEditResult::Invalid);
                    part.recipe=request::Recipe::Enclosure;
                    part.schema=item.enclosure->definition.constructionFrame?enclosure::FramedSchemaVersion:enclosure::SchemaVersion;
                    if(!enclosure::Encode(*item.enclosure,part.values))return reject(OrdinaryEditResult::Invalid);
                }else{
                    if(!item.profile||item.enclosure||!item.name||item.name->Length()>256
                        ||!receipt::SupportedProfile(*item.profile)||!receipt::ParseUUID(item.profileIdentifier,feature))
                        return reject(OrdinaryEditResult::Invalid);
                    part.recipe=request::Recipe::Profile;part.schema=profile::SchemaFor(*item.profile);
                    if(!profile::Encode(*item.profile,part.values))return reject(OrdinaryEditResult::Invalid);
                    NSString *name=[[NSString alloc] initWithCharacters:reinterpret_cast<const unichar*>(item.name->ToExtString())
                        length:item.name->Length()];
                    NSData *utf8=[name dataUsingEncoding:NSUTF8StringEncoding allowLossyConversion:NO];
                    if(!utf8||utf8.length==0||utf8.length>256)return reject(OrdinaryEditResult::Invalid);
                    part.name.assign(static_cast<const char*>(utf8.bytes),utf8.length);
                }
                if(feature!=permit->featureIDs_[i])return reject(OrdinaryEditResult::Invalid);
                actual.parts.push_back(std::move(part));
            }
            request::Digest digest;
            std::vector<std::uint8_t> expectedBytes,actualBytes;
            if(!request::Encode(actual,actualBytes)||!request::Encode(permit->descriptor_,expectedBytes)
                ||actualBytes!=expectedBytes||!request::CommandHash(actual,digest)||digest!=permit->key_.command)
                return reject(OrdinaryEditResult::Invalid);
            ledger.modelingReceipt.emplace();ledger.modelingReceipt->permit=std::move(permit);
            if(!creationReceiptMatches(ledger,false))return reject(OrdinaryEditResult::Invalid);
        }
        if (ledger.meshCopy && (requests.size()!=1 || !meshCopySourceMatches(*ledger.meshCopy,false)))
            return reject(OrdinaryEditResult::Invalid);
        if (!CaptureCreationRoots(_document, ledger.previousRoots)
            || ledger.previousRoots.size() + requests.size() > 50000
            || !_document->CaptureSavedGroups(ledger.groups)) { return reject(OrdinaryEditResult::Invalid); }
        std::unordered_set<const AIS_Shape*> presentations;
        for (const auto& request : requests) {
            if (request.presentation.IsNull() || request.presentation->Shape().IsNull()
                || !presentations.insert(request.presentation.get()).second
                || (request.representation != OcctGeometryRepresentation::BRep
                    && request.representation != OcctGeometryRepresentation::TriangleMesh)
                || !CandidateIsFinite(request.presentation->LocalTransformation())) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.name && (!OcctObjectNameIsValid(*request.name) || ledger.meshCopy))
                return reject(OrdinaryEditResult::Invalid);
            if (int(request.profile.has_value())+int(request.enclosure.has_value())+int(request.sweep.has_value())+int(request.loft.has_value())>1)
                return reject(OrdinaryEditResult::Invalid);
            std::vector<double> sweepValues;
            if (request.sweep ? (request.representation!=OcctGeometryRepresentation::BRep || permit || ledger.meshCopy
                    || !profile::IsIdentifier(request.sweepIdentifier)
                    || !sweep_persistence::Encode(*request.sweep,sweepValues))
                    : !request.sweepIdentifier.empty()) return reject(OrdinaryEditResult::Invalid);
            std::vector<double> loftValues;
            if (request.loft ? (request.representation!=OcctGeometryRepresentation::BRep || permit || ledger.meshCopy
                    || !profile::IsIdentifier(request.loftIdentifier)
                    || !loft_persistence::Encode(*request.loft,loftValues))
                    : !request.loftIdentifier.empty()) return reject(OrdinaryEditResult::Invalid);
            std::vector<double> enclosureValues;
            if (request.enclosure ? (request.representation != OcctGeometryRepresentation::BRep
                    || !profile::IsIdentifier(request.enclosureIdentifier)
                    || !enclosure::Encode(*request.enclosure,enclosureValues))
                    : !request.enclosureIdentifier.empty()) return reject(OrdinaryEditResult::Invalid);
            std::vector<double> profileValues;
            if (request.profile ? (request.representation != OcctGeometryRepresentation::BRep
                    || !profile::IsIdentifier(request.profileIdentifier)
                    || !profile::Encode(*request.profile, profileValues))
                : !request.profileIdentifier.empty()) return reject(OrdinaryEditResult::Invalid);
            ledger.records.push_back({request, request.presentation->Shape(),
                                      request.presentation->LocalTransformation(), {}});
        }
        if (!_host.admitCreation(ledger) || (ledger.meshCopy && !_host.admitMeshCopy(ledger))
            || !creationMatches(ledger, false)) { return reject(OrdinaryEditResult::Invalid); }
        _pending.emplace(std::move(ledger));
        _leaseLifetime = lifetime;
        _activeToken = ++_nextToken;
        const auto began = _command.begin(_document);
        if (began != OrdinaryCommandBeginResult::Started) {
            if (began == OrdinaryCommandBeginResult::OutcomeUnknown) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return reject(OrdinaryEditResult::OutcomeUnknown);
            }
            _pending.reset(); _activeToken = 0;
            return reject(began == OrdinaryCommandBeginResult::Busy ? OrdinaryEditResult::Busy
                : began == OrdinaryCommandBeginResult::Invalid ? OrdinaryEditResult::Invalid
                : OrdinaryEditResult::RetryableFailure);
        }
        _state = OrdinaryEditState::OpenOwned;
        return OrdinaryEditLease(self, _activeToken, lifetime);
    } catch (...) {
        if (_command.isRetained()) { _state = OrdinaryEditState::OutcomeUnknown; return reject(OrdinaryEditResult::OutcomeUnknown); }
        _pending.reset(); _activeToken = 0; return reject(OrdinaryEditResult::Invalid);
    }
}

// These helpers are called only by the main-thread ordinary controller. Old
// receipt effects may be stale; preserve their exact catalog bytes and validate
// only the new command's effects. No transaction or filesystem is opened here.
bool OrdinaryEditController::creationReceiptMatches(const OrdinaryCreationLedger& ledger,bool candidate) const noexcept {
    if(!ledger.modelingReceipt)return true;
    try {
        const auto& r=*ledger.modelingReceipt;const auto& p=*r.permit;
        if(_document.IsNull()||_document->Document()!=p.document_)return false;
        receipt::Catalog actual;const auto status=receipt::Read(p.document_,actual);
        const auto& expected=candidate?r.candidate:p.previous_;
        if((status!=receipt::ReadStatus::Absent&&status!=receipt::ReadStatus::Valid)
            ||!actual.matches(expected))return false;
        if(!candidate)return true;
        if(!r.staged||r.record.effects.size()!=ledger.records.size())return false;
        for(std::size_t i=0;i<ledger.records.size();++i){
            receipt::Effect actualEffect;
            if(!receipt::CaptureEffect(_document,ledger.records[i].candidate.object.label,actualEffect)
                ||!(actualEffect==r.record.effects[i]))return false;
        }
        return true;
    }catch(...){return false;}
}
bool OrdinaryEditController::stageCreationReceipt(OrdinaryCreationLedger& ledger) noexcept {
    if(!ledger.modelingReceipt)return true;
    try {
        auto& r=*ledger.modelingReceipt;auto& p=*r.permit;
        if(!p.current()||r.staged||ledger.records.size()!=p.featureIDs_.size())return false;
        r.record.key=p.key_;r.record.operation=p.operation_;
        for(std::size_t i=0;i<ledger.records.size();++i){
            receipt::Effect effect;
            if(!receipt::CaptureEffect(_document,ledger.records[i].candidate.object.label,effect)
                ||effect.featureID!=p.featureIDs_[i])return false;
            r.record.effects.push_back(effect);
        }
        if(!receipt::Valid(r.record)||!receipt::Stage(_document,r.record,p.previous_)
            ||receipt::Read(p.document_,r.candidate)!=receipt::ReadStatus::Valid)return false;
        r.staged=true;
#ifdef DEBUG
        if(p.debugStageFailure_){p.debugStageFailure_=false;return false;} // after receipt write, before commit
#endif
        return p.current()&&creationReceiptMatches(ledger,true);
    }catch(...){return false;}
}

bool OrdinaryEditController::creationMatches(const OrdinaryCreationLedger& ledger, bool candidate) const noexcept {
    try {
        if (!creationReceiptMatches(ledger,candidate && ledger.modelingReceipt && ledger.modelingReceipt->staged)) return false;
        if (ledger.meshCopy && !meshCopySourceMatches(*ledger.meshCopy,candidate)) return false;
        OrdinaryCreationCatalog actual;
        OcctSavedGroupState groups;
        if (!CaptureCreationRoots(_document, actual) || !_document->CaptureSavedGroups(groups)
            || !ledger.groups.IsEqual(groups)
            || actual.size() != ledger.previousRoots.size() + (candidate ? ledger.records.size() : 0)) { return false; }
        for (const auto& entry : ledger.previousRoots) {
            const auto match = actual.find(entry.first);
            if (match == actual.end() || !CreationRootsEqual(entry.second, match->second)) { return false; }
        }
        if (!candidate) { return true; }
        std::unordered_set<std::string> entities, definitions, labels;
        for (const auto& record : ledger.records) {
            const auto& expected = record.candidate;
            const auto key = CreationLabelKey(expected.object.label);
            OcctObjectNameState stored;
            OcctReferenceAxis axis;
            if (ledger.previousRoots.count(key) || !actual.count(key) || !labels.insert(key).second
                || !entities.insert(expected.object.entityIdentifier).second
                || !definitions.insert(expected.object.definitionIdentifier).second
                || !_document->CaptureObjectNameStateForLabel(expected.object.label, stored)
                || !expected.IsEqual(stored)) { return false; }
            if (record.requested.name && (!stored.namePresent || !stored.name.IsEqual(*record.requested.name))) return false;
            if (record.requested.profile) {
                std::vector<double> values;
                if (!profile::Encode(*record.requested.profile, values)
                    || stored.object.profile.label.IsNull()
                    || stored.object.profile.identifier != record.requested.profileIdentifier
                    || stored.object.profile.values != values
                    || !stored.object.profile.IsCurrent(_document->Document(), expected.object.label)) return false;
            } else if (!stored.object.profile.label.IsNull()) return false;
            if (record.requested.enclosure) {
                std::vector<double> values;
                if (!enclosure::Encode(*record.requested.enclosure,values)
                    || stored.object.enclosure.label.IsNull()
                    || stored.object.enclosure.identifier != record.requested.enclosureIdentifier
                    || stored.object.enclosure.values != values
                    || !stored.object.enclosure.IsCurrent(_document->Document(),expected.object.label)) return false;
            } else if (!stored.object.enclosure.label.IsNull()) return false;
            if (record.requested.sweep) {
                std::vector<double> values;
                if (!sweep_persistence::Encode(*record.requested.sweep,values)
                    || stored.object.sweep.label.IsNull()
                    || stored.object.sweep.identifier!=record.requested.sweepIdentifier
                    || !sweep_persistence::SameBits(stored.object.sweep.values,values)
                    || !stored.object.sweep.IsCurrent(_document->Document(),expected.object.label)) return false;
            } else if (!stored.object.sweep.label.IsNull()) return false;
            if (record.requested.loft) {
                std::vector<double> values;
                if (!loft_persistence::Encode(*record.requested.loft,values)
                    || stored.object.loft.label.IsNull() || stored.object.loft.identifier!=record.requested.loftIdentifier
                    || !loft_persistence::SameBits(stored.object.loft.values,values)
                    || !stored.object.loft.IsCurrent(_document->Document(),expected.object.label))return false;
            } else if (!stored.object.loft.label.IsNull())return false;
            if (ledger.meshCopy) {
                const auto& source=*ledger.meshCopy;
                OcctScalarAppearanceState appearance;
                if (!_document->CaptureScalarAppearanceForMeshCopy(expected.object.label,appearance)
                    || !source.appearance.IsEqual(appearance)
                    || _document->ReadReferenceAxisForLabel(expected.object.label,axis)!=source.axisState
                    || axis.pivotSpace!=source.axis.pivotSpace || axis.directionSpace!=source.axis.directionSpace
                    || !stored.namePresent || !stored.name.IsEqual(source.destinationName)) return false;
                for (int i=1;i<=3;++i)
                    if (axis.pivot.Coord(i)!=source.axis.pivot.Coord(i)
                        || axis.direction.Coord(i)!=source.axis.direction.Coord(i)) return false;
            } else if (!CreationIntegerEquals(expected.object.label,11,record.requested.material)
                || !CreationIntegerEquals(expected.object.label,12,record.requested.color)
                || _document->ReadReferenceAxisForLabel(expected.object.label,axis)!=OcctReferenceAxisReadState::ImplicitDefault) return false;
        }
        for (const auto& entry : ledger.previousRoots) {
            if (entities.count(entry.second.entityIdentifier) || definitions.count(entry.second.definitionIdentifier)) { return false; }
        }
        return true;
    } catch (...) { return false; }
}

OrdinaryEditResult OrdinaryEditController::stageCreationAndCommit(std::uint64_t token) noexcept {
    try {
        auto& ledger = std::get<OrdinaryCreationLedger>(*_pending);
        if (!creationMatches(ledger, false)) { return cancel(token); }
        if (_command.observe() != OrdinaryCommandObservation::OpenOwned) {
            _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown;
        }
        for (std::size_t index = 0; index < ledger.records.size(); ++index) {
            auto& record = ledger.records[index];
#ifdef DEBUG
            if (_stageFailureIndex == static_cast<int>(index)) {
                _stageFailureIndex = -1; throw Standard_Failure("Creation staging fault");
            }
#endif
            if (!record.requested.presentation->Shape().IsEqual(record.shape)
                || !MatricesEqual(record.requested.presentation->LocalTransformation(), record.transform)) {
                throw Standard_Failure("Creation private geometry changed");
            }
            const auto label = _document->AddShape(record.requested.presentation, record.requested.representation);
            if (label.IsNull()) { throw Standard_Failure("Creation returned no root"); }
            if (ledger.meshCopy) {
                auto& source=*ledger.meshCopy;
                const auto from=source.previous.object.object.label;
                if (!_document->CopyObjectAppearance(from,label)
                    || !_document->CopyReferenceAxis(from,label)
                    || !_document->SetObjectNameForLabel(label,source.destinationName))
                    throw Standard_Failure("Mesh copy metadata staging failed");
                // Preserve dormant legacy fields too: CopyObjectAppearance deliberately
                // omits them under an owned PBR material; our exact ledger retains them.
                if (source.appearance.legacyPresent[0])
                    _document->SaveObjectMaterial(label,static_cast<Graphic3d_NameOfMaterial>(source.appearance.legacyValues[0]));
                if (source.appearance.legacyPresent[1])
                    _document->SaveObjectColor(label,static_cast<Quantity_NameOfColor>(source.appearance.legacyValues[1]));
#ifdef DEBUG
                if (_stageFailureIndex==2) { _stageFailureIndex=-1;throw Standard_Failure("Mesh copy metadata fault"); }
#endif
                if (!_document->SetObjectVisibilityForLabel(from,Standard_False)
                    || !_document->CaptureObjectVisibilityStateForLabel(from,source.candidate)
                    || !source.previous.HasSameObjectAndLayers(source.candidate)
                    || source.candidate.IsEffectivelyVisible())
                    throw Standard_Failure("Mesh copy source-hide staging failed");
#ifdef DEBUG
                if (_stageFailureIndex==3) { _stageFailureIndex=-1;throw Standard_Failure("Mesh copy source-hide fault"); }
#endif
            } else {
                _document->SaveObjectMaterial(label,record.requested.material);
                _document->SaveObjectColor(label,record.requested.color);
            }
            if (record.requested.name && !_document->SetObjectNameForLabel(label, *record.requested.name))
                throw Standard_Failure("Creation part name staging failed");
            if (record.requested.profile && !profile::Stage(_document->Document(), label,
                    *record.requested.profile, record.requested.profileIdentifier))
                throw Standard_Failure("Profile definition staging failed");
            if (record.requested.enclosure && !enclosure::Stage(_document->Document(),label,
                    *record.requested.enclosure,record.requested.enclosureIdentifier))
                throw Standard_Failure("Enclosure definition staging failed");
            if (record.requested.sweep && !sweep_persistence::Stage(_document->Document(),label,
                    *record.requested.sweep,record.requested.sweepIdentifier))
                throw Standard_Failure("Sweep definition staging failed");
            if (record.requested.loft && !loft_persistence::Stage(_document->Document(),label,
                    *record.requested.loft,record.requested.loftIdentifier))
                throw Standard_Failure("Loft definition staging failed");
            if (!_document->CaptureObjectNameStateForLabel(label, record.candidate)
                || !record.candidate.object.shape.IsEqual(record.shape)
                || record.candidate.object.scalars != EncodedTransform(record.transform)
                || record.candidate.object.storedRepresentation != record.requested.representation
                || record.candidate.object.resolvedRepresentation != record.requested.representation) {
                throw Standard_Failure("Creation candidate readback failed");
            }
            for (bool present : record.candidate.object.present) {
                if (!present) { throw Standard_Failure("Incomplete creation transform"); }
            }
        }
#ifdef DEBUG
        if (_stageFailureIndex == static_cast<int>(ledger.records.size())) {
            _stageFailureIndex = -1; throw Standard_Failure("Creation staged abort fault");
        }
#endif
        const bool hasFeature = std::any_of(ledger.records.begin(), ledger.records.end(),
            [](const auto& record) { return record.requested.profile.has_value() || record.requested.enclosure.has_value() || record.requested.sweep.has_value() || record.requested.loft.has_value(); });
        if (((ledger.meshCopy || hasFeature) && !_document->ValidateGeometryRepresentations())
            || !creationMatches(ledger, true)) { throw Standard_Failure("Creation catalog readback failed"); }
        if(!stageCreationReceipt(ledger) || (ledger.modelingReceipt && !ledger.modelingReceipt->permit->current()))
            throw Standard_Failure("Modeling receipt staging or epoch fence failed");
        ledger.candidateSealed = true;
        if (_command.commitAndObserve() == OrdinaryCommandObservation::Unavailable) {
            _state = OrdinaryEditState::OutcomeUnknown; _activeToken = 0; return OrdinaryEditResult::OutcomeUnknown;
        }
    } catch (...) {}
    _state = OrdinaryEditState::OutcomeUnknown; _activeToken = 0; return reconcile();
}

OrdinaryEditResult OrdinaryEditController::reconcileCreationImpl() noexcept {
    try {
        auto& ledger = std::get<OrdinaryCreationLedger>(*_pending);
#ifdef DEBUG
        if (_truthUnavailableCount > 0) {
            --_truthUnavailableCount; _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown;
        }
#endif
        auto observation = _command.observe();
        if (observation == OrdinaryCommandObservation::OpenOwned) { observation = _command.abortAndObserve(); }
        const bool candidate = observation == OrdinaryCommandObservation::ClosedWithCandidateMarker;
        const bool previous = observation == OrdinaryCommandObservation::ClosedWithPriorMarker;
        if ((!candidate && !previous) || (candidate && (!ledger.candidateSealed
            || (ledger.modelingReceipt && !ledger.modelingReceipt->staged))) || !creationMatches(ledger, candidate)) {
            _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown;
        }
        _committed = candidate; _state = OrdinaryEditState::RepairPending;
        const bool repaired=ledger.meshCopy ? _host.repairMeshCopy(ledger,candidate) : _host.repairCreation(ledger,candidate);
        if (!repaired) { return OrdinaryEditResult::OutcomeUnknown; }
        if (!creationMatches(ledger, candidate)) { _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown; }
#if DEBUG
        if(candidate&&ledger.modelingReceipt&&ledger.modelingReceipt->permit->debugBeforeReleaseFailure_){
            ledger.modelingReceipt->permit->debugBeforeReleaseFailure_=false;
            // Simulate a failing allocation at the LAST recoverable boundary.
            // Keep stamp, candidate record and Pending resolution for reconciliation.
            throw std::bad_alloc();
        }
#endif
        _state = OrdinaryEditState::Publishing;
        if (!_command.releaseClosed()) { _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown; }
        if(ledger.modelingReceipt){
            auto& resolution=*ledger.modelingReceipt->permit->resolution_;
            // Closed geometry + receipt truth is sealed before callbacks/NotifyChanges.
            static_assert(std::is_nothrow_move_assignable_v<receipt::Record>,
                "Receipt resolution transfer after command release must never allocate or throw");
            // Preserve the candidate record through the final creationMatches.
            // After release only a proven noexcept move and scalar seal remain.
            // An Aborted Pending resolution already has its original empty record.
            if(candidate)resolution.record_=std::move(ledger.modelingReceipt->record);
            resolution.state_=candidate?NativeModelingReceiptResolution::State::Committed:NativeModelingReceiptResolution::State::Aborted;
        }
        if (candidate && !_didPublish) { _didPublish = true; try { _document->NotifyChanges(); } catch (...) {} }
        clearResolved();
        return candidate ? OrdinaryEditResult::Committed : OrdinaryEditResult::RetryableFailure;
    } catch (...) { _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown; }
}

OrdinaryEditLease OrdinaryEditController::beginAppearance(
    const std::shared_ptr<const OcctPBRScalarPreparation>& prepared, OrdinaryEditResult* failure) noexcept {
    const auto reject=[&](OrdinaryEditResult r){if(failure)*failure=r;return OrdinaryEditLease();};
    if(![NSThread isMainThread]||!prepared||_document.IsNull())return reject(OrdinaryEditResult::Invalid);
    if(blocksNormalWork())return reject(OrdinaryEditResult::Busy);
    if(_nextToken==std::numeric_limits<std::uint64_t>::max())return reject(OrdinaryEditResult::Invalid);
    _entering=true;struct Reset{bool& b;~Reset(){b=false;}} reset{_entering};
    try{
        auto self=shared_from_this();auto lifetime=std::make_shared<const std::uint8_t>(0);
        if(_document->Document().IsNull()||_document->Document()->HasOpenCommand())return reject(OrdinaryEditResult::Busy);
        OrdinaryAppearanceLedger ledger;ledger.prepared=prepared;ledger.previous=_document->PBRScalarOriginal(prepared);
        if(!ledger.previous||!_document->PBRScalarStateMatches(ledger.previous)||!_host.admitAppearance(ledger)
            ||!_document->PBRScalarStateMatches(ledger.previous))return reject(OrdinaryEditResult::Invalid);
        _pending.emplace(std::move(ledger));_leaseLifetime=lifetime;_activeToken=++_nextToken;
        const auto began=_command.begin(_document);
        if(began!=OrdinaryCommandBeginResult::Started){
            if(began==OrdinaryCommandBeginResult::OutcomeUnknown){_state=OrdinaryEditState::OutcomeUnknown;return reject(OrdinaryEditResult::OutcomeUnknown);}
            _pending.reset();_activeToken=0;return reject(began==OrdinaryCommandBeginResult::Busy?OrdinaryEditResult::Busy:OrdinaryEditResult::RetryableFailure);
        }
        _state=OrdinaryEditState::OpenOwned;return OrdinaryEditLease(self,_activeToken,lifetime);
    }catch(...){if(_command.isRetained()){_state=OrdinaryEditState::OutcomeUnknown;return reject(OrdinaryEditResult::OutcomeUnknown);}
        _pending.reset();_activeToken=0;return reject(OrdinaryEditResult::Invalid);}
}
OrdinaryEditResult OrdinaryEditController::stageAppearanceAndCommit(std::uint64_t token) noexcept {
    try{
        auto& ledger=std::get<OrdinaryAppearanceLedger>(*_pending);
        if(!_document->PBRScalarStateMatches(ledger.previous))return cancel(token);
        if(_command.observe()!=OrdinaryCommandObservation::OpenOwned){_state=OrdinaryEditState::OutcomeUnknown;return OrdinaryEditResult::OutcomeUnknown;}
#ifdef DEBUG
        if(_stageFailureIndex==0){_stageFailureIndex=-1;throw Standard_Failure("Appearance before-stage fault");}
#endif
        if(!_document->StagePBRScalarPatch(ledger.prepared,ledger.candidate))throw Standard_Failure("Appearance exact stage/readback failure");
#ifdef DEBUG
        if(_stageFailureIndex==1){_stageFailureIndex=-1;throw Standard_Failure("Appearance after-stage fault");}
#endif
        ledger.candidateSealed=true;
        if(_command.commitAndObserve()==OrdinaryCommandObservation::Unavailable){_state=OrdinaryEditState::OutcomeUnknown;_activeToken=0;return OrdinaryEditResult::OutcomeUnknown;}
    }catch(...){}
    _state=OrdinaryEditState::OutcomeUnknown;_activeToken=0;return reconcile();
}
OrdinaryEditResult OrdinaryEditController::reconcileAppearanceImpl() noexcept {
    try{
        auto& ledger=std::get<OrdinaryAppearanceLedger>(*_pending);
#ifdef DEBUG
        if(_truthUnavailableCount>0){--_truthUnavailableCount;_state=OrdinaryEditState::OutcomeUnknown;return OrdinaryEditResult::OutcomeUnknown;}
#endif
        auto observed=_command.observe();if(observed==OrdinaryCommandObservation::OpenOwned)observed=_command.abortAndObserve();
        const bool candidate=observed==OrdinaryCommandObservation::ClosedWithCandidateMarker;
        const bool previous=observed==OrdinaryCommandObservation::ClosedWithPriorMarker;
        const auto& expected=candidate?ledger.candidate:ledger.previous;
        if((!candidate&&!previous)||(candidate&&!ledger.candidateSealed)||!_document->PBRScalarStateMatches(expected)){
            _state=OrdinaryEditState::OutcomeUnknown;return OrdinaryEditResult::OutcomeUnknown;}
        _committed=candidate;_state=OrdinaryEditState::RepairPending;
        if(!_host.repairAppearance(ledger,candidate))return OrdinaryEditResult::OutcomeUnknown;
        if(!_document->PBRScalarStateMatches(expected)){_state=OrdinaryEditState::OutcomeUnknown;return OrdinaryEditResult::OutcomeUnknown;}
        _state=OrdinaryEditState::Publishing;
        if(!_command.releaseClosed()){_state=OrdinaryEditState::OutcomeUnknown;return OrdinaryEditResult::OutcomeUnknown;}
        if(candidate&&!_didPublish){_didPublish=true;try{_document->NotifyChanges();}catch(...){}}
        clearResolved();return candidate?OrdinaryEditResult::Committed:OrdinaryEditResult::RetryableFailure;
    }catch(...){_state=OrdinaryEditState::OutcomeUnknown;return OrdinaryEditResult::OutcomeUnknown;}
}

OrdinaryEditLease OrdinaryEditController::beginGrouping(
    const std::vector<OcctSavedGroup>& groups, OrdinaryEditResult* failure) noexcept {
    const auto reject = [&](OrdinaryEditResult result) {
        if (failure) { *failure = result; }
        return OrdinaryEditLease();
    };
    if (![NSThread isMainThread]) { return reject(OrdinaryEditResult::Invalid); }
    if (blocksNormalWork()) { return reject(OrdinaryEditResult::Busy); }
    if (_document.IsNull() || groups.size() > 128
        || _nextToken == std::numeric_limits<std::uint64_t>::max()) {
        return reject(OrdinaryEditResult::Invalid);
    }
    _entering = true;
    struct EnterReset { bool& flag; ~EnterReset() { flag = false; } } reset{_entering};
    try {
        const auto self = shared_from_this();
        const auto lifetime = std::make_shared<const std::uint8_t>(0);
        const auto document = _document->Document();
        if (document.IsNull() || document->HasOpenCommand()) { return reject(OrdinaryEditResult::Busy); }
        OrdinaryGroupingLedger ledger;
        if (!_document->CaptureSavedGroups(ledger.previous)) { return reject(OrdinaryEditResult::Invalid); }
        ledger.requested = groups;
        std::unordered_set<std::string> groupIDs, entities;
        for (const auto& group : groups) {
            if (group.identifier.size() != 36 || !Standard_GUID::CheckGUIDFormat(group.identifier.c_str())
                || !groupIDs.insert(group.identifier).second || !OcctObjectNameIsValid(group.name)
                || group.members.size() > 32) { return reject(OrdinaryEditResult::Invalid); }
            for (const auto& label : group.members) {
                OcctObjectNameState object;
                if (!_document->CaptureObjectNameStateForLabel(label, object)
                    || !entities.insert(object.object.entityIdentifier).second) { return reject(OrdinaryEditResult::Invalid); }
                ledger.objects.push_back(std::move(object));
            }
        }
        bool changed = groups.size() != ledger.previous.groups.size();
        for (const auto& group : ledger.previous.groups) {
            const auto match = std::find_if(groups.begin(), groups.end(), [&](const auto& value) { return value.identifier == group.identifier; });
            changed = changed || match == groups.end();
            if (match != groups.end()) {
                changed = changed || !match->name.IsEqual(group.name) || match->originPresent!=group.originPresent
                    || (group.originPresent&&!match->origin.IsEqual(group.origin,0.0))
                    || match->members.size() != group.members.size();
                for (const auto& label : group.members) {
                    changed = changed || std::none_of(match->members.begin(), match->members.end(), [&](const auto& l) { return l.IsEqual(label); });
                }
            }
            for (const auto& label : group.members) {
                OcctObjectNameState object;
                if (!_document->CaptureObjectNameStateForLabel(label, object)) { return reject(OrdinaryEditResult::Invalid); }
                if (entities.insert(object.object.entityIdentifier).second) { ledger.objects.push_back(std::move(object)); }
            }
        }
        if (!changed) { return reject(OrdinaryEditResult::NoChange); }
        if (!_host.admitGrouping(ledger) || !groupingMatches(ledger, false)) { return reject(OrdinaryEditResult::Invalid); }
        _pending.emplace(std::move(ledger));
        _leaseLifetime = lifetime;
        _activeToken = ++_nextToken;
        const auto began = _command.begin(_document);
        if (began != OrdinaryCommandBeginResult::Started) {
            if (began == OrdinaryCommandBeginResult::OutcomeUnknown) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return reject(OrdinaryEditResult::OutcomeUnknown);
            }
            _pending.reset();
            _activeToken = 0;
            return reject(began == OrdinaryCommandBeginResult::Busy
                ? OrdinaryEditResult::Busy : began == OrdinaryCommandBeginResult::Invalid
                ? OrdinaryEditResult::Invalid : OrdinaryEditResult::RetryableFailure);
        }
        _state = OrdinaryEditState::OpenOwned;
        return OrdinaryEditLease(self, _activeToken, lifetime);
    } catch (...) {
        if (_command.isRetained()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return reject(OrdinaryEditResult::OutcomeUnknown);
        }
        _pending.reset();
        _activeToken = 0;
        return reject(OrdinaryEditResult::Invalid);
    }
}

bool OrdinaryEditController::groupingMatches(const OrdinaryGroupingLedger& ledger, bool candidate) const noexcept {
    OcctSavedGroupState state;
    if (_document.IsNull() || !_document->CaptureSavedGroups(state)
        || !(candidate ? ledger.candidate : ledger.previous).IsEqual(state)) { return false; }
    for (const auto& object : ledger.objects) { if (!captureMatches(object)) { return false; } }
    return true;
}
OrdinaryEditResult OrdinaryEditController::stageGroupingAndCommit(std::uint64_t token) noexcept {
    try {
        auto& ledger = std::get<OrdinaryGroupingLedger>(*_pending);
        if (!groupingMatches(ledger, false)) { return cancel(token); }
        if (_command.observe() != OrdinaryCommandObservation::OpenOwned) {
            _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown;
        }
#ifdef DEBUG
        if (_stageFailureIndex == 0) { _stageFailureIndex = -1; throw Standard_Failure("Grouping staging fault"); }
#endif
        if (!_document->StageSavedGroups(ledger.requested) || !_document->CaptureSavedGroups(ledger.candidate)) {
            throw Standard_Failure("Grouping readback failed");
        }
        for (const auto& object : ledger.objects) {
            if (!captureMatches(object)) { throw Standard_Failure("Grouping changed object authority"); }
        }
#ifdef DEBUG
        if (_stageFailureIndex == 1) { _stageFailureIndex = -1; throw Standard_Failure("Grouping staged abort fault"); }
#endif
        ledger.candidateSealed = true;
        if (_command.commitAndObserve() == OrdinaryCommandObservation::Unavailable) {
            _state = OrdinaryEditState::OutcomeUnknown; _activeToken = 0; return OrdinaryEditResult::OutcomeUnknown;
        }
    } catch (...) {}
    _state = OrdinaryEditState::OutcomeUnknown; _activeToken = 0; return reconcile();
}
OrdinaryEditResult OrdinaryEditController::reconcileGroupingImpl() noexcept {
    try {
        auto& ledger = std::get<OrdinaryGroupingLedger>(*_pending);
#ifdef DEBUG
        if (_truthUnavailableCount > 0) {
            --_truthUnavailableCount; _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown;
        }
#endif
        auto observation = _command.observe();
        if (observation == OrdinaryCommandObservation::OpenOwned) { observation = _command.abortAndObserve(); }
        const bool candidate = observation == OrdinaryCommandObservation::ClosedWithCandidateMarker;
        const bool previous = observation == OrdinaryCommandObservation::ClosedWithPriorMarker;
        if ((!candidate && !previous) || (candidate && !ledger.candidateSealed) || !groupingMatches(ledger, candidate)) {
            _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown;
        }
        _committed = candidate; _state = OrdinaryEditState::RepairPending;
        if (!_host.repairGrouping(ledger, candidate)) { return OrdinaryEditResult::OutcomeUnknown; }
        if (!groupingMatches(ledger, candidate)) { _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown; }
        _state = OrdinaryEditState::Publishing;
        if (!_command.releaseClosed()) { _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown; }
        if (candidate && !_didPublish) { _didPublish = true; try { _document->NotifyChanges(); } catch (...) {} }
        clearResolved();
        return candidate ? OrdinaryEditResult::Committed : OrdinaryEditResult::RetryableFailure;
    } catch (...) { _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown; }
}

OrdinaryEditLease OrdinaryEditController::beginNames(
    const std::vector<OrdinaryNameChange>& changes, OrdinaryEditResult* failure) noexcept {
    const auto reject = [&](OrdinaryEditResult result) {
        if (failure) { *failure = result; }
        return OrdinaryEditLease();
    };
    if (![NSThread isMainThread]) { return reject(OrdinaryEditResult::Invalid); }
    if (blocksNormalWork()) { return reject(OrdinaryEditResult::Busy); }
    if (_document.IsNull() || changes.empty() || changes.size() > 1024
        || _nextToken == std::numeric_limits<std::uint64_t>::max()) {
        return reject(OrdinaryEditResult::Invalid);
    }
    _entering = true;
    struct EnterReset { bool& flag; ~EnterReset() { flag = false; } } reset{_entering};
    try {
        const auto self = shared_from_this();
        const auto lifetime = std::make_shared<const std::uint8_t>(0);
        const auto document = _document->Document();
        if (document.IsNull() || document->HasOpenCommand()) { return reject(OrdinaryEditResult::Busy); }
        OrdinaryNameLedger ledger;
        ledger.records.reserve(changes.size());
        std::unordered_set<std::string> entities;
        bool changed = false;
        for (const auto& request : changes) {
            OrdinaryNameRecord record;
            if (!OcctObjectNameIsValid(request.name)
                || !_document->CaptureObjectNameStateForLabel(request.label, record.previous)
                || !entities.insert(record.previous.object.entityIdentifier).second) {
                return reject(OrdinaryEditResult::Invalid);
            }
            changed = changed || !record.previous.namePresent || !record.previous.name.IsEqual(request.name);
            record.requested = request;
            ledger.records.push_back(std::move(record));
        }
        if (!changed) { return reject(OrdinaryEditResult::NoChange); }
        if (!_host.admitNames(ledger)) { return reject(OrdinaryEditResult::Invalid); }
        for (const auto& record : ledger.records) {
            if (!captureMatches(record.previous)) { return reject(OrdinaryEditResult::Invalid); }
        }
        _pending.emplace(std::move(ledger));
        _leaseLifetime = lifetime;
        _activeToken = ++_nextToken;
        const auto began = _command.begin(_document);
        if (began != OrdinaryCommandBeginResult::Started) {
            if (began == OrdinaryCommandBeginResult::OutcomeUnknown) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return reject(OrdinaryEditResult::OutcomeUnknown);
            }
            _pending.reset();
            _activeToken = 0;
            return reject(began == OrdinaryCommandBeginResult::Busy
                ? OrdinaryEditResult::Busy : began == OrdinaryCommandBeginResult::Invalid
                ? OrdinaryEditResult::Invalid : OrdinaryEditResult::RetryableFailure);
        }
        _state = OrdinaryEditState::OpenOwned;
        return OrdinaryEditLease(self, _activeToken, lifetime);
    } catch (...) {
        if (_command.isRetained()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return reject(OrdinaryEditResult::OutcomeUnknown);
        }
        _pending.reset();
        _activeToken = 0;
        return reject(OrdinaryEditResult::Invalid);
    }
}

bool OrdinaryEditController::captureMatches(const OcctObjectNameState& expected) const noexcept {
    OcctObjectNameState actual;
    return !_document.IsNull()
        && _document->CaptureObjectNameStateForLabel(expected.object.label, actual)
        && expected.IsEqual(actual);
}

OrdinaryEditResult OrdinaryEditController::stageNamesAndCommit(std::uint64_t token) noexcept {
    // stageAndCommit has already checked thread, state and lease ownership.
    try {
        auto& ledger = std::get<OrdinaryNameLedger>(*_pending);
        for (const auto& record : ledger.records) {
            if (!captureMatches(record.previous)) { return cancel(token); }
        }
        if (_command.observe() != OrdinaryCommandObservation::OpenOwned) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        std::size_t index = 0;
        for (auto& record : ledger.records) {
#ifdef DEBUG
            if (_stageFailureIndex == static_cast<int>(index)) {
                _stageFailureIndex = -1;
                throw Standard_Failure("Ordinary name staging fault");
            }
#endif
            ++index;
            if (!_document->SetObjectNameForLabel(record.requested.label, record.requested.name)
                || !_document->CaptureObjectNameStateForLabel(record.requested.label, record.candidate)
                || !record.candidate.object.IsEqual(record.previous.object)
                || !record.candidate.namePresent
                || !record.candidate.name.IsEqual(record.requested.name)) {
                throw Standard_Failure("Ordinary name candidate readback failed");
            }
        }
        ledger.candidateSealed = true;
        if (_command.commitAndObserve() == OrdinaryCommandObservation::Unavailable) {
            _state = OrdinaryEditState::OutcomeUnknown;
            _activeToken = 0;
            return OrdinaryEditResult::OutcomeUnknown;
        }
    } catch (...) {
        // The owned command and exact metadata ledger determine the outcome.
    }
    _state = OrdinaryEditState::OutcomeUnknown;
    _activeToken = 0;
    return reconcile();
}

OrdinaryEditResult OrdinaryEditController::reconcileNamesImpl() noexcept {
    try {
        auto& ledger = std::get<OrdinaryNameLedger>(*_pending);
#ifdef DEBUG
        if (_truthUnavailableCount > 0) {
            --_truthUnavailableCount;
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
#endif
        auto observation = _command.observe();
        if (observation == OrdinaryCommandObservation::OpenOwned) { observation = _command.abortAndObserve(); }
        const bool candidate = observation == OrdinaryCommandObservation::ClosedWithCandidateMarker;
        const bool previous = observation == OrdinaryCommandObservation::ClosedWithPriorMarker;
        if ((!candidate && !previous) || (candidate && !ledger.candidateSealed)) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        for (const auto& record : ledger.records) {
            if (!captureMatches(candidate ? record.candidate : record.previous)) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return OrdinaryEditResult::OutcomeUnknown;
            }
        }
        _committed = candidate;
        _state = OrdinaryEditState::RepairPending;
        if (!_host.repairNames(ledger, candidate)) { return OrdinaryEditResult::OutcomeUnknown; }
        for (const auto& record : ledger.records) {
            if (!captureMatches(candidate ? record.candidate : record.previous)) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return OrdinaryEditResult::OutcomeUnknown;
            }
        }
        _state = OrdinaryEditState::Publishing;
        if (!_command.releaseClosed()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        if (candidate && !_didPublish) {
            _didPublish = true;
            try { _document->NotifyChanges(); } catch (...) {}
        }
        clearResolved();
        return candidate ? OrdinaryEditResult::Committed : OrdinaryEditResult::RetryableFailure;
    } catch (...) {
        _state = OrdinaryEditState::OutcomeUnknown;
        return OrdinaryEditResult::OutcomeUnknown;
    }
}

OrdinaryEditLease OrdinaryEditController::beginVisibility(
    const std::vector<OrdinaryVisibilityChange>& changes, OrdinaryEditResult* failure) noexcept {
    const auto reject = [&](OrdinaryEditResult result) {
        if (failure) { *failure = result; }
        return OrdinaryEditLease();
    };
    if (![NSThread isMainThread]) { return reject(OrdinaryEditResult::Invalid); }
    if (blocksNormalWork()) { return reject(OrdinaryEditResult::Busy); }
    if (_document.IsNull() || changes.empty() || changes.size() > 1024
        || _nextToken == std::numeric_limits<std::uint64_t>::max()) {
        return reject(OrdinaryEditResult::Invalid);
    }
    _entering = true;
    struct EnterReset { bool& flag; ~EnterReset() { flag = false; } } reset{_entering};
    try {
        const auto self = shared_from_this();
        const auto lifetime = std::make_shared<const std::uint8_t>(0);
        const auto document = _document->Document();
        if (document.IsNull() || document->HasOpenCommand()) { return reject(OrdinaryEditResult::Busy); }
        OrdinaryVisibilityLedger ledger;
        ledger.records.reserve(changes.size());
        std::unordered_set<std::string> entities;
        bool changed = false;
        for (const auto& request : changes) {
            OrdinaryVisibilityRecord record;
            if (!_document->CaptureObjectVisibilityStateForLabel(request.label, record.previous)
                || !entities.insert(record.previous.object.object.entityIdentifier).second) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.visible) {
                for (bool hidden : record.previous.layerInvisibleAttributePresent) {
                    if (hidden) { return reject(OrdinaryEditResult::Invalid); }
                }
            }
            changed = changed || record.previous.invisibleAttributePresent == request.visible;
            record.requested = request;
            ledger.records.push_back(std::move(record));
        }
        if (!changed) { return reject(OrdinaryEditResult::NoChange); }
        if (!_host.admitVisibility(ledger)) { return reject(OrdinaryEditResult::Invalid); }
        for (const auto& record : ledger.records) {
            if (!captureMatches(record.previous)) { return reject(OrdinaryEditResult::Invalid); }
        }
        _pending.emplace(std::move(ledger));
        _leaseLifetime = lifetime;
        _activeToken = ++_nextToken;
        const auto began = _command.begin(_document);
        if (began != OrdinaryCommandBeginResult::Started) {
            if (began == OrdinaryCommandBeginResult::OutcomeUnknown) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return reject(OrdinaryEditResult::OutcomeUnknown);
            }
            _pending.reset();
            _activeToken = 0;
            return reject(began == OrdinaryCommandBeginResult::Busy
                ? OrdinaryEditResult::Busy : began == OrdinaryCommandBeginResult::Invalid
                ? OrdinaryEditResult::Invalid : OrdinaryEditResult::RetryableFailure);
        }
        _state = OrdinaryEditState::OpenOwned;
        return OrdinaryEditLease(self, _activeToken, lifetime);
    } catch (...) {
        if (_command.isRetained()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return reject(OrdinaryEditResult::OutcomeUnknown);
        }
        _pending.reset();
        _activeToken = 0;
        return reject(OrdinaryEditResult::Invalid);
    }
}

bool OrdinaryEditController::captureMatches(const OcctObjectVisibilityState& expected) const noexcept {
    OcctObjectVisibilityState actual;
    return !_document.IsNull()
        && _document->CaptureObjectVisibilityStateForLabel(expected.object.object.label, actual)
        && expected.IsEqual(actual);
}

OrdinaryEditResult OrdinaryEditController::stageVisibilityAndCommit(std::uint64_t token) noexcept {
    // stageAndCommit has already checked thread, state and lease ownership.
    try {
        auto& ledger = std::get<OrdinaryVisibilityLedger>(*_pending);
        for (const auto& record : ledger.records) {
            if (!captureMatches(record.previous)) { return cancel(token); }
        }
        if (_command.observe() != OrdinaryCommandObservation::OpenOwned) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        std::size_t index = 0;
        for (auto& record : ledger.records) {
#ifdef DEBUG
            if (_stageFailureIndex == static_cast<int>(index)) {
                _stageFailureIndex = -1;
                throw Standard_Failure("Ordinary visibility staging fault");
            }
#endif
            ++index;
            if (!_document->SetObjectVisibilityForLabel(record.requested.label, record.requested.visible)
                || !_document->CaptureObjectVisibilityStateForLabel(record.requested.label, record.candidate)
                || !record.candidate.HasSameObjectAndLayers(record.previous)
                || record.candidate.invisibleAttributePresent == record.requested.visible) {
                throw Standard_Failure("Ordinary visibility candidate readback failed");
            }
        }
        ledger.candidateSealed = true;
        if (_command.commitAndObserve() == OrdinaryCommandObservation::Unavailable) {
            _state = OrdinaryEditState::OutcomeUnknown;
            _activeToken = 0;
            return OrdinaryEditResult::OutcomeUnknown;
        }
    } catch (...) {
        // The owned command and exact metadata ledger determine the outcome.
    }
    _state = OrdinaryEditState::OutcomeUnknown;
    _activeToken = 0;
    return reconcile();
}

OrdinaryEditResult OrdinaryEditController::reconcileVisibilityImpl() noexcept {
    try {
        auto& ledger = std::get<OrdinaryVisibilityLedger>(*_pending);
#ifdef DEBUG
        if (_truthUnavailableCount > 0) {
            --_truthUnavailableCount;
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
#endif
        auto observation = _command.observe();
        if (observation == OrdinaryCommandObservation::OpenOwned) { observation = _command.abortAndObserve(); }
        const bool candidate = observation == OrdinaryCommandObservation::ClosedWithCandidateMarker;
        const bool previous = observation == OrdinaryCommandObservation::ClosedWithPriorMarker;
        if ((!candidate && !previous) || (candidate && !ledger.candidateSealed)) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        for (const auto& record : ledger.records) {
            if (!captureMatches(candidate ? record.candidate : record.previous)) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return OrdinaryEditResult::OutcomeUnknown;
            }
        }
        _committed = candidate;
        _state = OrdinaryEditState::RepairPending;
        if (!_host.repairVisibility(ledger, candidate)) { return OrdinaryEditResult::OutcomeUnknown; }
        for (const auto& record : ledger.records) {
            if (!captureMatches(candidate ? record.candidate : record.previous)) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return OrdinaryEditResult::OutcomeUnknown;
            }
        }
        _state = OrdinaryEditState::Publishing;
        if (!_command.releaseClosed()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        if (candidate && !_didPublish) {
            _didPublish = true;
            try { _document->NotifyChanges(); } catch (...) {}
        }
        clearResolved();
        return candidate ? OrdinaryEditResult::Committed : OrdinaryEditResult::RetryableFailure;
    } catch (...) {
        _state = OrdinaryEditState::OutcomeUnknown;
        return OrdinaryEditResult::OutcomeUnknown;
    }
}

bool OrdinaryEditController::captureMatches(const OcctObjectTransformState& expected) const noexcept {
    OcctObjectTransformState actual;
    return !_document.IsNull()
        && _document->CaptureObjectTransformStateForLabel(expected.label, actual)
        && expected.IsEqual(actual);
}

// This is source/value/content admission, not the original Viewer epoch proof.
// A future native wrapper must revalidate its original snapshot/stamp and Stop
// before calling beginTransform; no provider/permit can select this operation.
bool OrdinaryEditController::savedCutSourceChangeMatches(
    const OrdinaryTransformChange& request,const OcctObjectTransformState& previous) const noexcept {
    if(!NSThread.isMainThread||_document.IsNull()||!request.cutSourceRebuild||!request.cutSourcePatch
        ||!request.cutSource||request.cut||request.rotationAroundPivot
        ||request.operation!=OrdinaryTransformOperation::CylindricalCutSourceRebuild
        ||previous.resolvedRepresentation!=OcctGeometryRepresentation::BRep
        ||!previous.retained.value||!request.label.IsEqual(previous.label)
        ||!MatricesEqual(previous.transform,request.transform))return false;
    try {
        namespace e=saved_cut_source_edit;
        OcctCylindricalCutSource source;
        if(!_document->CaptureCylindricalCutSource(request.label,source)||!source.rebuilding
            ||!source.original.IsEqual(previous)
            ||!sweep_rebuild::SameRawScalars(source.original.scalars,previous.scalars)
            ||!_document->SavedCutSceneStateMatches(request.cutSource))return false;
        const auto& built=*request.cutSourceRebuild;
        const std::atomic_bool checking(false);e::Values expected;
        if(!e::PrepareValues(*source.original.retained.value,*request.cutSourcePatch,checking,expected)
            ||expected.oldBytes!=built.values.oldBytes||expected.newBytes!=built.values.newBytes
            ||expected.changed!=built.values.changed||built.noChange==expected.changed)return false;
        std::size_t aggregate=0;e::ShapeCommitment oldBase,oldResult;
        if(!e::Commit(source.base,checking,aggregate,oldBase)
            ||!e::Commit(source.original.shape,checking,aggregate,oldResult)
            ||!(oldBase==built.sourceBase)||!(oldResult==built.sourceResult))return false;
        if(built.noChange){
            // No detached replacement exists for an unchanged patch. Keep the
            // real original shape; admit host/scene currentness before NoChange.
            if(!built.newBase.IsNull()||!built.newResult.IsNull()||!built.cut.solid.IsNull()
                ||expected.oldBytes!=expected.newBytes||!request.shape.IsEqual(previous.shape)
                ||!(built.generatedBase==oldBase)||!(built.generatedResult==oldResult))return false;
        }else{
            if(built.newBase.IsNull()||built.newResult.IsNull()
                ||built.newBase.ShapeType()!=TopAbs_SOLID||built.newBase.Orientation()!=TopAbs_FORWARD
                ||built.newResult.ShapeType()!=TopAbs_SOLID||built.newResult.Orientation()!=TopAbs_FORWARD
                ||!request.shape.IsEqual(built.newResult)||!built.cut.solid.IsEqual(built.newResult)
                ||request.shape.IsEqual(previous.shape))return false;
            e::ShapeCommitment base,result;
            if(!e::Commit(built.newBase,checking,aggregate,base)
                ||!e::Commit(built.newResult,checking,aggregate,result)
                ||!(base==built.generatedBase)||!(result==built.generatedResult))return false;
        }
        // Complete unchanged scene, including shared live source contents,
        // remains required after the synchronous content readers too.
        return _document->SavedCutSceneStateMatches(request.cutSource);
    }catch(...){return false;}
}

// Whole-program counterpart: source/value/content admission over the COMPLETE
// freshly captured recipe, not the original Viewer epoch proof. The native
// wrapper revalidates its snapshot/stamp and Stop before beginTransform.
bool OrdinaryEditController::savedProgramSourceChangeMatches(
    const OrdinaryTransformChange& request,const OcctObjectTransformState& previous) const noexcept {
    if(!NSThread.isMainThread||_document.IsNull()||!request.cutProgramSourceRebuild||!request.cutProgramSourcePatch
        ||!request.cutSource||request.cut||request.cutSourcePatch||request.cutSourceRebuild||request.cutProgramEdit
        ||request.rotationAroundPivot
        ||request.operation!=OrdinaryTransformOperation::CylindricalCutProgramSourceRebuild
        ||previous.resolvedRepresentation!=OcctGeometryRepresentation::BRep
        ||!previous.retained.value||!request.label.IsEqual(previous.label)
        ||!MatricesEqual(previous.transform,request.transform))return false;
    try {
        namespace e=saved_cut_source_edit;
        namespace p=saved_program_source_edit;
        OcctCylindricalCutProgramSource source;
        if(!_document->CaptureCylindricalCutProgramSource(request.label,source)
            ||!std::holds_alternative<retained_boolean::Program>(source.recipe)
            ||!source.original.IsEqual(previous)
            ||!sweep_rebuild::SameRawScalars(source.original.scalars,previous.scalars)
            ||!_document->SavedCutSceneStateMatches(request.cutSource))return false;
        const auto& built=*request.cutProgramSourceRebuild;
        const std::atomic_bool checking(false);p::Values expected;
        if(!p::PrepareValues(source.recipe,*request.cutProgramSourcePatch,checking,expected)
            ||expected.oldBytes!=source.recipeBytes
            ||expected.oldBytes!=built.values.oldBytes||expected.newBytes!=built.values.newBytes
            ||expected.changed!=built.values.changed||built.noChange==expected.changed)return false;
        std::size_t aggregate=0;e::ShapeCommitment oldBase,oldResult;
        if(!e::Commit(source.base,checking,aggregate,oldBase)
            ||!e::Commit(source.original.shape,checking,aggregate,oldResult)
            ||!(oldBase==built.sourceBase)||!(oldResult==built.sourceResult))return false;
        if(built.noChange){
            // No detached replacement exists for an unchanged patch. Keep the
            // real original shape; admit host/scene currentness before NoChange.
            if(!built.newBase.IsNull()||!built.newResult.IsNull()||!built.build.solid.IsNull()
                ||built.build.status!=saved_boolean_build::Status::Refused
                ||expected.oldBytes!=expected.newBytes||!request.shape.IsEqual(previous.shape)
                ||!(built.generatedBase==oldBase)||!(built.generatedResult==oldResult))return false;
        }else{
            if(built.newBase.IsNull()||built.newResult.IsNull()
                ||built.newBase.ShapeType()!=TopAbs_SOLID||built.newBase.Orientation()!=TopAbs_FORWARD
                ||built.newResult.ShapeType()!=TopAbs_SOLID||built.newResult.Orientation()!=TopAbs_FORWARD
                ||built.build.status!=saved_boolean_build::Status::Built
                ||built.build.exactProgram!=expected.newBytes
                ||built.build.correspondence.classification
                    !=saved_boolean_result::Classification::MatchedOrientedBoundary
                ||!request.shape.IsEqual(built.newResult)||!built.build.solid.IsEqual(built.newResult)
                ||request.shape.IsEqual(previous.shape))return false;
            e::ShapeCommitment base,result;
            if(!e::Commit(built.newBase,checking,aggregate,base)
                ||!e::Commit(built.newResult,checking,aggregate,result)
                ||!(base==built.generatedBase)||!(result==built.generatedResult))return false;
        }
        // Complete unchanged scene, including shared live source contents,
        // remains required after the synchronous content readers too.
        return _document->SavedCutSceneStateMatches(request.cutSource);
    }catch(...){return false;}
}

// Receipt coupling is opt-in for one exact stored rebuild. Ordinary touch,
// arbitrary profiles and every other transform retain their existing path.
bool OrdinaryEditController::bindPlacementReceipt(OrdinaryTransformLedger& ledger,
    const OrdinaryTransformChange& change,const OcctObjectTransformState& previous,
    std::shared_ptr<NativeModelingCommitPermit> permit) noexcept {
    try {
        if(!permit||ledger.modelingReceipt||permit->admitted_||!permit->attached_||!permit->current()
            ||permit->operation_!=receipt::Operation::SetPlacement||!permit->expectedPlacement_||!permit->expectedSource_
            ||!permit->featureIDs_.empty()||!permit->admission_
            ||permit->admission_->terminal()!=request::Terminal::HandedToOrdinary
            ||permit->document_!=_document->Document()||!permit->resolution_
            ||permit->resolution_->state()!=NativeModelingReceiptResolution::State::Pending
            ||!change.placementContinuation||!permit->descriptor_.placement)return false;
        permit->admitted_=true;
        const auto& continuation=*change.placementContinuation;
        const auto& intent=*permit->descriptor_.placement;
        if(change.operation!=(intent.kind?OrdinaryTransformOperation::Rotate:OrdinaryTransformOperation::Translate)
            ||change.rotationAroundPivot||change.label!=continuation.label_
            ||!change.shape.IsEqual(continuation.shape_)||!change.shape.IsEqual(previous.shape)
            ||!MatricesEqual(change.transform,continuation.candidate_))return false;
        std::vector<std::uint8_t> expected,actual;request::Digest digest;
        placement::Evidence source;receipt::Catalog catalog;
        const auto status=receipt::Read(permit->document_,catalog);
        if(!request::Encode(permit->descriptor_,expected)||!request::Encode(continuation.descriptor_,actual)
            ||expected!=actual||!request::CommandHash(continuation.descriptor_,digest)||digest!=permit->key_.command
            ||!placement::Capture(_document,previous.label,source)||!(source==*permit->expectedPlacement_)
            ||!(placement::Effect(source)==*permit->expectedSource_)
            ||(status!=receipt::ReadStatus::Absent&&status!=receipt::ReadStatus::Valid)
            ||!catalog.matches(permit->previous_))return false;
        ledger.modelingReceipt.emplace();ledger.modelingReceipt->permit=std::move(permit);return true;
    }catch(...){return false;}
}
bool OrdinaryEditController::capturePlacementReceipt(const OrdinaryTransformLedger& ledger,placement::Evidence& out) noexcept {
    out={};try{
        if(!ledger.modelingReceipt||ledger.records.size()!=1)return false;
        const auto& permit=ledger.modelingReceipt->permit;
        if(!permit||permit->operation_!=receipt::Operation::SetPlacement||!permit->expectedPlacement_
            ||permit->document_!=_document->Document()||!permit->admitted_||!permit->attached_)return false;
        const auto label=ledger.records.front().previous.label;
        if(_document->Document()->HasOpenCommand()){
            // No general open-command bypass: only this controller's exact
            // retained transform ledger and original moved permit may capture.
            if(!_pending||!std::holds_alternative<OrdinaryTransformLedger>(*_pending)
                ||&std::get<OrdinaryTransformLedger>(*_pending)!=&ledger||_activeToken==0
                ||_command.observe()!=OrdinaryCommandObservation::OpenOwned)return false;
            return placement::EvidenceReader::Read(_document,label,out,true);
        }
        return placement::Capture(_document,label,out);
    }catch(...){out={};return false;}
}
bool OrdinaryEditController::bindRebuildReceipt(OrdinaryTransformLedger& ledger,
    const OrdinaryTransformChange& change, const OcctObjectTransformState& previous,
    std::shared_ptr<NativeModelingCommitPermit> permit) noexcept {
    try {
        if (!permit || ledger.modelingReceipt || permit->admitted_ || !permit->attached_
            || !permit->current() || !permit->expectedSource_ || !permit->admission_
            || permit->admission_->terminal()!=request::Terminal::HandedToOrdinary
            || permit->document_!=_document->Document() || !permit->resolution_
            || permit->resolution_->state()!=NativeModelingReceiptResolution::State::Pending
            || permit->featureIDs_.size()!=1) return false;
        permit->admitted_=true; // Even failed admission cannot replay this capability.
        request::Descriptor actual; actual.operation=static_cast<request::Operation>(permit->operation_);
        request::Part part; receipt::UUID feature;
        if (actual.operation==request::Operation::RebuildEnclosure) {
            if (change.operation!=OrdinaryTransformOperation::EnclosureRebuild || !change.enclosureRebuild
                || change.profileRebuild || !receipt::ParseUUID(previous.enclosure.identifier,feature)) return false;
            part.recipe=request::Recipe::Enclosure;
            part.schema=change.enclosureRebuild->definition.constructionFrame?enclosure::FramedSchemaVersion:enclosure::SchemaVersion;
            if (!enclosure::Encode(*change.enclosureRebuild,part.values)) return false;
        } else if (actual.operation==request::Operation::RebuildProfile) {
            if (change.operation!=OrdinaryTransformOperation::ProfileRebuild || !change.profileRebuild
                || change.enclosureRebuild || !receipt::SupportedProfile(previous.profile.parameters)
                || !receipt::SupportedProfile(*change.profileRebuild)
                || !receipt::ParseUUID(previous.profile.identifier,feature)) return false;
            part.recipe=request::Recipe::Profile; part.schema=profile::SchemaFor(*change.profileRebuild);
            if (!profile::Encode(*change.profileRebuild,part.values)) return false;
        } else if(actual.operation==request::Operation::RebuildLoftStation){
            if(change.operation!=OrdinaryTransformOperation::LoftStationRebuild
                ||!change.loftRebuild||!change.loftStationEdit||change.profileRebuild||change.enclosureRebuild||change.sweepRebuild
                ||permit->expectedSource_->policy!=receipt::ExactLoftPolicy4097
                ||permit->expectedSource_->feature!=receipt::Feature::RectangularLoft
                ||!receipt::ParseUUID(previous.loft.identifier,feature)
                ||!loft_rebuild::Matches(previous.loft.definition,*change.loftStationEdit,*change.loftRebuild)
                ||!receipt::LoftStationDescriptor(previous.loft.definition,*change.loftStationEdit,actual))return false;
        } else return false;
        if(actual.operation!=request::Operation::RebuildLoftStation)actual.parts.push_back(std::move(part));
        request::Digest digest;
        std::vector<std::uint8_t> expectedBytes,actualBytes;
        receipt::Effect source; receipt::Catalog catalog;
        const auto status=receipt::Read(permit->document_,catalog);
        if (feature!=permit->featureIDs_.front() || feature!=permit->expectedSource_->featureID
            || !request::Encode(actual,actualBytes) || !request::Encode(permit->descriptor_,expectedBytes)
            || actualBytes!=expectedBytes || !request::CommandHash(actual,digest) || digest!=permit->key_.command
            || !receipt::CaptureEffect(_document,previous.label,source) || !(source==*permit->expectedSource_)
            || (status!=receipt::ReadStatus::Absent && status!=receipt::ReadStatus::Valid)
            || !catalog.matches(permit->previous_)) return false;
        ledger.modelingReceipt.emplace(); ledger.modelingReceipt->permit=std::move(permit); return true;
    } catch (...) { return false; }
}
bool OrdinaryEditController::rebuildReceiptMatches(const OrdinaryTransformLedger& ledger,bool candidate) noexcept {
    if (!ledger.modelingReceipt) return true;
    try {
        const auto& r=*ledger.modelingReceipt; const auto& p=*r.permit;
        if (ledger.records.size()!=1 || !p.expectedSource_ || _document->Document()!=p.document_) return false;
        receipt::Catalog catalog; const auto status=receipt::Read(p.document_,catalog);
        const auto& expected=candidate?r.candidate:p.previous_;
        if ((status!=receipt::ReadStatus::Absent && status!=receipt::ReadStatus::Valid)
            || !catalog.matches(expected)) return false;
        receipt::Effect live;
        if(p.operation_==receipt::Operation::SetPlacement){
            placement::Evidence evidence;if(!capturePlacementReceipt(ledger,evidence))return false;
            live=placement::Effect(evidence);
            if(!p.expectedPlacement_||evidence.recipeBytes!=p.expectedPlacement_->recipeBytes
                ||evidence.recipe!=p.expectedPlacement_->recipe||evidence.geometry!=p.expectedPlacement_->geometry
                ||evidence.stateBytes.size()<206||p.expectedPlacement_->stateBytes.size()<206
                ||std::vector<std::uint8_t>(evidence.stateBytes.begin()+206,evidence.stateBytes.end())
                    !=std::vector<std::uint8_t>(p.expectedPlacement_->stateBytes.begin()+206,p.expectedPlacement_->stateBytes.end()))return false;
        }else if (!receipt::CaptureEffect(_document,ledger.records.front().previous.label,live)) return false;
        return candidate ? r.staged && r.record.effects.size()==1 && live==r.record.effects.front()
            : live==*p.expectedSource_;
    } catch (...) { return false; }
}
bool OrdinaryEditController::stageRebuildReceipt(OrdinaryTransformLedger& ledger) noexcept {
    if (!ledger.modelingReceipt) return true;
    try {
        auto& r=*ledger.modelingReceipt; auto& p=*r.permit;
        if (ledger.records.size()!=1 || !p.current() || !p.expectedSource_ || r.staged) return false;
        receipt::Effect effect;
        if(p.operation_==receipt::Operation::SetPlacement){
            placement::Evidence evidence;if(!capturePlacementReceipt(ledger,evidence))return false;
            effect=placement::Effect(evidence);
            if(!p.expectedPlacement_||evidence.recipeBytes!=p.expectedPlacement_->recipeBytes
                ||evidence.recipe!=p.expectedPlacement_->recipe||evidence.geometry!=p.expectedPlacement_->geometry
                ||evidence.stateBytes.size()<206||p.expectedPlacement_->stateBytes.size()<206
                ||std::vector<std::uint8_t>(evidence.stateBytes.begin()+206,evidence.stateBytes.end())
                    !=std::vector<std::uint8_t>(p.expectedPlacement_->stateBytes.begin()+206,p.expectedPlacement_->stateBytes.end()))return false;
        }else if(!receipt::CaptureEffect(_document,ledger.records.front().candidate.label,effect))return false;
        if (effect.entity!=p.expectedSource_->entity || effect.definition!=p.expectedSource_->definition
            || effect.feature!=p.expectedSource_->feature || effect.featureID!=p.expectedSource_->featureID) return false;
        r.record.key=p.key_; r.record.operation=p.operation_; r.record.effects={effect};
        if(p.operation_==receipt::Operation::SetPlacement)r.record.policy=receipt::ExactPlacementPolicy8193;
        if(p.operation_==receipt::Operation::RebuildLoftStation){
            if(effect.policy!=receipt::ExactLoftPolicy4097||effect.feature!=receipt::Feature::RectangularLoft)return false;
            r.record.policy=receipt::ExactLoftPolicy4097;
        }
        if (!receipt::Valid(r.record) || !receipt::Stage(_document,r.record,p.previous_)
            || receipt::Read(p.document_,r.candidate)!=receipt::ReadStatus::Valid) return false;
        r.staged=true;
#if DEBUG
        if (p.debugStageFailure_) { p.debugStageFailure_=false; return false; }
#endif
        return p.current() && rebuildReceiptMatches(ledger,true);
    } catch (...) { return false; }
}

OrdinaryEditResult OrdinaryEditController::stageAndCommit(std::uint64_t token) noexcept {
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    if (_entering || _reconciling || _state != OrdinaryEditState::OpenOwned
        || !_pending || token == 0 || token != _activeToken) { return OrdinaryEditResult::Busy; }
    if (std::holds_alternative<OrdinaryAppearanceLedger>(*_pending)) { return stageAppearanceAndCommit(token); }
    if (std::holds_alternative<OrdinaryCreationLedger>(*_pending)) { return stageCreationAndCommit(token); }
    if (std::holds_alternative<OrdinaryGroupingLedger>(*_pending)) { return stageGroupingAndCommit(token); }
    if (std::holds_alternative<OrdinaryVisibilityLedger>(*_pending)) { return stageVisibilityAndCommit(token); }
    if (std::holds_alternative<OrdinaryNameLedger>(*_pending)) { return stageNamesAndCommit(token); }
    try {
        auto& ledger = std::get<OrdinaryTransformLedger>(*_pending);
        if (ledger.modelingReceipt && (!ledger.modelingReceipt->permit->current()
            || !rebuildReceiptMatches(ledger,false))) return cancel(token);
        // Reject stale baseline before the first write; ownership alone is
        // insufficient admission for a batch mutation.
        for (const auto& record : ledger.records) {
            if (!captureMatches(record.previous)) { return cancel(token); }
        }
        if (_command.observe() != OrdinaryCommandObservation::OpenOwned) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        if ((!SweepGuardMatches(_document,ledger,false)||!CutGuardMatches(_document,ledger,false)||!TransformGroupsMatch(_document,ledger,false))) throw Standard_Failure("Transform source catalog changed");
        std::size_t index = 0;
        for (auto& record : ledger.records) {
#ifdef DEBUG
            if (_stageFailureIndex == static_cast<int>(index)) {
                _stageFailureIndex = -1;
                throw Standard_Failure("Ordinary transform staging fault");
            }
#endif
            ++index;
            // UV regeneration changes geometry-owned tangent-frame authority.
            // Partition authority, when present, has already been proved exact
            // for the candidate and remains byte-identical in this command.
            if (record.requested.operation == OrdinaryTransformOperation::MeshUVAtlas) {
                if (!_document->ValidateTriangleUVAtlas(record.previous.label,
                        record.requested.shape, record.requested.meshUVAtlasOptions)) {
                    throw Standard_Failure("Stale mesh UV atlas candidate");
                }
                // Validation must observe the original frame authority. Only
                // the already-validated candidate may clear it in this command.
                record.previous.label.ForgetAttribute(persistence::AuthoredFrameAttributeID());
            }
            if(record.requested.operation==OrdinaryTransformOperation::MeshVertexMove
                && (!record.requested.meshVertexMove
                    || !_document->ValidateMeshVertexMove(record.previous.label,
                        record.requested.meshVertexMove->vertices,record.requested.meshVertexMove->worldDelta,
                        record.requested.shape))) {
                throw Standard_Failure("Stale mesh vertex candidate");
            }
            if(record.requested.operation==OrdinaryTransformOperation::MeshRegionExtrude) {
                OcctMeshRegionExtrudePreview region;
                if(!record.requested.meshRegionExtrude
                    || record.requested.meshRegionExtrude->sideUVPolicy!=1
                    || !_document->CaptureMeshRegionExtrudePreview(record.previous.label,
                        record.requested.meshRegionExtrude->seedTriangle,region)
                    || region.triangleIndices!=record.requested.meshRegionExtrude->resolvedTriangles
                    || !_document->ValidateMeshRegionExtrude(record.previous.label,
                        record.requested.meshRegionExtrude->seedTriangle,
                        record.requested.meshRegionExtrude->distanceMM,
                        OcctMeshRegionMutationCandidate{record.requested.shape,
                            record.requested.meshRegionExtrude->candidatePartition,0}))
                    throw Standard_Failure("Stale mesh region extrusion candidate");
            }
            if(record.requested.operation==OrdinaryTransformOperation::MeshRegionInset) {
                OcctMeshRegionExtrudePreview region;
                if(!record.requested.meshRegionInset
                    || !_document->CaptureMeshRegionInsetPreview(record.previous.label,
                        record.requested.meshRegionInset->seedTriangle,region)
                    || region.triangleIndices!=record.requested.meshRegionInset->resolvedTriangles
                    || !_document->ValidateMeshRegionInset(record.previous.label,
                        record.requested.meshRegionInset->seedTriangle,
                        record.requested.meshRegionInset->distanceMM,
                        OcctMeshRegionMutationCandidate{record.requested.shape,
                            record.requested.meshRegionInset->candidatePartition,
                            record.requested.meshRegionInset->centerSeedTriangle}))
                    throw Standard_Failure("Stale mesh region inset candidate");
            }
            if (record.requested.operation == OrdinaryTransformOperation::MeshWindingRepair
                && !_document->ValidateMeshWindingRepair(
                    record.previous.label, record.requested.shape)) {
                throw Standard_Failure("Stale mesh winding candidate");
            }
            Handle(AIS_Shape) candidate = new AIS_Shape(record.requested.shape);
            candidate->SetLocalTransformation(record.requested.transform);
            bool sweepStaged=false,loftStaged=false,cutStaged=false,cutSourceStaged=false,programSourceStaged=false;
            if (record.requested.operation==OrdinaryTransformOperation::SweepRebuild) {
                bool pairedFault=false;
#if DEBUG
                if (_stageFailureIndex==2) { _stageFailureIndex=-1;pairedFault=true; }
#endif
                sweepStaged=record.requested.sweepRebuild && _document->StageSavedSweepReplacement(
                    record.previous,record.requested.shape,*record.requested.sweepRebuild,pairedFault);
                if (!sweepStaged) throw Standard_Failure("Saved sweep paired staging failed");
            }
            if(record.requested.operation==OrdinaryTransformOperation::LoftStationRebuild) {
                bool pairedFault=false;
#if DEBUG
                if(_stageFailureIndex==2){_stageFailureIndex=-1;pairedFault=true;}
#endif
                loftStaged=record.requested.loftRebuild && record.requested.loftStationEdit
                    && _document->StageSavedLoftReplacement(record.previous,record.requested.shape,
                        *record.requested.loftRebuild,*record.requested.loftStationEdit,pairedFault);
                if(!loftStaged)throw Standard_Failure("Saved loft paired staging failed");
            }
            if(record.requested.operation==OrdinaryTransformOperation::CylindricalCut) {
                bool pairedFault=false;
#if DEBUG
                if(_stageFailureIndex==2){_stageFailureIndex=-1;pairedFault=true;}
#endif
                cutStaged=_document->StageCylindricalCutReplacement(record.previous,record.requested.shape,record.requested.cut,record.requested.cutProgramEdit,pairedFault);

#if DEBUG // Cut475 phase diagnostics only
                Cut475Trace("ordinary.stage-return",int(cutStaged));
#endif // Cut475 phase diagnostics only
                if(!cutStaged)throw Standard_Failure("Saved cylindrical cut paired staging failed");
            }
            if(record.requested.operation==OrdinaryTransformOperation::CylindricalCutSourceRebuild) {
                bool pairedFault=false;
#if DEBUG
                if(_stageFailureIndex==2){_stageFailureIndex=-1;pairedFault=true;}
#endif
                if(ledger.modelingReceipt||ledger.records.size()!=1||!record.requested.cutSourcePatch
                    ||!record.requested.cutSourceRebuild||ledger.cutSourcePayload)
                    throw Standard_Failure("Saved cut source staging admission changed");
                cutSourceStaged=_document->StageSavedCutSourceReplacement(record.previous,
                    *record.requested.cutSourcePatch,record.requested.cutSourceRebuild,ledger.cutSourcePayload,pairedFault);
                if(!cutSourceStaged)throw Standard_Failure("Saved cut source paired staging failed");
            }
            if(record.requested.operation==OrdinaryTransformOperation::CylindricalCutProgramSourceRebuild) {
                bool pairedFault=false;
#if DEBUG
                if(_stageFailureIndex==2){_stageFailureIndex=-1;pairedFault=true;}
#endif
                if(ledger.modelingReceipt||ledger.records.size()!=1||!record.requested.cutProgramSourcePatch
                    ||!record.requested.cutProgramSourceRebuild||ledger.cutSourcePayload)
                    throw Standard_Failure("Saved program source staging admission changed");
                programSourceStaged=_document->StageSavedProgramSourceReplacement(record.previous,
                    *record.requested.cutProgramSourcePatch,record.requested.cutProgramSourceRebuild,ledger.cutSourcePayload,pairedFault);
                if(!programSourceStaged)throw Standard_Failure("Saved program source paired staging failed");
            }
            const bool featureStaged=sweepStaged||loftStaged||cutStaged||cutSourceStaged||programSourceStaged;
            const bool regionExtrude=record.requested.operation==OrdinaryTransformOperation::MeshRegionExtrude;
            const bool regionInset=record.requested.operation==OrdinaryTransformOperation::MeshRegionInset;
            const bool regionMutation=regionExtrude||regionInset;
            const auto& regionPartition=regionExtrude
                ?record.requested.meshRegionExtrude->candidatePartition
                :regionInset?record.requested.meshRegionInset->candidatePartition
                    :record.previous.meshRegionPartition;
            // The old record was validated by the preparation checks above.
            // Clear before replacing the shape so neither old nor new digest is
            // ever interpreted against the wrong geometry inside this command.
            if(regionMutation && !record.previous.meshRegionPartition.empty()
                && !_document->ClearMeshRegionPartition(record.previous.label))
                throw Standard_Failure("Mesh region partition clear failed");
            if ((!featureStaged && !record.previous.shape.IsEqual(record.requested.shape)
                    && !_document->ReplaceShape(record.previous.label, candidate))
                || (!featureStaged && !_document->SaveObjectTransform(record.previous.label, candidate))
                || (regionMutation
                    && !_document->MarkAuthoredMeshUVLayout(record.previous.label))
                || (regionMutation && !regionPartition.empty()
                    && !_document->StageMeshRegionPartition(record.previous.label,regionPartition))
                || (record.requested.operation == OrdinaryTransformOperation::MeshUVAtlas
                    && !_document->MarkTriangleUVAtlas(record.previous.label, record.requested.meshUVAtlasOptions))
                || (record.requested.operation == OrdinaryTransformOperation::ProfileRebuild
                    && !profile::Stage(_document->Document(), record.previous.label,
                        *record.requested.profileRebuild, record.previous.profile.identifier))
                || (record.requested.operation == OrdinaryTransformOperation::EnclosureRebuild
                    && !enclosure::Stage(_document->Document(),record.previous.label,
                        *record.requested.enclosureRebuild,record.previous.enclosure.identifier))
                || !_document->CaptureObjectTransformStateForLabel(record.previous.label, record.candidate)
                || !record.candidate.shape.IsEqual(record.requested.shape)
                || record.candidate.entityIdentifier != record.previous.entityIdentifier
                || record.candidate.definitionIdentifier != record.previous.definitionIdentifier
                || (!cutStaged && record.requested.operation != OrdinaryTransformOperation::ProfileRebuild
                    && !record.candidate.profile.IsEqual(record.previous.profile))
                || (!cutStaged && record.requested.operation != OrdinaryTransformOperation::EnclosureRebuild
                    && !record.candidate.enclosure.IsEqual(record.previous.enclosure))
                || (featureStaged ? (record.candidate.present!=record.previous.present
                    || !sweep_rebuild::SameRawScalars(record.candidate.scalars,record.previous.scalars))
                    : record.candidate.scalars != EncodedTransform(record.requested.transform))
                || (!sweepStaged && !record.candidate.sweep.IsEqual(record.previous.sweep))
                || (!loftStaged && !record.candidate.loft.IsEqual(record.previous.loft))
                || (!cutStaged && !cutSourceStaged && !programSourceStaged && !record.candidate.retained.IsEqual(record.previous.retained))
                || record.candidate.meshRegionPartition!=regionPartition
                || record.candidate.meshUVAtlasVersion != (record.requested.operation == OrdinaryTransformOperation::MeshUVAtlas
                    ? record.requested.meshUVAtlasOptions.version
                    : regionMutation
                    ? 3 : record.previous.meshUVAtlasVersion)) {
                throw Standard_Failure("Ordinary transform candidate readback failed");
            }
            if (record.requested.operation == OrdinaryTransformOperation::ProfileRebuild) {
                std::vector<double> values;
                if (!profile::Encode(*record.requested.profileRebuild, values)
                    || record.candidate.profile.identifier != record.previous.profile.identifier
                    || !record.candidate.profile.label.IsEqual(record.previous.profile.label)
                    || record.candidate.profile.values != values
                    || !record.candidate.profile.IsCurrent(_document->Document(), record.previous.label)
                    || !_document->ValidateGeometryRepresentations()) {
                    throw Standard_Failure("Profile rebuild candidate readback failed");
                }
            }
            if (record.requested.operation == OrdinaryTransformOperation::EnclosureRebuild) {
                std::vector<double> values;
                if (!enclosure::Encode(*record.requested.enclosureRebuild,values)
                    || record.candidate.enclosure.identifier != record.previous.enclosure.identifier
                    || !record.candidate.enclosure.label.IsEqual(record.previous.enclosure.label)
                    || record.candidate.enclosure.values != values
                    || !record.candidate.enclosure.IsCurrent(_document->Document(),record.previous.label)
                    || !_document->ValidateGeometryRepresentations())
                    throw Standard_Failure("Enclosure rebuild candidate readback failed");
            }
            if (record.requested.operation == OrdinaryTransformOperation::MeshUVAtlas) {
                if (record.candidate.authoredFramesPresent) {
                    throw Standard_Failure("Ordinary UV retained stale authored frames");
                }
                const auto& options=record.requested.meshUVAtlasOptions;
                Standard_Integer exactRepackPrefix=0;
                if(options.version==2 && record.previous.meshUVAtlasVersion==3) {
                    std::atomic_bool cancelled{false};meshedit::NativeTopologyCapture original;
                    if(meshedit::CaptureNativeTopology(record.previous.shape,original,cancelled)
                            !=meshedit::TopologyResult::Ready
                        || original.sourceMesh.IsNull() || original.sourceMesh->NbTriangles()<=0
                        || original.sourceMesh->NbTriangles()>4096)
                        throw Standard_Failure("Ordinary UV source prefix unavailable");
                    exactRepackPrefix=3*original.sourceMesh->NbTriangles();
                }
                if (options.version == 2 && (record.candidate.meshUVAtlasSettings[0]!=options.resolution
                    || record.candidate.meshUVAtlasSettings[1]!=options.gutterPixels
                    || record.candidate.meshUVAtlasSettings[2]<=0
                    || (record.previous.meshUVAtlasVersion==3
                        && record.candidate.meshUVAtlasSettings[2]!=exactRepackPrefix)
                    || (record.previous.meshUVAtlasVersion==2
                        && record.candidate.meshUVAtlasSettings[2]!=record.previous.meshUVAtlasSettings[2]))) {
                    throw Standard_Failure("Ordinary UV settings readback failed");
                }
            } else if (regionMutation) {
                if(record.candidate.meshUVAtlasSettings!=std::array<Standard_Integer,3>{}
                    || record.candidate.authoredFramesPresent)
                    throw Standard_Failure("Mesh region extrusion metadata readback failed");
            } else if (record.candidate.meshUVAtlasSettings != record.previous.meshUVAtlasSettings
                || record.candidate.authoredFramesPresent != record.previous.authoredFramesPresent
                || record.candidate.authoredFramesIdentity != record.previous.authoredFramesIdentity) {
                throw Standard_Failure("Ordinary transform changed geometry-owned metadata");
            }
            if (record.requested.operation == OrdinaryTransformOperation::MeshUVAtlas
                || record.requested.operation == OrdinaryTransformOperation::MeshVertexMove
                || record.requested.operation == OrdinaryTransformOperation::MeshRegionExtrude
                || record.requested.operation == OrdinaryTransformOperation::MeshRegionInset
                || record.requested.operation == OrdinaryTransformOperation::MeshWindingRepair) {
                Standard_Size bytes=0;
                if(!Core3DValidateOwnedFrameUsage(_document->Document(),bytes))
                    throw Standard_Failure("Mesh edit normal frame readback failed");
            }
            gp_Ax1 referenceAxis;
            if (!_document->ResolveReferenceAxisInWorld(record.candidate.label,
                    record.candidate.shape.Location(), referenceAxis)) {
                throw Standard_Failure("Ordinary transform exceeds persisted reference-axis limits");
            }
            if (!sweepStaged && !cutStaged && !cutSourceStaged && !programSourceStaged) for (bool present : record.candidate.present) {
                if (!present) { throw Standard_Failure("Incomplete ordinary transform candidate"); }
            }
        }
        if(ledger.groupOriginChanges) {
            if(!_document->StageSavedGroups(ledger.groupsRequested.groups)
                ||!_document->CaptureSavedGroups(ledger.groupsCandidate))
                throw Standard_Failure("Collective group origin staging failed");
        } else ledger.groupsCandidate=ledger.groupsPrevious;
#ifdef DEBUG
        if (ledger.groupOriginChanges && _stageFailureIndex == static_cast<int>(index)) {
            _stageFailureIndex = -1; throw Standard_Failure("Collective group origin post-stage fault");
        }
#endif
#if DEBUG
        if ((ledger.sweepGuard||ledger.cutPrevious) && _stageFailureIndex==3) {
            _stageFailureIndex=-1;throw Standard_Failure("Saved sweep pre-seal fault");
        }
#endif
        if(ledger.cutPrevious) {
            if(ledger.records.size()!=1)throw Standard_Failure("Saved cut single-owner mismatch");
            const auto& request=ledger.records[0].requested;
            const bool sealed=request.operation==OrdinaryTransformOperation::CylindricalCutSourceRebuild
                ?(request.cutSourcePatch&&_document->SealSavedCutSourceState(ledger.cutPrevious,*request.cutSourcePatch,
                    request.cutSourceRebuild,ledger.cutSourcePayload,ledger.cutCandidate))
                :request.operation==OrdinaryTransformOperation::CylindricalCutProgramSourceRebuild
                ?(request.cutProgramSourcePatch&&_document->SealSavedProgramSourceState(ledger.cutPrevious,*request.cutProgramSourcePatch,
                    request.cutProgramSourceRebuild,ledger.cutSourcePayload,ledger.cutCandidate))
                :request.operation==OrdinaryTransformOperation::CylindricalCut
                ?_document->SealSavedCutSceneState(ledger.cutPrevious,request.shape,request.cut,ledger.cutCandidate)
                :_document->SealSavedCutPlacementState(ledger.cutPrevious,request.transform,ledger.cutCandidate);

#if DEBUG // Cut475 phase diagnostics only
            Cut475Trace("ordinary.seal-return",int(sealed));
#endif // Cut475 phase diagnostics only
            if(!sealed)throw Standard_Failure("Saved cut complete scene mismatch");
        }
        if (!SealSweepCandidate(_document,ledger)) throw Standard_Failure("Saved sweep complete candidate mismatch");
        if (!stageRebuildReceipt(ledger) || (ledger.modelingReceipt && !ledger.modelingReceipt->permit->current()))
            throw Standard_Failure("Ordinary rebuild receipt staging failed");
        ledger.candidateSealed = true;

#if DEBUG // Cut475 phase diagnostics only
        if(ledger.cutPrevious)Cut475Trace("ordinary.before-close");
#endif // Cut475 phase diagnostics only
        if (_command.commitAndObserve() == OrdinaryCommandObservation::Unavailable) {
            _state = OrdinaryEditState::OutcomeUnknown;
            _activeToken = 0;
            return OrdinaryEditResult::OutcomeUnknown;
        }
    } catch (...) {

#if DEBUG // Cut475 phase diagnostics only
        if(_pending && std::holds_alternative<OrdinaryTransformLedger>(*_pending) && std::get<OrdinaryTransformLedger>(*_pending).cutPrevious)Cut475Trace("ordinary.stage-catch");
#endif // Cut475 phase diagnostics only
        // The retained before/candidate ledger decides the outcome. No blind
        // rollback or success claim based on a Commit Boolean/exception.
    }
    _state = OrdinaryEditState::OutcomeUnknown;
    _activeToken = 0;
    return reconcile();
}

OrdinaryEditResult OrdinaryEditController::cancel(std::uint64_t token) noexcept {
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    if (_entering || _reconciling || token == 0 || token != _activeToken || !_pending) {
        return OrdinaryEditResult::Busy;
    }
    const auto observation = _command.abortAndObserve();
    _activeToken = 0;
    _state = OrdinaryEditState::OutcomeUnknown;
    if (observation == OrdinaryCommandObservation::Unavailable) { return OrdinaryEditResult::OutcomeUnknown; }
    return reconcile();
}

bool OrdinaryEditController::presentationMatches(
    const OrdinaryTransformLedger& ledger, bool committed) const noexcept {
    try {
        if ((!SweepGuardMatches(_document,ledger,committed)||!CutGuardMatches(_document,ledger,committed)||!TransformGroupsMatch(_document,ledger,committed))) return false;
        for (const auto& record : ledger.records) {
            const auto& expected = committed ? record.candidate : record.previous;
            const auto& presentation = record.requested.presentation;
            if (!captureMatches(expected) || presentation.IsNull()
                || presentation->Shape().IsNull() || !presentation->Shape().IsEqual(expected.shape)
                || !MatricesEqual(presentation->LocalTransformation(), expected.transform)
                || !_document->ShapeLabel(presentation).IsEqual(expected.label)
                || !_document->IsPresentationEditable(presentation)) { return false; }
        }
        return true;
    } catch (...) { return false; }
}

void OrdinaryEditController::clearResolved() noexcept {
    _pending.reset();
    _leaseLifetime.reset();
    _activeToken = 0;
    _committed = false;
    _didPublish = false;
    _state = OrdinaryEditState::Idle;
}

OrdinaryEditResult OrdinaryEditController::reconcile() noexcept {
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    if (_entering || _reconciling || _state == OrdinaryEditState::Publishing) { return OrdinaryEditResult::Busy; }
    if (!_pending) { return _command.isRetained() ? OrdinaryEditResult::OutcomeUnknown : OrdinaryEditResult::NoChange; }
    // An active lease is the only authority allowed to stage or cancel.
    if (_state == OrdinaryEditState::OpenOwned && _activeToken != 0) {
        if (!_leaseLifetime.expired()) { return OrdinaryEditResult::Busy; }
        // A lease can be destroyed on the wrong thread without touching OCAF.
        // Its expired lifetime lets this main-thread retry recover ownership.
        _activeToken = 0;
        _state = OrdinaryEditState::OutcomeUnknown;
    }
    _reconciling = true;
    struct ReconcileReset { bool& flag; ~ReconcileReset() { flag = false; } } reset{_reconciling};
    return reconcileImpl();
}

OrdinaryEditResult OrdinaryEditController::reconcileImpl() noexcept {
    if (std::holds_alternative<OrdinaryAppearanceLedger>(*_pending)) { return reconcileAppearanceImpl(); }
    if (std::holds_alternative<OrdinaryCreationLedger>(*_pending)) { return reconcileCreationImpl(); }
    if (std::holds_alternative<OrdinaryGroupingLedger>(*_pending)) { return reconcileGroupingImpl(); }
    if (std::holds_alternative<OrdinaryVisibilityLedger>(*_pending)) { return reconcileVisibilityImpl(); }
    if (std::holds_alternative<OrdinaryNameLedger>(*_pending)) { return reconcileNamesImpl(); }
    try {
        auto& ledger = std::get<OrdinaryTransformLedger>(*_pending);
#ifdef DEBUG
        if (_truthUnavailableCount > 0) {
            --_truthUnavailableCount;
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
#endif
        auto observation = _command.observe();
        if (observation == OrdinaryCommandObservation::OpenOwned) { observation = _command.abortAndObserve(); }

#if DEBUG // Cut475 phase diagnostics only
        if(ledger.cutPrevious)Cut475Trace("ordinary.closed-observation",int(observation));
#endif // Cut475 phase diagnostics only
        const bool candidate = observation == OrdinaryCommandObservation::ClosedWithCandidateMarker;
        const bool previous = observation == OrdinaryCommandObservation::ClosedWithPriorMarker;

#if DEBUG // Cut475 phase diagnostics only
        if(ledger.cutPrevious)Cut475Trace("ordinary.candidate-sealed",int(ledger.candidateSealed));
#endif // Cut475 phase diagnostics only
        if ((!candidate && !previous) || (candidate && !ledger.candidateSealed)) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        for (const auto& record : ledger.records) {
            if (!captureMatches(candidate ? record.candidate : record.previous)) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return OrdinaryEditResult::OutcomeUnknown;
            }
        }

#if DEBUG // Cut475 phase diagnostics only
        if(ledger.cutPrevious)Cut475Trace("ordinary.before-repair-guard");
#endif // Cut475 phase diagnostics only
        if ((!SweepGuardMatches(_document,ledger,candidate)||!CutGuardMatches(_document,ledger,candidate)||!TransformGroupsMatch(_document,ledger,candidate))) return OrdinaryEditResult::OutcomeUnknown;
        if (!rebuildReceiptMatches(ledger,candidate)) return OrdinaryEditResult::OutcomeUnknown;

#if DEBUG // Cut475 phase diagnostics only
        if(ledger.cutPrevious)Cut475Trace("ordinary.repair-enter");
#endif // Cut475 phase diagnostics only
        _committed = candidate;
        _state = OrdinaryEditState::RepairPending;
        if (!_host.repairTransform(ledger, candidate) || !presentationMatches(ledger, candidate)) {
            std::vector<Handle(AIS_Shape)> replacements;
            if (!_host.rebuildTransform(ledger, candidate, replacements)
                || replacements.size() != ledger.records.size()) { return OrdinaryEditResult::OutcomeUnknown; }
            auto repaired = ledger;
            std::unordered_set<const AIS_Shape*> unique;
            for (std::size_t index = 0; index < replacements.size(); ++index) {
                if (replacements[index].IsNull() || !unique.insert(replacements[index].get()).second) {
                    return OrdinaryEditResult::OutcomeUnknown;
                }
                repaired.records[index].requested.presentation = replacements[index];
            }
            if (!presentationMatches(repaired, candidate)) { return OrdinaryEditResult::OutcomeUnknown; }
            for (std::size_t index = 0; index < replacements.size(); ++index) {
                ledger.records[index].requested.presentation = replacements[index];
            }
        }

#if DEBUG // Cut475 phase diagnostics only
        if(ledger.cutPrevious)Cut475Trace("ordinary.after-repair-guard");
#endif // Cut475 phase diagnostics only
        if ((!SweepGuardMatches(_document,ledger,candidate)||!CutGuardMatches(_document,ledger,candidate)||!TransformGroupsMatch(_document,ledger,candidate))) return OrdinaryEditResult::OutcomeUnknown;
        if (!rebuildReceiptMatches(ledger,candidate)) return OrdinaryEditResult::OutcomeUnknown;
#if DEBUG
        if (candidate && ledger.modelingReceipt && ledger.modelingReceipt->permit->debugBeforeReleaseFailure_) {
            ledger.modelingReceipt->permit->debugBeforeReleaseFailure_=false;
            throw std::bad_alloc(); // Retained marker still proves the actual closed outcome.
        }
#endif
        // Retain Publishing through synchronous observers. Host/observer
        // reentrancy remains Busy until this entire method has returned.
        _state = OrdinaryEditState::Publishing;

#if DEBUG // Cut475 phase diagnostics only
        if(ledger.cutPrevious)Cut475Trace("ordinary.release-enter");
#endif // Cut475 phase diagnostics only
        if (!_command.releaseClosed()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        if (ledger.modelingReceipt) {
            auto& resolution=*ledger.modelingReceipt->permit->resolution_;
            static_assert(std::is_nothrow_move_assignable_v<receipt::Record>);
            if (candidate) resolution.record_=std::move(ledger.modelingReceipt->record);
            resolution.state_=candidate?NativeModelingReceiptResolution::State::Committed
                :NativeModelingReceiptResolution::State::Aborted;
        }
        if (candidate && !_didPublish) {
            _didPublish = true;
            try { _document->NotifyChanges(); } catch (...) {}
        }
        clearResolved();
        return candidate ? OrdinaryEditResult::Committed : OrdinaryEditResult::RetryableFailure;
    } catch (...) {
        _state = OrdinaryEditState::OutcomeUnknown;
        return OrdinaryEditResult::OutcomeUnknown;
    }
}

} // namespace core3d
