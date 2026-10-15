#import "Core3DModelingTypes.h"
#import "Core3DViewController.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/NativeOpeningContext.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/ReceiptRecord.hxx"
#include "../OCCTKit/RetainedSolidAttribute.hxx"
#include "../OCCTKit/RetainedFinishingProducer.hxx"
#include "../OCCTKit/AssetAtlasPersistence.hxx"
#include "../OCCTKit/AssetAtlasBuild.hxx"
#include "Core3DViewer.h"

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#if DEBUG
// DEBUG fixture construction only: literal primitives with retained-solid
// records and real row-268 finishing receipts, mirroring
// core3d::asset_atlas::AssetAtlasProbe::AddPart (OcctDocument.mm).
#include "../OCCTKit/NativeOpeningSurfaceProbe.hxx"
#include "../OCCTKit/ProfilePersistence.hxx"
#include "../OCCTKit/RetainedPartBoolean.hxx"
#include "../OCCTKit/RectangularLoftSolid.hxx"
#include <BRep_Builder.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_CurveOnClosedSurface.hxx>
#include <BRep_CurveOnSurface.hxx>
#include <BRep_ListIteratorOfListOfCurveRepresentation.hxx>
#include <BRep_TEdge.hxx>
#include <BRepTools.hxx>
#include <BinTools.hxx>
#include <BinTools_FormatVersion.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <TNaming_Builder.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDocStd_FormatVersion.hxx>
#include <TDocStd_Application.hxx>
#include <PCDM_StoreStatus.hxx>
#include <Standard_Failure.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <Image_Texture.hxx>
#include <NCollection_Buffer.hxx>
#include <NCollection_BaseAllocator.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Plane.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <TColStd_IndexedMapOfTransient.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_LocationSet.hxx>
#include <CommonCrypto/CommonDigest.h>
#include <algorithm>
#include <functional>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#endif

namespace {
using AtlasDefinition = core3d::asset_atlas::Definition;
using AtlasKey = core3d::asset_atlas::Key;
using OwnerKey = core3d::retained_recipe::OwnerKey;
using UUID = core3d::retained_recipe::UUID;
using OpeningContext = core3d::native_opening::Context;

enum class AtlasOpeningState : std::uint8_t {
    Open, Prepared, Applying, Regenerating, Baking, Cancelled, Settled, Recovery
};

struct ControllerAtlasInput final {
    Handle(OcctDocument) owner;
    std::shared_ptr<OpeningContext> context;
    AtlasKey atlas;
    bool hasAtlas = false;
    std::vector<OwnerKey> captured;     // deduplicated, selection order
    std::vector<std::string> selected;  // entity identifiers, same order
};

struct AtlasCandidate final {
    int resolutionTexels = 0;
    int gutterTexels = 0;
    std::vector<UUID> remove;
    std::vector<UUID> add;
};

NSString *IdentifierText(const UUID& value) {
    return [NSString stringWithUTF8String:core3d::retained_solid::UUIDText(value).c_str()] ?: @"";
}

bool Integer(id value, int minimum, int maximum, int& output) {
    if (![value isKindOfClass:NSNumber.class]
        || CFGetTypeID((__bridge CFTypeRef)value) == CFBooleanGetTypeID()) return false;
    const double scalar = [value doubleValue];
    if (!std::isfinite(scalar) || scalar < minimum || scalar > maximum
        || scalar != std::trunc(scalar)) return false;
    output = int(scalar); return true;
}

bool IdentifierList(id value, std::vector<UUID>& output) {
    output.clear();
    if (![value isKindOfClass:NSArray.class]) return false;
    for (id entry in (NSArray *)value) {
        if (![entry isKindOfClass:NSString.class]) return false;
        const char *raw = [(NSString *)entry UTF8String];
        const std::string text = raw ? raw : "";
        UUID uuid{};
        if (text.empty() || text.size() > 128 || !core3d::receipt::ParseUUID(text, uuid))
            return false;
        bool duplicate = false;
        for (const auto& existing : output) if (existing == uuid) { duplicate = true; break; }
        if (!duplicate) output.push_back(uuid);
    }
    return true;
}

bool ExactCandidate(NSDictionary *candidate, AtlasCandidate& output) {
    if (![candidate isKindOfClass:NSDictionary.class] || candidate.count != 4) return false;
    NSSet *expected = [NSSet setWithArray:@[
        @"resolutionTexels", @"gutterTexels",
        @"removeMemberEntityIdentifiers", @"addMemberEntityIdentifiers"]];
    if (![[NSSet setWithArray:candidate.allKeys] isEqualToSet:expected]
        || !Integer(candidate[@"resolutionTexels"], 256, 4096, output.resolutionTexels)
        || !Integer(candidate[@"gutterTexels"], 1, 8, output.gutterTexels)
        || (output.resolutionTexels & (output.resolutionTexels - 1)) != 0
        || !IdentifierList(candidate[@"removeMemberEntityIdentifiers"], output.remove)
        || !IdentifierList(candidate[@"addMemberEntityIdentifiers"], output.add)) return false;
    return true;
}

bool KeyForSelected(const Handle(OcctDocument)& owner, const std::string& selected,
                    OwnerKey& key) noexcept {
    key = {};
    try {
        if (owner.IsNull() || selected.empty()) return false;
        const auto document = owner->Document();
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return false;
        TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
        TDF_Label match; unsigned count = 0;
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            if (owner->EntityIdentifierForLabel(roots.Value(index)) == selected) {
                match = roots.Value(index); ++count;
            }
        }
        return count == 1 && !match.IsNull()
            && core3d::receipt::ParseUUID(owner->DocumentIdentifier(), key.document)
            && core3d::receipt::ParseUUID(owner->EntityIdentifierForLabel(match), key.entity)
            && core3d::receipt::ParseUUID(owner->DefinitionIdentifierForLabel(match), key.definition)
            && core3d::retained_recipe::Valid(key);
    } catch (...) { key = {}; return false; }
}

bool KeyForEntityUUID(const Handle(OcctDocument)& owner, const UUID& entity,
                      OwnerKey& key) noexcept {
    key = {};
    try {
        if (owner.IsNull()) return false;
        const auto document = owner->Document();
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return false;
        TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
        TDF_Label match; unsigned count = 0;
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            UUID candidate{};
            if (!core3d::receipt::ParseUUID(
                    owner->EntityIdentifierForLabel(roots.Value(index)), candidate)) continue;
            if (candidate == entity) { match = roots.Value(index); ++count; }
        }
        return count == 1 && !match.IsNull()
            && core3d::receipt::ParseUUID(owner->DocumentIdentifier(), key.document)
            && core3d::receipt::ParseUUID(owner->EntityIdentifierForLabel(match), key.entity)
            && core3d::receipt::ParseUUID(owner->DefinitionIdentifierForLabel(match), key.definition)
            && core3d::retained_recipe::Valid(key);
    } catch (...) { key = {}; return false; }
}

bool SameMembers(const std::vector<OwnerKey>& first,
                 const std::vector<OwnerKey>& second) noexcept {
    if (first.size() != second.size()) return false;
    for (std::size_t index = 0; index < first.size(); ++index)
        if (!(first[index] == second[index])) return false;
    return true;
}

bool CaptureControllerAtlasInput(Core3DViewController *controller,
                                 ControllerAtlasInput& output) noexcept {
    output = {};
    try {
        if (!NSThread.isMainThread || !controller) return false;
        Core3DSceneSnapshot *scene = [controller captureSceneSnapshot];
        if (!scene || scene.selection.selectedElements.count == 0
            || scene.selection.selectedElements.count > core3d::asset_atlas::kMaximumMembers)
            return false;
        GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)controller.glController : nil;
        if (!gl) return false;
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl.viewer;
        const CGSize drawable = controller.viewportDrawableSize;
        if (!viewer || !std::isfinite(drawable.width) || !std::isfinite(drawable.height)
            || drawable.width < 1 || drawable.height < 1
            || drawable.width > UINT32_MAX || drawable.height > UINT32_MAX) return false;
        output.owner = viewer->getDocument();
        if (output.owner.IsNull() || output.owner->Document().IsNull()) return false;
        for (Core3DSceneElementIdentifier *element in scene.selection.selectedElements) {
            if (element.kind != Core3DSceneElementKindObject
                || element.entityIdentifier.length == 0
                || element.entityIdentifier.length > 128) return false;
            const char *raw = element.entityIdentifier.UTF8String;
            const std::string text = raw ? raw : "";
            OwnerKey key;
            if (!KeyForSelected(output.owner, text, key)) return false;
            bool duplicate = false;
            for (const auto& existing : output.captured)
                if (existing == key) { duplicate = true; break; }
            if (!duplicate) {
                output.captured.push_back(key);
                output.selected.push_back(text);
            }
        }
        if (output.captured.empty()) return false;
        // An existing atlas admits every captured owner; a member belongs to at
        // most one atlas, so the first containing record is the only match.
        std::vector<core3d::asset_atlas::persistence::Record> records;
        if (!core3d::asset_atlas::persistence::ReadAll(output.owner->Document(), records))
            return false;
        for (const auto& record : records) {
            if (!record.value) continue;
            bool admits = true;
            for (const auto& key : output.captured) {
                bool found = false;
                for (const auto& member : record.value->definition.members)
                    if (member.owner == key) { found = true; break; }
                if (!found) { admits = false; break; }
            }
            if (admits) {
                output.atlas = record.value->definition.key;
                output.hasAtlas = true;
                break;
            }
        }
        if (!output.hasAtlas) {
            // Mint the prospective atlas identity once per opening; nothing is
            // committed until a prepared candidate is applied.
            const char *minted = NSUUID.UUID.UUIDString.UTF8String;
            if (!minted || !core3d::receipt::ParseUUID(minted, output.atlas.atlas))
                return false;
            output.atlas.document = output.captured.front().document;
        }
        output.context = viewer->captureNativeOpeningContext(
            std::uint32_t(drawable.width), std::uint32_t(drawable.height), output.selected);
        return output.context && output.context->isCurrent(64, 64);
    } catch (...) { output = {}; return false; }
}

Core3DAssetAtlasCurrentness PublicCurrentness(OcctAssetAtlasCurrentness value) noexcept {
    switch (value) {
        case OcctAssetAtlasCurrentness::Current:
            return Core3DAssetAtlasCurrentnessCurrent;
        case OcctAssetAtlasCurrentness::Stale:
            return Core3DAssetAtlasCurrentnessStale;
        case OcctAssetAtlasCurrentness::Absent:
            return Core3DAssetAtlasCurrentnessAbsent;
    }
}

NSString *CurrentnessText(OcctAssetAtlasCurrentness value) {
    switch (value) {
        case OcctAssetAtlasCurrentness::Current: return @"current";
        case OcctAssetAtlasCurrentness::Stale: return @"stale";
        case OcctAssetAtlasCurrentness::Absent: return @"absent";
    }
}

Core3DProfileConstructionResult AtlasMutation(
    const std::shared_ptr<core3d::native_opening::CommandLease>& lease,
    OcctAssetAtlasOutcome outcome) noexcept {
    if (!lease) return Core3DProfileConstructionResultBusy;
    if (outcome == OcctAssetAtlasOutcome::Committed)
        return lease->commit() ? Core3DProfileConstructionResultCommitted
                               : Core3DProfileConstructionResultRecoveryRequired;
    if (!lease->abort()) return Core3DProfileConstructionResultRecoveryRequired;
    if (outcome == OcctAssetAtlasOutcome::Busy)
        return Core3DProfileConstructionResultBusy;
    if (outcome == OcctAssetAtlasOutcome::Absent)
        return Core3DProfileConstructionResultUnchanged;
    return Core3DProfileConstructionResultRejected;
}

Core3DProfileConstructionResult PaintedAtlasMutation(
    const std::shared_ptr<core3d::native_opening::CommandLease>& lease,
    OcctPaintedAtlasBakeOutcome outcome) noexcept {
    if(!lease)return Core3DProfileConstructionResultBusy;
    if(outcome==OcctPaintedAtlasBakeOutcome::Committed)
        return lease->commit()?Core3DProfileConstructionResultCommitted
                              :Core3DProfileConstructionResultRecoveryRequired;
    if(!lease->abort())return Core3DProfileConstructionResultRecoveryRequired;
    if(outcome==OcctPaintedAtlasBakeOutcome::Busy)
        return Core3DProfileConstructionResultBusy;
    if(outcome==OcctPaintedAtlasBakeOutcome::Absent)
        return Core3DProfileConstructionResultUnchanged;
    return Core3DProfileConstructionResultRejected;
}

void DeliverPreparation(void (^completion)(Core3DAssetAtlasPreparationResult,
                                            NSString *),
                        Core3DAssetAtlasPreparationResult result,
                        NSString *detail) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail);
    else dispatch_async(dispatch_get_main_queue(), ^{ completion(result, detail); });
}

void DeliverMutation(void (^completion)(Core3DProfileConstructionResult, NSString *),
                     Core3DProfileConstructionResult result, NSString *detail) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail);
    else dispatch_async(dispatch_get_main_queue(), ^{ completion(result, detail); });
}
} // namespace

@interface Core3DAssetAtlasOpening () {
@package
    Handle(OcctDocument) _owner;
    std::shared_ptr<OpeningContext> _context;
    AtlasKey _atlas;
    bool _hasAtlas;
    std::vector<OwnerKey> _captured;
    std::vector<std::string> _selected;
    std::vector<OwnerKey> _stagedMembers;
    OcctAssetAtlasSettings _stagedSettings;
    std::atomic<AtlasOpeningState> _state;
}
- (instancetype)initWithInput:(ControllerAtlasInput&&)input;
@end

@implementation Core3DAssetAtlasOpening
- (instancetype)initWithInput:(ControllerAtlasInput&&)input {
    if ((self = [super init])) {
        _owner = input.owner; _context = std::move(input.context);
        _atlas = input.atlas; _hasAtlas = input.hasAtlas;
        _captured = std::move(input.captured);
        _selected = std::move(input.selected);
        _stagedSettings = {};
        _state.store(AtlasOpeningState::Open);
    }
    return self;
}

- (Core3DAssetAtlasCurrentness)currentness {
    if (!NSThread.isMainThread || _owner.IsNull() || !_hasAtlas)
        return Core3DAssetAtlasCurrentnessAbsent;
    return PublicCurrentness(_owner->AssetAtlasCurrentness(_atlas));
}

- (NSDictionary<NSString *, id> *)descriptor {
    if (!NSThread.isMainThread || _owner.IsNull()) return @{};
    try {
        AtlasDefinition value;
        OcctAssetAtlasCurrentness native = OcctAssetAtlasCurrentness::Absent;
        if (_hasAtlas) native = _owner->AssetAtlasCurrentness(_atlas, &value);
        NSMutableArray<NSString *> *members = [NSMutableArray array];
        std::vector<OwnerKey> keys;
        if (native != OcctAssetAtlasCurrentness::Absent) {
            for (const auto& member : value.members) {
                keys.push_back(member.owner);
                [members addObject:IdentifierText(member.owner.entity)];
            }
        } else {
            for (const auto& key : _captured)
                [members addObject:IdentifierText(key.entity)];
        }
        NSMutableArray<NSString *> *painted = [NSMutableArray array];
        if (!keys.empty()) {
            core3d::asset_atlas::Capture capture;
            if (core3d::asset_atlas::build::CaptureMembers(
                    _owner->Document(), keys, capture)) {
                for (const auto& entry : capture.members)
                    if (entry.painted)
                        [painted addObject:IdentifierText(entry.slot.owner.entity)];
            }
        }
        double occupancy = 0;
        unsigned long long corners = 0;
        if (native != OcctAssetAtlasCurrentness::Absent) {
            for (const auto& chart : value.charts) {
                occupancy += (chart.rectUV[2] - chart.rectUV[0])
                    * (chart.rectUV[3] - chart.rectUV[1]);
                corners += 3ULL * chart.triangleCount;
            }
        }
        const unsigned long long generation = _context
            ? (unsigned long long)_context->openingFence().documentGeneration() : 0;
        const bool committed = native != OcctAssetAtlasCurrentness::Absent;
        return @{@"schema": @"shapeyard.asset-atlas.opening.v1",
            @"atlasIdentifier": committed ? IdentifierText(value.key.atlas) : @"",
            @"memberEntityIdentifiers": members,
            @"memberCount": @(members.count),
            @"chartCount": @(committed ? value.charts.size() : 0),
            @"resourceCount": @(committed ? value.resources.size() : 0),
            @"resolutionTexels": @(committed ? value.resolutionTexels : 2048),
            @"gutterTexels": @(committed ? value.gutterTexels : 2),
            @"globalTexelsPerMM": @(committed ? value.globalTexelsPerMM : 0),
            @"occupancy": @(occupancy),
            @"aggregateFinalCornerCount": @(corners),
            @"currentness": CurrentnessText(native),
            @"diagnosis": @"",
            @"paintedMemberEntityIdentifiers": painted,
            @"sourceDocumentGeneration": @(generation)};
    } catch (...) { return @{}; }
}

- (void)prepareCandidate:(NSDictionary<NSString *, id> *)candidate
              completion:(void (^)(Core3DAssetAtlasPreparationResult,
                                   NSString *))completion {
    if (!NSThread.isMainThread || _state.load() != AtlasOpeningState::Open
        || _owner.IsNull() || !_context) {
        DeliverPreparation(completion, Core3DAssetAtlasPreparationResultRejected,
                           @"Opening is not available for preparation.");
        return;
    }
    AtlasDefinition current;
    if (_hasAtlas) {
        const auto read = _owner->AssetAtlasCurrentness(_atlas, &current);
        if (read == OcctAssetAtlasCurrentness::Stale) {
            DeliverPreparation(completion, Core3DAssetAtlasPreparationResultStaleSource,
                               @"Source changed. Regenerate re-derives the asset atlas from the current members.");
            return;
        }
        if (read == OcctAssetAtlasCurrentness::Absent) {
            DeliverPreparation(completion, Core3DAssetAtlasPreparationResultRejected,
                               @"The asset atlas record is no longer committed.");
            return;
        }
    }
    AtlasCandidate request;
    if (!ExactCandidate(candidate, request)) {
        DeliverPreparation(completion, Core3DAssetAtlasPreparationResultRejected,
                           @"Use a power-of-two 256...4096 resolution, 1...8 gutter texels, and member identifier lists.");
        return;
    }
    std::vector<OwnerKey> members;
    if (_hasAtlas) {
        for (const auto& member : current.members) members.push_back(member.owner);
    } else {
        members = _captured;
    }
    for (const auto& uuid : request.remove) {
        bool found = false;
        for (std::size_t index = 0; index < members.size(); ++index) {
            if (members[index].entity == uuid) {
                members.erase(members.begin() + index); found = true; break;
            }
        }
        if (!found) {
            DeliverPreparation(completion, Core3DAssetAtlasPreparationResultMissingMember,
                               @"The removed member is not in this asset atlas.");
            return;
        }
    }
    std::vector<core3d::asset_atlas::persistence::Record> records;
    if (!request.add.empty()
        && !core3d::asset_atlas::persistence::ReadAll(_owner->Document(), records)) {
        DeliverPreparation(completion, Core3DAssetAtlasPreparationResultRejected,
                           @"The asset atlas records could not be read.");
        return;
    }
    for (const auto& uuid : request.add) {
        bool present = false;
        for (const auto& existing : members)
            if (existing.entity == uuid) { present = true; break; }
        if (present) continue;
        OwnerKey key;
        if (!KeyForEntityUUID(_owner, uuid, key)) {
            DeliverPreparation(completion, Core3DAssetAtlasPreparationResultMissingMember,
                               @"The added member is missing from the current document.");
            return;
        }
        if (!(key.document == _atlas.document)) {
            DeliverPreparation(completion, Core3DAssetAtlasPreparationResultForeignMember,
                               @"The added member belongs to a different document.");
            return;
        }
        bool foreign = false;
        for (const auto& record : records) {
            if (!record.value || (_hasAtlas && record.value->definition.key == _atlas))
                continue;
            for (const auto& member : record.value->definition.members)
                if (member.owner == key) { foreign = true; break; }
            if (foreign) break;
        }
        if (foreign) {
            DeliverPreparation(completion, Core3DAssetAtlasPreparationResultForeignMember,
                               @"The added member already belongs to another asset atlas.");
            return;
        }
        members.push_back(key);
    }
    if (members.empty()) {
        DeliverPreparation(completion, Core3DAssetAtlasPreparationResultRejected,
                           @"An asset atlas keeps at least one admitted member.");
        return;
    }
    if (members.size() > core3d::asset_atlas::kMaximumMembers) {
        DeliverPreparation(completion, Core3DAssetAtlasPreparationResultOverBudget,
                           @"An asset atlas admits at most eight members.");
        return;
    }
    for (const auto& key : members) {
        const auto status = core3d::asset_atlas::build::ClassifyMember(
            _owner->Document(), key);
        if (status == core3d::asset_atlas::build::Status::Captured) continue;
        Core3DAssetAtlasPreparationResult result = Core3DAssetAtlasPreparationResultRejected;
        NSString *detail = @"A member is not an admitted retained-finishing owner.";
        switch (status) {
            case core3d::asset_atlas::build::Status::MissingMember:
                result = Core3DAssetAtlasPreparationResultMissingMember;
                detail = @"A member is missing from the current document."; break;
            case core3d::asset_atlas::build::Status::ForeignMember:
                result = Core3DAssetAtlasPreparationResultForeignMember;
                detail = @"A member belongs to a different document or atlas."; break;
            case core3d::asset_atlas::build::Status::UnsupportedSurface:
                result = Core3DAssetAtlasPreparationResultUnsupportedSurface;
                detail = @"A member's finishing used the diagnosed fallback or an unsupported surface."; break;
            default: break;
        }
        DeliverPreparation(completion, result, detail);
        return;
    }
    core3d::asset_atlas::Capture capture;
    if (!core3d::asset_atlas::build::CaptureMembers(_owner->Document(), members, capture)) {
        DeliverPreparation(completion, Core3DAssetAtlasPreparationResultRejected,
                           @"The asset atlas members could not be captured.");
        return;
    }
    for (const auto& entry : capture.members) {
        if (!entry.painted) continue;
        NSString *member = IdentifierText(entry.slot.owner.entity);
        DeliverPreparation(completion,
            Core3DAssetAtlasPreparationResultPaintedRebakeRequired,
            [NSString stringWithFormat:
                @"Member %@ carries image-backed painted content; a texture-preserving rebake is a separate explicit operation.",
                member]);
        return;
    }
    if (_hasAtlas) {
        std::vector<OwnerKey> currentMembers;
        currentMembers.reserve(current.members.size());
        for (const auto& member : current.members) currentMembers.push_back(member.owner);
        if (SameMembers(members, currentMembers)
            && current.resolutionTexels == request.resolutionTexels
            && current.gutterTexels == request.gutterTexels) {
            DeliverPreparation(completion, Core3DAssetAtlasPreparationResultUnchanged,
                               @"The asset atlas already matches these values.");
            return;
        }
    }
    AtlasOpeningState expected = AtlasOpeningState::Open;
    if (!_state.compare_exchange_strong(expected, AtlasOpeningState::Prepared)) {
        DeliverPreparation(completion, Core3DAssetAtlasPreparationResultRejected,
                           @"Opening changed during preparation.");
        return;
    }
    _stagedMembers = std::move(members);
    _stagedSettings = {request.resolutionTexels, request.gutterTexels};
    DeliverPreparation(completion, Core3DAssetAtlasPreparationResultPrepared,
                       @"Asset atlas prepared. Apply is one undoable asset-atlas change.");
}

- (void)applyWithCompletion:(void (^)(Core3DProfileConstructionResult,
                                      NSString *))completion {
    AtlasOpeningState expected = AtlasOpeningState::Prepared;
    if (!NSThread.isMainThread
        || !_state.compare_exchange_strong(expected, AtlasOpeningState::Applying)
        || _owner.IsNull() || !_context) {
        DeliverMutation(completion, Core3DProfileConstructionResultRejected,
                        @"No current asset-atlas preparation.");
        return;
    }
    const auto lease = _context->beginCommandLease(_context->openingFence(), 64, 64);
    const auto result = AtlasMutation(lease,
        lease ? _owner->BuildAssetAtlas(_atlas, _stagedMembers, _stagedSettings)
              : OcctAssetAtlasOutcome::Busy);
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? AtlasOpeningState::Recovery : AtlasOpeningState::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) _context.reset();
    DeliverMutation(completion, result,
        result == Core3DProfileConstructionResultCommitted
            ? @"Asset atlas committed as one undoable change."
            : result == Core3DProfileConstructionResultRecoveryRequired
                ? @"Command close is unknown; native recovery ownership is retained."
                : result == Core3DProfileConstructionResultBusy
                    ? @"The asset atlas edit could not begin a native command."
                    : @"The asset atlas edit was refused without history.");
}

- (void)regenerateWithCompletion:(void (^)(Core3DProfileConstructionResult,
                                           NSString *))completion {
    if (!_hasAtlas) {
        DeliverMutation(completion, Core3DProfileConstructionResultRejected,
                        @"There is no committed asset atlas to regenerate.");
        return;
    }
    AtlasOpeningState value = _state.load();
    while (value == AtlasOpeningState::Open || value == AtlasOpeningState::Prepared) {
        if (_state.compare_exchange_weak(value, AtlasOpeningState::Regenerating)) break;
    }
    if (!NSThread.isMainThread || _state.load() != AtlasOpeningState::Regenerating
        || _owner.IsNull() || !_context) {
        DeliverMutation(completion, Core3DProfileConstructionResultRejected,
                        @"Opening is not available for regeneration.");
        return;
    }
    const auto lease = _context->beginCommandLease(_context->openingFence(), 64, 64);
    const auto result = AtlasMutation(lease,
        lease ? _owner->RegenerateAssetAtlas(_atlas)
              : OcctAssetAtlasOutcome::Busy);
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? AtlasOpeningState::Recovery : AtlasOpeningState::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) _context.reset();
    DeliverMutation(completion, result,
        result == Core3DProfileConstructionResultCommitted
            ? @"Asset atlas regenerated from the current members as one undoable change."
            : result == Core3DProfileConstructionResultRecoveryRequired
                ? @"Command close is unknown; native recovery ownership is retained."
                : result == Core3DProfileConstructionResultBusy
                    ? @"The asset atlas regeneration could not begin a native command."
                    : @"Regeneration was refused without reusing the stale atlas.");
}

- (void)bakePaintedWithCompletion:(void (^)(Core3DProfileConstructionResult,
                                            NSString *))completion {
    if(!_hasAtlas){
        DeliverMutation(completion,Core3DProfileConstructionResultRejected,
                        @"There is no committed asset atlas to bake.");
        return;
    }
    AtlasOpeningState value=_state.load();
    while(value==AtlasOpeningState::Open||value==AtlasOpeningState::Prepared){
        if(_state.compare_exchange_weak(value,AtlasOpeningState::Baking))break;
    }
    if(!NSThread.isMainThread||_state.load()!=AtlasOpeningState::Baking
        ||_owner.IsNull()||!_context){
        DeliverMutation(completion,Core3DProfileConstructionResultRejected,
                        @"Opening is not available for a painted bake.");
        return;
    }
    const auto lease=_context->beginCommandLease(_context->openingFence(),64,64);
    const auto result=PaintedAtlasMutation(lease,
        lease?_owner->BakePaintedAtlas(_atlas):OcctPaintedAtlasBakeOutcome::Busy);
    _state.store(result==Core3DProfileConstructionResultRecoveryRequired
        ?AtlasOpeningState::Recovery:AtlasOpeningState::Settled);
    if(result!=Core3DProfileConstructionResultRecoveryRequired)_context.reset();
    DeliverMutation(completion,result,
        result==Core3DProfileConstructionResultCommitted
            ?@"Painted atlas baked and committed as one undoable change."
            :result==Core3DProfileConstructionResultRecoveryRequired
                ?@"Command close is unknown; native recovery ownership is retained."
                :@"Painted atlas bake was refused without history.");
}

- (BOOL)cancel {
    AtlasOpeningState value = _state.load();
    while (value == AtlasOpeningState::Open || value == AtlasOpeningState::Prepared) {
        if (_state.compare_exchange_weak(value, AtlasOpeningState::Cancelled)) {
            _stagedMembers.clear(); _context.reset(); return YES;
        }
    }
    return NO;
}
@end

#if DEBUG
namespace {
using core3d::retained_finishing::UnwrapPolicy;

UUID FixtureIndexedUUID(std::uint8_t seed, std::size_t index) {
    UUID value{}; value.fill(seed);
    value[0] = std::uint8_t(index); value[1] = std::uint8_t(index >> 8);
    return value;
}

bool FixtureSetUUID(const TDF_Label& label, const char* attributeID, const UUID& value) {
    return !TDataStd_AsciiString::Set(label, Standard_GUID(attributeID),
        TCollection_AsciiString(core3d::retained_solid::UUIDText(value).c_str())).IsNull();
}

struct FixturePart final {
    OwnerKey key;
    TDF_Label owner;
};

struct PlanarRepresentationCensus final {
    int incidences = 0, stored = 0, nonnull = 0;
    int lines = 0, circles = 0, representationEntries = 0;
};

PlanarRepresentationCensus CensusStoredPlanar(const TopoDS_Shape& shape) {
    PlanarRepresentationCensus result;
    for (TopExp_Explorer faces(shape, TopAbs_FACE); faces.More(); faces.Next()) {
        const TopoDS_Face face = TopoDS::Face(faces.Current());
        TopLoc_Location faceLocation;
        const Handle(Geom_Surface) surface = BRep_Tool::Surface(face, faceLocation);
        if (surface.IsNull()
            || GeomAdaptor_Surface(surface).GetType() != GeomAbs_Plane) continue;
        for (TopExp_Explorer edges(face, TopAbs_EDGE); edges.More(); edges.Next()) {
            ++result.incidences;
            TopoDS_Edge edge = TopoDS::Edge(edges.Current());
            if (face.Orientation() == TopAbs_REVERSED) edge.Reverse();
            const TopLoc_Location relative = faceLocation.Predivided(edge.Location());
            const Handle(BRep_TEdge) implementation =
                Handle(BRep_TEdge)::DownCast(edge.TShape());
            if (implementation.IsNull()) continue;
            result.representationEntries += implementation->Curves().Size();
            for (BRep_ListIteratorOfListOfCurveRepresentation it(
                    implementation->Curves()); it.More(); it.Next()) {
                const Handle(BRep_CurveRepresentation)& representation = it.Value();
                if (representation.IsNull()
                    || !representation->IsCurveOnSurface(surface, relative)) continue;
                ++result.stored;
                Handle(Geom2d_Curve) curve = representation->PCurve();
                if (representation->IsCurveOnClosedSurface()
                    && edge.Orientation() == TopAbs_REVERSED)
                    curve = representation->PCurve2();
                if (curve.IsNull()) break;
                ++result.nonnull;
                if (curve->DynamicType() == STANDARD_TYPE(Geom2d_TrimmedCurve))
                    curve = Handle(Geom2d_TrimmedCurve)::DownCast(curve)->BasisCurve();
                if (curve->DynamicType() == STANDARD_TYPE(Geom2d_Line)) ++result.lines;
                if (curve->DynamicType() == STANDARD_TYPE(Geom2d_Circle)) ++result.circles;
                break;
            }
        }
    }
    return result;
}

struct PeriodicEllipseCensus final {
    int stored = 0, ellipses = 0, splines = 0, shifted = 0;
    bool correspondence = true;
    double first = 0, last = 0, centerX = 0, centerY = 0;
    double majorRadius = 0, minorRadius = 0;
};

PeriodicEllipseCensus CensusPeriodicEllipses(const TopoDS_Shape& shape) {
    PeriodicEllipseCensus result;
    const double shiftedFirst = 5 * std::acos(-1.0) / 2;
    const double shiftedLast = 3 * std::acos(-1.0);
    for (TopExp_Explorer faces(shape, TopAbs_FACE); faces.More(); faces.Next()) {
        const TopoDS_Face face = TopoDS::Face(faces.Current());
        TopLoc_Location faceLocation;
        const Handle(Geom_Surface) surface = BRep_Tool::Surface(face, faceLocation);
        if (surface.IsNull()
            || GeomAdaptor_Surface(surface).GetType() != GeomAbs_Plane) continue;
        for (TopExp_Explorer edges(face, TopAbs_EDGE); edges.More(); edges.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(edges.Current());
            TopLoc_Location curveLocation;
            Standard_Real curveFirst = 0, curveLast = 0;
            const Handle(Geom_Curve) curve3d = BRep_Tool::Curve(
                edge, curveLocation, curveFirst, curveLast);
            if (curve3d.IsNull()) continue;
            Handle(Geom_Curve) basis3d = curve3d;
            if (basis3d->DynamicType() == STANDARD_TYPE(Geom_TrimmedCurve))
                basis3d = Handle(Geom_TrimmedCurve)::DownCast(basis3d)->BasisCurve();
            if (basis3d->DynamicType() != STANDARD_TYPE(Geom_Ellipse)) continue;
            const TopLoc_Location relative =
                faceLocation.Predivided(edge.Location());
            const Handle(BRep_TEdge) implementation =
                Handle(BRep_TEdge)::DownCast(edge.TShape());
            if (implementation.IsNull()) continue;
            for (BRep_ListIteratorOfListOfCurveRepresentation it(
                    implementation->Curves()); it.More(); it.Next()) {
                const Handle(BRep_CurveRepresentation)& representation = it.Value();
                if (representation.IsNull()
                    || !representation->IsCurveOnSurface(surface, relative)) continue;
                const Handle(BRep_CurveOnSurface) onSurface =
                    Handle(BRep_CurveOnSurface)::DownCast(representation);
                if (onSurface.IsNull()) continue;
                ++result.stored;
                Handle(Geom2d_Curve) pcurve = onSurface->PCurve();
                if (pcurve.IsNull()) continue;
                if (pcurve->DynamicType() == STANDARD_TYPE(Geom2d_TrimmedCurve))
                    pcurve = Handle(Geom2d_TrimmedCurve)::DownCast(pcurve)->BasisCurve();
                if (pcurve->DynamicType() == STANDARD_TYPE(Geom2d_BSplineCurve)) {
                    ++result.splines;
                    continue;
                }
                const Handle(Geom2d_Ellipse) ellipse =
                    Handle(Geom2d_Ellipse)::DownCast(pcurve);
                if (ellipse.IsNull()) continue;
                ++result.ellipses;
                result.first = onSurface->First();
                result.last = onSurface->Last();
                result.centerX = ellipse->Location().X();
                result.centerY = ellipse->Location().Y();
                result.majorRadius = ellipse->MajorRadius();
                result.minorRadius = ellipse->MinorRadius();
                if (Abs(result.first - shiftedFirst) <= Precision::PConfusion()
                    && Abs(result.last - shiftedLast) <= Precision::PConfusion())
                    ++result.shifted;
                for (const double parameter : {result.first,
                        (result.first + result.last) / 2, result.last}) {
                    gp_Pnt fromCurve = curve3d->Value(parameter);
                    fromCurve.Transform(curveLocation.Transformation());
                    const gp_Pnt2d uv = pcurve->Value(parameter);
                    gp_Pnt fromSurface = surface->Value(uv.X(), uv.Y());
                    fromSurface.Transform(faceLocation.Transformation());
                    if (fromCurve.SquareDistance(fromSurface)
                        > Precision::SquareConfusion()) result.correspondence = false;
                }
            }
        }
    }
    return result;
}

TopoDS_Shape PeriodicEllipseFixture(double first, double last) {
    const Handle(Geom_Ellipse) ellipse = new Geom_Ellipse(
        gp_Elips(gp_Ax2(gp::Origin(), gp::DZ()), 4, 2));
    BRepBuilderAPI_MakeEdge arc(ellipse, first, last);
    const gp_Pnt center = gp::Origin();
    BRepBuilderAPI_MakeEdge toCenter(ellipse->Value(last), center);
    BRepBuilderAPI_MakeEdge fromCenter(center, ellipse->Value(first));
    if (!arc.IsDone() || !toCenter.IsDone() || !fromCenter.IsDone())
        return {};
    BRepBuilderAPI_MakeWire wire;
    wire.Add(arc.Edge());
    wire.Add(toCenter.Edge());
    wire.Add(fromCenter.Edge());
    if (!wire.IsDone()) return {};
    BRepBuilderAPI_MakeFace face(gp_Pln(gp::Origin(), gp::DZ()),
        wire.Wire(), Standard_False);
    if (!face.IsDone()) return {};
    TopoDS_Face result = face.Face();
    TopLoc_Location faceLocation;
    const Handle(Geom_Surface) surface = BRep_Tool::Surface(result, faceLocation);
    if (surface.IsNull()) return {};
    for (TopExp_Explorer edges(result, TopAbs_EDGE); edges.More(); edges.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(edges.Current());
        TopLoc_Location curveLocation;
        Standard_Real curveFirst = 0, curveLast = 0;
        Handle(Geom_Curve) curve3d = BRep_Tool::Curve(
            edge, curveLocation, curveFirst, curveLast);
        if (curve3d.IsNull()) continue;
        if (curve3d->DynamicType() == STANDARD_TYPE(Geom_TrimmedCurve))
            curve3d = Handle(Geom_TrimmedCurve)::DownCast(curve3d)->BasisCurve();
        if (curve3d->DynamicType() != STANDARD_TYPE(Geom_Ellipse)) continue;
        const TopLoc_Location relative =
            faceLocation.Predivided(edge.Location());
        const Handle(BRep_TEdge) implementation =
            Handle(BRep_TEdge)::DownCast(edge.TShape());
        if (implementation.IsNull()) return {};
        BRep_ListOfCurveRepresentation& curves = implementation->ChangeCurves();
        for (BRep_ListIteratorOfListOfCurveRepresentation it(curves);
             it.More();) {
            const Handle(BRep_CurveRepresentation) representation = it.Value();
            if (!representation.IsNull()
                && representation->IsCurveOnSurface(surface, relative))
                curves.Remove(it);
            else it.Next();
        }
        const Handle(Geom2d_Ellipse) pcurve = new Geom2d_Ellipse(
            gp_Elips2d(gp_Ax2d(gp::Origin2d(), gp::DX2d()), 4, 2));
        Handle(BRep_CurveOnSurface) stored = new BRep_CurveOnSurface(
            pcurve, surface, relative);
        stored->SetRange(first, last);
        curves.Append(stored);
        BRep_Builder topology;
        topology.Range(edge, first, last);
        TopoDS_Vertex start, finish;
        TopExp::Vertices(TopoDS::Edge(edge.Oriented(TopAbs_FORWARD)),
            start, finish);
        if (!start.IsNull())
            topology.UpdateVertex(start, first, edge, Precision::Confusion());
        if (!finish.IsNull())
            topology.UpdateVertex(finish, last, edge, Precision::Confusion());
        implementation->Modified(Standard_True);
        return result;
    }
    return {};
}

NSDictionary<NSString *, id> *PeriodicEllipseScaleControl(
    double first, double last) {
    const TopoDS_Shape source = PeriodicEllipseFixture(first, last);
    gp_Trsf scale;
    scale.SetScale(gp::Origin(), 1000);
    TopoDS_Shape scaled;
    const bool scaleBuilt = !source.IsNull()
        && core3d::analytic_boolean_scale::Scale(source, scale, scaled);
    const auto before = CensusPeriodicEllipses(source);
    const auto after = CensusPeriodicEllipses(scaled);
    return @{ @"scaleBuilt": @(scaleBuilt),
        @"sourceStored": @(before.stored), @"sourceEllipses": @(before.ellipses),
        @"sourceFirst": @(before.first), @"sourceLast": @(before.last),
        @"sourceCenterX": @(before.centerX), @"sourceCenterY": @(before.centerY),
        @"sourceMajorRadius": @(before.majorRadius),
        @"sourceMinorRadius": @(before.minorRadius),
        @"sourceCorrespondence": @(before.correspondence),
        @"outputStored": @(after.stored), @"outputEllipses": @(after.ellipses),
        @"outputSplines": @(after.splines), @"outputFirst": @(after.first),
        @"outputLast": @(after.last), @"outputCenterX": @(after.centerX),
        @"outputCenterY": @(after.centerY),
        @"outputMajorRadius": @(after.majorRadius),
        @"outputMinorRadius": @(after.minorRadius),
        @"outputCorrespondence": @(after.correspondence) };
}

NSString *ShapeSHA256(const TopoDS_Shape& shape, NSData **bytesOut);

struct V3Difference final {
    int count = 0, line = 0, geometryIndex = 0, geometryType = 0;
    std::string section, beforeLine, afterLine;
    std::string beforeRepresentations, afterRepresentations;
    std::string beforeFlags, afterFlags;
};

std::vector<std::string> V3Lines(const std::string& text) {
    std::vector<std::string> result;
    std::istringstream stream(text);
    for (std::string line; std::getline(stream, line);) result.push_back(line);
    return result;
}

std::string LimitedLine(const std::string& value) {
    return value.substr(0, std::min<std::size_t>(value.size(), 240));
}

struct RepresentationIndices final {
    TColStd_IndexedMapOfTransient surfaces;
    TopTools_LocationSet locations;
};

RepresentationIndices IndexRepresentations(const TopoDS_Shape& shape) {
    RepresentationIndices result;
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    for (int index = 1; index <= edges.Extent(); ++index) {
        const Handle(BRep_TEdge) edge = Handle(BRep_TEdge)::DownCast(
            TopoDS::Edge(edges(index)).TShape());
        if (edge.IsNull()) continue;
        for (BRep_ListIteratorOfListOfCurveRepresentation it(edge->Curves());
             it.More(); it.Next()) {
            const Handle(BRep_CurveRepresentation)& representation = it.Value();
            if (representation.IsNull()) continue;
            result.locations.Add(representation->Location());
            if (representation->IsCurveOnSurface()
                && !representation->Surface().IsNull())
                result.surfaces.Add(representation->Surface());
        }
    }
    return result;
}

std::string RepresentationList(const TopoDS_Edge& edge,
                               const RepresentationIndices& indices) {
    const Handle(BRep_TEdge) implementation =
        Handle(BRep_TEdge)::DownCast(edge.TShape());
    if (implementation.IsNull()) return "null-edge";
    std::ostringstream out;
    out << std::hexfloat;
    int ordinal = 0;
    for (BRep_ListIteratorOfListOfCurveRepresentation it(
            implementation->Curves()); it.More(); it.Next(), ++ordinal) {
        if (ordinal) out << ';';
        const Handle(BRep_CurveRepresentation)& representation = it.Value();
        if (representation.IsNull()) { out << ordinal << ":null"; continue; }
        out << ordinal << ':';
        if (representation->IsCurve3D()) out << "c3";
        else if (representation->IsCurveOnClosedSurface()) out << "closed";
        else if (representation->IsCurveOnSurface()) out << "cos";
        else if (representation->IsRegularity()) out << "reg";
        else if (representation->IsPolygon3D()) out << "p3";
        else if (representation->IsPolygonOnClosedTriangulation()) out << "pct";
        else if (representation->IsPolygonOnTriangulation()) out << "pt";
        else if (representation->IsPolygonOnClosedSurface()) out << "pcs";
        else if (representation->IsPolygonOnSurface()) out << "ps";
        else out << "other";
        out << ",l" << indices.locations.Index(representation->Location());
        const Handle(BRep_CurveOnSurface) onSurface =
            Handle(BRep_CurveOnSurface)::DownCast(representation);
        if (!onSurface.IsNull()) {
            out << ",s" << indices.surfaces.FindIndex(onSurface->Surface())
                << ",r=" << onSurface->First() << ',' << onSurface->Last();
            gp_Pnt2d first, last;
            onSurface->UVPoints(first, last);
            out << ",uv=" << first.X() << ',' << first.Y() << ','
                << last.X() << ',' << last.Y();
        }
    }
    return LimitedLine(out.str());
}

std::string EdgeFlags(const TopoDS_Edge& edge) {
    const Handle(BRep_TEdge) implementation =
        Handle(BRep_TEdge)::DownCast(edge.TShape());
    std::ostringstream out;
    out << "F" << edge.Free() << " M" << edge.Modified()
        << " C" << edge.Checked() << " O" << edge.Orientable()
        << " Cl" << edge.Closed() << " I" << edge.Infinite()
        << " Cv" << edge.Convex();
    if (!implementation.IsNull())
        out << " SR" << implementation->SameRange()
            << " SP" << implementation->SameParameter()
            << " D" << implementation->Degenerated();
    out << " tol=" << std::hexfloat << BRep_Tool::Tolerance(edge);
    return out.str();
}

V3Difference CompareV3(const TopoDS_Shape& before,
                       const TopoDS_Shape& after) {
    V3Difference result;
    std::string beforeText, afterText;
    if (!core3d::retained_part_boolean::ExactShapeBytes(before, beforeText)
        || !core3d::retained_part_boolean::ExactShapeBytes(after, afterText))
        return result;
    const auto beforeLines = V3Lines(beforeText), afterLines = V3Lines(afterText);
    const std::size_t count = std::max(beforeLines.size(), afterLines.size());
    std::size_t first = count;
    for (std::size_t index = 0; index < count; ++index) {
        const std::string lhs = index < beforeLines.size() ? beforeLines[index] : "<EOF>";
        const std::string rhs = index < afterLines.size() ? afterLines[index] : "<EOF>";
        if (lhs != rhs) {
            ++result.count;
            if (first == count) first = index;
        }
    }
    if (!result.count) return result;
    result.line = int(first + 1);
    result.beforeLine = LimitedLine(first < beforeLines.size()
        ? beforeLines[first] : "<EOF>");
    result.afterLine = LimitedLine(first < afterLines.size()
        ? afterLines[first] : "<EOF>");
    const std::array<const char *, 8> headers = {"Locations", "Curve2ds",
        "Curves", "Polygon3D", "PolygonOnTriangulations", "Surfaces",
        "Triangulations", "TShapes"};
    std::size_t sectionLine = first;
    for (std::size_t index = first + 1; index-- > 0;) {
        const std::string& line = beforeLines[index];
        const auto found = std::find_if(headers.begin(), headers.end(),
            [&](const char *header) { return line.rfind(header, 0) == 0; });
        if (found != headers.end()) {
            result.section = *found;
            sectionLine = index;
            break;
        }
    }
    if (result.section == "Curve2ds" || result.section == "Curves"
        || result.section == "Surfaces") {
        result.geometryIndex = int(first - sectionLine);
        std::istringstream geometry(result.beforeLine);
        geometry >> result.geometryType;
    }
    TopTools_IndexedMapOfShape beforeEdges, afterEdges;
    TopExp::MapShapes(before, TopAbs_EDGE, beforeEdges);
    TopExp::MapShapes(after, TopAbs_EDGE, afterEdges);
    const RepresentationIndices beforeIndices = IndexRepresentations(before);
    const RepresentationIndices afterIndices = IndexRepresentations(after);
    const int edgeCount = std::min(beforeEdges.Extent(), afterEdges.Extent());
    for (int index = 1; index <= edgeCount; ++index) {
        const TopoDS_Edge beforeEdge = TopoDS::Edge(beforeEdges(index));
        const TopoDS_Edge afterEdge = TopoDS::Edge(afterEdges(index));
        const std::string beforeReps = RepresentationList(beforeEdge, beforeIndices);
        const std::string afterReps = RepresentationList(afterEdge, afterIndices);
        const std::string beforeFlags = EdgeFlags(beforeEdge);
        const std::string afterFlags = EdgeFlags(afterEdge);
        if (beforeReps != afterReps || beforeFlags != afterFlags) {
            result.beforeRepresentations = beforeReps;
            result.afterRepresentations = afterReps;
            result.beforeFlags = beforeFlags;
            result.afterFlags = afterFlags;
            break;
        }
    }
    return result;
}

NSDictionary<NSString *, id> *LoftBoreScaleControl(double unit) {
    const double scale = .001 / unit;
    core3d::rectangular_loft::Definition definition;
    definition.loftIdentifier = 1;
    definition.correspondence = {10, 11, 12, 13};
    definition.dimensionMetersPerUnit = unit;
    definition.stations = {
        {100, {200, 201, 202, 203}, definition.correspondence,
            0, -0.0, 0, 20 * scale, 12 * scale},
        {103, {212, 213, 214, 215}, definition.correspondence,
            20 * scale, -0.0, 0, 20 * scale, 12 * scale},
        {101, {204, 205, 206, 207}, definition.correspondence,
            60 * scale, 6 * scale, 0, 30 * scale, 20 * scale},
        {104, {216, 217, 218, 219}, definition.correspondence,
            100 * scale, -0.0, 0, 20 * scale, 12 * scale},
        {102, {208, 209, 210, 211}, definition.correspondence,
            120 * scale, -0.0, 0, 20 * scale, 12 * scale},
    };
    core3d::rectangular_loft::Admission admission;
    const auto prepared = core3d::rectangular_loft::Prepare(definition, admission);
    const std::atomic_bool stop(false);
    core3d::rectangular_loft::SolidResult loft;
    const auto loftStatus = core3d::rectangular_loft::Build(prepared, stop, loft);
    core3d::analytic_boolean::Result bore;
    auto boreStatus = core3d::analytic_boolean::Status::UnsupportedSource;
    if (loftStatus == core3d::rectangular_loft::BuildStatus::Built) {
        BRepBuilderAPI_Copy detached(loft.solid, Standard_True, Standard_False);
        core3d::analytic_boolean::Recipe recipe;
        recipe.metersPerUnit = unit;
        recipe.operation = core3d::analytic_boolean::Operation::Difference;
        recipe.tool.identifier = 1;
        recipe.tool.kind = core3d::analytic_boolean::OperandKind::Cylinder;
        recipe.tool.axis = core3d::analytic_boolean::Axis::X;
        recipe.tool.point = {0, 6 * scale, 60 * scale};
        recipe.tool.radius = 3 * scale;
        boreStatus = core3d::analytic_boolean::Build(
            detached.Shape(), recipe, stop, bore);
    }
    const auto periodic = CensusPeriodicEllipses(bore.solid);
    NSString *rawDigest = ShapeSHA256(bore.solid, nullptr);
    TopoDS_Shape reopened;
    if (!bore.solid.IsNull()) {
        std::ostringstream writer(std::ios::binary);
        BinTools::Write(bore.solid, writer, Standard_False, Standard_False,
            BinTools_FormatVersion_VERSION_4);
        std::istringstream reader(writer.str(), std::ios::binary);
        BinTools::Read(reopened, reader);
    }
    const auto reopenedPeriodic = CensusPeriodicEllipses(reopened);
    NSString *reopenedDigest = ShapeSHA256(reopened, nullptr);
    const V3Difference difference = CompareV3(bore.solid, reopened);
    NSMutableArray *bounds = [NSMutableArray arrayWithCapacity:6];
    for (double value : bore.resultBounds) [bounds addObject:@(value)];
    return @{ @"admission": @(int(admission)),
        @"loftStatus": @(int(loftStatus)), @"boreStatus": @(int(boreStatus)),
        @"loftValid": @(!loft.solid.IsNull()
            && BRepCheck_Analyzer(loft.solid, Standard_True).IsValid()),
        @"boreValid": @(!bore.solid.IsNull()
            && BRepCheck_Analyzer(bore.solid, Standard_True).IsValid()),
        @"loftVolume": @(loft.volume), @"sourceVolume": @(bore.sourceVolume),
        @"resultVolume": @(bore.resultVolume), @"resultBounds": bounds,
        @"storedPeriodic": @(periodic.stored),
        @"analyticEllipses": @(periodic.ellipses),
        @"periodicSplines": @(periodic.splines),
        @"shiftedEllipses": @(periodic.shifted),
        @"parameterCorrespondence": @(periodic.correspondence),
        @"roundTripStable": @(!reopened.IsNull()
            && [rawDigest isEqualToString:reopenedDigest]),
        @"rawDigest": rawDigest ?: @"", @"reopenedDigest": reopenedDigest ?: @"",
        @"v3DifferenceCount": @(difference.count),
        @"v3FirstDifferenceLine": @(difference.line),
        @"v3Section": [NSString stringWithUTF8String:difference.section.c_str()],
        @"v3BeforeLine": [NSString stringWithUTF8String:difference.beforeLine.c_str()],
        @"v3AfterLine": [NSString stringWithUTF8String:difference.afterLine.c_str()],
        @"v3GeometryIndex": @(difference.geometryIndex),
        @"v3GeometryType": @(difference.geometryType),
        @"v3BeforeRepresentations": [NSString stringWithUTF8String:
            difference.beforeRepresentations.c_str()],
        @"v3AfterRepresentations": [NSString stringWithUTF8String:
            difference.afterRepresentations.c_str()],
        @"v3BeforeFlags": [NSString stringWithUTF8String:
            difference.beforeFlags.c_str()],
        @"v3AfterFlags": [NSString stringWithUTF8String:
            difference.afterFlags.c_str()],
        @"reopenedEllipses": @(reopenedPeriodic.ellipses),
        @"reopenedSplines": @(reopenedPeriodic.splines),
        @"reopenedShifted": @(reopenedPeriodic.shifted),
        @"reopenedFirst": @(reopenedPeriodic.first),
        @"reopenedLast": @(reopenedPeriodic.last),
        @"reopenedMajorRadius": @(reopenedPeriodic.majorRadius),
        @"reopenedMinorRadius": @(reopenedPeriodic.minorRadius) };
}

TopoDS_Shape AnalyticFixtureShape(int kind, double unit) {
    const double n = .001 / unit;
    TopoDS_Shape shape = kind == 1
        ? BRepPrimAPI_MakeCylinder(10 * n, 25 * n).Shape()
        : kind == 2 ? BRepPrimAPI_MakeBox(12 * n, 12 * n, 12 * n).Shape()
                    : BRepPrimAPI_MakeBox(40 * n, 30 * n, 20 * n).Shape();
    std::vector<std::array<double, 3>> centers;
    if (kind == 1) centers = {{{-4 * n, 0, 0}}};
    else if (kind == 2) centers = {{{4 * n, 6 * n, 0}}};
    else centers = {{{12 * n, 15 * n, 0}}, {{28 * n, 15 * n, 0}}};
    const double radius = (kind == 2 ? 1.5 : kind == 1 ? 2 : 3) * n;
    const std::atomic_bool stop(false);
    for (const auto& center : centers) {
        core3d::analytic_boolean::Recipe recipe;
        recipe.metersPerUnit = unit;
        recipe.operation = core3d::analytic_boolean::Operation::Difference;
        recipe.tool.identifier = 1;
        recipe.tool.kind = core3d::analytic_boolean::OperandKind::Cylinder;
        recipe.tool.axis = core3d::analytic_boolean::Axis::Z;
        recipe.tool.point = center;
        recipe.tool.radius = radius;
        core3d::analytic_boolean::Result cut;
        const auto status = core3d::analytic_boolean::Build(shape, recipe, stop, cut);
        if (status != core3d::analytic_boolean::Status::Built)
            throw std::invalid_argument("analytic pcurve fixture cut kind "
                + std::to_string(kind) + " unit " + std::to_string(unit)
                + " status " + std::to_string(int(status)));
        shape = cut.solid;
    }
    return shape;
}

NSString *ShapeSHA256(const TopoDS_Shape& shape, NSData **bytesOut) {
    std::string bytes;
    if (!core3d::retained_part_boolean::ExactShapeBytes(shape, bytes)) return nil;
    NSData *data = [NSData dataWithBytes:bytes.data() length:bytes.size()];
    if (bytesOut) *bytesOut = data;
    unsigned char digest[CC_SHA256_DIGEST_LENGTH]{};
    CC_SHA256(bytes.data(), CC_LONG(bytes.size()), digest);
    NSMutableString *hex = [NSMutableString stringWithCapacity:64];
    for (unsigned char value : digest) [hex appendFormat:@"%02x", value];
    return hex;
}

NSDictionary<NSString *, NSNumber *> *CensusDictionary(
    const PlanarRepresentationCensus& value) {
    return @{@"incidences": @(value.incidences), @"stored": @(value.stored),
        @"nonnull": @(value.nonnull), @"lines": @(value.lines),
        @"circles": @(value.circles),
        @"representationEntries": @(value.representationEntries)};
}

enum class PlanarControl { RemoveLine, RemoveCircle, StoredNull, Paired };

bool AlterFirstPlanarRepresentation(TopoDS_Shape& shape, PlanarControl control) {
    for (TopExp_Explorer faces(shape, TopAbs_FACE); faces.More(); faces.Next()) {
        const TopoDS_Face face = TopoDS::Face(faces.Current());
        TopLoc_Location faceLocation;
        const Handle(Geom_Surface) surface = BRep_Tool::Surface(face, faceLocation);
        if (surface.IsNull()
            || GeomAdaptor_Surface(surface).GetType() != GeomAbs_Plane) continue;
        for (TopExp_Explorer edges(face, TopAbs_EDGE); edges.More(); edges.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(edges.Current());
            const TopLoc_Location relative = faceLocation.Predivided(edge.Location());
            const Handle(BRep_TEdge) implementation =
                Handle(BRep_TEdge)::DownCast(edge.TShape());
            if (implementation.IsNull()) continue;
            BRep_ListOfCurveRepresentation& curves = implementation->ChangeCurves();
            for (BRep_ListIteratorOfListOfCurveRepresentation it(curves);
                 it.More(); it.Next()) {
                const Handle(BRep_CurveRepresentation) representation = it.Value();
                if (representation.IsNull()
                    || !representation->IsCurveOnSurface(surface, relative)
                    || representation->PCurve().IsNull()) continue;
                Handle(Geom2d_Curve) basis = representation->PCurve();
                if (basis->DynamicType() == STANDARD_TYPE(Geom2d_TrimmedCurve))
                    basis = Handle(Geom2d_TrimmedCurve)::DownCast(basis)->BasisCurve();
                const bool circle = basis->DynamicType() == STANDARD_TYPE(Geom2d_Circle);
                if ((control == PlanarControl::RemoveLine && circle)
                    || (control == PlanarControl::RemoveCircle && !circle)) continue;
                if (control == PlanarControl::RemoveLine
                    || control == PlanarControl::RemoveCircle) {
                    curves.Remove(it);
                    return true;
                }
                const Handle(BRep_CurveOnSurface) onSurface =
                    Handle(BRep_CurveOnSurface)::DownCast(representation);
                if (onSurface.IsNull()) return false;
                if (control == PlanarControl::StoredNull) {
                    onSurface->PCurve(Handle(Geom2d_Curve)());
                    return true;
                }
                Handle(Geom2d_Curve) first =
                    Handle(Geom2d_Curve)::DownCast(representation->PCurve()->Copy());
                Handle(Geom2d_Curve) second =
                    Handle(Geom2d_Curve)::DownCast(representation->PCurve()->Copy());
                Handle(BRep_CurveOnClosedSurface) paired =
                    new BRep_CurveOnClosedSurface(first, second, surface,
                        relative, GeomAbs_C0);
                paired->SetRange(onSurface->First(), onSurface->Last());
                curves.Remove(it);
                curves.Append(paired);
                return true;
            }
        }
    }
    return false;
}

NSDictionary<NSString *, NSNumber *> *AnalyticScaleControls() {
    gp_Trsf scale; scale.SetScale(gp::Origin(), 1000);
    TopoDS_Shape absentLine = BRepPrimAPI_MakeBox(0.04, 0.03, 0.02).Shape();
    TopoDS_Shape absentCircle = BRepPrimAPI_MakeCylinder(0.01, 0.025).Shape();
    TopoDS_Shape storedNull = BRepPrimAPI_MakeBox(0.04, 0.03, 0.02).Shape();
    TopoDS_Shape paired = BRepPrimAPI_MakeBox(0.04, 0.03, 0.02).Shape();
    gp_Trsf move; move.SetTranslation(gp_Vec(0.01, -0.02, 0.03));
    paired.Location(TopLoc_Location(move));
    const bool alteredLine = AlterFirstPlanarRepresentation(
        absentLine, PlanarControl::RemoveLine);
    const bool alteredCircle = AlterFirstPlanarRepresentation(
        absentCircle, PlanarControl::RemoveCircle);
    const bool alteredNull = AlterFirstPlanarRepresentation(
        storedNull, PlanarControl::StoredNull);
    const bool alteredPair = AlterFirstPlanarRepresentation(
        paired, PlanarControl::Paired);
    TopoDS_Shape lineResult, circleResult, nullResult, pairResult;
    const bool lineScaled = core3d::analytic_boolean_scale::Scale(
        absentLine, scale, lineResult);
    const bool circleScaled = core3d::analytic_boolean_scale::Scale(
        absentCircle, scale, circleResult);
    const bool nullScaled = core3d::analytic_boolean_scale::Scale(
        storedNull, scale, nullResult);
    const bool pairScaled = core3d::analytic_boolean_scale::Scale(
        paired, scale, pairResult);
    const auto line = CensusStoredPlanar(lineResult);
    const auto circle = CensusStoredPlanar(circleResult);
    const auto null = CensusStoredPlanar(nullResult);
    const auto pair = CensusStoredPlanar(pairResult);
    return @{@"absentLineStayedAbsent": @(
            alteredLine && lineScaled && line.stored < line.incidences),
        @"absentCircleStayedAbsent": @(
            alteredCircle && circleScaled && circle.stored < circle.incidences),
        @"storedNullStayedStoredNull": @(
            alteredNull && nullScaled && null.stored == null.incidences
            && null.nonnull < null.stored),
        @"pairedReversedLocatedPreserved": @(
            alteredPair && pairScaled && pair.stored == pair.incidences
            && pair.nonnull == pair.stored)};
}

// One literal primitive with stored triangulation, a retained-solid record and
// a real row-268 finishing receipt, each committed in its own command. Every
// member uses production saved-cut recipe geometry so the selected member's
// production source editor can apply a dimension change. This mirrors
// AssetAtlasProbe::AddPart; the shapes and the
// finishing settings are the frozen row-271 fixture values.
FixturePart FixtureAddPart(const Handle(TDocStd_Document)& doc, int kind,
                           std::uint8_t seed, UnwrapPolicy requested, double unit) {
    if (doc.IsNull() || doc->HasOpenCommand())
        throw std::invalid_argument("atlas fixture command");
    const double n = .001 / unit;
    TopoDS_Shape shape;
    if (kind == 1) shape = BRepPrimAPI_MakeCylinder(10 * n, 25 * n).Shape();
    else if (kind == 2) shape = BRepPrimAPI_MakeBox(12 * n, 12 * n, 12 * n).Shape();
    else shape = BRepPrimAPI_MakeBox(40 * n, 30 * n, 20 * n).Shape();
    FixturePart part;
    UUID documentID{}; documentID.fill(1);
    const UUID entity = FixtureIndexedUUID(seed, 1), definition = FixtureIndexedUUID(seed, 2);
    part.key = {documentID, entity, definition};
    core3d::retained_boolean::Program program;
    program.source.document = documentID;
    program.source.entity = entity;
    program.source.definition = definition;
    program.source.sourceFeature = FixtureIndexedUUID(seed, 3);
    program.source.derivedFeature = FixtureIndexedUUID(seed, 4);
    program.source.family = 1;
    program.source.metersPerUnit = unit;
    core3d::profile::Parameters source;
    source.metersPerUnit = unit;
    if (kind == 1) {
        source.definition.depth = 25 * n;
        source.definition.circle = core3d::ProfileCircularSection{{0, 0}, 10 * n, 0};
    } else {
        const double width = (kind == 2 ? 12 : 40) * n;
        const double depth = (kind == 2 ? 12 : 30) * n;
        source.definition.depth = (kind == 2 ? 12 : 20) * n;
        source.definition.points = {{0, 0}, {width, 0}, {width, depth}, {0, depth}};
    }
    if (!core3d::profile::Encode(source, program.source.values))
        throw std::invalid_argument("atlas fixture source recipe");
    program.source.schema = std::uint32_t(core3d::profile::SchemaFor(source));
    core3d::retained_boolean::Step pilot;
    pilot.operand.identifier = 1;
    pilot.operand.axis = core3d::analytic_boolean::Axis::Z;
    pilot.operand.point = kind == 1
        ? std::array<double, 3>{{-4 * n, 0, 0}}
        : kind == 2 ? std::array<double, 3>{{4 * n, 6 * n, 0}}
                    : std::array<double, 3>{{12 * n, 15 * n, 0}};
    pilot.operand.radius = (kind == 2 ? 1.5 : kind == 1 ? 2 : 3) * n;
    auto second = pilot;
    second.operand.identifier = 2;
    second.operand.point = kind == 1
        ? std::array<double, 3>{{4 * n, 0, 0}}
        : kind == 2 ? std::array<double, 3>{{8 * n, 6 * n, 0}}
                    : std::array<double, 3>{{28 * n, 15 * n, 0}};
    program.steps = {pilot}; if (kind == 0) program.steps.push_back(second);
    program.nextOperandID = kind == 0 ? 3 : 2;
    if (!core3d::retained_boolean::Valid(program))
        throw std::invalid_argument("atlas fixture program");
    core3d::saved_cut_source_edit::Patch editableSource;
    if (kind == 1) editableSource = core3d::saved_cut_source_values::CirclePatch{};
    else editableSource = core3d::saved_cut_source_values::PolygonPatch{};
    if (!core3d::saved_boolean_build::SourcePatch(program, editableSource))
        throw std::invalid_argument("atlas fixture source editing");
    BRepBuilderAPI_Copy retainedCopy(shape, Standard_True, Standard_False);
    if (!retainedCopy.IsDone() || retainedCopy.Shape().IsNull())
        throw std::invalid_argument("atlas fixture retained base");
    const TopoDS_Shape retainedBase = retainedCopy.Shape();
    const std::atomic_bool stop(false);
    shape = retainedBase;
    for (const auto& step : program.steps) {
        core3d::analytic_boolean::Recipe recipe;
        recipe.metersPerUnit = unit;
        recipe.operation = step.operation;
        recipe.tool = step.operand;
        core3d::analytic_boolean::Result cut;
        if (core3d::analytic_boolean::Build(shape, recipe, stop, cut)
            != core3d::analytic_boolean::Status::Built)
            throw std::invalid_argument("atlas fixture production cut");
        shape = cut.solid;
    }
    // Mesh BEFORE the retained record exists so the row-268 geometry fence
    // (whose digest covers stored triangulation) stays consistent.
    BRepMesh_IncrementalMesh mesher(shape, 2.5e-4 * n, Standard_False, 0.5, Standard_True);
    const auto tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    doc->NewCommand();
    part.owner = tool->AddShape(shape, Standard_False);
    if (part.owner.IsNull()) throw std::invalid_argument("atlas fixture owner");
    if (!FixtureSetUUID(part.owner, "0074F7C2-9EAA-4F89-B2DE-8716E155FF62", entity)
        || !FixtureSetUUID(part.owner, "3611F2B2-C694-4E12-AED8-A2A97A3D283B", definition))
        throw std::invalid_argument("atlas fixture identity");
    TDataStd_Integer::Set(part.owner,
        Standard_GUID("67E669F4-00C0-4C45-BC55-9CC5DA22A2B5"), 1);
    TDataStd_Name::Set(part.owner, TCollection_ExtendedString(
        kind == 1 ? "E2 atlas cylinder" : kind == 2 ? "E2 atlas painted box" : "E2 atlas box"));
    // Retained v1 program record through the public DEBUG install seam. The
    // selected member owns a real production Difference result and retained base.
    if (!core3d::native_opening::debug::Core3DDebugInstallRetainedSolidSeedRecord(
            doc, part.owner, program, shape, retainedBase) || !doc->CommitCommand())
        throw std::invalid_argument("atlas fixture retained admission");
    doc->NewCommand();
    core3d::retained_finishing::producer::Capture capture;
    if (!core3d::retained_finishing::producer::CaptureSource(doc, part.key, capture))
        throw std::invalid_argument("atlas fixture capture");
    TDF_Label ownerLabel;
    if (!core3d::retained_finishing::owner::ResolveOwnerLabel(doc, part.key, ownerLabel))
        throw std::invalid_argument("atlas fixture owner resolve");
    core3d::retained_finishing::producer::Settings settings;
    settings.requested = requested;
    settings.resolutionTexels = 1024;
    settings.gutterTexels = 2;
    core3d::retained_finishing::Definition candidate; std::string diagnosis;
    if (core3d::retained_finishing::producer::BuildDerivative(
            doc, ownerLabel, capture, settings, candidate, diagnosis)
        != core3d::retained_finishing::producer::Status::Produced)
        throw std::invalid_argument("atlas fixture receipt produce");
    core3d::retained_finishing::owner::Staging staging;
    if (core3d::retained_finishing::owner::Prepare(staging, doc, candidate, capture.source)
        != core3d::retained_finishing::owner::Outcome::Prepared)
        throw std::invalid_argument("atlas fixture receipt prepare");
    core3d::retained_finishing::SourceRevision observed;
    {
        core3d::retained_finishing::producer::Capture fenced;
        if (!core3d::retained_finishing::producer::CaptureSource(doc, part.key, fenced))
            throw std::invalid_argument("atlas fixture re-fence");
        observed = fenced.source;
    }
    if (core3d::retained_finishing::owner::Commit(staging, doc, observed)
        != core3d::retained_finishing::owner::Outcome::Committed
        || !doc->CommitCommand())
        throw std::invalid_argument("atlas fixture receipt commit");
    return part;
}

// Smallest honest image-backed painted content: a real PNG buffer behind a
// Common diffuse texture on the part's visual material.
void FixturePaintPart(const Handle(TDocStd_Document)& doc, const FixturePart& part) {
    if (doc.IsNull() || doc->HasOpenCommand())
        throw std::invalid_argument("atlas fixture paint command");
    NSData* png = [[NSData alloc] initWithBase64EncodedString:
        @"iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAYAAACqaXHeAAAApklEQVR42u3aQQ3AQAzEwFyZF3kKY06qTWAtK8+cndmBnJ1X7j9y/AYKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNAXQApoCaAFNAbSApgBaQFMALaApgBbQnJml/wF2vQsoQAG0gKYAWkBTAC2gKYAW0BRAC2gKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNL8P8AESbQf6Ta5RUwAAAABJRU5ErkJggg=="
        options:0];
    if (png.length == 0) throw std::invalid_argument("atlas fixture paint bytes");
    Handle(NCollection_Buffer) buffer = new NCollection_Buffer(
        NCollection_BaseAllocator::CommonBaseAllocator(), png.length);
    if (buffer.IsNull()) throw std::invalid_argument("atlas fixture paint buffer");
    std::memcpy(buffer->ChangeData(), png.bytes, png.length);
    const auto tool = XCAFDoc_DocumentTool::VisMaterialTool(doc->Main());
    if (tool.IsNull()) throw std::invalid_argument("atlas fixture paint tool");
    Handle(XCAFDoc_VisMaterial) material = new XCAFDoc_VisMaterial();
    XCAFDoc_VisMaterialCommon common = material->ConvertToCommonMaterial();
    common.DiffuseTexture = new Image_Texture(
        buffer, TCollection_AsciiString("asset-atlas-painted-fixture"));
    material->SetCommonMaterial(common);
    doc->NewCommand();
    const TDF_Label materialLabel = tool->AddMaterial(
        material, TCollection_AsciiString("asset atlas painted fixture"));
    if (materialLabel.IsNull()) throw std::invalid_argument("atlas fixture paint material");
    tool->SetShapeMaterial(part.owner, materialLabel);
    if (!doc->CommitCommand()) throw std::invalid_argument("atlas fixture paint commit");
}

// The frozen row-278a fixture: the row-271 two-part box/cylinder atlas members
// plus one painted non-member part, the atlas built over the two members with
// the frozen 2048/4 settings in exactly one command.
void StageAssetAtlasFixture(const Handle(TDocStd_Document)& doc, double unit) {
    using namespace core3d::asset_atlas;
    doc->ChangeStorageFormatVersion(TDocStd_FormatVersion(12));
    (void)XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    (void)XCAFDoc_DocumentTool::ColorTool(doc->Main());
    (void)XCAFDoc_DocumentTool::VisMaterialTool(doc->Main());
    XCAFDoc_DocumentTool::SetLengthUnit(doc, unit);
    UUID documentID{}; documentID.fill(1);
    if (!FixtureSetUUID(doc->Main(), "74386E4E-F620-498F-8092-E6D883AF33A4", documentID))
        throw std::invalid_argument("atlas fixture document identity");
    doc->SetUndoLimit(40); doc->ClearUndos();
    // The ready-state fixture selects identity seed 0x22; bind that stable
    // identity to the production-editable box while preserving member order.
    const FixturePart box = FixtureAddPart(doc, 0, 0x22, UnwrapPolicy::Planar, unit);
    const FixturePart cylinder = FixtureAddPart(doc, 1, 0x21, UnwrapPolicy::Cylindrical, unit);
    const FixturePart painted = FixtureAddPart(doc, 2, 0x23, UnwrapPolicy::Planar, unit);
    FixturePaintPart(doc, painted);
    Key atlas; atlas.document = documentID; atlas.atlas = FixtureIndexedUUID(0x77, 1);
    const std::vector<OwnerKey> members{box.key, cylinder.key};
    Capture capture;
    if (!build::CaptureMembers(doc, members, capture))
        throw std::invalid_argument("atlas fixture member capture");
    Definition candidate; std::vector<MemberUVAssignment> assignments;
    std::string diagnosis;
    if (build::BuildAtlas(doc, atlas, capture, {2048, 4},
            candidate, assignments, diagnosis) != build::Status::Built)
        throw std::invalid_argument("atlas fixture build");
    std::vector<Member> observed;
    if (!build::ObserveMembers(doc, candidate.members, observed))
        throw std::invalid_argument("atlas fixture observe");
    doc->NewCommand();
    owner::Staging staging;
    auto outcome = owner::Prepare(staging, doc, candidate, assignments, observed);
    if (outcome == owner::Outcome::Prepared) {
        std::vector<Member> fenced;
        if (!build::ObserveMembers(doc, candidate.members, fenced))
            outcome = owner::Outcome::StaleSource;
        else
            outcome = owner::Commit(staging, doc, fenced);
    }
    if (outcome != owner::Outcome::Committed) {
        owner::Cancel(staging);
        doc->AbortCommand();
        throw std::invalid_argument("atlas fixture owner commit");
    }
    if (!doc->CommitCommand()) throw std::invalid_argument("atlas fixture commit command");
    doc->ClearUndos();
    if (!Core3DValidateRetainedFinishingDocument(doc)
        || !Core3DValidateAssetAtlasDocument(doc))
        throw std::invalid_argument("atlas fixture validation");
}
} // namespace
#endif

#if DEBUG
namespace {
// Local mirror of the file-local Core3DCreateDebugBinXCAFFixture helper in
// Core3DViewController.mm (anonymous namespace there; not linkable here):
// one safe BinXCAF document, the caller's staging block, one bounded save.
NSData *CreateAssetAtlasDebugFixture(
    NSString *suffix,
    const std::function<void(const Handle(TDocStd_Document)&)>& populate) {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL *baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.%@", NSUUID.UUID.UUIDString, suffix]];
    NSString *xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData *result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        if (document.IsNull()) {
            throw Standard_Failure("Unable to create debug XCAF fixture");
        }
        XCAFDoc_DocumentTool::SetLengthUnit(document, 0.001);
        populate(document);
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure("Unable to save debug XCAF fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}
} // namespace
#endif

@implementation Core3DViewController (AssetAtlasOpenings)
- (Core3DAssetAtlasOpening *)openAssetAtlasEditor {
    ControllerAtlasInput input;
    return CaptureControllerAtlasInput(self, input)
        ? [[Core3DAssetAtlasOpening alloc] initWithInput:std::move(input)] : nil;
}

#if DEBUG
+ (NSData *)debugAssetAtlasFixtureAssetData:(double)metersPerUnit {
    if (!NSThread.isMainThread || (metersPerUnit != 0.001 && metersPerUnit != 1.0))
        return nil;
    return CreateAssetAtlasDebugFixture(
        metersPerUnit == 0.001 ? @"e2-asset-atlas-mm" : @"e2-asset-atlas-m",
        [metersPerUnit](const Handle(TDocStd_Document)& document) {
            StageAssetAtlasFixture(document, metersPerUnit);
        });
}

+ (NSDictionary<NSString *, id> *)
    debugAnalyticBooleanPlanarPCurves:(double)metersPerUnit {
    if (!NSThread.isMainThread || (metersPerUnit != 0.001 && metersPerUnit != 1.0))
        return nil;
    try {
        NSMutableArray *members = [NSMutableArray arrayWithCapacity:3];
        const int expectedIncidences[] = {28, 4, 26};
        const int expectedLines[] = {24, 0, 24};
        const int expectedCircles[] = {4, 4, 2};
        bool complete = true;
        for (int kind = 0; kind < 3; ++kind) {
            TopoDS_Shape shape = AnalyticFixtureShape(kind, metersPerUnit);
            const auto finalCensus = CensusStoredPlanar(shape);
            BRepMesh_IncrementalMesh mesh(shape,
                2.5e-4 * (.001 / metersPerUnit), Standard_False, 0.5, Standard_True);
            const auto postMeshCensus = CensusStoredPlanar(shape);
            NSData *rawBytes = nil;
            NSString *digest = ShapeSHA256(shape, &rawBytes);
            TopoDS_Shape reopened;
            BRep_Builder builder;
            const char *raw = static_cast<const char *>(rawBytes.bytes);
            std::istringstream stream(std::string(raw, rawBytes.length));
            stream.imbue(std::locale::classic());
            BRepTools::Read(reopened, stream, builder);
            const bool read = !reopened.IsNull();
            const auto reopenedCensus = read
                ? CensusStoredPlanar(reopened) : PlanarRepresentationCensus{};
            const bool memberComplete = finalCensus.incidences == expectedIncidences[kind]
                && finalCensus.stored == expectedIncidences[kind]
                && finalCensus.nonnull == expectedIncidences[kind]
                && finalCensus.lines == expectedLines[kind]
                && finalCensus.circles == expectedCircles[kind]
                && postMeshCensus.stored == finalCensus.stored
                && reopenedCensus.stored == finalCensus.stored
                && reopenedCensus.nonnull == finalCensus.nonnull;
            complete = complete && memberComplete;
            [members addObject:@{@"kind": @(kind),
                @"member": kind == 0 ? @"box-22"
                    : kind == 1 ? @"cylinder-21" : @"painted-box-23",
                @"savedMember": @(kind != 2),
                @"final": CensusDictionary(finalCensus),
                @"postMesh": CensusDictionary(postMeshCensus),
                @"reopened": CensusDictionary(reopenedCensus),
                @"rawBRepV3": rawBytes ?: NSData.data,
                @"rawBRepV3SHA256": digest ?: @"",
                @"complete": @(memberComplete)}];
        }
        return @{@"fixture": @"AnalyticBooleanStoredPlanarPCurves",
            @"metersPerUnit": @(metersPerUnit), @"members": members,
            @"controls": AnalyticScaleControls(),
            @"shiftedEllipse": PeriodicEllipseScaleControl(
                5 * std::acos(-1.0) / 2, 3 * std::acos(-1.0)),
            @"unshiftedEllipse": PeriodicEllipseScaleControl(
                std::acos(-1.0) / 2, std::acos(-1.0)),
            @"loftBore": LoftBoreScaleControl(metersPerUnit),
            @"allStoredNonNull": @(complete)};
    } catch (const Standard_Failure& failure) {
        return @{@"fixture": @"AnalyticBooleanStoredPlanarPCurves",
            @"error": [NSString stringWithUTF8String:failure.GetMessageString()] ?: @"OCCT failure"};
    } catch (const std::exception& failure) {
        return @{@"fixture": @"AnalyticBooleanStoredPlanarPCurves",
            @"error": [NSString stringWithUTF8String:failure.what()] ?: @"C++ failure"};
    } catch (...) {
        return @{@"fixture": @"AnalyticBooleanStoredPlanarPCurves",
            @"error": @"unknown failure"};
    }
}
#endif
@end
