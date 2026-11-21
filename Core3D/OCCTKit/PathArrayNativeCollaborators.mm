#import <Foundation/Foundation.h>

#include "PathArrayOwnerBridge.hxx"
#include "PatternOwnerBridge.hxx"
#include "NativeOpeningDependentReplay.hxx"
#include "BoundedCurveOwner.hxx"

#include <BRepBuilderAPI_Transform.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <algorithm>
#include <map>
#include <set>

#if DEBUG
#import <TargetConditionals.h>
#include "NativeOpeningSurfaceProbe.hxx"
#include "DetachedPlanarSweepProbe.hxx"
#include "../UI/Core3DViewer.h"
#include "../Viewport/Core3DSceneSnapshotFactory.hpp"
#if TARGET_OS_IOS
#import "GLView.h"
#endif
#include <BRepPrimAPI_MakeBox.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Name.hxx>
#include <TNaming_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Face.hxx>

namespace core3d::bounded_curve {
// Definition of the DEBUG friend forward-declared in BoundedCurveAttribute.hxx.
// It lets this fixture attach a canonical payload exactly the way the production
// owner path in OcctDocument.mm does, without widening the attribute's API.
// Identical to the Core3DNativeOpenings.mm definition.
struct PersistenceProbe {
    static bool Attach(const TDF_Label& record,
                       const std::shared_ptr<Payload>& payload) noexcept {
        try {
            if (record.IsNull() || !payload || record.HasAttribute()) return false;
            Handle(Attribute) attribute = new Attribute();
            attribute->value_ = payload;
            record.AddAttribute(attribute);
            return true;
        } catch (...) { return false; }
    }
};
} // namespace core3d::bounded_curve
#endif

namespace core3d::path_array_owner {
namespace {
constexpr std::uint32_t kFenceWidth = 64;
constexpr std::uint32_t kFenceHeight = 64;

std::string Text(const UUID& value) noexcept {
    try { return retained_solid::UUIDText(value); }
    catch (...) { return {}; }
}

std::size_t RecipeBytes(const pattern_recipe_clone::Source& source) noexcept {
    using Family = pattern_recipe_clone::Family;
    if (source.family == Family::Sweep)
        return source.sweep.values.size() * sizeof(double);
    if (source.family == Family::Loft)
        return source.loft.values.size() * sizeof(double);
    if (source.family == Family::AnalyticBoolean && source.analyticBoolean.value)
        return source.analyticBoolean.value->bytes.size();
    return 0;
}

std::string RecipeFeature(const pattern_recipe_clone::Source& source) noexcept {
    try {
        if (source.family == pattern_recipe_clone::Family::Sweep)
            return source.sweep.identifier;
        if (source.family == pattern_recipe_clone::Family::Loft)
            return source.loft.identifier;
        if (source.family == pattern_recipe_clone::Family::AnalyticBoolean
            && source.analyticBoolean.value
            && !source.analyticBoolean.value->definition.nodes.empty()) {
            const auto* feature = std::get_if<composite_recipe::FeatureNode>(
                &source.analyticBoolean.value->definition.nodes.back().value);
            return feature ? Text(feature->feature) : std::string{};
        }
    } catch (...) {}
    return {};
}

bool Measure(const TopoDS_Shape& shape, std::size_t recipeBytes,
             pattern_owner::AllLabelMeasuredCost& output,
             std::string& shapeBytes) noexcept {
    output = {}; shapeBytes.clear();
    try {
        if (shape.IsNull()
            || !retained_part_boolean::ExactShapeBytes(shape, shapeBytes)) return false;
        TopTools_IndexedMapOfShape topology;
        TopExp::MapShapes(shape, topology);
        output.recipeBytes = recipeBytes;
        output.shapeBytes = shapeBytes.size();
        output.topologyNodes = topology.Extent();
        output.retainedMemoryBytes = sizeof(output) + recipeBytes + shapeBytes.size();
        return true;
    } catch (...) { output = {}; shapeBytes.clear(); return false; }
}

bool Add(std::size_t value, std::size_t& total) noexcept {
    if (value > std::numeric_limits<std::size_t>::max() - total) return false;
    total += value; return true;
}

bool AddCost(const pattern_owner::AllLabelMeasuredCost& value,
             pattern_owner::AllLabelMeasuredCost& total) noexcept {
    if (value.topologyNodes > std::numeric_limits<Standard_Size>::max()
            - total.topologyNodes) return false;
    total.topologyNodes += value.topologyNodes;
    return Add(value.recordBytes, total.recordBytes)
        && Add(value.recipeBytes, total.recipeBytes)
        && Add(value.shapeBytes, total.shapeBytes)
        && Add(value.retainedMemoryBytes, total.retainedMemoryBytes);
}

class LivePathResolver final : public CurrentPathResolver {
public:
    LivePathResolver(OcctDocument& owner,
        std::shared_ptr<native_opening::Context> context) noexcept
        : owner_(owner), context_(std::move(context)) {}

    Refusal resolveCurrent(const path_array::CurveReference& reference,
                           PathAuthority& output) noexcept override {
        output = {};
        try {
            if (!context_ || context_->openingFence().document() != owner_.Document()
                || context_->openingFence().data() != owner_.Document()->GetData())
                return Refusal::StalePath;
            OcctBoundedCurveCapture exact;
            if (!owner_.ReadBoundedCurveExact(reference.owner, exact))
                return Refusal::MissingPath;
            if (!path_array::Matches(reference, exact.persisted))
                return Refusal::StalePath;
            bounded_curve::owner::OcafOwner c1(owner_, context_);
            const auto scene = bounded_curve::owner::SceneFence{
                context_->openingFence().documentGeneration(),
                context_->openingFence().modelRevision(),
                context_->openingFence().metersPerUnit()};
            auto receipt = c1.pickCurrentPath3D(
                exact.ownerReceipt.visibility.object.object.entityIdentifier, scene);
            if (!receipt || receipt->owner != reference.owner
                || receipt->feature != reference.feature
                || receipt->definitionRevision != reference.definitionRevision
                || receipt->canonicalDefinitionDigest
                    != reference.canonicalDefinitionDigest) return Refusal::StalePath;
            output.receipt = std::move(receipt);
            output.locator = reference;
            output.persisted = exact.persisted;
            output.ownerLabel = exact.ownerReceipt.visibility.object.object.label;
            return output.currentFor(reference) ? Refusal::None : Refusal::StalePath;
        } catch (...) { output = {}; return Refusal::StalePath; }
    }
private:
    OcctDocument& owner_;
    std::shared_ptr<native_opening::Context> context_;
};

class OcafD3Authority final : public D2ArrayAuthority {
public:
    Refusal captureCurrent(OcctDocument& owner,
        const std::vector<path_array::Record>& records,
        const std::string& selectedEntity, path_array::Record& selectedRecord,
        std::shared_ptr<const pattern_owner::AllLabelSnapshot>& output,
        SourceMetrics& metrics) noexcept override {
        selectedRecord = {}; output.reset(); metrics = {};
        try {
            UUID selected{};
            if (!receipt::ParseUUID(selectedEntity, selected))
                return Refusal::AmbiguousSelection;
            const path_array::Record* match = nullptr;
            for (const auto& record : records) {
                bool contains = false;
                for (const auto& member : record.definition.members)
                    contains = contains || member.identity == selected;
                if (contains) {
                    if (match) return Refusal::AmbiguousSelection;
                    match = &record;
                }
            }
            if (!match) return Refusal::AmbiguousSelection;
            const auto document = owner.Document();
            const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
            if (shapes.IsNull()) return Refusal::ClosedDocument;
            TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
            // Inventory root identities without the generic exact-label
            // reader: unrelated roots (for example the current and
            // replacement C1 curve records) legitimately refuse that
            // mesh-copy metadata reader and must not break this array's
            // admission. Unreadable or duplicate identities still refuse.
            std::map<UUID, TDF_Label> labels;
            for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
                pattern_owner::LabelReceipt identity;
                if (!pattern_owner::ReadReceipt(owner, roots.Value(index), identity)
                    || !labels.emplace(identity.entity, identity.label).second)
                    return Refusal::CorruptTable;
            }
            const auto sourceLabel = labels.find(match->definition.source.entity);
            if (sourceLabel == labels.end()) return Refusal::MissingSource;
            OcctExactLabelReceipt source;
            if (!owner.CaptureExactFreeLabel(sourceLabel->second, source))
                return Refusal::CorruptTable;
            if (source.visibility.object.object.entityIdentifier
                    != Text(match->definition.source.entity)
                || source.visibility.object.object.definitionIdentifier
                    != Text(match->definition.source.definition))
                return Refusal::MissingSource;

            auto snapshot = std::make_shared<pattern_owner::AllLabelSnapshot>();
            snapshot->documentData = document->GetData();
            snapshot->documentIdentifier = owner.DocumentIdentifier();
            snapshot->family = pattern_owner::AllLabelFamily::PathArrayD3;
            snapshot->recordLabel = match->label;
            snapshot->featureIdentifier = Text(match->definition.feature);
            snapshot->canonicalRecordBytes = match->bytes;
            snapshot->source = source;
            snapshot->documentTime = document->GetData()->Time();
            if (!pattern_recipe_clone::Capture(document,
                    snapshot->source.visibility.object.object.label,
                    snapshot->sourceRecipe)
                || !Measure(snapshot->source.visibility.object.object.shape,
                    RecipeBytes(snapshot->sourceRecipe), snapshot->measured,
                    snapshot->sourceShapeBytes)) return Refusal::UnsupportedSource;
            snapshot->measured.recordBytes = match->bytes.size();
            if (!Add(snapshot->measured.recordBytes,
                     snapshot->measured.retainedMemoryBytes))
                return Refusal::DocumentBudget;

            snapshot->members.reserve(match->definition.members.size());
            for (std::size_t ordinal = 0;
                 ordinal < match->definition.members.size(); ++ordinal) {
                const auto& retained = match->definition.members[ordinal];
                const auto found = labels.find(retained.identity);
                if (found == labels.end()) return Refusal::MissingMember;
                pattern_owner::AllLabelSnapshot::Member member;
                member.key = pattern_owner::D3Ordinal{std::uint32_t(ordinal)};
                if (!owner.CaptureExactFreeLabel(found->second, member.receipt))
                    return Refusal::CorruptTable;
                if (member.receipt.visibility.object.object.entityIdentifier
                        != Text(retained.identity))
                    return Refusal::MissingMember;
                member.localIdentifier = retained.localID;
                member.suppressed = retained.state == pattern::MemberState::Suppressed;
                if (!pattern_recipe_clone::Capture(document,
                        member.receipt.visibility.object.object.label, member.recipe))
                    return Refusal::UnsupportedSource;
                member.featureIdentifier = RecipeFeature(member.recipe);
                if (!Measure(member.receipt.visibility.object.object.shape,
                        RecipeBytes(member.recipe), member.measured,
                        member.shapeBytes)) return Refusal::StaleMember;
                if (ordinal == 0) {
                    if (!member.receipt.IsEqual(snapshot->source)
                        || !pattern_recipe_clone::IsEqual(
                            member.recipe, snapshot->sourceRecipe))
                        return Refusal::StaleSource;
                } else if (member.recipe.family != snapshot->sourceRecipe.family
                    || !AddCost(member.measured, snapshot->measured))
                    return Refusal::UnsupportedSource;
                snapshot->members.push_back(std::move(member));
            }
            metrics.sourceDocumentBytes = snapshot->measured.recipeBytes
                + snapshot->measured.shapeBytes;
            metrics.sourceMemoryBytes = snapshot->measured.retainedMemoryBytes;
            metrics.sourceTopologyNodes = snapshot->measured.topologyNodes;
            for (const auto& record : records)
                metrics.patternDocumentBytes += record.bytes.size();
            std::vector<composite_recipe::Record> composites;
            if (!composite_recipe::ReadAll(document, composites,
                    metrics.patternDocumentBytes)) return Refusal::DocumentBudget;
            for (const auto& record : composites)
                if (record.value) metrics.compositeDocumentBytes += record.value->bytes.size();
            selectedRecord = *match;
            output = std::move(snapshot);
            return Refusal::None;
        } catch (...) { selectedRecord = {}; output.reset(); metrics = {};
            return Refusal::CorruptTable; }
    }

    bool isCurrent(OcctDocument& owner,
        const std::shared_ptr<const pattern_owner::AllLabelSnapshot>& expected,
        const path_array::Definition& definition) noexcept override {
        try {
            if (!expected || expected->members.empty()
                || expected->family != pattern_owner::AllLabelFamily::PathArrayD3
                || expected->featureIdentifier != Text(definition.feature)) return false;
            std::vector<path_array::Record> records;
            if (!path_array::ReadAll(owner.Document(), records)) return false;
            path_array::Record record; SourceMetrics metrics;
            std::shared_ptr<const pattern_owner::AllLabelSnapshot> current;
            const auto entity = expected->members.front()
                .receipt.visibility.object.object.entityIdentifier;
            if (captureCurrent(owner, records, entity, record, current, metrics)
                    != Refusal::None
                || record.bytes != expected->canonicalRecordBytes || !current)
                return false;
            // The caller may already have staged C1 in this same command.
            // TDF_Data::Time is then newer even though every D3 byte/label is
            // still exact, so normalize only that transaction-wide counter.
            pattern_owner::AllLabelSnapshot normalized = *current;
            normalized.documentTime = expected->documentTime;
            return pattern_owner::IsExactlyEqual(*expected, normalized);
        } catch (...) { return false; }
    }
};

bool MatrixTransform(const path_array::Matrix& matrix, gp_Trsf& output) noexcept {
    try {
        output.SetValues(matrix[0], matrix[1], matrix[2], matrix[3],
                         matrix[4], matrix[5], matrix[6], matrix[7],
                         matrix[8], matrix[9], matrix[10], matrix[11]);
        return pattern_recipe_clone::IsProperRigidPlacement(output);
    } catch (...) { return false; }
}

bool StageRecipe(const Handle(TDocStd_Document)& document, const TDF_Label& label,
                 const pattern_recipe_clone::Prepared& prepared,
                 bool created) noexcept {
    try {
        if (prepared.family == pattern_recipe_clone::Family::None) return true;
        if (prepared.family == pattern_recipe_clone::Family::Sweep)
            return sweep_persistence::Stage(document, label, prepared.sweep,
                                             prepared.featureIdentifier);
        if (prepared.family == pattern_recipe_clone::Family::Loft)
            return loft_persistence::Stage(document, label, prepared.loft,
                                            prepared.featureIdentifier);
        if (prepared.family != pattern_recipe_clone::Family::AnalyticBoolean
            || !created) return false;
        return composite_recipe::Attribute::StageIndependentClone(
            document, label, prepared.binding, prepared.analyticBoolean);
    } catch (...) { return false; }
}

bool PreparedRecipeMatches(const pattern_recipe_clone::Prepared& expected,
                           const pattern_recipe_clone::Source& actual) noexcept {
    if (expected.family != actual.family) return false;
    if (expected.family == pattern_recipe_clone::Family::None) return true;
    if (RecipeFeature(actual) != expected.featureIdentifier) return false;
    if (expected.family == pattern_recipe_clone::Family::Sweep) {
        std::vector<double> a, b;
        return sweep_persistence::Encode(expected.sweep, a)
            && sweep_persistence::Encode(actual.sweep.definition, b) && a == b;
    }
    if (expected.family == pattern_recipe_clone::Family::Loft) {
        std::vector<double> a, b;
        return loft_persistence::Encode(expected.loft, a)
            && loft_persistence::Encode(actual.loft.definition, b) && a == b;
    }
    return expected.analyticBoolean && actual.analyticBoolean.value
        && expected.analyticBoolean->bytes == actual.analyticBoolean.value->bytes;
}

bool SameMember(const path_array::Member& left,
                const path_array::Member& right) noexcept {
    return left.identity == right.identity && left.localID == right.localID
        && left.coordinate.row == right.coordinate.row
        && left.coordinate.column == right.coordinate.column
        && left.state == right.state;
}

bool SamePlacement(const path_array::Placement& left,
                   const path_array::Placement& right) noexcept {
    return left.identity == right.identity && left.localID == right.localID
        && left.ordinal == right.ordinal
        && left.requestedArcLength == right.requestedArcLength
        && left.measuredArcLength == right.measuredArcLength
        && left.parameter == right.parameter
        && left.occurrenceFrame == right.occurrenceFrame;
}

bool SamePreparedRecipe(const pattern_recipe_clone::Prepared& left,
                        const pattern_recipe_clone::Prepared& right) noexcept {
    try {
        if (left.family != right.family
            || left.featureIdentifier != right.featureIdentifier
            || left.identities.global != right.identities.global
            || !pattern_recipe_clone::ShapeBytesEqual(left.binding, right.binding))
            return false;
        if (left.family == pattern_recipe_clone::Family::None) return true;
        if (left.family == pattern_recipe_clone::Family::Sweep) {
            std::vector<double> a, b;
            return sweep_persistence::Encode(left.sweep, a)
                && sweep_persistence::Encode(right.sweep, b) && a == b;
        }
        if (left.family == pattern_recipe_clone::Family::Loft) {
            std::vector<double> a, b;
            return loft_persistence::Encode(left.loft, a)
                && loft_persistence::Encode(right.loft, b) && a == b;
        }
        return left.analyticBoolean && right.analyticBoolean
            && left.analyticBoolean->bytes == right.analyticBoolean->bytes;
    } catch (...) { return false; }
}

bool SameClone(const OcctPreparedLabelClone& left,
               const OcctPreparedLabelClone& right) noexcept {
    return left.source.IsEqual(right.source)
        && left.representation == right.representation
        && pattern_recipe_clone::ShapeBytesEqual(left.detachedShape,
                                                  right.detachedShape);
}
} // namespace

struct NativeMutation final {
    struct Recipe final {
        std::uint32_t ordinal = 0;
        bool created = false;
        std::string entityIdentifier;
        pattern_recipe_clone::Prepared prepared;
    };
    std::shared_ptr<const pattern_owner::AllLabelSnapshot> before;
    OcctAllLabelPlan labels;
    std::vector<Recipe> recipes;
    std::vector<OcctIssuedLabelIdentity> issued;
    std::vector<std::string> recipeFeatureIdentifiers;
    std::vector<std::uint8_t> canonicalCandidateBytes;
};

#if DEBUG
namespace {
struct DebugMutationObservation final {
    std::vector<std::string> plannedEntities;
    std::vector<std::string> plannedRecipeFeatures;
    std::vector<std::string> createdEntities;
    std::vector<std::string> revalidatedEntities;
    std::vector<std::string> revalidatedRecipeFeatures;
    std::size_t reservationCount = 0;
    std::size_t historicalRemovalCount = 0;
    std::size_t candidateRemovalCount = 0;
    std::size_t liveRemovalCount = 0;
    std::size_t validationCount = 0;
    bool finalPlanEqual = false;
};

std::map<std::string, DebugMutationObservation> gDebugMutationObservations;

void RecordPreparedMutation(const PreparedEdit& prepared) {
    if (!prepared.admitted() || !prepared.native) return;
    DebugMutationObservation value;
    value.reservationCount = prepared.native->issued.size();
    value.historicalRemovalCount =
        prepared.opening.record.definition.removals.size();
    value.candidateRemovalCount = prepared.candidate.removals.size();
    value.liveRemovalCount = prepared.native->labels.removals.size();
    for (const auto& recipe : prepared.native->recipes) {
        value.plannedEntities.push_back(recipe.entityIdentifier);
        value.plannedRecipeFeatures.push_back(
            recipe.prepared.featureIdentifier);
    }
    for (const auto& created : prepared.native->labels.creates)
        value.createdEntities.push_back(
            created.identity.EntityIdentifier());
    gDebugMutationObservations[Text(prepared.candidate.source.entity)] =
        std::move(value);
}

void RecordRevalidatedMutation(const PreparedEdit& prepared,
                               const PreparedEdit& regenerated,
                               bool equal) {
    const auto found = gDebugMutationObservations.find(
        Text(prepared.candidate.source.entity));
    if (found == gDebugMutationObservations.end()) return;
    found->second.finalPlanEqual = equal;
    ++found->second.validationCount;
    if (!equal || !regenerated.native) return;
    for (const auto& recipe : regenerated.native->recipes) {
        found->second.revalidatedEntities.push_back(recipe.entityIdentifier);
        found->second.revalidatedRecipeFeatures.push_back(
            recipe.prepared.featureIdentifier);
    }
}

NSArray<NSString *> *Strings(const std::vector<std::string>& values) {
    NSMutableArray<NSString *> *result =
        [NSMutableArray arrayWithCapacity:values.size()];
    for (const auto& value : values)
        [result addObject:[[NSString alloc]
            initWithBytes:value.data() length:value.size()
            encoding:NSUTF8StringEncoding] ?: @""];
    return result;
}
} // namespace

NSDictionary<NSString *, id> *DebugNativeMutationEvidence(
    const std::string& sourceEntity) noexcept {
    try {
        const auto found = gDebugMutationObservations.find(sourceEntity);
        if (found == gDebugMutationObservations.end()) return nil;
        const auto& value = found->second;
        return @{
            @"schema": @"shapeyard.d3-native-mutation-evidence.v1",
            @"plannedEntityIdentifiers": Strings(value.plannedEntities),
            @"plannedRecipeFeatureIdentifiers":
                Strings(value.plannedRecipeFeatures),
            @"createdEntityIdentifiers": Strings(value.createdEntities),
            @"revalidatedEntityIdentifiers":
                Strings(value.revalidatedEntities),
            @"revalidatedRecipeFeatureIdentifiers":
                Strings(value.revalidatedRecipeFeatures),
            @"reservationCount": @(value.reservationCount),
            @"historicalRemovalCount": @(value.historicalRemovalCount),
            @"candidateRemovalCount": @(value.candidateRemovalCount),
            @"liveRemovalCount": @(value.liveRemovalCount),
            @"validationCount": @(value.validationCount),
            @"finalPlanEqual": @(value.finalPlanEqual)
        };
    } catch (...) { return nil; }
}
#endif

namespace {
class OcafD3Stager final : public Stager {
public:
    OcafD3Stager(OcctDocument& owner,
        std::shared_ptr<native_opening::Context> context) noexcept
        : owner_(owner), context_(std::move(context)) {}
    OcafD3Stager(OcctDocument& owner, native_opening::CommandLease& lease) noexcept
        : owner_(owner), borrowed_(&lease) {}

    bool begin(const Snapshot& opening,
               const PreparedEdit& prepared) noexcept override {
        if (borrowed_ || !context_ || !prepared.native
            || prepared.native->before != opening.labels) return false;
        try {
            lease_ = context_->beginCommandLease(context_->openingFence(),
                                                 kFenceWidth, kFenceHeight);
            return lease_ && lease_->ownsOpenCommand();
        } catch (...) { return false; }
    }

    bool stageAll(const Snapshot& opening,
                  const PreparedEdit& prepared) noexcept override {
        receipts_.clear();
        try {
            auto* lease = borrowed_ ? borrowed_ : lease_.get();
            if (!lease || !lease->ownsOpenCommand() || !prepared.native
                || (!borrowed_ && prepared.native->before != opening.labels)
                || (!borrowed_ && !authority_.isCurrent(owner_, opening.labels,
                                          opening.record.definition)))
                return false;
            // A survivor carrying a retained sweep/loft recipe cannot go
            // through the exact-label replacement route: ReplaceShape
            // re-captures the owner after SetShape, and a bound recipe record
            // read refuses the stale intermediate binding; forgetting the
            // record first breaks the replacement's receipt-current fence.
            // Those survivors are staged below in the proven
            // pattern_recipe_clone::StageReplacement order (recipe values,
            // then the raw shape set, then the recipe rebind) inside this
            // same command, with presentation preservation measured from
            // exact receipts. Recipe-less survivors, creates and removals keep
            // the shared exact-label route.
            std::set<std::string> recipeSurvivors;
            for (const auto& recipe : prepared.native->recipes)
                if (!recipe.created
                    && (recipe.prepared.family
                            == pattern_recipe_clone::Family::Sweep
                        || recipe.prepared.family
                            == pattern_recipe_clone::Family::Loft))
                    recipeSurvivors.insert(recipe.entityIdentifier);
            OcctAllLabelPlan directPlan = prepared.native->labels;
            std::vector<OcctAllLabelPlan::Replace> recipeReplacements;
            if (!recipeSurvivors.empty()) {
                directPlan.replacements.clear();
                for (const auto& replacement : prepared.native->labels
                         .replacements) {
                    if (recipeSurvivors.count(replacement.expected.visibility
                            .object.object.entityIdentifier))
                        recipeReplacements.push_back(replacement);
                    else
                        directPlan.replacements.push_back(replacement);
                }
            }
            if (!owner_.StageAllLabels(*lease, directPlan, receipts_))
                return false;
            for (const auto& replacement : recipeReplacements) {
                const auto& entity = replacement.expected.visibility.object
                    .object.entityIdentifier;
                const auto recipe = std::find_if(prepared.native->recipes
                        .begin(), prepared.native->recipes.end(),
                    [&](const NativeMutation::Recipe& value) {
                        return value.entityIdentifier == entity;
                    });
                OcctExactLabelReceipt stagedReceipt;
                if (recipe == prepared.native->recipes.end()
                    || !stageRecipeSurvivor(replacement, *recipe,
                            stagedReceipt))
                    return false;
                receipts_.push_back(std::move(stagedReceipt));
            }
            for (const auto& recipe : prepared.native->recipes) {
                if (recipeSurvivors.count(recipe.entityIdentifier)) continue;
                const auto found = std::find_if(receipts_.begin(), receipts_.end(),
                    [&](const OcctExactLabelReceipt& value) {
                        return value.visibility.object.object.entityIdentifier
                            == recipe.entityIdentifier;
                    });
                if (found == receipts_.end()
                    || !StageRecipe(owner_.Document(),
                        found->visibility.object.object.label,
                        recipe.prepared, recipe.created)) return false;
            }
            for (auto& receipt : receipts_) {
                OcctExactLabelReceipt refreshed;
                if (!owner_.CaptureExactFreeLabel(
                        receipt.visibility.object.object.label, refreshed)) return false;
                receipt = std::move(refreshed);
            }
            mutation_ = prepared.native;
            return true;
        } catch (...) { receipts_.clear(); return false; }
    }

private:
    // Stage one recipe-carrying survivor replacement in the proven
    // pattern_recipe_clone::StageReplacement order: update the recipe values
    // while the label still carries the current shape, set the replacement
    // shape, rebind the recipe record, then prove the exact receipt with only
    // the shape changed and every presentation field preserved.
    bool stageRecipeSurvivor(const OcctAllLabelPlan::Replace& replacement,
                             const NativeMutation::Recipe& recipe,
                             OcctExactLabelReceipt& receipt) noexcept {
        receipt = {};
        try {
            const auto& expected = replacement.expected;
            const TDF_Label label = expected.visibility.object.object.label;
            const auto document = owner_.Document();
            if (document.IsNull() || !document->HasOpenCommand()
                || label.IsNull() || replacement.clone.detachedShape.IsNull())
                return false;
            OcctExactLabelReceipt current;
            if (!owner_.ReadExactFreeLabel(expected, current)) return false;
            pattern_recipe_clone::Source currentRecipe;
            if (!pattern_recipe_clone::Capture(document, label, currentRecipe))
                return false;
            const auto shapes = XCAFDoc_DocumentTool::ShapeTool(
                document->Main());
            if (shapes.IsNull()) return false;
            if (recipe.prepared.family == pattern_recipe_clone::Family::Sweep) {
                if (currentRecipe.family
                        != pattern_recipe_clone::Family::Sweep
                    || currentRecipe.sweep.label.IsNull()
                    || recipe.prepared.featureIdentifier
                        != currentRecipe.sweep.identifier
                    || !sweep_persistence::Stage(document, label,
                            recipe.prepared.sweep, currentRecipe.sweep.identifier))
                    return false;
                shapes->SetShape(label, replacement.clone.detachedShape);
                TNaming_Builder(currentRecipe.sweep.label).Select(
                    replacement.clone.detachedShape,
                    replacement.clone.detachedShape);
                sweep_persistence::Record stagedRecipe;
                if (!sweep_persistence::Read(document, label, stagedRecipe)
                    || stagedRecipe.label.IsNull()
                    || !stagedRecipe.label.IsEqual(currentRecipe.sweep.label)
                    || stagedRecipe.identifier != currentRecipe.sweep.identifier
                    || !stagedRecipe.IsCurrent(document, label)) return false;
            } else if (recipe.prepared.family
                    == pattern_recipe_clone::Family::Loft) {
                if (currentRecipe.family
                        != pattern_recipe_clone::Family::Loft
                    || currentRecipe.loft.label.IsNull()
                    || recipe.prepared.featureIdentifier
                        != currentRecipe.loft.identifier
                    || !loft_persistence::Stage(document, label,
                            recipe.prepared.loft, currentRecipe.loft.identifier))
                    return false;
                shapes->SetShape(label, replacement.clone.detachedShape);
                TNaming_Builder(currentRecipe.loft.label).Select(
                    replacement.clone.detachedShape,
                    replacement.clone.detachedShape);
                loft_persistence::Record stagedRecipe;
                if (!loft_persistence::Read(document, label, stagedRecipe)
                    || stagedRecipe.label.IsNull()
                    || !stagedRecipe.label.IsEqual(currentRecipe.loft.label)
                    || stagedRecipe.identifier != currentRecipe.loft.identifier
                    || !stagedRecipe.IsCurrent(document, label)) return false;
            } else return false;
            if (!owner_.CaptureExactFreeLabel(label, receipt)
                || !receipt.visibility.object.object.shape.IsEqual(
                    replacement.clone.detachedShape)
                || !SurvivorPresentationPreserved(expected, receipt))
                { receipt = {}; return false; }
            return true;
        } catch (...) { receipt = {}; return false; }
    }

    static bool SurvivorPresentationPreserved(
        const OcctExactLabelReceipt& expected,
        const OcctExactLabelReceipt& receipt) noexcept {
        try {
            const auto& a = expected.visibility;
            const auto& b = receipt.visibility;
            return a.invisibleAttributePresent == b.invisibleAttributePresent
                && a.layerLinkPresent == b.layerLinkPresent
                && a.layers == b.layers
                && a.layerInvisibleAttributePresent
                    == b.layerInvisibleAttributePresent
                && a.object.namePresent == b.object.namePresent
                && (!a.object.namePresent || a.object.name.IsEqual(b.object.name))
                && a.object.object.entityIdentifier
                    == b.object.object.entityIdentifier
                && a.object.object.definitionIdentifier
                    == b.object.object.definitionIdentifier
                && a.object.object.present == b.object.object.present
                && a.object.object.scalars == b.object.object.scalars
                && expected.appearance.IsEqual(receipt.appearance);
        } catch (...) { return false; }
    }

public:
    bool readBackAll(const path_array::Definition& candidate) noexcept override {
        try {
            if (!mutation_
                || !owner_.ReadBackAllLabels(mutation_->labels, receipts_)) return false;
            path_array::Record record;
            if (!path_array::ReadFeature(owner_.Document(), candidate.feature, record)
                || record.bytes != mutation_->canonicalCandidateBytes) return false;
            for (const auto& recipe : mutation_->recipes) {
                const auto found = std::find_if(receipts_.begin(), receipts_.end(),
                    [&](const OcctExactLabelReceipt& value) {
                        return value.visibility.object.object.entityIdentifier
                            == recipe.entityIdentifier;
                    });
                pattern_recipe_clone::Source actual;
                if (found == receipts_.end()
                    || !pattern_recipe_clone::Capture(owner_.Document(),
                        found->visibility.object.object.label, actual)
                    || !PreparedRecipeMatches(recipe.prepared, actual)) return false;
            }
            return true;
        } catch (...) { return false; }
    }

    bool commit() noexcept override {
        if (borrowed_ || !lease_) return false;
        const bool result = lease_->commit(); lease_.reset(); return result;
    }
    bool abort() noexcept override {
        if (borrowed_ || !lease_) return false;
        const bool result = lease_->abort(); lease_.reset(); receipts_.clear();
        return result;
    }
private:
    OcctDocument& owner_;
    std::shared_ptr<native_opening::Context> context_;
    std::shared_ptr<native_opening::CommandLease> lease_;
    native_opening::CommandLease* borrowed_ = nullptr;
    OcafD3Authority authority_;
    std::shared_ptr<const NativeMutation> mutation_;
    std::vector<OcctExactLabelReceipt> receipts_;
};

const pattern_owner::AllLabelSnapshot::Member* FindBefore(
    const Snapshot& opening, const UUID& entity) noexcept {
    if (!opening.labels) return nullptr;
    const std::string text = Text(entity);
    for (const auto& member : opening.labels->members)
        if (member.receipt.visibility.object.object.entityIdentifier == text)
            return &member;
    return nullptr;
}

PreparedEdit PrepareMutation(OcctDocument& owner, PreparedEdit result,
    const std::vector<OcctIssuedLabelIdentity>& issued,
    const std::vector<std::string>* retainedRecipeFeatures = nullptr) noexcept {
    PreparedEdit refused; refused.opening = result.opening;
    try {
        if (!result.admitted() || !result.opening.labels) return refused;
        auto mutation = std::make_shared<NativeMutation>();
        mutation->before = result.opening.labels;
        mutation->issued = issued;
        if (!path_array::Encode(result.candidate,
                                mutation->canonicalCandidateBytes)) return refused;
        std::set<std::string> ledgerSet;
        for (const auto& removed : result.opening.record.definition.removals) {
            if (FindBefore(result.opening, removed.identity)
                || !ledgerSet.insert(Text(removed.identity)).second) return refused;
        }
        mutation->labels.retainedRemovalLedger.assign(
            ledgerSet.begin(), ledgerSet.end());

        // ReconcileMembers appends new removals after the canonical historical
        // prefix. Historical tombstones have no live label by design: compare
        // them byte-for-byte with the opening and retain them only in SYPA/1's
        // permanent ledger. Any altered, reordered, duplicated, resurrected or
        // ledger-inconsistent entry refuses before a command exists.
        const auto& openingDefinition = result.opening.record.definition;
        if (result.candidate.removals.size() < openingDefinition.removals.size())
            return refused;
        std::set<std::uint64_t> expectedRetired(
            openingDefinition.issuance.retiredLocalIDs.begin(),
            openingDefinition.issuance.retiredLocalIDs.end());
        if (expectedRetired.size()
                != openingDefinition.issuance.retiredLocalIDs.size()) return refused;
        std::set<std::uint64_t> historicalLocalIDs;
        for (std::size_t index = 0;
             index < openingDefinition.removals.size(); ++index) {
            const auto& historical = openingDefinition.removals[index];
            if (!SameMember(historical, result.candidate.removals[index])
                || !historicalLocalIDs.insert(historical.localID).second
                || !expectedRetired.count(historical.localID)) return refused;
        }
        if (historicalLocalIDs != expectedRetired) return refused;

        path_array::Definition allActive = result.candidate;
        for (std::size_t ordinal = 1; ordinal < allActive.members.size(); ++ordinal)
            if (allActive.members[ordinal].state == pattern::MemberState::Suppressed
                && !path_array::SetSuppressed(allActive,
                    std::uint32_t(ordinal), false)) return refused;
        std::vector<path_array::Placement> placements;
        path_array::BuildReceipt build;
        if (path_array::BuildPlacements(allActive, result.candidatePath.persisted,
                placements, build) != path_array::BuildRefusal::None
            || placements.size() != allActive.members.size()) return refused;

        gp_Trsf sourceFrame;
        if (!MatrixTransform(result.candidate.sourceFrame, sourceFrame)) return refused;
        std::size_t reservation = 0;
        for (std::size_t ordinal = 1;
             ordinal < result.candidate.members.size(); ++ordinal) {
            const auto& retained = result.candidate.members[ordinal];
            const auto* before = FindBefore(result.opening, retained.identity);
            gp_Trsf occurrence;
            if (!MatrixTransform(placements[ordinal].occurrenceFrame, occurrence))
                return refused;
            const gp_Trsf transform = occurrence * sourceFrame.Inverted();
            BRepBuilderAPI_Transform copied(
                result.opening.labels->source.visibility.object.object.shape,
                transform, Standard_True);
            if (!copied.IsDone() || copied.Shape().IsNull()) return refused;
            OcctPreparedLabelClone clone;
            clone.source = result.opening.labels->source;
            clone.detachedShape = copied.Shape();
            clone.representation = result.opening.labels->source
                .visibility.object.object.resolvedRepresentation;
            NativeMutation::Recipe recipe;
            recipe.ordinal = std::uint32_t(ordinal);
            recipe.created = before == nullptr;
            recipe.entityIdentifier = Text(retained.identity);
            const std::optional<gp_Trsf> baked = transform;
            if (before) {
                mutation->labels.replacements.push_back({before->receipt, clone});
                const auto& sourceRecipe = result.opening.labels->sourceRecipe;
                bool recipePrepared = false;
                if (sourceRecipe.family == pattern_recipe_clone::Family::None
                    && before->recipe.family == pattern_recipe_clone::Family::None) {
                    // Recipe-less survivors use the same detached preparation as
                    // recipe-less creates. Keep the existing survivor ownership,
                    // solid, and binding checks; typed recipe replacement stays strict.
                    recipePrepared = !sourceRecipe.owner.IsNull()
                        && !before->recipe.owner.IsNull()
                        && !sourceRecipe.owner.IsEqual(before->recipe.owner)
                        && !clone.detachedShape.IsNull()
                        && clone.detachedShape.ShapeType() == TopAbs_SOLID
                        && !clone.detachedShape.IsPartner(before->recipe.ownerShape)
                        && pattern_recipe_clone::Prepare(sourceRecipe,
                            clone.detachedShape, std::string{}, baked, recipe.prepared);
                } else {
                    recipePrepared = pattern_recipe_clone::PrepareReplacement(
                        sourceRecipe, before->recipe, clone.detachedShape,
                        baked, recipe.prepared);
                }
#if DEBUG
                NSLog(@"R179_D3_SURVIVOR ordinal=%zu sourceFamily=%u memberFamily=%u prepared=%d",
                      ordinal, unsigned(sourceRecipe.family),
                      unsigned(before->recipe.family), int(recipePrepared));
#endif
                if (!recipePrepared) return refused;
            } else {
                if (reservation >= issued.size()
                    || issued[reservation].EntityIdentifier()
                        != recipe.entityIdentifier) return refused;
                mutation->labels.creates.push_back({issued[reservation], clone});
                std::string feature;
                if (result.opening.labels->sourceRecipe.family
                        != pattern_recipe_clone::Family::None) {
                    feature = retainedRecipeFeatures
                        ? (reservation < retainedRecipeFeatures->size()
                            ? (*retainedRecipeFeatures)[reservation] : std::string{})
                        : OcctDocument::NewProfileIdentifier();
                    if (feature.empty()) return refused;
                } else if (retainedRecipeFeatures
                    && (reservation >= retainedRecipeFeatures->size()
                        || !(*retainedRecipeFeatures)[reservation].empty())) {
                    return refused;
                }
                if (!pattern_recipe_clone::Prepare(
                        result.opening.labels->sourceRecipe, clone.detachedShape,
                        feature, baked, recipe.prepared)) return refused;
                mutation->recipeFeatureIdentifiers.push_back(feature);
                ++reservation;
            }
            mutation->recipes.push_back(std::move(recipe));
        }
        if (reservation != issued.size()
            || mutation->recipeFeatureIdentifiers.size() != issued.size()
            || (retainedRecipeFeatures
                && *retainedRecipeFeatures
                    != mutation->recipeFeatureIdentifiers)) return refused;
        for (std::size_t index = openingDefinition.removals.size();
             index < result.candidate.removals.size(); ++index) {
            const auto& removed = result.candidate.removals[index];
            const auto* before = FindBefore(result.opening, removed.identity);
            const auto openingMember = std::find_if(
                openingDefinition.members.begin(), openingDefinition.members.end(),
                [&](const path_array::Member& member) {
                    return member.identity == removed.identity;
                });
            if (!before || openingMember == openingDefinition.members.end()
                || openingMember->localID != removed.localID
                || openingMember->coordinate.row != removed.coordinate.row
                || openingMember->coordinate.column != removed.coordinate.column
                || removed.state != pattern::MemberState::Removed
                || before->localIdentifier != removed.localID
                || !expectedRetired.insert(removed.localID).second) return refused;
            mutation->labels.removals.push_back(before->receipt);
            mutation->labels.retainedRemovalLedger.push_back(
                before->receipt.visibility.object.object.entityIdentifier);
            mutation->labels.retainedRemovalLedger.push_back(
                before->receipt.visibility.object.object.definitionIdentifier);
        }
        const std::set<std::uint64_t> candidateRetired(
            result.candidate.issuance.retiredLocalIDs.begin(),
            result.candidate.issuance.retiredLocalIDs.end());
        if (candidateRetired.size()
                != result.candidate.issuance.retiredLocalIDs.size()
            || candidateRetired != expectedRetired) return refused;
        result.native = std::move(mutation);
        return result;
    } catch (...) { return refused; }
}

bool SameLabelPlan(const OcctAllLabelPlan& left,
                   const OcctAllLabelPlan& right) noexcept {
    try {
        if (left.creates.size() != right.creates.size()
            || left.replacements.size() != right.replacements.size()
            || left.removals.size() != right.removals.size()
            || left.retainedRemovalLedger != right.retainedRemovalLedger)
            return false;
        for (std::size_t index = 0; index < left.creates.size(); ++index) {
            const auto& a = left.creates[index]; const auto& b = right.creates[index];
            if (a.identity.EntityIdentifier() != b.identity.EntityIdentifier()
                || a.identity.DefinitionIdentifier()
                    != b.identity.DefinitionIdentifier()
                || !SameClone(a.clone, b.clone)) return false;
        }
        for (std::size_t index = 0; index < left.replacements.size(); ++index) {
            const auto& a = left.replacements[index];
            const auto& b = right.replacements[index];
            if (!a.expected.IsEqual(b.expected)
                || !SameClone(a.clone, b.clone)) return false;
        }
        for (std::size_t index = 0; index < left.removals.size(); ++index)
            if (!left.removals[index].IsEqual(right.removals[index])) return false;
        return true;
    } catch (...) { return false; }
}

bool SameNativeMutation(const NativeMutation& left,
                        const NativeMutation& right) noexcept {
    try {
        if (left.canonicalCandidateBytes != right.canonicalCandidateBytes
            || left.issued.size() != right.issued.size()
            || left.recipeFeatureIdentifiers != right.recipeFeatureIdentifiers
            || left.recipes.size() != right.recipes.size()
            || !SameLabelPlan(left.labels, right.labels)) return false;
        for (std::size_t index = 0; index < left.issued.size(); ++index)
            if (left.issued[index].EntityIdentifier()
                    != right.issued[index].EntityIdentifier()
                || left.issued[index].DefinitionIdentifier()
                    != right.issued[index].DefinitionIdentifier()) return false;
        for (std::size_t index = 0; index < left.recipes.size(); ++index) {
            const auto& a = left.recipes[index]; const auto& b = right.recipes[index];
            if (a.ordinal != b.ordinal || a.created != b.created
                || a.entityIdentifier != b.entityIdentifier
                || !SamePreparedRecipe(a.prepared, b.prepared)) return false;
        }
        return true;
    } catch (...) { return false; }
}

Edit RetainedEdit(const path_array::Definition& value) {
    Edit edit;
    edit.distribution = value.distribution.mode;
    edit.count = value.distribution.count;
    edit.distance = value.distribution.distance;
    edit.includeStart = value.distribution.includeStart;
    edit.includeEnd = value.distribution.includeEnd;
    edit.closedPath = value.closedPath;
    edit.orientation = value.orientation.policy;
    edit.rollRadians = value.orientation.rollRadians;
    edit.hasUpVector = value.orientation.hasUpVector;
    edit.upVector = value.orientation.upVector;
    edit.maximumFrameStepRadians = value.orientation.maximumFrameStepRadians;
    edit.arcLengthTolerance = value.arcLengthTolerance;
    edit.minimumTangent = value.minimumTangent;
    for (std::size_t ordinal = 1; ordinal < value.members.size(); ++ordinal)
        if (value.members[ordinal].state == pattern::MemberState::Suppressed)
            edit.suppressedOrdinals.insert(std::uint32_t(ordinal));
    return edit;
}

bool RevalidateNativeMutation(OcctDocument& owner,
    const PreparedEdit& prepared,
    const std::shared_ptr<native_opening::Context>& context,
    PreparedEdit& regenerated) noexcept {
    regenerated = {};
    try {
        if (!prepared.admitted() || !prepared.native || !context
            || owner.Document().IsNull() || owner.Document()->HasOpenCommand())
            return false;
        Snapshot current;
        const std::string selected = Text(
            prepared.opening.record.definition.members.front().identity);
        if (CaptureNative(owner, selected, context, current) != Refusal::None
            || !current.admitted()
            || current.documentTime != prepared.opening.documentTime
            || !current.record.label.IsEqual(prepared.opening.record.label)
            || current.record.bytes != prepared.opening.record.bytes
            || !pattern_owner::IsExactlyEqual(*current.labels,
                                               *prepared.opening.labels))
            return false;

        LivePathResolver paths(owner, context);
        PathAuthority candidatePath;
        if (paths.resolveCurrent(prepared.candidate.path, candidatePath)
                != Refusal::None
            || !candidatePath.currentFor(prepared.candidate.path)) return false;
        Edit edit = RetainedEdit(prepared.candidate);
        edit.replacementPath = candidatePath;
        std::size_t next = 0;
        const pattern::IssueUUID issue = [&](UUID& value) {
            return next < prepared.native->issued.size()
                && receipt::ParseUUID(
                    prepared.native->issued[next++].EntityIdentifier(), value);
        };
        regenerated = Prepare(current, edit, Limits{}, issue);
        if (!regenerated.admitted()
            || next != prepared.native->issued.size()) return false;
        regenerated = PrepareMutation(owner, std::move(regenerated),
            prepared.native->issued,
            &prepared.native->recipeFeatureIdentifiers);
        if (!regenerated.admitted() || !regenerated.native
            || regenerated.candidatePath.locator.owner
                != prepared.candidatePath.locator.owner
            || regenerated.candidatePath.locator.feature
                != prepared.candidatePath.locator.feature
            || regenerated.candidatePath.locator.definitionRevision
                != prepared.candidatePath.locator.definitionRevision
            || regenerated.candidatePath.locator.canonicalDefinitionDigest
                != prepared.candidatePath.locator.canonicalDefinitionDigest
            || regenerated.placements.size() != prepared.placements.size())
            return false;
        for (std::size_t index = 0; index < prepared.placements.size(); ++index)
            if (!SamePlacement(prepared.placements[index],
                               regenerated.placements[index])) return false;
        return prepared.projection.instances == regenerated.projection.instances
            && prepared.projection.aggregateTopologyNodes
                == regenerated.projection.aggregateTopologyNodes
            && prepared.projection.projectedDocumentBytes
                == regenerated.projection.projectedDocumentBytes
            && prepared.projection.projectedMemoryBytes
                == regenerated.projection.projectedMemoryBytes
            && prepared.projection.admitted == regenerated.projection.admitted
            && prepared.buildReceipt.totalArcLength
                == regenerated.buildReceipt.totalArcLength
            && prepared.buildReceipt.maximumMeasuredArcError
                == regenerated.buildReceipt.maximumMeasuredArcError
            && prepared.buildReceipt.requestedInstances
                == regenerated.buildReceipt.requestedInstances
            && prepared.buildReceipt.emittedInstances
                == regenerated.buildReceipt.emittedInstances
            && prepared.buildReceipt.closedSeamCanonicalized
                == regenerated.buildReceipt.closedSeamCanonicalized
            && SameNativeMutation(*prepared.native, *regenerated.native);
    } catch (...) { regenerated = {}; return false; }
}
} // namespace

Refusal CaptureNative(OcctDocument& owner, const std::string& selectedEntity,
    const std::shared_ptr<native_opening::Context>& context,
    Snapshot& output) noexcept {
    OcafD3Authority labels; LivePathResolver paths(owner, context);
    return Capture(owner, selectedEntity, labels, paths, output);
}

Refusal RefreshPathNative(OcctDocument& owner, const Snapshot& opening,
    const std::string& selectedPathEntity,
    const std::shared_ptr<native_opening::Context>& context,
    Snapshot& refreshed, PathAuthority& replacement) noexcept {
    refreshed = {}; replacement = {};
    try {
        const Handle(TDocStd_Document) document = owner.Document();
        if (!context || !opening.admitted() || document.IsNull()
            || opening.record.definition.members.empty()
            || context->openingFence().document() != document
            || context->openingFence().data() != document->GetData())
            return Refusal::StalePath;

        const std::string d3Entity = Text(
            opening.record.definition.members.front().identity);
        Snapshot current;
        const Refusal d3 = CaptureNative(owner, d3Entity, context, current);
        if (d3 != Refusal::None || !current.admitted()
            || !current.record.label.IsEqual(opening.record.label)
            || current.record.bytes != opening.record.bytes
            || current.documentTime != opening.documentTime)
            return d3 == Refusal::None ? Refusal::StaleMember : d3;

        OcctBoundedCurveCapture exact;
        if (!owner.CaptureBoundedCurveExact(
                selectedPathEntity, *context, exact)
            || exact.persisted.value.definition.domain
                != bounded_curve::Domain::Path3D)
            return Refusal::MissingPath;

        bounded_curve::owner::OcafOwner c1(owner, context);
        const auto scene = bounded_curve::owner::SceneFence{
            context->openingFence().documentGeneration(),
            context->openingFence().modelRevision(),
            context->openingFence().metersPerUnit()};
        auto receipt = c1.pickCurrentPath3D(selectedPathEntity, scene);
        if (!receipt) return Refusal::StalePath;

        replacement.receipt = std::move(receipt);
        replacement.locator.owner = exact.persisted.ownerState.owner;
        replacement.locator.feature = exact.persisted.value.feature;
        replacement.locator.definitionRevision =
            exact.persisted.ownerState.definitionRevision;
        replacement.locator.canonicalDefinitionDigest =
            exact.persisted.ownerState.canonicalDefinitionDigest;
        replacement.persisted = exact.persisted;
        replacement.ownerLabel = exact.ownerReceipt.visibility.object.object.label;
        if (replacement.locator.owner.document
                != current.record.definition.owner.document
            || replacement.receipt->owner != replacement.locator.owner
            || replacement.receipt->feature != replacement.locator.feature
            || replacement.receipt->definitionRevision
                != replacement.locator.definitionRevision
            || replacement.receipt->canonicalDefinitionDigest
                != replacement.locator.canonicalDefinitionDigest
            || replacement.receipt->scene.documentGeneration
                != context->openingFence().documentGeneration()
            || replacement.receipt->scene.modelRevision
                != context->openingFence().modelRevision()
            || replacement.receipt->scene.metersPerUnit
                != context->openingFence().metersPerUnit()
            || !replacement.currentFor(replacement.locator)) {
            replacement = {};
            return Refusal::StalePath;
        }
        refreshed = std::move(current);
        return Refusal::None;
    } catch (...) {
        refreshed = {}; replacement = {}; return Refusal::StalePath;
    }
}

PreparedEdit PrepareNative(OcctDocument& owner, const Snapshot& opening,
                           const Edit& edit, const Limits& limits) noexcept {
    PreparedEdit refused; refused.opening = opening;
    try {
        std::set<UUID> before;
        for (const auto& member : opening.record.definition.members)
            before.insert(member.identity);
        std::uint32_t required = 0; double length = 0;
        const PathAuthority& path = edit.replacementPath
            ? *edit.replacementPath : opening.path;
        path_array::Definition candidate = opening.record.definition;
        candidate.path = path.locator;
        candidate.distribution = {edit.distribution, edit.count, edit.distance,
                                  edit.includeStart, edit.includeEnd};
        candidate.closedPath = edit.closedPath;
        candidate.orientation = {edit.orientation, edit.rollRadians,
            edit.hasUpVector, edit.upVector, edit.maximumFrameStepRadians};
        candidate.arcLengthTolerance = edit.arcLengthTolerance;
        candidate.minimumTangent = edit.minimumTangent;
        const auto countRefusal = path_array::RequiredInstanceCount(
            candidate, path.persisted, required, length);
        if (countRefusal != path_array::BuildRefusal::None) {
#if DEBUG
            NSLog(@"R179_D3_REFUSAL gate=required-instance-count buildRefusal=%u",
                  unsigned(countRefusal));
#endif
            return refused;
        }
        const std::size_t creates = required > candidate.members.size()
            ? required - candidate.members.size() : 0;
        std::set<std::string> ledgerSet;
        for (const auto& removed : candidate.removals)
            ledgerSet.insert(Text(removed.identity));
        std::vector<std::string> ledger(ledgerSet.begin(), ledgerSet.end());
        std::vector<OcctIssuedLabelIdentity> issued;
        if (creates && (!owner.ReserveExactLabelIdentities(
                Standard_Size(creates), ledger, issued)
                || issued.size() != creates)) return refused;
        std::size_t next = 0;
        const pattern::IssueUUID issue = [&](UUID& value) {
            return next < issued.size()
                && receipt::ParseUUID(issued[next++].EntityIdentifier(), value);
        };
        PreparedEdit result = Prepare(opening, edit, limits, issue);
        if (!result.admitted() || next != issued.size()) {
#if DEBUG
            NSLog(@"R179_D3_REFUSAL gate=prepare-or-issuance refusal=%u issued=%zu consumed=%zu",
                  unsigned(result.refusal), issued.size(), next);
#endif
            return refused;
        }
        auto prepared = PrepareMutation(owner, std::move(result), issued);
#if DEBUG
        RecordPreparedMutation(prepared);
        NSLog(@"R179_D3_PREPARE gate=native-mutation admitted=%d native=%d refusal=%u",
              int(prepared.admitted()), int(prepared.native != nullptr),
              unsigned(prepared.refusal));
#endif
        return prepared;
    } catch (...) { return refused; }
}

ApplyOutcome ApplyNative(OcctDocument& owner, const PreparedEdit& prepared,
    const std::shared_ptr<native_opening::Context>& context) noexcept {
    // Rebuild the complete detached plan after exact currentness, with the
    // original reservation handles and recipe feature identities. Equality is
    // proven before the one public Apply path is allowed to open its command.
    PreparedEdit regenerated;
    const bool revalidated = RevalidateNativeMutation(
        owner, prepared, context, regenerated);
#if DEBUG
    RecordRevalidatedMutation(prepared, regenerated, revalidated);
#endif
    if (!revalidated)
        return ApplyOutcome::Refused;
    OcafD3Authority labels; LivePathResolver paths(owner, context);
    OcafD3Stager stager(owner, context);
    const auto outcome = Apply(owner, prepared, labels, paths, stager);
#if DEBUG
    NSLog(@"R179_D3_APPLY admitted=%d native=%d outcome=%u",
          int(prepared.admitted()), int(prepared.native != nullptr), unsigned(outcome));
#endif
    // A proven commit is not delivered until the owning viewer has reconciled
    // the affected real AIS presentations to the committed OCAF geometry. An
    // unknown close retains the same plan for exact recovery instead.
    if (context && prepared.native && outcome != ApplyOutcome::Refused) {
        const auto publication =
            native_opening::PublicationFromAllLabelPlan(prepared.native->labels);
        if (outcome == ApplyOutcome::Committed)
            return context->publishCommittedEdit(publication)
                ? ApplyOutcome::Committed : ApplyOutcome::OutcomeUnknown;
        context->retainUnprovenEdit(publication);
    }
    return outcome;
}

namespace {
class D3DependentReplay final : public dependent_replay::PreparedReplay {
public:
    D3DependentReplay(dependent_replay::Dependency dependency, Snapshot opening,
        PreparedEdit prepared) noexcept
        : dependency_(std::move(dependency)), opening_(std::move(opening)),
          prepared_(std::move(prepared)) {}
    dependent_replay::Family family() const noexcept override {
        return dependent_replay::Family::PathArrayD3;
    }
    dependent_replay::UUID feature() const noexcept override { return dependency_.feature; }
    dependent_replay::UUID resultEntity() const noexcept override { return dependency_.resultEntity; }
    std::size_t documentBytes() const noexcept override {
        return opening_.pathArrayDocumentBytes + opening_.metrics.patternDocumentBytes
            + opening_.metrics.compositeDocumentBytes;
    }
    std::size_t memoryBytes() const noexcept override {
        return prepared_.placements.size() * sizeof(path_array::Placement);
    }
    std::size_t topologyNodes() const noexcept override {
        return static_cast<std::size_t>(opening_.metrics.sourceTopologyNodes);
    }
    bool openingCurrent(OcctDocument& owner) const noexcept override {
        OcafD3Authority authority;
        return authority.isCurrent(owner, opening_.labels,
                                   opening_.record.definition);
    }
    bool stage(OcctDocument& owner,
               native_opening::CommandLease& lease) const noexcept override {
        try {
            stager_ = std::make_unique<OcafD3Stager>(owner, lease);
            path_array::Record staged;
            return StageInsideOwnedCommand(owner.Document(), prepared_,
                                           *stager_, staged);
        } catch (...) { stager_.reset(); return false; }
    }
    bool read(OcctDocument&) const noexcept override {
        return stager_ && stager_->readBackAll(prepared_.candidate);
    }
private:
    dependent_replay::Dependency dependency_;
    Snapshot opening_;
    PreparedEdit prepared_;
    mutable std::unique_ptr<OcafD3Stager> stager_;
};
}

dependent_replay::Refusal PreparePathArrayD3ReplayImpl(OcctDocument& owner,
    const dependent_replay::Dependency& dependency,
    dependent_replay::Mutation mutation,
    const dependent_replay::Candidate& candidate,
    const std::shared_ptr<native_opening::Context>& context,
    std::shared_ptr<const dependent_replay::PreparedReplay>& output) noexcept {
    output.reset();
    try {
        if (mutation != dependent_replay::Mutation::Replace
            || dependency.memberIdentities.empty())
            return dependent_replay::Refusal::UnsupportedDescendant;
        Snapshot opening;
        if (CaptureNative(owner, Text(dependency.memberIdentities.front()),
                context, opening) != Refusal::None
            || opening.record.definition.feature != dependency.feature
            || opening.record.bytes != dependency.canonicalRecordBytes)
            return dependent_replay::Refusal::MissingRecipe;
        Edit edit = RetainedEdit(opening.record.definition);
        Snapshot prospective = opening;
        if (dependency.inputEntity == opening.record.definition.source.entity) {
            auto labels = std::make_shared<pattern_owner::AllLabelSnapshot>(
                *opening.labels);
            labels->source.visibility.object.object.shape = candidate.resultShape;
            prospective.labels = std::move(labels);
        }
        PreparedEdit prepared = PrepareNative(owner, prospective, edit, Limits{});
        if (!prepared.admitted() || !prepared.native)
            return dependent_replay::Refusal::MissingRecipe;
        prepared.opening = opening;
        output = std::make_shared<D3DependentReplay>(dependency,
            std::move(opening), std::move(prepared));
        return dependent_replay::Refusal::None;
    } catch (...) { output.reset(); return dependent_replay::Refusal::MissingRecipe; }
}

bool PrepareDependentReplay(OcctDocument& owner,
    const std::shared_ptr<native_opening::Context>& context,
    const std::shared_ptr<const c1_owner::Prepared>& prepared,
    DependentReplayPlan& output) noexcept {
    output = {};
    try {
        if (!context || !prepared || owner.Document().IsNull()
            || owner.Document()->HasOpenCommand()) return false;
        ProspectivePath prospective;
        prospective.seal = prepared;
        prospective.persisted = prepared->persisted;
        prospective.ownerLabel = prepared->opening.ownerLabel;
        prospective.locator.owner = prepared->persisted.ownerState.owner;
        prospective.locator.feature = prepared->persisted.value.feature;
        prospective.locator.definitionRevision =
            prepared->persisted.ownerState.definitionRevision;
        prospective.locator.canonicalDefinitionDigest =
            prepared->persisted.ownerState.canonicalDefinitionDigest;
        if (prospective.persisted.value.definition.domain
                != bounded_curve::Domain::Path3D) {
            // A non-Path3D curve can have no path-array dependent, so the
            // Path3D seal does not apply; a record that still names one is
            // an unsupported dependent and must fail closed.
            if (prospective.ownerLabel.IsNull()
                || !bounded_curve::ValidatePersisted(prepared->persisted))
                return false;
            std::vector<path_array::Record> records;
            if (!path_array::ReadAll(owner.Document(), records)) return false;
            for (const auto& record : records) {
                if (record.definition.path.owner
                        == prepared->opening.retained.authority.owner)
                    return false;
            }
            DependentReplayPlan plan; plan.c1Seal = prepared;
            plan.admitted = true;
            output = std::move(plan);
            return true;
        }
        if (!prospective.sealedFor(prospective.locator)) return false;

        std::vector<path_array::Record> records;
        if (!path_array::ReadAll(owner.Document(), records)) return false;
        DependentReplayPlan plan; plan.c1Seal = prepared;
        for (const auto& record : records) {
            if (record.definition.path.owner
                    != prepared->opening.retained.authority.owner) continue;
            Snapshot opening;
            if (CaptureNative(owner, Text(record.definition.members.front().identity),
                    context, opening) != Refusal::None) return false;
            Edit edit = RetainedEdit(record.definition);
            std::uint32_t required = 0; double length = 0;
            path_array::Definition candidate = record.definition;
            candidate.path = prospective.locator;
            if (path_array::RequiredInstanceCount(candidate, prospective.persisted,
                    required, length) != path_array::BuildRefusal::None) return false;
            const std::size_t creates = required > candidate.members.size()
                ? required - candidate.members.size() : 0;
            std::vector<OcctIssuedLabelIdentity> issued;
            if (creates && (!owner.ReserveExactLabelIdentities(
                    Standard_Size(creates), {}, issued)
                    || issued.size() != creates)) return false;
            std::size_t next = 0;
            const pattern::IssueUUID issue = [&](UUID& value) {
                return next < issued.size()
                    && receipt::ParseUUID(issued[next++].EntityIdentifier(), value);
            };
            PreparedEdit dependent = PrepareProspective(
                opening, edit, prospective, Limits{}, issue);
            if (!dependent.admitted() || next != issued.size()) return false;
            dependent = PrepareMutation(owner, std::move(dependent), issued);
            if (!dependent.admitted() || !dependent.native) return false;
            plan.arrays.push_back(std::move(dependent));
        }
        plan.admitted = true;
        output = std::move(plan);
        return true;
    } catch (...) { output = {}; return false; }
}

bool AppendDependentReplayPublication(const DependentReplayPlan& plan,
    native_opening::CommittedEditPublication& publication) noexcept {
    try {
        if (!plan.admitted || !plan.c1Seal) return false;
        native_opening::CommittedEditPublication staged;
        std::set<std::string> identities;
        const auto append = [&](const auto& source, auto& destination) {
            for (const auto& item : source) {
                // Preserve the viewer's existing aggregate publication ceiling
                // and identity checks, before there can be a committed edit.
                if (identities.size() >= 4096 || item.entityIdentifier.empty()
                    || item.entityIdentifier.size() > 128
                    || item.entityIdentifier.find('\0') != std::string::npos
                    || !identities.insert(item.entityIdentifier).second) return false;
                destination.push_back(item);
            }
            return true;
        };
        const auto appendPlan = [&](const auto& value) {
            return append(value.created, staged.created)
                && append(value.replaced, staged.replaced)
                && append(value.removed, staged.removed);
        };
        if (!appendPlan(publication)) return false;
        for (const auto& dependent : plan.arrays) {
            if (!dependent.admitted() || !dependent.native
                || !dependent.prospectivePath
                || dependent.prospectivePath->seal != plan.c1Seal) return false;
            if (!appendPlan(native_opening::PublicationFromAllLabelPlan(
                    dependent.native->labels))) return false;
        }
        if (identities.empty()) return false;
        publication = std::move(staged);
        return true;
    } catch (...) { return false; }
}

bool StageDependentReplayInsideOwnedCommand(OcctDocument& owner,
    native_opening::CommandLease& lease,
    const DependentReplayPlan& plan) noexcept {
    try {
        if (!plan.admitted || !plan.c1Seal || !lease.ownsOpenCommand()) return false;
        for (const auto& dependent : plan.arrays) {
            if (!dependent.prospectivePath
                || dependent.prospectivePath->seal != plan.c1Seal) return false;
            OcafD3Stager stager(owner, lease);
            path_array::Record staged;
            if (!StageInsideOwnedCommand(owner.Document(), dependent,
                                         stager, staged)) return false;
        }
        return true;
    } catch (...) { return false; }
}

bool VerifyDependentReplayAfterCommit(OcctDocument& owner,
    const DependentReplayPlan& plan) noexcept {
    try {
        if (!plan.admitted || owner.Document().IsNull()
            || owner.Document()->HasOpenCommand()) return false;
        std::vector<path_array::Record> records;
        if (!path_array::ReadAll(owner.Document(), records)) return false;
        OcafD3Authority authority;
        for (const auto& dependent : plan.arrays) {
            if (!dependent.native || dependent.candidate.members.empty()) return false;
            path_array::Record record; SourceMetrics metrics;
            std::shared_ptr<const pattern_owner::AllLabelSnapshot> snapshot;
            if (authority.captureCurrent(owner, records,
                    Text(dependent.candidate.members.front().identity), record,
                    snapshot, metrics) != Refusal::None
                || !snapshot
                || record.bytes != dependent.native->canonicalCandidateBytes
                || snapshot->members.size() != dependent.candidate.members.size())
                return false;
            for (const auto& recipe : dependent.native->recipes) {
                const auto member = std::find_if(snapshot->members.begin(),
                    snapshot->members.end(),
                    [&](const pattern_owner::AllLabelSnapshot::Member& value) {
                        return value.receipt.visibility.object.object.entityIdentifier
                            == recipe.entityIdentifier;
                    });
                if (member == snapshot->members.end()
                    || !PreparedRecipeMatches(recipe.prepared, member->recipe))
                    return false;
            }
        }
        return true;
    } catch (...) { return false; }
}

#if DEBUG
namespace {
// D3 admission boundary fixture: a real SYPA/1 array whose scene also carries
// unrelated roots that legitimately refuse the generic exact-label reader (a
// wire "curve-like" root and a subshape-styled box). The production resolver
// is not under test here; the injected resolver resolves exactly the staged
// persisted path so captureCurrent is the code under test.
struct D3ProbeObject {
    UUID entity{}, definition{};
    TDF_Label label;
};

UUID D3ProbeID() {
    UUID value{};
    [[NSUUID UUID] getUUIDBytes:value.data()];
    return value;
}

bool D3ProbeSetUUID(const TDF_Label& label, const char* attributeID,
                    const UUID& value) {
    try {
        return !TDataStd_AsciiString::Set(label, Standard_GUID(attributeID),
            TCollection_AsciiString(Text(value).c_str())).IsNull();
    } catch (...) { return false; }
}

bool D3ProbeStageRoot(const Handle(XCAFDoc_ShapeTool)& shapes,
                      const TopoDS_Shape& shape, D3ProbeObject& object) {
    try {
        const TDF_Label owner = shapes->AddShape(shape, Standard_False,
                                                 Standard_True);
        if (owner.IsNull() || shape.IsNull()) return false;
        object.label = owner;
        if (TDataStd_Integer::Set(owner,
                Standard_GUID("67E669F4-00C0-4C45-BC55-9CC5DA22A2B5"), 1).IsNull())
            return false;
        return D3ProbeSetUUID(owner, "0074F7C2-9EAA-4F89-B2DE-8716E155FF62",
                object.entity)
            && D3ProbeSetUUID(owner, "3611F2B2-C694-4E12-AED8-A2A97A3D283B",
                object.definition);
    } catch (...) { return false; }
}

class D3ProbeResolver final : public CurrentPathResolver {
public:
    D3ProbeResolver(bounded_curve::PersistedValue persisted,
                    TDF_Label ownerLabel) noexcept
        : persisted_(std::move(persisted)), ownerLabel_(ownerLabel) {}

    Refusal resolveCurrent(const path_array::CurveReference& reference,
                           PathAuthority& output) noexcept override {
        output = {};
        try {
            if (ownerLabel_.IsNull()
                || !path_array::Matches(reference, persisted_))
                return Refusal::StalePath;
            output.receipt = std::make_shared<c1_owner::PathReceipt>();
            output.locator = reference;
            output.persisted = persisted_;
            output.ownerLabel = ownerLabel_;
            return output.currentFor(reference) ? Refusal::None
                                                : Refusal::StalePath;
        } catch (...) { output = {}; return Refusal::StalePath; }
    }
private:
    bounded_curve::PersistedValue persisted_;
    TDF_Label ownerLabel_;
};

std::uint64_t RunD3AdmissionProbe() noexcept {
    try {
        OcctDocument owner;
        owner.InitDoc();
        const Handle(TDocStd_Document) document = owner.Document();
        if (document.IsNull()) return 0;
        XCAFDoc_DocumentTool::SetLengthUnit(document, 0.001);
        const Handle(XCAFDoc_ShapeTool) shapes =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return 0;

        const UUID documentID = D3ProbeID();
        if (!D3ProbeSetUUID(document->Main(),
                "74386E4E-F620-498F-8092-E6D883AF33A4", documentID)) return 0;
        D3ProbeObject source{D3ProbeID(), D3ProbeID(), TDF_Label()};
        D3ProbeObject member{D3ProbeID(), D3ProbeID(), TDF_Label()};
        D3ProbeObject array{D3ProbeID(), D3ProbeID(), TDF_Label()};
        D3ProbeObject styled{D3ProbeID(), D3ProbeID(), TDF_Label()};
        const TopoDS_Shape box = BRepPrimAPI_MakeBox(20, 12, 8).Shape();
        if (!D3ProbeStageRoot(shapes, box, source)
            || !D3ProbeStageRoot(shapes, BRepPrimAPI_MakeBox(20, 12, 8).Shape(),
                                 member)
            || !D3ProbeStageRoot(shapes, BRepPrimAPI_MakeBox(20, 12, 8).Shape(),
                                 array)) return 0;
        // Unrelated roots: one subshape-styled box that the generic mesh-copy
        // reader must refuse, and one wire curve-like root with a named-shape
        // record child. Neither is a source or member of the array.
        if (!D3ProbeStageRoot(shapes, BRepPrimAPI_MakeBox(6, 6, 6).Shape(),
                              styled)) return 0;
        TopoDS_Shape styledFace;
        for (TopExp_Explorer it(box, TopAbs_FACE); it.More(); it.Next()) {
            styledFace = it.Current(); break;
        }
        if (styledFace.IsNull()) return 0;
        shapes->AddSubShape(styled.label, styledFace);

        auto persisted = native_opening::debug::wire_probe::Fixture();
        persisted.ownerState.owner.document = documentID;
        persisted.ownerState.owner.entity = D3ProbeID();
        persisted.ownerState.owner.definition = D3ProbeID();
        bounded_curve::DetachedWire detached;
        if (bounded_curve::BuildWire(persisted, detached)
                != bounded_curve::BuildRefusal::None) return 0;
        D3ProbeObject wire{persisted.ownerState.owner.entity,
            persisted.ownerState.owner.definition, TDF_Label()};
        if (!D3ProbeStageRoot(shapes, detached.wire, wire)) return 0;
        const TDF_Label wireRecord = wire.label.FindChild(
            bounded_curve::MinimumRecordTag, Standard_True);
        TNaming_Builder(wireRecord).Select(detached.wire, detached.wire);

        path_array::Definition definition;
        definition.owner = {documentID, array.entity, array.definition};
        definition.feature = D3ProbeID();
        definition.source = {documentID, source.entity, source.definition,
            D3ProbeID()};
        definition.path.owner = persisted.ownerState.owner;
        definition.path.feature = persisted.value.feature;
        definition.path.definitionRevision =
            persisted.ownerState.definitionRevision;
        definition.path.canonicalDefinitionDigest =
            persisted.ownerState.canonicalDefinitionDigest;
        definition.distribution = {path_array::DistributionMode::Count,
            2, 0, true, true};
        definition.orientation.policy = path_array::OrientationPolicy::Fixed;
        definition.orientation.rollRadians = 0;
        definition.orientation.hasUpVector = false;
        definition.orientation.upVector = {{0, 0, 1}};
        definition.orientation.maximumFrameStepRadians = 0.1;
        definition.arcLengthTolerance = 1e-4;
        definition.minimumTangent = 1e-6;
        definition.issuance.nextLocalID = 3;
        definition.members = {
            {source.entity, 1, {0, 0}, pattern::MemberState::Active},
            {member.entity, 2, {0, 1}, pattern::MemberState::Active},
        };
        if (!path_array::Valid(definition)
            || !path_array::Matches(definition.path, persisted)) return 0;
        document->NewCommand();
        path_array::Record staged;
        if (!path_array::Stage(document, definition, staged)
            || !document->CommitCommand()) {
            document->AbortCommand();
            return 0;
        }
        std::vector<path_array::Record> records;
        if (!path_array::ReadAll(document, records) || records.size() != 1)
            return 0;

        std::uint64_t bits = 0;
        OcafD3Authority authority;
        const std::string selected = Text(source.entity);
        // Unrelated roots that refuse the generic exact-label reader no longer
        // break admission of the array's own source and members.
        path_array::Record selectedRecord;
        std::shared_ptr<const pattern_owner::AllLabelSnapshot> labels;
        SourceMetrics metrics;
        if (authority.captureCurrent(owner, records, selected, selectedRecord,
                labels, metrics) == Refusal::None && labels
            && labels->members.size() == 2) {
            Snapshot snapshot;
            snapshot.record = selectedRecord;
            snapshot.labels = labels;
            if (snapshot.hasCompleteOrdinalAuthority()) bits |= 1;
        }
        // The same labels admit through the full Capture with the exact
        // staged path.
        {
            D3ProbeResolver resolver(persisted, wire.label);
            Snapshot opening;
            if (Capture(owner, selected, authority, resolver, opening)
                    == Refusal::None && opening.admitted()) bits |= 2;
        }
        // Duplicate and missing identities still refuse.
        if (D3ProbeSetUUID(member.label, "0074F7C2-9EAA-4F89-B2DE-8716E155FF62",
                source.entity)) {
            path_array::Record ignored;
            std::shared_ptr<const pattern_owner::AllLabelSnapshot> snapshot;
            SourceMetrics ignoredMetrics;
            if (authority.captureCurrent(owner, records, selected, ignored,
                    snapshot, ignoredMetrics) == Refusal::CorruptTable)
                bits |= 4;
            if (!D3ProbeSetUUID(member.label,
                    "0074F7C2-9EAA-4F89-B2DE-8716E155FF62", member.entity))
                return bits;
        }
        // An owned member whose receipt the exact reader refuses still fails
        // closed instead of being silently skipped.
        TopoDS_Shape memberFace;
        for (TopExp_Explorer it(XCAFDoc_ShapeTool::GetShape(member.label),
                TopAbs_FACE); it.More(); it.Next()) {
            memberFace = it.Current(); break;
        }
        if (!memberFace.IsNull()) {
            document->NewCommand();
            const TDF_Label invalidOwnedSubshape =
                shapes->AddSubShape(member.label, memberFace);
            path_array::Record ignored;
            std::shared_ptr<const pattern_owner::AllLabelSnapshot> snapshot;
            SourceMetrics ignoredMetrics;
            if (!invalidOwnedSubshape.IsNull()
                && authority.captureCurrent(owner, records, selected, ignored,
                    snapshot, ignoredMetrics) == Refusal::CorruptTable)
                bits |= 8;
            document->AbortCommand();
            if (document->HasOpenCommand()) return bits;
        }
        // A stale referenced path still refuses after the labels admit.
        {
            auto tampered = persisted;
            tampered.ownerState.canonicalDefinitionDigest[0] ^= 0x5a;
            D3ProbeResolver resolver(tampered, wire.label);
            Snapshot opening;
            if (Capture(owner, selected, authority, resolver, opening)
                    == Refusal::StalePath) bits |= 16;
        }
        return bits;
    } catch (...) { return 0; }
}

// ---------------------------------------------------------------------------
// R179 D3 native collaborator probe fixture and scenario bodies.
//
// The GL-backed fixture mirrors the proven OwnerProbeFixture
// (BoundedCurveOwner.mm) and the R179 staging helpers of
// LifecycleProbeFixture (Core3DNativeOpenings.mm). Those helpers are
// file-static in their translation units, so the minimal staging is
// replicated here. Every scenario below drives the production collaborators
// against a real viewer document and derives every returned bit from
// measured records, receipts, resolver verdicts and history.
// ---------------------------------------------------------------------------

#if TARGET_OS_IOS
struct D3ProbeContextRestorer final {
    __strong EAGLContext *context = nil;
    ~D3ProbeContextRestorer() noexcept {
        (void)[EAGLContext setCurrentContext:context];
    }
};
#endif

struct D3ProbeCurve final {
    bounded_curve::PersistedValue persisted;
    bounded_curve::DetachedWire detached;
    TDF_Label label;
};

bool D3ProbeStageNamedRoot(const Handle(XCAFDoc_ShapeTool)& shapes,
                           const TopoDS_Shape& shape, const char* name,
                           D3ProbeObject& object) {
    if (!D3ProbeStageRoot(shapes, shape, object)) return false;
    try {
        TDataStd_Name::Set(object.label, TCollection_ExtendedString(name));
        return true;
    } catch (...) { return false; }
}

// Minimal replication of Core3DNativeOpenings.mm's file-static R179StageCurve:
// a real native C1 Path3D wire root plus its canonical record payload. The
// gentle profile keeps D3's unchanged 0.1-radian tangent limit satisfied.
bool D3ProbeStageCurve(const Handle(XCAFDoc_ShapeTool)& shapes,
                       const UUID& documentUUID, double nativePerMM,
                       const char* name, D3ProbeCurve& output) {
    try {
        auto persisted = native_opening::debug::wire_probe::Fixture();
        persisted.ownerState.owner.document = documentUUID;
        persisted.ownerState.owner.entity = D3ProbeID();
        persisted.ownerState.owner.definition = D3ProbeID();
        persisted.ownerState.feature = D3ProbeID();
        persisted.value.feature = persisted.ownerState.feature;
        persisted.value.definition.frame.identifier = D3ProbeID();
        for (double& scalar : persisted.value.definition.frame.origin)
            scalar *= nativePerMM;
        for (auto& pole : persisted.value.definition.controlPoints) {
            pole.identifier = D3ProbeID();
            pole.local[1] *= 0.01;
            pole.local[2] *= 0.01;
            for (double& scalar : pole.local) scalar *= nativePerMM;
        }
        std::vector<std::uint8_t> bytes;
        if (!bounded_curve::Encode(persisted.value, bytes)
            || !bounded_curve::Hash(bytes, bounded_curve::MaximumDefinitionBytes,
                persisted.ownerState.canonicalDefinitionDigest)) return false;
        if (bounded_curve::BuildWire(persisted, output.detached)
                != bounded_curve::BuildRefusal::None) return false;
        output.persisted = persisted;
        D3ProbeObject object{persisted.ownerState.owner.entity,
            persisted.ownerState.owner.definition, TDF_Label()};
        if (!D3ProbeStageNamedRoot(shapes, output.detached.wire, name, object))
            return false;
        output.label = object.label;
        return true;
    } catch (...) { return false; }
}

// Same payload attach as Core3DNativeOpenings.mm's R179AttachCurveRecord.
bool D3ProbeAttachCurveRecord(const D3ProbeCurve& curve) {
    try {
        const TDF_Label record = curve.label.FindChild(
            bounded_curve::MinimumRecordTag, Standard_True);
        auto payload = std::make_shared<bounded_curve::Payload>();
        payload->persisted = curve.persisted;
        payload->definitionBytes = curve.detached.canonicalDefinitionBytes;
        payload->ownerBytes = curve.detached.canonicalOwnerBytes;
        if (!bounded_curve::PersistenceProbe::Attach(record, payload))
            return false;
        TNaming_Builder(record).Select(curve.detached.wire, curve.detached.wire);
        return true;
    } catch (...) { return false; }
}

// The staged SYPA/1 definition: two active members on the current path, with
// every staged path first proven against the production instance-count law.
bool D3ProbeBuildDefinition(const UUID& documentUUID,
                            const D3ProbeObject& source,
                            const D3ProbeObject& member,
                            const D3ProbeObject& array,
                            const D3ProbeCurve& currentPath,
                            const D3ProbeCurve* replacementPath,
                            path_array::Definition& output) {
    output = {};
    output.owner = {documentUUID, array.entity, array.definition};
    output.feature = D3ProbeID();
    output.source = {documentUUID, source.entity, source.definition,
        D3ProbeID()};
    output.path.owner = currentPath.persisted.ownerState.owner;
    output.path.feature = currentPath.persisted.value.feature;
    output.path.definitionRevision =
        currentPath.persisted.ownerState.definitionRevision;
    output.path.canonicalDefinitionDigest =
        currentPath.persisted.ownerState.canonicalDefinitionDigest;
    output.distribution = {path_array::DistributionMode::Count, 2, 0, true,
        true};
    output.orientation.policy = path_array::OrientationPolicy::Fixed;
    output.orientation.rollRadians = 0;
    output.orientation.hasUpVector = false;
    output.orientation.upVector = {{0, 0, 1}};
    output.orientation.maximumFrameStepRadians = 0.1;
    output.arcLengthTolerance = 1e-4;
    output.minimumTangent = 1e-6;
    output.issuance.nextLocalID = 3;
    output.members = {
        {source.entity, 1, {0, 0}, pattern::MemberState::Active},
        {member.entity, 2, {0, 1}, pattern::MemberState::Active},
    };
    if (!path_array::Valid(output)) {
        NSLog(@"R179_D3_COLLABORATOR_DEF gate=valid");
        return false;
    }
    if (!path_array::Matches(output.path, currentPath.persisted)) {
        NSLog(@"R179_D3_COLLABORATOR_DEF gate=matches");
        return false;
    }
    std::vector<const bounded_curve::PersistedValue*> paths{
        &currentPath.persisted};
    if (replacementPath) paths.push_back(&replacementPath->persisted);
    for (const auto* path : paths) {
        auto candidate = output;
        candidate.path.owner = path->ownerState.owner;
        candidate.path.feature = path->value.feature;
        candidate.path.definitionRevision = path->ownerState.definitionRevision;
        candidate.path.canonicalDefinitionDigest =
            path->ownerState.canonicalDefinitionDigest;
        std::uint32_t count = 0;
        double length = 0;
        const auto status = path_array::RequiredInstanceCount(candidate, *path,
            count, length);
        if (status != path_array::BuildRefusal::None
            || count != output.members.size()) {
            NSLog(@"R179_D3_COLLABORATOR_DEF gate=required-count status=%u count=%u length=%g",
                  unsigned(status), unsigned(count), length);
            return false;
        }
    }
    return true;
}

struct D3CollaboratorProbeFixture final {
    core3d::Core3DViewer viewer;
    Handle(OcctDocument) document;
    std::vector<Handle(AIS_Shape)> presentations;
    D3ProbeObject source, member, array;
    D3ProbeCurve currentPath, replacementPath;
    path_array::Record arrayRecord;
    std::string sourceRecipeIdentifier, memberRecipeIdentifier;
    // Descriptive failure provenance only; never contributes an evidence bit.
    const char* setupStage = "allocate-gl-host";
    bool cleanupSucceeded = true;
#if TARGET_OS_IOS
    __strong GLView *host = nil;
#endif

    ~D3CollaboratorProbeFixture() noexcept { shutdown(); }

    bool perform(void (^work)(void)) noexcept {
#if TARGET_OS_IOS
        D3ProbeContextRestorer restore{[EAGLContext currentContext]};
        try {
            return host != nil && work != nil
                && [host debugPerformWithProbeFramebuffer:work];
        } catch (const Standard_Failure& failure) {
            NSLog(@"D3 collaborator probe wrapper failed: %.256s",
                  failure.GetMessageString() ?: "Standard_Failure");
            return false;
        } catch (...) { return false; }
#else
        (void)work;
        return false;
#endif
    }

    void shutdown() noexcept {
#if TARGET_OS_IOS
        if (host == nil) return;
        D3ProbeContextRestorer restore{[EAGLContext currentContext]};
        D3CollaboratorProbeFixture *fixture = this;
        __block bool cleaned = false;
        void (^cleanup)(void) = ^{
            try {
                if (!fixture->document.IsNull()
                    && !fixture->document->Document().IsNull()
                    && fixture->document->Document()->HasOpenCommand())
                    fixture->document->Document()->AbortCommand();
                fixture->viewer.release();
                cleaned = true;
            } catch (const Standard_Failure& failure) {
                NSLog(@"D3 collaborator probe cleanup failed: %.256s",
                      failure.GetMessageString() ?: "Standard_Failure");
            } catch (...) {}
        };
        bool performed = perform(cleanup);
        if (!performed && !cleaned) {
            try { performed = [host performWithRenderingContext:cleanup]; }
            catch (...) { performed = false; }
        }
        cleanupSucceeded = cleanupSucceeded && performed && cleaned;
        presentations.clear();
        document.Nullify();
        host = nil;
#endif
    }

    std::shared_ptr<native_opening::Context> context(
        const std::string& entity) noexcept {
        __block std::shared_ptr<native_opening::Context> result;
        D3CollaboratorProbeFixture *fixture = this;
        if (!perform(^{
            result = fixture->viewer.captureNativeOpeningContext(
                64, 64, {entity});
        })) return {};
        return result;
    }

    bool initialize(double metersPerUnit, bool withReplacementPath,
                    bool withRecipes) noexcept {
#if !TARGET_OS_IOS
        (void)metersPerUnit; (void)withReplacementPath; (void)withRecipes;
        return false;
#else
        try {
            D3ProbeContextRestorer restore{[EAGLContext currentContext]};
            host = [[GLView alloc] initWithFrame:
                CGRectMake(0.0, 0.0, 64.0, 64.0)];
            setupStage = "prepare-probe-framebuffer";
            if (host == nil || host->myGLContext == nil
                || ![host debugPrepareProbeFramebuffer]) {
                host = nil;
                return false;
            }
            setupStage = "restore-caller-context";
            if (![EAGLContext setCurrentContext:restore.context]) {
                shutdown();
                return false;
            }
            __block bool initialized = false;
            D3CollaboratorProbeFixture *fixture = this;
            const bool performed = perform(^{
                fixture->setupStage = "initialize-viewer-and-interactors";
                if (!fixture->viewer.InitViewer(fixture->host)
                    || fixture->viewer.getObjectInteractor() == nullptr
                    || fixture->viewer.getShapeInteractor() == nullptr
                    || fixture->viewer.AisContext().IsNull()
                    || fixture->viewer.ActiveView().IsNull()) return;
                fixture->setupStage = "viewer-document";
                fixture->document = fixture->viewer.getDocument();
                if (fixture->document.IsNull()
                    || fixture->document->Document().IsNull()
                    || fixture->document->Document()->GetData().IsNull()
                    || fixture->document->Document()->HasOpenCommand()) return;
                const Handle(TDocStd_Document) document =
                    fixture->document->Document();
                fixture->setupStage = "set-length-unit";
                XCAFDoc_DocumentTool::SetLengthUnit(document, metersPerUnit);
                fixture->setupStage = "document-uuid";
                UUID documentUUID{};
                if (!core3d::pattern_owner::Parse(
                        fixture->document->DocumentIdentifier(), documentUUID)) {
                    documentUUID = D3ProbeID();
                    if (!D3ProbeSetUUID(document->Main(),
                            "74386E4E-F620-498F-8092-E6D883AF33A4",
                            documentUUID)) return;
                }
                fixture->setupStage = "shape-tool";
                const Handle(XCAFDoc_ShapeTool) shapes =
                    XCAFDoc_DocumentTool::ShapeTool(document->Main());
                if (shapes.IsNull()) return;
                const double nativePerMM = 0.001 / metersPerUnit;
                fixture->setupStage = "stage-objects";
                fixture->source = {D3ProbeID(), D3ProbeID(), TDF_Label()};
                fixture->member = {D3ProbeID(), D3ProbeID(), TDF_Label()};
                fixture->array = {D3ProbeID(), D3ProbeID(), TDF_Label()};
                if (!D3ProbeStageNamedRoot(shapes, BRepPrimAPI_MakeBox(
                            20 * nativePerMM, 12 * nativePerMM,
                            8 * nativePerMM).Shape(),
                        "R179 D3 Source", fixture->source)
                    || !D3ProbeStageNamedRoot(shapes, BRepPrimAPI_MakeBox(
                            20 * nativePerMM, 12 * nativePerMM,
                            8 * nativePerMM).Shape(),
                        "R179 D3 Member", fixture->member)
                    || !D3ProbeStageNamedRoot(shapes, BRepPrimAPI_MakeBox(
                            20 * nativePerMM, 12 * nativePerMM,
                            8 * nativePerMM).Shape(),
                        "R179 D3 Array", fixture->array)
                    || !D3ProbeStageCurve(shapes, documentUUID, nativePerMM,
                        "R179 D3 Path", fixture->currentPath)
                    || (withReplacementPath
                        && !D3ProbeStageCurve(shapes, documentUUID,
                            nativePerMM, "R179 D3 Replacement Path",
                            fixture->replacementPath))) return;
                fixture->setupStage = "build-definition";
                path_array::Definition definition;
                if (!D3ProbeBuildDefinition(documentUUID, fixture->source,
                        fixture->member, fixture->array, fixture->currentPath,
                        withReplacementPath ? &fixture->replacementPath
                                            : nullptr,
                        definition)) return;
                fixture->setupStage = "stage-records";
                document->SetUndoLimit(16);
                document->NewCommand();
                bool staged = D3ProbeAttachCurveRecord(fixture->currentPath)
                    && (!withReplacementPath
                        || D3ProbeAttachCurveRecord(fixture->replacementPath));
                if (staged && withRecipes) {
                    // A real sweep recipe on source and member so the later
                    // growth edit stages a genuinely cloned member recipe.
                    const auto sweepDefinition =
                        core3d::planar_sweep::probe::Fixture(4, metersPerUnit);
                    fixture->sourceRecipeIdentifier =
                        OcctDocument::NewProfileIdentifier();
                    fixture->memberRecipeIdentifier =
                        OcctDocument::NewProfileIdentifier();
                    staged = !fixture->sourceRecipeIdentifier.empty()
                        && !fixture->memberRecipeIdentifier.empty()
                        && fixture->sourceRecipeIdentifier
                            != fixture->memberRecipeIdentifier
                        && core3d::sweep_persistence::Stage(document,
                            fixture->source.label, sweepDefinition,
                            fixture->sourceRecipeIdentifier)
                        && core3d::sweep_persistence::Stage(document,
                            fixture->member.label, sweepDefinition,
                            fixture->memberRecipeIdentifier);
                }
                path_array::Record stagedRecord;
                staged = staged && path_array::Stage(document, definition,
                    stagedRecord);
                if (!staged || !document->CommitCommand()) {
                    if (document->HasOpenCommand()) document->AbortCommand();
                    return;
                }
                fixture->setupStage = "readback";
                std::vector<path_array::Record> records;
                std::vector<core3d::bounded_curve::Record> curves;
                if (!path_array::ReadAll(document, records)
                    || records.size() != 1
                    || records.front().bytes != stagedRecord.bytes
                    || !core3d::bounded_curve::ReadAll(document, curves)
                    || curves.size() != (withReplacementPath ? 2U : 1U))
                    return;
                fixture->arrayRecord = records.front();
                fixture->setupStage = "publish-staged-shapes";
                document->ClearUndos();
                fixture->document->NotifyChanges();
                const auto ais = fixture->viewer.AisContext();
                TDF_LabelSequence roots;
                shapes->GetFreeShapes(roots);
                std::set<std::string> expected, published;
                expected.insert(Text(fixture->source.entity));
                expected.insert(Text(fixture->member.entity));
                expected.insert(Text(fixture->array.entity));
                expected.insert(Text(
                    fixture->currentPath.persisted.ownerState.owner.entity));
                if (withReplacementPath)
                    expected.insert(Text(fixture->replacementPath
                        .persisted.ownerState.owner.entity));
                for (Standard_Integer index = 1; index <= roots.Length();
                     ++index) {
                    const TDF_Label label = roots.Value(index);
                    const TopoDS_Shape shape =
                        XCAFDoc_ShapeTool::GetShape(label);
                    if (shape.IsNull()) return;
                    Handle(AIS_Shape) presentation = new AIS_Shape(shape);
                    ais->Display(presentation, AIS_Shaded, 0, Standard_False);
                    fixture->presentations.push_back(presentation);
                    const std::string entity =
                        fixture->document->EntityIdentifierForLabel(label);
                    if (!entity.empty()) published.insert(entity);
                }
                fixture->setupStage = "published-entity-set";
                for (const std::string& entity : expected)
                    if (!published.count(entity)) return;
                fixture->setupStage = "whole-shape-selection";
                const auto shapeInteractor =
                    fixture->viewer.getShapeInteractor();
                if (shapeInteractor->setSelectionMode(
                        core3d::ShapeSelectionMode::WholeShape)
                        != core3d::ShapeSelectionModeChangeResult::Succeeded
                    || !shapeInteractor->selectionModeAuthorityIsExact())
                    return;
                fixture->setupStage = "redraw-and-snapshot";
                ais->UpdateCurrentViewer();
                fixture->viewer.ActiveView()->FitAll();
                const auto snapshot =
                    fixture->viewer.captureSceneSnapshot(64, 64);
                fixture->setupStage = "snapshot-invariants";
                if (!snapshot || snapshot->publicationSourceIdentifier.empty()
                    || snapshot->metersPerUnit != metersPerUnit
                    || document->HasOpenCommand()
                    || document->GetAvailableUndos() != 0
                    || document->GetAvailableRedos() != 0) return;
                fixture->setupStage = "snapshot-entity-set";
                for (const std::string& entity : expected)
                    if (std::none_of(snapshot->instances.begin(),
                            snapshot->instances.end(),
                            [&](const core3d::scene::InstanceSnapshot&
                                    instance) {
                                return instance.entityIdentifier == entity;
                            })) return;
                fixture->setupStage = "done";
                initialized = true;
            });
            if (!performed || !initialized) {
                NSLog(@"R179_D3_COLLABORATOR_SETUP stage=%s performed=%d initialized=%d",
                      setupStage, int(performed), int(initialized));
                shutdown();
                return false;
            }
            return true;
        } catch (const Standard_Failure& failure) {
            NSLog(@"D3 collaborator probe initialization failed: %.256s",
                  failure.GetMessageString() ?: "Standard_Failure");
            shutdown();
            return false;
        } catch (...) { shutdown(); return false; }
#endif
    }
};

// Measured values from the same executed scenario runs. complete is set only
// when the scenario earned every bit; the readback seam refuses (~0) before
// that so a partial run can never masquerade as measured evidence.
struct D3ProbeReadbacks final {
    std::uint64_t values[4] = {~0ull, ~0ull, ~0ull, ~0ull};
    bool complete = false;
};

D3ProbeReadbacks& D3ProbeStoredReadbacks(std::int32_t scenario) noexcept {
    static thread_local D3ProbeReadbacks slots[4];
    return slots[scenario];
}

// Scenario 0: the production LivePathResolver resolves the staged C1 Path3D
// locator against the real document and returns the same owner label, receipt
// authority and persisted value an independent exact read reports.
std::uint64_t RunD3ProductionResolverProbe(D3ProbeReadbacks& readbacks)
    noexcept {
    D3CollaboratorProbeFixture fixture;
    if (!fixture.initialize(0.001, false, false)) return 0;
    __block std::uint64_t bits = 0;
    __block D3ProbeReadbacks measured;
    D3CollaboratorProbeFixture *fixturePointer = &fixture;
    if (!fixture.perform(^{
        OcctDocument& owner = *fixturePointer->document;
        const Handle(TDocStd_Document) document = owner.Document();
        const auto context = fixturePointer->context(Text(
            fixturePointer->currentPath.persisted.ownerState.owner.entity));
        if (!context) return;
        Snapshot opening;
        if (CaptureNative(owner, Text(fixturePointer->source.entity), context,
                opening) != Refusal::None || !opening.admitted()) return;
        bits |= 1;
        LivePathResolver resolver(owner, context);
        PathAuthority authority;
        if (resolver.resolveCurrent(opening.record.definition.path, authority)
                != Refusal::None
            || !authority.currentFor(opening.record.definition.path)) return;
        bits |= 2;
        OcctBoundedCurveCapture exact;
        if (!authority.receipt
            || !owner.ReadBoundedCurveExact(
                opening.record.definition.path.owner, exact)
            || !(authority.receipt->owner == authority.locator.owner)
            || authority.receipt->feature != authority.locator.feature
            || authority.receipt->definitionRevision
                != authority.locator.definitionRevision
            || authority.receipt->canonicalDefinitionDigest
                != authority.locator.canonicalDefinitionDigest
            || !authority.ownerLabel.IsEqual(
                exact.ownerReceipt.visibility.object.object.label)
            || !bounded_curve::owner::ValidPathReceipt(*authority.receipt))
            return;
        bits |= 4;
        std::vector<std::uint8_t> resolvedDefinition;
        if (!bounded_curve::Encode(authority.persisted.value,
                resolvedDefinition)
            || resolvedDefinition != exact.record.value->definitionBytes
            || !(authority.persisted.ownerState.owner
                == exact.persisted.ownerState.owner)
            || authority.persisted.ownerState.feature
                != exact.persisted.ownerState.feature
            || authority.persisted.ownerState.definitionRevision
                != exact.persisted.ownerState.definitionRevision
            || authority.persisted.ownerState.canonicalDefinitionDigest
                != exact.persisted.ownerState.canonicalDefinitionDigest)
            return;
        bits |= 8;
        path_array::CurveReference tampered = opening.record.definition.path;
        tampered.canonicalDefinitionDigest[0] ^= 0x5a;
        PathAuthority stale;
        if (resolver.resolveCurrent(tampered, stale) != Refusal::StalePath
            || stale.currentFor(tampered)
            || document->HasOpenCommand()
            || document->GetAvailableUndos() != 0) return;
        bits |= 16;
        measured.values[0] = authority.receipt->definitionRevision;
        measured.values[1] = authority.receipt->issuance;
        measured.values[2] = std::uint64_t(
            authority.persisted.value.definition.controlPoints.size());
        measured.values[3] = std::uint64_t(document->GetAvailableUndos());
    })) return 0;
    readbacks = measured;
    return bits;
}

// Scenario 1: a real replacement path is minted by RefreshPathNative, the
// edit is admitted by PrepareNative, the replacement path record is then
// changed inside the document, and ApplyNative refuses without touching the
// array record or history. The two independent resolver verdicts are measured
// after the refusal: the opening path still resolves current, the changed
// replacement resolves stale.
std::uint64_t RunD3ReplacementChangedProbe(D3ProbeReadbacks& readbacks)
    noexcept {
    D3CollaboratorProbeFixture fixture;
    if (!fixture.initialize(0.001, true, false)) return 0;
    __block std::uint64_t bits = 0;
    __block D3ProbeReadbacks measured;
    D3CollaboratorProbeFixture *fixturePointer = &fixture;
    if (!fixture.perform(^{
        OcctDocument& owner = *fixturePointer->document;
        const Handle(TDocStd_Document) document = owner.Document();
        const auto context = fixturePointer->context(
            Text(fixturePointer->source.entity));
        if (!context) return;
        Snapshot opening;
        if (CaptureNative(owner, Text(fixturePointer->source.entity), context,
                opening) != Refusal::None || !opening.admitted()) return;
        bits |= 1;
        Snapshot refreshed;
        PathAuthority replacement;
        if (RefreshPathNative(owner, opening,
                Text(fixturePointer->replacementPath.persisted.ownerState
                    .owner.entity),
                context, refreshed, replacement) != Refusal::None
            || !replacement.currentFor(replacement.locator)) return;
        bits |= 2;
        Edit edit = RetainedEdit(refreshed.record.definition);
        edit.replacementPath = replacement;
        const PreparedEdit prepared =
            PrepareNative(owner, refreshed, edit, Limits{});
        if (!prepared.admitted() || !prepared.native
            || !prepared.candidatePath.currentFor(prepared.candidate.path)
            || !(prepared.candidate.path.owner
                == fixturePointer->replacementPath.persisted.ownerState.owner))
            return;
        bits |= 4;
        // Change the replacement path inside the document between the
        // admitted prepare and the apply: one real committed revision bump.
        document->NewCommand();
        const TDF_Label record = fixturePointer->replacementPath.label
            .FindChild(bounded_curve::MinimumRecordTag, Standard_False);
        auto modified = fixturePointer->replacementPath.persisted;
        ++modified.ownerState.definitionRevision;
        std::vector<std::uint8_t> ownerBytes;
        bool mutated = !record.IsNull()
            && bounded_curve::EncodeOwnerState(modified.ownerState, ownerBytes);
        if (mutated) {
            record.ForgetAllAttributes(Standard_True);
            auto payload = std::make_shared<bounded_curve::Payload>();
            payload->persisted = modified;
            payload->definitionBytes = fixturePointer->replacementPath
                .detached.canonicalDefinitionBytes;
            payload->ownerBytes = ownerBytes;
            mutated = bounded_curve::PersistenceProbe::Attach(record, payload);
            if (mutated)
                TNaming_Builder(record).Select(
                    fixturePointer->replacementPath.detached.wire,
                    fixturePointer->replacementPath.detached.wire);
        }
        if (mutated && !document->CommitCommand()) mutated = false;
        if (!mutated) {
            if (document->HasOpenCommand()) document->AbortCommand();
            return;
        }
        OcctBoundedCurveCapture changed;
        if (!owner.ReadBoundedCurveExact(modified.ownerState.owner, changed)
            || changed.persisted.ownerState.definitionRevision
                != fixturePointer->replacementPath.persisted.ownerState
                    .definitionRevision + 1) return;
        const int undosBeforeApply = document->GetAvailableUndos();
        const auto outcome = ApplyNative(owner, prepared, context);
        if (outcome != ApplyOutcome::Refused) return;
        bits |= 8;
        LivePathResolver resolver(owner, context);
        PathAuthority openingPath, replacementNow;
        path_array::Record after;
        if (resolver.resolveCurrent(opening.record.definition.path,
                    openingPath) != Refusal::None
            || !openingPath.currentFor(opening.record.definition.path)
            || resolver.resolveCurrent(prepared.candidate.path,
                    replacementNow) != Refusal::StalePath
            || !path_array::ReadFeature(document,
                    opening.record.definition.feature, after)
            || after.bytes != opening.record.bytes
            || document->GetAvailableUndos() != undosBeforeApply
            || document->HasOpenCommand()) return;
        bits |= 16;
        measured.values[0] = std::uint64_t(outcome);
        measured.values[1] = changed.persisted.ownerState.definitionRevision;
        measured.values[2] = std::uint64_t(
            document->GetAvailableUndos() - undosBeforeApply);
        measured.values[3] = after.bytes == opening.record.bytes ? 1 : 0;
    })) return 0;
    readbacks = measured;
    return bits;
}

// Scenario 2: a real C1 owner capture/prepare/apply edits the array's path in
// one command. PrepareDependentReplay is additionally driven directly so the
// plan contents are measured, and VerifyDependentReplayAfterCommit re-proves
// the replayed record from the post-commit document.
std::uint64_t RunD3DependentReplayProbe(D3ProbeReadbacks& readbacks)
    noexcept {
    D3CollaboratorProbeFixture fixture;
    if (!fixture.initialize(0.001, false, false)) return 0;
    __block std::uint64_t bits = 0;
    __block D3ProbeReadbacks measured;
    D3CollaboratorProbeFixture *fixturePointer = &fixture;
    if (!fixture.perform(^{
        OcctDocument& owner = *fixturePointer->document;
        const Handle(TDocStd_Document) document = owner.Document();
        const std::string c1Entity = Text(
            fixturePointer->currentPath.persisted.ownerState.owner.entity);
        const auto context = fixturePointer->context(c1Entity);
        if (!context) return;
        c1_owner::OcafOwner c1(owner, context);
        const c1_owner::SceneFence scene{
            context->openingFence().documentGeneration(),
            context->openingFence().modelRevision(),
            context->openingFence().metersPerUnit()};
        const auto opening = c1.capture(c1Entity, scene);
        if (!opening || opening->retained.definition.controlPoints.empty()
            || !(opening->retained.authority.owner
                == fixturePointer->currentPath.persisted.ownerState.owner))
            return;
        bits |= 1;
        c1_owner::Candidate candidate;
        candidate.proposal.kind = bounded_curve::EditKind::MovePole;
        candidate.proposal.expected = opening->retained.authority;
        candidate.proposal.controlPoint =
            opening->retained.definition.controlPoints.front().identifier;
        candidate.proposal.replacementLocal =
            opening->retained.definition.controlPoints.front().local;
        candidate.proposal.replacementLocal[0] += 0.5;
        candidate.completeDefinition = opening->retained.definition;
        candidate.completeDefinition.controlPoints.front().local =
            candidate.proposal.replacementLocal;
        c1_owner::Receipt preparedReceipt;
        const auto prepared = c1.prepare(opening, candidate, preparedReceipt);
        if (!prepared
            || preparedReceipt.outcome != c1_owner::Outcome::prepared) return;
        DependentReplayPlan plan;
        if (!PrepareDependentReplay(owner, context, prepared, plan)
            || !plan.admitted || plan.c1Seal != prepared
            || plan.arrays.size() != 1 || !plan.arrays.front().native
            || plan.arrays.front().candidate.members.size() != 2) return;
        bits |= 2;
        const int undosBefore = document->GetAvailableUndos();
        const auto applied = c1.apply(prepared);
        if (applied.outcome != c1_owner::Outcome::committed
            || applied.historyDelta != 1
            || document->GetAvailableUndos() != undosBefore + 1
            || document->HasOpenCommand()) return;
        bits |= 4;
        OcctBoundedCurveCapture edited;
        std::vector<path_array::Record> arrays;
        if (!owner.ReadBoundedCurveExact(
                fixturePointer->currentPath.persisted.ownerState.owner, edited)
            || edited.persisted.ownerState.definitionRevision
                != fixturePointer->currentPath.persisted.ownerState
                    .definitionRevision + 1
            || !path_array::ReadAll(document, arrays) || arrays.size() != 1
            || !(arrays.front().definition.path.owner
                == edited.persisted.ownerState.owner)
            || arrays.front().definition.path.definitionRevision
                != edited.persisted.ownerState.definitionRevision
            || arrays.front().definition.path.canonicalDefinitionDigest
                != edited.persisted.ownerState.canonicalDefinitionDigest
            || arrays.front().definition.members.size() != 2
            || arrays.front().bytes == fixturePointer->arrayRecord.bytes)
            return;
        bits |= 8;
        if (!VerifyDependentReplayAfterCommit(owner, plan)
            || document->HasOpenCommand()) return;
        bits |= 16;
        measured.values[0] = std::uint64_t(applied.historyDelta);
        measured.values[1] = std::uint64_t(plan.arrays.size());
        measured.values[2] = edited.persisted.ownerState.definitionRevision;
        measured.values[3] = std::uint64_t(
            arrays.front().definition.members.size());
    })) return 0;
    readbacks = measured;
    return bits;
}

// Measured pre-save D3 state for the cold-reopen comparison.
struct D3ProbePreSaveState final {
    std::vector<std::uint8_t> recordBytes;
    std::vector<pattern::Member> members;
    std::vector<std::string> recipeIdentifiers;
    std::vector<std::vector<double>> recipeValues;
};

// Scenario 3, one unit system: grow the recipe-carrying array 2 -> 3 with
// ordinal 2 suppressed through the production CaptureNative/PrepareNative/
// ApplyNative path, save the real document, cold-reopen it through the
// production import, and re-derive every member ordinal/localID, suppression
// state, cloned recipe and record byte from the reopened document.
std::uint64_t RunD3ColdReopenUnitProbe(double unit, D3ProbePreSaveState* saved,
                                       std::string* savedPath,
                                       std::uint64_t unitReadbacks[4])
    noexcept {
    std::uint64_t bits = 0;
    D3CollaboratorProbeFixture writer;
    if (!writer.initialize(unit, false, true)) return bits;
    __block bool grown = false;
    __block bool savedOk = false;
    D3CollaboratorProbeFixture *writerPointer = &writer;
    if (!writer.perform(^{
        OcctDocument& owner = *writerPointer->document;
        const Handle(TDocStd_Document) document = owner.Document();
        const auto context = writerPointer->context(
            Text(writerPointer->source.entity));
        if (!context) return;
        Snapshot opening;
        if (CaptureNative(owner, Text(writerPointer->source.entity), context,
                opening) != Refusal::None || !opening.admitted()) return;
        Edit edit = RetainedEdit(opening.record.definition);
        edit.count = 3;
        edit.suppressedOrdinals.insert(2);
        const PreparedEdit prepared =
            PrepareNative(owner, opening, edit, Limits{});
        if (!prepared.admitted() || !prepared.native
            || prepared.candidate.members.size() != 3
            || prepared.candidate.members[2].state
                != pattern::MemberState::Suppressed) return;
        if (ApplyNative(owner, prepared, context) != ApplyOutcome::Committed
            || document->HasOpenCommand()) return;
        std::vector<path_array::Record> records;
        if (!path_array::ReadAll(document, records) || records.size() != 1
            || records.front().definition.members.size() != 3) return;
        const auto& members = records.front().definition.members;
        for (std::size_t ordinal = 0; ordinal < members.size(); ++ordinal) {
            if (members[ordinal].localID != ordinal + 1
                || members[ordinal].coordinate.row != 0
                || members[ordinal].coordinate.column != ordinal) return;
        }
        if (members[0].state != pattern::MemberState::Active
            || members[1].state != pattern::MemberState::Active
            || members[2].state != pattern::MemberState::Suppressed) return;
        OcafD3Authority authority;
        path_array::Record selectedRecord;
        std::shared_ptr<const pattern_owner::AllLabelSnapshot> snapshot;
        SourceMetrics metrics;
        if (authority.captureCurrent(owner, records,
                Text(writerPointer->source.entity), selectedRecord, snapshot,
                metrics) != Refusal::None || !snapshot
            || snapshot->members.size() != 3) return;
        saved->recipeIdentifiers.clear();
        saved->recipeValues.clear();
        for (const auto& member : snapshot->members) {
            if (member.recipe.family != pattern_recipe_clone::Family::Sweep
                || member.recipe.sweep.identifier.empty()) return;
            std::vector<double> values;
            if (!sweep_persistence::Encode(member.recipe.sweep.definition,
                    values)) return;
            saved->recipeIdentifiers.push_back(member.recipe.sweep.identifier);
            saved->recipeValues.push_back(std::move(values));
        }
        if (saved->recipeIdentifiers[0] == saved->recipeIdentifiers[1]
            || saved->recipeIdentifiers[0] == saved->recipeIdentifiers[2]
            || saved->recipeIdentifiers[1] == saved->recipeIdentifiers[2])
            return;
        saved->recordBytes = records.front().bytes;
        saved->members = members;
        grown = true;
        NSString *filename = [NSString stringWithFormat:
            @"r179-d3-collaborator-%@.cbf", NSUUID.UUID.UUIDString];
        NSString *temporary = [NSTemporaryDirectory()
            stringByAppendingPathComponent:filename];
        const std::string written = owner.save(temporary.UTF8String);
        if (written.empty()) return;
        NSDictionary *attributes = [[NSFileManager defaultManager]
            attributesOfItemAtPath:
                [NSString stringWithUTF8String:written.c_str()] error:nil];
        if (attributes == nil
            || [attributes[NSFileSize] unsignedLongLongValue] == 0) return;
        *savedPath = written;
        savedOk = true;
    })) return bits;
    if (!grown) return bits;
    bits |= 1;
    if (!savedOk) return bits;
    bits |= 2;
    // End the writer session outside its active framebuffer block, exactly
    // like the proven bounded-curve cold probe ordering.
    writer.shutdown();
    if (!writer.cleanupSucceeded || !writer.document.IsNull()) return bits;

    D3CollaboratorProbeFixture reader;
    if (!reader.initialize(unit, false, false)) return bits;
    __block bool reopened = false;
    __block bool identical = false;
    __block bool recipesEqual = false;
    __block std::uint64_t reopenedMembers = 0;
    __block std::uint64_t reopenedMaxLocalID = 0;
    __block std::uint64_t reopenedSweepRecipes = 0;
    D3CollaboratorProbeFixture *readerPointer = &reader;
    if (!reader.perform(^{
        const auto initialDocument = readerPointer->document->Document();
        if (readerPointer->viewer.ImportCbf(*savedPath)
                != core3d::AssetImportResult::Success) return;
        readerPointer->document = readerPointer->viewer.getDocument();
        if (readerPointer->document.IsNull()
            || readerPointer->document->Document().IsNull()
            || readerPointer->document->Document().get()
                == initialDocument.get()) return;
        const Handle(TDocStd_Document) document =
            readerPointer->document->Document();
        double reopenedUnit = 0;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document, reopenedUnit)
            || reopenedUnit != unit) return;
        reopened = true;
        OcctDocument& owner = *readerPointer->document;
        std::vector<path_array::Record> records;
        if (!path_array::ReadAll(document, records) || records.size() != 1
            || records.front().bytes != saved->recordBytes) return;
        const auto& members = records.front().definition.members;
        if (members.size() != saved->members.size()) return;
        for (std::size_t ordinal = 0; ordinal < members.size(); ++ordinal) {
            const auto& before = saved->members[ordinal];
            const auto& after = members[ordinal];
            if (!(after.identity == before.identity)
                || after.localID != before.localID
                || after.coordinate.row != before.coordinate.row
                || after.coordinate.column != before.coordinate.column
                || after.state != before.state) return;
        }
        identical = true;
        reopenedMembers = members.size();
        for (const auto& member : members)
            reopenedMaxLocalID = std::max<std::uint64_t>(reopenedMaxLocalID,
                member.localID);
        OcafD3Authority authority;
        path_array::Record selectedRecord;
        std::shared_ptr<const pattern_owner::AllLabelSnapshot> snapshot;
        SourceMetrics metrics;
        if (authority.captureCurrent(owner, records,
                Text(saved->members.front().identity), selectedRecord,
                snapshot, metrics) != Refusal::None || !snapshot
            || snapshot->members.size() != saved->members.size()) return;
        for (std::size_t ordinal = 0; ordinal < snapshot->members.size();
             ++ordinal) {
            const auto& member = snapshot->members[ordinal];
            if (member.recipe.family != pattern_recipe_clone::Family::Sweep
                || member.recipe.sweep.identifier
                    != saved->recipeIdentifiers[ordinal]) return;
            std::vector<double> values;
            if (!sweep_persistence::Encode(member.recipe.sweep.definition,
                    values)
                || values != saved->recipeValues[ordinal]) return;
            const auto* key =
                std::get_if<pattern_owner::D3Ordinal>(&member.key);
            if (!key || key->ordinal != ordinal
                || member.localIdentifier != saved->members[ordinal].localID
                || member.suppressed != (saved->members[ordinal].state
                    == pattern::MemberState::Suppressed)) return;
            ++reopenedSweepRecipes;
        }
        recipesEqual = true;
    })) return bits;
    if (!reopened) return bits;
    bits |= 4;
    if (!identical) return bits;
    bits |= 8;
    if (!recipesEqual) return bits;
    bits |= 16;
    unitReadbacks[0] = reopenedMembers;
    unitReadbacks[1] = 1;
    unitReadbacks[2] = reopenedMaxLocalID;
    unitReadbacks[3] = reopenedSweepRecipes;
    return bits;
}

std::uint64_t RunD3ColdReopenProbe(D3ProbeReadbacks& readbacks) noexcept {
    const std::array<double, 2> units{{0.001, 1.0}};
    std::uint64_t perUnitBits[2] = {0, 0};
    std::uint64_t perUnitReadbacks[2][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}};
    for (std::size_t index = 0; index < units.size(); ++index) {
        D3ProbePreSaveState saved;
        std::string savedPath;
        perUnitBits[index] = RunD3ColdReopenUnitProbe(units[index], &saved,
            &savedPath, perUnitReadbacks[index]);
    }
    const std::uint64_t bits = perUnitBits[0] & perUnitBits[1];
    if (bits == 0x1f)
        for (int value = 0; value < 4; ++value)
            readbacks.values[value] = std::min(perUnitReadbacks[0][value],
                perUnitReadbacks[1][value]);
    return bits;
}
} // namespace

extern "C" std::uint64_t
Core3DDebugPathArrayNativeCollaboratorsProbe(std::int32_t scenario) noexcept {
    try {
        if (scenario == 4) return RunD3AdmissionProbe();
        if (![NSThread isMainThread] || scenario < 0 || scenario > 3) return 0;
        auto& readbacks = D3ProbeStoredReadbacks(scenario);
        readbacks = {};
        std::uint64_t bits = 0;
        if (scenario == 0) bits = RunD3ProductionResolverProbe(readbacks);
        else if (scenario == 1) bits = RunD3ReplacementChangedProbe(readbacks);
        else if (scenario == 2) bits = RunD3DependentReplayProbe(readbacks);
        else bits = RunD3ColdReopenProbe(readbacks);
        readbacks.complete = bits == 0x1f;
        return bits;
    } catch (...) {}
    return 0;
}

// Companion readbacks measured from the same executed scenario runs (receipt
// revision and issuance, the measured apply refusal, history deltas, replayed
// array cardinality, reopened member/localID/recipe counts). Returns ~0 when
// the scenario has not run to completion in this process.
extern "C" std::uint64_t
Core3DDebugPathArrayNativeCollaboratorsReadback(std::int32_t scenario,
                                                std::int32_t index) noexcept {
    if (![NSThread isMainThread] || scenario < 0 || scenario > 3
        || index < 0 || index > 3) return ~0ull;
    const auto& readbacks = D3ProbeStoredReadbacks(scenario);
    return readbacks.complete ? readbacks.values[index] : ~0ull;
}

ApplyOutcome ApplyReplacementPathForDebugProbe(
    OcctDocument& owner, const Snapshot& opening,
    const path_array::CurveReference& replacement, const Edit& edit,
    const std::shared_ptr<native_opening::Context>& context) noexcept {
    try {
        if (!context) return ApplyOutcome::Refused;
        LivePathResolver paths(owner, context);
        PathAuthority authority;
        if (paths.resolveCurrent(replacement, authority) != Refusal::None
            || !authority.currentFor(replacement))
            return ApplyOutcome::Refused;
        Edit withReplacement = edit;
        withReplacement.replacementPath = authority;
        const PreparedEdit prepared =
            PrepareNative(owner, opening, withReplacement, {});
        if (!prepared.admitted()) return ApplyOutcome::Refused;
        return ApplyNative(owner, prepared, context);
    } catch (...) { return ApplyOutcome::Refused; }
}
#endif
} // namespace core3d::path_array_owner

namespace core3d::dependent_replay {
Refusal PreparePathArrayD3Replay(OcctDocument& owner,
    const Dependency& dependency, Mutation mutation, const Candidate& candidate,
    const std::shared_ptr<native_opening::Context>& context,
    std::shared_ptr<const PreparedReplay>& output) noexcept {
    return path_array_owner::PreparePathArrayD3ReplayImpl(
        owner, dependency, mutation, candidate, context, output);
}
}
