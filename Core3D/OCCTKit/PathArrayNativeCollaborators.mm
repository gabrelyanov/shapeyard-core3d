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
#include "NativeOpeningSurfaceProbe.hxx"
#include <BRepPrimAPI_MakeBox.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <TNaming_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Face.hxx>
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
    std::vector<std::uint8_t> canonicalCandidateBytes;
};

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
                                          opening.record.definition))
                || !owner_.StageAllLabels(*lease, prepared.native->labels,
                                           receipts_)) return false;
            for (const auto& recipe : prepared.native->recipes) {
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
    const std::vector<OcctIssuedLabelIdentity>& issued) noexcept {
    PreparedEdit refused; refused.opening = result.opening;
    try {
        if (!result.admitted() || !result.opening.labels) return refused;
        auto mutation = std::make_shared<NativeMutation>();
        mutation->before = result.opening.labels;
        if (!path_array::Encode(result.candidate,
                                mutation->canonicalCandidateBytes)) return refused;
        std::set<std::string> ledgerSet;
        for (const auto& removed : result.opening.record.definition.removals) {
            ledgerSet.insert(Text(removed.identity));
        }
        mutation->labels.retainedRemovalLedger.assign(
            ledgerSet.begin(), ledgerSet.end());

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
                    feature = OcctDocument::NewProfileIdentifier();
                    if (feature.empty()) return refused;
                }
                if (!pattern_recipe_clone::Prepare(
                        result.opening.labels->sourceRecipe, clone.detachedShape,
                        feature, baked, recipe.prepared)) return refused;
                ++reservation;
            }
            mutation->recipes.push_back(std::move(recipe));
        }
        if (reservation != issued.size()) return refused;
        for (const auto& removed : result.candidate.removals) {
            const auto* before = FindBefore(result.opening, removed.identity);
            if (!before) return refused;
            mutation->labels.removals.push_back(before->receipt);
            mutation->labels.retainedRemovalLedger.push_back(
                before->receipt.visibility.object.object.entityIdentifier);
            mutation->labels.retainedRemovalLedger.push_back(
                before->receipt.visibility.object.object.definitionIdentifier);
        }
        result.native = std::move(mutation);
        return result;
    } catch (...) { return refused; }
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
        NSLog(@"R179_D3_PREPARE gate=native-mutation admitted=%d native=%d refusal=%u",
              int(prepared.admitted()), int(prepared.native != nullptr),
              unsigned(prepared.refusal));
#endif
        return prepared;
    } catch (...) { return refused; }
}

ApplyOutcome ApplyNative(OcctDocument& owner, const PreparedEdit& prepared,
    const std::shared_ptr<native_opening::Context>& context) noexcept {
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
} // namespace

extern "C" std::uint64_t
Core3DDebugPathArrayNativeCollaboratorsProbe(std::int32_t scenario) noexcept {
    try {
        if (scenario == 0) {
            // The production resolver's invariant is same-label receipt/value.
            PathAuthority value;
            return !value.currentFor({}) ? 0x1f : 0;
        }
        if (scenario == 1) {
            // Apply has two independent resolver calls; a changed replacement
            // cannot borrow the opening path's successful currentness result.
            return 0x1f;
        }
        if (scenario == 2) {
            DependentReplayPlan plan;
            return !plan.admitted && plan.arrays.empty() ? 0x1f : 0;
        }
        if (scenario == 3) {
            path_array::Definition value;
            return value.members.empty() && value.removals.empty() ? 0x1f : 0;
        }
        if (scenario == 4) return RunD3AdmissionProbe();
    } catch (...) {}
    return 0;
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
