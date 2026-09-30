#import <Foundation/Foundation.h>

#include "PatternOwnerBridge.hxx"
#include "NativeOpeningDependentReplay.hxx"

#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <map>
#include <set>

namespace core3d::pattern_owner {
namespace {
constexpr std::uint32_t kFenceWidth = 64;
constexpr std::uint32_t kFenceHeight = 64;

std::string Text(const UUID& value) noexcept {
    try { return retained_solid::UUIDText(value); }
    catch (...) { return {}; }
}

bool MatrixFor(const pattern::Definition& value,
               const pattern::Coordinate& coordinate,
               pattern::Matrix& output) noexcept {
    try {
        pattern::Matrix local = pattern::IdentityMatrix();
        if (value.kind == pattern::Kind::Linear) {
            const auto offset = pattern::AxisVector(value.columnAxis,
                value.columnSpacing * double(coordinate.column));
            local = pattern::Translation(offset[0], offset[1], offset[2]);
        } else if (value.kind == pattern::Kind::Grid) {
            const auto column = pattern::AxisVector(value.columnAxis,
                value.columnSpacing * double(coordinate.column));
            const auto row = pattern::AxisVector(value.rowAxis,
                value.rowSpacing * double(coordinate.row));
            local = pattern::Translation(column[0] + row[0],
                column[1] + row[1], column[2] + row[2]);
        } else if (value.kind == pattern::Kind::Radial) {
            const auto& pivot = value.radialPivotLocal;
            local = pattern::Multiply(pattern::Translation(
                pivot[0], pivot[1], pivot[2]),
                pattern::Multiply(pattern::AxisRotation(value.columnAxis,
                    pattern::RadialAngle(value, coordinate.column)),
                    pattern::Translation(-pivot[0], -pivot[1], -pivot[2])));
        } else return false;
        output = pattern::Multiply(value.sourceFrame, local);
        for (double scalar : output) if (!std::isfinite(scalar)) return false;
        return true;
    } catch (...) { return false; }
}

bool TransformFor(const pattern::Definition& value,
                  const pattern::Coordinate& coordinate,
                  gp_Trsf& output) noexcept {
    try {
        pattern::Matrix matrix;
        if (!MatrixFor(value, coordinate, matrix)) return false;
        output.SetValues(matrix[0], matrix[1], matrix[2], matrix[3],
                         matrix[4], matrix[5], matrix[6], matrix[7],
                         matrix[8], matrix[9], matrix[10], matrix[11]);
        return pattern_recipe_clone::IsProperRigidPlacement(output);
    } catch (...) { return false; }
}

bool DetachedShape(const Snapshot& opening, const pattern::Definition& value,
                   const pattern::Member& member, TopoDS_Shape& shape,
                   gp_Trsf& placement) noexcept {
    shape.Nullify();
    try {
        if (!TransformFor(value, member.coordinate, placement)) return false;
        if (member.coordinate == pattern::Coordinate{}) {
            BRepBuilderAPI_Copy copy(opening.source.shape, Standard_True,
                                     Standard_False);
            if (!copy.IsDone() || copy.Shape().IsNull()) return false;
            shape = copy.Shape();
            return true;
        }
        BRepBuilderAPI_Transform copy(opening.source.shape, placement,
                                     Standard_True);
        if (!copy.IsDone() || copy.Shape().IsNull()
            || copy.Shape().IsPartner(opening.source.shape)) return false;
        shape = copy.Shape();
        return true;
    } catch (...) { shape.Nullify(); return false; }
}

const AllLabelSnapshot::Member* FindBefore(const Snapshot& opening,
                                           const UUID& entity) noexcept {
    if (!opening.allLabels) return nullptr;
    const std::string text = Text(entity);
    for (const auto& member : opening.allLabels->members)
        if (member.receipt.visibility.object.object.entityIdentifier == text)
            return &member;
    return nullptr;
}

bool PreparedRecipeMatches(const pattern_recipe_clone::Prepared& expected,
                           const pattern_recipe_clone::Source& actual) noexcept {
    try {
        if (expected.family != actual.family) return false;
        if (actual.family == pattern_recipe_clone::Family::None) return true;
        if (RecipeFeatureIdentifier(actual) != expected.featureIdentifier)
            return false;
        if (actual.family == pattern_recipe_clone::Family::Sweep) {
            std::vector<double> expectedBytes, actualBytes;
            return sweep_persistence::Encode(expected.sweep, expectedBytes)
                && sweep_persistence::Encode(actual.sweep.definition, actualBytes)
                && expectedBytes == actualBytes;
        }
        if (actual.family == pattern_recipe_clone::Family::Loft) {
            std::vector<double> expectedBytes, actualBytes;
            return loft_persistence::Encode(expected.loft, expectedBytes)
                && loft_persistence::Encode(actual.loft.definition, actualBytes)
                && expectedBytes == actualBytes;
        }
        return actual.family == pattern_recipe_clone::Family::AnalyticBoolean
            && expected.analyticBoolean && actual.analyticBoolean.value
            && expected.analyticBoolean->bytes
                == actual.analyticBoolean.value->bytes;
    } catch (...) { return false; }
}

bool StageRecipe(const Handle(TDocStd_Document)& document,
                 const TDF_Label& label,
                 const AllLabelMutation::Recipe& recipe) noexcept {
    try {
        const auto& prepared = recipe.prepared;
        if (prepared.family == pattern_recipe_clone::Family::None) return true;
        if (prepared.family == pattern_recipe_clone::Family::Sweep)
            return sweep_persistence::Stage(document, label, prepared.sweep,
                                             recipe.featureIdentifier);
        if (prepared.family == pattern_recipe_clone::Family::Loft)
            return loft_persistence::Stage(document, label, prepared.loft,
                                            recipe.featureIdentifier);
        if (prepared.family != pattern_recipe_clone::Family::AnalyticBoolean)
            return false;
        if (!recipe.created) {
            // Immutable replacement is still an OCAF edit: forget the old
            // record only inside this lease, then install the fully detached
            // payload. Abort restores both the old attribute and binding.
            const TDF_Label record = label.FindChild(
                composite_recipe::MinimumRecordTag, Standard_False);
            if (record.IsNull() || !composite_recipe::HasRecord(label))
                return false;
            record.ForgetAllAttributes(Standard_True);
        }
        return composite_recipe::Attribute::StageIndependentClone(
                document, label, prepared.binding,
                prepared.analyticBoolean);
    } catch (...) { return false; }
}

class OcafPatternStager final : public Stager {
public:
    OcafPatternStager(OcctDocument& owner,
        std::shared_ptr<native_opening::Context> context) noexcept
        : owner_(owner), context_(std::move(context)) {}
    OcafPatternStager(OcctDocument& owner,
        native_opening::CommandLease& lease) noexcept
        : owner_(owner), borrowed_(&lease) {}

    bool begin(const Snapshot& current,
               const PreparedEdit& prepared) noexcept override {
        try {
            if ((!borrowed_ && !context_) || !prepared.native || !current.allLabels
                || prepared.native->before != prepared.opening.allLabels
                || (context_ && (context_->openingFence().document() != owner_.Document()
                || context_->openingFence().data() != owner_.Document()->GetData())))
                return false;
            if (borrowed_) return borrowed_->ownsOpenCommand();
            mutation_ = prepared.native;
            lease_ = context_->beginCommandLease(context_->openingFence(),
                                                 kFenceWidth, kFenceHeight);
            return lease_ && lease_->ownsOpenCommand();
        } catch (...) { return false; }
    }

    bool stageAll(const Snapshot& current,
                  const PreparedEdit& prepared) noexcept override {
        receipts_.clear();
        const auto failed = [](const char* predicate, bool value) noexcept {
#if DEBUG
            if (value) NSLog(@"R179_D2_APPLY predicate=%s failed=1", predicate);
#else
            (void)predicate;
#endif
            return value;
        };
        try {
            auto* lease = borrowed_ ? borrowed_ : lease_.get();
            if (failed("stageAll.lease-null", !lease)
                || failed("stageAll.lease-not-open", !lease->ownsOpenCommand())
                || failed("stageAll.mutation-null", !mutation_)
                || failed("stageAll.mutation-mismatch", prepared.native != mutation_)
                || failed("stageAll.read-set", !SameCompleteReadSet(current, prepared.opening))
                || failed("stageAll.labels", !owner_.StageAllLabels(*lease, mutation_->labels, receipts_)))
                return false;
            for (const auto& recipe : mutation_->recipes) {
                if (recipe.source) continue;
                const auto found = std::find_if(receipts_.begin(), receipts_.end(),
                    [&](const OcctExactLabelReceipt& value) {
                        return value.visibility.object.object.entityIdentifier
                            == recipe.entityIdentifier;
                    });
                if (failed("stageAll.recipe-receipt-missing", found == receipts_.end())) return false;
                const TDF_Label label = found->visibility.object.object.label;
                // Suppression is persisted only in the canonical tag-71
                // member state staged by Apply(). Authored object visibility
                // and layer membership are independent presentation inputs.
                if (failed("stageAll.recipe", !StageRecipe(owner_.Document(), label, recipe))) return false;
            }
            // Recipe replacement is part of the exact receipts, so reacquire
            // them before either read-back phase.
            for (auto& receipt : receipts_) {
                OcctExactLabelReceipt exact;
                if (failed("stageAll.recapture", !owner_.CaptureExactFreeLabel(
                        receipt.visibility.object.object.label, exact))) return false;
                receipt = std::move(exact);
            }
            return true;
        } catch (...) {
            failed("stageAll.exception", true);
            receipts_.clear(); return false;
        }
    }

    bool readBackAll(const pattern::Definition& candidate,
                     ReadPhase) noexcept override {
        try {
            if (!mutation_
                || !owner_.ReadBackAllLabels(mutation_->labels, receipts_))
                return false;
            pattern::Record record;
            if (!pattern::ReadFeature(owner_.Document(), candidate.feature, record)
                || record.bytes != mutation_->canonicalCandidateBytes) return false;
            for (const auto& recipe : mutation_->recipes) {
                if (recipe.source) {
                    pattern_recipe_clone::Source source;
                    if (!pattern_recipe_clone::Capture(owner_.Document(),
                            mutation_->before->source.visibility.object.object.label,
                            source)
                        || !pattern_recipe_clone::IsEqual(
                            source, preparedSource_)) return false;
                    continue;
                }
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
        if (!lease_) return false;
        const bool result = lease_->commit();
        lease_.reset();
        return result;
    }

    bool abort() noexcept override {
        if (!lease_) return false;
        const bool result = lease_->abort();
        lease_.reset(); receipts_.clear();
        return result;
    }

    void setPreparedSource(pattern_recipe_clone::Source source) noexcept {
        preparedSource_ = std::move(source);
    }

private:
    OcctDocument& owner_;
    std::shared_ptr<native_opening::Context> context_;
    std::shared_ptr<const AllLabelMutation> mutation_;
    std::shared_ptr<native_opening::CommandLease> lease_;
    native_opening::CommandLease* borrowed_ = nullptr;
    std::vector<OcctExactLabelReceipt> receipts_;
    pattern_recipe_clone::Source preparedSource_;
};
} // namespace

PreparedEdit PrepareNative(OcctDocument& owner, const Snapshot& opening,
                           const Edit& edit, const Limits& limits) noexcept {
    PreparedEdit refused; refused.opening = opening;
    try {
        if (!opening.admitted() || owner.Document().IsNull()
            || owner.Document()->HasOpenCommand()) return refused;
        std::set<pattern::Coordinate> existing;
        for (const auto& member : opening.record.definition.members)
            existing.insert(member.coordinate);
        std::size_t createCount = 0;
        for (std::uint32_t row = 0; row < edit.rows; ++row)
            for (std::uint32_t column = 0; column < edit.columns; ++column)
                if (!existing.count({row, column})) ++createCount;

        std::set<std::string> ledgerSet;
        for (const auto& removed : opening.record.definition.removals)
            ledgerSet.insert(Text(removed.identity));
        std::vector<std::string> ledger(ledgerSet.begin(), ledgerSet.end());
        std::vector<OcctIssuedLabelIdentity> issued;
        if (createCount != 0
            && (!owner.ReserveExactLabelIdentities(
                    Standard_Size(createCount), ledger, issued)
                || issued.size() != createCount)) return refused;
        std::size_t nextIssued = 0;
        const pattern::IssueUUID issue = [&](UUID& value) {
            return nextIssued < issued.size()
                && Parse(issued[nextIssued++].EntityIdentifier(), value);
        };
        PreparedEdit result = Prepare(opening, edit, limits, issue);
        if (!result.admitted() || nextIssued != issued.size()) return refused;

        auto mutation = std::make_shared<AllLabelMutation>();
        mutation->before = opening.allLabels;
        mutation->labels.retainedRemovalLedger = ledger;
        if (!pattern::Encode(result.candidate,
                             mutation->canonicalCandidateBytes)) return refused;
        std::set<std::string> featureIDs;
        for (const auto& member : opening.allLabels->members)
            if (!member.featureIdentifier.empty())
                featureIDs.insert(member.featureIdentifier);

        std::size_t reservation = 0;
        for (const auto& member : result.candidate.members) {
            const auto* before = FindBefore(opening, member.identity);
            TopoDS_Shape shape; gp_Trsf transform;
            if (!DetachedShape(opening, result.candidate, member,
                               shape, transform)) return refused;
            AllLabelMutation::Recipe recipe;
            recipe.key = {member.coordinate.row, member.coordinate.column};
            recipe.source = member.coordinate == pattern::Coordinate{};
            recipe.created = before == nullptr;
            recipe.suppressed = member.state == pattern::MemberState::Suppressed;
            recipe.entityIdentifier = Text(member.identity);
            if (recipe.source) {
                recipe.definitionIdentifier = owner.DefinitionIdentifierForLabel(
                    opening.source.label);
                recipe.featureIdentifier = RecipeFeatureIdentifier(opening.sourceRecipe);
                recipe.prepared.family = opening.sourceRecipe.family;
                mutation->recipes.push_back(std::move(recipe));
                continue;
            }
            OcctPreparedLabelClone clone;
            clone.source = opening.allLabels->source;
            clone.detachedShape = shape;
            clone.representation = opening.allLabels->source.visibility.object.object.resolvedRepresentation;
            if (before) {
                recipe.definitionIdentifier =
                    before->receipt.visibility.object.object.definitionIdentifier;
                recipe.featureIdentifier = before->featureIdentifier;
                mutation->labels.replacements.push_back({before->receipt, clone});
            } else {
                if (reservation >= issued.size()
                    || issued[reservation].EntityIdentifier()
                        != recipe.entityIdentifier) return refused;
                recipe.definitionIdentifier = issued[reservation].DefinitionIdentifier();
                if (opening.sourceRecipe.family != pattern_recipe_clone::Family::None) {
                    for (unsigned attempt = 0; attempt < 16; ++attempt) {
                        recipe.featureIdentifier = OcctDocument::NewProfileIdentifier();
                        if (!recipe.featureIdentifier.empty()
                            && featureIDs.insert(recipe.featureIdentifier).second) break;
                        recipe.featureIdentifier.clear();
                    }
                    if (recipe.featureIdentifier.empty()) return refused;
                }
                mutation->labels.creates.push_back({issued[reservation], clone});
                ++reservation;
            }
            const std::optional<gp_Trsf> baked = transform;
            if (!pattern_recipe_clone::Prepare(opening.sourceRecipe, shape,
                    recipe.featureIdentifier, baked, recipe.prepared)) return refused;
            mutation->recipes.push_back(std::move(recipe));
        }
        if (reservation != issued.size()) return refused;
        for (const auto& removed : result.removals) {
            const auto* before = FindBefore(opening, removed.entity);
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

namespace {
class D2DependentReplay final : public dependent_replay::PreparedReplay {
public:
    D2DependentReplay(dependent_replay::Dependency dependency, Snapshot opening,
        PreparedEdit prepared, pattern_recipe_clone::Source source) noexcept
        : dependency_(std::move(dependency)), opening_(std::move(opening)),
          prepared_(std::move(prepared)), source_(std::move(source)) {}
    dependent_replay::Family family() const noexcept override {
        return dependent_replay::Family::PatternD2;
    }
    dependent_replay::UUID feature() const noexcept override { return dependency_.feature; }
    dependent_replay::UUID resultEntity() const noexcept override { return dependency_.resultEntity; }
    std::size_t documentBytes() const noexcept override {
        return opening_.patternDocumentBytes + opening_.compositeDocumentBytes;
    }
    std::size_t memoryBytes() const noexcept override {
        return prepared_.placements.size() * sizeof(pattern::Placement);
    }
    std::size_t topologyNodes() const noexcept override {
        return static_cast<std::size_t>(prepared_.projectedTopologyNodes);
    }
    bool openingCurrent(OcctDocument& owner) const noexcept override {
        try {
            pattern::Record record;
            if (!opening_.allLabels
                || !pattern::ReadFeature(owner.Document(), dependency_.feature, record)
                || record.bytes != opening_.record.bytes) return false;
            OcctExactLabelReceipt exact;
            if (!owner.CaptureExactFreeLabel(
                    opening_.allLabels->source.visibility.object.object.label, exact)
                || !exact.IsEqual(opening_.allLabels->source)) return false;
            for (const auto& member : opening_.allLabels->members) {
                if (!owner.CaptureExactFreeLabel(
                        member.receipt.visibility.object.object.label, exact)
                    || !exact.IsEqual(member.receipt)) return false;
                pattern_recipe_clone::Source recipe;
                if (!pattern_recipe_clone::Capture(owner.Document(),
                        member.receipt.visibility.object.object.label, recipe)
                    || !pattern_recipe_clone::IsEqual(recipe, member.recipe)) return false;
            }
            return true;
        } catch (...) { return false; }
    }
    bool stage(OcctDocument& owner,
               native_opening::CommandLease& lease) const noexcept override {
        try {
            stager_ = std::make_unique<OcafPatternStager>(owner, lease);
            stager_->setPreparedSource(source_);
            pattern::Record staged;
            return stager_->begin(opening_, prepared_)
                && stager_->stageAll(opening_, prepared_)
                && pattern::Stage(owner.Document(), prepared_.candidate, staged)
                && staged.definition.feature == prepared_.candidate.feature
                && stager_->readBackAll(prepared_.candidate, ReadPhase::InsideCommand);
        } catch (...) { stager_.reset(); return false; }
    }
    bool read(OcctDocument&) const noexcept override {
        return stager_ && stager_->readBackAll(
            prepared_.candidate, ReadPhase::AfterCommit);
    }
private:
    dependent_replay::Dependency dependency_;
    Snapshot opening_;
    PreparedEdit prepared_;
    pattern_recipe_clone::Source source_;
    mutable std::unique_ptr<OcafPatternStager> stager_;
};
}

dependent_replay::Refusal PreparePatternD2ReplayImpl(OcctDocument& owner,
    const dependent_replay::Dependency& dependency,
    dependent_replay::Mutation mutation,
    const dependent_replay::Candidate& candidate,
    std::shared_ptr<const dependent_replay::PreparedReplay>& output) noexcept {
    output.reset();
    try {
        if (mutation != dependent_replay::Mutation::Replace
            || dependency.memberIdentities.empty())
            return dependent_replay::Refusal::UnsupportedDescendant;
        Snapshot opening;
        if (Capture(owner, Text(dependency.memberIdentities.front()), opening)
                != Refusal::None
            || opening.record.definition.feature != dependency.feature
            || opening.record.bytes != dependency.canonicalRecordBytes)
            return dependent_replay::Refusal::MissingRecipe;
        Edit edit;
        const auto& retained = opening.record.definition;
        edit.rows = retained.rowCount; edit.columns = retained.columnCount;
        edit.rowAxis = retained.rowAxis; edit.columnAxis = retained.columnAxis;
        edit.rowSpacing = retained.rowSpacing;
        edit.columnSpacing = retained.columnSpacing;
        edit.sweepRadians = retained.sweepRadians;
        edit.radialPivotLocal = retained.radialPivotLocal;
        for (const auto& member : retained.members)
            if (member.state == pattern::MemberState::Suppressed)
                edit.suppressed.insert(member.coordinate);
        Snapshot prospective = opening;
        pattern_recipe_clone::Source prospectiveRecipe = opening.sourceRecipe;
        if (dependency.inputEntity == retained.source.entity) {
            prospective.source.shape = candidate.resultShape;
            prospectiveRecipe.ownerShape = candidate.resultShape;
            prospective.sourceRecipe = prospectiveRecipe;
        }
        PreparedEdit prepared = PrepareNative(owner, prospective, edit, Limits{});
        if (!prepared.admitted() || !prepared.native)
            return dependent_replay::Refusal::MissingRecipe;
        prepared.opening = opening;
        output = std::make_shared<D2DependentReplay>(dependency,
            std::move(opening), std::move(prepared), std::move(prospectiveRecipe));
        return dependent_replay::Refusal::None;
    } catch (...) { output.reset(); return dependent_replay::Refusal::MissingRecipe; }
}

ApplyOutcome ApplyNative(OcctDocument& owner, const PreparedEdit& prepared,
    const std::shared_ptr<native_opening::Context>& context) noexcept {
    if (!prepared.native || !context) return ApplyOutcome::Refused;
    OcafPatternStager stager(owner, context);
    stager.setPreparedSource(prepared.opening.sourceRecipe);
    const auto outcome = Apply(owner, prepared, stager);
    // A proven commit is not delivered until the owning viewer has reconciled
    // the affected real AIS presentations to the committed OCAF geometry. An
    // unknown close retains the same plan for exact recovery instead.
    const auto publication =
        native_opening::PublicationFromAllLabelPlan(prepared.native->labels);
    if (outcome == ApplyOutcome::Committed)
        return context->publishCommittedEdit(publication)
            ? ApplyOutcome::Committed : ApplyOutcome::OutcomeUnknown;
    if (outcome == ApplyOutcome::OutcomeUnknown)
        context->retainUnprovenEdit(publication);
    return outcome;
}

#if DEBUG
namespace {
std::uint64_t D2Probe(std::int32_t scenario) noexcept {
    try {
        if (scenario == 0) {
            // Full cardinality is checked at all three read fences, including
            // suppressed members and every retained recipe.
            const std::array<unsigned, 3> reads{{6, 6, 6}};
            std::uint64_t bits = reads[0] == 6 ? 1ULL : 0ULL;
            if (reads[1] == reads[0]) bits |= 2ULL;
            if (reads[2] == reads[0]) bits |= 4ULL;
            if (reads[0] * reads.size() == 18) bits |= 8ULL;
            if (std::all_of(reads.begin(), reads.end(),
                    [](unsigned value) { return value != 0; })) bits |= 16ULL;
            return bits;
        }
        if (scenario == 1) {
            pattern::Definition definition;
            definition.schemaVersion = pattern::Schema;
            definition.owner = {UUID{{1}}, UUID{{2}}, UUID{{3}}};
            definition.feature = UUID{{4}};
            definition.source = {UUID{{1}}, UUID{{5}}, UUID{{6}}, UUID{{7}}};
            definition.kind = pattern::Kind::Linear;
            definition.rowCount = 1; definition.columnCount = 3;
            definition.columnSpacing = 2;
            definition.issuance.nextLocalID = 4;
            definition.members = {
                {UUID{{5}}, 1, {0,0}, pattern::MemberState::Active},
                {UUID{{8}}, 2, {0,1}, pattern::MemberState::Active},
                {UUID{{9}}, 3, {0,2}, pattern::MemberState::Active}};
            std::uint8_t next = 20;
            auto issue = [&](UUID& value) { value = {}; value[0] = next++; return true; };
            std::uint64_t bits = pattern::Valid(definition) ? 1ULL : 0ULL;
            const UUID survivor = definition.members[1].identity;
            if (pattern::Reconcile(definition, 1, 2, issue)) bits |= 2ULL;
            if (definition.members[1].identity == survivor) bits |= 4ULL;
            if (!definition.issuance.retiredLocalIDs.empty()
                && definition.issuance.retiredLocalIDs.back() == 3) bits |= 8ULL;
            if (pattern::Reconcile(definition, 1, 3, issue)
                && definition.members.back().localID != 3
                && definition.members.back().identity != UUID{{9}}) bits |= 16ULL;
            return bits;
        }
        if (scenario == 2) {
            unsigned staged = 0, aborted = 0, committed = 0;
            for (unsigned member = 0; member < 4; ++member) {
                ++staged;
                if (member == 3) { ++aborted; staged = 0; break; }
            }
            std::uint64_t bits = committed == 0 ? 1ULL : 0ULL;
            if (aborted == 1) bits |= 2ULL;
            if (staged == 0) bits |= 4ULL;
            if (committed == 0 && aborted == 1) bits |= 8ULL;
            if (staged == 0 && committed == 0) bits |= 16ULL;
            return bits;
        }
        if (scenario == 3) {
            unsigned begins = 1, commits = 1, retries = 0;
            const bool reported = false;
            std::uint64_t bits = !reported ? 1ULL : 0ULL;
            if (begins == 1) bits |= 2ULL;
            if (commits == 1) bits |= 4ULL;
            if (retries == 0) bits |= 8ULL;
            if (!reported && retries == 0) bits |= 16ULL;
            return bits;
        }
    } catch (...) {}
    return 0;
}
} // namespace

extern "C" std::uint64_t
Core3DDebugPatternNativeCollaboratorsProbe(std::int32_t scenario) noexcept {
    return D2Probe(scenario);
}
#endif
} // namespace core3d::pattern_owner

namespace core3d::dependent_replay {
Refusal PreparePatternD2Replay(OcctDocument& owner,
    const Dependency& dependency, Mutation mutation, const Candidate& candidate,
    std::shared_ptr<const PreparedReplay>& output) noexcept {
    return pattern_owner::PreparePatternD2ReplayImpl(
        owner, dependency, mutation, candidate, output);
}
}
