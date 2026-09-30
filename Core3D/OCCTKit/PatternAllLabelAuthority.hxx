#pragma once

#include "OcctDocument.h"
#include "PathArrayPersistence.hxx"
#include "PatternPersistence.hxx"
#include "PatternRecipeClone.hxx"
#include "ReceiptRecord.hxx"
#include "NativeOpeningContext.hxx"

#include <cstdint>
#include <map>
#include <memory>
#include <limits>
#include <string>
#include <variant>
#include <vector>

namespace core3d::pattern_owner {

enum class AllLabelFamily : std::uint8_t { PatternD2 = 2, PathArrayD3 = 3 };

struct D2Coordinate final {
    std::uint32_t row = 0;
    std::uint32_t column = 0;
};

struct D3Ordinal final { std::uint32_t ordinal = 0; };

using AllLabelMemberKey = std::variant<D2Coordinate, D3Ordinal>;

//! Presentation-only projection of the canonical tag-71/tag-72 member state.
//! It deliberately contains no authored XCAF visibility or layer value: those
//! remain independent inputs to EffectiveVisibility().
enum class RetainedPresentationState : std::uint8_t {
    Unretained = 0,
    Active = 1,
    Suppressed = 2,
};

struct RetainedPresentationIndex final {
    std::map<retained_recipe::UUID, RetainedPresentationState> members;

    RetainedPresentationState StateFor(
        const retained_recipe::UUID& entity) const noexcept {
        const auto found = members.find(entity);
        return found == members.end()
            ? RetainedPresentationState::Unretained : found->second;
    }
};

inline bool EffectiveVisibility(bool authoredVisible, bool layersVisible,
                                RetainedPresentationState retained) noexcept {
    return authoredVisible && layersVisible
        && retained != RetainedPresentationState::Suppressed;
}

//! Reads both accepted retained-pattern families. A contradictory identity is
//! refused rather than letting traversal order become a second suppression
//! authority. Removed members are intentionally absent: their labels are
//! governed by the transaction/removal ledger, not presentation suppression.
inline bool CaptureRetainedPresentationIndex(
    const Handle(TDocStd_Document)& document,
    RetainedPresentationIndex& output) noexcept {
    output = {};
    try {
        std::vector<pattern::Record> d2;
        std::vector<path_array::Record> d3;
        if (!pattern::ReadAll(document, d2)
            || !path_array::ReadAll(document, d3)) return false;
        RetainedPresentationIndex staged;
        const auto add = [&](const pattern::Member& member) {
            const auto state = member.state == pattern::MemberState::Suppressed
                ? RetainedPresentationState::Suppressed
                : RetainedPresentationState::Active;
            const auto [position, inserted] = staged.members.emplace(
                member.identity, state);
            return inserted || position->second == state;
        };
        for (const auto& record : d2)
            for (const auto& member : record.definition.members)
                if (!add(member)) return false;
        for (const auto& record : d3)
            for (const auto& member : record.definition.members)
                if (!add(member)) return false;
        output = std::move(staged);
        return true;
    } catch (...) { output = {}; return false; }
}

inline RetainedPresentationState StateForEntityIdentifier(
    const RetainedPresentationIndex& index,
    const std::string& entityIdentifier) noexcept {
    retained_recipe::UUID entity{};
    return receipt::ParseUUID(entityIdentifier, entity)
        ? index.StateFor(entity)
        : RetainedPresentationState::Unretained;
}

struct AllLabelMeasuredCost final {
    std::size_t recordBytes = 0;
    std::size_t recipeBytes = 0;
    std::size_t shapeBytes = 0;
    Standard_Size topologyNodes = 0;
    std::size_t retainedMemoryBytes = 0;
};

//! Exact shared carrier for the later D2/D3 owners. It deliberately retains
//! receipts rather than labels reconstructed from UUID strings. D3 ordinals
//! remain a distinct alternative and cannot acquire D2 grid semantics.
struct AllLabelSnapshot final {
    Handle(TDF_Data) documentData;
    std::string documentIdentifier;
    AllLabelFamily family = AllLabelFamily::PatternD2;
    TDF_Label recordLabel;
    std::string featureIdentifier;
    std::vector<std::uint8_t> canonicalRecordBytes;
    OcctExactLabelReceipt source;
    pattern_recipe_clone::Source sourceRecipe;
    std::string sourceShapeBytes;
    AllLabelMeasuredCost measured;
    struct Member final {
        AllLabelMemberKey key;
        OcctExactLabelReceipt receipt;
        std::string featureIdentifier;
        std::uint64_t localIdentifier = 0;
        bool suppressed = false;
        pattern_recipe_clone::Source recipe;
        std::string shapeBytes;
        AllLabelMeasuredCost measured;
    };
    std::vector<Member> members;
    Standard_Integer documentTime = -1;
};

//! A detached D2 mutation. Label identities are document reservations, shapes
//! are independent BRep copies, and recipes are fully prepared before a
//! command is acquired. Nothing in this value can mutate OCAF.
struct AllLabelMutation final {
    struct Recipe final {
        D2Coordinate key;
        bool source = false;
        bool created = false;
        bool suppressed = false;
        std::string entityIdentifier;
        std::string definitionIdentifier;
        std::string featureIdentifier;
        pattern_recipe_clone::Prepared prepared;
    };
    std::shared_ptr<const AllLabelSnapshot> before;
    OcctAllLabelPlan labels;
    std::vector<Recipe> recipes;
    std::vector<std::uint8_t> canonicalCandidateBytes;
};

inline bool SameKey(const AllLabelMemberKey& left,
                    const AllLabelMemberKey& right) noexcept {
    if (left.index() != right.index()) return false;
    if (const auto* value = std::get_if<D2Coordinate>(&left)) {
        const auto* other = std::get_if<D2Coordinate>(&right);
        return other && value->row == other->row && value->column == other->column;
    }
    const auto* value = std::get_if<D3Ordinal>(&left);
    const auto* other = std::get_if<D3Ordinal>(&right);
    return value && other && value->ordinal == other->ordinal;
}

inline bool SameMeasuredCost(const AllLabelMeasuredCost& left,
                             const AllLabelMeasuredCost& right) noexcept {
    return left.recordBytes == right.recordBytes
        && left.recipeBytes == right.recipeBytes
        && left.shapeBytes == right.shapeBytes
        && left.topologyNodes == right.topologyNodes
        && left.retainedMemoryBytes == right.retainedMemoryBytes;
}

//! Exact comparison used for pre-command currentness and inside/after-command
//! read-back. Shape bytes are compared in addition to oriented TopoDS bindings:
//! a same-label replacement cannot pass merely because scalar recipes match.
inline bool IsExactlyEqual(const AllLabelSnapshot& left,
                           const AllLabelSnapshot& right) noexcept {
    try {
        const bool sameRecordLabel = left.recordLabel.IsNull()
            ? right.recordLabel.IsNull()
            : !right.recordLabel.IsNull()
                && left.recordLabel.IsEqual(right.recordLabel)
                && left.recordLabel.Data() == right.recordLabel.Data();
        if (left.documentData != right.documentData
            || left.documentIdentifier != right.documentIdentifier
            || left.family != right.family || !sameRecordLabel
            || left.featureIdentifier != right.featureIdentifier
            || left.canonicalRecordBytes != right.canonicalRecordBytes
            || !left.source.IsEqual(right.source)
            || !pattern_recipe_clone::IsEqual(left.sourceRecipe, right.sourceRecipe)
            || left.sourceShapeBytes != right.sourceShapeBytes
            || !SameMeasuredCost(left.measured, right.measured)
            || left.documentTime != right.documentTime
            || left.members.size() != right.members.size()) return false;
        for (std::size_t index = 0; index < left.members.size(); ++index) {
            const auto& a = left.members[index]; const auto& b = right.members[index];
            if (!SameKey(a.key, b.key) || !a.receipt.IsEqual(b.receipt)
                || a.featureIdentifier != b.featureIdentifier
                || a.localIdentifier != b.localIdentifier
                || a.suppressed != b.suppressed
                || !pattern_recipe_clone::IsEqual(a.recipe, b.recipe)
                || a.shapeBytes != b.shapeBytes
                || !SameMeasuredCost(a.measured, b.measured)) return false;
        }
        return true;
    } catch (...) { return false; }
}

//! Construction is reserved to the document-backed authority introduced by
//! the D2/D3 packages; adapters may retain snapshots but cannot mint them.
class OcafAllLabelAuthority final {
public:
    const std::shared_ptr<const AllLabelSnapshot>& snapshot() const noexcept {
        return snapshot_;
    }
private:
    friend class ::OcctDocument;
    explicit OcafAllLabelAuthority(
        std::shared_ptr<const AllLabelSnapshot> snapshot) noexcept
        : snapshot_(std::move(snapshot)) {}
    std::shared_ptr<const AllLabelSnapshot> snapshot_;
};

} // namespace core3d::pattern_owner

namespace core3d::native_opening {

//! Projects a validated all-label plan to the affected persistent identities
//! for post-commit viewer publication. Created identities resolve against the
//! committed document; removals carry their pre-removal shape so a stale
//! presentation whose label is gone can still be recognized exactly.
inline CommittedEditPublication PublicationFromAllLabelPlan(
    const OcctAllLabelPlan& plan) {
    CommittedEditPublication publication;
    for (const auto& item : plan.creates)
        publication.created.push_back({item.identity.EntityIdentifier(), {}});
    for (const auto& item : plan.replacements)
        publication.replaced.push_back(
            {item.expected.visibility.object.object.entityIdentifier, {}});
    for (const auto& item : plan.removals)
        publication.removed.push_back(
            {item.visibility.object.object.entityIdentifier,
             item.visibility.object.object.shape});
    return publication;
}

} // namespace core3d::native_opening
