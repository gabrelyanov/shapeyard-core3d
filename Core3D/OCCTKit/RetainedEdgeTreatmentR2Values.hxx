#pragma once

// B1 source contract revision 2.  This namespace is intentionally separate
// from the frozen revision-1 declarations in RetainedEdgeTreatmentDefinition.hxx.
#include "RetainedEdgeTreatmentDefinition.hxx"
#include "RetainedBooleanProgram.hxx"
#include "CompositeRecipeDefinition.hxx"
#include "RetainedPartBoolean.hxx"
#include "RectangularLoftDefinition.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <cstring>
#include <limits>
#include <variant>

namespace core3d::retained_edge_treatment::r2 {
namespace et = core3d::retained_edge_treatment;
namespace rr = core3d::retained_recipe;
namespace fs = core3d::retained_face_selector;

inline constexpr std::uint32_t SourceContractRevision = 2;
inline constexpr std::uint32_t PrefixBindingVersion = 1;
inline constexpr std::uint32_t MigrationVersion = 1;
inline constexpr std::size_t MaximumMigrationSteps = 8;
inline constexpr std::size_t MaximumMigrationAnchors = 16;

using UUID = rr::UUID;
using Digest = rr::Digest;
enum class SourceFamily : std::uint8_t { Profile = 1, Enclosure = 2, RetainedBoolean = 3 };
enum class PrefixFormat : std::uint8_t { SYRS = 1, A1Composite = 2 };

struct LegacyLink {
    std::uint32_t operandID = 0;
    UUID node{}, feature{}, toolNode{}, toolFeature{}, leftNode{}, rightNode{};
    bool operator==(const LegacyLink& other) const noexcept {
        return operandID == other.operandID && node == other.node && feature == other.feature
            && toolNode == other.toolNode && toolFeature == other.toolFeature
            && leftNode == other.leftNode && rightNode == other.rightNode;
    }
};
struct CompositeLink {
    UUID node{}, feature{}, leftNode{}, rightNode{};
    bool operator==(const CompositeLink& other) const noexcept {
        return node == other.node && feature == other.feature
            && leftNode == other.leftNode && rightNode == other.rightNode;
    }
};
struct LegacyPrefixBinding {
    UUID rootNode{};
    std::vector<LegacyLink> links;
    bool operator==(const LegacyPrefixBinding& other) const noexcept {
        return rootNode == other.rootNode && links == other.links;
    }
};
struct CompositePrefixBinding {
    std::vector<CompositeLink> links;
    bool operator==(const CompositePrefixBinding& other) const noexcept { return links == other.links; }
};
struct LegacyStepMap {
    std::uint64_t oldStepID = 0;
    UUID node{}, feature{};
    bool retired = false;
    bool operator==(const LegacyStepMap& other) const noexcept {
        return oldStepID == other.oldStepID && node == other.node
            && feature == other.feature && retired == other.retired;
    }
};
struct LegacyAnchorMap {
    std::uint64_t oldEdgeID = 0;
    UUID key{};
    bool retired = false;
    bool operator==(const LegacyAnchorMap& other) const noexcept {
        return oldEdgeID == other.oldEdgeID && key == other.key && retired == other.retired;
    }
};
struct MigrationM3Provenance {
    std::uint32_t version = MigrationVersion;
    Digest originalRecipeDigest{};
    std::uint64_t nextOperandID = 0, nextFilletStepID = 0, nextFilletEdgeID = 0;
    std::vector<LegacyStepMap> steps;
    std::vector<LegacyAnchorMap> anchors;
    bool operator==(const MigrationM3Provenance& other) const noexcept {
        return version == other.version && originalRecipeDigest == other.originalRecipeDigest
            && nextOperandID == other.nextOperandID && nextFilletStepID == other.nextFilletStepID
            && nextFilletEdgeID == other.nextFilletEdgeID && steps == other.steps
            && anchors == other.anchors;
    }
};
struct BooleanBaseBinding {
    std::uint32_t sourceContractRevision = SourceContractRevision;
    PrefixFormat format = PrefixFormat::SYRS;
    std::uint32_t prefixBindingVersion = PrefixBindingVersion;
    std::uint8_t sourceWireMajor = 0, sourceWireMinor = 0;
    rr::SourceIdentity source;
    UUID sourceNode{};
    std::uint32_t sourceSchema = 0;
    Digest sourceRecipeDigest{};
    double metersPerLocalUnit = 0;
    std::variant<LegacyPrefixBinding, CompositePrefixBinding> prefix;
    std::optional<MigrationM3Provenance> migration;
    bool operator==(const BooleanBaseBinding& other) const noexcept {
        return sourceContractRevision == other.sourceContractRevision && format == other.format
            && prefixBindingVersion == other.prefixBindingVersion
            && sourceWireMajor == other.sourceWireMajor && sourceWireMinor == other.sourceWireMinor
            && source == other.source && sourceNode == other.sourceNode
            && sourceSchema == other.sourceSchema && sourceRecipeDigest == other.sourceRecipeDigest
            && metersPerLocalUnit == other.metersPerLocalUnit && prefix == other.prefix
            && migration == other.migration;
    }
};
struct LegacyBooleanBase {
    retained_boolean::Recipe prefix;
    std::vector<std::uint8_t> canonicalPrefixBytes;
};
struct CompositeBooleanBase {
    composite_recipe::Definition prefix;
    std::vector<std::uint8_t> canonicalPrefixBytes;
};
struct RetainedBooleanBase {
    BooleanBaseBinding binding;
    std::variant<LegacyBooleanBase, CompositeBooleanBase> source;
};
using BaseRecipe = std::variant<profile::Parameters, enclosure::Parameters, RetainedBooleanBase>;
using BaseBinding = std::variant<et::BaseBinding, BooleanBaseBinding>;
struct Definition {
    std::uint32_t schema = 2;
    rr::OwnerKey owner;
    BaseBinding base;
    rr::IssuanceState issuance;
    UUID outputNode{};
    std::vector<et::Step> steps;
};

using InputRecipe = std::variant<profile::Parameters, enclosure::Parameters,
                                 rectangular_loft::Definition>;
struct RebuildBooleanInput { rr::RecipeLocator locator; InputRecipe requested; };
struct RebuildAnalyticTool { std::uint32_t operandID = 0; analytic_boolean::Operand requested; };
struct SetBooleanOperation { UUID feature{}; retained_part_boolean::Operation requested = retained_part_boolean::Operation::Subtract; };
struct SetInputPlacement { rr::RecipeLocator locator; composite_recipe::InputPlacement requested; };

struct LegacySelectorBinding {
    std::uint64_t oldStepID = 0;
    fs::SelectorIntent intent;
};
struct SelectorAppendIntent { fs::SelectorIntent intent; double amountMM = 0; };
struct MigrationM3 {
    std::uint32_t version = MigrationVersion;
    std::vector<LegacySelectorBinding> selectors;
    std::optional<SelectorAppendIntent> append;
};

namespace detail {
inline bool Nonzero(const UUID& value) { return rr::Nonzero(value); }
inline bool SortedMaps(const MigrationM3Provenance& value) noexcept {
    if (value.version != MigrationVersion || !rr::Nonzero(value.originalRecipeDigest)
        || value.nextOperandID == 0 || value.nextFilletStepID == 0 || value.nextFilletEdgeID == 0
        || value.steps.size() > MaximumMigrationSteps || value.anchors.size() > MaximumMigrationAnchors) return false;
    std::uint64_t previous = 0;
    for (const auto& row : value.steps) {
        if (!row.oldStepID || row.oldStepID <= previous || !Nonzero(row.node) || !Nonzero(row.feature)) return false;
        previous = row.oldStepID;
    }
    previous = 0;
    for (const auto& row : value.anchors) {
        if (!row.oldEdgeID || row.oldEdgeID <= previous || !Nonzero(row.key)) return false;
        previous = row.oldEdgeID;
    }
    return true;
}
inline bool Valid(const BooleanBaseBinding& value) noexcept {
    if (value.sourceContractRevision != SourceContractRevision
        || value.prefixBindingVersion != PrefixBindingVersion || !rr::Valid(value.source)
        || !Nonzero(value.sourceNode) || !rr::Nonzero(value.sourceRecipeDigest)
        || !value.sourceSchema || !std::isfinite(value.metersPerLocalUnit)
        || value.metersPerLocalUnit <= 0) return false;
    if (value.format == PrefixFormat::SYRS) {
        const auto* prefix = std::get_if<LegacyPrefixBinding>(&value.prefix);
        if (!prefix || !Nonzero(prefix->rootNode) || prefix->links.empty() || prefix->links.size() > 4
            || (value.sourceWireMajor != 1 && value.sourceWireMajor != 2)
            || value.sourceWireMinor < 1 || value.sourceWireMinor > 4
            || !value.migration || !SortedMaps(*value.migration)) return false;
        UUID expectedLeft = prefix->rootNode;
        std::set<std::uint32_t> operands;
        for (const auto& link : prefix->links) {
            if (!link.operandID || !operands.insert(link.operandID).second || !Nonzero(link.node)
                || !Nonzero(link.feature) || !Nonzero(link.toolNode) || !Nonzero(link.toolFeature)
                || link.leftNode != expectedLeft || link.rightNode != link.toolNode) return false;
            expectedLeft = link.node;
        }
        return expectedLeft == value.sourceNode;
    }
    if (value.format == PrefixFormat::A1Composite) {
        const auto* prefix = std::get_if<CompositePrefixBinding>(&value.prefix);
        return prefix && prefix->links.size() == 1 && value.sourceWireMajor == 1
            && value.sourceWireMinor == 0 && !value.migration && Nonzero(prefix->links[0].node)
            && Nonzero(prefix->links[0].feature) && Nonzero(prefix->links[0].leftNode)
            && Nonzero(prefix->links[0].rightNode) && prefix->links[0].node == value.sourceNode;
    }
    return false;
}
inline bool Valid(const Definition& value, et::Refusal& refusal) noexcept {
    if (const auto* old = std::get_if<et::BaseBinding>(&value.base)) {
        et::Definition lifted{value.schema, value.owner, *old, value.issuance, value.outputNode, value.steps};
        return et::detail::valid(lifted, refusal);
    }
    const auto* boolean = std::get_if<BooleanBaseBinding>(&value.base);
    if (value.schema != 2 || !rr::Valid(value.owner) || !boolean || !Valid(*boolean)
        || boolean->source.document != value.owner.document || boolean->source.entity != value.owner.entity
        || boolean->source.definition != value.owner.definition || !rr::Valid(value.issuance)
        || value.steps.size() > et::MaximumSteps) { refusal = et::Refusal::MalformedCarrier; return false; }
    et::BaseBinding shadow;
    shadow.family = et::SourceFamily::Profile;
    shadow.source = boolean->source;
    shadow.sourceNode = boolean->sourceNode;
    shadow.sourceSchema = boolean->sourceSchema;
    shadow.sourceRecipeDigest = boolean->sourceRecipeDigest;
    shadow.metersPerLocalUnit = boolean->metersPerLocalUnit;
    et::Definition checked{2, value.owner, shadow, value.issuance, value.outputNode, value.steps};
    if (!et::detail::valid(checked, refusal)) return false;
    refusal = et::Refusal::None;
    return true;
}

struct Writer : et::detail::Writer {};
struct Reader : et::detail::Reader {
    Reader(const std::vector<std::uint8_t>& bytes, std::size_t limit,
           std::size_t offset, bool valid) : et::detail::Reader{bytes, limit, offset, valid} {}
};
inline void Step(Writer& writer, const et::Step& step) {
    writer.raw(step.node); writer.raw(step.feature); writer.u(step.localID, 8);
    writer.u(std::uint8_t(step.kind), 1); writer.d(step.amountMM); writer.u(step.anchors.size(), 4);
    for (const auto& anchor : step.anchors) {
        writer.raw(anchor.key); writer.u(std::uint8_t(anchor.curve), 1);
        for (double value : anchor.pointMM) writer.d(value);
        for (double value : anchor.tangent) writer.d(value);
        for (double value : anchor.normalA) writer.d(value);
        for (double value : anchor.normalB) writer.d(value);
        writer.d(anchor.circleRadiusMM);
    }
    writer.u(step.selector ? 1 : 0, 1);
    if (step.selector) fs::WriteReceipt(writer, *step.selector);
}
inline bool Step(Reader& reader, et::Step& step) {
    std::uint64_t value = 0;
    if (!reader.raw(step.node) || !reader.raw(step.feature) || !reader.u(8, step.localID)
        || !reader.u(1, value)) return false;
    step.kind = et::Kind(value);
    if (!reader.d(step.amountMM) || !reader.u(4, value) || !value || value > et::MaximumAnchorsPerStep) return false;
    step.anchors.resize(std::size_t(value));
    for (auto& anchor : step.anchors) {
        if (!reader.raw(anchor.key) || !reader.u(1, value)) return false;
        anchor.curve = et::CurveKind(value);
        for (double& part : anchor.pointMM) if (!reader.d(part)) return false;
        for (double& part : anchor.tangent) if (!reader.d(part)) return false;
        for (double& part : anchor.normalA) if (!reader.d(part)) return false;
        for (double& part : anchor.normalB) if (!reader.d(part)) return false;
        if (!reader.d(anchor.circleRadiusMM)) return false;
    }
    if (!reader.u(1, value) || value > 1) return false;
    if (value) { fs::SelectorReceipt receipt; if (!fs::ReadReceipt(reader, receipt)) return false; step.selector = std::move(receipt); }
    return true;
}
} // namespace detail

inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output, et::Refusal& refusal) noexcept {
    output.clear();
    try {
        if (!detail::Valid(value, refusal)) return false;
        if (const auto* old = std::get_if<et::BaseBinding>(&value.base)) {
            return et::Encode(et::Definition{value.schema, value.owner, *old, value.issuance,
                                             value.outputNode, value.steps}, output, refusal);
        }
        const auto& base = std::get<BooleanBaseBinding>(value.base);
        detail::Writer writer;
        writer.raw("SYET", 4); writer.u(2, 4);
        writer.raw(value.owner.document); writer.raw(value.owner.entity); writer.raw(value.owner.definition);
        writer.u(1, 4); writer.u(3, 1); writer.u(base.sourceContractRevision, 4);
        writer.u(std::uint8_t(base.format), 1); writer.u(base.prefixBindingVersion, 4);
        writer.u(base.sourceWireMajor, 1); writer.u(base.sourceWireMinor, 1);
        writer.raw(base.source.document); writer.raw(base.source.entity); writer.raw(base.source.definition);
        writer.raw(base.source.sourceFeature); writer.raw(base.sourceNode); writer.u(base.sourceSchema, 4);
        writer.raw(base.sourceRecipeDigest); writer.d(base.metersPerLocalUnit);
        if (const auto* prefix = std::get_if<LegacyPrefixBinding>(&base.prefix)) {
            writer.u(prefix->links.size(), 4); writer.raw(prefix->rootNode);
            for (const auto& link : prefix->links) {
                writer.u(link.operandID, 4); writer.raw(link.node); writer.raw(link.feature);
                writer.raw(link.toolNode); writer.raw(link.toolFeature); writer.raw(link.leftNode); writer.raw(link.rightNode);
            }
        } else {
            const auto& compositePrefix = std::get<CompositePrefixBinding>(base.prefix);
            writer.u(compositePrefix.links.size(), 4);
            for (const auto& link : compositePrefix.links) {
                writer.raw(link.node); writer.raw(link.feature); writer.raw(link.leftNode); writer.raw(link.rightNode);
            }
        }
        writer.u(base.migration ? 1 : 0, 1);
        if (base.migration) {
            const auto& migration = *base.migration;
            writer.raw("M3RB", 4); writer.u(migration.version, 4); writer.raw(migration.originalRecipeDigest);
            writer.u(migration.nextOperandID, 8); writer.u(migration.nextFilletStepID, 8);
            writer.u(migration.nextFilletEdgeID, 8); writer.u(migration.steps.size(), 4);
            for (const auto& row : migration.steps) {
                writer.u(row.oldStepID, 8); writer.raw(row.node); writer.raw(row.feature); writer.u(row.retired ? 1 : 0, 1);
            }
            writer.u(migration.anchors.size(), 4);
            for (const auto& row : migration.anchors) {
                writer.u(row.oldEdgeID, 8); writer.raw(row.key); writer.u(row.retired ? 1 : 0, 1);
            }
        }
        writer.u(value.issuance.nextLocalID, 8); writer.u(value.issuance.retiredLocalIDs.size(), 4);
        for (auto identifier : value.issuance.retiredLocalIDs) writer.u(identifier, 8);
        writer.raw(value.outputNode); writer.u(value.steps.size(), 4);
        for (const auto& step : value.steps) detail::Step(writer, step);
        if (!writer.ok || writer.b.size() > et::MaximumEnvelopeBytes - 32) { refusal = et::Refusal::Budget; return false; }
        Digest digest{};
        if (!CC_SHA256(writer.b.data(), CC_LONG(writer.b.size()), digest.data())) return false;
        writer.raw(digest); output = std::move(writer.b); refusal = et::Refusal::None; return true;
    } catch (...) { output.clear(); refusal = et::Refusal::MalformedCarrier; return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, std::optional<Definition>& output,
                   et::Refusal& refusal) noexcept {
    output.reset();
    try {
        if (bytes.size() < 9 || std::memcmp(bytes.data(), "SYET", 4) != 0) { refusal = et::Refusal::MalformedCarrier; return false; }
        std::uint32_t schema = 0; std::memcpy(&schema, bytes.data() + 4, 4);
        if (schema == 1 || (schema == 2 && bytes.size() > 60 && bytes[60] != 3)) {
            std::optional<et::Definition> old;
            if (!et::Decode(bytes, old, refusal) || !old) return false;
            output = Definition{old->schema, old->owner, old->base, old->issuance, old->outputNode, old->steps};
            return true;
        }
        if (schema != 2 || bytes.size() > et::MaximumEnvelopeBytes || bytes.size() < 32) {
            refusal = et::Refusal::UnsupportedVersion; return false;
        }
        Digest actual{};
        if (!CC_SHA256(bytes.data(), CC_LONG(bytes.size() - 32), actual.data())
            || !std::equal(actual.begin(), actual.end(), bytes.end() - 32)) { refusal = et::Refusal::MalformedCarrier; return false; }
        detail::Reader reader{bytes, bytes.size() - 32, 8, true};
        Definition value; value.schema = 2; std::uint64_t integer = 0;
        if (!reader.raw(value.owner.document) || !reader.raw(value.owner.entity) || !reader.raw(value.owner.definition)
            || !reader.u(4, integer) || integer != 1 || !reader.u(1, integer) || integer != 3) {
            refusal = et::Refusal::MalformedCarrier; return false;
        }
        BooleanBaseBinding base;
        if (!reader.u(4, integer)) return false; base.sourceContractRevision = std::uint32_t(integer);
        if (!reader.u(1, integer)) return false; base.format = PrefixFormat(integer);
        if (!reader.u(4, integer)) return false; base.prefixBindingVersion = std::uint32_t(integer);
        if (!reader.u(1, integer)) return false; base.sourceWireMajor = std::uint8_t(integer);
        if (!reader.u(1, integer)) return false; base.sourceWireMinor = std::uint8_t(integer);
        if (!reader.raw(base.source.document) || !reader.raw(base.source.entity) || !reader.raw(base.source.definition)
            || !reader.raw(base.source.sourceFeature) || !reader.raw(base.sourceNode) || !reader.u(4, integer)) return false;
        base.sourceSchema = std::uint32_t(integer);
        if (!reader.raw(base.sourceRecipeDigest) || !reader.d(base.metersPerLocalUnit) || !reader.u(4, integer)) return false;
        const std::size_t links = std::size_t(integer);
        if (base.format == PrefixFormat::SYRS) {
            LegacyPrefixBinding prefix; prefix.links.resize(links);
            if (!reader.raw(prefix.rootNode)) return false;
            for (auto& link : prefix.links) {
                if (!reader.u(4, integer)) return false; link.operandID = std::uint32_t(integer);
                if (!reader.raw(link.node) || !reader.raw(link.feature) || !reader.raw(link.toolNode)
                    || !reader.raw(link.toolFeature) || !reader.raw(link.leftNode) || !reader.raw(link.rightNode)) return false;
            }
            base.prefix = std::move(prefix);
        } else if (base.format == PrefixFormat::A1Composite) {
            CompositePrefixBinding prefix; prefix.links.resize(links);
            for (auto& link : prefix.links) if (!reader.raw(link.node) || !reader.raw(link.feature)
                || !reader.raw(link.leftNode) || !reader.raw(link.rightNode)) return false;
            base.prefix = std::move(prefix);
        } else return false;
        if (!reader.u(1, integer) || integer > 1) return false;
        if (integer) {
            char tag[4]{}; MigrationM3Provenance migration;
            if (!reader.raw(tag, 4) || std::memcmp(tag, "M3RB", 4) != 0 || !reader.u(4, integer)) return false;
            migration.version = std::uint32_t(integer);
            if (!reader.raw(migration.originalRecipeDigest) || !reader.u(8, migration.nextOperandID)
                || !reader.u(8, migration.nextFilletStepID) || !reader.u(8, migration.nextFilletEdgeID)
                || !reader.u(4, integer) || integer > MaximumMigrationSteps) return false;
            migration.steps.resize(std::size_t(integer));
            for (auto& row : migration.steps) {
                if (!reader.u(8, row.oldStepID) || !reader.raw(row.node) || !reader.raw(row.feature)
                    || !reader.u(1, integer) || integer > 1) return false; row.retired = integer != 0;
            }
            if (!reader.u(4, integer) || integer > MaximumMigrationAnchors) return false;
            migration.anchors.resize(std::size_t(integer));
            for (auto& row : migration.anchors) {
                if (!reader.u(8, row.oldEdgeID) || !reader.raw(row.key) || !reader.u(1, integer) || integer > 1) return false;
                row.retired = integer != 0;
            }
            base.migration = std::move(migration);
        }
        value.base = std::move(base);
        if (!reader.u(8, value.issuance.nextLocalID) || !reader.u(4, integer) || integer > 4096) return false;
        value.issuance.retiredLocalIDs.resize(std::size_t(integer));
        for (auto& identifier : value.issuance.retiredLocalIDs) if (!reader.u(8, identifier)) return false;
        if (!reader.raw(value.outputNode) || !reader.u(4, integer) || integer > et::MaximumSteps) return false;
        value.steps.resize(std::size_t(integer));
        for (auto& step : value.steps) if (!detail::Step(reader, step)) return false;
        if (!reader.ok || reader.pos != reader.end || !detail::Valid(value, refusal)) return false;
        output = std::move(value); refusal = et::Refusal::None; return true;
    } catch (...) { output.reset(); refusal = et::Refusal::MalformedCarrier; return false; }
}
} // namespace core3d::retained_edge_treatment::r2
