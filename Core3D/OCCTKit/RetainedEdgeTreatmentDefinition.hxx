#pragma once

#include "RetainedFaceSelectorValues.hxx"
#include "RetainedRecipeIdentity.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <optional>
#include <set>
#include <vector>

namespace core3d::retained_edge_treatment {
using UUID = retained_recipe::UUID;
using Digest = retained_recipe::Digest;
enum class Kind : std::uint8_t { Chamfer = 1, ConstantFillet = 2 };
enum class SourceFamily : std::uint8_t { Profile = 1, Enclosure = 2 };
enum class CurveKind : std::uint8_t { Line = 1, Circle = 2 };
enum class Refusal : std::uint8_t {
    None = 0, UnsupportedBase, MultipleOwners, NoncurrentSource, MalformedCarrier,
    UnsupportedVersion, NonemptyBooleanPrefix, UnsupportedOperation, InvalidAmount,
    Clearance, UnsupportedEdge, AnchorMissing, AnchorAmbiguous, ContourExpansion,
    UnsupportedTransform, UnitMismatch, StaleSnapshot, UnsupportedDependency, Budget,
    IdentityMismatch, ReplayMismatch, NonRemoving, BuildFailed, StageFailed, Cancelled,
    Busy, OutcomeUnknown
};
struct EmptyBooleanPrefix {};
struct BaseBinding {
    SourceFamily family = SourceFamily::Profile;
    retained_recipe::SourceIdentity source;
    UUID sourceNode{};
    std::uint32_t sourceSchema = 0;
    Digest sourceRecipeDigest{};
    double metersPerLocalUnit = 0;
    EmptyBooleanPrefix booleanPrefix;
};
struct Anchor {
    UUID key{};
    CurveKind curve = CurveKind::Line;
    std::array<double, 3> pointMM{}, tangent{}, normalA{}, normalB{};
    double circleRadiusMM = 0;
    bool operator==(const Anchor& other) const noexcept {
        return key == other.key && curve == other.curve && pointMM == other.pointMM
            && tangent == other.tangent && normalA == other.normalA
            && normalB == other.normalB && circleRadiusMM == other.circleRadiusMM;
    }
};
struct Step {
    UUID node{}, feature{};
    std::uint64_t localID = 0;
    Kind kind = Kind::Chamfer;
    double amountMM = 0;
    std::vector<Anchor> anchors;
    // B2 is the only extension of the sole B1 carrier. Anchors remain stored once.
    std::optional<retained_face_selector::SelectorReceipt> selector;
    bool operator==(const Step& other) const noexcept {
        return node == other.node && feature == other.feature && localID == other.localID
            && kind == other.kind && amountMM == other.amountMM
            && anchors == other.anchors && selector == other.selector;
    }
};
struct Definition {
    std::uint32_t schema = 1;
    retained_recipe::OwnerKey owner;
    BaseBinding base;
    retained_recipe::IssuanceState issuance;
    UUID outputNode{};
    std::vector<Step> steps;
};
inline constexpr std::size_t MaximumSteps = 8;
inline constexpr std::size_t MaximumAnchorsPerStep = 64;
inline constexpr std::size_t MaximumAnchors = 512;
inline constexpr std::size_t MaximumEnvelopeBytes = 65'536;

namespace detail {
struct Writer {
    std::vector<std::uint8_t> b;
    bool ok = true;
    void raw(const void* pointer, std::size_t count) {
        if (!ok || count > MaximumEnvelopeBytes - b.size()) { ok = false; return; }
        const auto* bytes = static_cast<const std::uint8_t*>(pointer);
        b.insert(b.end(), bytes, bytes + count);
    }
    template<std::size_t N> void raw(const std::array<std::uint8_t, N>& value) { raw(value.data(), N); }
    void u(std::uint64_t value, unsigned count) {
        std::uint8_t bytes[8]{};
        for (unsigned index = 0; index < count; ++index) bytes[index] = std::uint8_t(value >> (8 * index));
        raw(bytes, count);
    }
    void d(double value) {
        if (!std::isfinite(value)) { ok = false; return; }
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        u(bits, 8);
    }
};
struct Reader {
    const std::vector<std::uint8_t>& b;
    std::size_t end = 0, pos = 0;
    bool ok = true;
    bool raw(void* pointer, std::size_t count) {
        if (!ok || count > end - pos) { ok = false; return false; }
        std::memcpy(pointer, b.data() + pos, count); pos += count; return true;
    }
    template<std::size_t N> bool raw(std::array<std::uint8_t, N>& value) { return raw(value.data(), N); }
    bool u(unsigned count, std::uint64_t& value) {
        value = 0;
        if (!ok || count > end - pos) { ok = false; return false; }
        for (unsigned index = 0; index < count; ++index) value |= std::uint64_t(b[pos++]) << (8 * index);
        return true;
    }
    bool d(double& value) {
        std::uint64_t bits = 0;
        if (!u(8, bits)) return false;
        std::memcpy(&value, &bits, sizeof(value));
        return std::isfinite(value);
    }
};
inline bool unit(const std::array<double, 3>& value) {
    const double squared = value[0] * value[0] + value[1] * value[1] + value[2] * value[2];
    return std::isfinite(squared) && std::abs(squared - 1.0) <= 1e-12;
}
inline bool valid(const Definition& definition, Refusal& refusal) {
    if (definition.schema != 1 && definition.schema != 2) { refusal = Refusal::UnsupportedVersion; return false; }
    if (!retained_recipe::Valid(definition.owner) || !retained_recipe::Valid(definition.base.source)
        || !retained_recipe::Nonzero(definition.base.sourceNode)
        || !retained_recipe::Nonzero(definition.base.sourceRecipeDigest)
        || definition.base.source.document != definition.owner.document
        || definition.base.source.entity != definition.owner.entity
        || definition.base.source.definition != definition.owner.definition) {
        refusal = Refusal::IdentityMismatch; return false;
    }
    if (definition.base.sourceSchema == 0 || !std::isfinite(definition.base.metersPerLocalUnit)
        || definition.base.metersPerLocalUnit <= 0 || !retained_recipe::Valid(definition.issuance)
        || definition.steps.size() > MaximumSteps) { refusal = Refusal::Budget; return false; }
    std::set<UUID> nodes, features, keys;
    std::set<std::uint64_t> live;
    std::size_t total = 0;
    bool hasSelector = false;
    for (const auto& step : definition.steps) {
        if (!retained_recipe::Nonzero(step.node) || !nodes.insert(step.node).second
            || !retained_recipe::Nonzero(step.feature) || !features.insert(step.feature).second
            || step.localID == 0 || step.localID >= definition.issuance.nextLocalID
            || !live.insert(step.localID).second) { refusal = Refusal::MalformedCarrier; return false; }
        if (!std::isfinite(step.amountMM) || step.amountMM <= 0 || step.amountMM > 20) {
            refusal = Refusal::InvalidAmount; return false;
        }
        if (step.anchors.empty() || step.anchors.size() > MaximumAnchorsPerStep
            || total + step.anchors.size() > MaximumAnchors) { refusal = Refusal::Budget; return false; }
        total += step.anchors.size();
        UUID previous{};
        bool first = true;
        for (const auto& anchor : step.anchors) {
            if (!retained_recipe::Nonzero(anchor.key) || !keys.insert(anchor.key).second
                || (!first && !(previous < anchor.key)) || !unit(anchor.tangent)
                || !unit(anchor.normalA) || !unit(anchor.normalB)) {
                refusal = Refusal::MalformedCarrier; return false;
            }
            for (double position : anchor.pointMM) {
                if (!std::isfinite(position) || std::abs(position) > 1'000'000.0) {
                    refusal = Refusal::MalformedCarrier; return false;
                }
            }
            if ((anchor.curve == CurveKind::Line && anchor.circleRadiusMM != 0)
                || (anchor.curve == CurveKind::Circle
                    && (!std::isfinite(anchor.circleRadiusMM) || anchor.circleRadiusMM <= 0))) {
                refusal = Refusal::UnsupportedEdge; return false;
            }
            previous = anchor.key; first = false;
        }
        if (step.selector) {
            hasSelector = true;
            if (definition.schema != 2 || step.kind != Kind::ConstantFillet
                || !retained_face_selector::ValidReceipt(*step.selector)
                || step.selector->entries.size() != step.anchors.size()) {
                refusal = Refusal::MalformedCarrier; return false;
            }
            for (std::size_t index = 0; index < step.anchors.size(); ++index) {
                if (step.selector->entries[index].anchorKey != step.anchors[index].key) {
                    refusal = Refusal::MalformedCarrier; return false;
                }
            }
        }
    }
    if (definition.schema == 1 && hasSelector) { refusal = Refusal::MalformedCarrier; return false; }
    for (auto identifier : definition.issuance.retiredLocalIDs) {
        if (live.count(identifier)) { refusal = Refusal::MalformedCarrier; return false; }
    }
    if ((definition.steps.empty() && definition.outputNode != definition.base.sourceNode)
        || (!definition.steps.empty() && definition.outputNode != definition.steps.back().node)) {
        refusal = Refusal::IdentityMismatch; return false;
    }
    refusal = Refusal::None; return true;
}
} // namespace detail

inline bool Encode(const Definition& definition, std::vector<std::uint8_t>& output, Refusal& refusal) noexcept {
    output.clear();
    try {
        if (!detail::valid(definition, refusal)) return false;
        detail::Writer writer;
        writer.raw("SYET", 4); writer.u(definition.schema, 4);
        writer.raw(definition.owner.document); writer.raw(definition.owner.entity); writer.raw(definition.owner.definition);
        writer.u(1, 4); writer.u(std::uint8_t(definition.base.family), 1);
        writer.raw(definition.base.source.document); writer.raw(definition.base.source.entity);
        writer.raw(definition.base.source.definition); writer.raw(definition.base.source.sourceFeature);
        writer.raw(definition.base.sourceNode); writer.u(definition.base.sourceSchema, 4);
        writer.raw(definition.base.sourceRecipeDigest); writer.d(definition.base.metersPerLocalUnit); writer.u(0, 4);
        writer.u(definition.issuance.nextLocalID, 8); writer.u(definition.issuance.retiredLocalIDs.size(), 4);
        for (auto identifier : definition.issuance.retiredLocalIDs) writer.u(identifier, 8);
        writer.raw(definition.outputNode); writer.u(definition.steps.size(), 4);
        for (const auto& step : definition.steps) {
            writer.raw(step.node); writer.raw(step.feature); writer.u(step.localID, 8);
            writer.u(std::uint8_t(step.kind), 1); writer.d(step.amountMM); writer.u(step.anchors.size(), 4);
            for (const auto& anchor : step.anchors) {
                writer.raw(anchor.key); writer.u(std::uint8_t(anchor.curve), 1);
                for (double part : anchor.pointMM) writer.d(part);
                for (double part : anchor.tangent) writer.d(part);
                for (double part : anchor.normalA) writer.d(part);
                for (double part : anchor.normalB) writer.d(part);
                writer.d(anchor.circleRadiusMM);
            }
            if (definition.schema == 2) {
                writer.u(step.selector ? 1 : 0, 1);
                if (step.selector && !retained_face_selector::WriteReceipt(writer, *step.selector)) {
                    refusal = Refusal::MalformedCarrier; return false;
                }
            }
        }
        if (!writer.ok || writer.b.size() > MaximumEnvelopeBytes - 32) { refusal = Refusal::Budget; return false; }
        Digest digest{};
        if (!CC_SHA256(writer.b.data(), CC_LONG(writer.b.size()), digest.data())) {
            refusal = Refusal::MalformedCarrier; return false;
        }
        writer.raw(digest); output = std::move(writer.b); refusal = Refusal::None; return true;
    } catch (...) { output.clear(); refusal = Refusal::MalformedCarrier; return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, std::optional<Definition>& output, Refusal& refusal) noexcept {
    output.reset();
    try {
        if (bytes.size() < 200 || bytes.size() > MaximumEnvelopeBytes || std::memcmp(bytes.data(), "SYET", 4)) {
            refusal = Refusal::MalformedCarrier; return false;
        }
        Digest actual{}, expected{};
        std::copy_n(bytes.end() - 32, 32, actual.begin());
        if (!CC_SHA256(bytes.data(), CC_LONG(bytes.size() - 32), expected.data()) || actual != expected) {
            refusal = Refusal::MalformedCarrier; return false;
        }
        detail::Reader reader{bytes, bytes.size() - 32};
        char magic[4]; std::uint64_t value = 0, count = 0; Definition definition;
        if (!reader.raw(magic, 4) || !reader.u(4, value) || (value != 1 && value != 2)) {
            refusal = Refusal::UnsupportedVersion; return false;
        }
        definition.schema = std::uint32_t(value);
        if (!reader.raw(definition.owner.document) || !reader.raw(definition.owner.entity)
            || !reader.raw(definition.owner.definition) || !reader.u(4, count) || count != 1) {
            refusal = Refusal::MalformedCarrier; return false;
        }
        if (!reader.u(1, value) || (value != 1 && value != 2)) { refusal = Refusal::UnsupportedBase; return false; }
        definition.base.family = SourceFamily(value);
        if (!reader.raw(definition.base.source.document) || !reader.raw(definition.base.source.entity)
            || !reader.raw(definition.base.source.definition) || !reader.raw(definition.base.source.sourceFeature)
            || !reader.raw(definition.base.sourceNode) || !reader.u(4, value)) return false;
        definition.base.sourceSchema = std::uint32_t(value);
        if (!reader.raw(definition.base.sourceRecipeDigest) || !reader.d(definition.base.metersPerLocalUnit)
            || !reader.u(4, count)) return false;
        if (count != 0) { refusal = Refusal::NonemptyBooleanPrefix; return false; }
        if (!reader.u(8, definition.issuance.nextLocalID) || !reader.u(4, count) || count > 4096) {
            refusal = Refusal::Budget; return false;
        }
        for (std::uint64_t index = 0; index < count; ++index) {
            if (!reader.u(8, value)) return false; definition.issuance.retiredLocalIDs.push_back(value);
        }
        if (!reader.raw(definition.outputNode) || !reader.u(4, count) || count > MaximumSteps) {
            refusal = Refusal::Budget; return false;
        }
        for (std::uint64_t index = 0; index < count; ++index) {
            Step step;
            if (!reader.raw(step.node) || !reader.raw(step.feature) || !reader.u(8, step.localID)
                || !reader.u(1, value) || (value != 1 && value != 2) || !reader.d(step.amountMM)) return false;
            step.kind = Kind(value);
            std::uint64_t anchorCount = 0;
            if (!reader.u(4, anchorCount) || anchorCount == 0 || anchorCount > MaximumAnchorsPerStep) {
                refusal = Refusal::Budget; return false;
            }
            for (std::uint64_t anchorIndex = 0; anchorIndex < anchorCount; ++anchorIndex) {
                Anchor anchor;
                if (!reader.raw(anchor.key) || !reader.u(1, value) || (value != 1 && value != 2)) return false;
                anchor.curve = CurveKind(value);
                for (double& part : anchor.pointMM) if (!reader.d(part)) return false;
                for (double& part : anchor.tangent) if (!reader.d(part)) return false;
                for (double& part : anchor.normalA) if (!reader.d(part)) return false;
                for (double& part : anchor.normalB) if (!reader.d(part)) return false;
                if (!reader.d(anchor.circleRadiusMM)) return false;
                step.anchors.push_back(anchor);
            }
            if (definition.schema == 2) {
                if (!reader.u(1, value) || value > 1) { refusal = Refusal::MalformedCarrier; return false; }
                if (value == 1) {
                    retained_face_selector::SelectorReceipt receipt;
                    if (!retained_face_selector::ReadReceipt(reader, receipt)) {
                        refusal = Refusal::MalformedCarrier; return false;
                    }
                    step.selector = std::move(receipt);
                }
            }
            definition.steps.push_back(std::move(step));
        }
        if (!reader.ok || reader.pos != reader.end || !detail::valid(definition, refusal)) return false;
        std::vector<std::uint8_t> exact;
        if (!Encode(definition, exact, refusal) || exact != bytes) {
            refusal = Refusal::MalformedCarrier; return false;
        }
        output = std::move(definition); return true;
    } catch (...) { output.reset(); refusal = Refusal::MalformedCarrier; return false; }
}

inline const char* RefusalCode(Refusal refusal) noexcept {
    static const char* codes[] = {"b1.None","b1.UnsupportedBase","b1.MultipleOwners","b1.NoncurrentSource","b1.MalformedCarrier","b1.UnsupportedVersion","b1.NonemptyBooleanPrefix","b1.UnsupportedOperation","b1.InvalidAmount","b1.Clearance","b1.UnsupportedEdge","b1.AnchorMissing","b1.AnchorAmbiguous","b1.ContourExpansion","b1.UnsupportedTransform","b1.UnitMismatch","b1.StaleSnapshot","b1.UnsupportedDependency","b1.Budget","b1.IdentityMismatch","b1.ReplayMismatch","b1.NonRemoving","b1.BuildFailed","b1.StageFailed","b1.Cancelled","b1.Busy","b1.OutcomeUnknown"};
    return codes[std::size_t(refusal)];
}
inline const char* RefusalMessage(Refusal refusal) noexcept {
    static const char* messages[] = {"","Select one current saved profile or enclosure for this edge treatment.","Apply retained edge treatments to one object at a time.","The saved source is no longer current; this edge treatment cannot be edited.","The saved edge treatment is incomplete or invalid.","This saved edge-treatment version is not supported.","This edge treatment requires a source with no preceding Boolean feature.","Only equal-distance chamfers and constant-radius fillets are supported here.","Enter a finite amount greater than 0 and no greater than 20 mm.","The amount is too large for the selected edges or nearby faces.","The selected edge geometry cannot be retained by this tool.","A selected edge no longer exists in the rebuilt source.","A selected edge no longer has a unique match.","The operation would change edges that were not selected.","This edge treatment requires an unscaled rigid source frame.","The document units changed; reopen the edge-treatment editor.","The model changed; reopen the edge-treatment editor.","A dependent feature cannot be replayed; no edit was applied.","This edge treatment exceeds the supported size or work limit.","The saved edge treatment does not belong to this source.","The rebuilt treatment does not match the saved feature.","The operation would add material or remove no measurable material.","The selected edge treatment could not be built.","The edge treatment could not be saved; no edit was applied.","The edge-treatment edit was cancelled.","Finish the current operation before editing edge treatments.","The edit needs recovery before more changes can be made."};
    return messages[std::size_t(refusal)];
}
} // namespace core3d::retained_edge_treatment
