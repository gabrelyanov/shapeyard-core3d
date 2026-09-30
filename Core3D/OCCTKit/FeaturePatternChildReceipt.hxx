#pragma once

#include "FeaturePatternDefinition.hxx"

#include <CommonCrypto/CommonDigest.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <set>
#include <vector>

namespace core3d::feature_pattern_child {
using feature_pattern::UUID;
using Digest = std::array<std::uint8_t, 32>;

inline constexpr std::size_t MaximumSelectors = 64;
inline constexpr std::size_t MaximumReceiptBytes = 64 * 1024;
inline constexpr std::size_t MaximumDocumentBytes = 8 * 1024 * 1024;
inline constexpr std::size_t PersistedReferenceBudgetBytes = 64;

enum class SelectorKind : std::uint8_t {
    SourceOperand = 1,
    HostBoundary = 2,
    GeneratedSection = 3
};

struct Selector final {
    SelectorKind kind = SelectorKind::SourceOperand;
    UUID semantic{};
    std::uint32_t ordinal = 0;
    Digest proof{};
};

inline bool operator==(const Selector& left, const Selector& right) noexcept {
    return left.kind == right.kind && left.semantic == right.semantic
        && left.ordinal == right.ordinal && left.proof == right.proof;
}

inline bool SelectorLess(const Selector& left, const Selector& right) noexcept {
    if (left.kind != right.kind) return left.kind < right.kind;
    if (left.semantic != right.semantic) return left.semantic < right.semantic;
    if (left.ordinal != right.ordinal) return left.ordinal < right.ordinal;
    return left.proof < right.proof;
}

struct Receipt final {
    UUID document{}, hostEntity{}, hostDefinition{}, patternFeature{};
    UUID childFeature{}, instanceIdentity{}, baselineRecipeIdentity{};
    std::uint64_t localID = 0;
    std::int32_t row = 0, column = 0;
    std::vector<Selector> selectors;
};

inline bool Nonzero(const UUID& value) noexcept {
    return std::any_of(value.begin(), value.end(), [](std::uint8_t byte) {
        return byte != 0;
    });
}

inline bool Nonzero(const Digest& value) noexcept {
    return std::any_of(value.begin(), value.end(), [](std::uint8_t byte) {
        return byte != 0;
    });
}

inline bool Valid(const Receipt& value) noexcept {
    if (!Nonzero(value.document) || !Nonzero(value.hostEntity)
        || !Nonzero(value.hostDefinition) || !Nonzero(value.patternFeature)
        || !Nonzero(value.childFeature) || !Nonzero(value.instanceIdentity)
        || !Nonzero(value.baselineRecipeIdentity) || value.localID == 0
        || value.selectors.empty() || value.selectors.size() > MaximumSelectors)
        return false;
    std::set<std::pair<std::uint8_t, UUID>> unique;
    for (const Selector& selector : value.selectors) {
        const auto kind = std::uint8_t(selector.kind);
        if (kind < std::uint8_t(SelectorKind::SourceOperand)
            || kind > std::uint8_t(SelectorKind::GeneratedSection)
            || !Nonzero(selector.semantic) || !Nonzero(selector.proof)
            || !unique.emplace(kind, selector.semantic).second) return false;
    }
    return true;
}

namespace detail {
inline void U32(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
    for (unsigned index = 0; index < 4; ++index)
        bytes.push_back(std::uint8_t(value >> (8 * index)));
}
inline void U64(std::vector<std::uint8_t>& bytes, std::uint64_t value) {
    for (unsigned index = 0; index < 8; ++index)
        bytes.push_back(std::uint8_t(value >> (8 * index)));
}
inline void Raw(std::vector<std::uint8_t>& bytes, const UUID& value) {
    bytes.insert(bytes.end(), value.begin(), value.end());
}
inline bool Hash(const std::vector<std::uint8_t>& bytes, Digest& output) noexcept {
    return bytes.size() <= MaximumReceiptBytes
        && CC_SHA256(bytes.data(), CC_LONG(bytes.size()), output.data()) != nullptr;
}
} // namespace detail

// SYFC/1 is canonical: selectors are ordered by semantic key and duplicate
// kind/identity pairs are invalid. OCAF labels and process addresses are never
// part of these bytes; the companion attribute owns those references.
inline bool Encode(const Receipt& value,
                   std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!Valid(value)) return false;
        std::vector<Selector> selectors = value.selectors;
        std::sort(selectors.begin(), selectors.end(), SelectorLess);
        std::vector<std::uint8_t> bytes{'S','Y','F','C',1,0,0,0};
        for (const UUID* identifier : {&value.document, &value.hostEntity,
                &value.hostDefinition, &value.patternFeature, &value.childFeature,
                &value.instanceIdentity, &value.baselineRecipeIdentity})
            detail::Raw(bytes, *identifier);
        detail::U64(bytes, value.localID);
        detail::U32(bytes, std::uint32_t(value.row));
        detail::U32(bytes, std::uint32_t(value.column));
        detail::U32(bytes, std::uint32_t(selectors.size()));
        for (const Selector& selector : selectors) {
            bytes.push_back(std::uint8_t(selector.kind));
            bytes.insert(bytes.end(), 3, 0);
            detail::U32(bytes, selector.ordinal);
            detail::Raw(bytes, selector.semantic);
            bytes.insert(bytes.end(), selector.proof.begin(), selector.proof.end());
        }
        if (bytes.size() > MaximumReceiptBytes - Digest{}.size()) return false;
        Digest digest{};
        if (!detail::Hash(bytes, digest)) return false;
        bytes.insert(bytes.end(), digest.begin(), digest.end());
        output = std::move(bytes);
        return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes,
                   Receipt& output) noexcept {
    output = {};
    try {
        constexpr std::size_t fixed = 8 + 7 * 16 + 8 + 4 + 4 + 4;
        if (bytes.size() < fixed + Digest{}.size()
            || bytes.size() > MaximumReceiptBytes
            || std::memcmp(bytes.data(), "SYFC\1\0\0\0", 8) != 0) return false;
        const std::size_t bodySize = bytes.size() - Digest{}.size();
        std::vector<std::uint8_t> body(bytes.begin(), bytes.begin() + bodySize);
        Digest expected{}, actual{};
        if (!detail::Hash(body, expected)) return false;
        std::copy_n(bytes.begin() + bodySize, actual.size(), actual.begin());
        if (actual != expected) return false;
        std::size_t at = 8;
        const auto need = [&](std::size_t count) {
            return at <= bodySize && count <= bodySize - at;
        };
        const auto raw = [&](auto& value) {
            if (!need(value.size())) return false;
            std::copy_n(bytes.begin() + at, value.size(), value.begin());
            at += value.size(); return true;
        };
        const auto u32 = [&](std::uint32_t& value) {
            if (!need(4)) return false; value = 0;
            for (unsigned index = 0; index < 4; ++index)
                value |= std::uint32_t(bytes[at++]) << (8 * index);
            return true;
        };
        const auto u64 = [&](std::uint64_t& value) {
            if (!need(8)) return false; value = 0;
            for (unsigned index = 0; index < 8; ++index)
                value |= std::uint64_t(bytes[at++]) << (8 * index);
            return true;
        };
        Receipt value; std::uint32_t row = 0, column = 0, count = 0;
        if (!raw(value.document) || !raw(value.hostEntity)
            || !raw(value.hostDefinition) || !raw(value.patternFeature)
            || !raw(value.childFeature) || !raw(value.instanceIdentity)
            || !raw(value.baselineRecipeIdentity) || !u64(value.localID)
            || !u32(row) || !u32(column) || !u32(count)
            || count == 0 || count > MaximumSelectors
            || std::size_t(count) > (bodySize - at) / 56
            || bodySize - at != std::size_t(count) * 56) return false;
        value.row = std::int32_t(row); value.column = std::int32_t(column);
        value.selectors.reserve(count);
        for (std::uint32_t index = 0; index < count; ++index) {
            if (!need(8) || bytes[at + 1] || bytes[at + 2] || bytes[at + 3]) return false;
            Selector selector; selector.kind = SelectorKind(bytes[at]); at += 4;
            if (!u32(selector.ordinal) || !raw(selector.semantic)
                || !raw(selector.proof)) return false;
            value.selectors.push_back(std::move(selector));
        }
        std::vector<std::uint8_t> canonical;
        if (at != bodySize || !Valid(value) || !Encode(value, canonical)
            || canonical != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

struct Budget final {
    std::size_t limit = MaximumDocumentBytes;
    std::size_t bytes = 0;
    std::size_t selectors = 0;
    std::size_t baselineReferences = 0;
    bool rejected = false;
};

inline bool Charge(Budget& budget, std::size_t canonicalBytes,
                   std::size_t selectorCount) noexcept {
    if (budget.rejected || budget.limit > MaximumDocumentBytes
        || selectorCount == 0 || selectorCount > MaximumSelectors
        || canonicalBytes > MaximumReceiptBytes
        || canonicalBytes > std::numeric_limits<std::size_t>::max()
            - PersistedReferenceBudgetBytes) return false;
    const std::size_t charge = canonicalBytes + PersistedReferenceBudgetBytes;
    if (budget.bytes > budget.limit || charge > budget.limit - budget.bytes
        || budget.selectors > std::numeric_limits<std::size_t>::max() - selectorCount
        || budget.baselineReferences == std::numeric_limits<std::size_t>::max()) {
        budget.rejected = true; return false;
    }
    budget.bytes += charge; budget.selectors += selectorCount;
    ++budget.baselineReferences; return true;
}
} // namespace core3d::feature_pattern_child
