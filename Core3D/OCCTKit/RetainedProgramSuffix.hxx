#pragma once
// A3/P2 retained program suffix on a composite (SYCR) Boolean owner.
// Versioned values only. No owner, label, command, geometry or AI authority.
//
// A shell/analytic Boolean RESULT accepts an appended retained cut program
// (cylindrical through-hole, ring of holes, wedge slot) and a retained fillet
// tail exactly as a plain part does today, but the program is carried as one
// ordered composite FEATURE node whose input is the PartBoolean feature node.
// The legacy SYRS envelope is not widened: retained_boolean::ValidSource keeps
// its family 1-3 analytic-source gate, legacy single-source envelopes stay
// byte-identical, and no SYRS family/schema number is added. The suffix payload
// is a new closed codec (SYPS/1) inside the existing 16 KiB per-feature and
// 64 KiB per-envelope budgets; the composite carrier, source slots and driver
// are unchanged.
//
// The feature kind is the registry-owned T-A family allocation 0x00001001.
// Code never repeats that persisted literal and the historical raw draft kind
// 3 remains unregistered/refused.
#include "RetainedBooleanProgram.hxx"
#include "RetainedFeatureRegistry.hxx"
#include <cstring>

namespace core3d::retained_program_suffix {
using retained_solid::UUID;
using retained_solid::Digest;

inline constexpr std::uint32_t FeatureKind = retained_feature::RetainedProgramSuffixKind;
inline constexpr std::uint32_t FeatureCodec = 1;
inline constexpr std::uint32_t PayloadVersion = 1;
inline constexpr std::size_t MaximumPayloadBytes = composite_recipe::MaximumFeaturePayloadBytes;
inline constexpr std::size_t MaximumSteps = retained_boolean::MaximumOperands;

// Persisted material choice for newly exposed tool-boundary patches, matching
// the A3 material policy: the default is persisted, never recomputed.
enum class CutSurfaceMaterial : std::uint8_t { TargetDefault = 1, ToolRegion = 2 };

// The fillet tail reuses the minor-4 retained fillet values verbatim through
// inheritance, exactly as retained_boolean::Program does.
struct Definition final : retained_fillet::Program {
    UUID baseFeature{};     // owning FeatureNode::inputs[0]; the composite prefix feature
    UUID derivedFeature{};  // issued once on first append, stable across later edits
    double carrierMetersPerUnit = 0;
    std::uint64_t nextOperandID = 1;
    std::vector<retained_boolean::Step> steps;
    CutSurfaceMaterial cutSurfaceMaterial = CutSurfaceMaterial::TargetDefault;
    std::uint32_t buildProfile = 1, proofProfile = 1, selectorVersion = 1;
};

inline bool HasWedge(const Definition& p) {
    return std::any_of(p.steps.begin(), p.steps.end(), [](const retained_boolean::Step& s) {
        return s.operand.kind == analytic_boolean::OperandKind::Wedge;
    });
}
inline bool HasRing(const Definition& p) {
    return std::any_of(p.steps.begin(), p.steps.end(), [](const retained_boolean::Step& s) {
        return s.operand.kind == analytic_boolean::OperandKind::CylinderRing;
    });
}

// Step semantics are exactly the SYRS minor-3/4 operand rules evaluated against
// the carrier units; only the single-source recipe binding is replaced by the
// composite feature input.
inline bool Valid(const Definition& p) noexcept {
    try {
        if (!retained_solid::Nonzero(p.baseFeature) || !retained_solid::Nonzero(p.derivedFeature)
            || p.baseFeature == p.derivedFeature
            || !std::isfinite(p.carrierMetersPerUnit) || p.carrierMetersPerUnit <= 0
            || p.steps.empty() || p.steps.size() > MaximumSteps
            || p.nextOperandID < 2 || p.nextOperandID > std::uint64_t(UINT32_MAX) + 1
            || p.buildProfile != 1 || p.proofProfile != 1 || p.selectorVersion != 1
            || (p.cutSurfaceMaterial != CutSurfaceMaterial::TargetDefault
                && p.cutSurfaceMaterial != CutSurfaceMaterial::ToolRegion)) return false;
        const double mm = p.carrierMetersPerUnit * 1000;
        if (!std::isfinite(mm) || mm <= 0) return false;
        std::size_t disks = 0;
        for (std::size_t i = 0; i < p.steps.size(); ++i) {
            const auto& step = p.steps[i];
            analytic_boolean::Recipe r;
            r.metersPerUnit = p.carrierMetersPerUnit;
            r.operation = step.operation;
            r.tool = step.operand;
            if (!analytic_boolean::Inspect(r) || step.operand.identifier >= p.nextOperandID) return false;
            if (step.operand.kind == analytic_boolean::OperandKind::CylinderRing) {
                if (analytic_boolean_ring::Inspect(
                        analytic_boolean_ring::FromOperand(step.operand, p.carrierMetersPerUnit),
                        p.carrierMetersPerUnit) != analytic_boolean_ring::Status::Clear) return false;
                disks += step.operand.count;
            } else ++disks;
            if (disks > analytic_boolean_ring::kMaximumExpandedDisks) return false;
            for (std::size_t j = 0; j < i; ++j)
                if (p.steps[j].operand.identifier == step.operand.identifier) return false;
        }
        if (!retained_fillet::Valid(p, mm)) return false;
        // Checked complete size before Encode allocates, including digest.
        const std::size_t fixed = 8 + 32 + 8 + 8 + 12 + 4 + 8 + retained_fillet::EncodedSize(p);
        return fixed <= MaximumPayloadBytes
            && p.steps.size() <= (MaximumPayloadBytes - fixed) / 96;
    } catch (...) { return false; }
}

// Every expanded section counts against the existing 32-section budget.
inline bool ExpandedSections(const Definition& p, std::vector<retained_boolean::Disk>& out) noexcept {
    out.clear();
    try {
        if (!Valid(p)) return false;
        for (std::size_t i = 0; i < p.steps.size(); ++i) {
            const auto& t = p.steps[i].operand;
            if (t.kind != analytic_boolean::OperandKind::CylinderRing) {
                out.push_back({t, i, 0});
                continue;
            }
            const auto ring = analytic_boolean_ring::FromOperand(t, p.carrierMetersPerUnit);
            for (std::uint32_t k = 0; k < t.count; ++k) {
                auto disk = analytic_boolean_ring::Expand(ring, k, p.carrierMetersPerUnit);
                if (!disk.identifier) { out.clear(); return false; }
                disk.radius = t.radius;
                out.push_back({disk, i, k});
            }
        }
        return !out.empty() && out.size() <= analytic_boolean_ring::kMaximumExpandedDisks;
    } catch (...) { out.clear(); return false; }
}

// SYPS/1: strict canonical codec with a trailing digest. Steps always use the
// full 96-byte minor-3 operand layout; the fillet tail is the existing minor-4
// retained_fillet codec. Decode requires a bit-exact canonical re-encode.
inline bool Encode(const Definition& p, std::vector<std::uint8_t>& out) noexcept {
    out.clear();
    try {
        if (!Valid(p)) return false;
        std::vector<std::uint8_t> bytes{'S', 'Y', 'P', 'S',
            std::uint8_t(PayloadVersion), 0, 0, 0};
        bytes.reserve(8 + 32 + 8 + 8 + 12 + 4 + 8 + p.steps.size() * 96
            + retained_fillet::EncodedSize(p) + 32);
        using retained_solid::U64;
        using retained_solid::Bits;
        bytes.insert(bytes.end(), p.baseFeature.begin(), p.baseFeature.end());
        bytes.insert(bytes.end(), p.derivedFeature.begin(), p.derivedFeature.end());
        U64(bytes, Bits(p.carrierMetersPerUnit));
        U64(bytes, p.nextOperandID);
        for (std::uint32_t version : {p.buildProfile, p.proofProfile, p.selectorVersion})
            for (unsigned k = 0; k < 4; ++k) bytes.push_back(std::uint8_t(version >> (8 * k)));
        bytes.push_back(std::uint8_t(p.cutSurfaceMaterial));
        bytes.push_back(0); bytes.push_back(0); bytes.push_back(0);
        U64(bytes, p.steps.size());
        for (const auto& step : p.steps) {
            bytes.push_back(std::uint8_t(step.operation));
            bytes.push_back(std::uint8_t(step.operand.kind));
            bytes.push_back(std::uint8_t(step.operand.extent));
            bytes.push_back(std::uint8_t(step.operand.axis));
            U64(bytes, step.operand.identifier);
            for (double value : step.operand.point) U64(bytes, Bits(value));
            U64(bytes, Bits(step.operand.radius));
            U64(bytes, Bits(step.operand.boltCircleRadius));
            for (unsigned k = 0; k < 4; ++k)
                bytes.push_back(std::uint8_t(step.operand.count >> (8 * k)));
            U64(bytes, Bits(step.operand.hostRadiusRatio));
            for (double value : {step.operand.directionAngle, step.operand.halfWidthApex,
                 step.operand.halfWidthMouth, step.operand.length}) U64(bytes, Bits(value));
        }
        std::vector<std::uint8_t> tail;
        if (!retained_fillet::Encode(p, p.carrierMetersPerUnit * 1000, tail)) return false;
        bytes.insert(bytes.end(), tail.begin(), tail.end());
        Digest digest;
        if (!retained_solid::Hash(bytes, digest)) return false;
        bytes.insert(bytes.end(), digest.begin(), digest.end());
        if (bytes.size() > MaximumPayloadBytes) return false;
        out = std::move(bytes);
        return true;
    } catch (...) { out.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& out) noexcept {
    out = {};
    try {
        constexpr std::size_t fixed = 8 + 32 + 8 + 8 + 12 + 4 + 8;
        if (bytes.size() < fixed + 96 + 24 + 32 || bytes.size() > MaximumPayloadBytes
            || std::memcmp(bytes.data(), "SYPS", 4) != 0
            || bytes[4] != std::uint8_t(PayloadVersion) || bytes[5] != 0
            || bytes[6] != 0 || bytes[7] != 0) return false;
        Definition p;
        std::size_t at = 8;
        std::copy_n(bytes.begin() + at, 16, p.baseFeature.begin()); at += 16;
        std::copy_n(bytes.begin() + at, 16, p.derivedFeature.begin()); at += 16;
        auto integer = [&]() {
            std::uint64_t v = 0;
            for (unsigned i = 0; i < 8; ++i) v |= std::uint64_t(bytes[at++]) << (8 * i);
            return v;
        };
        auto scalar = [&]() {
            const auto bits = integer();
            double value;
            std::memcpy(&value, &bits, 8);
            return value;
        };
        p.carrierMetersPerUnit = scalar();
        p.nextOperandID = integer();
        std::uint32_t* versions[] = {&p.buildProfile, &p.proofProfile, &p.selectorVersion};
        for (std::uint32_t* version : versions) {
            std::uint64_t decoded = 0;
            for (unsigned k = 0; k < 4; ++k) decoded |= std::uint64_t(bytes[at++]) << (8 * k);
            if (decoded > UINT32_MAX) return false;
            *version = std::uint32_t(decoded);
        }
        p.cutSurfaceMaterial = CutSurfaceMaterial(bytes[at++]);
        for (unsigned k = 0; k < 3; ++k) if (bytes[at++] != 0) return false;
        const std::uint64_t steps = integer();
        if (!steps || steps > MaximumSteps
            || steps > (bytes.size() - at - 32 - 24) / 96) return false;
        p.steps.reserve(std::size_t(steps));
        for (std::uint64_t i = 0; i < steps; ++i) {
            retained_boolean::Step step;
            step.operation = analytic_boolean::Operation(bytes[at++]);
            step.operand.kind = analytic_boolean::OperandKind(bytes[at++]);
            step.operand.extent = analytic_boolean::Extent(bytes[at++]);
            step.operand.axis = analytic_boolean::Axis(bytes[at++]);
            const auto id = integer();
            if (id > UINT32_MAX) return false;
            step.operand.identifier = std::uint32_t(id);
            for (double& value : step.operand.point) value = scalar();
            step.operand.radius = scalar();
            step.operand.boltCircleRadius = scalar();
            for (unsigned k = 0; k < 4; ++k)
                step.operand.count |= std::uint32_t(bytes[at++]) << (8 * k);
            step.operand.hostRadiusRatio = scalar();
            step.operand.directionAngle = scalar();
            step.operand.halfWidthApex = scalar();
            step.operand.halfWidthMouth = scalar();
            step.operand.length = scalar();
            p.steps.push_back(step);
        }
        retained_fillet::Program tail;
        if (!retained_fillet::Decode(std::vector<std::uint8_t>(bytes.begin() + at,
                bytes.end() - 32), p.carrierMetersPerUnit * 1000, tail)) return false;
        static_cast<retained_fillet::Program&>(p) = std::move(tail);
        std::vector<std::uint8_t> exact;
        if (!Encode(p, exact) || exact != bytes) return false;
        out = std::move(p);
        return true;
    } catch (...) { out = {}; return false; }
}

// Common identity access contains no operand projection or geometry authority.
struct Identity {
    UUID baseFeature{}, derivedFeature{};
    double carrierMetersPerUnit = 0;
};
inline Identity Identities(const Definition& p) {
    return {p.baseFeature, p.derivedFeature, p.carrierMetersPerUnit};
}
} // namespace core3d::retained_program_suffix
