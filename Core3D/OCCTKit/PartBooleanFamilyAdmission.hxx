#pragma once

// P1b installs one detached evidence signature, not a native command route.
// P1c owns retained document admission after editor/durability qualification.
#include "PartBooleanDefinition.hxx"
#include <array>
#include <cstdint>

namespace core3d::part_boolean::family_admission {

enum class SourceSignature : std::uint8_t {
    ProfileSchema5SingleAxialCapShell = 1,
    AnalyticRectangularPrismPair = 2,
};

struct EvidenceSignature final {
    SourceSignature left;
    InputFamily right;
    std::uint8_t operationMask;
    std::uint32_t buildProfile;
    std::uint32_t proofProfile;
};

inline constexpr std::uint8_t UnionMask = 1U << 0;
inline constexpr std::uint8_t SubtractMask = 1U << 1;
inline constexpr std::uint8_t IntersectMask = 1U << 2;
inline constexpr std::array<EvidenceSignature, 1> InstalledEvidenceSignatures{{
    {SourceSignature::ProfileSchema5SingleAxialCapShell,
     InputFamily::AnalyticRectangularPrism,
     std::uint8_t(UnionMask | SubtractMask | IntersectMask), 1, 1},
}};

inline constexpr bool NativeAdmissionEnabled = false;
inline constexpr bool BooleanRouteInstalled = false;
inline constexpr bool TreatmentFamilyInstalled = false;

// N1 compilation facts are kept separate from promotion/admission. They are
// false until the guarded N1 qualification receipt promotes an isolated delta.
inline constexpr bool OwnerInstalled = false;
inline constexpr bool OperandEditorInstalled = false;
inline constexpr bool RetainedInputsPersistenceInstalled = false;
inline constexpr bool AnalyticNativeAdmissionEnabled = false;
inline constexpr bool AnalyticBooleanRouteInstalled = false;

inline bool EvidenceOperationInstalled(Operation operation) noexcept {
    const std::uint8_t bit = operation == Operation::Union ? UnionMask
        : operation == Operation::Subtract ? SubtractMask
        : operation == Operation::Intersect ? IntersectMask : 0;
    return bit != 0 && (InstalledEvidenceSignatures.front().operationMask & bit) != 0;
}

} // namespace core3d::part_boolean::family_admission
