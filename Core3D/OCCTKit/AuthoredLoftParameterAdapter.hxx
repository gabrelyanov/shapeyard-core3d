#pragma once

#include "RectangularLoftPersistence.hxx"
#include "RetainedEdgeTreatmentR2Snapshot.hxx"

#include <cstdint>
#include <cstring>

namespace core3d::authored_loft {

enum class Field : std::uint64_t {
    StationZ = 1ull << 0,
    StationCenterX = 1ull << 1,
    StationCenterY = 1ull << 2,
    FrameTranslationX = 1ull << 3,
    FrameTranslationY = 1ull << 4,
    FrameTranslationZ = 1ull << 5,
    FrameQuaternionX = 1ull << 6,
    FrameQuaternionY = 1ull << 7,
    FrameQuaternionZ = 1ull << 8,
    FrameQuaternionW = 1ull << 9,
    FrameSignedScale = 1ull << 10,
};

enum class Status : std::uint8_t {
    Prepared = 0,
    Unchanged,
    Malformed,
    UnsupportedSource,
    IdentityChanged,
    FieldChanged,
    InvalidDefinition,
};

namespace detail {

inline std::uint64_t Bits(double value) noexcept {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

inline bool SameStationIdentity(const rectangular_loft::Station& left,
                                const rectangular_loft::Station& right) noexcept {
    return left.identifier == right.identifier
        && left.cornerIdentifiers == right.cornerIdentifiers
        && left.correspondence == right.correspondence;
}

inline bool Extract(const retained_edge_treatment::r2::Snapshot& snapshot,
                    rectangular_loft::Definition& definition,
                    retained_recipe::RecipeLocator& locator) noexcept {
    namespace r2 = retained_edge_treatment::r2;
    definition = {};
    locator = {};
    try {
        const auto* retained = std::get_if<r2::RetainedBooleanBase>(&snapshot.source());
        const auto* legacy = retained
            ? std::get_if<r2::LegacyBooleanBase>(&retained->source) : nullptr;
        const auto* binding = std::get_if<r2::BooleanBaseBinding>(&snapshot.definition().base);
        const auto* links = binding
            ? std::get_if<r2::LegacyPrefixBinding>(&binding->prefix) : nullptr;
        if (!legacy || !binding || !links || binding->format != r2::PrefixFormat::SYRS)
            return false;

        std::uint8_t family = 0;
        std::uint32_t schema = 0;
        retained_recipe::UUID sourceFeature{};
        const std::vector<double>* values = nullptr;
        if (const auto* program = std::get_if<retained_boolean::Program>(&legacy->prefix)) {
            family = program->source.family;
            schema = program->source.schema;
            sourceFeature = program->source.sourceFeature;
            values = &program->source.values;
        } else if (const auto* envelope = std::get_if<retained_boolean::Legacy>(&legacy->prefix)) {
            family = envelope->sourceFamily;
            schema = envelope->sourceSchema;
            sourceFeature = envelope->sourceFeature;
            values = &envelope->sourceValues;
        }
        if (family != 3 || schema != loft_persistence::Schema || !values
            || !loft_persistence::Decode(*values, definition)) return false;

        locator.owner = snapshot.definition().owner;
        locator.node = links->rootNode;
        locator.sourceFeature = sourceFeature;
        return retained_recipe::Valid(locator);
    } catch (...) {
        definition = {};
        locator = {};
        return false;
    }
}

} // namespace detail

inline Status Prepare(const retained_edge_treatment::r2::Snapshot& snapshot,
                      std::uint32_t stationIdentifier, Field field,
                      const rectangular_loft::Definition& requested,
                      retained_edge_treatment::r2::Edit& output) noexcept {
    namespace r2 = retained_edge_treatment::r2;
    output = r2::Edit(retained_edge_treatment::SetAmount{});
    try {
        const auto mask = static_cast<std::uint64_t>(field);
        if (!mask || (mask & (mask - 1)) != 0) return Status::Malformed;

        rectangular_loft::Definition original;
        retained_recipe::RecipeLocator locator;
        if (!detail::Extract(snapshot, original, locator)) return Status::UnsupportedSource;
        if (original.loftIdentifier != requested.loftIdentifier
            || original.correspondence != requested.correspondence
            || original.stations.size() != requested.stations.size()
            || detail::Bits(original.dimensionMetersPerUnit)
                != detail::Bits(requested.dimensionMetersPerUnit)
            || bool(original.constructionFrame) != bool(requested.constructionFrame))
            return Status::IdentityChanged;

        const bool stationField = mask <= static_cast<std::uint64_t>(Field::StationCenterY);
        if (stationField == (stationIdentifier == 0)) return Status::Malformed;
        std::uint64_t changed = 0;
        bool stationFound = false;
        for (std::size_t index = 0; index < original.stations.size(); ++index) {
            const auto& before = original.stations[index];
            const auto& after = requested.stations[index];
            if (!detail::SameStationIdentity(before, after)
                || detail::Bits(before.width) != detail::Bits(after.width)
                || detail::Bits(before.depth) != detail::Bits(after.depth))
                return Status::IdentityChanged;
            const bool target = before.identifier == stationIdentifier;
            stationFound = stationFound || target;
            if (detail::Bits(before.z) != detail::Bits(after.z)) {
                if (!target) return Status::FieldChanged;
                changed |= static_cast<std::uint64_t>(Field::StationZ);
            }
            if (detail::Bits(before.centerX) != detail::Bits(after.centerX)) {
                if (!target) return Status::FieldChanged;
                changed |= static_cast<std::uint64_t>(Field::StationCenterX);
            }
            if (detail::Bits(before.centerY) != detail::Bits(after.centerY)) {
                if (!target) return Status::FieldChanged;
                changed |= static_cast<std::uint64_t>(Field::StationCenterY);
            }
        }
        if (stationField && !stationFound) return Status::IdentityChanged;

        if (original.constructionFrame) {
            static constexpr Field fields[] = {
                Field::FrameTranslationX, Field::FrameTranslationY, Field::FrameTranslationZ,
                Field::FrameQuaternionX, Field::FrameQuaternionY, Field::FrameQuaternionZ,
                Field::FrameQuaternionW, Field::FrameSignedScale,
            };
            for (std::size_t index = 0; index < 8; ++index) {
                if (detail::Bits(original.constructionFrame->values[index])
                    != detail::Bits(requested.constructionFrame->values[index]))
                    changed |= static_cast<std::uint64_t>(fields[index]);
            }
        }
        if (!changed) return Status::Unchanged;
        if (changed != mask) return Status::FieldChanged;
        rectangular_loft::Inspection inspection;
        if (rectangular_loft::Inspect(requested, inspection)
            != rectangular_loft::Admission::Accepted) return Status::InvalidDefinition;
        output = r2::Edit(r2::RebuildBooleanInput{locator, requested});
        return Status::Prepared;
    } catch (...) {
        return Status::Malformed;
    }
}

} // namespace core3d::authored_loft
