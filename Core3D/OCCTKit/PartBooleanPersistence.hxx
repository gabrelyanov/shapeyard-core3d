#pragma once
#include "PartBooleanDefinition.hxx"
#include "ProfilePersistence.hxx"
#include "EnclosureParameters.hxx"
#include "RectangularLoftPersistence.hxx"
#include <cstring>
#include <limits>

namespace core3d::part_boolean {
class Writer final {
public:
    std::vector<std::uint8_t> bytes;
    bool valid = true;
    void raw(const std::uint8_t* data, std::size_t count) {
        if (!valid || count > MaximumPayloadBytes - bytes.size()) { valid = false; return; }
        bytes.insert(bytes.end(), data, data + count);
    }
    template<std::size_t N> void raw(const std::array<std::uint8_t, N>& value) { raw(value.data(), N); }
    void integer(std::uint64_t value, unsigned width) {
        if (!valid || width == 0 || width > 8) { valid = false; return; }
        std::uint8_t encoded[8]{};
        for (unsigned index = 0; index < width; ++index) encoded[index] = std::uint8_t(value >> (8 * index));
        raw(encoded, width);
    }
    void scalar(double value) {
        if (!std::isfinite(value)) { valid = false; return; }
        std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits)); integer(bits, 8);
    }
    void text(const std::string& value) {
        if (!ValidText(value) || value.size() > std::numeric_limits<std::uint16_t>::max()) { valid = false; return; }
        integer(value.size(), 2); raw(reinterpret_cast<const std::uint8_t*>(value.data()), value.size());
    }
};

class Reader final {
public:
    Reader(const std::vector<std::uint8_t>& bytes, std::size_t limit) : bytes_(bytes), limit_(limit) {}
    bool raw(std::uint8_t* output, std::size_t count) {
        if (!ok_ || count > limit_ - cursor_) { ok_ = false; return false; }
        std::copy_n(bytes_.begin() + cursor_, count, output); cursor_ += count; return true;
    }
    template<std::size_t N> bool raw(std::array<std::uint8_t, N>& output) { return raw(output.data(), N); }
    bool integer(unsigned width, std::uint64_t& value) {
        value = 0;
        if (!ok_ || width == 0 || width > 8 || width > limit_ - cursor_) { ok_ = false; return false; }
        for (unsigned index = 0; index < width; ++index) value |= std::uint64_t(bytes_[cursor_++]) << (8 * index);
        return true;
    }
    bool scalar(double& value) {
        std::uint64_t bits = 0; if (!integer(8, bits)) return false;
        std::memcpy(&value, &bits, sizeof(value)); return std::isfinite(value);
    }
    bool text(std::string& value) {
        value.clear(); std::uint64_t count = 0;
        if (!integer(2, count) || count > MaximumTextBytes || count > limit_ - cursor_) return false;
        value.assign(reinterpret_cast<const char*>(bytes_.data() + cursor_), std::size_t(count));
        cursor_ += std::size_t(count); return ValidText(value);
    }
    bool complete() const noexcept { return ok_ && cursor_ == limit_; }
    std::size_t cursor() const noexcept { return cursor_; }
    std::size_t remaining() const noexcept { return ok_ ? limit_ - cursor_ : 0; }
    bool take(std::size_t count, std::vector<std::uint8_t>& output) {
        output.clear();
        if (!ok_ || count > limit_ - cursor_) { ok_ = false; return false; }
        output.assign(bytes_.begin() + cursor_, bytes_.begin() + cursor_ + count);
        cursor_ += count;
        return true;
    }
private:
    const std::vector<std::uint8_t>& bytes_;
    std::size_t limit_ = 0, cursor_ = 0;
    bool ok_ = true;
};

inline void WriteMaterial(Writer& writer, const MaterialValue& value) {
    writer.raw(value.identifier); writer.integer(std::uint8_t(value.kind), 1); writer.integer(0, 3);
    for (double scalar : value.baseColorSRGB) writer.scalar(scalar);
    writer.scalar(value.metallic); writer.scalar(value.roughness);
    writer.raw(value.resource); writer.raw(value.resourceDigest);
}
inline bool ReadMaterial(Reader& reader, MaterialValue& value) {
    std::uint64_t kind = 0, reserved = 0;
    if (!reader.raw(value.identifier) || !reader.integer(1, kind)
        || !reader.integer(3, reserved) || reserved != 0) return false;
    value.kind = MaterialKind(kind);
    for (double& scalar : value.baseColorSRGB) if (!reader.scalar(scalar)) return false;
    return reader.scalar(value.metallic) && reader.scalar(value.roughness)
        && reader.raw(value.resource) && reader.raw(value.resourceDigest);
}

inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!Valid(value)) return false;
        Writer writer; writer.raw(reinterpret_cast<const std::uint8_t*>("SYPB"), 4);
        writer.integer(PayloadVersion, 1); writer.integer(0, 3);
        writer.integer(std::uint8_t(value.operation), 1);
        writer.integer(std::uint8_t(value.materialPolicy), 1);
        writer.integer(value.nativeAdmissionEnabled ? 1 : 0, 1); writer.integer(0, 1);
        for (std::uint32_t version : {value.versions.serializer, value.versions.build,
             value.versions.proof, value.versions.selector, value.versions.material}) writer.integer(version, 4);
        for (const InputBinding& input : value.inputs) {
            writer.raw(input.rootNode); writer.raw(input.originalSourceFeature);
            writer.integer(std::uint8_t(input.family), 1);
            writer.integer(input.originallyVisible ? 1 : 0, 1); writer.integer(0, 2);
            writer.raw(input.commitments.geometry); writer.raw(input.commitments.recipe);
            writer.raw(input.commitments.placement); writer.raw(input.commitments.material);
            writer.raw(input.commitments.groups); WriteMaterial(writer, input.originalMaterial);
            writer.text(input.originalName); writer.integer(input.originalGroups.size(), 2);
            for (const std::string& group : input.originalGroups) writer.text(group);
        }
        writer.integer(value.materials.size(), 2); writer.integer(value.regions.size(), 2);
        writer.integer(value.selectors.size(), 2); writer.integer(value.shellAliases.size(), 2);
        for (const MaterialValue& material : value.materials) WriteMaterial(writer, material);
        for (const RegionBinding& region : value.regions) {
            writer.raw(region.region); writer.integer(region.sourceInput, 1);
            writer.integer(std::uint8_t(region.kind), 1); writer.integer(region.materialIndex, 2);
        }
        for (const SelectorBinding& selector : value.selectors) {
            writer.raw(selector.selector); writer.raw(selector.region);
            writer.integer(std::uint8_t(selector.policy), 1); writer.integer(0, 1);
            writer.integer(selector.expectedCardinality, 2);
        }
        for (const ShellAdoptionAlias& alias : value.shellAliases) {
            writer.raw(alias.alias); writer.raw(alias.originalSourceFeature);
            writer.integer(alias.sourceInput, 1); writer.integer(0, 3);
            writer.integer(alias.shellStepIndex, 4);
        }
        if (!writer.valid || writer.bytes.size() > MaximumPayloadBytes - 32) return false;
        Digest digest{}; if (!retained_solid::Hash(writer.bytes, digest)) return false;
        writer.raw(digest); if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output) noexcept {
    output = {};
    try {
        if (bytes.size() < 8 + 4 + 20 + 32 || bytes.size() > MaximumPayloadBytes
            || std::memcmp(bytes.data(), "SYPB\2\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!retained_solid::Hash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (expected != actual) return false;
        Reader reader(bytes, bytes.size() - 32); std::array<std::uint8_t, 8> prefix{};
        std::uint64_t operation = 0, materialPolicy = 0, admission = 0, reserved = 0;
        Definition value;
        if (!reader.raw(prefix) || !reader.integer(1, operation) || !reader.integer(1, materialPolicy)
            || !reader.integer(1, admission) || admission != 0
            || !reader.integer(1, reserved) || reserved != 0) return false;
        value.operation = Operation(operation); value.materialPolicy = MaterialPolicy(materialPolicy);
        std::uint32_t* versions[] = {&value.versions.serializer, &value.versions.build,
            &value.versions.proof, &value.versions.selector, &value.versions.material};
        for (std::uint32_t* version : versions) {
            std::uint64_t decoded = 0;
            if (!reader.integer(4, decoded) || decoded > UINT32_MAX) return false;
            *version = std::uint32_t(decoded);
        }
        for (InputBinding& input : value.inputs) {
            std::uint64_t family = 0, visible = 0, groupCount = 0;
            if (!reader.raw(input.rootNode) || !reader.raw(input.originalSourceFeature)
                || !reader.integer(1, family) || !reader.integer(1, visible) || visible > 1
                || !reader.integer(2, reserved) || reserved != 0
                || !reader.raw(input.commitments.geometry) || !reader.raw(input.commitments.recipe)
                || !reader.raw(input.commitments.placement) || !reader.raw(input.commitments.material)
                || !reader.raw(input.commitments.groups) || !ReadMaterial(reader, input.originalMaterial)
                || !reader.text(input.originalName) || !reader.integer(2, groupCount)
                || groupCount > MaximumGroupsPerInput) return false;
            input.family = InputFamily(family); input.originallyVisible = visible != 0;
            input.originalGroups.reserve(std::size_t(groupCount));
            for (std::uint64_t index = 0; index < groupCount; ++index) {
                std::string group; if (!reader.text(group)) return false;
                input.originalGroups.push_back(std::move(group));
            }
        }
        std::uint64_t materialCount = 0, regionCount = 0, selectorCount = 0, aliasCount = 0;
        if (!reader.integer(2, materialCount) || materialCount == 0 || materialCount > MaximumMaterials
            || !reader.integer(2, regionCount) || regionCount == 0 || regionCount > MaximumRegions
            || !reader.integer(2, selectorCount) || selectorCount > MaximumSelectors
            || !reader.integer(2, aliasCount) || aliasCount != 1 || aliasCount > MaximumAliases) return false;
        value.materials.resize(std::size_t(materialCount));
        for (MaterialValue& material : value.materials) if (!ReadMaterial(reader, material)) return false;
        value.regions.resize(std::size_t(regionCount));
        for (RegionBinding& region : value.regions) {
            std::uint64_t input = 0, kind = 0, material = 0;
            if (!reader.raw(region.region) || !reader.integer(1, input) || input > UINT8_MAX
                || !reader.integer(1, kind) || !reader.integer(2, material) || material > UINT16_MAX) return false;
            region.sourceInput = std::uint8_t(input); region.kind = RegionKind(kind);
            region.materialIndex = std::uint16_t(material);
        }
        value.selectors.resize(std::size_t(selectorCount));
        for (SelectorBinding& selector : value.selectors) {
            std::uint64_t policy = 0, cardinality = 0;
            if (!reader.raw(selector.selector) || !reader.raw(selector.region)
                || !reader.integer(1, policy) || !reader.integer(1, reserved) || reserved != 0
                || !reader.integer(2, cardinality) || cardinality > UINT16_MAX) return false;
            selector.policy = SelectorPolicy(policy); selector.expectedCardinality = std::uint16_t(cardinality);
        }
        value.shellAliases.resize(std::size_t(aliasCount));
        for (ShellAdoptionAlias& alias : value.shellAliases) {
            std::uint64_t input = 0, step = 0;
            if (!reader.raw(alias.alias) || !reader.raw(alias.originalSourceFeature)
                || !reader.integer(1, input) || input > UINT8_MAX
                || !reader.integer(3, reserved) || reserved != 0
                || !reader.integer(4, step) || step > UINT32_MAX) return false;
            alias.sourceInput = std::uint8_t(input); alias.shellStepIndex = std::uint32_t(step);
        }
        std::vector<std::uint8_t> canonical;
        if (!reader.complete() || !Valid(value) || !Encode(value, canonical) || canonical != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

inline bool EncodeAnalytic(const AnalyticDefinition& value,
                           std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!Valid(value)) return false;
        Writer writer;
        writer.raw(reinterpret_cast<const std::uint8_t*>("SYPB"), 4);
        writer.integer(AnalyticPayloadVersion, 1); writer.integer(0, 3);
        writer.integer(std::uint8_t(value.operation), 1);
        writer.integer(std::uint8_t(value.materialPolicy), 1);
        writer.integer(0, 2); // persisted admission and reserved byte
        for (std::uint32_t version : {value.versions.serializer, value.versions.build,
             value.versions.proof, value.versions.selector, value.versions.material})
            writer.integer(version, 4);
        for (const AnalyticPrismInput& input : value.inputs) {
            writer.raw(input.rootNode); writer.raw(input.originalSourceFeature);
            writer.integer(input.originallyVisible ? 1 : 0, 1); writer.integer(0, 3);
            writer.raw(input.commitments.geometry); writer.raw(input.commitments.recipe);
            writer.raw(input.commitments.placement); writer.raw(input.commitments.material);
            writer.raw(input.commitments.groups);
            for (double scalar : input.dimensions) writer.scalar(scalar);
            for (double scalar : input.translation) writer.scalar(scalar);
            for (double scalar : input.rotationXYZW) writer.scalar(scalar);
            writer.scalar(input.metersPerUnit); WriteMaterial(writer, input.originalMaterial);
            writer.text(input.originalName); writer.integer(input.originalGroups.size(), 2);
            for (const auto& group : input.originalGroups) writer.text(group);
        }
        if (!writer.valid || writer.bytes.size() > MaximumPayloadBytes - 32) return false;
        Digest digest{};
        if (!retained_solid::Hash(writer.bytes, digest)) return false;
        writer.raw(digest);
        if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool DecodeAnalytic(const std::vector<std::uint8_t>& bytes,
                           AnalyticDefinition& output) noexcept {
    output = {};
    try {
        if (bytes.size() < 8 + 4 + 20 + 32 || bytes.size() > MaximumPayloadBytes
            || std::memcmp(bytes.data(), "SYPB\1\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!retained_solid::Hash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin());
        if (expected != actual) return false;
        Reader reader(bytes, bytes.size() - 32);
        std::array<std::uint8_t, 8> prefix{};
        std::uint64_t operation = 0, policy = 0, admission = 0, reserved = 0;
        AnalyticDefinition value;
        if (!reader.raw(prefix) || !reader.integer(1, operation)
            || !reader.integer(1, policy) || !reader.integer(1, admission)
            || admission != 0 || !reader.integer(1, reserved) || reserved != 0) return false;
        value.operation = Operation(operation); value.materialPolicy = MaterialPolicy(policy);
        std::uint32_t* versions[] = {&value.versions.serializer, &value.versions.build,
            &value.versions.proof, &value.versions.selector, &value.versions.material};
        for (auto* version : versions) {
            std::uint64_t decoded = 0;
            if (!reader.integer(4, decoded) || decoded > UINT32_MAX) return false;
            *version = std::uint32_t(decoded);
        }
        for (AnalyticPrismInput& input : value.inputs) {
            std::uint64_t visible = 0, groups = 0;
            if (!reader.raw(input.rootNode) || !reader.raw(input.originalSourceFeature)
                || !reader.integer(1, visible) || visible > 1
                || !reader.integer(3, reserved) || reserved != 0
                || !reader.raw(input.commitments.geometry)
                || !reader.raw(input.commitments.recipe)
                || !reader.raw(input.commitments.placement)
                || !reader.raw(input.commitments.material)
                || !reader.raw(input.commitments.groups)) return false;
            input.originallyVisible = visible != 0;
            for (double& scalar : input.dimensions) if (!reader.scalar(scalar)) return false;
            for (double& scalar : input.translation) if (!reader.scalar(scalar)) return false;
            for (double& scalar : input.rotationXYZW) if (!reader.scalar(scalar)) return false;
            if (!reader.scalar(input.metersPerUnit)
                || !ReadMaterial(reader, input.originalMaterial)
                || !reader.text(input.originalName) || !reader.integer(2, groups)
                || groups > MaximumGroupsPerInput) return false;
            input.originalGroups.reserve(std::size_t(groups));
            for (std::uint64_t index = 0; index < groups; ++index) {
                std::string group;
                if (!reader.text(group)) return false;
                input.originalGroups.push_back(std::move(group));
            }
        }
        std::vector<std::uint8_t> canonical;
        if (!reader.complete() || !Valid(value) || !EncodeAnalytic(value, canonical)
            || canonical != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

inline bool SameScalarBits(const std::vector<double>& left,
                           const std::vector<double>& right) noexcept {
    if (left.size() != right.size()) return false;
    for (std::size_t index = 0; index < left.size(); ++index)
        if (std::memcmp(&left[index], &right[index], sizeof(double)) != 0) return false;
    return true;
}

inline bool ValidNativeRecipe(const RecipeInput& input) noexcept {
    try {
        std::vector<double> exact;
        double recipeUnit = 0;
        if (input.family == RecipeInputFamily::Profile) {
            profile::Parameters decoded;
            if (input.recipeSchema < 1 || input.recipeSchema > 5
                || !profile::Decode(input.recipeScalars, decoded)
                || profile::SchemaFor(decoded) != int(input.recipeSchema)
                || !profile::Encode(decoded, exact)) return false;
            recipeUnit = decoded.metersPerUnit;
        } else if (input.family == RecipeInputFamily::Enclosure) {
            enclosure::Parameters decoded;
            if (input.recipeSchema < 1 || input.recipeSchema > 2
                || !enclosure::Decode(int(input.recipeSchema), input.recipeScalars, decoded)
                || !enclosure::Encode(decoded, exact)) return false;
            recipeUnit = decoded.metersPerUnit;
        } else if (input.family == RecipeInputFamily::RectangularLoft) {
            rectangular_loft::Definition decoded;
            if (input.recipeSchema != std::uint32_t(loft_persistence::Schema)
                || !loft_persistence::Decode(input.recipeScalars, decoded)
                || !loft_persistence::Encode(decoded, exact)) return false;
            recipeUnit = decoded.dimensionMetersPerUnit;
        } else return false;
        return SameScalarBits(input.recipeScalars, exact)
            && std::memcmp(&recipeUnit, &input.sourceMetersPerUnit, sizeof(double)) == 0;
    } catch (...) { return false; }
}

inline bool ValidRecipeDefinition(const RecipeDefinition& value) noexcept {
    return Valid(value) && ValidNativeRecipe(value.inputs[0])
        && ValidNativeRecipe(value.inputs[1]);
}

inline bool RecipePlacementDigest(const RecipeInput& input, Digest& output) noexcept {
    output.fill(0);
    try {
        Writer writer;
        static constexpr char domain[] = "SYPB3/placement";
        writer.raw(reinterpret_cast<const std::uint8_t*>(domain), sizeof(domain));
        for (double scalar : input.inputToCarrier) writer.scalar(scalar);
        writer.scalar(input.sourceMetersPerUnit);
        writer.scalar(input.carrierMetersPerUnit);
        return writer.valid && retained_solid::Hash(writer.bytes, output);
    } catch (...) { output.fill(0); return false; }
}

inline bool RecipeMaterialDigest(const MaterialValue& material, Digest& output) noexcept {
    output.fill(0);
    try {
        Writer writer;
        static constexpr char domain[] = "SYPB3/material";
        writer.raw(reinterpret_cast<const std::uint8_t*>(domain), sizeof(domain));
        WriteMaterial(writer, material);
        return writer.valid && retained_solid::Hash(writer.bytes, output);
    } catch (...) { output.fill(0); return false; }
}

inline bool RecipeGroupsDigest(const std::vector<std::string>& groups,
                               Digest& output) noexcept {
    output.fill(0);
    try {
        if (groups.size() > MaximumGroupsPerInput) return false;
        Writer writer;
        static constexpr char domain[] = "SYPB3/groups";
        writer.raw(reinterpret_cast<const std::uint8_t*>(domain), sizeof(domain));
        writer.integer(groups.size(), 2);
        for (const auto& group : groups) writer.text(group);
        return writer.valid && retained_solid::Hash(writer.bytes, output);
    } catch (...) { output.fill(0); return false; }
}

inline bool EncodeRecipe(const RecipeDefinition& value,
                         std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!ValidRecipeDefinition(value)) return false;
        Writer writer;
        writer.raw(reinterpret_cast<const std::uint8_t*>("SYPB"), 4);
        writer.integer(RecipePayloadVersion, 1); writer.integer(0, 3);
        writer.integer(std::uint8_t(value.operation), 1);
        writer.integer(std::uint8_t(value.materialPolicy), 1);
        writer.integer(0, 2); // admission and reserved
        for (std::uint32_t version : {value.versions.serializer, value.versions.build,
             value.versions.proof, value.versions.selector, value.versions.material})
            writer.integer(version, 4);
        writer.raw(value.owner.document); writer.raw(value.owner.entity);
        writer.raw(value.owner.definition); writer.raw(value.booleanNode);
        writer.raw(value.feature);
        for (const RecipeInput& input : value.inputs) {
            writer.integer(std::uint8_t(input.role), 1);
            writer.integer(std::uint8_t(input.family), 1);
            writer.integer(input.originallyVisible ? 1 : 0, 1);
            writer.integer(0, 1);
            writer.raw(input.sourceNode); writer.integer(input.sourceLocalID, 8);
            writer.raw(input.original.document); writer.raw(input.original.entity);
            writer.raw(input.original.definition); writer.raw(input.original.sourceFeature);
            writer.integer(input.shapeSlot, 4); writer.integer(input.recipeSchema, 4);
            writer.scalar(input.sourceMetersPerUnit); writer.scalar(input.carrierMetersPerUnit);
            for (double scalar : input.inputToCarrier) writer.scalar(scalar);
            writer.raw(input.commitments.geometry); writer.raw(input.commitments.recipe);
            writer.raw(input.commitments.placement); writer.raw(input.commitments.material);
            writer.raw(input.commitments.groups); WriteMaterial(writer, input.originalMaterial);
            writer.text(input.originalName); writer.integer(input.originalGroups.size(), 2);
            for (const auto& group : input.originalGroups) writer.text(group);
            writer.integer(input.recipeScalars.size(), 4);
            for (double scalar : input.recipeScalars) writer.scalar(scalar);
        }
        if (!writer.valid || writer.bytes.size() > MaximumPayloadBytes - 32) return false;
        Digest seal{};
        if (!retained_solid::Hash(writer.bytes, seal)) return false;
        writer.raw(seal);
        if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool DecodeRecipe(const std::vector<std::uint8_t>& bytes,
                         RecipeDefinition& output) noexcept {
    output = {};
    try {
        if (bytes.size() < 8 + 4 + 20 + 48 + 32 + 32
            || bytes.size() > MaximumPayloadBytes
            || std::memcmp(bytes.data(), "SYPB\3\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!retained_solid::Hash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin());
        if (expected != actual) return false;
        Reader reader(bytes, bytes.size() - 32);
        std::array<std::uint8_t, 8> prefix{};
        std::uint64_t operation = 0, policy = 0, admission = 0, reserved = 0;
        RecipeDefinition value;
        if (!reader.raw(prefix) || !reader.integer(1, operation)
            || !reader.integer(1, policy) || !reader.integer(1, admission)
            || admission != 0 || !reader.integer(1, reserved) || reserved != 0) return false;
        value.operation = Operation(operation); value.materialPolicy = MaterialPolicy(policy);
        std::uint32_t* versions[] = {&value.versions.serializer, &value.versions.build,
            &value.versions.proof, &value.versions.selector, &value.versions.material};
        for (auto* version : versions) {
            std::uint64_t decoded = 0;
            if (!reader.integer(4, decoded) || decoded > UINT32_MAX) return false;
            *version = std::uint32_t(decoded);
        }
        if (!reader.raw(value.owner.document) || !reader.raw(value.owner.entity)
            || !reader.raw(value.owner.definition) || !reader.raw(value.booleanNode)
            || !reader.raw(value.feature)) return false;
        for (RecipeInput& input : value.inputs) {
            std::uint64_t role = 0, family = 0, visible = 0, localID = 0;
            std::uint64_t slot = 0, schema = 0, groupCount = 0, scalarCount = 0;
            if (!reader.integer(1, role) || !reader.integer(1, family)
                || !reader.integer(1, visible) || visible > 1
                || !reader.integer(1, reserved) || reserved != 0
                || !reader.raw(input.sourceNode) || !reader.integer(8, localID)
                || !reader.raw(input.original.document) || !reader.raw(input.original.entity)
                || !reader.raw(input.original.definition) || !reader.raw(input.original.sourceFeature)
                || !reader.integer(4, slot) || slot > UINT32_MAX
                || !reader.integer(4, schema) || schema > UINT32_MAX
                || !reader.scalar(input.sourceMetersPerUnit)
                || !reader.scalar(input.carrierMetersPerUnit)) return false;
            input.role = RecipeInputRole(role); input.family = RecipeInputFamily(family);
            input.originallyVisible = visible != 0; input.sourceLocalID = localID;
            input.shapeSlot = std::uint32_t(slot); input.recipeSchema = std::uint32_t(schema);
            for (double& scalar : input.inputToCarrier) if (!reader.scalar(scalar)) return false;
            if (!reader.raw(input.commitments.geometry) || !reader.raw(input.commitments.recipe)
                || !reader.raw(input.commitments.placement) || !reader.raw(input.commitments.material)
                || !reader.raw(input.commitments.groups) || !ReadMaterial(reader, input.originalMaterial)
                || !reader.text(input.originalName) || !reader.integer(2, groupCount)
                || groupCount > MaximumGroupsPerInput) return false;
            input.originalGroups.reserve(std::size_t(groupCount));
            for (std::uint64_t index = 0; index < groupCount; ++index) {
                std::string group;
                if (!reader.text(group)) return false;
                input.originalGroups.push_back(std::move(group));
            }
            if (!reader.integer(4, scalarCount)
                || scalarCount == 0 || scalarCount > std::size_t(profile::MaximumScalars)) return false;
            input.recipeScalars.reserve(std::size_t(scalarCount));
            for (std::uint64_t index = 0; index < scalarCount; ++index) {
                double scalar = 0;
                if (!reader.scalar(scalar)) return false;
                input.recipeScalars.push_back(scalar);
            }
        }
        std::vector<std::uint8_t> canonical;
        if (!reader.complete() || !ValidRecipeDefinition(value)
            || !EncodeRecipe(value, canonical) || canonical != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

// SYPB/4 uses u32 lengths and u16 bounded counts throughout.  Nested writers
// still share the same 16 KiB ceiling; a nested record cannot reset the
// feature budget because the parent checks every length before insertion.
inline void WriteText32(Writer& writer, const std::string& value) {
    if (!ValidArchiveText(value, true) || value.size() > UINT32_MAX) {
        writer.valid = false; return;
    }
    writer.integer(value.size(), 4);
    writer.raw(reinterpret_cast<const std::uint8_t*>(value.data()), value.size());
}

inline bool ReadText32(Reader& reader, std::string& value) {
    std::uint64_t count = 0;
    if (!reader.integer(4, count) || count > MaximumTextBytes
        || count > reader.remaining()) return false;
    std::vector<std::uint8_t> bytes;
    if (!reader.take(std::size_t(count), bytes)) return false;
    value.assign(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    return ValidArchiveText(value, true);
}

inline void WriteUTF16(Writer& writer, const UTF16Name& value) {
    writer.integer(value.attributePresent ? 1 : 0, 1);
    writer.integer(value.codeUnits.size(), 4);
    for (std::uint16_t unit : value.codeUnits) writer.integer(unit, 2);
}

inline bool ReadUTF16(Reader& reader, UTF16Name& value) {
    std::uint64_t present = 0, count = 0;
    if (!reader.integer(1, present) || present > 1 || !reader.integer(4, count)
        || count > MaximumUTF16CodeUnits || count > reader.remaining() / 2) return false;
    value.attributePresent = present != 0;
    value.codeUnits.reserve(std::size_t(count));
    for (std::uint64_t index = 0; index < count; ++index) {
        std::uint64_t unit = 0;
        if (!reader.integer(2, unit)) return false;
        value.codeUnits.push_back(std::uint16_t(unit));
    }
    return Valid(value);
}

inline void WriteOccurrence(Writer& writer, const OccurrenceState& value) {
    writer.integer(value.encoding, 1); writer.integer(value.presenceMask, 1);
    writer.integer(0, 2);
    for (std::size_t index = 0; index < value.values.size(); ++index)
        if (value.presenceMask & (1U << index)) writer.scalar(value.values[index]);
    for (double scalar : value.derivedMatrix) writer.scalar(scalar);
}

inline bool ReadOccurrence(Reader& reader, OccurrenceState& value) {
    std::uint64_t encoding = 0, mask = 0, reserved = 0;
    if (!reader.integer(1, encoding) || !reader.integer(1, mask)
        || !reader.integer(2, reserved) || reserved != 0) return false;
    value.encoding = std::uint8_t(encoding); value.presenceMask = std::uint8_t(mask);
    for (std::size_t index = 0; index < value.values.size(); ++index)
        if ((value.presenceMask & (1U << index)) && !reader.scalar(value.values[index])) return false;
    for (double& scalar : value.derivedMatrix) if (!reader.scalar(scalar)) return false;
    return Valid(value);
}

inline void WriteI32(Writer& writer, std::int32_t value) {
    writer.integer(std::uint32_t(value), 4);
}
inline bool ReadI32(Reader& reader, std::int32_t& value) {
    std::uint64_t raw = 0; if (!reader.integer(4, raw)) return false;
    value = static_cast<std::int32_t>(std::uint32_t(raw)); return true;
}

inline void WriteAppearance(Writer& writer, const AppearanceState& value) {
    std::uint8_t flags = value.legacyMaterialPresent ? 1 : 0;
    flags |= value.legacyColorPresent ? 2 : 0;
    flags |= value.localPBRMarkerPresent ? 4 : 0;
    writer.integer(flags, 1); writer.integer(0, 3);
    if (value.legacyMaterialPresent) WriteI32(writer, value.legacyMaterial);
    if (value.legacyColorPresent) WriteI32(writer, value.legacyColor);
    writer.integer(value.material ? 1 : 0, 1);
    if (value.material) writer.integer(*value.material, 2);
}

inline bool ReadAppearance(Reader& reader, AppearanceState& value) {
    std::uint64_t flags = 0, reserved = 0, present = 0, ordinal = 0;
    if (!reader.integer(1, flags) || (flags & ~7U) != 0
        || !reader.integer(3, reserved) || reserved != 0) return false;
    value.legacyMaterialPresent = flags & 1;
    value.legacyColorPresent = flags & 2;
    value.localPBRMarkerPresent = flags & 4;
    if (value.legacyMaterialPresent && !ReadI32(reader, value.legacyMaterial)) return false;
    if (value.legacyColorPresent && !ReadI32(reader, value.legacyColor)) return false;
    if (!reader.integer(1, present) || present > 1) return false;
    if (present) {
        if (!reader.integer(2, ordinal)) return false;
        value.material = std::uint16_t(ordinal);
    }
    return true;
}

inline bool WriteMetadata(const CompleteMetadata& value,
                          std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        Writer writer;
        WriteUTF16(writer, value.name); WriteOccurrence(writer, value.occurrence);
        WriteAppearance(writer, value.appearance);
        writer.integer(value.groups.size(), 2);
        for (std::uint16_t reference : value.groups) writer.integer(reference, 2);
        writer.integer(value.invisibleAttributePresent ? 1 : 0, 1);
        writer.integer(value.layerLinkPresent ? 1 : 0, 1);
        writer.integer(value.layers.size(), 2);
        for (std::uint16_t reference : value.layers) writer.integer(reference, 2);
        if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool ReadMetadata(const std::vector<std::uint8_t>& bytes,
                         CompleteMetadata& value) noexcept {
    value = {};
    try {
        Reader reader(bytes, bytes.size());
        std::uint64_t count = 0, reference = 0, invisible = 0, linked = 0;
        if (!ReadUTF16(reader, value.name) || !ReadOccurrence(reader, value.occurrence)
            || !ReadAppearance(reader, value.appearance)
            || !reader.integer(2, count) || count > MaximumMetadataRows
            || count > reader.remaining() / 2) return false;
        value.groups.reserve(std::size_t(count));
        for (std::uint64_t index = 0; index < count; ++index) {
            if (!reader.integer(2, reference)) return false;
            value.groups.push_back(std::uint16_t(reference));
        }
        if (!reader.integer(1, invisible) || invisible > 1
            || !reader.integer(1, linked) || linked > 1
            || !reader.integer(2, count) || count > MaximumMetadataRows
            || count > reader.remaining() / 2) return false;
        value.invisibleAttributePresent = invisible != 0;
        value.layerLinkPresent = linked != 0;
        value.layers.reserve(std::size_t(count));
        for (std::uint64_t index = 0; index < count; ++index) {
            if (!reader.integer(2, reference)) return false;
            value.layers.push_back(std::uint16_t(reference));
        }
        return reader.complete();
    } catch (...) { value = {}; return false; }
}

inline void WriteSized(Writer& writer, const std::vector<std::uint8_t>& bytes) {
    writer.integer(bytes.size(), 4); writer.raw(bytes.data(), bytes.size());
}
inline bool ReadSized(Reader& reader, std::vector<std::uint8_t>& bytes,
                      std::size_t maximum = MaximumPayloadBytes) {
    std::uint64_t count = 0;
    return reader.integer(4, count) && count <= maximum
        && count <= reader.remaining() && reader.take(std::size_t(count), bytes);
}

inline bool WriteMaterialRow(const ScalarMaterialRow& value,
                             std::vector<std::uint8_t>& output) noexcept {
    Writer writer; writer.integer(value.ordinal, 2); WriteText32(writer, value.labelEntry);
    WriteI32(writer, value.faceCulling); WriteI32(writer, value.alphaMode);
    writer.scalar(value.alphaCutoff); writer.integer(value.pbrDefined ? 1 : 0, 1);
    writer.integer(value.commonDefined ? 1 : 0, 1); writer.integer(0, 2);
    for (double scalar : value.pbr) writer.scalar(scalar);
    for (double scalar : value.common) writer.scalar(scalar);
    if (!writer.valid) return false; output = std::move(writer.bytes); return true;
}

inline bool ReadMaterialRow(const std::vector<std::uint8_t>& bytes,
                            ScalarMaterialRow& value) noexcept {
    Reader reader(bytes, bytes.size()); std::uint64_t ordinal = 0, pbr = 0, common = 0, reserved = 0;
    if (!reader.integer(2, ordinal) || !ReadText32(reader, value.labelEntry)
        || !ReadI32(reader, value.faceCulling) || !ReadI32(reader, value.alphaMode)
        || !reader.scalar(value.alphaCutoff) || !reader.integer(1, pbr) || pbr > 1
        || !reader.integer(1, common) || common > 1 || !reader.integer(2, reserved)
        || reserved != 0) return false;
    value.ordinal = std::uint16_t(ordinal); value.pbrDefined = pbr != 0;
    value.commonDefined = common != 0;
    for (double& scalar : value.pbr) if (!reader.scalar(scalar)) return false;
    for (double& scalar : value.common) if (!reader.scalar(scalar)) return false;
    return reader.complete();
}

inline bool WriteGroupRow(const GroupRow& value,
                          std::vector<std::uint8_t>& output) noexcept {
    Writer writer; writer.integer(value.ordinal, 2); WriteText32(writer, value.identifier);
    WriteUTF16(writer, value.name); writer.integer(value.originPresent ? 1 : 0, 1);
    if (value.originPresent) for (double scalar : value.origin) writer.scalar(scalar);
    writer.integer(value.members.size(), 2);
    for (const auto& member : value.members) {
        writer.raw(member.document); writer.raw(member.entity); writer.raw(member.definition);
    }
    if (!writer.valid) return false; output = std::move(writer.bytes); return true;
}

inline bool ReadGroupRow(const std::vector<std::uint8_t>& bytes,
                         GroupRow& value) noexcept {
    Reader reader(bytes, bytes.size()); std::uint64_t ordinal = 0, present = 0, count = 0;
    if (!reader.integer(2, ordinal) || !ReadText32(reader, value.identifier)
        || !ReadUTF16(reader, value.name) || !reader.integer(1, present) || present > 1)
        return false;
    value.ordinal = std::uint16_t(ordinal); value.originPresent = present != 0;
    if (value.originPresent)
        for (double& scalar : value.origin) if (!reader.scalar(scalar)) return false;
    if (!reader.integer(2, count) || count == 0 || count > MaximumMetadataRows
        || count > reader.remaining() / 48) return false;
    value.members.resize(std::size_t(count));
    for (auto& member : value.members)
        if (!reader.raw(member.document) || !reader.raw(member.entity)
            || !reader.raw(member.definition)) return false;
    return reader.complete();
}

inline bool WriteLayerRow(const LayerRow& value,
                          std::vector<std::uint8_t>& output) noexcept {
    Writer writer; writer.integer(value.ordinal, 2); WriteText32(writer, value.labelEntry);
    WriteUTF16(writer, value.name); writer.integer(value.invisibleAttributePresent ? 1 : 0, 1);
    if (!writer.valid) return false; output = std::move(writer.bytes); return true;
}

inline bool ReadLayerRow(const std::vector<std::uint8_t>& bytes,
                         LayerRow& value) noexcept {
    Reader reader(bytes, bytes.size()); std::uint64_t ordinal = 0, invisible = 0;
    if (!reader.integer(2, ordinal) || !ReadText32(reader, value.labelEntry)
        || !ReadUTF16(reader, value.name) || !reader.integer(1, invisible)
        || invisible > 1 || !reader.complete()) return false;
    value.ordinal = std::uint16_t(ordinal);
    value.invisibleAttributePresent = invisible != 0; return true;
}

inline void WriteTables(Writer& writer,
                        const std::vector<ScalarMaterialRow>& materials,
                        const std::vector<GroupRow>& groups,
                        const std::vector<LayerRow>& layers) {
    writer.integer(materials.size(), 2);
    for (const auto& row : materials) { std::vector<std::uint8_t> bytes;
        if (!WriteMaterialRow(row, bytes)) writer.valid = false; else WriteSized(writer, bytes); }
    writer.integer(groups.size(), 2);
    for (const auto& row : groups) { std::vector<std::uint8_t> bytes;
        if (!WriteGroupRow(row, bytes)) writer.valid = false; else WriteSized(writer, bytes); }
    writer.integer(layers.size(), 2);
    for (const auto& row : layers) { std::vector<std::uint8_t> bytes;
        if (!WriteLayerRow(row, bytes)) writer.valid = false; else WriteSized(writer, bytes); }
}

inline bool ReadTables(Reader& reader, std::vector<ScalarMaterialRow>& materials,
                       std::vector<GroupRow>& groups, std::vector<LayerRow>& layers) {
    std::uint64_t count = 0; std::vector<std::uint8_t> bytes;
    if (!reader.integer(2, count) || count > MaximumMetadataRows) return false;
    materials.resize(std::size_t(count));
    for (auto& row : materials) if (!ReadSized(reader, bytes) || !ReadMaterialRow(bytes, row)) return false;
    if (!reader.integer(2, count) || count > MaximumMetadataRows) return false;
    groups.resize(std::size_t(count));
    for (auto& row : groups) if (!ReadSized(reader, bytes) || !ReadGroupRow(bytes, row)) return false;
    if (!reader.integer(2, count) || count > MaximumMetadataRows) return false;
    layers.resize(std::size_t(count));
    for (auto& row : layers) if (!ReadSized(reader, bytes) || !ReadLayerRow(bytes, row)) return false;
    return true;
}

inline void WriteCommitments(Writer& writer, const SignatureCommitments& value) {
    writer.raw(value.geometry); writer.raw(value.recipe); writer.raw(value.placement);
    writer.raw(value.material); writer.raw(value.groups);
}
inline bool ReadCommitments(Reader& reader, SignatureCommitments& value) {
    return reader.raw(value.geometry) && reader.raw(value.recipe)
        && reader.raw(value.placement) && reader.raw(value.material)
        && reader.raw(value.groups);
}

inline bool EncodeOriginalStateBlock(const OriginalStateBlock& value,
                                     std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        Writer writer; writer.integer(value.schema, 2);
        writer.integer(std::uint8_t(value.creationOperation), 1); writer.integer(2, 1);
        for (const OriginalInput& input : value.inputs) {
            writer.integer(std::uint8_t(input.role), 1); writer.integer(0, 3);
            writer.raw(input.sourceNode); writer.integer(input.sourceLocalID, 8);
            writer.integer(input.shapeSlot, 4); writer.raw(input.original.document);
            writer.raw(input.original.entity); writer.raw(input.original.definition);
            writer.raw(input.original.sourceFeature);
            writer.integer(std::uint8_t(input.recipeKind), 1); writer.integer(0, 3);
            writer.integer(input.recipeSchema, 4); WriteSized(writer, input.recipeBytes);
            writer.scalar(input.sourceMetersPerUnit); writer.scalar(input.carrierMetersPerUnit);
            for (double scalar : input.sourceRootToCarrier) writer.scalar(scalar);
            if (input.recipeKind != RecipeInputFamily::OrdinarySource) {
                for (double scalar : input.projection.dimensions) writer.scalar(scalar);
                for (double scalar : input.projection.translation) writer.scalar(scalar);
                for (double scalar : input.projection.rotationXYZW) writer.scalar(scalar);
            }
            WriteCommitments(writer, input.commitments);
            std::vector<std::uint8_t> metadata;
            if (!WriteMetadata(input.metadata, metadata)) writer.valid = false;
            else WriteSized(writer, metadata);
        }
        WriteTables(writer, value.materials, value.groups, value.layers);
        if (!writer.valid || writer.bytes.size() > MaximumPayloadBytes - 32) return false;
        Digest digest{}; if (!retained_solid::Hash(writer.bytes, digest)) return false;
        writer.raw(digest); if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool DecodeOriginalStateBlock(const std::vector<std::uint8_t>& bytes,
                                     OriginalStateBlock& value) noexcept {
    value = {};
    try {
        if (bytes.size() < 4 + 32 || bytes.size() > MaximumPayloadBytes) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!retained_solid::Hash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (expected != actual) return false;
        Reader reader(bytes, bytes.size() - 32);
        std::uint64_t schema = 0, operation = 0, count = 0, role = 0, reserved = 0;
        if (!reader.integer(2, schema) || schema != 1 || !reader.integer(1, operation)
            || !reader.integer(1, count) || count != 2) return false;
        value.schema = std::uint16_t(schema); value.creationOperation = Operation(operation);
        for (OriginalInput& input : value.inputs) {
            std::uint64_t localID = 0, slot = 0, kind = 0, recipeSchema = 0;
            if (!reader.integer(1, role) || !reader.integer(3, reserved) || reserved != 0
                || !reader.raw(input.sourceNode) || !reader.integer(8, localID)
                || !reader.integer(4, slot) || slot > UINT32_MAX
                || !reader.raw(input.original.document) || !reader.raw(input.original.entity)
                || !reader.raw(input.original.definition) || !reader.raw(input.original.sourceFeature)
                || !reader.integer(1, kind) || !reader.integer(3, reserved) || reserved != 0
                || !reader.integer(4, recipeSchema) || recipeSchema > UINT32_MAX
                || !ReadSized(reader, input.recipeBytes, MaximumRecipeBytes)) return false;
            input.role = RecipeInputRole(role); input.sourceLocalID = localID;
            input.shapeSlot = std::uint32_t(slot); input.recipeKind = RecipeInputFamily(kind);
            input.recipeSchema = std::uint32_t(recipeSchema);
            if (!reader.scalar(input.sourceMetersPerUnit)
                || !reader.scalar(input.carrierMetersPerUnit)) return false;
            for (double& scalar : input.sourceRootToCarrier) if (!reader.scalar(scalar)) return false;
            if (input.recipeKind != RecipeInputFamily::OrdinarySource) {
                for (double& scalar : input.projection.dimensions) if (!reader.scalar(scalar)) return false;
                for (double& scalar : input.projection.translation) if (!reader.scalar(scalar)) return false;
                for (double& scalar : input.projection.rotationXYZW) if (!reader.scalar(scalar)) return false;
            }
            std::vector<std::uint8_t> metadata;
            if (!ReadCommitments(reader, input.commitments) || !ReadSized(reader, metadata)
                || !ReadMetadata(metadata, input.metadata)) return false;
        }
        return ReadTables(reader, value.materials, value.groups, value.layers)
            && reader.complete();
    } catch (...) { value = {}; return false; }
}

inline bool EncodeEffectiveStateBlock(const EffectiveStateBlock& value,
                                      std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        Writer writer; writer.integer(value.metadataSchema, 2); writer.integer(2, 2);
        for (const EffectiveInput& input : value.inputs) {
            writer.integer(std::uint8_t(input.role), 1);
            writer.integer(std::uint8_t(input.family), 1); writer.integer(0, 2);
            writer.raw(input.sourceNode); writer.raw(input.originalSourceFeature);
            WriteCommitments(writer, input.commitments);
            if (input.family == InputFamily::AnalyticRectangularPrism) {
                for (double scalar : input.geometry.dimensions) writer.scalar(scalar);
                for (double scalar : input.geometry.translation) writer.scalar(scalar);
                for (double scalar : input.geometry.rotationXYZW) writer.scalar(scalar);
            } else if (input.family == InputFamily::FrozenSolid) {
                for (double scalar : input.frozenGeometry.sourceToCarrier)
                    writer.scalar(scalar);
            } else writer.valid = false;
            writer.scalar(input.sourceMetersPerUnit);
            std::vector<std::uint8_t> metadata;
            if (!WriteMetadata(input.metadata, metadata)) writer.valid = false;
            else WriteSized(writer, metadata);
        }
        WriteTables(writer, value.materials, value.groups, value.layers);
        if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool DecodeEffectiveStateBlock(const std::vector<std::uint8_t>& bytes,
                                      EffectiveStateBlock& value) noexcept {
    value = {};
    try {
        Reader reader(bytes, bytes.size()); std::uint64_t schema = 0, count = 0;
        if (!reader.integer(2, schema) || schema != 1
            || !reader.integer(2, count) || count != 2) return false;
        value.metadataSchema = std::uint16_t(schema);
        for (EffectiveInput& input : value.inputs) {
            std::uint64_t role = 0, family = 0, reserved = 0;
            if (!reader.integer(1, role) || !reader.integer(1, family)
                || !reader.integer(2, reserved) || reserved != 0
                || !reader.raw(input.sourceNode) || !reader.raw(input.originalSourceFeature)
                || !ReadCommitments(reader, input.commitments)) return false;
            input.role = RecipeInputRole(role); input.family = InputFamily(family);
            if (input.family == InputFamily::AnalyticRectangularPrism) {
                for (double& scalar : input.geometry.dimensions) if (!reader.scalar(scalar)) return false;
                for (double& scalar : input.geometry.translation) if (!reader.scalar(scalar)) return false;
                for (double& scalar : input.geometry.rotationXYZW) if (!reader.scalar(scalar)) return false;
            } else if (input.family == InputFamily::FrozenSolid) {
                for (double& scalar : input.frozenGeometry.sourceToCarrier)
                    if (!reader.scalar(scalar)) return false;
            } else return false;
            std::vector<std::uint8_t> metadata;
            if (!reader.scalar(input.sourceMetersPerUnit) || !ReadSized(reader, metadata)
                || !ReadMetadata(metadata, input.metadata)) return false;
        }
        return ReadTables(reader, value.materials, value.groups, value.layers)
            && reader.complete();
    } catch (...) { value = {}; return false; }
}

inline bool EncodeOriginals(const OriginalsDefinition& value,
                            std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!Valid(value)) return false;
        Writer writer; writer.raw(reinterpret_cast<const std::uint8_t*>("SYPB"), 4);
        writer.integer(OriginalsPayloadVersion, 1); writer.integer(0, 3);
        writer.integer(std::uint8_t(value.operation), 1);
        writer.integer(std::uint8_t(value.materialPolicy), 1); writer.integer(0, 2);
        for (std::uint32_t version : {value.versions.serializer, value.versions.build,
             value.versions.proof, value.versions.selector, value.versions.material})
            writer.integer(version, 4);
        writer.raw(value.owner.document); writer.raw(value.owner.entity);
        writer.raw(value.owner.definition); writer.raw(value.booleanNode); writer.raw(value.feature);
        writer.scalar(value.carrierMetersPerUnit);
        std::vector<std::uint8_t> originals, effective;
        if (!EncodeOriginalStateBlock(value.originals, originals)
            || !EncodeEffectiveStateBlock(value.effective, effective)) return false;
        WriteSized(writer, originals); WriteSized(writer, effective);
        if (!writer.valid || writer.bytes.size() > MaximumPayloadBytes - 32) return false;
        Digest digest{}; if (!retained_solid::Hash(writer.bytes, digest)) return false;
        writer.raw(digest); if (!writer.valid || writer.bytes.size() > MaximumPayloadBytes) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool DecodeOriginals(const std::vector<std::uint8_t>& bytes,
                            OriginalsDefinition& output) noexcept {
    output = {};
    try {
        if (bytes.size() < 8 + 4 + 20 + 80 + 8 + 8 + 32
            || bytes.size() > MaximumPayloadBytes
            || std::memcmp(bytes.data(), "SYPB\4\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!retained_solid::Hash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (expected != actual) return false;
        Reader reader(bytes, bytes.size() - 32); std::array<std::uint8_t, 8> prefix{};
        std::uint64_t operation = 0, policy = 0, reserved = 0;
        OriginalsDefinition value;
        if (!reader.raw(prefix) || !reader.integer(1, operation)
            || !reader.integer(1, policy) || !reader.integer(2, reserved) || reserved != 0)
            return false;
        value.operation = Operation(operation); value.materialPolicy = MaterialPolicy(policy);
        std::uint32_t* versions[] = {&value.versions.serializer, &value.versions.build,
            &value.versions.proof, &value.versions.selector, &value.versions.material};
        for (auto* version : versions) { std::uint64_t decoded = 0;
            if (!reader.integer(4, decoded) || decoded > UINT32_MAX) return false;
            *version = std::uint32_t(decoded); }
        if (!reader.raw(value.owner.document) || !reader.raw(value.owner.entity)
            || !reader.raw(value.owner.definition) || !reader.raw(value.booleanNode)
            || !reader.raw(value.feature) || !reader.scalar(value.carrierMetersPerUnit)) return false;
        std::vector<std::uint8_t> originals, effective;
        if (!ReadSized(reader, originals) || !ReadSized(reader, effective)
            || !reader.complete() || !DecodeOriginalStateBlock(originals, value.originals)
            || !DecodeEffectiveStateBlock(effective, value.effective) || !Valid(value)) return false;
        std::vector<std::uint8_t> canonical;
        if (!EncodeOriginals(value, canonical) || canonical != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}
} // namespace core3d::part_boolean
