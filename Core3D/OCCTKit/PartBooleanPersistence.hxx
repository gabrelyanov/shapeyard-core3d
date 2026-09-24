#pragma once
#include "PartBooleanDefinition.hxx"
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
} // namespace core3d::part_boolean
