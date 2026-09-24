#pragma once
#include "CompositeRecipeDefinition.hxx"
#include "RetainedBooleanProgram.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <algorithm>
#include <cstring>
#include <limits>
#include <set>

namespace core3d::composite_recipe {
inline constexpr std::uint32_t PartBooleanFeatureKind = 1; // Value reservation only; A1 owns build/proof.
inline constexpr std::uint32_t PartBooleanFeatureCodec = 1;

class Writer {
public:
    std::vector<std::uint8_t> bytes;
    bool valid = true;
    void raw(const std::uint8_t* data, std::size_t count) {
        if (!valid || count > MaximumEnvelopeBytes - bytes.size()) { valid = false; return; }
        bytes.insert(bytes.end(), data, data + count);
    }
    template<std::size_t N> void raw(const std::array<std::uint8_t, N>& value) { raw(value.data(), N); }
    void integer(std::uint64_t value, unsigned width = 8) {
        if (width == 0 || width > 8) { valid = false; return; }
        std::uint8_t encoded[8]{};
        for (unsigned index = 0; index < width; ++index) encoded[index] = std::uint8_t(value >> (8 * index));
        raw(encoded, width);
    }
    void scalar(double value) {
        if (!std::isfinite(value)) { valid = false; return; }
        std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits)); integer(bits);
    }
};

class Reader {
public:
    Reader(const std::vector<std::uint8_t>& bytes, std::size_t limit) : bytes_(bytes), limit_(limit) {}
    bool raw(std::uint8_t* output, std::size_t count) {
        if (!ok_ || count > limit_ - cursor_) { ok_ = false; return false; }
        std::copy_n(bytes_.begin() + cursor_, count, output); cursor_ += count; return true;
    }
    template<std::size_t N> bool raw(std::array<std::uint8_t, N>& output) { return raw(output.data(), N); }
    bool integer(unsigned width, std::uint64_t& value) {
        value = 0; if (!ok_ || width == 0 || width > 8 || width > limit_ - cursor_) { ok_ = false; return false; }
        for (unsigned index = 0; index < width; ++index) value |= std::uint64_t(bytes_[cursor_++]) << (8 * index);
        return true;
    }
    bool scalar(double& value) {
        std::uint64_t bits = 0; if (!integer(8, bits)) return false;
        std::memcpy(&value, &bits, sizeof(value)); return std::isfinite(value);
    }
    std::size_t cursor() const noexcept { return cursor_; }
    bool complete() const noexcept { return ok_ && cursor_ == limit_; }
private:
    const std::vector<std::uint8_t>& bytes_;
    std::size_t limit_ = 0, cursor_ = 0;
    bool ok_ = true;
};

inline bool Hash(const std::vector<std::uint8_t>& bytes, Digest& output) noexcept {
    output.fill(0);
    return !bytes.empty() && bytes.size() <= MaximumEnvelopeBytes
        && CC_SHA256(bytes.data(), CC_LONG(bytes.size()), output.data()) != nullptr;
}

inline bool EncodeScalarRecipe(RecipeKind kind, std::uint32_t schema,
                               const std::vector<double>& values,
                               std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (kind == RecipeKind::RetainedBoolean || schema == 0 || values.empty()
            || values.size() > std::size_t(profile::MaximumScalars)) return false;
        Writer writer; writer.raw(reinterpret_cast<const std::uint8_t*>("SYLV"), 4);
        writer.integer(1, 1); writer.integer(std::uint8_t(kind), 1);
        writer.integer(schema, 4); writer.integer(values.size(), 4);
        for (double value : values) writer.scalar(value);
        if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool DecodeScalarRecipe(const SourceRecipe& recipe, std::vector<double>& values) noexcept {
    values.clear();
    try {
        if (recipe.bytes.size() < 14 || recipe.bytes.size() > MaximumEnvelopeBytes
            || std::memcmp(recipe.bytes.data(), "SYLV", 4) != 0
            || recipe.bytes[4] != 1 || recipe.bytes[5] != std::uint8_t(recipe.kind)) return false;
        Reader reader(recipe.bytes, recipe.bytes.size());
        std::array<std::uint8_t, 6> prefix{}; std::uint64_t schema = 0, count = 0;
        if (!reader.raw(prefix) || !reader.integer(4, schema) || !reader.integer(4, count)
            || schema != recipe.schema || count == 0 || count > std::size_t(profile::MaximumScalars)
            || count > (recipe.bytes.size() - reader.cursor()) / 8) return false;
        values.reserve(std::size_t(count));
        for (std::uint64_t index = 0; index < count; ++index) {
            double value = 0; if (!reader.scalar(value)) return false; values.push_back(value);
        }
        return reader.complete();
    } catch (...) { values.clear(); return false; }
}

inline bool ValidRecipe(const SourceRecipe& recipe) noexcept {
    try {
        if (recipe.bytes.empty() || recipe.bytes.size() > MaximumEnvelopeBytes) return false;
        if (recipe.kind == RecipeKind::RetainedBoolean) {
            retained_boolean::Recipe decoded;
            std::vector<std::uint8_t> exact;
            return recipe.schema == 1 && retained_boolean::Decode(recipe.bytes, decoded)
                && retained_boolean::Encode(decoded, exact) && exact == recipe.bytes;
        }
        std::vector<double> values;
        if (!DecodeScalarRecipe(recipe, values)) return false;
        if (recipe.kind == RecipeKind::Profile) {
            profile::Parameters decoded;
            return recipe.schema >= 1 && recipe.schema <= 5 && profile::Decode(values, decoded)
                && profile::SchemaFor(decoded) == int(recipe.schema);
        }
        if (recipe.kind == RecipeKind::Enclosure) {
            enclosure::Parameters decoded;
            return recipe.schema >= 1 && recipe.schema <= 2
                && enclosure::Decode(int(recipe.schema), values, decoded);
        }
        if (recipe.kind == RecipeKind::RectangularLoft) {
            rectangular_loft::Definition decoded;
            return recipe.schema == std::uint32_t(loft_persistence::Schema)
                && loft_persistence::Decode(values, decoded);
        }
        return false;
    } catch (...) { return false; }
}

inline bool Valid(const Definition& definition) noexcept {
    try {
        if (definition.schemaVersion != 1 || !retained_recipe::Valid(definition.owner)
            || !retained_recipe::Valid(definition.issuance) || !retained_recipe::Nonzero(definition.outputNode)
            || definition.nodes.empty() || definition.nodes.size() > MaximumNodes) return false;
        std::set<UUID> nodeIDs, featureIDs;
        std::set<std::uint64_t> localIDs;
        std::set<UUID> sourceEntities, sourceDefinitions;
        std::set<std::uint32_t> shapeSlots;
        std::vector<std::size_t> depths;
        std::size_t sources = 0;
        std::uint64_t maximumLocalID = 0;
        for (const Node& node : definition.nodes) {
            const UUID& id = NodeID(node); const std::uint64_t local = LocalID(node);
            if (!retained_recipe::Nonzero(id) || !nodeIDs.insert(id).second || local == 0
                || local >= definition.issuance.nextLocalID || !localIDs.insert(local).second) return false;
            maximumLocalID = std::max(maximumLocalID, local);
            if (const auto* source = std::get_if<SourceNode>(&node.value)) {
                ++sources;
                if (sources > MaximumSourceNodes || !retained_recipe::Valid(source->original)
                    || source->original.document != definition.owner.document
                    || !sourceEntities.insert(source->original.entity).second
                    || !sourceDefinitions.insert(source->original.definition).second
                    || !ValidRecipe(source->recipe) || !ValidPlacement(source->inputToCarrier)
                    || !Valid(source->commitments) || !shapeSlots.insert(source->shapeSlot).second) return false;
                Digest recipeDigest;
                if (!Hash(source->recipe.bytes, recipeDigest) || recipeDigest != source->commitments.recipe) return false;
                depths.push_back(1); continue;
            }
            const auto& feature = std::get<FeatureNode>(node.value);
            if (!retained_recipe::Nonzero(feature.feature) || !featureIDs.insert(feature.feature).second
                || feature.kind != PartBooleanFeatureKind || feature.codecVersion != PartBooleanFeatureCodec
                || feature.inputs.size() != 2 || feature.inputs[0] == feature.inputs[1]
                || feature.parameters.size() > MaximumFeaturePayloadBytes) return false;
            std::size_t depth = 1;
            for (const UUID& input : feature.inputs) {
                const auto found = std::find_if(definition.nodes.begin(), definition.nodes.begin() + depths.size(),
                    [&](const Node& prior) { return NodeID(prior) == input; });
                if (found == definition.nodes.begin() + depths.size()) return false;
                const auto index = std::size_t(std::distance(definition.nodes.begin(), found));
                depth = std::max(depth, depths[index] + 1);
            }
            if (depth > MaximumDepth) return false; depths.push_back(depth);
        }
        if (sources == 0 || maximumLocalID >= definition.issuance.nextLocalID
            || definition.outputNode != NodeID(definition.nodes.back())
            || !std::holds_alternative<FeatureNode>(definition.nodes.back().value)) return false;
        for (std::uint32_t slot = 0; slot < sources; ++slot)
            if (shapeSlots.count(slot) != 1) return false;
        for (std::uint64_t retired : definition.issuance.retiredLocalIDs)
            if (localIDs.count(retired)) return false;
        return true;
    } catch (...) { return false; }
}

inline bool Encode(const Definition& definition, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!Valid(definition)) return false;
        Writer writer; writer.raw(reinterpret_cast<const std::uint8_t*>("SYCR"), 4);
        writer.integer(1, 1); writer.integer(0, 1); writer.integer(0, 2);
        writer.raw(definition.owner.document); writer.raw(definition.owner.entity); writer.raw(definition.owner.definition);
        writer.raw(definition.outputNode); writer.integer(definition.issuance.nextLocalID);
        writer.integer(definition.issuance.retiredLocalIDs.size(), 2);
        writer.integer(definition.nodes.size(), 2);
        for (std::uint64_t retired : definition.issuance.retiredLocalIDs) writer.integer(retired);
        for (const Node& node : definition.nodes) {
            writer.integer(std::uint8_t(Kind(node)), 1); writer.integer(0, 1); writer.integer(0, 2);
            writer.raw(NodeID(node)); writer.integer(LocalID(node));
            if (const auto* source = std::get_if<SourceNode>(&node.value)) {
                writer.raw(source->original.document); writer.raw(source->original.entity);
                writer.raw(source->original.definition); writer.raw(source->original.sourceFeature);
                writer.integer(std::uint8_t(source->recipe.kind), 1); writer.integer(0, 1);
                writer.integer(source->recipe.schema, 4); writer.integer(source->shapeSlot, 4);
                writer.scalar(source->inputToCarrier.sourceMetersPerUnit);
                writer.scalar(source->inputToCarrier.carrierMetersPerUnit);
                for (double scalar : source->inputToCarrier.matrix) writer.scalar(scalar);
                writer.raw(source->commitments.geometry); writer.raw(source->commitments.recipe);
                writer.raw(source->commitments.placement); writer.raw(source->commitments.material);
                writer.raw(source->commitments.groups);
                writer.integer(source->recipe.bytes.size(), 4);
                writer.raw(source->recipe.bytes.data(), source->recipe.bytes.size());
            } else {
                const auto& feature = std::get<FeatureNode>(node.value);
                writer.raw(feature.feature); writer.integer(feature.kind, 4);
                writer.integer(feature.codecVersion, 4); writer.integer(feature.inputs.size(), 2);
                writer.integer(feature.parameters.size(), 4);
                for (const UUID& input : feature.inputs) writer.raw(input);
                writer.raw(feature.parameters.data(), feature.parameters.size());
            }
        }
        if (!writer.valid || writer.bytes.size() > MaximumEnvelopeBytes - 32) return false;
        Digest digest; if (!Hash(writer.bytes, digest)) return false; writer.raw(digest);
        if (!writer.valid) return false; output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output) noexcept {
    output = {};
    try {
        if (bytes.size() < 8 + 64 + 12 + 32 || bytes.size() > MaximumEnvelopeBytes
            || std::memcmp(bytes.data(), "SYCR\1\0\0\0", 8) != 0) return false;
        Digest expected, actual; std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        if (!Hash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (actual != expected) return false;
        Reader reader(bytes, bytes.size() - 32); std::array<std::uint8_t, 8> prefix{};
        Definition definition; definition.schemaVersion = 1;
        std::uint64_t retiredCount = 0, nodeCount = 0;
        if (!reader.raw(prefix) || !reader.raw(definition.owner.document)
            || !reader.raw(definition.owner.entity) || !reader.raw(definition.owner.definition)
            || !reader.raw(definition.outputNode) || !reader.integer(8, definition.issuance.nextLocalID)
            || !reader.integer(2, retiredCount) || !reader.integer(2, nodeCount)
            || retiredCount > MaximumNodes || nodeCount == 0 || nodeCount > MaximumNodes) return false;
        for (std::uint64_t index = 0; index < retiredCount; ++index) {
            std::uint64_t value = 0; if (!reader.integer(8, value)) return false;
            definition.issuance.retiredLocalIDs.push_back(value);
        }
        for (std::uint64_t index = 0; index < nodeCount; ++index) {
            std::uint64_t kind = 0, reserved = 0, localID = 0;
            UUID nodeID{};
            if (!reader.integer(1, kind) || !reader.integer(1, reserved) || reserved != 0
                || !reader.integer(2, reserved) || reserved != 0 || !reader.raw(nodeID)
                || !reader.integer(8, localID)) return false;
            if (kind == std::uint8_t(NodeKind::Source)) {
                SourceNode source; source.node = nodeID; source.localID = localID;
                std::uint64_t recipeKind = 0, schema = 0, slot = 0, byteCount = 0;
                if (!reader.raw(source.original.document) || !reader.raw(source.original.entity)
                    || !reader.raw(source.original.definition) || !reader.raw(source.original.sourceFeature)
                    || !reader.integer(1, recipeKind) || !reader.integer(1, reserved) || reserved != 0
                    || !reader.integer(4, schema) || schema > UINT32_MAX
                    || !reader.integer(4, slot) || slot > UINT32_MAX
                    || !reader.scalar(source.inputToCarrier.sourceMetersPerUnit)
                    || !reader.scalar(source.inputToCarrier.carrierMetersPerUnit)) return false;
                source.recipe.kind = RecipeKind(recipeKind); source.recipe.schema = std::uint32_t(schema);
                source.shapeSlot = std::uint32_t(slot);
                for (double& scalar : source.inputToCarrier.matrix) if (!reader.scalar(scalar)) return false;
                if (!reader.raw(source.commitments.geometry) || !reader.raw(source.commitments.recipe)
                    || !reader.raw(source.commitments.placement) || !reader.raw(source.commitments.material)
                    || !reader.raw(source.commitments.groups) || !reader.integer(4, byteCount)
                    || byteCount == 0 || byteCount > MaximumEnvelopeBytes - reader.cursor()) return false;
                source.recipe.bytes.resize(std::size_t(byteCount));
                if (!reader.raw(source.recipe.bytes.data(), source.recipe.bytes.size())) return false;
                definition.nodes.push_back({std::move(source)});
            } else if (kind == std::uint8_t(NodeKind::Feature)) {
                FeatureNode feature; feature.node = nodeID; feature.localID = localID;
                std::uint64_t featureKind = 0, version = 0, inputs = 0, payload = 0;
                if (!reader.raw(feature.feature) || !reader.integer(4, featureKind) || featureKind > UINT32_MAX
                    || !reader.integer(4, version) || version > UINT32_MAX
                    || !reader.integer(2, inputs) || inputs > MaximumNodes
                    || !reader.integer(4, payload) || payload > MaximumFeaturePayloadBytes) return false;
                feature.kind = std::uint32_t(featureKind); feature.codecVersion = std::uint32_t(version);
                feature.inputs.resize(std::size_t(inputs));
                for (UUID& input : feature.inputs) if (!reader.raw(input)) return false;
                feature.parameters.resize(std::size_t(payload));
                if (!reader.raw(feature.parameters.data(), feature.parameters.size())) return false;
                definition.nodes.push_back({std::move(feature)});
            } else return false;
        }
        std::vector<std::uint8_t> canonical;
        if (!reader.complete() || !Encode(definition, canonical) || canonical != bytes) return false;
        output = std::move(definition); return true;
    } catch (...) { output = {}; return false; }
}
} // namespace core3d::composite_recipe
