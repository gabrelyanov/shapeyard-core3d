#pragma once

#include "FeaturePatternChildReceipt.hxx"
#include "RetainedSolidAttribute.hxx"

#include <TDF_Attribute.hxx>
#include <TDF_Label.hxx>
#include <TDF_RelocationTable.hxx>
#include <TDocStd_Document.hxx>
#include <TNaming_Builder.hxx>
#include <TNaming_NamedShape.hxx>
#include <TNaming_Tool.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Shape.hxx>
#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

namespace core3d::feature_pattern_baseline {
class BinaryDriver;
#if DEBUG
struct Probe;
#endif

using feature_pattern_child::Digest;
using feature_pattern_child::Nonzero;
using feature_pattern_child::UUID;

inline const Standard_GUID& AttributeID() {
    static const Standard_GUID id("8B4D2E17-6C3A-4F9E-B51D-2A7C9E04F683");
    return id;
}

inline constexpr std::size_t MaximumBaselineBytes = 64 * 1024;
inline constexpr std::size_t MaximumDocumentBytes = 8 * 1024 * 1024;

// Immutable retained D4 host-baseline envelope. The recipe bytes are opaque
// baseline evidence under the present D4 contract; this wrapper never claims
// they are an executable or decoded modeling recipe. Authority comes from the
// exact paired native references, the host/document UUIDs and the retained
// solid, not from these bytes or their integrity digest.
struct Envelope final {
    UUID document{}, hostEntity{}, hostDefinition{};
    UUID retainedRecipeFeature{}, baselineRecipeIdentity{};
    std::vector<std::uint8_t> exactRecipe;
};

inline bool Valid(const Envelope& value) noexcept {
    return Nonzero(value.document) && Nonzero(value.hostEntity)
        && Nonzero(value.hostDefinition) && Nonzero(value.retainedRecipeFeature)
        && Nonzero(value.baselineRecipeIdentity) && !value.exactRecipe.empty()
        && value.exactRecipe.size() <= MaximumBaselineBytes;
}

// SYFB/1 is canonical: fixed magic/version, the five actual identities, an
// explicit bounded recipe-byte count, the exact recipe bytes and trailing
// integrity bytes. OCAF labels and process addresses are never part of these
// bytes; the companion attribute owns the host reference.
inline bool Encode(const Envelope& value,
                   std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!Valid(value)) return false;
        std::vector<std::uint8_t> bytes{'S','Y','F','B',1,0,0,0};
        for (const UUID* identifier : {&value.document, &value.hostEntity,
                &value.hostDefinition, &value.retainedRecipeFeature,
                &value.baselineRecipeIdentity})
            feature_pattern_child::detail::Raw(bytes, *identifier);
        feature_pattern_child::detail::U32(
            bytes, std::uint32_t(value.exactRecipe.size()));
        bytes.insert(bytes.end(), value.exactRecipe.begin(),
                     value.exactRecipe.end());
        if (bytes.size() > MaximumBaselineBytes - Digest{}.size()) return false;
        Digest digest{};
        if (!feature_pattern_child::detail::Hash(bytes, digest)) return false;
        bytes.insert(bytes.end(), digest.begin(), digest.end());
        output = std::move(bytes);
        return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes,
                   Envelope& output) noexcept {
    output = {};
    try {
        constexpr std::size_t fixed = 8 + 5 * 16 + 4;
        if (bytes.size() < fixed + Digest{}.size()
            || bytes.size() > MaximumBaselineBytes
            || std::memcmp(bytes.data(), "SYFB\1\0\0\0", 8) != 0) return false;
        const std::size_t bodySize = bytes.size() - Digest{}.size();
        std::vector<std::uint8_t> body(bytes.begin(), bytes.begin() + bodySize);
        Digest expected{}, actual{};
        if (!feature_pattern_child::detail::Hash(body, expected)) return false;
        std::copy_n(bytes.begin() + bodySize, actual.size(), actual.begin());
        if (actual != expected) return false;
        std::size_t at = 8;
        const auto need = [&](std::size_t count) {
            return at <= bodySize && count <= bodySize - at;
        };
        const auto raw = [&](UUID& value) {
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
        Envelope value; std::uint32_t count = 0;
        if (!raw(value.document) || !raw(value.hostEntity)
            || !raw(value.hostDefinition) || !raw(value.retainedRecipeFeature)
            || !raw(value.baselineRecipeIdentity) || !u32(count)
            || count == 0 || !need(count) || at + count != bodySize)
            return false;
        value.exactRecipe.assign(bytes.begin() + at,
                                 bytes.begin() + at + count);
        at += count;
        std::vector<std::uint8_t> canonical;
        if (at != bodySize || !Valid(value) || !Encode(value, canonical)
            || canonical != bytes) return false;
        output = std::move(value);
        return true;
    } catch (...) { output = {}; return false; }
}

struct Payload final {
    Envelope envelope;
    std::vector<std::uint8_t> canonicalBytes;
    TDF_Label host;
};

// Dedicated typed role for the retained D4 host baseline. This is not a
// default/custom TDataStd_ByteArray, a frame GUID, a generic string container
// or a second SYFP record. Backup/Restore/Paste share immutable payload
// ownership exactly like the retained-solid attribute; read-only inspection
// never creates missing attributes or issues identities.
class Core3D_FeaturePatternBaseline final : public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTI_INLINE(Core3D_FeaturePatternBaseline, TDF_Attribute)
    const Standard_GUID& ID() const override { return AttributeID(); }
    Handle(TDF_Attribute) NewEmpty() const override {
        return new Core3D_FeaturePatternBaseline();
    }
    const std::shared_ptr<const Payload>& value() const noexcept { return value_; }
    bool initialize(std::shared_ptr<const Payload> value) noexcept {
        if (value_ || !value) return false;
        value_ = std::move(value); return true;
    }
    bool replace(std::shared_ptr<const Payload> value) noexcept {
        if (!value_ || !value) return false;
        Backup(); value_ = std::move(value); return true;
    }
    void Restore(const Handle(TDF_Attribute)& source) override {
        const auto original =
            Handle(Core3D_FeaturePatternBaseline)::DownCast(source);
        if (original.IsNull())
            Standard_Failure::Raise("Feature baseline restore type");
        value_ = original->value_;
    }
    void Paste(const Handle(TDF_Attribute)& target,
               const Handle(TDF_RelocationTable)& relocation) const override {
        const auto destination =
            Handle(Core3D_FeaturePatternBaseline)::DownCast(target);
        if (destination.IsNull() || !value_ || relocation.IsNull())
            Standard_Failure::Raise("Feature baseline paste type");
        TDF_Label host;
        if (!relocation->HasRelocation(value_->host, host) || host.IsNull())
            Standard_Failure::Raise("Feature baseline paste relocation");
        auto payload = std::make_shared<Payload>(*value_);
        payload->host = host;
        destination->Backup(); destination->value_ = std::move(payload);
    }
private:
    friend class BinaryDriver;
#if DEBUG
    friend struct Probe;
#endif
    std::shared_ptr<const Payload> value_;
};
using Attribute = Core3D_FeaturePatternBaseline;

// Strict local read of one typed baseline: the attribute must carry a sealed
// canonical payload anchored to the label's actual father. The retained solid
// and the paired-record role are validated by the whole-document census.
inline bool Read(const TDF_Label& baselineLabel,
                 std::shared_ptr<const Payload>& output) noexcept {
    output.reset();
    try {
        Handle(Attribute) attribute;
        if (baselineLabel.IsNull()
            || !baselineLabel.FindAttribute(AttributeID(), attribute)
            || attribute.IsNull() || !attribute->value()) return false;
        const auto& value = attribute->value();
        std::vector<std::uint8_t> exact;
        if (value->host.IsNull()
            || !baselineLabel.Father().IsEqual(value->host)
            || !Encode(value->envelope, exact)
            || exact != value->canonicalBytes) return false;
        output = value;
        return true;
    } catch (...) { output.reset(); return false; }
}

// Stages one typed host baseline on a fresh distinct direct metadata child of
// the actual host, inside the caller's already-owned command. The generated
// retainedRecipeFeature and baselineRecipeIdentity persist as evidence; the
// actual baseline solid stays bound on the baseline label. Identity attributes
// on the document and host labels must already match the payload exactly.
inline bool Stage(const TDF_Label& baselineLabel, const TDF_Label& host,
                  const retained_recipe::OwnerKey& owner,
                  const UUID& retainedRecipeFeature,
                  const UUID& baselineRecipeIdentity,
                  const std::vector<std::uint8_t>& exactRecipe,
                  const TopoDS_Shape& solid) noexcept {
    try {
        if (baselineLabel.IsNull() || host.IsNull()
            || baselineLabel.Data() != host.Data()
            || !baselineLabel.Father().IsEqual(host)
            || baselineLabel.HasAttribute()) return false;
        Envelope envelope;
        envelope.document = owner.document;
        envelope.hostEntity = owner.entity;
        envelope.hostDefinition = owner.definition;
        envelope.retainedRecipeFeature = retainedRecipeFeature;
        envelope.baselineRecipeIdentity = baselineRecipeIdentity;
        envelope.exactRecipe = exactRecipe;
        if (!Valid(envelope) || solid.IsNull()
            || solid.ShapeType() != TopAbs_SOLID) return false;
        const Handle(TDocStd_Document) document =
            TDocStd_Document::Get(baselineLabel);
        UUID documentID{}, hostEntity{}, hostDefinition{};
        if (document.IsNull()
            || !retained_solid::ReadUUID(document->Main(),
                    Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"),
                    documentID)
            || !retained_solid::ReadUUID(host,
                    Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"),
                    hostEntity)
            || !retained_solid::ReadUUID(host,
                    Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"),
                    hostDefinition)
            || documentID != owner.document || hostEntity != owner.entity
            || hostDefinition != owner.definition) return false;
        std::vector<std::uint8_t> bytes;
        if (!Encode(envelope, bytes)) return false;
        auto payload = std::make_shared<Payload>();
        payload->envelope = std::move(envelope);
        payload->canonicalBytes = std::move(bytes);
        payload->host = host;
        Handle(Attribute) attribute = new Attribute();
        if (!attribute->initialize(std::move(payload))) return false;
        baselineLabel.AddAttribute(attribute);
        TNaming_Builder(baselineLabel).Select(solid, solid);
        Handle(TNaming_NamedShape) named;
        return baselineLabel.FindAttribute(TNaming_NamedShape::GetID(), named)
            && !named.IsNull() && !TNaming_Tool::GetShape(named).IsNull();
    } catch (...) { return false; }
}

// An authorized host edit that replaces the baseline updates the typed payload
// and the bound solid in the same owned command. The baseline identities and
// the paired references are preserved; only the recipe evidence and the solid
// evolve, so child receipts keep their exact baseline identity.
inline bool Replace(const TDF_Label& baselineLabel,
                    const std::vector<std::uint8_t>& exactRecipe,
                    const TopoDS_Shape& solid) noexcept {
    try {
        Handle(Attribute) attribute;
        if (baselineLabel.IsNull()
            || !baselineLabel.FindAttribute(AttributeID(), attribute)
            || attribute.IsNull() || !attribute->value()
            || exactRecipe.empty() || exactRecipe.size() > MaximumBaselineBytes
            || solid.IsNull() || solid.ShapeType() != TopAbs_SOLID)
            return false;
        Envelope envelope = attribute->value()->envelope;
        envelope.exactRecipe = exactRecipe;
        std::vector<std::uint8_t> bytes;
        if (!Encode(envelope, bytes)) return false;
        auto payload = std::make_shared<Payload>(*attribute->value());
        payload->envelope = std::move(envelope);
        payload->canonicalBytes = std::move(bytes);
        if (!attribute->replace(std::move(payload))) return false;
        TNaming_Builder(baselineLabel).Select(solid, solid);
        Handle(TNaming_NamedShape) named;
        return baselineLabel.FindAttribute(TNaming_NamedShape::GetID(), named)
            && !named.IsNull() && !TNaming_Tool::GetShape(named).IsNull();
    } catch (...) { return false; }
}

struct Budget final {
    std::size_t limit = MaximumDocumentBytes;
    std::size_t bytes = 0;
    std::size_t records = 0;
    bool rejected = false;
    void reset() noexcept { bytes = 0; records = 0; rejected = false; }
};

inline bool Charge(Budget& budget, std::size_t canonicalBytes) noexcept {
    if (budget.rejected || budget.limit > MaximumDocumentBytes
        || canonicalBytes == 0 || canonicalBytes > MaximumBaselineBytes)
        return false;
    if (budget.bytes > budget.limit
        || canonicalBytes > budget.limit - budget.bytes
        || budget.records == std::numeric_limits<std::size_t>::max()) {
        budget.rejected = true; return false;
    }
    budget.bytes += canonicalBytes; ++budget.records; return true;
}
} // namespace core3d::feature_pattern_baseline
