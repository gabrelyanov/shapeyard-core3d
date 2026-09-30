#pragma once

#include "FeaturePatternBaselineAttribute.hxx"
#include "FeaturePatternChildReceipt.hxx"
#include "FeaturePatternPersistence.hxx"

#include <TDF_Attribute.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDF_Reference.hxx>
#include <TDF_RelocationTable.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TDocStd_Document.hxx>
#include <TNaming_NamedShape.hxx>
#include <TNaming_Tool.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Shape.hxx>
#include <map>
#include <memory>
#include <set>
#include <vector>

namespace core3d::feature_pattern_child {
class BinaryDriver;
#if DEBUG
struct Probe;
#endif

inline const Standard_GUID& AttributeID() {
    static const Standard_GUID id("E441D87D-BFEB-486A-9AA2-64C0CDCBBA86");
    return id;
}

struct Payload final {
    Receipt receipt;
    std::vector<std::uint8_t> canonicalBytes;
    TDF_Label host;
    TDF_Label baselineRecipe;
    TDF_Label source;
    TDF_Label patternRecord;
};

inline constexpr const char* SourceLinkMarker = "Shapeyard SYFC source v1";
inline constexpr const char* PatternLinkMarker = "Shapeyard SYFC pattern v1";

inline bool SetLink(const TDF_Label& record, const char* marker,
                    const TDF_Label& target) noexcept {
    try {
        if (record.IsNull() || target.IsNull() || record.Data() != target.Data()) return false;
        TDF_Label link = TDF_TagSource::NewChild(record);
        TDataStd_AsciiString::Set(link, TCollection_AsciiString(marker));
        TDF_Reference::Set(link, target);
        return true;
    } catch (...) { return false; }
}

inline bool GetLink(const TDF_Label& record, const char* marker,
                    TDF_Label& target) noexcept {
    target.Nullify();
    try {
        unsigned matches = 0;
        for (TDF_ChildIterator child(record, Standard_False); child.More(); child.Next()) {
            Handle(TDataStd_AsciiString) name; Handle(TDF_Reference) reference;
            if (!child.Value().FindAttribute(TDataStd_AsciiString::GetID(), name)
                || name.IsNull() || name->Get().ToCString() != std::string(marker)) continue;
            if (++matches != 1
                || !child.Value().FindAttribute(TDF_Reference::GetID(), reference)
                || reference.IsNull() || reference->Get().IsNull()
                || reference->Get().Data() != record.Data()) return false;
            target = reference->Get();
        }
        return matches == 1 && !target.IsNull();
    } catch (...) { target.Nullify(); return false; }
}

class Core3D_FeaturePatternChild final : public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTI_INLINE(Core3D_FeaturePatternChild, TDF_Attribute)
    const Standard_GUID& ID() const override { return AttributeID(); }
    Handle(TDF_Attribute) NewEmpty() const override {
        return new Core3D_FeaturePatternChild();
    }
    const std::shared_ptr<const Payload>& value() const noexcept { return value_; }
    bool initialize(std::shared_ptr<const Payload> value) noexcept {
        if (value_ || !value) return false;
        value_ = std::move(value); return true;
    }
    void Restore(const Handle(TDF_Attribute)& source) override {
        const auto original = Handle(Core3D_FeaturePatternChild)::DownCast(source);
        if (original.IsNull()) Standard_Failure::Raise("Feature child restore type");
        value_ = original->value_;
    }
    void Paste(const Handle(TDF_Attribute)& target,
               const Handle(TDF_RelocationTable)& relocation) const override {
        const auto destination = Handle(Core3D_FeaturePatternChild)::DownCast(target);
        if (destination.IsNull() || !value_ || relocation.IsNull())
            Standard_Failure::Raise("Feature child paste type");
        TDF_Label host, baseline, source, patternRecord;
        if (!relocation->HasRelocation(value_->host, host)
            || !relocation->HasRelocation(value_->baselineRecipe, baseline)
            || !relocation->HasRelocation(value_->source, source)
            || !relocation->HasRelocation(value_->patternRecord, patternRecord)
            || host.IsNull() || baseline.IsNull() || source.IsNull()
            || patternRecord.IsNull() || host.Data() != baseline.Data()
            || host.Data() != source.Data() || host.Data() != patternRecord.Data())
            Standard_Failure::Raise("Feature child paste relocation");
        auto payload = std::make_shared<Payload>(*value_);
        payload->host = host; payload->baselineRecipe = baseline;
        payload->source = source; payload->patternRecord = patternRecord;
        destination->Backup(); destination->value_ = std::move(payload);
    }
private:
    friend class BinaryDriver;
#if DEBUG
    friend struct Probe;
#endif
    std::shared_ptr<const Payload> value_;
};
using Attribute = Core3D_FeaturePatternChild;

inline bool Attach(const TDF_Label& record, const TDF_Label& host,
                   const TDF_Label& baselineRecipe, const TDF_Label& source,
                   const TDF_Label& patternRecord,
                   const Receipt& receipt) noexcept {
    try {
        if (record.IsNull() || host.IsNull() || baselineRecipe.IsNull()
            || source.IsNull() || patternRecord.IsNull()
            || record.Data() != host.Data() || host.Data() != baselineRecipe.Data()
            || host.Data() != source.Data() || source.Data() != patternRecord.Data()
            || !record.Father().IsEqual(host) || baselineRecipe.IsEqual(record)) return false;
        std::vector<std::uint8_t> bytes;
        if (!Encode(receipt, bytes)) return false;
        Handle(Attribute) existing;
        if (record.FindAttribute(AttributeID(), existing)) return false;
        auto payload = std::make_shared<Payload>(); payload->receipt = receipt;
        payload->canonicalBytes = std::move(bytes); payload->host = host;
        payload->baselineRecipe = baselineRecipe; payload->source = source;
        payload->patternRecord = patternRecord;
        Handle(Attribute) attribute = new Attribute();
        if (!attribute->initialize(std::move(payload))) return false;
        record.AddAttribute(attribute);
        // The real cross-record edge is persisted by OCCT's reference driver.
        // The SYFC bytes carry only the immutable semantic baseline identity.
        TDF_Reference::Set(record, baselineRecipe);
        return SetLink(record, SourceLinkMarker, source)
            && SetLink(record, PatternLinkMarker, patternRecord);
    } catch (...) { return false; }
}

// Codec-only compatibility used by package-11 isolation tests. Production
// document staging always supplies the two explicit paired links above.
inline bool Attach(const TDF_Label& record, const TDF_Label& host,
                   const TDF_Label& baselineRecipe,
                   const Receipt& receipt) noexcept {
    return Attach(record, host, baselineRecipe, baselineRecipe, baselineRecipe, receipt);
}

inline bool Read(const TDF_Label& record,
                 std::shared_ptr<const Payload>& output) noexcept {
    output.reset();
    try {
        Handle(Attribute) attribute; Handle(TDF_Reference) reference;
        if (record.IsNull() || !record.FindAttribute(AttributeID(), attribute)
            || attribute.IsNull() || !attribute->value()
            || !record.FindAttribute(TDF_Reference::GetID(), reference)
            || reference.IsNull() || reference->Get().IsNull()) return false;
        // The native baseline reference is resolved here, after traversal,
        // alongside the deferred source/pattern child links: it must exist,
        // be non-null, live in the record's document and not be the record
        // itself. Any link already stored on the payload must match exactly.
        const TDF_Label baseline = reference->Get();
        const auto& value = attribute->value();
        std::vector<std::uint8_t> exact;
        TDF_Label source, patternRecord;
        if (!record.Father().IsEqual(value->host)
            || baseline.Data() != record.Data() || baseline.IsEqual(record)
            || (!value->baselineRecipe.IsNull()
                && !baseline.IsEqual(value->baselineRecipe))
            || !GetLink(record, SourceLinkMarker, source)
            || !GetLink(record, PatternLinkMarker, patternRecord)
            || (!value->source.IsNull() && !source.IsEqual(value->source))
            || (!value->patternRecord.IsNull()
                && !patternRecord.IsEqual(value->patternRecord))
            || !Encode(value->receipt, exact) || exact != value->canonicalBytes)
            return false;
        if (value->baselineRecipe.IsNull() || value->source.IsNull()
            || value->patternRecord.IsNull()) {
            auto resolved = std::make_shared<Payload>(*value);
            resolved->baselineRecipe = baseline;
            resolved->source = source; resolved->patternRecord = patternRecord;
            output = std::move(resolved);
        } else output = value;
        return true;
    } catch (...) { output.reset(); return false; }
}

enum class PairStatus : std::uint8_t { Absent, Legacy, Valid, Invalid };

struct PairedRecord final {
    feature_pattern::Record pattern;
    TDF_Label host, baselineRecipe, source;
    std::vector<std::shared_ptr<const Payload>> children;
};

// Read-only whole-document pairing census. A legacy tag-73 record is reported
// distinctly and is never modified. Any SYFC without exactly one matching
// SYFP, or any partial/foreign host/source link, poisons the whole census.
inline PairStatus ReadPairs(const Handle(TDocStd_Document)& document,
                            std::vector<PairedRecord>& output) noexcept {
    output.clear();
    try {
        std::vector<feature_pattern::Record> patterns;
        if (document.IsNull() || document->GetData().IsNull()
            || !feature_pattern::ReadAll(document, patterns)) return PairStatus::Invalid;
        std::map<UUID, std::size_t> byFeature;
        for (std::size_t index = 0; index < patterns.size(); ++index)
            if (!byFeature.emplace(patterns[index].definition.feature, index).second)
                return PairStatus::Invalid;
        std::vector<PairedRecord> paired(patterns.size());
        for (std::size_t index = 0; index < patterns.size(); ++index)
            paired[index].pattern = patterns[index];
        std::size_t attributes = 0;
        for (TDF_ChildIterator label(document->GetData()->Root(), Standard_True);
             label.More(); label.Next()) {
            Handle(Attribute) attribute;
            if (!label.Value().FindAttribute(AttributeID(), attribute)) continue;
            if (++attributes > feature_pattern::MaximumGeneratedFeatures * (patterns.size() + 1))
                return PairStatus::Invalid;
            std::shared_ptr<const Payload> payload;
            if (!Read(label.Value(), payload) || !payload) return PairStatus::Invalid;
            const auto found = byFeature.find(payload->receipt.patternFeature);
            if (found == byFeature.end()) return PairStatus::Invalid;
            PairedRecord& pair = paired[found->second];
            const auto& definition = pair.pattern.definition;
            if (!payload->patternRecord.IsEqual(pair.pattern.label)
                || payload->receipt.document != definition.host.document
                || payload->receipt.hostEntity != definition.host.entity
                || payload->receipt.hostDefinition != definition.host.definition
                || payload->receipt.patternFeature != definition.feature
                || payload->source.IsEqual(payload->host)
                || payload->source.Data() != payload->host.Data()) return PairStatus::Invalid;
            if (pair.host.IsNull()) {
                pair.host = payload->host; pair.baselineRecipe = payload->baselineRecipe;
                pair.source = payload->source;
            } else if (!pair.host.IsEqual(payload->host)
                || !pair.baselineRecipe.IsEqual(payload->baselineRecipe)
                || !pair.source.IsEqual(payload->source)) return PairStatus::Invalid;
            pair.children.push_back(std::move(payload));
        }
        if (patterns.empty()) return attributes == 0 ? PairStatus::Absent : PairStatus::Invalid;
        bool legacy = false;
        for (const PairedRecord& pair : paired) {
            if (pair.children.empty()) { legacy = true; continue; }
            std::uint32_t active = 0; std::set<UUID> children;
            if (!feature_pattern::ActiveCount(pair.pattern.definition, active)
                || pair.children.size() != active) return PairStatus::Invalid;
            for (const auto& child : pair.children)
                if (!children.insert(child->receipt.childFeature).second) return PairStatus::Invalid;
            for (const pattern::Member& member : pair.pattern.definition.distribution.members)
                if (member.state == pattern::MemberState::Active
                    && children.erase(feature_pattern::ChildFeatureID(
                        pair.pattern.definition, member)) != 1) return PairStatus::Invalid;
            if (!children.empty()) return PairStatus::Invalid;
        }
        if (legacy) return attributes == 0 ? PairStatus::Legacy : PairStatus::Invalid;
        output = std::move(paired); return PairStatus::Valid;
    } catch (...) { output.clear(); return PairStatus::Invalid; }
}

struct BaselineRecord final {
    TDF_Label label, host;
    std::shared_ptr<const feature_pattern_baseline::Payload> value;
    TopoDS_Shape solid;
};

enum class BaselineStatus : std::uint8_t { Absent, Legacy, Valid, Invalid };

// Separate strict baseline-role census shared by native admission and
// persistence/publication. It first uses the exact ReadPairs result, then
// requires, for each nonlegacy paired D4, exactly one typed baseline on a
// distinct direct metadata child of its actual host. The payload's
// document/entity/definition must match both the real label UUID attributes
// and the pair; every child must name the same nonzero baseline identity and
// this exact baseline label; the bound TNaming solid must be nonnull and a
// solid. Wrong-type, orphan, duplicate or misplaced baseline attributes and
// incompatible baseline ownership poison the whole census, and each distinct
// baseline is counted once toward the byte budgets. Absence of D4 baselines
// stays valid for documents without paired D4, and tag-73-only legacy
// documents stay unmodified and uneditable: no baseline is synthesized for
// them. Read-only: it never creates labels, attributes or identities.
inline BaselineStatus ReadBaselines(
    const Handle(TDocStd_Document)& document,
    std::vector<BaselineRecord>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull())
            return BaselineStatus::Invalid;
        std::vector<PairedRecord> pairs;
        const PairStatus pairStatus = ReadPairs(document, pairs);
        if (pairStatus == PairStatus::Invalid) return BaselineStatus::Invalid;
        std::vector<TDF_Label> carriers;
        {
            std::size_t visited = 0;
            for (TDF_ChildIterator label(document->GetData()->Root(),
                     Standard_True); label.More(); label.Next()) {
                if (++visited > 100000) return BaselineStatus::Invalid;
                if (label.Value().IsAttribute(
                        feature_pattern_baseline::AttributeID()))
                    carriers.push_back(label.Value());
            }
        }
        if (pairStatus == PairStatus::Absent)
            return carriers.empty()
                ? BaselineStatus::Absent : BaselineStatus::Invalid;
        if (pairStatus == PairStatus::Legacy)
            return carriers.empty()
                ? BaselineStatus::Legacy : BaselineStatus::Invalid;
        std::size_t aggregate = 0;
        std::vector<BaselineRecord> staged;
        std::vector<bool> claimed(pairs.size(), false);
        for (const TDF_Label& carrier : carriers) {
            std::shared_ptr<const feature_pattern_baseline::Payload> payload;
            if (!feature_pattern_baseline::Read(carrier, payload) || !payload)
                return BaselineStatus::Invalid;
            std::size_t pairIndex = pairs.size();
            for (std::size_t index = 0; index < pairs.size(); ++index)
                if (pairs[index].baselineRecipe.IsEqual(carrier)) {
                    pairIndex = index; break;
                }
            if (pairIndex == pairs.size() || claimed[pairIndex])
                return BaselineStatus::Invalid;
            const PairedRecord& pair = pairs[pairIndex];
            if (!pair.host.IsEqual(payload->host)
                || !carrier.Father().IsEqual(pair.host))
                return BaselineStatus::Invalid;
            for (const auto& child : pair.children)
                if (!child || child->receipt.baselineRecipeIdentity
                        != payload->envelope.baselineRecipeIdentity)
                    return BaselineStatus::Invalid;
            UUID documentID{}, hostEntity{}, hostDefinition{};
            if (!retained_solid::ReadUUID(document->Main(),
                    Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"),
                    documentID)
                || !retained_solid::ReadUUID(pair.host,
                    Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"),
                    hostEntity)
                || !retained_solid::ReadUUID(pair.host,
                    Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"),
                    hostDefinition)
                || payload->envelope.document != documentID
                || payload->envelope.hostEntity != hostEntity
                || payload->envelope.hostDefinition != hostDefinition
                || payload->envelope.document
                    != pair.pattern.definition.host.document
                || payload->envelope.hostEntity
                    != pair.pattern.definition.host.entity
                || payload->envelope.hostDefinition
                    != pair.pattern.definition.host.definition)
                return BaselineStatus::Invalid;
            Handle(TNaming_NamedShape) named;
            if (!carrier.FindAttribute(TNaming_NamedShape::GetID(), named)
                || named.IsNull()) return BaselineStatus::Invalid;
            const TopoDS_Shape solid = TNaming_Tool::GetShape(named);
            if (solid.IsNull() || solid.ShapeType() != TopAbs_SOLID)
                return BaselineStatus::Invalid;
            for (TDF_AttributeIterator attribute(carrier);
                 attribute.More(); attribute.Next())
                if (attribute.Value()->ID()
                        != feature_pattern_baseline::AttributeID()
                    && attribute.Value()->ID() != TNaming_NamedShape::GetID())
                    return BaselineStatus::Invalid;
            std::size_t nested = 0;
            for (TDF_ChildIterator child(carrier, Standard_True);
                 child.More(); child.Next())
                if (++nested > 100000 || child.Value().HasAttribute())
                    return BaselineStatus::Invalid;
            if (payload->canonicalBytes.empty()
                || payload->canonicalBytes.size()
                    > feature_pattern_baseline::MaximumBaselineBytes
                || aggregate > feature_pattern_baseline::MaximumDocumentBytes
                || payload->canonicalBytes.size()
                    > feature_pattern_baseline::MaximumDocumentBytes - aggregate)
                return BaselineStatus::Invalid;
            aggregate += payload->canonicalBytes.size();
            claimed[pairIndex] = true;
            staged.push_back({carrier, pair.host, payload, solid});
        }
        for (const bool found : claimed)
            if (!found) return BaselineStatus::Invalid;
        output = std::move(staged);
        return BaselineStatus::Valid;
    } catch (...) { output.clear(); return BaselineStatus::Invalid; }
}
} // namespace core3d::feature_pattern_child
