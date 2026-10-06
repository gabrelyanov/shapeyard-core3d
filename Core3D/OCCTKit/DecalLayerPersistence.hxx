#pragma once

// Inline, fail-closed SYDL/1 record access.  The owner supplies the exact
// retained-definition label and read-owned TDocStd_Document; this reader does
// not resolve labels, mint receipts, adopt resources, or open a transaction.
#include "DecalLayerDefinition.hxx"
#include "BoundedCurveAttribute.hxx"
#include "CompositeRecipeAttribute.hxx"
#include "FaceImagePersistence.hxx"
#include "OcctDocument.h"
#include "RetainedEdgeTreatmentAttribute.hxx"
#include "RetainedEdgeTreatmentBuild.hxx"
#include "RetainedSolidAttribute.hxx"

#include <BRepBuilderAPI_Copy.hxx>
#include <TDF_Attribute.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelMap.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_UAttribute.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TopoDS_Iterator.hxx>

#include <atomic>
#include <functional>
#include <optional>
#include <string>

// This is the native SYDL image-reference/adoption scheme version. It is
// deliberately independent of the SYDL codec and bake/color/encoder versions.
namespace core3d::decal_layer::image_contract {
inline constexpr std::uint32_t kProducerVersion = 1;
inline bool Supported(const ImageRef& value) noexcept {
    return core3d::decal_layer::Valid(value)
        && value.producerVersion == kProducerVersion;
}
} // namespace image_contract

namespace core3d::decal_layer::persistence {

inline const Standard_GUID& MarkerID() {
    static const Standard_GUID value("E4D1A9E1-724B-44B2-903D-31E501D41001"); return value;
}
inline const Standard_GUID& VersionID() {
    static const Standard_GUID value("E4D1A9E1-724B-44B2-903D-31E501D41002"); return value;
}
inline const Standard_GUID& LayerCountID() {
    static const Standard_GUID value("E4D1A9E1-724B-44B2-903D-31E501D41003"); return value;
}
inline const Standard_GUID& ChunkCountID() {
    static const Standard_GUID value("E4D1A9E1-724B-44B2-903D-31E501D41004"); return value;
}
inline const Standard_GUID& DigestID() {
    static const Standard_GUID value("E4D1A9E1-724B-44B2-903D-31E501D41005"); return value;
}
inline constexpr int RecordTag = 279;
inline constexpr Standard_Size ChunkCharacters = 256;
inline constexpr Standard_Size MaximumChunks = 65'536; // exactly 8 MiB as hex
inline constexpr int MaximumVisitedLabels = 65'536;

enum class ReadState : std::uint8_t { Absent = 0, Present = 1, Malformed = 2 };

inline bool EncodeHex(const std::vector<std::uint8_t>& bytes,
                      std::string& output) noexcept {
    output.clear();
    try {
        if (bytes.empty() || bytes.size() > kMaximumBytes) return false;
        static constexpr char digits[] = "0123456789abcdef";
        output.reserve(bytes.size() * 2);
        for (std::uint8_t byte : bytes) {
            output.push_back(digits[byte >> 4]); output.push_back(digits[byte & 15]);
        }
        return true;
    } catch (...) { output.clear(); return false; }
}
inline bool DecodeHex(const std::string& text,
                      std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (text.empty() || (text.size() & 1) != 0
            || text.size() > kMaximumBytes * 2) return false;
        output.reserve(text.size() / 2);
        for (std::size_t index = 0; index < text.size(); index += 2) {
            unsigned byte = 0;
            for (unsigned half = 0; half < 2; ++half) {
                const char c = text[index + half];
                if (c < '0' || (c > '9' && (c < 'a' || c > 'f'))) return false;
                byte = byte * 16 + unsigned(c <= '9' ? c - '0' : c - 'a' + 10);
            }
            output.push_back(std::uint8_t(byte));
        }
        return true;
    } catch (...) { output.clear(); return false; }
}
inline bool DigestHex(const std::vector<std::uint8_t>& bytes,
                      std::string& output) noexcept {
    output.clear(); Digest digest{};
    if (!Hash(bytes, digest)) return false;
    static constexpr char digits[] = "0123456789abcdef";
    try {
        output.reserve(64);
        for (std::uint8_t byte : digest) {
            output.push_back(digits[byte >> 4]); output.push_back(digits[byte & 15]);
        }
        return true;
    } catch (...) { output.clear(); return false; }
}
inline bool HasSchemaAttribute(const TDF_Label& label) noexcept {
    try {
        Handle(TDF_Attribute) ignored;
        return label.FindAttribute(MarkerID(), ignored)
            || label.FindAttribute(VersionID(), ignored)
            || label.FindAttribute(LayerCountID(), ignored)
            || label.FindAttribute(ChunkCountID(), ignored)
            || label.FindAttribute(DigestID(), ignored);
    } catch (...) { return true; }
}

inline ReadState Read(const Handle(TDocStd_Document)& document,
                      const TDF_Label& owner, Definition& output,
                      std::vector<std::uint8_t>* canonicalBytes = nullptr,
                      TDF_Label* recordLabel = nullptr) noexcept {
    output = {};
    if (canonicalBytes) canonicalBytes->clear();
    if (recordLabel) *recordLabel = {};
    try {
        if (document.IsNull() || document->GetData().IsNull() || owner.IsNull()
            || owner.Data() != document->GetData() || HasSchemaAttribute(owner))
            return ReadState::Malformed;
        TDF_Label record; int visited = 0;
        for (TDF_ChildIterator child(owner, Standard_True); child.More(); child.Next()) {
            if (++visited > MaximumVisitedLabels) return ReadState::Malformed;
            if (!HasSchemaAttribute(child.Value())) continue;
            Handle(TDF_Attribute) marker;
            if (!child.Value().Father().IsEqual(owner)
                || child.Value().Tag() != RecordTag
                || !child.Value().FindAttribute(MarkerID(), marker)
                || Handle(TDataStd_UAttribute)::DownCast(marker).IsNull()
                || !record.IsNull()) return ReadState::Malformed;
            record = child.Value();
        }
        if (record.IsNull()) return ReadState::Absent;
        Handle(TDataStd_Integer) version, layers, chunks;
        Handle(TDataStd_AsciiString) digest;
        if (!record.FindAttribute(VersionID(), version)
            || !record.FindAttribute(LayerCountID(), layers)
            || !record.FindAttribute(ChunkCountID(), chunks)
            || !record.FindAttribute(DigestID(), digest)
            || version.IsNull() || layers.IsNull() || chunks.IsNull() || digest.IsNull()
            || version->Get() != 1 || layers->Get() <= 0
            || layers->Get() > Standard_Integer(kMaximumLayers)
            || chunks->Get() <= 0 || chunks->Get() > MaximumChunks)
            return ReadState::Malformed;
        int attributes = 0;
        for (TDF_AttributeIterator attribute(record); attribute.More(); attribute.Next()) {
            const Standard_GUID& id = attribute.Value()->ID();
            if (id != MarkerID() && id != VersionID() && id != LayerCountID()
                && id != ChunkCountID() && id != DigestID()) return ReadState::Malformed;
            ++attributes;
        }
        const std::string expectedDigest = digest->Get().ToCString();
        if (attributes != 5 || expectedDigest.size() != 64) return ReadState::Malformed;
        for (char value : expectedDigest)
            if (!((value >= '0' && value <= '9') || (value >= 'a' && value <= 'f')))
                return ReadState::Malformed;
        std::string hex;
        if (std::size_t(chunks->Get()) > kMaximumBytes * 2 / ChunkCharacters + 1)
            return ReadState::Malformed;
        hex.reserve(std::size_t(chunks->Get()) * ChunkCharacters);
        for (Standard_Integer index = 1; index <= chunks->Get(); ++index) {
            const TDF_Label chunk = record.FindChild(index, Standard_False);
            Handle(TDataStd_AsciiString) text; int chunkAttributes = 0;
            if (chunk.IsNull() || !chunk.FindAttribute(TDataStd_AsciiString::GetID(), text)
                || text.IsNull()) return ReadState::Malformed;
            for (TDF_AttributeIterator attribute(chunk); attribute.More(); attribute.Next()) {
                if (attribute.Value()->ID() != TDataStd_AsciiString::GetID())
                    return ReadState::Malformed;
                ++chunkAttributes;
            }
            const std::string value = text->Get().ToCString();
            if (chunkAttributes != 1 || value.empty()
                || value.size() > std::size_t(ChunkCharacters)
                || (index != chunks->Get() && value.size() != std::size_t(ChunkCharacters)))
                return ReadState::Malformed;
            for (char c : value)
                if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')))
                    return ReadState::Malformed;
            hex += value;
        }
        int directChildren = 0;
        for (TDF_ChildIterator child(record, Standard_False); child.More(); child.Next()) {
            if (++directChildren > MaximumVisitedLabels
                || child.Value().Tag() > chunks->Get()) return ReadState::Malformed;
            int nested = 0;
            for (TDF_ChildIterator below(child.Value(), Standard_True); below.More(); below.Next())
                if (++nested > MaximumVisitedLabels || below.Value().HasAttribute())
                    return ReadState::Malformed;
        }
        std::vector<std::uint8_t> bytes; Definition value; Refusal refusal;
        std::string actualDigest;
        if (!DecodeHex(hex, bytes) || !Decode(bytes, value, refusal)
            || value.layers.size() != std::size_t(layers->Get())
            || !DigestHex(bytes, actualDigest) || actualDigest != expectedDigest)
            return ReadState::Malformed;
        output = std::move(value);
        if (canonicalBytes) *canonicalBytes = std::move(bytes);
        if (recordLabel) *recordLabel = record;
        return ReadState::Present;
    } catch (...) {
        output = {};
        if (canonicalBytes) canonicalBytes->clear();
        if (recordLabel) *recordLabel = {};
        return ReadState::Malformed;
    }
}

// Export admission walks the exact selected/all root closure. It follows
// occurrence references to their definitions and visits every assembly leaf;
// an invalid/cyclic/oversized graph cannot prove absence and therefore fails
// closed. The limits match native import's bounded assembly census.
enum class ExportClosureState : std::uint8_t {
    Absent = 0,
    ContainsRecord = 1,
    Unproven = 2,
    Cancelled = 3,
};

inline constexpr Standard_Size MaximumExportTraversalNodes = 32'768;
inline constexpr Standard_Size MaximumExportDefinitions = 4'096;
inline constexpr Standard_Size MaximumExportOccurrenceDepth = 128;

struct ExportTraversalFrame {
    TDF_Label label;
    Standard_Size depth = 0;
    bool leaving = false;
};

template <typename IsCancelled>
inline ExportClosureState ReadExportClosure(
    const Handle(TDocStd_Document)& document,
    const TDF_LabelSequence& roots,
    IsCancelled&& isCancelled) noexcept {
    try {
        if (document.IsNull() || document->GetData().IsNull()) {
            return ExportClosureState::Unproven;
        }

        std::vector<ExportTraversalFrame> stack;
        stack.reserve(64);
        for (Standard_Integer index = roots.Length(); index >= 1; --index) {
            if (isCancelled()) return ExportClosureState::Cancelled;
            const TDF_Label& root = roots.Value(index);
            if (root.IsNull() || root.Data() != document->GetData()
                || stack.size() >= MaximumExportTraversalNodes) {
                return ExportClosureState::Unproven;
            }
            stack.push_back({root, 0, false});
        }

        TDF_LabelMap activePath;
        TDF_LabelMap visitedLabels;
        Standard_Size traversalNodes = 0;
        Standard_Size definitionCount = 0;
        while (!stack.empty()) {
            if (isCancelled()) return ExportClosureState::Cancelled;
            const ExportTraversalFrame frame = stack.back();
            stack.pop_back();
            if (frame.leaving) {
                activePath.Remove(frame.label);
                visitedLabels.Add(frame.label);
                continue;
            }
            if (visitedLabels.Contains(frame.label)) continue;
            if (frame.label.IsNull()
                || frame.label.Data() != document->GetData()
                || activePath.Contains(frame.label)
                || frame.depth > MaximumExportOccurrenceDepth
                || ++traversalNodes > MaximumExportTraversalNodes) {
                return ExportClosureState::Unproven;
            }

            activePath.Add(frame.label);
            stack.push_back({frame.label, frame.depth, true});
            if (XCAFDoc_ShapeTool::IsReference(frame.label)
                || XCAFDoc_ShapeTool::IsComponent(frame.label)) {
                TDF_Label referred;
                if (!XCAFDoc_ShapeTool::GetReferredShape(frame.label, referred)
                    || referred.IsNull()
                    || referred.Data() != document->GetData()
                    || !XCAFDoc_ShapeTool::IsShape(referred)) {
                    return ExportClosureState::Unproven;
                }
                stack.push_back({referred, frame.depth + 1, false});
                continue;
            }
            if (!XCAFDoc_ShapeTool::IsShape(frame.label)
                || ++definitionCount > MaximumExportDefinitions) {
                return ExportClosureState::Unproven;
            }

            Definition ignored;
            if (Read(document, frame.label, ignored) != ReadState::Absent) {
                return ExportClosureState::ContainsRecord;
            }
            if (isCancelled()) return ExportClosureState::Cancelled;
            if (!XCAFDoc_ShapeTool::IsAssembly(frame.label)) continue;

            std::vector<TDF_Label> components;
            for (TDF_ChildIterator child(frame.label, Standard_False);
                 child.More(); child.Next()) {
                if (isCancelled()) return ExportClosureState::Cancelled;
                const TDF_Label& component = child.Value();
                if (!XCAFDoc_ShapeTool::IsComponent(component)) continue;
                if (components.size() >= MaximumExportTraversalNodes
                    || traversalNodes > MaximumExportTraversalNodes
                    || components.size()
                        > MaximumExportTraversalNodes - traversalNodes
                    || stack.size() > MaximumExportTraversalNodes
                    || components.size()
                        > MaximumExportTraversalNodes - stack.size()) {
                    return ExportClosureState::Unproven;
                }
                components.push_back(component);
            }
            for (auto component = components.rbegin();
                 component != components.rend(); ++component) {
                stack.push_back({*component, frame.depth + 1, false});
            }
        }
        return ExportClosureState::Absent;
    } catch (...) {
        return ExportClosureState::Unproven;
    }
}

// Portion 3 owns commands and calls this only on a fresh/cleared record label.
inline bool WriteChunks(const TDF_Label& record, const std::string& hex,
                        Standard_Integer layerCount,
                        const std::string& digest) noexcept {
    try {
        const std::size_t count = (hex.size() + ChunkCharacters - 1) / ChunkCharacters;
        if (record.IsNull() || hex.empty() || count == 0 || count > MaximumChunks
            || layerCount <= 0 || layerCount > Standard_Integer(kMaximumLayers)
            || digest.size() != 64) return false;
        TDataStd_UAttribute::Set(record, MarkerID());
        TDataStd_Integer::Set(record, VersionID(), 1);
        TDataStd_Integer::Set(record, LayerCountID(), layerCount);
        TDataStd_Integer::Set(record, ChunkCountID(), Standard_Integer(count));
        TDataStd_AsciiString::Set(record, DigestID(), TCollection_AsciiString(digest.c_str()));
        for (std::size_t index = 0; index < count; ++index) {
            const std::string part = hex.substr(index * ChunkCharacters, ChunkCharacters);
            TDataStd_AsciiString::Set(
                record.FindChild(Standard_Integer(index + 1), Standard_True),
                TCollection_AsciiString(part.c_str()));
        }
        return true;
    } catch (...) { return false; }
}

} // namespace core3d::decal_layer::persistence

namespace core3d::decal_layer::source {

//! Fresh, content-derived SYDL source tuple. Full digests are retained beside
//! the three shortened persisted tokens so prefix equality is never authority.
struct Witness final {
    OwnerKey owner{};
    UUID sourceRecipe{};
    std::uint64_t sourceRevision = 0;
    std::uint64_t geometryRevision = 0;
    std::uint64_t placementRevision = 0;
    Digest sourceProof{};
    Digest recipeDigest{};
    Digest geometryDigest{};
    Digest placementDigest{};
};

inline std::uint64_t Prefix64(const Digest& value) noexcept {
    std::uint64_t result = 0;
    for (unsigned index = 0; index < 8; ++index)
        result |= std::uint64_t(value[index]) << (8U * index);
    return result == 0 ? 1 : result;
}

inline bool ExactMatch(const Definition& saved,
                       const Witness& observed) noexcept {
    return saved.owner == observed.owner
        && saved.sourceRecipe == observed.sourceRecipe
        && saved.sourceRevision == observed.sourceRevision
        && saved.geometryRevision == observed.geometryRevision
        && saved.placementRevision == observed.placementRevision
        && saved.sourceProof == observed.sourceProof;
}

namespace detail {
struct InputGeometry final {
    std::uint8_t kind = 0;
    UUID identity{};
    std::uint32_t slot = 0;
    Digest geometry{};
};

inline bool Cancelled(const std::function<bool()>& cancelled) noexcept {
    try { return cancelled && cancelled(); } catch (...) { return true; }
}

inline bool GeometryDigest(const TopoDS_Shape& shape,
                           retained_edge_treatment::ReplayBudget& budget,
                           const std::function<bool()>& cancelled,
                           Digest& output) noexcept {
    output = {};
    try {
        if (shape.IsNull() || Cancelled(cancelled)
            || !retained_edge_treatment::detail::ChargeTopology(shape, budget))
            return false;
        BRepBuilderAPI_Copy copier(shape, Standard_True, Standard_False);
        if (!copier.IsDone() || copier.Shape().IsNull() || Cancelled(cancelled))
            return false;
        TopoDS_Shape detached = copier.Shape();
        std::vector<std::pair<TopoDS_Shape, unsigned>> pending{{detached, 0}};
        while (!pending.empty()) {
            auto current = std::move(pending.back()); pending.pop_back();
            if (current.second > 128 || !budget.visit(1)
                || Cancelled(cancelled)) return false;
            current.first.Free(Standard_True);
            current.first.Modified(Standard_False);
            current.first.Checked(Standard_False);
            for (TopoDS_Iterator child(current.first); child.More(); child.Next()) {
                if (pending.size()
                    >= retained_topology_budget::MaximumTopologyVisits)
                    return false;
                pending.emplace_back(child.Value(), current.second + 1);
            }
        }
        if (Cancelled(cancelled)
            || !retained_edge_treatment::detail::CommitGeometry(
                detached, budget, output)
            || Cancelled(cancelled) || !Nonzero(output)) return false;
        return true;
    } catch (...) { output = {}; return false; }
}

inline void WriteTransform(face_image::detail::Writer& writer,
                           const gp_Trsf& transform) {
    for (Standard_Integer row = 1; row <= 3; ++row)
        for (Standard_Integer column = 1; column <= 4; ++column)
            writer.real(transform.Value(row, column));
}
} // namespace detail

//! Pure read-side capture shared by ordinary/private consumers and the future
//! Portion-3 issuer. It allocates detached temporary shapes only; no OCAF
//! command, ID, resource, record or cache is created or changed.
inline bool CaptureSource(
    const Handle(OcctDocument)& wrapper,
    const OwnerKey& requestedOwner,
    retained_edge_treatment::ReplayBudget& budget,
    const std::function<bool()>& cancelled,
    Witness& output) noexcept {
    output = {};
    try {
        if (wrapper.IsNull() || !retained_recipe::Valid(requestedOwner)
            || detail::Cancelled(cancelled)) return false;
        const Handle(TDocStd_Document)& document = wrapper->Document();
        if (document.IsNull() || document->GetData().IsNull()) return false;
        // Resolve through the document-owned identity index before any family
        // dispatch. This proves exactly one free simple owner in this document.
        TDF_Label owner;
        if (!face_image::owner::ResolveOwnerLabel(
                document, requestedOwner, owner)
            || owner.IsNull() || owner.Data() != document->GetData()
            || detail::Cancelled(cancelled)) return false;

        std::vector<retained_solid::Record> retainedRecords;
        std::vector<bounded_curve::Record> curveRecords;
        std::vector<composite_recipe::Record> compositeRecords;
        if (!retained_solid::ReadAll(document, retainedRecords)
            || detail::Cancelled(cancelled)
            || !bounded_curve::ReadAll(document, curveRecords)
            || detail::Cancelled(cancelled)) return false;
        std::size_t legacyBytes = 0, curveBytes = 0;
        for (const auto& record : retainedRecords) {
            if (!record.value
                || record.value->bytes.size()
                    > retained_solid::MaximumAggregateEnvelopeBytes - legacyBytes)
                return false;
            legacyBytes += record.value->bytes.size();
        }
        for (const auto& record : curveRecords) {
            if (!record.value
                || record.value->definitionBytes.size()
                    > bounded_curve::MaximumDocumentAggregateBytes - curveBytes)
                return false;
            curveBytes += record.value->definitionBytes.size();
        }
        if (!composite_recipe::ReadAll(document, compositeRecords,
                legacyBytes, curveBytes, curveRecords.size())
            || detail::Cancelled(cancelled)) return false;

        const retained_solid::Record* retained = nullptr;
        const composite_recipe::Record* composite = nullptr;
        for (const auto& record : retainedRecords)
            if (record.owner.IsEqual(owner)) {
                if (retained) return false; retained = &record;
            }
        for (const auto& record : compositeRecords)
            if (record.owner.IsEqual(owner)) {
                if (composite) return false; composite = &record;
            }
        if ((retained != nullptr) == (composite != nullptr)) return false;

        std::uint8_t family = retained ? 1 : 2;
        std::vector<std::uint8_t> rootBytes;
        Digest recipeDigest{};
        TopoDS_Shape current;
        std::vector<detail::InputGeometry> inputs;
        if (retained) {
            if (!retained->value || retained->value->base.IsNull()
                || retained->current.IsNull()) return false;
            rootBytes = retained->value->bytes;
            if (!retained_solid::Hash(rootBytes, recipeDigest)) return false;
            const auto identity = retained_boolean::Identities(
                retained->value->envelope);
            output.sourceRecipe = identity.derivedFeature;
            detail::InputGeometry input;
            input.kind = 1; input.identity = identity.sourceFeature;
            if (!detail::GeometryDigest(retained->value->base, budget,
                    cancelled, input.geometry)) return false;
            inputs.push_back(input); current = retained->current;
        } else {
            if (!composite->value || composite->current.IsNull()) return false;
            rootBytes = composite->value->bytes;
            if (!composite_recipe::Hash(rootBytes, recipeDigest)) return false;
            output.sourceRecipe = composite->value->definition.outputNode;
            for (const composite_recipe::Node& node :
                 composite->value->definition.nodes) {
                const auto* source = std::get_if<composite_recipe::SourceNode>(
                    &node.value);
                if (!source) continue;
                if (source->shapeSlot >= composite->value->sourceShapes.size())
                    return false;
                detail::InputGeometry input;
                input.kind = 2; input.identity = source->node;
                input.slot = source->shapeSlot;
                if (!detail::GeometryDigest(
                        composite->value->sourceShapes[source->shapeSlot],
                        budget, cancelled, input.geometry)) return false;
                inputs.push_back(input);
            }
            if (inputs.empty()) return false;
            current = composite->current;
        }
        if (!retained_recipe::Nonzero(output.sourceRecipe)
            || !Nonzero(recipeDigest) || detail::Cancelled(cancelled)) return false;

        std::vector<retained_edge_treatment::Record> treatments;
        std::vector<retained_edge_treatment::RecordR2> treatmentsR2;
        retained_edge_treatment::Refusal treatmentRefusal{};
        std::size_t priorBytes = legacyBytes;
        for (const auto& record : compositeRecords) {
            if (!record.value
                || record.value->bytes.size() > 8U * 1024U * 1024U - priorBytes)
                return false;
            priorBytes += record.value->bytes.size();
        }
        if (!retained_edge_treatment::ReadAllMixed(document, priorBytes,
                treatments, treatmentsR2, treatmentRefusal)
            || detail::Cancelled(cancelled)) return false;
        const retained_edge_treatment::Record* treatment = nullptr;
        const retained_edge_treatment::RecordR2* treatmentR2 = nullptr;
        for (const auto& record : treatments)
            if (record.owner.IsEqual(owner)) {
                if (treatment) return false; treatment = &record;
            }
        for (const auto& record : treatmentsR2)
            if (record.owner.IsEqual(owner)) {
                if (treatmentR2) return false; treatmentR2 = &record;
            }
        if (treatment && treatmentR2) return false;
        std::uint8_t arm = 0;
        std::vector<std::uint8_t> treatmentBytes;
        if (treatment) {
            if (!treatment->value
                || !(treatment->value->definition.owner == requestedOwner)
                || treatment->value->definition.base.sourceRecipeDigest
                    != recipeDigest) return false;
            arm = 1; treatmentBytes = treatment->value->bytes;
            std::vector<std::uint8_t> canonical;
            retained_edge_treatment::Refusal encodeRefusal{};
            if (!retained_edge_treatment::Encode(
                    treatment->value->definition, canonical, encodeRefusal)
                || canonical != treatmentBytes) return false;
            detail::InputGeometry input;
            input.kind = 3; input.identity = output.sourceRecipe;
            if (!detail::GeometryDigest(treatment->value->base, budget,
                    cancelled, input.geometry)) return false;
            inputs.push_back(input);
        } else if (treatmentR2) {
            if (!treatmentR2->value
                || !(treatmentR2->value->definition.owner == requestedOwner))
                return false;
            const auto* base = std::get_if<
                retained_edge_treatment::r2::BooleanBaseBinding>(
                    &treatmentR2->value->definition.base);
            if (!base || base->sourceRecipeDigest != recipeDigest
                || (family == 1 && base->format
                    != retained_edge_treatment::r2::PrefixFormat::SYRS)
                || (family == 2 && base->format
                    == retained_edge_treatment::r2::PrefixFormat::SYRS))
                return false;
            arm = 2; treatmentBytes = treatmentR2->value->bytes;
            std::vector<std::uint8_t> canonical;
            retained_edge_treatment::Refusal encodeRefusal{};
            if (!retained_edge_treatment::r2::Encode(
                    treatmentR2->value->definition, canonical, encodeRefusal)
                || canonical != treatmentBytes) return false;
            detail::InputGeometry input;
            input.kind = 3; input.identity = output.sourceRecipe;
            if (!detail::GeometryDigest(treatmentR2->value->base, budget,
                    cancelled, input.geometry)) return false;
            inputs.push_back(input);
        }

        Digest currentGeometry{};
        if (!detail::GeometryDigest(current, budget, cancelled,
                currentGeometry)) return false;
        Standard_Real metersPerUnit = 0.0;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document, metersPerUnit)
            || !std::isfinite(metersPerUnit) || metersPerUnit <= 0.0)
            return false;
        gp_Trsf authored;
        if (!wrapper->TryObjectTransformForLabel(owner, authored)) return false;
        const gp_Trsf location = current.Location().Transformation();

        face_image::detail::Writer placement;
        placement.raw(reinterpret_cast<const std::uint8_t*>("E4SP"), 4);
        placement.integer(1, 4);
        placement.raw(requestedOwner.document);
        placement.raw(requestedOwner.entity);
        placement.raw(requestedOwner.definition);
        placement.real(metersPerUnit);
        detail::WriteTransform(placement, authored);
        detail::WriteTransform(placement, location);
        Digest placementDigest{};
        if (!placement.ok
            || !face_image::HashFaceImageBytes(
                placement.bytes, placementDigest)
            || !Nonzero(placementDigest)) return false;

        output.owner = requestedOwner;
        output.recipeDigest = recipeDigest;
        output.geometryDigest = currentGeometry;
        output.placementDigest = placementDigest;
        output.sourceRevision = Prefix64(recipeDigest);
        output.geometryRevision = Prefix64(currentGeometry);
        output.placementRevision = Prefix64(placementDigest);

        face_image::detail::Writer source;
        source.raw(reinterpret_cast<const std::uint8_t*>("E4SW"), 4);
        source.integer(1, 4); source.integer(family, 1);
        source.raw(output.owner.document); source.raw(output.owner.entity);
        source.raw(output.owner.definition); source.raw(output.sourceRecipe);
        source.integer(output.sourceRevision, 8);
        source.integer(output.geometryRevision, 8);
        source.integer(output.placementRevision, 8);
        source.real(metersPerUnit); source.raw(recipeDigest);
        source.raw(currentGeometry); source.raw(placementDigest);
        source.integer(arm, 1);
        source.integer(treatmentBytes.size(), 8);
        if (!treatmentBytes.empty())
            source.raw(treatmentBytes.data(), treatmentBytes.size());
        source.integer(inputs.size(), 4);
        for (const auto& input : inputs) {
            source.integer(input.kind, 1); source.raw(input.identity);
            source.integer(input.slot, 4); source.raw(input.geometry);
        }
        if (!source.ok || source.bytes.size() > kMaximumBytes
            || detail::Cancelled(cancelled)
            || !face_image::HashFaceImageBytes(
                source.bytes, output.sourceProof)
            || !Nonzero(output.sourceProof)
            || detail::Cancelled(cancelled)) {
            output = {}; return false;
        }
        return true;
    } catch (...) { output = {}; return false; }
}

} // namespace core3d::decal_layer::source
