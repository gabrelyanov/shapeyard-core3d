#pragma once

#include "FeaturePatternDefinition.hxx"
#include "PatternPersistence.hxx"

#include <cstring>
#include <set>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_ByteArray.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_TagSource.hxx>
#include <TDocStd_Document.hxx>

namespace core3d::feature_pattern {
namespace persistence_detail {
inline void U64(std::vector<std::uint8_t>& output, std::uint64_t value) {
    for (unsigned index = 0; index < 8; ++index)
        output.push_back(std::uint8_t(value >> (8 * index)));
}
inline void U32(std::vector<std::uint8_t>& output, std::uint32_t value) {
    for (unsigned index = 0; index < 4; ++index)
        output.push_back(std::uint8_t(value >> (8 * index)));
}
inline void Scalar(std::vector<std::uint8_t>& output, double value) {
    std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits)); U64(output, bits);
}
inline void UUIDBytes(std::vector<std::uint8_t>& output, const UUID& value) {
    output.insert(output.end(), value.begin(), value.end());
}
} // namespace persistence_detail

inline bool Encode(const Definition& value,
                   std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!Valid(value)) return false;
        std::vector<std::uint8_t> distribution;
        if (!pattern::Encode(value.distribution, distribution)) return false;
        std::vector<std::uint8_t> bytes{'S','Y','F','P',1,0,0,0};
        for (const UUID& identifier : {value.host.document, value.host.entity,
                value.host.definition, value.feature, value.sourceCut.document,
                value.sourceCut.entity, value.sourceCut.definition,
                value.sourceCut.sourceFeature})
            persistence_detail::UUIDBytes(bytes, identifier);
        persistence_detail::U64(bytes, value.sourceCutStepID);
        persistence_detail::Scalar(bytes, value.metersPerUnit);
        persistence_detail::Scalar(bytes, value.minimumHostLigamentMM);
        persistence_detail::U32(bytes, value.expectedBoundarySectionsPerFeature);
        persistence_detail::U32(bytes, std::uint32_t(distribution.size()));
        bytes.insert(bytes.end(), distribution.begin(), distribution.end());
        if (bytes.size() > MaximumEnvelopeBytes - 32) return false;
        retained_recipe::Digest digest{};
        if (!retained_solid::Hash(bytes, digest)) return false;
        bytes.insert(bytes.end(), digest.begin(), digest.end());
        output = std::move(bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes,
                   Definition& output) noexcept {
    output = {};
    try {
        constexpr std::size_t fixed = 8 + 8 * 16 + 8 + 8 + 8 + 4 + 4;
        if (bytes.size() < fixed + 32 || bytes.size() > MaximumEnvelopeBytes
            || std::memcmp(bytes.data(), "SYFP\1\0\0\0", 8) != 0) return false;
        const std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        retained_recipe::Digest expected{}, actual{};
        if (!retained_solid::Hash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin());
        if (actual != expected) return false;
        std::size_t at = 8;
        const auto need = [&](std::size_t count) {
            return at <= body.size() && count <= body.size() - at;
        };
        const auto uuid = [&](UUID& value) {
            if (!need(16)) return false;
            std::copy_n(body.begin() + at, 16, value.begin()); at += 16; return true;
        };
        const auto u64 = [&](std::uint64_t& value) {
            if (!need(8)) return false; value = 0;
            for (unsigned index = 0; index < 8; ++index)
                value |= std::uint64_t(body[at++]) << (8 * index);
            return true;
        };
        const auto u32 = [&](std::uint32_t& value) {
            if (!need(4)) return false; value = 0;
            for (unsigned index = 0; index < 4; ++index)
                value |= std::uint32_t(body[at++]) << (8 * index);
            return true;
        };
        const auto scalar = [&](double& value) {
            std::uint64_t bits = 0; if (!u64(bits)) return false;
            std::memcpy(&value, &bits, sizeof(value)); return std::isfinite(value);
        };
        Definition value; std::uint32_t distributionSize = 0;
        if (!uuid(value.host.document) || !uuid(value.host.entity)
            || !uuid(value.host.definition) || !uuid(value.feature)
            || !uuid(value.sourceCut.document) || !uuid(value.sourceCut.entity)
            || !uuid(value.sourceCut.definition) || !uuid(value.sourceCut.sourceFeature)
            || !u64(value.sourceCutStepID) || !scalar(value.metersPerUnit)
            || !scalar(value.minimumHostLigamentMM)
            || !u32(value.expectedBoundarySectionsPerFeature)
            || !u32(distributionSize) || distributionSize == 0
            || !need(distributionSize) || at + distributionSize != body.size()) return false;
        const std::vector<std::uint8_t> distribution(body.begin() + at, body.end());
        if (!pattern::Decode(distribution, value.distribution) || !Valid(value)) return false;
        std::vector<std::uint8_t> exact;
        if (!Encode(value, exact) || exact != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

inline constexpr int DocumentRootTag = 73;
inline const char* DocumentMarker = "Shapeyard retained cut feature patterns v1";

struct Record {
    TDF_Label label;
    Definition definition;
    std::vector<std::uint8_t> bytes;
};

inline bool ReadBytes(const TDF_Label& label,
                      std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        Handle(TDataStd_ByteArray) attribute;
        if (label.IsNull() || !label.FindAttribute(TDataStd_ByteArray::GetID(), attribute)
            || attribute.IsNull() || attribute->Lower() != 0 || attribute->Upper() < 0
            || std::size_t(attribute->Upper()) >= MaximumEnvelopeBytes) return false;
        output.reserve(std::size_t(attribute->Upper()) + 1);
        for (Standard_Integer index = 0; index <= attribute->Upper(); ++index)
            output.push_back(std::uint8_t(attribute->Value(index)));
        return true;
    } catch (...) { output.clear(); return false; }
}

inline bool ReadAll(const Handle(TDocStd_Document)& document,
                    std::vector<Record>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const TDF_Label root = document->Main().FindChild(DocumentRootTag, Standard_False);
        if (root.IsNull()) return true;
        // Abort or undo rolls back attribute deltas but leaves label nodes.
        // A subtree is logical absence only when its root and every descendant
        // carry no live attribute; any live attribute anywhere makes it
        // nonempty. The inspection is read-only, iterative, and bounded at
        // 100000 labels (the nearby persistence census ceiling); exceeding the
        // bound fails closed. It never creates labels, writes markers, forgets
        // attributes, changes history, or replaces the document.
        const auto attributeFree = [](const TDF_Label& subtree) {
            std::vector<TDF_Label> pending{subtree};
            std::size_t visited = 0;
            while (!pending.empty()) {
                const TDF_Label label = pending.back(); pending.pop_back();
                if (++visited > 100000 || label.HasAttribute()) return false;
                for (TDF_ChildIterator child(label, Standard_False);
                     child.More(); child.Next())
                    pending.push_back(child.Value());
            }
            return true;
        };
        if (attributeFree(root)) return true;
        Handle(TDataStd_AsciiString) marker;
        if (!root.FindAttribute(TDataStd_AsciiString::GetID(), marker)
            || marker.IsNull()
            || marker->Get().ToCString() != std::string(DocumentMarker)) return false;
        std::vector<Record> staged; std::set<UUID> features; std::size_t aggregate = 0;
        for (TDF_ChildIterator child(root, Standard_False); child.More(); child.Next()) {
            if (attributeFree(child.Value())) continue;
            Record record; record.label = child.Value();
            if (!ReadBytes(record.label, record.bytes)
                || !Decode(record.bytes, record.definition)
                || !features.insert(record.definition.feature).second
                || record.bytes.size() > MaximumDocumentBytes - aggregate) return false;
            aggregate += record.bytes.size(); staged.push_back(std::move(record));
        }
        output = std::move(staged); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool ReadFeature(const Handle(TDocStd_Document)& document,
                        const UUID& feature, Record& output) noexcept {
    output = {};
    std::vector<Record> records;
    if (!ReadAll(document, records)) return false;
    for (Record& record : records) if (record.definition.feature == feature) {
        output = std::move(record); return true;
    }
    return true;
}

// The caller stages rebuilt host geometry and every child feature in the same
// already-open command. Exact readback is required before that command commits.
inline bool Stage(const Handle(TDocStd_Document)& document,
                  const Definition& definition, Record& output) noexcept {
    output = {};
    try {
        if (document.IsNull() || !document->HasOpenCommand()) return false;
        std::vector<std::uint8_t> bytes;
        if (!Encode(definition, bytes)) return false;
        std::vector<Record> before;
        if (!ReadAll(document, before)) return false;
        std::size_t aggregate = bytes.size(); TDF_Label target;
        for (const Record& record : before) {
            if (record.definition.feature == definition.feature) target = record.label;
            else if (record.bytes.size() > MaximumDocumentBytes - aggregate) return false;
            else aggregate += record.bytes.size();
        }
        TDF_Label root = document->Main().FindChild(DocumentRootTag, Standard_True);
        Handle(TDataStd_AsciiString) marker;
        if (root.FindAttribute(TDataStd_AsciiString::GetID(), marker)) {
            if (marker.IsNull()
                || marker->Get().ToCString() != std::string(DocumentMarker)) return false;
        } else TDataStd_AsciiString::Set(root, TCollection_AsciiString(DocumentMarker));
        if (target.IsNull()) target = TDF_TagSource::NewChild(root);
        target.ForgetAttribute(TDataStd_ByteArray::GetID());
        const auto attribute = TDataStd_ByteArray::Set(target, 0,
            Standard_Integer(bytes.size()) - 1, Standard_False);
        if (attribute.IsNull()) return false;
        for (std::size_t index = 0; index < bytes.size(); ++index)
            attribute->SetValue(Standard_Integer(index), Standard_Byte(bytes[index]));
        Record readback;
        if (!ReadFeature(document, definition.feature, readback)
            || readback.label.IsNull() || readback.bytes != bytes) return false;
        output = std::move(readback); return true;
    } catch (...) { output = {}; return false; }
}
} // namespace core3d::feature_pattern
