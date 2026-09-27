#pragma once
#include "PatternDefinition.hxx"

#include <cstring>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_ByteArray.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_TagSource.hxx>
#include <TDocStd_Document.hxx>

namespace core3d::pattern {
namespace detail {
inline void U64(std::vector<std::uint8_t>& out, std::uint64_t value) {
    for (unsigned index = 0; index < 8; ++index) out.push_back(std::uint8_t(value >> (8 * index)));
}
inline void U32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    for (unsigned index = 0; index < 4; ++index) out.push_back(std::uint8_t(value >> (8 * index)));
}
inline void Scalar(std::vector<std::uint8_t>& out, double value) {
    std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits)); U64(out, bits);
}
inline void UUIDBytes(std::vector<std::uint8_t>& out, const UUID& value) {
    out.insert(out.end(), value.begin(), value.end());
}
inline bool Hash(const std::vector<std::uint8_t>& bytes, retained_recipe::Digest& digest) noexcept {
    return retained_solid::Hash(bytes, digest);
}
} // namespace detail

inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!Valid(value)) return false;
        std::vector<std::uint8_t> bytes{'S','Y','P','T',1,0,0,0};
        for (const UUID& id : {value.owner.document, value.owner.entity, value.owner.definition,
                               value.feature, value.source.document, value.source.entity,
                               value.source.definition, value.source.sourceFeature}) detail::UUIDBytes(bytes, id);
        bytes.push_back(std::uint8_t(value.kind)); bytes.push_back(std::uint8_t(value.columnAxis));
        bytes.push_back(std::uint8_t(value.rowAxis)); bytes.push_back(0);
        detail::U32(bytes, value.rowCount); detail::U32(bytes, value.columnCount);
        detail::Scalar(bytes, value.rowSpacing); detail::Scalar(bytes, value.columnSpacing);
        detail::Scalar(bytes, value.sweepRadians);
        for (double scalar : value.radialPivotLocal) detail::Scalar(bytes, scalar);
        for (double scalar : value.sourceFrame) detail::Scalar(bytes, scalar);
        detail::U64(bytes, value.issuance.nextLocalID);
        detail::U32(bytes, std::uint32_t(value.issuance.retiredLocalIDs.size()));
        detail::U32(bytes, std::uint32_t(value.members.size()));
        detail::U32(bytes, std::uint32_t(value.removals.size()));
        for (std::uint64_t retired : value.issuance.retiredLocalIDs) detail::U64(bytes, retired);
        const auto member = [&](const Member& item) {
            detail::UUIDBytes(bytes, item.identity); detail::U64(bytes, item.localID);
            detail::U32(bytes, item.coordinate.row); detail::U32(bytes, item.coordinate.column);
            bytes.push_back(std::uint8_t(item.state)); bytes.push_back(0); bytes.push_back(0); bytes.push_back(0);
        };
        for (const Member& item : value.members) member(item);
        for (const Member& item : value.removals) member(item);
        if (bytes.size() > MaximumEnvelopeBytes - 32) return false;
        retained_recipe::Digest digest{}; if (!detail::Hash(bytes, digest)) return false;
        bytes.insert(bytes.end(), digest.begin(), digest.end()); output = std::move(bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output) noexcept {
    output = {};
    try {
        constexpr std::size_t fixed = 8 + 8 * 16 + 4 + 8 + 8 * 3 + 8 * 3 + 8 * 16 + 8 + 12;
        if (bytes.size() < fixed + 32 || bytes.size() > MaximumEnvelopeBytes
            || std::memcmp(bytes.data(), "SYPT\1\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        retained_recipe::Digest expected{}, actual{}; if (!detail::Hash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (actual != expected) return false;
        std::size_t at = 8;
        const auto need = [&](std::size_t count) { return at <= body.size() && count <= body.size() - at; };
        const auto uuid = [&](UUID& value) { if (!need(16)) return false; std::copy_n(body.begin() + at, 16, value.begin()); at += 16; return true; };
        const auto u64 = [&](std::uint64_t& value) { if (!need(8)) return false; value = 0; for (unsigned i=0;i<8;++i)value|=std::uint64_t(body[at++])<<(8*i); return true; };
        const auto u32 = [&](std::uint32_t& value) { if (!need(4)) return false; value = 0; for (unsigned i=0;i<4;++i)value|=std::uint32_t(body[at++])<<(8*i); return true; };
        const auto scalar = [&](double& value) { std::uint64_t bits = 0; if (!u64(bits)) return false; std::memcpy(&value, &bits, 8); return std::isfinite(value); };
        Definition value;
        if (!uuid(value.owner.document) || !uuid(value.owner.entity) || !uuid(value.owner.definition)
            || !uuid(value.feature) || !uuid(value.source.document) || !uuid(value.source.entity)
            || !uuid(value.source.definition) || !uuid(value.source.sourceFeature) || !need(4)) return false;
        value.kind = Kind(body[at++]); value.columnAxis = Axis(body[at++]); value.rowAxis = Axis(body[at++]);
        if (body[at++] != 0 || !u32(value.rowCount) || !u32(value.columnCount)
            || !scalar(value.rowSpacing) || !scalar(value.columnSpacing) || !scalar(value.sweepRadians)) return false;
        for (double& number : value.radialPivotLocal) if (!scalar(number)) return false;
        for (double& number : value.sourceFrame) if (!scalar(number)) return false;
        std::uint32_t retiredCount = 0, memberCount = 0, removalCount = 0;
        if (!u64(value.issuance.nextLocalID) || !u32(retiredCount) || !u32(memberCount) || !u32(removalCount)
            || retiredCount > MaximumInstances * 8u || memberCount > MaximumInstances
            || removalCount > MaximumInstances * 8u) return false;
        value.issuance.retiredLocalIDs.reserve(retiredCount);
        for (std::uint32_t index=0; index<retiredCount; ++index) { std::uint64_t id=0; if(!u64(id))return false; value.issuance.retiredLocalIDs.push_back(id); }
        const auto readMember = [&](Member& item) {
            if (!uuid(item.identity) || !u64(item.localID) || !u32(item.coordinate.row)
                || !u32(item.coordinate.column) || !need(4)) return false;
            item.state = MemberState(body[at++]); return body[at++] == 0 && body[at++] == 0 && body[at++] == 0;
        };
        value.members.resize(memberCount); for (Member& item : value.members) if (!readMember(item)) return false;
        value.removals.resize(removalCount); for (Member& item : value.removals) if (!readMember(item)) return false;
        if (at != body.size() || !Valid(value)) return false;
        std::vector<std::uint8_t> exact; if (!Encode(value, exact) || exact != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

inline constexpr int DocumentRootTag = 71;
inline const char* DocumentMarker = "Shapeyard retained patterns v1";

struct Record {
    TDF_Label label;
    Definition definition;
    std::vector<std::uint8_t> bytes;
};

inline bool ReadByteArray(const TDF_Label& label, std::vector<std::uint8_t>& output) noexcept {
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
        Handle(TDataStd_AsciiString) marker;
        if (!root.FindAttribute(TDataStd_AsciiString::GetID(), marker) || marker.IsNull()
            || marker->Get().ToCString() != std::string(DocumentMarker)) return false;
        std::vector<Record> staged; std::set<UUID> features; std::size_t aggregate = 0;
        for (TDF_ChildIterator child(root, Standard_False); child.More(); child.Next()) {
            Record record; record.label = child.Value();
            if (!ReadByteArray(record.label, record.bytes) || !Decode(record.bytes, record.definition)
                || !features.insert(record.definition.feature).second
                || record.bytes.size() > MaximumDocumentPatternBytes - aggregate) return false;
            aggregate += record.bytes.size(); staged.push_back(std::move(record));
        }
        output = std::move(staged); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool ReadFeature(const Handle(TDocStd_Document)& document, const UUID& feature,
                        Record& output) noexcept {
    output = {};
    std::vector<Record> records; if (!ReadAll(document, records)) return false;
    for (Record& record : records) if (record.definition.feature == feature) {
        output = std::move(record); return true;
    }
    return true;
}

// Must run inside the caller's one OCAF command. Existing records are replaced
// in place, so undo/redo covers the recipe and all member-label rebuilds atomically.
inline bool Stage(const Handle(TDocStd_Document)& document, const Definition& definition,
                  Record& output) noexcept {
    output = {};
    try {
        if (document.IsNull() || !document->HasOpenCommand()) return false;
        std::vector<std::uint8_t> bytes; if (!Encode(definition, bytes)) return false;
        std::vector<Record> before; if (!ReadAll(document, before)) return false;
        std::size_t aggregate = bytes.size(); TDF_Label target;
        for (const Record& record : before) {
            if (record.definition.feature == definition.feature) target = record.label;
            else if (record.bytes.size() > MaximumDocumentPatternBytes - aggregate) return false;
            else aggregate += record.bytes.size();
        }
        TDF_Label root = document->Main().FindChild(DocumentRootTag, Standard_True);
        Handle(TDataStd_AsciiString) marker;
        if (root.FindAttribute(TDataStd_AsciiString::GetID(), marker)) {
            if (marker.IsNull() || marker->Get().ToCString() != std::string(DocumentMarker)) return false;
        } else TDataStd_AsciiString::Set(root, TCollection_AsciiString(DocumentMarker));
        if (target.IsNull()) target = TDF_TagSource::NewChild(root);
        target.ForgetAttribute(TDataStd_ByteArray::GetID());
        const auto attribute = TDataStd_ByteArray::Set(target, 0,
            Standard_Integer(bytes.size()) - 1, Standard_False);
        if (attribute.IsNull()) return false;
        for (std::size_t index = 0; index < bytes.size(); ++index)
            attribute->SetValue(Standard_Integer(index), Standard_Byte(bytes[index]));
        Record readback; if (!ReadFeature(document, definition.feature, readback)
            || readback.label.IsNull() || readback.bytes != bytes) return false;
        output = std::move(readback); return true;
    } catch (...) { output = {}; return false; }
}
} // namespace core3d::pattern
