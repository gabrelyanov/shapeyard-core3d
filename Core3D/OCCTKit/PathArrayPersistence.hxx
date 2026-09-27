#pragma once

#include "PathArrayDefinition.hxx"

#include <TDataStd_AsciiString.hxx>
#include <TDataStd_ByteArray.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_TagSource.hxx>
#include <TDocStd_Document.hxx>

#include <algorithm>
#include <cstring>
#include <set>
#include <string>
#include <vector>

namespace core3d::path_array {
namespace detail {
inline void UUIDBytes(bounded_curve::Writer& writer, const UUID& value) {
    writer.raw(value);
}
inline void Owner(bounded_curve::Writer& writer, const OwnerKey& value) {
    UUIDBytes(writer, value.document); UUIDBytes(writer, value.entity);
    UUIDBytes(writer, value.definition);
}
inline void Source(bounded_curve::Writer& writer, const SourceIdentity& value) {
    UUIDBytes(writer, value.document); UUIDBytes(writer, value.entity);
    UUIDBytes(writer, value.definition); UUIDBytes(writer, value.sourceFeature);
}
inline bool ReadOwner(bounded_curve::Reader& reader, OwnerKey& value) {
    return reader.raw(value.document) && reader.raw(value.entity)
        && reader.raw(value.definition);
}
inline bool ReadSource(bounded_curve::Reader& reader, SourceIdentity& value) {
    return reader.raw(value.document) && reader.raw(value.entity)
        && reader.raw(value.definition) && reader.raw(value.sourceFeature);
}
} // namespace detail

inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!Valid(value)) return false;
        bounded_curve::Writer writer(MaximumEnvelopeBytes);
        writer.raw(reinterpret_cast<const std::uint8_t*>("SYPA"), 4);
        writer.integer(Schema, 1); writer.integer(0, 3);
        detail::Owner(writer, value.owner); detail::UUIDBytes(writer, value.feature);
        detail::Source(writer, value.source); detail::Owner(writer, value.path.owner);
        detail::UUIDBytes(writer, value.path.feature);
        writer.integer(value.path.definitionRevision, 8);
        writer.raw(value.path.canonicalDefinitionDigest);
        writer.integer(std::uint8_t(value.distribution.mode), 1);
        writer.integer(value.distribution.includeStart ? 1 : 0, 1);
        writer.integer(value.distribution.includeEnd ? 1 : 0, 1);
        writer.integer(value.closedPath ? 1 : 0, 1);
        writer.integer(value.distribution.count, 4);
        writer.scalar(value.distribution.distance);
        writer.integer(std::uint8_t(value.orientation.policy), 1);
        writer.integer(value.orientation.hasUpVector ? 1 : 0, 1);
        writer.integer(0, 2);
        writer.scalar(value.orientation.rollRadians);
        for (double scalar : value.orientation.upVector) writer.scalar(scalar);
        writer.scalar(value.orientation.maximumFrameStepRadians);
        writer.scalar(value.arcLengthTolerance); writer.scalar(value.minimumTangent);
        for (double scalar : value.sourceFrame) writer.scalar(scalar);
        writer.integer(value.issuance.nextLocalID, 8);
        writer.integer(value.issuance.retiredLocalIDs.size(), 4);
        writer.integer(value.members.size(), 4); writer.integer(value.removals.size(), 4);
        for (std::uint64_t retired : value.issuance.retiredLocalIDs) writer.integer(retired, 8);
        const auto writeMember = [&](const Member& member) {
            detail::UUIDBytes(writer, member.identity); writer.integer(member.localID, 8);
            writer.integer(member.coordinate.row, 4); writer.integer(member.coordinate.column, 4);
            writer.integer(std::uint8_t(member.state), 1); writer.integer(0, 3);
        };
        for (const Member& member : value.members) writeMember(member);
        for (const Member& member : value.removals) writeMember(member);
        if (!writer.valid || writer.bytes.size() > MaximumEnvelopeBytes - 32) return false;
        Digest digest{};
        if (!bounded_curve::Hash(writer.bytes, MaximumEnvelopeBytes, digest)) return false;
        writer.raw(digest); if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output) noexcept {
    output = {};
    try {
        constexpr std::size_t Minimum = 8 + 64 + 64 + 48 + 16 + 8 + 32
            + 4 + 4 + 8 + 4 + 8 + 24 + 8 + 16 + 128 + 8 + 12 + 2 * 36 + 32;
        if (bytes.size() < Minimum || bytes.size() > MaximumEnvelopeBytes
            || std::memcmp(bytes.data(), "SYPA\1\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!bounded_curve::Hash(body, MaximumEnvelopeBytes, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (expected != actual) return false;
        bounded_curve::Reader reader(bytes, bytes.size() - 32);
        std::array<std::uint8_t, 8> prefix{}; std::uint64_t raw = 0, reserved = 0;
        Definition value;
        if (!reader.raw(prefix) || !detail::ReadOwner(reader, value.owner)
            || !reader.raw(value.feature) || !detail::ReadSource(reader, value.source)
            || !detail::ReadOwner(reader, value.path.owner) || !reader.raw(value.path.feature)
            || !reader.integer(8, value.path.definitionRevision)
            || !reader.raw(value.path.canonicalDefinitionDigest)
            || !reader.integer(1, raw)) return false;
        value.distribution.mode = DistributionMode(raw);
        if (!reader.integer(1, raw) || raw > 1) return false;
        value.distribution.includeStart = raw != 0;
        if (!reader.integer(1, raw) || raw > 1) return false;
        value.distribution.includeEnd = raw != 0;
        if (!reader.integer(1, raw) || raw > 1) return false;
        value.closedPath = raw != 0;
        if (!reader.integer(4, raw) || raw > UINT32_MAX) return false;
        value.distribution.count = std::uint32_t(raw);
        if (!reader.scalar(value.distribution.distance) || !reader.integer(1, raw)) return false;
        value.orientation.policy = OrientationPolicy(raw);
        if (!reader.integer(1, raw) || raw > 1) return false;
        value.orientation.hasUpVector = raw != 0;
        if (!reader.integer(2, reserved) || reserved != 0
            || !reader.scalar(value.orientation.rollRadians)) return false;
        for (double& scalar : value.orientation.upVector) if (!reader.scalar(scalar)) return false;
        if (!reader.scalar(value.orientation.maximumFrameStepRadians)
            || !reader.scalar(value.arcLengthTolerance) || !reader.scalar(value.minimumTangent)) return false;
        for (double& scalar : value.sourceFrame) if (!reader.scalar(scalar)) return false;
        std::uint64_t retiredCount = 0, memberCount = 0, removalCount = 0;
        if (!reader.integer(8, value.issuance.nextLocalID)
            || !reader.integer(4, retiredCount) || !reader.integer(4, memberCount)
            || !reader.integer(4, removalCount)
            || retiredCount > MaximumInstances * 8u || memberCount > MaximumInstances
            || removalCount > MaximumInstances * 8u) return false;
        value.issuance.retiredLocalIDs.resize(std::size_t(retiredCount));
        for (std::uint64_t& retired : value.issuance.retiredLocalIDs)
            if (!reader.integer(8, retired)) return false;
        const auto readMember = [&](Member& member) {
            std::uint64_t row = 0, column = 0, state = 0;
            if (!reader.raw(member.identity) || !reader.integer(8, member.localID)
                || !reader.integer(4, row) || !reader.integer(4, column)
                || !reader.integer(1, state) || !reader.integer(3, reserved)
                || reserved != 0 || row > UINT32_MAX || column > UINT32_MAX) return false;
            member.coordinate = {std::uint32_t(row), std::uint32_t(column)};
            member.state = MemberState(state); return true;
        };
        value.members.resize(std::size_t(memberCount));
        for (Member& member : value.members) if (!readMember(member)) return false;
        value.removals.resize(std::size_t(removalCount));
        for (Member& member : value.removals) if (!readMember(member)) return false;
        std::vector<std::uint8_t> canonical;
        if (!reader.complete() || !Valid(value) || !Encode(value, canonical)
            || canonical != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

inline constexpr int DocumentRootTag = 72;
inline const char* DocumentMarker = "Shapeyard retained path arrays v1";

struct Record {
    TDF_Label label;
    Definition definition;
    std::vector<std::uint8_t> bytes;
};

inline bool ReadByteArray(const TDF_Label& label,
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
        Handle(TDataStd_AsciiString) marker;
        if (!root.FindAttribute(TDataStd_AsciiString::GetID(), marker) || marker.IsNull()
            || marker->Get().ToCString() != std::string(DocumentMarker)) return false;
        std::vector<Record> staged; std::set<UUID> features; std::size_t aggregate = 0;
        for (TDF_ChildIterator child(root, Standard_False); child.More(); child.Next()) {
            Record record; record.label = child.Value();
            if (!ReadByteArray(record.label, record.bytes)
                || !Decode(record.bytes, record.definition)
                || !features.insert(record.definition.feature).second
                || record.bytes.size() > MaximumDocumentBytes - aggregate) return false;
            aggregate += record.bytes.size(); staged.push_back(std::move(record));
        }
        output = std::move(staged); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool ReadFeature(const Handle(TDocStd_Document)& document, const UUID& feature,
                        Record& output) noexcept {
    output = {}; std::vector<Record> records;
    if (!ReadAll(document, records)) return false;
    for (Record& record : records) if (record.definition.feature == feature) {
        output = std::move(record); return true;
    }
    return true;
}

// Called inside the controller's single OCAF command after source/path
// authority and every rebuilt member have been staged.
inline bool Stage(const Handle(TDocStd_Document)& document,
                  const Definition& definition, Record& output) noexcept {
    output = {};
    try {
        if (document.IsNull() || !document->HasOpenCommand()) return false;
        std::vector<std::uint8_t> bytes; if (!Encode(definition, bytes)) return false;
        std::vector<Record> before; if (!ReadAll(document, before)) return false;
        std::size_t aggregate = bytes.size(); TDF_Label target;
        for (const Record& record : before) {
            if (record.definition.feature == definition.feature) target = record.label;
            else if (record.bytes.size() > MaximumDocumentBytes - aggregate) return false;
            else aggregate += record.bytes.size();
        }
        TDF_Label root = document->Main().FindChild(DocumentRootTag, Standard_True);
        Handle(TDataStd_AsciiString) marker;
        if (root.FindAttribute(TDataStd_AsciiString::GetID(), marker)) {
            if (marker.IsNull() || marker->Get().ToCString()
                != std::string(DocumentMarker)) return false;
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
} // namespace core3d::path_array
