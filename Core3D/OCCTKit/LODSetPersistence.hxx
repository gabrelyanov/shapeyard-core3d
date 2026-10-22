#pragma once

#include "ExportMeshDecimationNativeOwner.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <set>
#include <string>
#include <vector>

// F3-S2's only durable representation. SYLS/1 contains intent and immutable
// references to F2 records; it never contains a BRep or an external path.
namespace core3d::lod_set_persistence {

using UUID = export_decimation::F2UUID;
using Digest = export_decimation::F2Digest;
inline constexpr std::size_t kMaximumRecordBytes = 65536;
inline constexpr std::size_t kMaximumSets = 64;
inline constexpr std::size_t kMaximumLevels = 8;
inline constexpr std::size_t kMaximumNameBytes = 128;

enum class State : std::uint8_t { Unbuilt = 0, Current = 1, Stale = 2 };
enum class Reason : std::uint8_t {
    None = 0, SourceChanged = 1, SettingsChanged = 2,
    MissingArtifact = 3, ProvenanceInvalid = 4
};

struct QualityIntent {
    double maximumSurfaceDistanceMM = 0;
    double maximumSilhouetteDeviationPixels = 0;
    double hardEdgeAngleDegrees = 0;
    double contactClearanceMM = 0;
    std::uint32_t measurementSampleCount = 0;
    std::uint32_t silhouetteResolution = 0;
    Digest silhouetteCameraSetSHA256{};
    double maximumUVStretch = 0;
    double maximumNormalDeviationDegrees = 0;
    std::uint32_t uvMethodVersion = 0;
    std::uint32_t normalMethodVersion = 0;
    std::uint32_t overlapMethodVersion = 0;
};

struct F2Reference {
    Digest derivativeIdentifier{};
    Digest settingsSHA256{};
    Digest sourceArtifactSHA256{};
    Digest sourceGeometrySHA256{};
    Digest outputArtifactSHA256{};
    Digest outputGeometrySHA256{};
    Digest remapReceiptSHA256{};
    Digest independentMeasurementSHA256{};
    Digest algorithmBuildSHA256{};
    std::uint32_t measuredTriangleCount = 0;
    std::string artifactName;
};

struct Level {
    UUID identifier{};
    std::string name;
    std::uint32_t targetTriangleCount = 0;
    QualityIntent quality;
    Digest intentSHA256{};
    State state = State::Unbuilt;
    Reason reason = Reason::None;
    bool hasF2Reference = false;
    F2Reference f2;
};

struct Record {
    UUID documentIdentifier{};
    UUID setIdentifier{};
    std::uint64_t revision = 0;
    double metersPerUnit = 0;
    UUID sourceEntityIdentifier{};
    UUID sourceDefinitionIdentifier{};
    UUID sourceFeatureIdentifier{};
    Digest lastBuiltSourceSHA256{};
    bool hasLastBuiltSource = false;
    Digest observedSourceSHA256{};
    std::uint64_t nativeEditRevision = 0;
    std::uint64_t settingsRevision = 0;
    State state = State::Unbuilt;
    Reason reason = Reason::None;
    Digest attachmentMetadataSHA256{};
    std::vector<Level> levels;
};

struct Store {
    UUID documentIdentifier{};
    std::vector<Record> records;
};

inline bool NonzeroDigest(const Digest& value) noexcept {
    for (std::uint8_t byte : value) if (byte != 0) return true;
    return false;
}

inline bool SafeName(const std::string& value) noexcept {
    if (value.size() <= 4 || value.size() > kMaximumNameBytes
        || value.substr(value.size() - 4) != ".glb") return false;
    return value.find('/') == std::string::npos
        && value.find('\\') == std::string::npos
        && value != ".glb" && value != "..glb";
}

inline bool ValidState(State state) noexcept {
    return state == State::Unbuilt || state == State::Current || state == State::Stale;
}

inline bool ValidReason(Reason reason) noexcept {
    return static_cast<std::uint8_t>(reason)
        <= static_cast<std::uint8_t>(Reason::ProvenanceInvalid);
}

inline bool ValidQuality(const QualityIntent& q) noexcept {
    const double values[] = {q.maximumSurfaceDistanceMM,
        q.maximumSilhouetteDeviationPixels, q.hardEdgeAngleDegrees,
        q.contactClearanceMM, q.maximumUVStretch,
        q.maximumNormalDeviationDegrees};
    for (double value : values)
        if (!std::isfinite(value) || (value == 0.0 && std::signbit(value))) return false;
    return q.maximumSurfaceDistanceMM > 0
        && q.maximumSilhouetteDeviationPixels >= 0
        && q.hardEdgeAngleDegrees > 0 && q.hardEdgeAngleDegrees < 180
        && q.contactClearanceMM >= 0
        && q.measurementSampleCount > 0
        && q.measurementSampleCount <= export_decimation::kMaximumMeasurementSamples
        && q.silhouetteResolution > 0
        && q.silhouetteResolution <= export_decimation::kMaximumSilhouetteResolution
        && NonzeroDigest(q.silhouetteCameraSetSHA256)
        && q.maximumUVStretch >= 0
        && q.maximumNormalDeviationDegrees >= 0
        && q.maximumNormalDeviationDegrees < 180
        && q.uvMethodVersion > 0 && q.normalMethodVersion > 0
        && q.overlapMethodVersion > 0;
}

inline bool ValidLevel(const Level& level) noexcept {
    if (!export_decimation::Nonzero(level.identifier)
        || level.name.empty() || level.name.size() > kMaximumNameBytes
        || level.targetTriangleCount < export_decimation::kMinimumClosedMeshTriangles
        || level.targetTriangleCount > export_decimation::kMaximumTriangleTarget
        || !ValidQuality(level.quality) || !NonzeroDigest(level.intentSHA256)
        || !ValidState(level.state) || !ValidReason(level.reason)) return false;
    if (level.state == State::Unbuilt)
        return level.reason == Reason::None && !level.hasF2Reference;
    // A source edit can stale intent that has never produced an artifact. Such
    // a level has no F2 row to retain; Current is the only state that requires
    // a reference. Stale levels that do have historical output keep it exact.
    if (!level.hasF2Reference)
        return level.state == State::Stale && level.reason != Reason::None;
    return NonzeroDigest(level.f2.derivativeIdentifier)
        && NonzeroDigest(level.f2.settingsSHA256)
        && NonzeroDigest(level.f2.sourceArtifactSHA256)
        && NonzeroDigest(level.f2.sourceGeometrySHA256)
        && NonzeroDigest(level.f2.outputArtifactSHA256)
        && NonzeroDigest(level.f2.outputGeometrySHA256)
        && NonzeroDigest(level.f2.remapReceiptSHA256)
        && NonzeroDigest(level.f2.independentMeasurementSHA256)
        && NonzeroDigest(level.f2.algorithmBuildSHA256)
        && level.f2.measuredTriangleCount >= export_decimation::kMinimumClosedMeshTriangles
        && level.f2.measuredTriangleCount <= level.targetTriangleCount
        && SafeName(level.f2.artifactName)
        && (level.state != State::Current || level.reason == Reason::None);
}

inline bool ValidRecord(const Record& value) noexcept {
    if (!export_decimation::Nonzero(value.documentIdentifier)
        || !export_decimation::Nonzero(value.setIdentifier)
        || value.revision == 0
        || (value.metersPerUnit != 0.001 && value.metersPerUnit != 1.0)
        || !export_decimation::Nonzero(value.sourceEntityIdentifier)
        || !export_decimation::Nonzero(value.sourceDefinitionIdentifier)
        || !export_decimation::Nonzero(value.sourceFeatureIdentifier)
        || !NonzeroDigest(value.observedSourceSHA256)
        || !ValidState(value.state) || !ValidReason(value.reason)
        || !NonzeroDigest(value.attachmentMetadataSHA256)
        || value.levels.size() < 2 || value.levels.size() > kMaximumLevels)
        return false;
    if (value.state == State::Current && !value.hasLastBuiltSource) return false;
    if (value.state == State::Unbuilt && value.hasLastBuiltSource) return false;
    if (value.hasLastBuiltSource && !NonzeroDigest(value.lastBuiltSourceSHA256)) return false;
    if (value.state == State::Unbuilt && value.reason != Reason::None) return false;
    std::set<UUID> identifiers;
    std::uint32_t previous = 0;
    for (std::size_t index = 0; index < value.levels.size(); ++index) {
        const Level& level = value.levels[index];
        if (!ValidLevel(level) || !identifiers.insert(level.identifier).second
            || (index != 0 && level.targetTriangleCount >= previous)) return false;
        previous = level.targetTriangleCount;
    }
    return true;
}

namespace detail {
using Writer = export_decimation::detail::Writer;
using Reader = export_decimation::detail::Reader;

inline void U16(Writer& writer, std::uint16_t value) {
    writer.U8(static_cast<std::uint8_t>(value));
    writer.U8(static_cast<std::uint8_t>(value >> 8));
}

inline bool U16(Reader& reader, std::uint16_t& value) {
    std::uint8_t low = 0, high = 0;
    if (!reader.U8(low) || !reader.U8(high)) return false;
    value = static_cast<std::uint16_t>(low | (std::uint16_t(high) << 8));
    return true;
}

inline void Quality(Writer& w, const QualityIntent& q) {
    w.F64(q.maximumSurfaceDistanceMM);
    w.F64(q.maximumSilhouetteDeviationPixels);
    w.F64(q.hardEdgeAngleDegrees); w.F64(q.contactClearanceMM);
    w.U32(q.measurementSampleCount); w.U32(q.silhouetteResolution);
    w.Digest(q.silhouetteCameraSetSHA256);
    w.F64(q.maximumUVStretch); w.F64(q.maximumNormalDeviationDegrees);
    w.U32(q.uvMethodVersion); w.U32(q.normalMethodVersion);
    w.U32(q.overlapMethodVersion);
}

inline bool Quality(Reader& r, QualityIntent& q) {
    return r.F64(q.maximumSurfaceDistanceMM)
        && r.F64(q.maximumSilhouetteDeviationPixels)
        && r.F64(q.hardEdgeAngleDegrees) && r.F64(q.contactClearanceMM)
        && r.U32(q.measurementSampleCount) && r.U32(q.silhouetteResolution)
        && r.Digest(q.silhouetteCameraSetSHA256)
        && r.F64(q.maximumUVStretch) && r.F64(q.maximumNormalDeviationDegrees)
        && r.U32(q.uvMethodVersion) && r.U32(q.normalMethodVersion)
        && r.U32(q.overlapMethodVersion);
}

inline void Reference(Writer& w, const F2Reference& f2) {
    w.Digest(f2.derivativeIdentifier); w.Digest(f2.settingsSHA256);
    w.Digest(f2.sourceArtifactSHA256); w.Digest(f2.sourceGeometrySHA256);
    w.Digest(f2.outputArtifactSHA256); w.Digest(f2.outputGeometrySHA256);
    w.Digest(f2.remapReceiptSHA256); w.Digest(f2.independentMeasurementSHA256);
    w.Digest(f2.algorithmBuildSHA256); w.U32(f2.measuredTriangleCount);
    w.Text(f2.artifactName);
}

inline bool Reference(Reader& r, F2Reference& f2) {
    return r.Digest(f2.derivativeIdentifier) && r.Digest(f2.settingsSHA256)
        && r.Digest(f2.sourceArtifactSHA256) && r.Digest(f2.sourceGeometrySHA256)
        && r.Digest(f2.outputArtifactSHA256) && r.Digest(f2.outputGeometrySHA256)
        && r.Digest(f2.remapReceiptSHA256)
        && r.Digest(f2.independentMeasurementSHA256)
        && r.Digest(f2.algorithmBuildSHA256) && r.U32(f2.measuredTriangleCount)
        && r.Text(f2.artifactName, kMaximumNameBytes);
}

inline void RecordPayload(Writer& w, const Record& record) {
    w.UUID(record.documentIdentifier); w.UUID(record.setIdentifier);
    w.U64(record.revision); w.F64(record.metersPerUnit);
    w.UUID(record.sourceEntityIdentifier); w.UUID(record.sourceDefinitionIdentifier);
    w.UUID(record.sourceFeatureIdentifier); w.U8(record.hasLastBuiltSource ? 1 : 0);
    if (record.hasLastBuiltSource) w.Digest(record.lastBuiltSourceSHA256);
    w.Digest(record.observedSourceSHA256); w.U64(record.nativeEditRevision);
    w.U64(record.settingsRevision); w.U8(static_cast<std::uint8_t>(record.state));
    w.U8(static_cast<std::uint8_t>(record.reason)); w.Digest(record.attachmentMetadataSHA256);
    w.U8(static_cast<std::uint8_t>(record.levels.size()));
    for (const Level& level : record.levels) {
        w.UUID(level.identifier); w.Text(level.name); w.U32(level.targetTriangleCount);
        Quality(w, level.quality); w.Digest(level.intentSHA256);
        w.U8(static_cast<std::uint8_t>(level.state));
        w.U8(static_cast<std::uint8_t>(level.reason));
        w.U8(level.hasF2Reference ? 1 : 0);
        if (level.hasF2Reference) Reference(w, level.f2);
    }
}

inline bool RecordPayload(Reader& r, Record& record) {
    std::uint8_t present = 0, state = 0, reason = 0, count = 0;
    if (!r.UUID(record.documentIdentifier) || !r.UUID(record.setIdentifier)
        || !r.U64(record.revision) || !r.F64(record.metersPerUnit)
        || !r.UUID(record.sourceEntityIdentifier) || !r.UUID(record.sourceDefinitionIdentifier)
        || !r.UUID(record.sourceFeatureIdentifier) || !r.U8(present) || present > 1)
        return false;
    record.hasLastBuiltSource = present != 0;
    if (record.hasLastBuiltSource && !r.Digest(record.lastBuiltSourceSHA256)) return false;
    if (!r.Digest(record.observedSourceSHA256) || !r.U64(record.nativeEditRevision)
        || !r.U64(record.settingsRevision) || !r.U8(state) || !r.U8(reason)
        || !r.Digest(record.attachmentMetadataSHA256) || !r.U8(count)
        || count < 2 || count > kMaximumLevels) return false;
    record.state = static_cast<State>(state); record.reason = static_cast<Reason>(reason);
    record.levels.reserve(count);
    for (std::uint8_t index = 0; index < count; ++index) {
        Level level;
        if (!r.UUID(level.identifier) || !r.Text(level.name, kMaximumNameBytes)
            || !r.U32(level.targetTriangleCount) || !Quality(r, level.quality)
            || !r.Digest(level.intentSHA256) || !r.U8(state) || !r.U8(reason)
            || !r.U8(present) || present > 1) return false;
        level.state = static_cast<State>(state); level.reason = static_cast<Reason>(reason);
        level.hasF2Reference = present != 0;
        if (level.hasF2Reference && !Reference(r, level.f2)) return false;
        record.levels.push_back(std::move(level));
    }
    return ValidRecord(record);
}
} // namespace detail

inline bool EncodeStore(const Store& store, std::vector<std::uint8_t>& out) {
    out.clear();
    try {
        if (!export_decimation::Nonzero(store.documentIdentifier)
            || store.records.empty() || store.records.size() > kMaximumSets) return false;
        detail::Writer payload;
        payload.UUID(store.documentIdentifier);
        payload.U32(static_cast<std::uint32_t>(store.records.size()));
        UUID prior{}; bool havePrior = false;
        for (const Record& record : store.records) {
            if (!ValidRecord(record) || record.documentIdentifier != store.documentIdentifier
                || (havePrior && !(prior < record.setIdentifier))) return false;
            detail::Writer encoded; detail::RecordPayload(encoded, record);
            payload.Blob(encoded.bytes); prior = record.setIdentifier; havePrior = true;
        }
        detail::Writer envelope; envelope.Raw("SYLS", 4);
        detail::U16(envelope, 1); detail::U16(envelope, 0);
        envelope.U32(static_cast<std::uint32_t>(payload.bytes.size()));
        envelope.Raw(payload.bytes.data(), payload.bytes.size());
        if (envelope.bytes.size() > kMaximumRecordBytes) return false;
        out = std::move(envelope.bytes); return true;
    } catch (...) { out.clear(); return false; }
}

inline bool DecodeStore(const std::vector<std::uint8_t>& bytes, Store& store) {
    store = {};
    try {
        if (bytes.size() < 12 || bytes.size() > kMaximumRecordBytes
            || std::memcmp(bytes.data(), "SYLS", 4) != 0) return false;
        detail::Reader header{bytes.data() + 4, bytes.size() - 4};
        std::uint16_t version = 0, flags = 0; std::uint32_t length = 0;
        if (!detail::U16(header, version) || version != 1
            || !detail::U16(header, flags) || flags != 0 || !header.U32(length)
            || length != header.left) return false;
        detail::Reader payload{header.at, length}; std::uint32_t count = 0;
        if (!payload.UUID(store.documentIdentifier)
            || !export_decimation::Nonzero(store.documentIdentifier)
            || !payload.U32(count) || count < 1 || count > kMaximumSets) return false;
        store.records.reserve(count);
        for (std::uint32_t index = 0; index < count; ++index) {
            std::vector<std::uint8_t> encoded;
            if (!payload.Blob(encoded, kMaximumRecordBytes) || encoded.empty()) return false;
            detail::Reader recordReader{encoded.data(), encoded.size()}; Record record;
            if (!detail::RecordPayload(recordReader, record) || !recordReader.Done()
                || record.documentIdentifier != store.documentIdentifier) return false;
            store.records.push_back(std::move(record));
        }
        if (!payload.Done()) return false;
        for (std::size_t index = 1; index < store.records.size(); ++index)
            if (!(store.records[index - 1].setIdentifier < store.records[index].setIdentifier))
                return false;
        std::vector<std::uint8_t> exact;
        return EncodeStore(store, exact) && exact == bytes;
    } catch (...) { store = {}; return false; }
}

enum class StoreOccurrence : std::uint8_t { Absent, Present, Corrupt };
struct StoreRead {
    StoreOccurrence occurrence = StoreOccurrence::Absent;
    std::vector<std::uint8_t> bytes;
    std::string refusal;
    bool readable() const noexcept { return occurrence != StoreOccurrence::Corrupt; }
};

inline bool SameStore(const StoreRead& left, const StoreRead& right) noexcept {
    return left.occurrence == right.occurrence && left.bytes == right.bytes;
}

} // namespace core3d::lod_set_persistence
