#pragma once

#include "LODSetPersistence.hxx"

#include <algorithm>

namespace core3d::lod_set_owner {

namespace persistence = lod_set_persistence;
namespace f2 = export_decimation;

enum class Refusal : std::uint8_t {
    None, InvalidRecord, ForeignDocument, CorruptStore,
    MissingF2Reference, F2ReferenceMismatch, StoreFull, NotFound
};

inline const persistence::Record *Find(const persistence::Store& store,
                                       const persistence::UUID& identifier) noexcept {
    for (const auto& record : store.records)
        if (record.setIdentifier == identifier) return &record;
    return nullptr;
}

inline bool ExactReference(const persistence::F2Reference& reference,
                           const f2::F2Record& record) noexcept {
    return reference.derivativeIdentifier == record.derivativeIdentifier
        && reference.settingsSHA256 == record.settingsSHA256
        && reference.sourceArtifactSHA256 == record.sourceArtifactSHA256
        && reference.sourceGeometrySHA256 == record.sourceGeometrySHA256
        && reference.outputArtifactSHA256 == record.outputArtifactSHA256
        && reference.outputGeometrySHA256 == record.outputGeometrySHA256
        && reference.remapReceiptSHA256 == record.remapReceiptSHA256
        && reference.independentMeasurementSHA256 == record.independentMeasurementSHA256
        && reference.algorithmBuildSHA256 == record.algorithmBuildSHA256
        && reference.measuredTriangleCount == record.emittedTriangleCount;
}

// Current is conferred only by exact readback from the real F2 store. A stale
// level may retain historical reference bytes, while Unbuilt has no reference.
inline bool ValidateReferences(const persistence::Record& record,
                               const f2::F2Store* f2Store,
                               Refusal& refusal) noexcept {
    refusal = Refusal::InvalidRecord;
    if (!persistence::ValidRecord(record)) return false;
    for (std::size_t levelIndex = 0; levelIndex < record.levels.size(); ++levelIndex) {
        const auto& level = record.levels[levelIndex];
        if (level.state != persistence::State::Current) continue;
        if (!f2Store || f2Store->documentUUID != record.documentIdentifier) {
            refusal = Refusal::MissingF2Reference; return false;
        }
        const f2::F2Record* match = nullptr;
        for (const auto& candidate : f2Store->records)
            if (candidate.derivativeIdentifier == level.f2.derivativeIdentifier) {
                match = &candidate; break;
            }
        if (!match || match->stale || match->level != levelIndex) {
            refusal = Refusal::MissingF2Reference; return false;
        }
        if (!ExactReference(level.f2, *match)) {
            refusal = Refusal::F2ReferenceMismatch; return false;
        }
    }
    refusal = Refusal::None; return true;
}

inline bool Upsert(const persistence::Store* before,
                   persistence::Record candidate,
                   const f2::F2Store* f2Store,
                   persistence::Store& result,
                   Refusal& refusal) {
    result = {};
    try {
        if (!ValidateReferences(candidate, f2Store, refusal)) return false;
        if (before) {
            if (before->documentIdentifier != candidate.documentIdentifier) {
                refusal = Refusal::ForeignDocument; return false;
            }
            result = *before;
        } else {
            result.documentIdentifier = candidate.documentIdentifier;
        }
        bool replaced = false;
        for (auto& record : result.records)
            if (record.setIdentifier == candidate.setIdentifier) {
                if (candidate.revision != record.revision + 1) {
                    refusal = Refusal::InvalidRecord; result = {}; return false;
                }
                record = std::move(candidate); replaced = true; break;
            }
        if (!replaced) {
            if (candidate.revision != 1) {
                refusal = Refusal::InvalidRecord; result = {}; return false;
            }
            result.records.push_back(std::move(candidate));
        }
        if (result.records.size() > persistence::kMaximumSets) {
            refusal = Refusal::StoreFull; result = {}; return false;
        }
        std::sort(result.records.begin(), result.records.end(),
            [](const auto& left, const auto& right) {
                return left.setIdentifier < right.setIdentifier;
            });
        std::vector<std::uint8_t> canonical;
        if (!persistence::EncodeStore(result, canonical)) {
            refusal = Refusal::StoreFull; result = {}; return false;
        }
        refusal = Refusal::None; return true;
    } catch (...) { result = {}; refusal = Refusal::StoreFull; return false; }
}

inline bool Remove(const persistence::Store& before,
                   const persistence::UUID& identifier,
                   persistence::Store& result,
                   Refusal& refusal) {
    result = before;
    const auto oldSize = result.records.size();
    result.records.erase(std::remove_if(result.records.begin(), result.records.end(),
        [&](const auto& record) { return record.setIdentifier == identifier; }),
        result.records.end());
    if (result.records.size() == oldSize) {
        result = {}; refusal = Refusal::NotFound; return false;
    }
    refusal = Refusal::None; return true;
}

} // namespace core3d::lod_set_owner
