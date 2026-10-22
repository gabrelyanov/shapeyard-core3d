#pragma once

#include "LODSetPersistence.hxx"

#include <algorithm>
#include <atomic>

namespace core3d::lod_set_owner {

namespace persistence = lod_set_persistence;
namespace f2 = export_decimation;

enum class Refusal : std::uint8_t {
    None, InvalidRecord, ForeignDocument, CorruptStore,
    MissingF2Reference, F2ReferenceMismatch, StoreFull, NotFound
};

#if DEBUG
enum class Fault : std::uint8_t {
    None, BeforeStage, AfterF2Stage, AfterSYLSStage, AfterReadback, UnknownClose
};
inline std::atomic<Fault> debugFault{Fault::None};
#endif

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

inline bool AttachCurrentReference(persistence::Record& record,
                                   std::size_t levelIndex,
                                   const f2::F2Store& f2Store,
                                   const std::string& artifactName,
                                   Refusal& refusal) noexcept {
    refusal = Refusal::InvalidRecord;
    if (levelIndex >= record.levels.size()
        || f2Store.documentUUID != record.documentIdentifier
        || !persistence::SafeName(artifactName)) return false;
    const f2::F2Record* match = nullptr;
    for (const auto& candidate : f2Store.records) {
        if (candidate.entityUUID == record.sourceEntityIdentifier
            && candidate.definitionUUID == record.sourceDefinitionIdentifier
            && candidate.level == levelIndex) {
            if (match) { refusal = Refusal::F2ReferenceMismatch; return false; }
            match = &candidate;
        }
    }
    if (!match || match->stale) {
        refusal = Refusal::MissingF2Reference; return false;
    }
    auto& level = record.levels[levelIndex];
    level.hasF2Reference = true;
    level.state = persistence::State::Current;
    level.reason = persistence::Reason::None;
    level.f2.derivativeIdentifier = match->derivativeIdentifier;
    level.f2.settingsSHA256 = match->settingsSHA256;
    level.f2.sourceArtifactSHA256 = match->sourceArtifactSHA256;
    level.f2.sourceGeometrySHA256 = match->sourceGeometrySHA256;
    level.f2.outputArtifactSHA256 = match->outputArtifactSHA256;
    level.f2.outputGeometrySHA256 = match->outputGeometrySHA256;
    level.f2.remapReceiptSHA256 = match->remapReceiptSHA256;
    level.f2.independentMeasurementSHA256 = match->independentMeasurementSHA256;
    level.f2.algorithmBuildSHA256 = match->algorithmBuildSHA256;
    level.f2.measuredTriangleCount = match->emittedTriangleCount;
    level.f2.artifactName = artifactName;
    record.hasLastBuiltSource = true;
    record.lastBuiltSourceSHA256 = match->sourceRevisionSHA256;
    const bool allCurrent = std::all_of(record.levels.begin(), record.levels.end(),
        [](const auto& candidate) { return candidate.state == persistence::State::Current; });
    record.state = allCurrent ? persistence::State::Current : persistence::State::Stale;
    record.reason = allCurrent ? persistence::Reason::None : persistence::Reason::SettingsChanged;
    if (!persistence::ValidRecord(record)
        || !ValidateReferences(record, &f2Store, refusal)) return false;
    refusal = Refusal::None; return true;
}

// Called only by a source owner while its ordinary command is open. It mutates
// the in-memory candidates as one unit; the document seam stages/readbacks both
// carriers or returns false so the source owner aborts the whole command.
inline bool MarkSourceStale(persistence::Store& store,
                            const persistence::UUID& entity,
                            const persistence::UUID& definition,
                            f2::F2Store* f2Store,
                            bool& changed,
                            Refusal& refusal) noexcept {
    changed = false; refusal = Refusal::None;
    try {
        for (auto& record : store.records) {
            if (record.sourceEntityIdentifier != entity
                || record.sourceDefinitionIdentifier != definition) continue;
            ++record.revision;
            ++record.nativeEditRevision;
            record.state = persistence::State::Stale;
            record.reason = persistence::Reason::SourceChanged;
            for (auto& level : record.levels) {
                level.state = persistence::State::Stale;
                level.reason = persistence::Reason::SourceChanged;
                if (!level.hasF2Reference) continue;
                if (!f2Store) { refusal = Refusal::MissingF2Reference; return false; }
                f2::F2Record* match = nullptr;
                for (auto& row : f2Store->records)
                    if (row.derivativeIdentifier == level.f2.derivativeIdentifier) {
                        match = &row; break;
                    }
                if (!match || !ExactReference(level.f2, *match)) {
                    refusal = match ? Refusal::F2ReferenceMismatch
                                    : Refusal::MissingF2Reference;
                    return false;
                }
                f2::MarkStale(*match);
            }
            if (!persistence::ValidRecord(record)) {
                refusal = Refusal::InvalidRecord; return false;
            }
            changed = true;
        }
        return true;
    } catch (...) { refusal = Refusal::StoreFull; return false; }
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
