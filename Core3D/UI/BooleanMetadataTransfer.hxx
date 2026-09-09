// Internal Boolean metadata staging. Caller owns the active OCAF command.
#pragma once
#include "OcctDocument.h"
#include <TDataStd_Name.hxx>
#include <algorithm>
#include <pthread.h>
#include <vector>

namespace core3d::boolean_metadata {
struct BooleanLabelReplacement {
    TDF_Label source;
    TDF_Label result;
};

// Exact authored name transfer, including absent/empty legacy values. This is
// internal staging in an existing owned command, not a public rename command.
inline bool CopyBooleanName(OcctDocument& document, const TDF_Label& source,
                            const TDF_Label& result,
                            OcctObjectNameState& expectedResult) noexcept {
    expectedResult = {};
    try {
        const auto native = document.Document();
        if (pthread_main_np() == 0 || native.IsNull() || !native->HasOpenCommand()
            || source.IsNull() || result.IsNull() || source.IsEqual(result)
            || source.Data() != native->GetData() || result.Data() != native->GetData()) return false;
        OcctObjectNameState original, before, sourceAfter, after;
        if (!document.CaptureObjectNameStateForLabel(source, original)
            || !document.CaptureObjectNameStateForLabel(result, before)
            || original.name.Length() > 256
            || (original.namePresent && original.name.Length() > 0
                && !OcctObjectNameIsValid(original.name))) return false;
        if (original.namePresent) TDataStd_Name::Set(result, original.name);
        else result.ForgetAttribute(TDataStd_Name::GetID());
        if (!document.CaptureObjectNameStateForLabel(source, sourceAfter)
            || !document.CaptureObjectNameStateForLabel(result, after)
            || !original.IsEqual(sourceAfter) || !before.object.IsEqual(after.object)
            || original.namePresent != after.namePresent
            || !original.name.IsEqual(after.name)) return false;
        expectedResult = std::move(after);
        return true;
    } catch (...) { expectedResult = {}; return false; }
}

// Exact native source/result labels are supplied by the Boolean operation. The
// first ordered subject supplies a merged result; no name/order matching here.
// Run while all consumed source labels are still live, before RemoveShape.
inline bool StageBooleanGroupTransfer(OcctDocument& document,
    const std::vector<TDF_Label>& consumed,
    const std::vector<BooleanLabelReplacement>& replacements,
    OcctSavedGroupState& before, OcctSavedGroupState& after) noexcept {
    before = {}; after = {};
    try {
        const auto native = document.Document();
        if (pthread_main_np() == 0 || native.IsNull() || !native->HasOpenCommand()
            || consumed.empty() || consumed.size() > 8
            || replacements.empty() || replacements.size() > consumed.size()
            || !document.CaptureSavedGroups(before)) return false;
        auto contains = [](const auto& labels, const TDF_Label& sought) {
            return std::any_of(labels.begin(), labels.end(), [&](const auto& label) { return label.IsEqual(sought); });
        };
        std::vector<TDF_Label> uniqueSources, uniqueResults;
        for (const auto& source : consumed) {
            OcctObjectNameState captured;
            if (contains(uniqueSources, source) || !document.CaptureObjectNameStateForLabel(source, captured)) return false;
            uniqueSources.push_back(source);
        }
        std::vector<TDF_Label> replacedSources;
        for (const auto& replacement : replacements) {
            OcctObjectNameState captured;
            if (!contains(consumed, replacement.source) || contains(replacedSources, replacement.source)
                || contains(consumed, replacement.result) || contains(uniqueResults, replacement.result)
                || !document.CaptureObjectNameStateForLabel(replacement.result, captured)) return false;
            replacedSources.push_back(replacement.source);uniqueResults.push_back(replacement.result);
        }
        auto groups = before.groups;
        bool changed = false;
        for (auto& group : groups) {
            std::vector<TDF_Label> members;
            for (const auto& member : group.members) {
                if (!contains(consumed, member)) { members.push_back(member); continue; }
                changed = true;
                const auto replacement = std::find_if(replacements.begin(), replacements.end(),
                    [&](const auto& item) { return item.source.IsEqual(member); });
                if (replacement != replacements.end()) members.push_back(replacement->result);
            }
            group.members = std::move(members);
            // Preserve even emptied native records/IDs, consistent with the
            // existing catalog contract; empty records are not displayed groups.
        }
        if (changed && !document.StageSavedGroups(groups)) return false;
        return document.CaptureSavedGroups(after);
    } catch (...) { after = {}; return false; }
}
} // namespace core3d::boolean_metadata
