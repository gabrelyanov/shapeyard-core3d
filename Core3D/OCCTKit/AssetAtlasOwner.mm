#import <Foundation/Foundation.h>

// E2a all-member staging for the asset atlas. This file never opens its own
// mutation route: Prepare resolves and fences all members only, Commit
// installs the atlas attribute whose layoutProof binds every staged member
// UV assignment inside the caller's already-open OCAF command under the
// existing document mutation owner. Refusal leaves no delta; a command abort
// restores the prior record exactly. Member UV assignments are staging
// values bound into layoutProof; they are never persisted as bytes and no
// member shape is ever mutated (the row-268 geometry fences digest the
// stored triangulation, so installing UVs onto member shapes would stale
// them).
#include "AssetAtlasPersistence.hxx"
#include "OcctDocument.h"

namespace core3d::asset_atlas::owner {

bool ResolveAtlasLabel(const Handle(TDocStd_Document)& document, const Key& key,
                       TDF_Label& output) noexcept {
    output = TDF_Label();
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || !retained_recipe::Nonzero(key.document)
            || !retained_recipe::Nonzero(key.atlas)) return false;
        UUID documentID{};
        if (!retained_solid::ReadUUID(document->Main(),
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
            || !(documentID == key.document)) return false;
        persistence::Record record;
        if (!persistence::Read(document, key, record) || !record.value) return false;
        output = record.label;
        return true;
    } catch (...) { output = TDF_Label(); return false; }
}

Outcome Prepare(Staging& staging, const Handle(TDocStd_Document)& document,
                const Definition& candidate,
                const std::vector<MemberUVAssignment>& assignments,
                const std::vector<Member>& observed) noexcept {
    staging = Staging{};
    try {
        if (document.IsNull() || document->GetData().IsNull()) return Outcome::Malformed;
        UUID documentID{};
        if (!retained_solid::ReadUUID(document->Main(),
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID))
            return Outcome::Malformed;
        Key resolved = candidate.key;
        resolved.document = documentID;
        const Refusal refusal = staging.transaction.Prepare(candidate, resolved, observed);
        if (refusal == Refusal::StaleSource) return Outcome::StaleSource;
        if (refusal == Refusal::OwnerMismatch) return Outcome::OwnerMismatch;
        if (refusal == Refusal::TransactionBusy) return Outcome::Busy;
        if (refusal != Refusal::None) return Outcome::Malformed;
        if (!PlausibleUVAssignments(candidate, assignments)) {
            staging.transaction.Cancel();
            return Outcome::Malformed;
        }
        std::vector<std::uint8_t> bytes;
        if (!Encode(candidate, bytes)) {
            staging.transaction.Cancel();
            return Outcome::Malformed;
        }
        staging.candidate = candidate;
        staging.bytes = std::move(bytes);
        staging.assignments = assignments;
        staging.observed = observed;
        return Outcome::Prepared;
    } catch (...) { staging = Staging{}; return Outcome::Malformed; }
}

Outcome Commit(Staging& staging, const Handle(TDocStd_Document)& document,
               const std::vector<Member>& observed) noexcept {
    try {
        if (document.IsNull() || document->GetData().IsNull()) {
            staging = Staging{};
            return Outcome::Malformed;
        }
        UUID documentID{};
        if (!retained_solid::ReadUUID(document->Main(),
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)) {
            staging = Staging{};
            return Outcome::Malformed;
        }
        Key resolved = staging.candidate.key;
        resolved.document = documentID;
        const Refusal refusal = staging.transaction.Commit(resolved, observed);
        if (refusal != Refusal::None) {
            staging = Staging{};
            return refusal == Refusal::StaleSource ? Outcome::StaleSource
                : refusal == Refusal::OwnerMismatch ? Outcome::OwnerMismatch
                : Outcome::Malformed;
        }
        // No open command means no mutation authority; the staged value is
        // discarded whole rather than partially installed.
        if (!document->HasOpenCommand()) {
            staging = Staging{};
            return Outcome::Refused;
        }
        // The staged assignments are validated against the candidate once
        // more at commit; the attribute's layoutProof binds them.
        if (!PlausibleUVAssignments(staging.candidate, staging.assignments)) {
            staging = Staging{};
            return Outcome::Malformed;
        }
        auto payload = std::make_shared<persistence::Payload>();
        payload->definition = staging.candidate;
        payload->bytes = staging.bytes;
        if (!persistence::Attribute::StageCommitted(document, payload)) {
            staging = Staging{};
            return Outcome::PersistenceFailure;
        }
        staging = Staging{};
        return Outcome::Committed;
    } catch (...) { return Outcome::PersistenceFailure; }
}

void Cancel(Staging& staging) noexcept {
    staging.transaction.Cancel();
    staging = Staging{};
}

} // namespace core3d::asset_atlas::owner
