#import <Foundation/Foundation.h>

// E3 bounded owner store (278b portion 1). This file never opens its own
// mutation route: Prepare resolves the owner label and fences the candidate
// against the caller-captured B2 receipt/resource manifest only; Commit
// re-proves the identical fence and installs the binding record inside the
// caller's already-open OCAF command under the existing document mutation
// owner. Refusal leaves no delta; a command abort restores the prior record
// exactly. Resource adoption revalidates the envelope (including its carried
// original/working content hashes) and charges the single aggregate budget;
// a duplicate identity with identical bytes is a zero-delta refusal and with
// different bytes a foreign identity. No document registration, transaction,
// replay, UI opening or scene/export seam lives here: those are later
// portions.
#include "FaceImagePersistence.hxx"
#include "OcctDocument.h"

#include <TDF_LabelSequence.hxx>

namespace core3d::face_image::owner {

bool ResolveOwnerLabel(const Handle(TDocStd_Document)& document, const OwnerKey& key,
                       TDF_Label& output) noexcept {
    output = TDF_Label();
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || !retained_recipe::Valid(key)) return false;
        UUID documentID{};
        if (!retained_solid::ReadUUID(document->Main(),
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
            || !(documentID == key.document)) return false;
        const auto tool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (tool.IsNull()) return false;
        TDF_LabelSequence labels;
        tool->GetFreeShapes(labels);
        if (labels.Length() < 0 || labels.Length() > profile::MaximumLabels) return false;
        TDF_Label found; int matches = 0;
        for (Standard_Integer index = 1; index <= labels.Length(); ++index) {
            const TDF_Label label = labels.Value(index);
            UUID entity{}, definition{};
            if (!XCAFDoc_ShapeTool::IsSimpleShape(label) || !XCAFDoc_ShapeTool::IsFree(label)
                || !retained_solid::ReadUUID(label,
                    Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entity)
                || !retained_solid::ReadUUID(label,
                    Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definition)
                || !(entity == key.entity) || !(definition == key.definition)) continue;
            if (++matches > 1) return false;
            found = label;
        }
        if (matches != 1) return false;
        output = found;
        return true;
    } catch (...) { output = TDF_Label(); return false; }
}

namespace {
// Every bound resource must exist in this document's own store with the
// identical original-content identity the caller fenced. A UUID absent from
// the store is missing (a UUID minted by another document is equally absent
// here); a content mismatch between the fence and the store is stale. This
// check never trusts the caller's fence alone and never manufactures
// currentness itself.
Outcome CheckBoundResources(const Handle(TDocStd_Document)& document,
                            const Definition& candidate, const Observed& observed) noexcept {
    try {
        for (const auto& binding : candidate.bindings) {
            persistence::resources::Record record;
            if (!persistence::resources::Read(document, binding.resource, record))
                return Outcome::Malformed;
            if (!record.value) return Outcome::MissingResource;
            bool fenced = false;
            for (const auto& fence : observed.resources)
                if (fence.resource == binding.resource) {
                    fenced = true;
                    if (!(fence.content == record.value->envelope.originalContent))
                        return Outcome::StaleResource;
                    break;
                }
            if (!fenced) return Outcome::MissingResource;
        }
        return Outcome::Prepared;
    } catch (...) { return Outcome::Malformed; }
}

Outcome MapRefusal(Refusal refusal) noexcept {
    switch (refusal) {
        case Refusal::StaleSource: return Outcome::StaleSource;
        case Refusal::StaleFace: return Outcome::StaleFace;
        case Refusal::AmbiguousFaceRemap: return Outcome::AmbiguousFaceRemap;
        case Refusal::UnsupportedSurface: return Outcome::UnsupportedSurface;
        case Refusal::MissingResource: return Outcome::MissingResource;
        case Refusal::StaleResource: return Outcome::StaleResource;
        case Refusal::ForeignResource: return Outcome::ForeignResource;
        case Refusal::UnsupportedDownstream: return Outcome::UnsupportedDownstream;
        case Refusal::OwnerMismatch: return Outcome::OwnerMismatch;
        case Refusal::TransactionBusy: return Outcome::Busy;
        default: return Outcome::Malformed;
    }
}
} // namespace

Outcome Prepare(Staging& staging, const Handle(TDocStd_Document)& document,
                const Definition& candidate, const Observed& observed) noexcept {
    staging = Staging{};
    try {
        if (document.IsNull() || document->GetData().IsNull()) return Outcome::Malformed;
        TDF_Label label;
        if (!ResolveOwnerLabel(document, candidate.owner, label)) return Outcome::OwnerMismatch;
        // A malformed existing record refuses preparation closed rather than
        // being silently overwritten.
        Definition prior;
        if (persistence::bindings::Read(document, label, prior)
                == persistence::bindings::ReadState::Malformed)
            return Outcome::Malformed;
        const Outcome store = CheckBoundResources(document, candidate, observed);
        if (store != Outcome::Prepared) return store;
        const Refusal refusal = staging.transaction.Prepare(candidate, candidate.owner, observed);
        if (refusal != Refusal::None) return MapRefusal(refusal);
        std::vector<std::uint8_t> bytes;
        if (!Encode(candidate, bytes)) {
            staging.transaction.Cancel();
            return Outcome::Malformed;
        }
        staging.candidate = candidate;
        staging.bytes = std::move(bytes);
        staging.ownerLabel = label;
        return Outcome::Prepared;
    } catch (...) { staging = Staging{}; return Outcome::Malformed; }
}

Outcome Commit(Staging& staging, const Handle(TDocStd_Document)& document,
               const Observed& observed) noexcept {
    try {
        if (document.IsNull() || document->GetData().IsNull() || staging.ownerLabel.IsNull()
            || staging.ownerLabel.Data() != document->GetData()) {
            staging = Staging{};
            return Outcome::Malformed;
        }
        const Refusal refusal =
            staging.transaction.Commit(staging.candidate, staging.candidate.owner, observed);
        if (refusal != Refusal::None) {
            staging = Staging{};
            return MapRefusal(refusal);
        }
        const Outcome store = CheckBoundResources(document, staging.candidate, observed);
        if (store != Outcome::Prepared) {
            staging = Staging{};
            return store;
        }
        // No open command means no mutation authority; the staged value is
        // discarded whole rather than partially installed.
        if (!document->HasOpenCommand()) {
            staging = Staging{};
            return Outcome::Refused;
        }
        // Zero-delta refusal: an identical committed record contributes no
        // mutation and no history.
        Definition prior; std::vector<std::uint8_t> priorBytes;
        const auto state =
            persistence::bindings::Read(document, staging.ownerLabel, prior, &priorBytes);
        if (state == persistence::bindings::ReadState::Malformed) {
            staging = Staging{};
            return Outcome::Malformed;
        }
        if (state == persistence::bindings::ReadState::Present && priorBytes == staging.bytes) {
            staging = Staging{};
            return Outcome::Refused;
        }
        if (!persistence::bindings::StageCommitted(document, staging.ownerLabel,
                                                   staging.candidate, staging.bytes)) {
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

Outcome AdoptResource(const Handle(TDocStd_Document)& document,
                      const ResourceEnvelope& candidate) noexcept {
    try {
        if (document.IsNull() || document->GetData().IsNull()) return Outcome::Malformed;
        Refusal refusal = Refusal::None;
        if (!Valid(candidate, refusal)) return Outcome::Malformed;
        std::vector<std::uint8_t> bytes;
        if (!Encode(candidate, bytes)) return Outcome::Malformed;
        persistence::resources::Record prior;
        if (!persistence::resources::Read(document, candidate.resource, prior))
            return Outcome::Malformed;
        if (prior.value) {
            // Identity reuse: identical bytes are a zero-delta refusal;
            // different content under an existing identity is foreign.
            return prior.value->bytes == bytes ? Outcome::Refused : Outcome::ForeignResource;
        }
        if (!document->HasOpenCommand()) return Outcome::Refused;
        auto payload = std::make_shared<persistence::resources::Payload>();
        payload->envelope = candidate;
        payload->bytes = std::move(bytes);
        if (!persistence::resources::Attribute::StageCommitted(document, payload))
            return Outcome::PersistenceFailure;
        return Outcome::Committed;
    } catch (...) { return Outcome::PersistenceFailure; }
}

bool ReadResource(const Handle(TDocStd_Document)& document, const UUID& resource,
                  ResourceEnvelope& output) noexcept {
    output = {};
    try {
        if (document.IsNull() || !retained_recipe::Nonzero(resource)) return false;
        persistence::resources::Record record;
        if (!persistence::resources::Read(document, resource, record) || !record.value)
            return false;
        std::vector<std::uint8_t> exact;
        if (!Encode(record.value->envelope, exact) || exact != record.value->bytes)
            return false;
        output = record.value->envelope;
        return true;
    } catch (...) { output = {}; return false; }
}

bool ResourceManifest(const Handle(TDocStd_Document)& document,
                      std::vector<ResourceFence>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        std::vector<persistence::resources::Record> records;
        if (!persistence::resources::ReadAll(document, records)) return false;
        std::vector<ResourceFence> manifest;
        manifest.reserve(records.size());
        for (const auto& record : records)
            manifest.push_back({record.value->envelope.resource,
                                record.value->envelope.originalContent});
        output = std::move(manifest);
        return true;
    } catch (...) { output.clear(); return false; }
}

Outcome RemoveResource(const Handle(TDocStd_Document)& document,
                       const UUID& resource) noexcept {
    try {
        if (document.IsNull() || !retained_recipe::Nonzero(resource)) return Outcome::Malformed;
        persistence::resources::Record prior;
        if (!persistence::resources::Read(document, resource, prior)) return Outcome::Malformed;
        if (!prior.value) return Outcome::Refused; // zero-delta: nothing to remove
        // Typed removal refuses while any committed binding still references
        // the resource (278b portion 4b): the store never orphans a binding.
        if (XCAFDoc_DocumentTool::CheckShapeTool(document->Main())) {
            const auto tool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
            TDF_LabelSequence labels;
            if (!tool.IsNull()) tool->GetFreeShapes(labels);
            if (tool.IsNull() || labels.Length() < 0
                || labels.Length() > profile::MaximumLabels) return Outcome::Malformed;
            for (Standard_Integer index = 1; index <= labels.Length(); ++index) {
                Definition committed;
                const auto state = persistence::bindings::Read(
                    document, labels.Value(index), committed);
                if (state == persistence::bindings::ReadState::Malformed)
                    return Outcome::Malformed;
                if (state != persistence::bindings::ReadState::Present) continue;
                for (const auto& binding : committed.bindings)
                    if (binding.resource == resource) return Outcome::Refused;
            }
        }
        if (!document->HasOpenCommand()) return Outcome::Refused;
        if (!persistence::resources::Remove(document, resource))
            return Outcome::PersistenceFailure;
        return Outcome::Committed;
    } catch (...) { return Outcome::PersistenceFailure; }
}

} // namespace core3d::face_image::owner
