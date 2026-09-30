#import <Foundation/Foundation.h>

// E1 one-owner staging for the retained finishing receipt. This file never
// opens its own mutation route: Prepare resolves and fences only, Commit
// installs inside the caller's already-open OCAF command under the existing
// document mutation owner. Refusal leaves no delta; a command abort restores
// the prior receipt exactly.
#include "RetainedFinishingBinaryDriver.hxx"
#include "OcctDocument.h"

#include <TDF_LabelSequence.hxx>
#include <TDataStd_AsciiString.hxx>

namespace core3d::retained_finishing::owner {

bool ResolveOwnerLabel(const Handle(TDocStd_Document)& document, const OwnerKey& key,
                       TDF_Label& output) noexcept {
    output = TDF_Label();
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || !retained_recipe::Valid(key)) return false;
        UUID documentID{};
        if (!retained_solid::ReadUUID(document->Main(),
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
            || documentID != key.document) return false;
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
                || entity != key.entity || definition != key.definition) continue;
            if (++matches > 1) return false;
            found = label;
        }
        if (matches != 1) return false;
        output = found;
        return true;
    } catch (...) { output = TDF_Label(); return false; }
}

Outcome Prepare(Staging& staging, const Handle(TDocStd_Document)& document,
                const Definition& candidate, const SourceRevision& observed) noexcept {
    staging = Staging{};
    try {
        if (document.IsNull()) return Outcome::Malformed;
        TDF_Label label;
        if (!ResolveOwnerLabel(document, candidate.owner, label)) return Outcome::OwnerMismatch;
        const Refusal refusal = staging.transaction.Prepare(candidate, candidate.owner, observed);
        if (refusal == Refusal::StaleSource) return Outcome::StaleSource;
        if (refusal == Refusal::OwnerMismatch) return Outcome::OwnerMismatch;
        if (refusal == Refusal::TransactionBusy) return Outcome::Busy;
        if (refusal != Refusal::None) return Outcome::Malformed;
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
               const SourceRevision& observed) noexcept {
    try {
        if (document.IsNull() || staging.ownerLabel.IsNull()
            || staging.ownerLabel.Data() != document->GetData()) {
            staging = Staging{};
            return Outcome::Malformed;
        }
        const Refusal refusal =
            staging.transaction.Commit(staging.candidate.owner, observed);
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
        auto payload = std::make_shared<Payload>();
        payload->definition = staging.candidate;
        payload->bytes = staging.bytes;
        if (!Attribute::StageCommitted(document, staging.ownerLabel, payload)) {
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

} // namespace core3d::retained_finishing::owner
