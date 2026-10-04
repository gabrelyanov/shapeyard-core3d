// RetainedSemanticChamferAdapter.hxx — B1a ordinary semantic Chamfer adapter seam.
//
// Header-only, included once from Core3DViewController.mm. This is the single
// kind-specific delegate for the existing ordinary selector-append bridge
// entry: the proved semantic Chamfer append is exactly the shared
// kind-parameterised selector append, kind-bound to Chamfer. The adapter holds
// no codec, staging, receipt, proof or budget authority — the carrier version
// semantics stay in OCCTKit/RetainedEdgeTreatmentDefinition.hxx, staging stays
// in OCCTKit/OcctDocument.mm, and preparation stays in Core3DViewer.mm. It
// widens no public kind/receipt/proof authority: it only names the one kind
// the schema-3 contract admits through this bridge entry.
#pragma once

#include "../OCCTKit/RetainedEdgeTreatmentDefinition.hxx"

namespace core3d {
namespace retained_semantic_chamfer_adapter {

// The kind bound to the semantic Chamfer bridge entry. ConstantFillet never
// routes through this adapter; it keeps the schema-2 contract at the existing
// entry.
inline retained_edge_treatment::Kind SelectorAppendKind() noexcept {
    return retained_edge_treatment::Kind::Chamfer;
}

} // namespace retained_semantic_chamfer_adapter
} // namespace core3d
