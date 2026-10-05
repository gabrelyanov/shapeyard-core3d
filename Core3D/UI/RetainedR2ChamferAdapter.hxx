// RetainedR2ChamferAdapter.hxx — B1c R2 semantic Chamfer adapter seam.
//
// The public chamfer bridge delegates to the existing R2 selector-append
// pipeline with this one explicit kind binding. Codec, receipt issuance,
// proof validation, geometry replay and staging authority remain in their
// existing serial owners; this adapter cannot mint or decode any of them.
#pragma once

#include "../OCCTKit/RetainedEdgeTreatmentDefinition.hxx"

namespace core3d::retained_r2_chamfer_adapter {

inline retained_edge_treatment::Kind SelectorAppendKind() noexcept {
    return retained_edge_treatment::Kind::Chamfer;
}

} // namespace core3d::retained_r2_chamfer_adapter
