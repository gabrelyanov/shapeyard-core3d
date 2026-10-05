#pragma once

#include "../OCCTKit/RetainedEdgeTreatmentSnapshot.hxx"

#include <AIS_Shape.hxx>
#include <TDF_Label.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace core3d {

//! Header-only ownership seam between the serial Bevel controller and the
//! retained edge-treatment owner. Document/AIS authority stays in the viewer;
//! only immutable snapshot, anchor, edit and detached-geometry values cross
//! into the preview worker.
class RetainedBevelAdapter final {
public:
    struct Capture final {
        std::shared_ptr<const retained_edge_treatment::Snapshot> snapshot;
        Handle(AIS_Shape) presentation;
        std::vector<retained_edge_treatment::Anchor> anchors;
        retained_edge_treatment::ReplayBudget budget;
    };

    struct PreparedPreview final {
        std::shared_ptr<retained_edge_treatment::Work> work;
        std::shared_ptr<const retained_edge_treatment::DetachedInput> input;
        retained_edge_treatment::Edit edit;
    };

    using CaptureFunction = std::function<Standard_Boolean(
        const TDF_Label&,
        const TopoDS_Shape&,
        const std::vector<TopoDS_Edge>&,
        Capture&,
        retained_edge_treatment::Refusal&)>;
    using PrepareFunction = std::function<Standard_Boolean(
        const Capture&,
        const retained_edge_treatment::Edit&,
        PreparedPreview&,
        retained_edge_treatment::Refusal&)>;
    using BuildFunction = std::function<
        std::shared_ptr<const retained_edge_treatment::DetachedResult>(
            const std::shared_ptr<const retained_edge_treatment::DetachedInput>&,
            retained_edge_treatment::Refusal&)>;
    using CancelFunction = std::function<void(
        const std::shared_ptr<retained_edge_treatment::Work>&)>;

    RetainedBevelAdapter() = default;
    RetainedBevelAdapter(CaptureFunction capture,
                         PrepareFunction prepare,
                         BuildFunction build,
                         CancelFunction cancel)
        : capture_(std::move(capture)),
          prepare_(std::move(prepare)),
          build_(std::move(build)),
          cancel_(std::move(cancel)) {}

    explicit operator bool() const noexcept {
        return bool(capture_) && bool(prepare_) && bool(build_) && bool(cancel_);
    }

    std::shared_ptr<const Capture> CaptureRetainedBevelSelection(
        const TDF_Label& label,
        const TopoDS_Shape& source,
        const std::vector<TopoDS_Edge>& edges,
        retained_edge_treatment::Refusal& refusal) const noexcept {
        refusal = retained_edge_treatment::Refusal::UnsupportedBase;
        if (!*this || label.IsNull() || source.IsNull() || edges.empty()) {
            return {};
        }
        try {
            auto capture = std::make_shared<Capture>();
            if (!capture_(label, source, edges, *capture, refusal)
                || !capture->snapshot || !capture->snapshot->current()
                || capture->snapshot->effectiveDefinition().base.family
                    != retained_edge_treatment::SourceFamily::Profile
                || capture->anchors.empty()) {
                return {};
            }
            refusal = retained_edge_treatment::Refusal::None;
            return capture;
        } catch (...) {
            refusal = retained_edge_treatment::Refusal::BuildFailed;
            return {};
        }
    }

    std::shared_ptr<const PreparedPreview> PrepareRetainedBevelPreview(
        const std::shared_ptr<const Capture>& capture,
        const Standard_Real signedDistanceLocal,
        retained_edge_treatment::Refusal& refusal) const noexcept {
        refusal = retained_edge_treatment::Refusal::InvalidAmount;
        if (!*this || !capture || !capture->snapshot
            || capture->snapshot->effectiveDefinition().base.family
                != retained_edge_treatment::SourceFamily::Profile
            || !std::isfinite(signedDistanceLocal)
            || signedDistanceLocal == 0.0) {
            return {};
        }
        const double amountMM = std::abs(signedDistanceLocal)
            * capture->snapshot->dimensionMetersPerUnit() * 1000.0;
        if (!std::isfinite(amountMM) || amountMM <= 0.0 || amountMM > 20.0) {
            return {};
        }
        try {
            retained_edge_treatment::Append append;
            append.kind = signedDistanceLocal < 0.0
                ? retained_edge_treatment::Kind::Chamfer
                : retained_edge_treatment::Kind::ConstantFillet;
            append.amountMM = amountMM;
            append.anchors = capture->anchors;
            const retained_edge_treatment::Edit edit(append);
            auto prepared = std::make_shared<PreparedPreview>();
            if (!prepare_(*capture, edit, *prepared, refusal)
                || !prepared->work || !prepared->input) {
                return {};
            }
            refusal = retained_edge_treatment::Refusal::None;
            return prepared;
        } catch (...) {
            refusal = retained_edge_treatment::Refusal::BuildFailed;
            return {};
        }
    }

    std::shared_ptr<const retained_edge_treatment::DetachedResult>
    BuildRetainedBevelPreview(
        const std::shared_ptr<const retained_edge_treatment::DetachedInput>& input,
        retained_edge_treatment::Refusal& refusal) const noexcept {
        if (!*this || !input) {
            refusal = retained_edge_treatment::Refusal::BuildFailed;
            return {};
        }
        return build_(input, refusal);
    }

    void CancelRetainedBevelPreview(
        const std::shared_ptr<retained_edge_treatment::Work>& work) const noexcept {
        if (*this && work) {
            try {
                cancel_(work);
            } catch (...) {
            }
        }
    }

private:
    CaptureFunction capture_;
    PrepareFunction prepare_;
    BuildFunction build_;
    CancelFunction cancel_;
};

} // namespace core3d
