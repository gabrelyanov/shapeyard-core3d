#pragma once

#include <cstdint>
#include <memory>

namespace core3d {
struct ObjectFrameIdentity;
class Core3DViewer;
namespace retained_edge_treatment {
enum class Kind : std::uint8_t;
enum class Refusal : std::uint8_t;
namespace r2 {
class RawTargetCapture;
class RawPreviewSession;
class RawPreviewCandidate;
class DetachedInput;
}
}

// Native command adapter only. P1 deliberately owns no presentation and opens
// no document transaction; later portions may schedule its detached inputs.
class RawR2BevelPreviewAdapter final {
public:
    explicit RawR2BevelPreviewAdapter(Core3DViewer& viewer) noexcept;

    std::shared_ptr<retained_edge_treatment::r2::RawPreviewSession> open(
        const std::shared_ptr<const retained_edge_treatment::r2::RawTargetCapture>&,
        const ObjectFrameIdentity&, std::uint64_t presentationRevision,
        std::uint32_t width, std::uint32_t height,
        retained_edge_treatment::Refusal&) noexcept;

    std::shared_ptr<retained_edge_treatment::r2::RawPreviewCandidate> prepare(
        const std::shared_ptr<retained_edge_treatment::r2::RawPreviewSession>&,
        retained_edge_treatment::Kind, double amountMM,
        retained_edge_treatment::Refusal&) noexcept;

    std::shared_ptr<const retained_edge_treatment::r2::DetachedInput> detach(
        const std::shared_ptr<retained_edge_treatment::r2::RawPreviewCandidate>&,
        retained_edge_treatment::Refusal&) noexcept;

    bool cancel(
        const std::shared_ptr<retained_edge_treatment::r2::RawPreviewCandidate>&)
        noexcept;

private:
    Core3DViewer& viewer_;
};
} // namespace core3d
