#pragma once

// Typed evidence boundary only. B2 defines values and observations; B3 owns the
// native matcher and is the first increment permitted to return success.
#include "ProfileSourceBoundaryExpectation.hxx"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace core3d::complete_profile_source {

struct SourceBinding {
    std::vector<std::uint8_t> canonicalSourceBytes;
    std::array<std::uint8_t, 32> canonicalSourceDigest{};
    std::size_t canonicalSourceLength = 0;
    std::uint64_t metersPerUnitBits = 0;
    bool constructionFramePresent = false;
    std::array<std::uint64_t, 8> constructionFrameBits{};
    // Empty for this B2 extrusion constructor. The field is retained so a later
    // frozen-shell observer cannot silently substitute an unshelled binding.
    std::vector<std::array<std::uint8_t, 32>> frozenShellBindings;
};

enum class ObservedCellKind { Vertex, Edge, Face };
enum class ObservedSupport {
    Point,
    Line,
    Circle,
    Plane,
    Cylinder,
    SurfaceOfExtrusion
};
enum class RepresentationKind {
    Curve3D,
    PlanarPCurve,
    CylindricalPCurve,
    GeneratedPCurve,
    Seam,
    Regularity,
    VertexLink
};

struct RepresentationOwner {
    RepresentationKind kind = RepresentationKind::Curve3D;
    std::size_t ownerOccurrence = 0;
    std::size_t representationOccurrence = 0;
    bool stored = false;
    bool forward = false;
};

struct ObservedCell {
    ObservedCellKind kind = ObservedCellKind::Vertex;
    ObservedSupport support = ObservedSupport::Point;
    SemanticKey expectedKey;
    std::size_t nativeOccurrence = 0;
    std::optional<AngularInterval> completeInterval;
    std::vector<RepresentationOwner> representations;
    std::vector<std::size_t> connectedVertexOccurrences;
};

struct ObservationDebt {
    std::size_t stages = 0;
    std::size_t visits = 0;
    std::size_t faceEdgeCensus = 0;
    bool exhausted = false;
};

struct BoundaryObservation {
    // A cold/read-only observer fills this from a private detached capture. The
    // observation is never an expectation input and confers no edit authority.
    std::vector<ObservedCell> cells;
    std::array<std::uint8_t, 32> exactGeometryCommitment{};
    std::size_t vertexCount = 0;
    std::size_t edgeCount = 0;
    std::size_t faceCount = 0;
    std::size_t wireCount = 0;
    std::size_t pcurveCount = 0;
    std::size_t seamOwnerCount = 0;
    ObservationDebt debt;
};

namespace detail { struct B3WitnessFactory; }

class CorrespondenceWitness final {
public:
    CorrespondenceWitness(const CorrespondenceWitness&) = default;
    CorrespondenceWitness& operator=(const CorrespondenceWitness&) = default;
    const SourceBinding& source() const noexcept { return source_; }
    const ExtrusionExpectation& expectation() const noexcept { return expectation_; }
    const BoundaryObservation& observation() const noexcept { return observation_; }
    const std::array<std::uint8_t, 32>& geometryCommitment() const noexcept {
        return observation_.exactGeometryCommitment;
    }
private:
    SourceBinding source_;
    ExtrusionExpectation expectation_;
    BoundaryObservation observation_;
    CorrespondenceWitness(SourceBinding source, ExtrusionExpectation expectation,
                          BoundaryObservation observation)
        : source_(std::move(source)), expectation_(std::move(expectation)),
          observation_(std::move(observation)) {}
    friend struct detail::B3WitnessFactory;
};

// Deliberately has no Success enumerator and no witness member in B2. This
// prevents a caller from treating a well-formed observation, equal volume or a
// valid native shape as correspondence before B3 implements the whole matcher.
enum class InspectionStatus {
    MatcherUnavailableUntilB3,
    InvalidCanonicalSource,
    UnsupportedExpectation,
    IncompleteObservation,
    Mismatch,
    Cancelled,
    Budget
};
struct InspectionResult {
    InspectionStatus status = InspectionStatus::MatcherUnavailableUntilB3;
    explicit operator bool() const noexcept { return false; }
};

inline InspectionResult MatcherUnavailableUntilB3() noexcept { return {}; }

// Observation comparison is useful to the B2 guard without accepting either
// capture. It is intentionally strict and remains distinct from correspondence.
inline bool SameObservationIdentity(const BoundaryObservation& left,
                                    const BoundaryObservation& right) noexcept {
    if (left.exactGeometryCommitment != right.exactGeometryCommitment
        || left.vertexCount != right.vertexCount || left.edgeCount != right.edgeCount
        || left.faceCount != right.faceCount || left.wireCount != right.wireCount
        || left.pcurveCount != right.pcurveCount
        || left.seamOwnerCount != right.seamOwnerCount
        || left.cells.size() != right.cells.size()) return false;
    for (std::size_t i = 0; i < left.cells.size(); ++i) {
        const auto& a = left.cells[i]; const auto& b = right.cells[i];
        if (a.kind != b.kind || a.support != b.support
            || a.nativeOccurrence != b.nativeOccurrence
            || a.expectedKey.loopOrdinal != b.expectedKey.loopOrdinal
            || a.expectedKey.elementOrdinal != b.expectedKey.elementOrdinal
            || a.expectedKey.loopIdentifier != b.expectedKey.loopIdentifier
            || a.expectedKey.elementIdentifier != b.expectedKey.elementIdentifier
            || a.completeInterval.has_value() != b.completeInterval.has_value()
            || a.representations.size() != b.representations.size()
            || a.connectedVertexOccurrences != b.connectedVertexOccurrences) return false;
        if (a.completeInterval && (a.completeInterval->startDegrees != b.completeInterval->startDegrees
            || a.completeInterval->sweepDegrees != b.completeInterval->sweepDegrees
            || a.completeInterval->startRadians != b.completeInterval->startRadians
            || a.completeInterval->endRadians != b.completeInterval->endRadians)) return false;
        for (std::size_t owner = 0; owner < a.representations.size(); ++owner) {
            const auto& x = a.representations[owner];
            const auto& y = b.representations[owner];
            if (x.kind != y.kind || x.ownerOccurrence != y.ownerOccurrence
                || x.representationOccurrence != y.representationOccurrence
                || x.stored != y.stored || x.forward != y.forward) return false;
        }
    }
    return true;
}

} // namespace core3d::complete_profile_source
