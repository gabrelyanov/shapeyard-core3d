#pragma once

// Complete, counted correspondence for a privately owned Profile extrusion.
#include "ProfileSourceBoundaryExpectation.hxx"
#include "SavedBooleanResultCorrespondence.hxx"
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepTools.hxx>
#include <CommonCrypto/CommonDigest.h>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ios>
#include <locale>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <set>
#include <streambuf>
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
    VertexLink,
    Polygon3D,
    PolygonOnTriangulation
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

enum class InspectionStatus {
    MatchedCompleteExtrusion,
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
    std::optional<CorrespondenceWitness> witness;
    explicit operator bool() const noexcept {
        return status == InspectionStatus::MatchedCompleteExtrusion
            && witness.has_value();
    }
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

namespace detail {
namespace old = core3d::saved_cut_whole_result;
namespace d = core3d::enclosure_correspondence::detail;
namespace od = core3d::saved_cut_bore_result::detail;
namespace whole = core3d::saved_boolean_result::detail;
namespace tb = core3d::retained_topology_budget;

struct B3WitnessFactory {
    static CorrespondenceWitness Make(SourceBinding source,
        ExtrusionExpectation expectation, BoundaryObservation observation) {
        return CorrespondenceWitness(std::move(source), std::move(expectation),
            std::move(observation));
    }
};

inline bool SameWire(const std::vector<std::pair<std::size_t, bool>>& actual,
                     const std::vector<std::pair<std::size_t, bool>>& expected) {
    if (actual.size() != expected.size()) return false;
    const std::set<std::pair<std::size_t, bool>> a(actual.begin(), actual.end());
    const std::set<std::pair<std::size_t, bool>> b(expected.begin(), expected.end());
    return a.size() == actual.size() && a == b;
}

inline gp_Vec Point(const gp_Pnt& point) { return d::V(point); }

inline std::size_t EdgeRole(const ExpectedEdgeUse& use,
                            const ExtrusionExpectation& expected) {
    switch (use.level) {
        case EdgeLevel::LowerBoundary: return use.edge;
        case EdgeLevel::UpperBoundary:
            return expected.boundaryEdges.size() + use.edge;
        case EdgeLevel::Extrusion:
            return 2 * expected.boundaryEdges.size() + use.edge;
    }
    return std::numeric_limits<std::size_t>::max();
}

inline bool Source(SourceBinding& result, const profile::Parameters& parameters,
                   const ExtrusionExpectation& expected,
                   ProfileProducerAccounting& accounting) {
    result = {};
    if (accounting.stopped()
        || !accounting.visit(expected.canonicalValues.size())) return false;
    result.canonicalSourceBytes.resize(expected.canonicalValues.size() * sizeof(double));
    for (std::size_t i = 0; i < expected.canonicalValues.size(); ++i) {
        const auto bits = retained_solid::Bits(expected.canonicalValues[i]);
        for (unsigned byte = 0; byte < 8; ++byte)
            result.canonicalSourceBytes[8 * i + byte] =
                std::uint8_t(bits >> (56 - 8 * byte));
    }
    result.canonicalSourceLength = result.canonicalSourceBytes.size();
    if (result.canonicalSourceBytes.empty()
        || !CC_SHA256(result.canonicalSourceBytes.data(),
            CC_LONG(result.canonicalSourceBytes.size()),
            result.canonicalSourceDigest.data())) return false;
    result.metersPerUnitBits = retained_solid::Bits(parameters.metersPerUnit);
    result.constructionFramePresent = parameters.constructionFrame.has_value();
    if (parameters.constructionFrame)
        for (unsigned i = 0; i < 8; ++i)
            result.constructionFrameBits[i] =
                retained_solid::Bits(parameters.constructionFrame->values[i]);
    return true;
}

inline constexpr std::size_t MaximumGeometryBytes = 8 * 1024 * 1024;
class GeometryBuffer final : public std::streambuf {
    std::vector<char> bytes_ = std::vector<char>(MaximumGeometryBytes);
public:
    GeometryBuffer() { setp(bytes_.data(), bytes_.data() + bytes_.size()); }
    std::size_t size() const { return std::size_t(pptr() - pbase()); }
    const char* data() const { return bytes_.data(); }
protected:
    int_type overflow(int_type) override { return traits_type::eof(); }
};

inline bool Commit(const TopoDS_Shape& shape, const tb::Census& census,
                   ProfileProducerAccounting& accounting,
                   std::array<std::uint8_t, 32>& digest) {
    if (accounting.stopped()
        || !tb::ReserveTraversal(census, accounting.budget,
            accounting.kernelSite)) return false;
    GeometryBuffer buffer;
    std::ostream stream(&buffer);
    stream.imbue(std::locale::classic());
    BRepTools::Write(shape, stream, Standard_False, Standard_False,
        TopTools_FormatVersion_VERSION_3);
    return !accounting.stopped() && stream.good() && buffer.size() > 0
        && buffer.size() < MaximumGeometryBytes
        && CC_SHA256(buffer.data(), CC_LONG(buffer.size()), digest.data());
}

inline bool CircleMatch(const d::Curve& actual,
                        const ExpectedBoundaryEdgePair& expected,
                        bool upper, const gp_Vec& extrusion,
                        double mm, double error) {
    if (!actual.circle || !expected.interval) return false;
    const gp_Vec center = Point(expected.supportCenter) + (upper ? extrusion : gp_Vec{});
    const double radius = expected.supportX.Magnitude();
    const double a = actual.a.Magnitude(), b = actual.b.Magnitude();
    const double wantedSpan = std::abs(expected.interval->endRadians
                                      - expected.interval->startRadians);
    if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(a)
        || !std::isfinite(b) || a <= 0 || b <= 0
        || !d::Close(actual.c, center, mm, error)
        || std::abs(a - radius) * mm > error
        || std::abs(b - radius) * mm > error
        || std::abs(actual.a.Dot(actual.b)) / (a * b) * radius * mm > error
        || std::abs((actual.last - actual.first) - wantedSpan) * radius * mm > error)
        return false;
    const gp_Vec observedNormal = actual.a.Crossed(actual.b);
    const gp_Vec expectedNormal = expected.supportX.Crossed(expected.supportY);
    const double on = observedNormal.Magnitude(), en = expectedNormal.Magnitude();
    return on > 0 && en > 0
        && observedNormal.Crossed(expectedNormal).Magnitude() / (on * en)
            * radius * mm <= error;
}

inline bool PlaneContains(const d::Surface& surface,
                          const std::vector<gp_Vec>& points,
                          double mm, double error) {
    if (surface.cylinder || points.empty()) return false;
    const gp_Vec normal = surface.x.Crossed(surface.y);
    const double magnitude = normal.Magnitude();
    if (!std::isfinite(magnitude) || magnitude <= 0) return false;
    for (const auto& point : points)
        if (std::abs((point - surface.c).Dot(normal)) / magnitude * mm > error)
            return false;
    return true;
}

inline bool CylinderMatch(const d::Surface& surface,
                          const ExpectedBoundaryEdgePair& edge,
                          const gp_Vec& extrusion, double mm, double error) {
    if (!surface.cylinder) return false;
    const double radius = edge.supportX.Magnitude();
    const double x = surface.x.Magnitude(), y = surface.y.Magnitude();
    const double z = surface.z.Magnitude(), e = extrusion.Magnitude();
    if (!std::isfinite(radius) || radius <= 0 || x <= 0 || y <= 0
        || z <= 0 || e <= 0 || std::abs(x - radius) * mm > error
        || std::abs(y - radius) * mm > error
        || surface.z.Crossed(extrusion).Magnitude() / (z * e)
            * radius * mm > error) return false;
    const gp_Vec delta = Point(edge.supportCenter) - surface.c;
    return delta.Crossed(surface.z).Magnitude() / z * mm <= error;
}

inline bool CompletePCurveIdentity(const d::Curve& curve, const d::PCurve& pcurve,
                                   const d::Surface& surface,
                                   double mm, double error) {
    if (surface.cylinder || !curve.circle || !pcurve.circle) return false;
    const gp_Vec center = surface.c + surface.x * pcurve.c.X()
        + surface.y * pcurve.c.Y();
    const gp_Vec a = surface.x * pcurve.a.X() + surface.y * pcurve.a.Y();
    const gp_Vec b = surface.x * pcurve.b.X() + surface.y * pcurve.b.Y();
    double coefficients = 0, residual = 0, residualMM = 0;
    if (!d::TrimUpperAdd(d::Norm(a - curve.a), d::Norm(b - curve.b), coefficients)
        || !d::TrimUpperAdd(d::Norm(center - curve.c), coefficients, residual)
        || !d::TrimUpperMultiply(residual, mm, residualMM)) return false;
    for (const auto& box : surface.boxes) {
        const double ru = std::hypot(pcurve.a.X(), pcurve.b.X());
        const double rv = std::hypot(pcurve.a.Y(), pcurve.b.Y());
        if (pcurve.c.X() - ru < box[0] || pcurve.c.X() + ru > box[1]
            || pcurve.c.Y() - rv < box[2] || pcurve.c.Y() + rv > box[3]) return false;
    }
    return d::TrimResidualWithin(curve, pcurve, surface, mm, residualMM, error);
}

} // namespace detail

inline InspectionResult InspectCompleteProfileBase(
    const TopoDS_Shape& detached, const profile::Parameters& parameters,
    ProfileProducerAccounting& accounting) noexcept {
    namespace x = detail;
    InspectionResult result;
    const auto fail = [&](InspectionStatus status) {
        InspectionResult refused; refused.status = accounting.stopped()
            ? InspectionStatus::Cancelled
            : accounting.budget.exhausted ? InspectionStatus::Budget : status;
        return refused;
    };
    try {
        if (accounting.stopped()) return fail(InspectionStatus::Cancelled);
        ExtrusionExpectation expected;
        if (!ConstructExtrusionExpectation(parameters, accounting, expected))
            return fail(InspectionStatus::UnsupportedExpectation);
        if (detached.IsNull() || detached.ShapeType() != TopAbs_SOLID
            || detached.Orientation() != TopAbs_FORWARD || parameters.definition.revolve
            || !parameters.shells.empty()) return fail(InspectionStatus::IncompleteObservation);

        x::tb::Census census;
        if (x::tb::CensusTopology(detached, accounting.budget,
                accounting.cancelled, census, accounting.kernelSite, true)
                != x::tb::WalkStatus::Completed)
            return fail(InspectionStatus::IncompleteObservation);
        // Collection, point-owner analysis, mapping, pcurves, representation
        // ownership, links, validity and exterior classification are distinct
        // complete passes over the same private topology.
        for (unsigned pass = 0; pass < 8; ++pass)
            if (!x::tb::ReserveTraversal(census, accounting.budget,
                    accounting.kernelSite)) return fail(InspectionStatus::Budget);

        retained_solid::Envelope sourceView;
        sourceView.metersPerUnit = parameters.metersPerUnit;
        x::old::detail::Graph graph;
        if (!x::old::detail::Collect(detached, sourceView, accounting.cancelled,
                graph, expected.loops.size()))
            return fail(InspectionStatus::IncompleteObservation);
        if (graph.edges.size() != expected.edgeCellCount()
            || graph.faces.size() != expected.faceCellCount())
            return fail(InspectionStatus::Mismatch);

        // Collect every generated planar pcurve and its relative location
        // before the arithmetic allowance is computed and frozen. Stored
        // pcurves were already collected with the graph; this completes the
        // same preflight for representations generated on demand below.
        std::map<std::pair<unsigned, unsigned>, std::vector<x::d::PCurve>> allPCurves
            = graph.pcurves;
        for (unsigned f = 0; f < graph.faces.size(); ++f)
            for (const auto& wire : graph.faces[f].wires)
                for (const auto& use : wire) {
                    const auto key = std::make_pair(f, use.edge);
                    if (!allPCurves.count(key)) {
                        x::d::PCurve pcurve;
                        if (!accounting.visit(1)
                            || !x::d::PairLocation(graph.edges[use.edge].shape.Location(),
                                graph.faces[f].surface.location, graph.budget)
                            || !x::d::ReadPCurve(graph.edges[use.edge].shape,
                                graph.faces[f].surface, graph.edges[use.edge].curve,
                                pcurve, graph.budget)
                            || !x::d::PCurveMagnitude(pcurve, graph.faces[f].surface,
                                expected.millimetresPerUnit, graph.budget))
                            return fail(InspectionStatus::IncompleteObservation);
                        allPCurves.emplace(key, std::vector<x::d::PCurve>{pcurve});
                    }
                }

        std::vector<x::old::detail::PointWitness> pointWitnesses;
        if (!accounting.visit(census.occurrences)
            || !x::old::detail::PointOwners(graph, expected.millimetresPerUnit,
                accounting.cancelled, graph.budget, pointWitnesses))
            return fail(InspectionStatus::IncompleteObservation);
        for (const auto& pair : expected.vertices)
            for (const auto& point : {pair.lower, pair.upper})
                if (!x::d::Track(std::abs(point.X()) + std::abs(point.Y())
                    + std::abs(point.Z()), expected.millimetresPerUnit,
                    graph.budget)) return fail(InspectionStatus::IncompleteObservation);
        if (!x::d::Track(graph.budget.maximumLocationCompositionMagnitude,
                expected.millimetresPerUnit, graph.budget))
            return fail(InspectionStatus::IncompleteObservation);
        const double error = std::max(1e-9, 2048
            * std::numeric_limits<double>::epsilon()
            * graph.budget.arithmeticMagnitudeMM);
        if (!std::isfinite(error) || error > 1e-6)
            return fail(InspectionStatus::IncompleteObservation);
        for (const auto& point : pointWitnesses)
            if (!x::d::Close(point.vertex, point.value,
                    expected.millimetresPerUnit, error))
                return fail(InspectionStatus::Mismatch);
        const double fixedArithmetic = graph.budget.arithmeticMagnitudeMM;
        const double fixedComposition = graph.budget.maximumLocationCompositionMagnitude;

        TopTools_IndexedMapOfShape vertices;
        for (const auto& edge : graph.edges) {
            vertices.Add(edge.vertices[0]); vertices.Add(edge.vertices[1]);
        }
        if (vertices.Extent() != static_cast<int>(expected.vertexCellCount()))
            return fail(InspectionStatus::Mismatch);
        std::vector<int> vertexRole(vertices.Extent(), -1);
        std::vector<bool> claimedVertices(expected.vertexCellCount(), false);
        for (int i = 1; i <= vertices.Extent(); ++i) {
            if (!accounting.visit(expected.vertexCellCount()))
                return fail(InspectionStatus::Budget);
            const gp_Vec actual = x::d::V(BRep_Tool::Pnt(TopoDS::Vertex(vertices(i))));
            unsigned matches = 0; std::size_t role = 0;
            for (std::size_t v = 0; v < expected.vertices.size(); ++v)
                for (unsigned upper = 0; upper < 2; ++upper) {
                    const std::size_t candidate = 2 * v + upper;
                    const gp_Vec wanted = x::Point(upper
                        ? expected.vertices[v].upper : expected.vertices[v].lower);
                    if (x::d::Close(actual, wanted, expected.millimetresPerUnit,
                            error)) { ++matches; role = candidate; }
                }
            if (matches != 1 || claimedVertices[role])
                return fail(InspectionStatus::Mismatch);
            claimedVertices[role] = true; vertexRole[i - 1] = int(role);
        }
        if (std::find(claimedVertices.begin(), claimedVertices.end(), false)
            != claimedVertices.end()) return fail(InspectionStatus::Mismatch);

        const std::size_t boundaryCount = expected.boundaryEdges.size();
        const std::size_t edgeRoleCount = expected.edgeCellCount();
        std::vector<int> edgeRole(graph.edges.size(), -1);
        std::vector<bool> roleForward(graph.edges.size(), false), claimedEdges(edgeRoleCount, false);
        const gp_Vec extrusion = x::Point(expected.vertices.front().upper)
            - x::Point(expected.vertices.front().lower);
        for (std::size_t i = 0; i < graph.edges.size(); ++i) {
            if (!accounting.visit(edgeRoleCount + 1))
                return fail(InspectionStatus::Budget);
            const auto& edge = graph.edges[i];
            const int a = vertexRole[vertices.FindIndex(edge.vertices[0]) - 1];
            const int b = vertexRole[vertices.FindIndex(edge.vertices[1]) - 1];
            unsigned matches = 0; std::size_t matched = 0; bool forward = false;
            for (std::size_t j = 0; j < boundaryCount; ++j)
                for (unsigned upper = 0; upper < 2; ++upper) {
                    const auto& wanted = expected.boundaryEdges[j];
                    const int start = int(2 * wanted.startVertex + upper);
                    const int end = int(2 * wanted.endVertex + upper);
                    if (!((a == start && b == end) || (a == end && b == start))) continue;
                    if (wanted.support == SegmentSupport::Line) {
                        if (edge.curve.circle) continue;
                    } else if (!x::CircleMatch(edge.curve, wanted, upper != 0,
                            extrusion, expected.millimetresPerUnit, error)) continue;
                    ++matches; matched = upper * boundaryCount + j;
                    forward = a == start && b == end;
                    if (start == end) {
                        const double theta = wanted.interval->startRadians;
                        gp_Vec tangent = wanted.supportX * (-std::sin(theta))
                            + wanted.supportY * std::cos(theta);
                        if (wanted.interval->sweepDegrees < 0) tangent.Reverse();
                        const gp_Vec actualTangent = edge.curve.a
                            * (-std::sin(edge.curve.first))
                            + edge.curve.b * std::cos(edge.curve.first);
                        forward = actualTangent.Dot(tangent) > 0;
                    }
                }
            for (std::size_t j = 0; j < expected.extrusionEdges.size(); ++j) {
                const int lower = int(2 * expected.extrusionEdges[j].vertex);
                const int upper = lower + 1;
                if (edge.curve.circle
                    || !((a == lower && b == upper) || (a == upper && b == lower))) continue;
                ++matches; matched = 2 * boundaryCount + j;
                forward = a == lower && b == upper;
            }
            if (matches != 1 || claimedEdges[matched]
                || !x::od::Bits(BRep_Tool::Parameter(edge.vertices[0], edge.shape),
                    edge.curve.first)
                || !x::od::Bits(BRep_Tool::Parameter(edge.vertices[1], edge.shape),
                    edge.curve.last)) return fail(InspectionStatus::Mismatch);
            claimedEdges[matched] = true; edgeRole[i] = int(matched);
            roleForward[i] = forward;
        }
        if (std::find(claimedEdges.begin(), claimedEdges.end(), false)
            != claimedEdges.end()) return fail(InspectionStatus::Mismatch);

        struct ExpectedFace {
            std::vector<std::vector<std::pair<std::size_t, bool>>> wires;
            int side = -1;
            int cap = -1;
        };
        std::vector<ExpectedFace> expectedFaces;
        for (unsigned cap = 0; cap < 2; ++cap) {
            ExpectedFace face; face.cap = int(cap);
            for (const auto& wire : expected.caps[cap].wires) {
                std::vector<std::pair<std::size_t, bool>> uses;
                for (const auto& use : wire.uses)
                    uses.emplace_back(x::EdgeRole(use, expected), use.forward);
                face.wires.push_back(std::move(uses));
            }
            expectedFaces.push_back(std::move(face));
        }
        for (std::size_t side = 0; side < expected.sides.size(); ++side) {
            ExpectedFace face; face.side = int(side);
            std::vector<std::pair<std::size_t, bool>> uses;
            for (const auto& use : expected.sides[side].boundary)
                uses.emplace_back(x::EdgeRole(use, expected), use.forward);
            face.wires.push_back(std::move(uses));
            expectedFaces.push_back(std::move(face));
        }
        std::vector<int> faceRole(graph.faces.size(), -1);
        std::vector<bool> claimedFaces(expectedFaces.size(), false);
        for (std::size_t f = 0; f < graph.faces.size(); ++f) {
            if (!accounting.visit(expectedFaces.size() + census.edgeUsesUnderFaces))
                return fail(InspectionStatus::Budget);
            const auto& actual = graph.faces[f];
            std::vector<std::vector<std::pair<std::size_t, bool>>> wires;
            for (const auto& wire : actual.wires) {
                std::vector<std::pair<std::size_t, bool>> uses;
                for (const auto& use : wire) uses.emplace_back(
                    std::size_t(edgeRole[use.edge]), use.forward == roleForward[use.edge]);
                wires.push_back(std::move(uses));
            }
            unsigned matches = 0; std::size_t role = 0;
            for (std::size_t r = 0; r < expectedFaces.size(); ++r) {
                const auto& wanted = expectedFaces[r];
                if (wanted.wires.size() != wires.size()) continue;
                std::set<std::size_t> used; bool wireMatch = true;
                for (const auto& wire : wires) {
                    unsigned count = 0; std::size_t id = 0;
                    for (std::size_t w = 0; w < wanted.wires.size(); ++w)
                        if (x::SameWire(wire, wanted.wires[w])) { ++count; id = w; }
                    if (count != 1 || !used.insert(id).second) { wireMatch = false; break; }
                }
                if (!wireMatch) continue;
                bool support = false;
                if (wanted.cap >= 0) {
                    std::vector<gp_Vec> points;
                    for (const auto& pair : expected.vertices)
                        points.push_back(x::Point(wanted.cap ? pair.upper : pair.lower));
                    support = x::PlaneContains(actual.surface, points,
                        expected.millimetresPerUnit, error);
                } else {
                    const auto& side = expected.sides[wanted.side];
                    const auto& edge = expected.boundaryEdges[side.boundaryEdge];
                    if (edge.support == SegmentSupport::Line) {
                        const auto& a = expected.vertices[edge.startVertex];
                        const auto& b = expected.vertices[edge.endVertex];
                        support = x::PlaneContains(actual.surface,
                            {x::Point(a.lower), x::Point(b.lower),
                             x::Point(a.upper), x::Point(b.upper)},
                            expected.millimetresPerUnit, error);
                    } else support = x::CylinderMatch(actual.surface, edge,
                        extrusion, expected.millimetresPerUnit, error);
                }
                if (support) { ++matches; role = r; }
            }
            if (matches != 1 || claimedFaces[role])
                return fail(InspectionStatus::Mismatch);
            claimedFaces[role] = true; faceRole[f] = int(role);
        }
        if (std::find(claimedFaces.begin(), claimedFaces.end(), false)
            != claimedFaces.end()) return fail(InspectionStatus::Mismatch);

        std::map<unsigned, unsigned> seams;
        std::vector<x::old::detail::HostWall> hostWalls;
        for (std::size_t side = 0; side < expected.sides.size(); ++side) {
            if (!expected.sides[side].periodicSeamOwner) continue;
            const std::size_t sideRole = side + 2;
            const auto face = std::find(faceRole.begin(), faceRole.end(), int(sideRole));
            const std::size_t seamRole = 2 * boundaryCount
                + *expected.sides[side].periodicSeamOwner;
            const auto seam = std::find(edgeRole.begin(), edgeRole.end(), int(seamRole));
            const std::size_t lowerRole = expected.sides[side].boundaryEdge;
            const std::size_t upperRole = boundaryCount + lowerRole;
            const auto lower = std::find(edgeRole.begin(), edgeRole.end(), int(lowerRole));
            const auto upper = std::find(edgeRole.begin(), edgeRole.end(), int(upperRole));
            if (face == faceRole.end() || seam == edgeRole.end()
                || lower == edgeRole.end() || upper == edgeRole.end())
                return fail(InspectionStatus::Mismatch);
            const unsigned faceID = unsigned(face - faceRole.begin());
            const unsigned seamID = unsigned(seam - edgeRole.begin());
            if (!seams.emplace(seamID, faceID).second)
                return fail(InspectionStatus::Mismatch);
            x::old::detail::HostWall wall;
            wall.face = faceID; wall.seam = seamID;
            wall.rims = {unsigned(lower - edgeRole.begin()),
                         unsigned(upper - edgeRole.begin())};
            hostWalls.push_back(wall);
        }

        for (const auto& entry : allPCurves) {
            const auto& surface = graph.faces[entry.first.first].surface;
            const auto& curve = graph.edges[entry.first.second].curve;
            for (const auto& pcurve : entry.second) {
                if (!accounting.visit(1)) return fail(InspectionStatus::Budget);
                const bool longPlanar = !surface.cylinder && curve.circle
                    && curve.last - curve.first > std::acos(-1.0);
                if (longPlanar
                    ? !x::CompletePCurveIdentity(curve, pcurve, surface,
                        expected.millimetresPerUnit, error)
                    : !x::d::PCurveIdentity(curve, pcurve, surface,
                        expected.millimetresPerUnit, error))
                    return fail(InspectionStatus::Mismatch);
            }
        }
        if (!accounting.visit(graph.edges.size() + census.edgeUsesUnderFaces)
            || !x::whole::RepresentationOwners(graph, seams, graph.budget)
            || graph.budget.arithmeticMagnitudeMM > fixedArithmetic
            || graph.budget.maximumLocationCompositionMagnitude > fixedComposition)
            return fail(InspectionStatus::Mismatch);
        if (!accounting.visit(vertices.Extent() + census.edgeUsesUnderFaces)
            || !x::whole::VertexLinks(graph, vertices, {}, hostWalls))
            return fail(InspectionStatus::Mismatch);

        long euler = long(vertices.Extent()) - long(graph.edges.size());
        for (const auto& face : graph.faces) euler += 2 - long(face.wires.size());
        if (euler != 2 - 2 * long(expected.loops.size() - 1))
            return fail(InspectionStatus::Mismatch);
        if (accounting.stopped()
            || !BRepCheck_Analyzer(detached, Standard_True).IsValid())
            return fail(InspectionStatus::IncompleteObservation);
        BRepClass3d_SolidClassifier outside(detached);
        outside.PerformInfinitePoint(Precision::Confusion());
        if (accounting.stopped() || outside.State() != TopAbs_OUT)
            return fail(InspectionStatus::Mismatch);

        BoundaryObservation observation;
        observation.vertexCount = std::size_t(vertices.Extent());
        observation.edgeCount = graph.edges.size();
        observation.faceCount = graph.faces.size();
        for (const auto& face : graph.faces) observation.wireCount += face.wires.size();
        observation.seamOwnerCount = seams.size();
        observation.pcurveCount = allPCurves.size();
        if (!x::Commit(detached, census, accounting,
                observation.exactGeometryCommitment))
            return fail(InspectionStatus::IncompleteObservation);
        for (std::size_t v = 0; v < expected.vertices.size(); ++v)
            for (unsigned upper = 0; upper < 2; ++upper) {
                ObservedCell cell; cell.kind = ObservedCellKind::Vertex;
                cell.support = ObservedSupport::Point;
                cell.expectedKey = expected.vertices[v].key;
                cell.nativeOccurrence = 2 * v + upper;
                cell.representations.push_back({RepresentationKind::VertexLink,
                    cell.nativeOccurrence, 0, true, true});
                observation.cells.push_back(std::move(cell));
            }
        for (std::size_t i = 0; i < graph.edges.size(); ++i) {
            ObservedCell cell; cell.kind = ObservedCellKind::Edge;
            const std::size_t role = std::size_t(edgeRole[i]);
            if (role < 2 * boundaryCount) {
                const auto& wanted = expected.boundaryEdges[role % boundaryCount];
                cell.expectedKey = wanted.key; cell.completeInterval = wanted.interval;
                cell.support = wanted.support == SegmentSupport::Line
                    ? ObservedSupport::Line : ObservedSupport::Circle;
            } else {
                cell.expectedKey = expected.extrusionEdges[role - 2 * boundaryCount].key;
                cell.support = ObservedSupport::Line;
            }
            cell.nativeOccurrence = i;
            cell.connectedVertexOccurrences = {
                std::size_t(vertexRole[vertices.FindIndex(graph.edges[i].vertices[0]) - 1]),
                std::size_t(vertexRole[vertices.FindIndex(graph.edges[i].vertices[1]) - 1])};
            const auto data = Handle(BRep_TEdge)::DownCast(graph.edges[i].shape.TShape());
            std::size_t representation = 0;
            for (BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());
                    it.More(); it.Next(), ++representation) {
                const auto& native = it.Value(); RepresentationKind kind;
                if (native->IsCurve3D()) kind = RepresentationKind::Curve3D;
                else if (native->IsCurveOnClosedSurface()) kind = RepresentationKind::Seam;
                else if (native->IsCurveOnSurface()) kind = RepresentationKind::PlanarPCurve;
                else if (native->IsRegularity()) kind = RepresentationKind::Regularity;
                else if (native->IsPolygon3D()) kind = RepresentationKind::Polygon3D;
                else kind = RepresentationKind::PolygonOnTriangulation;
                cell.representations.push_back({kind, i, representation, true,
                    roleForward[i]});
            }
            observation.cells.push_back(std::move(cell));
        }
        for (std::size_t f = 0; f < graph.faces.size(); ++f) {
            ObservedCell cell; cell.kind = ObservedCellKind::Face;
            const int role = faceRole[f];
            if (role < 2) {
                cell.support = ObservedSupport::Plane;
                cell.expectedKey.loopOrdinal = std::numeric_limits<std::size_t>::max();
                cell.expectedKey.elementOrdinal = std::size_t(role);
            } else {
                const auto& side = expected.sides[std::size_t(role - 2)];
                cell.expectedKey = side.key;
                cell.support = side.support == SegmentSupport::Line
                    ? ObservedSupport::Plane : ObservedSupport::Cylinder;
            }
            cell.nativeOccurrence = f; observation.cells.push_back(std::move(cell));
        }
        observation.debt.stages = accounting.budget.buildStages;
        observation.debt.visits = accounting.budget.topologyVisits;
        observation.debt.faceEdgeCensus = std::size_t(census.faceEdge.Extent());
        observation.debt.exhausted = accounting.budget.exhausted;
        SourceBinding source;
        if (!x::Source(source, parameters, expected, accounting))
            return fail(InspectionStatus::InvalidCanonicalSource);
        result.status = InspectionStatus::MatchedCompleteExtrusion;
        result.witness = x::B3WitnessFactory::Make(std::move(source),
            std::move(expected), std::move(observation));
        return result;
    } catch (...) {
        return fail(InspectionStatus::IncompleteObservation);
    }
}

} // namespace core3d::complete_profile_source
