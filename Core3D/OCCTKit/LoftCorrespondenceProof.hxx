#pragma once

// Independent continuous and boundary proof for detached correspondence
// geometry. This component derives every expectation from persisted values;
// it consumes no correspondence table emitted by the builder.
#include "LoftCorrespondenceAdapter.hxx"
#include "PlanarSweepSolid.hxx"

#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Plane.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <math_DirectPolynomialRoots.hxx>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <vector>

namespace core3d::loft_correspondence::proof {

struct Inspection {
    MappingInspection mapping;
    double millimetersPerUnit = 0;
    double expectedVolume = 0;
    std::array<double, 6> expectedBounds{};
    double minimumSignedArea = 0;
    std::size_t intervalSubdivisions = 0;
};

enum class ProofStatus : std::uint8_t {
    Proven = 0,
    InvalidDefinition,
    Cancelled,
    BudgetDenied,
    InvalidSolid,
    WrongTopology,
    WrongBoundary,
    WrongPCurve,
    WrongVolume,
    WrongBounds,
    SelfInterference,
    KernelFailure
};

struct Result {
    ProofStatus status = ProofStatus::InvalidDefinition;
    TopoDS_Solid solid;
    double volume = 0;
    std::array<double, 6> bounds{};
    std::size_t faceCount = 0;
    std::size_t edgeCount = 0;
    std::size_t vertexCount = 0;
    std::uint32_t diagnosticStage = 0;
};

namespace detail {
struct Polynomial {
    std::array<double, 5> c{}; // ascending powers
    int degree = 0;
};

inline int Degree(const Polynomial& value) noexcept {
    int degree = std::min(4, std::max(0, value.degree));
    while (degree && value.c[std::size_t(degree)] == 0) --degree;
    return degree;
}

inline double Value(const Polynomial& value, double x) noexcept {
    const int degree = Degree(value);
    long double result = value.c[std::size_t(degree)];
    for (int i = degree - 1; i >= 0; --i)
        result = result * static_cast<long double>(x) + value.c[std::size_t(i)];
    return static_cast<double>(result);
}

struct Interval { double low = 0, high = 0; };

inline double Down(double value) noexcept {
    return std::nextafter(value, -std::numeric_limits<double>::infinity());
}
inline double Up(double value) noexcept {
    return std::nextafter(value, std::numeric_limits<double>::infinity());
}
inline Interval Add(Interval a, Interval b) noexcept {
    return {Down(a.low + b.low), Up(a.high + b.high)};
}
inline Interval Multiply(Interval a, Interval b) noexcept {
    const std::array<double, 4> products = {
        a.low * b.low, a.low * b.high, a.high * b.low, a.high * b.high};
    return {Down(*std::min_element(products.begin(), products.end())),
            Up(*std::max_element(products.begin(), products.end()))};
}
inline Interval Range(const Polynomial& value, Interval x) noexcept {
    const int degree = Degree(value);
    Interval result{value.c[std::size_t(degree)], value.c[std::size_t(degree)]};
    for (int i = degree - 1; i >= 0; --i)
        result = Add(Multiply(result, x), {value.c[std::size_t(i)], value.c[std::size_t(i)]});
    return result;
}

inline Polynomial Derivative(const Polynomial& value) noexcept {
    Polynomial result;
    const int degree = Degree(value);
    result.degree = std::max(0, degree - 1);
    for (int i = 1; i <= degree; ++i)
        result.c[std::size_t(i - 1)] = value.c[std::size_t(i)] * i;
    return result;
}

inline Polynomial Square(const Polynomial& value) noexcept {
    Polynomial result;
    const int degree = Degree(value);
    result.degree = std::min(4, degree * 2);
    for (int i = 0; i <= degree; ++i)
        for (int j = 0; j <= degree && i + j <= 4; ++j)
            result.c[std::size_t(i + j)] += value.c[std::size_t(i)] * value.c[std::size_t(j)];
    return result;
}

inline std::vector<double> Roots(const Polynomial& value) {
    std::vector<double> output;
    const int degree = Degree(value);
    if (!degree) return output;
    std::unique_ptr<math_DirectPolynomialRoots> roots;
    if (degree == 1)
        roots = std::make_unique<math_DirectPolynomialRoots>(value.c[1], value.c[0]);
    else if (degree == 2)
        roots = std::make_unique<math_DirectPolynomialRoots>(value.c[2], value.c[1], value.c[0]);
    else
        roots = std::make_unique<math_DirectPolynomialRoots>(value.c[3], value.c[2],
                                                              value.c[1], value.c[0]);
    if (!roots->IsDone() || roots->InfiniteRoots()) throw Standard_Failure("root isolation failed");
    for (int index = 1; index <= roots->NbSolutions(); ++index) {
        const double root = roots->Value(index);
        if (std::isfinite(root) && root > 0 && root < 1) output.push_back(root);
    }
    std::sort(output.begin(), output.end());
    output.erase(std::unique(output.begin(), output.end(), [](double a, double b) {
        return std::abs(a - b) <= 64 * std::numeric_limits<double>::epsilon()
            * std::max({1.0, std::abs(a), std::abs(b)});
    }), output.end());
    return output;
}

// All stationary points are enumerated by the direct low-degree root solver,
// then independently enclosed by sign-changing derivative intervals. The
// outward-rounded Horner interval of the original polynomial must be strictly
// positive over each enclosure. At most 64 bisections are shared by a span;
// uncertainty refuses instead of becoming sampled acceptance.
inline bool ProvePositive(const Polynomial& value,
                          std::size_t& subdivisions) {
    for (double endpoint : {0.0, 1.0})
        if (!(Range(value, {endpoint, endpoint}).low > 0)) return false;
    const Polynomial derivative = Derivative(value);
    const std::vector<double> roots = Roots(derivative);
    for (std::size_t index = 0; index < roots.size(); ++index) {
        const double root = roots[index];
        double low = index ? (roots[index - 1] + root) * 0.5 : 0.0;
        double high = index + 1 < roots.size() ? (root + roots[index + 1]) * 0.5 : 1.0;
        double fLow = Value(derivative, low), fHigh = Value(derivative, high);
        if (!std::isfinite(fLow) || !std::isfinite(fHigh)) return false;
        if (fLow == 0) low = Down(low);
        if (fHigh == 0) high = Up(high);
        fLow = Value(derivative, low); fHigh = Value(derivative, high);
        if (fLow != 0 && fHigh != 0 && std::signbit(fLow) == std::signbit(fHigh))
            return false; // repeated/uncertain derivative root
        bool certified = false;
        while (subdivisions < 64) {
            const Interval bound = Range(value, {low, high});
            if (bound.low > 0) { certified = true; break; }
            if (bound.high <= 0) return false;
            const double middle = low * 0.5 + high * 0.5;
            const double fMiddle = Value(derivative, middle);
            if (!std::isfinite(fMiddle)) return false;
            if (fMiddle == 0) {
                low = Down(middle); high = Up(middle);
            } else if (fLow == 0 || std::signbit(fLow) != std::signbit(fMiddle)) {
                high = middle; fHigh = fMiddle;
            } else {
                low = middle; fLow = fMiddle;
            }
            ++subdivisions;
        }
        if (!certified) return false;
    }
    return true;
}

inline Polynomial DeterminantPolynomial(const std::array<gp_Pnt, 4>& lower,
                                        const std::array<gp_Pnt, 4>& upper,
                                        Polynomial& aSquared,
                                        Polynomial& bSquared) noexcept {
    const std::array<double, 2> a0 = {lower[1].X() - lower[0].X(),
                                      lower[1].Y() - lower[0].Y()};
    const std::array<double, 2> a1 = {upper[1].X() - upper[0].X() - a0[0],
                                      upper[1].Y() - upper[0].Y() - a0[1]};
    const std::array<double, 2> b0 = {lower[3].X() - lower[0].X(),
                                      lower[3].Y() - lower[0].Y()};
    const std::array<double, 2> b1 = {upper[3].X() - upper[0].X() - b0[0],
                                      upper[3].Y() - upper[0].Y() - b0[1]};
    const auto det = [](const std::array<double, 2>& a,
                        const std::array<double, 2>& b) {
        return a[0] * b[1] - a[1] * b[0];
    };
    Polynomial determinant;
    determinant.degree = 2;
    determinant.c[0] = det(a0, b0);
    determinant.c[1] = det(a0, b1) + det(a1, b0);
    determinant.c[2] = det(a1, b1);
    aSquared.degree = bSquared.degree = 2;
    aSquared.c[0] = a0[0] * a0[0] + a0[1] * a0[1];
    aSquared.c[1] = 2 * (a0[0] * a1[0] + a0[1] * a1[1]);
    aSquared.c[2] = a1[0] * a1[0] + a1[1] * a1[1];
    bSquared.c[0] = b0[0] * b0[0] + b0[1] * b0[1];
    bSquared.c[1] = 2 * (b0[0] * b1[0] + b0[1] * b1[1]);
    bSquared.c[2] = b1[0] * b1[0] + b1[1] * b1[1];
    return determinant;
}

inline Polynomial ClearancePolynomial(const Polynomial& determinant,
                                      const Polynomial& lengthSquared,
                                      double minimum) noexcept {
    Polynomial result = Square(determinant);
    result.degree = 4;
    const double squared = minimum * minimum;
    for (int index = 0; index <= 2; ++index)
        result.c[std::size_t(index)] -= squared * lengthSquared.c[std::size_t(index)];
    return result;
}

inline bool Reserve(const retained_topology_budget::Census& census,
                    budget::Counter& counter, budget::Site site) noexcept {
    return budget::ReserveTraversal(census, counter, site);
}

inline bool PointOfVertex(const TopoDS_Vertex& vertex, gp_Pnt& point) noexcept {
    try { point = BRep_Tool::Pnt(vertex); return true; }
    catch (...) { return false; }
}

inline bool EdgeEndpoints(const TopoDS_Edge& edge, gp_Pnt& first,
                          gp_Pnt& last) noexcept {
    try {
        TopoDS_Vertex a, b;
        TopExp::Vertices(edge, a, b, Standard_True);
        return !a.IsNull() && !b.IsNull() && PointOfVertex(a, first) && PointOfVertex(b, last);
    } catch (...) { return false; }
}

inline int MatchEdge(const TopTools_IndexedMapOfShape& edges,
                     const gp_Pnt& a, const gp_Pnt& b, double tolerance) noexcept {
    int match = 0;
    for (int index = 1; index <= edges.Extent(); ++index) {
        gp_Pnt first, last;
        if (!EdgeEndpoints(TopoDS::Edge(edges(index)), first, last)) return 0;
        if ((!loft_correspondence::detail::SamePoint(first, a, tolerance)
             || !loft_correspondence::detail::SamePoint(last, b, tolerance))
            && (!loft_correspondence::detail::SamePoint(first, b, tolerance)
                || !loft_correspondence::detail::SamePoint(last, a, tolerance))) continue;
        if (match) return 0;
        match = index;
    }
    return match;
}

inline bool MatchSurface(const TopoDS_Face& face,
                         const std::array<gp_Pnt, 4>& expected,
                         double tolerance) noexcept {
    try {
        TopLoc_Location location;
        const Handle(Geom_BSplineSurface) surface =
            Handle(Geom_BSplineSurface)::DownCast(BRep_Tool::Surface(face, location));
        if (surface.IsNull() || surface->UDegree() != 1 || surface->VDegree() != 1
            || surface->NbUPoles() != 2 || surface->NbVPoles() != 2
            || surface->IsURational() || surface->IsVRational()) return false;
        double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
        BRepTools::UVBounds(face, u0, u1, v0, v1);
        if (std::abs(u0) > tolerance || std::abs(u1 - 1) > tolerance
            || std::abs(v0) > tolerance || std::abs(v1 - 1) > tolerance) return false;
        const std::array<gp_Pnt, 4> actual = {
            surface->Pole(1, 1).Transformed(location.Transformation()),
            surface->Pole(2, 1).Transformed(location.Transformation()),
            surface->Pole(1, 2).Transformed(location.Transformation()),
            surface->Pole(2, 2).Transformed(location.Transformation())};
        for (std::size_t index = 0; index < 4; ++index)
            if (!loft_correspondence::detail::SamePoint(actual[index], expected[index], tolerance))
                return false;
        return true;
    } catch (...) { return false; }
}

inline bool ConsistentPCurves(const TopoDS_Face& face, double tolerance,
                              std::size_t& uses) noexcept {
    try {
        TopLoc_Location surfaceLocation;
        const Handle(Geom_Surface) surface = BRep_Tool::Surface(face, surfaceLocation);
        if (surface.IsNull()) return false;
        std::size_t count = 0;
        for (TopExp_Explorer iterator(face, TopAbs_EDGE); iterator.More(); iterator.Next()) {
            ++uses; ++count;
            const TopoDS_Edge edge = TopoDS::Edge(iterator.Current());
            TopLoc_Location curveLocation;
            double first = 0, last = 0, pFirst = 0, pLast = 0;
            const Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, curveLocation, first, last);
            const Handle(Geom2d_Curve) pcurve =
                BRep_Tool::CurveOnSurface(edge, face, pFirst, pLast);
            if (curve.IsNull() || pcurve.IsNull() || !std::isfinite(first)
                || !std::isfinite(last) || first >= last
                || first != pFirst || last != pLast) return false;
            for (double parameter : {first, last}) {
                const gp_Pnt curvePoint = curve->Value(parameter).Transformed(
                    curveLocation.Transformation());
                const gp_Pnt2d uv = pcurve->Value(parameter);
                const gp_Pnt surfacePoint = surface->Value(uv.X(), uv.Y()).Transformed(
                    surfaceLocation.Transformation());
                if (!loft_correspondence::detail::SamePoint(curvePoint, surfacePoint, tolerance))
                    return false;
            }
        }
        return count == 4;
    } catch (...) { return false; }
}

inline TopAbs_Orientation Use(const TopoDS_Edge& edge,
                              const TopoDS_Face& face) noexcept {
    TopAbs_Orientation found = TopAbs_EXTERNAL;
    for (TopExp_Explorer iterator(face, TopAbs_EDGE); iterator.More(); iterator.Next()) {
        const TopoDS_Edge use = TopoDS::Edge(iterator.Current());
        if (!use.IsSame(edge)) continue;
        if (found != TopAbs_EXTERNAL) return TopAbs_INTERNAL;
        found = use.Orientation();
    }
    return found;
}
} // namespace detail

inline Admission Inspect(const Definition& definition,
                         const std::atomic_bool& cancelled,
                         budget::Counter& counter,
                         Inspection& output) noexcept {
    output = {};
    if (cancelled.load(std::memory_order_relaxed)) return Admission::Cancelled;
    try {
        MappingInspection mapping;
        const Admission mappingAdmission = loft_correspondence::detail::ValidateMapping(
            definition, mapping);
        if (mappingAdmission != Admission::Accepted) return mappingAdmission;
        rectangular_loft::Inspection base;
        if (rectangular_loft::Inspect(definition.base, base)
            != rectangular_loft::Admission::Accepted) return Admission::InvalidBase;
        Inspection candidate;
        candidate.mapping = mapping;
        candidate.millimetersPerUnit = base.millimetersPerUnit;
        candidate.expectedBounds = {INFINITY, INFINITY, INFINITY,
                                    -INFINITY, -INFINITY, -INFINITY};
        const double scale = definition.base.constructionFrame
            ? std::abs(definition.base.constructionFrame->values[7]) : 1.0;
        const double minimum = std::max(32 * Precision::Confusion() / scale,
            rectangular_loft::detail::minimumPhysical / (base.millimetersPerUnit * scale));
        double volume = 0;
        candidate.minimumSignedArea = INFINITY;
        for (std::size_t station = 0; station < definition.base.stations.size(); ++station) {
            for (gp_Pnt point : loft_correspondence::detail::AuthoredCorners(
                     definition, station, true)) {
                for (int axis = 0; axis < 3; ++axis) {
                    candidate.expectedBounds[std::size_t(axis)] = std::min(
                        candidate.expectedBounds[std::size_t(axis)], point.Coord(axis + 1));
                    candidate.expectedBounds[std::size_t(axis + 3)] = std::max(
                        candidate.expectedBounds[std::size_t(axis + 3)], point.Coord(axis + 1));
                }
            }
            if (!station) continue;
            if (cancelled.load(std::memory_order_relaxed)) return Admission::Cancelled;
            if (!counter.beginStage(budget::Site::C25AnalyticInput)
                || !counter.visit(1, budget::Site::C25AnalyticInput))
                return Admission::BudgetDenied;
            const auto lower = loft_correspondence::detail::LaneCorners(
                definition, station - 1, false);
            const auto upper = loft_correspondence::detail::LaneCorners(
                definition, station, false);
            detail::Polynomial aSquared, bSquared;
            const detail::Polynomial determinant = detail::DeterminantPolynomial(
                lower, upper, aSquared, bSquared);
            std::size_t spanSubdivisions = 0;
            if (!detail::ProvePositive(determinant, spanSubdivisions)
                || !detail::ProvePositive(detail::ClearancePolynomial(
                       determinant, aSquared, minimum), spanSubdivisions)
                || !detail::ProvePositive(detail::ClearancePolynomial(
                       determinant, bSquared, minimum), spanSubdivisions))
                return Admission::ContinuousClearance;
            candidate.intervalSubdivisions += spanSubdivisions;
            const double d0 = detail::Value(determinant, 0);
            const double dm = detail::Value(determinant, 0.5);
            const double d1 = detail::Value(determinant, 1);
            candidate.minimumSignedArea = std::min(candidate.minimumSignedArea,
                                                   std::min({d0, dm, d1}));
            const double height = definition.base.stations[station].z
                - definition.base.stations[station - 1].z;
            volume += height * (d0 + 4 * dm + d1) / 6;
        }
        candidate.expectedVolume = volume * scale * scale * scale;
        if (!std::isfinite(candidate.expectedVolume) || candidate.expectedVolume <= 0)
            return Admission::NumericFailure;
        output = std::move(candidate);
        return Admission::Accepted;
    } catch (...) { output = {}; return Admission::NumericFailure; }
}

inline Result Verify(const Definition& definition, const Inspection& inspection,
                     const TopoDS_Shape& candidate,
                     const std::atomic_bool& cancelled,
                     budget::Counter& counter) noexcept {
    Result output;
    if (cancelled.load(std::memory_order_relaxed)) {
        output.status = ProofStatus::Cancelled; return output;
    }
    if (candidate.IsNull() || candidate.ShapeType() != TopAbs_SOLID
        || !std::isfinite(inspection.expectedVolume) || inspection.expectedVolume <= 0)
        return output;
    try {
        OCC_CATCH_SIGNALS
        output.diagnosticStage = 1;
        retained_topology_budget::Census census;
        const auto censusStatus = retained_topology_budget::CensusTopology(
            candidate, counter, cancelled, census, budget::Site::C11GeometryCensus, true);
        if (censusStatus == retained_topology_budget::WalkStatus::Cancelled) {
            output.status = ProofStatus::Cancelled; return output;
        }
        if (censusStatus == retained_topology_budget::WalkStatus::BudgetDenied) {
            output.status = ProofStatus::BudgetDenied; return output;
        }
        if (censusStatus != retained_topology_budget::WalkStatus::Completed) {
            output.status = ProofStatus::KernelFailure; return output;
        }
        output.diagnosticStage = 2;
        for (int pass = 0; pass < 6; ++pass)
            if (!detail::Reserve(census, counter, budget::Site::C07CurveClassification)) {
                output.status = ProofStatus::BudgetDenied; return output;
            }
        TopTools_IndexedMapOfShape all;
        output.diagnosticStage = 3;
        TopExp::MapShapes(candidate, all);
        int solids = 0, shells = 0, vertices = 0;
        for (int index = 1; index <= all.Extent(); ++index) {
            if (all(index).ShapeType() == TopAbs_SOLID) ++solids;
            else if (all(index).ShapeType() == TopAbs_SHELL) ++shells;
            else if (all(index).ShapeType() == TopAbs_VERTEX) ++vertices;
        }
        const std::size_t stations = definition.base.stations.size();
        const int expectedFaces = int(4 * (stations - 1) + 2);
        const int expectedEdges = int(8 * stations - 4);
        const int expectedVertices = int(4 * stations);
        if (solids != 1 || shells != 1 || census.faces.Extent() != expectedFaces
            || census.edges.Extent() != expectedEdges || vertices != expectedVertices) {
            output.status = ProofStatus::WrongTopology; return output;
        }
        TopoDS_Solid solid = TopoDS::Solid(candidate);
        output.diagnosticStage = 41;
        if (solid.Orientation() != TopAbs_FORWARD) {
            output.status = ProofStatus::InvalidSolid; return output;
        }
        output.diagnosticStage = 42;
        if (!BRepCheck_Analyzer(solid, Standard_True).IsValid()) {
            output.status = ProofStatus::InvalidSolid; return output;
        }
        output.diagnosticStage = 5;
        const auto interference = planar_sweep::detail::CheckInterference(solid, cancelled);
        if (interference == planar_sweep::BuildStatus::Cancelled) {
            output.status = ProofStatus::Cancelled; return output;
        }
        if (interference != planar_sweep::BuildStatus::Built) {
            output.status = ProofStatus::SelfInterference; return output;
        }
        GProp_GProps properties;
        output.diagnosticStage = 6;
        BRepGProp::VolumeProperties(solid, properties);
        const double volume = properties.Mass();
        if (!std::isfinite(volume) || volume <= 0
            || std::abs(volume - inspection.expectedVolume)
               > inspection.expectedVolume * 1e-7) {
            output.status = ProofStatus::WrongVolume; return output;
        }
        Bnd_Box box;
        output.diagnosticStage = 7;
        BRepBndLib::AddOptimal(solid, box, Standard_False, Standard_False);
        if (box.IsVoid() || box.IsOpen()) {
            output.status = ProofStatus::WrongBounds; return output;
        }
        std::array<double, 6> bounds;
        box.Get(bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5]);
        for (std::size_t index = 0; index < 6; ++index)
            if (!std::isfinite(bounds[index])
                || std::abs(bounds[index] - inspection.expectedBounds[index])
                   > 0.001 / inspection.millimetersPerUnit) {
                output.status = ProofStatus::WrongBounds; return output;
            }

        const double tolerance = std::max(32 * Precision::Confusion(),
                                          1e-7 / inspection.millimetersPerUnit);
        if (!counter.visit(std::size_t(census.faces.Extent() + census.edges.Extent()),
                           budget::Site::C07CurveClassification)) {
            output.status = ProofStatus::BudgetDenied; return output;
        }
        std::set<int> semanticEdges;
        output.diagnosticStage = 8;
        for (std::size_t station = 0; station < stations; ++station) {
            const auto points = loft_correspondence::detail::LaneCorners(
                definition, station, true);
            for (std::size_t lane = 0; lane < 4; ++lane) {
                const int edge = detail::MatchEdge(census.edges, points[lane],
                                                   points[(lane + 1) % 4], tolerance);
                if (!edge || !semanticEdges.insert(edge).second) {
                    output.status = ProofStatus::WrongBoundary; return output;
                }
            }
            if (!station) continue;
            const auto lower = loft_correspondence::detail::LaneCorners(
                definition, station - 1, true);
            for (std::size_t lane = 0; lane < 4; ++lane) {
                const int edge = detail::MatchEdge(census.edges, lower[lane],
                                                   points[lane], tolerance);
                if (!edge || !semanticEdges.insert(edge).second) {
                    output.status = ProofStatus::WrongBoundary; return output;
                }
            }
        }
        if (semanticEdges.size() != std::size_t(expectedEdges)) {
            output.status = ProofStatus::WrongBoundary; return output;
        }

        std::set<int> semanticFaces;
        output.diagnosticStage = 9;
        std::size_t pcurveUses = 0;
        for (std::size_t station = 0; station + 1 < stations; ++station) {
            const auto lower = loft_correspondence::detail::LaneCorners(
                definition, station, true);
            const auto upper = loft_correspondence::detail::LaneCorners(
                definition, station + 1, true);
            for (std::size_t lane = 0; lane < 4; ++lane) {
                const std::size_t next = (lane + 1) % 4;
                const std::array<gp_Pnt, 4> expected = {
                    lower[lane], lower[next], upper[lane], upper[next]};
                int match = 0;
                for (int faceIndex = 1; faceIndex <= census.faces.Extent(); ++faceIndex) {
                    const TopoDS_Face face = TopoDS::Face(census.faces(faceIndex));
                    if (!detail::MatchSurface(face, expected, tolerance)) continue;
                    if (match) { output.status = ProofStatus::WrongBoundary; return output; }
                    match = faceIndex;
                }
                if (!match || !semanticFaces.insert(match).second) {
                    output.status = ProofStatus::WrongBoundary; return output;
                }
                if (!detail::ConsistentPCurves(TopoDS::Face(census.faces(match)),
                                               tolerance, pcurveUses)) {
                    output.status = ProofStatus::WrongPCurve; return output;
                }
            }
        }
        int capCount = 0;
        for (int faceIndex = 1; faceIndex <= census.faces.Extent(); ++faceIndex) {
            if (semanticFaces.count(faceIndex)) continue;
            TopLoc_Location location;
            if (Handle(Geom_Plane)::DownCast(BRep_Tool::Surface(
                    TopoDS::Face(census.faces(faceIndex)), location)).IsNull()) {
                output.status = ProofStatus::WrongBoundary; return output;
            }
            ++capCount;
            if (!detail::ConsistentPCurves(TopoDS::Face(census.faces(faceIndex)),
                                           tolerance, pcurveUses)) {
                output.status = ProofStatus::WrongPCurve; return output;
            }
        }
        if (capCount != 2 || pcurveUses != std::size_t(4 * expectedFaces)) {
            output.status = ProofStatus::WrongBoundary; return output;
        }

        TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
        output.diagnosticStage = 10;
        TopExp::MapShapesAndAncestors(solid, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
        if (edgeFaces.Extent() != expectedEdges) {
            output.status = ProofStatus::WrongTopology; return output;
        }
        for (int index = 1; index <= edgeFaces.Extent(); ++index) {
            const auto& uses = edgeFaces.FindFromIndex(index);
            if (uses.Extent() != 2) { output.status = ProofStatus::WrongTopology; return output; }
            auto iterator = uses.cbegin();
            const TopoDS_Face first = TopoDS::Face(*iterator++);
            const TopoDS_Face second = TopoDS::Face(*iterator);
            const TopoDS_Edge edge = TopoDS::Edge(edgeFaces.FindKey(index));
            const auto a = detail::Use(edge, first), b = detail::Use(edge, second);
            if (!((a == TopAbs_FORWARD && b == TopAbs_REVERSED)
                  || (a == TopAbs_REVERSED && b == TopAbs_FORWARD))) {
                output.status = ProofStatus::WrongTopology; return output;
            }
        }
        output.status = ProofStatus::Proven;
        output.solid = solid;
        output.volume = volume;
        output.bounds = bounds;
        output.faceCount = std::size_t(expectedFaces);
        output.edgeCount = std::size_t(expectedEdges);
        output.vertexCount = std::size_t(expectedVertices);
        output.diagnosticStage = 100;
        return output;
    } catch (...) {
        output.status = cancelled.load(std::memory_order_relaxed)
            ? ProofStatus::Cancelled : ProofStatus::KernelFailure;
        return output;
    }
}

inline Result ConstructAndProve(const Definition& definition,
                                const std::atomic_bool& cancelled,
                                budget::Counter& counter,
                                Inspection* inspected = nullptr) noexcept {
    Inspection inspection;
    const Admission admission = Inspect(definition, cancelled, counter, inspection);
    if (admission != Admission::Accepted) {
        Result result;
        result.status = admission == Admission::Cancelled ? ProofStatus::Cancelled
            : admission == Admission::BudgetDenied ? ProofStatus::BudgetDenied
            : ProofStatus::InvalidDefinition;
        return result;
    }
    if (inspected) *inspected = inspection;
    const BuildResult built = BuildUnproven(definition, cancelled, counter);
    if (built.status != BuildStatus::BuiltUnproven) {
        Result result;
        result.status = built.status == BuildStatus::Cancelled ? ProofStatus::Cancelled
            : built.status == BuildStatus::BudgetDenied ? ProofStatus::BudgetDenied
            : built.status == BuildStatus::InvalidDefinition ? ProofStatus::InvalidDefinition
            : ProofStatus::KernelFailure;
        return result;
    }
    return Verify(definition, inspection, built.solid, cancelled, counter);
}

} // namespace core3d::loft_correspondence::proof
