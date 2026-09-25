#pragma once

#include "VariableRadiusFilletDefinition.hxx"
#include <BOPAlgo_ArgumentAnalyzer.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Law_Function.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopoDS.hxx>
#include <atomic>
#include <cmath>
#include <locale>
#include <set>
#include <sstream>

namespace core3d::variable_radius_fillet {

struct SectionMeasurement final {
    UUID edge{};
    double parameter = 0;
    double expectedRadiusLocal = 0;
    double measuredRadiusLocal = 0;
};

struct BuildEvidence final {
    std::vector<UUID> consumedEdges;
    std::vector<SectionMeasurement> sections;
    double minimumClearanceLocal = 0;
    double removedVolumeLocal3 = 0;
};

struct BuildResult final {
    Refusal refusal = Refusal::KernelFailure;
    TopoDS_Shape solid;
    BuildEvidence evidence;
    bool built() const noexcept { return refusal == Refusal::None && !solid.IsNull(); }
};

namespace detail {
inline gp_Pnt point(const std::array<double, 3>& value) {
    return {value[0], value[1], value[2]};
}
inline gp_Dir direction(const std::array<double, 3>& value) {
    return {value[0], value[1], value[2]};
}
inline bool samePoint(const gp_Pnt& a, const gp_Pnt& b, double tolerance) noexcept {
    return a.Distance(b) <= tolerance;
}
inline bool onLineSegment(const gp_Pnt& pointValue, const gp_Pnt& start,
                          const gp_Pnt& end, double tolerance) noexcept {
    const gp_Vec segment(start, end), offset(start, pointValue);
    const double length = segment.Magnitude();
    if (length <= tolerance) return false;
    const double along = offset.Dot(segment) / (length * length);
    return along > 0 && along < 1
        && offset.Crossed(segment).Magnitude() / length <= tolerance;
}

inline bool matches(const TopoDS_Edge& edge, const OrientedEdgeAnchor& anchor,
                    double tolerance, TopoDS_Edge& oriented, Refusal& refusal) {
    if (edge.IsNull() || BRep_Tool::Degenerated(edge)) return false;
    BRepAdaptor_Curve curve(edge);
    if (curve.GetType() != GeomAbs_Line || BRep_Tool::IsClosed(edge)) return false;
    TopoDS_Vertex first, last; TopExp::Vertices(edge, first, last, Standard_True);
    if (first.IsNull() || last.IsNull()) return false;
    const gp_Pnt a = BRep_Tool::Pnt(first), b = BRep_Tool::Pnt(last);
    const gp_Pnt expectedStart = point(anchor.startLocal), expectedEnd = point(anchor.endLocal);
    if (!onLineSegment(point(anchor.pointLocal), a, b, tolerance)) return false;
    const bool aligned = samePoint(a, expectedStart, tolerance)
        && samePoint(b, expectedEnd, tolerance);
    const bool reversed = samePoint(a, expectedEnd, tolerance)
        && samePoint(b, expectedStart, tolerance);
    if (!aligned && !reversed) { refusal = Refusal::OrientationDrift; return false; }
    oriented = edge;
    if (reversed) oriented.Reverse();
    BRepAdaptor_Curve check(oriented);
    const gp_Vec actual(check.Value(check.FirstParameter()), check.Value(check.LastParameter()));
    if (!gp_Dir(actual).IsParallel(direction(anchor.tangent), 1e-10)) {
        refusal = Refusal::OrientationDrift; return false;
    }
    return true;
}

inline bool resolve(const TopoDS_Shape& shape, const OrientedEdgeAnchor& anchor,
                    double tolerance, TopoDS_Edge& result, Refusal& refusal) {
    result.Nullify();
    TopTools_IndexedMapOfShape edges; TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    if (edges.Extent() > 4096) { refusal = Refusal::Budget; return false; }
    unsigned matchesFound = 0;
    for (int index = 1; index <= edges.Extent(); ++index) {
        TopoDS_Edge oriented;
        Refusal candidateRefusal = Refusal::None;
        if (!matches(TopoDS::Edge(edges(index)), anchor, tolerance, oriented, candidateRefusal)) {
            if (candidateRefusal == Refusal::OrientationDrift) refusal = candidateRefusal;
            continue;
        }
        result = oriented;
        if (++matchesFound > 1) { result.Nullify(); refusal = Refusal::AnchorAmbiguous; return false; }
    }
    if (matchesFound != 1) {
        if (refusal != Refusal::OrientationDrift) refusal = Refusal::AnchorMissing;
        return false;
    }
    refusal = Refusal::None; return true;
}

inline bool rectangularFaceClearance(const TopoDS_Face& face, const TopoDS_Edge& selected,
                                     double tolerance, double& clearance) {
    BRepAdaptor_Surface surface(face);
    if (surface.GetType() != GeomAbs_Plane) return false;
    TopTools_IndexedMapOfShape edges; TopExp::MapShapes(face, TopAbs_EDGE, edges);
    if (edges.Extent() != 4) return false;
    BRepAdaptor_Curve selectedCurve(selected);
    if (selectedCurve.GetType() != GeomAbs_Line) return false;
    unsigned oppositeCount = 0; double candidate = 0;
    for (int index = 1; index <= edges.Extent(); ++index) {
        const TopoDS_Edge edge = TopoDS::Edge(edges(index));
        if (edge.IsSame(selected)) continue;
        BRepAdaptor_Curve curve(edge);
        if (curve.GetType() != GeomAbs_Line) return false;
        if (!curve.Line().Direction().IsParallel(selectedCurve.Line().Direction(), 1e-10)) continue;
        BRepExtrema_DistShapeShape distance(selected, edge); distance.Perform();
        if (!distance.IsDone() || distance.NbSolution() == 0
            || !std::isfinite(distance.Value()) || distance.Value() <= tolerance) return false;
        candidate = distance.Value(); ++oppositeCount;
    }
    if (oppositeCount != 1) return false;
    clearance = candidate; return true;
}

inline bool continuousClearance(const TopoDS_Shape& shape, const TopoDS_Edge& edge,
                                double tolerance, double& clearance) {
    clearance = INFINITY;
    TopTools_IndexedDataMapOfShapeListOfShape owners;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, owners);
    if (!owners.Contains(edge)) return false;
    TopTools_IndexedMapOfShape uniqueFaces;
    for (TopTools_ListIteratorOfListOfShape iterator(owners.FindFromKey(edge));
         iterator.More(); iterator.Next()) uniqueFaces.Add(iterator.Value());
    if (uniqueFaces.Extent() != 2) return false;
    for (int index = 1; index <= uniqueFaces.Extent(); ++index) {
        double faceClearance = 0;
        if (!rectangularFaceClearance(TopoDS::Face(uniqueFaces(index)), edge,
                                      tolerance, faceClearance)) return false;
        clearance = std::min(clearance, faceClearance);
    }
    return std::isfinite(clearance) && clearance > tolerance;
}

inline bool measuredSection(const TopoDS_Shape& shape, const OrientedEdgeAnchor& anchor,
                            double parameter, double expectedRadius, double tolerance,
                            double& measured) {
    measured = 0;
    const gp_Pnt start = point(anchor.startLocal), end = point(anchor.endLocal);
    const gp_Pnt pin(start.X() + (end.X() - start.X()) * parameter,
                     start.Y() + (end.Y() - start.Y()) * parameter,
                     start.Z() + (end.Z() - start.Z()) * parameter);
    BRepAlgoAPI_Section section(shape, gp_Pln(pin, direction(anchor.tangent)), Standard_False);
    section.Approximation(Standard_False); section.Build();
    if (!section.IsDone()) return false;
    TopTools_IndexedMapOfShape edges; TopExp::MapShapes(section.Shape(), TopAbs_EDGE, edges);
    unsigned matchesFound = 0;
    for (int index = 1; index <= edges.Extent(); ++index) {
        BRepAdaptor_Curve curve(TopoDS::Edge(edges(index)));
        double radius = 0;
        if (curve.GetType() == GeomAbs_Circle) {
            radius = curve.Circle().Radius();
        } else {
            const double first = curve.FirstParameter(), last = curve.LastParameter();
            if (!std::isfinite(first) || !std::isfinite(last) || first >= last) continue;
            const std::array<gp_Pnt, 5> points{{
                curve.Value(first), curve.Value(first + (last - first) * .25),
                curve.Value(first + (last - first) * .5),
                curve.Value(first + (last - first) * .75), curve.Value(last)}};
            const auto circumradius = [&](const gp_Pnt& a, const gp_Pnt& b,
                                          const gp_Pnt& c, double& value) {
                const gp_Vec ab(a, b), ac(a, c); const double cross = ab.Crossed(ac).Magnitude();
                if (!std::isfinite(cross) || cross <= tolerance * tolerance) return false;
                value = a.Distance(b) * b.Distance(c) * c.Distance(a) / (2 * cross);
                return std::isfinite(value) && value > tolerance;
            };
            double left = 0, middle = 0, right = 0;
            if (!circumradius(points[0], points[1], points[2], left)
                || !circumradius(points[1], points[2], points[3], middle)
                || !circumradius(points[2], points[3], points[4], right)
                || std::abs(left - middle) > std::max(tolerance * 2, middle * .002)
                || std::abs(right - middle) > std::max(tolerance * 2, middle * .002)) continue;
            radius = (left + middle + right) / 3;
        }
        // A variable-radius rolling-ball blend is generally not a circle of the
        // authored law radius in a plane normal to the original edge. Keep the
        // independently measured section as evidence and bound it against the
        // authored value; the kernel law itself is checked separately below.
        if (radius < expectedRadius * .5 || radius > expectedRadius * 2) continue;
        measured = radius; ++matchesFound;
    }
    return matchesFound == 1;
}

inline bool selfIntersectionFree(const TopoDS_Shape& shape) {
    BOPAlgo_ArgumentAnalyzer analyzer;
    analyzer.SetShape1(shape); analyzer.SetRunParallel(Standard_False);
    analyzer.StopOnFirstFaulty() = Standard_True;
    analyzer.ArgumentTypeMode() = Standard_False; analyzer.SelfInterMode() = Standard_True;
    analyzer.SmallEdgeMode() = Standard_False; analyzer.RebuildFaceMode() = Standard_False;
    analyzer.TangentMode() = Standard_False; analyzer.MergeVertexMode() = Standard_False;
    analyzer.MergeEdgeMode() = Standard_False; analyzer.ContinuityMode() = Standard_False;
    analyzer.CurveOnSurfaceMode() = Standard_False; analyzer.Perform();
    return !analyzer.HasErrors() && !analyzer.HasWarnings() && !analyzer.HasFaulty();
}

inline bool exactShapeBytes(const TopoDS_Shape& shape, std::vector<std::uint8_t>& bytes) {
    bytes.clear(); std::ostringstream stream; stream.imbue(std::locale::classic());
    BRepTools::Write(shape, stream, Standard_False, Standard_False,
                     TopTools_FormatVersion_VERSION_3);
    if (!stream.good()) return false;
    const std::string value = stream.str();
    bytes.assign(value.begin(), value.end()); return !bytes.empty();
}
} // namespace detail

inline BuildResult Build(const TopoDS_Shape& source, const Definition& definition,
                         const std::atomic_bool& cancelled) noexcept {
    BuildResult output;
    const auto decline = [&](Refusal refusal) {
        BuildResult result; result.refusal = cancelled.load() ? Refusal::Cancelled : refusal; return result;
    };
    try {
        if (cancelled.load()) return decline(Refusal::Cancelled);
        Refusal refusal = Refusal::KernelFailure;
        if (source.IsNull()) return decline(Refusal::KernelFailure);
        if (!Validate(definition, refusal)) return decline(refusal);
        BRepBuilderAPI_Copy detached(source, Standard_True, Standard_False);
        if (!detached.IsDone()) return decline(Refusal::KernelFailure);
        const TopoDS_Shape input = detached.Shape();
        const double tolerance = std::max(Precision::Confusion() * 32,
            1e-7 / definition.metersPerLocalUnit);
        std::vector<TopoDS_Edge> edges; edges.reserve(definition.edges.size());
        double minimumClearance = INFINITY;
        for (const auto& anchor : definition.edges) {
            if (cancelled.load()) return decline(Refusal::Cancelled);
            TopoDS_Edge edge;
            if (!detail::resolve(input, anchor, tolerance, edge, refusal)) return decline(refusal);
            if (BRep_Tool::IsClosed(edge)) return decline(Refusal::ClosedLoop);
            double clearance = 0;
            if (!detail::continuousClearance(input, edge, tolerance, clearance))
                return decline(Refusal::Clearance);
            minimumClearance = std::min(minimumClearance, clearance);
            edges.push_back(edge); output.evidence.consumedEdges.push_back(anchor.identifier);
        }
        const double maximumRadius = std::max(definition.stations[0].radiusLocal,
                                               definition.stations[1].radiusLocal);
        if (!std::isfinite(minimumClearance) || maximumRadius >= minimumClearance / 2)
            return decline(Refusal::Clearance);

        BRepFilletAPI_MakeFillet fillet(input);
        std::set<int> contours;
        for (const auto& edge : edges) {
            fillet.Add(definition.stations[0].radiusLocal,
                       definition.stations[1].radiusLocal, edge);
            const int contour = fillet.Contour(edge);
            if (contour <= 0 || fillet.NbEdges(contour) != 1 || !contours.insert(contour).second)
                return decline(Refusal::ContourExpansion);
        }
        if (fillet.NbContours() != int(edges.size())) return decline(Refusal::ContourExpansion);
        fillet.Build();
        if (!fillet.IsDone() || fillet.Shape().IsNull()) return decline(Refusal::KernelFailure);
        for (const auto& edge : edges) {
            const int contour = fillet.Contour(edge); Standard_Real first = 0, last = 0;
            const Handle(Law_Function) law = fillet.GetLaw(contour, edge);
            if (law.IsNull() || !fillet.GetBounds(contour, edge, first, last)
                || std::abs(law->Value(first) - definition.stations[0].radiusLocal) > tolerance
                || std::abs(law->Value(last) - definition.stations[1].radiusLocal) > tolerance)
                return decline(Refusal::EndpointMismatch);
        }
        const TopoDS_Shape candidate = fillet.Shape();
        if (!BRepCheck_Analyzer(candidate).IsValid()) return decline(Refusal::KernelFailure);
        if (!detail::selfIntersectionFree(candidate)) return decline(Refusal::SelfIntersection);

        GProp_GProps before, after; BRepGProp::VolumeProperties(input, before);
        BRepGProp::VolumeProperties(candidate, after);
        const double removed = before.Mass() - after.Mass();
        if (!std::isfinite(removed) || removed <= std::max(1e-12, before.Mass() * 1e-10))
            return decline(Refusal::NonRemoving);

        constexpr std::array<double, 3> pins{{0.2, 0.5, 0.8}};
        for (std::size_t edgeIndex = 0; edgeIndex < definition.edges.size(); ++edgeIndex) {
            double previousMeasured = 0;
            for (double pin : pins) {
                double expected = 0, measured = 0;
                if (!RadiusAt(definition, pin, expected)
                    || !detail::measuredSection(candidate, definition.edges[edgeIndex], pin,
                                                expected, tolerance, measured))
                    return decline(Refusal::SectionMismatch);
                output.evidence.sections.push_back({definition.edges[edgeIndex].identifier,
                                                    pin, expected, measured});
                if (previousMeasured > 0) {
                    const double delta = definition.stations[1].radiusLocal
                        - definition.stations[0].radiusLocal;
                    if ((delta > 0 && measured <= previousMeasured)
                        || (delta < 0 && measured >= previousMeasured)
                        || (delta == 0 && std::abs(measured - previousMeasured) > tolerance))
                        return decline(Refusal::SectionMismatch);
                }
                previousMeasured = measured;
            }
        }
        output.refusal = Refusal::None; output.solid = candidate;
        output.evidence.minimumClearanceLocal = minimumClearance;
        output.evidence.removedVolumeLocal3 = removed;
        return output;
    } catch (...) { return decline(Refusal::KernelFailure); }
}

inline BuildResult BuildDeterministically(const TopoDS_Shape& source,
                                          const Definition& definition,
                                          const std::atomic_bool& cancelled) noexcept {
    BuildResult first = Build(source, definition, cancelled);
    if (!first.built()) return first;
    BuildResult second = Build(source, definition, cancelled);
    if (!second.built()) return second;
    std::vector<std::uint8_t> a, b;
    if (!detail::exactShapeBytes(first.solid, a) || !detail::exactShapeBytes(second.solid, b)
        || a != b || first.evidence.consumedEdges != second.evidence.consumedEdges
        || first.evidence.sections.size() != second.evidence.sections.size()) {
        BuildResult mismatch; mismatch.refusal = Refusal::ReplayMismatch; return mismatch;
    }
    for (std::size_t index = 0; index < first.evidence.sections.size(); ++index) {
        const auto& lhs = first.evidence.sections[index];
        const auto& rhs = second.evidence.sections[index];
        if (lhs.edge != rhs.edge || lhs.parameter != rhs.parameter
            || lhs.expectedRadiusLocal != rhs.expectedRadiusLocal
            || lhs.measuredRadiusLocal != rhs.measuredRadiusLocal) {
            BuildResult mismatch; mismatch.refusal = Refusal::ReplayMismatch; return mismatch;
        }
    }
    return first;
}

} // namespace core3d::variable_radius_fillet
