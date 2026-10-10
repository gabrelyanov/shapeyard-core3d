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
#include <Law_BSpFunc.hxx>
#include <Law_BSpline.hxx>
#include <Law_Composite.hxx>
#include <Law_Constant.hxx>
#include <Law_Function.hxx>
#include <Law_Linear.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
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
#include <string>

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

#if DEBUG
// Test observation only: the native bridge uses this to prove orientation
// refusals happen before any edge is handed to the OCCT fillet builder and to
// inspect the actual post-Build law without changing production decisions.
struct DebugLawSpanObservation final {
    std::size_t piece = 0;
    double first = 0;
    double last = 0;
    double derivativeFirst = 0;
    double derivativeLast = 0;
    double extremumParameter = 0;
    double extremumDerivative = 0;
    double minimumDerivative = 0;
    double maximumDerivative = 0;
    double authoredMinimumDerivative = 0;
    double authoredMaximumDerivative = 0;
};

struct DebugLawJoinObservation final {
    double parameter = 0;
    double valueLeft = 0;
    double valueRight = 0;
    double derivativeLeft = 0;
    double derivativeRight = 0;
};

struct DebugLawPieceObservation final {
    std::string type;
    double first = 0;
    double last = 0;
    int continuity = -1;
    int degree = 0;
    bool rational = false;
    std::vector<double> c1Intervals;
    std::vector<double> knots;
    std::vector<int> multiplicities;
    std::vector<double> poles;
};

struct DebugRealizedLawObservation final {
    bool attempted = false;
    bool exception = false;
    bool supported = false;
    bool c1OnPhysicalInterval = false;
    bool nonnegativeDerivative = false;
    bool noConstantSpan = false;
    double physicalFirst = 0;
    double physicalLast = 0;
    double boundsFirst = 0;
    double boundsLast = 0;
    double authoredFirst = 0;
    double authoredLast = 0;
    std::array<double, 2> stationParameters{};
    std::array<double, 2> stationRadii{};
    std::array<double, 2> nativeStationParameters{};
    std::array<double, 2> nativeStationRadii{};
    std::array<double, 5> sampleParameters{};
    std::array<double, 5> samplePositionsZ{};
    std::array<double, 5> sampleValues{};
    std::array<double, 5> sampleDerivatives{};
    std::array<double, 99> gridValues{};
    std::array<double, 99> gridDerivatives{};
    std::vector<DebugLawPieceObservation> pieces;
    std::vector<DebugLawSpanObservation> spans;
    std::vector<DebugLawJoinObservation> joins;
};

struct DebugStationObservation final {
    UUID identifier{};
    double authoredParameter = 0;
    double nativeNormalizedParameter = 0;
    double nativeParameter = 0;
    double radius = 0;
    double physicalZ = 0;
    double realizedRadius = 0;
};

struct DebugMultiStationLawObservation final {
    bool attempted = false;
    bool exception = false;
    double physicalFirst = 0;
    double physicalLast = 0;
    double authoredFirst = 0;
    double authoredLast = 0;
    std::vector<DebugStationObservation> stations;
};

struct DebugBuildObservation final {
    std::size_t filletEntryCount = 0;
    std::vector<DebugRealizedLawObservation> realizedLaws;
    std::vector<DebugMultiStationLawObservation> multiStationLaws;
};
inline thread_local DebugBuildObservation* debugBuildObservation = nullptr;
#endif

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

struct AuthoredParameterMap final {
    double nativeFirst = 0;
    double nativeLast = 0;
    double authoredFirst = 0;
    double authoredLast = 0;

    double parameter(double authored) const noexcept {
        return authoredFirst + (authoredLast - authoredFirst) * authored;
    }
    double normalized(double authored) const noexcept {
        return (parameter(authored) - nativeFirst) / (nativeLast - nativeFirst);
    }
};

inline bool authoredParameterMap(const TopoDS_Edge& edge,
                                 const OrientedEdgeAnchor& anchor,
                                 double tolerance,
                                 AuthoredParameterMap& output) noexcept {
    try {
        BRepAdaptor_Curve curve(edge);
        if (curve.GetType() != GeomAbs_Line) return false;
        const double first = curve.FirstParameter(), last = curve.LastParameter();
        if (!std::isfinite(first) || !std::isfinite(last) || !(last > first)) return false;
        const gp_Pnt firstPoint = curve.Value(first), lastPoint = curve.Value(last);
        const gp_Pnt authoredStart = point(anchor.startLocal);
        const gp_Pnt authoredEnd = point(anchor.endLocal);
        const bool startAtFirst = samePoint(firstPoint, authoredStart, tolerance)
            && samePoint(lastPoint, authoredEnd, tolerance);
        const bool startAtLast = samePoint(lastPoint, authoredStart, tolerance)
            && samePoint(firstPoint, authoredEnd, tolerance);
        if (startAtFirst == startAtLast) return false;
        output.nativeFirst = first;
        output.nativeLast = last;
        output.authoredFirst = startAtFirst ? first : last;
        output.authoredLast = startAtFirst ? last : first;
        const double middle = output.parameter(.5);
        const gp_Pnt expectedMiddle(
            (authoredStart.X() + authoredEnd.X()) * .5,
            (authoredStart.Y() + authoredEnd.Y()) * .5,
            (authoredStart.Z() + authoredEnd.Z()) * .5);
        return samePoint(curve.Value(middle), expectedMiddle, tolerance);
    } catch (...) {
        return false;
    }
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

inline bool outwardNormal(const TopoDS_Face& face, gp_Dir& normal) {
    BRepAdaptor_Surface surface(face);
    if (surface.GetType() != GeomAbs_Plane) return false;
    normal = surface.Plane().Axis().Direction();
    if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
    else if (face.Orientation() != TopAbs_FORWARD) return false;
    return true;
}

inline bool signedNormalCorrespondence(const std::array<gp_Dir, 2>& supportNormals,
                                       const OrientedEdgeAnchor& anchor,
                                       bool reverseEnumeration = false) {
    const gp_Dir payloadA = direction(anchor.normalA);
    const gp_Dir payloadB = direction(anchor.normalB);
    const gp_Dir& first = supportNormals[reverseEnumeration ? 1 : 0];
    const gp_Dir& second = supportNormals[reverseEnumeration ? 0 : 1];
    unsigned assignments = 0;
    if (first.IsEqual(payloadA, 1e-10) && second.IsEqual(payloadB, 1e-10))
        ++assignments;
    if (first.IsEqual(payloadB, 1e-10) && second.IsEqual(payloadA, 1e-10))
        ++assignments;
    return assignments == 1;
}

inline bool continuousClearance(const TopoDS_Shape& shape, const TopoDS_Edge& edge,
                                const OrientedEdgeAnchor& anchor, double tolerance,
                                double& clearance, Refusal& refusal,
                                bool reverseEnumeration = false) {
    clearance = INFINITY;
    TopTools_IndexedDataMapOfShapeListOfShape owners;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, owners);
    if (!owners.Contains(edge)) { refusal = Refusal::Clearance; return false; }
    TopTools_IndexedMapOfShape uniqueFaces;
    for (TopTools_ListIteratorOfListOfShape iterator(owners.FindFromKey(edge));
         iterator.More(); iterator.Next()) uniqueFaces.Add(iterator.Value());
    if (uniqueFaces.Extent() != 2) { refusal = Refusal::Clearance; return false; }
    std::array<gp_Dir, 2> supportNormals;
    for (int index = 1; index <= uniqueFaces.Extent(); ++index) {
        double faceClearance = 0;
        const TopoDS_Face face = TopoDS::Face(uniqueFaces(index));
        if (!rectangularFaceClearance(face, edge, tolerance, faceClearance)
            || !outwardNormal(face, supportNormals[std::size_t(index - 1)])) {
            refusal = Refusal::Clearance; return false;
        }
        clearance = std::min(clearance, faceClearance);
    }
    if (!std::isfinite(clearance) || clearance <= tolerance) {
        refusal = Refusal::Clearance; return false;
    }
    if (!signedNormalCorrespondence(supportNormals, anchor, reverseEnumeration)) {
        refusal = Refusal::OrientationDrift; return false;
    }
    refusal = Refusal::None; return true;
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

#if DEBUG
inline bool finiteLawSample(double value, double derivative) noexcept {
    return std::isfinite(value) && std::isfinite(derivative);
}

inline void appendClippedC1Intervals(const Handle(Law_Function)& law,
                                     double first, double last,
                                     DebugLawPieceObservation& output) {
    const int count = law->NbIntervals(GeomAbs_C1);
    if (count <= 0 || count > 128) return;
    TColStd_Array1OfReal intervals(1, count + 1);
    law->Intervals(intervals, GeomAbs_C1);
    for (int index = intervals.Lower(); index <= intervals.Upper(); ++index) {
        const double value = std::max(first, std::min(last, intervals(index)));
        if (output.c1Intervals.empty()
            || std::abs(value - output.c1Intervals.back()) > Precision::PConfusion())
            output.c1Intervals.push_back(value);
    }
}

inline void observeRealizedLaw(const Handle(Law_Function)& law,
                               const TopoDS_Edge& edge,
                               const Definition& definition,
                               const AuthoredParameterMap& mapping,
                               const std::array<double, 2>& nativeRadii,
                               double boundsFirst, double boundsLast,
                               DebugRealizedLawObservation& output) noexcept {
    output = {};
    output.attempted = true;
    try {
        if (law.IsNull()) return;
        BRepAdaptor_Curve curve(edge);
        const double physicalFirst = curve.FirstParameter();
        const double physicalLast = curve.LastParameter();
        if (!std::isfinite(physicalFirst) || !std::isfinite(physicalLast)
            || !(physicalLast > physicalFirst)) return;
        output.physicalFirst = physicalFirst;
        output.physicalLast = physicalLast;
        output.boundsFirst = boundsFirst;
        output.boundsLast = boundsLast;
        output.authoredFirst = mapping.authoredFirst;
        output.authoredLast = mapping.authoredLast;
        output.stationParameters = {{definition.stations[0].parameter,
                                     definition.stations[1].parameter}};
        output.stationRadii = {{definition.stations[0].radiusLocal,
                                definition.stations[1].radiusLocal}};
        output.nativeStationParameters = {{mapping.nativeFirst, mapping.nativeLast}};
        output.nativeStationRadii = nativeRadii;

        constexpr std::array<double, 5> samples{{0, 0.2, 0.5, 0.8, 1}};
        for (std::size_t index = 0; index < samples.size(); ++index) {
            const double parameter = mapping.parameter(samples[index]);
            double value = 0, derivative = 0;
            law->D1(parameter, value, derivative);
            output.sampleParameters[index] = parameter;
            output.samplePositionsZ[index] = curve.Value(parameter).Z();
            output.sampleValues[index] = value;
            output.sampleDerivatives[index] = derivative
                * (mapping.authoredLast - mapping.authoredFirst);
        }
        for (std::size_t index = 0; index < output.gridValues.size(); ++index) {
            const double t = double(index + 1) / 100;
            const double parameter = mapping.parameter(t);
            double derivative = 0;
            law->D1(parameter, output.gridValues[index], derivative);
            output.gridDerivatives[index] = derivative
                * (mapping.authoredLast - mapping.authoredFirst);
        }

        struct Part final {
            Handle(Law_Function) law;
            double first = 0;
            double last = 0;
        };
        std::vector<Part> parts;
        const Handle(Law_Composite) composite = Handle(Law_Composite)::DownCast(law);
        if (composite.IsNull()) {
            double first = 0, last = 0;
            law->Bounds(first, last);
            parts.push_back({law, first, last});
        } else {
            const Law_Laws& laws = composite->ChangeLaws();
            for (Law_ListIteratorOfLaws iterator(laws); iterator.More(); iterator.Next()) {
                double first = 0, last = 0;
                const Handle(Law_Function) part = iterator.Value();
                if (part.IsNull()) continue;
                part->Bounds(first, last);
                parts.push_back({part, first, last});
            }
        }
        std::sort(parts.begin(), parts.end(), [](const Part& lhs, const Part& rhs) {
            return lhs.first < rhs.first;
        });

        const double parameterTolerance = std::max(
            Precision::PConfusion(), (physicalLast - physicalFirst) * 1e-12);
        const double valueTolerance = 1e-8 * (0.001 / definition.metersPerLocalUnit);
        const double derivativeTolerance = std::max(
            1e-12, valueTolerance / (physicalLast - physicalFirst) * 8);
        bool supported = true;
        bool c1 = true;
        bool nonnegative = true;
        bool noConstant = true;
        double coveredUntil = physicalFirst;
        std::vector<Part> relevant;

        for (const Part& part : parts) {
            const double first = std::max(physicalFirst, part.first);
            const double last = std::min(physicalLast, part.last);
            if (!(last - first > parameterTolerance)) continue;
            if (first > coveredUntil + parameterTolerance) c1 = false;
            coveredUntil = std::max(coveredUntil, last);
            relevant.push_back(part);

            DebugLawPieceObservation piece;
            piece.type = part.law->DynamicType()->Name();
            piece.first = first;
            piece.last = last;
            piece.continuity = int(part.law->Continuity());
            appendClippedC1Intervals(part.law, first, last, piece);
            const std::size_t pieceIndex = output.pieces.size();

            const Handle(Law_BSpFunc) spline = Handle(Law_BSpFunc)::DownCast(part.law);
            const Handle(Law_Linear) linear = Handle(Law_Linear)::DownCast(part.law);
            const Handle(Law_Constant) constant = Handle(Law_Constant)::DownCast(part.law);
            if (!spline.IsNull()) {
                const Handle(Law_BSpline) basis = spline->Curve();
                if (basis.IsNull()) {
                    supported = false;
                } else {
                    piece.degree = basis->Degree();
                    piece.rational = basis->IsRational();
                    TColStd_Array1OfReal knots(1, basis->NbKnots());
                    TColStd_Array1OfInteger multiplicities(1, basis->NbKnots());
                    TColStd_Array1OfReal poles(1, basis->NbPoles());
                    basis->Knots(knots);
                    basis->Multiplicities(multiplicities);
                    basis->Poles(poles);
                    for (int index = 1; index <= basis->NbKnots(); ++index) {
                        piece.knots.push_back(knots(index));
                        piece.multiplicities.push_back(multiplicities(index));
                    }
                    for (int index = 1; index <= basis->NbPoles(); ++index)
                        piece.poles.push_back(poles(index));
                    if (piece.rational || basis->IsPeriodic()
                        || piece.degree < 1 || piece.degree > 3) {
                        supported = false;
                    } else {
                        std::size_t spanCount = 0;
                        for (int knot = 1; knot < basis->NbKnots(); ++knot) {
                            const double spanFirst = std::max(first, knots(knot));
                            const double spanLast = std::min(last, knots(knot + 1));
                            if (!(spanLast - spanFirst > parameterTolerance)) continue;
                            ++spanCount;
                            double valueAtFirst = 0, derivativeAtFirst = 0, secondAtFirst = 0;
                            double valueAtLast = 0, derivativeAtLast = 0, secondAtLast = 0;
                            basis->LocalD2(spanFirst, knot, knot + 1, valueAtFirst,
                                           derivativeAtFirst, secondAtFirst);
                            basis->LocalD2(spanLast, knot, knot + 1, valueAtLast,
                                           derivativeAtLast, secondAtLast);
                            DebugLawSpanObservation span;
                            span.piece = pieceIndex;
                            span.first = spanFirst;
                            span.last = spanLast;
                            span.derivativeFirst = derivativeAtFirst;
                            span.derivativeLast = derivativeAtLast;
                            span.extremumParameter = spanFirst;
                            span.extremumDerivative = derivativeAtFirst;
                            span.minimumDerivative = std::min(derivativeAtFirst,
                                                              derivativeAtLast);
                            span.maximumDerivative = std::max(derivativeAtFirst,
                                                              derivativeAtLast);
                            if (piece.degree == 3
                                && std::isfinite(secondAtFirst)
                                && std::isfinite(secondAtLast)
                                && std::abs(secondAtFirst - secondAtLast) > 1e-18) {
                                const double extremum = spanFirst
                                    + (spanLast - spanFirst) * secondAtFirst
                                        / (secondAtFirst - secondAtLast);
                                if (extremum > spanFirst && extremum < spanLast) {
                                    double extremumValue = 0, extremumDerivative = 0;
                                    basis->LocalD1(extremum, knot, knot + 1,
                                                   extremumValue, extremumDerivative);
                                    span.extremumParameter = extremum;
                                    span.extremumDerivative = extremumDerivative;
                                    span.minimumDerivative = std::min(
                                        span.minimumDerivative, extremumDerivative);
                                    span.maximumDerivative = std::max(
                                        span.maximumDerivative, extremumDerivative);
                                }
                            }
                            if (!finiteLawSample(valueAtFirst, derivativeAtFirst)
                                || !finiteLawSample(valueAtLast, derivativeAtLast)
                                || !std::isfinite(span.minimumDerivative)
                                || !std::isfinite(span.maximumDerivative)) {
                                supported = false;
                            } else {
                                if (span.minimumDerivative < -derivativeTolerance)
                                    nonnegative = false;
                                if (span.maximumDerivative <= derivativeTolerance)
                                    noConstant = false;
                            }
                            const double authoredScale =
                                mapping.authoredLast - mapping.authoredFirst;
                            span.authoredMinimumDerivative = std::min(
                                span.minimumDerivative * authoredScale,
                                span.maximumDerivative * authoredScale);
                            span.authoredMaximumDerivative = std::max(
                                span.minimumDerivative * authoredScale,
                                span.maximumDerivative * authoredScale);
                            output.spans.push_back(span);
                        }
                        if (spanCount == 0) supported = false;

                        for (int knot = 2; knot < basis->NbKnots(); ++knot) {
                            const double parameter = knots(knot);
                            if (!(parameter > first + parameterTolerance
                                  && parameter < last - parameterTolerance)) continue;
                            DebugLawJoinObservation join;
                            join.parameter = parameter;
                            basis->LocalD1(parameter, knot - 1, knot,
                                           join.valueLeft, join.derivativeLeft);
                            basis->LocalD1(parameter, knot, knot + 1,
                                           join.valueRight, join.derivativeRight);
                            if (!finiteLawSample(join.valueLeft, join.derivativeLeft)
                                || !finiteLawSample(join.valueRight, join.derivativeRight)
                                || std::abs(join.valueLeft - join.valueRight) > valueTolerance
                                || std::abs(join.derivativeLeft - join.derivativeRight)
                                    > derivativeTolerance)
                                c1 = false;
                            output.joins.push_back(join);
                        }
                    }
                }
            } else if (!linear.IsNull() || !constant.IsNull()) {
                double valueFirst = 0, derivativeFirst = 0;
                double valueLast = 0, derivativeLast = 0;
                part.law->D1(first, valueFirst, derivativeFirst);
                part.law->D1(last, valueLast, derivativeLast);
                DebugLawSpanObservation span;
                span.piece = pieceIndex;
                span.first = first;
                span.last = last;
                span.derivativeFirst = derivativeFirst;
                span.derivativeLast = derivativeLast;
                span.extremumParameter = first;
                span.extremumDerivative = derivativeFirst;
                span.minimumDerivative = std::min(derivativeFirst, derivativeLast);
                span.maximumDerivative = std::max(derivativeFirst, derivativeLast);
                const double authoredScale = mapping.authoredLast - mapping.authoredFirst;
                span.authoredMinimumDerivative = std::min(
                    derivativeFirst * authoredScale, derivativeLast * authoredScale);
                span.authoredMaximumDerivative = std::max(
                    derivativeFirst * authoredScale, derivativeLast * authoredScale);
                output.spans.push_back(span);
                if (!finiteLawSample(valueFirst, derivativeFirst)
                    || !finiteLawSample(valueLast, derivativeLast)) supported = false;
                if (span.minimumDerivative < -derivativeTolerance) nonnegative = false;
                if (!constant.IsNull() || span.maximumDerivative <= derivativeTolerance)
                    noConstant = false;
            } else {
                supported = false;
            }
            output.pieces.push_back(std::move(piece));
        }

        if (coveredUntil < physicalLast - parameterTolerance || relevant.empty()) c1 = false;
        for (std::size_t index = 1; index < relevant.size(); ++index) {
            const double joinParameter = relevant[index].first;
            if (!(joinParameter > physicalFirst + parameterTolerance
                  && joinParameter < physicalLast - parameterTolerance)) continue;
            DebugLawJoinObservation join;
            join.parameter = joinParameter;
            relevant[index - 1].law->D1(joinParameter, join.valueLeft,
                                         join.derivativeLeft);
            relevant[index].law->D1(joinParameter, join.valueRight,
                                     join.derivativeRight);
            if (!finiteLawSample(join.valueLeft, join.derivativeLeft)
                || !finiteLawSample(join.valueRight, join.derivativeRight)
                || std::abs(join.valueLeft - join.valueRight) > valueTolerance
                || std::abs(join.derivativeLeft - join.derivativeRight)
                    > derivativeTolerance)
                c1 = false;
            output.joins.push_back(join);
        }

        output.supported = supported;
        output.c1OnPhysicalInterval = supported && c1;
        output.nonnegativeDerivative = supported && nonnegative;
        output.noConstantSpan = supported && noConstant;
    } catch (...) {
        output.exception = true;
    }
}
#endif

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
        std::vector<detail::AuthoredParameterMap> mappings;
        mappings.reserve(definition.edges.size());
        double minimumClearance = INFINITY;
        for (const auto& anchor : definition.edges) {
            if (cancelled.load()) return decline(Refusal::Cancelled);
            TopoDS_Edge edge;
            if (!detail::resolve(input, anchor, tolerance, edge, refusal)) return decline(refusal);
            if (BRep_Tool::IsClosed(edge)) return decline(Refusal::ClosedLoop);
            double clearance = 0;
            if (!detail::continuousClearance(input, edge, anchor, tolerance,
                                             clearance, refusal))
                return decline(refusal);
            detail::AuthoredParameterMap mapping;
            if (!detail::authoredParameterMap(edge, anchor, tolerance, mapping))
                return decline(Refusal::OrientationDrift);
            minimumClearance = std::min(minimumClearance, clearance);
            edges.push_back(edge); mappings.push_back(mapping);
            output.evidence.consumedEdges.push_back(anchor.identifier);
        }
        const double maximumRadius = std::max(definition.stations[0].radiusLocal,
                                               definition.stations[1].radiusLocal);
        if (!std::isfinite(minimumClearance) || maximumRadius >= minimumClearance / 2)
            return decline(Refusal::Clearance);

        BRepFilletAPI_MakeFillet fillet(input);
        std::set<int> contours;
        for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex) {
            const auto& edge = edges[edgeIndex];
            const auto& mapping = mappings[edgeIndex];
            const bool authoredStartsAtNativeFirst =
                mapping.authoredFirst == mapping.nativeFirst;
            const std::array<double, 2> nativeRadii{{
                definition.stations[authoredStartsAtNativeFirst ? 0 : 1].radiusLocal,
                definition.stations[authoredStartsAtNativeFirst ? 1 : 0].radiusLocal}};
#if DEBUG
            if (debugBuildObservation) ++debugBuildObservation->filletEntryCount;
#endif
            fillet.Add(nativeRadii[0], nativeRadii[1], edge);
            const int contour = fillet.Contour(edge);
            if (contour <= 0 || fillet.NbEdges(contour) != 1 || !contours.insert(contour).second)
                return decline(Refusal::ContourExpansion);
        }
        if (fillet.NbContours() != int(edges.size())) return decline(Refusal::ContourExpansion);
        fillet.Build();
        if (!fillet.IsDone() || fillet.Shape().IsNull()) return decline(Refusal::KernelFailure);
        for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex) {
            const auto& edge = edges[edgeIndex];
            const auto& mapping = mappings[edgeIndex];
            const bool authoredStartsAtNativeFirst =
                mapping.authoredFirst == mapping.nativeFirst;
            const std::array<double, 2> nativeRadii{{
                definition.stations[authoredStartsAtNativeFirst ? 0 : 1].radiusLocal,
                definition.stations[authoredStartsAtNativeFirst ? 1 : 0].radiusLocal}};
            const int contour = fillet.Contour(edge); Standard_Real first = 0, last = 0;
            const Handle(Law_Function) law = fillet.GetLaw(contour, edge);
            if (law.IsNull() || !fillet.GetBounds(contour, edge, first, last)
                || std::abs(law->Value(mapping.authoredFirst)
                    - definition.stations[0].radiusLocal) > tolerance
                || std::abs(law->Value(mapping.authoredLast)
                    - definition.stations[1].radiusLocal) > tolerance)
                return decline(Refusal::EndpointMismatch);
#if DEBUG
            if (debugBuildObservation) {
                DebugRealizedLawObservation observed;
                detail::observeRealizedLaw(law, edge, definition, mapping,
                                           nativeRadii, first, last, observed);
                debugBuildObservation->realizedLaws.push_back(std::move(observed));
            }
#endif
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

// ===== Stage 2: multi-station law. Everything above remains the stage-1 =====
// ===== two-station build; nothing here edits its gates or evidence.      =====

#include <TColgp_Array1OfPnt2d.hxx>

// Stage 2 routes the authored stations through the OCCT-native
// BRepFilletAPI_MakeFillet::Add(TColgp_Array1OfPnt2d, E) interpolating law.
// The generic Add(Law_Function, E) entry point throws Standard_NoSuchObject
// for every law object in this OCCT build (verified locally with OCCT's own
// Law_Linear and Law_Interpol), so no custom Law_Function subclass is used.
// The authored contract pins the exact radius at every station and lets the
// kernel interpolate between stations. Readback checks station values and
// bounded positive samples against the existing 0.5x-2x piecewise-linear
// admission-reference window; this is not an exact between-station guarantee.

inline BuildResult BuildMultiStation(const TopoDS_Shape& source,
                                     const MultiStationDefinition& definition,
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
        std::vector<detail::AuthoredParameterMap> mappings;
        mappings.reserve(definition.edges.size());
        double minimumClearance = INFINITY;
        for (const auto& anchor : definition.edges) {
            if (cancelled.load()) return decline(Refusal::Cancelled);
            TopoDS_Edge edge;
            if (!detail::resolve(input, anchor, tolerance, edge, refusal)) return decline(refusal);
            if (BRep_Tool::IsClosed(edge)) return decline(Refusal::ClosedLoop);
            double clearance = 0;
            if (!detail::continuousClearance(input, edge, anchor, tolerance,
                                             clearance, refusal))
                return decline(refusal);
            detail::AuthoredParameterMap mapping;
            if (!detail::authoredParameterMap(edge, anchor, tolerance, mapping))
                return decline(Refusal::OrientationDrift);
            minimumClearance = std::min(minimumClearance, clearance);
            edges.push_back(edge); mappings.push_back(mapping);
            output.evidence.consumedEdges.push_back(anchor.identifier);
        }
        double maximumRadius = 0;
        if (!MaximumRadiusLocal(definition, maximumRadius)
            || !std::isfinite(minimumClearance) || maximumRadius >= minimumClearance / 2)
            return decline(Refusal::Clearance);

        BRepFilletAPI_MakeFillet fillet(input);
        std::set<int> contours;
        struct NativeStation final {
            Station station;
            double normalized = 0;
        };
        std::vector<std::vector<NativeStation>> nativeStationsByEdge;
        nativeStationsByEdge.reserve(edges.size());
        for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex) {
            const auto& edge = edges[edgeIndex];
            const auto& mapping = mappings[edgeIndex];
#if DEBUG
            if (debugBuildObservation) ++debugBuildObservation->filletEntryCount;
#endif
            std::vector<NativeStation> nativeStations;
            nativeStations.reserve(definition.stations.size());
            for (const auto& station : definition.stations)
                nativeStations.push_back({station, mapping.normalized(station.parameter)});
            std::sort(nativeStations.begin(), nativeStations.end(),
                      [](const NativeStation& lhs, const NativeStation& rhs) {
                          return lhs.normalized < rhs.normalized;
                      });
            TColgp_Array1OfPnt2d lawPoints(1, Standard_Integer(definition.stations.size()));
            for (std::size_t index = 0; index < nativeStations.size(); ++index)
                lawPoints.SetValue(Standard_Integer(index) + 1,
                    gp_Pnt2d(nativeStations[index].normalized,
                             nativeStations[index].station.radiusLocal));
            fillet.Add(lawPoints, edge);
            nativeStationsByEdge.push_back(std::move(nativeStations));
            const int contour = fillet.Contour(edge);
            if (contour <= 0 || fillet.NbEdges(contour) != 1 || !contours.insert(contour).second)
                return decline(Refusal::ContourExpansion);
        }
        if (fillet.NbContours() != int(edges.size())) return decline(Refusal::ContourExpansion);
        fillet.Build();
        if (!fillet.IsDone() || fillet.Shape().IsNull()) return decline(Refusal::KernelFailure);
        for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex) {
            const auto& edge = edges[edgeIndex];
            const auto& mapping = mappings[edgeIndex];
            const int contour = fillet.Contour(edge);
            const Handle(Law_Function) law = fillet.GetLaw(contour, edge);
            if (law.IsNull()) return decline(Refusal::EndpointMismatch);
            BRepAdaptor_Curve spine(edge);
            const double first = spine.FirstParameter(), last = spine.LastParameter();
            if (!std::isfinite(first) || !std::isfinite(last) || !(last > first))
                return decline(Refusal::EndpointMismatch);
            const auto lawAt = [&](double authoredParameter) {
                return law->Value(mapping.parameter(authoredParameter));
            };
            // The realized kernel law must pin every authored station, not only
            // the endpoints; a law that misses a station refuses here.
            for (const auto& station : definition.stations) {
                if (!std::isfinite(lawAt(station.parameter))
                    || std::abs(lawAt(station.parameter) - station.radiusLocal) > tolerance)
                    return decline(Refusal::EndpointMismatch);
            }
#if DEBUG
            if (debugBuildObservation) {
                DebugMultiStationLawObservation observed;
                observed.attempted = true;
                observed.physicalFirst = first;
                observed.physicalLast = last;
                observed.authoredFirst = mapping.authoredFirst;
                observed.authoredLast = mapping.authoredLast;
                try {
                    BRepAdaptor_Curve observedCurve(edge);
                    for (const auto& native : nativeStationsByEdge[edgeIndex]) {
                        const double nativeParameter = first
                            + (last - first) * native.normalized;
                        observed.stations.push_back({
                            native.station.identifier,
                            native.station.parameter,
                            native.normalized,
                            nativeParameter,
                            native.station.radiusLocal,
                            observedCurve.Value(nativeParameter).Z(),
                            law->Value(nativeParameter)});
                    }
                } catch (...) {
                    observed.exception = true;
                }
                debugBuildObservation->multiStationLaws.push_back(std::move(observed));
            }
#endif
            // Between stations, check finite positive realised samples at the
            // three fixed quarter-segment positions against 0.5x-2x RadiusAt
            // reference values. These bounded samples are not equality checks
            // or an independent whole-law clearance proof.
            for (std::size_t segment = 1; segment < definition.stations.size(); ++segment) {
                const double from = definition.stations[segment - 1].parameter;
                const double to = definition.stations[segment].parameter;
                for (int sample = 1; sample <= 3; ++sample) {
                    const double parameter = from + (to - from) * double(sample) / 4;
                    double envelope = 0;
                    if (!RadiusAt(definition, parameter, envelope))
                        return decline(Refusal::EndpointMismatch);
                    const double realized = lawAt(parameter);
                    if (!std::isfinite(realized) || realized <= 0
                        || realized < envelope * .5 || realized > envelope * 2)
                        return decline(Refusal::EndpointMismatch);
                    if (realized >= minimumClearance / 2)
                        return decline(Refusal::Clearance);
                }
            }
        }
        const TopoDS_Shape candidate = fillet.Shape();
        if (!BRepCheck_Analyzer(candidate).IsValid()) return decline(Refusal::KernelFailure);
        if (!detail::selfIntersectionFree(candidate)) return decline(Refusal::SelfIntersection);

        GProp_GProps before, after; BRepGProp::VolumeProperties(input, before);
        BRepGProp::VolumeProperties(candidate, after);
        const double removed = before.Mass() - after.Mass();
        if (!std::isfinite(removed) || removed <= std::max(1e-12, before.Mass() * 1e-10))
            return decline(Refusal::NonRemoving);

        // Independent section proof: one measured normal-plane section at every
        // interior station parameter. Endpoint sections are proved by the exact
        // kernel-law check above; a plane through an endpoint vertex is not a
        // reliable independent measurement. Consecutive interior sections must
        // follow the authored segment direction, exactly as in stage 1.
        for (std::size_t edgeIndex = 0; edgeIndex < definition.edges.size(); ++edgeIndex) {
            double previousMeasured = 0; double previousAuthored = 0;
            for (std::size_t stationIndex = 1; stationIndex + 1 < definition.stations.size();
                 ++stationIndex) {
                const auto& station = definition.stations[stationIndex];
                double expected = 0, measured = 0;
                if (!RadiusAt(definition, station.parameter, expected)
                    || !detail::measuredSection(candidate, definition.edges[edgeIndex],
                                                station.parameter, expected, tolerance, measured))
                    return decline(Refusal::SectionMismatch);
                output.evidence.sections.push_back({definition.edges[edgeIndex].identifier,
                                                    station.parameter, expected, measured});
                if (previousMeasured > 0) {
                    const double delta = station.radiusLocal - previousAuthored;
                    if ((delta > 0 && measured <= previousMeasured)
                        || (delta < 0 && measured >= previousMeasured)
                        || (delta == 0 && std::abs(measured - previousMeasured) > tolerance))
                        return decline(Refusal::SectionMismatch);
                }
                previousMeasured = measured; previousAuthored = station.radiusLocal;
            }
        }
        output.refusal = Refusal::None; output.solid = candidate;
        output.evidence.minimumClearanceLocal = minimumClearance;
        output.evidence.removedVolumeLocal3 = removed;
        return output;
    } catch (...) { return decline(Refusal::KernelFailure); }
}

inline BuildResult BuildMultiStationDeterministically(
    const TopoDS_Shape& source, const MultiStationDefinition& definition,
    const std::atomic_bool& cancelled) noexcept {
    BuildResult first = BuildMultiStation(source, definition, cancelled);
    if (!first.built()) return first;
    BuildResult second = BuildMultiStation(source, definition, cancelled);
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
