#pragma once

// T-C C4: exact spline/line/arc face construction and revolve/extrude builds
// against the app's OCCT headers, reusing the same work-plane basis, wire
// admission and clearance discipline as ProfileCurveFace.hxx. Legacy
// line/arc-only sections keep routing through BuildProfileCurveFace; this
// builder is entered only when a section carries spline payloads.
#include "SplineProfileAdmission.hxx"
#include "ProfileDefinition.hxx"
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Wire.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <GProp_GProps.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Circle.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <atomic>

namespace core3d {

struct SplineProfileFaceResult {
    TopoDS_Face face;
    double expectedArea = 0;
    double expectedMomentU = 0;
    double expectedMomentV = 0;
    std::array<double, 4> authoredOuterBounds{};
};

// Build the planar section face for a spline-carrying section. The caller has
// already run InspectSplineProfileSection; this builder re-proves geometry at
// kernel level: wire closure, planar face validity, self-intersection,
// nonadjacent-edge clearance and inner-loop containment, then cross-checks the
// measured area/moments against the authored quadrature (independent sections).
inline bool BuildSplineProfileFace(const ProfileCurveSection& section,
    const SplineProfileSpec& spec, int plane, const std::atomic_bool& cancelled,
    SplineProfileFaceResult& output) noexcept {
    output = {};
    try {
        constexpr double clearance = 1e-3, tolerance = 1e-7;
        SplineProfileSectionMoments moments;
        if (cancelled.load() || plane < 0 || plane > 2
            || !InspectSplineProfileSection(section, spec, moments,
                [&] { return cancelled.load(); })) return false;
        const gp_Dir normal = plane == 0 ? gp::DZ()
            : (plane == 1 ? gp_Dir(0, -1, 0) : gp::DX());
        const gp_Dir uDirection = plane == 2 ? gp::DY() : gp::DX();
        const gp_Pln support(gp::Origin(), normal);
        const double degreesToRadians = std::acos(-1.0) / 180.0;
        const auto splineFor = [&](ProfileCurveID identifier) -> const SplineCurveSegment* {
            for (const auto& candidate : spec.segments)
                if (candidate.identifier == identifier) return &candidate;
            return nullptr;
        };
        struct Loop {
            TopoDS_Wire positiveWire;
            TopoDS_Face positiveFace;
            std::vector<TopoDS_Edge> edges;
            gp_Pnt representative;
            double signedArea = 0;
        };
        std::vector<Loop> prepared;
        std::vector<double> loopSignedAreas;
        const auto distant = [&](const TopoDS_Shape& a, const TopoDS_Shape& b) {
            if (cancelled.load()) return false;
            BRepExtrema_DistShapeShape distance(a, b);
            return !cancelled.load() && distance.IsDone() && distance.NbSolution() > 0
                && std::isfinite(distance.Value()) && distance.Value() >= clearance;
        };
        for (std::size_t index = 0; index <= section.inner.size(); ++index) {
            if (cancelled.load()) return false;
            const auto& loop = index == 0 ? section.outer : section.inner[index - 1];
            Loop built;
            std::vector<TopoDS_Vertex> vertices;
            for (const auto& vertex : loop.vertices) {
                if (cancelled.load()) return false;
                BRepBuilderAPI_MakeVertex made(ProfilePointInPlane(vertex.point, plane));
                if (!made.IsDone()) return false;
                vertices.push_back(made.Vertex());
            }
            BRepBuilderAPI_MakeWire maker;
            double signedArea = 0;
            {
                // Loop-level signed area from the authored quadrature alone
                // drives winding normalization (never kernel reordering).
                spline_profile_admission::LoopMoments loopMoments;
                std::set<ProfileCurveID> loopIDs, loopSplines;
                std::size_t loopElements = 0;
                if (!InspectSplineProfileLoop(loop, spec, loopIDs, loopSplines, loopElements,
                        loopMoments)) return false;
                signedArea = loopMoments.signedArea;
            }
            for (std::size_t edgeIndex = 0; edgeIndex < loop.segments.size(); ++edgeIndex) {
                if (cancelled.load()) return false;
                const auto& segment = loop.segments[edgeIndex];
                const auto& first = vertices[edgeIndex];
                const auto& last = vertices[(edgeIndex + 1) % vertices.size()];
                TopoDS_Edge edge;
                if (segment.kind == ProfileCurveKind::Line) {
                    BRepBuilderAPI_MakeEdge made(first, last);
                    if (!made.IsDone()) return false;
                    edge = made.Edge();
                } else if (segment.kind == ProfileCurveKind::CircularArc) {
                    const Handle(Geom_Circle) circle = new Geom_Circle(gp_Circ(
                        gp_Ax2(ProfilePointInPlane(segment.center, plane), normal, uDirection),
                        segment.radius));
                    const double a = segment.startDegrees * degreesToRadians;
                    const double b = (segment.startDegrees + segment.sweepDegrees) * degreesToRadians;
                    BRepBuilderAPI_MakeEdge made(circle,
                        segment.sweepDegrees > 0 ? first : last,
                        segment.sweepDegrees > 0 ? last : first, std::min(a, b), std::max(a, b));
                    if (!made.IsDone()) return false;
                    edge = segment.sweepDegrees > 0 ? made.Edge() : TopoDS::Edge(made.Edge().Reversed());
                } else if (segment.kind == ProfileCurveKind::Spline) {
                    const SplineCurveSegment* spline = splineFor(segment.identifier);
                    if (!spline) return false;
                    const int poleCount = int(spline->poles.size());
                    const int knotCount = int(spline->knots.size());
                    TColgp_Array1OfPnt poles(1, poleCount);
                    for (int i = 1; i <= poleCount; ++i)
                        poles(i) = ProfilePointInPlane(spline->poles[std::size_t(i - 1)].point, plane);
                    TColStd_Array1OfReal knots(1, knotCount);
                    TColStd_Array1OfInteger multiplicities(1, knotCount);
                    for (int i = 1; i <= knotCount; ++i) {
                        knots(i) = spline->knots[std::size_t(i - 1)];
                        multiplicities(i) = spline->multiplicities[std::size_t(i - 1)];
                    }
                    Handle(Geom_BSplineCurve) curve;
                    if (spline->weights.empty()) {
                        curve = new Geom_BSplineCurve(poles, knots, multiplicities, spline->degree);
                    } else {
                        TColStd_Array1OfReal weights(1, poleCount);
                        for (int i = 1; i <= poleCount; ++i)
                            weights(i) = spline->weights[std::size_t(i - 1)];
                        curve = new Geom_BSplineCurve(poles, weights, knots, multiplicities,
                                                      spline->degree);
                    }
                    BRepBuilderAPI_MakeEdge made(curve, first, last);
                    if (!made.IsDone()) return false;
                    edge = made.Edge();
                } else return false;
                if (edge.IsNull()) return false;
                maker.Add(edge);
                if (!maker.IsDone()) return false;
                built.edges.push_back(edge);
            }
            const TopoDS_Wire authoredWire = maker.Wire();
            if (authoredWire.IsNull()
                || BRepCheck_Wire(authoredWire).Closed() != BRepCheck_NoError) return false;
            built.positiveWire = signedArea > 0
                ? authoredWire : TopoDS::Wire(authoredWire.Reversed());
            BRepBuilderAPI_MakeFace face(support, built.positiveWire, Standard_True);
            if (!face.IsDone()) return false;
            built.positiveFace = face.Face();
            BRepCheck_Wire wireCheck(built.positiveWire);
            TopoDS_Edge firstIntersection, secondIntersection;
            if (wireCheck.Closed2d(built.positiveFace) != BRepCheck_NoError
                || wireCheck.SelfIntersect(built.positiveFace, firstIntersection,
                                           secondIntersection) != BRepCheck_NoError
                || !BRepCheck_Analyzer(built.positiveFace, Standard_True).IsValid()) return false;
            for (std::size_t i = 0; i < built.edges.size(); ++i)
                for (std::size_t j = i + 1; j < built.edges.size(); ++j) {
                    if (j == i + 1 || (i == 0 && j + 1 == built.edges.size())) continue;
                    if (!distant(built.edges[i], built.edges[j])) return false;
                }
            built.representative = ProfilePointInPlane(loop.vertices.front().point, plane);
            built.signedArea = signedArea;
            prepared.push_back(std::move(built));
        }
        // Disjoint connected loops have a consistent containment relation.
        for (std::size_t i = 1; i < prepared.size(); ++i) {
            if (!distant(prepared.front().positiveWire, prepared[i].positiveWire)) return false;
            BRepClass_FaceClassifier outer(prepared.front().positiveFace,
                                           prepared[i].representative, tolerance);
            if (outer.State() != TopAbs_IN) return false;
            for (std::size_t j = 1; j < i; ++j) {
                if (!distant(prepared[i].positiveWire, prepared[j].positiveWire)) return false;
                BRepClass_FaceClassifier a(prepared[i].positiveFace,
                                           prepared[j].representative, tolerance);
                BRepClass_FaceClassifier b(prepared[j].positiveFace,
                                           prepared[i].representative, tolerance);
                if (a.State() != TopAbs_OUT || b.State() != TopAbs_OUT) return false;
            }
        }
        BRepBuilderAPI_MakeFace face(support, prepared.front().positiveWire, Standard_True);
        if (!face.IsDone()) return false;
        for (std::size_t i = 1; i < prepared.size(); ++i) {
            face.Add(TopoDS::Wire(prepared[i].positiveWire.Reversed()));
            if (!face.IsDone()) return false;
        }
        const TopoDS_Face result = face.Face();
        if (cancelled.load() || result.IsNull()
            || !BRepCheck_Analyzer(result, Standard_True).IsValid()) return false;
        // Independent-section proof: the kernel-measured area and centroid must
        // reproduce the authored quadrature (exact for spline integrands).
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(result, properties);
        const auto centroid = properties.CentreOfMass();
        const double measuredArea = properties.Mass();
        double measuredMomentU = 0, measuredMomentV = 0;
        switch (plane) {
            case 0: measuredMomentU = measuredArea * centroid.X();
                    measuredMomentV = measuredArea * centroid.Y(); break;
            case 1: measuredMomentU = measuredArea * centroid.X();
                    measuredMomentV = measuredArea * centroid.Z(); break;
            default: measuredMomentU = measuredArea * centroid.Y();
                     measuredMomentV = measuredArea * centroid.Z(); break;
        }
        const double areaScale = std::max(1.0, moments.area);
        const double momentScale = std::max(1.0,
            std::max(std::abs(moments.momentU), std::abs(moments.momentV)));
        if (!std::isfinite(measuredArea) || measuredArea <= 0
            || std::abs(measuredArea - moments.area) > 1e-6 * areaScale
            || std::abs(measuredMomentU - moments.momentU) > 1e-6 * momentScale
            || std::abs(measuredMomentV - moments.momentV) > 1e-6 * momentScale) return false;
        output.face = result;
        output.expectedArea = moments.area;
        output.expectedMomentU = moments.momentU;
        output.expectedMomentV = moments.momentV;
        output.authoredOuterBounds = moments.outerBounds;
        return true;
    } catch (...) { output = {}; return false; }
}

struct SplineRevolveResult {
    TopoDS_Solid solid;
    double measuredVolume = 0;
    int seamFaces = 0;
};

inline gp_Ax1 SplineRevolveAxis3D(const SplineRevolveAxis& axis, int plane) {
    const gp_Pnt origin = ProfilePointInPlane(axis.origin, plane);
    const gp_Pnt2d head2(axis.origin.X() + axis.direction.X(),
                         axis.origin.Y() + axis.direction.Y());
    const gp_Pnt head = ProfilePointInPlane(head2, plane);
    return gp_Ax1(origin, gp_Dir(gp_Vec(origin, head)));
}

// Revolution seam proof: the section-plane caps of a partial revolution are
// exactly the planar faces whose plane contains the axis line; a full
// revolution has none. The cap dihedral must equal the authored angle.
inline bool ProveSplineRevolveSeam(const TopoDS_Solid& solid, const gp_Ax1& axis,
    double angleDegrees, int& seamFaces) noexcept {
    seamFaces = 0;
    try {
        // Capture the axis-containing planar cap normals (bounded: <= 2).
        gp_Dir caps[2];
        int capCount = 0;
        for (TopExp_Explorer explorer(solid, TopAbs_FACE); explorer.More(); explorer.Next()) {
            BRepAdaptor_Surface surface(TopoDS::Face(explorer.Current()), Standard_True);
            if (surface.GetType() != GeomAbs_Plane) continue;
            const gp_Pln plane = surface.Plane();
            const gp_Dir planeNormal = plane.Axis().Direction();
            const double parallel = std::abs(planeNormal.X() * axis.Direction().X()
                + planeNormal.Y() * axis.Direction().Y()
                + planeNormal.Z() * axis.Direction().Z());
            if (parallel > 1e-6) continue;
            const gp_Vec offset(plane.Location(), axis.Location());
            const double distance = std::abs(offset.X() * planeNormal.X()
                + offset.Y() * planeNormal.Y() + offset.Z() * planeNormal.Z());
            if (distance > 1e-7) continue;
            if (capCount >= 2) return false;
            caps[capCount++] = planeNormal;
        }
        seamFaces = capCount;
        const bool full = angleDegrees == 360.0;
        if (full) return capCount == 0;
        if (capCount != 2) return false;
        const double theta = angleDegrees * std::acos(-1.0) / 180.0;
        const double dot = caps[0].X() * caps[1].X() + caps[0].Y() * caps[1].Y()
            + caps[0].Z() * caps[1].Z();
        const gp_Vec cross = gp_Vec(caps[0]).Crossed(gp_Vec(caps[1]));
        return std::abs(std::abs(dot) - std::abs(std::cos(theta))) <= 1e-6
            && std::abs(cross.Magnitude() - std::abs(std::sin(theta))) <= 1e-6;
    } catch (...) { seamFaces = 0; return false; }
}

// Revolve an admitted spline face about the explicit authored axis. Full and
// partial revolutions share one path; the seam proof distinguishes them.
inline bool BuildSplineProfileRevolve(const TopoDS_Face& face,
    const SplineRevolveAxis& axis, int plane, double angleDegrees,
    double expectedVolume, const std::atomic_bool& cancelled,
    SplineRevolveResult& output) noexcept {
    output = {};
    try {
        if (face.IsNull() || cancelled.load() || !std::isfinite(expectedVolume)
            || expectedVolume <= 1e-8) return false;
        const gp_Ax1 axis3D = SplineRevolveAxis3D(axis, plane);
        TopoDS_Shape result;
        if (angleDegrees == 360.0) {
            BRepPrimAPI_MakeRevol sweep(face, axis3D, Standard_True);
            if (!sweep.IsDone()) return false;
            result = sweep.Shape();
        } else {
            BRepPrimAPI_MakeRevol sweep(face, axis3D,
                angleDegrees * std::acos(-1.0) / 180.0, Standard_True);
            if (!sweep.IsDone()) return false;
            result = sweep.Shape();
        }
        if (cancelled.load() || result.IsNull() || result.ShapeType() != TopAbs_SOLID) return false;
        auto solid = TopoDS::Solid(result);
        if (!BRepLib::OrientClosedSolid(solid)
            || !BRepCheck_Analyzer(solid, Standard_True).IsValid()) return false;
        GProp_GProps properties;
        BRepGProp::VolumeProperties(solid, properties);
        if (!std::isfinite(properties.Mass()) || properties.Mass() <= 0
            || std::abs(properties.Mass() - expectedVolume)
                > std::max(1e-8, expectedVolume * 1e-6)) return false;
        int seamFaces = 0;
        if (!ProveSplineRevolveSeam(solid, axis3D, angleDegrees, seamFaces)) return false;
        if (cancelled.load()) return false;
        output.solid = solid;
        output.measuredVolume = properties.Mass();
        output.seamFaces = seamFaces;
        return true;
    } catch (...) { output = {}; return false; }
}

// Extrusion of an admitted spline face reuses the existing prism builder and
// the same validity/volume proof as the legacy path.
inline bool BuildSplineProfileExtrude(const TopoDS_Face& face, int plane, double depth,
    double expectedVolume, const std::atomic_bool& cancelled, TopoDS_Solid& output) noexcept {
    output = TopoDS_Solid();
    try {
        if (face.IsNull() || cancelled.load()) return false;
        const gp_Vec direction = plane == 0 ? gp_Vec(0, 0, depth)
            : plane == 1 ? gp_Vec(0, depth, 0) : gp_Vec(depth, 0, 0);
        BRepPrimAPI_MakePrism prism(face, direction, Standard_True, Standard_False);
        if (!prism.IsDone()) return false;
        const TopoDS_Shape result = prism.Shape();
        if (cancelled.load() || result.IsNull() || result.ShapeType() != TopAbs_SOLID) return false;
        auto solid = TopoDS::Solid(result);
        if (!BRepLib::OrientClosedSolid(solid)
            || !BRepCheck_Analyzer(solid, Standard_True).IsValid()) return false;
        GProp_GProps properties;
        BRepGProp::VolumeProperties(solid, properties);
        if (!std::isfinite(properties.Mass()) || properties.Mass() <= 0
            || std::abs(properties.Mass() - expectedVolume)
                > std::max(1e-8, expectedVolume * 1e-6)) return false;
        output = solid;
        return true;
    } catch (...) { output = TopoDS_Solid(); return false; }
}

} // namespace core3d
