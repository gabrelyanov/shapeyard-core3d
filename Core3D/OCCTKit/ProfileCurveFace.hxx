#pragma once

// Exact line/arc face construction against the app's OCCT headers.
// Uses the same U/V work-plane basis as ProfileDefinition and the existing
// detached profile worker. No document, presentation or viewport mutations.
#include "ProfileCurveStructure.hxx"
#include "ProfileDefinition.hxx"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Wire.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Circle.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <gp.hxx>
#include <atomic>
#include <utility>

namespace core3d {
struct ProfileCurveFaceResult {
    TopoDS_Face face;
    double expectedArea = 0;
    double expectedFirstMomentX = 0;
    std::array<double,4> authoredOuterBounds{};
};

inline bool BuildProfileCurveFace(const ProfileCurveSection& section, int plane,
    const std::atomic_bool& cancelled, ProfileCurveFaceResult& output) noexcept {
    output = {};
    try {
        constexpr double clearance = 1e-3, tolerance = 1e-7;
        if (cancelled.load() || plane < 0 || plane > 2 || section.inner.size() > 16
            || section.outer.vertices.empty()) return false;
        const gp_Dir normal = plane == 0 ? gp::DZ()
            : (plane == 1 ? gp_Dir(0,-1,0) : gp::DX());
        const gp_Dir uDirection = plane == 2 ? gp::DY() : gp::DX();
        const gp_Pln support(gp::Origin(),normal);
        const double degreesToRadians = std::acos(-1.0)/180.0;
        const gp_Pnt2d origin = section.outer.vertices.front().point;
        std::set<ProfileCurveID> identifiers;
        std::size_t verticesInspected = 0, segmentsInspected = 0;
        struct Loop {
            TopoDS_Wire positiveWire;
            TopoDS_Face positiveFace;
            std::vector<TopoDS_Edge> edges;
            gp_Pnt representative;
            ProfileCurveLoopInspection inspection;
        };
        std::vector<Loop> prepared;
        const auto distant = [&](const TopoDS_Shape& a,const TopoDS_Shape& b) {
            if (cancelled.load()) return false;
            BRepExtrema_DistShapeShape distance(a,b);
            return !cancelled.load() && distance.IsDone() && distance.NbSolution() > 0
                && std::isfinite(distance.Value()) && distance.Value() >= clearance;
        };
        for (std::size_t index = 0; index <= section.inner.size(); ++index) {
            if (cancelled.load()) return false;
            const auto& loop = index == 0 ? section.outer : section.inner[index-1];
            Loop built;
            if (!InspectProfileCurveLoopStructure(loop,origin,identifiers,
                verticesInspected,segmentsInspected,built.inspection)) return false;
            std::vector<TopoDS_Vertex> vertices;
            for (const auto& vertex : loop.vertices) {
                if (cancelled.load()) return false;
                BRepBuilderAPI_MakeVertex made(ProfilePointInPlane(vertex.point,plane));
                if (!made.IsDone()) return false;
                vertices.push_back(made.Vertex());
            }
            BRepBuilderAPI_MakeWire maker;
            for (std::size_t edgeIndex = 0; edgeIndex < loop.segments.size(); ++edgeIndex) {
                if (cancelled.load()) return false;
                const auto& segment = loop.segments[edgeIndex];
                const auto& first = vertices[edgeIndex];
                const auto& last = vertices[(edgeIndex+1)%vertices.size()];
                TopoDS_Edge edge;
                if (segment.kind == ProfileCurveKind::Line) {
                    BRepBuilderAPI_MakeEdge made(first,last);
                    if (!made.IsDone()) return false;
                    edge = made.Edge();
                } else {
                    const Handle(Geom_Circle) circle = new Geom_Circle(gp_Circ(
                        gp_Ax2(ProfilePointInPlane(segment.center,plane),normal,uDirection),segment.radius));
                    const double a = segment.startDegrees*degreesToRadians;
                    const double b = (segment.startDegrees+segment.sweepDegrees)*degreesToRadians;
                    // Build an ascending circle interval with the corresponding
                    // shared vertices, then restore authored traversal direction.
                    BRepBuilderAPI_MakeEdge made(circle,
                        segment.sweepDegrees > 0 ? first : last,
                        segment.sweepDegrees > 0 ? last : first,std::min(a,b),std::max(a,b));
                    if (!made.IsDone()) return false;
                    edge = segment.sweepDegrees > 0 ? made.Edge() : TopoDS::Edge(made.Edge().Reversed());
                }
                if (edge.IsNull()) return false;
                maker.Add(edge);
                if (!maker.IsDone()) return false;
                built.edges.push_back(edge);
            }
            const TopoDS_Wire authoredWire = maker.Wire();
            if (authoredWire.IsNull() || BRepCheck_Wire(authoredWire).Closed() != BRepCheck_NoError) return false;
            built.positiveWire = built.inspection.signedArea > 0 ? authoredWire : TopoDS::Wire(authoredWire.Reversed());
            BRepBuilderAPI_MakeFace face(support,built.positiveWire,Standard_True);
            if (!face.IsDone()) return false;
            built.positiveFace = face.Face();
            BRepCheck_Wire wireCheck(built.positiveWire);
            TopoDS_Edge firstIntersection,secondIntersection;
            if (wireCheck.Closed2d(built.positiveFace) != BRepCheck_NoError
                || wireCheck.SelfIntersect(built.positiveFace,firstIntersection,secondIntersection) != BRepCheck_NoError
                || !BRepCheck_Analyzer(built.positiveFace,Standard_True).IsValid()) return false;
            // Adjacent edges meet by design. Nonadjacent boundaries must retain
            // clearance; cancellation remains observable between bounded queries.
            for (std::size_t i = 0; i < built.edges.size(); ++i)
                for (std::size_t j = i+1; j < built.edges.size(); ++j) {
                    if (j == i+1 || (i == 0 && j+1 == built.edges.size())) continue;
                    if (!distant(built.edges[i],built.edges[j])) return false;
                }
            GProp_GProps properties;
            BRepGProp::SurfaceProperties(built.positiveFace,properties);
            const double area = std::abs(built.inspection.signedArea);
            if (!std::isfinite(properties.Mass())
                || std::abs(properties.Mass()-area) > 1e-9*std::max(1.0,area)) return false;
            built.representative = ProfilePointInPlane(loop.vertices.front().point,plane);
            prepared.push_back(std::move(built));
        }
        // Disjoint connected loops have a consistent containment relation.
        // Clearance rules exclude touching boundaries before point classification.
        for (std::size_t i = 1; i < prepared.size(); ++i) {
            if (!distant(prepared.front().positiveWire,prepared[i].positiveWire)) return false;
            BRepClass_FaceClassifier outer(prepared.front().positiveFace,prepared[i].representative,tolerance);
            if (outer.State() != TopAbs_IN) return false;
            for (std::size_t j = 1; j < i; ++j) {
                if (!distant(prepared[i].positiveWire,prepared[j].positiveWire)) return false;
                BRepClass_FaceClassifier a(prepared[i].positiveFace,prepared[j].representative,tolerance);
                BRepClass_FaceClassifier b(prepared[j].positiveFace,prepared[i].representative,tolerance);
                if (a.State() != TopAbs_OUT || b.State() != TopAbs_OUT) return false;
            }
        }
        BRepBuilderAPI_MakeFace face(support,prepared.front().positiveWire,Standard_True);
        if (!face.IsDone()) return false;
        double area = 0,firstMoment = 0;
        for (std::size_t i = 0; i < prepared.size(); ++i) {
            if (cancelled.load()) return false;
            const auto& loop = prepared[i];
            const double orientation = loop.inspection.signedArea > 0 ? 1 : -1;
            const double role = i == 0 ? 1 : -1;
            area += role*std::abs(loop.inspection.signedArea);
            firstMoment += role*orientation*loop.inspection.signedFirstMomentX;
            if (i != 0) face.Add(TopoDS::Wire(loop.positiveWire.Reversed()));
            if (!face.IsDone()) return false;
        }
        firstMoment += origin.X()*area;
        const TopoDS_Face result = face.Face();
        if (cancelled.load() || result.IsNull() || !std::isfinite(area) || area < 1e-6
            || !std::isfinite(firstMoment) || !BRepCheck_Analyzer(result,Standard_True).IsValid()) return false;
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(result,properties);
        const auto centroid = properties.CentreOfMass();
        const double centroidX = plane == 2 ? centroid.Y() : centroid.X();
        if (!std::isfinite(properties.Mass()) || !std::isfinite(centroidX)
            || std::abs(properties.Mass()-area) > 1e-9*std::max(1.0,area)
            || std::abs(properties.Mass()*centroidX-firstMoment) > 1e-9*std::max(1.0,std::abs(firstMoment))) return false;
        if (cancelled.load()) return false;
        output.face = result; output.expectedArea = area;
        output.expectedFirstMomentX = firstMoment;
        output.authoredOuterBounds = prepared.front().inspection.bounds;
        return true;
    } catch (...) { output = {}; return false; }
}
} // namespace core3d
