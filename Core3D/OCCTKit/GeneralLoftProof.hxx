#pragma once

// Independent C3 admission over the untrusted OCCT result. Volume and kernel
// success are supplemental only; every required side, seam, cap and authored
// curve correspondence is checked. Any unavailable measurement is refusal.
#include "GeneralLoftSolid.hxx"
#include <BOPAlgo_ArgumentAnalyzer.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Plane.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shell.hxx>
#include <deque>
#include <set>

namespace core3d::general_loft {
enum class ProofStatus : std::uint8_t {
    Admitted = 0, NotBuilt, Cancelled, InputChanged, MissingSide,
    ReorderedCorrespondence, MissingSeam, MissingCap, BadSection,
    Disconnected, InvalidSolid, SelfIntersection, Inconclusive, BadMetric
};

struct ProofReceipt {
    ProofStatus status = ProofStatus::NotBuilt;
    std::size_t stationCount = 0, segmentCount = 0, sideCount = 0;
    std::size_t seamCount = 0, capCount = 0, sectionSamples = 0;
    double volume = 0, conservativeUpperVolume = 0;
    bool compatibilityRepairDisabled = true;
    bool immutableInput = true;
};

struct AdmittedSolid {
    TopoDS_Solid solid;
    ProofReceipt proof;
    bool admitted() const noexcept {
        return proof.status == ProofStatus::Admitted && !solid.IsNull();
    }
};

namespace proof_detail {
inline bool SameShapeIn(const TopoDS_Shape& needle, const TopTools_IndexedMapOfShape& map) {
    return !needle.IsNull() && map.FindIndex(needle) > 0;
}
inline bool OpposedUses(const TopoDS_Edge& edge, const TopoDS_Face& a, const TopoDS_Face& b) {
    TopAbs_Orientation oa = TopAbs_EXTERNAL, ob = TopAbs_EXTERNAL;
    for (TopExp_Explorer it(a, TopAbs_EDGE); it.More(); it.Next())
        if (it.Current().IsSame(edge)) oa = it.Current().Orientation();
    for (TopExp_Explorer it(b, TopAbs_EDGE); it.More(); it.Next())
        if (it.Current().IsSame(edge)) ob = it.Current().Orientation();
    return (oa == TopAbs_FORWARD && ob == TopAbs_REVERSED)
        || (oa == TopAbs_REVERSED && ob == TopAbs_FORWARD);
}
inline bool PointOnFace(const gp_Pnt& point, const TopoDS_Face& face, double tolerance) {
    BRepBuilderAPI_MakeVertex vertex(point);
    if (!vertex.IsDone()) return false;
    BRepExtrema_DistShapeShape distance(vertex.Vertex(), face);
    return distance.IsDone() && distance.NbSolution() > 0
        && std::isfinite(distance.Value()) && distance.Value() <= tolerance;
}
inline bool SharesBoundary(const TopoDS_Face& a, const TopoDS_Face& b) {
    for (TopExp_Explorer ai(a, TopAbs_VERTEX); ai.More(); ai.Next())
        for (TopExp_Explorer bi(b, TopAbs_VERTEX); bi.More(); bi.Next())
            if (ai.Current().IsSame(bi.Current())) return true;
    return false;
}
inline bool ReferenceRuledSamples(const Definition& definition,
                                  const KernelBuild& build,
                                  const std::atomic_bool& cancelled,
                                  std::size_t& sampleCount) {
    sampleCount = 0;
    constexpr std::array<double, 5> curveSamples{{0, 0.25, 0.5, 0.75, 1}};
    constexpr std::array<double, 3> intervalSamples{{0.25, 0.5, 0.75}};
    const double tolerance = 8 * build.positionalTolerance;
    for (std::size_t station = 0; station + 1 < definition.stations.size(); ++station) {
        for (std::size_t segment = 0; segment < definition.stations[station].segments.size(); ++segment) {
            const auto a = solid_detail::ExactCurve(
                definition.stations[station].segments[segment].curveState.definition);
            const auto b = solid_detail::ExactCurve(
                definition.stations[station + 1].segments[segment].curveState.definition);
            if (a.IsNull() || b.IsNull()) return false;
            for (double u : curveSamples) for (double t : intervalSamples) {
                if (cancelled.load(std::memory_order_relaxed)) return false;
                const gp_Pnt pa = a->Value(a->FirstParameter()
                    + u * (a->LastParameter() - a->FirstParameter()));
                const gp_Pnt pb = b->Value(b->FirstParameter()
                    + u * (b->LastParameter() - b->FirstParameter()));
                const gp_Pnt expected(pa.X() + t * (pb.X() - pa.X()),
                                      pa.Y() + t * (pb.Y() - pa.Y()),
                                      pa.Z() + t * (pb.Z() - pa.Z()));
                if (!PointOnFace(expected, build.generatedSides[station][segment], tolerance))
                    return false;
                ++sampleCount;
            }
        }
    }
    return true;
}
} // namespace proof_detail

inline AdmittedSolid ProveDetached(const Definition& definition,
                                   const KernelBuild& build,
                                   const std::atomic_bool& cancelled) noexcept {
    AdmittedSolid output;
    ProofReceipt& receipt = output.proof;
    if (cancelled.load(std::memory_order_relaxed)) {
        receipt.status = ProofStatus::Cancelled; return output;
    }
    if (Validate(definition) != Admission::Accepted
        || build.status != KernelBuildStatus::BuiltUnproven || build.solid.IsNull())
        return output;
    try {
        receipt.stationCount = definition.stations.size();
        receipt.segmentCount = definition.stations.front().segments.size();
        if (build.stationWires.size() != receipt.stationCount
            || build.stationEdges.size() != receipt.stationCount
            || build.generatedSides.size() + 1 != receipt.stationCount) {
            receipt.status = ProofStatus::InputChanged; return output;
        }
        for (std::size_t station = 0; station < receipt.stationCount; ++station) {
            if (build.stationEdges[station].size() != receipt.segmentCount) {
                receipt.status = ProofStatus::ReorderedCorrespondence; return output;
            }
            if (station + 1 < receipt.stationCount
                && build.generatedSides[station].size() != receipt.segmentCount) {
                receipt.status = ProofStatus::MissingSide; return output;
            }
        }

        TopTools_IndexedMapOfShape solids, shells, faces, edges;
        TopExp::MapShapes(build.solid, TopAbs_SOLID, solids);
        TopExp::MapShapes(build.solid, TopAbs_SHELL, shells);
        TopExp::MapShapes(build.solid, TopAbs_FACE, faces);
        TopExp::MapShapes(build.solid, TopAbs_EDGE, edges);
        if (solids.Extent() != 1 || shells.Extent() != 1
            || !BRep_Tool::IsClosed(TopoDS::Shell(shells(1)))
            || !BRepCheck_Analyzer(build.solid, Standard_True).IsValid()) {
            receipt.status = ProofStatus::InvalidSolid; return output;
        }
        const std::size_t expectedSides = (receipt.stationCount - 1) * receipt.segmentCount;
        if (faces.Extent() != Standard_Integer(expectedSides + 2)
            || !proof_detail::SameShapeIn(build.firstCap, faces)
            || !proof_detail::SameShapeIn(build.lastCap, faces)) {
            receipt.status = ProofStatus::MissingCap; return output;
        }
        TopLoc_Location capLocation;
        if (Handle(Geom_Plane)::DownCast(BRep_Tool::Surface(build.firstCap, capLocation)).IsNull()
            || Handle(Geom_Plane)::DownCast(BRep_Tool::Surface(build.lastCap, capLocation)).IsNull()) {
            receipt.status = ProofStatus::MissingCap; return output;
        }
        if (build.firstCap.Orientation() == build.lastCap.Orientation()) {
            receipt.status = ProofStatus::MissingCap; return output;
        }
        receipt.capCount = 2;

        std::set<int> sideIndices;
        for (const auto& interval : build.generatedSides)
            for (const TopoDS_Face& face : interval) {
                const int index = faces.FindIndex(face);
                if (index <= 0 || !sideIndices.insert(index).second) {
                    receipt.status = ProofStatus::MissingSide; return output;
                }
            }
        if (sideIndices.size() != expectedSides) {
            receipt.status = ProofStatus::MissingSide; return output;
        }
        receipt.sideCount = sideIndices.size();
        for (const auto& interval : build.generatedSides)
            for (const TopoDS_Face& face : interval)
                for (TopExp_Explorer eit(face, TopAbs_EDGE); eit.More(); eit.Next()) {
                    const TopoDS_Edge edge = TopoDS::Edge(eit.Current());
                    TopLoc_Location location; Standard_Real first = 0, last = 0;
                    if (BRep_Tool::Curve(edge, location, first, last).IsNull()) {
                        receipt.status = ProofStatus::BadSection; return output;
                    }
                    Standard_Real pfirst = 0, plast = 0;
                    if (BRep_Tool::CurveOnSurface(edge, face, pfirst, plast).IsNull()) {
                        receipt.status = ProofStatus::BadSection; return output;
                    }
                }

        // Every authored station seam must survive once as the boundary shared
        // by its two neighbouring interval faces (or by side + cap at an end).
        for (std::size_t station = 0; station < receipt.stationCount; ++station)
            for (std::size_t segment = 0; segment < receipt.segmentCount; ++segment) {
                const TopoDS_Edge& seam = build.stationEdges[station][segment];
                if (!proof_detail::SameShapeIn(seam, edges)) {
                    receipt.status = ProofStatus::MissingSeam; return output;
                }
                const TopoDS_Face& a = station == 0 ? build.firstCap
                    : build.generatedSides[station - 1][segment];
                const TopoDS_Face& b = station + 1 == receipt.stationCount ? build.lastCap
                    : build.generatedSides[station][segment];
                if (!proof_detail::OpposedUses(seam, a, b)) {
                    receipt.status = ProofStatus::ReorderedCorrespondence; return output;
                }
                ++receipt.seamCount;
            }

        TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
        TopExp::MapShapesAndAncestors(build.solid, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
        for (Standard_Integer i = 1; i <= edgeFaces.Extent(); ++i) {
            const TopTools_ListOfShape& uses = edgeFaces.FindFromIndex(i);
            if (uses.Extent() != 2) {
                receipt.status = ProofStatus::Disconnected; return output;
            }
            auto it = uses.cbegin(); const TopoDS_Face first = TopoDS::Face(*it++);
            const TopoDS_Face second = TopoDS::Face(*it);
            if (!proof_detail::OpposedUses(TopoDS::Edge(edgeFaces.FindKey(i)), first, second)) {
                receipt.status = ProofStatus::Disconnected; return output;
            }
        }

        // Face-adjacency connectivity is separate from shell/volume validity.
        std::set<int> visited; std::deque<int> pending{1};
        while (!pending.empty()) {
            const int current = pending.front(); pending.pop_front();
            if (!visited.insert(current).second) continue;
            for (TopExp_Explorer eit(faces(current), TopAbs_EDGE); eit.More(); eit.Next()) {
                const int ei = edgeFaces.FindIndex(eit.Current());
                if (ei <= 0) { receipt.status = ProofStatus::Inconclusive; return output; }
                for (const TopoDS_Shape& neighbour : edgeFaces.FindFromIndex(ei)) {
                    const int fi = faces.FindIndex(neighbour); if (fi > 0 && !visited.count(fi)) pending.push_back(fi);
                }
            }
        }
        if (visited.size() != std::size_t(faces.Extent())) {
            receipt.status = ProofStatus::Disconnected; return output;
        }

        if (!proof_detail::ReferenceRuledSamples(
                definition, build, cancelled, receipt.sectionSamples)) {
            receipt.status = cancelled.load() ? ProofStatus::Cancelled : ProofStatus::BadSection;
            return output;
        }

        // Independent continuous surface-pair bound. Adjacent faces are
        // allowed to meet only on their shared topological edge; every other
        // side pair must have a completed positive separation query.
        std::vector<TopoDS_Face> sideFaces;
        for (const auto& interval : build.generatedSides)
            sideFaces.insert(sideFaces.end(), interval.begin(), interval.end());
        for (std::size_t i = 0; i < sideFaces.size(); ++i)
            for (std::size_t j = i + 1; j < sideFaces.size(); ++j) {
                if (proof_detail::SharesBoundary(sideFaces[i], sideFaces[j])) continue;
                BRepExtrema_DistShapeShape distance(sideFaces[i], sideFaces[j]);
                if (!distance.IsDone() || distance.NbSolution() == 0
                    || !std::isfinite(distance.Value())) {
                    receipt.status = ProofStatus::Inconclusive; return output;
                }
                if (distance.Value() <= build.positionalTolerance) {
                    receipt.status = ProofStatus::SelfIntersection; return output;
                }
            }

        BOPAlgo_ArgumentAnalyzer interference;
        interference.SetShape1(build.solid); interference.SetRunParallel(Standard_False);
        interference.StopOnFirstFaulty() = Standard_True;
        interference.ArgumentTypeMode() = Standard_False;
        interference.SelfInterMode() = Standard_True;
        interference.SmallEdgeMode() = Standard_False;
        interference.RebuildFaceMode() = Standard_False;
        interference.TangentMode() = Standard_False;
        interference.MergeVertexMode() = Standard_False;
        interference.MergeEdgeMode() = Standard_False;
        interference.ContinuityMode() = Standard_False;
        interference.CurveOnSurfaceMode() = Standard_True;
        interference.Perform();
        if (interference.HasErrors() || interference.HasWarnings()
            || interference.HasFaulty()) {
            receipt.status = ProofStatus::SelfIntersection; return output;
        }

        GProp_GProps properties; BRepGProp::VolumeProperties(build.solid, properties, 1e-10);
        Bnd_Box box; BRepBndLib::Add(build.solid, box, Standard_False);
        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
        if (box.IsVoid()) { receipt.status = ProofStatus::Inconclusive; return output; }
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        receipt.volume = properties.Mass();
        receipt.conservativeUpperVolume = (xmax - xmin) * (ymax - ymin) * (zmax - zmin);
        if (!std::isfinite(receipt.volume) || !std::isfinite(receipt.conservativeUpperVolume)
            || receipt.volume <= 0 || receipt.volume > receipt.conservativeUpperVolume
                + 64 * build.positionalTolerance) {
            receipt.status = ProofStatus::BadMetric; return output;
        }
        if (cancelled.load(std::memory_order_relaxed)) {
            receipt.status = ProofStatus::Cancelled; return output;
        }
        receipt.status = ProofStatus::Admitted;
        output.solid = build.solid;
        return output;
    } catch (...) {
        receipt.status = ProofStatus::Inconclusive; return output;
    }
}

inline AdmittedSolid BuildAndProveDetached(const Definition& definition,
                                           const std::atomic_bool& cancelled) noexcept {
    const KernelBuild built = BuildDetached(definition, cancelled);
    return ProveDetached(definition, built, cancelled);
}
} // namespace core3d::general_loft
