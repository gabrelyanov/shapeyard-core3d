#pragma once

#include "RetainedEdgeTreatmentSnapshot.hxx"
#include "RetainedFaceSelectorValues.hxx"

#include <BRepAdaptor_Curve.hxx>
#include <ElCLib.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <atomic>
#include <limits>
#include <memory>
#include <set>

namespace core3d::retained_face_selector {
namespace et = core3d::retained_edge_treatment;

enum class Refusal : std::uint8_t {
    None = 0, InvalidIntent, FaceMissing, FaceAmbiguous, IncompleteBoundary,
    InvalidFaceUse, OwnershipMismatch, ExactCount, PartialCircle, StaleSource,
    UnsupportedTransform, SplitEdge, TruncatedDiscovery, SplineIdentityUnavailable,
    ReplayMismatch, UnsupportedCarrier, AdmissionUnavailable, Budget, Cancelled,
    Busy, NativeFailure, ReadOnlyInvariant
};

inline const char* RefusalCode(Refusal refusal) noexcept {
    static const char* values[] = {
        "b2.None", "b2.InvalidIntent", "b2.FaceMissing", "b2.FaceAmbiguous",
        "b2.IncompleteBoundary", "b2.InvalidFaceUse", "b2.OwnershipMismatch",
        "b2.ExactCount", "b2.PartialCircle", "b2.StaleSource", "b2.UnsupportedTransform",
        "b2.SplitEdge", "b2.TruncatedDiscovery", "b2.SplineIdentityUnavailable",
        "b2.ReplayMismatch", "b2.UnsupportedCarrier", "b2.AdmissionUnavailable",
        "b2.Budget", "b2.Cancelled", "b2.Busy", "b2.NativeFailure", "b2.ReadOnlyInvariant"
    };
    return values[std::size_t(refusal)];
}
inline const char* RefusalMessage(Refusal refusal) noexcept {
    static const char* values[] = {
        "",
        "The geometric selector is invalid. No model changes were made.",
        "The selector does not match a current planar face. No model changes were made.",
        "The selector matches more than one face. No model changes were made.",
        "The selector does not prove every boundary edge of one face. No model changes were made.",
        "A boundary edge has a seam, repeated use, or unsupported orientation. No model changes were made.",
        "A boundary edge does not have the required face ownership. No model changes were made.",
        "The proved selection does not have the required exact edge count. No model changes were made.",
        "A circle rim must be one complete closed circle. No model changes were made.",
        "The source changed after selector capture. Refresh the selection and try again. No model changes were made.",
        "This source transform is not supported for geometric selectors. No model changes were made.",
        "The selector matches split or ambiguous edges. No model changes were made.",
        "Native edge discovery is incomplete. No model changes were made.",
        "Authored spline edge identity is not available. No model changes were made.",
        "The saved selector no longer proves the same face and edges. No model changes were made.",
        "This source has no supported retained selector carrier. No model changes were made.",
        "Retained selector admission is not available for this source. No model changes were made.",
        "Geometric selector proof exceeded its bounded work limit. No model changes were made.",
        "Geometric selector proof was cancelled. No model changes were made.",
        "The model is busy. Try the selector again when the current operation finishes. No model changes were made.",
        "Native geometric selector proof failed. No model changes were made.",
        "Selector inspection failed its read-only integrity check."
    };
    return values[std::size_t(refusal)];
}

struct BoundaryUse {
    TopoDS_Wire wire;
    TopoDS_Edge edge;
    TopAbs_Orientation rawFaceUse = TopAbs_EXTERNAL;
    FaceUseDirection direction = FaceUseDirection::Forward;
    std::vector<TopoDS_Face> ownerFaces;
    bool selected = false;
};

struct Resolution;

class FaceMembershipProof final {
public:
    const SelectorIntent& intent() const noexcept { return intent_; }
    const TopoDS_Face& face() const noexcept { return face_; }
    const PlaneWitness& plane() const noexcept { return plane_; }
    const std::vector<BoundaryUse>& boundaryUses() const noexcept { return boundaryUses_; }
    std::uint32_t matchedFaceCount() const noexcept { return 1; }
    std::uint32_t wireCount() const noexcept { return wireCount_; }
    std::uint32_t uniqueEdgeCount() const noexcept { return uniqueEdgeCount_; }
    std::uint32_t selectedEdgeCount() const noexcept { return selectedEdgeCount_; }
    bool discoveryComplete() const noexcept { return true; }
    Coverage coverage() const noexcept { return coverage_; }
private:
    FaceMembershipProof() = default;
    SelectorIntent intent_;
    TopoDS_Face face_;
    PlaneWitness plane_;
    std::vector<BoundaryUse> boundaryUses_;
    std::uint32_t wireCount_ = 0, uniqueEdgeCount_ = 0, selectedEdgeCount_ = 0;
    Coverage coverage_ = Coverage::EntireBoundary;
    friend Refusal Resolve(const TopoDS_Shape&, const SelectorIntent&, double,
        et::ReplayBudget&, const std::atomic_bool&, Resolution&) noexcept;
};

struct Resolution {
    Refusal refusal = Refusal::NativeFailure;
    std::shared_ptr<const FaceMembershipProof> proof;
};

namespace detail {
inline double Component(const gp_Pnt& point, Axis axis) noexcept {
    if (axis == Axis::X) return point.X();
    if (axis == Axis::Y) return point.Y();
    return point.Z();
}
inline std::array<double, 3> Components(const gp_Dir& direction) noexcept {
    return {direction.X(), direction.Y(), direction.Z()};
}
inline gp_Dir RequestedNormal(const PlanarFaceScope& scope) {
    const double sign = scope.side == Side::Max ? 1.0 : -1.0;
    if (scope.axis == Axis::X) return gp_Dir(sign, 0, 0);
    if (scope.axis == Axis::Y) return gp_Dir(0, sign, 0);
    return gp_Dir(0, 0, sign);
}
inline bool SamePoint(const gp_Pnt& point, const std::array<double, 3>& expected, double localTolerance) {
    return point.Distance(gp_Pnt(expected[0], expected[1], expected[2])) <= localTolerance;
}
inline bool Parallel(const gp_Dir& first, const std::array<double, 3>& second, double tolerance = 1e-8) {
    return first.IsParallel(gp_Dir(second[0], second[1], second[2]), tolerance);
}
inline bool ScopeOf(const SelectorIntent& intent, PlanarFaceScope& output) {
    if (const auto* planar = std::get_if<PlanarFaceBoundary>(&intent)) { output = planar->face; return true; }
    if (const auto* line = std::get_if<Line>(&intent)) { output = line->face; return true; }
    return false;
}
inline bool Charge(et::ReplayBudget& budget, std::size_t visits) noexcept {
    return visits <= 4096 && budget.chargeStage(visits);
}
inline bool FacePlane(const TopoDS_Face& face, gp_Pln& plane) {
    BRepAdaptor_Surface surface(face, true);
    if (surface.GetType() != GeomAbs_Plane) return false;
    plane = surface.Plane();
    if (face.Orientation() == TopAbs_REVERSED) plane.SetAxis(plane.Axis().Reversed());
    return true;
}
} // namespace detail

inline Refusal Resolve(const TopoDS_Shape& inputStage, const SelectorIntent& intent,
    double metersPerLocalUnit, et::ReplayBudget& budget, const std::atomic_bool& cancelled,
    Resolution& output) noexcept {
    output = {};
    try {
        if (!ValidIntent(intent, true) || !std::isfinite(metersPerLocalUnit) || metersPerLocalUnit <= 0) {
            output.refusal = Refusal::InvalidIntent; return output.refusal;
        }
        if (std::holds_alternative<ReservedSplineEdge>(intent)) {
            output.refusal = Refusal::SplineIdentityUnavailable; return output.refusal;
        }
        if (inputStage.IsNull()) { output.refusal = Refusal::FaceMissing; return output.refusal; }
        if (cancelled.load()) { output.refusal = Refusal::Cancelled; return output.refusal; }
        const double localTolerance = 1e-4 / (1000.0 * metersPerLocalUnit);

        TopTools_IndexedMapOfShape faceMap;
        TopExp::MapShapes(inputStage, TopAbs_FACE, faceMap);
        if (!detail::Charge(budget, std::size_t(faceMap.Extent()))) {
            output.refusal = Refusal::Budget; return output.refusal;
        }
        std::vector<TopoDS_Face> candidates;
        PlanarFaceScope scope;
        if (detail::ScopeOf(intent, scope)) {
            double extreme = scope.side == Side::Max
                ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
            for (int index = 1; index <= faceMap.Extent(); ++index) {
                gp_Pln plane; if (!detail::FacePlane(TopoDS::Face(faceMap(index)), plane)) continue;
                if (!plane.Axis().Direction().IsParallel(detail::RequestedNormal(scope), 1e-8)) continue;
                const double coordinate = detail::Component(plane.Location(), scope.axis);
                extreme = scope.side == Side::Max ? std::max(extreme, coordinate) : std::min(extreme, coordinate);
            }
            if (!std::isfinite(extreme)) { output.refusal = Refusal::FaceMissing; return output.refusal; }
            for (int index = 1; index <= faceMap.Extent(); ++index) {
                gp_Pln plane; const TopoDS_Face face = TopoDS::Face(faceMap(index));
                if (!detail::FacePlane(face, plane)
                    || !plane.Axis().Direction().IsParallel(detail::RequestedNormal(scope), 1e-8)) continue;
                if (std::abs(detail::Component(plane.Location(), scope.axis) - extreme) <= localTolerance)
                    candidates.push_back(face);
            }
        } else {
            const auto& circle = std::get<CircleRim>(intent);
            const gp_Pnt center(circle.centerMM[0] / (1000.0 * metersPerLocalUnit),
                circle.centerMM[1] / (1000.0 * metersPerLocalUnit),
                circle.centerMM[2] / (1000.0 * metersPerLocalUnit));
            for (int index = 1; index <= faceMap.Extent(); ++index) {
                gp_Pln plane; const TopoDS_Face face = TopoDS::Face(faceMap(index));
                if (!detail::FacePlane(face, plane) || !detail::Parallel(plane.Axis().Direction(), circle.normal)
                    || plane.Distance(center) > localTolerance) continue;
                candidates.push_back(face);
            }
        }
        if (candidates.empty()) { output.refusal = Refusal::FaceMissing; return output.refusal; }
        if (candidates.size() > 1) { output.refusal = Refusal::FaceAmbiguous; return output.refusal; }

        auto proof = std::shared_ptr<FaceMembershipProof>(new FaceMembershipProof);
        proof->intent_ = intent; proof->face_ = candidates.front();
        gp_Pln selectedPlane; detail::FacePlane(proof->face_, selectedPlane);
        proof->plane_.outwardNormal = detail::Components(selectedPlane.Axis().Direction());
        proof->plane_.offsetMM = selectedPlane.Axis().Direction().XYZ().Dot(selectedPlane.Location().XYZ())
            * 1000.0 * metersPerLocalUnit;

        TopTools_IndexedDataMapOfShapeListOfShape owners;
        TopExp::MapShapesAndAncestors(inputStage, TopAbs_EDGE, TopAbs_FACE, owners);
        TopTools_IndexedMapOfShape uniqueEdges;
        std::set<int> usesOnFace;
        for (TopExp_Explorer wireExplorer(proof->face_, TopAbs_WIRE); wireExplorer.More(); wireExplorer.Next()) {
            if (cancelled.load()) { output.refusal = Refusal::Cancelled; return output.refusal; }
            const TopoDS_Wire wire = TopoDS::Wire(wireExplorer.Current());
            ++proof->wireCount_;
            std::size_t explored = 0;
            for (BRepTools_WireExplorer edgeExplorer(wire, proof->face_); edgeExplorer.More(); edgeExplorer.Next()) {
                ++explored;
                const TopoDS_Edge edge = edgeExplorer.Current();
                const TopAbs_Orientation orientation = edge.Orientation();
                if (orientation != TopAbs_FORWARD && orientation != TopAbs_REVERSED) {
                    output.refusal = Refusal::InvalidFaceUse; return output.refusal;
                }
                const int edgeIndex = owners.FindIndex(edge);
                if (edgeIndex <= 0 || !usesOnFace.insert(edgeIndex).second || BRep_Tool::Degenerated(edge)) {
                    output.refusal = Refusal::InvalidFaceUse; return output.refusal;
                }
                BoundaryUse use; use.wire = wire; use.edge = edge; use.rawFaceUse = orientation;
                use.direction = orientation == TopAbs_FORWARD ? FaceUseDirection::Forward : FaceUseDirection::Reversed;
                const TopTools_ListOfShape& ancestors = owners.FindFromIndex(edgeIndex);
                for (TopTools_ListIteratorOfListOfShape iterator(ancestors); iterator.More(); iterator.Next()) {
                    const TopoDS_Face owner = TopoDS::Face(iterator.Value());
                    if (std::none_of(use.ownerFaces.begin(), use.ownerFaces.end(), [&](const TopoDS_Face& prior) {
                        return prior.IsSame(owner);
                    })) use.ownerFaces.push_back(owner);
                }
                if (use.ownerFaces.size() != 2 || std::none_of(use.ownerFaces.begin(), use.ownerFaces.end(),
                    [&](const TopoDS_Face& owner) { return owner.IsSame(proof->face_); })) {
                    output.refusal = Refusal::OwnershipMismatch; return output.refusal;
                }
                uniqueEdges.Add(edge); proof->boundaryUses_.push_back(std::move(use));
            }
            std::size_t direct = 0;
            for (TopExp_Explorer directExplorer(wire, TopAbs_EDGE); directExplorer.More(); directExplorer.Next()) ++direct;
            if (explored == 0 || explored != direct) { output.refusal = Refusal::IncompleteBoundary; return output.refusal; }
        }
        if (proof->wireCount_ == 0 || proof->wireCount_ > 64 || proof->boundaryUses_.size() > 64) {
            output.refusal = proof->boundaryUses_.size() > 64 ? Refusal::TruncatedDiscovery : Refusal::IncompleteBoundary;
            return output.refusal;
        }
        proof->uniqueEdgeCount_ = std::uint32_t(uniqueEdges.Extent());
        if (proof->uniqueEdgeCount_ != proof->boundaryUses_.size()) {
            output.refusal = Refusal::InvalidFaceUse; return output.refusal;
        }

        if (const auto* line = std::get_if<Line>(&intent)) {
            proof->coverage_ = Coverage::SelectedLine;
            const gp_Pnt witness(line->locationMM[0] / (1000.0 * metersPerLocalUnit),
                line->locationMM[1] / (1000.0 * metersPerLocalUnit),
                line->locationMM[2] / (1000.0 * metersPerLocalUnit));
            for (auto& use : proof->boundaryUses_) {
                BRepAdaptor_Curve curve(use.edge);
                if (curve.GetType() != GeomAbs_Line || !detail::Parallel(curve.Line().Direction(), line->direction)) continue;
                const double parameter = ElCLib::Parameter(curve.Line(), witness);
                if (parameter < curve.FirstParameter() - localTolerance || parameter > curve.LastParameter() + localTolerance
                    || curve.Line().Distance(witness) > localTolerance) continue;
                use.selected = true; ++proof->selectedEdgeCount_;
            }
            if (proof->selectedEdgeCount_ == 0) { output.refusal = Refusal::FaceMissing; return output.refusal; }
            if (proof->selectedEdgeCount_ > 1) { output.refusal = Refusal::SplitEdge; return output.refusal; }
        } else if (const auto* circle = std::get_if<CircleRim>(&intent)) {
            proof->coverage_ = Coverage::EntireBoundary;
            if (proof->wireCount_ != 1 || proof->boundaryUses_.size() != 1) {
                output.refusal = Refusal::PartialCircle; return output.refusal;
            }
            auto& use = proof->boundaryUses_.front(); BRepAdaptor_Curve curve(use.edge);
            if (curve.GetType() != GeomAbs_Circle || !curve.IsClosed()
                || std::abs((curve.LastParameter() - curve.FirstParameter()) - 2.0 * M_PI) > 1e-8
                || curve.Circle().Location().Distance(gp_Pnt(circle->centerMM[0] / (1000.0 * metersPerLocalUnit),
                    circle->centerMM[1] / (1000.0 * metersPerLocalUnit),
                    circle->centerMM[2] / (1000.0 * metersPerLocalUnit))) > localTolerance
                || std::abs(curve.Circle().Radius() * 1000.0 * metersPerLocalUnit - circle->radiusMM) > 1e-4) {
                output.refusal = Refusal::PartialCircle; return output.refusal;
            }
            use.selected = true; proof->selectedEdgeCount_ = 1;
        } else {
            proof->coverage_ = Coverage::EntireBoundary;
            for (auto& use : proof->boundaryUses_) {
                if (BRepAdaptor_Curve(use.edge).GetType() != GeomAbs_Line) {
                    output.refusal = Refusal::IncompleteBoundary; return output.refusal;
                }
                use.selected = true; ++proof->selectedEdgeCount_;
            }
        }
        if (proof->selectedEdgeCount_ != ExpectedCount(intent)) {
            output.refusal = Refusal::ExactCount; return output.refusal;
        }
        output.proof = std::move(proof); output.refusal = Refusal::None; return output.refusal;
    } catch (...) { output = {}; output.refusal = Refusal::NativeFailure; return output.refusal; }
}

inline Refusal VerifyReceipt(const FaceMembershipProof& proof, const et::Step& step,
    double, et::ReplayBudget&) noexcept {
    if (!step.selector || !ValidReceipt(*step.selector)) return Refusal::ReplayMismatch;
    const auto& receipt = *step.selector;
    if (!(receipt.intent == proof.intent()) || !(receipt.face == proof.plane())
        || receipt.coverage != proof.coverage() || receipt.wireCount != proof.wireCount()
        || receipt.boundaryUseCount != proof.boundaryUses().size()
        || receipt.boundaryUniqueEdgeCount != proof.uniqueEdgeCount()
        || receipt.entries.size() != step.anchors.size()) return Refusal::ReplayMismatch;
    for (std::size_t index = 0; index < step.anchors.size(); ++index) {
        if (receipt.entries[index].anchorKey != step.anchors[index].key) return Refusal::ReplayMismatch;
    }
    return Refusal::None;
}

inline et::Refusal MapToB1(Refusal refusal) noexcept {
    using B1 = et::Refusal;
    switch (refusal) {
    case Refusal::None: return B1::None;
    case Refusal::FaceMissing: case Refusal::IncompleteBoundary: case Refusal::ExactCount: return B1::AnchorMissing;
    case Refusal::FaceAmbiguous: case Refusal::SplitEdge: return B1::AnchorAmbiguous;
    case Refusal::InvalidFaceUse: case Refusal::OwnershipMismatch: case Refusal::PartialCircle:
    case Refusal::SplineIdentityUnavailable: return B1::UnsupportedEdge;
    case Refusal::ReplayMismatch: return B1::ReplayMismatch;
    case Refusal::InvalidIntent: return B1::MalformedCarrier;
    case Refusal::UnsupportedCarrier: case Refusal::AdmissionUnavailable: return B1::UnsupportedBase;
    case Refusal::StaleSource: return B1::StaleSnapshot;
    case Refusal::UnsupportedTransform: return B1::UnsupportedTransform;
    case Refusal::Budget: case Refusal::TruncatedDiscovery: return B1::Budget;
    case Refusal::Cancelled: return B1::Cancelled;
    case Refusal::Busy: return B1::Busy;
    default: return B1::BuildFailed;
    }
}
} // namespace core3d::retained_face_selector
