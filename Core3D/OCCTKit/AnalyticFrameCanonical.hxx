#pragma once

// D1054/D1087 (21 Oct 2026): canonical analytic frames on newly owned R2
// geometry. The same finite construction is applied in place both to the
// framed-polygon source before its existing private-source materializer and
// to the analytic-prefix result before publication and the D981 certificate.
//
// The exact detachment commitment (RetainedEdgeTreatmentBuild.hxx :246-260 /
// :367-374) compares the published prefix shape with its bounded BinTools V4
// readback byte-for-byte through the mesh-independent V3 serialisation. The
// V4 read phase reconstructs every analytic frame in the prebuilt kernel:
// directions pass through gp_Dir::SetCoord (gp_Dir.hxx :367-375, the same
// arithmetic as the constructor at :322-330), circle/ellipse frames through
// the gp_Ax2(P, N, Vx) constructor (gp_Ax2.hxx :71-79, the stored Y is
// discarded and re-derived), plane/cylinder frames through gp_Ax3(P, N, Vx)
// (gp_Ax3.hxx :82-88) plus the stored-Y DotCross reversal rule
// (BinTools_SurfaceSet ReadAx3). That reconstruction is not a fixed point
// for every produced frame: NP7B-DIAG measured 3D line directions, ellipse
// axes and plane frames wandering in their last bits across reads (largest
// observed component delta 2.220446049250313e-16; typical axis-noise
// ~1e-17), so a freshly built moved prefix legitimately failed its own
// detachment commitment with C(S) != C(R(S)).
//
// Canonicalisation here rebuilds every analytic frame of the private result
// through ONE deterministic routine: a candidate frame is (a) re-normalised
// through the read-side path, (b) may flush a candidate component whose
// magnitude is below a fixed absolute epsilon to signed-zero-normalised +0
// (see below), and (c) is accepted only when one genuine V4 read of exactly
// that stored frame reproduces every stored component bit-for-bit. The
// acceptance read is performed by the same prebuilt
// BinTools_CurveSet/BinTools_SurfaceSet single-object write/read entry
// points the detachment readback links against — literally the same gp_Dir/
// gp_Ax2/gp_Ax3 constructor path with the same compiled arithmetic, so no
// reimplementation of that arithmetic can drift from the gate. Because the
// reader map is not a contraction (NP7B-DIAG measured 2-cycles), unstable
// directions first receive a deterministic 1e-14 component seed; candidates
// are then tried in a fixed order: the reader's output for that seed, its
// zero-flushed form, then ulp-perturbation shells of growing radius (raw and
// zero-flushed). The first bitwise fixed point wins, so the routine is
// idempotent under readback: R(canon(x)) == canon(x)
// bit-for-bit. A frame that is already readback-stable (every frame saved
// by an older document after a V4 read, and every axis-aligned frame) is its
// own first candidate and is left untouched, so already-saved geometry is
// unchanged by this pass. The seed moves an unstable direction component by
// at most half the fixed epsilon before normalisation: topology,
// correspondence, origins, radii and physical meaning are unchanged. No
// stored component is ever flushed after derivation —
// derived axes are exactly what the reader re-derives; the flush only
// shapes search candidates.
//
// Flush epsilon justification: the measured reconstruction noise on unit
// direction components is at most one ulp at magnitude ~1
// (2.220446049250313e-16), with oscillating derived components ~1e-17.
// ComponentFlushEpsilon = 1e-14 sits two orders of magnitude above that
// noise band and far below any modeling-meaningful component (kernel
// resolution is ~1e-7 mm); acceptance is always gated on the exact bitwise
// readback, so the value only influences which representative is tried,
// never whether it is accepted.
//
// Budget: one bounded census of the already-private source or prefix result
// without a stage debit plus one reserved traversal covering the replacement pass —
// the ReserveReadReconstruction pattern from the sibling producer header —
// and pure per-frame single-object probes, which are arithmetic on private
// geometry, not topology passes. No whole-shape readback is added (zero
// materialisations; the unchanged detachment gate remains the sole
// whole-shape verifier). Charges are not reset and cancellation/budget
// exhaustion propagate.

#include "RetainedTopologyBudget.hxx"

#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BinTools_CurveSet.hxx>
#include <BinTools_OStream.hxx>
#include <BinTools_SurfaceSet.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <TopLoc_Location.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace core3d::analytic_frame_canonical {
namespace tb = core3d::retained_topology_budget;

enum class Status : std::uint8_t {
    KeptOriginal = 0,  // no frame needed (or accepted) a change; input untouched
    Adopted,           // at least one analytic frame was canonicalised in place
    Cancelled,
    BudgetExceeded
};

// See the header comment: two orders of magnitude above the measured
// readback noise band, far below any modeling-meaningful component.
inline constexpr double ComponentFlushEpsilon = 1e-14;
// Bounded deterministic search neighbourhood around the reader output.
inline constexpr int MaximumUlpSearchRadius = 4;

namespace detail {

inline bool SameBits(Standard_Real lhs, Standard_Real rhs) noexcept {
    return std::memcmp(&lhs, &rhs, sizeof(Standard_Real)) == 0;
}
inline bool SameBitsOrSignedZero(Standard_Real lhs, Standard_Real rhs) noexcept {
    return (lhs == 0.0 && rhs == 0.0) || SameBits(lhs, rhs);
}
inline bool SameDirection(const gp_Dir& lhs, const gp_Dir& rhs) noexcept {
    return SameBits(lhs.X(), rhs.X()) && SameBits(lhs.Y(), rhs.Y())
        && SameBits(lhs.Z(), rhs.Z());
}
inline bool SameDirectionOrSignedZero(const gp_Dir& lhs,
    const gp_Dir& rhs) noexcept {
    return SameBitsOrSignedZero(lhs.X(), rhs.X())
        && SameBitsOrSignedZero(lhs.Y(), rhs.Y())
        && SameBitsOrSignedZero(lhs.Z(), rhs.Z());
}
inline bool SameFrame(const gp_Ax3& lhs, const gp_Ax3& rhs) noexcept {
    return SameBits(lhs.Location().X(), rhs.Location().X())
        && SameBits(lhs.Location().Y(), rhs.Location().Y())
        && SameBits(lhs.Location().Z(), rhs.Location().Z())
        && SameDirection(lhs.Direction(), rhs.Direction())
        && SameDirection(lhs.XDirection(), rhs.XDirection())
        && SameDirection(lhs.YDirection(), rhs.YDirection());
}
inline bool SameFrameOrSignedZero(const gp_Ax3& lhs, const gp_Ax3& rhs) noexcept {
    return SameBitsOrSignedZero(lhs.Location().X(), rhs.Location().X())
        && SameBitsOrSignedZero(lhs.Location().Y(), rhs.Location().Y())
        && SameBitsOrSignedZero(lhs.Location().Z(), rhs.Location().Z())
        && SameDirectionOrSignedZero(lhs.Direction(), rhs.Direction())
        && SameDirectionOrSignedZero(lhs.XDirection(), rhs.XDirection())
        && SameDirectionOrSignedZero(lhs.YDirection(), rhs.YDirection());
}
inline bool IsSignedCoordinateAxis(const gp_Dir& direction) noexcept {
    const auto axisComponent = [](Standard_Real value) {
        return value == -1.0 || value == 0.0 || value == 1.0;
    };
    if (!axisComponent(direction.X()) || !axisComponent(direction.Y())
        || !axisComponent(direction.Z())) return false;
    const int nonzero = (direction.X() != 0.0 ? 1 : 0)
        + (direction.Y() != 0.0 ? 1 : 0)
        + (direction.Z() != 0.0 ? 1 : 0);
    return nonzero == 1;
}
inline bool IsSignedCoordinateFrame(const gp_Dir& normal,
    const gp_Dir& xDirection) noexcept {
    return IsSignedCoordinateAxis(normal)
        && IsSignedCoordinateAxis(xDirection);
}

inline Standard_Real FlushComponent(Standard_Real value) noexcept {
    // Signed-zero-normalised flush: any sub-epsilon component becomes +0.0.
    return std::fabs(value) < ComponentFlushEpsilon ? 0.0 : value;
}
inline Standard_Real CanonicalSeedComponent(Standard_Real value) noexcept {
    if (std::fabs(value) < ComponentFlushEpsilon) return 0.0;
    return std::round(value / ComponentFlushEpsilon)
        * ComponentFlushEpsilon;
}
inline gp_Dir CanonicalSeedDirection(const gp_Dir& direction) {
    gp_Dir seed(1., 0., 0.);
    seed.SetCoord(CanonicalSeedComponent(direction.X()),
        CanonicalSeedComponent(direction.Y()),
        CanonicalSeedComponent(direction.Z()));
    return seed;
}
inline Standard_Real StepUlps(Standard_Real value, int steps) noexcept {
    const Standard_Real direction = steps > 0 ? HUGE_VAL : -HUGE_VAL;
    for (int i = 0, n = steps > 0 ? steps : -steps; i < n; ++i)
        value = std::nextafter(value, direction);
    return value;
}

// One genuine V4 read of a single curve/surface through the prebuilt
// BinTools sets — the same compiled reconstruction the whole-shape
// detachment readback performs per primitive.
inline Handle(Geom_Curve) ReadbackCurve(const Handle(Geom_Curve)& curve) {
    std::ostringstream buffer(std::ios::binary);
    {
        BinTools_OStream out(buffer);
        BinTools_CurveSet::WriteCurve(curve, out);
    }
    std::istringstream input(buffer.str(), std::ios::binary);
    Handle(Geom_Curve) read;
    BinTools_CurveSet::ReadCurve(input, read);
    return read;
}
inline Handle(Geom_Surface) ReadbackSurface(const Handle(Geom_Surface)& surface) {
    std::ostringstream buffer(std::ios::binary);
    {
        BinTools_OStream out(buffer);
        BinTools_SurfaceSet::WriteSurface(surface, out);
    }
    std::istringstream input(buffer.str(), std::ios::binary);
    Handle(Geom_Surface) read;
    BinTools_SurfaceSet::ReadSurface(input, read);
    return read;
}

// The read-side direction reconstruction measured on the real reader: a
// Geom_Line read is exactly SetCoord on the stored components.
inline gp_Dir ReadbackDirection(const gp_Dir& stored) {
    const Handle(Geom_Line) probe = Handle(Geom_Line)::DownCast(
        ReadbackCurve(new Geom_Line(gp_Pnt(0., 0., 0.), stored)));
    return probe.IsNull() ? stored : probe->Lin().Direction();
}

// Deterministic candidate sequence: the base itself, its zero-flushed form,
// then ulp-perturbation shells of growing radius (raw and zero-flushed) in a
// fixed lexicographic order. accept() claims the first bitwise fixed point.
template <typename Accept>
inline bool ForEachDirectionCandidate(const gp_Dir& base, Accept&& accept) {
    const Standard_Real bx = base.X(), by = base.Y(), bz = base.Z();
    gp_Dir candidate(1., 0., 0.);
    if (accept(base)) return true;
    candidate.SetCoord(FlushComponent(bx), FlushComponent(by), FlushComponent(bz));
    if (accept(candidate)) return true;
    for (int radius = 1; radius <= MaximumUlpSearchRadius; ++radius) {
        for (int dx = -radius; dx <= radius; ++dx)
        for (int dy = -radius; dy <= radius; ++dy)
        for (int dz = -radius; dz <= radius; ++dz) {
            if (std::max(std::max(dx < 0 ? -dx : dx, dy < 0 ? -dy : dy),
                    dz < 0 ? -dz : dz) != radius)
                continue;
            const Standard_Real cx = StepUlps(bx, dx);
            const Standard_Real cy = StepUlps(by, dy);
            const Standard_Real cz = StepUlps(bz, dz);
            candidate.SetCoord(cx, cy, cz);
            if (accept(candidate)) return true;
            candidate.SetCoord(FlushComponent(cx), FlushComponent(cy),
                FlushComponent(cz));
            if (accept(candidate)) return true;
        }
    }
    return false;
}

// A stored direction is canonical exactly when one real read reproduces it
// bit-for-bit.
inline bool MakeReadbackFixedDirection(const gp_Dir& stored, gp_Dir& fixed) {
    const bool ok = ForEachDirectionCandidate(
        ReadbackDirection(CanonicalSeedDirection(stored)),
        [&](const gp_Dir& candidate) {
            if (!SameDirection(ReadbackDirection(candidate), candidate))
                return false;
            fixed = candidate;
            return true;
        });
    return ok;
}

// Circle/ellipse frames: the V4 read discards the stored Y and rebuilds the
// frame through gp_Ax2(P, N, Vx). A candidate is built through that same
// constructor and accepted only when one real read of exactly that stored
// frame reproduces it bit-for-bit (derived Y included).
inline bool CanonicaliseAx2Frame(const gp_Pnt& location, const gp_Dir& storedN,
    const gp_Dir& storedX, gp_Ax2& canonical) {
    gp_Dir n(1., 0., 0.);
    if (!MakeReadbackFixedDirection(storedN, n)) return false;
    // The reader's own output for the stored frame is the search base.
    const Handle(Geom_Circle) seedProbe = Handle(Geom_Circle)::DownCast(
        ReadbackCurve(new Geom_Circle(
            gp_Ax2(location, n, CanonicalSeedDirection(storedX)), 1.)));
    if (seedProbe.IsNull()) return false;
    const gp_Dir base = seedProbe->Circ().XAxis().Direction();
    bool found = false;
    ForEachDirectionCandidate(base, [&](const gp_Dir& candidate) {
        const gp_Ax2 trial(location, n, candidate);
        const Handle(Geom_Circle) read = Handle(Geom_Circle)::DownCast(
            ReadbackCurve(new Geom_Circle(trial, 1.)));
        if (read.IsNull() || !SameFrame(read->Circ().Position(), trial))
            return false;
        canonical = trial;
        found = true;
        return true;
    });
    return found;
}

// Plane/cylinder frames: the V4 read rebuilds through gp_Ax3(P, N, Vx) plus
// the stored-Y DotCross reversal rule; the real probe includes both.
inline bool CanonicaliseAx3Frame(const gp_Pnt& location, const gp_Dir& storedN,
    const gp_Dir& storedX, gp_Ax3& canonical) {
    gp_Dir n(1., 0., 0.);
    if (!MakeReadbackFixedDirection(storedN, n)) return false;
    const Handle(Geom_Plane) seedProbe = Handle(Geom_Plane)::DownCast(
        ReadbackSurface(new Geom_Plane(
            gp_Ax3(location, n, CanonicalSeedDirection(storedX)))));
    if (seedProbe.IsNull()) return false;
    const gp_Dir base = seedProbe->Pln().XAxis().Direction();
    bool found = false;
    ForEachDirectionCandidate(base, [&](const gp_Dir& candidate) {
        const gp_Ax3 trial(location, n, candidate);
        const Handle(Geom_Plane) read = Handle(Geom_Plane)::DownCast(
            ReadbackSurface(new Geom_Plane(trial)));
        if (read.IsNull() || !SameFrame(read->Pln().Position(), trial))
            return false;
        canonical = trial;
        found = true;
        return true;
    });
    return found;
}

// Returns the canonical replacement curve, or a null handle when the stored
// geometry is already canonical (or carries no V4-reconstructed frame).
inline Handle(Geom_Curve) CanonicaliseCurve(const Handle(Geom_Curve)& curve) {
    if (curve.IsNull()) return nullptr;
    if (const Handle(Geom_Line) line = Handle(Geom_Line)::DownCast(curve)) {
        const gp_Lin geometric = line->Lin();
        if (IsSignedCoordinateAxis(geometric.Direction())) return nullptr;
        const Handle(Geom_Line) read = Handle(Geom_Line)::DownCast(
            ReadbackCurve(curve));
        if (!read.IsNull() && SameDirection(read->Lin().Direction(),
                geometric.Direction())) return nullptr;
        gp_Dir direction(1., 0., 0.);
        if (!MakeReadbackFixedDirection(geometric.Direction(), direction))
            return nullptr;
        // A V4 probe can normalise -0 to +0 even when the complete stored
        // shape is already an exact V3 fixed point. Signed zero is still part
        // of the commitment, so preserve the producer's original bits.
        if (SameDirectionOrSignedZero(direction, geometric.Direction()))
            return nullptr;
        return new Geom_Line(geometric.Location(), direction);
    }
    if (const Handle(Geom_Circle) circle = Handle(Geom_Circle)::DownCast(curve)) {
        const gp_Circ geometric = circle->Circ();
        if (IsSignedCoordinateFrame(geometric.Axis().Direction(),
                geometric.XAxis().Direction())) return nullptr;
        const Handle(Geom_Circle) read = Handle(Geom_Circle)::DownCast(
            ReadbackCurve(curve));
        if (!read.IsNull() && SameFrame(read->Circ().Position(),
                geometric.Position())) return nullptr;
        gp_Ax2 frame;
        if (!CanonicaliseAx2Frame(geometric.Location(), geometric.Axis().Direction(),
                geometric.XAxis().Direction(), frame))
            return nullptr;
        if (SameFrameOrSignedZero(frame, geometric.Position())) return nullptr;
        return new Geom_Circle(gp_Circ(frame, geometric.Radius()));
    }
    if (const Handle(Geom_Ellipse) ellipse = Handle(Geom_Ellipse)::DownCast(curve)) {
        const gp_Elips geometric = ellipse->Elips();
        if (IsSignedCoordinateFrame(geometric.Axis().Direction(),
                geometric.XAxis().Direction())) return nullptr;
        const Handle(Geom_Ellipse) read = Handle(Geom_Ellipse)::DownCast(
            ReadbackCurve(curve));
        if (!read.IsNull() && SameFrame(read->Elips().Position(),
                geometric.Position())) return nullptr;
        gp_Ax2 frame;
        if (!CanonicaliseAx2Frame(geometric.Location(), geometric.Axis().Direction(),
                geometric.XAxis().Direction(), frame))
            return nullptr;
        if (SameFrameOrSignedZero(frame, geometric.Position())) return nullptr;
        return new Geom_Ellipse(gp_Elips(frame, geometric.MajorRadius(),
            geometric.MinorRadius()));
    }
    if (const Handle(Geom_TrimmedCurve) trimmed
        = Handle(Geom_TrimmedCurve)::DownCast(curve)) {
        const Handle(Geom_Curve) basis = CanonicaliseCurve(trimmed->BasisCurve());
        if (basis.IsNull()) return nullptr;
        return new Geom_TrimmedCurve(basis, trimmed->FirstParameter(),
            trimmed->LastParameter());
    }
    return nullptr;
}

// Returns the canonical replacement surface, or a null handle when the
// stored geometry is already canonical (or carries no V4-reconstructed
// frame).
inline Handle(Geom_Surface) CanonicaliseSurface(const Handle(Geom_Surface)& surface) {
    if (surface.IsNull()) return nullptr;
    if (const Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surface)) {
        const gp_Pln geometric = plane->Pln();
        if (IsSignedCoordinateFrame(geometric.Axis().Direction(),
                geometric.XAxis().Direction())) return nullptr;
        const Handle(Geom_Plane) read = Handle(Geom_Plane)::DownCast(
            ReadbackSurface(surface));
        if (!read.IsNull() && SameFrame(read->Pln().Position(),
                geometric.Position())) return nullptr;
        gp_Ax3 frame;
        if (!CanonicaliseAx3Frame(geometric.Location(), geometric.Axis().Direction(),
                geometric.XAxis().Direction(), frame))
            return nullptr;
        if (SameFrameOrSignedZero(frame, geometric.Position())) return nullptr;
        return new Geom_Plane(frame);
    }
    if (const Handle(Geom_CylindricalSurface) cylinder
        = Handle(Geom_CylindricalSurface)::DownCast(surface)) {
        const gp_Cylinder geometric = cylinder->Cylinder();
        if (IsSignedCoordinateFrame(geometric.Axis().Direction(),
                geometric.XAxis().Direction())) return nullptr;
        const Handle(Geom_CylindricalSurface) read
            = Handle(Geom_CylindricalSurface)::DownCast(
                ReadbackSurface(surface));
        if (!read.IsNull() && SameFrame(read->Cylinder().Position(),
                geometric.Position())) return nullptr;
        gp_Ax3 frame;
        if (!CanonicaliseAx3Frame(geometric.Location(), geometric.Axis().Direction(),
                geometric.XAxis().Direction(), frame))
            return nullptr;
        if (SameFrameOrSignedZero(frame, geometric.Position())) return nullptr;
        return new Geom_CylindricalSurface(frame, geometric.Radius());
    }
    if (const Handle(Geom_RectangularTrimmedSurface) trimmed
        = Handle(Geom_RectangularTrimmedSurface)::DownCast(surface)) {
        const Handle(Geom_Surface) basis = CanonicaliseSurface(trimmed->BasisSurface());
        if (basis.IsNull()) return nullptr;
        Standard_Real u1 = 0, u2 = 0, v1 = 0, v2 = 0;
        trimmed->Bounds(u1, u2, v1, v2);
        return new Geom_RectangularTrimmedSurface(basis, u1, u2, v1, v2);
    }
    return nullptr;
}

} // namespace detail

// Canonicalises every V4-reconstructed analytic frame of a still-private
// newly owned source or analytic-prefix result in place. One bounded census
// without a stage debit (the ReserveReadReconstruction pattern) supplies the
// face/edge maps; one reserved traversal covers the replacement pass;
// replacement geometry is computed completely before any mutation, and
// shared handles keep one shared canonical replacement via the memo tables.
inline Status CanonicalisePrivateAnalyticPrefixFrames(const TopoDS_Shape& shape,
    const std::atomic_bool& stop, tb::Counter& budget) noexcept {
    if (stop.load()) return Status::Cancelled;
    if (budget.exhausted) return Status::BudgetExceeded;
    if (shape.IsNull()) return Status::KeptOriginal;
    try {
        tb::Census census;
        const tb::WalkStatus walked = tb::CensusTopology(shape, budget, stop,
            census, tb::Site::C19R2Prefix, false);
        if (walked == tb::WalkStatus::Cancelled || stop.load())
            return Status::Cancelled;
        if (walked == tb::WalkStatus::BudgetDenied || budget.exhausted)
            return Status::BudgetExceeded;
        if (walked != tb::WalkStatus::Completed) return Status::KeptOriginal;
        // Reserve the replacement pass over the same shape up front; the
        // reserved pass is not charged again below.
        if (!tb::ReserveTraversal(census, budget, tb::Site::C19R2Prefix))
            return Status::BudgetExceeded;

        struct FaceUpdate {
            TopoDS_Face face;
            Handle(Geom_Surface) oldSurface;
            Handle(Geom_Surface) surface;
            TopLoc_Location location;
            Standard_Real tolerance;
        };
        struct EdgeUpdate {
            TopoDS_Edge edge;
            Handle(Geom_Curve) curve;
            TopLoc_Location location;
            Standard_Real tolerance;
        };
        struct PcurveUpdate {
            TopoDS_Edge edge;
            Handle(Geom_Surface) oldSurface;
            Handle(Geom_Surface) newSurface;
            TopLoc_Location location;
            Handle(Geom2d_Curve) firstCurve;
            Handle(Geom2d_Curve) secondCurve;
            Standard_Real first = 0;
            Standard_Real last = 0;
            Standard_Real tolerance = 0;
        };
        std::vector<FaceUpdate> faceUpdates;
        std::vector<EdgeUpdate> edgeUpdates;
        std::vector<PcurveUpdate> pcurveUpdates;
        std::unordered_map<const void*, Handle(Geom_Surface)> surfaceMemo;
        std::unordered_map<const void*, Handle(Geom_Curve)> curveMemo;

        for (Standard_Integer i = 1; i <= census.faces.Extent(); ++i) {
            const TopoDS_Face face = TopoDS::Face(census.faces.FindKey(i));
            TopLoc_Location location;
            const Handle(Geom_Surface) surface = BRep_Tool::Surface(face, location);
            if (surface.IsNull()) continue;
            const void* key = surface.get();
            auto found = surfaceMemo.find(key);
            if (found == surfaceMemo.end())
                found = surfaceMemo.emplace(key,
                    detail::CanonicaliseSurface(surface)).first;
            if (!found->second.IsNull())
                faceUpdates.push_back(
                    {face, surface, found->second, location,
                        BRep_Tool::Tolerance(face)});
        }
        for (Standard_Integer i = 1; i <= census.edges.Extent(); ++i) {
            const TopoDS_Edge edge = TopoDS::Edge(census.edges.FindKey(i));
            TopLoc_Location location;
            Standard_Real first = 0, last = 0;
            const Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, location,
                first, last);
            if (curve.IsNull()) continue;
            const void* key = curve.get();
            auto found = curveMemo.find(key);
            if (found == curveMemo.end())
                found = curveMemo.emplace(key,
                    detail::CanonicaliseCurve(curve)).first;
            if (!found->second.IsNull())
                edgeUpdates.push_back(
                    {edge, found->second, location, BRep_Tool::Tolerance(edge)});
        }
        // A curve-on-surface representation is keyed by the surface handle.
        // Capture every stored pcurve before replacing a face surface, then
        // move that representation to the replacement handle. Otherwise the
        // old surface remains reachable from the edges and the face can fail
        // BRepCheck even though its new surface is locally valid.
        for (const FaceUpdate& faceUpdate : faceUpdates) {
            for (TopExp_Explorer explorer(faceUpdate.face, TopAbs_EDGE);
                explorer.More(); explorer.Next()) {
                const TopoDS_Edge edge = TopoDS::Edge(explorer.Current());
                Standard_Real first = 0, last = 0;
                Standard_Boolean stored = Standard_False;
                const Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(
                    edge, faceUpdate.face, first, last, &stored);
                if (!stored || pcurve.IsNull()) continue;
                auto found = std::find_if(pcurveUpdates.begin(),
                    pcurveUpdates.end(), [&](const PcurveUpdate& update) {
                        return update.edge.IsSame(edge)
                            && update.oldSurface == faceUpdate.oldSurface
                            && update.newSurface == faceUpdate.surface;
                    });
                if (found == pcurveUpdates.end()) {
                    PcurveUpdate update;
                    update.edge = edge;
                    update.oldSurface = faceUpdate.oldSurface;
                    update.newSurface = faceUpdate.surface;
                    update.firstCurve = pcurve;
                    update.first = first;
                    update.last = last;
                    update.tolerance = BRep_Tool::Tolerance(edge);
                    pcurveUpdates.push_back(std::move(update));
                } else if (found->firstCurve != pcurve
                    && found->secondCurve.IsNull()) {
                    found->secondCurve = pcurve;
                }
            }
        }
        if (faceUpdates.empty() && edgeUpdates.empty()) return Status::KeptOriginal;
        if (stop.load()) return Status::Cancelled;

        BRep_Builder builder;
        for (const PcurveUpdate& update : pcurveUpdates) {
            if (update.secondCurve.IsNull())
                builder.UpdateEdge(update.edge, update.firstCurve,
                    update.newSurface, update.location, update.tolerance);
            else
                builder.UpdateEdge(update.edge, update.firstCurve,
                    update.secondCurve, update.newSurface, update.location,
                    update.tolerance);
            builder.Range(update.edge, update.newSurface, update.location,
                update.first, update.last);
        }
        for (const FaceUpdate& update : faceUpdates)
            builder.UpdateFace(update.face, update.surface, update.location,
                update.tolerance);
        for (const PcurveUpdate& update : pcurveUpdates) {
            const Handle(Geom2d_Curve) empty;
            if (update.secondCurve.IsNull())
                builder.UpdateEdge(update.edge, empty, update.oldSurface,
                    update.location, update.tolerance);
            else
                builder.UpdateEdge(update.edge, empty, empty,
                    update.oldSurface, update.location, update.tolerance);
        }
        for (const EdgeUpdate& update : edgeUpdates)
            builder.UpdateEdge(update.edge, update.curve, update.location,
                update.tolerance);
        return Status::Adopted;
    } catch (...) {
        return Status::KeptOriginal;
    }
}

} // namespace core3d::analytic_frame_canonical
