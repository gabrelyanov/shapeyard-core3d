#pragma once

#include "RetainedEdgeTreatmentSnapshot.hxx"
#include "RetainedFaceSelector.hxx"
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BinTools.hxx>
#include <BRepTools.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <TopExp.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <cstdio>
#include <istream>
#include <locale>
#include <ostream>
#include <streambuf>
#include <utility>

namespace core3d::retained_edge_treatment {
inline bool ResolveAnchors(const TopoDS_Shape& source, const std::vector<Anchor>& anchors,
    double metersPerLocalUnit, std::vector<TopoDS_Edge>& edges,
    ReplayBudget& budget, Refusal& refusal) noexcept;
inline bool Replay(const TopoDS_Shape& base, const Definition& definition, TopoDS_Shape& output,
    std::vector<StepProof>& proofs, ReplayBudget& budget, Refusal& refusal) noexcept;
inline bool EquivalentReplayGeometry(const TopoDS_Shape& actual, const TopoDS_Shape& expectation,
    ReplayBudget& budget, Refusal& refusal) noexcept;

namespace detail {
// Bounded exact V3 geometry commitment, mirroring the retained-fillet
// determinism policy: canonical mesh-independent bytes are compared, never
// TShape identity and never a substituted numeric tolerance.
inline constexpr std::size_t MaximumGeometryBytes = 8388608;
class GeometryBuffer final : public std::streambuf {
    std::vector<char> bytes;
    std::size_t extent = 0;
public:
    GeometryBuffer() : bytes(MaximumGeometryBytes) { setp(bytes.data(), bytes.data() + bytes.size()); }
    std::size_t size() const { return std::max(extent, std::size_t(pptr() - pbase())); }
    const char* data() const { return bytes.data(); }
    void read() { extent = size(); setg(bytes.data(), bytes.data(), bytes.data() + extent); }
protected:
    int_type overflow(int_type) override { return traits_type::eof(); }
    pos_type seekoff(off_type offset, std::ios_base::seekdir dir, std::ios_base::openmode mode) override {
        extent = size();
        const bool input = (mode & std::ios_base::in) != 0, output = (mode & std::ios_base::out) != 0;
        if (!input && !output) return pos_type(off_type(-1));
        const off_type current = input ? off_type(gptr() - eback()) : off_type(pptr() - pbase());
        const off_type origin = dir == std::ios_base::beg ? 0 : dir == std::ios_base::cur ? current : off_type(extent);
        const off_type position = origin + offset;
        if (position < 0 || position > off_type(input ? extent : bytes.size())) return pos_type(off_type(-1));
        if (input) setg(eback(), eback() + position, eback() + extent);
        else { setp(bytes.data(), bytes.data() + bytes.size()); pbump(int(position)); }
        return pos_type(position);
    }
    pos_type seekpos(pos_type position, std::ios_base::openmode mode) override {
        return seekoff(off_type(position), std::ios_base::beg, mode);
    }
};
inline bool ChargeTopology(const TopoDS_Shape& shape, ReplayBudget& budget) noexcept {
    TopTools_IndexedMapOfShape map;
    TopExp::MapShapes(shape, map);
    return budget.chargeStage(std::size_t(map.Extent()));
}
inline bool CommitGeometry(const TopoDS_Shape& shape, ReplayBudget& budget, Digest& digest) noexcept {
    try {
        if (shape.IsNull() || !ChargeTopology(shape, budget)) return false;
        GeometryBuffer buffer; std::ostream stream(&buffer); stream.imbue(std::locale::classic());
        BRepTools::Write(shape, stream, Standard_False, Standard_False, TopTools_FormatVersion_VERSION_3);
        if (!stream.good() || buffer.size() == 0 || buffer.size() >= MaximumGeometryBytes) return false;
        return CC_SHA256(buffer.data(), CC_LONG(buffer.size()), digest.data()) != nullptr;
    } catch (...) { return false; }
}
// OCCT's documented persistence phase re-constructs gp_Dir/gp_Ax axes on
// readback. Verification applies the bounded binary readback only to the
// independently rebuilt expectation, never to an observed candidate.
inline bool ReadbackGeometry(const TopoDS_Shape& shape, ReplayBudget& budget, TopoDS_Shape& reopened) noexcept {
    reopened.Nullify();
    try {
        GeometryBuffer buffer; std::ostream writer(&buffer);
        BinTools::Write(shape, writer, Standard_False, Standard_False, BinTools_FormatVersion_VERSION_4);
        if (!writer.good() || buffer.size() == 0 || buffer.size() >= MaximumGeometryBytes) return false;
        buffer.read(); std::istream reader(&buffer); BinTools::Read(reopened, reader);
        return reader.good() && !reopened.IsNull() && ChargeTopology(reopened, budget)
            && BRepCheck_Analyzer(reopened).IsValid();
    } catch (...) { reopened.Nullify(); return false; }
}
} // namespace detail

// D253 mutable-topology isolation. Replay and verification must never run on
// a shared live/stored TopoDS handle: the kernel and native validity checking
// legitimately write mutable TShape bookkeeping (Modified/Checked) on faces
// they visit, including untouched faces a fillet reuses, and on a shared
// handle that bookkeeping escapes into the retained base/original topology —
// no OCAF attribute delta backs up a bit inside a shared TShape. A
// BRepBuilderAPI_Copy is not byte-stable (the existing D235/D236 evidence in
// OcctDocument.mm: Curve2ds 50 vs 40), so detachment is a bounded BinTools
// round trip — the same persistence phase the exact geometry policy already
// trusts for readback — verified against the original under the existing
// mesh-independent V3 commitment (detail::CommitGeometry). No new equivalence
// arm and no tolerance: a shape whose readback reconstruction would not
// reproduce the exact committed bytes is refused, never replayed on shared
// topology. The round trip preserves the seven packed per-TShape flags, so
// the detached copy carries the same bookkeeping state as the original.
inline bool DetachReplayGeometry(const TopoDS_Shape& shared, ReplayBudget& budget,
    TopoDS_Shape& detached, Refusal& refusal) noexcept {
    detached.Nullify();
    try {
        if (shared.IsNull()) { refusal = Refusal::ReplayMismatch; return false; }
        detail::GeometryBuffer buffer; std::ostream writer(&buffer);
        BinTools::Write(shared, writer, Standard_False, Standard_False, BinTools_FormatVersion_VERSION_4);
        if (!writer.good() || buffer.size() == 0 || buffer.size() >= detail::MaximumGeometryBytes) {
            refusal = Refusal::Budget; return false;
        }
        buffer.read(); std::istream reader(&buffer); BinTools::Read(detached, reader);
        if (!reader.good() || detached.IsNull() || !detail::ChargeTopology(detached, budget)) {
            detached.Nullify(); refusal = Refusal::Budget; return false;
        }
        Digest committedShared{}, committedDetached{};
        if (!detail::CommitGeometry(shared, budget, committedShared)
            || !detail::CommitGeometry(detached, budget, committedDetached)) {
            detached.Nullify(); refusal = Refusal::Budget; return false;
        }
        if (committedShared != committedDetached) {
            detached.Nullify(); refusal = Refusal::ReplayMismatch; return false;
        }
        refusal = Refusal::None; return true;
    } catch (...) { detached.Nullify(); refusal = Refusal::BuildFailed; return false; }
}

// Authoritative replay equivalence: the observed current shape must commit to
// the exact mesh-independent V3 bytes of an authoritative replay of the stored
// definition, or of that replay after the bounded binary readback phase above.
// A fresh rebuild is never required to share TShape identity with the stored
// current shape, and no tolerance loosening enters the comparison.
inline bool EquivalentReplayGeometry(const TopoDS_Shape& actual, const TopoDS_Shape& expectation,
    ReplayBudget& budget, Refusal& refusal) noexcept {
    try {
        if (actual.IsNull() || expectation.IsNull()) { refusal = Refusal::ReplayMismatch; return false; }
        Digest committedActual{}, committedExpected{};
        if (!detail::CommitGeometry(actual, budget, committedActual)
            || !detail::CommitGeometry(expectation, budget, committedExpected)) {
            refusal = Refusal::Budget; return false;
        }
        if (committedActual == committedExpected) { refusal = Refusal::None; return true; }
        TopoDS_Shape reopened;
        if (!detail::ReadbackGeometry(expectation, budget, reopened)
            || !detail::CommitGeometry(reopened, budget, committedExpected)) {
            refusal = Refusal::Budget; return false;
        }
        refusal = committedActual == committedExpected ? Refusal::None : Refusal::ReplayMismatch;
        return refusal == Refusal::None;
    } catch (...) { refusal = Refusal::ReplayMismatch; return false; }
}

inline bool VerifyCurrent(const TopoDS_Shape& base, const TopoDS_Shape& current,
    const Definition& definition, const std::vector<StepProof>& proofs, Refusal& refusal) noexcept {
    try {
        if (base.IsNull() || current.IsNull() || !BRepCheck_Analyzer(current).IsValid()
            || proofs.size() != definition.steps.size()) { refusal = Refusal::ReplayMismatch; return false; }
        TopoDS_Shape replayed; std::vector<StepProof> fresh; ReplayBudget budget;
        if (!Replay(base, definition, replayed, fresh, budget, refusal) || fresh.size() != proofs.size()) {
            refusal = Refusal::ReplayMismatch; return false;
        }
        if (!EquivalentReplayGeometry(current, replayed, budget, refusal)) return false;
        for (std::size_t index = 0; index < fresh.size(); ++index) {
            if (fresh[index].feature != proofs[index].feature
                || fresh[index].consumedKeys != proofs[index].consumedKeys) {
                refusal = Refusal::ReplayMismatch; return false;
            }
        }
        refusal = Refusal::None; return true;
    } catch (...) { refusal = Refusal::ReplayMismatch; return false; }
}

inline bool ResolveAnchors(const TopoDS_Shape& source, const std::vector<Anchor>& anchors,
    double metersPerLocalUnit, std::vector<TopoDS_Edge>& edges,
    ReplayBudget& budget, Refusal& refusal) noexcept {
    edges.clear();
    if (source.IsNull() || anchors.empty() || anchors.size() > MaximumAnchorsPerStep
        || !std::isfinite(metersPerLocalUnit) || metersPerLocalUnit <= 0) {
        refusal = Refusal::UnsupportedEdge; return false;
    }
    const double localPerMM = 0.001 / metersPerLocalUnit;
    TopTools_IndexedMapOfShape map; TopExp::MapShapes(source, TopAbs_EDGE, map);
    if (!budget.chargeStage(std::size_t(map.Extent()))) { refusal = Refusal::Budget; return false; }
    std::set<int> used;
    for (const Anchor& anchor : anchors) {
        int unique = 0; TopoDS_Edge selected;
        for (int index = 1; index <= map.Extent(); ++index) {
            const TopoDS_Edge edge = TopoDS::Edge(map(index)); BRepAdaptor_Curve curve(edge);
            const bool kind = (anchor.curve == CurveKind::Line && curve.GetType() == GeomAbs_Line)
                || (anchor.curve == CurveKind::Circle && curve.GetType() == GeomAbs_Circle);
            if (!kind) continue;
            const gp_Pnt point = curve.Value((curve.FirstParameter() + curve.LastParameter()) * .5);
            if (point.Distance(gp_Pnt(anchor.pointMM[0] * localPerMM,
                anchor.pointMM[1] * localPerMM, anchor.pointMM[2] * localPerMM))
                <= 1e-4 * localPerMM) {
                ++unique; selected = edge;
            }
        }
        if (unique == 0) { refusal = Refusal::AnchorMissing; return false; }
        if (unique != 1 || used.count(map.FindIndex(selected))) { refusal = Refusal::AnchorAmbiguous; return false; }
        used.insert(map.FindIndex(selected)); edges.push_back(selected);
    }
    refusal = Refusal::None; return true;
}

inline bool Replay(const TopoDS_Shape& base, const Definition& definition, TopoDS_Shape& output,
    std::vector<StepProof>& proofs, ReplayBudget& budget, Refusal& refusal) noexcept {
    output.Nullify(); proofs.clear();
    try {
        if (base.IsNull() || !detail::valid(definition, refusal)) return false;
        // D253: every replay — detached build, synchronous verification from a
        // stored base and source-rebind — runs on genuinely private topology
        // produced by the shared verified detachment helper, so selector
        // resolution, anchor resolution, the kernel and validity checking
        // never write mutable TShape bookkeeping into live/stored shapes.
        TopoDS_Shape current;
        if (!DetachReplayGeometry(base, budget, current, refusal)) return false;
        const std::atomic_bool neverCancelled{false};
        for (const Step& step : definition.steps) {
            if (step.selector) {
                retained_face_selector::Resolution resolution;
                const auto selectorRefusal = retained_face_selector::Resolve(current, step.selector->intent,
                    definition.base.metersPerLocalUnit, budget, neverCancelled, resolution);
                if (selectorRefusal != retained_face_selector::Refusal::None || !resolution.proof) {
                    refusal = retained_face_selector::MapToB1(selectorRefusal); return false;
                }
                const auto receiptRefusal = retained_face_selector::VerifyReceipt(*resolution.proof, step,
                    definition.base.metersPerLocalUnit, budget);
                if (receiptRefusal != retained_face_selector::Refusal::None) {
                    refusal = retained_face_selector::MapToB1(receiptRefusal); return false;
                }
            }
            std::vector<TopoDS_Edge> edges;
            if (!ResolveAnchors(current, step.anchors, definition.base.metersPerLocalUnit,
                edges, budget, refusal)) return false;
            GProp_GProps before; BRepGProp::VolumeProperties(current, before); TopoDS_Shape candidate;
            const double localAmount = step.amountMM * 0.001 / definition.base.metersPerLocalUnit;
            if (step.kind == Kind::Chamfer) {
                BRepFilletAPI_MakeChamfer build(current);
                for (const auto& edge : edges) build.Add(localAmount, edge);
                build.Build(); if (!build.IsDone()) { refusal = Refusal::BuildFailed; return false; }
                candidate = build.Shape();
            } else {
                BRepFilletAPI_MakeFillet build(current);
                for (const auto& edge : edges) build.Add(localAmount, edge);
                build.Build(); if (!build.IsDone()) { refusal = Refusal::BuildFailed; return false; }
                candidate = build.Shape();
            }
#if DEBUG
            const int rawKernelRootType = candidate.IsNull() ? -1 : int(candidate.ShapeType());
            std::fprintf(stderr, "B1B2_REPLAY phase=kernel-root-raw type=%d\n", rawKernelRootType);
#endif
            // Normalize every successful kernel result to a single forward
            // valid solid before volume measurement, proof values or
            // assignment to current: a direct solid, or a compound with
            // exactly one immediate solid child (TopoDS_Iterator carries the
            // cumulative location/orientation; the child's geometry and
            // location are preserved, never rebuilt or healed). Empty,
            // multi-child, nested-container, shell, compsolid and other roots
            // are refused with the existing build failure.
            if (candidate.IsNull()) { refusal = Refusal::BuildFailed; return false; }
            if (candidate.ShapeType() == TopAbs_COMPOUND) {
                TopoDS_Iterator child(candidate);
                if (!child.More() || child.Value().ShapeType() != TopAbs_SOLID) {
                    refusal = Refusal::BuildFailed; return false;
                }
                const TopoDS_Shape only = child.Value(); child.Next();
                if (child.More()) { refusal = Refusal::BuildFailed; return false; }
                candidate = only;
            }
            if (candidate.ShapeType() != TopAbs_SOLID || candidate.Orientation() != TopAbs_FORWARD) {
                refusal = Refusal::BuildFailed; return false;
            }
#if DEBUG
            std::fprintf(stderr, "B1B2_REPLAY phase=kernel-root-accepted raw=%d type=%d\n",
                rawKernelRootType, int(candidate.ShapeType()));
#endif
            GProp_GProps after; BRepGProp::VolumeProperties(candidate, after);
            // Native volumes are local-unit-cubed; the check and the proof
            // fields below are physical cubic millimetres. For a millimetre
            // document the factor is 1, so the enforced physical threshold is
            // unchanged; no tolerance is loosened for any other unit.
            const double mmPerLocalUnit = definition.base.metersPerLocalUnit * 1000.0;
            const double mm3PerLocalUnitCubed = mmPerLocalUnit * mmPerLocalUnit * mmPerLocalUnit;
            const double removedMM3 = (before.Mass() - after.Mass()) * mm3PerLocalUnitCubed;
            if (!BRepCheck_Analyzer(candidate).IsValid() || !std::isfinite(removedMM3)
                || removedMM3 <= 1e-5) {
                refusal = Refusal::NonRemoving; return false;
            }
            StepProof proof; proof.feature = step.feature;
            proof.inputVolumeMM3 = before.Mass() * mm3PerLocalUnitCubed;
            proof.outputVolumeMM3 = after.Mass() * mm3PerLocalUnitCubed;
            for (const auto& anchor : step.anchors) proof.consumedKeys.push_back(anchor.key);
            proofs.push_back(std::move(proof)); current = candidate;
        }
        output = current; refusal = Refusal::None; return true;
    } catch (...) { output.Nullify(); proofs.clear(); refusal = Refusal::BuildFailed; return false; }
}

// D253 bounded source-edit rebind. A source rebuild legitimately moves native
// edges, so the stored geometric anchor witnesses (physical midpoint, tangent,
// normals, radius) no longer resolve on the rebuilt stage. The two-phase
// contract below is the single bounded source-rebind validation shared by the
// detached construction and OcctDocument's commit validation: it verifies the
// original definition against its old source stage and its exact canonical
// bytes, resolves the unchanged selector intent against the old and the newly
// built stage, establishes a unique one-to-one correspondence between the old
// and new selected boundary uses from source-local boundary roles (curve kind,
// directed tangent and the adjacent outward face normals), and carries every
// anchor key, feature/node/local ID, owner/source identity, issuance state,
// kind, amount and semantic intent forward with the geometric witnesses
// recomputed from the matched new native edges. Any missing, ambiguous or
// non-unique match refuses atomically. Ordinary strict Replay and
// VerifyCurrent stay unchanged for persisted carriers and amount edits.
struct SelectorUseRole {
    UUID anchorKey{};
    CurveKind curve = CurveKind::Line;
    std::array<double, 3> pointMM{}, tangent{}, normalA{}, normalB{};
    double circleRadiusMM = 0;
    retained_face_selector::FaceUseDirection direction =
        retained_face_selector::FaceUseDirection::Forward;
};
// Value-only contract data; no topology handle crosses the detached boundary.
struct SourceRebindRoles {
    Definition original{};
    std::vector<std::uint8_t> originalBytes;
    // One entry per selector step, in definition step order; each use vector
    // stays parallel to that step's key-ordered anchors.
    std::vector<std::pair<UUID, std::vector<SelectorUseRole>>> selectorSteps;
};
struct SourceRebindResult {
    Definition definition{};
    std::vector<std::uint8_t> bytes;
    TopoDS_Shape treated;
    std::vector<StepProof> proofs;
};

namespace detail {
// Measurement-only counterpart of the viewer's issuance-time anchor witness:
// identical geometry and tolerances, but it never mints an anchor key.
inline bool MeasureUseWitness(const retained_face_selector::BoundaryUse& use,
    double metersPerLocalUnit, CurveKind& curve, std::array<double, 3>& pointMM,
    std::array<double, 3>& tangent, std::array<double, 3>& normalA,
    std::array<double, 3>& normalB, double& circleRadiusMM) noexcept {
    try {
        if (use.ownerFaces.size() != 2 || !std::isfinite(metersPerLocalUnit)
            || metersPerLocalUnit <= 0) return false;
        BRepAdaptor_Curve adaptor(use.edge);
        const auto type = adaptor.GetType();
        if (type != GeomAbs_Line && type != GeomAbs_Circle) return false;
        const double parameter = (adaptor.FirstParameter() + adaptor.LastParameter()) * .5;
        gp_Pnt point; gp_Vec derivative; adaptor.D1(parameter, point, derivative);
        if (derivative.SquareMagnitude() <= Precision::SquareConfusion()) return false;
        derivative.Normalize();
        if (use.direction == retained_face_selector::FaceUseDirection::Reversed) derivative.Reverse();
        const double millimetersPerLocal = metersPerLocalUnit * 1000.0;
        curve = type == GeomAbs_Line ? CurveKind::Line : CurveKind::Circle;
        pointMM = {point.X() * millimetersPerLocal, point.Y() * millimetersPerLocal,
            point.Z() * millimetersPerLocal};
        tangent = {derivative.X(), derivative.Y(), derivative.Z()};
        circleRadiusMM = type == GeomAbs_Circle
            ? adaptor.Circle().Radius() * millimetersPerLocal : 0;
        std::array<std::array<double, 3>, 2> normals{};
        for (std::size_t index = 0; index < 2; ++index) {
            const TopoDS_Face& face = use.ownerFaces[index];
            Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            ShapeAnalysis_Surface analysis(surface);
            const gp_Pnt2d uv = analysis.ValueOfUV(point, 1e-7);
            BRepLProp_SLProps properties(BRepAdaptor_Surface(face), uv.X(), uv.Y(), 1, 1e-9);
            if (!properties.IsNormalDefined()) return false;
            gp_Dir normal = properties.Normal();
            if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
            normals[index] = {normal.X(), normal.Y(), normal.Z()};
        }
        if (normals[1] < normals[0]) std::swap(normals[0], normals[1]);
        normalA = normals[0]; normalB = normals[1];
        return true;
    } catch (...) { return false; }
}
inline bool SameDirection(const std::array<double, 3>& first,
    const std::array<double, 3>& second) noexcept {
    try {
        if (!unit(first) || !unit(second)) return false;
        return gp_Dir(first[0], first[1], first[2])
            .IsEqual(gp_Dir(second[0], second[1], second[2]), 1e-8);
    } catch (...) { return false; }
}
// The unchanged physical witness tolerance, (1e-4 mm)^2.
inline bool SameWitnessPoint(const std::array<double, 3>& first,
    const std::array<double, 3>& second) noexcept {
    const double dx = first[0] - second[0], dy = first[1] - second[1],
        dz = first[2] - second[2];
    return std::isfinite(dx) && std::isfinite(dy) && std::isfinite(dz)
        && dx * dx + dy * dy + dz * dz <= 1e-8;
}
} // namespace detail

// Phase one, on the main-thread authority side: verify the original definition
// against its old source stage and its exact canonical bytes, and capture the
// old selected-use roles as pure value data. Every step is strictly replayed
// on its correct old input stage, which both proves the stored witnesses
// against the old native geometry and yields that stage for selector steps.
inline bool CaptureSourceRebindRoles(const TopoDS_Shape& oldStage,
    const Definition& original, const std::vector<std::uint8_t>& originalBytes,
    ReplayBudget& budget, Refusal& refusal, SourceRebindRoles& output) noexcept {
    output = {};
    try {
        std::vector<std::uint8_t> canonical;
        if (oldStage.IsNull() || !Encode(original, canonical, refusal)
            || canonical != originalBytes) {
            refusal = Refusal::MalformedCarrier; return false;
        }
        if (!detail::ChargeTopology(oldStage, budget)) { refusal = Refusal::Budget; return false; }
        const std::atomic_bool neverCancelled{false};
        TopoDS_Shape current = oldStage;
        for (const Step& step : original.steps) {
            if (step.selector) {
                retained_face_selector::Resolution resolution;
                const auto selectorRefusal = retained_face_selector::Resolve(current,
                    step.selector->intent, original.base.metersPerLocalUnit, budget,
                    neverCancelled, resolution);
                if (selectorRefusal != retained_face_selector::Refusal::None || !resolution.proof
                    || retained_face_selector::VerifyReceipt(*resolution.proof, step,
                        original.base.metersPerLocalUnit, budget)
                        != retained_face_selector::Refusal::None
                    || resolution.proof->selectedEdgeCount() != step.anchors.size()) {
                    refusal = Refusal::ReplayMismatch; return false;
                }
                const auto& proof = *resolution.proof;
                std::vector<SelectorUseRole> measured;
                for (const auto& use : proof.boundaryUses()) {
                    SelectorUseRole role;
                    if (use.selected
                        && !detail::MeasureUseWitness(use, original.base.metersPerLocalUnit,
                            role.curve, role.pointMM, role.tangent, role.normalA, role.normalB,
                            role.circleRadiusMM)) {
                        refusal = Refusal::UnsupportedEdge; return false;
                    }
                    measured.push_back(role);
                }
                // Confirm each stored anchor against the actual old native
                // geometry: exactly one selected old use must reproduce its
                // complete witness, including its position and use direction.
                std::set<int> matched;
                std::vector<SelectorUseRole> ordered;
                for (std::size_t index = 0; index < step.anchors.size(); ++index) {
                    const Anchor& anchor = step.anchors[index];
                    int found = -1;
                    for (std::size_t useIndex = 0; useIndex < measured.size(); ++useIndex) {
                        if (!proof.boundaryUses()[useIndex].selected
                            || matched.count(int(useIndex))) continue;
                        const SelectorUseRole& role = measured[useIndex];
                        if (role.curve == anchor.curve
                            && proof.boundaryUses()[useIndex].direction
                                == step.selector->entries[index].direction
                            && detail::SameDirection(role.tangent, anchor.tangent)
                            && detail::SameDirection(role.normalA, anchor.normalA)
                            && detail::SameDirection(role.normalB, anchor.normalB)
                            && std::abs(role.circleRadiusMM - anchor.circleRadiusMM) <= 1e-4
                            && detail::SameWitnessPoint(role.pointMM, anchor.pointMM)) {
                            if (found >= 0) { refusal = Refusal::AnchorAmbiguous; return false; }
                            found = int(useIndex);
                        }
                    }
                    if (found < 0) { refusal = Refusal::ReplayMismatch; return false; }
                    matched.insert(found);
                    SelectorUseRole role = measured[std::size_t(found)];
                    role.anchorKey = anchor.key;
                    role.direction = proof.boundaryUses()[std::size_t(found)].direction;
                    ordered.push_back(role);
                }
                output.selectorSteps.push_back({step.feature, std::move(ordered)});
            }
            Definition single = original;
            single.steps = {step};
            single.outputNode = step.node;
            TopoDS_Shape next; std::vector<StepProof> proofs;
            if (!Replay(current, single, next, proofs, budget, refusal)) {
                output = {}; return false;
            }
            current = next;
        }
        output.original = original; output.originalBytes = originalBytes;
        refusal = Refusal::None; return true;
    } catch (...) { output = {}; refusal = Refusal::ReplayMismatch; return false; }
}

// Phase two, usable from the detached build: resolve the unchanged selector
// intent against the actually rebuilt source stage, rebind the verified
// witnesses through the unique role correspondence, update the canonical
// source binding with the exact requested typed recipe/schema/digest, and
// strictly replay every suffix step on its correct newly rebuilt input stage.
inline bool ApplySourceRebind(const SourceRebindRoles& roles,
    const BaseRecipe& requested, const TopoDS_Shape& newStage, ReplayBudget& budget,
    Refusal& refusal, SourceRebindResult& output) noexcept {
    output = {};
    try {
        if (newStage.IsNull() || !BRepCheck_Analyzer(newStage).IsValid()
            || !detail::ChargeTopology(newStage, budget)) {
            refusal = Refusal::ReplayMismatch; return false;
        }
        std::vector<double> values; std::vector<std::uint8_t> sourceBytes;
        std::uint32_t sourceSchema = 0; SourceFamily family = SourceFamily::Profile;
        if (const auto* profile = std::get_if<core3d::profile::Parameters>(&requested)) {
            if (!core3d::profile::Encode(*profile, values)
                || !core3d::composite_recipe::EncodeScalarRecipe(
                    core3d::composite_recipe::RecipeKind::Profile,
                    core3d::profile::SchemaFor(*profile), values, sourceBytes)) {
                refusal = Refusal::MalformedCarrier; return false;
            }
            sourceSchema = std::uint32_t(core3d::profile::SchemaFor(*profile));
            family = SourceFamily::Profile;
        } else if (const auto* enclosure = std::get_if<core3d::enclosure::Parameters>(&requested)) {
            if (!core3d::enclosure::Encode(*enclosure, values)
                || !core3d::composite_recipe::EncodeScalarRecipe(
                    core3d::composite_recipe::RecipeKind::Enclosure,
                    enclosure->definition.constructionFrame ? 2 : 1, values, sourceBytes)) {
                refusal = Refusal::MalformedCarrier; return false;
            }
            sourceSchema = enclosure->definition.constructionFrame ? 2 : 1;
            family = SourceFamily::Enclosure;
        } else { refusal = Refusal::MalformedCarrier; return false; }
        Digest sourceDigest{};
        if (!core3d::composite_recipe::Hash(sourceBytes, sourceDigest)
            || !retained_recipe::Nonzero(sourceDigest)
            || family != roles.original.base.family
            || sourceSchema != roles.original.base.sourceSchema) {
            refusal = Refusal::IdentityMismatch; return false;
        }
        Definition rebound = roles.original;
        rebound.base.sourceRecipeDigest = sourceDigest;
        const std::atomic_bool neverCancelled{false};
        TopoDS_Shape current = newStage;
        std::size_t selectorIndex = 0;
        for (std::size_t index = 0; index < rebound.steps.size(); ++index) {
            Step step = rebound.steps[index];
            if (step.selector) {
                if (selectorIndex >= roles.selectorSteps.size()
                    || roles.selectorSteps[selectorIndex].first != step.feature
                    || roles.selectorSteps[selectorIndex].second.size() != step.anchors.size()) {
                    refusal = Refusal::ReplayMismatch; return false;
                }
                const auto& oldRoles = roles.selectorSteps[selectorIndex].second;
                ++selectorIndex;
                retained_face_selector::Resolution resolution;
                const auto selectorRefusal = retained_face_selector::Resolve(current,
                    step.selector->intent, rebound.base.metersPerLocalUnit, budget,
                    neverCancelled, resolution);
                if (selectorRefusal != retained_face_selector::Refusal::None || !resolution.proof) {
                    refusal = retained_face_selector::MapToB1(selectorRefusal); return false;
                }
                const auto& proof = *resolution.proof;
                if (proof.selectedEdgeCount() != step.anchors.size()) {
                    refusal = Refusal::ReplayMismatch; return false;
                }
                std::vector<SelectorUseRole> measured;
                for (const auto& use : proof.boundaryUses()) {
                    SelectorUseRole role;
                    if (use.selected
                        && !detail::MeasureUseWitness(use, rebound.base.metersPerLocalUnit,
                            role.curve, role.pointMM, role.tangent, role.normalA, role.normalB,
                            role.circleRadiusMM)) {
                        refusal = Refusal::UnsupportedEdge; return false;
                    }
                    measured.push_back(role);
                }
                // Unique one-to-one correspondence from each old selected use to
                // a new selected use by source-local boundary role. Position is
                // deliberately excluded: it is exactly what a source edit
                // changes, while opposite sides stay distinguishable through
                // the directed tangent and the adjacent outward normals.
                std::set<int> consumed;
                std::vector<SelectorUseRole> matchedNew;
                for (std::size_t anchorIndex = 0; anchorIndex < step.anchors.size();
                    ++anchorIndex) {
                    const SelectorUseRole& oldRole = oldRoles[anchorIndex];
                    if (oldRole.anchorKey != step.anchors[anchorIndex].key) {
                        refusal = Refusal::ReplayMismatch; return false;
                    }
                    int found = -1;
                    for (std::size_t useIndex = 0; useIndex < measured.size(); ++useIndex) {
                        if (!proof.boundaryUses()[useIndex].selected
                            || consumed.count(int(useIndex))) continue;
                        const SelectorUseRole& candidate = measured[useIndex];
                        if (candidate.curve == oldRole.curve
                            && detail::SameDirection(candidate.tangent, oldRole.tangent)
                            && detail::SameDirection(candidate.normalA, oldRole.normalA)
                            && detail::SameDirection(candidate.normalB, oldRole.normalB)
                            && std::abs(candidate.circleRadiusMM - oldRole.circleRadiusMM)
                                <= 1e-4) {
                            if (found >= 0) { refusal = Refusal::AnchorAmbiguous; return false; }
                            found = int(useIndex);
                        }
                    }
                    if (found < 0) { refusal = Refusal::AnchorMissing; return false; }
                    consumed.insert(found);
                    SelectorUseRole role = measured[std::size_t(found)];
                    role.anchorKey = oldRole.anchorKey;
                    role.direction = proof.boundaryUses()[std::size_t(found)].direction;
                    matchedNew.push_back(role);
                }
                if (consumed.size() != proof.selectedEdgeCount()) {
                    refusal = Refusal::ReplayMismatch; return false;
                }
                // Carry identity, amount and semantic intent forward verbatim;
                // refresh only the geometric witnesses and the permitted
                // receipt face/use observations from the new native edges.
                retained_face_selector::SelectorReceipt receipt = *step.selector;
                receipt.face = proof.plane();
                receipt.coverage = proof.coverage();
                receipt.wireCount = proof.wireCount();
                receipt.boundaryUseCount = std::uint32_t(proof.boundaryUses().size());
                receipt.boundaryUniqueEdgeCount = proof.uniqueEdgeCount();
                for (std::size_t anchorIndex = 0; anchorIndex < step.anchors.size();
                    ++anchorIndex) {
                    Anchor& anchor = step.anchors[anchorIndex];
                    const SelectorUseRole& role = matchedNew[anchorIndex];
                    anchor.curve = role.curve;
                    anchor.pointMM = role.pointMM;
                    anchor.tangent = role.tangent;
                    anchor.normalA = role.normalA;
                    anchor.normalB = role.normalB;
                    anchor.circleRadiusMM = role.circleRadiusMM;
                    receipt.entries[anchorIndex] = {role.anchorKey, role.direction};
                }
                step.selector = std::move(receipt);
            }
            Definition single = rebound;
            single.steps = {step};
            single.outputNode = step.node;
            TopoDS_Shape next; std::vector<StepProof> proofs;
            if (!Replay(current, single, next, proofs, budget, refusal)) {
                output = {}; return false;
            }
            if (proofs.size() != 1) { output = {}; refusal = Refusal::ReplayMismatch; return false; }
            rebound.steps[index] = step;
            output.proofs.push_back(proofs.front());
            current = next;
        }
        if (selectorIndex != roles.selectorSteps.size()) {
            output = {}; refusal = Refusal::ReplayMismatch; return false;
        }
        rebound.outputNode = rebound.steps.empty()
            ? rebound.base.sourceNode : rebound.steps.back().node;
        output.definition = rebound;
        if (!Encode(output.definition, output.bytes, refusal)) { output = {}; return false; }
        output.treated = current;
        refusal = Refusal::None; return true;
    } catch (...) { output = {}; refusal = Refusal::BuildFailed; return false; }
}
} // namespace core3d::retained_edge_treatment
