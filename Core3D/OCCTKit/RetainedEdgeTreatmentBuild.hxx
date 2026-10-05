#pragma once

#include "RetainedEdgeTreatmentSnapshot.hxx"
#include "RetainedTopologyBudget.hxx"
#include "RectangularLoftPersistence.hxx"
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
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <cstdio>
#include <istream>
#include <locale>
#include <ostream>
#include <streambuf>
#include <utility>

namespace core3d::retained_edge_treatment {
namespace tb = core3d::retained_topology_budget;
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
// C11: bounded stage census. The stage debit is preserved; the unbounded
// whole-topology map is replaced by the explicit-stack occurrence walk that
// charges every occurrence and enforces the combined face+edge census before
// map growth.
inline bool ChargeTopology(const TopoDS_Shape& shape, ReplayBudget& budget) noexcept {
    const std::atomic_bool neverCancelled{false};
    tb::Census census;
    return tb::CensusTopology(shape, budget, neverCancelled, census,
        tb::Site::C11GeometryCensus, true) == tb::WalkStatus::Completed;
}
inline bool CommitGeometry(const TopoDS_Shape& shape, ReplayBudget& budget, Digest& digest) noexcept {
    try {
        if (shape.IsNull()) return false;
        const std::atomic_bool neverCancelled{false};
        tb::Census census;
        if (tb::CensusTopology(shape, budget, neverCancelled, census,
                tb::Site::C11GeometryCensus, true) != tb::WalkStatus::Completed) return false;
        // C11: the exact V3 serialization pass is reserved (one admitted
        // occurrence census) before it runs; the 8 MiB cap and exact bytes
        // are unchanged.
        if (!tb::ReserveTraversal(census, budget, tb::Site::C11GeometryCensus)) return false;
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
        const std::atomic_bool neverCancelled{false};
        // C12: the source pass is debited before the bounded binary write/read.
        if (tb::ChargeTraversal(shape, budget, neverCancelled, tb::Site::C12Detachment)
            != tb::WalkStatus::Completed) return false;
        GeometryBuffer buffer; std::ostream writer(&buffer);
        BinTools::Write(shape, writer, Standard_False, Standard_False, BinTools_FormatVersion_VERSION_4);
        if (!writer.good() || buffer.size() == 0 || buffer.size() >= MaximumGeometryBytes) return false;
        buffer.read(); std::istream reader(&buffer); BinTools::Read(reopened, reader);
        if (!reader.good() || reopened.IsNull()) return false;
        // C12: the reopened shape is censused before its validity analysis,
        // and the analyzer pass is reserved.
        tb::Census census;
        if (tb::CensusTopology(reopened, budget, neverCancelled, census,
                tb::Site::C11GeometryCensus, true) != tb::WalkStatus::Completed
            || !tb::ReserveTraversal(census, budget, tb::Site::C12Detachment)) return false;
        return BRepCheck_Analyzer(reopened).IsValid();
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
#if DEBUG
    // Row-424 attribution: name the actual failing stage with its refusal, the
    // existing budget counters and — once both commits exist — the two
    // mesh-independent V3 commitment digests (first 8 bytes each). Diagnostics
    // only; no branch, budget or equivalence behavior changes.
    const auto traceFailure=[&](const char* stage){
        std::fprintf(stderr,"B1B2_DETACH stage=%s refusal=%s buildStages=%zu topologyVisits=%zu\n",
            stage,RefusalCode(refusal),budget.buildStages,budget.topologyVisits);
    };
    const auto traceDigest=[](const char* tag,const Digest& digest){
        std::fprintf(stderr,"B1B2_DETACH digest=%s %02x%02x%02x%02x%02x%02x%02x%02x\n",tag,
            digest[0],digest[1],digest[2],digest[3],digest[4],digest[5],digest[6],digest[7]);
    };
#endif
    try {
        if (shared.IsNull()) { refusal = Refusal::ReplayMismatch;
#if DEBUG
            traceFailure("null-input");
#endif
            return false; }
        // C12: the source pass is debited before serialization; the round
        // trip, both exact commitments and the reopened census keep their
        // existing charges below.
        const std::atomic_bool neverCancelled{false};
        std::size_t sourceOccurrences = 0;
        if (tb::ChargeTraversal(shared, budget, neverCancelled, tb::Site::C12Detachment,
                &sourceOccurrences) != tb::WalkStatus::Completed
            || !budget.visit(sourceOccurrences, tb::Site::C12Detachment)) {
            refusal = Refusal::Budget;
#if DEBUG
            traceFailure("source-pass");
#endif
            return false;
        }
        detail::GeometryBuffer buffer; std::ostream writer(&buffer);
        BinTools::Write(shared, writer, Standard_False, Standard_False, BinTools_FormatVersion_VERSION_4);
        if (!writer.good() || buffer.size() == 0 || buffer.size() >= detail::MaximumGeometryBytes) {
            refusal = Refusal::Budget;
#if DEBUG
            traceFailure("write");
#endif
            return false;
        }
        buffer.read(); std::istream reader(&buffer); BinTools::Read(detached, reader);
        if (!reader.good() || detached.IsNull() || !detail::ChargeTopology(detached, budget)) {
            detached.Nullify(); refusal = Refusal::Budget;
#if DEBUG
            traceFailure("read");
#endif
            return false;
        }
        Digest committedShared{}, committedDetached{};
        if (!detail::CommitGeometry(shared, budget, committedShared)
            || !detail::CommitGeometry(detached, budget, committedDetached)) {
            detached.Nullify(); refusal = Refusal::Budget;
#if DEBUG
            traceFailure("commit");
#endif
            return false;
        }
        if (committedShared != committedDetached) {
            detached.Nullify(); refusal = Refusal::ReplayMismatch;
#if DEBUG
            traceFailure("commitment-mismatch");
            traceDigest("shared",committedShared);
            traceDigest("detached",committedDetached);
#endif
            return false;
        }
        refusal = Refusal::None; return true;
    } catch (...) { detached.Nullify(); refusal = Refusal::BuildFailed;
#if DEBUG
        traceFailure("exception");
#endif
        return false; }
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

// L05/C13: the budgeted overload continues the caller's operation budget
// through the whole actual/expectation/readback verification; the analyzer
// input pass is charged first, and a budget failure survives error mapping
// instead of being rewritten to ReplayMismatch.
inline bool VerifyCurrent(const TopoDS_Shape& base, const TopoDS_Shape& current,
    const Definition& definition, const std::vector<StepProof>& proofs,
    ReplayBudget& budget, Refusal& refusal) noexcept {
    try {
        if (base.IsNull() || current.IsNull()) { refusal = Refusal::ReplayMismatch; return false; }
        const std::atomic_bool neverCancelled{false};
        if (tb::ChargeTraversal(current, budget, neverCancelled, tb::Site::C13CommitVerify)
            != tb::WalkStatus::Completed) { refusal = Refusal::Budget; return false; }
        if (!BRepCheck_Analyzer(current).IsValid()
            || proofs.size() != definition.steps.size()) { refusal = Refusal::ReplayMismatch; return false; }
        TopoDS_Shape replayed; std::vector<StepProof> fresh;
        if (!Replay(base, definition, replayed, fresh, budget, refusal) || fresh.size() != proofs.size()) {
            if (refusal != Refusal::Budget) refusal = Refusal::ReplayMismatch;
            return false;
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
// Compatibility signature for independent calls outside an operation; B2
// operation-internal callers must use the budgeted overload above.
inline bool VerifyCurrent(const TopoDS_Shape& base, const TopoDS_Shape& current,
    const Definition& definition, const std::vector<StepProof>& proofs, Refusal& refusal) noexcept {
    ReplayBudget budget;
    return VerifyCurrent(base, current, definition, proofs, budget, refusal);
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
    // C14: bounded combined face+edge census with the stage debit preserved;
    // the edge map used for lookups is produced by the same bounded walk.
    const std::atomic_bool neverCancelled{false};
    tb::Census census;
    if (tb::CensusTopology(source, budget, neverCancelled, census,
            tb::Site::C14AnchorResolve, true) != tb::WalkStatus::Completed) {
        refusal = Refusal::Budget; return false;
    }
    const TopTools_IndexedMapOfShape& map = census.edges;
    std::set<int> used;
    for (const Anchor& anchor : anchors) {
        int unique = 0; TopoDS_Edge selected;
        for (int index = 1; index <= map.Extent(); ++index) {
            // C14: every anchor-edge comparison is charged, including
            // unsuccessful matches.
            if (!budget.visit(1, tb::Site::C14AnchorResolve)) {
                refusal = Refusal::Budget; return false;
            }
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
        // C15: proofs accumulate into a private temporary and are published
        // only on whole-operation success; every refusal path below leaves
        // output null and proofs cleared.
        std::vector<StepProof> pending;
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
            // C16: the kernel build stage is debited before the build, and the
            // relevant input pass (volume measurement plus kernel traversal)
            // is charged up front.
            if (!budget.beginStage(tb::Site::C16KernelBuild)
                || tb::ChargeTraversal(current, budget, neverCancelled, tb::Site::C16KernelBuild)
                    != tb::WalkStatus::Completed) {
                refusal = Refusal::Budget; return false;
            }
            GProp_GProps before; BRepGProp::VolumeProperties(current, before); TopoDS_Shape candidate;
            const double localAmount = step.amountMM * 0.001 / definition.base.metersPerLocalUnit;
            if (step.kind == Kind::Chamfer) {
                BRepFilletAPI_MakeChamfer build(current);
                for (const auto& edge : edges) {
                    if (!budget.visit(1, tb::Site::C16KernelBuild)) {
                        refusal = Refusal::Budget; return false;
                    }
                    build.Add(localAmount, edge);
                }
                build.Build(); if (!build.IsDone()) { refusal = Refusal::BuildFailed; return false; }
                candidate = build.Shape();
            } else {
                BRepFilletAPI_MakeFillet build(current);
                for (const auto& edge : edges) {
                    if (!budget.visit(1, tb::Site::C16KernelBuild)) {
                        refusal = Refusal::Budget; return false;
                    }
                    build.Add(localAmount, edge);
                }
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
                // C16: compound-child inspection is charged per child.
                TopoDS_Iterator child(candidate);
                if (!child.More()) { refusal = Refusal::BuildFailed; return false; }
                if (!budget.visit(1, tb::Site::C16KernelBuild)) {
                    refusal = Refusal::Budget; return false;
                }
                if (child.Value().ShapeType() != TopAbs_SOLID) {
                    refusal = Refusal::BuildFailed; return false;
                }
                const TopoDS_Shape only = child.Value(); child.Next();
                if (child.More()) {
                    if (!budget.visit(1, tb::Site::C16KernelBuild)) {
                        refusal = Refusal::Budget; return false;
                    }
                    refusal = Refusal::BuildFailed; return false;
                }
                candidate = only;
            }
            if (candidate.ShapeType() != TopAbs_SOLID || candidate.Orientation() != TopAbs_FORWARD) {
                refusal = Refusal::BuildFailed; return false;
            }
            // C16: the produced shape is bounded and charged immediately,
            // before its volume measurement and validity analysis; both later
            // passes are reserved from the measured census.
            tb::Census candidateCensus;
            if (tb::CensusTopology(candidate, budget, neverCancelled, candidateCensus,
                    tb::Site::C16KernelBuild, false) != tb::WalkStatus::Completed
                || !tb::ReserveTraversal(candidateCensus, budget, tb::Site::C16KernelBuild)
                || !tb::ReserveTraversal(candidateCensus, budget, tb::Site::C16KernelBuild)) {
                refusal = Refusal::Budget; return false;
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
            pending.push_back(std::move(proof)); current = candidate;
        }
        output = current; proofs = std::move(pending); refusal = Refusal::None; return true;
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
    // CLOUD-8242: one entry per raw-anchor step of an admitted Profile source,
    // in definition step order, parallel to that step's key-ordered anchors.
    // The measured old-stage roles let phase two prove a unique one-to-one
    // raw-target correspondence without a midpoint or first-match oracle.
    std::vector<std::pair<UUID, std::vector<SelectorUseRole>>> rawSteps;
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
    std::array<double, 3>& normalB, double& circleRadiusMM,
    tb::Counter* shared = nullptr, tb::Site site = tb::Site::None) noexcept {
    try {
        if (use.ownerFaces.size() != 2 || !std::isfinite(metersPerLocalUnit)
            || metersPerLocalUnit <= 0) return false;
        // C17/C18: the edge witness measurement is one charged unit.
        if (shared && !shared->visit(1, site)) return false;
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
            // C18: each adjacent-face normal measurement is charged.
            if (shared && !shared->visit(1, site)) return false;
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
// CLOUD-8242: the issuance-time raw capture keeps the adjacent-face order,
// while the measured role orders the outward normals lexicographically; an old
// stored witness therefore matches a measured role with either normal order.
inline bool SameUnorderedNormals(const SelectorUseRole& role, const Anchor& anchor) noexcept {
    return (SameDirection(role.normalA, anchor.normalA)
            && SameDirection(role.normalB, anchor.normalB))
        || (SameDirection(role.normalA, anchor.normalB)
            && SameDirection(role.normalB, anchor.normalA));
}
// A stored raw anchor is proven against the old stage only when one measured
// role reproduces its complete witness: curve kind, physical position,
// directed tangent, both outward normals and the circle radius.
inline bool RawRoleMatchesAnchor(const SelectorUseRole& role, const Anchor& anchor) noexcept {
    return role.curve == anchor.curve
        && SameWitnessPoint(role.pointMM, anchor.pointMM)
        && SameDirection(role.tangent, anchor.tangent)
        && SameUnorderedNormals(role, anchor)
        && std::abs(role.circleRadiusMM - anchor.circleRadiusMM) <= 1e-4;
}
// Role geometry that survives a source edit: the position is deliberately
// excluded because it is exactly what the edit moves; opposite sides stay
// distinguishable through the directed tangent and the ordered normals. Both
// roles are measured with the lexicographic normal ordering, so the comparison
// is ordered here.
inline bool SameRawRoleGeometry(const SelectorUseRole& first,
    const SelectorUseRole& second) noexcept {
    return first.curve == second.curve
        && SameDirection(first.tangent, second.tangent)
        && SameDirection(first.normalA, second.normalA)
        && SameDirection(first.normalB, second.normalB)
        && std::abs(first.circleRadiusMM - second.circleRadiusMM) <= 1e-4;
}
// CLOUD-8242: measured raw-edge witness roles for one stage. Every edge with
// exactly two adjacent owner faces and an admitted curve kind is measured with
// the issuance-time geometry (physical midpoint, directed tangent,
// lexicographically ordered outward normals, circle radius); the walk and
// every measurement are charged to the operation budget. Pure measurement: no
// identity is minted and no topology handle leaves the caller.
inline bool MeasureStageRawEdgeWitnesses(const TopoDS_Shape& stage,
    double metersPerLocalUnit, ReplayBudget& budget, tb::Site site,
    std::vector<SelectorUseRole>& output) noexcept {
    output.clear();
    try {
        if (stage.IsNull() || !std::isfinite(metersPerLocalUnit) || metersPerLocalUnit <= 0)
            return false;
        const std::atomic_bool neverCancelled{false};
        tb::Census census;
        if (tb::CensusTopology(stage, budget, neverCancelled, census, site, false)
                != tb::WalkStatus::Completed
            || !budget.visit(census.occurrences + census.edgeUsesUnderFaces, site)) return false;
        TopTools_IndexedDataMapOfShapeListOfShape adjacency;
        TopExp::MapShapesAndAncestors(stage, TopAbs_EDGE, TopAbs_FACE, adjacency);
        const double millimetersPerLocal = metersPerLocalUnit * 1000.0;
        for (int index = 1; index <= adjacency.Extent(); ++index) {
            if (!budget.visit(1, site)) return false;
            const TopoDS_Edge edge = TopoDS::Edge(adjacency.FindKey(index));
            if (adjacency.FindFromIndex(index).Extent() != 2) continue;
            BRepAdaptor_Curve curve(edge);
            const auto type = curve.GetType();
            if (type != GeomAbs_Line && type != GeomAbs_Circle) continue;
            const double parameter = (curve.FirstParameter() + curve.LastParameter()) * .5;
            gp_Pnt point; gp_Vec derivative; curve.D1(parameter, point, derivative);
            if (derivative.SquareMagnitude() <= Precision::SquareConfusion()) return false;
            derivative.Normalize();
            SelectorUseRole role;
            role.curve = type == GeomAbs_Line ? CurveKind::Line : CurveKind::Circle;
            role.pointMM = {point.X() * millimetersPerLocal, point.Y() * millimetersPerLocal,
                point.Z() * millimetersPerLocal};
            role.tangent = {derivative.X(), derivative.Y(), derivative.Z()};
            role.circleRadiusMM = type == GeomAbs_Circle
                ? curve.Circle().Radius() * millimetersPerLocal : 0;
            std::array<std::array<double, 3>, 2> normals{};
            std::size_t faceIndex = 0;
            for (TopTools_ListIteratorOfListOfShape it(adjacency.FindFromIndex(index));
                it.More(); it.Next(), ++faceIndex) {
                if (!budget.visit(1, site)) return false;
                const TopoDS_Face face = TopoDS::Face(it.Value());
                Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
                ShapeAnalysis_Surface analysis(surface);
                const gp_Pnt2d uv = analysis.ValueOfUV(point, 1e-7);
                BRepLProp_SLProps properties(BRepAdaptor_Surface(face), uv.X(), uv.Y(), 1, 1e-9);
                if (!properties.IsNormalDefined()) return false;
                gp_Dir normal = properties.Normal();
                if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
                normals[faceIndex] = {normal.X(), normal.Y(), normal.Z()};
            }
            if (faceIndex != 2) return false;
            if (normals[1] < normals[0]) std::swap(normals[0], normals[1]);
            role.normalA = normals[0];
            role.normalB = normals[1];
            output.push_back(role);
        }
        return true;
    } catch (...) { output.clear(); return false; }
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
        // C15-style publication: roles accumulate into a private temporary
        // and are published only on whole-operation success.
        SourceRebindRoles pending;
        for (const Step& step : original.steps) {
            if (step.selector) {
                retained_face_selector::Resolution resolution;
                const auto selectorRefusal = retained_face_selector::Resolve(current,
                    step.selector->intent, original.base.metersPerLocalUnit, budget,
                    neverCancelled, resolution);
                retained_face_selector::Refusal receiptRefusal =
                    retained_face_selector::Refusal::ReplayMismatch;
                if (selectorRefusal == retained_face_selector::Refusal::None && resolution.proof) {
                    receiptRefusal = retained_face_selector::VerifyReceipt(*resolution.proof, step,
                        original.base.metersPerLocalUnit, budget);
                }
                // C17: a budget refusal propagates as Budget instead of the
                // blanket ReplayMismatch; every other failure keeps the
                // existing mapping.
                if (selectorRefusal != retained_face_selector::Refusal::None || !resolution.proof
                    || receiptRefusal != retained_face_selector::Refusal::None
                    || resolution.proof->selectedEdgeCount() != step.anchors.size()) {
                    refusal = (selectorRefusal == retained_face_selector::Refusal::Budget
                        || receiptRefusal == retained_face_selector::Refusal::Budget)
                        ? Refusal::Budget : Refusal::ReplayMismatch;
                    return false;
                }
                const auto& proof = *resolution.proof;
                std::vector<SelectorUseRole> measured;
                for (const auto& use : proof.boundaryUses()) {
                    // C17: every captured witness loop entry is charged.
                    if (!budget.visit(1, tb::Site::C17SourceRebindOld)) {
                        refusal = Refusal::Budget; return false;
                    }
                    SelectorUseRole role;
                    if (use.selected
                        && !detail::MeasureUseWitness(use, original.base.metersPerLocalUnit,
                            role.curve, role.pointMM, role.tangent, role.normalA, role.normalB,
                            role.circleRadiusMM, &budget, tb::Site::C17SourceRebindOld)) {
                        refusal = budget.exhausted ? Refusal::Budget : Refusal::UnsupportedEdge;
                        return false;
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
                        // C17: every old-side role/witness correspondence
                        // comparison is charged, including nonmatches.
                        if (!budget.visit(1, tb::Site::C17SourceRebindOld)) {
                            refusal = Refusal::Budget; return false;
                        }
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
                pending.selectorSteps.push_back({step.feature, std::move(ordered)});
            } else if (original.base.family == SourceFamily::Profile) {
                // CLOUD-8242: a raw-anchor step of an admitted Profile source
                // captures complete measured roles on the actual old pre-step
                // stage, and every stored anchor must be reproduced by exactly
                // one measured role before any rebind is attempted.
                std::vector<SelectorUseRole> measured;
                if (!detail::MeasureStageRawEdgeWitnesses(current,
                        original.base.metersPerLocalUnit, budget,
                        tb::Site::C17SourceRebindOld, measured)) {
                    refusal = budget.exhausted ? Refusal::Budget : Refusal::UnsupportedEdge;
                    return false;
                }
                std::set<int> matched;
                std::vector<SelectorUseRole> ordered;
                for (const Anchor& anchor : step.anchors) {
                    int found = -1;
                    for (std::size_t useIndex = 0; useIndex < measured.size(); ++useIndex) {
                        // C17: every old-side raw role/witness correspondence
                        // comparison is charged, including nonmatches.
                        if (!budget.visit(1, tb::Site::C17SourceRebindOld)) {
                            refusal = Refusal::Budget; return false;
                        }
                        if (matched.count(int(useIndex))) continue;
                        if (detail::RawRoleMatchesAnchor(measured[useIndex], anchor)) {
                            if (found >= 0) { refusal = Refusal::AnchorAmbiguous; return false; }
                            found = int(useIndex);
                        }
                    }
                    if (found < 0) { refusal = Refusal::ReplayMismatch; return false; }
                    matched.insert(found);
                    SelectorUseRole role = measured[std::size_t(found)];
                    role.anchorKey = anchor.key;
                    ordered.push_back(role);
                }
                pending.rawSteps.push_back({step.feature, std::move(ordered)});
            }
            Definition single = original;
            single.steps = {step};
            single.outputNode = step.node;
            TopoDS_Shape next; std::vector<StepProof> proofs;
            if (!Replay(current, single, next, proofs, budget, refusal)) {
                return false;
            }
            current = next;
        }
        pending.original = original; pending.originalBytes = originalBytes;
        output = std::move(pending);
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
        // C18: the new-stage census is debited before the analyzer, and the
        // analyzer pass is reserved; a budget failure is never mapped to
        // ReplayMismatch.
        if (newStage.IsNull()) { refusal = Refusal::ReplayMismatch; return false; }
        const std::atomic_bool neverCancelledRebind{false};
        tb::Census newStageCensus;
        if (tb::CensusTopology(newStage, budget, neverCancelledRebind, newStageCensus,
                tb::Site::C18SourceRebindNew, true) != tb::WalkStatus::Completed
            || !tb::ReserveTraversal(newStageCensus, budget, tb::Site::C18SourceRebindNew)) {
            refusal = Refusal::Budget; return false;
        }
        if (!BRepCheck_Analyzer(newStage).IsValid()) {
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
        } else if (const auto* loft = std::get_if<core3d::rectangular_loft::Definition>(&requested)) {
            if (!core3d::loft_persistence::Encode(*loft, values)
                || !core3d::composite_recipe::EncodeScalarRecipe(
                    core3d::composite_recipe::RecipeKind::RectangularLoft,
                    std::uint32_t(core3d::loft_persistence::Schema), values, sourceBytes)) {
                refusal = Refusal::MalformedCarrier; return false;
            }
            sourceSchema = std::uint32_t(core3d::loft_persistence::Schema);
            family = SourceFamily::RectangularLoft;
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
        std::size_t rawIndex = 0;
        // Publish-on-success: suffix proofs accumulate privately and are
        // published only when the whole rebind succeeds.
        std::vector<StepProof> pendingProofs;
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
                    // C18: every new-side witness loop entry is charged.
                    if (!budget.visit(1, tb::Site::C18SourceRebindNew)) {
                        refusal = Refusal::Budget; return false;
                    }
                    SelectorUseRole role;
                    if (use.selected
                        && !detail::MeasureUseWitness(use, rebound.base.metersPerLocalUnit,
                            role.curve, role.pointMM, role.tangent, role.normalA, role.normalB,
                            role.circleRadiusMM, &budget, tb::Site::C18SourceRebindNew)) {
                        refusal = budget.exhausted ? Refusal::Budget : Refusal::UnsupportedEdge;
                        return false;
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
                        // C18: every new-side role correspondence comparison
                        // is charged, including nonmatches.
                        if (!budget.visit(1, tb::Site::C18SourceRebindNew)) {
                            refusal = Refusal::Budget; return false;
                        }
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
            } else if (rebound.base.family == SourceFamily::Profile) {
                // CLOUD-8242: rebind a raw-anchor step through the unique
                // one-to-one role correspondence proven against the actually
                // rebuilt pre-step stage. Only the geometric witnesses are
                // refreshed; keys, node/feature/local identities, kind and
                // amount carry forward verbatim. A missing or ambiguous
                // correspondence refuses atomically before any mutation.
                if (rawIndex >= roles.rawSteps.size()
                    || roles.rawSteps[rawIndex].first != step.feature
                    || roles.rawSteps[rawIndex].second.size() != step.anchors.size()) {
                    refusal = Refusal::ReplayMismatch; return false;
                }
                const auto& oldRoles = roles.rawSteps[rawIndex].second;
                ++rawIndex;
                std::vector<SelectorUseRole> measured;
                if (!detail::MeasureStageRawEdgeWitnesses(current,
                        rebound.base.metersPerLocalUnit, budget,
                        tb::Site::C18SourceRebindNew, measured)) {
                    refusal = budget.exhausted ? Refusal::Budget : Refusal::UnsupportedEdge;
                    return false;
                }
                std::set<int> consumed;
                for (std::size_t anchorIndex = 0; anchorIndex < step.anchors.size();
                    ++anchorIndex) {
                    const SelectorUseRole& oldRole = oldRoles[anchorIndex];
                    if (oldRole.anchorKey != step.anchors[anchorIndex].key) {
                        refusal = Refusal::ReplayMismatch; return false;
                    }
                    int found = -1;
                    for (std::size_t useIndex = 0; useIndex < measured.size(); ++useIndex) {
                        // C18: every new-side raw role correspondence
                        // comparison is charged, including nonmatches.
                        if (!budget.visit(1, tb::Site::C18SourceRebindNew)) {
                            refusal = Refusal::Budget; return false;
                        }
                        if (consumed.count(int(useIndex))) continue;
                        if (detail::SameRawRoleGeometry(measured[useIndex], oldRole)) {
                            if (found >= 0) { refusal = Refusal::AnchorAmbiguous; return false; }
                            found = int(useIndex);
                        }
                    }
                    if (found < 0) { refusal = Refusal::AnchorMissing; return false; }
                    consumed.insert(found);
                    Anchor& anchor = step.anchors[anchorIndex];
                    anchor.curve = measured[std::size_t(found)].curve;
                    anchor.pointMM = measured[std::size_t(found)].pointMM;
                    anchor.tangent = measured[std::size_t(found)].tangent;
                    anchor.normalA = measured[std::size_t(found)].normalA;
                    anchor.normalB = measured[std::size_t(found)].normalB;
                    anchor.circleRadiusMM = measured[std::size_t(found)].circleRadiusMM;
                }
            }
            Definition single = rebound;
            single.steps = {step};
            single.outputNode = step.node;
            TopoDS_Shape next; std::vector<StepProof> proofs;
            if (!Replay(current, single, next, proofs, budget, refusal)) {
                return false;
            }
            if (proofs.size() != 1) { refusal = Refusal::ReplayMismatch; return false; }
            rebound.steps[index] = step;
            pendingProofs.push_back(proofs.front());
            current = next;
        }
        if (selectorIndex != roles.selectorSteps.size()
            || rawIndex != roles.rawSteps.size()) {
            refusal = Refusal::ReplayMismatch; return false;
        }
        rebound.outputNode = rebound.steps.empty()
            ? rebound.base.sourceNode : rebound.steps.back().node;
        output.definition = rebound;
        if (!Encode(output.definition, output.bytes, refusal)) { output = {}; return false; }
        output.treated = current;
        output.proofs = std::move(pendingProofs);
        refusal = Refusal::None; return true;
    } catch (...) { output = {}; refusal = Refusal::BuildFailed; return false; }
}
} // namespace core3d::retained_edge_treatment
