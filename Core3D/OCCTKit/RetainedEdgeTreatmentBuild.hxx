#pragma once

#include "RetainedEdgeTreatmentSnapshot.hxx"
#include "RetainedFaceSelector.hxx"
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BinTools.hxx>
#include <BRepTools.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <cstdio>
#include <istream>
#include <locale>
#include <ostream>
#include <streambuf>

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
        TopoDS_Shape current = base;
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
} // namespace core3d::retained_edge_treatment
