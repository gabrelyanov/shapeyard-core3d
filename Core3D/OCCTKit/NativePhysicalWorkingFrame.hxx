#pragma once
// R4 / D681 / D683: private physical-millimetre OCCT working frame.
//
// The caller owns detachment policy and supplies both the operation-wide debt
// and cancellation token. This helper owns only checked dimensional transport,
// one whole-shape scale, bounded subshape history mapping, and one inverse exit.
// It is deliberately a quoted private header with no PBX registration.

#include "RetainedTopologyBudget.hxx"

#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace core3d::native_physical_working_frame {

enum class Status : unsigned char {
    Ready,
    Finished,
    InvalidUnit,
    InvalidScalar,
    NullShape,
    Cancelled,
    BudgetDenied,
    TransformFailed,
    MapMissing,
    MapAmbiguous,
    AlreadyUsed
};

struct Scale final {
    double metersPerLocalUnit = 0;
    double toWorking = 0;
    double toDocument = 0;
    double toWorking2 = 0;
    double toDocument2 = 0;
    double toWorking3 = 0;
    double toDocument3 = 0;

    static bool Make(double unit, Scale& result) noexcept {
        result = Scale{};
        if (!std::isfinite(unit) || unit <= 0) return false;
        const double s = 1000.0 * unit;
        if (!std::isfinite(s) || s <= 0) return false;
        const double q = 1.0 / s;
        const double s2 = s * s, q2 = q * q;
        const double s3 = s2 * s, q3 = q2 * q;
        if (!std::isfinite(q) || q <= 0
            || !std::isfinite(s2) || s2 <= 0
            || !std::isfinite(q2) || q2 <= 0
            || !std::isfinite(s3) || s3 <= 0
            || !std::isfinite(q3) || q3 <= 0) return false;
        result = Scale{unit, s, q, s2, q2, s3, q3};
        return true;
    }
};

namespace detail {
inline bool CheckedProduct(double value, double factor, double& result) noexcept {
    result = 0;
    if (!std::isfinite(value) || !std::isfinite(factor)) return false;
    const double product = value * factor;
    if (!std::isfinite(product) || (value != 0 && product == 0)) return false;
    result = product;
    return true;
}

template <typename StopToken>
inline Status ChargeShape(const TopoDS_Shape& root,
                          retained_topology_budget::Counter& debt,
                          const StopToken& stop,
                          retained_topology_budget::Site site,
                          TopTools_IndexedMapOfShape* faceEdges = nullptr) noexcept {
    try {
        if (root.IsNull()) return Status::NullShape;
        struct Frame { TopoDS_Iterator iterator; };
        std::deque<Frame> stack;
        TopoDS_Shape current = root;
        for (;;) {
            if (stop.load()) return Status::Cancelled;
            if (!debt.visit(1, site)) return Status::BudgetDenied;
            if (faceEdges && (current.ShapeType() == TopAbs_FACE
                              || current.ShapeType() == TopAbs_EDGE)) {
                if (!faceEdges->Contains(current)
                    && faceEdges->Extent() >= static_cast<int>(
                        retained_topology_budget::MaximumStageFaceEdgeCensus)) {
                    debt.deny(site,
                        retained_topology_budget::Dimension::StageCensus, 1);
                    return Status::BudgetDenied;
                }
                faceEdges->Add(current);
            }
            stack.push_back(Frame{TopoDS_Iterator(current)});
            while (!stack.empty() && !stack.back().iterator.More())
                stack.pop_back();
            if (stack.empty()) break;
            current = stack.back().iterator.Value();
            stack.back().iterator.Next();
        }
        return Status::Ready;
    } catch (...) { return Status::TransformFailed; }
}

inline bool FiniteVertices(const TopoDS_Shape& shape) noexcept {
    try {
        std::size_t count = 0;
        for (TopExp_Explorer vertex(shape, TopAbs_VERTEX); vertex.More(); vertex.Next()) {
            const gp_Pnt point = BRep_Tool::Pnt(TopoDS::Vertex(vertex.Current()));
            if (!std::isfinite(point.X()) || !std::isfinite(point.Y())
                || !std::isfinite(point.Z())) return false;
            ++count;
        }
        return count != 0;
    } catch (...) { return false; }
}
} // namespace detail

class Frame final {
public:
    Frame() = default;
    Frame(const Frame&) = delete;
    Frame& operator=(const Frame&) = delete;
    Frame(Frame&& other) noexcept { moveFrom(std::move(other)); }
    Frame& operator=(Frame&& other) noexcept {
        if (this != &other) { clear(); moveFrom(std::move(other)); }
        return *this;
    }

    template <typename StopToken>
    static Status Enter(TopoDS_Shape privateShape,
                        double metersPerLocalUnit,
                        const std::vector<TopoDS_Shape>& selected,
                        retained_topology_budget::Counter& debt,
                        const StopToken& stop,
                        Frame& output) noexcept {
        if (output.state_ != State::Empty) return Status::AlreadyUsed;
        if (stop.load()) return Status::Cancelled;
        Scale scale;
        if (!Scale::Make(metersPerLocalUnit, scale)) return Status::InvalidUnit;
        if (privateShape.IsNull()) return Status::NullShape;
        try {
            if (!debt.beginStage(retained_topology_budget::Site::C11GeometryCensus))
                return Status::BudgetDenied;
            TopTools_IndexedMapOfShape sourceFaceEdges;
            Status status = detail::ChargeShape(privateShape, debt, stop,
                retained_topology_budget::Site::C11GeometryCensus,
                &sourceFaceEdges);
            if (status != Status::Ready) return status;
            if (!detail::FiniteVertices(privateShape)) return Status::InvalidScalar;

            TopoDS_Shape working;
            std::vector<TopoDS_Shape> mapped;
            const bool identity = scale.toWorking == 1.0;
            std::unique_ptr<BRepBuilderAPI_Transform> transformer;
            if (identity) {
                working = privateShape;
            } else {
                if (stop.load()) return Status::Cancelled;
                gp_Trsf transform;
                transform.SetScale(gp_Pnt(0, 0, 0), scale.toWorking);
                transformer = std::make_unique<BRepBuilderAPI_Transform>(
                    privateShape, transform, Standard_True, Standard_False);
                working = transformer->Shape();
                if (working.IsNull()) return Status::TransformFailed;
                if (stop.load()) return Status::Cancelled;
            }

            TopTools_IndexedMapOfShape workingFaceEdges;
            status = detail::ChargeShape(working, debt, stop,
                retained_topology_budget::Site::C11GeometryCensus,
                &workingFaceEdges);
            if (status != Status::Ready) return status;
            if (!detail::FiniteVertices(working)) return Status::InvalidScalar;

            if (!selected.empty()
                && !debt.beginStage(retained_topology_budget::Site::C14AnchorResolve))
                return Status::BudgetDenied;
            TopTools_IndexedMapOfShape distinctMapped;
            mapped.reserve(selected.size());
            for (const TopoDS_Shape& source : selected) {
                if (stop.load()) return Status::Cancelled;
                if (source.IsNull() || !sourceFaceEdges.Contains(source))
                    return Status::MapMissing;
                TopoDS_Shape target;
                if (identity) {
                    target = source;
                } else {
                    const TopTools_ListOfShape& history = transformer->Modified(source);
                    if (history.Extent() != 1) return history.IsEmpty()
                        ? Status::MapMissing : Status::MapAmbiguous;
                    target = history.First();
                }
                if (target.IsNull() || !workingFaceEdges.Contains(target))
                    return Status::MapMissing;
                if (target.Orientation() != source.Orientation())
                    return Status::MapAmbiguous;
                if (distinctMapped.Contains(target)) return Status::MapAmbiguous;
                distinctMapped.Add(target);
                mapped.push_back(target);
            }
            output.scale_ = scale;
            output.working_ = working;
            output.mapped_ = std::move(mapped);
            output.usedTransform_ = !identity;
            output.state_ = State::Active;
            return Status::Ready;
        } catch (...) { return Status::TransformFailed; }
    }

    template <typename StopToken>
    Status Finish(TopoDS_Shape workingResult,
                  retained_topology_budget::Counter& debt,
                  const StopToken& stop,
                  TopoDS_Shape& documentResult) noexcept {
        documentResult.Nullify();
        if (state_ != State::Active) return Status::AlreadyUsed;
        state_ = State::Consumed;
        mapped_.clear();
        working_.Nullify();
        if (stop.load()) return Status::Cancelled;
        if (workingResult.IsNull()) return Status::NullShape;
        try {
            if (!debt.beginStage(retained_topology_budget::Site::C13CommitVerify))
                return Status::BudgetDenied;
            Status status = detail::ChargeShape(workingResult, debt, stop,
                retained_topology_budget::Site::C13CommitVerify);
            if (status != Status::Ready) return status;
            if (!detail::FiniteVertices(workingResult)) return Status::InvalidScalar;
            TopoDS_Shape candidate;
            if (scale_.toDocument == 1.0) {
                candidate = workingResult;
            } else {
                gp_Trsf inverse;
                inverse.SetScale(gp_Pnt(0, 0, 0), scale_.toDocument);
                BRepBuilderAPI_Transform transformed(
                    workingResult, inverse, Standard_True, Standard_False);
                candidate = transformed.Shape();
                if (candidate.IsNull()) return Status::TransformFailed;
                if (stop.load()) return Status::Cancelled;
            }
            status = detail::ChargeShape(candidate, debt, stop,
                retained_topology_budget::Site::C13CommitVerify);
            if (status != Status::Ready) return status;
            if (!detail::FiniteVertices(candidate)) return Status::InvalidScalar;
            if (stop.load()) return Status::Cancelled;
            documentResult = candidate;
            return Status::Finished;
        } catch (...) { return Status::TransformFailed; }
    }

    bool active() const noexcept { return state_ == State::Active; }
    bool usedTransform() const noexcept { return usedTransform_; }
    const TopoDS_Shape& workingShape() const noexcept { return working_; }
    const std::vector<TopoDS_Shape>& mappedSubshapes() const noexcept {
        return mapped_;
    }
    const Scale& scale() const noexcept { return scale_; }

    Status toWorkingLength(double value, double& result) const noexcept {
        return convert(value, scale_.toWorking, result);
    }
    Status toWorkingArea(double value, double& result) const noexcept {
        return convert(value, scale_.toWorking2, result);
    }
    Status toWorkingVolume(double value, double& result) const noexcept {
        return convert(value, scale_.toWorking3, result);
    }
    Status toDocumentLength(double value, double& result) const noexcept {
        return convert(value, scale_.toDocument, result);
    }
    Status toDocumentArea(double value, double& result) const noexcept {
        return convert(value, scale_.toDocument2, result);
    }
    Status toDocumentVolume(double value, double& result) const noexcept {
        return convert(value, scale_.toDocument3, result);
    }
    Status toWorkingPoint(const gp_Pnt& value, gp_Pnt& result) const noexcept {
        double x = 0, y = 0, z = 0;
        if (convert(value.X(), scale_.toWorking, x) != Status::Ready
            || convert(value.Y(), scale_.toWorking, y) != Status::Ready
            || convert(value.Z(), scale_.toWorking, z) != Status::Ready)
            return Status::InvalidScalar;
        result.SetCoord(x, y, z);
        return Status::Ready;
    }
    Status preservePhysicalMillimetres(double value, double& result) const noexcept {
        if (!active() || !std::isfinite(value)) { result = 0; return Status::InvalidScalar; }
        result = value;
        return Status::Ready;
    }

private:
    enum class State : unsigned char { Empty, Active, Consumed };
    State state_ = State::Empty;
    Scale scale_;
    TopoDS_Shape working_;
    std::vector<TopoDS_Shape> mapped_;
    bool usedTransform_ = false;

    Status convert(double value, double factor, double& result) const noexcept {
        if (!active() || !detail::CheckedProduct(value, factor, result))
            return Status::InvalidScalar;
        return Status::Ready;
    }
    void clear() noexcept {
        state_ = State::Empty; scale_ = Scale{}; working_.Nullify();
        mapped_.clear(); usedTransform_ = false;
    }
    void moveFrom(Frame&& other) noexcept {
        state_ = other.state_; scale_ = other.scale_;
        working_ = std::move(other.working_); mapped_ = std::move(other.mapped_);
        usedTransform_ = other.usedTransform_; other.clear();
    }
};

} // namespace core3d::native_physical_working_frame
