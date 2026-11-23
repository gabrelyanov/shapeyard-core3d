#pragma once
// The TShape Checked bits of every distinct sub-shape of the given shapes, put back when this
// object goes out of scope (also on early exits). Taking a part's picture off the screen and
// putting it back (Display / Remove / Redisplay) re-meshes it and re-marks its shapes; the
// mark is written into the saved bytes, so a preview + Cancel would change a stored part that
// the user never changed (Fable engine review 16, item 1, 23 Nov 2026; Shell has the same guard
// since c6c65672). Same idiom as StoredShapeCacheBits in OcctDocument.mm.
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_TShape.hxx>
#include <utility>
#include <vector>

namespace core3d {
class CheckedBitsGuard final {
public:
    explicit CheckedBitsGuard(const std::vector<TopoDS_Shape>& shapes) noexcept {
        try {
            for (const TopoDS_Shape& shape : shapes) {
                if (shape.IsNull()) continue;
                TopTools_IndexedMapOfShape all;
                TopExp::MapShapes(shape, all);
                if (bits_.size() + std::size_t(all.Extent()) > 400000) return;
                bits_.emplace_back(shape.TShape(), shape.TShape()->Checked());
                for (int i = 1; i <= all.Extent(); ++i)
                    if (!all(i).TShape().IsNull())
                        bits_.emplace_back(all(i).TShape(), all(i).TShape()->Checked());
            }
        } catch (...) { bits_.clear(); }
    }
    ~CheckedBitsGuard() {
        for (auto& entry : bits_)
            if (!entry.first.IsNull()) entry.first->Checked(entry.second);
    }
    CheckedBitsGuard(const CheckedBitsGuard&) = delete;
    CheckedBitsGuard& operator=(const CheckedBitsGuard&) = delete;
private:
    std::vector<std::pair<Handle(TopoDS_TShape), Standard_Boolean>> bits_;
};
} // namespace core3d
