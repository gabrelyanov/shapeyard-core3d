// Keeps the face TShape Checked bits of every part as the document holds them,
// and puts them back when the scope ends (also on early exits).
//
// The Checked bit is a meshing cache (meshing sets it on a face it has healed),
// but it is written into the exact saved BRep stream, and a kept-originals Join
// or Cut proves its stored result byte for byte against a replay of its
// originals before every save. Anything that meshes a part again for drawing or
// picking after the operation committed (selecting the joined part, the
// selection refresh after Undo or Redo) must therefore not leave the bit
// changed, or the project refuses every later save (base lane, 17 Nov: Sphere
// Pattern source Union). Same rule as OcctDocument's StoredShapeCacheBits and
// Core3DViewer::redrawDocument, for the steps that run after them.
#pragma once

#include <Standard_Handle.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_TShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <cstddef>
#include <unordered_set>
#include <utility>
#include <vector>

namespace core3d {

class CommittedFaceCheckedBits final {
public:
    explicit CommittedFaceCheckedBits(const Handle(TDocStd_Document)& document) noexcept {
        try {
            if (document.IsNull()) return;
            const Handle(XCAFDoc_ShapeTool) shapes =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            if (shapes.IsNull()) return;
            TDF_LabelSequence roots;
            shapes->GetFreeShapes(roots);
            std::unordered_set<const TopoDS_TShape*> seen;
            constexpr std::size_t maximumFaces = 200000;
            for (int i = 1; i <= roots.Length(); ++i) {
                const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(roots.Value(i));
                for (TopExp_Explorer face(shape, TopAbs_FACE); face.More(); face.Next()) {
                    const Handle(TopoDS_TShape)& tshape = face.Current().TShape();
                    if (tshape.IsNull() || !seen.insert(tshape.get()).second) continue;
                    if (bits_.size() >= maximumFaces) { bits_.clear(); return; }
                    bits_.emplace_back(tshape, tshape->Checked());
                }
            }
        } catch (...) { bits_.clear(); }
    }
    ~CommittedFaceCheckedBits() {
        for (auto& entry : bits_)
            if (!entry.first.IsNull()) entry.first->Checked(entry.second);
    }
    CommittedFaceCheckedBits(const CommittedFaceCheckedBits&) = delete;
    CommittedFaceCheckedBits& operator=(const CommittedFaceCheckedBits&) = delete;

private:
    std::vector<std::pair<Handle(TopoDS_TShape), Standard_Boolean>> bits_;
};

} // namespace core3d
