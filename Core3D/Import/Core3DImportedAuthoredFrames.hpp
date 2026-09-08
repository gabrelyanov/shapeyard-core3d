#pragma once
#include "Core3DGLBReader.hpp"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/AuthoredFrameAttributeID.hxx"
#include "../OCCTKit/NativeAuthoredFrameGeometry.hxx"
#include <TDataStd_ByteArray.hxx>
#include <TDF_LabelMap.hxx>
#include <TopoDS.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>

namespace core3d::gltf {
// Only call in the isolated worker after MarkImportedTriangleMeshDefinitions,
// before initial validation/identity migration. Never use on an adopted scene.
inline bool FinalizeImportedAuthoredFrames(OcctDocument& wrapper,
    const std::vector<GLBImportedFrames>& records, const std::atomic_bool* cancelled) noexcept {
    const auto document=wrapper.Document();
    const auto stopped=[&]() { return cancelled && cancelled->load(std::memory_order_acquire); };
    if (document.IsNull() || document->HasOpenCommand()
        || document->GetAvailableUndos()!=0 || document->GetAvailableRedos()!=0 || stopped()) return false;
    if (records.empty()) return true;
    if (records.size()>2048) return false;
    struct Command {
        Handle(TDocStd_Document) doc;
        int previousLimit=0;
        bool started=false;
        ~Command() noexcept {
            try {
                if (started) { if (doc->HasOpenCommand()) doc->AbortCommand(); doc->ClearUndos(); }
                doc->SetUndoLimit(previousLimit);
            } catch (...) {}
        }
    } command{document,document->GetUndoLimit(),false};
    try {
        Standard_Size existing=0;
        if (!Core3DValidateAuthoredFrameOwners(document,existing) || existing!=0) return false;
        const auto shapes=XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return false;
        TDF_LabelMap labels;
        Standard_Size planned=0;
        for (const auto& record:records) {
            if (stopped() || record.label.IsNull() || record.label.Data()!=document->GetData()
                || !labels.Add(record.label) || record.archive.empty()
                || record.archive.size()>core3d::scene::authored::kMaximumArchiveBytes) return false;
            const auto shape=shapes->GetShape(record.label);
            if (shape.IsNull() || shape.ShapeType()!=TopAbs_FACE) return false;
            std::vector<core3d::scene::Float4> frames;
            if (!core3d::persistence::DecodeNativeAuthoredFrames(TopoDS::Face(shape),
                    record.archive.data(),record.archive.size(),frames)) return false;
            const auto bytes=record.archive.size()+64U*frames.size();
            if (bytes>64U*1024U*1024U-planned) return false;
            planned+=bytes;
        }
        document->SetUndoLimit(1); command.started=true; document->NewCommand();
        if (!document->HasOpenCommand()) return false;
        std::vector<OcctPBRMaterialUpdate> normalBindings;
        normalBindings.reserve(records.size());
        for (const auto& record:records) {
            if (stopped()) return false;
            const auto attribute=TDataStd_ByteArray::Set(record.label,
                core3d::persistence::AuthoredFrameAttributeID(),0,int(record.archive.size())-1,Standard_False);
            for (std::size_t i=0;i<record.archive.size();++i) {
                if ((i & 4095U)==0 && stopped()) return false;
                attribute->SetValue(int(i),record.archive[i]);
            }
            OcctAuthoredFrameRecord owned;
            if (Core3DReadAuthoredFrameOwner(document,record.label,owned)!=OcctAuthoredFrameReadState::Authored
                || owned.archive!=record.archive) return false;
            const auto material=XCAFDoc_VisMaterialTool::GetShapeMaterial(record.label);
            if (!material.IsNull() && material->HasPbrMaterial() && !material->PbrMaterial().NormalTexture.IsNull())
                normalBindings.push_back({record.label,material->PbrMaterial(),Handle(Image_Texture)(),Handle(Image_Texture)()});
        }
        // One batch projects every final normal binding before mutation. This
        // installs canonical local ownership and exact recipe2 through the
        // production writer while preserving imported alpha/culling/materials.
        if (!normalBindings.empty() && !wrapper.SaveObjectPBRMaterials(normalBindings)) return false;
        Standard_Size actual=0;
        if (stopped() || !Core3DValidateOwnedFrameUsage(document,actual) || actual!=planned) return false;
        if (!document->CommitCommand()) return false;
        document->ClearUndos();
        return !document->HasOpenCommand() && document->GetAvailableUndos()==0
            && document->GetAvailableRedos()==0 && !stopped();
    } catch (...) { return false; }
}
} // namespace core3d::gltf
