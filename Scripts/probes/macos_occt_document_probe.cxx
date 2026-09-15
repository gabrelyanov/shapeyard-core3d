// Desktop dependency qualification only; this does not exercise Shapeyard UI.
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BinXCAFDrivers.hxx>
#include <GProp_GProps.hxx>
#include <RWGltf_CafWriter.hxx>
#include <Standard_Failure.hxx>
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

static void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    if (argc != 2) return 64;
    try {
        Handle(TDocStd_Application) app = new TDocStd_Application;
        BinXCAFDrivers::DefineFormat(app);
        Handle(TDocStd_Document) document;
        app->NewDocument("BinXCAF", document);
        require(!document.IsNull(), "document creation failed");
        auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        document->SetUndoLimit(4);
        document->NewCommand();
        shapes->AddShape(BRepPrimAPI_MakeBox(2.0, 3.0, 4.0).Shape(), false);
        require(document->CommitCommand(), "history commit failed");
        require(document->Undo(), "undo failed");
        TDF_LabelSequence labels;
        shapes->GetFreeShapes(labels);
        require(labels.IsEmpty(), "undo retained box");
        require(document->Redo(), "redo failed");
        labels.Clear();
        shapes->GetFreeShapes(labels);
        require(labels.Length() == 1, "redo did not restore box");
        const std::string nativePath = std::string(argv[1]) + "/box.xbf";
        require(app->SaveAs(document, TCollection_ExtendedString(nativePath.c_str())) == PCDM_SS_OK,
                "native save failed");
        app->Close(document);
        document.Nullify();
        shapes.Nullify();
        labels.Clear();
        require(app->Open(TCollection_ExtendedString(nativePath.c_str()), document) == PCDM_RS_OK,
                "native reopen failed");
        shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        shapes->GetFreeShapes(labels);
        require(labels.Length() == 1, "reopened shape count differs");
        const auto box = shapes->GetShape(labels.First());
        GProp_GProps properties;
        BRepGProp::VolumeProperties(box, properties);
        require(std::abs(properties.Mass() - 24.0) < 1.e-8, "reopened box volume differs");
        BRepMesh_IncrementalMesh mesh(box, 0.1);
        require(mesh.IsDone(), "box triangulation failed");
        const std::string glbPath = std::string(argv[1]) + "/box.glb";
        RWGltf_CafWriter writer(glbPath.c_str(), true);
        TColStd_IndexedDataMapOfStringString metadata;
        require(writer.Perform(document, metadata, Message_ProgressRange()), "GLB export failed");
        app->Close(document);
        std::cout << "PASS: macOS OCCT box, OCAF Undo/Redo, save/reopen, volume=24, GLB export\n";
        return 0;
    } catch (const Standard_Failure& error) {
        std::cerr << error.GetMessageString() << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
    }
    return 1;
}
