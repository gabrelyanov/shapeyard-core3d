//
//  ShapeInteractor.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#include "ShapeInteractor.hpp"
#include "../OCCTKit/Core3DSTEPExchangeLock.h"
#include <TopExp_Explorer.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRep_Tool.hxx>
#include <StlAPI_Writer.hxx>
#include <RWObj_CafWriter.hxx>
#include <RWGltf_CafWriter.hxx>
#include <STEPControl_Writer.hxx>
#include <Interface_Static.hxx>
#include <TCollection_AsciiString.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <TDF_LabelSequence.hxx>
#include <BRepTools.hxx>
#include <Prs3d_Drawer.hxx>
#include <StdPrs_ToolTriangulatedShape.hxx>
#include <Standard_Failure.hxx>
#include <Standard_ErrorHandler.hxx>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdio>

#include "../Export/obj/vectornd.h"
#include "../Export/obj/geometry.h"
#include "../Export/obj/importstl.h"
#include "../Export/obj/exportobj.h"

namespace core3d {
	namespace {
		bool CanExportCommittedDocument(
			const Handle(OcctDocument)& document,
			const std::string& filename) {
			const Handle(TDocStd_Document) transaction = document.IsNull()
				? Handle(TDocStd_Document)()
				: document->ChangeDocument();
			if (filename.empty() || transaction.IsNull()
				|| transaction->HasOpenCommand()) {
				if (!filename.empty()) {
					std::remove(filename.c_str());
				}
				return false;
			}
			return true;
		}

		Standard_Boolean IsTopologicallyValid(const TopoDS_Shape& shape) {
			if (shape.IsNull()) {
				return Standard_False;
			}
			try {
				BRepCheck_Analyzer analyzer(shape, Standard_True);
				return analyzer.IsValid();
			} catch (...) {
				return Standard_False;
			}
		}

		void AbortOpenCommandNoThrow(
			const Handle(TDocStd_Document)& document) noexcept {
			try {
				if (!document.IsNull() && document->HasOpenCommand()) {
					document->AbortCommand();
				}
			} catch (...) {
			}
		}
	}

    ShapeInteractor::ShapeInteractor(Handle(Core3DContext) context, Handle(Core3DView) view, Handle(OcctDocument) doc)
    : Interactor(context, view, doc) {

    }

	void ShapeInteractor::copyMaterial(Handle(AIS_Shape) &to, const Handle(AIS_Shape) &from) {
		to->UnsetColor();
		to->SetMaterial(myDoc->MaterialNameForShape(from));
		Quantity_Color c;
		from->Color(c);
		to->SetColor(c.Name());
	}

	Standard_Boolean ShapeInteractor::setChamferValueForSelection(const Standard_Real value) {
		auto doc = myDoc->ChangeDocument();
		if (doc.IsNull() || (doc->HasOpenCommand() && !_ownsChamferCommand)) {
			return Standard_False;
		}
		const Standard_Real normalizedValue = value < 0
			? std::fmax(value / 100.0, -20.0)
			: std::fmin(value / 100.0, 20.0);
		if (std::abs(_chamferValue - normalizedValue) < 1e-3) {
			return Standard_True;
		}
		if (std::abs(normalizedValue) <= FLT_EPSILON) {
			discardChamferPreview();
			return Standard_True;
		}

		const Standard_Boolean isChamfer = normalizedValue < 0;

		struct ChamferResult {
			Standard_Size selectionIndex;
			Handle(AIS_Shape) presentation;
		};
		std::vector<ChamferResult> results;
		results.reserve(_detectedEdges.size());
		try {
			OCC_CATCH_SIGNALS
			for (Standard_Size index = 0; index < _detectedEdges.size(); ++index) {
				const EdgesSelection& sel = _detectedEdges[index];
				if (sel.detectedOwner.IsNull() || !sel.detectedOwner->HasSelectable()
					|| sel.documentLabel.IsNull() || sel.edges.empty()) {
					return Standard_False;
				}
				Handle(AIS_Shape) ownerShape =
					Handle(AIS_Shape)::DownCast(sel.detectedOwner->Selectable());
				if (ownerShape.IsNull() || ownerShape->Shape().IsNull()) {
					return Standard_False;
				}

				TopoDS_Shape resultShape;
				if (isChamfer) {
					BRepFilletAPI_MakeChamfer builder(ownerShape->Shape());
					for (const TopoDS_Edge& edge : sel.edges) {
						builder.Add(std::abs(normalizedValue), edge);
					}
					builder.Build();
					if (!builder.IsDone()) { return Standard_False; }
					resultShape = builder.Shape();
				} else {
					BRepFilletAPI_MakeFillet builder(ownerShape->Shape());
					for (const TopoDS_Edge& edge : sel.edges) {
						builder.Add(normalizedValue, edge);
					}
					builder.Build();
					if (!builder.IsDone()) { return Standard_False; }
					resultShape = builder.Shape();
				}
				if (!IsTopologicallyValid(resultShape)) { return Standard_False; }

				Handle(AIS_Shape) result = new AIS_Shape(resultShape);
				result->SetLocalTransformation(sel.transform);
				myDoc->LoadObjectMeterial(sel.documentLabel, result);
				results.push_back({index, result});
			}
		} catch (const Standard_Failure&) {
			return Standard_False;
		} catch (...) {
			return Standard_False;
		}
		if (results.empty() || results.size() != _detectedEdges.size()) {
			return Standard_False;
		}

		discardChamferPreview();
		try {
			OCC_CATCH_SIGNALS
			doc->NewCommand();
			if (!doc->HasOpenCommand()) {
				return Standard_False;
			}
			_ownsChamferCommand = Standard_True;
			for (const ChamferResult& result : results) {
				const EdgesSelection& sel = _detectedEdges[result.selectionIndex];
				const TDF_Label resultLabel = myDoc->AddShape(result.presentation);
				if (resultLabel.IsNull()) {
					throw Standard_Failure("Unable to add chamfer result");
				}
				if (!myDoc->CopyObjectAppearance(
						sel.documentLabel, resultLabel)) {
					throw Standard_Failure(
						"Unable to copy chamfer result appearance");
				}
			}
		} catch (...) {
			if (_ownsChamferCommand && doc->HasOpenCommand()) {
				doc->AbortCommand();
			}
			_ownsChamferCommand = Standard_False;
			return Standard_False;
		}

		for (const ChamferResult& result : results) {
			EdgesSelection& sel = _detectedEdges[result.selectionIndex];
			if (!sel.detectedOwner.IsNull() && sel.detectedOwner->HasSelectable()) {
				Handle(AIS_Shape) ownerShape =
					Handle(AIS_Shape)::DownCast(sel.detectedOwner->Selectable());
				if (!ownerShape.IsNull()) {
					myContext->Display(ownerShape, AIS_WireFrame, _topAbsSelMode, Standard_False);
				}
			}
			sel.filletShapePrs = result.presentation;
			myContext->Display(sel.filletShapePrs, AIS_Shaded, _topAbsSelMode, Standard_False);
			setInteractiveObjectSelectionMode(sel.filletShapePrs);
		}
		_chamferValue = normalizedValue;
		myContext->UpdateCurrentViewer();
		return Standard_True;
	}

	void ShapeInteractor::discardChamferPreview() {
		auto doc = myDoc->ChangeDocument();
		if (_ownsChamferCommand && !doc.IsNull() && doc->HasOpenCommand()) {
			doc->AbortCommand();
		}
		_ownsChamferCommand = Standard_False;
		for (EdgesSelection& sel : _detectedEdges) {
			if (!sel.tempFilletShapePrs.IsNull()) {
				myContext->Remove(sel.tempFilletShapePrs, Standard_False);
				sel.tempFilletShapePrs.Nullify();
			}
			if (!sel.filletShapePrs.IsNull()) {
				myContext->Remove(sel.filletShapePrs, Standard_False);
				sel.filletShapePrs.Nullify();
			}
			if (!sel.detectedOwner.IsNull() && sel.detectedOwner->HasSelectable()) {
				Handle(AIS_Shape) ownerShape =
					Handle(AIS_Shape)::DownCast(sel.detectedOwner->Selectable());
				if (!ownerShape.IsNull()) {
					myContext->Display(ownerShape, AIS_Shaded, _topAbsSelMode, Standard_False);
					setInteractiveObjectSelectionMode(ownerShape);
				}
			}
		}
		_chamferValue = 0;
		myContext->UpdateCurrentViewer();
	}

	void ShapeInteractor::cancelChamfer() {
		discardChamferPreview();
		for (EdgesSelection& sel : _detectedEdges) {
			sel.edges.clear();
		}
		_detectedEdges.clear();
	}

	void ShapeInteractor::resetWireframeTemplateShape() {
		auto doc = myDoc->ChangeDocument();
		if (!_ownsChamferCommand || doc.IsNull() || !doc->HasOpenCommand()) {
			cancelChamfer();
			return;
		}

		for (const EdgesSelection& sel : _detectedEdges) {
			if (sel.documentLabel.IsNull() || sel.filletShapePrs.IsNull()) {
				cancelChamfer();
				return;
			}
		}

		Standard_Boolean removedAll = Standard_True;
		try {
			OCC_CATCH_SIGNALS
			for (const EdgesSelection& sel : _detectedEdges) {
				if (!myDoc->RemoveShape(sel.documentLabel)) {
					removedAll = Standard_False;
					break;
				}
			}
		} catch (...) {
			removedAll = Standard_False;
		}
		if (!removedAll) {
			cancelChamfer();
			return;
		}

		Standard_Boolean committed = Standard_False;
		try {
			committed = doc->CommitCommand();
		} catch (...) {
			committed = Standard_False;
		}
		if (!committed) {
			cancelChamfer();
			return;
		}

		_ownsChamferCommand = Standard_False;
		for (EdgesSelection& sel : _detectedEdges) {
			if (!sel.detectedOwner.IsNull() && sel.detectedOwner->HasSelectable()) {
				Handle(AIS_Shape) ownerShape =
					Handle(AIS_Shape)::DownCast(sel.detectedOwner->Selectable());
				if (!ownerShape.IsNull()) {
					myContext->Remove(ownerShape, Standard_False);
				}
			}
			if (!sel.filletShapePrs.IsNull()) {
				myContext->Display(sel.filletShapePrs, AIS_Shaded, _topAbsSelMode, Standard_False);
				setInteractiveObjectSelectionMode(sel.filletShapePrs);
			}
			sel.edges.clear();
		}
		_detectedEdges.clear();
		_chamferValue = 0;
		myDoc->NotifyChanges();
		myContext->UpdateCurrentViewer();
	}

    Standard_Size ShapeInteractor::saveSelectionEdges(bool preventRechamfer) {
	// checkFilleted:
	//      - false - enable the chamfer functionality anyway
	//      - true - disable the chamfer functionality if it has already been used
        
        resetWireframeTemplateShape();

        for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
            const Handle(SelectMgr_EntityOwner) detectedOwner = myContext->SelectedOwner();
			if (detectedOwner.IsNull() || !detectedOwner->HasSelectable()) { continue; }
            Handle(AIS_Shape) ownerShape = Handle(AIS_Shape)::DownCast(detectedOwner->Selectable());
			if (ownerShape.IsNull() || ownerShape->Shape().IsNull()) { continue; }
            const Handle(StdSelect_BRepOwner) &aBRepOwnerOfSelection = Handle(StdSelect_BRepOwner)::DownCast(detectedOwner);
			if (aBRepOwnerOfSelection.IsNull()) { continue; }

			Standard_Integer found = -1;
			for (Standard_Size index = 0; index < _detectedEdges.size(); ++index) {
				if (_detectedEdges[index].detectedOwner.IsNull()
					|| !_detectedEdges[index].detectedOwner->HasSelectable()) { continue; }
				Handle(AIS_Shape) existing = Handle(AIS_Shape)::DownCast(
					_detectedEdges[index].detectedOwner->Selectable());
				if (!existing.IsNull() && existing->Shape().IsSame(ownerShape->Shape())) {
					found = static_cast<Standard_Integer>(index);
					break;
				}
			}
			const Standard_Boolean isNewSelection = found < 0;
			if (isNewSelection) {
				EdgesSelection sel;
				sel.documentLabel = myDoc->ShapeLabel(ownerShape);
				if (sel.documentLabel.IsNull()) { continue; }
				sel.transform = ownerShape->LocalTransformation();
				sel.materialName = myDoc->MaterialNameForLabel(sel.documentLabel);
				sel.colorName = myDoc->ColorNameForLabel(sel.documentLabel);
				sel.detectedOwner = detectedOwner;
				_detectedEdges.push_back(sel);
				found = static_cast<Standard_Integer>(_detectedEdges.size() - 1);
			}

			EdgesSelection& selection = _detectedEdges[found];
			const Standard_Size originalEdgeCount = selection.edges.size();
            auto selectedType = aBRepOwnerOfSelection->Shape().ShapeType();
            if (aBRepOwnerOfSelection->IsSelected() && (selectedType == _topAbsSelMode || (selectedType == TopAbs_SOLID && _topAbsSelMode == TopAbs_SHAPE))) {
				auto addEdge = [&](const TopoDS_Edge& edge) {
					Standard_Real first = 0;
					Standard_Real last = 0;
					Handle_Geom_Curve curve = BRep_Tool::Curve(edge, first, last);
					if (curve.IsNull()) { return; }
					GeomAdaptor_Curve adaptor(curve);
					const GeomAbs_CurveType curveType = adaptor.GetType();
					const Standard_Boolean isChamferable = curveType == GeomAbs_Line
						|| curveType == GeomAbs_BSplineCurve || adaptor.IsClosed();
					if (!isChamferable && preventRechamfer) { return; }
					for (const TopoDS_Edge& existing : selection.edges) {
						if (existing.IsSame(edge)) { return; }
					}
					selection.edges.push_back(edge);
				};
				const TopoDS_Shape selectedShape = aBRepOwnerOfSelection->Shape();
				if (selectedShape.ShapeType() == TopAbs_EDGE) {
					addEdge(TopoDS::Edge(selectedShape));
				} else {
					for (TopExp_Explorer exp(selectedShape, TopAbs_EDGE); exp.More(); exp.Next()) {
						addEdge(TopoDS::Edge(exp.Current()));
					}
				}
            }
			if (selection.edges.size() > 64) {
				selection.edges.resize(originalEdgeCount);
			}
			if (isNewSelection && selection.edges.empty()) {
				_detectedEdges.erase(_detectedEdges.begin() + found);
			}
        }
        return _detectedEdges.size();
    }

    const size_t ShapeInteractor::getNumberOfDetectedEdges() const {
        return _detectedEdges.size();
    }

    void ShapeInteractor::setSelectionMode(ShapeSelectionMode mode) {
        switch (mode) {
            case ShapeSelectionMode::Edge:
            case ShapeSelectionMode::Vertex:
                myContext->SetPixelTolerance(32);
                break;
            default:
                myContext->SetPixelTolerance();
                break;
        }

		if (_previousSelectionMode != mode) {
			myContext->ClearSelected(Standard_True);
			resetWireframeTemplateShape();
		}
        else {
            return;
        }

		TopAbs_ShapeEnum selMode;
		switch (mode) {
			case ShapeSelectionMode::WholeShape:
				selMode = TopAbs_ShapeEnum::TopAbs_SHAPE;
				break;
			case ShapeSelectionMode::Face:
				selMode = TopAbs_ShapeEnum::TopAbs_FACE;
				break;
			case ShapeSelectionMode::Edge:
				selMode = TopAbs_ShapeEnum::TopAbs_EDGE;
				break;
			case ShapeSelectionMode::Vertex:
				selMode = TopAbs_ShapeEnum::TopAbs_VERTEX;
				break;
			default:
				selMode = TopAbs_ShapeEnum::TopAbs_SHAPE;
				break;
		}

		AIS_ListOfInteractive objects;
		myContext->DisplayedObjects(objects);
		AIS_ListIteratorOfListOfInteractive iobject(objects);
		_previousSelectionMode = mode;
		_topAbsSelMode = selMode;

		while (iobject.More()) {
			setInteractiveObjectSelectionMode(iobject.Value());
			iobject.Next();
		}

		std::cout << "set selection mode=" << selMode << std::endl;
    }

	void ShapeInteractor::setInteractiveObjectSelectionMode(const Handle(AIS_InteractiveObject) aio) {
		if (aio.IsNull() || !myDoc->IsPresentationEditable(aio)) {
			if (!aio.IsNull()) {
				myContext->Deactivate(aio);
			}
			return;
		}
		myContext->Deactivate(aio, aio->GlobalSelectionMode());//(Standard_Integer)_previousSelectionMode);
		myContext->Activate(aio, (Standard_Integer)_previousSelectionMode,  Standard_True);
		myContext->SetSelectionModeActive (aio, AIS_Shape::SelectionMode (_topAbsSelMode), true, AIS_SelectionModesConcurrency::AIS_SelectionModesConcurrency_Single);
	}

    const ShapeSelectionMode ShapeInteractor::getSelectionMode() const {
        return _previousSelectionMode;
    }

    const Standard_Boolean ShapeInteractor::isEmptyOfDisplayedObjects() const {
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(objects);
        return objects.IsEmpty();
    }

    const Standard_Size ShapeInteractor::getNumberOfDisplayedShapes() const {
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        return objects.Size();
    }

    void ShapeInteractor::exportShapes() {
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        AIS_ListIteratorOfListOfInteractive iobject(objects);

        while (iobject.More()) {
            printf(">>> Export shape\n");

            std::vector<TopoDS_Face>   faces;
            std::vector<TopoDS_Edge>   edges;
            std::vector<TopoDS_Vertex> vertices;
            TopoDS_Shape myShape = Handle(AIS_Shape)::DownCast(iobject.Value())->Shape();
            extractGeometryShapes(myShape, faces, edges, vertices);

            iobject.Next();
        }
    }

    void ShapeInteractor::extractGeometryShapes(const TopoDS_Shape &shape,
                                                std::vector<TopoDS_Face> &faces,
                                                std::vector<TopoDS_Edge> &edges,
                                                std::vector<TopoDS_Vertex> &vertices) {
        faces.resize(0);
        edges.resize(0);
        vertices.resize(0);

        TopExp_Explorer exp;
        for (exp.Init(shape, TopAbs_FACE); exp.More(); exp.Next()) {
            faces.push_back(TopoDS::Face(exp.Current()));
        }
        for (exp.Init(shape, TopAbs_EDGE); exp.More(); exp.Next()) {
            edges.push_back(TopoDS::Edge(exp.Current()));
        }
        for (exp.Init(shape, TopAbs_VERTEX); exp.More(); exp.Next()) {
            vertices.push_back(TopoDS::Vertex(exp.Current()));
            gp_Pnt pnt = BRep_Tool::Pnt(TopoDS::Vertex(exp.Current()));
            printf(">>> Vertex: %f, %f, %f\n", pnt.X(), pnt.Y(), pnt.Z());
        }
    }

    void ShapeInteractor::exportToStl(const std::string &filename, const Standard_Boolean isASCII/* = Standard_True*/) {
        if (!CanExportCommittedDocument(myDoc, filename)) {
            return;
        }
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        AIS_ListIteratorOfListOfInteractive iobject(objects);

        TopoDS_Compound resultShape;
        BRep_Builder builder;
        builder.MakeCompound(resultShape);

        while (iobject.More()) {
            TopoDS_Shape shape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(iobject.Value())->Shape(),
                                                          iobject.Value()->LocalTransformation());
            builder.Add(resultShape, shape);
            iobject.Next();
        }

        StlAPI_Writer aStlWriter;
        aStlWriter.ASCIIMode() = isASCII;
        if (!aStlWriter.Write(resultShape, filename.c_str())) {
            printf("Error creating STL file: %s\n", filename.c_str());
        }
    }

    void ShapeInteractor::exportToObj(const std::string &filename) {
        if (!CanExportCommittedDocument(myDoc, filename)) {
            return;
        }
//#define converter2obj
#ifdef converter2obj
        const auto tmp_stl = filename + ".tmp";
        exportToStl(tmp_stl, Standard_False);

        //  create a geometry tesselation object
        Geometry tessel;
        //  fill up the tesselation object with STL data (load STL)
        tessel.visit (ImportSTL (tmp_stl));
        //  write down the tesselation object into OBJ file (save OBJ)
        tessel.visit (ExportOBJ (filename));
        printf("OBJ file: %s\n", filename.c_str());
#else
        // OBJ export writes the mesh stored in the XCAF document. RWObj_CafWriter does
        // NOT tessellate — its header states "Triangulation data should be precomputed
        // within shapes!" — so any untriangulated face makes it raise a Standard_Failure.
        // Shapes authored or edited in-session are only tessellated for on-screen display,
        // never in the document, and because the write below used to be unguarded that
        // exception unwound across the Obj-C++/Swift boundary into std::terminate, instantly
        // quitting the app. (STL is unaffected: it reads the already-tessellated display
        // shapes and StlAPI_Writer meshes internally.) Mirror the model-load / glTF paths:
        // triangulate the free shapes first, and guard the whole operation so a failure
        // yields an empty file (handled as nil upstream) instead of crashing.
        myDoc->ChangeDocument()->NewCommand();
        bool exportSucceeded = false;

        try {
            myDoc->ApplyTransforms();

            Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(myDoc->Document()->Main());
            TDF_LabelSequence labels;
            shapeTool->GetFreeShapes(labels);

            if (labels.IsEmpty()) {
                // Nothing to export. RWObj_CafWriter creates the output file at
                // construction, so an empty document would otherwise leave a
                // header-only .obj that passes the non-empty guard upstream. Bail
                // out before the writer is constructed so no file is produced.
                printf("Error creating OBJ file (no shapes to export): %s\n", filename.c_str());
            } else {
                TopoDS_Compound compound;
                BRep_Builder builder;
                builder.MakeCompound(compound);
                for (Standard_Integer i = 1; i <= labels.Length(); ++i) {
                    TopoDS_Shape shape = shapeTool->GetShape(labels.Value(i));
                    if (!shape.IsNull()) {
                        builder.Add(compound, shape);
                    }
                }

                // Use the display-relative deflection (as the load path does) instead of a
                // fixed absolute value: on a large, mm-scale scene an absolute deflection
                // explodes the triangle count and can OOM the export on iPad.
                Handle(Prs3d_Drawer) drawer = myContext->DefaultDrawer();
                Standard_Real deflection = StdPrs_ToolTriangulatedShape::GetDeflection(compound, drawer);
                if (!BRepTools::Triangulation(compound, deflection)) {
                    BRepMesh_IncrementalMesh mesher;
                    mesher.ChangeParameters().Deflection = deflection;
                    mesher.ChangeParameters().Angle = drawer->DeviationAngle();
                    mesher.ChangeParameters().InParallel = Standard_True;
                    mesher.SetShape(compound);
                    mesher.Perform();
                }

                TColStd_IndexedDataMapOfStringString aFileInfo;
                aFileInfo.Add("Author", "Shapeyard 3D");

                auto writer = RWObj_CafWriter(TCollection_AsciiString(filename.c_str()));
                exportSucceeded = writer.Perform(myDoc->Document(), aFileInfo, Message_ProgressRange());
                if (!exportSucceeded) {
                    printf("Error creating OBJ file (writer reported failure): %s\n", filename.c_str());
                }
            }
        } catch (const Standard_Failure &theFailure) {
            printf("Error creating OBJ file (exception): %s [%s]\n", filename.c_str(), theFailure.GetMessageString());
        } catch (...) {
            printf("Error creating OBJ file (exception): %s\n", filename.c_str());
        }

        // Always roll back the transient NewCommand()/ApplyTransforms() so the live
        // document is left untouched for continued editing — even if the export above
        // threw. Leaving the command open would let a later NewCommand() commit stale
        // (double-transformed) geometry.
        AbortOpenCommandNoThrow(myDoc->ChangeDocument());

        // A failed or aborted write can leave a partial (non-empty) file behind, which
        // would pass the size check upstream and share a corrupt OBJ — remove it so
        // failure reliably surfaces as nil (no share sheet) instead.
        if (!exportSucceeded) {
            std::remove(filename.c_str());
        }
#endif
    }

    void ShapeInteractor::exportToGltf(const std::string &filename) {
        if (!CanExportCommittedDocument(myDoc, filename)) {
            return;
        }
        bool exportSucceeded = false;
        try {
            // Bake presentation transforms only inside a transient command;
            // every exit below aborts it so export cannot mutate the editor.
            myDoc->ChangeDocument()->NewCommand();
            myDoc->ApplyTransforms();

            Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(
                    myDoc->Document()->Main());
            TDF_LabelSequence labels;
            if (!shapeTool.IsNull()) {
                shapeTool->GetFreeShapes(labels);
            }
            if (!shapeTool.IsNull() && !labels.IsEmpty()) {
                TopoDS_Compound compound;
                BRep_Builder builder;
                builder.MakeCompound(compound);
                for (Standard_Integer i = 1; i <= labels.Length(); ++i) {
                    const TopoDS_Shape shape =
                        shapeTool->GetShape(labels.Value(i));
                    if (!shape.IsNull()) {
                        builder.Add(compound, shape);
                    }
                }
                Handle(Prs3d_Drawer) drawer = myContext->DefaultDrawer();
                const Standard_Real deflection =
                    StdPrs_ToolTriangulatedShape::GetDeflection(
                        compound, drawer);
                if (!BRepTools::Triangulation(compound, deflection)) {
                    BRepMesh_IncrementalMesh mesher;
                    mesher.ChangeParameters().Deflection = deflection;
                    mesher.ChangeParameters().Angle = drawer->DeviationAngle();
                    mesher.ChangeParameters().InParallel = Standard_True;
                    mesher.SetShape(compound);
                    mesher.Perform();
                }

                TColStd_IndexedDataMapOfStringString fileInfo;
                fileInfo.Add("Author", "Shapeyard 3D");
                RWGltf_CafWriter writer(
                    TCollection_AsciiString(filename.c_str()),
                    Standard_True);
                exportSucceeded = writer.Perform(
                    myDoc->Document(),
                    fileInfo,
                    Message_ProgressRange());
            }
        } catch (const Standard_Failure& failure) {
            printf("Error creating glTF file (exception): %s [%s]\n",
                   filename.c_str(), failure.GetMessageString());
        } catch (...) {
            printf("Error creating glTF file (exception): %s\n", filename.c_str());
        }

        AbortOpenCommandNoThrow(myDoc->ChangeDocument());
        if (!exportSucceeded) {
            std::remove(filename.c_str());
        }
    }

    void ShapeInteractor::exportToStep(const std::string &filename) {
        if (!CanExportCommittedDocument(myDoc, filename)) {
            return;
        }
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        AIS_ListIteratorOfListOfInteractive iobject(objects);

        TopoDS_Compound resultShape;
        BRep_Builder builder;
        builder.MakeCompound(resultShape);

        while (iobject.More()) {
            TopoDS_Shape shape = BRepBuilderAPI_Transform(
                Handle(AIS_Shape)::DownCast(iobject.Value())->Shape(),
                iobject.Value()->LocalTransformation());
            builder.Add(resultShape, shape);
            iobject.Next();
        }

        {
            std::lock_guard<std::mutex> stepExchangeLock(
                Core3DSTEPExchangeMutex());
            Interface_Static::SetCVal("write.step.schema", "AP214IS");
            STEPControl_Writer stepWriter;
            stepWriter.Transfer(resultShape, STEPControl_AsIs);
            if (stepWriter.Write(filename.c_str()) != IFSelect_RetDone) {
                printf("Error creating STEP file: %s\n", filename.c_str());
            }
        }
    }
}
