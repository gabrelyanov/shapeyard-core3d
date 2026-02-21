//
//  ShapeInteractor.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#include "ShapeInteractor.hpp"
#include <TopExp_Explorer.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRep_Tool.hxx>
#include <StlAPI_Writer.hxx>
#include <RWObj_CafWriter.hxx>
#include <TCollection_AsciiString.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_Transform.hxx>

#include "../Export/obj/vectornd.h"
#include "../Export/obj/geometry.h"
#include "../Export/obj/importstl.h"
#include "../Export/obj/exportobj.h"

namespace core3d {

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
		if (abs(abs(_chamferValue) - abs(value)) < 1e-1) //todo: temporal throttle
			return true;

		bool isChamfer = value < 0 && (abs(value) > FLT_EPSILON);
		bool isFillet = value > 0 && (abs(value) > FLT_EPSILON);

        if (isChamfer) {
            _chamferValue = std::fmax(value / 100, -20);
        } else {
            _chamferValue = std::fmin(value / 100, 20);
        }

		bool correctResult = true;
		for (EdgesSelection &sel : _detectedEdges) {
			bool hasFails = false;
			bool useTemplate = true;
			if (!sel.detectedOwner.IsNull()) {
				Handle(AIS_Shape) ownerShape = Handle(AIS_Shape)::DownCast(sel.detectedOwner->Selectable());
				
				if (!ownerShape.IsNull() && sel.detectedOwner->IsSelected()) {
//					copyMaterial(ownerShape, Handle(AIS_Shape)::DownCast(sel.detectedOwner->Selectable()));
					bool hasAffectedEdges = false;
					myContext->Display (ownerShape, AIS_WireFrame, _topAbsSelMode, Standard_False);

					BRepFilletAPI_MakeFillet  MF(useTemplate ? ownerShape->Shape() : sel.tempFilletShapePrs->Shape());
					BRepFilletAPI_MakeChamfer MC(useTemplate ? ownerShape->Shape() : sel.tempFilletShapePrs->Shape());

					useTemplate = false; //use template once

					//proceed all affected edges
					for (const TopoDS_Edge& edge : sel.edges) {
						if (isChamfer) {
							MC.Add(abs(_chamferValue), edge);
							hasAffectedEdges = true;
						} else if (isFillet) {
							MF.Add(_chamferValue, edge);
							hasAffectedEdges = true;
						}
					}

					if (isChamfer) {
						try {
							MC.Build();
							if (!MC.IsDone())
								hasFails = true;
						} catch (Standard_Failure f) {
							isChamfer = false;
						}
					} else if (isFillet) {
						try {
							MF.Build();
							if (!MF.IsDone())
								hasFails = true;
						} catch (Standard_Failure f) {
							isFillet = false;
						}
					}

					if (hasAffectedEdges && !hasFails) {
						if (isChamfer)
							sel.tempFilletShapePrs = new AIS_Shape(MC.Shape());
						else if (isFillet) //fillet
							sel.tempFilletShapePrs = new AIS_Shape(MF.Shape());
						else
							sel.tempFilletShapePrs = new AIS_Shape(ownerShape->Shape());
					} else { //reset fillet or chamfer
						sel.tempFilletShapePrs = new AIS_Shape(ownerShape->Shape());
						if (hasFails)
							correctResult = false;
					}
					copyMaterial(sel.tempFilletShapePrs, Handle(AIS_Shape)::DownCast(sel.detectedOwner->Selectable()));
				}
			}
		}

        if (!correctResult) {
            return false;
        }

		//update shape with new fillet
        auto doc = myDoc->ChangeDocument();
        if(doc->HasOpenCommand()) {
            doc->AbortCommand();
        }
        doc->NewCommand();

		for (EdgesSelection &sel : _detectedEdges) {
			if (!sel.tempFilletShapePrs.IsNull()) {
                if (!sel.filletShapePrs.IsNull()) {
                    
                    myDoc->RemoveShape(sel.filletShapePrs);
                    myContext->Remove(sel.filletShapePrs, Standard_False);
                }
				sel.filletShapePrs = new AIS_Shape(sel.tempFilletShapePrs->Shape());
				sel.filletShapePrs->SetLocalTransformation(sel.transform);
				copyMaterial(sel.filletShapePrs, sel.tempFilletShapePrs);

                // add to doc
                myDoc->AddShape(sel.filletShapePrs);
//                sel.filletShapePrs->SetMaterial(Graphic3d_NameOfMaterial_ShinyPlastified);
//                sel.filletShapePrs->SetColor(Quantity_NOC_GRAY80);

				sel.tempFilletShapePrs = Handle(AIS_Shape)(NULL);
				myContext->Display (sel.filletShapePrs, AIS_Shaded, _topAbsSelMode, Standard_False);
				setInteractiveObjectSelectionMode(sel.filletShapePrs);
			}
		}
        
		return correctResult;
	}

	void ShapeInteractor::resetWireframeTemplateShape() {
		for (EdgesSelection &sel : _detectedEdges) {
			if (!sel.detectedOwner.IsNull() && sel.detectedOwner->HasSelectable()) {
				Handle(AIS_Shape) ownerShape = Handle(AIS_Shape)::DownCast(sel.detectedOwner->Selectable());
				if (!ownerShape.IsNull() && !sel.filletShapePrs.IsNull()) {
                    myDoc->RemoveShape(ownerShape);
					myContext->Remove(ownerShape, Standard_True);
					ownerShape = Handle(AIS_Shape)(NULL);
				}
			}
			sel.edges.clear();
		}
		//std::cout << "clear edges count=" << _detectedEdges.size() << std::endl;
		_detectedEdges.clear();
        myDoc->ChangeDocument()->CommitCommand();
        myDoc->NotifyChanges();
	}

    Standard_Size ShapeInteractor::saveSelectionEdges(bool preventRechamfer) {
	// checkFilleted:
	//      - false - enable the chamfer functionality anyway
	//      - true - disable the chamfer functionality if it has already been used
        
        resetWireframeTemplateShape();

        const auto &detEdges = _detectedEdges;
        auto findOwner = [&detEdges](const TopoDS_Shape &theOther) -> Standard_Integer {
            for (int i = 0; i < detEdges.size(); ++i) {
                Handle(AIS_Shape) ownerShape = Handle(AIS_Shape)::DownCast(detEdges[i].detectedOwner->Selectable());
                if (ownerShape->Shape().IsEqual(theOther))
                    return i;
            }
            return -1;
        };

        for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
            const Handle(SelectMgr_EntityOwner) detectedOwner = myContext->SelectedOwner();
            Handle(AIS_Shape) ownerShape = Handle(AIS_Shape)::DownCast(detectedOwner->Selectable());
            
            const Handle(StdSelect_BRepOwner) &aBRepOwnerOfSelection = Handle(StdSelect_BRepOwner)::DownCast(detectedOwner);

            int found = findOwner(ownerShape->Shape());
            if (found == -1) {
                EdgesSelection sel;
                sel.transform = ownerShape->LocalTransformation();
                sel.detectedOwner = detectedOwner;
                sel.tempFilletShapePrs = Handle(AIS_Shape)(NULL);
                sel.filletShapePrs = Handle(AIS_Shape)(NULL);
                _detectedEdges.push_back(sel);
            }

            int i = 0;
            auto selectedType = aBRepOwnerOfSelection->Shape().ShapeType();
            if (aBRepOwnerOfSelection->IsSelected() && (selectedType == _topAbsSelMode || (selectedType == TopAbs_SOLID && _topAbsSelMode == TopAbs_SHAPE))) {
                for (TopExp_Explorer exp (aBRepOwnerOfSelection->Shape(), TopAbs_EDGE); exp.More(); exp.Next()) {
                    if(exp.Current().ShapeType() == TopAbs_EDGE) {
                        auto &edge = TopoDS::Edge(exp.Current());
                        Standard_Real f, l; //first and last parameter
                        Handle_Geom_Curve curve = BRep_Tool::Curve(edge, f, l);
						if (!curve.IsNull()) {
							GeomAbs_CurveType cType = GeomAdaptor_Curve(curve).GetType();
							bool isChamfable = !curve.IsNull() && ((GeomAbs_Line == cType) || (GeomAbs_BSplineCurve == cType) || GeomAdaptor_Curve(curve).IsClosed());
							if(isChamfable || !preventRechamfer) {
								_detectedEdges.back().edges.push_back(edge);
								++i;
							}
//							std::cout << "is chamfable edge-" << isChamfable << ", total=" << i << std::endl;
						}
                    }
                }
            }
            if (i > 64 || i == 0) //todo: debug limitiations
                _detectedEdges.pop_back();
        }
        // undo commit
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
		myContext->Deactivate(aio, aio->GlobalSelectionMode());//(Standard_Integer)_previousSelectionMode);
		myContext->Activate(aio, (Standard_Integer)_previousSelectionMode,  Standard_True);
        myContext->SetSelectionModeActive (aio, AIS_Shape::SelectionMode (_topAbsSelMode), true, AIS_SelectionModesConcurrency::AIS_SelectionModesConcurrency_Single);
		myContext->Activate((Standard_Integer)_previousSelectionMode);
	}

    const ShapeSelectionMode ShapeInteractor::getSelectionMode() const {
        return _previousSelectionMode;
    }

    const Standard_Boolean ShapeInteractor::isEmptyOfDisplayedObjects() const {
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(objects);
        return objects.IsEmpty();
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
		myDoc->ChangeDocument()->NewCommand();
        myDoc->ApplyTransforms();
        auto writer = RWObj_CafWriter(TCollection_AsciiString(filename.c_str()));
        TColStd_IndexedDataMapOfStringString aFileInfo;
        aFileInfo.Add("Author", "Shapeyard 3D");
        writer.Perform(myDoc->Document(), aFileInfo, Message_ProgressRange());
		myDoc->ChangeDocument()->AbortCommand();
#endif
    }
}
