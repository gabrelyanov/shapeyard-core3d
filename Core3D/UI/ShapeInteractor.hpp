//
//  ShapeInteractor.hpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#ifndef ShapeInteractor_hpp
#define ShapeInteractor_hpp

#include "Interactor.hpp"
#include <AIS_Shape.hxx>
#include <TopoDS.hxx>
#include <TDocStd_Document.hxx>

namespace core3d {

	//flag for don't chamfer or fillet for already filleted shape
	#define PREVENT_RECHAMFER true

    enum struct ShapeSelectionMode {
        WholeShape = 0,
        Vertex,
        Edge,
        Wire,
        Face
    };

	struct EdgesSelection {
		Handle(SelectMgr_EntityOwner) detectedOwner;
		TDF_Label documentLabel;
		std::vector<TopoDS_Edge> edges;
		Handle(AIS_Shape) tempFilletShapePrs;
		Handle(AIS_Shape) filletShapePrs;
		gp_Trsf transform;
		Graphic3d_NameOfMaterial materialName = Graphic3d_NameOfMaterial_ShinyPlastified;
		Quantity_NameOfColor colorName = Quantity_NOC_GRAY80;
	};

    class ShapeInteractor : public Interactor {
        ShapeSelectionMode _previousSelectionMode = ShapeSelectionMode::WholeShape; //whole shape
		TopAbs_ShapeEnum _topAbsSelMode = TopAbs_ShapeEnum::TopAbs_SHAPE;
		Handle(AIS_Shape) _temporalChamferShapePrs;
		Handle(AIS_InteractiveObject) _subtractorObjectPrs;
		std::vector<EdgesSelection> _detectedEdges;
        Standard_Real _chamferValue = 0;
		Standard_Boolean _ownsChamferCommand = Standard_False;

        void extractGeometryShapes(const TopoDS_Shape &shape,
                                   std::vector<TopoDS_Face> &faces,
                                   std::vector<TopoDS_Edge> &edges,
                                   std::vector<TopoDS_Vertex> &vertices);
    public:
        ShapeInteractor() = delete;
        ShapeInteractor(Handle(Core3DContext), Handle(Core3DView), Handle(OcctDocument) doc);
        
        const size_t getNumberOfDetectedEdges() const;
        void setSelectionMode(const ShapeSelectionMode mode);
        const ShapeSelectionMode getSelectionMode() const;
		Standard_Boolean setChamferValueForSelection(const Standard_Real value);
		void resetWireframeTemplateShape();
		void cancelChamfer();
		Standard_Size saveSelectionEdges(bool preventRechamfer = PREVENT_RECHAMFER);
		const Standard_Boolean isEmptyOfDisplayedObjects() const;
		const Standard_Size getNumberOfDisplayedShapes() const;
        void exportShapes();
        void exportToStl(const std::string &filename, const Standard_Boolean isASCII = Standard_True);
        void exportToObj(const std::string &filename);
        void exportToGltf(const std::string &filename);
        void exportToStep(const std::string &filename);

	private:
		void setInteractiveObjectSelectionMode(const Handle(AIS_InteractiveObject) aio);
		void copyMaterial(Handle(AIS_Shape) &to, const Handle(AIS_Shape) &from);
		void discardChamferPreview();

    };
}


#endif /* ShapeInteractor_hpp */
