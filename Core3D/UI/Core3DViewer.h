//
//  Core3DViewer.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 11.04.2024.
//

#ifndef Core3DViewer_H
#define Core3DViewer_H

#include "OcctViewer.h"
#include "AIS_Manipulator.hxx"

#import <Core3D/PrimitiveType.h>

#include "ObjectInteractor.hpp"
#include "ShapeInteractor.hpp"

#include "OrthoProjectionType.h"
#include "../Scene/OcctSceneSnapshotBuilder.hpp"

namespace core3d {
    typedef unsigned char selection_t;

    enum class AssetImportResult {
        Success = 0,
        InvalidData,
        TemporaryFileFailure,
        Busy,
        UnsupportedVersion,
        InternalFailure,
    };

    class Core3DViewer: public OcctViewer {
    public:
        static constexpr selection_t kSelectionTypeNone = 0;
        static constexpr selection_t kSelectionTypeManipulator = 1 << 0;
        static constexpr selection_t kSelectionTypeObject = 1 << 1;

        //! Release derived interactors before the base OCCT graphics handles.
        Standard_EXPORT void release() noexcept;

        Standard_EXPORT bool InitViewer (UIView* theWin);
        
        Standard_EXPORT NSString* addTestPrimitives();
        void addPrimitive(PrimitiveType primitiveType);
        void addPrimitivesFromJSON(NSString* json);
        
        void showGrid(bool show);
        
        void redraw();
        
        /***
         Interactions
         */
        void Select(int theX, int theY);
        void StartRotation(int theX, int theY);
        void Rotation(int theX, int theY);
        void FinishInteraction(int theX, int theY);
        void CancelInteraction(int theX, int theY);
        
        const bool hitTest(const int x, const int y) const;

        void deselectAll();
		const int selectedCount() const;

        std::shared_ptr<ObjectInteractor> getObjectInteractor();
        std::shared_ptr<ShapeInteractor> getShapeInteractor();
        Handle(OcctDocument) getDocument();
        bool dumpOfDisplayedColoredObjects(const Standard_Integer width,
                                           const Standard_Integer height,
                                           const TCollection_AsciiString& fileName);
        bool dumpOfDisplayedObjects(const Standard_Integer width,
                                    const Standard_Integer height,
                                    const TCollection_AsciiString& fileName);
        bool dumpShape(const TopoDS_Shape& shape,
                         const Standard_Integer width,
                         const Standard_Integer height,
                         const TCollection_AsciiString& fileName);
        bool saveSnapshot(const TCollection_AsciiString& thePath,
                            int theWidth,
                            int theHeight);
        
        AssetImportResult ImportCbf(const std::string &theFilename);
        AssetImportResult ValidateCbf(const std::string &theFilename) const;
        void redrawDocument();

        void setPreviewMode();
        inline void setInteractiveCallback(const std::function<void(int,int)> cb) {
            _interactiveCallback = cb;
        }

        void setOrthoProjection(const OrthoProjectionType orthoType);

        //! Capture committed OCAF geometry and semantic camera state into
        //! immutable renderer-neutral values. Main-thread only.
        scene::OcctSceneSnapshotBuilder::SnapshotPointer captureSceneSnapshot(
            std::uint32_t viewportWidth,
            std::uint32_t viewportHeight) noexcept;

        //! Capture only the current semantic camera and established revision
        //! vector. Main-thread only and constant with respect to mesh size.
        std::optional<scene::FrameSnapshot> captureSceneFrameSnapshot(
            std::uint32_t viewportWidth,
            std::uint32_t viewportHeight) noexcept;

        //! Capture an immutable idle move/rotate gizmo paired with the most
        //! recent full scene. Empty content is a valid explicit clear.
        scene::OcctSceneSnapshotBuilder::OverlayPointer
        captureScenePresentationOverlay() noexcept;
    private:
        // document traversal
        bool traverseDocument (const Handle(TDocStd_Document)& theDoc);
        bool traverseLabel (const Handle(TDocStd_Document)& theDoc,
                            const TDF_Label& theLabel,
                                            const TCollection_AsciiString& theNamePrefix,
                                            const TopLoc_Location& theLoc,
                                            MapOfPrsForShapes& theMapOfShapes);
        void recreateInteractors(PrimitiveManipulatorType theManipulatorType,
                                 ShapeSelectionMode theSelectionMode);

    private:
        std::shared_ptr<ObjectInteractor> _objectInteractor;
        std::shared_ptr<ShapeInteractor> _shapeInteractor;
        
        std::function<void(int,int)> _interactiveCallback;
        scene::OcctSceneSnapshotBuilder _sceneSnapshotBuilder;
    };
}

#endif // Core3DViewer_H
