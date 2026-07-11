// Copyright (c) 2017 OPEN CASCADE SAS
//
// This file is part of the examples of the Open CASCADE Technology software library.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE

#ifndef OcctDocument_h
#define OcctDocument_h

#include <XCAFApp_Application.hxx>
#include <TDocStd_Document.hxx>
#include <AIS_InteractiveObject.hxx>
#include <AIS_Shape.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <string>
//! The document
class OcctDocument : public Standard_Transient
{
  DEFINE_STANDARD_RTTIEXT(OcctDocument, Standard_Transient)
  
public:
  Standard_EXPORT OcctDocument();

  Standard_EXPORT virtual ~OcctDocument();

  Standard_EXPORT void InitDoc();

  //! Return persistent identifiers without modifying the document. An empty
  //! string means that the requested identifier has not been migrated yet.
  Standard_EXPORT std::string DocumentIdentifier() const;
  Standard_EXPORT std::string EntityIdentifierForLabel(const TDF_Label& label) const;
  Standard_EXPORT std::string DefinitionIdentifierForLabel(const TDF_Label& label) const;
  //! Return Shapeyard's persisted object-local translation/rotation/uniform
  //! scale. This is independent of an XCAF assembly occurrence location.
  Standard_EXPORT gp_Trsf ObjectTransformForLabel(const TDF_Label& label) const;

  //! Assign identifiers to a legacy document before normal editing begins.
  //! Migration is atomic, leaves no undo/redo entry, and refuses to run over
  //! an open command or existing user history. Snapshot/read paths must never
  //! call this method.
  Standard_EXPORT Standard_Boolean MigrateLegacyIdentifiers();
  Standard_EXPORT Standard_Boolean MigrateLegacyIdentifiers(
      const Handle(TDocStd_Document)& document);

    Handle(TDocStd_Document)& ChangeDocument() {
        return myOcafDoc;
    }

    const Handle(TDocStd_Document)& Document() const {
        return myOcafDoc;
    }

    void SaveObjectTransform(const TDF_Label& label, const Handle(AIS_Shape) anAis);
    void LoadObjectTransform(const TDF_Label& label, const Handle(AIS_Shape) anAis);

    void SaveObjectMaterial(Handle(AIS_Shape) object, const Graphic3d_NameOfMaterial name_of_material);
    void SaveObjectColor(Handle(AIS_Shape) object, const Quantity_NameOfColor name_of_color);
    void SaveObjectMaterial(const TDF_Label& label, const Graphic3d_NameOfMaterial);
    void SaveObjectColor(const TDF_Label& label, const Quantity_NameOfColor name_of_color);
    void LoadObjectMeterial(const TDF_Label& label, const Handle(AIS_Shape) anAis);

    TDF_Label AddShape(Handle(AIS_Shape) object);
    TDF_Label AddShape(Handle(AIS_InteractiveObject) object);
    TDF_Label ShapeLabel(Handle(AIS_InteractiveObject) object) const;
    
    Graphic3d_NameOfMaterial MaterialNameForShape(Handle(AIS_Shape) object);
    Graphic3d_NameOfMaterial MaterialNameForLabel(const TDF_Label& label) const;
    Quantity_NameOfColor ColorNameForLabel(const TDF_Label& label) const;
    Standard_Boolean TryMaterialNameForLabel(
        const TDF_Label& label,
        Graphic3d_NameOfMaterial& material) const;
    Standard_Boolean TryColorNameForLabel(
        const TDF_Label& label,
        Quantity_NameOfColor& color) const;

    void ReplaceShape(const TDF_Label& label, Handle(AIS_Shape) aisShape);
    
    void RemoveShape(TopoDS_Shape object);
    void RemoveShape(Handle(AIS_Shape) object);
    void RemoveShape(Handle(AIS_InteractiveObject) object);
    Standard_Boolean RemoveShape(const TDF_Label& label);
    
    void ApplyTransforms();

	Standard_Boolean undo();
	Standard_Boolean redo();
    const bool canUndo() const;
    const bool canRedo() const;
    
    std::string save(const std::string& path);

    void NotifyChanges();

private:
    
  Handle(TDocStd_Application) myApp;
  Handle(TDocStd_Document) myOcafDoc;
};

#endif // OcctDocument_h
