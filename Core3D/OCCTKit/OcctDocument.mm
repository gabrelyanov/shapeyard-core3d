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

#import <Foundation/Foundation.h>

#include "OcctDocument.h"

#include <Standard_ErrorHandler.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>

#include <TCollection_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <BinDrivers.hxx>

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDataStd_Real.hxx>
#include <gp_Trsf.hxx>
#include <GP_Quaternion.hxx>
#include <TNaming.hxx>

IMPLEMENT_STANDARD_RTTIEXT(OcctDocument, Standard_Transient)

// =======================================================================
// function : OcctViewer
// purpose  :
// =======================================================================
OcctDocument::OcctDocument()
{
  try
  {
    OCC_CATCH_SIGNALS
    myApp = new TDocStd_Application();
  }
  catch (const Standard_Failure& theFailure)
  {
    Message::SendFail (TCollection_AsciiString("Error in creating application") + theFailure.GetMessageString());
  }
}

// =======================================================================
// function : ~OcctDocument
// purpose  :
// =======================================================================
OcctDocument::~OcctDocument()
{
    std::cout << "~OcctDocument" << std::endl;
}

// =======================================================================
// function : InitDoc
// purpose  :
// =======================================================================
void OcctDocument::InitDoc()
{
    
    std::cout << "InitDoc()" << std::endl;
  // close old document
  if (!myOcafDoc.IsNull())
  {
    if (myOcafDoc->HasOpenCommand())
    {
      myOcafDoc->AbortCommand();
    }

    myOcafDoc->Main().Root().ForgetAllAttributes(Standard_True);
    myApp->Close(myOcafDoc);
    myOcafDoc.Nullify();
  }


  // create a new document
  myApp->NewDocument(TCollection_ExtendedString("BinOcaf"), myOcafDoc);
  BinDrivers::DefineFormat(myApp);

  // set maximum number of available "undo" actions
  if (!myOcafDoc.IsNull())
  {
    myOcafDoc->SetUndoLimit(40);

	// Create the persistent XCAF tools before the first undoable command. If the
	// first shape command creates these infrastructure attributes, Undo removes
	// them and a viewport redraw recreates them outside history; Redo then fails
	// because the same labels already carry those attributes.
	(void)XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
	(void)XCAFDoc_DocumentTool::ColorTool(myOcafDoc->Main());
  }
}

void OcctDocument::RemoveShape(Handle(AIS_InteractiveObject) object) {
    RemoveShape(ShapeLabel(object));
}

void OcctDocument::RemoveShape(Handle(AIS_Shape) aisShape) {
    RemoveShape(ShapeLabel(aisShape));
}

void OcctDocument::RemoveShape(TopoDS_Shape object) {
    if (myOcafDoc.IsNull() || object.IsNull()) {
        return;
    }
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label;
    if(shapeTool->FindShape(object, label)
       || shapeTool->FindShape(object, label, Standard_True)) {
        RemoveShape(label);
    }
}

TDF_Label OcctDocument::ShapeLabel(Handle(AIS_InteractiveObject) object) const {
    TDF_Label label;
    if (myOcafDoc.IsNull() || object.IsNull()) {
        return label;
    }

    Handle(AIS_Shape) aisShape = Handle(AIS_Shape)::DownCast(object);
    if (aisShape.IsNull() || aisShape->Shape().IsNull()) {
        return label;
    }

    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
    if (shapeTool.IsNull()) {
        return label;
    }
    shapeTool->FindShape(aisShape->Shape(), label)
        || shapeTool->FindShape(aisShape->Shape(), label, Standard_True);
    return label;
}

Standard_Boolean OcctDocument::RemoveShape(const TDF_Label& label) {
    if (myOcafDoc.IsNull() || label.IsNull()) {
        return Standard_False;
    }
    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
    return !shapeTool.IsNull() && shapeTool->RemoveShape(label, Standard_True);
}

TDF_Label OcctDocument::AddShape(Handle(AIS_InteractiveObject) object) {
    Handle(AIS_Shape) aisShape = Handle(AIS_Shape)::DownCast(object);
    return AddShape(aisShape);
}

TDF_Label OcctDocument::AddShape(Handle(AIS_Shape) aisShape) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label = shapeTool->NewShape();
    shapeTool->SetShape(label, aisShape->Shape());
    SaveObjectTransform(label, aisShape);
    return label;
}

void OcctDocument::ReplaceShape(const TDF_Label& label, Handle(AIS_Shape) aisShape) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    shapeTool->SetShape(label, aisShape->Shape());
    SaveObjectTransform(label, aisShape);
}

void OcctDocument::SaveObjectTransform(const TDF_Label& label, const Handle(AIS_Shape) anAis) {
    auto t = anAis->LocalTransformation();
    TDataStd_Real::Set(label.FindChild(1), t.TranslationPart().X());
    TDataStd_Real::Set(label.FindChild(2), t.TranslationPart().Y());
    TDataStd_Real::Set(label.FindChild(3), t.TranslationPart().Z());
    TDataStd_Real::Set(label.FindChild(4), t.GetRotation().X());
    TDataStd_Real::Set(label.FindChild(5), t.GetRotation().Y());
    TDataStd_Real::Set(label.FindChild(6), t.GetRotation().Z());
    TDataStd_Real::Set(label.FindChild(7), t.GetRotation().W());
    TDataStd_Real::Set(label.FindChild(8), t.ScaleFactor());
 
}

void OcctDocument::SaveObjectMaterial(Handle(AIS_Shape) object
                                      , const Graphic3d_NameOfMaterial name_of_material) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label;
    Handle(TDataStd_Integer) aCurrentint;
    if(shapeTool->FindShape(object->Shape(), label)) {
        SaveObjectMaterial(label, name_of_material);
    }

}

void OcctDocument::SaveObjectColor(Handle(AIS_Shape) object
                                   , const Quantity_NameOfColor name_of_color) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label;
    Handle(TDataStd_Integer) aCurrentint;
    if(shapeTool->FindShape(object->Shape(), label)) {
        SaveObjectColor(label, name_of_color);
    }
}

void OcctDocument::SaveObjectMaterial(const TDF_Label& label, const Graphic3d_NameOfMaterial name_of_material) {
    TDataStd_Integer::Set(label.FindChild(11), name_of_material);
}

void OcctDocument::SaveObjectColor(const TDF_Label& label, const Quantity_NameOfColor name_of_color) {
    TDataStd_Integer::Set(label.FindChild(12), name_of_color);
}

Graphic3d_NameOfMaterial OcctDocument::MaterialNameForShape(Handle(AIS_Shape) object) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label;
    if(shapeTool->FindShape(object->Shape(), label)) {
        return MaterialNameForLabel(label);
    }
    return Graphic3d_NameOfMaterial_UserDefined;
}

Graphic3d_NameOfMaterial OcctDocument::MaterialNameForLabel(const TDF_Label& label) const {
    Handle(TDataStd_Integer) attribute;
    if (!label.IsNull()
        && label.FindChild(11).FindAttribute(TDataStd_Integer::GetID(), attribute)
        && !attribute.IsNull()) {
        return static_cast<Graphic3d_NameOfMaterial>(attribute->Get());
    }
    return Graphic3d_NameOfMaterial_ShinyPlastified;
}

Quantity_NameOfColor OcctDocument::ColorNameForLabel(const TDF_Label& label) const {
    Handle(TDataStd_Integer) attribute;
    if (!label.IsNull()
        && label.FindChild(12).FindAttribute(TDataStd_Integer::GetID(), attribute)
        && !attribute.IsNull()) {
        return static_cast<Quantity_NameOfColor>(attribute->Get());
    }
    return Quantity_NOC_GRAY80;
}

void OcctDocument::LoadObjectMeterial(const TDF_Label& label, const Handle(AIS_Shape) anAis) {
    Handle(TDataStd_Integer) aCurrentint;
    
    label.FindChild(11).FindAttribute(TDataStd_Integer::GetID(), aCurrentint);
    
    if(!aCurrentint.IsNull()) {
        const Graphic3d_NameOfMaterial name_of_material = (Graphic3d_NameOfMaterial)aCurrentint->Get();
        Graphic3d_MaterialAspect m = Graphic3d_MaterialAspect(name_of_material);
        anAis->SetMaterial(m);
    }

    label.FindChild(12).FindAttribute(TDataStd_Integer::GetID(), aCurrentint);
    
    if(!aCurrentint.IsNull()) {
        const Quantity_NameOfColor name_of_color = (Quantity_NameOfColor)aCurrentint->Get();
        anAis->SetColor(Quantity_Color(name_of_color));
    }

}


void OcctDocument::ApplyTransforms() {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_LabelSequence aLabels;
    shapeTool->GetFreeShapes (aLabels);
    for (Standard_Integer aLabIter = 1; aLabIter <= aLabels.Length(); ++aLabIter)
    {
        const TDF_Label& aLabel = aLabels.Value (aLabIter);
        const auto t = LabelTransform(aLabel);
        TNaming::Displace(aLabel, TopLoc_Location(t));
    }
}

const gp_Trsf OcctDocument::LabelTransform(const TDF_Label& aRefLabel) {
    Handle(TDataStd_Real) aCurrentReal;
    Standard_Real x = 0.;
    Standard_Real y = 0.;
    Standard_Real z = 0.;
    Standard_Real rx = 0.;
    Standard_Real ry = 0.;
    Standard_Real rz = 0.;
    Standard_Real rw = 1.;
    Standard_Real scale = 1.;
    
    aRefLabel.FindChild(1).FindAttribute(TDataStd_Real::GetID(), aCurrentReal);
    if(!aCurrentReal.IsNull())
        x = aCurrentReal->Get();
    
    aRefLabel.FindChild(2).FindAttribute(TDataStd_Real::GetID(), aCurrentReal);
    if(!aCurrentReal.IsNull())
        y = aCurrentReal->Get();
    
    aRefLabel.FindChild(3).FindAttribute(TDataStd_Real::GetID(), aCurrentReal);
    if(!aCurrentReal.IsNull())
        z = aCurrentReal->Get();
    
    aRefLabel.FindChild(4).FindAttribute(TDataStd_Real::GetID(), aCurrentReal);
    if(!aCurrentReal.IsNull())
        rx = aCurrentReal->Get();
    
    aRefLabel.FindChild(5).FindAttribute(TDataStd_Real::GetID(), aCurrentReal);
    if(!aCurrentReal.IsNull())
        ry = aCurrentReal->Get();
    
    aRefLabel.FindChild(6).FindAttribute(TDataStd_Real::GetID(), aCurrentReal);
    if(!aCurrentReal.IsNull())
        rz = aCurrentReal->Get();
    
    aRefLabel.FindChild(7).FindAttribute(TDataStd_Real::GetID(), aCurrentReal);
    if(!aCurrentReal.IsNull())
        rw = aCurrentReal->Get();
    
    aRefLabel.FindChild(8).FindAttribute(TDataStd_Real::GetID(), aCurrentReal);
    if(!aCurrentReal.IsNull())
        scale = aCurrentReal->Get();
    
    gp_Trsf t = gp_Trsf();
    t.SetTranslation({x, y, z});
    t.SetRotationPart({rx, ry, rz, rw});
    t.SetScaleFactor(scale);
    return t;
}

void OcctDocument::LoadObjectTransform(const TDF_Label& aRefLabel, const Handle(AIS_Shape) anAis) {
    anAis->SetLocalTransformation(LabelTransform(aRefLabel));
}

Standard_Boolean OcctDocument::undo() {
    if (!canUndo()) {
		return Standard_False;
    }
    try {
        if (myOcafDoc->Undo()) {
            NotifyChanges();
			return Standard_True;
        }
    } catch (const Standard_Failure& ex) {
        std::cout << ex.GetMessageString() << std::endl;
    }
	return Standard_False;
}
Standard_Boolean OcctDocument::redo() {
    if (!canRedo()) {
		return Standard_False;
    }
    try {
		if (myOcafDoc->Redo()) {
			NotifyChanges();
			return Standard_True;
		}
	} catch(const Standard_Failure& ex) {
        std::cout << ex.GetMessageString() << std::endl;
    }
	return Standard_False;
}

const bool OcctDocument::canUndo() const {
	return !myOcafDoc.IsNull() && !myOcafDoc->HasOpenCommand()
		&& myOcafDoc->GetAvailableUndos() > 0;
}

const bool OcctDocument::canRedo() const {
	return !myOcafDoc.IsNull() && !myOcafDoc->HasOpenCommand()
		&& myOcafDoc->GetAvailableRedos() > 0;
}

std::string OcctDocument::save(const std::string& path) {
    if (myOcafDoc.IsNull() || myOcafDoc->HasOpenCommand()) {
        return {};
    }

    auto app =  Handle(TDocStd_Application)::DownCast(myOcafDoc->Application());
    if (app.IsNull()) {
        return {};
    }

    try {
        PCDM_StoreStatus status = app->SaveAs(myOcafDoc, path.c_str()); // ".cbf"
        if (status != PCDM_SS_OK) {
            return {};
        }
        return path + ".cbf";
    } catch (const Standard_Failure& failure) {
        std::cout << "Save CBF failure: " << failure.GetMessageString() << std::endl;
        return {};
    }
}

void OcctDocument::NotifyChanges() {
    [[NSNotificationCenter defaultCenter]
     postNotificationName:@"OcctDocumentChanges"
     object:[NSValue valueWithPointer:this]];
}
