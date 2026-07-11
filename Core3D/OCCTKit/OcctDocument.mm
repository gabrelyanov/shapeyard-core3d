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
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <BinDrivers.hxx>

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDataStd_Real.hxx>
#include <gp_Trsf.hxx>
#include <GP_Quaternion.hxx>
#include <TNaming.hxx>
#include <Standard_GUID.hxx>
#include <TDF_LabelMap.hxx>
#include <XCAFPrs_DocumentExplorer.hxx>

#include <set>
#include <vector>

IMPLEMENT_STANDARD_RTTIEXT(OcctDocument, Standard_Transient)

namespace {

// These GUIDs are persistent schema identifiers. They identify the attribute
// role; the UUID string stored in each attribute identifies the document,
// occurrence, or shared shape definition itself.
const Standard_GUID& DocumentIdentifierAttributeID()
{
    static const Standard_GUID anId("74386E4E-F620-498F-8092-E6D883AF33A4");
    return anId;
}

const Standard_GUID& EntityIdentifierAttributeID()
{
    static const Standard_GUID anId("0074F7C2-9EAA-4F89-B2DE-8716E155FF62");
    return anId;
}

const Standard_GUID& DefinitionIdentifierAttributeID()
{
    static const Standard_GUID anId("3611F2B2-C694-4E12-AED8-A2A97A3D283B");
    return anId;
}

std::string ReadIdentifier(const TDF_Label& theLabel,
                           const Standard_GUID& theAttributeID)
{
    if (theLabel.IsNull()) {
        return {};
    }

    Handle(TDataStd_AsciiString) anIdentifier;
    if (!theLabel.FindAttribute(theAttributeID, anIdentifier)
        || anIdentifier.IsNull()) {
        return {};
    }

    const TCollection_AsciiString& aValue = anIdentifier->Get();
    if (aValue.IsEmpty() || !Standard_GUID::CheckGUIDFormat(aValue.ToCString())) {
        return {};
    }
    return aValue.ToCString();
}

std::string NewIdentifier()
{
    NSString* aValue = NSUUID.UUID.UUIDString;
    return aValue == nil ? std::string() : std::string(aValue.UTF8String);
}

Standard_Boolean AssignNewIdentifier(
    const TDF_Label& theLabel,
    const Standard_GUID& theAttributeID)
{
    if (theLabel.IsNull()) {
        return Standard_False;
    }

    const std::string anIdentifier = NewIdentifier();
    if (anIdentifier.empty()) {
        return Standard_False;
    }
    TDataStd_AsciiString::Set(
        theLabel,
        theAttributeID,
        TCollection_AsciiString(anIdentifier.c_str()));
    return Standard_True;
}

Standard_Boolean AssignIdentifierIfMissing(
    const TDF_Label& theLabel,
    const Standard_GUID& theAttributeID)
{
    return !ReadIdentifier(theLabel, theAttributeID).empty()
        || AssignNewIdentifier(theLabel, theAttributeID);
}

void AbortCommandNoThrow(const Handle(TDocStd_Document)& theDocument) noexcept
{
    if (theDocument.IsNull()) {
        return;
    }
    try {
        if (theDocument->HasOpenCommand()) {
            theDocument->AbortCommand();
        }
    } catch (...) {
    }
}

} // namespace

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

  // Install document infrastructure and identity before enabling normal undo
  // history. These are schema attributes, not user-authored edits.
  if (!myOcafDoc.IsNull())
  {
	// Create the persistent XCAF tools before the first undoable command. If the
	// first shape command creates these infrastructure attributes, Undo removes
	// them and a viewport redraw recreates them outside history; Redo then fails
	// because the same labels already carry those attributes.
	(void)XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
	(void)XCAFDoc_DocumentTool::ColorTool(myOcafDoc->Main());
	if (!AssignIdentifierIfMissing(
	        myOcafDoc->Main(), DocumentIdentifierAttributeID())) {
	  Message::SendFail("Unable to assign Core3D document identifier");
	}
	myOcafDoc->ClearUndos();
	myOcafDoc->SetUndoLimit(40);
  }
}

std::string OcctDocument::DocumentIdentifier() const
{
    return myOcafDoc.IsNull()
        ? std::string()
        : ReadIdentifier(myOcafDoc->Main(), DocumentIdentifierAttributeID());
}

std::string OcctDocument::EntityIdentifierForLabel(const TDF_Label& label) const
{
    return ReadIdentifier(label, EntityIdentifierAttributeID());
}

std::string OcctDocument::DefinitionIdentifierForLabel(const TDF_Label& label) const
{
    return ReadIdentifier(label, DefinitionIdentifierAttributeID());
}

Standard_Boolean OcctDocument::MigrateLegacyIdentifiers()
{
    return MigrateLegacyIdentifiers(myOcafDoc);
}

Standard_Boolean OcctDocument::MigrateLegacyIdentifiers(
    const Handle(TDocStd_Document)& document)
{
    if (document.IsNull() || document->HasOpenCommand()) {
        return Standard_False;
    }

    TDF_LabelMap entityLabels;
    TDF_LabelMap definitionLabels;
    try {
        OCC_CATCH_SIGNALS
        XCAFPrs_DocumentExplorer anExplorer(
            document,
            XCAFPrs_DocumentExplorerFlags_None);
        for (; anExplorer.More(); anExplorer.Next()) {
            const XCAFPrs_DocumentNode& aNode = anExplorer.Current();
            if (!aNode.Label.IsNull()) {
                entityLabels.Add(aNode.Label);
            }
            const TDF_Label& aDefinitionLabel = aNode.RefLabel.IsNull()
                ? aNode.Label
                : aNode.RefLabel;
            if (!aDefinitionLabel.IsNull()) {
                definitionLabels.Add(aDefinitionLabel);
            }
        }
    } catch (...) {
        return Standard_False;
    }

    const Standard_Boolean needsDocumentIdentifier =
        ReadIdentifier(document->Main(), DocumentIdentifierAttributeID()).empty();
    std::vector<TDF_Label> entityIdentifiersNeedingAssignment;
    std::vector<TDF_Label> definitionIdentifiersNeedingAssignment;
    std::set<std::string> entityIdentifiers;
    std::set<std::string> definitionIdentifiers;
    for (TDF_MapIteratorOfLabelMap anEntity(entityLabels);
         anEntity.More(); anEntity.Next()) {
        const std::string anIdentifier = ReadIdentifier(
            anEntity.Key(), EntityIdentifierAttributeID());
        if (anIdentifier.empty()
            || !entityIdentifiers.insert(anIdentifier).second) {
            entityIdentifiersNeedingAssignment.push_back(anEntity.Key());
        }
    }
    for (TDF_MapIteratorOfLabelMap aDefinition(definitionLabels);
         aDefinition.More(); aDefinition.Next()) {
        const std::string anIdentifier = ReadIdentifier(
            aDefinition.Key(), DefinitionIdentifierAttributeID());
        if (anIdentifier.empty()
            || !definitionIdentifiers.insert(anIdentifier).second) {
            definitionIdentifiersNeedingAssignment.push_back(aDefinition.Key());
        }
    }

    if (!needsDocumentIdentifier
        && entityIdentifiersNeedingAssignment.empty()
        && definitionIdentifiersNeedingAssignment.empty()) {
        return Standard_True;
    }

    // Identity migration is a schema operation and must never erase an active
    // user's history. Callers run it immediately after import/open, before the
    // document is published for editing.
    if (document->GetAvailableUndos() != 0
        || document->GetAvailableRedos() != 0) {
        return Standard_False;
    }

    const Standard_Integer aPreviousUndoLimit = document->GetUndoLimit();
    try {
        OCC_CATCH_SIGNALS
        document->SetUndoLimit(1);
        document->NewCommand();
        if (!document->HasOpenCommand()) {
            document->SetUndoLimit(aPreviousUndoLimit);
            return Standard_False;
        }

        if (needsDocumentIdentifier
            && !AssignNewIdentifier(
                document->Main(), DocumentIdentifierAttributeID())) {
            throw Standard_Failure("Unable to migrate document identifier");
        }
        for (const TDF_Label& aLabel : entityIdentifiersNeedingAssignment) {
            if (!AssignNewIdentifier(
                    aLabel, EntityIdentifierAttributeID())) {
                throw Standard_Failure("Unable to migrate entity identifier");
            }
        }
        for (const TDF_Label& aLabel : definitionIdentifiersNeedingAssignment) {
            if (!AssignNewIdentifier(
                    aLabel, DefinitionIdentifierAttributeID())) {
                throw Standard_Failure("Unable to migrate definition identifier");
            }
        }

        if (!document->CommitCommand()) {
            AbortCommandNoThrow(document);
            document->SetUndoLimit(aPreviousUndoLimit);
            return Standard_False;
        }
        document->ClearUndos();
        document->SetUndoLimit(aPreviousUndoLimit);
        return Standard_True;
    } catch (...) {
        AbortCommandNoThrow(document);
        document->ClearUndos();
        document->SetUndoLimit(aPreviousUndoLimit);
        return Standard_False;
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
	if (myOcafDoc.IsNull()
	    || !myOcafDoc->HasOpenCommand()
	    || aisShape.IsNull()
	    || aisShape->Shape().IsNull()) {
	    return TDF_Label();
	}
	Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
	if (shapeTool.IsNull()) {
	    return TDF_Label();
	}
	TDF_Label label = shapeTool->NewShape();
	shapeTool->SetShape(label, aisShape->Shape());
	if (!AssignNewIdentifier(label, EntityIdentifierAttributeID())
	    || !AssignNewIdentifier(label, DefinitionIdentifierAttributeID())) {
	    return TDF_Label();
	}
	SaveObjectTransform(label, aisShape);
	return label;
}

void OcctDocument::ReplaceShape(const TDF_Label& label, Handle(AIS_Shape) aisShape) {
	// Stable entity and definition identifiers belong to the label, so replacing
	// its geometry deliberately leaves both identity attributes untouched.
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
    Graphic3d_NameOfMaterial material;
    return TryMaterialNameForLabel(label, material)
        ? material
        : Graphic3d_NameOfMaterial_ShinyPlastified;
}

Standard_Boolean OcctDocument::TryMaterialNameForLabel(
    const TDF_Label& label,
    Graphic3d_NameOfMaterial& material) const {
    Handle(TDataStd_Integer) attribute;
    const TDF_Label materialLabel = label.IsNull()
        ? TDF_Label()
        : label.FindChild(11, Standard_False);
    if (!materialLabel.IsNull()
        && materialLabel.FindAttribute(TDataStd_Integer::GetID(), attribute)
        && !attribute.IsNull()) {
        material = static_cast<Graphic3d_NameOfMaterial>(attribute->Get());
        return Standard_True;
    }
    return Standard_False;
}

Quantity_NameOfColor OcctDocument::ColorNameForLabel(const TDF_Label& label) const {
    Quantity_NameOfColor color;
    return TryColorNameForLabel(label, color)
        ? color
        : Quantity_NOC_GRAY80;
}

Standard_Boolean OcctDocument::TryColorNameForLabel(
    const TDF_Label& label,
    Quantity_NameOfColor& color) const {
    Handle(TDataStd_Integer) attribute;
    const TDF_Label colorLabel = label.IsNull()
        ? TDF_Label()
        : label.FindChild(12, Standard_False);
    if (!colorLabel.IsNull()
        && colorLabel.FindAttribute(TDataStd_Integer::GetID(), attribute)
        && !attribute.IsNull()) {
        color = static_cast<Quantity_NameOfColor>(attribute->Get());
        return Standard_True;
    }
    return Standard_False;
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
        const auto t = ObjectTransformForLabel(aLabel);
        TNaming::Displace(aLabel, TopLoc_Location(t));
    }
}

gp_Trsf OcctDocument::ObjectTransformForLabel(const TDF_Label& aRefLabel) const {
    const auto readReal = [&aRefLabel](const Standard_Integer theTag,
                                      const Standard_Real theDefault) {
        if (aRefLabel.IsNull()) {
            return theDefault;
        }
        const TDF_Label aChild = aRefLabel.FindChild(theTag, Standard_False);
        if (aChild.IsNull()) {
            return theDefault;
        }
        Handle(TDataStd_Real) anAttribute;
        return aChild.FindAttribute(TDataStd_Real::GetID(), anAttribute)
            && !anAttribute.IsNull()
            ? anAttribute->Get()
            : theDefault;
    };

    const Standard_Real x = readReal(1, 0.0);
    const Standard_Real y = readReal(2, 0.0);
    const Standard_Real z = readReal(3, 0.0);
    const Standard_Real rx = readReal(4, 0.0);
    const Standard_Real ry = readReal(5, 0.0);
    const Standard_Real rz = readReal(6, 0.0);
    const Standard_Real rw = readReal(7, 1.0);
    const Standard_Real scale = readReal(8, 1.0);
    
    gp_Trsf t = gp_Trsf();
    t.SetTranslation({x, y, z});
    t.SetRotationPart({rx, ry, rz, rw});
    t.SetScaleFactor(scale);
    return t;
}

void OcctDocument::LoadObjectTransform(const TDF_Label& aRefLabel, const Handle(AIS_Shape) anAis) {
    anAis->SetLocalTransformation(ObjectTransformForLabel(aRefLabel));
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
