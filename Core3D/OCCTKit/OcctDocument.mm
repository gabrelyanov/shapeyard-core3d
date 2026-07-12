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
#include <Standard_Failure.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>

#include <TCollection_AsciiString.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <BinDrivers_DocumentStorageDriver.hxx>
#include <BinXCAFDrivers_DocumentStorageDriver.hxx>
#include <BinDrivers_DocumentRetrievalDriver.hxx>
#include <BinMDF_ADriverTable.hxx>
#include <BinMDF_TagSourceDriver.hxx>
#include <BinMDataStd_AsciiStringDriver.hxx>
#include <BinMDataStd_GenericEmptyDriver.hxx>
#include <BinMDataStd_GenericExtStringDriver.hxx>
#include <BinMDataStd_IntegerDriver.hxx>
#include <BinMDataStd_RealDriver.hxx>
#include <BinMDataStd_TreeNodeDriver.hxx>
#include <BinMDataStd_UAttributeDriver.hxx>
#include <BinMNaming_NamedShapeDriver.hxx>
#include <BinMXCAFDoc_ColorDriver.hxx>
#include <BinMXCAFDoc_LengthUnitDriver.hxx>
#include <BinMXCAFDoc_LocationDriver.hxx>
#include <BinMXCAFDoc_VisMaterialDriver.hxx>
#include <BinMXCAFDoc_VisMaterialToolDriver.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <Storage_TypeData.hxx>

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <TDataStd_Real.hxx>
#include <TDataStd_TreeNode.hxx>
#include <gp_Trsf.hxx>
#include <GP_Quaternion.hxx>
#include <TNaming.hxx>
#include <Standard_GUID.hxx>
#include <TDF_LabelMap.hxx>
#include <XCAFPrs_DocumentExplorer.hxx>
#include <Image_Texture.hxx>
#include <NCollection_Buffer.hxx>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

IMPLEMENT_STANDARD_RTTIEXT(OcctDocument, Standard_Transient)

namespace {

constexpr Standard_Integer kMaximumVisualMaterialDefinitions = 2048;
constexpr Standard_Real kDefaultMetersPerUnit = 0.001;
constexpr Standard_Real kMaximumEmissionFactor = 65504.0;
constexpr Standard_Size kMaximumEmbeddedTextureBytes =
    32ull * 1024ull * 1024ull;
constexpr Standard_Size kMaximumAggregateTextureBytes =
    128ull * 1024ull * 1024ull;
constexpr Standard_Integer kMaximumPersistentTextureIdentifierBytes = 256;
constexpr Standard_Integer kMaximumPersistentNameCharacters = 4096;
constexpr Standard_Integer kPersistentRecordHeaderBytes =
    3 * static_cast<Standard_Integer>(sizeof(Standard_Integer));
constexpr const char* kBufferTexturePrefix = "texturebuf://";
thread_local bool gSafeBinaryReadRejected = false;

void RejectSafeBinaryRead() noexcept
{
    gSafeBinaryReadRejected = true;
}

bool TryPersistentRecordEnd(
    const BinObjMgt_Persistent& theSource,
    Standard_Integer& theRecordEnd)
{
    const Standard_Integer aLength = theSource.Length();
    if (aLength < 0
        || aLength
            > std::numeric_limits<Standard_Integer>::max()
                - kPersistentRecordHeaderBytes) {
        return false;
    }
    theRecordEnd = kPersistentRecordHeaderBytes + aLength;
    return true;
}

bool ReadBoundedPersistentAsciiString(
    const BinObjMgt_Persistent& theSource,
    std::string& theValue)
{
    theValue.clear();
    const Standard_Integer aPosition = theSource.Position();
    Standard_Integer aRecordEnd = 0;
    if (!TryPersistentRecordEnd(theSource, aRecordEnd)
        || aPosition < kPersistentRecordHeaderBytes
        || aPosition > aRecordEnd
        || aPosition
            > std::numeric_limits<Standard_Integer>::max() - 3) {
        return false;
    }
    const Standard_Integer anAlignedPosition =
        (aPosition + 3) & ~Standard_Integer(3);
    if (anAlignedPosition > aRecordEnd
        || !theSource.SetPosition(anAlignedPosition)) {
        return false;
    }

    for (Standard_Integer anIndex = 0;
         anIndex <= kMaximumPersistentTextureIdentifierBytes;
         ++anIndex) {
        if (theSource.Position() >= aRecordEnd) {
            return false;
        }
        Standard_Character aCharacter = '\0';
        if (!theSource.GetCharacter(aCharacter).IsOK()) {
            return false;
        }
        if (aCharacter == '\0') {
            return true;
        }
        if (anIndex == kMaximumPersistentTextureIdentifierBytes) {
            return false;
        }
        theValue.push_back(aCharacter);
    }
    return false;
}

bool StripBufferTexturePrefixes(std::string& theIdentifier)
{
    const std::string aPrefix(kBufferTexturePrefix);
    while (theIdentifier.rfind(aPrefix, 0) == 0) {
        theIdentifier.erase(0, aPrefix.size());
    }
    return !theIdentifier.empty()
        && theIdentifier.size()
            <= static_cast<std::size_t>(
                kMaximumPersistentTextureIdentifierBytes
                - aPrefix.size());
}

bool PreflightBoundedPersistentExtendedString(
    const BinObjMgt_Persistent& theSource)
{
    const Standard_Integer aStart = theSource.Position();
    Standard_Integer aRecordEnd = 0;
    if (!TryPersistentRecordEnd(theSource, aRecordEnd)
        || aStart < kPersistentRecordHeaderBytes
        || aStart > aRecordEnd
        || aStart
            > std::numeric_limits<Standard_Integer>::max() - 3) {
        return false;
    }
    const Standard_Integer anAlignedPosition =
        (aStart + 3) & ~Standard_Integer(3);
    if (anAlignedPosition > aRecordEnd
        || !theSource.SetPosition(anAlignedPosition)) {
        return false;
    }
    for (Standard_Integer anIndex = 0;
         anIndex <= kMaximumPersistentNameCharacters;
         ++anIndex) {
        if (theSource.Position() > aRecordEnd
                - static_cast<Standard_Integer>(
                    sizeof(Standard_ExtCharacter))) {
            return false;
        }
        Standard_ExtCharacter aCharacter = 0;
        if (!theSource.GetExtCharacter(aCharacter).IsOK()) {
            return false;
        }
        if (aCharacter == 0) {
            return theSource.SetPosition(aStart);
        }
        if (anIndex == kMaximumPersistentNameCharacters) {
            return false;
        }
    }
    return false;
}

bool PreflightEmbeddedTexture(
    const BinObjMgt_Persistent& theSource,
    Standard_Size& theAggregateBytes)
{
    std::string anIdentifier;
    if (!ReadBoundedPersistentAsciiString(theSource, anIdentifier)) {
        return false;
    }
    if (anIdentifier.empty()) {
        return true;
    }

    Standard_Boolean usesBuffer = Standard_False;
    if (!theSource.GetBoolean(usesBuffer).IsOK() || !usesBuffer
        || !StripBufferTexturePrefixes(anIdentifier)) {
        // External paths and offsets are deliberately unsupported for
        // self-contained, sandbox-safe projects.
        return false;
    }

    Standard_Integer aLength = 0;
    if (!theSource.GetInteger(aLength).IsOK()
        || aLength <= 0
        || static_cast<Standard_Size>(aLength)
            > kMaximumEmbeddedTextureBytes) {
        return false;
    }
    const Standard_Integer aPosition = theSource.Position();
    Standard_Integer aRecordEnd = 0;
    if (!TryPersistentRecordEnd(theSource, aRecordEnd)
        || aPosition < kPersistentRecordHeaderBytes
        || aPosition > aRecordEnd
        || aLength > aRecordEnd - aPosition
        || theAggregateBytes > kMaximumAggregateTextureBytes
        || static_cast<Standard_Size>(aLength)
            > kMaximumAggregateTextureBytes - theAggregateBytes) {
        return false;
    }
    theAggregateBytes += static_cast<Standard_Size>(aLength);
    return theSource.SetPosition(aPosition + aLength);
}

bool SkipShortReals(
    const BinObjMgt_Persistent& theSource,
    const Standard_Integer theCount)
{
    for (Standard_Integer anIndex = 0; anIndex < theCount; ++anIndex) {
        Standard_ShortReal aValue = 0.0f;
        if (!theSource.GetShortReal(aValue).IsOK()) {
            return false;
        }
    }
    return true;
}

bool NormalizeEmbeddedTexture(Handle(Image_Texture)& theTexture)
{
    if (theTexture.IsNull()) {
        return true;
    }
    const Handle(NCollection_Buffer)& aBuffer = theTexture->DataBuffer();
    if (aBuffer.IsNull()) {
        return false;
    }
    std::string anIdentifier(theTexture->TextureId().ToCString());
    if (!StripBufferTexturePrefixes(anIdentifier)) {
        return false;
    }
    theTexture = new Image_Texture(
        aBuffer, TCollection_AsciiString(anIdentifier.c_str()));
    return true;
}

template <typename Driver>
class Core3DFailClosedDriver final : public Driver
{
public:
    explicit Core3DFailClosedDriver(
        const Handle(Message_Messenger)& theMessageDriver)
    : Driver(theMessageDriver)
    {
    }

    Standard_Boolean Paste(
        const BinObjMgt_Persistent& theSource,
        const Handle(TDF_Attribute)& theTarget,
        BinObjMgt_RRelocationTable& theRelocationTable) const override
    {
        const Standard_Boolean succeeded = Driver::Paste(
            theSource, theTarget, theRelocationTable);
        if (!succeeded) {
            RejectSafeBinaryRead();
        }
        return succeeded;
    }
};

class Core3DBoundedAsciiStringDriver final
    : public BinMDataStd_AsciiStringDriver
{
public:
    explicit Core3DBoundedAsciiStringDriver(
        const Handle(Message_Messenger)& theMessageDriver)
    : BinMDataStd_AsciiStringDriver(theMessageDriver)
    {
    }

    Standard_Boolean Paste(
        const BinObjMgt_Persistent& theSource,
        const Handle(TDF_Attribute)& theTarget,
        BinObjMgt_RRelocationTable& theRelocationTable) const override
    {
        const Standard_Integer aStart = theSource.Position();
        std::string aValue;
        if (!ReadBoundedPersistentAsciiString(theSource, aValue)
            || !theSource.SetPosition(aStart)) {
            RejectSafeBinaryRead();
            return Standard_False;
        }
        return BinMDataStd_AsciiStringDriver::Paste(
            theSource, theTarget, theRelocationTable);
    }
};

class Core3DBoundedExtendedStringDriver final
    : public BinMDataStd_GenericExtStringDriver
{
public:
    explicit Core3DBoundedExtendedStringDriver(
        const Handle(Message_Messenger)& theMessageDriver)
    : BinMDataStd_GenericExtStringDriver(theMessageDriver)
    {
    }

    Standard_Boolean Paste(
        const BinObjMgt_Persistent& theSource,
        const Handle(TDF_Attribute)& theTarget,
        BinObjMgt_RRelocationTable& theRelocationTable) const override
    {
        if (!PreflightBoundedPersistentExtendedString(theSource)) {
            RejectSafeBinaryRead();
            return Standard_False;
        }
        return BinMDataStd_GenericExtStringDriver::Paste(
            theSource, theTarget, theRelocationTable);
    }
};

class Core3DBoundedVisMaterialDriver final : public BinMDF_ADriver
{
public:
    Core3DBoundedVisMaterialDriver(
        const Handle(Message_Messenger)& theMessageDriver,
        const std::shared_ptr<Standard_Size>& theAggregateTextureBytes)
    : BinMDF_ADriver(
          theMessageDriver,
          STANDARD_TYPE(XCAFDoc_VisMaterial)->Name()),
      myDelegate(new BinMXCAFDoc_VisMaterialDriver(theMessageDriver)),
      myAggregateTextureBytes(theAggregateTextureBytes)
    {
    }

    Handle(TDF_Attribute) NewEmpty() const override
    {
        return new XCAFDoc_VisMaterial();
    }

    Standard_Boolean Paste(
        const BinObjMgt_Persistent& theSource,
        const Handle(TDF_Attribute)& theTarget,
        BinObjMgt_RRelocationTable& theRelocationTable) const override
    {
        const Standard_Integer aStart = theSource.Position();
        auto restoreStart = [&]() {
            return theSource.SetPosition(aStart);
        };
        Standard_Byte aMajor = 0;
        Standard_Byte aMinor = 0;
        Standard_Byte aFaceCulling = 0;
        Standard_Byte anAlphaMode = 0;
        Standard_Boolean hasPBR = Standard_False;
        Standard_Boolean hasCommon = Standard_False;
        Standard_Size anAggregate = myAggregateTextureBytes == nullptr
            ? 0
            : *myAggregateTextureBytes;

        bool isSafe = theSource.GetByte(aMajor).IsOK()
            && theSource.GetByte(aMinor).IsOK()
            && aMajor == 1 && aMinor <= 1
            && theSource.GetByte(aFaceCulling).IsOK()
            && theSource.GetByte(anAlphaMode).IsOK()
            && (aFaceCulling == static_cast<Standard_Byte>('0')
                || aFaceCulling == static_cast<Standard_Byte>('B')
                || aFaceCulling == static_cast<Standard_Byte>('F')
                || aFaceCulling == static_cast<Standard_Byte>('1'))
            && (anAlphaMode == static_cast<Standard_Byte>('O')
                || anAlphaMode == static_cast<Standard_Byte>('M')
                || anAlphaMode == static_cast<Standard_Byte>('B')
                || anAlphaMode == static_cast<Standard_Byte>('b')
                || anAlphaMode == static_cast<Standard_Byte>('A'))
            && SkipShortReals(theSource, 1)
            && theSource.GetBoolean(hasPBR).IsOK();
        if (isSafe && hasPBR) {
            isSafe = SkipShortReals(theSource, 4 + 3 + 2)
                && PreflightEmbeddedTexture(theSource, anAggregate)
                && PreflightEmbeddedTexture(theSource, anAggregate)
                && PreflightEmbeddedTexture(theSource, anAggregate)
                && PreflightEmbeddedTexture(theSource, anAggregate)
                && PreflightEmbeddedTexture(theSource, anAggregate);
        }
        if (isSafe) {
            isSafe = theSource.GetBoolean(hasCommon).IsOK();
        }
        if (isSafe && hasCommon) {
            isSafe = SkipShortReals(theSource, 3 * 4 + 2)
                && PreflightEmbeddedTexture(theSource, anAggregate);
        }
        if (isSafe && hasPBR && aMinor >= 1) {
            isSafe = SkipShortReals(theSource, 1);
        }
        if (!isSafe || !theSource.IsOK() || !restoreStart()) {
            restoreStart();
            RejectSafeBinaryRead();
            return Standard_False;
        }

        if (!myDelegate->Paste(
                theSource, theTarget, theRelocationTable)) {
            RejectSafeBinaryRead();
            return Standard_False;
        }
        const Handle(XCAFDoc_VisMaterial) aMaterial =
            Handle(XCAFDoc_VisMaterial)::DownCast(theTarget);
        if (aMaterial.IsNull()) {
            RejectSafeBinaryRead();
            return Standard_False;
        }
        if (aMaterial->HasPbrMaterial()) {
            XCAFDoc_VisMaterialPBR aPBR = aMaterial->PbrMaterial();
            if (!NormalizeEmbeddedTexture(aPBR.BaseColorTexture)
                || !NormalizeEmbeddedTexture(
                    aPBR.MetallicRoughnessTexture)
                || !NormalizeEmbeddedTexture(aPBR.EmissiveTexture)
                || !NormalizeEmbeddedTexture(aPBR.OcclusionTexture)
                || !NormalizeEmbeddedTexture(aPBR.NormalTexture)) {
                RejectSafeBinaryRead();
                return Standard_False;
            }
            aMaterial->SetPbrMaterial(aPBR);
        }
        if (aMaterial->HasCommonMaterial()) {
            XCAFDoc_VisMaterialCommon aCommon =
                aMaterial->CommonMaterial();
            if (!NormalizeEmbeddedTexture(aCommon.DiffuseTexture)) {
                RejectSafeBinaryRead();
                return Standard_False;
            }
            aMaterial->SetCommonMaterial(aCommon);
        }
        if (myAggregateTextureBytes != nullptr) {
            *myAggregateTextureBytes = anAggregate;
        }
        return Standard_True;
    }

    void Paste(
        const Handle(TDF_Attribute)& theSource,
        BinObjMgt_Persistent& theTarget,
        BinObjMgt_SRelocationTable& theRelocationTable) const override
    {
        myDelegate->Paste(theSource, theTarget, theRelocationTable);
    }

private:
    Handle(BinMXCAFDoc_VisMaterialDriver) myDelegate;
    std::shared_ptr<Standard_Size> myAggregateTextureBytes;
};

class Core3DBoundedBinXCAFRetrievalDriver final
    : public BinDrivers_DocumentRetrievalDriver
{
public:
    Core3DBoundedBinXCAFRetrievalDriver()
    : myAggregateTextureBytes(std::make_shared<Standard_Size>(0))
    {
    }

    void Read(
        Standard_IStream& theStream,
        const Handle(Storage_Data)& theStorageData,
        const Handle(CDM_Document)& theDocument,
        const Handle(CDM_Application)& theApplication,
        const Handle(PCDM_ReaderFilter)& theFilter =
            Handle(PCDM_ReaderFilter)(),
        const Message_ProgressRange& theProgress =
            Message_ProgressRange()) override
    {
        ResetAggregateTextureBytes();
        if (theStorageData.IsNull() || theStorageData->TypeData().IsNull()) {
            RejectSafeBinaryRead();
            return;
        }
        const Handle(TColStd_HSequenceOfAsciiString) aPersistentTypes =
            theStorageData->TypeData()->Types();
        if (aPersistentTypes.IsNull()
            || aPersistentTypes->Length() < 0
            || aPersistentTypes->Length() > 128) {
            RejectSafeBinaryRead();
            return;
        }
        TColStd_SequenceOfAsciiString aTypeNames;
        for (Standard_Integer anIndex = 1;
             anIndex <= aPersistentTypes->Length(); ++anIndex) {
            const TCollection_AsciiString& aName =
                aPersistentTypes->Value(anIndex);
            if (aName.IsEmpty() || aName.Length() > 128) {
                RejectSafeBinaryRead();
                return;
            }
            aTypeNames.Append(aName);
        }
        Handle(BinMDF_ADriverTable) aSupportedDrivers =
            AttributeDrivers(Message::DefaultMessenger());
        aSupportedDrivers->AssignIds(aTypeNames);
        for (Standard_Integer anIndex = 1;
             anIndex <= aTypeNames.Length(); ++anIndex) {
            if (aSupportedDrivers->GetDriver(anIndex).IsNull()) {
                RejectSafeBinaryRead();
                return;
            }
        }
        try {
            BinDrivers_DocumentRetrievalDriver::Read(
                theStream,
                theStorageData,
                theDocument,
                theApplication,
                theFilter,
                theProgress);
        } catch (...) {
            ResetAggregateTextureBytes();
            throw;
        }
        ResetAggregateTextureBytes();
    }

    Handle(BinMDF_ADriverTable) AttributeDrivers(
        const Handle(Message_Messenger)& theMessageDriver) override
    {
        // Project files intentionally support a narrow OCAF schema. Starting
        // with BinDrivers::AttributeDrivers() would expose unrelated array,
        // list, named-data, function, note, and geometric-constraint readers
        // that trust attacker-controlled element counts before app validation.
        Handle(BinMDF_ADriverTable) aTable = new BinMDF_ADriverTable();
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDF_TagSourceDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_GenericEmptyDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DBoundedExtendedStringDriver(theMessageDriver));
        aTable->AddDriver(
            new Core3DBoundedAsciiStringDriver(theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_IntegerDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_RealDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_TreeNodeDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_UAttributeDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMXCAFDoc_ColorDriver>(
                theMessageDriver));

        const Handle(BinMNaming_NamedShapeDriver) aNamedShapeDriver =
            new Core3DFailClosedDriver<BinMNaming_NamedShapeDriver>(
                theMessageDriver);
        aTable->AddDriver(aNamedShapeDriver);
        Handle(BinMXCAFDoc_LocationDriver) aLocationDriver =
            new Core3DFailClosedDriver<BinMXCAFDoc_LocationDriver>(
                theMessageDriver);
        aLocationDriver->SetNSDriver(aNamedShapeDriver);
        aTable->AddDriver(aLocationDriver);
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMXCAFDoc_LengthUnitDriver>(
                theMessageDriver));
        aTable->AddDriver(new Core3DBoundedVisMaterialDriver(
            theMessageDriver, myAggregateTextureBytes));
        aTable->AddDriver(
            new Core3DFailClosedDriver<
                BinMXCAFDoc_VisMaterialToolDriver>(theMessageDriver));
        return aTable;
    }

    void Clear() override
    {
        BinDrivers_DocumentRetrievalDriver::Clear();
        ResetAggregateTextureBytes();
    }

private:
    void ResetAggregateTextureBytes() noexcept
    {
        if (myAggregateTextureBytes != nullptr) {
            *myAggregateTextureBytes = 0;
        }
    }

    std::shared_ptr<Standard_Size> myAggregateTextureBytes;
};

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

//! Marks XCAF visualization material assignments authored by Shapeyard's PBR
//! editor. Imported XCAF styles remain untouched and legacy child-11/12 style
//! overrides retain their historical precedence until the user authors PBR.
const Standard_GUID& LocalPBRMaterialAttributeID()
{
    static const Standard_GUID anId("248A5203-4A22-4F2B-85C4-BE0BA89A5E4D");
    return anId;
}

//! Marks immutable table entries created by Shapeyard. Imported material
//! libraries must never be garbage-collected by local authoring operations.
const Standard_GUID& OwnedPBRMaterialDefinitionAttributeID()
{
    static const Standard_GUID anId("750690D2-357B-4101-B0EC-70DF20A2EC97");
    return anId;
}

void RemoveUnreferencedOwnedMaterial(
    const Handle(XCAFDoc_VisMaterialTool)& theTool,
    const TDF_Label& theMaterialLabel)
{
    if (theTool.IsNull() || theMaterialLabel.IsNull()) {
        return;
    }
    Handle(TDataStd_Integer) anOwnedMarker;
    if (!theMaterialLabel.FindAttribute(
            OwnedPBRMaterialDefinitionAttributeID(), anOwnedMarker)
        || anOwnedMarker.IsNull() || anOwnedMarker->Get() != 1) {
        return;
    }
    Handle(TDataStd_TreeNode) aReferenceRoot;
    if (theMaterialLabel.FindAttribute(
            XCAFDoc::VisMaterialRefGUID(), aReferenceRoot)
        && !aReferenceRoot.IsNull() && aReferenceRoot->HasFirst()) {
        return;
    }
    theTool->RemoveMaterial(theMaterialLabel);
}

Standard_Boolean IsExclusivelyReferencedOwnedMaterial(
    const TDF_Label& theMaterialLabel,
    const TDF_Label& theShapeLabel)
{
    if (theMaterialLabel.IsNull() || theShapeLabel.IsNull()) {
        return Standard_False;
    }
    Handle(TDataStd_Integer) anOwnedMarker;
    if (!theMaterialLabel.FindAttribute(
            OwnedPBRMaterialDefinitionAttributeID(), anOwnedMarker)
        || anOwnedMarker.IsNull() || anOwnedMarker->Get() != 1) {
        return Standard_False;
    }
    Handle(TDataStd_TreeNode) aReferenceRoot;
    if (!theMaterialLabel.FindAttribute(
            XCAFDoc::VisMaterialRefGUID(), aReferenceRoot)
        || aReferenceRoot.IsNull() || !aReferenceRoot->HasFirst()) {
        return Standard_False;
    }
    const Handle(TDataStd_TreeNode) aFirst = aReferenceRoot->First();
    return !aFirst.IsNull()
        && !aFirst->HasNext()
        && aFirst->Label().IsEqual(theShapeLabel);
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

void Core3DBeginSafeBinaryRead()
{
    gSafeBinaryReadRejected = false;
}

Standard_Boolean Core3DSafeBinaryReadWasRejected()
{
    return gSafeBinaryReadRejected ? Standard_True : Standard_False;
}

void Core3DDefineSafeBinXCAFFormat(
    const Handle(TDocStd_Application)& application)
{
    if (application.IsNull()) {
        return;
    }
    application->DefineFormat(
        TCollection_AsciiString("BinOcaf"),
        TCollection_AsciiString("Binary OCAF Document"),
        TCollection_AsciiString("cbf"),
        new Core3DBoundedBinXCAFRetrievalDriver(),
        new BinDrivers_DocumentStorageDriver());
    application->DefineFormat(
        TCollection_AsciiString("BinXCAF"),
        TCollection_AsciiString("Binary XCAF Document"),
        TCollection_AsciiString("xbf"),
        new Core3DBoundedBinXCAFRetrievalDriver(),
        new BinXCAFDrivers_DocumentStorageDriver());
}

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


  // Register both readers before creating the document: old projects remain
  // readable while new saves use the XCAF driver required by visual materials.
  Core3DDefineSafeBinXCAFFormat(myApp);
  myApp->NewDocument(TCollection_ExtendedString("BinXCAF"), myOcafDoc);

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
	(void)XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
	XCAFDoc_DocumentTool::SetLengthUnit(
	    myOcafDoc, kDefaultMetersPerUnit);
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
  try {
    OCC_CATCH_SIGNALS
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

    const TCollection_ExtendedString aBinXCAFFormat("BinXCAF");
    const Standard_Boolean needsStorageFormat =
        !document->StorageFormat().IsEqual(aBinXCAFFormat);
    const Standard_Boolean needsDocumentIdentifier =
        ReadIdentifier(document->Main(), DocumentIdentifierAttributeID()).empty();
    const Standard_Boolean needsVisMaterialTool =
        !XCAFDoc_DocumentTool::CheckVisMaterialTool(document->Main());
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

    const Standard_Boolean needsSchemaMigration =
        needsDocumentIdentifier
        || needsVisMaterialTool
        || !entityIdentifiersNeedingAssignment.empty()
        || !definitionIdentifiersNeedingAssignment.empty();
    if (!needsStorageFormat
        && !needsSchemaMigration) {
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
    const TCollection_ExtendedString aPreviousStorageFormat =
        document->StorageFormat();
    try {
        OCC_CATCH_SIGNALS

        if (needsSchemaMigration) {
            document->SetUndoLimit(1);
            document->NewCommand();
            if (!document->HasOpenCommand()) {
                throw Standard_Failure("Unable to start document migration");
            }

            if (needsDocumentIdentifier
                && !AssignNewIdentifier(
                    document->Main(), DocumentIdentifierAttributeID())) {
                throw Standard_Failure("Unable to migrate document identifier");
            }
            if (needsVisMaterialTool
                && XCAFDoc_DocumentTool::VisMaterialTool(
                       document->Main()).IsNull()) {
                throw Standard_Failure("Unable to migrate XCAF material tool");
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
                throw Standard_Failure("Unable to commit document migration");
            }
            document->ClearUndos();
        }

        // Storage format is document metadata, not an OCAF attribute. Promote
        // it only after schema changes are safely committed, and never wrap a
        // format-only upgrade in an empty undo command.
        if (needsStorageFormat) {
            document->ChangeStorageFormat(aBinXCAFFormat);
        }
        document->SetUndoLimit(aPreviousUndoLimit);
        return Standard_True;
    } catch (...) {
        AbortCommandNoThrow(document);
        try {
            document->ClearUndos();
        } catch (...) {
        }
        try {
            document->SetUndoLimit(aPreviousUndoLimit);
        } catch (...) {
        }
        if (needsStorageFormat
            && !document->StorageFormat().IsEqual(
                aPreviousStorageFormat)) {
            try {
                document->ChangeStorageFormat(aPreviousStorageFormat);
            } catch (...) {
                // The candidate document will be rejected by the caller. Do
                // not let a secondary format-restore failure escape C++.
            }
        }
        return Standard_False;
    }
  } catch (...) {
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
    TDF_Label aPreviousMaterialLabel;
    XCAFDoc_VisMaterialTool::GetShapeMaterial(
        label, aPreviousMaterialLabel);
    const Standard_Boolean didRemove =
        !shapeTool.IsNull()
        && shapeTool->RemoveShape(label, Standard_True);
    if (didRemove && myOcafDoc->HasOpenCommand()
        && !aPreviousMaterialLabel.IsNull()
        && XCAFDoc_DocumentTool::CheckVisMaterialTool(
            myOcafDoc->Main())) {
        RemoveUnreferencedOwnedMaterial(
            XCAFDoc_DocumentTool::VisMaterialTool(
                myOcafDoc->Main()),
            aPreviousMaterialLabel);
    }
    return didRemove;
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

Standard_Boolean OcctDocument::SaveObjectPBRMaterial(
    const TDF_Label& label,
    const XCAFDoc_VisMaterialPBR& material) {
    const Quantity_Color& aBaseColor = material.BaseColor.GetRGB();
    const auto isFiniteUnit = [](const Standard_Real theValue) {
        return std::isfinite(theValue)
            && theValue >= 0.0 && theValue <= 1.0;
    };
    const auto isFiniteNonNegative = [](const Standard_Real theValue) {
        return std::isfinite(theValue) && theValue >= 0.0;
    };
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull() || !material.IsDefined
        || !isFiniteUnit(aBaseColor.Red())
        || !isFiniteUnit(aBaseColor.Green())
        || !isFiniteUnit(aBaseColor.Blue())
        || !isFiniteUnit(material.BaseColor.Alpha())
        || !isFiniteUnit(material.Metallic)
        || !isFiniteUnit(material.Roughness)
        || !isFiniteNonNegative(material.EmissiveFactor.x())
        || !isFiniteNonNegative(material.EmissiveFactor.y())
        || !isFiniteNonNegative(material.EmissiveFactor.z())
        || material.EmissiveFactor.x() > kMaximumEmissionFactor
        || material.EmissiveFactor.y() > kMaximumEmissionFactor
        || material.EmissiveFactor.z() > kMaximumEmissionFactor
        || !material.BaseColorTexture.IsNull()
        || !material.MetallicRoughnessTexture.IsNull()
        || !material.EmissiveTexture.IsNull()
        || !material.OcclusionTexture.IsNull()
        || !material.NormalTexture.IsNull()
        || !std::isfinite(material.RefractionIndex)
        || material.RefractionIndex < 1.0f
        || material.RefractionIndex > 3.0f) {
        return Standard_False;
    }

    Handle(XCAFDoc_VisMaterialTool) aTool =
        XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
    if (aTool.IsNull()) {
        return Standard_False;
    }

    Graphic3d_AlphaMode anAlphaMode =
        material.BaseColor.Alpha() < 0.999f
            ? Graphic3d_AlphaMode_Blend
            : Graphic3d_AlphaMode_Opaque;
    Standard_ShortReal anAlphaCutoff = 0.5f;
    Graphic3d_TypeOfBackfacingModel aFaceCulling =
        Graphic3d_TypeOfBackfacingModel_Auto;
    Handle(TDataStd_Integer) aLocalMarker;
    const Standard_Boolean hasLocalPBR =
        label.FindAttribute(LocalPBRMaterialAttributeID(), aLocalMarker)
        && !aLocalMarker.IsNull() && aLocalMarker->Get() == 1;
    Graphic3d_NameOfMaterial aLegacyMaterial;
    const Standard_Boolean hasLegacyMaterial =
        TryMaterialNameForLabel(label, aLegacyMaterial);
    const Handle(XCAFDoc_VisMaterial) aPreviousMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    TDF_Label aPreviousMaterialLabel;
    XCAFDoc_VisMaterialTool::GetShapeMaterial(
        label, aPreviousMaterialLabel);
    if (!aPreviousMaterial.IsNull()
        && (hasLocalPBR || !hasLegacyMaterial)) {
        anAlphaMode = aPreviousMaterial->AlphaMode();
        anAlphaCutoff = aPreviousMaterial->AlphaCutOff();
        aFaceCulling = aPreviousMaterial->FaceCulling();
    }

    Handle(XCAFDoc_VisMaterial) aMaterial = new XCAFDoc_VisMaterial();
    aMaterial->SetPbrMaterial(material);
    // ES2/OpenGL compatibility consumes the common approximation while Metal
    // and glTF consume the authoritative PBR definition.
    aMaterial->SetCommonMaterial(aMaterial->ConvertToCommonMaterial());
    aMaterial->SetAlphaMode(anAlphaMode, anAlphaCutoff);
    aMaterial->SetFaceCulling(aFaceCulling);

    TDF_Label aMaterialLabel;
    TDF_LabelSequence existingLabels;
    aTool->GetMaterials(existingLabels);
    for (Standard_Integer index = 1;
         index <= existingLabels.Length(); ++index) {
        const Handle(XCAFDoc_VisMaterial) existing =
            XCAFDoc_VisMaterialTool::GetMaterial(existingLabels.Value(index));
        if (!existing.IsNull() && existing->IsEqual(aMaterial)) {
            aMaterialLabel = existingLabels.Value(index);
            break;
        }
    }
    if (aMaterialLabel.IsNull()) {
        if (existingLabels.Length()
            >= kMaximumVisualMaterialDefinitions) {
            // A net-zero immutable replacement must remain possible at the
            // table limit. Unlink and reclaim only an exclusively referenced
            // Shapeyard-owned definition; aborting the surrounding command
            // restores both links if allocation below fails.
            if (!IsExclusivelyReferencedOwnedMaterial(
                    aPreviousMaterialLabel, label)) {
                return Standard_False;
            }
            aTool->UnSetShapeMaterial(label);
            RemoveUnreferencedOwnedMaterial(
                aTool, aPreviousMaterialLabel);
            TDF_LabelSequence remainingLabels;
            aTool->GetMaterials(remainingLabels);
            if (remainingLabels.Length()
                >= kMaximumVisualMaterialDefinitions) {
                return Standard_False;
            }
        }
        aMaterialLabel = aTool->AddMaterial(
            aMaterial,
            TCollection_AsciiString("Shapeyard PBR"));
        if (!aMaterialLabel.IsNull()) {
            TDataStd_Integer::Set(
                aMaterialLabel,
                OwnedPBRMaterialDefinitionAttributeID(),
                1);
        }
    }
    if (aMaterialLabel.IsNull()) {
        return Standard_False;
    }
    aTool->SetShapeMaterial(label, aMaterialLabel);
    TDataStd_Integer::Set(label, LocalPBRMaterialAttributeID(), 1);

    // Canonical PBR and legacy preset tags must never compete for precedence.
    for (const Standard_Integer aTag : {11, 12}) {
        const TDF_Label aLegacyLabel = label.FindChild(aTag, Standard_False);
        if (!aLegacyLabel.IsNull()) {
            aLegacyLabel.ForgetAttribute(TDataStd_Integer::GetID());
        }
    }
    if (!aPreviousMaterialLabel.IsNull()
        && !aPreviousMaterialLabel.IsEqual(aMaterialLabel)) {
        RemoveUnreferencedOwnedMaterial(
            aTool, aPreviousMaterialLabel);
    }
    return Standard_True;
}

Standard_Boolean OcctDocument::ClearObjectVisualMaterial(
    const TDF_Label& label) {
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull()) {
        return Standard_False;
    }
    if (XCAFDoc_DocumentTool::CheckVisMaterialTool(myOcafDoc->Main())) {
        Handle(XCAFDoc_VisMaterialTool) aTool =
            XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
        if (!aTool.IsNull()) {
            TDF_Label aPreviousMaterialLabel;
            XCAFDoc_VisMaterialTool::GetShapeMaterial(
                label, aPreviousMaterialLabel);
            aTool->UnSetShapeMaterial(label);
            RemoveUnreferencedOwnedMaterial(
                aTool, aPreviousMaterialLabel);
        }
    }
    label.ForgetAttribute(LocalPBRMaterialAttributeID());
    return Standard_True;
}

Standard_Boolean OcctDocument::CopyObjectAppearance(
    const TDF_Label& source,
    const TDF_Label& destination) {
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || source.IsNull() || destination.IsNull()) {
        return Standard_False;
    }

    Handle(TDataStd_Integer) aLocalPBRMarker;
    const Standard_Boolean hasLocalPBR =
        source.FindAttribute(
            LocalPBRMaterialAttributeID(), aLocalPBRMarker)
        && !aLocalPBRMarker.IsNull()
        && aLocalPBRMarker->Get() == 1;
    Graphic3d_NameOfMaterial aLegacyMaterial;
    Quantity_NameOfColor aLegacyColor;
    const Standard_Boolean hasLegacyMaterial =
        TryMaterialNameForLabel(source, aLegacyMaterial);
    const Standard_Boolean hasLegacyColor =
        TryColorNameForLabel(source, aLegacyColor);
    const auto clearDestinationLegacyAppearance = [&destination]() {
        for (const Standard_Integer aTag : {11, 12}) {
            const TDF_Label aLegacyLabel =
                destination.FindChild(aTag, Standard_False);
            if (!aLegacyLabel.IsNull()) {
                aLegacyLabel.ForgetAttribute(TDataStd_Integer::GetID());
            }
        }
    };
    clearDestinationLegacyAppearance();

    TDF_Label aVisualMaterialLabel;
    const Standard_Boolean hasVisualMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(
            source, aVisualMaterialLabel)
        && !aVisualMaterialLabel.IsNull();
    if (hasVisualMaterial) {
        Handle(XCAFDoc_VisMaterialTool) aTool =
            XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
        if (aTool.IsNull()) {
            return Standard_False;
        }
        TDF_Label aPreviousDestinationMaterialLabel;
        XCAFDoc_VisMaterialTool::GetShapeMaterial(
            destination, aPreviousDestinationMaterialLabel);
        aTool->SetShapeMaterial(destination, aVisualMaterialLabel);
        if (!aPreviousDestinationMaterialLabel.IsNull()
            && !aPreviousDestinationMaterialLabel.IsEqual(
                aVisualMaterialLabel)) {
            RemoveUnreferencedOwnedMaterial(
                aTool, aPreviousDestinationMaterialLabel);
        }
        if (hasLocalPBR) {
            TDataStd_Integer::Set(
                destination, LocalPBRMaterialAttributeID(), 1);
        } else {
            destination.ForgetAttribute(LocalPBRMaterialAttributeID());
        }
        if (hasLocalPBR) {
            return Standard_True;
        }
        if (hasLegacyMaterial) {
            SaveObjectMaterial(destination, aLegacyMaterial);
        }
        if (hasLegacyColor) {
            SaveObjectColor(destination, aLegacyColor);
        }
        return Standard_True;
    }

    if (!ClearObjectVisualMaterial(destination)) {
        return Standard_False;
    }
    // Preserve attribute absence as well as attribute values. Filling missing
    // children with defaults can change a preset-only source's effective base
    // color or make future precedence checks treat it as explicitly styled.
    if (hasLegacyMaterial) {
        SaveObjectMaterial(destination, aLegacyMaterial);
    }
    if (hasLegacyColor) {
        SaveObjectColor(destination, aLegacyColor);
    }
    return Standard_True;
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

Standard_Boolean OcctDocument::TryPBRMaterialForLabel(
    const TDF_Label& label,
    XCAFDoc_VisMaterialPBR& material) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    Handle(TDataStd_Integer) aMarker;
    if (!label.FindAttribute(LocalPBRMaterialAttributeID(), aMarker)
        || aMarker.IsNull() || aMarker->Get() != 1) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterial) aMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (aMaterial.IsNull() || !aMaterial->HasPbrMaterial()) {
        return Standard_False;
    }
    material = aMaterial->PbrMaterial();
    return material.IsDefined;
}

Standard_Boolean OcctDocument::TryEffectivePBRMaterialForLabel(
    const TDF_Label& label,
    XCAFDoc_VisMaterialPBR& material) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    Handle(TDataStd_Integer) aMarker;
    const Standard_Boolean hasLocalPBR =
        label.FindAttribute(LocalPBRMaterialAttributeID(), aMarker)
        && !aMarker.IsNull() && aMarker->Get() == 1;
    Graphic3d_NameOfMaterial aLegacyMaterial;
    Quantity_NameOfColor aLegacyColor;
    const Standard_Boolean hasLegacyMaterial =
        TryMaterialNameForLabel(label, aLegacyMaterial);
    const Standard_Boolean hasLegacyColor =
        TryColorNameForLabel(label, aLegacyColor);
    if (!hasLocalPBR && hasLegacyMaterial) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterial) aVisualMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (aVisualMaterial.IsNull()) {
        return Standard_False;
    }
    if (aVisualMaterial->HasPbrMaterial()) {
        material = aVisualMaterial->PbrMaterial();
    } else if (aVisualMaterial->HasCommonMaterial()) {
        material = aVisualMaterial->ConvertToPbrMaterial();
    } else {
        return Standard_False;
    }
    if (!hasLocalPBR && hasLegacyColor) {
        material.BaseColor = Quantity_ColorRGBA(
            Quantity_Color(aLegacyColor), material.BaseColor.Alpha());
    }
    return material.IsDefined;
}

Standard_Boolean OcctDocument::SupportsScalarPBRMaterialEditingForLabel(
    const TDF_Label& label) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterial) material =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (material.IsNull()) {
        return Standard_True;
    }
    if (material->HasPbrMaterial()) {
        const XCAFDoc_VisMaterialPBR& pbr = material->PbrMaterial();
        if (!pbr.BaseColorTexture.IsNull()
            || !pbr.MetallicRoughnessTexture.IsNull()
            || !pbr.EmissiveTexture.IsNull()
            || !pbr.OcclusionTexture.IsNull()
            || !pbr.NormalTexture.IsNull()) {
            return Standard_False;
        }
    }
    return !material->HasCommonMaterial()
        || material->CommonMaterial().DiffuseTexture.IsNull();
}

void OcctDocument::LoadObjectMeterial(const TDF_Label& label, const Handle(AIS_Shape) anAis) {
    const Handle(XCAFDoc_VisMaterial) aVisualMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    Handle(TDataStd_Integer) aLocalPBRMarker;
    const Standard_Boolean hasLocalPBR =
        label.FindAttribute(
            LocalPBRMaterialAttributeID(), aLocalPBRMarker)
        && !aLocalPBRMarker.IsNull()
        && aLocalPBRMarker->Get() == 1;
    Graphic3d_NameOfMaterial aLegacyMaterial;
    Quantity_NameOfColor aLegacyColor;
    const Standard_Boolean hasLegacyMaterial =
        TryMaterialNameForLabel(label, aLegacyMaterial);
    const Standard_Boolean hasLegacyColor =
        TryColorNameForLabel(label, aLegacyColor);
    if (!aVisualMaterial.IsNull()) {
        Graphic3d_MaterialAspect anAspect;
        aVisualMaterial->FillMaterialAspect(anAspect);
        anAis->SetMaterial(anAspect);
        anAis->SetColor(aVisualMaterial->BaseColor().GetRGB());
        if (hasLocalPBR) {
            return;
        }
    }
    if (hasLegacyMaterial) {
        Graphic3d_MaterialAspect m =
            Graphic3d_MaterialAspect(aLegacyMaterial);
        anAis->SetMaterial(m);
    }
    if (hasLegacyColor) {
        anAis->SetColor(Quantity_Color(aLegacyColor));
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
            std::cout << "Save CBF failed with status " << status << std::endl;
            return {};
        }
        const TCollection_ExtendedString aBinXCAFFormat("BinXCAF");
        return path + (myOcafDoc->StorageFormat().IsEqual(aBinXCAFFormat)
            ? ".xbf"
            : ".cbf");
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
