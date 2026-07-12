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
#include <XCAFDoc_VisMaterialPBR.hxx>
#include <Image_Texture.hxx>
#include <NCollection_Buffer.hxx>

#include <string>
#include <unordered_map>
#include <vector>

class Message_ProgressRange;

//! Persistent geometry representation owned by each XCAF definition label.
//! The non-negative values are serialized schema values: never renumber or
//! reuse them. Invalid is a read-only fail-closed sentinel and must never be
//! written to a document.
enum class OcctGeometryRepresentation : Standard_Integer
{
    Invalid = -1,
    LegacyUnknown = 0,
    BRep = 1,
    TriangleMesh = 2,
};

//! Export formats supported directly by the persisted geometry contract.
//! Values are bit flags returned by SupportedGeometryExportFormats().
enum class OcctGeometryExportFormat : Standard_Integer
{
    Obj = 1 << 0,
    Stl = 1 << 1,
    Gltf = 1 << 2,
    Step = 1 << 3,
};

//! Validate/canonicalize the narrow texture representation produced by the
//! mobile material editor. Only complete, single-frame PNG/JPEG images within
//! the shared project/snapshot safety budgets are accepted. The returned
//! texture owns an exact byte copy and uses a content-addressed SHA-256 ID.
//! Texture semantics (base color, emissive, and future supported slots) live
//! on the material binding rather than in this exact-byte resource.
Standard_EXPORT Standard_Boolean Core3DCreateAuthoredTexture(
    const Standard_Byte* bytes,
    Standard_Size size,
    const std::string& mediaType,
    Handle(Image_Texture)& texture);
//! Validate an already embedded app-authored texture, including its canonical
//! `texture-sha256-...` identifier.
Standard_EXPORT Standard_Boolean Core3DValidateAuthoredTexture(
    const Handle(Image_Texture)& texture);
//! Exact ID-and-byte equality without relying on Image_Texture handle
//! identity. Base-color authoring uses this to enforce synchronized PBR/Common
//! representations; PBR-only slots use it for no-op detection and deduping.
Standard_EXPORT Standard_Boolean Core3DTexturesMatch(
    const Handle(Image_Texture)& first,
    const Handle(Image_Texture)& second);

//! Source-compatible spellings retained for existing native clients. New code
//! should use the semantic-neutral helpers above.
Standard_EXPORT Standard_Boolean Core3DCreateAuthoredBaseColorTexture(
    const Standard_Byte* bytes,
    Standard_Size size,
    const std::string& mediaType,
    Handle(Image_Texture)& texture);
Standard_EXPORT Standard_Boolean Core3DValidateAuthoredBaseColorTexture(
    const Handle(Image_Texture)& texture);
Standard_EXPORT Standard_Boolean Core3DBaseColorTexturesMatch(
    const Handle(Image_Texture)& first,
    const Handle(Image_Texture)& second);

//! Serialization-stable texture budgets shared by writer projection and
//! post-load validation. Encoded bytes are charged for every material slot;
//! decoded bytes are charged once per exact identifier-and-byte resource.
struct Core3DEmbeddedTextureBudgetState
{
    Standard_Size serializedOccurrenceBytes = 0;
    Standard_Size decodedResourceBytes = 0;
    std::unordered_map<
        std::string, Handle(NCollection_Buffer)> resourcesByIdentifier;
};

Standard_EXPORT Standard_Boolean
Core3DAccumulateEmbeddedTextureBudget(
    const Handle(Image_Texture)& texture,
    Core3DEmbeddedTextureBudgetState& state,
    Standard_Size maximumSerializedOccurrenceBytes,
    Standard_Size maximumDecodedResourceBytes);

//! One whole-object PBR material update. Batch persistence uses the complete
//! set to prove the final serialized texture-occurrence budget before it
//! mutates the immutable visual-material table.
struct OcctPBRMaterialUpdate
{
    TDF_Label label;
    XCAFDoc_VisMaterialPBR material;
    Handle(Image_Texture) prevalidatedBaseColorTexture;
    Handle(Image_Texture) prevalidatedEmissiveTexture;
};

//! Register the app-owned BinOcaf/BinXCAF project formats with a narrow,
//! fail-closed attribute schema and bounded visual-material/string readers.
//! This is defense in depth for trusted Shapeyard project packages; raw XBF/CBF
//! remains a private persistence format and must not be exposed as an arbitrary
//! untrusted import surface without a separately hardened OCCT shape parser.
Standard_EXPORT void Core3DDefineSafeBinXCAFFormat(
    const Handle(TDocStd_Application)& application);
//! Reset/query the current thread's fail-closed retrieval signal. OCCT treats
//! a driver Paste(false) as a warning, so every Open must bracket and inspect
//! this signal before accepting the returned document.
Standard_EXPORT void Core3DBeginSafeBinaryRead();
Standard_EXPORT Standard_Boolean Core3DSafeBinaryReadWasRejected();

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
  //! Read the persisted representation without modifying the document.
  //! Missing attributes on valid legacy BRep are returned as LegacyUnknown.
  //! Invalid is returned for a non-definition label, unknown integer, or a
  //! marker/geometry mismatch.
  Standard_EXPORT OcctGeometryRepresentation GeometryRepresentationForLabel(
      const TDF_Label& label) const;
  //! Validate one definition's marker against its stored geometry. A missing
  //! or explicit LegacyUnknown marker is accepted only for legacy BRep.
  Standard_EXPORT Standard_Boolean ValidateGeometryRepresentationForLabel(
      const TDF_Label& label) const;
  //! Validate all definitions without stamping or otherwise mutating OCAF.
  //! An empty document is valid and representation-neutral.
  Standard_EXPORT Standard_Boolean ValidateGeometryRepresentations() const;
  Standard_EXPORT Standard_Boolean ValidateGeometryRepresentations(
      const Handle(TDocStd_Document)& document) const;
  //! Return the document-wide intersection of safe export formats. Invalid or
  //! empty documents return zero; any TriangleMesh definition removes STEP.
  Standard_EXPORT Standard_Integer SupportedGeometryExportFormats() const;
  Standard_EXPORT Standard_Boolean CanExportGeometry(
      OcctGeometryExportFormat format) const;
  //! Distinguish a valid committed empty document from an invalid document.
  //! Empty export handoffs remain constructible so the format writer can
  //! preserve its established no-artifact/error contract.
  Standard_EXPORT Standard_Boolean IsGeometryDocumentEmpty() const;
  //! Set/copy a definition marker inside the caller's existing OCAF command.
  //! These methods never open, commit, or abort a command. Copy resolves a
  //! valid LegacyUnknown source to an explicit BRep destination marker.
  Standard_EXPORT Standard_Boolean SetGeometryRepresentationForLabel(
      const TDF_Label& label,
      OcctGeometryRepresentation representation);
  //! Validate the current marker for a definition mutation. A valid legacy
  //! BRep is stamped BRep inside the caller's already-open command; explicit
  //! BRep/TriangleMesh markers are preserved.
  Standard_EXPORT Standard_Boolean EnsureGeometryRepresentationForMutation(
      const TDF_Label& label);
  Standard_EXPORT Standard_Boolean CopyGeometryRepresentation(
      const TDF_Label& source,
      const TDF_Label& destination);
  //! Stamp every unmarked analytic definition produced by a fresh STEP
  //! transfer. This isolated-import schema operation requires zero user
  //! history, creates no retained undo entry, and is never called for legacy
  //! project load.
  Standard_EXPORT Standard_Boolean MarkImportedBRepDefinitions();
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
    //! Persist a renderer-neutral XCAF metallic-roughness material. The caller
    //! must own the surrounding document command so assignment, Undo, and Redo
    //! remain one atomic edit.
    Standard_Boolean SaveObjectPBRMaterial(
        const TDF_Label& label,
        const XCAFDoc_VisMaterialPBR& material);
    //! Save using a base-color handle already validated by the current bounded
    //! authoring operation. The exact handle must equal material's base-color
    //! texture; the supported emissive slot and every scalar/document invariant
    //! remain checked independently.
    Standard_Boolean SaveObjectPBRMaterial(
        const TDF_Label& label,
        const XCAFDoc_VisMaterialPBR& material,
        const Handle(Image_Texture)& prevalidatedBaseColorTexture);
    //! Save using base-color and emissive handles already validated by the
    //! current bounded authoring operation. Each nonnull handle must be the
    //! exact handle stored in its corresponding PBR slot.
    Standard_Boolean SaveObjectPBRMaterial(
        const TDF_Label& label,
        const XCAFDoc_VisMaterialPBR& material,
        const Handle(Image_Texture)& prevalidatedBaseColorTexture,
        const Handle(Image_Texture)& prevalidatedEmissiveTexture);
    //! Validate and persist a complete authoring batch. The final material
    //! definition set is checked against the safe reader's per-serialized-slot
    //! texture-byte budget before any table entry is added, removed, or linked.
    Standard_Boolean SaveObjectPBRMaterials(
        const std::vector<OcctPBRMaterialUpdate>& updates);
    //! DEBUG seam for exercising aggregate occurrence limits with small valid
    //! images. Values above the production 128 MiB ceiling reset to the
    //! production ceiling.
    void SetMaximumSerializedTextureOccurrenceBytesForTesting(
        Standard_Size maximumBytes);
    //! DEBUG seam for unique decoded-resource aggregate budget tests.
    void SetMaximumDecodedTextureResourceBytesForTesting(
        Standard_Size maximumBytes);
    //! DEBUG seam for proving batch replacement at a full immutable material
    //! table without allocating thousands of definitions.
    void SetMaximumVisualMaterialDefinitionsForTesting(
        Standard_Size maximumDefinitions);
    //! Remove a canonical XCAF material assignment before applying a legacy
    //! preset/color. Existing legacy projects continue to load unchanged.
    Standard_Boolean ClearObjectVisualMaterial(const TDF_Label& label);
    //! Copy the effective source appearance without mutating shared XCAF
    //! material definitions. Used by duplicate and topology-changing tools.
    Standard_Boolean CopyObjectAppearance(
        const TDF_Label& source,
        const TDF_Label& destination);
    void LoadObjectMeterial(const TDF_Label& label, const Handle(AIS_Shape) anAis);
    //! Apply only Shapeyard-owned whole-object overrides. Imported XCAF
    //! material is deliberately excluded because an occurrence presentation
    //! already owns the explorer-resolved definition/instance style.
    void LoadObjectAuthoredMaterialOverrides(
        const TDF_Label& label,
        const Handle(AIS_Shape) anAis);

    TDF_Label AddShape(Handle(AIS_Shape) object);
    TDF_Label AddShape(
        Handle(AIS_Shape) object,
        OcctGeometryRepresentation representation);
    TDF_Label AddShape(Handle(AIS_InteractiveObject) object);
    TDF_Label AddShape(
        Handle(AIS_InteractiveObject) object,
        OcctGeometryRepresentation representation);
    //! False for an XCAF component occurrence whose persistent edits cannot be
    //! represented safely by the current definition-owned editing model.
    Standard_Boolean IsPresentationEditable(
        Handle(AIS_InteractiveObject) object) const;
    //! True only for a free, simple, whole XCAF definition label. Material
    //! authoring uses this stricter gate in addition to AIS editability.
    Standard_Boolean IsEditableFreeSimpleDefinitionLabel(
        const TDF_Label& label) const;
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
    Standard_Boolean TryPBRMaterialForLabel(
        const TDF_Label& label,
        XCAFDoc_VisMaterialPBR& material) const;
    //! Read the effective whole-object PBR assignment used by presentation and
    //! editing. Unlike TryPBRMaterialForLabel(), this also accepts imported
    //! XCAF materials when no legacy Shapeyard override takes precedence.
    Standard_Boolean TryEffectivePBRMaterialForLabel(
        const TDF_Label& label,
        XCAFDoc_VisMaterialPBR& material) const;
    //! False when scalar authoring would discard any texture owned by either
    //! the PBR or Common representation of the assigned visual material.
    Standard_Boolean SupportsScalarPBRMaterialEditingForLabel(
        const TDF_Label& label) const;
    //! True when base-color texture assignment/removal can be represented
    //! without discarding unsupported PBR/Common texture maps.
    Standard_Boolean SupportsBaseColorTextureEditingForLabel(
        const TDF_Label& label) const;
    //! True when emissive texture assignment/removal can be represented without
    //! discarding unsupported maps or a non-authored base/Common resource.
    Standard_Boolean SupportsEmissiveTextureEditingForLabel(
        const TDF_Label& label) const;
    //! Persistent, undoable provenance for the editor's zero-to-white
    //! emissive-factor promotion. The marker lets texture removal restore zero
    //! without destroying a pre-existing nonzero factor.
    Standard_Boolean IsEmissiveTextureFactorAutoPromotedForLabel(
        const TDF_Label& label) const;
    Standard_Boolean SetEmissiveTextureFactorAutoPromotedForLabel(
        const TDF_Label& label,
        Standard_Boolean isAutoPromoted);

    //! Replace geometry on an existing editable free definition. The caller
    //! must own an open command on this exact document; identifiers and
    //! appearance remain attached to the stable label.
    Standard_Boolean ReplaceShape(
        const TDF_Label& label,
        Handle(AIS_Shape) aisShape);
    
    void RemoveShape(TopoDS_Shape object);
    void RemoveShape(Handle(AIS_Shape) object);
    void RemoveShape(Handle(AIS_InteractiveObject) object);
    Standard_Boolean RemoveShape(const TDF_Label& label);
    
    void ApplyTransforms();
    //! Bake app-owned object transforms into every free shape while observing
    //! the supplied worker progress/cancellation range. Returns false when the
    //! operation was interrupted before every root was displaced.
    Standard_Boolean ApplyTransforms(
        const Message_ProgressRange& progress);

    //! Open an app-created private XBF handoff into this otherwise empty
    //! document. This is intentionally narrower than general import and may be
    //! called only by an isolated native-export worker.
    Standard_EXPORT Standard_Boolean OpenPrivateExportSnapshot(
        const std::string& path,
        const Message_ProgressRange& progress);
    //! Deterministically release the worker-owned export document.
    Standard_EXPORT void ClosePrivateExportSnapshot() noexcept;

	Standard_Boolean undo();
	Standard_Boolean redo();
    const bool canUndo() const;
    const bool canRedo() const;
    
    std::string save(const std::string& path);
    std::string save(
        const std::string& path,
        const Message_ProgressRange& progress);

    void NotifyChanges();

private:
    Standard_Boolean CanSaveObjectPBRMaterials(
        const std::vector<OcctPBRMaterialUpdate>& updates,
        std::vector<TDF_Label>* reclaimMaterialLabels) const;
    
  Handle(TDocStd_Application) myApp;
  Handle(TDocStd_Document) myOcafDoc;
  Standard_Size myMaximumSerializedTextureOccurrenceBytes;
  Standard_Size myMaximumDecodedTextureResourceBytes;
  Standard_Size myMaximumVisualMaterialDefinitions;
};

#endif // OcctDocument_h
