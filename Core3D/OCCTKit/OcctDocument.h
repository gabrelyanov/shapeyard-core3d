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
#include "RectangularLoftPersistence.hxx"
#include "RectangularLoftRebuild.hxx"
#include "CylindricalCutDefinition.hxx"
#include "SavedCutSourceEdit.hxx"
#include "RetainedBooleanEditValues.hxx"

#include <XCAFApp_Application.hxx>
#include <TDocStd_Document.hxx>
#include <AIS_InteractiveObject.hxx>
#include <AIS_Shape.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterialPBR.hxx>
#include <Image_Texture.hxx>
#include <NCollection_Buffer.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <cstdint>
#include "ProfilePersistence.hxx"
#include "EnclosurePersistence.hxx"

#include <TDF_Data.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TopoDS_Shape.hxx>
#include <array>
#include <string>
#include <unordered_map>
#include <vector>
#include <optional>
#include <map>
#include <memory>
#include <cmath>
#include "SourceFaceProvenanceRecord.hxx"
#include "CurvedFaceUVUnwrap.hpp"
class XCAFDoc_VisMaterial;

class Message_ProgressRange;
namespace core3d { class OrdinaryEditController; class SavedCutSourceDetachedResult; class SavedProgramSourceDetachedResult; }

//! Persistent geometry representation owned by each XCAF definition label.
//! The non-negative values are serialized schema values: never renumber or
//! reuse them. Invalid is a read-only fail-closed sentinel and must never be
//! written to a document.
enum class OcctMeshWindingRepairResult { Invalid, Unchanged, Prepared };

enum class OcctGeometryRepresentation : Standard_Integer
{
    Invalid = -1,
    LegacyUnknown = 0,
    BRep = 1,
    TriangleMesh = 2,
};

//! Exact admitted whole-object scalar appearance for source-retained mesh copies.
//! No textures, per-face styles or imported color links are silently discarded.
struct OcctScalarAppearanceState {
    TDF_Label materialLabel;
    std::array<bool,2> legacyPresent = {};
    std::array<Standard_Integer,2> legacyValues = {};
    bool localPBR = false;
    // Captured numeric values, never a mutable material handle or tolerant
    // Quantity_Color comparison. Fixed field order is internal, not a schema.
    std::vector<double> visualValues;
    bool IsEqual(const OcctScalarAppearanceState& other) const noexcept {
        const bool sameLabel = materialLabel.IsNull() ? other.materialLabel.IsNull()
            : !other.materialLabel.IsNull() && materialLabel.IsEqual(other.materialLabel)
                && materialLabel.Data()==other.materialLabel.Data();
        return sameLabel && legacyPresent==other.legacyPresent && legacyValues==other.legacyValues
            && localPBR==other.localPBR && visualValues==other.visualValues;
    }
};

struct OcctMeshUVAtlasPreview {
    Standard_Integer authoredResolution = 0;
    Standard_Integer authoredGutterPixels = 0;
    std::vector<double> triangleUVs;
    Standard_Integer chartCount = 0;
    Standard_Real occupancy = 0;
};

struct OcctMeshUVAtlasOptions {
    Standard_Integer version = 1;
    Standard_Integer resolution = 0;
    Standard_Integer gutterPixels = 0;
    // Explicit retained native source for live D5 reads. Null keeps planar behavior.
    // Carried by OrdinaryTransformChange through validation and Mark in one Undo.
    TDF_Label curvedSource = {};
};

//! Read-only resolved region. Ordinals are session-local and never persistent IDs.
struct OcctMeshRegionExtrudePreview {
    std::vector<std::uint32_t> triangleIndices;
    std::vector<std::array<Standard_Real,3>> localBoundary;
    std::array<Standard_Real,3> localUnitNormal = {};
};

//! Prepared mesh mutation output. Partition bytes are canonical OCAF payload;
//! empty is permitted only when no partition existed or extrusion proved that
//! every former barrier became a natural noncoplanar boundary.
struct OcctMeshRegionMutationCandidate {
    TopoDS_Shape shape;
    std::vector<Standard_Byte> partition;
    std::uint32_t centerSeedTriangle = 0; // Inset only; session-local.
};

//! Vertex-coordinate mutation plus opaque rebound region authority.
struct OcctMeshVertexMutationCandidate {
    TopoDS_Shape shape;
    std::vector<Standard_Byte> partition;
};

//! Exact durable state for transform reconciliation. Attribute presence is
//! significant: a missing legacy default and an authored zero are not the
//! same OCAF state, even when their resulting matrices match. No AIS handles.
struct OcctObjectTransformState
{
    TDF_Label label;
    Handle(TDF_Data) documentData;
    TopoDS_Shape shape;
    core3d::profile::Record profile;
    core3d::enclosure::Record enclosure;
    core3d::sweep_persistence::Record sweep;
    core3d::loft_persistence::Record loft;
    core3d::retained_solid::Record retained;
    gp_Trsf transform;
    std::array<Standard_Real, 8> scalars = {{0, 0, 0, 0, 0, 0, 1, 1}};
    std::array<Standard_Boolean, 8> present = {};
    std::string entityIdentifier;
    std::string definitionIdentifier;
    Standard_Integer meshUVAtlasVersion = 0; // 0 absent, 1 grid, 2 coherent atlas, 3 authored flat-corner UV layout
    std::array<Standard_Integer, 3> meshUVAtlasSettings = {}; // resolution, gutter, original-node prefix; v2 only
    // Canonical SYRP bytes. Empty is the only absent representation; any
    // persisted malformed record makes state capture fail closed.
    std::vector<Standard_Byte> meshRegionPartition;
    Standard_Boolean authoredFramesPresent = Standard_False;
    std::array<Standard_Byte, 32> authoredFramesIdentity = {}; // validated immutable archive digest
    OcctGeometryRepresentation storedRepresentation = OcctGeometryRepresentation::Invalid;
    OcctGeometryRepresentation resolvedRepresentation = OcctGeometryRepresentation::Invalid;

    //! Invalid/default snapshots never compare equal. Exact stored scalars,
    //! oriented shape identity, label/data and metadata must all agree.
    Standard_EXPORT Standard_Boolean IsEqual(
        const OcctObjectTransformState& other) const noexcept;
};

//! Main-only source data. Viewer-issued wrapper additionally binds selection,
//! source stamp, publication/revisions and exact catalog; this is not a lease.
struct OcctCylindricalCutSource {
    OcctObjectTransformState original;
    core3d::retained_solid::Envelope envelope; // Creation template has no derived UUID/radius until preparation.
    TopoDS_Shape base;
    double effectiveMM=0;
    bool rebuilding=false;
};

//! Main-only whole-program source data for the public append/radius packet.
//! The COMPLETE retained recipe (legacy or program) plus its exact persisted
//! bytes; no single-bore projection is exposed as native authority. Capture
//! requires an existing retained carrier: bare-source first cuts stay legacy.
struct OcctCylindricalCutProgramSource {
    OcctObjectTransformState original;
    core3d::retained_boolean::Recipe recipe;
    std::vector<std::uint8_t> recipeBytes;
    TopoDS_Shape base;
    double effectiveMM=0;
};

//! Exact authored name and object authority. Attribute absence is distinct
//! from an authored empty legacy name; read paths never create attributes.
struct OcctObjectNameState
{
    OcctObjectTransformState object;
    Standard_Boolean namePresent = Standard_False;
    TCollection_ExtendedString name;
    Standard_EXPORT Standard_Boolean IsEqual(const OcctObjectNameState& other) const noexcept;
};

//! Flat saved groups keep members as editable free definitions in world space.
//! Record labels and an optional container are exact persistent authority.
//! Empty records may remain after deleting their last object, but are not UI groups.
struct OcctSavedGroup {
    TDF_Label recordLabel;
    std::string identifier;
    TCollection_ExtendedString name;
    Standard_Boolean originPresent = Standard_False;
    gp_Pnt origin; //!< Finite document/world coordinates for the flat group.
    std::vector<TDF_Label> members;
};
struct OcctSavedGroupState {
    Handle(TDF_Data) documentData;
    TDF_Label container;
    std::vector<OcctSavedGroup> groups;
    Standard_EXPORT Standard_Boolean IsEqual(const OcctSavedGroupState& other) const noexcept;
};

//! Exact object visibility plus the layer associations that can independently
//! hide it. This family supports editable free definitions, never occurrences.
struct OcctObjectVisibilityState {
    OcctObjectNameState object;
    Standard_Boolean invisibleAttributePresent = Standard_False;
    Standard_Boolean layerLinkPresent = Standard_False;
    std::vector<TDF_Label> layers;
    std::vector<bool> layerInvisibleAttributePresent;
    Standard_EXPORT Standard_Boolean HasSameObjectAndLayers(const OcctObjectVisibilityState& other) const noexcept;
    Standard_EXPORT Standard_Boolean IsEqual(const OcctObjectVisibilityState& other) const noexcept;
    Standard_EXPORT Standard_Boolean IsEffectivelyVisible() const noexcept;
};

//! A bounded nonempty Unicode object name. Display names are never identity.
Standard_EXPORT Standard_Boolean OcctObjectNameIsValid(
    const TCollection_ExtendedString& name) noexcept;

//! Export formats supported directly by the persisted geometry contract.
//! Values are bit flags returned by SupportedGeometryExportFormats().
enum class OcctGeometryExportFormat : Standard_Integer
{
    Obj = 1 << 0,
    Stl = 1 << 1,
    Gltf = 1 << 2,
    Step = 1 << 3,
};

//! Coordinate authority for one component of a persisted reference axis.
//! The non-negative values are serialized schema values: never renumber or
//! reuse them.
enum class OcctReferenceSpace : Standard_Integer
{
    Object = 0,
    World = 1,
};

//! A definition-owned oriented line used by operations such as Radial Array.
//! Pivot and direction deliberately have independent spaces so common mixed
//! choices such as Object Origin + World Z remain representable. A full frame
//! is intentionally absent because roll has no meaning for an oriented line.
struct OcctReferenceAxis
{
    OcctReferenceSpace pivotSpace = OcctReferenceSpace::Object;
    gp_Pnt pivot = gp_Pnt(0.0, 0.0, 0.0);
    OcctReferenceSpace directionSpace = OcctReferenceSpace::World;
    gp_Dir direction = gp_Dir(0.0, 0.0, 1.0);
};

//! Missing legacy metadata is one explicit, non-mutating default. Invalid is
//! fail-closed and is never silently treated as missing.
enum class OcctReferenceAxisReadState : Standard_Integer
{
    Invalid = -1,
    ImplicitDefault = 0,
    Authored = 1,
};

//! Persistent internal attribute used to prove ownership of one Duplicate
//! OCAF command across fail-closed commit/abort reconciliation. It may exist
//! only as TDataStd_Integer on TDocStd_Document::Main().
Standard_EXPORT const Standard_GUID&
Core3DDuplicateCommandOwnerAttributeID();

//! Persistent internal attribute used to prove ownership of one Radial Array
//! OCAF command across fail-closed commit/abort reconciliation. Like the
//! Duplicate sentinel, it may exist only as TDataStd_Integer on Main().
Standard_EXPORT const Standard_GUID&
Core3DRadialArrayCommandOwnerAttributeID();

//! Shared ordinary-edit ownership sentinel; Integer on document Main only.
//! Marker observations prove closure/ownership, never geometry outcome.
Standard_EXPORT const Standard_GUID&
Core3DOrdinaryEditCommandOwnerAttributeID();

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

//! Replace XCAF renderer wrappers with role-aware, presentation-only wrappers.
//! Persistent image IDs/bytes remain unchanged; GPU identity includes whether
//! the binding carries color or numeric samples. Safe to call repeatedly.
Standard_EXPORT void Core3DPrepareRendererTextures(
    const Handle(Graphic3d_AspectFillArea3d)& aspect);

//! Validate the bounded opaque 8-bit PNG numeric channel representation.
//! This checks actual decoded samples/layout, not only the image's media type.
Standard_EXPORT Standard_Boolean Core3DValidateNumericTexture(
    const Handle(Image_Texture)& texture);

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
//! Read-only geometry validation shared by live normal authoring and isolated
//! saved-document admission. Never meshes or writes OCAF/cache attributes.
Standard_EXPORT Standard_Boolean Core3DValidateNormalTextureGeometry(
    const TDF_Label& label, Standard_Size* requiredNativeBytes = nullptr) noexcept;
//! Persisted owned-normal recipe: 0 absent, 1 pinned Mikk v1, 2 supplied frames, -1 invalid/unknown.
Standard_EXPORT Standard_Integer Core3DNormalTextureRecipeForLabel(
    const TDF_Label& label) noexcept;

enum class OcctAuthoredFrameReadState { Invalid = -1, Absent = 0, Authored = 1 };
struct OcctAuthoredFrameRecord {
    std::vector<Standard_Byte> archive;
    std::array<Standard_Byte, 32> identity = {};
    Standard_Size cornerCount = 0;
    Standard_Size nativeBytes = 0; // retained archive plus 64 bytes per expanded corner
};
//! Read-only local mesh ownership and exact geometry association. Clears output
//! on absence/failure; never returns a mutable OCAF attribute to an edit lease.
Standard_EXPORT OcctAuthoredFrameReadState Core3DReadAuthoredFrameOwner(
    const Handle(TDocStd_Document)& document, const TDF_Label& label,
    OcctAuthoredFrameRecord& record) noexcept;
//! Full existing document admission, including retained owner and shape budgets.
Standard_EXPORT Standard_Boolean Core3DValidateRetainedSolidDocument(const Handle(TDocStd_Document)& document);

//! Scans every label, including hidden/unbound/orphan records and foreign arrays.
//! This validates frame ownership, not the rest of the document schema. Callers
//! must combine it with their existing geometry/material admission and budgets.
//! A smaller cap supports exact-limit checks; a cap above 64 MiB is rejected.
Standard_EXPORT Standard_Boolean Core3DValidateAuthoredFrameOwners(
    const Handle(TDocStd_Document)& document, Standard_Size& nativeBytes,
    Standard_Size maximumBytes = 64U * 1024U * 1024U) noexcept;
//! Native basis capability:0 invalid,1 Mikk,2 validated supplied frames.
//! Additional bytes exclude supplied archives/expanded corners already charged
//! by Core3DValidateAuthoredFrameOwners. Never replaces an invalid supplied basis.
Standard_EXPORT Standard_Integer Core3DNormalTextureBasisForLabel(
    const Handle(TDocStd_Document)& document, const TDF_Label& label,
    Standard_Size* additionalNativeBytes = nullptr) noexcept;
//! A bound normal map must name the exact basis owned by its native geometry.
Standard_EXPORT Standard_Boolean Core3DValidateNormalTextureBinding(
    const Handle(TDocStd_Document)& document, const TDF_Label& label,
    Standard_Size* additionalNativeBytes = nullptr) noexcept;
//! Complete supplied-owner plus owned-normal usage, including hidden bindings
//! and orphan recipe rejection. Legacy unowned/no-frame materials remain separate.
Standard_EXPORT Standard_Boolean Core3DValidateOwnedFrameUsage(
    const Handle(TDocStd_Document)& document, Standard_Size& nativeBytes,
    Standard_Size maximumBytes = 64U * 1024U * 1024U) noexcept;
enum class OcctMaterialTextureSlot { BaseColor, Emissive, MetallicRoughness, Occlusion, Normal };
Standard_EXPORT Handle(Image_Texture)& Core3DMaterialTexture(
    XCAFDoc_VisMaterialPBR& material, OcctMaterialTextureSlot slot);

//! Optional fields are patches, not a round-tripped full display material.
struct OcctPBRScalarPatch {
    std::optional<std::array<double,3>> baseColorSRGB;
    std::optional<double> metallic, roughness;
    bool IsValid() const noexcept {
        const auto unit=[](double x){return std::isfinite(x)&&x>=0&&x<=1;};
        if (!baseColorSRGB&&!metallic&&!roughness) return false;
        if (baseColorSRGB) for(double x:*baseColorSRGB) if(!unit(x)) return false;
        return (!metallic||unit(*metallic))&&(!roughness||unit(*roughness));
    }
};
// Private immutable values; no mutable material or viewer handle is exposed.
#ifdef DEBUG
struct OcctPBRScalarDebugEvidence {
    std::vector<std::uint8_t> material,preserved,table;
    std::map<std::string,std::array<unsigned char,32>> geometry;
    std::map<std::string,std::vector<std::uint8_t>> geometryStreams;
};
#endif
struct OcctSavedCutSceneState;
struct OcctPBRScalarState;
struct OcctPBRScalarPreparation;

struct OcctPBRMaterialUpdate
{
    TDF_Label label;
    XCAFDoc_VisMaterialPBR material;
    Handle(Image_Texture) prevalidatedBaseColorTexture;
    Handle(Image_Texture) prevalidatedEmissiveTexture;
};

//! Multiplicity-aware projection for operations that create independent free
//! definitions. A source may appear at most once in one admission request;
//! destinationCount is the number of complete geometry/appearance copies.
struct OcctGeometryDuplicationRequest
{
    TDF_Label sourceDefinition;
    Standard_Size destinationCount = 0;
    // A geometry-baking copy adds a construction frame to an existing profile.
    // Existing framed recipes already include these eight scalar labels.
    bool requiresProfileConstructionFrame = false;
    // Explicitly owned copies only. Default callers keep rejecting enclosures;
    // admitted owners must preserve their recipe in staging and recovery.
    bool preservesEnclosureRecipe = false;
    // Baked enclosure copies compose an eight-scalar construction frame.
    // Charge the additional labels only when the source has no frame yet.
    bool requiresEnclosureConstructionFrame = false;
};

//! Register the app-owned BinOcaf/BinXCAF project formats with a narrow,
//! fail-closed attribute schema and bounded visual-material/string readers.
//! This is defense in depth for trusted Shapeyard project packages; raw XBF/CBF
//! remains a private persistence format and must not be exposed as an arbitrary
//! untrusted import surface without a separately hardened OCCT shape parser.
Standard_EXPORT void Core3DDefineSafeBinXCAFFormat(
    const Handle(TDocStd_Application)& application);
#include "NativeEditAuthority.hpp"
#include <memory>
namespace core3d::authority { class NativeObservedApplication; }
#if DEBUG
#include <map>
#include <string>
//! Private full-reader framing fixtures; no receipt owner or authority integration.
Standard_EXPORT std::map<std::string, bool> Core3DDebugReceiptFramingProbe(Standard_Integer scenario);
Standard_EXPORT std::map<std::string,bool> Core3DDebugRetainedSolidProbe(Standard_Integer scenario);
//! Detached/value-only prerequisites; no ordinary source-edit authority.
Standard_EXPORT std::map<std::string,bool> Core3DDebugSavedCutSourcePrerequisiteProbe(Standard_Integer scenario);
//! Detached enclosure matcher qualification only; no owner/edit authority.
Standard_EXPORT std::map<std::string,bool> Core3DDebugEnclosureCorrespondenceProbe(Standard_Integer scenario);
//! Read-only recipe/interval qualification; no document, shape or edit authority.
Standard_EXPORT std::map<std::string,bool> Core3DDebugSavedCutBoreClearanceProbe(Standard_Integer scenario);
//! DEBUG observer/whole-result fixtures only; no source-edit authority.
Standard_EXPORT std::map<std::string,bool> Core3DDebugSavedCutResultCorrespondenceProbe(Standard_Integer scenario);
Standard_EXPORT std::map<std::string,bool> Core3DDebugSavedBooleanProgramProbe();
Standard_EXPORT std::map<std::string,bool> Core3DDebugSavedBooleanRingProbe(Standard_Integer scenario);
Standard_EXPORT std::map<std::string,bool> Core3DDebugSavedBooleanFilletProbe(Standard_Integer scenario);
Standard_EXPORT void Core3DDebugSetRetainedFilletFailureCount(Standard_Integer count);
Standard_EXPORT std::map<std::string,bool> Core3DDebugSavedBooleanWedgeProbe(Standard_Integer scenario);
//! DEBUG archive-rounding and adversarial trim-domain checks; no edit authority.
Standard_EXPORT std::map<std::string,bool> Core3DDebugSavedCutTrimDomainProbe();
Standard_EXPORT std::map<std::string,bool> Core3DDebugCircularHostProofProbe(Standard_Integer scenario);
void Core3DDebugDefineLegacyReceiptFormats(const Handle(TDocStd_Application)& application);
namespace core3d::persistence { struct AuthoredFrameReadBudget; }
namespace core3d::debug { struct LiveTransactionProbeState; class LiveObservedApplication; }
//! Isolated tests with a custom wire budget and no final geometry-owner gate.
//! Production registration always validates owner association after retrieval.
Standard_EXPORT void Core3DDebugDefineFrameBinXCAFFormat(
    const Handle(TDocStd_Application)& application,
    const std::shared_ptr<core3d::persistence::AuthoredFrameReadBudget>& budget);
#endif

//! Reset/query the current thread's fail-closed retrieval signal. OCCT treats
//! a driver Paste(false) as a warning, so every Open must bracket and inspect
//! this signal before accepting the returned document.
Standard_EXPORT void Core3DBeginSafeBinaryRead();
Standard_EXPORT Standard_Boolean Core3DSafeBinaryReadWasRejected();

namespace core3d { class NativeDocumentSession; }

//! The document
class OcctDocument : public Standard_Transient
{
  DEFINE_STANDARD_RTTIEXT(OcctDocument, Standard_Transient)
  
public:
  // Benchmark assets need more than 40 steps; 1000 keeps memory bounded on
  // device while preserving a long editable native session.
  static constexpr Standard_Integer kNativeSessionUndoLimit = 1000;

  Standard_EXPORT OcctDocument();

  Standard_EXPORT virtual ~OcctDocument();

  Standard_EXPORT void InitDoc();
  // Internal committed-adoption boundary; no public AI token is exposed.
  void ObserveSuccessfulNativeDocumentAdoption() noexcept;
  // Private live-import ownership; native readiness is checked by the viewer.
  // Native queued file preparation. Admission belongs to the viewer, before
  // cancelling controller workers. These values are never accepted from wire data.
  std::optional<core3d::authority::QueuedLoadReservation> BeginNativeQueuedLoad() noexcept;
  bool OwnsNativeQueuedLoad(const core3d::authority::QueuedLoadReservation&) noexcept;
  core3d::authority::QueuedLoadEnd EndNativeQueuedLoadPrivateWork(
      const core3d::authority::QueuedLoadReservation&, bool privateWorkSettled) noexcept;
  std::optional<core3d::authority::ReplacementReservation> PromoteNativeQueuedLoad(
      const core3d::authority::QueuedLoadReservation&, bool nativeEditReady) noexcept;

  std::optional<core3d::authority::ReplacementReservation> BeginNativeReplacement() noexcept;
  core3d::authority::ReplacementEnd EndNativeReplacement(
      const core3d::authority::ReplacementReservation& reservation,
      bool accepted, bool restored) noexcept;

  // Production planning observation. Readiness is supplied only by the native
  // viewer/controller after checking their actual worker/preview/recovery fences.
  // This is not a serialized token; the public bridge retains it opaquely.
  std::optional<core3d::authority::Stamp> CaptureNativePlanningStamp(bool nativeEditReady) noexcept;
  void ObserveNativePlanningInteraction() noexcept;
#if DEBUG
  std::optional<core3d::authority::Stamp> DebugNativeMutationStamp() noexcept;
#endif
#if DEBUG
  //! Bounded diagnostic observation only; never a production AI edit token.
  Standard_EXPORT bool DebugStartLiveTransactionProbe() noexcept;
  Standard_EXPORT void DebugStopLiveTransactionProbe() noexcept;
  Standard_EXPORT std::shared_ptr<const core3d::debug::LiveTransactionProbeState>
      DebugLiveTransactionProbe() const noexcept;
  Standard_EXPORT bool DebugLiveTransactionProbeValid() const noexcept;
  //! Call only after the live viewer has successfully adopted its candidate.
  void DebugObserveSuccessfulDocumentAdoption() noexcept;
#endif

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
  //! Read only the persisted representation marker after constant-time
  //! definition-label checks. This deliberately does not reclassify stored
  //! geometry and is therefore suitable only for hot paths whose document was
  //! already admitted by the normal load/import/mutation validation gates.
  //! Missing markers are returned as LegacyUnknown; malformed markers or
  //! non-definition labels return Invalid.
  Standard_EXPORT OcctGeometryRepresentation StoredGeometryRepresentationForLabel(
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
  //! Source-compatible count-one wrapper for existing duplication callers.
  Standard_EXPORT Standard_Boolean CanDuplicateGeometryDefinitions(
      const std::vector<TDF_Label>& sourceDefinitionLabels) const;
  //! Read-only preallocation gate for creating the requested number of
  //! independent free definitions per unique source label. The current closed
  //! document and projected result must remain inside every aggregate
  //! geometry, graph, mobile-leaf, and OCAF-label budget. Callers must still
  //! validate the mutated document before committing its command.
  Standard_EXPORT Standard_Boolean CanDuplicateGeometryDefinitions(
      const std::vector<OcctGeometryDuplicationRequest>& requests) const;
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
  //! Copy geometry-owned UV/frame metadata between exact local mesh payloads.
  //! Requires this document's open command; the caller owns abort/reconciliation.
  //! Appearance copying is separate because Boolean/mirror may replace geometry.
  Standard_EXPORT Standard_Boolean CopyGeometryOwnedMeshMetadata(
      const TDF_Label& source, const TDF_Label& destination);
  //! Definition-owned planar-region selection authority. Standard OCAF
  //! attributes only; the caller owns the already-open command.
  //! A geometry replacement must capture/validate the old state, clear its
  //! valid record, write the new shape, then stage already-prepared bytes and
  //! verify exact readback, all in one abortable ordinary command. Stage never
  //! repairs malformed authority and neither shape-first nor partition-first
  //! replacement is admitted without that clear step.
  Standard_EXPORT Standard_Boolean CaptureMeshRegionPartition(
      const TDF_Label& label, std::vector<Standard_Byte>& encoded) const noexcept;
  Standard_EXPORT Standard_Boolean StageMeshRegionPartition(
      const TDF_Label& label, const std::vector<Standard_Byte>& encoded) noexcept;
  Standard_EXPORT Standard_Boolean ClearMeshRegionPartition(
      const TDF_Label& label) noexcept;
  //! Definition-owned source-face provenance for a source-retained mesh copy
  //! (schema v1, standard OCAF attribute types only; no new driver type). The
  //! strict reader recomputes the copy triangulation digest on every read:
  //! Absent (no record), Malformed (schema violation), Stale (a well-formed
  //! record whose digest no longer matches the current triangulation, e.g.
  //! after a mesh region edit, vertex move, or triangle-reordering rebuild)
  //! or Present. Order-preserving atlas rebuilds keep the record Present;
  //! consumers must only act on Present; there is no silent fallback.
  //! Triangle ranges refer to the pre-atlas emission order; parameters are
  //! part-local.
  Standard_EXPORT core3d::provenance::CopySourceFaceProvenanceReadState
      TryCopySourceFaceProvenanceForLabel(
          const TDF_Label& label,
          core3d::provenance::SourceFaceProvenanceRecord& record) const noexcept;
  //! Persist the N1 in-memory provenance on the fresh copy label inside the
  //! caller's already-open creation command, so one Undo removes it and Redo
  //! restores it. Exact readback is required; malformed authority fails closed.
  Standard_EXPORT Standard_Boolean StageCopySourceFaceProvenance(
      const TDF_Label& label,
      const std::vector<core3d::meshcopy::SourceFaceRecord>& faces) noexcept;
#if DEBUG
  Standard_EXPORT Standard_Boolean DebugStageFirstMeshRegionPartition(
      const TDF_Label& label) noexcept;
  Standard_EXPORT Standard_Boolean DebugCorruptMeshRegionPartition(
      const TDF_Label& label, Standard_Integer mode) noexcept;
  //! Transaction-free, self-restoring wrong-length digest probe on a private
  //! snapshot. corruptedState is the mutated read state (-1 if not observed).
  //! Codes: 0 success, 1 not main thread, 2 null document, 3 not Present,
  //! 4 null record label, 5 missing digest, 60+state unexpected mutated read
  //! (61 also covers Malformed with a nonempty digest), 7 restore failed,
  //! 8 exception. No live project or undo command may depend on this probe.
  Standard_EXPORT Standard_Integer DebugCorruptCopySourceFaceProvenance(
      const TDF_Label& label, Standard_Integer& corruptedState) noexcept;
#endif
  //! Stamp every unmarked analytic definition produced by a fresh STEP
  //! transfer. This isolated-import schema operation requires zero user
  //! history, creates no retained undo entry, and is never called for legacy
  //! project load.
  Standard_EXPORT Standard_Boolean MarkImportedBRepDefinitions();
  //! Stamp every unmarked triangle-only definition produced by a fresh,
  //! flattened mesh import. The complete isolated document is classified and
  //! admitted against aggregate mesh limits before an atomic marker command.
  Standard_EXPORT Standard_Boolean MarkImportedTriangleMeshDefinitions();
  //! Return Shapeyard's persisted object-local translation/rotation/uniform
  //! scale. This is independent of an XCAF assembly occurrence location.
  Standard_EXPORT gp_Trsf ObjectTransformForLabel(const TDF_Label& label) const;
  //! Validate every persisted transform scalar before constructing OCCT
  //! quaternion/transform values. Missing legacy scalars use the established
  //! identity defaults; malformed finite, quaternion, scale, or coordinate
  //! values fail closed.
  Standard_EXPORT Standard_Boolean TryObjectTransformForLabel(
      const TDF_Label& label,
      gp_Trsf& transform) const;

  //! Read the definition-owned oriented line without mutating OCAF. Seven
  //! absent attributes return Object Origin + World Z as ImplicitDefault;
  //! partial, unknown, misplaced, nonfinite, oversized, or non-unit records
  //! return Invalid.
  Standard_EXPORT OcctReferenceAxisReadState ReadReferenceAxisForLabel(
      const TDF_Label& label,
      OcctReferenceAxis& axis) const;
  //! Resolve Object-bound components through the validated persisted object
  //! transform and occurrence location. World-bound components remain fixed.
  Standard_EXPORT Standard_Boolean ResolveReferenceAxisInWorld(
      const TDF_Label& label,
      const TopLoc_Location& occurrenceLocation,
      gp_Ax1& axis) const;
  //! Write/reset/copy a complete reference record inside the caller's already
  //! open command. These methods never open, commit, or abort a command.
  Standard_EXPORT Standard_Boolean SetReferenceAxisForLabel(
      const TDF_Label& label,
      const OcctReferenceAxis& axis);
  Standard_EXPORT Standard_Boolean ResetReferenceAxisForLabel(
      const TDF_Label& label);
  Standard_EXPORT Standard_Boolean CopyReferenceAxis(
      const TDF_Label& source,
      const TDF_Label& destination);
  //! Copy through the exact source-local to destination-local transform used
  //! by a geometry bake. Object-bound components are transformed; World-bound
  //! components are copied unchanged.
  Standard_EXPORT Standard_Boolean CopyReferenceAxisThroughBakedTransform(
      const TDF_Label& source,
      const TDF_Label& destination,
      const gp_Trsf& sourceLocalToDestinationLocal);
  //! Validate reference attributes document-wide, including their placement.
  //! This read-only check performs no topology, meshing, or AIS traversal.
  Standard_EXPORT Standard_Boolean ValidateReferenceAxes() const;
  Standard_EXPORT Standard_Boolean ValidateReferenceAxes(
      const Handle(TDocStd_Document)& document) const;

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

    //! Stage all transform scalars in the caller's open command and verify
    //! exact attribute readback. Success proves staging only, never commit.
    Standard_Boolean SaveObjectTransform(
        const TDF_Label& label, const Handle(AIS_Shape) anAis);
    void LoadObjectTransform(const TDF_Label& label, const Handle(AIS_Shape) anAis);
    Standard_EXPORT Standard_Boolean CaptureObjectVisibilityStateForLabel(
        const TDF_Label& label, OcctObjectVisibilityState& state) const noexcept;
    //! Stage the object's own flag only. Showing a layer-hidden object is
    //! rejected without editing its layer or publishing a false visible result.
    Standard_EXPORT Standard_Boolean SetObjectVisibilityForLabel(
        const TDF_Label& label, Standard_Boolean visible) noexcept;
    //! Read-only, bounded semantic validation; no lazy metadata allocation.
    Standard_EXPORT static std::string NewSavedGroupIdentifier() noexcept;
    //! Allocate a new feature identity without opening or mutating a document.
    Standard_EXPORT static std::string NewProfileIdentifier() noexcept;
    Standard_EXPORT static Standard_Boolean IsAdmittedSavedGroupOrigin(const gp_Pnt& point) noexcept;
    Standard_EXPORT Standard_Boolean CaptureSavedGroups(OcctSavedGroupState& state) const noexcept;
    //! Replace the bounded catalog in an already owned command. Input record
    //! labels are ignored; stable group IDs retain their canonical record slots.
    //! Caller must abort on false and prove closure independently.
    Standard_EXPORT Standard_Boolean StageSavedGroups(const std::vector<OcctSavedGroup>& groups) noexcept;
    Standard_EXPORT Standard_Boolean CaptureObjectNameStateForLabel(
        const TDF_Label& label, OcctObjectNameState& state) const noexcept;
    //! Stage only the name in the caller's open command; verify exact readback
    //! and unchanged geometry/identity/transform. Never commit or notify here.
    Standard_EXPORT Standard_Boolean SetObjectNameForLabel(
        const TDF_Label& label, const TCollection_ExtendedString& name) noexcept;
    //! Read-only main-thread capture, valid during an owned open command or
    //! after closure. Does not create labels/attributes or repair metadata.
    //! On failure clears the output so a caller cannot reuse stale proof.
    Standard_EXPORT Standard_Boolean CaptureObjectTransformStateForLabel(
        const TDF_Label& label, OcctObjectTransformState& state) const noexcept;
    //! Write exactly one translation scalar in the caller's already-open OCAF
    //! command. Axis is 0...2 and value uses raw document model units. The
    //! definition, representation, coordinate ceiling, and read-back are
    //! independently validated; untouched rotation/scale attributes are never
    //! rewritten.
    Standard_EXPORT Standard_Boolean SetObjectPositionComponentForLabel(
        const TDF_Label& label,
        Standard_Integer axis,
        Standard_Real value);

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
#ifdef DEBUG
    Standard_EXPORT std::optional<OcctPBRScalarDebugEvidence> DebugPBRScalarEvidence(const TDF_Label&) const noexcept;
#endif
    // Complete immutable source and staged readback for one scalar appearance edit.
    Standard_EXPORT std::shared_ptr<const OcctPBRScalarPreparation> PreparePBRScalarPatch(
        const TDF_Label&, const OcctPBRScalarPatch&, bool& changed) const noexcept;
    Standard_EXPORT std::shared_ptr<const OcctPBRScalarState> PBRScalarOriginal(
        const std::shared_ptr<const OcctPBRScalarPreparation>&) const noexcept;
    Standard_EXPORT TDF_Label PBRScalarTarget(const std::shared_ptr<const OcctPBRScalarPreparation>&) const noexcept;
    // Cut-only read evidence; cannot grant a lease or choose ignored fields.
    Standard_EXPORT std::shared_ptr<const OcctSavedCutSceneState> CaptureSavedCutSceneState(const TDF_Label&) const noexcept;
    Standard_EXPORT bool SavedCutSceneStateMatches(const std::shared_ptr<const OcctSavedCutSceneState>&) const noexcept;
    Standard_EXPORT bool PBRScalarStateMatches(const std::shared_ptr<const OcctPBRScalarState>&) const noexcept;
    Standard_EXPORT bool StagePBRScalarPatch(const std::shared_ptr<const OcctPBRScalarPreparation>&,
        std::shared_ptr<const OcctPBRScalarState>& sealed) noexcept;
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
    // Bounded scalar-only metadata guard for the saved-sweep edit catalog.
    // Unsupported textures/imported color links are refused, never discarded.
    Standard_EXPORT Standard_Boolean CaptureScalarAppearanceForSavedCut(
        const TDF_Label& label,OcctScalarAppearanceState& output)const noexcept;
    Standard_EXPORT Standard_Boolean CaptureCylindricalCutSource(
        const TDF_Label& label,OcctCylindricalCutSource& output)const noexcept;
    Standard_EXPORT Standard_Boolean CaptureCylindricalCutProgramSource(
        const TDF_Label& label,OcctCylindricalCutProgramSource& output)const noexcept;
    Standard_EXPORT Standard_Boolean CaptureRetainedFilletAnchors(const TDF_Label& label,
        const std::vector<core3d::retained_fillet::EdgeAnchor>& requested,
        std::vector<core3d::retained_fillet::EdgeAnchor>& captured,
        core3d::retained_fillet::Outcome* outcome=nullptr) const noexcept;
    Standard_EXPORT core3d::retained_fillet::Candidates RetainedFilletCandidates(const TDF_Label& label) const noexcept;

    Standard_EXPORT Standard_Boolean CaptureScalarAppearanceForSavedSweepRebuild(
        const TDF_Label& label, OcctScalarAppearanceState& output) const noexcept;
    Standard_EXPORT Standard_Boolean CaptureScalarAppearanceForMeshCopy(
        const TDF_Label& label, OcctScalarAppearanceState& output) const noexcept;
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
    //! Read the stored v2 atlas without regenerating it; images remain readable.
    Standard_EXPORT Standard_Boolean CaptureMeshUVAtlasPreview(const TDF_Label& label, OcctMeshUVAtlasPreview& preview) const noexcept;
    //! Additive curved-layout presence, used to distinguish a planar v2 atlas migration.
    Standard_EXPORT Standard_Boolean HasCurvedUVLayoutForLabel(const TDF_Label& label) const noexcept;
    //! Same v2/prefix payload and ordinary MeshUVAtlas transaction as planar.
    //! options.curvedSource must resolve to the exact retained source B-rep.
    Standard_EXPORT Standard_Boolean PrepareCurvedUVAtlas(const TDF_Label& label,
        TopoDS_Shape& candidate, const OcctMeshUVAtlasOptions& options,
        OcctMeshUVAtlasPreview* preview = nullptr) const noexcept;
    //! Read digest-bound diagnostics and optional deterministic replay inputs.
    Standard_EXPORT Standard_Boolean ReadCurvedUVLayoutForLabel(const TDF_Label& label,
        shapeyard::uv::curved::CurvedUVLayoutRecord& layout,
        curveduv::PackSummary& coverage) const noexcept;
#if DEBUG
    Standard_EXPORT Standard_Boolean DebugCurvedUVLayoutForLabel(const TDF_Label& label,
        shapeyard::uv::curved::CurvedUVLayoutRecord& layout,
        curveduv::PackSummary& coverage) const noexcept;
#endif
    //! Bounded single-face mesh atlas; null curvedSource preserves the planar path.
    Standard_EXPORT Standard_Boolean PrepareTriangleUVAtlas(const TDF_Label& label, TopoDS_Shape& candidate, const OcctMeshUVAtlasOptions& options = {}, OcctMeshUVAtlasPreview* preview = nullptr) const noexcept;
    //! Private consistent-winding candidate; source ownership and material contracts remain strict.
    Standard_EXPORT OcctMeshWindingRepairResult PrepareMeshWindingRepair(const TDF_Label& label, TopoDS_Shape& candidate) const noexcept;
    Standard_EXPORT Standard_Boolean ValidateMeshWindingRepair(const TDF_Label& label, const TopoDS_Shape& candidate) const noexcept;
    //! Bounded flat-corner editing; preserves atlas/maps, rejects supplied frames.
    Standard_EXPORT Standard_Boolean CanEditMeshVertices(const TDF_Label& label) const noexcept;
    Standard_EXPORT Standard_Boolean PrepareMeshVertexMove(const TDF_Label& label,
        const std::vector<std::uint32_t>& vertices, const gp_Vec& worldDelta,
        OcctMeshVertexMutationCandidate& candidate) const noexcept;
    //! Compatibility path cannot discard live region-partition authority.
    Standard_EXPORT Standard_Boolean PrepareMeshVertexMove(const TDF_Label& label,
        const std::vector<std::uint32_t>& vertices, const gp_Vec& worldDelta,
        TopoDS_Shape& candidate) const noexcept;
    Standard_EXPORT Standard_Boolean ValidateMeshVertexMove(const TDF_Label& label,
        const std::vector<std::uint32_t>& vertices, const gp_Vec& worldDelta,
        const OcctMeshVertexMutationCandidate& candidate) const noexcept;
    Standard_EXPORT Standard_Boolean ValidateMeshVertexMove(const TDF_Label& label,
        const std::vector<std::uint32_t>& vertices, const gp_Vec& worldDelta,
        const TopoDS_Shape& candidate) const noexcept;
    //! Connected planar region topology construction; positive distance is world mm.
    Standard_EXPORT Standard_Boolean CaptureMeshRegionExtrudePreview(const TDF_Label& label,
        std::uint32_t seedTriangle, OcctMeshRegionExtrudePreview& preview) const noexcept;
    Standard_EXPORT Standard_Boolean PrepareMeshRegionExtrude(const TDF_Label& label,
        std::uint32_t seedTriangle, Standard_Real distanceMM,
        OcctMeshRegionMutationCandidate& candidate) const noexcept;
    //! Compatibility entry for the existing viewer. It succeeds only for a
    //! legacy no-partition source, so no authority can be silently discarded.
    Standard_EXPORT Standard_Boolean PrepareMeshRegionExtrude(const TDF_Label& label,
        std::uint32_t seedTriangle, Standard_Real distanceMM, TopoDS_Shape& candidate) const noexcept;
    Standard_EXPORT Standard_Boolean ValidateMeshRegionExtrude(const TDF_Label& label,
        std::uint32_t seedTriangle, Standard_Real distanceMM,
        const OcctMeshRegionMutationCandidate& candidate) const noexcept;
    Standard_EXPORT Standard_Boolean ValidateMeshRegionExtrude(const TDF_Label& label,
        std::uint32_t seedTriangle, Standard_Real distanceMM, const TopoDS_Shape& candidate) const noexcept;
    //! Convex connected planar inset. Resolution is partition-aware and the
    //! shape plus nonempty replacement partition are prepared as one value.
    Standard_EXPORT Standard_Boolean CaptureMeshRegionInsetPreview(const TDF_Label& label,
        std::uint32_t seedTriangle, OcctMeshRegionExtrudePreview& preview) const noexcept;
    Standard_EXPORT Standard_Boolean PrepareMeshRegionInset(const TDF_Label& label,
        std::uint32_t seedTriangle, Standard_Real distanceMM,
        OcctMeshRegionMutationCandidate& candidate) const noexcept;
    Standard_EXPORT Standard_Boolean ValidateMeshRegionInset(const TDF_Label& label,
        std::uint32_t seedTriangle, Standard_Real distanceMM,
        const OcctMeshRegionMutationCandidate& candidate) const noexcept;
    //! Marks exact Shapeyard-authored triangle-major UV storage in an open command.
    Standard_EXPORT Standard_Boolean MarkAuthoredMeshUVLayout(const TDF_Label& label) noexcept;
    Standard_EXPORT Standard_Boolean ValidateTriangleUVAtlas(const TDF_Label& label, const TopoDS_Shape& candidate, const OcctMeshUVAtlasOptions& options = {}) const noexcept;
    Standard_EXPORT Standard_Boolean MarkTriangleUVAtlas(const TDF_Label& label, const OcctMeshUVAtlasOptions& options = {}) noexcept;
#ifdef DEBUG
    Standard_EXPORT Standard_Boolean DebugProbeMeshUVRepackRecipeMismatch(const TDF_Label& label) const noexcept;
#endif
    //! False for a read-only XCAF component occurrence.
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
    Standard_Boolean SupportsNormalTextureGeometryForLabel(const TDF_Label& label) const noexcept;
    Standard_Boolean SupportsMaterialTextureEditingForLabel(
        const TDF_Label& label, OcctMaterialTextureSlot slot) const;
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
    //! Topology-changing legacy tools must refuse saved sweeps and lofts before opening work.
    Standard_EXPORT Standard_Boolean HasNoSavedSweepForTopology(const TDF_Label& label) const noexcept;

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
  friend class core3d::NativeDocumentSession;
  void CloseNativeSession() noexcept;
  bool myNativeSessionClosed = false;
  friend class core3d::OrdinaryEditController;
  // Only the ordinary owner may pair geometry and an already-current record.
  Standard_Boolean StageSavedSweepReplacement(const OcctObjectTransformState& previous,
      const TopoDS_Shape& candidate, const core3d::planar_sweep::Definition& definition,
      bool debugFailAfterShape = false) noexcept;
  Standard_Boolean StageSavedLoftReplacement(const OcctObjectTransformState& previous,
      const TopoDS_Shape& candidate,const core3d::rectangular_loft::Definition& definition,
      const core3d::rectangular_loft::StationDimensionEdit& edit,bool debugFailAfterShape=false) noexcept;
  bool SealSavedCutPlacementState(const std::shared_ptr<const OcctSavedCutSceneState>& previous,
      const gp_Trsf& expected,std::shared_ptr<const OcctSavedCutSceneState>& candidate) const noexcept;
  bool SealSavedCutSceneState(const std::shared_ptr<const OcctSavedCutSceneState>& previous,
      const TopoDS_Shape& result,const std::shared_ptr<const core3d::retained_solid::Payload>& payload,
      std::shared_ptr<const OcctSavedCutSceneState>& candidate) const noexcept;
  Standard_Boolean StageCylindricalCutReplacement(const OcctObjectTransformState& previous,
      const TopoDS_Shape& candidate,const std::shared_ptr<const core3d::retained_solid::Payload>& payload,
      const std::optional<core3d::retained_boolean::ProgramEdit>& edit,bool debugFailAfterShape=false)noexcept;
  // Only this same ordinary owner may update retained source, base and result.
  Standard_Boolean StageSavedCutSourceReplacement(const OcctObjectTransformState& previous,
      const core3d::saved_cut_source_edit::Patch& patch,
      const std::shared_ptr<const core3d::SavedCutSourceDetachedResult>& built,
      std::shared_ptr<const core3d::retained_solid::Payload>& staged,
      bool debugFailAfterShape=false) noexcept;
  bool SealSavedCutSourceState(const std::shared_ptr<const OcctSavedCutSceneState>& previous,
      const core3d::saved_cut_source_edit::Patch& patch,
      const std::shared_ptr<const core3d::SavedCutSourceDetachedResult>& built,
      const std::shared_ptr<const core3d::retained_solid::Payload>& payload,
      std::shared_ptr<const OcctSavedCutSceneState>& candidate) const noexcept;
  // Explicit whole-program source semantics, additive beside the legacy pair.
  // The actual patch is independently reapplied to the freshly captured
  // complete original recipe; old/new bytes and all four content commitments
  // must match the exact native-constructed detached result.
  Standard_Boolean StageSavedProgramSourceReplacement(const OcctObjectTransformState& previous,
      const core3d::saved_cut_source_edit::Patch& patch,
      const std::shared_ptr<const core3d::SavedProgramSourceDetachedResult>& built,
      std::shared_ptr<const core3d::retained_solid::Payload>& staged,
      bool debugFailAfterShape=false) noexcept;
  bool SealSavedProgramSourceState(const std::shared_ptr<const OcctSavedCutSceneState>& previous,
      const core3d::saved_cut_source_edit::Patch& patch,
      const std::shared_ptr<const core3d::SavedProgramSourceDetachedResult>& built,
      const std::shared_ptr<const core3d::retained_solid::Payload>& payload,
      std::shared_ptr<const OcctSavedCutSceneState>& candidate) const noexcept;
  // Pure native eligibility for an exclusively owned document, including the
  // private import worker. UI-facing admission retains its main-thread guard.
  Standard_Boolean HasNativeNormalTextureGeometry(const TDF_Label& label) const noexcept;
    std::shared_ptr<const OcctPBRScalarState> CapturePBRScalarState(const TDF_Label&) const noexcept;
    Standard_Boolean SaveObjectPBRMaterialsImpl(const std::vector<OcctPBRMaterialUpdate>&,
        const Handle(XCAFDoc_VisMaterial)& scalarMaterial);
    Standard_Boolean CanSaveObjectPBRMaterials(
        const std::vector<OcctPBRMaterialUpdate>& updates,
        std::vector<TDF_Label>* reclaimMaterialLabels,
        const Handle(XCAFDoc_VisMaterial)& scalarMaterial = Handle(XCAFDoc_VisMaterial)()) const;
    
#if DEBUG
  // Exact pointer created below and kept alive by myApp; no foreign downcast.
  core3d::debug::LiveObservedApplication* myObservedApplication = nullptr;
  std::shared_ptr<core3d::debug::LiveTransactionProbeState> myLiveProbe;
#endif
  std::shared_ptr<core3d::authority::NativeEditAuthority> myNativeAuthority;
  core3d::authority::NativeObservedApplication* myAuthorityApplication = nullptr;
  Handle(TDocStd_Application) myApp;
  Handle(TDocStd_Document) myOcafDoc;
  Standard_Size myMaximumSerializedTextureOccurrenceBytes;
  Standard_Size myMaximumDecodedTextureResourceBytes;
  Standard_Size myMaximumVisualMaterialDefinitions;
};

#endif // OcctDocument_h
