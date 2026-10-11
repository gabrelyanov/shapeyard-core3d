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
#include "RetainedRecipeSnapshot.hxx"
#include "RetainedEdgeTreatmentSnapshot.hxx"
#include "RetainedEdgeTreatmentR2Snapshot.hxx"
#include "RetainedFaceSelector.hxx"
#include "BoundedCurveAttribute.hxx"
#include "BoundedCurveBuild.hxx"
#include "GeneralLoftPersistence.hxx"
#include "SplineProfilePersistence.hxx"
#include "FeaturePatternChildAttribute.hxx"
#include "RetainedFinishingRecord.hxx"
#include "FaceImageDefinition.hxx"
#include "PaintedAtlasBakeDefinition.hxx"
namespace core3d::asset_atlas { struct Capture; struct Definition; struct Key; }
namespace core3d::painted_atlas_bake::owner { struct CurrentnessResult; }
namespace core3d::face_image::owner { enum class Outcome : std::uint8_t; struct Staging; }
namespace core3d::face_image::persistence::bindings { enum class ReadState : int; }

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
#include <TDF_Delta.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TNaming_Evolution.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_TShape.hxx>
#include <array>
#include <deque>
#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <optional>
#include <map>
#include <memory>
#include <cmath>
#include "SourceFaceProvenanceRecord.hxx"
#include "CurvedFaceUVUnwrap.hpp"
class XCAFDoc_VisMaterial;

class Message_ProgressRange;
namespace core3d { class OrdinaryEditController; class Core3DViewer; class BevelOperationController; class SavedCutSourceDetachedResult; class SavedProgramSourceDetachedResult; }
namespace core3d::native_opening { class CommandLease; }
namespace core3d::native_opening { class Context; }
namespace core3d::bounded_curve::owner { class OcafOwner; }
namespace core3d::pattern_owner { struct AllLabelSnapshot; }
namespace core3d::dependent_replay {
class Plan;
class Preparer;
class SourceMutation;
struct Limits;
enum class Mutation : std::uint8_t;
enum class Refusal : std::uint8_t;
}
namespace core3d::part_boolean::owner { class PartBooleanOwner; }
namespace core3d::retained_feature { class OcafOwnerService; }
namespace core3d::composite_recipe { struct Payload; }
namespace core3d::retained_program_suffix { struct Probe; }
namespace core3d::authored_parameter { class Capture; class Owner; }
namespace core3d::authored_boolean { struct Candidate; }

// D253 native treatment history companion — document-internal measured state
// for one staged retained-treatment transaction. One companion records the
// real owner-label-subtree TNaming naming state (label entries, attribute
// presence, versions, evolution, shape-pair and current-binding state), the
// exact source-recipe/SYET-carrier state and the original topology's
// transient Modified/Checked bookkeeping, once before staging and once after
// coherent staging. The ordinary edit owner binds it to the actual committed
// TDF delta only after proving closure with the sealed candidate; undo() and
// redo() engage it only for the matching live delta, run real OCAF history
// first, validate that the captured semantic state was restored, and only
// then restore the captured naming-version scalars (and, on Undo, the
// original topology's measured bookkeeping bits) before NotifyChanges. It
// carries only measured state needed for reversible restoration — never
// archive bytes, never app/AI authority — and is released with the native
// document session.
namespace core3d::treatment_history {
struct NamingState {
    TDF_Label label;
    std::string labelEntry;
    Standard_Integer version = 0;
    TNaming_Evolution evolution = TNaming_PRIMITIVE;
    std::vector<std::pair<TopoDS_Shape, TopoDS_Shape>> shapePairs;
    TopoDS_Shape currentBinding;
};
struct FlagState {
    Handle(TopoDS_TShape) tshape;
    bool modified = false;
    bool checked = false;
};
struct Companion {
    Handle(TDF_Data) data;
    TDF_Label ownerLabel;
    std::string ownerEntry;
    std::vector<NamingState> before;
    std::vector<NamingState> after;
    TopoDS_Shape ownerShapeBefore;
    TopoDS_Shape ownerShapeAfter;
    std::vector<std::uint8_t> sourceBytesBefore;
    std::vector<std::uint8_t> sourceBytesAfter;
    bool syetBefore = false;
    bool syetAfter = false;
    std::vector<std::uint8_t> syetBytesBefore;
    std::vector<std::uint8_t> syetBytesAfter;
    std::vector<FlagState> flagsBefore;
    Standard_Integer undoDepthBefore = -1;
    bool afterCaptured = false;
    Handle(TDF_Delta) delta;
};
} // namespace core3d::treatment_history

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

struct OcctRetainedFinishingSettings final {
    int unwrapPolicy = 0;
    int resolutionTexels = 1024;
    int gutterTexels = 2;
};

enum class OcctRetainedFinishingOutcome : int {
    Committed = 0, Refused, StaleSource, UnsupportedSurface,
    OwnerMismatch, Busy, Malformed, PersistenceFailure, Absent
};

enum class OcctRetainedFinishingCurrentness : int {
    Absent = 0, Current = 1, Stale = 2
};

struct OcctAssetAtlasSettings final {
    int resolutionTexels = 2048; // power of two in [256, 4096]; ONE shared texture
    int gutterTexels = 2;        // in [1, 8]
};

enum class OcctAssetAtlasOutcome : int {
    Committed = 0, Refused, StaleSource, MissingMember, ForeignMember,
    OverBudget, PaintedRebakeRequired, UnsupportedSurface, OwnerMismatch,
    Busy, Malformed, PersistenceFailure, Absent
};

enum class OcctAssetAtlasCurrentness : int { Absent = 0, Current = 1, Stale = 2 };

//! Explicit opt-in painted-atlas bake. Existing Build/Regenerate/Edit entry
//! points deliberately retain their PaintedRebakeRequired refusal.
enum class OcctPaintedAtlasBakeOutcome : int {
    Committed = 0, Refused, StaleSource, StaleBinding, MissingResource,
    ForeignResource, OverBudget, UnsupportedSurface, OwnerMismatch, Busy,
    Malformed, PersistenceFailure, Absent
};

enum class OcctPaintedAtlasBakeCurrentness : int {
    Absent = 0, Current = 1, Stale = 2
};

struct OcctPaintedAtlasCurrentnessLease;

//! Read-only E2b consumer handoff. The UV assignment and PNG envelopes are
//! regenerated/read from current SYEA/SYEB state and never installed on the
//! master shape or material. An absent or stale bake yields no derivative.
struct OcctPaintedAtlasDerivative final {
    core3d::painted_atlas_bake::Definition bake;
    core3d::asset_atlas::MemberUVAssignment assignment;
    std::vector<core3d::face_image::ResourceEnvelope> resources;
    OcctPaintedAtlasCurrentnessLease* currentnessLease = nullptr;

    Standard_EXPORT OcctPaintedAtlasDerivative() noexcept;
    OcctPaintedAtlasDerivative(const OcctPaintedAtlasDerivative&) = delete;
    OcctPaintedAtlasDerivative& operator=(
        const OcctPaintedAtlasDerivative&) = delete;
    Standard_EXPORT OcctPaintedAtlasDerivative(
        OcctPaintedAtlasDerivative&& other) noexcept;
    Standard_EXPORT OcctPaintedAtlasDerivative& operator=(
        OcctPaintedAtlasDerivative&& other) noexcept;
    Standard_EXPORT ~OcctPaintedAtlasDerivative();
    Standard_EXPORT void reset() noexcept;
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
    std::optional<core3d::retained_edge_treatment::Record> edgeTreatment;
    std::optional<core3d::retained_edge_treatment::r2::Record> edgeTreatmentR2;
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

//! Immutable exact authority for one editable free definition. Presence of
//! authored name, visibility, layers and scalar appearance is significant.
struct OcctExactLabelReceipt {
    Handle(TDF_Data) documentData;
    std::string documentIdentifier;
    OcctObjectVisibilityState visibility;
    OcctScalarAppearanceState appearance;
    Standard_EXPORT Standard_Boolean IsEqual(
        const OcctExactLabelReceipt& other) const noexcept;
};

//! An identity reservation minted only by the owning OcctDocument. UUIDs are
//! values to install, never caller assertions about existing OCAF state.
class OcctIssuedLabelIdentity final {
public:
    const std::string& EntityIdentifier() const noexcept { return entityIdentifier_; }
    const std::string& DefinitionIdentifier() const noexcept { return definitionIdentifier_; }
private:
    friend class OcctDocument;
    Handle(TDF_Data) documentData_;
    std::string documentIdentifier_;
    std::string entityIdentifier_;
    std::string definitionIdentifier_;
    std::uint64_t reservation_ = 0;
};

//! Detached geometry plus an exact source policy. The document re-reads the
//! source before copying name, visibility, transform and admitted appearance.
struct OcctPreparedLabelClone {
    OcctExactLabelReceipt source;
    TopoDS_Shape detachedShape;
    OcctGeometryRepresentation representation = OcctGeometryRepresentation::Invalid;
};

//! One preflightable all-label mutation. Later D2/D3 authorities may retain
//! this plan without receiving access to identity attributes or transactions.
struct OcctAllLabelPlan {
    struct Create { OcctIssuedLabelIdentity identity; OcctPreparedLabelClone clone; };
    struct Replace { OcctExactLabelReceipt expected; OcctPreparedLabelClone clone; };
    std::vector<Create> creates;
    std::vector<Replace> replacements;
    std::vector<OcctExactLabelReceipt> removals;
    std::vector<std::string> retainedRemovalLedger;
};

//! Exact C1 authority read from one free owner and its unique record label.
//! This is an in-process capture only: labels and data handles are never
//! serialized and cannot be reconstructed by a caller.
struct OcctBoundedCurveCapture {
    Handle(TDF_Data) documentData;
    std::string documentIdentifier;
    OcctExactLabelReceipt ownerReceipt;
    core3d::bounded_curve::Record record;
    core3d::bounded_curve::PersistedValue persisted;
    TopoDS_Wire wire;
    Standard_EXPORT Standard_Boolean IsEqual(
        const OcctBoundedCurveCapture& other) const noexcept;
};

//! Exact C3-N authority: one canonical SYGL record bound to one admitted solid.
//! The labels are in-process receipts and cannot be reconstructed from Swift.
struct OcctGeneralLoftCapture {
    Handle(TDF_Data) documentData;
    std::string documentIdentifier;
    OcctExactLabelReceipt ownerReceipt;
    core3d::general_loft::persistence::Record record;
    TopoDS_Solid solid;
    Standard_EXPORT Standard_Boolean IsEqual(
        const OcctGeneralLoftCapture& other) const noexcept;
};

//! Exact C4 owner + sidecar source record. The BRep is checked against an
//! independent rebuild before this receipt can cross into an editor opening.
struct OcctSplineProfileCapture {
    Handle(TDF_Data) documentData;
    std::string documentIdentifier;
    OcctExactLabelReceipt ownerReceipt;
    core3d::spline_profile::Record record;
    core3d::spline_profile::Definition definition;
    TopoDS_Solid solid;
    Standard_EXPORT Standard_Boolean IsEqual(
        const OcctSplineProfileCapture& other) const noexcept;
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
Standard_EXPORT Standard_Boolean Core3DValidateCompositeRecipeDocument(const Handle(TDocStd_Document)& document);
//! Retained finishing receipt admission: exact owner bindings, canonical
//! SYEF/1 bytes, aggregate budget, and an admitted retained carrier owner.
Standard_EXPORT Standard_Boolean Core3DValidateRetainedFinishingDocument(const Handle(TDocStd_Document)& document);
//! Asset-wide atlas admission: exact key/member bindings, canonical SYEA/1
//! bytes, aggregate budgets, and admitted retained carrier owners.
Standard_EXPORT Standard_Boolean Core3DValidateAssetAtlasDocument(const Handle(TDocStd_Document)& document);
Standard_EXPORT Standard_Boolean Core3DValidatePaintedAtlasBakeDocument(const Handle(TDocStd_Document)& document);
//! E3 face-image admission (278b portion 2): strict whole-document SYFR/1
//! resource-table read (canonical bytes, unique identities, the single
//! aggregate budget, closed neighborhoods), strict per-owner SYFI/1 binding
//! reads with the exact document/entity/definition identity binding and an
//! admitted retained carrier owner, and a closed-placement census admitting
//! the schema GUIDs on exactly the validated labels and nowhere else.
Standard_EXPORT Standard_Boolean Core3DValidateFaceImageDocument(const Handle(TDocStd_Document)& document);

#if DEBUG
namespace core3d::face_image {
namespace persistence::resources { class Core3D_FaceImageResource; }
//! E3 DEBUG evidence (278b portion 2; declaration surfaced in portion 4b so
//! the UI-layer observation seam can consume it): measured registration state
//! for the later portions' probes. Read-only; no edit authority and no
//! mutation route. Definitions live in OcctDocument.mm beside the persistence
//! admission they observe.
struct FaceImageProbe final {
    struct Observation final {
        Standard_Integer resources = 0;
        Standard_Integer boundOwners = 0;
        Standard_Size aggregateBytes = 0;
        bool complete = false;
    };
    // Friend seam: distinguishes a fully published payload from an empty
    // attribute shell without reparsing its bytes.
    static bool Published(
        const Handle(persistence::resources::Core3D_FaceImageResource)& attribute) noexcept;
    static Observation Observe(const Handle(TDocStd_Document)& document) noexcept;
};
} // namespace core3d::face_image
#endif

//! Read-only classification used by destructive native operation gates. A
//! malformed/unknown record is deliberately not collapsed to legacy absence.
//! `CurrentProfile` may only be consumed by the already-admitted cap-shell
//! route, which still proves its complete selectors, or by the bounded
//! plain-profile compatibility operation below, which proves its own capture,
//! independent rebuild correspondence, transactional retention and readback.
enum class OcctRetainedRecipeCoverage : Standard_Integer {
    Absent = 0,
    CurrentProfile = 1,
    PresentOutsideP4Coverage = 2,
    InvalidOrUnknown = 3,
};

//! Value-only authority captured for one plain-profile compatibility
//! operation input. No mutable OCAF attribute, open command, or edit lease
//! escapes the document; every field is re-proved against the live document
//! immediately before the operation's measured command.
struct OcctPlainProfileOperationCapture {
    TDF_Label label;
    TDF_Label recordLabel;
    std::string entityIdentifier;
    std::string definitionIdentifier;
    std::string featureIdentifier;
    std::vector<double> recipeValues;
    TopoDS_Shape boundRoot;
    gp_Trsf placement;
    Standard_Real metersPerUnit = 0.0;
};

// Immutable, document-bound H2/H4 values. Definitions stay private to the
// native document implementation so callers cannot manufacture archive proof.
struct OcctPlainProfileCutPreparation;
struct OcctPlainProfileCutReceipt;
#if DEBUG
//! Immutable, read-only H6 evidence. Indexed keys keep every original source,
//! edge, scalar recipe and commitment separately observable without exposing
//! OCAF labels or mutable native handles.
struct OcctPlainProfileCutDebugEvidence {
    std::map<std::string, std::vector<std::uint8_t>> bytes;
    std::map<std::string, double> numbers;
    std::map<std::string, bool> checks;
};
#endif

//! Scans every label, including hidden/unbound/orphan records and foreign arrays.
//! Byte arrays are limited to geometry-owned authored frames and exact canonical
//! retained-pattern records beneath the marker-bearing tag-71 document root.
//! The pattern exception is identity/placement validated and has its own 8 MiB
//! budget; it never contributes to or weakens native frame accounting. Callers
//! must combine this with their existing geometry/material admission and budgets.
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
//! Complete supplied-owner plus owned-normal usage, including hidden bindings,
//! exact retained-pattern placement, and orphan recipe rejection. Legacy
//! unowned/no-frame materials remain separate.
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
    bool hasMaterialBinding = false;
    std::vector<std::uint8_t> material,preserved,table;
    std::vector<std::uint8_t> materialAttributes;
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
    // D1a callers opt in to preserving a complete current sweep/loft recipe.
    // Other duplication paths remain fail-closed for these owners.
    bool preservesSweepLoftRecipe = false;
    // Mirror bakes geometry and must compose a new recipe construction frame;
    // local-transform Duplicate/arrays preserve the authored frame verbatim.
    bool requiresSweepLoftConstructionFrame = false;
    // D1-C typed handshake. Production callers leave this false until the
    // native promotion guard is retained; false keeps every composite owner
    // on the established fail-closed path.
    bool preservesAnalyticBooleanRecipe = false;
};

//! Register the app-owned BinOcaf/BinXCAF project formats with a narrow,
//! fail-closed attribute schema and bounded visual-material/string readers.
//! Retrieval additionally admits only LayerRef graph nodes with validated
//! layer/shape roles and the two exact byte-array roles documented above.
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
//! A1a detached geometry evidence only. It creates no document command and
//! cannot route the existing general Boolean UI worker through retained ownership.
Standard_EXPORT std::map<std::string,bool> Core3DDebugRetainedPartBooleanProbe();
Standard_EXPORT std::map<std::string,bool> Core3DDebugSavedBooleanRingProbe(Standard_Integer scenario);
Standard_EXPORT std::map<std::string,bool> Core3DDebugSavedBooleanFilletProbe(Standard_Integer scenario);
Standard_EXPORT void Core3DDebugSetRetainedFilletFailureCount(Standard_Integer count);
Standard_EXPORT std::map<std::string,bool> Core3DDebugSavedBooleanWedgeProbe(Standard_Integer scenario);
Standard_EXPORT std::map<std::string,bool> Core3DDebugNativeBooleanOwnerProbe(Standard_Integer scenario);
//! One-shot, non-reentrant capture of canonical pre-G0 writer bytes. The map
//! owns every payload; no temporary path is exposed as corpus authority.
Standard_EXPORT std::map<std::string, std::vector<std::uint8_t>>
Core3DDebugLegacyCorpusCapture();
//! P1 internal synthetic registry/OCAF evidence. No product feature is installed.
Standard_EXPORT std::map<std::string,bool> Core3DDebugRetainedFeatureRegistryProbe(Standard_Integer scenario);
//! A3/P2 DEBUG retained-program-suffix evidence; no edit authority and no
//! public append route.
Standard_EXPORT std::map<std::string,bool> Core3DDebugRetainedProgramSuffixProbe(Standard_Integer scenario);
//! DEBUG archive-rounding and adversarial trim-domain checks; no edit authority.
Standard_EXPORT std::map<std::string,bool> Core3DDebugSavedCutTrimDomainProbe();
Standard_EXPORT std::map<std::string,bool> Core3DDebugCircularHostProofProbe(Standard_Integer scenario);
//! E1 DEBUG native persistence evidence only; no finishing edit authority.
extern "C" Standard_EXPORT std::uint64_t Core3DDebugRetainedFinishingProbe(std::int32_t scenario) noexcept;
extern "C" Standard_EXPORT std::uint64_t Core3DDebugRetainedFinishingProducerProbe(
    std::int32_t scenario) noexcept;
//! E2a DEBUG asset-atlas evidence only; no atlas edit authority.
extern "C" Standard_EXPORT std::uint64_t Core3DDebugAssetAtlasProbe(
    std::int32_t scenario) noexcept;
//! E2b DEBUG preservation evidence. Readback returns independently checkable
//! emitted pixel/UV/sampler/tangent evidence from the most recent probe.
extern "C" Standard_EXPORT std::uint64_t Core3DDebugPaintedAtlasBakeProbe(
    std::int32_t scenario) noexcept;
extern "C" Standard_EXPORT void Core3DDebugDenyNextPaintedAtlasLease() noexcept;
extern "C" Standard_EXPORT std::uint64_t Core3DDebugPaintedAtlasBakeReadback(
    std::int32_t scenario, std::int32_t field) noexcept;
//! Seeds the probe's E4Decals fixture resources (slot 0 base-checker, 1
//! data-linear, 2 normal-linear); the test SHA-256-checks the exact bytes.
extern "C" Standard_EXPORT std::uint64_t Core3DDebugPaintedAtlasBakeSeedResource(
    std::int32_t slot, const void* bytes, std::int32_t length) noexcept;
//! Exports measured post-cold-reopen byte blobs from the last probe run:
//! field = unit*8 + kind, kind 0 baked baseColor PNG, 1 baked normal PNG, 2
//! original base-checker resource bytes. Length comes from the Readback.
extern "C" Standard_EXPORT const void* Core3DDebugPaintedAtlasBakeExport(
    std::int32_t scenario, std::int32_t field) noexcept;
void Core3DDebugDefineLegacyReceiptFormats(const Handle(TDocStd_Application)& application);
namespace core3d::persistence { struct AuthoredFrameReadBudget; }
namespace core3d::debug { struct LiveTransactionProbeState; class LiveObservedApplication; }
//! Isolated tests with a custom wire budget and no final frame geometry-owner
//! gate. Pattern placement and bounded LayerRef graph roles remain mandatory;
//! production registration always validates all owner associations after read.
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
  //! Never opens a command or repairs metadata. Call immediately before a
  //! destructive operation's NewCommand; callers must reject every result
  //! except the explicitly admitted route above.
  Standard_EXPORT OcctRetainedRecipeCoverage RetainedRecipeCoverageForLabel(
      const TDF_Label& label) const noexcept;
  //! Read-only eligibility and capture for the plain-profile compatibility
  //! operation. Succeeds only for a well-formed, current plain profile: an
  //! empty shell tail, no composite/retained-solid/sweep/loft/enclosure
  //! co-owner, no revolve section, exact entity/definition/feature identity,
  //! placement and units, and an independent recipe rebuild that provably
  //! corresponds to the captured root BRep. All record and topology work is
  //! bounded; unknown, corrupt, foreign, stale, aliased, or
  //! later-feature-bearing inputs refuse. Never opens a command.
  Standard_EXPORT Standard_Boolean CapturePlainProfileOperationSource(
      const TDF_Label& label,
      OcctPlainProfileOperationCapture& output) const noexcept;
  //! Recapture fence: repeats the full eligibility capture and requires every
  //! captured identity, scalar byte, bound root, placement and unit to match
  //! the live document. Call immediately before the operation's NewCommand.
  Standard_EXPORT Standard_Boolean PlainProfileOperationSourceCurrent(
      const OcctPlainProfileOperationCapture& capture) const noexcept;
  //! In-transaction readback after the root replacement: the new root must be
  //! stored, while the old profile record label, feature identifier, exact
  //! scalar bytes, original bound shape, entity/definition identifiers,
  //! placement and units are preserved and the retained record is now stale
  //! (never erased or rebound). Call only with the operation's command still
  //! open so a failed proof aborts without history.
  Standard_EXPORT Standard_Boolean VerifyPlainProfileOperationReplacement(
      const OcctPlainProfileOperationCapture& capture,
      const TopoDS_Shape& newRoot) const noexcept;
  //! Detached H2 proof. This performs no OCAF command, label allocation, or
  //! identity write. Ordered tools are actor order, never label order.
  Standard_EXPORT Standard_Boolean PreparePlainProfileCut(
      const OcctPlainProfileOperationCapture& subjectCapture,
      const std::vector<OcctPlainProfileOperationCapture>& orderedToolCaptures,
      TopoDS_Shape& canonicalCandidate,
      const gp_Trsf& resultOccurrence,
      std::shared_ptr<const OcctPlainProfileCutPreparation>& prepared) const noexcept;
  Standard_EXPORT Standard_Boolean PlainProfileCutSourcesCurrent(
      const std::shared_ptr<const OcctPlainProfileCutPreparation>& prepared) const noexcept;
  //! H4 paired write inside the caller's already-open measured command.
  Standard_EXPORT Standard_Boolean StagePlainProfileCutResult(
      const TDF_Label& resultLabel,
      const std::shared_ptr<const OcctPlainProfileCutPreparation>& prepared,
      std::shared_ptr<const OcctPlainProfileCutReceipt>& receipt) noexcept;
  Standard_EXPORT Standard_Boolean VerifyPlainProfileCutResult(
      const TDF_Label& resultLabel,
      const std::shared_ptr<const OcctPlainProfileCutReceipt>& receipt) const noexcept;
  Standard_EXPORT Standard_Boolean VerifyPlainProfileCutSourcesRestored(
      const std::shared_ptr<const OcctPlainProfileCutPreparation>& prepared) const noexcept;
#if DEBUG
  //! Read-only H6 observer. Missing, invalid, non-v4 or unreconstructible data
  //! returns false and clears output; it performs no repair or write-on-read.
  Standard_EXPORT Standard_Boolean DebugPlainProfileCutRetention(
      const std::string& entityIdentifier,
      OcctPlainProfileCutDebugEvidence& output) const noexcept;
  //! DEBUG-only, read-only proof that the committed source slots and result
  //! remain strict-digest fixed points through the selected native BinTools
  //! writer/reader. False clears output; no cached evidence is substituted.
  Standard_EXPORT Standard_Boolean DebugPlainProfileCutNativeRoundTrip(
      const std::string& entityIdentifier,
      Standard_Boolean direct,
      OcctPlainProfileCutDebugEvidence& output) const noexcept;
#endif
  // Benchmark assets need more than 40 steps; 1000 keeps memory bounded on
  // device while preserving a long editable native session.
  static constexpr Standard_Integer kNativeSessionUndoLimit = 1000;

  Standard_EXPORT OcctDocument();

  Standard_EXPORT virtual ~OcctDocument();

  Standard_EXPORT void InitDoc();
  // Internal native service. The pointer is document-owned and is replaced
  // only with the adopted TDocStd_Document. N2 may retain sessions, never this
  // pointer as authority after an adoption boundary.
  core3d::part_boolean::owner::PartBooleanOwner* PartBooleanOwnerService() noexcept;
  const core3d::part_boolean::owner::PartBooleanOwner* PartBooleanOwnerService() const noexcept;
  bool NativeBooleanOwnerBlocksOtherWork() const noexcept;
  core3d::retained_feature::OcafOwnerService* RetainedFeatureOwnerService() noexcept;
  const core3d::retained_feature::OcafOwnerService* RetainedFeatureOwnerService() const noexcept;
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
    Standard_EXPORT Standard_Boolean CaptureExactFreeLabel(
        const TDF_Label& label, OcctExactLabelReceipt& receipt) const noexcept;
    Standard_EXPORT Standard_Boolean ReadExactFreeLabel(
        const OcctExactLabelReceipt& expected,
        OcctExactLabelReceipt& receipt) const noexcept;
    //! UUID generation and collision checks occur here without an OCAF write.
    Standard_EXPORT Standard_Boolean ReserveExactLabelIdentities(
        Standard_Size count, const std::vector<std::string>& retainedRemovalLedger,
        std::vector<OcctIssuedLabelIdentity>& identities) noexcept;
    Standard_EXPORT Standard_Boolean StageCreateExactFreeLabel(
        core3d::native_opening::CommandLease& lease,
        const OcctIssuedLabelIdentity& identity,
        const OcctPreparedLabelClone& clone,
        OcctExactLabelReceipt& receipt) noexcept;
    Standard_EXPORT Standard_Boolean StageReplaceExactFreeLabel(
        core3d::native_opening::CommandLease& lease,
        const OcctExactLabelReceipt& expected,
        const OcctPreparedLabelClone& clone,
        OcctExactLabelReceipt& receipt,
        const std::function<Standard_Boolean(const TDF_Label&)>& shapeStager = {}) noexcept;
    Standard_EXPORT Standard_Boolean StageRemoveExactFreeLabel(
        core3d::native_opening::CommandLease& lease,
        const OcctExactLabelReceipt& expected) noexcept;
    Standard_EXPORT Standard_Boolean StageAllLabels(
        core3d::native_opening::CommandLease& lease,
        const OcctAllLabelPlan& plan,
        std::vector<OcctExactLabelReceipt>& receipts,
        const std::function<Standard_Boolean(const TDF_Label&)>& shapeStager = {}) noexcept;
    Standard_EXPORT Standard_Boolean ReadBackAllLabels(
        const OcctAllLabelPlan& plan,
        const std::vector<OcctExactLabelReceipt>& receipts) const noexcept;
    //! Capture tag-71 D2 authority from every actual free source/member label.
    //! The returned immutable snapshot includes exact presentation, oriented
    //! shape bytes, retained family recipes and measured bounded costs.
    Standard_EXPORT Standard_Boolean CapturePatternAllLabelSnapshot(
        const std::string& selectedEntityIdentifier,
        std::shared_ptr<const core3d::pattern_owner::AllLabelSnapshot>& snapshot) const noexcept;
    //! Recapture the same feature and require byte/identity/recipe/appearance/
    //! shape exactness. Valid both outside and inside the caller-owned command.
    Standard_EXPORT Standard_Boolean ReadPatternAllLabelSnapshot(
        const core3d::pattern_owner::AllLabelSnapshot& expected,
        std::shared_ptr<const core3d::pattern_owner::AllLabelSnapshot>& snapshot) const noexcept;
    //! Stage the tag-73 SYFP record and every host-owned SYFC receipt under the
    //! same caller-owned lease. Host/base/source labels are actual OCAF labels;
    //! exact paired readback is required before success is returned.
    Standard_EXPORT Standard_Boolean StageFeaturePatternPair(
        core3d::native_opening::CommandLease& lease,
        const TDF_Label& host,
        const TDF_Label& baselineRecipe,
        const TDF_Label& source,
        const core3d::feature_pattern::Definition& definition,
        const std::vector<core3d::feature_pattern_child::Receipt>& children,
        core3d::feature_pattern_child::PairedRecord& paired) noexcept;
    //! Read-only exact authority. Legacy tag-73-only state and any partial,
    //! mixed or foreign pair are deliberately not editor authority.
    Standard_EXPORT Standard_Boolean ReadFeaturePatternPair(
        const core3d::feature_pattern::UUID& feature,
        core3d::feature_pattern_child::PairedRecord& paired) const noexcept;
    //! Enumerate the complete transitive D2/D3/D4 closure and ask the real
    //! family collaborators to prepare every detached replay before a command.
    //! The returned plan is document-owned evidence: callers cannot construct,
    //! append, omit, or reorder its retained records.
    Standard_EXPORT core3d::dependent_replay::Refusal PrepareDependentReplayPlan(
        const OcctExactLabelReceipt& target,
        core3d::dependent_replay::Mutation mutation,
        const core3d::dependent_replay::Limits& limits,
        core3d::dependent_replay::Preparer& preparer,
        std::shared_ptr<const core3d::dependent_replay::Plan>& plan) noexcept;
    //! Additive overload (278b portion 4b) for production source edits: after
    //! the D2/D3/D4 closure is prepared it also captures the edited owner's
    //! committed E3 face-image attachment and proves the one-to-one
    //! reattachment of every bound face on the detached prospective post-edit
    //! stage BEFORE any mutation. A lost, split, merged, ambiguous or
    //! unsupported correspondence refuses the whole source command here; the
    //! prepared replay is staged and read inside the same one command by
    //! StageDependentReplayPlan/ReadDependentReplayPlan. An owner without
    //! face-image records behaves byte-identically to the base overload.
    Standard_EXPORT core3d::dependent_replay::Refusal PrepareDependentReplayPlan(
        const OcctExactLabelReceipt& target,
        core3d::dependent_replay::Mutation mutation,
        const core3d::dependent_replay::Limits& limits,
        core3d::dependent_replay::Preparer& preparer,
        const TopoDS_Shape& prospectiveStage,
        std::shared_ptr<const core3d::dependent_replay::Plan>& plan) noexcept;
    //! Stage the source mutation and every prepared descendant under the
    //! caller's one lease. This method does not commit or begin nested work.
    Standard_EXPORT core3d::dependent_replay::Refusal StageDependentReplayPlan(
        core3d::native_opening::CommandLease& lease,
        const core3d::dependent_replay::Plan& plan,
        core3d::dependent_replay::SourceMutation& sourceMutation) noexcept;
    //! Exact post-stage/post-commit proof. Every prepared family collaborator
    //! must reread its record, recipe, labels, and stable child/member IDs.
    Standard_EXPORT core3d::dependent_replay::Refusal ReadDependentReplayPlan(
        const core3d::dependent_replay::Plan& plan) const noexcept;
    //! Capture requires a viewer-issued context for this exact document. The
    //! reader below intentionally remains usable inside an owned command.
    Standard_EXPORT Standard_Boolean CaptureBoundedCurveExact(
        const std::string& entityIdentifier,
        const core3d::native_opening::Context& context,
        OcctBoundedCurveCapture& capture) const noexcept;
    Standard_EXPORT Standard_Boolean ReadBoundedCurveExact(
        const core3d::retained_recipe::OwnerKey& owner,
        OcctBoundedCurveCapture& capture) const noexcept;
    Standard_EXPORT Standard_Boolean StageBoundedCurveCreate(
        core3d::native_opening::CommandLease& lease,
        const OcctIssuedLabelIdentity& identity,
        const core3d::bounded_curve::PersistedValue& persisted,
        const core3d::bounded_curve::DetachedWire& detached,
        const std::string& requestedName,
        OcctBoundedCurveCapture& capture) noexcept;
    Standard_EXPORT Standard_Boolean StageBoundedCurveReplacement(
        core3d::native_opening::CommandLease& lease,
        const OcctBoundedCurveCapture& opening,
        const core3d::bounded_curve::PersistedValue& persisted,
        const core3d::bounded_curve::DetachedWire& detached,
        OcctBoundedCurveCapture& capture) noexcept;
    Standard_EXPORT Standard_Boolean CaptureGeneralLoftExact(
        const std::string& entityIdentifier,
        const core3d::native_opening::Context& context,
        OcctGeneralLoftCapture& capture) const noexcept;
    Standard_EXPORT Standard_Boolean ReadGeneralLoftExact(
        const core3d::retained_recipe::OwnerKey& owner,
        OcctGeneralLoftCapture& capture) const noexcept;
    Standard_EXPORT Standard_Boolean HasUnsupportedGeneralLoftDependent(
        const OcctGeneralLoftCapture& capture) const noexcept;
    Standard_EXPORT Standard_Boolean StageGeneralLoftCreate(
        core3d::native_opening::CommandLease& lease,
        const OcctIssuedLabelIdentity& identity,
        const core3d::general_loft::Definition& definition,
        const core3d::general_loft::AdmittedSolid& admitted,
        const std::string& requestedName,
        OcctGeneralLoftCapture& capture) noexcept;
    Standard_EXPORT Standard_Boolean StageGeneralLoftReplacement(
        core3d::native_opening::CommandLease& lease,
        const OcctGeneralLoftCapture& opening,
        const core3d::general_loft::Definition& definition,
        const core3d::general_loft::AdmittedSolid& admitted,
        OcctGeneralLoftCapture& capture) noexcept;
    Standard_EXPORT Standard_Boolean CaptureSplineProfileExact(
        const std::string& entityIdentifier,
        const core3d::native_opening::Context& context,
        OcctSplineProfileCapture& capture) const noexcept;
    Standard_EXPORT Standard_Boolean ReadSplineProfileExact(
        const core3d::retained_recipe::OwnerKey& owner,
        OcctSplineProfileCapture& capture) const noexcept;
    Standard_EXPORT Standard_Boolean StageSplineProfileCreate(
        core3d::native_opening::CommandLease& lease,
        const OcctIssuedLabelIdentity& identity,
        const core3d::spline_profile::Definition& definition,
        const core3d::spline_profile::DetachedSolid& detached,
        const std::string& requestedName,
        OcctSplineProfileCapture& capture) noexcept;
    Standard_EXPORT Standard_Boolean StageSplineProfileReplacement(
        core3d::native_opening::CommandLease& lease,
        const OcctSplineProfileCapture& opening,
        const core3d::spline_profile::Definition& definition,
        const core3d::spline_profile::DetachedSolid& detached,
        OcctSplineProfileCapture& capture) noexcept;
    //! Stage only the name in the caller's open command; verify exact readback
    //! and unchanged geometry/identity/transform. Never commit or notify here.
    Standard_EXPORT Standard_Boolean SetObjectNameForLabel(
        const TDF_Label& label, const TCollection_ExtendedString& name) noexcept;
    //! Read-only main-thread capture, valid during an owned open command or
    //! after closure. Does not create labels/attributes or repair metadata.
    //! On failure clears the output so a caller cannot reuse stale proof.
    Standard_EXPORT Standard_Boolean CaptureObjectTransformStateForLabel(
        const TDF_Label& label, OcctObjectTransformState& state) const noexcept;
    Standard_EXPORT std::shared_ptr<const core3d::retained_edge_treatment::Snapshot>
    CaptureRetainedEdgeTreatment(const TDF_Label& owner,
        const core3d::retained_recipe::RevisionFence& expected,
        core3d::retained_edge_treatment::Refusal&) const noexcept;
    Standard_EXPORT Standard_Boolean ValidateRetainedEdgeTreatments(
        core3d::retained_edge_treatment::Refusal&) const noexcept;
    Standard_EXPORT std::shared_ptr<const core3d::retained_edge_treatment::r2::Snapshot>
    CaptureRetainedEdgeTreatmentR2(const TDF_Label& owner,
        const core3d::retained_recipe::RevisionFence& expected,
        core3d::retained_edge_treatment::Refusal&) const noexcept;
    Standard_EXPORT std::shared_ptr<const core3d::retained_edge_treatment::r2::MigrationCapture>
    CaptureRetainedBooleanMigrationR2(const TDF_Label& owner,
        const core3d::retained_recipe::RevisionFence& expected,
        core3d::retained_edge_treatment::Refusal&) const noexcept;
    Standard_EXPORT std::shared_ptr<const core3d::retained_edge_treatment::r2::EnrollmentCapture>
    CaptureRetainedBooleanEnrollmentR2(const TDF_Label& owner,
        const core3d::retained_recipe::RevisionFence& expected,
        core3d::retained_edge_treatment::Refusal&) const noexcept;
    Standard_EXPORT Standard_Boolean ValidateRetainedEdgeTreatmentsR2(
        core3d::retained_edge_treatment::Refusal&) const noexcept;
    //! N0 authored-parameter authority: capture the complete supported source
    //! plus suffix without opening a command. Unsupported carriers refuse;
    //! no record is projected away into a weaker DTO.
    Standard_EXPORT std::shared_ptr<const core3d::authored_parameter::Capture>
    CaptureAuthoredParameterAuthority(
        const TDF_Label& owner,
        const core3d::native_opening::Context& context) const noexcept;
    //! Exact pre-transaction reread of document/data, owner, source/suffix,
    //! units, history depths and the treatment-companion census.
    Standard_EXPORT Standard_Boolean ReadAuthoredParameterAuthority(
        const core3d::authored_parameter::Capture& capture) const noexcept;
    //! The sole future transaction entrance for registered adapters. N0 never
    //! calls it because no adapter can produce a candidate yet.
    Standard_EXPORT Standard_Boolean BeginAuthoredParameterTransaction(
        const core3d::authored_parameter::Capture& capture,
        core3d::native_opening::Context& context,
        std::uint32_t viewportWidth, std::uint32_t viewportHeight,
        std::shared_ptr<core3d::native_opening::CommandLease>& lease) noexcept;
    //! True while a measured treatment-history companion has not been bound
    //! to its own committed delta or discharged by verified prior settlement.
    Standard_EXPORT Standard_Boolean HasUnresolvedTreatmentHistoryCompanion() const noexcept;
    //! Prepare one independent retained Profile treatment copy without
    //! mutating the document. The returned root is produced by replay from the
    //! returned payload's independently detached untreated base. Absence of a
    //! source carrier succeeds with empty outputs.
    Standard_EXPORT Standard_Boolean PrepareRetainedEdgeTreatmentIndependentCopy(
        const TDF_Label& sourceOwner,
        const std::string& destinationProfileIdentifier,
        const core3d::profile::Parameters& destinationParameters,
        core3d::retained_edge_treatment::ReplayBudget& budget,
        std::shared_ptr<const core3d::retained_edge_treatment::Payload>& prepared,
        TopoDS_Shape& preparedRoot,
        const TopoDS_Shape& destinationBase = TopoDS_Shape(),
        const std::optional<gp_Trsf>& sourceToDestination = std::nullopt) noexcept;
    //! Bind a prepared copy to the real native destination identities, replay
    //! the final program independently, verify the staged replay root, stage
    //! the carrier, and read both source and destination back exactly.
    Standard_EXPORT Standard_Boolean StageRetainedEdgeTreatmentIndependentCopy(
        const TDF_Label& sourceOwner,
        const core3d::profile::Record& sourceProfile,
        const TDF_Label& destinationOwner,
        const core3d::profile::Record& destinationProfile,
        core3d::retained_edge_treatment::ReplayBudget& budget,
        const std::shared_ptr<const core3d::retained_edge_treatment::Payload>& prepared) noexcept;
    Standard_EXPORT core3d::retained_face_selector::Resolution ResolveRetainedFaceSelector(
        const core3d::retained_edge_treatment::Snapshot& snapshot,
        const core3d::retained_face_selector::SelectorIntent& intent,
        const core3d::retained_recipe::RevisionFence& expected) const noexcept;
    //! Operation-internal continuation. The caller supplies capture debt and
    //! receives the successor counter; the public overload above remains the
    //! fresh top-level inspection entry and still includes capture debt.
    Standard_EXPORT core3d::retained_face_selector::Resolution ResolveRetainedFaceSelector(
        const core3d::retained_edge_treatment::Snapshot& snapshot,
        const core3d::retained_face_selector::SelectorIntent& intent,
        const core3d::retained_recipe::RevisionFence& expected,
        core3d::retained_edge_treatment::ReplayBudget& budget) const noexcept;
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
    Standard_EXPORT std::optional<OcctPBRScalarDebugEvidence> DebugPBRScalarEvidence(
        const TDF_Label&, bool allowUnboundMaterial = false) const noexcept;
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
    Standard_EXPORT Standard_Boolean CaptureRetainedFinishingSource(
        const core3d::retained_recipe::OwnerKey& owner,
        core3d::retained_finishing::SourceRevision& output) noexcept;
    Standard_EXPORT OcctRetainedFinishingOutcome ProduceRetainedFinishing(
        const core3d::retained_recipe::OwnerKey& owner,
        const OcctRetainedFinishingSettings& settings) noexcept;
    Standard_EXPORT OcctRetainedFinishingOutcome RegenerateRetainedFinishing(
        const core3d::retained_recipe::OwnerKey& owner) noexcept;
    Standard_EXPORT OcctRetainedFinishingCurrentness RetainedFinishingCurrentness(
        const core3d::retained_recipe::OwnerKey& owner,
        core3d::retained_finishing::Definition* record = nullptr) const noexcept;
    Standard_EXPORT Standard_Boolean CaptureAssetAtlasMembers(
        const std::vector<core3d::retained_recipe::OwnerKey>& members,
        core3d::asset_atlas::Capture& output) noexcept;
    Standard_EXPORT OcctAssetAtlasOutcome BuildAssetAtlas(
        const core3d::asset_atlas::Key& atlas,
        const std::vector<core3d::retained_recipe::OwnerKey>& members,
        const OcctAssetAtlasSettings& settings) noexcept;
    Standard_EXPORT OcctAssetAtlasOutcome RegenerateAssetAtlas(
        const core3d::asset_atlas::Key& atlas) noexcept;
    Standard_EXPORT OcctAssetAtlasOutcome EditAssetAtlasMembership(
        const core3d::asset_atlas::Key& atlas,
        const std::vector<core3d::retained_recipe::OwnerKey>& remove,
        const std::vector<core3d::retained_recipe::OwnerKey>& add) noexcept;
    Standard_EXPORT OcctAssetAtlasCurrentness AssetAtlasCurrentness(
        const core3d::asset_atlas::Key& atlas,
        core3d::asset_atlas::Definition* record = nullptr) const noexcept;
    Standard_EXPORT OcctPaintedAtlasBakeOutcome BakePaintedAtlas(
        const core3d::asset_atlas::Key& atlas) noexcept;
    Standard_EXPORT OcctPaintedAtlasBakeCurrentness PaintedAtlasBakeCurrentness(
        const core3d::asset_atlas::Key& atlas,
        core3d::painted_atlas_bake::owner::CurrentnessResult* record = nullptr)
        const noexcept;
    Standard_EXPORT Standard_Boolean PaintedAtlasDerivativeForOwner(
        const core3d::retained_recipe::OwnerKey& owner,
        OcctPaintedAtlasDerivative& derivative) const noexcept;
    //! E3 face-image document surface (278b portion 2). The mutating entry
    //! points never open their own mutation route: like the finishing/atlas
    //! owners above, the caller holds the already-open OCAF command under the
    //! existing document mutation owner (Busy otherwise), so every committed
    //! adoption, binding commit and removal is one ordinary undoable command
    //! and a command abort/undo restores the prior state exactly. Refusal
    //! leaves no delta. Cold reopen is admitted only through the registered
    //! bounded driver chain and Core3DValidateFaceImageDocument.
    Standard_EXPORT core3d::face_image::owner::Outcome AdoptFaceImageResource(
        const core3d::face_image::ResourceEnvelope& candidate) noexcept;
    Standard_EXPORT core3d::face_image::owner::Outcome RemoveFaceImageResource(
        const core3d::face_image::UUID& resource) noexcept;
    Standard_EXPORT Standard_Boolean ReadFaceImageResource(
        const core3d::face_image::UUID& resource,
        core3d::face_image::ResourceEnvelope& output) const noexcept;
    //! Role-independent manifest (resource identity + original-content
    //! digest) for the caller-captured Observed resource fences.
    Standard_EXPORT Standard_Boolean FaceImageResourceManifest(
        std::vector<core3d::face_image::ResourceFence>& output) const noexcept;
    //! Binding transaction hooks into the portion-1 owner store. Prepare
    //! resolves the owner label and fences only; Commit re-proves the
    //! identical fence and installs the canonical SYFI/1 record inside the
    //! caller's open command; Cancel discards the staging whole.
    Standard_EXPORT core3d::face_image::owner::Outcome PrepareFaceImageBindings(
        core3d::face_image::owner::Staging& staging,
        const core3d::face_image::Definition& candidate,
        const core3d::face_image::Observed& observed) noexcept;
    Standard_EXPORT core3d::face_image::owner::Outcome CommitFaceImageBindings(
        core3d::face_image::owner::Staging& staging,
        const core3d::face_image::Observed& observed) noexcept;
    Standard_EXPORT void CancelFaceImageBindings(
        core3d::face_image::owner::Staging& staging) noexcept;
    //! Whole-record removal (last-binding removal) inside the caller's open
    //! command; zero-delta when no record exists.
    Standard_EXPORT core3d::face_image::owner::Outcome RemoveFaceImageBindings(
        const core3d::face_image::OwnerKey& owner) noexcept;
    //! Strict fail-closed binding-record read: Absent/Malformed/Present, never
    //! a degraded verdict. Malformed also covers an unresolved owner.
    Standard_EXPORT core3d::face_image::persistence::bindings::ReadState ReadFaceImageBindings(
        const core3d::face_image::OwnerKey& owner,
        core3d::face_image::Definition& output,
        std::vector<std::uint8_t>* bytes = nullptr) const noexcept;
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
        Handle(AIS_Shape) aisShape,
        const std::function<Standard_Boolean(const TDF_Label&)>& shapeStager = {});
    Standard_Boolean HasUnroutedRetainedDependent(
        const TDF_Label& label) const noexcept;
    
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
  friend class core3d::bounded_curve::owner::OcafOwner;
  friend class core3d::NativeDocumentSession;
  friend class core3d::part_boolean::owner::PartBooleanOwner;
  // A0: only the shared native owner may stage the sealed analytic
  // composite/R2 pair under its document-issued capture and command lease.
  friend class core3d::authored_parameter::Owner;
  friend class core3d::retained_feature::OcafOwnerService;
  // A3/P2 DEBUG probe: save/reopen byte evidence for the retained program
  // suffix. No production staging authority is granted by this friendship.
  friend struct core3d::retained_program_suffix::Probe;
  bool StagePartBooleanPayload(
      const TDF_Label&, const std::shared_ptr<const core3d::composite_recipe::Payload>&) noexcept;
  Standard_Boolean StageAuthoredBooleanPair(
      const core3d::authored_parameter::Capture&,
      const core3d::authored_boolean::Candidate&) noexcept;
  void CloseNativeSession() noexcept;
  bool myNativeSessionClosed = false;
  friend class core3d::OrdinaryEditController;
  friend class core3d::Core3DViewer;
  // The bevel/fillet apply path stages the captured retained edit through the
  // same private gate as the ordinary edit owner; no other caller is admitted.
  friend class core3d::BevelOperationController;
  // D253 native treatment history companion lifecycle, owned by the document
  // and driven only by the ordinary edit owner (finalize on proven candidate
  // closure, discharge on verified prior-state settlement) and by the native
  // history boundary (engage only the matching live TDF delta). No app/AI
  // authority API.
  Standard_Boolean FinalizeTreatmentHistoryCompanion() noexcept;
  Standard_Boolean SettleTreatmentHistoryCompanionOnPrior() noexcept;
  void ReleaseTreatmentHistoryCompanions() noexcept;
  void PruneTreatmentHistoryCompanions() noexcept;
  Standard_Boolean ApplyTreatmentHistoryAfterHistoryChange(
      const Handle(TDF_Delta)& engaged, Standard_Boolean undoDirection) noexcept;
  Standard_Boolean StageRetainedEdgeTreatment(
      const core3d::retained_edge_treatment::Snapshot& original,
      const core3d::retained_edge_treatment::Edit& edit,
      const core3d::retained_edge_treatment::DetachedResult& built,
      core3d::retained_edge_treatment::Record& readback,
      core3d::retained_edge_treatment::Refusal&) noexcept;
  Standard_Boolean StageRetainedEdgeTreatmentR2(
      const core3d::retained_edge_treatment::r2::Snapshot&,
      const core3d::retained_edge_treatment::r2::DetachedResult&,
      core3d::retained_edge_treatment::r2::Record&,
      core3d::retained_edge_treatment::Refusal&,
      bool debugFailAfterShape = false,
      bool debugFailAtReadback = false) noexcept;
  Standard_Boolean StageRetainedBooleanMigrationR2(
      const core3d::retained_edge_treatment::r2::MigrationCapture&,
      const core3d::retained_edge_treatment::r2::MigrationM3&,
      const core3d::retained_edge_treatment::r2::DetachedResult&,
      core3d::retained_edge_treatment::r2::Record&,
      core3d::retained_edge_treatment::Refusal&) noexcept;
  Standard_Boolean StageRetainedBooleanEnrollmentR2(
      const core3d::retained_edge_treatment::r2::EnrollmentCapture&,
      const core3d::retained_edge_treatment::r2::DetachedResult&,
      core3d::retained_edge_treatment::r2::Record&,
      core3d::retained_edge_treatment::Refusal&) noexcept;
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
  std::unique_ptr<core3d::part_boolean::owner::PartBooleanOwner> myPartBooleanOwner;
  std::unique_ptr<core3d::retained_feature::OcafOwnerService> myRetainedFeatureOwner;
  // D253: the unfinalized measured companion of an in-flight treatment
  // transaction, and the bounded set of companions bound to live TDF deltas.
  std::unique_ptr<core3d::treatment_history::Companion> myPendingTreatmentCompanion;
  std::deque<std::unique_ptr<core3d::treatment_history::Companion>> myTreatmentHistoryCompanions;
  Standard_Size myMaximumSerializedTextureOccurrenceBytes;
  Standard_Size myMaximumDecodedTextureResourceBytes;
  Standard_Size myMaximumVisualMaterialDefinitions;
  std::uint64_t myNextExactIdentityReservation = 0;
  std::unordered_map<std::uint64_t, std::pair<std::string, std::string>>
      myExactIdentityReservations;
  std::unordered_set<std::string> myExactIdentityIssuanceLedger;
  // Nonempty only during synchronous StageDependentReplayPlan. Direct legacy
  // ReplaceShape/RemoveShape calls remain fail-closed for retained dependents.
  std::unordered_set<std::string> myDependentReplayAuthorization;
};

#endif // OcctDocument_h
