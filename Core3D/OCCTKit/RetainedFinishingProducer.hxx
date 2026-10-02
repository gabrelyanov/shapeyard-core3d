#pragma once

// E1b retained-finishing producer. The product value is derived only from the
// current retained owner and its live BRep tessellation. Serialized callers
// supply bounded settings, never source authority or proof digests.
#include "CurrentTessellationMeshCopy.hxx"
#include "CurvedFaceUVUnwrap.hpp"
#include "RetainedFinishingAttribute.hxx"
#include "SourceFaceProvenanceRecord.hxx"

#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <atomic>
#include <cmath>
#include <cstring>
#include <locale>
#include <sstream>

namespace core3d::retained_finishing::producer {

struct Settings final {
    UnwrapPolicy requested = UnwrapPolicy::Planar;
    int resolutionTexels = 1024;
    int gutterTexels = 2;
};

struct Capture final {
    SourceRevision source;
    Digest resourceManifest{};
    std::vector<MaterialResource> resources;
};

enum class Status : std::uint8_t {
    Captured, Produced, Refused, StaleSource, UnsupportedSurface,
    OwnerMismatch, Busy, Malformed, PersistenceFailure
};

namespace detail {

inline bool HashBytes(const void* bytes, std::size_t size, Digest& output) noexcept {
    try {
        if (!bytes || size == 0 || size > kMaximumBytes) return false;
        const auto* first = static_cast<const std::uint8_t*>(bytes);
        return HashFinishingBytes(first, size, output)
            && Nonzero(output);
    } catch (...) { output = {}; return false; }
}

inline bool HashText(const std::string& text, Digest& output) noexcept {
    return HashBytes(text.data(), text.size(), output);
}

inline std::uint64_t Prefix64(const Digest& value) noexcept {
    std::uint64_t result = 0;
    for (std::size_t index = 0; index < 8; ++index)
        result |= std::uint64_t(value[index]) << (8 * index);
    return result == 0 ? 1 : result;
}

inline UUID UUIDFromDigest(const Digest& value, std::uint8_t discriminator) noexcept {
    UUID result{};
    std::copy_n(value.begin(), result.size(), result.begin());
    result[6] = std::uint8_t((result[6] & 0x0f) | 0x40);
    result[8] = std::uint8_t((result[8] & 0x3f) | 0x80);
    result[15] ^= discriminator;
    if (!retained_recipe::Nonzero(result)) result[15] = discriminator ? discriminator : 1;
    return result;
}

inline bool Resolve(const Handle(TDocStd_Document)& document, const OwnerKey& owner,
                    TDF_Label& label, retained_solid::Record& record) noexcept {
    label = {}; record = {};
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || !retained_recipe::Valid(owner)) return false;
        UUID documentID{};
        if (!retained_solid::ReadUUID(document->Main(),
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
            || documentID != owner.document) return false;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return false;
        TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
        int matches = 0;
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            UUID entity{}, definition{}; const TDF_Label candidate = roots.Value(index);
            if (!XCAFDoc_ShapeTool::IsSimpleShape(candidate)
                || !retained_solid::ReadUUID(candidate,
                    Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entity)
                || !retained_solid::ReadUUID(candidate,
                    Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definition)
                || entity != owner.entity || definition != owner.definition) continue;
            if (++matches != 1) return false;
            label = candidate;
        }
        return matches == 1 && retained_solid::Read(document, label, record)
            && record.value && !record.current.IsNull();
    } catch (...) { label = {}; record = {}; return false; }
}

inline bool Source(const Handle(TDocStd_Document)& document, const OwnerKey& owner,
                   SourceRevision& output, TDF_Label* ownerLabel = nullptr,
                   retained_solid::Record* retained = nullptr) noexcept {
    output = {};
    try {
        TDF_Label label; retained_solid::Record record;
        if (!Resolve(document, owner, label, record)) return false;
        std::ostringstream geometry(std::ios::out | std::ios::binary);
        BRepTools::Write(record.current, geometry);
        const std::string shapeBytes = geometry.str();
        if (shapeBytes.empty() || !HashText(shapeBytes, output.geometry)
            || !retained_solid::Hash(record.value->bytes, output.recipe)) return false;

        std::ostringstream placement; placement.imbue(std::locale::classic());
        const gp_Trsf transform = record.current.Location().Transformation();
        placement.precision(17);
        for (int row = 1; row <= 3; ++row)
            for (int column = 1; column <= 4; ++column)
                placement << transform.Value(row, column) << ';';
        if (!HashText(placement.str(), output.placement)) return false;

        std::vector<std::uint8_t> materialBytes(owner.entity.begin(), owner.entity.end());
        materialBytes.insert(materialBytes.end(), owner.definition.begin(), owner.definition.end());
        materialBytes.insert(materialBytes.end(), output.recipe.begin(), output.recipe.end());
        if (!retained_solid::Hash(materialBytes, output.material)) return false;
        materialBytes.insert(materialBytes.end(), output.geometry.begin(), output.geometry.end());
        if (!retained_solid::Hash(materialBytes, output.groups)) return false;

        Digest generation{};
        if (!HashBytes(owner.document.data(), owner.document.size(), generation)) return false;
        std::vector<std::uint8_t> revisionBytes;
        for (const Digest* digest : {&output.geometry, &output.recipe, &output.placement,
                                     &output.material, &output.groups})
            revisionBytes.insert(revisionBytes.end(), digest->begin(), digest->end());
        Digest revision{};
        if (!retained_solid::Hash(revisionBytes, revision)) return false;
        output.documentGeneration = Prefix64(generation);
        output.modelRevision = Prefix64(revision);
        if (ownerLabel) *ownerLabel = label;
        if (retained) *retained = std::move(record);
        return true;
    } catch (...) { output = {}; return false; }
}

inline bool Triangles(const meshcopy::CurrentTessellationCopy& copy,
                      std::vector<shapeyard::uv::Triangle>& triangles,
                      std::vector<shapeyard::uv::curved::FaceInput>& faces) noexcept {
    triangles.clear(); faces.clear();
    try {
        TopLoc_Location location;
        const auto mesh = BRep_Tool::Triangulation(copy.face, location);
        if (mesh.IsNull() || !location.IsIdentity() || mesh->NbTriangles() != copy.triangles)
            return false;
        triangles.resize(std::size_t(mesh->NbTriangles()));
        for (int triangle = 1; triangle <= mesh->NbTriangles(); ++triangle) {
            int nodes[3]; mesh->Triangle(triangle).Get(nodes[0], nodes[1], nodes[2]);
            for (int corner = 0; corner < 3; ++corner) {
                if (nodes[corner] < 1 || nodes[corner] > mesh->NbNodes()) return false;
                const gp_Pnt point = mesh->Node(nodes[corner]);
                triangles[std::size_t(triangle - 1)].points[corner] =
                    {point.X(), point.Y(), point.Z()};
            }
        }
        if (!provenance::FinishingFaceInputs(copy.faces, faces)) return false;
        for (auto& face : faces) face.toleranceMM = 1.0e-7;
        return !triangles.empty() && !faces.empty();
    } catch (...) { triangles.clear(); faces.clear(); return false; }
}

inline bool DigestScalars(const double* values, std::size_t count, Digest& output) noexcept {
    return values && count > 0 && HashBytes(values, count * sizeof(double), output);
}

} // namespace detail

inline bool CaptureSource(const Handle(TDocStd_Document)& document, const OwnerKey& owner,
                          Capture& output) noexcept {
    output = {};
    try {
        if (!detail::Source(document, owner, output.source)) return false;
        MaterialResource resource;
        resource.identity = detail::UUIDFromDigest(output.source.material, 0x31);
        resource.content = output.source.material;
        output.resources.push_back(resource);
        std::vector<std::uint8_t> bytes(resource.identity.begin(), resource.identity.end());
        bytes.insert(bytes.end(), resource.content.begin(), resource.content.end());
        return retained_solid::Hash(bytes, output.resourceManifest)
            && Nonzero(output.resourceManifest);
    } catch (...) { output = {}; return false; }
}

inline Status BuildDerivative(const Handle(TDocStd_Document)& document,
                              const TDF_Label& ownerLabel, const Capture& capture,
                              const Settings& settings, Definition& candidate,
                              std::string& diagnosis) noexcept {
    candidate = {}; diagnosis.clear();
    try {
        if (document.IsNull() || ownerLabel.IsNull()
            || ownerLabel.Data() != document->GetData()
            || settings.resolutionTexels < 256 || settings.resolutionTexels > 4096
            || (settings.resolutionTexels & (settings.resolutionTexels - 1)) != 0
            || settings.gutterTexels < 1 || settings.gutterTexels > 8)
            return Status::Malformed;
        if (settings.requested == UnwrapPolicy::Conical
            || settings.requested == UnwrapPolicy::Spherical)
            return Status::UnsupportedSurface;

        UUID documentID{}, entity{}, definition{};
        if (!retained_solid::ReadUUID(document->Main(),
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
            || !retained_solid::ReadUUID(ownerLabel,
                Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entity)
            || !retained_solid::ReadUUID(ownerLabel,
                Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definition))
            return Status::OwnerMismatch;
        candidate.owner = {documentID, entity, definition};

        SourceRevision observed; retained_solid::Record retained;
        if (!detail::Source(document, candidate.owner, observed, nullptr, &retained)
            || !(observed == capture.source)) return Status::StaleSource;
        std::atomic_bool cancelled{false}; meshcopy::CurrentTessellationCopy copy;
        if (meshcopy::PrepareCurrentTessellationCopy(retained.current, copy, cancelled)
            != meshcopy::PreparationResult::Ready) return Status::UnsupportedSurface;
        std::vector<shapeyard::uv::Triangle> triangles;
        std::vector<shapeyard::uv::curved::FaceInput> faces;
        if (!detail::Triangles(copy, triangles, faces)) return Status::Malformed;
        const shapeyard::uv::Settings atlasSettings{
            settings.resolutionTexels, settings.gutterTexels};
        const auto atlas = shapeyard::uv::curved::unwrap(triangles, faces, atlasSettings);
        if (!atlas.ok || !shapeyard::uv::curved::validate(
                triangles, faces, atlasSettings, atlas.atlas.corners)) {
            diagnosis = atlas.reason.empty() ? "independent atlas validation failed" : atlas.reason;
            return Status::Refused;
        }

        candidate.source = capture.source;
        candidate.materials = capture.resources;
        candidate.finishing = detail::UUIDFromDigest(capture.resourceManifest, 0x68);
        candidate.unwrap = settings.requested;
        candidate.quality = Quality::VerifiedChart;
        bool fallback = false;
        for (const auto& face : atlas.layout.faces)
            if (face.kernel == curveduv::KernelTag::Fallback) fallback = true;
        if (fallback) {
            candidate.unwrap = UnwrapPolicy::DiagnosedFallback;
            candidate.quality = Quality::DiagnosedFallback;
            diagnosis = "One or more source faces used the explicit planar fallback.";
        }

        std::vector<std::uint8_t> sourceBytes;
        for (const Digest* digest : {&capture.source.geometry, &capture.source.recipe,
                                     &capture.source.placement, &capture.source.material,
                                     &capture.source.groups})
            sourceBytes.insert(sourceBytes.end(), digest->begin(), digest->end());
        if (!retained_solid::Hash(sourceBytes, candidate.tessellation.sourceRevision))
            return Status::Malformed;
        const std::array<std::int32_t, 3> encodedSettings = {
            std::int32_t(settings.requested), settings.resolutionTexels, settings.gutterTexels};
        if (!detail::HashBytes(encodedSettings.data(), sizeof encodedSettings,
                candidate.tessellation.settings)) return Status::Malformed;
        TopLoc_Location location; const auto mesh = BRep_Tool::Triangulation(copy.face, location);
        std::ostringstream meshDescription;
        meshDescription << copy.sourceFaces << ':' << copy.triangles << ':'
                        << copy.recordedDeflection;
        if (!detail::HashText(meshDescription.str(), candidate.tessellation.artifact))
            return Status::Malformed;
        const std::string build = "e1b-live-finishing-producer-v1";
        if (!detail::HashText(build, candidate.tessellation.build)) return Status::Malformed;

        std::vector<std::uint8_t> chartBytes;
        for (const auto& corners : atlas.atlas.corners)
            for (const auto& uv : corners)
                for (double scalar : uv) {
                    const auto* first = reinterpret_cast<const std::uint8_t*>(&scalar);
                    chartBytes.insert(chartBytes.end(), first, first + sizeof scalar);
                }
        if (!HashFinishingBytes(chartBytes, candidate.chartProof)) return Status::Malformed;
        const UUID material = candidate.materials.front().identity;
        for (std::size_t index = 0; index < faces.size(); ++index) {
            FaceAssignment assignment;
            Digest identityDigest{};
            const std::array<std::uint64_t, 2> identity = {
                std::uint64_t(index + 1), std::uint64_t(faces[index].triangleCount)};
            if (!detail::HashBytes(identity.data(), sizeof identity, identityDigest))
                return Status::Malformed;
            assignment.selector = detail::UUIDFromDigest(identityDigest, 0x52);
            assignment.material = material;
            assignment.selectorProof = identityDigest;
            assignment.expectedCardinality = 1;
            candidate.assignments.push_back(assignment);
        }
        for (std::size_t triangle = 0; triangle < triangles.size(); ++triangle)
            for (int corner = 0; corner < 3; ++corner) {
                FinalCorner value; const auto& point = triangles[triangle].points[corner];
                const auto& uv = atlas.atlas.corners[triangle][corner];
                if (!detail::DigestScalars(point.data(), point.size(), value.position)
                    || !detail::DigestScalars(uv.data(), uv.size(), value.uv))
                    return Status::Malformed;
                const std::array<double, 3> normal = {0.0, 0.0, 1.0};
                if (!detail::DigestScalars(normal.data(), normal.size(), value.normal))
                    return Status::Malformed;
                value.material = material; candidate.finalCorners.push_back(value);
            }
        Refusal refusal;
        return Valid(candidate, refusal) ? Status::Produced : Status::Malformed;
    } catch (...) { candidate = {}; diagnosis.clear(); return Status::Malformed; }
}

} // namespace core3d::retained_finishing::producer
