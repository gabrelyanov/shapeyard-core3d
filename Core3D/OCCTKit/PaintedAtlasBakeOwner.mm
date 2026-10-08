#import <Foundation/Foundation.h>
#import <CoreGraphics/CoreGraphics.h>
#import <ImageIO/ImageIO.h>

#include "PaintedAtlasBake.hxx"
#include "OcctDocument.h"

#include <Image_Texture.hxx>
#include <NCollection_Buffer.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>

namespace core3d::painted_atlas_bake::kernel {
// Bounded ImageIO decode to exact straight RGBA8 working pixels (the same
// decoder the capture path uses; declared in PaintedAtlasBake.hxx so the
// DEBUG probe measures persisted PNG bytes through the identical route).
bool DecodeImage(const std::vector<std::uint8_t>& bytes, Image& output) noexcept {
    output = {};
    @autoreleasepool {
        try {
            if (bytes.empty() || bytes.size() > face_image::kMaximumEncodedImageBytes)
                return false;
            CFDataRef data = CFDataCreate(kCFAllocatorDefault, bytes.data(), bytes.size());
            if (!data) return false;
            CGImageSourceRef source = CGImageSourceCreateWithData(data, nullptr);
            CFRelease(data);
            if (!source) return false;
            CGImageRef image = CGImageSourceCreateImageAtIndex(source, 0, nullptr);
            CFRelease(source);
            if (!image) return false;
            const std::size_t width = CGImageGetWidth(image), height = CGImageGetHeight(image);
            if (width == 0 || height == 0
                || width > std::size_t(face_image::kMaximumImageDimension)
                || height > std::size_t(face_image::kMaximumImageDimension)
                || width > face_image::kMaximumImagePixels / height) {
                CGImageRelease(image); return false;
            }
            std::vector<std::uint8_t> rgba(width * height * 4, 0);
            CGColorSpaceRef color = CGImageGetColorSpace(image);
            if (color) CFRetain(color);
            else color = CGColorSpaceCreateDeviceRGB();
            CGContextRef context = color ? CGBitmapContextCreate(rgba.data(), width, height, 8,
                width * 4, color, kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big) : nullptr;
            if (color) CGColorSpaceRelease(color);
            if (!context) { CGImageRelease(image); return false; }
            CGContextSetBlendMode(context, kCGBlendModeCopy);
            CGContextDrawImage(context, CGRectMake(0, 0, width, height), image);
            CGContextRelease(context); CGImageRelease(image);
            // Face-image normalized inputs are straight alpha. Convert the
            // CoreGraphics premultiplied readback without touching hidden RGB
            // on fully transparent texels (those contribute exactly zero).
            for (std::size_t offset = 0; offset < rgba.size(); offset += 4) {
                const unsigned alpha = rgba[offset + 3];
                if (alpha == 0) {
                    rgba[offset] = rgba[offset + 1] = rgba[offset + 2] = 0;
                } else if (alpha < 255) {
                    for (unsigned channel = 0; channel < 3; ++channel)
                        rgba[offset + channel] = std::uint8_t(std::min(255U,
                            (unsigned(rgba[offset + channel]) * 255U + alpha / 2U) / alpha));
                }
            }
            output.width = std::uint32_t(width); output.height = std::uint32_t(height);
            output.rgba = std::move(rgba); return true;
        } catch (...) { output = {}; return false; }
    }
}
} // namespace core3d::painted_atlas_bake::kernel

namespace core3d::painted_atlas_bake::owner {
namespace {
namespace fi = core3d::face_image;
namespace aa = core3d::asset_atlas;

UUID UUIDFromDigest(const Digest& digest) noexcept {
    UUID value{};
    std::copy_n(digest.begin(), value.size(), value.begin());
    if (!retained_recipe::Nonzero(value)) value.back() = 1;
    return value;
}

enum class TextureCapture : std::uint8_t { Captured, Foreign, Missing };

// XCAF texture slots must carry document-owned bytes: a file-backed texture
// references foreign bytes and is refused ForeignResource, never silently
// read from disk.
TextureCapture SyntheticEnvelope(const Handle(Image_Texture)& texture, const OwnerKey& owner,
                       Role role, const Digest& selector, const Digest& bindingProof,
                       CapturedSource& output) noexcept {
    output = {};
    try {
        if (texture.IsNull()) return TextureCapture::Missing;
        if (!texture->FilePath().IsEmpty()) return TextureCapture::Foreign;
        const Handle(NCollection_Buffer)& buffer = texture->DataBuffer();
        if (buffer.IsNull() || !buffer->Data() || buffer->Size() == 0
            || buffer->Size() > fi::kMaximumEncodedImageBytes) return TextureCapture::Missing;
        std::vector<std::uint8_t> bytes(buffer->Data(), buffer->Data() + buffer->Size());
        Digest content{};
        if (!fi::HashFaceImageBytes(bytes, content)) return TextureCapture::Missing;
        kernel::Image decoded;
        if (!kernel::DecodeImage(bytes, decoded)) return TextureCapture::Missing;
        output.envelope.resource = UUIDFromDigest(content);
        output.envelope.originalContent = content;
        output.envelope.workingContent = content;
        output.envelope.originalFormat = fi::ImageEncoding::PNG;
        output.envelope.workingFormat = fi::ImageEncoding::PNG;
        output.envelope.alpha = fi::AlphaInterpretation::Straight;
        output.envelope.originalWidthTexels = decoded.width;
        output.envelope.originalHeightTexels = decoded.height;
        output.envelope.workingWidthTexels = decoded.width;
        output.envelope.workingHeightTexels = decoded.height;
        output.envelope.provenance = content;
        output.envelope.originalBytes = bytes; output.envelope.workingBytes = std::move(bytes);
        output.fence.owner = owner; output.fence.resource = output.envelope.resource;
        output.fence.originalContent = content; output.fence.workingContent = content;
        output.fence.selectorProof = selector; output.fence.bindingProof = bindingProof;
        output.fence.role = role;
        output.fence.colorSpace = (role == Role::BaseColor || role == Role::Emissive)
            ? ColorSpace::SRGB : ColorSpace::Linear;
        std::vector<std::uint8_t> id(owner.entity.begin(), owner.entity.end());
        id.insert(id.end(), owner.definition.begin(), owner.definition.end());
        id.push_back(std::uint8_t(role)); id.insert(id.end(), content.begin(), content.end());
        Digest binding{};
        if (!retained_solid::Hash(id, binding)) return TextureCapture::Missing;
        output.fence.binding = UUIDFromDigest(binding);
        return Valid(output.fence) ? TextureCapture::Captured : TextureCapture::Missing;
    } catch (...) { output = {}; return TextureCapture::Missing; }
}

// Captures every SYFI/1 binding of every member plus the XCAF texture slots
// of painted members. Outcome-typed: an empty capture means there is nothing
// to bake (Refused); a dangling or undecodable resource is MissingResource;
// a file-backed XCAF texture is ForeignResource; a malformed SYFI record or
// resource table is Malformed.
Outcome CaptureSources(const Handle(TDocStd_Document)& document,
                       const aa::Capture& members,
                       std::vector<CapturedSource>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull()) return Outcome::Malformed;
        const auto materialTool = XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
        for (const auto& member : members.members) {
            fi::Definition definition;
            std::vector<std::uint8_t> bindingBytes;
            const auto state = fi::persistence::bindings::Read(
                document, member.ownerLabel, definition, &bindingBytes);
            if (state == fi::persistence::bindings::ReadState::Malformed)
                return Outcome::Malformed;
            if (state == fi::persistence::bindings::ReadState::Present) {
                for (const auto& binding : definition.bindings) {
                    fi::persistence::resources::Record resource;
                    if (!fi::persistence::resources::Read(document, binding.resource, resource))
                        return Outcome::Malformed;
                    if (!resource.value) return Outcome::MissingResource;
                    CapturedSource source;
                    source.fence.owner = definition.owner;
                    source.fence.binding = binding.binding;
                    source.fence.resource = binding.resource;
                    source.fence.selectorProof = binding.selectorProof;
                    source.fence.bindingProof = definition.bindingProof;
                    source.fence.originalContent = resource.value->envelope.originalContent;
                    source.fence.workingContent = resource.value->envelope.workingContent;
                    source.fence.role = binding.role;
                    source.fence.colorSpace = binding.colorSpace;
                    source.fence.transform = binding.transform;
                    source.envelope = resource.value->envelope;
                    if (!Valid(source.fence)) return Outcome::Malformed;
                    output.push_back(std::move(source));
                }
            }
            if (!member.painted) continue;
            TDF_Label materialLabel;
            XCAFDoc_VisMaterialTool::GetShapeMaterial(member.ownerLabel, materialLabel);
            Handle(XCAFDoc_VisMaterial) material;
            if (!materialLabel.IsNull() && !materialTool.IsNull())
                material = materialTool->GetMaterial(materialLabel);
            // A member marked painted with no readable material has no
            // document-owned image content at all.
            // A member marked painted through SYFI/1 alone has no XCAF
            // material; its bindings above are its painted content.
            if (material.IsNull()) continue;
            const auto add = [&](const Handle(Image_Texture)& texture, Role role) -> Outcome {
                if (texture.IsNull()) return Outcome::Prepared;
                for (const auto& existing : output)
                    if (existing.fence.owner == member.slot.owner && existing.fence.role == role)
                        return Outcome::Prepared;
                CapturedSource source;
                const auto captured = SyntheticEnvelope(texture, member.slot.owner, role,
                    member.slot.source.geometry, member.slot.source.recipe, source);
                if (captured == TextureCapture::Foreign) return Outcome::ForeignResource;
                if (captured != TextureCapture::Captured) return Outcome::MissingResource;
                output.push_back(std::move(source)); return Outcome::Prepared;
            };
            if (material->HasCommonMaterial()) {
                const auto added = add(material->CommonMaterial().DiffuseTexture,
                                       Role::BaseColor);
                if (added != Outcome::Prepared) return added;
            }
            if (material->HasPbrMaterial()) {
                const auto& pbr = material->PbrMaterial();
                if (add(pbr.BaseColorTexture, Role::BaseColor) != Outcome::Prepared
                    || add(pbr.EmissiveTexture, Role::Emissive) != Outcome::Prepared
                    || add(pbr.MetallicRoughnessTexture, Role::MetallicRoughness) != Outcome::Prepared
                    || add(pbr.OcclusionTexture, Role::Occlusion) != Outcome::Prepared
                    || add(pbr.NormalTexture, Role::Normal) != Outcome::Prepared) {
                    // Recompute the first failing outcome (the calls are
                    // idempotent: already-captured roles short-circuit).
                    Outcome failure = add(pbr.BaseColorTexture, Role::BaseColor);
                    if (failure == Outcome::Prepared) failure = add(pbr.EmissiveTexture, Role::Emissive);
                    if (failure == Outcome::Prepared) failure = add(pbr.MetallicRoughnessTexture, Role::MetallicRoughness);
                    if (failure == Outcome::Prepared) failure = add(pbr.OcclusionTexture, Role::Occlusion);
                    if (failure == Outcome::Prepared) failure = add(pbr.NormalTexture, Role::Normal);
                    return failure;
                }
            }
        }
        if (output.empty()) return Outcome::Refused;
        std::sort(output.begin(), output.end(), [&](const CapturedSource& a,
                                                    const CapturedSource& b) {
            auto memberIndex = [&](const OwnerKey& key) {
                for (std::size_t index = 0; index < members.members.size(); ++index)
                    if (members.members[index].slot.owner == key) return index;
                return members.members.size();
            };
            const auto ai = memberIndex(a.fence.owner), bi = memberIndex(b.fence.owner);
            if (ai != bi) return ai < bi;
            if (a.fence.role != b.fence.role)
                return std::uint8_t(a.fence.role) < std::uint8_t(b.fence.role);
            return a.fence.binding < b.fence.binding;
        });
        return Outcome::Prepared;
    } catch (...) { output.clear(); return Outcome::Malformed; }
}

bool SameCapture(const std::vector<CapturedSource>& first,
                 const std::vector<CapturedSource>& second) noexcept {
    if (first.size() != second.size()) return false;
    for (std::size_t index = 0; index < first.size(); ++index)
        if (!(first[index].fence == second[index].fence)
            || first[index].envelope.originalBytes != second[index].envelope.originalBytes
            || first[index].envelope.workingBytes != second[index].envelope.workingBytes)
            return false;
    return true;
}

Outcome MapBuild(aa::build::Status status) noexcept {
    switch (status) {
        case aa::build::Status::StaleSource: return Outcome::StaleSource;
        case aa::build::Status::MissingMember: return Outcome::StaleSource;
        case aa::build::Status::ForeignMember: return Outcome::OwnerMismatch;
        case aa::build::Status::OverBudget: return Outcome::OverBudget;
        case aa::build::Status::UnsupportedSurface: return Outcome::UnsupportedSurface;
        case aa::build::Status::OwnerMismatch: return Outcome::OwnerMismatch;
        case aa::build::Status::Busy: return Outcome::Busy;
        case aa::build::Status::PersistenceFailure: return Outcome::PersistenceFailure;
        case aa::build::Status::Malformed: return Outcome::Malformed;
        case aa::build::Status::Built: return Outcome::Prepared;
        case aa::build::Status::Captured:
        case aa::build::Status::PaintedRebakeRequired:
        case aa::build::Status::Refused: return Outcome::Refused;
    }
}

// A committed SYEA member that no longer classifies as Captured makes the
// bake input stale or inadmissible before any detached work runs.
Outcome ClassifyMembers(const Handle(TDocStd_Document)& document,
                        const aa::Definition& atlas) noexcept {
    try {
        for (const auto& member : atlas.members) {
            switch (aa::build::ClassifyMember(document, member.owner)) {
                case aa::build::Status::Captured: break;
                case aa::build::Status::MissingMember: return Outcome::StaleSource;
                case aa::build::Status::ForeignMember: return Outcome::OwnerMismatch;
                case aa::build::Status::UnsupportedSurface: return Outcome::UnsupportedSurface;
                default: return Outcome::Malformed;
            }
        }
        return Outcome::Prepared;
    } catch (...) { return Outcome::Malformed; }
}

// The strict SYEA whole-document reader intentionally collapses a broken
// member binding into malformed.  Before returning that generic outcome from
// the product entry, inspect only the immutable atlas payload and live XCAF
// identities to preserve the frozen OwnerMismatch distinction.  This helper
// never admits or decodes a record; it can only refine a refusal.
bool HasAtlasOwnerMismatch(const Handle(TDocStd_Document)& document,
                           const aa::Key& key) noexcept {
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const TDF_Label root = document->Main().FindChild(
            aa::persistence::RootTag, Standard_False);
        if (root.IsNull()) return false;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return false;
        TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
        for (TDF_ChildIterator child(root, Standard_False); child.More(); child.Next()) {
            Handle(aa::persistence::Attribute) attribute;
            if (!child.Value().FindAttribute(aa::persistence::AttributeID(), attribute)
                || attribute.IsNull() || !attribute->value()
                || !(attribute->value()->definition.key == key)) continue;
            for (const auto& member : attribute->value()->definition.members) {
                int matches = 0;
                for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
                    UUID entity{}, definition{};
                    const TDF_Label candidate = roots.Value(index);
                    if (XCAFDoc_ShapeTool::IsSimpleShape(candidate)
                        && retained_solid::ReadUUID(candidate,
                            Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entity)
                        && retained_solid::ReadUUID(candidate,
                            Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definition)
                        && entity == member.owner.entity
                        && definition == member.owner.definition)
                        ++matches;
                }
                if (matches != 1) return true;
            }
            return false;
        }
        return false;
    } catch (...) { return false; }
}

// The baked derivative resources are document-owned SYFR/1 envelopes. Their
// identities are deterministic role-bound digests recorded by SYEB/1; their
// content fields remain exact PNG digests. Re-adopting identical bytes is a
// no-op; the same identity with different bytes is ForeignResource.
Outcome AdoptBakedResources(const Handle(TDocStd_Document)& document,
                            const std::vector<kernel::Output>& outputs) noexcept {
    try {
        for (const auto& output : outputs) {
            fi::ResourceEnvelope envelope;
            envelope.resource = UUIDFromDigest(output.descriptor.identity);
            envelope.originalContent = output.descriptor.content;
            envelope.workingContent = output.descriptor.content;
            envelope.originalFormat = fi::ImageEncoding::PNG;
            envelope.workingFormat = fi::ImageEncoding::PNG;
            envelope.alpha = fi::AlphaInterpretation::Straight;
            envelope.originalWidthTexels = output.descriptor.widthTexels;
            envelope.originalHeightTexels = output.descriptor.heightTexels;
            envelope.workingWidthTexels = output.descriptor.widthTexels;
            envelope.workingHeightTexels = output.descriptor.heightTexels;
            std::string provenance = "shapeyard:e2b-painted-bake.v1:";
            provenance.push_back(char('a' + std::uint8_t(output.descriptor.role)));
            std::vector<std::uint8_t> provenanceBytes(provenance.begin(), provenance.end());
            provenanceBytes.push_back(std::uint8_t(output.descriptor.role));
            provenanceBytes.insert(provenanceBytes.end(),
                output.descriptor.content.begin(), output.descriptor.content.end());
            if (!fi::HashFaceImageBytes(provenanceBytes, envelope.provenance))
                return Outcome::Malformed;
            envelope.originalBytes = output.png;
            envelope.workingBytes = output.png;
            fi::persistence::resources::Record existing;
            if (!fi::persistence::resources::Read(document, envelope.resource, existing))
                return Outcome::Malformed;
            if (existing.value) {
                if (!(existing.value->envelope.workingContent == output.descriptor.content))
                    return Outcome::ForeignResource;
                continue;
            }
            const auto adopted = fi::owner::AdoptResource(document, envelope);
            if (adopted == fi::owner::Outcome::Committed) continue;
            if (adopted == fi::owner::Outcome::ForeignResource) return Outcome::ForeignResource;
            return Outcome::PersistenceFailure;
        }
        return Outcome::Committed;
    } catch (...) { return Outcome::PersistenceFailure; }
}
} // namespace

Outcome Prepare(Staging& staging, const Handle(TDocStd_Document)& document,
                const aa::Key& key) noexcept {
    staging = {};
    try {
        if (document.IsNull() || !document->HasOpenCommand()) return Outcome::Busy;
        aa::persistence::Record prior;
        if (!aa::persistence::Read(document, key, prior))
            return HasAtlasOwnerMismatch(document, key)
                ? Outcome::OwnerMismatch : Outcome::Malformed;
        if (!prior.value) return Outcome::Absent;
        staging.priorAtlasBytes = prior.value->bytes;
        const Outcome classified = ClassifyMembers(document, prior.value->definition);
        if (classified != Outcome::Prepared) return classified;
        std::vector<aa::Member> currentMembers;
        if (!aa::build::ObserveMembers(document, prior.value->definition.members,
                                       currentMembers)
            || !aa::Current(prior.value->definition, currentMembers))
            return Outcome::StaleSource;
        std::vector<OwnerKey> owners;
        for (const auto& member : prior.value->definition.members) owners.push_back(member.owner);
        aa::Capture capture;
        if (!aa::build::CaptureMembers(document, owners, capture)) return Outcome::StaleSource;
        const Outcome captured = CaptureSources(document, capture, staging.capturedSources);
        if (captured != Outcome::Prepared) return captured;
        // A malformed persisted SYEB/1 record refuses before any bake work.
        persistence::Record existing;
        const auto state = persistence::Read(document, key, existing);
        if (state == persistence::ReadState::Malformed) return Outcome::Malformed;
        if (state == persistence::ReadState::Present)
            staging.priorBakeResources = existing.definition.resources;
        // Aggregate decoded budget preflight: distinct baked roles at the
        // fenced atlas resolution must fit the frozen 128 MiB cap.
        std::vector<Role> roles;
        for (const auto& source : staging.capturedSources)
            if (std::find(roles.begin(), roles.end(), source.fence.role) == roles.end())
                roles.push_back(source.fence.role);
        const std::uint64_t resolution = std::uint64_t(prior.value->definition.resolutionTexels);
        if (resolution > std::uint64_t(kMaximumBakedDimension)
            || resolution * resolution * 4 > kMaximumDecodedImageBytes
            || resolution * resolution * 4 * roles.size() > kMaximumDecodedImageBytes)
            return Outcome::OverBudget;
        aa::build::LayoutEvidence layout;
        std::string diagnosis;
        const aa::build::Settings settings{prior.value->definition.resolutionTexels,
                                           prior.value->definition.gutterTexels};
        const auto built = aa::build::BuildAtlas(document, key, capture, settings,
            staging.atlas, staging.assignments, diagnosis, &layout,
            aa::build::PaintedAdmission::Preserve);
        if (built != aa::build::Status::Built) return MapBuild(built);
        if (!aa::build::ObserveMembers(document, staging.atlas.members,
                                       staging.observedMembers)) return Outcome::StaleSource;
        std::vector<kernel::Source> sources;
        for (const auto& capturedSource : staging.capturedSources) {
            kernel::Image image;
            if (!kernel::DecodeImage(capturedSource.envelope.workingBytes, image))
                return Outcome::MissingResource;
            sources.push_back({capturedSource.fence, std::move(image)});
        }
        if (!kernel::Bake(staging.atlas, layout, sources, staging.outputs,
                          staging.evidence)) return Outcome::Refused;
        staging.bake.key = key; staging.bake.layoutProof = staging.atlas.layoutProof;
        for (const auto& source : staging.capturedSources)
            staging.bake.bindings.push_back(source.fence);
        for (const auto& output : staging.outputs)
            staging.bake.resources.push_back(output.descriptor);
        if (!BindBakeProof(staging.bake)) return Outcome::Malformed;
        std::vector<std::uint8_t> atlasBytes;
        if (!aa::Encode(staging.atlas, atlasBytes)) return Outcome::Malformed;
        staging.unchanged = state == persistence::ReadState::Present
            && existing.definition == staging.bake
            && prior.value->bytes == atlasBytes;
        return Outcome::Prepared;
    } catch (...) { staging = {}; return Outcome::Malformed; }
}

Outcome Commit(Staging& staging, const Handle(TDocStd_Document)& document) noexcept {
    try {
        if (document.IsNull() || !document->HasOpenCommand()) {
            staging = {}; return Outcome::Busy;
        }
        std::vector<aa::Member> members;
        if (!aa::build::ObserveMembers(document, staging.atlas.members, members)
            || members != staging.observedMembers) { staging = {}; return Outcome::StaleSource; }
        aa::Capture capture;
        std::vector<OwnerKey> owners;
        for (const auto& member : staging.atlas.members) owners.push_back(member.owner);
        std::vector<CapturedSource> sources;
        if (!aa::build::CaptureMembers(document, owners, capture)) {
            staging = {}; return Outcome::StaleSource;
        }
        const Outcome recaptured = CaptureSources(document, capture, sources);
        if (recaptured == Outcome::MissingResource || recaptured == Outcome::ForeignResource
            || recaptured == Outcome::Malformed) {
            staging = {}; return recaptured;
        }
        if (recaptured != Outcome::Prepared || !SameCapture(staging.capturedSources, sources)) {
            staging = {}; return Outcome::StaleBinding;
        }
        if (staging.unchanged) { staging = {}; return Outcome::Committed; }
        // The bake never silently rewrites the atlas: the rebuilt SYEA/1 is
        // recommitted only when its canonical bytes actually changed.
        std::vector<std::uint8_t> atlasBytes;
        if (!aa::Encode(staging.atlas, atlasBytes)) { staging = {}; return Outcome::Malformed; }
        if (atlasBytes != staging.priorAtlasBytes) {
            aa::owner::Staging atlasStaging;
            auto atlasOutcome = aa::owner::Prepare(atlasStaging, document, staging.atlas,
                                                   staging.assignments, members);
            if (atlasOutcome != aa::owner::Outcome::Prepared) {
                staging = {}; return atlasOutcome == aa::owner::Outcome::StaleSource
                    ? Outcome::StaleSource : Outcome::PersistenceFailure;
            }
            atlasOutcome = aa::owner::Commit(atlasStaging, document, members);
            if (atlasOutcome != aa::owner::Outcome::Committed) {
                aa::owner::Cancel(atlasStaging); staging = {};
                return Outcome::PersistenceFailure;
            }
        }
        const Outcome adopted = AdoptBakedResources(document, staging.outputs);
        if (adopted != Outcome::Committed) { staging = {}; return adopted; }
        // A re-bake replaces, rather than accumulates, this atlas's derivative
        // resources. Remove superseded identities in the same OCAF command,
        // but retain any identity still named by another SYEB record or a
        // committed SYFI binding.
        std::vector<persistence::Record> bakeRecords;
        if (!persistence::ReadAll(document, bakeRecords)) {
            staging = {}; return Outcome::PersistenceFailure;
        }
        for (const auto& priorResource : staging.priorBakeResources) {
            bool current = false;
            for (const auto& resource : staging.bake.resources)
                if (resource.identity == priorResource.identity) { current = true; break; }
            if (current) continue;
            bool shared = false;
            for (const auto& record : bakeRecords) {
                if (record.definition.key == staging.bake.key) continue;
                for (const auto& resource : record.definition.resources)
                    if (resource.identity == priorResource.identity) {
                        shared = true; break;
                    }
                if (shared) break;
            }
            if (shared) continue;
            const auto removed = fi::owner::RemoveResource(
                document, UUIDFromDigest(priorResource.identity));
            if (removed != fi::owner::Outcome::Committed
                && removed != fi::owner::Outcome::Refused) {
                staging = {}; return Outcome::PersistenceFailure;
            }
        }
        if (!persistence::StageCommitted(document, staging.bake)) {
            staging = {}; return Outcome::PersistenceFailure;
        }
        staging = {}; return Outcome::Committed;
    } catch (...) { staging = {}; return Outcome::PersistenceFailure; }
}

void Cancel(Staging& staging) noexcept { staging = {}; }

Outcome CaptureForExport(ExportCapture& capture,
                         const Handle(TDocStd_Document)& document,
                         const aa::Key& key) noexcept {
    capture = {};
    try {
        if (document.IsNull() || document->HasOpenCommand())
            return Outcome::Busy;
        aa::persistence::Record savedAtlas;
        if (!aa::persistence::Read(document, key, savedAtlas))
            return HasAtlasOwnerMismatch(document, key)
                ? Outcome::OwnerMismatch : Outcome::Malformed;
        if (!savedAtlas.value) return Outcome::Absent;
        const Outcome classified = ClassifyMembers(
            document, savedAtlas.value->definition);
        if (classified != Outcome::Prepared) return classified;
        if (!aa::build::ObserveMembers(document,
                savedAtlas.value->definition.members,
                capture.observedMembers)
            || !aa::Current(savedAtlas.value->definition,
                            capture.observedMembers)) {
            capture = {};
            return Outcome::StaleSource;
        }
        std::vector<OwnerKey> owners;
        owners.reserve(savedAtlas.value->definition.members.size());
        for (const auto& member : savedAtlas.value->definition.members)
            owners.push_back(member.owner);
        if (!aa::build::CaptureMembers(document, owners, capture.members)) {
            capture = {};
            return Outcome::StaleSource;
        }
        const Outcome sources = CaptureSources(
            document, capture.members, capture.sources);
        if (sources != Outcome::Prepared) {
            capture = {};
            return sources;
        }
        persistence::Record savedBake;
        const auto bakeState = persistence::Read(document, key, savedBake);
        if (bakeState == persistence::ReadState::Absent) {
            capture = {};
            return Outcome::Absent;
        }
        if (bakeState != persistence::ReadState::Present
            || !(savedBake.definition.key == key)
            || savedBake.definition.layoutProof
                != savedAtlas.value->definition.layoutProof
            || savedBake.definition.bindings.size()
                != capture.sources.size()) {
            capture = {};
            return Outcome::Malformed;
        }
        for (std::size_t index = 0; index < capture.sources.size(); ++index) {
            if (!(capture.sources[index].fence
                    == savedBake.definition.bindings[index])) {
                capture = {};
                return Outcome::StaleBinding;
            }
        }
        capture.savedAtlas = savedAtlas.value->definition;
        capture.savedBake = savedBake.definition;
        capture.canonicalAtlasBytes = savedAtlas.value->bytes;
        if (capture.canonicalAtlasBytes.empty()) {
            capture = {};
            return Outcome::Malformed;
        }
        return Outcome::Prepared;
    } catch (...) {
        capture = {};
        return Outcome::Malformed;
    }
}

Outcome BakeForExport(const ExportCapture& capture,
                      const aa::Definition& finalAtlas,
                      const std::vector<aa::MemberUVAssignment>& assignments,
                      const aa::build::LayoutEvidence& layout,
                      ExportBake& output) noexcept {
    output = {};
    try {
        if (!(finalAtlas.key == capture.savedAtlas.key)
            || finalAtlas.members.size() != capture.savedAtlas.members.size()
            || assignments.size() != finalAtlas.members.size()
            || capture.members.members.size() != finalAtlas.members.size()
            || capture.sources.empty()
            || layout.charts.empty()
            || layout.chartMembers.size() != layout.charts.size())
            return Outcome::Malformed;
        for (std::size_t index = 0; index < finalAtlas.members.size(); ++index) {
            const auto& finalMember = finalAtlas.members[index];
            const auto& savedMember = capture.savedAtlas.members[index];
            if (!(finalMember.owner == savedMember.owner)
                || !(finalMember.finishing == savedMember.finishing)
                || !(finalMember.source == savedMember.source)
                || assignments[index].member != finalMember.member)
                return Outcome::StaleSource;
        }
        std::vector<kernel::Source> sources;
        sources.reserve(capture.sources.size());
        for (const auto& captured : capture.sources) {
            kernel::Image image;
            if (!kernel::DecodeImage(captured.envelope.workingBytes, image))
                return Outcome::MissingResource;
            sources.push_back({captured.fence, std::move(image)});
        }
        output.atlas = finalAtlas;
        output.assignments = assignments;
        if (!kernel::Bake(output.atlas, layout, sources,
                          output.outputs, output.evidence)) {
            output = {};
            return Outcome::Refused;
        }
        output.bake.key = finalAtlas.key;
        output.bake.layoutProof = finalAtlas.layoutProof;
        for (const auto& source : capture.sources)
            output.bake.bindings.push_back(source.fence);
        for (const auto& baked : output.outputs)
            output.bake.resources.push_back(baked.descriptor);
        if (!BindBakeProof(output.bake)) {
            output = {};
            return Outcome::Malformed;
        }
        return Outcome::Prepared;
    } catch (...) {
        output = {};
        return Outcome::Malformed;
    }
}

Outcome BuildAndBakeForExport(
    const ExportCapture& capture,
    const std::vector<aa::build::FinalMemberInput>& finalMembers,
    const aa::build::Settings& settings,
    ExportBake& output,
    std::string& diagnosis) noexcept {
    output = {};
    diagnosis.clear();
    try {
        aa::Definition atlas;
        std::vector<aa::MemberUVAssignment> assignments;
        aa::build::LayoutEvidence layout;
        const auto built = aa::build::BuildFinalAtlas(
            capture.savedAtlas.key, finalMembers, settings, atlas,
            assignments, diagnosis, &layout);
        if (built != aa::build::Status::Built) {
            if (diagnosis.empty())
                diagnosis = "build-status-" +
                    std::to_string(static_cast<unsigned>(built));
            return MapBuild(built);
        }
        const auto outcome = BakeForExport(
            capture, atlas, assignments, layout, output);
        if (outcome != Outcome::Prepared && diagnosis.empty())
            diagnosis = "bake-outcome-" +
                std::to_string(static_cast<unsigned>(outcome));
        return outcome;
    } catch (...) {
        output = {};
        diagnosis.clear();
        return Outcome::Malformed;
    }
}

Outcome BuildAndBakeTransientForExport(
    const TransientExportCapture& capture,
    const aa::build::FinalMemberInput& finalMember,
    const aa::build::Settings& settings,
    ExportBake& output,
    std::string& diagnosis) noexcept {
    output = {};
    diagnosis.clear();
    try {
        if (!(capture.member.owner == finalMember.savedMember.owner)
            || !(capture.member.finishing
                == finalMember.savedMember.finishing)
            || !(capture.member.source == finalMember.savedMember.source)
            || capture.sources.empty()) {
            diagnosis = "transient-capture-mismatch";
            return Outcome::StaleSource;
        }
        aa::Definition atlas;
        std::vector<aa::MemberUVAssignment> assignments;
        aa::build::LayoutEvidence layout;
        const auto built = aa::build::BuildFinalAtlas(
            capture.atlas, {finalMember}, settings, atlas,
            assignments, diagnosis, &layout);
        if (built != aa::build::Status::Built) {
            if (diagnosis.empty())
                diagnosis = "build-status-"
                    + std::to_string(static_cast<unsigned>(built));
            return MapBuild(built);
        }
        if (atlas.members.size() != 1 || assignments.size() != 1
            || !(atlas.members.front().owner == capture.member.owner)
            || !(atlas.members.front().finishing
                == capture.member.finishing)
            || !(atlas.members.front().source == capture.member.source)
            || assignments.front().member
                != atlas.members.front().member) {
            diagnosis = "transient-layout-mismatch";
            return Outcome::StaleSource;
        }
        std::vector<kernel::Source> sources;
        sources.reserve(capture.sources.size());
        for (const auto& captured : capture.sources) {
            kernel::Image image;
            if (!kernel::DecodeImage(
                    captured.envelope.workingBytes, image)) {
                diagnosis = "transient-source-decode";
                return Outcome::MissingResource;
            }
            sources.push_back({captured.fence, std::move(image)});
        }
        output.atlas = std::move(atlas);
        output.assignments = std::move(assignments);
        if (!kernel::Bake(output.atlas, layout, sources,
                output.outputs, output.evidence)) {
            output = {};
            diagnosis = "transient-kernel-bake";
            return Outcome::Refused;
        }
        output.bake.key = output.atlas.key;
        output.bake.layoutProof = output.atlas.layoutProof;
        for (const auto& source : capture.sources)
            output.bake.bindings.push_back(source.fence);
        for (const auto& baked : output.outputs)
            output.bake.resources.push_back(baked.descriptor);
        if (!BindBakeProof(output.bake)) {
            output = {};
            diagnosis = "transient-bake-proof";
            return Outcome::Malformed;
        }
        return Outcome::Prepared;
    } catch (...) {
        output = {};
        diagnosis = "transient-exception";
        return Outcome::Malformed;
    }
}

Outcome Currentness(const Handle(TDocStd_Document)& document, const aa::Key& key,
                    Definition* output) noexcept {
    if (output) *output = {};
    try {
        persistence::Record record;
        const auto state = persistence::Read(document, key, record);
        if (state == persistence::ReadState::Absent) return Outcome::Absent;
        if (state != persistence::ReadState::Present) return Outcome::Malformed;
        aa::persistence::Record atlas;
        if (!aa::persistence::Read(document, key, atlas)) return Outcome::Malformed;
        if (!atlas.value) return Outcome::StaleSource;
        if (!(atlas.value->definition.layoutProof == record.definition.layoutProof))
            return Outcome::StaleSource;
        // All-member recheck: the committed member fences must still match the
        // live owners (a real source edit stales the bake even when the atlas
        // record itself was never rewritten).
        std::vector<aa::Member> observed;
        if (!aa::build::ObserveMembers(document, atlas.value->definition.members, observed)
            || !aa::Current(atlas.value->definition, observed)) return Outcome::StaleSource;
        std::vector<OwnerKey> owners;
        for (const auto& member : atlas.value->definition.members) owners.push_back(member.owner);
        aa::Capture capture;
        if (!aa::build::CaptureMembers(document, owners, capture)) return Outcome::StaleSource;
        std::vector<CapturedSource> sources;
        const Outcome captured = CaptureSources(document, capture, sources);
        if (captured == Outcome::MissingResource || captured == Outcome::ForeignResource)
            return captured;
        if (captured == Outcome::Malformed) return Outcome::Malformed;
        if (captured != Outcome::Prepared) return Outcome::StaleBinding;
        if (sources.size() != record.definition.bindings.size()) return Outcome::StaleBinding;
        for (std::size_t index = 0; index < sources.size(); ++index)
            if (!(sources[index].fence == record.definition.bindings[index]))
                return Outcome::StaleBinding;
        if (output) *output = record.definition;
        return Outcome::Committed;
    } catch (...) { return Outcome::Malformed; }
}
} // namespace core3d::painted_atlas_bake::owner
