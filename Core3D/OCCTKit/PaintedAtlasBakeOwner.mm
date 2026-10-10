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

#ifdef DEBUG
#include <atomic>
#endif
#include <new>

namespace core3d::painted_atlas_bake::kernel {
namespace {
template <typename Reference>
class ScopedCF final {
public:
    ScopedCF() = default;
    explicit ScopedCF(Reference value) noexcept : value_(value) {}
    ScopedCF(const ScopedCF&) = delete;
    ScopedCF& operator=(const ScopedCF&) = delete;
    ~ScopedCF() { reset(); }
    Reference get() const noexcept { return value_; }
    explicit operator bool() const noexcept { return value_ != nullptr; }
    void reset(Reference value = nullptr) noexcept {
        if (value_) CFRelease(value_);
        value_ = value;
    }
private:
    Reference value_ = nullptr;
};
} // namespace

// Bounded ImageIO decode to exact straight RGBA8 working pixels (the same
// decoder the capture path uses; declared in PaintedAtlasBake.hxx so the
// DEBUG probe measures persisted PNG bytes through the identical route).
bool DecodeImage(
    const std::vector<std::uint8_t>& bytes,
    Image& output,
    const decal_layer::bake::accounting::View& operation) noexcept {
    namespace accounting = decal_layer::bake::accounting;
    using accounting::FailureSite;
    using accounting::Retention;
    using accounting::StorageDimension;
    using accounting::Ticket;
    output.reset();
    FailureSite activeAllocation = FailureSite::ImageEncodedCopy;
    @autoreleasepool {
        try {
            if (!operation.valid() || bytes.empty()
                || bytes.size() > face_image::kMaximumEncodedImageBytes)
                return false;
            Ticket encodedTicket;
            if (!encodedTicket.acquire(operation, StorageDimension::EncodedTexture,
                    bytes.size(), Retention::Scratch,
                    FailureSite::ImageEncodedCopy)
                || !operation.EnterAllocation(FailureSite::ImageEncodedCopy))
                return false;
            ScopedCF<CFDataRef> data(CFDataCreate(
                kCFAllocatorDefault, bytes.data(), bytes.size()));
            if (!data) return false;
            Ticket sourceTicket;
            if (!sourceTicket.acquire(operation, StorageDimension::EncodedTexture,
                    bytes.size(), Retention::Scratch,
                    FailureSite::ImageSourceMetadata)
                || !operation.EnterAllocation(FailureSite::ImageSourceMetadata)) {
                return false;
            }
            ScopedCF<CGImageSourceRef> source(
                CGImageSourceCreateWithData(data.get(), nullptr));
            if (!source) return false;
            ScopedCF<CFDictionaryRef> properties(
                CGImageSourceCopyPropertiesAtIndex(source.get(), 0, nullptr));
            if (!properties) return false;
            const auto number = [&](CFStringRef key, std::size_t& value) {
                CFTypeRef raw = CFDictionaryGetValue(properties.get(), key);
                long long candidate = 0;
                if (!raw || CFGetTypeID(raw) != CFNumberGetTypeID()
                    || !CFNumberGetValue(static_cast<CFNumberRef>(raw),
                        kCFNumberLongLongType, &candidate)
                    || candidate <= 0) return false;
                value = std::size_t(candidate);
                return static_cast<unsigned long long>(candidate)
                    <= std::numeric_limits<std::size_t>::max();
            };
            std::size_t width = 0, height = 0, depth = 0;
            CFTypeRef colorModelValue = CFDictionaryGetValue(
                properties.get(), kCGImagePropertyColorModel);
            CFTypeRef alphaValue = CFDictionaryGetValue(
                properties.get(), kCGImagePropertyHasAlpha);
            std::size_t components = 0;
            if (!number(kCGImagePropertyPixelWidth, width)
                || !number(kCGImagePropertyPixelHeight, height)
                || !number(kCGImagePropertyDepth, depth)
                || !colorModelValue
                || CFGetTypeID(colorModelValue) != CFStringGetTypeID()
                || !alphaValue
                || CFGetTypeID(alphaValue) != CFBooleanGetTypeID()) {
                return false;
            }
            const auto colorModel = static_cast<CFStringRef>(colorModelValue);
            if (CFEqual(colorModel, kCGImagePropertyColorModelRGB)) components = 3;
            else if (CFEqual(colorModel,
                         kCGImagePropertyColorModelGray)) components = 1;
            else if (CFEqual(colorModel,
                         kCGImagePropertyColorModelCMYK)) components = 4;
            else if (CFEqual(colorModel,
                         kCGImagePropertyColorModelLab)) components = 3;
            else {
                return false;
            }
            if (CFBooleanGetValue(static_cast<CFBooleanRef>(alphaValue)))
                ++components;
            properties.reset();
            if (width == 0 || height == 0
                || width > std::size_t(face_image::kMaximumImageDimension)
                || height > std::size_t(face_image::kMaximumImageDimension)
                || width > face_image::kMaximumImagePixels / height) {
                return false;
            }
            std::size_t componentBits = 0;
            if (!operation.CheckedProduct(components, depth, 1, componentBits,
                    FailureSite::ImageDecodedBacking)) return false;
            const std::size_t bytesPerPixel = componentBits / 8
                + (componentBits % 8 == 0 ? 0 : 1);
            Ticket decodedTicket;
            if (!decodedTicket.acquireProduct(operation,
                    StorageDimension::PrivateStorage, width, height,
                    bytesPerPixel, Retention::Scratch,
                    FailureSite::ImageDecodedBacking)) {
                return false;
            }
            const void* optionKeys[] = {
                kCGImageSourceShouldCache,
                kCGImageSourceShouldCacheImmediately,
            };
            const void* optionValues[] = {kCFBooleanFalse, kCFBooleanFalse};
            ScopedCF<CFDictionaryRef> options(CFDictionaryCreate(kCFAllocatorDefault,
                optionKeys, optionValues, 2, &kCFTypeDictionaryKeyCallBacks,
                &kCFTypeDictionaryValueCallBacks));
            if (!options
                || !operation.EnterAllocation(FailureSite::ImageCreate)) {
                return false;
            }
            activeAllocation = FailureSite::ImageCreate;
            ScopedCF<CGImageRef> image(CGImageSourceCreateImageAtIndex(
                source.get(), 0, options.get()));
            options.reset();
            source.reset();
            sourceTicket.reset();
            encodedTicket.reset();
            if (!image) return false;
            if (CGImageGetWidth(image.get()) != width
                || CGImageGetHeight(image.get()) != height) return false;
            Ticket rgbaTicket;
            if (!rgbaTicket.acquireProduct(operation,
                    StorageDimension::PrivateStorage, width, height, 4,
                    Retention::Retained, FailureSite::ImageRGBA)
                || !operation.EnterAllocation(FailureSite::ImageRGBA)) {
                return false;
            }
            activeAllocation = FailureSite::ImageRGBA;
            std::vector<std::uint8_t> rgba(width * height * 4, 0);
            Ticket contextTicket;
            if (!contextTicket.acquireProduct(operation,
                    StorageDimension::PrivateStorage, width, height, 4,
                    Retention::Scratch, FailureSite::ImageBitmapContext)
                || !operation.EnterAllocation(
                    FailureSite::ImageBitmapContext)) {
                return false;
            }
            activeAllocation = FailureSite::ImageBitmapContext;
            CGColorSpaceRef borrowedColor = CGImageGetColorSpace(image.get());
            if (borrowedColor) CFRetain(borrowedColor);
            ScopedCF<CGColorSpaceRef> color(borrowedColor
                ? borrowedColor : CGColorSpaceCreateDeviceRGB());
            ScopedCF<CGContextRef> context(color
                ? CGBitmapContextCreate(rgba.data(), width, height, 8,
                    width * 4, color.get(), kCGImageAlphaPremultipliedLast
                        | kCGBitmapByteOrder32Big)
                : nullptr);
            if (!context) return false;
            CGContextSetBlendMode(context.get(), kCGBlendModeCopy);
            CGContextDrawImage(context.get(), CGRectMake(0, 0, width, height),
                image.get());
            context.reset();
            color.reset();
            image.reset();
            decodedTicket.reset();
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
            output.rgba = std::move(rgba);
            output.rgbaTicket = std::move(rgbaTicket);
            return true;
        } catch (const std::bad_alloc&) {
            output.reset();
            operation.AllocationFailed(activeAllocation);
            return false;
        } catch (...) { output.reset(); return false; }
    }
}

bool DecodeImage(const std::vector<std::uint8_t>& bytes,
                 Image& output) noexcept {
    auto operation = decal_layer::bake::accounting::MakeOperationOwner();
    if (!operation.valid()) { output.reset(); return false; }
    if (!DecodeImage(bytes, output, operation.view)) return false;
    output.operationOwner = std::move(operation);
    return true;
}
} // namespace core3d::painted_atlas_bake::kernel

namespace core3d::painted_atlas_bake::owner {
namespace {
namespace fi = core3d::face_image;
namespace aa = core3d::asset_atlas;
#ifdef DEBUG
thread_local std::size_t gDebugCaptureSourceOrdinal = 0;
thread_local std::size_t gDebugDenyCaptureSourceOrdinal = 0;
struct DebugSourceStorageState final {
    decal_layer::bake::accounting::FailureSite deny =
        decal_layer::bake::accounting::FailureSite::None;
    std::array<std::size_t, 4> attempts{};
    std::array<std::size_t, 4> entries{};
    std::array<std::size_t, 4> denials{};
};
thread_local DebugSourceStorageState gDebugSourceStorage;
thread_local bool gDebugDenyCaptureReadPreflight = false;
thread_local std::size_t gDebugBindingReadEntries = 0;
thread_local std::size_t gDebugResourceReadEntries = 0;
std::atomic_bool gDebugDenyNextPrivateSourceVectorReservation{false};
std::atomic_size_t gDebugPrivateSourceVectorAttempts{0};
std::atomic_size_t gDebugPrivateSourceVectorEntries{0};
std::atomic_size_t gDebugPrivateSourceVectorDenials{0};
thread_local bool gDebugOrdinaryTraceArmed = false;
thread_local void* gDebugExpectedOrdinaryContext = nullptr;
thread_local bool gDebugOrdinaryCommitOverlap = false;
thread_local std::size_t gDebugAdoptionEntries = 0;
thread_local std::size_t gDebugPersistenceEntries = 0;

std::size_t DebugSourceStorageIndex(
    decal_layer::bake::accounting::FailureSite site) noexcept {
    using Site = decal_layer::bake::accounting::FailureSite;
    switch (site) {
        case Site::SourceWorkingBytes: return 0;
        case Site::SourceOriginalBytes: return 1;
        case Site::SourceIdentityBytes: return 2;
        case Site::SourceVectorStorage: return 3;
        default: return 0;
    }
}
#endif

bool AddReadBytes(std::size_t& total, std::size_t bytes) noexcept {
    if (bytes > std::numeric_limits<std::size_t>::max() - total) return false;
    total += bytes;
    return true;
}

struct OrdinaryReadAdmission final {
    decal_layer::bake::accounting::Ticket encoded;
    decal_layer::bake::accounting::Ticket local;
};

bool AcquireOrdinaryStorage(
    decal_layer::bake::accounting::Ticket& ticket,
    const decal_layer::bake::accounting::View& operation,
    decal_layer::bake::accounting::StorageDimension dimension,
    std::size_t bytes,
    decal_layer::bake::accounting::Retention retention,
    decal_layer::bake::accounting::FailureSite site) noexcept {
    if (bytes == 0) return true;
    return ticket.acquire(operation, dimension, bytes, retention, site)
        && operation.EnterAllocation(site);
}

Outcome PreflightAtlasRead(
    const Handle(TDocStd_Document)& document,
    const decal_layer::bake::accounting::View& operation,
    OrdinaryReadAdmission& admission) noexcept {
    using namespace decal_layer::bake::accounting;
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || !operation.valid()) return Outcome::Malformed;
        const TDF_Label root = document->Main().FindChild(
            aa::persistence::RootTag, Standard_False);
        std::size_t records = 0, members = 0, largestCanonical = 0;
        if (!root.IsNull()) {
            for (TDF_ChildIterator child(root, Standard_False);
                 child.More(); child.Next()) {
                Handle(aa::persistence::Attribute) attribute;
                if (!child.Value().FindAttribute(
                        aa::persistence::AttributeID(), attribute)
                    || attribute.IsNull() || !attribute->value()) continue;
                if (!AddReadBytes(records, 1U)
                    || !AddReadBytes(members,
                        attribute->value()->definition.members.size()))
                    return Outcome::OverBudget;
                largestCanonical = std::max(largestCanonical,
                    attribute->value()->bytes.size());
            }
        }
        std::size_t recordBytes = 0, memberBytes = 0, localBytes = 0;
        if (!operation.CheckedProduct(records, sizeof(aa::persistence::Record),
                2U, recordBytes, FailureSite::AtlasReadScratch)
            || !operation.CheckedProduct(members + aa::kMaximumMembers,
                sizeof(TDF_Label), 2U, memberBytes,
                FailureSite::AtlasReadScratch)
            || !AddReadBytes(localBytes, recordBytes)
            || !AddReadBytes(localBytes, memberBytes)
            || !AcquireOrdinaryStorage(admission.encoded, operation,
                StorageDimension::EncodedTexture, largestCanonical,
                Retention::Scratch, FailureSite::AtlasReadScratch)
            || !AcquireOrdinaryStorage(admission.local, operation,
                StorageDimension::PrivateStorage, localBytes,
                Retention::Scratch, FailureSite::AtlasReadScratch))
            return Outcome::OverBudget;
        return Outcome::Prepared;
    } catch (const std::bad_alloc&) {
        operation.AllocationFailed(
            decal_layer::bake::accounting::FailureSite::AtlasReadScratch);
        return Outcome::OverBudget;
    } catch (...) { return Outcome::Malformed; }
}

Outcome PreflightBakeRead(
    const Handle(TDocStd_Document)& document,
    const decal_layer::bake::accounting::View& operation,
    OrdinaryReadAdmission& admission) noexcept {
    using namespace decal_layer::bake::accounting;
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || !operation.valid()) return Outcome::Malformed;
        const TDF_Label root = document->Main().FindChild(
            persistence::RootTag, Standard_False);
        std::size_t hexBytes = 0, records = 0, values = 0;
        if (!root.IsNull()) {
            for (TDF_ChildIterator child(root, Standard_False);
                 child.More(); child.Next()) {
                Handle(TDataStd_Integer) chunks, bindings, resources;
                if (!child.Value().FindAttribute(
                        persistence::ChunkCountID(), chunks)
                    || chunks.IsNull() || chunks->Get() <= 0
                    || chunks->Get() > persistence::MaximumChunks)
                    continue;
                if (!AddReadBytes(records, 1U)) return Outcome::OverBudget;
                if (child.Value().FindAttribute(
                        persistence::BindingCountID(), bindings)
                    && !bindings.IsNull() && bindings->Get() > 0
                    && !AddReadBytes(values, std::size_t(bindings->Get())))
                    return Outcome::OverBudget;
                if (child.Value().FindAttribute(
                        persistence::ResourceCountID(), resources)
                    && !resources.IsNull() && resources->Get() > 0
                    && !AddReadBytes(values, std::size_t(resources->Get())))
                    return Outcome::OverBudget;
                for (Standard_Integer index = 1; index <= chunks->Get(); ++index) {
                    Handle(TDataStd_AsciiString) text;
                    const TDF_Label label = child.Value().FindChild(
                        index, Standard_False);
                    if (!label.IsNull()
                        && label.FindAttribute(
                            TDataStd_AsciiString::GetID(), text)
                        && !text.IsNull()
                        && !AddReadBytes(hexBytes,
                            std::size_t(std::max(0, text->Get().Length()))))
                        return Outcome::OverBudget;
                }
            }
        }
        const std::size_t canonicalBytes = hexBytes / 2U + hexBytes % 2U;
        std::size_t encodedBytes = 0, recordBytes = 0, valueBytes = 0;
        if (!operation.CheckedProduct(canonicalBytes, 5U, 1U,
                encodedBytes, FailureSite::BakeReadScratch)
            || !operation.CheckedProduct(records, sizeof(persistence::Record),
                2U, recordBytes, FailureSite::BakeReadScratch)
            || !operation.CheckedProduct(values,
                std::max(sizeof(BindingFence), sizeof(BakedResource)),
                2U, valueBytes, FailureSite::BakeReadScratch)
            || !AddReadBytes(recordBytes, valueBytes)
            || !AcquireOrdinaryStorage(admission.encoded, operation,
                StorageDimension::EncodedTexture, encodedBytes,
                Retention::Scratch, FailureSite::BakeReadScratch)
            || !AcquireOrdinaryStorage(admission.local, operation,
                StorageDimension::PrivateStorage, recordBytes,
                Retention::Scratch, FailureSite::BakeReadScratch))
            return Outcome::OverBudget;
        return Outcome::Prepared;
    } catch (const std::bad_alloc&) {
        operation.AllocationFailed(
            decal_layer::bake::accounting::FailureSite::BakeReadScratch);
        return Outcome::OverBudget;
    } catch (...) { return Outcome::Malformed; }
}

#ifdef DEBUG
bool OrdinaryStagingOverlapCharged(const Staging& staging) noexcept {
    if (!staging.operation.valid()
        || staging.priorAtlasBytes.empty()
        || staging.priorAtlasBytesTicket.bytes
            < staging.priorAtlasBytes.capacity()
        || staging.observedMembers.empty()
        || staging.observedMembersTicket.bytes
            < staging.observedMembers.capacity() * sizeof(aa::Member)
        || staging.capturedSources.empty()
        || staging.capturedSources.vectorTicket.bytes == 0)
        return false;
    const void* context = staging.operation.view.context;
    if (staging.priorAtlasBytesTicket.view.context != context
        || staging.observedMembersTicket.view.context != context
        || staging.capturedSources.vectorTicket.view.context != context)
        return false;
    for (const auto& source : staging.capturedSources)
        if (source.originalBytesTicket.bytes != 0
                && source.originalBytesTicket.view.context != context
            || source.workingBytesTicket.bytes != 0
                && source.workingBytesTicket.view.context != context)
            return false;
    return true;
}
#endif

std::size_t BorrowedBindingCount(const TDF_Label& owner) noexcept {
    if (owner.IsNull()) return 0;
    const TDF_Label record = owner.FindChild(
        fi::persistence::bindings::RecordTag, Standard_False);
    Handle(TDF_Attribute) marker;
    Handle(TDataStd_Integer) count;
    if (record.IsNull()
        || !record.FindAttribute(
            fi::persistence::bindings::MarkerID(), marker)
        || Handle(TDataStd_UAttribute)::DownCast(marker).IsNull()
        || !record.FindAttribute(
            fi::persistence::bindings::BindingCountID(), count)
        || count.IsNull() || count->Get() <= 0
        || count->Get() > Standard_Integer(fi::kMaximumBindings))
        return 0;
    return std::size_t(count->Get());
}

struct BindingReadAdmission final {
    decal_layer::bake::accounting::Ticket scratch;
    std::size_t bindingCount = 0;
};

Outcome PreflightBindingRead(
    const Handle(TDocStd_Document)& document,
    const TDF_Label& owner,
    const decal_layer::bake::accounting::View& operation,
    BindingReadAdmission& admission) noexcept {
    using namespace decal_layer::bake::accounting;
    admission = {};
    try {
        if (document.IsNull() || owner.IsNull() || !operation.valid())
            return Outcome::Malformed;
        TDF_Label record;
        for (TDF_ChildIterator child(owner, Standard_True);
             child.More(); child.Next()) {
            if (!fi::persistence::bindings::HasSchemaAttribute(child.Value()))
                continue;
            Handle(TDF_Attribute) marker;
            if (child.Value().Father().IsEqual(owner)
                && child.Value().Tag() == fi::persistence::bindings::RecordTag
                && child.Value().FindAttribute(
                    fi::persistence::bindings::MarkerID(), marker)
                && !Handle(TDataStd_UAttribute)::DownCast(marker).IsNull()
                && record.IsNull())
                record = child.Value();
        }
        if (record.IsNull()) return Outcome::Prepared;
        Handle(TDataStd_Integer) count, chunks;
        Handle(TDataStd_AsciiString) digest;
        if (!record.FindAttribute(
                fi::persistence::bindings::BindingCountID(), count)
            || !record.FindAttribute(
                fi::persistence::bindings::ChunkCountID(), chunks)
            || !record.FindAttribute(
                fi::persistence::bindings::DigestID(), digest)
            || count.IsNull() || chunks.IsNull() || digest.IsNull()
            || count->Get() <= 0
            || count->Get() > Standard_Integer(fi::kMaximumBindings)
            || chunks->Get() <= 0
            || chunks->Get() > fi::persistence::bindings::MaximumChunks)
            return Outcome::Prepared;
        admission.bindingCount = std::size_t(count->Get());
        std::size_t scratch = std::size_t(std::max(0, digest->Get().Length()));
        std::size_t hexBytes = 0;
        for (Standard_Integer index = 1; index <= chunks->Get(); ++index) {
            const TDF_Label chunk = record.FindChild(index, Standard_False);
            Handle(TDataStd_AsciiString) text;
            if (chunk.IsNull()
                || !chunk.FindAttribute(TDataStd_AsciiString::GetID(), text)
                || text.IsNull()) continue;
            const Standard_Integer rawLength = text->Get().Length();
            if (rawLength < 0
                || !AddReadBytes(hexBytes, std::size_t(rawLength))
                || !AddReadBytes(scratch, std::size_t(rawLength)))
                return Outcome::OverBudget;
        }
        const std::size_t canonicalBytes = hexBytes / 2U + hexBytes % 2U;
        std::size_t bindingBytes = 0;
        if (!operation.CheckedProduct(admission.bindingCount,
                sizeof(fi::Binding), 1, bindingBytes,
                FailureSite::BindingReadScratch)
            || !AddReadBytes(scratch, hexBytes)
            || !AddReadBytes(scratch, canonicalBytes)
            || !AddReadBytes(scratch, canonicalBytes)
            || !AddReadBytes(scratch, canonicalBytes)
            || !AddReadBytes(scratch, bindingBytes)
            || !AddReadBytes(scratch, 64U))
            return Outcome::OverBudget;
#ifdef DEBUG
        if (gDebugDenyCaptureReadPreflight) {
            gDebugDenyCaptureReadPreflight = false;
            (void)operation.Reserve(StorageDimension::PrivateStorage,
                std::numeric_limits<std::size_t>::max(), Retention::Scratch,
                FailureSite::BindingReadScratch);
            return Outcome::OverBudget;
        }
#endif
        if (!admission.scratch.acquire(operation,
                StorageDimension::PrivateStorage, scratch,
                Retention::Scratch, FailureSite::BindingReadScratch))
            return Outcome::OverBudget;
        return Outcome::Prepared;
    } catch (...) {
        admission = {};
        return Outcome::Malformed;
    }
}

bool AcquireSourceStorage(
    decal_layer::bake::accounting::Ticket& ticket,
    const decal_layer::bake::accounting::View& operation,
    decal_layer::bake::accounting::StorageDimension dimension,
    std::size_t bytes,
    decal_layer::bake::accounting::Retention retention,
    decal_layer::bake::accounting::FailureSite site) noexcept {
#ifdef DEBUG
    const std::size_t debugIndex = DebugSourceStorageIndex(site);
    ++gDebugSourceStorage.attempts[debugIndex];
    const bool privateSourceVector =
        site == decal_layer::bake::accounting::FailureSite::SourceVectorStorage;
    if (privateSourceVector)
        gDebugPrivateSourceVectorAttempts.fetch_add(1, std::memory_order_relaxed);
    if (privateSourceVector
        && gDebugDenyNextPrivateSourceVectorReservation.exchange(
            false, std::memory_order_acq_rel)) {
        gDebugPrivateSourceVectorDenials.fetch_add(1, std::memory_order_relaxed);
        (void)operation.Reserve(dimension,
            std::numeric_limits<std::size_t>::max(), retention, site);
        return false;
    }
    if (gDebugSourceStorage.deny == site) {
        gDebugSourceStorage.deny =
            decal_layer::bake::accounting::FailureSite::None;
        ++gDebugSourceStorage.denials[debugIndex];
        (void)operation.Reserve(dimension,
            std::numeric_limits<std::size_t>::max(), retention, site);
        return false;
    }
#endif
    if (!ticket.acquire(operation, dimension, bytes, retention, site)
        || !operation.EnterAllocation(site)) return false;
#ifdef DEBUG
    ++gDebugSourceStorage.entries[debugIndex];
    if (privateSourceVector)
        gDebugPrivateSourceVectorEntries.fetch_add(1, std::memory_order_relaxed);
#endif
    return true;
}

UUID UUIDFromDigest(const Digest& digest) noexcept {
    UUID value{};
    std::copy_n(digest.begin(), value.size(), value.begin());
    if (!retained_recipe::Nonzero(value)) value.back() = 1;
    return value;
}

enum class TextureCapture : std::uint8_t {
    Captured, Foreign, Missing, OverBudget
};

// XCAF texture slots must carry document-owned bytes: a file-backed texture
// references foreign bytes and is refused ForeignResource, never silently
// read from disk.
TextureCapture SyntheticEnvelope(const Handle(Image_Texture)& texture, const OwnerKey& owner,
                       Role role, const Digest& selector, const Digest& bindingProof,
                       CapturedSource& output,
                       const decal_layer::bake::accounting::View& operation) noexcept {
    output = {};
    auto activeAllocation =
        decal_layer::bake::accounting::FailureSite::SourceWorkingBytes;
    try {
        if (texture.IsNull()) return TextureCapture::Missing;
        if (!texture->FilePath().IsEmpty()) return TextureCapture::Foreign;
        const Handle(NCollection_Buffer)& buffer = texture->DataBuffer();
        if (buffer.IsNull() || !buffer->Data() || buffer->Size() == 0
            || buffer->Size() > fi::kMaximumEncodedImageBytes) return TextureCapture::Missing;
        decal_layer::bake::accounting::Ticket workingTicket;
        if (!AcquireSourceStorage(workingTicket, operation,
                decal_layer::bake::accounting::StorageDimension::EncodedTexture,
                buffer->Size(),
                decal_layer::bake::accounting::Retention::Retained,
                activeAllocation)) return TextureCapture::OverBudget;
        std::vector<std::uint8_t> bytes;
        bytes.reserve(buffer->Size());
        bytes.insert(bytes.end(), buffer->Data(), buffer->Data() + buffer->Size());
        if (bytes.capacity() != buffer->Size()) {
            operation.AllocationFailed(activeAllocation);
            return TextureCapture::OverBudget;
        }
        Digest content{};
        if (!fi::HashFaceImageBytes(bytes, content)) return TextureCapture::Missing;
        kernel::Image decoded;
        if (!kernel::DecodeImage(bytes, decoded, operation))
            return operation.Recheck(
                    decal_layer::bake::accounting::FailureSite::ImageRGBA)
                ? TextureCapture::Missing : TextureCapture::OverBudget;
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
        activeAllocation =
            decal_layer::bake::accounting::FailureSite::SourceOriginalBytes;
        decal_layer::bake::accounting::Ticket originalTicket;
        if (!AcquireSourceStorage(originalTicket, operation,
                decal_layer::bake::accounting::StorageDimension::EncodedTexture,
                bytes.capacity(),
                decal_layer::bake::accounting::Retention::Retained,
                activeAllocation)) return TextureCapture::OverBudget;
        output.envelope.originalBytes = bytes;
        if (output.envelope.originalBytes.capacity() != bytes.capacity()) {
            operation.AllocationFailed(activeAllocation);
            return TextureCapture::OverBudget;
        }
        output.envelope.workingBytes = std::move(bytes);
        output.fence.owner = owner; output.fence.resource = output.envelope.resource;
        output.fence.originalContent = content; output.fence.workingContent = content;
        output.fence.selectorProof = selector; output.fence.bindingProof = bindingProof;
        output.fence.role = role;
        output.fence.colorSpace = (role == Role::BaseColor || role == Role::Emissive)
            ? ColorSpace::SRGB : ColorSpace::Linear;
        activeAllocation =
            decal_layer::bake::accounting::FailureSite::SourceIdentityBytes;
        constexpr std::size_t identityBytes = UUID{}.size() * 2U + 1U
            + Digest{}.size();
        static_assert(identityBytes == 65U);
        decal_layer::bake::accounting::Ticket identityTicket;
        if (!AcquireSourceStorage(identityTicket, operation,
                decal_layer::bake::accounting::StorageDimension::PrivateStorage,
                identityBytes,
                decal_layer::bake::accounting::Retention::Scratch,
                activeAllocation)) return TextureCapture::OverBudget;
        std::vector<std::uint8_t> id;
        id.reserve(identityBytes);
        id.insert(id.end(), owner.entity.begin(), owner.entity.end());
        id.insert(id.end(), owner.definition.begin(), owner.definition.end());
        id.push_back(std::uint8_t(role)); id.insert(id.end(), content.begin(), content.end());
        if (id.size() != identityBytes || id.capacity() != identityBytes) {
            operation.AllocationFailed(activeAllocation);
            return TextureCapture::OverBudget;
        }
        Digest binding{};
        if (!retained_solid::Hash(id, binding)) return TextureCapture::Missing;
        output.fence.binding = UUIDFromDigest(binding);
        output.originalBytesTicket = std::move(originalTicket);
        output.workingBytesTicket = std::move(workingTicket);
        return Valid(output.fence) ? TextureCapture::Captured : TextureCapture::Missing;
    } catch (const std::bad_alloc&) {
        output = {};
        operation.AllocationFailed(activeAllocation);
        return TextureCapture::OverBudget;
    } catch (...) { output = {}; return TextureCapture::Missing; }
}

// Captures every SYFI/1 binding of every member plus the XCAF texture slots
// of painted members. Outcome-typed: an empty capture means there is nothing
// to bake (Refused); a dangling or undecodable resource is MissingResource;
// a file-backed XCAF texture is ForeignResource; a malformed SYFI record or
// resource table is Malformed.
Outcome CaptureSources(const Handle(TDocStd_Document)& document,
                       const aa::Capture& members,
                       CapturedSourceStorage& output,
                       const decal_layer::bake::accounting::View& operation) noexcept {
    output.reset();
    CapturedSourceStorage staged;
    auto activeAllocation =
        decal_layer::bake::accounting::FailureSite::CapturedSourceStorage;
    try {
        if (document.IsNull() || !operation.valid()) return Outcome::Malformed;
#ifdef DEBUG
        ++gDebugCaptureSourceOrdinal;
        if (gDebugDenyCaptureSourceOrdinal != 0
            && gDebugCaptureSourceOrdinal == gDebugDenyCaptureSourceOrdinal) {
            gDebugDenyCaptureSourceOrdinal = 0;
            (void)operation.Reserve(
                decal_layer::bake::accounting::StorageDimension::PrivateStorage,
                std::numeric_limits<std::size_t>::max(),
                decal_layer::bake::accounting::Retention::Scratch,
                decal_layer::bake::accounting::FailureSite::ImageDecodedBacking);
            return Outcome::OverBudget;
        }
#endif
        const auto materialTool = XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
        std::size_t sourceCapacity = 0;
        for (const auto& member : members.members) {
            if (!AddReadBytes(sourceCapacity,
                    BorrowedBindingCount(member.ownerLabel)))
                return Outcome::OverBudget;
            if (!member.painted) continue;
            TDF_Label materialLabel;
            XCAFDoc_VisMaterialTool::GetShapeMaterial(
                member.ownerLabel, materialLabel);
            Handle(XCAFDoc_VisMaterial) material;
            if (!materialLabel.IsNull() && !materialTool.IsNull())
                material = materialTool->GetMaterial(materialLabel);
            if (material.IsNull()) continue;
            const auto countTexture = [&](const Handle(Image_Texture)& texture) {
                return texture.IsNull()
                    || AddReadBytes(sourceCapacity, 1U);
            };
            if (material->HasCommonMaterial()
                && !countTexture(material->CommonMaterial().DiffuseTexture))
                return Outcome::OverBudget;
            if (material->HasPbrMaterial()) {
                const auto& pbr = material->PbrMaterial();
                if (!countTexture(pbr.BaseColorTexture)
                    || !countTexture(pbr.EmissiveTexture)
                    || !countTexture(pbr.MetallicRoughnessTexture)
                    || !countTexture(pbr.OcclusionTexture)
                    || !countTexture(pbr.NormalTexture))
                    return Outcome::OverBudget;
            }
        }
        if (sourceCapacity != 0) {
            std::size_t sourceBytes = 0;
            if (!operation.CheckedProduct(sourceCapacity,
                    sizeof(CapturedSource), 1, sourceBytes,
                    activeAllocation)
                || !staged.vectorTicket.acquire(operation,
                    decal_layer::bake::accounting::StorageDimension::PrivateStorage,
                    sourceBytes,
                    decal_layer::bake::accounting::Retention::Retained,
                    activeAllocation)
                || !operation.EnterAllocation(activeAllocation))
                return Outcome::OverBudget;
            staged.values.reserve(sourceCapacity);
            if (staged.values.capacity() != sourceCapacity) {
                operation.AllocationFailed(activeAllocation);
                return Outcome::OverBudget;
            }
        }
        for (const auto& member : members.members) {
            BindingReadAdmission bindingAdmission;
            const Outcome bindingPreflight = PreflightBindingRead(
                document, member.ownerLabel, operation, bindingAdmission);
            if (bindingPreflight != Outcome::Prepared) return bindingPreflight;
            fi::Definition definition;
            std::vector<std::uint8_t> bindingBytes;
            activeAllocation =
                decal_layer::bake::accounting::FailureSite::BindingReadMaterialize;
            if (!operation.EnterAllocation(activeAllocation))
                return Outcome::OverBudget;
#ifdef DEBUG
            ++gDebugBindingReadEntries;
#endif
            const auto state = fi::persistence::bindings::Read(
                document, member.ownerLabel, definition, &bindingBytes);
            if (state == fi::persistence::bindings::ReadState::Malformed)
                return Outcome::Malformed;
            if (state == fi::persistence::bindings::ReadState::Present) {
                for (const auto& binding : definition.bindings) {
                    ResourceReadAdmission readAdmission;
                    const Outcome preflight = PreflightResourceRead(
                        document, binding.resource, operation, readAdmission);
                    if (preflight != Outcome::Prepared) return preflight;
                    fi::persistence::resources::Record resource;
                    activeAllocation = decal_layer::bake::accounting::
                        FailureSite::ResourceReadMaterialize;
                    if (!operation.EnterAllocation(activeAllocation))
                        return Outcome::OverBudget;
#ifdef DEBUG
                    ++gDebugResourceReadEntries;
#endif
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
                    source.originalBytesTicket =
                        std::move(readAdmission.originalBytes);
                    source.workingBytesTicket =
                        std::move(readAdmission.workingBytes);
                    if (!Valid(source.fence)) return Outcome::Malformed;
                    if (staged.values.size() >= staged.values.capacity()) {
                        operation.AllocationFailed(
                            decal_layer::bake::accounting::FailureSite::
                                CapturedSourceStorage);
                        return Outcome::OverBudget;
                    }
                    staged.values.push_back(std::move(source));
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
                for (const auto& existing : staged.values)
                    if (existing.fence.owner == member.slot.owner && existing.fence.role == role)
                        return Outcome::Prepared;
                CapturedSource source;
                const auto captured = SyntheticEnvelope(texture, member.slot.owner, role,
                    member.slot.source.geometry, member.slot.source.recipe, source,
                    operation);
                if (captured == TextureCapture::Foreign) return Outcome::ForeignResource;
                if (captured == TextureCapture::OverBudget) return Outcome::OverBudget;
                if (captured != TextureCapture::Captured) return Outcome::MissingResource;
                if (staged.values.size() >= staged.values.capacity()) {
                    operation.AllocationFailed(
                        decal_layer::bake::accounting::FailureSite::
                            CapturedSourceStorage);
                    return Outcome::OverBudget;
                }
                staged.values.push_back(std::move(source)); return Outcome::Prepared;
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
        if (staged.empty()) return Outcome::Refused;
        std::sort(staged.values.begin(), staged.values.end(),
                  [&](const CapturedSource& a, const CapturedSource& b) {
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
        output = std::move(staged);
        return Outcome::Prepared;
    } catch (const std::bad_alloc&) {
        output.reset();
        operation.AllocationFailed(activeAllocation);
        return Outcome::OverBudget;
    } catch (...) { output.reset(); return Outcome::Malformed; }
}

Outcome DecodeCapturedSources(
    const std::vector<CapturedSource>& captured,
    const decal_layer::bake::accounting::View& operation,
    kernel::SourceStorage& output) noexcept {
    using namespace decal_layer::bake::accounting;
    output.reset();
    try {
        std::size_t bytes = 0;
        if (captured.empty()
            || !operation.CheckedProduct(captured.size(), sizeof(kernel::Source),
                    1, bytes, FailureSite::SourceVectorStorage)
            || !AcquireSourceStorage(output.vectorTicket, operation,
                    StorageDimension::PrivateStorage, bytes,
                    Retention::Scratch, FailureSite::SourceVectorStorage))
            return Outcome::OverBudget;
        output.values.reserve(captured.size());
        if (output.values.capacity() != captured.size()) {
            operation.AllocationFailed(FailureSite::SourceVectorStorage);
            output.reset();
            return Outcome::OverBudget;
        }
        for (const auto& source : captured) {
            kernel::Image image;
            if (!kernel::DecodeImage(source.envelope.workingBytes, image, operation)) {
                output.reset();
                return operation.Recheck(FailureSite::ImageRGBA)
                    ? Outcome::MissingResource : Outcome::OverBudget;
            }
            output.values.push_back({source.fence, std::move(image)});
        }
        return Outcome::Prepared;
    } catch (const std::bad_alloc&) {
        operation.AllocationFailed(FailureSite::SourceVectorStorage);
        output.reset();
        return Outcome::OverBudget;
    } catch (...) {
        output.reset();
        return Outcome::Malformed;
    }
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

Outcome PreflightResourceRead(
    const Handle(TDocStd_Document)& document,
    const face_image::UUID& resource,
    const decal_layer::bake::accounting::View& operation,
    ResourceReadAdmission& admission) noexcept {
    namespace accounting = decal_layer::bake::accounting;
    namespace resources = face_image::persistence::resources;
    admission.reset();
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || !retained_recipe::Nonzero(resource) || !operation.valid())
            return Outcome::Malformed;
        const TDF_Label root = document->Main().FindChild(
            resources::RootTag, Standard_False);
        if (root.IsNull()) return Outcome::Prepared;
        std::size_t records = 0;
        std::size_t serializationBytes = 0;
        std::size_t selectedSerializationBytes = 0;
        std::size_t originalBytes = 0;
        std::size_t workingBytes = 0;
        for (TDF_ChildIterator child(root, Standard_False);
             child.More(); child.Next()) {
            const TDF_Label label = child.Value();
            if (!label.IsAttribute(resources::AttributeID())) continue;
            if (!AddReadBytes(records, 1U)) return Outcome::OverBudget;
            Handle(resources::Attribute) attribute;
            if (!label.FindAttribute(resources::AttributeID(), attribute)
                || attribute.IsNull() || !attribute->value())
                continue;
            const auto& value = attribute->value();
            if (!AddReadBytes(serializationBytes, value->bytes.size()))
                return Outcome::OverBudget;
            if (value->envelope.resource == resource) {
                admission.selected = true;
                selectedSerializationBytes = value->bytes.size();
                originalBytes = value->envelope.originalBytes.size();
                workingBytes = value->envelope.workingBytes.size();
            }
        }
        std::size_t tableBytes = 0;
        if (!operation.CheckedProduct(records, sizeof(resources::Record), 1,
                tableBytes, accounting::FailureSite::ResourceTableScratch)
            || !AddReadBytes(serializationBytes, selectedSerializationBytes))
            return Outcome::OverBudget;
        if (!admission.table.acquire(operation,
                accounting::StorageDimension::PrivateStorage, tableBytes,
                accounting::Retention::Scratch,
                accounting::FailureSite::ResourceTableScratch)
            || !admission.serialization.acquire(operation,
                accounting::StorageDimension::EncodedTexture,
                serializationBytes, accounting::Retention::Scratch,
                accounting::FailureSite::ResourceSerializationScratch))
            return Outcome::OverBudget;
        if (admission.selected
            && (!admission.originalBytes.acquire(operation,
                    accounting::StorageDimension::EncodedTexture,
                    originalBytes, accounting::Retention::Retained,
                    accounting::FailureSite::SourceOriginalBytes)
                || !admission.workingBytes.acquire(operation,
                    accounting::StorageDimension::EncodedTexture,
                    workingBytes, accounting::Retention::Retained,
                    accounting::FailureSite::SourceWorkingBytes))) {
            admission.reset();
            return Outcome::OverBudget;
        }
        return Outcome::Prepared;
    } catch (const std::bad_alloc&) {
        admission.reset();
        operation.AllocationFailed(
            accounting::FailureSite::ResourceTableScratch);
        return Outcome::OverBudget;
    } catch (...) {
        admission.reset();
        return Outcome::Malformed;
    }
}

#ifdef DEBUG
void DebugDenyCaptureSourcesAtOrdinal(std::size_t ordinal) noexcept
{
    gDebugCaptureSourceOrdinal = 0;
    gDebugDenyCaptureSourceOrdinal = ordinal;
}

void DebugClearCaptureSourcesDenial() noexcept
{
    gDebugCaptureSourceOrdinal = 0;
    gDebugDenyCaptureSourceOrdinal = 0;
}

DebugCaptureReadEvidence DebugExerciseE4CaptureReadPreflight(
    const Handle(TDocStd_Document)& document,
    const aa::Key& key) noexcept {
    DebugCaptureReadEvidence evidence;
    try {
        aa::persistence::Record atlas;
        if (document.IsNull() || document->HasOpenCommand()
            || !aa::persistence::Read(document, key, atlas) || !atlas.value)
            return evidence;
        std::vector<OwnerKey> owners;
        for (const auto& member : atlas.value->definition.members)
            owners.push_back(member.owner);
        aa::Capture members;
        if (!aa::build::CaptureMembers(document, owners, members))
            return evidence;

        auto operation = decal_layer::bake::accounting::MakeOperationOwner();
        CapturedSourceStorage captured;
        gDebugBindingReadEntries = 0;
        gDebugResourceReadEntries = 0;
        evidence.normalCaptured = operation.valid()
            && CaptureSources(document, members, captured, operation.view)
                == Outcome::Prepared
            && gDebugBindingReadEntries != 0
            && gDebugResourceReadEntries != 0;
        evidence.capturedVectorPreflighted = captured.size() != 0
            && captured.values.capacity() >= captured.size();
        evidence.capturedVectorTicketRetained =
            captured.vectorTicket.bytes
                == captured.values.capacity() * sizeof(CapturedSource)
            && captured.vectorTicket.retention
                == decal_layer::bake::accounting::Retention::Retained;
        for (const auto& member : members.members) {
            fi::Definition definition;
            std::vector<std::uint8_t> canonical;
            if (fi::persistence::bindings::Read(document, member.ownerLabel,
                    definition, &canonical)
                    != fi::persistence::bindings::ReadState::Present)
                continue;
            evidence.bindingBytesPreserved =
                !canonical.empty() && !definition.bindings.empty();
            for (const auto& source : captured) {
                fi::persistence::resources::Record record;
                if (!fi::persistence::resources::Read(
                        document, source.fence.resource, record)
                    || !record.value) continue;
                if (source.envelope == record.value->envelope) {
                    evidence.envelopeBytesPreserved = true;
                    evidence.ticketsRetained =
                        source.originalBytesTicket.bytes
                            == source.envelope.originalBytes.size()
                        && source.workingBytesTicket.bytes
                            == source.envelope.workingBytes.size();
                }
            }
        }

        auto deniedOperation =
            decal_layer::bake::accounting::MakeOperationOwner();
        CapturedSourceStorage denied;
        gDebugBindingReadEntries = 0;
        gDebugResourceReadEntries = 0;
        gDebugDenyCaptureReadPreflight = true;
        const Outcome deniedOutcome = CaptureSources(
            document, members, denied, deniedOperation.view);
        gDebugDenyCaptureReadPreflight = false;
        evidence.deniedOverBudget =
            deniedOutcome == Outcome::OverBudget && denied.empty();
        evidence.deniedBeforeBindingRead = gDebugBindingReadEntries == 0;
        evidence.deniedBeforeResourceRead = gDebugResourceReadEntries == 0;
        return evidence;
    } catch (...) {
        gDebugDenyCaptureReadPreflight = false;
        return {};
    }
}

DebugSourceStorageEvidence DebugExerciseE4SourceStorage(
    const Handle(TDocStd_Document)& document,
    const aa::Key& key) noexcept {
    using Site = decal_layer::bake::accounting::FailureSite;
    DebugSourceStorageEvidence evidence;
    try {
        aa::persistence::Record atlas;
        if (document.IsNull() || document->HasOpenCommand()
            || !aa::persistence::Read(document, key, atlas) || !atlas.value)
            return evidence;
        std::vector<OwnerKey> owners;
        for (const auto& member : atlas.value->definition.members)
            owners.push_back(member.owner);
        aa::Capture members;
        if (!aa::build::CaptureMembers(document, owners, members)) return evidence;

        auto operation = decal_layer::bake::accounting::MakeOperationOwner();
        CapturedSourceStorage captured;
        gDebugSourceStorage = {};
        if (!operation.valid()
            || CaptureSources(document, members, captured, operation.view)
                != Outcome::Prepared) return evidence;
        auto tracked = std::find_if(captured.begin(), captured.end(),
            [](const CapturedSource& source) {
                return source.originalBytesTicket.bytes != 0
                    && source.workingBytesTicket.bytes != 0;
            });
        if (tracked == captured.end()) return evidence;
        evidence.captured = true;
        evidence.envelopeBytesEqual =
            tracked->envelope.originalBytes == tracked->envelope.workingBytes;
        evidence.identityValid = Valid(tracked->fence);
        evidence.originalBytes = tracked->originalBytesTicket.bytes;
        evidence.workingBytes = tracked->workingBytesTicket.bytes;

        CapturedSource copied(*tracked);
        evidence.copyChargedSeparately =
            copied.envelope.originalBytes == tracked->envelope.originalBytes
            && copied.envelope.workingBytes == tracked->envelope.workingBytes
            && copied.originalBytesTicket.bytes
                == tracked->originalBytesTicket.bytes
            && copied.workingBytesTicket.bytes
                == tracked->workingBytesTicket.bytes;
        CapturedSource moved(std::move(copied));
        evidence.moveTransferred = copied.originalBytesTicket.bytes == 0
            && copied.workingBytesTicket.bytes == 0
            && moved.originalBytesTicket.bytes == evidence.originalBytes
            && moved.workingBytesTicket.bytes == evidence.workingBytes;

        kernel::SourceStorage decoded;
        evidence.decoded = DecodeCapturedSources(
                captured.values, operation.view, decoded) == Outcome::Prepared
            && !decoded.values.empty()
            && fi::HashFaceImageBytes(
                decoded.values.front().image.rgba, evidence.decodedPixels);
        evidence.vectorBytes = decoded.vectorTicket.bytes;

        const auto deniedCapture = [&](Site site) {
            auto deniedOperation =
                decal_layer::bake::accounting::MakeOperationOwner();
            CapturedSourceStorage refused;
            gDebugSourceStorage = {};
            gDebugSourceStorage.deny = site;
            const auto outcome = CaptureSources(
                document, members, refused, deniedOperation.view);
            const auto index = DebugSourceStorageIndex(site);
            return outcome == Outcome::OverBudget && refused.empty()
                && gDebugSourceStorage.attempts[index] == 1
                && gDebugSourceStorage.entries[index] == 0
                && gDebugSourceStorage.denials[index] == 1;
        };
        evidence.workingDeniedBeforeEntry =
            deniedCapture(Site::SourceWorkingBytes);
        evidence.originalDeniedBeforeEntry =
            deniedCapture(Site::SourceOriginalBytes);

        auto vectorOperation =
            decal_layer::bake::accounting::MakeOperationOwner();
        CapturedSourceStorage vectorCaptured;
        gDebugSourceStorage = {};
        if (CaptureSources(document, members, vectorCaptured,
                vectorOperation.view) == Outcome::Prepared) {
            kernel::SourceStorage refused;
            gDebugSourceStorage = {};
            gDebugSourceStorage.deny = Site::SourceVectorStorage;
            evidence.vectorDeniedBeforeEntry = DecodeCapturedSources(
                    vectorCaptured.values, vectorOperation.view, refused)
                    == Outcome::OverBudget
                && refused.values.empty()
                && gDebugSourceStorage.attempts[3] == 1
                && gDebugSourceStorage.entries[3] == 0
                && gDebugSourceStorage.denials[3] == 1;
        }
        gDebugSourceStorage = {};
        return evidence;
    } catch (...) {
        gDebugSourceStorage = {};
        return {};
    }
}

void DebugDenyNextSourceVectorReservation() noexcept {
    gDebugSourceStorage = {};
    gDebugPrivateSourceVectorAttempts.store(0, std::memory_order_relaxed);
    gDebugPrivateSourceVectorEntries.store(0, std::memory_order_relaxed);
    gDebugPrivateSourceVectorDenials.store(0, std::memory_order_relaxed);
    gDebugDenyNextPrivateSourceVectorReservation.store(
        true, std::memory_order_release);
}

std::array<std::size_t, 3>
DebugTakeSourceVectorReservationObservation() noexcept {
    const std::array<std::size_t, 3> result{{
        gDebugPrivateSourceVectorAttempts.exchange(0, std::memory_order_acq_rel),
        gDebugPrivateSourceVectorEntries.exchange(0, std::memory_order_acq_rel),
        gDebugPrivateSourceVectorDenials.exchange(0, std::memory_order_acq_rel),
    }};
    gDebugDenyNextPrivateSourceVectorReservation.store(
        false, std::memory_order_release);
    gDebugSourceStorage = {};
    return result;
}

DebugOrdinaryStagingEvidence DebugExerciseE4OrdinaryStaging(
    const Handle(TDocStd_Document)& document,
    const aa::Key& key) noexcept {
    DebugOrdinaryStagingEvidence evidence;
    const Standard_Integer undosBefore = document.IsNull()
        ? -1 : document->GetAvailableUndos();
    const auto abort = [&]() noexcept {
        if (!document.IsNull() && document->HasOpenCommand())
            document->AbortCommand();
    };
    try {
        if (document.IsNull() || document->HasOpenCommand()) return evidence;

        document->NewCommand();
        Staging commitStaging;
        const Outcome commitPrepared = Prepare(commitStaging, document, key);
        evidence.prepareCommitPrepared = commitPrepared == Outcome::Prepared;
        evidence.prepareCommitOverlapCharged =
            evidence.prepareCommitPrepared
            && OrdinaryStagingOverlapCharged(commitStaging);
        gDebugOrdinaryTraceArmed = evidence.prepareCommitPrepared;
        gDebugExpectedOrdinaryContext = commitStaging.operation.view.context;
        gDebugOrdinaryCommitOverlap = false;
        const Outcome committed = evidence.prepareCommitPrepared
            ? Commit(commitStaging, document) : commitPrepared;
        gDebugOrdinaryTraceArmed = false;
        gDebugExpectedOrdinaryContext = nullptr;
        evidence.operationStableAtCommit = gDebugOrdinaryCommitOverlap;
        evidence.normalCommitted = committed == Outcome::Committed;
        Cancel(commitStaging);
        abort();

        document->NewCommand();
        Staging cancelStaging;
        const Outcome cancelPrepared = Prepare(cancelStaging, document, key);
        evidence.prepareCancelPrepared = cancelPrepared == Outcome::Prepared;
        evidence.prepareCancelOverlapCharged =
            evidence.prepareCancelPrepared
            && OrdinaryStagingOverlapCharged(cancelStaging);
        Cancel(cancelStaging);
        evidence.cancelReleased = !cancelStaging.operation.valid()
            && cancelStaging.priorAtlasBytes.empty()
            && cancelStaging.priorBakeResources.empty()
            && cancelStaging.observedMembers.empty()
            && cancelStaging.capturedSources.empty()
            && cancelStaging.priorAtlasBytesTicket.bytes == 0
            && cancelStaging.priorBakeResourcesTicket.bytes == 0
            && cancelStaging.observedMembersTicket.bytes == 0;
        abort();

        document->NewCommand();
        Staging deniedStaging;
        DebugDenyCaptureSourcesAtOrdinal(2);
        const Outcome denialPrepared = Prepare(deniedStaging, document, key);
        evidence.denialPrepared = denialPrepared == Outcome::Prepared;
        evidence.denialOverlapCharged = evidence.denialPrepared
            && OrdinaryStagingOverlapCharged(deniedStaging);
        gDebugAdoptionEntries = 0;
        gDebugPersistenceEntries = 0;
        const Outcome denied = evidence.denialPrepared
            ? Commit(deniedStaging, document) : denialPrepared;
        DebugClearCaptureSourcesDenial();
        evidence.denialOverBudget = denied == Outcome::OverBudget;
        evidence.noAdoptionEntry = gDebugAdoptionEntries == 0;
        evidence.noPersistenceEntry = gDebugPersistenceEntries == 0;
        Cancel(deniedStaging);
        abort();
        evidence.historyUnchanged =
            document->GetAvailableUndos() == undosBefore;
        return evidence;
    } catch (...) {
        gDebugOrdinaryTraceArmed = false;
        gDebugExpectedOrdinaryContext = nullptr;
        DebugClearCaptureSourcesDenial();
        abort();
        return {};
    }
}
#endif

Outcome Prepare(Staging& staging, const Handle(TDocStd_Document)& document,
                const aa::Key& key) noexcept {
    staging = {};
    using namespace decal_layer::bake::accounting;
    FailureSite activeAllocation = FailureSite::AtlasReadScratch;
    try {
        staging.operation = decal_layer::bake::accounting::MakeOperationOwner();
        if (!staging.operation.valid()) return Outcome::OverBudget;
        if (document.IsNull() || !document->HasOpenCommand()) return Outcome::Busy;
        OrdinaryReadAdmission atlasRead;
        const Outcome atlasPreflight = PreflightAtlasRead(
            document, staging.operation.view, atlasRead);
        if (atlasPreflight != Outcome::Prepared) return atlasPreflight;
        aa::persistence::Record prior;
        if (!aa::persistence::Read(document, key, prior))
            return HasAtlasOwnerMismatch(document, key)
                ? Outcome::OwnerMismatch : Outcome::Malformed;
        if (!prior.value) return Outcome::Absent;
        activeAllocation = FailureSite::PriorAtlasBytes;
        if (!AcquireOrdinaryStorage(staging.priorAtlasBytesTicket,
                staging.operation.view, StorageDimension::EncodedTexture,
                prior.value->bytes.size(), Retention::Retained,
                activeAllocation)) return Outcome::OverBudget;
        staging.priorAtlasBytes = prior.value->bytes;
        if (staging.priorAtlasBytes.capacity()
                > staging.priorAtlasBytesTicket.bytes) {
            staging.operation.view.AllocationFailed(activeAllocation);
            return Outcome::OverBudget;
        }
        const Outcome classified = ClassifyMembers(document, prior.value->definition);
        if (classified != Outcome::Prepared) return classified;
        std::vector<aa::Member> currentMembers;
        Ticket currentMembersTicket;
        activeAllocation = FailureSite::OrdinaryMemberStorage;
        if (!AcquireOrdinaryStorage(currentMembersTicket,
                staging.operation.view, StorageDimension::PrivateStorage,
                prior.value->definition.members.size() * sizeof(aa::Member),
                Retention::Scratch, activeAllocation))
            return Outcome::OverBudget;
        currentMembers.reserve(prior.value->definition.members.size());
        if (!aa::build::ObserveMembers(document, prior.value->definition.members,
                                       currentMembers)
            || !aa::Current(prior.value->definition, currentMembers))
            return Outcome::StaleSource;
        std::vector<OwnerKey> owners;
        Ticket ownersTicket;
        activeAllocation = FailureSite::OrdinaryOwnerStorage;
        if (!AcquireOrdinaryStorage(ownersTicket, staging.operation.view,
                StorageDimension::PrivateStorage,
                prior.value->definition.members.size() * sizeof(OwnerKey),
                Retention::Scratch, activeAllocation))
            return Outcome::OverBudget;
        owners.reserve(prior.value->definition.members.size());
        for (const auto& member : prior.value->definition.members) owners.push_back(member.owner);
        aa::Capture capture;
        Ticket captureTicket;
        activeAllocation = FailureSite::OrdinaryCaptureStorage;
        if (!AcquireOrdinaryStorage(captureTicket, staging.operation.view,
                StorageDimension::PrivateStorage,
                prior.value->definition.members.size()
                    * sizeof(aa::MemberCapture),
                Retention::Scratch, activeAllocation))
            return Outcome::OverBudget;
        if (!aa::build::CaptureMembers(document, owners, capture)) return Outcome::StaleSource;
        const Outcome captured = CaptureSources(document, capture,
            staging.capturedSources, staging.operation.view);
        if (captured != Outcome::Prepared) return captured;
        // A malformed persisted SYEB/1 record refuses before any bake work.
        OrdinaryReadAdmission bakeRead;
        const Outcome bakePreflight = PreflightBakeRead(
            document, staging.operation.view, bakeRead);
        if (bakePreflight != Outcome::Prepared) return bakePreflight;
        persistence::Record existing;
        const auto state = persistence::Read(document, key, existing);
        if (state == persistence::ReadState::Malformed) return Outcome::Malformed;
        if (state == persistence::ReadState::Present) {
            activeAllocation = FailureSite::PriorBakeResources;
            if (!AcquireOrdinaryStorage(staging.priorBakeResourcesTicket,
                    staging.operation.view, StorageDimension::PrivateStorage,
                    existing.definition.resources.size()
                        * sizeof(BakedResource),
                    Retention::Retained, activeAllocation))
                return Outcome::OverBudget;
            staging.priorBakeResources = existing.definition.resources;
            if (staging.priorBakeResources.capacity()
                    * sizeof(BakedResource)
                > staging.priorBakeResourcesTicket.bytes) {
                staging.operation.view.AllocationFailed(activeAllocation);
                return Outcome::OverBudget;
            }
        }
        // Aggregate decoded budget preflight: distinct baked roles at the
        // fenced atlas resolution must fit the frozen 128 MiB cap.
        std::vector<Role> roles;
        Ticket rolesTicket;
        activeAllocation = FailureSite::OrdinaryRoleStorage;
        if (!AcquireOrdinaryStorage(rolesTicket, staging.operation.view,
                StorageDimension::PrivateStorage,
                staging.capturedSources.size() * sizeof(Role),
                Retention::Scratch, activeAllocation))
            return Outcome::OverBudget;
        roles.reserve(staging.capturedSources.size());
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
        activeAllocation = FailureSite::OrdinaryObservedStorage;
        if (!AcquireOrdinaryStorage(staging.observedMembersTicket,
                staging.operation.view, StorageDimension::PrivateStorage,
                staging.atlas.members.size() * sizeof(aa::Member),
                Retention::Retained, activeAllocation))
            return Outcome::OverBudget;
        staging.observedMembers.reserve(staging.atlas.members.size());
        if (!aa::build::ObserveMembers(document, staging.atlas.members,
                                       staging.observedMembers)) return Outcome::StaleSource;
        kernel::SourceStorage sources;
        const auto decoded = DecodeCapturedSources(
            staging.capturedSources.values, staging.operation.view, sources);
        if (decoded != Outcome::Prepared) return decoded;
        if (!kernel::Bake(staging.atlas, layout, sources.values, staging.outputs,
                          staging.evidence)) return Outcome::Refused;
        staging.bake.key = key; staging.bake.layoutProof = staging.atlas.layoutProof;
        for (const auto& source : staging.capturedSources)
            staging.bake.bindings.push_back(source.fence);
        for (const auto& output : staging.outputs)
            staging.bake.resources.push_back(output.descriptor);
        if (!BindBakeProof(staging.bake)) return Outcome::Malformed;
        std::vector<std::uint8_t> atlasBytes;
        Ticket canonicalTicket;
        activeAllocation = FailureSite::OrdinaryCanonicalEncoding;
        if (!AcquireOrdinaryStorage(canonicalTicket, staging.operation.view,
                StorageDimension::EncodedTexture, aa::kMaximumBytes,
                Retention::Scratch, activeAllocation))
            return Outcome::OverBudget;
        if (!aa::Encode(staging.atlas, atlasBytes)) return Outcome::Malformed;
        staging.unchanged = state == persistence::ReadState::Present
            && existing.definition == staging.bake
            && prior.value->bytes == atlasBytes;
        return Outcome::Prepared;
    } catch (const std::bad_alloc&) {
        staging.operation.view.AllocationFailed(activeAllocation);
        staging = {};
        return Outcome::OverBudget;
    } catch (...) { staging = {}; return Outcome::Malformed; }
}

Outcome Commit(Staging& staging, const Handle(TDocStd_Document)& document) noexcept {
    using namespace decal_layer::bake::accounting;
    FailureSite activeAllocation = FailureSite::OrdinaryObservedStorage;
    try {
        if (document.IsNull() || !document->HasOpenCommand()) {
            staging = {}; return Outcome::Busy;
        }
        // Commit locals borrow the staging view. Retain the same concrete
        // operation until every local ticket has unwound, including paths
        // where resetStaging() clears the caller-owned Staging first.
        Owner commitOwner = staging.operation;
        if (!commitOwner.valid()) {
            staging = {}; return Outcome::OverBudget;
        }
        Ticket membersTicket;
        if (!AcquireOrdinaryStorage(membersTicket, staging.operation.view,
                StorageDimension::PrivateStorage,
                staging.atlas.members.size() * sizeof(aa::Member),
                Retention::Scratch, activeAllocation)) {
            staging = {}; return Outcome::OverBudget;
        }
        std::vector<aa::Member> members;
        members.reserve(staging.atlas.members.size());
        if (!aa::build::ObserveMembers(document, staging.atlas.members, members)
            || members != staging.observedMembers) { staging = {}; return Outcome::StaleSource; }
        aa::Capture capture;
        std::vector<OwnerKey> owners;
        Ticket ownersTicket;
        activeAllocation = FailureSite::OrdinaryOwnerStorage;
        if (!AcquireOrdinaryStorage(ownersTicket, staging.operation.view,
                StorageDimension::PrivateStorage,
                staging.atlas.members.size() * sizeof(OwnerKey),
                Retention::Scratch, activeAllocation)) {
            staging = {}; return Outcome::OverBudget;
        }
        owners.reserve(staging.atlas.members.size());
        for (const auto& member : staging.atlas.members) owners.push_back(member.owner);
        Ticket captureTicket;
        activeAllocation = FailureSite::OrdinaryCaptureStorage;
        if (!AcquireOrdinaryStorage(captureTicket, staging.operation.view,
                StorageDimension::PrivateStorage,
                staging.atlas.members.size() * sizeof(aa::MemberCapture),
                Retention::Scratch, activeAllocation)) {
            staging = {}; return Outcome::OverBudget;
        }
        Ticket atlasCanonicalTicket;
        activeAllocation = FailureSite::OrdinaryCanonicalEncoding;
        if (!AcquireOrdinaryStorage(atlasCanonicalTicket,
                staging.operation.view, StorageDimension::EncodedTexture,
                aa::kMaximumBytes, Retention::Scratch, activeAllocation)) {
            staging = {}; return Outcome::OverBudget;
        }
        OrdinaryReadAdmission bakeRead;
        const Outcome bakePreflight = PreflightBakeRead(
            document, staging.operation.view, bakeRead);
        if (bakePreflight != Outcome::Prepared) {
            staging = {}; return bakePreflight;
        }
        Ticket recordTicket;
        activeAllocation = FailureSite::OrdinaryRecordStorage;
        if (!AcquireOrdinaryStorage(recordTicket, staging.operation.view,
                StorageDimension::PrivateStorage, kMaximumBytes,
                Retention::Scratch, activeAllocation)) {
            staging = {}; return Outcome::OverBudget;
        }
        CapturedSourceStorage sources;
        const auto resetStaging = [&]() noexcept {
            // The local recapture tickets borrow staging.operation. Release
            // them before the staging reset destroys that owner.
            sources.reset();
            staging = {};
        };
        if (!aa::build::CaptureMembers(document, owners, capture)) {
            staging = {}; return Outcome::StaleSource;
        }
#ifdef DEBUG
        if (gDebugOrdinaryTraceArmed) {
            gDebugOrdinaryCommitOverlap =
                gDebugExpectedOrdinaryContext == staging.operation.view.context
                && OrdinaryStagingOverlapCharged(staging)
                && membersTicket.bytes != 0 && ownersTicket.bytes != 0
                && captureTicket.bytes != 0
                && atlasCanonicalTicket.bytes != 0
                && recordTicket.bytes != 0;
        }
#endif
        const Outcome recaptured = CaptureSources(document, capture, sources,
            staging.operation.view);
        if (recaptured == Outcome::OverBudget) {
            resetStaging(); return Outcome::OverBudget;
        }
        if (recaptured == Outcome::MissingResource || recaptured == Outcome::ForeignResource
            || recaptured == Outcome::Malformed) {
            resetStaging(); return recaptured;
        }
        if (recaptured != Outcome::Prepared
            || !SameCapture(staging.capturedSources.values, sources.values)) {
            resetStaging(); return Outcome::StaleBinding;
        }
        if (staging.unchanged) { resetStaging(); return Outcome::Committed; }
        // The bake never silently rewrites the atlas: the rebuilt SYEA/1 is
        // recommitted only when its canonical bytes actually changed.
        std::vector<std::uint8_t> atlasBytes;
        if (!aa::Encode(staging.atlas, atlasBytes)) {
            resetStaging(); return Outcome::Malformed;
        }
        if (atlasBytes != staging.priorAtlasBytes) {
            aa::owner::Staging atlasStaging;
            auto atlasOutcome = aa::owner::Prepare(atlasStaging, document, staging.atlas,
                                                   staging.assignments, members);
            if (atlasOutcome != aa::owner::Outcome::Prepared) {
                resetStaging();
                return atlasOutcome == aa::owner::Outcome::StaleSource
                    ? Outcome::StaleSource : Outcome::PersistenceFailure;
            }
            atlasOutcome = aa::owner::Commit(atlasStaging, document, members);
            if (atlasOutcome != aa::owner::Outcome::Committed) {
                aa::owner::Cancel(atlasStaging); resetStaging();
                return Outcome::PersistenceFailure;
            }
        }
#ifdef DEBUG
        ++gDebugAdoptionEntries;
#endif
        const Outcome adopted = AdoptBakedResources(document, staging.outputs);
        if (adopted != Outcome::Committed) {
            resetStaging(); return adopted;
        }
        // A re-bake replaces, rather than accumulates, this atlas's derivative
        // resources. Remove superseded identities in the same OCAF command,
        // but retain any identity still named by another SYEB record or a
        // committed SYFI binding.
        std::vector<persistence::Record> bakeRecords;
        if (!persistence::ReadAll(document, bakeRecords)) {
            resetStaging(); return Outcome::PersistenceFailure;
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
                resetStaging(); return Outcome::PersistenceFailure;
            }
        }
#ifdef DEBUG
        ++gDebugPersistenceEntries;
#endif
        if (!persistence::StageCommitted(document, staging.bake)) {
            resetStaging(); return Outcome::PersistenceFailure;
        }
        resetStaging(); return Outcome::Committed;
    } catch (const std::bad_alloc&) {
        staging.operation.view.AllocationFailed(activeAllocation);
        staging = {};
        return Outcome::OverBudget;
    } catch (...) { staging = {}; return Outcome::PersistenceFailure; }
}

void Cancel(Staging& staging) noexcept { staging = {}; }

Outcome CaptureForExport(ExportCapture& capture,
                         const Handle(TDocStd_Document)& document,
                         const aa::Key& key) noexcept {
    capture = {};
    try {
        capture.operation = decal_layer::bake::accounting::MakeOperationOwner();
        if (!capture.operation.valid()) return Outcome::OverBudget;
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
            document, capture.members, capture.sources,
            capture.operation.view);
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
        kernel::SourceStorage sources;
        const auto decoded = DecodeCapturedSources(
            capture.sources.values, capture.operation.view, sources);
        if (decoded != Outcome::Prepared) return decoded;
        output.atlas = finalAtlas;
        output.assignments = assignments;
        if (!kernel::Bake(output.atlas, layout, sources.values,
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
        kernel::SourceStorage sources;
        const auto decoded = DecodeCapturedSources(
            capture.sources, capture.operation, sources);
        if (decoded != Outcome::Prepared) {
            diagnosis = "transient-source-decode";
            return decoded;
        }
        output.atlas = std::move(atlas);
        output.assignments = std::move(assignments);
        if (!kernel::Bake(output.atlas, layout, sources.values,
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
        auto operation = decal_layer::bake::accounting::MakeOperationOwner();
        if (!operation.valid()) return Outcome::OverBudget;
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
        CapturedSourceStorage sources;
        const Outcome captured = CaptureSources(document, capture, sources,
            operation.view);
        if (captured == Outcome::OverBudget) return Outcome::OverBudget;
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
