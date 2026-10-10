#import "Core3DAttachmentRigOpening.h"

#import "Core3DViewController.h"
#import "Core3DViewController+ExportManager.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/AttachmentRigMetadataPersistence.hxx"
#include "../OCCTKit/CylindricalCutDefinition.hxx"
#include "../OCCTKit/NativeOpeningContext.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/RetainedBooleanProgram.hxx"
#include "Core3DViewer.h"

#include <CommonCrypto/CommonDigest.h>
#include <TDF_Tool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

NSErrorDomain const Core3DAttachmentRigErrorDomain = @"Core3DAttachmentRigErrorDomain";

namespace core3d::attachment_rig {
namespace {
constexpr std::size_t kMaxNameUTF8Bytes = kMaxDisplayNameUTF8Bytes;
constexpr std::size_t kMaxReceiptBytes = kMaxSemanticReceiptBytes;
constexpr std::size_t kUUIDBytes = 16;
constexpr std::size_t kOwnerBytes = 48;

bool Fail(std::string& refusal, const char *reason) noexcept {
    try { refusal = reason; } catch (...) {}
    return false;
}

bool UTF8(const std::string& value, bool nonempty, std::size_t maximum) noexcept {
    if ((nonempty && value.empty()) || value.size() > maximum) return false;
    const auto *p = reinterpret_cast<const unsigned char *>(value.data());
    std::size_t i = 0;
    while (i < value.size()) {
        const unsigned char a = p[i++];
        if (a == 0) return false;
        if (a < 0x80) continue;
        unsigned need = 0; std::uint32_t cp = 0;
        if ((a & 0xe0) == 0xc0) { need = 1; cp = a & 0x1f; if (cp < 2) return false; }
        else if ((a & 0xf0) == 0xe0) { need = 2; cp = a & 0x0f; }
        else if ((a & 0xf8) == 0xf0) { need = 3; cp = a & 0x07; }
        else return false;
        if (need > value.size() - i) return false;
        for (unsigned n = 0; n < need; ++n) {
            const unsigned char b = p[i++]; if ((b & 0xc0) != 0x80) return false;
            cp = (cp << 6) | (b & 0x3f);
        }
        if ((need == 2 && cp < 0x800) || (need == 3 && cp < 0x10000)
            || (cp >= 0xd800 && cp <= 0xdfff) || cp > 0x10ffff) return false;
    }
    return true;
}

bool UTF8Encoding(const std::string& value) noexcept {
    const auto *p = reinterpret_cast<const unsigned char *>(value.data());
    std::size_t i = 0;
    while (i < value.size()) {
        const unsigned char a = p[i++];
        if (a < 0x80) continue;
        unsigned need = 0; std::uint32_t cp = 0;
        if ((a & 0xe0) == 0xc0) { need = 1; cp = a & 0x1f; if (cp < 2) return false; }
        else if ((a & 0xf0) == 0xe0) { need = 2; cp = a & 0x0f; }
        else if ((a & 0xf8) == 0xf0) { need = 3; cp = a & 0x07; }
        else return false;
        if (need > value.size() - i) return false;
        for (unsigned n = 0; n < need; ++n) {
            const unsigned char b = p[i++]; if ((b & 0xc0) != 0x80) return false;
            cp = (cp << 6) | (b & 0x3f);
        }
        if ((need == 2 && cp < 0x800) || (need == 3 && cp < 0x10000)
            || (cp >= 0xd800 && cp <= 0xdfff) || cp > 0x10ffff) return false;
    }
    return true;
}

bool BinaryID(const std::string& value, std::size_t length) noexcept {
    return value.size() == length
        && std::any_of(value.begin(), value.end(), [](unsigned char byte) { return byte != 0; });
}

double Dot(const std::array<double, 3>& a, const std::array<double, 3>& b) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

bool RigidFrame(const FrameMM& frame) noexcept {
    for (double value : frame.originMM)
        if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<double>::max() / 2.0) return false;
    for (const auto *axis : {&frame.x, &frame.y, &frame.z})
        for (double value : *axis) if (!std::isfinite(value)) return false;
    const double xx = Dot(frame.x, frame.x), yy = Dot(frame.y, frame.y), zz = Dot(frame.z, frame.z);
    const double xy = Dot(frame.x, frame.y), xz = Dot(frame.x, frame.z), yz = Dot(frame.y, frame.z);
    const double determinant = frame.x[0] * (frame.y[1] * frame.z[2] - frame.y[2] * frame.z[1])
        - frame.y[0] * (frame.x[1] * frame.z[2] - frame.x[2] * frame.z[1])
        + frame.z[0] * (frame.x[1] * frame.y[2] - frame.x[2] * frame.y[1]);
    for (double value : {xx, yy, zz, xy, xz, yz, determinant}) if (!std::isfinite(value)) return false;
    return std::abs(xx - 1) <= kFrameTolerance && std::abs(yy - 1) <= kFrameTolerance
        && std::abs(zz - 1) <= kFrameTolerance && std::abs(xy) <= kFrameTolerance
        && std::abs(xz) <= kFrameTolerance && std::abs(yz) <= kFrameTolerance
        && std::abs(determinant - 1) <= kFrameTolerance;
}

struct Writer final {
    std::vector<std::uint8_t> bytes;
    bool good = true;
    void raw(const void *source, std::size_t count) {
        if (!good || count > kMaxABR1Bytes - std::min(bytes.size(), kMaxABR1Bytes)) { good = false; return; }
        const auto *first = static_cast<const std::uint8_t *>(source);
        bytes.insert(bytes.end(), first, first + count);
    }
    void u8(std::uint8_t value) { raw(&value, 1); }
    void u32(std::uint32_t value) {
        std::uint8_t b[4]; for (unsigned i = 0; i < 4; ++i) b[i] = std::uint8_t(value >> (8 * i)); raw(b, 4);
    }
    void u64(std::uint64_t value) {
        std::uint8_t b[8]; for (unsigned i = 0; i < 8; ++i) b[i] = std::uint8_t(value >> (8 * i)); raw(b, 8);
    }
    void number(double value) { std::uint64_t bits = 0; std::memcpy(&bits, &value, 8); u64(bits); }
    void text(const std::string& value) { u32(std::uint32_t(value.size())); raw(value.data(), value.size()); }
    void frame(const FrameMM& value) {
        for (double number : value.originMM) this->number(number);
        for (double number : value.x) this->number(number);
        for (double number : value.y) this->number(number);
        for (double number : value.z) this->number(number);
    }
};

struct Reader final {
    const std::uint8_t *bytes = nullptr; std::size_t size = 0, at = 0;
    bool raw(void *output, std::size_t count) {
        if (count > size - std::min(at, size)) return false;
        if (count) std::memcpy(output, bytes + at, count); at += count; return true;
    }
    bool u8(std::uint8_t& value) { return raw(&value, 1); }
    bool u32(std::uint32_t& value) {
        std::uint8_t b[4]; if (!raw(b, 4)) return false;
        value = std::uint32_t(b[0]) | std::uint32_t(b[1]) << 8 | std::uint32_t(b[2]) << 16 | std::uint32_t(b[3]) << 24;
        return true;
    }
    bool u64(std::uint64_t& value) {
        std::uint8_t b[8]; if (!raw(b, 8)) return false; value = 0;
        for (unsigned i = 0; i < 8; ++i) value |= std::uint64_t(b[i]) << (8 * i); return true;
    }
    bool number(double& value) { std::uint64_t bits = 0; if (!u64(bits)) return false; std::memcpy(&value, &bits, 8); return true; }
    bool text(std::string& value, std::size_t maximum) {
        std::uint32_t count = 0; if (!u32(count) || count > maximum || count > size - std::min(at, size)) return false;
        std::string temporary(reinterpret_cast<const char *>(bytes + at), count); at += count; value.swap(temporary); return true;
    }
    bool fixed(std::string& value, std::size_t count) {
        if (count > size - std::min(at, size)) return false;
        std::string temporary(reinterpret_cast<const char *>(bytes + at), count); at += count; value.swap(temporary); return true;
    }
    bool frame(FrameMM& value) {
        for (double& number : value.originMM) if (!this->number(number)) return false;
        for (double& number : value.x) if (!this->number(number)) return false;
        for (double& number : value.y) if (!this->number(number)) return false;
        for (double& number : value.z) if (!this->number(number)) return false;
        return true;
    }
};
} // namespace

bool Validate(const Metadata& metadata, std::string& refusal) noexcept {
    try {
        if (metadata.schemaVersion != kSchemaVersion || metadata.targetProfile.schemaVersion != kSchemaVersion)
            return Fail(refusal, "unknown schema");
        const auto& profile = metadata.targetProfile;
        if (!UTF8(profile.profileID, true, kMaxNameUTF8Bytes)
            || !UTF8(profile.profileVersion, true, kMaxNameUTF8Bytes)) return Fail(refusal, "invalid local policy identity");
        if (profile.externallyVerified) return Fail(refusal, "external verification is not available in S1");
        if (profile.requiredRoles.size() > kMaxRequiredRoles) return Fail(refusal, "too many required roles");
        std::set<std::string> required;
        for (const auto& role : profile.requiredRoles)
            if (!UTF8(role, true, kMaxRoleUTF8Bytes) || !required.insert(role).second)
                return Fail(refusal, "invalid or duplicate required role");
        if (metadata.attachments.size() > kMaxAttachments) return Fail(refusal, "too many attachments");
        if (metadata.joints.size() > kMaxJoints) return Fail(refusal, "too many joints");
        std::set<std::string> attachmentIDs, roles;
        std::map<std::string, std::size_t> roleCounts;
        for (const auto& attachment : metadata.attachments) {
            if (!BinaryID(attachment.attachmentID, kUUIDBytes)
                || !attachmentIDs.insert(attachment.attachmentID).second) return Fail(refusal, "invalid or duplicate attachment id");
            if (!UTF8(attachment.displayName, false, kMaxNameUTF8Bytes)) return Fail(refusal, "invalid attachment display name");
            if (!UTF8(attachment.role, true, kMaxRoleUTF8Bytes)) return Fail(refusal, "invalid attachment role");
            if (!roles.insert(attachment.role).second) return Fail(refusal, "duplicate role is not admitted by the local policy");
            ++roleCounts[attachment.role];
            if (!RigidFrame(attachment.frameMM)) return Fail(refusal, "attachment frame is not finite right-handed orthonormal");
            if (attachment.rigid.has_value() == attachment.surface.has_value()) return Fail(refusal, "exactly one attachment binding is required");
            const std::string& owner = attachment.rigid ? attachment.rigid->owner : attachment.surface->owner;
            if (!BinaryID(owner, kOwnerBytes)
                || !BinaryID(owner.substr(0, 16), 16) || !BinaryID(owner.substr(16, 16), 16)
                || !BinaryID(owner.substr(32, 16), 16)) return Fail(refusal, "invalid owner tuple");
            if (attachment.surface) {
                if (attachment.surface->faceSelector.empty() || attachment.surface->faceSelector.size() > kMaxReceiptBytes
                    || !RigidFrame(attachment.surface->localFrameMM)) return Fail(refusal, "invalid semantic surface receipt");
                return Fail(refusal, "surface binding requires S3 admission");
            }
        }
        for (const auto& role : required)
            if (roleCounts[role] != 1) return Fail(refusal, "required role must occur exactly once");
        std::map<std::string, const Joint *> joints;
        for (const auto& joint : metadata.joints) {
            if (!BinaryID(joint.jointID, kUUIDBytes) || !joints.emplace(joint.jointID, &joint).second)
                return Fail(refusal, "invalid or duplicate joint id");
            if (!UTF8(joint.displayName, false, kMaxNameUTF8Bytes) || !RigidFrame(joint.bindTransformMM))
                return Fail(refusal, "invalid joint value");
            if (joint.parentJointID && (!BinaryID(*joint.parentJointID, kUUIDBytes) || *joint.parentJointID == joint.jointID))
                return Fail(refusal, "invalid or self joint parent");
        }
        for (const auto& item : joints) {
            std::set<std::string> path; const Joint *joint = item.second; std::size_t depth = 0;
            while (joint->parentJointID) {
                if (++depth > kMaxHierarchyDepth || !path.insert(joint->jointID).second) return Fail(refusal, "cyclic or over-depth hierarchy");
                const auto parent = joints.find(*joint->parentJointID);
                if (parent == joints.end()) return Fail(refusal, "missing joint parent");
                joint = parent->second;
            }
        }
        refusal.clear(); return true;
    } catch (...) { return Fail(refusal, "validation exception"); }
}

std::vector<ReattachReceipt> ReattachSurfaceBindings(const Metadata& metadata, std::string& refusal) noexcept {
    std::vector<ReattachReceipt> receipts;
    if (!Validate(metadata, refusal)) return receipts;
    try {
        receipts.reserve(metadata.attachments.size());
        for (const auto& attachment : metadata.attachments)
            receipts.push_back({attachment.attachmentID, ReattachOutcome::Current, "rigid binding remains current"});
    } catch (...) { receipts.clear(); Fail(refusal, "reattach allocation exception"); }
    return receipts;
}

bool EncodeABR1(const Metadata& metadata, std::vector<std::uint8_t>& output, std::string& refusal) noexcept {
    try {
        if (!Validate(metadata, refusal)) return false;
        Writer payload;
        payload.text(metadata.targetProfile.profileID); payload.text(metadata.targetProfile.profileVersion);
        auto roles = metadata.targetProfile.requiredRoles; std::sort(roles.begin(), roles.end());
        payload.u32(std::uint32_t(roles.size())); for (const auto& role : roles) payload.text(role);
        payload.u8(metadata.targetProfile.externallyVerified ? 1 : 0);
        std::vector<const Attachment *> attachments; attachments.reserve(metadata.attachments.size());
        for (const auto& attachment : metadata.attachments) attachments.push_back(&attachment);
        std::sort(attachments.begin(), attachments.end(), [](auto *a, auto *b) { return a->attachmentID < b->attachmentID; });
        payload.u32(std::uint32_t(attachments.size()));
        for (const auto *attachment : attachments) {
            payload.raw(attachment->attachmentID.data(), kUUIDBytes); payload.text(attachment->displayName);
            payload.text(attachment->role); payload.frame(attachment->frameMM);
            if (attachment->rigid) {
                payload.u8(0); payload.raw(attachment->rigid->owner.data(), kOwnerBytes);
            } else {
                payload.u8(1); payload.raw(attachment->surface->owner.data(), kOwnerBytes);
                payload.text(attachment->surface->faceSelector); payload.frame(attachment->surface->localFrameMM);
            }
        }
        std::vector<const Joint *> joints; joints.reserve(metadata.joints.size());
        for (const auto& joint : metadata.joints) joints.push_back(&joint);
        std::sort(joints.begin(), joints.end(), [](auto *a, auto *b) { return a->jointID < b->jointID; });
        payload.u32(std::uint32_t(joints.size()));
        for (const auto *joint : joints) {
            payload.raw(joint->jointID.data(), kUUIDBytes); payload.text(joint->displayName);
            payload.u8(joint->parentJointID ? 1 : 0);
            if (joint->parentJointID) payload.raw(joint->parentJointID->data(), kUUIDBytes);
            payload.frame(joint->bindTransformMM);
        }
        if (!payload.good || payload.bytes.size() > UINT32_MAX) return Fail(refusal, "ABR1 payload exceeds bound");
        Writer frame; const std::uint8_t magic[4] = {'A','B','R','1'}; frame.raw(magic, 4);
        frame.u32(kSchemaVersion); frame.u32(std::uint32_t(payload.bytes.size()));
        frame.raw(payload.bytes.data(), payload.bytes.size());
        std::array<unsigned char, CC_SHA256_DIGEST_LENGTH> digest{};
        if (!frame.good || frame.bytes.size() + digest.size() > kMaxABR1Bytes
            || !CC_SHA256(frame.bytes.data(), CC_LONG(frame.bytes.size()), digest.data())) return Fail(refusal, "ABR1 checksum failure");
        frame.raw(digest.data(), digest.size()); if (!frame.good) return Fail(refusal, "ABR1 frame exceeds bound");
        output = std::move(frame.bytes); refusal.clear(); return true;
    } catch (...) { return Fail(refusal, "ABR1 encode exception"); }
}

enum class DecodeRefusalClass { Encoding, Metadata };

static bool DecodeABR1Classified(const std::vector<std::uint8_t>& bytes,
                                 Metadata& output,
                                 std::string& refusal,
                                 DecodeRefusalClass& refusalClass) noexcept {
    refusalClass = DecodeRefusalClass::Encoding;
    try {
        if (bytes.size() < 44 || bytes.size() > kMaxABR1Bytes) return Fail(refusal, "ABR1 length outside bound");
        Reader header{bytes.data(), bytes.size(), 0}; std::uint8_t magic[4]; std::uint32_t schema = 0, payloadLength = 0;
        if (!header.raw(magic, 4) || std::memcmp(magic, "ABR1", 4) != 0 || !header.u32(schema) || !header.u32(payloadLength))
            return Fail(refusal, "invalid ABR1 header");
        if (schema != kSchemaVersion) return Fail(refusal, "unknown ABR1 schema");
        if (std::size_t(payloadLength) != bytes.size() - 44) return Fail(refusal, "ABR1 length mismatch or trailing bytes");
        std::array<unsigned char, CC_SHA256_DIGEST_LENGTH> digest{};
        if (!CC_SHA256(bytes.data(), CC_LONG(12 + payloadLength), digest.data())
            || std::memcmp(digest.data(), bytes.data() + 12 + payloadLength, digest.size()) != 0)
            return Fail(refusal, "ABR1 checksum mismatch");
        Reader reader{bytes.data() + 12, payloadLength, 0}; Metadata temporary; temporary.schemaVersion = schema;
        temporary.targetProfile.schemaVersion = schema;
        if (!reader.text(temporary.targetProfile.profileID, kMaxNameUTF8Bytes)
            || !reader.text(temporary.targetProfile.profileVersion, kMaxNameUTF8Bytes)) return Fail(refusal, "truncated policy");
        if (!UTF8Encoding(temporary.targetProfile.profileID)
            || !UTF8Encoding(temporary.targetProfile.profileVersion)) return Fail(refusal, "invalid policy UTF-8 encoding");
        std::uint32_t count = 0; if (!reader.u32(count) || count > kMaxRequiredRoles) return Fail(refusal, "invalid required-role count");
        std::string previous;
        for (std::uint32_t i = 0; i < count; ++i) {
            std::string role; if (!reader.text(role, kMaxRoleUTF8Bytes) || !UTF8(role, true, kMaxRoleUTF8Bytes)
                || (i && !(previous < role))) return Fail(refusal, "noncanonical required roles");
            previous = role; temporary.targetProfile.requiredRoles.push_back(std::move(role));
        }
        std::uint8_t flag = 0; if (!reader.u8(flag) || flag > 1) return Fail(refusal, "invalid verification flag");
        temporary.targetProfile.externallyVerified = flag == 1;
        if (!reader.u32(count) || count > kMaxAttachments) return Fail(refusal, "invalid attachment count");
        previous.clear();
        for (std::uint32_t i = 0; i < count; ++i) {
            Attachment attachment; if (!reader.fixed(attachment.attachmentID, kUUIDBytes)
                || (i && !(previous < attachment.attachmentID))) return Fail(refusal, "noncanonical attachment ids");
            previous = attachment.attachmentID;
            if (!reader.text(attachment.displayName, kMaxNameUTF8Bytes)
                || !reader.text(attachment.role, kMaxRoleUTF8Bytes) || !reader.frame(attachment.frameMM))
                return Fail(refusal, "truncated attachment");
            if (!UTF8Encoding(attachment.displayName) || !UTF8Encoding(attachment.role))
                return Fail(refusal, "invalid attachment UTF-8 encoding");
            std::uint8_t binding = 0; std::string owner;
            if (!reader.u8(binding) || binding > 1 || !reader.fixed(owner, kOwnerBytes)) return Fail(refusal, "invalid attachment binding");
            if (binding == 0) attachment.rigid = RigidOwnerBinding{std::move(owner)};
            else {
                SurfaceBinding surface; surface.owner = std::move(owner);
                if (!reader.text(surface.faceSelector, kMaxReceiptBytes) || !reader.frame(surface.localFrameMM))
                    return Fail(refusal, "truncated surface binding");
                attachment.surface = std::move(surface);
            }
            temporary.attachments.push_back(std::move(attachment));
        }
        if (!reader.u32(count) || count > kMaxJoints) return Fail(refusal, "invalid joint count");
        previous.clear();
        for (std::uint32_t i = 0; i < count; ++i) {
            Joint joint; if (!reader.fixed(joint.jointID, kUUIDBytes) || (i && !(previous < joint.jointID)))
                return Fail(refusal, "noncanonical joint ids");
            previous = joint.jointID;
            if (!reader.text(joint.displayName, kMaxNameUTF8Bytes) || !reader.u8(flag) || flag > 1)
                return Fail(refusal, "truncated joint");
            if (!UTF8Encoding(joint.displayName)) return Fail(refusal, "invalid joint UTF-8 encoding");
            if (flag) { std::string parent; if (!reader.fixed(parent, kUUIDBytes)) return Fail(refusal, "truncated parent"); joint.parentJointID = std::move(parent); }
            if (!reader.frame(joint.bindTransformMM)) return Fail(refusal, "truncated bind frame");
            temporary.joints.push_back(std::move(joint));
        }
        if (reader.at != reader.size) return Fail(refusal, "trailing ABR1 payload bytes");
        if (!Validate(temporary, refusal)) { refusalClass = DecodeRefusalClass::Metadata; return false; }
        output = std::move(temporary); refusal.clear(); return true;
    } catch (...) { return Fail(refusal, "ABR1 decode exception"); }
}

bool DecodeABR1(const std::vector<std::uint8_t>& bytes, Metadata& output, std::string& refusal) noexcept {
    DecodeRefusalClass refusalClass;
    return DecodeABR1Classified(bytes, output, refusal, refusalClass);
}
} // namespace core3d::attachment_rig

@interface Core3DAttachmentRigOwnerKey ()
@property(nonatomic, copy, readwrite) NSData *documentID;
@property(nonatomic, copy, readwrite) NSData *entityID;
@property(nonatomic, copy, readwrite) NSData *definitionID;
- (instancetype)initPrivate;
@end
@implementation Core3DAttachmentRigOwnerKey
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DAttachmentRigOwnerObservation ()
@property(nonatomic, strong, readwrite) Core3DAttachmentRigOwnerKey *ownerKey;
@property(nonatomic, copy, readwrite) NSString *entityIdentifier;
@property(nonatomic, copy, readwrite) NSData *sourceFeatureID;
@property(nonatomic, copy, readwrite) NSData *recipeWitnessBytes;
@property(nonatomic, copy, readwrite) NSData *operandWitnessBytes;
@property(nonatomic, copy, readwrite) NSData *dependentWitnessBytes;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *placementTransformDocument;
@property(nonatomic, assign, readwrite) double positiveUniformScale;
- (instancetype)initPrivate;
@end
@implementation Core3DAttachmentRigOwnerObservation
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DAttachmentRigFrameObservation ()
@property(nonatomic, copy, readwrite) NSData *attachmentID;
@property(nonatomic, strong, readwrite) Core3DAttachmentRigOwnerKey *ownerKey;
@property(nonatomic, copy, readwrite) NSString *displayName;
@property(nonatomic, copy, readwrite) NSString *role;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *localFrameMM;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *worldFrameMM;
- (instancetype)initPrivate;
@end
@implementation Core3DAttachmentRigFrameObservation
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DAttachmentRigJointObservation ()
@property(nonatomic, copy, readwrite) NSData *jointID;
@property(nonatomic, copy, readwrite, nullable) NSData *parentJointID;
@property(nonatomic, copy, readwrite) NSString *displayName;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *bindFrameMM;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *worldFrameMM;
- (instancetype)initPrivate;
@end
@implementation Core3DAttachmentRigJointObservation
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DAttachmentRigValidation ()
@property(nonatomic, copy, readwrite) NSData *canonicalMetadataBytes;
@property(nonatomic, copy, readwrite) NSString *policyID;
@property(nonatomic, copy, readwrite) NSString *policyVersion;
@property(nonatomic, copy, readwrite) NSArray<NSString *> *requiredRoles;
@property(nonatomic, assign, readwrite, getter=isExternallyVerified) BOOL externallyVerified;
@property(nonatomic, copy, readwrite) NSArray<Core3DAttachmentRigFrameObservation *> *attachments;
@property(nonatomic, copy, readwrite) NSArray<Core3DAttachmentRigJointObservation *> *joints;
- (instancetype)initPrivate;
@end
@implementation Core3DAttachmentRigValidation
- (instancetype)initPrivate { return [super init]; }
@end

namespace {
using Metadata = core3d::attachment_rig::Metadata;
using Frame = core3d::attachment_rig::FrameMM;
using OpeningContext = core3d::native_opening::Context;
using UUID = core3d::retained_solid::UUID;

NSError *RigError(Core3DAttachmentRigError code, NSString *reason) {
    return [NSError errorWithDomain:Core3DAttachmentRigErrorDomain code:code
                           userInfo:@{NSLocalizedDescriptionKey: reason ?: @"attachment rig refused"}];
}

NSString *Text(const std::string& value) { return [[NSString alloc] initWithBytes:value.data() length:value.size() encoding:NSUTF8StringEncoding] ?: @""; }
NSData *Data(const void *bytes, std::size_t count) { return [NSData dataWithBytes:bytes length:count]; }
NSData *Data(const std::string& value) { return Data(value.data(), value.size()); }
NSData *Data(const UUID& value) { return Data(value.data(), value.size()); }
NSString *UUIDText(const UUID& value) { return Text(core3d::retained_solid::UUIDText(value)); }

struct Evidence final {
    std::vector<std::uint8_t> bytes; bool good = true;
    void raw(const void *p, std::size_t n) {
        constexpr std::size_t limit = 4U * 1024U * 1024U;
        if (!good || n > limit - std::min(bytes.size(), limit)) { good = false; return; }
        const auto *b = static_cast<const std::uint8_t *>(p); bytes.insert(bytes.end(), b, b + n);
    }
    void u64(std::uint64_t n) { std::uint8_t b[8]; for (unsigned i=0;i<8;++i)b[i]=std::uint8_t(n>>(8*i)); raw(b,8); }
    void number(double n) { std::uint64_t bits=0; std::memcpy(&bits,&n,8); u64(bits); }
    void field(const void *p,std::size_t n){u64(n);raw(p,n);} void text(const std::string&s){field(s.data(),s.size());}
    NSData *finish(){return good&& !bytes.empty() ? Data(bytes.data(),bytes.size()) : nil;}
};

struct NativeOwner final {
    std::string tuple;
    gp_Trsf placement;
    Core3DAttachmentRigOwnerObservation *observation = nil;
};
struct NativeObservation final {
    Core3DSceneSnapshot *scene = nil;
    NSString *document = nil;
    NSArray<NSString *> *selected = nil;
    NSArray<Core3DAttachmentRigOwnerObservation *> *owners = nil;
    NSData *source = nil;
    NSInteger history = 0;
    std::vector<NativeOwner> nativeOwners;
};

bool RootForEntity(const Handle(OcctDocument)& document, NSString *entity, TDF_Label& result) {
    result.Nullify(); if (document.IsNull() || entity.length == 0) return false;
    const auto native = document->Document(); if (native.IsNull() || native->GetData().IsNull()) return false;
    const auto shapes = XCAFDoc_DocumentTool::ShapeTool(native->Main()); if (shapes.IsNull()) return false;
    TDF_LabelSequence roots; shapes->GetFreeShapes(roots); unsigned matches = 0;
    const char *text = entity.UTF8String;
    for (Standard_Integer i=1;i<=roots.Length();++i) if (document->EntityIdentifierForLabel(roots.Value(i)) == (text?text:"")) { result=roots.Value(i); ++matches; }
    return matches == 1 && !result.IsNull();
}

bool PositiveUniform(const gp_Trsf& transform, double& scale, std::array<double,9>& rotation) {
    const gp_Mat matrix = transform.VectorialPart(); double norms[3]{};
    for (unsigned c=1;c<=3;++c) { double sum=0; for(unsigned r=1;r<=3;++r){double v=matrix.Value(r,c);if(!std::isfinite(v))return false;sum+=v*v;} norms[c-1]=std::sqrt(sum); }
    scale=(norms[0]+norms[1]+norms[2])/3.0;
    if(!std::isfinite(scale)||scale<=0) return false;
    for(double n:norms) if(std::abs(n-scale)>1e-9*std::max(1.0,scale)) return false;
    unsigned at=0; for(unsigned c=1;c<=3;++c)for(unsigned r=1;r<=3;++r)rotation[at++]=matrix.Value(r,c)/scale;
    const double det = rotation[0]*(rotation[4]*rotation[8]-rotation[5]*rotation[7])
        -rotation[3]*(rotation[1]*rotation[8]-rotation[2]*rotation[7])
        +rotation[6]*(rotation[1]*rotation[5]-rotation[2]*rotation[4]);
    return std::isfinite(det)&&std::abs(det-1)<=1e-9;
}

bool CaptureOwner(const Handle(OcctDocument)& document, NSString *entity, NativeOwner& output) {
    TDF_Label label; if(!RootForEntity(document,entity,label)) return false;
    OcctCylindricalCutProgramSource program; OcctCylindricalCutSource single;
    core3d::retained_boolean::Recipe recipe; std::vector<std::uint8_t> recipeBytes; OcctObjectTransformState state;
    if(document->CaptureCylindricalCutProgramSource(label,program)){recipe=program.recipe;recipeBytes=program.recipeBytes;state=program.original;}
    else if(document->CaptureCylindricalCutSource(label,single)&&single.envelope.sourceFamily==2){recipe=single.envelope;state=single.original;if(!core3d::retained_boolean::Encode(recipe,recipeBytes))return false;}
    else return false;
    const auto identities=core3d::retained_boolean::Identities(recipe);
    if(entity.UTF8String==nullptr || document->DocumentIdentifier()!=core3d::retained_solid::UUIDText(identities.document)
       || std::string(entity.UTF8String)!=core3d::retained_solid::UUIDText(identities.entity)
       || document->DefinitionIdentifierForLabel(label)!=core3d::retained_solid::UUIDText(identities.definition)) return false;
    std::array<double,9> rotation{};double scale=0;if(!PositiveUniform(state.transform,scale,rotation))return false;
    std::string tuple(reinterpret_cast<const char*>(identities.document.data()),16);
    tuple.append(reinterpret_cast<const char*>(identities.entity.data()),16);
    tuple.append(reinterpret_cast<const char*>(identities.definition.data()),16);
    Evidence recipeEvidence; recipeEvidence.text("shapeyard.f4.recipe.v1");recipeEvidence.raw(tuple.data(),tuple.size());
    recipeEvidence.raw(identities.sourceFeature.data(),16);recipeEvidence.raw(identities.derivedFeature.data(),16);
    recipeEvidence.field(recipeBytes.data(),recipeBytes.size());
    Evidence operand;operand.text("shapeyard.f4.operands.v1");operand.field(recipeBytes.data(),recipeBytes.size());
    if(const auto *legacy=std::get_if<core3d::retained_boolean::Legacy>(&recipe)){operand.u64(1);operand.u64(legacy->operandID);}
    else {const auto& retained=std::get<core3d::retained_boolean::Program>(recipe);operand.u64(retained.steps.size());for(const auto& step:retained.steps){operand.u64(step.operand.identifier);operand.u64(std::uint64_t(step.operand.kind));}}
    Evidence dependent;dependent.text("shapeyard.f4.dependents.v1");dependent.raw(identities.sourceFeature.data(),16);dependent.raw(identities.derivedFeature.data(),16);
    for(double v:state.scalars)dependent.number(v);for(bool p:state.present)dependent.u64(p);
    NSData *recipeData=recipeEvidence.finish(),*operandData=operand.finish(),*dependentData=dependent.finish();
    if(!recipeData||!operandData||!dependentData)return false;
    Core3DAttachmentRigOwnerKey *key=[[Core3DAttachmentRigOwnerKey alloc] initPrivate];
    key.documentID=Data(identities.document);key.entityID=Data(identities.entity);key.definitionID=Data(identities.definition);
    Core3DAttachmentRigOwnerObservation *observation=[[Core3DAttachmentRigOwnerObservation alloc] initPrivate];
    observation.ownerKey=key;observation.entityIdentifier=[entity copy];observation.sourceFeatureID=Data(identities.sourceFeature);
    observation.recipeWitnessBytes=recipeData;observation.operandWitnessBytes=operandData;observation.dependentWitnessBytes=dependentData;
    NSMutableArray<NSNumber*> *placement=[NSMutableArray arrayWithCapacity:12];const gp_Mat matrix=state.transform.VectorialPart();
    for(unsigned c=1;c<=3;++c)for(unsigned r=1;r<=3;++r)[placement addObject:@(matrix.Value(r,c))];
    const gp_XYZ t=state.transform.TranslationPart();[placement addObjectsFromArray:@[@(t.X()),@(t.Y()),@(t.Z())]];
    observation.placementTransformDocument=placement;observation.positiveUniformScale=scale;
    output.tuple=std::move(tuple);output.placement=state.transform;output.observation=observation;return true;
}

bool Observe(Core3DViewController *controller,const Handle(OcctDocument)& document,NSArray<NSString*> *expected,NativeObservation& output){
    output={};if(!controller||document.IsNull())return false;Core3DSceneSnapshot *scene=[controller captureExportSceneSnapshot];if(!scene)return false;
    if(!std::isfinite(scene.metersPerUnit)||scene.metersPerUnit<=0||scene.selectionMode!=Core3DSceneElementKindObject
       ||scene.selection.selectedElements.count==0||scene.selection.selectedElements.count>64)return false;
    NSMutableArray<NSString*> *selected=[NSMutableArray array];NSMutableSet<NSString*> *unique=[NSMutableSet set];
    for(Core3DSceneElementIdentifier *element in scene.selection.selectedElements){if(element.kind!=Core3DSceneElementKindObject||element.topologyIndex!=0||element.entityIdentifier.length==0||[unique containsObject:element.entityIdentifier])return false;[unique addObject:element.entityIdentifier];[selected addObject:element.entityIdentifier];}
    [selected sortUsingSelector:@selector(compare:)];if(expected&&![selected isEqualToArray:expected])return false;
    std::vector<NativeOwner> native;native.reserve(selected.count);NSMutableArray *owners=[NSMutableArray arrayWithCapacity:selected.count];
    Evidence source;source.text("shapeyard.f4.capture.v1");source.text(document->DocumentIdentifier());source.number(scene.metersPerUnit);
    source.u64(scene.revisions.documentGeneration);source.u64(scene.revisions.modelRevision);source.u64(scene.revisions.presentationRevision);source.u64(controller.documentUndoCount);source.u64(selected.count);
    for(NSString *entity in selected){NativeOwner owner;if(!CaptureOwner(document,entity,owner))return false;source.field(owner.observation.recipeWitnessBytes.bytes,owner.observation.recipeWitnessBytes.length);source.field(owner.observation.operandWitnessBytes.bytes,owner.observation.operandWitnessBytes.length);source.field(owner.observation.dependentWitnessBytes.bytes,owner.observation.dependentWitnessBytes.length);for(NSNumber *v in owner.observation.placementTransformDocument)source.number(v.doubleValue);[owners addObject:owner.observation];native.push_back(std::move(owner));}
    NSData *sourceData=source.finish();if(!sourceData)return false;
    output.scene=scene;output.document=Text(document->DocumentIdentifier());output.selected=selected;output.owners=owners;output.source=sourceData;output.history=controller.documentUndoCount;output.nativeOwners=std::move(native);return true;
}

NSArray<NSNumber*> *FrameArray(const Frame& frame){NSMutableArray *a=[NSMutableArray arrayWithCapacity:12];for(double v:frame.originMM)[a addObject:@(v)];for(double v:frame.x)[a addObject:@(v)];for(double v:frame.y)[a addObject:@(v)];for(double v:frame.z)[a addObject:@(v)];return a;}
Frame Compose(const Frame& parent,const Frame& local){Frame r;for(unsigned i=0;i<3;++i)r.originMM[i]=parent.originMM[i]+parent.x[i]*local.originMM[0]+parent.y[i]*local.originMM[1]+parent.z[i]*local.originMM[2];auto axis=[&](const std::array<double,3>&v){std::array<double,3>o{};for(unsigned i=0;i<3;++i)o[i]=parent.x[i]*v[0]+parent.y[i]*v[1]+parent.z[i]*v[2];return o;};r.x=axis(local.x);r.y=axis(local.y);r.z=axis(local.z);return r;}
Frame OwnerWorld(const NativeOwner& owner,double metersPerUnit,const Frame& local){Frame r;std::array<double,9> rotation{};double scale=0;PositiveUniform(owner.placement,scale,rotation);const gp_XYZ t=owner.placement.TranslationPart();const double c=1000.0*metersPerUnit;const double origin[3]={local.originMM[0],local.originMM[1],local.originMM[2]};for(unsigned row=0;row<3;++row)r.originMM[row]=c*(row==0?t.X():(row==1?t.Y():t.Z()))+scale*(rotation[row]*origin[0]+rotation[3+row]*origin[1]+rotation[6+row]*origin[2]);auto axis=[&](const std::array<double,3>&v){std::array<double,3>o{};for(unsigned row=0;row<3;++row)o[row]=rotation[row]*v[0]+rotation[3+row]*v[1]+rotation[6+row]*v[2];return o;};r.x=axis(local.x);r.y=axis(local.y);r.z=axis(local.z);return r;}
Core3DAttachmentRigError ErrorForRefusal(const std::string& refusal){if(refusal.find("surface")!=std::string::npos)return Core3DAttachmentRigErrorUnsupportedSurface;if(refusal.find("ABR1")!=std::string::npos||refusal.find("schema")!=std::string::npos||refusal.find("truncated")!=std::string::npos||refusal.find("trailing")!=std::string::npos||refusal.find("checksum")!=std::string::npos||refusal.find("canonical")!=std::string::npos)return Core3DAttachmentRigErrorInvalidEncoding;return Core3DAttachmentRigErrorInvalidMetadata;}
Core3DAttachmentRigError ErrorForDecodeRefusal(core3d::attachment_rig::DecodeRefusalClass refusalClass,const std::string& refusal){return refusalClass==core3d::attachment_rig::DecodeRefusalClass::Encoding?Core3DAttachmentRigErrorInvalidEncoding:ErrorForRefusal(refusal);}
} // namespace

@interface Core3DAttachmentRigOpening ()
@property(nonatomic, copy, readwrite) NSArray<Core3DAttachmentRigOwnerObservation *> *owners;
@property(nonatomic, copy, readwrite) NSArray<NSString *> *selectedEntityIdentifiers;
@property(nonatomic, copy, readwrite) NSData *sourceWitnessBytes;
@property(nonatomic, copy, readwrite, nullable) NSData *metadataProposalBytes;
@property(nonatomic, strong, readwrite, nullable) Core3DAttachmentRigValidation *validation;
@property(nonatomic, copy, readwrite) NSString *documentIdentifier;
@property(nonatomic, copy, readwrite) NSString *contextIdentifier;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@property(nonatomic, assign, readwrite) uint64_t documentGeneration;
@property(nonatomic, assign, readwrite) uint64_t modelRevision;
@property(nonatomic, assign, readwrite) uint64_t presentationRevision;
@property(nonatomic, assign, readwrite) NSInteger historyDepth;
@end

@implementation Core3DAttachmentRigOpening {
    __weak Core3DViewController *_controller;
    Handle(OcctDocument) _document;
    std::shared_ptr<OpeningContext> _context;
    std::vector<NativeOwner> _nativeOwners;
    std::shared_ptr<std::atomic_bool> _cancelled;
    std::shared_ptr<std::atomic_bool> _bindingConsumed;
    bool _canBind;
}

- (instancetype)initPrivate { return [super init]; }

+ (instancetype)captureController:(Core3DViewController *)controller error:(NSError **)error {
    if(error)*error=nil;if(!NSThread.isMainThread||!controller){if(error)*error=RigError(Core3DAttachmentRigErrorInvalidThread,@"main-thread controller required");return nil;}
    @try {GLViewController *gl=[controller.glController isKindOfClass:GLViewController.class]?(GLViewController*)controller.glController:nil;auto viewer=gl?gl.viewer:nullptr;Handle(OcctDocument) document=viewer?viewer->getDocument():Handle(OcctDocument)();if(!viewer||document.IsNull()){if(error)*error=RigError(Core3DAttachmentRigErrorUnavailable,@"native document unavailable");return nil;}
        NativeObservation observation;if(!Observe(controller,document,nil,observation)){if(error)*error=RigError(Core3DAttachmentRigErrorUnavailable,@"retained native owner observation refused");return nil;}
        const CGSize drawable=controller.viewportDrawableSize;if(!std::isfinite(drawable.width)||!std::isfinite(drawable.height)||drawable.width<1||drawable.height<1||drawable.width>UINT32_MAX||drawable.height>UINT32_MAX){if(error)*error=RigError(Core3DAttachmentRigErrorUnavailable,@"viewport unavailable");return nil;}
        std::vector<std::string> receipts;for(NSString *entity in observation.selected)receipts.emplace_back(entity.UTF8String?:"");auto context=viewer->captureNativeOpeningContext(std::uint32_t(drawable.width),std::uint32_t(drawable.height),receipts);if(!context||!context->isCurrent(64,64)){if(error)*error=RigError(Core3DAttachmentRigErrorStale,@"native opening fence changed");return nil;}
        Core3DAttachmentRigOpening *opening=[[self alloc]initPrivate];opening->_controller=controller;opening->_document=document;opening->_context=std::move(context);opening->_nativeOwners=std::move(observation.nativeOwners);opening->_cancelled=std::make_shared<std::atomic_bool>(false);opening->_bindingConsumed=std::make_shared<std::atomic_bool>(false);opening->_canBind=true;
        opening.owners=observation.owners;opening.selectedEntityIdentifiers=observation.selected;opening.sourceWitnessBytes=observation.source;opening.documentIdentifier=observation.document;opening.contextIdentifier=NSUUID.UUID.UUIDString;opening.metersPerUnit=observation.scene.metersPerUnit;opening.documentGeneration=observation.scene.revisions.documentGeneration;opening.modelRevision=observation.scene.revisions.modelRevision;opening.presentationRevision=observation.scene.revisions.presentationRevision;opening.historyDepth=observation.history;return opening;
    }@catch(__unused NSException *exception){if(error)*error=RigError(Core3DAttachmentRigErrorUnavailable,@"native capture exception");return nil;}
}

+ (NSData *)canonicalABR1ByValidatingBytes:(NSData *)bytes error:(NSError **)error {
    if(error)*error=nil;if(!bytes||bytes.length>1024U*1024U){if(error)*error=RigError(Core3DAttachmentRigErrorInvalidArgument,@"bounded ABR1 bytes required");return nil;}
    std::vector<std::uint8_t> input(bytes.length);if(bytes.length)std::memcpy(input.data(),bytes.bytes,bytes.length);Metadata metadata;std::string refusal;core3d::attachment_rig::DecodeRefusalClass refusalClass;if(!core3d::attachment_rig::DecodeABR1Classified(input,metadata,refusal,refusalClass)){if(error)*error=RigError(ErrorForDecodeRefusal(refusalClass,refusal),Text(refusal));return nil;}std::vector<std::uint8_t> canonical;if(!core3d::attachment_rig::EncodeABR1(metadata,canonical,refusal)){if(error)*error=RigError(ErrorForRefusal(refusal),Text(refusal));return nil;}return Data(canonical.data(),canonical.size());
}

- (Core3DAttachmentRigOpening *)openingByBindingMetadataProposal:(NSData *)bytes error:(NSError **)error {
    if(error)*error=nil;if(!NSThread.isMainThread){if(error)*error=RigError(Core3DAttachmentRigErrorInvalidThread,@"main-thread opening required");return nil;}if(!_canBind||_bindingConsumed->load()){if(error)*error=RigError(Core3DAttachmentRigErrorReplayed,@"opening already used");return nil;}if(_cancelled->load()){if(error)*error=RigError(Core3DAttachmentRigErrorCancelled,@"opening cancelled");return nil;}if(![self isCurrent:error])return nil;
    std::vector<std::uint8_t> input(bytes.length);if(bytes.length)std::memcpy(input.data(),bytes.bytes,bytes.length);Metadata metadata;std::string refusal;core3d::attachment_rig::DecodeRefusalClass refusalClass;if(!core3d::attachment_rig::DecodeABR1Classified(input,metadata,refusal,refusalClass)){if(error)*error=RigError(ErrorForDecodeRefusal(refusalClass,refusal),Text(refusal));return nil;}
    const auto& profile=metadata.targetProfile;if(profile.profileID!="local.validation.policy"||profile.profileVersion!="v1"||profile.externallyVerified||profile.requiredRoles.size()!=2||profile.requiredRoles[0]!="socket.primary"||profile.requiredRoles[1]!="socket.secondary"){
        if(error)*error=RigError(Core3DAttachmentRigErrorInvalidMetadata,@"native local validation policy mismatch");return nil;}
    std::vector<std::uint8_t> canonical;if(!core3d::attachment_rig::EncodeABR1(metadata,canonical,refusal)){if(error)*error=RigError(ErrorForRefusal(refusal),Text(refusal));return nil;}
    NSMutableArray *attachments=[NSMutableArray arrayWithCapacity:metadata.attachments.size()];
    for(const auto& attachment:metadata.attachments){const std::string& tuple=attachment.rigid->owner;auto owner=std::find_if(_nativeOwners.begin(),_nativeOwners.end(),[&](const NativeOwner&v){return v.tuple==tuple;});if(owner==_nativeOwners.end()){if(error)*error=RigError(Core3DAttachmentRigErrorForeignOwner,@"attachment owner is foreign to captured document");return nil;}Core3DAttachmentRigFrameObservation *record=[[Core3DAttachmentRigFrameObservation alloc] initPrivate];record.attachmentID=Data(attachment.attachmentID);record.ownerKey=owner->observation.ownerKey;record.displayName=Text(attachment.displayName);record.role=Text(attachment.role);record.localFrameMM=FrameArray(attachment.frameMM);record.worldFrameMM=FrameArray(OwnerWorld(*owner,self.metersPerUnit,attachment.frameMM));[attachments addObject:record];}
    std::map<std::string,const core3d::attachment_rig::Joint*> jointsByID;for(const auto& joint:metadata.joints)jointsByID[joint.jointID]=&joint;std::map<std::string,Frame> worlds;std::function<Frame(const core3d::attachment_rig::Joint&)> world=[&](const auto&joint){auto found=worlds.find(joint.jointID);if(found!=worlds.end())return found->second;Frame value=joint.parentJointID?Compose(world(*jointsByID[*joint.parentJointID]),joint.bindTransformMM):joint.bindTransformMM;worlds[joint.jointID]=value;return value;};
    NSMutableArray *jointRecords=[NSMutableArray arrayWithCapacity:metadata.joints.size()];for(const auto& joint:metadata.joints){Core3DAttachmentRigJointObservation *record=[[Core3DAttachmentRigJointObservation alloc] initPrivate];record.jointID=Data(joint.jointID);record.parentJointID=joint.parentJointID?Data(*joint.parentJointID):nil;record.displayName=Text(joint.displayName);record.bindFrameMM=FrameArray(joint.bindTransformMM);record.worldFrameMM=FrameArray(world(joint));[jointRecords addObject:record];}
    bool expected=false;if(!_bindingConsumed->compare_exchange_strong(expected,true)){if(error)*error=RigError(Core3DAttachmentRigErrorReplayed,@"opening already used");return nil;}
    Core3DAttachmentRigValidation *validation=[[Core3DAttachmentRigValidation alloc] initPrivate];validation.canonicalMetadataBytes=Data(canonical.data(),canonical.size());validation.policyID=Text(metadata.targetProfile.profileID);validation.policyVersion=Text(metadata.targetProfile.profileVersion);NSMutableArray *roles=[NSMutableArray array];for(const auto& role:metadata.targetProfile.requiredRoles)[roles addObject:Text(role)];validation.requiredRoles=roles;validation.externallyVerified=metadata.targetProfile.externallyVerified;validation.attachments=attachments;validation.joints=jointRecords;
    Core3DAttachmentRigOpening *bound=[[Core3DAttachmentRigOpening alloc]initPrivate];bound->_controller=_controller;bound->_document=_document;bound->_context=_context;bound->_nativeOwners=_nativeOwners;bound->_cancelled=_cancelled;bound->_bindingConsumed=_bindingConsumed;bound->_canBind=false;bound.owners=self.owners;bound.selectedEntityIdentifiers=self.selectedEntityIdentifiers;bound.sourceWitnessBytes=self.sourceWitnessBytes;bound.metadataProposalBytes=[bytes copy];bound.validation=validation;bound.documentIdentifier=self.documentIdentifier;bound.contextIdentifier=self.contextIdentifier;bound.metersPerUnit=self.metersPerUnit;bound.documentGeneration=self.documentGeneration;bound.modelRevision=self.modelRevision;bound.presentationRevision=self.presentationRevision;bound.historyDepth=self.historyDepth;return bound;
}

- (BOOL)isCurrent:(NSError **)error {if(error)*error=nil;if(!NSThread.isMainThread||!_controller){if(error)*error=RigError(Core3DAttachmentRigErrorInvalidThread,@"main-thread live controller required");return NO;}if(_cancelled->load()){if(error)*error=RigError(Core3DAttachmentRigErrorCancelled,@"opening cancelled");return NO;}if(!_context||!_context->isCurrent(64,64)){if(error)*error=RigError(Core3DAttachmentRigErrorStale,@"native opening fence changed");return NO;}NativeObservation fresh;if(!Observe(_controller,_document,self.selectedEntityIdentifiers,fresh)||![fresh.document isEqualToString:self.documentIdentifier]||![fresh.source isEqualToData:self.sourceWitnessBytes]||fresh.history!=self.historyDepth){if(error)*error=RigError(Core3DAttachmentRigErrorStale,@"captured dependency changed");return NO;}return YES;}
- (void)cancel {if(_cancelled)_cancelled->store(true);}
- (Core3DAttachmentRigApplyResult)apply { return Core3DAttachmentRigApplyResultUnavailable; }
@end
