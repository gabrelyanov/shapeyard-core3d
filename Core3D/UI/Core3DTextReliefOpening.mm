// E5-S1 detached Text and Emblem Relief opening. A controller/viewer-issued
// opening bound to one document, one native source entity and one issued
// planar-face selection. It admits trusted catalog-pinned SYTO/1 resources
// into canonical C1 contours and builds detached RAISED solids with the exact
// authored depth; it never writes the document, never runs a Boolean and
// keeps Emboss/Deboss unavailable. No command lease, retained record,
// history entry, registry admission or publication is reachable from here.

#import "Core3DTextReliefOpening.h"
#import "Core3DViewController.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/NativeOpeningContext.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/BoundedCurveBuild.hxx"
#include "../OCCTKit/TextReliefBuild.hxx"
#include "../OCCTKit/TextReliefNativeOwner.hxx"
#include "../OCCTKit/TextReliefPersistence.hxx"
#include "Core3DViewer.h"
#if DEBUG
#include "../OCCTKit/VariableRadiusFilletNativeOwner.hxx"
#include <BRepBuilderAPI_Copy.hxx>
#include <TopoDS_Shell.hxx>
#endif

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <CommonCrypto/CommonDigest.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

NSErrorDomain const Core3DTextReliefErrorDomain = @"Core3DTextReliefErrorDomain";
NSString *const Core3DTextReliefRefusalReasonKey = @"e5.refusal";
NSString *const Core3DTextReliefRefusalRawValueKey = @"e5.refusalRaw";

namespace {

namespace tr = core3d::text_relief;
namespace tb = core3d::retained_topology_budget;
using OpeningContext = core3d::native_opening::Context;

NSError *E5Error(Core3DTextReliefError code, NSString *reason) {
    return [NSError errorWithDomain:Core3DTextReliefErrorDomain code:code
        userInfo:@{NSLocalizedDescriptionKey: reason ?: @"text relief refused"}];
}
NSError *E5Refusal(Core3DTextReliefError code, const char *reason, int raw) {
    NSString *text = [NSString stringWithUTF8String:reason];
    return [NSError errorWithDomain:Core3DTextReliefErrorDomain code:code
        userInfo:@{NSLocalizedDescriptionKey: text,
                   Core3DTextReliefRefusalReasonKey: text,
                   Core3DTextReliefRefusalRawValueKey: @(raw)}];
}
NSError *E5Refusal(Core3DTextReliefError code, tr::Refusal refusal) {
    return E5Refusal(code, tr::Reason(refusal), static_cast<int>(refusal));
}
NSError *E5Refusal(Core3DTextReliefError code, tr::BuildRefusal refusal) {
    return E5Refusal(code, tr::Reason(refusal), static_cast<int>(refusal));
}
NSData *E5Data(const void *bytes, std::size_t count) {
    return [NSData dataWithBytes:bytes length:count];
}
NSData *E5Data(const std::vector<std::uint8_t>& bytes) {
    return E5Data(bytes.data(), bytes.size());
}
template <std::size_t N> NSData *E5Data(const std::array<std::uint8_t, N>& bytes) {
    return E5Data(bytes.data(), bytes.size());
}
NSArray<NSNumber *> *E5Vector(const std::array<double, 3>& value) {
    return @[ @(value[0]), @(value[1]), @(value[2]) ];
}

tr::UUID MintUUID() {
    tr::UUID value{};
    arc4random_buf(value.data(), value.size());
    value[6] = std::uint8_t((value[6] & 0x0f) | 0x40);
    value[8] = std::uint8_t((value[8] & 0x3f) | 0x80);
    return value;
}

bool ParseUUIDText(const std::string& text, tr::UUID& output) noexcept {
    output.fill(0);
    try {
        std::string hex;
        for (char c : text) if (c != '-') hex.push_back(c);
        if (hex.size() != 32) return false;
        for (std::size_t index = 0; index < output.size(); ++index) {
            unsigned value = 0;
            for (unsigned half = 0; half < 2; ++half) {
                const char c = hex[index * 2 + half]; value <<= 4;
                if (c >= '0' && c <= '9') value |= unsigned(c - '0');
                else if (c >= 'a' && c <= 'f') value |= unsigned(c - 'a' + 10);
                else if (c >= 'A' && c <= 'F') value |= unsigned(c - 'A' + 10);
                else { output.fill(0); return false; }
            }
            output[index] = std::uint8_t(value);
        }
        return core3d::bounded_curve::Nonzero(output);
    } catch (...) { output.fill(0); return false; }
}

tr::Digest SHA256(const void *bytes, std::size_t count) {
    tr::Digest digest{};
    CC_SHA256(bytes, CC_LONG(count), digest.data());
    if (!core3d::retained_recipe::Nonzero(digest)) digest[0] = 1;
    return digest;
}
tr::Digest SHA256(const std::vector<std::uint8_t>& bytes) {
    return SHA256(bytes.data(), bytes.size());
}

bool E5CatalogPin(NSString *handle, const tr::Digest& digest,
                  std::size_t byteCount, tr::ResourceKind kind) noexcept {
    const char *name = handle.UTF8String;
    if (!name) return false;
    const char *expected = nullptr;
    std::size_t expectedBytes = 0;
    tr::ResourceKind expectedKind = tr::ResourceKind::FontOutline;
    if (std::strcmp(name, "shapeyard-block-latin-1") == 0) {
        expected = "c15c1c690ef851579dec71eb76cac011ec7cfadc4473a5f87c71a9b79bc134fd";
        expectedBytes = 8248;
    } else if (std::strcmp(name, "shapeyard-block-latin-wide-1") == 0) {
        expected = "73b41ceb388a8a78cd429eb71899b1233dc38e7d6403e6b248808c4b9e9f19eb";
        expectedBytes = 8248;
    } else if (std::strcmp(name, "shapeyard-diamond-1") == 0) {
        expected = "2c63b541f476797061f6bd22953b1af2cba2fb3aadaac321fdc8f0df2f1bb5a9";
        expectedBytes = 152; expectedKind = tr::ResourceKind::EmblemVector;
    } else return false;
    if (byteCount != expectedBytes || kind != expectedKind) return false;
    char actual[65] = {};
    static const char hex[] = "0123456789abcdef";
    for (std::size_t i = 0; i < digest.size(); ++i) {
        actual[2 * i] = hex[digest[i] >> 4]; actual[2 * i + 1] = hex[digest[i] & 15];
    }
    return std::strcmp(actual, expected) == 0;
}

tr::Digest SelectorProof(const std::string& document, const std::string& entity,
                         std::uint32_t topologyIndex, const core3d::bounded_curve::Frame& frame) {
    tr::Digest digest{};
    CC_SHA256_CTX sha; CC_SHA256_Init(&sha);
    CC_SHA256_Update(&sha, document.data(), CC_LONG(document.size()));
    CC_SHA256_Update(&sha, entity.data(), CC_LONG(entity.size()));
    CC_SHA256_Update(&sha, &topologyIndex, CC_LONG(sizeof(topologyIndex)));
    CC_SHA256_Update(&sha, frame.identifier.data(), CC_LONG(frame.identifier.size()));
    const std::uint64_t revision = frame.revision;
    CC_SHA256_Update(&sha, &revision, CC_LONG(sizeof(revision)));
    for (const auto& axis : {frame.origin, frame.xAxis, frame.yAxis, frame.zAxis})
        for (double component : axis) {
            std::uint64_t bits = 0; std::memcpy(&bits, &component, sizeof(bits));
            CC_SHA256_Update(&sha, &bits, CC_LONG(sizeof(bits)));
        }
    CC_SHA256_Final(digest.data(), &sha);
    if (!core3d::retained_recipe::Nonzero(digest)) digest[0] = 1;
    return digest;
}

bool RootForEntity(const Handle(OcctDocument)& document, NSString *entity, TDF_Label& result) {
    result.Nullify();
    if (document.IsNull() || entity.length == 0) return false;
    const auto native = document->Document();
    if (native.IsNull() || native->GetData().IsNull()) return false;
    const auto shapes = XCAFDoc_DocumentTool::ShapeTool(native->Main());
    if (shapes.IsNull()) return false;
    TDF_LabelSequence roots; shapes->GetFreeShapes(roots); unsigned matches = 0;
    const char *text = entity.UTF8String;
    for (Standard_Integer index = 1; index <= roots.Length(); ++index)
        if (document->EntityIdentifierForLabel(roots.Value(index)) == (text ? text : "")) {
            result = roots.Value(index); ++matches;
        }
    return matches == 1 && !result.IsNull();
}

bool ReadShape(const std::vector<std::uint8_t>& bytes, TopoDS_Shape& shape) {
    shape.Nullify();
    if (bytes.empty()) return false;
    const std::string value(bytes.begin(), bytes.end());
    std::istringstream stream(value);
    stream.imbue(std::locale::classic());
    BRep_Builder builder;
    BRepTools::Read(shape, stream, builder);
    return stream.good() && !shape.IsNull();
}

std::size_t BoundaryEdges(const TopoDS_Face& face) {
    std::size_t count = 0;
    for (TopExp_Explorer explorer(face, TopAbs_EDGE); explorer.More(); explorer.Next()) ++count;
    return count;
}

struct IssuedSelection final {
    std::string entity;
    std::uint32_t topologyIndex = 0;
    std::vector<std::uint8_t> sourceBytes;
    TopoDS_Shape sourceShape;                    // in-process issued authority
    core3d::bounded_curve::Frame frame;        // origin on the face plane, zAxis outward
    tr::HostReceipt host;
    std::uint16_t boundaryEdges = 0;
    tr::Digest selectorProof{};
};

// One detached raised build request: the public API fills the caller fields;
// the DEBUG probe may additionally pin the identity seed and arm the
// foreign/inward/off-face failure legs.
struct E5RequestSpec final {
    std::vector<std::uint8_t> bytes;
    tr::Digest expected{};
    tr::ResourceKind kind = tr::ResourceKind::FontOutline;
    std::string text;
    double heightMM = 10, depthMM = 1, trackingMM = 0;
    int alignment = 1;      // 1 left, 2 center, 3 right
    double offsetUMM = 0, offsetVMM = 0, rotation = 0;
    tr::UUID seed{};
    bool mintSeed = true;
    bool foreignDigest = false, inwardFrame = false;
    bool forceOffset = false;
    double forcedOffsetUMM = 0;
};

struct E5Build final {
    bool built = false;
    bool admissionPhase = false;
    bool admittedOnly = false;
    tr::Refusal admission = tr::Refusal::None;
    tr::BuildRefusal refusal = tr::BuildRefusal::None;
    tr::RaisedBuildResult raised;
    std::vector<std::vector<std::uint8_t>> curveBytes;
    std::size_t contourCount = 0;
    tr::Digest resourceDigest{};
    tr::UUID identitySeed{};
    core3d::bounded_curve::Frame frame;
    std::vector<tr::CurveRecord> admittedCurves;
    std::vector<tr::ContourRecord> admittedContours;
    double advanceMM = 0;
    std::size_t stagesConsumed = 0, visitsConsumed = 0;
    bool budgetExhausted = false, cancelledObserved = false, stickyHeld = true;
};

} // namespace

@interface Core3DTextReliefIssuedFace ()
@property(nonatomic, copy, readwrite) NSData *identifier;
@property(nonatomic, copy, readwrite) NSData *selectorProof;
@property(nonatomic, copy, readwrite) NSString *entityIdentifier;
@property(nonatomic, assign, readwrite) uint32_t topologyIndex;
@property(nonatomic, assign, readwrite) uint16_t boundaryEdgeCount;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *frameOriginLocal;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *frameXAxis;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *frameYAxis;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *frameZAxis;
- (instancetype)initPrivate;
@end
@implementation Core3DTextReliefIssuedFace
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DTextReliefComponentObservation ()
@property(nonatomic, assign, readwrite) uint32_t componentIndex;
@property(nonatomic, copy, readwrite) NSData *identifier;
@property(nonatomic, assign, readwrite) double areaMM2;
@property(nonatomic, assign, readwrite) double volumeMM3;
@property(nonatomic, assign, readwrite) double minAlongNormalMM;
@property(nonatomic, assign, readwrite) double maxAlongNormalMM;
- (instancetype)initPrivate;
@end
@implementation Core3DTextReliefComponentObservation
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DTextReliefCandidate ()
@property(nonatomic, copy, readwrite) NSData *candidateBRepBytes;
@property(nonatomic, copy, readwrite) NSData *geometryDigest;
@property(nonatomic, copy, readwrite) NSData *resourceSHA256;
@property(nonatomic, copy, readwrite) NSArray<NSData *> *canonicalCurveBytes;
@property(nonatomic, copy, readwrite) NSArray<Core3DTextReliefComponentObservation *> *componentObservations;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@property(nonatomic, assign, readwrite) double totalAreaMM2;
@property(nonatomic, assign, readwrite) double totalVolumeMM3;
@property(nonatomic, assign, readwrite) double textAdvanceMM;
@property(nonatomic, assign, readwrite) NSUInteger connectedSolidCount;
@property(nonatomic, copy) NSData *e5ResourceBytes;
@property(nonatomic, copy) NSData *e5IdentitySeed;
@property(nonatomic, copy) NSString *e5OpeningContextIdentifier;
@property(nonatomic, assign) Core3DTextReliefResourceKind e5Kind;
@property(nonatomic, copy) NSString *e5Text;
@property(nonatomic, assign) double e5HeightMM;
@property(nonatomic, assign) double e5DepthMM;
@property(nonatomic, assign) double e5TrackingMM;
@property(nonatomic, assign) Core3DTextReliefAlignment e5Alignment;
@property(nonatomic, assign) double e5OffsetUMM;
@property(nonatomic, assign) double e5OffsetVMM;
@property(nonatomic, assign) double e5RotationRadians;
- (instancetype)initPrivate;
@end
@implementation Core3DTextReliefCandidate
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DTextReliefAuthoredIntent ()
@property(nonatomic, copy, readwrite) NSData *ownerIdentifier;
@property(nonatomic, copy, readwrite) NSData *featureIdentifier;
@property(nonatomic, assign, readwrite) uint64_t revision;
@property(nonatomic, assign, readwrite) Core3DTextReliefResourceKind resourceKind;
@property(nonatomic, copy, readwrite) NSString *catalogHandle;
@property(nonatomic, copy, readwrite) NSString *text;
@property(nonatomic, copy, readwrite) NSString *emblemIdentifier;
@property(nonatomic, assign, readwrite) double heightMM;
@property(nonatomic, assign, readwrite) double depthMM;
@property(nonatomic, assign, readwrite) double trackingMM;
@property(nonatomic, assign, readwrite) Core3DTextReliefAlignment alignment;
@property(nonatomic, assign, readwrite) Core3DTextReliefMode mode;
@property(nonatomic, assign, readwrite) double offsetUMM;
@property(nonatomic, assign, readwrite) double offsetVMM;
@property(nonatomic, assign, readwrite) double rotationRadians;
@property(nonatomic, assign, readwrite) double metersPerUnit;
- (instancetype)initPrivate;
@end
@implementation Core3DTextReliefAuthoredIntent
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DTextReliefOpening ()
@property(nonatomic, strong, readwrite) Core3DTextReliefIssuedFace *issuedFace;
@property(nonatomic, copy, readwrite) NSString *documentIdentifier;
@property(nonatomic, copy, readwrite) NSString *contextIdentifier;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@property(nonatomic, assign, readwrite) NSInteger historyDepth;
@property(nonatomic, strong, readwrite, nullable) Core3DTextReliefAuthoredIntent *authoredIntent;
@property(nonatomic, copy, readwrite, nullable) NSData *appliedOwnerIdentifier;
@property(nonatomic, copy, readwrite, nullable) NSData *appliedFeatureIdentifier;
@end

@implementation Core3DTextReliefOpening {
    __weak Core3DViewController *_controller;
    Handle(OcctDocument) _document;
    std::shared_ptr<OpeningContext> _context;
    IssuedSelection _issued;
    std::shared_ptr<std::atomic_bool> _cancelled;
    std::uint32_t _viewportWidth, _viewportHeight;
    std::optional<OcctTextReliefCapture> _existing;
    Core3DTextReliefCandidate *_preparedCandidate;
    NSString *_preparedCatalogHandle;
    bool _preparedRemoval;
    std::optional<tr::UUID> _recoveryFeature;
    bool _recoveryExpectsPresent;
#if DEBUG
    std::shared_ptr<std::atomic_int> _debugCancelAfterBuild;
    int _debugApplyFault;
#endif
}

- (instancetype)initPrivate { return [super init]; }

+ (instancetype)captureController:(Core3DViewController *)controller error:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !controller) {
        if (error) *error = E5Error(Core3DTextReliefErrorInvalidThread,
                                    @"main-thread controller required");
        return nil;
    }
    @try {
        Core3DSceneSnapshot *scene = [controller captureSceneSnapshot];
        if (!scene || scene.selectionMode != Core3DSceneElementKindFace
            || scene.selection.selectedElements.count != 1) {
            if (error) *error = E5Error(Core3DTextReliefErrorInvalidSelection,
                                        @"exactly one selected native face is required");
            return nil;
        }
        Core3DSceneElementIdentifier *element = scene.selection.selectedElements.firstObject;
        if (element.kind != Core3DSceneElementKindFace
            || element.entityIdentifier.length == 0
            || element.entityIdentifier.length > 128) {
            if (error) *error = E5Error(Core3DTextReliefErrorInvalidSelection,
                                        @"selected element is not a native face");
            return nil;
        }
        GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)controller.glController : nil;
        auto viewer = gl ? gl.viewer : nullptr;
        Handle(OcctDocument) document = viewer ? viewer->getDocument() : Handle(OcctDocument)();
        const CGSize drawable = controller.viewportDrawableSize;
        if (!viewer || document.IsNull() || !std::isfinite(drawable.width)
            || !std::isfinite(drawable.height) || drawable.width < 1 || drawable.height < 1
            || drawable.width > UINT32_MAX || drawable.height > UINT32_MAX) {
            if (error) *error = E5Error(Core3DTextReliefErrorUnavailable,
                                        @"native document unavailable");
            return nil;
        }
        const std::uint32_t width = std::uint32_t(drawable.width);
        const std::uint32_t height = std::uint32_t(drawable.height);
        const std::string entity = element.entityIdentifier.UTF8String ?: "";
        auto context = viewer->captureNativeOpeningContext(width, height, {entity});
        const Handle(TDocStd_Document) native = document->Document();
        if (!context || native.IsNull() || native->GetData().IsNull() || native->HasOpenCommand()
            || context->openingFence().document() != native
            || context->openingFence().data() != native->GetData()
            || !context->isCurrent(width, height)) {
            if (error) *error = E5Error(Core3DTextReliefErrorStale,
                                        @"native opening fence changed");
            return nil;
        }
        const double metersPerUnit = context->openingFence().metersPerUnit();
        if (!std::isfinite(metersPerUnit) || metersPerUnit <= 0 || metersPerUnit > 1) {
            if (error) *error = E5Error(Core3DTextReliefErrorUnavailable,
                                        @"native document unit unavailable");
            return nil;
        }
        TDF_Label label;
        if (!RootForEntity(document, element.entityIdentifier, label)
            || !XCAFDoc_ShapeTool::IsFree(label) || !XCAFDoc_ShapeTool::IsSimpleShape(label)) {
            if (error) *error = E5Error(Core3DTextReliefErrorInvalidSelection,
                                        @"selected entity is not a retained native source");
            return nil;
        }
        const TopoDS_Shape source = XCAFDoc_ShapeTool::GetShape(label);
        const TopoDS_Solid admittedHost = tr::detail::RaisedHostSolid(source);
        const TopoDS_Shape selectionSource = admittedHost.IsNull()
            ? source : TopoDS_Shape(admittedHost);
        IssuedSelection issued; issued.entity = entity;
        issued.topologyIndex = element.topologyIndex;
        issued.sourceShape = selectionSource;
        std::string exactBytes;
        if (selectionSource.IsNull()
            || !core3d::retained_part_boolean::ExactShapeBytes(selectionSource, exactBytes)
            || exactBytes.empty()) {
            if (error) *error = E5Error(Core3DTextReliefErrorUnavailable,
                                        @"selected native source unavailable");
            return nil;
        }
        issued.sourceBytes.assign(exactBytes.begin(), exactBytes.end());
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(selectionSource, TopAbs_FACE, faces);
        if (issued.topologyIndex >= std::uint32_t(faces.Extent())) {
            if (error) *error = E5Error(Core3DTextReliefErrorInvalidSelection,
                                        @"selected face is not in the native source");
            return nil;
        }
        const TopoDS_Face face = TopoDS::Face(
            faces.FindKey(Standard_Integer(issued.topologyIndex) + 1));
        BRepAdaptor_Surface surface(face);
        if (surface.GetType() != GeomAbs_Plane) {
            if (error) *error = E5Refusal(Core3DTextReliefErrorInvalidSelection,
                                          "e5.non-planar-host", 0);
            return nil;
        }
        const std::size_t boundary = BoundaryEdges(face);
        if (boundary < 3 || boundary > 64) {
            if (error) *error = E5Error(Core3DTextReliefErrorInvalidSelection,
                                        @"selected face boundary is unsupported");
            return nil;
        }
        // Native placement frame: origin at the face centroid on the plane,
        // zAxis the outward normal (reversed faces included), xAxis/yAxis a
        // deterministic in-plane right-handed basis (plain double math).
        gp_Dir outwardDir = surface.Plane().Axis().Direction();
        if (face.Orientation() == TopAbs_REVERSED) outwardDir.Reverse();
        const double nx = outwardDir.X(), ny = outwardDir.Y(), nz = outwardDir.Z();
        GProp_GProps faceProps;
        BRepGProp::SurfaceProperties(face, faceProps);
        const gp_Pnt centre = faceProps.CentreOfMass();
        const double globalAxes[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        int chosen = 0;
        double least = 2;
        for (int index = 0; index < 3; ++index) {
            const double parallel = std::abs(globalAxes[index][0] * nx
                + globalAxes[index][1] * ny + globalAxes[index][2] * nz);
            if (parallel < least) { least = parallel; chosen = index; }
        }
        const double along = globalAxes[chosen][0] * nx + globalAxes[chosen][1] * ny
            + globalAxes[chosen][2] * nz;
        double xx = globalAxes[chosen][0] - nx * along;
        double xy = globalAxes[chosen][1] - ny * along;
        double xz = globalAxes[chosen][2] - nz * along;
        const double xNorm = std::sqrt(xx * xx + xy * xy + xz * xz);
        if (xNorm < 1e-9) {
            if (error) *error = E5Error(Core3DTextReliefErrorInvalidSelection,
                                        @"selected face has no stable in-plane basis");
            return nil;
        }
        xx /= xNorm; xy /= xNorm; xz /= xNorm;
        // y = z cross x keeps the basis right-handed.
        const double yx = ny * xz - nz * xy;
        const double yy = nz * xx - nx * xz;
        const double yz = nx * xy - ny * xx;
        core3d::bounded_curve::Frame frame;
        frame.identifier = MintUUID();
        frame.revision = 1;
        frame.origin = {{centre.X(), centre.Y(), centre.Z()}};
        frame.xAxis = {{xx, xy, xz}};
        frame.yAxis = {{yx, yy, yz}};
        frame.zAxis = {{nx, ny, nz}};
        issued.frame = frame;
        if (!ParseUUIDText(entity, issued.host.entity)) {
            if (error) *error = E5Error(Core3DTextReliefErrorUnavailable,
                                        @"native host identity is not a UUID");
            return nil;
        }
        issued.host.faceFeature = MintUUID();
        issued.host.frame = frame.identifier;
        issued.host.frameRevision = 1;
        issued.host.shapeDigest = SHA256(issued.sourceBytes);
        issued.boundaryEdges = std::uint16_t(boundary);
        issued.selectorProof = SelectorProof(document->DocumentIdentifier(), entity,
                                             issued.topologyIndex, frame);
        Core3DTextReliefIssuedFace *publicFace =
            [[Core3DTextReliefIssuedFace alloc] initPrivate];
        publicFace.identifier = E5Data(MintUUID());
        publicFace.selectorProof = E5Data(issued.selectorProof);
        publicFace.entityIdentifier = element.entityIdentifier;
        publicFace.topologyIndex = issued.topologyIndex;
        publicFace.boundaryEdgeCount = issued.boundaryEdges;
        publicFace.frameOriginLocal = E5Vector(frame.origin);
        publicFace.frameXAxis = E5Vector(frame.xAxis);
        publicFace.frameYAxis = E5Vector(frame.yAxis);
        publicFace.frameZAxis = E5Vector(frame.zAxis);
        Core3DTextReliefOpening *opening = [[self alloc] initPrivate];
        opening->_controller = controller;
        opening->_document = document;
        opening->_context = std::move(context);
        opening->_issued = std::move(issued);
        opening->_cancelled = std::make_shared<std::atomic_bool>(false);
        opening->_viewportWidth = width;
        opening->_viewportHeight = height;
#if DEBUG
        opening->_debugCancelAfterBuild = std::make_shared<std::atomic_int>(0);
#endif
        opening.issuedFace = publicFace;
        opening.documentIdentifier =
            [NSString stringWithUTF8String:document->DocumentIdentifier().c_str()];
        opening.contextIdentifier = NSUUID.UUID.UUIDString;
        opening.metersPerUnit = metersPerUnit;
        opening.historyDepth = controller.documentUndoCount;
        return opening;
    } @catch (__unused NSException *exception) {
        if (error) *error = E5Error(Core3DTextReliefErrorUnavailable,
                                    @"native capture exception");
        return nil;
    }
}

+ (instancetype)captureController:(Core3DViewController *)controller
         existingFeatureIdentifier:(NSData *)featureIdentifier
                              error:(NSError **)error {
    if (error) *error = nil;
    if (![featureIdentifier isKindOfClass:NSData.class]
        || featureIdentifier.length != 16) {
        if (error) *error = E5Error(Core3DTextReliefErrorInvalidArgument,
                                    @"16-byte relief feature identifier required");
        return nil;
    }
    Core3DTextReliefOpening *opening = [self captureController:controller error:error];
    if (!opening) return nil;
    tr::UUID feature{};
    std::memcpy(feature.data(), featureIdentifier.bytes, feature.size());
    OcctTextReliefCapture capture;
    if (!opening->_document->ReadTextReliefExact(feature, capture)
        || !capture.record.payload
        || capture.record.payload->value.host.entity != opening->_issued.host.entity) {
        if (error) *error = E5Error(Core3DTextReliefErrorStale,
                                    @"retained relief is not owned by selected host");
        return nil;
    }
    opening->_existing = std::move(capture);
    const auto& record = opening->_existing->record.payload->value;
    Core3DTextReliefAuthoredIntent *intent =
        [[Core3DTextReliefAuthoredIntent alloc] initPrivate];
    intent.ownerIdentifier = E5Data(record.owner.entity);
    intent.featureIdentifier = E5Data(record.feature);
    intent.revision = record.revision;
    intent.resourceKind = record.kind == tr::AuthoredKind::Font
        ? Core3DTextReliefResourceKindFontOutline
        : Core3DTextReliefResourceKindEmblemVector;
    intent.catalogHandle =
        [NSString stringWithUTF8String:record.catalogHandle.c_str()];
    intent.text = [NSString stringWithUTF8String:record.textUTF8.c_str()];
    intent.emblemIdentifier =
        [NSString stringWithUTF8String:record.emblemRoot.c_str()];
    intent.heightMM = record.heightMM;
    intent.depthMM = record.depthMM;
    intent.trackingMM = record.trackingMM;
    intent.alignment = static_cast<Core3DTextReliefAlignment>(
        static_cast<NSInteger>(record.alignment));
    intent.mode = static_cast<Core3DTextReliefMode>(
        static_cast<NSInteger>(record.mode));
    intent.offsetUMM = record.offsetUMM;
    intent.offsetVMM = record.offsetVMM;
    intent.rotationRadians = record.rotationRadians;
    intent.metersPerUnit = record.metersPerLocalUnit;
    opening.authoredIntent = intent;
    opening.appliedOwnerIdentifier = E5Data(
        record.owner.entity);
    opening.appliedFeatureIdentifier = featureIdentifier;
    return opening;
}

- (BOOL)isCurrent:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !_controller) {
        if (error) *error = E5Error(Core3DTextReliefErrorInvalidThread,
                                    @"main-thread live controller required");
        return NO;
    }
    if (_cancelled->load()) {
        if (error) *error = E5Refusal(Core3DTextReliefErrorCancelled,
                                      tr::BuildRefusal::Cancelled);
        return NO;
    }
    if (!_context || !_context->isCurrent(_viewportWidth, _viewportHeight)) {
        if (error) *error = E5Error(Core3DTextReliefErrorStale,
                                    @"native opening fence changed");
        return NO;
    }
    Core3DSceneSnapshot *scene = [_controller captureSceneSnapshot];
    if (!scene || scene.selectionMode != Core3DSceneElementKindFace
        || scene.selection.selectedElements.count != 1) {
        if (error) *error = E5Error(Core3DTextReliefErrorStale,
                                    @"native face selection changed");
        return NO;
    }
    Core3DSceneElementIdentifier *element = scene.selection.selectedElements.firstObject;
    if (element.kind != Core3DSceneElementKindFace
        || element.topologyIndex != _issued.topologyIndex
        || _issued.entity != (element.entityIdentifier.UTF8String ?: "")
        || _controller.documentUndoCount != self.historyDepth) {
        if (error) *error = E5Error(Core3DTextReliefErrorStale,
                                    @"captured dependency changed");
        return NO;
    }
    return YES;
}

- (void)cancel { if (_cancelled) _cancelled->store(true); }

// Shared detached raised build core. Precondition: arguments validated and
// (public path) isCurrent held. Everything is computed from the captured
// source bytes; the document is never touched.
- (E5Build)buildRaisedWithSpec:(const E5RequestSpec&)spec
                        budget:(tb::Counter&)budget
                     admitOnly:(BOOL)admitOnly {
    E5Build out;
    @try {
        if (_cancelled->load()) {
            out.refusal = tr::BuildRefusal::Cancelled;
            out.cancelledObserved = true;
            return out;
        }
        const double unit = self.metersPerUnit;
        const double localScale = 0.001 / unit;
        const std::size_t stagesBefore = budget.buildStages;
        const std::size_t visitsBefore = budget.topologyVisits;
        // Resource grammar first: the text advance drives alignment, and the
        // same decode is re-validated by admission below.
        tr::Resource resource;
        const tr::Refusal decoded = tr::DecodeResource(spec.bytes, resource);
        if (decoded != tr::Refusal::None) {
            out.admission = decoded; out.admissionPhase = true; return out;
        }
        if (resource.kind != spec.kind) {
            out.admission = tr::Refusal::UnsupportedKind;
            out.admissionPhase = true; return out;
        }
        const double glyphScale = spec.heightMM / double(resource.unitsPerEm);
        double advanceMM = 0;
        if (spec.kind == tr::ResourceKind::FontOutline) {
            std::vector<std::uint32_t> scalars;
            if (!tr::DecodeUTF8(spec.text, scalars)) {
                out.admission = tr::Refusal::InvalidText;
                out.admissionPhase = true; return out;
            }
            for (std::size_t index = 0; index < scalars.size(); ++index) {
                const auto found = std::lower_bound(resource.items.begin(),
                    resource.items.end(), scalars[index],
                    [](const tr::Item& item, std::uint32_t value) {
                        return item.scalar < value; });
                if (found == resource.items.end() || found->scalar != scalars[index]) {
                    out.admission = tr::Refusal::GlyphUnsupported;
                    out.admissionPhase = true; return out;
                }
                advanceMM += double(found->advance) * glyphScale;
                if (index + 1 != scalars.size()) advanceMM += spec.trackingMM;
            }
        } else {
            if (!spec.text.empty()) {
                out.admission = tr::Refusal::InvalidText;
                out.admissionPhase = true; return out;
            }
            advanceMM = double(resource.items.front().advance) * glyphScale;
        }
        out.advanceMM = advanceMM;
        // Placement frame: in-plane rotation, then U/V and alignment shifts.
        core3d::bounded_curve::Frame frame = _issued.frame;
        if (spec.inwardFrame) {
            // Rotate pi about the x axis: stays right-handed, flips z inward.
            for (int axis = 0; axis < 3; ++axis) {
                frame.yAxis[axis] = -frame.yAxis[axis];
                frame.zAxis[axis] = -frame.zAxis[axis];
            }
        }
        const double cosine = std::cos(spec.rotation), sine = std::sin(spec.rotation);
        std::array<double, 3> xAxis{}, yAxis{};
        for (int axis = 0; axis < 3; ++axis) {
            xAxis[axis] = frame.xAxis[axis] * cosine + frame.yAxis[axis] * sine;
            yAxis[axis] = frame.yAxis[axis] * cosine - frame.xAxis[axis] * sine;
        }
        frame.xAxis = xAxis;
        frame.yAxis = yAxis;
        const double alignmentShift = spec.alignment == 2 ? -advanceMM / 2
            : spec.alignment == 3 ? -advanceMM : 0;
        const double uMM = spec.forceOffset ? spec.forcedOffsetUMM : spec.offsetUMM;
        const double shiftX = (uMM + alignmentShift) * localScale;
        const double shiftY = spec.offsetVMM * localScale;
        for (int axis = 0; axis < 3; ++axis)
            frame.origin[axis] += frame.xAxis[axis] * shiftX
                + frame.yAxis[axis] * shiftY;
        if (core3d::bounded_curve::ValidateFrame(frame)
                != core3d::bounded_curve::Refusal::None) {
            out.admission = tr::Refusal::InvalidLayout;
            out.admissionPhase = true; return out;
        }
        const tr::UUID seed = spec.mintSeed ? MintUUID() : spec.seed;
        tr::Request admission;
        admission.kind = spec.kind;
        admission.rawResourceBytes = spec.bytes;
        admission.expectedResourceSHA256 = spec.expected;
        admission.textUTF8 = spec.text;
        admission.frame = frame;
        admission.identitySeed = seed;
        admission.heightMM = spec.heightMM;
        admission.trackingMM = spec.trackingMM;
        tr::Result admitted;
        const tr::Refusal admittedRefusal = tr::Admit(admission, admitted);
        if (admittedRefusal != tr::Refusal::None) {
            out.admission = admittedRefusal; out.admissionPhase = true; return out;
        }
        out.resourceDigest = admitted.resourceSHA256;
        out.identitySeed = seed;
        out.frame = frame;
        out.admittedCurves = admitted.curves;
        out.admittedContours = admitted.contours;
        out.contourCount = admitted.contours.size();
        for (const tr::CurveRecord& curve : admitted.curves)
            out.curveBytes.push_back(curve.canonicalBytes);
        out.stagesConsumed = budget.buildStages - stagesBefore;
        out.visitsConsumed = budget.topologyVisits - visitsBefore;
        if (admitOnly) {
            out.admittedOnly = true;
            out.built = true;
            return out;
        }
        const TopoDS_Shape host = _issued.sourceShape;
        if (host.IsNull()) {
            out.refusal = tr::BuildRefusal::InvalidHost; return out;
        }
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(host, TopAbs_FACE, faces);
        if (_issued.topologyIndex >= std::uint32_t(faces.Extent())) {
            out.refusal = tr::BuildRefusal::InvalidHost; return out;
        }
        tr::RaisedBuildRequest raised;
        raised.identitySeed = seed;
        raised.frame = frame;
        raised.hostSolid = host;
        raised.hostFace = TopoDS::Face(
            faces.FindKey(Standard_Integer(_issued.topologyIndex) + 1));
        raised.host = _issued.host;
        raised.curves = std::move(admitted.curves);
        raised.contours = std::move(admitted.contours);
        raised.resourceSHA256 = out.resourceDigest;
        raised.depthMM = spec.depthMM;
        raised.metersPerLocalUnit = unit;
        if (spec.foreignDigest) {
            // A receipt bound to another solid's bytes is a foreign host.
            const TopoDS_Shape foreign = BRepPrimAPI_MakeBox(1, 1, 1).Shape();
            std::string foreignBytes;
            if (!core3d::retained_part_boolean::ExactShapeBytes(foreign, foreignBytes)
                || foreignBytes.empty()) {
                out.refusal = tr::BuildRefusal::InternalFailure; return out;
            }
            const std::vector<std::uint8_t> foreignVector(foreignBytes.begin(),
                                                          foreignBytes.end());
            raised.host.shapeDigest = SHA256(foreignVector);
        }
        out.raised = tr::BuildDetachedRaisedRelief(raised, budget, *_cancelled);
        out.stagesConsumed = budget.buildStages - stagesBefore;
        out.visitsConsumed = budget.topologyVisits - visitsBefore;
        out.budgetExhausted = budget.exhausted;
        // Sticky counter: once exhausted, every later charge on this same
        // counter must deny.
        const bool extraCharge = budget.beginStage();
        out.stickyHeld = budget.exhausted ? !extraCharge : true;
        out.cancelledObserved = out.raised.refusal == tr::BuildRefusal::Cancelled;
        if (out.raised.refusal != tr::BuildRefusal::None) {
            out.refusal = out.raised.refusal;
            return out;
        }
#if DEBUG
        if (_debugCancelAfterBuild && _debugCancelAfterBuild->exchange(0) == 1) {
            // Stop arrived during the uninterruptible kernel call: drain
            // without delivering a candidate.
            _cancelled->store(true);
            out.raised = {};
            out.refusal = tr::BuildRefusal::Cancelled;
            out.cancelledObserved = true;
            return out;
        }
#endif
        out.built = true;
        return out;
    } @catch (__unused NSException *exception) {
        out = {};
        out.refusal = tr::BuildRefusal::InternalFailure;
        return out;
    }
}

- (NSError *)errorForBuild:(const E5Build&)build {
    if (build.admissionPhase)
        return E5Refusal(Core3DTextReliefErrorResourceRefused, build.admission);
    if (build.refusal == tr::BuildRefusal::Cancelled)
        return E5Refusal(Core3DTextReliefErrorCancelled, build.refusal);
    if (build.refusal == tr::BuildRefusal::TopologyBudgetExceeded)
        return E5Refusal(Core3DTextReliefErrorBudget, build.refusal);
    return E5Refusal(Core3DTextReliefErrorBuildRefused, build.refusal);
}

- (Core3DTextReliefCandidate *)deliverCandidate:(E5Build&)build {
    const double mmPerLocal = 1000.0 * self.metersPerUnit;
    Core3DTextReliefCandidate *candidate =
        [[Core3DTextReliefCandidate alloc] initPrivate];
    std::string compoundBytes;
    if (!core3d::retained_part_boolean::ExactShapeBytes(build.raised.compound,
                                                        compoundBytes)
        || compoundBytes.empty())
        return nil;
    candidate.candidateBRepBytes = E5Data(compoundBytes.data(), compoundBytes.size());
    const std::vector<std::uint8_t> compoundVector(compoundBytes.begin(),
                                                   compoundBytes.end());
    candidate.geometryDigest = E5Data(SHA256(compoundVector));
    candidate.resourceSHA256 = E5Data(build.resourceDigest);
    NSMutableArray<NSData *> *curves = [NSMutableArray array];
    for (const auto& bytes : build.curveBytes) [curves addObject:E5Data(bytes)];
    candidate.canonicalCurveBytes = curves;
    candidate.metersPerUnit = self.metersPerUnit;
    candidate.textAdvanceMM = build.advanceMM;
    candidate.connectedSolidCount = build.raised.components.size();
    double totalArea = 0, totalVolume = 0;
    NSMutableArray<Core3DTextReliefComponentObservation *> *components =
        [NSMutableArray array];
    for (const tr::RaisedComponent& component : build.raised.components) {
        Core3DTextReliefComponentObservation *observation =
            [[Core3DTextReliefComponentObservation alloc] initPrivate];
        observation.componentIndex = component.component;
        observation.identifier = E5Data(component.identifier);
        observation.areaMM2 = component.areaLocal2 * mmPerLocal * mmPerLocal;
        observation.volumeMM3 = component.volumeLocal3 * mmPerLocal
            * mmPerLocal * mmPerLocal;
        observation.minAlongNormalMM = component.minAlongNormalLocal * mmPerLocal;
        observation.maxAlongNormalMM = component.maxAlongNormalLocal * mmPerLocal;
        totalArea += observation.areaMM2;
        totalVolume += observation.volumeMM3;
        [components addObject:observation];
    }
    candidate.componentObservations = components;
    candidate.totalAreaMM2 = totalArea;
    candidate.totalVolumeMM3 = totalVolume;
    return candidate;
}

- (Core3DTextReliefCandidate *)detachedRaisedCandidateWithResourceBytes:(NSData *)resourceBytes
        expectedSHA256:(NSString *)expectedSHA256 kind:(Core3DTextReliefResourceKind)kind
        text:(NSString *)text heightMM:(double)heightMM depthMM:(double)depthMM
        trackingMM:(double)trackingMM alignment:(Core3DTextReliefAlignment)alignment
        offsetUMM:(double)offsetUMM offsetVMM:(double)offsetVMM
        rotationRadians:(double)rotationRadians mode:(Core3DTextReliefMode)mode
        error:(NSError **)error {
    if (error) *error = nil;
    if (mode == Core3DTextReliefModeEmboss || mode == Core3DTextReliefModeDeboss) {
        if (error) *error = E5Refusal(Core3DTextReliefErrorModeUnavailable,
                                      "e5.mode-unavailable", static_cast<int>(mode));
        return nil;
    }
    if (mode != Core3DTextReliefModeRaised
        || (kind != Core3DTextReliefResourceKindFontOutline
            && kind != Core3DTextReliefResourceKindEmblemVector)
        || (alignment != Core3DTextReliefAlignmentLeft
            && alignment != Core3DTextReliefAlignmentCenter
            && alignment != Core3DTextReliefAlignmentRight)
        || ![resourceBytes isKindOfClass:NSData.class] || resourceBytes.length == 0
        || ![text isKindOfClass:NSString.class]
        || !std::isfinite(heightMM) || !std::isfinite(depthMM)
        || !std::isfinite(trackingMM) || !std::isfinite(offsetUMM)
        || !std::isfinite(offsetVMM) || !std::isfinite(rotationRadians)) {
        if (error) *error = E5Error(Core3DTextReliefErrorInvalidArgument,
                                    @"finite raised text/emblem arguments required");
        return nil;
    }
    E5RequestSpec spec;
    spec.bytes.resize(resourceBytes.length);
    std::memcpy(spec.bytes.data(), resourceBytes.bytes, resourceBytes.length);
    const char *hex = expectedSHA256.UTF8String;
    if (!hex || std::strlen(hex) != 64) {
        if (error) *error = E5Error(Core3DTextReliefErrorInvalidArgument,
                                    @"64 lowercase hex characters required");
        return nil;
    }
    for (unsigned index = 0; index < 32; ++index) {
        unsigned value = 0;
        for (unsigned half = 0; half < 2; ++half) {
            const char c = hex[index * 2 + half];
            value <<= 4;
            if (c >= '0' && c <= '9') value |= unsigned(c - '0');
            else if (c >= 'a' && c <= 'f') value |= unsigned(c - 'a' + 10);
            else {
                if (error) *error = E5Error(Core3DTextReliefErrorInvalidArgument,
                                            @"64 lowercase hex characters required");
                return nil;
            }
        }
        spec.expected[index] = std::uint8_t(value);
    }
    spec.kind = tr::ResourceKind(kind);
    spec.text = text.UTF8String ?: "";
    spec.heightMM = heightMM;
    spec.depthMM = depthMM;
    spec.trackingMM = trackingMM;
    spec.alignment = int(alignment);
    spec.offsetUMM = offsetUMM;
    spec.offsetVMM = offsetVMM;
    spec.rotation = rotationRadians;
    if (![self isCurrent:error]) return nil;
    tb::Counter budget;
    E5Build build = [self buildRaisedWithSpec:spec budget:budget admitOnly:NO];
    if (!build.built) {
        if (error) *error = [self errorForBuild:build];
        return nil;
    }
    if (_cancelled->load()) {
        if (error) *error = E5Refusal(Core3DTextReliefErrorCancelled,
                                      tr::BuildRefusal::Cancelled);
        return nil;
    }
    Core3DTextReliefCandidate *candidate = [self deliverCandidate:build];
    candidate.e5ResourceBytes = resourceBytes;
    candidate.e5IdentitySeed = E5Data(build.identitySeed);
    candidate.e5OpeningContextIdentifier = self.contextIdentifier;
    candidate.e5Kind = kind;
    candidate.e5Text = text;
    candidate.e5HeightMM = heightMM;
    candidate.e5DepthMM = depthMM;
    candidate.e5TrackingMM = trackingMM;
    candidate.e5Alignment = alignment;
    candidate.e5OffsetUMM = offsetUMM;
    candidate.e5OffsetVMM = offsetVMM;
    candidate.e5RotationRadians = rotationRadians;
    if (!candidate && error)
        *error = E5Refusal(Core3DTextReliefErrorBuildRefused,
                           tr::BuildRefusal::InternalFailure);
    return candidate;
}

- (Core3DTextReliefOpening *)openingByPreparingCandidate:(Core3DTextReliefCandidate *)candidate
                                                   error:(NSError **)error {
    if (error) *error = E5Error(Core3DTextReliefErrorApplyUnavailable,
                                @"candidate prepare is unavailable in S1");
    return nil;
}

- (Core3DTextReliefOpening *)openingByPreparingCandidate:(Core3DTextReliefCandidate *)candidate
                                           catalogHandle:(NSString *)catalogHandle
                                                   error:(NSError **)error {
    if (error) *error = nil;
    if (![candidate isKindOfClass:Core3DTextReliefCandidate.class]
        || ![catalogHandle isKindOfClass:NSString.class]
        || catalogHandle.length == 0 || catalogHandle.length > 128
        || candidate.e5ResourceBytes.length == 0
        || candidate.e5IdentitySeed.length != 16
        || ![candidate.e5OpeningContextIdentifier isEqual:self.contextIdentifier]
        || _preparedCandidate || _preparedRemoval || ![self isCurrent:error]) {
        if (error && !*error) *error = E5Error(Core3DTextReliefErrorInvalidArgument,
                                               @"candidate preparation refused");
        return nil;
    }
    tr::Digest candidateDigest{};
    if (candidate.resourceSHA256.length != candidateDigest.size()) {
        if (error) *error = E5Error(Core3DTextReliefErrorResourceRefused,
                                    @"candidate resource pin unavailable");
        return nil;
    }
    std::memcpy(candidateDigest.data(), candidate.resourceSHA256.bytes,
                candidateDigest.size());
    if (!E5CatalogPin(catalogHandle, candidateDigest,
                      candidate.e5ResourceBytes.length,
                      tr::ResourceKind(candidate.e5Kind))) {
        if (error) *error = E5Error(Core3DTextReliefErrorResourceRefused,
                                    @"catalog resource pin mismatch");
        return nil;
    }
    if (_existing && _existing->record.payload) {
        const auto& prior = _existing->record.payload->value;
        NSString *priorHandle = [NSString stringWithUTF8String:prior.catalogHandle.c_str()];
        const tr::ResourceKind priorKind = prior.kind == tr::AuthoredKind::Font
            ? tr::ResourceKind::FontOutline : tr::ResourceKind::EmblemVector;
        if (!E5CatalogPin(priorHandle, prior.resourceSHA256,
                          prior.resourceByteCount, priorKind)) {
            if (error) *error = E5Error(Core3DTextReliefErrorResourceRefused,
                                        @"retained catalog resource unavailable");
            return nil;
        }
    }
    Core3DTextReliefOpening *prepared = [[Core3DTextReliefOpening alloc] initPrivate];
    prepared->_controller = _controller; prepared->_document = _document;
    prepared->_context = _context; prepared->_issued = _issued;
    prepared->_cancelled = _cancelled; prepared->_viewportWidth = _viewportWidth;
    prepared->_viewportHeight = _viewportHeight; prepared->_existing = _existing;
#if DEBUG
    prepared->_debugCancelAfterBuild = _debugCancelAfterBuild;
    prepared->_debugApplyFault = _debugApplyFault;
#endif
    prepared.issuedFace = self.issuedFace;
    prepared.documentIdentifier = self.documentIdentifier;
    prepared.contextIdentifier = self.contextIdentifier;
    prepared.metersPerUnit = self.metersPerUnit;
    prepared.historyDepth = self.historyDepth;
    prepared.appliedOwnerIdentifier = self.appliedOwnerIdentifier;
    prepared.appliedFeatureIdentifier = self.appliedFeatureIdentifier;
    prepared->_preparedCandidate = candidate;
    prepared->_preparedCatalogHandle = [catalogHandle copy];
    return prepared;
}

- (Core3DTextReliefOpening *)openingByPreparingRemovalWithError:(NSError **)error {
    if (error) *error = nil;
    if (!_existing || _preparedCandidate || _preparedRemoval || ![self isCurrent:error]) {
        if (error && !*error) *error = E5Error(Core3DTextReliefErrorInvalidArgument,
                                               @"existing relief required for removal");
        return nil;
    }
    Core3DTextReliefOpening *prepared = [[Core3DTextReliefOpening alloc] initPrivate];
    prepared->_controller = _controller; prepared->_document = _document;
    prepared->_context = _context; prepared->_issued = _issued;
    prepared->_cancelled = _cancelled; prepared->_viewportWidth = _viewportWidth;
    prepared->_viewportHeight = _viewportHeight; prepared->_existing = _existing;
#if DEBUG
    prepared->_debugCancelAfterBuild = _debugCancelAfterBuild;
    prepared->_debugApplyFault = _debugApplyFault;
#endif
    prepared.issuedFace = self.issuedFace; prepared.documentIdentifier = self.documentIdentifier;
    prepared.contextIdentifier = self.contextIdentifier; prepared.metersPerUnit = self.metersPerUnit;
    prepared.historyDepth = self.historyDepth; prepared.appliedOwnerIdentifier = self.appliedOwnerIdentifier;
    prepared.appliedFeatureIdentifier = self.appliedFeatureIdentifier;
    prepared->_preparedRemoval = true;
    return prepared;
}

- (Core3DTextReliefApplyResult)apply {
    namespace bc = core3d::bounded_curve;
    if (![NSThread isMainThread] || !_document || !_context
        || (!_preparedCandidate && !_preparedRemoval) || _cancelled->load())
        return Core3DTextReliefApplyResultUnavailable;
#if DEBUG
    if (_debugApplyFault == 1) return Core3DTextReliefApplyResultRefused;
#endif
    try {
        core3d::native_opening::CommittedEditPublication publication;
        const int undoBefore = _document->Document()->GetAvailableUndos();
        if (_preparedRemoval) {
            if (!_existing || !_existing->record.payload) return Core3DTextReliefApplyResultRefused;
            publication.removed.push_back({_existing->ownerReceipt.visibility.object.object.entityIdentifier,
                                           _existing->record.shape});
            for (const auto& curve : _existing->curves)
                publication.removed.push_back({
                    curve.ownerReceipt.visibility.object.object.entityIdentifier,
                    curve.record.current});
            auto lease = _context->beginCommandLease(
                _context->openingFence(), _viewportWidth, _viewportHeight);
            if (!lease) return Core3DTextReliefApplyResultRefused;
#if DEBUG
            if (_debugApplyFault == 5) _context->debugReportNextCloseUnproven();
#endif
            // D1028: measure the E5 multi-root removal side before staging;
            // the companion settles on the verified prior state after every
            // abort and binds to the real committed delta after a proven
            // commit, so the in-session removal/undo observations are exact.
            if (!_document->CaptureTextReliefRemovalCompanionBefore(*_existing)) {
                return lease->abort() ? Core3DTextReliefApplyResultRefused
                                      : Core3DTextReliefApplyResultRecoveryRequired;
            }
            if (!_document->StageTextReliefRemove(*lease, *_existing)
                || !_document->CaptureTextReliefRemovalCompanionAfter()) {
                if (!lease->abort())
                    return Core3DTextReliefApplyResultRecoveryRequired;
                return _document->SettleTextReliefRemovalHistoryOnPrior()
                    ? Core3DTextReliefApplyResultRefused
                    : Core3DTextReliefApplyResultRecoveryRequired;
            }
            _recoveryFeature = _existing->record.payload->value.feature;
            _recoveryExpectsPresent = false;
#if DEBUG
            if (_debugApplyFault >= 2 && _debugApplyFault <= 4) {
                if (!lease->abort())
                    return Core3DTextReliefApplyResultRecoveryRequired;
                return _document->SettleTextReliefRemovalHistoryOnPrior()
                    ? Core3DTextReliefApplyResultRefused
                    : Core3DTextReliefApplyResultRecoveryRequired;
            }
#endif
            if (!lease->commit()) {
                // The close is unproven: bind the companion to the live delta
                // if the commit actually landed, otherwise settle on the
                // verified prior state.
                if (!_document->FinalizeTextReliefRemovalHistory()
                    && !_document->SettleTextReliefRemovalHistoryOnPrior())
                    return Core3DTextReliefApplyResultRecoveryRequired;
                _context->retainUnprovenEdit(publication);
                return Core3DTextReliefApplyResultRecoveryRequired;
            }
            if (_document->Document()->GetAvailableUndos() - undoBefore != 1
                || !_document->FinalizeTextReliefRemovalHistory())
                return Core3DTextReliefApplyResultRecoveryRequired;
            if (!_context->publishCommittedEdit(publication))
                return Core3DTextReliefApplyResultRecoveryRequired;
            _existing.reset(); self.appliedOwnerIdentifier = nil;
            self.appliedFeatureIdentifier = nil;
            _recoveryFeature.reset();
            return Core3DTextReliefApplyResultApplied;
        }

        Core3DTextReliefCandidate *candidate = _preparedCandidate;
        E5RequestSpec spec;
        spec.bytes.resize(candidate.e5ResourceBytes.length);
        std::memcpy(spec.bytes.data(), candidate.e5ResourceBytes.bytes, spec.bytes.size());
        std::memcpy(spec.expected.data(), candidate.resourceSHA256.bytes, spec.expected.size());
        std::memcpy(spec.seed.data(), candidate.e5IdentitySeed.bytes, spec.seed.size());
        spec.mintSeed = false; spec.kind = tr::ResourceKind(candidate.e5Kind);
        spec.text = candidate.e5Text.UTF8String ?: "";
        spec.heightMM = candidate.e5HeightMM; spec.depthMM = candidate.e5DepthMM;
        spec.trackingMM = candidate.e5TrackingMM; spec.alignment = int(candidate.e5Alignment);
        spec.offsetUMM = candidate.e5OffsetUMM; spec.offsetVMM = candidate.e5OffsetVMM;
        spec.rotation = candidate.e5RotationRadians;
        tb::Counter budget;
        E5Build build = [self buildRaisedWithSpec:spec budget:budget admitOnly:NO];
        std::string exactShape;
        if (!build.built || build.admittedCurves.empty()
            || !core3d::retained_part_boolean::ExactShapeBytes(build.raised.compound, exactShape)
            || candidate.candidateBRepBytes.length != exactShape.size()
            || std::memcmp(candidate.candidateBRepBytes.bytes, exactShape.data(), exactShape.size()) != 0
            || candidate.canonicalCurveBytes.count != build.admittedCurves.size())
            return Core3DTextReliefApplyResultRefused;
        for (std::size_t i = 0; i < build.admittedCurves.size(); ++i)
            if (![candidate.canonicalCurveBytes[i] isEqual:E5Data(build.admittedCurves[i].canonicalBytes)])
                return Core3DTextReliefApplyResultRefused;

        OcctExactLabelReceipt host;
        TDF_Label hostLabel;
        if (!RootForEntity(_document, self.issuedFace.entityIdentifier, hostLabel)
            || !_document->CaptureExactFreeLabel(hostLabel, host))
            return Core3DTextReliefApplyResultRefused;
        const std::size_t curveCount = build.admittedCurves.size();
        std::vector<OcctIssuedLabelIdentity> issued;
        if (!_document->ReserveExactLabelIdentities(
                curveCount + (_existing ? 0 : 1), {}, issued)
            || issued.size() != curveCount + (_existing ? 0 : 1))
            return Core3DTextReliefApplyResultRefused;

        tr::UUID documentUUID{};
        if (!ParseUUIDText(_document->DocumentIdentifier(), documentUUID))
            return Core3DTextReliefApplyResultRefused;
        std::vector<bc::PersistedValue> persisted(curveCount);
        std::vector<bc::DetachedWire> detached(curveCount);
        tr::AuthoredRecord record;
        for (std::size_t i = 0; i < curveCount; ++i) {
            persisted[i].value = build.admittedCurves[i].value;
            auto& owner = persisted[i].ownerState;
            owner.owner.document = documentUUID;
            if (!ParseUUIDText(issued[i].EntityIdentifier(), owner.owner.entity)
                || !ParseUUIDText(issued[i].DefinitionIdentifier(), owner.owner.definition))
                return Core3DTextReliefApplyResultRefused;
            owner.feature = persisted[i].value.feature;
            owner.definitionRevision = 1;
            owner.nextLocalID = persisted[i].value.definition.controlPoints.size() + 1;
            std::vector<std::uint8_t> definitionBytes;
            if (!bc::Encode(persisted[i].value, definitionBytes)
                || !bc::Hash(definitionBytes, bc::MaximumDefinitionBytes,
                             owner.canonicalDefinitionDigest)
                || bc::BuildWire(persisted[i], detached[i]) != bc::BuildRefusal::None)
                return Core3DTextReliefApplyResultRefused;
            tr::Digest digest{};
            if (!bc::Hash(build.admittedCurves[i].canonicalBytes,
                          bc::MaximumDefinitionBytes, digest))
                return Core3DTextReliefApplyResultRefused;
            record.curves.push_back({build.admittedCurves[i].value.feature, digest});
        }
        for (const auto& contour : build.admittedContours) {
            double area = 0;
            for (std::size_t i = 0; i < contour.curveCount; ++i) {
                const auto& points = build.admittedCurves[contour.firstCurve + i]
                    .value.definition.controlPoints;
                for (std::size_t p = 0; p + 1 < points.size(); ++p)
                    area += points[p].local[0] * points[p + 1].local[1]
                        - points[p + 1].local[0] * points[p].local[1];
            }
            record.loops.push_back({std::uint32_t(contour.component),
                std::uint32_t(contour.contour), std::uint32_t(contour.firstCurve),
                std::uint32_t(contour.curveCount),
                area < 0 ? tr::LoopRole::Hole : tr::LoopRole::Outer});
        }
        if (_existing) {
            record = _existing->record.payload->value;
            record.revision += 1; record.curves.clear(); record.loops.clear();
            for (std::size_t i = 0; i < curveCount; ++i) {
                tr::Digest digest{};
                bc::Hash(build.admittedCurves[i].canonicalBytes,
                         bc::MaximumDefinitionBytes, digest);
                record.curves.push_back({build.admittedCurves[i].value.feature, digest});
            }
            for (const auto& contour : build.admittedContours)
                record.loops.push_back({std::uint32_t(contour.component),
                    std::uint32_t(contour.contour), std::uint32_t(contour.firstCurve),
                    std::uint32_t(contour.curveCount), tr::LoopRole::Outer});
        } else {
            record.owner.document = documentUUID;
            if (!ParseUUIDText(issued.back().EntityIdentifier(), record.owner.entity)
                || !ParseUUIDText(issued.back().DefinitionIdentifier(), record.owner.definition))
                return Core3DTextReliefApplyResultRefused;
            record.feature = MintUUID(); record.constructionSeed = build.identitySeed;
            record.rootOutput = record.owner.entity; record.revision = 1;
            record.host.entity = _issued.host.entity;
            record.host.faceFeature = _issued.host.faceFeature;
            record.host.frame = _issued.frame.identifier; record.host.frameRevision = 1;
            record.host.shapeDigest = _issued.host.shapeDigest;
            const char *domains[] = {"source", "geometry", "placement", "material", "group"};
            tr::Digest *targets[] = {&record.host.sourceCommitment,
                &record.host.geometryCommitment, &record.host.placementCommitment,
                &record.host.materialCommitment, &record.host.groupCommitment};
            for (int d = 0; d < 5; ++d) {
                std::vector<std::uint8_t> material(_issued.sourceBytes);
                material.insert(material.end(), domains[d], domains[d] + std::strlen(domains[d]));
                *targets[d] = SHA256(material);
            }
            record.host.selectorProof.assign(_issued.selectorProof.begin(), _issued.selectorProof.end());
        }
        record.host.origin = build.frame.origin; record.host.xAxis = build.frame.xAxis;
        record.host.yAxis = build.frame.yAxis; record.host.zAxis = build.frame.zAxis;
        for (auto values : {&record.host.origin, &record.host.xAxis,
                            &record.host.yAxis, &record.host.zAxis})
            for (double& value : *values) if (value == 0) value = 0.0;
        record.kind = candidate.e5Kind == Core3DTextReliefResourceKindFontOutline
            ? tr::AuthoredKind::Font : tr::AuthoredKind::Emblem;
        record.catalogHandle = _preparedCatalogHandle.UTF8String ?: "";
        record.resourceByteCount = std::uint32_t(candidate.e5ResourceBytes.length);
        record.resourceSHA256 = build.resourceDigest;
        record.textUTF8 = spec.text; record.script = "latin";
        record.emblemRoot = record.kind == tr::AuthoredKind::Emblem
            ? record.catalogHandle : "";
        record.heightMM = spec.heightMM; record.trackingMM = spec.trackingMM;
        record.depthMM = spec.depthMM; record.metersPerLocalUnit = self.metersPerUnit;
        record.offsetUMM = spec.offsetUMM; record.offsetVMM = spec.offsetVMM;
        record.rotationRadians = spec.rotation;
        record.alignment = tr::AuthoredAlignment(spec.alignment);
        record.mode = tr::AuthoredMode::Raised;
        std::vector<tr::AuthoredComponent> components;
        for (const auto& component : build.raised.components) {
            tr::UUID identifier = component.identifier;
            if (_existing)
                for (const auto& old : _existing->record.payload->value.components)
                    if (old.key == component.component) identifier = old.identifier;
            components.push_back({identifier, component.component});
        }
        record.components = std::move(components);
        // Surviving C1 identities: a curve whose canonical definition is
        // unchanged keeps its existing feature UUID (and its label, staged
        // below); genuinely new curves keep their minted identities and
        // removed ones are never revived by position.
        std::set<tr::UUID> reusedCurveFeatures;
        if (_existing) {
            std::map<tr::Digest, std::vector<tr::UUID>> surviving;
            for (const auto& old : _existing->record.payload->value.curves)
                surviving[old.digest].push_back(old.feature);
            for (auto& entry : record.curves) {
                auto found = surviving.find(entry.digest);
                if (found == surviving.end() || found->second.empty()) continue;
                entry.feature = found->second.back();
                found->second.pop_back();
                reusedCurveFeatures.insert(entry.feature);
            }
        }
        const tr::AuthoredRefusal recordRefusal = tr::Validate(record);
#if DEBUG
        if (recordRefusal != tr::AuthoredRefusal::None)
            NSLog(@"E5_APPLY refused=record-validation raw=%d",
                  static_cast<int>(recordRefusal));
#endif
        if (recordRefusal != tr::AuthoredRefusal::None)
            return Core3DTextReliefApplyResultRefused;

        auto lease = _context->beginCommandLease(
            _context->openingFence(), _viewportWidth, _viewportHeight);
        if (!lease) return Core3DTextReliefApplyResultRefused;
#if DEBUG
        if (_debugApplyFault == 5) _context->debugReportNextCloseUnproven();
#endif
        std::vector<OcctBoundedCurveCapture> stagedCurves;
        for (std::size_t i = 0; i < curveCount; ++i) {
            if (reusedCurveFeatures.count(record.curves[i].feature))
                continue;
            OcctBoundedCurveCapture staged;
            if (!_document->StageBoundedCurveCreate(*lease, issued[i], persisted[i],
                    detached[i], "Text relief contour", staged)) {
#if DEBUG
                NSLog(@"E5_APPLY refused=curve-stage index=%zu", i);
#endif
                return lease->abort() ? Core3DTextReliefApplyResultRefused
                                      : Core3DTextReliefApplyResultRecoveryRequired;
            }
            stagedCurves.push_back(staged);
            publication.created.push_back({issued[i].EntityIdentifier(), {}});
#if DEBUG
            if (_debugApplyFault == 2) {
                return lease->abort() ? Core3DTextReliefApplyResultRefused
                                      : Core3DTextReliefApplyResultRecoveryRequired;
            }
#endif
        }
        OcctPreparedLabelClone clone; clone.source = host;
        clone.detachedShape = build.raised.compound;
        clone.representation = OcctGeometryRepresentation::BRep;
        OcctTextReliefCapture staged;
        bool stagedOwner = false;
        if (_existing) {
            stagedOwner = _document->StageTextReliefReplacement(
                *lease, *_existing, clone, record, staged);
            publication.replaced.push_back({_existing->ownerReceipt.visibility.object.object.entityIdentifier,
                                            _existing->record.shape});
            if (stagedOwner) for (const auto& old : _existing->curves) {
                if (old.record.value
                    && reusedCurveFeatures.count(
                        old.record.value->persisted.value.feature))
                    continue;
                publication.removed.push_back({
                    old.ownerReceipt.visibility.object.object.entityIdentifier,
                    old.record.current});
                if (!_document->StageRemoveExactFreeLabel(*lease, old.ownerReceipt)) {
                    stagedOwner = false; break;
                }
            }
        } else {
            stagedOwner = _document->StageTextReliefCreate(
                *lease, issued.back(), clone, record, staged);
            publication.created.push_back({issued.back().EntityIdentifier(), {}});
        }
        if (!stagedOwner) {
#if DEBUG
            NSLog(@"E5_APPLY refused=owner-stage existing=%d", _existing.has_value());
#endif
            return lease->abort() ? Core3DTextReliefApplyResultRefused
                                  : Core3DTextReliefApplyResultRecoveryRequired;
        }
        _recoveryFeature = record.feature;
        _recoveryExpectsPresent = true;
#if DEBUG
        if (_debugApplyFault == 3 || _debugApplyFault == 4) {
            return lease->abort() ? Core3DTextReliefApplyResultRefused
                                  : Core3DTextReliefApplyResultRecoveryRequired;
        }
#endif
        if (!lease->commit()) {
            _context->retainUnprovenEdit(publication);
            return Core3DTextReliefApplyResultRecoveryRequired;
        }
        OcctTextReliefCapture read;
        if (_document->Document()->GetAvailableUndos() - undoBefore != 1
            || !_document->ReadTextReliefExact(record.feature, read)
            || !staged.IsEqual(read))
            return Core3DTextReliefApplyResultRecoveryRequired;
#if DEBUG
        if (_debugApplyFault == 6) {
            // The command committed, but its first publication is unproven.
            // Retain the actual plan so reconciliation can publish it once.
            _context->retainUnprovenEdit(publication);
            return Core3DTextReliefApplyResultRecoveryRequired;
        }
#endif
        if (!_context->publishCommittedEdit(publication))
            return Core3DTextReliefApplyResultRecoveryRequired;
        _existing = std::move(read);
        self.appliedOwnerIdentifier = E5Data(record.owner.entity);
        self.appliedFeatureIdentifier = E5Data(record.feature);
        _recoveryFeature.reset();
        return Core3DTextReliefApplyResultApplied;
    } catch (...) { return Core3DTextReliefApplyResultRefused; }
}

- (BOOL)reconcileRecovery {
    if (![NSThread isMainThread] || !_context || !_recoveryFeature
        || _document.IsNull() || _document->Document().IsNull()
        || _document->Document()->HasOpenCommand()) return NO;
    OcctTextReliefCapture actual;
    const bool present = _document->ReadTextReliefExact(*_recoveryFeature, actual);
    if (present != _recoveryExpectsPresent) return NO;
    const bool settled = _context->reconcileRecovery(true);
    if (settled) {
        if (present) {
            _existing = std::move(actual);
            self.appliedOwnerIdentifier = E5Data(
                _existing->record.payload->value.owner.entity);
            self.appliedFeatureIdentifier = E5Data(*_recoveryFeature);
        } else {
            _existing.reset(); self.appliedOwnerIdentifier = nil;
            self.appliedFeatureIdentifier = nil;
        }
        _recoveryFeature.reset();
    }
    return settled;
}

+ (BOOL)isPubliclyAvailable { return YES; }

#if DEBUG
- (void)e5DebugArmCancelAfterBuild {
    if (_debugCancelAfterBuild) _debugCancelAfterBuild->store(1);
}

- (void)e5DebugArmApplyFault:(int)fault { _debugApplyFault = fault; }

- (BOOL)e5DebugArmReplayPairFault {
    return _existing && _existing->record.payload && !_document.IsNull()
        && _document->DebugArmTextReliefReplayPairFault(
            _existing->record.payload->value.feature);
}

- (NSDictionary *)e5DebugReplayObservation {
    if (!_existing || !_existing->record.payload || _document.IsNull())
        return @{@"ok": @NO};
    core3d::treatment_history::TextReliefReplaySide side;
    if (!_document->DebugCaptureTextReliefReplaySide(
            _existing->record.payload->value.feature, side)
        || side.owners.size() != 1) return @{@"ok": @NO};
    const auto& capture = side.owners.front();
    const auto shapeData = [](const TopoDS_Shape& shape) -> NSData * {
        if (shape.IsNull()) return [NSData data];
        std::string bytes;
        if (!core3d::retained_part_boolean::ExactShapeBytes(shape, bytes)) return nil;
        return E5Data(bytes.data(), bytes.size());
    };
    NSMutableArray *naming = [NSMutableArray array], *flags = [NSMutableArray array];
    for (const auto& state : side.naming) {
        NSMutableArray *pairs = [NSMutableArray array];
        for (const auto& pair : state.shapePairs) {
            NSData *old = shapeData(pair.first), *next = shapeData(pair.second);
            if (!old || !next) return @{@"ok": @NO};
            [pairs addObject:@[old, next]];
        }
        NSData *binding = shapeData(state.currentBinding);
        if (!binding) return @{@"ok": @NO};
        [naming addObject:@{ @"label": @(state.labelEntry.c_str()),
            @"version": @(state.version), @"evolution": @(int(state.evolution)),
            @"pairs": pairs, @"binding": binding }];
    }
    for (const auto& root : side.flags) {
        NSMutableArray *values = [NSMutableArray array];
        for (const auto& flag : root)
            [values addObject:@[@(flag.modified), @(flag.checked)]];
        [flags addObject:values];
    }
    NSMutableArray *features = [NSMutableArray array], *definitions = [NSMutableArray array];
    NSMutableArray *owners = [NSMutableArray array], *wires = [NSMutableArray array];
    NSMutableArray *curveNaming = [NSMutableArray array];
    for (const auto& curve : capture.curves) {
        NSData *shape = shapeData(curve.wire);
        if (!shape) return @{@"ok": @NO};
        [features addObject:E5Data(curve.persisted.value.feature)];
        [definitions addObject:E5Data(curve.record.value->definitionBytes)];
        [owners addObject:E5Data(curve.record.value->ownerBytes)];
        [wires addObject:shape];
        std::optional<Standard_Integer> ownerVersion, recordVersion;
        for (const auto& state : side.naming) {
            if (state.label.IsEqual(curve.record.owner)) ownerVersion = state.version;
            if (state.label.IsEqual(curve.record.label)) recordVersion = state.version;
        }
        if (!ownerVersion || !recordVersion) return @{@"ok": @NO};
        [curveNaming addObject:@[@(*ownerVersion), @(*recordVersion)]];
    }
    NSData *shape = shapeData(capture.record.shape);
    if (!shape) return @{@"ok": @NO};
    NSMutableDictionary *result = [@{ @"ok": @YES,
        @"recordBytes": E5Data(capture.record.payload->bytes), @"shapeBytes": shape,
        @"curveFeatures": features, @"curveDefinitionBytes": definitions,
        @"curveOwnerBytes": owners, @"curveShapeBytes": wires,
        @"curveNaming": curveNaming, @"memberNaming": naming, @"memberFlags": flags } mutableCopy];
    const auto native = _document->Document();
    const auto shapes = XCAFDoc_DocumentTool::ShapeTool(native->Main());
    TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
    const auto host = core3d::retained_solid::UUIDText(capture.record.payload->value.host.entity);
    for (Standard_Integer index = 1; index <= roots.Length(); ++index)
        if (_document->EntityIdentifierForLabel(roots.Value(index)) == host) {
            core3d::variable_radius_fillet::owner::Capture source;
            if (!_document->CaptureVariableRadiusFilletOwner(roots.Value(index), source))
                return @{@"ok": @NO};
            result[@"sourceOwnerBytes"] = E5Data(source.ownerBytes);
            result[@"sourceLawBytes"] = E5Data(source.lawBytes);
            result[@"sourceBaseBytes"] = E5Data(source.baseBytes);
            result[@"sourceShapeBytes"] = E5Data(source.currentBytes);
        }
    return result;
}

// Negative proposals use copied native source topology and the production
// source replay preparer. They never install topology or fabricate a record.
- (NSDictionary *)e5DebugReplayRefusal:(int)scenario {
    if (!_existing || !_existing->record.payload || _document.IsNull()
        || !_context || scenario < 1 || scenario > 3) return @{@"ok": @NO};
    namespace dr = core3d::dependent_replay;
    const auto native = _document->Document();
    const auto shapes = XCAFDoc_DocumentTool::ShapeTool(native->Main());
    TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
    const auto& record = _existing->record.payload->value;
    const auto host = core3d::retained_solid::UUIDText(record.host.entity);
    TDF_Label source;
    for (Standard_Integer index = 1; index <= roots.Length(); ++index)
        if (_document->EntityIdentifierForLabel(roots.Value(index)) == host)
            source = roots.Value(index);
    OcctExactLabelReceipt receipt;
    if (source.IsNull() || native->HasOpenCommand()
        || !_document->CaptureExactFreeLabel(source, receipt)) return @{@"ok": @NO};
    // BRep_Builder::Add changes topology bookkeeping on its child. Negative
    // proposals must own copied topology before assembling any container.
    BRepBuilderAPI_Copy detachedSource(XCAFDoc_ShapeTool::GetShape(source));
    if (!detachedSource.IsDone()) return @{@"ok": @NO};
    const TopoDS_Shape original = detachedSource.Shape();
    BRep_Builder builder;
    TopoDS_Compound compound; builder.MakeCompound(compound);
    if (scenario == 2) {
        builder.Add(compound, original);
        BRepBuilderAPI_Copy copy(original);
        if (!copy.IsDone()) return @{@"ok": @NO};
        builder.Add(compound, copy.Shape());
    } else {
        for (TopExp_Explorer face(original, TopAbs_FACE); face.More(); face.Next()) {
            const TopoDS_Face value = TopoDS::Face(face.Current());
            BRepAdaptor_Surface surface(value);
            bool isHostNormal = false;
            if (surface.GetType() == GeomAbs_Plane) {
                gp_Dir normal = surface.Plane().Axis().Direction();
                if (value.Orientation() == TopAbs_REVERSED) normal.Reverse();
                isHostNormal = std::abs(normal.X() - record.host.zAxis[0]) < 1e-9
                    && std::abs(normal.Y() - record.host.zAxis[1]) < 1e-9
                    && std::abs(normal.Z() - record.host.zAxis[2]) < 1e-9;
            }
            if (scenario != 1 || !isHostNormal) builder.Add(compound, value);
        }
    }
    dr::Candidate candidate; candidate.targetEntityIdentifier = host;
    candidate.resultShape = compound;
    dr::ProductionPreparer preparer(std::move(candidate), _context);
    std::shared_ptr<const dr::Plan> plan;
    const int before = native->GetAvailableUndos();
    const auto refusal = _document->PrepareDependentReplayPlan(receipt,
        dr::Mutation::Replace, dr::Limits{}, preparer, compound, plan);
    OcctExactLabelReceipt after;
    const bool sourceEqual = _document->ReadExactFreeLabel(receipt, after);
    return @{ @"ok": @YES, @"refusalRaw": @(int(refusal)), @"prepared": @(bool(plan)),
        @"sourceEqual": @(sourceEqual), @"openCommand": @(native->HasOpenCommand()),
        @"historyBefore": @(before), @"historyAfter": @(native->GetAvailableUndos()) };
}

- (NSDictionary *)e5DebugObservation {
    NSMutableDictionary *result = [NSMutableDictionary dictionaryWithObject:@NO forKey:@"ok"];
    if (!_existing || !_existing->record.payload) return result;
    const auto& record = _existing->record.payload->value;
    std::vector<std::uint8_t> bytes;
    std::string shapeBytes;
    // D1003: the cross-session exactness obligation is on the canonical
    // persisted representation, so the observation is the persisted form on
    // both sides: write, read back with the matching reader, write again
    // (the D981 item-1 pattern). One pass is a fixed point for the measured
    // reader-renormalised direction coefficients; in-session comparisons
    // stay exact.
    TopoDS_Shape persistedShape;
    std::string persistedShapeBytes;
    if (!tr::EncodeAuthored(record, bytes)
        || !core3d::retained_part_boolean::ExactShapeBytes(
            _existing->record.shape, shapeBytes)
        || !ReadShape(std::vector<std::uint8_t>(shapeBytes.begin(),
                                                shapeBytes.end()),
                      persistedShape)
        || !core3d::retained_part_boolean::ExactShapeBytes(
            persistedShape, persistedShapeBytes)) return result;
    NSMutableArray *components = [NSMutableArray array];
    for (const auto& component : record.components)
        [components addObject:E5Data(component.identifier)];
    result[@"ok"] = @YES;
    result[@"recordBytes"] = E5Data(bytes);
    result[@"shapeBytes"] = E5Data(persistedShapeBytes.data(),
                                   persistedShapeBytes.size());
    result[@"owner"] = E5Data(record.owner.entity);
    result[@"root"] = E5Data(record.rootOutput);
    result[@"feature"] = E5Data(record.feature);
    result[@"components"] = components;
    result[@"revision"] = @(record.revision);
    result[@"kind"] = @(static_cast<int>(record.kind));
    result[@"catalogHandle"] = [NSString stringWithUTF8String:record.catalogHandle.c_str()];
    result[@"text"] = [NSString stringWithUTF8String:record.textUTF8.c_str()];
    result[@"heightMM"] = @(record.heightMM);
    result[@"trackingMM"] = @(record.trackingMM);
    result[@"depthMM"] = @(record.depthMM);
    result[@"offsetUMM"] = @(record.offsetUMM);
    result[@"offsetVMM"] = @(record.offsetVMM);
    result[@"rotation"] = @(record.rotationRadians);
    result[@"alignment"] = @(static_cast<int>(record.alignment));
    result[@"curveCount"] = @(record.curves.size());
    result[@"loopCount"] = @(record.loops.size());
    result[@"history"] = @(_document->Document()->GetAvailableUndos());
    return result;
}
#endif

@end

#if DEBUG
// DEBUG probe used by ShapeyardTests/E5TextReliefNativeTests.swift to
// exercise the same detached admission/raised build path with a pinned
// identity seed, explicit shared-budget limits, an armed post-build Stop
// checkpoint and the foreign/inward/off-face failure legs. It mutates
// nothing and mints no authority; production decisions never read it.
extern "C" void *Core3DDebugE5TextReliefOpeningProbe(
    void *openingRaw, void *resourceRaw, void *hashRaw, void *seedRaw,
    int32_t kind, int32_t scenario, double stageAllowance,
    double visitAllowance, int32_t armCancelAfterBuild) {
    @autoreleasepool {
        Core3DTextReliefOpening *opening =
            (__bridge Core3DTextReliefOpening *)openingRaw;
        NSData *resource = (__bridge NSData *)resourceRaw;
        NSString *hash = (__bridge NSString *)hashRaw;
        NSData *seed = seedRaw ? (__bridge NSData *)seedRaw : nil;
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        result[@"ok"] = @YES;
        if (!opening || ![resource isKindOfClass:NSData.class]
            || ![hash isKindOfClass:NSString.class] || ![NSThread isMainThread]) {
            result[@"ok"] = @NO;
            return (__bridge_retained void *)result;
        }
        E5RequestSpec spec;
        spec.bytes.resize(resource.length);
        if (resource.length)
            std::memcpy(spec.bytes.data(), resource.bytes, resource.length);
        const char *hex = hash.UTF8String;
        if (!hex || std::strlen(hex) != 64) {
            result[@"ok"] = @NO;
            return (__bridge_retained void *)result;
        }
        bool hexOK = true;
        for (unsigned index = 0; index < 32 && hexOK; ++index) {
            unsigned value = 0;
            for (unsigned half = 0; half < 2; ++half) {
                const char c = hex[index * 2 + half];
                value <<= 4;
                if (c >= '0' && c <= '9') value |= unsigned(c - '0');
                else if (c >= 'a' && c <= 'f') value |= unsigned(c - 'a' + 10);
                else hexOK = false;
            }
            spec.expected[index] = std::uint8_t(value);
        }
        if (!hexOK) {
            result[@"ok"] = @NO;
            return (__bridge_retained void *)result;
        }
        spec.kind = tr::ResourceKind(kind);
        spec.text = "TREASURE";
        spec.heightMM = 10;
        spec.depthMM = 1;
        spec.trackingMM = 0.25;
        spec.alignment = 2;
        if (seed && seed.length == 16) {
            std::memcpy(spec.seed.data(), seed.bytes, 16);
            spec.mintSeed = false;
        }
        BOOL admitOnly = NO;
        switch (scenario) {
            case 1: break;                          // text build
            case 2: break;                          // emblem build (kind 2)
            case 3: spec.foreignDigest = true; break;
            case 4: spec.inwardFrame = true; break;
            case 5: spec.forceOffset = true; spec.forcedOffsetUMM = 500; break;
            case 6: admitOnly = YES; break;         // admission only
            case 7: spec.text = "0"; admitOnly = YES; break; // curved counter
            default:
                result[@"ok"] = @NO;
                return (__bridge_retained void *)result;
        }
        if (spec.kind == tr::ResourceKind::EmblemVector) spec.text = "";
        tb::Counter budget;
        if (stageAllowance >= 0 && stageAllowance < double(tb::MaximumBuildStages))
            budget.buildStages = tb::MaximumBuildStages - std::size_t(stageAllowance);
        if (visitAllowance >= 0 && visitAllowance < double(tb::MaximumTopologyVisits))
            budget.topologyVisits = tb::MaximumTopologyVisits - std::size_t(visitAllowance);
        E5Build build;
        @try {
            if (armCancelAfterBuild == 1)
                [opening e5DebugArmCancelAfterBuild];
            build = [opening buildRaisedWithSpec:spec budget:budget admitOnly:admitOnly];
        } @catch (__unused NSException *exception) {
            result[@"ok"] = @NO;
            return (__bridge_retained void *)result;
        }
        result[@"built"] = @(build.built);
        result[@"admissionPhase"] = @(build.admissionPhase);
        result[@"refusalRaw"] = @(build.admissionPhase
            ? static_cast<int>(build.admission) : static_cast<int>(build.refusal));
        result[@"reason"] = [NSString stringWithUTF8String:
            build.admissionPhase ? tr::Reason(build.admission)
                                 : tr::Reason(build.refusal)];
        result[@"stagesConsumed"] = @(double(build.stagesConsumed));
        result[@"visitsConsumed"] = @(double(build.visitsConsumed));
        result[@"budgetExhausted"] = @(build.budgetExhausted);
        result[@"stickyHeld"] = @(build.stickyHeld);
        result[@"cancelledObserved"] = @(build.cancelledObserved);
        result[@"advanceMM"] = @(build.advanceMM);
        result[@"curveCount"] = @(double(build.curveBytes.size()));
        result[@"contourCount"] = @(double(build.contourCount));
        NSMutableArray *curves = [NSMutableArray array];
        for (const auto& bytes : build.curveBytes) [curves addObject:E5Data(bytes)];
        result[@"curveBytes"] = curves;
        if (build.built && !build.admittedOnly) {
            const double mmPerLocal = 1000.0 * opening.metersPerUnit;
            std::string compoundBytes;
            if (core3d::retained_part_boolean::ExactShapeBytes(build.raised.compound,
                                                               compoundBytes))
                result[@"candidateBytes"] = E5Data(compoundBytes.data(),
                                                   compoundBytes.size());
            else
                result[@"candidateBytes"] = NSData.data;
            double area = 0, volume = 0;
            for (const tr::RaisedComponent& component : build.raised.components) {
                area += component.areaLocal2 * mmPerLocal * mmPerLocal;
                volume += component.volumeLocal3 * mmPerLocal * mmPerLocal * mmPerLocal;
            }
            result[@"componentCount"] = @(double(build.raised.components.size()));
            result[@"areaMM2"] = @(area);
            result[@"volumeMM3"] = @(volume);
        } else {
            result[@"candidateBytes"] = NSData.data;
            result[@"componentCount"] = @0;
        }
        return (__bridge_retained void *)result;
    }
}

// DEBUG probe wrapping the SYTR/1 codec exactly: decode, then re-encode and
// compare bytes. Hand-assembled vectors stay on the test side.
extern "C" void *Core3DDebugE5TextReliefCodecProbe(void *dataRaw) {
    @autoreleasepool {
        NSData *data = (__bridge NSData *)dataRaw;
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        std::vector<std::uint8_t> bytes(data ? data.length : 0);
        if (data && data.length) std::memcpy(bytes.data(), data.bytes, data.length);
        tr::ReliefRecipe recipe;
        std::vector<std::uint8_t> exact;
        const bool accepted = tr::Decode(bytes, recipe);
        result[@"accepted"] = @(accepted);
        result[@"exact"] = @(accepted && tr::Encode(recipe, exact) && exact == bytes);
        return (__bridge_retained void *)result;
    }
}

// Narrow DEBUG observation/fault seams. They can only inspect an opening that
// already owns a real OcctDocument capture or arm a one-shot branch in the
// production apply path; neither manufactures an owner, receipt or codec.
extern "C" void *Core3DDebugE5TextReliefObservation(void *openingRaw) {
    @autoreleasepool {
        Core3DTextReliefOpening *opening =
            (__bridge Core3DTextReliefOpening *)openingRaw;
        NSDictionary *value = opening ? [opening e5DebugObservation] : @{@"ok": @NO};
        return (__bridge_retained void *)value;
    }
}

extern "C" void Core3DDebugE5TextReliefArmApplyFault(
    void *openingRaw, int32_t fault) {
    Core3DTextReliefOpening *opening =
        (__bridge Core3DTextReliefOpening *)openingRaw;
    if (opening && fault >= 0 && fault <= 6) [opening e5DebugArmApplyFault:fault];
}

extern "C" int32_t Core3DDebugE5TextReliefArmReplayPairFault(void *openingRaw) {
    Core3DTextReliefOpening *opening = (__bridge Core3DTextReliefOpening *)openingRaw;
    return opening && [opening e5DebugArmReplayPairFault] ? 1 : 0;
}

extern "C" void *Core3DDebugE5TextReliefReplayObservation(void *openingRaw) {
    @autoreleasepool {
        Core3DTextReliefOpening *opening = (__bridge Core3DTextReliefOpening *)openingRaw;
        return (__bridge_retained void *)(opening ? [opening e5DebugReplayObservation]
                                                   : @{@"ok": @NO});
    }
}

extern "C" void *Core3DDebugE5TextReliefReplayRefusal(void *openingRaw, int32_t scenario) {
    @autoreleasepool {
        Core3DTextReliefOpening *opening = (__bridge Core3DTextReliefOpening *)openingRaw;
        return (__bridge_retained void *)(opening ? [opening e5DebugReplayRefusal:scenario]
                                                   : @{@"ok": @NO});
    }
}

// Transitional link compatibility for older test bundles. The former body
// that manufactured an out-of-document owner was intentionally removed.
extern "C" void *Core3DDebugE5TextReliefOwnerProbe(
    int32_t scenario, double, void *goldenRaw) {
    @autoreleasepool {
        NSMutableDictionary *result = [NSMutableDictionary dictionaryWithObject:@NO
                                                                          forKey:@"ok"];
        if (scenario != 2 || !goldenRaw) return (__bridge_retained void *)result;
        NSData *golden = (__bridge NSData *)goldenRaw;
        std::vector<std::uint8_t> bytes(golden.length);
        if (golden.length) std::memcpy(bytes.data(), golden.bytes, golden.length);
        tr::AuthoredRecord value; std::vector<std::uint8_t> exact;
        const bool accepted = tr::DecodeAuthored(bytes, value)
            && tr::EncodeAuthored(value, exact) && exact == bytes;
        auto reserved = bytes; if (reserved.size() > 5) reserved[5] = 1;
        tr::AuthoredRecord refused;
        result[@"goldenAccepted"] = @(accepted);
        result[@"goldenMatchesWriter"] = @(accepted && exact == bytes);
        result[@"reservedRefused"] = @(!tr::DecodeAuthored(reserved, refused));
        auto future = bytes; if (future.size() > 4) future[4] = 3;
        result[@"futureRefused"] = @(!tr::DecodeAuthored(future, refused));
        auto truncated = bytes; if (!truncated.empty()) truncated.pop_back();
        result[@"truncatedRefused"] = @(!tr::DecodeAuthored(truncated, refused));
        result[@"recordBytes"] = golden;
        result[@"ok"] = @(accepted);
        return (__bridge_retained void *)result;
    }
}

// DEBUG probe for the S1-unavailable seams; no controller required.
extern "C" void *Core3DDebugE5TextReliefUnavailableProbe(void) {
    @autoreleasepool {
        Core3DTextReliefOpening *opening = [[Core3DTextReliefOpening alloc] initPrivate];
        NSError *error = nil;
        Core3DTextReliefCandidate *candidate =
            [[Core3DTextReliefCandidate alloc] initPrivate];
        id prepared = [opening openingByPreparingCandidate:candidate error:&error];
        return (__bridge_retained void *)@{
            @"publiclyAvailable": @([Core3DTextReliefOpening isPubliclyAvailable]),
            @"applyRaw": @([opening apply]),
            @"prepared": @(prepared != nil),
            @"prepareError": @(error.code),
        };
    }
}

#endif
