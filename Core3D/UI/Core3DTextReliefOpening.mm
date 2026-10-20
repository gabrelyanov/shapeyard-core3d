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
#include "../OCCTKit/TextReliefBuild.hxx"
#include "Core3DViewer.h"

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
#include <memory>
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

tr::Digest SHA256(const void *bytes, std::size_t count) {
    tr::Digest digest{};
    CC_SHA256(bytes, CC_LONG(count), digest.data());
    if (!core3d::retained_recipe::Nonzero(digest)) digest[0] = 1;
    return digest;
}
tr::Digest SHA256(const std::vector<std::uint8_t>& bytes) {
    return SHA256(bytes.data(), bytes.size());
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
- (instancetype)initPrivate;
@end
@implementation Core3DTextReliefCandidate
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DTextReliefOpening ()
@property(nonatomic, strong, readwrite) Core3DTextReliefIssuedFace *issuedFace;
@property(nonatomic, copy, readwrite) NSString *documentIdentifier;
@property(nonatomic, copy, readwrite) NSString *contextIdentifier;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@property(nonatomic, assign, readwrite) NSInteger historyDepth;
@end

@implementation Core3DTextReliefOpening {
    __weak Core3DViewController *_controller;
    Handle(OcctDocument) _document;
    std::shared_ptr<OpeningContext> _context;
    IssuedSelection _issued;
    std::shared_ptr<std::atomic_bool> _cancelled;
    std::uint32_t _viewportWidth, _viewportHeight;
#if DEBUG
    std::shared_ptr<std::atomic_int> _debugCancelAfterBuild;
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
        IssuedSelection issued; issued.entity = entity;
        issued.topologyIndex = element.topologyIndex;
        std::string exactBytes;
        if (source.IsNull()
            || !core3d::retained_part_boolean::ExactShapeBytes(source, exactBytes)
            || exactBytes.empty()) {
            if (error) *error = E5Error(Core3DTextReliefErrorUnavailable,
                                        @"selected native source unavailable");
            return nil;
        }
        issued.sourceBytes.assign(exactBytes.begin(), exactBytes.end());
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(source, TopAbs_FACE, faces);
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
        issued.host.entity = MintUUID();
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
        TopoDS_Shape host;
        if (!ReadShape(_issued.sourceBytes, host)) {
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

- (Core3DTextReliefApplyResult)apply { return Core3DTextReliefApplyResultUnavailable; }

+ (BOOL)isPubliclyAvailable { return NO; }

#if DEBUG
- (void)e5DebugArmCancelAfterBuild {
    if (_debugCancelAfterBuild) _debugCancelAfterBuild->store(1);
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
