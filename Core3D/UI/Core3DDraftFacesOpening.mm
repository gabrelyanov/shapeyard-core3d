// B4-S1 detached Draft Faces opening. No command lease, retained record,
// history entry, registry admission or publication is reachable from here.

#import "Core3DDraftFacesOpening.h"
#import "Core3DViewController.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/DraftFacesRegistry.hxx"
#include "../OCCTKit/DraftFacesNativeOwner.hxx"
#include "../OCCTKit/NativeOpeningContext.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/ProfilePersistence.hxx"
#include "Core3DViewer.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <gp_Pnt2d.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <CommonCrypto/CommonDigest.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

NSErrorDomain const Core3DDraftFacesErrorDomain = @"Core3DDraftFacesErrorDomain";
NSString *const Core3DDraftFacesRefusalReasonKey = @"b4.refusal";
NSString *const Core3DDraftFacesRefusalRawValueKey = @"b4.refusalRaw";

namespace {
namespace df = core3d::draft_faces;
namespace rf = core3d::retained_feature;
namespace cr = core3d::composite_recipe;
namespace tb = core3d::retained_topology_budget;
using OpeningContext = core3d::native_opening::Context;

NSError *DFError(Core3DDraftFacesError code, NSString *reason) {
    return [NSError errorWithDomain:Core3DDraftFacesErrorDomain code:code
        userInfo:@{NSLocalizedDescriptionKey: reason ?: @"draft faces refused"}];
}
NSError *DFRefusal(Core3DDraftFacesError code, df::Refusal refusal) {
    NSString *reason = [NSString stringWithUTF8String:df::Reason(refusal)];
    return [NSError errorWithDomain:Core3DDraftFacesErrorDomain code:code
        userInfo:@{NSLocalizedDescriptionKey: reason,
                   Core3DDraftFacesRefusalReasonKey: reason,
                   Core3DDraftFacesRefusalRawValueKey: @(static_cast<int>(refusal))}];
}
NSData *DFData(const void *bytes, std::size_t count) {
    return [NSData dataWithBytes:bytes length:count];
}
NSData *DFData(const std::vector<std::uint8_t>& bytes) {
    return DFData(bytes.data(), bytes.size());
}
template <std::size_t N> NSData *DFData(const std::array<std::uint8_t, N>& bytes) {
    return DFData(bytes.data(), bytes.size());
}
NSArray<NSNumber *> *DFVector(const std::array<double, 3>& value) {
    return @[ @(value[0]), @(value[1]), @(value[2]) ];
}

df::UUID MintUUID() {
    df::UUID value{};
    arc4random_buf(value.data(), value.size());
    value[6] = std::uint8_t((value[6] & 0x0f) | 0x40);
    value[8] = std::uint8_t((value[8] & 0x3f) | 0x80);
    return value;
}
df::UUID TaggedUUID(std::uint8_t tag) { df::UUID value{}; value.back() = tag; return value; }
df::Digest TaggedDigest(std::uint8_t tag) { df::Digest value{}; value.back() = tag; return value; }

df::Digest SelectorProof(const std::string& document, const std::string& entity,
                         std::uint32_t topologyIndex, df::SurfaceKind surface,
                         const std::array<double, 3>& normal) {
    df::Digest digest{};
    CC_SHA256_CTX sha; CC_SHA256_Init(&sha);
    CC_SHA256_Update(&sha, document.data(), CC_LONG(document.size()));
    CC_SHA256_Update(&sha, entity.data(), CC_LONG(entity.size()));
    CC_SHA256_Update(&sha, &topologyIndex, CC_LONG(sizeof(topologyIndex)));
    const std::uint8_t kind = std::uint8_t(surface);
    CC_SHA256_Update(&sha, &kind, 1);
    for (double component : normal) {
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

struct IssuedFaceValue final {
    df::FaceIntent intent;
    std::string entity;
    std::uint32_t topologyIndex = 0;
};
struct IssuedSelection final {
    std::vector<IssuedFaceValue> faces;
    std::vector<std::uint8_t> sourceBytes;
    std::string entity;
};

bool IssueFace(const TopoDS_Shape& source, const std::string& entity,
               std::uint32_t topologyIndex, IssuedFaceValue& issued) {
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(source, TopAbs_FACE, faces);
    if (topologyIndex >= std::uint32_t(faces.Extent())) return false;
    const TopoDS_Face face = TopoDS::Face(faces.FindKey(Standard_Integer(topologyIndex) + 1));
    BRepAdaptor_Surface surface(face);
    const df::SurfaceKind kind = df::detail::kind(surface.GetType());
    if (std::uint8_t(kind) == 0) return false;
    std::array<double, 3> normal{{1, 0, 0}};
    if (kind == df::SurfaceKind::Planar)
        normal = df::detail::components(df::detail::outwardPlaneNormal(face, surface));
    issued.intent.identifier = MintUUID();
    issued.intent.surface = kind;
    issued.intent.expectedBoundaryEdges = std::uint16_t(df::detail::boundaryEdges(face));
    issued.intent.expectedCardinality = 1;
    issued.intent.outwardNormalLocal = normal;
    issued.entity = entity;
    issued.topologyIndex = topologyIndex;
    return issued.intent.expectedBoundaryEdges >= 3 && issued.intent.expectedBoundaryEdges <= 64;
}

bool Vector3(NSArray<NSNumber *> *values, std::array<double, 3>& output) {
    if (![values isKindOfClass:NSArray.class] || values.count != 3) return false;
    for (NSUInteger index = 0; index < 3; ++index) {
        id value = values[index];
        if (![value isKindOfClass:NSNumber.class]) return false;
        output[index] = [value doubleValue];
    }
    return true;
}

bool NeutralSample(const TopoDS_Face& face, const gp_Pln& plane, double tolerance,
                   std::array<double, 3>& output) {
    BRepAlgoAPI_Section section(face, plane, Standard_False);
    section.Approximation(Standard_False); section.Build();
    if (!section.IsDone() || section.Shape().IsNull()) return false;
    for (TopExp_Explorer edge(section.Shape(), TopAbs_EDGE); edge.More(); edge.Next()) {
        BRepAdaptor_Curve curve(TopoDS::Edge(edge.Current()));
        const double first = curve.FirstParameter(), last = curve.LastParameter();
        if (std::isfinite(first) && std::isfinite(last) && last > first) {
            const gp_Pnt sample = curve.Value((first + last) * 0.5);
            if (plane.Distance(sample) <= tolerance) {
                output = {{sample.X(), sample.Y(), sample.Z()}}; return true;
            }
        }
    }
    std::vector<gp_Pnt> points;
    for (TopExp_Explorer vertex(face, TopAbs_VERTEX); vertex.More(); vertex.Next()) {
        const gp_Pnt point = BRep_Tool::Pnt(TopoDS::Vertex(vertex.Current()));
        if (plane.Distance(point) <= tolerance) points.push_back(point);
    }
    if (points.empty()) return false;
    gp_XYZ sum(0, 0, 0); for (const gp_Pnt& point : points) sum += point.XYZ();
    sum /= double(points.size()); output = {{sum.X(), sum.Y(), sum.Z()}}; return true;
}

df::Definition FixtureDefinition(double unit, double angle, bool multi = false,
                                 double neutralZMM = 0) {
    const double scale = 0.001 / unit;
    df::Definition value; value.metersPerLocalUnit = unit;
    value.neutralOriginLocal = {{0, 0, neutralZMM * scale}};
    value.neutralNormalLocal = {{0, 0, 1}};
    value.pullDirectionLocal = {{0, 0, 1}};
    value.signedAngleRadians = angle;
    df::FaceIntent left; left.identifier = TaggedUUID(1);
    left.selectorProof = TaggedDigest(0xA1); left.surface = df::SurfaceKind::Planar;
    left.expectedBoundaryEdges = 4; left.expectedCardinality = 1;
    left.sampleOnNeutralLocal = {{0, 10 * scale, neutralZMM * scale}};
    left.outwardNormalLocal = {{-1, 0, 0}}; value.faces.push_back(left);
    if (multi) {
        df::FaceIntent right = left; right.identifier = TaggedUUID(2);
        right.selectorProof = TaggedDigest(0xA2);
        right.sampleOnNeutralLocal = {{30 * scale, 10 * scale, neutralZMM * scale}};
        right.outwardNormalLocal = {{1, 0, 0}}; value.faces.push_back(right);
    }
    return value;
}

NSDictionary *BuildProbe(int32_t scenario, double unit, double angle,
                         double stageAllowance, double visitAllowance,
                         int32_t cancelMode) {
    NSMutableDictionary *result = [NSMutableDictionary dictionary];
    result[@"ok"] = @YES;
    const double scale = 0.001 / unit;
    TopoDS_Shape source;
    df::Definition definition;
    if (scenario == 3) {
        source = BRepPrimAPI_MakeCylinder(10 * scale, 40 * scale).Shape();
        definition = FixtureDefinition(unit, angle);
        definition.faces[0].surface = df::SurfaceKind::Cylindrical;
        definition.faces[0].sampleOnNeutralLocal = {{10 * scale, 0, 0}};
        definition.faces[0].outwardNormalLocal = {{1, 0, 0}};
    } else if (scenario == 4) {
        source = BRepPrimAPI_MakeCone(10 * scale, 5 * scale, 40 * scale).Shape();
        definition = FixtureDefinition(unit, angle);
        definition.faces[0].surface = df::SurfaceKind::Conical;
        definition.faces[0].sampleOnNeutralLocal = {{10 * scale, 0, 0}};
        definition.faces[0].outwardNormalLocal = {{1, 0, 0}};
    } else {
        source = BRepPrimAPI_MakeBox(30 * scale, 20 * scale, 40 * scale).Shape();
        definition = FixtureDefinition(unit, angle, scenario == 2 || scenario == 6,
                                       scenario == 5 ? 50 : 0);
    }
    if (scenario == 3 || scenario == 4) {
        for (TopExp_Explorer explorer(source, TopAbs_FACE); explorer.More(); explorer.Next()) {
            const TopoDS_Face face = TopoDS::Face(explorer.Current());
            BRepAdaptor_Surface surface(face);
            if (df::detail::kind(surface.GetType()) == definition.faces[0].surface) {
                definition.faces[0].expectedBoundaryEdges =
                    std::uint16_t(df::detail::boundaryEdges(face));
                break;
            }
        }
    }
    std::vector<std::uint8_t> sourceBefore, sourceAfter;
    df::detail::exactShapeBytes(source, sourceBefore);
    tb::Counter budget;
    if (stageAllowance >= 0 && stageAllowance < double(tb::MaximumBuildStages))
        budget.buildStages = tb::MaximumBuildStages - std::size_t(stageAllowance);
    if (visitAllowance >= 0 && visitAllowance < double(tb::MaximumTopologyVisits))
        budget.topologyVisits = tb::MaximumTopologyVisits - std::size_t(visitAllowance);
    std::atomic_bool cancelled{cancelMode == 1};
    const std::size_t stagesBefore = budget.buildStages, visitsBefore = budget.topologyVisits;
    df::BuildResult built;
    if (scenario == 5) {
        df::detail::SectionWitness witness;
        const gp_Pln outside(gp_Pnt(0, 0, 50 * scale), gp::DZ());
        if (!df::detail::sectionWitness(source, outside,
                std::max(Precision::Confusion() * 32, 1e-7 / unit), witness))
            built.refusal = df::Refusal::NoNeutralIntersection;
        else
            built = df::BuildDeterministically(source, definition, cancelled, budget);
    } else {
        built = df::BuildDeterministically(source, definition, cancelled, budget);
    }
    const bool staleSuppressed = cancelMode == 3 && built.built();
    if ((cancelMode == 2 || cancelMode == 3) && built.built()) {
        cancelled.store(true); built.solid.Nullify(); built.refusal = df::Refusal::Cancelled;
    }
    df::detail::exactShapeBytes(source, sourceAfter);
    result[@"built"] = @(built.built());
    result[@"reason"] = [NSString stringWithUTF8String:df::Reason(built.refusal)];
    result[@"refusalRaw"] = @(static_cast<int>(built.refusal));
    result[@"sourceUnchanged"] = @(sourceBefore == sourceAfter);
    result[@"stagesConsumed"] = @(double(budget.buildStages - stagesBefore));
    result[@"visitsConsumed"] = @(double(budget.topologyVisits - visitsBefore));
    result[@"budgetExhausted"] = @(budget.exhausted);
    result[@"staleSuppressed"] = @(staleSuppressed);
    result[@"connectedSolids"] = @(built.evidence.connectedSolids);
    result[@"neutralEdges"] = @(built.evidence.neutralSectionEdges);
    result[@"neutralPerimeterMM"] = @(built.evidence.neutralSectionLengthLocal * 1000 * unit);
    result[@"measuredAngle"] = @(built.evidence.measuredSignedAngleRadians);
    NSMutableArray *sections = [NSMutableArray array];
    for (const df::AnalyticSection& section : built.evidence.sections) {
        [sections addObject:@{ @"face": DFData(section.face),
            @"zMM": @(section.pullOffsetLocal * 1000 * unit),
            @"normalOffsetMM": @(section.measuredNormalOffsetLocal * 1000 * unit) }];
    }
    result[@"sections"] = sections;
    std::vector<std::uint8_t> candidate;
    if (built.built()) df::detail::exactShapeBytes(built.solid, candidate);
    result[@"candidateBytes"] = DFData(candidate);
    NSMutableArray *consumed = [NSMutableArray array];
    for (const auto& add : built.evidence.adds) [consumed addObject:DFData(add.face)];
    result[@"consumed"] = consumed;
    return result;
}
} // namespace

@interface Core3DDraftFacesIssuedFace ()
@property(nonatomic, copy, readwrite) NSData *identifier;
@property(nonatomic, copy, readwrite) NSData *selectorProof;
@property(nonatomic, copy, readwrite) NSString *entityIdentifier;
@property(nonatomic, assign, readwrite) uint32_t topologyIndex;
@property(nonatomic, assign, readwrite) Core3DDraftFacesSurfaceKind surfaceKind;
@property(nonatomic, assign, readwrite) uint16_t boundaryEdgeCount;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *outwardNormalLocal;
- (instancetype)initPrivate;
@end
@implementation Core3DDraftFacesIssuedFace
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DDraftFacesSectionObservation ()
@property(nonatomic, copy, readwrite) NSData *faceIdentifier;
@property(nonatomic, assign, readwrite) double pullOffsetMM;
@property(nonatomic, assign, readwrite) double measuredNormalOffsetMM;
- (instancetype)initPrivate;
@end
@implementation Core3DDraftFacesSectionObservation
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DDraftFacesCandidate ()
@property(nonatomic, copy, readwrite) NSData *candidateBRepBytes;
@property(nonatomic, copy, readwrite) NSData *geometryDigest;
@property(nonatomic, copy, readwrite) NSData *familyProof;
@property(nonatomic, copy, readwrite) NSData *canonicalDefinitionBytes;
@property(nonatomic, copy, readwrite) NSArray<NSData *> *consumedFaceIdentifiers;
@property(nonatomic, copy, readwrite) NSArray<Core3DDraftFacesSectionObservation *> *sectionObservations;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *neutralOriginMM;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *neutralNormal;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *pullDirection;
@property(nonatomic, assign, readwrite) double signedAngleDegrees;
@property(nonatomic, assign, readwrite) double measuredSignedAngleRadians;
@property(nonatomic, assign, readwrite) double neutralSectionPerimeterMM;
@property(nonatomic, assign, readwrite) NSUInteger neutralSectionEdgeCount;
@property(nonatomic, assign, readwrite) NSUInteger connectedSolidCount;
- (instancetype)initPrivate;
@end
@implementation Core3DDraftFacesCandidate
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DDraftFacesOpening ()
@property(nonatomic, copy, readwrite) NSArray<Core3DDraftFacesIssuedFace *> *issuedFaces;
@property(nonatomic, copy, readwrite) NSString *documentIdentifier;
@property(nonatomic, copy, readwrite) NSString *contextIdentifier;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@property(nonatomic, assign, readwrite) NSInteger historyDepth;
@end

@implementation Core3DDraftFacesOpening {
    __weak Core3DViewController *_controller;
    Handle(OcctDocument) _document;
    std::shared_ptr<OpeningContext> _context;
    IssuedSelection _issued;
    std::shared_ptr<std::atomic_bool> _cancelled;
    std::uint32_t _viewportWidth, _viewportHeight;
    Core3DDraftFacesCandidate *_issuedCandidate;
}
- (instancetype)initPrivate { return [super init]; }

+ (instancetype)captureController:(Core3DViewController *)controller error:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !controller) {
        if (error) *error = DFError(Core3DDraftFacesErrorInvalidThread,
                                    @"main-thread controller required");
        return nil;
    }
    @try {
        Core3DSceneSnapshot *scene = [controller captureSceneSnapshot];
        if (!scene || scene.selectionMode != Core3DSceneElementKindFace
            || scene.selection.selectedElements.count < 1
            || scene.selection.selectedElements.count > df::MaximumFaces) {
            if (error) *error = DFError(Core3DDraftFacesErrorInvalidSelection,
                                        @"one to sixteen selected native faces required");
            return nil;
        }
        NSString *entityIdentifier = scene.selection.selectedElements.firstObject.entityIdentifier;
        for (Core3DSceneElementIdentifier *element in scene.selection.selectedElements)
            if (element.kind != Core3DSceneElementKindFace
                || ![element.entityIdentifier isEqualToString:entityIdentifier]) {
                if (error) *error = DFError(Core3DDraftFacesErrorInvalidSelection,
                                            @"selected faces must share one native source");
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
            if (error) *error = DFError(Core3DDraftFacesErrorUnavailable,
                                        @"native document unavailable");
            return nil;
        }
        const std::uint32_t width = std::uint32_t(drawable.width), height = std::uint32_t(drawable.height);
        const std::string entity = entityIdentifier.UTF8String ?: "";
        auto context = viewer->captureNativeOpeningContext(width, height, {entity});
        const auto native = document->Document();
        if (!context || native.IsNull() || native->GetData().IsNull() || native->HasOpenCommand()
            || context->openingFence().document() != native
            || context->openingFence().data() != native->GetData()
            || !context->isCurrent(width, height)) {
            if (error) *error = DFError(Core3DDraftFacesErrorStale,
                                        @"native opening fence changed");
            return nil;
        }
        TDF_Label label;
        if (!RootForEntity(document, entityIdentifier, label)
            || !XCAFDoc_ShapeTool::IsFree(label) || !XCAFDoc_ShapeTool::IsSimpleShape(label)) {
            if (error) *error = DFError(Core3DDraftFacesErrorInvalidSelection,
                                        @"selected entity is not a retained native source");
            return nil;
        }
        const TopoDS_Shape source = XCAFDoc_ShapeTool::GetShape(label);
        IssuedSelection issued; issued.entity = entity;
        if (source.IsNull() || !df::detail::exactShapeBytes(source, issued.sourceBytes)) {
            if (error) *error = DFError(Core3DDraftFacesErrorUnavailable,
                                        @"selected native source unavailable");
            return nil;
        }
        NSMutableArray *publicFaces = [NSMutableArray array];
        std::set<std::uint32_t> unique;
        for (Core3DSceneElementIdentifier *element in scene.selection.selectedElements) {
            if (!unique.insert(element.topologyIndex).second) {
                if (error) *error = DFError(Core3DDraftFacesErrorInvalidSelection,
                                            @"duplicate selected face"); return nil;
            }
            IssuedFaceValue value;
            if (!IssueFace(source, entity, element.topologyIndex, value)) {
                if (error) *error = DFError(Core3DDraftFacesErrorInvalidSelection,
                                            @"selected face cannot be issued"); return nil;
            }
            value.intent.selectorProof = SelectorProof(document->DocumentIdentifier(), entity,
                element.topologyIndex, value.intent.surface, value.intent.outwardNormalLocal);
            Core3DDraftFacesIssuedFace *face = [[Core3DDraftFacesIssuedFace alloc] initPrivate];
            face.identifier = DFData(value.intent.identifier);
            face.selectorProof = DFData(value.intent.selectorProof);
            face.entityIdentifier = element.entityIdentifier;
            face.topologyIndex = element.topologyIndex;
            face.surfaceKind = Core3DDraftFacesSurfaceKind(std::uint8_t(value.intent.surface));
            face.boundaryEdgeCount = value.intent.expectedBoundaryEdges;
            face.outwardNormalLocal = DFVector(value.intent.outwardNormalLocal);
            [publicFaces addObject:face]; issued.faces.push_back(value);
        }
        const double unit = context->openingFence().metersPerUnit();
        if (!std::isfinite(unit) || unit <= 0 || unit > 1) {
            if (error) *error = DFError(Core3DDraftFacesErrorUnavailable,
                                        @"native document unit unavailable"); return nil;
        }
        Core3DDraftFacesOpening *opening = [[self alloc] initPrivate];
        opening->_controller = controller; opening->_document = document;
        opening->_context = std::move(context); opening->_issued = std::move(issued);
        opening->_cancelled = std::make_shared<std::atomic_bool>(false);
        opening->_viewportWidth = width; opening->_viewportHeight = height;
        opening.issuedFaces = publicFaces;
        opening.documentIdentifier = [NSString stringWithUTF8String:document->DocumentIdentifier().c_str()];
        opening.contextIdentifier = NSUUID.UUID.UUIDString;
        opening.metersPerUnit = unit; opening.historyDepth = controller.documentUndoCount;
        return opening;
    } @catch (__unused NSException *exception) {
        if (error) *error = DFError(Core3DDraftFacesErrorUnavailable, @"native capture exception");
        return nil;
    }
}

- (BOOL)isCurrent:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !_controller) {
        if (error) *error = DFError(Core3DDraftFacesErrorInvalidThread,
                                    @"main-thread live controller required"); return NO;
    }
    if (_cancelled->load()) {
        if (error) *error = DFError(Core3DDraftFacesErrorCancelled, @"opening cancelled"); return NO;
    }
    if (!_context || !_context->isCurrent(_viewportWidth, _viewportHeight)
        || _controller.documentUndoCount != self.historyDepth) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale, @"native opening fence changed");
        return NO;
    }
    Core3DSceneSnapshot *scene = [_controller captureSceneSnapshot];
    if (!scene || scene.selectionMode != Core3DSceneElementKindFace
        || scene.selection.selectedElements.count != _issued.faces.size()) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale, @"native face selection changed");
        return NO;
    }
    std::set<std::uint32_t> indices;
    for (Core3DSceneElementIdentifier *element in scene.selection.selectedElements) {
        if (element.kind != Core3DSceneElementKindFace
            || _issued.entity != (element.entityIdentifier.UTF8String ?: "")) {
            if (error) *error = DFError(Core3DDraftFacesErrorStale, @"native face selection changed");
            return NO;
        }
        indices.insert(element.topologyIndex);
    }
    for (const IssuedFaceValue& face : _issued.faces)
        if (!indices.count(face.topologyIndex)) {
            if (error) *error = DFError(Core3DDraftFacesErrorStale, @"native face selection changed");
            return NO;
        }
    return YES;
}

- (void)cancel { if (_cancelled) _cancelled->store(true); }

- (Core3DDraftFacesCandidate *)detachedCandidateWithNeutralOriginLocal:(NSArray<NSNumber *> *)origin
                                                     neutralNormalLocal:(NSArray<NSNumber *> *)normal
                                                      pullDirectionLocal:(NSArray<NSNumber *> *)pull
                                                        signedAngleRadians:(double)angle
                                                                     error:(NSError **)error {
    if (error) *error = nil;
    if (![self isCurrent:error]) return nil;
    df::Definition definition; definition.metersPerLocalUnit = self.metersPerUnit;
    if (!Vector3(origin, definition.neutralOriginLocal)
        || !Vector3(normal, definition.neutralNormalLocal)
        || !Vector3(pull, definition.pullDirectionLocal) || !std::isfinite(angle)) {
        if (error) *error = DFError(Core3DDraftFacesErrorInvalidArgument,
                                    @"finite three-component plane and pull required"); return nil;
    }
    definition.signedAngleRadians = angle;
    TopoDS_Shape source;
    if (!df::detail::readShape(_issued.sourceBytes, source)) {
        if (error) *error = DFError(Core3DDraftFacesErrorUnavailable, @"captured source unreadable");
        return nil;
    }
    const gp_Pln plane(df::detail::point(definition.neutralOriginLocal),
                       df::detail::direction(definition.neutralNormalLocal));
    const double tolerance = std::max(Precision::Confusion() * 32,
                                      1e-7 / self.metersPerUnit);
    TopTools_IndexedMapOfShape faces; TopExp::MapShapes(source, TopAbs_FACE, faces);
    for (const IssuedFaceValue& issued : _issued.faces) {
        df::FaceIntent intent = issued.intent;
        const TopoDS_Face face = TopoDS::Face(
            faces.FindKey(Standard_Integer(issued.topologyIndex) + 1));
        if (!NeutralSample(face, plane, tolerance, intent.sampleOnNeutralLocal)) {
            if (error) *error = DFRefusal(Core3DDraftFacesErrorBuildRefused,
                                          df::Refusal::NoNeutralIntersection); return nil;
        }
        definition.faces.push_back(intent);
    }
    df::Refusal validity;
    if (!df::Validate(definition, validity)) {
        if (error) *error = DFRefusal(Core3DDraftFacesErrorBuildRefused, validity); return nil;
    }
    std::vector<std::uint8_t> parameters;
    if (!df::Encode(definition, parameters)) {
        if (error) *error = DFRefusal(Core3DDraftFacesErrorBuildRefused,
                                      df::Refusal::MalformedPayload); return nil;
    }
    cr::FeatureNode feature; feature.node = MintUUID(); feature.feature = MintUUID();
    feature.kind = df::FeatureKind; feature.codecVersion = df::CodecVersion;
    feature.inputs.push_back(MintUUID()); feature.parameters = parameters;
    rf::ReplayValue input; input.shape = rf::ShapeKind::Solid; input.detachedShape = _issued.sourceBytes;
    rf::ReplayValue output; tb::Counter budget; df::Refusal refusal; df::BuildEvidence evidence;
    if (!df::detail::buildDetachedObserved(feature, {input}, budget, *_cancelled,
                                            output, &refusal, &evidence)) {
        if (error) {
            const Core3DDraftFacesError code = refusal == df::Refusal::Cancelled
                ? Core3DDraftFacesErrorCancelled
                : refusal == df::Refusal::TopologyBudgetExceeded
                    ? Core3DDraftFacesErrorBudget : Core3DDraftFacesErrorBuildRefused;
            *error = DFRefusal(code, refusal);
        }
        return nil;
    }
    if (![self isCurrent:error] || _cancelled->load()) return nil;
    Core3DDraftFacesCandidate *candidate = [[Core3DDraftFacesCandidate alloc] initPrivate];
    candidate.candidateBRepBytes = DFData(output.detachedShape);
    candidate.geometryDigest = DFData(output.geometry); candidate.familyProof = DFData(output.familyProof);
    candidate.canonicalDefinitionBytes = DFData(parameters); candidate.metersPerUnit = self.metersPerUnit;
    const double mmScale = 1000 * self.metersPerUnit;
    candidate.neutralOriginMM = @[
        @(definition.neutralOriginLocal[0] * mmScale),
        @(definition.neutralOriginLocal[1] * mmScale),
        @(definition.neutralOriginLocal[2] * mmScale) ];
    candidate.neutralNormal = DFVector(definition.neutralNormalLocal);
    candidate.pullDirection = DFVector(definition.pullDirectionLocal);
    candidate.signedAngleDegrees = definition.signedAngleRadians * 180 / M_PI;
    candidate.measuredSignedAngleRadians = evidence.measuredSignedAngleRadians;
    candidate.neutralSectionPerimeterMM = evidence.neutralSectionLengthLocal * 1000 * self.metersPerUnit;
    candidate.neutralSectionEdgeCount = evidence.neutralSectionEdges;
    candidate.connectedSolidCount = evidence.connectedSolids;
    NSMutableArray *consumed = [NSMutableArray array];
    for (const auto& add : evidence.adds) [consumed addObject:DFData(add.face)];
    candidate.consumedFaceIdentifiers = consumed;
    NSMutableArray *sections = [NSMutableArray array];
    for (const auto& value : evidence.sections) {
        Core3DDraftFacesSectionObservation *section = [[Core3DDraftFacesSectionObservation alloc] initPrivate];
        section.faceIdentifier = DFData(value.face);
        section.pullOffsetMM = value.pullOffsetLocal * 1000 * self.metersPerUnit;
        section.measuredNormalOffsetMM = value.measuredNormalOffsetLocal * 1000 * self.metersPerUnit;
        [sections addObject:section];
    }
    candidate.sectionObservations = sections;
    _issuedCandidate = candidate;
    return candidate;
}

- (Core3DDraftFacesOpening *)openingByPreparingCandidate:(Core3DDraftFacesCandidate *)candidate
                                                   error:(NSError **)error {
    if (error) *error = DFError(Core3DDraftFacesErrorApplyUnavailable,
                                @"candidate prepare is unavailable in S1");
    return nil;
}
- (Core3DDraftFacesApplyResult)apply { return Core3DDraftFacesApplyResultUnavailable; }
+ (BOOL)isPubliclyAvailable { return NO; }

// Internal B4-S2 owner-probe bridge: hands the production native owner the
// exact document, issuing context, entity and viewport this opening captured.
// It changes no state and grants no mutation authority of its own.
- (BOOL)s2OwnerDocument:(Handle(OcctDocument)&)document
                context:(std::shared_ptr<OpeningContext>&)context
                 entity:(std::string&)entity
                  width:(std::uint32_t&)width
                 height:(std::uint32_t&)height {
    if (_document.IsNull() || !_context || _issued.entity.empty()) return NO;
    document = _document; context = _context; entity = _issued.entity;
    width = _viewportWidth; height = _viewportHeight;
    return YES;
}

- (BOOL)s4aValidateIssuedCandidate:(Core3DDraftFacesCandidate *)candidate
                             error:(NSError **)error {
    if (candidate == nil || candidate != _issuedCandidate) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale,
                                    @"candidate was not issued by this opening");
        return NO;
    }
    return [self isCurrent:error];
}
@end

// B4-S3 production editing bridge. This block is deliberately outside DEBUG:
// shipping manual editors use the same retained owner as the S2 lifecycle.
namespace {
namespace dfo = core3d::draft_faces::owner;

struct DFEditingState final {
    cr::Record record;
    df::Definition definition;
    TopoDS_Shape source;
    std::vector<IssuedFaceValue> available;
    df::UUID ownerEntity{}, ownerDefinition{}, baseFeature{}, feature{};
    double unit = 0;
};

std::shared_ptr<dfo::NativeOwner> DFProductionOwnerFor(OcctDocument *document) {
    static std::map<void *, std::shared_ptr<dfo::NativeOwner>> owners;
    const auto found = owners.find(document);
    if (found != owners.end()) return found->second;
    auto created = std::make_shared<dfo::NativeOwner>(*document);
    owners.emplace(document, created);
    return created;
}

NSString *DFSelectorDescription(const std::array<double, 3>& normal) {
    const char *axis = "Z"; double value = normal[2];
    if (std::abs(normal[0]) >= std::abs(normal[1])
        && std::abs(normal[0]) >= std::abs(normal[2])) { axis = "X"; value = normal[0]; }
    else if (std::abs(normal[1]) >= std::abs(normal[2])) { axis = "Y"; value = normal[1]; }
    return [NSString stringWithFormat:@"Planar face %s%@", axis, value < 0 ? @"−" : @"+"];
}

bool DFDataUUID(NSData *data, df::UUID& value) {
    if (![data isKindOfClass:NSData.class] || data.length != value.size()) return false;
    std::memcpy(value.data(), data.bytes, value.size()); return true;
}

std::vector<std::uint8_t> DFBytes(NSData *data) {
    std::vector<std::uint8_t> bytes(data.length);
    if (data.length) std::memcpy(bytes.data(), data.bytes, data.length);
    return bytes;
}

bool DFReadEditingState(const Handle(OcctDocument)& document, NSString *entityIdentifier,
                        std::map<std::string, df::UUID>& choiceIdentifiers,
                        DFEditingState& state, NSError **error) {
    state = {};
    if (error) *error = nil;
    TDF_Label carrier;
    if (document.IsNull() || !RootForEntity(document, entityIdentifier, carrier)) {
        if (error) *error = DFError(Core3DDraftFacesErrorUnavailable,
                                    @"retained Draft Faces carrier unavailable");
        return false;
    }
    const auto native = document->Document();
    if (native.IsNull() || native->HasOpenCommand()
        || !cr::Read(native, carrier, state.record) || !state.record.value
        || state.record.value->definition.schemaVersion != 3
        || state.record.value->definition.nodes.size() != 2) {
        if (error) *error = DFError(Core3DDraftFacesErrorUnavailable,
                                    @"retained Draft Faces record unavailable");
        return false;
    }
    const auto *source = std::get_if<cr::SourceNode>(
        &state.record.value->definition.nodes[0].value);
    const auto *feature = std::get_if<cr::FeatureNode>(
        &state.record.value->definition.nodes[1].value);
    if (!source || !feature || feature->kind != df::FeatureKind
        || feature->codecVersion != df::CodecVersion || feature->inputs.size() != 1
        || feature->inputs[0] != source->node
        || state.record.value->definition.outputNode != feature->node
        || source->shapeSlot >= state.record.value->sourceShapes.size()
        || !df::Decode(feature->parameters, state.definition)) {
        if (error) *error = DFError(Core3DDraftFacesErrorUnavailable,
                                    @"retained Draft Faces grammar unavailable");
        return false;
    }
    state.source = state.record.value->sourceShapes[source->shapeSlot];
    state.ownerEntity = state.record.value->definition.owner.entity;
    state.ownerDefinition = state.record.value->definition.owner.definition;
    state.baseFeature = source->original.sourceFeature; state.feature = feature->feature;
    state.unit = state.definition.metersPerLocalUnit;
    if (state.source.IsNull() || !std::isfinite(state.unit) || state.unit <= 0 || state.unit > 1) {
        if (error) *error = DFError(Core3DDraftFacesErrorUnavailable,
                                    @"retained Draft Faces source unavailable");
        return false;
    }
    const std::string entity = entityIdentifier.UTF8String ?: "";
    state.available.clear();
    const gp_Pln plane(df::detail::point(state.definition.neutralOriginLocal),
                       df::detail::direction(state.definition.neutralNormalLocal));
    const double tolerance = std::max(Precision::Confusion() * 32, 1e-7 / state.unit);
    TopTools_IndexedMapOfShape faces; TopExp::MapShapes(state.source, TopAbs_FACE, faces);
    for (Standard_Integer index = 1; index <= faces.Extent(); ++index) {
        IssuedFaceValue issued;
        if (!IssueFace(state.source, entity, std::uint32_t(index - 1), issued)
            || issued.intent.surface != df::SurfaceKind::Planar) continue;
        issued.intent.selectorProof = SelectorProof(document->DocumentIdentifier(), entity,
            issued.topologyIndex, issued.intent.surface, issued.intent.outwardNormalLocal);
        if (!NeutralSample(TopoDS::Face(faces.FindKey(index)), plane, tolerance,
                           issued.intent.sampleOnNeutralLocal)) continue;
        const auto stored = std::find_if(state.definition.faces.begin(), state.definition.faces.end(),
            [&](const df::FaceIntent& value) { return value.selectorProof == issued.intent.selectorProof; });
        const std::string proof(reinterpret_cast<const char *>(issued.intent.selectorProof.data()),
                                issued.intent.selectorProof.size());
        if (stored != state.definition.faces.end()) issued.intent = *stored;
        else {
            const auto found = choiceIdentifiers.find(proof);
            if (found != choiceIdentifiers.end()) issued.intent.identifier = found->second;
            else choiceIdentifiers.emplace(proof, issued.intent.identifier);
        }
        choiceIdentifiers[proof] = issued.intent.identifier;
        state.available.push_back(std::move(issued));
    }
    if (state.available.empty()) {
        if (error) *error = DFError(Core3DDraftFacesErrorUnavailable,
                                    @"no editable planar faces remain");
        return false;
    }
    return true;
}
} // namespace

@interface Core3DDraftFacesNativeReceipt ()
@property(nonatomic, assign, readwrite) Core3DDraftFacesNativeOutcome outcome;
@property(nonatomic, copy, readwrite) NSString *reason;
@property(nonatomic, assign, readwrite) uint64_t recoverySession;
@property(nonatomic, assign, readwrite) NSInteger measuredHistoryDelta;
@property(nonatomic, assign, readwrite, getter=isSettled) BOOL settled;
- (instancetype)initPrivate;
@end
@implementation Core3DDraftFacesNativeReceipt
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DDraftFacesNativeFaceSnapshot ()
@property(nonatomic, copy, readwrite) NSData *identifier;
@property(nonatomic, copy, readwrite) NSString *selectorDescription;
@property(nonatomic, assign, readwrite) Core3DDraftFacesSurfaceKind surfaceKind;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *sampleOnNeutralMM;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *outwardNormal;
@property(nonatomic, assign, readwrite, getter=isSelected) BOOL selected;
- (instancetype)initPrivate;
@end
@implementation Core3DDraftFacesNativeFaceSnapshot
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DDraftFacesNativeSnapshot ()
@property(nonatomic, copy, readwrite) NSData *ownerEntityIdentifier;
@property(nonatomic, copy, readwrite) NSData *ownerDefinitionIdentifier;
@property(nonatomic, copy, readwrite) NSData *baseFeatureIdentifier;
@property(nonatomic, copy, readwrite) NSData *featureIdentifier;
@property(nonatomic, copy, readwrite) NSArray<Core3DDraftFacesNativeFaceSnapshot *> *faces;
@property(nonatomic, copy, readwrite) NSArray<Core3DDraftFacesNativeFaceSnapshot *> *availableFaces;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *neutralOriginMM;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *neutralNormal;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *pullDirection;
@property(nonatomic, assign, readwrite) double signedAngleDegrees;
@property(nonatomic, copy, readwrite) NSArray<NSData *> *orderedDownstreamTreatmentIdentifiers;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@property(nonatomic, copy, readwrite) NSData *revision;
- (instancetype)initPrivate;
@end
@implementation Core3DDraftFacesNativeSnapshot
- (instancetype)initPrivate { return [super init]; }
@end

namespace {
Core3DDraftFacesNativeReceipt *DFPublicReceipt(const dfo::Receipt& value) {
    Core3DDraftFacesNativeReceipt *receipt = [[Core3DDraftFacesNativeReceipt alloc] initPrivate];
    receipt.outcome = Core3DDraftFacesNativeOutcome(std::uint8_t(value.outcome));
    receipt.reason = [NSString stringWithUTF8String:value.reason.c_str()];
    receipt.recoverySession = value.session;
    receipt.measuredHistoryDelta = value.measuredHistoryDelta;
    receipt.settled = value.settled;
    return receipt;
}

Core3DDraftFacesNativeFaceSnapshot *DFPublicFace(const df::FaceIntent& face,
                                                 bool selected, double unit) {
    Core3DDraftFacesNativeFaceSnapshot *value =
        [[Core3DDraftFacesNativeFaceSnapshot alloc] initPrivate];
    value.identifier = DFData(face.identifier);
    value.selectorDescription = DFSelectorDescription(face.outwardNormalLocal);
    value.surfaceKind = Core3DDraftFacesSurfaceKind(std::uint8_t(face.surface));
    value.sampleOnNeutralMM = @[ @(face.sampleOnNeutralLocal[0] * 1000 * unit),
        @(face.sampleOnNeutralLocal[1] * 1000 * unit),
        @(face.sampleOnNeutralLocal[2] * 1000 * unit) ];
    value.outwardNormal = DFVector(face.outwardNormalLocal); value.selected = selected;
    return value;
}

Core3DDraftFacesNativeSnapshot *DFPublicSnapshot(const DFEditingState& state) {
    Core3DDraftFacesNativeSnapshot *value = [[Core3DDraftFacesNativeSnapshot alloc] initPrivate];
    value.ownerEntityIdentifier = DFData(state.ownerEntity);
    value.ownerDefinitionIdentifier = DFData(state.ownerDefinition);
    value.baseFeatureIdentifier = DFData(state.baseFeature);
    value.featureIdentifier = DFData(state.feature);
    NSMutableArray *selected = [NSMutableArray array];
    for (const auto& face : state.definition.faces)
        [selected addObject:DFPublicFace(face, true, state.unit)];
    NSMutableArray *available = [NSMutableArray array];
    for (const auto& issued : state.available) {
        const bool isSelected = std::any_of(state.definition.faces.begin(), state.definition.faces.end(),
            [&](const df::FaceIntent& face) { return face.identifier == issued.intent.identifier; });
        [available addObject:DFPublicFace(issued.intent, isSelected, state.unit)];
    }
    value.faces = selected; value.availableFaces = available;
    const double scale = 1000 * state.unit;
    value.neutralOriginMM = @[ @(state.definition.neutralOriginLocal[0] * scale),
        @(state.definition.neutralOriginLocal[1] * scale),
        @(state.definition.neutralOriginLocal[2] * scale) ];
    value.neutralNormal = DFVector(state.definition.neutralNormalLocal);
    value.pullDirection = DFVector(state.definition.pullDirectionLocal);
    value.signedAngleDegrees = state.definition.signedAngleRadians * 180 / M_PI;
    value.orderedDownstreamTreatmentIdentifiers = @[];
    value.metersPerUnit = state.unit;
    value.revision = DFData(state.record.value->bytes);
    return value;
}
} // namespace

@implementation Core3DDraftFacesNativeEditingBridge {
    __weak Core3DViewController *_controller;
    Handle(OcctDocument) _document;
    NSString *_entityIdentifier;
    std::shared_ptr<dfo::NativeOwner> _owner;
    std::shared_ptr<const dfo::Capture> _pendingCapture;
    std::vector<std::uint8_t> _pendingParameters;
    df::UUID _pendingFeature;
    NSData *_pendingRevision;
    std::map<std::string, df::UUID> _choiceIdentifiers;
    bool _recoveryPending;
    bool _createMode;
    std::vector<std::uint8_t> _pendingCandidateBytes;
    std::shared_ptr<std::atomic_bool> _createStop;
}

- (instancetype)initPrivate { return [super init]; }

+ (instancetype)captureController:(Core3DViewController *)controller
                 entityIdentifier:(NSString *)entityIdentifier error:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !controller || entityIdentifier.length == 0) {
        if (error) *error = DFError(Core3DDraftFacesErrorInvalidArgument,
                                    @"main-thread controller and entity required");
        return nil;
    }
    GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)controller.glController : nil;
    auto viewer = gl ? gl.viewer : nullptr;
    Handle(OcctDocument) document = viewer ? viewer->getDocument() : Handle(OcctDocument)();
    if (document.IsNull() || document->Document().IsNull()) {
        if (error) *error = DFError(Core3DDraftFacesErrorUnavailable,
                                    @"native document unavailable");
        return nil;
    }
    Core3DDraftFacesNativeEditingBridge *bridge = [[self alloc] initPrivate];
    bridge->_controller = controller; bridge->_document = document;
    bridge->_entityIdentifier = [entityIdentifier copy];
    bridge->_owner = DFProductionOwnerFor(document.get());
    DFEditingState state;
    if (!DFReadEditingState(document, entityIdentifier, bridge->_choiceIdentifiers, state, error))
        return nil;
    return bridge;
}

+ (instancetype)captureCreateController:(Core3DViewController *)controller
                                 opening:(Core3DDraftFacesOpening *)opening
                               candidate:(Core3DDraftFacesCandidate *)candidate
                                   error:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !controller || !opening || !candidate) {
        if (error) *error = DFError(Core3DDraftFacesErrorInvalidArgument,
                                    @"main-thread controller, opening, and candidate required");
        return nil;
    }
    GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)controller.glController : nil;
    auto viewer = gl ? gl.viewer : nullptr;
    Handle(OcctDocument) document = viewer ? viewer->getDocument() : Handle(OcctDocument)();
    Handle(OcctDocument) openingDocument;
    std::shared_ptr<OpeningContext> openingContext;
    std::string entity;
    std::uint32_t width = 0, height = 0;
    if (document.IsNull()
        || ![opening s2OwnerDocument:openingDocument context:openingContext
                              entity:entity width:width height:height]
        || document != openingDocument) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale,
                                    @"opening belongs to a different native document");
        return nil;
    }
    NSString *entityIdentifier = [NSString stringWithUTF8String:entity.c_str()];
    TDF_Label carrier;
    if (!RootForEntity(document, entityIdentifier, carrier)) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale,
                                    @"selected retained carrier changed");
        return nil;
    }
    if (cr::HasRecord(carrier)) {
        if (error) *error = DFError(Core3DDraftFacesErrorInvalidSelection,
                                    @"b4.carrier-already-drafted");
        return nil;
    }
    if (![opening s4aValidateIssuedCandidate:candidate error:error]) return nil;
    core3d::profile::Record profileRecord;
    const auto native = document->Document();
    if (!core3d::profile::Read(native, carrier, profileRecord)) {
        if (error) *error = DFError(Core3DDraftFacesErrorUnavailable,
                                    @"b4.source-profile-malformed");
        return nil;
    }
    if (profileRecord.label.IsNull()) {
        if (error) *error = DFError(Core3DDraftFacesErrorUnavailable,
                                    @"b4.source-profile-unretained");
        return nil;
    }
    if (!profileRecord.IsCurrent(native, carrier)
        || profileRecord.parameters.metersPerUnit != opening.metersPerUnit
        || candidate.metersPerUnit != opening.metersPerUnit) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale,
                                    @"b4.source-profile-stale");
        return nil;
    }
    auto capture = dfo::NativeOwner::MintCreateCapture(
        *document, openingContext, carrier, entity, profileRecord.parameters, width, height);
    if (!capture) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale,
                                    @"b4.create-capture-refused");
        return nil;
    }
    Core3DDraftFacesNativeEditingBridge *bridge = [[self alloc] initPrivate];
    bridge->_controller = controller; bridge->_document = document;
    bridge->_entityIdentifier = [entityIdentifier copy];
    bridge->_owner = DFProductionOwnerFor(document.get());
    bridge->_pendingCapture = std::move(capture);
    bridge->_pendingParameters = DFBytes(candidate.canonicalDefinitionBytes);
    bridge->_pendingCandidateBytes = DFBytes(candidate.candidateBRepBytes);
    bridge->_createStop = std::make_shared<std::atomic_bool>(false);
    bridge->_createMode = true;
    return bridge;
}

- (BOOL)mintCapture:(std::shared_ptr<const dfo::Capture>&)capture error:(NSError **)error {
    Core3DViewController *controller = _controller;
    GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)controller.glController : nil;
    auto viewer = gl ? gl.viewer : nullptr;
    const CGSize drawable = controller.viewportDrawableSize;
    if (!controller || !viewer || !std::isfinite(drawable.width) || !std::isfinite(drawable.height)
        || drawable.width < 1 || drawable.height < 1 || drawable.width > UINT32_MAX
        || drawable.height > UINT32_MAX) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale, @"native editor fence changed");
        return NO;
    }
    const std::uint32_t width = std::uint32_t(drawable.width), height = std::uint32_t(drawable.height);
    const std::string entity = _entityIdentifier.UTF8String ?: "";
    auto context = viewer->captureNativeOpeningContext(width, height, {entity});
    TDF_Label carrier;
    if (!context || !RootForEntity(_document, _entityIdentifier, carrier)) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale, @"native editor fence changed");
        return NO;
    }
    capture = dfo::NativeOwner::MintEditCapture(*_document, context, carrier, entity, width, height);
    if (!capture) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale, @"native edit capture refused");
        return NO;
    }
    return YES;
}

- (Core3DDraftFacesNativeSnapshot *)readSnapshotWithError:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !_controller) {
        if (error) *error = DFError(Core3DDraftFacesErrorInvalidThread,
                                    @"main-thread live controller required");
        return nil;
    }
    DFEditingState state;
    if (!DFReadEditingState(_document, _entityIdentifier, _choiceIdentifiers, state, error)) return nil;
    std::shared_ptr<const dfo::Capture> capture;
    if (![self mintCapture:capture error:error]) return nil;
    return DFPublicSnapshot(state);
}

- (Core3DDraftFacesNativeSnapshot *)prepareFeatureIdentifier:(NSData *)featureIdentifier
                                            expectedRevision:(NSData *)expectedRevision
                                      selectedFaceIdentifiers:(NSArray<NSData *> *)selectedFaceIdentifiers
                                              neutralOriginMM:(NSArray<NSNumber *> *)neutralOriginMM
                                                neutralNormal:(NSArray<NSNumber *> *)neutralNormal
                                                pullDirection:(NSArray<NSNumber *> *)pullDirection
                                           signedAngleDegrees:(double)signedAngleDegrees
                                                        error:(NSError **)error {
    if (error) *error = nil;
    [self cancelPrepared];
    if (!NSThread.isMainThread || _recoveryPending) {
        if (error) *error = DFError(Core3DDraftFacesErrorUnavailable,
                                    @"native owner recovery is pending"); return nil;
    }
    DFEditingState state;
    if (!DFReadEditingState(_document, _entityIdentifier, _choiceIdentifiers, state, error)) return nil;
    if (![expectedRevision isEqualToData:DFData(state.record.value->bytes)]
        || ![featureIdentifier isEqualToData:DFData(state.feature)]) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale,
                                    @"Draft Faces revision changed"); return nil;
    }
    df::Definition edited = state.definition;
    edited.faces.clear();
    if (selectedFaceIdentifiers.count < 1 || selectedFaceIdentifiers.count > df::MaximumFaces) {
        if (error) *error = DFError(Core3DDraftFacesErrorInvalidSelection,
                                    @"one to sixteen faces required"); return nil;
    }
    std::set<df::UUID> unique;
    for (NSData *identifier in selectedFaceIdentifiers) {
        df::UUID nativeID{};
        if (!DFDataUUID(identifier, nativeID) || !unique.insert(nativeID).second) {
            if (error) *error = DFError(Core3DDraftFacesErrorInvalidSelection,
                                        @"duplicate or malformed face identity"); return nil;
        }
        const auto found = std::find_if(state.available.begin(), state.available.end(),
            [&](const IssuedFaceValue& value) { return value.intent.identifier == nativeID; });
        if (found == state.available.end()) {
            if (error) *error = DFError(Core3DDraftFacesErrorInvalidSelection,
                                        @"face identity is stale"); return nil;
        }
        edited.faces.push_back(found->intent);
    }
    std::array<double, 3> originMM{}, normal{}, pull{};
    if (!Vector3(neutralOriginMM, originMM) || !Vector3(neutralNormal, normal)
        || !Vector3(pullDirection, pull) || !std::isfinite(signedAngleDegrees)) {
        if (error) *error = DFError(Core3DDraftFacesErrorInvalidArgument,
                                    @"finite plane, pull, and angle required"); return nil;
    }
    const double localPerMM = 0.001 / state.unit;
    for (unsigned index = 0; index < 3; ++index)
        edited.neutralOriginLocal[index] = originMM[index] * localPerMM;
    edited.neutralNormalLocal = normal; edited.pullDirectionLocal = pull;
    edited.signedAngleRadians = signedAngleDegrees * M_PI / 180;
    const gp_Pln plane(df::detail::point(edited.neutralOriginLocal),
                       df::detail::direction(edited.neutralNormalLocal));
    const double tolerance = std::max(Precision::Confusion() * 32, 1e-7 / state.unit);
    TopTools_IndexedMapOfShape topology; TopExp::MapShapes(state.source, TopAbs_FACE, topology);
    for (auto& face : edited.faces) {
        const auto found = std::find_if(state.available.begin(), state.available.end(),
            [&](const IssuedFaceValue& value) { return value.intent.identifier == face.identifier; });
        if (found == state.available.end()
            || !NeutralSample(TopoDS::Face(topology.FindKey(Standard_Integer(found->topologyIndex) + 1)),
                              plane, tolerance, face.sampleOnNeutralLocal)) {
            if (error) *error = DFRefusal(Core3DDraftFacesErrorBuildRefused,
                                          df::Refusal::NoNeutralIntersection); return nil;
        }
    }
    df::Refusal refusal;
    std::vector<std::uint8_t> parameters;
    if (!df::Validate(edited, refusal) || !df::Encode(edited, parameters)) {
        if (error) *error = DFRefusal(Core3DDraftFacesErrorBuildRefused, refusal); return nil;
    }
    auto graph = state.record.value->definition;
    auto& feature = std::get<cr::FeatureNode>(graph.nodes[1].value);
    feature.parameters = parameters;
    rf::ReplayValue input; input.shape = rf::ShapeKind::Solid;
    if (!df::detail::exactShapeBytes(state.source, input.detachedShape)) {
        if (error) *error = DFError(Core3DDraftFacesErrorUnavailable, @"native source unreadable");
        return nil;
    }
    rf::ReplayValue output; tb::Counter budget; const std::atomic_bool stop{false};
    if (!df::detail::buildDetachedObserved(feature, {input}, budget, stop, output, &refusal, nullptr)) {
        if (error) *error = DFRefusal(Core3DDraftFacesErrorBuildRefused, refusal); return nil;
    }
    std::shared_ptr<const dfo::Capture> capture;
    if (![self mintCapture:capture error:error]) return nil;
    _pendingCapture = std::move(capture); _pendingParameters = parameters;
    _pendingFeature = state.feature; _pendingRevision = [expectedRevision copy];
    state.definition = edited;
    return DFPublicSnapshot(state);
}

- (Core3DDraftFacesNativeReceipt *)commitFeatureIdentifier:(NSData *)featureIdentifier
                                          expectedRevision:(NSData *)expectedRevision
                                                     error:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !_pendingCapture || _pendingParameters.empty()
        || !_pendingRevision || ![expectedRevision isEqualToData:_pendingRevision]
        || ![featureIdentifier isEqualToData:DFData(_pendingFeature)]) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale,
                                    @"no exact reviewed Draft Faces revision"); return nil;
    }
    const std::atomic_bool stop{false};
    const dfo::Receipt native = _owner->editParameters(_pendingCapture, _pendingParameters, stop);
    Core3DDraftFacesNativeReceipt *receipt = DFPublicReceipt(native);
    _recoveryPending = native.outcome == dfo::Outcome::RecoveryRequired;
    if (!_recoveryPending) [self cancelPrepared];
    return receipt;
}

- (Core3DDraftFacesNativeReceipt *)commitCreateWithError:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !_createMode || !_pendingCapture
        || _pendingParameters.empty() || _pendingCandidateBytes.empty() || !_createStop) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale,
                                    @"no exact reviewed Draft Faces create candidate");
        return nil;
    }
    const dfo::Receipt native = _owner->create(
        _pendingCapture, _pendingParameters, _pendingCandidateBytes, *_createStop);
    Core3DDraftFacesNativeReceipt *receipt = DFPublicReceipt(native);
    _recoveryPending = native.outcome == dfo::Outcome::RecoveryRequired;
    if (!_recoveryPending) { _createMode = false; [self cancelPrepared]; }
    return receipt;
}

- (Core3DDraftFacesNativeReceipt *)cancelCreateWithError:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !_createMode || !_pendingCapture || !_createStop) {
        if (error) *error = DFError(Core3DDraftFacesErrorStale,
                                    @"no pending Draft Faces create candidate");
        return nil;
    }
    _createStop->store(true);
    const dfo::Receipt native = _owner->create(
        _pendingCapture, _pendingParameters, _pendingCandidateBytes, *_createStop);
    Core3DDraftFacesNativeReceipt *receipt = DFPublicReceipt(native);
    _recoveryPending = native.outcome == dfo::Outcome::RecoveryRequired;
    if (!_recoveryPending) { _createMode = false; [self cancelPrepared]; }
    return receipt;
}

- (void)cancelPrepared {
    if (_recoveryPending) return;
    _pendingCapture.reset(); _pendingParameters.clear(); _pendingFeature = {};
    _pendingCandidateBytes.clear(); _createStop.reset();
    _pendingRevision = nil;
}

- (Core3DDraftFacesNativeReceipt *)reconcileRecoverySession:(uint64_t)session {
    const dfo::Receipt native = _owner->reconcile(session);
    if (native.settled) {
        _recoveryPending = false; _createMode = false; [self cancelPrepared];
    }
    return DFPublicReceipt(native);
}

- (BOOL)blocksOtherWork { return _owner && _owner->blocksOtherWork(); }
- (double)metersPerUnit {
    DFEditingState state; NSError *error = nil;
    return DFReadEditingState(_document, _entityIdentifier, _choiceIdentifiers, state, &error)
        ? state.unit : 0;
}
@end

#if DEBUG
extern "C" void *Core3DDebugB4DraftFacesBuildProbe(
    int32_t scenario, double unit, double angle, double stageAllowance,
    double visitAllowance, int32_t cancelMode) {
    @autoreleasepool {
        @try { return (__bridge_retained void *)BuildProbe(
            scenario, unit, angle, stageAllowance, visitAllowance, cancelMode); }
        @catch (__unused NSException *exception) {
            return (__bridge_retained void *)@{ @"ok": @NO };
        }
    }
}

extern "C" void *Core3DDebugB4DraftFacesCodecProbe(void *dataRaw) {
    @autoreleasepool {
        NSData *data = (__bridge NSData *)dataRaw;
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        std::vector<std::uint8_t> bytes(data.length);
        if (data.length) std::memcpy(bytes.data(), data.bytes, data.length);
        df::Definition decoded; std::vector<std::uint8_t> exact;
        const bool accepted = df::Decode(bytes, decoded) && df::Encode(decoded, exact);
        result[@"accepted"] = @(accepted);
        result[@"exact"] = @(accepted && exact == bytes);
        result[@"faceCount"] = @(accepted ? decoded.faces.size() : 0);
        result[@"codecVersion"] = @(df::CodecVersion);
        result[@"selectorVersion"] = @(df::SelectorVersion);
        result[@"proofVersion"] = @(df::ProofVersion);
        result[@"maximumPayloadBytes"] = @(df::MaximumPayloadBytes);
        result[@"registryInstalled"] = @(df::DraftRegistryEntry().execution.installed());
        return (__bridge_retained void *)result;
    }
}

extern "C" void *Core3DDebugB4DraftFacesUnavailableProbe(void) {
    @autoreleasepool {
        Core3DDraftFacesOpening *opening = [[Core3DDraftFacesOpening alloc] initPrivate];
        NSError *error = nil;
        Core3DDraftFacesCandidate *candidate = [[Core3DDraftFacesCandidate alloc] initPrivate];
        id prepared = [opening openingByPreparingCandidate:candidate error:&error];
        return (__bridge_retained void *)@{
            @"publiclyAvailable": @([Core3DDraftFacesOpening isPubliclyAvailable]),
            @"applyRaw": @([opening apply]), @"prepared": @(prepared != nil),
            @"prepareError": @(error.code),
            @"registryInstalled": @(df::DraftRegistryEntry().execution.installed()) };
    }
}

// B4-S2 retained native owner probes. These drive the production
// draft_faces::owner::NativeOwner in DraftFacesNativeOwner.hxx through real
// documents, the real S1 opening/candidate and the production composite
// codec; they never install a debug registry, fabricate a receipt or preset a
// result. Every evidence value below is read back from the actual document.
namespace {
namespace dfo = core3d::draft_faces::owner;

bool S2ProbeViewer(Core3DViewController *controller, Handle(OcctDocument)& document,
                   std::shared_ptr<core3d::Core3DViewer>& viewer) {
    document = Handle(OcctDocument)(); viewer.reset();
    if (!NSThread.isMainThread || !controller) return false;
    GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)controller.glController : nil;
    if (!gl) return false;
    viewer = gl.viewer;
    if (!viewer) { viewer.reset(); return false; }
    document = viewer->getDocument();
    return !document.IsNull() && !document->Document().IsNull()
        && !document->Document()->GetData().IsNull();
}

std::shared_ptr<dfo::NativeOwner> S2OwnerFor(OcctDocument *document) {
    return DFProductionOwnerFor(document);
}

core3d::profile::Parameters S2BoxProfile(double unit, double widthMM,
                                         double heightMM, double depthMM) {
    const double scale = 0.001 / unit;
    core3d::profile::Parameters parameters;
    parameters.metersPerUnit = unit;
    parameters.definition.plane = 0;
    parameters.definition.depth = depthMM * scale;
    parameters.definition.revolve = false;
    parameters.definition.points = {
        gp_Pnt2d(0, 0), gp_Pnt2d(widthMM * scale, 0),
        gp_Pnt2d(widthMM * scale, heightMM * scale), gp_Pnt2d(0, heightMM * scale)};
    return parameters;
}

std::vector<std::uint8_t> S2Bytes(NSData *data) {
    std::vector<std::uint8_t> bytes(data.length);
    if (data.length) std::memcpy(bytes.data(), data.bytes, data.length);
    return bytes;
}

void S2ReceiptEvidence(NSMutableDictionary *result, const dfo::Receipt& receipt) {
    result[@"outcome"] = @(int(receipt.outcome));
    result[@"reason"] = [NSString stringWithUTF8String:receipt.reason.c_str()];
    result[@"session"] = @(double(receipt.session));
    result[@"delta"] = @(receipt.measuredHistoryDelta);
    result[@"settled"] = @(receipt.settled);
}

bool S2Viewport(Core3DViewController *controller, std::uint32_t& width, std::uint32_t& height) {
    const CGSize drawable = controller.viewportDrawableSize;
    if (!std::isfinite(drawable.width) || !std::isfinite(drawable.height)
        || drawable.width < 1 || drawable.height < 1
        || drawable.width > UINT32_MAX || drawable.height > UINT32_MAX) return false;
    width = std::uint32_t(drawable.width); height = std::uint32_t(drawable.height);
    return true;
}
} // namespace

extern "C" void *Core3DDebugB4DraftFacesSourceFixtureProbe(
    void *controllerRaw, double unit, double widthMM, double heightMM, double depthMM) {
    @autoreleasepool {
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        result[@"ok"] = @NO;
        @try {
            Core3DViewController *controller = (__bridge Core3DViewController *)controllerRaw;
            Handle(OcctDocument) document; std::shared_ptr<core3d::Core3DViewer> viewer;
            if (!S2ProbeViewer(controller, document, viewer)
                || (unit != 0.001 && unit != 1.0)
                || !(widthMM > 0) || !(heightMM > 0) || !(depthMM > 0))
                return (__bridge_retained void *)result;
            const auto native = document->Document();
            const auto shapes = XCAFDoc_DocumentTool::ShapeTool(native->Main());
            TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
            if (native->HasOpenCommand() || roots.Length() != 0
                || native->GetAvailableUndos() != 0 || native->GetAvailableRedos() != 0)
                return (__bridge_retained void *)result;
            const auto parameters = S2BoxProfile(unit, widthMM, heightMM, depthMM);
            const TopoDS_Shape solid =
                dfo::detail::BuildProductionProfileSource(parameters);
            if (solid.IsNull()) return (__bridge_retained void *)result;
            XCAFDoc_DocumentTool::SetLengthUnit(native, unit);
            const TDF_Label label = shapes->AddShape(solid, Standard_False, Standard_False);
            if (label.IsNull() || !document->MigrateLegacyIdentifiers())
                return (__bridge_retained void *)result;
            native->ClearUndos();
            native->SetUndoLimit(OcctDocument::kNativeSessionUndoLimit);
            document->NotifyChanges();
            // Direct OCAF fixture insertion does not itself guarantee an AIS
            // presentation in this hosted viewer. Reuse the notification-
            // synchronized presentation when present; otherwise install one
            // exact presentation for the real label so the ordinary face-
            // selection path (and its label/identity checks) remains in use.
            const Handle(AIS_InteractiveContext) context = viewer->AisContext();
            if (context.IsNull()) return (__bridge_retained void *)result;
            AIS_ListOfInteractive displayed;
            context->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
            Handle(AIS_Shape) presentation;
            for (AIS_ListIteratorOfListOfInteractive iterator(displayed);
                 iterator.More(); iterator.Next()) {
                const Handle(AIS_Shape) candidate =
                    Handle(AIS_Shape)::DownCast(iterator.Value());
                if (candidate.IsNull()
                    || !document->ShapeLabel(candidate).IsEqual(label)) continue;
                if (!presentation.IsNull()) return (__bridge_retained void *)result;
                presentation = candidate;
            }
            if (presentation.IsNull()) {
                presentation = new AIS_Shape(solid);
                context->Display(presentation, AIS_Shaded, 0, Standard_False);
                document->LoadObjectMeterial(label, presentation);
            }
            context->UpdateCurrentViewer();
            result[@"entity"] = [NSString stringWithUTF8String:
                document->EntityIdentifierForLabel(label).c_str()];
            result[@"unit"] = @(unit);
            result[@"ok"] = @YES;
        } @catch (__unused NSException *exception) { result[@"ok"] = @NO; }
        return (__bridge_retained void *)result;
    }
}

extern "C" void *Core3DDebugB4DraftFacesRetainedSourceFixtureProbe(
    void *controllerRaw, double unit, double widthMM, double heightMM, double depthMM) {
    @autoreleasepool {
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        result[@"ok"] = @NO;
        @try {
            Core3DViewController *controller = (__bridge Core3DViewController *)controllerRaw;
            Handle(OcctDocument) document; std::shared_ptr<core3d::Core3DViewer> viewer;
            if (!S2ProbeViewer(controller, document, viewer)
                || (unit != 0.001 && unit != 1.0)
                || !(widthMM > 0) || !(heightMM > 0) || !(depthMM > 0))
                return (__bridge_retained void *)result;
            const auto native = document->Document();
            const auto shapes = XCAFDoc_DocumentTool::ShapeTool(native->Main());
            TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
            if (native->HasOpenCommand() || roots.Length() != 0
                || native->GetAvailableUndos() != 0 || native->GetAvailableRedos() != 0)
                return (__bridge_retained void *)result;
            const auto parameters = S2BoxProfile(unit, widthMM, heightMM, depthMM);
            const TopoDS_Shape solid =
                dfo::detail::BuildProductionProfileSource(parameters);
            if (solid.IsNull()) return (__bridge_retained void *)result;
            XCAFDoc_DocumentTool::SetLengthUnit(native, unit);
            const TDF_Label label = shapes->AddShape(solid, Standard_False, Standard_False);
            if (label.IsNull() || !document->MigrateLegacyIdentifiers())
                return (__bridge_retained void *)result;
            native->NewCommand();
            const std::string profileIdentifier = OcctDocument::NewProfileIdentifier();
            if (profileIdentifier.empty()
                || !core3d::profile::Stage(native, label, parameters, profileIdentifier)
                || !native->CommitCommand()) {
                if (native->HasOpenCommand()) native->AbortCommand();
                return (__bridge_retained void *)result;
            }
            core3d::profile::Record retained;
            if (!core3d::profile::Read(native, label, retained)
                || !retained.IsCurrent(native, label))
                return (__bridge_retained void *)result;
            native->ClearUndos();
            native->SetUndoLimit(OcctDocument::kNativeSessionUndoLimit);
            document->NotifyChanges();
            const Handle(AIS_InteractiveContext) context = viewer->AisContext();
            if (context.IsNull()) return (__bridge_retained void *)result;
            AIS_ListOfInteractive displayed;
            context->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
            Handle(AIS_Shape) presentation;
            for (AIS_ListIteratorOfListOfInteractive iterator(displayed);
                 iterator.More(); iterator.Next()) {
                const Handle(AIS_Shape) displayedShape =
                    Handle(AIS_Shape)::DownCast(iterator.Value());
                if (displayedShape.IsNull()
                    || !document->ShapeLabel(displayedShape).IsEqual(label)) continue;
                if (!presentation.IsNull()) return (__bridge_retained void *)result;
                presentation = displayedShape;
            }
            if (presentation.IsNull()) {
                presentation = new AIS_Shape(solid);
                context->Display(presentation, AIS_Shaded, 0, Standard_False);
                document->LoadObjectMeterial(label, presentation);
            }
            context->UpdateCurrentViewer();
            result[@"entity"] = [NSString stringWithUTF8String:
                document->EntityIdentifierForLabel(label).c_str()];
            result[@"profileIdentifier"] = [NSString stringWithUTF8String:
                retained.identifier.c_str()];
            result[@"unit"] = @(unit);
            result[@"ok"] = @YES;
        } @catch (__unused NSException *exception) { result[@"ok"] = @NO; }
        return (__bridge_retained void *)result;
    }
}

extern "C" void *Core3DDebugB4DraftFacesRecordProbe(void *controllerRaw, void *entityRaw) {
    @autoreleasepool {
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        result[@"ok"] = @NO; result[@"hasRecord"] = @NO;
        @try {
            Core3DViewController *controller = (__bridge Core3DViewController *)controllerRaw;
            NSString *entity = (__bridge NSString *)entityRaw;
            Handle(OcctDocument) document; std::shared_ptr<core3d::Core3DViewer> viewer;
            TDF_Label label;
            if (!S2ProbeViewer(controller, document, viewer)
                || !RootForEntity(document, entity, label))
                return (__bridge_retained void *)result;
            const auto native = document->Document();
            result[@"ok"] = @YES;
            result[@"entity"] = entity;
            result[@"undoDepth"] = @(native->GetAvailableUndos());
            result[@"registryInstalled"] = @(df::DraftRegistryEntry().execution.installed());
            result[@"debugRegistryActive"] =
                @(core3d::retained_feature::DebugRegistryOverride() != nullptr);
            const TopoDS_Shape current = XCAFDoc_ShapeTool::GetShape(label);
            std::vector<std::uint8_t> carrierBytes;
            if (!current.IsNull() && df::detail::exactShapeBytes(current, carrierBytes)) {
                result[@"carrierBytes"] = DFData(carrierBytes);
                Bnd_Box bounds; BRepBndLib::Add(current, bounds);
                Standard_Real unit = 0;
                if (!bounds.IsVoid()
                    && XCAFDoc_DocumentTool::GetLengthUnit(native, unit) && unit > 0)
                    result[@"zmaxMM"] = @(bounds.CornerMax().Z() * 1000 * unit);
            }
            cr::Record record;
            if (!cr::Read(native, label, record)) { result[@"ok"] = @NO; return (__bridge_retained void *)result; }
            if (!record.value) return (__bridge_retained void *)result;
            result[@"hasRecord"] = @YES;
            result[@"graphBytes"] = DFData(record.value->bytes);
            const auto& definition = record.value->definition;
            result[@"nodeCount"] = @(definition.nodes.size());
            result[@"ownerEntity"] = DFData(definition.owner.entity);
            result[@"ownerDefinition"] = DFData(definition.owner.definition);
            if (const auto* lastFeature =
                    std::get_if<cr::FeatureNode>(&definition.nodes.back().value)) {
                result[@"kind"] = @(lastFeature->kind);
                result[@"codecVersion"] = @(lastFeature->codecVersion);
                result[@"params"] = DFData(lastFeature->parameters);
                result[@"nodeUUID"] = DFData(lastFeature->node);
                result[@"featureUUID"] = DFData(lastFeature->feature);
                result[@"nodeLocalID"] = @(lastFeature->localID);
            }
            // B4 draft grammar detail: exactly one source plus one SYDF
            // feature. Old files of other families report only the generic
            // fields above, so their preserved meaning is observable too.
            if (definition.nodes.size() == 2
                && std::get_if<cr::SourceNode>(&definition.nodes[0].value)
                && std::get_if<cr::FeatureNode>(&definition.nodes[1].value)
                && std::get<cr::FeatureNode>(definition.nodes[1].value).kind
                    == df::FeatureKind) {
                const auto& source = std::get<cr::SourceNode>(definition.nodes[0].value);
                const auto& feature = std::get<cr::FeatureNode>(definition.nodes[1].value);
                result[@"draftGrammar"] = @YES;
                result[@"sourceUUID"] = DFData(source.node);
                result[@"sourceLocalID"] = @(source.localID);
                result[@"sourceRecipeBytes"] = DFData(source.recipe.bytes);
                result[@"sourceGeometryCommitment"] = DFData(source.commitments.geometry);
                result[@"sourceFeatureUUID"] = DFData(source.original.sourceFeature);
                df::Definition parameters;
                if (df::Decode(feature.parameters, parameters)) {
                    NSMutableArray *faceIDs = [NSMutableArray array];
                    for (const auto& face : parameters.faces)
                        [faceIDs addObject:DFData(face.identifier)];
                    result[@"faceIDs"] = faceIDs;
                    result[@"faceCount"] = @(parameters.faces.size());
                }
            }
        } @catch (__unused NSException *exception) { result[@"ok"] = @NO; }
        return (__bridge_retained void *)result;
    }
}

extern "C" void *Core3DDebugB4DraftFacesOwnerCreateProbe(
    void *controllerRaw, void *openingRaw, void *candidateRaw,
    double widthMM, double heightMM, double depthMM, int32_t fault) {
    @autoreleasepool {
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        result[@"ok"] = @NO; result[@"minted"] = @NO;
        @try {
            Core3DViewController *controller = (__bridge Core3DViewController *)controllerRaw;
            Core3DDraftFacesOpening *opening = (__bridge Core3DDraftFacesOpening *)openingRaw;
            Core3DDraftFacesCandidate *candidate = (__bridge Core3DDraftFacesCandidate *)candidateRaw;
            Handle(OcctDocument) document; std::shared_ptr<core3d::Core3DViewer> viewer;
            Handle(OcctDocument) openingDocument;
            std::shared_ptr<OpeningContext> openingContext;
            std::string openingEntity; std::uint32_t openingWidth = 0, openingHeight = 0;
            TDF_Label label;
            if (!S2ProbeViewer(controller, document, viewer) || !opening || !candidate
                || ![opening s2OwnerDocument:openingDocument context:openingContext
                                     entity:openingEntity
                                      width:openingWidth height:openingHeight]
                || document != openingDocument
                || !RootForEntity(document, opening.issuedFaces.firstObject.entityIdentifier, label))
                return (__bridge_retained void *)result;
            const auto parameters = S2BoxProfile(opening.metersPerUnit,
                                                 widthMM, heightMM, depthMM);
            auto capture = dfo::NativeOwner::MintCreateCapture(
                *document, openingContext, label, openingEntity,
                parameters, openingWidth, openingHeight);
            if (!capture) {
                const auto native = document->Document();
                const TopoDS_Shape source = XCAFDoc_ShapeTool::GetShape(label);
                std::vector<std::uint8_t> sourceBytes;
                core3d::composite_recipe::SourceRecipe sourceRecipe;
                core3d::retained_recipe::OwnerKey ownerKey{};
                result[@"capture.mainThread"] = @(pthread_main_np() != 0);
                result[@"capture.document"] = @(!native.IsNull());
                result[@"capture.commandClosed"] =
                    @(!native.IsNull() && !native->HasOpenCommand());
                result[@"capture.fenceDocument"] =
                    @(openingContext->openingFence().document() == native);
                result[@"capture.current"] =
                    @(openingContext->isCurrent(openingWidth, openingHeight));
                result[@"capture.free"] = @(XCAFDoc_ShapeTool::IsFree(label));
                result[@"capture.simple"] = @(XCAFDoc_ShapeTool::IsSimpleShape(label));
                result[@"capture.entity"] =
                    @(document->EntityIdentifierForLabel(label) == openingEntity);
                result[@"capture.noRecord"] = @(!cr::HasRecord(label));
                result[@"capture.unit"] =
                    @(openingContext->openingFence().metersPerUnit() == parameters.metersPerUnit);
                result[@"capture.source"] =
                    @(!source.IsNull() && source.ShapeType() == TopAbs_SOLID
                      && source.Orientation() == TopAbs_FORWARD
                      && df::detail::exactShapeBytes(source, sourceBytes));
                result[@"capture.receipt"] = @(dfo::detail::ProveSourceReceipt(
                    parameters, sourceBytes, sourceRecipe));
                result[@"capture.ownerKey"] =
                    @(dfo::detail::ReadOwnerKey(*document, label, ownerKey));
                // A failed source receipt never becomes owner authority. Drive
                // that absent capability through the real owner so the
                // negative remains a typed, atomic product refusal.
                const Standard_Integer undoBefore = native->GetAvailableUndos();
                const Standard_Integer redoBefore = native->GetAvailableRedos();
                const std::atomic_bool neverStopped{false};
                const auto receipt = S2OwnerFor(document.get())->create(
                    capture, S2Bytes(candidate.canonicalDefinitionBytes),
                    S2Bytes(candidate.candidateBRepBytes), neverStopped,
                    dfo::FaultPoint(fault));
                const TopoDS_Shape afterSource = XCAFDoc_ShapeTool::GetShape(label);
                std::vector<std::uint8_t> afterBytes;
                result[@"capture.atomic"] = @(
                    !afterSource.IsNull()
                    && df::detail::exactShapeBytes(afterSource, afterBytes)
                    && afterBytes == sourceBytes && !cr::HasRecord(label)
                    && native->GetAvailableUndos() == undoBefore
                    && native->GetAvailableRedos() == redoBefore
                    && !native->HasOpenCommand());
                result[@"ok"] = @YES;
                S2ReceiptEvidence(result, receipt);
                return (__bridge_retained void *)result;
            }
            result[@"minted"] = @YES;
            std::atomic_bool stop{fault == int32_t(dfo::FaultPoint::StopBeforeLease)};
            const auto receipt = S2OwnerFor(document.get())->create(
                capture, S2Bytes(candidate.canonicalDefinitionBytes),
                S2Bytes(candidate.candidateBRepBytes), stop, dfo::FaultPoint(fault));
            result[@"ok"] = @YES;
            S2ReceiptEvidence(result, receipt);
        } @catch (__unused NSException *exception) { result[@"ok"] = @NO; }
        return (__bridge_retained void *)result;
    }
}

extern "C" void *Core3DDebugB4DraftFacesOwnerEditProbe(
    void *controllerRaw, void *entityRaw, void *paramsRaw, int32_t fault) {
    @autoreleasepool {
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        result[@"ok"] = @NO; result[@"minted"] = @NO;
        @try {
            Core3DViewController *controller = (__bridge Core3DViewController *)controllerRaw;
            NSString *entity = (__bridge NSString *)entityRaw;
            NSData *params = (__bridge NSData *)paramsRaw;
            Handle(OcctDocument) document; std::shared_ptr<core3d::Core3DViewer> viewer;
            TDF_Label label; std::uint32_t width = 0, height = 0;
            if (!S2ProbeViewer(controller, document, viewer)
                || !RootForEntity(document, entity, label)
                || !S2Viewport(controller, width, height))
                return (__bridge_retained void *)result;
            const std::string entityText = entity.UTF8String ?: "";
            auto context = viewer->captureNativeOpeningContext(width, height, {entityText});
            if (!context) return (__bridge_retained void *)result;
            auto capture = dfo::NativeOwner::MintEditCapture(
                *document, context, label, entityText, width, height);
            if (!capture) return (__bridge_retained void *)result;
            result[@"minted"] = @YES;
            std::atomic_bool stop{fault == int32_t(dfo::FaultPoint::StopBeforeLease)};
            const auto receipt = S2OwnerFor(document.get())->editParameters(
                capture, S2Bytes(params), stop, dfo::FaultPoint(fault));
            result[@"ok"] = @YES;
            S2ReceiptEvidence(result, receipt);
        } @catch (__unused NSException *exception) { result[@"ok"] = @NO; }
        return (__bridge_retained void *)result;
    }
}

extern "C" void *Core3DDebugB4DraftFacesOwnerSourceEditProbe(
    void *controllerRaw, void *entityRaw,
    double widthMM, double heightMM, double depthMM, int32_t fault) {
    @autoreleasepool {
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        result[@"ok"] = @NO; result[@"minted"] = @NO;
        @try {
            Core3DViewController *controller = (__bridge Core3DViewController *)controllerRaw;
            NSString *entity = (__bridge NSString *)entityRaw;
            Handle(OcctDocument) document; std::shared_ptr<core3d::Core3DViewer> viewer;
            TDF_Label label; std::uint32_t width = 0, height = 0;
            if (!S2ProbeViewer(controller, document, viewer)
                || !RootForEntity(document, entity, label)
                || !S2Viewport(controller, width, height))
                return (__bridge_retained void *)result;
            const std::string entityText = entity.UTF8String ?: "";
            auto context = viewer->captureNativeOpeningContext(width, height, {entityText});
            if (!context) return (__bridge_retained void *)result;
            const double unit = context->openingFence().metersPerUnit();
            std::vector<double> values;
            if (!core3d::profile::Encode(
                    S2BoxProfile(unit, widthMM, heightMM, depthMM), values))
                return (__bridge_retained void *)result;
            auto capture = dfo::NativeOwner::MintEditCapture(
                *document, context, label, entityText, width, height);
            if (!capture) return (__bridge_retained void *)result;
            result[@"minted"] = @YES;
            std::atomic_bool stop{fault == int32_t(dfo::FaultPoint::StopBeforeLease)};
            const auto receipt = S2OwnerFor(document.get())->editSource(
                capture, values, stop, dfo::FaultPoint(fault));
            result[@"ok"] = @YES;
            S2ReceiptEvidence(result, receipt);
        } @catch (__unused NSException *exception) { result[@"ok"] = @NO; }
        return (__bridge_retained void *)result;
    }
}

extern "C" void *Core3DDebugB4DraftFacesOwnerReconcileProbe(
    void *controllerRaw, double session) {
    @autoreleasepool {
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        result[@"ok"] = @NO;
        @try {
            Core3DViewController *controller = (__bridge Core3DViewController *)controllerRaw;
            Handle(OcctDocument) document; std::shared_ptr<core3d::Core3DViewer> viewer;
            if (!S2ProbeViewer(controller, document, viewer) || session < 1)
                return (__bridge_retained void *)result;
            const auto receipt = S2OwnerFor(document.get())->reconcile(std::uint64_t(session));
            result[@"ok"] = @YES;
            S2ReceiptEvidence(result, receipt);
        } @catch (__unused NSException *exception) { result[@"ok"] = @NO; }
        return (__bridge_retained void *)result;
    }
}

extern "C" void *Core3DDebugB4DraftFacesCorruptProbe(
    void *fileRaw, void *graphRaw, int32_t mode) {
    @autoreleasepool {
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        result[@"ok"] = @NO;
        @try {
            NSData *file = (__bridge NSData *)fileRaw;
            NSData *graph = (__bridge NSData *)graphRaw;
            std::vector<std::uint8_t> fileBytes = S2Bytes(file);
            const std::vector<std::uint8_t> graphBytes = S2Bytes(graph);
            if (graphBytes.size() < 100 || fileBytes.size() < graphBytes.size())
                return (__bridge_retained void *)result;
            std::size_t offset = 0, occurrences = 0;
            for (std::size_t cursor = 0; cursor + graphBytes.size() <= fileBytes.size(); ++cursor)
                if (std::memcmp(fileBytes.data() + cursor, graphBytes.data(),
                                graphBytes.size()) == 0) { offset = cursor; ++occurrences; }
            if (occurrences != 1) return (__bridge_retained void *)result;
            std::vector<std::uint8_t> corrupted = graphBytes;
            bool fixHash = true;
            const auto sydf = [&]() -> std::size_t {
                for (std::size_t cursor = 0; cursor + 4 <= corrupted.size(); ++cursor)
                    if (std::memcmp(corrupted.data() + cursor, "SYDF", 4) == 0) return cursor;
                return std::string::npos;
            };
            const auto sylv = [&]() -> std::size_t {
                for (std::size_t cursor = 0; cursor + 6 <= corrupted.size(); ++cursor)
                    if (std::memcmp(corrupted.data() + cursor, "SYLV", 4) == 0) return cursor;
                return std::string::npos;
            };
            const std::size_t payload = sydf();
            switch (mode) {
                case 1: if (payload == std::string::npos) return (__bridge_retained void *)result;
                    corrupted[payload + 4] = 2; break;          // future SYDF schema
                case 2: if (payload == std::string::npos) return (__bridge_retained void *)result;
                    corrupted[payload + 5] = 9; break;          // unknown selector key
                case 3: if (payload == std::string::npos) return (__bridge_retained void *)result;
                    {   // nonfinite angle
                        const std::uint8_t nanBytes[8] = {0, 0, 0, 0, 0, 0, 0xF8, 0x7F};
                        std::memcpy(corrupted.data() + payload + 88, nanBytes, 8);
                    } break;
                case 4: corrupted[24] ^= 0xFF; break;           // foreign owner entity
                case 5: corrupted[4] = 99; break;               // future envelope version
                case 6: if (payload == std::string::npos) return (__bridge_retained void *)result;
                    corrupted[payload + 6] = 9; break;          // unknown proof key
                case 7: if (payload == std::string::npos) return (__bridge_retained void *)result;
                    corrupted[payload + 20] ^= 0xFF; fixHash = false; break; // digest mismatch
                case 8: { const std::size_t recipe = sylv();
                    if (recipe == std::string::npos) return (__bridge_retained void *)result;
                    corrupted[recipe + 5] = 0x7E; } break;      // bad source input kind
                default: return (__bridge_retained void *)result;
            }
            if (fixHash) {
                df::Digest digest{};
                std::vector<std::uint8_t> body(corrupted.begin(), corrupted.end() - 32);
                if (!cr::Hash(body, digest)) return (__bridge_retained void *)result;
                std::copy(digest.begin(), digest.end(), corrupted.end() - 32);
            }
            std::memcpy(fileBytes.data() + offset, corrupted.data(), corrupted.size());
            result[@"corrupted"] = DFData(fileBytes);
            result[@"offset"] = @(offset);
            result[@"ok"] = @YES;
        } @catch (__unused NSException *exception) { result[@"ok"] = @NO; }
        return (__bridge_retained void *)result;
    }
}

extern "C" void *Core3DDebugB4DraftFacesCompositeDecodeProbe(void *dataRaw) {
    @autoreleasepool {
        NSData *data = (__bridge NSData *)dataRaw;
        std::vector<std::uint8_t> bytes = S2Bytes(data);
        cr::Definition definition;
        const bool accepted = cr::Decode(bytes, definition);
        return (__bridge_retained void *)@{
            @"accepted": @(accepted),
            @"schemaVersion": @(accepted ? definition.schemaVersion : 0),
            @"nodeCount": @(accepted ? definition.nodes.size() : 0) };
    }
}
#endif
