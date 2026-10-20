// B4-S1 detached Draft Faces opening. No command lease, retained record,
// history entry, registry admission or publication is reachable from here.

#import "Core3DDraftFacesOpening.h"
#import "Core3DViewController.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/DraftFacesRegistry.hxx"
#include "../OCCTKit/NativeOpeningContext.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "Core3DViewer.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRep_Tool.hxx>
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
    candidate.sectionObservations = sections; return candidate;
}

- (Core3DDraftFacesOpening *)openingByPreparingCandidate:(Core3DDraftFacesCandidate *)candidate
                                                   error:(NSError **)error {
    if (error) *error = DFError(Core3DDraftFacesErrorApplyUnavailable,
                                @"candidate prepare is unavailable in S1");
    return nil;
}
- (Core3DDraftFacesApplyResult)apply { return Core3DDraftFacesApplyResultUnavailable; }
+ (BOOL)isPubliclyAvailable { return NO; }
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
#endif
