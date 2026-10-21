//
//  Core3DVariableFilletOpening.mm
//  Core3D
//
//  B3 retained variable-radius fillet opening. A controller/viewer-issued
//  opening bound to one document, one native source entity and one issued
//  oriented-edge selection. It copies the actual source solid without
//  mutating it, runs the existing native deterministic builders through
//  VariableRadiusFilletRegistry.hxx, and returns either a detached candidate
//  (consumed edge set, realised law pins, detached BRep bytes + digest and
//  independent section observations) or an exact typed refusal. S2 binds an
//  accepted detached candidate to the persisted owner and the complete
//  D2/D3/D4 closure before opening one lease, then stages, reads, closes and
//  publishes exactly once.
//

#import "Core3DVariableFilletOpening.h"

#import "Core3DViewController.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/NativeOpeningContext.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/VariableRadiusFilletNativeOwner.hxx"
#include "../OCCTKit/VariableRadiusFilletRegistry.hxx"
#include "Core3DViewer.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_Tool.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Pln.hxx>

#include <CommonCrypto/CommonDigest.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

NSErrorDomain const Core3DVariableFilletErrorDomain = @"Core3DVariableFilletErrorDomain";
NSString *const Core3DVariableFilletRefusalReasonKey = @"b3.refusal";
NSString *const Core3DVariableFilletRefusalRawValueKey = @"b3.refusalRaw";

namespace {

namespace vrf = core3d::variable_radius_fillet;
namespace vro = core3d::variable_radius_fillet::owner;
namespace rf = core3d::retained_feature;
namespace cr = core3d::composite_recipe;
using OpeningContext = core3d::native_opening::Context;

constexpr double kMinimumRadiusMM = 0.001;
constexpr double kMaximumRadiusMM = 1000000.0;

NSError *VRError(Core3DVariableFilletError code, NSString *reason) {
    return [NSError errorWithDomain:Core3DVariableFilletErrorDomain code:code
                           userInfo:@{NSLocalizedDescriptionKey: reason ?: @"variable fillet refused"}];
}

NSError *VRRefusal(Core3DVariableFilletError code, vrf::Refusal refusal) {
    NSString *reason = [NSString stringWithUTF8String:vrf::Reason(refusal)];
    return [NSError errorWithDomain:Core3DVariableFilletErrorDomain code:code
                           userInfo:@{NSLocalizedDescriptionKey: reason,
                                      Core3DVariableFilletRefusalReasonKey: reason,
                                      Core3DVariableFilletRefusalRawValueKey:
                                          @(static_cast<int>(refusal))}];
}

NSData *VRData(const void *bytes, std::size_t count) {
    return [NSData dataWithBytes:bytes length:count];
}
NSData *VRData(const std::vector<std::uint8_t>& bytes) {
    return VRData(bytes.data(), bytes.size());
}
std::vector<std::uint8_t> VRBytes(NSData *data) {
    if (![data isKindOfClass:NSData.class] || data.length == 0) return {};
    const auto *first = static_cast<const std::uint8_t *>(data.bytes);
    return {first, first + data.length};
}
template <std::size_t N>
NSData *VRData(const std::array<std::uint8_t, N>& bytes) {
    return VRData(bytes.data(), bytes.size());
}
NSArray<NSNumber *> *VRVector(const std::array<double, 3>& value) {
    return @[ @(value[0]), @(value[1]), @(value[2]) ];
}

vrf::UUID MintUUID() {
    vrf::UUID identifier{};
    arc4random_buf(identifier.data(), identifier.size());
    identifier[6] = std::uint8_t((identifier[6] & 0x0f) | 0x40);
    identifier[8] = std::uint8_t((identifier[8] & 0x3f) | 0x80);
    return identifier;
}

vrf::Digest SelectorProof(const std::string& document, const std::string& entity,
                          std::uint32_t topologyIndex,
                          const vrf::OrientedEdgeAnchor& anchor) {
    vrf::Digest digest{};
    CC_SHA256_CTX sha; CC_SHA256_Init(&sha);
    CC_SHA256_Update(&sha, document.data(), CC_LONG(document.size()));
    CC_SHA256_Update(&sha, entity.data(), CC_LONG(entity.size()));
    CC_SHA256_Update(&sha, &topologyIndex, CC_LONG(sizeof(topologyIndex)));
    auto feed = [&](const std::array<double, 3>& value) {
        for (double component : value) {
            std::uint64_t bits = 0; std::memcpy(&bits, &component, 8);
            CC_SHA256_Update(&sha, &bits, CC_LONG(8));
        }
    };
    feed(anchor.pointLocal); feed(anchor.tangent);
    feed(anchor.startLocal); feed(anchor.endLocal);
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
    for (Standard_Integer i = 1; i <= roots.Length(); ++i)
        if (document->EntityIdentifierForLabel(roots.Value(i)) == (text ? text : "")) {
            result = roots.Value(i); ++matches;
        }
    return matches == 1 && !result.IsNull();
}

struct IssuedAnchor final {
    vrf::OrientedEdgeAnchor anchor;
    std::string entity;
    std::uint32_t topologyIndex = 0;
    std::vector<std::uint8_t> sourceBytes;
    std::string refusal;       // empty on success
    Core3DVariableFilletError code = Core3DVariableFilletErrorUnavailable;
};

// Native issuance: resolve the selected edge on the actual source shape and
// derive point/tangent/outward normals/start/end from it. Closed, periodic,
// seamed or degenerate edges and nonplanar supports refuse here.
IssuedAnchor IssueAnchor(const TopoDS_Shape& shape, const std::string& entity,
                         std::uint32_t topologyIndex) {
    IssuedAnchor issued;
    try {
        TopTools_IndexedMapOfShape edges;
        TopExp::MapShapes(shape, TopAbs_EDGE, edges);
        if (topologyIndex >= static_cast<std::uint32_t>(edges.Extent())) {
            issued.refusal = "b3.anchor-missing";
            issued.code = Core3DVariableFilletErrorInvalidSelection;
            return issued;
        }
        const TopoDS_Edge edge = TopoDS::Edge(edges.FindKey(Standard_Integer(topologyIndex) + 1));
        if (edge.IsNull() || BRep_Tool::Degenerated(edge) || BRep_Tool::IsClosed(edge)) {
            issued.refusal = "b3.closed-loop-seam-rule-required";
            issued.code = Core3DVariableFilletErrorClosedLoopUnsupported;
            return issued;
        }
        BRepAdaptor_Curve curve(edge);
        if (curve.GetType() != GeomAbs_Line) {
            issued.refusal = "b3.closed-loop-seam-rule-required";
            issued.code = Core3DVariableFilletErrorClosedLoopUnsupported;
            return issued;
        }
        const double first = curve.FirstParameter(), last = curve.LastParameter();
        if (!std::isfinite(first) || !std::isfinite(last) || !(last > first)) {
            issued.refusal = "b3.closed-loop-seam-rule-required";
            issued.code = Core3DVariableFilletErrorClosedLoopUnsupported;
            return issued;
        }
        const gp_Pnt startPoint = curve.Value(first), endPoint = curve.Value(last);
        gp_Pnt midPoint; gp_Vec tangentVector;
        curve.D1((first + last) * 0.5, midPoint, tangentVector);
        if (tangentVector.SquareMagnitude() <= 1e-24) {
            issued.refusal = "b3.closed-loop-seam-rule-required";
            issued.code = Core3DVariableFilletErrorClosedLoopUnsupported;
            return issued;
        }
        tangentVector.Normalize();
        TopTools_IndexedDataMapOfShapeListOfShape adjacency;
        TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, adjacency);
        const auto& owners = adjacency.FindFromKey(edge);
        std::array<double, 3> normals[2]; std::size_t faceCount = 0;
        for (TopTools_ListIteratorOfListOfShape it(owners); it.More(); it.Next()) {
            const TopoDS_Face face = TopoDS::Face(it.Value());
            BRepAdaptor_Surface surface(face);
            if (surface.GetType() != GeomAbs_Plane) {
                issued.refusal = "b3.unsupported-edge";
                issued.code = Core3DVariableFilletErrorClosedLoopUnsupported;
                return issued;
            }
            gp_Dir normal = surface.Plane().Axis().Direction();
            if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
            bool duplicate = false;
            for (std::size_t index = 0; index < faceCount; ++index) {
                const double dot = normals[index][0] * normal.X()
                    + normals[index][1] * normal.Y() + normals[index][2] * normal.Z();
                if (dot > 1.0 - 1e-8) duplicate = true;
            }
            if (duplicate) continue;
            if (faceCount >= 2) {
                issued.refusal = "b3.unsupported-edge";
                issued.code = Core3DVariableFilletErrorClosedLoopUnsupported;
                return issued;
            }
            normals[faceCount++] = { normal.X(), normal.Y(), normal.Z() };
        }
        if (faceCount != 2) {
            issued.refusal = "b3.unsupported-edge";
            issued.code = Core3DVariableFilletErrorClosedLoopUnsupported;
            return issued;
        }
        if (normals[1] < normals[0]) std::swap(normals[0], normals[1]);
        auto point = [](const gp_Pnt& value) {
            return std::array<double, 3>{ value.X(), value.Y(), value.Z() };
        };
        vrf::OrientedEdgeAnchor& anchor = issued.anchor;
        anchor.identifier = MintUUID();
        anchor.curve = vrf::CurveKind::Line;
        anchor.orientation = vrf::Orientation::Forward;
        anchor.pointLocal = point(midPoint);
        anchor.tangent = { tangentVector.X(), tangentVector.Y(), tangentVector.Z() };
        anchor.normalA = normals[0];
        anchor.normalB = normals[1];
        anchor.startLocal = point(startPoint);
        anchor.endLocal = point(endPoint);
        issued.entity = entity;
        issued.topologyIndex = topologyIndex;
        if (!vrf::detail::exactShapeBytes(shape, issued.sourceBytes)) {
            issued.refusal = "b3.kernel-failure";
            issued.code = Core3DVariableFilletErrorUnavailable;
            return issued;
        }
        return issued;
    } catch (...) {
        issued = {};
        issued.refusal = "b3.kernel-failure";
        issued.code = Core3DVariableFilletErrorUnavailable;
        return issued;
    }
}

bool RestoreRetainedAnchor(const vro::Capture& capture,
                           const std::string& entity,
                           IssuedAnchor& issued) noexcept {
    issued = {};
    try {
        vrf::Definition linear;
        vrf::MultiStationDefinition stations;
        if (vrf::Decode(capture.lawBytes, linear) && linear.edges.size() == 1)
            issued.anchor = linear.edges.front();
        else if (vrf::Decode(capture.lawBytes, stations)
                 && stations.edges.size() == 1)
            issued.anchor = stations.edges.front();
        else return false;
        issued.entity = entity;
        issued.topologyIndex = capture.topologyIndex;
        issued.sourceBytes = capture.baseBytes;
        return !issued.sourceBytes.empty();
    } catch (...) { issued = {}; return false; }
}

// ----- Independent section observation of the emitted detached candidate ---

struct SectionObservationValue final {
    double parameter = 0;
    std::array<double, 3> planeOriginMM{};
    std::array<double, 3> planeNormal{};
    std::array<std::array<double, 2>, 5> rayPointsMM{};
    std::array<double, 5> rayRadiiMM{};
    double contactARadiusMM = 0, contactBRadiusMM = 0;
    double midpointRadiusMM = 0, symmetryDeviationMM = 0, planeResidualMM = 0;
};

bool PlaneForSupport(const TopoDS_Face& face, const std::array<double, 3>& expectedOutward,
                     gp_Pln& plane) {
    BRepAdaptor_Surface surface(face);
    if (surface.GetType() != GeomAbs_Plane) return false;
    gp_Pln found = surface.Plane();
    gp_Dir normal = found.Axis().Direction();
    if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
    const double dot = normal.X() * expectedOutward[0] + normal.Y() * expectedOutward[1]
        + normal.Z() * expectedOutward[2];
    if (std::abs(dot - 1.0) > 1e-9) return false;
    plane = found;
    return true;
}

// Separate observation path: the fillet branch is the unique nonplanar face
// adjacent through shared edges to both identified support faces. Selection
// uses topology and support identity only, never radius proximity.
bool ObserveSections(const std::vector<std::uint8_t>& candidateBytes,
                     const vrf::OrientedEdgeAnchor& anchor, double mmPerLocal,
                     std::vector<SectionObservationValue>& output,
                     std::string& failure) {
    output.clear();
    try {
        TopoDS_Shape candidate;
        if (!vrf::detail::readShape(candidateBytes, candidate)) {
            failure = "candidate bytes unreadable"; return false;
        }
        if (!BRepCheck_Analyzer(candidate).IsValid()) {
            failure = "candidate is not a valid solid"; return false;
        }
        const gp_Pnt startPoint(anchor.startLocal[0], anchor.startLocal[1], anchor.startLocal[2]);
        const gp_Pnt endPoint(anchor.endLocal[0], anchor.endLocal[1], anchor.endLocal[2]);
        // Structural sanity bound only: 0.02 physical mm in local units.
        const double planeTolerance = 0.02 / mmPerLocal;
        // Support planes: outward normal match plus incidence with the
        // original edge line, both independent of the built fillet face.
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(candidate, TopAbs_FACE, faces);
        TopoDS_Face supportA, supportB;
        gp_Pln planeA, planeB;
        unsigned countA = 0, countB = 0;
        for (Standard_Integer index = 1; index <= faces.Extent(); ++index) {
            const TopoDS_Face face = TopoDS::Face(faces.FindKey(index));
            gp_Pln plane;
            if (PlaneForSupport(face, anchor.normalA, plane)
                && plane.Distance(startPoint) <= planeTolerance
                && plane.Distance(endPoint) <= planeTolerance) {
                ++countA; supportA = face; planeA = plane;
            }
            if (PlaneForSupport(face, anchor.normalB, plane)
                && plane.Distance(startPoint) <= planeTolerance
                && plane.Distance(endPoint) <= planeTolerance) {
                ++countB; supportB = face; planeB = plane;
            }
        }
        if (countA != 1 || countB != 1) {
            failure = "support faces not uniquely identified"; return false;
        }
        TopTools_IndexedDataMapOfShapeListOfShape adjacency;
        TopExp::MapShapesAndAncestors(candidate, TopAbs_EDGE, TopAbs_FACE, adjacency);
        TopTools_IndexedMapOfShape supportEdgesA, supportEdgesB;
        TopExp::MapShapes(supportA, TopAbs_EDGE, supportEdgesA);
        TopExp::MapShapes(supportB, TopAbs_EDGE, supportEdgesB);
        TopoDS_Face fillet; unsigned filletCount = 0;
        for (Standard_Integer index = 1; index <= faces.Extent(); ++index) {
            const TopoDS_Face face = TopoDS::Face(faces.FindKey(index));
            BRepAdaptor_Surface surface(face);
            if (surface.GetType() == GeomAbs_Plane) continue;
            TopTools_IndexedMapOfShape own;
            TopExp::MapShapes(face, TopAbs_EDGE, own);
            bool touchesA = false, touchesB = false;
            for (Standard_Integer e = 1; e <= own.Extent(); ++e) {
                if (supportEdgesA.Contains(own.FindKey(e))) touchesA = true;
                if (supportEdgesB.Contains(own.FindKey(e))) touchesB = true;
            }
            if (touchesA && touchesB) { ++filletCount; fillet = face; }
        }
        if (filletCount != 1) {
            failure = "fillet branch not uniquely identified"; return false;
        }
        const gp_Vec axis(anchor.tangent[0], anchor.tangent[1], anchor.tangent[2]);
        const double rayAngles[5] = { 0.0, M_PI / 8.0, M_PI / 4.0, 3.0 * M_PI / 8.0, M_PI / 2.0 };
        const double sampleTs[3] = { 0.2, 0.5, 0.8 };
        for (double t : sampleTs) {
            const gp_Pnt origin = startPoint.Translated(
                gp_Vec(startPoint, endPoint).Multiplied(t));
            const gp_Pln plane(origin, gp_Dir(axis));
            BRepAlgoAPI_Section section(fillet, plane, Standard_False);
            section.ComputePCurveOn1(Standard_True);
            section.Approximation(Standard_True);
            section.Build();
            if (!section.IsDone()) { failure = "section computation failed"; return false; }
            TopTools_IndexedMapOfShape sectionEdges;
            TopExp::MapShapes(section.Shape(), TopAbs_EDGE, sectionEdges);
            if (sectionEdges.Extent() != 1) {
                failure = "section chain missing or disconnected"; return false;
            }
            BRepAdaptor_Curve cut(TopoDS::Edge(sectionEdges.FindKey(1)));
            const double u0 = cut.FirstParameter(), u1 = cut.LastParameter();
            if (!std::isfinite(u0) || !std::isfinite(u1) || !(u1 > u0)) {
                failure = "section curve has no finite domain"; return false;
            }
            struct Sample { double a, b, z; };
            std::vector<Sample> samples;
            constexpr int sampleCount = 361;
            samples.reserve(sampleCount);
            for (int i = 0; i < sampleCount; ++i) {
                const gp_Pnt point = cut.Value(u0 + (u1 - u0) * double(i) / (sampleCount - 1));
                const double a = planeA.Distance(point), b = planeB.Distance(point);
                const double z = std::abs(gp_Vec(plane.Axis().Direction())
                    .Dot(gp_Vec(origin, point)));
                samples.push_back({ a, b, z });
            }
            // Order the chain from the support-B contact (b == 0) toward the
            // support-A contact (a == 0); contacts are endpoints, never
            // ill-conditioned ray double roots.
            if (samples.front().b > samples.back().b) std::reverse(samples.begin(), samples.end());
            if (samples.front().b > planeTolerance || samples.back().a > planeTolerance) {
                failure = "section chain does not terminate on the supports"; return false;
            }
            SectionObservationValue observed;
            observed.parameter = t;
            observed.planeOriginMM = { origin.X() * mmPerLocal, origin.Y() * mmPerLocal,
                origin.Z() * mmPerLocal };
            observed.planeNormal = { anchor.tangent[0], anchor.tangent[1], anchor.tangent[2] };
            observed.contactBRadiusMM = samples.front().a * mmPerLocal;
            observed.contactARadiusMM = samples.back().b * mmPerLocal;
            double residual = 0;
            for (const Sample& sample : samples) residual = std::max(residual, sample.z);
            observed.planeResidualMM = residual * mmPerLocal;
            for (unsigned ray = 0; ray < 5; ++ray) {
                double a = 0, b = 0;
                if (ray == 0) {
                    a = samples.front().a; b = samples.front().b;
                } else if (ray == 4) {
                    a = samples.back().a; b = samples.back().b;
                } else {
                    const double phi = rayAngles[ray];
                    bool found = false;
                    for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
                        const double p0 = std::atan2(samples[i].b, samples[i].a);
                        const double p1 = std::atan2(samples[i + 1].b, samples[i + 1].a);
                        if (p0 <= phi && phi <= p1 && p1 > p0) {
                            const double f = (phi - p0) / (p1 - p0);
                            a = samples[i].a + (samples[i + 1].a - samples[i].a) * f;
                            b = samples[i].b + (samples[i + 1].b - samples[i].b) * f;
                            found = true; break;
                        }
                    }
                    if (!found) { failure = "ray intersection missing"; return false; }
                }
                observed.rayPointsMM[ray] = { a * mmPerLocal, b * mmPerLocal };
                observed.rayRadiiMM[ray] = std::hypot(a, b) * mmPerLocal;
            }
            observed.midpointRadiusMM = observed.rayRadiiMM[2];
            observed.symmetryDeviationMM = std::abs(observed.rayPointsMM[2][0]
                - observed.rayPointsMM[2][1]);
            output.push_back(observed);
        }
        return true;
    } catch (...) {
        output.clear(); failure = "section observation exception"; return false;
    }
}

// ----- Shared detached build core (public path and DEBUG probe) ------------

struct OpeningBuild final {
    bool built = false;
    vrf::Refusal refusal = vrf::Refusal::KernelFailure;
    vrf::OrientedEdgeAnchor anchor;
    vrf::BuildEvidence evidence;
    rf::ReplayValue output;
    std::vector<std::uint8_t> parameters;
    std::size_t consumedWork = 0, consumedIdentities = 0, consumedTopology = 0;
    bool budgetExhausted = false;
    bool cancelledObserved = false;
    std::vector<SectionObservationValue> sections;
#if DEBUG
    std::vector<vrf::DebugRealizedLawObservation> realizedLaws;
    std::vector<vrf::DebugMultiStationLawObservation> multiStationLaws;
#endif
};

} // namespace

@interface Core3DVariableFilletStation ()
@property(nonatomic, copy, readwrite) NSData *identifier;
@property(nonatomic, assign, readwrite) double parameter;
@property(nonatomic, assign, readwrite) double radiusMM;
@end
@implementation Core3DVariableFilletStation
- (instancetype)initWithIdentifier:(NSData *)identifier parameter:(double)parameter
                          radiusMM:(double)radiusMM {
    const unsigned char zero[16] = {};
    if (identifier.length != 16 || !std::memcmp(identifier.bytes, zero, 16)
        || !std::isfinite(parameter) || parameter < 0 || parameter > 1
        || !std::isfinite(radiusMM)) return nil;
    if (!(self = [super init])) return nil;
    _identifier = [identifier copy]; _parameter = parameter; _radiusMM = radiusMM;
    return self;
}
@end

@interface Core3DVariableFilletIssuedEdge ()
@property(nonatomic, copy, readwrite) NSData *identifier;
@property(nonatomic, copy, readwrite) NSString *entityIdentifier;
@property(nonatomic, assign, readwrite) uint32_t topologyIndex;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *pointLocal;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *tangent;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *normalA;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *normalB;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *startLocal;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *endLocal;
- (instancetype)initPrivate;
@end
@implementation Core3DVariableFilletIssuedEdge
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DVariableFilletOwnerValues ()
@property(nonatomic, copy, readwrite) NSData *featureIdentifier;
@property(nonatomic, copy, readwrite) NSString *entityIdentifier;
@property(nonatomic, assign, readwrite) uint32_t issuedEdgeTopologyIndex;
@property(nonatomic, assign, readwrite) NSInteger stationCount;
@property(nonatomic, assign, readwrite) double startRadiusMillimetres;
@property(nonatomic, assign, readwrite) double endRadiusMillimetres;
- (instancetype)initPrivate;
@end
@implementation Core3DVariableFilletOwnerValues
- (instancetype)initPrivate { return [super init]; }
@end

NSArray<Core3DVariableFilletOwnerValues *> *
Core3DVariableFilletOwnerValuesForDocument(
    const Handle(OcctDocument)& document, const TDF_Label& carrier,
    NSString *entityIdentifier, double metersPerUnit) {
    if (![NSThread isMainThread] || document.IsNull() || carrier.IsNull()
        || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0 || !std::isfinite(metersPerUnit)
        || metersPerUnit <= 0 || metersPerUnit > 1) return @[];
    try {
        vro::Capture capture;
        if (!document->CaptureVariableRadiusFilletOwner(carrier, capture)
            || !capture.present) return @[];

        std::size_t stationCount = 0;
        double startRadiusLocal = 0;
        double endRadiusLocal = 0;
        vrf::Definition linear;
        vrf::MultiStationDefinition stations;
        if (vrf::Decode(capture.lawBytes, linear)
            && !linear.stations.empty()) {
            stationCount = linear.stations.size();
            startRadiusLocal = linear.stations.front().radiusLocal;
            endRadiusLocal = linear.stations.back().radiusLocal;
        } else if (vrf::Decode(capture.lawBytes, stations)
                   && !stations.stations.empty()) {
            stationCount = stations.stations.size();
            startRadiusLocal = stations.stations.front().radiusLocal;
            endRadiusLocal = stations.stations.back().radiusLocal;
        } else {
            return @[];
        }
        if (stationCount > NSIntegerMax) return @[];
        const double millimetresPerLocal = 1000.0 * metersPerUnit;
        const double startRadius = startRadiusLocal * millimetresPerLocal;
        const double endRadius = endRadiusLocal * millimetresPerLocal;
        if (!std::isfinite(startRadius) || !std::isfinite(endRadius)) return @[];

        Core3DVariableFilletOwnerValues *values =
            [[Core3DVariableFilletOwnerValues alloc] initPrivate];
        if (!values) return @[];
        values.featureIdentifier = VRData(capture.feature);
        values.entityIdentifier = [entityIdentifier copy];
        values.issuedEdgeTopologyIndex = capture.topologyIndex;
        values.stationCount = static_cast<NSInteger>(stationCount);
        values.startRadiusMillimetres = startRadius;
        values.endRadiusMillimetres = endRadius;
        return @[ values ];
    } catch (...) {
        return @[];
    }
}

@interface Core3DVariableFilletLawPin ()
@property(nonatomic, assign, readwrite) double parameter;
@property(nonatomic, assign, readwrite) double positionMM;
@property(nonatomic, assign, readwrite) double radiusMM;
@property(nonatomic, assign, readwrite) double derivativeMM;
- (instancetype)initPrivate;
@end
@implementation Core3DVariableFilletLawPin
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DVariableFilletLawObservation ()
@property(nonatomic, assign, readwrite) BOOL attempted;
@property(nonatomic, assign, readwrite) BOOL supported;
@property(nonatomic, assign, readwrite) BOOL c1OnPhysicalInterval;
@property(nonatomic, assign, readwrite) BOOL nonnegativeDerivative;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *stationParameters;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *stationRadiiMM;
@property(nonatomic, copy, readwrite) NSArray<Core3DVariableFilletStation *> *stations;
@property(nonatomic, assign, readwrite) Core3DVariableFilletEdgeOrientation orientation;
@property(nonatomic, assign, readwrite) Core3DVariableFilletLawKind lawKind;
@property(nonatomic, copy, readwrite, nullable) NSData *featureIdentifier;
@property(nonatomic, copy, readwrite, nullable) NSData *ownerDefinitionIdentifier;
@property(nonatomic, copy, readwrite) NSArray<Core3DVariableFilletLawPin *> *pins;
- (instancetype)initPrivate;
@end
@implementation Core3DVariableFilletLawObservation
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DVariableFilletSectionObservation ()
@property(nonatomic, assign, readwrite) double parameter;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *planeOriginMM;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *planeNormal;
@property(nonatomic, copy, readwrite) NSArray<NSArray<NSNumber *> *> *rayPointsMM;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *rayRadiiMM;
@property(nonatomic, assign, readwrite) double contactARadiusMM;
@property(nonatomic, assign, readwrite) double contactBRadiusMM;
@property(nonatomic, assign, readwrite) double midpointRadiusMM;
@property(nonatomic, assign, readwrite) double symmetryDeviationMM;
@property(nonatomic, assign, readwrite) double planeResidualMM;
- (instancetype)initPrivate;
@end
@implementation Core3DVariableFilletSectionObservation
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DVariableFilletCandidate ()
@property(nonatomic, copy, readwrite) NSData *candidateBRepBytes;
@property(nonatomic, copy, readwrite) NSData *geometryDigest;
@property(nonatomic, copy, readwrite) NSData *familyProof;
@property(nonatomic, copy, readwrite) NSData *canonicalDefinitionBytes;
@property(nonatomic, copy, readwrite) NSArray<NSData *> *consumedEdgeIdentifiers;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@property(nonatomic, assign, readwrite) double minimumClearanceMM;
@property(nonatomic, assign, readwrite) double removedVolumeMM3;
@property(nonatomic, copy, readwrite) NSArray<Core3DVariableFilletLawObservation *> *lawObservations;
@property(nonatomic, copy, readwrite) NSArray<Core3DVariableFilletSectionObservation *> *sectionObservations;
@property(nonatomic, assign, readwrite) BOOL explicitDirectionEdit;
- (instancetype)initPrivate;
@end
@implementation Core3DVariableFilletCandidate
- (instancetype)initPrivate { return [super init]; }
@end

@interface Core3DVariableFilletOpening ()
@property(nonatomic, strong, readwrite) Core3DVariableFilletIssuedEdge *issuedEdge;
@property(nonatomic, copy, readwrite) NSString *documentIdentifier;
@property(nonatomic, copy, readwrite) NSString *contextIdentifier;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@property(nonatomic, assign, readwrite) NSInteger historyDepth;
@property(nonatomic, assign, readwrite) NSInteger dependentReplayCount;
@property(nonatomic, assign, readwrite) NSInteger measuredHistoryDelta;
@end

@implementation Core3DVariableFilletOpening {
    __weak Core3DViewController *_controller;
    Handle(OcctDocument) _document;
    std::shared_ptr<OpeningContext> _context;
    IssuedAnchor _issued;
    vro::Capture _retained;
    std::shared_ptr<vro::Prepared> _prepared;
    std::shared_ptr<std::atomic_bool> _cancelled;
    std::uint32_t _viewportWidth, _viewportHeight;
    std::uint32_t _selectionTopologyIndex;
#if DEBUG
    std::shared_ptr<std::atomic_int> _debugCancelAfterBuild;
#endif
}

- (instancetype)initPrivate { return [super init]; }

+ (instancetype)captureController:(Core3DViewController *)controller error:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !controller) {
        if (error) *error = VRError(Core3DVariableFilletErrorInvalidThread,
                                    @"main-thread controller required");
        return nil;
    }
    @try {
        Core3DSceneSnapshot *scene = [controller captureSceneSnapshot];
        if (!scene || scene.selectionMode != Core3DSceneElementKindEdge
            || scene.selection.selectedElements.count != 1) {
            if (error) *error = VRError(Core3DVariableFilletErrorInvalidSelection,
                                        @"exactly one selected native edge is required");
            return nil;
        }
        Core3DSceneElementIdentifier *element = scene.selection.selectedElements.firstObject;
        if (element.kind != Core3DSceneElementKindEdge
            || element.entityIdentifier.length == 0
            || element.entityIdentifier.length > 128) {
            if (error) *error = VRError(Core3DVariableFilletErrorInvalidSelection,
                                        @"selected element is not a native edge");
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
            if (error) *error = VRError(Core3DVariableFilletErrorUnavailable,
                                        @"native document unavailable");
            return nil;
        }
        const std::uint32_t width = std::uint32_t(drawable.width);
        const std::uint32_t height = std::uint32_t(drawable.height);
        std::string entity = element.entityIdentifier.UTF8String ?: "";
        auto context = viewer->captureNativeOpeningContext(width, height, {entity});
        const Handle(TDocStd_Document) native = document->Document();
        if (!context || native.IsNull() || native->GetData().IsNull() || native->HasOpenCommand()
            || context->openingFence().document() != native
            || context->openingFence().data() != native->GetData()
            || !context->isCurrent(width, height)) {
            if (error) *error = VRError(Core3DVariableFilletErrorStale,
                                        @"native opening fence changed");
            return nil;
        }
        const double metersPerUnit = context->openingFence().metersPerUnit();
        if (!std::isfinite(metersPerUnit) || metersPerUnit <= 0 || metersPerUnit > 1) {
            if (error) *error = VRError(Core3DVariableFilletErrorUnavailable,
                                        @"native document unit unavailable");
            return nil;
        }
        TDF_Label label;
        if (!RootForEntity(document, element.entityIdentifier, label)
            || !XCAFDoc_ShapeTool::IsFree(label) || !XCAFDoc_ShapeTool::IsSimpleShape(label)) {
            if (error) *error = VRError(Core3DVariableFilletErrorInvalidSelection,
                                        @"selected entity is not a retained native source");
            return nil;
        }
        const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(label);
        if (shape.IsNull()) {
            if (error) *error = VRError(Core3DVariableFilletErrorUnavailable,
                                        @"selected native source has no shape");
            return nil;
        }
        vro::Capture retained;
        if (!document->CaptureVariableRadiusFilletOwner(label, retained)) {
            if (error) *error = VRError(Core3DVariableFilletErrorUnavailable,
                                        @"retained owner capture failed");
            return nil;
        }
        IssuedAnchor issued;
        if (retained.present) {
            if (!RestoreRetainedAnchor(retained, entity, issued)) {
                if (error) *error = VRError(Core3DVariableFilletErrorUnavailable,
                                            @"retained owner law is invalid");
                return nil;
            }
        } else {
            issued = IssueAnchor(shape, entity, element.topologyIndex);
        }
        if (!issued.refusal.empty()) {
            if (error) {
                *error = [NSError errorWithDomain:Core3DVariableFilletErrorDomain
                                             code:issued.code
                                         userInfo:@{NSLocalizedDescriptionKey:
                                                        [NSString stringWithUTF8String:issued.refusal.c_str()],
                                                    Core3DVariableFilletRefusalReasonKey:
                                                        [NSString stringWithUTF8String:issued.refusal.c_str()]}];
            }
            return nil;
        }
        if (!retained.present)
            issued.anchor.selectorProof = SelectorProof(document->DocumentIdentifier(), entity,
                                                        element.topologyIndex, issued.anchor);
        Core3DVariableFilletIssuedEdge *edge = [[Core3DVariableFilletIssuedEdge alloc] initPrivate];
        edge.identifier = VRData(issued.anchor.identifier);
        edge.entityIdentifier = element.entityIdentifier;
        edge.topologyIndex = element.topologyIndex;
        edge.pointLocal = VRVector(issued.anchor.pointLocal);
        edge.tangent = VRVector(issued.anchor.tangent);
        edge.normalA = VRVector(issued.anchor.normalA);
        edge.normalB = VRVector(issued.anchor.normalB);
        edge.startLocal = VRVector(issued.anchor.startLocal);
        edge.endLocal = VRVector(issued.anchor.endLocal);
        Core3DVariableFilletOpening *opening = [[self alloc] initPrivate];
        opening->_controller = controller;
        opening->_document = document;
        opening->_context = std::move(context);
        opening->_issued = std::move(issued);
        opening->_retained = std::move(retained);
        opening->_cancelled = std::make_shared<std::atomic_bool>(false);
        opening->_viewportWidth = width;
        opening->_viewportHeight = height;
        opening->_selectionTopologyIndex = element.topologyIndex;
#if DEBUG
        opening->_debugCancelAfterBuild = std::make_shared<std::atomic_int>(0);
#endif
        opening.issuedEdge = edge;
        opening.documentIdentifier = [NSString stringWithUTF8String:document->DocumentIdentifier().c_str()];
        opening.contextIdentifier = NSUUID.UUID.UUIDString;
        opening.metersPerUnit = metersPerUnit;
        opening.historyDepth = controller.documentUndoCount;
        opening.dependentReplayCount = 0;
        opening.measuredHistoryDelta = 0;
        return opening;
    } @catch (__unused NSException *exception) {
        if (error) *error = VRError(Core3DVariableFilletErrorUnavailable,
                                    @"native capture exception");
        return nil;
    }
}

- (BOOL)isCurrent:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !_controller) {
        if (error) *error = VRError(Core3DVariableFilletErrorInvalidThread,
                                    @"main-thread live controller required");
        return NO;
    }
    if (_cancelled->load()) {
        if (error) *error = VRError(Core3DVariableFilletErrorCancelled, @"opening cancelled");
        return NO;
    }
    if (!_context || !_context->isCurrent(_viewportWidth, _viewportHeight)) {
        if (error) *error = VRError(Core3DVariableFilletErrorStale,
                                    @"native opening fence changed");
        return NO;
    }
    Core3DSceneSnapshot *scene = [_controller captureSceneSnapshot];
    if (!scene || scene.selectionMode != Core3DSceneElementKindEdge
        || scene.selection.selectedElements.count != 1) {
        if (error) *error = VRError(Core3DVariableFilletErrorStale,
                                    @"native edge selection changed");
        return NO;
    }
    Core3DSceneElementIdentifier *element = scene.selection.selectedElements.firstObject;
    if (element.kind != Core3DSceneElementKindEdge
        || element.topologyIndex != _selectionTopologyIndex
        || _issued.entity != (element.entityIdentifier.UTF8String ?: "")
        || _controller.documentUndoCount != self.historyDepth) {
        if (error) *error = VRError(Core3DVariableFilletErrorStale,
                                    @"captured dependency changed");
        return NO;
    }
    return YES;
}

- (void)cancel { if (_cancelled) _cancelled->store(true); }

// Shared detached build core. Precondition: isCurrent held, arguments valid.
- (OpeningBuild)buildLinear:(double)startRadiusMM endRadiusMM:(double)endRadiusMM
                  reversed:(BOOL)reversed
                    budget:(rf::ReplayBudget&)budget {
    OpeningBuild result;
    @try {
        const double unit = self.metersPerUnit;
        const double scale = 0.001 / unit;
        vrf::Definition definition;
        definition.schema = vrf::CodecVersion;
        definition.law = vrf::LawKind::LinearStartEnd;
        definition.convention = vrf::ParameterConvention::OrientedNormalizedArcLength;
        definition.metersPerLocalUnit = unit;
        definition.stations[0] = { MintUUID(), 0.0, startRadiusMM * scale };
        definition.stations[1] = { MintUUID(), 1.0, endRadiusMM * scale };
        definition.edges.push_back(_issued.anchor);
        const vrf::Orientation requestedOrientation = reversed
            ? vrf::Orientation::Reversed : vrf::Orientation::Forward;
        if (definition.edges[0].orientation != requestedOrientation) {
            std::swap(definition.edges[0].startLocal, definition.edges[0].endLocal);
            definition.edges[0].orientation = requestedOrientation;
        }
        result.anchor = definition.edges[0];
        vrf::Refusal validity = vrf::Refusal::None;
        if (!vrf::Validate(definition, validity)) { result.refusal = validity; return result; }
        if (!vrf::Encode(definition, result.parameters)) return result;
        {   // Canonical self-check: decode/re-encode must round-trip exactly.
            vrf::Definition decoded; std::vector<std::uint8_t> reencoded;
            if (!vrf::Decode(result.parameters, decoded) || !(decoded == definition)
                || !vrf::Encode(decoded, reencoded) || reencoded != result.parameters) {
                result.refusal = vrf::Refusal::MalformedPayload; return result;
            }
        }
        cr::FeatureNode feature;
        feature.node = MintUUID(); feature.feature = MintUUID();
        feature.kind = vrf::FeatureKind; feature.codecVersion = vrf::CodecVersion;
        feature.inputs.push_back(MintUUID());
        feature.parameters = result.parameters;
        rf::ReplayValue input;
        input.shape = rf::ShapeKind::Solid;
        input.detachedShape = _issued.sourceBytes;
        const std::vector<rf::ReplayValue> inputs{input};
        const std::size_t workBefore = budget.workRemaining;
        const std::size_t identitiesBefore = budget.identitiesRemaining;
        const std::size_t topologyBefore = budget.topologyRemaining;
        if (!budget.consume(1, 2, _issued.sourceBytes.size())) {
            result.refusal = vrf::Refusal::Budget;
            result.budgetExhausted = budget.exhausted;
            return result;
        }
#if DEBUG
        vrf::DebugBuildObservation debugObservation;
        vrf::DebugBuildObservation *previousObserver = vrf::debugBuildObservation;
        vrf::debugBuildObservation = &debugObservation;
#endif
        const bool built = vrf::detail::buildDetachedObserved(
            feature, inputs, budget, result.output, *_cancelled,
            &result.refusal, &result.evidence);
#if DEBUG
        vrf::debugBuildObservation = previousObserver;
        result.realizedLaws = debugObservation.realizedLaws;
#endif
        result.consumedWork = workBefore - budget.workRemaining;
        result.consumedIdentities = identitiesBefore - budget.identitiesRemaining;
        result.consumedTopology = topologyBefore - budget.topologyRemaining;
        result.budgetExhausted = budget.exhausted;
        result.cancelledObserved = result.refusal == vrf::Refusal::Cancelled;
        if (!built) return result;
#if DEBUG
        if (_debugCancelAfterBuild && _debugCancelAfterBuild->exchange(0) == 1) {
            // Stop arrived during the uninterruptible kernel call: drain
            // without delivering a candidate.
            _cancelled->store(true);
            result.output = {};
            result.refusal = vrf::Refusal::Cancelled;
            result.cancelledObserved = true;
            return result;
        }
#endif
        result.built = true;
        return result;
    } @catch (__unused NSException *exception) { return result; }
}

- (OpeningBuild)buildMultiStation:(NSArray<Core3DVariableFilletStation *> *)stations
                        reversed:(BOOL)reversed
                          budget:(rf::ReplayBudget&)budget {
    OpeningBuild result;
    @try {
        const double unit = self.metersPerUnit;
        const double scale = 0.001 / unit;
        vrf::MultiStationDefinition definition;
        definition.schema = vrf::MultiStationCodecVersion;
        definition.law = vrf::LawKind::MultiStation;
        definition.convention = vrf::ParameterConvention::OrientedNormalizedArcLength;
        definition.metersPerLocalUnit = unit;
        for (Core3DVariableFilletStation *station in stations) {
            vrf::Station value;
            std::memcpy(value.identifier.data(), station.identifier.bytes, 16);
            value.parameter = station.parameter;
            value.radiusLocal = station.radiusMM * scale;
            definition.stations.push_back(value);
        }
        definition.edges.push_back(_issued.anchor);
        const vrf::Orientation requestedOrientation = reversed
            ? vrf::Orientation::Reversed : vrf::Orientation::Forward;
        if (definition.edges[0].orientation != requestedOrientation) {
            std::swap(definition.edges[0].startLocal, definition.edges[0].endLocal);
            definition.edges[0].orientation = requestedOrientation;
        }
        result.anchor = definition.edges[0];
        vrf::Refusal validity = vrf::Refusal::None;
        if (!vrf::Validate(definition, validity)) { result.refusal = validity; return result; }
        if (!vrf::Encode(definition, result.parameters)) return result;
        {
            vrf::MultiStationDefinition decoded; std::vector<std::uint8_t> reencoded;
            if (!vrf::Decode(result.parameters, decoded) || !(decoded == definition)
                || !vrf::Encode(decoded, reencoded) || reencoded != result.parameters) {
                result.refusal = vrf::Refusal::MalformedPayload; return result;
            }
        }
        cr::FeatureNode feature;
        feature.node = MintUUID(); feature.feature = MintUUID();
        feature.kind = vrf::FeatureKind; feature.codecVersion = vrf::MultiStationCodecVersion;
        feature.inputs.push_back(MintUUID());
        feature.parameters = result.parameters;
        rf::ReplayValue input;
        input.shape = rf::ShapeKind::Solid;
        input.detachedShape = _issued.sourceBytes;
        const std::vector<rf::ReplayValue> inputs{input};
        const std::size_t workBefore = budget.workRemaining;
        const std::size_t identitiesBefore = budget.identitiesRemaining;
        const std::size_t topologyBefore = budget.topologyRemaining;
        if (!budget.consume(1, 2, _issued.sourceBytes.size())) {
            result.refusal = vrf::Refusal::Budget;
            result.budgetExhausted = budget.exhausted;
            return result;
        }
#if DEBUG
        vrf::DebugBuildObservation debugObservation;
        vrf::DebugBuildObservation *previousObserver = vrf::debugBuildObservation;
        vrf::debugBuildObservation = &debugObservation;
#endif
        const bool built = vrf::detail::buildDetachedMultiStationObserved(
            feature, inputs, budget, result.output, *_cancelled,
            &result.refusal, &result.evidence);
#if DEBUG
        vrf::debugBuildObservation = previousObserver;
        result.multiStationLaws = debugObservation.multiStationLaws;
#endif
        result.consumedWork = workBefore - budget.workRemaining;
        result.consumedIdentities = identitiesBefore - budget.identitiesRemaining;
        result.consumedTopology = topologyBefore - budget.topologyRemaining;
        result.budgetExhausted = budget.exhausted;
        result.cancelledObserved = result.refusal == vrf::Refusal::Cancelled;
        if (!built) return result;
#if DEBUG
        if (_debugCancelAfterBuild && _debugCancelAfterBuild->exchange(0) == 1) {
            _cancelled->store(true);
            result.output = {};
            result.refusal = vrf::Refusal::Cancelled;
            result.cancelledObserved = true;
            return result;
        }
#endif
        result.built = true;
        return result;
    } @catch (__unused NSException *exception) { return result; }
}

- (Core3DVariableFilletCandidate *)deliverCandidate:(OpeningBuild&)build
                                              error:(NSError **)error {
    if (_cancelled->load() || build.cancelledObserved) {
        if (error) *error = VRRefusal(Core3DVariableFilletErrorCancelled, vrf::Refusal::Cancelled);
        return nil;
    }
    const double mmPerLocal = 1000.0 * self.metersPerUnit;
    std::string failure;
    if (!ObserveSections(build.output.detachedShape, build.anchor, mmPerLocal,
                         build.sections, failure)) {
        if (error) *error = VRError(Core3DVariableFilletErrorBuildRefused,
                                    [NSString stringWithUTF8String:
                                        ("b3.section-observation: " + failure).c_str()]);
        return nil;
    }
    Core3DVariableFilletCandidate *candidate = [[Core3DVariableFilletCandidate alloc] initPrivate];
    candidate.candidateBRepBytes = VRData(build.output.detachedShape);
    candidate.geometryDigest = VRData(build.output.geometry);
    candidate.familyProof = VRData(build.output.familyProof);
    candidate.canonicalDefinitionBytes = VRData(build.parameters);
    NSMutableArray<NSData *> *consumed = [NSMutableArray array];
    for (const auto& edge : build.evidence.consumedEdges) [consumed addObject:VRData(edge)];
    candidate.consumedEdgeIdentifiers = consumed;
    candidate.metersPerUnit = self.metersPerUnit;
    candidate.minimumClearanceMM = build.evidence.minimumClearanceLocal * mmPerLocal;
    candidate.removedVolumeMM3 = build.evidence.removedVolumeLocal3
        * mmPerLocal * mmPerLocal * mmPerLocal;
    NSMutableArray<Core3DVariableFilletSectionObservation *> *sections = [NSMutableArray array];
    for (const SectionObservationValue& value : build.sections) {
        Core3DVariableFilletSectionObservation *section =
            [[Core3DVariableFilletSectionObservation alloc] initPrivate];
        section.parameter = value.parameter;
        section.planeOriginMM = VRVector(value.planeOriginMM);
        section.planeNormal = VRVector(value.planeNormal);
        NSMutableArray<NSArray<NSNumber *> *> *points = [NSMutableArray array];
        NSMutableArray<NSNumber *> *radii = [NSMutableArray array];
        for (unsigned ray = 0; ray < 5; ++ray) {
            [points addObject:@[ @(value.rayPointsMM[ray][0]), @(value.rayPointsMM[ray][1]) ]];
            [radii addObject:@(value.rayRadiiMM[ray])];
        }
        section.rayPointsMM = points;
        section.rayRadiiMM = radii;
        section.contactARadiusMM = value.contactARadiusMM;
        section.contactBRadiusMM = value.contactBRadiusMM;
        section.midpointRadiusMM = value.midpointRadiusMM;
        section.symmetryDeviationMM = value.symmetryDeviationMM;
        section.planeResidualMM = value.planeResidualMM;
        [sections addObject:section];
    }
    candidate.sectionObservations = sections;
    NSMutableArray<Core3DVariableFilletLawObservation *> *laws = [NSMutableArray array];
#if DEBUG
    for (const auto& law : build.realizedLaws) {
        Core3DVariableFilletLawObservation *observation =
            [[Core3DVariableFilletLawObservation alloc] initPrivate];
        observation.attempted = law.attempted;
        observation.supported = law.supported;
        observation.c1OnPhysicalInterval = law.c1OnPhysicalInterval;
        observation.nonnegativeDerivative = law.nonnegativeDerivative;
        observation.stationParameters = @[ @(law.stationParameters[0]),
                                           @(law.stationParameters[1]) ];
        observation.stationRadiiMM = @[ @(law.stationRadii[0] * mmPerLocal),
                                        @(law.stationRadii[1] * mmPerLocal) ];
        observation.stations = @[];
        observation.orientation = build.anchor.orientation == vrf::Orientation::Forward
            ? Core3DVariableFilletEdgeOrientationForward
            : Core3DVariableFilletEdgeOrientationReversed;
        observation.lawKind = Core3DVariableFilletLawKindLinearTwoStationV1;
        NSMutableArray<Core3DVariableFilletLawPin *> *pins = [NSMutableArray array];
        // The landed F01 r2 observation samples the law at the fixed authored
        // grid t = {0, .2, .5, .8, 1} (observeRealizedLaw's `samples`);
        // sampleParameters carries the NATIVE physical parameter of each
        // sample (e.g. 0/8/20/32/40 local), so the pin's authored t is the
        // grid itself, reported exactly.
        constexpr std::array<double, 5> authoredSampleGrid{{0, 0.2, 0.5, 0.8, 1}};
        for (unsigned index = 0; index < 5; ++index) {
            Core3DVariableFilletLawPin *pin = [[Core3DVariableFilletLawPin alloc] initPrivate];
            pin.parameter = authoredSampleGrid[index];
            pin.positionMM = law.samplePositionsZ[index] * mmPerLocal;
            pin.radiusMM = law.sampleValues[index] * mmPerLocal;
            pin.derivativeMM = law.sampleDerivatives[index] * mmPerLocal;
            [pins addObject:pin];
        }
        observation.pins = pins;
        [laws addObject:observation];
    }
    for (const auto& law : build.multiStationLaws) {
        Core3DVariableFilletLawObservation *observation =
            [[Core3DVariableFilletLawObservation alloc] initPrivate];
        observation.attempted = law.attempted;
        observation.supported = !law.stations.empty();
        observation.c1OnPhysicalInterval = law.attempted && !law.exception;
        observation.nonnegativeDerivative = law.attempted && !law.exception;
        NSMutableArray<NSNumber *> *parameters = [NSMutableArray array];
        NSMutableArray<NSNumber *> *radii = [NSMutableArray array];
        for (const auto& station : law.stations) {
            [parameters addObject:@(station.authoredParameter)];
            [radii addObject:@(station.realizedRadius * mmPerLocal)];
        }
        observation.stationParameters = parameters;
        observation.stationRadiiMM = radii;
        observation.stations = @[];
        observation.orientation = build.anchor.orientation == vrf::Orientation::Forward
            ? Core3DVariableFilletEdgeOrientationForward
            : Core3DVariableFilletEdgeOrientationReversed;
        observation.lawKind = Core3DVariableFilletLawKindMultiStationV2;
        observation.pins = @[];
        [laws addObject:observation];
    }
#endif
    candidate.lawObservations = laws;
    return candidate;
}

- (Core3DVariableFilletCandidate *)detachedCandidateWithStartRadiusMM:(double)startRadiusMM
                                                          endRadiusMM:(double)endRadiusMM
                                                          orientation:(Core3DVariableFilletEdgeOrientation)orientation
                                                                error:(NSError **)error {
    if (error) *error = nil;
    if (!std::isfinite(startRadiusMM) || !std::isfinite(endRadiusMM)
        || startRadiusMM < kMinimumRadiusMM || startRadiusMM > kMaximumRadiusMM
        || endRadiusMM < kMinimumRadiusMM || endRadiusMM > kMaximumRadiusMM) {
        if (error) *error = VRRefusal(Core3DVariableFilletErrorInvalidLaw,
                                      vrf::Refusal::InvalidRadius);
        return nil;
    }
    if (orientation != Core3DVariableFilletEdgeOrientationForward
        && orientation != Core3DVariableFilletEdgeOrientationReversed) {
        if (error) *error = VRError(Core3DVariableFilletErrorInvalidArgument,
                                    @"unknown edge orientation");
        return nil;
    }
    if (![self isCurrent:error]) return nil;
    rf::ReplayBudget budget;
    OpeningBuild build = [self buildLinear:startRadiusMM endRadiusMM:endRadiusMM
                                  reversed:orientation == Core3DVariableFilletEdgeOrientationReversed
                                    budget:budget];
    if (!build.built) {
        if (error) {
            const Core3DVariableFilletError code =
                build.refusal == vrf::Refusal::Cancelled ? Core3DVariableFilletErrorCancelled
                : build.refusal == vrf::Refusal::Budget ? Core3DVariableFilletErrorBudget
                : Core3DVariableFilletErrorBuildRefused;
            *error = VRRefusal(code, build.refusal);
        }
        return nil;
    }
    return [self deliverCandidate:build error:error];
}

- (Core3DVariableFilletCandidate *)detachedCandidateWithStations:(NSArray<Core3DVariableFilletStation *> *)stations
                                                     orientation:(Core3DVariableFilletEdgeOrientation)orientation
                                                           error:(NSError **)error {
    if (error) *error = nil;
    // Station count/order/identity admission is owned by the native
    // validator (b3.invalid-station-identity below three, b3.budget above
    // sixteen); no test-side limits are substituted here.
    if (![stations isKindOfClass:NSArray.class]) {
        if (error) *error = VRError(Core3DVariableFilletErrorInvalidArgument,
                                    @"station list required");
        return nil;
    }
    for (Core3DVariableFilletStation *station in stations) {
        if (![station isKindOfClass:Core3DVariableFilletStation.class]
            || station.radiusMM < kMinimumRadiusMM || station.radiusMM > kMaximumRadiusMM) {
            if (error) *error = VRRefusal(Core3DVariableFilletErrorInvalidLaw,
                                          vrf::Refusal::InvalidRadius);
            return nil;
        }
    }
    if (orientation != Core3DVariableFilletEdgeOrientationForward
        && orientation != Core3DVariableFilletEdgeOrientationReversed) {
        if (error) *error = VRError(Core3DVariableFilletErrorInvalidArgument,
                                    @"unknown edge orientation");
        return nil;
    }
    if (![self isCurrent:error]) return nil;
    rf::ReplayBudget budget;
    OpeningBuild build = [self buildMultiStation:stations
                                       reversed:orientation == Core3DVariableFilletEdgeOrientationReversed
                                         budget:budget];
    if (!build.built) {
        if (error) {
            const Core3DVariableFilletError code =
                build.refusal == vrf::Refusal::Cancelled ? Core3DVariableFilletErrorCancelled
                : build.refusal == vrf::Refusal::Budget ? Core3DVariableFilletErrorBudget
                : Core3DVariableFilletErrorBuildRefused;
            *error = VRRefusal(code, build.refusal);
        }
        return nil;
    }
    return [self deliverCandidate:build error:error];
}

- (Core3DVariableFilletCandidate *)detachedDirectionEditCandidateWithError:(NSError **)error {
    if (error) *error = nil;
    if (![self isCurrent:error]) return nil;
    if (!_retained.present) {
        if (error) *error = VRError(Core3DVariableFilletErrorApplyUnavailable,
                                    @"retained variable fillet owner is absent");
        return nil;
    }
    @try {
        const double mmPerLocal = 1000.0 * self.metersPerUnit;
        rf::ReplayBudget budget;
        OpeningBuild build;
        vrf::Definition linear;
        vrf::MultiStationDefinition multi;
        if (vrf::Decode(_retained.lawBytes, linear) && linear.edges.size() == 1) {
            const BOOL reversed = linear.edges.front().orientation == vrf::Orientation::Forward;
            build = [self buildLinear:linear.stations[1].radiusLocal * mmPerLocal
                          endRadiusMM:linear.stations[0].radiusLocal * mmPerLocal
                             reversed:reversed budget:budget];
        } else if (vrf::Decode(_retained.lawBytes, multi) && multi.edges.size() == 1) {
            NSMutableArray<Core3DVariableFilletStation *> *stations = [NSMutableArray array];
            for (auto value = multi.stations.rbegin(); value != multi.stations.rend(); ++value) {
                Core3DVariableFilletStation *station =
                    [[Core3DVariableFilletStation alloc]
                        initWithIdentifier:VRData(value->identifier)
                                 parameter:1.0 - value->parameter
                                  radiusMM:value->radiusLocal * mmPerLocal];
                if (!station) {
                    if (error) *error = VRError(Core3DVariableFilletErrorUnavailable,
                                                @"retained station is malformed");
                    return nil;
                }
                [stations addObject:station];
            }
            const BOOL reversed = multi.edges.front().orientation == vrf::Orientation::Forward;
            build = [self buildMultiStation:stations reversed:reversed budget:budget];
        } else {
            if (error) *error = VRError(Core3DVariableFilletErrorUnavailable,
                                        @"retained variable fillet law is malformed");
            return nil;
        }
        if (!build.built) {
            if (error) *error = VRRefusal(
                build.refusal == vrf::Refusal::Cancelled
                    ? Core3DVariableFilletErrorCancelled
                    : build.refusal == vrf::Refusal::Budget
                        ? Core3DVariableFilletErrorBudget
                        : Core3DVariableFilletErrorBuildRefused,
                build.refusal);
            return nil;
        }
        Core3DVariableFilletCandidate *candidate = [self deliverCandidate:build error:error];
        candidate.explicitDirectionEdit = candidate != nil;
        return candidate;
    } @catch (__unused NSException *exception) {
        if (error) *error = VRError(Core3DVariableFilletErrorUnavailable,
                                    @"direction edit candidate exception");
        return nil;
    }
}

- (Core3DVariableFilletLawObservation *)readRetainedLawWithError:(NSError **)error {
    if (error) *error = nil;
    if (![self isCurrent:error]) return nil;
    try {
        TDF_Label label;
        if (!RootForEntity(_document, self.issuedEdge.entityIdentifier, label)) {
            if (error) *error = VRError(Core3DVariableFilletErrorStale,
                                        @"retained owner entity is unavailable");
            return nil;
        }
        vro::Capture capture;
        if (!_document->CaptureVariableRadiusFilletOwner(label, capture)
            || !capture.present) {
            if (error) *error = VRError(Core3DVariableFilletErrorApplyUnavailable,
                                        @"retained variable fillet owner is absent");
            return nil;
        }
        Core3DVariableFilletLawObservation *observation =
            [[Core3DVariableFilletLawObservation alloc] initPrivate];
        NSMutableArray<NSNumber *> *parameters = [NSMutableArray array];
        NSMutableArray<NSNumber *> *radii = [NSMutableArray array];
        NSMutableArray<Core3DVariableFilletStation *> *readStations = [NSMutableArray array];
        const double mmPerLocal = capture.present ? 1000.0 * self.metersPerUnit : 0;
        vrf::Definition linear;
        vrf::MultiStationDefinition stations;
        if (vrf::Decode(capture.lawBytes, linear)) {
            for (const auto& station : linear.stations) {
                [parameters addObject:@(station.parameter)];
                [radii addObject:@(station.radiusLocal * mmPerLocal)];
                Core3DVariableFilletStation *value = [[Core3DVariableFilletStation alloc]
                    initWithIdentifier:VRData(station.identifier)
                             parameter:station.parameter
                              radiusMM:station.radiusLocal * mmPerLocal];
                if (!value) return nil;
                [readStations addObject:value];
            }
            observation.orientation = linear.edges.front().orientation
                == vrf::Orientation::Forward
                ? Core3DVariableFilletEdgeOrientationForward
                : Core3DVariableFilletEdgeOrientationReversed;
            observation.lawKind = Core3DVariableFilletLawKindLinearTwoStationV1;
        } else if (vrf::Decode(capture.lawBytes, stations)) {
            for (const auto& station : stations.stations) {
                [parameters addObject:@(station.parameter)];
                [radii addObject:@(station.radiusLocal * mmPerLocal)];
                Core3DVariableFilletStation *value = [[Core3DVariableFilletStation alloc]
                    initWithIdentifier:VRData(station.identifier)
                             parameter:station.parameter
                              radiusMM:station.radiusLocal * mmPerLocal];
                if (!value) return nil;
                [readStations addObject:value];
            }
            observation.orientation = stations.edges.front().orientation
                == vrf::Orientation::Forward
                ? Core3DVariableFilletEdgeOrientationForward
                : Core3DVariableFilletEdgeOrientationReversed;
            observation.lawKind = Core3DVariableFilletLawKindMultiStationV2;
        } else {
            if (error) *error = VRError(Core3DVariableFilletErrorUnavailable,
                                        @"retained variable fillet law is malformed");
            return nil;
        }
        observation.attempted = YES;
        observation.supported = YES;
        observation.c1OnPhysicalInterval = YES;
        observation.nonnegativeDerivative = YES;
        observation.stationParameters = parameters;
        observation.stationRadiiMM = radii;
        observation.stations = readStations;
        observation.featureIdentifier = VRData(capture.feature);
        vro::UUID ownerDefinition{};
        if (core3d::receipt::ParseUUID(
                capture.exact.visibility.object.object.definitionIdentifier,
                ownerDefinition)
            && core3d::retained_recipe::Nonzero(ownerDefinition))
            observation.ownerDefinitionIdentifier = VRData(ownerDefinition);
        observation.pins = @[];
        return observation;
    } catch (...) {
        if (error) *error = VRError(Core3DVariableFilletErrorUnavailable,
                                    @"retained law readback exception");
        return nil;
    }
}

- (Core3DVariableFilletOpening *)prepareCandidate:(Core3DVariableFilletCandidate *)candidate
                                             fault:(vro::FaultPoint)fault
                                             error:(NSError **)error {
    if (error) *error = nil;
    if (![candidate isKindOfClass:Core3DVariableFilletCandidate.class]
        || candidate.candidateBRepBytes.length == 0
        || candidate.canonicalDefinitionBytes.length == 0
        || candidate.consumedEdgeIdentifiers.count != 1
        || ![candidate.consumedEdgeIdentifiers.firstObject isEqual:self.issuedEdge.identifier]
        || candidate.metersPerUnit != self.metersPerUnit) {
        if (error) *error = VRError(Core3DVariableFilletErrorInvalidArgument,
                                    @"candidate does not belong to this issued edge");
        return nil;
    }
    if (![self isCurrent:error]) return nil;
    try {
        TDF_Label label;
        if (!RootForEntity(_document, self.issuedEdge.entityIdentifier, label)) {
            if (error) *error = VRError(Core3DVariableFilletErrorStale,
                                        @"candidate source entity is unavailable");
            return nil;
        }
        auto prepared = std::make_shared<vro::Prepared>();
        if (!vro::Prepare(*_document, label, _context,
                _viewportWidth, _viewportHeight, _issued.topologyIndex,
                VRBytes(candidate.candidateBRepBytes),
                VRBytes(candidate.canonicalDefinitionBytes), *prepared, fault,
                candidate.explicitDirectionEdit
                    ? vro::EditIntent::ReverseDirection : vro::EditIntent::Ordinary)) {
            if (error) *error = VRError(Core3DVariableFilletErrorBuildRefused,
                                        @"dependent replay preparation refused");
            return nil;
        }
        Core3DVariableFilletOpening *opening = [[Core3DVariableFilletOpening alloc] initPrivate];
        opening->_controller = _controller;
        opening->_document = _document;
        opening->_context = _context;
        opening->_issued = _issued;
        opening->_retained = prepared->before;
        opening->_prepared = std::move(prepared);
        opening->_cancelled = std::make_shared<std::atomic_bool>(false);
        opening->_viewportWidth = _viewportWidth;
        opening->_viewportHeight = _viewportHeight;
        opening->_selectionTopologyIndex = _selectionTopologyIndex;
#if DEBUG
        opening->_debugCancelAfterBuild = std::make_shared<std::atomic_int>(0);
#endif
        opening.issuedEdge = self.issuedEdge;
        opening.documentIdentifier = self.documentIdentifier;
        opening.contextIdentifier = self.contextIdentifier;
        opening.metersPerUnit = self.metersPerUnit;
        opening.historyDepth = self.historyDepth;
        opening.dependentReplayCount = NSInteger(opening->_prepared->replay->dependencies().size());
        opening.measuredHistoryDelta = 0;
        return opening;
    } catch (...) {
        if (error) *error = VRError(Core3DVariableFilletErrorUnavailable,
                                    @"candidate preparation exception");
        return nil;
    }
}

- (Core3DVariableFilletOpening *)openingByPreparingCandidate:(Core3DVariableFilletCandidate *)candidate
                                                       error:(NSError **)error {
    return [self prepareCandidate:candidate fault:vro::FaultPoint::None error:error];
}

- (Core3DVariableFilletApplyResult)apply {
    if (!_prepared || _cancelled == nullptr)
        return Core3DVariableFilletApplyResultUnavailable;
    const vro::Result result = vro::Apply(*_document, *_prepared, *_cancelled);
    self.measuredHistoryDelta = result.measuredHistoryDelta;
    switch (result.outcome) {
        case vro::Outcome::Refused: return Core3DVariableFilletApplyResultRefused;
        case vro::Outcome::Cancelled: return Core3DVariableFilletApplyResultCancelled;
        case vro::Outcome::Committed:
            _retained = _prepared->expected;
            return Core3DVariableFilletApplyResultCommitted;
        case vro::Outcome::OutcomeUnknown:
            return Core3DVariableFilletApplyResultOutcomeUnknown;
        case vro::Outcome::RecoveryRequired:
            return Core3DVariableFilletApplyResultRecoveryRequired;
    }
    return Core3DVariableFilletApplyResultRecoveryRequired;
}

- (Core3DVariableFilletApplyResult)reconcile {
    if (!_prepared) return Core3DVariableFilletApplyResultUnavailable;
    switch (vro::Reconcile(*_document, *_prepared)) {
        case vro::Outcome::Refused: return Core3DVariableFilletApplyResultRefused;
        case vro::Outcome::Cancelled: return Core3DVariableFilletApplyResultCancelled;
        case vro::Outcome::Committed: return Core3DVariableFilletApplyResultCommitted;
        case vro::Outcome::OutcomeUnknown: return Core3DVariableFilletApplyResultOutcomeUnknown;
        case vro::Outcome::RecoveryRequired:
            return Core3DVariableFilletApplyResultRecoveryRequired;
    }
    return Core3DVariableFilletApplyResultRecoveryRequired;
}

#if DEBUG
- (void)b3DebugArmCancelAfterBuild {
    if (_debugCancelAfterBuild) _debugCancelAfterBuild->store(1);
}
- (NSDictionary *)b3DebugOwnerEvidence {
    if (!_prepared) return @{};
    vro::UUID feature{};
    std::uint32_t topology = 0;
    std::vector<std::uint8_t> law, base;
    if (!vro::DecodeOwner(_prepared->ownerBytes, feature, topology, law, base))
        return @{};
    return @{
        @"dependentCount": @(self.dependentReplayCount),
        @"priorOwner": VRData(_prepared->before.ownerBytes),
        @"expectedOwner": VRData(_prepared->ownerBytes),
        @"law": VRData(_prepared->lawBytes),
        @"base": VRData(base),
        @"feature": VRData(feature),
    };
}
#endif

@end

#if DEBUG
// DEBUG probe used by ShapeyardTests/B3VariableFilletNativeTests.swift to
// exercise the same detached build path with explicit shared-budget limits
// and an armed post-build Stop checkpoint. It mutates nothing and mints no
// authority; production decisions never read it.
extern "C" void *Core3DDebugB3VariableFilletOwnerProbe(
    void *openingRaw, void *candidateRaw, int32_t faultRaw,
    int32_t cancelBeforeApply) {
    @autoreleasepool {
        Core3DVariableFilletOpening *opening =
            (__bridge Core3DVariableFilletOpening *)openingRaw;
        Core3DVariableFilletCandidate *candidate =
            (__bridge Core3DVariableFilletCandidate *)candidateRaw;
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        result[@"prepared"] = @NO;
        if (!opening || !candidate || ![NSThread isMainThread]
            || faultRaw < 0 || faultRaw > int32_t(vro::FaultPoint::Publication))
            return (__bridge_retained void *)result;
        NSError *error = nil;
        Core3DVariableFilletOpening *prepared =
            [opening prepareCandidate:candidate
                                fault:vro::FaultPoint(faultRaw)
                                error:&error];
        result[@"error"] = error.localizedDescription ?: @"";
        if (!prepared) return (__bridge_retained void *)result;
        result[@"prepared"] = @YES;
        [result addEntriesFromDictionary:[prepared b3DebugOwnerEvidence]];
        result[@"opening"] = prepared;
        if (cancelBeforeApply == 1) [prepared cancel];
        const Core3DVariableFilletApplyResult applied = [prepared apply];
        result[@"applyRaw"] = @(applied);
        result[@"historyDelta"] = @(prepared.measuredHistoryDelta);
        return (__bridge_retained void *)result;
    }
}

extern "C" void *Core3DDebugB3VariableFilletOpeningProbe(
    void *openingRaw, int32_t scenario, double workRemaining,
    double identitiesRemaining, double topologyRemaining,
    int32_t armCancelAfterBuild) {
    @autoreleasepool {
        Core3DVariableFilletOpening *opening =
            (__bridge Core3DVariableFilletOpening *)openingRaw;
        NSMutableDictionary *result = [NSMutableDictionary dictionary];
        result[@"ok"] = @YES;
        if (!opening || ![NSThread isMainThread]) {
            result[@"ok"] = @NO; return (__bridge_retained void *)result;
        }
        rf::ReplayBudget budget;
        if (workRemaining > 0) budget.workRemaining = std::size_t(workRemaining);
        if (identitiesRemaining > 0) budget.identitiesRemaining = std::size_t(identitiesRemaining);
        if (topologyRemaining > 0) budget.topologyRemaining = std::size_t(topologyRemaining);
        OpeningBuild build;
        @try {
            if (armCancelAfterBuild == 1)
                [opening b3DebugArmCancelAfterBuild];
            if (scenario == 1) {
                build = [opening buildLinear:2 endRadiusMM:5 reversed:NO budget:budget];
            } else if (scenario == 2) {
                Core3DVariableFilletStation *(^station)(unsigned, double, double) =
                    ^Core3DVariableFilletStation *(unsigned tag, double t, double radius) {
                    unsigned char bytes[16] = {};
                    bytes[15] = static_cast<unsigned char>(0x40 + tag);
                    return [[Core3DVariableFilletStation alloc]
                        initWithIdentifier:[NSData dataWithBytes:bytes length:16]
                                 parameter:t radiusMM:radius];
                };
                NSArray<Core3DVariableFilletStation *> *stations = @[
                    station(0, 0, 2), station(1, 0.25, 3), station(2, 0.5, 4),
                    station(3, 0.75, 6), station(4, 1, 8) ];
                build = [opening buildMultiStation:stations reversed:NO budget:budget];
            } else {
                result[@"ok"] = @NO; return (__bridge_retained void *)result;
            }
        } @catch (__unused NSException *exception) {
            result[@"ok"] = @NO; return (__bridge_retained void *)result;
        }
        result[@"built"] = @(build.built);
        result[@"refusalRaw"] = @(static_cast<int>(build.refusal));
        result[@"reason"] = [NSString stringWithUTF8String:vrf::Reason(build.refusal)];
        result[@"consumedWork"] = @(double(build.consumedWork));
        result[@"consumedIdentities"] = @(double(build.consumedIdentities));
        result[@"consumedTopology"] = @(double(build.consumedTopology));
        result[@"budgetExhausted"] = @(build.budgetExhausted);
        result[@"cancelledObserved"] = @(build.cancelledObserved);
        result[@"candidateBytes"] = build.built ? VRData(build.output.detachedShape)
                                                : (NSData *)NSData.data;
        result[@"consumedCount"] = @(double(build.evidence.consumedEdges.size()));
        return (__bridge_retained void *)result;
    }
}
#endif
