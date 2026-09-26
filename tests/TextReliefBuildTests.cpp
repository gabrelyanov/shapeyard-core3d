#include "TextReliefBuild.hxx"
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax2.hxx>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

using namespace core3d;

namespace {
namespace bc = bounded_curve;
namespace tr = text_relief;
namespace rb = retained_part_boolean;

constexpr double Scale = 0.001; // metres per local millimetre

bc::UUID uid(std::uint8_t seed, std::uint8_t index) {
    bc::UUID value{};
    for (std::size_t i = 0; i < value.size(); ++i) value[i] = std::uint8_t(0x10 + i);
    value[0] = seed ? seed : 0x5f;
    value[1] = index;
    return value;
}

bc::Digest tagDigest(const char* tag) {
    std::vector<std::uint8_t> bytes(tag, tag + std::strlen(tag));
    bc::Digest digest{};
    assert(bc::Hash(bytes, 1024, digest));
    return digest;
}

bc::Frame frameAt(double x, double y, double z) {
    bc::Frame frame;
    frame.identifier = uid(40, 1);
    frame.revision = 1;
    frame.origin = {x, y, z};
    return frame;
}

tr::CurveRecord lineCurve(const bc::Frame& frame, std::uint8_t seed, std::uint8_t slot,
                          double x0, double y0, double x1, double y1) {
    bc::Value value;
    value.feature = uid(seed, slot);
    value.definition.domain = bc::Domain::Sketch2D;
    value.definition.frame = frame;
    value.definition.degree = 1;
    value.definition.controlPoints = {
        {uid(seed, std::uint8_t(slot + 100)), {x0, y0, 0}},
        {uid(seed, std::uint8_t(slot + 150)), {x1, y1, 0}}};
    value.definition.knots = {{0, 2}, {1, 2}};
    tr::CurveRecord record;
    record.value = value;
    assert(bc::Encode(value, record.canonicalBytes));
    return record;
}

tr::CurveRecord cubicCurve(const bc::Frame& frame, std::uint8_t seed, std::uint8_t slot,
                           const double x[4], const double y[4]) {
    bc::Value value;
    value.feature = uid(seed, slot);
    value.definition.domain = bc::Domain::Sketch2D;
    value.definition.frame = frame;
    value.definition.degree = 3;
    for (int pole = 0; pole < 4; ++pole)
        value.definition.controlPoints.push_back(
            {uid(seed, std::uint8_t(slot + 10 * (pole + 1))), {x[pole], y[pole], 0}});
    value.definition.knots = {{0, 4}, {1, 4}};
    tr::CurveRecord record;
    record.value = value;
    assert(bc::Encode(value, record.canonicalBytes));
    return record;
}

struct LoopSpec {
    std::vector<tr::CurveRecord> curves;
    tr::ContourRecord contour{};
};

// Explicit segments; closure is the caller's responsibility so open and
// gapped profiles can be authored directly.
LoopSpec segmentLoop(const bc::Frame& frame, std::uint8_t seed, std::uint32_t component,
                     std::uint32_t contour,
                     const std::vector<std::array<double, 4>>& segments) {
    LoopSpec spec;
    spec.contour = {component, contour, 0, std::uint32_t(segments.size())};
    for (std::size_t i = 0; i < segments.size(); ++i)
        spec.curves.push_back(lineCurve(frame, seed, std::uint8_t(2 * i + 1),
            segments[i][0], segments[i][1], segments[i][2], segments[i][3]));
    return spec;
}

// CCW polygon through the given vertices (order is authored winding).
LoopSpec polygonLoop(const bc::Frame& frame, std::uint8_t seed, std::uint32_t component,
                     std::uint32_t contour,
                     const std::vector<std::array<double, 2>>& points) {
    std::vector<std::array<double, 4>> segments;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto& a = points[i];
        const auto& b = points[(i + 1) % points.size()];
        segments.push_back({a[0], a[1], b[0], b[1]});
    }
    return segmentLoop(frame, seed, component, contour, segments);
}

LoopSpec squareLoop(const bc::Frame& frame, std::uint8_t seed, std::uint32_t component,
                    std::uint32_t contour, double cx, double cy, double half, bool ccw) {
    std::vector<std::array<double, 2>> points = {
        {cx - half, cy - half}, {cx + half, cy - half},
        {cx + half, cy + half}, {cx - half, cy + half}};
    if (!ccw) std::reverse(points.begin(), points.end());
    return polygonLoop(frame, seed, component, contour, points);
}

// Four-segment cubic near-circle, CCW, kappa = 4/3 (sqrt(2) - 1).
LoopSpec circleLoop(const bc::Frame& frame, std::uint8_t seed, std::uint32_t component,
                    std::uint32_t contour, double cx, double cy, double radius) {
    const double kappa = 0.5522847498307936;
    const double angle[5] = {0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI};
    LoopSpec spec;
    spec.contour = {component, contour, 0, 4};
    for (int segment = 0; segment < 4; ++segment) {
        const double c0 = std::cos(angle[segment]), s0 = std::sin(angle[segment]);
        const double c1 = std::cos(angle[segment + 1]), s1 = std::sin(angle[segment + 1]);
        const double x[4] = {cx + radius * c0, cx + radius * (c0 - kappa * s0),
                             cx + radius * (c1 + kappa * s1), cx + radius * c1};
        const double y[4] = {cy + radius * s0, cy + radius * (s0 + kappa * c0),
                             cy + radius * (s1 - kappa * c1), cy + radius * s1};
        spec.curves.push_back(cubicCurve(frame, seed, std::uint8_t(2 * segment + 1), x, y));
    }
    return spec;
}

struct Host {
    TopoDS_Shape solid;
    TopoDS_Face topFace;
    tr::HostReceipt receipt;
};

Host makeHost() {
    Host host;
    host.solid = BRepPrimAPI_MakeBox(gp_Pnt(-0.05, -0.05, -0.02),
                                     gp_Pnt(0.05, 0.05, 0.01)).Shape();
    for (TopExp_Explorer it(host.solid, TopAbs_FACE); it.More(); it.Next()) {
        TopoDS_Face face = TopoDS::Face(it.Current());
        BRepAdaptor_Surface surface(face);
        if (surface.GetType() != GeomAbs_Plane) continue;
        const gp_Pln plane = surface.Plane();
        if (std::abs(plane.Axis().Direction().Z()) > 0.9
            && std::abs(plane.Location().Z() - 0.01) < 1e-9) host.topFace = face;
    }
    assert(!host.topFace.IsNull());
    host.receipt.entity = uid(50, 1);
    host.receipt.faceFeature = uid(50, 2);
    host.receipt.frame = uid(50, 3);
    host.receipt.frameRevision = 1;
    std::string exact;
    assert(rb::ExactShapeBytes(host.solid, exact));
    const std::vector<std::uint8_t> bytes(exact.begin(), exact.end());
    assert(bc::Hash(bytes, 64 * tr::MaximumRecipeBytes, host.receipt.shapeDigest));
    return host;
}

retained_recipe::OwnerKey ownerKey() { return {uid(70, 1), uid(70, 2), uid(70, 3)}; }

retained_recipe::DependencyRead hostRead() {
    retained_recipe::DependencyRead read;
    read.locator = {ownerKey(), uid(60, 1), uid(60, 2)};
    read.geometry = tagDigest("host-geometry");
    read.recipe = tagDigest("host-recipe");
    read.placement = tagDigest("host-placement");
    read.material = tagDigest("host-material");
    read.groups = tagDigest("host-groups");
    return read;
}

tr::ReliefBuildRequest makeRequest(const Host& host, std::vector<LoopSpec> loops,
                                   tr::ReliefOperation operation, double depthMM,
                                   const bc::Frame& frame) {
    tr::ReliefBuildRequest request;
    request.identitySeed = uid(80, 1);
    request.owner = ownerKey();
    request.frame = frame;
    request.hostSolid = host.solid;
    request.hostFace = host.topFace;
    request.host = host.receipt;
    request.hostCaptured = hostRead();
    request.hostCurrent = hostRead();
    request.resourceSHA256 = tagDigest("e5-resource");
    request.operation = operation;
    request.depthMM = depthMM;
    request.metersPerLocalUnit = Scale;
    std::size_t cursor = 0;
    for (LoopSpec& loop : loops) {
        loop.contour.firstCurve = cursor;
        cursor += loop.curves.size();
        loop.contour.contour = request.contours.size();
        request.contours.push_back(loop.contour);
        for (auto& curve : loop.curves) request.curves.push_back(std::move(curve));
    }
    return request;
}

tr::ReliefBuildRequest makeRequest(const Host& host, std::vector<LoopSpec> loops,
                                   tr::ReliefOperation operation, double depthMM) {
    return makeRequest(host, std::move(loops), operation, depthMM, frameAt(0, 0, 0.01));
}

void assertNear(double actual, double expected, double relative) {
    assert(std::isfinite(actual));
    assert(std::abs(actual - expected) <= relative * std::max(1.0, std::abs(expected)));
}

std::vector<std::vector<std::array<double, 3>>> loopPoles(
    const std::vector<tr::CurveRecord>& curves, std::size_t first, std::size_t count) {
    std::vector<std::vector<std::array<double, 3>>> poles;
    for (std::size_t k = 0; k < count; ++k) {
        std::vector<std::array<double, 3>> curvePoles;
        for (const auto& point : curves[first + k].value.definition.controlPoints)
            curvePoles.push_back(point.local);
        poles.push_back(curvePoles);
    }
    return poles;
}

// OCCT face area (model units) of one loop built from its local-mm poles.
double occtLoopAreaModel(const std::vector<std::vector<std::array<double, 3>>>& poles,
                         const bc::Frame& frame) {
    const auto model = [&](const std::array<double, 3>& local) {
        return gp_Pnt(frame.origin[0] + Scale * local[0],
                      frame.origin[1] + Scale * local[1],
                      frame.origin[2] + Scale * local[2]);
    };
    BRepBuilderAPI_MakeWire wire;
    for (const auto& curvePoles : poles) {
        if (curvePoles.size() == 2) {
            BRepBuilderAPI_MakeEdge edge(model(curvePoles[0]), model(curvePoles[1]));
            assert(edge.IsDone());
            wire.Add(edge.Edge());
        } else {
            TColgp_Array1OfPnt array(1, 4);
            for (int pole = 0; pole < 4; ++pole)
                array.SetValue(pole + 1, model(curvePoles[pole]));
            Handle(Geom_BezierCurve) curve = new Geom_BezierCurve(array);
            BRepBuilderAPI_MakeEdge edge(curve, 0.0, 1.0);
            assert(edge.IsDone());
            wire.Add(edge.Edge());
        }
    }
    assert(wire.IsDone());
    BRepBuilderAPI_MakeFace face(
        gp_Pln(gp_Pnt(frame.origin[0], frame.origin[1], frame.origin[2]), gp::DZ()),
        wire.Wire(), Standard_True);
    assert(face.IsDone());
    GProp_GProps properties;
    BRepGProp::SurfaceProperties(face.Face(), properties);
    return properties.Mass();
}

void testE5ReliefEmbossFusesPrismAndRecipeBytesAreByteIdentical() {
    const Host host = makeHost();
    const bc::Frame frame = frameAt(0, 0, 0.01);
    const auto loops = [&] {
        return std::vector<LoopSpec>{squareLoop(frame, 11, 0, 0, -5, 0, 2, true),
                                     squareLoop(frame, 12, 0, 1, 5, 0, 2, true)};
    };
    const auto first = tr::BuildPlanarRelief(makeRequest(host, loops(), tr::ReliefOperation::Emboss, 2));
    const auto second = tr::BuildPlanarRelief(makeRequest(host, loops(), tr::ReliefOperation::Emboss, 2));
    assert(first.refusal == tr::BuildRefusal::None);
    assert(second.refusal == tr::BuildRefusal::None);
    assert(first.recipeBytes == second.recipeBytes);
    assert(first.recipe.feature == second.recipe.feature);
    assert(rb::SolidCount(first.solid) == 1);
    const double hostVolume = rb::Volume(host.solid);
    const double expected = hostVolume + 2 * (16.0 * 2.0) * 1e-9;
    assertNear(rb::Volume(first.solid), expected, 1e-9);
    tr::ReliefRecipe decoded;
    std::vector<std::uint8_t> again;
    assert(tr::Decode(first.recipeBytes, decoded));
    assert(tr::Encode(decoded, again) && again == first.recipeBytes);
    assert(decoded.loops.size() == 2
           && decoded.loops[0].role == tr::LoopRole::Outer
           && decoded.loops[1].role == tr::LoopRole::Outer);
}

void testE5ReliefDebossCutsExactSignedDepth() {
    const Host host = makeHost();
    const bc::Frame frame = frameAt(0, 0, 0.01);
    const auto built = tr::BuildPlanarRelief(makeRequest(host,
        {squareLoop(frame, 13, 0, 0, 0, 0, 2, true)}, tr::ReliefOperation::Deboss, -2));
    assert(built.refusal == tr::BuildRefusal::None);
    assert(rb::SolidCount(built.solid) == 1);
    const double expected = rb::Volume(host.solid) - 16.0 * 2.0 * 1e-9;
    assertNear(rb::Volume(built.solid), expected, 1e-9);
    assert(built.recipe.operation == tr::ReliefOperation::Deboss);
    assert(built.recipe.depthMM == -2);
}

void testE5ReliefHoleNestingByWindingBuildsCounteredPrism() {
    const Host host = makeHost();
    const bc::Frame frame = frameAt(0, 0, 0.01);
    auto outer = squareLoop(frame, 14, 0, 0, 0, 0, 4, true);
    auto hole = squareLoop(frame, 15, 0, 1, 0, 0, 1, false); // authored reversed (CW)
    const auto outerPoles = loopPoles(outer.curves, 0, 4);
    const auto holePoles = loopPoles(hole.curves, 0, 4);
    const double outerArea = tr::detail::SignedLoopAreaLocal(outerPoles);
    const double holeArea = tr::detail::SignedLoopAreaLocal(holePoles);
    assert(outerArea > 0 && holeArea < 0); // winding evidence before any OCCT call
    // Formula cross-check against the OCCT probe face of each single loop.
    assertNear(occtLoopAreaModel(outerPoles, frame), std::abs(outerArea) * Scale * Scale, 1e-6);
    assertNear(occtLoopAreaModel(holePoles, frame), std::abs(holeArea) * Scale * Scale, 1e-6);
    const auto built = tr::BuildPlanarRelief(makeRequest(host,
        {std::move(outer), std::move(hole)}, tr::ReliefOperation::Emboss, 2));
    assert(built.refusal == tr::BuildRefusal::None);
    assert(built.recipe.loops.size() == 2
           && built.recipe.loops[0].role == tr::LoopRole::Outer
           && built.recipe.loops[1].role == tr::LoopRole::Hole);
    const double expected = rb::Volume(host.solid) + (64.0 - 4.0) * 2.0 * 1e-9;
    assertNear(rb::Volume(built.solid), expected, 1e-9);
}

void testE5ReliefCubicCounterProfilesBuildExactly() {
    const Host host = makeHost();
    const bc::Frame frame = frameAt(0, 0, 0.01);
    auto outer = squareLoop(frame, 16, 0, 0, 0, 0, 4, true);
    auto hole = circleLoop(frame, 17, 0, 1, 0, 0, 2);
    const auto holePoles = loopPoles(hole.curves, 0, 4);
    const double circleArea = tr::detail::SignedLoopAreaLocal(holePoles);
    assert(circleArea > 0);
    assertNear(circleArea, M_PI * 4.0, 5e-3); // kappa near-circle vs pi r^2
    assertNear(occtLoopAreaModel(holePoles, frame), circleArea * Scale * Scale, 1e-6);
    const auto built = tr::BuildPlanarRelief(makeRequest(host,
        {std::move(outer), std::move(hole)}, tr::ReliefOperation::Emboss, 2));
    assert(built.refusal == tr::BuildRefusal::None);
    assert(built.recipe.loops.size() == 2
           && built.recipe.loops[0].role == tr::LoopRole::Outer
           && built.recipe.loops[1].role == tr::LoopRole::Hole);
}

void testE5ReliefRejectsOpenProfile() {
    const Host host = makeHost();
    const bc::Frame frame = frameAt(0, 0, 0.01);
    // Three of four sides: the chain never returns to its start.
    auto open = tr::BuildPlanarRelief(makeRequest(host,
        {segmentLoop(frame, 18, 0, 0, {{-2, -2, 2, -2}, {2, -2, 2, 2}, {2, 2, -2, 2}})},
        tr::ReliefOperation::Emboss, 2));
    assert(open.refusal == tr::BuildRefusal::OpenProfile);
    assert(open.solid.IsNull() && open.recipeBytes.empty());
    // A 1e-4 mm endpoint gap fails the same chained closure gate.
    auto gapped = tr::BuildPlanarRelief(makeRequest(host,
        {segmentLoop(frame, 19, 0, 0, {{-2, -2, 2, -2}, {2, -2, 2, 2}, {2, 2, -2, 2},
                                       {-2, 2, -2, -2 + 1e-4}})},
        tr::ReliefOperation::Emboss, 2));
    assert(gapped.refusal == tr::BuildRefusal::OpenProfile);
    assert(gapped.solid.IsNull() && gapped.recipeBytes.empty());
}

void testE5ReliefRejectsSelfIntersectingProfile() {
    const Host host = makeHost();
    const bc::Frame frame = frameAt(0, 0, 0.01);
    // Asymmetric bowtie: self-crossing but with a nonzero net signed area, so
    // the refusal must come from the intersection gates, not TinyTopology.
    auto built = tr::BuildPlanarRelief(makeRequest(host,
        {polygonLoop(frame, 20, 0, 0, {{0, 0}, {5, 4}, {4, 0}, {0, 3}})},
        tr::ReliefOperation::Emboss, 2));
    assert(built.refusal == tr::BuildRefusal::SelfIntersectingProfile);
    assert(built.solid.IsNull() && built.recipeBytes.empty());
}

void testE5ReliefRejectsNonPlanarHost() {
    const Host host = makeHost();
    const bc::Frame frame = frameAt(0, 0, 0.01);
    // Lateral cylinder face as host face.
    Host cylinder;
    cylinder.solid = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp::DZ()),
                                              0.02, 0.01).Shape();
    for (TopExp_Explorer it(cylinder.solid, TopAbs_FACE); it.More(); it.Next()) {
        TopoDS_Face face = TopoDS::Face(it.Current());
        if (BRepAdaptor_Surface(face).GetType() == GeomAbs_Cylinder)
            cylinder.topFace = face;
    }
    assert(!cylinder.topFace.IsNull());
    cylinder.receipt = host.receipt;
    std::string exact;
    assert(rb::ExactShapeBytes(cylinder.solid, exact));
    const std::vector<std::uint8_t> bytes(exact.begin(), exact.end());
    assert(bc::Hash(bytes, 64 * tr::MaximumRecipeBytes, cylinder.receipt.shapeDigest));
    auto curved = tr::BuildPlanarRelief(makeRequest(cylinder,
        {squareLoop(frame, 21, 0, 0, 0, 0, 2, true)}, tr::ReliefOperation::Deboss, -2));
    assert(curved.refusal == tr::BuildRefusal::NonPlanarHost);
    assert(curved.solid.IsNull() && curved.recipeBytes.empty());
    // Planar face whose plane does not contain the frame origin.
    const bc::Frame lifted = frameAt(0, 0, 0.02);
    auto offset = tr::BuildPlanarRelief(makeRequest(host,
        {squareLoop(lifted, 22, 0, 0, 0, 0, 2, true)}, tr::ReliefOperation::Deboss, -2,
        lifted));
    assert(offset.refusal == tr::BuildRefusal::HostPlaneMismatch);
    assert(offset.solid.IsNull() && offset.recipeBytes.empty());
}

void testE5ReliefRejectsDepthOutOfRange() {
    const Host host = makeHost();
    const bc::Frame frame = frameAt(0, 0, 0.01);
    const auto loops = [&] { return std::vector<LoopSpec>{squareLoop(frame, 23, 0, 0, 0, 0, 2, true)}; };
    const struct { tr::ReliefOperation operation; double depth; } cases[] = {
        {tr::ReliefOperation::Emboss, 0},
        {tr::ReliefOperation::Emboss, 0.009},
        {tr::ReliefOperation::Deboss, -0.009},
        {tr::ReliefOperation::Emboss, 10000.01},
        {tr::ReliefOperation::Deboss, -10000.01},
        {tr::ReliefOperation::Emboss, std::numeric_limits<double>::quiet_NaN()},
        {tr::ReliefOperation::Emboss, -2},   // sign mismatch
        {tr::ReliefOperation::Deboss, 2},    // sign mismatch
    };
    for (const auto& value : cases) {
        const auto built = tr::BuildPlanarRelief(
            makeRequest(host, loops(), value.operation, value.depth));
        assert(built.refusal == tr::BuildRefusal::DepthOutOfRange);
        assert(built.solid.IsNull() && built.recipeBytes.empty());
    }
}

void testE5ReliefFailsClosedAtE5Caps() {
    const Host host = makeHost();
    const bc::Frame frame = frameAt(0, 0, 0.01);
    const auto dummyCurve = [&] { return lineCurve(frame, 24, 1, 0, 0, 1, 0); };
    // 33 loops in one component trip the per-component cap before curve work.
    {
        auto request = makeRequest(host, {}, tr::ReliefOperation::Emboss, 2);
        request.contours.clear();
        for (std::uint32_t i = 0; i < 33; ++i)
            request.contours.push_back({0, i, i, 1});
        request.curves = {dummyCurve()};
        const auto built = tr::BuildPlanarRelief(request);
        assert(built.refusal == tr::BuildRefusal::ComponentContourLimit);
        assert(built.solid.IsNull() && built.recipeBytes.empty());
    }
    // 257 loops trip the contour cap before any curve decode.
    {
        auto request = makeRequest(host, {}, tr::ReliefOperation::Emboss, 2);
        request.contours.clear();
        for (std::uint32_t i = 0; i < 257; ++i)
            request.contours.push_back({i / 32, i % 32, i, 1});
        request.curves = {dummyCurve()};
        const auto built = tr::BuildPlanarRelief(request);
        assert(built.refusal == tr::BuildRefusal::ContourLimit);
        assert(built.solid.IsNull() && built.recipeBytes.empty());
    }
    // 129 curves trip the curve cap with a valid contour header.
    {
        auto request = makeRequest(host, {}, tr::ReliefOperation::Emboss, 2);
        request.contours = {{0, 0, 0, 129}};
        request.curves.clear();
        for (int i = 0; i < 129; ++i) request.curves.push_back(dummyCurve());
        const auto built = tr::BuildPlanarRelief(request);
        assert(built.refusal == tr::BuildRefusal::CurveLimit);
        assert(built.solid.IsNull() && built.recipeBytes.empty());
    }
}

void testE5ReliefRejectsInconsistentLoopsAndTamperedC1() {
    const Host host = makeHost();
    const bc::Frame frame = frameAt(0, 0, 0.01);
    // A partition that does not tile [0, curves.size()) exactly.
    {
        auto request = makeRequest(host, {squareLoop(frame, 25, 0, 0, 0, 0, 2, true)},
                                   tr::ReliefOperation::Emboss, 2);
        request.contours[0].curveCount = 3;
        const auto built = tr::BuildPlanarRelief(request);
        assert(built.refusal == tr::BuildRefusal::InconsistentLoops);
        assert(built.solid.IsNull() && built.recipeBytes.empty());
    }
    // One flipped canonical byte breaks the C1 SHA-256 trailer.
    {
        auto request = makeRequest(host, {squareLoop(frame, 26, 0, 0, 0, 0, 2, true)},
                                   tr::ReliefOperation::Emboss, 2);
        request.curves[2].canonicalBytes[request.curves[2].canonicalBytes.size() / 2] ^= 0x01;
        const auto built = tr::BuildPlanarRelief(request);
        assert(built.refusal == tr::BuildRefusal::C1Refused);
        assert(built.solid.IsNull() && built.recipeBytes.empty());
    }
    // A curve authored against frame revision 2 under a revision-1 request.
    {
        bc::Frame other = frame;
        other.revision = 2;
        auto request = makeRequest(host, {squareLoop(other, 27, 0, 0, 0, 0, 2, true)},
                                   tr::ReliefOperation::Emboss, 2);
        const auto built = tr::BuildPlanarRelief(request);
        assert(built.refusal == tr::BuildRefusal::FrameMismatch);
        assert(built.solid.IsNull() && built.recipeBytes.empty());
    }
}

void testE5ReliefBooleanFailureFailsClosedWithoutRecipe() {
    const Host host = makeHost();
    // Frame origin outside the host footprint: the cut prism never meets the
    // host interior, so the retained seam refuses and nothing is published.
    const bc::Frame aside = frameAt(0.06, 0, 0.01);
    auto outside = tr::BuildPlanarRelief(makeRequest(host,
        {squareLoop(aside, 28, 0, 0, 60, 0, 2, true)}, tr::ReliefOperation::Deboss, -2,
        aside));
    assert(outside.refusal == tr::BuildRefusal::BooleanFailure);
    assert(outside.booleanDetail == rb::Refusal::NoInteriorOverlap
           || outside.booleanDetail == rb::Refusal::EmptyResult);
    assert(outside.solid.IsNull() && outside.recipeBytes.empty());
    // A drifted current host read is staleness, detected before any boolean.
    const bc::Frame frame = frameAt(0, 0, 0.01);
    auto request = makeRequest(host, {squareLoop(frame, 29, 0, 0, 0, 0, 2, true)},
                               tr::ReliefOperation::Deboss, -2);
    request.hostCurrent.geometry[0] ^= 0xff;
    const auto stale = tr::BuildPlanarRelief(request);
    assert(stale.refusal == tr::BuildRefusal::StaleHost);
    assert(stale.solid.IsNull() && stale.recipeBytes.empty());
}

void testE5ReliefRecipeCodecRoundTripAndCorruptionRefuses() {
    tr::ReliefRecipe recipe;
    recipe.owner = ownerKey();
    recipe.host.entity = uid(50, 1);
    recipe.host.faceFeature = uid(50, 2);
    recipe.host.frame = uid(50, 3);
    recipe.host.frameRevision = 1;
    recipe.host.shapeDigest = tagDigest("host-shape");
    recipe.operation = tr::ReliefOperation::Emboss;
    recipe.depthMM = 2;
    recipe.metersPerLocalUnit = Scale;
    recipe.resourceSHA256 = tagDigest("e5-resource");
    std::vector<bc::Digest> digests;
    for (int i = 0; i < 4; ++i) {
        bc::Digest digest = tagDigest(i % 2 == 0 ? "curve-even" : "curve-odd");
        digest[0] = std::uint8_t(i + 1);
        digests.push_back(digest);
        recipe.curves.push_back({uid(90, std::uint8_t(i + 1)), digest});
    }
    recipe.loops.push_back({0, 0, 0, 4, tr::LoopRole::Outer});
    recipe.feature = tr::DeriveReliefFeature(uid(80, 1), recipe.host,
        recipe.resourceSHA256, recipe.operation, recipe.depthMM,
        recipe.metersPerLocalUnit, digests);
    assert(tr::Validate(recipe) == tr::RecipeRefusal::None);
    // Identity is stable for byte-identical inputs and drifts with any field.
    const auto same = tr::DeriveReliefFeature(uid(80, 1), recipe.host,
        recipe.resourceSHA256, recipe.operation, recipe.depthMM,
        recipe.metersPerLocalUnit, digests);
    const auto deeper = tr::DeriveReliefFeature(uid(80, 1), recipe.host,
        recipe.resourceSHA256, recipe.operation, 3,
        recipe.metersPerLocalUnit, digests);
    assert(same == recipe.feature && deeper != recipe.feature);
    std::vector<std::uint8_t> bytes;
    assert(tr::Encode(recipe, bytes));
    tr::ReliefRecipe decoded;
    std::vector<std::uint8_t> again;
    assert(tr::Decode(bytes, decoded));
    assert(tr::Encode(decoded, again) && again == bytes);
    assert(decoded.feature == recipe.feature && decoded.depthMM == recipe.depthMM
           && decoded.loops.size() == 1 && decoded.loops[0].role == tr::LoopRole::Outer);
    auto corrupted = bytes;
    corrupted[corrupted.size() / 2] ^= 0x01;
    tr::ReliefRecipe cleared;
    assert(!tr::Decode(corrupted, cleared));
    assert(!bc::Nonzero(cleared.feature) && cleared.curves.empty() && cleared.loops.empty());
    auto truncated = bytes;
    truncated.pop_back();
    assert(!tr::Decode(truncated, cleared));
}
}

int main() {
    testE5ReliefEmbossFusesPrismAndRecipeBytesAreByteIdentical();
    testE5ReliefDebossCutsExactSignedDepth();
    testE5ReliefHoleNestingByWindingBuildsCounteredPrism();
    testE5ReliefCubicCounterProfilesBuildExactly();
    testE5ReliefRejectsOpenProfile();
    testE5ReliefRejectsSelfIntersectingProfile();
    testE5ReliefRejectsNonPlanarHost();
    testE5ReliefRejectsDepthOutOfRange();
    testE5ReliefFailsClosedAtE5Caps();
    testE5ReliefRejectsInconsistentLoopsAndTamperedC1();
    testE5ReliefBooleanFailureFailsClosedWithoutRecipe();
    testE5ReliefRecipeCodecRoundTripAndCorruptionRefuses();
    std::cout << "TextReliefBuildTests: PASS\n";
}
