#include "VariableRadiusFilletBuild.hxx"
#include "RetainedFilletProgram.hxx"
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

using namespace core3d;

namespace {
retained_recipe::UUID uuid(std::uint8_t value) {
    retained_recipe::UUID result{}; result.back() = value; return result;
}
retained_recipe::Digest digest(std::uint8_t value) {
    retained_recipe::Digest result{}; result.back() = value; return result;
}
variable_radius_fillet::Definition definition(double metersPerUnit) {
    using namespace variable_radius_fillet;
    const double scale = 0.001 / metersPerUnit;
    Definition value; value.metersPerLocalUnit = metersPerUnit;
    value.stations = {{{uuid(1), 0, 2 * scale}, {uuid(2), 1, 5 * scale}}};
    OrientedEdgeAnchor edge; edge.identifier = uuid(3);
    edge.pointLocal = {{0, 0, 20 * scale}}; edge.tangent = {{0, 0, 1}};
    edge.normalA = {{-1, 0, 0}}; edge.normalB = {{0, -1, 0}};
    edge.startLocal = {{0, 0, 0}}; edge.endLocal = {{0, 0, 40 * scale}};
    edge.selectorProof = digest(4); value.edges.push_back(edge); return value;
}
std::vector<std::uint8_t> hex(const char* value) {
    std::vector<std::uint8_t> output;
    for (std::size_t index = 0; value[index]; index += 2) {
        unsigned byte = 0; std::sscanf(value + index, "%2x", &byte);
        output.push_back(std::uint8_t(byte));
    }
    return output;
}
}

void testB3LinearLawCodecRoundTripAndLegacyScalarBytesUnchanged() {
    using namespace variable_radius_fillet;
    for (double unit : {0.001, 1.0}) {
        const Definition input = definition(unit); std::vector<std::uint8_t> bytes;
        assert(Encode(input, bytes)); Definition decoded; assert(Decode(bytes, decoded));
        assert(decoded == input); std::vector<std::uint8_t> second; assert(Encode(decoded, second));
        assert(bytes == second);
    }
    retained_fillet::Program legacy; legacy.nextFilletStepID = 2; legacy.nextFilletEdgeID = 2;
    retained_fillet::Step step; step.stepIdentifier = 1; step.radiusLocal = 2;
    retained_fillet::EdgeAnchor anchor; anchor.identifier = 1; anchor.axis = {{0, 0, 1}};
    step.anchors.push_back(anchor); legacy.filletSteps.push_back(step);
    std::vector<std::uint8_t> legacyBytes; assert(retained_fillet::Encode(legacy, 1, legacyBytes));
    const auto golden = hex("01000000000000000200000000000000020000000000000001000000000000000000000000000040010000000000000001000000000000000100000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000f03f0000000000000000");
    assert(legacyBytes == golden);
}

void testB3RejectsNonFiniteNonPositiveReversedAndClosedLoopPayloads() {
    using namespace variable_radius_fillet;
    Refusal refusal; Definition value = definition(0.001);
    value.stations[0].radiusLocal = 0; assert(!Validate(value, refusal)); assert(refusal == Refusal::InvalidRadius);
    value = definition(0.001); value.stations[1].radiusLocal = std::numeric_limits<double>::infinity();
    assert(!Validate(value, refusal)); assert(refusal == Refusal::InvalidRadius);
    value = definition(0.001); value.edges[0].orientation = Orientation::Reversed;
    assert(!Validate(value, refusal)); assert(refusal == Refusal::OrientationDrift);
    value = definition(0.001); value.edges[0].curve = CurveKind(2);
    assert(!Validate(value, refusal)); assert(refusal == Refusal::ClosedLoop);
}

void testB3KernelBuildPinsThreeMeasuredSectionsAndConsumesOneOpenContour() {
    using namespace variable_radius_fillet;
    for (double unit : {0.001, 1.0}) {
        const double scale = 0.001 / unit;
        const TopoDS_Shape box = BRepPrimAPI_MakeBox(30 * scale, 30 * scale, 40 * scale).Shape();
        const std::atomic_bool cancelled{false};
        const BuildResult result = BuildDeterministically(box, definition(unit), cancelled);
        if (!result.built()) std::cerr << "B3 build refusal at unit " << unit
            << ": " << Reason(result.refusal) << "\n";
        assert(result.built()); assert(result.evidence.consumedEdges == std::vector<UUID>{uuid(3)});
        assert(result.evidence.sections.size() == 3);
        assert(result.evidence.sections[0].parameter == 0.2);
        assert(result.evidence.sections[1].parameter == 0.5);
        assert(result.evidence.sections[2].parameter == 0.8);
        assert(result.evidence.sections[0].measuredRadiusLocal > 0);
        assert(result.evidence.sections[0].measuredRadiusLocal
            < result.evidence.sections[1].measuredRadiusLocal);
        assert(result.evidence.sections[1].measuredRadiusLocal
            < result.evidence.sections[2].measuredRadiusLocal);
    }
}

void testB3KernelRejectsClosedLoopAndInsufficientWholeLawClearance() {
    using namespace variable_radius_fillet;
    const std::atomic_bool cancelled{false}; Definition value = definition(0.001);
    // K19: a 2 -> 5 mm law needs faces wider than 5 mm beside a box edge, so
    // the too-tight box is 4 x 4 (it was 6 x 6 under the old half-width rule).
    const TopoDS_Shape tight = BRepPrimAPI_MakeBox(4, 4, 40).Shape();
    const BuildResult clearance = Build(tight, value, cancelled);
    assert(!clearance.built()); assert(clearance.refusal == Refusal::Clearance);
    value.edges[0].curve = CurveKind(2); const BuildResult closed = Build(tight, value, cancelled);
    assert(!closed.built()); assert(closed.refusal == Refusal::ClosedLoop);
}

void testB3ReplayDeterminismPreservesOrientationOrRefusesSourceDrift() {
    using namespace variable_radius_fillet;
    const std::atomic_bool cancelled{false}; const Definition value = definition(0.001);
    const TopoDS_Shape original = BRepPrimAPI_MakeBox(30, 30, 40).Shape();
    assert(BuildDeterministically(original, value, cancelled).built());
    const TopoDS_Shape edited = BRepPrimAPI_MakeBox(30, 30, 50).Shape();
    const BuildResult drift = BuildDeterministically(edited, value, cancelled);
    assert(!drift.built());
    assert(drift.refusal == Refusal::OrientationDrift || drift.refusal == Refusal::AnchorMissing);
}

// K19 (desk lamp rows 127e2 / S40, toy car 7a): Taper 0.3 -> 1 mm on the
// 16 x 24 x 2 lid. Before the repair every law below is refused with
// Refusal::Clearance: the 2 mm side is "too narrow" for a 1 mm radius under
// the half-width rule, the 0.707 mm Flat Bevel face likewise, and the end
// face clipped by the bevel has five edges, not four.
namespace {
variable_radius_fillet::Definition lidLaw(
    const std::array<double, 3>& start, const std::array<double, 3>& end,
    const std::array<double, 3>& tangent, const std::array<double, 3>& normalA,
    const std::array<double, 3>& normalB, double startRadius, double endRadius) {
    using namespace variable_radius_fillet;
    Definition value; value.metersPerLocalUnit = 0.001;
    value.stations = {{{uuid(1), 0, startRadius}, {uuid(2), 1, endRadius}}};
    OrientedEdgeAnchor edge; edge.identifier = uuid(3);
    edge.pointLocal = {{(start[0] + end[0]) / 2, (start[1] + end[1]) / 2,
                        (start[2] + end[2]) / 2}};
    edge.tangent = tangent; edge.normalA = normalA; edge.normalB = normalB;
    edge.startLocal = start; edge.endLocal = end;
    edge.selectorProof = digest(4); value.edges.push_back(edge); return value;
}
TopoDS_Shape flatBevelledLid() {
    // Lid 16 x 24 x 2 with a 0.5 mm Flat Bevel on the top long edge x = 0.
    const TopoDS_Shape lid = BRepPrimAPI_MakeBox(16, 24, 2).Shape();
    TopTools_IndexedMapOfShape edges; TopExp::MapShapes(lid, TopAbs_EDGE, edges);
    BRepFilletAPI_MakeChamfer bevel(lid); unsigned added = 0;
    for (int index = 1; index <= edges.Extent(); ++index) {
        const TopoDS_Edge edge = TopoDS::Edge(edges(index));
        TopoDS_Vertex first, last; TopExp::Vertices(edge, first, last);
        const gp_Pnt a = BRep_Tool::Pnt(first), b = BRep_Tool::Pnt(last);
        if (std::abs(a.X()) < 1e-9 && std::abs(b.X()) < 1e-9
            && std::abs(a.Z() - 2) < 1e-9 && std::abs(b.Z() - 2) < 1e-9) {
            bevel.Add(0.5, edge); ++added;
        }
    }
    assert(added == 1); bevel.Build(); assert(bevel.IsDone());
    return bevel.Shape();
}
void requireBuilt(const char* name, const variable_radius_fillet::BuildResult& result) {
    if (!result.built()) std::cerr << "K19 " << name << " refused: "
        << variable_radius_fillet::Reason(result.refusal) << "\n";
    assert(result.built());
    assert(result.evidence.sections.size() == 3);
    assert(result.evidence.sections[0].measuredRadiusLocal
        < result.evidence.sections[1].measuredRadiusLocal);
    assert(result.evidence.sections[1].measuredRadiusLocal
        < result.evidence.sections[2].measuredRadiusLocal);
}
}

void testK19TaperAdmitsThinLidAndFlatBevelledLidEdgesAndStillRefusesTooLarge() {
    using namespace variable_radius_fillet;
    const std::atomic_bool cancelled{false};
    const double h = std::sqrt(0.5);

    // 1. Plain lid, top long edge x = 0, z = 2: side face 2 mm high.
    const TopoDS_Shape lid = BRepPrimAPI_MakeBox(16, 24, 2).Shape();
    const Definition topEdge = lidLaw({{0, 0, 2}}, {{0, 24, 2}}, {{0, 1, 0}},
                                      {{-1, 0, 0}}, {{0, 0, 1}}, 0.3, 1.0);
    requireBuilt("plain lid top edge", BuildDeterministically(lid, topEdge, cancelled));

    // 2. Flat-bevelled lid, the edge between the top face and the bevel face
    //    (x = 0.5, z = 2): the bevel face is 0.707 mm wide and the faces turn
    //    45 degrees, so a 1 mm radius takes 0.414 mm of it.
    const TopoDS_Shape bevelled = flatBevelledLid();
    const Definition bevelEdge = lidLaw({{0.5, 0, 2}}, {{0.5, 24, 2}}, {{0, 1, 0}},
                                        {{-h, 0, h}}, {{0, 0, 1}}, 0.3, 1.0);
    requireBuilt("bevelled lid bevel edge",
                 BuildDeterministically(bevelled, bevelEdge, cancelled));

    // 3. Flat-bevelled lid, bottom edge of the end face y = 0 that the bevel
    //    clipped to five edges (nearest other edge 1.5 mm away).
    const Definition endEdge = lidLaw({{0, 0, 0}}, {{16, 0, 0}}, {{1, 0, 0}},
                                      {{0, -1, 0}}, {{0, 0, -1}}, 0.3, 1.0);
    requireBuilt("bevelled lid five-edge end face",
                 BuildDeterministically(bevelled, endEdge, cancelled));

    // 4. A radius the 2 mm side truly cannot take is still a clean clearance
    //    refusal, and the range reader names the 2 mm bound for the message.
    const Definition tooLarge = lidLaw({{0, 0, 2}}, {{0, 24, 2}}, {{0, 1, 0}},
                                       {{-1, 0, 0}}, {{0, 0, 1}}, 0.3, 2.5);
    const BuildResult refused = Build(lid, tooLarge, cancelled);
    assert(!refused.built()); assert(refused.refusal == Refusal::Clearance);
    assert(refused.evidence.sections.empty());
    double limit = 0;
    assert(RadiusLimitLocal(lid, topEdge.edges[0], 0.001, limit));
    assert(limit > 1.99 && limit <= 2.0);
}

int main() {
    testB3LinearLawCodecRoundTripAndLegacyScalarBytesUnchanged();
    testB3RejectsNonFiniteNonPositiveReversedAndClosedLoopPayloads();
    testB3KernelBuildPinsThreeMeasuredSectionsAndConsumesOneOpenContour();
    testB3KernelRejectsClosedLoopAndInsufficientWholeLawClearance();
    testB3ReplayDeterminismPreservesOrientationOrRefusesSourceDrift();
    testK19TaperAdmitsThinLidAndFlatBevelledLidEdgesAndStillRefusesTooLarge();
    std::cout << "VariableRadiusFilletTests: PASS\n";
}
