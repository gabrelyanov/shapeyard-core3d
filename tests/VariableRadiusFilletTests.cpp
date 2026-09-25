#include "VariableRadiusFilletBuild.hxx"
#include "RetainedFilletProgram.hxx"
#include <BRepPrimAPI_MakeBox.hxx>
#include <cassert>
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
    const TopoDS_Shape tight = BRepPrimAPI_MakeBox(6, 6, 40).Shape();
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

int main() {
    testB3LinearLawCodecRoundTripAndLegacyScalarBytesUnchanged();
    testB3RejectsNonFiniteNonPositiveReversedAndClosedLoopPayloads();
    testB3KernelBuildPinsThreeMeasuredSectionsAndConsumesOneOpenContour();
    testB3KernelRejectsClosedLoopAndInsufficientWholeLawClearance();
    testB3ReplayDeterminismPreservesOrientationOrRefusesSourceDrift();
    std::cout << "VariableRadiusFilletTests: PASS\n";
}
