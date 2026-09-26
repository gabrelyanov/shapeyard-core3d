// Stage 2 multi-station variable-radius fillet law tests. These extend the
// stage-1 contract in VariableRadiusFilletTests.cpp without touching it; the
// final test below re-runs the stage-1 admission and byte contract as a
// guard that nothing in stage 2 weakened it.
#include "VariableRadiusFilletBuild.hxx"
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
variable_radius_fillet::OrientedEdgeAnchor edgeAnchor(double scale) {
    variable_radius_fillet::OrientedEdgeAnchor edge; edge.identifier = uuid(20);
    edge.pointLocal = {{0, 0, 20 * scale}}; edge.tangent = {{0, 0, 1}};
    edge.normalA = {{-1, 0, 0}}; edge.normalB = {{0, -1, 0}};
    edge.startLocal = {{0, 0, 0}}; edge.endLocal = {{0, 0, 40 * scale}};
    edge.selectorProof = digest(21); return edge;
}
// Five stations at t = 0, 1/4, 1/2, 3/4, 1 with radii 2, 3, 4, 6, 8 mm: a
// piecewise law with varying segment slopes so every interior station pin is
// exercised at both unit scales.
variable_radius_fillet::MultiStationDefinition definition(double metersPerUnit) {
    using namespace variable_radius_fillet;
    const double scale = 0.001 / metersPerUnit;
    MultiStationDefinition value; value.metersPerLocalUnit = metersPerUnit;
    const double radii[5] = {2, 3, 4, 6, 8};
    for (std::size_t index = 0; index < 5; ++index)
        value.stations.push_back({uuid(std::uint8_t(10 + index)),
                                  double(index) / 4, radii[index] * scale});
    value.edges.push_back(edgeAnchor(scale)); return value;
}
// Radii 2, 5, 3, 6, 4 mm: an up-down-up-down law for the direction-change
// proof. This OCCT build loses the section arc at sharp radius minima when
// the model unit is the metre (see NOTES.md), so this profile is exercised
// at the millimetre unit only.
variable_radius_fillet::MultiStationDefinition valleyDefinition(double metersPerUnit) {
    auto value = definition(metersPerUnit);
    const double scale = 0.001 / metersPerUnit;
    const double radii[5] = {2, 5, 3, 6, 4};
    for (std::size_t index = 0; index < 5; ++index)
        value.stations[index].radiusLocal = radii[index] * scale;
    return value;
}
variable_radius_fillet::Definition linearDefinition(double metersPerUnit) {
    using namespace variable_radius_fillet;
    const double scale = 0.001 / metersPerUnit;
    Definition value; value.metersPerLocalUnit = metersPerUnit;
    value.stations = {{{uuid(1), 0, 2 * scale}, {uuid(2), 1, 5 * scale}}};
    value.edges.push_back(edgeAnchor(scale));
    value.edges[0].identifier = uuid(3); value.edges[0].selectorProof = digest(4);
    return value;
}
}

void testB3Stage2MultiStationCodecRoundTripInMMAndM() {
    using namespace variable_radius_fillet;
    for (double unit : {0.001, 1.0}) {
        const MultiStationDefinition input = definition(unit);
        std::vector<std::uint8_t> bytes;
        assert(Encode(input, bytes)); MultiStationDefinition decoded;
        assert(Decode(bytes, decoded)); assert(decoded == input);
        std::vector<std::uint8_t> second; assert(Encode(decoded, second));
        assert(bytes == second);
        // The codec versions are fail-closed in both directions: the stage-1
        // decoder refuses v2 bytes and the stage-2 decoder refuses v1 bytes.
        Definition stage1; assert(!Decode(bytes, stage1));
        std::vector<std::uint8_t> stage1Bytes; assert(Encode(linearDefinition(unit), stage1Bytes));
        MultiStationDefinition rejected; assert(!Decode(stage1Bytes, rejected));
    }
}

void testB3Stage2RejectsTooFewTooManyNonMonotoneAndDuplicateStations() {
    using namespace variable_radius_fillet;
    Refusal refusal;
    MultiStationDefinition value = definition(0.001);
    value.stations.resize(2);
    assert(!Validate(value, refusal)); assert(refusal == Refusal::InvalidStationIdentity);
    value = definition(0.001);
    for (std::size_t index = 5; index < 17; ++index)
        value.stations.push_back({uuid(std::uint8_t(30 + index)),
                                  double(index) / 16, (2 + double(index) / 16) * 1});
    assert(value.stations.size() == 17);
    assert(!Validate(value, refusal)); assert(refusal == Refusal::Budget);
    value = definition(0.001); value.stations[2].parameter = value.stations[1].parameter;
    assert(!Validate(value, refusal)); assert(refusal == Refusal::InvalidStationIdentity);
    value = definition(0.001); value.stations[2].parameter = 0.125;
    assert(!Validate(value, refusal)); assert(refusal == Refusal::InvalidStationIdentity);
    value = definition(0.001); value.stations[3].identifier = value.stations[2].identifier;
    assert(!Validate(value, refusal)); assert(refusal == Refusal::InvalidStationIdentity);
    value = definition(0.001); value.stations[0].parameter = 1e-300;
    assert(!Validate(value, refusal)); assert(refusal == Refusal::InvalidStationIdentity);
    value = definition(0.001); value.stations[4].parameter = 0.999999;
    assert(!Validate(value, refusal)); assert(refusal == Refusal::InvalidStationIdentity);
    value = definition(0.001);
    value.stations[2].parameter = std::numeric_limits<double>::quiet_NaN();
    assert(!Validate(value, refusal)); assert(refusal == Refusal::InvalidStationIdentity);
    value = definition(0.001); value.stations[1].radiusLocal = 0;
    assert(!Validate(value, refusal)); assert(refusal == Refusal::InvalidRadius);
    value = definition(0.001);
    value.stations[3].radiusLocal = std::numeric_limits<double>::infinity();
    assert(!Validate(value, refusal)); assert(refusal == Refusal::InvalidRadius);
}

void testB3Stage2KernelBuildPinsInteriorStationSectionsAndKernelLaw() {
    using namespace variable_radius_fillet;
    for (double unit : {0.001, 1.0}) {
        const double scale = 0.001 / unit;
        const TopoDS_Shape box = BRepPrimAPI_MakeBox(30 * scale, 30 * scale, 40 * scale).Shape();
        const std::atomic_bool cancelled{false};
        const BuildResult result = BuildMultiStationDeterministically(box, definition(unit), cancelled);
        if (!result.built()) std::cerr << "B3 stage-2 build refusal at unit " << unit
            << ": " << Reason(result.refusal) << "\n";
        assert(result.built());
        assert(result.evidence.consumedEdges == std::vector<UUID>{uuid(20)});
        // One independently measured section per interior station.
        assert(result.evidence.sections.size() == 3);
        assert(result.evidence.sections[0].parameter == 0.25);
        assert(result.evidence.sections[1].parameter == 0.5);
        assert(result.evidence.sections[2].parameter == 0.75);
        assert(result.evidence.sections[0].expectedRadiusLocal == 3 * scale);
        assert(result.evidence.sections[1].expectedRadiusLocal == 4 * scale);
        assert(result.evidence.sections[2].expectedRadiusLocal == 6 * scale);
        for (const auto& section : result.evidence.sections)
            assert(section.measuredRadiusLocal > 0);
        assert(result.evidence.sections[0].measuredRadiusLocal
            < result.evidence.sections[1].measuredRadiusLocal);
        assert(result.evidence.sections[1].measuredRadiusLocal
            < result.evidence.sections[2].measuredRadiusLocal);
    }
}

void testB3Stage2KernelFollowsRadiusDirectionChangesAcrossStations() {
    using namespace variable_radius_fillet;
    const double scale = 1; // millimetre unit; see valleyDefinition note
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(30, 30, 40).Shape();
    const std::atomic_bool cancelled{false};
    const BuildResult result = BuildMultiStationDeterministically(
        box, valleyDefinition(0.001), cancelled);
    if (!result.built()) std::cerr << "B3 stage-2 direction refusal: "
        << Reason(result.refusal) << "\n";
    assert(result.built());
    assert(result.evidence.consumedEdges == std::vector<UUID>{uuid(20)});
    assert(result.evidence.sections.size() == 3);
    assert(result.evidence.sections[0].expectedRadiusLocal == 5 * scale);
    assert(result.evidence.sections[1].expectedRadiusLocal == 3 * scale);
    assert(result.evidence.sections[2].expectedRadiusLocal == 6 * scale);
    // The authored law goes down then up across the interior stations; the
    // independently measured sections must follow each direction change.
    assert(result.evidence.sections[0].measuredRadiusLocal
        > result.evidence.sections[1].measuredRadiusLocal);
    assert(result.evidence.sections[1].measuredRadiusLocal
        < result.evidence.sections[2].measuredRadiusLocal);
}

void testB3Stage2KernelRejectsInsufficientWholeLawClearanceAndClosedLoop() {
    using namespace variable_radius_fillet;
    const std::atomic_bool cancelled{false};
    MultiStationDefinition value = definition(0.001);
    const TopoDS_Shape tight = BRepPrimAPI_MakeBox(6, 6, 40).Shape();
    const BuildResult clearance = BuildMultiStation(tight, value, cancelled);
    assert(!clearance.built()); assert(clearance.refusal == Refusal::Clearance);
    value.edges[0].curve = CurveKind(2);
    const BuildResult closed = BuildMultiStation(tight, value, cancelled);
    assert(!closed.built()); assert(closed.refusal == Refusal::ClosedLoop);
}

void testB3Stage2ReplayDeterminismPreservesOrientationOrRefusesSourceDrift() {
    using namespace variable_radius_fillet;
    const std::atomic_bool cancelled{false};
    const MultiStationDefinition value = definition(0.001);
    const TopoDS_Shape original = BRepPrimAPI_MakeBox(30, 30, 40).Shape();
    assert(BuildMultiStationDeterministically(original, value, cancelled).built());
    const TopoDS_Shape edited = BRepPrimAPI_MakeBox(30, 30, 50).Shape();
    const BuildResult drift = BuildMultiStationDeterministically(edited, value, cancelled);
    assert(!drift.built());
    assert(drift.refusal == Refusal::OrientationDrift || drift.refusal == Refusal::AnchorMissing);
}

void testB3Stage2Stage1LinearLawAdmissionAndCodecBytesUnchanged() {
    using namespace variable_radius_fillet;
    // Admission constants and the stage-1 codec are untouched by stage 2.
    assert(CodecVersion == 1);
    assert(MultiStationCodecVersion == 2);
    assert(MaximumEdges == 16);
    assert(MaximumPayloadBytes == 4096);
    assert(MinimumStations == 3);
    assert(MaximumStations == 16);
    for (double unit : {0.001, 1.0}) {
        const Definition linear = linearDefinition(unit);
        std::vector<std::uint8_t> bytes; assert(Encode(linear, bytes));
        // SYVF/1 fixture layout: 16 header + 2*32 stations + 4 count + 196 edge.
        assert(bytes.size() == 280);
        Definition decoded; assert(Decode(bytes, decoded)); assert(decoded == linear);
        const double scale = 0.001 / unit;
        const TopoDS_Shape box = BRepPrimAPI_MakeBox(30 * scale, 30 * scale, 40 * scale).Shape();
        const std::atomic_bool cancelled{false};
        const BuildResult result = BuildDeterministically(box, linear, cancelled);
        if (!result.built()) std::cerr << "B3 stage-1 guard refusal at unit " << unit
            << ": " << Reason(result.refusal) << "\n";
        assert(result.built()); assert(result.evidence.sections.size() == 3);
    }
}

int main() {
    testB3Stage2MultiStationCodecRoundTripInMMAndM();
    testB3Stage2RejectsTooFewTooManyNonMonotoneAndDuplicateStations();
    testB3Stage2KernelBuildPinsInteriorStationSectionsAndKernelLaw();
    testB3Stage2KernelFollowsRadiusDirectionChangesAcrossStations();
    testB3Stage2KernelRejectsInsufficientWholeLawClearanceAndClosedLoop();
    testB3Stage2ReplayDeterminismPreservesOrientationOrRefusesSourceDrift();
    testB3Stage2Stage1LinearLawAdmissionAndCodecBytesUnchanged();
    std::cout << "VariableRadiusFilletMultiStationTests: PASS\n";
}
