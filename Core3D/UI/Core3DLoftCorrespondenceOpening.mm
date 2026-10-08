#import "Core3DLoftCorrespondenceOpening.h"

#include "../OCCTKit/LoftCorrespondenceProof.hxx"

#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Vec2d.hxx>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numeric>
#include <vector>

@implementation Core3DLoftCorrespondenceOpening
@end

#if DEBUG
namespace {
namespace lc = core3d::loft_correspondence;
namespace proof = core3d::loft_correspondence::proof;
namespace tb = core3d::retained_topology_budget;

constexpr std::uint64_t EvidenceVersion = 0x53594c4331000001ULL; // SYLC1/v1

std::uint64_t Bits(double value) noexcept {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

lc::Definition MakeDefinition(double unit, const std::vector<int>& phases,
                              bool implicit = false, bool nearClearance = false) {
    const double raw = 0.001 / unit;
    lc::Definition definition;
    definition.implicitEqualIndex = implicit;
    definition.base.loftIdentifier = 1;
    definition.base.correspondence = {2, 3, 4, 5};
    definition.base.dimensionMetersPerUnit = unit;
    for (std::size_t index = 0; index < phases.size(); ++index) {
        lc::ElementID stationID = lc::ElementID(100 + index * 100);
        if (phases.size() == 3 && index == 1) stationID = 150;
        core3d::rectangular_loft::Station station;
        station.identifier = stationID;
        station.cornerIdentifiers = {stationID + 1, stationID + 2,
                                     stationID + 3, stationID + 4};
        station.correspondence = definition.base.correspondence;
        station.z = (phases.size() == 3 ? 60.0 : 120.0) * double(index) * raw;
        station.centerX = (index == 0 ? 0.0 : index + 1 == phases.size() ? 10.0 : 4.0) * raw;
        station.centerY = (index == 0 ? 0.0 : index + 1 == phases.size() ? 5.0 : -3.0) * raw;
        if (nearClearance) {
            // Keep each endpoint old-valid in both document unit systems. In
            // metre documents OCCT's unchanged 32*Confusion native floor is
            // physically larger than 0.001 mm, so use its next-safe value.
            const double small = unit == 1.0 ? 0.0033 : 0.0011;
            station.width = (index == 0 ? 1.0 : small) * raw;
            station.depth = (index == 0 ? small : 1.0) * raw;
        } else {
            station.width = (index > 0 && index + 1 < phases.size() ? 88.0 : 80.0) * raw;
            station.depth = (index > 0 && index + 1 < phases.size() ? 44.0 : 40.0) * raw;
        }
        definition.base.stations.push_back(station);
        lc::StationMapping mapping;
        mapping.station = station.identifier;
        for (int lane = 0; lane < 4; ++lane)
            mapping.targetsByLane[std::size_t(lane)] =
                station.cornerIdentifiers[std::size_t((lane + phases[index]) % 4)];
        definition.mappings.push_back(mapping);
    }
    return definition;
}

lc::Admission Inspect(const lc::Definition& definition, proof::Inspection* inspected = nullptr,
                      tb::Counter* observed = nullptr) {
    std::atomic_bool cancelled{false};
    tb::Counter local;
    proof::Inspection result;
    const lc::Admission status = proof::Inspect(definition, cancelled,
                                                 observed ? *observed : local, result);
    if (inspected) *inspected = result;
    return status;
}

proof::Result Build(const lc::Definition& definition, proof::Inspection* inspected = nullptr,
                    tb::Counter* observed = nullptr) {
    std::atomic_bool cancelled{false};
    tb::Counter local;
    return proof::ConstructAndProve(definition, cancelled,
                                    observed ? *observed : local, inspected);
}

bool CorruptFirstPCurve(TopoDS_Solid& solid) {
    try {
        for (TopExp_Explorer faces(solid, TopAbs_FACE); faces.More(); faces.Next()) {
            const TopoDS_Face face = TopoDS::Face(faces.Current());
            if (Handle(Geom_BSplineSurface)::DownCast(BRep_Tool::Surface(face)).IsNull()) continue;
            TopExp_Explorer edges(face, TopAbs_EDGE);
            if (!edges.More()) return false;
            const TopoDS_Edge edge = TopoDS::Edge(edges.Current());
            double first = 0, last = 0;
            const Handle(Geom2d_Curve) original =
                BRep_Tool::CurveOnSurface(edge, face, first, last);
            if (original.IsNull()) return false;
            Handle(Geom2d_Curve) changed =
                Handle(Geom2d_Curve)::DownCast(original->Copy());
            if (changed.IsNull()) return false;
            changed->Translate(gp_Vec2d(0.125, 0.0));
            BRep_Builder builder;
            // Keep this adversarial copy generically BRep-valid under a wide
            // declared tolerance. The correspondence proof must still reject
            // the exact pcurve/support mismatch against its pinned tolerance.
            constexpr double corruptionTolerance = 1.0;
            builder.UpdateEdge(edge, changed, face, corruptionTolerance);
            builder.Range(edge, face, first, last);
            builder.UpdateFace(face, corruptionTolerance);
            return true;
        }
    } catch (...) {}
    return false;
}

bool ScenarioGeometry(double unit, double* values, std::uint64_t* words) {
    proof::Inspection implicitInspection, plusInspection, minusInspection;
    const lc::Definition implicit = MakeDefinition(unit, {0, 0}, true);
    const lc::Definition plus = MakeDefinition(unit, {0, 1});
    const lc::Definition minus = MakeDefinition(unit, {0, 3});
    const proof::Result a = Build(implicit, &implicitInspection);
    const proof::Result b = Build(plus, &plusInspection);
    const proof::Result c = Build(minus, &minusInspection);
    words[0] = std::uint64_t(a.status); words[1] = std::uint64_t(b.status);
    words[2] = std::uint64_t(c.status); words[3] = b.faceCount;
    words[4] = b.edgeCount; words[5] = b.vertexCount;
    words[6] = plusInspection.mapping.phases.size() == 2
        ? (std::uint64_t(plusInspection.mapping.phases[0]) << 8)
            | plusInspection.mapping.phases[1] : 0;
    values[0] = a.volume; values[1] = b.volume; values[2] = c.volume;
    const auto a0 = lc::detail::LaneCorners(implicit, 0, true);
    const auto a1 = lc::detail::LaneCorners(implicit, 1, true);
    const auto b1 = lc::detail::LaneCorners(plus, 1, true);
    const auto c1 = lc::detail::LaneCorners(minus, 1, true);
    values[3] = a0[0].X(); values[4] = a0[0].Y(); values[5] = a0[0].Z();
    values[6] = a1[0].X(); values[7] = a1[0].Y(); values[8] = a1[0].Z();
    values[9] = b1[0].X(); values[10] = b1[0].Y(); values[11] = b1[0].Z();
    values[12] = c1[0].X(); values[13] = c1[0].Y(); values[14] = c1[0].Z();
    values[15] = (a0[0].X() + b1[0].X()) * 0.5;
    values[16] = (a0[0].Y() + b1[0].Y()) * 0.5;
    values[17] = plusInspection.minimumSignedArea;
    std::atomic_bool cancelled{false};
    const std::array<lc::Definition, 3> definitions = {implicit, plus, minus};
    for (std::size_t index = 0; index < definitions.size(); ++index) {
        tb::Counter inspectCounter;
        proof::Inspection checked;
        words[18 + index * 4] = std::uint64_t(proof::Inspect(
            definitions[index], cancelled, inspectCounter, checked));
        lc::BuildResult built = lc::BuildUnproven(definitions[index], cancelled, inspectCounter);
        words[19 + index * 4] = std::uint64_t(built.status);
        words[20 + index * 4] = built.diagnosticStage;
        const proof::Result proved = proof::Verify(definitions[index], checked, built.solid,
                                                   cancelled, inspectCounter);
        words[21 + index * 4] = (std::uint64_t(proved.status) << 32)
            | proved.diagnosticStage;
    }
    return true;
}

bool ScenarioPermutations(double unit, double*, std::uint64_t* words) {
    std::array<int, 4> permutation = {0, 1, 2, 3};
    std::size_t unchanged = 0, accepted = 0, halfTurn = 0, nonCyclic = 0, other = 0;
    do {
        lc::Definition definition = MakeDefinition(unit, {0, 0});
        const auto corners = definition.base.stations[1].cornerIdentifiers;
        for (int lane = 0; lane < 4; ++lane)
            definition.mappings[1].targetsByLane[std::size_t(lane)] =
                corners[std::size_t(permutation[std::size_t(lane)])];
        switch (Inspect(definition)) {
            case lc::Admission::Unchanged: ++unchanged; break;
            case lc::Admission::Accepted: ++accepted; break;
            case lc::Admission::HalfTurn: ++halfTurn; break;
            case lc::Admission::NonCyclic: ++nonCyclic; break;
            default: ++other; break;
        }
    } while (std::next_permutation(permutation.begin(), permutation.end()));
    words[0] = unchanged; words[1] = accepted; words[2] = halfTurn;
    words[3] = nonCyclic; words[4] = other;
    return true;
}

bool ScenarioRefusals(double unit, double*, std::uint64_t* words) {
    words[0] = std::uint64_t(Inspect(MakeDefinition(unit, {0, 2})));
    lc::Definition noncyclic = MakeDefinition(unit, {0, 0});
    std::swap(noncyclic.mappings[1].targetsByLane[1],
              noncyclic.mappings[1].targetsByLane[2]);
    words[1] = std::uint64_t(Inspect(noncyclic));
    words[2] = std::uint64_t(Inspect(MakeDefinition(unit, {0, 1}, false, true)));
    lc::Definition duplicate = MakeDefinition(unit, {0, 1});
    duplicate.mappings[1].targetsByLane[1] = duplicate.mappings[1].targetsByLane[0];
    words[3] = std::uint64_t(Inspect(duplicate));
    lc::Definition count = MakeDefinition(unit, {0, 1});
    count.mappings.pop_back();
    words[4] = std::uint64_t(Inspect(count));
    lc::Definition order = MakeDefinition(unit, {0, 1});
    order.base.stations[1].z = order.base.stations[0].z;
    words[5] = std::uint64_t(Inspect(order));
    return true;
}

bool ScenarioAdversaries(double unit, double* values, std::uint64_t* words) {
    std::atomic_bool cancelled{false};
    const lc::Definition plus = MakeDefinition(unit, {0, 1});
    const lc::Definition minus = MakeDefinition(unit, {0, 3});
    proof::Inspection expected;
    tb::Counter inspectBudget;
    if (proof::Inspect(plus, cancelled, inspectBudget, expected) != lc::Admission::Accepted)
        return false;
    tb::Counter validBuildBudget;
    lc::BuildResult validBuild = lc::BuildUnproven(plus, cancelled, validBuildBudget);
    if (validBuild.status != lc::BuildStatus::BuiltUnproven) return false;
    tb::Counter validProofBudget = validBuildBudget;
    const proof::Result valid = proof::Verify(plus, expected, validBuild.solid,
                                              cancelled, validProofBudget);

    tb::Counter wrongBuildBudget;
    lc::BuildResult wrongBuild = lc::BuildUnproven(minus, cancelled, wrongBuildBudget);
    if (wrongBuild.status != lc::BuildStatus::BuiltUnproven) return false;
    tb::Counter wrongProofBudget = wrongBuildBudget;
    const proof::Result wrongHand = proof::Verify(plus, expected, wrongBuild.solid,
                                                  cancelled, wrongProofBudget);

    tb::Counter corruptBuildBudget;
    lc::BuildResult corruptBuild = lc::BuildUnproven(plus, cancelled, corruptBuildBudget);
    if (corruptBuild.status != lc::BuildStatus::BuiltUnproven
        || !CorruptFirstPCurve(corruptBuild.solid)) return false;
    tb::Counter corruptProofBudget = corruptBuildBudget;
    const proof::Result corrupt = proof::Verify(plus, expected, corruptBuild.solid,
                                                cancelled, corruptProofBudget);

    const lc::Definition three = MakeDefinition(unit, {0, 0, 0}, true);
    const lc::Definition two = MakeDefinition(unit, {0, 0}, true);
    proof::Inspection threeExpected;
    tb::Counter threeInspect;
    if (proof::Inspect(three, cancelled, threeInspect, threeExpected)
        != lc::Admission::Accepted) return false;
    tb::Counter twoBuildBudget;
    lc::BuildResult twoBuild = lc::BuildUnproven(two, cancelled, twoBuildBudget);
    if (twoBuild.status != lc::BuildStatus::BuiltUnproven) return false;
    tb::Counter missingProofBudget = twoBuildBudget;
    const proof::Result missing = proof::Verify(three, threeExpected, twoBuild.solid,
                                                cancelled, missingProofBudget);
    words[0] = std::uint64_t(valid.status); words[1] = std::uint64_t(wrongHand.status);
    words[2] = std::uint64_t(corrupt.status); words[3] = std::uint64_t(missing.status);
    values[0] = expected.expectedVolume;
    proof::Inspection minusExpected;
    tb::Counter minusInspect;
    words[4] = std::uint64_t(proof::Inspect(minus, cancelled, minusInspect, minusExpected));
    values[1] = minusExpected.expectedVolume;
    return true;
}

bool ScenarioIdentityAndFrame(double unit, double* values, std::uint64_t* words) {
    lc::Definition middle = MakeDefinition(unit, {0, 1, 0});
    const proof::Result middleResult = Build(middle);
    lc::Definition eight = MakeDefinition(unit, {0, 1, 0, 3, 0, 1, 0, 3});
    const lc::Admission eightStatus = Inspect(eight);

    lc::Definition positive = MakeDefinition(unit, {0, 1});
    core3d::profile::ConstructionFrame positiveFrame;
    positiveFrame.values = {-0.0, 2.0 * (0.001 / unit), -0.0, 0, 0, 0, 1, 1.5};
    positive.base.constructionFrame = positiveFrame;
    lc::Definition negative = positive;
    negative.base.constructionFrame->values = {-0.0, 2.0 * (0.001 / unit), -0.0,
                                                -0.0, -0.0, -0.0, -1.0, -1.5};
    const std::array<double, 8> positiveBits = positive.base.constructionFrame->values;
    const std::array<double, 8> negativeBits = negative.base.constructionFrame->values;
    const proof::Result positiveResult = Build(positive);
    const proof::Result negativeResult = Build(negative);

    words[0] = std::uint64_t(middleResult.status);
    words[1] = std::uint64_t(eightStatus);
    words[2] = std::uint64_t(positiveResult.status);
    words[3] = std::uint64_t(negativeResult.status);
    words[4] = middle.base.stations[1].identifier;
    words[5] = middle.base.stations[1].cornerIdentifiers[0];
    words[6] = middle.base.correspondence[0];
    words[7] = middle.mappings[1].targetsByLane[0];
    words[8] = Bits(positiveBits[0]); words[9] = Bits(negativeBits[0]);
    words[10] = Bits(positiveBits[6]); words[11] = Bits(negativeBits[6]);
    words[12] = Bits(positiveBits[7]); words[13] = Bits(negativeBits[7]);
    words[14] = middleResult.faceCount; words[15] = middleResult.edgeCount;
    words[16] = middleResult.vertexCount; words[17] = eight.mappings.size();
    values[0] = positiveResult.volume; values[1] = negativeResult.volume;
    values[2] = middleResult.volume;
    return true;
}

bool ScenarioBudget(double unit, double* values, std::uint64_t* words) {
    const lc::Definition definition = MakeDefinition(unit, {0, 1});
    std::atomic_bool running{false};
    tb::Counter measured;
    const proof::Result baseline = proof::ConstructAndProve(definition, running, measured);
    if (baseline.status != proof::ProofStatus::Proven || measured.exhausted) return false;

    tb::Counter exact;
    if (!exact.visit(tb::MaximumTopologyVisits - measured.topologyVisits)) return false;
    for (std::size_t stage = measured.buildStages; stage < tb::MaximumBuildStages; ++stage)
        if (!exact.beginStage()) return false;
    const proof::Result exactResult = proof::ConstructAndProve(definition, running, exact);

    tb::Counter visitsShort;
    if (!visitsShort.visit(tb::MaximumTopologyVisits - measured.topologyVisits + 1)) return false;
    const proof::Result visitsResult = proof::ConstructAndProve(definition, running, visitsShort);

    tb::Counter stagesShort;
    for (std::size_t stage = measured.buildStages - 1; stage < tb::MaximumBuildStages; ++stage)
        if (!stagesShort.beginStage()) return false;
    const proof::Result stagesResult = proof::ConstructAndProve(definition, running, stagesShort);

    tb::Counter inherited;
    inherited.exhausted = true;
    const proof::Result inheritedResult = proof::ConstructAndProve(definition, running, inherited);
    std::atomic_bool stopped{true};
    tb::Counter cancelled;
    const proof::Result cancelledResult = proof::ConstructAndProve(definition, stopped, cancelled);

    words[0] = std::uint64_t(baseline.status); words[1] = measured.topologyVisits;
    words[2] = measured.buildStages; words[3] = std::uint64_t(exactResult.status);
    words[4] = exact.topologyVisits; words[5] = exact.buildStages;
    words[6] = std::uint64_t(visitsResult.status); words[7] = visitsShort.exhausted;
    words[8] = std::uint64_t(stagesResult.status); words[9] = stagesShort.exhausted;
    words[10] = std::uint64_t(inheritedResult.status); words[11] = inherited.exhausted;
    words[12] = std::uint64_t(cancelledResult.status); words[13] = cancelled.topologyVisits;
    values[0] = baseline.volume;
    return true;
}
} // namespace

extern "C" std::uint64_t Core3DDebugLoftCorrespondenceGeometryProbe(
    int32_t scenario, double metersPerUnit, double *values, size_t valueCapacity,
    std::uint64_t *words, size_t wordCapacity) {
    if ((metersPerUnit != 0.001 && metersPerUnit != 1.0) || !values || !words
        || valueCapacity < 32 || wordCapacity < 32 || scenario < 0 || scenario > 5) return 0;
    std::fill(values, values + valueCapacity, 0.0);
    std::fill(words, words + wordCapacity, std::uint64_t(0));
    try {
        bool ok = false;
        switch (scenario) {
            case 0: ok = ScenarioGeometry(metersPerUnit, values, words); break;
            case 1: ok = ScenarioPermutations(metersPerUnit, values, words); break;
            case 2: ok = ScenarioRefusals(metersPerUnit, values, words); break;
            case 3: ok = ScenarioAdversaries(metersPerUnit, values, words); break;
            case 4: ok = ScenarioIdentityAndFrame(metersPerUnit, values, words); break;
            case 5: ok = ScenarioBudget(metersPerUnit, values, words); break;
        }
        return ok ? EvidenceVersion : 0;
    } catch (...) { return 0; }
}
#endif
