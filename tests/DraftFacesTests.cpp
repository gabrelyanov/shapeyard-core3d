#include "DraftFacesBuild.hxx"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <cassert>
#include <iostream>
#include <limits>

using namespace core3d;

namespace {
retained_recipe::UUID uuid(std::uint8_t value) {
    retained_recipe::UUID output{}; output.back() = value; return output;
}
retained_recipe::Digest digest(std::uint8_t value) {
    retained_recipe::Digest output{}; output.back() = value; return output;
}

draft_faces::Definition sideDefinition(double metersPerUnit, double angleRadians) {
    using namespace draft_faces;
    const double scale = 0.001 / metersPerUnit;
    Definition value; value.metersPerLocalUnit = metersPerUnit;
    value.neutralOriginLocal = {{0, 0, 0}};
    value.neutralNormalLocal = {{0, 0, 1}};
    value.pullDirectionLocal = {{0, 0, 1}};
    value.signedAngleRadians = angleRadians;
    FaceIntent face; face.identifier = uuid(1); face.surface = SurfaceKind::Planar;
    face.expectedBoundaryEdges = 4; face.expectedCardinality = 1;
    face.sampleOnNeutralLocal = {{0, 10 * scale, 0}};
    face.outwardNormalLocal = {{-1, 0, 0}}; face.selectorProof = digest(2);
    value.faces.push_back(face); return value;
}

draft_faces::Definition bottomDefinition() {
    using namespace draft_faces;
    Definition value = sideDefinition(0.001, 10 * M_PI / 180.0);
    value.faces[0].sampleOnNeutralLocal = {{15, 10, 0}};
    value.faces[0].outwardNormalLocal = {{0, 0, -1}};
    return value;
}
} // namespace

void testB4PayloadCodecRoundTripsInBothUnitsAndUsesDistinctSYCRKind() {
    using namespace draft_faces;
    static_assert(FeatureKind == 0x00002004);
    static_assert(FeatureKind != 0x00002003);
    for (double unit : {0.001, 1.0}) {
        const Definition input = sideDefinition(unit, -7.5 * M_PI / 180.0);
        std::vector<std::uint8_t> bytes; assert(Encode(input, bytes));
        assert(bytes.size() == 204); assert(std::memcmp(bytes.data(), "SYDF", 4) == 0);
        Definition decoded; assert(Decode(bytes, decoded)); assert(decoded == input);
        std::vector<std::uint8_t> exact; assert(Encode(decoded, exact)); assert(exact == bytes);
        exact.push_back(0); assert(!Decode(exact, decoded));
    }
}

void testB4PlanarDraftProvesSignedAngleNeutralIntersectionAndAnalyticSections() {
    using namespace draft_faces;
    const std::atomic_bool cancelled{false};
    for (double unit : {0.001, 1.0}) {
        const double scale = 0.001 / unit;
        const TopoDS_Shape box = BRepPrimAPI_MakeBox(30 * scale, 20 * scale, 40 * scale).Shape();
        for (double angle : {10 * M_PI / 180.0, -6 * M_PI / 180.0}) {
            const BuildResult result = BuildDeterministically(
                box, sideDefinition(unit, angle), cancelled);
            if (!result.built()) std::cerr << "B4 planar refusal: "
                << Reason(result.refusal) << " unit=" << unit << " angle=" << angle << "\n";
            assert(result.built()); assert(result.evidence.adds.size() == 1);
            assert(result.evidence.adds[0].addDone);
            assert(result.evidence.adds[0].status == int(Draft_NoError));
            assert(result.evidence.sections.size() == 3);
            assert(std::abs(result.evidence.measuredSignedAngleRadians - angle) < 1e-10);
            assert(result.evidence.neutralSectionEdges == 4);
            assert(std::abs(result.evidence.neutralSectionLengthLocal - 100 * scale) < 1e-8);
            assert(result.evidence.connectedSolids == 1);
            assert(result.evidence.sourceBytesUnchanged);
            assert(result.evidence.candidateBytesDistinct);
        }
    }
}

void testB4CylindricalAndConicalFacesRefuseWithDistinctTypedReasons() {
    using namespace draft_faces;
    const std::atomic_bool cancelled{false};
    Definition cylinder = sideDefinition(0.001, 10 * M_PI / 180.0);
    cylinder.faces[0].surface = SurfaceKind::Cylindrical;
    cylinder.faces[0].sampleOnNeutralLocal = {{10, 0, 0}};
    cylinder.faces[0].outwardNormalLocal = {{1, 0, 0}};
    const BuildResult cylinderResult = Build(
        BRepPrimAPI_MakeCylinder(10, 40).Shape(), cylinder, cancelled);
    assert(!cylinderResult.built());
    assert(cylinderResult.refusal == Refusal::UnsupportedCylindricalFace);

    Definition cone = cylinder; cone.faces[0].surface = SurfaceKind::Conical;
    const BuildResult coneResult = Build(BRepPrimAPI_MakeCone(10, 5, 40).Shape(), cone, cancelled);
    assert(!coneResult.built());
    assert(coneResult.refusal == Refusal::UnsupportedConicalFace);
}

void testB4InvertedTangentAndObliquePullDirectionsRefuseBeforeKernelWork() {
    using namespace draft_faces;
    Refusal refusal; Definition value = sideDefinition(0.001, 0.1);
    value.pullDirectionLocal = {{0, 0, -1}};
    assert(!Validate(value, refusal)); assert(refusal == Refusal::InvertedPullDirection);
    value.pullDirectionLocal = {{1, 0, 0}};
    assert(!Validate(value, refusal)); assert(refusal == Refusal::TangentPullDirection);
    value.pullDirectionLocal = {{std::sqrt(0.5), 0, std::sqrt(0.5)}};
    assert(!Validate(value, refusal)); assert(refusal == Refusal::ObliquePullDirection);
}

void testB4AddDoneFailureIsTypedAndLeavesDetachedSourceUnchanged() {
    using namespace draft_faces;
    const std::atomic_bool cancelled{false};
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(30, 20, 40).Shape();
    std::vector<std::uint8_t> before, after;
    assert(detail::exactShapeBytes(box, before));
    const BuildResult result = Build(box, bottomDefinition(), cancelled);
    assert(!result.built()); assert(result.refusal == Refusal::AddDoneFailure);
    assert(result.evidence.adds.size() == 1); assert(!result.evidence.adds[0].addDone);
    assert(result.evidence.adds[0].status != int(Draft_NoError));
    assert(detail::exactShapeBytes(box, after)); assert(before == after);
}

void testB4TopologyChangeRefusesWithoutRebindingTheFaceIntent() {
    using namespace draft_faces;
    const std::atomic_bool cancelled{false}; const Definition value = sideDefinition(0.001, 0.1);
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(30, 20, 40).Shape();
    assert(BuildDeterministically(box, value, cancelled).built());
    const gp_Ax2 axis(gp_Pnt(-1, 10, 20), gp::DX());
    const TopoDS_Shape bore = BRepPrimAPI_MakeCylinder(axis, 3, 32).Shape();
    BRepAlgoAPI_Cut cut(box, bore); cut.Build(); assert(cut.IsDone());
    const BuildResult changed = Build(cut.Shape(), value, cancelled);
    assert(!changed.built());
    assert(changed.refusal == Refusal::AnchorMissing
        || changed.refusal == Refusal::AnchorAmbiguous);
}

int main() {
    testB4PayloadCodecRoundTripsInBothUnitsAndUsesDistinctSYCRKind();
    testB4PlanarDraftProvesSignedAngleNeutralIntersectionAndAnalyticSections();
    testB4CylindricalAndConicalFacesRefuseWithDistinctTypedReasons();
    testB4InvertedTangentAndObliquePullDirectionsRefuseBeforeKernelWork();
    testB4AddDoneFailureIsTypedAndLeavesDetachedSourceUnchanged();
    testB4TopologyChangeRefusesWithoutRebindingTheFaceIntent();
    std::cout << "DraftFacesTests: PASS\n";
}
