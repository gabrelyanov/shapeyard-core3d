// T-C C4 spline profile + revolve admission tests (draft package
// r4-tc-c4-spline-profile-revolve-kimi). Standalone C++17, built and run by
// run_prototype_tests.sh with retained OCCT libraries; no document, viewport
// or simulator. Each test maps to NOTES.md §6.
#include "SplineProfileFace.hxx"
#include "ProfileCurveFace.hxx"
#include <algorithm>
#include <cstdio>
#include <cmath>

using namespace core3d;

static int failures = 0;
static int checks = 0;
#define CHECK(condition, name) do { ++checks; if (!(condition)) { \
    ++failures; std::printf("FAIL %s\n", name); } else { \
    std::printf("ok %s\n", name); } } while (0)

static ProfileCurveVertex V(ProfileCurveID id, double u, double v) {
    return ProfileCurveVertex{id, gp_Pnt2d(u, v)};
}
static ProfileCurveSegment L(ProfileCurveID id, ProfileCurveID a, ProfileCurveID b) {
    ProfileCurveSegment s; s.identifier = id; s.startVertex = a; s.endVertex = b;
    s.kind = ProfileCurveKind::Line; return s;
}
static ProfileCurveSegment S(ProfileCurveID id, ProfileCurveID a, ProfileCurveID b) {
    ProfileCurveSegment s; s.identifier = id; s.startVertex = a; s.endVertex = b;
    s.kind = ProfileCurveKind::Spline; return s;
}
// Cubic Bezier as a clamped B-spline segment.
static SplineCurveSegment Cubic(ProfileCurveID id, ProfileCurveID poleBase,
    double u0, double v0, double u1, double v1, double u2, double v2,
    double u3, double v3) {
    SplineCurveSegment s;
    s.identifier = id; s.degree = 3;
    s.poles = {V(poleBase, u0, v0), V(poleBase + 1, u1, v1),
               V(poleBase + 2, u2, v2), V(poleBase + 3, u3, v3)};
    s.knots = {0.0, 1.0};
    s.multiplicities = {4, 4};
    return s;
}

// Mixed contour: lines on the constant-V sides, cubic splines bulging on the
// constant-U sides. All pole U in [9, 21].
static ProfileCurveSection MixedSection() {
    ProfileCurveSection section;
    section.outer.identifier = 1;
    section.outer.vertices = {V(11, 10, 0), V(12, 20, 0), V(13, 20, 30), V(14, 10, 30)};
    section.outer.segments = {L(21, 11, 12), S(22, 12, 13), L(23, 13, 14), S(24, 14, 11)};
    return section;
}
static SplineProfileSpec MixedSpec() {
    SplineProfileSpec spec;
    spec.segments = {Cubic(22, 31, 20, 0, 21, 10, 19, 20, 20, 30),
                     Cubic(24, 41, 10, 30, 9, 20, 11, 10, 10, 0)};
    return spec;
}
static const SplineRevolveAxis kLegacyAxis{gp_Pnt2d(0, 0), gp_Pnt2d(0, 1)};

static void testSplineFaceBuildsFromMixedContour() {
    auto section = MixedSection(); auto spec = MixedSpec();
    std::atomic_bool cancelled{false};
    SplineProfileFaceResult built;
    CHECK(BuildSplineProfileFace(section, spec, 0, cancelled, built) && !built.face.IsNull()
        && built.expectedArea > 100, "testSplineFaceBuildsFromMixedContour");
}
static void testGaussQuadratureExactForLinearSpline() {
    // Degree-1 splines are lines: the quadrature must reproduce the polygon area.
    ProfileCurveSection section;
    section.outer.identifier = 1;
    section.outer.vertices = {V(11, 5, 0), V(12, 15, 0), V(13, 15, 10)};
    section.outer.segments = {S(21, 11, 12), S(22, 12, 13), L(23, 13, 11)};
    SplineCurveSegment a; a.identifier = 21; a.degree = 1;
    a.poles = {V(31, 5, 0), V(32, 15, 0)}; a.knots = {0, 1}; a.multiplicities = {2, 2};
    SplineCurveSegment b; b.identifier = 22; b.degree = 1;
    b.poles = {V(33, 15, 0), V(34, 15, 10)}; b.knots = {0, 1}; b.multiplicities = {2, 2};
    SplineProfileSpec spec; spec.segments = {a, b};
    SplineProfileSectionMoments moments;
    CHECK(InspectSplineProfileSection(section, spec, moments)
        && std::abs(moments.area - 50.0) < 1e-9, "testGaussQuadratureExactForLinearSpline");
}
static void testArcMomentMatchesLegacyClosedForm() {
    // The arc accumulation must equal InspectProfileCurveLoopStructure's values.
    ProfileCurveLoop loop; loop.identifier = 1;
    loop.vertices = {V(11, 10, 0), V(12, 20, 0), V(13, 20, 10), V(14, 10, 10)};
    ProfileCurveSegment arc; arc.identifier = 22; arc.startVertex = 12; arc.endVertex = 13;
    arc.kind = ProfileCurveKind::CircularArc; arc.center = gp_Pnt2d(20, 5); arc.radius = 5;
    arc.startDegrees = -90; arc.sweepDegrees = 180;
    loop.segments = {L(21, 11, 12), arc, L(23, 13, 14), L(24, 14, 11)};
    std::set<ProfileCurveID> ids; std::size_t nv = 0, ns = 0;
    ProfileCurveLoopInspection legacy;
    CHECK(InspectProfileCurveLoopStructure(loop, gp_Pnt2d(10, 0), ids, nv, ns, legacy),
        "testArcMomentMatchesLegacyClosedForm.legacy");
    SplineProfileSpec spec; // no splines: loop must still admit through the new path
    spline_profile_admission::LoopMoments mixed;
    std::set<ProfileCurveID> ids2, splines; std::size_t elements = 0;
    const double shift = legacy.signedArea; // same origin independence check below
    (void)shift;
    CHECK(InspectSplineProfileLoop(loop, spec, ids2, splines, elements, mixed)
        && std::abs(mixed.signedArea - legacy.signedArea) < 1e-9
        && std::abs(mixed.signedMomentU - (legacy.signedFirstMomentX + 10.0 * legacy.signedArea)) < 1e-9,
        "testArcMomentMatchesLegacyClosedForm");
}
static void testSplineRevolveFullVolumeMatchesPappus() {
    auto section = MixedSection(); auto spec = MixedSpec();
    spec.revolveAxis = kLegacyAxis;
    std::atomic_bool cancelled{false};
    SplineProfileFaceResult face;
    double area = 0, volume = 0;
    bool admitted = SplineProfileExpectedVolume(section, spec, 0, 360.0, true,
        [&] { return cancelled.load(); }, area, volume)
        && BuildSplineProfileFace(section, spec, 0, cancelled, face);
    SplineRevolveResult revolved;
    CHECK(admitted && BuildSplineProfileRevolve(face.face, *spec.revolveAxis, 0, 360.0,
            volume, cancelled, revolved)
        && revolved.seamFaces == 0 && revolved.measuredVolume > 0,
        "testSplineRevolveFullVolumeMatchesPappus");
}
static void testSplineRevolvePartialSeamFaces() {
    auto section = MixedSection(); auto spec = MixedSpec();
    spec.revolveAxis = kLegacyAxis;
    std::atomic_bool cancelled{false};
    SplineProfileFaceResult face;
    double area = 0, volume = 0;
    bool admitted = SplineProfileExpectedVolume(section, spec, 0, 120.0, true,
        [&] { return cancelled.load(); }, area, volume)
        && BuildSplineProfileFace(section, spec, 0, cancelled, face);
    SplineRevolveResult revolved;
    CHECK(admitted && BuildSplineProfileRevolve(face.face, *spec.revolveAxis, 0, 120.0,
            volume, cancelled, revolved) && revolved.seamFaces == 2,
        "testSplineRevolvePartialSeamFaces");
}
static void testAxisCrossingRefused() {
    auto section = MixedSection(); auto spec = MixedSpec();
    spec.revolveAxis = SplineRevolveAxis{gp_Pnt2d(15, 0), gp_Pnt2d(0, 1)}; // through the section
    std::atomic_bool cancelled{false};
    double area = 0, volume = 0;
    CHECK(!SplineProfileExpectedVolume(section, spec, 0, 360.0, true,
        [&] { return cancelled.load(); }, area, volume), "testAxisCrossingRefused");
}
static void testAxisTouchingPerpendicularTangentAdmitted() {
    ProfileCurveSection section;
    section.outer.identifier = 1;
    section.outer.vertices = {V(11, 20, 0), V(12, 40, 0), V(13, 40, 20)};
    section.outer.segments = {S(21, 11, 12), L(22, 12, 13), L(23, 13, 11)};
    SplineProfileSpec spec;
    spec.segments = {Cubic(21, 31, 20, 0, 26.5, 0, 33.5, 0, 40, 0)}; // tangent perp. at axis
    spec.revolveAxis = SplineRevolveAxis{gp_Pnt2d(20, 0), gp_Pnt2d(0, 1)};
    std::atomic_bool cancelled{false};
    double area = 0, volume = 0;
    CHECK(SplineProfileExpectedVolume(section, spec, 0, 360.0, true,
        [&] { return cancelled.load(); }, area, volume) && volume > 0,
        "testAxisTouchingPerpendicularTangentAdmitted");
}
static void testAxisTouchingAmbiguousTangentRefused() {
    ProfileCurveSection section;
    section.outer.identifier = 1;
    section.outer.vertices = {V(11, 20, 0), V(12, 40, 0), V(13, 40, 20)};
    section.outer.segments = {S(21, 11, 12), L(22, 12, 13), L(23, 13, 11)};
    SplineProfileSpec spec;
    spec.segments = {Cubic(21, 31, 20, 0, 28, 5, 34, 10, 40, 0)}; // axial tangent at contact
    spec.revolveAxis = SplineRevolveAxis{gp_Pnt2d(20, 0), gp_Pnt2d(0, 1)};
    std::atomic_bool cancelled{false};
    double area = 0, volume = 0;
    CHECK(!SplineProfileExpectedVolume(section, spec, 0, 360.0, true,
        [&] { return cancelled.load(); }, area, volume),
        "testAxisTouchingAmbiguousTangentRefused");
}
static ProfileCurveSection HoistedHoleSection(bool reversedWinding) {
    auto section = MixedSection();
    ProfileCurveLoop hole; hole.identifier = 2;
    if (!reversedWinding) {
        hole.vertices = {V(51, 12, 5), V(52, 14, 5), V(53, 14, 15), V(54, 12, 15)};
        hole.segments = {L(61, 51, 52), L(62, 52, 53), L(63, 53, 54), L(64, 54, 51)};
    } else {
        hole.vertices = {V(51, 12, 5), V(52, 12, 15), V(53, 14, 15), V(54, 14, 5)};
        hole.segments = {L(61, 51, 52), L(62, 52, 53), L(63, 53, 54), L(64, 54, 51)};
    }
    section.inner.push_back(hole);
    return section;
}
static void testInnerLoopPolicyRefuseRejectsHoles() {
    auto section = HoistedHoleSection(false); auto spec = MixedSpec();
    spec.revolveAxis = kLegacyAxis; // policy stays Refuse
    std::atomic_bool cancelled{false};
    double area = 0, volume = 0;
    CHECK(!SplineProfileExpectedVolume(section, spec, 0, 360.0, true,
        [&] { return cancelled.load(); }, area, volume),
        "testInnerLoopPolicyRefuseRejectsHoles");
}
static void testInnerLoopVoidRevolveVolume() {
    auto section = HoistedHoleSection(false); auto spec = MixedSpec();
    spec.revolveAxis = kLegacyAxis;
    spec.innerLoopPolicy = SplineInnerLoopPolicy::Void;
    std::atomic_bool cancelled{false};
    double area = 0, withHole = 0, areaSolid = 0, solidVolume = 0;
    bool admitted = SplineProfileExpectedVolume(section, spec, 0, 360.0, true,
        [&] { return cancelled.load(); }, area, withHole);
    auto solidSection = MixedSection(); auto solidSpec = MixedSpec();
    solidSpec.revolveAxis = kLegacyAxis;
    admitted = admitted && SplineProfileExpectedVolume(solidSection, solidSpec, 0, 360.0, true,
        [&] { return cancelled.load(); }, areaSolid, solidVolume);
    SplineProfileFaceResult face; SplineRevolveResult revolved;
    CHECK(admitted && withHole < solidVolume && withHole > 0
        && BuildSplineProfileFace(section, spec, 0, cancelled, face)
        && BuildSplineProfileRevolve(face.face, *spec.revolveAxis, 0, 360.0, withHole,
            cancelled, revolved),
        "testInnerLoopVoidRevolveVolume");
}
static void testInnerLoopWindingCorrected() {
    auto section = HoistedHoleSection(true); auto spec = MixedSpec();
    spec.revolveAxis = kLegacyAxis;
    spec.innerLoopPolicy = SplineInnerLoopPolicy::Void;
    std::atomic_bool cancelled{false};
    double area = 0, volume = 0;
    CHECK(SplineProfileExpectedVolume(section, spec, 0, 360.0, true,
        [&] { return cancelled.load(); }, area, volume) && volume > 0,
        "testInnerLoopWindingCorrected");
}
static void testSelfIntersectingSplineRefused() {
    auto section = MixedSection(); auto spec = MixedSpec();
    // The former U=8 fixture is a separated simple contour and is retained as
    // a positive control. U=4 makes the two spline edges cross twice.
    spec.segments = {Cubic(22, 31, 20, 0, 8, 10, 8, 20, 20, 30),
                     Cubic(24, 41, 10, 30, 9, 20, 11, 10, 10, 0)};
    std::atomic_bool cancelled{false};
    SplineProfileFaceResult positiveFace;
    const bool positiveBuilt = BuildSplineProfileFace(
        section, spec, 0, cancelled, positiveFace);

    spec.segments[0] = Cubic(22, 31, 20, 0, 4, 10, 4, 20, 20, 30);
    gp_Pnt2d right0, right15, right30, derivative;
    gp_Pnt2d left0, left15, left30;
    const bool sampled = SplineEvaluate(spec.segments[0], 0.0, right0, derivative)
        && SplineEvaluate(spec.segments[0], 0.5, right15, derivative)
        && SplineEvaluate(spec.segments[0], 1.0, right30, derivative)
        && SplineEvaluate(spec.segments[1], 0.0, left30, derivative)
        && SplineEvaluate(spec.segments[1], 0.5, left15, derivative)
        && SplineEvaluate(spec.segments[1], 1.0, left0, derivative);
    const bool separationChangesSign = sampled
        && right0.X() - left0.X() > 0
        && right15.X() - left15.X() < 0
        && right30.X() - left30.X() > 0;
    SplineProfileSectionMoments negativeMoments;
    const bool negativeInspected = InspectSplineProfileSection(
        section, spec, negativeMoments);
    SplineProfileFaceResult negativeFace;
    const bool negativeBuilt = BuildSplineProfileFace(
        section, spec, 0, cancelled, negativeFace);
    CHECK(positiveBuilt && !positiveFace.face.IsNull() && separationChangesSign
        && negativeInspected && !negativeBuilt && negativeFace.face.IsNull(),
        "testSelfIntersectingSplineRefused");
}
static void testLegacyLineArcSectionUnchanged() {
    // Legacy curves sections (no spline payloads) keep the legacy builder; the
    // new path refuses them so routing stays explicit.
    ProfileCurveSection section;
    section.outer.identifier = 1;
    section.outer.vertices = {V(11, 10, 0), V(12, 20, 0), V(13, 20, 10), V(14, 10, 10)};
    section.outer.segments = {L(21, 11, 12), L(22, 12, 13), L(23, 13, 14), L(24, 14, 11)};
    SplineProfileSpec spec;
    SplineProfileSectionMoments moments;
    CHECK(!InspectSplineProfileSection(section, spec, moments),
        "testLegacyLineArcSectionUnchanged.newPathRefuses");
    ProfileCurveFaceResult legacy;
    std::atomic_bool cancelled{false};
    CHECK(BuildProfileCurveFace(section, 0, cancelled, legacy) && !legacy.face.IsNull()
        && std::abs(legacy.expectedArea - 100.0) < 1e-9,
        "testLegacyLineArcSectionUnchanged.legacyBuilder");
}
static void testSplineExtrudeVolume() {
    auto section = MixedSection(); auto spec = MixedSpec(); // no axis: extrusion
    std::atomic_bool cancelled{false};
    double area = 0, volume = 0;
    SplineProfileFaceResult face; TopoDS_Solid solid;
    CHECK(SplineProfileExpectedVolume(section, spec, 0, 5.0, false,
            [&] { return cancelled.load(); }, area, volume)
        && BuildSplineProfileFace(section, spec, 0, cancelled, face)
        && BuildSplineProfileExtrude(face.face, 0, 5.0, volume, cancelled, solid),
        "testSplineExtrudeVolume");
}
static void testEditRebuildParityAndDeterminism() {
    // Rebuilding from identical source values is bit-deterministic; editing one
    // pole changes the solid and the edited value remains the only source change.
    auto section = MixedSection(); auto spec = MixedSpec();
    spec.revolveAxis = kLegacyAxis;
    struct Measurement {
        bool succeeded = false;
        double authoredVolume = 0;
        double measuredVolume = 0;
    };
    const auto build = [](const ProfileCurveSection& s, const SplineProfileSpec& p) {
        std::atomic_bool stop{false};
        double area = 0;
        Measurement measurement;
        SplineProfileFaceResult face;
        SplineRevolveResult out;
        const bool admitted = SplineProfileExpectedVolume(s, p, 0, 360.0, true,
            [&] { return stop.load(); }, area, measurement.authoredVolume);
        const bool faceBuilt = admitted && BuildSplineProfileFace(s, p, 0, stop, face);
        const bool revolved = faceBuilt && BuildSplineProfileRevolve(
            face.face, *p.revolveAxis, 0, 360.0, measurement.authoredVolume, stop, out);
        measurement.measuredVolume = out.measuredVolume;
        const double parityTolerance = std::max(
            1e-8, measurement.authoredVolume * 1e-6);
        measurement.succeeded = revolved && out.seamFaces == 0
            && std::isfinite(measurement.authoredVolume)
            && std::isfinite(measurement.measuredVolume)
            && measurement.authoredVolume > 0 && measurement.measuredVolume > 0
            && std::abs(measurement.measuredVolume - measurement.authoredVolume)
                <= parityTolerance;
        return measurement;
    };
    const double pi = std::acos(-1.0);
    const double baselineAuthored = 9000.0 * pi;
    const double editedAuthored = (65127.0 / 7.0) * pi;
    const SplineProfileSpec baselineSource = spec;
    const Measurement first = build(section, spec);
    const Measurement second = build(section, spec);
    CHECK(first.succeeded && second.succeeded
        && std::abs(first.authoredVolume - baselineAuthored) < 1e-9
        && first.authoredVolume == second.authoredVolume
        && first.measuredVolume == second.measuredVolume,
        "testEditRebuildParityAndDeterminism.replayDeterministic");
    spec.segments[0].poles[1].point = gp_Pnt2d(22.0, 10.0); // edit one pole
    SplineProfileSpec expectedEditedSource = baselineSource;
    expectedEditedSource.segments[0].poles[1].point = gp_Pnt2d(22.0, 10.0);
    const bool onlyPole32Changed = spec.segments == expectedEditedSource.segments
        && spec.revolveAxis->origin.X() == baselineSource.revolveAxis->origin.X()
        && spec.revolveAxis->origin.Y() == baselineSource.revolveAxis->origin.Y()
        && spec.revolveAxis->direction.X() == baselineSource.revolveAxis->direction.X()
        && spec.revolveAxis->direction.Y() == baselineSource.revolveAxis->direction.Y()
        && spec.innerLoopPolicy == baselineSource.innerLoopPolicy;
    const Measurement editedFirst = build(section, spec);
    const Measurement editedSecond = build(section, spec);
    spec.segments[0].poles[1].point = gp_Pnt2d(21.0, 10.0);
    const bool sourceRestored = spec.segments == baselineSource.segments;
    const Measurement restored = build(section, spec);
    CHECK(onlyPole32Changed && editedFirst.succeeded && editedSecond.succeeded
        && std::abs(editedFirst.authoredVolume - editedAuthored) < 1e-9
        && editedFirst.authoredVolume == editedSecond.authoredVolume
        && editedFirst.measuredVolume == editedSecond.measuredVolume
        && editedFirst.authoredVolume != first.authoredVolume
        && editedFirst.measuredVolume != first.measuredVolume
        && sourceRestored && restored.succeeded
        && restored.authoredVolume == first.authoredVolume
        && restored.measuredVolume == first.measuredVolume,
        "testEditRebuildParityAndDeterminism.editedPoleRebuilds");
}

static void testRegistryAllocationUsedAndRawKindRefused() {
    // C4 consumes the central G0 allocation. A matching numeric key is still
    // refused by a sealed registry until a complete descriptor is installed;
    // a raw integer cannot activate this geometry-only slice.
    const retained_feature::RegistryView sealed(nullptr, 0);
    const retained_feature::Key rawKey{
        SplineProfileRevolveRegistryKey.kind,
        SplineProfileRevolveRegistryKey.codecVersion};
    CHECK(SplineProfileRevolveRegistryKey.kind
            == retained_feature::SplineProfileRevolveKind
        && SplineProfileRevolveRegistryKey.codecVersion == 1
        && sealed.valid() && sealed.find(rawKey) == nullptr,
        "testRegistryAllocationUsedAndRawKindRefused");
}

int main() {
    testSplineFaceBuildsFromMixedContour();
    testGaussQuadratureExactForLinearSpline();
    testArcMomentMatchesLegacyClosedForm();
    testSplineRevolveFullVolumeMatchesPappus();
    testSplineRevolvePartialSeamFaces();
    testAxisCrossingRefused();
    testAxisTouchingPerpendicularTangentAdmitted();
    testAxisTouchingAmbiguousTangentRefused();
    testInnerLoopPolicyRefuseRejectsHoles();
    testInnerLoopVoidRevolveVolume();
    testInnerLoopWindingCorrected();
    testSelfIntersectingSplineRefused();
    testLegacyLineArcSectionUnchanged();
    testSplineExtrudeVolume();
    testEditRebuildParityAndDeterminism();
    testRegistryAllocationUsedAndRawKindRefused();
    std::printf("%s (%d/%d checks)\n", failures == 0 ? "PASS" : "FAIL", checks - failures, checks);
    return failures == 0 ? 0 : 1;
}
