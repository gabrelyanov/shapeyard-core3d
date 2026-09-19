// B04/D34 diagnostic regression. A passing test preserves the precise current
// refusal; it does NOT qualify ring admission on a revolved retained source.
#include "CircularHostProofProbe.hxx"
#include "RetainedBooleanEditValues.hxx"
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <cassert>
#include <iostream>

using namespace core3d;
namespace fixture = saved_cut_circular_host::probe;

static retained_solid::Envelope wheel() {
    auto e = saved_cut_bore_clearance::probe::identity(.001);
    e.sourceFamily = 1;
    e.sourceSchema = 1;
    e.axis = unsigned(analytic_boolean::Axis::Y);
    e.radius = 20;
    profile::Parameters p;
    p.metersPerUnit = .001;
    p.definition.plane = 0;
    p.definition.revolve = true;
    p.definition.depth = 360;
    p.definition.points = {{0, -20}, {140, -20}, {140, 20}, {0, 20}};
    assert(profile::Encode(p, e.sourceValues));
    assert(retained_solid::Valid(e));
    return e;
}

static void validVolume(const TopoDS_Shape& shape, double timesPi) {
    assert(!shape.IsNull());
    assert(BRepCheck_Analyzer(shape, Standard_True).IsValid());
    unsigned solids = 0;
    for (TopExp_Explorer it(shape, TopAbs_SOLID); it.More(); it.Next()) ++solids;
    assert(solids == 1);
    assert(fixture::Volume(shape, timesPi));
}

int main() {
    const auto e = wheel();
    retained_boolean::Program p;
    assert(retained_boolean::Promote(e, p));
    analytic_boolean_ring::Ring ring{2, analytic_boolean::Axis::Y, {}, 95, 15, 95. / 140., 6};
    assert(analytic_boolean_ring::Inspect(ring, .001) == analytic_boolean_ring::Status::Clear);
    auto operand = analytic_boolean_ring::Expand(ring, 0, .001);
    assert(operand.identifier == 2 && operand.radius == 15 && operand.point[0] == 95);

    // Isolate the FIRST production failure, before any ligament calculation.
    saved_cut_whole_result::Expected expected;
    assert(!saved_cut_whole_result::ExpectedSource(e, expected));
    double extent = -1;
    assert(!saved_boolean_result::detail::HostRadialExtent(p, operand, extent));
    assert(extent == 0);
    assert(saved_cut_bore_clearance::Inspect(e).status == saved_cut_bore_clearance::Status::UnsupportedFamily);
    std::vector<std::uint8_t> before, after;
    assert(retained_boolean::Encode(retained_boolean::Recipe(e), before));
    assert(!retained_boolean::Apply(e,
        retained_boolean::AppendRing{{analytic_boolean::Axis::Y, {}, 95, 15, 6}}, 1));
    assert(retained_boolean::Encode(retained_boolean::Recipe(e), after) && before == after);
    std::cout << "B04 production: ExpectedSource=false; HostRadialExtent=false (extent=0); "
                 "clearance=UnsupportedFamily; AppendRing=refused; UI=ring.OutsideOrInsufficientLigament\n";

    // Construct exactly the production XY full-revolution section about Y.
    profile::Parameters source;
    assert(profile::Decode(e.sourceValues, source));
    BRepBuilderAPI_MakePolygon outline;
    for (const auto& point : source.definition.points) outline.Add(gp_Pnt(point.X(), point.Y(), 0));
    outline.Close();
    assert(outline.IsDone());
    BRepBuilderAPI_MakeFace face(outline.Wire());
    assert(face.IsDone());
    BRepPrimAPI_MakeRevol revolve(face.Face(), gp_Ax1(gp::Origin(), gp::DY()), Standard_True);
    assert(revolve.IsDone());
    const auto base = revolve.Shape();
    validVolume(base, 784000);
    auto cut = fixture::Cut(base, e);
    validVolume(cut, 768000);
    for (unsigned n = 0; n < ring.count; ++n) {
        const auto disk = analytic_boolean_ring::Expand(ring, n, .001);
        const auto view = saved_boolean_result::detail::GeometryView(p, disk);
        assert(saved_cut_bore_clearance::Inspect(view).status == saved_cut_bore_clearance::Status::UnsupportedFamily);
        cut = fixture::Cut(cut, view);
        validVolume(cut, 768000 - 9000 * (n + 1));
    }
    std::cout << "B04 detached kernel: R140 x 40 revolve, r20 hub, 6 x r15 @ R95; "
                 "valid single solid; volume=" << fixture::VolumeMM3(cut)
              << " mm^3 (714000*pi); rim gap=30; hub gap=60; adjacent chord gap=65 mm\n";

    // Same radial dimensions on an already supported circular-profile host.
    // This is a CONTROL only: never replace the B04 retained revolve recipe.
    auto circular = fixture::Wheel(0, 1);
    assert(profile::Decode(circular.sourceValues, source));
    source.definition.circle->outerRadius = 140;
    assert(profile::Encode(source, circular.sourceValues));
    const auto admitted = retained_boolean::Apply(circular,
        retained_boolean::AppendRing{{analytic_boolean::Axis::Y, {}, 95, 15, 6}}, 1);
    assert(admitted);
    const auto& circularProgram = std::get<retained_boolean::Program>(admitted->recipe);
    assert(saved_boolean_result::detail::SeparateDisks(circularProgram));
    assert(saved_boolean_result::detail::AdmitDisks(circularProgram));
    std::cout << "Circular-profile control: identical B04 radial dimensions admitted\n";

    // R150 disk + r20 bore at 130 is tangent; annulus inner R40 + r10
    // bore at 49 overlaps by 1, and at 50.001 leaves only .001 mm (< .002).
    for (const auto& check : fixture::RunClearanceIntervals()) {
        std::cout << "existing clearance " << check.first << "=" << check.second << '\n';
        assert(check.second);
    }
    std::cout << "PASS diagnostic regression; B04 ring admission remains BLOCKED (D34)\n";
}
