#include "../Core3D/OCCTKit/PlanarSweepTangentArc.hxx"
#include "../Core3D/OCCTKit/PlanarSweepDefinition.hxx"
#include "../Core3D/OCCTKit/ProfileDefinition.hxx"
#include <cassert>
#include <iostream>
#include <limits>

using core3d::planar_sweep::TangentArcBetween;

static double distance(const gp_Pnt2d& a, const gp_Pnt2d& b) {
    return std::hypot(a.X() - b.X(), a.Y() - b.Y());
}
static double lineDistance(const gp_Pnt2d& p, const gp_Pnt2d& a, const gp_Pnt2d& b) {
    return std::abs((p.X()-a.X())*(b.Y()-a.Y())-(p.Y()-a.Y())*(b.X()-a.X())) / distance(a,b);
}
static void close(double a, double b) { assert(std::abs(a-b) < 1e-9); }
static void check(const gp_Pnt2d& a, const gp_Pnt2d& corner, const gp_Pnt2d& b, double radius) {
    const auto result = TangentArcBetween(a,corner,b,radius);
    assert(result);
    const auto& r = *result;
    close(r.radius,radius);
    close(distance(r.trimA,r.center),radius); close(distance(r.trimB,r.center),radius);
    close(lineDistance(r.center,a,corner),radius); close(lineDistance(r.center,corner,b),radius);
    close(lineDistance(r.trimA,a,corner),0); close(lineDistance(r.trimB,corner,b),0);
    assert(distance(r.trimA,corner) <= distance(a,corner));
    assert(distance(r.trimB,corner) <= distance(b,corner));
    assert(std::abs(r.sweepDegrees) > 0 && std::abs(r.sweepDegrees) <= 180);
    const double radians = std::acos(-1.0)/180, side = r.sweepDegrees > 0 ? 1 : -1;
    const double start = r.startDegrees*radians, end = (r.startDegrees+r.sweepDegrees)*radians;
    close(r.center.X()+radius*std::cos(start),r.trimA.X());
    close(r.center.Y()+radius*std::sin(start),r.trimA.Y());
    close(r.center.X()+radius*std::cos(end),r.trimB.X());
    close(r.center.Y()+radius*std::sin(end),r.trimB.Y());
    // Oriented G1 tangents, not merely perpendicular radius vectors.
    close(-side*std::sin(start),(corner.X()-a.X())/distance(a,corner));
    close(side*std::cos(start),(corner.Y()-a.Y())/distance(a,corner));
    close(-side*std::sin(end),(b.X()-corner.X())/distance(b,corner));
    close(side*std::cos(end),(b.Y()-corner.Y())/distance(b,corner));
}

static core3d::ProfileCurveLoop rectangle(double minX) {
    core3d::ProfileCurveLoop loop;
    loop.identifier = 1;
    loop.vertices = {{10,{minX,0}},{11,{20,0}},{12,{20,40}},{13,{minX,40}}};
    for (unsigned i=0;i<4;++i) {
        core3d::ProfileCurveSegment edge;
        edge.identifier=20+i; edge.startVertex=10+i; edge.endVertex=10+(i+1)%4;
        loop.segments.push_back(edge);
    }
    return loop;
}
static void checkInnerAxisBounds() {
    core3d::ProfileCurveSection section;
    section.outer=rectangle(-10);
    core3d::ProfileCurveLoop inner;
    inner.identifier=100;
    inner.vertices={{110,{2,17}},{111,{2,23}}};
    inner.segments={{120,110,111,core3d::ProfileCurveKind::CircularArc,{2,20},3,-90,180},
                    {121,111,110,core3d::ProfileCurveKind::CircularArc,{2,20},3,90,180}};
    section.inner.push_back(inner);
    core3d::ProfileCurveSectionInspection inspection;
    assert(core3d::InspectProfileCurveSection(section,inspection));
    assert(inspection.innerBounds.size()==1);
    // The radial minimum is inside an arc, not at either authored endpoint.
    close(inspection.innerBounds[0][0],-1);
    close(inspection.innerBounds[0][1],5);
    close(inspection.innerBounds[0][2],17);
    close(inspection.innerBounds[0][3],23);
    core3d::ProfileDefinition definition;
    definition.curves=section;
    double area=0,volume=0;
    assert(core3d::ProfileDefinitionExpectedVolume(definition,area,volume));
    definition.revolve=true; definition.depth=360;
    assert(!core3d::ProfileDefinitionExpectedVolume(definition,area,volume));
    definition.curves->outer=rectangle(0);
    assert(!core3d::ProfileDefinitionExpectedVolume(definition,area,volume));
    // Positive control retains an axis-touching outer and a strictly positive hole.
    for (auto& vertex:definition.curves->inner[0].vertices) vertex.point.SetX(10);
    for (auto& edge:definition.curves->inner[0].segments) edge.center.SetX(10);
    assert(core3d::ProfileDefinitionExpectedVolume(definition,area,volume));
    assert(core3d::InspectProfileCurveSection(*definition.curves,inspection));
    close(inspection.innerBounds[0][0],7);
}

static core3d::planar_sweep::Definition arm(double bend) {
    core3d::planar_sweep::Definition d;
    d.pathIdentifier=1; d.plane=1; d.radius=6; d.dimensionMetersPerUnit=.001;
    const auto first=TangentArcBetween({0,0},{0,300+bend},{200,300+bend},bend);
    const auto second=TangentArcBetween({bend,300+bend},{200+bend,300+bend},{200+bend,100},bend);
    assert(first && second);
    d.vertices={{100,{0,0}},{101,first->trimA},{102,first->trimB},
                {103,second->trimA},{104,second->trimB},{105,{200+bend,100}}};
    for (unsigned i=0;i<5;++i) {
        core3d::ProfileCurveSegment edge;
        edge.identifier=200+i; edge.startVertex=100+i; edge.endVertex=101+i;
        if (i==1 || i==3) {
            const auto& arc=i==1 ? *first : *second;
            edge.kind=core3d::ProfileCurveKind::CircularArc;
            edge.center=arc.center; edge.radius=arc.radius;
            edge.startDegrees=arc.startDegrees; edge.sweepDegrees=arc.sweepDegrees;
        }
        d.segments.push_back(edge);
    }
    return d;
}
static void checkArmAdmission() {
    using namespace core3d::planar_sweep;
    Inspection inspection;
    for (double bend : {24.0,40.0,50.0,60.0}) {
        const auto d=arm(bend);
        assert(Inspect(d,inspection)==Admission::Accepted);
        close(inspection.length,700-bend+std::acos(-1.0)*bend);
        close(inspection.expectedVolume,36*std::acos(-1.0)*inspection.length);
    }
    assert(Inspect(arm(23),inspection)==Admission::TightBend);
    auto nonTangent=arm(40); nonTangent.vertices[0].point.SetX(1);
    assert(Inspect(nonTangent,inspection)==Admission::NonTangent);
    auto selfContact=arm(40);
    selfContact.segments[3].sweepDegrees=-180;
    selfContact.vertices[4].point=gp_Pnt2d(200,260);
    selfContact.vertices[5].point=gp_Pnt2d(-100,260);
    assert(Inspect(selfContact,inspection)==Admission::NonlocalContact);
}

int main() {
    checkInnerAxisBounds();
    checkArmAdmission();
    check({0,0},{100,0},{100,100},40);
    check({0,0},{100,0},{100,-100},60);
    check({-50,-50},{50,50},{150,0},12);
    check({0,0},{100,0},{20,60},20);
    check({0,0},{100,0},{180,60},20);
    check({20,30},{20,130},{-80,130},40);
    check({0,0},{40,0},{40,40},40); // Trim exactly at endpoints is allowed.
    assert(!TangentArcBetween({0,0},{100,0},{200,0},10));
    assert(!TangentArcBetween({0,0},{100,0},{0,0},10));
    assert(!TangentArcBetween({0,0},{0,0},{100,0},10));
    assert(!TangentArcBetween({0,0},{100,0},{100,0},10));
    assert(!TangentArcBetween({0,0},{20,0},{20,100},21));
    assert(!TangentArcBetween({0,0},{100,0},{100,20},21));
    for (double r : {0.0,-1.0,std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::quiet_NaN()})
        assert(!TangentArcBetween({0,0},{100,0},{100,100},r));
    assert(!TangentArcBetween({std::numeric_limits<double>::quiet_NaN(),0},{100,0},{100,100},10));
    assert(!TangentArcBetween({0,0},{100,std::numeric_limits<double>::infinity()},{100,100},10));
    std::cout << "PlanarSweepTangentArcTests passed\n";
}
