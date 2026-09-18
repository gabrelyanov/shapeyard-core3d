#include "SavedBooleanWedgeProbe.hxx"
#include <cassert>
#include <iostream>
#include <limits>
using namespace core3d;
int main() {
    for(double unit:{.001,1.})for(double occurrence:{1.,2.}) {
        const double factor=unit*1000, effective=factor*occurrence;
        const auto source=saved_cut_circular_host::probe::Wheel(0,0,unit);
        const auto appended=retained_boolean::Apply(source,retained_boolean::AppendRing{{analytic_boolean::Axis::Z,{},80/factor,10,6}},effective);
        assert(appended);
        const auto original=std::get<retained_boolean::Program>(appended->recipe);
        const auto id=original.steps.back().operand.identifier;
        const auto edited=retained_boolean::Apply(original,retained_boolean::SetRingBoltRadius{id,90*occurrence},effective);
        assert(edited&&edited->changed&&edited->selectedOperandID==id);
        const auto next=std::get<retained_boolean::Program>(edited->recipe);
        const auto& ring=next.steps.back().operand;
        assert(ring.boltCircleRadius==90/factor);
        assert(std::abs(ring.hostRadiusRatio-.6)<1e-15);
        // No other operand/source/identity/codec field is rewritten.
        auto restored=next;restored.steps.back().operand=original.steps.back().operand;
        assert(saved_boolean_ring_probe::Bytes(restored)==appended->newBytes);
        const auto unchanged=retained_boolean::Apply(next,retained_boolean::SetRingBoltRadius{id,ring.boltCircleRadius*effective},effective);
        assert(unchanged&&!unchanged->changed&&unchanged->newBytes==edited->newBytes);
        const auto resized=saved_boolean_build::SourcePatch(next,saved_cut_source_values::CirclePatch{180/factor,{},{}});
        assert(resized&&resized->changed);
        const auto& anchored=std::get<retained_boolean::Program>(resized->recipe).steps.back().operand;
        assert(retained_solid::Bits(anchored.hostRadiusRatio)==retained_solid::Bits(ring.hostRadiusRatio));
        assert(std::abs(anchored.boltCircleRadius*factor-108)<1e-10);
        assert(anchored.radius==ring.radius&&anchored.identifier==ring.identifier&&anchored.count==ring.count);
        for(double bad:{0.,-1.,1e7,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN(),149*occurrence,10*occurrence})
            assert(!retained_boolean::Apply(next,retained_boolean::SetRingBoltRadius{id,bad},effective));
        assert(!retained_boolean::Apply(next,retained_boolean::SetRingBoltRadius{1,90},effective));
        assert(!retained_boolean::Apply(next,retained_boolean::SetRingBoltRadius{999,90},effective));
        assert(saved_boolean_ring_probe::Bytes(next)==edited->newBytes);
    }
    // Independent kernel checks for the dimensions used by the manual B05 rows.
    for(bool open:{false,true}) {
        const auto append=saved_boolean_wedge_probe::Append(saved_boolean_wedge_probe::Loft(),saved_boolean_wedge_probe::Wedge(open));
        assert(append);
        const auto widths=retained_boolean::Apply(append->recipe,retained_boolean::SetWedgeWidths{2,1,open?7.:3.},1);
        assert(widths);
        const auto length=retained_boolean::Apply(widths->recipe,retained_boolean::SetWedgeLength{2,open?50.:12.},1);
        assert(length);
        const double removed=open?120*(56+(6./50)*(2416./3)):120*12*4;
        assert(saved_boolean_wedge_probe::Removed(std::get<retained_boolean::Program>(length->recipe),17280*std::acos(-1.)+removed));
    }
    const auto ring=saved_boolean_ring_probe::Append(saved_cut_circular_host::probe::Wheel());assert(ring);
    const auto wedge=retained_boolean::Apply(ring->recipe,retained_boolean::AppendWedge{{analytic_boolean::Axis::Z,{-130,0,0},0,.5,2,10}},1);
    assert(wedge);
    const auto mixed=retained_boolean::Apply(wedge->recipe,retained_boolean::SetRingBoltRadius{2,90},1);assert(mixed);
    const auto& mixedProgram=std::get<retained_boolean::Program>(mixed->recipe);
    assert(saved_boolean_wedge_probe::Build(mixedProgram).status==saved_boolean_build::Status::Built);
    std::cout<<"Ring bolt-radius edits, scale, refusal, exact no-op and later host anchoring passed\n";
}
