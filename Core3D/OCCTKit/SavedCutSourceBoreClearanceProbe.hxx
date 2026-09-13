#pragma once
// Proposed native probe only: not registered or run. Actual codecs/gp_Trsf and
// interval helper execute; no detached geometry or Boolean result is invented.
#include "SavedCutSourceBoreClearance.hxx"
#include <string>
#include <utility>

namespace core3d::saved_cut_bore_clearance::probe {
using Rows=std::vector<std::pair<std::string,bool>>;
inline retained_solid::Envelope identity(double units){
    retained_solid::Envelope e;
    unsigned i=1;for(auto*p:{&e.document,&e.entity,&e.definition,&e.sourceFeature,&e.derivedFeature})p->fill(std::uint8_t(i++));
    e.metersPerUnit=units;e.radius=1/(units*1000);return e;
}
inline std::array<double,3> point(int plane,double u,double v,double w=0){
    return plane==0?std::array<double,3>{u,v,w}:plane==1?std::array<double,3>{u,w,v}:std::array<double,3>{w,u,v};
}
inline int axis(int plane){return plane==0?2:plane==1?1:0;}
inline retained_solid::Envelope bracket(double units,int plane=1){
    auto e=identity(units);e.sourceFamily=1;e.sourceSchema=1;e.axis=std::uint8_t(axis(plane));const double f=units*1000;
    profile::Parameters p;p.metersPerUnit=units;p.definition.plane=plane;p.definition.depth=8/f;
    for(auto xy:std::array<std::array<double,2>,6>{{{0,0},{60,0},{60,8},{8,8},{8,50},{0,50}}})p.definition.points.emplace_back(xy[0]/f,xy[1]/f);
    profile::Encode(p,e.sourceValues);e.point=point(plane,30/f,4/f);return e;
}
inline retained_solid::Envelope box(double units,int plane=0){
    auto e=identity(units);e.sourceFamily=2;e.sourceSchema=1;e.axis=std::uint8_t(axis(plane));const double f=units*1000;
    enclosure::Parameters p;p.metersPerUnit=units;p.definition.plane=plane;
    p.definition.dimensions={100/f,60/f,30/f,2/f,2/f,4/f};enclosure::Encode(p,e.sourceValues);
    e.point=point(plane,50/f,30/f);return e;
}
inline bool same(const retained_solid::Envelope&a,const retained_solid::Envelope&b){
    std::vector<std::uint8_t>x,y;return retained_solid::Encode(a,x)&&retained_solid::Encode(b,y)&&x==y;
}
inline bool clear(const Report&r,double boundary,double radius,double wall,double mm){
    return r.status==Status::ClearRecipeDisk&&r.boundaryDistanceLowerMM<=boundary+1e-8&&r.boundaryDistanceUpperMM>=boundary-1e-8
        &&std::abs(r.radiusOriginalMM-radius)<1e-10&&std::abs(r.wallIntervalRecipe[1]*mm-wall)<1e-10
        &&r.ligamentLowerMM>boundary-radius-1e-6&&r.ligamentLowerMM<=boundary-radius+1e-8
        &&r.numericUncertaintyMM<=MaximumNumericUncertaintyMM;
}
inline Rows Run(){Rows rows;auto add=[&](std::string s,bool ok){rows.emplace_back(std::move(s),ok);};
    for(double units:{.001,1.0})for(int plane=0;plane<3;++plane){
        const std::string tag=std::to_string(units)+"/"+std::to_string(plane)+"/";const double mm=units*1000;
        auto a=bracket(units,plane);a.point[axis(plane)]=-0.0;a.sourceValues[7]=-0.0;a.sourceValues[8]=-0.0;const auto old=a;
        add(tag+"bracket",clear(Inspect(a),4,1,8,mm)&&same(a,old)&&retained_solid::Bits(a.point[axis(plane)])==retained_solid::Bits(-0.0));
        a.point[axis(plane)]=500/mm;add(tag+"through-all-longitudinal-point",clear(Inspect(a),4,1,8,mm));a=old;
        profile::Parameters p;profile::Decode(a.sourceValues,p);
        p.definition.points[1].SetX(80/mm);p.definition.points[2].SetX(80/mm);
        p.definition.points[4].SetY(70/mm);p.definition.points[5].SetY(70/mm);p.definition.depth=12/mm;
        profile::Encode(p,a.sourceValues);add(tag+"bracket-explicit-outline-depth",clear(Inspect(a),4,1,12,mm));
        p.definition.depth=6/mm;profile::Encode(p,a.sourceValues);add(tag+"bracket-depth-shrink",clear(Inspect(a),4,1,6,mm));
        a=old;a.point=point(plane,30/mm,.5/mm);add(tag+"notch",Inspect(a).status==Status::OutsideOrInsufficientLigament);
        a=old;a.point=point(plane,30/mm,1/mm);add(tag+"tangent",Inspect(a).status==Status::OutsideOrInsufficientLigament);
        a=old;a.point=point(plane,30/mm,1.001/mm);add(tag+"ligament-below-kernel",Inspect(a).status==Status::OutsideOrInsufficientLigament);
        a=old;a.point=point(plane,30/mm,1.003/mm);add(tag+"ligament-above-kernel",Inspect(a).status==Status::ClearRecipeDisk);
        a=old;a.point=point(plane,30/mm,30/mm);add(tag+"concave-outside-even-in-bounds",Inspect(a).status==Status::OutsideOrInsufficientLigament);
        a=old;a.axis=std::uint8_t((a.axis+1)%3);add(tag+"wrong-axis",Inspect(a).status==Status::Nonparallel);
        auto e=box(units,plane);add(tag+"enclosure",clear(Inspect(e),28,1,2,mm));
        enclosure::Parameters ep;enclosure::Decode(int(e.sourceSchema),e.sourceValues,ep);
        ep.definition.dimensions.width=120/mm;enclosure::Encode(ep,e.sourceValues);add(tag+"enclosure-width-origin-stays",clear(Inspect(e),28,1,2,mm));
        ep.definition.dimensions.depth=80/mm;ep.definition.dimensions.floor=3/mm;enclosure::Encode(ep,e.sourceValues);add(tag+"enclosure-depth-floor",clear(Inspect(e),28,1,3,mm));
        ep.definition.dimensions.width=50/mm;enclosure::Encode(ep,e.sourceValues);add(tag+"enclosure-width-crosses-bore",Inspect(e).status==Status::OutsideOrInsufficientLigament);
        e=box(units,plane);e.point=point(plane,3/mm,3/mm);e.radius=.7/mm;add(tag+"full-rounded-corner-not-bounds",Inspect(e).status==Status::OutsideOrInsufficientLigament);
        e.radius=.58/mm;add(tag+"rounded-corner-clear",Inspect(e).status==Status::ClearRecipeDisk);
        e.radius=.584/mm;add(tag+"rounded-corner-kernel-refusal",Inspect(e).status==Status::OutsideOrInsufficientLigament);
        e.radius=(2-std::sqrt(2.0))/mm;add(tag+"rounded-corner-tangent",Inspect(e).status==Status::OutsideOrInsufficientLigament);
        for(double scale:{.5,1.0,2.0}){
            a=old;profile::Decode(a.sourceValues,p);profile::ConstructionFrame f;
            // Nonorigin rotation ABOUT extrusion preserves original X/Y/Z tool.
            const double angle=std::acos(-1.0)/8;f.values={11/mm,-7/mm,9/mm,0,0,0,std::cos(angle),scale};f.values[3+axis(plane)]=std::sin(angle);
            p.constructionFrame=f;profile::Encode(p,a.sourceValues);a.sourceSchema=2;gp_Trsf tr;f.Transform(tr);
            gp_Pnt c(old.point[0],old.point[1],old.point[2]);c.Transform(tr);a.point={c.X(),c.Y(),c.Z()};
            const auto saved=a;add(tag+"frame-once-scale-"+std::to_string(scale),clear(Inspect(a),4*scale,1,8,mm)&&same(a,saved));
            // Radius stays original-local1mm: accidental second F factor fails.
            p.constructionFrame->values[7]=-scale;profile::Encode(p,a.sourceValues);add(tag+"reflected-"+std::to_string(scale),Inspect(a).status==Status::UnsupportedFrame);
        }
        e=box(units,plane);enclosure::Decode(int(e.sourceSchema),e.sourceValues,ep);
        profile::ConstructionFrame ef;ef.values={5/mm,-3/mm,7/mm,0,0,0,std::cos(.2),2};ef.values[3+axis(plane)]=std::sin(.2);
        ep.definition.constructionFrame=ef;enclosure::Encode(ep,e.sourceValues);e.sourceSchema=2;
        gp_Trsf et;ef.Transform(et);gp_Pnt ec(e.point[0],e.point[1],e.point[2]);ec.Transform(et);e.point={ec.X(),ec.Y(),ec.Z()};
        const auto savedEnclosure=e;add(tag+"enclosure-frame-once",clear(Inspect(e),56,1,2,mm)&&same(e,savedEnclosure));
        ep.definition.constructionFrame->values[7]=-2;enclosure::Encode(ep,e.sourceValues);
        add(tag+"enclosure-reflected",Inspect(e).status==Status::UnsupportedFrame);
        a=old;profile::Decode(a.sourceValues,p);profile::ConstructionFrame tilted;
        tilted.values[3+(axis(plane)+1)%3]=std::sin(.1);tilted.values[6]=std::cos(.1);p.constructionFrame=tilted;
        profile::Encode(p,a.sourceValues);a.sourceSchema=2;add(tag+"oblique-frame",Inspect(a).status==Status::Nonparallel);
        a=old;a.metersPerUnit=0;add(tag+"bad-unit",Inspect(a).status==Status::RefusedValues);
        a=old;a.metersPerUnit*=2;add(tag+"unit-does-not-match-recipe",Inspect(a).status==Status::RefusedValues);
        a=old;a.point=point(plane,30/mm,1.002/mm);add(tag+"strict-kernel-boundary",Inspect(a).status!=Status::ClearRecipeDisk);
        e=box(units,plane);e.sourceValues[6]=e.sourceValues[4];add(tag+"floor-reaches-top",Inspect(e).status==Status::RefusedValues);
        a=old;profile::Decode(a.sourceValues,p);p.constructionFrame=profile::ConstructionFrame{};
        profile::Encode(p,a.sourceValues);a.sourceSchema=2;a.sourceValues[a.sourceValues.size()-2]=2;
        add(tag+"invalid-frame-quaternion",Inspect(a).status==Status::RefusedValues);
        a=old;a.point[0]=std::numeric_limits<double>::infinity();add(tag+"nonfinite-tool",Inspect(a).status==Status::RefusedValues);
        a=old;a.sourceValues.back()=std::numeric_limits<double>::quiet_NaN();add(tag+"malformed-source",Inspect(a).status==Status::RefusedValues);
    }return rows;
}
}
