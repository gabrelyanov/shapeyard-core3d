#pragma once
#include "AnalyticBooleanOperand.hxx"
#include "SavedCutResultBoundaryExpectation.hxx"
#include <cfenv>
#include <map>
#include <set>

namespace core3d::analytic_boolean_wedge {
inline constexpr double MinimumHalfWidthMM=.05,MinimumLengthMM=.1,MaximumDimensionMM=1e6;
struct Wedge {
    std::array<double,3> point{};
    double directionAngle=0,halfWidthApex=0,halfWidthMouth=0,length=0;
};
enum class Status : std::uint8_t { Clear, InvalidWidths, InvalidLength, InvalidAngle,
    NonConvexOrDegenerate, OutsideDomain, WrongAxis, OutsideOrInsufficientLigament,
    UnsupportedConfiguration, NumericUncertain };
inline Wedge FromOperand(const analytic_boolean::Operand& t){
    return {t.point,t.directionAngle,t.halfWidthApex,t.halfWidthMouth,t.length};
}
inline Status Inspect(const Wedge& w,analytic_boolean::Axis axis,double metersPerUnit) noexcept {
    const double mm=metersPerUnit*1000;
    if(!std::isfinite(mm)||mm<=0||unsigned(axis)>2)return Status::OutsideDomain;
    for(double value:w.point)if(!std::isfinite(value)||!std::isfinite(value*mm)||std::abs(value*mm)>1e6)return Status::OutsideDomain;
    if(!std::isfinite(w.directionAngle)||w.directionAngle<0||w.directionAngle>=2*std::acos(-1.0))return Status::InvalidAngle;
    for(double value:{w.halfWidthApex,w.halfWidthMouth})if(!std::isfinite(value)||!std::isfinite(value*mm)
        ||value*mm<MinimumHalfWidthMM||value*mm>MaximumDimensionMM)return Status::InvalidWidths;
    if(!std::isfinite(w.length)||!std::isfinite(w.length*mm)||w.length*mm<MinimumLengthMM||w.length*mm>MaximumDimensionMM)return Status::InvalidLength;
    return Status::Clear;
}
using SectionPoints=std::array<std::array<double,2>,4>;
inline SectionPoints Section(const Wedge& w,analytic_boolean::Axis axis) noexcept {
    const unsigned u=(unsigned(axis)+1)%3,v=(unsigned(axis)+2)%3;
    const double c=std::cos(w.directionAngle),s=std::sin(w.directionAngle);
    SectionPoints points{};const std::array<double,4> along{0,w.length,w.length,0};
    const std::array<double,4> across{-w.halfWidthApex,-w.halfWidthMouth,w.halfWidthMouth,w.halfWidthApex};
    for(unsigned k=0;k<4;++k)points[k]={w.point[u]+along[k]*c-across[k]*s,w.point[v]+along[k]*s+across[k]*c};
    return points;
}
inline SectionPoints Expand(const analytic_boolean::Operand& t) noexcept {return Section(FromOperand(t),t.axis);}
// The band integral is exact for affine clipping position s(z). This also
// covers a prism (equal endpoints); closed slots use the full section area.
inline double BandVolume(const Wedge& w,double s0,double s1,double height) noexcept {
    return height*(w.halfWidthApex*(s0+s1)+(w.halfWidthMouth-w.halfWidthApex)/w.length
        *(s0*s0+s0*s1+s1*s1)/3);
}
inline double ExpectedRemovedVolume(const Wedge& w,double height) noexcept {
    return (w.halfWidthApex+w.halfWidthMouth)*w.length*height;
}
namespace detail {
namespace i=saved_cut_bore_clearance::detail;
using Expected=saved_cut_whole_result::Expected;
using Use=enclosure_correspondence::EdgeUse;
using Edge=enclosure_correspondence::ExpectedEdge;
using Face=enclosure_correspondence::ExpectedFace;
inline bool Clear(i::I gap,double mm){
    gap=i::mul(gap,i::I(mm));return i::good(gap)&&gap.lo>saved_cut_bore_clearance::KernelSeparationMM
        &&i::up(gap.hi-gap.lo)<=saved_cut_bore_clearance::MaximumNumericUncertaintyMM;
}
inline i::I Cross(i::I ax,i::I ay,i::I bx,i::I by){return i::sub(i::mul(ax,by),i::mul(ay,bx));}
inline std::vector<gp_Pnt2d> Polygon(const SectionPoints& s){std::vector<gp_Pnt2d> p;for(const auto& x:s)p.emplace_back(x[0],x[1]);return p;}
// A separating-axis certificate covers crossings and containment as well as
// endpoint distances; crossed rectangles with no contained vertex must refuse.
inline bool SeparatedPolygons(const SectionPoints& a,const SectionPoints& b,double mm){
    for(const auto* p:{&a,&b})for(unsigned k=0;k<4;++k){
        const auto& x=(*p)[k];const auto& y=(*p)[(k+1)%4];
        const auto nx=i::sub(i::I(x[1]),i::I(y[1])),ny=i::sub(i::I(y[0]),i::I(x[0]));
        const auto norm=i::norm(nx,ny);if(!i::good(norm)||norm.lo<=0)return false;
        const auto range=[&](const SectionPoints& points){
            auto lo=i::add(i::mul(i::I(points[0][0]),nx),i::mul(i::I(points[0][1]),ny)),hi=lo;
            for(unsigned j=1;j<4;++j){const auto q=i::add(i::mul(i::I(points[j][0]),nx),i::mul(i::I(points[j][1]),ny));lo=i::minimum(lo,q);hi=i::maximum(hi,q);}return std::make_pair(lo,hi);};
        const auto ra=range(a),rb=range(b);
        if(Clear(i::div(i::sub(ra.first,rb.second),norm),mm)||Clear(i::div(i::sub(rb.first,ra.second),norm),mm))return true;
    }return false;
}
inline bool SeparatedDisk(const SectionPoints& section,const std::array<double,2>& center,double radius,double mm){
    const auto polygon=Polygon(section);
    if(i::inside(polygon,center[0],center[1]))return false;
    auto distance=i::I(std::numeric_limits<double>::max());
    for(unsigned k=0;k<4;++k)distance=i::minimum(distance,i::segmentDistance(i::I(center[0]),i::I(center[1]),polygon[k],polygon[(k+1)%4]));
    return Clear(i::sub(distance,i::I(radius)),mm);
}
}
} // namespace core3d::analytic_boolean_wedge

namespace core3d::analytic_boolean_wedge {
struct Admission {
    Status status=Status::UnsupportedConfiguration;
    bool open=false;
    double removedVolume=0;
    unsigned bands=0;
};
namespace detail {
struct Segment {unsigned start=0,end=0;int original=-1;bool originalForward=true;};
using Loop=std::vector<Segment>;
struct FaceLoops {Face face;std::vector<Loop> loops;};
inline Loop VerticesLoop(const std::vector<unsigned>& vertices){
    Loop loop;for(unsigned k=0;k<vertices.size();++k)loop.push_back({vertices[k],vertices[(k+1)%vertices.size()],-1});return loop;
}
inline void Reverse(Loop& loop){std::reverse(loop.begin(),loop.end());for(auto& edge:loop)std::swap(edge.start,edge.end);}
inline Loop ReadLoop(const std::vector<Use>& wire,const Expected& expected){
    Loop loop;for(const auto& use:wire){const auto& edge=expected.edges.at(use.edge);
        loop.push_back({use.forward?edge.start:edge.end,use.forward?edge.end:edge.start,int(use.edge),use.forward});}return loop;
}
inline bool Assemble(const Expected& original,const std::vector<FaceLoops>& faces,Expected& out){
    out.edges.clear();out.faces.clear();std::map<std::pair<unsigned,unsigned>,unsigned> lines;
    std::map<unsigned,unsigned> circles;
    for(const auto& entry:faces){Face f=entry.face;f.wires.clear();
        for(const auto& loop:entry.loops){std::vector<Use> wire;
            if(loop.empty())return false;
            for(unsigned k=0;k<loop.size();++k){const auto& s=loop[k];
                if(s.end!=loop[(k+1)%loop.size()].start)return false;
                unsigned id=0;bool forward=false;
                if(s.original>=0&&original.edges.at(unsigned(s.original)).circle){
                    const auto inserted=circles.emplace(unsigned(s.original),unsigned(out.edges.size()));id=inserted.first->second;
                    if(inserted.second)out.edges.push_back(original.edges[unsigned(s.original)]);
                    forward=s.start==out.edges[id].start;
                    // A full-circle edge has identical endpoints: retain its
                    // original direction through the unmodified source wire.
                    if(s.start==s.end)forward=s.originalForward;
                }else{
                    if(s.start==s.end)return false;const auto key=std::minmax(s.start,s.end);
                    const auto inserted=lines.emplace(key,unsigned(out.edges.size()));id=inserted.first->second;
                    if(inserted.second){Edge edge;edge.start=key.first;edge.end=key.second;out.edges.push_back(edge);}
                    forward=s.start==out.edges[id].start;
                }wire.push_back({id,forward});
            }f.wires.push_back(std::move(wire));
        }out.faces.push_back(std::move(f));
    }
    std::vector<std::array<unsigned,2>> uses(out.edges.size());
    for(const auto& f:out.faces)for(const auto& wire:f.wires)for(const auto& use:wire)++uses[use.edge][use.forward?0:1];
    for(const auto& use:uses)if(use[0]!=1||use[1]!=1)return false;return true;
}
inline bool Cycles(const Loop& segments,std::vector<Loop>& loops){
    std::map<unsigned,unsigned> next;std::set<unsigned> incoming;
    for(unsigned k=0;k<segments.size();++k)if(!next.emplace(segments[k].start,k).second||!incoming.insert(segments[k].end).second)return false;
    std::set<unsigned> used;
    for(unsigned k=0;k<segments.size();++k){if(used.count(k))continue;Loop loop;unsigned j=k;
        do{if(!used.insert(j).second)return false;loop.push_back(segments[j]);const auto it=next.find(segments[j].end);if(it==next.end())return false;j=it->second;}while(j!=k);
        if(loop.size()<3)return false;loops.push_back(std::move(loop));
    }return used.size()==segments.size();
}
inline bool Wall(Expected& out,std::vector<FaceLoops>& faces,std::vector<unsigned> ids,const gp_Vec& inward){
    if(ids.size()<4)return false;FaceLoops f;f.face.origin=out.vertices[ids[0]];f.face.normalOrAxis=inward;
    auto loop=VerticesLoop(ids);gp_Vec normal;
    for(unsigned j=1;j+1<ids.size();++j)normal+=gp_Vec(out.vertices[ids[0]],out.vertices[ids[j]]).Crossed(gp_Vec(out.vertices[ids[0]],out.vertices[ids[j+1]]));
    if(!std::isfinite(normal.Dot(inward))||normal.Dot(inward)==0)return false;
    if(normal.Dot(inward)<0)Reverse(loop);f.loops.push_back(std::move(loop));faces.push_back(std::move(f));return true;
}
struct Station {double z=0,s=0;std::vector<unsigned> vertices;unsigned crossed=0;std::array<unsigned,2> clips{};};
inline bool PolygonClear(const std::vector<gp_Pnt2d>& host,const SectionPoints& section,double mm,int except=-1){
    for(unsigned v=0;v<4;++v){
        if(except<0&&!i::inside(host,section[v][0],section[v][1]))return false;
        for(unsigned edge=0;edge<host.size();++edge){if(int(edge)==except)continue;
            if(!Clear(i::segmentDistance(i::I(section[v][0]),i::I(section[v][1]),host[edge],host[(edge+1)%host.size()]),mm))return false;
        }
    }
    for(const auto& point:host)for(unsigned k=0;k<4;++k){
        const gp_Pnt2d a(section[k][0],section[k][1]),b(section[(k+1)%4][0],section[(k+1)%4][1]);
        if(!Clear(i::segmentDistance(i::I(point.X()),i::I(point.Y()),a,b),mm))return false;
    }
    // A concave source can cross a tool edge although all tool vertices are
    // inside. Reject every proper or uncertain boundary intersection.
    for(unsigned edge=0;edge<host.size();++edge){if(int(edge)==except)continue;
        const auto& a=host[edge];const auto& b=host[(edge+1)%host.size()];
        for(unsigned k=0;k<4;++k){const auto& c=section[k];const auto& d=section[(k+1)%4];
            const auto side=[&](double x,double y,double ax,double ay,double bx,double by){return Cross(i::sub(i::I(bx),i::I(ax)),i::sub(i::I(by),i::I(ay)),i::sub(i::I(x),i::I(ax)),i::sub(i::I(y),i::I(ay)));};
            const auto ac=side(c[0],c[1],a.X(),a.Y(),b.X(),b.Y()),ad=side(d[0],d[1],a.X(),a.Y(),b.X(),b.Y());
            const auto ca=side(a.X(),a.Y(),c[0],c[1],d[0],d[1]),cb=side(b.X(),b.Y(),c[0],c[1],d[0],d[1]);
            const auto same=[](i::I x,i::I y){return i::good(x)&&i::good(y)&&((x.lo>0&&y.lo>0)||(x.hi<0&&y.hi<0));};
            if(!same(ac,ad)&&!same(ca,cb))return false;
        }
    }return true;
}
}
// Derive all split edges, cap notches and wall cycles from source recipe
// vertices. No Boolean output, observed face count or volume enters admission.
inline Admission ExpectedBoundary(const retained_solid::Envelope& source,const analytic_boolean::Operand& tool,
    detail::Expected& out) noexcept {
    using namespace detail;Admission report;
    try {
        if(Inspect(FromOperand(tool),tool.axis,source.metersPerUnit)!=Status::Clear){report.status=Status::OutsideDomain;return report;}
        if(std::fegetround()!=FE_TONEAREST){report.status=Status::NumericUncertain;return report;}
        const Expected original=out;Expected sourceExpected;
        if(!saved_cut_whole_result::ExpectedSource(source,sourceExpected))return report;
        const double mm=source.metersPerUnit*1000;
        const unsigned axis=unsigned(tool.axis),u=(axis+1)%3,v=(axis+2)%3;
        gp_Vec axial;axial.SetCoord(axis+1,1);
        for(unsigned cap:original.caps){const auto& f=original.faces.at(cap);
            if(f.cylinder||f.normalOrAxis.Magnitude()<=0||f.normalOrAxis.Crossed(axial).Magnitude()>1e-12*f.normalOrAxis.Magnitude()){
                report.status=Status::WrongAxis;return report;}}
        const auto wedge=FromOperand(tool);const auto section=Section(wedge,tool.axis);
        const double c=std::cos(wedge.directionAngle),s=std::sin(wedge.directionAngle);
        gp_Vec direction,normal;direction.SetCoord(u+1,c);direction.SetCoord(v+1,s);normal.SetCoord(u+1,-s);normal.SetCoord(v+1,c);
        std::vector<Station> stations;
        if(source.sourceFamily==3){
            rectangular_loft::Definition loft;if(!loft_persistence::Decode(source.sourceValues,loft)||sourceExpected.vertices.size()!=4*loft.stations.size())return report;
            for(unsigned k=0;k<loft.stations.size();++k){Station station;for(unsigned j=0;j<4;++j)station.vertices.push_back(4*k+j);
                station.z=original.vertices[4*k].Coord(axis+1);stations.push_back(std::move(station));}
        }else if(source.sourceFamily==1){
            profile::Parameters p;if(!profile::Decode(source.sourceValues,p))return report;
            if(!p.definition.circle){
            for(unsigned cap:sourceExpected.caps){Station station;const auto& face=sourceExpected.faces[cap];if(face.wires.size()!=1)return report;
                for(const auto& use:face.wires[0]){const auto& e=sourceExpected.edges[use.edge];if(e.circle)return report;station.vertices.push_back(use.forward?e.start:e.end);}
                station.z=original.vertices[station.vertices[0]].Coord(axis+1);stations.push_back(std::move(station));}}
        }else if(source.sourceFamily!=2)return report;
        const bool curved=stations.empty();
        if(curved){
            // The enclosure cavity and outer circle are convex. Every vertex
            // clears them; for annuli the complete tool must also avoid the
            // inner disk, not merely have four exterior vertices.
            for(const auto& point:section){auto view=source;view.point=tool.point;view.point[u]=point[0];view.point[v]=point[1];view.radius=.001/mm;
                const auto clear=saved_cut_bore_clearance::Inspect(view);
                const double distance=clear.ligamentLowerMM+clear.radiusOriginalMM;
                if(clear.recipeUnitToOriginalMM<=0||!std::isfinite(distance)||distance<=saved_cut_bore_clearance::KernelSeparationMM
                    ||clear.numericUncertaintyMM>saved_cut_bore_clearance::MaximumNumericUncertaintyMM)return report;}
            if(source.sourceFamily==1)for(const auto& face:sourceExpected.faces)if(face.cylinder&&face.radialSign<0)
                if(!SeparatedDisk(section,{face.origin.Coord(u+1),face.origin.Coord(v+1)},face.radius,mm))return report;
            for(unsigned cap:sourceExpected.caps){Station station;station.z=sourceExpected.faces[cap].origin.Coord(axis+1);stations.push_back(station);}
        }
        std::sort(stations.begin(),stations.end(),[](const Station& a,const Station& b){return a.z<b.z;});
        if(stations.size()<2)return report;
        bool closed=true;
        for(auto& station:stations){if(curved)continue;std::vector<gp_Pnt2d> polygon;
            for(unsigned id:station.vertices){const auto& p=original.vertices[id];
                if(std::abs(p.Coord(axis+1)-station.z)*mm>1e-9)return report;polygon.emplace_back(p.Coord(u+1),p.Coord(v+1));}
            if(PolygonClear(polygon,section,mm))continue;closed=false;
            if(!i::inside(polygon,section[0][0],section[0][1])||!i::inside(polygon,section[3][0],section[3][1]))return report;
            unsigned matches=0;
            for(unsigned edge=0;edge<polygon.size();++edge){const auto& a=polygon[edge];const auto& b=polygon[(edge+1)%polygon.size()];
                const double dx=b.X()-a.X(),dy=b.Y()-a.Y(),norm=std::hypot(dx,dy);
                if(norm<=0||std::abs(dx*c+dy*s)>norm*1e-12)continue;
                const auto distance=i::add(i::mul(i::sub(i::I(a.X()),i::I(tool.point[u])),i::I(c)),i::mul(i::sub(i::I(a.Y()),i::I(tool.point[v])),i::I(s)));
                if(!Clear(distance,mm)||!Clear(i::sub(i::I(wedge.length),distance),mm))continue;
                const double clip=i::midpoint(distance),width=wedge.halfWidthApex+(wedge.halfWidthMouth-wedge.halfWidthApex)*clip/wedge.length;
                auto clipped=section;clipped[1]={tool.point[u]+clip*c+width*s,tool.point[v]+clip*s-width*c};
                clipped[2]={tool.point[u]+clip*c-width*s,tool.point[v]+clip*s+width*c};
                if(!PolygonClear(polygon,clipped,mm,int(edge)))continue;
                // Crossings must lie strictly within the segment, not its extension.
                const auto length2=i::add(i::sq(i::sub(i::I(b.X()),i::I(a.X()))),i::sq(i::sub(i::I(b.Y()),i::I(a.Y()))));
                const auto along=[&](const std::array<double,2>& point){return i::add(
                    i::mul(i::sub(i::I(point[0]),i::I(a.X())),i::sub(i::I(b.X()),i::I(a.X()))),
                    i::mul(i::sub(i::I(point[1]),i::I(a.Y())),i::sub(i::I(b.Y()),i::I(a.Y()))));};
                const auto ta=along(clipped[1]),tb=along(clipped[2]);
                const auto gap=i::minimum(i::minimum(ta,tb),i::minimum(i::sub(length2,ta),i::sub(length2,tb)));
                if(!Clear(i::div(gap,i::root(length2)),mm))continue;
                unsigned edgeMatches=0;
                for(unsigned j=0;j<original.edges.size();++j){const auto& e=original.edges[j];if(e.circle)continue;
                    const auto& x=original.vertices[e.start];const auto& y=original.vertices[e.end];
                    if(std::abs(x.Coord(axis+1)-station.z)*mm>1e-9||std::abs(y.Coord(axis+1)-station.z)*mm>1e-9)continue;
                    const gp_Pnt2d aa(x.Coord(u+1),x.Coord(v+1)),bb(y.Coord(u+1),y.Coord(v+1));
                    const auto d1=i::segmentDistance(i::I(clipped[1][0]),i::I(clipped[1][1]),aa,bb);
                    const auto d2=i::segmentDistance(i::I(clipped[2][0]),i::I(clipped[2][1]),aa,bb);
                    if(i::good(d1)&&i::good(d2)&&std::max(d1.hi,d2.hi)*mm<=1e-9){station.crossed=j;++edgeMatches;}
                }
                if(edgeMatches!=1)return report;station.s=clip;++matches;
            }if(matches!=1)return report;
        }
        if(!closed)for(const auto& station:stations)if(station.s<=0)return report;
        Expected next=original;std::vector<FaceLoops> faces;
        const auto vertex=[&](double along,double across,double z){gp_Pnt p(tool.point[0],tool.point[1],tool.point[2]);p.SetCoord(axis+1,z);
            p.SetCoord(u+1,tool.point[u]+along*c-across*s);p.SetCoord(v+1,tool.point[v]+along*s+across*c);
            const unsigned id=unsigned(next.vertices.size());next.vertices.push_back(p);return id;};
        std::array<std::array<unsigned,4>,2> capPoints{};
        const double bottom=stations.front().z,top=stations.back().z;
        if(closed){
            for(unsigned level=0;level<2;++level){const double z=level?top:bottom;
                capPoints[level]={vertex(0,-wedge.halfWidthApex,z),vertex(wedge.length,-wedge.halfWidthMouth,z),
                    vertex(wedge.length,wedge.halfWidthMouth,z),vertex(0,wedge.halfWidthApex,z)};}
            for(unsigned f=0;f<original.faces.size();++f){FaceLoops entry;entry.face=original.faces[f];
                for(const auto& wire:entry.face.wires)entry.loops.push_back(ReadLoop(wire,original));
                if(f==original.caps[0]||f==original.caps[1]){const unsigned level=entry.face.origin.Coord(axis+1)>bottom+(top-bottom)/2;
                    const auto& ids=capPoints[level];auto loop=VerticesLoop({ids[0],ids[1],ids[2],ids[3]});
                    if(entry.face.normalOrAxis.Dot(axial)>0)Reverse(loop);entry.loops.push_back(std::move(loop));}
                faces.push_back(std::move(entry));}
            for(unsigned k=0;k<4;++k){const unsigned j=(k+1)%4;
                const auto inward=axial.Crossed(gp_Vec(next.vertices[capPoints[0][k]],next.vertices[capPoints[0][j]]));
                if(!Wall(next,faces,{capPoints[0][k],capPoints[0][j],capPoints[1][j],capPoints[1][k]},inward))return report;}
            ++next.hostGenus;report.removedVolume=ExpectedRemovedVolume(wedge,top-bottom);
        }else{
            const std::array<std::array<unsigned,2>,2> apex{{{vertex(0,-wedge.halfWidthApex,bottom),vertex(0,wedge.halfWidthApex,bottom)},
                {vertex(0,-wedge.halfWidthApex,top),vertex(0,wedge.halfWidthApex,top)}}};
            std::map<unsigned,unsigned> crossed;
            for(unsigned k=0;k<stations.size();++k){auto& st=stations[k];const double width=wedge.halfWidthApex+(wedge.halfWidthMouth-wedge.halfWidthApex)*st.s/wedge.length;
                st.clips={vertex(st.s,-width,st.z),vertex(st.s,width,st.z)};if(!crossed.emplace(st.crossed,k).second)return report;}
            for(unsigned f=0;f<original.faces.size();++f){FaceLoops entry;entry.face=original.faces[f];unsigned intersections=0;
                for(const auto& wire:entry.face.wires)for(const auto& use:wire)intersections+=crossed.count(use.edge);
                if(!intersections){for(const auto& wire:entry.face.wires)entry.loops.push_back(ReadLoop(wire,original));faces.push_back(std::move(entry));continue;}
                if(entry.face.cylinder)return report;
                std::vector<Loop> untouched;Loop segments;std::vector<std::pair<unsigned,unsigned>> gaps;
                for(const auto& wire:entry.face.wires){
                    bool pierced=false;for(const auto& use:wire)pierced=pierced||crossed.count(use.edge);
                    if(!pierced){untouched.push_back(ReadLoop(wire,original));continue;}
                    for(const auto& use:wire){const auto& e=original.edges[use.edge];const unsigned a=use.forward?e.start:e.end,b=use.forward?e.end:e.start;
                    const auto found=crossed.find(use.edge);if(found==crossed.end()){segments.push_back({a,b,int(use.edge)});continue;}
                    const auto& st=stations[found->second];const bool firstPositive=gp_Vec(next.vertices[st.clips[0]],original.vertices[a]).Dot(normal)>0;
                    const unsigned first=st.clips[firstPositive?1:0],second=st.clips[firstPositive?0:1];
                    segments.push_back({a,first,-1});segments.push_back({second,b,-1});gaps.emplace_back(first,second);
                }}
                if(f==original.caps[0]||f==original.caps[1]){
                    if(intersections!=1||gaps.size()!=1)return report;const unsigned level=entry.face.origin.Coord(axis+1)>bottom+(top-bottom)/2;
                    const bool positive=gaps[0].first==stations[level?stations.size()-1:0].clips[1];const unsigned a=apex[level][positive?1:0],b=apex[level][positive?0:1];
                    segments.push_back({gaps[0].first,a,-1});segments.push_back({a,b,-1});segments.push_back({b,gaps[0].second,-1});
                }else{
                    if(intersections!=2||gaps.size()!=2)return report;
                    segments.push_back({gaps[0].first,gaps[1].second,-1});segments.push_back({gaps[1].first,gaps[0].second,-1});
                }
                std::vector<Loop> loops;if(!Cycles(segments,loops)||loops.size()!=(intersections==1?1u:2u))return report;
                if(!untouched.empty()&&intersections!=1)return report;
                for(auto& loop:loops){FaceLoops split;split.face=entry.face;split.loops.push_back(std::move(loop));
                    for(const auto& oldLoop:untouched)split.loops.push_back(oldLoop);faces.push_back(std::move(split));}
            }
            // Cap indices can move when an earlier side face splits. Derive
            // their indices by their unique source plane and normal.
            for(unsigned k=0;k<2;++k){unsigned matches=0;
                for(unsigned f=0;f<faces.size();++f)if(faces[f].face.origin.IsEqual(original.faces[original.caps[k]].origin,0)
                    &&faces[f].face.normalOrAxis.IsEqual(original.faces[original.caps[k]].normalOrAxis,0,0)){next.caps[k]=f;++matches;}
                if(matches!=1)return report;}
            if(!Wall(next,faces,{apex[0][0],apex[0][1],apex[1][1],apex[1][0]},direction))return report;
            for(unsigned side=0;side<2;++side){std::vector<unsigned> ids{apex[0][side]};for(const auto& st:stations)ids.push_back(st.clips[side]);ids.push_back(apex[1][side]);
                gp_Vec along=direction*wedge.length+normal*((side?1:-1)*(wedge.halfWidthMouth-wedge.halfWidthApex));
                const gp_Vec inward=axial.Crossed(along)*(side?-1:1);if(!Wall(next,faces,std::move(ids),inward))return report;}
            for(unsigned k=1;k<stations.size();++k)report.removedVolume+=BandVolume(wedge,stations[k-1].s,stations[k].s,stations[k].z-stations[k-1].z);
        }
        if(!Assemble(original,faces,next))return report;
        report.bands=unsigned(stations.size()-1);const unsigned df=closed?4:report.bands+3,de=closed?12:3*report.bands+9,dv=closed?8:2*report.bands+6;
        if(next.faces.size()!=original.faces.size()+df||next.edges.size()!=original.edges.size()+de||next.vertices.size()!=original.vertices.size()+dv
            ||!std::isfinite(report.removedVolume)||report.removedVolume<=0)return report;
        report.open=!closed;report.status=Status::Clear;out=std::move(next);return report;
    }catch(...){return report;}
}
} // namespace core3d::analytic_boolean_wedge
