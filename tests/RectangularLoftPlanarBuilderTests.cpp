// Detached native regression: links retained macOS OCCT; no document or simulator.
#include "RectangularLoftSolid.hxx"
#include "DetachedRectangularLoftProbe.hxx"
#include "SavedCutEnclosureExtractor.hxx"
#include <TopExp_Explorer.hxx>
#include <iostream>
#include <stdexcept>

namespace loft=core3d::rectangular_loft;
namespace reader=core3d::enclosure_correspondence;
static void require(bool value,const char* reason) {
    if (!value) throw std::runtime_error(reason);
}
static loft::Definition fixture(int count,double unit,int kind,int frame) {
    auto d=loft::probe::Fixture(3,unit);
    d.stations.resize(count);
    const double k=0.001/unit;
    for (int i=0;i<count;++i) {
        auto& s=d.stations[i];s.z=i*20*k;
        // kind 0: translated rectangles, kind 1: coplanar station seams,
        // kind 2: alternating trapezoids, including equal-width neighbours.
        s.centerX=(kind==1?0:i*2)*k;s.centerY=(kind==1?0:(i%3-1)*3)*k;
        s.width=(kind==2?20+(i/2%2)*6:20)*k;
        s.depth=(kind==2?12+(i%2)*4:12)*k;
    }
    if (frame) {
        core3d::profile::ConstructionFrame f;
        f.values={13*k,-17*k,29*k,0.2,-0.4,0.4,0.8,frame==1?1.5:-2};
        d.constructionFrame=f;
    }
    return d;
}
static void check(const loft::Definition& d) {
    loft::Inspection inspection;require(loft::Inspect(d,inspection)==loft::Admission::Accepted,"inspection");
    const auto original=loft::probe::Snapshot(d);
    loft::Admission admission;auto prepared=loft::Prepare(d,admission);
    std::atomic_bool stop{false};loft::SolidResult out;
    const auto status=loft::Build(prepared,stop,out);
    if(status!=loft::BuildStatus::Built) {
        std::cerr<<"build status "<<loft::probe::Name(status)<<" stations="<<d.stations.size()
                 <<" unit="<<d.dimensionMetersPerUnit<<" width="<<d.stations[0].width<<" frame="<<bool(d.constructionFrame)<<'\n';
    }
    require(status==loft::BuildStatus::Built,"build");
    require(original==loft::probe::Snapshot(prepared->definition),"immutable recipe");
    require(BRepCheck_Analyzer(out.solid,Standard_True).IsValid(),"valid solid");
    TopTools_IndexedMapOfShape faces,edges,vertices,shells;
    TopExp::MapShapes(out.solid,TopAbs_FACE,faces);TopExp::MapShapes(out.solid,TopAbs_EDGE,edges);
    TopExp::MapShapes(out.solid,TopAbs_VERTEX,vertices);TopExp::MapShapes(out.solid,TopAbs_SHELL,shells);
    const int n=int(d.stations.size());
    require(shells.Extent()==1 && BRep_Tool::IsClosed(shells(1)),"one closed shell");
    require(faces.Extent()==4*(n-1)+2,"authored face count");
    require(edges.Extent()==8*n-4 && vertices.Extent()==4*n,"shared authored edges/vertices");
    require(std::abs(out.volume-inspection.expectedVolume)<=inspection.expectedVolume*1e-10,"exact volume");
    for(int i=0;i<6;++i)require(std::abs(out.bounds[i]-inspection.expectedBounds[i])<1e-7/inspection.millimetersPerUnit,"exact bounds");
    std::vector<int> forward(edges.Extent()),reverse(edges.Extent());
    reader::Inspection report;
    for(int i=1;i<=faces.Extent();++i) {
        auto face=TopoDS::Face(faces(i));TopLoc_Location location;
        require(!Handle(Geom_Plane)::DownCast(BRep_Tool::Surface(face,location)).IsNull(),"stored Geom_Plane");
        reader::detail::Surface surface;
        require(reader::detail::ReadSurface(face,inspection.millimetersPerUnit,report,surface),"retained-cut ReadSurface");
        require(!surface.cylinder,"reader plane");
        int uses=0;
        for(TopExp_Explorer it(face,TopAbs_EDGE);it.More();it.Next()) {
            const auto edge=TopoDS::Edge(it.Current());const int index=edges.FindIndex(edge)-1;
            require(index>=0,"edge identity");++uses;
            if(edge.Orientation()==TopAbs_FORWARD)++forward[index];
            else if(edge.Orientation()==TopAbs_REVERSED)++reverse[index];
            else require(false,"edge orientation");
            reader::detail::Curve curve;reader::detail::PCurve pc;
            require(reader::detail::ReadCurve(edge,inspection.millimetersPerUnit,report,curve),"retained-cut ReadCurve");
            require(reader::detail::ReadPCurve(edge,surface,curve,pc,report),"retained-cut ReadPCurve");
            require(reader::detail::PCurveIdentity(curve,pc,surface,inspection.millimetersPerUnit,1e-6),"pcurve line identity");
        }
        require(uses==4,"four-edge planar wire");
    }
    for(int i=1;i<=edges.Extent();++i) {
        const auto edge=TopoDS::Edge(edges(i));TopLoc_Location location;double a=0,b=0;
        require(!Handle(Geom_Line)::DownCast(BRep_Tool::Curve(edge,location,a,b)).IsNull(),"stored Geom_Line");
        require(forward[i-1]==1 && reverse[i-1]==1,"opposite shared edge uses");
    }
    // Match each authored corner and ring edge, including all interior seams.
    gp_Trsf placement;if(d.constructionFrame)require(d.constructionFrame->Transform(placement),"frame");
    for(const auto& s:d.stations) {
        auto points=loft::detail::Corners(s);std::array<TopoDS_Vertex,4> ring;
        for(int i=0;i<4;++i) {
            points[i].Transform(placement);int matches=0;
            for(int j=1;j<=vertices.Extent();++j) {
                auto v=TopoDS::Vertex(vertices(j));
                if(BRep_Tool::Pnt(v).Distance(points[i])<1e-7/inspection.millimetersPerUnit){ring[i]=v;++matches;}
            }
            require(matches==1,"unique authored corner");
        }
        for(int i=0;i<4;++i) {
            int matches=0;
            for(int j=1;j<=edges.Extent();++j) {
                TopoDS_Vertex a,b;TopExp::Vertices(TopoDS::Edge(edges(j)),a,b);
                if((a.IsSame(ring[i])&&b.IsSame(ring[(i+1)%4])) ||
                   (b.IsSame(ring[i])&&a.IsSame(ring[(i+1)%4])))++matches;
            }
            require(matches==1,"authored station seam retained");
        }
    }
    stop=true;require(loft::Build(prepared,stop,out)==loft::BuildStatus::Cancelled && out.solid.IsNull() && out.volume==0,"cancel clears prior result");
    stop=false;require(loft::Build({},stop,out)==loft::BuildStatus::InvalidDefinition && out.solid.IsNull(),"invalid clears output");
}
int main() {
    try {
        int cases=0;
        for(int count:{2,3,8})for(double unit:{0.001,1.0})for(int kind:{0,1,2})for(int frame:{0,1,2}) {
            check(fixture(count,unit,kind,frame));++cases;
        }
        for(int kind:{0,1,2,3})for(double unit:{0.001,1.0})for(int frame:{0,1,2,3}) {
            check(loft::probe::Fixture(kind,unit,frame));++cases;
        }
        for(const auto& rejection:loft::probe::Rejections()) {
            loft::Inspection inspection;require(loft::Inspect(rejection.definition,inspection)==rejection.expected,"existing admission rejection");
        }
        // Minimum physical size in mm builds. At the native 32*Confusion
        // threshold in metres, the prior builder also returned InvalidSolid:
        // preserve that verified kernel refusal and the empty-output contract.
        for(double unit:{0.001,1.0}) {
            auto d=fixture(2,unit,1,0);const double size=std::max(0.001/(unit*1000),32*Precision::Confusion());
            for(auto& s:d.stations){s.width=size;s.depth=size;}d.stations[1].z=size;
            if(unit==0.001){check(d);++cases;}
            else {
                loft::Admission admission;const auto prepared=loft::Prepare(d,admission);
                require(prepared!=nullptr,"tiny admitted recipe");
                std::atomic_bool stop{false};loft::SolidResult out;
                require(loft::Build(prepared,stop,out)==loft::BuildStatus::InvalidSolid && out.solid.IsNull() && out.volume==0,
                        "preexisting tiny metre kernel refusal publishes nothing");
            }
        }
        {
            auto d=fixture(3,0.001,2,0);loft::Admission admission;
            const auto prepared=loft::Prepare(d,admission);std::atomic_bool stop{false};loft::SolidResult good,out;
            require(loft::Build(prepared,stop,good)==loft::BuildStatus::Built,"verification fixture");
            auto wrong=prepared->inspection;wrong.expectedVolume*=2;out=good;
            require(loft::detail::Verify(good.solid,wrong,stop,out)==loft::BuildStatus::VerificationFailed && out.solid.IsNull(),"wrong volume clears output");
            wrong=prepared->inspection;wrong.expectedBounds[0]+=1;out=good;
            require(loft::detail::Verify(good.solid,wrong,stop,out)==loft::BuildStatus::VerificationFailed && out.solid.IsNull(),"wrong bounds clears output");
            stop=true;out=good;
            require(loft::detail::Verify(good.solid,prepared->inspection,stop,out)==loft::BuildStatus::Cancelled && out.solid.IsNull(),"late cancellation clears output");
        }
        std::cout<<"PASS "<<cases<<" loft cases; planes/lines, topology, exact metrics, retained-cut readers, cancellation; 26 admission rejections; prior tiny-metre InvalidSolid preserved\n";
    }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
