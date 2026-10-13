#pragma once

// DEBUG qualification proof for two analytically stored rectangular prisms.
// Expected geometry is constructed only from the stored dimensions, placement
// and operation.  The candidate is observed afterwards; no Boolean result or
// candidate volume participates in constructing the expected boundary.
#include "PartBooleanCorrespondence.hxx"
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace core3d::part_boolean::analytic_placement_proof {

inline constexpr double NearTolerance = 1e-6;
inline constexpr std::size_t MaximumWork = 4096;
inline constexpr std::size_t MaximumCells = 84;
inline constexpr std::size_t MaximumVerticesPerCell = 16;

struct Vec final { double x = 0, y = 0, z = 0; };
inline Vec operator+(Vec a, Vec b) noexcept { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec operator-(Vec a, Vec b) noexcept { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Vec operator-(Vec a) noexcept { return {-a.x,-a.y,-a.z}; }
inline Vec operator*(Vec a, double s) noexcept { return {a.x*s,a.y*s,a.z*s}; }
inline Vec operator/(Vec a, double s) noexcept { return {a.x/s,a.y/s,a.z/s}; }
inline double Dot(Vec a, Vec b) noexcept { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec Cross(Vec a, Vec b) noexcept {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
inline double Length(Vec a) noexcept { return std::sqrt(Dot(a,a)); }
inline bool Finite(Vec a) noexcept {
    return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);
}
inline Vec Point(const gp_Pnt& p) noexcept { return {p.X(),p.Y(),p.Z()}; }

inline Vec Rotate(const std::array<double,4>& q, Vec p) noexcept {
    const double x=q[0],y=q[1],z=q[2],w=q[3];
    return {
        (1-2*(y*y+z*z))*p.x + 2*(x*y-z*w)*p.y + 2*(x*z+y*w)*p.z,
        2*(x*y+z*w)*p.x + (1-2*(x*x+z*z))*p.y + 2*(y*z-x*w)*p.z,
        2*(x*z-y*w)*p.x + 2*(y*z+x*w)*p.y + (1-2*(x*x+y*y))*p.z};
}

struct Plane final {
    Vec normal;
    double offset = 0;
    std::uint8_t input = 0;
    std::uint8_t axis = 0;
    bool maximum = false;
};
struct Prism final {
    Vec origin;
    std::array<Vec,3> axes{};
    std::array<double,3> lengths{};
    std::array<Vec,8> corners{};
    std::array<Plane,6> planes{};
};
enum class Role : std::uint8_t { Source = 1, Cut = 2 };
struct Cell final {
    Plane plane;
    Role role = Role::Source;
    std::vector<Vec> polygon;
    double area = 0;
    Vec centroid;
};

struct Proof final {
    bool sourceEquations = false;
    bool operationOracle = false;
    bool completePlanarBoundary = false;
    bool correspondenceComplete = false;
    bool materialsComplete = false;
    bool connectedPositiveSolid = false;
    bool completeBoundaryCoverage = false;
    bool outwardOrientation = false;
    bool uniqueSourceAndCutRoles = false;
    bool materialPolicyBound = false;
    bool boundedArrangement = false;
    bool storedQuaternionBitsBound = false;
    bool nonidentityRotation = false;
    double expectedVolume = 0;
    double observedVolume = 0;
    std::size_t work = 0;
    std::size_t expectedCells = 0;
    std::array<std::array<std::uint64_t,4>,2> quaternionBits{};
    bool proven() const noexcept {
        return sourceEquations && operationOracle && completePlanarBoundary
            && correspondenceComplete && materialsComplete && connectedPositiveSolid
            && completeBoundaryCoverage && outwardOrientation
            && uniqueSourceAndCutRoles && materialPolicyBound
            && boundedArrangement && storedQuaternionBitsBound
            && nonidentityRotation;
    }
};

struct Work final {
    std::size_t count = 0;
    bool refused = false;
    bool take(std::size_t amount = 1) noexcept {
        if (amount > MaximumWork-count) { refused=true; return false; }
        count += amount; return true;
    }
};

inline bool SamePoint(Vec a, Vec b) noexcept {
    return Length(a-b) <= NearTolerance;
}

inline bool BuildPrism(const AnalyticPrismInput& input,
                       double carrierMetersPerUnit, std::uint8_t index,
                       Prism& prism) noexcept {
    if (!Valid(input) || !std::isfinite(carrierMetersPerUnit)
        || carrierMetersPerUnit<=0) return false;
    const double scale=input.metersPerUnit/carrierMetersPerUnit;
    prism.origin={input.translation[0]*scale,input.translation[1]*scale,
                  input.translation[2]*scale};
    prism.axes={Rotate(input.rotationXYZW,{1,0,0}),
                Rotate(input.rotationXYZW,{0,1,0}),
                Rotate(input.rotationXYZW,{0,0,1})};
    for(std::size_t axis=0;axis<3;++axis){
        prism.lengths[axis]=input.dimensions[axis]*scale;
        if(!Finite(prism.axes[axis])||!std::isfinite(prism.lengths[axis])
            ||prism.lengths[axis]<=0
            ||std::abs(Length(prism.axes[axis])-1)>1e-12) return false;
    }
    for(std::size_t bits=0;bits<8;++bits){
        Vec p=prism.origin;
        for(std::size_t axis=0;axis<3;++axis)
            if(bits&(1U<<axis)) p=p+prism.axes[axis]*prism.lengths[axis];
        if(!Finite(p)) return false;
        prism.corners[bits]=p;
    }
    for(std::size_t axis=0;axis<3;++axis){
        const Vec low=-prism.axes[axis], high=prism.axes[axis];
        prism.planes[2*axis]={low,Dot(low,prism.origin),index,
                              std::uint8_t(axis),false};
        const Vec end=prism.origin+high*prism.lengths[axis];
        prism.planes[2*axis+1]={high,Dot(high,end),index,
                                std::uint8_t(axis),true};
    }
    return true;
}

inline std::vector<Vec> FacePolygon(const Prism& prism,
                                    const Plane& plane) {
    std::vector<Vec> result;
    for(const Vec corner:prism.corners)
        if(std::abs(Dot(plane.normal,corner)-plane.offset)<=NearTolerance)
            result.push_back(corner);
    if(result.size()!=4) return {};
    Vec centre{}; for(Vec p:result) centre=centre+p; centre=centre/4;
    Vec u=result.front()-centre; const double ul=Length(u);
    if(ul<=NearTolerance) return {}; u=u/ul;
    const Vec v=Cross(plane.normal,u);
    std::sort(result.begin(),result.end(),[&](Vec a,Vec b){
        const Vec da=a-centre,db=b-centre;
        return std::atan2(Dot(da,v),Dot(da,u))
             < std::atan2(Dot(db,v),Dot(db,u));
    });
    return result;
}

inline void Clean(std::vector<Vec>& polygon) {
    std::vector<Vec> clean;
    for(Vec p:polygon)
        if(clean.empty()||!SamePoint(clean.back(),p)) clean.push_back(p);
    if(clean.size()>1&&SamePoint(clean.front(),clean.back())) clean.pop_back();
    polygon=std::move(clean);
}

inline bool Clip(const std::vector<Vec>& source, const Plane& plane,
                 bool inside, Work& work, std::vector<Vec>& output) {
    output.clear();
    if(source.empty()) return true;
    if(source.size()>MaximumVerticesPerCell||!work.take(source.size())) return false;
    auto distance=[&](Vec p){
        const double d=Dot(plane.normal,p)-plane.offset;
        return inside?d:-d;
    };
    Vec a=source.back(); double da=distance(a); bool aIn=da<=NearTolerance;
    for(Vec b:source){
        const double db=distance(b); const bool bIn=db<=NearTolerance;
        if(aIn!=bIn){
            const double denominator=da-db;
            if(!std::isfinite(denominator)||std::abs(denominator)<=1e-15)
                return false;
            const double t=da/denominator;
            if(!std::isfinite(t)||t<-NearTolerance||t>1+NearTolerance) return false;
            output.push_back(a+(b-a)*t);
        }
        if(bIn) output.push_back(b);
        a=b; da=db; aIn=bIn;
    }
    Clean(output);
    if(output.size()>MaximumVerticesPerCell) return false;
    return true;
}

inline bool Measure(Cell& cell) noexcept {
    if(cell.polygon.size()<3) return false;
    const Vec base=cell.polygon.front();
    double area=0; Vec moment{};
    for(std::size_t i=1;i+1<cell.polygon.size();++i){
        const Vec cross=Cross(cell.polygon[i]-base,cell.polygon[i+1]-base);
        const double signedArea=Dot(cross,cell.plane.normal)/2;
        if(!std::isfinite(signedArea)||signedArea<=1e-12) return false;
        const Vec centre=(base+cell.polygon[i]+cell.polygon[i+1])/3;
        area+=signedArea; moment=moment+centre*signedArea;
    }
    if(!std::isfinite(area)||area<=1e-10) return false;
    cell.area=area; cell.centroid=moment/area;
    return Finite(cell.centroid);
}

inline bool AppendCell(std::vector<Cell>& cells, Plane boundary, Role role,
                       std::vector<Vec> polygon, Work& work) {
    if(polygon.size()<3) return true;
    if(cells.size()>=MaximumCells||!work.take()) return false;
    Cell cell{boundary,role,std::move(polygon)};
    if(!Measure(cell)) return false;
    cells.push_back(std::move(cell)); return true;
}

inline bool InsideCell(Vec point,const Cell& cell) noexcept {
    for(std::size_t i=0;i<cell.polygon.size();++i){
        const Vec a=cell.polygon[i],b=cell.polygon[(i+1)%cell.polygon.size()];
        if(Dot(Cross(b-a,point-a),cell.plane.normal)<-NearTolerance) return false;
    }
    return true;
}

inline bool SegmentCovered(Vec a,Vec b,const std::vector<const Cell*>& cells,
                           Work& work) {
    std::vector<std::array<double,2>> intervals;
    for(const Cell* cell:cells){
        if(!work.take(cell->polygon.size())) return false;
        double low=0,high=1; const Vec delta=b-a;
        for(std::size_t i=0;i<cell->polygon.size()&&low<=high;++i){
            const Vec p=cell->polygon[i];
            const Vec edge=cell->polygon[(i+1)%cell->polygon.size()]-p;
            const Vec inward=Cross(cell->plane.normal,edge);
            const double start=Dot(inward,a-p)+NearTolerance;
            const double rate=Dot(inward,delta);
            if(std::abs(rate)<=1e-15){ if(start<0){low=1;high=0;} }
            else if(rate>0) low=std::max(low,-start/rate);
            else high=std::min(high,-start/rate);
        }
        if(low<=high+1e-12) intervals.push_back({std::max(0.0,low),std::min(1.0,high)});
    }
    std::sort(intervals.begin(),intervals.end(),[](auto x,auto y){return x[0]<y[0];});
    double covered=0;
    for(const auto interval:intervals){
        if(interval[0]>covered+1e-9) return false;
        covered=std::max(covered,interval[1]);
    }
    return covered>=1-1e-9;
}

inline bool AppendInside(const std::vector<Vec>& face,
                         const std::array<Plane,6>& other, Plane boundary,
                         Role role, Work& work,std::vector<Cell>& cells) {
    std::vector<Vec> current=face,next;
    for(const Plane& plane:other){
        if(!Clip(current,plane,true,work,next)) return false;
        current=std::move(next);
        if(current.empty()) return true;
    }
    return AppendCell(cells,boundary,role,std::move(current),work);
}

inline bool AppendOutside(const std::vector<Vec>& face,
                          const std::array<Plane,6>& other, Plane boundary,
                          Role role, Work& work,std::vector<Cell>& cells) {
    std::vector<Vec> remaining=face,inside,outside;
    for(const Plane& plane:other){
        if(!Clip(remaining,plane,false,work,outside)
            ||!AppendCell(cells,boundary,role,std::move(outside),work)
            ||!Clip(remaining,plane,true,work,inside)) return false;
        remaining=std::move(inside);
        if(remaining.empty()) break;
    }
    return true;
}

inline bool Arrangement(const AnalyticDefinition& definition,
                        const std::array<Prism,2>& prisms,Work& work,
                        std::vector<Cell>& cells) {
    for(std::size_t input=0;input<2;++input){
        for(const Plane& sourcePlane:prisms[input].planes){
            std::vector<Vec> face=FacePolygon(prisms[input],sourcePlane);
            if(face.size()!=4) return false;
            Plane boundary=sourcePlane; Role role=Role::Source;
            if(definition.operation==Operation::Intersect){
                if(!AppendInside(face,prisms[1-input].planes,boundary,role,work,cells)) return false;
            }else if(definition.operation==Operation::Union){
                if(!AppendOutside(face,prisms[1-input].planes,boundary,role,work,cells)) return false;
            }else if(input==0){
                if(!AppendOutside(face,prisms[1].planes,boundary,role,work,cells)) return false;
            }else{
                std::reverse(face.begin(),face.end());
                boundary.normal=-boundary.normal; boundary.offset=-boundary.offset;
                role=Role::Cut;
                if(!AppendInside(face,prisms[0].planes,boundary,role,work,cells)) return false;
            }
        }
    }
    return !cells.empty()&&!work.refused;
}

inline bool SamePlane(const Plane& expected,Vec normal,double offset) noexcept {
    return Length(expected.normal-normal)<=NearTolerance
        && correspondence::Near(expected.offset,offset,NearTolerance);
}

inline bool VerifyBoundary(const TopoDS_Shape& shape,
                           const std::vector<Cell>& cells,Work& work,
                           bool& planar,bool& coverage,bool& orientation,
                           bool& uniqueRoles) {
    planar=true;coverage=true;orientation=true;uniqueRoles=true;
    struct Bucket { Plane plane; Role role; double expected=0,observed=0;
                    Vec expectedMoment{},observedMoment{}; std::vector<const Cell*> cells; };
    std::vector<Bucket> buckets;
    for(const Cell& cell:cells){
        auto found=std::find_if(buckets.begin(),buckets.end(),[&](const Bucket& b){
            return SamePlane(b.plane,cell.plane.normal,cell.plane.offset);});
        if(found==buckets.end()){
            buckets.push_back({cell.plane,cell.role,cell.area,0,
                               cell.centroid*cell.area,{}, {&cell}});
        }else{
            if(found->plane.input!=cell.plane.input||found->role!=cell.role)
                uniqueRoles=false;
            found->expected+=cell.area;
            found->expectedMoment=found->expectedMoment+cell.centroid*cell.area;
            found->cells.push_back(&cell);
        }
    }
    std::size_t faceCount=0;
    for(TopExp_Explorer it(shape,TopAbs_FACE);it.More();it.Next()){
        ++faceCount;if(!work.take()) return false;
        const TopoDS_Face face=TopoDS::Face(it.Current());
        BRepAdaptor_Surface surface(face,Standard_True);
        if(surface.GetType()!=GeomAbs_Plane){planar=false;return true;}
        const gp_Dir direction=surface.Plane().Axis().Direction();
        Vec normal={direction.X(),direction.Y(),direction.Z()};
        if(face.Orientation()==TopAbs_REVERSED) normal=-normal;
        const double offset=Dot(normal,Point(surface.Plane().Location()));
        std::vector<Bucket*> matches;
        for(Bucket& bucket:buckets)
            if(SamePlane(bucket.plane,normal,offset)) matches.push_back(&bucket);
        if(matches.size()!=1){orientation=false;coverage=false;continue;}
        Bucket& bucket=*matches.front();
        GProp_GProps properties;BRepGProp::SurfaceProperties(face,properties);
        const double area=properties.Mass();const Vec centre=Point(properties.CentreOfMass());
        if(!std::isfinite(area)||area<=Precision::Confusion()||!Finite(centre)) return false;
        bucket.observed+=area;bucket.observedMoment=bucket.observedMoment+centre*area;
        for(TopExp_Explorer edgeIt(face,TopAbs_EDGE);edgeIt.More();edgeIt.Next()){
            if(!work.take()) return false;
            const TopoDS_Edge edge=TopoDS::Edge(edgeIt.Current());
            BRepAdaptor_Curve curve(edge);
            if(curve.GetType()!=GeomAbs_Line){coverage=false;continue;}
            TopoDS_Vertex first,last;TopExp::Vertices(edge,first,last,Standard_True);
            if(first.IsNull()||last.IsNull()
                ||!SegmentCovered(Point(BRep_Tool::Pnt(first)),Point(BRep_Tool::Pnt(last)),
                                   bucket.cells,work)) coverage=false;
        }
    }
    planar=planar&&faceCount>=6;
    for(const Bucket& bucket:buckets){
        if(!correspondence::Near(bucket.expected,bucket.observed,NearTolerance)
            ||!correspondence::Near(bucket.expectedMoment.x,bucket.observedMoment.x,NearTolerance)
            ||!correspondence::Near(bucket.expectedMoment.y,bucket.observedMoment.y,NearTolerance)
            ||!correspondence::Near(bucket.expectedMoment.z,bucket.observedMoment.z,NearTolerance))
            coverage=false;
    }
    return true;
}

inline Proof Prove(const AnalyticDefinition& definition,
                   const build::AnalyticBuild& built,
                   double carrierMetersPerUnit) noexcept {
    Proof proof; Work work;
    try{
        if(!Valid(definition)||!built.complete||!std::isfinite(carrierMetersPerUnit)
            ||carrierMetersPerUnit<=0) return proof;
        std::array<Prism,2> prisms;
        if(!BuildPrism(definition.inputs[0],carrierMetersPerUnit,0,prisms[0])
            ||!BuildPrism(definition.inputs[1],carrierMetersPerUnit,1,prisms[1])) return proof;
        for(std::size_t input=0;input<2;++input){
            for(std::size_t i=0;i<4;++i)
                std::memcpy(&proof.quaternionBits[input][i],
                            &definition.inputs[input].rotationXYZW[i],sizeof(double));
            const auto& q=definition.inputs[input].rotationXYZW;
            const bool identityMatrix=std::abs(Rotate(q,{1,0,0}).x-1)<=1e-12
                &&std::abs(Rotate(q,{1,0,0}).y)<=1e-12
                &&std::abs(Rotate(q,{1,0,0}).z)<=1e-12
                &&std::abs(Rotate(q,{0,1,0}).x)<=1e-12
                &&std::abs(Rotate(q,{0,1,0}).y-1)<=1e-12
                &&std::abs(Rotate(q,{0,1,0}).z)<=1e-12
                &&std::abs(Rotate(q,{0,0,1}).x)<=1e-12
                &&std::abs(Rotate(q,{0,0,1}).y)<=1e-12
                &&std::abs(Rotate(q,{0,0,1}).z-1)<=1e-12;
            proof.nonidentityRotation=proof.nonidentityRotation||!identityMatrix;
        }
        proof.storedQuaternionBitsBound=true;
        std::vector<Cell> cells;
        if(!Arrangement(definition,prisms,work,cells)) return proof;
        proof.expectedCells=cells.size();
        double orientedVolume=0;
        for(const Cell& cell:cells)
            orientedVolume+=cell.area*Dot(cell.centroid,cell.plane.normal)/3;
        proof.expectedVolume=orientedVolume;
        if(!std::isfinite(proof.expectedVolume)||proof.expectedVolume<=NearTolerance)
            return proof;
        proof.observedVolume=retained_part_boolean::Volume(built.candidate.solid);
        proof.operationOracle=correspondence::Near(proof.observedVolume,proof.expectedVolume);
        proof.sourceEquations=true;
        for(std::size_t input=0;input<2;++input){
            const double expected=prisms[input].lengths[0]*prisms[input].lengths[1]
                *prisms[input].lengths[2];
            proof.sourceEquations=proof.sourceEquations
                &&correspondence::Near(retained_part_boolean::Volume(built.sources[input]),expected);
        }
        bool planar=false,coverage=false,orientation=false,roles=false;
        if(!VerifyBoundary(built.candidate.solid,cells,work,planar,coverage,orientation,roles))
            return proof;
        proof.completePlanarBoundary=planar;
        proof.completeBoundaryCoverage=coverage;
        proof.outwardOrientation=orientation;
        proof.uniqueSourceAndCutRoles=roles;
        proof.correspondenceComplete=planar&&coverage&&orientation&&roles
            &&built.evidence.historyObserved&&built.evidence.inputBytesUnchanged;
        proof.materialPolicyBound=definition.materialPolicy
            ==MaterialPolicy::RejectConflictingMerges&&roles;
        proof.materialsComplete=proof.materialPolicyBound;
        for(const auto& input:definition.inputs)
            proof.materialsComplete=proof.materialsComplete
                &&input.originalMaterial.kind==MaterialKind::ResolvedScalars
                &&Valid(input.originalMaterial);
        proof.connectedPositiveSolid=retained_part_boolean::IsOneValidForwardSolid(
            built.candidate.solid)&&proof.observedVolume>Precision::Confusion();
        proof.boundedArrangement=!work.refused&&work.count<=MaximumWork
            &&cells.size()<=MaximumCells;
        proof.work=work.count;
        return proof;
    }catch(...){return {};}
}

// N2 authored-owner facade.  The independently authored oracle above remains
// usable by the detached DEBUG qualification probe, while this entry reserves
// every one of its bounded value-derived arrangement operations and each native
// source/result observation on the caller's existing operation continuation.
// The reservation is deliberately conservative (MaximumWork) and happens
// before the oracle runs: denial is sticky and no uncounted fallback executes.
namespace tb = core3d::retained_topology_budget;

inline tb::WalkStatus ProveCounted(
    const AnalyticDefinition& definition,
    const build::AnalyticBuild& built,
    double carrierMetersPerUnit,
    tb::Counter& budget, const std::atomic_bool& cancelled,
    Proof& proof) noexcept {
    proof = {};
    if (cancelled.load()) return tb::WalkStatus::Cancelled;
    try {
        if (!Valid(definition) || !built.complete
            || !std::isfinite(carrierMetersPerUnit)
            || carrierMetersPerUnit <= 0) return tb::WalkStatus::Completed;

        // The expected prism/arrangement pass is bounded independently of the
        // candidate.  Reserve its full finite envelope before constructing any
        // expected cells, then reserve the candidate-bound verification stage.
        if (!budget.beginStage(tb::Site::C25AnalyticInput)
            || !budget.visit(MaximumWork, tb::Site::C25AnalyticInput))
            return tb::WalkStatus::BudgetDenied;
        if (cancelled.load()) return tb::WalkStatus::Cancelled;
        if (!budget.beginStage(tb::Site::C26AnalyticBoolean))
            return tb::WalkStatus::BudgetDenied;

        // Prove observes two source volumes, the result volume, the complete
        // planar boundary and final solid validity.  Charge one measured native
        // traversal for each pass before any of those observations can run.
        tb::WalkStatus walk = retained_part_boolean::ChargeAnalyticPass(
            built.sources[0], budget, cancelled, tb::Site::C25AnalyticInput);
        if (walk != tb::WalkStatus::Completed) return walk;
        walk = retained_part_boolean::ChargeAnalyticPass(
            built.sources[1], budget, cancelled, tb::Site::C25AnalyticInput);
        if (walk != tb::WalkStatus::Completed) return walk;
        for (int pass = 0; pass < 3; ++pass) {
            walk = retained_part_boolean::ChargeAnalyticPass(
                built.candidate.solid, budget, cancelled,
                tb::Site::C26AnalyticBoolean);
            if (walk != tb::WalkStatus::Completed) return walk;
        }
        if (cancelled.load()) return tb::WalkStatus::Cancelled;
        proof = Prove(definition, built, carrierMetersPerUnit);
        return tb::WalkStatus::Completed;
    } catch (...) {
        proof = {};
        return tb::WalkStatus::Failed;
    }
}

} // namespace core3d::part_boolean::analytic_placement_proof
