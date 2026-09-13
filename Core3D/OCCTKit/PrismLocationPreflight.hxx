#pragma once
// External proposed location admission. Raw topology is traversed without
// cumulative locations BEFORE any cumulative iterator or BRep_Tool lookup.
#include <TopoDS_Iterator.hxx>
#include <TopLoc_Datum3D.hxx>
#include <BRep_TFace.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_TVertex.hxx>
#include <BRep_CurveRepresentation.hxx>
#include <BRep_PointRepresentation.hxx>
#include <BRep_ListIteratorOfListOfCurveRepresentation.hxx>
#include <BRep_ListIteratorOfListOfPointRepresentation.hxx>
#include <gp_Trsf.hxx>
#include <array>
#include <vector>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <limits>

namespace core3d::saved_cut_prism_prototype::locations {
using Matrix=std::array<std::array<double,4>,4>;
inline Matrix Identity(){Matrix m{};for(unsigned i=0;i<4;++i)m[i][i]=1;return m;}
inline bool TransformValid(const gp_Trsf& t){
    if(!std::isfinite(t.ScaleFactor())||t.ScaleFactor()<=0||t.IsNegative())return false;
    for(unsigned r=1;r<=3;++r)for(unsigned c=1;c<=4;++c)if(!std::isfinite(t.Value(r,c)))return false;
    return true;
}
inline double Norm(const gp_Trsf& t){
    double result=1;
    for(unsigned r=1;r<=3;++r){double sum=0;for(unsigned c=1;c<=4;++c)sum+=std::abs(t.Value(r,c));result=std::max(result,sum);}
    return result;
}
inline bool RawLocation(const TopLoc_Location& original){
    auto location=original;unsigned count=0;
    while(!location.IsIdentity()){
        if(++count>8||location.FirstDatum().IsNull())return false;
        const int power=location.FirstPower();if(power!=1&&power!=-1)return false;
        const auto& t=location.FirstDatum()->Transformation();if(!TransformValid(t))return false;
        if(power<0&&!TransformValid(t.Inverted()))return false;
        location=location.NextLocation();
    }return true;
}
inline bool RawShape(const TopoDS_Shape& root,const std::atomic_bool& stop){
    struct Item{TopoDS_Shape shape;unsigned depth;};std::vector<Item> stack{{root,0}};unsigned visited=0;
    constexpr TopAbs_ShapeEnum types[]={TopAbs_SOLID,TopAbs_SHELL,TopAbs_FACE,TopAbs_WIRE,TopAbs_EDGE,TopAbs_VERTEX};
    while(!stack.empty()){
        if(stop.load()||++visited>1408)return false;const auto item=stack.back();stack.pop_back();
        if(item.shape.IsNull()||item.depth>5||item.shape.ShapeType()!=types[item.depth]
            ||!RawLocation(item.shape.Location()))return false;
        if(item.depth==2){
            const auto face=Handle(BRep_TFace)::DownCast(item.shape.TShape());
            if(face.IsNull()||!RawLocation(face->Location()))return false;
        }else if(item.depth==4){
            const auto edge=Handle(BRep_TEdge)::DownCast(item.shape.TShape());if(edge.IsNull())return false;
            unsigned count=0;for(BRep_ListIteratorOfListOfCurveRepresentation it(edge->Curves());it.More();it.Next())
                if(++count>16||it.Value().IsNull()||!RawLocation(it.Value()->Location()))return false;
        }else if(item.depth==5){
            const auto vertex=Handle(BRep_TVertex)::DownCast(item.shape.TShape());if(vertex.IsNull())return false;
            unsigned count=0;for(BRep_ListIteratorOfListOfPointRepresentation it(vertex->Points());it.More();it.Next())
                if(++count>16||it.Value().IsNull()||!RawLocation(it.Value()->Location()))return false;
        }
        const unsigned limits[]={1,66,1,64,2,0};unsigned count=0;
        for(TopoDS_Iterator child(item.shape,Standard_False,Standard_False);child.More();child.Next()){
            if(++count>limits[item.depth]||item.depth==5||stack.size()>=1408)return false;
            stack.push_back({child.Value(),item.depth+1});
        }
    }return true;
}
// Absolute product bounds are accumulated outward, not reconstructed from a
// possibly cancelling final matrix. Raw path6×8 bounds combined chains to48
// elementary powers, with a few identical adjacent datums merged by OCCT.
inline bool Composition(const TopLoc_Location& original,Matrix& output,double& errorUnits){
    std::vector<gp_Trsf> atoms;errorUnits=1;auto location=original;unsigned entries=0;
    while(!location.IsIdentity()){
        if(++entries>48||location.FirstDatum().IsNull())return false;
        const int power=location.FirstPower();if(power==0||power<-6||power>6)return false;
        gp_Trsf value=location.FirstDatum()->Transformation();if(!TransformValid(value))return false;
        double conditioning=1;
        if(power<0){const auto inverse=value.Inverted();if(!TransformValid(inverse))return false;
            conditioning=Norm(value)*Norm(inverse);if(!std::isfinite(conditioning))return false;value=inverse;}
        for(int i=0;i<std::abs(power);++i){if(atoms.size()>=48)return false;atoms.push_back(value);errorUnits+=conditioning;}
        location=location.NextLocation();
    }
    output=Identity();
    for(auto it=atoms.rbegin();it!=atoms.rend();++it){
        Matrix next=Identity();for(unsigned r=0;r<3;++r)for(unsigned c=0;c<4;++c)next[r][c]=std::abs(it->Value(r+1,c+1));
        Matrix product{};
        for(unsigned r=0;r<4;++r)for(unsigned c=0;c<4;++c)for(unsigned k=0;k<4;++k){
            if(output[r][k]==0||next[k][c]==0)continue;
            const double term=output[r][k]*next[k][c];if(!std::isfinite(term))return false;
            const double up=std::nextafter(term,std::numeric_limits<double>::infinity());
            const double sum=product[r][c]+up;if(!std::isfinite(sum))return false;
            product[r][c]=std::nextafter(sum,std::numeric_limits<double>::infinity());
        }
        output=product;
    }return std::isfinite(errorUnits);
}
} // namespace core3d::saved_cut_prism_prototype::locations
